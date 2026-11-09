//! Parity tests for `kinematics::deltesian` (port of `klippy/kinematics/deltesian.py`).
//!
//! Every expected value comes from `pyoracle`: the frozen `klippy/kinematics/deltesian.py` runs
//! on top of the real `configfile.ConfigWrapper`, the real `gcode.Coord` and Python fakes for the
//! rails, steppers, toolhead, moves and homing state. The port runs against Rust fakes that are
//! fed the same inputs. Both sides are compared type-strictly (`recording::Value` keeps int and
//! float apart; floats compare by their hex bits): log lines, config options read (with their
//! defaults and bounds), rail lookups, `setup_itersolve` calls, `set_trapq` calls, `set_position`
//! calls, `get_status`, `calc_position`, `home` (axes and `home_rails` arguments),
//! `check_move` (errors and `limit_speed` calls) and every config error path.
//! The tests skip when `KLIPPY_PYTHON` is unset (outside the oracle image).

use indexmap::IndexMap;
use kinematics::deltesian;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, IntArgs,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Endstop, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper,
    StepperFactory, StepperFactoryApi, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, HomingStateApi, HomingStateHandle, HomingTargetApi, Kinematics, MoveApi, ToolHead,
    ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use pyoracle::{Oracle, OracleResult};
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::any::Any;
use std::cell::RefCell;
use kos::sync::Mutex;
use std::sync::{Arc, Once};

// ---------------------------------------------------------------------------
// Python side: the reference module with fakes around it.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import collections, configparser, logging, math, sys, types
import configfile, gcode

LOG = []
class _Handler(logging.Handler):
    def emit(self, record):
        LOG.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_Handler())
logging.getLogger().setLevel(logging.DEBUG)

CONFIG_CALLS = []
RAIL_CALLS = []
ALL_RAILS = []
SECTIONS = {}
FILECONFIG = None
MAX_V = 0.
MAX_A = 0.

def _bound(v):
    return 'None' if v is None else repr(v)
def _default(v):
    return 'required' if v is configfile.sentinel else repr(v)

# The real ConfigWrapper, recording every getfloat call.
class RecConfig(configfile.ConfigWrapper):
    def getsection(self, section):
        return RecConfig(self.printer, self.fileconfig, self.access_tracking, section)
    def getfloat(self, option, default=configfile.sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        CONFIG_CALLS.append((self.section, option, _default(default), _bound(minval),
                             _bound(maxval), _bound(above), _bound(below)))
        return super().getfloat(option, default, minval, maxval, above, below, note_valid)

def load_scenario(sections, max_velocity, max_accel):
    global SECTIONS, FILECONFIG, MAX_V, MAX_A
    SECTIONS = sections
    FILECONFIG = configparser.RawConfigParser()
    for name, opts in sections.items():
        FILECONFIG.add_section(name)
        for k, v in opts.items():
            FILECONFIG.set(name, k, repr(v))
    MAX_V, MAX_A = max_velocity, max_accel
    del CONFIG_CALLS[:]
    del RAIL_CALLS[:]
    del ALL_RAILS[:]
    del LOG[:]

class FakeStepper:
    def __init__(self, name):
        self.name = name
        self.trapq_calls = []
    def set_trapq(self, tq):
        self.trapq_calls.append(tq)
    def get_name(self, short=False):
        return self.name

HomingInfo = collections.namedtuple('homing_info',
    ['speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir',
     'second_homing_speed'])

class FakeRail:
    def __init__(self, kind, config, need_position_minmax, default_position_endstop):
        name = config.get_name()
        RAIL_CALLS.append((kind, name, need_position_minmax, repr(default_position_endstop)))
        vals = SECTIONS[name]
        self.name = name
        self.steppers = [FakeStepper(name)]
        self.iterations = []
        self.set_positions = []
        if 'position_endstop' in vals:
            self.endstop = vals['position_endstop']
        elif default_position_endstop is not None:
            self.endstop = default_position_endstop
        else:
            self.endstop = 0.
        self.positive_dir = vals.get('positive_dir', 0.) != 0.
        self.range_val = (vals.get('position_min', 0.), vals.get('position_max', 100.))
        ALL_RAILS.append(self)
    def get_name(self, short=False):
        return self.name
    def get_range(self):
        return self.range_val
    def get_homing_info(self):
        return HomingInfo(5., self.endstop, 5., 5., self.positive_dir, 2.5)
    def get_steppers(self):
        return list(self.steppers)
    def setup_itersolve(self, alloc, *params):
        self.iterations.append((alloc, list(params)))
    def set_position(self, coord):
        self.set_positions.append([coord[0], coord[1], coord[2]])

def LookupRail(config, need_position_minmax=True, default_position_endstop=None,
               units_in_radians=False):
    return FakeRail('LookupRail', config, need_position_minmax, default_position_endstop)
def LookupMultiRail(config, need_position_minmax=True, default_position_endstop=None,
                    units_in_radians=False):
    return FakeRail('LookupMultiRail', config, need_position_minmax, default_position_endstop)

stepper_mod = types.ModuleType('stepper')
stepper_mod.LookupRail = LookupRail
stepper_mod.LookupMultiRail = LookupMultiRail
sys.modules['stepper'] = stepper_mod
import kinematics.deltesian as deltesian

class FakeToolhead:
    Coord = gcode.Coord
    def get_trapq(self):
        return 'trapq'
    def get_max_velocity(self):
        return MAX_V, MAX_A

def make_kin():
    return deltesian.load_kinematics(FakeToolhead(), RecConfig(None, FILECONFIG, {}, 'printer'))

# Mirrors toolhead.Move.__init__ (axes_d, move_d, extruder-only branch) and move_error.
class FakeMove:
    def __init__(self, start_pos, end_pos):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.axes_d = axes_d = [ep - sp for sp, ep in zip(start_pos, end_pos)]
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        if move_d < .000000001:
            self.end_pos = ((start_pos[0], start_pos[1], start_pos[2]) + self.end_pos[3:])
            axes_d[0] = axes_d[1] = axes_d[2] = 0.
            self.move_d = move_d = max([abs(ad) for ad in axes_d[3:]])
        self.limited = []
    def limit_speed(self, speed, accel):
        self.limited.append((speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return gcode.CommandError(m)

class FakeHomingState:
    def __init__(self, axes, fail_at=None):
        self.axes = list(axes)
        self.fail_at = fail_at
        self.set_axes_calls = []
        self.home_rails_calls = []
    def get_axes(self):
        return list(self.axes)
    def set_axes(self, axes):
        self.axes = list(axes)
        self.set_axes_calls.append(list(axes))
    def home_rails(self, rails, forcepos, movepos):
        self.home_rails_calls.append(([r.get_name() for r in rails], list(forcepos), list(movepos)))
        if self.fail_at == len(self.home_rails_calls) - 1:
            raise gcode.CommandError('homing failed')

def snapshot():
    return [(r.name, r.iterations, r.set_positions,
             [(s.name, s.trapq_calls) for s in r.steppers]) for r in ALL_RAILS]
"#;

// ---------------------------------------------------------------------------
// Scenarios: one description feeds both sides.
// ---------------------------------------------------------------------------

type Section = (&'static str, Vec<(&'static str, f64)>);

#[derive(Clone)]
struct Scn {
    sections: Vec<Section>,
    max_v: f64,
    max_a: f64,
}

fn pyf(x: f64) -> String {
    assert!(x.is_finite(), "non-finite float in a Python literal");
    format!("{x:?}")
}

fn pyl(xs: &[f64]) -> String {
    let parts: Vec<String> = xs.iter().map(|x| pyf(*x)).collect();
    format!("[{}]", parts.join(", "))
}

impl Scn {
    fn base() -> Scn {
        Scn {
            sections: vec![
                ("printer", vec![]),
                (
                    "stepper_left",
                    vec![
                        ("arm_x_length", 20.0),
                        ("arm_length", 100.0),
                        ("position_endstop", 30.0),
                    ],
                ),
                (
                    "stepper_right",
                    vec![("arm_x_length", 22.0), ("arm_length", 105.0)],
                ),
                (
                    "stepper_y",
                    vec![
                        ("position_endstop", 0.0),
                        ("position_min", 0.0),
                        ("position_max", 250.0),
                        ("positive_dir", 0.0),
                    ],
                ),
            ],
            max_v: 200.0,
            max_a: 2000.0,
        }
    }

    fn set(mut self, section: &str, key: &'static str, value: f64) -> Scn {
        let entry = self
            .sections
            .iter_mut()
            .find(|(name, _)| *name == section)
            .expect("known section");
        match entry.1.iter_mut().find(|(k, _)| *k == key) {
            Some(slot) => slot.1 = value,
            None => entry.1.push((key, value)),
        }
        self
    }

    fn without(mut self, section: &str, key: &str) -> Scn {
        let entry = self
            .sections
            .iter_mut()
            .find(|(name, _)| *name == section)
            .expect("known section");
        entry.1.retain(|(k, _)| *k != key);
        self
    }

    fn limits(mut self, max_v: f64, max_a: f64) -> Scn {
        self.max_v = max_v;
        self.max_a = max_a;
        self
    }

    fn py_load(&self) -> String {
        let sections: Vec<String> = self
            .sections
            .iter()
            .map(|(name, opts)| {
                let opts: Vec<String> = opts
                    .iter()
                    .map(|(k, v)| format!("'{k}': {}", pyf(*v)))
                    .collect();
                format!("'{name}': {{{}}}", opts.join(", "))
            })
            .collect();
        format!(
            "load_scenario({{{}}}, {}, {})",
            sections.join(", "),
            pyf(self.max_v),
            pyf(self.max_a)
        )
    }
}

/// Slow thresholds (`slow_ratio` 0.3) inside the reachable X range.
fn slow_scn() -> Scn {
    Scn::base().set("printer", "slow_ratio", 0.3)
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn float_repr(v: f64) -> String {
    pycompat::text::float_repr(v)
}

struct Shared {
    sections: Vec<(String, Vec<(String, f64)>)>,
    config_calls: Mutex<Vec<Value>>,
    rail_calls: Mutex<Vec<Value>>,
    rails: Mutex<Vec<Arc<FakeRail>>>,
}

impl Shared {
    fn new(scn: &Scn) -> Arc<Shared> {
        Arc::new(Shared {
            sections: scn
                .sections
                .iter()
                .map(|(name, opts)| {
                    (
                        name.to_string(),
                        opts.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
                    )
                })
                .collect(),
            config_calls: Mutex::new(Vec::new()),
            rail_calls: Mutex::new(Vec::new()),
            rails: Mutex::new(Vec::new()),
        })
    }

    fn values(&self, section: &str) -> Vec<(String, f64)> {
        self.sections
            .iter()
            .find(|(name, _)| name == section)
            .map(|(_, v)| v.clone())
            .unwrap_or_default()
    }
}

#[derive(Debug)]
struct FakePrinter;

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, _n: &str, _o: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, _n: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _c: &Config,
        _s: &str,
        _d: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _c: EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _r: &str) {}
}

/// `configfile.ConfigWrapper.getfloat` (`klippy/configfile.py:29`) over the scenario values.
struct FakeConfig {
    section: String,
    shared: Arc<Shared>,
}

fn bound_text(opts: &GetFloatOpts, arg: IntArgs, v: Option<f64>) -> String {
    v.map_or_else(|| "None".to_string(), |v| opts.ints.value(arg, v).repr())
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::new(FakePrinter)
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    /// The port's untracked presence check for `minimum_z_position`; Python
    /// makes no such call, so it is not recorded.
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        assert!(!opts.note_valid && opts.default == Some(None), "deltesian reads no string option");
        let found = self.shared.values(&self.section).into_iter().find(|(k, _)| k == option);
        Ok(found.map(|(_, v)| float_repr(v)))
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("deltesian reads no int option")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default = match &opts.default {
            None => "required".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => opts.ints.value(IntArgs::DEFAULT, *v).repr(),
        };
        self.shared.config_calls.lock().push(Value::Tuple(vec![
            Value::str(&self.section),
            Value::str(option),
            Value::str(&default),
            Value::str(&bound_text(&opts, IntArgs::MINVAL, opts.minval)),
            Value::str(&bound_text(&opts, IntArgs::MAXVAL, opts.maxval)),
            Value::str(&bound_text(&opts, IntArgs::ABOVE, opts.above)),
            Value::str(&bound_text(&opts, IntArgs::BELOW, opts.below)),
        ]));
        let found = self
            .shared
            .values(&self.section)
            .into_iter()
            .find(|(k, _)| k == option)
            .map(|(_, v)| v);
        let Some(v) = found else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
            };
        };
        let section = &self.section;
        if let Some(b) = opts.minval {
            if v < b {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have minimum of {}",
                    opts.ints.value(IntArgs::MINVAL, b).str()
                )));
            }
        }
        if let Some(b) = opts.maxval {
            if v > b {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have maximum of {}",
                    opts.ints.value(IntArgs::MAXVAL, b).str()
                )));
            }
        }
        if let Some(b) = opts.above {
            if v <= b {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be above {}",
                    opts.ints.value(IntArgs::ABOVE, b).str()
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be below {}",
                    opts.ints.value(IntArgs::BELOW, b).str()
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("deltesian reads no boolean option")
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("deltesian reads no choice option")
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("deltesian reads no lists option")
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("deltesian reads no list option")
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("deltesian reads no int list option")
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("deltesian reads no float list option")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            section: section.to_string(),
            shared: self.shared.clone(),
        })
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("deltesian never calls has_section")
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        panic!("deltesian never calls get_prefix_sections")
    }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> {
        panic!("deltesian never calls get_prefix_options")
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeStepper {
    name: String,
    trapq_calls: Mutex<Vec<Option<Trapq>>>,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("unused")
    }
    fn get_name(&self, _s: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        1
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, _alloc: &str, _params: &[PyValue]) {
        panic!("deltesian sets up itersolve on the rails")
    }
    fn get_step_dist(&self) -> f64 {
        1.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (1.0, 200.0)
    }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("deltesian sets positions on the rails")
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _t: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _p: i64) -> f64 {
        0.0
    }
    fn dump_steps(&self, _c: i64, _s: i64, _e: i64) -> (Vec<PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<klippy_api::stepper::StepperKinematics>,
    ) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        self.trapq_calls.lock().last().cloned().unwrap_or(None)
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        self.trapq_calls.lock().push(tq);
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _a: char) -> bool {
        false
    }
}

struct FakeRail {
    name: String,
    stepper: Arc<FakeStepper>,
    range_val: (f64, f64),
    endstop: f64,
    positive_dir: bool,
    iterations: Mutex<Vec<(String, Vec<PyValue>)>>,
    set_positions: Mutex<Vec<[f64; 3]>>,
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        self.range_val
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop: self.endstop,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir: self.positive_dir,
            second_homing_speed: 2.5,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        let s: Stepper = self.stepper.clone();
        vec![s]
    }
    fn get_endstops(&self) -> Vec<(Endstop, String)> {
        Vec::new()
    }
    fn lookup_endstop(&self, _p: Option<&str>, _n: &str) -> KResult<Endstop> {
        panic!("unused")
    }
    fn add_stepper(&self, _s: Stepper, _p: Option<&str>, _n: Option<&str>) -> KResult<()> {
        Ok(())
    }
    fn add_stepper_from_config(&self, _c: &Config) -> KResult<()> {
        Ok(())
    }
    fn setup_itersolve(&self, alloc: &str, params: &[PyValue]) {
        self.iterations
            .lock()
            .push((alloc.to_string(), params.to_vec()));
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {
        panic!("deltesian sets the trapq on the steppers")
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.set_positions.lock().push(coord);
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeStepperFactory {
    shared: Arc<Shared>,
}

impl FakeStepperFactory {
    /// `stepper.LookupRail` / `LookupMultiRail` of the Python fake.
    fn make(&self, kind: &str, config: &Config, opts: RailOpts) -> KResult<Rail> {
        let name = config.get_name();
        let default_repr = match opts.default_position_endstop {
            Some(v) => float_repr(v),
            None => "None".to_string(),
        };
        self.shared.rail_calls.lock().push(Value::Tuple(vec![
            Value::str(kind),
            Value::str(&name),
            Value::Bool(opts.need_position_minmax),
            Value::str(&default_repr),
        ]));
        let vals = self.shared.values(&name);
        let get = |key: &str| vals.iter().find(|(k, _)| k == key).map(|(_, v)| *v);
        let endstop = get("position_endstop")
            .or(opts.default_position_endstop)
            .unwrap_or(0.0);
        let rail = Arc::new(FakeRail {
            name: name.clone(),
            stepper: Arc::new(FakeStepper {
                name: name.clone(),
                trapq_calls: Mutex::new(Vec::new()),
            }),
            range_val: (
                get("position_min").unwrap_or(0.0),
                get("position_max").unwrap_or(100.0),
            ),
            endstop,
            positive_dir: get("positive_dir").unwrap_or(0.0) != 0.0,
            iterations: Mutex::new(Vec::new()),
            set_positions: Mutex::new(Vec::new()),
        });
        self.shared.rails.lock().push(rail.clone());
        Ok(rail)
    }
}

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _c: &Config, _u: bool) -> KResult<Stepper> {
        panic!("unused")
    }
    fn lookup_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail> {
        self.make("LookupRail", config, opts)
    }
    fn lookup_multi_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail> {
        self.make("LookupMultiRail", config, opts)
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("unused")
    }
    fn parse_step_distance(&self, _c: &Config, _u: Option<bool>, _n: bool) -> KResult<(f64, f64)> {
        panic!("unused")
    }
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeToolhead {
    factory: StepperFactory,
    max_v: f64,
    max_a: f64,
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _t: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (self.max_v, self.max_a)
    }
    fn set_max_velocities(
        &self,
        _v: Option<f64>,
        _a: Option<f64>,
        _s: Option<f64>,
        _m: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (self.max_v, self.max_a, 0.0, 0.0)
    }
    fn register_lookahead_callback(
        &self,
        _c: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _t: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: klippy_api::toolhead::PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("unused")
    }
    fn add_extra_axis(&self, _e: klippy_api::toolhead::ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _t: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        Vec::new()
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

/// A `toolhead.Move` built from the values the Python `FakeMove` computed.
struct FakeMove {
    start: Vec<f64>,
    end: Vec<f64>,
    axes_d: Vec<f64>,
    move_d: f64,
    limited: Mutex<Vec<(f64, f64)>>,
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        Coord(self.start.clone())
    }
    fn end_pos(&self) -> Coord {
        Coord(self.end.clone())
    }
    fn axes_d(&self) -> Coord {
        Coord(self.axes_d.clone())
    }
    fn axes_r(&self) -> Coord {
        panic!("deltesian never reads axes_r")
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn is_kinematic_move(&self) -> bool {
        true
    }
    fn accel(&self) -> f64 {
        panic!("deltesian never reads accel")
    }
    fn max_cruise_v2(&self) -> f64 {
        panic!("deltesian never reads max_cruise_v2")
    }
    fn start_v(&self) -> f64 {
        0.0
    }
    fn cruise_v(&self) -> f64 {
        0.0
    }
    fn accel_t(&self) -> f64 {
        0.0
    }
    fn cruise_t(&self) -> f64 {
        0.0
    }
    fn decel_t(&self) -> f64 {
        0.0
    }
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> {
        self.limited.lock().push((speed, accel));
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let e = &self.end;
        let text = pycompat::text::percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                pycompat::PctValue::Str(msg),
                pycompat::PctValue::Float(e[0]),
                pycompat::PctValue::Float(e[1]),
                pycompat::PctValue::Float(e[2]),
                pycompat::PctValue::Float(e[3]),
            ],
        )
        .expect("move_error format");
        KlippyError::command(text)
    }
}

struct FakeHomingState {
    fail_at: Option<usize>,
    axes: Mutex<Vec<i64>>,
    set_axes_calls: Mutex<Vec<Vec<i64>>>,
    home_rails_calls: Mutex<Vec<HomeRailsCall>>,
}

/// `(rail names, forcepos, movepos)` of one `home_rails` call.
type HomeRailsCall = (Vec<String>, Vec<Option<f64>>, Vec<Option<f64>>);

impl HomingStateApi for FakeHomingState {
    fn set_axes(&self, axes: Vec<i64>) {
        self.set_axes_calls.lock().push(axes.clone());
        *self.axes.lock() = axes;
    }
    fn get_axes(&self) -> Vec<i64> {
        self.axes.lock().clone()
    }
    fn get_trigger_position(&self, _n: &str) -> KResult<i64> {
        panic!("deltesian never reads trigger positions")
    }
    fn set_stepper_adjustment(&self, _n: &str, _a: f64) {
        panic!("deltesian never adjusts steppers")
    }
    fn set_homed_position(&self, _p: Vec<Option<f64>>) -> KResult<()> {
        panic!("deltesian never sets the homed position")
    }
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        let mut calls = self.home_rails_calls.lock();
        calls.push((
            rails.iter().map(|r| r.get_name(None)).collect(),
            forcepos,
            movepos,
        ));
        if self.fail_at == Some(calls.len() - 1) {
            return Err(KlippyError::command("homing failed"));
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

thread_local! {
    static LOG: RefCell<Vec<(String, String)>> = const { RefCell::new(Vec::new()) };
}

static SINK: Once = Once::new();

/// Each test runs on its own thread, so a thread-local buffer isolates its log records.
fn install_sink() {
    SINK.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG.with(|l| {
                l.borrow_mut()
                    .push((rec.level.name().to_string(), rec.message.to_string()));
            });
        }));
    });
}

fn take_log() -> Value {
    LOG.with(|l| {
        Value::List(
            l.borrow_mut()
                .drain(..)
                .map(|(level, msg)| Value::Tuple(vec![Value::str(&level), Value::str(&msg)]))
                .collect(),
        )
    })
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let text = n.inner().to_string();
            match text.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(text),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(to_value).collect()),
        PyValue::Tuple(l) => Value::Tuple(l.iter().map(to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (to_value(k), to_value(v))).collect()),
        PyValue::Host(_) => panic!("host handle in a status value"),
    }
}

fn opt_float_value(v: Option<f64>) -> Value {
    v.map_or(Value::Null, Value::float)
}

fn floats_value(xs: &[f64]) -> Value {
    Value::List(xs.iter().map(|x| Value::float(*x)).collect())
}

fn rust_snapshot(shared: &Shared) -> Value {
    Value::List(
        shared
            .rails
            .lock()
            .iter()
            .map(|r| {
                let iterations = r
                    .iterations
                    .lock()
                    .iter()
                    .map(|(alloc, params)| {
                        Value::Tuple(vec![
                            Value::str(alloc),
                            Value::List(params.iter().map(to_value).collect()),
                        ])
                    })
                    .collect();
                let set_positions = r
                    .set_positions
                    .lock()
                    .iter()
                    .map(|c| floats_value(c))
                    .collect();
                let trapqs = r
                    .stepper
                    .trapq_calls
                    .lock()
                    .iter()
                    .map(|t| match t {
                        Some(_) => Value::str("trapq"),
                        None => Value::Null,
                    })
                    .collect();
                Value::Tuple(vec![
                    Value::str(&r.name),
                    Value::List(iterations),
                    Value::List(set_positions),
                    Value::List(vec![Value::Tuple(vec![
                        Value::str(&r.stepper.name),
                        Value::List(trapqs),
                    ])]),
                ])
            })
            .collect(),
    )
}

/// Compares what Python returned (or raised) with what the port returned.
fn cmp_value(label: &str, py: OracleResult, rust: KResult<Value>) {
    match (py, rust) {
        (Ok(expected), Ok(actual)) => eq(&format!("{label}: value"), &expected, &actual),
        (Err(e), Err(a)) => {
            assert_eq!(a.class_name(), e.class_name, "{label}: error class");
            assert_eq!(a.message(), e.message, "{label}: error message");
        }
        (Ok(expected), Err(a)) => panic!(
            "{label}: python returned {} but the port raised {}: {}",
            expected.render(),
            a.class_name(),
            a.message()
        ),
        (Err(e), Ok(actual)) => panic!(
            "{label}: python raised {}: {} but the port returned {}",
            e.class_name,
            e.message,
            actual.render()
        ),
    }
}

fn expect_py(label: &str, r: OracleResult) -> Value {
    r.unwrap_or_else(|e| panic!("{label}: python raised {}: {}", e.class_name, e.message))
}

/// Compares two recorded values: floats by their bits (CPython's `float.hex()` and the
/// recording formatter spell the same float differently), everything else exactly.
fn eq(label: &str, expected: &Value, actual: &Value) {
    let record = Record {
        v: 1,
        unit: "kinematics.deltesian".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "value", expected, actual) {
        panic!("{label}: {d}");
    }
}

fn oracle() -> Option<Oracle> {
    match Oracle::start() {
        Some(mut py) => {
            expect_py("setup", py.exec(SETUP));
            Some(py)
        }
        None => {
            println!("skipping deltesian oracle tests: KLIPPY_PYTHON is not set");
            None
        }
    }
}

struct Harness {
    kin: Kinematics,
    shared: Arc<Shared>,
}

/// Builds the kinematics on both sides and compares everything init leaves behind. Returns the
/// harness when both sides succeeded.
fn init_case(py: &mut Oracle, scn: &Scn, label: &str) -> Option<Harness> {
    expect_py(label, py.exec(&scn.py_load()));
    let py_res = py.exec("kin = make_kin()");
    install_sink();
    take_log();
    let shared = Shared::new(scn);
    let toolhead: ToolHead = Arc::new(FakeToolhead {
        factory: Arc::new(FakeStepperFactory {
            shared: shared.clone(),
        }),
        max_v: scn.max_v,
        max_a: scn.max_a,
    });
    let cfg: Config = Arc::new(FakeConfig {
        section: "printer".to_string(),
        shared: shared.clone(),
    });
    let rust_res = deltesian::load_kinematics(&toolhead, &cfg);

    eq(&format!("{label}: log records"), &expect_py(label, py.eval("LOG")), &take_log());
    eq(
        &format!("{label}: config options read"),
        &expect_py(label, py.eval("CONFIG_CALLS")),
        &Value::List(shared.config_calls.lock().clone()),
    );
    eq(
        &format!("{label}: rail lookups"),
        &expect_py(label, py.eval("RAIL_CALLS")),
        &Value::List(shared.rail_calls.lock().clone()),
    );
    eq(
        &format!("{label}: rail setup"),
        &expect_py(label, py.eval("snapshot()")),
        &rust_snapshot(&shared),
    );
    match (py_res, rust_res) {
        (Ok(_), Ok(kin)) => {
            let h = Harness { kin, shared };
            compare_status(py, &h, label);
            let steppers = h
                .kin
                .get_steppers()
                .iter()
                .map(|s| Value::str(&s.get_name(None)))
                .collect();
            eq(
                &format!("{label}: get_steppers"),
                &expect_py(label, py.eval("[s.get_name() for s in kin.get_steppers()]")),
                &Value::List(steppers),
            );
            Some(h)
        }
        (py_res, rust_res) => {
            cmp_value(
                label,
                py_res.map(|_| Value::Null),
                rust_res.map(|_| Value::Null),
            );
            None
        }
    }
}

fn compare_status(py: &mut Oracle, h: &Harness, label: &str) {
    eq(
        &format!("{label}: get_status"),
        &expect_py(label, py.eval("kin.get_status(0.0)")),
        &to_value(&h.kin.get_status(0.0)),
    );
}

fn compare_rails(py: &mut Oracle, h: &Harness, label: &str) {
    eq(
        &format!("{label}: rail calls"),
        &expect_py(label, py.eval("snapshot()")),
        &rust_snapshot(&h.shared),
    );
}

fn op_set_position(py: &mut Oracle, h: &Harness, label: &str, newpos: &[f64], axes: &str) {
    let py_res = py.eval(&format!("kin.set_position({}, '{axes}')", pyl(newpos)));
    let rust_res = h.kin.set_position(newpos.to_vec(), axes).map(|_| Value::Null);
    cmp_value(label, py_res, rust_res);
    compare_status(py, h, label);
    compare_rails(py, h, label);
}

fn op_clear(py: &mut Oracle, h: &Harness, label: &str, axes: &str) {
    let py_res = py.eval(&format!("kin.clear_homing_state('{axes}')"));
    h.kin.clear_homing_state(axes);
    cmp_value(label, py_res, Ok(Value::Null));
    compare_status(py, h, label);
}

fn rec_f64(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("float value"),
        Value::Int(i) => *i as f64,
        other => panic!("not numeric: {other:?}"),
    }
}

fn eval_floats(py: &mut Oracle, expr: &str) -> Vec<f64> {
    match expect_py(expr, py.eval(expr)) {
        Value::List(items) | Value::Tuple(items) => items.iter().map(rec_f64).collect(),
        other => panic!("{expr}: not a list: {other:?}"),
    }
}

/// Runs `kin.check_move` on a Python move and the port on the same numbers.
fn op_check_move(py: &mut Oracle, h: &Harness, label: &str, start: &str, end: &str) {
    expect_py(label, py.exec(&format!("mv = FakeMove({start}, {end})")));
    let mv = FakeMove {
        start: eval_floats(py, "list(mv.start_pos)"),
        end: eval_floats(py, "list(mv.end_pos)"),
        axes_d: eval_floats(py, "list(mv.axes_d)"),
        move_d: eval_floats(py, "[mv.move_d]")[0],
        limited: Mutex::new(Vec::new()),
    };
    let py_res = py.eval("kin.check_move(mv)");
    let rust_res = h.kin.check_move(&mv).map(|_| Value::Null);
    cmp_value(label, py_res, rust_res);
    let limited = Value::List(
        mv.limited
            .lock()
            .iter()
            .map(|(s, a)| Value::Tuple(vec![Value::float(*s), Value::float(*a)]))
            .collect(),
    );
    eq(
        &format!("{label}: limit_speed calls"),
        &expect_py(label, py.eval("mv.limited")),
        &limited,
    );
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_matches_python() {
    let Some(mut py) = oracle() else { return };
    let tuned = Scn::base()
        .set("printer", "min_angle", 10.0)
        .set("printer", "slow_ratio", 2.5)
        .set("printer", "print_width", 30.0)
        .set("printer", "minimum_z_position", 5.0)
        .set("printer", "max_z_velocity", 100.0)
        .set("printer", "max_z_accel", 500.0);
    let cases: Vec<(&str, Scn)> = vec![
        ("defaults", Scn::base()),
        ("every printer option", tuned),
        (
            "right inherits arm_x_length, arm_length and endstop",
            Scn::base()
                .without("stepper_right", "arm_x_length")
                .without("stepper_right", "arm_length"),
        ),
        ("slow_ratio 0 disables slowing", Scn::base().set("printer", "slow_ratio", 0.0)),
        ("slow_ratio 0.3", slow_scn()),
        ("min_angle 90", Scn::base().set("printer", "min_angle", 90.0)),
        ("min_angle 0", Scn::base().set("printer", "min_angle", 0.0)),
        ("print_width 0 is falsy", Scn::base().set("printer", "print_width", 0.0)),
        ("explicit minimum_z_position 0.0", Scn::base().set("printer", "minimum_z_position", 0.0)),
        ("negative minimum_z_position", Scn::base().set("printer", "minimum_z_position", -5.0)),
        (
            "positive y homing with negative min",
            Scn::base()
                .set("stepper_y", "position_min", -50.0)
                .set("stepper_y", "position_max", 150.0)
                .set("stepper_y", "position_endstop", 150.0)
                .set("stepper_y", "positive_dir", 1.0),
        ),
        (
            "asymmetric arms",
            Scn::base()
                .set("stepper_left", "arm_x_length", 30.0)
                .set("stepper_left", "arm_length", 120.0)
                .set("stepper_left", "position_endstop", 10.0)
                .set("stepper_right", "arm_x_length", 10.0)
                .set("stepper_right", "arm_length", 80.0)
                .set("stepper_right", "position_endstop", 50.0),
        ),
        (
            "z limits at the toolhead maximum",
            Scn::base()
                .set("printer", "max_z_velocity", 300.0)
                .set("printer", "max_z_accel", 3000.0)
                .limits(300.0, 3000.0),
        ),
    ];
    for (label, scn) in &cases {
        let h = init_case(&mut py, scn, label);
        assert!(h.is_some(), "{label}: both sides must initialise");
    }
}

#[test]
fn init_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Scn)> = vec![
        ("missing left arm_x_length", Scn::base().without("stepper_left", "arm_x_length")),
        ("left arm_x_length 0", Scn::base().set("stepper_left", "arm_x_length", 0.0)),
        ("right arm_x_length negative", Scn::base().set("stepper_right", "arm_x_length", -1.0)),
        ("missing left arm_length", Scn::base().without("stepper_left", "arm_length")),
        ("left arm_length equals arm_x_length", Scn::base().set("stepper_left", "arm_length", 20.0)),
        ("right arm_length equals arm_x_length", Scn::base().set("stepper_right", "arm_length", 22.0)),
        ("min_angle below 0", Scn::base().set("printer", "min_angle", -1.0)),
        ("min_angle above 90", Scn::base().set("printer", "min_angle", 91.0)),
        ("print_width negative", Scn::base().set("printer", "print_width", -1.0)),
        ("minimum_z_position above z_max", Scn::base().set("printer", "minimum_z_position", 1000.0)),
        ("slow_ratio negative", Scn::base().set("printer", "slow_ratio", -0.5)),
        ("max_z_velocity 0", Scn::base().set("printer", "max_z_velocity", 0.0)),
        ("max_z_velocity above max", Scn::base().set("printer", "max_z_velocity", 500.0)),
        ("max_z_accel 0", Scn::base().set("printer", "max_z_accel", 0.0)),
        ("max_z_accel above max", Scn::base().set("printer", "max_z_accel", 5000.0)),
        ("slow_ratio squared overflows", Scn::base().set("printer", "slow_ratio", 1e200)),
        ("arm_length squared overflows", Scn::base().set("stepper_left", "arm_length", 1e200)),
    ];
    for (label, scn) in &cases {
        let h = init_case(&mut py, scn, label);
        assert!(h.is_none(), "{label}: both sides must fail");
    }
    // `maxval=x_kin_range` is an int in Python, so the message reads "maximum of 40".
    let wide = Scn::base().set("printer", "print_width", 41.0);
    let h = init_case(&mut py, &wide, "print_width above x_kin_range");
    assert!(h.is_none(), "print_width above x_kin_range: both sides must fail");
}

#[test]
fn calc_position_matches_python() {
    let Some(mut py) = oracle() else { return };
    for (label, scn) in [("symmetric", Scn::base()), ("asymmetric", {
        Scn::base()
            .set("stepper_left", "arm_x_length", 30.0)
            .set("stepper_left", "arm_length", 120.0)
            .set("stepper_right", "arm_x_length", 10.0)
            .set("stepper_right", "arm_length", 80.0)
    })] {
        let h = init_case(&mut py, &scn, label).expect("init");
        let positions: Vec<(&str, [f64; 3])> = vec![
            ("origin", [0.0, 0.0, 10.0]),
            ("offset", [5.0, -3.0, 20.0]),
            ("equal actuators", [10.0, 10.0, 0.0]),
            ("right higher", [10.0, 14.5, 7.0]),
            ("tiny difference", [127.98, 127.98000000000002, 1.0]),
            ("negative", [-40.0, -35.5, -2.0]),
            ("domain error", [0.0, 500.0, 0.0]),
            ("overflow", [0.0, 1e200, 0.0]),
        ];
        for (name, [l, r, y]) in positions {
            let label = format!("{label} {name}");
            let dict = format!(
                "{{'stepper_left': {}, 'stepper_right': {}, 'stepper_y': {}}}",
                pyf(l),
                pyf(r),
                pyf(y)
            );
            let py_res = py.eval(&format!("kin.calc_position({dict})"));
            let mut map = IndexMap::new();
            map.insert("stepper_left".to_string(), l);
            map.insert("stepper_right".to_string(), r);
            map.insert("stepper_y".to_string(), y);
            let rust_res = h.kin.calc_position(&map).map(|v| {
                Value::List(v.into_iter().map(opt_float_value).collect())
            });
            cmp_value(&label, py_res, rust_res);
        }
        // KeyError for the first missing stepper in rail order.
        for (name, keys) in [
            ("missing left", vec!["stepper_right", "stepper_y"]),
            ("missing right", vec!["stepper_left", "stepper_y"]),
            ("missing y", vec!["stepper_left", "stepper_right"]),
            ("missing all", vec![]),
            ("other names", vec!["x", "stepper_y"]),
        ] {
            let label = format!("{label} {name}");
            let dict: Vec<String> = keys.iter().map(|k| format!("'{k}': 1.5")).collect();
            let py_res = py.eval(&format!("kin.calc_position({{{}}})", dict.join(", ")));
            let map: IndexMap<String, f64> = keys.iter().map(|k| (k.to_string(), 1.5)).collect();
            let rust_res = h.kin.calc_position(&map).map(|v| {
                Value::List(v.into_iter().map(opt_float_value).collect())
            });
            cmp_value(&label, py_res, rust_res);
        }
    }
}

#[test]
fn set_position_and_homing_state_match_python() {
    let Some(mut py) = oracle() else { return };
    let h = init_case(&mut py, &Scn::base(), "init").expect("init");
    op_set_position(&mut py, &h, "set xz", &[1.0, 2.0, 3.0], "xz");
    op_clear(&mut py, &h, "clear x", "x");
    op_set_position(&mut py, &h, "set no axes", &[4.0, 5.0, 6.0], "");
    op_set_position(&mut py, &h, "set y", &[4.0, 5.0, 6.0], "y");
    op_set_position(&mut py, &h, "set repeated z", &[7.0, 8.0, 9.0], "zz");
    op_set_position(&mut py, &h, "set four coordinates", &[1.0, 2.0, 3.0, 4.0], "");
    op_clear(&mut py, &h, "clear unknown letters", "q");
    op_clear(&mut py, &h, "clear xyz", "xyz");
    op_set_position(&mut py, &h, "partial invalid axes", &[1.0, 2.0, 3.0], "xq");
    op_set_position(&mut py, &h, "invalid first axis", &[1.0, 2.0, 3.0], "qx");
    op_set_position(&mut py, &h, "short position", &[1.0, 2.0], "xyz");
    op_set_position(&mut py, &h, "set xyz", &[0.0, 0.0, 0.0], "xyz");
    op_clear(&mut py, &h, "clear y", "y");
    op_clear(&mut py, &h, "clear nothing", "");
    op_clear(&mut py, &h, "clear xz", "xz");
}

#[test]
fn home_matches_python() {
    let Some(mut py) = oracle() else { return };
    let scenarios = [
        ("negative direction", Scn::base()),
        (
            "positive direction",
            Scn::base()
                .set("stepper_y", "position_min", -50.0)
                .set("stepper_y", "position_max", 150.0)
                .set("stepper_y", "position_endstop", 150.0)
                .set("stepper_y", "positive_dir", 1.0),
        ),
        (
            "asymmetric arms",
            Scn::base()
                .set("stepper_left", "arm_x_length", 30.0)
                .set("stepper_left", "arm_length", 120.0)
                .set("stepper_right", "arm_x_length", 10.0)
                .set("stepper_right", "arm_length", 80.0),
        ),
    ];
    let axis_sets: [&[i64]; 9] = [
        &[],
        &[0],
        &[1],
        &[2],
        &[0, 1],
        &[0, 2],
        &[1, 2],
        &[0, 1, 2],
        &[2, 2, 1],
    ];
    for (name, scn) in &scenarios {
        let h = init_case(&mut py, scn, name).expect("init");
        for axes in axis_sets {
            for fail_at in [None, Some(0usize), Some(1usize)] {
                let label = format!("{name} axes {axes:?} fail_at {fail_at:?}");
                let axes_py: Vec<String> = axes.iter().map(|a| a.to_string()).collect();
                let fail_py = fail_at.map_or("None".to_string(), |n| n.to_string());
                expect_py(
                    &label,
                    py.exec(&format!(
                        "hs = FakeHomingState([{}], {fail_py})",
                        axes_py.join(", ")
                    )),
                );
                let hs = Arc::new(FakeHomingState {
                    fail_at,
                    axes: Mutex::new(axes.to_vec()),
                    set_axes_calls: Mutex::new(Vec::new()),
                    home_rails_calls: Mutex::new(Vec::new()),
                });
                let handle: HomingStateHandle = hs.clone();
                let py_res = py.eval("kin.home(hs)");
                let rust_res = h.kin.home(handle).map(|_| Value::Null);
                cmp_value(&label, py_res, rust_res);
                let int_list = |v: &[i64]| Value::List(v.iter().map(|i| Value::Int(*i)).collect());
                let set_axes = Value::List(
                    hs.set_axes_calls.lock().iter().map(|c| int_list(c)).collect(),
                );
                let home_rails = Value::List(
                    hs.home_rails_calls
                        .lock()
                        .iter()
                        .map(|(names, force, mv)| {
                            Value::Tuple(vec![
                                Value::List(names.iter().map(|n| Value::str(n)).collect()),
                                Value::List(force.iter().map(|v| opt_float_value(*v)).collect()),
                                Value::List(mv.iter().map(|v| opt_float_value(*v)).collect()),
                            ])
                        })
                        .collect(),
                );
                let expected = expect_py(
                    &label,
                    py.eval("[hs.set_axes_calls, hs.home_rails_calls, hs.axes]"),
                );
                let actual = Value::List(vec![
                    set_axes,
                    home_rails,
                    int_list(&hs.axes.lock()),
                ]);
                eq(&format!("{label}: homing state"), &expected, &actual);
            }
        }
    }
}

/// Moves: homed on every axis, then partially homed, then unhomed.
fn move_cases() -> Vec<(&'static str, &'static str, &'static str)> {
    vec![
        ("x in range", "[0.0, 10.0, 5.0, 0.0]", "[10.0, 10.0, 5.0, 0.0]"),
        ("x above range", "[0.0, 10.0, 5.0, 0.0]", "[30.0, 10.0, 5.0, 0.0]"),
        ("x below range", "[0.0, 10.0, 5.0, 0.0]", "[-25.0, 10.0, 5.0, 0.0]"),
        ("x at the limits", "[-20.0, 10.0, 5.0, 0.0]", "[22.0, 10.0, 5.0, 0.0]"),
        ("y in range", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 200.0, 5.0, 0.0]"),
        ("y above range", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 300.0, 5.0, 0.0]"),
        ("y below range", "[0.0, 10.0, 5.0, 0.0]", "[0.0, -5.0, 5.0, 0.0]"),
        ("z below minimum", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 10.0, -1.0, 0.0]"),
        ("z in range", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 10.0, 12.0, 0.0]"),
        ("diagonal xz", "[0.0, 10.0, 5.0, 0.0]", "[10.0, 10.0, 15.0, 0.0]"),
        ("diagonal xyz", "[0.0, 10.0, 5.0, 0.0]", "[7.0, 40.0, 9.0, 0.0]"),
        ("z just below the top", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 10.0, 29.0, 0.0]"),
        ("z just above the top", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 10.0, 30.5, 0.0]"),
        ("z far above the top", "[0.0, 10.0, 5.0, 0.0]", "[0.0, 10.0, 40.0, 0.0]"),
        ("z above the top at x extreme", "[0.0, 10.0, 5.0, 0.0]", "[19.0, 10.0, 40.0, 0.0]"),
        ("top limit domain error", "[0.0, 10.0, 5.0, 0.0]", "[200.0, 10.0, 40.0, 0.0]"),
        ("homing move", "[10.0, 0.0, 0.0, 0.0]", "[0.0, 0.0, kin.home_z, 0.0]"),
        ("homing move from the origin", "[0.0, 0.0, 0.0, 0.0]", "[0.0, 0.0, kin.home_z, 0.0]"),
        ("home_z with y motion", "[10.0, 0.0, 0.0, 0.0]", "[0.0, 5.0, kin.home_z, 0.0]"),
        ("x zero but z not home_z", "[10.0, 0.0, 0.0, 0.0]", "[0.0, 0.0, 1.0, 0.0]"),
        ("extruder only", "[1.0, 2.0, 3.0, 0.0]", "[1.0, 2.0, 3.0, 5.0]"),
        ("zero length", "[1.0, 2.0, 3.0, 0.0]", "[1.0, 2.0, 3.0, 0.0]"),
    ]
}

#[test]
fn check_move_matches_python() {
    let Some(mut py) = oracle() else { return };
    let h = init_case(&mut py, &Scn::base(), "init").expect("init");
    for (name, start, end) in move_cases() {
        op_check_move(&mut py, &h, &format!("unhomed {name}"), start, end);
    }
    op_set_position(&mut py, &h, "home x", &[0.0, 0.0, 0.0], "x");
    for (name, start, end) in move_cases() {
        op_check_move(&mut py, &h, &format!("x homed {name}"), start, end);
    }
    op_set_position(&mut py, &h, "home all", &[0.0, 0.0, 0.0], "xyz");
    for (name, start, end) in move_cases() {
        op_check_move(&mut py, &h, &format!("homed {name}"), start, end);
    }
    // A toolhead whose velocity and acceleration differ changes the z limits.
    let scn = Scn::base()
        .set("printer", "max_z_velocity", 40.0)
        .set("printer", "max_z_accel", 400.0)
        .set("printer", "print_width", 30.0);
    let h = init_case(&mut py, &scn, "narrow").expect("init");
    op_set_position(&mut py, &h, "narrow home", &[0.0, 0.0, 0.0], "xyz");
    for (name, start, end) in move_cases() {
        op_check_move(&mut py, &h, &format!("narrow {name}"), start, end);
    }
}

/// `check_move` reads `self.max_velocity`, which `DeltesianKinematics` never defines: moves past
/// the slow thresholds raise `AttributeError` in the reference.
#[test]
fn check_move_slow_region_matches_python() {
    let Some(mut py) = oracle() else { return };
    let h = init_case(&mut py, &slow_scn(), "init").expect("init");
    op_set_position(&mut py, &h, "home all", &[0.0, 0.0, 0.0], "xyz");
    let moves = [
        ("below the slow threshold", "[0.0, 0.0, 5.0, 0.0]", "[5.0, 0.0, 5.0, 0.0]"),
        ("past the slow threshold", "[0.0, 0.0, 5.0, 0.0]", "[10.0, 0.0, 5.0, 0.0]"),
        ("past the very slow threshold", "[0.0, 0.0, 5.0, 0.0]", "[19.5, 0.0, 5.0, 0.0]"),
        ("slow start position", "[10.0, 0.0, 5.0, 0.0]", "[8.0, 0.0, 5.0, 0.0]"),
        ("negative x", "[0.0, 0.0, 5.0, 0.0]", "[-15.0, 0.0, 5.0, 0.0]"),
        ("y only", "[10.0, 0.0, 5.0, 0.0]", "[10.0, 50.0, 5.0, 0.0]"),
        ("z only", "[10.0, 0.0, 5.0, 0.0]", "[10.0, 0.0, 9.0, 0.0]"),
        ("x with z limit", "[0.0, 0.0, 5.0, 0.0]", "[10.0, 0.0, 12.0, 0.0]"),
        ("squared start overflows", "[1e200, 0.0, 0.0, 0.0]", "[0.0, 0.0, kin.home_z, 0.0]"),
    ];
    for (name, start, end) in moves {
        op_check_move(&mut py, &h, name, start, end);
    }
    // Disabled slowing never reaches the attribute.
    let off = Scn::base().set("printer", "slow_ratio", 0.0);
    let h = init_case(&mut py, &off, "init without slowing").expect("init");
    op_set_position(&mut py, &h, "home all", &[0.0, 0.0, 0.0], "xyz");
    for (name, start, end) in moves {
        op_check_move(&mut py, &h, &format!("no slowing {name}"), start, end);
    }
}
