//! Parity tests for `kinematics::rotary_delta` (port of
//! `klippy/kinematics/rotary_delta.py`).
//!
//! Every expected value comes from the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a Python
//! `RotaryDeltaKinematics` / `RotaryDeltaCalibration` is built on
//! both sides with minimal Python fakes for the toolhead, the
//! config, the stepper factory and the rails, fed identical
//! geometry, and the resulting state (`max_z`, `limit_z`, ...),
//! the calibration math (`actuator_to_cartesian`,
//! `get_position_from_stable`, `calc_stable_position`), the move
//! checks, the homing call and the `save_state` output are
//! compared bit-for-bit (floats by bits via the recording value
//! format), together with the raised error classes and messages.
//!
//! The calibration math is a true differential test of the C
//! `rotary_delta_stepper_alloc` / `itersolve_calc_position_from_coord`
//! pair: the Python side runs the real chelper C code, while the
//! Rust side runs the port's local reproduction of it.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use kinematics::rotary_delta::{load_kinematics, RotaryDeltaCalibration, RotaryDeltaKinematics};
use klippy_api::config::{Config, GetFloatOpts, PrinterConfig};
use klippy_api::gcode::GCodeApi;
use klippy_api::printer::{LookupDefault, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, DeltaCalibrationApi, Kinematics, KinematicsApi, MoveApi, ToolHeadApi,
};
use klippy_api::printer::Printer;
use klippy_api::{log, KResult, KlippyError, ObjectRef};
use pycompat::PyValue;
use pycompat::libm;
use pycompat::text::{percent_format, PctValue};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

const SETUP: &str = r#"
import math, logging as _logging
import stepper as _stepper
from kinematics import rotary_delta

class Error(Exception): pass
class CommandError(Exception): pass
_sentinel = object()

class FakeConfig:
    def __init__(self, name, opts, sections=None):
        self.name = name; self.opts = opts; self.sections = sections or {}
    def get_name(self): return self.name
    def get_printer(self): return None
    def getsection(self, name): return self.sections[name]
    def getfloat(self, opt, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if opt not in self.opts:
            if default is not _sentinel:
                return default
            raise Error("Option '%s' in section '%s' must be specified" % (opt, self.name))
        v = float(self.opts[opt])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (opt, self.name, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (opt, self.name, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (opt, self.name, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (opt, self.name, below))
        return v

class FakeTrapq: pass
TRAPQ = FakeTrapq()

class FakeToolhead:
    def get_max_velocity(self): return (300., 3000.)
    def get_trapq(self): return TRAPQ
    def Coord(self, t):
        t = tuple(t)
        if len(t) < 4: t = t + (0,) * (4 - len(t))
        return t

class FakeHomingInfo:
    def __init__(self, position_endstop): self.position_endstop = position_endstop

class FakeStepper:
    def __init__(self, step_dist):
        self.step_dist = step_dist; self.trapqs = []; self.positions = []
    def get_step_dist(self): return self.step_dist
    def set_trapq(self, tq): self.trapqs.append(tq)
    def set_position(self, coord): self.positions.append(tuple(coord))

class FakeRail:
    def __init__(self, name, endstop, step_dist):
        self.name = name; self.endstop = endstop
        self.stepper = FakeStepper(step_dist); self.itersolve = None
    def get_name(self, short=False): return self.name
    def get_homing_info(self): return FakeHomingInfo(self.endstop)
    def get_steppers(self): return [self.stepper]
    def setup_itersolve(self, alloc, *params): self.itersolve = (alloc, tuple(params))
    def set_position(self, coord): self.stepper.set_position(coord)
    def calc_position_from_coord(self, coord):
        sr, sh, ang, ua, la = self.itersolve[1]
        x, y, z = coord
        ca, sa = math.cos(ang), math.sin(ang)
        sjz = y*ca - x*sa; sjx = x*ca + y*sa - sr; sjy = z - sh
        dx, dy = sjx, sjy
        ua2 = ua*ua; la2 = la*la - sjz*sjz
        inv = 1./dy; c1 = .5*inv*(dx*dx+dy*dy+ua2-la2); c2 = dx*inv
        scale = c2*c2 + 1.; sx = c1*c2 + math.sqrt(scale*ua2 - c1*c1); sy = c1*scale - c2*sx
        return math.atan2(sy, sx)

RAIL_CALLS = []
RAILS = {}
STEPDISTS = {'stepper_a': 0.01, 'stepper_b': 0.011, 'stepper_c': 0.0125}
def fake_lookup_rail(config, need_position_minmax=True, default_position_endstop=None, units_in_radians=False):
    RAIL_CALLS.append((config.get_name(), need_position_minmax, default_position_endstop, units_in_radians))
    name = config.get_name()
    if 'position_endstop' in config.opts:
        endstop = config.opts['position_endstop']
    elif default_position_endstop is None:
        raise Error("Option '%s' in section '%s' must be specified" % ('position_endstop', name))
    else:
        endstop = default_position_endstop
    rail = FakeRail(name, endstop, STEPDISTS[name])
    RAILS[name] = rail
    return rail
_stepper.LookupRail = fake_lookup_rail

class FakeMove:
    def __init__(self, end_pos, axes_d, accel):
        self.end_pos = tuple(end_pos); self.axes_d = list(axes_d)
        self.accel = accel; self.limit_calls = []
    def limit_speed(self, speed, accel): self.limit_calls.append((speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return CommandError(m)

class FakeHomingState:
    def __init__(self): self.axes = None; self.calls = []
    def set_axes(self, axes): self.axes = list(axes)
    def home_rails(self, rails, forcepos, movepos): self.calls.append((len(rails), list(forcepos), list(movepos)))

class FakeGCode:
    def __init__(self): self.infos = []
    def respond_info(self, msg, log=True): self.infos.append((msg, log))
class FakePrinter:
    def __init__(self): self.gcode = FakeGCode()
    def lookup_object(self, name, default=None):
        assert name == 'gcode'
        return self.gcode
class FakePrinterConfig:
    def __init__(self, printer): self.printer = printer; self.set_calls = []
    def set(self, section, option, value): self.set_calls.append((section, option, value))
    def get_printer(self): return self.printer

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

def make_py_kin():
    global kin
    sections = {
      'stepper_a': FakeConfig('stepper_a', {'position_endstop': 252.0, 'upper_arm_length': 170.0, 'lower_arm_length': 320.0}),
      'stepper_b': FakeConfig('stepper_b', {}),
      'stepper_c': FakeConfig('stepper_c', {}),
    }
    cfg = FakeConfig('rotary_delta', {'shoulder_radius': 33.9, 'shoulder_height': 412.9, 'minimum_z_position': 0.0}, sections)
    kin = rotary_delta.RotaryDeltaKinematics(FakeToolhead(), cfg)
    return kin

def run_check(move):
    try:
        kin.check_move(move)
        return ('ok', kin.limit_xy2, kin.need_home, list(move.limit_calls))
    except Exception as e:
        return ('err', type(e).__name__, str(e), kin.limit_xy2, kin.need_home, list(move.limit_calls))
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

const SHOULDER_RADIUS: f64 = 33.9;
const SHOULDER_HEIGHT: f64 = 412.9;
const UPPER_ARM: f64 = 170.0;
const LOWER_ARM: f64 = 320.0;
const ENDSTOP_A: f64 = 252.0;
const STEPDISTS: [f64; 3] = [0.01, 0.011, 0.0125];

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn fake_config() -> Config {
    Arc::new(FakeConfig {
        name: "rotary_delta".to_string(),
        floats: IndexMap::from([
            ("shoulder_radius".to_string(), SHOULDER_RADIUS),
            ("shoulder_height".to_string(), SHOULDER_HEIGHT),
            ("minimum_z_position".to_string(), 0.0),
        ]),
        sections: Arc::new(IndexMap::from([
            (
                "stepper_a".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_a".to_string(),
                    floats: IndexMap::from([
                        ("position_endstop".to_string(), ENDSTOP_A),
                        ("upper_arm_length".to_string(), UPPER_ARM),
                        ("lower_arm_length".to_string(), LOWER_ARM),
                    ]),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
            (
                "stepper_b".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_b".to_string(),
                    floats: IndexMap::new(),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
            (
                "stepper_c".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_c".to_string(),
                    floats: IndexMap::new(),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
        ])),
    })
}

/// The fake config error messages expand a float the way Python's
/// `%s` does (shortest round-trip spelling, `repr`).
fn float_str(v: f64) -> String {
    format!("{v:?}")
}

struct FakeConfig {
    name: String,
    floats: IndexMap<String, f64>,
    sections: Arc<IndexMap<String, Arc<FakeConfig>>>,
}

impl FakeConfig {
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(&v) = self.floats.get(option) else {
            return match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            };
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
}

impl klippy_api::config::ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        panic!("get_printer: not used by rotary_delta")
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, _option: &str, _opts: klippy_api::config::GetStrOpts) -> KResult<Option<String>> {
        panic!("get: not used by rotary_delta")
    }
    fn getint(&self, _option: &str, _opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint: not used by rotary_delta")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        FakeConfig::getfloat(self, option, opts)
    }
    fn getboolean(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetBoolOpts,
    ) -> KResult<Option<bool>> {
        panic!("getboolean: not used by rotary_delta")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice: not used by rotary_delta")
    }
    fn getlists(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetListsOpts,
    ) -> KResult<PyValue> {
        panic!("getlists: not used by rotary_delta")
    }
    fn getlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        panic!("getlist: not used by rotary_delta")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist: not used by rotary_delta")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist: not used by rotary_delta")
    }
    fn getsection(&self, section: &str) -> Config {
        let inner = self
            .sections
            .get(section)
            .unwrap_or_else(|| panic!("getsection: unknown section {section}"));
        Arc::new(FakeConfig {
            name: inner.name.clone(),
            floats: inner.floats.clone(),
            sections: Arc::clone(&inner.sections),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections: not used by rotary_delta")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options: not used by rotary_delta")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate: not used by rotary_delta")
    }
}

struct FakeStepper {
    step_dist: f64,
    trapqs: std::sync::Mutex<Vec<bool>>,
    positions: std::sync::Mutex<Vec<[f64; 3]>>,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("get_mcu: not used by rotary_delta")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        panic!("get_name: not used by rotary_delta")
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid: not used by rotary_delta")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians: not used by rotary_delta")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration: not used by rotary_delta")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration: not used by rotary_delta")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("stepper setup_itersolve: not used by rotary_delta")
    }
    fn get_step_dist(&self) -> f64 {
        self.step_dist
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance: not used by rotary_delta")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance: not used by rotary_delta")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted: not used by rotary_delta")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted: not used by rotary_delta")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("stepper calc_position_from_coord: not used by rotary_delta")
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.positions.lock().expect("positions").push(coord);
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position: not used by rotary_delta")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        panic!("get_mcu_position: not used by rotary_delta")
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        panic!("get_past_mcu_position: not used by rotary_delta")
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: i64) -> f64 {
        panic!("mcu_to_commanded_position: not used by rotary_delta")
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        panic!("dump_steps: not used by rotary_delta")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics: not used by rotary_delta")
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<StepperKinematics>,
    ) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics: not used by rotary_delta")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end: not used by rotary_delta")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq: not used by rotary_delta")
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        self.trapqs.lock().expect("trapqs").push(tq.is_some());
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        panic!("add_active_callback: not used by rotary_delta")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis: not used by rotary_delta")
    }
}

struct FakeRail {
    name: String,
    endstop: f64,
    stepper: Arc<FakeStepper>,
    itersolve: std::sync::Mutex<Option<(String, Vec<f64>)>>,
}

impl FakeRail {
    fn calc(&self, x: f64, y: f64, z: f64) -> f64 {
        let (_alloc, params) = self.itersolve.lock().expect("itersolve").clone().expect("itersolve set");
        let (sr, sh, ang, ua, la) = (params[0], params[1], params[2], params[3], params[4]);
        let ca = libm::cos(ang);
        let sa = libm::sin(ang);
        let sjz = y * ca - x * sa;
        let sjx = x * ca + y * sa - sr;
        let sjy = z - sh;
        let ua2 = ua * ua;
        let la2 = la * la - sjz * sjz;
        let inv = 1.0 / sjy;
        let c1 = 0.5 * inv * (sjx * sjx + sjy * sjy + ua2 - la2);
        let c2 = sjx * inv;
        let scale = c2 * c2 + 1.0;
        let sx = c1 * c2 + libm::sqrt(scale * ua2 - c1 * c1);
        let sy = c1 * scale - c2 * sx;
        libm::atan2(sy, sx)
    }
}

impl RailApi for FakeRail {
    fn get_name(&self, short: Option<bool>) -> String {
        if short.unwrap_or(false) {
            if let Some(rest) = self.name.strip_prefix("stepper") {
                return rest.to_string();
            }
            return self.name.split_whitespace().last().unwrap_or("").to_string();
        }
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        panic!("get_range: not used by rotary_delta")
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop: self.endstop,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir: true,
            second_homing_speed: 2.5,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        vec![self.stepper.clone()]
    }
    fn get_endstops(&self) -> Vec<(Arc<dyn klippy_api::mcu::EndstopApi>, String)> {
        panic!("get_endstops: not used by rotary_delta")
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Arc<dyn klippy_api::mcu::EndstopApi>> {
        panic!("lookup_endstop: not used by rotary_delta")
    }
    fn add_stepper(
        &self,
        _stepper: Stepper,
        _endstop_pin: Option<&str>,
        _endstop_name: Option<&str>,
    ) -> KResult<()> {
        panic!("add_stepper: not used by rotary_delta")
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        panic!("add_stepper_from_config: not used by rotary_delta")
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        let floats: Vec<f64> = params
            .iter()
            .map(|p| p.to_f64().expect("float itersolve params"))
            .collect();
        *self.itersolve.lock().expect("itersolve") = Some((alloc_func.to_string(), floats));
    }
    fn set_trapq(&self, tq: Option<Trapq>) {
        self.stepper.set_trapq(tq);
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.stepper.set_position(coord);
    }
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 {
        self.calc(coord[0], coord[1], coord[2])
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position: not used by rotary_delta")
    }
}

struct FakeFactory {
    stepdists: IndexMap<String, f64>,
    // Concrete handles so tests can read the fakes' records.
    fakes: std::sync::Mutex<Vec<Arc<FakeRail>>>,
    calls: std::sync::Mutex<Vec<(String, bool, Option<f64>, bool)>>,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper: not used by rotary_delta")
    }
    fn lookup_rail(&self, config: &Config, opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        self.calls.lock().expect("calls").push((
            config.get_name(),
            opts.need_position_minmax,
            opts.default_position_endstop,
            opts.units_in_radians,
        ));
        let endstop = config
            .getfloat(
                "position_endstop",
                GetFloatOpts {
                    default: opts.default_position_endstop.map(Some),
                    ..Default::default()
                },
            )?
            .expect("position_endstop available");
        let name = config.get_name();
        let step_dist = *self.stepdists.get(&name).expect("step_dist");
        let fake = Arc::new(FakeRail {
            name,
            endstop,
            stepper: Arc::new(FakeStepper {
                step_dist,
                trapqs: std::sync::Mutex::new(Vec::new()),
                positions: std::sync::Mutex::new(Vec::new()),
            }),
            itersolve: std::sync::Mutex::new(None),
        });
        self.fakes.lock().expect("fakes").push(fake.clone());
        let rail: klippy_api::stepper::Rail = fake;
        Ok(rail)
    }
    fn lookup_multi_rail(
        &self,
        _config: &Config,
        _opts: RailOpts,
    ) -> KResult<klippy_api::stepper::Rail> {
        panic!("lookup_multi_rail: not used by rotary_delta")
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        panic!("generic_rail: not used by rotary_delta")
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance: not used by rotary_delta")
    }
}

struct FakeToolhead {
    trapq: Trapq,
    factory: StepperFactory,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position: not used by rotary_delta")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position: not used by rotary_delta")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics: not used by rotary_delta")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation: not used by rotary_delta")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        panic!("get_last_move_time: not used by rotary_delta")
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        panic!("dwell: not used by rotary_delta")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move: not used by rotary_delta")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move: not used by rotary_delta")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move: not used by rotary_delta")
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves: not used by rotary_delta")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status: not used by rotary_delta")
    }
    fn get_trapq(&self) -> Trapq {
        self.trapq.clone()
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (300.0, 3000.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities: not used by rotary_delta")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback: not used by rotary_delta")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy: not used by rotary_delta")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed: not used by rotary_delta")
    }
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder: not used by rotary_delta")
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder: not used by rotary_delta")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis: not used by rotary_delta")
    }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis: not used by rotary_delta")
    }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("get_extra_axes: not used by rotary_delta")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats: not used by rotary_delta")
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

struct FakeMove {
    end_pos: Coord,
    axes_d: Coord,
    accel: f64,
    limit_calls: Arc<std::sync::Mutex<Vec<(f64, f64)>>>,
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        panic!("start_pos: not used by rotary_delta")
    }
    fn end_pos(&self) -> Coord {
        self.end_pos.clone()
    }
    fn axes_d(&self) -> Coord {
        self.axes_d.clone()
    }
    fn axes_r(&self) -> Coord {
        panic!("axes_r: not used by rotary_delta")
    }
    fn move_d(&self) -> f64 {
        panic!("move_d: not used by rotary_delta")
    }
    fn is_kinematic_move(&self) -> bool {
        panic!("is_kinematic_move: not used by rotary_delta")
    }
    fn accel(&self) -> f64 {
        self.accel
    }
    fn max_cruise_v2(&self) -> f64 {
        panic!("max_cruise_v2: not used by rotary_delta")
    }
    fn start_v(&self) -> f64 {
        panic!("start_v: not used by rotary_delta")
    }
    fn cruise_v(&self) -> f64 {
        panic!("cruise_v: not used by rotary_delta")
    }
    fn accel_t(&self) -> f64 {
        panic!("accel_t: not used by rotary_delta")
    }
    fn cruise_t(&self) -> f64 {
        panic!("cruise_t: not used by rotary_delta")
    }
    fn decel_t(&self) -> f64 {
        panic!("decel_t: not used by rotary_delta")
    }
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> {
        self.limit_calls
            .lock()
            .expect("limit_calls")
            .push((speed, accel));
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos.0;
        let text = percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                PctValue::Str(msg),
                PctValue::Float(ep[0]),
                PctValue::Float(ep[1]),
                PctValue::Float(ep[2]),
                PctValue::Float(ep[3]),
            ],
        )
        .expect("valid percent format");
        KlippyError::Command(text)
    }
}

struct FakeHomingState {
    axes: std::sync::Mutex<Vec<i64>>,
    calls: std::sync::Mutex<Vec<(usize, Vec<Option<f64>>, Vec<Option<f64>>)>>,
}

impl klippy_api::toolhead::HomingStateApi for FakeHomingState {
    fn set_axes(&self, axes: Vec<i64>) {
        *self.axes.lock().expect("axes") = axes;
    }
    fn get_axes(&self) -> Vec<i64> {
        self.axes.lock().expect("axes").clone()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        panic!("get_trigger_position: not used by rotary_delta")
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {
        panic!("set_stepper_adjustment: not used by rotary_delta")
    }
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        panic!("set_homed_position: not used by rotary_delta")
    }
    fn home_rails(
        &self,
        rails: Vec<klippy_api::stepper::Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        self.calls
            .lock()
            .expect("calls")
            .push((rails.len(), forcepos, movepos));
        Ok(())
    }
}

type Infos = Arc<std::sync::Mutex<Vec<(String, bool)>>>;

struct FakeGCode {
    infos: Infos,
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: klippy_api::gcode::RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        panic!("register_command: not used by rotary_delta")
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        panic!("register_mux_command: not used by rotary_delta")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help: not used by rotary_delta")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status: not used by rotary_delta")
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {
        panic!("register_output_handler: not used by rotary_delta")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart: not used by rotary_delta")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script: not used by rotary_delta")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command: not used by rotary_delta")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex: not used by rotary_delta")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("create_gcode_command: not used by rotary_delta")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw: not used by rotary_delta")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos
            .lock()
            .expect("infos")
            .push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode: not used by rotary_delta")
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        panic!("cmd_m112: not used by rotary_delta")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands: not used by rotary_delta")
    }
}

struct FakePrinter {
    gcode: Arc<FakeGCode>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        panic!("get_start_args: not used by rotary_delta")
    }
    fn get_reactor(&self) -> reactor::Reactor {
        panic!("get_reactor: not used by rotary_delta")
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("get_state_message: not used by rotary_delta")
    }
    fn is_shutdown(&self) -> bool {
        panic!("is_shutdown: not used by rotary_delta")
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("update_error_msg: not used by rotary_delta")
    }
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("add_object: not used by rotary_delta")
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        assert_eq!(name, "gcode");
        Ok(Some(Arc::new(FakeGCode {
            infos: self.gcode.infos.clone(),
        }) as ObjectRef))
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects: not used by rotary_delta")
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object: not used by rotary_delta")
    }
    fn run(&self) -> Option<String> {
        panic!("run: not used by rotary_delta")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("set_rollover_info: not used by rotary_delta")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_shutdown: not used by rotary_delta")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown: not used by rotary_delta")
    }
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {
        panic!("register_event_handler: not used by rotary_delta")
    }
    fn send_event(&self, _event: &str, _params: &[klippy_api::object::EventArg]) -> KResult<Vec<PyValue>> {
        panic!("send_event: not used by rotary_delta")
    }
    fn request_exit(&self, _result: &str) {
        panic!("request_exit: not used by rotary_delta")
    }
}

struct FakePrinterConfig {
    printer: Printer,
    set_calls: Arc<std::sync::Mutex<Vec<(String, String, String)>>>,
}

impl klippy_api::config::PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config: not used by rotary_delta")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config: not used by rotary_delta")
    }
    fn log_config(&self, _config: &Config) {
        panic!("log_config: not used by rotary_delta")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("check_unused_options: not used by rotary_delta")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("runtime_warning: not used by rotary_delta")
    }
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
        panic!("deprecate: not used by rotary_delta")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {
        panic!("deprecate_gcode: not used by rotary_delta")
    }
    fn deprecate_mcu_code(
        &self,
        _mcu: &klippy_api::mcu::Mcu,
        _feature: &str,
        _msg: Option<String>,
    ) {
        panic!("deprecate_mcu_code: not used by rotary_delta")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status: not used by rotary_delta")
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.set_calls.lock().expect("set_calls").push((
            section.to_string(),
            option.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn remove_section(&self, _section: &str) {
        panic!("remove_section: not used by rotary_delta")
    }
}

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        log::set_sink(Box::new(|rec| {
            LOG_LINES.with(|lines| {
                lines.borrow_mut().push(rec.message.to_string());
            });
        }));
    });
}

fn log_lines() -> Vec<String> {
    install_log_sink();
    LOG_LINES.with(|lines| lines.borrow().clone())
}
// `chelper.check_build_c_library` compiles `c_helper.so` to a shared
// temp name and renames it on the first `get_ffi()` call. Cargo test
// threads each start a CPython and the concurrent first builds race on
// the rename (`FileNotFoundError: ... '_temp_c_helper.so' ->
// 'c_helper.so'`). Serialize the first build under a lock so every
// later `get_ffi()` finds the library built and skips the rebuild.
static CHELPER_BUILD_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());


fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            {
                let _guard = CHELPER_BUILD_LOCK
                    .lock()
                    .expect("chelper build lock");
                py.exec("import chelper; chelper.get_ffi()")
                    .expect("chelper build");
            }
            py.exec(SETUP).expect("rotary_delta fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping rotary_delta oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    check_value(label, &expected, actual);
}

fn check_value(label: &str, expected: &Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "kinematics.rotary_delta".to_string(),
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
    if let Err(d) = assert_equal(&record, "ret", expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<Value>,
) {
    match (expected, actual) {
        (Ok(want), Ok(got)) => check_value(label, &want, got),
        (Err(want), Err(got)) => {
            assert_eq!(
                (want.class_name.as_str(), want.message.as_str()),
                (got.class_name(), got.message()),
                "{label}: error class/message"
            );
        }
        (Ok(want), Err(got)) => {
            panic!("{label}: rust raised {}: {} but python returned {want:?}", got.class_name(), got.message())
        }
        (Err(want), Ok(got)) => {
            panic!(
                "{label}: python raised {}: {} but rust returned {got:?}",
                want.class_name, want.message
            )
        }
    }
}

fn pv_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(
            n.0.to_string()
                .parse::<i64>()
                .expect("status ints fit in i64"),
        ),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pv_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pv_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pv_to_value(k), pv_to_value(v)))
                .collect(),
        ),
        PyValue::Host(_) => panic!("host value in rotary_delta result"),
    }
}


struct Harness {
    kin: Arc<RotaryDeltaKinematics>,
    factory: Arc<FakeFactory>,
}

fn make_factory() -> Arc<FakeFactory> {
    Arc::new(FakeFactory {
        stepdists: IndexMap::from([
            ("stepper_a".to_string(), STEPDISTS[0]),
            ("stepper_b".to_string(), STEPDISTS[1]),
            ("stepper_c".to_string(), STEPDISTS[2]),
        ]),
        fakes: std::sync::Mutex::new(Vec::new()),
        calls: std::sync::Mutex::new(Vec::new()),
    })
}

fn harness() -> Harness {
    let factory = make_factory();
    let toolhead = Arc::new(FakeToolhead {
        trapq: Arc::new(FakeTrapq),
        factory: factory.clone(),
    });
    let toolhead_trait: klippy_api::toolhead::ToolHead = toolhead.clone();
    let kin = Arc::new(
        RotaryDeltaKinematics::new(&toolhead_trait, &fake_config()).expect("rust kinematics init"),
    );
    Harness { kin, factory }
}

fn float_tuple(v: &[f64]) -> Value {
    Value::Tuple(v.iter().map(|x| Value::float(*x)).collect())
}

fn float_list(v: &[f64]) -> Value {
    Value::List(v.iter().map(|x| Value::float(*x)).collect())
}

/// `tuple(kin.axes_min)` on the reference: `toolhead.Coord` pads the
/// 3-tuple with the int `0` (`gcode.py:15-16`), so the encoding is
/// three floats followed by an int.
fn axis_value(coord: &[f64]) -> Value {
    Value::Tuple(vec![
        Value::float(coord[0]),
        Value::float(coord[1]),
        Value::float(coord[2]),
        Value::Int(0),
    ])
}

// ---------------------------------------------------------------------------
// Calibration tests.
// ---------------------------------------------------------------------------

#[test]
fn calibration_math_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "cal = rotary_delta.RotaryDeltaCalibration(33.9, 412.9, [30., 150., 270.], \
         [170., 171., 169.], [320., 321., 319.], [252., 251., 253.], [0.01, 0.011, 0.0125])",
    )
    .expect("py calibration");
    let cal = Arc::new(RotaryDeltaCalibration::new(
        SHOULDER_RADIUS,
        SHOULDER_HEIGHT,
        [30.0, 150.0, 270.0],
        [170.0, 171.0, 169.0],
        [320.0, 321.0, 319.0],
        [252.0, 251.0, 253.0],
        STEPDISTS,
    ));

    check(
        "actuator_to_cartesian at endstops",
        py.eval("cal.actuator_to_cartesian([252., 251., 253.])"),
        float_list(
            &cal.actuator_to_cartesian(&[252.0, 251.0, 253.0])
                .expect("actuator"),
        ),
    );
    check(
        "actuator_to_cartesian generic offsets",
        py.eval("cal.actuator_to_cartesian([250.3, 248.7, 254.1])"),
        float_list(
            &cal.actuator_to_cartesian(&[250.3, 248.7, 254.1])
                .expect("actuator"),
        ),
    );
    check(
        "actuator_to_cartesian negative offsets",
        py.eval("cal.actuator_to_cartesian([-1.5, 0.25, 100.75])"),
        float_list(
            &cal.actuator_to_cartesian(&[-1.5, 0.25, 100.75])
                .expect("actuator"),
        ),
    );
    check(
        "stable roundtrip position",
        py.eval("cal.get_position_from_stable([10., -20., 30.])"),
        float_list(
            &cal.get_position_from_stable(&[10.0, -20.0, 30.0])
                .expect("get_position_from_stable"),
        ),
    );
    check(
        "stable roundtrip coord",
        py.eval("cal.calc_stable_position([5., -5., 100.])"),
        float_list(
            &cal.calc_stable_position(&[5.0, -5.0, 100.0])
                .expect("calc_stable_position"),
        ),
    );
    check(
        "stable coord at origin",
        py.eval("cal.calc_stable_position([0., 0., 252.])"),
        float_list(
            &cal.calc_stable_position(&[0.0, 0.0, 252.0])
                .expect("calc_stable_position"),
        ),
    );
    check(
        "coordinate_descent_params basic",
        py.eval("cal.coordinate_descent_params(False)"),
        pv_to_value(&cal.coordinate_descent_params(false)),
    );
    check(
        "coordinate_descent_params extended",
        py.eval("cal.coordinate_descent_params(True)"),
        pv_to_value(&cal.coordinate_descent_params(true)),
    );
}

#[test]
fn calibration_new_from_params_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "cal = rotary_delta.RotaryDeltaCalibration(33.9, 412.9, [30., 150., 270.], \
         [170., 170., 170.], [320., 320., 320.], [252., 252., 252.], [0.01, 0.01, 0.01])",
    )
    .expect("py calibration");
    let cal = Arc::new(RotaryDeltaCalibration::new(
        SHOULDER_RADIUS,
        SHOULDER_HEIGHT,
        [30.0, 150.0, 270.0],
        [UPPER_ARM, UPPER_ARM, UPPER_ARM],
        [LOWER_ARM, LOWER_ARM, LOWER_ARM],
        [ENDSTOP_A, ENDSTOP_A, ENDSTOP_A],
        [0.01, 0.01, 0.01],
    ));
    py.exec(
        "cal2 = cal.new_calibration({'shoulder_radius': 34.5, 'shoulder_height': 400., \
         'angle_a': 31., 'angle_b': 149., 'angle_c': 271., \
         'upper_arm_a': 171., 'upper_arm_b': 172., 'upper_arm_c': 173., \
         'lower_arm_a': 319., 'lower_arm_b': 318., 'lower_arm_c': 317., \
         'endstop_a': 250., 'endstop_b': 251., 'endstop_c': 253., \
         'stepdist_a': 0.02, 'stepdist_b': 0.02, 'stepdist_c': 0.02})",
    )
    .expect("py new_calibration");
    let params = PyValue::Dict(IndexMap::from([
        (PyValue::Str("shoulder_radius".into()), PyValue::Float(34.5)),
        (PyValue::Str("shoulder_height".into()), PyValue::Float(400.0)),
        (PyValue::Str("angle_a".into()), PyValue::Float(31.0)),
        (PyValue::Str("angle_b".into()), PyValue::Float(149.0)),
        (PyValue::Str("angle_c".into()), PyValue::Float(271.0)),
        (
            PyValue::Str("upper_arm_a".into()),
            PyValue::Float(171.0),
        ),
        (
            PyValue::Str("upper_arm_b".into()),
            PyValue::Float(172.0),
        ),
        (
            PyValue::Str("upper_arm_c".into()),
            PyValue::Float(173.0),
        ),
        (
            PyValue::Str("lower_arm_a".into()),
            PyValue::Float(319.0),
        ),
        (
            PyValue::Str("lower_arm_b".into()),
            PyValue::Float(318.0),
        ),
        (
            PyValue::Str("lower_arm_c".into()),
            PyValue::Float(317.0),
        ),
        (PyValue::Str("endstop_a".into()), PyValue::Float(250.0)),
        (PyValue::Str("endstop_b".into()), PyValue::Float(251.0)),
        (PyValue::Str("endstop_c".into()), PyValue::Float(253.0)),
        (PyValue::Str("stepdist_a".into()), PyValue::Float(0.02)),
        (PyValue::Str("stepdist_b".into()), PyValue::Float(0.02)),
        (PyValue::Str("stepdist_c".into()), PyValue::Float(0.02)),
    ]));
    let cal2 = cal.new_calibration(&params).expect("rust new_calibration");
    check(
        "new_calibration params",
        py.eval("cal2.coordinate_descent_params(False)"),
        pv_to_value(&cal2.coordinate_descent_params(false)),
    );
    check(
        "new_calibration stable",
        py.eval("cal2.get_position_from_stable([10., -20., 30.])"),
        float_list(
            &cal2.get_position_from_stable(&[10.0, -20.0, 30.0])
                .expect("stable"),
        ),
    );
    // Missing key raises KeyError on both sides.
    let bad = PyValue::Dict(IndexMap::from([(PyValue::Str("shoulder_radius".into()), PyValue::Float(1.0))]));
    check_err(
        "new_calibration missing key",
        py.eval("cal.new_calibration({'shoulder_radius': 1.0})"),
        cal.new_calibration(&bad).map(|_| Value::Null),
    );
}


#[test]
fn calibration_domain_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    // Angles all zero: the three sphere centers line up so that
    // `j == 0` in trilateration and Python raises ZeroDivisionError.
    py.exec(
        "deg = rotary_delta.RotaryDeltaCalibration(33.9, 412.9, [0., 0., 0.], \
         [170., 170., 170.], [320., 320., 320.], [252., 252., 252.], [0.01, 0.01, 0.01])",
    )
    .expect("py degenerate calibration");
    let deg = Arc::new(RotaryDeltaCalibration::new(
        SHOULDER_RADIUS,
        SHOULDER_HEIGHT,
        [0.0, 0.0, 0.0],
        [UPPER_ARM, UPPER_ARM, UPPER_ARM],
        [LOWER_ARM, LOWER_ARM, LOWER_ARM],
        [ENDSTOP_A, ENDSTOP_A, ENDSTOP_A],
        [0.01, 0.01, 0.01],
    ));
    check_err(
        "degenerate actuator raises",
        py.eval("deg.actuator_to_cartesian([252., 252., 252.])"),
        deg.get_position_from_stable(&[0.0, 0.0, 0.0])
            .map(|v| float_list(&v)),
    );
    // No failure: calc_stable_position with a zero stepdist raises
    // ZeroDivisionError in the final division on both sides.
    py.exec(
        "zero = rotary_delta.RotaryDeltaCalibration(33.9, 412.9, [30., 150., 270.], \
         [170., 170., 170.], [320., 320., 320.], [252., 252., 252.], [0.0, 0.01, 0.01])",
    )
    .expect("py zero stepdist calibration");
    let zero = Arc::new(RotaryDeltaCalibration::new(
        SHOULDER_RADIUS,
        SHOULDER_HEIGHT,
        [30.0, 150.0, 270.0],
        [UPPER_ARM, UPPER_ARM, UPPER_ARM],
        [LOWER_ARM, LOWER_ARM, LOWER_ARM],
        [ENDSTOP_A, ENDSTOP_A, ENDSTOP_A],
        [0.0, 0.01, 0.01],
    ));
    check_err(
        "zero stepdist calc_stable raises",
        py.eval("zero.calc_stable_position([0., 0., 100.])"),
        zero.calc_stable_position(&[0.0, 0.0, 100.0])
            .map(|v| float_list(&v)),
    );
}

// ---------------------------------------------------------------------------
// Kinematics tests.
// ---------------------------------------------------------------------------

fn kin_state(mut py: pyoracle::Oracle) -> (pyoracle::Oracle, Harness) {
    py.exec("make_py_kin()").expect("py kinematics");
    let harness = harness();
    (py, harness)
}

#[test]
fn kinematics_init_matches_python() {
    let Some(py) = oracle() else { return };
    // Install the log sink before the harness logs the height line.
    let _ = log_lines();
    let (mut py, h) = kin_state(py);

    check(
        "kinematics state",
        py.eval(
            "(kin.max_z_velocity, kin.max_z, kin.min_z, kin.limit_z, kin.max_xy2, \
             kin.limit_xy2, kin.need_home, kin.home_position, tuple(kin.axes_min), \
             tuple(kin.axes_max))",
        ),
        Value::Tuple(vec![
            Value::float(h.kin.max_z_velocity()),
            Value::float(h.kin.max_z()),
            Value::float(h.kin.min_z()),
            Value::float(h.kin.limit_z()),
            Value::float(h.kin.max_xy2()),
            Value::float(h.kin.limit_xy2()),
            Value::Bool(h.kin.need_home()),
            float_tuple(&h.kin.home_position()),
            axis_value(&h.kin.axes_min().0),
            axis_value(&h.kin.axes_max().0),
        ]),
    );

    // Rail factory calls: names, need_position_minmax, default
    // endstops and radians flag.
    let rail_calls: Vec<Value> = h
        .factory
        .calls
        .lock()
        .expect("calls")
        .iter()
        .map(|(name, npm, default, radians)| {
            Value::Tuple(vec![
                Value::str(name),
                Value::Bool(*npm),
                match default {
                    Some(v) => Value::float(*v),
                    None => Value::Null,
                },
                Value::Bool(*radians),
            ])
        })
        .collect();
    check(
        "rail factory calls",
        py.eval("RAIL_CALLS"),
        Value::List(rail_calls),
    );

    // IterSolve setup per rail (alloc + 5 float params).
    let rail_fakes = h.factory.fakes.lock().expect("fakes").clone();
    let itersolve: Vec<Value> = rail_fakes
        .iter()
        .map(|rail| {
            let (alloc, params) = rail
                .itersolve
                .lock()
                .expect("itersolve")
                .clone()
                .expect("itersolve");
            Value::Tuple(vec![
                Value::str(&alloc),
                float_tuple(&params),
            ])
        })
        .collect();
    check(
        "itersolve setup",
        py.eval("[(r.itersolve[0], r.itersolve[1]) for r in RAILS.values()]"),
        Value::List(itersolve),
    );

    // Each stepper got the toolhead trapq and the final
    // set_position([0, 0, 0]) from __init__.
    let stepper_calls: Vec<Value> = rail_fakes
        .iter()
        .map(|rail| {
            let stepper = rail.stepper.clone();
            let trapqs = stepper.trapqs.lock().expect("trapqs");
            let positions = stepper.positions.lock().expect("positions");
            Value::Tuple(vec![
                Value::Bool(*trapqs.first().expect("trapq set")),
                float_tuple(positions.first().expect("set_position called")),
            ])
        })
        .collect();
    check(
        "stepper trapq and position calls",
        py.eval(
            "([(s.trapqs[0] is TRAPQ, tuple(s.positions[0])) for s in \
             [r.stepper for r in RAILS.values()]])",
        ),
        Value::List(stepper_calls),
    );

    // The build-height log line emitted during __init__ on this thread.
    // Only distinct lines are compared; the line is identical for every
    // init because all tests use the same geometry.
    let unique: Vec<String> = {
        let logs = log_lines();
        let mut seen: Vec<String> = Vec::new();
        for line in logs.iter() {
            if !seen.contains(line) {
                seen.push(line.clone());
            }
        }
        seen
    };
    check(
        "delta height log line",
        py.eval("list(log_lines)"),
        Value::List(unique.iter().map(|m| Value::str(m)).collect()),
    );

    // get_steppers: one stepper per rail with the step distances.
    let steppers = h.kin.get_steppers();
    check(
        "get_steppers step distances",
        py.eval("[s.get_step_dist() for s in kin.get_steppers()]"),
        Value::List(
            steppers
                .iter()
                .map(|s| Value::float(s.get_step_dist()))
                .collect(),
        ),
    );

    // get_calibration params match the constructed configuration.
    let cal = h.kin.get_calibration().expect("calibration");
    check(
        "calibration params",
        py.eval("kin.get_calibration().coordinate_descent_params(False)"),
        pv_to_value(&cal.coordinate_descent_params(false)),
    );
    // Direct calc_position equivalence with the recorded spos.
    let mut positions = IndexMap::new();
    positions.insert("stepper_a".to_string(), 250.0);
    positions.insert("stepper_b".to_string(), 251.0);
    positions.insert("stepper_c".to_string(), 253.0);
    let spos = h
        .kin
        .calc_position(&positions)
        .expect("calc_position");
    let spos: Vec<f64> = spos.into_iter().map(Option::unwrap).collect();
    check(
        "calc_position direct",
        py.eval(
            "kin.calc_position({'stepper_a': 250.0, 'stepper_b': 251.0, 'stepper_c': 253.0})",
        ),
        float_list(&spos),
    );
    check_err(
        "calc_position missing key",
        py.eval("kin.calc_position({'stepper_a': 250.0, 'stepper_c': 253.0})"),
        {
            let mut missing = IndexMap::new();
            missing.insert("stepper_a".to_string(), 250.0);
            missing.insert("stepper_c".to_string(), 253.0);
            h.kin
                .calc_position(&missing)
                .map(|v| float_list(&v.iter().map(|o| o.unwrap()).collect::<Vec<_>>()))
        },
    );
}

#[test]
fn kinematics_set_position_and_status_match_python() {
    let Some(py) = oracle() else { return };
    let (mut py, h) = kin_state(py);

    check(
        "status before set_position",
        py.eval("kin.get_status(0)"),
        pv_to_value(&h.kin.get_status(0.0)),
    );

    h.kin
        .set_position(vec![1.0, 2.0, 3.0], "xyz")
        .expect("set_position");
    py.exec("kin.set_position([1.0, 2.0, 3.0], 'xyz')")
        .expect("py set_position");
    check(
        "status after set_position xyz",
        py.eval("kin.get_status(0)"),
        pv_to_value(&h.kin.get_status(0.0)),
    );
    check(
        "limit_xy2 after set_position",
        py.eval("kin.limit_xy2"),
        Value::float(h.kin.limit_xy2()),
    );

    // A partial homing axes string does not clear need_home.
    h.kin.set_position(vec![0.0, 0.0, 0.0], "xy").expect("set_position xy");
    py.exec("kin.set_position([0.0, 0.0, 0.0], 'xy')")
        .expect("py set_position xy");
    check(
        "status after set_position xy",
        py.eval("kin.get_status(0)"),
        pv_to_value(&h.kin.get_status(0.0)),
    );

    // clear_homing_state("") is a no-op; ("xyz") re-arms homing.
    h.kin.clear_homing_state("");
    py.exec("kin.clear_homing_state('')").expect("py clear empty");
    check(
        "status after clear empty",
        py.eval("kin.get_status(0)"),
        pv_to_value(&h.kin.get_status(0.0)),
    );
    h.kin.clear_homing_state("xyz");
    py.exec("kin.clear_homing_state('xyz')").expect("py clear xyz");
    check(
        "status after clear xyz",
        py.eval("kin.get_status(0)"),
        pv_to_value(&h.kin.get_status(0.0)),
    );
}


fn rust_check(h: &Harness, end: [f64; 4], axes_d_z: f64, accel: f64) -> Value {
    let limit_calls = Arc::new(std::sync::Mutex::new(Vec::new()));
    let mv = FakeMove {
        end_pos: Coord(vec![end[0], end[1], end[2], 0.0]),
        axes_d: Coord(vec![0.0, 0.0, axes_d_z, 0.0]),
        accel,
        limit_calls: limit_calls.clone(),
    };
    match h.kin.check_move(&mv) {
        Ok(()) => Value::Tuple(vec![
            Value::str("ok"),
            Value::float(h.kin.limit_xy2()),
            Value::Bool(h.kin.need_home()),
            Value::List(
                limit_calls
                    .lock()
                    .expect("limit")
                    .iter()
                    .map(|(s, a)| Value::Tuple(vec![Value::float(*s), Value::float(*a)]))
                    .collect(),
            ),
        ]),
        Err(e) => Value::Tuple(vec![
            Value::str("err"),
            Value::str(e.class_name()),
            Value::str(e.message()),
            Value::float(h.kin.limit_xy2()),
            Value::Bool(h.kin.need_home()),
            Value::List(
                limit_calls
                    .lock()
                    .expect("limit")
                    .iter()
                    .map(|(s, a)| Value::Tuple(vec![Value::float(*s), Value::float(*a)]))
                    .collect(),
            ),
        ]),
    }
}

fn check_case(
    py: &mut pyoracle::Oracle,
    h: &Harness,
    label: &str,
    end: [f64; 4],
    axes_d_z: f64,
    accel: f64,
) {
    let py_result = py.eval(&format!(
        "run_check(FakeMove({}, [0., 0., {axes_d_z:?}], {accel:?}))",
        float_tuple_py(&end),
    ));
    check(label, py_result, rust_check(h, end, axes_d_z, accel));
}

fn float_tuple_py(v: &[f64]) -> String {
    let parts: Vec<String> = v.iter().map(|x| format!("{x:?}")).collect();
    format!("({},)", parts.join(", "))
}

#[test]
fn kinematics_check_move_matches_python() {
    let Some(py) = oracle() else { return };
    let (mut py, h) = kin_state(py);

    // Fresh kinematics needs homing first.
    check_case(&mut py, &h, "need home error", [5.0, 5.0, 10.0, 0.0], 0.0, 300.0);

    // Home, then a more-XY move updates limit_xy2.
    h.kin
        .set_position(vec![0.0, 0.0, 0.0], "xyz")
        .expect("set_position");
    py.exec("kin.set_position([0.0, 0.0, 0.0], 'xyz')")
        .expect("py set_position");
    let mid = [5.0f64, 5.0, 10.0, 0.0];
    check_case(&mut py, &h, "xy move within limits", mid, 0.0, 300.0);
    // Same end again with no Z motion: fast path, limit kept.
    check_case(&mut py, &h, "xy move fast path", mid, 0.0, 300.0);

    // A Z move limits the speed with max_z_velocity.
    check_case(&mut py, &h, "z move limits speed", [5.0, 5.0, 20.0, 0.0], 3.0, 250.0);

    // Out of range in XY.
    check_case(
        &mut py,
        &h,
        "out of range xy",
        [2000.0, 0.0, 10.0, 0.0],
        0.0,
        300.0,
    );
    // Out of range above max_z.
    check_case(
        &mut py,
        &h,
        "out of range above max_z",
        [1.0, 1.0, 300.0, 0.0],
        0.0,
        300.0,
    );
    // Below minimum z.
    check_case(
        &mut py,
        &h,
        "below minimum z",
        [0.0, 0.0, -1.0, 0.0],
        0.0,
        300.0,
    );
    // Homing-position exception: same XY as the home position and
    // Z within [min_z, home_z] is accepted with limit_xy2 = -1.
    let home = h.kin.home_position();
    check_case(
        &mut py,
        &h,
        "homing position exception",
        [home[0], home[1], 0.5, 0.0],
        0.0,
        300.0,
    );
    // Above limit_z the XY radius shrinks by (max_z - end_z).
    let high_z = h.kin.limit_z() + 2.0;
    check_case(&mut py, &h, "above limit_z", [10.0, 0.0, high_z, 0.0], 0.0, 300.0);
}

#[test]
fn kinematics_home_matches_python() {
    let Some(py) = oracle() else { return };
    let (mut py, h) = kin_state(py);

    let py_home = py.exec(
        "hs = FakeHomingState()\nkin.home(hs)\n(hs.axes, hs.calls)",
    );
    let _ = py_home.expect("py home");
    let rust_state = Arc::new(FakeHomingState {
        axes: std::sync::Mutex::new(Vec::new()),
        calls: std::sync::Mutex::new(Vec::new()),
    });
    h.kin
        .home(rust_state.clone())
        .expect("rust home");
    check(
        "home axes and rails",
        py.eval("(hs.axes, hs.calls)"),
        Value::Tuple(vec![
            Value::List(
                rust_state
                    .axes
                    .lock()
                    .expect("axes")
                    .iter()
                    .map(|a| Value::Int(*a))
                    .collect(),
            ),
            Value::List(
                rust_state
                    .calls
                    .lock()
                    .expect("calls")
                    .iter()
                    .map(|(n, force, movepos)| {
                        Value::Tuple(vec![
                            Value::Int(*n as i64),
                            Value::List(
                                force
                                    .iter()
                                    .map(|v| match v {
                                        Some(f) => Value::float(*f),
                                        None => Value::Null,
                                    })
                                    .collect(),
                            ),
                            Value::List(
                                movepos
                                    .iter()
                                    .map(|v| match v {
                                        Some(f) => Value::float(*f),
                                        None => Value::Null,
                                    })
                                    .collect(),
                            ),
                        ])
                    })
                    .collect(),
            ),
        ]),
    );
}

#[test]
fn calibration_save_state_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "cal = rotary_delta.RotaryDeltaCalibration(33.9, 412.9, [30., 150., 270.], \
         [170., 170., 170.], [320., 320., 320.], [252., 251., 253.], [0.01, 0.011, 0.0125])",
    )
    .expect("py calibration");
    let cal = Arc::new(RotaryDeltaCalibration::new(
        SHOULDER_RADIUS,
        SHOULDER_HEIGHT,
        [30.0, 150.0, 270.0],
        [UPPER_ARM, UPPER_ARM, UPPER_ARM],
        [LOWER_ARM, LOWER_ARM, LOWER_ARM],
        [252.0, 251.0, 253.0],
        STEPDISTS,
    ));

    py.exec(
        "pf = FakePrinter()\npcf = FakePrinterConfig(pf)\ncal.save_state(pcf)",
    )
    .expect("py save_state");
    let infos: Infos = Arc::new(std::sync::Mutex::new(Vec::new()));
    let printer: Printer = Arc::new(FakePrinter {
        gcode: Arc::new(FakeGCode {
            infos: infos.clone(),
        }),
    });
    let set_calls = Arc::new(std::sync::Mutex::new(Vec::new()));
    let pcf: PrinterConfig = Arc::new(FakePrinterConfig {
        printer,
        set_calls: set_calls.clone(),
    });
    cal.save_state(&pcf).expect("rust save_state");

    check(
        "save_state set calls",
        py.eval("pcf.set_calls"),
        Value::List(
            set_calls
                .lock()
                .expect("set")
                .iter()
                .map(|(s, o, v)| {
                    Value::Tuple(vec![Value::str(s), Value::str(o), Value::str(v)])
                })
                .collect(),
        ),
    );
    check(
        "save_state respond_info",
        py.eval("pf.gcode.infos"),
        Value::List(
            infos
                .lock()
                .expect("infos")
                .iter()
                .map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)]))
                .collect(),
        ),
    );
}

#[test]
fn config_errors_match_python() {
    let Some(mut py) = oracle() else { return };

    // Missing a required option.
    // Construction reads the stepper sections before the
    // kinematics options, so the sections must resolve too.
    let sections = |opt_a: f64| -> Arc<IndexMap<String, Arc<FakeConfig>>> {
        Arc::new(IndexMap::from([
            (
                "stepper_a".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_a".to_string(),
                    floats: IndexMap::from([("position_endstop".to_string(), opt_a)]),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
            (
                "stepper_b".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_b".to_string(),
                    floats: IndexMap::new(),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
            (
                "stepper_c".to_string(),
                Arc::new(FakeConfig {
                    name: "stepper_c".to_string(),
                    floats: IndexMap::new(),
                    sections: Arc::new(IndexMap::new()),
                }),
            ),
        ]))
    };
    let make = |opts: IndexMap<String, f64>, opt_a: f64| -> Config {
        Arc::new(FakeConfig {
            name: "rotary_delta".to_string(),
            floats: opts,
            sections: sections(opt_a),
        })
    };
    let py_missing = py.eval(
        "(lambda: (lambda c: rotary_delta.RotaryDeltaKinematics(FakeToolhead(), c))(\
         FakeConfig('rotary_delta', {'shoulder_radius': 33.9}, \
         {'stepper_a': FakeConfig('stepper_a', {'position_endstop': 252.0}), \
          'stepper_b': FakeConfig('stepper_b', {}), \
          'stepper_c': FakeConfig('stepper_c', {})})))()",
    );
    let rust_missing = RotaryDeltaKinematics::new(
        &harness_toolhead(),
        &make(
            IndexMap::from([("shoulder_radius".to_string(), SHOULDER_RADIUS)]),
            ENDSTOP_A,
        ),
    )
    .map(|_| Value::Null);
    check_err("missing required option", py_missing, rust_missing);

    // above=0 violation.
    let py_above = py.eval(
        "(lambda: (lambda c: rotary_delta.RotaryDeltaKinematics(FakeToolhead(), c))(\
         FakeConfig('rotary_delta', {'shoulder_radius': 0.0, 'shoulder_height': 412.9}, \
         {'stepper_a': FakeConfig('stepper_a', {'position_endstop': 252.0}), \
          'stepper_b': FakeConfig('stepper_b', {}), \
          'stepper_c': FakeConfig('stepper_c', {})})))()",
    );
    let rust_above = RotaryDeltaKinematics::new(
        &harness_toolhead(),
        &make(
            IndexMap::from([
                ("shoulder_radius".to_string(), 0.0),
                ("shoulder_height".to_string(), SHOULDER_HEIGHT),
            ]),
            ENDSTOP_A,
        ),
    )
    .map(|_| Value::Null);
    check_err("above violation", py_above, rust_above);
}

fn harness_toolhead() -> klippy_api::toolhead::ToolHead {
    Arc::new(FakeToolhead {
        trapq: Arc::new(FakeTrapq),
        factory: make_factory(),
    })
}

#[test]
fn load_kinematics_returns_kinematics() {
    let toolhead = harness_toolhead();
    let kin: Kinematics = load_kinematics(&toolhead, &fake_config()).expect("load_kinematics");
    let steppers = kin.get_steppers();
    assert_eq!(steppers.len(), 3);
}
