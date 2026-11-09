//! Parity tests for `kinematics::corexz` (port of `klippy/kinematics/corexz.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`). A `CoreXZKinematics` is built on both
//! sides from minimal fakes for the stepper module, toolhead, config, rails,
//! endstops, steppers, homing state and moves, and the same calls are driven
//! with identical inputs. Both sides append every call the kinematics makes on
//! a fake (config reads with their arguments, rail lookups, endstop wiring,
//! itersolve setup, trapq propagation, stepper positions, homing calls,
//! `limit_speed`) to an ordered event log, and the logs are compared with the
//! returned values (floats by bits), raised errors (class and message) and
//! `get_status` dicts through `recording::assert_equal`.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit.

use std::any::Any;
use std::sync::Arc;

use kinematics::corexz::load_kinematics;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Endstop, EndstopApi, Mcu};
use klippy_api::printer::Printer;
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, ExtraAxis, HomingStateApi, HomingTargetApi, Kinematics, LookaheadCallback, MoveApi,
    PrinterExtruder, ToolHead, ToolHeadApi,
};
use kos::sync::Mutex;
use pycompat::{PctValue, PyValue};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, collections
import gcode
EVENTS = []
SPECS = {}
TRAPQ = ('trapq', 1)
def take():
    out = list(EVENTS)
    del EVENTS[:]
    return out
class FakeStepper:
    def __init__(self, name):
        self._name = name
    def get_name(self, short=False):
        assert short == False, short
        return self._name
    def set_trapq(self, tq):
        EVENTS.append(('stepper_set_trapq', self._name, tq[1]))
    def set_position(self, coord):
        EVENTS.append(('stepper_set_position', self._name, coord[0], coord[1], coord[2]))
class FakeEndstop:
    def __init__(self, owner):
        self._owner = owner
    def add_stepper(self, s):
        EVENTS.append(('endstop_add_stepper', self._owner, s.get_name()))
HomingInfo = collections.namedtuple('homing_info', ['speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir', 'second_homing_speed'])
class FakeRail:
    def __init__(self, name, spec):
        rng, homing, steppers, n_endstops = spec
        self._name = name
        self._range = rng
        self._homing = HomingInfo(*homing)
        self._steppers = [FakeStepper(n) for n in steppers]
        self._endstops = [(FakeEndstop('%s#%d' % (name, i)), name) for i in range(n_endstops)]
    def get_name(self, short=False):
        assert short == False, short
        return self._name
    def get_range(self):
        return self._range
    def get_homing_info(self):
        return self._homing
    def get_steppers(self):
        return list(self._steppers)
    def get_endstops(self):
        EVENTS.append(('get_endstops', self._name))
        return list(self._endstops)
    def setup_itersolve(self, alloc_func, *params):
        EVENTS.append(('setup_itersolve', self._name, alloc_func, params))
    def set_position(self, coord):
        EVENTS.append(('rail_set_position', self._name, coord[0], coord[1], coord[2]))
        for s in self._steppers:
            s.set_position(coord)
def LookupMultiRail(config, **kw):
    assert not kw, kw
    EVENTS.append(('LookupMultiRail', config.get_name()))
    return FakeRail(config.get_name(), SPECS[config.get_name()])
_fake_stepper = types.ModuleType('stepper')
_fake_stepper.LookupMultiRail = LookupMultiRail
sys.modules['stepper'] = _fake_stepper
import kinematics.corexz as corexz
class FakeConfig:
    def __init__(self, name, floats):
        self._name = name
        self._floats = dict(floats)
    def get_name(self):
        return self._name
    def getsection(self, sec):
        EVENTS.append(('getsection', sec))
        return FakeConfig(sec, {})
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
        EVENTS.append(('getfloat', self._name, option, default, minval, maxval, above, below))
        return self._floats.get(option, default)
class FakeToolHead:
    Coord = gcode.Coord
    def __init__(self, max_vel, max_accel):
        self._mv = max_vel
        self._ma = max_accel
    def get_trapq(self):
        EVENTS.append(('get_trapq',))
        return TRAPQ
    def get_max_velocity(self):
        EVENTS.append(('get_max_velocity',))
        return (self._mv, self._ma)
class FakeHoming:
    def __init__(self, axes):
        self._axes = list(axes)
    def get_axes(self):
        EVENTS.append(('get_axes',))
        return list(self._axes)
    def home_rails(self, rails, forcepos, movepos):
        EVENTS.append(('home_rails', [r.get_name() for r in rails], list(forcepos), list(movepos)))
class FakeMove:
    def __init__(self, start_pos, end_pos, move_d):
        self.end_pos = tuple(end_pos)
        self.axes_d = [e - s for s, e in zip(start_pos, end_pos)]
        self.move_d = move_d
    def limit_speed(self, speed, accel):
        EVENTS.append(('limit_speed', speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return gcode.CommandError(m)
def build(max_vel, max_accel, specs, floats):
    del EVENTS[:]
    SPECS.clear()
    SPECS.update(specs)
    return corexz.CoreXZKinematics(FakeToolHead(max_vel, max_accel), FakeConfig('printer', floats))
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("corexz fakes");
            Some(py)
        }
        None => {
            println!("skipping corexz oracle tests: KLIPPY_PYTHON is not set");
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes. Every call the kinematics makes lands in the shared log in
// the same tuple shape the Python fakes use.
// ---------------------------------------------------------------------------

type Log = Arc<Mutex<Vec<Value>>>;

fn s(x: &str) -> Value {
    Value::Str(x.to_string())
}

fn f(x: f64) -> Value {
    Value::float(x)
}

fn fo(x: Option<f64>) -> Value {
    match x {
        Some(v) => Value::float(v),
        None => Value::Null,
    }
}

fn push(log: &Log, items: Vec<Value>) {
    log.lock().push(Value::Tuple(items));
}

fn take(log: &Log) -> Value {
    Value::List(log.lock().drain(..).collect())
}

struct DummyTrapq {
    id: i64,
}

impl TrapqApi for DummyTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeStepper {
    name: String,
    log: Log,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu unused")
    }
    fn get_name(&self, short: Option<bool>) -> String {
        assert!(!short.unwrap_or(false));
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid unused")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians unused")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration unused")
    }
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {
        panic!("setup_default_pulse_duration unused")
    }
    fn setup_itersolve(&self, _f: &str, _p: &[PyValue]) {
        panic!("setup_itersolve unused")
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist unused")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance unused")
    }
    fn set_rotation_distance(&self, _r: f64) {
        panic!("set_rotation_distance unused")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted unused")
    }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> {
        panic!("set_dir_inverted unused")
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord unused")
    }
    fn set_position(&self, c: [f64; 3]) {
        push(
            &self.log,
            vec![s("stepper_set_position"), s(&self.name), f(c[0]), f(c[1]), f(c[2])],
        );
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position unused")
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        panic!("get_mcu_position unused")
    }
    fn get_past_mcu_position(&self, _t: f64) -> i64 {
        panic!("get_past_mcu_position unused")
    }
    fn mcu_to_commanded_position(&self, _m: i64) -> f64 {
        panic!("mcu_to_commanded_position unused")
    }
    fn dump_steps(&self, _c: i64, _s: i64, _e: i64) -> (Vec<PullHistoryStep>, i64) {
        panic!("dump_steps unused")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics unused")
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics unused")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end unused")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq unused")
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        let id = match &tq {
            Some(t) => Value::Int(t.as_any().downcast_ref::<DummyTrapq>().expect("DummyTrapq").id),
            None => Value::Null,
        };
        push(&self.log, vec![s("stepper_set_trapq"), s(&self.name), id]);
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        panic!("add_active_callback unused")
    }
    fn is_active_axis(&self, _a: char) -> bool {
        panic!("is_active_axis unused")
    }
}

struct FakeEndstop {
    owner: String,
    log: Log,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> klippy_api::KResult<Mcu> {
        panic!("get_mcu unused")
    }
    fn add_stepper(&self, stepper: Stepper) {
        push(
            &self.log,
            vec![s("endstop_add_stepper"), s(&self.owner), s(&stepper.get_name(None))],
        );
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        panic!("get_steppers unused")
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> klippy_api::KResult<reactor::Completion<bool>> {
        panic!("home_start unused")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        panic!("home_wait unused")
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("query_endstop unused")
    }
}

struct FakeRail {
    name: String,
    range: (f64, f64),
    homing: HomingInfo,
    steppers: Vec<Stepper>,
    endstops: Vec<(Endstop, String)>,
    log: Log,
}

impl RailApi for FakeRail {
    fn get_name(&self, short: Option<bool>) -> String {
        assert!(!short.unwrap_or(false));
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        self.range
    }
    fn get_homing_info(&self) -> HomingInfo {
        self.homing
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn get_endstops(&self) -> Vec<(Endstop, String)> {
        push(&self.log, vec![s("get_endstops"), s(&self.name)]);
        self.endstops.clone()
    }
    fn lookup_endstop(&self, _pin: Option<&str>, _name: &str) -> KResult<Endstop> {
        panic!("lookup_endstop unused")
    }
    fn add_stepper(&self, _stepper: Stepper, _pin: Option<&str>, _name: Option<&str>) -> KResult<()> {
        panic!("add_stepper unused")
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        panic!("add_stepper_from_config unused")
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        push(
            &self.log,
            vec![
                s("setup_itersolve"),
                s(&self.name),
                s(alloc_func),
                Value::Tuple(params.iter().map(pyvalue_to_value).collect()),
            ],
        );
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {
        panic!("set_trapq unused")
    }
    fn set_position(&self, c: [f64; 3]) {
        push(
            &self.log,
            vec![s("rail_set_position"), s(&self.name), f(c[0]), f(c[1]), f(c[2])],
        );
        for st in &self.steppers {
            st.set_position(c);
        }
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord unused")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position unused")
    }
}

#[derive(Clone)]
struct RailSpec {
    range: (f64, f64),
    endstop: f64,
    positive_dir: bool,
    steppers: Vec<&'static str>,
    endstops: usize,
}

struct FakeFactory {
    specs: Vec<(String, RailSpec)>,
    log: Log,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper unused")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail unused")
    }
    fn lookup_multi_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail> {
        assert_eq!(opts, RailOpts::default(), "LookupMultiRail takes no keyword arguments");
        let name = config.get_name();
        push(&self.log, vec![s("LookupMultiRail"), s(&name)]);
        let spec = self
            .specs
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, sp)| sp.clone())
            .unwrap_or_else(|| panic!("no rail spec for {name}"));
        let steppers: Vec<Stepper> = spec
            .steppers
            .iter()
            .map(|n| {
                let st: Stepper = Arc::new(FakeStepper { name: n.to_string(), log: self.log.clone() });
                st
            })
            .collect();
        let endstops: Vec<(Endstop, String)> = (0..spec.endstops)
            .map(|i| {
                let e: Endstop =
                    Arc::new(FakeEndstop { owner: format!("{name}#{i}"), log: self.log.clone() });
                (e, name.clone())
            })
            .collect();
        Ok(Arc::new(FakeRail {
            name,
            range: spec.range,
            homing: HomingInfo {
                speed: 5.0,
                position_endstop: spec.endstop,
                retract_speed: 2.0,
                retract_dist: 1.0,
                positive_dir: spec.positive_dir,
                second_homing_speed: 1.0,
            },
            steppers,
            endstops,
            log: self.log.clone(),
        }))
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail unused")
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance unused")
    }
}

struct FakeConfig {
    name: String,
    floats: Vec<(String, f64)>,
    log: Log,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        panic!("get_printer unused")
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get unused")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint unused")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default = opts.default.flatten();
        push(
            &self.log,
            vec![
                s("getfloat"),
                s(&self.name),
                s(option),
                fo(default),
                fo(opts.minval),
                fo(opts.maxval),
                fo(opts.above),
                fo(opts.below),
            ],
        );
        Ok(self.floats.iter().find(|(k, _)| k == option).map(|(_, v)| *v).or(default))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean unused")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice unused")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists unused")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist unused")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist unused")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist unused")
    }
    fn getsection(&self, section: &str) -> Config {
        push(&self.log, vec![s("getsection"), s(section)]);
        Arc::new(FakeConfig { name: section.to_string(), floats: Vec::new(), log: self.log.clone() })
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section unused")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections unused")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options unused")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate unused")
    }
}

struct FakeToolHead {
    max_velocity: f64,
    max_accel: f64,
    factory: StepperFactory,
    log: Log,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position unused")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position unused")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation unused")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        panic!("get_last_move_time unused")
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        panic!("dwell unused")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move unused")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move unused")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move unused")
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves unused")
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        panic!("get_status unused")
    }
    fn get_trapq(&self) -> Trapq {
        push(&self.log, vec![s("get_trapq")]);
        Arc::new(DummyTrapq { id: 1 })
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        push(&self.log, vec![s("get_max_velocity")]);
        (self.max_velocity, self.max_accel)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities unused")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback unused")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy unused")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed unused")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder unused")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder unused")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis unused")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis unused")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes unused")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats unused")
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

struct FakeHoming {
    axes: Vec<i64>,
    log: Log,
}

impl HomingStateApi for FakeHoming {
    fn set_axes(&self, _axes: Vec<i64>) {
        panic!("set_axes unused")
    }
    fn get_axes(&self) -> Vec<i64> {
        push(&self.log, vec![s("get_axes")]);
        self.axes.clone()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        panic!("get_trigger_position unused")
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {
        panic!("set_stepper_adjustment unused")
    }
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        panic!("set_homed_position unused")
    }
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        push(
            &self.log,
            vec![
                s("home_rails"),
                Value::List(rails.iter().map(|r| s(&r.get_name(None))).collect()),
                Value::List(forcepos.into_iter().map(fo).collect()),
                Value::List(movepos.into_iter().map(fo).collect()),
            ],
        );
        Ok(())
    }
}

struct FakeMove {
    end_pos: Vec<f64>,
    axes_d: Vec<f64>,
    move_d: f64,
    log: Log,
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        panic!("start_pos unused")
    }
    fn end_pos(&self) -> Coord {
        Coord(self.end_pos.clone())
    }
    fn axes_d(&self) -> Coord {
        Coord(self.axes_d.clone())
    }
    fn axes_r(&self) -> Coord {
        panic!("axes_r unused")
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn is_kinematic_move(&self) -> bool {
        panic!("is_kinematic_move unused")
    }
    fn accel(&self) -> f64 {
        panic!("accel unused")
    }
    fn max_cruise_v2(&self) -> f64 {
        panic!("max_cruise_v2 unused")
    }
    fn start_v(&self) -> f64 {
        panic!("start_v unused")
    }
    fn cruise_v(&self) -> f64 {
        panic!("cruise_v unused")
    }
    fn accel_t(&self) -> f64 {
        panic!("accel_t unused")
    }
    fn cruise_t(&self) -> f64 {
        panic!("cruise_t unused")
    }
    fn decel_t(&self) -> f64 {
        panic!("decel_t unused")
    }
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> {
        push(&self.log, vec![s("limit_speed"), f(speed), f(accel)]);
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos;
        let m = pycompat::text::percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                PctValue::Str(msg),
                PctValue::Float(ep[0]),
                PctValue::Float(ep[1]),
                PctValue::Float(ep[2]),
                PctValue::Float(ep[3]),
            ],
        )
        .expect("move_error format");
        KlippyError::Command(m)
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let text = n.0.to_string();
            match text.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(text),
            }
        }
        PyValue::Float(x) => Value::float(*x),
        PyValue::Str(t) => Value::Str(t.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host{h}")),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "kinematics.corexz".to_string(),
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
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// Compares an oracle result with a Rust result: equal values, or the same
/// exception class and message.
fn check_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(v), Ok(a)) => check(label, Ok(v), a),
        (Err(pe), Err(re)) => {
            assert_eq!(re.class_name(), pe.class_name.as_str(), "{label}: exception class");
            assert_eq!(re.message(), pe.message.as_str(), "{label}: exception message");
        }
        (Ok(v), Err(re)) => {
            panic!("{label}: oracle returned {v:?}, port raised {}: {}", re.class_name(), re.message())
        }
        (Err(pe), Ok(a)) => {
            panic!("{label}: oracle raised {}: {}, port returned {a:?}", pe.class_name, pe.message)
        }
    }
}

fn py_f(x: f64) -> String {
    assert!(x.is_finite());
    format!("{x:?}")
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_floats(xs: &[f64]) -> String {
    format!("[{}]", xs.iter().map(|x| py_f(*x)).collect::<Vec<_>>().join(", "))
}

// ---------------------------------------------------------------------------
// Scenario construction.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Build {
    label: &'static str,
    max_vel: f64,
    max_accel: f64,
    rails: [RailSpec; 3],
    floats: Vec<(&'static str, f64)>,
}

const SECTIONS: [&str; 3] = ["stepper_x", "stepper_y", "stepper_z"];

fn base_build() -> Build {
    Build {
        label: "default",
        max_vel: 300.0,
        max_accel: 3000.0,
        rails: [
            RailSpec {
                range: (0.0, 200.0),
                endstop: 0.0,
                positive_dir: false,
                steppers: vec!["stepper_x", "stepper_x1"],
                endstops: 1,
            },
            RailSpec {
                range: (-20.0, 180.0),
                endstop: -20.0,
                positive_dir: false,
                steppers: vec!["stepper_y"],
                endstops: 1,
            },
            RailSpec {
                range: (0.0, 50.0),
                endstop: 50.0,
                positive_dir: true,
                steppers: vec!["stepper_z", "stepper_z1", "stepper_z2"],
                endstops: 1,
            },
        ],
        floats: Vec::new(),
    }
}

fn builds() -> Vec<Build> {
    let mut out = vec![base_build()];

    let mut overrides = base_build();
    overrides.label = "max_z_overrides";
    overrides.floats = vec![("max_z_velocity", 100.0), ("max_z_accel", 500.0)];
    out.push(overrides);

    let mut inverted = base_build();
    inverted.label = "inverted_homing";
    inverted.max_vel = 250.0;
    inverted.max_accel = 1500.0;
    inverted.floats = vec![("max_z_velocity", 12.5)];
    inverted.rails = [
        RailSpec {
            range: (-10.5, 220.25),
            endstop: 220.25,
            positive_dir: true,
            steppers: vec!["stepper_x"],
            endstops: 2,
        },
        RailSpec {
            range: (3.5, 190.75),
            endstop: 190.75,
            positive_dir: true,
            steppers: vec!["stepper_y"],
            endstops: 1,
        },
        RailSpec {
            range: (0.5, 30.75),
            endstop: 0.5,
            positive_dir: false,
            steppers: vec!["stepper_z", "stepper_z1"],
            endstops: 3,
        },
    ];
    out.push(inverted);

    let mut stepperless = base_build();
    stepperless.label = "no_steppers_on_x_and_z";
    stepperless.rails[0].steppers = Vec::new();
    stepperless.rails[0].endstops = 0;
    stepperless.rails[2].steppers = Vec::new();
    stepperless.rails[2].endstops = 0;
    out.push(stepperless);

    out
}

fn py_specs(b: &Build) -> String {
    let items: Vec<String> = SECTIONS
        .iter()
        .zip(b.rails.iter())
        .map(|(n, r)| {
            format!(
                "'{n}': (({}, {}), (5.0, {}, 2.0, 1.0, {}, 1.0), [{}], {})",
                py_f(r.range.0),
                py_f(r.range.1),
                py_f(r.endstop),
                py_bool(r.positive_dir),
                r.steppers.iter().map(|x| format!("'{x}'")).collect::<Vec<_>>().join(", "),
                r.endstops
            )
        })
        .collect();
    format!("{{{}}}", items.join(", "))
}

fn py_float_opts(b: &Build) -> String {
    let items: Vec<String> =
        b.floats.iter().map(|(k, v)| format!("'{k}': {}", py_f(*v))).collect();
    format!("{{{}}}", items.join(", "))
}

struct Rig {
    kin: Kinematics,
    log: Log,
}

/// Builds the kinematics on both sides. Returns `None` when both raised the
/// same exception; the constructor event logs are compared either way.
fn build_pair(py: &mut pyoracle::Oracle, b: &Build, label: &str) -> Option<Rig> {
    let py_res = py.exec(&format!(
        "kin = None\nkin = build({}, {}, {}, {})",
        py_f(b.max_vel),
        py_f(b.max_accel),
        py_specs(b),
        py_float_opts(b)
    ));
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let factory: StepperFactory = Arc::new(FakeFactory {
        specs: SECTIONS
            .iter()
            .zip(b.rails.iter())
            .map(|(n, r)| (n.to_string(), r.clone()))
            .collect(),
        log: log.clone(),
    });
    let th: ToolHead = Arc::new(FakeToolHead {
        max_velocity: b.max_vel,
        max_accel: b.max_accel,
        factory,
        log: log.clone(),
    });
    let cfg: Config = Arc::new(FakeConfig {
        name: "printer".to_string(),
        floats: b.floats.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
        log: log.clone(),
    });
    let rust_res = load_kinematics(&th, &cfg);
    let rig = match (py_res, rust_res) {
        (Ok(_), Ok(kin)) => Some(Rig { kin, log: log.clone() }),
        (Err(pe), Err(re)) => {
            assert_eq!(re.class_name(), pe.class_name.as_str(), "{label}: constructor class");
            assert_eq!(re.message(), pe.message.as_str(), "{label}: constructor message");
            None
        }
        (Ok(_), Err(re)) => {
            panic!("{label}: oracle built, port raised {}: {}", re.class_name(), re.message())
        }
        (Err(pe), Ok(_)) => {
            panic!("{label}: oracle raised {}: {}, port built", pe.class_name, pe.message)
        }
    };
    compare_events(py, &format!("{label}/constructor events"), &log);
    rig
}

fn compare_events(py: &mut pyoracle::Oracle, label: &str, log: &Log) {
    let expected = py.eval("take()");
    check(label, expected, take(log));
}

// ---------------------------------------------------------------------------
// Operations driven on both sides.
// ---------------------------------------------------------------------------

fn op_status(py: &mut pyoracle::Oracle, rig: &Rig, label: &str) {
    let expected = py.eval("kin.get_status(0.0)");
    check(label, expected, pyvalue_to_value(&rig.kin.get_status(0.0).unwrap()));
}

fn op_steppers(py: &mut pyoracle::Oracle, rig: &Rig, label: &str) {
    let expected = py.eval("[s.get_name() for s in kin.get_steppers()]");
    let names: Vec<Value> = rig.kin.get_steppers().iter().map(|st| s(&st.get_name(None))).collect();
    check(label, expected, Value::List(names));
}

fn op_calc_position(py: &mut pyoracle::Oracle, rig: &Rig, label: &str, pos: &[(&str, f64)]) {
    let dict = pos
        .iter()
        .map(|(k, v)| format!("'{k}': {}", py_f(*v)))
        .collect::<Vec<_>>()
        .join(", ");
    let expected = py.eval(&format!("kin.calc_position({{{dict}}})"));
    let map: indexmap::IndexMap<String, f64> =
        pos.iter().map(|(k, v)| (k.to_string(), *v)).collect();
    let actual = rig
        .kin
        .calc_position(&map)
        .map(|v| Value::List(v.into_iter().map(fo).collect()));
    check_result(label, expected, actual);
}

fn op_set_position(
    py: &mut pyoracle::Oracle,
    rig: &Rig,
    label: &str,
    pos: &[f64],
    axes: &str,
) {
    let expected = py.eval(&format!("kin.set_position({}, '{axes}')", py_floats(pos)));
    let actual = rig.kin.set_position(pos.to_vec(), axes).map(|_| Value::Null);
    check_result(label, expected, actual);
    compare_events(py, &format!("{label}: events"), &rig.log);
    op_status(py, rig, &format!("{label}: status"));
}

fn op_clear(py: &mut pyoracle::Oracle, rig: &Rig, label: &str, axes: &str) {
    let expected = py.eval(&format!("kin.clear_homing_state('{axes}')"));
    rig.kin.clear_homing_state(axes).expect("clear_homing_state");
    check_result(label, expected, Ok(Value::Null));
    compare_events(py, &format!("{label}: events"), &rig.log);
    op_status(py, rig, &format!("{label}: status"));
}

fn op_home(py: &mut pyoracle::Oracle, rig: &Rig, label: &str, axes: &[i64]) {
    let list = axes.iter().map(|a| a.to_string()).collect::<Vec<_>>().join(", ");
    py.exec(&format!("hs = FakeHoming([{list}])")).expect("FakeHoming");
    let expected = py.eval("kin.home(hs)");
    let hs = Arc::new(FakeHoming { axes: axes.to_vec(), log: rig.log.clone() });
    let actual = rig.kin.home(hs).map(|_| Value::Null);
    check_result(label, expected, actual);
    compare_events(py, &format!("{label}: events"), &rig.log);
}

fn op_check_move(
    py: &mut pyoracle::Oracle,
    rig: &Rig,
    label: &str,
    start: [f64; 4],
    end: [f64; 4],
    move_d: f64,
) {
    py.exec(&format!(
        "m = FakeMove({}, {}, {})",
        py_floats(&start),
        py_floats(&end),
        py_f(move_d)
    ))
    .expect("FakeMove");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove {
        end_pos: end.to_vec(),
        axes_d: start.iter().zip(end.iter()).map(|(a, b)| b - a).collect(),
        move_d,
        log: rig.log.clone(),
    };
    let actual = rig.kin.check_move(&mv).map(|_| Value::Null);
    check_result(label, expected, actual);
    compare_events(py, &format!("{label}: events"), &rig.log);
}

/// Moves derived from the rail ranges: inside, on the limits, beyond them,
/// and with zero `axes_d` on an axis whose end position is out of range.
fn move_cases(b: &Build) -> Vec<(&'static str, [f64; 4], [f64; 4], f64)> {
    let lo = [b.rails[0].range.0, b.rails[1].range.0, b.rails[2].range.0];
    let hi = [b.rails[0].range.1, b.rails[1].range.1, b.rails[2].range.1];
    let mid = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0];
    let at = |x: f64, y: f64, z: f64, e: f64| [x, y, z, e];
    let m0 = at(mid[0], mid[1], mid[2], 0.0);
    vec![
        ("xy_inside", m0, at(mid[0] + 1.5, mid[1] - 2.25, mid[2], 0.0), 2.7),
        ("xyz_inside", m0, at(mid[0] + 1.0, mid[1] + 2.0, mid[2] + 0.5, 0.1), 2.3333),
        ("z_only_inside", m0, at(mid[0], mid[1], mid[2] + 3.0, 0.0), 3.0),
        ("x_over_max", m0, at(hi[0] + 0.5, mid[1], mid[2], 0.0), 7.5),
        ("x_at_max", m0, at(hi[0], mid[1], mid[2], 0.0), 7.5),
        ("x_at_min", m0, at(lo[0], mid[1], mid[2], 0.0), 7.5),
        ("y_under_min", m0, at(mid[0], lo[1] - 0.25, mid[2], 0.0), 7.5),
        ("z_over_max_with_xy", m0, at(mid[0] + 1.0, mid[1] + 1.0, hi[2] + 1.0, 0.0), 9.25),
        ("z_at_min_with_xy", m0, at(mid[0] + 1.0, mid[1], lo[2], 0.0), 9.25),
        ("extrude_only", m0, at(mid[0], mid[1], mid[2], 1.0), 1.0),
        ("no_motion", m0, m0, 0.0),
        (
            "y_move_x_already_out",
            at(hi[0] + 1.0, mid[1], mid[2], 0.0),
            at(hi[0] + 1.0, mid[1] + 1.0, mid[2], 0.0),
            1.0,
        ),
        (
            "z_move_x_already_out",
            at(hi[0] + 1.0, mid[1], mid[2], 0.0),
            at(hi[0] + 1.0, mid[1], mid[2] + 1.0, 0.0),
            1.0,
        ),
    ]
}

fn run_move_cases(py: &mut pyoracle::Oracle, rig: &Rig, b: &Build, phase: &str) {
    for (name, start, end, move_d) in move_cases(b) {
        op_check_move(
            py,
            rig,
            &format!("{}/{phase}/{name}", b.label),
            start,
            end,
            move_d,
        );
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constructor_matches_python() {
    let Some(mut py) = oracle() else { return };
    for b in builds() {
        let rig = build_pair(&mut py, &b, b.label).expect("constructor succeeds");
        op_status(&mut py, &rig, &format!("{}/initial status", b.label));
        op_steppers(&mut py, &rig, &format!("{}/steppers", b.label));
    }
}

#[test]
fn constructor_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    // Z has no endstop: the first X stepper wiring raises IndexError.
    let mut no_z_endstop = base_build();
    no_z_endstop.label = "z_without_endstop";
    no_z_endstop.rails[2].endstops = 0;
    assert!(build_pair(&mut py, &no_z_endstop, no_z_endstop.label).is_none());

    // X has no endstop: the X steppers are wired to Z first, then the Z
    // steppers hit the IndexError.
    let mut no_x_endstop = base_build();
    no_x_endstop.label = "x_without_endstop";
    no_x_endstop.rails[0].endstops = 0;
    assert!(build_pair(&mut py, &no_x_endstop, no_x_endstop.label).is_none());
}

#[test]
fn calc_position_matches_python() {
    let Some(mut py) = oracle() else { return };
    let b = base_build();
    let rig = build_pair(&mut py, &b, b.label).expect("constructor succeeds");
    for (i, (x, y, z)) in [
        (10.0, 20.0, 4.0),
        (0.0, 0.0, 0.0),
        (-5.5, 123.25, 100.0),
        (1e-9, -2.5e9, 3.75),
        (0.1, 0.2, 0.3),
    ]
    .into_iter()
    .enumerate()
    {
        op_calc_position(
            &mut py,
            &rig,
            &format!("calc_position #{i}"),
            &[("stepper_x", x), ("stepper_y", y), ("stepper_z", z)],
        );
    }
    // Extra keys are ignored; missing keys raise KeyError for the first
    // missing rail in x, y, z order.
    op_calc_position(
        &mut py,
        &rig,
        "calc_position extra key",
        &[("stepper_x", 1.0), ("stepper_y", 2.0), ("stepper_z", 3.0), ("extruder", 9.0)],
    );
    op_calc_position(&mut py, &rig, "calc_position no z", &[("stepper_x", 1.0), ("stepper_y", 2.0)]);
    op_calc_position(&mut py, &rig, "calc_position only y", &[("stepper_y", 2.0)]);
    op_calc_position(&mut py, &rig, "calc_position empty", &[]);
}

#[test]
fn set_position_and_status_match_python() {
    let Some(mut py) = oracle() else { return };
    for b in builds() {
        let rig = build_pair(&mut py, &b, b.label).expect("constructor succeeds");
        let l = |what: &str| format!("{}/{what}", b.label);
        let p1 = [10.0, 20.0, 30.0, 0.0];
        let p2 = [5.5, 6.25, 7.125, 1.5];
        op_set_position(&mut py, &rig, &l("set_position no homing axes"), &p1, "");
        op_set_position(&mut py, &rig, &l("set_position x"), &p1, "x");
        op_set_position(&mut py, &rig, &l("set_position yz"), &p2, "yz");
        op_set_position(&mut py, &rig, &l("set_position xyz"), &p2, "xyz");
        op_clear(&mut py, &rig, &l("clear y"), "y");
        op_clear(&mut py, &rig, &l("clear nothing"), "");
        op_set_position(&mut py, &rig, &l("set_position no axes after clear"), &p1, "");
        op_clear(&mut py, &rig, &l("clear xz"), "xz");
        op_set_position(&mut py, &rig, &l("set_position z"), &p1, "z");
        op_set_position(&mut py, &rig, &l("set_position with extra axes in string"), &p2, "xe");
        op_clear(&mut py, &rig, &l("clear xyz"), "xyz");
        // Fewer than three coordinates: the first stepper write raises IndexError.
        op_set_position(&mut py, &rig, &l("set_position short"), &[1.0, 2.0], "x");
    }
}

#[test]
fn home_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases: [(&str, Vec<i64>); 11] = [
        ("all", vec![0, 1, 2]),
        ("z only", vec![2]),
        ("x only", vec![0]),
        ("y then x", vec![1, 0]),
        ("z x y", vec![2, 0, 1]),
        ("no axes", vec![]),
        ("axis 3 raises", vec![3]),
        ("axis -1 wraps", vec![-1]),
        ("axis -3 wraps", vec![-3]),
        ("axis -4 raises", vec![-4]),
        ("x then axis 5 raises", vec![0, 5]),
    ];
    for b in builds() {
        let rig = build_pair(&mut py, &b, b.label).expect("constructor succeeds");
        for (name, axes) in &cases {
            op_home(&mut py, &rig, &format!("{}/home {name}", b.label), axes);
        }
    }
}

#[test]
fn check_move_matches_python() {
    let Some(mut py) = oracle() else { return };
    for b in builds() {
        let rig = build_pair(&mut py, &b, b.label).expect("constructor succeeds");
        let pos = [1.0, 2.0, 3.0, 0.0];
        run_move_cases(&mut py, &rig, &b, "unhomed");
        op_set_position(&mut py, &rig, &format!("{}/home x", b.label), &pos, "x");
        run_move_cases(&mut py, &rig, &b, "x homed");
        op_clear(&mut py, &rig, &format!("{}/clear x", b.label), "x");
        op_set_position(&mut py, &rig, &format!("{}/home z", b.label), &pos, "z");
        run_move_cases(&mut py, &rig, &b, "z homed");
        op_set_position(&mut py, &rig, &format!("{}/home xyz", b.label), &pos, "xyz");
        run_move_cases(&mut py, &rig, &b, "xyz homed");
        op_clear(&mut py, &rig, &format!("{}/clear y", b.label), "y");
        run_move_cases(&mut py, &rig, &b, "y cleared");
    }
}
