//! Parity tests for `kinematics::corexy` (port of `klippy/kinematics/corexy.py`).
use indexmap::IndexMap;
use kinematics::corexy::{load_kinematics, CoreXYKinematics};
use klippy_api::config::{Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts, GetListOpts, GetFloatListOpts, GetIntListOpts, GetStrOpts};
use klippy_api::mcu::{Endstop, EndstopApi};
use klippy_api::printer::Printer;
use klippy_api::stepper::{HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory, StepperFactoryApi, StepperKinematics, Trapq};
use klippy_api::toolhead::{Coord, DeltaCalibration, ExtraAxis, HomingStateApi, HomingStateHandle, Kinematics, MoveApi, PrinterExtruder, ToolHead, ToolHeadApi};
use klippy_api::{KResult, KlippyError};
use kos::sync::Mutex;
use pycompat::PyValue;
use recording::{assert_equal, Exception, Record, Value as Rv};
use std::sync::Arc;
fn pyvalue_to_rec(v: &PyValue) -> Rv {
    match v {
        PyValue::None => Rv::Null,
        PyValue::Bool(b) => Rv::Bool(*b),
        PyValue::Int(n) => Rv::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Rv::float(*f),
        PyValue::Str(s) => Rv::Str(s.clone()),
        PyValue::Bytes(b) => Rv::Bytes(b.clone()),
        PyValue::List(l) => Rv::List(l.iter().map(pyvalue_to_rec).collect()),
        PyValue::Tuple(t) => Rv::Tuple(t.iter().map(pyvalue_to_rec).collect()),
        PyValue::Dict(d) => Rv::Dict(d.iter().map(|(k, v)| (pyvalue_to_rec(k), pyvalue_to_rec(v))).collect()),
        PyValue::Host(_) => Rv::Null,
    }
}
fn floats_to_rec(v: &[Option<f64>]) -> Rv { Rv::List(v.iter().map(|o| match o { Some(f) => Rv::float(*f), None => Rv::Null }).collect()) }
fn dummy_record(seq: u64, function: &str) -> Record { Record { v: 1, unit: "kinematics.corexy".to_string(), scenario: "parity".to_string(), seq, function: function.to_string(), args: vec![], ret: None, exc: None, side_effects: Default::default(), object_id: None, kind: recording::RecordKind::Python } }
fn check_equal(seq: u64, function: &str, expected: &Rv, actual: &Rv) { let rec = dummy_record(seq, function); assert_equal(&rec, "ret", expected, actual).unwrap_or_else(|d| panic!("{d}")); }
fn check_err(seq: u64, function: &str, expected: &Exception, actual: &KlippyError) { assert_eq!(actual.class_name(), expected.class_name, "seq {seq} {function} class"); assert_eq!(actual.message(), expected.message, "seq {seq} {function} message"); }
#[derive(Clone, Default)]
struct FakeStepperState { trapq_calls: usize, trapq_is_some: bool, itersolve_calls: Vec<(String, Vec<PyValue>)>, position: [f64; 3] }
struct FakeStepper { name: String, state: Mutex<FakeStepperState> }
impl FakeStepper { fn new(name: &str) -> Self { Self { name: name.to_string(), state: Mutex::new(Default::default()) } } }
impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { panic!("unused") }
    fn get_name(&self, _short: Option<bool>) -> String { self.name.clone() }
    fn get_oid(&self) -> i64 { 0 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) { self.state.lock().itersolve_calls.push((alloc_func.to_string(), params.to_vec())); }
    fn get_step_dist(&self) -> f64 { 0.0 }
    fn get_rotation_distance(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) { (false, false) }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 { coord[0] }
    fn set_position(&self, coord: [f64; 3]) { self.state.lock().position = coord; }
    fn get_commanded_position(&self) -> f64 { 0.0 }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 { 0 }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 { 0 }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 { mcu_pos as f64 }
    fn dump_steps(&self, _count: i64, _start_clock: i64, _end_clock: i64) -> (Vec<PullHistoryStep>, i64) { (vec![], 0) }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> { None }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> { None }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<Trapq> { None }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> { let mut s = self.state.lock(); s.trapq_calls += 1; s.trapq_is_some = tq.is_some(); None }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _axis: char) -> bool { false }
}
struct FakeEndstop { steppers: Mutex<Vec<Stepper>> }
impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> klippy_api::KResult<klippy_api::mcu::Mcu> { panic!("unused") }
    fn add_stepper(&self, stepper: Stepper) { self.steppers.lock().push(stepper); }
    fn get_steppers(&self) -> Vec<Stepper> { self.steppers.lock().clone() }
    fn home_start(&self, _print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, _triggered: Option<bool>) -> klippy_api::KResult<reactor::Completion<bool>> { panic!("unused") }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> { Ok(0.0) }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> { Ok(0) }
}
struct FakeRailInner { name: String, range: (f64, f64), hi: HomingInfo, steppers: Vec<Stepper>, endstops: Vec<(Endstop, String)>, itersolve_calls: Vec<(String, Vec<PyValue>)>, set_position_calls: Vec<[f64; 3]> }
struct FakeRail { inner: Mutex<FakeRailInner> }
impl FakeRail {
    fn new(name: &str, lo: f64, hi_: f64, pos_endstop: f64, positive_dir: bool) -> Self {
        let stepper: Stepper = Arc::new(FakeStepper::new(name));
        let endstop: Endstop = Arc::new(FakeEndstop { steppers: Mutex::new(vec![]) });
        Self { inner: Mutex::new(FakeRailInner { name: name.to_string(), range: (lo, hi_), hi: HomingInfo { speed: 5.0, position_endstop: pos_endstop, retract_speed: 5.0, retract_dist: 5.0, positive_dir, second_homing_speed: 2.5 }, steppers: vec![stepper], endstops: vec![(endstop, name.to_string())], itersolve_calls: vec![], set_position_calls: vec![] }) }
    }
}
impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String { self.inner.lock().name.clone() }
    fn get_range(&self) -> (f64, f64) { self.inner.lock().range }
    fn get_homing_info(&self) -> HomingInfo { self.inner.lock().hi }
    fn get_steppers(&self) -> Vec<Stepper> { self.inner.lock().steppers.clone() }
    fn get_endstops(&self) -> Vec<(Endstop, String)> { self.inner.lock().endstops.clone() }
    fn lookup_endstop(&self, _endstop_pin: Option<&str>, _name: &str) -> KResult<Endstop> { panic!("unused") }
    fn add_stepper(&self, _stepper: Stepper, _endstop_pin: Option<&str>, _name: Option<&str>) -> KResult<()> { Ok(()) }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> { Ok(()) }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) { self.inner.lock().itersolve_calls.push((alloc_func.to_string(), params.to_vec())); }
    fn set_trapq(&self, _tq: Option<Trapq>) {}
    fn set_position(&self, coord: [f64; 3]) { self.inner.lock().set_position_calls.push(coord); }
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 { coord[0] }
    fn get_commanded_position(&self) -> f64 { 0.0 }
}
struct DummyTrapq;
impl klippy_api::stepper::TrapqApi for DummyTrapq {
    fn as_any(&self) -> &dyn std::any::Any { self }
}
struct FakeFactory;
impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> { panic!("unused") }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> { panic!("unused") }
    fn lookup_multi_rail(&self, config: &Config, _opts: RailOpts) -> KResult<Rail> { let name = config.get_name(); let (lo, hi, es, dir) = match name.as_str() { "stepper_x" => (0.0, 100.0, 0.0, false), "stepper_y" => (0.0, 200.0, 0.0, false), "stepper_z" => (0.0, 300.0, 0.0, true), other => panic!("unexpected {other}"), }; Ok(Arc::new(FakeRail::new(&name, lo, hi, es, dir)) as Rail) }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> { panic!("unused") }
    fn parse_step_distance(&self, _config: &Config, _units_in_radians: Option<bool>, _note_valid: bool) -> KResult<(f64, f64)> { Ok((0.0, 0.0)) }
}
#[derive(Default)]
struct FakeConfigState { values: IndexMap<String, f64>, err_on: Option<(String, String)> }
struct FakeConfig { name: String, center: Arc<FakeConfigState> }
impl FakeConfig {
    fn new(name: &str, center: Arc<FakeConfigState>) -> Self { Self { name: name.to_string(), center } }
    fn section(&self, section: &str) -> Config { Arc::new(FakeConfig::new(section, self.center.clone())) as Config }
}
impl klippy_api::config::ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { panic!("unused") }
    fn get_name(&self) -> String { self.name.clone() }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> { Ok(None) }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v = self.center.values.get(option).copied();
        push_hist((option.to_string(), opts.default, opts.above, opts.maxval));
        if let Some((opt, msg)) = self.center.err_on.clone() { if opt == option { return Err(KlippyError::Config(msg)); } }
        Ok(v.or(opts.default.and_then(|d| d)))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, section: &str) -> Config { self.section(section) }
    fn has_section(&self, _section: &str) -> bool { true }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> { vec![] }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> { Ok(vec![]) }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}
type HistEntry = (String, Option<Option<f64>>, Option<f64>, Option<f64>);
thread_local! {
    // Per test thread: parallel tests clear and read their own history.
    static HIST: std::cell::RefCell<Vec<HistEntry>> = const { std::cell::RefCell::new(Vec::new()) };
}
fn push_hist(entry: HistEntry) { HIST.with(|h| h.borrow_mut().push(entry)); }
fn clear_hist() { HIST.with(|h| h.borrow_mut().clear()); }
fn getfloat_hist() -> Vec<HistEntry> { HIST.with(|h| h.borrow().clone()) }
impl FakeConfigState { fn getfloat_hist(&self) -> Vec<(String, Option<Option<f64>>, Option<f64>, Option<f64>)> { getfloat_hist() } }
struct FakeToolHead { max_vel: f64, max_accel: f64, trapq: Trapq }
impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> { vec![0.0; 4] }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> { Ok(()) }
    fn get_kinematics(&self) -> Kinematics { panic!("unused") }
    fn flush_step_generation(&self) -> KResult<()> { Ok(()) }
    fn get_last_move_time(&self) -> KResult<f64> { Ok(0.0) }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _newpos: Vec<f64>, _speed: f64, _drip_completion: &reactor::Completion<bool>) -> KResult<()> { Ok(()) }
}
impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> { Ok(PyValue::None) }
    fn get_trapq(&self) -> Trapq { self.trapq.clone() }
    fn get_max_velocity(&self) -> (f64, f64) { (self.max_vel, self.max_accel) }
    fn set_max_velocities(&self, _a: Option<f64>, _b: Option<f64>, _c: Option<f64>, _d: Option<f64>) -> (f64, f64, f64, f64) { (0.0, 0.0, 0.0, 0.0) }
    fn register_lookahead_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder { panic!("unused") }
    fn add_extra_axis(&self, _ea: ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> { vec![] }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> StepperFactory { Arc::new(FakeFactory) }
}
#[derive(Clone)]
struct FakeMove { end_pos: Coord, axes_d: Coord, move_d: f64, limit_speed_calls: Arc<Mutex<Vec<(f64, f64)>>> }
impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord { Coord(vec![0.0, 0.0, 0.0, 0.0]) }
    fn end_pos(&self) -> Coord { self.end_pos.clone() }
    fn axes_d(&self) -> Coord { self.axes_d.clone() }
    fn axes_r(&self) -> Coord { Coord(vec![0.0, 0.0, 0.0, 0.0]) }
    fn move_d(&self) -> f64 { self.move_d }
    fn is_kinematic_move(&self) -> bool { true }
    fn accel(&self) -> f64 { 0.0 }
    fn max_cruise_v2(&self) -> f64 { 0.0 }
    fn start_v(&self) -> f64 { 0.0 }
    fn cruise_v(&self) -> f64 { 0.0 }
    fn accel_t(&self) -> f64 { 0.0 }
    fn cruise_t(&self) -> f64 { 0.0 }
    fn decel_t(&self) -> f64 { 0.0 }
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> { self.limit_speed_calls.lock().push((speed, accel)); Ok(()) }
    fn move_error(&self, msg: String) -> KlippyError { let ep = &self.end_pos.0; let text = pycompat::text::percent_format("%s: %.3f %.3f %.3f [%.3f]", &[pycompat::text::PctValue::Str(msg), pycompat::text::PctValue::Float(ep.first().copied().unwrap_or(0.0)), pycompat::text::PctValue::Float(ep.get(1).copied().unwrap_or(0.0)), pycompat::text::PctValue::Float(ep.get(2).copied().unwrap_or(0.0)), pycompat::text::PctValue::Float(ep.get(3).copied().unwrap_or(0.0))]).unwrap(); KlippyError::Command(text) }
}
#[derive(Default)]
struct FakeHomingInner { axes: Vec<i64>, home_rails_calls: Vec<(Vec<Rail>, Vec<Option<f64>>, Vec<Option<f64>>)> }
struct FakeHoming { inner: Mutex<FakeHomingInner> }
impl HomingStateApi for FakeHoming {
    fn set_axes(&self, axes: Vec<i64>) { self.inner.lock().axes = axes; }
    fn get_axes(&self) -> Vec<i64> { self.inner.lock().axes.clone() }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> { Ok(0) }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> { Ok(()) }
    fn home_rails(&self, rails: Vec<Rail>, forcepos: Vec<Option<f64>>, movepos: Vec<Option<f64>>) -> KResult<()> { self.inner.lock().home_rails_calls.push((rails, forcepos, movepos)); Ok(()) }
}
fn homing_handle(fake: &Arc<FakeHoming>) -> HomingStateHandle { fake.clone() }
fn make_config(values: &[(&str, f64)]) -> (Config, Arc<FakeConfigState>) { clear_hist(); let st = Arc::new(FakeConfigState { values: values.iter().map(|(k, v)| (k.to_string(), *v)).collect(), ..Default::default() }); let cfg: Config = Arc::new(FakeConfig::new("printer", st.clone())) as Config; (cfg, st) }
fn make_toolhead(max_vel: f64, max_accel: f64) -> ToolHead { Arc::new(FakeToolHead { max_vel, max_accel, trapq: Arc::new(DummyTrapq) as Trapq }) as ToolHead }
fn make_kin(values: &[(&str, f64)]) -> (Kinematics, Arc<FakeConfigState>, ToolHead, Config) { let toolhead = make_toolhead(500.0, 3000.0); let (config, st) = make_config(values); let kin = load_kinematics(&toolhead, &config).expect("load_kinematics"); (kin, st, toolhead, config) }
const PY_SETUP: &str = r"
import sys, types
from collections import namedtuple
class Error(Exception):
    pass
class CommandError(Exception):
    pass
homing_info = namedtuple('homing_info', ['speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir', 'second_homing_speed'])
class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.trapq_calls = 0
    def get_name(self, short=False):
        return self._name
    def set_trapq(self, tq):
        self.trapq_calls += 1
    def setup_itersolve(self, alloc_func, *params):
        pass
    def set_position(self, coord):
        pass
class FakeEndstop:
    def __init__(self):
        self.steppers = []
    def add_stepper(self, s):
        self.steppers.append(s.get_name())
RAILS = {}
class FakeRail:
    def __init__(self, name):
        self._name = name
        cfg = {'stepper_x': (0.0, 100.0, 0.0, False), 'stepper_y': (0.0, 200.0, 0.0, False), 'stepper_z': (0.0, 300.0, 0.0, True)}[name]
        self._lo, self._hi, pos_endstop, positive_dir = cfg
        self.steppers = [FakeStepper(name)]
        self.endstops = [(FakeEndstop(), name)]
    def get_name(self, short=False):
        return self._name
    def get_range(self):
        return (self._lo, self._hi)
    def get_homing_info(self):
        return homing_info(5.0, {'stepper_x': 0.0, 'stepper_y': 0.0, 'stepper_z': 0.0}[self._name], 5.0, 5.0, {'stepper_x': False, 'stepper_y': False, 'stepper_z': True}[self._name], 2.5)
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def setup_itersolve(self, alloc_func, *params):
        pass
    def set_trapq(self, tq):
        for s in self.steppers:
            s.set_trapq(tq)
    def set_position(self, coord):
        for s in self.steppers:
            s.set_position(coord)
def LookupMultiRail(config):
    rail = FakeRail(config.get_name())
    RAILS[config.get_name()] = rail
    return rail
fake_stepper = types.ModuleType('stepper')
fake_stepper.LookupMultiRail = LookupMultiRail
sys.modules['stepper'] = fake_stepper
import kinematics.corexy as corexy
def Coord(t):
    t = tuple(t)
    if len(t) < 4:
        t = t + (0,) * (4 - len(t))
    return t
class FakeSection:
    def __init__(self, name, parent):
        self._name = name
        self._parent = parent
    def get_name(self):
        return self._name
    def get_printer(self):
        return None
    def getsection(self, sec):
        return FakeSection(sec, self._parent)
    def has_section(self, sec):
        return True
    def getfloat(self, option, default=None, above=None, maxval=None, minval=None, below=None, note_valid=True):
        self._parent['log'].append((option, default, above, maxval))
        values = self._parent['values']
        if option in self._parent.get('err', {}):
            raise Error(self._parent['err'][option])
        if option in values:
            v = values[option]
        else:
            v = default
        if v is None:
            raise Error('required')
        return float(v)
class FakeToolhead:
    def __init__(self, max_velocity, max_accel):
        self._max = (max_velocity, max_accel)
        self.Coord = Coord
    def get_trapq(self):
        return 'TQ'
    def get_max_velocity(self):
        return self._max
def make_kin(values=None, err=None, max_velocity=500.0, max_accel=3000.0):
    parent = {'values': dict(values or {}), 'log': [], 'err': dict(err or {})}
    config = FakeSection('printer', parent)
    toolhead = FakeToolhead(max_velocity, max_accel)
    kin = corexy.load_kinematics(toolhead, config)
    return kin, config, toolhead, parent
class FakeMove:
    def __init__(self, end_pos, axes_d, move_d):
        self.end_pos = Coord(end_pos)
        self.axes_d = Coord(axes_d)
        self.move_d = move_d
        self.limit_calls = []
    def limit_speed(self, speed, accel):
        self.limit_calls.append((speed, accel))
    def move_error(self, msg='Move out of range'):
        ep = self.end_pos
        return CommandError('%s: %.3f %.3f %.3f [%.3f]' % (msg, ep[0], ep[1], ep[2], ep[3]))
class FakeHoming:
    def __init__(self, axes):
        self._axes = list(axes)
        self.home_rails_calls = []
    def get_axes(self):
        return list(self._axes)
    def home_rails(self, rails, forcepos, movepos):
        self.home_rails_calls.append((list(rails), list(forcepos), list(movepos)))
";
fn oracle_start(py: &mut pyoracle::Oracle) { py.exec(PY_SETUP).expect("py setup"); }
#[test]
fn calc_position_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping: set KLIPPY_PYTHON"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    let expected = py.eval("kin.calc_position({'stepper_x': 12.5, 'stepper_y': 30.0, 'stepper_z': 7.25})").unwrap();
    let (kin, _, _, _) = make_kin(&[]);
    let mut map = IndexMap::new();
    map.insert("stepper_x".to_string(), 12.5);
    map.insert("stepper_y".to_string(), 30.0);
    map.insert("stepper_z".to_string(), 7.25);
    let actual = floats_to_rec(&kin.calc_position(&map).expect("calc_position"));
    check_equal(1, "calc_position", &expected, &actual);
}
#[test]
fn load_kinematics_config_reads_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, parent = make_kin({'max_z_velocity': 25.0, 'max_z_accel': 200.0})").unwrap();
    let expected_log = py.eval("parent['log']").unwrap();
    let (_kin, st, _, _) = make_kin(&[("max_z_velocity", 25.0), ("max_z_accel", 200.0)]);
    let hist = st.getfloat_hist();
    let actual_log = Rv::List(hist.iter().map(|(o, d, above, maxval)| Rv::Tuple(vec![Rv::Str(o.clone()), match d { Some(Some(v)) => Rv::float(*v), _ => Rv::Null }, match above { Some(v) => Rv::float(*v), None => Rv::Null }, match maxval { Some(v) => Rv::float(*v), None => Rv::Null }])).collect());
    check_equal(2, "config log", &expected_log, &actual_log);
}
#[test]
fn load_kinematics_config_error_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    let expected = py.eval("make_kin({'max_z_velocity': 999.0}, {'max_z_velocity': 'bad'})").unwrap_err();
    let (toolhead, config) = { let th = make_toolhead(500.0, 3000.0); let st = Arc::new(FakeConfigState { values: IndexMap::new(), err_on: Some(("max_z_velocity".to_string(), "bad".to_string())), ..Default::default() }); let c: Config = Arc::new(FakeConfig::new("printer", st)) as Config; (th, c) };
    let err = match load_kinematics(&toolhead, &config) { Ok(_) => panic!("expected error"), Err(e) => e };
    check_err(3, "config error", &expected, &err);
}
#[test]
fn get_status_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    let expected = py.eval("kin.get_status(0.0)").unwrap();
    let (kin, _, _, _) = make_kin(&[]);
    let actual = pyvalue_to_rec(&kin.get_status(0.0).unwrap());
    check_equal(4, "get_status", &expected, &actual);
}
#[test]
fn set_position_and_clear_homing_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    py.exec("kin.set_position([10.0, 20.0, 30.0, 0], 'xy')").unwrap();
    let expected = py.eval("kin.get_status(0.0)").unwrap();
    py.exec("kin.clear_homing_state('x')").unwrap();
    let expected2 = py.eval("kin.get_status(0.0)").unwrap();
    let (kin, _, _, _) = make_kin(&[]);
    kin.set_position(vec![10.0, 20.0, 30.0, 0.0], "xy").unwrap();
    let actual = pyvalue_to_rec(&kin.get_status(0.0).unwrap());
    check_equal(5, "set_position status", &expected, &actual);
    kin.clear_homing_state("x").expect("clear_homing_state");
    let actual2 = pyvalue_to_rec(&kin.get_status(0.0).unwrap());
    check_equal(6, "clear_homing_state status", &expected2, &actual2);
}
#[test]
fn check_move_xy_limits_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    let expected = py.eval("kin.check_move(FakeMove([150.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], 1.0))").unwrap_err();
    let (kin, _, _, _) = make_kin(&[]);
    let mv = FakeMove { end_pos: Coord(vec![150.0, 0.0, 0.0, 0.0]), axes_d: Coord(vec![1.0, 0.0, 0.0, 0.0]), move_d: 1.0, limit_speed_calls: Arc::new(Mutex::new(vec![])) };
    let err = match kin.check_move(&mv) { Ok(()) => panic!("expected error"), Err(e) => e };
    check_err(7, "check_move not homed", &expected, &err);
}
#[test]
fn check_move_homed_ok_and_z_limit_match_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    py.exec("kin.set_position([50.0, 100.0, 150.0, 0], 'xyz')").unwrap();
    py.exec("mv_ok = FakeMove([50.0, 100.0, 150.0, 0.0], [0.5, 0.25, 0.0, 0.0], 0.75)").unwrap();
    py.exec("kin.check_move(mv_ok)").unwrap();
    let expected_ok = py.eval("mv_ok.limit_calls").unwrap();
    py.exec("mv_z = FakeMove([50.0, 100.0, 160.0, 0.0], [1.0, 2.0, 5.0, 0.0], 10.0)").unwrap();
    py.exec("kin.check_move(mv_z)").unwrap();
    let expected_z = py.eval("mv_z.limit_calls").unwrap();
    let expected_err = py.eval("kin.check_move(FakeMove([150.0, 100.0, 150.0, 0.0], [1.0, 0.0, 0.0, 0.0], 1.0))").unwrap_err();
    let (kin, _, _, _) = make_kin(&[]);
    kin.set_position(vec![50.0, 100.0, 150.0, 0.0], "xyz").unwrap();
    let calls_ok = Arc::new(Mutex::new(vec![]));
    let mv_ok = FakeMove { end_pos: Coord(vec![50.0, 100.0, 150.0, 0.0]), axes_d: Coord(vec![0.5, 0.25, 0.0, 0.0]), move_d: 0.75, limit_speed_calls: calls_ok.clone() };
    kin.check_move(&mv_ok).unwrap();
    let actual_ok = Rv::List(calls_ok.lock().iter().map(|(a, b)| Rv::Tuple(vec![Rv::float(*a), Rv::float(*b)])).collect());
    let mv_z_calls = Arc::new(Mutex::new(vec![]));
    let mv_z = FakeMove { end_pos: Coord(vec![50.0, 100.0, 160.0, 0.0]), axes_d: Coord(vec![1.0, 2.0, 5.0, 0.0]), move_d: 10.0, limit_speed_calls: mv_z_calls.clone() };
    kin.check_move(&mv_z).unwrap();
    let actual_z = Rv::List(mv_z_calls.lock().iter().map(|(a, b)| Rv::Tuple(vec![Rv::float(*a), Rv::float(*b)])).collect());
    let mv_err = FakeMove { end_pos: Coord(vec![150.0, 100.0, 150.0, 0.0]), axes_d: Coord(vec![1.0, 0.0, 0.0, 0.0]), move_d: 1.0, limit_speed_calls: Arc::new(Mutex::new(vec![])) };
    let err = match kin.check_move(&mv_err) { Ok(()) => panic!("expected error"), Err(e) => e };
    check_equal(8, "check_move ok limit_calls", &expected_ok, &actual_ok);
    check_equal(9, "check_move z limit_calls", &expected_z, &actual_z);
    check_err(10, "check_move out of range", &expected_err, &err);
}
#[test]
fn home_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    py.exec("hs = FakeHoming([0, 2])").unwrap();
    py.exec("kin.home(hs)").unwrap();
    let expected = py.eval("[([(r.get_name(),) for r in c[0]], c[1], c[2]) for c in hs.home_rails_calls]").unwrap();
    let (kin, _, _, _) = make_kin(&[]);
    let hs = Arc::new(FakeHoming { inner: Mutex::new(FakeHomingInner { axes: vec![0, 2], ..Default::default() }) });
    kin.home(homing_handle(&hs)).unwrap();
    let calls = hs.inner.lock().home_rails_calls.clone();
    let actual = Rv::List(calls.iter().map(|(rails, forcepos, homepos)| Rv::Tuple(vec![Rv::List(rails.iter().map(|r| Rv::Tuple(vec![Rv::Str(r.get_name(None))])).collect()), Rv::List(forcepos.iter().map(|o| match o { Some(f) => Rv::float(*f), None => Rv::Null }).collect()), Rv::List(homepos.iter().map(|o| match o { Some(f) => Rv::float(*f), None => Rv::Null }).collect())])).collect());
    check_equal(11, "home calls", &expected, &actual);
}
#[test]
fn get_steppers_matches_oracle() {
    let Some(mut py) = pyoracle::Oracle::start() else { eprintln!("skipping"); return; };
    oracle_start(&mut py);
    py.exec("kin, _, _, _ = make_kin()").unwrap();
    let expected = py.eval("[s.get_name() for s in kin.get_steppers()]").unwrap();
    let (kin, _, _, _) = make_kin(&[]);
    let actual = Rv::List(kin.get_steppers().iter().map(|s| Rv::Str(s.get_name(None))).collect());
    check_equal(12, "get_steppers", &expected, &actual);
}
#[allow(dead_code)]
fn _assert_types() { let _ = std::any::type_name::<Printer>(); let _ = std::any::type_name::<DeltaCalibration>(); let _ = std::any::type_name::<CoreXYKinematics>(); }
