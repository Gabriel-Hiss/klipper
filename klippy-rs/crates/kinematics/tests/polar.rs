//! Parity tests for `kinematics::polar` (port of `klippy/kinematics/polar.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): a fresh `PolarKinematics` is built
//! on both sides with minimal Python fakes for the stepper factory,
//! toolhead and config, and the same calls are driven with identical
//! inputs. Returned values (floats by bits), raised errors (class and
//! message), `get_status` dicts, config reads, factory/itersolve/trapq
//! wiring, stepper positions, homing calls and `limit_speed` calls are
//! compared via `recording::assert_equal`.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use kinematics::polar::{distance_to_center, load_kinematics, PolarKinematics};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::mcu::{EndstopApi, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper,
    StepperFactory, StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, HomingStateHandle, Kinematics, KinematicsApi, LoadKinematics, LookaheadCallback,
    MoveApi, ToolHeadApi,
};
use klippy_api::{EventArg, KlippyError, ObjectRef};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, collections
TRAPQ_SENTINEL = ('trapq', 1)
class CommandError(Exception):
    pass
class Coord(tuple):
    __slots__ = ()
    def __new__(cls, t):
        if len(t) < 4:
            t = tuple(t) + (0,) * (4 - len(t))
        return tuple.__new__(cls, t)
class FakePrinter:
    pass
class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.itersolve_calls = []
        self.trapqs = []
        self.positions = []
    def get_name(self, short=False):
        assert short == False, short
        return self._name
    def setup_itersolve(self, alloc_func, *params):
        self.itersolve_calls.append((alloc_func, params))
    def set_trapq(self, tq):
        self.trapqs.append(tq)
    def set_position(self, coord):
        _ = (coord[0], coord[1], coord[2])
        self.positions.append((coord[0], coord[1], coord[2]))
HomingInfo = collections.namedtuple('homing_info', ['speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir', 'second_homing_speed'])
class FakeRail:
    def __init__(self, name, rng, homing, steppers):
        self._name = name
        self._range = rng
        self._homing = homing
        self._steppers = list(steppers)
        self.itersolve_calls = []
    def get_name(self, short=False):
        assert short == False, short
        return self._name
    def get_range(self):
        return self._range
    def get_homing_info(self):
        return self._homing
    def get_steppers(self):
        return list(self._steppers)
    def setup_itersolve(self, alloc_func, *params):
        self.itersolve_calls.append((alloc_func, params))
    def set_trapq(self, tq):
        for s in self._steppers:
            s.set_trapq(tq)
    def set_position(self, coord):
        for s in self._steppers:
            s.set_position(coord)
FACTORY_CALLS = []
RAIL_CFG = {}
def _printer_stepper(config, units_in_radians=False):
    FACTORY_CALLS.append(('PrinterStepper', config.get_name(), units_in_radians))
    return FakeStepper('stepper_bed')
def _lookup_rail(config, **kw):
    FACTORY_CALLS.append(('LookupRail', config.get_name(), kw))
    cfg = RAIL_CFG['arm']
    return FakeRail('stepper_arm', tuple(cfg['range']), cfg['homing'], [FakeStepper('stepper_arm')])
def _lookup_multi_rail(config, **kw):
    FACTORY_CALLS.append(('LookupMultiRail', config.get_name(), kw))
    cfg = RAIL_CFG['z']
    return FakeRail('stepper_z', tuple(cfg['range']), cfg['homing'], [FakeStepper('stepper_z')])
_fake_stepper = types.ModuleType('stepper')
_fake_stepper.PrinterStepper = _printer_stepper
_fake_stepper.LookupRail = _lookup_rail
_fake_stepper.LookupMultiRail = _lookup_multi_rail
sys.modules['stepper'] = _fake_stepper
import kinematics.polar as polar
class FakeToolhead:
    Coord = Coord
    def __init__(self, mv, ma):
        self._mv = mv
        self._ma = ma
        self.trapq = TRAPQ_SENTINEL
    def get_trapq(self):
        return self.trapq
    def get_max_velocity(self):
        return (self._mv, self._ma)
def _norm(v):
    if isinstance(v, bool) or v is None:
        return v
    if isinstance(v, (int, float)):
        return float(v)
    return v
class FakeConfig:
    def __init__(self, name, floats):
        self._name = name
        self._floats = dict(floats)
        self.getfloat_calls = []
    def get_name(self):
        return self._name
    def get_printer(self):
        return FakePrinter()
    def getsection(self, sec):
        return FakeConfig(sec, {})
    def has_section(self, sec):
        return sec in ('stepper_bed', 'stepper_arm', 'stepper_z', 'kinematics')
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
        self.getfloat_calls.append((option, _norm(default), _norm(minval), _norm(maxval), _norm(above), _norm(below)))
        if option in self._floats:
            return self._floats[option]
        return default
class FakeHomingState:
    def __init__(self, axes):
        self.axes = list(axes)
        self.set_axes_calls = []
        self.home_rails_calls = []
    def get_axes(self):
        return list(self.axes)
    def set_axes(self, axes):
        self.set_axes_calls.append(list(axes))
        self.axes = list(axes)
    def home_rails(self, rails, forcepos, movepos):
        self.home_rails_calls.append((' '.join([r.get_name() for r in rails]), list(forcepos), list(movepos)))
class FakeMove:
    def __init__(self, start_pos, end_pos, axes_d, move_d, max_cruise_v2):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.axes_d = tuple(axes_d)
        self.move_d = move_d
        self.max_cruise_v2 = max_cruise_v2
        self.limit_calls = []
    def limit_speed(self, speed, accel):
        self.limit_calls.append((speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return CommandError(m)
def build(max_vel, max_acc, arm_range, z_range, arm_endstop, arm_pos_dir, z_endstop, z_pos_dir, floats):
    FACTORY_CALLS.clear()
    RAIL_CFG.clear()
    RAIL_CFG['arm'] = {'range': arm_range, 'homing': HomingInfo(5.0, arm_endstop, 2.0, 1.0, arm_pos_dir, 1.0)}
    RAIL_CFG['z'] = {'range': z_range, 'homing': HomingInfo(5.0, z_endstop, 2.0, 1.0, z_pos_dir, 1.0)}
    toolhead = FakeToolhead(max_vel, max_acc)
    config = FakeConfig('kinematics', floats)
    kin = polar.PolarKinematics(toolhead, config)
    return (kin, toolhead, config)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct DummyTrapq {
    id: u64,
}

impl TrapqApi for DummyTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

fn dummy_trapq() -> Trapq {
    let t: Arc<DummyTrapq> = Arc::new(DummyTrapq { id: 1 });
    let t: Trapq = t;
    t
}

fn trapq_id(tq: &Trapq) -> u64 {
    tq.as_any().downcast_ref::<DummyTrapq>().map(|d| d.id).unwrap_or(999)
}

struct FakeStepper {
    name: String,
    iters: Mutex<Vec<(String, Vec<PyValue>)>>,
    trapqs: Mutex<Vec<u64>>,
    positions: Mutex<Vec<[f64; 3]>>,
}

impl FakeStepper {
    fn new(name: &str) -> FakeStepper {
        FakeStepper {
            name: name.to_string(),
            iters: Mutex::new(Vec::new()),
            trapqs: Mutex::new(Vec::new()),
            positions: Mutex::new(Vec::new()),
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu unused")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        0
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        self.iters.lock().push((alloc_func.to_string(), params.to_vec()));
    }
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.positions.lock().push(coord);
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: f64) -> f64 {
        0.0
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<StepperKinematics>,
    ) -> Option<StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        if let Some(t) = tq {
            self.trapqs.lock().push(trapq_id(&t));
        }
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeRail {
    name: String,
    range: (f64, f64),
    homing: HomingInfo,
    steppers: Vec<Stepper>,
    iters: Mutex<Vec<(String, Vec<PyValue>)>>,
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
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
    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)> {
        Vec::new()
    }
    fn lookup_endstop(
        &self, _endstop_pin: Option<&str>, _name: &str,
    ) -> KResult<Arc<dyn EndstopApi>> {
        panic!("lookup_endstop unused")
    }
    fn add_stepper(
        &self, _stepper: Stepper, _endstop_pin: Option<&str>, _endstop_name: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        self.iters.lock().push((alloc_func.to_string(), params.to_vec()));
    }
    fn set_trapq(&self, tq: Option<Trapq>) {
        for s in &self.steppers {
            s.set_trapq(tq.clone());
        }
    }
    fn set_position(&self, coord: [f64; 3]) {
        for s in &self.steppers {
            s.set_position(coord);
        }
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeFactory {
    bed: Stepper,
    arm: Rail,
    z: Rail,
    calls: Mutex<Vec<(String, String, String)>>,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, config: &Config, units_in_radians: bool) -> KResult<Stepper> {
        self.calls.lock().push((
            "PrinterStepper".to_string(),
            config.get_name(),
            if units_in_radians { "true".to_string() } else { "false".to_string() },
        ));
        Ok(self.bed.clone())
    }
    fn lookup_rail(&self, config: &Config, _opts: RailOpts) -> KResult<Rail> {
        self.calls.lock().push((
            "LookupRail".to_string(),
            config.get_name(),
            "{}".to_string(),
        ));
        Ok(self.arm.clone())
    }
    fn lookup_multi_rail(&self, config: &Config, _opts: RailOpts) -> KResult<Rail> {
        self.calls.lock().push((
            "LookupMultiRail".to_string(),
            config.get_name(),
            "{}".to_string(),
        ));
        Ok(self.z.clone())
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail unused")
    }
    fn parse_step_distance(
        &self, _config: &Config, _units_in_radians: Option<bool>, _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance unused")
    }
    fn extruder_stepper(&self, _config: &Config) -> KResult<Option<klippy_api::toolhead::ExtruderStepper>> { panic!("extruder_stepper is not used") }
}

#[derive(Clone)]
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
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self, _name: &str, _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self, _config: &Config, _section: &str, _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FloatCall {
    option: String,
    default: Option<f64>,
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
    below: Option<f64>,
}

struct FakeConfig {
    printer: Printer,
    name: String,
    floats: IndexMap<String, f64>,
    float_calls: Mutex<Vec<FloatCall>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
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
        let default_val = match opts.default {
            Some(Some(v)) => Some(v),
            Some(None) => None,
            None => None,
        };
        self.float_calls.lock().push(FloatCall {
            option: option.to_string(),
            default: default_val,
            minval: opts.minval,
            maxval: opts.maxval,
            above: opts.above,
            below: opts.below,
        });
        if let Some(v) = self.floats.get(option) {
            return Ok(Some(*v));
        }
        match opts.default {
            Some(Some(v)) => Ok(Some(v)),
            Some(None) => Ok(None),
            None => Err(KlippyError::Config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean unused")
    }
    fn getchoice(
        &self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
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
    fn getfloatlist(
        &self, _option: &str, _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist unused")
    }
    fn getsection(&self, section: &str) -> Config {
        let c = FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            floats: IndexMap::new(),
            float_calls: Mutex::new(Vec::new()),
        };
        Arc::new(c)
    }
    fn has_section(&self, section: &str) -> bool {
        ["stepper_bed", "stepper_arm", "stepper_z", "kinematics"].contains(&section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeToolHead {
    max_velocity: (f64, f64),
    trapq: Trapq,
    factory: StepperFactory,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        Vec::new()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self, _newpos: Vec<f64>, _speed: f64, _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_trapq(&self) -> Trapq {
        self.trapq.clone()
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        self.max_velocity
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder unused")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

struct FakeHomingState {
    axes: Mutex<Vec<i64>>,
    set_axes_calls: Mutex<Vec<Vec<i64>>>,
    home_calls: Mutex<Vec<(Vec<String>, Vec<Option<f64>>, Vec<Option<f64>>)>>,
}

impl klippy_api::toolhead::HomingStateApi for FakeHomingState {
    fn set_axes(&self, axes: Vec<i64>) {
        *self.axes.lock() = axes.clone();
        self.set_axes_calls.lock().push(axes);
    }
    fn get_axes(&self) -> Vec<i64> {
        self.axes.lock().clone()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        Ok(0)
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        Ok(())
    }
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        let names: Vec<String> = rails.iter().map(|r| r.get_name(None)).collect();
        self.home_calls.lock().push((names, forcepos, movepos));
        Ok(())
    }
}

struct FakeMove {
    start_pos: Vec<f64>,
    end_pos: Vec<f64>,
    axes_d: Vec<f64>,
    move_d: f64,
    max_cruise_v2: f64,
    limit_calls: Mutex<Vec<(f64, f64)>>,
}

impl FakeMove {
    fn new(
        start_pos: Vec<f64>,
        end_pos: Vec<f64>,
        axes_d: Vec<f64>,
        move_d: f64,
        max_cruise_v2: f64,
    ) -> FakeMove {
        FakeMove {
            start_pos,
            end_pos,
            axes_d,
            move_d,
            max_cruise_v2,
            limit_calls: Mutex::new(Vec::new()),
        }
    }
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        Coord(self.start_pos.clone())
    }
    fn end_pos(&self) -> Coord {
        Coord(self.end_pos.clone())
    }
    fn axes_d(&self) -> Coord {
        Coord(self.axes_d.clone())
    }
    fn axes_r(&self) -> Coord {
        Coord(Vec::new())
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn is_kinematic_move(&self) -> bool {
        true
    }
    fn accel(&self) -> f64 {
        0.0
    }
    fn max_cruise_v2(&self) -> f64 {
        self.max_cruise_v2
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
        self.limit_calls.lock().push((speed, accel));
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos;
        let m = pycompat::text::percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                pycompat::PctValue::Str(msg),
                pycompat::PctValue::Float(ep[0]),
                pycompat::PctValue::Float(ep[1]),
                pycompat::PctValue::Float(ep[2]),
                pycompat::PctValue::Float(ep[3]),
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
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
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
        unit: "kinematics.polar".to_string(),
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

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let (oracle_exc, rust_err) = match (expected, actual) {
        (Err(e), Err(r)) => (e, r),
        (Ok(v), Ok(_)) => panic!("{label}: oracle returned {v:?} but rust must fail"),
        (Ok(v), Err(_)) => panic!("{label}: oracle returned {v:?} but rust must fail"),
        (Err(e), Ok(_)) => panic!("{label}: oracle raised {} but rust succeeded", e.class_name),
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: message"
    );
}

#[allow(clippy::too_many_arguments)]
struct Params {
    max_vel: f64,
    max_acc: f64,
    arm_range: (f64, f64),
    z_range: (f64, f64),
    arm_endstop: f64,
    arm_pos_dir: bool,
    z_endstop: f64,
    z_pos_dir: bool,
    floats: IndexMap<String, f64>,
}

fn default_params() -> Params {
    let floats = IndexMap::new();
    Params {
        max_vel: 250.0,
        max_acc: 3000.0,
        arm_range: (0.0, 150.0),
        z_range: (10.0, 250.0),
        arm_endstop: 20.0,
        arm_pos_dir: true,
        z_endstop: 15.0,
        z_pos_dir: false,
        floats,
    }
}

struct Harness {
    polar: Arc<PolarKinematics>,
    fake_config: Arc<FakeConfig>,
    factory: Arc<FakeFactory>,
    bed: Arc<FakeStepper>,
    arm_rail: Arc<FakeRail>,
    z_rail: Arc<FakeRail>,
    arm_stepper: Arc<FakeStepper>,
    z_stepper: Arc<FakeStepper>,
}

fn rust_build(p: &Params) -> Harness {
    let bed = Arc::new(FakeStepper::new("stepper_bed"));
    let arm_stepper = Arc::new(FakeStepper::new("stepper_arm"));
    let z_stepper = Arc::new(FakeStepper::new("stepper_z"));
    let bed_t: Stepper = bed.clone();
    let arm_s: Stepper = arm_stepper.clone();
    let z_s: Stepper = z_stepper.clone();
    let arm_homing = HomingInfo {
        speed: 5.0,
        position_endstop: p.arm_endstop,
        retract_speed: 2.0,
        retract_dist: 1.0,
        positive_dir: p.arm_pos_dir,
        second_homing_speed: 1.0,
    };
    let z_homing = HomingInfo {
        speed: 5.0,
        position_endstop: p.z_endstop,
        retract_speed: 2.0,
        retract_dist: 1.0,
        positive_dir: p.z_pos_dir,
        second_homing_speed: 1.0,
    };
    let arm_rail = Arc::new(FakeRail {
        name: "stepper_arm".to_string(),
        range: p.arm_range,
        homing: arm_homing,
        steppers: vec![arm_s],
        iters: Mutex::new(Vec::new()),
    });
    let z_rail = Arc::new(FakeRail {
        name: "stepper_z".to_string(),
        range: p.z_range,
        homing: z_homing,
        steppers: vec![z_s],
        iters: Mutex::new(Vec::new()),
    });
    let arm_t: Rail = arm_rail.clone();
    let z_t: Rail = z_rail.clone();
    let factory = Arc::new(FakeFactory {
        bed: bed_t,
        arm: arm_t,
        z: z_t,
        calls: Mutex::new(Vec::new()),
    });
    let factory_t: StepperFactory = factory.clone();
    let toolhead = Arc::new(FakeToolHead {
        max_velocity: (p.max_vel, p.max_acc),
        trapq: dummy_trapq(),
        factory: factory_t,
    }) as klippy_api::toolhead::ToolHead;
    let printer: Printer = Arc::new(FakePrinter);
    let floats: IndexMap<String, f64> = p.floats.clone();
    let fake_config = Arc::new(FakeConfig {
        printer,
        name: "kinematics".to_string(),
        floats,
        float_calls: Mutex::new(Vec::new()),
    });
    let config: Config = fake_config.clone();
    let polar = PolarKinematics::new(&toolhead, &config).expect("rust build");
    Harness {
        polar,
        fake_config,
        factory,
        bed,
        arm_rail,
        z_rail,
        arm_stepper,
        z_stepper,
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("polar fakes");
            Some(py)
        }
        None => None,
    }
}

fn py_build(py: &mut pyoracle::Oracle, p: &Params) {
    let mut floats = String::from("{");
    let mut first = true;
    for (k, v) in p.floats.iter() {
        if !first {
            floats.push(',');
        }
        first = false;
        floats.push_str(&format!("{:?}: {}", k, pycompat::text::float_repr(*v)));
    }
    floats.push('}');
    let code = format!(
        "kin, toolhead, config = build({}, {}, {}, {}, {}, {}, {}, {}, {})",
        pycompat::text::float_repr(p.max_vel),
        pycompat::text::float_repr(p.max_acc),
        format!(
            "({}, {})",
            pycompat::text::float_repr(p.arm_range.0),
            pycompat::text::float_repr(p.arm_range.1)
        ),
        format!(
            "({}, {})",
            pycompat::text::float_repr(p.z_range.0),
            pycompat::text::float_repr(p.z_range.1)
        ),
        pycompat::text::float_repr(p.arm_endstop),
        if p.arm_pos_dir { "True" } else { "False" },
        pycompat::text::float_repr(p.z_endstop),
        if p.z_pos_dir { "True" } else { "False" },
        floats
    );
    py.exec(&code).expect("py build");
}

fn vec_float(v: &Vec<f64>) -> String {
    let parts: Vec<String> = v.iter().map(|x| pycompat::text::float_repr(*x)).collect();
    format!("[{}]", parts.join(", "))
}

fn float_calls_value(c: &FakeConfig) -> Value {
    Value::List(
        c.float_calls
            .lock()
            .iter()
            .map(|f| {
                let opt_f = |o: Option<f64>| match o {
                    Some(x) => Value::float(x),
                    None => Value::Null,
                };
                Value::Tuple(vec![
                    Value::Str(f.option.clone()),
                    opt_f(f.default),
                    opt_f(f.minval),
                    opt_f(f.maxval),
                    opt_f(f.above),
                    opt_f(f.below),
                ])
            })
            .collect(),
    )
}

fn factory_calls_value(h: &Harness) -> Value {
    Value::List(
        h.factory
            .calls
            .lock()
            .iter()
            .map(|(k, name, extra)| {
                if k == "PrinterStepper" {
                    Value::Tuple(vec![
                        Value::Str(k.clone()),
                        Value::Str(name.clone()),
                        Value::Bool(extra == "true"),
                    ])
                } else {
                    Value::Tuple(vec![
                        Value::Str(k.clone()),
                        Value::Str(name.clone()),
                        Value::Dict(Vec::new()),
                    ])
                }
            })
            .collect(),
    )
}

fn iters_value(iters: &Mutex<Vec<(String, Vec<PyValue>)>>) -> Value {
    Value::List(
        iters
            .lock()
            .iter()
            .map(|(alloc, params)| {
                Value::Tuple(vec![
                    Value::Str(alloc.clone()),
                    Value::Tuple(params.iter().map(pyvalue_to_value).collect()),
                ])
            })
            .collect(),
    )
}

fn limit_value(m: &FakeMove) -> Value {
    Value::List(
        m.limit_calls
            .lock()
            .iter()
            .map(|(s, a)| Value::Tuple(vec![Value::float(*s), Value::float(*a)]))
            .collect(),
    )
}

fn pos_value(pos: &Mutex<Vec<[f64; 3]>>) -> Value {
    Value::List(
        pos
            .lock()
            .iter()
            .map(|c| {
                Value::Tuple(vec![
                    Value::float(c[0]),
                    Value::float(c[1]),
                    Value::float(c[2]),
                ])
            })
            .collect(),
    )
}


fn opt_list_value(v: &[Option<f64>]) -> Value {
    Value::List(
        v.iter()
            .map(|o| match o {
                Some(x) => Value::float(*x),
                None => Value::Null,
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_wiring_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let mut p = default_params();
    p.floats.insert("max_z_velocity".to_string(), 120.0);
    p.floats.insert("max_angular_velocity".to_string(), 2.0);
    let h = rust_build(&p);
    py_build(&mut py, &p);
    check(
        "factory_calls",
        py.eval("FACTORY_CALLS"),
        factory_calls_value(&h),
    );
    check(
        "getfloat_calls",
        py.eval("config.getfloat_calls"),
        float_calls_value(&h.fake_config),
    );
    // stepper itersolve (bed) and rail itersolve (arm/z)
    check(
        "bed_itersolve",
        py.eval("[s.itersolve_calls for s in kin.steppers if s.get_name() == 'stepper_bed']"),
        Value::List(vec![iters_value(&h.bed.iters)]),
    );
    check(
        "arm_itersolve",
        py.eval("[r.itersolve_calls for r in kin.rails if r.get_name() == 'stepper_arm']"),
        Value::List(vec![iters_value(&h.arm_rail.iters)]),
    );
    check(
        "z_itersolve",
        py.eval("[r.itersolve_calls for r in kin.rails if r.get_name() == 'stepper_z']"),
        Value::List(vec![iters_value(&h.z_rail.iters)]),
    );
    check(
        "trapq_counts",
        py.eval("[len(s.trapqs) for s in kin.steppers]"),
        Value::List(vec![
            Value::int(h.bed.trapqs.lock().len() as i64),
            Value::int(h.arm_stepper.trapqs.lock().len() as i64),
            Value::int(h.z_stepper.trapqs.lock().len() as i64),
        ]),
    );
    check(
        "get_status_init",
        py.eval("kin.get_status(0.0)"),
        pyvalue_to_value(&h.polar.get_status(0.0).unwrap()),
    );
}

#[test]
fn init_defaults_match_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    check(
        "getfloat_defaults",
        py.eval("config.getfloat_calls"),
        float_calls_value(&h.fake_config),
    );
    assert_eq!((h.polar.max_velocity(), h.polar.max_accel()), (250.0, 3000.0));
    assert_eq!((h.polar.max_z_velocity(), h.polar.max_z_accel()), (250.0, 3000.0));
    assert_eq!(h.polar.v_rad_max(), 0.0);
    assert_eq!(h.polar.limit_xy2(), -1.0);
    assert_eq!(h.polar.limit_z(), (1.0, -1.0));
    assert_eq!(h.polar.axes_min(), Coord(vec![-150.0, -150.0, 10.0, 0.0]));
    assert_eq!(h.polar.axes_max(), Coord(vec![150.0, 150.0, 250.0, 0.0]));
}

#[test]
fn module_factory_empty_matches_python() {
    // polar.py has no printer-section entry point; load_kinematics is the entry.
    let factory = kinematics::polar::module_factory();
    assert!(factory.load_config.is_none());
}

fn dummy_toolhead_for_load(p: &Params) -> klippy_api::toolhead::ToolHead {
    let bed = Arc::new(FakeStepper::new("stepper_bed"));
    let arm_s = Arc::new(FakeStepper::new("stepper_arm"));
    let z_s = Arc::new(FakeStepper::new("stepper_z"));
    let bed_t: Stepper = bed.clone();
    let arm_t: Stepper = arm_s.clone();
    let z_t: Stepper = z_s.clone();
    let arm_rail = Arc::new(FakeRail {
        name: "stepper_arm".to_string(),
        range: p.arm_range,
        homing: HomingInfo {
            speed: 5.0,
            position_endstop: p.arm_endstop,
            retract_speed: 2.0,
            retract_dist: 1.0,
            positive_dir: p.arm_pos_dir,
            second_homing_speed: 1.0,
        },
        steppers: vec![arm_t],
        iters: Mutex::new(Vec::new()),
    });
    let z_rail = Arc::new(FakeRail {
        name: "stepper_z".to_string(),
        range: p.z_range,
        homing: HomingInfo {
            speed: 5.0,
            position_endstop: p.z_endstop,
            retract_speed: 2.0,
            retract_dist: 1.0,
            positive_dir: p.z_pos_dir,
            second_homing_speed: 1.0,
        },
        steppers: vec![z_t],
        iters: Mutex::new(Vec::new()),
    });
    let arm_t: Rail = arm_rail.clone();
    let z_t: Rail = z_rail.clone();
    let factory = Arc::new(FakeFactory {
        bed: bed_t,
        arm: arm_t,
        z: z_t,
        calls: Mutex::new(Vec::new()),
    });
    let factory_t: StepperFactory = factory.clone();
    Arc::new(FakeToolHead {
        max_velocity: (p.max_vel, p.max_acc),
        trapq: dummy_trapq(),
        factory: factory_t,
    })
}

fn dummy_config_for_load(p: &Params) -> Config {
    let printer: Printer = Arc::new(FakePrinter);
    Arc::new(FakeConfig {
        printer,
        name: "kinematics".to_string(),
        floats: p.floats.clone(),
        float_calls: Mutex::new(Vec::new()),
    })
}

#[test]
fn calc_position_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    for (label, bed, arm, z) in [
        ("origin", 0.0, 0.0, 0.0),
        ("quarter_turn", std::f64::consts::FRAC_PI_2, 100.0, 10.0),
        ("half_turn", std::f64::consts::PI, 50.0, 200.0),
        ("neg_arm", -0.5, -20.0, 30.0),
        ("large", std::f64::consts::TAU, 150.0, 250.0),
    ] {
        let mut map = IndexMap::new();
        map.insert("stepper_bed".to_string(), bed);
        map.insert("stepper_arm".to_string(), arm);
        map.insert("stepper_z".to_string(), z);
        let expected = py.eval(&format!(
            "kin.calc_position({{'stepper_bed': {}, 'stepper_arm': {}, 'stepper_z': {}}})",
            pycompat::text::float_repr(bed),
            pycompat::text::float_repr(arm),
            pycompat::text::float_repr(z)
        ));
        let actual = h.polar.calc_position(&map).expect("rust calc");
        check(label, expected, opt_list_value(&actual));
    }
}

#[test]
fn calc_position_missing_key_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    // missing bed
    let expected = py.eval("kin.calc_position({'stepper_arm': 10.0, 'stepper_z': 5.0})");
    let mut map = IndexMap::new();
    map.insert("stepper_arm".to_string(), 10.0);
    map.insert("stepper_z".to_string(), 5.0);
    check_err("missing_bed", expected, h.polar.calc_position(&map));
    // missing arm
    let expected = py.eval("kin.calc_position({'stepper_bed': 0.0, 'stepper_z': 5.0})");
    let mut map = IndexMap::new();
    map.insert("stepper_bed".to_string(), 0.0);
    map.insert("stepper_z".to_string(), 5.0);
    check_err("missing_arm", expected, h.polar.calc_position(&map));
    // missing z
    let expected = py.eval("kin.calc_position({'stepper_bed': 0.0, 'stepper_arm': 10.0})");
    let mut map = IndexMap::new();
    map.insert("stepper_bed".to_string(), 0.0);
    map.insert("stepper_arm".to_string(), 10.0);
    check_err("missing_z", expected, h.polar.calc_position(&map));
}

#[test]
fn set_position_and_status_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    for (label, axes) in [("none", ""), ("x", "x"), ("y", "y"), ("xy", "xy"), ("z", "z"), ("xyz", "xyz")] {
        py.exec("for s in kin.steppers: s.positions.clear()")
            .expect("clear py");
        h.bed.positions.lock().clear();
        h.arm_stepper.positions.lock().clear();
        h.z_stepper.positions.lock().clear();
        let pos = vec![10.0, 20.0, 30.0, 0.0];
        let expected = py.eval(&format!(
            "[kin.set_position({}, {:?}), kin.get_status(0.0)][1]",
            vec_float(&pos),
            axes
        ));
        h.polar.set_position(pos, axes).expect("rust set");
        check(label, expected, pyvalue_to_value(&h.polar.get_status(0.0).unwrap()));
        check(
            &format!("{label}_positions"),
            py.eval("[list(s.positions) for s in kin.steppers]"),
            Value::List(vec![
                pos_value(&h.bed.positions),
                pos_value(&h.arm_stepper.positions),
                pos_value(&h.z_stepper.positions),
            ]),
        );
    }
}

#[test]
fn set_position_short_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    let expected = py.eval("kin.set_position([10.0, 20.0], 'xy')");
    check_err("short_xy", expected, h.polar.set_position(vec![10.0, 20.0], "xy"));
    let expected = py.eval("kin.set_position([], 'z')");
    check_err("empty_z", expected, h.polar.set_position(Vec::new(), "z"));
}

#[test]
fn clear_homing_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    py.exec("kin.set_position([1.0, 1.0, 1.0, 0.0], 'xyz')").expect("py set");
    h.polar
        .set_position(vec![1.0, 1.0, 1.0, 0.0], "xyz")
        .expect("rust set");
    for (label, axes) in [("x", "x"), ("y", "y"), ("z", "z"), ("xy", "xy"), ("", "")] {
        // reset to homed on both sides before each clear probe
        py.exec("kin.set_position([1.0, 1.0, 1.0, 0.0], 'xyz')").expect("py reset");
        h.polar
            .set_position(vec![1.0, 1.0, 1.0, 0.0], "xyz")
            .expect("rust reset");
        let expected = py.eval(&format!(
            "[kin.clear_homing_state({:?}), kin.get_status(0.0)][1]",
            axes
        ));
        h.polar.clear_homing_state(axes).expect("clear_homing_state");
        check(label, expected, pyvalue_to_value(&h.polar.get_status(0.0).unwrap()));
    }
}

#[test]
fn home_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    for (label, axes) in [
        ("xy", "[0, 1]"),
        ("z", "[2]"),
        ("xyz", "[0, 1, 2]"),
        ("xonly", "[0]"),
        ("yonly", "[1]"),
        ("none", "[]"),
    ] {
        py.exec("hs = FakeHomingState([])").expect("py hs create");
        py.exec("kin.set_position([1.0, 1.0, 1.0, 0.0], 'xyz')")
            .expect("py reset");
        h.polar
            .set_position(vec![1.0, 1.0, 1.0, 0.0], "xyz")
            .expect("rust reset");
        // set axes on both sides to the probe set
        let expected = py.eval(&format!(
            "[hs.set_axes({}), kin.home(hs), (hs.axes, hs.home_rails_calls, kin.get_status(0.0))][2]",
            axes
        ));
        let hs = Arc::new(FakeHomingState {
            axes: Mutex::new(match label {
                "xy" => vec![0, 1],
                "z" => vec![2],
                "xyz" => vec![0, 1, 2],
                "xonly" => vec![0],
                "yonly" => vec![1],
                _ => Vec::new(),
            }),
            set_axes_calls: Mutex::new(Vec::new()),
            home_calls: Mutex::new(Vec::new()),
        });
        let hs_t: HomingStateHandle = hs.clone();
        h.polar.home(hs_t).expect("rust home");
        let home_value = Value::Tuple(vec![
            Value::List(hs.axes.lock().iter().map(|a| Value::int(*a)).collect()),
            Value::List(
                hs.home_calls
                    .lock()
                    .iter()
                    .map(|(names, force, movepos)| {
                        Value::Tuple(vec![
                            Value::Str(names.join(" ")),
                            opt_list_value(force),
                            opt_list_value(movepos),
                        ])
                    })
                    .collect(),
            ),
            pyvalue_to_value(&h.polar.get_status(0.0).unwrap()),
        ]);
        check(label, expected, home_value);
    }
}

#[test]
fn home_negative_dir_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let mut p = default_params();
    p.arm_pos_dir = false;
    p.z_pos_dir = true;
    p.arm_endstop = 100.0;
    p.z_endstop = 50.0;
    let h = rust_build(&p);
    py_build(&mut py, &p);
    py.exec("hs = FakeHomingState([])").expect("py hs");
    let expected = py.eval(
        "[hs.set_axes([0, 1, 2]), kin.home(hs), (hs.axes, hs.home_rails_calls)][2]",
    );
    let hs = Arc::new(FakeHomingState {
        axes: Mutex::new(vec![0, 1, 2]),
        set_axes_calls: Mutex::new(Vec::new()),
        home_calls: Mutex::new(Vec::new()),
    });
    let hs_t: HomingStateHandle = hs.clone();
    h.polar.home(hs_t).expect("rust home");
    let value = Value::Tuple(vec![
        Value::List(hs.axes.lock().iter().map(|a| Value::int(*a)).collect()),
        Value::List(
            hs.home_calls
                .lock()
                .iter()
                .map(|(names, force, movepos)| {
                    Value::Tuple(vec![
                        Value::Str(names.join(" ")),
                        opt_list_value(force),
                        opt_list_value(movepos),
                    ])
                })
                .collect(),
        ),
    ]);
    check("neg_dir", expected, value);
}

#[test]
fn check_move_xy_limits_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    // unhomed: any xy outside 0 radius errors with "Must home axis first"
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (10.0, 0.0, 10.0, 0.0), (10.0, 0.0, 0.0, 0.0), 10.0, 100.0)",
    )
    .expect("py move unhomed");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![10.0, 0.0, 10.0, 0.0],
        vec![10.0, 0.0, 0.0, 0.0],
        10.0,
        100.0,
    );
    check_err("unhomed_xy", expected, h.polar.check_move(&mv));
    // homed but outside arm range -> default move_error
    py.exec("kin.set_position([0.0, 0.0, 10.0, 0.0], 'xyz')").expect("py home");
    h.polar
        .set_position(vec![0.0, 0.0, 10.0, 0.0], "xyz")
        .expect("rust home");
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (200.0, 0.0, 10.0, 0.0), (200.0, 0.0, 0.0, 0.0), 200.0, 100.0)",
    )
    .expect("py move out");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![200.0, 0.0, 10.0, 0.0],
        vec![200.0, 0.0, 0.0, 0.0],
        200.0,
        100.0,
    );
    check_err("out_of_range_xy", expected, h.polar.check_move(&mv));
    // inside range, no Z, no angular limit -> ok, no limit calls
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (10.0, 0.0, 10.0, 0.0), (10.0, 0.0, 0.0, 0.0), 10.0, 100.0)",
    )
    .expect("py move in");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![10.0, 0.0, 10.0, 0.0],
        vec![10.0, 0.0, 0.0, 0.0],
        10.0,
        100.0,
    );
    h.polar.check_move(&mv).expect("rust move in");
    check("in_range_ok", expected, Value::Null);
    check(
        "in_range_limits",
        py.eval("m.limit_calls"),
        limit_value(&mv),
    );
}

#[test]
fn check_move_z_limits_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let mut p = default_params();
    p.floats.insert("max_z_velocity".to_string(), 80.0);
    p.floats.insert("max_z_accel".to_string(), 900.0);
    let h = rust_build(&p);
    py_build(&mut py, &p);
    // unhomed Z
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (0.0, 0.0, 20.0, 0.0), (0.0, 0.0, 10.0, 0.0), 10.0, 100.0)",
    )
    .expect("py z unhomed");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![0.0, 0.0, 20.0, 0.0],
        vec![0.0, 0.0, 10.0, 0.0],
        10.0,
        100.0,
    );
    check_err("unhomed_z", expected, h.polar.check_move(&mv));
    // homed Z, inside range -> Z speed limiting
    py.exec("kin.set_position([0.0, 0.0, 10.0, 0.0], 'xyz')").expect("py home");
    h.polar
        .set_position(vec![0.0, 0.0, 10.0, 0.0], "xyz")
        .expect("rust home");
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (0.0, 0.0, 20.0, 0.0), (0.0, 0.0, 10.0, 0.0), 10.0, 100.0)",
    )
    .expect("py z in");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![0.0, 0.0, 20.0, 0.0],
        vec![0.0, 0.0, 10.0, 0.0],
        10.0,
        100.0,
    );
    h.polar.check_move(&mv).expect("rust z in");
    check("z_in_ok", expected, Value::Null);
    check("z_limits", py.eval("m.limit_calls"), limit_value(&mv));
    // homed Z, outside range -> default error
    py.exec(
        "m = FakeMove((0.0, 0.0, 10.0, 0.0), (0.0, 0.0, 500.0, 0.0), (0.0, 0.0, 490.0, 0.0), 490.0, 100.0)",
    )
    .expect("py z out");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![0.0, 0.0, 10.0, 0.0],
        vec![0.0, 0.0, 500.0, 0.0],
        vec![0.0, 0.0, 490.0, 0.0],
        490.0,
        100.0,
    );
    check_err("z_out", expected, h.polar.check_move(&mv));
}

#[test]
fn check_move_angular_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let mut p = default_params();
    p.floats.insert("max_angular_velocity".to_string(), 1.0);
    let h = rust_build(&p);
    py_build(&mut py, &p);
    py.exec("kin.set_position([0.0, 0.0, 10.0, 0.0], 'xyz')").expect("py home");
    h.polar
        .set_position(vec![0.0, 0.0, 10.0, 0.0], "xyz")
        .expect("rust home");
    // shallow path close to centre -> angular limiting
    py.exec(
        "m = FakeMove((10.0, 1.0, 10.0, 0.0), (10.0, -1.0, 10.0, 0.0), (0.0, -2.0, 0.0, 0.0), 2.0, 100.0)",
    )
    .expect("py angular");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![10.0, 1.0, 10.0, 0.0],
        vec![10.0, -1.0, 10.0, 0.0],
        vec![0.0, -2.0, 0.0, 0.0],
        2.0,
        100.0,
    );
    h.polar.check_move(&mv).expect("rust angular");
    check("angular_ok", expected, Value::Null);
    check("angular_limits", py.eval("m.limit_calls"), limit_value(&mv));
    // path through centre -> min_dist 0, no limiting
    py.exec(
        "m = FakeMove((10.0, 0.0, 10.0, 0.0), (-10.0, 0.0, 10.0, 0.0), (-20.0, 0.0, 0.0, 0.0), 20.0, 100.0)",
    )
    .expect("py centre");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![10.0, 0.0, 10.0, 0.0],
        vec![-10.0, 0.0, 10.0, 0.0],
        vec![-20.0, 0.0, 0.0, 0.0],
        20.0,
        100.0,
    );
    h.polar.check_move(&mv).expect("rust centre");
    check("centre_ok", expected, Value::Null);
    check("centre_limits", py.eval("m.limit_calls"), limit_value(&mv));
    // max_cruise_v2 negative -> math domain error on both sides
    py.exec(
        "m = FakeMove((10.0, 1.0, 10.0, 0.0), (10.0, -1.0, 10.0, 0.0), (0.0, -2.0, 0.0, 0.0), 2.0, -4.0)",
    )
    .expect("py sqrt err");
    let expected = py.eval("kin.check_move(m)");
    let mv = FakeMove::new(
        vec![10.0, 1.0, 10.0, 0.0],
        vec![10.0, -1.0, 10.0, 0.0],
        vec![0.0, -2.0, 0.0, 0.0],
        2.0,
        -4.0,
    );
    check_err("sqrt_domain", expected, h.polar.check_move(&mv));
}

#[test]
fn distance_to_center_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    for (label, p1, p2) in [
        ("same", vec![3.0, 4.0], vec![3.0, 4.0]),
        ("beyond", vec![10.0, 0.0], vec![20.0, 0.0]),
        ("before", vec![20.0, 0.0], vec![10.0, 0.0]),
        ("mid", vec![10.0, 1.0], vec![10.0, -1.0]),
        ("diag", vec![1.0, 2.0], vec![5.0, 6.0]),
        ("origin", vec![0.0, 0.0], vec![0.0, 0.0]),
    ] {
        let expected = py.eval(&format!(
            "polar.distance_to_center({}, {})",
            vec_float(&p1),
            vec_float(&p2)
        ));
        let actual = distance_to_center(&p1, &p2).expect("rust dist");
        check(label, expected, Value::float(actual));
    }
}

#[test]
fn get_steppers_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    let h = rust_build(&p);
    py_build(&mut py, &p);
    check(
        "stepper_names",
        py.eval("[s.get_name() for s in kin.get_steppers()]"),
        Value::List(vec![
            Value::Str("stepper_bed".to_string()),
            Value::Str("stepper_arm".to_string()),
            Value::Str("stepper_z".to_string()),
        ]),
    );
    assert_eq!(h.polar.get_steppers().len(), 3);
}

#[test]
fn load_kinematics_direct_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping pyoracle");
        return;
    };
    let p = default_params();
    py_build(&mut py, &p);
    let expected = py.eval("polar.load_kinematics(toolhead, config).get_status(0.0)");
    let toolhead = dummy_toolhead_for_load(&p);
    let config = dummy_config_for_load(&p);
    let kin: Kinematics = load_kinematics(&toolhead, &config).expect("rust load");
    check(
        "load_status",
        expected,
        pyvalue_to_value(&kin.get_status(0.0).unwrap()),
    );
    let factory_fn: LoadKinematics = load_kinematics;
    let kin2: Kinematics = factory_fn(&toolhead, &config).expect("rust load via fn");
    check(
        "load_via_fn_status",
        py.eval("kin.get_status(0.0)"),
        pyvalue_to_value(&kin2.get_status(0.0).unwrap()),
    );
}
