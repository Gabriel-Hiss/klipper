//! Parity tests for `extras::manual_stepper` (`klippy/extras/manual_stepper.py`).

use extras::manual_stepper::{load_config_prefix, module_factory, calc_move_time, ManualStepper};
use extras::stepper_enable::PrinterStepperEnableApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetFloatListOpts, GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfig, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GCmd,
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts,
    MoveTransform, RegisterCommandOpts,
};
use klippy_api::mcu::{Endstop, EndstopApi, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper,
    StepperFactoryApi, StepperKinematics, Trapq,
};
use klippy_api::toolhead::{
    Coord, ExtraAxis, ExtraAxisApi, FlushCallback, HomingStateHandle, HomingTarget,
    HomingTargetApi, Kinematics, KinematicsApi, LookaheadCallback, MotionQueuingApi,
    MoveApi, PrinterExtruder, PrinterHomingApi, SyncEmitterHandle, ToolHead, ToolHeadApi,
    TrapqAppend,
};
use klippy_api::{EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import sys, types, math
# stub stepper (klippy/stepper.py imports chelper)
stepper_stub = types.ModuleType("stepper")
class FakeStepper:
    def __init__(self, name="stepper_x"):
        self._name = name
        self.Positions = []
    def get_name(self, short=False):
        return self._name
    def set_position(self, coord):
        logs.append(("stepper.set_position", tuple(coord)))
        self.Positions.append(tuple(coord))
    def setup_itersolve(self, alloc, *params):
        logs.append(("stepper.setup_itersolve", alloc, tuple(params)))
    def set_trapq(self, trapq):
        logs.append(("stepper.set_trapq", "trapq"))
class FakeRail:
    def __init__(self, config=None):
        self._name = "rail_x"
        self.steppers = [FakeStepper("stepper_x")]
        self.endstops = [("es1", "rail_x")]
    def get_name(self, short=False):
        return self._name
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def set_position(self, coord):
        logs.append(("rail.set_position", tuple(coord)))
    def setup_itersolve(self, alloc, *params):
        logs.append(("rail.setup_itersolve", alloc, tuple(params)))
    def set_trapq(self, trapq):
        logs.append(("rail.set_trapq", "trapq"))
def _lookup_rail(config, **kw):
    logs.append(("lookup_rail", kw.get("need_position_minmax"), kw.get("default_position_endstop"), kw.get("units_in_radians", False)))
    return FakeRail(config)
def _printer_stepper(config):
    logs.append(("printer_stepper",))
    return FakeStepper("stepper_x")
stepper_stub.LookupRail = _lookup_rail
stepper_stub.PrinterStepper = _printer_stepper
sys.modules["stepper"] = stepper_stub
# stub force_move with the real calc_move_time body
fm = types.ModuleType("extras.force_move")
def calc_move_time(dist, speed, accel):
    axis_r = 1.
    if dist < 0.:
        axis_r = -1.
        dist = -dist
    if not accel or not dist:
        return axis_r, 0., dist / speed, speed
    max_cruise_v2 = dist * accel
    if max_cruise_v2 < speed**2:
        speed = math.sqrt(max_cruise_v2)
    accel_t = speed / accel
    accel_decel_d = accel_t * speed
    cruise_t = (dist - accel_decel_d) / speed
    return axis_r, accel_t, cruise_t, speed
fm.calc_move_time = calc_move_time
sys.modules["extras.force_move"] = fm
import extras as _extras_pkg
_extras_pkg.force_move = fm
import importlib
logs = []
ms_mod = importlib.import_module("extras.manual_stepper")
ManualStepper = ms_mod.ManualStepper

class CmdErr(Exception): pass
_sentinel = object()
class FakePrinter:
    def __init__(self, objs=None):
        self.objects = dict(objs or {})
    def load_object(self, config, name):
        logs.append(("load_object", name))
        return self.objects[name]
    def lookup_object(self, name, default=None):
        return self.objects.get(name)
    def command_error(self, msg):
        return CmdErr(msg)
    def config_error(self, msg):
        return CmdErr(msg)
class FakeConfig:
    def __init__(self, printer, name, opts=None):
        self.printer = printer
        self.name = name
        self.opts = dict(opts or {})
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def get(self, opt, default=_sentinel, note_valid=True):
        if opt not in self.opts:
            if default is _sentinel:
                raise CmdErr("Option '%s' in section '%s' must be specified" % (opt, self.name))
            return default
        return self.opts[opt]
    def getfloat(self, opt, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if opt not in self.opts:
            if default is _sentinel:
                raise CmdErr("Option '%s' in section '%s' must be specified" % (opt, self.name))
            return default
        v = float(self.opts[opt])
        if minval is not None and v < minval:
            raise CmdErr("Option '%s' in section '%s' must have minimum of %s" % (opt, self.name, minval))
        if above is not None and v <= above:
            raise CmdErr("Option '%s' in section '%s' must be above %s" % (opt, self.name, above))
        return v
class FakeToolhead:
    def __init__(self):
        self.last = 10.0
        self.extra_axes = []
    def get_last_move_time(self):
        logs.append(("toolhead.get_last_move_time",))
        return self.last
    def dwell(self, d):
        logs.append(("toolhead.dwell", d))
        self.last += d
    def flush_step_generation(self):
        logs.append(("toolhead.flush_step_generation",))
    def add_extra_axis(self, ea, pos):
        logs.append(("toolhead.add_extra_axis", pos))
        self.extra_axes.append(ea)
    def remove_extra_axis(self, ea):
        logs.append(("toolhead.remove_extra_axis",))
        self.extra_axes.remove(ea)
    def get_extra_axes(self):
        return list(self.extra_axes)
class FakeMQ:
    def __init__(self):
        self.n = 0
    def allocate_trapq(self):
        self.n += 1
        logs.append(("allocate_trapq", self.n))
        return ("trapq", self.n)
    def lookup_trapq_append(self):
        def append(trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, sv, cv, accel):
            logs.append(("trapq_append", print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, sv, cv, accel))
        return append
    def note_mcu_movequeue_activity(self, t, is_step_gen=True):
        logs.append(("note_mcu_movequeue_activity", t, is_step_gen))
    def drip_update_time(self, s, e, c):
        logs.append(("drip_update_time", s, e, c))
    def wipe_trapq(self, trapq):
        logs.append(("wipe_trapq", "trapq"))
class FakeStepperEnable:
    def set_motors_enable(self, names, enable):
        logs.append(("set_motors_enable", tuple(names), enable))
        return True
class FakeHoming:
    def manual_home(self, toolhead, endstops, pos, speed, probe_pos, triggered, check_triggered):
        logs.append(("manual_home", tuple(pos), speed, probe_pos, triggered, check_triggered, tuple(str(e) for e in endstops)))
        return pos
class FakeConfigFile:
    def deprecate_gcode(self, cmd, param=None, value=None, msg=None):
        logs.append(("deprecate_gcode", cmd, param, str(value)))
class FakeGCode:
    def register_mux_command(self, cmd, key, value, func, desc=None):
        logs.append(("register_mux_command", cmd, key, value, desc))
class FakeGcmd:
    error = CmdErr
    def __init__(self, params=None, commandline="MANUAL_STEPPER"):
        self.params = dict(params or {})
        self._commandline = commandline
    def get_commandline(self):
        return self._commandline
    def get(self, name, default=_sentinel, parser=str, minval=None, maxval=None, above=None, below=None):
        if name not in self.params:
            if default is _sentinel:
                raise CmdErr("Error on '%s': missing %s" % (self._commandline, name))
            return default
        v = parser(self.params[name])
        return v
    def get_int(self, name, default=_sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)
    def get_float(self, name, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        def parse(x):
            return float(x)
        if name not in self.params:
            if default is _sentinel:
                raise CmdErr("Error on '%s': missing %s" % (self._commandline, name))
            return default
        v = float(self.params[name])
        if minval is not None and v < minval:
            raise CmdErr("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if above is not None and v <= above:
            raise CmdErr("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        return v
class FakeMove:
    def __init__(self, start, end, axes_d, axes_r, move_d, max_cruise_v2=100.0, is_kin=True, accel=1000.0, start_v=0.0, cruise_v=10.0, accel_t=0.1, cruise_t=0.2, decel_t=0.1):
        self.start_pos = tuple(start); self.end_pos = tuple(end)
        self.axes_d = tuple(axes_d); self.axes_r = tuple(axes_r)
        self.move_d = move_d; self.max_cruise_v2 = max_cruise_v2
        self.is_kinematic_move = is_kin; self.accel = accel
        self.start_v = start_v; self.cruise_v = cruise_v
        self.accel_t = accel_t; self.cruise_t = cruise_t; self.decel_t = decel_t
    def limit_speed(self, speed, accel):
        logs.append(("limit_speed", speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        return CmdErr("%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3]))

def make_ms(opts=None, name="manual_stepper foo"):
    global logs
    logs = []
    printer = FakePrinter()
    toolhead = FakeToolhead(); mq = FakeMQ()
    se = FakeStepperEnable(); homing = FakeHoming(); cfile = FakeConfigFile(); gcode = FakeGCode()
    printer.objects = {"toolhead": toolhead, "motion_queuing": mq, "stepper_enable": se,
                       "homing": homing, "configfile": cfile, "gcode": gcode, "gcode_move": object()}
    base = {"velocity": 7.0, "accel": 500.0}
    if opts: base.update(opts)
    config = FakeConfig(printer, name, base)
    ms = ManualStepper(config)
    return (ms, printer, toolhead, mq)
def state_of(ms):
    return (ms.commanded_pos, ms.next_cmd_time, ms.axis_gcode_id, ms.velocity, ms.accel, ms.can_home, ms.pos_min, ms.pos_max)
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("manual_stepper fakes");
            Some(py)
        }
        None => {
            println!("skipping manual_stepper oracle tests: set KLIPPY_PYTHON to run against the reference");
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.manual_stepper".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let oracle_msg = match expected {
        Ok(v) => panic!("{label}: expected oracle error, got {v:?}"),
        Err(e) => e.message,
    };
    match actual {
        Ok(v) => panic!("{label}: expected error, got {v:?}"),
        Err(e) => {
            assert_eq!(e.message(), oracle_msg.as_str(), "{label}: error message");
            assert_eq!(e.class_name(), "CommandError", "{label}: error class");
        }
    }
}

type Log = Arc<Mutex<Vec<Value>>>;

fn log_entry(log: &Log, vals: Vec<Value>) {
    log.lock().push(Value::Tuple(vals));
}
fn s(v: &str) -> Value {
    Value::str(v)
}
fn f(v: f64) -> Value {
    Value::float(v)
}
fn i(v: i64) -> Value {
    Value::int(v)
}

struct FakeStepper {
    name: String,
    log: Log,
}
impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("unused")
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
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, alloc: &str, params: &[PyValue]) {
        let p = match params.first() {
            Some(PyValue::Bytes(b)) => Value::Tuple(vec![Value::bytes(b)]),
            _ => Value::Null,
        };
        log_entry(&self.log, vec![s("stepper.setup_itersolve"), s(alloc), p]);
    }
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _b: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, coord: [f64; 3]) {
        log_entry(&self.log, vec![s("stepper.set_position"), Value::Tuple(vec![f(coord[0]), f(coord[1]), f(coord[2])])]);
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _p: Option<f64>) -> i64 {
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
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        log_entry(&self.log, vec![s("stepper.set_trapq"), s("trapq")]);
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _a: char) -> bool {
        false
    }
}

struct FakeEndstop {
    name: String,
}
impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("unused")
    }
    fn add_stepper(&self, _s: Stepper) {}
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn home_start(
        &self,
        _t: f64,
        _s: f64,
        _c: i64,
        _r: f64,
        _tr: Option<bool>,
    ) -> Completion<bool> {
        Reactor::new(false).completion()
    }
    fn home_wait(&self, _t: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn query_endstop(&self, _t: f64) -> KResult<i64> {
        Ok(0)
    }
}

struct FakeRail {
    name: String,
    steppers: Vec<Stepper>,
    endstops: Vec<(Endstop, String)>,
    log: Log,
}
impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: 0.0,
            position_endstop: 0.0,
            retract_speed: 0.0,
            retract_dist: 0.0,
            positive_dir: false,
            second_homing_speed: 0.0,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn get_endstops(&self) -> Vec<(Endstop, String)> {
        self.endstops.clone()
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
        let p = match params.first() {
            Some(PyValue::Bytes(b)) => Value::Tuple(vec![Value::bytes(b)]),
            _ => Value::Null,
        };
        log_entry(&self.log, vec![s("rail.setup_itersolve"), s(alloc), p]);
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {
        log_entry(&self.log, vec![s("rail.set_trapq"), s("trapq")]);
    }
    fn set_position(&self, coord: [f64; 3]) {
        log_entry(&self.log, vec![s("rail.set_position"), Value::Tuple(vec![f(coord[0]), f(coord[1]), f(coord[2])])]);
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeFactory {
    rail: Rail,
    stepper: Stepper,
    log: Log,
}
impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units: bool) -> KResult<Stepper> {
        log_entry(&self.log, vec![s("printer_stepper")]);
        Ok(Arc::clone(&self.stepper))
    }
    fn lookup_rail(&self, _config: &Config, opts: RailOpts) -> KResult<Rail> {
        log_entry(
            &self.log,
            vec![
                s("lookup_rail"),
                Value::Bool(opts.need_position_minmax),
                match opts.default_position_endstop {
                    Some(v) => f(v),
                    None => Value::Null,
                },
                Value::Bool(opts.units_in_radians),
            ],
        );
        Ok(Arc::clone(&self.rail))
    }
    fn lookup_multi_rail(&self, _c: &Config, _o: RailOpts) -> KResult<Rail> {
        panic!("unused")
    }
    fn generic_rail(&self, _c: &Config, _o: RailOpts) -> KResult<Rail> {
        panic!("unused")
    }
    fn parse_step_distance(
        &self,
        _c: &Config,
        _u: Option<bool>,
        _n: bool,
    ) -> KResult<(f64, f64)> {
        panic!("unused")
    }
}

struct FakeMQ {
    log: Log,
    n: Mutex<i64>,
}
impl MotionQueuingApi for FakeMQ {
    fn allocate_trapq(&self) -> Trapq {
        let mut n = self.n.lock();
        *n += 1;
        log_entry(&self.log, vec![s("allocate_trapq"), i(*n)]);
        klippy_core::motion_handles::TrapqHandle::alloc()
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        let log = Arc::clone(&self.log);
        Arc::new(move |_t, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, sv, cv, accel| {
            log_entry(
                &log,
                vec![
                    s("trapq_append"), f(print_time), f(accel_t), f(cruise_t), f(decel_t),
                    f(x), f(y), f(z), f(rx), f(ry), f(rz), f(sv), f(cv), f(accel),
                ],
            );
        })
    }
    fn wipe_trapq(&self, _trapq: &Trapq) {
        log_entry(&self.log, vec![s("wipe_trapq"), s("trapq")]);
    }
    fn allocate_syncemitter(&self, _m: Mcu, _n: &str, _a: bool) -> KResult<SyncEmitterHandle> {
        panic!("unused")
    }
    fn queue_syncemitter_msg(&self, _se: &SyncEmitterHandle, _r: u64, _d: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: Mcu, _q: klippy_api::mcu::CommandQueue, _n: i64) {}
    fn register_flush_callback(&self, _cb: FlushCallback, _a: bool) {}
    fn unregister_flush_callback(&self, _cb: &FlushCallback) {}
    fn get_kin_flush_delay(&self) -> f64 {
        0.0
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, _t: f64) -> f64 {
        0.0
    }
    fn note_mcu_movequeue_activity(&self, t: f64, is_step_gen: bool) {
        log_entry(&self.log, vec![s("note_mcu_movequeue_activity"), f(t), Value::Bool(is_step_gen)]);
    }
    fn drip_update_time(&self, start: f64, end: f64, _drip: &Completion<bool>) -> KResult<()> {
        log_entry(&self.log, vec![s("drip_update_time"), f(start), f(end), s("completion")]);
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}
klippy_api::printer_object!(FakeMQ; dyn MotionQueuingApi);

struct FakeStepperEnable {
    log: Log,
}
impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _c: &Config, _s: &Stepper) -> KResult<()> {
        Ok(())
    }
    fn set_motors_enable(&self, names: &[String], enable: bool) -> KResult<bool> {
        // Python passes the g-code int through; record it as the int
        // so the oracle comparison is type-equal.
        log_entry(
            &self.log,
            vec![
                s("set_motors_enable"),
                Value::Tuple(names.iter().map(|n| s(n)).collect()),
                i(if enable { 1 } else { 0 }),
            ],
        );
        Ok(true)
    }
    fn motor_off(&self) -> KResult<()> {
        Ok(())
    }
    fn lookup_enable(&self, _n: &str) -> KResult<Arc<extras::stepper_enable::EnableTracking>> {
        panic!("unused")
    }
    fn get_steppers(&self) -> Vec<String> {
        Vec::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
}
klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

struct FakeHoming {
    log: Log,
}
impl PrinterHomingApi for FakeHoming {
    fn manual_home(
        &self,
        _toolhead: HomingTarget,
        endstops: Vec<(Endstop, String)>,
        pos: Vec<Option<f64>>,
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<()> {
        let floats: Vec<Value> = pos
            .iter()
            .map(|v| match v {
                Some(x) => f(*x),
                None => Value::Null,
            })
            .collect();
        log_entry(
            &self.log,
            vec![
                s("manual_home"),
                Value::Tuple(floats),
                f(speed),
                Value::Bool(probe_pos),
                Value::Bool(triggered),
                Value::Bool(check_triggered),
                i(endstops.len() as i64),
            ],
        );
        Ok(())
    }
    fn probing_move(
        &self,
        _m: Endstop,
        _p: Vec<Option<f64>>,
        _s: f64,
        _c: bool,
    ) -> KResult<Vec<f64>> {
        panic!("unused")
    }
    fn check_probe_first_home(&self, _g: &GCmd) -> KResult<bool> {
        panic!("unused")
    }
}
klippy_api::printer_object!(FakeHoming; dyn PrinterHomingApi);

struct FakeConfigFile {
    log: Log,
}
impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("unused")
    }
    fn read_config(&self, _f: &str) -> KResult<Config> {
        panic!("unused")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("unused")
    }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _m: &str) {}
    fn deprecate(&self, _s: &str, _o: &str, _v: Option<PyValue>, _m: Option<String>) {}
    fn deprecate_gcode(&self, cmd: &str, opts: DeprecateGcodeOpts) {
        let value = opts.value.map(|v| v.str()).unwrap_or_else(|| "None".to_string());
        log_entry(
            &self.log,
            vec![
                s("deprecate_gcode"),
                s(cmd),
                s(opts.param.as_deref().unwrap_or("None")),
                s(&value),
            ],
        );
    }
    fn deprecate_mcu_code(&self, _m: &Mcu, _f: &str, _m2: Option<String>) {}
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, _s: &str, _o: &str, _v: &str) {}
    fn remove_section(&self, _s: &str) {}
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

struct FakeGCode {
    log: Log,
}
impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        log_entry(
            &self.log,
            vec![
                s("register_mux_command"),
                s(cmd),
                s(key),
                s(value.unwrap_or("None")),
                s(desc.unwrap_or("None")),
            ],
        );
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _cb: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("unused")
    }
    fn create_gcode_command(
        &self,
        _c: &str,
        _l: &str,
        _p: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("unused")
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCodeMove {
    log: Log,
}
impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set_move_transform(
        &self,
        _t: MoveTransform,
        _f: bool,
    ) -> KResult<Option<MoveTransform>> {
        panic!("unused")
    }
    fn reset_last_position(&self) {}
    fn cmd_g1(&self, _g: &GCmd) -> KResult<()> {
        panic!("unused")
    }
}
klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeToolhead {
    log: Log,
    last: Mutex<f64>,
    extra_axes: Mutex<Vec<Option<ExtraAxis>>>,
    factory: Arc<FakeFactory>,
}
impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _p: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        log_entry(&self.log, vec![s("toolhead.flush_step_generation")]);
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        log_entry(&self.log, vec![s("toolhead.get_last_move_time")]);
        *self.last.lock()
    }
    fn dwell(&self, delay: f64) {
        log_entry(&self.log, vec![s("toolhead.dwell"), f(delay)]);
        *self.last.lock() += delay;
    }
    fn drip_move(
        &self,
        _p: Vec<f64>,
        _s: f64,
        _d: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}
impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _p: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        klippy_core::motion_handles::TrapqHandle::alloc()
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _a: Option<f64>,
        _b: Option<f64>,
        _c: Option<f64>,
        _d: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _cb: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("unused")
    }
    fn add_extra_axis(&self, ea: ExtraAxis, axis_pos: f64) {
        log_entry(&self.log, vec![s("toolhead.add_extra_axis"), f(axis_pos)]);
        self.extra_axes.lock().push(Some(ea));
    }
    fn remove_extra_axis(&self, ea: &ExtraAxis) {
        log_entry(&self.log, vec![s("toolhead.remove_extra_axis")]);
        let mut v = self.extra_axes.lock();
        v.retain(|x| match x {
            Some(h) => !Arc::ptr_eq(h, ea),
            None => true,
        });
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        self.extra_axes.lock().clone()
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        Arc::clone(&self.factory) as klippy_api::stepper::StepperFactory
    }
}
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    log: Log,
}
impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> reactor::Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let map = self.objects.lock();
        match map.get(name) {
            Some(o) => Ok(Some(Arc::clone(o))),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown printer object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _c: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        log_entry(&self.log, vec![s("load_object"), s(section)]);
        let map = self.objects.lock();
        match map.get(section) {
            Some(o) => Ok(Some(Arc::clone(o))),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown printer object '{section}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _cb: EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _r: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    opts: IndexMap<String, f64>,
    str_opts: IndexMap<String, String>,
}
impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.str_opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("unused")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opts.get(option) {
            Some(v) => {
                if let Some(min) = opts.minval {
                    if *v < min {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.name,
                            pycompat::text::float_repr(min)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if *v <= above {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.name,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                Ok(Some(*v))
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("unused")
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("unused")
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("unused")
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("unused")
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("unused")
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("unused")
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("unused")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _p: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) {}
}

struct FakeGcmd {
    params: IndexMap<String, String>,
    commandline: String,
}
impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => {
                let parsed = match opts.parser {
                    klippy_api::gcode::GetParser::Str => PyValue::Str(v.clone()),
                    klippy_api::gcode::GetParser::Int => {
                        PyValue::Int(pycompat::PyInt::from_i64(v.parse::<i64>().unwrap_or(0)))
                    }
                    klippy_api::gcode::GetParser::IntAutoBase => PyValue::Int(
                        pycompat::PyInt::from_i64(v.parse::<i64>().unwrap_or(0)),
                    ),
                    klippy_api::gcode::GetParser::Float => {
                        PyValue::Float(v.parse::<f64>().unwrap_or(0.0))
                    }
                };
                Ok(parsed)
            }
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            Some(v) => {
                let parsed: i64 = v.parse().map_err(|_| {
                    KlippyError::Command(format!(
                        "Error on '{}': unable to parse {v}",
                        self.commandline
                    ))
                })?;
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            Some(v) => {
                let parsed: f64 = v.parse().map_err(|_| {
                    KlippyError::Command(format!(
                        "Error on '{}': unable to parse {v}",
                        self.commandline
                    ))
                })?;
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must have minimum of {}",
                            self.commandline,
                            pycompat::text::float_repr(min)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if parsed <= above {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must be above {}",
                            self.commandline,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn get_command(&self) -> String {
        "MANUAL_STEPPER".to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool {
        false
    }
}

struct FakeMove {
    start: Vec<f64>,
    end: Vec<f64>,
    axes_d: Vec<f64>,
    axes_r: Vec<f64>,
    move_d: f64,
    max_cruise_v2: f64,
    is_kin: bool,
    accel_v: f64,
    start_v: f64,
    cruise_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    log: Log,
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
        Coord(self.axes_r.clone())
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn max_cruise_v2(&self) -> f64 {
        self.max_cruise_v2
    }
    fn is_kinematic_move(&self) -> bool {
        self.is_kin
    }
    fn accel(&self) -> f64 {
        self.accel_v
    }
    fn start_v(&self) -> f64 {
        self.start_v
    }
    fn cruise_v(&self) -> f64 {
        self.cruise_v
    }
    fn accel_t(&self) -> f64 {
        self.accel_t
    }
    fn cruise_t(&self) -> f64 {
        self.cruise_t
    }
    fn decel_t(&self) -> f64 {
        self.decel_t
    }
    fn limit_speed(&self, speed: f64, accel: f64) {
        log_entry(&self.log, vec![s("limit_speed"), f(speed), f(accel)]);
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end;
        KlippyError::Command(format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, ep[0], ep[1], ep[2], ep[3]
        ))
    }
}

struct Harness {
    ms: Arc<ManualStepper>,
    log: Log,
    toolhead: Arc<FakeToolhead>,
    _printer: Printer,
    _config: Config,
}

fn make_ms(
    opts: Vec<(&str, f64)>,
    str_opts: Vec<(&str, &str)>,
    can_home: bool,
) -> Harness {
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(IndexMap::new()),
        log: Arc::clone(&log),
    }) as Printer;
    let stepper = Arc::new(FakeStepper {
        name: "stepper_x".to_string(),
        log: Arc::clone(&log),
    }) as Stepper;
    let endstop = Arc::new(FakeEndstop { name: "rail_x".to_string() }) as Endstop;
    let rail = Arc::new(FakeRail {
        name: "rail_x".to_string(),
        steppers: vec![Arc::clone(&stepper)],
        endstops: vec![(Arc::clone(&endstop), "rail_x".to_string())],
        log: Arc::clone(&log),
    }) as Rail;
    let factory = Arc::new(FakeFactory {
        rail: Arc::clone(&rail),
        stepper: Arc::clone(&stepper),
        log: Arc::clone(&log),
    });
    let toolhead = Arc::new(FakeToolhead {
        log: Arc::clone(&log),
        last: Mutex::new(10.0),
        extra_axes: Mutex::new(Vec::new()),
        factory: Arc::clone(&factory),
    });
    printer
        .add_object("toolhead", Arc::clone(&toolhead) as ObjectRef)
        .expect("add toolhead");
    let mq = Arc::new(FakeMQ {
        log: Arc::clone(&log),
        n: Mutex::new(0),
    });
    printer
        .add_object("motion_queuing", Arc::clone(&mq) as ObjectRef)
        .expect("add mq");
    let se = Arc::new(FakeStepperEnable {
        log: Arc::clone(&log),
    });
    printer
        .add_object("stepper_enable", Arc::clone(&se) as ObjectRef)
        .expect("add se");
    let homing = Arc::new(FakeHoming {
        log: Arc::clone(&log),
    });
    printer
        .add_object("homing", Arc::clone(&homing) as ObjectRef)
        .expect("add homing");
    let configfile = Arc::new(FakeConfigFile {
        log: Arc::clone(&log),
    });
    printer
        .add_object("configfile", Arc::clone(&configfile) as ObjectRef)
        .expect("add configfile");
    let gcode = Arc::new(FakeGCode {
        log: Arc::clone(&log),
    });
    printer
        .add_object("gcode", Arc::clone(&gcode) as ObjectRef)
        .expect("add gcode");
    let gcode_move = Arc::new(FakeGCodeMove {
        log: Arc::clone(&log),
    });
    printer
        .add_object("gcode_move", Arc::clone(&gcode_move) as ObjectRef)
        .expect("add gcode_move");
    let mut map = IndexMap::new();
    map.insert("velocity".to_string(), 7.0);
    map.insert("accel".to_string(), 500.0);
    for (k, v) in opts {
        map.insert(k.to_string(), v);
    }
    let mut smap = IndexMap::new();
    for (k, v) in str_opts {
        smap.insert(k.to_string(), v.to_string());
    }
    if can_home && !smap.contains_key("endstop_pin") {
        smap.insert("endstop_pin".to_string(), "PF1".to_string());
    }
    let config = Arc::new(FakeConfig {
        printer: Arc::clone(&printer),
        name: "manual_stepper foo".to_string(),
        opts: map,
        str_opts: smap,
    }) as Config;
    let ms = ManualStepper::new(&config).expect("ManualStepper::new");
    Harness {
        ms,
        log,
        toolhead,
        _printer: printer,
        _config: config,
    }
}

fn gcmd(params: Vec<(&str, &str)>) -> GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeGcmd {
        params: map,
        commandline: "MANUAL_STEPPER".to_string(),
    }) as GCmd
}

fn state_value(ms: &ManualStepper) -> Value {
    fn opt_f(v: Option<f64>) -> Value {
        match v {
            Some(x) => f(x),
            None => Value::Null,
        }
    }
    Value::Tuple(vec![
        f(ms.commanded_pos()),
        f(ms.next_cmd_time()),
        match ms.get_axis_gcode_id() {
            Some(v) => s(&v),
            None => Value::Null,
        },
        f(ms.velocity()),
        f(ms.accel()),
        Value::Bool(ms.can_home()),
        opt_f(ms.pos_min()),
        opt_f(ms.pos_max()),
    ])
}

#[test]
fn calc_move_time_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        (10.0, 5.0, 500.0),
        (-10.0, 5.0, 500.0),
        (0.0, 5.0, 500.0),
        (3.0, 10.0, 0.0),
        (0.1, 7.0, 500.0),
        (-0.5, 2.0, 10.0),
        (100.0, 300.0, 3000.0),
    ];
    for (dist, speed, accel) in cases {
        let expected = py.eval(&format!("calc_move_time({dist:?}, {speed:?}, {accel:?})"));
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel).expect("calc");
        let actual = Value::Tuple(vec![f(axis_r), f(accel_t), f(cruise_t), f(cruise_v)]);
        check(&format!("calc_move_time({dist},{speed},{accel})"), expected, actual);
    }
}

#[test]
fn construction_defaults_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    let h = make_ms(vec![], vec![], false);
    let expected = py.eval("state_of(ms)");
    check("construction.state", expected, state_value(&h.ms));
    let expected_logs = py.eval("list(logs)");
    check(
        "construction.logs",
        expected_logs,
        Value::List(h.log.lock().clone()),
    );
    assert_eq!(h.ms.get_name(), "manual_stepper foo");
}

#[test]
fn construction_with_endstop_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms({'position_min': -5.0, 'position_max': 50.0, 'endstop_pin': 'PF1'})")
        .expect("py make_ms");
    let h = make_ms(
        vec![("position_min", -5.0), ("position_max", 50.0)],
        vec![("endstop_pin", "PF1")],
        true,
    );
    let expected = py.eval("(ms.can_home, ms.pos_min, ms.pos_max, list(logs))");
    let actual = Value::Tuple(vec![
        Value::Bool(h.ms.can_home()),
        f(h.ms.pos_min().unwrap()),
        f(h.ms.pos_max().unwrap()),
        Value::List(h.log.lock().clone()),
    ]);
    check("construction_endstop.state", expected, actual);
}

#[test]
fn do_move_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.do_move(12.5, 20.0, 400.0)").expect("py do_move");
    let expected_logs = py.eval("list(logs)");
    let h = make_ms(vec![], vec![], false);
    h.ms.do_move(12.5, 20.0, 400.0, true).expect("do_move");
    check("do_move.logs", expected_logs, Value::List(h.log.lock().clone()));
    let exp_state = py.eval("state_of(ms)");
    check("do_move.state", exp_state, state_value(&h.ms));
}

#[test]
fn do_set_position_and_sync_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.do_set_position(3.25); ms.dwell(2.0); ms.dwell(-1.0)")
        .expect("py setpos");
    let expected_logs = py.eval("list(logs)");
    let h = make_ms(vec![], vec![], false);
    h.ms.do_set_position(3.25).expect("set_position");
    h.ms.dwell(2.0);
    h.ms.dwell(-1.0);
    check(
        "set_position.logs",
        expected_logs,
        Value::List(h.log.lock().clone()),
    );
    let exp_pos = py.eval("ms.get_position()");
    check(
        "set_position.pos",
        exp_pos,
        Value::List(h.ms.get_position().iter().map(|v| f(*v)).collect()),
    );
}

#[test]
fn cmd_move_sync_and_enable_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.cmd_MANUAL_STEPPER(FakeGcmd({'ENABLE': '1', 'SET_POSITION': '2.0', 'MOVE': '9.0', 'SPEED': '15.0', 'ACCEL': '300.0'}))")
        .expect("py cmd");
    let expected_logs = py.eval("list(logs)");
    let h = make_ms(vec![], vec![], false);
    h.ms.cmd_manual_stepper(&gcmd(vec![
        ("ENABLE", "1"),
        ("SET_POSITION", "2.0"),
        ("MOVE", "9.0"),
        ("SPEED", "15.0"),
        ("ACCEL", "300.0"),
    ]))
    .expect("cmd");
    check("cmd.logs", expected_logs, Value::List(h.log.lock().clone()));
    let exp_state = py.eval("state_of(ms)");
    check("cmd.state", exp_state, state_value(&h.ms));
}

#[test]
fn cmd_move_out_of_range_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms({'position_min': 0.0, 'position_max': 10.0})")
        .expect("py make_ms");
    let py_result = py.eval("ms.cmd_MANUAL_STEPPER(FakeGcmd({'MOVE': '99.0'}))");
    let h = make_ms(
        vec![("position_min", 0.0), ("position_max", 10.0)],
        vec![],
        false,
    );
    let rust_result = h
        .ms
        .cmd_manual_stepper(&gcmd(vec![("MOVE", "99.0")]))
        .map(|_| Value::Null);
    match (py_result, rust_result) {
        (Err(pe), Err(re)) => {
            assert_eq!(pe.message, re.message(), "out of range message");
            assert_eq!(re.class_name(), "CommandError");
        }
        (p, r) => panic!("expected both error, py={p:?} rust={r:?}"),
    }
}

#[test]
fn cmd_stop_on_endstop_variants_match_python() {
    let Some(mut py) = oracle() else { return };
    let variants = [
        ("home", "home", false),
        ("probe", "probe", false),
        ("try_home", "home", true),
        ("inverted_home", "home", false),
        ("-2", "home", true),
        ("1", "home", false),
    ];
    for (stop, _norm, _try) in variants {
        py.exec("ms, printer, toolhead, mq = make_ms({'endstop_pin': 'PF1'})")
            .expect("py make_ms");
        let py_result = py.eval(&format!(
            "ms.cmd_MANUAL_STEPPER(FakeGcmd({{'STOP_ON_ENDSTOP': '{stop}', 'MOVE': '5.0'}}))"
        ));
        let h = make_ms(vec![], vec![("endstop_pin", "PF1")], true);
        let rust_result = h
            .ms
            .cmd_manual_stepper(&gcmd(vec![("STOP_ON_ENDSTOP", stop), ("MOVE", "5.0")]));
        match (py_result, rust_result) {
            (Ok(_), Ok(_)) => {}
            (Err(pe), Err(re)) => {
                assert_eq!(pe.message, re.message(), "stop {stop} message");
                continue;
            }
            (p, r) => panic!("stop {stop}: py={p:?} rust={r:?}"),
        }
        let py_scalars = py
            .eval("[(l[2], l[3], l[4], l[5]) for l in logs if l[0] == 'manual_home']")
            .expect("py scalars");
        let rust_scalars: Vec<Value> = h
            .log
            .lock()
            .iter()
            .filter_map(|v| match v {
                Value::Tuple(items) if items.first() == Some(&s("manual_home")) => {
                    Some(Value::Tuple(vec![
                        items[2].clone(),
                        items[3].clone(),
                        items[4].clone(),
                        items[5].clone(),
                    ]))
                }
                _ => None,
            })
            .collect();
        check(&format!("stop_{stop}.scalars"), Ok(py_scalars), Value::List(rust_scalars));
    }
}

#[test]
fn cmd_stop_unknown_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms({'endstop_pin': 'PF1'})")
        .expect("py make_ms");
    let py_result = py.eval("ms.cmd_MANUAL_STEPPER(FakeGcmd({'STOP_ON_ENDSTOP': 'bogus', 'MOVE': '5.0'}))");
    let h = make_ms(vec![], vec![("endstop_pin", "PF1")], true);
    let rust_result = h
        .ms
        .cmd_manual_stepper(&gcmd(vec![("STOP_ON_ENDSTOP", "bogus"), ("MOVE", "5.0")]))
        .map(|_| Value::Null);
    check_err("stop_unknown", py_result, rust_result);
}

#[test]
fn homing_without_endstop_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    let py_result = py.eval("ms.do_homing_move(5.0, 10.0, 100.0, False, True, True)");
    let h = make_ms(vec![], vec![], false);
    let rust_result = h
        .ms
        .do_homing_move(5.0, 10.0, 100.0, false, true, true)
        .map(|_| Value::Null);
    check_err("homing_no_endstop", py_result, rust_result);
}

#[test]
fn gcode_axis_register_unregister_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': 'A', 'INSTANTANEOUS_CORNER_VELOCITY': '2.0'}))")
        .expect("py register");
    let expected_logs = py.eval("list(logs)");
    let exp_axis = py.eval("ms.get_axis_gcode_id()");
    let h = make_ms(vec![], vec![], false);
    h.ms.cmd_manual_stepper(&gcmd(vec![
        ("GCODE_AXIS", "A"),
        ("INSTANTANEOUS_CORNER_VELOCITY", "2.0"),
    ]))
    .expect("register axis");
    check(
        "gcode_axis.register.logs",
        expected_logs,
        Value::List(h.log.lock().clone()),
    );
    check(
        "gcode_axis.register.id",
        exp_axis,
        match h.ms.get_axis_gcode_id() {
            Some(v) => s(&v),
            None => Value::Null,
        },
    );
    let py_dup = py.eval("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': 'A'}))");
    let rust_dup = h.ms.command_with_gcode_axis(&gcmd(vec![("GCODE_AXIS", "A")]));
    check_err("gcode_axis.duplicate", py_dup, rust_dup.map(|_| Value::Null));
    py.exec("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': ''}))")
        .expect("py unregister");
    h.ms.command_with_gcode_axis(&gcmd(vec![("GCODE_AXIS", "")]))
        .expect("unregister");
    let exp_axis2 = py.eval("ms.get_axis_gcode_id()");
    check(
        "gcode_axis.unregister.id",
        exp_axis2,
        match h.ms.get_axis_gcode_id() {
            Some(v) => s(&v),
            None => Value::Null,
        },
    );
    let py_bad = py.eval("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': 'X'}))");
    let rust_bad = h.ms.command_with_gcode_axis(&gcmd(vec![("GCODE_AXIS", "X")]));
    check_err("gcode_axis.invalid", py_bad, rust_bad.map(|_| Value::Null));
    py.exec("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': 'B'}))")
        .expect("py register B");
    h.ms.command_with_gcode_axis(&gcmd(vec![("GCODE_AXIS", "B")]))
        .expect("register B");
    let py_blocked = py.eval("ms.cmd_MANUAL_STEPPER(FakeGcmd({'MOVE': '1.0'}))");
    let rust_blocked = h
        .ms
        .cmd_manual_stepper(&gcmd(vec![("MOVE", "1.0")]))
        .map(|_| Value::Null);
    check_err("gcode_axis.blocked", py_blocked, rust_blocked);
}

#[test]
fn process_check_calc_junction_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.command_with_gcode_axis(FakeGcmd({'GCODE_AXIS': 'A', 'LIMIT_VELOCITY': '50.0', 'LIMIT_ACCEL': '200.0'}))")
        .expect("py axis");
    py.exec("logs.clear()").expect("clear");
    py.exec("mv = FakeMove([0.,0.,0.,0.], [5.,0.,0.,5.], [5.,0.,0.,5.], [1.,0.,0.,1.], 7.0710678118654755)")
        .expect("py move");
    py.exec("ms.process_move(20.0, mv, 3)").expect("py process");
    let expected_logs = py.eval("list(logs)");
    let h = make_ms(vec![], vec![], false);
    h.ms.command_with_gcode_axis(&gcmd(vec![
        ("GCODE_AXIS", "A"),
        ("LIMIT_VELOCITY", "50.0"),
        ("LIMIT_ACCEL", "200.0"),
    ]))
    .expect("axis");
    h.log.lock().clear();
    let mv = FakeMove {
        start: vec![0.0, 0.0, 0.0, 0.0],
        end: vec![5.0, 0.0, 0.0, 5.0],
        axes_d: vec![5.0, 0.0, 0.0, 5.0],
        axes_r: vec![1.0, 0.0, 0.0, 1.0],
        move_d: 7.0710678118654755,
        max_cruise_v2: 100.0,
        is_kin: true,
        accel_v: 1000.0,
        start_v: 0.0,
        cruise_v: 10.0,
        accel_t: 0.1,
        cruise_t: 0.2,
        decel_t: 0.1,
        log: Arc::clone(&h.log),
    };
    h.ms.process_move_impl(20.0, &mv, 3);
    check(
        "process_move.logs",
        expected_logs,
        Value::List(h.log.lock().clone()),
    );
    py.exec("logs.clear(); ms.check_move(mv, 3)").expect("py check");
    let expected_check = py.eval("list(logs)");
    h.log.lock().clear();
    h.ms.check_move_impl(&mv, 3).expect("check");
    check(
        "check_move.logs",
        expected_check,
        Value::List(h.log.lock().clone()),
    );
    py.exec("prev = FakeMove([0.,0.,0.,0.], [5.,0.,0.,0.], [5.,0.,0.,0.], [1.,0.,0.,0.], 5.0)").expect("prev");
    let py_j = py.eval("ms.calc_junction(prev, mv, 3)");
    let prev = FakeMove {
        start: vec![0.0, 0.0, 0.0, 0.0],
        end: vec![5.0, 0.0, 0.0, 0.0],
        axes_d: vec![5.0, 0.0, 0.0, 0.0],
        axes_r: vec![1.0, 0.0, 0.0, 0.0],
        move_d: 5.0,
        max_cruise_v2: 100.0,
        is_kin: true,
        accel_v: 1000.0,
        start_v: 0.0,
        cruise_v: 10.0,
        accel_t: 0.1,
        cruise_t: 0.2,
        decel_t: 0.1,
        log: Arc::clone(&h.log),
    };
    let rust_j = h.ms.calc_junction_impl(&prev, &mv, 3);
    check("calc_junction", py_j, f(rust_j));
}

#[test]
fn drip_and_wrappers_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("ms, printer, toolhead, mq = make_ms()").expect("py make_ms");
    py.exec("ms.drip_move([4.0,0.,0.,0.], 10.0, 'comp')").expect("py drip");
    let expected_logs = py.eval("list(logs)");
    let h = make_ms(vec![], vec![], false);
    let reactor = Reactor::new(false);
    let comp = reactor.completion::<bool>();
    h.ms.drip_move(vec![4.0, 0.0, 0.0, 0.0], 10.0, &comp)
        .expect("drip");
    let py_filtered = py
        .eval("[tuple('completion' if x == 'comp' else x for x in l) if l[0] == 'drip_update_time' else l for l in logs]")
        .expect("py filtered");
    check(
        "drip.logs",
        Ok(py_filtered),
        Value::List(h.log.lock().clone()),
    );
    let _ = expected_logs;
    // Single-stepper rail name is stepper_x on both sides.
    let py_calc = py.eval("ms.calc_position({'stepper_x': 4.0})");
    let mut pos_map = IndexMap::new();
    pos_map.insert("stepper_x".to_string(), 4.0);
    let rust_calc = h
        .ms
        .calc_position(&pos_map)
        .expect("calc_position");
    let rust_vals: Vec<Value> = rust_calc
        .iter()
        .map(|v| match v {
            Some(x) => f(*x),
            None => Value::Null,
        })
        .collect();
    check("calc_position", py_calc, Value::List(rust_vals));
    let py_steppers = py.eval("[s.get_name() for s in ms.get_steppers()]");
    let rust_steppers = Value::List(
        h.ms
            .get_steppers()
            .iter()
            .map(|st| s(&st.get_name(None)))
            .collect(),
    );
    check("get_steppers", py_steppers, rust_steppers);
}

#[test]
fn module_factory_and_load_config_prefix() {
    let factory = module_factory();
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.load_config.is_none());
    let Some(_py) = oracle() else { return };
    // load_config_prefix builds a full object (its construction logs
    // match the construction oracle in the earlier tests).
    let h = make_ms(vec![], vec![], false);
    let printer = h._printer.clone();
    let config = h._config.clone();
    // Exercise load_config_prefix through a fresh config sharing the
    // same fake printer state: it must succeed and name itself.
    let obj = load_config_prefix(&config).expect("load_config_prefix");
    assert!(obj.is_some());
    let _ = printer;
}
