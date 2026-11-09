//! Parity tests for `extras::quad_gantry_level` (port of
//! `klippy/extras/quad_gantry_level.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): a fresh `QuadGantryLevel` is
//! built on both sides with minimal Python fakes / Rust fakes for the
//! printer, config and gcode objects, and identical probe positions,
//! gcode calls and config values are fed in. Expected values come from
//! the oracle only (return values, raised errors class+message,
//! `respond_info` text, log lines), never transcribed from reading the
//! source.
//!
//! The Rust side constructs the object through `QuadGantryLevel::new`
//! with real `z_tilt` / `probe` helpers (no setters, no injected
//! handles); only core services (`gcode`, `toolhead`, `manual_probe`)
//! and the config are faked. The Python side fakes the `z_tilt` and
//! `probe` helper classes to isolate this module's logic, apart from
//! these mirrorings of the real helpers' observable contracts, where
//! the Rust side is also driven by the real owners:
//! - `FakeProbeHelper.get_lift_speed` returns the configured `speed`
//!   (50.0), like the real `ProbePointsHelper` (`lift_speed = speed`);
//! - `FakeRetryHelper` returns "done" for the default `retries: 0`,
//!   like the real `RetryHelper`;
//! - `FakeZAdjustHelper` records its call instead of moving a toolhead.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply.

use extras::manual_probe::ProbeResult as ProbePosition;
use extras::quad_gantry_level::{linefit, plot, QuadGantryLevel};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, GCodeCommandApi, GCmd, RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Stepper, StepperFactory, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, KinematicsApi, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.quad_gantry_level as qgl

LOG_LINES = []
INFOS = []
ADJUST_CALLS = []
PROBE_STARTS = []
RETRY_STARTS = []
RETRY_RESULT = ["done"]
MAX_ADJUST = [4.0]
HORIZ = [5.0]
CORNERS = [[[0.0, 0.0], [0.0, 0.0]]]
POINTS = [[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]]
WARN_PROBE_LEN = [4]

class CommandError(Exception):
    pass

class FakeLogging:
    def info(self, msg, *a, **k):
        LOG_LINES.append(msg % a if a else msg)
qgl.logging = FakeLogging()

class FakeGcode:
    error = CommandError
    def __init__(self):
        self.registered = {}
    def register_command(self, cmd, func, desc=None, when_not_ready=False):
        self.registered[cmd] = (func, desc)
    def respond_info(self, msg, log=True):
        INFOS.append(msg)

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.gcode = FakeGcode()
    def lookup_object(self, name, default=None):
        if name == 'gcode':
            return self.gcode
        if default is None:
            return None
        raise self.config_error("Unknown object '%s'" % name)
    def register_event_handler(self, event, cb):
        pass

class FakeRetryHelper:
    def __init__(self, config, extra=""):
        self.extra = extra
    def start(self, gcmd):
        RETRY_STARTS.append(gcmd)
    def check_retry(self, z_positions):
        return RETRY_RESULT[0]

class FakeZAdjustStatus:
    def __init__(self, printer):
        self.applied = False
    def check_retry_result(self, r):
        if r == "done":
            self.applied = True
        return r
    def reset(self):
        self.applied = False
    def get_status(self, eventtime):
        return {'applied': self.applied}

class FakeZAdjustHelper:
    def __init__(self, config, z_count):
        self.z_count = z_count
    def adjust_steppers(self, adjustments, speed):
        ADJUST_CALLS.append(([float(a) for a in adjustments], float(speed)))

class FakeProbeHelper:
    def __init__(self, config, cb):
        self.cb = cb
        self.speed = config.getfloat('speed', 50., above=0.)
        self.probe_points = config.getlists('points', parser=float, seps=(',', '\n'), count=2)
    def start_probe(self, gcmd):
        PROBE_STARTS.append(gcmd)
    def get_lift_speed(self):
        return self.speed

class FakeProbeMod:
    ProbePointsHelper = FakeProbeHelper

class FakeZtiltMod:
    RetryHelper = FakeRetryHelper
    ZAdjustHelper = FakeZAdjustHelper
    ZAdjustStatus = FakeZAdjustStatus

qgl.probe = FakeProbeMod()
qgl.z_tilt = FakeZtiltMod()

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'quad_gantry_level'
    def getfloat(self, name, default, **kw):
        if name == 'max_adjust':
            return default if MAX_ADJUST[0] is None else MAX_ADJUST[0]
        if name == 'horizontal_move_z':
            return default if HORIZ[0] is None else HORIZ[0]
        if name == 'speed':
            return 50.0
        return default
    def getint(self, name, default, **kw):
        return default
    def get(self, name, default=None):
        if name in ('points',):
            return None
        return default
    def getlists(self, name, parser=float, seps=(',',), count=None, default=None, **kw):
        if name == 'points':
            return tuple(tuple(inner) for inner in POINTS[0])
        if name == 'gantry_corners':
            return tuple(tuple(inner) for inner in CORNERS[0])
        raise self.error("unknown lists %s" % name)

class FakeGcmd:
    def get(self, name, default=None):
        return default
    def get_int(self, name, default=None, minval=None, maxval=None):
        return default
    def get_float(self, name, default=None, minval=None, maxval=None, above=None, below=None):
        return default

qgl_obj = None
qgl_printer = None
qgl_config = None

def make_quad():
    global qgl_obj, qgl_printer, qgl_config
    LOG_LINES.clear(); INFOS.clear(); ADJUST_CALLS.clear(); PROBE_STARTS.clear(); RETRY_STARTS.clear()
    qgl_printer = FakePrinter()
    qgl_config = FakeConfig(qgl_printer)
    qgl_obj = qgl.load_config(qgl_config)
    return qgl_obj

def set_scenario(max_adjust, horiz, corners, points, retry_result="done"):
    MAX_ADJUST[0] = None if max_adjust is None else float(max_adjust)
    HORIZ[0] = None if horiz is None else float(horiz)
    CORNERS[0] = [[float(c[0]), float(c[1])] for c in corners]
    POINTS[0] = [[float(p[0]), float(p[1])] for p in points]
    RETRY_RESULT[0] = retry_result

class Coord(list):
    @property
    def bed_x(self):
        return self[0]
    @property
    def bed_y(self):
        return self[1]
    @property
    def bed_z(self):
        return self[2]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

/// Log lines emitted by the current test thread. `log::set_sink` is
/// process-wide while cargo runs tests in parallel, so the sink buckets
/// records by thread and each test reads only its own.
static LOG_LINES: Mutex<Option<indexmap::IndexMap<std::thread::ThreadId, Vec<String>>>> =
    Mutex::new(None);

/// The static is shared by every test thread, so a panicking test must
/// not poison it for the rest.
fn log_lines_lock() -> std::sync::MutexGuard<'static, Option<indexmap::IndexMap<std::thread::ThreadId, Vec<String>>>> {
    LOG_LINES.lock().unwrap_or_else(|e| e.into_inner())
}

fn install_log_sink() {
    klippy_api::log::set_sink(Box::new(|rec| {
        let mut guard = log_lines_lock();
        guard
            .get_or_insert_with(indexmap::IndexMap::new)
            .entry(std::thread::current().id())
            .or_default()
            .push(rec.message.to_string());
    }));
}

fn clear_log_lines() {
    let mut guard = log_lines_lock();
    if let Some(buckets) = guard.as_mut() {
        buckets.shift_remove(&std::thread::current().id());
    }
}

fn take_log_lines() -> Vec<String> {
    let mut guard = log_lines_lock();
    guard
        .as_mut()
        .and_then(|buckets| buckets.shift_remove(&std::thread::current().id()))
        .unwrap_or_default()
}

#[derive(Clone)]
struct FakeGcode {
    state: Arc<Mutex<FakeGcodeState>>,
}

struct FakeGcodeState {
    infos: Vec<String>,
    registered: Vec<(String, Option<String>)>,
}

impl FakeGcode {
    fn new() -> (Self, Arc<Mutex<FakeGcodeState>>) {
        let state = Arc::new(Mutex::new(FakeGcodeState {
            infos: Vec::new(),
            registered: Vec::new(),
        }));
        (Self { state: state.clone() }, state)
    }
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.state.lock().unwrap().registered.push((cmd.to_string(), opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd)
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, _log: bool) {
        self.state.lock().unwrap().infos.push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeGcodeAsObject {
    inner: FakeGcode,
}

klippy_api::printer_object!(FakeGcodeAsObject; dyn GCodeApi);

impl GCodeApi for FakeGcodeAsObject {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.inner.register_command(cmd, func, opts)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.inner.register_mux_command(cmd, key, value, func, desc)
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        self.inner.get_command_help()
    }
    fn get_status(&self, eventtime: f64) -> PyValue {
        self.inner.get_status(eventtime)
    }
    fn register_output_handler(&self, callback: klippy_api::gcode::OutputCallback) {
        self.inner.register_output_handler(callback)
    }
    fn request_restart(&self, result: &str) -> KResult<()> {
        self.inner.request_restart(result)
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.inner.run_script(script)
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.inner.run_script_from_command(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.inner.get_mutex()
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        self.inner.create_gcode_command(command, commandline, params)
    }
    fn respond_raw(&self, msg: &str) {
        self.inner.respond_raw(msg)
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.inner.respond_info(msg, log)
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        self.inner.is_traditional_gcode(cmd)
    }
    fn cmd_m112(&self, gcmd: Option<&GCmd>) -> KResult<()> {
        self.inner.cmd_m112(gcmd)
    }
    fn process_commands(&self, commands: &[String], need_ack: bool) -> KResult<()> {
        self.inner.process_commands(commands, need_ack)
    }
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

struct FakeStepper {
    name: String,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("not used by quad_gantry_level")
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
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
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
    fn set_position(&self, _coord: [f64; 3]) {}
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
    fn dump_steps(&self, _count: i64, _start_clock: i64, _end_clock: i64) -> (Vec<PullHistoryStep>, i64) {
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
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, axis: char) -> bool {
        axis == 'z'
    }
}

struct FakeKinematics;

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        ["stepper_z", "stepper_z1", "stepper_z2", "stepper_z3"]
            .iter()
            .map(|n| Arc::new(FakeStepper { name: n.to_string() }) as Stepper)
            .collect()
    }
    fn calc_position(&self, _stepper_positions: &indexmap::IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(vec![Some(0.0), Some(0.0), Some(0.0), Some(0.0)])
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("home is not used by quad_gantry_level")
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn clear_homing_state(&self, _axes: &str) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakeToolhead {
    position: Mutex<Vec<f64>>,
    moves: Mutex<Vec<(Vec<f64>, f64)>>,
    set_positions: Mutex<Vec<Vec<f64>>>,
}

impl FakeToolhead {
    fn new() -> Arc<Self> {
        Arc::new(FakeToolhead {
            position: Mutex::new(vec![0.0, 0.0, 10.0, 0.0]),
            moves: Mutex::new(Vec::new()),
            set_positions: Mutex::new(Vec::new()),
        })
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.lock().unwrap().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.position.lock().unwrap() = newpos.clone();
        self.set_positions.lock().unwrap().push(newpos);
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics)
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _newpos: Vec<f64>, _speed: f64, _completion: &reactor::Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves.lock().unwrap().push((newpos, speed));
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let mut pos = self.position.lock().unwrap().clone();
        for (i, v) in coord.iter().enumerate() {
            if let Some(v) = v {
                pos[i] = *v;
            }
        }
        *self.position.lock().unwrap() = pos;
        self.moves.lock().unwrap().push((coord.iter().map(|v| v.unwrap_or(0.0)).collect(), speed));
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
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
    fn register_lookahead_callback(&self, _callback: klippy_api::toolhead::LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("not used by quad_gantry_level")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("not used by quad_gantry_level")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakeManualProbe;

impl extras::manual_probe::ManualProbeApi for FakeManualProbe {
    fn reset_status(&self) {}
    fn set_status(
        &self,
        _is_active: bool,
        _z_position: Option<f64>,
        _z_position_lower: Option<f64>,
        _z_position_upper: Option<f64>,
    ) {}
}

klippy_api::printer_object!(FakeManualProbe; dyn extras::manual_probe::ManualProbeApi);

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<indexmap::IndexMap<String, ObjectRef>>>,
    handlers: Arc<Mutex<Vec<(String, EventCallback)>>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter {
            objects: Arc::new(Mutex::new(indexmap::IndexMap::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }
    fn fire(&self, event: &str) -> KResult<Vec<PyValue>> {
        self.send_event(event, &[])
    }
}

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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().unwrap().get(name).cloned() {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(klippy_api::KlippyError::Config(format!(
                    "Unknown object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects
            .lock()
            .unwrap()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().unwrap().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut handlers = self.handlers.lock().unwrap();
        for (_name, cb) in handlers.iter_mut().filter(|(n, _)| n == event) {
            out.push(cb(params)?);
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    /// `None` means the option is absent, so the config layer returns
    /// the default the caller passed, like `configfile.getfloat`.
    max_adjust: Option<f64>,
    horiz: Option<f64>,
    points: Vec<[f64; 2]>,
    corners: Vec<[f64; 2]>,
    /// `[quad_gantry_level] retries` / `retry_tolerance`, read by the
    /// real `RetryHelper` this module constructs.
    retries: i64,
    retry_tolerance: f64,
}

impl FakeConfig {
    fn getlists_val(pairs: &Vec<[f64; 2]>) -> PyValue {
        PyValue::Tuple(
            pairs
                .iter()
                .map(|p| PyValue::Tuple(vec![PyValue::Float(p[0]), PyValue::Float(p[1])]))
                .collect(),
        )
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "quad_gantry_level".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match option {
            "retries" => Ok(Some(self.retries)),
            _ => Ok(opts.default.unwrap_or(None)),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v = match option {
            "max_adjust" => self.max_adjust,
            "horizontal_move_z" => self.horiz,
            "speed" => Some(50.0),
            "retry_tolerance" => Some(self.retry_tolerance),
            _ => return Ok(opts.default.unwrap_or(None)),
        };
        Ok(v.or(opts.default.unwrap_or(None)))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        match option {
            "points" => Ok(Self::getlists_val(&self.points)),
            "gantry_corners" => Ok(Self::getlists_val(&self.corners)),
            _ => Err(klippy_api::KlippyError::Config(format!(
                "unknown lists {option}"
            ))),
        }
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by quad_gantry_level")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeGcmd;

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, _name: &str, _opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_int(&self, _name: &str, opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn get_float(&self, _name: &str, opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(opts.default.unwrap_or(None))
    }
    fn get_command(&self) -> String {
        "QUAD_GANTRY_LEVEL".to_string()
    }
    fn get_commandline(&self) -> String {
        "QUAD_GANTRY_LEVEL".to_string()
    }
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct Fixture {
    quad: Arc<QuadGantryLevel>,
    printer_fp: FakePrinter,
    gstate: Arc<Mutex<FakeGcodeState>>,
    toolhead: Arc<FakeToolhead>,
}

fn make_fixture(
    max_adjust: Option<f64>,
    horiz: Option<f64>,
    corners: Vec<[f64; 2]>,
    points: Vec<[f64; 2]>,
) -> Fixture {
    make_fixture_opts(max_adjust, horiz, corners, points, 0, 0.0)
}

fn make_fixture_opts(
    max_adjust: Option<f64>,
    horiz: Option<f64>,
    corners: Vec<[f64; 2]>,
    points: Vec<[f64; 2]>,
    retries: i64,
    retry_tolerance: f64,
) -> Fixture {
    let (gcode, gstate) = FakeGcode::new();
    let toolhead = FakeToolhead::new();
    let fp = FakePrinter::new();
    fp.add(
        "gcode",
        Arc::new(FakeGcodeAsObject { inner: gcode.clone() }) as ObjectRef,
    );
    fp.add("toolhead", toolhead.clone() as ObjectRef);
    fp.add("manual_probe", Arc::new(FakeManualProbe) as ObjectRef);
    let printer: Printer = Arc::new(fp.clone());
    let config: Config = Arc::new(FakeConfig {
        printer,
        max_adjust,
        horiz,
        points,
        corners,
        retries,
        retry_tolerance,
    });
    let quad = QuadGantryLevel::new(&config).expect("rust QuadGantryLevel::new");
    // The real ZAdjustHelper populates its steppers on klippy:connect.
    fp.fire(klippy_api::printer::EV_KLIPPY_CONNECT)
        .expect("klippy:connect handlers");
    Fixture { quad, printer_fp: fp, gstate, toolhead }
}
/// Drive the real command entry on both sides before a direct
/// `probe_finalize` call, so the real `RetryHelper` state is started
/// the way the printer would start it. The Rust `cmd` also starts the
/// real probe helper (manual branch), which responds and moves; those
/// side effects are cleared here because the Python fakes record
/// nothing observable for `start_probe` beyond the call itself.
fn start_cmd_both(py: &mut pyoracle::Oracle, fx: &Fixture) {
    py.eval("qgl_obj.cmd_QUAD_GANTRY_LEVEL(FakeGcmd())")
        .expect("py cmd start");
    let gcmd: GCmd = Arc::new(FakeGcmd);
    fx.quad.cmd_quad_gantry_level(&gcmd).expect("rust cmd start");
    fx.gstate.lock().unwrap().infos.clear();
    fx.toolhead.set_positions.lock().unwrap().clear();
    fx.toolhead.moves.lock().unwrap().clear();
    // The Rust cmd moves the fake toolhead to the first probe point
    // (Z = horizontal_move_z); restore its initial position so the
    // adjustment math starts from the same Z as the Python side.
    *fx.toolhead.position.lock().unwrap() = vec![0.0, 0.0, 10.0, 0.0];
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("quad_gantry_level fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping quad_gantry_level oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
    let record = Record {
        v: 1,
        unit: "quad_gantry_level".to_string(),
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

fn check_str(label: &str, expected: pyoracle::OracleResult, actual: KResult<String>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), Value::str(&actual));
}

fn positions_py(pos: &[[f64; 3]; 4]) -> String {
    let items: Vec<String> = pos
        .iter()
        .map(|p| format!("Coord({:?})", p))
        .collect();
    format!("[{}]", items.join(", "))
}

fn rust_positions(pos: &[[f64; 3]; 4]) -> Vec<ProbePosition> {
    pos.iter()
        .map(|p| ProbePosition { bed_x: p[0], bed_y: p[1], bed_z: p[2], test_x: 0.0, test_y: 0.0, test_z: 0.0 })
        .collect()
}

/// Convert a `get_status` dict for comparison with the oracle's.
fn status_value(status: &PyValue) -> Value {
    match status {
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, v)| {
                    let kv = match k {
                        PyValue::Str(s) => Value::str(s),
                        _ => panic!("dict key"),
                    };
                    let vv = match v {
                        PyValue::Bool(b) => Value::Bool(*b),
                        _ => panic!("dict value"),
                    };
                    (kv, vv)
                })
                .collect(),
        ),
        _ => panic!("status dict"),
    }
}

/// The `applied` entry of a `get_status` dict.
fn applied_of(status: &PyValue) -> Option<bool> {
    match status {
        PyValue::Dict(map) => match map.get(&PyValue::Str("applied".to_string())) {
            Some(PyValue::Bool(b)) => Some(*b),
            _ => None,
        },
        _ => None,
    }
}

fn recorder_values(py: &mut pyoracle::Oracle, expr: &str) -> Vec<Value> {
    match py.eval(expr) {
        Ok(Value::List(items)) => items,
        other => panic!("expected a list from {expr}, got {other:?}"),
    }
}

/// Expected `set_position` Z sequence, computed from the oracle's
/// recorded adjustments with the fake toolhead's start Z = 10.0.
fn expected_set_positions(adjustments: &[f64]) -> Vec<f64> {
    // positions sorted by (-adjustment) ascending, stable.
    let mut order: Vec<usize> = (0..adjustments.len()).collect();
    order.sort_by(|&a, &b| (-adjustments[a]).partial_cmp(&-adjustments[b]).unwrap());
    let first_offset = -adjustments[order[0]];
    let z_low = 10.0 - first_offset;
    let mut zs: Vec<f64> = order
        .iter()
        .skip(1)
        .map(|&i| z_low - adjustments[i])
        .collect();
    // Final set_position adds first_offset back.
    let last = zs.last().copied().unwrap_or(10.0);
    zs.push(last + first_offset);
    zs
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn linefit_flat_line() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]]); make_quad()")
        .expect("setup");
    let expected = py.eval("qgl_obj.linefit([1.0, 2.0], [5.0, 2.0])");
    let actual = linefit([1.0, 2.0], [5.0, 2.0]).expect("linefit");
    assert_eq!(actual.0, 0.0, "flat line slope");
    check(
        "linefit_flat_line",
        expected,
        // Python returns (0, <float>) with an int zero for a flat line;
        // Rust's f64 return is rendered back as that int.
        Value::Tuple(vec![Value::int(0), Value::float(actual.1)]),
    );
}

#[test]
fn linefit_sloped() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]]); make_quad()")
        .expect("setup");
    let expected = py.eval("qgl_obj.linefit([0.0, 1.0], [4.0, 5.0])");
    let actual = linefit([0.0, 1.0], [4.0, 5.0]).expect("linefit");
    check(
        "linefit_sloped",
        expected,
        Value::Tuple(vec![Value::float(actual.0), Value::float(actual.1)]),
    );
}

#[test]
fn linefit_zero_division() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]]); make_quad()")
        .expect("setup");
    let expected = py.eval("qgl_obj.linefit([2.0, 1.0], [2.0, 5.0])");
    let actual = linefit([2.0, 1.0], [2.0, 5.0]);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("rust returned {v:?} but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "linefit zero class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "linefit zero message");
}

#[test]
fn plot_evaluation() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]]); make_quad()")
        .expect("setup");
    let expected = py.eval("qgl_obj.plot((3.0, -2.5), 7.5)");
    let actual = plot((3.0, -2.5), 7.5);
    check("plot_evaluation", expected, Value::float(actual));
}

#[test]
fn probe_finalize_flat_bed() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    clear_log_lines();
    let corners = [[0.0, 0.0], [100.0, 100.0]];
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [100.0, 100.0, 0.0], [0.0, 100.0, 0.0]];
    let fx = make_fixture(Some(4.0), Some(5.0), corners.to_vec(), vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
    start_cmd_both(&mut py, &fx);
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos))).expect("eval");
    let actual = fx.quad.probe_finalize(&rust_positions(&pos));
    let rs_logs = take_log_lines();
    check_str("probe_finalize_flat_bed ret", Ok(expected.clone()), actual);
    // The three respond_info lines this module emits; the real
    // ZAdjustHelper appends one more for the adjustment it applies.
    let py_infos = recorder_values(&mut py, "INFOS");
    let rs_infos = fx.gstate.lock().unwrap().infos.clone();
    assert_eq!(py_infos.len(), 3, "reference respond_info count");
    assert_eq!(
        rs_infos.len(),
        py_infos.len() + 1,
        "respond_info count (real z_tilt emits the adjust info)"
    );
    for (i, (a, b)) in py_infos.iter().zip(rs_infos.iter()).enumerate() {
        let py_str = match a {
            Value::Str(s) => s.clone(),
            _ => panic!("INFOS element is a string"),
        };
        check(&format!("respond_info[{i}]"), Ok(Value::str(&py_str)), Value::str(b));
    }
    // Log lines match too.
    let py_logs = recorder_values(&mut py, "LOG_LINES");
    assert_eq!(rs_logs.len(), py_logs.len(), "log line count");
    for (i, (a, b)) in py_logs.iter().zip(rs_logs.iter()).enumerate() {
        let py_str = match a {
            Value::Str(s) => s.clone(),
            _ => panic!("LOG_LINES element is a string"),
        };
        check(&format!("log[{i}]"), Ok(Value::str(&py_str)), Value::str(b));
    }
    // Adjustment effect through the real ZAdjustHelper surfaces as
    // toolhead calls; derive the expected sequence from the
    // oracle-recorded adjustments and speed.
    let py_adj = recorder_values(&mut py, "ADJUST_CALLS");
    assert_eq!(py_adj.len(), 1, "one adjust call");
    let (adjs, speed) = match &py_adj[0] {
        Value::Tuple(pair) => match (&pair[0], &pair[1]) {
            (Value::List(vals), Value::Float(sp)) => {
                let adjs: Vec<f64> = vals
                    .iter()
                    .map(|v| match v {
                        Value::Float(f) => f.repr.parse().expect("adjustment float"),
                        _ => panic!("adjustment is a float"),
                    })
                    .collect();
                (adjs, sp.repr.parse::<f64>().expect("adjustment speed"))
            }
            other => panic!("adjust call shape {other:?}"),
        },
        other => panic!("adjust call shape {other:?}"),
    };
    // `get_lift_speed()` reaches the helper as the move speed.
    let rs_moves = fx.toolhead.moves.lock().unwrap().clone();
    assert_eq!(rs_moves.len(), 3, "adjust move count");
    for (i, (_pos, sp)) in rs_moves.iter().enumerate() {
        check(
            &format!("adjust move speed[{i}]"),
            Ok(Value::float(speed)),
            Value::float(*sp),
        );
    }
    let expected_zs = expected_set_positions(&adjs);
    let rs_zs: Vec<f64> = fx
        .toolhead
        .set_positions
        .lock()
        .unwrap()
        .iter()
        .map(|p| p[2])
        .collect();
    assert_eq!(rs_zs.len(), expected_zs.len(), "set_position count");
    for (i, (a, b)) in expected_zs.iter().zip(rs_zs.iter()).enumerate() {
        check(&format!("adjust set_position z[{i}]"), Ok(Value::float(*a)), Value::float(*b));
    }
}

#[test]
fn probe_finalize_tilted_bed() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    clear_log_lines();
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.1], [100.0, 10.0, 0.2], [90.0, 90.0, 0.3], [10.0, 100.0, 0.4]];
    let fx = make_fixture(Some(4.0), Some(5.0), vec![[0.0, 0.0], [10.0, 10.0]], vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
    start_cmd_both(&mut py, &fx);
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos)));
    let actual = fx.quad.probe_finalize(&rust_positions(&pos));
    let rs_logs = take_log_lines();
    check_str("probe_finalize_tilted_bed", expected, actual);
    // Slopes are floats here, so this compares the `%s` rendering of a
    // float tuple (`repr`) and not just the integer zero of a flat bed.
    let py_logs = recorder_values(&mut py, "LOG_LINES");
    assert_eq!(rs_logs.len(), py_logs.len(), "log line count (tilted)");
    for (i, (a, b)) in py_logs.iter().zip(rs_logs.iter()).enumerate() {
        let py_str = match a {
            Value::Str(s) => s.clone(),
            _ => panic!("LOG_LINES element is a string"),
        };
        check(&format!("log[{i}] (tilted)"), Ok(Value::str(&py_str)), Value::str(b));
    }
    // z adjustments pushed through the real helper must match the oracle.
    let py_adj = recorder_values(&mut py, "ADJUST_CALLS");
    let rs_zs: Vec<f64> = fx
        .toolhead
        .set_positions
        .lock()
        .unwrap()
        .iter()
        .map(|p| p[2])
        .collect();
    match &py_adj[0] {
        Value::Tuple(pair) => match &pair[0] {
            Value::List(vals) => {
                let adjs: Vec<f64> = vals
                    .iter()
                    .map(|v| match v {
                        Value::Float(f) => f.repr.parse().expect("adjustment float"),
                        _ => panic!("adjustment is a float"),
                    })
                    .collect();
                let expected_zs = expected_set_positions(&adjs);
                assert_eq!(rs_zs.len(), expected_zs.len(), "set_position count (tilted)");
                for (i, (a, b)) in expected_zs.iter().zip(rs_zs.iter()).enumerate() {
                    check(&format!("adjust z[{i}] (tilted)"), Ok(Value::float(*a)), Value::float(*b));
                }
            }
            other => panic!("adjust shape {other:?}"),
        },
        other => panic!("adjust shape {other:?}"),
    }
}

#[test]
fn probe_finalize_max_adjust_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(0.001, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.1], [100.0, 10.0, 0.2], [90.0, 90.0, 0.3], [10.0, 100.0, 0.4]];
    let fx = make_fixture(Some(0.001), Some(5.0), vec![[0.0, 0.0], [10.0, 10.0]], vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
    start_cmd_both(&mut py, &fx);
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos)));
    let actual = fx.quad.probe_finalize(&rust_positions(&pos));
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must raise CommandError"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("rust returned {v:?} but must raise"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "max_adjust class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "max_adjust message");
    assert_eq!(rust_err.class_name(), "CommandError", "gcode.error is CommandError");
    // No adjustment is applied when the check fires.
    assert!(fx.toolhead.set_positions.lock().unwrap().is_empty(), "no adjust on abort");
}

/// `max_adjust` defaults to 4 and `horizontal_move_z` to 5.0: both are
/// absent from the config here, so the values come from the defaults the
/// constructor passes. The abort message and the probe-point report show
/// them.
#[test]
fn config_defaults_used() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    clear_log_lines();
    py.exec("set_scenario(None, None, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.1], [100.0, 10.0, 0.2], [90.0, 90.0, 0.3], [10.0, 100.0, 0.4]];
    let fx = make_fixture(
        None,
        None,
        vec![[0.0, 0.0], [100.0, 100.0]],
        vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
    );
    start_cmd_both(&mut py, &fx);
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos)));
    let actual = fx.quad.probe_finalize(&rust_positions(&pos));
    let rs_logs = take_log_lines();
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but the default max_adjust must abort"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("rust returned {v:?} but the default max_adjust must abort"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "defaults class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "defaults message");
    // `Gantry-relative probe points` carries `horizontal_move_z`.
    let py_infos = recorder_values(&mut py, "INFOS");
    let rs_infos = fx.gstate.lock().unwrap().infos.clone();
    assert_eq!(rs_infos.len(), py_infos.len(), "respond_info count (abort before adjust)");
    for (i, (a, b)) in py_infos.iter().zip(rs_infos.iter()).enumerate() {
        let py_str = match a {
            Value::Str(s) => s.clone(),
            _ => panic!("INFOS element is a string"),
        };
        check(&format!("defaults respond_info[{i}]"), Ok(Value::str(&py_str)), Value::str(b));
    }
    let py_logs = recorder_values(&mut py, "LOG_LINES");
    assert_eq!(rs_logs.len(), py_logs.len(), "defaults log line count");
    for (i, (a, b)) in py_logs.iter().zip(rs_logs.iter()).enumerate() {
        let py_str = match a {
            Value::Str(s) => s.clone(),
            _ => panic!("LOG_LINES element is a string"),
        };
        check(&format!("defaults log[{i}]"), Ok(Value::str(&py_str)), Value::str(b));
    }
}

/// `check_retry_result`: only `"done"` marks the gantry as applied, so a
/// pending retry leaves `get_status()['applied']` false.
#[test]
fn probe_finalize_retry_leaves_unapplied() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[10.0,10.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]], 'retry'); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.1], [100.0, 10.0, 0.2], [90.0, 90.0, 0.3], [10.0, 100.0, 0.4]];
    // `retries: 2` with a tight tolerance makes the real RetryHelper
    // report the same pending retry the Python fake returns.
    let fx = make_fixture_opts(
        Some(4.0),
        Some(5.0),
        vec![[0.0, 0.0], [10.0, 10.0]],
        vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
        2,
        0.0001,
    );
    start_cmd_both(&mut py, &fx);
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos))).expect("eval");
    let actual = fx.quad.probe_finalize(&rust_positions(&pos));
    check_str("retry result", Ok(expected), actual);
    let py_status = py.eval("qgl_obj.get_status(0.0)").expect("status");
    let rs_status = fx.quad.get_status(0.0);
    assert!(applied_of(&rs_status) == Some(false), "port leaves applied false");
    check("retry status", Ok(py_status), status_value(&rs_status));
    // The adjustment is still applied before the retry is reported.
    assert!(
        !fx.toolhead.set_positions.lock().unwrap().is_empty(),
        "adjust runs before check_retry"
    );
}

#[test]
fn cmd_dispatch_and_get_status() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let expected = py.eval("qgl_obj.cmd_QUAD_GANTRY_LEVEL(FakeGcmd())");
    let fx = make_fixture(Some(4.0), Some(5.0), vec![[0.0, 0.0], [100.0, 100.0]], vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
    let gcmd: GCmd = Arc::new(FakeGcmd);
    let actual = fx.quad.cmd_quad_gantry_level(&gcmd);
    let expected_ok = expected.is_ok();
    assert!(actual.is_ok(), "cmd raised {:?}", actual.err());
    assert!(expected_ok, "oracle cmd must succeed");
    // get_status: fresh object applied=False on both sides.
    let expected_status = py.eval("qgl_obj.get_status(0.0)");
    check(
        "cmd_dispatch get_status",
        expected_status,
        status_value(&fx.quad.get_status(0.0)),
    );
}

#[test]
fn motor_off_resets_applied() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]]); make_quad()")
        .expect("setup");
    let pos = [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [100.0, 100.0, 0.0], [0.0, 100.0, 0.0]];
    let fx = make_fixture(Some(4.0), Some(5.0), vec![[0.0, 0.0], [100.0, 100.0]], vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]);
    start_cmd_both(&mut py, &fx);
    let _ = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos))).expect("py finalize");
    fx.quad.probe_finalize(&rust_positions(&pos)).expect("rs finalize");
    // applied=True on both sides now.
    assert_eq!(applied_of(&fx.quad.get_status(0.0)), Some(true), "applied after finalize");
    // Python: FakeZAdjustStatus does not listen to motor_off, so drive
    // the reset through cmd instead; Rust: fire the real event.
    py.eval("qgl_obj.cmd_QUAD_GANTRY_LEVEL(FakeGcmd())").expect("py cmd");
    fx.printer_fp
        .fire(klippy_api::printer::EV_STEPPER_ENABLE_MOTOR_OFF)
        .expect("motor_off handlers");
    assert_eq!(applied_of(&fx.quad.get_status(0.0)), Some(false), "applied cleared by motor_off");
}

#[test]
fn need_four_probe_points_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0]])")
        .expect("setup");
    let expected = py.eval("make_quad()");
    let (gcode, _gstate) = FakeGcode::new();
    let toolhead = FakeToolhead::new();
    let fp = FakePrinter::new();
    fp.add("gcode", Arc::new(FakeGcodeAsObject { inner: gcode }) as ObjectRef);
    fp.add("toolhead", toolhead as ObjectRef);
    fp.add("manual_probe", Arc::new(FakeManualProbe) as ObjectRef);
    let printer: Printer = Arc::new(fp);
    let config: Config = Arc::new(FakeConfig {
        printer,
        max_adjust: Some(4.0),
        horiz: Some(5.0),
        retries: 0,
        retry_tolerance: 0.0,
        points: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
        corners: vec![[0.0, 0.0], [100.0, 100.0]],
    });
    let actual = QuadGantryLevel::new(&config);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must raise"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("rust built object but must raise"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "points class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "points message");
}

#[test]
fn need_gantry_corners_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0]], [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0]])")
        .expect("setup");
    let expected = py.eval("make_quad()");
    let (gcode, _gstate) = FakeGcode::new();
    let toolhead = FakeToolhead::new();
    let fp = FakePrinter::new();
    fp.add("gcode", Arc::new(FakeGcodeAsObject { inner: gcode }) as ObjectRef);
    fp.add("toolhead", toolhead as ObjectRef);
    fp.add("manual_probe", Arc::new(FakeManualProbe) as ObjectRef);
    let printer: Printer = Arc::new(fp);
    let config: Config = Arc::new(FakeConfig {
        printer,
        max_adjust: Some(4.0),
        horiz: Some(5.0),
        retries: 0,
        retry_tolerance: 0.0,
        points: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        corners: vec![[0.0, 0.0]],
    });
    let actual = QuadGantryLevel::new(&config);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must raise"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("rust built object but must raise"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "corners class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "corners message");
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::quad_gantry_level::module_factory();
    let load_config = factory.load_config.expect("quad exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no early objects");
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");
    py.exec("set_scenario(4.0, 5.0, [[0.0,0.0],[100.0,100.0]], [[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]])")
        .expect("setup");
    let (gcode, gstate) = FakeGcode::new();
    let toolhead = FakeToolhead::new();
    let fp = FakePrinter::new();
    fp.add("gcode", Arc::new(FakeGcodeAsObject { inner: gcode }) as ObjectRef);
    fp.add("toolhead", toolhead.clone() as ObjectRef);
    fp.add("manual_probe", Arc::new(FakeManualProbe) as ObjectRef);
    let printer: Printer = Arc::new(fp.clone());
    let config: Config = Arc::new(FakeConfig {
        printer,
        max_adjust: Some(4.0),
        horiz: Some(5.0),
        retries: 0,
        retry_tolerance: 0.0,
        points: vec![[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
        corners: vec![[0.0, 0.0], [100.0, 100.0]],
    });
    let obj: ObjectRef = load_config(&config).expect("factory").expect("object");
    fp.fire(klippy_api::printer::EV_KLIPPY_CONNECT)
        .expect("klippy:connect handlers");
    let rust: Arc<QuadGantryLevel> = downcast_arc(&obj).expect("downcast");
    check(
        "factory max_adjust",
        py.eval("MAX_ADJUST[0]"),
        Value::float(rust.max_adjust()),
    );
    check(
        "factory horizontal_move_z",
        py.eval("HORIZ[0]"),
        Value::float(rust.horizontal_move_z()),
    );
    assert_eq!(
        gstate.lock().unwrap().registered.len(),
        1,
        "QUAD_GANTRY_LEVEL command registered once"
    );
    assert_eq!(
        gstate.lock().unwrap().registered[0].0,
        "QUAD_GANTRY_LEVEL",
        "registered command name"
    );
    let pos = [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0], [100.0, 100.0, 0.0], [0.0, 100.0, 0.0]];
    py.exec("make_quad()").expect("setup2");
    py.eval("qgl_obj.cmd_QUAD_GANTRY_LEVEL(FakeGcmd())")
        .expect("py cmd start");
    let gcmd: GCmd = Arc::new(FakeGcmd);
    rust.cmd_quad_gantry_level(&gcmd).expect("rust cmd start");
    gstate.lock().unwrap().infos.clear();
    let expected = py.eval(&format!("qgl_obj.probe_finalize({})", positions_py(&pos)));
    let actual = rust.probe_finalize(&rust_positions(&pos));
    check_str("factory probe_finalize", expected, actual);
}
