//! Parity tests for `klippy/extras/z_tilt.py`.
//!
//! Expected values come from the frozen Python reference through
//! `pyoracle` (CPython 3.8.2 + `klippy/`): a Python `ZTilt` is built
//! with minimal fakes for the printer, config, G-code, toolhead and
//! steppers (plus a stub `extras.probe`), and the same inputs drive the
//! Rust port through its contract fakes. Compared: return values,
//! raised errors (class and message), G-code `respond_info` text, the
//! toolhead call sequence with stepper trapq attach tokens,
//! `get_status` dicts and log lines. Tests skip when `$KLIPPY_PYTHON`
//! is not set.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit (the recorder needs the real probe stack), so none are replayed.

// Test fakes are plain synchronised bags of state, not printer objects.
#![allow(clippy::disallowed_types, clippy::vec_init_then_push)]

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use extras::manual_probe::ProbeResult;
use extras::z_tilt::{ZAdjustStatus, ZTilt};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as CmdGetFloatOpts,
    GetIntOpts as CmdGetIntOpts, GetOpts as CmdGetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Stepper, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateHandle, HomingTargetApi, KinematicsApi, MoveApi, PrinterExtruder,
    ToolHeadApi,
};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes (inside the oracle process).
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, logging, configparser
from collections import namedtuple

probe_result = namedtuple('probe_result', ['bed_x', 'bed_y', 'bed_z', 'test_x', 'test_y', 'test_z'])

_probe_helpers = []
class ProbePointsHelper:
    def __init__(self, config, callback):
        self.callback = callback
        self.config = config
        _probe_helpers.append(self)
    def minimum_points(self, n): pass
    def start_probe(self, gcmd): pass
    def get_lift_speed(self): return 50.0
_stub = types.ModuleType('extras.probe')
_stub.ProbePointsHelper = ProbePointsHelper
sys.modules['extras.probe'] = _stub
import extras.z_tilt
import gcode as gcode_module

log_lines = []
class _Capture(logging.Handler):
    def emit(self, record): log_lines.append((record.levelname, record.getMessage()))
_root = logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

def _flist(values):
    return '[%s]' % ', '.join(repr(v) for v in values)

class FakeGCode:
    def __init__(self):
        self.responses = []
        self.raw = []
        self.commands = {}
    def respond_info(self, msg, log=True):
        self.responses.append(msg)
    def respond_raw(self, msg):
        self.raw.append(msg)
    def error(self, msg):
        return CommandError(msg)
    def register_command(self, cmd, func, desc=None):
        self.commands[cmd] = desc

ConfigError = configparser.Error
class CommandError(Exception): pass

class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.trapq_log = []
    def get_name(self): return self._name
    def is_active_axis(self, axis): return axis == 'z'
    def set_trapq(self, tq):
        self.trapq_log.append('none' if tq is None else 'trapq')

class FakeKinematics:
    def __init__(self, steppers):
        self._steppers = steppers
    def get_steppers(self): return list(self._steppers)

class FakeToolhead:
    def __init__(self, rig):
        self.rig = rig
    def get_kinematics(self):
        self.rig['calls'].append("('get_kinematics',)")
        return self.rig['kinematics']
    def get_position(self):
        self.rig['calls'].append("('get_position',)")
        return list(self.rig['position'])
    def set_position(self, pos):
        self.rig['calls'].append("('set_position', %s)" % _flist(pos))
        self.rig['position'] = list(pos)
    def move(self, pos, speed):
        self.rig['calls'].append("('move', %s, %s)" % (_flist(pos), repr(speed)))
        if self.rig.get('fail_move'):
            raise CommandError('move failed')
    def flush_step_generation(self):
        self.rig['calls'].append("('flush',)")
    def get_trapq(self):
        self.rig['calls'].append("('get_trapq',)")
        return self.rig['trapq']
    def manual_move(self, coord, speed):
        self.rig['calls'].append("('manual_move', %s, %s)" % (_flist(coord), repr(speed)))

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.handlers = []
    def lookup_object(self, name, default=None):
        if name in self.objects: return self.objects[name]
        if default is None: return None
        raise ConfigError('missing ' + name)
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def config_error(self, msg): return ConfigError(msg)
    def command_error(self, msg): return CommandError(msg)

class FakeConfig:
    def __init__(self, printer, name, values):
        self.printer = printer
        self.name = name
        self.values = values
    def get_printer(self): return self.printer
    def get_name(self): return self.name
    def getint(self, opt, default=None, minval=None, maxval=None, note_valid=True):
        return int(self.values.get(opt, default))
    def getfloat(self, opt, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
        return float(self.values.get(opt, default))
    def getlists(self, opt, default=None, seps=(',',), parser=str, count=None, note_valid=True):
        return self.values.get(opt, default)

def make_world(stepper_names, position=(10.0, 20.0, 5.0, 0.0)):
    printer = FakePrinter()
    gcode = FakeGCode()
    steppers = [FakeStepper(name) for name in stepper_names]
    rig = {'position': list(position), 'calls': [], 'trapq': object()}
    rig['kinematics'] = FakeKinematics(steppers)
    toolhead = FakeToolhead(rig)
    printer.objects['gcode'] = gcode
    printer.objects['toolhead'] = toolhead
    return printer, gcode, steppers, rig, toolhead

def make_config(printer, z_positions, retries=None, tolerance=None):
    values = {'z_positions': list(z_positions), 'points': [(0.0, 0.0), (100.0, 0.0), (50.0, 100.0)]}
    if retries is not None: values['retries'] = retries
    if tolerance is not None: values['retry_tolerance'] = tolerance
    return FakeConfig(printer, 'z_tilt', values)

def make_ztilt(printer, z_positions, retries=None, tolerance=None):
    return extras.z_tilt.ZTilt(make_config(printer, z_positions, retries, tolerance))

def make_gcmd(gcode, params):
    cmdline = 'Z_TILT_ADJUST ' + ' '.join('%s=%s' % (k, v) for k, v in params.items())
    return gcode_module.GCodeCommand(gcode, 'Z_TILT_ADJUST', cmdline, dict(params), False)

def fire_event(printer, event):
    for name, cb in list(printer.handlers):
        if name == event:
            cb()
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("z_tilt fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping z_tilt oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

#[derive(Default, Clone)]
struct Rig {
    position: Vec<f64>,
    calls: Vec<String>,
    fail_move: bool,
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
    trapq_log: Mutex<Vec<String>>,
}

impl FakeStepper {
    fn new(name: &str) -> Arc<Self> {
        Arc::new(FakeStepper {
            name: name.to_string(),
            trapq_log: Mutex::new(Vec::new()),
        })
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        unimplemented!()
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
        1.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (1.0, 0.0)
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
        self.trapq_log
            .lock()
            .unwrap()
            .push(if tq.is_some() {
                "trapq".to_string()
            } else {
                "none".to_string()
            });
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, axis: char) -> bool {
        axis == 'z'
    }
}

klippy_api::printer_object!(FakeStepper; dyn McuStepperApi);

struct FakeKinematics {
    steppers: Vec<Stepper>,
}

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        unimplemented!()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        unimplemented!()
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        unimplemented!()
    }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        unimplemented!()
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        unimplemented!()
    }
    fn clear_homing_state(&self, _clear_axes: &str) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakeToolhead {
    rig: Arc<Mutex<Rig>>,
    kinematics: klippy_api::toolhead::Kinematics,
}

fn fmt_floats(values: &[f64]) -> String {
    let parts: Vec<String> = values
        .iter()
        .map(|v| pycompat::text::float_repr(*v))
        .collect();
    format!("[{}]", parts.join(", "))
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.rig.lock().unwrap().calls.push("('get_position',)".to_string());
        self.rig.lock().unwrap().position.clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        self.rig
            .lock()
            .unwrap()
            .calls
            .push(format!("('set_position', {})", fmt_floats(&newpos)));
        self.rig.lock().unwrap().position = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        self.rig
            .lock()
            .unwrap()
            .calls
            .push("('get_kinematics',)".to_string());
        self.kinematics.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.rig.lock().unwrap().calls.push("('flush',)".to_string());
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        unimplemented!()
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.rig.lock().unwrap().calls.push(format!(
            "('move', {}, {})",
            fmt_floats(&newpos),
            pycompat::text::float_repr(speed)
        ));
        if self.rig.lock().unwrap().fail_move {
            return Err(KlippyError::Command("move failed".to_string()));
        }
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let parts: Vec<String> = coord
            .iter()
            .map(|v| {
                v.map(pycompat::text::float_repr)
                    .unwrap_or_else(|| "None".to_string())
            })
            .collect();
        self.rig.lock().unwrap().calls.push(format!(
            "('manual_move', [{}], {})",
            parts.join(", "),
            pycompat::text::float_repr(speed)
        ));
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        unimplemented!()
    }
    fn get_trapq(&self) -> Trapq {
        self.rig.lock().unwrap().calls.push("('get_trapq',)".to_string());
        Arc::new(FakeTrapq) as Trapq
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
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        unimplemented!()
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        unimplemented!()
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

#[derive(Default)]
struct GCodeShared {
    responses: Vec<String>,
    commands: Vec<String>,
    descs: BTreeMap<String, Option<String>>,
}

struct FakeGCode {
    shared: Arc<Mutex<GCodeShared>>,
    handlers: Mutex<BTreeMap<String, klippy_api::gcode::GCodeCommandHandler>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        let mut shared = self.shared.lock().unwrap();
        shared.commands.push(cmd.to_string());
        shared.descs.insert(cmd.to_string(), opts.desc);
        if let Some(f) = func {
            self.handlers.lock().unwrap().insert(cmd.to_string(), f);
            Ok(None)
        } else {
            Ok(self.handlers.lock().unwrap().remove(cmd))
        }
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        unimplemented!()
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        unimplemented!()
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        unimplemented!()
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        unimplemented!()
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGCodeCommand::new(
            command,
            commandline,
            params.clone(),
        ))
    }
    fn respond_raw(&self, msg: &str) {
        self.shared.lock().unwrap().responses.push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.shared.lock().unwrap().responses.push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        unimplemented!()
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        unimplemented!()
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

/// `GCodeCommandApi` backed by `klippy_core::gcode_parse`, so parameter errors
/// carry the reference class and exact text.
struct FakeGCodeCommand {
    inner: klippy_core::gcode_parse::GCodeCommand,
}

impl FakeGCodeCommand {
    fn new(command: &str, commandline: &str, params: IndexMap<String, String>) -> Self {
        FakeGCodeCommand {
            inner: klippy_core::gcode_parse::GCodeCommand::new(
                command.to_string(),
                commandline.to_string(),
                params,
                false,
            ),
        }
    }
    fn map_err(e: klippy_core::gcode_parse::CommandError) -> KlippyError {
        KlippyError::Command(e.0)
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: CmdGetOpts) -> KResult<PyValue> {
        let value = match self.inner.get_command_parameters().get(name) {
            Some(v) => v.clone(),
            None => match opts.default {
                None => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': missing {}",
                        self.inner.get_commandline(),
                        name
                    )))
                }
                Some(d) => return Ok(d),
            },
        };
        let commandline = self.inner.get_commandline().to_string();
        let unable = || {
            KlippyError::command(format!(
                "Error on '{}': unable to parse {}",
                commandline, value
            ))
        };
        let parsed = match opts.parser {
            GetParser::Str => PyValue::Str(value.clone()),
            GetParser::Int => value
                .parse::<i64>()
                .map(|n| PyValue::Int(PyInt::from_i64(n)))
                .map_err(|_| unable())?,
            GetParser::IntAutoBase => value
                .parse::<i64>()
                .map(|n| PyValue::Int(PyInt::from_i64(n)))
                .map_err(|_| unable())?,
            GetParser::Float => value
                .parse::<f64>()
                .map(PyValue::Float)
                .map_err(|_| unable())?,
        };
        let bound_str = |b: &PyValue| match b {
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Float(f) => pycompat::text::float_repr(*f),
            PyValue::Bool(b) => b.to_string(),
            _ => format!("{b:?}"),
        };
        let as_float = |v: &PyValue| match v {
            PyValue::Int(n) => n.0.to_string().parse::<f64>().unwrap_or(0.0),
            PyValue::Float(f) => *f,
            PyValue::Bool(b) => {
                if *b {
                    1.0
                } else {
                    0.0
                }
            }
            _ => 0.0,
        };
        let v = as_float(&parsed);
        if let Some(b) = opts.minval {
            if v < as_float(&b) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    commandline,
                    name,
                    bound_str(&b)
                )));
            }
        }
        if let Some(b) = opts.maxval {
            if v > as_float(&b) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    commandline,
                    name,
                    bound_str(&b)
                )));
            }
        }
        if let Some(b) = opts.above {
            if v <= as_float(&b) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be above {}",
                    commandline,
                    name,
                    bound_str(&b)
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= as_float(&b) {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be below {}",
                    commandline,
                    name,
                    bound_str(&b)
                )));
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: CmdGetIntOpts) -> KResult<Option<i64>> {
        self.inner
            .get_int_opt(name, opts.default, opts.minval, opts.maxval)
            .map_err(Self::map_err)
    }
    fn get_float(&self, name: &str, opts: CmdGetFloatOpts) -> KResult<Option<f64>> {
        self.inner
            .get_float_opt(
                name,
                opts.default,
                opts.minval,
                opts.maxval,
                opts.above,
                opts.below,
                opts.ints,
            )
            .map_err(Self::map_err)
    }
    fn get_command(&self) -> String {
        self.inner.get_command().to_string()
    }
    fn get_commandline(&self) -> String {
        self.inner.get_commandline().to_string()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.inner.get_command_parameters().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        klippy_core::gcode_parse::GCodeCommand::get_raw_command_parameters(&self.inner)
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakePrinter {
    objects: Mutex<BTreeMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn fire_event(&self, event: &str) -> KResult<Vec<PyValue>> {
        self.send_event(event, &[])
    }
    fn handler_names(&self) -> Vec<String> {
        self.handlers
            .lock()
            .unwrap()
            .iter()
            .map(|(e, _)| e.clone())
            .collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        unimplemented!()
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("Ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock().unwrap();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!("duplicate object {name}")));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().unwrap().get(name).cloned() {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("unknown object {name}")))
                }
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
        unimplemented!()
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .lock()
            .unwrap()
            .push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let mut handlers = self.handlers.lock().unwrap();
        for (event_name, cb) in handlers.iter_mut() {
            if event_name == event {
                results.push(cb(params)?);
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    values: Mutex<BTreeMap<String, PyValue>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        match self.values.lock().unwrap().get(option) {
            Some(PyValue::Str(s)) => Ok(Some(s.clone())),
            _ => Ok(None),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.values.lock().unwrap().get(option).cloned() {
            Some(PyValue::Int(n)) => Ok(Some(n.0.to_string().parse().unwrap())),
            _ => match opts.default {
                None => Err(KlippyError::Config(format!("missing {option}"))),
                Some(v) => Ok(v),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.values.lock().unwrap().get(option).cloned() {
            Some(PyValue::Float(f)) => Ok(Some(f)),
            _ => match opts.default {
                None => Err(KlippyError::Config(format!("missing {option}"))),
                Some(v) => Ok(v),
            },
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        unimplemented!()
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        unimplemented!()
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match self.values.lock().unwrap().get(option).cloned() {
            Some(v) => Ok(v),
            None => match opts.default {
                None => Err(KlippyError::Config(format!("missing {option}"))),
                Some(v) => Ok(v.unwrap_or(PyValue::None)),
            },
        }
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimplemented!()
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        unimplemented!()
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        unimplemented!()
    }
    fn getsection(&self, _section: &str) -> Config {
        unimplemented!()
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

// ---------------------------------------------------------------------------
// Log capture (the process-wide sink writes into a thread-local, one per test
// thread, so parallel tests do not see each other's lines).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

static SINK: std::sync::LazyLock<()> = std::sync::LazyLock::new(|| {
    let _ = log::set_sink(Box::new(|rec: &log::Record<'_>| {
        LOG_LINES.with(|l| {
            l.borrow_mut()
                .push((rec.level.name().to_string(), rec.message.to_string()))
        });
    }));
});

/// Install the sink and clear this thread's captured lines.
fn log_capture() {
    std::sync::LazyLock::force(&SINK);
    log_clear();
}

fn log_clear() {
    LOG_LINES.with(|l| l.borrow_mut().clear());
}

fn log_snapshot() -> Vec<(String, String)> {
    LOG_LINES.with(|l| l.borrow().clone())
}

// ---------------------------------------------------------------------------
// World builders and comparison helpers.
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    steppers: Vec<Arc<FakeStepper>>,
    rig: Arc<Mutex<Rig>>,
    config: Config,
    ztilt: Arc<ZTilt>,
}

fn z_positions_value(pairs: &[(f64, f64)]) -> PyValue {
    PyValue::Tuple(
        pairs
            .iter()
            .map(|(x, y)| PyValue::Tuple(vec![PyValue::Float(*x), PyValue::Float(*y)]))
            .collect(),
    )
}

fn build_world(
    z_positions: &[(f64, f64)],
    retries: Option<i64>,
    tolerance: Option<f64>,
    stepper_names: &[&str],
) -> World {
    let rig = Arc::new(Mutex::new(Rig {
        position: vec![10.0, 20.0, 5.0, 0.0],
        calls: Vec::new(),
        fail_move: false,
    }));
    let steppers: Vec<Arc<FakeStepper>> = stepper_names
        .iter()
        .map(|n| FakeStepper::new(n))
        .collect();
    let stepper_api: Vec<Stepper> = steppers
        .iter()
        .map(|s| s.clone() as Stepper)
        .collect();
    let kinematics = Arc::new(FakeKinematics {
        steppers: stepper_api,
    }) as klippy_api::toolhead::Kinematics;
    let toolhead = Arc::new(FakeToolhead {
        rig: rig.clone(),
        kinematics,
    });
    let gcode = Arc::new(FakeGCode {
        shared: Arc::new(Mutex::new(GCodeShared::default())),
        handlers: Mutex::new(BTreeMap::new()),
    });
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(BTreeMap::new()),
        handlers: Mutex::new(Vec::new()),
    });
    {
        let mut objects = printer.objects.lock().unwrap();
        objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        objects.insert("toolhead".to_string(), toolhead.clone() as ObjectRef);
    }
    let mut values = BTreeMap::new();
    values.insert("z_positions".to_string(), z_positions_value(z_positions));
    values.insert(
        "points".to_string(),
        z_positions_value(&[(0.0, 0.0), (100.0, 0.0), (50.0, 100.0)]),
    );
    if let Some(r) = retries {
        values.insert("retries".to_string(), PyValue::Int(PyInt::from_i64(r)));
    }
    if let Some(t) = tolerance {
        values.insert("retry_tolerance".to_string(), PyValue::Float(t));
    }
    let config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        name: "z_tilt".to_string(),
        values: Mutex::new(values),
    }) as Config;
    let ztilt = ZTilt::new(&config).expect("build ZTilt");
    World {
        printer,
        gcode,
        steppers,
        rig,
        config,
        ztilt,
    }
}

fn pr(bed_x: f64, bed_y: f64, bed_z: f64) -> ProbeResult {
    ProbeResult {
        bed_x,
        bed_y,
        bed_z,
        test_x: bed_x,
        test_y: bed_y,
        test_z: bed_z,
    }
}

fn empty_gcmd() -> klippy_api::gcode::GCmd {
    Arc::new(FakeGCodeCommand::new(
        "Z_TILT_ADJUST",
        "Z_TILT_ADJUST",
        IndexMap::new(),
    ))
}

fn positions_expr(positions: &[ProbeResult]) -> String {
    let items: Vec<String> = positions
        .iter()
        .map(|p| {
            format!(
                "probe_result({:?}, {:?}, {:?}, {:?}, {:?}, {:?})",
                p.bed_x, p.bed_y, p.bed_z, p.test_x, p.test_y, p.test_z
            )
        })
        .collect();
    format!("[{}]", items.join(", "))
}

fn mkrecord(function: &str, seq: u64) -> Record {
    Record {
        v: 1,
        unit: "extras.z_tilt".to_string(),
        scenario: function.to_string(),
        seq,
        function: function.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap_or(i64::MAX)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::Host(h) => Value::Opaque {
            type_name: "host".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn check_ret(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Result<Value, KlippyError>,
    function: &str,
) {
    let record = mkrecord(function, 1);
    match (expected, actual) {
        (Ok(expected), Ok(actual)) => {
            assert_equal(&record, "ret", &expected, &actual).expect("return value");
        }
        (Ok(expected), Err(e)) => {
            panic!(
                "{label}: oracle returned {expected:?}, port raised {}: {}",
                e.class_name(),
                e.message()
            )
        }
        (Err(exc), Ok(actual)) => {
            panic!(
                "{label}: oracle raised {}: {}, port returned {actual}",
                exc.class_name, exc.message
            )
        }
        (Err(exc), Err(e)) => {
            assert_eq!(
                e.class_name(),
                exc.class_name.as_str(),
                "{label} exception class"
            );
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
    }
}

fn check_val(label: &str, expected: pyoracle::OracleResult, actual: Value, function: &str) {
    let record = mkrecord(function, 1);
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    assert_equal(&record, "ret", &expected, &actual).expect("value");
}

fn str_list(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::Str(s.clone())).collect())
}

fn stepper_trapq_lists(world: &World) -> Vec<Value> {
    world
        .steppers
        .iter()
        .map(|s| Value::List(s.trapq_log.lock().unwrap().iter().map(|t| Value::Str(t.clone())).collect()))
        .collect()
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn construction_registers_handlers_and_command() {
    let mut py = oracle().expect("oracle");
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n",
    )
    .expect("python world");
    // Python order: motor_off (ZAdjustStatus) then klippy:connect (ZAdjustHelper).
    check_val(
        "event handlers",
        py.eval("[e for e, _ in printer.handlers]"),
        str_list(&world.printer.handler_names()),
        "ZTilt",
    );
    let (commands, desc) = {
        let shared = world.gcode.shared.lock().unwrap();
        (shared.commands.clone(), shared.descs.get("Z_TILT_ADJUST").cloned().flatten())
    };
    check_val("commands", py.eval("list(gcode.commands)"), str_list(&commands), "ZTilt");
    check_val(
        "Z_TILT_ADJUST help",
        py.eval("gcode.commands['Z_TILT_ADJUST']"),
        desc.map_or(Value::Null, Value::Str),
        "ZTilt",
    );
    check_val("status", py.eval("zt.get_status(0.0)"), pyvalue_to_value(&world.ztilt.get_status(0.0)), "ZTilt");
}

#[test]
fn load_config_builds_the_printer_object() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt_obj = extras.z_tilt.load_config(make_config(printer, [(0.0, 0.0), (200.0, 0.0)]))\n",
    )
    .expect("python world");
    let factory = extras::z_tilt::module_factory();
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_printer_objects.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    let load_config = factory.load_config.expect("z_tilt load_config");
    let loaded = load_config(&world.config)
        .expect("load_config")
        .expect("printer object");
    let expected = py.eval("zt_obj.get_status(0.0)");
    check_val(
        "loaded object status",
        expected,
        pyvalue_to_value(&loaded.get_status(0.0).unwrap().expect("get_status")),
        "load_config",
    );
    let expected_desc = py.eval("gcode.commands");
    let registered = world.gcode.shared.lock().unwrap();
    check_val(
        "registered commands",
        expected_desc,
        Value::Dict(
            registered
                .descs
                .iter()
                .map(|(cmd, desc)| {
                    (
                        Value::Str(cmd.clone()),
                        desc.as_ref()
                            .map(|d| Value::Str(d.clone()))
                            .unwrap_or(Value::Null),
                    )
                })
                .collect(),
        ),
        "load_config",
    );
}

#[test]
fn probe_finalize_empty_results_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n\
         log_lines.clear()\n",
    )
    .expect("python world");
    let expected = py.eval("zt.probe_finalize([])");
    let actual = world.ztilt.probe_finalize(&[]).map(Value::Str);
    check_ret("empty probe results", expected, actual, "probe_finalize");
    let expected_logs = py.eval("[msg for _, msg in log_lines]");
    let rust_logs: Vec<Value> = log_snapshot()
        .iter()
        .filter(|(lvl, _)| lvl == "INFO")
        .map(|(_, msg)| Value::Str(msg.clone()))
        .collect();
    check_val(
        "log before the IndexError",
        expected_logs,
        Value::List(rust_logs),
        "probe_finalize",
    );
}

#[test]
fn probe_finalize_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0), (0.0, 200.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2", "stepper_z3"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2', 'stepper_z3'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0), (0.0, 200.0)])\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n",
    )
    .expect("python world");
    world
        .printer
        .fire_event("klippy:connect")
        .expect("rust handle_connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    world.rig.lock().unwrap().calls.clear();
    py.exec("rig['calls'].clear()").expect("clear py calls");
    for s in &world.steppers {
        s.trapq_log.lock().unwrap().clear();
    }
    log_lines_clear(&mut py);
    log_clear();

    let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 2.0), pr(0.0, 100.0, 3.0)];
    let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
    let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
    check_ret("probe_finalize return", expected, actual, "probe_finalize");

    let expected_calls = py.eval("rig['calls']");
    check_val(
        "toolhead calls",
        expected_calls,
        str_list(&world.rig.lock().unwrap().calls),
        "probe_finalize",
    );
    let expected_trapq = py.eval("[s.trapq_log for s in steppers]");
    check_val(
        "stepper trapq",
        expected_trapq,
        Value::List(stepper_trapq_lists(&world)),
        "probe_finalize",
    );

    let expected_resp = py.eval("gcode.responses");
    check_val(
        "adjustment report",
        expected_resp,
        str_list(&world.gcode.shared.lock().unwrap().responses),
        "probe_finalize",
    );

    let expected_status = py.eval("zt.get_status(0.0)");
    check_val(
        "status",
        expected_status,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "probe_finalize",
    );

    let expected_logs = py.eval("[msg for _, msg in log_lines]");
    let rust_logs: Vec<Value> = log_snapshot()
        .iter()
        .filter(|(lvl, _)| lvl == "INFO")
        .map(|(_, msg)| Value::Str(msg.clone()))
        .collect();
    check_val("log messages", expected_logs, Value::List(rust_logs), "probe_finalize");
}

fn log_lines_clear(py: &mut pyoracle::Oracle) {
    py.exec("log_lines.clear()").expect("clear py logs");
}

#[test]
fn probe_finalize_retries_when_out_of_tolerance() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        Some(2),
        Some(0.0001),
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)], 2, 0.0001)\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("rust connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 1.5)];
    let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
    let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
    check_ret("retry result", expected, actual, "probe_finalize");
    let expected_resp = py.eval("gcode.responses");
    check_val(
        "retry responses",
        expected_resp,
        str_list(&world.gcode.shared.lock().unwrap().responses),
        "probe_finalize",
    );
    let expected_status = py.eval("zt.get_status(0.0)");
    check_val(
        "status after retry",
        expected_status,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "probe_finalize",
    );
}

#[test]
fn retry_start_parameter_errors() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)], 2, 0.1)\n",
    )
    .expect("python world");
    let cases = [
        (
            "Z_TILT_ADJUST RETRIES=31",
            IndexMap::from([("RETRIES".to_string(), "31".to_string())]),
        ),
        (
            "Z_TILT_ADJUST RETRY_TOLERANCE=1.5",
            IndexMap::from([("RETRY_TOLERANCE".to_string(), "1.5".to_string())]),
        ),
        (
            "Z_TILT_ADJUST RETRY_TOLERANCE=-0.1",
            IndexMap::from([("RETRY_TOLERANCE".to_string(), "-0.1".to_string())]),
        ),
        (
            "Z_TILT_ADJUST RETRIES=abc",
            IndexMap::from([("RETRIES".to_string(), "abc".to_string())]),
        ),
    ];
    for (cmdline, params) in cases {
        let ps = params
            .iter()
            .map(|(k, v)| format!("{k:?}: {v:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "zt.retry_helper.start(make_gcmd(gcode, {{{ps}}}))"
        ));
        let world = build_world(
            &[(0.0, 0.0), (200.0, 0.0)],
            Some(2),
            Some(0.1),
            &["stepper_z1", "stepper_z2"],
        );
        let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCodeCommand::new("Z_TILT_ADJUST", cmdline, params));
        let actual = world
            .ztilt
            .retry_helper()
            .start(&gcmd)
            .map(|_| Value::Null);
        check_ret(&format!("start {cmdline}"), expected, actual, "start");
    }
}

#[test]
fn too_many_retries_raises() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        Some(1),
        Some(0.01),
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)], 1, 0.01)\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    for (i, &range) in [0.5f64, 0.6, 0.7].iter().enumerate() {
        let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 1.0 + range)];
        let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
        let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
        check_ret(
            &format!("retry sequence step {i} (range {range:?})"),
            expected,
            actual,
            "probe_finalize",
        );
    }
}

#[test]
fn increasing_range_aborts() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        Some(30),
        Some(0.0001),
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)], 30, 0.0001)\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    for (i, &range) in [0.4f64, 0.5, 0.6, 0.7].iter().enumerate() {
        let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 1.0 + range)];
        let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
        let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
        check_ret(&format!("increasing step {i}"), expected, actual, "probe_finalize");
    }
}

#[test]
fn handle_connect_config_errors() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(&[(0.0, 0.0), (200.0, 0.0)], None, None, &["stepper_z1"]);
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n",
    )
    .expect("python world");
    let expected = py.eval("zt.z_helper.handle_connect()");
    let actual = world
        .printer
        .fire_event("klippy:connect")
        .map(|_| Value::Null);
    check_ret("count mismatch", expected, actual, "handle_connect");

    let world2 = build_world(&[(0.0, 0.0)], None, None, &["stepper_z1"]);
    py.exec(
        "printer2, gcode2, steppers2, rig2, toolhead2 = make_world(['stepper_z1'])\n\
         zt2 = make_ztilt(printer2, [(0.0, 0.0)])\n",
    )
    .expect("python world 2");
    let expected2 = py.eval("zt2.z_helper.handle_connect()");
    let actual2 = world2
        .printer
        .fire_event("klippy:connect")
        .map(|_| Value::Null);
    check_ret("needs multiple", expected2, actual2, "handle_connect");
}

#[test]
fn adjust_steppers_move_failure_reattaches() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n\
         zt.z_helper.handle_connect()\n\
         rig['fail_move'] = True\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world.rig.lock().unwrap().fail_move = true;
    world.rig.lock().unwrap().calls.clear();
    py.exec("rig['calls'].clear()").expect("clear");
    log_clear();
    let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 2.0)];
    let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
    let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
    check_ret("move failure", expected, actual, "probe_finalize");
    let expected_calls = py.eval("rig['calls']");
    check_val(
        "failure calls",
        expected_calls,
        str_list(&world.rig.lock().unwrap().calls),
        "probe_finalize",
    );
    let error_logs: Vec<String> = log_snapshot()
        .iter()
        .filter(|(lvl, _)| lvl == "ERROR")
        .map(|(_, msg)| msg.clone())
        .collect();
    assert!(
        error_logs.iter().any(|m| m == "ZAdjustHelper adjust_steppers"),
        "{error_logs:?}"
    );
}

#[test]
fn cmd_resets_applied_before_param_error() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        Some(2),
        Some(0.01),
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)], 2, 0.01)\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n\
         zt.probe_finalize([probe_result(0.0, 0.0, 1.0, 0.0, 0.0, 1.0), probe_result(100.0, 0.0, 1.001, 100.0, 0.0, 1.001)])\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 1.001)];
    world.ztilt.probe_finalize(&positions).expect("done");
    let expected_status = py.eval("zt.get_status(0.0)");
    check_val(
        "applied after probe_finalize",
        expected_status,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "probe_finalize",
    );
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCodeCommand::new(
        "Z_TILT_ADJUST",
        "Z_TILT_ADJUST RETRIES=31",
        IndexMap::from([("RETRIES".to_string(), "31".to_string())]),
    ));
    let expected = py.eval("zt.cmd_Z_TILT_ADJUST(make_gcmd(gcode, {'RETRIES': '31'}))");
    let actual = world.ztilt.cmd_z_tilt_adjust(&gcmd).map(|_| Value::Null);
    check_ret("cmd param error", expected, actual, "start");
    let expected_status = py.eval("zt.get_status(0.0)");
    check_val(
        "reset happens before the parameter error",
        expected_status,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "cmd_Z_TILT_ADJUST",
    );
}

#[test]
fn motor_off_resets_applied_via_event() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n\
         zt.probe_finalize([probe_result(0.0, 0.0, 1.0, 0.0, 0.0, 1.0), probe_result(100.0, 0.0, 1.0, 100.0, 0.0, 1.0)])\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    world
        .ztilt
        .probe_finalize(&[pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 1.0)])
        .expect("done");
    let expected = py.eval("zt.get_status(0.0)");
    check_val(
        "applied after probe_finalize",
        expected,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "get_status",
    );
    py.exec("fire_event(printer, 'stepper_enable:motor_off')")
        .expect("python motor_off");
    world
        .printer
        .fire_event("stepper_enable:motor_off")
        .expect("motor_off");
    let expected = py.eval("zt.get_status(0.0)");
    check_val(
        "applied after motor_off",
        expected,
        pyvalue_to_value(&world.ztilt.get_status(0.0)),
        "get_status",
    );
}

#[test]
fn sorted_adjustment_order_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0), (100.0, 100.0)],
        Some(2),
        Some(0.1),
        &["stepper_z1", "stepper_z2", "stepper_z3"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2', 'stepper_z3'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0), (100.0, 100.0)], 2, 0.1)\n\
         zt.z_helper.handle_connect()\n\
         zt.retry_helper.start(make_gcmd(gcode, {}))\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    world
        .ztilt
        .retry_helper()
        .start(&empty_gcmd())
        .expect("rust retry start");
    world.rig.lock().unwrap().calls.clear();
    py.exec("rig['calls'].clear()").expect("clear");
    let positions = [pr(0.0, 0.0, 0.5), pr(100.0, 0.0, 0.25), pr(50.0, 100.0, 0.75)];
    let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
    let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
    check_ret("sorted adjust", expected, actual, "probe_finalize");
    let expected_calls = py.eval("rig['calls']");
    check_val(
        "sorted calls",
        expected_calls,
        str_list(&world.rig.lock().unwrap().calls),
        "probe_finalize",
    );
}

#[test]
fn status_direct_use() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         st = extras.z_tilt.ZAdjustStatus(printer)\n\
         status_log = []\n\
         status_log.append(st.check_retry_result('done'))\n\
         status_log.append(st.get_status(0.0))\n\
         status_log.append(st.check_retry_result('retry'))\n\
         status_log.append(st.get_status(0.0))\n\
         st.reset()\n\
         status_log.append(st.get_status(0.0))\n\
         fire_event(printer, 'stepper_enable:motor_off')\n\
         status_log.append(st.get_status(0.0))\n",
    )
    .expect("python world");
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(BTreeMap::new()),
        handlers: Mutex::new(Vec::new()),
    });
    let as_printer: Printer = printer.clone();
    let status = ZAdjustStatus::new(&as_printer);
    let mut actual = Vec::new();
    actual.push(pyvalue_to_value(&PyValue::Str(
        status.check_retry_result("done"),
    )));
    actual.push(pyvalue_to_value(&status.get_status(0.0)));
    actual.push(pyvalue_to_value(&PyValue::Str(
        status.check_retry_result("retry"),
    )));
    actual.push(pyvalue_to_value(&status.get_status(0.0)));
    status.reset();
    actual.push(pyvalue_to_value(&status.get_status(0.0)));
    printer
        .fire_event("stepper_enable:motor_off")
        .expect("motor_off");
    actual.push(pyvalue_to_value(&status.get_status(0.0)));
    let expected = py.eval("status_log");
    check_val("status sequence", expected, Value::List(actual), "get_status");
}

#[test]
fn probe_finalize_without_retry_start_raises() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    let world = build_world(
        &[(0.0, 0.0), (200.0, 0.0)],
        None,
        None,
        &["stepper_z1", "stepper_z2"],
    );
    py.exec(
        "printer, gcode, steppers, rig, toolhead = make_world(['stepper_z1', 'stepper_z2'])\n\
         zt = make_ztilt(printer, [(0.0, 0.0), (200.0, 0.0)])\n\
         zt.z_helper.handle_connect()\n",
    )
    .expect("python world");
    world.printer.fire_event("klippy:connect").expect("connect");
    let positions = [pr(0.0, 0.0, 1.0), pr(100.0, 0.0, 2.0)];
    let expected = py.eval(&format!("zt.probe_finalize({})", positions_expr(&positions)));
    let actual = world.ztilt.probe_finalize(&positions).map(Value::Str);
    check_ret(
        "probe_finalize before retry start",
        expected,
        actual,
        "probe_finalize",
    );
    let expected_calls = py.eval("rig['calls']");
    check_val(
        "calls before the AttributeError",
        expected_calls,
        str_list(&world.rig.lock().unwrap().calls),
        "probe_finalize",
    );
    let expected_resp = py.eval("gcode.responses");
    check_val(
        "reported adjustments",
        expected_resp,
        str_list(&world.gcode.shared.lock().unwrap().responses),
        "probe_finalize",
    );
}
