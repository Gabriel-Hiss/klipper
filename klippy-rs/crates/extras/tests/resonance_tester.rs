//! Parity tests for `extras::resonance_tester`
//! (port of `klippy/extras/resonance_tester.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`:
//! the Python side builds the real classes with minimal printer/config/gcode
//! fakes (`SETUP` below) and the Rust side drives the port through fakes of
//! the same contract traits. Compared: `TestAxis` names/directions/matches,
//! `_parse_axis` results and errors, both generator sequences bit-for-bit,
//! generator config defaults and bounds, `lookup_z_limits`, the tester's
//! config-derived fields and chip grouping, `connect`/`_parse_chips` errors,
//! filename and NAME-suffix validation, the registered command table,
//! `TEST_RESONANCES`/`SHAPER_CALIBRATE`/`MEASURE_AXES_NOISE` argument errors,
//! the executor's move stream and `SET_VELOCITY_LIMIT` scripts, the raw-data
//! and resonances outputs, the shaper recommendation, and the `configfile`
//! writes.
use extras::adxl345::{AccelChipApi, AccelQueryHelper};
use extras::input_shaper::InputShaperApi;
use extras::resonance_tester::{
    lookup_z_limits, module_factory, parse_axis, ResonanceTester, SweepingVibrationsTestGenerator,
    TestAxis, VibrationPulseTestGenerator,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, KinematicsApi, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python fakes: the reference classes with minimal dependencies.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.resonance_tester as rt
import extras.adxl345 as adxl345_mod
import extras.shaper_calibrate as shaper_calibrate_mod

# The reference runs the heavy calculations in a forked child process; forking
# the oracle interpreter is unsafe, so run the same methods inline.
def _inline_background_process_exec(self, method, args):
    return method(*args)
shaper_calibrate_mod.ShaperCalibrate.background_process_exec = _inline_background_process_exec

_sentinel = object()

class CommandError(Exception):
    pass

class FakeReactor:
    def __init__(self):
        self.now = 0.0
    def monotonic(self):
        return self.now
    def pause(self, waketime):
        self.now = waketime
        return waketime

class FakeToolHead:
    def __init__(self):
        self.moves = []
        self.move_time = 0.0
        self.pos = [10.0, 20.0, 5.0, 0.0]
    def get_position(self):
        return self.pos
    def get_last_move_time(self):
        t = self.move_time
        self.move_time += 5.0
        return t
    def get_status(self, eventtime):
        return {'max_velocity': 100.0, 'max_accel': 2000.0,
                'minimum_cruise_ratio': 0.5, 'square_corner_velocity': 5.0}
    def limit_next_junction_speed(self, speed):
        self.moves.append(('limit', speed))
    def set_max_velocities(self, mv, ma, scv, mcr):
        self.moves.append(('setmax', mv, ma, scv, mcr))
        return (100.0, 2000.0, 5.0, 0.5)
    def move(self, newpos, speed):
        self.moves.append(('move', list(newpos), speed))
    def manual_move(self, coord, speed):
        self.moves.append(('manual', list(coord), speed))
    def wait_moves(self):
        self.moves.append(('wait',))
    def dwell(self, delay):
        self.moves.append(('dwell', delay))

class FakeGCmd:
    error = CommandError
    def __init__(self, params=None, commandline='TEST_RESONANCES'):
        self.params = params or {}
        self.commandline = commandline
        self.infos = []
    def get(self, name, default=_sentinel, parser=str, minval=None, maxval=None,
            above=None, below=None):
        if name not in self.params:
            if default is _sentinel:
                raise self.error("Error on '%s': missing %s"
                                 % (self.commandline, name))
            return default
        value = self.params[name]
        try:
            value = parser(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s"
                             % (self.commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s"
                             % (self.commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s"
                             % (self.commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s"
                             % (self.commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s"
                             % (self.commandline, name, below))
        return value
    def get_float(self, name, default=_sentinel, minval=None, maxval=None,
                  above=None, below=None):
        return self.get(name, default, float, minval, maxval, above, below)
    def get_int(self, name, default=_sentinel, minval=None, maxval=None):
        return self.get(name, default, int, minval, maxval)
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))

class FakeGCode:
    def __init__(self):
        self.registered = {}
        self.scripts = []
        self.infos = []
    def register_command(self, cmd, func, desc=None):
        self.registered[cmd] = (func is not None, desc)
    def run_script_from_command(self, script):
        self.scripts.append(script)
    def create_gcode_command(self, command, commandline, params):
        return FakeGCmd(params, commandline)
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.reactor = FakeReactor()
        self.gcode = FakeGCode()
        self.toolhead = FakeToolHead()
        self.configfile = FakeConfigFile({'printer': {'max_velocity': 100.0,
                                                      'max_accel': 2000.0}})
        self.handlers = {}
    def lookup_object(self, name, default=_sentinel):
        if name == 'gcode':
            return self.gcode
        if name == 'toolhead':
            return self.toolhead
        if name == 'configfile' and name not in self.objects:
            return self.configfile
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def get_reactor(self):
        return self.reactor
    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, opts=None, lists=None):
        self._printer = printer
        self.opts = opts or {}
        self.lists = lists or {}
        self.section = 'resonance_tester'
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def _missing(self, option):
        return self.error("Option '%s' in section '%s' must be specified"
                          % (option, self.section))
    def get(self, option, default=_sentinel, note_valid=True):
        if option in self.opts:
            return self.opts[option]
        if default is _sentinel:
            raise self._missing(option)
        return default
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        if option in self.opts:
            v = float(self.opts[option])
        elif default is _sentinel:
            raise self._missing(option)
        else:
            return default
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s"
                             % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s"
                             % (option, self.section, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s"
                             % (option, self.section, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s"
                             % (option, self.section, below))
        return v
    def getlists(self, option, default=_sentinel, seps=(',', '\n'), count=None,
                 parser=str, note_valid=True):
        if option not in self.lists:
            if default is _sentinel:
                raise self._missing(option)
            return default
        return self.lists[option]

class FakeConfigFile:
    def __init__(self, settings):
        self.settings = settings
        self.sets = []
    def get_status(self, eventtime):
        return {'settings': self.settings}
    def set(self, section, option, value):
        self.sets.append((section, option, value))

class PyChip:
    # The real AccelQueryHelper, so the sample window and filtering are the
    # reference implementation rather than a hand-written stand-in.
    def __init__(self, printer, name, rows=None):
        self.printer = printer
        self.name = name
        self.rows = rows or []
    def start_internal_client(self):
        client = adxl345_mod.AccelQueryHelper(self.printer)
        if self.rows:
            client.handle_batch({'data': [tuple(r) for r in self.rows]})
        return client
    def read_reg(self, reg):
        return 0
    def set_reg(self, reg, val):
        pass

class PyNotAChip:
    name = 'notachip'

class PyInputShaper:
    def __init__(self):
        self.calls = []
    def disable_shaping(self):
        self.calls.append('disable')
    def enable_shaping(self):
        self.calls.append('enable')
    def cmd_SET_INPUT_SHAPER(self, gcmd):
        self.calls.append('cmd ' + ','.join(sorted(gcmd.params.keys())))

# Deterministic accelerometer data: 1000 samples at 1 kHz, all zero. The PSD
# of a constant signal is exactly zero in both implementations, so the noise
# and calibration outputs carry no float-rounding dependence.
ZERO_ROWS = [(i * 0.001, 0.0, 0.0, 0.0) for i in range(1000)]

def make_printer():
    return FakePrinter()

def make_tester(printer, opts=None, lists=None, objects=None):
    for name, obj in (objects or {}).items():
        printer.objects[name] = obj
    cfg = FakeConfig(printer, opts, lists)
    return rt.ResonanceTester(cfg)

def default_lists(points):
    return {'probe_points': tuple(tuple(p) for p in points)}
"#;

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: kos::sync::Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.lock().get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unknown config object '{name}'")))
            }
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
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
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

fn py_float_repr(v: f64) -> String {
    pycompat::text::float_repr(v)
}

fn opt_repr(v: Option<f64>) -> String {
    v.map(py_float_repr).unwrap_or_else(|| "None".to_string())
}

#[derive(Default)]
struct FakeGCodeInner {
    registered: IndexMap<String, (bool, Option<String>)>,
    scripts: Vec<String>,
    infos: Vec<(String, bool)>,
}

struct FakeGCode {
    inner: kos::sync::Mutex<FakeGCodeInner>,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        Arc::new(FakeGCode { inner: kos::sync::Mutex::new(FakeGCodeInner::default()) })
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let has_func = func.is_some();
        self.inner
            .lock()
            .registered
            .insert(cmd.to_string(), (has_func, opts.desc.clone()));
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
    fn run_script(&self, script: &str) -> KResult<()> {
        self.inner.lock().scripts.push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.inner.lock().scripts.push(script.to_string());
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("mutex unused")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGCmd::new(params.clone(), commandline.to_string(), command))
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        self.inner.lock().infos.push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCmd {
    params: IndexMap<String, String>,
    commandline: String,
    command: String,
    infos: kos::sync::Mutex<Vec<(String, bool)>>,
}

impl FakeGCmd {
    fn new(
        params: IndexMap<String, String>,
        commandline: String,
        command: &str,
    ) -> FakeGCmd {
        FakeGCmd {
            params,
            commandline,
            command: command.to_string(),
            infos: kos::sync::Mutex::new(Vec::new()),
        }
    }
    fn arc(params: &[(&str, &str)]) -> Arc<FakeGCmd> {
        Arc::new(FakeGCmd::new(
            params_map(params),
            "TEST_RESONANCES".to_string(),
            "TEST_RESONANCES",
        ))
    }
}

fn params_map(params: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut map = IndexMap::new();
    for &(k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    map
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(default) => Ok(default),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(default) => Ok(default),
            },
            Some(v) => {
                let n: i64 = match v.parse() {
                    Ok(n) => n,
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {}",
                            self.commandline, v
                        )))
                    }
                };
                self.check_int_bounds(name, n, opts.minval, opts.maxval)?;
                Ok(Some(n))
            }
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(default) => Ok(default),
            },
            Some(v) => {
                let n = match pycompat::text::float_from_string(v) {
                    Ok(n) => n,
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {}",
                            self.commandline, v
                        )))
                    }
                };
                if let Some(minval) = opts.minval {
                    if n < minval {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {} must have minimum of {}",
                            self.commandline,
                            name,
                            py_float_repr(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if n > maxval {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {} must have maximum of {}",
                            self.commandline,
                            name,
                            py_float_repr(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if n <= above {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {} must be above {}",
                            self.commandline,
                            name,
                            py_float_repr(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if n >= below {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {} must be below {}",
                            self.commandline,
                            name,
                            py_float_repr(below)
                        )));
                    }
                }
                Ok(Some(n))
            }
        }
    }
    fn get_command(&self) -> String {
        self.command.clone()
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
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

impl FakeGCmd {
    fn check_int_bounds(
        &self,
        name: &str,
        n: i64,
        minval: Option<i64>,
        maxval: Option<i64>,
    ) -> KResult<()> {
        if let Some(minval) = minval {
            if n < minval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline, name, minval
                )));
            }
        }
        if let Some(maxval) = maxval {
            if n > maxval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline, name, maxval
                )));
            }
        }
        Ok(())
    }
}

#[derive(Default)]
struct FakeToolHeadInner {
    moves: Vec<String>,
    move_time: f64,
}

struct FakeToolHead {
    inner: kos::sync::Mutex<FakeToolHeadInner>,
}

impl FakeToolHead {
    fn new() -> Arc<FakeToolHead> {
        Arc::new(FakeToolHead {
            inner: kos::sync::Mutex::new(FakeToolHeadInner {
                moves: Vec::new(),
                move_time: 0.0,
            }),
        })
    }
    fn push(&self, entry: String) {
        self.inner.lock().moves.push(entry);
    }
}

impl PrinterObject for FakeToolHead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ToolHeadApi, dyn HomingTargetApi);
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![10.0, 20.0, 5.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics) as Kinematics
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        let mut inner = self.inner.lock();
        let t = inner.move_time;
        inner.move_time += 5.0;
        Ok(t)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        self.push(format!("('dwell', {})", py_float_repr(delay)));
        Ok(())
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let list: Vec<String> = newpos.iter().map(|&v| py_float_repr(v)).collect();
        self.push(format!("('move', [{}], {})", list.join(", "), py_float_repr(speed)));
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let list: Vec<String> = coord.iter().map(|v| opt_repr(*v)).collect();
        self.push(format!("('manual', [{}], {})", list.join(", "), py_float_repr(speed)));
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> {
        self.push("('wait',)".to_string());
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("max_velocity".to_string()), PyValue::Float(100.0));
        d.insert(PyValue::Str("max_accel".to_string()), PyValue::Float(2000.0));
        d.insert(PyValue::Str("minimum_cruise_ratio".to_string()), PyValue::Float(0.5));
        d.insert(PyValue::Str("square_corner_velocity".to_string()), PyValue::Float(5.0));
        PyValue::Dict(d)
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (100.0, 2000.0)
    }
    fn set_max_velocities(
        &self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        self.push(format!(
            "('setmax', {}, {}, {}, {})",
            opt_repr(max_velocity),
            opt_repr(max_accel),
            opt_repr(square_corner_velocity),
            opt_repr(min_cruise_ratio)
        ));
        (100.0, 2000.0, 5.0, 0.5)
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, true)
    }
    fn limit_next_junction_speed(&self, speed: f64) {
        self.push(format!("('limit', {})", py_float_repr(speed)));
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("extruder unused")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("extruder unused")
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
        Arc::new(FakeStepperFactory)
    }
}

struct FakeTrapq;
impl klippy_api::stepper::TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}
klippy_api::printer_object!(FakeTrapq);

struct FakeKinematics;
impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(Vec::new())
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}
klippy_api::printer_object!(FakeKinematics);

struct FakeStepperFactory;
impl klippy_api::stepper::StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(
        &self,
        _config: &Config,
        _units_in_radians: bool,
    ) -> KResult<klippy_api::stepper::Stepper> {
        Err(KlippyError::config("unused"))
    }
    fn lookup_rail(
        &self,
        _config: &Config,
        _opts: klippy_api::stepper::RailOpts,
    ) -> KResult<klippy_api::stepper::Rail> {
        Err(KlippyError::config("unused"))
    }
    fn lookup_multi_rail(
        &self,
        _config: &Config,
        _opts: klippy_api::stepper::RailOpts,
    ) -> KResult<klippy_api::stepper::Rail> {
        Err(KlippyError::config("unused"))
    }
    fn generic_rail(
        &self,
        _config: &Config,
        _opts: klippy_api::stepper::RailOpts,
    ) -> KResult<klippy_api::stepper::Rail> {
        Err(KlippyError::config("unused"))
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
}
klippy_api::printer_object!(FakeStepperFactory);

struct FakeConfigFile {
    settings: IndexMap<PyValue, PyValue>,
    sets: kos::sync::Mutex<Vec<(String, String, String)>>,
}

impl FakeConfigFile {
    fn new(settings: IndexMap<PyValue, PyValue>) -> Arc<FakeConfigFile> {
        Arc::new(FakeConfigFile { settings, sets: kos::sync::Mutex::new(Vec::new()) })
    }
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("configfile printer unused")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::config("unused"))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::config("unused"))
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut outer = IndexMap::new();
        outer.insert(
            PyValue::Str("settings".to_string()),
            PyValue::Dict(self.settings.clone()),
        );
        PyValue::Dict(outer)
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.sets
            .lock()
            .push((section.to_string(), option.to_string(), value.to_string()));
    }
    fn remove_section(&self, _section: &str) {}
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

struct FakeChip {
    name: String,
    printer: Printer,
    rows: Vec<(f64, f64, f64, f64)>,
}

impl AccelChipApi for FakeChip {
    fn start_internal_client(&self) -> KResult<Arc<AccelQueryHelper>> {
        let client = Arc::new(AccelQueryHelper::new(self.printer.clone())?);
        if !self.rows.is_empty() {
            let data: Vec<PyValue> = self
                .rows
                .iter()
                .map(|&(t, x, y, z)| {
                    PyValue::Tuple(vec![
                        PyValue::Float(t),
                        PyValue::Float(x),
                        PyValue::Float(y),
                        PyValue::Float(z),
                    ])
                })
                .collect();
            let mut msg = IndexMap::new();
            msg.insert(PyValue::Str("data".to_string()), PyValue::List(data));
            client.handle_batch(PyValue::Dict(msg));
        }
        Ok(client)
    }
    fn read_reg(&self, _reg: i64) -> KResult<i64> {
        Ok(0)
    }
    fn set_reg(&self, _reg: i64, _val: i64) -> KResult<()> {
        Ok(())
    }
    fn name(&self) -> String {
        self.name.clone()
    }
}
klippy_api::printer_object!(FakeChip; dyn AccelChipApi);

/// An object that exists but is not an accelerometer (`hasattr` fails).
struct FakeNotAChip;
klippy_api::printer_object!(FakeNotAChip);

struct FakeInputShaper {
    calls: kos::sync::Mutex<Vec<String>>,
}

impl FakeInputShaper {
    fn new() -> Arc<FakeInputShaper> {
        Arc::new(FakeInputShaper { calls: kos::sync::Mutex::new(Vec::new()) })
    }
}

impl InputShaperApi for FakeInputShaper {
    fn disable_shaping(&self) -> KResult<()> {
        self.calls.lock().push("disable".to_string());
        Ok(())
    }
    fn enable_shaping(&self) -> KResult<()> {
        self.calls.lock().push("enable".to_string());
        Ok(())
    }
    fn cmd_set_input_shaper(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let mut keys: Vec<String> =
            gcmd.get_command_parameters().keys().cloned().collect();
        keys.sort();
        self.calls
            .lock()
            .push(format!("cmd {}", keys.join(",")));
        Ok(())
    }
}
klippy_api::printer_object!(FakeInputShaper; dyn InputShaperApi);

struct FakeConfigValues {
    printer: Printer,
    floats: IndexMap<String, f64>,
    strs: IndexMap<String, String>,
    lists: IndexMap<String, PyValue>,
}

impl ConfigApi for FakeConfigValues {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "resonance_tester".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.strs.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section 'resonance_tester' must be specified"
            ))),
            Some(default) => Ok(default),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section 'resonance_tester' must be specified"
            ))),
            Some(default) => Ok(default),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v = if let Some(v) = self.floats.get(option) {
            *v
        } else if let Some(raw) = self.strs.get(option) {
            pycompat::text::float_from_string(raw).expect("float option")
        } else {
            match opts.default {
                None => {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section 'resonance_tester' must be specified"
                    )))
                }
                Some(default) => return Ok(default),
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section 'resonance_tester' must have minimum of {}",
                    py_float_repr(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section 'resonance_tester' must have maximum of {}",
                    py_float_repr(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section 'resonance_tester' must be above {}",
                    py_float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section 'resonance_tester' must be below {}",
                    py_float_repr(below)
                )));
            }
        }
        Ok(Some(v))
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
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        if let Some(v) = self.lists.get(option) {
            return Ok(v.clone());
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section 'resonance_tester' must be specified"
            ))),
            Some(default) => Ok(default.unwrap_or(PyValue::None)),
        }
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
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
        panic!("getsection unused")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

// ---------------------------------------------------------------------------
// Test environment.
// ---------------------------------------------------------------------------

struct Env {
    printer: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    configfile: Arc<FakeConfigFile>,
}

impl Env {
    fn new() -> Env {
        let gcode = FakeGCode::new();
        let toolhead = FakeToolHead::new();
        let mut settings = IndexMap::new();
        let mut printer_section = IndexMap::new();
        printer_section.insert(
            PyValue::Str("max_velocity".to_string()),
            PyValue::Float(100.0),
        );
        printer_section.insert(PyValue::Str("max_accel".to_string()), PyValue::Float(2000.0));
        settings.insert(
            PyValue::Str("printer".to_string()),
            PyValue::Dict(printer_section),
        );
        let configfile = FakeConfigFile::new(settings);
        let printer: Printer = Arc::new(FakePrinter {
            objects: kos::sync::Mutex::new(IndexMap::new()),
            reactor: Reactor::new(false),
        });
        printer
            .add_object("gcode", Arc::clone(&gcode) as ObjectRef)
            .expect("add gcode");
        printer
            .add_object("toolhead", Arc::clone(&toolhead) as ObjectRef)
            .expect("add toolhead");
        printer
            .add_object("configfile", Arc::clone(&configfile) as ObjectRef)
            .expect("add configfile");
        Env { printer, gcode, toolhead, configfile }
    }

    fn add(&self, name: &str, obj: ObjectRef) {
        self.printer.add_object(name, obj).expect("add object");
    }

    fn add_chip(&self, name: &str, rows: &[(f64, f64, f64, f64)]) {
        self.add(
            name,
            Arc::new(FakeChip {
                name: name.split(' ').next_back().unwrap_or(name).to_string(),
                printer: self.printer.clone(),
                rows: rows.to_vec(),
            }) as ObjectRef,
        );
    }

    fn config(
        &self,
        floats: &[(&str, f64)],
        strs: &[(&str, &str)],
        points: &[(f64, f64, f64)],
    ) -> Config {
        let mut float_map = IndexMap::new();
        for &(k, v) in floats {
            float_map.insert(k.to_string(), v);
        }
        let mut str_map = IndexMap::new();
        for &(k, v) in strs {
            str_map.insert(k.to_string(), v.to_string());
        }
        let mut lists = IndexMap::new();
        let rows: Vec<PyValue> = points
            .iter()
            .map(|&(x, y, z)| {
                PyValue::Tuple(vec![
                    PyValue::Float(x),
                    PyValue::Float(y),
                    PyValue::Float(z),
                ])
            })
            .collect();
        lists.insert("probe_points".to_string(), PyValue::Tuple(rows));
        Arc::new(FakeConfigValues {
            printer: self.printer.clone(),
            floats: float_map,
            strs: str_map,
            lists,
        })
    }

    fn tester(
        &self,
        floats: &[(&str, f64)],
        strs: &[(&str, &str)],
        points: &[(f64, f64, f64)],
    ) -> Arc<ResonanceTester> {
        let config = self.config(floats, strs, points);
        let factory = module_factory();
        let obj = (factory.load_config.expect("load_config"))(&config)
            .expect("config accepted")
            .expect("object returned");
        klippy_api::downcast_arc::<ResonanceTester>(&obj).expect("ResonanceTester")
    }

    fn moves(&self) -> Vec<String> {
        self.toolhead.inner.lock().moves.clone()
    }

    fn scripts(&self) -> Vec<String> {
        self.gcode.inner.lock().scripts.clone()
    }

    fn sets(&self) -> Vec<(String, String, String)> {
        self.configfile.sets.lock().clone()
    }
}

// ---------------------------------------------------------------------------
// Assertion helpers.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("resonance_tester fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping resonance_tester oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "resonance_tester".to_string(),
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

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Result<impl std::fmt::Debug, KlippyError>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but the port must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but the call must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "{label}: class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "{label}: message");
}

fn floats(values: &[f64]) -> Value {
    Value::List(values.iter().map(|&v| Value::float(v)).collect())
}

fn tuple_of(values: &[f64]) -> Value {
    Value::Tuple(values.iter().map(|&v| Value::float(v)).collect())
}

fn seq_value(seq: &[(f64, f64, f64)]) -> Value {
    Value::List(
        seq.iter()
            .map(|&(t, a, f)| Value::Tuple(vec![Value::float(t), Value::float(a), Value::float(f)]))
            .collect(),
    )
}

fn strings(values: &[String]) -> Value {
    Value::List(values.iter().map(|v| Value::str(v)).collect())
}

fn infos_value(infos: &[(String, bool)]) -> Value {
    Value::List(
        infos
            .iter()
            .map(|(m, l)| Value::List(vec![Value::str(m), Value::Bool(*l)]))
            .collect(),
    )
}

fn sets_value(sets: &[(String, String, String)]) -> Value {
    Value::List(
        sets.iter()
            .map(|(s, o, v)| Value::List(vec![Value::str(s), Value::str(o), Value::str(v)]))
            .collect(),
    )
}

fn py_list_of_lists(expr: &str) -> String {
    format!("[list(x) for x in {expr}]")
}

// ---------------------------------------------------------------------------
// `TestAxis` and `_parse_axis`.
// ---------------------------------------------------------------------------

#[test]
fn test_axis_matches_python() {
    let Some(mut py) = oracle() else { return };
    for axis in ["x", "y", "z"] {
        let expected = py.eval(&format!(
            "[rt.TestAxis(axis='{axis}').get_name(), \
             rt.TestAxis(axis='{axis}').get_dir(), \
             rt.TestAxis(axis='{axis}').get_point(2.5), \
             [rt.TestAxis(axis='{axis}').matches(c) for c in ('x', 'y', 'z', 'xy', 'xyz', '')]]"
        ));
        let parsed = TestAxis::new_axis(axis).expect("axis");
        let actual = Value::List(vec![
            Value::str(parsed.get_name()),
            tuple_of(parsed.get_dir()),
            tuple_of(&parsed.get_point(2.5)),
            Value::List(
                ["x", "y", "z", "xy", "xyz", ""]
                    .iter()
                    .map(|c| Value::Bool(parsed.matches(c)))
                    .collect(),
            ),
        ]);
        check(&format!("TestAxis({axis})"), expected, actual);
    }
}

#[test]
fn test_axis_vib_dir_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases = [(1.0, 2.0, 3.0), (0.0, 1.0, 1.0), (-1.5, 0.5, 2.0), (0.0, 0.0, 1.0)];
    for (x, y, z) in cases {
        let dir = format!("rt.TestAxis(vib_dir=({x:?}, {y:?}, {z:?}))");
        let expected =
            py.eval(&format!("[{dir}.get_name(), {dir}.get_dir(), {dir}.get_point(-3.0)]"));
        let axis = TestAxis::new_vib_dir(&[x, y, z]).expect("vib_dir");
        let actual = Value::List(vec![
            Value::str(axis.get_name()),
            floats(axis.get_dir()),
            tuple_of(&axis.get_point(-3.0)),
        ]);
        check(&format!("TestAxis(vib_dir=({x}, {y}, {z}))"), expected, actual);
    }
}

#[test]
fn test_axis_constructor_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    // `ord()` needs exactly one character; `axis='a'` indexes out of the table.
    for axis in ["q", "xy", "", "a"] {
        let expected = py.eval(&format!("rt.TestAxis(axis={axis:?})"));
        let actual = TestAxis::new_axis(axis).map(|a| a.get_name().to_string());
        check_err(&format!("TestAxis(axis={axis:?})"), expected, actual);
    }
    // Negative list indexes wrap, so 'u', 'v' and 'w' select x, y and z.
    for axis in ["u", "v", "w"] {
        let expected = py.eval(&format!(
            "[rt.TestAxis(axis={axis:?}).get_name(), rt.TestAxis(axis={axis:?}).get_dir()]"
        ));
        let parsed = TestAxis::new_axis(axis).expect("wrapped index");
        let actual =
            Value::List(vec![Value::str(parsed.get_name()), tuple_of(parsed.get_dir())]);
        check(&format!("TestAxis(axis={axis:?})"), expected, actual);
    }
}

#[test]
fn parse_axis_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_gcmd = FakeGCmd({})").expect("gcmd");
    let gcmd = FakeGCmd::arc(&[]);
    let cases = ["x", "X", "Y", "z", "1,2", " 1 , 2 ", "0,1,0", "1,2,3", "-1.5,0.5,2"];
    for raw in cases {
        let expected = py.eval(&format!(
            "(lambda a: [a.get_name(), a.get_dir()])(rt._parse_axis(py_gcmd, {raw:?}))"
        ));
        let parsed = parse_axis(&*gcmd, Some(raw)).expect("parsed").expect("present");
        let actual = Value::List(vec![
            Value::str(parsed.get_name()),
            if raw.trim().len() == 1 && ["x", "y", "z"].contains(&raw.trim().to_lowercase().as_str()) {
                tuple_of(parsed.get_dir())
            } else {
                floats(parsed.get_dir())
            },
        ]);
        check(&format!("_parse_axis({raw:?})"), expected, actual);
    }
    let expected_none = py.eval("rt._parse_axis(py_gcmd, None)");
    check("_parse_axis(None)", expected_none, Value::Null);
}

#[test]
fn parse_axis_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_gcmd = FakeGCmd({})").expect("gcmd");
    let gcmd = FakeGCmd::arc(&[]);
    for raw in ["", "bad", "1", "1,2,3,4", "1,2,x", "x,y,z", "  ,  ", "0,0", "0,0,0"] {
        let expected = py.eval(&format!("rt._parse_axis(py_gcmd, {raw:?})"));
        let actual = parse_axis(&*gcmd, Some(raw)).map(|a| a.map(|v| v.get_name().to_string()));
        check_err(&format!("_parse_axis({raw:?})"), expected, actual);
    }
}

// ---------------------------------------------------------------------------
// Test generators.
// ---------------------------------------------------------------------------

#[test]
fn vibration_pulse_generator_defaults() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_printer_g = FakePrinter()").expect("printer");
    py.exec("py_cfg_g = FakeConfig(py_printer_g, {})").expect("cfg");
    let expected = py.eval(
        "(lambda g: [g.min_freq, g.max_freq, g.max_freq_z, g.accel_per_hz, \
         g.accel_per_hz_z, g.hz_per_sec])(rt.VibrationPulseTestGenerator(py_cfg_g))",
    );
    let env = Env::new();
    let gen = VibrationPulseTestGenerator::new(&env.config(&[], &[], &[])).expect("defaults");
    let actual = floats(&[
        gen.min_freq,
        gen.max_freq,
        gen.max_freq_z,
        gen.accel_per_hz,
        gen.accel_per_hz_z,
        gen.hz_per_sec,
    ]);
    check("VibrationPulseTestGenerator defaults", expected, actual);
}

#[test]
fn vibration_pulse_generator_overrides() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_printer_g2 = FakePrinter()").expect("printer");
    py.exec(
        "py_cfg_g2 = FakeConfig(py_printer_g2, {'min_freq': '10', 'max_freq': '40', \
         'max_freq_z': '20', 'accel_per_hz': '30', 'accel_per_hz_z': '7.5', \
         'hz_per_sec': '0.5'})",
    )
    .expect("cfg");
    let expected = py.eval(
        "(lambda g: [g.min_freq, g.max_freq, g.max_freq_z, g.accel_per_hz, \
         g.accel_per_hz_z, g.hz_per_sec])(rt.VibrationPulseTestGenerator(py_cfg_g2))",
    );
    let env = Env::new();
    let gen = VibrationPulseTestGenerator::new(&env.config(
        &[
            ("min_freq", 10.0),
            ("max_freq", 40.0),
            ("max_freq_z", 20.0),
            ("accel_per_hz", 30.0),
            ("accel_per_hz_z", 7.5),
            ("hz_per_sec", 0.5),
        ],
        &[],
        &[],
    ))
    .expect("overrides");
    let actual = floats(&[
        gen.min_freq,
        gen.max_freq,
        gen.max_freq_z,
        gen.accel_per_hz,
        gen.accel_per_hz_z,
        gen.hz_per_sec,
    ]);
    check("VibrationPulseTestGenerator overrides", expected, actual);
}

#[test]
fn vibration_pulse_generator_sequences() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_printer_s = FakePrinter()").expect("printer");
    py.exec("py_cfg_s = FakeConfig(py_printer_s, {'min_freq': '10', 'max_freq': '40'})")
        .expect("cfg");
    let cases: [(&[(&str, &str)], bool); 4] = [
        (&[], false),
        (&[("FREQ_START", "12"), ("FREQ_END", "14")], false),
        (&[("FREQ_START", "6"), ("FREQ_END", "7")], true),
        (
            &[
                ("FREQ_START", "20"),
                ("FREQ_END", "21"),
                ("ACCEL_PER_HZ", "12.5"),
                ("HZ_PER_SEC", "1.5"),
            ],
            false,
        ),
    ];
    for (params, is_z) in cases {
        let py_params = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "(lambda g: (g.prepare_test(FakeGCmd({{{py_params}}}), {}), \
             [list(g.gen_test()), g.get_max_freq()])[1])\
             (rt.VibrationPulseTestGenerator(py_cfg_s))",
            if is_z { "True" } else { "False" }
        ));
        let env = Env::new();
        let mut gen = VibrationPulseTestGenerator::new(&env.config(
            &[("min_freq", 10.0), ("max_freq", 40.0)],
            &[],
            &[],
        ))
        .expect("generator");
        let gcmd = FakeGCmd::arc(params);
        gen.prepare_test(&*gcmd, is_z).expect("prepare");
        let actual = Value::List(vec![seq_value(&gen.gen_test()), Value::float(gen.get_max_freq())]);
        check(&format!("gen_test {params:?} is_z={is_z}"), expected, actual);
    }
}

#[test]
fn vibration_pulse_generator_bounds() {
    let Some(mut py) = oracle() else { return };
    let cases: &[&[(&str, f64)]] = &[
        &[("min_freq", 0.0)],
        &[("max_freq", 301.0)],
        &[("max_freq", 2.0)],
        &[("max_freq_z", 301.0)],
        &[("accel_per_hz", 0.0)],
        &[("accel_per_hz_z", -1.0)],
        &[("hz_per_sec", 0.05)],
        &[("hz_per_sec", 2.5)],
        &[("min_freq", 5.0), ("max_freq_z", 4.0)],
    ];
    for floats in cases {
        let py_dict = floats
            .iter()
            .map(|(k, v)| format!("'{k}': '{v:?}'"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "rt.VibrationPulseTestGenerator(FakeConfig(FakePrinter(), {{{py_dict}}}))"
        ));
        let env = Env::new();
        let actual = VibrationPulseTestGenerator::new(&env.config(floats, &[], &[]))
            .map(|g| g.min_freq);
        check_err(&format!("generator bounds {floats:?}"), expected, actual);
    }
}

#[test]
fn sweeping_generator_defaults_and_sequences() {
    let Some(mut py) = oracle() else { return };
    py.exec("py_printer_w = FakePrinter()").expect("printer");
    py.exec("py_cfg_w = FakeConfig(py_printer_w, {'min_freq': '10', 'max_freq': '20'})")
        .expect("cfg");
    let expected = py.eval(
        "(lambda g: [g.vibration_generator.min_freq, g.sweeping_accel, \
         g.sweeping_accel_z, g.sweeping_period])\
         (rt.SweepingVibrationsTestGenerator(py_cfg_w))",
    );
    let env = Env::new();
    let gen = SweepingVibrationsTestGenerator::new(&env.config(
        &[("min_freq", 10.0), ("max_freq", 20.0)],
        &[],
        &[],
    ))
    .expect("sweeping defaults");
    let actual = floats(&[
        gen.vibration_generator.min_freq,
        gen.sweeping_accel,
        gen.sweeping_accel_z,
        gen.sweeping_period,
    ]);
    check("SweepingVibrationsTestGenerator defaults", expected, actual);

    let cases: [(&[(&str, &str)], bool); 3] = [
        (&[], false),
        (&[("FREQ_START", "12"), ("FREQ_END", "13"), ("SWEEPING_PERIOD", "0")], false),
        (
            &[
                ("FREQ_START", "12"),
                ("FREQ_END", "12.5"),
                ("SWEEPING_ACCEL", "120"),
                ("SWEEPING_PERIOD", "0.4"),
            ],
            true,
        ),
    ];
    for (params, is_z) in cases {
        let py_params = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "(lambda g: (g.prepare_test(FakeGCmd({{{py_params}}}), {}), \
             [list(g.gen_test()), g.get_max_freq()])[1])\
             (rt.SweepingVibrationsTestGenerator(py_cfg_w))",
            if is_z { "True" } else { "False" }
        ));
        let env = Env::new();
        let mut gen = SweepingVibrationsTestGenerator::new(&env.config(
            &[("min_freq", 10.0), ("max_freq", 20.0)],
            &[],
            &[],
        ))
        .expect("sweeping generator");
        let gcmd = FakeGCmd::arc(params);
        gen.prepare_test(&*gcmd, is_z).expect("prepare");
        let actual =
            Value::List(vec![seq_value(&gen.gen_test()), Value::float(gen.get_max_freq())]);
        check(&format!("sweeping gen_test {params:?} is_z={is_z}"), expected, actual);
    }
}

#[test]
fn sweeping_generator_bounds() {
    let Some(mut py) = oracle() else { return };
    let cases: &[&[(&str, f64)]] = &[
        &[("sweeping_accel", 0.0)],
        &[("sweeping_accel_z", -1.0)],
        &[("sweeping_period", -0.5)],
    ];
    for floats in cases {
        let py_dict = floats
            .iter()
            .map(|(k, v)| format!("'{k}': '{v:?}'"))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "rt.SweepingVibrationsTestGenerator(FakeConfig(FakePrinter(), {{{py_dict}}}))"
        ));
        let env = Env::new();
        let actual = SweepingVibrationsTestGenerator::new(&env.config(floats, &[], &[]))
            .map(|g| g.sweeping_period);
        check_err(&format!("sweeping bounds {floats:?}"), expected, actual);
    }
}

// ---------------------------------------------------------------------------
// `lookup_z_limits`.
// ---------------------------------------------------------------------------

fn z_limit_settings(z_velocity: Option<f64>, z_accel: Option<f64>) -> IndexMap<PyValue, PyValue> {
    let mut printer_section = IndexMap::new();
    if let Some(v) = z_velocity {
        printer_section.insert(
            PyValue::Str("max_z_velocity".to_string()),
            PyValue::Float(v),
        );
    }
    if let Some(v) = z_accel {
        printer_section.insert(PyValue::Str("max_z_accel".to_string()), PyValue::Float(v));
    }
    printer_section.insert(
        PyValue::Str("max_velocity".to_string()),
        PyValue::Float(120.0),
    );
    printer_section.insert(PyValue::Str("max_accel".to_string()), PyValue::Float(3000.0));
    let mut settings = IndexMap::new();
    settings.insert(PyValue::Str("printer".to_string()), PyValue::Dict(printer_section));
    settings
}

#[test]
fn lookup_z_limits_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_cf = FakeConfigFile({'printer': {'max_velocity': 120.0, 'max_accel': 3000.0}})\n\
         py_cf2 = FakeConfigFile({'printer': {'max_z_velocity': 10.0, \
         'max_z_accel': 100.0, 'max_velocity': 120.0, 'max_accel': 3000.0}})\n\
         py_cf3 = FakeConfigFile({})",
    )
    .expect("settings");

    let cases: [(IndexMap<PyValue, PyValue>, &str); 3] = [
        (z_limit_settings(None, None), "py_cf"),
        (z_limit_settings(Some(10.0), Some(100.0)), "py_cf2"),
        (IndexMap::new(), "py_cf3"),
    ];
    for (settings, expr) in cases {
        let expected = py.eval(&format!("rt.lookup_z_limits({expr})"));
        let cf = FakeConfigFile::new(settings);
        let actual = lookup_z_limits(&*cf).map(|(v, a)| {
            Value::Tuple(vec![
                v.map(Value::float).unwrap_or(Value::Null),
                a.map(Value::float).unwrap_or(Value::Null),
            ])
        });
        match expected {
            Ok(value) => check(
                &format!("lookup_z_limits {expr}"),
                Ok(value),
                actual.expect("limits resolved"),
            ),
            Err(e) => {
                let err = actual.expect_err("must fail");
                assert_eq!(err.class_name(), e.class_name.as_str(), "{expr}: class");
                assert_eq!(err.message(), e.message.as_str(), "{expr}: message");
            }
        }
    }
}

#[test]
fn lookup_z_limits_missing_settings_key() {
    let Some(mut py) = oracle() else { return };
    py.exec("class NoSettings:\n    def get_status(self, eventtime):\n        return {}\npy_no = NoSettings()")
        .expect("status");
    let expected = py.eval("rt.lookup_z_limits(py_no)");

    struct EmptyStatus;
    impl PrinterConfigApi for EmptyStatus {
        fn get_printer(&self) -> Printer {
            panic!("unused")
        }
        fn read_config(&self, _filename: &str) -> KResult<Config> {
            panic!("unused")
        }
        fn read_main_config(&self) -> KResult<Config> {
            panic!("unused")
        }
        fn log_config(&self, _config: &Config) {}
        fn check_unused_options(&self, _config: &Config) -> KResult<()> {
            Ok(())
        }
        fn runtime_warning(&self, _msg: &str) {}
        fn deprecate(
            &self,
            _section: &str,
            _option: &str,
            _value: Option<PyValue>,
            _msg: Option<String>,
        ) {
        }
        fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
        fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {}
        fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
            PyValue::Dict(IndexMap::new())
        }
        fn set(&self, _section: &str, _option: &str, _value: &str) {}
        fn remove_section(&self, _section: &str) {}
    }

    let actual = lookup_z_limits(&EmptyStatus).map(|_| 0.0);
    check_err("lookup_z_limits without settings", expected, actual);
}

// ---------------------------------------------------------------------------
// `ResonanceTester.__init__`.
// ---------------------------------------------------------------------------

fn chip_names_value(names: &[(String, String)]) -> Value {
    Value::List(
        names
            .iter()
            .map(|(axes, chip)| Value::Tuple(vec![Value::str(axes), Value::str(chip)]))
            .collect(),
    )
}

/// `(config strings, probe points)` for one `ResonanceTester.__init__` case.
type InitCase = (&'static [(&'static str, &'static str)], &'static [(f64, f64, f64)]);

#[test]
fn tester_init_fields_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases: [InitCase; 5] = [
        (&[("accel_chip", "adxl345")], &[(10.0, 20.0, 5.0)]),
        (
            &[("accel_chip", "adxl345"), ("accel_chip_z", "adxl345")],
            &[(10.0, 20.0, 5.0)],
        ),
        (
            &[
                ("accel_chip_x", "adxl345 hotend"),
                ("accel_chip_y", "adxl345 hotend"),
                ("accel_chip_z", "adxl345 bed"),
            ],
            &[(1.0, 2.0, 3.0), (-4.5, 0.5, 2.25)],
        ),
        (
            &[
                ("accel_chip", "adxl345"),
                ("accel_chip_z", "adxl345 bed"),
                ("move_speed", "7.5"),
                ("max_smoothing", "0.2"),
            ],
            &[(0.0, 0.0, 5.0)],
        ),
        (
            &[
                ("accel_chip", " adxl345 "),
                ("accel_chip_z", ""),
                ("max_smoothing", "0.05"),
            ],
            &[(-1.0, -2.0, -3.0)],
        ),
    ];
    for (strs, points) in cases {
        let py_dict = strs
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        let point_rows = points
            .iter()
            .map(|(x, y, z)| format!("({x:?}, {y:?}, {z:?})"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "py_printer_i = FakePrinter()\n\
             py_t_i = make_tester(py_printer_i, {{{py_dict}}}, \
             {{'probe_points': ({point_rows},)}})"
        ))
        .expect("python tester");
        let expected = py.eval(
            "[py_t_i.accel_chip_names, py_t_i.move_speed, py_t_i.max_smoothing, \
             [list(p) for p in py_t_i.probe_points]]",
        );
        let env = Env::new();
        let tester = env.tester(&[], strs, points);
        let actual = Value::List(vec![
            chip_names_value(tester.accel_chip_names()),
            Value::float(tester.move_speed()),
            tester.max_smoothing().map(Value::float).unwrap_or(Value::Null),
            Value::List(
                tester
                    .probe_points()
                    .iter()
                    .map(|row| floats(row))
                    .collect(),
            ),
        ]);
        check(&format!("ResonanceTester fields {strs:?}"), expected, actual);
    }
}

#[test]
fn tester_init_requires_accel_chip_y() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_y = FakePrinter()\n\
         py_err_y = None\n\
         try:\n    make_tester(py_printer_y, {'accel_chip_x': ' '}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         except Exception as e:\n    py_err_y = e",
    )
    .expect("python error");
    let expected = py.eval("[type(py_err_y).__name__, str(py_err_y)]");
    let env = Env::new();
    let config = env.config(&[], &[("accel_chip_x", " ")], &[(1.0, 2.0, 3.0)]);
    let actual = ResonanceTester::new(&config).map(|t| t.move_speed());
    check_typed_err("missing accel_chip_y", expected, actual);
}

#[test]
fn tester_connect_and_parse_chips() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_c = FakePrinter()\n\
         py_chip_c = PyChip(py_printer_c, 'adxl345')\n\
         py_notchip = PyNotAChip()\n\
         py_printer_c.objects['adxl345'] = py_chip_c\n\
         py_printer_c.objects['notachip'] = py_notchip\n\
         py_t_c = make_tester(py_printer_c, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_c.connect()",
    )
    .expect("python connect");
    let expected = py.eval("[len(py_t_c.accel_chips), [c[0] for c in py_t_c.accel_chips]]");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    env.add("notachip", Arc::new(FakeNotAChip) as ObjectRef);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let chips = tester.accel_chips();
    let actual = Value::List(vec![
        Value::int(chips.len() as i64),
        Value::List(chips.iter().map(|(axes, _)| Value::str(axes)).collect()),
    ]);
    check("connect", expected, actual);

    // `_parse_chips` returns the objects it resolved.
    let expected_ok = py.eval("len(py_t_c._parse_chips('adxl345, adxl345 '))");
    let parsed = tester.parse_chips("adxl345, adxl345 ").expect("parsed");
    check("_parse_chips length", expected_ok, Value::int(parsed.len() as i64));

    py.exec("py_err_chip = None\ntry:\n    py_t_c._parse_chips('nope')\nexcept Exception as e:\n    py_err_chip = e")
        .expect("python error");
    let expected_err = py.eval("[type(py_err_chip).__name__, str(py_err_chip)]");
    let actual_err = tester.parse_chips("nope").map(|_| 0);
    check_typed_err("_parse_chips unknown", expected_err, actual_err);

    py.exec("py_err_chip2 = None\ntry:\n    py_t_c._parse_chips('notachip')\nexcept Exception as e:\n    py_err_chip2 = e")
        .expect("python error");
    let expected_err2 = py.eval("[type(py_err_chip2).__name__, str(py_err_chip2)]");
    let actual_err2 = tester.parse_chips("notachip").map(|_| 0);
    check_typed_err("_parse_chips not a chip", expected_err2, actual_err2);
}

fn check_typed_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Result<impl std::fmt::Debug, KlippyError>,
) {
    let value = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but the call must fail"),
        Err(e) => e,
    };
    match value {
        Value::List(items) => {
            assert_eq!(items.len(), 2, "{label}: expected [class, message]");
            let (class, message) = match (&items[0], &items[1]) {
                (Value::Str(c), Value::Str(m)) => (c.clone(), m.clone()),
                _ => panic!("{label}: expected string pair"),
            };
            assert_eq!(rust_err.class_name(), class, "{label}: class");
            assert_eq!(rust_err.message(), message, "{label}: message");
        }
        other => panic!("{label}: expected [class, message], got {other:?}"),
    }
}

#[test]
fn tester_connect_rejects_non_accelerometer() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_n = FakePrinter()\n\
         py_printer_n.objects['notachip'] = PyNotAChip()\n\
         py_t_n = make_tester(py_printer_n, {'accel_chip': 'notachip'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_err_n = None\ntry:\n    py_t_n.connect()\nexcept Exception as e:\n    py_err_n = e",
    )
    .expect("python connect");
    let expected = py.eval("[type(py_err_n).__name__, str(py_err_n)]");
    let env = Env::new();
    env.add("notachip", Arc::new(FakeNotAChip) as ObjectRef);
    let tester = env.tester(&[], &[("accel_chip", "notachip")], &[(1.0, 2.0, 3.0)]);
    let actual = tester.connect();
    check_typed_err("connect not a chip", expected, actual);
}

#[test]
fn filename_and_name_suffix() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_f = FakePrinter()\n\
         py_t_f = make_tester(py_printer_f, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})",
    )
    .expect("python tester");
    let env = Env::new();
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);

    let expected = py.eval(
        "[py_t_f.get_filename('resonances', 'abc', rt.TestAxis(axis='x'), \
         [1.0, 2.0, 3.0], 'adxl 345'), py_t_f.get_filename('raw_data', 'abc'), \
         py_t_f.get_filename('calibration_data', 'n1', rt.TestAxis(vib_dir=(1, 2, 3)), \
         [-1.5, 0.25, 10.0], None)]",
    );
    let actual = Value::List(vec![
        Value::str(&tester.get_filename(
            "resonances",
            "abc",
            Some(&TestAxis::new_axis("x").expect("x")),
            Some(&[1.0, 2.0, 3.0]),
            Some("adxl 345"),
        )),
        Value::str(&tester.get_filename("raw_data", "abc", None, None, None)),
        Value::str(&tester.get_filename(
            "calibration_data",
            "n1",
            Some(&TestAxis::new_vib_dir(&[1.0, 2.0, 3.0]).expect("vib_dir")),
            Some(&[-1.5, 0.25, 10.0]),
            None,
        )),
    ]);
    check("get_filename", expected, actual);

    let names = ["valid", "valid-name_1", "bad name!", "", "a/b", "ünïcode", "-_"];
    let expected = py.eval(&format!(
        "[py_t_f.is_valid_name_suffix(n) for n in ({})]",
        names.iter().map(|n| format!("{n:?}")).collect::<Vec<_>>().join(", ")
    ));
    let actual = Value::List(
        names
            .iter()
            .map(|n| Value::Bool(tester.is_valid_name_suffix(n)))
            .collect(),
    );
    check("is_valid_name_suffix", expected, actual);
}

#[test]
fn command_registration_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_r = FakePrinter()\n\
         py_t_r = make_tester(py_printer_r, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})",
    )
    .expect("python tester");
    let expected = py.eval("sorted(py_printer_r.gcode.registered.items())");
    let env = Env::new();
    let _tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    let mut registered: Vec<(String, (bool, Option<String>))> = env
        .gcode
        .inner
        .lock()
        .registered
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();
    registered.sort_by(|a, b| a.0.cmp(&b.0));
    let actual = Value::List(
        registered
            .iter()
            .map(|(cmd, (has_func, desc))| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::Tuple(vec![
                        Value::Bool(*has_func),
                        desc.as_ref().map(|d| Value::str(d)).unwrap_or(Value::Null),
                    ]),
                ])
            })
            .collect(),
    );
    check("registered commands", expected, actual);
}

// ---------------------------------------------------------------------------
// `TEST_RESONANCES`.
// ---------------------------------------------------------------------------

#[test]
fn test_resonances_argument_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_e = FakePrinter()\n\
         py_printer_e.objects['adxl345'] = PyChip(py_printer_e, 'adxl345')\n\
         py_printer_e.objects['notachip'] = PyNotAChip()\n\
         py_t_e = make_tester(py_printer_e, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_e.connect()",
    )
    .expect("python tester");
    let cases: &[&[(&str, &str)]] = &[
        &[],
        &[("AXIS", "q")],
        &[("AXIS", "1")],
        &[("AXIS", "x,1")],
        &[("AXIS", "x"), ("POINT", "1,2")],
        &[("AXIS", "x"), ("POINT", "a,b,c")],
        &[("AXIS", "x"), ("OUTPUT", "bad")],
        &[("AXIS", "x"), ("OUTPUT", "resonances,bad")],
        &[("AXIS", "x"), ("NAME", "bad name!")],
        &[("AXIS", "x"), ("NAME", "")],
        &[("AXIS", "x"), ("CHIPS", "nope")],
        &[("AXIS", "x"), ("CHIPS", "notachip")],
        &[("AXIS", "z")],
    ];
    for &params in cases {
        let py_params = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "py_err_r = None\ntry:\n    py_t_e.cmd_TEST_RESONANCES(FakeGCmd({{{py_params}}}))\n\
             except Exception as e:\n    py_err_r = e"
        ))
        .expect("python call");
        let expected = py.eval("[type(py_err_r).__name__, str(py_err_r)]");
        let env = Env::new();
        env.add_chip("adxl345", &[]);
        env.add("notachip", Arc::new(FakeNotAChip) as ObjectRef);
        let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
        tester.connect().expect("connect");
        let gcmd = FakeGCmd::arc(params);
        let actual = tester.cmd_test_resonances(&*gcmd);
        check_typed_err(&format!("TEST_RESONANCES {params:?}"), expected, actual);
    }
}

#[test]
fn test_resonances_raw_data_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_d = FakePrinter()\n\
         py_printer_d.objects['adxl345'] = PyChip(py_printer_d, 'adxl345')\n\
         py_t_d = make_tester(py_printer_d, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_d.connect()\n\
         py_gcmd_d = FakeGCmd({'AXIS': 'x', 'OUTPUT': 'raw_data', 'NAME': 'abc', \
         'FREQ_START': '20', 'FREQ_END': '21'})\n\
         py_t_d.cmd_TEST_RESONANCES(py_gcmd_d)",
    )
    .expect("python command");
    let expected_scripts = py.eval("py_printer_d.gcode.scripts");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_d.infos"));
    let expected_moves = py.eval("[repr(m) for m in py_printer_d.toolhead.moves]");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("OUTPUT", "raw_data"),
        ("NAME", "abc"),
        ("FREQ_START", "20"),
        ("FREQ_END", "21"),
    ]);
    tester.cmd_test_resonances(&*gcmd).expect("raw data");

    check("raw scripts", expected_scripts, strings(&env.scripts()));
    check(
        "raw infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
    check("raw moves", expected_moves, strings(&env.moves()));
}

#[test]
fn test_resonances_probe_point_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_p = FakePrinter()\n\
         py_printer_p.objects['adxl345'] = PyChip(py_printer_p, 'adxl345')\n\
         py_t_p = make_tester(py_printer_p, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_p.connect()\n\
         py_gcmd_p = FakeGCmd({'AXIS': 'x', 'OUTPUT': 'raw_data', 'NAME': 'abc', \
         'POINT': '4,5,6', 'FREQ_START': '20', 'FREQ_END': '20.5'})\n\
         py_t_p.cmd_TEST_RESONANCES(py_gcmd_p)",
    )
    .expect("python command");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_p.infos"));
    let expected_moves = py.eval("[repr(m) for m in py_printer_p.toolhead.moves]");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("OUTPUT", "raw_data"),
        ("NAME", "abc"),
        ("POINT", "4,5,6"),
        ("FREQ_START", "20"),
        ("FREQ_END", "20.5"),
    ]);
    tester.cmd_test_resonances(&*gcmd).expect("probe point");

    check(
        "point infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
    check("point moves", expected_moves, strings(&env.moves()));
}

#[test]
fn test_resonances_empty_point_string_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_q = FakePrinter()\n\
         py_printer_q.objects['adxl345'] = PyChip(py_printer_q, 'adxl345')\n\
         py_t_q = make_tester(py_printer_q, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_q.connect()\n\
         py_gcmd_q = FakeGCmd({'AXIS': 'x', 'OUTPUT': 'raw_data', 'NAME': 'abc', \
         'POINT': '', 'FREQ_START': '20', 'FREQ_END': '20.5'})\n\
         py_t_q.cmd_TEST_RESONANCES(py_gcmd_q)",
    )
    .expect("python command");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_q.infos"));

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("OUTPUT", "raw_data"),
        ("NAME", "abc"),
        ("POINT", ""),
        ("FREQ_START", "20"),
        ("FREQ_END", "20.5"),
    ]);
    tester.cmd_test_resonances(&*gcmd).expect("empty point");

    check(
        "empty point infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
}

#[test]
fn test_resonances_resonances_output_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_z = FakePrinter()\n\
         py_printer_z.objects['adxl345'] = PyChip(py_printer_z, 'adxl345', ZERO_ROWS)\n\
         py_t_z = make_tester(py_printer_z, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_z.connect()\n\
         py_gcmd_z = FakeGCmd({'AXIS': 'x', 'NAME': 'abc', \
         'FREQ_START': '20', 'FREQ_END': '20.5'})\n\
         py_t_z.cmd_TEST_RESONANCES(py_gcmd_z)",
    )
    .expect("python command");
    let expected_scripts = py.eval("py_printer_z.gcode.scripts");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_z.infos"));

    let rows: Vec<(f64, f64, f64, f64)> =
        (0..1000).map(|i| (i as f64 * 0.001, 0.0, 0.0, 0.0)).collect();
    let env = Env::new();
    env.add_chip("adxl345", &rows);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("NAME", "abc"),
        ("FREQ_START", "20"),
        ("FREQ_END", "20.5"),
    ]);
    tester.cmd_test_resonances(&*gcmd).expect("resonances output");

    check("csv scripts", expected_scripts, strings(&env.scripts()));
    check(
        "csv infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
}

#[test]
fn test_resonances_input_shaping_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_h = FakePrinter()\n\
         py_shaper_h = PyInputShaper()\n\
         py_printer_h.objects['adxl345'] = PyChip(py_printer_h, 'adxl345')\n\
         py_printer_h.objects['input_shaper'] = py_shaper_h\n\
         py_t_h = make_tester(py_printer_h, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_h.connect()\n\
         py_gcmd_h = FakeGCmd({'AXIS': 'x', 'OUTPUT': 'raw_data', 'NAME': 'abc', \
         'FREQ_START': '20', 'FREQ_END': '20.5'})\n\
         py_t_h.cmd_TEST_RESONANCES(py_gcmd_h)",
    )
    .expect("python command");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_h.infos"));
    let expected_calls = py.eval("py_shaper_h.calls");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let shaper = FakeInputShaper::new();
    env.add("input_shaper", Arc::clone(&shaper) as ObjectRef);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("OUTPUT", "raw_data"),
        ("NAME", "abc"),
        ("FREQ_START", "20"),
        ("FREQ_END", "20.5"),
    ]);
    tester.cmd_test_resonances(&*gcmd).expect("input shaping");

    check(
        "input shaping infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
    check(
        "input shaping calls",
        expected_calls,
        strings(&shaper.calls.lock().clone()),
    );
}

#[test]
fn test_resonances_z_limit_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_l = FakePrinter()\n\
         py_printer_l.objects['adxl345'] = PyChip(py_printer_l, 'adxl345')\n\
         py_printer_l.objects['configfile'] = FakeConfigFile( \
         {'printer': {'max_velocity': 1.0, 'max_accel': 10.0}})\n\
         py_t_l = make_tester(py_printer_l, {'accel_chip': 'adxl345', \
         'accel_chip_z': 'adxl345'}, {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_l.connect()\n\
         py_err_l = None\ntry:\n    py_t_l.cmd_TEST_RESONANCES(FakeGCmd( \
         {'AXIS': 'z', 'OUTPUT': 'raw_data', 'NAME': 'abc'}))\n\
         except Exception as e:\n    py_err_l = e",
    )
    .expect("python command");
    let expected = py.eval("[type(py_err_l).__name__, str(py_err_l)]");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let mut settings = IndexMap::new();
    let mut printer_section = IndexMap::new();
    printer_section.insert(
        PyValue::Str("max_velocity".to_string()),
        PyValue::Float(1.0),
    );
    printer_section.insert(PyValue::Str("max_accel".to_string()), PyValue::Float(10.0));
    settings.insert(
        PyValue::Str("printer".to_string()),
        PyValue::Dict(printer_section),
    );
    let configfile = FakeConfigFile::new(settings);
    env.printer
        .add_object("configfile", Arc::clone(&configfile) as ObjectRef)
        .expect("replace configfile");
    let tester = env.tester(
        &[],
        &[("accel_chip", "adxl345"), ("accel_chip_z", "adxl345")],
        &[(1.0, 2.0, 3.0)],
    );
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[("AXIS", "z"), ("OUTPUT", "raw_data"), ("NAME", "abc")]);
    let actual = tester.cmd_test_resonances(&*gcmd);
    check_typed_err("z limits", expected, actual);
}

// ---------------------------------------------------------------------------
// `MEASURE_AXES_NOISE`.
// ---------------------------------------------------------------------------

#[test]
fn measure_axes_noise_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_m = FakePrinter()\n\
         py_printer_m.objects['adxl345'] = PyChip(py_printer_m, 'adxl345', ZERO_ROWS)\n\
         py_t_m = make_tester(py_printer_m, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_m.connect()\n\
         py_gcmd_m = FakeGCmd({'MEAS_TIME': '1.5'})\n\
         py_t_m.cmd_MEASURE_AXES_NOISE(py_gcmd_m)",
    )
    .expect("python command");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_m.infos"));
    let expected_moves = py.eval("[repr(m) for m in py_printer_m.toolhead.moves]");

    let rows: Vec<(f64, f64, f64, f64)> =
        (0..1000).map(|i| (i as f64 * 0.001, 0.0, 0.0, 0.0)).collect();
    let env = Env::new();
    env.add_chip("adxl345", &rows);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[("MEAS_TIME", "1.5")]);
    tester.cmd_measure_axes_noise(&*gcmd).expect("noise");

    check(
        "noise infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
    check("noise moves", expected_moves, strings(&env.moves()));
}

#[test]
fn measure_axes_noise_error_paths_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_n2 = FakePrinter()\n\
         py_printer_n2.objects['adxl345'] = PyChip(py_printer_n2, 'adxl345')\n\
         py_t_n2 = make_tester(py_printer_n2, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_n2.connect()\n\
         py_err_n2 = None\ntry:\n    py_t_n2.cmd_MEASURE_AXES_NOISE(FakeGCmd({}))\n\
         except Exception as e:\n    py_err_n2 = e\n\
         py_err_n3 = None\ntry:\n    py_t_n2.cmd_MEASURE_AXES_NOISE(FakeGCmd({'MEAS_TIME': '0'}))\n\
         except Exception as e:\n    py_err_n3 = e",
    )
    .expect("python command");
    let expected_no_data = py.eval("[type(py_err_n2).__name__, str(py_err_n2)]");
    let expected_bad_time = py.eval("[type(py_err_n3).__name__, str(py_err_n3)]");

    let env = Env::new();
    env.add_chip("adxl345", &[]);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[]);
    let actual = tester.cmd_measure_axes_noise(&*gcmd);
    check_typed_err("noise no data", expected_no_data, actual);

    let gcmd = FakeGCmd::arc(&[("MEAS_TIME", "0")]);
    let actual = tester.cmd_measure_axes_noise(&*gcmd);
    check_typed_err("noise bad MEAS_TIME", expected_bad_time, actual);
}

// ---------------------------------------------------------------------------
// `SHAPER_CALIBRATE`.
// ---------------------------------------------------------------------------

#[test]
fn shaper_calibrate_argument_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_s2 = FakePrinter()\n\
         py_printer_s2.objects['adxl345'] = PyChip(py_printer_s2, 'adxl345')\n\
         py_printer_s2.objects['notachip'] = PyNotAChip()\n\
         py_t_s2 = make_tester(py_printer_s2, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_s2.connect()",
    )
    .expect("python tester");
    let cases: &[&[(&str, &str)]] = &[
        &[("AXIS", "q")],
        &[("AXIS", "xy")],
        &[("AXIS", "z")],
        &[("NAME", "bad name!")],
        &[("MAX_SMOOTHING", "0.01")],
        &[("CHIPS", "nope")],
        &[("CHIPS", "notachip")],
    ];
    for &params in cases {
        let py_params = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "py_err_s = None\ntry:\n    py_t_s2.cmd_SHAPER_CALIBRATE(FakeGCmd({{{py_params}}}))\n\
             except Exception as e:\n    py_err_s = e"
        ))
        .expect("python call");
        let expected = py.eval("[type(py_err_s).__name__, str(py_err_s)]");
        let env = Env::new();
        env.add_chip("adxl345", &[]);
        env.add("notachip", Arc::new(FakeNotAChip) as ObjectRef);
        let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
        tester.connect().expect("connect");
        let gcmd = FakeGCmd::arc(params);
        let actual = tester.cmd_shaper_calibrate(&*gcmd);
        check_typed_err(&format!("SHAPER_CALIBRATE {params:?}"), expected, actual);
    }
}

#[test]
fn shaper_calibrate_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "py_printer_o = FakePrinter()\n\
         py_shaper_o = PyInputShaper()\n\
         py_cf_o = FakeConfigFile({'printer': {'max_velocity': 100.0, \
         'max_accel': 2000.0}})\n\
         py_printer_o.objects['adxl345'] = PyChip(py_printer_o, 'adxl345', ZERO_ROWS)\n\
         py_printer_o.objects['input_shaper'] = py_shaper_o\n\
         py_printer_o.objects['configfile'] = py_cf_o\n\
         py_t_o = make_tester(py_printer_o, {'accel_chip': 'adxl345'}, \
         {'probe_points': ((1.0, 2.0, 3.0),)})\n\
         py_t_o.connect()\n\
         py_gcmd_o = FakeGCmd({'AXIS': 'x', 'NAME': 'abc', \
         'FREQ_START': '20', 'FREQ_END': '22'})\n\
         py_t_o.cmd_SHAPER_CALIBRATE(py_gcmd_o)",
    )
    .expect("python command");
    let expected_infos = py.eval(&py_list_of_lists("py_gcmd_o.infos"));
    let expected_sets = py.eval(&py_list_of_lists("py_cf_o.sets"));
    let expected_calls = py.eval("py_shaper_o.calls");

    let rows: Vec<(f64, f64, f64, f64)> =
        (0..1000).map(|i| (i as f64 * 0.001, 0.0, 0.0, 0.0)).collect();
    let env = Env::new();
    env.add_chip("adxl345", &rows);
    let shaper = FakeInputShaper::new();
    env.add("input_shaper", Arc::clone(&shaper) as ObjectRef);
    let tester = env.tester(&[], &[("accel_chip", "adxl345")], &[(1.0, 2.0, 3.0)]);
    tester.connect().expect("connect");
    let gcmd = FakeGCmd::arc(&[
        ("AXIS", "x"),
        ("NAME", "abc"),
        ("FREQ_START", "20"),
        ("FREQ_END", "22"),
    ]);
    tester.cmd_shaper_calibrate(&*gcmd).expect("shaper calibrate");

    check(
        "shaper infos",
        expected_infos,
        infos_value(&gcmd.infos.lock().clone()),
    );
    check("shaper configfile sets", expected_sets, sets_value(&env.sets()));
    check(
        "shaper input_shaper calls",
        expected_calls,
        strings(&shaper.calls.lock().clone()),
    );
}

// ---------------------------------------------------------------------------
// Module entry point.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_is_load_config_only() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}
