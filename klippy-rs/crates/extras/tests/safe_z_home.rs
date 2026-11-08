//! Parity tests for `extras::safe_z_home` (port of `klippy/extras/safe_z_home.py`).
//!
//! Expected values come from the live reference through `pyoracle` (CPython
//! 3.8.2 + the frozen `klippy/`): a fresh `SafeZHoming` is built on both
//! sides with minimal Python fakes for the printer, config wrapper, toolhead,
//! kinematics and G-code dispatch, then fed identical config values and
//! `G28` commands and the resulting fields, side-effect logs and errors are
//! compared exactly (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use std::sync::{Arc, Mutex};

use extras::safe_z_home::{lookup_z_endstop_config, SafeZHoming};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{ExtraAxis, Kinematics, PrinterExtruder, ToolHeadApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::value::py_float_repr;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.safe_z_home as safe_mod

sentinel = object()

class CommandError(Exception):
    pass

log = []

class FakeReactor:
    def monotonic(self):
        return 123.0

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=sentinel):
        log.append("load_object:" + section)
        if section in self.objects:
            return self.objects[section]
        if section not in ("homing",):
            raise self.config_error("Unable to load module '%s'" % (section,))
        obj = {"homing": True}
        self.objects[section] = obj
        return obj

class FakeKinematics:
    def __init__(self, state):
        self.state = state
    def get_status(self, eventtime):
        log.append("get_status")
        return {'homed_axes': self.state['homed']}
    def clear_homing_state(self, axes):
        log.append("clear_homing_state:" + axes)
        for c in axes:
            self.state['homed'] = self.state['homed'].replace(c, '')

class FakeToolHead:
    def __init__(self, state):
        self.state = state
        self.kin = FakeKinematics(state)
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        log.append("get_position")
        return list(self.state['pos'])
    def set_position(self, pos, homing_axes=""):
        pos = [float(v) for v in pos]
        log.append("set_position:" + repr(list(pos)) + ":" + homing_axes)
        self.state['pos'] = list(pos)
    def manual_move(self, coord, speed):
        log.append("manual_move:" + repr(list(coord)) + ":" + repr(speed))
        for i in range(len(coord)):
            if coord[i] is not None:
                if i >= len(self.state['pos']):
                    raise IndexError("list assignment index out of range")
                self.state['pos'][i] = coord[i]

class FakeGcmd:
    error = CommandError
    def __init__(self, gcode, command, commandline, params):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
    def get(self, name, default=sentinel, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return dict(self._params)

class FakeGCode:
    error = CommandError
    def __init__(self, printer):
        self.printer = printer
        self.handlers = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            log.append("register_command:" + cmd + ":None")
            old = self.handlers.get(cmd)
            if cmd in self.handlers:
                del self.handlers[cmd]
            return old
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        log.append("register_command:" + cmd + ":func")
        return None
    def create_gcode_command(self, command, commandline, params):
        log.append("create_gcode_command:" + command + ":" + commandline + ":" + repr(list(params.items())))
        return FakeGcmd(self, command, commandline, dict(params))

def _parse_error(option, section):
    return configparser.Error("Unable to parse option '%s' in section '%s'" % (option, section))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options, all_sections):
        self._printer = printer
        self._name = name
        self._options = dict(options)
        self._all = all_sections
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def has_section(self, section):
        return section in self._all
    def getsection(self, section):
        return FakeConfig(self._printer, section, dict(self._all.get(section, {})), self._all)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in self._all if s.startswith(prefix)]
    def get(self, option, default=sentinel, note_valid=True):
        if option not in self._options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        return self._options[option]
    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self._options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        try:
            v = float(self._options[option])
        except Exception:
            raise _parse_error(option, self._name)
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def getfloatlist(self, option, default=sentinel, sep=',', count=None, note_valid=True):
        if option not in self._options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        raw = self._options[option]
        if len(raw.strip()) == 0:
            res = []
        else:
            res = []
            for p in raw.split(sep):
                p = p.strip()
                try:
                    res.append(float(p))
                except Exception:
                    raise _parse_error(option, self._name)
        if count is not None and len(res) != count:
            raise configparser.Error("Option '%s' in section '%s' must have %d elements" % (option, self._name, count))
        return tuple(res)
    def getboolean(self, option, default=sentinel, note_valid=True):
        if option not in self._options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        raw = self._options[option].strip().lower()
        if raw in ('1', 'yes', 'true', 'on'):
            return True
        if raw in ('0', 'no', 'false', 'off'):
            return False
        raise _parse_error(option, self._name)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fmt_params(params: &IndexMap<String, String>) -> String {
    if params.is_empty() {
        return "[]".to_string();
    }
    let parts: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("('{k}', '{v}')"))
        .collect();
    format!("[{}]", parts.join(", "))
}

fn fmt_coord(coord: &[Option<f64>]) -> String {
    let parts: Vec<String> = coord
        .iter()
        .map(|c| match c {
            Some(v) => py_float_repr(*v),
            None => "None".to_string(),
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn fmt_pos(pos: &[f64]) -> String {
    let parts: Vec<String> = pos.iter().map(|v| py_float_repr(*v)).collect();
    format!("[{}]", parts.join(", "))
}

#[derive(Clone, Default)]
struct SharedState {
    pos: Vec<f64>,
    homed: String,
}

type Log = Arc<Mutex<Vec<String>>>;
type State = Arc<Mutex<SharedState>>;

struct FakeKinematics {
    state: State,
    log: Log,
}
klippy_api::printer_object!(FakeKinematics; dyn klippy_api::toolhead::KinematicsApi);

#[allow(dead_code)]
fn _unused_kinematics_alias() -> Option<klippy_api::toolhead::Kinematics> {
    None
}

impl klippy_api::toolhead::KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        panic!("calc_position is not used by safe_z_home")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("kinematics set_position is not used by safe_z_home")
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        panic!("check_move is not used by safe_z_home")
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("home is not used by safe_z_home")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        self.log.lock().unwrap().push("get_status".to_string());
        let homed = self.state.lock().unwrap().homed.clone();
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str(homed));
        PyValue::Dict(d)
    }
    fn clear_homing_state(&self, clear_axes: &str) {
        self.log
            .lock()
            .unwrap()
            .push(format!("clear_homing_state:{clear_axes}"));
        let mut s = self.state.lock().unwrap();
        for c in clear_axes.chars() {
            s.homed = s.homed.replace(c, "");
        }
    }
}

struct FakeToolHead {
    state: State,
    log: Log,
}
klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        self.log.lock().unwrap().push("get_position".to_string());
        self.state.lock().unwrap().pos.clone()
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.log
            .lock()
            .unwrap()
            .push(format!("set_position:{}:{homing_axes}", fmt_pos(&newpos)));
        self.state.lock().unwrap().pos = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics {
            state: self.state.clone(),
            log: self.log.clone(),
        }) as Kinematics
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by safe_z_home")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by safe_z_home")
    }
    fn dwell(&self, _delay: f64) {
        panic!("dwell is not used by safe_z_home")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<f64>,
    ) -> KResult<()> {
        panic!("drip_move is not used by safe_z_home")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by safe_z_home")
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.log.lock().unwrap().push(format!(
            "manual_move:{}:{}",
            fmt_coord(&coord),
            py_float_repr(speed)
        ));
        let mut s = self.state.lock().unwrap();
        for (i, c) in coord.iter().enumerate() {
            if let Some(v) = c {
                if i >= s.pos.len() {
                    return Err(klippy_api::KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list assignment index out of range".to_string(),
                    });
                }
                s.pos[i] = *v;
            }
        }
        Ok(())
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by safe_z_home")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("toolhead get_status is not used by safe_z_home")
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by safe_z_home")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by safe_z_home")
    }
    fn set_max_velocities(
        &self,
        _a: Option<f64>,
        _b: Option<f64>,
        _c: Option<f64>,
        _d: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by safe_z_home")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by safe_z_home")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by safe_z_home")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by safe_z_home")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by safe_z_home")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by safe_z_home")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by safe_z_home")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by safe_z_home")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by safe_z_home")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by safe_z_home")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by safe_z_home")
    }
}

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}
impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
            Some(raw) => Ok(PyValue::Str(raw.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
            Some(raw) => match raw.trim().parse::<i64>() {
                Ok(v) => Ok(Some(v)),
                Err(_) => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_float(&self, name: &str, opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
            Some(raw) => match pycompat::text::float_from_string(raw) {
                Ok(v) => Ok(Some(v)),
                Err(_) => Err(klippy_api::KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))),
            },
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
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeGCode {
    log: Log,
    handlers: Mutex<IndexMap<String, GCodeCommandHandler>>,
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        if func.is_none() {
            self.log
                .lock()
                .unwrap()
                .push(format!("register_command:{cmd}:None"));
            let mut h = self.handlers.lock().unwrap();
            let old = h.swap_remove(cmd);
            return Ok(old);
        }
        let mut h = self.handlers.lock().unwrap();
        if h.contains_key(cmd) {
            return Err(klippy_api::KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        self.log
            .lock()
            .unwrap()
            .push(format!("register_command:{cmd}:func"));
        h.insert(cmd.to_string(), func.unwrap());
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
        panic!("register_mux_command is not used by safe_z_home")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used by safe_z_home")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by safe_z_home")
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {
        panic!("register_output_handler is not used by safe_z_home")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by safe_z_home")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by safe_z_home")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by safe_z_home")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by safe_z_home")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        self.log.lock().unwrap().push(format!(
            "create_gcode_command:{command}:{commandline}:{}",
            fmt_params(params)
        ));
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        }) as klippy_api::gcode::GCmd
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by safe_z_home")
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by safe_z_home")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by safe_z_home")
    }
}

struct FakePrinter {
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    log: Log,
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match name {
            "gcode" => Ok(Some(self.gcode.clone() as ObjectRef)),
            "toolhead" => Ok(Some(self.toolhead.clone() as ObjectRef)),
            _ => match default {
                LookupDefault::Raise => Err(klippy_api::KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.log
            .lock()
            .unwrap()
            .push(format!("load_object:{section}"));
        if section != "homing" {
            return Err(klippy_api::KlippyError::config(format!(
                "Unable to load module '{section}'"
            )));
        }
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

// --- FakeConfig ---

fn unable(option: &str, section: &str) -> klippy_api::KlippyError {
    klippy_api::KlippyError::config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}
fn must_specified(option: &str, section: &str) -> klippy_api::KlippyError {
    klippy_api::KlippyError::config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
    all: Arc<IndexMap<String, IndexMap<String, String>>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            None => match opts.default {
                None => Err(must_specified(option, &self.name)),
                Some(v) => Ok(v),
            },
            Some(v) => Ok(Some(v.clone())),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.options.get(option) {
            None => match opts.default {
                None => return Err(must_specified(option, &self.name)),
                Some(v) => return Ok(v),
            },
            Some(v) => v,
        };
        let v = raw
            .trim()
            .parse::<i64>()
            .map_err(|_| unable(option, &self.name))?;
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minv}",
                    self.name
                )));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxv}",
                    self.name
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.options.get(option) {
            None => match opts.default {
                None => return Err(must_specified(option, &self.name)),
                Some(v) => return Ok(v),
            },
            Some(v) => v,
        };
        let v = match pycompat::text::float_from_string(raw) {
            Ok(x) => x,
            Err(_) => return Err(unable(option, &self.name)),
        };
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    py_float_repr(minv)
                )));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    py_float_repr(maxv)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    py_float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    py_float_repr(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let raw = match self.options.get(option) {
            None => match opts.default {
                None => return Err(must_specified(option, &self.name)),
                Some(v) => return Ok(v),
            },
            Some(v) => v,
        };
        let l = raw.trim().to_lowercase();
        if ["1", "yes", "true", "on"].contains(&l.as_str()) {
            return Ok(Some(true));
        }
        if ["0", "no", "false", "off"].contains(&l.as_str()) {
            return Ok(Some(false));
        }
        Err(unable(option, &self.name))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by safe_z_home")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by safe_z_home")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by safe_z_home")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by safe_z_home")
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let raw = match self.options.get(option) {
            None => match opts.default {
                None => return Err(must_specified(option, &self.name)),
                Some(v) => return Ok(v),
            },
            Some(v) => v,
        };
        let mut res = Vec::new();
        if !raw.trim().is_empty() {
            for part in raw.split(&opts.sep) {
                let p = part.trim().to_string();
                match pycompat::text::float_from_string(&p) {
                    Ok(v) => res.push(v),
                    Err(_) => return Err(unable(option, &self.name)),
                }
            }
        }
        if let Some(count) = opts.count {
            if res.len() != count {
                return Err(klippy_api::KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.name
                )));
            }
        }
        Ok(Some(res))
    }
    fn getsection(&self, section: &str) -> Config {
        let opts = self.all.get(section).cloned().unwrap_or_default();
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            options: opts,
            all: self.all.clone(),
        }) as Config
    }
    fn has_section(&self, section: &str) -> bool {
        self.all.contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.all
            .iter()
            .filter(|(k, _)| k.starts_with(prefix))
            .map(|(k, _)| self.getsection(k))
            .collect()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

// ---------------------------------------------------------------------------
// Scenario helpers.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Scenario {
    safe_opts: Vec<(String, String)>,
    all_sections: Vec<(String, Vec<(String, String)>)>,
    pos: Vec<f64>,
    homed: String,
    registered: bool,
}

fn map_of(pairs: &[(String, String)]) -> IndexMap<String, String> {
    pairs.iter().cloned().collect()
}

fn py_str(s: &str) -> String {
    format!("'{s}'")
}

fn py_float_list(pos: &[f64]) -> String {
    let parts: Vec<String> = pos
        .iter()
        .map(|v| {
            let r = py_float_repr(*v);
            // Python source needs e.g. 10.0 (valid float literal); py_float_repr gives that.
            r
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn py_dict(pairs: &[(String, String)]) -> String {
    if pairs.is_empty() {
        return "{}".to_string();
    }
    let parts: Vec<String> = pairs
        .iter()
        .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn make_rust_fixture(
    scenario: &Scenario,
) -> (
    Option<Arc<SafeZHoming>>,
    KResult<()>,
    Config,
    Log,
    State,
    Arc<FakeGCode>,
    Arc<FakePrinter>,
) {
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let state: State = Arc::new(Mutex::new(SharedState {
        pos: scenario.pos.clone(),
        homed: scenario.homed.clone(),
    }));
    let gcode = Arc::new(FakeGCode {
        log: log.clone(),
        handlers: Mutex::new(IndexMap::new()),
    });
    let toolhead = Arc::new(FakeToolHead {
        state: state.clone(),
        log: log.clone(),
    });
    let printer: Printer = Arc::new(FakePrinter {
        gcode: gcode.clone(),
        toolhead: toolhead.clone(),
        log: log.clone(),
    }) as Printer;
    // Build all-section map for config: keep insertion order.
    let mut all = IndexMap::new();
    for (sec, opts) in &scenario.all_sections {
        all.insert(sec.clone(), map_of(opts));
    }
    // Ensure safe_z_home section exists for has_section checks, even when
    // the scenario lists it only via safe_opts.
    if !all.contains_key("safe_z_home") {
        all.insert("safe_z_home".to_string(), map_of(&scenario.safe_opts));
    }
    let all_arc = Arc::new(all);
    let safe_map = all_arc.get("safe_z_home").unwrap().clone();
    if scenario.registered {
        let state2 = state.clone();
        let log2 = log.clone();
        let handler: GCodeCommandHandler = Box::new(move |gcmd: &klippy_api::gcode::GCmd| {
            let params = gcmd.get_command_parameters();
            log2.lock()
                .unwrap()
                .push(format!("prev_G28:{}", fmt_params(&params)));
            let mut s = state2.lock().unwrap();
            if params.contains_key("X") {
                if !s.homed.contains('x') {
                    s.homed.push('x');
                }
                if s.pos.len() >= 1 {
                    s.pos[0] = 0.0;
                }
            }
            if params.contains_key("Y") {
                if !s.homed.contains('y') {
                    s.homed.push('y');
                }
                if s.pos.len() >= 2 {
                    s.pos[1] = 0.0;
                }
            }
            if params.contains_key("Z") {
                if !s.homed.contains('z') {
                    s.homed.push('z');
                }
                if s.pos.len() >= 3 {
                    s.pos[2] = 0.0;
                }
            }
            Ok(())
        });
        let _ = gcode.register_command("G28", Some(handler), RegisterCommandOpts::default());
    }
    log.lock().unwrap().clear();
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: "safe_z_home".to_string(),
        options: safe_map,
        all: all_arc,
    }) as Config;
    let result = SafeZHoming::new(&config);
    let err_state = match &result {
        Ok(_) => Ok(()),
        Err(e) => Err(e.clone()),
    };
    let _printer_obj = printer.clone();
    let printer_fake = Arc::new(FakePrinter {
        gcode: gcode.clone(),
        toolhead: toolhead.clone(),
        log: log.clone(),
    });
    (
        result.ok().map(|a| a),
        err_state,
        config,
        log,
        state,
        gcode,
        printer_fake,
    )
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("safe_z_home fakes");
            Some(py)
        }
        None => {
            println!("skipping safe_z_home oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
            None
        }
    }
}

fn py_setup(py: &mut pyoracle::Oracle, scenario: &Scenario) {
    // Insertion order of all_sections must match the Rust side.
    let safe_dict = py_dict(&scenario.safe_opts);
    let mut all_parts: Vec<String> = Vec::new();
    for (sec, opts) in &scenario.all_sections {
        all_parts.push(format!("{}: {}", py_str(sec), py_dict(opts)));
    }
    if !scenario
        .all_sections
        .iter()
        .any(|(s, _)| s == "safe_z_home")
    {
        all_parts.push(format!("'safe_z_home': {safe_dict}"));
    }
    let all_dict = format!("{{{}}}", all_parts.join(", "));
    let pos_str = py_float_list(&scenario.pos);
    let homed_str = py_str(&scenario.homed);
    let prev_code = if scenario.registered {
        r#"
def _prev(gcmd):
    params = gcmd.get_command_parameters()
    log.append("prev_G28:" + repr(list(params.items())))
    if 'X' in params:
        if 'x' not in th_state['homed']:
            th_state['homed'] += 'x'
        th_state['pos'][0] = 0.0
    if 'Y' in params:
        if 'y' not in th_state['homed']:
            th_state['homed'] += 'y'
        th_state['pos'][1] = 0.0
    if 'Z' in params:
        if 'z' not in th_state['homed']:
            th_state['homed'] += 'z'
        th_state['pos'][2] = 0.0
gcode.register_command('G28', _prev)
"#
    } else {
        ""
    };
    let code = format!(
        r#"
log.clear()
printer = FakePrinter()
th_state = {{'pos': {pos_str}, 'homed': {homed_str}}}
toolhead = FakeToolHead(th_state)
gcode = FakeGCode(printer)
printer.objects['toolhead'] = toolhead
printer.objects['gcode'] = gcode
{prev_code}
log.clear()
config = FakeConfig(printer, 'safe_z_home', {safe_dict}, {all_dict})
"#,
        pos_str = pos_str,
        homed_str = homed_str,
        prev_code = prev_code,
        safe_dict = safe_dict,
        all_dict = all_dict,
    );
    py.exec(&code).expect("python scenario setup");
}

fn py_log(py: &mut pyoracle::Oracle) -> Vec<Value> {
    match py.eval("list(log)") {
        Ok(Value::List(items)) => items,
        other => panic!("py log eval failed: {other:?}"),
    }
}

fn rust_log(log: &Log) -> Vec<Value> {
    log.lock()
        .unwrap()
        .iter()
        .map(|s| Value::Str(s.clone()))
        .collect()
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "safe_z_home".to_string(),
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

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: f64) {
    check(label, expected, Value::float(actual));
}
fn check_bool(label: &str, expected: pyoracle::OracleResult, actual: bool) {
    check(label, expected, Value::Bool(actual));
}


fn check_cmd_err(label: &str, expected: pyoracle::OracleResult, actual: &KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust succeeded but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

// Construct on both sides; asserts construction parity (error or fields+logs).
// Returns the Rust object (None if construction failed on both).
fn construct_pair(
    py: &mut pyoracle::Oracle,
    label: &str,
    scenario: &Scenario,
) -> Option<Arc<SafeZHoming>> {
    py_setup(py, scenario);
    let (rust_obj, rust_res, _cfg, log, _state, _gcode, _pf) = make_rust_fixture(scenario);
    let py_res = py.exec("obj = safe_mod.load_config(config)");
    let py_log_vals = py_log(py);
    let rust_log_vals = rust_log(&log);
    // Compare logs regardless of success/failure.
    let record = Record {
        v: 1,
        unit: "safe_z_home".to_string(),
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
    if let Err(d) = assert_equal(
        &record,
        "log",
        &Value::List(py_log_vals),
        &Value::List(rust_log_vals),
    ) {
        panic!("{label}: construction log divergence: {d}");
    }
    match (&py_res, &rust_res) {
        (Ok(_), Ok(_)) => {
            let obj = rust_obj.unwrap();
            check_float(
                &format!("{label} home_x_pos"),
                py.eval("obj.home_x_pos"),
                obj.home_x_pos(),
            );
            check_float(
                &format!("{label} home_y_pos"),
                py.eval("obj.home_y_pos"),
                obj.home_y_pos(),
            );
            check_float(&format!("{label} z_hop"), py.eval("obj.z_hop"), obj.z_hop());
            check_float(
                &format!("{label} z_hop_speed"),
                py.eval("obj.z_hop_speed"),
                obj.z_hop_speed(),
            );
            check_float(&format!("{label} max_z"), py.eval("obj.max_z"), obj.max_z());
            check_float(&format!("{label} speed"), py.eval("obj.speed"), obj.speed());
            check_bool(
                &format!("{label} move_to_previous"),
                py.eval("obj.move_to_previous"),
                obj.move_to_previous(),
            );
            Some(obj)
        }
        (Err(_), Err(_)) => {
            // Rebuild rust error for comparison: py_res already carries oracle exc.
            let scenario2 = scenario.clone();
            let (_o, rust_res2, _c, _l, _s, _g, _p) = make_rust_fixture(&scenario2);
            // rust_res2 error for reporting; compare against oracle exc stored in py_res.
            let _ = rust_res2;
            // Compare directly using stored oracle error: re-run pattern via helper
            // isComplex; use assert on classes from fresh check below.
            // We already know both are Err; compare class/message via fresh pair:
            let (obj_opt, rust_res3, _c3, _l3, _s3, _g3, _p3) = make_rust_fixture(scenario);
            assert!(obj_opt.is_none());
            let py_err = py_res.err().unwrap();
            let rust_err = rust_res3.err().unwrap();
            assert_eq!(
                rust_err.class_name(),
                py_err.class_name.as_str(),
                "{label}: exception class"
            );
            assert_eq!(
                rust_err.message(),
                py_err.message.as_str(),
                "{label}: exception message"
            );
            None
        }
        (Ok(_), Err(e)) => panic!(
            "{label}: oracle succeeded but rust raised {}: {}",
            e.class_name(),
            e.message()
        ),
        (Err(e), Ok(_)) => panic!(
            "{label}: oracle raised {}: {} but rust succeeded",
            e.class_name, e.message
        ),
    }
}

// Run one G28 command on both sides with identical params; compare errors+logs.
fn run_g28(
    py: &mut pyoracle::Oracle,
    label: &str,
    obj: &Arc<SafeZHoming>,
    log: &Log,
    state: &State,
    params: Vec<(String, String)>,
    _gcode: &Arc<FakeGCode>,
) {
    // Reset both sides to a known starting point? The caller sets pos/homed
    // before construction; construction does not move. We keep that state.
    py.exec("log.clear()").expect("clear py log");
    log.lock().unwrap().clear();
    let params_dict = py_dict(&params);
    py.exec(&format!(
        "gcmd = FakeGcmd(gcode, 'G28', 'G28', {params_dict})"
    ))
    .expect("py gcmd");
    let mut r_params = IndexMap::new();
    for (k, v) in params {
        r_params.insert(k, v);
    }
    let r_gcmd = Arc::new(FakeGcmd {
        command: "G28".to_string(),
        commandline: "G28".to_string(),
        params: r_params,
    }) as klippy_api::gcode::GCmd;
    let py_res = py.eval("obj.cmd_G28(gcmd)");
    let rust_res = obj.cmd_g28(&r_gcmd);
    match (&py_res, &rust_res) {
        (Ok(_), Ok(_)) => {}
        (Err(_), Err(_)) => check_cmd_err(label, py_res, &rust_res),
        (Ok(_), Err(e)) => panic!(
            "{label}: oracle succeeded but rust raised {}: {}",
            e.class_name(),
            e.message()
        ),
        (Err(e), Ok(_)) => panic!(
            "{label}: oracle raised {}: {} but rust succeeded",
            e.class_name, e.message
        ),
    }
    let py_log_vals = py_log(py);
    let rust_log_vals = rust_log(log);
    let record = Record {
        v: 1,
        unit: "safe_z_home".to_string(),
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
    if let Err(d) = assert_equal(
        &record,
        "log",
        &Value::List(py_log_vals),
        &Value::List(rust_log_vals),
    ) {
        panic!("{label}: G28 log divergence: {d}");
    }
    let _ = state;
}

fn base_scenario() -> Scenario {
    Scenario {
        safe_opts: vec![("home_xy_position".to_string(), "10, 20".to_string())],
        all_sections: vec![(
            "stepper_z".to_string(),
            vec![("position_max".to_string(), "200".to_string())],
        )],
        pos: vec![0.0, 0.0, 0.0, 0.0],
        homed: "xyz".to_string(),
        registered: true,
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn config_defaults() {
    let Some(mut py) = oracle() else { return };
    let sc = base_scenario();
    let obj = construct_pair(&mut py, "config_defaults", &sc).expect("defaults construct");
    assert_eq!(obj.z_hop(), 0.0);
    assert_eq!(obj.z_hop_speed(), 15.0);
    assert_eq!(obj.speed(), 50.0);
    assert!(!obj.move_to_previous());
}

#[test]
fn config_explicit() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "1.5, -2.25".to_string()),
        ("z_hop".to_string(), "5".to_string()),
        ("z_hop_speed".to_string(), "12.5".to_string()),
        ("speed".to_string(), "33".to_string()),
        ("move_to_previous".to_string(), "true".to_string()),
    ];
    sc.all_sections = vec![(
        "stepper_z".to_string(),
        vec![("position_max".to_string(), "187.5".to_string())],
    )];
    let obj = construct_pair(&mut py, "config_explicit", &sc).expect("explicit construct");
    assert_eq!(obj.home_x_pos(), 1.5);
    assert_eq!(obj.home_y_pos(), -2.25);
}

#[test]
fn missing_home_xy() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = Vec::new();
    construct_pair(&mut py, "missing_home_xy", &sc);
}

#[test]
fn home_xy_count_mismatch() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![("home_xy_position".to_string(), "10".to_string())];
    construct_pair(&mut py, "home_xy_count_short", &sc);
    let mut sc2 = base_scenario();
    sc2.safe_opts = vec![("home_xy_position".to_string(), "1, 2, 3".to_string())];
    construct_pair(&mut py, "home_xy_count_long", &sc2);
}

#[test]
fn home_xy_unparseable() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![("home_xy_position".to_string(), "a, b".to_string())];
    construct_pair(&mut py, "home_xy_unparseable", &sc);
}

#[test]
fn z_hop_speed_above() {
    let Some(mut py) = oracle() else { return };
    for (label, val) in [("zero", "0"), ("neg", "-1")] {
        let mut sc = base_scenario();
        sc.safe_opts = vec![
            ("home_xy_position".to_string(), "10, 20".to_string()),
            ("z_hop_speed".to_string(), val.to_string()),
        ];
        construct_pair(&mut py, &format!("z_hop_speed_above_{label}"), &sc);
    }
}

#[test]
fn speed_above() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("speed".to_string(), "0".to_string()),
    ];
    construct_pair(&mut py, "speed_above_zero", &sc);
}

#[test]
fn missing_z_endstop() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.all_sections = Vec::new();
    construct_pair(&mut py, "missing_z_endstop", &sc);
}

#[test]
fn position_max_missing_and_bad() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.all_sections = vec![("stepper_z".to_string(), Vec::new())];
    construct_pair(&mut py, "position_max_missing", &sc);
    let mut sc2 = base_scenario();
    sc2.all_sections = vec![(
        "stepper_z".to_string(),
        vec![("position_max".to_string(), "abc".to_string())],
    )];
    construct_pair(&mut py, "position_max_unparseable", &sc2);
}

#[test]
fn carriage_fallback() {
    let Some(mut py) = oracle() else { return };
    // carriage named z via name suffix.
    let mut sc = base_scenario();
    sc.all_sections = vec![(
        "carriage z".to_string(),
        vec![("position_max".to_string(), "150".to_string())],
    )];
    let obj = construct_pair(&mut py, "carriage_name_z", &sc).expect("carriage z construct");
    assert_eq!(obj.max_z(), 150.0);
    // carriage with explicit axis=z while name is not z.
    let mut sc2 = base_scenario();
    sc2.all_sections = vec![(
        "carriage foo".to_string(),
        vec![
            ("axis".to_string(), "z".to_string()),
            ("position_max".to_string(), "160".to_string()),
        ],
    )];
    let obj2 = construct_pair(&mut py, "carriage_axis_z", &sc2).expect("carriage axis construct");
    assert_eq!(obj2.max_z(), 160.0);
    // carriage x (not z) -> missing error.
    let mut sc3 = base_scenario();
    sc3.all_sections = vec![(
        "carriage x".to_string(),
        vec![("position_max".to_string(), "150".to_string())],
    )];
    construct_pair(&mut py, "carriage_not_z", &sc3);
    // First matching carriage wins.
    let mut sc4 = base_scenario();
    sc4.all_sections = vec![
        (
            "carriage x".to_string(),
            vec![("position_max".to_string(), "150".to_string())],
        ),
        (
            "carriage z".to_string(),
            vec![("position_max".to_string(), "170".to_string())],
        ),
    ];
    let obj4 =
        construct_pair(&mut py, "carriage_second_matches", &sc4).expect("carriage scan construct");
    assert_eq!(obj4.max_z(), 170.0);
}

#[test]
fn stepper_z_preferred_over_carriage() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.all_sections = vec![
        (
            "carriage z".to_string(),
            vec![("position_max".to_string(), "150".to_string())],
        ),
        (
            "stepper_z".to_string(),
            vec![("position_max".to_string(), "250".to_string())],
        ),
    ];
    let obj = construct_pair(&mut py, "stepper_z_preferred", &sc).expect("preferred construct");
    assert_eq!(obj.max_z(), 250.0);
}

#[test]
fn homing_override_conflict() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.all_sections = vec![
        (
            "stepper_z".to_string(),
            vec![("position_max".to_string(), "200".to_string())],
        ),
        ("homing_override".to_string(), Vec::new()),
    ];
    construct_pair(&mut py, "homing_override_conflict", &sc);
}

#[test]
fn move_to_previous_bad_boolean() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("move_to_previous".to_string(), "maybe".to_string()),
    ];
    construct_pair(&mut py, "move_to_previous_bad", &sc);
}

#[test]
fn boolean_spellings() {
    let Some(mut py) = oracle() else { return };
    for spelling in ["yes", "on", "1", "True", "false", "no", "0"] {
        let mut sc = base_scenario();
        sc.safe_opts = vec![
            ("home_xy_position".to_string(), "10, 20".to_string()),
            ("move_to_previous".to_string(), spelling.to_string()),
        ];
        construct_pair(&mut py, &format!("bool_{spelling}"), &sc);
    }
}

#[test]
fn lookup_z_helper() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.all_sections = vec![(
        "carriage z".to_string(),
        vec![("position_max".to_string(), "150".to_string())],
    )];
    py_setup(&mut py, &sc);
    let expected = py.eval("safe_mod.manual_probe.lookup_z_endstop_config(config)");
    // manual_probe is imported by safe_z_home as a module attribute.
    let expected = match expected {
        Ok(v) => {
            // helper returns a FakeConfig; encode its name for comparison.
            let _ = v;
            py.eval("safe_mod.manual_probe.lookup_z_endstop_config(config).get_name()")
        }
        Err(e) => Err(e),
    };
    let (_o, _r, config, _l, _s, _g, _p) = make_rust_fixture(&sc);
    let z = lookup_z_endstop_config(&config)
        .expect("rust lookup")
        .expect("found");
    check("lookup_z_helper", expected, Value::Str(z.get_name()));
}

#[test]
fn g28_no_hop_all_axes() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.pos = vec![1.0, 2.0, 3.0, 0.0];
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    // Use the fixture's construction in both sides: re-construct via pairs
    // would reset state; instead drive both directly here.
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    log.lock().unwrap().clear();
    // Note: construction logs are intentionally dropped; G28 logs start clean.
    py.exec("log.clear()").expect("clear");
    // Replace the freshly built python object name: exec above sets obj.
    run_g28(
        &mut py,
        "g28_no_hop_all",
        &rust_obj,
        &log,
        &state,
        Vec::new(),
        &gcode,
    );
}

#[test]
fn g28_axis_subsets_no_hop() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(String, String)>)> = vec![
        ("x_only", vec![("X".to_string(), "0".to_string())]),
        ("y_only", vec![("Y".to_string(), "0".to_string())]),
        ("z_only", vec![("Z".to_string(), "0".to_string())]),
        (
            "xy_only",
            vec![
                ("X".to_string(), "0".to_string()),
                ("Y".to_string(), "0".to_string()),
            ],
        ),
        (
            "xz_only",
            vec![
                ("X".to_string(), "0".to_string()),
                ("Z".to_string(), "0".to_string()),
            ],
        ),
        (
            "yz_only",
            vec![
                ("Y".to_string(), "0".to_string()),
                ("Z".to_string(), "0".to_string()),
            ],
        ),
    ];
    for (label, params) in cases {
        let mut sc = base_scenario();
        sc.pos = vec![5.0, 6.0, 7.0, 0.0];
        sc.homed = "xyz".to_string();
        py_setup(&mut py, &sc);
        let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
        let rust_obj = obj.expect("construct");
        py.exec("obj = safe_mod.load_config(config)")
            .expect("py construct");
        run_g28(&mut py, label, &rust_obj, &log, &state, params, &gcode);
    }
}

#[test]
fn g28_z_requires_xy() {
    let Some(mut py) = oracle() else { return };
    // Z-only with x homed but y not homed must fail.
    let mut sc = base_scenario();
    sc.pos = vec![1.0, 2.0, 3.0, 0.0];
    sc.homed = "x".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "z_only_missing_y",
        &rust_obj,
        &log,
        &state,
        vec![("Z".to_string(), "0".to_string())],
        &gcode,
    );
    // Full G28 succeeds because XY is homed first, then Z check passes.
    let mut sc2 = base_scenario();
    sc2.pos = vec![1.0, 2.0, 3.0, 0.0];
    sc2.homed = "".to_string();
    py_setup(&mut py, &sc2);
    let (obj2, _res2, _cfg2, log2, state2, gcode2, _pf2) = make_rust_fixture(&sc2);
    let rust_obj2 = obj2.expect("construct2");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct2");
    run_g28(
        &mut py,
        "full_from_unhomed",
        &rust_obj2,
        &log2,
        &state2,
        Vec::new(),
        &gcode2,
    );
}

#[test]
fn g28_pre_hop_homed_below() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "5".to_string()),
        ("z_hop_speed".to_string(), "12".to_string()),
    ];
    sc.pos = vec![1.0, 2.0, 2.0, 0.0]; // below hop
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "pre_hop_below",
        &rust_obj,
        &log,
        &state,
        vec![("X".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn g28_pre_hop_homed_above() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "5".to_string()),
        ("z_hop_speed".to_string(), "12".to_string()),
    ];
    sc.pos = vec![1.0, 2.0, 8.0, 0.0]; // above hop
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "pre_hop_above",
        &rust_obj,
        &log,
        &state,
        vec![("X".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn g28_pre_hop_unhomed_z() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "5".to_string()),
        ("z_hop_speed".to_string(), "12".to_string()),
    ];
    sc.pos = vec![1.0, 2.0, 100.0, 0.0]; // far above, but unhomed forces hop
    sc.homed = "xy".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "pre_hop_unhomed",
        &rust_obj,
        &log,
        &state,
        vec![("X".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn g28_second_hop_and_move_previous() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "5".to_string()),
        ("z_hop_speed".to_string(), "12".to_string()),
        ("speed".to_string(), "40".to_string()),
        ("move_to_previous".to_string(), "true".to_string()),
    ];
    sc.pos = vec![1.5, 2.5, 10.0, 0.0];
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    // Z only: safe XY move, home Z (prev puts Z at 0), second hop to 5,
    // then move XY back to (1.5, 2.5).
    run_g28(
        &mut py,
        "second_hop_move_prev",
        &rust_obj,
        &log,
        &state,
        vec![("Z".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn g28_second_hop_negative() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "-5".to_string()),
        ("z_hop_speed".to_string(), "12".to_string()),
    ];
    sc.pos = vec![1.0, 2.0, 10.0, 0.0];
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    // After Z homing Z=0 which is not < -5, so no second hop.
    run_g28(
        &mut py,
        "second_hop_negative",
        &rust_obj,
        &log,
        &state,
        vec![("Z".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn g28_no_prev_type_error() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.registered = false;
    sc.pos = vec![1.0, 2.0, 3.0, 0.0];
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "no_prev_xy",
        &rust_obj,
        &log,
        &state,
        vec![("X".to_string(), "0".to_string())],
        &gcode,
    );
}

#[test]
fn factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::safe_z_home::module_factory();
    let load_config = factory
        .load_config
        .expect("safe_z_home exposes load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let sc = base_scenario();
    py_setup(&mut py, &sc);
    // Drive the factory on the Rust side against the same config.
    let (_o, _r, config, log, state, gcode, _pf) = make_rust_fixture(&sc);
    // make_rust_fixture already built one object; build a second through
    // the factory using a fresh config sharing the same printer fakes.
    // Simplest: use the object's config by constructing a fresh pair.
    let _ = (py, sc.clone(), config, log, state, gcode);
    // Direct factory check with a minimal fresh config:
    let mut sc2 = base_scenario();
    sc2.safe_opts = vec![("home_xy_position".to_string(), "3, 4".to_string())];
    let (obj2, _res2, cfg2, _l2, _s2, _g2, _p2) = make_rust_fixture(&sc2);
    // The fixture built via SafeZHoming::new; now build via factory on a
    // fresh fixture config to prove the entry point works.
    let obj_via_factory = load_config(&cfg2)
        .expect("factory load_config")
        .expect("factory returns object");
    let via_factory: Arc<SafeZHoming> =
        klippy_api::downcast_arc(&obj_via_factory).expect("downcast");
    assert_eq!(via_factory.home_x_pos(), 3.0);
    assert_eq!(via_factory.home_y_pos(), 4.0);
    let _ = obj2;
    // Phantom Any use to anchor imports for unused symbols in this file.
    let _ = std::any::TypeId::of::<i32>();
}

#[test]
fn short_position_index_error() {
    let Some(mut py) = oracle() else { return };
    let mut sc = base_scenario();
    sc.safe_opts = vec![
        ("home_xy_position".to_string(), "10, 20".to_string()),
        ("z_hop".to_string(), "5".to_string()),
    ];
    sc.pos = vec![1.0, 2.0]; // too short for pos[2]
    sc.homed = "xyz".to_string();
    py_setup(&mut py, &sc);
    let (obj, _res, _cfg, log, state, gcode, _pf) = make_rust_fixture(&sc);
    let rust_obj = obj.expect("construct");
    py.exec("obj = safe_mod.load_config(config)")
        .expect("py construct");
    run_g28(
        &mut py,
        "short_pos",
        &rust_obj,
        &log,
        &state,
        vec![("X".to_string(), "0".to_string())],
        &gcode,
    );
}
