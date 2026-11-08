//! Parity tests for `extras::manual_probe` (port of `klippy/extras/manual_probe.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the Python module is built on
//! minimal fakes for the printer, gcode dispatch, toolhead, kinematics,
//! gcode_move and configfile, and the Rust port is driven through fakes of
//! the same contract traits with identical inputs. Log lines, status dicts,
//! toolhead moves, configfile writes and raised errors (class and message)
//! are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::manual_probe::{
    create_probe_result, lookup_z_endstop_config, module_factory, verify_no_manual_probe,
    ManualProbe, ManualProbeHelper, FinalizeCallback, ProbeResult, BISECT_MAX, Z_BOB_MINIMUM,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{McuStepperApi, Stepper, StepperFactory, Trapq, TrapqApi};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateHandle, KinematicsApi, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::BTreeSet;
use std::sync::{Arc, Mutex};
use indexmap::IndexMap;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------
const SETUP: &str = r#"
import configparser
import collections
import extras.manual_probe as manual_probe

class CommandError(Exception):
    pass

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.handlers = {}
        self.help = {}
        self.log = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            return self.handlers.pop(cmd, None)
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
        return None
    def respond_info(self, msg, log=True):
        self.log.append(msg)
    def respond_raw(self, msg):
        self.log.append(msg)

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            raise configparser.Error("Unable to find object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=None):
        return self.lookup_object(section, default)

class FakeStepper:
    def __init__(self, name):
        self._name = name
    def get_name(self, short=False):
        return self._name
    def get_commanded_position(self):
        return 0.0

class FakeKinematics:
    def __init__(self, toolhead):
        self.toolhead = toolhead
    def get_steppers(self):
        return [FakeStepper('stepper_x'), FakeStepper('stepper_y'), FakeStepper('stepper_z')]
    def calc_position(self, spos):
        return list(self.toolhead.position)

class FakeToolhead:
    def __init__(self, position):
        self.position = list(position)
        self.kin = FakeKinematics(self)
        self.moves = []
        self.fail_move = False
        self.flushes = 0
    def get_position(self):
        return list(self.position)
    def flush_step_generation(self):
        self.flushes += 1
    def get_kinematics(self):
        return self.kin
    def manual_move(self, coord, speed):
        if self.fail_move:
            raise CommandError('Move out of range')
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

Coord = collections.namedtuple('Coord', 'x y z e')

class FakeGCodeMove:
    def __init__(self, offset):
        self.offset = offset
    def get_status(self, eventtime=None):
        return {'homing_origin': Coord(0.0, 0.0, self.offset, 0.0)}

class FakeConfigFile:
    def __init__(self):
        self.writes = []
    def set(self, section, option, value):
        self.writes.append((section, option, value))

_sentinel = object()

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, sections):
        self._printer = printer
        self._name = name
        self.sections = sections
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def has_section(self, section):
        return section in self.sections
    def getsection(self, section):
        return FakeConfig(self._printer, section, self.sections)
    def get_prefix_sections(self, prefix):
        return [FakeConfig(self._printer, name, self.sections)
                for name in self.sections if name.startswith(prefix)]
    def _opt(self, option, default):
        opts = self.sections.get(self._name, {})
        if option in opts:
            return opts[option]
        if default is _sentinel:
            raise configparser.Error("No option '%s' in section '%s'" % (option, self._name))
        return default
    def get(self, option, default=_sentinel, note_valid=True):
        return self._opt(option, default)
    def getfloat(self, option, default=_sentinel, note_valid=True):
        v = self._opt(option, default)
        return v if v is _sentinel or v is None else float(v)

class FakeGcmd:
    error = CommandError
    def __init__(self, gcode, command, params):
        self._gcode = gcode
        self._command = command
        s = command
        for k in sorted(params):
            s += ' %s=%s' % (k, params[k])
        self._commandline = s
        self._params = dict(params)
    def respond_info(self, msg, log=True):
        self._gcode.log.append(msg)
    def get(self, name, default=_sentinel, parser=str, minval=None, maxval=None,
            above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
        return value
    def get_float(self, name, default=_sentinel, **kw):
        return self.get(name, default, parser=float, **kw)

def make_sections(kinematics='cartesian', z_endstop='5.0', include_z=True):
    s = {
        'printer': {'kinematics': kinematics},
        'stepper_a': {'position_endstop': '10.0'},
        'stepper_b': {'position_endstop': '11.0'},
        'stepper_c': {'position_endstop': '12.0'},
    }
    if include_z:
        s['stepper_z'] = {'position_endstop': z_endstop}
    return s

def py_setup(kinematics='cartesian', z_endstop='5.0', include_z=True, offset=0.0):
    p = FakePrinter()
    g = FakeGCode()
    th = FakeToolhead([10.0, 10.0, 10.0, 0.0])
    gm = FakeGCodeMove(offset)
    cf = FakeConfigFile()
    p.add_object('gcode', g)
    p.add_object('toolhead', th)
    p.add_object('gcode_move', gm)
    p.add_object('configfile', cf)
    c = FakeConfig(p, 'manual_probe', make_sections(kinematics, z_endstop, include_z))
    mp = manual_probe.load_config(c)
    p.add_object('manual_probe', mp)
    return p, g, th, gm, cf, c, mp
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Sections = Arc<Mutex<IndexMap<String, IndexMap<String, String>>>>;

fn sections(entries: &[(&str, &[(&str, &str)])]) -> Sections {
    let mut map = IndexMap::new();
    for (name, opts) in entries {
        let mut inner = IndexMap::new();
        for (k, v) in *opts {
            inner.insert(k.to_string(), v.to_string());
        }
        map.insert(name.to_string(), inner);
    }
    Arc::new(Mutex::new(map))
}

fn cartesian_sections() -> Sections {
    sections(&[
        ("printer", &[("kinematics", "cartesian")]),
        ("stepper_a", &[("position_endstop", "10.0")]),
        ("stepper_b", &[("position_endstop", "11.0")]),
        ("stepper_c", &[("position_endstop", "12.0")]),
        ("stepper_z", &[("position_endstop", "5.0")]),
    ])
}

#[derive(Clone, Default)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(self.objects.lock().unwrap().get(name).cloned())
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.lock().unwrap().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _config: &Config, section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(self.objects.lock().unwrap().get(section).cloned())
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _c: EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _r: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    sections: Sections,
}

impl FakeConfig {
    fn make(printer: Printer, name: &str, sections: Sections) -> Config {
        Arc::new(FakeConfig { printer, name: name.to_string(), sections })
    }
    fn opt(&self, option: &str) -> Option<String> {
        self.sections.lock().unwrap().get(&self.name)?.get(option).cloned()
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { self.name.clone() }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opt(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "No option '{option}' in section '{}'", self.name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opt(option) {
            Some(v) => v.parse::<f64>().map(Some).map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'", self.name))
            }),
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "No option '{option}' in section '{}'", self.name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::make(self.printer.clone(), section, self.sections.clone())
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.lock().unwrap().contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        let names: Vec<String> = self.sections.lock().unwrap().keys()
            .filter(|n| n.starts_with(prefix)).cloned().collect();
        names.iter().map(|n| self.getsection(n)).collect()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> { Vec::new() }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) {}
}

struct FakeGCode {
    handlers: Mutex<IndexMap<String, GCodeCommandHandler>>,
    tombstones: Mutex<BTreeSet<String>>,
    help: Mutex<IndexMap<String, String>>,
    log: Arc<Mutex<Vec<String>>>,
}

impl FakeGCode {
    fn new() -> Arc<Self> { Arc::new(FakeGCode {
        handlers: Mutex::new(IndexMap::new()),
        tombstones: Mutex::new(BTreeSet::new()),
        help: Mutex::new(IndexMap::new()),
        log: Arc::new(Mutex::new(Vec::new())),
    }) }
    fn commands(&self) -> Vec<String> {
        let mut v: Vec<String> = self.handlers.lock().unwrap().keys().cloned().collect();
        v.sort();
        v
    }
    fn dispatch(&self, cmd: &str, params: &[(&str, &str)]) -> KResult<()> {
        let mut handler = {
            let mut h = self.handlers.lock().unwrap();
            match h.shift_remove(cmd) {
                Some(h) => h,
                None => return Err(KlippyError::command(format!("unknown command {cmd}"))),
            }
        };
        let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGcmd::with_log(cmd, params, self.log.clone()));
        let result = handler(&gcmd);
        let tombstoned = self.tombstones.lock().unwrap().contains(cmd);
        if !tombstoned && result.is_ok() {
            self.handlers.lock().unwrap().insert(cmd.to_string(), handler);
        }
        result
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(&self, cmd: &str, func: Option<GCodeCommandHandler>, opts: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> {
        match func {
            None => {
                self.tombstones.lock().unwrap().insert(cmd.to_string());
                Ok(self.handlers.lock().unwrap().shift_remove(cmd))
            }
            Some(f) => {
                if self.handlers.lock().unwrap().contains_key(cmd) {
                    return Err(KlippyError::config(format!(
                        "gcode command {cmd} already registered")));
                }
                self.tombstones.lock().unwrap().remove(cmd);
                if let Some(d) = opts.desc {
                    self.help.lock().unwrap().insert(cmd.to_string(), d);
                }
                self.handlers.lock().unwrap().insert(cmd.to_string(), f);
                Ok(None)
            }
        }
    }
    fn register_mux_command(&self, _c: &str, _k: &str, _v: Option<&str>, _f: GCodeCommandHandler, _d: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { self.help.lock().unwrap().clone() }
    fn get_status(&self, _eventtime: f64) -> PyValue { PyValue::None }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { Reactor::new(false).mutex(false) }
    fn create_gcode_command(&self, command: &str, _commandline: &str, _params: &IndexMap<String, String>) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGcmd::new(command, &[]))
    }
    fn respond_raw(&self, msg: &str) { self.log.lock().unwrap().push(msg.to_string()); }
    fn respond_info(&self, msg: &str, _log: bool) { self.log.lock().unwrap().push(msg.to_string()); }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool { false }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    log: Arc<Mutex<Vec<String>>>,
}

impl FakeGcmd {
    fn new(command: &str, params: &[(&str, &str)]) -> Self {
        Self::with_log(command, params, Arc::new(Mutex::new(Vec::new())))
    }
    fn with_log(command: &str, params: &[(&str, &str)], log: Arc<Mutex<Vec<String>>>) -> Self {
        let mut commandline = command.to_string();
        let mut map = IndexMap::new();
        for (k, v) in params {
            commandline.push_str(&format!(" {k}={v}"));
            map.insert(k.to_string(), v.to_string());
        }
        FakeGcmd { command: command.to_string(), commandline, params: map, log }
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        let value = match self.params.get(name) {
            Some(v) => v.clone(),
            None => match opts.default {
                None => return Err(KlippyError::command(format!(
                    "Error on '{}': missing {}", self.commandline, name))),
                Some(d) => return Ok(d),
            },
        };
        use klippy_api::gcode::GetParser;
        match opts.parser {
            GetParser::Str => Ok(PyValue::Str(value)),
            GetParser::Float => value.parse::<f64>().map(PyValue::Float).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {}", self.commandline, value))
            }),
            GetParser::Int | GetParser::IntAutoBase => value.parse::<i64>().map(|n| {
                PyValue::Int(pycompat::PyInt::from_i64(n))
            }).map_err(|_| KlippyError::command(format!(
                "Error on '{}': unable to parse {}", self.commandline, value))),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let v = self.get(name, GGetOpts { default: opts.default.map(|d| d.map(|n| PyValue::Int(pycompat::PyInt::from_i64(n))).unwrap_or(PyValue::None)), ..GGetOpts::default() })?;
        match v { PyValue::Int(n) => Ok(Some(n.inner().to_string().parse().unwrap())), _ => Ok(None) }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            Some(v) => v.parse::<f64>().map(Some).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {}", self.commandline, v))
            }),
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}", self.commandline, name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_command(&self) -> String { self.command.clone() }
    fn get_commandline(&self) -> String { self.commandline.clone() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, msg: &str, _log: bool) { self.log.lock().unwrap().push(msg.to_string()); }
    fn respond_raw(&self, msg: &str) { self.log.lock().unwrap().push(msg.to_string()); }
    fn ack(&self, _msg: Option<&str>) -> bool { false }
}

fn gcmd(command: &str, params: &[(&str, &str)]) -> klippy_api::gcode::GCmd {
    Arc::new(FakeGcmd::new(command, params))
}

struct FakeConfigFile {
    writes: Mutex<Vec<(String, String, String)>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer { panic!("get_printer not used") }
    fn read_config(&self, _f: &str) -> KResult<Config> { panic!("not used") }
    fn read_main_config(&self) -> KResult<Config> { panic!("not used") }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> { Ok(()) }
    fn runtime_warning(&self, _m: &str) {}
    fn deprecate(&self, _s: &str, _o: &str, _v: Option<PyValue>, _m: Option<String>) {}
    fn deprecate_gcode(&self, _c: &str, _o: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _m: &klippy_api::mcu::Mcu, _f: &str, _m2: Option<String>) {}
    fn get_status(&self, _e: Option<f64>) -> PyValue { PyValue::None }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.writes.lock().unwrap().push((section.to_string(), option.to_string(), value.to_string()));
    }
    fn remove_section(&self, _s: &str) {}
}

impl PrinterObject for FakeConfigFile {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

struct FakeGCodeMove {
    offset: f64,
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("homing_origin".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(0.0), PyValue::Float(0.0),
                PyValue::Float(self.offset), PyValue::Float(0.0)]));
        PyValue::Dict(d)
    }
    fn set_move_transform(&self, _t: klippy_api::gcode::MoveTransform, _f: bool) -> KResult<Option<klippy_api::gcode::MoveTransform>> { Ok(None) }
    fn reset_last_position(&self) {}
    fn cmd_g1(&self, _gcmd: &klippy_api::gcode::GCmd) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeTrapq;
impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any { self }
}

struct FakeStepper { name: String }
impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { panic!("not used") }
    fn get_name(&self, _short: Option<bool>) -> String { self.name.clone() }
    fn get_oid(&self) -> i64 { 0 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 { 0.0 }
    fn get_rotation_distance(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) { (false, false) }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 { 0.0 }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 { 0.0 }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 { 0 }
    fn get_past_mcu_position(&self, _p: f64) -> i64 { 0 }
    fn mcu_to_commanded_position(&self, _m: i64) -> f64 { 0.0 }
    fn dump_steps(&self, _c: i64, _s: i64, _e: i64) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) { (Vec::new(), 0) }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> { None }
    fn set_stepper_kinematics(&self, _sk: Option<klippy_api::stepper::StepperKinematics>) -> Option<klippy_api::stepper::StepperKinematics> { None }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<Trapq> { None }
    fn set_trapq(&self, _t: Option<Trapq>) -> Option<Trapq> { None }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _a: char) -> bool { false }
}

struct FakeKinematics { position: Arc<GCell<Vec<f64>>> }
impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        ["stepper_x", "stepper_y", "stepper_z"].iter()
            .map(|n| Arc::new(FakeStepper { name: n.to_string() }) as Stepper).collect()
    }
    fn calc_position(&self, _spos: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(self.position.borrow().iter().map(|v| Some(*v)).collect())
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> { Ok(()) }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> { Ok(()) }
    fn home(&self, _h: HomingStateHandle) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> PyValue { PyValue::None }
    fn clear_homing_state(&self, _c: &str) {}
}

struct FakeToolhead {
    position: Arc<GCell<Vec<f64>>>,
    moves: Mutex<Vec<(Vec<Option<f64>>, f64)>>,
    fail_move: Mutex<bool>,
    flushes: Mutex<i64>,
}

impl FakeToolhead {
    fn set_position(&self, pos: Vec<f64>) { *self.position.borrow_mut() = pos; }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> { self.position.borrow().clone() }
    fn set_position(&self, newpos: Vec<f64>, _h: &str) -> KResult<()> {
        self.set_position(newpos);
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        Arc::new(FakeKinematics { position: self.position.clone() })
    }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flushes.lock().unwrap() += 1;
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 { 0.0 }
    fn dwell(&self, _d: f64) {}
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &reactor::Completion<f64>) -> KResult<()> { Ok(()) }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        if *self.fail_move.lock().unwrap() {
            return Err(KlippyError::command("Move out of range"));
        }
        self.moves.lock().unwrap().push((coord.clone(), speed));
        let mut pos = self.position.borrow().clone();
        for (i, v) in coord.iter().enumerate() {
            if let Some(v) = v { pos[i] = *v; }
        }
        self.set_position(pos);
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _e: f64) -> PyValue { PyValue::None }
    fn get_trapq(&self) -> Trapq { Arc::new(FakeTrapq) }
    fn get_max_velocity(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_max_velocities(&self, _v: Option<f64>, _a: Option<f64>, _s: Option<f64>, _m: Option<f64>) -> (f64, f64, f64, f64) { (0.0, 0.0, 0.0, 0.0) }
    fn register_lookahead_callback(&self, _c: klippy_api::toolhead::LookaheadCallback) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder { panic!("not used") }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _e: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> { Vec::new() }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> StepperFactory { panic!("not used") }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct RustWorld {
    printer: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    configfile: Arc<FakeConfigFile>,
    probe: Arc<ManualProbe>,
}

fn rust_world(offset: f64) -> RustWorld {
    let fp = FakePrinter::default();
    let printer: Printer = Arc::new(fp.clone());
    let gcode = FakeGCode::new();
    let toolhead = Arc::new(FakeToolhead {
        position: Arc::new(GCell::new(vec![10.0, 10.0, 10.0, 0.0])),
        moves: Mutex::new(Vec::new()),
        fail_move: Mutex::new(false),
        flushes: Mutex::new(0),
    });
    let configfile = Arc::new(FakeConfigFile { writes: Mutex::new(Vec::new()) });
    let gmov = Arc::new(FakeGCodeMove { offset });
    fp.add("gcode", gcode.clone() as ObjectRef);
    fp.add("toolhead", toolhead.clone() as ObjectRef);
    fp.add("gcode_move", gmov as ObjectRef);
    fp.add("configfile", configfile.clone() as ObjectRef);
    let config = FakeConfig::make(printer.clone(), "manual_probe", cartesian_sections());
    let probe = ManualProbe::new(&config).expect("rust ManualProbe::new");
    fp.add("manual_probe", probe.clone() as ObjectRef);
    RustWorld { printer, gcode, toolhead, configfile, probe }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("manual_probe fakes");
            Some(py)
        }
        None => {
            println!("skipping manual_probe oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
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
        unit: "manual_probe".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect()),
        PyValue::Host(h) => Value::Opaque { type_name: "host".to_string(), addr: format!("{h:#x}") },
    }
}

fn str_list(v: &[String]) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn probe_result_value(r: &ProbeResult) -> Value {
    Value::Tuple(vec![
        Value::float(r.bed_x), Value::float(r.bed_y), Value::float(r.bed_z),
        Value::float(r.test_x), Value::float(r.test_y), Value::float(r.test_z)])
}

fn py_setup_world(py: &mut pyoracle::Oracle) {
    py.exec("p, g, th, gm, cf, c, mp = py_setup()").expect("py_setup");
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_and_probe_result() {
    let Some(mut py) = oracle() else { return };
    check("Z_BOB_MINIMUM", py.eval("manual_probe.Z_BOB_MINIMUM"), Value::float(Z_BOB_MINIMUM));
    check("BISECT_MAX", py.eval("manual_probe.BISECT_MAX"), Value::float(BISECT_MAX));
    for (label, pos, off) in [
        ("default offsets", [1.0, 2.0, 3.0], [0.0, 0.0, 0.0]),
        ("nonzero offsets", [1.5, -2.25, 3.125], [0.25, 0.5, 1.0]),
        ("negative z offset", [-1.0, 0.0, 10.0], [1.0, 1.0, -2.5]),
    ] {
        let expr = format!(
            "manual_probe.create_probe_result({pos:?}, {off:?})");
        let expected = py.eval(&expr);
        check(label, expected, probe_result_value(&create_probe_result(&pos, off)));
    }
}

#[test]
fn lookup_z_endstop_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    // stepper_z present.
    py.exec("p0 = FakePrinter(); c1 = FakeConfig(p0, 'manual_probe', make_sections())").expect("setup1");
    let printer: Printer = Arc::new(FakePrinter::default());
    let config = FakeConfig::make(printer, "manual_probe", cartesian_sections());
    let name_expr = "manual_probe.lookup_z_endstop_config(c1).get_name()";
    check("stepper_z present", py.eval(name_expr),
        Value::str(&lookup_z_endstop_config(&config).expect("rust lookup").expect("rust section").get_name()));
    // No stepper_z, carriage z.
    py.exec("sec2 = make_sections(include_z=False); sec2['carriage carriage_x'] = {'axis': 'x'}; sec2['carriage carriage_z'] = {'axis': 'z'}; c2 = FakeConfig(p0, 'manual_probe', sec2)").expect("setup2");
    let sec2 = sections(&[
        ("printer", &[("kinematics", "cartesian")]),
        ("stepper_a", &[("position_endstop", "10.0")]),
        ("stepper_b", &[("position_endstop", "11.0")]),
        ("stepper_c", &[("position_endstop", "12.0")]),
        ("carriage carriage_x", &[("axis", "x")]),
        ("carriage carriage_z", &[("axis", "z")]),
    ]);
    let config2 = FakeConfig::make(Arc::new(FakePrinter::default()), "manual_probe", sec2);
    check("carriage z", py.eval("manual_probe.lookup_z_endstop_config(c2).get_name()"),
        Value::str(&lookup_z_endstop_config(&config2).expect("rust lookup").expect("rust section").get_name()));
    // No stepper_z, no z carriage (axis defaults to carriage name).
    py.exec("sec3 = make_sections(include_z=False); c3 = FakeConfig(p0, 'manual_probe', sec3)").expect("setup3");
    let sec3 = sections(&[
        ("printer", &[("kinematics", "cartesian")]),
        ("stepper_a", &[("position_endstop", "10.0")]),
        ("stepper_b", &[("position_endstop", "11.0")]),
        ("stepper_c", &[("position_endstop", "12.0")]),
    ]);
    let config3 = FakeConfig::make(Arc::new(FakePrinter::default()), "manual_probe", sec3);
    let py_none = py.eval("(manual_probe.lookup_z_endstop_config(c3) is None)");
    let rust_none = lookup_z_endstop_config(&config3).expect("rust lookup").is_none();
    check("no z config is None", py_none, Value::Bool(rust_none));
}

#[test]
fn init_registers_commands_and_status() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    let w = rust_world(0.0);
    check("registered commands", py.eval("sorted(g.handlers.keys())"),
        str_list(&w.gcode.commands()));
    check("help texts", py.eval("sorted(g.help.items())"),
        Value::List(w.gcode.get_command_help().iter().map(|(k, v)| {
            Value::Tuple(vec![Value::str(k), Value::str(v)])
        }).collect()));
    check("initial status", py.eval("mp.get_status(0.0)"),
        pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
    check("z_position_endstop", py.eval("mp.z_position_endstop"), Value::float(w.probe.z_position_endstop().expect("z endstop")));
    check("z_endstop_config_name", py.eval("mp.z_endstop_config_name"), Value::str(w.probe.z_endstop_config_name().expect("name")));
}

#[test]
fn manual_probe_helper_lifecycle() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    py.exec("helper = manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'MANUAL_PROBE', {}), mp.manual_probe_finalize)").expect("py helper");
    let w = rust_world(0.0);
    let cb: FinalizeCallback = Box::new({
        let p = w.probe.clone();
        move |res| p.manual_probe_finalize(res)
    });
    let _helper = ManualProbeHelper::new(&w.printer, &gcmd("MANUAL_PROBE", &[]), cb).expect("rust helper");
    check("starting log", py.eval("g.log"), str_list(&w.gcode.log.lock().unwrap()));
    check("active status", py.eval("mp.status"), pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
    check("helper commands", py.eval("sorted(g.handlers.keys())"), str_list(&w.gcode.commands()));

    for z in ["+", "++", "-", "--", "1.5", "-0.25"] {
        let label = format!("TESTZ Z={z}");
        py.exec(&format!("helper.cmd_TESTZ(FakeGcmd(g, 'TESTZ', {{'Z': '{z}'}}))")).expect("py TESTZ");
        let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGcmd::new("TESTZ", &[("Z", z)]));
        // Drive the rust helper through its registered handler.
        w.gcode.dispatch("TESTZ", &[("Z", z)]).expect("rust TESTZ");
        let _ = gcmd;
        check(&format!("{label} log"), py.eval("g.log"), str_list(&w.gcode.log.lock().unwrap()));
        check(&format!("{label} status"), py.eval("mp.status"), pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
        check(&format!("{label} moves"), py.eval("th.moves"),
            Value::List(w.toolhead.moves.lock().unwrap().iter().map(|(coord, speed)| {
                Value::Tuple(vec![
                    Value::List(coord.iter().map(|c| match c { Some(f) => Value::float(*f), None => Value::Null }).collect()),
                    Value::float(*speed)])
            }).collect()));
    }
    // ACCEPT succeeds: nozzle below start, same XY.
    py.exec("helper.cmd_ACCEPT(FakeGcmd(g, 'ACCEPT', {}))").expect("py ACCEPT");
    w.gcode.dispatch("ACCEPT", &[]).expect("rust ACCEPT");
    check("accept log", py.eval("g.log"), str_list(&w.gcode.log.lock().unwrap()));
    check("accept status reset", py.eval("mp.status"), pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
    check("accept unregisters", py.eval("sorted(g.handlers.keys())"), str_list(&w.gcode.commands()));
}

#[test]
fn manual_probe_abort_and_failed_accept() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    py.exec("helper2 = manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'MANUAL_PROBE', {}), mp.manual_probe_finalize)").expect("py helper2");
    let w = rust_world(0.0);
    let cb: FinalizeCallback = Box::new({
        let p = w.probe.clone();
        move |res| p.manual_probe_finalize(res)
    });
    let _helper = ManualProbeHelper::new(&w.printer, &gcmd("MANUAL_PROBE", &[]), cb).expect("rust helper");
    // Move XY away, then ACCEPT must fail.
    py.exec("th.position = [11.0, 10.0, 9.0, 0.0]").expect("py move xy");
    w.toolhead.set_position(vec![11.0, 10.0, 9.0, 0.0]);
    py.exec("helper2.cmd_ACCEPT(FakeGcmd(g, 'ACCEPT', {}))").expect("py accept fail");
    w.gcode.dispatch("ACCEPT", &[]).expect("rust accept fail");
    check("failed accept log", py.eval("g.log"), str_list(&w.gcode.log.lock().unwrap()));
    check("failed accept status", py.eval("mp.status"), pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
    // Fresh pair for ABORT.
    py.exec("p, g, th, gm, cf, c, mp = py_setup()").expect("py setup2");
    py.exec("helper3 = manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'MANUAL_PROBE', {}), mp.manual_probe_finalize)").expect("py helper3");
    let w2 = rust_world(0.0);
    let cb2: FinalizeCallback = Box::new({
        let p = w2.probe.clone();
        move |res| p.manual_probe_finalize(res)
    });
    let _h2 = ManualProbeHelper::new(&w2.printer, &gcmd("MANUAL_PROBE", &[]), cb2).expect("rust helper2");
    py.exec("helper3.cmd_ABORT(FakeGcmd(g, 'ABORT', {}))").expect("py abort");
    w2.gcode.dispatch("ABORT", &[]).expect("rust abort");
    check("abort log", py.eval("g.log"), str_list(&w2.gcode.log.lock().unwrap()));
    check("abort status", py.eval("mp.status"), pyvalue_to_value(&w2.probe.get_status(0.0).expect("status")));
    check("abort unregisters", py.eval("sorted(g.handlers.keys())"), str_list(&w2.gcode.commands()));
}

#[test]
fn verify_no_manual_probe_already_active() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    py.exec("g.register_command('ACCEPT', lambda gcmd: None)").expect("py register accept");
    let w = rust_world(0.0);
    w.gcode.register_command("ACCEPT", Some(Box::new(|_g| Ok(()) )), RegisterCommandOpts::default()).expect("rust register accept");
    let expected = py.eval("manual_probe.verify_no_manual_probe(p)");
    check_err("verify already active", expected, verify_no_manual_probe(&w.printer));
    // And it raises when starting a second helper.
    let expected = py.eval("manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'MANUAL_PROBE', {}), mp.manual_probe_finalize)");
    let cb: FinalizeCallback = Box::new(|_res| Ok(()));
    let actual = ManualProbeHelper::new(&w.printer, &gcmd("MANUAL_PROBE", &[]), cb).map(|_h| ());
    check_err("helper while active", expected, actual);
}

#[test]
fn z_endstop_calibrate_and_offset_apply() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    // Z_ENDSTOP_CALIBRATE: helper at z=10, move down, accept -> writes config.
    let w = rust_world(0.0);
    let cb: FinalizeCallback = Box::new({
        let p = w.probe.clone();
        move |res| p.z_endstop_finalize(res)
    });
    let _h = ManualProbeHelper::new(&w.printer, &gcmd("Z_ENDSTOP_CALIBRATE", &[]), cb).expect("rust calibrate");
    py.exec("h = manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'Z_ENDSTOP_CALIBRATE', {}), mp.z_endstop_finalize)").expect("py calibrate2");
    // Drive: TESTZ to z=8 via explicit float, then ACCEPT.
    py.exec("h.cmd_TESTZ(FakeGcmd(g, 'TESTZ', {'Z': '-2.0'}))").expect("py testz");
    w.gcode.dispatch("TESTZ", &[("Z", "-2.0")]).expect("rust testz");
    py.exec("h.cmd_ACCEPT(FakeGcmd(g, 'ACCEPT', {}))").expect("py accept");
    w.gcode.dispatch("ACCEPT", &[]).expect("rust accept");
    check("calibrate log", py.eval("g.log"), str_list(&w.gcode.log.lock().unwrap()));
    check("calibrate writes", py.eval("cf.writes"),
        Value::List(w.configfile.writes.lock().unwrap().iter().map(|(s, o, v)| {
            Value::Tuple(vec![Value::str(s), Value::str(o), Value::str(v)])
        }).collect()));
    // Z_OFFSET_APPLY_ENDSTOP with zero offset.
    py.exec("mp.cmd_Z_OFFSET_APPLY_ENDSTOP(FakeGcmd(g, 'Z_OFFSET_APPLY_ENDSTOP', {}))").expect("py offset0");
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGcmd::new("Z_OFFSET_APPLY_ENDSTOP", &[]));
    let actual = w.probe.cmd_z_offset_apply_endstop(&gcmd).map(|_| ());
    check("offset zero log", Ok(py.eval("g.log").expect("py log")), str_list(&w.gcode.log.lock().unwrap()));
    let _ = actual;
    // Nonzero offset.
    py.exec("gm.offset = 0.5").expect("py offset set");
    py.exec("mp.cmd_Z_OFFSET_APPLY_ENDSTOP(FakeGcmd(g, 'Z_OFFSET_APPLY_ENDSTOP', {}))").expect("py offset");
    let w2 = rust_world(0.5);
    let gcmd2: klippy_api::gcode::GCmd = Arc::new(FakeGcmd::new("Z_OFFSET_APPLY_ENDSTOP", &[]));
    w2.probe.cmd_z_offset_apply_endstop(&gcmd2).expect("rust offset");
    // Compare against a fresh python world for the offset message.
    py.exec("p, g, th, gm, cf, c, mp = py_setup(offset=0.5)").expect("py setup offset");
    py.exec("mp.cmd_Z_OFFSET_APPLY_ENDSTOP(FakeGcmd(g, 'Z_OFFSET_APPLY_ENDSTOP', {}))").expect("py offset fresh");
    check("offset log", py.eval("g.log"), str_list(&w2.gcode.log.lock().unwrap()));
    check("offset writes", py.eval("cf.writes"),
        Value::List(w2.configfile.writes.lock().unwrap().iter().map(|(s, o, v)| {
            Value::Tuple(vec![Value::str(s), Value::str(o), Value::str(v)])
        }).collect()));
}

#[test]
fn move_z_error_finalizes() {
    let Some(mut py) = oracle() else { return };
    py_setup_world(&mut py);
    py.exec("th.fail_move = True").expect("py fail");
    py.exec("h = manual_probe.ManualProbeHelper(p, FakeGcmd(g, 'MANUAL_PROBE', {}), mp.manual_probe_finalize)").expect("py helper");
    let w = rust_world(0.0);
    *w.toolhead.fail_move.lock().unwrap() = true;
    let cb: FinalizeCallback = Box::new({
        let p = w.probe.clone();
        move |res| p.manual_probe_finalize(res)
    });
    let helper = ManualProbeHelper::new(&w.printer, &gcmd("MANUAL_PROBE", &[]), cb).expect("rust helper");
    let expected = py.eval("h.cmd_TESTZ(FakeGcmd(g, 'TESTZ', {'Z': '1.0'}))");
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGcmd::new("TESTZ", &[("Z", "1.0")]));
    check_err("move_z error propagates", expected, helper.cmd_testz(&gcmd).map(|_| ()));
    check("move_z error status reset", py.eval("mp.status"), pyvalue_to_value(&w.probe.get_status(0.0).expect("status")));
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    // Fresh printer without a manual_probe object yet (a second registration
    // of MANUAL_PROBE on the same printer would be a config error).
    let fp = FakePrinter::default();
    let printer: Printer = Arc::new(fp.clone());
    let gcode = FakeGCode::new();
    let toolhead = Arc::new(FakeToolhead {
        position: Arc::new(GCell::new(vec![10.0, 10.0, 10.0, 0.0])),
        moves: Mutex::new(Vec::new()),
        fail_move: Mutex::new(false),
        flushes: Mutex::new(0),
    });
    let configfile = Arc::new(FakeConfigFile { writes: Mutex::new(Vec::new()) });
    let gmov = Arc::new(FakeGCodeMove { offset: 0.0 });
    fp.add("gcode", gcode.clone() as ObjectRef);
    fp.add("toolhead", toolhead as ObjectRef);
    fp.add("gcode_move", gmov as ObjectRef);
    fp.add("configfile", configfile as ObjectRef);
    let config = FakeConfig::make(printer, "manual_probe", cartesian_sections());
    let obj = load_config(&config).expect("factory load_config").expect("object");
    let rust: Arc<ManualProbe> = downcast_arc(&obj).expect("downcast");
    py_setup_world(&mut py);
    check("factory status", py.eval("mp.get_status(0.0)"),
        pyvalue_to_value(&rust.get_status(0.0).expect("status")));
}
