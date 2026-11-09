//! Parity tests for `extras::axis_twist_compensation`
//! (port of `klippy/extras/axis_twist_compensation.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the real
//! `extras.axis_twist_compensation`, `extras.manual_probe` and
//! `extras.probe.run_single_probe` run on minimal Python fakes for the
//! printer (with real event dispatch), gcode dispatch, toolhead, configfile and
//! the probe object. The Rust port is driven through fakes of the same contract
//! traits with identical inputs, and log lines, toolhead moves, stored
//! configuration values, the compensation state, registered commands and events, probe
//! session counters and raised errors (class and message) are compared after
//! every step.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit, so
//! replay fixtures do not apply; expected values come from the oracle alone.
//! Configuration writes pass through the real Python `PrinterConfig.set`
//! and `ConfigAutoSave.set`, including their conversion of numeric values to strings.
//!
//! The `probe` owner is `crate::probe`: `PrinterProbeApi` (`get_probe_params`,
//! `get_offsets`, `start_probe_session`), `ProbeSessionApi` (`run_probe`,
//! `pull_probed_results`, `end_probe_session`), `ProbeParams`,
//! `run_single_probe(&Arc<dyn PrinterProbeApi>, &GCmd)` and `ProbeResultList`.

use extras::axis_twist_compensation::{module_factory, AxisTwistCompensation};
use extras::manual_probe::{ManualProbe, ProbeResult};
use extras::probe::{PrinterProbeApi, ProbeParams, ProbeResultList, ProbeSessionApi};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi,
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GetParser,
    RegisterCommandOpts,
};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT, EV_PROBE_UPDATE_RESULTS,
};
use klippy_api::stepper::{McuStepperApi, Stepper, StepperFactory, Trapq, TrapqApi};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateHandle, KinematicsApi, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

/// `kos::sync::Mutex` behind the `std` lock signature (std locks are
/// forbidden by the platform rules).
struct Mutex<T>(kos::sync::Mutex<T>);

impl<T: Default> Default for Mutex<T> {
    fn default() -> Self {
        Mutex::new(T::default())
    }
}

impl<T> Mutex<T> {
    fn new(value: T) -> Self {
        Mutex(kos::sync::Mutex::new(value))
    }

    fn lock(&self) -> Result<kos::sync::MutexGuard<'_, T>, std::convert::Infallible> {
        Ok(self.0.lock())
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------
const SETUP: &str = r#"
import configparser
import configfile
import extras.manual_probe as manual_probe
import extras.axis_twist_compensation as atc

class CommandError(Exception):
    pass

_sentinel = object()

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
    def get_command(self):
        return self._command
    def respond_info(self, msg, log=True):
        self._gcode.log.append(msg)
    def get(self, name, default=_sentinel, parser=str):
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
    def get_int(self, name, default=_sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int)
    def get_float(self, name, default=_sentinel, **kw):
        return self.get(name, default, parser=float)

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
    def dispatch(self, cmd, params):
        handler = self.handlers.get(cmd)
        if handler is None:
            raise CommandError('unknown command %s' % (cmd,))
        handler(FakeGcmd(self, cmd, params))

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.events = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unable to find object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_sentinel):
        return self.lookup_object(section, default)
    def register_event_handler(self, event, callback):
        self.events.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.events.get(event, [])]

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
    def get_position(self):
        return list(self.position)
    def flush_step_generation(self):
        pass
    def get_kinematics(self):
        return self.kin
    def manual_move(self, coord, speed):
        if self.fail_move:
            raise CommandError('Move out of range')
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

class FakeGCodeMove:
    pass

class FakeConfigFile(configfile.PrinterConfig):
    def __init__(self):
        # Exercise the reference's set path without SAVE_CONFIG registration.
        self.autosave = configfile.ConfigAutoSave.__new__(configfile.ConfigAutoSave)
        self.autosave.fileconfig = configparser.RawConfigParser()
        self.autosave.status_save_pending = {}
        self.autosave.save_config_pending = False
        self.writes = []
    def set(self, section, option, value):
        super().set(section, option, value)
        self.writes.append((section, option,
                            self.autosave.fileconfig.get(section, option)))

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
    def get_prefix_sections(self, prefix):
        return [FakeConfig(self._printer, name, self.sections)
                for name in self.sections if name.startswith(prefix)]
    def getsection(self, section):
        return FakeConfig(self._printer, section, self.sections)
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
    def getlists(self, option, default=_sentinel, seps=(',',), count=None,
                 parser=str, note_valid=True):
        v = self._opt(option, default)
        if isinstance(v, str):
            parts = [p.strip() for p in v.split(',')] if v.strip() else []
            return tuple([parser(p) for p in parts])
        return v

class FakeSession:
    def __init__(self, probe):
        self.probe = probe
    def run_probe(self, gcmd):
        z = self.probe.zs.pop(0)
        pos = self.probe.toolhead.position
        ox, oy, oz = self.probe.offsets
        self.result = manual_probe.ProbeResult(pos[0] + ox, pos[1] + oy, z,
                                               pos[0], pos[1], pos[2])
    def pull_probed_results(self):
        return [self.result]
    def end_probe_session(self):
        self.probe.ended += 1

class FakeProbe:
    def __init__(self, toolhead, offsets, zs, lift_speed):
        self.toolhead = toolhead
        self.offsets = offsets
        self.zs = zs
        self.lift_speed = lift_speed
        self.started = 0
        self.ended = 0
    def get_offsets(self, gcmd=None):
        return self.offsets
    def get_probe_params(self, gcmd=None):
        return {'probe_speed': 5.0, 'lift_speed': self.lift_speed, 'samples': 1,
                'sample_retract_dist': 2.0, 'samples_tolerance': 0.1,
                'samples_tolerance_retries': 0, 'samples_result': 'average'}
    def start_probe_session(self, gcmd):
        self.started += 1
        return FakeSession(self)

def make_sections(opts):
    return {
        'printer': {'kinematics': 'cartesian'},
        'stepper_a': {'position_endstop': '10.0'},
        'stepper_b': {'position_endstop': '11.0'},
        'stepper_c': {'position_endstop': '12.0'},
        'stepper_z': {'position_endstop': '5.0'},
        'axis_twist_compensation': dict(opts),
    }

def py_world(opts, with_probe=True, connect=True, zs=(), offsets=(0.0, 0.0, 0.0),
             lift_speed=7.0):
    p = FakePrinter()
    g = FakeGCode()
    th = FakeToolhead([10.0, 10.0, 10.0, 0.0])
    cf = FakeConfigFile()
    p.add_object('gcode', g)
    p.add_object('toolhead', th)
    p.add_object('gcode_move', FakeGCodeMove())
    p.add_object('configfile', cf)
    sections = make_sections(opts)
    p.add_object('manual_probe',
                 manual_probe.load_config(FakeConfig(p, 'manual_probe', sections)))
    probe = None
    if with_probe:
        probe = FakeProbe(th, offsets, list(zs), lift_speed)
        p.add_object('probe', probe)
    a = atc.load_config(FakeConfig(p, 'axis_twist_compensation', sections))
    p.add_object('axis_twist_compensation', a)
    if connect:
        p.send_event('klippy:connect')
    return p, g, th, cf, probe, a
"#;

const PY_STATE: &str = "[list(a.z_compensations), a.compensation_start_x, a.compensation_end_x, \
                        list(a.zy_compensations), a.compensation_start_y, a.compensation_end_y]";

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Sections = Arc<Mutex<IndexMap<String, IndexMap<String, String>>>>;

fn rust_sections(opts: &[(&str, &str)]) -> Sections {
    let mut map: IndexMap<String, IndexMap<String, String>> = IndexMap::new();
    let mut put = |name: &str, entries: &[(&str, &str)]| {
        let mut inner = IndexMap::new();
        for (k, v) in entries {
            inner.insert(k.to_string(), v.to_string());
        }
        map.insert(name.to_string(), inner);
    };
    put("printer", &[("kinematics", "cartesian")]);
    put("stepper_a", &[("position_endstop", "10.0")]);
    put("stepper_b", &[("position_endstop", "11.0")]);
    put("stepper_c", &[("position_endstop", "12.0")]);
    put("stepper_z", &[("position_endstop", "5.0")]);
    put("axis_twist_compensation", opts);
    Arc::new(Mutex::new(map))
}

#[derive(Clone, Default)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
    events: Arc<Mutex<IndexMap<String, Vec<EventCallback>>>>,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }

    fn event_counts(&self) -> Vec<(String, usize)> {
        self.events.lock().unwrap().iter().map(|(k, v)| (k.clone(), v.len())).collect()
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
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().unwrap().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::config(format!("Unable to find object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.lock().unwrap().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.events.lock().unwrap().entry(event.to_string()).or_default().push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut events = self.events.lock().unwrap();
        let mut out = Vec::new();
        if let Some(handlers) = events.get_mut(event) {
            for handler in handlers.iter_mut() {
                out.push(handler(params)?);
            }
        }
        Ok(out)
    }
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
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::config(format!("No option '{option}' in section '{}'", self.name))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opt(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opt(option) {
            Some(v) => v.parse::<f64>().map(Some).map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            }),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match self.opt(option) {
            Some(v) => {
                let parts: Vec<&str> =
                    if v.trim().is_empty() { Vec::new() } else { v.split(',').map(str::trim).collect() };
                let mut items = Vec::new();
                for part in parts {
                    items.push(PyValue::Float(part.parse::<f64>().map_err(|_| {
                        KlippyError::config(format!(
                            "Unable to parse option '{option}' in section '{}'",
                            self.name
                        ))
                    })?));
                }
                Ok(PyValue::Tuple(items))
            }
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d.unwrap_or(PyValue::None)),
            },
        }
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::make(self.printer.clone(), section, self.sections.clone())
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.lock().unwrap().contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeGCode {
    handlers: Mutex<IndexMap<String, Arc<Mutex<GCodeCommandHandler>>>>,
    help: Mutex<IndexMap<String, String>>,
    log: Arc<Mutex<Vec<String>>>,
}

impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode {
            handlers: Mutex::new(IndexMap::new()),
            help: Mutex::new(IndexMap::new()),
            log: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn commands(&self) -> Vec<String> {
        let mut v: Vec<String> = self.handlers.lock().unwrap().keys().cloned().collect();
        v.sort();
        v
    }

    /// Runs the registered handler like `FakeGCode.dispatch` in the Python
    /// fakes: the handler stays registered unless it unregisters itself, and it
    /// may register a new handler under the same name while it runs.
    fn dispatch(&self, cmd: &str, params: &[(&str, &str)]) -> KResult<()> {
        let handler = self.handlers.lock().unwrap().get(cmd).cloned();
        let Some(handler) = handler else {
            return Err(KlippyError::command(format!("unknown command {cmd}")));
        };
        let gcmd: GCmd = Arc::new(FakeGcmd::new(cmd, params, self.log.clone()));
        let mut func = handler.lock().unwrap();
        (*func)(&gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        match func {
            None => {
                // The Python fakes return the popped function; no caller of the
                // port looks at it, and a running handler still owns its entry.
                self.handlers.lock().unwrap().shift_remove(cmd);
                Ok(None)
            }
            Some(f) => {
                let mut handlers = self.handlers.lock().unwrap();
                if handlers.contains_key(cmd) {
                    return Err(KlippyError::config(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                if let Some(d) = opts.desc {
                    self.help.lock().unwrap().insert(cmd.to_string(), d);
                }
                handlers.insert(cmd.to_string(), Arc::new(Mutex::new(f)));
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        _c: &str,
        _k: &str,
        _v: Option<&str>,
        _f: GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        self.help.lock().unwrap().clone()
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
        command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd::new(command, &[], self.log.clone()))
    }
    fn respond_raw(&self, msg: &str) {
        self.log.lock().unwrap().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.lock().unwrap().push(msg.to_string());
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

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    log: Arc<Mutex<Vec<String>>>,
}

impl FakeGcmd {
    fn new(command: &str, params: &[(&str, &str)], log: Arc<Mutex<Vec<String>>>) -> Self {
        let mut sorted: Vec<(&str, &str)> = params.to_vec();
        sorted.sort();
        let mut commandline = command.to_string();
        let mut map = IndexMap::new();
        for (k, v) in &sorted {
            commandline.push_str(&format!(" {k}={v}"));
            map.insert(k.to_string(), v.to_string());
        }
        FakeGcmd { command: command.to_string(), commandline, params: map, log }
    }

    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))
    }

    fn unparsable(&self, value: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, value))
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(value) => match opts.parser {
                GetParser::Str => Ok(PyValue::Str(value.clone())),
                GetParser::Float => value
                    .parse::<f64>()
                    .map(PyValue::Float)
                    .map_err(|_| self.unparsable(value)),
                GetParser::Int | GetParser::IntAutoBase => value
                    .parse::<i64>()
                    .map(|n| PyValue::Int(pycompat::PyInt::from_i64(n)))
                    .map_err(|_| self.unparsable(value)),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(value) => value.parse::<i64>().map(Some).map_err(|_| self.unparsable(value)),
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(self.missing(name)),
                Some(d) => Ok(d),
            },
            Some(value) => value.parse::<f64>().map(Some).map_err(|_| self.unparsable(value)),
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.lock().unwrap().push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.log.lock().unwrap().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeConfigFile {
    writes: Mutex<Vec<(String, String, String)>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used")
    }
    fn read_config(&self, _f: &str) -> KResult<Config> {
        panic!("not used")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("not used")
    }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _m: &str) {}
    fn deprecate(&self, _s: &str, _o: &str, _v: Option<PyValue>, _m: Option<String>) {}
    fn deprecate_gcode(&self, _c: &str, _o: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _m: &klippy_api::mcu::Mcu, _f: &str, _m2: Option<String>) {}
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.writes.lock().unwrap().push((
            section.to_string(),
            option.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn remove_section(&self, _s: &str) {}
}

impl PrinterObject for FakeConfigFile {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

struct FakeGCodeMove;

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set_move_transform(
        &self,
        _t: klippy_api::gcode::MoveTransform,
        _f: bool,
    ) -> KResult<Option<klippy_api::gcode::MoveTransform>> {
        Ok(None)
    }
    fn reset_last_position(&self) -> KResult<()> { Ok(()) }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeTrapq;
impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct FakeStepper {
    name: String,
}
impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("not used")
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
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
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
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _p: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _m: i64) -> f64 {
        0.0
    }
    fn dump_steps(
        &self,
        _c: i64,
        _s: i64,
        _e: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<klippy_api::stepper::StepperKinematics>,
    ) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, _t: Option<Trapq>) -> Option<Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _a: char) -> bool {
        false
    }
}

struct FakeKinematics {
    position: Arc<GCell<Vec<f64>>>,
}
impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        ["stepper_x", "stepper_y", "stepper_z"]
            .iter()
            .map(|n| Arc::new(FakeStepper { name: n.to_string() }) as Stepper)
            .collect()
    }
    fn calc_position(&self, _spos: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(self.position.borrow().iter().map(|v| Some(*v)).collect())
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _h: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _c: &str) {}
}

struct FakeToolhead {
    position: Arc<GCell<Vec<f64>>>,
    moves: Mutex<Vec<(Vec<Option<f64>>, f64)>>,
    fail_move: Mutex<bool>,
}

impl FakeToolhead {
    fn set_position(&self, pos: Vec<f64>) {
        *self.position.borrow_mut() = pos;
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.borrow().clone()
    }
    fn set_position(&self, newpos: Vec<f64>, _h: &str) -> KResult<()> {
        self.set_position(newpos);
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        Arc::new(FakeKinematics { position: self.position.clone() })
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &reactor::Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        if *self.fail_move.lock().unwrap() {
            return Err(KlippyError::command("Move out of range"));
        }
        self.moves.lock().unwrap().push((coord.clone(), speed));
        let mut pos = self.position.borrow().clone();
        for (i, v) in coord.iter().enumerate() {
            if let Some(v) = v {
                pos[i] = *v;
            }
        }
        self.set_position(pos);
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _v: Option<f64>,
        _a: Option<f64>,
        _s: Option<f64>,
        _m: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(
        &self,
        _c: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("not used")
    }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

/// Shared state of the fake probe and its sessions (`FakeProbe` in the Python
/// fakes).
struct ProbeState {
    toolhead: Arc<FakeToolhead>,
    offsets: [f64; 3],
    zs: Mutex<Vec<f64>>,
    lift_speed: f64,
    started: Mutex<i64>,
    ended: Mutex<i64>,
}

struct FakeProbe {
    state: Arc<ProbeState>,
}

struct FakeSession {
    state: Arc<ProbeState>,
    result: Mutex<Option<ProbeResult>>,
}

impl ProbeSessionApi for FakeSession {
    fn run_probe(&self, _gcmd: &GCmd) -> KResult<()> {
        let z = self.state.zs.lock().unwrap().remove(0);
        let pos = self.state.toolhead.position.borrow().clone();
        let [ox, oy, _oz] = self.state.offsets;
        *self.result.lock().unwrap() = Some(ProbeResult {
            bed_x: pos[0] + ox,
            bed_y: pos[1] + oy,
            bed_z: z,
            test_x: pos[0],
            test_y: pos[1],
            test_z: pos[2],
        });
        Ok(())
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.result.lock().unwrap().iter().copied().collect())
    }
    fn end_probe_session(&self) -> KResult<()> {
        *self.state.ended.lock().unwrap() += 1;
        Ok(())
    }
}

impl PrinterProbeApi for FakeProbe {
    fn get_probe_params(&self, _gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        Ok(ProbeParams {
            probe_speed: 5.0,
            lift_speed: self.state.lift_speed,
            samples: 1,
            sample_retract_dist: 2.0,
            samples_tolerance: 0.1,
            samples_tolerance_retries: 0,
            samples_result: "average".to_string(),
        })
    }
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        let [x, y, z] = self.state.offsets;
        Ok((x, y, z))
    }
    fn start_probe_session(&self, _gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        *self.state.started.lock().unwrap() += 1;
        Ok(Arc::new(FakeSession { state: self.state.clone(), result: Mutex::new(None) }))
    }
}

klippy_api::printer_object!(FakeProbe; dyn PrinterProbeApi);

// ---------------------------------------------------------------------------
// Worlds: the same scenario built on both sides.
// ---------------------------------------------------------------------------

struct Spec<'a> {
    opts: &'a [(&'a str, &'a str)],
    with_probe: bool,
    connect: bool,
    zs: Vec<f64>,
    offsets: [f64; 3],
    lift_speed: f64,
}

impl Default for Spec<'_> {
    fn default() -> Self {
        Spec {
            opts: &[],
            with_probe: true,
            connect: true,
            zs: Vec::new(),
            offsets: [0.0, 0.0, 0.0],
            lift_speed: 7.0,
        }
    }
}

struct World {
    fp: FakePrinter,
    printer: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    configfile: Arc<FakeConfigFile>,
    probe: Option<Arc<ProbeState>>,
    atc: Arc<AxisTwistCompensation>,
}

fn pyf(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x == f64::INFINITY {
        "float('inf')".to_string()
    } else if x == f64::NEG_INFINITY {
        "float('-inf')".to_string()
    } else {
        format!("{x:?}")
    }
}

fn pyb(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn pydict(entries: &[(&str, &str)]) -> String {
    let items: Vec<String> = entries.iter().map(|(k, v)| format!("'{k}': '{v}'")).collect();
    format!("{{{}}}", items.join(", "))
}

struct Parts {
    fp: FakePrinter,
    printer: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    configfile: Arc<FakeConfigFile>,
    probe: Option<Arc<ProbeState>>,
    atc_config: Config,
}

fn rust_world_parts(spec: &Spec) -> Parts {
    let fp = FakePrinter::default();
    let printer: Printer = Arc::new(fp.clone());
    let gcode = FakeGCode::new();
    let toolhead = Arc::new(FakeToolhead {
        position: Arc::new(GCell::new(vec![10.0, 10.0, 10.0, 0.0])),
        moves: Mutex::new(Vec::new()),
        fail_move: Mutex::new(false),
    });
    let configfile = Arc::new(FakeConfigFile { writes: Mutex::new(Vec::new()) });
    fp.add("gcode", gcode.clone() as ObjectRef);
    fp.add("toolhead", toolhead.clone() as ObjectRef);
    fp.add("gcode_move", Arc::new(FakeGCodeMove) as ObjectRef);
    fp.add("configfile", configfile.clone() as ObjectRef);
    let sections = rust_sections(spec.opts);
    let mp_config = FakeConfig::make(printer.clone(), "manual_probe", sections.clone());
    let manual_probe = ManualProbe::new(&mp_config).expect("rust ManualProbe::new");
    fp.add("manual_probe", manual_probe as ObjectRef);
    let probe = if spec.with_probe {
        let state = Arc::new(ProbeState {
            toolhead: toolhead.clone(),
            offsets: spec.offsets,
            zs: Mutex::new(spec.zs.clone()),
            lift_speed: spec.lift_speed,
            started: Mutex::new(0),
            ended: Mutex::new(0),
        });
        fp.add("probe", Arc::new(FakeProbe { state: state.clone() }) as ObjectRef);
        Some(state)
    } else {
        None
    };
    let atc_config = FakeConfig::make(printer.clone(), "axis_twist_compensation", sections);
    Parts { fp, printer, gcode, toolhead, configfile, probe, atc_config }
}

fn rust_world(spec: &Spec) -> World {
    let Parts { fp, printer, gcode, toolhead, configfile, probe, atc_config } =
        rust_world_parts(spec);
    let atc = AxisTwistCompensation::new(&atc_config).expect("rust AxisTwistCompensation::new");
    fp.add("axis_twist_compensation", atc.clone() as ObjectRef);
    if spec.connect {
        printer.send_event(EV_KLIPPY_CONNECT, &[]).expect("rust klippy:connect");
    }
    World { fp, printer, gcode, toolhead, configfile, probe, atc }
}

/// Builds the scenario on the oracle and in Rust.
fn both(py: &mut pyoracle::Oracle, spec: &Spec) -> World {
    let zs: Vec<String> = spec.zs.iter().map(|z| pyf(*z)).collect();
    let code = format!(
        "p, g, th, cf, probe, a = py_world({}, with_probe={}, connect={}, zs=[{}], offsets=({}, {}, {}), lift_speed={})",
        pydict(spec.opts),
        pyb(spec.with_probe),
        pyb(spec.connect),
        zs.join(", "),
        pyf(spec.offsets[0]),
        pyf(spec.offsets[1]),
        pyf(spec.offsets[2]),
        pyf(spec.lift_speed),
    );
    py.exec(&code).expect("py_world");
    rust_world(spec)
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("axis_twist_compensation fakes");
            Some(py)
        }
        None => {
            println!("skipping axis_twist_compensation oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
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
        unit: "axis_twist_compensation".to_string(),
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

/// Both sides must fail with the same exception class and message, or both
/// succeed.
fn same_outcome(label: &str, expected: &pyoracle::OracleResult, actual: &KResult<()>) {
    match (expected, actual) {
        (Ok(_), Ok(())) => {}
        (Err(e), Err(r)) => {
            assert_eq!(r.class_name(), e.class_name.as_str(), "{label}: exception class");
            assert_eq!(r.message(), e.message.as_str(), "{label}: exception message");
        }
        (Ok(v), Err(r)) => panic!("{label}: oracle returned {v:?} but rust raised {r:?}"),
        (Err(e), Ok(())) => {
            panic!("{label}: oracle raised {}: {} but rust returned Ok", e.class_name, e.message)
        }
    }
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
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect(),
        ),
        PyValue::Host(h) => Value::Opaque { type_name: "host".to_string(), addr: format!("{h:#x}") },
    }
}

fn str_list(v: &[String]) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn fv(v: Option<f64>) -> Value {
    match v {
        Some(f) => Value::float(f),
        None => Value::Null,
    }
}

fn fl(v: &[f64]) -> Value {
    Value::List(v.iter().map(|f| Value::float(*f)).collect())
}

fn state_value(a: &AxisTwistCompensation) -> Value {
    let s = a.state.cloned();
    Value::List(vec![
        fl(&s.z_compensations),
        fv(s.compensation_start_x),
        fv(s.compensation_end_x),
        fl(&s.zy_compensations),
        fv(s.compensation_start_y),
        fv(s.compensation_end_y),
    ])
}

fn probe_result_value(r: &ProbeResult) -> Value {
    Value::Tuple(vec![
        Value::float(r.bed_x),
        Value::float(r.bed_y),
        Value::float(r.bed_z),
        Value::float(r.test_x),
        Value::float(r.test_y),
        Value::float(r.test_z),
    ])
}

/// Compares everything observable after a step.
fn compare_world(py: &mut pyoracle::Oracle, w: &World, label: &str) {
    check(
        &format!("{label}: log"),
        py.eval("g.log"),
        str_list(&w.gcode.log.lock().unwrap()),
    );
    check(
        &format!("{label}: moves"),
        py.eval("th.moves"),
        Value::List(
            w.toolhead
                .moves
                .lock()
                .unwrap()
                .iter()
                .map(|(coord, speed)| {
                    Value::Tuple(vec![
                        Value::List(
                            coord
                                .iter()
                                .map(|c| match c {
                                    Some(f) => Value::float(*f),
                                    None => Value::Null,
                                })
                                .collect(),
                        ),
                        Value::float(*speed),
                    ])
                })
                .collect(),
        ),
    );
    check(&format!("{label}: position"), py.eval("th.position"), fl(&w.toolhead.position.borrow()));
    check(
        &format!("{label}: commands"),
        py.eval("sorted(g.handlers.keys())"),
        str_list(&w.gcode.commands()),
    );
    check(
        &format!("{label}: configfile writes"),
        py.eval("cf.writes"),
        Value::List(
            w.configfile
                .writes
                .lock()
                .unwrap()
                .iter()
                .map(|(s, o, v)| Value::Tuple(vec![Value::str(s), Value::str(o), Value::str(v)]))
                .collect(),
        ),
    );
    check(&format!("{label}: compensation state"), py.eval(PY_STATE), state_value(&w.atc));
    if let Some(state) = &w.probe {
        check(
            &format!("{label}: probe counters"),
            py.eval("[probe.started, probe.ended, len(probe.zs)]"),
            Value::List(vec![
                Value::int(*state.started.lock().unwrap()),
                Value::int(*state.ended.lock().unwrap()),
                Value::int(state.zs.lock().unwrap().len() as i64),
            ]),
        );
    }
}

/// Dispatches one command on both sides and compares outcome and world.
fn step(
    py: &mut pyoracle::Oracle,
    w: &World,
    label: &str,
    cmd: &str,
    params: &[(&str, &str)],
) {
    let expected = py.exec(&format!("g.dispatch('{cmd}', {})", pydict(params)));
    let actual = w.gcode.dispatch(cmd, params);
    same_outcome(&format!("{label}: {cmd} {params:?}"), &expected, &actual);
    compare_world(py, w, &format!("{label} after {cmd} {params:?}"));
}

type ResultRow = [f64; 6];

fn py_results(rows: &[ResultRow]) -> String {
    let items: Vec<String> = rows
        .iter()
        .map(|r| {
            let args: Vec<String> = r.iter().map(|f| pyf(*f)).collect();
            format!("manual_probe.ProbeResult({})", args.join(", "))
        })
        .collect();
    format!("rl = [{}]", items.join(", "))
}

/// Sends `probe:update_results` with the same list on both sides.
fn update_results(py: &mut pyoracle::Oracle, w: &World, label: &str, rows: &[ResultRow]) {
    py.exec(&py_results(rows)).expect("py results");
    let expected = py.eval("p.send_event('probe:update_results', rl)");
    let list: Arc<ProbeResultList> = Arc::new(GCell::new(
        rows.iter()
            .map(|r| ProbeResult {
                bed_x: r[0],
                bed_y: r[1],
                bed_z: r[2],
                test_x: r[3],
                test_y: r[4],
                test_z: r[5],
            })
            .collect(),
    ));
    let actual = w.printer.send_event(EV_PROBE_UPDATE_RESULTS, &[EventArg::handle(list.clone())]);
    match actual {
        Ok(returned) => {
            check(
                &format!("{label}: handler returns"),
                expected,
                Value::List(returned.iter().map(pyvalue_to_value).collect()),
            );
            check(
                &format!("{label}: updated list"),
                py.eval("rl"),
                Value::List(list.borrow().iter().map(probe_result_value).collect()),
            );
        }
        Err(e) => same_outcome(label, &expected, &Err(e)),
    }
}

const FULL_OPTS: &[(&str, &str)] = &[
    ("horizontal_move_z", "6.5"),
    ("speed", "60"),
    ("calibrate_start_x", "20"),
    ("calibrate_end_x", "200"),
    ("calibrate_y", "100"),
    ("z_compensations", "0.1, 0.2, -0.05"),
    ("compensation_start_x", "20"),
    ("compensation_end_x", "200"),
    ("calibrate_start_y", "30"),
    ("calibrate_end_y", "180"),
    ("calibrate_x", "110"),
    ("compensation_start_y", "30"),
    ("compensation_end_y", "180"),
    ("zy_compensations", "0.03,-0.02"),
];

const X_OPTS: &[(&str, &str)] = &[
    ("horizontal_move_z", "6"),
    ("speed", "80"),
    ("calibrate_start_x", "20.3"),
    ("calibrate_end_x", "199.7"),
    ("calibrate_y", "100"),
    ("z_compensations", "0.5, 0.25, 0.125"),
    ("compensation_start_x", "1"),
    ("compensation_end_x", "2"),
    ("zy_compensations", "0.01, 0.02"),
    ("compensation_start_y", "5"),
    ("compensation_end_y", "15"),
];

const Y_OPTS: &[(&str, &str)] = &[
    ("horizontal_move_z", "4.5"),
    ("calibrate_x", "110.1"),
    ("calibrate_start_y", "30"),
    ("calibrate_end_y", "180.9"),
    ("z_compensations", "0.5, 0.25"),
    ("compensation_start_x", "1"),
    ("compensation_end_x", "2"),
    ("zy_compensations", "0.01, 0.02, 0.03"),
    ("compensation_start_y", "5"),
    ("compensation_end_y", "15"),
];

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_reads_every_option_and_registers() {
    let Some(mut py) = oracle() else { return };
    for (label, opts) in [("full config", FULL_OPTS), ("defaults", &[][..])] {
        let w = both(&mut py, &Spec { opts, ..Spec::default() });
        check(
            &format!("{label}: scalar options"),
            py.eval(
                "[a.horizontal_move_z, a.speed, a.calibrate_start_x, a.calibrate_end_x, a.calibrate_y, \
                 a.calibrate_start_y, a.calibrate_end_y, a.calibrate_x]",
            ),
            Value::List(vec![
                Value::float(w.atc.horizontal_move_z),
                Value::float(w.atc.speed),
                fv(w.atc.calibrate_start_x),
                fv(w.atc.calibrate_end_x),
                fv(w.atc.calibrate_y),
                fv(w.atc.calibrate_start_y),
                fv(w.atc.calibrate_end_y),
                fv(w.atc.calibrate_x),
            ]),
        );
        check(&format!("{label}: state"), py.eval(PY_STATE), state_value(&w.atc));
        check(
            &format!("{label}: commands"),
            py.eval("sorted(g.handlers.keys())"),
            str_list(&w.gcode.commands()),
        );
        check(
            &format!("{label}: help"),
            py.eval("sorted(g.help.items())"),
            Value::List(
                {
                    let mut items: Vec<(String, String)> =
                        w.gcode.get_command_help().into_iter().collect();
                    items.sort();
                    items
                }
                .iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::str(v)]))
                .collect(),
            ),
        );
        check(
            &format!("{label}: event handlers"),
            py.eval("[(k, len(v)) for k, v in p.events.items()]"),
            Value::List(
                w.fp
                    .event_counts()
                    .iter()
                    .map(|(k, n)| Value::Tuple(vec![Value::str(k), Value::int(*n as i64)]))
                    .collect(),
            ),
        );
    }
}

#[test]
fn second_instance_fails_on_duplicate_command() {
    let Some(mut py) = oracle() else { return };
    let w = both(&mut py, &Spec { opts: FULL_OPTS, ..Spec::default() });
    let expected = py.exec(
        "atc.load_config(FakeConfig(p, 'axis_twist_compensation', make_sections({})))",
    );
    let sections = rust_sections(&[]);
    let config = FakeConfig::make(w.printer.clone(), "axis_twist_compensation", sections);
    let actual = AxisTwistCompensation::new(&config).map(|_| ());
    same_outcome("duplicate registration", &expected, &actual);
}

#[test]
fn connect_requires_probe() {
    let Some(mut py) = oracle() else { return };
    let w = both(&mut py, &Spec { opts: FULL_OPTS, with_probe: false, connect: false, ..Spec::default() });
    let expected = py.exec("p.send_event('klippy:connect')");
    let actual = w.printer.send_event(EV_KLIPPY_CONNECT, &[]).map(|_| ());
    same_outcome("connect without probe", &expected, &actual);
    // The calibration command then fails on the missing probe like Python.
    step(&mut py, &w, "no probe", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[]);
}

#[test]
fn calibrate_before_connect_is_attribute_error() {
    let Some(mut py) = oracle() else { return };
    let w = both(&mut py, &Spec { opts: FULL_OPTS, connect: false, ..Spec::default() });
    step(&mut py, &w, "before connect", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[]);
}

#[test]
fn clear_compensations_per_axis() {
    let Some(mut py) = oracle() else { return };
    for (label, py_arg, rust_arg) in [
        ("all axes", "", None),
        ("axis X", "'X'", Some("X")),
        ("axis Y", "'Y'", Some("Y")),
        ("other axis", "'Z'", Some("Z")),
        ("lowercase axis", "'x'", Some("x")),
    ] {
        let w = both(&mut py, &Spec { opts: FULL_OPTS, ..Spec::default() });
        py.exec(&format!("a.clear_compensations({py_arg})")).expect("py clear");
        w.atc.clear_compensations(rust_arg);
        check(&format!("{label}: state"), py.eval(PY_STATE), state_value(&w.atc));
    }
}

#[test]
fn update_results_interpolates_tables() {
    let Some(mut py) = oracle() else { return };
    // X table only: samples, interior, exact ends, and both extrapolations.
    let x_rows: Vec<ResultRow> = [10.0, 20.0, 65.5, 110.0, 150.25, 200.0, 250.0, -40.0]
        .iter()
        .map(|x| [x + 0.5, 12.5, 0.3125, *x, 77.0, 0.25])
        .collect();
    let opts_x: &[(&str, &str)] = &[
        ("z_compensations", "0.1, 0.2, -0.05"),
        ("compensation_start_x", "20"),
        ("compensation_end_x", "200"),
        ("zy_compensations", ""),
    ];
    let w = both(&mut py, &Spec { opts: opts_x, ..Spec::default() });
    update_results(&mut py, &w, "x table", &x_rows);

    // Y table only, four samples.
    let y_rows: Vec<ResultRow> = [30.0, 31.7, 75.0, 105.5, 180.0, 181.0, 0.0]
        .iter()
        .map(|y| [3.0, y - 0.25, -0.1, 4.0, *y, 0.2])
        .collect();
    let opts_y: &[(&str, &str)] = &[
        ("zy_compensations", "0.03, -0.02, 0.01, 0.0"),
        ("compensation_start_y", "30"),
        ("compensation_end_y", "180"),
    ];
    let w = both(&mut py, &Spec { opts: opts_y, ..Spec::default() });
    update_results(&mut py, &w, "y table", &y_rows);

    // Both tables add up in the order x then y.
    let w = both(&mut py, &Spec { opts: FULL_OPTS, ..Spec::default() });
    let both_rows: Vec<ResultRow> = vec![
        [20.0, 30.0, 0.1, 20.0, 30.0, 1.0],
        [100.5, 99.5, -0.3, 101.25, 100.75, 0.7],
        [199.9, 181.0, 1e-9, 199.9, 181.0, -2.0],
        [-5.0, -5.0, 12345.678, -5.0, -5.0, 0.0],
    ];
    update_results(&mut py, &w, "both tables", &both_rows);

    // Two samples per table: the index is always clamped to 0.
    let opts_two: &[(&str, &str)] = &[
        ("z_compensations", "0.2, -0.2"),
        ("compensation_start_x", "0"),
        ("compensation_end_x", "10"),
        ("zy_compensations", "-0.1, 0.4"),
        ("compensation_start_y", "10"),
        ("compensation_end_y", "0"),
    ];
    let w = both(&mut py, &Spec { opts: opts_two, ..Spec::default() });
    let two_rows: Vec<ResultRow> = vec![
        [1.0, 2.0, 3.0, 5.0, 5.0, 4.0],
        [1.0, 2.0, 3.0, -5.0, 15.0, 4.0],
        [1.0, 2.0, 3.0, 15.0, -5.0, 4.0],
    ];
    update_results(&mut py, &w, "two samples", &two_rows);

    // No tables: the result is rebuilt with `bed_z + 0.`, which turns -0.0 into 0.0.
    let w = both(&mut py, &Spec::default());
    update_results(
        &mut py,
        &w,
        "no tables",
        &[[1.0, 2.0, -0.0, 3.0, 4.0, 5.0], [1.0, 2.0, 7.5, 3.0, 4.0, 5.0]],
    );

    // An empty result list is accepted.
    update_results(&mut py, &w, "empty list", &[]);
}

#[test]
fn update_results_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let row = |x: f64, y: f64| -> Vec<ResultRow> { vec![[1.0, 2.0, 3.0, x, y, 4.0]] };
    type ErrorCase<'a> = (&'a str, Vec<(&'a str, &'a str)>, Vec<ResultRow>);
    let cases: Vec<ErrorCase> = vec![
        (
            "x table without range",
            vec![("z_compensations", "0.1, 0.2")],
            row(5.0, 5.0),
        ),
        (
            "x table without start",
            vec![("z_compensations", "0.1, 0.2"), ("compensation_end_x", "100")],
            row(5.0, 5.0),
        ),
        (
            "x table without end",
            vec![("z_compensations", "0.1, 0.2"), ("compensation_start_x", "100")],
            row(5.0, 5.0),
        ),
        (
            "y table without range",
            vec![("zy_compensations", "0.1, 0.2")],
            row(5.0, 5.0),
        ),
        (
            "single sample",
            vec![
                ("z_compensations", "0.1"),
                ("compensation_start_x", "0"),
                ("compensation_end_x", "10"),
            ],
            row(5.0, 5.0),
        ),
        (
            "empty range",
            vec![
                ("z_compensations", "0.1, 0.2"),
                ("compensation_start_x", "5"),
                ("compensation_end_x", "5"),
            ],
            row(5.0, 5.0),
        ),
        (
            "nan coordinate",
            vec![
                ("z_compensations", "0.1, 0.2"),
                ("compensation_start_x", "0"),
                ("compensation_end_x", "10"),
            ],
            row(f64::NAN, 5.0),
        ),
        (
            "infinite coordinate",
            vec![
                ("z_compensations", "0.1, 0.2"),
                ("compensation_start_x", "0"),
                ("compensation_end_x", "10"),
            ],
            row(f64::INFINITY, 5.0),
        ),
        (
            "negative infinite coordinate",
            vec![
                ("z_compensations", "0.1, 0.2"),
                ("compensation_start_x", "0"),
                ("compensation_end_x", "10"),
            ],
            row(f64::NEG_INFINITY, 5.0),
        ),
        (
            "y error after x applied",
            vec![
                ("z_compensations", "0.1, 0.2"),
                ("compensation_start_x", "0"),
                ("compensation_end_x", "10"),
                ("zy_compensations", "0.1, 0.2"),
                ("compensation_start_y", "3"),
            ],
            row(5.0, 5.0),
        ),
    ];
    for (label, opts, rows) in cases {
        let w = both(&mut py, &Spec { opts: &opts, ..Spec::default() });
        update_results(&mut py, &w, label, &rows);
    }
}

type Script = Vec<(&'static str, Vec<(&'static str, &'static str)>)>;

/// Steps of a full manual calibration: the command, then for each manual
/// probe the TESTZ move and ACCEPT.
fn calibration_script(params: &[(&'static str, &'static str)], testz: &[&'static str]) -> Script {
    let mut script = vec![("AXIS_TWIST_COMPENSATION_CALIBRATE", params.to_vec())];
    for z in testz {
        script.push(("TESTZ", vec![("Z", *z)]));
        script.push(("ACCEPT", vec![]));
    }
    script
}

fn run_script(
    py: &mut pyoracle::Oracle,
    w: &World,
    label: &str,
    script: &[(&str, Vec<(&str, &str)>)],
) {
    for (cmd, params) in script {
        step(py, w, label, cmd, params);
    }
}

fn after_calibration_results(py: &mut pyoracle::Oracle, w: &World, label: &str) {
    update_results(
        py,
        w,
        &format!("{label}: compensation applied"),
        &[
            [20.0, 30.0, 0.1, 20.0, 30.0, 1.0],
            [100.5, 99.5, -0.3, 101.25, 100.75, 0.7],
            [199.9, 181.0, 1e-9, 150.0, 90.0, -2.0],
            [-5.0, -5.0, 12.5, -5.0, 300.0, 0.0],
        ],
    );
}

#[test]
fn calibrate_x_default_axis_and_samples() {
    let Some(mut py) = oracle() else { return };
    let spec = Spec {
        opts: FULL_OPTS,
        zs: vec![1.1, 1.2, 1.15],
        offsets: [23.0, -5.5, 1.2],
        lift_speed: 9.0,
        ..Spec::default()
    };
    let w = both(&mut py, &spec);
    compare_world(&mut py, &w, "initial");
    run_script(
        &mut py,
        &w,
        "calibrate X",
        &calibration_script(&[], &["-3.0", "-2.75", "-3.25"]),
    );
    after_calibration_results(&mut py, &w, "calibrate X");
}

#[test]
fn calibrate_x_two_and_five_samples() {
    let Some(mut py) = oracle() else { return };
    let spec = Spec {
        opts: X_OPTS,
        zs: vec![0.9, 1.05],
        offsets: [0.0, 0.0, 0.0],
        lift_speed: 11.5,
        ..Spec::default()
    };
    let w = both(&mut py, &spec);
    run_script(
        &mut py,
        &w,
        "two samples",
        &calibration_script(&[("SAMPLE_COUNT", "2"), ("AXIS", "X")], &["-3.1", "-3.2"]),
    );
    after_calibration_results(&mut py, &w, "two samples");

    let spec = Spec {
        opts: X_OPTS,
        zs: vec![1.01, 0.97, 1.13, 1.02, 0.99],
        offsets: [-17.25, 4.125, 0.4],
        lift_speed: 3.0,
        ..Spec::default()
    };
    let w = both(&mut py, &spec);
    run_script(
        &mut py,
        &w,
        "five samples",
        &calibration_script(
            &[("SAMPLE_COUNT", "5")],
            &["-3.3", "-2.9", "-3.05", "-3.4", "-3.15"],
        ),
    );
    after_calibration_results(&mut py, &w, "five samples");
}

#[test]
fn calibrate_y_axis() {
    let Some(mut py) = oracle() else { return };
    let spec = Spec {
        opts: Y_OPTS,
        zs: vec![0.8, 0.85, 0.83, 0.9],
        offsets: [10.0, 20.0, 1.0],
        lift_speed: 6.0,
        ..Spec::default()
    };
    let w = both(&mut py, &spec);
    run_script(
        &mut py,
        &w,
        "calibrate Y",
        &calibration_script(
            &[("AXIS", "Y"), ("SAMPLE_COUNT", "4")],
            &["-2.5", "-2.45", "-2.6", "-2.4"],
        ),
    );
    after_calibration_results(&mut py, &w, "calibrate Y");
}

#[test]
fn calibrate_abort_and_failed_accept() {
    let Some(mut py) = oracle() else { return };
    // ABORT after the first point.
    let spec = Spec { opts: FULL_OPTS, zs: vec![1.0, 1.1, 1.2], ..Spec::default() };
    let w = both(&mut py, &spec);
    run_script(&mut py, &w, "abort", &calibration_script(&[], &["-3.0"]));
    step(&mut py, &w, "abort", "ABORT", &[]);
    after_calibration_results(&mut py, &w, "abort");

    // ACCEPT without moving the nozzle down is a failed manual probe.
    let w = both(&mut py, &spec);
    step(&mut py, &w, "failed accept", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[]);
    step(&mut py, &w, "failed accept", "ACCEPT", &[]);
    after_calibration_results(&mut py, &w, "failed accept");

    // ABORT on the Y axis keeps the X table that was never cleared.
    let spec = Spec { opts: FULL_OPTS, zs: vec![1.0, 1.1], ..Spec::default() };
    let w = both(&mut py, &spec);
    step(
        &mut py,
        &w,
        "abort Y",
        "AXIS_TWIST_COMPENSATION_CALIBRATE",
        &[("AXIS", "Y"), ("SAMPLE_COUNT", "2")],
    );
    step(&mut py, &w, "abort Y", "TESTZ", &[("Z", "-1.5")]);
    step(&mut py, &w, "abort Y", "ACCEPT", &[]);
    step(&mut py, &w, "abort Y", "ABORT", &[]);
    after_calibration_results(&mut py, &w, "abort Y");
}

#[test]
fn calibrate_command_errors() {
    let Some(mut py) = oracle() else { return };
    let base = |opts: &[(&'static str, &'static str)]| -> Vec<(&'static str, &'static str)> {
        opts.to_vec()
    };
    type Case = (&'static str, Vec<(&'static str, &'static str)>, Vec<(&'static str, &'static str)>);
    let cases: Vec<Case> = vec![
        ("sample count 1", base(FULL_OPTS), vec![("SAMPLE_COUNT", "1")]),
        ("sample count 0", base(FULL_OPTS), vec![("SAMPLE_COUNT", "0")]),
        ("sample count negative", base(FULL_OPTS), vec![("SAMPLE_COUNT", "-4")]),
        ("invalid axis Z", base(FULL_OPTS), vec![("AXIS", "Z")]),
        ("invalid axis lowercase", base(FULL_OPTS), vec![("AXIS", "x")]),
        (
            "x without start",
            vec![("calibrate_end_x", "200"), ("calibrate_y", "100"), ("z_compensations", "0.1,0.2")],
            vec![],
        ),
        (
            "x without end",
            vec![("calibrate_start_x", "20"), ("calibrate_y", "100"), ("z_compensations", "0.1,0.2")],
            vec![],
        ),
        (
            "x without y",
            vec![("calibrate_start_x", "20"), ("calibrate_end_x", "200")],
            vec![],
        ),
        (
            "x with zero start",
            vec![("calibrate_start_x", "0"), ("calibrate_end_x", "200"), ("calibrate_y", "100")],
            vec![],
        ),
        (
            "x with zero y",
            vec![("calibrate_start_x", "20"), ("calibrate_end_x", "200"), ("calibrate_y", "0.0")],
            vec![],
        ),
        (
            "y without x",
            vec![("calibrate_start_y", "30"), ("calibrate_end_y", "180"), ("zy_compensations", "0.1,0.2")],
            vec![("AXIS", "Y")],
        ),
        (
            "y without start",
            vec![("calibrate_x", "110"), ("calibrate_end_y", "180")],
            vec![("AXIS", "Y")],
        ),
        (
            "y with zero x",
            vec![("calibrate_x", "0"), ("calibrate_start_y", "30"), ("calibrate_end_y", "180")],
            vec![("AXIS", "Y")],
        ),
        (
            "y with zero start",
            vec![("calibrate_x", "110"), ("calibrate_start_y", "0"), ("calibrate_end_y", "180")],
            vec![("AXIS", "Y")],
        ),
        (
            "y without end is a type error",
            vec![("calibrate_x", "110"), ("calibrate_start_y", "30")],
            vec![("AXIS", "Y")],
        ),
    ];
    for (label, opts, params) in cases {
        let spec = Spec {
            opts: &opts,
            zs: vec![1.0, 1.1, 1.2],
            offsets: [1.0, 2.0, 0.5],
            ..Spec::default()
        };
        let w = both(&mut py, &spec);
        step(&mut py, &w, label, "AXIS_TWIST_COMPENSATION_CALIBRATE", &params);
    }
}

#[test]
fn calibrate_refuses_while_manual_probe_runs() {
    let Some(mut py) = oracle() else { return };
    let spec = Spec { opts: FULL_OPTS, zs: vec![1.0, 1.1, 1.2], ..Spec::default() };
    let w = both(&mut py, &spec);
    py.exec("g.register_command('ACCEPT', lambda gcmd: None)").expect("py ACCEPT");
    w.gcode
        .register_command("ACCEPT", Some(Box::new(|_g| Ok(()))), RegisterCommandOpts::default())
        .expect("rust ACCEPT");
    step(&mut py, &w, "manual probe active", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[]);
    step(&mut py, &w, "manual probe active Y", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[("AXIS", "Y")]);
}

#[test]
fn calibrate_toolhead_move_failure() {
    let Some(mut py) = oracle() else { return };
    let spec = Spec { opts: FULL_OPTS, zs: vec![1.0, 1.1, 1.2], ..Spec::default() };
    let w = both(&mut py, &spec);
    py.exec("th.fail_move = True").expect("py fail");
    *w.toolhead.fail_move.lock().unwrap() = true;
    step(&mut py, &w, "move failure", "AXIS_TWIST_COMPENSATION_CALIBRATE", &[]);
}

#[test]
fn factory_builds_the_real_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let load_config = factory.load_config.expect("load_config");
    let spec = Spec { opts: FULL_OPTS, ..Spec::default() };
    let (fp, gcode, config) = {
        let parts = rust_world_parts(&spec);
        (parts.fp, parts.gcode, parts.atc_config)
    };
    let obj = load_config(&config).expect("factory load_config").expect("object");
    let atc: Arc<AxisTwistCompensation> = downcast_arc(&obj).expect("downcast");
    fp.add("axis_twist_compensation", obj);
    py.exec(&format!(
        "p, g, th, cf, probe, a = py_world({}, with_probe=True, connect=True)",
        pydict(FULL_OPTS)
    ))
    .expect("py_world");
    check("factory state", py.eval(PY_STATE), state_value(&atc));
    check(
        "factory commands",
        py.eval("sorted(g.handlers.keys())"),
        str_list(&gcode.commands()),
    );
}
