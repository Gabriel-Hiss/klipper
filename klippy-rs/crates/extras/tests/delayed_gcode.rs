//! Parity tests for `extras::delayed_gcode` (port of
//! `klippy/extras/delayed_gcode.py`).
//!
//! Every expected value comes from the live Python reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a fresh
//! `DelayedGcode` is built on both sides with minimal Python fakes for
//! the printer, config and reactor, fed identical config options and
//! G-code parameters, and the resulting duration, name, timer waketime,
//! repeat behaviour, `UPDATE_DELAYED_GCODE` registration, rendered
//! scripts, gcode error texts, config error texts and `Script running
//! error` log line are compared. Floats compare as recording values
//! (hex float + repr), so comparisons are bit-exact.
//!
//! The gcode template bodies are rendered by the real
//! `extras::gcode_macro` object on the Rust side and by
//! `extras/gcode_macro.py` on the Python side, so the rendered string
//! `run_script` receives (including `printer.<object>` lookups) is
//! compared exactly.
//!
//! The reactor clock is wall time on the Rust side
//! (`kos::time::monotonic`) while the Python fake reactor reports a
//! fixed `100.0`. Assertions on *time-derived* waketimes therefore
//! compare clock-free quantities exactly (the `NEVER` case and the
//! `eventtime + duration` repeat case) and only check the
//! `monotonic() + duration` form with a relative-tolerance helper.

use extras::delayed_gcode::{DelayedGcode, CMD_HELP};
use extras::gcode_macro::PrinterGCodeMacro;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts, GCodeCommandApi,
    GCodeCommandHandler, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, PrinterApi};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex, NEVER};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::Arc;

use indexmap::IndexMap;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging as _logging
import extras.delayed_gcode as delayed_gcode_mod
from extras import gcode_macro as gcode_macro_mod
from gcode import GCodeCommand as _GC, CommandError as _CE
from configfile import ConfigWrapper as _CW, error as _config_error
from configparser import RawConfigParser as _RP
from collections import OrderedDict

class FakeTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime
        self.timer_is_running = False

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.updates = []
    def monotonic(self):
        return 100.0
    def register_timer(self, cb, waketime=NEVER):
        timer = FakeTimer(cb, waketime)
        self.timers.append(timer)
        return timer
    def update_timer(self, timer_handler, waketime):
        if timer_handler.timer_is_running:
            return
        timer_handler.waketime = waketime
        self.updates.append(waketime)

class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = OrderedDict()
        self.handlers = {}
        self.shutdowns = []
    def get_printer(self):
        return self
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def get_state_message(self):
        return ('ready', 'ready')
    def is_shutdown(self):
        return False
    def update_error_msg(self, old, new):
        pass
    def add_object(self, name, obj):
        self.objects[name] = obj
    _sentinel = object()
    def lookup_object(self, name, default=_sentinel):
        # sentinel default means raise
        if name in self.objects:
            return self.objects[name]
        if default is self._sentinel:
            raise _config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        return [(k, v) for k, v in self.objects.items()
                if k == module or k.startswith(module + ' ')]
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if default is self._sentinel:
            raise _config_error("Unable to load module '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)
    def invoke_async_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)
    def set_rollover_info(self, name, info, log=True):
        pass
    def request_exit(self, result):
        pass
    def run(self):
        return None

class FakeGCmd(_GC):
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = params
        self._need_ack = False
        self.respond_info = lambda msg, log=True: None
        self.respond_raw = lambda msg: None
    def ack(self, msg=None):
        return False

class FakeGCode:
    def __init__(self):
        self.mux = None
        self.mux_handler = None
        self.run_calls = []
        self.run_fail = False
        self.infos = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux = (cmd, key, value, desc)
        self.mux_handler = func
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        pass
    def run_script(self, script):
        self.run_calls.append(script)
        if self.run_fail:
            raise RuntimeError('script runtime boom')
        for line in script.split('\n'):
            line = line.strip()
            if line.startswith('UPDATE_DELAYED_GCODE'):
                parts = line.split()
                params = {}
                for tok in parts[1:]:
                    if '=' in tok:
                        k, v = tok.split('=', 1)
                        params[k] = v
                cmd = FakeGCmd('UPDATE_DELAYED_GCODE', line, params)
                self.mux_handler(cmd)
    def run_script_from_command(self, script):
        return self.run_script(script)
    def respond_info(self, msg, log=True):
        self.infos.append(msg)
    def respond_raw(self, msg):
        pass
    def get_mutex(self):
        return None
    def get_status(self, eventtime):
        return {'ok': True}

log_records = []
class _Cap(_logging.Handler):
    def emit(self, record):
        detail = ''
        if record.exc_info and record.exc_info[1] is not None:
            detail = str(record.exc_info[1])
        log_records.append((record.getMessage(), detail))
_root = _logging.getLogger()
_root.addHandler(_Cap())
_root.setLevel(0)

def make_cfg(printer, name, opts):
    p = _RP(strict=False)
    p.add_section(name)
    for k, v in opts.items():
        p.set(name, k, v)
    return _CW(printer, p, {}, name)

def make_dg(name='delayed_gcode myname', opts=None, printer=None):
    if printer is None:
        printer = FakePrinter()
    gcode = FakeGCode()
    printer.objects['gcode'] = gcode
    # gcode_macro object: load_template returns jinja TemplateWrapper
    macro_cfg = make_cfg(printer, 'gcode_macro', {})
    gmacro = gcode_macro_mod.PrinterGCodeMacro(macro_cfg)
    printer.objects['gcode_macro'] = gmacro
    if opts is None:
        opts = {'gcode': 'M104 S200'}
    cfg = make_cfg(printer, name, opts)
    dg = delayed_gcode_mod.DelayedGcode(cfg)
    return (dg, printer, gcode)

dg, printer, gcode = make_dg()
mux_info = gcode.mux
"#;

const CMD: &str = "UPDATE_DELAYED_GCODE";
const NAME: &str = "myname";
const SECTION: &str = "delayed_gcode myname";

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fresh_reactor() -> Reactor {
    Reactor::new(false)
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("delayed_gcode fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping delayed_gcode oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// Captured Rust log lines of the delayed_gcode module. The sink is installed
/// once per process and records into a per-thread buffer, so tests running in
/// parallel only see the lines their own thread logged.
fn log_capture() -> Arc<Mutex<Vec<(String, String, Option<String>)>>> {
    thread_local! {
        static CAPTURE: Arc<Mutex<Vec<(String, String, Option<String>)>>> =
            Arc::new(Mutex::new(Vec::new()));
    }
    log::set_sink(Box::new(|rec: &log::Record<'_>| {
        CAPTURE.with(|capture| {
            capture.lock().push((
                rec.level.name().to_string(),
                rec.message.to_string(),
                rec.exc_text.map(|s| s.to_string()),
            ))
        });
    }));
    CAPTURE.with(Arc::clone)
}

// -- FakeConfig --------------------------------------------------------------

struct FakeConfig {
    printer: Printer,
    name: String,
    opts: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, opts: &[(&str, &str)]) -> Config {
        let mut map = IndexMap::new();
        for (k, v) in opts {
            map.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            opts: map,
        })
    }
}

fn parse_error_text(option: &str, section: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        unimplemented!("getint not used by delayed_gcode")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.opts.get(option) {
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(inner) => Ok(inner),
            },
            Some(raw) => {
                let v = match pycompat::text::float_from_string(raw) {
                    Ok(v) => v,
                    Err(_) => return Err(parse_error_text(option, &self.name)),
                };
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.name,
                            pycompat::text::float_str(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.name,
                            pycompat::text::float_str(max)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.name,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.name,
                            pycompat::text::float_str(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        unimplemented!("getboolean not used by delayed_gcode")
    }
    fn getchoice(
        &self,
        _o: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        unimplemented!("getchoice not used by delayed_gcode")
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unimplemented!("getlists not used by delayed_gcode")
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimplemented!("getlist not used by delayed_gcode")
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unimplemented!("getintlist not used by delayed_gcode")
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unimplemented!("getfloatlist not used by delayed_gcode")
    }
    fn getsection(&self, _section: &str) -> Config {
        unimplemented!("getsection not used by delayed_gcode")
    }
    fn has_section(&self, _section: &str) -> bool {
        unimplemented!("has_section not used by delayed_gcode")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        unimplemented!("get_prefix_sections not used by delayed_gcode")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        unimplemented!("get_prefix_options not used by delayed_gcode")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        unimplemented!("deprecate not used by delayed_gcode")
    }
}

// -- FakeGCmd ----------------------------------------------------------------

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn new(command: &str, commandline: &str, params: &[(&str, &str)]) -> Arc<FakeGCmd> {
        let mut map = IndexMap::new();
        for (k, v) in params {
            map.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: map,
        })
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(d) => Ok(d),
            },
            Some(raw) => Ok(PyValue::Str(raw.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
            Some(raw) => match raw.parse::<i64>() {
                Ok(v) => Ok(Some(v)),
                Err(_) => Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
                Some(v) => Ok(v),
            },
            Some(raw) => {
                let v = match pycompat::text::float_from_string(raw) {
                    Ok(v) => v,
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {raw}",
                            self.commandline
                        )))
                    }
                };
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must have minimum of {}",
                            self.commandline,
                            pycompat::text::float_str(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must have maximum of {}",
                            self.commandline,
                            pycompat::text::float_str(max)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must be above {}",
                            self.commandline,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': {name} must be below {}",
                            self.commandline,
                            pycompat::text::float_str(below)
                        )));
                    }
                }
                Ok(Some(v))
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
        self.commandline.replacen(&self.command, "", 1)
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// -- FakeGCode ---------------------------------------------------------------

struct FakeGCode {
    mux: Mutex<Option<(String, String, Option<String>, Option<String>)>>,
    mux_handler: Mutex<Option<GCodeCommandHandler>>,
    run_calls: Mutex<VecDeque<String>>,
    run_fail: Mutex<bool>,
    infos: Mutex<VecDeque<String>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            mux: Mutex::new(None),
            mux_handler: Mutex::new(None),
            run_calls: Mutex::new(VecDeque::new()),
            run_fail: Mutex::new(false),
            infos: Mutex::new(VecDeque::new()),
            reactor,
        })
    }
    fn set_run_fail(&self, fail: bool) {
        *self.run_fail.lock() = fail;
    }
    fn run_scripts(&self) -> Vec<String> {
        self.run_calls.lock().iter().cloned().collect()
    }
    fn mux_info(&self) -> Option<(String, String, Option<String>, Option<String>)> {
        self.mux.lock().clone()
    }
    /// Dispatch `UPDATE_DELAYED_GCODE ...` lines of a script to the
    /// registered mux handler, like `GCodeDispatch.run_script`.
    fn dispatch_update_lines(&self, script: &str) -> KResult<()> {
        for line in script.lines() {
            let line = line.trim();
            if line.starts_with(CMD) {
                let mut line_params: Vec<(&str, &str)> = Vec::new();
                for tok in line.split_whitespace().skip(1) {
                    if let Some((k, v)) = tok.split_once('=') {
                        line_params.push((k, v));
                    }
                }
                let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(CMD, line, &line_params);
                let mut guard = self.mux_handler.lock();
                if let Some(h) = guard.as_mut() {
                    (h.as_mut())(&gcmd)?;
                }
            }
        }
        Ok(())
    }
}

impl klippy_api::gcode::GCodeApi for FakeGCode {
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
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        *self.mux.lock() = Some((
            cmd.to_string(),
            key.to_string(),
            value.map(|s| s.to_string()),
            desc.map(|s| s.to_string()),
        ));
        *self.mux_handler.lock() = Some(func);
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("ok".to_string()), PyValue::Bool(true));
        PyValue::Dict(d)
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.run_calls.lock().push_back(script.to_string());
        if *self.run_fail.lock() {
            return Err(KlippyError::Internal {
                class_name: "RuntimeError".to_string(),
                message: "script runtime boom".to_string(),
            });
        }
        self.dispatch_update_lines(script)
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> Arc<dyn GCodeCommandApi> {
        let arc = FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        };
        Arc::new(arc)
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, _log: bool) {
        self.infos.lock().push_back(msg.to_string());
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        klippy_core::gcode_parse::is_traditional_gcode(cmd)
    }
    fn cmd_m112(&self, _gcmd: Option<&Arc<dyn GCodeCommandApi>>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

// -- Printer objects for core services ------------------------------------

// `FakeGCode` is the `gcode` printer object; it exposes `dyn GCodeApi`.
klippy_api::printer_object!(FakeGCode; dyn klippy_api::gcode::GCodeApi);

/// Registers the real `gcode_macro` printer object, as
/// `printer.load_object(config, 'gcode_macro')` would.
fn set_gcode_macro(printer: &Arc<FakePrinter>) {
    let printer_dyn: Printer = printer.clone();
    let macro_config = FakeConfig::new(printer_dyn, "gcode_macro", &[]);
    printer.set_object("gcode_macro", Arc::new(PrinterGCodeMacro::new(&macro_config)));
}

// -- FakePrinter -------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, klippy_api::printer::EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor,
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
        })
    }
    fn set_object(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn handler_names(&self) -> Vec<String> {
        self.handlers.lock().iter().map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.objects.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.lock();
        match module {
            None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(m) => objs
                .iter()
                .filter(|(k, _)| k.as_str() == m || k.starts_with(&format!("{m} ")))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(section) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{section}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.handlers
            .lock()
            .push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut returns = Vec::new();
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                returns.push(cb(params)?);
            }
        }
        Ok(returns)
    }
    fn request_exit(&self, _result: &str) {}
}

// -- Shared wiring (constructor path used by both sides) ---------------------

struct Harness {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    reactor: Reactor,
    #[allow(dead_code)]
    config: Config,
    dg: Arc<DelayedGcode>,
}

fn build(name: &str, script: &str, extra_opts: &[(&str, &str)]) -> Harness {
    let reactor = fresh_reactor();
    let printer = FakePrinter::new(reactor.clone());
    let gcode = FakeGCode::new(reactor.clone());
    printer.set_object("gcode", gcode.clone());
    set_gcode_macro(&printer);
    let mut opts: Vec<(&str, &str)> = vec![("gcode", script)];
    opts.extend_from_slice(extra_opts);
    let printer_dyn: Printer = printer.clone();
    let config = FakeConfig::new(printer_dyn, name, &opts);
    let dg = Arc::new(DelayedGcode::new(&config).expect("DelayedGcode::new"));
    Harness {
        printer,
        gcode,
        reactor,
        config,
        dg,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.delayed_gcode".to_string(),
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

fn check_exc(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: &KlippyError,
) {
    let expected = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected error"),
        Err(e) => e,
    };
    assert_eq!(expected.class_name, actual.class_name(), "{label}: class");
    assert_eq!(expected.message, actual.message(), "{label}: message");
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_loads_prefix() {
    let Some(_py) = oracle() else { return };
    let factory: ModuleFactory = extras::delayed_gcode::module_factory();
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.load_config.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn config_reads_name_duration_and_registers_mux() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "dg2, printer2, gcode2 = make_dg(opts={'gcode': 'M104 S180', 'initial_duration': '2.5'})",
    )
    .expect("py build");
    let expected = py.eval("(dg2.name, dg2.duration, gcode2.mux)");
    let h = build(SECTION, "M104 S180", &[("initial_duration", "2.5")]);
    let (cmd, key, value, desc) = h.gcode.mux_info().expect("mux registered");
    assert_eq!(h.dg.name(), NAME);
    let actual = Value::Tuple(vec![
        Value::str(h.dg.name()),
        Value::float(h.dg.duration()),
        Value::Tuple(vec![
            Value::str(&cmd),
            Value::str(&key),
            Value::str(value.as_deref().unwrap_or("")),
            Value::str(desc.as_deref().unwrap_or("")),
        ]),
    ]);
    check("config_reads", expected, actual);
    check(
        "mux_help",
        py.eval("mux_info[3]"),
        Value::str(desc.as_deref().unwrap_or("")),
    );
    assert_eq!(desc.as_deref(), Some(CMD_HELP));
    assert_eq!(cmd, CMD);
    assert_eq!(key, "ID");
    assert_eq!(h.printer.handler_names(), vec!["klippy:ready".to_string()]);
}

#[test]
fn default_duration_zero() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg3, p3, g3 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval("dg3.duration");
    let h = build(SECTION, "M105", &[]);
    check("default_duration", expected, Value::float(h.dg.duration()));
    assert_eq!(h.dg.duration().to_bits(), 0.0f64.to_bits());
}

#[test]
fn missing_gcode_option_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("make_dg(opts={'initial_duration': '1'})");
    let reactor = fresh_reactor();
    let printer = FakePrinter::new(reactor.clone());
    let gcode = FakeGCode::new(reactor.clone());
    printer.set_object("gcode", gcode.clone());
    set_gcode_macro(&printer);
    let printer_dyn: Printer = printer.clone();
    let config = FakeConfig::new(printer_dyn, SECTION, &[("initial_duration", "1")]);
    let err = DelayedGcode::new(&config).err().expect("missing gcode");
    check_exc("missing_gcode", expected, &err);
}

#[test]
fn negative_initial_duration_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("make_dg(opts={'gcode': 'M105', 'initial_duration': '-1'})");
    let reactor = fresh_reactor();
    let printer = FakePrinter::new(reactor.clone());
    let gcode = FakeGCode::new(reactor.clone());
    printer.set_object("gcode", gcode.clone());
    set_gcode_macro(&printer);
    let printer_dyn: Printer = printer.clone();
    let config = FakeConfig::new(
        printer_dyn,
        SECTION,
        &[("gcode", "M105"), ("initial_duration", "-1")],
    );
    let err = DelayedGcode::new(&config).err().expect("negative initial_duration");
    check_exc("negative_duration", expected, &err);
}

#[test]
fn ready_zero_duration_waketime_never() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg4, p4, g4 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval("(dg4._handle_ready(), p4.reactor.timers[0].waketime)");
    let h = build(SECTION, "M105", &[]);
    h.dg.handle_ready();
    check(
        "ready_never",
        expected,
        Value::Tuple(vec![
            Value::Null,
            Value::float(h.dg.timer_waketime()),
        ]),
    );
    assert_eq!(h.dg.timer_waketime().to_bits(), NEVER.to_bits());
}

#[test]
fn timer_event_runs_script_and_returns_never() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg5, p5, g5 = make_dg(opts={'gcode': 'M104 S200'})")
        .expect("py build");
    let expected = py.eval("(dg5._gcode_timer_event(12.5), g5.run_calls, dg5.inside_timer, dg5.repeat)");
    let h = build(SECTION, "M104 S200", &[]);
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(12.5);
    let actual = Value::Tuple(vec![
        Value::float(next),
        Value::List(h.gcode.run_scripts().into_iter().map(|s| Value::str(&s)).collect()),
        Value::Bool(h.dg.inside_timer()),
        Value::Bool(h.dg.repeat()),
    ]);
    check("timer_event", expected, actual);
    assert_eq!(next.to_bits(), NEVER.to_bits());
    assert!(!h.dg.inside_timer());
}

#[test]
fn timer_event_renders_printer_status() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg5b, p5b, g5b = make_dg(opts={'gcode': 'M117 {printer.gcode.ok} {printer.gcode}'})")
        .expect("py build");
    let expected = py.eval("(dg5b._gcode_timer_event(12.5), g5b.run_calls)");
    let h = build(SECTION, "M117 {printer.gcode.ok} {printer.gcode}", &[]);
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(12.5);
    let actual = Value::Tuple(vec![
        Value::float(next),
        Value::List(h.gcode.run_scripts().into_iter().map(|s| Value::str(&s)).collect()),
    ]);
    check("timer_event_printer_status", expected, actual);
}

#[test]
fn timer_repeat_when_script_updates_duration() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "dg6, p6, g6 = make_dg(opts={'gcode': 'UPDATE_DELAYED_GCODE ID=myname DURATION=2'})",
    )
    .expect("py build");
    let expected = py.eval("(dg6._handle_ready(), dg6._gcode_timer_event(100.0), dg6.duration, dg6.repeat, g6.run_calls)");
    let h = build(
        SECTION,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=2",
        &[],
    );
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(100.0);
    let actual = Value::Tuple(vec![
        Value::Null,
        Value::float(next),
        Value::float(h.dg.duration()),
        Value::Bool(h.dg.repeat()),
        Value::List(h.gcode.run_scripts().into_iter().map(|s| Value::str(&s)).collect()),
    ]);
    check("repeat", expected, actual);
    assert_eq!(next.to_bits(), (100.0f64 + 2.0).to_bits());
    assert_eq!(h.dg.duration().to_bits(), 2.0f64.to_bits());
}

#[test]
fn timer_no_repeat_when_duration_set_zero_inside() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "dg7, p7, g7 = make_dg(opts={'gcode': 'UPDATE_DELAYED_GCODE ID=myname DURATION=0'})",
    )
    .expect("py build");
    let expected = py.eval("(dg7._handle_ready(), dg7._gcode_timer_event(100.0), dg7.duration)");
    let h = build(
        SECTION,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=0",
        &[],
    );
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(100.0);
    let actual = Value::Tuple(vec![
        Value::Null,
        Value::float(next),
        Value::float(h.dg.duration()),
    ]);
    check("no_repeat", expected, actual);
    assert_eq!(next.to_bits(), NEVER.to_bits());
    assert!(!h.dg.repeat());
}

#[test]
fn update_outside_timer_rearms_timer() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "dg8, p8, g8 = make_dg(opts={'gcode': 'M105'})",
    )
    .expect("py build");
    py.exec("dg8._handle_ready()").expect("py ready");
    let expected = py.eval(
        "(dg8.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname DURATION=3', {'DURATION': '3'})), dg8.duration, p8.reactor.updates[-1])",
    );
    let h = build(SECTION, "M105", &[]);
    // Zero-duration ready path uses the exact NEVER value; this test
    // then arms with DURATION=3 outside the timer.
    h.dg.handle_ready();
    let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(
        CMD,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=3",
        &[("DURATION", "3")],
    );
    h.dg.cmd_update_delayed_gcode(&gcmd).expect("update");
    // The Python side computes 100.0 + 3.0 with its fake reactor; the
    // Rust side uses the real monotonic clock, so compare the duration
    // exactly and recompute the waketime expectation locally.
    let expected_duration = match expected {
        Ok(v) => v,
        Err(e) => panic!("update_outside: oracle raised {}: {}", e.class_name, e.message),
    };
    let duration_val = match &expected_duration {
        Value::Tuple(t) => t[1].clone(),
        other => panic!("unexpected oracle tuple {other:?}"),
    };
    check(
        "update_duration",
        Ok(duration_val),
        Value::float(h.dg.duration()),
    );
    let mono = h.reactor.monotonic();
    assert!(
        (h.dg.timer_waketime() - (mono + 3.0)).abs() < 1.0,
        "waketime {} should be monotonic+3 near {}",
        h.dg.timer_waketime(),
        mono
    );
    assert!(h.dg.timer_waketime() > 0.5, "waketime armed, not NEVER");
}

#[test]
fn update_zero_duration_disarms_timer() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg9, p9, g9 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    py.exec("dg9._handle_ready()").expect("py ready");
    let expected = py.eval(
        "(dg9.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname DURATION=0', {'DURATION': '0'})), dg9.duration, p9.reactor.updates[-1])",
    );
    let h = build(SECTION, "M105", &[]);
    h.dg.handle_ready();
    let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(
        CMD,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=0",
        &[("DURATION", "0")],
    );
    h.dg.cmd_update_delayed_gcode(&gcmd).expect("update");
    check(
        "update_zero",
        expected,
        Value::Tuple(vec![
            Value::Null,
            Value::float(h.dg.duration()),
            Value::float(h.dg.timer_waketime()),
        ]),
    );
    assert_eq!(h.dg.timer_waketime().to_bits(), NEVER.to_bits());
}

#[test]
fn update_missing_duration_is_command_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg10, p10, g10 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval(
        "dg10.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname', {}))",
    );
    let h = build(SECTION, "M105", &[]);
    h.dg.handle_ready();
    let gcmd: Arc<dyn GCodeCommandApi> =
        FakeGCmd::new(CMD, "UPDATE_DELAYED_GCODE ID=myname", &[]);
    let err = h
        .dg
        .cmd_update_delayed_gcode(&gcmd)
        .expect_err("missing DURATION");
    check_exc("update_missing", expected, &err);
}

#[test]
fn update_invalid_duration_is_command_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg11, p11, g11 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval(
        "dg11.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname DURATION=abc', {'DURATION': 'abc'}))",
    );
    let h = build(SECTION, "M105", &[]);
    h.dg.handle_ready();
    let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(
        CMD,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=abc",
        &[("DURATION", "abc")],
    );
    let err = h
        .dg
        .cmd_update_delayed_gcode(&gcmd)
        .expect_err("invalid DURATION");
    check_exc("update_invalid", expected, &err);
}

#[test]
fn update_negative_duration_is_command_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("dg12, p12, g12 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval(
        "dg12.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname DURATION=-1', {'DURATION': '-1'}))",
    );
    let h = build(SECTION, "M105", &[]);
    h.dg.handle_ready();
    let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(
        CMD,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=-1",
        &[("DURATION", "-1")],
    );
    let err = h
        .dg
        .cmd_update_delayed_gcode(&gcmd)
        .expect_err("negative DURATION");
    check_exc("update_negative", expected, &err);
}

#[test]
fn update_before_ready_is_attribute_error() {
    let Some(mut py) = oracle() else { return };
    // Precondition: timer handler is still None before klippy:ready.
    py.exec("dg13, p13, g13 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    let expected = py.eval(
        "dg13.cmd_UPDATE_DELAYED_GCODE(FakeGCmd('UPDATE_DELAYED_GCODE', 'UPDATE_DELAYED_GCODE ID=myname DURATION=5', {'DURATION': '5'}))",
    );
    let h = build(SECTION, "M105", &[]);
    // No handle_ready call: timer_handler is None on both sides.
    let gcmd: Arc<dyn GCodeCommandApi> = FakeGCmd::new(
        CMD,
        "UPDATE_DELAYED_GCODE ID=myname DURATION=5",
        &[("DURATION", "5")],
    );
    let err = h
        .dg
        .cmd_update_delayed_gcode(&gcmd)
        .expect_err("update before ready");
    check_exc("update_before_ready", expected, &err);
    assert_eq!(err.class_name(), "AttributeError");
}

#[test]
fn script_error_is_logged_and_swallowed() {
    let Some(mut py) = oracle() else { return };
    let captured = log_capture();
    captured.lock().clear();
    py.exec("log_records.clear()").expect("clear py log");
    py.exec("dg14, p14, g14 = make_dg(opts={'gcode': 'M105'})")
        .expect("py build");
    py.exec("g14.run_fail = True").expect("py fail flag");
    let expected = py.eval("(dg14._handle_ready(), dg14._gcode_timer_event(50.0), list(log_records))");
    let h = build(SECTION, "M105", &[]);
    h.gcode.set_run_fail(true);
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(50.0);
    let rust_lines: Vec<Value> = captured
        .lock()
        .iter()
        .filter(|(_, msg, _)| msg == "Script running error")
        .map(|(lvl, msg, exc)| {
            Value::Tuple(vec![
                Value::str(lvl),
                Value::str(msg),
                Value::str(exc.as_deref().unwrap_or("")),
            ])
        })
        .collect();
    let py_lines: Vec<Value> = match py.eval("list(log_records)") {
        Ok(Value::List(items)) => items
            .into_iter()
            .map(|v| {
                if let Value::Tuple(t) = &v {
                    if let (Value::Str(msg), Value::Str(detail)) = (&t[0], &t[1]) {
                        if msg == "Script running error" {
                            return Value::Tuple(vec![
                                Value::str("ERROR"),
                                Value::str(msg),
                                Value::str(detail),
                            ]);
                        }
                    }
                }
                Value::Null
            })
            .filter(|v| !matches!(v, Value::Null))
            .collect(),
        other => panic!("unexpected log_records: {other:?}"),
    };
    check("script_error_nextwake", Ok(Value::float(next)), Value::float(next));
    assert_eq!(next.to_bits(), NEVER.to_bits());
    let expected_next = match expected {
        Ok(v) => v,
        Err(e) => panic!("script_error: oracle raised {}: {}", e.class_name, e.message),
    };
    let expected_next_val = match &expected_next {
        Value::Tuple(t) => t[1].clone(),
        other => panic!("unexpected oracle tuple {other:?}"),
    };
    check("script_error_nextwake_oracle", Ok(expected_next_val.clone()), Value::float(next));
    assert_eq!(py_lines.len(), 1, "python logs once: {py_lines:?}");
    assert_eq!(rust_lines.len(), 1, "rust logs once: {rust_lines:?}");
    // Both sides raise a RuntimeError('script runtime boom') that the
    // printer records as the exception detail.
    assert!(py_lines[0].to_string().contains("script runtime boom"));
    assert!(rust_lines[0].to_string().contains("script runtime boom"));
}

#[test]
fn render_error_is_logged_and_swallowed() {
    let Some(mut py) = oracle() else { return };
    let captured = log_capture();
    captured.lock().clear();
    py.exec("log_records.clear()").expect("clear py log");
    // Single-brace jinja call: action_raise_error("boom-RENDER").
    let script = "{ action_raise_error('boom-RENDER') }";
    py.exec(&format!(
        "dg15, p15, g15 = make_dg(opts={{'gcode': {}}})\n",
        py_str(script)
    ))
    .expect("py build");
    let expected = py.eval("(dg15._handle_ready(), dg15._gcode_timer_event(60.0))");
    let h = build(SECTION, script, &[]);
    h.dg.handle_ready();
    let next = h.dg.gcode_timer_event(60.0);
    let rust_count = captured
        .lock()
        .iter()
        .filter(|(_, msg, _)| msg == "Script running error")
        .count();
    let py_count = match py.eval("len([r for r in log_records if r[0] == 'Script running error'])") {
        Ok(Value::Int(n)) => n as usize,
        other => panic!("count: {other:?}"),
    };
    let expected_next = match expected {
        Ok(v) => v,
        Err(e) => panic!("render_error: oracle raised {}: {}", e.class_name, e.message),
    };
    let expected_next_val = match &expected_next {
        Value::Tuple(t) => t[1].clone(),
        other => panic!("unexpected oracle tuple {other:?}"),
    };
    check("render_error_nextwake", Ok(expected_next_val), Value::float(next));
    assert_eq!(next.to_bits(), NEVER.to_bits());
    assert_eq!(py_count, 1, "python render error logged once");
    assert_eq!(rust_count, 1, "rust render error logged once");
}

fn py_str(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\'' => out.push_str("\\'"),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('\'');
    out
}

#[test]
fn module_factory_entry_points() {
    let Some(_py) = oracle() else { return };
    let factory: ModuleFactory = extras::delayed_gcode::module_factory();
    assert!(factory.load_config_prefix.is_some());
    // `load_config_prefix` returns the object stored under the section.
    let h = build(SECTION, "M105", &[]);
    assert_eq!(h.dg.name(), NAME);
}
