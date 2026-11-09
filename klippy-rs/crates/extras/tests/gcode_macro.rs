//! Parity tests for `extras::gcode_macro` (port of `klippy/extras/gcode_macro.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): both sides are built with
//! minimal Python/Rust fakes (printer registry, gcode dispatcher,
//! webhooks, config wrapper, gcmd) and fed identical inputs. Returned
//! values, exception class/message, command/mux registrations,
//! rendered scripts, `get_status` dicts and shutdown/respond/remote
//! side effects are compared exactly (`recording::assert_equal`).
//! The Jinja renderer of the reference is the real `jinja2` package,
//! so templates chosen here stay inside the subset the Rust engine
//! reproduces (see `extras_macros_templates` docs).

use extras::gcode_macro::{
    create_template_context, GCodeMacro, PrinterGCodeMacro, PrinterGCodeMacroApi,
    TemplateWrapper, SET_GCODE_VARIABLE_HELP,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as CmdGetFloatOpts,
    GetIntOpts as CmdGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import json
import extras.gcode_macro as gcode_macro

class CommandError(Exception):
    pass

# The reference's `printer.command_error` is `gcode.CommandError`
# (`klippy/klippy.py:27`); `traceback.format_exception_only` renders the
# module, so the fake must live in `gcode` too.
CommandError.__module__ = "gcode"

class FakeReactor:
    def monotonic(self):
        return 1234.5
    def assert_no_pause(self):
        import contextlib
        @contextlib.contextmanager
        def cm():
            yield
        return cm()

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.shutdowns = []
        self.event_handlers = []
        self._reactor = FakeReactor()
    def get_reactor(self):
        return self._reactor
    def add_object(self, name, obj):
        if name in self.objects:
            raise configparser.Error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=configparser.Error):
        if name in self.objects:
            return self.objects[name]
        if default is configparser.Error:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def load_object(self, config, section, default=configparser.Error):
        if section in self.objects:
            return self.objects[section]
        raise configparser.Error("Unknown config object '%s'" % (section,))
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)
    def register_event_handler(self, event, cb):
        self.event_handlers.append((event, cb))

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.handlers = {}
        self.calls = []
        self.mux_calls = []
        self.scripts = []
        self.responds = []
    def is_traditional_gcode(self, cmd):
        try:
            cmd = cmd.upper().split()[0]
            val = float(cmd[1:])
            return cmd[0].isupper() and cmd[1].isdigit()
        except:
            return False
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            old = self.handlers.pop(cmd, None)
            self.calls.append("register_command|%s|<None>|%s" % (cmd, desc))
            return old[0] if old else None
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = [func, desc]
        self.calls.append("register_command|%s|fn|%s" % (cmd, desc))
        return None
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_calls.append("register_mux_command|%s|%s|%s|%s" % (cmd, key, value, desc))
        self.handlers[cmd] = [func, desc]
    def run_script_from_command(self, script):
        self.scripts.append(script)
    def run_script(self, script):
        self.scripts.append(script)
    def respond_info(self, msg, log=True):
        self.responds.append("%s|%s" % (bool(log), msg))
    def get_status(self, eventtime):
        return {'commands': {}}

class FakeWebHooks:
    def __init__(self):
        self.calls = []
        self.fail_commands = False
    def call_remote_method(self, method, **kwargs):
        self.calls.append("%s|%s" % (method, json.dumps(kwargs, sort_keys=True)))
        if self.fail_commands:
            raise CommandError("remote method %s failed" % (method,))

class FakeStatusObj:
    def __init__(self, status):
        self.status = status
    def get_status(self, eventtime):
        return dict(self.status)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options=None):
        self._printer = printer
        self._name = name
        self._options = options or {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=configparser.Error):
        if option not in self._options:
            if default is configparser.Error:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        return self._options[option]
    def get_prefix_options(self, prefix):
        return [k for k in self._options if k.startswith(prefix)]

class FakeGCommand:
    error = CommandError
    def __init__(self, command, commandline, params=None, rawparams=""):
        self._command = command
        self._commandline = commandline
        self._params = params or {}
        self._rawparams = rawparams
    def get(self, name, default=configparser.Error):
        if name not in self._params:
            if default is configparser.Error:
                raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
            return default
        return self._params[name]
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return dict(self._params)
    def get_raw_command_parameters(self):
        return self._rawparams

def build_world(section, options, script='M104 S{temp}\n'):
    p = FakePrinter()
    gc = FakeGCode()
    wh = FakeWebHooks()
    p.objects['gcode'] = gc
    p.objects['webhooks'] = wh
    opts = {'gcode': script}
    if options:
        opts.update(options)
    c = FakeConfig(p, section, opts)
    pgm = gcode_macro.load_config(c)
    p.objects['gcode_macro'] = pgm
    return p, gc, wh, c, pgm
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<GCell<IndexMap<String, ObjectRef>>>,
    shutdowns: Arc<GCell<Vec<String>>>,
    handlers: Arc<GCell<Vec<(String, EventCallback)>>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Arc::new(GCell::new(IndexMap::new())),
            shutdowns: Arc::new(GCell::new(Vec::new())),
            handlers: Arc::new(GCell::new(Vec::new())),
        }
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }
    fn shutdowns(&self) -> Vec<String> {
        self.shutdowns.borrow().clone()
    }
    fn handler_count(&self) -> usize {
        self.handlers.borrow().len()
    }
    fn fire_handler(&self, index: usize, args: &[klippy_api::EventArg]) -> KResult<PyValue> {
        let mut cb = {
            let mut h = self.handlers.borrow_mut();
            h.remove(index).1
        };
        (cb)(args)
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
        if self.objects.borrow().contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        self.objects.borrow_mut().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(section) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
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
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        _event: &str,
        _params: &[klippy_api::EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeGCode {
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
    calls: GCell<Vec<String>>,
    descs: GCell<IndexMap<String, Option<String>>>,
    mux_calls: GCell<Vec<String>>,
    scripts: GCell<Vec<String>>,
    responds: GCell<Vec<String>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            handlers: GCell::new(IndexMap::new()),
            calls: GCell::new(Vec::new()),
            descs: GCell::new(IndexMap::new()),
            mux_calls: GCell::new(Vec::new()),
            scripts: GCell::new(Vec::new()),
            responds: GCell::new(Vec::new()),
        }
    }
    fn scripts(&self) -> Vec<String> {
        self.scripts.borrow().clone()
    }
    fn responds(&self) -> Vec<String> {
        self.responds.borrow().clone()
    }
    fn desc_of(&self, cmd: &str) -> Option<String> {
        self.descs.borrow().get(cmd).cloned().flatten()
    }
    fn register_prev(&self, cmd: &str, handler: String) -> Result<(), String> {
        self.calls.borrow_mut().push(format!("register_command|{cmd}|fn|None"));
        if self.handlers.borrow().contains_key(cmd) {
            return Err(format!("gcode command {cmd} already registered"));
        }
        let h: GCodeCommandHandler = Box::new(move |_gcmd: &klippy_api::gcode::GCmd| {
            let _ = &handler;
            Ok(())
        });
        self.handlers.borrow_mut().insert(cmd.to_string(), h);
        Ok(())
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
                let old = self.handlers.borrow_mut().shift_remove(cmd);
                self.calls.borrow_mut().push(format!(
                    "register_command|{}|<None>|{}",
                    cmd,
                    opts.desc.as_deref().unwrap_or("None")
                ));
                Ok(old)
            }
            Some(handler) => {
                if self.handlers.borrow().contains_key(cmd) {
                    return Err(KlippyError::config(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                self.calls.borrow_mut().push(format!(
                    "register_command|{}|fn|{}",
                    cmd,
                    opts.desc.as_deref().unwrap_or("None")
                ));
                self.descs.borrow_mut().insert(cmd.to_string(), opts.desc);
                self.handlers.borrow_mut().insert(cmd.to_string(), handler);
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux_calls.borrow_mut().push(format!(
            "register_mux_command|{cmd}|{key}|{}|{}",
            value.unwrap_or(""),
            desc.unwrap_or("")
        ));
        self.handlers.borrow_mut().insert(cmd.to_string(), func);
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        let mut out = IndexMap::new();
        for (k, v) in self.descs.borrow().iter() {
            if let Some(d) = v {
                out.insert(k.clone(), d.clone());
            }
        }
        out
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("commands".to_string()),
            PyValue::Dict(IndexMap::new()),
        );
        PyValue::Dict(d)
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.borrow_mut().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.scripts.borrow_mut().push(script.to_string());
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("create_gcode_command is not used by gcode_macro")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        self.responds.borrow_mut().push(format!(
            "{}|{}",
            if log { "True" } else { "False" },
            msg
        ));
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        // Mirror klippy/gcode.py is_traditional_gcode in the Python fake.
        let first = cmd.to_uppercase();
        let first = first.split_whitespace().next().unwrap_or("");
        if first.len() < 2 {
            return false;
        }
        let head = first.chars().next().unwrap();
        let tail = &first[1..];
        if !head.is_uppercase() || !first.chars().nth(1).unwrap().is_ascii_digit() {
            return false;
        }
        !tail.is_empty() && tail.parse::<f64>().is_ok()
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        let mut cmds: IndexMap<PyValue, PyValue> = IndexMap::new();
        let _ = &mut cmds;
        d.insert(PyValue::Str("commands".to_string()), PyValue::Dict(cmds));
        Some(PyValue::Dict(d))
    }
}

impl PrinterObject for FakeWebHooks {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn WebHooksApi);
}

struct FakeWebHooks {
    calls: GCell<Vec<String>>,
    fail_commands: bool,
}

impl FakeWebHooks {
    fn new(fail_commands: bool) -> FakeWebHooks {
        FakeWebHooks {
            calls: GCell::new(Vec::new()),
            fail_commands,
        }
    }
    fn calls(&self) -> Vec<String> {
        self.calls.borrow().clone()
    }
}

impl WebHooksApi for FakeWebHooks {
    fn register_endpoint(
        &self,
        _path: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        _path: &str,
        _key: &str,
        _value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn call_remote_method(
        &self,
        method: &str,
        params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        // Match the Python fake's `json.dumps(kwargs, sort_keys=True)`.
        let mut pairs: Vec<(PyValue, PyValue)> = params
            .iter()
            .map(|(k, v)| (PyValue::Str(k.clone()), v.clone()))
            .collect();
        pairs.sort_by(|a, b| match (&a.0, &b.0) {
            (PyValue::Str(x), PyValue::Str(y)) => x.cmp(y),
            _ => std::cmp::Ordering::Equal,
        });
        let dict = PyValue::Dict(pairs.into_iter().collect());
        let rendered = pycompat::json::dumps(&dict).unwrap_or_else(|_| "{}".to_string());
        self.calls.borrow_mut().push(format!("{method}|{rendered}"));
        if self.fail_commands {
            return Err(KlippyError::command(format!(
                "remote method {method} failed"
            )));
        }
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: Vec<(&str, &str)>) -> FakeConfig {
        let mut map = IndexMap::new();
        for (k, v) in options {
            map.insert(k.to_string(), v.to_string());
        }
        FakeConfig {
            printer,
            name: name.to_string(),
            options: map,
        }
    }
}

/// Build a `Config` from `FakeConfig`, coercing to `Arc<dyn ConfigApi>`.
fn cfg_of(printer: Printer, name: &str, options: Vec<(&str, &str)>) -> Config {
    Arc::new(FakeConfig::new(printer, name, options))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _o: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _o: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
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
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection is not used by gcode_macro")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        Ok(self.options.keys().filter(|k| k.starts_with(prefix)).cloned().collect())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeGCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    rawparams: String,
}

impl GCodeCommandApi for FakeGCommand {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_int(&self, _name: &str, _opts: CmdGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: CmdGetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
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
        self.rawparams.clone()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeStatusObj {
    status: PyValue,
}

impl FakeStatusObj {
    fn new(status: PyValue) -> FakeStatusObj {
        FakeStatusObj { status }
    }
}

impl PrinterObject for FakeStatusObj {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status.clone())
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("gcode_macro fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping gcode_macro oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn pyvalue_to_record(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            // PyInt may exceed i64; render via decimal string when needed.
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_record).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_record).collect()),
        PyValue::Dict(pairs) => Value::Dict(
            pairs.iter().map(|(k, val)| (pyvalue_to_record(k), pyvalue_to_record(val))).collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host:{h}")),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "gcode_macro".to_string(),
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

fn check_str(label: &str, expected: pyoracle::OracleResult, actual: &str) {
    check(label, expected, Value::str(actual));
}

fn normalize_addr(s: &str) -> String {
    // CPython renders `ast.literal_eval` node reprs with a per-run
    // allocator address (`<_ast.Name object at 0x...>`); the port renders
    // `0x0`. Both are the same message modulo that address.
    let bytes = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'0'
            && i + 2 < bytes.len()
            && bytes[i + 1] == b'x'
            && bytes[i + 2].is_ascii_hexdigit()
        {
            let mut j = i + 3;
            while j < bytes.len() && bytes[j].is_ascii_hexdigit() {
                j += 1;
            }
            out.push_str("0xADDR");
            i = j;
        } else {
            out.push(bytes[i] as char);
            i += 1;
        }
    }
    out
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        normalize_addr(&actual.message()),
        normalize_addr(&oracle_exc.message),
        "{label}: exception message"
    );
}

// Rust world for one macro config.
struct RustWorld {
    printer: FakePrinter,
    gcode: Arc<FakeGCode>,
    webhooks: Arc<FakeWebHooks>,
    config: Config,
    macro_arc: Arc<GCodeMacro>,
}

fn rust_world(section: &str, options: Vec<(&str, &str)>, prev_handler_cmd: Option<&str>) -> Result<RustWorld, KlippyError> {
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    if let Some(cmd) = prev_handler_cmd {
        gcode.register_prev(cmd, "prev".to_string()).expect("prev handler");
    }
    let webhooks = Arc::new(FakeWebHooks::new(false));
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        Arc::new(printer.clone()),
        section,
        options.clone(),
    )));
    // NOTE: PrinterGCodeMacro built from a temporary config still carries
    // the same printer handle, which is all that matters here.
    let printer_arc: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("webhooks", webhooks.clone() as ObjectRef);
    printer.add("gcode_macro", pgm as ObjectRef);
    let config: Config = Arc::new(FakeConfig::new(printer_arc, section, options));
    let macro_arc = GCodeMacro::new(&config)?;
    Ok(RustWorld {
        printer,
        gcode,
        webhooks,
        config,
        macro_arc,
    })
}

// Python world constructor. Returns the factory expression for the macro
// object name (always `mac`).
fn py_world(py: &mut pyoracle::Oracle, section: &str, options: Vec<(&str, &str)>, script: &str, prev_handler_cmd: Option<&str>) {
    let opts = options.iter().map(|(k, v)| format!("'{k}': {v:?}")).collect::<Vec<_>>().join(", ");
    let opts_expr = format!("{{{opts}}}");
    let prev = match prev_handler_cmd {
        Some(cmd) => format!("gc.register_command('{cmd}', lambda g: None);"),
        None => String::new(),
    };
    let code = format!(
        "p, gc, wh, c, pgm = build_world({section:?}, {opts_expr}, {script:?}); {prev} mac = gcode_macro.load_config_prefix(c)"
    );
    py.exec(&code).expect("py world");
}

fn rust_world_ok(section: &str, options: Vec<(&str, &str)>) -> RustWorld {
    rust_world(section, options, None).expect("rust world")
}


// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn factory_shape_matches_python() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::gcode_macro::module_factory();
    assert!(factory.load_config.is_some(), "gcode_macro exposes load_config");
    assert!(
        factory.load_config_prefix.is_some(),
        "gcode_macro exposes load_config_prefix"
    );
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    // Python module has both entry points.
    let expected = py.eval("hasattr(gcode_macro, 'load_config') and hasattr(gcode_macro, 'load_config_prefix')");
    check("factory_shape_matches_python", expected, Value::Bool(true));
    let fp = FakePrinter::new();
    let printer: Printer = Arc::new(fp.clone());
    let gcode = Arc::new(FakeGCode::new());
    fp.add("gcode", gcode.clone() as ObjectRef);
    let config: Config = Arc::new(FakeConfig::new(printer.clone(), "gcode_macro", Vec::new()));
    let load_config = factory.load_config.expect("load_config");
    let obj = load_config(&config).expect("factory load_config").expect("object");
    let pgm: Arc<PrinterGCodeMacro> = klippy_api::downcast_arc(&obj).expect("downcast");
    // The factory object loads a working template.
    let cfg2: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "gcode_macro TEST",
        vec![("gcode", "M104 S{temp}\n"), ("variable_temp", "210")],
    ));
    let tw = pgm.load_template(&cfg2, "gcode", None).expect("factory load_template");
    check_str("factory template name", py.eval("'gcode_macro TEST:gcode'"), tw.name());
}

#[test]
fn macro_name_whitespace_rejected() {
    let Some(mut py) = oracle() else { return };
    py.exec("p0, gc0, wh0, c0, pgm0 = build_world('gcode_macro A B', None, 'G1\\n')").expect("setup");
    let expected = py.eval("gcode_macro.load_config_prefix(c0)");
    let _printer: Printer = Arc::new(FakePrinter::new());
    let gcode = Arc::new(FakeGCode::new());
    let fp = FakePrinter::new();
    let printer3: Printer = Arc::new(fp.clone());
    fp.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro A B",
        vec![("gcode", "G1\n")],
    )));
    fp.add("gcode_macro", pgm as ObjectRef);
    let config: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro A B",
        vec![("gcode", "G1\n")],
    ));
    let actual = GCodeMacro::new(&config).err().expect("must fail");
    check_err("macro_name_whitespace_rejected", expected, &actual);
    assert_eq!(actual.class_name(), "Error");
}

#[test]
fn command_registration_and_defaults() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "gcode_macro PRINT_START", vec![], "M104 S{temp}\n", None);
    let expected_calls = py.eval("gc.calls");
    let expected_mux = py.eval("gc.mux_calls");
    let world = rust_world_ok("gcode_macro PRINT_START", vec![("gcode", "M104 S{temp}\n")]);
    check("command_registration calls", expected_calls, Value::List(world.gcode.calls.borrow().iter().map(|s| Value::str(s)).collect()));
    check("command_registration mux", expected_mux, Value::List(world.gcode.mux_calls.borrow().iter().map(|s| Value::str(s)).collect()));
    assert_eq!(world.gcode.desc_of("PRINT_START").as_deref(), Some("G-Code macro"));
    let expected_help = py.eval("gcode_macro.GCodeMacro.cmd_SET_GCODE_VARIABLE_help");
    check_str("SET_GCODE_VARIABLE help", expected_help, SET_GCODE_VARIABLE_HELP);
}

#[test]
fn mixed_case_alias_and_mux_value() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "gcode_macro MyMacro", vec![], "G1\n", None);
    let expected = py.eval("gc.mux_calls");
    let world = rust_world_ok("gcode_macro MyMacro", vec![("gcode", "G1\n")]);
    check("mixed_case mux", expected, Value::List(world.gcode.mux_calls.borrow().iter().map(|s| Value::str(s)).collect()));
    assert_eq!(world.macro_arc.alias(), "MYMACRO");
    assert_eq!(world.macro_arc.name(), "MyMacro");
}

#[test]
fn custom_description_used() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        "gcode_macro CUSTOM_DESC",
        vec![("description", "My macro")],
        "G1\n",
        None,
    );
    let expected = py.eval("gc.calls");
    let world = rust_world_ok(
        "gcode_macro CUSTOM_DESC",
        vec![("gcode", "G1\n"), ("description", "My macro")],
    );
    check("custom_description calls", expected, Value::List(world.gcode.calls.borrow().iter().map(|s| Value::str(s)).collect()));
    assert_eq!(world.gcode.desc_of("CUSTOM_DESC").as_deref(), Some("My macro"));
}

#[test]
fn rename_type_mismatch_rejected() {
    let Some(mut py) = oracle() else { return };
    py.exec("pR, gcR, whR, cR, pgmR = build_world('gcode_macro G29', {'rename_existing': 'FOO'}, 'G1\\n')").expect("setup");
    let expected = py.eval("gcode_macro.load_config_prefix(cR)");
    let _printer: Printer = Arc::new(FakePrinter::new());
    let gcode = Arc::new(FakeGCode::new());
    let fp = FakePrinter::new();
    let printer3: Printer = Arc::new(fp.clone());
    fp.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro G29",
        vec![("gcode", "G1\n"), ("rename_existing", "FOO")],
    )));
    fp.add("gcode_macro", pgm as ObjectRef);
    let config: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro G29",
        vec![("gcode", "G1\n"), ("rename_existing", "FOO")],
    ));
    let actual = GCodeMacro::new(&config).err().expect("must fail");
    check_err("rename_type_mismatch_rejected", expected, &actual);
}

#[test]
fn rename_registers_connect_and_defers_command() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        "gcode_macro G29",
        vec![("rename_existing", "G28")],
        "G1\n",
        Some("G29"),
    );
    let expected_calls = py.eval("gc.calls");
    let expected_events = py.eval("[e for e, _ in p.event_handlers]");
    let world = rust_world("gcode_macro G29", vec![("gcode", "G1\n"), ("rename_existing", "G28")], Some("G29"))
        .expect("rust world");
    // Only the mux registration happened immediately; no alias command.
    check("rename deferred calls", expected_calls, Value::List(world.gcode.calls.borrow().iter().map(|s| Value::str(s)).collect()));
    check("rename event count", expected_events, Value::List(vec![Value::str("klippy:connect")]));
    assert_eq!(world.printer.handlers.borrow()[0].0, "klippy:connect");
}

#[test]
fn rename_connect_missing_command_error() {
    let Some(mut py) = oracle() else { return };
    // Pre-register nothing: handle_connect unregisters a missing command.
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro G29', {'rename_existing': 'G28'}, 'G1\\n'); mac = gcode_macro.load_config_prefix(c)")
        .expect("setup");
    let world = rust_world("gcode_macro G29", vec![("gcode", "G1\n"), ("rename_existing", "G28")], None)
        .expect("rust world");
    let actual = world.macro_arc.handle_connect().err().expect("must fail");
    // Re-run the python call to capture the exception.
    let expected = py.eval("mac.handle_connect()");
    check_err("rename_connect_missing_command_error", expected, &actual);
}

#[test]
fn rename_connect_reregisters() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro G29', {'rename_existing': 'G28'}, 'G1\\n'); gc.register_command('G29', lambda g: None); mac = gcode_macro.load_config_prefix(c); mac.handle_connect()")
        .expect("setup");
    let expected_calls = py.eval("gc.calls");
    let expected_descs = py.eval("sorted([k for k in gc.handlers])");
    let world = rust_world("gcode_macro G29", vec![("gcode", "G1\n"), ("rename_existing", "G28")], Some("G29"))
        .expect("rust world");
    world.macro_arc.handle_connect().expect("handle_connect");
    check("rename connect calls", expected_calls, Value::List(world.gcode.calls.borrow().iter().map(|s| Value::str(s)).collect()));
    let mut keys: Vec<_> = world.gcode.handlers.borrow().keys().cloned().collect();
    keys.sort();
    check("rename connect handlers", expected_descs, Value::List(keys.iter().map(|s| Value::str(s)).collect()));
    assert_eq!(
        world.gcode.desc_of("G28").as_deref(),
        Some("Renamed builtin of 'G29'")
    );
    assert_eq!(world.gcode.desc_of("G29").as_deref(), Some("G-Code macro"));
}

#[test]
fn variables_load_and_status() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        "gcode_macro TEST_VARS",
        vec![],
        "G1\n",
        None,
    );
    // Rebuild with variable options on both sides.
    py.exec("p2, gc2, wh2, c2, pgm2 = build_world('gcode_macro TEST_VARS', {'variable_int': '42', 'variable_float': '1.5', 'variable_str': \"'abc'\", 'variable_list': '[1, 2]', 'variable_dict': \"{'a': 1}\", 'variable_none': 'None', 'variable_bool': 'True', 'variable_tuple': '(1, 2)'}, 'G1\\n'); mac2 = gcode_macro.load_config_prefix(c2)")
        .expect("setup");
    let expected = py.eval("mac2.get_status(0.0)");
    let world = rust_world_ok(
        "gcode_macro TEST_VARS",
        vec![
            ("gcode", "G1\n"),
            ("variable_int", "42"),
            ("variable_float", "1.5"),
            ("variable_str", "'abc'"),
            ("variable_list", "[1, 2]"),
            ("variable_dict", "{'a': 1}"),
            ("variable_none", "None"),
            ("variable_bool", "True"),
            ("variable_tuple", "(1, 2)"),
        ],
    );
    let actual = PrinterObject::get_status(&*world.macro_arc, 0.0).expect("status");
    check("variables status", expected, pyvalue_to_record(&actual));
}

#[test]
fn variables_invalid_literal_rejected() {
    let Some(mut py) = oracle() else { return };
    // A bare name (`ValueError`), text that is not Python (`SyntaxError`),
    // a leading blank (`IndentationError`), an unclosed display (EOF), two
    // atoms in a row and mixed str/bytes concatenation.
    let cases = ["xx", "not a literal", " 1", "[1, 2", "1 2 3", "1 'a'", "'a' b''", "1)"];
    for text in cases {
        py.exec(&format!(
            "pB, gcB, whB, cB, pgmB = build_world('gcode_macro BAD_VAR', {{'variable_t': {text:?}}}, 'G1\\n')"
        ))
        .expect("setup");
        let expected = py.eval("gcode_macro.load_config_prefix(cB)");
        let printer = FakePrinter::new();
        let gcode = Arc::new(FakeGCode::new());
        let printer3: Printer = Arc::new(printer.clone());
        printer.add("gcode", gcode.clone() as ObjectRef);
        let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
            printer3.clone(),
            "gcode_macro BAD_VAR",
            vec![("gcode", "G1\n"), ("variable_t", text)],
        )));
        printer.add("gcode_macro", pgm as ObjectRef);
        let config: Config = Arc::new(FakeConfig::new(
            printer3,
            "gcode_macro BAD_VAR",
            vec![("gcode", "G1\n"), ("variable_t", text)],
        ));
        let actual = GCodeMacro::new(&config).err().expect("must fail");
        check_err(&format!("variables_invalid_literal_rejected {text:?}"), expected, &actual);
    }
}

#[test]
fn set_gcode_variable_updates() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro SET_TEST', {'variable_t': '42'}, 'G1\\n'); mac = gcode_macro.load_config_prefix(c); mac.cmd_SET_GCODE_VARIABLE(FakeGCommand('SET_GCODE_VARIABLE', 'SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=t VALUE=17', {'VARIABLE': 't', 'VALUE': '17'}, ' MACRO=SET_TEST VARIABLE=t VALUE=17'))")
        .expect("setup");
    let expected = py.eval("mac.get_status(0.0)");
    let world = rust_world_ok(
        "gcode_macro SET_TEST",
        vec![("gcode", "G1\n"), ("variable_t", "42")],
    );
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
        command: "SET_GCODE_VARIABLE".to_string(),
        commandline: "SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=t VALUE=17".to_string(),
        params: {
            let mut m = IndexMap::new();
            m.insert("VARIABLE".to_string(), "t".to_string());
            m.insert("VALUE".to_string(), "17".to_string());
            m
        },
        rawparams: " MACRO=SET_TEST VARIABLE=t VALUE=17".to_string(),
    });
    world.macro_arc.cmd_SET_GCODE_VARIABLE(&gcmd).expect("set var");
    let actual = PrinterObject::get_status(&*world.macro_arc, 0.0).expect("status");
    check("set_gcode_variable_updates", expected, pyvalue_to_record(&actual));
}

#[test]
fn set_gcode_variable_unknown() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro SET_TEST', {'variable_t': '42'}, 'G1\\n'); mac = gcode_macro.load_config_prefix(c)")
        .expect("setup");
    let expected = py.eval("mac.cmd_SET_GCODE_VARIABLE(FakeGCommand('SET_GCODE_VARIABLE', 'SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=nope VALUE=17', {'VARIABLE': 'nope', 'VALUE': '17'}, ''))");
    let world = rust_world_ok(
        "gcode_macro SET_TEST",
        vec![("gcode", "G1\n"), ("variable_t", "42")],
    );
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
        command: "SET_GCODE_VARIABLE".to_string(),
        commandline: "SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=nope VALUE=17".to_string(),
        params: {
            let mut m = IndexMap::new();
            m.insert("VARIABLE".to_string(), "nope".to_string());
            m.insert("VALUE".to_string(), "17".to_string());
            m
        },
        rawparams: String::new(),
    });
    let actual = world.macro_arc.cmd_SET_GCODE_VARIABLE(&gcmd).err().expect("must fail");
    check_err("set_gcode_variable_unknown", expected, &actual);
}

#[test]
fn set_gcode_variable_bad_literal() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro SET_TEST', {'variable_t': '42'}, 'G1\\n'); mac = gcode_macro.load_config_prefix(c)")
        .expect("setup");
    let expected = py.eval("mac.cmd_SET_GCODE_VARIABLE(FakeGCommand('SET_GCODE_VARIABLE', 'SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=t VALUE=xx', {'VARIABLE': 't', 'VALUE': 'xx'}, ''))");
    let world = rust_world_ok(
        "gcode_macro SET_TEST",
        vec![("gcode", "G1\n"), ("variable_t", "42")],
    );
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
        command: "SET_GCODE_VARIABLE".to_string(),
        commandline: "SET_GCODE_VARIABLE MACRO=SET_TEST VARIABLE=t VALUE=xx".to_string(),
        params: {
            let mut m = IndexMap::new();
            m.insert("VARIABLE".to_string(), "t".to_string());
            m.insert("VALUE".to_string(), "xx".to_string());
            m
        },
        rawparams: String::new(),
    });
    let actual = world.macro_arc.cmd_SET_GCODE_VARIABLE(&gcmd).err().expect("must fail");
    check_err("set_gcode_variable_bad_literal", expected, &actual);
}

#[test]
fn macro_cmd_render_with_vars_params() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro RENDER_TEST', {'variable_temp': '210'}, 'M104 S{temp} {params.SPEED}\\n'); mac = gcode_macro.load_config_prefix(c); mac.cmd(FakeGCommand('RENDER_TEST', 'RENDER_TEST SPEED=100', {'SPEED': '100'}, ' SPEED=100'))")
        .expect("setup");
    let expected = py.eval("gc.scripts");
    let world = rust_world_ok(
        "gcode_macro RENDER_TEST",
        vec![("gcode", "M104 S{temp} {params.SPEED}\n"), ("variable_temp", "210")],
    );
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
        command: "RENDER_TEST".to_string(),
        commandline: "RENDER_TEST SPEED=100".to_string(),
        params: {
            let mut m = IndexMap::new();
            m.insert("SPEED".to_string(), "100".to_string());
            m
        },
        rawparams: " SPEED=100".to_string(),
    });
    world.macro_arc.cmd(&gcmd).expect("cmd");
    check("macro_cmd_render", expected, Value::List(world.gcode.scripts().iter().map(|s| Value::str(s)).collect()));
}

/// The render-time `UndefinedError` a macro raises, as Klipper wraps it
/// (`Error evaluating '<macro>': jinja2.exceptions.UndefinedError: ...`).
#[test]
fn macro_cmd_undefined_error_text() {
    let Some(mut py) = oracle() else { return };
    let scripts = [
        "M104 S{params.TEMP|float}\n",
        "M104 S{params.TEMP + 1}\n",
        "G1 X{printer.toolhead.position[7] + 1}\n",
        "G1 X{printer.nosuch.x}\n",
        "G1 X{(1 if params.TEMP) + 1}\n",
    ];
    for script in scripts {
        py.exec(&format!(
            "p, gc, wh, c, pgm = build_world('gcode_macro UNDEF', None, {script:?}); \
             p.objects['toolhead'] = FakeStatusObj({{'position': [1.0, 2.0]}}); \
             mac = gcode_macro.load_config_prefix(c)"
        ))
        .expect("setup");
        let expected = py.eval("mac.cmd(FakeGCommand('UNDEF', 'UNDEF SPEED=1', {'SPEED': '1'}, ' SPEED=1'))");
        let world = rust_world_ok("gcode_macro UNDEF", vec![("gcode", script)]);
        let mut toolhead: IndexMap<PyValue, PyValue> = IndexMap::new();
        toolhead.insert(
            PyValue::Str("position".to_string()),
            PyValue::List(vec![PyValue::Float(1.0), PyValue::Float(2.0)]),
        );
        world.printer.add(
            "toolhead",
            Arc::new(FakeStatusObj::new(PyValue::Dict(toolhead))) as ObjectRef,
        );
        let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
            command: "UNDEF".to_string(),
            commandline: "UNDEF SPEED=1".to_string(),
            params: {
                let mut m = IndexMap::new();
                m.insert("SPEED".to_string(), "1".to_string());
                m
            },
            rawparams: " SPEED=1".to_string(),
        });
        let actual = world.macro_arc.cmd(&gcmd).expect_err("render must fail");
        check_err(script, expected, &actual);
    }
}

#[test]
fn macro_recursive_guard() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro RECUR_TEST', None, 'RECUR_TEST\\n'); mac = gcode_macro.load_config_prefix(c); mac.in_script = True")
        .expect("setup");
    let expected = py.eval("mac.cmd(FakeGCommand('RECUR_TEST', 'RECUR_TEST', {}, ' RECUR_TEST'))");
    let world = rust_world_ok("gcode_macro RECUR_TEST", vec![("gcode", "RECUR_TEST\n")]);
    world.macro_arc.set_in_script(true);
    let gcmd: klippy_api::gcode::GCmd = Arc::new(FakeGCommand {
        command: "RECUR_TEST".to_string(),
        commandline: "RECUR_TEST".to_string(),
        params: IndexMap::new(),
        rawparams: " RECUR_TEST".to_string(),
    });
    let actual = world.macro_arc.cmd(&gcmd).err().expect("must fail");
    check_err("macro_recursive_guard", expected, &actual);
    // The guard releases after a failed run: a second failure is recursion again.
    let actual2 = world.macro_arc.cmd(&gcmd).err().expect("still recursive");
    assert_eq!(actual2.message(), "Macro RECUR_TEST called recursively");
}

#[test]
fn template_syntax_error_is_command_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro SYNTAX', None, 'G1\\n')").expect("setup");
    let expected = py.eval("pgm.load_template(FakeConfig(p, 'gcode_macro SYNTAX', {'gcode': '{% if x %}G1'}), 'gcode')");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro SYNTAX",
        vec![("gcode", "{% if x %}G1")],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro SYNTAX",
        vec![("gcode", "{% if x %}G1")],
    ));
    let actual = pgm.load_template(&cfg, "gcode", None).err().expect("must fail");
    check_err("template_syntax_error_is_command_error", expected, &actual);
}

#[test]
fn template_evaluate_error_is_command_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro EVAL_ERR', None, 'explicit\\n'); tw = pgm.load_template(FakeConfig(p, 'gcode_macro EVAL_ERR', {'gcode': \"{ action_raise_error('boom') }\"}), 'gcode')").expect("setup");
    let expected = py.eval("tw.render()");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro EVAL_ERR",
        vec![("gcode", "{ action_raise_error('boom') }")],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro EVAL_ERR",
        vec![("gcode", "{ action_raise_error('boom') }")],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    let actual = tw.render(None).err().expect("must fail");
    check_err("template_evaluate_error_is_command_error", expected, &actual);
}

#[test]
fn action_respond_info_routes() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro RESPOND', None, \"{ action_respond_info('hi') }\\nG1\\n\"); tw = pgm.load_template(c, 'gcode')")
        .expect("setup");
    let expected_script = py.eval("tw.render()");
    let expected_responds = py.eval("gc.responds");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro RESPOND",
        vec![("gcode", "{ action_respond_info('hi') }\nG1\n")],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro RESPOND",
        vec![("gcode", "{ action_respond_info('hi') }\nG1\n")],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    let script = tw.render(None).expect("render");
    check_str("action_respond_info script", expected_script, &script);
    check("action_respond_info responds", expected_responds, Value::List(gcode.responds().iter().map(|s| Value::str(s)).collect()));
}

#[test]
fn action_emergency_stop_invokes_shutdown() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro STOP_TEST', None, \"{ action_emergency_stop('stopped') }\"); tw = pgm.load_template(c, 'gcode'); tw.render()").expect("setup");
    let expected = py.eval("p.shutdowns");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro STOP_TEST",
        vec![("gcode", "{ action_emergency_stop('stopped') }")],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro STOP_TEST",
        vec![("gcode", "{ action_emergency_stop('stopped') }")],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    tw.render(None).expect("render");
    check("action_emergency_stop shutdowns", expected, Value::List(printer.shutdowns().iter().map(|s| Value::str(s)).collect()));
}

#[test]
fn action_call_remote_method_calls_and_swallows() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro REMOTE', None, \"{ action_call_remote_method('do_thing', x=1) }\"); wh.fail_commands = True; tw = pgm.load_template(c, 'gcode'); tw.render()").expect("setup");
    let expected_calls = py.eval("wh.calls");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let wh = Arc::new(FakeWebHooks::new(true));
    printer.add("webhooks", wh.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro REMOTE",
        vec![("gcode", "{ action_call_remote_method('do_thing', x=1) }")],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro REMOTE",
        vec![("gcode", "{ action_call_remote_method('do_thing', x=1) }")],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    let script = tw.render(None).expect("swallowed command error still renders");
    let expected_script = py.eval("tw.render()");
    check_str("remote swallowed script", expected_script, &script);
    check("remote swallowed calls", expected_calls, Value::List(wh.calls().iter().map(|s| Value::str(s)).collect()));
}

#[test]
fn create_template_context_printer_status() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro CTX', None, \"{ printer['status_obj']['value'] }\\n\"); p.objects['status_obj'] = FakeStatusObj({'value': 7}); tw = pgm.load_template(c, 'gcode')").expect("setup");
    let expected = py.eval("tw.render()");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    d.insert(PyValue::Str("value".to_string()), PyValue::Int(pycompat::int::PyInt::from_i64(7)));
    printer.add("status_obj", Arc::new(FakeStatusObj::new(PyValue::Dict(d))) as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro CTX",
        vec![],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3.clone(),
        "gcode_macro CTX",
        vec![("gcode", "{ printer['status_obj']['value'] }\n")],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    let script = tw.render(None).expect("render");
    check_str("create_template_context printer status", expected, &script);
    // Context keys in Python order.
    let ctx = pgm.create_template_context(Some(0.0));
    let keys: Vec<String> = ctx.keys().cloned().collect();
    assert_eq!(
        keys,
        vec![
            "printer",
            "action_emergency_stop",
            "action_respond_info",
            "action_raise_error",
            "action_call_remote_method"
        ]
    );
    let ctx2 = create_template_context(&printer3, Some(0.0));
    assert_eq!(ctx2.keys().cloned().collect::<Vec<_>>(), keys);
}

#[test]
fn load_template_default_used() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro DEF', None, 'G1\\n'); tw = pgm.load_template(FakeConfig(p, 'some_section', {}), 'gcode', 'G4 P0\\n'); tw.render()").expect("setup");
    let expected = py.eval("'G4 P0'");
    // The rendered script has no trailing newline in both engines.
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro DEF",
        vec![],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(printer3.clone(), "some_section", vec![]));
    let tw = pgm.load_template(&cfg, "gcode", Some("G4 P0\n")).expect("load default");
    let script = tw.render(None).expect("render default");
    check_str("load_template default", expected, &script);
}

#[test]
fn template_wrapper_name_and_source() {
    let Some(mut py) = oracle() else { return };
    py.exec("p, gc, wh, c, pgm = build_world('gcode_macro NAMED', None, 'M104 S{temp}\\n'); tw = pgm.load_template(c, 'gcode')").expect("setup");
    let expected_name = py.eval("tw.name");
    let expected_source = py.eval("c.get('gcode')");
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", gcode.clone() as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro NAMED",
        vec![],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro NAMED",
        vec![("gcode", "M104 S{temp}\n")],
    ));
    let tw: TemplateWrapper = pgm.load_template(&cfg, "gcode", None).expect("load");
    check_str("wrapper name", expected_name, tw.name());
    check_str("wrapper source", expected_source, tw.source());
}

#[test]
fn rename_connect_event_handler_used() {
    // The klippy:connect handler registered at construction performs the
    // same rename as a direct handle_connect call.
    let world = rust_world(
        "gcode_macro G29",
        vec![("gcode", "G1\n"), ("rename_existing", "G28")],
        Some("G29"),
    )
    .expect("rust world");
    assert_eq!(world.printer.handler_count(), 1);
    let res = world.printer.fire_handler(0, &[]).expect("handler runs");
    assert!(matches!(res, PyValue::None));
    assert!(world.gcode.handlers.borrow().contains_key("G28"));
    assert_eq!(
        world.gcode.desc_of("G28").as_deref(),
        Some("Renamed builtin of 'G29'")
    );
}

// ---------------------------------------------------------------------------
// `GetStatusWrapper` laziness: which objects' `get_status` a render calls.
// ---------------------------------------------------------------------------

const STATUS_SETUP: &str = r#"
class CountingStatusObj:
    def __init__(self, status):
        self.status = status
        self.eventtimes = []
    def get_status(self, eventtime):
        self.eventtimes.append(eventtime)
        return dict(self.status)

class PlainObj:
    pass

def run_status_case(script, renders):
    p, gc, wh, c, pgm = build_world('gcode_macro CNT', None, script)
    p.objects['obj_a'] = CountingStatusObj({'value': 1})
    p.objects['obj_b'] = CountingStatusObj({'value': 2})
    p.objects['plain'] = PlainObj()
    tw = pgm.load_template(c, 'gcode')
    ctx = pgm.create_template_context(5.0)
    outs = []
    for _ in range(renders):
        try:
            outs.append(tw.render(ctx))
        except Exception as e:
            outs.append('%s: %s' % (type(e).__name__, e))
    return [outs, p.objects['obj_a'].eventtimes, p.objects['obj_b'].eventtimes]
"#;

/// A status object recording the eventtime of every `get_status` call.
struct CountingStatusObj {
    status: PyValue,
    eventtimes: GCell<Vec<f64>>,
}

impl PrinterObject for CountingStatusObj {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        self.eventtimes.borrow_mut().push(eventtime);
        Some(self.status.clone())
    }
}

/// A printer object without `get_status`.
struct PlainObj;
klippy_api::printer_object!(PlainObj);

fn status_dict(value: i64) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    d.insert(
        PyValue::Str("value".to_string()),
        PyValue::Int(pycompat::int::PyInt::from_i64(value)),
    );
    PyValue::Dict(d)
}

/// `run_status_case` on the port: the same objects in the same order, one
/// context (`create_template_context(5.0)`) shared by `renders` renders.
fn rust_status_case(script: &str, renders: usize) -> Value {
    let printer = FakePrinter::new();
    let printer3: Printer = Arc::new(printer.clone());
    printer.add("gcode", Arc::new(FakeGCode::new()) as ObjectRef);
    printer.add("webhooks", Arc::new(FakeWebHooks::new(false)) as ObjectRef);
    let pgm = Arc::new(PrinterGCodeMacro::new(&cfg_of(
        printer3.clone(),
        "gcode_macro CNT",
        vec![],
    )));
    printer.add("gcode_macro", pgm.clone() as ObjectRef);
    let counters: Vec<Arc<CountingStatusObj>> = [1, 2]
        .into_iter()
        .map(|v| {
            Arc::new(CountingStatusObj {
                status: status_dict(v),
                eventtimes: GCell::new(Vec::new()),
            })
        })
        .collect();
    printer.add("obj_a", counters[0].clone() as ObjectRef);
    printer.add("obj_b", counters[1].clone() as ObjectRef);
    printer.add("plain", Arc::new(PlainObj) as ObjectRef);
    let cfg: Config = Arc::new(FakeConfig::new(
        printer3,
        "gcode_macro CNT",
        vec![("gcode", script)],
    ));
    let tw = pgm.load_template(&cfg, "gcode", None).expect("load");
    let ctx = pgm.create_template_context(Some(5.0));
    let outs = (0..renders)
        .map(|_| match tw.render(Some(ctx.clone())) {
            Ok(s) => Value::str(&s),
            Err(e) => Value::str(&format!("{}: {}", e.class_name(), e.message())),
        })
        .collect();
    let times = |c: &CountingStatusObj| {
        Value::List(c.eventtimes.borrow().iter().map(|&t| Value::float(t)).collect())
    };
    Value::List(vec![Value::List(outs), times(&counters[0]), times(&counters[1])])
}

#[test]
fn get_status_wrapper_calls_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(STATUS_SETUP).expect("status fakes");
    let cases: [(&str, &str, usize); 10] = [
        ("untouched", "G1 X0\n", 1),
        ("one object", "{ printer.obj_a.value }\n", 1),
        (
            "same object twice",
            "{ printer.obj_a.value }{ printer['obj_a'].value }{ printer[' obj_a '].value }\n",
            1,
        ),
        ("iteration", "{% for n in printer %}{n},{% endfor %}\n", 1),
        (
            "in",
            "{ 'obj_b' in printer }{ 'nosuch' in printer }{ 'plain' in printer }\n",
            1,
        ),
        ("missing object", "{ printer.nosuch|default('missing') }\n", 1),
        ("missing object used", "{ printer.nosuch.value }\n", 1),
        ("missing element", "{ printer[3]|default('missing') }{ printer[3] + 1 }\n", 1),
        ("object without get_status", "{ printer.plain|default('none') }\n", 1),
        ("shared context", "{ printer.obj_a.value }{ printer.obj_b.value }\n", 2),
    ];
    for (label, script, renders) in cases {
        let expected = py.eval(&format!("run_status_case({script:?}, {renders})"));
        check(label, expected, rust_status_case(script, renders));
    }
}
