//! Parity tests for `extras::firmware_retraction`
//! (port of `klippy/extras/firmware_retraction.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `FirmwareRetraction`
//! is built on both sides with minimal Python / Rust fakes for the
//! printer, config and G-code services, fed identical config values and
//! G-code parameters, and the results are compared exactly (floats by
//! bits via the recording value format):
//! - config options read, with defaults and minvals,
//! - G-code commands registered, with help strings,
//! - `get_status` dicts,
//! - `GET_RETRACTION` response messages,
//! - `G10` / `G11` scripts passed to `run_script_from_command`,
//! - raised errors (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone. When `$KLIPPY_PYTHON` is unset the tests skip with a
//! message.
//!
//! Known divergence: `GetFloatOpts.minval` is modelled as `f64`, so the
//! int minval `1` in Python renders as `"1"` in error messages while
//! Rust renders `"1.0"`. Error comparisons normalize `1.0` to `1` on
//! the Rust side; the float minval `0.` renders `"0.0"` on both sides
//! and is compared verbatim. The production `core` impls have the same
//! rendering.

use extras::firmware_retraction::{FirmwareRetraction, module_factory};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetIntListOpts,
    GetFloatListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi, EventCallback};
use pycompat::text::{float_from_string, float_str, int_from_string};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

use kos::sync::Mutex;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.firmware_retraction as firmware_retraction

class Error(Exception):
    pass

class CommandError(Exception):
    pass

class _Sentinel:
    pass

sentinel = _Sentinel()

class FakeGCodeCommand:
    error = CommandError
    def __init__(self, gcode, command, commandline, params):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
        self.respond_info = gcode.respond_info
        self.respond_raw = gcode.respond_raw
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return self._params
    def get(self, name, default=sentinel, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
        except:
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
    def get_int(self, name, default=sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)
    def get_float(self, name, default=sentinel, minval=None, maxval=None, above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval, above=above, below=below)

class FakeGCode:
    def __init__(self):
        self.registered = []
        self.scripts = []
        self.infos = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append((cmd, desc))
        return None
    def run_script_from_command(self, script):
        self.scripts.append(script)
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))
    def respond_raw(self, msg):
        pass

class FakePrinter:
    def __init__(self, gcode):
        self._gcode = gcode
    def lookup_object(self, name, default=sentinel):
        if name == 'gcode':
            return self._gcode
        if default is sentinel:
            raise Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    def __init__(self, printer, section, values):
        self._printer = printer
        self.section = section
        self.values = dict(values)
        self.reads = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        self.reads.append((option, default, minval))
        if option not in self.values:
            if default is sentinel:
                raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        v = float(self.values[option])
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v

gcode = None
printer = None
config = None
fr = None

def reset(values=None):
    global gcode, printer, config, fr
    gcode = FakeGCode()
    printer = FakePrinter(gcode)
    config = FakeConfig(printer, 'firmware_retraction', values or {})
    fr = firmware_retraction.FirmwareRetraction(config)

def py_status():
    return fr.get_status(0.)

def py_registered():
    return [[c, d] for (c, d) in gcode.registered]

def py_reads():
    return [[o, (float(d) if d is not None else None), (float(m) if m is not None else None)] for (o, d, m) in config.reads]

def py_scripts():
    return list(gcode.scripts)

def py_infos():
    return [[m, l] for (m, l) in gcode.infos]

def do_set(params):
    gcmd = FakeGCodeCommand(gcode, 'SET_RETRACTION', 'SET_RETRACTION', params)
    return fr.cmd_SET_RETRACTION(gcmd)

def do_get():
    gcmd = FakeGCodeCommand(gcode, 'GET_RETRACTION', 'GET_RETRACTION', {})
    return fr.cmd_GET_RETRACTION(gcmd)

def do_g10():
    gcmd = FakeGCodeCommand(gcode, 'G10', 'G10', {})
    return fr.cmd_G10(gcmd)

def do_g11():
    gcmd = FakeGCodeCommand(gcode, 'G11', 'G11', {})
    return fr.cmd_G11(gcmd)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[allow(clippy::type_complexity)]
struct Shared {
    registered: Mutex<Vec<(String, Option<String>)>>,
    scripts: Mutex<Vec<String>>,
    infos: Mutex<Vec<(String, bool)>>,
}

impl Shared {
    fn new() -> Shared {
        Shared {
            registered: Mutex::new(Vec::new()),
            scripts: Mutex::new(Vec::new()),
            infos: Mutex::new(Vec::new()),
        }
    }
}

fn cmd_error(msg: String) -> KlippyError {
    KlippyError::Command(msg)
}

fn config_error(msg: String) -> KlippyError {
    KlippyError::Config(msg)
}

struct FakeGCode {
    shared: Arc<Shared>,
    handlers: Mutex<Vec<(String, GCodeCommandHandler)>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        let g = FakeGCode {
            shared: Arc::new(Shared::new()),
            handlers: Mutex::new(Vec::new()),
            reactor: Reactor::new(false),
        };
        Arc::new(g)
    }

    fn registered(&self) -> Vec<(String, Option<String>)> {
        self.shared.registered.lock().clone()
    }

    fn scripts(&self) -> Vec<String> {
        self.shared.scripts.lock().clone()
    }

    fn infos(&self) -> Vec<(String, bool)> {
        self.shared.infos.lock().clone()
    }

    fn dispatch(&self, command: &str, params: &[(&str, &str)]) -> KResult<()> {
        let gcmd = self.make_gcmd(command, params);
        let mut guard = self.handlers.lock();
        for (name, handler) in guard.iter_mut() {
            if name == command {
                return handler(&gcmd);
            }
        }
        Err(cmd_error(format!("No handler for command '{command}'")))
    }

    fn make_gcmd(&self, command: &str, params: &[(&str, &str)]) -> GCmd {
        Arc::new(FakeGCmd {
            shared: Arc::clone(&self.shared),
            command: command.to_string(),
            commandline: command.to_string(),
            params: params
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        })
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.shared
            .registered
            .lock()
            .push((cmd.to_string(), opts.desc.clone()));
        match func {
            Some(handler) => {
                self.handlers.lock().push((cmd.to_string(), handler));
                Ok(None)
            }
            None => Ok(None),
        }
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
        PyValue::Dict(IndexMap::new())
    }

    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}

    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }

    fn run_script(&self, script: &str) -> KResult<()> {
        self.shared.scripts.lock().push(script.to_string());
        Ok(())
    }

    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.shared.scripts.lock().push(script.to_string());
        Ok(())
    }

    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }

    fn create_gcode_command(
        &self,
        command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        self.make_gcmd(command, &[])
    }

    fn respond_raw(&self, _msg: &str) {}

    fn respond_info(&self, msg: &str, log: bool) {
        self.shared.infos.lock().push((msg.to_string(), log));
    }

    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }

    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }

    fn process_commands(
        &self,
        _commands: &[String],
        _need_ack: bool,
    ) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    shared: Arc<Shared>,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn missing(name: &str, commandline: &str) -> KlippyError {
        cmd_error(format!("Error on '{commandline}': missing {name}"))
    }

    fn unable(commandline: &str, value: &str) -> KlippyError {
        cmd_error(format!("Error on '{commandline}': unable to parse {value}"))
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        let raw = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &self.commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let value: PyValue = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int | GetParser::IntAutoBase => {
                let base = if opts.parser == GetParser::IntAutoBase { 0 } else { 10 };
                match int_from_string(&raw, base) {
                    Ok(n) => PyValue::Int(n),
                    Err(_) => return Err(Self::unable(&self.commandline, &raw)),
                }
            }
            GetParser::Float => match float_from_string(&raw) {
                Ok(f) => PyValue::Float(f),
                Err(_) => return Err(Self::unable(&self.commandline, &raw)),
            },
        };
        // Bound checks via ordering on the numeric/string value.
        let violates = |bound: &Option<PyValue>, kind: &str| -> Option<String> {
            let b = bound.as_ref()?;
            let bad = match kind {
                "min" => pycompat::value::py_rich_compare("<", &value, b).unwrap_or(false),
                "max" => pycompat::value::py_rich_compare(">", &value, b).unwrap_or(false),
                "above" => pycompat::value::py_rich_compare("<=", &value, b).unwrap_or(false),
                "below" => pycompat::value::py_rich_compare(">=", &value, b).unwrap_or(false),
                _ => false,
            };
            if bad {
                Some(format!(
                    "Error on '{}': {} must have {} {}",
                    self.commandline,
                    name,
                    match kind {
                        "min" => "minimum of",
                        "max" => "maximum of",
                        "above" => "above",
                        _ => "below",
                    },
                    b.str()
                ))
            } else {
                None
            }
        };
        if let Some(m) = violates(&opts.minval, "min") {
            return Err(cmd_error(m));
        }
        if let Some(m) = violates(&opts.maxval, "max") {
            return Err(cmd_error(m));
        }
        if let Some(m) = violates(&opts.above, "above") {
            return Err(cmd_error(m));
        }
        if let Some(m) = violates(&opts.below, "below") {
            return Err(cmd_error(m));
        }
        Ok(value)
    }

    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &self.commandline)),
                    d => Ok(d.flatten()),
                };
            }
            Some(v) => v.clone(),
        };
        let parsed = match int_from_string(&raw, 10).and_then(|n| {
            n.0.to_string().parse::<i64>().map_err(|_| pycompat::text::PyError::new("ValueError", "int out of range".to_string()))
        }) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&self.commandline, &raw)),
        };
        if let Some(mn) = opts.minval {
            if parsed < mn {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    mn
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if parsed > mx {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    mx
                )));
            }
        }
        Ok(Some(parsed))
    }

    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &self.commandline)),
                    d => Ok(d.flatten()),
                };
            }
            Some(v) => v.clone(),
        };
        let parsed = match float_from_string(&raw) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&self.commandline, &raw)),
        };
        if let Some(mn) = opts.minval {
            if parsed < mn {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    float_str(mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if parsed > mx {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    float_str(mx)
                )));
            }
        }
        if let Some(ab) = opts.above {
            if parsed <= ab {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    float_str(ab)
                )));
            }
        }
        if let Some(be) = opts.below {
            if parsed >= be {
                return Err(cmd_error(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    float_str(be)
                )));
            }
        }
        Ok(Some(parsed))
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
        self.shared.infos.lock().push((msg.to_string(), log));
    }

    fn respond_raw(&self, _msg: &str) {}

    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakePrinter {
    gcode: ObjectRef,
}

impl FakePrinter {
    fn new(gcode: Arc<FakeGCode>) -> Arc<FakePrinter> {
        Arc::new(FakePrinter { gcode })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
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

    fn update_error_msg(&self, _old: &str, _new: &str) {}

    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }

    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if name == "gcode" {
            Ok(Some(Arc::clone(&self.gcode)))
        } else {
            match default {
                LookupDefault::Raise => Err(config_error(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            }
        }
    }

    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        vec![("gcode".to_string(), Arc::clone(&self.gcode))]
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

    fn register_event_handler(&self, _event: &str, _cb: EventCallback) {}

    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }

    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    values: IndexMap<String, f64>,
    reads: Mutex<Vec<(String, Option<Option<f64>>, Option<f64>)>>,
}

impl FakeConfig {
    fn new(
        printer: Printer,
        values: &[(&str, f64)],
    ) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer,
            section: "firmware_retraction".to_string(),
            values: values
                .iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect(),
            reads: Mutex::new(Vec::new()),
        })
    }

    fn reads(&self) -> Vec<(String, Option<Option<f64>>, Option<f64>)> {
        self.reads.lock().clone()
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }

    fn get_name(&self) -> String {
        self.section.clone()
    }

    fn get(&self, _option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(match opts.default {
            None => None,
            Some(d) => d,
        })
    }

    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match opts.default {
            None => Err(config_error(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
            Some(d) => Ok(d),
        }
    }

    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads
            .lock()
            .push((option.to_string(), opts.default, opts.minval));
        match self.values.get(option) {
            None => match opts.default {
                None => Err(config_error(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
                d => Ok(d.flatten()),
            },
            Some(&v) => {
                if let Some(mn) = opts.minval {
                    if v < mn {
                        return Err(config_error(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            float_str(mn)
                        )));
                    }
                }
                if let Some(mx) = opts.maxval {
                    if v > mx {
                        return Err(config_error(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            float_str(mx)
                        )));
                    }
                }
                if let Some(ab) = opts.above {
                    if v <= ab {
                        return Err(config_error(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            float_str(ab)
                        )));
                    }
                }
                if let Some(be) = opts.below {
                    if v >= be {
                        return Err(config_error(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            float_str(be)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }

    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(match opts.default {
            None => None,
            Some(d) => d,
        })
    }

    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }

    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::Tuple(Vec::new()))
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
        // Never called by this module; return self as a placeholder
        // section view (the test only reads through the root config).
        // Production code does not use subsections here.
        let other = FakeConfig {
            printer: Arc::clone(&self.printer),
            section: _section.to_string(),
            values: self.values.clone(),
            reads: Mutex::new(Vec::new()),
        };
        Arc::new(other)
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

/// Convert a `PyValue` status dict to the recording model.
fn to_rec(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(to_rec).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(to_rec).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (to_rec(k), to_rec(v)))
                .collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("firmware_retraction fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping firmware_retraction oracle tests: set KLIPPY_PYTHON to a \
                 CPython 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn check(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Value,
) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.firmware_retraction".to_string(),
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

/// Normalize the known int-minval rendering divergence (`"1"` vs `"1.0"`).
fn norm_minval(message: &str) -> String {
    message.replace("must have minimum of 1.0", "must have minimum of 1")
}

/// Assert the Rust error equals the oracle exception (class exact,
/// message exact modulo the documented int-minval rendering).
fn check_error(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: &KlippyError,
) {
    match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected error {}", actual.class_name()),
        Err(e) => {
            assert_eq!(
                actual.class_name(),
                e.class_name,
                "{label}: error class for '{}'",
                actual.message()
            );
            assert_eq!(
                norm_minval(actual.message()),
                e.message,
                "{label}: error message"
            );
        }
    }
}

fn status_value(fr: &Arc<FirmwareRetraction>) -> Value {
    to_rec(&fr.get_status(0.0).expect("get_status"))
}

fn registered_value(g: &Arc<FakeGCode>) -> Value {
    Value::List(
        g.registered()
            .into_iter()
            .map(|(cmd, desc)| match desc {
                Some(d) => Value::List(vec![Value::Str(cmd), Value::Str(d)]),
                None => Value::List(vec![Value::Str(cmd), Value::Null]),
            })
            .collect(),
    )
}

fn reads_value(c: &Arc<FakeConfig>) -> Value {
    Value::List(
        c.reads()
            .into_iter()
            .map(|(opt, def, min)| {
                Value::List(vec![
                    Value::Str(opt),
                    match def {
                        Some(Some(f)) => Value::float(f),
                        Some(None) => Value::Null,
                        None => Value::Null,
                    },
                    match min {
                        Some(f) => Value::float(f),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

fn scripts_value(scripts: &[String]) -> Value {
    Value::List(scripts.iter().map(|s| Value::Str(s.clone())).collect())
}

fn infos_value(infos: &[(String, bool)]) -> Value {
    Value::List(
        infos
            .iter()
            .map(|(m, l)| Value::List(vec![Value::Str(m.clone()), Value::Bool(*l)]))
            .collect(),
    )
}

struct Fixture {
    gcode: Arc<FakeGCode>,
    config: Arc<FakeConfig>,
    fr: Arc<FirmwareRetraction>,
}

fn fixture(values: &[(&str, f64)]) -> Fixture {
    let gcode = FakeGCode::new();
    let printer = FakePrinter::new(Arc::clone(&gcode));
    let config = FakeConfig::new(printer, values);
    let cfg: Config = config.clone();
    let fr = FirmwareRetraction::new(&cfg).expect("FirmwareRetraction::new");
    Fixture { gcode, config, fr }
}


// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn defaults_and_registrations_match_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[]);
    py.exec("reset()").expect("reset");
    check(
        "defaults: registered",
        py.eval("py_registered()"),
        registered_value(&fix.gcode),
    );
    check("defaults: reads", py.eval("py_reads()"), reads_value(&fix.config));
    check("defaults: status", py.eval("py_status()"), status_value(&fix.fr));
    let _ = fix;
}

#[test]
fn custom_config_status_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[
        ("retract_length", 2.5),
        ("retract_speed", 45.0),
        ("unretract_extra_length", 0.5),
        ("unretract_speed", 33.0),
    ]);
    py.exec(
        "reset({'retract_length': 2.5, 'retract_speed': 45.0, \
         'unretract_extra_length': 0.5, 'unretract_speed': 33.0})",
    )
    .expect("reset");
    check(
        "custom: registered",
        py.eval("py_registered()"),
        registered_value(&fix.gcode),
    );
    check("custom: reads", py.eval("py_reads()"), reads_value(&fix.config));
    check("custom: status", py.eval("py_status()"), status_value(&fix.fr));
}

#[test]
fn get_retraction_message_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[
        ("retract_length", 2.123456),
        ("retract_speed", 45.0),
        ("unretract_extra_length", 0.5),
        ("unretract_speed", 33.0),
    ]);
    py.exec(
        "reset({'retract_length': 2.123456, 'retract_speed': 45.0, \
         'unretract_extra_length': 0.5, 'unretract_speed': 33.0})",
    )
    .expect("reset");
    fix.gcode.dispatch("GET_RETRACTION", &[]).expect("dispatch");
    py.eval("do_get()").expect("do_get");
    check(
        "get: infos",
        py.eval("py_infos()"),
        infos_value(&fix.gcode.infos()),
    );
    // State is not changed by GET.
    check("get: status", py.eval("py_status()"), status_value(&fix.fr));
}

#[test]
fn set_retraction_updates_and_resets_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[]);
    py.exec("reset()").expect("reset");
    let params = [
        ("RETRACT_LENGTH", "1.5"),
        ("RETRACT_SPEED", "30"),
        ("UNRETRACT_EXTRA_LENGTH", "0.25"),
        ("UNRETRACT_SPEED", "15"),
    ];
    fix.gcode.dispatch("SET_RETRACTION", &params).expect("dispatch");
    py.eval("do_set({'RETRACT_LENGTH': '1.5', 'RETRACT_SPEED': '30', 'UNRETRACT_EXTRA_LENGTH': '0.25', 'UNRETRACT_SPEED': '15'})")
        .expect("do_set");
    check("set: status", py.eval("py_status()"), status_value(&fix.fr));
    // GET after SET reflects the new values.
    fix.gcode.dispatch("GET_RETRACTION", &[]).expect("dispatch get");
    py.eval("do_get()").expect("do_get");
    check(
        "set-then-get: infos",
        py.eval("py_infos()"),
        infos_value(&fix.gcode.infos()),
    );
}

#[test]
fn set_retraction_partial_params_keep_current_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[
        ("retract_length", 3.0),
        ("retract_speed", 50.0),
        ("unretract_extra_length", 1.0),
        ("unretract_speed", 40.0),
    ]);
    py.exec(
        "reset({'retract_length': 3.0, 'retract_speed': 50.0, \
         'unretract_extra_length': 1.0, 'unretract_speed': 40.0})",
    )
    .expect("reset");
    // Only RETRACT_LENGTH is supplied; the rest keep current values.
    fix.gcode
        .dispatch("SET_RETRACTION", &[("RETRACT_LENGTH", "4.5")])
        .expect("dispatch");
    py.eval("do_set({'RETRACT_LENGTH': '4.5'})").expect("do_set");
    check("partial: status", py.eval("py_status()"), status_value(&fix.fr));
}

#[test]
fn g10_g11_state_machine_and_scripts_match_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[
        ("retract_length", 2.5),
        ("retract_speed", 45.0),
        ("unretract_extra_length", 0.5),
        ("unretract_speed", 33.0),
    ]);
    py.exec(
        "reset({'retract_length': 2.5, 'retract_speed': 45.0, \
         'unretract_extra_length': 0.5, 'unretract_speed': 33.0})",
    )
    .expect("reset");
    let steps: Vec<(&str, &str)> = vec![
        ("G11", "do_g11()"),
        ("G10", "do_g10()"),
        ("G10", "do_g10()"),
        ("G11", "do_g11()"),
        ("G11", "do_g11()"),
    ];
    for (cmd, expr) in &steps {
        fix.gcode.dispatch(cmd, &[]).expect("dispatch");
        py.eval(expr).expect("py cmd");
        check(
            &format!("scripts after {cmd}"),
            py.eval("py_scripts()"),
            scripts_value(&fix.gcode.scripts()),
        );
    }
}

#[test]
fn g10_g11_after_set_retraction_matches_python() {
    let Some(mut py) = oracle() else { return };
    let fix = fixture(&[]);
    py.exec("reset()").expect("reset");
    // Retract, then SET_RETRACTION resets is_retracted, so G11 is a no-op.
    fix.gcode.dispatch("G10", &[]).expect("g10");
    py.eval("do_g10()").expect("py g10");
    fix.gcode
        .dispatch("SET_RETRACTION", &[("RETRACT_SPEED", "30")])
        .expect("set");
    py.eval("do_set({'RETRACT_SPEED': '30'})").expect("py set");
    fix.gcode.dispatch("G11", &[]).expect("g11");
    py.eval("do_g11()").expect("py g11");
    check(
        "set resets: scripts",
        py.eval("py_scripts()"),
        scripts_value(&fix.gcode.scripts()),
    );
    // G10 again retracts with the new speed.
    fix.gcode.dispatch("G10", &[]).expect("g10 again");
    py.eval("do_g10()").expect("py g10 again");
    check(
        "set resets: scripts after second G10",
        py.eval("py_scripts()"),
        scripts_value(&fix.gcode.scripts()),
    );
}

#[test]
fn g10_g11_fractional_speed_truncation_matches_python() {
    let Some(mut py) = oracle() else { return };
    // 10.5 mm/s * 60 = 630 exactly; 2.5 with %.5f rounding covered above.
    let fix = fixture(&[
        ("retract_length", 0.0),
        ("retract_speed", 10.5),
        ("unretract_extra_length", 0.0),
        ("unretract_speed", 10.5),
    ]);
    py.exec(
        "reset({'retract_length': 0.0, 'retract_speed': 10.5, \
         'unretract_extra_length': 0.0, 'unretract_speed': 10.5})",
    )
    .expect("reset");
    fix.gcode.dispatch("G10", &[]).expect("g10");
    py.eval("do_g10()").expect("py g10");
    fix.gcode.dispatch("G11", &[]).expect("g11");
    py.eval("do_g11()").expect("py g11");
    check(
        "fractional: scripts",
        py.eval("py_scripts()"),
        scripts_value(&fix.gcode.scripts()),
    );
}

#[test]
fn set_retraction_error_paths_match_python() {
    // Each case triggers exactly one CommandError on both sides.
    let cases: Vec<(&str, &str, &str)> = vec![
        ("RETRACT_LENGTH", "-1", "{'RETRACT_LENGTH': '-1'}"),
        ("RETRACT_SPEED", "0.5", "{'RETRACT_SPEED': '0.5'}"),
        ("UNRETRACT_EXTRA_LENGTH", "-2", "{'UNRETRACT_EXTRA_LENGTH': '-2'}"),
        ("UNRETRACT_SPEED", "0", "{'UNRETRACT_SPEED': '0'}"),
        ("RETRACT_LENGTH", "abc", "{'RETRACT_LENGTH': 'abc'}"),
    ];
    for (param, value, py_params) in cases {
        let label = format!("set error {param}={value}");
        let Some(mut py) = oracle() else { return };
        let fix = fixture(&[]);
        py.exec("reset()").expect("reset");
        let rust = fix
            .gcode
            .dispatch("SET_RETRACTION", &[(param, value)])
            .expect_err("SET_RETRACTION must fail");
        let expected = py.eval(&format!("do_set({py_params})"));
        check_error(&label, expected, &rust);
        // Earlier parameters stick, later ones do not; compare status.
        check(
            &format!("{label}: status"),
            py.eval("py_status()"),
            status_value(&fix.fr),
        );
    }
}

#[test]
fn config_below_minimum_matches_python() {
    // Float minval (0.) renders identically; int minval (1) is
    // normalized by check_error.
    let cases: Vec<(&str, f64, &str)> = vec![
        ("retract_length", -1.0, "{'retract_length': -1.0}"),
        ("retract_speed", 0.5, "{'retract_speed': 0.5}"),
        ("unretract_extra_length", -1.0, "{'unretract_extra_length': -1.0}"),
        ("unretract_speed", 0.5, "{'unretract_speed': 0.5}"),
    ];
    for (option, value, py_values) in cases {
        let label = format!("config error {option}");
        let Some(mut py) = oracle() else { return };
        let gcode = FakeGCode::new();
        let printer = FakePrinter::new(Arc::clone(&gcode));
        let config = FakeConfig::new(printer, &[(option, value)]);
        let cfg: Config = config.clone();
        let rust = match FirmwareRetraction::new(&cfg) {
            Ok(_) => panic!("construction must fail"),
            Err(e) => e,
        };
        let expected = py.exec(&format!("reset({py_values})"));
        check_error(&label, expected, &rust);
    }
}

#[test]
fn module_factory_load_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config present");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let gcode = FakeGCode::new();
    let printer = FakePrinter::new(Arc::clone(&gcode));
    let config = FakeConfig::new(printer, &[]);
    let cfg: Config = config.clone();
    let obj = load_config(&cfg).expect("load_config").expect("object");
    // The returned object answers get_status like FirmwareRetraction.
    let value = to_rec(&obj.get_status(0.0).expect("get_status"));
    py.exec("reset()").expect("reset");
    check("factory: status", py.eval("py_status()"), value);
    check(
        "factory: registered",
        py.eval("py_registered()"),
        registered_value(&gcode),
    );
}
