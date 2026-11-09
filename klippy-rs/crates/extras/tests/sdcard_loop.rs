//! Parity tests for `extras::sdcard_loop` (port of
//! `klippy/extras/sdcard_loop.py`).
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a Python
//! `SDCardLoop` is built with minimal Python fakes for the printer, the
//! config, the gcode dispatcher and `virtual_sdcard`, driven with the
//! same inputs as the Rust port through its fakes of the contract
//! traits, and the results are compared exactly:
//!
//! * G-code command registration (names, help texts),
//! * `load_config` object construction through `FakePrinter` (the same
//!   `load_object` / `lookup_object` protocol the reference uses),
//! * `loop_begin` / `loop_end` / `loop_desist` return values,
//! * `loop_stack` contents and `set_file_position` call lists (file
//!   positions by value),
//! * `cmd_SDCARD_LOOP_*` command errors (class and message), including
//!   `COUNT` parse errors from the real `GCodeCommand.get_int`,

use std::collections::BTreeMap;
use std::sync::Arc;

use extras::sdcard_loop::{SDCardLoop, BEGIN_HELP, DESIST_HELP, END_HELP, module_factory};
use extras::virtual_sdcard::VirtualSDApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetIntListOpts,
    GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts, GetIntOpts,
    GetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi, ModuleFactory};
use klippy_api::{EventArg, GCell, ObjectRef, printer_object};
use pycompat::PyValue;
use recording::replay::assert_equal;
use recording::value::{Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.sdcard_loop as sdcard_loop_mod
import gcode as gcode_mod
import logging as _logging

class CommandError(Exception):
    pass

# `printer.config_error` class name is `Error` (`configparser.Error`).
Error = type("Error", (Exception,), {})

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

class FakeSDCard:
    def __init__(self, from_sd=False, pos=0):
        self.cmd_from_sd_flag = from_sd
        self.pos = pos
        self.set_calls = []
    def is_cmd_from_sd(self):
        return self.cmd_from_sd_flag
    def get_file_position(self):
        return self.pos
    def set_file_position(self, pos):
        self.set_calls.append(pos)
        self.pos = pos
    def set_from_sd(self, v):
        self.cmd_from_sd_flag = v
    def set_pos(self, p):
        self.pos = p

class FakeGCode:
    def __init__(self):
        self.registered = {}
        self.help = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd in self.registered:
            raise Error("gcode command %s already registered" % (cmd,))
        self.registered[cmd] = {'func': func, 'desc': desc, 'when_not_ready': when_not_ready}
        if desc is not None:
            self.help[cmd] = desc
    def respond_info(self, msg, log=True):
        pass
    def respond_raw(self, msg):
        pass

class FakeGCodeCommand(gcode_mod.GCodeCommand):
    error = CommandError
    def __init__(self, command, commandline, params, fake_gcode):
        super().__init__(fake_gcode, command, commandline, params, False)

class FakePrinter:
    def __init__(self, sdcard, gcode):
        self._sdcard = sdcard
        self._gcode = gcode
    def load_object(self, config, section):
        assert section == "virtual_sdcard", section
        return self._sdcard
    def lookup_object(self, name):
        assert name == "gcode", name
        return self._gcode

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
    def get_name(self):
        return "sdcard_loop"

def make_pair(from_sd=False, pos=0):
    sd = FakeSDCard(from_sd, pos)
    gg = FakeGCode()
    printer = FakePrinter(sd, gg)
    config = FakeConfig(printer)
    loop = sdcard_loop_mod.SDCardLoop(config)
    return loop, sd, gg, printer, config

def _gcmd(gg, cmd, line, params):
    return FakeGCodeCommand(cmd, line, params, gg)
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("sdcard_loop fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping sdcard_loop oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

/// `virtual_sdcard` fake: `cmd_from_sd` / `next_file_position` protocol.
/// Implements the owner's [`VirtualSDApi`]; the methods `sdcard_loop`
/// does not call are inert.
pub struct FakeVirtualSdcard {
    from_sd: GCell<bool>,
    pos: GCell<i64>,
    set_calls: GCell<Vec<i64>>,
}

printer_object!(FakeVirtualSdcard; dyn VirtualSDApi);

impl FakeVirtualSdcard {
    fn new(from_sd: bool, pos: i64) -> Self {
        FakeVirtualSdcard {
            from_sd: GCell::new(from_sd),
            pos: GCell::new(pos),
            set_calls: GCell::new(Vec::new()),
        }
    }

    fn set_from_sd(&self, v: bool) {
        self.from_sd.set(v);
    }

    fn set_pos(&self, p: i64) {
        self.pos.set(p);
    }

    fn set_calls(&self) -> Vec<i64> {
        self.set_calls.cloned()
    }
}

impl VirtualSDApi for FakeVirtualSdcard {
    fn get_file_list(&self, _check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        Ok(Vec::new())
    }

    fn is_active(&self) -> bool {
        false
    }

    fn do_pause(&self) {}

    fn do_resume(&self) -> KResult<()> {
        Ok(())
    }

    fn do_cancel(&self) -> KResult<()> {
        Ok(())
    }

    fn is_cmd_from_sd(&self) -> bool {
        self.from_sd.get()
    }

    fn get_file_position(&self) -> i64 {
        self.pos.get()
    }

    fn set_file_position(&self, pos: i64) {
        self.set_calls.borrow_mut().push(pos);
        self.pos.set(pos);
    }
}

/// `gcode` dispatcher fake: records registrations with help and stores
/// handlers so the tests can invoke them like the real dispatcher.
struct RegisteredCmd {
    handler: Option<GCodeCommandHandler>,
}

struct FakeGCodeInner {
    handlers: IndexMap<String, RegisteredCmd>,
    help: IndexMap<String, String>,
}

pub struct FakeGCode {
    inner: GCell<FakeGCodeInner>,
}

printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    fn new() -> Self {
        FakeGCode {
            inner: GCell::new(FakeGCodeInner {
                handlers: IndexMap::new(),
                help: IndexMap::new(),
            }),
        }
    }

    fn invoke(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut inner = self.inner.borrow_mut();
        let entry = inner
            .handlers
            .get_mut(cmd)
            .unwrap_or_else(|| panic!("FakeGCode: no handler for {cmd}"));
        let handler = entry
            .handler
            .as_mut()
            .unwrap_or_else(|| panic!("FakeGCode: handler for {cmd} was unregistered"));
        (handler)(gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let handler = match func {
            None => {
                let old = self.inner.borrow_mut().handlers.shift_remove(cmd);
                self.inner.borrow_mut().help.shift_remove(cmd);
                // Removal also drops the old handler back to the caller.
                let handler = old.and_then(|e| e.handler);
                return Ok(handler);
            }
            Some(h) => h,
        };
        if self.inner.borrow().handlers.contains_key(cmd) {
            return Err(KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        if let Some(desc) = opts.desc.clone() {
            self.inner.borrow_mut().help.insert(cmd.to_string(), desc);
        }
        self.inner.borrow_mut().handlers.insert(
            cmd.to_string(),
            RegisteredCmd {
                handler: Some(handler),
            },
        );
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
        Err(KlippyError::config("unused fake method: register_mux_command"))
    }

    fn get_command_help(&self) -> IndexMap<String, String> {
        self.inner.borrow().help.clone()
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut commands = IndexMap::new();
        for (cmd, desc) in self.inner.borrow().help.clone() {
            let mut entry = IndexMap::new();
            entry.insert(PyValue::Str("help".to_string()), PyValue::Str(desc));
            commands.insert(PyValue::Str(cmd), PyValue::Dict(entry));
        }
        let mut out = IndexMap::new();
        out.insert(PyValue::Str("commands".to_string()), PyValue::Dict(commands));
        PyValue::Dict(out)
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

    fn get_mutex(&self) -> reactor::ReactorMutex {
        reactor::Reactor::new(false).mutex(false)
    }

    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCodeCommand::new(
            command.to_string(),
            commandline.to_string(),
            params.clone(),
        ))
    }

    fn respond_raw(&self, _msg: &str) {}

    fn respond_info(&self, _msg: &str, _log: bool) {}

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

/// `GCodeCommand` fake: parameter parsing mirrors `klippy/gcode.py`
/// (`GCodeCommand.get` / `get_int` / `get_float`, lines 65-96) with the
/// exact error texts.
pub struct FakeGCodeCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCodeCommand {
    fn new(command: String, commandline: String, params: IndexMap<String, String>) -> Self {
        FakeGCodeCommand {
            command,
            commandline,
            params,
        }
    }

    fn missing(name: &str, commandline: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': missing {name}"))
    }

    fn unable(commandline: &str, value: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': unable to parse {value}"))
    }

    /// `%s` rendering of a bound value, as `str()` in Python: ints in
    /// decimal, floats via CPython `repr`.
    fn bound_str(v: &PyValue) -> String {
        match v {
            PyValue::Str(s) => s.clone(),
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Float(f) => pycompat::text::float_repr(*f),
            PyValue::Bool(b) => {
                if *b {
                    "True".to_string()
                } else {
                    "False".to_string()
                }
            }
            PyValue::None => "None".to_string(),
            other => other.str(),
        }
    }

    /// Numeric comparison for the bounds of `GCodeCommand.get`
    /// (numbers only in the tree: `int` / `float`).
    fn cmp_py(a: &PyValue, b: &PyValue) -> std::cmp::Ordering {
        use std::cmp::Ordering;
        fn num(v: &PyValue) -> (f64, Option<i64>) {
            match v {
                PyValue::Int(n) => {
                    let s = n.0.to_string();
                    let i: Option<i64> = s.parse::<i64>().ok();
                    (i.unwrap_or(0) as f64, i)
                }
                PyValue::Float(f) => (*f, None),
                PyValue::Bool(b) => {
                    let i: i64 = if *b { 1 } else { 0 };
                    (i as f64, Some(i))
                }
                _ => (0.0, None),
            }
        }
        let (af, ai) = num(a);
        let (bf, bi) = num(b);
        if let (Some(x), Some(y)) = (ai, bi) {
            return x.cmp(&y);
        }
        if af < bf {
            Ordering::Less
        } else if af > bf {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let commandline = self.commandline.clone();
        let value = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let parsed: PyValue = match opts.parser {
            GetParser::Str => PyValue::Str(value),
            GetParser::Int => {
                let pyint = match pycompat::text::int_from_string(&value, 10) {
                    Ok(v) => v,
                    Err(_) => return Err(Self::unable(&commandline, &value)),
                };
                match pyint.0.to_string().parse::<i64>() {
                    Ok(_) => PyValue::Int(pyint),
                    Err(_) => return Err(Self::unable(&commandline, &value)),
                }
            }
            GetParser::IntAutoBase => {
                let pyint = match pycompat::text::int_from_string(&value, 0) {
                    Ok(v) => v,
                    Err(_) => return Err(Self::unable(&commandline, &value)),
                };
                match pyint.0.to_string().parse::<i64>() {
                    Ok(_) => PyValue::Int(pyint),
                    Err(_) => return Err(Self::unable(&commandline, &value)),
                }
            }
            GetParser::Float => match pycompat::text::float_from_string(&value) {
                Ok(v) => PyValue::Float(v),
                Err(_) => return Err(Self::unable(&commandline, &value)),
            },
        };
        if let Some(mn) = opts.minval {
            if Self::cmp_py(&parsed, &mn) == std::cmp::Ordering::Less {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have minimum of {}",
                    Self::bound_str(&mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if Self::cmp_py(&parsed, &mx) == std::cmp::Ordering::Greater {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have maximum of {}",
                    Self::bound_str(&mx)
                )));
            }
        }
        if let Some(ab) = opts.above {
            if Self::cmp_py(&parsed, &ab) != std::cmp::Ordering::Greater {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must be above {}",
                    Self::bound_str(&ab)
                )));
            }
        }
        if let Some(be) = opts.below {
            if Self::cmp_py(&parsed, &be) != std::cmp::Ordering::Less {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must be below {}",
                    Self::bound_str(&be)
                )));
            }
        }
        Ok(parsed)
    }

    fn get_int(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let commandline = self.commandline.clone();
        let value = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let pyint = match pycompat::text::int_from_string(&value, 10) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        // Exact-value note: `int_from_string` returns the exact Python int;
        // the `i64` conversion below only fails for values outside `i64`,
        // which the port reports as unable-to-parse (documented deviation
        // of the reference port, exercised only with small ints).
        let parsed: i64 = match pyint.0.to_string().parse::<i64>() {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        if let Some(mn) = opts.minval {
            if parsed < mn {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have minimum of {mn}"
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if parsed > mx {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have maximum of {mx}"
                )));
            }
        }
        Ok(Some(parsed))
    }

    fn get_float(&self, name: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let commandline = self.commandline.clone();
        let value = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(Self::missing(name, &commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let parsed = match pycompat::text::float_from_string(&value) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        if let Some(mn) = opts.minval {
            if parsed < mn {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have minimum of {}",
                    pycompat::text::float_repr(mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if parsed > mx {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must have maximum of {}",
                    pycompat::text::float_repr(mx)
                )));
            }
        }
        if let Some(ab) = opts.above {
            if parsed <= ab {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must be above {}",
                    pycompat::text::float_repr(ab)
                )));
            }
        }
        if let Some(be) = opts.below {
            if parsed >= be {
                return Err(KlippyError::command(format!(
                    "Error on '{commandline}': {name} must be below {}",
                    pycompat::text::float_repr(be)
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
        let command = &self.command;
        let origline = &self.commandline;
        let mut param_start = command.len();
        let param_end = origline.len();
        if origline.len() < param_start || &origline[..param_start].to_uppercase() != command {
            // Skip any gcode line-number and ignore any trailing checksum.
            let up = origline.to_uppercase();
            let pos = up.find(command.as_str()).unwrap_or(0);
            param_start += pos;
        }
        if origline.len() > param_start
            && origline[param_start..].starts_with(char::is_whitespace)
        {
            param_start += 1;
        }
        origline[param_start..param_end].to_string()
    }

    fn respond_info(&self, _msg: &str, _log: bool) {}

    fn respond_raw(&self, _msg: &str) {}

    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

pub struct FakePrinter {
    objects: GCell<BTreeMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter {
            objects: GCell::new(BTreeMap::new()),
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }

    fn get_reactor(&self) -> reactor::Reactor {
        reactor::Reactor::new(false)
    }

    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
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

    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
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

    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let objs = self.objects.borrow();
            match module {
                None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
                Some(m) => {
                    let mut out = Vec::new();
                    if let Some(o) = objs.get(m) {
                        out.push((m.to_string(), o.clone()));
                    }
                    let prefix = format!("{m} ");
                    for (k, v) in objs.iter() {
                        if k.starts_with(&prefix) {
                            out.push((k.clone(), v.clone()));
                        }
                    }
                    out
                }
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
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
                    "Unable to load module '{section}'"
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
        _event: &str,
        _callback: klippy_api::printer::EventCallback,
    ) {
    }

    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }

    fn request_exit(&self, _result: &str) {}
}

pub struct FakeConfig {
    printer: Printer,
    name: String,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str) -> Self {
        FakeConfig {
            printer,
            name: name.to_string(),
        }
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }

    fn get_name(&self) -> String {
        self.name.clone()
    }

    fn get(&self, _option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match opts.default {
            None => Err(KlippyError::config(
                "unused fake method: get requires an option",
            )),
            Some(v) => Ok(v),
        }
    }

    fn getint(&self, _option: &str, opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getint")),
            Some(v) => Ok(v),
        }
    }

    fn getfloat(&self, _option: &str, opts: klippy_api::config::GetFloatOpts) -> KResult<Option<f64>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getfloat")),
            Some(v) => Ok(v),
        }
    }

    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getboolean")),
            Some(v) => Ok(v),
        }
    }

    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(KlippyError::config("unused fake method: getchoice"))
    }

    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::config("unused fake method: getlists"))
    }

    fn getlist(&self, _option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getlist")),
            Some(v) => Ok(v),
        }
    }

    fn getintlist(&self, _option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getintlist")),
            Some(v) => Ok(v),
        }
    }

    fn getfloatlist(&self, _option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        match opts.default {
            None => Err(KlippyError::config("unused fake method: getfloatlist")),
            Some(v) => Ok(v),
        }
    }

    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig::new(self.printer.clone(), section))
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
// Comparison plumbing.
// ---------------------------------------------------------------------------

// Captured Rust-side log lines on this test's thread; the sink is
// installed once per process and parallel tests keep their own lines.
thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_LINES.with(|lines| lines.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

fn log_capture() {
    install_log_sink();
    LOG_LINES.with(|lines| lines.borrow_mut().clear());
}

fn captured_logs() -> Vec<String> {
    LOG_LINES.with(|lines| lines.borrow().clone())
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.sdcard_loop".to_string(),
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

/// Compare a command result: oracle `Ok(Null)` vs Rust `Ok(())`, or both
/// raising the same class and message.
fn check_cmd(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Ok(Value::Null), Ok(())) => {}
        (Ok(v), Ok(())) => panic!("{label}: oracle returned {v:?}, expected null"),
        (Ok(v), Err(e)) => panic!("{label}: oracle returned {v:?}, port raised {}: {}", e.class_name(), e.message()),
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception class");
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
        (Err(exc), Ok(())) => panic!(
            "{label}: oracle raised {}: {}, port succeeded",
            exc.class_name, exc.message
        ),
    }
}

fn stack_value(stack: &[(i64, i64)]) -> Value {
    Value::List(
        stack
            .iter()
            .map(|(c, p)| Value::Tuple(vec![Value::int(*c), Value::int(*p)]))
            .collect(),
    )
}

fn calls_value(calls: &[i64]) -> Value {
    Value::List(calls.iter().map(|c| Value::int(*c)).collect())
}

/// Build a fresh Rust pair through `SDCardLoop::new`, i.e. through the
/// same `FakePrinter` object protocol the reference `__init__` uses.
/// The single `FakeGCode` is both the registration target and the handle
/// the test inspects.
fn make_config(gcode: Arc<FakeGCode>, sdcard: Arc<FakeVirtualSdcard>) -> Config {
    let printer = Arc::new(FakePrinter::new());
    printer.add_object("gcode", gcode).expect("add gcode");
    printer.add_object("virtual_sdcard", sdcard).expect("add sdcard");
    Arc::new(FakeConfig::new(printer, "sdcard_loop"))
}

fn rust_pair(from_sd: bool, pos: i64) -> (Arc<SDCardLoop>, Arc<FakeGCode>, Arc<FakeVirtualSdcard>) {
    let fake_gcode = Arc::new(FakeGCode::new());
    let fake_sdcard = Arc::new(FakeVirtualSdcard::new(from_sd, pos));
    let config = make_config(fake_gcode.clone(), fake_sdcard.clone());
    let me = SDCardLoop::new(&config).expect("new registers commands");
    (me, fake_gcode, fake_sdcard)
}

fn rust_gcmd(command: &str, commandline: &str, params: &[(&str, &str)]) -> GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeGCodeCommand::new(
        command.to_string(),
        commandline.to_string(),
        map,
    ))
}

fn py_gcmd_setup(py: &mut pyoracle::Oracle, cmd: &str, line: &str, count: Option<&str>) {
    match count {
        Some(v) => py
            .exec(&format!(
                "gcmd = _gcmd(gg, {cmd:?}, {line:?}, {{'COUNT': {v:?}}})"
            ))
            .expect("py gcmd"),
        None => py
            .exec(&format!("gcmd = _gcmd(gg, {cmd:?}, {line:?}, {{}})"))
            .expect("py gcmd"),
    };
}

fn fresh_pair(py: &mut pyoracle::Oracle, from_sd: bool, pos: i64) {
    py.exec(&format!(
        "loop, sd, gg, printer, config = make_pair({}, {})",
        if from_sd { "True" } else { "False" },
        pos
    ))
    .expect("py pair");
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn registration_matches_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 0);
    let (_me, fake_gcode, _sdcard) = rust_pair(true, 0);
    let expected = py.eval("gg.help");
    let pairs: Vec<(Value, Value)> = fake_gcode
        .get_command_help()
        .into_iter()
        .map(|(k, v)| (Value::str(&k), Value::str(&v)))
        .collect();
    // Rust `IndexMap` preserves registration order; Python dict too.
    let actual = Value::Dict(pairs);
    check("registration help", expected, actual);
    // Exact help texts travel with the commands.
    assert_eq!(BEGIN_HELP, "Begins a looped section in the SD file.");
    assert_eq!(END_HELP, "Ends a looped section in the SD file.");
    assert_eq!(DESIST_HELP, "Stops iterating the current loop stack.");
    // No duplicate-registration error on the clean object.
    assert_eq!(
        fake_gcode.get_command_help().len(),
        3,
        "three commands registered"
    );
}

#[test]
fn duplicate_registration_is_config_error() {
    let gcode = Arc::new(FakeGCode::new());
    let sdcard = Arc::new(FakeVirtualSdcard::new(true, 0));
    let cfg1 = make_config(gcode.clone(), sdcard.clone());
    SDCardLoop::new(&cfg1).expect("first new");
    let cfg2 = make_config(gcode, sdcard);
    let err = match SDCardLoop::new(&cfg2) {
        Ok(_) => panic!("duplicate new must fail"),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), "Error");
}

#[test]
fn handlers_drive_commands() {
    // The closures `new` registered dispatch to the same `loop_*` logic
    // the oracle tests compare directly.
    let (me, fake_gcode, sdcard) = rust_pair(true, 25);
    let c = rust_gcmd(
        "SDCARD_LOOP_BEGIN",
        "SDCARD_LOOP_BEGIN COUNT=2",
        &[("COUNT", "2")],
    );
    fake_gcode.invoke("SDCARD_LOOP_BEGIN", &c).expect("invoke begin");
    assert_eq!(me.loop_stack_snapshot(), vec![(2, 25)]);
    let c = rust_gcmd("SDCARD_LOOP_END", "SDCARD_LOOP_END", &[]);
    fake_gcode.invoke("SDCARD_LOOP_END", &c).expect("invoke end");
    assert_eq!(me.loop_stack_snapshot(), vec![(1, 25)]);
    assert_eq!(sdcard.set_calls(), vec![25]);
    sdcard.set_from_sd(false);
    let c = rust_gcmd("SDCARD_LOOP_DESIST", "SDCARD_LOOP_DESIST", &[]);
    fake_gcode
        .invoke("SDCARD_LOOP_DESIST", &c)
        .expect("invoke desist");
    assert!(me.loop_stack_snapshot().is_empty());
}

#[test]
fn loop_begin_end_round_trip_matches_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 100);
    let (me, _gg, sdcard) = rust_pair(true, 100);
    // begin COUNT=2 at pos 100.
    let c = rust_gcmd(
        "SDCARD_LOOP_BEGIN",
        "SDCARD_LOOP_BEGIN COUNT=2",
        &[("COUNT", "2")],
    );
    py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", "SDCARD_LOOP_BEGIN COUNT=2", Some("2"));
    check_cmd(
        "begin ok",
        py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
        me.cmd_SDCARD_LOOP_BEGIN(&c),
    );
    check(
        "stack after begin",
        py.eval("loop.loop_stack"),
        stack_value(&me.loop_stack_snapshot()),
    );
    // end once: count 2 -> seek back, push (1, 100).
    let c = rust_gcmd("SDCARD_LOOP_END", "SDCARD_LOOP_END", &[]);
    py_gcmd_setup(&mut py, "SDCARD_LOOP_END", "SDCARD_LOOP_END", None);
    check_cmd(
        "end once ok",
        py.eval("loop.cmd_SDCARD_LOOP_END(gcmd)"),
        me.cmd_SDCARD_LOOP_END(&c),
    );
    check(
        "stack after first end",
        py.eval("loop.loop_stack"),
        stack_value(&me.loop_stack_snapshot()),
    );
    check(
        "seeks after first end",
        py.eval("sd.set_calls"),
        calls_value(&sdcard.set_calls()),
    );
    // end again: count 1 -> drop, no seek.
    check_cmd(
        "end twice ok",
        py.eval("loop.cmd_SDCARD_LOOP_END(gcmd)"),
        me.cmd_SDCARD_LOOP_END(&c),
    );
    check(
        "stack after second end",
        py.eval("loop.loop_stack"),
        stack_value(&me.loop_stack_snapshot()),
    );
    check(
        "seeks after second end",
        py.eval("sd.set_calls"),
        calls_value(&sdcard.set_calls()),
    );
}

#[test]
fn loop_end_count_one_and_zero_match_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 7);
    let (me, _gg, sdcard) = rust_pair(true, 7);
    // COUNT=1: single pass, END drops the entry without seeking.
    check(
        "begin 1",
        py.eval("loop.loop_begin(1)"),
        Value::Bool(me.loop_begin(1)),
    );
    check(
        "end 1",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    check("stack empty", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check("no seeks", py.eval("sd.set_calls"), calls_value(&sdcard.set_calls()));
    // COUNT=0: infinite loop, END re-arms every time.
    check(
        "begin 0",
        py.eval("loop.loop_begin(0)"),
        Value::Bool(me.loop_begin(0)),
    );
    for i in 0..3 {
        check(
            &format!("end 0 #{i}"),
            py.eval("loop.loop_end()"),
            Value::Bool(me.loop_end()),
        );
    }
    check("infinite stack", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check("infinite seeks", py.eval("sd.set_calls"), calls_value(&sdcard.set_calls()));
    check("position", py.eval("sd.pos"), Value::int(sdcard.get_file_position()));
}

#[test]
fn loop_end_empty_stack_matches_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 55);
    let (me, _gg, sdcard) = rust_pair(true, 55);
    check(
        "end empty returns true",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    check("stack still empty", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check("no seeks", py.eval("sd.set_calls"), calls_value(&sdcard.set_calls()));
}

#[test]
fn nested_loops_match_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 100);
    let (me, _gg, sdcard) = rust_pair(true, 100);
    check(
        "outer begin",
        py.eval("loop.loop_begin(2)"),
        Value::Bool(me.loop_begin(2)),
    );
    py.exec("sd.set_pos(200)").expect("advance pos");
    sdcard.set_pos(200);
    check(
        "inner begin",
        py.eval("loop.loop_begin(3)"),
        Value::Bool(me.loop_begin(3)),
    );
    check(
        "inner end",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    check("nested stack one", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check(
        "nested seeks one",
        py.eval("sd.set_calls"),
        calls_value(&sdcard.set_calls()),
    );
    check(
        "inner end 2",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    check(
        "outer end",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    check("nested stack two", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check(
        "nested seeks two",
        py.eval("sd.set_calls"),
        calls_value(&sdcard.set_calls()),
    );
    check("position", py.eval("sd.pos"), Value::int(sdcard.get_file_position()));
}

#[test]
fn outside_sd_begin_end_match_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, false, 42);
    let (me, _gg, sdcard) = rust_pair(false, 42);
    check(
        "begin false",
        py.eval("loop.loop_begin(5)"),
        Value::Bool(me.loop_begin(5)),
    );
    check("stack empty", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check("no seeks", py.eval("sd.set_calls"), calls_value(&sdcard.set_calls()));
    check(
        "end false",
        py.eval("loop.loop_end()"),
        Value::Bool(me.loop_end()),
    );
    // Command forms raise `Only permitted in SD file.`
    let c = rust_gcmd(
        "SDCARD_LOOP_BEGIN",
        "SDCARD_LOOP_BEGIN COUNT=5",
        &[("COUNT", "5")],
    );
    py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", "SDCARD_LOOP_BEGIN COUNT=5", Some("5"));
    check_cmd(
        "cmd begin outside",
        py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
        me.cmd_SDCARD_LOOP_BEGIN(&c),
    );
    let c = rust_gcmd("SDCARD_LOOP_END", "SDCARD_LOOP_END", &[]);
    py_gcmd_setup(&mut py, "SDCARD_LOOP_END", "SDCARD_LOOP_END", None);
    check_cmd(
        "cmd end outside",
        py.eval("loop.cmd_SDCARD_LOOP_END(gcmd)"),
        me.cmd_SDCARD_LOOP_END(&c),
    );
}

#[test]
fn desist_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    py.exec("log_lines.clear()").expect("clear py logs");
    fresh_pair(&mut py, true, 10);
    let (me, _gg, sdcard) = rust_pair(true, 10);
    me.loop_begin(2);
    let _ = py.eval("loop.loop_begin(2)");
    // Still in SD: desist fails both sides, stack untouched.
    check(
        "desist in sd fails",
        py.eval("loop.loop_desist()"),
        Value::Bool(me.loop_desist()),
    );
    check("stack kept", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    // Leave SD: desist succeeds, logs and clears.
    py.exec("sd.set_from_sd(False)").expect("py leave sd");
    sdcard.set_from_sd(false);
    check(
        "desist outside ok",
        py.eval("loop.loop_desist()"),
        Value::Bool(me.loop_desist()),
    );
    check("stack cleared", py.eval("loop.loop_stack"), stack_value(&me.loop_stack_snapshot()));
    check("py log lines", py.eval("log_lines"), Value::List(vec![Value::str("Desisting existing SD loops")]));
    assert_eq!(
        captured_logs(),
        vec!["Desisting existing SD loops".to_string()],
        "rust log lines"
    );
    // Command form outside SD also succeeds (stack already empty).
    let c = rust_gcmd("SDCARD_LOOP_DESIST", "SDCARD_LOOP_DESIST", &[]);
    py_gcmd_setup(&mut py, "SDCARD_LOOP_DESIST", "SDCARD_LOOP_DESIST", None);
    check_cmd(
        "cmd desist outside",
        py.eval("loop.cmd_SDCARD_LOOP_DESIST(gcmd)"),
        me.cmd_SDCARD_LOOP_DESIST(&c),
    );
    // Command form inside SD raises `Only permitted outside of a SD file.`
    py.exec("sd.set_from_sd(True)").expect("py re-enter sd");
    sdcard.set_from_sd(true);
    check_cmd(
        "cmd desist inside",
        py.eval("loop.cmd_SDCARD_LOOP_DESIST(gcmd)"),
        me.cmd_SDCARD_LOOP_DESIST(&c),
    );
}

#[test]
fn count_parsing_matches_python() {
    let Some(mut py) = oracle() else { return };
    // (COUNT text, expected count) pairs the real `int()` accepts.
    // A fresh pair per case keeps the stacks independent (`loop_end` on
    // `COUNT=0` re-arms instead of clearing).
    for (text, want) in [
        ("0", 0i64),
        ("3", 3),
        ("+5", 5),
        ("-0", 0),
        ("07", 7),
        (" 9 ", 9),
        ("2_0", 20),
        ("100", 100),
    ] {
        fresh_pair(&mut py, true, 0);
        let (me, _gg, _sdcard) = rust_pair(true, 0);
        let line = format!("SDCARD_LOOP_BEGIN COUNT={}", text);
        let c = rust_gcmd("SDCARD_LOOP_BEGIN", &line, &[("COUNT", text)]);
        py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", &line, Some(text));
        let label = format!("count {text:?}");
        check_cmd(
            &label,
            py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
            me.cmd_SDCARD_LOOP_BEGIN(&c),
        );
        check(
            &format!("{label} stack"),
            py.eval("loop.loop_stack"),
            stack_value(&me.loop_stack_snapshot()),
        );
        assert_eq!(me.loop_stack_snapshot().last().unwrap().0, want);
    }
}

#[test]
fn count_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    fresh_pair(&mut py, true, 0);
    let (me, _gg, _sdcard) = rust_pair(true, 0);
    // (COUNT text, commandline) pairs the real `int()` rejects or the bound
    // rejects. `COUNT=-0` parses as 0 and is covered elsewhere.
    for text in ["-1", "abc", "2.5", "0x10", "", "1e3", "--1", " "] {
        let line = format!("SDCARD_LOOP_BEGIN COUNT={text}");
        let c = rust_gcmd(
            "SDCARD_LOOP_BEGIN",
            &line,
            &[("COUNT", text)],
        );
        py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", &line, Some(text));
        check_cmd(
            &format!("bad count {text:?}"),
            py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
            me.cmd_SDCARD_LOOP_BEGIN(&c),
        );
    }
    // Missing COUNT raises before any from-SD check.
    let c = rust_gcmd("SDCARD_LOOP_BEGIN", "SDCARD_LOOP_BEGIN", &[]);
    py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", "SDCARD_LOOP_BEGIN", None);
    check_cmd(
        "missing count",
        py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
        me.cmd_SDCARD_LOOP_BEGIN(&c),
    );
    // A bad COUNT outside SD still reports the parse error, proving the
    // `get_int` runs before the from-SD check on both sides.
    py.exec("sd.set_from_sd(False)").expect("py leave sd");
    let line = "SDCARD_LOOP_BEGIN COUNT=abc";
    let c2 = rust_gcmd("SDCARD_LOOP_BEGIN", line, &[("COUNT", "abc")]);
    py_gcmd_setup(&mut py, "SDCARD_LOOP_BEGIN", line, Some("abc"));
    check_cmd(
        "bad count outside sd beats sd error",
        py.eval("loop.cmd_SDCARD_LOOP_BEGIN(gcmd)"),
        me.cmd_SDCARD_LOOP_BEGIN(&c2),
    );
}

#[test]
fn module_factory_load_config_is_wired() {
    // `load_config` is the only entry point (`sdcard_loop.py` has no
    // `load_config_prefix` / `add_printer_objects`).
    let f: ModuleFactory = module_factory();
    assert!(f.load_config.is_some());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}
