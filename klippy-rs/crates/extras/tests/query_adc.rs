//! Parity tests for `klippy/extras/query_adc.py`.
//!
//! Every expected value comes live from the frozen reference through
//! the `pyoracle` crate: the oracle builds the real Python
//! `query_adc.QueryADC` with minimal printer/config/G-code fakes and
//! drives the real `gcode.GCodeCommand` argument parsing, so the
//! `NAME` / `PULLUP` lookup, the exact `respond_info` message text
//! (including `%.6f` / `%.3f` / `%.0f` rendering and the clamped
//! resistance computation), and the `CommandError` class/message
//! for a non-positive or unparseable `PULLUP` are all produced by
//! CPython itself. The port is driven through fakes of the
//! `klippy_api` contract traits with the same inputs, and the two
//! sides are compared. The test skips with a message outside the
//! text), the unknown-name listing (sorted names, empty registry),
//! a known pin without and with `PULLUP` (including both clamp
//! boundaries, zero/negative/out-of-range values, `nan`/`inf`
//! timestamps and values), `register_adc` overwrite semantics, the
//! early return before `PULLUP` parsing, and the `CommandError`
//! paths.
use extras::query_adc::{module_factory, QueryAdc, CMD_QUERY_ADC_HELP};
use indexmap::IndexMap;
use klippy_api::config::{Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts as ConfigGetFloatOpts, GetIntListOpts, GetIntOpts as ConfigGetIntOpts,
    GetListOpts, GetListsOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler,
    GetFloatOpts, GetIntOpts, GetOpts, RegisterCommandOpts};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AdcCallback, AdcPinApi, AdcSampleOpts, AsyncResponseWrapper,
    AsyncResponseWrapperApi, CommandQueue, CommandQueryWrapper, CommandWrapper, Mcu, McuApi,
    PinParams, TriggerDispatch};
use klippy_api::object::{ObjectRef, EventArg};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi, ModuleFactory};
use pycompat::text::{float_from_string, float_repr, int_from_string};
use pycompat::PyValue;
use reactor::Reactor;
use recording::Value;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Fakes of the klippy_api contract traits.
// ---------------------------------------------------------------------------

/// Fake `Printer`: object registry plus a real reactor handle.
struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            reactor,
        })
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
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.borrow_mut();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.borrow();
        if let Some(obj) = objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(n, o)| (n.clone(), o.clone())).collect()
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
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

/// Fake `ConfigWrapper`: only the printer accessor is exercised by
/// `query_adc` (`QueryADC.__init__` reads nothing else). The option
/// getters report the absence plainly rather than returning a
/// plausible value.
struct FakeConfig {
    printer: Printer,
    name: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Err(KlippyError::Config(format!(
            "Option '{option}' not found in section '{}' (fake config)",
            self.name
        )))
    }
    fn getint(&self, _option: &str, _opts: ConfigGetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::Config("getint unsupported by fake config".to_string()))
    }
    fn getfloat(&self, _option: &str, _opts: ConfigGetFloatOpts) -> KResult<Option<f64>> {
        Err(KlippyError::Config("getfloat unsupported by fake config".to_string()))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(KlippyError::Config("getboolean unsupported by fake config".to_string()))
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(KlippyError::Config("getchoice unsupported by fake config".to_string()))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::Config("getlists unsupported by fake config".to_string()))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::Config("getlist unsupported by fake config".to_string()))
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::Config("getintlist unsupported by fake config".to_string()))
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::Config("getfloatlist unsupported by fake config".to_string()))
    }
    fn getsection(&self, _section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: self.name.clone(),
        })
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

/// One stored handler registration on the fake dispatch.
struct StoredHandler {
    when_not_ready: bool,
    desc: Option<String>,
    func: Option<GCodeCommandHandler>,
}

/// Fake `GCodeDispatch`: records registrations and lets the test
/// dispatch through the stored handler, exactly like the real
/// dispatch serving a parsed line.
struct FakeGCode {
    reactor: Reactor,
    handlers: GCell<IndexMap<String, StoredHandler>>,
    raw: GCell<Vec<String>>,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakeGCode {
            reactor,
            handlers: GCell::new(IndexMap::new()),
            raw: GCell::new(Vec::new()),
        })
    }

    /// Serve one command through the registered handler.
    fn dispatch(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handlers = self.handlers.borrow_mut();
        let stored = handlers.get_mut(cmd).expect("command registered");
        let func = stored.func.as_mut().expect("handler present");
        func(gcmd)
    }

    fn registrations(&self) -> Vec<(String, bool, Option<String>)> {
        self.handlers
            .borrow()
            .iter()
            .map(|(cmd, s)| (cmd.clone(), s.when_not_ready, s.desc.clone()))
            .collect()
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut handlers = self.handlers.borrow_mut();
        match func {
            None => {
                let old = handlers.shift_remove(cmd);
                Ok(old.and_then(|s| s.func))
            }
            Some(f) => {
                handlers.insert(
                    cmd.to_string(),
                    StoredHandler {
                        when_not_ready: opts.when_not_ready,
                        desc: opts.desc,
                        func: Some(f),
                    },
                );
                Ok(None)
            }
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
        self.handlers
            .borrow()
            .iter()
            .filter_map(|(cmd, s)| s.desc.clone().map(|d| (cmd.clone(), d)))
            .collect()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCodeCommand::new(
            command,
            commandline,
            params.clone(),
        ))
    }
    fn respond_raw(&self, msg: &str) {
        self.raw.borrow_mut().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.raw.borrow_mut().push(msg.to_string());
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

/// Fake `GCodeCommand`: a params dict plus exact `get` / `get_float`
/// / `respond_info` semantics from `klippy/gcode.py:65-96`. Records
/// `(msg, log)` respond_info calls for comparison.
struct FakeGCodeCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    responses: GCell<Vec<(String, bool)>>,
}

impl FakeGCodeCommand {
    fn new(command: &str, commandline: &str, params: IndexMap<String, String>) -> Self {
        FakeGCodeCommand {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            responses: GCell::new(Vec::new()),
        }
    }

    fn command_error(&self, text: String) -> KlippyError {
        KlippyError::Command(text)
    }

    fn err_missing(&self, name: &str) -> KlippyError {
        self.command_error(format!("Error on '{}': missing {name}", self.commandline))
    }
    fn err_parse(&self, value: &str) -> KlippyError {
        self.command_error(format!(
            "Error on '{}': unable to parse {value}",
            self.commandline
        ))
    }
    fn err_minimum(&self, name: &str, bound: &str) -> KlippyError {
        self.command_error(format!(
            "Error on '{}': {name} must have minimum of {bound}",
            self.commandline
        ))
    }
    fn err_maximum(&self, name: &str, bound: &str) -> KlippyError {
        self.command_error(format!(
            "Error on '{}': {name} must have maximum of {bound}",
            self.commandline
        ))
    }
    fn err_above(&self, name: &str, bound: &str) -> KlippyError {
        self.command_error(format!(
            "Error on '{}': {name} must be above {bound}",
            self.commandline
        ))
    }
    fn err_below(&self, name: &str, bound: &str) -> KlippyError {
        self.command_error(format!(
            "Error on '{}': {name} must be below {bound}",
            self.commandline
        ))
    }
}



impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.err_missing(name)),
                Some(d) => Ok(d),
            };
        };
        let (value, numeric): (PyValue, Option<f64>) = match opts.parser {
            klippy_api::gcode::GetParser::Str => (PyValue::Str(raw.clone()), None),
            klippy_api::gcode::GetParser::Int => match int_from_string(raw, 10) {
                Ok(n) => (PyValue::Int(n), None),
                Err(_) => return Err(self.err_parse(raw)),
            },
            klippy_api::gcode::GetParser::IntAutoBase => match int_from_string(raw, 0) {
                Ok(n) => (PyValue::Int(n), None),
                Err(_) => return Err(self.err_parse(raw)),
            },
            klippy_api::gcode::GetParser::Float => match float_from_string(raw) {
                Ok(f) => (PyValue::Float(f), Some(f)),
                Err(_) => return Err(self.err_parse(raw)),
            },
        };
        if let Some(v) = numeric {
            if let Some(minval) = opts.minval.as_ref().and_then(|b| b.to_f64()) {
                if v < minval {
                    return Err(self.err_minimum(name, &opts.minval.as_ref().unwrap().str()));
                }
            }
            if let Some(maxval) = opts.maxval.as_ref().and_then(|b| b.to_f64()) {
                if v > maxval {
                    return Err(self.err_maximum(name, &opts.maxval.as_ref().unwrap().str()));
                }
            }
            if let Some(above) = opts.above.as_ref().and_then(|b| b.to_f64()) {
                if v <= above {
                    return Err(self.err_above(name, &opts.above.as_ref().unwrap().str()));
                }
            }
            if let Some(below) = opts.below.as_ref().and_then(|b| b.to_f64()) {
                if v >= below {
                    return Err(self.err_below(name, &opts.below.as_ref().unwrap().str()));
                }
            }
        }
        Ok(value)
    }

    fn get_int(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(self.err_missing(name)),
                    Some(d) => Ok(d),
                };
            }
            Some(raw) => raw.clone(),
        };
        let n = int_from_string(&raw, 10).map_err(|_| self.err_parse(&raw))?;
        let s = n.0.to_string();
        let v: i64 = s
            .parse()
            .map_err(|_| self.err_parse(&raw))?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(self.err_minimum(name, &minval.to_string()));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(self.err_maximum(name, &maxval.to_string()));
            }
        }
        Ok(Some(v))
    }

    fn get_float(&self, name: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            None => {
                return match opts.default {
                    None => Err(self.err_missing(name)),
                    Some(d) => Ok(d),
                };
            }
            Some(raw) => raw.clone(),
        };
        let v = float_from_string(&raw).map_err(|_| self.err_parse(&raw))?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(self.err_minimum(name, &float_repr(minval)));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(self.err_maximum(name, &float_repr(maxval)));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(self.err_above(name, &float_repr(above)));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(self.err_below(name, &float_repr(below)));
            }
        }
        Ok(Some(v))
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
        let mut param_start = command.len().min(origline.len());
        let mut param_end = origline.len();
        if !origline[..param_start].eq_ignore_ascii_case(command) {
            param_start += origline
                .to_uppercase()
                .find(&command.to_uppercase())
                .unwrap_or(0);
            if let Some(end) = origline.rfind('*') {
                if origline[end + 1..].chars().all(|c| c.is_ascii_digit()) {
                    param_end = end;
                }
            }
        }
        if param_start < origline.len()
            && origline[param_start..].starts_with(char::is_whitespace)
        {
            param_start += 1;
        }
        origline[param_start..param_end].to_string()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.responses.borrow_mut().push((msg.to_string(), log));
    }
    fn respond_raw(&self, _msg: &str) {
        // Raw output is not exercised by QUERY_ADC; responses under
        // test flow through respond_info (recorded above).
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

/// Fake `MCU` (opaque handle for `AdcPinApi::get_mcu`).
struct FakeMcu {
    printer: Printer,
    name: String,
}

impl FakeMcu {
    fn new(printer: Printer) -> Arc<Self> {
        Arc::new(FakeMcu {
            printer,
            name: "mcu".to_string(),
        })
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("setup_pin unsupported by fake mcu".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("lookup_command unsupported by fake mcu".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu("lookup_query_command unsupported by fake mcu".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        struct FakeAsync;
        impl AsyncResponseWrapperApi for FakeAsync {
            fn unregister(&self) {}
        }
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Mcu("get_constant_float unsupported by fake mcu".to_string()))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Mcu("create_trigger_dispatch unsupported by fake mcu".to_string()))
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
}

/// Fake `MCU_adc`: a fixed `(timestamp, value)` last state.
struct FakeAdcPin {
    state: GCell<(f64, f64)>,
    mcu: Mcu,
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_adc_sample(&self, _report_time: f64, _opts: AdcSampleOpts) {}
    fn setup_adc_callback(&self, _callback: AdcCallback) {}
    fn get_last_value(&self) -> (f64, f64) {
        self.state.get()
    }
}

// ---------------------------------------------------------------------------
// Oracle comparison.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r"
import gcode as gcode_mod
import extras.query_adc as query_adc_mod

RESULTS = []

def _respond_info(msg, log=True):
    RESULTS.append([msg, bool(log)])

def _respond_raw(msg):
    RESULTS.append(['__raw__:' + msg, False])

class _Dispatch:
    def respond_info(self, msg, log=True):
        _respond_info(msg, log)
    def respond_raw(self, msg):
        _respond_raw(msg)

class _FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.registrations = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            return self.handlers.pop(cmd, None)
        self.handlers[cmd] = func
        self.registrations.append([cmd, bool(when_not_ready), desc])

class _FakePrinter:
    def __init__(self):
        self.objects = {}
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is not None:
            return default
        raise KeyError(name)

class _FakeConfig:
    __slots__ = ('_printer',)
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

_fake_gcode = _FakeGCode()
printer = _FakePrinter()
printer.objects['gcode'] = _fake_gcode
q = query_adc_mod.QueryADC(_FakeConfig(printer))

class _ADC:
    def __init__(self, state):
        self._state = state
    def get_last_value(self):
        return self._state
";

/// One oracle scenario: pins to register (insertion order), then one
/// `QUERY_ADC` invocation with these gcode params and command line.
struct Case {
    label: &'static str,
    pins: Vec<(&'static str, f64, f64)>,
    params: Vec<(&'static str, &'static str)>,
    commandline: &'static str,
}

fn pynum(v: f64) -> String {
    // CPython repr of a float literal that evaluates to the same value.
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0.0 { "float('inf')".to_string() } else { "float('-inf')".to_string() }
    } else {
        format!("{v:?}")
    }
}

fn py_params(params: &[(&str, &str)]) -> String {
    let items: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect();
    format!("{{{}}}", items.join(", "))
}

fn run_oracle_case(py: &mut pyoracle::Oracle, case: &Case) -> pyoracle::OracleResult {
    let mut setup = String::new();
    for (name, ts, val) in &case.pins {
        setup.push_str(&format!(
            "q.register_adc({name:?}, _ADC(({ts}, {val})))\n",
            ts = pynum(*ts),
            val = pynum(*val),
        ));
    }
    let invocation = format!(
        "RESULTS.clear()\n\
         gcmd = gcode_mod.GCodeCommand(_Dispatch(), \"QUERY_ADC\", {cmdline:?}, {params}, False)\n\
         q.cmd_QUERY_ADC(gcmd)\n",
        cmdline = case.commandline,
        params = py_params(&case.params),
    );
    match py.exec(&setup) {
        Ok(_) => {}
        err => return err,
    }
    match py.exec(&invocation) {
        Ok(_) => py.eval("RESULTS"),
        err => err,
    }
}

struct RustResult {
    responses: Vec<(String, bool)>,
    error: Option<(String, String)>,
}

fn run_rust_case(case: &Case) -> RustResult {
    let reactor = Reactor::new(false);
    let printer_raw = FakePrinter::new(reactor.clone());
    let printer: Printer = printer_raw.clone();
    let gcode = FakeGCode::new(reactor);
    let gcode_obj: ObjectRef = gcode.clone();
    printer_raw.objects.borrow_mut().insert("gcode".to_string(), gcode_obj);
    let query = QueryAdc::new(printer.clone()).expect("construct QueryAdc");
    let mcu: Mcu = FakeMcu::new(printer.clone());
    for (name, ts, val) in &case.pins {
        let pin = Arc::new(FakeAdcPin {
            state: GCell::new((*ts, *val)),
            mcu: mcu.clone(),
        });
        query.register_adc(name, pin);
    }
    let params: IndexMap<String, String> = case
        .params
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let gcmd_fake = Arc::new(FakeGCodeCommand::new("QUERY_ADC", case.commandline, params));
    let gcmd: GCmd = gcmd_fake.clone();
    let result = gcode.dispatch("QUERY_ADC", &gcmd);
    let responses = gcmd_fake.responses.borrow().clone();
    match result {
        Ok(()) => RustResult {
            responses,
            error: None,
        },
        Err(e) => RustResult {
            responses,
            error: Some((e.class_name().to_string(), e.message().to_string())),
        },
    }
}

fn decode_responses(v: &Value) -> Vec<(String, bool)> {
    match v {
        Value::List(items) => items
            .iter()
            .map(|item| match item {
                Value::List(parts) => {
                    let msg = match &parts[0] {
                        Value::Str(s) => s.clone(),
                        other => panic!("expected message string, got {other}"),
                    };
                    let log = match &parts[1] {
                        Value::Bool(b) => *b,
                        other => panic!("expected log bool, got {other}"),
                    };
                    (msg, log)
                }
                other => panic!("expected [msg, log] pair, got {other}"),
            })
            .collect(),
        other => panic!("expected a list of responses, got {other}"),
    }
}

fn check_case(py: &mut pyoracle::Oracle, case: &Case) {
    let expected = run_oracle_case(py, case);
    let actual = run_rust_case(case);
    match expected {
        Ok(v) => {
            let expected_responses = decode_responses(&v);
            assert_eq!(
                actual.error, None,
                "{}: port raised {:?} but reference succeeded with {expected_responses:?}",
                case.label, actual.error
            );
            assert_eq!(
                actual.responses, expected_responses,
                "{}: respond_info divergence",
                case.label
            );
        }
        Err(exc) => {
            assert_eq!(
                actual.error,
                Some((exc.class_name.clone(), exc.message.clone())),
                "{}: port error divergence (responses: {:?})",
                case.label,
                actual.responses
            );
        }
    }
}

fn registration_cases() -> Vec<Case> {
    vec![
        Case {
            label: "missing NAME, empty registry",
            pins: vec![],
            params: vec![],
            commandline: "QUERY_ADC",
        },
        Case {
            label: "missing NAME lists sorted pins",
            pins: vec![("zeta", 1.0, 0.5), ("mypy", 2.0, 0.25), ("alpha", 3.0, 0.75)],
            params: vec![],
            commandline: "QUERY_ADC",
        },
        Case {
            label: "unknown NAME lists sorted pins",
            pins: vec![("zeta", 1.0, 0.5), ("mypy", 2.0, 0.25), ("alpha", 3.0, 0.75)],
            params: vec![("NAME", "unknown")],
            commandline: "QUERY_ADC NAME=unknown",
        },
        Case {
            label: "empty NAME lists pins",
            pins: vec![("mypy", 2.0, 0.25)],
            params: vec![("NAME", "")],
            commandline: "QUERY_ADC NAME=",
        },
        Case {
            label: "known pin, no PULLUP",
            pins: vec![("mypy", 123.456, 0.0123)],
            params: vec![("NAME", "mypy")],
            commandline: "QUERY_ADC NAME=mypi",
        },
        Case {
            label: "known pin, negative timestamp and value",
            pins: vec![("mypy", -5.25, -1.0)],
            params: vec![("NAME", "mypy")],
            commandline: "QUERY_ADC NAME=mypi",
        },
        Case {
            label: "PULLUP mid-range",
            pins: vec![("mypy", 1.0, 0.5)],
            params: vec![("NAME", "mypy"), ("PULLUP", "1000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=1000",
        },
        Case {
            label: "overwritten pin reports latest value",
            pins: vec![("mypy", 1.0, 0.1), ("mypy", 9.5, 0.9)],
            params: vec![("NAME", "mypy")],
            commandline: "QUERY_ADC NAME=mypi",
        },
        Case {
            label: "PULLUP clamps high value",
            pins: vec![("mypy", 1.0, 0.999999)],
            params: vec![("NAME", "mypy"), ("PULLUP", "4700")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=4700",
        },
        Case {
            label: "PULLUP clamps low value",
            pins: vec![("mypy", 1.0, -0.5)],
            params: vec![("NAME", "mypy"), ("PULLUP", "10000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=10000",
        },
        Case {
            label: "PULLUP with zero value",
            pins: vec![("mypy", 1.0, 0.0)],
            params: vec![("NAME", "mypy"), ("PULLUP", "1000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=1000",
        },
        Case {
            label: "PULLUP with unit value",
            pins: vec![("mypy", 1.0, 1.0)],
            params: vec![("NAME", "mypy"), ("PULLUP", "1000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=1000",
        },
        Case {
            label: "PULLUP with fractional value",
            pins: vec![("mypy", 12.3456, 0.3333)],
            params: vec![("NAME", "mypy"), ("PULLUP", "4.7")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=4.7",
        },
        Case {
            label: "nan timestamp and value",
            pins: vec![("mypy", f64::NAN, f64::NAN)],
            params: vec![("NAME", "mypy"), ("PULLUP", "1000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=1000",
        },
        Case {
            label: "inf timestamp and value",
            pins: vec![("mypy", f64::INFINITY, f64::INFINITY)],
            params: vec![("NAME", "mypy"), ("PULLUP", "1000")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=1000",
        },
        Case {
            label: "PULLUP zero is not above zero",
            pins: vec![("mypy", 1.0, 0.5)],
            params: vec![("NAME", "mypy"), ("PULLUP", "0")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=0",
        },
        Case {
            label: "PULLUP negative is not above zero",
            pins: vec![("mypy", 1.0, 0.5)],
            params: vec![("NAME", "mypy"), ("PULLUP", "-5")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=-5",
        },
        Case {
            label: "PULLUP unparseable",
            pins: vec![("mypy", 1.0, 0.5)],
            params: vec![("NAME", "mypy"), ("PULLUP", "abc")],
            commandline: "QUERY_ADC NAME=mypi PULLUP=abc",
        },
        Case {
            label: "unknown NAME returns before PULLUP parsing",
            pins: vec![("mypy", 1.0, 0.5)],
            params: vec![("NAME", "nope"), ("PULLUP", "abc")],
            commandline: "QUERY_ADC NAME=nope PULLUP=abc",
        },
    ]
}

/// Registration and construction parity: the constructor serves
/// `QUERY_ADC` with the exact help text, and `load_config` returns
/// a usable object on the reference side.
#[test]
fn registration_and_load_config_match_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping: KLIPPY_PYTHON is not set (outside the oracle image)");
        return;
    };
    py.exec(PY_SETUP)
        .unwrap_or_else(|e| panic!("oracle setup failed: {e:?}"));
    let registrations = py.eval("_fake_gcode.registrations").expect("registrations");
    let expected: Vec<Value> = vec![Value::List(vec![
        Value::Str("QUERY_ADC".to_string()),
        Value::Bool(false),
        Value::Str(CMD_QUERY_ADC_HELP.to_string()),
    ])];
    assert_eq!(
        registrations,
        Value::List(expected),
        "Python registration record"
    );
    let is_some = py
        .eval("query_adc_mod.load_config(_FakeConfig(printer)) is not None")
        .expect("load_config");
    assert_eq!(is_some, Value::Bool(true), "Python load_config returns QueryADC");

    // The port records the same single registration, built through
    // `load_config` exactly as the reference builds `QueryADC`
    // (`config.get_printer()` then `printer.lookup_object('gcode')`).
    let reactor = Reactor::new(false);
    let printer_raw = FakePrinter::new(reactor.clone());
    let printer: Printer = printer_raw.clone();
    let gcode = FakeGCode::new(reactor);
    let gcode_obj: ObjectRef = gcode.clone();
    printer_raw.objects.borrow_mut().insert("gcode".to_string(), gcode_obj);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: "query_adc".to_string(),
    });
    let loaded = (module_factory().load_config.expect("load_config present"))(&config)
        .expect("port load_config");
    assert!(loaded.is_some(), "port load_config returns QueryAdc");
    assert_eq!(
        gcode.registrations(),
        vec![(
            "QUERY_ADC".to_string(),
            false,
            Some(CMD_QUERY_ADC_HELP.to_string())
        )],
        "port registration record"
    );
    // And the dispatch serves through the stored handler.
    let dispatch_responses = {
        let params: IndexMap<String, String> = IndexMap::new();
        let gcmd_fake = Arc::new(FakeGCodeCommand::new("QUERY_ADC", "QUERY_ADC", params));
        let gcmd: GCmd = gcmd_fake.clone();
        gcode.dispatch("QUERY_ADC", &gcmd).expect("dispatch");
        let responses = gcmd_fake.responses.borrow().clone();
        responses
    };
    assert_eq!(
        dispatch_responses,
        vec![("Available ADC objects: ".to_string(), true)],
    );
    // The handler responds through the `gcmd` (bound `respond_info`),
    // never through the dispatch's own output channel.
    assert!(
        gcode.raw.borrow().is_empty(),
        "dispatch output channel must stay empty"
    );
}

/// `module_factory` exposes exactly the Python entry points.
#[test]
fn module_factory_matches_python() {
    let factory: ModuleFactory = module_factory();
    assert!(factory.load_config.is_some(), "load_config present");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn query_adc_matches_python() {
    let cases = registration_cases();
    for case in &cases {
        let Some(mut py) = pyoracle::Oracle::start() else {
            println!("skipping: KLIPPY_PYTHON is not set (outside the oracle image)");
            return;
        };
        py.exec(PY_SETUP)
            .unwrap_or_else(|e| panic!("oracle setup failed: {e:?}"));
        check_case(&mut py, case);
    }
}
