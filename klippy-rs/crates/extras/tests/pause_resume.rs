//! Parity tests for `extras::pause_resume` (port of `klippy/extras/pause_resume.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PauseResume` is built
//! on both sides with minimal Python fakes for the printer, config, gcode,
//! webhooks, virtual_sdcard and mcu, then fed identical gcode commands,
//! webhook endpoint calls and `klippy:connect` events; the resulting side
//! effects (gcode scripts, `respond_info` lines, virtual_sdcard calls and
//! webhook `send` payloads), `get_status` dicts and `recover_velocity`
//! values are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::pause_resume::{module_factory, PauseResume};
use extras::virtual_sdcard::VirtualSDApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GcmdGetFloatOpts,
    GetIntOpts as GcmdGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::McuApi;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{
    ClientConnection, WebHooksApi, WebHooksCallback, WebRequest, WebRequestApi,
};
use klippy_api::{downcast_arc, EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.pause_resume as pause_resume

SENTINEL = object()

class CommandError(Exception):
    pass

class FakeGCode:
    def __init__(self):
        self.cmds = {}
        self.helps = {}
        self.scripts = []
        self.from_cmd_scripts = []
        self.responses = []
        self.registered = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append((cmd, desc))
        self.cmds[cmd] = func
        self.helps[cmd] = desc
    def respond_info(self, msg, log=True):
        self.responses.append(('respond_info', msg, log))
    def respond_raw(self, msg):
        self.responses.append(('respond_raw', msg))
    def run_script(self, script):
        self.scripts.append(script)
    def run_script_from_command(self, script):
        self.from_cmd_scripts.append(script)

class FakeGCodeCommand:
    def __init__(self, gcode, command, commandline, params):
        self._gcode = gcode
        self._command = command
        self._commandline = commandline
        self._params = params
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return self._params
    def respond_info(self, msg, log=True):
        self._gcode.respond_info(msg, log)
    def respond_raw(self, msg):
        self._gcode.respond_raw(msg)
    def get_float(self, name, default=SENTINEL, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is SENTINEL:
                raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = float(value)
        except Exception:
            raise CommandError("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise CommandError("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise CommandError("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise CommandError("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise CommandError("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value

class FakeWebHooks:
    def __init__(self):
        self.endpoints = {}
        self.registered = []
    def register_endpoint(self, path, callback):
        if path in self.endpoints:
            raise Exception("Path already registered to an endpoint")
        self.endpoints[path] = callback
        self.registered.append(path)

class FakeWebRequest:
    def __init__(self):
        self.response = None
        self.is_error = False
    def send(self, data):
        if self.response is not None:
            raise Exception("Multiple calls to send not allowed")
        self.response = data

class FakeVirtualSD:
    def __init__(self):
        self.active = False
        self.calls = []
    def is_active(self):
        return self.active
    def do_pause(self):
        self.calls.append('do_pause')
    def do_resume(self):
        self.calls.append('do_resume')
    def do_cancel(self):
        self.calls.append('do_cancel')

class FakeMCU:
    def __init__(self, constants):
        self._constants = constants
    def get_constants(self):
        return self._constants

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, objects):
        self.objects = objects
        self.handlers = {}
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, callback):
        self.handlers[event] = callback

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options):
        self._printer = printer
        self._options = options
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'pause_resume'
    def getfloat(self, option, default=SENTINEL):
        if option not in self._options:
            if default is SENTINEL:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, 'pause_resume'))
            return default
        try:
            return float(self._options[option])
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, 'pause_resume'))

def make_gcmd(gcode, command, commandline, params):
    return FakeGCodeCommand(gcode, command, commandline, params)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Log {
    scripts: Arc<Mutex<Vec<String>>>,
    from_cmd_scripts: Arc<Mutex<Vec<String>>>,
    responses: Arc<Mutex<Vec<(String, String, bool)>>>,
    registered_cmds: Arc<Mutex<Vec<(String, Option<String>)>>>,
}

impl Default for Log {
    fn default() -> Log {
        Log {
            scripts: Arc::new(Mutex::new(Vec::new())),
            from_cmd_scripts: Arc::new(Mutex::new(Vec::new())),
            responses: Arc::new(Mutex::new(Vec::new())),
            registered_cmds: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

fn pvs(values: &[String]) -> Vec<Value> {
    values.iter().map(|v| Value::str(v)).collect()
}

#[derive(Clone)]
struct FakeGCode {
    log: Log,
    handlers: Arc<Mutex<IndexMap<String, GCodeCommandHandler>>>,
}

impl Default for FakeGCode {
    fn default() -> FakeGCode {
        FakeGCode { log: Log::default(), handlers: Arc::new(Mutex::new(IndexMap::new())) }
    }
}

impl FakeGCode {
    fn snapshot(&self) -> Value {
        let scripts = self.log.scripts.lock().clone();
        let from_cmd = self.log.from_cmd_scripts.lock().clone();
        let responses = self.log.responses.lock().clone();
        Value::Dict(vec![
            (Value::str("scripts"), Value::List(pvs(&scripts))),
            (Value::str("from_cmd_scripts"), Value::List(pvs(&from_cmd))),
            (
                Value::str("responses"),
                Value::List(
                    responses
                        .iter()
                        .map(|(kind, msg, log)| {
                            Value::Tuple(vec![
                                Value::str(kind),
                                Value::str(msg),
                                Value::Bool(*log),
                            ])
                        })
                        .collect(),
                ),
            ),
        ])
    }
    fn run_handler(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        let handler = handlers.get_mut(cmd).expect("command registered");
        handler(gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut handlers = self.handlers.lock();
        if func.is_none() {
            return Ok(handlers.shift_remove(cmd));
        }
        if handlers.contains_key(cmd) {
            return Err(KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        self.log
            .registered_cmds
            .lock()
            .push((cmd.to_string(), opts.desc.clone()));
        handlers.insert(cmd.to_string(), func.unwrap());
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
        self.log.scripts.lock().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.log.from_cmd_scripts.lock().push(script.to_string());
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
    ) -> GCmd {
        panic!("create_gcode_command is not used by pause_resume")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        self.log
            .responses
            .lock()
            .push(("respond_info".to_string(), msg.to_string(), log));
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

#[derive(Clone)]
struct FakeGCodeCommand {
    gcode: FakeGCode,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCodeCommand {
    fn new(
        gcode: FakeGCode,
        command: &str,
        commandline: &str,
        params: &[(&str, &str)],
    ) -> FakeGCodeCommand {
        let mut map = IndexMap::new();
        for (k, v) in params {
            map.insert(k.to_string(), v.to_string());
        }
        FakeGCodeCommand {
            gcode,
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: map,
        }
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<PyValue> {
        unimplemented!()
    }
    fn get_int(&self, _name: &str, _opts: GcmdGetIntOpts) -> KResult<Option<i64>> {
        unimplemented!()
    }
    fn get_float(&self, name: &str, opts: GcmdGetFloatOpts) -> KResult<Option<f64>> {
        let Some(value) = self.params.get(name) else {
            match opts.default {
                Some(Some(v)) => return Ok(Some(v)),
                Some(None) => return Ok(None),
                None => {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': missing {}",
                        self.commandline, name
                    )));
                }
            }
        };
        let parsed: f64 = match value.parse() {
            Ok(v) if !value.trim().is_empty() => v,
            _ => {
                return Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, value
                )));
            }
        };
        if let Some(minval) = opts.minval {
            if parsed < minval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    pycompat::text::float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if parsed > maxval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    pycompat::text::float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if parsed <= above {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    pycompat::text::float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if parsed >= below {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    pycompat::text::float_str(below)
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
        self.gcode
            .log
            .responses
            .lock()
            .push(("respond_info".to_string(), msg.to_string(), log));
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

#[derive(Clone)]
struct FakeWebHooks {
    endpoints: Arc<Mutex<IndexMap<String, WebHooksCallback>>>,
    registered: Arc<Mutex<Vec<String>>>,
}

impl Default for FakeWebHooks {
    fn default() -> FakeWebHooks {
        FakeWebHooks {
            endpoints: Arc::new(Mutex::new(IndexMap::new())),
            registered: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl FakeWebHooks {
    fn run_endpoint(&self, path: &str, req: WebRequest) -> KResult<()> {
        let map = self.endpoints.lock();
        let cb = map.get(path).expect("endpoint registered");
        cb(req)
    }
}

impl WebHooksApi for FakeWebHooks {
    fn register_endpoint(&self, path: &str, callback: WebHooksCallback) -> KResult<()> {
        let mut map = self.endpoints.lock();
        if map.contains_key(path) {
            return Err(KlippyError::Other {
                class_name: "WebRequestError".to_string(),
                message: "Path already registered to an endpoint".to_string(),
            });
        }
        self.registered.lock().push(path.to_string());
        map.insert(path.to_string(), callback);
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
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

#[derive(Clone)]
struct FakeWebRequest {
    sent: Arc<Mutex<Option<PyValue>>>,
}

impl FakeWebRequest {
    fn new() -> FakeWebRequest {
        FakeWebRequest { sent: Arc::new(Mutex::new(None)) }
    }
    fn sent_value(&self) -> Value {
        let sent = self.sent.lock().clone();
        match sent {
            None => Value::Null,
            Some(PyValue::Dict(d)) => Value::Dict(
                d.iter()
                    .map(|(k, v)| {
                        let key = match k {
                            PyValue::Str(s) => Value::str(s),
                            other => Value::str(&other.str()),
                        };
                        let val = match v {
                            PyValue::Bool(b) => Value::Bool(*b),
                            PyValue::Str(s) => Value::str(s),
                            other => Value::str(&other.str()),
                        };
                        (key, val)
                    })
                    .collect(),
            ),
            Some(other) => Value::str(&other.str()),
        }
    }
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> ClientConnection {
        panic!("client_conn is not used by pause_resume")
    }
    fn id(&self) -> Option<PyValue> {
        None
    }
    fn method(&self) -> String {
        String::new()
    }
    fn params(&self) -> PyValue {
        PyValue::None
    }
    fn response(&self) -> Option<PyValue> {
        None
    }
    fn is_error(&self) -> bool {
        false
    }
    fn get_client_connection(&self) -> ClientConnection {
        panic!("get_client_connection is not used by pause_resume")
    }
    fn get(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        unimplemented!()
    }
    fn get_str(&self, _item: &str, _default: Option<String>) -> KResult<String> {
        unimplemented!()
    }
    fn get_int(&self, _item: &str, _default: Option<i64>) -> KResult<i64> {
        unimplemented!()
    }
    fn get_float(&self, _item: &str, _default: Option<f64>) -> KResult<f64> {
        unimplemented!()
    }
    fn get_dict(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        unimplemented!()
    }
    fn get_method(&self) -> String {
        String::new()
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        let mut sent = self.sent.lock();
        if sent.is_some() {
            return Err(KlippyError::Other {
                class_name: "WebRequestError".to_string(),
                message: "Multiple calls to send not allowed".to_string(),
            });
        }
        *sent = Some(data);
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        None
    }
}

#[derive(Clone)]
struct FakeVirtualSD {
    calls: Arc<Mutex<Vec<String>>>,
    active: Arc<std::sync::atomic::AtomicBool>,
    file_position: Arc<std::sync::atomic::AtomicI64>,
}

impl Default for FakeVirtualSD {
    fn default() -> FakeVirtualSD {
        FakeVirtualSD {
            calls: Arc::new(Mutex::new(Vec::new())),
            active: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            file_position: Arc::new(std::sync::atomic::AtomicI64::new(0)),
        }
    }
}

impl FakeVirtualSD {
    fn set_active(&self, active: bool) {
        self.active.store(active, std::sync::atomic::Ordering::SeqCst);
    }
    fn calls_value(&self) -> Value {
        let calls = self.calls.lock().clone();
        Value::List(calls.iter().map(|c| Value::str(c)).collect())
    }
}

impl VirtualSDApi for FakeVirtualSD {
    fn get_file_list(&self, _check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        Ok(Vec::new())
    }
    fn is_active(&self) -> bool {
        self.active.load(std::sync::atomic::Ordering::SeqCst)
    }
    fn do_pause(&self) {
        self.calls.lock().push("do_pause".to_string());
    }
    fn do_resume(&self) -> KResult<()> {
        self.calls.lock().push("do_resume".to_string());
        Ok(())
    }
    fn do_cancel(&self) -> KResult<()> {
        self.calls.lock().push("do_cancel".to_string());
        Ok(())
    }
    fn is_cmd_from_sd(&self) -> bool {
        false
    }
    fn get_file_position(&self) -> i64 {
        self.file_position.load(std::sync::atomic::Ordering::SeqCst)
    }
    fn set_file_position(&self, pos: i64) {
        self.file_position.store(pos, std::sync::atomic::Ordering::SeqCst);
    }
}

#[derive(Clone)]
struct FakeMcu {
    constants: IndexMap<String, String>,
}

impl Default for FakeMcu {
    fn default() -> FakeMcu {
        let mut constants = IndexMap::new();
        constants.insert("software_version".to_string(), "v1.2.3".to_string());
        FakeMcu { constants }
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by pause_resume")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: klippy_api::mcu::PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        unimplemented!()
    }
    fn create_oid(&self) -> KResult<i64> {
        unimplemented!()
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        unimplemented!()
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        unimplemented!()
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        unimplemented!()
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        unimplemented!()
    }
    fn min_schedule_time(&self) -> f64 {
        unimplemented!()
    }
    fn max_nominal_duration(&self) -> f64 {
        unimplemented!()
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        unimplemented!()
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        unimplemented!()
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        unimplemented!()
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        unimplemented!()
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.constants.clone()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        unimplemented!()
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        unimplemented!()
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        unimplemented!()
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        unimplemented!()
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        unimplemented!()
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        unimplemented!()
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        unimplemented!()
    }
}

/// One printer object serving whichever interface the module asks for.
#[derive(Clone, Default)]
struct FakeObject {
    gcode: FakeGCode,
    webhooks: FakeWebHooks,
    sd: FakeVirtualSD,
    mcu: FakeMcu,
}

impl FakeObject {
    fn with_gcode(gcode: FakeGCode) -> FakeObject {
        FakeObject { gcode, ..Default::default() }
    }
    fn with_webhooks(webhooks: FakeWebHooks) -> FakeObject {
        FakeObject { webhooks, ..Default::default() }
    }
    fn with_sd(sd: FakeVirtualSD) -> FakeObject {
        FakeObject { sd, ..Default::default() }
    }
    fn with_mcu(mcu: FakeMcu) -> FakeObject {
        FakeObject { mcu, ..Default::default() }
    }
}

macro_rules! delegate_gcode {
    () => {
        fn register_command(
            &self,
            cmd: &str,
            func: Option<GCodeCommandHandler>,
            opts: RegisterCommandOpts,
        ) -> KResult<Option<GCodeCommandHandler>> {
            self.gcode.register_command(cmd, func, opts)
        }
        fn register_mux_command(
            &self,
            cmd: &str,
            key: &str,
            value: Option<&str>,
            func: GCodeCommandHandler,
            desc: Option<&str>,
        ) -> KResult<()> {
            self.gcode.register_mux_command(cmd, key, value, func, desc)
        }
        fn get_command_help(&self) -> IndexMap<String, String> {
            self.gcode.get_command_help()
        }
        fn get_status(&self, eventtime: f64) -> PyValue {
            self.gcode.get_status(eventtime)
        }
        fn register_output_handler(&self, callback: klippy_api::gcode::OutputCallback) {
            self.gcode.register_output_handler(callback)
        }
        fn request_restart(&self, result: &str) -> KResult<()> {
            self.gcode.request_restart(result)
        }
        fn run_script(&self, script: &str) -> KResult<()> {
            self.gcode.run_script(script)
        }
        fn run_script_from_command(&self, script: &str) -> KResult<()> {
            self.gcode.run_script_from_command(script)
        }
        fn get_mutex(&self) -> reactor::ReactorMutex {
            self.gcode.get_mutex()
        }
        fn create_gcode_command(
            &self,
            command: &str,
            commandline: &str,
            params: &IndexMap<String, String>,
        ) -> GCmd {
            self.gcode.create_gcode_command(command, commandline, params)
        }
        fn respond_raw(&self, msg: &str) {
            self.gcode.respond_raw(msg)
        }
        fn respond_info(&self, msg: &str, log: bool) {
            self.gcode.respond_info(msg, log)
        }
        fn is_traditional_gcode(&self, cmd: &str) -> bool {
            self.gcode.is_traditional_gcode(cmd)
        }
        fn cmd_m112(&self, gcmd: Option<&GCmd>) -> KResult<()> {
            self.gcode.cmd_m112(gcmd)
        }
        fn process_commands(&self, commands: &[String], need_ack: bool) -> KResult<()> {
            self.gcode.process_commands(commands, need_ack)
        }
    };
}

impl GCodeApi for FakeObject {
    delegate_gcode!();
}

impl WebHooksApi for FakeObject {
    fn register_endpoint(&self, path: &str, callback: WebHooksCallback) -> KResult<()> {
        self.webhooks.register_endpoint(path, callback)
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        self.webhooks.register_mux_endpoint(path, key, value, callback)
    }
    fn call_remote_method(
        &self,
        method: &str,
        params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        self.webhooks.call_remote_method(method, params)
    }
    fn get_status(&self, eventtime: f64) -> PyValue {
        self.webhooks.get_status(eventtime)
    }
}

impl VirtualSDApi for FakeObject {
    fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        self.sd.get_file_list(check_subdirs)
    }
    fn is_active(&self) -> bool {
        self.sd.is_active()
    }
    fn do_pause(&self) {
        self.sd.do_pause()
    }
    fn do_resume(&self) -> KResult<()> {
        self.sd.do_resume()
    }
    fn do_cancel(&self) -> KResult<()> {
        self.sd.do_cancel()
    }
    fn is_cmd_from_sd(&self) -> bool {
        self.sd.is_cmd_from_sd()
    }
    fn get_file_position(&self) -> i64 {
        self.sd.get_file_position()
    }
    fn set_file_position(&self, pos: i64) {
        self.sd.set_file_position(pos)
    }
}

impl McuApi for FakeObject {
    fn get_name(&self) -> String {
        self.mcu.get_name()
    }
    fn get_printer(&self) -> Printer {
        self.mcu.get_printer()
    }
    fn is_fileoutput(&self) -> bool {
        self.mcu.is_fileoutput()
    }
    fn setup_pin(
        &self,
        pin_type: &str,
        pin_params: klippy_api::mcu::PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        self.mcu.setup_pin(pin_type, pin_params)
    }
    fn create_oid(&self) -> KResult<i64> {
        self.mcu.create_oid()
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.mcu.register_config_callback(cb)
    }
    fn add_config_cmd(&self, cmd: &str, opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        self.mcu.add_config_cmd(cmd, opts)
    }
    fn request_move_queue_slot(&self) {
        self.mcu.request_move_queue_slot()
    }
    fn get_query_slot(&self, oid: i64) -> i64 {
        self.mcu.get_query_slot(oid)
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        self.mcu.seconds_to_clock(time)
    }
    fn min_schedule_time(&self) -> f64 {
        self.mcu.min_schedule_time()
    }
    fn max_nominal_duration(&self) -> f64 {
        self.mcu.max_nominal_duration()
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        self.mcu.lookup_command(msgformat, cq)
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        self.mcu.lookup_query_command(msgformat, respformat, opts)
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        self.mcu.try_lookup_command(msgformat)
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        self.mcu.alloc_command_queue()
    }
    fn register_serial_response(
        &self,
        cb: klippy_api::mcu::SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        self.mcu.register_serial_response(cb, msg, oid)
    }
    fn check_valid_response(&self, msgformat: &str) -> bool {
        self.mcu.check_valid_response(msgformat)
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.mcu.get_enumerations()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.mcu.get_constants()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        self.mcu.get_constant_float(name)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        self.mcu.print_time_to_clock(print_time)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        self.mcu.clock_to_print_time(clock)
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.mcu.estimated_print_time(eventtime)
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.mcu.clock32_to_clock64(clock32)
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        self.mcu.calibrate_clock(print_time, eventtime)
    }
    fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        self.mcu.get_status(eventtime)
    }
    fn stats(&self, eventtime: f64) -> (bool, String) {
        self.mcu.stats(eventtime)
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        self.mcu.serialqueue()
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        self.mcu.create_trigger_dispatch()
    }
}

impl PrinterObject for FakeObject {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(
        dyn GCodeApi,
        dyn WebHooksApi,
        dyn VirtualSDApi,
        dyn McuApi
    );
}

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, FakeObject>>>,
    handlers: Arc<Mutex<Vec<(String, EventCallback)>>>,
}

impl Default for FakePrinter {
    fn default() -> FakePrinter {
        FakePrinter {
            objects: Arc::new(Mutex::new(IndexMap::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
        }
    }
}

impl FakePrinter {
    fn insert(&self, name: &str, obj: FakeObject) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn fire_connect(&self) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        for (event, cb) in handlers.iter_mut() {
            if event == "klippy:connect" {
                cb(&[])?;
            }
        }
        Ok(())
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
    fn add_object(&self, name: &str, _obj: ObjectRef) -> KResult<()> {
        let _ = name;
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let found = self.objects.lock().get(name).cloned();
        match found {
            Some(obj) => Ok(Some(Arc::new(obj) as ObjectRef)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
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
    options: IndexMap<String, String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "pause_resume".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.options.get(option) {
            Some(value) => match value.parse::<f64>() {
                Ok(v) if !value.trim().is_empty() => Ok(Some(v)),
                _ => Err(KlippyError::config(format!(
                    "Unable to parse option '{option}' in section 'pause_resume'"
                ))),
            },
            None => match opts.default {
                Some(Some(v)) => Ok(Some(v)),
                Some(None) => Ok(None),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section 'pause_resume' must be specified"
                ))),
            },
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by pause_resume")
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
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pause_resume fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pause_resume oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "pause_resume".to_string(),
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

fn check_err<E: std::fmt::Debug>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<E>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v:?} but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class",
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message",
    );
}

struct Ctx {
    pause: PauseResume,
    gcode: FakeGCode,
    webhooks: FakeWebHooks,
    sd: FakeVirtualSD,
    printer: Arc<FakePrinter>,
}

fn rust_ctx(with_sd: bool, recover: Option<&str>) -> Ctx {
    let gcode = FakeGCode::default();
    let webhooks = FakeWebHooks::default();
    let sd = FakeVirtualSD::default();
    let printer = Arc::new(FakePrinter::default());
    printer.insert("gcode", FakeObject::with_gcode(gcode.clone()));
    printer.insert("webhooks", FakeObject::with_webhooks(webhooks.clone()));
    if with_sd {
        printer.insert("virtual_sdcard", FakeObject::with_sd(sd.clone()));
    }
    printer.insert("mcu", FakeObject::with_mcu(FakeMcu::default()));
    let printer_arc: Printer = printer.clone();
    let mut options = IndexMap::new();
    if let Some(v) = recover {
        options.insert("recover_velocity".to_string(), v.to_string());
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc,
        options,
    });
    let pause = PauseResume::new(&config).expect("rust PauseResume::new");
    Ctx { pause, gcode, webhooks, sd, printer }
}

/// Build the Python-side object with the same fakes in the oracle namespace.
fn py_build(py: &mut pyoracle::Oracle, with_sd: bool, recover: Option<&str>) {
    let opts = match recover {
        Some(v) => format!("{{'recover_velocity': {v:?}}}"),
        None => "{}".to_string(),
    };
    let sd_stmt = if with_sd { "objects['virtual_sdcard'] = vsd;" } else { "" };
    let setup = format!(
        "g = FakeGCode(); wh = FakeWebHooks(); vsd = FakeVirtualSD(); mcu = FakeMCU({{'software_version': 'v1.2.3'}}); objects = {{'gcode': g, 'webhooks': wh, 'mcu': mcu}}; {sd_stmt} p = FakePrinter(objects); c = FakeConfig(p, {opts}); obj = pause_resume.load_config(c)"
    );
    py.exec(&setup).expect("py load_config");
}

fn fire_connect(py: &mut pyoracle::Oracle, ctx: &Ctx) {
    py.exec("p.handlers['klippy:connect']()").expect("py handle_connect");
    ctx.printer.fire_connect().expect("rust handle_connect");
}
fn status_value(pause: &PauseResume) -> Value {
    use klippy_api::PrinterObject as _;
    match pause.get_status(0.0) {
        Some(PyValue::Dict(d)) => Value::Dict(
            d.iter()
                .map(|(k, v)| {
                    let key = match k {
                        PyValue::Str(s) => Value::str(s),
                        other => Value::str(&other.str()),
                    };
                    let val = match v {
                        PyValue::Bool(b) => Value::Bool(*b),
                        other => Value::str(&other.str()),
                    };
                    (key, val)
                })
                .collect(),
        ),
        other => Value::str(&format!("{other:?}")),
    }
}

fn gcmd(ctx: &Ctx, command: &str, commandline: &str, params: &[(&str, &str)]) -> GCmd {
    Arc::new(FakeGCodeCommand::new(ctx.gcode.clone(), command, commandline, params))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constructor_registers_commands_endpoints_and_defaults() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, true, None);
    let ctx = rust_ctx(true, None);

    let expected_registered = py
        .eval("[(cmd, desc) for cmd, desc in g.registered]")
        .expect("eval registered commands");
    let actual_registered = Value::List(
        ctx.gcode
            .log
            .registered_cmds
            .lock()
            .iter()
            .map(|(cmd, desc)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    match desc {
                        Some(d) => Value::str(d),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    );
    check("registered_commands", Ok(expected_registered), actual_registered);

    let expected_endpoints = py.eval("list(wh.registered)").expect("eval endpoints");
    let actual_endpoints = Value::List(
        ctx.webhooks
            .registered
            .lock()
            .iter()
            .map(|e| Value::str(e))
            .collect(),
    );
    check("registered_endpoints", Ok(expected_endpoints), actual_endpoints);

    let expected_recover = py.eval("obj.recover_velocity").expect("eval recover_velocity");
    check("recover_velocity_default", Ok(expected_recover), Value::float(ctx.pause.recover_velocity()));

    let expected_status = py.eval("obj.get_status(0.0)").expect("eval get_status");
    check("get_status_initial", Ok(expected_status), status_value(&ctx.pause));
}

fn py_gcode(py: &mut pyoracle::Oracle) -> Value {
    py.eval("{'scripts': g.scripts, 'from_cmd_scripts': g.from_cmd_scripts, 'responses': g.responses}")
        .expect("eval side effects")
}

fn snapshot_pair(label: &str, py: &mut pyoracle::Oracle, ctx: &Ctx) {
    check(label, Ok(py_gcode(py)), ctx.gcode.snapshot());
    check(label, Ok(py.eval("vsd.calls").expect("eval sd calls")), ctx.sd.calls_value());
    check(
        label,
        Ok(py.eval("obj.get_status(0.0)").expect("eval status")),
        status_value(&ctx.pause),
    );
}

#[test]
fn pause_then_resume_without_sd() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");
    snapshot_pair("after_pause", &mut py, &ctx);

    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE again");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE again");
    snapshot_pair("after_second_pause", &mut py, &ctx);

    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME', {}))").expect("py RESUME");
    ctx.gcode
        .run_handler("RESUME", &gcmd(&ctx, "RESUME", "RESUME", &[]))
        .expect("rust RESUME");
    snapshot_pair("after_resume", &mut py, &ctx);
}

#[test]
fn resume_aborts_when_not_paused() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME VELOCITY=100', {'VELOCITY': '100'}))")
        .expect("py RESUME abort");
    ctx.gcode
        .run_handler(
            "RESUME",
            &gcmd(&ctx, "RESUME", "RESUME VELOCITY=100", &[("VELOCITY", "100")]),
        )
        .expect("rust RESUME abort");
    snapshot_pair("resume_aborted", &mut py, &ctx);
}


#[test]
fn resume_velocity_variants() {
    let cases: &[(&str, &[(&str, &str)])] = &[
        ("RESUME", &[]),
        ("RESUME VELOCITY=125.5", &[("VELOCITY", "125.5")]),
        ("RESUME VELOCITY=0", &[("VELOCITY", "0")]),
        ("RESUME VELOCITY=-3.25", &[("VELOCITY", "-3.25")]),
        ("RESUME VELOCITY=12.345678", &[("VELOCITY", "12.345678")]),
        ("RESUME VELOCITY=1000000", &[("VELOCITY", "1000000")]),
    ];
    for (commandline, params) in cases {
        let Some(mut py) = oracle() else { return };
        py_build(&mut py, false, None);
        let ctx = rust_ctx(false, None);
        fire_connect(&mut py, &ctx);
        py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
        ctx.gcode
            .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
            .expect("rust PAUSE");
        let dict = params
            .iter()
            .map(|(k, v)| format!("{k:?}: {v:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        let params_py = format!("{{{dict}}}");
        py.exec(&format!(
            "obj.cmd_RESUME(make_gcmd(g, 'RESUME', {commandline:?}, {params_py}))"
        ))
        .expect("py RESUME");
        ctx.gcode
            .run_handler("RESUME", &gcmd(&ctx, "RESUME", commandline, params))
            .expect("rust RESUME");
        snapshot_pair(&format!("resume_velocity_{commandline}"), &mut py, &ctx);
    }
}

#[test]
fn resume_invalid_velocity_raises_command_error() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");

    let expected = py.eval(
        "obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME VELOCITY=abc', {'VELOCITY': 'abc'}))",
    );
    let actual = ctx.gcode.run_handler(
        "RESUME",
        &gcmd(&ctx, "RESUME", "RESUME VELOCITY=abc", &[("VELOCITY", "abc")]),
    );
    check_err("resume_invalid_velocity", expected, actual);
    snapshot_pair("after_failed_resume", &mut py, &ctx);
}

#[test]
fn clear_pause_allows_pause_again() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");
    py.exec("obj.cmd_CLEAR_PAUSE(make_gcmd(g, 'CLEAR_PAUSE', 'CLEAR_PAUSE', {}))")
        .expect("py CLEAR_PAUSE");
    ctx.gcode
        .run_handler("CLEAR_PAUSE", &gcmd(&ctx, "CLEAR_PAUSE", "CLEAR_PAUSE", &[]))
        .expect("rust CLEAR_PAUSE");
    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME', {}))").expect("py RESUME aborted");
    ctx.gcode
        .run_handler("RESUME", &gcmd(&ctx, "RESUME", "RESUME", &[]))
        .expect("rust RESUME aborted");
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE again");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE again");
    snapshot_pair("after_clear_and_repause", &mut py, &ctx);
}

#[test]
fn cancel_print_without_sd() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");
    py.exec("obj.cmd_CANCEL_PRINT(make_gcmd(g, 'CANCEL_PRINT', 'CANCEL_PRINT', {}))")
        .expect("py CANCEL_PRINT");
    ctx.gcode
        .run_handler("CANCEL_PRINT", &gcmd(&ctx, "CANCEL_PRINT", "CANCEL_PRINT", &[]))
        .expect("rust CANCEL_PRINT");
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE after cancel");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE after cancel");
    snapshot_pair("after_cancel_and_repause", &mut py, &ctx);
}

#[test]
fn direct_send_commands() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    // `send_pause_command` is idempotent: the second call emits nothing.
    py.exec("obj.send_pause_command(); obj.send_pause_command()")
        .expect("py send_pause twice");
    ctx.pause.send_pause_command().expect("rust send_pause");
    ctx.pause.send_pause_command().expect("rust send_pause again");
    snapshot_pair("after_send_pause_twice", &mut py, &ctx);

    // PAUSE after an external pause command saves state but does not re-emit.
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");
    py.exec("obj.send_resume_command()").expect("py send_resume");
    ctx.pause.send_resume_command().expect("rust send_resume");
    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME', {}))").expect("py RESUME");
    ctx.gcode
        .run_handler("RESUME", &gcmd(&ctx, "RESUME", "RESUME", &[]))
        .expect("rust RESUME");
    snapshot_pair("after_send_resume_and_resume", &mut py, &ctx);
}

#[test]
fn sd_active_pause_resume_cycle() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, true, None);
    let ctx = rust_ctx(true, None);
    fire_connect(&mut py, &ctx);

    py.exec("vsd.active = True").expect("py activate sd");
    ctx.sd.set_active(true);
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE sd");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE sd");
    snapshot_pair("after_sd_pause", &mut py, &ctx);

    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME', {}))").expect("py RESUME sd");
    ctx.gcode
        .run_handler("RESUME", &gcmd(&ctx, "RESUME", "RESUME", &[]))
        .expect("rust RESUME sd");
    snapshot_pair("after_sd_resume", &mut py, &ctx);
}

#[test]
fn sd_inactive_falls_back_to_gcode_response() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, true, None);
    let ctx = rust_ctx(true, None);
    fire_connect(&mut py, &ctx);
    // `virtual_sdcard` exists but is not printing: `is_sd_active()` is
    // False, so both sides emit the plain `action:paused` response.
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE inactive sd");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE inactive sd");
    snapshot_pair("after_inactive_sd_pause", &mut py, &ctx);
}

#[test]
fn cancel_print_with_sd_active() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, true, None);
    let ctx = rust_ctx(true, None);
    fire_connect(&mut py, &ctx);

    py.exec("vsd.active = True").expect("py activate sd");
    ctx.sd.set_active(true);
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE sd");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE sd");
    py.exec("obj.cmd_CANCEL_PRINT(make_gcmd(g, 'CANCEL_PRINT', 'CANCEL_PRINT', {}))")
        .expect("py CANCEL_PRINT sd");
    ctx.gcode
        .run_handler("CANCEL_PRINT", &gcmd(&ctx, "CANCEL_PRINT", "CANCEL_PRINT", &[]))
        .expect("rust CANCEL_PRINT sd");
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE after sd cancel");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE after sd cancel");
    snapshot_pair("after_sd_cancel_and_repause", &mut py, &ctx);
}

#[test]
fn connect_without_sdcard_object() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);
    check(
        "is_sd_active_without_sd",
        Ok(py.eval("obj.is_sd_active()").expect("eval is_sd_active")),
        Value::Bool(ctx.pause.is_sd_active()),
    );
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE no sd");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE no sd");
    snapshot_pair("after_no_sd_pause", &mut py, &ctx);
}

#[test]
fn webhooks_run_gcode_scripts() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("wr = FakeWebRequest(); wh.endpoints['pause_resume/pause'](wr)").expect("py pause endpoint");
    ctx.webhooks
        .run_endpoint("pause_resume/pause", Arc::new(FakeWebRequest::new()) as WebRequest)
        .expect("rust pause endpoint");
    py.exec("wr = FakeWebRequest(); wh.endpoints['pause_resume/resume'](wr)").expect("py resume endpoint");
    ctx.webhooks
        .run_endpoint("pause_resume/resume", Arc::new(FakeWebRequest::new()) as WebRequest)
        .expect("rust resume endpoint");
    py.exec("wr = FakeWebRequest(); wh.endpoints['pause_resume/cancel'](wr)").expect("py cancel endpoint");
    ctx.webhooks
        .run_endpoint("pause_resume/cancel", Arc::new(FakeWebRequest::new()) as WebRequest)
        .expect("rust cancel endpoint");
    snapshot_pair("after_webhook_scripts", &mut py, &ctx);
}

#[test]
fn webhooks_check_continue_and_bootloader() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    fire_connect(&mut py, &ctx);

    py.exec("wr = FakeWebRequest(); wh.endpoints['pause_resume/check_continue_print_state'](wr)")
        .expect("py check endpoint");
    let rust_req = Arc::new(FakeWebRequest::new());
    ctx.webhooks
        .run_endpoint("pause_resume/check_continue_print_state", rust_req.clone() as WebRequest)
        .expect("rust check endpoint");
    check(
        "check_continue_print_state",
        Ok(py.eval("wr.response").expect("eval check response")),
        rust_req.sent_value(),
    );

    py.exec("wr = FakeWebRequest(); wh.endpoints['getBootLoaderVersion'](wr)")
        .expect("py bootloader endpoint");
    let rust_req = Arc::new(FakeWebRequest::new());
    ctx.webhooks
        .run_endpoint("getBootLoaderVersion", rust_req.clone() as WebRequest)
        .expect("rust bootloader endpoint");
    check(
        "bootloader_version",
        Ok(py.eval("wr.response").expect("eval bootloader response")),
        rust_req.sent_value(),
    );
}

#[test]
fn webhooks_bootloader_missing_version() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    let ctx = rust_ctx(false, None);
    py.exec("mcu._constants = {}").expect("py clear constants");
    ctx.printer.insert(
        "mcu",
        FakeObject::with_mcu(FakeMcu { constants: IndexMap::new() }),
    );
    fire_connect(&mut py, &ctx);
    py.exec("wr = FakeWebRequest(); wh.endpoints['getBootLoaderVersion'](wr)")
        .expect("py bootloader endpoint");
    let rust_req = Arc::new(FakeWebRequest::new());
    ctx.webhooks
        .run_endpoint("getBootLoaderVersion", rust_req.clone() as WebRequest)
        .expect("rust bootloader endpoint");
    check(
        "bootloader_version_missing",
        Ok(py.eval("wr.response").expect("eval bootloader response")),
        rust_req.sent_value(),
    );
}

#[test]
fn custom_recover_velocity_used_for_resume() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, Some("70.5"));
    let ctx = rust_ctx(false, Some("70.5"));
    fire_connect(&mut py, &ctx);
    check(
        "recover_velocity_custom",
        Ok(py.eval("obj.recover_velocity").expect("eval recover_velocity")),
        Value::float(ctx.pause.recover_velocity()),
    );
    py.exec("obj.cmd_PAUSE(make_gcmd(g, 'PAUSE', 'PAUSE', {}))").expect("py PAUSE");
    ctx.gcode
        .run_handler("PAUSE", &gcmd(&ctx, "PAUSE", "PAUSE", &[]))
        .expect("rust PAUSE");
    py.exec("obj.cmd_RESUME(make_gcmd(g, 'RESUME', 'RESUME', {}))").expect("py RESUME");
    ctx.gcode
        .run_handler("RESUME", &gcmd(&ctx, "RESUME", "RESUME", &[]))
        .expect("rust RESUME");
    snapshot_pair("after_resume_custom_velocity", &mut py, &ctx);
}

#[test]
fn constructor_missing_gcode_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let _ = py.exec(
        "g = FakeGCode(); wh = FakeWebHooks(); vsd = FakeVirtualSD(); p = FakePrinter({'webhooks': wh}); c = FakeConfig(p, {}); pause_resume.load_config(c)",
    );
    let expected = py.eval("pause_resume.load_config(c)");
    let printer = Arc::new(FakePrinter::default());
    printer.insert("webhooks", FakeObject::with_webhooks(FakeWebHooks::default()));
    let printer_arc: Printer = printer.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc,
        options: IndexMap::new(),
    });
    check_err("constructor_missing_gcode", expected, PauseResume::new(&config).map(|_| ()));
}

#[test]
fn module_factory_load_config_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, false, None);
    // The factory builds a second object, so give it fresh gcode/webhooks
    // fakes: the real `gcode.register_command` (and the webhooks endpoint
    // registry) raises on a duplicate, so reusing the first build's printer
    // would fail in both languages.
    let gcode2 = FakeGCode::default();
    let webhooks2 = FakeWebHooks::default();
    let printer2 = Arc::new(FakePrinter::default());
    printer2.insert("gcode", FakeObject::with_gcode(gcode2));
    printer2.insert(
        "webhooks",
        FakeObject::with_webhooks(webhooks2),
    );
    printer2.insert("mcu", FakeObject::with_mcu(FakeMcu::default()));
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config entry point");
    let printer_arc: Printer = printer2.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc,
        options: IndexMap::new(),
    });
    let built = load_config(&config).expect("factory load_config");
    let built = built.expect("factory returns object");
    let downcast = downcast_arc::<PauseResume>(&built).expect("downcasts to PauseResume");
    check(
        "factory_object_status",
        Ok(py.eval("obj.get_status(0.0)").expect("eval status")),
        status_value(&downcast),
    );
}
