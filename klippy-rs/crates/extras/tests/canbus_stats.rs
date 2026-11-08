//! Parity tests for `extras::canbus_stats` (port of `klippy/extras/canbus_stats.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterCANBusStats`
//! is built on both sides with minimal Python/Rust fakes for the printer,
//! config, MCU and query command, then fed identical `handle_connect`,
//! `query_event`, `handle_usb_canbus_state` and `handle_shutdown` calls.
//! Returned values, error class/message, `get_status` dicts, `stats`
//! strings, MCU lookup arguments, serial registrations, deprecation calls
//! and log lines are compared exactly (floats by bits via the recording
//! value format, except the `query_event` wake time, which is
//! `monotonic() + 1.` and is checked as the same formula on both sides).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::canbus_stats::PrinterCANBusStats;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueue,
    CommandQueryWrapper, CommandQueryWrapperApi, CommandWrapper, CommandWrapperApi,
    ConfigCallback, LookupQueryCommandOpts, McuApi, Pin, PinParams, QuerySendOpts,
    SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging as _logging
import extras.canbus_stats as canbus_stats

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

_sentinel = object()

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.monotonic_value = 1000.0
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append((cb, waketime))
        return ('timer', len(self.timers) - 1)
    def monotonic(self):
        return self.monotonic_value

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, reactor):
        self.reactor = reactor
        self.objects = {}
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakeQueryCmd:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []
    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        self.calls.append((data, minclock, reqclock, retry))
        item = self.responses.pop(0)
        if isinstance(item, Exception):
            raise item
        return item

class FakeCommand:
    pass

class FakeMCU:
    def __init__(self, name, has_cmd, usb_valid, query_responses):
        self._name = name
        self.has_cmd = has_cmd
        self.usb_valid = usb_valid
        self.query_cmd = FakeQueryCmd(query_responses)
        self.try_calls = []
        self.lookup_query_calls = []
        self.check_calls = []
        self.serial_calls = []
    def get_name(self):
        return self._name
    def try_lookup_command(self, msgformat):
        self.try_calls.append(msgformat)
        if self.has_cmd:
            return FakeCommand()
        return None
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        self.lookup_query_calls.append((msgformat, respformat, oid, cq, is_async))
        return self.query_cmd
    def check_valid_response(self, msgformat):
        self.check_calls.append(msgformat)
        return self.usb_valid
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_calls.append((cb, msg, oid))
        return ('serial', len(self.serial_calls) - 1)

class FakeConfigFile:
    def __init__(self):
        self.deprecate_calls = []
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        self.deprecate_calls.append((mcu.get_name(), feature, msg))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name):
        self._printer = printer
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Self {
        FakePrinter {
            reactor,
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
        }
    }
    fn add_obj(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn events(&self) -> Vec<String> {
        let mut ev: Vec<String> = Vec::new();
        for (e, _) in self.handlers.lock().iter() {
            if !ev.contains(e) {
                ev.push(e.clone());
            }
        }
        ev
    }
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
        self.add_obj(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
        if let Some(obj) = objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        match module {
            None => objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(m) => {
                let mut out = Vec::new();
                let prefix = format!("{m} ");
                for (k, v) in objects.iter() {
                    if k.as_str() == m || k.starts_with(&prefix) {
                        out.push((k.clone(), v.clone()));
                    }
                }
                out
            }
        }
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
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
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut handlers = self.handlers.lock();
        for (e, cb) in handlers.iter_mut() {
            if e.as_str() == event {
                out.push((*cb)(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

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
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
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
        panic!("getsection is not used by canbus_stats")
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

struct FakeQueryCmd {
    results: Mutex<VecDeque<KResult<PyValue>>>,
    calls: Mutex<Vec<(Vec<PyValue>, QuerySendOpts)>>,
}

impl FakeQueryCmd {
    fn new(results: Vec<KResult<PyValue>>) -> Self {
        FakeQueryCmd {
            results: Mutex::new(results.into()),
            calls: Mutex::new(Vec::new()),
        }
    }
    fn set_results(&self, results: Vec<KResult<PyValue>>) {
        *self.results.lock() = results.into();
    }
}

impl CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        self.calls.lock().push((data, opts));
        self.results.lock().pop_front().expect("queued query response")
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by canbus_stats")
    }
}

struct FakeCommand;

impl CommandWrapperApi for FakeCommand {
    fn send(&self, _data: Vec<PyValue>, _opts: klippy_api::mcu::SendOpts) -> KResult<()> {
        panic!("CommandWrapper::send is not used by canbus_stats")
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: klippy_api::mcu::SendOpts) -> KResult<()> {
        panic!("CommandWrapper::send_wait_ack is not used by canbus_stats")
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeAsync;
impl AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    name: String,
    has_cmd: bool,
    usb_valid: bool,
    query: Arc<FakeQueryCmd>,
    try_calls: Mutex<Vec<String>>,
    lookup_query_calls: Mutex<Vec<(String, LookupQueryCommandOpts)>>,
    check_calls: Mutex<Vec<String>>,
    serial_calls: Mutex<Vec<(String, Option<i64>, SerialResponseCallback)>>,
}

impl FakeMcu {
    fn new(name: &str, has_cmd: bool, usb_valid: bool, query: Arc<FakeQueryCmd>) -> Self {
        FakeMcu {
            name: name.to_string(),
            has_cmd,
            usb_valid,
            query,
            try_calls: Mutex::new(Vec::new()),
            lookup_query_calls: Mutex::new(Vec::new()),
            check_calls: Mutex::new(Vec::new()),
            serial_calls: Mutex::new(Vec::new()),
        }
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        panic!("McuApi::get_printer is not used by canbus_stats")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("McuApi::setup_pin is not used by canbus_stats")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("McuApi::create_oid is not used by canbus_stats")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("McuApi::register_config_callback is not used by canbus_stats")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("McuApi::add_config_cmd is not used by canbus_stats")
    }
    fn request_move_queue_slot(&self) {
        panic!("McuApi::request_move_queue_slot is not used by canbus_stats")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("McuApi::get_query_slot is not used by canbus_stats")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("McuApi::seconds_to_clock is not used by canbus_stats")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("McuApi::min_schedule_time is not used by canbus_stats")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("McuApi::max_nominal_duration is not used by canbus_stats")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("McuApi::lookup_command is not used by canbus_stats")
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.lookup_query_calls.lock().push((respformat.to_string(), opts));
        assert_eq!(msgformat, "get_canbus_status");
        let q: CommandQueryWrapper = self.query.clone();
        Ok(q)
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.try_calls.lock().push(msgformat.to_string());
        if self.has_cmd {
            let cmd: CommandWrapper = Arc::new(FakeCommand);
            Some(cmd)
        } else {
            None
        }
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("McuApi::alloc_command_queue is not used by canbus_stats")
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.serial_calls.lock().push((msg.to_string(), oid, cb));
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, msgformat: &str) -> bool {
        self.check_calls.lock().push(msgformat.to_string());
        self.usb_valid
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("McuApi::get_enumerations is not used by canbus_stats")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("McuApi::get_constants is not used by canbus_stats")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("McuApi::get_constant_float is not used by canbus_stats")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("McuApi::print_time_to_clock is not used by canbus_stats")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("McuApi::clock_to_print_time is not used by canbus_stats")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("McuApi::estimated_print_time is not used by canbus_stats")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("McuApi::clock32_to_clock64 is not used by canbus_stats")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("McuApi::calibrate_clock is not used by canbus_stats")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("McuApi::get_status is not used by canbus_stats")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("McuApi::stats is not used by canbus_stats")
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("McuApi::create_trigger_dispatch is not used by canbus_stats")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeConfigFile {
    deprecate_calls: Mutex<Vec<(String, String)>>,
}

impl FakeConfigFile {
    fn new() -> Self {
        FakeConfigFile {
            deprecate_calls: Mutex::new(Vec::new()),
        }
    }
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("PrinterConfigApi::get_printer is not used by canbus_stats")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("PrinterConfigApi::read_config is not used by canbus_stats")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("PrinterConfigApi::read_main_config is not used by canbus_stats")
    }
    fn log_config(&self, _config: &Config) {
        panic!("PrinterConfigApi::log_config is not used by canbus_stats")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("PrinterConfigApi::check_unused_options is not used by canbus_stats")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("PrinterConfigApi::runtime_warning is not used by canbus_stats")
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("PrinterConfigApi::deprecate is not used by canbus_stats")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {
        panic!("PrinterConfigApi::deprecate_gcode is not used by canbus_stats")
    }
    fn deprecate_mcu_code(&self, mcu: &klippy_api::mcu::Mcu, feature: &str, _msg: Option<String>) {
        self.deprecate_calls.lock().push((mcu.get_name(), feature.to_string()));
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("PrinterConfigApi::get_status is not used by canbus_stats")
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) {
        panic!("PrinterConfigApi::set is not used by canbus_stats")
    }
    fn remove_section(&self, _section: &str) {
        panic!("PrinterConfigApi::remove_section is not used by canbus_stats")
    }
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("canbus_stats fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping canbus_stats oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

#[allow(clippy::type_complexity)]
fn log_capture() -> Arc<Mutex<Vec<(String, String)>>> {
    use std::sync::LazyLock;
    static CAPTURE: LazyLock<Arc<Mutex<Vec<(String, String)>>>> =
        LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
    let captured = CAPTURE.clone();
    let sink_capture = captured.clone();
    log::set_sink(Box::new(move |rec: &log::Record<'_>| {
        sink_capture.lock().push((rec.level.name().to_string(), rec.message.to_string()));
    }));
    captured
}

/// Serializes the log-capturing tests: they share the process-wide log sink
/// and capture vector, so clearing/reading must be atomic w.r.t. each other.
fn log_lock() -> std::sync::MutexGuard<'static, ()> {
    use std::sync::{LazyLock, Mutex as StdMutex};
    static LOCK: LazyLock<StdMutex<()>> = LazyLock::new(|| StdMutex::new(()));
    LOCK.lock().expect("log test lock")
}

fn vt(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.inner().to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(vt).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(vt).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (vt(k), vt(v))).collect()),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("0x{h:x}"),
        },
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "canbus_stats".to_string(),
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

fn check_ok(label: &str, expected: pyoracle::OracleResult, actual: &KResult<()>) {
    match (&expected, actual) {
        (Ok(_), Ok(())) => {}
        (Err(pe), Err(re)) => {
            assert_eq!(re.class_name(), pe.class_name.as_str(), "{label}: exception class");
            assert_eq!(re.message(), pe.message.as_str(), "{label}: exception message");
        }
        (Ok(v), Err(re)) => panic!("{label}: oracle returned {v:?} but rust raised {}: {}", re.class_name(), re.message()),
        (Err(pe), Ok(())) => panic!("{label}: oracle raised {}: {} but rust succeeded", pe.class_name, pe.message),
    }
}

fn check_err_f64(label: &str, expected: pyoracle::OracleResult, actual: &KResult<f64>) {
    match (&expected, actual) {
        (Err(pe), Err(re)) => {
            assert_eq!(re.class_name(), pe.class_name.as_str(), "{label}: exception class");
            assert_eq!(re.message(), pe.message.as_str(), "{label}: exception message");
        }
        (Ok(v), Ok(_)) => panic!("{label}: oracle returned {v:?} but rust must fail"),
        (Ok(v), Err(_)) => panic!("{label}: oracle returned {v:?} but rust raised"),
        (Err(pe), Ok(_)) => panic!("{label}: oracle raised {}: {} but rust succeeded", pe.class_name, pe.message),
    }
}

fn stats_value(stats: (bool, String)) -> Value {
    Value::Tuple(vec![Value::Bool(stats.0), Value::str(&stats.1)])
}

fn py_str(s: &str) -> String {
    format!("'{s}'")
}

fn query_params(rx: i64, tx: i64, retries: i64, state: i64) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("rx_error".to_string()), PyValue::Int(PyInt::from_i64(rx)));
    d.insert(PyValue::Str("tx_error".to_string()), PyValue::Int(PyInt::from_i64(tx)));
    d.insert(PyValue::Str("tx_retries".to_string()), PyValue::Int(PyInt::from_i64(retries)));
    d.insert(PyValue::Str("canbus_bus_state".to_string()), PyValue::Int(PyInt::from_i64(state)));
    PyValue::Dict(d)
}

#[allow(dead_code)]
struct Pair {
    rust: PrinterCANBusStats,
    printer: Printer,
    fake_printer: Arc<FakePrinter>,
    fake_mcu: Arc<FakeMcu>,
    fake_query: Arc<FakeQueryCmd>,
    fake_cfg: Arc<FakeConfigFile>,
    config: Config,
    reactor: Reactor,
}

fn section_mcu_name(section: &str) -> Option<String> {
    section.split_whitespace().last().map(|s| s.to_string())
}

fn fresh_pair(py: &mut pyoracle::Oracle, section: &str, has_cmd: bool, usb_valid: bool) -> Pair {
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor.clone()));
    let printer: Printer = fake_printer.clone();
    let name = section_mcu_name(section).unwrap_or_else(|| "mcu".to_string());
    let mcu_name = if name == "mcu" { "mcu".to_string() } else { format!("mcu {name}") };
    let fake_query = Arc::new(FakeQueryCmd::new(Vec::new()));
    let fake_mcu = Arc::new(FakeMcu::new(&name, has_cmd, usb_valid, fake_query.clone()));
    let fake_cfg = Arc::new(FakeConfigFile::new());
    let mcu_obj: ObjectRef = fake_mcu.clone();
    let cfg_obj: ObjectRef = fake_cfg.clone();
    fake_printer.add_obj(&mcu_name, mcu_obj);
    fake_printer.add_obj("configfile", cfg_obj);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: section.to_string(),
    });
    let rust = PrinterCANBusStats::new(&config).expect("rust init");

    // Python side (bools must render as Python True/False).
    let py_section = py_str(section);
    let py_has_cmd = if has_cmd { "True" } else { "False" };
    let py_usb_valid = if usb_valid { "True" } else { "False" };
    py.exec(&format!(
        "_reactor = FakeReactor()\n\
         _printer = FakePrinter(_reactor)\n\
         _mcu = FakeMCU({py_name}, {py_has_cmd}, {py_usb_valid}, [])\n\
         _cfg = FakeConfigFile()\n\
         _printer.add_object({mcu_name_py}, _mcu)\n\
         _printer.add_object('configfile', _cfg)\n\
         _c = FakeConfig(_printer, {py_section})\n\
         obj = canbus_stats.load_config_prefix(_c)",
        py_name = py_str(&name),
        py_has_cmd = py_has_cmd,
        py_usb_valid = py_usb_valid,
        mcu_name_py = py_str(&mcu_name),
        py_section = py_section,
    ))
    .expect("py init");
    Pair {
        rust,
        printer,
        fake_printer,
        fake_mcu,
        fake_query,
        fake_cfg,
        config,
        reactor,
    }
}

fn py_set_responses(py: &mut pyoracle::Oracle, responses: &[&str]) {
    let list = responses.join(", ");
    py.exec(&format!("_mcu.query_cmd.responses = [{list}]")).expect("py set responses");
}

fn rust_set_responses(pair: &Pair, responses: Vec<KResult<PyValue>>) {
    pair.fake_query.set_results(responses);
}

fn check_status(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("obj.get_status(0.)");
    let actual = vt(&pair.rust.status_value());
    check(label, expected, actual);
}

fn check_stats(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("obj.stats(0.)");
    let actual = stats_value(PrinterObject::stats(&pair.rust, 0.).expect("stats is Some"));
    check(label, expected, actual);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_name_and_status() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats mcu1", true, true);
    let expected = py.eval("obj.name");
    check("init name", expected, Value::str(pair.rust.get_name()));
    check_status("init status", &mut py, &pair);
    check_stats("init stats", &mut py, &pair);
    // Both sides register connect then shutdown handlers, in that order.
    let expected = py.eval("list(_printer.handlers.keys())");
    let mut rust_events = pair.fake_printer.events();
    rust_events.sort();
    let py_events_expected = match py.eval("sorted(list(_printer.handlers.keys()))") {
        Ok(Value::List(v)) => v,
        other => panic!("unexpected handlers: {other:?}"),
    };
    let rust_events_v: Vec<Value> = rust_events.iter().map(|e| Value::str(e)).collect();
    // Reuse check() with the sorted expected value.
    let mut sorted = py_events_expected.clone();
    let _ = &mut sorted;
    check("init handlers", Ok(Value::List(sorted)), Value::List(rust_events_v));
    let _ = expected;
}

#[test]
fn name_extraction_variants() {
    let Some(mut py) = oracle() else { return };
    for section in ["canbus_stats mcu", "canbus_stats   mcu2", "  canbus_stats mcu3  ", "canbus_stats mcu"] {
        let pair = fresh_pair(&mut py, section, true, false);
        let expected = py.eval("obj.name");
        check(
            &format!("name extraction {section:?}"),
            expected,
            Value::str(pair.rust.get_name()),
        );
    }
}

#[test]
fn empty_name_raises_index_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("_reactor = FakeReactor()\n_printer = FakePrinter(_reactor)")
        .expect("py empty-name fixtures");
    for section in ["", "   "] {
        // Python side: load_config_prefix raises IndexError.
        let expected = py.eval(&format!(
            "canbus_stats.load_config_prefix(FakeConfig(_printer, {}))",
            py_str(section)
        ));
        let expected_exc = match expected {
            Err(e) => e,
            Ok(v) => panic!("rust must fail for {section:?} but oracle returned {v:?}"),
        };
        assert_eq!(expected_exc.class_name.as_str(), "IndexError", "empty name class {section:?}");
        assert_eq!(expected_exc.message.as_str(), "list index out of range", "empty name message {section:?}");

        // Rust side.
        let reactor = Reactor::new(false);
        let fake_printer = Arc::new(FakePrinter::new(reactor));
        let printer: Printer = fake_printer.clone();
        let config: Config = Arc::new(FakeConfig {
            printer,
            name: section.to_string(),
        });
        let res = PrinterCANBusStats::new(&config);
        match res {
            Err(e) => {
                assert_eq!(e.class_name(), "IndexError");
                assert_eq!(e.message(), "list index out of range");
            }
            Ok(_) => panic!("rust new must fail for {section:?}"),
        }
    }
}

#[test]
fn connect_deprecate_path() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats dep1", false, true);
    // Rust connect.
    let rust_res = pair.rust.handle_connect();
    let rust_status = pair.rust.status_value();
    // Python connect.
    let py_res = py.eval("obj.handle_connect()");
    check_ok("deprecate connect", py_res, &rust_res);
    // Deprecation recorded once with the MCU name and feature, msg None.
    let expected = py.eval("[c for c in _cfg.deprecate_calls]");
    let rust_calls: Vec<Value> = pair
        .fake_cfg
        .deprecate_calls
        .lock()
        .iter()
        .map(|(m, f)| Value::Tuple(vec![Value::str(m), Value::str(f), Value::Null]))
        .collect();
    check("deprecate calls", expected, Value::List(rust_calls));
    // No serial registration and no query lookup; status unchanged.
    let expected = py.eval("len(_mcu.serial_calls)");
    check("deprecate no serial", expected, Value::int(pair.fake_mcu.serial_calls.lock().len() as i64));
    let expected = py.eval("len(_mcu.lookup_query_calls)");
    check("deprecate no lookup_query", expected, Value::int(pair.fake_mcu.lookup_query_calls.lock().len() as i64));
    check_status("deprecate status", &mut py, &pair);
    check_stats("deprecate stats", &mut py, &pair);
    let _ = rust_status;
}

#[test]
fn connect_success_registers_serial_and_query() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats ok1", true, true);
    let rust_res = pair.rust.handle_connect();
    let py_res = py.eval("obj.handle_connect()");
    check_ok("connect success", py_res, &rust_res);
    // try_lookup_command argument.
    let expected = py.eval("list(_mcu.try_calls)");
    let rust_try: Vec<Value> = pair.fake_mcu.try_calls.lock().iter().map(|s| Value::str(s)).collect();
    check("try_lookup args", expected, Value::List(rust_try));
    // lookup_query_command arguments: (msg, resp, oid=None, cq=None, is_async=False).
    let expected = py.eval("list(_mcu.lookup_query_calls)");
    let rust_lookup: Vec<Value> = pair
        .fake_mcu
        .lookup_query_calls
        .lock()
        .iter()
        .map(|(resp, opts)| {
            Value::Tuple(vec![
                Value::str("get_canbus_status"),
                Value::str(resp),
                Value::Null,
                Value::Null,
                Value::Bool(opts.is_async),
            ])
        })
        .collect();
    check("lookup_query args", expected, Value::List(rust_lookup));
    // check_valid_response argument.
    let expected = py.eval("list(_mcu.check_calls)");
    let rust_check: Vec<Value> = pair.fake_mcu.check_calls.lock().iter().map(|s| Value::str(s)).collect();
    check("check_valid args", expected, Value::List(rust_check));
    // Serial registration: message and oid None.
    let expected = py.eval("[(m, o) for (_, m, o) in _mcu.serial_calls]");
    let rust_serial: Vec<Value> = pair
        .fake_mcu
        .serial_calls
        .lock()
        .iter()
        .map(|(m, o, _)| Value::Tuple(vec![Value::str(m), match o { Some(v) => Value::int(*v), None => Value::Null }]))
        .collect();
    check("serial registration", expected, Value::List(rust_serial));
    // No deprecations.
    assert!(pair.fake_cfg.deprecate_calls.lock().is_empty());
    check_status("connect status untouched", &mut py, &pair);
}

#[test]
fn connect_without_usb_registers_no_serial() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats nousb", true, false);
    let rust_res = pair.rust.handle_connect();
    let py_res = py.eval("obj.handle_connect()");
    check_ok("connect no usb", py_res, &rust_res);
    let expected = py.eval("len(_mcu.serial_calls)");
    check("no usb serial", expected, Value::int(pair.fake_mcu.serial_calls.lock().len() as i64));
    // Query command still stored: query_event works.
    rust_set_responses(&pair, vec![Ok(query_params(1, 2, 3, 0))]);
    py_set_responses(&mut py, &["{'rx_error': 1, 'tx_error': 2, 'tx_retries': 3, 'canbus_bus_state': 0}"]);
    let before = pair.reactor.monotonic();
    let rust_ret = pair.rust.query_event(0.).expect("rust query");
    let after = pair.reactor.monotonic();
    assert!(rust_ret >= before + 1. && rust_ret <= after + 1., "wake is monotonic+1");
    let py_ret = match py.eval("obj.query_event(0.)") {
        Ok(Value::Float(f)) => f.to_f64().expect("float hex"),
        other => panic!("unexpected py query return: {other:?}"),
    };
    assert_eq!(py_ret, 1001.0, "py wake is 1000.0+1");
    check_status("no usb status after query", &mut py, &pair);
    check_stats("no usb stats after query", &mut py, &pair);
}

#[test]
fn connect_missing_mcu_raises() {
    let Some(mut py) = oracle() else { return };
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor.clone()));
    let printer: Printer = fake_printer.clone();
    // No MCU object inserted.
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: "canbus_stats ghost".to_string(),
    });
    let rust = PrinterCANBusStats::new(&config).expect("rust init");
    let rust_res = rust.handle_connect();
    // Python: fresh printer without mcu.
    py.exec("_reactor = FakeReactor()\n_printer = FakePrinter(_reactor)\n_c = FakeConfig(_printer, 'canbus_stats ghost')\nobj = canbus_stats.load_config_prefix(_c)")
        .expect("py init");
    let py_res = py.eval("obj.handle_connect()");
    check_ok("missing mcu connect", py_res, &rust_res);
}

#[test]
fn connect_missing_configfile_in_deprecate_path() {
    let Some(mut py) = oracle() else { return };
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor));
    let printer: Printer = fake_printer.clone();
    let fake_query = Arc::new(FakeQueryCmd::new(Vec::new()));
    let fake_mcu = Arc::new(FakeMcu::new("ghost2", false, false, fake_query));
    let mcu_obj: ObjectRef = fake_mcu.clone();
    fake_printer.add_obj("mcu ghost2", mcu_obj);
    // No configfile object.
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: "canbus_stats ghost2".to_string(),
    });
    let rust = PrinterCANBusStats::new(&config).expect("rust init");
    let rust_res = rust.handle_connect();
    py.exec("_reactor = FakeReactor()\n_printer = FakePrinter(_reactor)\n_mcu = FakeMCU('ghost2', False, False, [])\n_printer.add_object('mcu ghost2', _mcu)\n_c = FakeConfig(_printer, 'canbus_stats ghost2')\nobj = canbus_stats.load_config_prefix(_c)")
        .expect("py init");
    let py_res = py.eval("obj.handle_connect()");
    check_ok("missing configfile connect", py_res, &rust_res);
}

#[test]
fn query_unwrap_sequence() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats seq1", true, false);
    pair.rust.handle_connect().expect("rust connect");
    py.eval("obj.handle_connect()").expect("py connect");
    // Counter sequence incl. wraparound at 2**32 and zero deltas.
    let seq: Vec<(i64, i64, i64, i64)> = vec![
        (10, 20, 30, 0),
        (15, 25, 35, 1),
        (15, 25, 35, 2),
        (5, 10, 20, 0),
        (0x1_0000_0005, 0x1_0000_0010, 0x1_0000_0020, 3),
        (0x1_0000_0005, 0x1_0000_0010, 0x1_0000_0020, 3),
    ];
    for (i, (rx, tx, rt, st)) in seq.iter().enumerate() {
        rust_set_responses(&pair, vec![Ok(query_params(*rx, *tx, *rt, *st))]);
        py_set_responses(&mut py, &[&format!("{{'rx_error': {rx}, 'tx_error': {tx}, 'tx_retries': {rt}, 'canbus_bus_state': {st}}}")]);
        let before = pair.reactor.monotonic();
        let rust_ret = pair.rust.query_event(0.).expect("rust query");
        let after = pair.reactor.monotonic();
        assert!(rust_ret >= before + 1. && rust_ret <= after + 1., "query {i} wake");
        let py_ret = match py.eval("obj.query_event(0.)") {
            Ok(Value::Float(f)) => f.to_f64().expect("float hex"),
            other => panic!("unexpected py query return: {other:?}"),
        };
        assert_eq!(py_ret, 1001.0, "query {i} py wake");
        check_status(&format!("query {i} status"), &mut py, &pair);
        check_stats(&format!("query {i} stats"), &mut py, &pair);
        // Send was called with no data and default opts.
        let calls = pair.fake_query.calls.lock();
        let (data, opts) = &calls[calls.len() - 1];
        assert!(data.is_empty(), "query {i} send data empty");
        assert_eq!(*opts, QuerySendOpts::default(), "query {i} send opts");
    }
}

#[test]
fn query_before_connect_attribute_error() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats nodrain", true, false);
    let rust_res = pair.rust.query_event(0.);
    let py_res = py.eval("obj.query_event(0.)");
    check_err_f64("query before connect", py_res, &rust_res);
    check_status("before connect status", &mut py, &pair);
}

#[test]
fn query_send_error_propagates_and_keeps_status() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats err1", true, false);
    pair.rust.handle_connect().expect("rust connect");
    py.eval("obj.handle_connect()").expect("py connect");
    // First query succeeds.
    rust_set_responses(&pair, vec![Ok(query_params(7, 8, 9, 1))]);
    py_set_responses(&mut py, &["{'rx_error': 7, 'tx_error': 8, 'tx_retries': 9, 'canbus_bus_state': 1}"]);
    pair.rust.query_event(0.).expect("rust first query");
    py.eval("obj.query_event(0.)").expect("py first query");
    // Second query: send raises on both sides with the same class/message.
    // The Rust side raises `mcu.error` (class "error"); the Python side
    // uses a same-named exception class so class/message match exactly.
    rust_set_responses(&pair, vec![Err(KlippyError::Mcu("Timeout on wait for 'get_canbus_status' response".to_string()))]);
    py.exec("class error(Exception): pass\n_mcu.query_cmd.responses = [error('Timeout on wait for \\'get_canbus_status\\' response')]")
        .expect("py err response");
    let rust_res = pair.rust.query_event(0.);
    let py_res = py.eval("obj.query_event(0.)");
    check_err_f64("send error", py_res, &rust_res);
    check_status("status kept after send error", &mut py, &pair);
}

#[test]
fn query_missing_key_key_error() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats miss1", true, false);
    pair.rust.handle_connect().expect("rust connect");
    py.eval("obj.handle_connect()").expect("py connect");
    let mut incomplete = IndexMap::new();
    incomplete.insert(PyValue::Str("tx_error".to_string()), PyValue::Int(PyInt::from_i64(1)));
    incomplete.insert(PyValue::Str("tx_retries".to_string()), PyValue::Int(PyInt::from_i64(1)));
    incomplete.insert(PyValue::Str("canbus_bus_state".to_string()), PyValue::Int(PyInt::from_i64(0)));
    rust_set_responses(&pair, vec![Ok(PyValue::Dict(incomplete))]);
    py_set_responses(&mut py, &["{'tx_error': 1, 'tx_retries': 1, 'canbus_bus_state': 0}"]);
    let rust_res = pair.rust.query_event(0.);
    let py_res = py.eval("obj.query_event(0.)");
    check_err_f64("missing rx_error", py_res, &rust_res);
    check_status("status kept after key error", &mut py, &pair);
}

#[test]
fn usb_state_logs_and_errors() {
    let Some(mut py) = oracle() else { return };
    let _log_lock = log_lock();
    let captured = log_capture();
    let pair = fresh_pair(&mut py, "canbus_stats usb1", true, true);
    // discard truthy.
    let params_on = {
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("discard".to_string()), PyValue::Int(PyInt::from_i64(1)));
        PyValue::Dict(d)
    };
    let params_off = {
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("discard".to_string()), PyValue::Int(PyInt::from_i64(0)));
        PyValue::Dict(d)
    };
    captured.lock().clear();
    py.exec("log_lines.clear()").expect("clear py logs");
    let rust_on = pair.rust.handle_usb_canbus_state(&params_on);
    let py_on = py.eval("obj.handle_usb_canbus_state({'discard': 1})");
    check_ok("usb discard on", py_on, &rust_on);
    let rust_off = pair.rust.handle_usb_canbus_state(&params_off);
    let py_off = py.eval("obj.handle_usb_canbus_state({'discard': 0})");
    check_ok("usb discard off", py_off, &rust_off);
    // Logs: python bare messages; rust (level, message) pairs filtered to this bridge.
    let expected = py.eval("list(log_lines)");
    let expected_lines = match expected {
        Ok(Value::List(v)) => v,
        other => panic!("unexpected py logs: {other:?}"),
    };
    let rust_lines: Vec<Value> = captured
        .lock()
        .iter()
        .filter(|(_, m)| m.contains("usb1"))
        .map(|(_, m)| Value::str(m))
        .collect();
    check("usb logs", Ok(Value::List(expected_lines)), Value::List(rust_lines));
    // Missing key raises the same KeyError on both sides.
    let rust_missing = pair.rust.handle_usb_canbus_state(&PyValue::Dict(IndexMap::new()));
    let py_missing = py.eval("obj.handle_usb_canbus_state({})");
    check_ok("usb missing key", py_missing, &rust_missing);
}

#[test]
fn usb_state_via_serial_callback() {
    let Some(mut py) = oracle() else { return };
    let _log_lock = log_lock();
    let captured = log_capture();
    let pair = fresh_pair(&mut py, "canbus_stats usb2", true, true);
    pair.rust.handle_connect().expect("rust connect");
    py.eval("obj.handle_connect()").expect("py connect");
    // Fire the recorded serial callback on the Rust side with discard=1.
    let cb = {
        let calls = pair.fake_mcu.serial_calls.lock();
        assert_eq!(calls.len(), 1, "one serial registration");
        calls[0].2.clone()
    };
    captured.lock().clear();
    py.exec("log_lines.clear()").expect("clear py logs");
    cb(query_params_discarded(1));
    py.eval("obj.handle_usb_canbus_state({'discard': 1})").expect("py usb");
    let expected = py.eval("list(log_lines)");
    let expected_lines = match expected {
        Ok(Value::List(v)) => v,
        other => panic!("unexpected py logs: {other:?}"),
    };
    let rust_lines: Vec<Value> = captured
        .lock()
        .iter()
        .filter(|(_, m)| m.contains("usb2"))
        .map(|(_, m)| Value::str(m))
        .collect();
    check("serial callback log", Ok(Value::List(expected_lines)), Value::List(rust_lines));
}

fn query_params_discarded(discard: i64) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("discard".to_string()), PyValue::Int(PyInt::from_i64(discard)));
    PyValue::Dict(d)
}

#[test]
fn shutdown_marks_unknown() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats shut1", true, false);
    // Shutdown before any query: status unchanged (all None).
    pair.rust.handle_shutdown();
    py.eval("obj.handle_shutdown()").expect("py shutdown");
    check_status("shutdown before query", &mut py, &pair);
    check_stats("shutdown before query stats", &mut py, &pair);
    // Query then shutdown: bus_state becomes 'unknown'.
    pair.rust.handle_connect().expect("rust connect");
    py.eval("obj.handle_connect()").expect("py connect");
    rust_set_responses(&pair, vec![Ok(query_params(3, 4, 5, 1))]);
    py_set_responses(&mut py, &["{'rx_error': 3, 'tx_error': 4, 'tx_retries': 5, 'canbus_bus_state': 1}"]);
    pair.rust.query_event(0.).expect("rust query");
    py.eval("obj.query_event(0.)").expect("py query");
    pair.rust.handle_shutdown();
    py.eval("obj.handle_shutdown()").expect("py shutdown 2");
    check_status("shutdown after query", &mut py, &pair);
    check_stats("shutdown after query stats", &mut py, &pair);
    // Second shutdown is a no-op for the string state.
    pair.rust.handle_shutdown();
    py.eval("obj.handle_shutdown()").expect("py shutdown 3");
    check_status("double shutdown", &mut py, &pair);
}

#[test]
fn factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::canbus_stats::module_factory();
    assert!(factory.load_config.is_none(), "canbus_stats has no load_config");
    let load_prefix = factory.load_config_prefix.expect("canbus_stats exposes load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor));
    let printer: Printer = fake_printer.clone();
    let fake_query = Arc::new(FakeQueryCmd::new(Vec::new()));
    let fake_mcu = Arc::new(FakeMcu::new("factory1", true, false, fake_query));
    let mcu_obj: ObjectRef = fake_mcu.clone();
    fake_printer.add_obj("mcu factory1", mcu_obj);
    let fake_cfg = Arc::new(FakeConfigFile::new());
    let cfg_obj: ObjectRef = fake_cfg.clone();
    fake_printer.add_obj("configfile", cfg_obj);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: "canbus_stats factory1".to_string(),
    });
    let obj = load_prefix(&config).expect("factory load").expect("factory returns object");
    let rust: Arc<PrinterCANBusStats> = klippy_api::downcast_arc(&obj).expect("downcast");
    py.exec("_reactor = FakeReactor()\n_printer = FakePrinter(_reactor)\n_mcu = FakeMCU('factory1', True, False, [])\n_printer.add_object('mcu factory1', _mcu)\n_cfg = FakeConfigFile()\n_printer.add_object('configfile', _cfg)\n_c = FakeConfig(_printer, 'canbus_stats factory1')\nobj = canbus_stats.load_config_prefix(_c)")
        .expect("py factory");
    let expected = py.eval("obj.name");
    check("factory name", expected, Value::str(rust.get_name()));
    let expected = py.eval("obj.get_status(0.)");
    check("factory status", expected, vt(&rust.status_value()));
    let expected = py.eval("obj.stats(0.)");
    check("factory stats", expected, stats_value(PrinterObject::stats(&*rust, 0.).expect("stats is Some")));
}

#[test]
fn event_dispatch_via_printer() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, "canbus_stats evt1", true, false);
    // Drive klippy:connect through send_event on both sides.
    let rust_ev = pair.printer.send_event("klippy:connect", &[]);
    let py_ev = py.eval("_printer.send_event('klippy:connect')");
    match (py_ev, rust_ev) {
        (Ok(Value::List(v)), Ok(rv)) => {
            assert_eq!(v.len(), 1, "py connect handlers");
            assert_eq!(rv.len(), 1, "rust connect handlers");
            // Handler returns None on both sides.
            check("connect handler ret", Ok(v[0].clone()), vt(&rv[0]));
        }
        other => panic!("unexpected connect dispatch: {other:?}"),
    }
    // Queue a query response and verify the timer would run via direct query.
    rust_set_responses(&pair, vec![Ok(query_params(2, 3, 4, 0))]);
    py_set_responses(&mut py, &["{'rx_error': 2, 'tx_error': 3, 'tx_retries': 4, 'canbus_bus_state': 0}"]);
    pair.rust.query_event(0.).expect("rust query");
    py.eval("obj.query_event(0.)").expect("py query");
    check_status("dispatch status", &mut py, &pair);
    // Drive klippy:shutdown through send_event.
    let rust_shut = pair.printer.send_event("klippy:shutdown", &[]);
    let py_shut = py.eval("_printer.send_event('klippy:shutdown')");
    match (py_shut, rust_shut) {
        (Ok(Value::List(v)), Ok(rv)) => {
            assert_eq!(v.len(), 1);
            assert_eq!(rv.len(), 1);
        }
        other => panic!("unexpected shutdown dispatch: {other:?}"),
    }
    check_status("dispatch shutdown status", &mut py, &pair);
    check_stats("dispatch shutdown stats", &mut py, &pair);
}
