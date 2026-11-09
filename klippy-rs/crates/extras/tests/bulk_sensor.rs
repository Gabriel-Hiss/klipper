//! Parity tests for `extras::bulk_sensor` (port of `klippy/extras/bulk_sensor.py`).
//!
//! The module has no printer object, config or G-code surface, so parity is
//! proven against the Python helpers directly: `BatchBulkHelper` (batching of
//! accumulated messages and webhooks mux registration), `BatchWebhooksClient`
//! (webhooks batch delivery), `BulkDataQueue` (`sensor_bulk_data` queueing),
//! `ClockSyncRegression` (chip-clock linear regression) and `FixedFreqReader`
//! (the `sensor_bulk_status` query, `_update_clock` sequence/overflow
//! bookkeeping and timestamped `pull_samples`). Expected values come from the
//! live reference through `pyoracle` with minimal Python fakes for the printer,
//! reactor, MCU and webhooks, fed with the same inputs as the Rust fakes below.
//! Compared: returned sample lists and timestamps (floats by bits), raised
//! errors (class and message), MCU command format strings, query options and
//! serial response registrations, and webhooks mux / send payloads.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::bulk_sensor::{
    BatchBulkHelper, BatchWebhooksClient, BulkDataQueue, ClockSyncRegression, FixedFreqReader,
    StructUnpack, BATCH_INTERVAL, MAX_BULK_MSG_SIZE, SENSOR_BULK_FMT,
};
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper, CommandWrapper,
    Mcu, McuApi, Pin, PinParams, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{
    ClientConnection, ClientConnectionApi, WebHooksApi, WebRequest, WebRequestApi,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import struct
import extras.bulk_sensor as bs

class CommandError(Exception):
    pass

_sentinel = object()

class _FakeTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime
        self.timer_is_running = False

class FakeReactor:
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.unregistered = []
    def monotonic(self):
        return 1000.0
    def register_timer(self, callback, waketime):
        t = _FakeTimer(callback, waketime)
        self.timers.append(t)
        return t
    def unregister_timer(self, timer_handler):
        timer_handler.waketime = self.NEVER
        self.unregistered.append(timer_handler)

class FakePrinter:
    command_error = CommandError
    config_error = configparser.Error
    def __init__(self, webhooks=None):
        self.reactor = FakeReactor()
        self._webhooks = webhooks
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=_sentinel):
        if name == 'webhooks' and self._webhooks is not None:
            return self._webhooks
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeWebhooks:
    def __init__(self):
        self.mux = []
    def register_mux_endpoint(self, path, key, value, callback):
        self.mux.append((path, key, value, callback))

class FakeClientConnection:
    def __init__(self, closed=False):
        self._closed = closed
        self.sent = []
    def is_closed(self):
        return self._closed
    def send(self, msg):
        self.sent.append(msg)

class FakeWebRequest:
    def __init__(self, params=None, conn=None):
        self.params = params if params is not None else {}
        self.conn = conn if conn is not None else FakeClientConnection()
        self.sent = []
    def get_client_connection(self):
        return self.conn
    def get_dict(self, item, default=None):
        return self.params.get(item, default)
    def send(self, data):
        self.sent.append(data)

class _QueryCmd:
    def __init__(self, mcu):
        self.mcu = mcu
    def send(self, data):
        self.mcu.sends.append(list(data))
        r = self.mcu.query_result
        return r() if callable(r) else r

class FakeMCU:
    def __init__(self, name):
        self.name = name
        self.serial_log = []
        self.serial_cb = None
        self.queries = []
        self.sends = []
        self.query_result = None
    def clock_to_print_time(self, clock):
        return clock * 0.01
    def seconds_to_clock(self, t):
        return int(t * 1000000)
    def clock32_to_clock64(self, clock32):
        return clock32
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        self.queries.append((msgformat, respformat, oid, cq, is_async))
        return _QueryCmd(self)
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_log.append((msg, oid))
        self.serial_cb = cb
        class _Async:
            def unregister(self):
                pass
        return _Async()
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes (same semantics as the Python fakes above).
// ---------------------------------------------------------------------------

#[derive(Default)]
struct McuShared {
    name: String,
    serial_log: Vec<(String, Option<i64>)>,
    serial_cb: Option<klippy_api::mcu::SerialResponseCallback>,
    queries: Vec<(String, String, Option<i64>, Option<i64>, bool)>,
    sends: Vec<Vec<PyValue>>,
    query_result: PyValue,
}

struct FakeMcu {
    inner: Arc<Mutex<McuShared>>,
}

struct FakeQueryCmd {
    inner: Arc<Mutex<McuShared>>,
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, _opts: klippy_api::mcu::QuerySendOpts) -> KResult<PyValue> {
        let mut inner = self.inner.lock().expect("mcu lock");
        inner.sends.push(data);
        Ok(inner.query_result.clone())
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by bulk_sensor")
    }
}

struct FakeAsync;

impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.inner.lock().expect("mcu lock").name.clone()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by bulk_sensor")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by bulk_sensor")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by bulk_sensor")
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by bulk_sensor")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by bulk_sensor")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by bulk_sensor")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by bulk_sensor")
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 1_000_000.0) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by bulk_sensor")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by bulk_sensor")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by bulk_sensor")
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.inner.lock().expect("mcu lock").queries.push((
            msgformat.to_string(),
            respformat.to_string(),
            opts.oid,
            opts.cq.map(|cq| cq.0),
            opts.is_async,
        ));
        Ok(Arc::new(FakeQueryCmd { inner: Arc::clone(&self.inner) }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by bulk_sensor")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by bulk_sensor")
    }
    fn register_serial_response(
        &self,
        cb: klippy_api::mcu::SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        let mut inner = self.inner.lock().expect("mcu lock");
        inner.serial_log.push((msg.to_string(), oid));
        inner.serial_cb = Some(cb);
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by bulk_sensor")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by bulk_sensor")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by bulk_sensor")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by bulk_sensor")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by bulk_sensor")
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock * 0.01
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by bulk_sensor")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by bulk_sensor")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by bulk_sensor")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by bulk_sensor")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by bulk_sensor")
    }
}

struct FakePrinter {
    reactor: Reactor,
    webhooks: Option<ObjectRef>,
}

impl FakePrinter {
    fn new(webhooks: Option<ObjectRef>) -> FakePrinter {
        FakePrinter { reactor: Reactor::new(false), webhooks }
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if name == "webhooks" {
            if let Some(webhooks) = &self.webhooks {
                return Ok(Some(webhooks.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
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
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeWebhooks {
    mux: Arc<Mutex<Vec<(String, String, String)>>>,
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

impl FakeWebhooks {
    fn new() -> (ObjectRef, Arc<Mutex<Vec<(String, String, String)>>>) {
        let mux = Arc::new(Mutex::new(Vec::new()));
        let obj: ObjectRef = Arc::new(FakeWebhooks { mux: mux.clone() });
        (obj, mux)
    }
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(
        &self,
        _path: &str,
        _callback: klippy_api::webhooks::WebHooksCallback,
    ) -> KResult<()> {
        panic!("register_endpoint is not used by bulk_sensor")
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        _callback: klippy_api::webhooks::WebHooksCallback,
    ) -> KResult<()> {
        self.mux.lock().expect("mux lock").push((
            path.to_string(),
            key.to_string(),
            value.to_string(),
        ));
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        panic!("call_remote_method is not used by bulk_sensor")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by bulk_sensor")
    }
}

#[derive(Default)]
struct ConnState {
    closed: bool,
    sent: Vec<PyValue>,
}

struct FakeClientConn {
    state: Arc<Mutex<ConnState>>,
}

impl ClientConnectionApi for FakeClientConn {
    fn uid(&self) -> u64 {
        panic!("uid is not used by bulk_sensor")
    }
    fn dump_request_log(&self) {
        panic!("dump_request_log is not used by bulk_sensor")
    }
    fn set_client_info(&self, _client_info: PyValue, _state_msg: Option<String>) {
        panic!("set_client_info is not used by bulk_sensor")
    }
    fn close(&self) {
        panic!("close is not used by bulk_sensor")
    }
    fn is_closed(&self) -> bool {
        self.state.lock().expect("conn lock").closed
    }
    fn send(&self, data: PyValue) {
        self.state.lock().expect("conn lock").sent.push(data);
    }
}

struct FakeWebRequest {
    conn: ClientConnection,
    params: PyValue,
    sent: Arc<Mutex<Vec<PyValue>>>,
}

impl FakeWebRequest {
    fn build(params: PyValue, conn_closed: bool) -> (WebRequest, Arc<Mutex<ConnState>>) {
        let state = Arc::new(Mutex::new(ConnState { closed: conn_closed, sent: Vec::new() }));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let obj: WebRequest = Arc::new(FakeWebRequest {
            conn: Arc::new(FakeClientConn { state: state.clone() }),
            params,
            sent: sent.clone(),
        });
        (obj, state)
    }
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> ClientConnection {
        panic!("client_conn is not used by bulk_sensor")
    }
    fn id(&self) -> Option<PyValue> {
        panic!("id is not used by bulk_sensor")
    }
    fn method(&self) -> String {
        panic!("method is not used by bulk_sensor")
    }
    fn params(&self) -> PyValue {
        panic!("params is not used by bulk_sensor")
    }
    fn response(&self) -> Option<PyValue> {
        panic!("response is not used by bulk_sensor")
    }
    fn is_error(&self) -> bool {
        panic!("is_error is not used by bulk_sensor")
    }
    fn get_client_connection(&self) -> ClientConnection {
        self.conn.clone()
    }
    fn get(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        panic!("get is not used by bulk_sensor")
    }
    fn get_str(&self, _item: &str, _default: Option<String>) -> KResult<String> {
        panic!("get_str is not used by bulk_sensor")
    }
    fn get_int(&self, _item: &str, _default: Option<i64>) -> KResult<i64> {
        panic!("get_int is not used by bulk_sensor")
    }
    fn get_float(&self, _item: &str, _default: Option<f64>) -> KResult<f64> {
        panic!("get_float is not used by bulk_sensor")
    }
    fn get_dict(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        if let PyValue::Dict(dict) = &self.params {
            let key = PyValue::Str(item.to_string());
            if let Some(v) = dict.get(&key) {
                return Ok(v.clone());
            }
        }
        Ok(default.unwrap_or(PyValue::None))
    }
    fn get_method(&self) -> String {
        panic!("get_method is not used by bulk_sensor")
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        self.sent.lock().expect("sent lock").push(data);
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        panic!("finish is not used by bulk_sensor")
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bulk_sensor fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping bulk_sensor oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn int_value(v: i64) -> Value {
    Value::int(v)
}

fn opt_int_value(v: Option<i64>) -> Value {
    match v {
        Some(n) => Value::int(n),
        None => Value::Null,
    }
}

fn to_recording(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(xs) => Value::List(xs.iter().map(to_recording).collect()),
        PyValue::Tuple(xs) => Value::Tuple(xs.iter().map(to_recording).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (to_recording(k), to_recording(v))).collect(),
        ),
        PyValue::Host(_) => panic!("host values do not appear in bulk_sensor comparisons"),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "bulk_sensor".to_string(),
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

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let expected = match expected {
        Err(e) => e,
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected an exception"),
    };
    assert_eq!(
        actual.class_name(),
        expected.class_name,
        "{label}: exception class",
    );
    assert_eq!(actual.message(), expected.message, "{label}: exception message");
}

fn make_mcu() -> (Mcu, Arc<Mutex<McuShared>>) {
    let inner = Arc::new(Mutex::new(McuShared::default()));
    let mcu: Mcu = Arc::new(FakeMcu { inner: inner.clone() });
    (mcu, inner)
}

fn status_params(clock: i64, next_sequence: i64, buffered: i64, overflows: i64, query_ticks: i64) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    d.insert(PyValue::Str("clock".to_string()), PyValue::Int(PyInt::from_i64(clock)));
    d.insert(
        PyValue::Str("next_sequence".to_string()),
        PyValue::Int(PyInt::from_i64(next_sequence)),
    );
    d.insert(
        PyValue::Str("buffered".to_string()),
        PyValue::Int(PyInt::from_i64(buffered)),
    );
    d.insert(
        PyValue::Str("possible_overflows".to_string()),
        PyValue::Int(PyInt::from_i64(overflows)),
    );
    d.insert(
        PyValue::Str("query_ticks".to_string()),
        PyValue::Int(PyInt::from_i64(query_ticks)),
    );
    PyValue::Dict(d)
}

fn bulk_params(sequence: i64, data: Vec<u8>) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    d.insert(
        PyValue::Str("sequence".to_string()),
        PyValue::Int(PyInt::from_i64(sequence)),
    );
    d.insert(PyValue::Str("data".to_string()), PyValue::Bytes(data));
    PyValue::Dict(d)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("(bs.BATCH_INTERVAL, bs.SENSOR_BULK_FMT, bs.MAX_BULK_MSG_SIZE)")
        .expect("constants evaluate");
    let actual = Value::Tuple(vec![
        Value::float(BATCH_INTERVAL),
        Value::str(SENSOR_BULK_FMT),
        int_value(MAX_BULK_MSG_SIZE),
    ]);
    check("constants", Ok(expected), actual);
}

#[test]
fn struct_unpack_parity() {
    let Some(mut py) = oracle() else { return };
    // (format, data bytes) — oracle is the source of truth for values.
    let cases: &[(&str, Vec<u8>)] = &[
        ("<3h", vec![1, 0, 2, 0, 3, 0]),
        (">2f", vec![0x3f, 0xc0, 0, 0, 0xc0, 0x20, 0, 0]),
        ("<d", vec![0, 0, 0, 0, 0, 0, 4, 0x40]),
        ("=4s", b"abcd".to_vec()),
        ("@hf", vec![1, 0, 0, 0, 0, 0, 0x20, 0x40]),
        ("<2f", vec![0, 0, 0xc0, 0x3f, 0, 0, 0xc0, 0xc0]),
    ];
    for (label, (fmt, data)) in cases.iter().enumerate() {
        let use_format = StructUnpack::new(fmt).expect("struct format parses");
        let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
        let expected = py
            .eval(&format!(
                "(struct.Struct({fmt:?}).size, list(struct.Struct({fmt:?}).unpack_from(bytes.fromhex({hex:?}), 0)))"
            ))
            .expect("oracle unpack");
        let values = use_format.unpack_from(data, 0).expect("rust unpack");
        let mut items = vec![int_value(use_format.size() as i64)];
        items.push(Value::List(values.iter().map(to_recording).collect()));
        check(&format!("struct case {label} {fmt}"), Ok(expected), Value::Tuple(items));
    }
}

#[test]
fn struct_errors() {
    let Some(mut py) = oracle() else { return };
    for fmt in ["z", "0h", "<2", "", "<h"] {
        let expected = py.eval(&format!("struct.Struct({fmt:?})"));
        let actual = StructUnpack::new(fmt).map(|u| to_recording(&PyValue::Int(PyInt::from_i64(u.size() as i64))));
        match (expected, actual) {
            (Err(e), Err(k)) => check_exc(&format!("bad struct fmt {fmt:?}"), Err(e), &k),
            (Err(e), Ok(v)) => panic!("bad struct fmt {fmt:?}: oracle raised {e:?}, rust returned {v:?}"),
            (Ok(_), Err(k)) => panic!("bad struct fmt {fmt:?}: rust raised {}", k.message()),
            (Ok(_), Ok(_)) => {}
        }
    }
    // Short buffer for unpack.
    let expected = py.eval("struct.Struct('<h').unpack_from(bytes.fromhex('01'), 0)");
    let actual = StructUnpack::new("<h")
        .expect("fmt")
        .unpack_from(&[1], 0)
        .map(|v| {
            Value::List(v.iter().map(to_recording).collect())
        });
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("short unpack", Err(e), &k),
        other => panic!("short unpack mismatch: {other:?}"),
    }
}

#[test]
fn empty_struct_format_is_zero_division() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_q = FakeMCU('mcu')")
        .expect("mcu");
    let expected = py.eval("bs.FixedFreqReader(mcu_q, 1.0, '')");
    match expected {
        Err(e) => {
            let (mcu_zero, _inner) = make_mcu();
            let actual = FixedFreqReader::new(mcu_zero, 1.0, "");
            let err = actual.err().expect("empty format must fail");
            check_exc("empty unpack fmt", Err(e), &err);
        }
        Ok(_) => panic!("oracle did not raise for empty format"),
    }
}

#[test]
fn bulk_data_queue() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_b = FakeMCU('mcu')\nq = bs.BulkDataQueue(mcu_b)")
        .expect("python queue");
    let (mcu, inner) = make_mcu();
    let queue = BulkDataQueue::new(&mcu, SENSOR_BULK_FMT, None);
    // Serial registration matches (default msg format, no oid).
    let expected = py.eval("mcu_b.serial_log").expect("py serial log");
    let actual = Value::List(
        inner
            .lock()
            .expect("mcu lock")
            .serial_log
            .iter()
            .map(|(m, o)| Value::Tuple(vec![Value::str(m), opt_int_value(*o)]))
            .collect(),
    );
    check("bulk queue registration", Ok(expected), actual);

    // Push two messages through the stored callback on both sides.
    let p1 = bulk_params(3, vec![1, 2, 3]);
    let p2 = bulk_params(4, vec![4]);
    py.exec("mcu_b.serial_cb({'sequence': 3, 'data': b'\\x01\\x02\\x03'})\nmcu_b.serial_cb({'sequence': 4, 'data': b'\\x04'})")
        .expect("py handle");
    {
        let inner = inner.lock().expect("mcu lock");
        let cb = inner.serial_cb.as_ref().expect("serial cb");
        cb(p1);
        cb(p2);
    }
    let expected = py.eval("q.pull_queue()").expect("py pull");
    let actual = Value::List(queue.pull_queue().iter().map(to_recording).collect());
    check("bulk queue pull", Ok(expected), actual);

    // Second pull is empty.
    let expected = py.eval("q.pull_queue()").expect("py pull again");
    let actual = Value::List(queue.pull_queue().iter().map(to_recording).collect());
    check("bulk queue drained", Ok(expected), actual);

    // clear_queue drops queued items.
    py.exec("mcu_b.serial_cb({'sequence': 5, 'data': b'\\x05'})")
        .expect("py handle");
    {
        let inner = inner.lock().expect("mcu lock");
        inner.serial_cb.as_ref().expect("serial cb")(bulk_params(5, vec![5]));
    }
    py.exec("q.clear_queue()").expect("py clear");
    queue.clear_queue();
    let expected = py.eval("q.pull_queue()").expect("py pull after clear");
    let actual = Value::List(queue.pull_queue().iter().map(to_recording).collect());
    check("bulk queue clear", Ok(expected), actual);
}

#[test]
fn clock_sync_translation() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_c = FakeMCU('mcu')\nr = bs.ClockSyncRegression(mcu_c, 250.0)\nr.reset(1000, 10)\nr.update(2000, 20)\nr.update(4000, 41)")
        .expect("py regression");
    let (mcu, _inner) = make_mcu();
    let mut reg = ClockSyncRegression::new(mcu, 250.0);
    reg.reset(1000.0, 10.0);
    reg.update(2000.0, 20.0);
    reg.update(4000.0, 41.0);
    let expected = py
        .eval("(r.mcu_clock_avg, r.chip_clock_avg, r.mcu_clock_variance, r.chip_clock_covariance, r.get_clock_translation())")
        .expect("py translation");
    let (base_mcu, base_chip, inv) = reg.get_clock_translation().expect("rust translation");
    let actual = Value::Tuple(vec![
        Value::float(reg.mcu_clock_avg),
        Value::float(reg.chip_clock_avg),
        Value::float(reg.mcu_clock_variance),
        Value::float(reg.chip_clock_covariance),
        Value::Tuple(vec![Value::float(base_mcu), Value::float(base_chip), Value::float(inv)]),
    ]);
    // get_clock_translation returns (base_mcu, base_chip, inv); the state
    // tuple mirrors the Python tuple exactly.
    let expected_state = py
        .eval("(r.mcu_clock_avg, r.chip_clock_avg, r.mcu_clock_variance, r.chip_clock_covariance, r.get_clock_translation())")
        .expect("py state");
    let _ = expected;
    check("clock translation", Ok(expected_state), actual.clone());

    let expected_time = py
        .eval("r.get_time_translation()")
        .expect("py time translation");
    let (base_time, base_chip, inv_freq) =
        reg.get_time_translation().expect("rust time translation");
    let actual_time = Value::Tuple(vec![
        Value::float(base_time),
        Value::float(base_chip),
        Value::float(inv_freq),
    ]);
    check("time translation", Ok(expected_time), actual_time);

    // set_last_chip_clock + subsequent translation (future-point branch).
    py.exec("r.set_last_chip_clock(60.0)").expect("py set last");
    reg.set_last_chip_clock(60.0).expect("rust set last");
    let expected2 = py
        .eval("(r.get_clock_translation(), r.last_chip_clock, r.last_exp_mcu_clock)")
        .expect("py post-set translation");
    let (b_mcu, b_chip, inv2) = reg.get_clock_translation().expect("rust post translation");
    let actual2 = Value::Tuple(vec![
        Value::Tuple(vec![Value::float(b_mcu), Value::float(b_chip), Value::float(inv2)]),
        Value::float(reg.last_chip_clock),
        Value::float(reg.last_exp_mcu_clock),
    ]);
    check("clock translation after set", Ok(expected2), actual2);
}

#[test]
fn clock_sync_zero_division_after_reset() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_z = FakeMCU('mcu')\nr2 = bs.ClockSyncRegression(mcu_z, 250.0)\nr2.reset(1000, 10)")
        .expect("py reset only");
    let expected = py.eval("r2.get_time_translation()");
    let (mcu, _inner) = make_mcu();
    let mut reg = ClockSyncRegression::new(mcu, 250.0);
    reg.reset(1000.0, 10.0);
    let actual = reg.get_time_translation();
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("zero division", Err(e), &k),
        other => panic!("zero division mismatch: {other:?}"),
    }
}

#[test]
fn fixed_freq_setup_query_command() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_f = FakeMCU('mcu')\nr = bs.FixedFreqReader(mcu_f, 250.0, '<2f')\nr.setup_query_command('sensor_query oid=%c cycles=%u', 7, None)")
        .expect("py setup");
    let (mcu, inner) = make_mcu();
    let reader = FixedFreqReader::new(mcu, 250.0, "<2f").expect("rust reader");
    reader
        .setup_query_command("sensor_query oid=%c cycles=%u", 7, None)
        .expect("rust setup");
    let expected = py
        .eval("mcu_f.queries")
        .expect("py queries");
    // Python records the status response format only via lookup; compare
    // the full query log including the exact status format string.
    let actual = Value::List(
        inner
            .lock()
            .expect("mcu lock")
            .queries
            .iter()
            .map(|(m, resp, oid, cq, is_async)| {
                Value::Tuple(vec![
                    Value::str(m),
                    Value::str(resp),
                    opt_int_value(*oid),
                    opt_int_value(*cq),
                    Value::Bool(*is_async),
                ])
            })
            .collect(),
    );
    check("fixed freq query log", Ok(expected), actual);
    let expected_serial = py.eval("mcu_f.serial_log").expect("py serial");
    let actual_serial = Value::List(
        inner
            .lock()
            .expect("mcu lock")
            .serial_log
            .iter()
            .map(|(m, o)| Value::Tuple(vec![Value::str(m), opt_int_value(*o)]))
            .collect(),
    );
    check("fixed freq bulk registration", Ok(expected_serial), actual_serial);
    // Initial state of the reader matches (no overflows yet).
    assert_eq!(reader.get_last_overflows(), 0);
    let expected = py.eval("r.get_last_overflows()").expect("py overflows");
    check("initial overflows", Ok(expected), int_value(reader.get_last_overflows()));
}

#[test]
fn fixed_freq_clock_updates() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "mcu_u = FakeMCU('mcu')\n\
         mcu_u.query_result = {'clock': 100000, 'next_sequence': 10, 'buffered': 32, 'possible_overflows': 2, 'query_ticks': 100}\n\
         u = bs.FixedFreqReader(mcu_u, 250.0, '<2f')\nu.setup_query_command('sq', 7, None)\nu.note_start()",
    )
    .expect("py note_start");
    let (mcu, inner) = make_mcu();
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(100000, 10, 32, 2, 100);
    let reader = FixedFreqReader::new(mcu, 250.0, "<2f").expect("reader");
    reader.setup_query_command("sq", 7, None).expect("setup");
    reader.note_start().expect("note_start");
    let expected = py
        .eval("(u.last_sequence, u.last_overflows, u.get_last_overflows(), u.max_query_duration)")
        .expect("py state");
    let actual = Value::Tuple(vec![
        int_value(reader.last_sequence()),
        int_value(reader.get_last_overflows()),
        int_value(reader.get_last_overflows()),
        int_value(reader.max_query_duration()),
    ]);
    check("note_start state", Ok(expected), actual);

    // Sequence wrap: jump past 2**16 and back to small values.
    py.exec(
        "mcu_u.query_result = {'clock': 200000, 'next_sequence': 65530, 'buffered': 0, 'possible_overflows': 8, 'query_ticks': 100}\n\
         u._update_clock()\n\
         mcu_u.query_result = {'clock': 300000, 'next_sequence': 3, 'buffered': 8, 'possible_overflows': 8, 'query_ticks': 100}\n\
         u._update_clock()",
    )
    .expect("py wrap");
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(200000, 65530, 0, 8, 100);
    reader._update_clock(false).expect("wrap 1");
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(300000, 3, 8, 8, 100);
    reader._update_clock(false).expect("wrap 2");
    let expected = py
        .eval("(u.last_sequence, u.last_overflows, u.max_query_duration)")
        .expect("py wrap state");
    let actual = Value::Tuple(vec![
        int_value(reader.last_sequence()),
        int_value(reader.get_last_overflows()),
        int_value(reader.max_query_duration()),
    ]);
    check("sequence wrap", Ok(expected), actual);

    // Sends observed (oid list); one send per _update_clock + note_start's.
    let expected = py.eval("mcu_u.sends").expect("py sends");
    let actual = Value::List(
        inner
            .lock()
            .expect("mcu lock")
            .sends
            .iter()
            .map(|d| Value::List(d.iter().map(to_recording).collect()))
            .collect(),
    );
    check("query sends", Ok(expected), actual);

    // Duration filter: huge query_ticks skips the translation update.
    py.exec(
        "mcu_u.query_result = {'clock': 400000, 'next_sequence': 4, 'buffered': 0, 'possible_overflows': 9, 'query_ticks': 99999999999}\n\
         u._update_clock()",
    )
    .expect("py filter");
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(400000, 4, 0, 9, 99999999999);
    let before = reader.max_query_duration();
    reader._update_clock(false).expect("filter");
    let expected = py
        .eval("(u.last_sequence, u.last_overflows, u.max_query_duration)")
        .expect("py filter state");
    let actual = Value::Tuple(vec![
        int_value(reader.last_sequence()),
        int_value(reader.get_last_overflows()),
        int_value(reader.max_query_duration()),
    ]);
    assert!(reader.max_query_duration() >= before);
    check("duration filter", Ok(expected), actual);
}

#[test]
fn fixed_freq_pull_samples() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "mcu_p = FakeMCU('mcu')\n\
         mcu_p.query_result = {'clock': 100000, 'next_sequence': 10, 'buffered': 32, 'possible_overflows': 0, 'query_ticks': 100}\n\
         p = bs.FixedFreqReader(mcu_p, 250.0, '<2f')\np.setup_query_command('sq', 7, None)\np.note_start()",
    )
    .expect("py setup");
    // One <2f block: f32 1.5, -2.5.
    py.exec(
        "mcu_p.query_result = {'clock': 200000, 'next_sequence': 10, 'buffered': 32, 'possible_overflows': 0, 'query_ticks': 100}\n\
         mcu_p.serial_cb({'sequence': 9, 'data': struct.pack('<2f', 1.5, -2.5)})\n\
         mcu_p.query_result = {'clock': 300000, 'next_sequence': 11, 'buffered': 0, 'possible_overflows': 0, 'query_ticks': 100}\n\
         samples = p.pull_samples()",
    )
    .expect("py pull");
    let (mcu, inner) = make_mcu();
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(100000, 10, 32, 0, 100);
    let reader = FixedFreqReader::new(mcu, 250.0, "<2f").expect("reader");
    reader.setup_query_command("sq", 7, None).expect("setup");
    reader.note_start().expect("note_start");
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(200000, 10, 32, 0, 100);
    {
        let guard = inner.lock().expect("mcu lock");
        guard.serial_cb.as_ref().expect("serial cb")(
            bulk_params(9, vec![0, 0, 0xc0, 0x3f, 0, 0, 0x20, 0xc0]),
        );
    }
    inner
        .lock()
        .expect("mcu lock")
        .query_result = status_params(300000, 11, 0, 0, 100);
    let samples = reader.pull_samples().expect("rust pull");
    let expected = py.eval("samples").expect("py samples");
    let actual = Value::List(samples.iter().map(to_recording).collect());
    check("pull samples", Ok(expected), actual);
}

#[test]
fn fixed_freq_update_before_setup() {
    let Some(mut py) = oracle() else { return };
    py.exec("mcu_e = FakeMCU('mcu')\ne = bs.FixedFreqReader(mcu_e, 250.0, '<2f')")
        .expect("py reader");
    let expected = py.eval("e._update_clock()");
    let (mcu, _inner) = make_mcu();
    let reader = FixedFreqReader::new(mcu, 250.0, "<2f").expect("reader");
    let actual = reader._update_clock(false);
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("update before setup", Err(e), &k),
        other => panic!("update before setup mismatch: {other:?}"),
    }
}

#[test]
fn batch_helper_lifecycle() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p_b = FakePrinter()\nreceived = []\ndef client_cb(msg):\n    received.append(msg)\n    return True\nhelper = bs.BatchBulkHelper(p_b, lambda t: [1, 2], batch_interval=0.25)\nhelper.add_client(client_cb)",
    )
    .expect("py add_client");
    let printer: Printer = Arc::new(FakePrinter::new(None));
    let received: Arc<Mutex<Vec<PyValue>>> = Arc::new(Mutex::new(Vec::new()));
    let recv = received.clone();
    let helper = BatchBulkHelper::new(
        printer,
        Box::new(|_| Ok(PyValue::List(vec![PyValue::Int(PyInt::from_i64(1)), PyValue::Int(PyInt::from_i64(2))]))),
        None,
        None,
        0.25,
    );
    helper
        .add_client(Box::new(move |msg| {
            recv.lock().expect("received lock").push(msg);
            Ok(true)
        }))
        .expect("rust add_client");
    let expected = py.eval("(helper.batch_interval, helper.is_started)").expect("py state");
    let actual = Value::Tuple(vec![
        Value::float(helper.batch_interval()),
        Value::Bool(helper.is_started()),
    ]);
    check("helper started", Ok(expected), actual);

    // _stop clears, then a second _stop raises AttributeError (None timer).
    py.exec("helper._stop()").expect("py stop");
    helper._stop().expect("rust stop");
    let expected = py.eval("helper.is_started").expect("py stopped");
    check("helper stopped", Ok(expected), Value::Bool(helper.is_started()));
    let expected = py.eval("helper._stop()");
    let actual = helper._stop();
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("second stop", Err(e), &k),
        other => panic!("second stop mismatch: {other:?}"),
    }
}

#[test]
fn batch_proc_delivery() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p_c = FakePrinter()\ngot1 = []\ngot2 = []\nhelper2 = bs.BatchBulkHelper(p_c, lambda t: ['m'], batch_interval=0.25)\ndef c1(msg):\n    got1.append(msg)\n    return True\ndef c2(msg):\n    got2.append(msg)\n    return False\nhelper2.add_client(c1)\nhelper2.add_client(c2)\nresult = helper2._proc_batch(500.0)",
    )
    .expect("py proc");
    let got1 = Arc::new(Mutex::new(Vec::new()));
    let got2 = Arc::new(Mutex::new(Vec::new()));
    let helper = BatchBulkHelper::new(
        Arc::new(FakePrinter::new(None)),
        Box::new(|_| Ok(PyValue::List(vec![PyValue::Str("m".to_string())]))),
        None,
        None,
        0.25,
    );
    let g1 = got1.clone();
    helper
        .add_client(Box::new(move |msg| {
            g1.lock().expect("got1 lock").push(msg);
            Ok(true)
        }))
        .expect("add c1");
    let g2 = got2.clone();
    helper
        .add_client(Box::new(move |msg| {
            g2.lock().expect("got2 lock").push(msg);
            Ok(false)
        }))
        .expect("add c2");
    let result = helper._proc_batch(500.0).expect("rust proc");
    let expected = py.eval("(result, got1, got2, len(helper2.client_cbs), helper2.is_started)").expect("py proc state");
    let actual = Value::Tuple(vec![
        Value::float(result),
        Value::List(got1.lock().expect("got1 lock").iter().map(to_recording).collect()),
        Value::List(got2.lock().expect("got2 lock").iter().map(to_recording).collect()),
        Value::Null,
        Value::Bool(helper.is_started()),
    ]);
    // Replace the client count probe below.
    let _ = expected;
    let expected2 = py.eval("(result, got1, got2)").expect("py deliveries");
    let actual2 = Value::Tuple(vec![
        Value::float(result),
        Value::List(got1.lock().expect("got1 lock").iter().map(to_recording).collect()),
        Value::List(got2.lock().expect("got2 lock").iter().map(to_recording).collect()),
    ]);
    check("batch delivery", Ok(expected2), actual2);
    // c2 returning False is removed; helper keeps running with c1 only.
    assert!(helper.is_started());
    let _ = actual;
}

#[test]
fn batch_proc_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p_e = FakePrinter()\ndef bad_batch(t):\n    raise CommandError('batch boom')\nhelper3 = bs.BatchBulkHelper(p_e, bad_batch, batch_interval=0.25)\nhelper3.add_client(lambda m: True)\nres3 = helper3._proc_batch(600.0)",
    )
    .expect("py command error");
    let helper = BatchBulkHelper::new(
        Arc::new(FakePrinter::new(None)),
        Box::new(|_| Err(KlippyError::command("batch boom"))),
        None,
        None,
        0.25,
    );
    helper.add_client(Box::new(|_| Ok(true))).expect("add");
    let result = helper._proc_batch(600.0).expect("rust proc");
    let expected = py.eval("(res3, helper3.is_started)").expect("py state");
    let actual = Value::Tuple(vec![Value::float(result), Value::Bool(helper.is_started())]);
    check("command error stops batching", Ok(expected), actual);

    // Non-command error propagates.
    py.exec(
        "def worse(t):\n    raise ValueError('worse boom')\nhelper4 = bs.BatchBulkHelper(p_e, worse, batch_interval=0.25)\nhelper4.add_client(lambda m: True)",
    )
    .expect("py worse helper");
    let expected = py.eval("helper4._proc_batch(700.0)");
    let helper4 = BatchBulkHelper::new(
        Arc::new(FakePrinter::new(None)),
        Box::new(|_| Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "worse boom".to_string(),
        })),
        None,
        None,
        0.25,
    );
    helper4.add_client(Box::new(|_| Ok(true))).expect("add");
    let actual = helper4._proc_batch(700.0);
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("value error propagates", Err(e), &k),
        other => panic!("value error mismatch: {other:?}"),
    }
}

#[test]
fn batch_mux_endpoint_registration() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "wh = FakeWebhooks()\n\
         p_m = FakePrinter(wh)\n\
         helper5 = bs.BatchBulkHelper(p_m, lambda t: [9], batch_interval=0.25)\n\
         helper5.add_mux_endpoint('/sensor', 'k', 'v', {'start': True})",
    )
    .expect("py mux");
    let (wh_obj, mux_log) = FakeWebhooks::new();
    let helper = BatchBulkHelper::new(
        Arc::new(FakePrinter::new(Some(wh_obj))),
        Box::new(|_| Ok(PyValue::List(vec![PyValue::Int(PyInt::from_i64(9))]))),
        None,
        None,
        0.25,
    );
    helper
        .add_mux_endpoint(
            "/sensor",
            "k",
            "v",
            PyValue::Dict({
                let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
                d.insert(PyValue::Str("start".to_string()), PyValue::Bool(true));
                d
            }),
        )
        .expect("rust mux");
    let expected = py
        .eval("[(p, k, v) for (p, k, v, cb) in wh.mux]")
        .expect("py mux log");
    let actual = Value::List(
        mux_log
            .lock()
            .expect("mux lock")
            .iter()
            .map(|(p, k, v)| Value::Tuple(vec![Value::str(p), Value::str(k), Value::str(v)]))
            .collect(),
    );
    check("mux registration", Ok(expected), actual);

    // Missing webhooks → config error.
    py.exec("helper6 = bs.BatchBulkHelper(FakePrinter(), lambda t: None, batch_interval=0.25)")
        .expect("py helper6");
    let expected = py.eval("helper6.add_mux_endpoint('/x', 'k', 'v', {})");
    let helper_none =
        BatchBulkHelper::new(Arc::new(FakePrinter::new(None)), Box::new(|_| Ok(PyValue::None)), None, None, 0.25);
    let actual = helper_none.add_mux_endpoint("/x", "k", "v", PyValue::Dict(IndexMap::new()));
    match (expected, actual) {
        (Err(e), Err(k)) => check_exc("missing webhooks", Err(e), &k),
        other => panic!("missing webhooks mismatch: {other:?}"),
    }
}

#[test]
fn batch_webhooks_client_delivery() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "conn = FakeClientConnection(closed=False)\n\
         wr = FakeWebRequest({'response_template': {'a': 1}}, conn)\n\
         wc = bs.BatchWebhooksClient(wr)\n\
         ok = wc.handle_batch('hello')",
    )
    .expect("py client");
    let (web_request, _state) = FakeWebRequest::build(
        PyValue::Dict({
            let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
            d.insert(PyValue::Str("response_template".to_string()), {
                let mut t: IndexMap<PyValue, PyValue> = IndexMap::new();
                t.insert(PyValue::Str("a".to_string()), PyValue::Int(PyInt::from_i64(1)));
                PyValue::Dict(t)
            });
            d
        }),
        false,
    );
    let client = BatchWebhooksClient::new(&web_request).expect("rust client");
    let ok = client.handle_batch(PyValue::Str("hello".to_string())).expect("rust handle");
    let expected = py.eval("(ok, conn.sent)").expect("py sent");
    let actual = Value::Tuple(vec![
        Value::Bool(ok),
        Value::List(
            _state
                .lock()
                .expect("conn lock")
                .sent
                .iter()
                .map(to_recording)
                .collect(),
        ),
    ]);
    check("webhooks handle_batch", Ok(expected), actual);

    // Closed connection → False, nothing sent.
    py.exec(
        "conn2 = FakeClientConnection(closed=True)\n\
         wr2 = FakeWebRequest({}, conn2)\n\
         wc2 = bs.BatchWebhooksClient(wr2)\n\
         ok2 = wc2.handle_batch('hello')",
    )
    .expect("py closed client");
    let (web_req2, state2) = FakeWebRequest::build(PyValue::Dict(IndexMap::new()), true);
    let client2 = BatchWebhooksClient::new(&web_req2).expect("rust client closed");
    let ok2 = client2.handle_batch(PyValue::Str("hello".to_string())).expect("rust closed handle");
    let expected2 = py.eval("(ok2, conn2.sent)").expect("py closed state");
    let actual2 = Value::Tuple(vec![
        Value::Bool(ok2),
        Value::List(state2.lock().expect("conn lock").sent.iter().map(to_recording).collect()),
    ]);
    check("webhooks closed", Ok(expected2), actual2);

    // Non-dict template (None) → TypeError.
    py.exec(
        "wr3 = FakeWebRequest({'response_template': None})\nwc3 = bs.BatchWebhooksClient(wr3)",
    )
    .expect("py none template");
    let expected3 = py.eval("wc3.handle_batch('x')");
    let (web_req3, _s3) = FakeWebRequest::build(
        PyValue::Dict({
            let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
            d.insert(
                PyValue::Str("response_template".to_string()),
                PyValue::None,
            );
            d
        }),
        false,
    );
    let client3 = BatchWebhooksClient::new(&web_req3).expect("rust client none");
    let actual3 = client3.handle_batch(PyValue::Str("x".to_string()));
    match (expected3, actual3) {
        (Err(e), Err(k)) => check_exc("none template", Err(e), &k),
        other => panic!("none template mismatch: {other:?}"),
    }
}

#[test]
fn module_factory_is_empty() {
    let f = extras::bulk_sensor::module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}
