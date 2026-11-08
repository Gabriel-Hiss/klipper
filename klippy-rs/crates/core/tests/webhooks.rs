//! Parity tests for `klippy_core::webhooks` (port of `klippy/webhooks.py`).
//!
//! Every expected value comes from the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`). A Python
//! `WebHooks` / `ClientConnection` / `GCodeHelper` / `QueryStatusHelper`
//! is built next to the Rust one with minimal fakes for the printer,
//! reactor, `gcode` and sockets, both sides are fed identical frames, and
//! the request values, endpoint responses, raised errors (class and
//! message), bytes written to the sockets, client state, shutdowns,
//! rollover info and log lines are compared.
//!
//! `pyoracle` evaluates one expression per `eval`, so multi-statement
//! Python goes through `exec` or the helper functions in [`SETUP`].

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts, GetIntOpts, GetOpts,
    OutputCallback, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{ClientConnectionApi, WebHooksApi, WebHooksCallback};
use klippy_core::webhooks::{
    parse_objects_arg, ClientConnHandle, ClientConnection, GCodeHelper, HostInfo,
    QueryStatusHelper, RawSocket, WebHooks, WebRequest,
};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex, NOW};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::os::unix::fs::FileTypeExt;
use std::os::unix::net::UnixStream;
use std::sync::{Arc, LazyLock, Once};
use std::time::Duration;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import webhooks
# The port reproduces the stdlib `json` branch of webhooks.py:9-37.
assert not hasattr(webhooks, 'msgspec'), 'oracle has msgspec installed'
import gcode as _gcode
import configfile as _configfile
import logging as _logging
import socket as _socket
import os as _os
import sys as _sys
import stat as _stat
import contextlib as _contextlib

# Deterministic host info for the `info` endpoint; the Rust side takes the
# same values through `HostInfo`.
_socket.gethostname = lambda: 'host-x'
_os.getpid = lambda: 4242
_os.getuid = lambda: 1000
_os.getgid = lambda: 100
_sys.executable = '/fake/python'
webhooks.__file__ = '/fake/klippy/webhooks.py'

NEVER = 9999999999999999.

class FakeCompletion:
    def __init__(self, reactor=None):
        self.reactor = reactor
        self.result = None
        self.done = False
    def complete(self, r):
        self.result = r
        self.done = True
    def test(self):
        return self.done
    def wait(self, waketime=NEVER, waketime_result=None):
        # A real wait pauses until the reactor runs the query timer.
        if not self.done and self.reactor is not None:
            self.reactor.fire_timers()
        return self.result

class FakeReactor:
    NOW = 0.
    NEVER = NEVER
    def __init__(self):
        self.now = 100.0
        self.timers = []
        self.cbs = []
        self.fds = {}
        self.wakes = []
    def register_fd(self, fd, rcb, wcb=None):
        self.fds[fd] = (rcb, wcb)
        return ('fdh', fd)
    def unregister_fd(self, handle):
        self.fds.pop(handle[1], None)
    def set_fd_wake(self, handle, readable, writeable):
        self.wakes.append((handle[1], readable, writeable))
    def register_callback(self, cb, waketime=None):
        self.cbs.append(cb)
        return FakeCompletion()
    def run_callbacks(self, eventtime):
        cbs = self.cbs
        self.cbs = []
        for cb in cbs:
            cb(eventtime)
    def completion(self):
        return FakeCompletion(self)
    def register_timer(self, cb, waketime=None):
        timer = {'cb': cb, 'alive': True}
        self.timers.append(timer)
        return timer
    def unregister_timer(self, timer):
        timer['alive'] = False
    def fire_timers(self):
        for timer in list(self.timers):
            if timer['alive']:
                timer['cb'](self.now)
    def assert_no_pause(self):
        return _contextlib.nullcontext()
    def monotonic(self):
        return 1000.0

_SENT = object()

class FakePrinter:
    command_error = _gcode.CommandError
    config_error = _configfile.error
    def __init__(self, reactor, start_args):
        self._reactor = reactor
        self._start_args = dict(start_args)
        self.objects = {}
        self.shutdowns = []
        self.rollovers = []
        self.handlers = {}
    def get_reactor(self): return self._reactor
    def get_start_args(self): return self._start_args
    def get_state_message(self): return ('Ready', 'ready')
    def set_rollover_info(self, name, info, log=True):
        self.rollovers.append((name, info))
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def lookup_object(self, name, default=_SENT):
        if name in self.objects:
            return self.objects[name]
        if default is _SENT:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        return list(self.objects.items())

class FakeGCode:
    def __init__(self):
        self.help = {'G1': 'move help', 'M104': 'heat help'}
        self.scripts = []
        self.out_handlers = []
    def get_status(self, eventtime):
        return {'commands': {}}
    def get_command_help(self):
        return self.help
    def run_script(self, script):
        self.scripts.append(script)
        if script == 'FAIL':
            raise _gcode.CommandError('boom')
    def register_output_handler(self, cb):
        self.out_handlers.append(cb)

class FakeSock:
    def __init__(self, fd=999):
        self.q = []
        self.sent = bytearray()
        self.closed = False
        self.send_chunk = None
        self.send_error = None
        self._fd = fd
    def fileno(self):
        return self._fd
    def setblocking(self, v):
        pass
    def recv(self, n):
        item = self.q.pop(0)
        if isinstance(item, Exception):
            raise item
        return item
    def send(self, data):
        if self.send_error is not None:
            raise self.send_error
        if self.send_chunk is not None:
            n = min(self.send_chunk, len(data))
        else:
            n = len(data)
        self.sent += data[:n]
        return n
    def close(self):
        self.closed = True

class FakeStatusObject:
    def __init__(self, status):
        self._status = status
    def get_status(self, eventtime):
        return dict(self._status)

class _Plain:
    pass

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append((record.levelname, record.getMessage()))
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

def _mk(start_args=None):
    reactor = FakeReactor()
    printer = FakePrinter(reactor, start_args or {})
    gcode = FakeGCode()
    printer.objects['gcode'] = gcode
    wh = webhooks.WebHooks(printer)
    printer.objects['webhooks'] = wh
    gh = webhooks.GCodeHelper(printer)
    qh = webhooks.QueryStatusHelper(printer)
    return reactor, printer, gcode, wh, gh, qh

def _conn(sock=None):
    return webhooks.ClientConnection(wh.sconn, FakeSock() if sock is None else sock)

def _req(conn, data):
    if isinstance(data, list):
        data = bytes(data)
    return webhooks.WebRequest(conn, data)

def _safe(v):
    # The recording reader cannot parse the surrogate pairs json.dumps emits
    # for characters above U+FFFF; both sides spell them `<U+XXXX>` instead.
    if isinstance(v, str):
        return ''.join(
            c if ord(c) < 0x10000 and not 0xD800 <= ord(c) < 0xE000
            else '<U+%X>' % ord(c) for c in v)
    if isinstance(v, dict):
        return {_safe(k): _safe(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_safe(x) for x in v]
    if isinstance(v, tuple):
        return tuple(_safe(x) for x in v)
    return v

def _fields(conn, data):
    wr = _req(conn, data)
    return _safe((wr.id, wr.method, wr.params, wr.response, wr.is_error))

def _m(conn, data, name, *args):
    return _safe(getattr(_req(conn, data), name)(*args))

def _call(conn, data):
    wr = _req(conn, data)
    wh.get_callback(wr.get_method())(wr)
    return wr.finish()

def _shake(conn, data):
    res = _call(conn, data)
    out = dict(res['result'])
    curtime = out.pop('curtime')
    return ({'id': res['id'], 'result': out}, type(curtime).__name__)

def _take_logs(*conns):
    out = []
    for lvl, msg in log_lines:
        for c in conns:
            msg = msg.replace(str(c.uid), '<uid>')
        out.append((lvl, msg))
    del log_lines[:]
    return out

def _rollovers(*conns):
    out = []
    for name, info in printer.rollovers:
        for c in conns:
            name = name.replace(str(c.uid), '<uid>')
            if info is not None:
                info = info.replace(str(c.uid), '<uid>')
        out.append((name, info))
    return out

def _qstate():
    return {'timer': qh.query_timer is not None,
            'pending': len(qh.pending_queries),
            'clients': len(qh.clients),
            'last': dict(qh.last_query)}

def _q_flow(conn, data, now, subscribe):
    wr = _req(conn, data)
    reactor.now = now
    if subscribe:
        qh._handle_subscribe(wr)
    else:
        qh._handle_query(wr)
    return (wr.finish(), _qstate())

def _fire(now):
    reactor.now = now
    out = []
    for timer in list(reactor.timers):
        if timer['alive']:
            out.append(timer['cb'](now))
    return (out, _qstate())

def _sub_objects():
    return [dict(v[1]) for v in qh.clients.values()]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    start_args: Mutex<PyValue>,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    shutdowns: Mutex<Vec<String>>,
    rollovers: Mutex<Vec<(String, Option<String>)>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor, start_args: Vec<(&str, PyValue)>) -> FakePrinter {
        let mut map = IndexMap::new();
        for (k, v) in start_args {
            map.insert(py_str(k), v);
        }
        FakePrinter {
            reactor,
            start_args: Mutex::new(PyValue::Dict(map)),
            objects: Mutex::new(IndexMap::new()),
            shutdowns: Mutex::new(Vec::new()),
            rollovers: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
        }
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn shutdowns(&self) -> Vec<String> {
        self.shutdowns.lock().clone()
    }
    fn rollovers(&self) -> Vec<(String, Option<String>)> {
        self.rollovers.lock().clone()
    }
    fn handler_events(&self) -> Vec<String> {
        self.handlers.lock().iter().map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.start_args.lock().clone()
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        if self.objects.lock().contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.lock().get(name).cloned() {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.lock().clone();
        match module {
            None => objs.into_iter().collect(),
            Some(m) => {
                let mut out = Vec::new();
                if let Some(o) = objs.get(m).cloned() {
                    out.push((m.to_string(), o));
                }
                let prefix = format!("{m} ");
                for (n, o) in objs.iter() {
                    if n.starts_with(&prefix) {
                        out.push((n.clone(), o.clone()));
                    }
                }
                out
            }
        }
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
    fn set_rollover_info(&self, name: &str, info: Option<&str>, _log: bool) {
        self.rollovers
            .lock()
            .push((name.to_string(), info.map(|s| s.to_string())));
    }
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn register_event_handler(
        &self,
        event: &str,
        callback: EventCallback,
    ) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut guard = self.handlers.lock();
        for (name, cb) in guard.iter_mut() {
            if name == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

fn py_str(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn py_dict(pairs: Vec<(PyValue, PyValue)>) -> PyValue {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k, v);
    }
    PyValue::Dict(m)
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(pycompat::PyInt::from_i64(v))
}

fn empty_py_dict() -> PyValue {
    PyValue::Dict(IndexMap::new())
}

/// The `gcode` fake: the same command help, script log and output
/// handlers as the Python `FakeGCode`.
struct FakeGCode {
    help: IndexMap<String, String>,
    scripts: Mutex<Vec<String>>,
    out_handlers: Mutex<Vec<OutputCallback>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> FakeGCode {
        let mut help = IndexMap::new();
        help.insert("G1".to_string(), "move help".to_string());
        help.insert("M104".to_string(), "heat help".to_string());
        FakeGCode {
            help,
            scripts: Mutex::new(Vec::new()),
            out_handlers: Mutex::new(Vec::new()),
            reactor,
        }
    }
    fn emit(&self, msg: &str) {
        let mut handlers = self.out_handlers.lock();
        handlers[0](msg.to_string());
    }
}

impl GCodeApi for FakeGCode {
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
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        self.help.clone()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        py_dict(vec![(py_str("commands"), empty_py_dict())])
    }
    fn register_output_handler(&self, callback: OutputCallback) {
        self.out_handlers.lock().push(callback);
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().push(script.to_string());
        if script == "FAIL" {
            return Err(KlippyError::Command("boom".to_string()));
        }
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd)
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
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
        Some(py_dict(vec![(py_str("commands"), empty_py_dict())]))
    }
}

struct FakeGCmd;

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_int(&self, _name: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        String::new()
    }
    fn get_commandline(&self) -> String {
        String::new()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

/// A printer object whose `get_status` returns a settable dict, or that
/// has no `get_status` at all (`status` is `None`).
struct FakeStatusObject {
    status: Mutex<Option<PyValue>>,
}

impl FakeStatusObject {
    fn with(status: Vec<(&str, PyValue)>) -> FakeStatusObject {
        let obj = FakeStatusObject {
            status: Mutex::new(None),
        };
        obj.set(status);
        obj
    }
    fn plain() -> FakeStatusObject {
        FakeStatusObject {
            status: Mutex::new(None),
        }
    }
    fn set(&self, status: Vec<(&str, PyValue)>) {
        *self.status.lock() = Some(py_dict(
            status.into_iter().map(|(k, v)| (py_str(k), v)).collect(),
        ));
    }
}

impl PrinterObject for FakeStatusObject {
    klippy_api::printer_object_base!();
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        self.status.lock().clone()
    }
}

struct FakeSock {
    recv: Mutex<VecDeque<Result<Vec<u8>, i32>>>,
    sent: Mutex<Vec<u8>>,
    closed: Mutex<bool>,
    send_chunk: Mutex<Option<usize>>,
    send_error: Mutex<Option<i32>>,
}

impl FakeSock {
    fn new() -> FakeSock {
        FakeSock {
            recv: Mutex::new(VecDeque::new()),
            sent: Mutex::new(Vec::new()),
            closed: Mutex::new(false),
            send_chunk: Mutex::new(None),
            send_error: Mutex::new(None),
        }
    }
    fn queue(&self, data: Vec<u8>) {
        self.recv.lock().push_back(Ok(data));
    }
    fn queue_err(&self, errno: i32) {
        self.recv.lock().push_back(Err(errno));
    }
    fn sent(&self) -> Value {
        Value::bytes(&self.sent.lock().clone())
    }
}

impl RawSocket for FakeSock {
    fn fileno(&self) -> i32 {
        999
    }
    fn recv(&self, _max: usize) -> Result<Vec<u8>, i32> {
        self.recv.lock().pop_front().expect("queued recv")
    }
    fn send(&self, data: &[u8]) -> Result<usize, i32> {
        if let Some(e) = *self.send_error.lock() {
            return Err(e);
        }
        let n = match *self.send_chunk.lock() {
            Some(c) => c.min(data.len()),
            None => data.len(),
        };
        self.sent.lock().extend_from_slice(&data[..n]);
        Ok(n)
    }
    fn close(&self) {
        *self.closed.lock() = true;
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

type LogLines = Vec<(String, String)>;

/// Tests share the process-wide log sink and run one at a time.
static SERIAL: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));
static LOGS: LazyLock<Mutex<LogLines>> = LazyLock::new(|| Mutex::new(Vec::new()));
static LOG_INIT: Once = Once::new();

fn install_log_sink() {
    LOG_INIT.call_once(|| {
        log::set_sink(Box::new(|rec| {
            LOGS.lock()
                .push((rec.level.name().to_string(), rec.message.to_string()));
        }));
    });
}

fn take_logs() -> LogLines {
    std::mem::take(&mut *LOGS.lock())
}

fn norm_text(s: &str, uids: &[u64]) -> String {
    let mut out = s.to_string();
    for uid in uids {
        out = out.replace(&uid.to_string(), "<uid>");
    }
    out
}

/// The Rust log lines since the last call, shaped like Python's `_take_logs`.
fn logs_value(uids: &[u64]) -> Value {
    Value::List(
        take_logs()
            .into_iter()
            .map(|(level, msg)| {
                Value::Tuple(vec![Value::str(&level), Value::str(&norm_text(&msg, uids))])
            })
            .collect(),
    )
}

fn rollovers_value(rs: &RustSide, uids: &[u64]) -> Value {
    Value::List(
        rs.fake
            .rollovers()
            .into_iter()
            .map(|(name, info)| {
                Value::Tuple(vec![
                    Value::str(&norm_text(&name, uids)),
                    match info {
                        Some(s) => Value::str(&norm_text(&s, uids)),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

fn shutdowns_value(rs: &RustSide) -> Value {
    Value::List(rs.fake.shutdowns().iter().map(|s| Value::str(s)).collect())
}

fn rx_record(scenario: &str) -> Record {
    Record {
        v: 1,
        unit: "webhooks".to_string(),
        scenario: scenario.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

/// Compares an oracle result with the Rust outcome: values by type and
/// bits, errors by Python class and message.
fn compare(label: &str, py: pyoracle::OracleResult, rust: KResult<Value>) {
    match (py, rust) {
        (Ok(expected), Ok(actual)) => {
            let record = rx_record(label);
            if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
                panic!("{label}: {d}");
            }
        }
        (Err(e), Err(a)) => assert_eq!(
            (a.class_name(), a.message()),
            (e.class_name.as_str(), e.message.as_str()),
            "{label}: exception mismatch"
        ),
        (Ok(expected), Err(a)) => panic!(
            "{label}: oracle returned {expected:?}, Rust raised {}: {}",
            a.class_name(),
            a.message()
        ),
        (Err(e), Ok(actual)) => panic!(
            "{label}: oracle raised {}: {}, Rust returned {actual:?}",
            e.class_name, e.message
        ),
    }
}

/// `compare` for errors whose message has an address in it: the class
/// and the text up to `prefix_end` must match.
fn compare_prefix(label: &str, py: pyoracle::OracleResult, rust: KResult<Value>, prefix_end: &str) {
    match (py, rust) {
        (Err(e), Err(a)) => {
            assert_eq!(a.class_name(), e.class_name.as_str(), "{label}: class");
            let cut = |m: &str| m.split(prefix_end).next().unwrap_or("").to_string();
            assert!(
                e.message.contains(prefix_end) && a.message().contains(prefix_end),
                "{label}: py {:?} rust {:?}",
                e.message,
                a.message()
            );
            assert_eq!(cut(a.message()), cut(&e.message), "{label}: message prefix");
        }
        (py, rust) => panic!("{label}: expected errors, py={py:?} rust={rust:?}"),
    }
}

/// Spells characters above U+FFFF `<U+XXXX>`, like `_safe` in [`SETUP`].
fn safe_str(s: &str) -> String {
    s.chars()
        .map(|c| {
            if (c as u32) < 0x10000 {
                c.to_string()
            } else {
                format!("<U+{:X}>", c as u32)
            }
        })
        .collect()
}

fn from_py(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(i) => Value::Int(i),
            Err(_) => Value::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(&safe_str(s)),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(from_py).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(from_py).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (from_py(k), from_py(v))).collect()),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn opt(v: Option<PyValue>) -> Value {
    from_py(&v.unwrap_or(PyValue::None))
}

fn vdict(pairs: Vec<(&str, Value)>) -> Value {
    Value::Dict(pairs.into_iter().map(|(k, v)| (Value::str(k), v)).collect())
}

/// A Python `bytes` expression for the frame.
fn pyb(data: &[u8]) -> String {
    format!("bytes({data:?})")
}

/// A request frame `{"id": id, "method": .., "params": ..}`.
fn req(method: &str, params: &str, id: &str) -> Vec<u8> {
    format!("{{\"id\":{id},\"method\":\"{method}\",\"params\":{params}}}").into_bytes()
}

/// `frame \x03` for each frame.
fn framed(frames: &[&[u8]]) -> Vec<u8> {
    let mut out = Vec::new();
    for f in frames {
        out.extend_from_slice(f);
        out.push(3);
    }
    out
}

fn uid_of(conn: &Arc<ClientConnection>) -> u64 {
    ClientConnectionApi::uid(&**conn)
}

fn handle(conn: &Arc<ClientConnection>) -> ClientConnHandle {
    conn.clone() as ClientConnHandle
}

fn host_info() -> HostInfo {
    HostInfo {
        hostname: "host-x".to_string(),
        klipper_path: "/fake".to_string(),
        python_path: "/fake/python".to_string(),
        process_id: 4242,
        user_id: 1000,
        group_id: 100,
    }
}

/// Runs the callbacks queued by `register_callback` on the real reactor
/// (the Rust side of the Python `reactor.run_callbacks`).
fn pump_reactor(reactor: &Reactor) {
    let r = reactor.clone();
    let _done = reactor.register_callback(move |_: f64| r.end(), NOW);
    reactor.run();
    reactor.finalize();
}

struct RustSide {
    reactor: Reactor,
    fake: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    wh: Arc<WebHooks>,
    gh: Arc<GCodeHelper>,
    qh: Arc<QueryStatusHelper>,
}

/// `_mk` on the Rust side: `gcode`, `webhooks`, then the two helpers.
fn rust_side(start_args: Vec<(&str, PyValue)>) -> KResult<RustSide> {
    let reactor = Reactor::new(false);
    let fake = Arc::new(FakePrinter::new(reactor.clone(), start_args));
    let printer: Printer = fake.clone() as Printer;
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    fake.add("gcode", gcode.clone() as ObjectRef);
    let wh = WebHooks::new(&printer, host_info())?;
    fake.add("webhooks", wh.clone() as ObjectRef);
    let gh = GCodeHelper::new(&printer)?;
    let qh = QueryStatusHelper::new(&printer)?;
    Ok(RustSide {
        reactor,
        fake,
        gcode,
        wh,
        gh,
        qh,
    })
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("webhooks fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping webhooks oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// One Python and one Rust webhooks stack built from the same start args.
struct Rig {
    py: pyoracle::Oracle,
    rs: RustSide,
    _serial: kos::sync::MutexGuard<'static, ()>,
}

impl Rig {
    fn new(py_args: &str, rs_args: Vec<(&str, PyValue)>) -> Option<Rig> {
        let serial = SERIAL.lock();
        install_log_sink();
        let mut py = oracle()?;
        py.exec(&format!(
            "reactor, printer, gcode, wh, gh, qh = _mk({py_args})"
        ))
        .expect("python webhooks setup");
        py.exec("del log_lines[:]").expect("clear python log");
        let rs = rust_side(rs_args).expect("rust webhooks setup");
        take_logs();
        Some(Rig {
            py,
            rs,
            _serial: serial,
        })
    }

    /// A Rust client on the fake socket; the Python twin is made with
    /// `py_conn`.
    fn rust_conn(&self) -> (Arc<ClientConnection>, Arc<FakeSock>) {
        let sock = Arc::new(FakeSock::new());
        let conn = ClientConnection::new(&self.rs.wh.get_connection(), sock.clone());
        (conn, sock)
    }

    /// `<name>sock = FakeSock(); <name> = _conn(<name>sock)`.
    fn py_conn(&mut self, name: &str) {
        self.py
            .exec(&format!("{name}sock = FakeSock()\n{name} = _conn({name}sock)"))
            .expect("python client");
    }

    fn pump(&mut self) {
        self.py
            .exec("reactor.run_callbacks(20.0)")
            .expect("python callbacks");
        pump_reactor(&self.rs.reactor);
    }

    fn compare_logs(&mut self, label: &str, py_conns: &str, uids: &[u64]) {
        let py = self.py.eval(&format!("_take_logs({py_conns})"));
        compare(label, py, Ok(logs_value(uids)));
    }

    fn compare_rollovers(&mut self, label: &str, py_conns: &str, uids: &[u64]) {
        let py = self.py.eval(&format!("_rollovers({py_conns})"));
        compare(label, py, Ok(rollovers_value(&self.rs, uids)));
    }

    fn compare_shutdowns(&mut self, label: &str) {
        let py = self.py.eval("list(printer.shutdowns)");
        compare(label, py, Ok(shutdowns_value(&self.rs)));
    }
}

/// `_call`: parse a frame, run its endpoint, return `finish()`.
fn call_py(rs: &RustSide, conn: &ClientConnHandle, data: &[u8]) -> KResult<Option<PyValue>> {
    let wr = WebRequest::new(conn.clone(), data)?;
    let cb = rs.wh.get_callback(&wr.get_method())?;
    cb(wr.clone())?;
    Ok(wr.finish())
}

fn call_value(rs: &RustSide, conn: &ClientConnHandle, data: &[u8]) -> KResult<Value> {
    call_py(rs, conn, data).map(opt)
}

// ---------------------------------------------------------------------------
// WebRequest.
// ---------------------------------------------------------------------------

#[test]
fn web_request_parse_and_getters_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rconn, _rsock) = rig.rust_conn();
    let conn = handle(&rconn);

    let huge = format!("1{}", "0".repeat(400));
    let data = format!(
        "{{\"id\": 7, \"method\": \"info\", \"params\": {{\"a\": 1, \"b\": \"x\", \"c\": 2.5, \
         \"d\": true, \"e\": null, \"f\": [1, 2], \"g\": {{\"k\": 1}}, \"huge\": {huge}, \
         \"neg0\": -0, \"negf0\": -0.0, \"exp\": 1E5, \"big\": 12345678901234567890, \
         \"s\": \"\\u00e9\\n\\\"q\\\" \\ud83d\\ude00\", \"dup\": 1, \"dup\": 3}}}}"
    )
    .into_bytes();
    let pyd = pyb(&data);
    let wr = WebRequest::new(conn.clone(), &data).expect("parse");
    compare(
        "request fields",
        rig.py.eval(&format!("_fields(c, {pyd})")),
        Ok(Value::Tuple(vec![
            opt(wr.id()),
            Value::str(&wr.method()),
            from_py(&wr.params()),
            opt(wr.response()),
            Value::Bool(wr.is_error()),
        ])),
    );
    compare(
        "get_method / get_client_connection",
        rig.py
            .eval(&format!("_m(c, {pyd}, 'get_method')")),
        Ok(Value::str(&wr.get_method())),
    );

    let s = |v: String| Value::str(&v);
    let cases: Vec<(&str, &str, KResult<Value>)> = vec![
        ("get a", "'get', 'a'", wr.get("a", None).map(|v| from_py(&v))),
        (
            "get missing",
            "'get', 'zzz'",
            wr.get("zzz", None).map(|v| from_py(&v)),
        ),
        (
            "get default",
            "'get', 'zzz', 'def'",
            wr.get("zzz", Some(py_str("def"))).map(|v| from_py(&v)),
        ),
        (
            "get None default",
            "'get', 'zzz', None",
            wr.get("zzz", Some(PyValue::None)).map(|v| from_py(&v)),
        ),
        ("get null", "'get', 'e'", wr.get("e", None).map(|v| from_py(&v))),
        ("get_str b", "'get_str', 'b'", wr.get_str("b", None).map(s)),
        ("get_str a", "'get_str', 'a'", wr.get_str("a", None).map(s)),
        ("get_str null", "'get_str', 'e'", wr.get_str("e", None).map(s)),
        (
            "get_str default",
            "'get_str', 'zzz', 'dflt'",
            wr.get_str("zzz", Some("dflt".to_string())).map(s),
        ),
        (
            "get_str missing",
            "'get_str', 'zzz'",
            wr.get_str("zzz", None).map(s),
        ),
        (
            "get_int a",
            "'get_int', 'a'",
            wr.get_int("a", None).map(Value::int),
        ),
        (
            "get_int bool",
            "'get_int', 'd'",
            wr.get_int("d", None).map(Value::int),
        ),
        (
            "get_int float",
            "'get_int', 'c'",
            wr.get_int("c", None).map(Value::int),
        ),
        (
            "get_int default",
            "'get_int', 'zzz', 5",
            wr.get_int("zzz", Some(5)).map(Value::int),
        ),
        (
            "get_int missing",
            "'get_int', 'zzz'",
            wr.get_int("zzz", None).map(Value::int),
        ),
        (
            "get_float c",
            "'get_float', 'c'",
            wr.get_float("c", None).map(Value::float),
        ),
        (
            "get_float int",
            "'get_float', 'a'",
            wr.get_float("a", None).map(Value::float),
        ),
        (
            "get_float big int",
            "'get_float', 'big'",
            wr.get_float("big", None).map(Value::float),
        ),
        (
            "get_float huge int",
            "'get_float', 'huge'",
            wr.get_float("huge", None).map(Value::float),
        ),
        (
            "get_float str",
            "'get_float', 'b'",
            wr.get_float("b", None).map(Value::float),
        ),
        (
            "get_float bool",
            "'get_float', 'd'",
            wr.get_float("d", None).map(Value::float),
        ),
        (
            "get_float default",
            "'get_float', 'zzz', 2.5",
            wr.get_float("zzz", Some(2.5)).map(Value::float),
        ),
        (
            "get_float missing",
            "'get_float', 'zzz'",
            wr.get_float("zzz", None).map(Value::float),
        ),
        (
            "get_dict g",
            "'get_dict', 'g'",
            wr.get_dict("g", None).map(|v| from_py(&v)),
        ),
        (
            "get_dict list",
            "'get_dict', 'f'",
            wr.get_dict("f", None).map(|v| from_py(&v)),
        ),
        (
            "get_dict None default",
            "'get_dict', 'zzz', None",
            wr.get_dict("zzz", Some(PyValue::None)).map(|v| from_py(&v)),
        ),
        (
            "get_dict {} default",
            "'get_dict', 'zzz', {}",
            wr.get_dict("zzz", Some(empty_py_dict())).map(|v| from_py(&v)),
        ),
        (
            "get_dict missing",
            "'get_dict', 'zzz'",
            wr.get_dict("zzz", None).map(|v| from_py(&v)),
        ),
    ];
    for (label, args, rust) in cases {
        compare(label, rig.py.eval(&format!("_m(c, {pyd}, {args})")), rust);
    }

    // finish() stores the default `{}`, so a later send is rejected.
    rig.py.exec(&format!("wa = _req(c, {pyd})")).expect("py wa");
    let wa = WebRequest::new(conn.clone(), &data).expect("wa");
    compare("finish default", rig.py.eval("wa.finish()"), Ok(opt(wa.finish())));
    compare(
        "send after finish",
        rig.py.eval("wa.send({'x': 1})"),
        wa.send(py_dict(vec![(py_str("x"), py_int(1))])).map(|_| Value::Null),
    );
    compare(
        "response after finish",
        rig.py.eval("(wa.response, wa.is_error)"),
        Ok(Value::Tuple(vec![opt(wa.response()), Value::Bool(wa.is_error())])),
    );

    // send, finish, second send.
    rig.py.exec(&format!("wb = _req(c, {pyd})")).expect("py wb");
    let wb = WebRequest::new(conn.clone(), &data).expect("wb");
    compare(
        "send ok",
        rig.py.eval("wb.send({'ok': 1})"),
        wb.send(py_dict(vec![(py_str("ok"), py_int(1))])).map(|_| Value::Null),
    );
    compare("finish sent", rig.py.eval("wb.finish()"), Ok(opt(wb.finish())));
    compare(
        "double send",
        rig.py.eval("wb.send({'ok': 2})"),
        wb.send(py_dict(vec![(py_str("ok"), py_int(2))])).map(|_| Value::Null),
    );

    // set_error.
    rig.py.exec(&format!("wc = _req(c, {pyd})")).expect("py wc");
    let wc = WebRequest::new(conn.clone(), &data).expect("wc");
    rig.py
        .exec("wc.set_error(webhooks.WebRequestError('bad thing'))")
        .expect("py set_error");
    wc.set_error(KlippyError::Other {
        class_name: "WebRequestError".to_string(),
        message: "bad thing".to_string(),
    });
    compare(
        "send after set_error",
        rig.py.eval("wc.send({'late': 1})"),
        wc.send(py_dict(vec![(py_str("late"), py_int(1))])).map(|_| Value::Null),
    );
    compare(
        "finish error",
        rig.py.eval("(wc.finish(), wc.is_error)"),
        Ok(Value::Tuple(vec![opt(wc.finish()), Value::Bool(wc.is_error())])),
    );

    // A notification has no id: finish() is None and stores nothing.
    let ntf = b"{\"method\": \"m\", \"params\": {}}".to_vec();
    rig.py
        .exec(&format!("wd = _req(c, {})", pyb(&ntf)))
        .expect("py wd");
    let wd = WebRequest::new(conn.clone(), &ntf).expect("wd");
    compare(
        "notification send",
        rig.py.eval("wd.send({'n': 1})"),
        wd.send(py_dict(vec![(py_str("n"), py_int(1))])).map(|_| Value::Null),
    );
    compare(
        "notification finish",
        rig.py.eval("(wd.finish(), wd.response, wd.id)"),
        Ok(Value::Tuple(vec![opt(wd.finish()), opt(wd.response()), opt(wd.id())])),
    );
}

#[test]
fn web_request_parse_errors_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rconn, _rsock) = rig.rust_conn();
    let conn = handle(&rconn);

    let good = b"{\"id\":1,\"method\":\"m\",\"params\":{\"a\":\"\xc3\xa9 \xf0\x9f\x98\x80\"}}".to_vec();
    let utf16le = |s: &str| -> Vec<u8> { s.encode_utf16().flat_map(|u| u.to_le_bytes()).collect() };
    let utf16be = |s: &str| -> Vec<u8> { s.encode_utf16().flat_map(|u| u.to_be_bytes()).collect() };
    let utf32le = |s: &str| -> Vec<u8> { s.chars().flat_map(|c| (c as u32).to_le_bytes()).collect() };
    let text = "{\"id\":1,\"method\":\"m\",\"params\":{\"a\":\"\u{e9} \u{1f600}\"}}";
    let with_prefix = |prefix: &[u8], body: Vec<u8>| -> Vec<u8> {
        let mut v = prefix.to_vec();
        v.extend(body);
        v
    };
    let mut bad_cont = b"{\"id\":1,\"method\":\"m\",\"params\":{\"a\":\"".to_vec();
    bad_cont.extend_from_slice(b"\xc3(\"}}");
    let mut truncated = b"{\"id\":1,\"method\":\"m\",\"params\":{\"a\":\"".to_vec();
    truncated.extend_from_slice(b"\xe2\x82");
    let mut bad_start = b"{\"id\":1,\"method\":\"m\",\"params\":{\"a\":\"".to_vec();
    bad_start.extend_from_slice(b"\xff\"}}");

    let cases: Vec<(&str, Vec<u8>)> = vec![
        ("good", good.clone()),
        ("top-level list", b"[1, 2]".to_vec()),
        ("top-level string", b"\"hello\"".to_vec()),
        ("top-level int", b"42".to_vec()),
        ("method int", b"{\"id\":1,\"method\":7,\"params\":{}}".to_vec()),
        ("params list", b"{\"id\":1,\"method\":\"m\",\"params\":[]}".to_vec()),
        ("params null", b"{\"id\":1,\"method\":\"m\",\"params\":null}".to_vec()),
        ("no method", b"{\"id\":1,\"params\":{}}".to_vec()),
        ("no params", b"{\"id\":1,\"method\":\"m\"}".to_vec()),
        ("null id", b"{\"id\":null,\"method\":\"m\"}".to_vec()),
        ("string id", b"{\"id\":\"abc\",\"method\":\"m\"}".to_vec()),
        ("float id", b"{\"id\":1.5,\"method\":\"m\"}".to_vec()),
        ("bad json", b"{not json}".to_vec()),
        ("empty", Vec::new()),
        ("whitespace", b"   ".to_vec()),
        (
            "trailing data",
            b"{\"id\":1,\"method\":\"m\",\"params\":{}} trailing".to_vec(),
        ),
        ("unterminated", b"{\"id\":1,\"method\":".to_vec()),
        ("single byte 0xff", b"\xff".to_vec()),
        ("bom only utf-16", b"\xff\xfe".to_vec()),
        ("invalid start byte", bad_start),
        ("invalid continuation byte", bad_cont),
        ("unexpected end of data", truncated),
        ("utf-8 bom", with_prefix(b"\xef\xbb\xbf", good.clone())),
        (
            "double utf-8 bom",
            with_prefix(b"\xef\xbb\xbf\xef\xbb\xbf", good.clone()),
        ),
        ("utf-16-le no bom", utf16le(text)),
        ("utf-16-le bom", with_prefix(b"\xff\xfe", utf16le(text))),
        ("utf-16-be bom", with_prefix(b"\xfe\xff", utf16be(text))),
        ("utf-16-be no bom", utf16be(text)),
        ("utf-32-le no bom", utf32le(text)),
        ("utf-32-le bom", with_prefix(b"\xff\xfe\x00\x00", utf32le(text))),
    ];
    for (label, data) in cases {
        let py = rig.py.eval(&format!("_fields(c, {})", pyb(&data)));
        let rust = WebRequest::new(conn.clone(), &data).map(|wr| {
            Value::Tuple(vec![
                opt(wr.id()),
                Value::str(&wr.method()),
                from_py(&wr.params()),
                opt(wr.response()),
                Value::Bool(wr.is_error()),
            ])
        });
        compare(&format!("parse {label}"), py, rust);
    }
}

// ---------------------------------------------------------------------------
// WebHooks endpoints.
// ---------------------------------------------------------------------------

#[test]
fn builtin_endpoints_match_python() {
    let start = "{'log_file': '/tmp/k.log', 'config_file': '/tmp/p.cfg', \
                 'software_version': 'v0.12-t', 'cpu_info': 'cpu x'}";
    let Some(mut rig) = Rig::new(
        start,
        vec![
            ("log_file", py_str("/tmp/k.log")),
            ("config_file", py_str("/tmp/p.cfg")),
            ("software_version", py_str("v0.12-t")),
            ("cpu_info", py_str("cpu x")),
        ],
    ) else {
        return;
    };
    rig.py_conn("c");
    let (rconn, _rsock) = rig.rust_conn();
    let conn = handle(&rconn);
    let uids = [uid_of(&rconn)];
    rig.compare_logs("connection logs", "c", &uids);

    let run = |rig: &mut Rig, label: &str, data: &[u8]| {
        compare(
            label,
            rig.py.eval(&format!("_call(c, {})", pyb(data))),
            call_value(&rig.rs, &conn, data),
        );
    };

    // Registration order of every endpoint: webhooks, gcode, objects.
    run(&mut rig, "list_endpoints", &req("list_endpoints", "{}", "1"));

    // shakehands embeds the live clock: compare everything but curtime.
    let shake = req("shakehands", "{}", "7");
    let res = call_py(&rig.rs, &conn, &shake)
        .expect("shakehands")
        .expect("shakehands result");
    let PyValue::Dict(mut m) = res else {
        panic!("shakehands result is not a dict");
    };
    let Some(PyValue::Dict(mut inner)) = m.get(&py_str("result")).cloned() else {
        panic!("shakehands has no result dict");
    };
    let curtime = inner.shift_remove(&py_str("curtime")).expect("curtime");
    m.insert(py_str("result"), PyValue::Dict(inner));
    compare(
        "shakehands",
        rig.py.eval(&format!("_shake(c, {})", pyb(&shake))),
        Ok(Value::Tuple(vec![
            from_py(&PyValue::Dict(m)),
            Value::str(match curtime {
                PyValue::Float(_) => "float",
                _ => "other",
            }),
        ])),
    );

    // info: host fields, start args, and the client info logging.
    run(
        &mut rig,
        "info",
        &req("info", "{\"client_info\": {\"program\": \"moon\", \"version\": \"1\"}}", "3"),
    );
    run(&mut rig, "info without client_info", &req("info", "{}", "4"));
    run(&mut rig, "info empty client_info", &req("info", "{\"client_info\": {}}", "5"));
    run(&mut rig, "info bad client_info", &req("info", "{\"client_info\": 5}", "6"));
    rig.compare_logs("info logs", "c", &uids);
    rig.compare_rollovers("info rollovers", "c", &uids);

    // A notification (no id) runs the endpoint and returns None.
    let ntf = b"{\"method\": \"list_endpoints\"}".to_vec();
    run(&mut rig, "notification", &ntf);

    // emergency_stop.
    run(&mut rig, "emergency_stop", &req("emergency_stop", "{}", "8"));
    rig.compare_shutdowns("estop shutdown");

    // get_status and stats of the webhooks object.
    compare(
        "get_status",
        rig.py.eval("wh.get_status(1.5)"),
        Ok(from_py(&WebHooksApi::get_status(&*rig.rs.wh, 1.5))),
    );
    compare(
        "printer object get_status",
        rig.py.eval("wh.get_status(1.5)"),
        Ok(opt(PrinterObject::get_status(&*rig.rs.wh, 1.5))),
    );
    compare(
        "stats",
        rig.py.eval("wh.stats(1.5)"),
        Ok(match PrinterObject::stats(&*rig.rs.wh, 1.5) {
            Some((flag, text)) => Value::Tuple(vec![Value::Bool(flag), Value::str(&text)]),
            None => Value::Null,
        }),
    );

    // Unknown endpoint: error and an info log line.
    compare(
        "unknown endpoint",
        rig.py.eval("wh.get_callback('does/not/exist')"),
        rig.rs.wh.get_callback("does/not/exist").map(|_| Value::Null),
    );
    run(&mut rig, "unknown endpoint request", &req("does/not/exist", "{}", "9"));
    rig.compare_logs("unknown endpoint logs", "c", &uids);
}

#[test]
fn register_endpoint_conflict_matches_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    let cb: WebHooksCallback = Arc::new(|_wr| Ok(()));
    for path in ["shakehands", "list_endpoints", "gcode/help", "objects/query"] {
        compare(
            &format!("duplicate {path}"),
            rig.py
                .eval(&format!("wh.register_endpoint('{path}', lambda wr: None)")),
            rig.rs
                .wh
                .register_endpoint(path, cb.clone())
                .map(|_| Value::Null),
        );
    }
    compare(
        "new endpoint",
        rig.py.eval("wh.register_endpoint('my/path', lambda wr: None)"),
        rig.rs.wh.register_endpoint("my/path", cb.clone()).map(|_| Value::Null),
    );
    compare(
        "new endpoint twice",
        rig.py.eval("wh.register_endpoint('my/path', lambda wr: None)"),
        rig.rs.wh.register_endpoint("my/path", cb.clone()).map(|_| Value::Null),
    );
    // A mux endpoint cannot take over a plain endpoint.
    compare(
        "mux on plain endpoint",
        rig.py
            .eval("wh.register_mux_endpoint('my/path', 'k', 'v', lambda wr: None)"),
        rig.rs
            .wh
            .register_mux_endpoint("my/path", "k", "v", cb.clone())
            .map(|_| Value::Null),
    );
    compare(
        "list after registrations",
        rig.py.eval("list(wh._endpoints.keys())"),
        Ok(Value::List(
            rig.rs
                .wh
                .endpoint_paths()
                .iter()
                .map(|p| Value::str(p))
                .collect(),
        )),
    );
}

#[test]
fn mux_endpoints_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rconn, _rsock) = rig.rust_conn();
    let conn = handle(&rconn);
    rig.py
        .exec(
            "def _mux_a(wr):\n    wr.send({'which': 'a', 'params': wr.params})\n\
             def _mux_b(wr):\n    wr.send({'which': 'b', 'params': wr.params})\n\
             def _mux_n(wr):\n    wr.send({'which': 'n'})",
        )
        .expect("py mux callbacks");

    let mk = |which: &'static str| -> WebHooksCallback {
        Arc::new(move |wr| {
            let mut pairs = vec![(py_str("which"), py_str(which))];
            if which != "n" {
                pairs.push((py_str("params"), wr.params()));
            }
            wr.send(py_dict(pairs))
        })
    };
    for (value, name) in [("a", "_mux_a"), ("b", "_mux_b")] {
        compare(
            &format!("mux register {value}"),
            rig.py.eval(&format!(
                "wh.register_mux_endpoint('mux/test', 'sensor', '{value}', {name})"
            )),
            rig.rs
                .wh
                .register_mux_endpoint("mux/test", "sensor", value, mk(if value == "a" { "a" } else { "b" }))
                .map(|_| Value::Null),
        );
    }
    compare(
        "mux register second path",
        rig.py
            .eval("wh.register_mux_endpoint('mux/other', 'name', 'n', _mux_n)"),
        rig.rs
            .wh
            .register_mux_endpoint("mux/other", "name", "n", mk("n"))
            .map(|_| Value::Null),
    );

    // The key must stay the same for a path; the message is exact.
    compare(
        "mux key violation",
        rig.py
            .eval("wh.register_mux_endpoint('mux/test', 'otherkey', 'c', _mux_a)"),
        rig.rs
            .wh
            .register_mux_endpoint("mux/test", "otherkey", "c", mk("a"))
            .map(|_| Value::Null),
    );
    // A repeated value lists the registered callbacks (reprs carry
    // addresses), so only class and the text before the list compare.
    compare_prefix(
        "mux duplicate value",
        rig.py
            .eval("wh.register_mux_endpoint('mux/test', 'sensor', 'a', _mux_a)"),
        rig.rs
            .wh
            .register_mux_endpoint("mux/test", "sensor", "a", mk("a"))
            .map(|_| Value::Null),
        " already registered (",
    );
    compare(
        "endpoint order",
        rig.py.eval("list(wh._endpoints.keys())"),
        Ok(Value::List(
            rig.rs
                .wh
                .endpoint_paths()
                .iter()
                .map(|p| Value::str(p))
                .collect(),
        )),
    );

    let requests: Vec<(&str, Vec<u8>)> = vec![
        ("dispatch a", req("mux/test", "{\"sensor\": \"a\", \"extra\": [1]}", "1")),
        ("dispatch b", req("mux/test", "{\"sensor\": \"b\"}", "2")),
        ("dispatch other path", req("mux/other", "{\"name\": \"n\"}", "3")),
        ("invalid value", req("mux/test", "{\"sensor\": \"zzz\"}", "4")),
        ("missing key", req("mux/test", "{}", "5")),
        ("int key", req("mux/test", "{\"sensor\": 5}", "6")),
        ("float key", req("mux/test", "{\"sensor\": 1.5}", "7")),
        ("bool key", req("mux/test", "{\"sensor\": true}", "8")),
        ("null key", req("mux/test", "{\"sensor\": null}", "9")),
        ("list key", req("mux/test", "{\"sensor\": [1]}", "10")),
        ("dict key", req("mux/test", "{\"sensor\": {\"a\": 1}}", "11")),
        ("str key with quote", req("mux/test", "{\"sensor\": \"it's\"}", "12")),
    ];
    for (label, data) in requests {
        compare(
            label,
            rig.py.eval(&format!("_call(c, {})", pyb(&data))),
            call_value(&rig.rs, &conn, &data),
        );
    }
}

#[test]
fn remote_methods_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("r1");
    rig.py_conn("r2");
    let (rconn1, rsock1) = rig.rust_conn();
    let (rconn2, rsock2) = rig.rust_conn();
    let c1 = handle(&rconn1);
    let c2 = handle(&rconn2);
    let uids = [uid_of(&rconn1), uid_of(&rconn2)];
    rig.compare_logs("connection logs", "r1, r2", &uids);

    let register = |rig: &mut Rig, label: &str, py_conn: &str, conn: &ClientConnHandle, params: &str| {
        let data = req("register_remote_method", params, "11");
        compare(
            label,
            rig.py.eval(&format!("_call({py_conn}, {})", pyb(&data))),
            call_value(&rig.rs, conn, &data),
        );
    };
    register(
        &mut rig,
        "register r1",
        "r1",
        &c1,
        "{\"response_template\": {\"id\": 77, \"jsonrpc\": \"2.0\"}, \"remote_method\": \"rm\"}",
    );
    register(
        &mut rig,
        "register r2",
        "r2",
        &c2,
        "{\"response_template\": {\"params\": \"mine\"}, \"remote_method\": \"rm\"}",
    );
    register(
        &mut rig,
        "register missing template",
        "r1",
        &c1,
        "{\"remote_method\": \"rm\"}",
    );
    register(
        &mut rig,
        "register bad template",
        "r1",
        &c1,
        "{\"response_template\": 5, \"remote_method\": \"rm\"}",
    );
    register(
        &mut rig,
        "register missing method",
        "r1",
        &c1,
        "{\"response_template\": {}}",
    );
    register(
        &mut rig,
        "register bad method",
        "r1",
        &c1,
        "{\"response_template\": {}, \"remote_method\": 5}",
    );
    // Registering again replaces the template and keeps the position.
    register(
        &mut rig,
        "register r1 again",
        "r1",
        &c1,
        "{\"response_template\": {\"id\": 78}, \"remote_method\": \"rm\"}",
    );
    register(
        &mut rig,
        "register r1 second method",
        "r1",
        &c1,
        "{\"response_template\": {\"id\": 1}, \"remote_method\": \"other\"}",
    );
    rig.compare_logs("registration logs", "r1, r2", &uids);

    let mut params = IndexMap::new();
    params.insert("x".to_string(), py_int(1));
    params.insert("y".to_string(), PyValue::List(vec![py_int(1), py_str("two")]));
    compare(
        "call rm",
        rig.py.eval("wh.call_remote_method('rm', x=1, y=[1, 'two'])"),
        rig.rs.wh.call_remote_method("rm", &params).map(|_| Value::Null),
    );
    compare("r1 received", rig.py.eval("bytes(r1sock.sent)"), Ok(rsock1.sent()));
    compare("r2 received", rig.py.eval("bytes(r2sock.sent)"), Ok(rsock2.sent()));

    compare(
        "call missing method",
        rig.py.eval("wh.call_remote_method('nope')"),
        rig.rs.wh.call_remote_method("nope", &IndexMap::new()).map(|_| Value::Null),
    );

    // r1 closes: only r2 is served and r1 is dropped from the map.
    rig.py.exec("r1.close()").expect("py close");
    ClientConnectionApi::close(&*rconn1);
    compare(
        "call after r1 closed",
        rig.py.eval("wh.call_remote_method('rm', x=2)"),
        {
            let mut p = IndexMap::new();
            p.insert("x".to_string(), py_int(2));
            rig.rs.wh.call_remote_method("rm", &p).map(|_| Value::Null)
        },
    );
    compare("r1 sent after close", rig.py.eval("bytes(r1sock.sent)"), Ok(rsock1.sent()));
    compare("r2 sent after r1 closed", rig.py.eval("bytes(r2sock.sent)"), Ok(rsock2.sent()));

    // The other method only had r1: no active connection, then gone.
    compare(
        "call method with closed conn",
        rig.py.eval("wh.call_remote_method('other')"),
        rig.rs.wh.call_remote_method("other", &IndexMap::new()).map(|_| Value::Null),
    );
    compare(
        "call method after removal",
        rig.py.eval("wh.call_remote_method('other')"),
        rig.rs.wh.call_remote_method("other", &IndexMap::new()).map(|_| Value::Null),
    );
    rig.py.exec("r2.close()").expect("py close r2");
    ClientConnectionApi::close(&*rconn2);
    compare(
        "call with no active connections",
        rig.py.eval("wh.call_remote_method('rm')"),
        rig.rs.wh.call_remote_method("rm", &IndexMap::new()).map(|_| Value::Null),
    );
    compare(
        "call after removal",
        rig.py.eval("wh.call_remote_method('rm')"),
        rig.rs.wh.call_remote_method("rm", &IndexMap::new()).map(|_| Value::Null),
    );
    rig.compare_logs("close logs", "r1, r2", &uids);
}

// ---------------------------------------------------------------------------
// ClientConnection framing, sends and request dispatch.
// ---------------------------------------------------------------------------

#[test]
fn client_framing_and_sends_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, rsock) = rig.rust_conn();
    let uids = [uid_of(&rc)];
    rig.compare_logs("new connection logs", "c", &uids);
    rig.compare_rollovers("new connection rollovers", "c", &uids);

    // Two frames and half of a third in one read, the rest of the third,
    // a notification and the start of a fifth in the next.
    let f1 = req("list_endpoints", "{}", "1");
    let f2 = req(
        "info",
        "{\"client_info\": {\"program\": \"x\", \"version\": \"1\"}}",
        "2",
    );
    let partial = req("emergency_stop", "{}", "3");
    let split = partial.len() / 2;
    let notification = b"{\"method\": \"info\"}".to_vec();
    let tail = b"{\"id\": 5, \"met".to_vec();
    let mut chunk1 = framed(&[&f1, &f2]);
    chunk1.extend_from_slice(&partial[..split]);
    let mut chunk2 = partial[split..].to_vec();
    chunk2.push(3);
    chunk2.extend(framed(&[&notification]));
    chunk2.extend_from_slice(&tail);
    rsock.queue(chunk1.clone());
    rsock.queue(chunk2.clone());
    rig.py
        .exec(&format!(
            "csock.q.extend([{}, {}])",
            pyb(&chunk1),
            pyb(&chunk2)
        ))
        .expect("py queue");

    rig.py.exec("c.process_received(10.0)").expect("py recv 1");
    rc.process_received(10.0);
    compare(
        "partial data 1",
        rig.py.eval("c.partial_data"),
        Ok(Value::bytes(&rc.partial_data())),
    );
    rig.pump();
    rig.py.exec("c.process_received(11.0)").expect("py recv 2");
    rc.process_received(11.0);
    compare(
        "partial data 2",
        rig.py.eval("c.partial_data"),
        Ok(Value::bytes(&rc.partial_data())),
    );
    rig.pump();

    compare("bytes sent", rig.py.eval("bytes(csock.sent)"), Ok(rsock.sent()));
    rig.compare_shutdowns("shutdowns");
    rig.compare_logs("frame logs", "c", &uids);
    rig.compare_rollovers("frame rollovers", "c", &uids);
    let log_value = |rc: &Arc<ClientConnection>| {
        Value::List(
            rc.request_log()
                .iter()
                .map(|(t, r)| Value::Tuple(vec![Value::float(*t), Value::bytes(r)]))
                .collect(),
        )
    };
    compare("request log", rig.py.eval("list(c.request_log)"), Ok(log_value(&rc)));

    // The request log keeps the last 20 frames.
    let many: Vec<Vec<u8>> = (0..25)
        .map(|i| format!("{{\"method\": \"list_endpoints\", \"n\": {i}}}").into_bytes())
        .collect();
    let refs: Vec<&[u8]> = many.iter().map(|v| v.as_slice()).collect();
    let chunk3 = framed(&refs);
    rsock.queue(chunk3.clone());
    rig.py
        .exec(&format!("csock.q.append({})", pyb(&chunk3)))
        .expect("py queue 3");
    rig.py.exec("c.process_received(12.0)").expect("py recv 3");
    rc.process_received(12.0);
    rig.pump();
    compare("rotated request log", rig.py.eval("list(c.request_log)"), Ok(log_value(&rc)));
    compare(
        "request log length",
        rig.py.eval("len(c.request_log)"),
        Ok(Value::int(rc.request_log().len() as i64)),
    );
    rig.compare_logs("rotation logs", "c", &uids);

    // dump_request_log writes one log record.
    rig.py.exec("c.dump_request_log()").expect("py dump");
    ClientConnectionApi::dump_request_log(&*rc);
    rig.compare_logs("dump logs", "c", &uids);

    // An empty read is a closed socket.
    rsock.queue(Vec::new());
    rig.py.exec("csock.q.append(b'')").expect("py queue empty");
    rig.py.exec("c.process_received(13.0)").expect("py recv empty");
    rc.process_received(13.0);
    compare(
        "closed state",
        rig.py.eval("(c.is_closed(), csock.closed)"),
        Ok(Value::Tuple(vec![
            Value::Bool(ClientConnectionApi::is_closed(&*rc)),
            Value::Bool(*rsock.closed.lock()),
        ])),
    );
    rig.compare_logs("close logs", "c", &uids);
    rig.compare_rollovers("close rollovers", "c", &uids);

    // Closing twice is a no-op, and a send on a closed client only buffers.
    rig.py.exec("c.close()").expect("py close again");
    ClientConnectionApi::close(&*rc);
    rig.py.exec("c.send({'late': 1})").expect("py late send");
    ClientConnectionApi::send(&*rc, py_dict(vec![(py_str("late"), py_int(1))]));
    compare(
        "send after close",
        rig.py.eval("(bytes(c.send_buffer), bytes(csock.sent))"),
        Ok(Value::Tuple(vec![Value::bytes(&rc.send_buffer()), rsock.sent()])),
    );
    rig.compare_logs("late logs", "c", &uids);
}

#[test]
fn client_request_dispatch_errors_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, rsock) = rig.rust_conn();
    let uids = [uid_of(&rc)];
    rig.py
        .exec(
            "def _raise_cmd(wr):\n    raise printer.command_error('cmd failed')\n\
             def _raise_web(wr):\n    raise webhooks.WebRequestError('web failed')\n\
             def _raise_other(wr):\n    raise ValueError('oops')\n\
             def _send_twice(wr):\n    wr.send({'a': 1})\n    wr.send({'a': 2})\n\
             def _echo(wr):\n    wr.send({'echo': wr.params})\n\
             wh.register_endpoint('t/cmd', _raise_cmd)\n\
             wh.register_endpoint('t/web', _raise_web)\n\
             wh.register_endpoint('t/other', _raise_other)\n\
             wh.register_endpoint('t/twice', _send_twice)\n\
             wh.register_endpoint('t/echo', _echo)",
        )
        .expect("py endpoints");
    let reg = |path: &str, cb: WebHooksCallback| {
        rig.rs.wh.register_endpoint(path, cb).expect("register");
    };
    reg(
        "t/cmd",
        Arc::new(|_wr| Err(KlippyError::Command("cmd failed".to_string()))),
    );
    reg(
        "t/web",
        Arc::new(|_wr| {
            Err(KlippyError::Other {
                class_name: "WebRequestError".to_string(),
                message: "web failed".to_string(),
            })
        }),
    );
    reg(
        "t/other",
        Arc::new(|_wr| {
            Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "oops".to_string(),
            })
        }),
    );
    reg(
        "t/twice",
        Arc::new(|wr| {
            wr.send(py_dict(vec![(py_str("a"), py_int(1))]))?;
            wr.send(py_dict(vec![(py_str("a"), py_int(2))]))
        }),
    );
    reg(
        "t/echo",
        Arc::new(|wr| wr.send(py_dict(vec![(py_str("echo"), wr.params())]))),
    );
    rig.compare_logs("setup logs", "c", &uids);

    let frames: Vec<Vec<u8>> = vec![
        req("t/cmd", "{}", "1"),
        req("t/web", "{}", "2"),
        req("t/other", "{}", "3"),
        req("t/twice", "{}", "4"),
        req("t/echo", "{\"k\": [1, 2.5, \"s\", null, true]}", "5"),
        req("no/such/path", "{}", "6"),
        b"{\"method\": \"t/other\"}".to_vec(),
        b"{\"method\": \"t/cmd\"}".to_vec(),
        req("list_endpoints", "{}", "\"str-id\""),
        b"{broken".to_vec(),
    ];
    let refs: Vec<&[u8]> = frames.iter().map(|f| f.as_slice()).collect();
    let chunk = framed(&refs);
    rsock.queue(chunk.clone());
    rig.py
        .exec(&format!("csock.q.append({})", pyb(&chunk)))
        .expect("py queue");
    rig.py.exec("c.process_received(5.0)").expect("py recv");
    rc.process_received(5.0);
    rig.pump();
    compare("bytes sent", rig.py.eval("bytes(csock.sent)"), Ok(rsock.sent()));
    rig.compare_shutdowns("shutdowns");
    rig.compare_logs("dispatch logs", "c", &uids);
}

#[test]
fn client_write_paths_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, rsock) = rig.rust_conn();
    let uids = [uid_of(&rc)];
    let state = |rc: &Arc<ClientConnection>, sock: &FakeSock| {
        Value::Tuple(vec![
            Value::Bool(rc.is_blocking()),
            Value::int(rc.blocking_count()),
            Value::bytes(&rc.send_buffer()),
            sock.sent(),
            Value::Bool(ClientConnectionApi::is_closed(&**rc)),
        ])
    };
    let py_state = "(c.is_blocking, c.blocking_count, bytes(c.send_buffer), bytes(csock.sent), c.is_closed())";

    // Partial writes of 5 bytes block the client and keep the rest.
    *rsock.send_chunk.lock() = Some(5);
    rig.py.exec("csock.send_chunk = 5").expect("py chunk");
    let first = py_dict(vec![(py_str("k"), py_str("v"))]);
    rig.py.exec("c.send({'k': 'v'})").expect("py send");
    ClientConnectionApi::send(&*rc, first);
    compare("after first send", rig.py.eval(py_state), Ok(state(&rc, &rsock)));

    // While blocked, a send only appends to the buffer.
    rig.py.exec("c.send({'k2': 'v2'})").expect("py send 2");
    ClientConnectionApi::send(&*rc, py_dict(vec![(py_str("k2"), py_str("v2"))]));
    compare("after blocked send", rig.py.eval(py_state), Ok(state(&rc, &rsock)));

    // A write event drains 5 more bytes and stays blocked.
    rig.py.exec("c._do_send(40.0)").expect("py do_send");
    rc.do_send(Some(40.0));
    compare("after second write", rig.py.eval(py_state), Ok(state(&rc, &rsock)));

    // Without the chunk limit the buffer drains and the block clears.
    *rsock.send_chunk.lock() = None;
    rig.py
        .exec("csock.send_chunk = None\nc._do_send(41.0)")
        .expect("py drain");
    rc.do_send(Some(41.0));
    compare("after drain", rig.py.eval(py_state), Ok(state(&rc, &rsock)));
    compare(
        "no wake logs",
        rig.py.eval("_take_logs(c)"),
        Ok(logs_value(&uids)),
    );
    rig.compare_shutdowns("no shutdowns");

    // `ServerSocket.stats` counts a blocked client down and closes it.
    let server = rig.rs.wh.get_connection();
    server.track_client(rc.clone());
    rig.py.exec("wh.sconn.clients[c.uid] = c").expect("py track");
    rc.set_blocking(true);
    rc.set_blocking_count(2);
    rig.py
        .exec("c.is_blocking = True\nc.blocking_count = 2")
        .expect("py blocking");
    for round in 0..4 {
        compare(
            &format!("stats result {round}"),
            rig.py.eval("wh.sconn.stats(50.0)"),
            Ok({
                let (flag, text) = server.stats(50.0);
                Value::Tuple(vec![Value::Bool(flag), Value::str(&text)])
            }),
        );
        compare(
            &format!("stats state {round}"),
            rig.py
                .eval("(c.blocking_count, c.is_closed(), len(wh.sconn.clients), csock.closed)"),
            Ok(Value::Tuple(vec![
                Value::int(rc.blocking_count()),
                Value::Bool(ClientConnectionApi::is_closed(&*rc)),
                Value::int(server.client_count() as i64),
                Value::Bool(*rsock.closed.lock()),
            ])),
        );
    }
    rig.compare_logs("stats logs", "c", &uids);
    rig.compare_rollovers("stats rollovers", "c", &uids);

    // A write error other than EAGAIN closes the client.
    rig.py_conn("e");
    let (re, resock) = rig.rust_conn();
    let euids = [uid_of(&re)];
    rig.compare_logs("setup logs", "e", &euids);
    *resock.send_error.lock() = Some(32);
    rig.py
        .exec("import errno\nesock.send_error = OSError(errno.EPIPE, 'Broken pipe')")
        .expect("py epipe");
    rig.py.exec("e.send({'a': 1})").expect("py send epipe");
    ClientConnectionApi::send(&*re, py_dict(vec![(py_str("a"), py_int(1))]));
    compare(
        "epipe state",
        rig.py
            .eval("(e.is_blocking, bytes(e.send_buffer), e.is_closed(), esock.closed)"),
        Ok(Value::Tuple(vec![
            Value::Bool(re.is_blocking()),
            Value::bytes(&re.send_buffer()),
            Value::Bool(ClientConnectionApi::is_closed(&*re)),
            Value::Bool(*resock.closed.lock()),
        ])),
    );
    rig.compare_logs("epipe logs", "e", &euids);

    // EAGAIN counts as a zero-byte write and blocks.
    rig.py_conn("g");
    let (rg, rgsock) = rig.rust_conn();
    let guids = [uid_of(&rg)];
    rig.compare_logs("setup logs g", "g", &guids);
    *rgsock.send_error.lock() = Some(11);
    rig.py
        .exec("gsock.send_error = OSError(errno.EAGAIN, 'again')")
        .expect("py eagain");
    rig.py.exec("g.send({'a': 1})").expect("py send eagain");
    ClientConnectionApi::send(&*rg, py_dict(vec![(py_str("a"), py_int(1))]));
    compare(
        "eagain state",
        rig.py
            .eval("(g.is_blocking, g.blocking_count, bytes(g.send_buffer), g.is_closed())"),
        Ok(Value::Tuple(vec![
            Value::Bool(rg.is_blocking()),
            Value::int(rg.blocking_count()),
            Value::bytes(&rg.send_buffer()),
            Value::Bool(ClientConnectionApi::is_closed(&*rg)),
        ])),
    );

    // JSON encoding: an unserializable value shuts down, NaN is allowed.
    rig.py_conn("j");
    let (rj, rjsock) = rig.rust_conn();
    let juids = [uid_of(&rj)];
    rig.compare_logs("setup logs j", "j", &juids);
    rig.py.exec("j.send({'a': b'x'})").expect("py send bytes");
    ClientConnectionApi::send(&*rj, py_dict(vec![(py_str("a"), PyValue::Bytes(b"x".to_vec()))]));
    rig.compare_shutdowns("json shutdown");
    rig.compare_logs("json logs", "j", &juids);
    rig.py
        .exec("j.send({'n': float('nan'), 'i': float('inf'), 'f': 1.0, 'u': '\\u00e9', 'l': [1, None, True]})")
        .expect("py send nan");
    ClientConnectionApi::send(
        &*rj,
        py_dict(vec![
            (py_str("n"), PyValue::Float(f64::NAN)),
            (py_str("i"), PyValue::Float(f64::INFINITY)),
            (py_str("f"), PyValue::Float(1.0)),
            (py_str("u"), py_str("\u{e9}")),
            (
                py_str("l"),
                PyValue::List(vec![py_int(1), PyValue::None, PyValue::Bool(true)]),
            ),
        ]),
    );
    compare(
        "json bytes",
        rig.py.eval("(bytes(jsock.sent), bytes(j.send_buffer))"),
        Ok(Value::Tuple(vec![rjsock.sent(), Value::bytes(&rj.send_buffer())])),
    );
}

#[test]
fn client_read_errors_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("a");
    rig.py_conn("b");
    let (ra, rasock) = rig.rust_conn();
    let (rb, rbsock) = rig.rust_conn();
    let uids = [uid_of(&ra), uid_of(&rb)];
    rig.compare_logs("setup logs", "a, b", &uids);

    // EBADF closes the client through the empty-data path.
    rasock.queue_err(9);
    rig.py
        .exec("import errno\nasock.q.append(OSError(errno.EBADF, 'bad fd'))")
        .expect("py ebadf");
    rig.py.exec("a.process_received(14.0)").expect("py recv a");
    ra.process_received(14.0);
    compare(
        "EBADF closes",
        rig.py.eval("(a.is_closed(), asock.closed)"),
        Ok(Value::Tuple(vec![
            Value::Bool(ClientConnectionApi::is_closed(&*ra)),
            Value::Bool(*rasock.closed.lock()),
        ])),
    );

    // Any other read error is ignored.
    rbsock.queue_err(5);
    rig.py
        .exec("bsock.q.append(OSError(errno.EIO, 'io error'))")
        .expect("py eio");
    rig.py.exec("b.process_received(15.0)").expect("py recv b");
    rb.process_received(15.0);
    compare(
        "EIO ignored",
        rig.py.eval("(b.is_closed(), bsock.closed, b.partial_data, len(b.request_log))"),
        Ok(Value::Tuple(vec![
            Value::Bool(ClientConnectionApi::is_closed(&*rb)),
            Value::Bool(*rbsock.closed.lock()),
            Value::bytes(&rb.partial_data()),
            Value::int(rb.request_log().len() as i64),
        ])),
    );
    rig.compare_logs("read error logs", "a, b", &uids);
    rig.compare_rollovers("read error rollovers", "a, b", &uids);
}

// ---------------------------------------------------------------------------
// ServerSocket.
// ---------------------------------------------------------------------------

fn socket_path(tag: &str, side: &str) -> String {
    format!(
        "{}/klippy_wh_{}_{tag}_{side}",
        std::env::temp_dir().display(),
        std::process::id()
    )
}

fn read_frame(stream: &mut UnixStream) -> Vec<u8> {
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .expect("read timeout");
    let mut out = Vec::new();
    let mut buf = [0u8; 4096];
    loop {
        let n = stream.read(&mut buf).expect("read response");
        if n == 0 {
            break;
        }
        out.extend_from_slice(&buf[..n]);
        if out.contains(&3) {
            break;
        }
    }
    out
}

#[test]
fn server_socket_config_matches_python() {
    // Disabled servers: no apiserver, a falsy one, or `debuginput`.
    let py_path = socket_path("cfg", "py");
    let rs_path = socket_path("cfg", "rs");
    for (label, py_args, rs_args) in [
        ("no apiserver", "{}".to_string(), vec![]),
        (
            "empty apiserver",
            "{'apiserver': ''}".to_string(),
            vec![("apiserver", py_str(""))],
        ),
        (
            "none apiserver",
            "{'apiserver': None, 'debuginput': None}".to_string(),
            vec![("apiserver", PyValue::None), ("debuginput", PyValue::None)],
        ),
        (
            "debuginput",
            format!("{{'apiserver': '{py_path}', 'debuginput': 'x.gcode'}}"),
            vec![
                ("apiserver", py_str(&rs_path)),
                ("debuginput", py_str("x.gcode")),
            ],
        ),
        (
            "empty debuginput string",
            format!("{{'apiserver': '{py_path}', 'debuginput': ''}}"),
            vec![
                ("apiserver", py_str(&rs_path)),
                ("debuginput", py_str("")),
            ],
        ),
    ] {
        let Some(mut rig) = Rig::new(&py_args, rs_args) else {
            return;
        };
        compare(
            &format!("{label}: enabled"),
            rig.py.eval("wh.sconn.sock is not None"),
            Ok(Value::Bool(rig.rs.wh.get_connection().is_enabled())),
        );
        compare(
            &format!("{label}: handlers"),
            rig.py.eval("list(printer.handlers.keys())"),
            Ok(Value::List(
                rig.rs
                    .fake
                    .handler_events()
                    .iter()
                    .map(|e| Value::str(e))
                    .collect(),
            )),
        );
        compare(
            &format!("{label}: no socket file"),
            rig.py.eval(&format!("_os.path.exists('{py_path}')")),
            Ok(Value::Bool(std::path::Path::new(&rs_path).exists())),
        );
    }
}

#[test]
fn server_socket_constructor_errors_match_python() {
    let Some(mut py) = oracle() else {
        return;
    };
    let _serial = SERIAL.lock();
    install_log_sink();

    // Bind failure: the directory does not exist.
    let missing = format!(
        "{}/klippy_wh_missing_{}/sock",
        std::env::temp_dir().display(),
        std::process::id()
    );
    compare(
        "bind in missing directory",
        py.exec(&format!("_mk({{'apiserver': '{missing}'}})")),
        rust_side(vec![("apiserver", py_str(&missing))]).map(|_| Value::Null),
    );

    // The path is a directory: os.remove fails and exists() is true.
    let dir = socket_path("dir", "both");
    let _ = std::fs::remove_dir(&dir);
    std::fs::create_dir(&dir).expect("create dir");
    py.exec("del log_lines[:]").expect("clear log");
    take_logs();
    compare(
        "remove a directory",
        py.exec(&format!("_mk({{'apiserver': '{dir}'}})")),
        rust_side(vec![("apiserver", py_str(&dir))]).map(|_| Value::Null),
    );
    compare(
        "remove a directory logs",
        py.eval("_take_logs()"),
        Ok(logs_value(&[])),
    );
    let _ = std::fs::remove_dir(&dir);

    // A truthy non-path `apiserver` fails in os.remove.
    compare(
        "int apiserver",
        py.exec("_mk({'apiserver': 5})"),
        rust_side(vec![("apiserver", py_int(5))]).map(|_| Value::Null),
    );
    compare(
        "list apiserver",
        py.exec("_mk({'apiserver': ['a']})"),
        rust_side(vec![("apiserver", PyValue::List(vec![py_str("a")]))]).map(|_| Value::Null),
    );
}

#[test]
fn server_socket_lifecycle_matches_python() {
    let py_path = socket_path("life", "py");
    let rs_path = socket_path("life", "rs");
    // A stale regular file is removed before binding.
    std::fs::write(&py_path, b"stale").expect("stale py file");
    std::fs::write(&rs_path, b"stale").expect("stale rs file");
    let Some(mut rig) = Rig::new(
        &format!("{{'apiserver': '{py_path}'}}"),
        vec![("apiserver", py_str(&rs_path))],
    ) else {
        let _ = std::fs::remove_file(&py_path);
        let _ = std::fs::remove_file(&rs_path);
        return;
    };
    let server = rig.rs.wh.get_connection();
    compare(
        "enabled",
        rig.py.eval("wh.sconn.sock is not None"),
        Ok(Value::Bool(server.is_enabled())),
    );
    compare(
        "socket file",
        rig.py
            .eval(&format!("_stat.S_ISSOCK(_os.stat('{py_path}').st_mode)")),
        Ok(Value::Bool(
            std::fs::metadata(&rs_path)
                .expect("rs socket file")
                .file_type()
                .is_socket(),
        )),
    );
    compare(
        "handlers",
        rig.py.eval("list(printer.handlers.keys())"),
        Ok(Value::List(
            rig.rs
                .fake
                .handler_events()
                .iter()
                .map(|e| Value::str(e))
                .collect(),
        )),
    );

    // Accept a client on each side.
    let mut py_stream = UnixStream::connect(&py_path).expect("connect py");
    let mut rs_stream = UnixStream::connect(&rs_path).expect("connect rs");
    rig.py.exec("wh.sconn._handle_accept(1.0)").expect("py accept");
    server.handle_accept(1.0);
    compare(
        "clients after accept",
        rig.py.eval("len(wh.sconn.clients)"),
        Ok(Value::int(server.client_count() as i64)),
    );
    let rc = server.clients_snapshot()[0].clone();
    let uids = [uid_of(&rc)];
    rig.py
        .exec("pc = list(wh.sconn.clients.values())[0]")
        .expect("py client");
    rig.compare_logs("accept logs", "pc", &uids);
    rig.compare_rollovers("accept rollovers", "pc", &uids);

    // One request over the real socket.
    let frame = framed(&[&req("info", "{}", "1")]);
    py_stream.write_all(&frame).expect("write py");
    rs_stream.write_all(&frame).expect("write rs");
    rig.py.exec("pc.process_received(2.0)").expect("py recv");
    rc.process_received(2.0);
    rig.pump();
    let py_answer = read_frame(&mut py_stream);
    let rs_answer = read_frame(&mut rs_stream);
    assert!(!py_answer.is_empty(), "the Python server answered");
    assert_eq!(rs_answer, py_answer, "socket response bytes");

    // analyze_shutdown dumps each client's request log.
    rig.py
        .exec("for cb in printer.handlers['klippy:analyze_shutdown']: cb('msg', {})")
        .expect("py analyze");
    rig.rs
        .fake
        .send_event(
            "klippy:analyze_shutdown",
            &[EventArg::Value(py_str("msg")), EventArg::Value(empty_py_dict())],
        )
        .expect("rs analyze");
    rig.compare_logs("analyze logs", "pc", &uids);

    // The peer hangs up: the empty read closes and untracks the client.
    drop(py_stream);
    drop(rs_stream);
    rig.py.exec("pc.process_received(3.0)").expect("py recv eof");
    rc.process_received(3.0);
    compare(
        "clients after hangup",
        rig.py.eval("(len(wh.sconn.clients), pc.is_closed())"),
        Ok(Value::Tuple(vec![
            Value::int(server.client_count() as i64),
            Value::Bool(ClientConnectionApi::is_closed(&*rc)),
        ])),
    );
    rig.compare_logs("hangup logs", "pc", &uids);
    rig.compare_rollovers("hangup rollovers", "pc", &uids);

    // klippy:disconnect closes the clients and the listener.
    let mut py_stream2 = UnixStream::connect(&py_path).expect("connect py 2");
    let mut rs_stream2 = UnixStream::connect(&rs_path).expect("connect rs 2");
    rig.py.exec("wh.sconn._handle_accept(4.0)").expect("py accept 2");
    server.handle_accept(4.0);
    rig.py
        .exec("pc2 = list(wh.sconn.clients.values())[0]")
        .expect("py client 2");
    let rc2 = server.clients_snapshot()[0].clone();
    let uids2 = [uid_of(&rc), uid_of(&rc2)];
    rig.py
        .exec("for cb in printer.handlers['klippy:disconnect']: cb()")
        .expect("py disconnect");
    rig.rs
        .fake
        .send_event("klippy:disconnect", &[])
        .expect("rs disconnect");
    compare(
        "clients after disconnect",
        rig.py.eval("(len(wh.sconn.clients), pc2.is_closed())"),
        Ok(Value::Tuple(vec![
            Value::int(server.client_count() as i64),
            Value::Bool(ClientConnectionApi::is_closed(&*rc2)),
        ])),
    );
    rig.compare_logs("disconnect logs", "pc, pc2", &uids2);
    assert_eq!(
        read_frame(&mut rs_stream2),
        read_frame(&mut py_stream2),
        "closed clients read end of file"
    );
    let py_refused = UnixStream::connect(&py_path).map_err(|e| e.kind());
    let rs_refused = UnixStream::connect(&rs_path).map_err(|e| e.kind());
    assert_eq!(rs_refused.is_err(), py_refused.is_err(), "listener closed");
    assert_eq!(
        rs_refused.err(),
        py_refused.err(),
        "connect error kind after disconnect"
    );
    let _ = std::fs::remove_file(&py_path);
    let _ = std::fs::remove_file(&rs_path);
}

// ---------------------------------------------------------------------------
// GCodeHelper.
// ---------------------------------------------------------------------------

#[test]
fn gcode_helper_endpoints_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, _rsock) = rig.rust_conn();
    let conn = handle(&rc);
    let uids = [uid_of(&rc)];
    rig.compare_logs("setup logs", "c", &uids);

    let run = |rig: &mut Rig, label: &str, data: &[u8]| {
        compare(
            label,
            rig.py.eval(&format!("_call(c, {})", pyb(data))),
            call_value(&rig.rs, &conn, data),
        );
    };
    run(&mut rig, "gcode/help", &req("gcode/help", "{}", "1"));
    run(&mut rig, "gcode/script", &req("gcode/script", "{\"script\": \"M104 S200\"}", "2"));
    run(&mut rig, "gcode/script missing", &req("gcode/script", "{}", "3"));
    run(&mut rig, "gcode/script wrong type", &req("gcode/script", "{\"script\": 5}", "4"));
    run(&mut rig, "gcode/script error", &req("gcode/script", "{\"script\": \"FAIL\"}", "5"));
    run(&mut rig, "gcode/restart", &req("gcode/restart", "{}", "6"));
    run(
        &mut rig,
        "gcode/firmware_restart",
        &req("gcode/firmware_restart", "{}", "7"),
    );
    compare(
        "scripts",
        rig.py.eval("list(gcode.scripts)"),
        Ok(Value::List(
            rig.rs.gcode.scripts.lock().iter().map(|s| Value::str(s)).collect(),
        )),
    );

    // subscribe_output: one handler registration for any number of clients.
    rig.py_conn("s1");
    rig.py_conn("s2");
    let (rs1, rs1sock) = rig.rust_conn();
    let (rs2, rs2sock) = rig.rust_conn();
    let suids = [uid_of(&rs1), uid_of(&rs2)];
    rig.compare_logs("subscriber logs", "s1, s2", &suids);
    let sub = |rig: &mut Rig, label: &str, py_conn: &str, conn: &ClientConnHandle, params: &str, id: &str| {
        let data = req("gcode/subscribe_output", params, id);
        compare(
            label,
            rig.py.eval(&format!("_call({py_conn}, {})", pyb(&data))),
            call_value(&rig.rs, conn, &data),
        );
    };
    sub(
        &mut rig,
        "subscribe s1",
        "s1",
        &handle(&rs1),
        "{\"response_template\": {\"id\": 55, \"params\": \"x\"}}",
        "8",
    );
    sub(&mut rig, "subscribe s2", "s2", &handle(&rs2), "{}", "9");
    sub(
        &mut rig,
        "subscribe bad template",
        "s2",
        &handle(&rs2),
        "{\"response_template\": 3}",
        "10",
    );
    compare(
        "handler registered once",
        rig.py.eval("(len(gcode.out_handlers), gh.is_output_registered, len(gh.clients))"),
        Ok(Value::Tuple(vec![
            Value::int(rig.rs.gcode.out_handlers.lock().len() as i64),
            Value::Bool(rig.rs.gh.is_output_registered()),
            Value::int(rig.rs.gh.client_count() as i64),
        ])),
    );

    rig.py
        .exec("gcode.out_handlers[0]('some output line')")
        .expect("py output");
    rig.rs.gcode.emit("some output line");
    compare("s1 output", rig.py.eval("bytes(s1sock.sent)"), Ok(rs1sock.sent()));
    compare("s2 output", rig.py.eval("bytes(s2sock.sent)"), Ok(rs2sock.sent()));

    // A closed subscriber is dropped on the next output line.
    rig.py.exec("s1.close()").expect("py close s1");
    ClientConnectionApi::close(&*rs1);
    rig.py
        .exec("gcode.out_handlers[0]('second line')")
        .expect("py output 2");
    rig.rs.gcode.emit("second line");
    compare("s1 after close", rig.py.eval("bytes(s1sock.sent)"), Ok(rs1sock.sent()));
    compare("s2 after close", rig.py.eval("bytes(s2sock.sent)"), Ok(rs2sock.sent()));
    compare(
        "subscribers left",
        rig.py.eval("len(gh.clients)"),
        Ok(Value::int(rig.rs.gh.client_count() as i64)),
    );
    rig.compare_logs("output logs", "c, s1, s2", &[uids[0], suids[0], suids[1]]);
}

// ---------------------------------------------------------------------------
// QueryStatusHelper.
// ---------------------------------------------------------------------------

struct QueryObjects {
    toolhead: Arc<FakeStatusObject>,
    heaters: Arc<FakeStatusObject>,
}

/// The same printer objects on both sides, in the same registration order.
fn add_query_objects(rig: &mut Rig) -> QueryObjects {
    rig.py
        .exec(
            "printer.objects['toolhead'] = FakeStatusObject({'x': 1.5, 'y': 2.5, 'name': 'th'})\n\
             printer.objects['heaters'] = FakeStatusObject({'t': 210.0})\n\
             printer.objects['empty'] = FakeStatusObject({})\n\
             printer.objects['plain'] = _Plain()",
        )
        .expect("py objects");
    let toolhead = Arc::new(FakeStatusObject::with(vec![
        ("x", PyValue::Float(1.5)),
        ("y", PyValue::Float(2.5)),
        ("name", py_str("th")),
    ]));
    let heaters = Arc::new(FakeStatusObject::with(vec![("t", PyValue::Float(210.0))]));
    rig.rs.fake.add("toolhead", toolhead.clone() as ObjectRef);
    rig.rs.fake.add("heaters", heaters.clone() as ObjectRef);
    rig.rs
        .fake
        .add("empty", Arc::new(FakeStatusObject::with(vec![])) as ObjectRef);
    rig.rs
        .fake
        .add("plain", Arc::new(FakeStatusObject::plain()) as ObjectRef);
    QueryObjects { toolhead, heaters }
}

fn qstate(qh: &QueryStatusHelper) -> Value {
    vdict(vec![
        ("timer", Value::Bool(qh.has_timer())),
        ("pending", Value::int(qh.pending_len() as i64)),
        ("clients", Value::int(qh.client_count() as i64)),
        (
            "last",
            Value::Dict(
                qh.last_query_snapshot()
                    .iter()
                    .map(|(name, fields)| {
                        (
                            Value::str(name),
                            Value::Dict(
                                fields.iter().map(|(k, v)| (Value::str(k), from_py(v))).collect(),
                            ),
                        )
                    })
                    .collect(),
            ),
        ),
    ])
}

/// `_q_flow`: `_handle_query` / `_handle_subscribe` with the reactor
/// running the query timer at `now` while the request waits.
fn q_flow_rust(
    rs: &RustSide,
    conn: &ClientConnHandle,
    data: &[u8],
    now: f64,
    subscribe: bool,
) -> KResult<Value> {
    let wr = WebRequest::new(conn.clone(), data)?;
    let ticket = rs.qh.begin_query(&wr, subscribe)?;
    rs.qh.do_query(now);
    rs.qh.finish_query(&wr, ticket, subscribe)?;
    Ok(Value::Tuple(vec![opt(wr.finish()), qstate(&rs.qh)]))
}

fn fire_rust(rs: &RustSide, now: f64) -> Value {
    let woke = if rs.qh.has_timer() {
        vec![Value::float(rs.qh.do_query(now))]
    } else {
        vec![]
    };
    Value::Tuple(vec![Value::List(woke), qstate(&rs.qh)])
}

fn sub_objects_rust(rs: &RustSide, uids: &[u64]) -> Value {
    Value::List(
        uids.iter()
            .filter_map(|uid| rs.qh.client_objects(*uid))
            .map(|objects| {
                Value::Dict(
                    objects
                        .iter()
                        .map(|(name, items)| {
                            (
                                Value::str(name),
                                match items {
                                    Some(list) => {
                                        Value::List(list.iter().map(|i| Value::str(i)).collect())
                                    }
                                    None => Value::Null,
                                },
                            )
                        })
                        .collect(),
                )
            })
            .collect(),
    )
}

#[test]
fn query_list_and_validation_match_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, _rsock) = rig.rust_conn();
    let conn = handle(&rc);
    let _objects = add_query_objects(&mut rig);

    // Only objects with a get_status are listed, in registration order.
    let list = req("objects/list", "{}", "1");
    compare(
        "objects/list",
        rig.py.eval(&format!("_call(c, {})", pyb(&list))),
        call_value(&rig.rs, &conn, &list),
    );

    let cases: Vec<(&str, Vec<u8>)> = vec![
        ("objects missing", req("objects/query", "{}", "2")),
        ("objects list", req("objects/query", "{\"objects\": [1]}", "3")),
        ("objects string", req("objects/query", "{\"objects\": \"x\"}", "4")),
        ("value int", req("objects/query", "{\"objects\": {\"a\": 1}}", "5")),
        ("value string", req("objects/query", "{\"objects\": {\"a\": \"x\"}}", "6")),
        ("value dict", req("objects/query", "{\"objects\": {\"a\": {}}}", "7")),
        ("element int", req("objects/query", "{\"objects\": {\"a\": [1]}}", "8")),
        (
            "second element int",
            req("objects/query", "{\"objects\": {\"ok\": [\"x\"], \"a\": [\"x\", 2]}}", "9"),
        ),
        (
            "bad template",
            req("objects/query", "{\"objects\": {}, \"response_template\": 5}", "10"),
        ),
        (
            "invalid objects before template",
            req("objects/query", "{\"objects\": {\"a\": 1}, \"response_template\": 5}", "11"),
        ),
        ("subscribe missing", req("objects/subscribe", "{}", "12")),
        ("subscribe bad value", req("objects/subscribe", "{\"objects\": {\"a\": 1}}", "13")),
    ];
    for (label, data) in cases {
        let is_subscribe = data
            .windows(17)
            .any(|w| w == b"objects/subscribe");
        let py_fn = if is_subscribe { "_handle_subscribe" } else { "_handle_query" };
        let py = rig.py.eval(&format!(
            "(qh.{py_fn}(_req(c, {})), _qstate())",
            pyb(&data)
        ));
        // Validation errors are raised before anything is queued; a
        // request that validated would wait for the timer.
        let rust = WebRequest::new(conn.clone(), &data)
            .and_then(|wr| rig.rs.qh.begin_query(&wr, is_subscribe))
            .map(|_| Value::Null);
        compare(label, py, rust);
    }
    compare(
        "nothing queued",
        rig.py.eval("_qstate()"),
        Ok(qstate(&rig.rs.qh)),
    );
    // `parse_objects_arg` keeps the order of the argument.
    let parsed = parse_objects_arg(&py_dict(vec![
        (py_str("a"), PyValue::None),
        (py_str("b"), PyValue::List(vec![py_str("x")])),
    ]))
    .expect("parse objects");
    assert_eq!(
        parsed.cloned().keys().cloned().collect::<Vec<_>>(),
        vec!["a".to_string(), "b".to_string()]
    );
}

#[test]
fn query_flow_matches_python() {
    let Some(mut rig) = Rig::new("{}", vec![]) else {
        return;
    };
    rig.py_conn("c");
    let (rc, rsock) = rig.rust_conn();
    let conn = handle(&rc);
    let uids = [uid_of(&rc)];
    rig.compare_logs("setup logs", "c", &uids);
    let objs = add_query_objects(&mut rig);

    let flow = |rig: &mut Rig, label: &str, data: &[u8], now: f64, subscribe: bool| {
        compare(
            label,
            rig.py.eval(&format!(
                "_q_flow(c, {}, {now:?}, {})",
                pyb(data),
                if subscribe { "True" } else { "False" }
            )),
            q_flow_rust(&rig.rs, &conn, data, now, subscribe),
        );
    };

    // One-shot queries: field lists, all fields, an unknown object, an
    // empty status and an object without get_status.
    flow(
        &mut rig,
        "query mixed",
        &req(
            "objects/query",
            "{\"objects\": {\"toolhead\": [\"x\", \"y\", \"nope\"], \"heaters\": null, \
             \"unknown_obj\": [\"z\"], \"empty\": null, \"plain\": null, \"gcode\": null}}",
            "1",
        ),
        100.0,
        false,
    );
    // The same query again reports every value again (is_query).
    flow(
        &mut rig,
        "query again",
        &req("objects/query", "{\"objects\": {\"heaters\": null, \"toolhead\": [\"x\"]}}", "2"),
        100.5,
        false,
    );
    flow(
        &mut rig,
        "query empty objects",
        &req("objects/query", "{\"objects\": {}}", "3"),
        101.0,
        false,
    );

    // Subscribe: the first answer has every value, `None` is frozen into
    // the keys the first round saw.
    flow(
        &mut rig,
        "subscribe",
        &req(
            "objects/subscribe",
            "{\"objects\": {\"heaters\": null, \"toolhead\": [\"x\", \"y\"], \"empty\": null, \
             \"unknown_obj\": [\"z\"]}, \"response_template\": {\"id\": 77}}",
            "4",
        ),
        102.0,
        true,
    );
    compare(
        "frozen subscription",
        rig.py.eval("_sub_objects()"),
        Ok(sub_objects_rust(&rig.rs, &[uids[0]])),
    );

    // The second round over an unchanged status sends nothing; a changed
    // value and a new key only report the subscribed, changed fields.
    let fire = |rig: &mut Rig, label: &str, now: f64| {
        compare(
            label,
            rig.py.eval(&format!("_fire({now:?})")),
            Ok(fire_rust(&rig.rs, now)),
        );
        compare(
            &format!("{label}: sent"),
            rig.py.eval("bytes(csock.sent)"),
            Ok(rsock.sent()),
        );
    };
    fire(&mut rig, "refresh unchanged", 102.25);
    objs.heaters.set(vec![("t", PyValue::Float(211.0)), ("target", PyValue::Float(5.0))]);
    rig.py
        .exec("printer.objects['heaters']._status = {'t': 211.0, 'target': 5.0}")
        .expect("py heaters");
    fire(&mut rig, "refresh changed", 102.5);
    objs.toolhead.set(vec![
        ("x", PyValue::Float(1.5)),
        ("y", PyValue::Float(2.75)),
        ("name", py_str("th2")),
    ]);
    rig.py
        .exec("printer.objects['toolhead']._status = {'x': 1.5, 'y': 2.75, 'name': 'th2'}")
        .expect("py toolhead");
    fire(&mut rig, "refresh other object", 102.75);
    fire(&mut rig, "refresh unchanged again", 103.0);
    // An int and a float with the same value are equal.
    objs.heaters.set(vec![("t", py_int(211)), ("target", PyValue::Float(5.0))]);
    rig.py
        .exec("printer.objects['heaters']._status = {'t': 211, 'target': 5.0}")
        .expect("py heaters int");
    fire(&mut rig, "refresh int equals float", 103.25);

    // A new one-shot query during a subscription still reports everything.
    flow(
        &mut rig,
        "query during subscription",
        &req("objects/query", "{\"objects\": {\"heaters\": [\"t\"]}}", "5"),
        103.5,
        false,
    );
    fire(&mut rig, "refresh after query", 103.75);

    // Subscribing again from the same client replaces its subscription.
    flow(
        &mut rig,
        "subscribe again",
        &req(
            "objects/subscribe",
            "{\"objects\": {\"toolhead\": [\"y\"]}, \"response_template\": {\"id\": 78, \"jsonrpc\": \"2.0\"}}",
            "6",
        ),
        104.0,
        true,
    );
    compare(
        "subscription after resubscribe",
        rig.py.eval("_sub_objects()"),
        Ok(sub_objects_rust(&rig.rs, &[uids[0]])),
    );
    objs.toolhead.set(vec![
        ("x", PyValue::Float(1.5)),
        ("y", PyValue::Float(3.0)),
        ("name", py_str("th2")),
    ]);
    rig.py
        .exec("printer.objects['toolhead']._status = {'x': 1.5, 'y': 3.0, 'name': 'th2'}")
        .expect("py toolhead 2");
    fire(&mut rig, "refresh resubscribed", 104.25);

    // A closed client is dropped, which ends the timer.
    rig.py.exec("c.close()").expect("py close");
    ClientConnectionApi::close(&*rc);
    fire(&mut rig, "refresh after close", 104.5);
    fire(&mut rig, "refresh without timer", 104.75);
    rig.compare_logs("query logs", "c", &uids);
    rig.compare_shutdowns("query shutdowns");
}
