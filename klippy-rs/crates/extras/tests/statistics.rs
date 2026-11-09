//! Parity tests for `extras::statistics` (port of `klippy/extras/statistics.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`.
//! The Python side is built with minimal fakes (`FakeReactor`, `FakePrinter`,
//! `FakeConfig`, and a controllable host source in `_vals` standing in for
//! `time.process_time`, `os.getloadavg` and `/proc/meminfo`); the Rust side
//! gets the same fakes through the contract traits: `FakePrinter` for
//! [`PrinterApi`], `FakeHost` for [`HostStats`]. Both sides are then fed the
//! same host values and compared on the `stats` return value, the
//! `get_status` dict, the log lines and the `generate_stats` return.
//!
//! The Rust timer arming (`update_timer(..., NOW)`) has no observable channel
//! with the frozen contracts: the `Reactor` exposes no timer introspection.
//! The Python fake records the arming calls, so that contract is pinned from
//! the reference side; the Rust `handle_ready` follows the same code path
//! and its post-ready `generate_stats` behaviour is compared directly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone.

use extras::statistics::{load_config, HostStats, PrinterStats, PrinterSysStats};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntOpts, GetIntListOpts, GetListsOpts, GetListOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::any::Any;
use std::sync::Arc;
use indexmap::IndexMap;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.statistics as _s
import time as _time
import os as _os
import logging as _logging
from contextlib import contextmanager

_vals = {'ptime': 100.0, 'load': 0.125,
         'meminfo': "MemTotal: 16384000 kB\nMemFree: 4000000 kB\nMemAvailable: 1234567 kB\n",
         'fail_reads': 0, 'open_fails': False}
_last_file = {}

_time.process_time = lambda: _vals['ptime']
_os.getloadavg = lambda: (_vals['load'], 0.0, 0.0)

class FakeMemFile:
    def __init__(self):
        self.closed = False
    def seek(self, pos):
        pass
    def read(self):
        if _vals['fail_reads'] > 0:
            _vals['fail_reads'] -= 1
            raise OSError('simulated read error')
        return _vals['meminfo']
    def close(self):
        self.closed = True

def _fake_open(path, mode='r'):
    if _vals['open_fails']:
        raise OSError('simulated open error')
    _last_file['obj'] = FakeMemFile()
    return _last_file['obj']
_s.open = _fake_open

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
        self.update_calls = []
        self.prevent_pause = False
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append((cb, waketime))
        return ('timer', len(self.timers) - 1)
    def update_timer(self, timer, waketime):
        self.update_calls.append((timer, waketime))
    @contextmanager
    def assert_no_pause(self):
        prev = self.prevent_pause
        self.prevent_pause = True
        try:
            yield
        finally:
            self.prevent_pause = prev

class FakePrinter:
    def __init__(self, reactor, start_args):
        self._reactor = reactor
        self._start_args = start_args
        self.objects = {}
        self.handlers = {}
    def get_reactor(self):
        return self._reactor
    def get_start_args(self):
        return self._start_args
    def add_object(self, name, obj):
        if name in self.objects:
            raise Exception("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        return [(n, self.objects[n]) for n in self.objects
                if n == module or n.startswith(module + ' ')]
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

class FakeProvider:
    def __init__(self, ret, reactor=None):
        self.ret = ret
        self.reactor = reactor
        self.pause_seen = None
    def stats(self, eventtime):
        if self.reactor is not None:
            self.pause_seen = self.reactor.prevent_pause
        return self.ret

class FakeNoStats:
    pass

def _fresh(start_args=None):
    reactor = FakeReactor()
    printer = FakePrinter(reactor, {'debugoutput': None} if start_args is None else start_args)
    config = FakeConfig(printer)
    return reactor, printer, config
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

/// Host values on the Rust side, mirroring `_vals` on the Python side.
struct FakeHost {
    ptime: Mutex<f64>,
    load: Mutex<f64>,
    mem: Mutex<Option<String>>,
}

impl FakeHost {
    fn new() -> FakeHost {
        FakeHost {
            ptime: Mutex::new(100.0),
            load: Mutex::new(0.125),
            mem: Mutex::new(Some(
                "MemTotal: 16384000 kB\nMemFree: 4000000 kB\nMemAvailable: 1234567 kB\n"
                    .to_string(),
            )),
        }
    }
}

impl HostStats for FakeHost {
    fn process_time(&self) -> f64 {
        *self.ptime.lock()
    }
    fn load_avg(&self) -> f64 {
        *self.load.lock()
    }
    fn meminfo(&self) -> Option<String> {
        self.mem.lock().clone()
    }
}

/// A printer object without a `stats` method, like `FakeNoStats` in Python.
struct NoStats;

impl PrinterObject for NoStats {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn Any + Send + Sync> {
        self
    }
}
/// A printer object with a `stats` method, like `FakeProvider` in Python.
struct FakeProvider {
    ret: (bool, String),
    reactor: Reactor,
    pause_seen: Arc<Mutex<Option<bool>>>,
}

impl PrinterObject for FakeProvider {
    klippy_api::printer_object_base!();
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        let probe = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.reactor.verify_can_pause();
        }));
        *self.pause_seen.lock() = Some(probe.is_err());
        Some(self.ret.clone())
    }
}

struct FakePrinter {
    reactor: Reactor,
    start_args: PyValue,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
    added: Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new(reactor: Reactor, start_args: PyValue) -> FakePrinter {
        FakePrinter {
            reactor,
            start_args,
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            added: Mutex::new(Vec::new()),
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.start_args.clone()
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
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        self.added.lock().push(name.to_string());
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
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
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        let out: Vec<(String, ObjectRef)> = match module {
            None => objects
                .iter()
                .map(|(name, obj)| (name.clone(), obj.clone()))
                .collect(),
            Some(module) => {
                let prefix = format!("{module} ");
                objects
                    .iter()
                    .filter(|(name, _)| name.as_str() == module || name.starts_with(&prefix))
                    .map(|(name, obj)| (name.clone(), obj.clone()))
                    .collect()
            }
        };
        out
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, LookupDefault::ReturnNone)
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
        let mut pending: Vec<EventCallback> = Vec::new();
        {
            let mut handlers = self.handlers.lock();
            let mut rest = Vec::new();
            for (name, cb) in std::mem::take(&mut *handlers) {
                if name == event {
                    pending.push(cb);
                } else {
                    rest.push((name, cb));
                }
            }
            *handlers = rest;
        }
        let mut results = Vec::new();
        for cb in pending.iter_mut() {
            results.push(cb(params)?);
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

/// Only `get_printer` is exercised by this module.
struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "statistics".to_string()
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
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
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
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
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

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn pyvalue_to_value(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().expect("int fits i64")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::Host(id) => Value::Opaque {
            type_name: "Host".to_string(),
            addr: format!("0x{id:x}"),
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
        unit: "extras.statistics".to_string(),
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

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("statistics fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping statistics oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

/// The pyoracle helper shared namespace: make a fresh fake printer and a
/// `PrinterSysStats` on both sides with the same host values.
struct Pair {
    sys: PrinterSysStats,
    host: Arc<FakeHost>,
    fake_printer: Arc<FakePrinter>,
}

fn fresh_pair(py: &mut pyoracle::Oracle, start_args: Option<&str>) -> Pair {
    let debugoutput = match start_args {
        None => PyValue::None,
        Some(path) => PyValue::Str(path.to_string()),
    };
    let mut args = IndexMap::new();
    args.insert(PyValue::Str("debugoutput".to_string()), debugoutput);
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor, PyValue::Dict(args)));
    let printer: Printer = fake_printer.clone();
    let host = Arc::new(FakeHost::new());
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let sys = PrinterSysStats::new_with_host(&config, host.clone());
    py.exec("_reactor, _printer, _config = _fresh()").expect("py fresh");
    py.exec("sysstats = _s.PrinterSysStats(_config)").expect("py sysstats");
    Pair {
        sys,
        host,
        fake_printer,
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

// The single `log::info` sink, installed once per process; lines are kept
// per test thread so parallel tests cannot clear or read each other's lines.
thread_local! {
    static LOGS: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| logs.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

fn log_capture() {
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn captured_logs() -> Vec<String> {
    LOGS.with(|logs| logs.borrow().clone())
}

/// Set all three host values on both sides at once.
fn set_host(
    py: &mut pyoracle::Oracle,
    host: &FakeHost,
    ptime: f64,
    load: f64,
    mem: Option<String>,
) {
    *host.ptime.lock() = ptime;
    *host.load.lock() = load;
    *host.mem.lock() = mem.clone();
    let mem_repr = match &mem {
        Some(text) => format!("{text:?}"),
        None => "None".to_string(),
    };
    py.exec(&format!(
        "_vals['ptime'] = {ptime:?}; _vals['load'] = {load:?}; _vals['meminfo'] = {mem_repr}"
    ))
    .expect("set py vals");
}

/// One `stats(10.0)` call on both sides, compared.
fn check_stats(label: &str, py: &mut pyoracle::Oracle, sys: &PrinterSysStats) {
    let expected = py.eval("sysstats.stats(10.0)");
    let (flag, msg) = sys.stats(10.0);
    check(
        label,
        expected,
        Value::Tuple(vec![Value::Bool(flag), Value::str(&msg)]),
    );
}

/// One `get_status(10.0)` on both sides, compared.
fn check_status(label: &str, py: &mut pyoracle::Oracle, sys: &PrinterSysStats) {
    let expected = py.eval("sysstats.get_status(10.0)");
    let status = sys.get_status(10.0).expect("system_stats has get_status");
    check(label, expected, pyvalue_to_value(&status));
}

#[test]
fn sysstats_stats_and_status() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, None);
    // Sequence: zero delta, forward delta, backward delta, missing line,
    // malformed first line, tab separator, underscores in int().
    let steps: Vec<(f64, f64, Option<String>)> = vec![
        (100.0, 0.125, Some("MemTotal: 16384000 kB\nMemFree: 4000000 kB\nMemAvailable: 1234567 kB\n".to_string())),
        (101.25, 0.5, Some("MemTotal: 16384000 kB\nMemFree: 4000000 kB\nMemAvailable: 1234567 kB\n".to_string())),
        (101.25, 0.25, Some("MemAvailable: notanint kB\n".to_string())),
        (99.0, 1.0, Some("Foo: 1\nBar: 2\n".to_string())),
        (102.0, 2.0, Some("MemAvailable: oops\nMemAvailable: 9999 kB\n".to_string())),
        (102.5, 0.0, Some("MemAvailable:\t654321 kB\n".to_string())),
        (103.5, 0.75, Some("MemAvailable: 1_234 kB\n".to_string())),
    ];
    for (i, (ptime, load, mem)) in steps.iter().enumerate() {
        set_host(&mut py, &pair.host, *ptime, *load, mem.clone());
        check_stats(&format!("stats step {i}"), &mut py, &pair.sys);
        check_status(&format!("status step {i}"), &mut py, &pair.sys);
    }
}

#[test]
fn meminfo_open_fails() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, None);
    // `open("/proc/meminfo")` fails in Python at construction, so
    // `mem_file` is None and memory is never read. The Rust host reports
    // `None`, which the port treats the same way.
    set_host(&mut py, &pair.host, 100.0, 0.5, None);
    py.exec("_vals['open_fails'] = True; sysstats = _s.PrinterSysStats(_config); _vals['open_fails'] = False")
        .expect("py open fails");
    check("mem_file none", py.eval("sysstats.mem_file is None"), Value::Bool(true));
    check_stats("stats open-fails", &mut py, &pair.sys);
    check_status("status open-fails", &mut py, &pair.sys);
}

#[test]
fn meminfo_transient_failure() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, None);
    // First read raises in Python (host returns None); second succeeds.
    set_host(&mut py, &pair.host, 100.0, 0.5, None);
    py.exec("_vals['fail_reads'] = 1; _vals['open_fails'] = False")
        .expect("fail one read");
    check_stats("stats read-fails", &mut py, &pair.sys);
    set_host(&mut py, &pair.host, 100.5, 0.5, Some("MemAvailable: 424242 kB\n".to_string()));
    check_stats("stats read-recovers", &mut py, &pair.sys);
    check_status("status read-recovers", &mut py, &pair.sys);
}

#[test]
fn disconnect_closes_mem() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, None);
    set_host(&mut py, &pair.host, 100.0, 0.5, Some("MemAvailable: 111 kB\n".to_string()));
    check_stats("stats before disconnect", &mut py, &pair.sys);
    // Python: the file object is closed and `mem_file` becomes None.
    py.exec("_f = sysstats.mem_file; _printer.send_event('klippy:disconnect')")
        .expect("py disconnect");
    check("file closed", py.eval("_f.closed"), Value::Bool(true));
    check("mem_file none", py.eval("sysstats.mem_file is None"), Value::Bool(true));
    pair
        .fake_printer
        .send_event("klippy:disconnect", &[])
        .expect("rust disconnect");
    // The host would now report new memory, but the port must skip it.
    set_host(&mut py, &pair.host, 101.0, 0.5, Some("MemAvailable: 222 kB\n".to_string()));
    check_stats("stats after disconnect", &mut py, &pair.sys);
    check_status("status after disconnect", &mut py, &pair.sys);
}
#[test]
fn load_config_objects_and_ready_gating() {
    let Some(mut py) = oracle() else { return };
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor.clone(), PyValue::Dict(IndexMap::new())));
    let printer: Printer = fake_printer.clone();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });

    // Python side: load_config through the same fakes.
    py.exec("reactor = FakeReactor(); printer = FakePrinter(reactor, {'debugoutput': None}); config = FakeConfig(printer)").expect("py setup");
    py.exec("ps = _s.load_config(config)").expect("py load_config");
    check("py objects", py.eval("list(printer.objects)"), Value::List(vec![Value::str("system_stats")]));
    check("py return type", py.eval("type(ps).__name__"), Value::str("PrinterStats"));
    check("py no get_status", py.eval("hasattr(ps, 'get_status')"), Value::Bool(false));
    check("py ready handlers", py.eval("len(printer.handlers.get('klippy:ready', []))"), Value::int(1));
    check("py disconnect handlers", py.eval("len(printer.handlers.get('klippy:disconnect', []))"), Value::int(1));

    // Rust side: same calls.
    let returned = load_config(&config).expect("rust load_config");
    assert!(returned.is_some(), "load_config returns PrinterStats");
    assert_eq!(*fake_printer.added.lock(), vec!["system_stats".to_string()]);
    let obj = returned.unwrap();
    assert!(
        klippy_api::downcast::<PrinterStats>(obj.as_ref()).is_some(),
        "load_config returns a PrinterStats"
    );
    // PrinterStats has no get_status (objects/list excludes it).
    assert!(obj.get_status(0.0).is_none(), "PrinterStats has no get_status");

    // `klippy:ready` arms the timer at NOW when debugoutput is None.
    py.exec("printer.send_event('klippy:ready')").expect("py ready");
    check("py armed at NOW", py.eval("reactor.update_calls"), Value::List(vec![Value::Tuple(vec![Value::Tuple(vec![Value::str("timer"), Value::int(0)]), Value::float(0.0)])]));
    fake_printer.send_event("klippy:ready", &[]).expect("rust ready");
    // The armed timer fires generate_stats, which calls system_stats (False
    // part only) and so logs nothing but returns eventtime + 1.
    let ps = klippy_api::downcast::<PrinterStats>(obj.as_ref()).expect("PrinterStats");
    assert_eq!(ps.generate_stats(7.0), 8.0);

    // With debugoutput set, `klippy:ready` must not arm the timer.
    py.exec("reactor2 = FakeReactor(); printer2 = FakePrinter(reactor2, {'debugoutput': '/tmp/out'}); config2 = FakeConfig(printer2); ps2 = _s.load_config(config2); printer2.send_event('klippy:ready')").expect("py ready gated");
    check("py not armed", py.eval("reactor2.update_calls"), Value::List(vec![]));
}

/// Run `generate_stats` through `klippy:ready` collection on both sides
/// (provider objects added to the printer, ready sent) and compare the
/// return value, the log lines and the `assert_no_pause` region: first
/// with every provider returning False (no line), then with four
/// providers (False / True / True-empty / False). `PrinterStats` is built
/// directly so `system_stats` does not contribute to the log line.
#[test]
fn generate_stats_aggregation() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    // Phase 1: every provider returns False, so nothing is logged and the
    // next wakeup is still eventtime + 1.
    py.exec("_reactor, _printer, _config = _fresh(); ps = _s.PrinterStats(_config)").expect("py ps");
    py.exec("_providers = [FakeProvider((False, 'x'), _reactor), FakeProvider((False, 'y'), _reactor)]; [_printer.add_object('prov%d' % i, p) for i, p in enumerate(_providers)]; _printer.send_event('klippy:ready'); log_lines.clear()").expect("py ready");
    let py_ret = py.eval("ps.generate_stats(50.0)");
    let py_log = match py.eval("list(log_lines)") { Ok(Value::List(items)) => items, other => panic!("unexpected py log: {other:?}") };
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor.clone(), PyValue::Dict(IndexMap::new())));
    let printer: Printer = fake_printer.clone();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let ps = PrinterStats::new(&config);
    let pause_slots: Vec<Arc<Mutex<Option<bool>>>> = vec![(false, "x"), (false, "y")].iter().enumerate().map(|(i, (flag, msg))| {
        let slot = Arc::new(Mutex::new(None));
        let name = format!("prov{i}");
        fake_printer.add_object(&name, Arc::new(FakeProvider { ret: (*flag, msg.to_string()), reactor: reactor.clone(), pause_seen: slot.clone() }) as ObjectRef).expect("add prov");
        slot
    }).collect();
    fake_printer.send_event("klippy:ready", &[]).expect("rust ready");
    let rust_ret = ps.generate_stats(50.0);
    check("rust ret equals py no-log", py_ret, Value::float(rust_ret));
    let rust_pause: Vec<Value> = pause_slots.iter().map(|slot| Value::Bool((*slot.lock()).unwrap())).collect();
    check("pause held phase1", Ok(Value::List(vec![Value::Bool(true), Value::Bool(true)])), Value::List(rust_pause));
    let rust_log: Vec<Value> = captured_logs().iter().filter(|line| line.starts_with("Stats ")).map(|line| Value::str(line)).collect();
    check("no log line", Ok(Value::List(py_log)), Value::List(rust_log));

    // Phase 2: one provider returns True and the line fires with the
    // joined non-empty messages.
    py.exec("_reactor, _printer, _config = _fresh(); ps = _s.PrinterStats(_config)").expect("py ps2");
    py.exec("_providers = [FakeProvider((False, 'a'), _reactor), FakeProvider((True, 'GO'), _reactor), FakeProvider((True, ''), _reactor), FakeProvider((False, 'c'), _reactor)]; [_printer.add_object('prov%d' % i, p) for i, p in enumerate(_providers)]; _printer.send_event('klippy:ready'); log_lines.clear()").expect("py ready2");
    let py_ret = py.eval("ps.generate_stats(103.27)");
    // Each provider must have run inside the assert_no_pause region.
    check("pause held", py.eval("[p.pause_seen for p in _providers]"), Value::List(vec![Value::Bool(true), Value::Bool(true), Value::Bool(true), Value::Bool(true)]));
    let py_log = match py.eval("list(log_lines)") { Ok(Value::List(items)) => items, other => panic!("unexpected py log: {other:?}") };
    let reactor2 = Reactor::new(false);
    let fake_printer2 = Arc::new(FakePrinter::new(reactor2.clone(), PyValue::Dict(IndexMap::new())));
    let printer2: Printer = fake_printer2.clone();
    let config2: Config = Arc::new(FakeConfig { printer: printer2.clone() });
    let ps2 = PrinterStats::new(&config2);
    let specs: Vec<(bool, &str)> = vec![(false, "a"), (true, "GO"), (true, ""), (false, "c")];
    let pause_slots2: Vec<Arc<Mutex<Option<bool>>>> = specs.iter().enumerate().map(|(i, (flag, msg))| {
        let slot = Arc::new(Mutex::new(None));
        let name = format!("prov{i}");
        fake_printer2.add_object(&name, Arc::new(FakeProvider { ret: (*flag, msg.to_string()), reactor: reactor2.clone(), pause_seen: slot.clone() }) as ObjectRef).expect("add prov2");
        slot
    }).collect();
    fake_printer2.send_event("klippy:ready", &[]).expect("rust ready2");
    let rust_ret = ps2.generate_stats(103.27);
    check("rust ret equals py", py_ret, Value::float(rust_ret));
    let rust_pause2: Vec<Value> = pause_slots2.iter().map(|slot| Value::Bool((*slot.lock()).unwrap())).collect();
    check("rust pause held phase2", Ok(Value::List(vec![Value::Bool(true), Value::Bool(true), Value::Bool(true), Value::Bool(true)])), Value::List(rust_pause2));
    let rust_log: Vec<Value> = captured_logs().iter().filter(|line| line.starts_with("Stats ")).map(|line| Value::str(line)).collect();
    check("log line", Ok(Value::List(py_log)), Value::List(rust_log));
}
#[test]
fn handle_ready_skips_objects_without_stats() {
    let Some(mut py) = oracle() else { return };
    // Python side: an object without `stats` is not collected.
    py.exec("_reactor, _printer, _config = _fresh(); ps = _s.load_config(_config); _printer.add_object('nostats', FakeNoStats()); _printer.send_event('klippy:ready')").expect("py ready");
    check("py collected", py.eval("len(ps.stats_cb)"), Value::int(1));

    // Rust side: the same shape; the no-stats object must not break
    // collection, and the collected system_stats must produce the
    // usual (False, msg) aggregate without logging.
    let reactor = Reactor::new(false);
    let fake_printer = Arc::new(FakePrinter::new(reactor.clone(), PyValue::Dict(IndexMap::new())));
    let printer: Printer = fake_printer.clone();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let ps_obj = load_config(&config).expect("rust load_config");
    fake_printer.add_object("nostats", Arc::new(NoStats) as ObjectRef).expect("add nostats");
    fake_printer.send_event("klippy:ready", &[]).expect("rust ready");
    let ps = klippy_api::downcast::<PrinterStats>(ps_obj.as_ref().unwrap().as_ref()).expect("PrinterStats");
    assert_eq!(ps.generate_stats(7.0), 8.0);
}
