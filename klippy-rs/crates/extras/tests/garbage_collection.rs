// Parity tests for `extras::garbage_collection` (port of
// `klippy/extras/garbage_collection.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `GarbageCollection`
//! is built on both sides with minimal Python fakes for the printer and
//! the config wrapper, then fed identical event registrations and
//! handler invocations. Compared exactly: the registered event names,
//! the handler return values (`None`), `TypeError` for a handler called
//! with arguments, the debug log messages emitted by each handler,
//! and the module factory shape.
//!
//! The reference calls the CPython `gc` module for the actual collection:
//! `gc.collect(0); gc.collect(1); gc.collect(2); gc.freeze()` on ready and
//! `gc.unfreeze()` on disconnect. A Rust heap has no Python reference
//! collector, so the port keeps the event wiring and the debug logs and
//! records the frozen flag instead. Internal CPython gc counters such as
//! `gc.get_count()` are interpreter internals, not part of this module's
//! contract, and are not compared. On the reference interpreter
//! `hasattr(gc, 'freeze')` is always true (Python 3.8), so both handlers
//! are always registered.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use std::sync::{Arc, LazyLock};

use extras::garbage_collection::{
    GarbageCollection, DISCONNECT_EVENT, DISCONNECT_LOG, READY_EVENT, READY_LOG,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import gc
import logging
import extras.garbage_collection as garbage_collection

LOG_CAPTURE = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        LOG_CAPTURE.append(record.getMessage())
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.DEBUG)

class FakePrinter:
    def __init__(self):
        self._handlers = []
        self.registered = []
    def register_event_handler(self, event, callback):
        self._handlers.append((event, callback))
        self.registered.append(event)
    def send_event(self, event, params=()):
        returns = []
        for e, cb in self._handlers:
            if e == event:
                returns.append(cb(*params))
        return returns

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

def py_registered_names():
    return list(p.registered)

def py_logs():
    return list(LOG_CAPTURE)

def py_clear_logs():
    LOG_CAPTURE.clear()

def py_send(event, args=()):
    return p.send_event(event, args)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinterShared {
    handlers: Vec<(String, EventCallback)>,
    registered: Vec<String>,
}

#[derive(Clone)]
struct FakePrinter {
    shared: Arc<Mutex<FakePrinterShared>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter {
            shared: Arc::new(Mutex::new(FakePrinterShared {
                handlers: Vec::new(),
                registered: Vec::new(),
            })),
        }
    }
    fn registered_names(&self) -> Vec<String> {
        self.shared.lock().registered.clone()
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        _name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
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
        let mut guard = self.shared.lock();
        guard.handlers.push((event.to_string(), callback));
        guard.registered.push(event.to_string());
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut returns = Vec::new();
        let mut indices = Vec::new();
        {
            let guard = self.shared.lock();
            for (i, (e, _)) in guard.handlers.iter().enumerate() {
                if e == event {
                    indices.push(i);
                }
            }
        }
        for idx in indices {
            let mut guard = self.shared.lock();
            let cb = &mut guard.handlers[idx].1;
            let ret = cb(params)?;
            returns.push(ret);
        }
        Ok(returns)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "garbage_collection".to_string()
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
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
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
        panic!("getsection is not used by garbage_collection")
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
// Rust log capture (process-wide sink, installed once).
// ---------------------------------------------------------------------------

static RUST_LOGS: LazyLock<Arc<Mutex<Vec<String>>>> = LazyLock::new(|| {
    let logs: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let sink_logs = logs.clone();
    let _ = klippy_api::log::set_sink(Box::new(move |rec| {
        sink_logs.lock().push(rec.message.to_string());
    }));
    logs
});

static LOG_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

fn rust_logs_clear() {
    let mut guard = RUST_LOGS.lock();
    guard.clear();
}

fn rust_logs_snapshot() -> Vec<String> {
    RUST_LOGS.lock().clone()
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("garbage_collection fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping garbage_collection oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn fresh_pair(py: &mut pyoracle::Oracle) -> (GarbageCollection, Config, FakePrinter) {
    py.exec("py_clear_logs(); p = FakePrinter(); c = FakeConfig(p); gc_obj = garbage_collection.load_config(c)")
        .expect("py load_config");
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc,
    });
    let rust = GarbageCollection::new(&config);
    (rust, config, printer)
}

fn check_value(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "garbage_collection".to_string(),
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

fn check_none(label: &str, expected: pyoracle::OracleResult, actual: KResult<Vec<PyValue>>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    let actual_value = Value::List(
        actual
            .iter()
            .map(|v| match v {
                PyValue::None => Value::Null,
                other => Value::Str(format!("{other:?}")),
            })
            .collect(),
    );
    check_value(label, Ok(expected), actual_value);
}

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: KResult<Vec<PyValue>>) {
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

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn feature_check_and_event_names_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("hasattr(gc, 'freeze') and hasattr(gc, 'unfreeze')")
        .expect("eval feature check");
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer);
    let cfg: Config = Arc::new(FakeConfig { printer: printer_arc });
    let runtime = GarbageCollection::new(&cfg);
    check_value("feature check is true", Ok(expected), Value::Bool(runtime.is_registered()));
    let expected = py.eval("garbage_collection.__name__").expect("eval module name");
    check_value(
        "module name",
        Ok(expected),
        Value::Str("extras.garbage_collection".to_string()),
    );
    assert_eq!(READY_EVENT, "klippy:ready");
    assert_eq!(DISCONNECT_EVENT, "klippy:disconnect");
    assert_eq!(READY_LOG, "Running full garbage collection and freezing");
    assert_eq!(DISCONNECT_LOG, "Unfreezing garbage collection");
}

#[test]
fn load_config_registers_both_handlers_in_order() {
    let Some(mut py) = oracle() else { return };
    let (_rust, _config, printer) = fresh_pair(&mut py);
    let expected = py.eval("py_registered_names()").expect("eval registered");
    let actual: Value = Value::List(
        printer
            .registered_names()
            .iter()
            .map(|s| Value::Str(s.clone()))
            .collect(),
    );
    check_value("registered event names", Ok(expected), actual);
}

#[test]
fn ready_handler_returns_none_and_logs() {
    let Some(mut py) = oracle() else { return };
    let (rust, _config, printer) = fresh_pair(&mut py);
    let _guard = LOG_LOCK.lock();
    rust_logs_clear();
    let expected = py.eval("py_send('klippy:ready')").expect("eval ready send");
    let printer_arc: Printer = Arc::new(printer.clone());
    let actual = printer_arc.send_event(READY_EVENT, &[]);
    check_none("ready handler returns [None]", Ok(expected), actual);
    let expected_logs = py.eval("py_logs()").expect("eval py logs");
    check_value(
        "ready log message",
        Ok(expected_logs),
        Value::List(
            rust_logs_snapshot()
                .iter()
                .map(|s| Value::Str(s.clone()))
                .collect(),
        ),
    );
    assert!(rust.is_frozen(), "frozen after ready");
}

#[test]
fn disconnect_handler_returns_none_and_logs() {
    let Some(mut py) = oracle() else { return };
    let (rust, _config, printer) = fresh_pair(&mut py);
    let _guard = LOG_LOCK.lock();
    let printer_arc: Printer = Arc::new(printer.clone());
    let expected_ready = py.eval("py_send('klippy:ready')").expect("eval ready");
    let actual_ready = printer_arc.send_event(READY_EVENT, &[]);
    check_none("ready first", Ok(expected_ready), actual_ready);
    rust_logs_clear();
    py.exec("py_clear_logs()").expect("clear py logs");
    let expected = py.eval("py_send('klippy:disconnect')").expect("eval disconnect");
    let actual = printer_arc.send_event(DISCONNECT_EVENT, &[]);
    check_none("disconnect handler returns [None]", Ok(expected), actual);
    let expected_logs = py.eval("py_logs()").expect("eval py logs");
    check_value(
        "disconnect log message",
        Ok(expected_logs),
        Value::List(
            rust_logs_snapshot()
                .iter()
                .map(|s| Value::Str(s.clone()))
                .collect(),
        ),
    );
    assert!(!rust.is_frozen(), "unfrozen after disconnect");
}

#[test]
fn handler_with_args_raises_type_error() {
    let Some(mut py) = oracle() else { return };
    let (_rust, _config, printer) = fresh_pair(&mut py);
    let printer_arc: Printer = Arc::new(printer.clone());
    let expected = py.eval("py_send('klippy:ready', (1,))");
    let actual = printer_arc.send_event(
        READY_EVENT,
        &[EventArg::Value(PyValue::Int(PyInt::from_i64(1)))],
    );
    check_exc("ready with args TypeError", expected, actual);
    let expected = py.eval("py_send('klippy:disconnect', (1, 2))");
    let actual = printer_arc.send_event(
        DISCONNECT_EVENT,
        &[
            EventArg::Value(PyValue::Int(PyInt::from_i64(1))),
            EventArg::Value(PyValue::Int(PyInt::from_i64(2))),
        ],
    );
    check_exc("disconnect with args TypeError", expected, actual);
}

#[test]
fn direct_handler_methods_match() {
    let Some(mut py) = oracle() else { return };
    let (rust, _config, _printer) = fresh_pair(&mut py);
    let _guard = LOG_LOCK.lock();
    let expected = py.eval("gc_obj._handle_ready()").expect("py _handle_ready");
    check_value("direct _handle_ready returns None", Ok(expected), Value::Null);
    assert_eq!(rust.handle_ready().expect("rust handle_ready"), PyValue::None);
    let expected = py.eval("gc_obj._handle_disconnect()").expect("py _handle_disconnect");
    check_value("direct _handle_disconnect returns None", Ok(expected), Value::Null);
    assert_eq!(
        rust.handle_disconnect().expect("rust handle_disconnect"),
        PyValue::None
    );
}

#[test]
fn repeated_ready_disconnect_is_idempotent() {
    let Some(mut py) = oracle() else { return };
    let (rust, _config, printer) = fresh_pair(&mut py);
    let _guard = LOG_LOCK.lock();
    let printer_arc: Printer = Arc::new(printer.clone());
    rust_logs_clear();
    py.exec("py_clear_logs()").expect("clear");
    for event in [READY_EVENT, READY_EVENT, DISCONNECT_EVENT, DISCONNECT_EVENT] {
        let expected = py.eval(&format!("py_send('{event}')")).expect("eval send");
        let actual = printer_arc.send_event(event, &[]);
        check_none(&format!("send {event} returns [None]"), Ok(expected), actual);
    }
    assert!(!rust.is_frozen());
    let expected_logs = py.eval("py_logs()").expect("eval logs");
    check_value(
        "repeated log sequence",
        Ok(expected_logs),
        Value::List(
            rust_logs_snapshot()
                .iter()
                .map(|s| Value::Str(s.clone()))
                .collect(),
        ),
    );
}

#[test]
fn module_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::garbage_collection::module_factory();
    let load_config = factory.load_config.expect("exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "no add_early_printer_objects"
    );
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");

    let _guard = LOG_LOCK.lock();
    rust_logs_clear();
    py.exec("py_clear_logs(); p = FakePrinter(); c = FakeConfig(p); gc_obj = garbage_collection.load_config(c)")
        .expect("py factory load_config");
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig {
        printer: printer_arc.clone(),
    });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory returns an object");
    let rust: Arc<GarbageCollection> = klippy_api::downcast_arc(&obj).expect("downcast");
    assert!(rust.is_registered());
    let expected = py.eval("py_registered_names()").expect("eval registered");
    let actual: Value = Value::List(
        printer
            .registered_names()
            .iter()
            .map(|s| Value::Str(s.clone()))
            .collect(),
    );
    check_value("factory registered names", Ok(expected), actual);
    let expected = py.eval("py_send('klippy:ready')").expect("eval ready");
    let actual = printer_arc.send_event(READY_EVENT, &[]);
    check_none("factory ready returns [None]", Ok(expected), actual);
    assert!(rust.is_frozen(), "frozen after factory ready");
    let expected = py.eval("py_send('klippy:disconnect')").expect("eval disconnect");
    let actual = printer_arc.send_event(DISCONNECT_EVENT, &[]);
    check_none("factory disconnect returns [None]", Ok(expected), actual);
    assert!(!rust.is_frozen(), "unfrozen after factory disconnect");
}
