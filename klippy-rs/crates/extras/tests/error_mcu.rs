//! Parity tests for `extras::error_mcu` (port of `klippy/extras/error_mcu.py`).
//!
//! Expected values come from the live reference through `pyoracle` (CPython
//! 3.8.2 + frozen `klippy/`): a Python `PrinterMCUError` is built with
//! minimal fakes for the printer, config and MCU, fed identical shutdown /
//! notify events, and the rewritten messages (`printer.updates`), the
//! exception class/message and the log lines (GC stats line, failed
//! mcu_version lookup) are compared bit-for-bit with the Rust port driven
//! through fakes of the contract traits.

use extras::error_mcu::{error_hint, PrinterMCUErrorApi, PrinterMcuError};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    EventCallback, LookupDefault, PrinterApi, EV_KLIPPY_ANALYZE_SHUTDOWN,
    EV_KLIPPY_NOTIFY_MCU_ERROR,
};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Exception, Record, RecordKind, SideEffects, Value};
use std::any::Any;
use std::sync::Arc;

const SETUP: &str = r#"
import extras.error_mcu as error_mcu
import logging as _logging

class FakeReactor:
    def get_gc_stats(self):
        return (0., 0., 0.)

class FakeMcu:
    def __init__(self, status, raises=False):
        self._status = status
        self._raises = raises
    def get_status(self, eventtime=None):
        if self._raises:
            raise RuntimeError('no status')
        return dict(self._status)

class NoStatusMcu:
    pass

class FakePrinter:
    def __init__(self, start_args, objects):
        self.start_args = dict(start_args)
        self.reactor = FakeReactor()
        self.updates = []
        self.handlers = {}
        self.objects = {}
        for name, mcu in objects:
            self.objects[name] = mcu
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def update_error_msg(self, old, new):
        self.updates.append((old, new))
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        prefix = module + ' '
        objs = [(n, self.objects[n]) for n in self.objects if n.startswith(prefix)]
        if module in self.objects:
            return [(module, self.objects[module])] + objs
        return objs

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
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("error_mcu fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping error_mcu oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_str(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn py_dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    let mut m: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        m.insert(PyValue::Str(k.to_string()), v);
    }
    PyValue::Dict(m)
}

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

struct PrinterShared {
    start_args: PyValue,
    updates: Vec<(String, String)>,
    handlers: Vec<(String, EventCallback)>,
    handler_names: Vec<String>,
    objects: Vec<(String, ObjectRef)>,
}

struct FakePrinter {
    reactor: Reactor,
    shared: Arc<Mutex<PrinterShared>>,
}

impl FakePrinter {
    fn new(start_args: PyValue, objects: Vec<(String, ObjectRef)>) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            shared: Arc::new(Mutex::new(PrinterShared {
                start_args,
                updates: Vec::new(),
                handlers: Vec::new(),
                handler_names: Vec::new(),
                objects,
            })),
        })
    }

    fn updates(&self) -> Vec<(String, String)> {
        self.shared.lock().updates.clone()
    }

    fn handler_names(&self) -> Vec<String> {
        self.shared.lock().handler_names.clone()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.shared.lock().start_args.clone()
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
    fn update_error_msg(&self, old_msg: &str, new_msg: &str) {
        self.shared.lock().updates.push((old_msg.to_string(), new_msg.to_string()));
    }
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut s = self.shared.lock();
        if s.objects.iter().any(|(n, _)| n == name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        s.objects.push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let s = self.shared.lock();
        for (n, o) in s.objects.iter() {
            if n == name {
                return Ok(Some(o.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let s = self.shared.lock();
        match module {
            None => s.objects.clone(),
            Some(m) => {
                let prefix = format!("{m} ");
                let prefixed: Vec<(String, ObjectRef)> =
                    s.objects.iter().filter(|(n, _)| n.starts_with(&prefix)).cloned().collect();
                if let Some(exact) = s.objects.iter().find(|(n, _)| n == m) {
                    let mut out = vec![exact.clone()];
                    out.extend(prefixed);
                    out
                } else {
                    prefixed
                }
            }
        }
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
        let mut s = self.shared.lock();
        s.handler_names.push(event.to_string());
        s.handlers.push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut taken: Vec<EventCallback> = Vec::new();
        {
            let mut s = self.shared.lock();
            let mut remaining: Vec<(String, EventCallback)> = Vec::new();
            for (name, cb) in std::mem::take(&mut s.handlers).drain(..) {
                if name == event {
                    taken.push(cb);
                } else {
                    remaining.push((name, cb));
                }
            }
            s.handlers = remaining;
        }
        let mut results = Vec::new();
        for cb in taken.iter_mut() {
            results.push((*cb)(params)?);
        }
        // Return callbacks for later use (order preserved among matches only;
        // tests dispatch once per event).
        {
            let mut s = self.shared.lock();
            for cb in taken {
                s.handlers.push((event.to_string(), cb));
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: klippy_api::printer::Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> klippy_api::printer::Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "error_mcu".to_string()
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
        Arc::new(FakeConfig { printer: self.printer.clone() })
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

fn make_config(printer: Arc<FakePrinter>) -> Config {
    Arc::new(FakeConfig { printer })
}

struct FakeMcu {
    status: Option<PyValue>,
}

impl PrinterObject for FakeMcu {
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        self.status.clone()
    }
}

fn mcu_object(status: Option<PyValue>) -> ObjectRef {
    Arc::new(FakeMcu { status })
}

fn mcu_status_dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    py_dict(pairs)
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "error_mcu".to_string(),
        scenario: label.to_string(),
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

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = record_for(label);
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_exc(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<()>,
) {
    let (exp_class, exp_msg) = match expected {
        Ok(v) => panic!("{label}: oracle did not raise, returned {v:?}"),
        Err(Exception { class_name, message }) => (class_name, message),
    };
    let act = match actual {
        Ok(()) => panic!("{label}: rust did not raise"),
        Err(e) => (e.class_name().to_string(), e.message().to_string()),
    };
    check(
        label,
        Ok(Value::Tuple(vec![Value::str(&exp_class), Value::str(&exp_msg)])),
        Value::Tuple(vec![Value::str(&act.0), Value::str(&act.1)]),
    );
}

fn updates_value(updates: Vec<(String, String)>) -> Value {
    Value::List(
        updates
            .iter()
            .map(|(a, b)| Value::Tuple(vec![Value::str(a), Value::str(b)]))
            .collect(),
    )
}

fn oracle_updates(py: &mut pyoracle::Oracle) -> pyoracle::OracleResult {
    py.eval("list(printer.updates)")
}

#[allow(clippy::type_complexity)]
fn log_capture() -> Arc<Mutex<Vec<(String, String)>>> {
    use std::sync::LazyLock;
    static CAPTURE: LazyLock<Arc<Mutex<Vec<(String, String)>>>> =
        LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
    let captured = CAPTURE.clone();
    let sink_capture = captured.clone();
    log::set_sink(Box::new(move |rec: &log::Record<'_>| {
        sink_capture
            .lock()
            .push((rec.level.name().to_string(), rec.message.to_string()));
    }));
    captured
}

fn rust_log_messages(captured: &Mutex<Vec<(String, String)>>) -> Value {
    Value::List(
        captured
            .lock()
            .iter()
            .map(|(_, m)| Value::str(m))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn error_hint_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        "Timer too close to print",
        "Timer too close",
        "Missed scheduling of next step",
        "Missed scheduling of next ",
        "Missed scheduling of nex",
        "ADC out of range: 1.2",
        "Rescheduled timer in the past (x)",
        "Stepper too far in past (y)",
        "Command request: M112",
        "Something else",
        "",
        " ADC out of range",
    ];
    for (i, msg) in cases.iter().enumerate() {
        let label = format!("error_hint {i}");
        // Escape single quotes/backslashes for the Python literal.
        let lit = msg.replace('\\', "\\\\").replace('\'', "\\'");
        check(
            &label,
            py.eval(&format!("error_mcu.error_hint('{lit}')")),
            Value::str(error_hint(msg)),
        );
    }
}

#[test]
fn factory_registers_handlers() {
    let Some(mut py) = oracle() else { return };
    // Python: PrinterMCUError registers analyze first, then notify.
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("py construct");
    let expected: Vec<String> = match py.eval("list(printer.handlers.keys())") {
        Ok(Value::List(items)) => items
            .into_iter()
            .map(|v| match v {
                Value::Str(s) => s,
                other => panic!("unexpected handler name: {other:?}"),
            })
            .collect(),
        other => panic!("unexpected py handlers: {other:?}"),
    };
    let printer = FakePrinter::new(py_dict(vec![("software_version", py_str("v1"))]), Vec::new());
    let cfg = make_config(printer.clone());
    let _em = PrinterMcuError::new(&cfg);
    check(
        "registered handlers",
        Ok(Value::List(expected.iter().map(|s| Value::str(s)).collect())),
        Value::List(
            printer
                .handler_names()
                .iter()
                .map(|s| Value::str(s))
                .collect(),
        ),
    );
    assert_eq!(printer.handler_names(), vec![
        EV_KLIPPY_ANALYZE_SHUTDOWN.to_string(),
        EV_KLIPPY_NOTIFY_MCU_ERROR.to_string(),
    ]);
}

#[test]
fn handlers_match_python() {
    let Some(mut py) = oracle() else { return };
    let captured = log_capture();
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("py initial setup");

    // --- analyze shutdown, MCU shutdown: event types and hints ---
    let analyze_cases: Vec<(&str, &str, &str, &str)> = vec![
        ("timer_close", "Timer too close (x)", "is_shutdown", "mcu"),
        ("timer_close_prev", "Timer too close (x)", "shutdown", "mcu"),
        ("missed", "Missed scheduling of next step", "is_shutdown", "mcu"),
        ("missed_short", "Missed scheduling of nex", "is_shutdown", "mcu"),
        ("adc", "ADC out of range: 1.2", "shutdown", "mcu"),
        ("rescheduled", "Rescheduled timer in the past (x)", "shutdown", "mcu"),
        ("stepper", "Stepper too far in past (y)", "is_shutdown", "mcu"),
        ("cmdreq", "Command request: M112", "shutdown", "mcu"),
        ("other_reason", "Some other reason", "is_shutdown", "mcu aux"),
    ];
    for (label, reason, event_type, mcu_name) in analyze_cases {
        py.exec("printer.updates.clear()").expect("clear");
        py.exec("log_lines.clear()").expect("clear py log");
        captured.lock().clear();
        // Fresh run with the real inputs.
        // Reset with the real inputs (repr-safe literals).
        let r = reason.replace('\\', "\\\\").replace('\'', "\\'");
        let e = event_type.replace('\\', "\\\\").replace('\'', "\\'");
        let m = mcu_name.replace('\\', "\\\\").replace('\'', "\\'");
        py.exec(&format!(
            "em._handle_analyze_shutdown('MCU shutdown', {{'mcu': '{m}', 'reason': '{r}', 'event_type': '{e}'}})"
        ))
        .expect("py analyze");
        let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
        let cfg = make_config(printer.clone());
        let em = PrinterMcuError::new(&cfg);
        em.handle_analyze_shutdown(
            &py_str("MCU shutdown"),
            &py_dict(vec![
                ("mcu", py_str(mcu_name)),
                ("reason", py_str(reason)),
                ("event_type", py_str(event_type)),
            ]),
        )
        .expect("rust analyze");
        check(
            &format!("analyze updates {label}"),
            oracle_updates(&mut py),
            updates_value(printer.updates()),
        );
        check(
            &format!("analyze logs {label}"),
            py.eval("list(log_lines)"),
            rust_log_messages(&captured),
        );
    }

    // --- analyze shutdown, non-MCU message ---
    py.exec("printer.updates.clear()").expect("clear");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "em._handle_analyze_shutdown('Lost communication with MCU', {})",
    )
    .expect("py analyze other");
    let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.handle_analyze_shutdown(&py_str("Lost communication with MCU"), &py_dict(Vec::new()))
        .expect("rust analyze other");
    check(
        "analyze other updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "analyze other logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- clarify callbacks: Some / None filtering and join ---
    // The lookup key is the exact `reason` string (`error_mcu.py:81`),
    // so the registered key must match the triggered reason exactly.
    py.exec(
        "def _c1(msg, details):\n    return 'help one'\n\
         def _c2(msg, details):\n    return None\n\
         def _c3(msg, details):\n    return 'help two'\n\
         em.add_clarify('ADC out of range: 1.2', _c1)\n\
         em.add_clarify('ADC out of range: 1.2', _c2)\n\
         em.add_clarify('ADC out of range: 1.2', _c3)",
    )
    .expect("py clarify");
    py.exec("printer.updates.clear()").expect("clear");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "em._handle_analyze_shutdown('MCU shutdown', {'mcu': 'mcu', 'reason': 'ADC out of range: 1.2', 'event_type': 'shutdown'})",
    )
    .expect("py analyze clarify");
    let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.add_clarify(
        "ADC out of range: 1.2",
        Arc::new(|_, _| Ok(Some("help one".to_string()))),
    );
    em.add_clarify("ADC out of range: 1.2", Arc::new(|_, _| Ok(None)));
    em.add_clarify(
        "ADC out of range: 1.2",
        Arc::new(|_, _| Ok(Some("help two".to_string()))),
    );
    em.handle_analyze_shutdown(
        &py_str("MCU shutdown"),
        &py_dict(vec![
            ("mcu", py_str("mcu")),
            ("reason", py_str("ADC out of range: 1.2")),
            ("event_type", py_str("shutdown")),
        ]),
    )
    .expect("rust analyze clarify");
    check(
        "clarify updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "clarify logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- protocol error: mixed versions, missing version, raising get_status ---
    py.exec(
        "printer = FakePrinter({'software_version': 'v0.12.0-1'}, [\
        ('mcu', FakeMcu({'mcu_version': 'v0.12.0-1'})), \
        ('mcu extboard', FakeMcu({'mcu_version': 'v0.11.0-2'})), \
        ('mcu aux', FakeMcu({})), \
        ('mcu rpi', FakeMcu({}, True)), \
        ('mcu dead', NoStatusMcu())])\n\
        em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("py protocol setup");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "em._handle_notify_mcu_error('Protocol error', {'error': 'Mismatched protocol version'})",
    )
    .expect("py protocol");
    let printer = FakePrinter::new(
        py_dict(vec![("software_version", py_str("v0.12.0-1"))]),
        vec![
            ("mcu".to_string(), mcu_object(Some(mcu_status_dict(vec![("mcu_version", py_str("v0.12.0-1"))])))),
            ("mcu extboard".to_string(), mcu_object(Some(mcu_status_dict(vec![("mcu_version", py_str("v0.11.0-2"))])))),
            ("mcu aux".to_string(), mcu_object(Some(mcu_status_dict(Vec::new())))),
            ("mcu rpi".to_string(), mcu_object(None)),
            ("mcu dead".to_string(), mcu_object(None)),
        ],
    );
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.handle_notify_mcu_error(
        &py_str("Protocol error"),
        &py_dict(vec![("error", py_str("Mismatched protocol version"))]),
    )
    .expect("rust protocol");
    check(
        "protocol updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "protocol logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- protocol error: no MCUs ---
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("py protocol empty setup");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "em._handle_notify_mcu_error('Protocol error', {'error': 'Details here'})",
    )
    .expect("py protocol empty");
    let printer = FakePrinter::new(py_dict(vec![("software_version", py_str("v1"))]), Vec::new());
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.handle_notify_mcu_error(
        &py_str("Protocol error"),
        &py_dict(vec![("error", py_str("Details here"))]),
    )
    .expect("rust protocol empty");
    check(
        "protocol empty updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "protocol empty logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- connect error ---
    py.exec("printer.updates.clear()").expect("clear");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "em._handle_notify_mcu_error('MCU error during connect', {'error': 'Timeout with MCU'})",
    )
    .expect("py connect error");
    let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.handle_notify_mcu_error(
        &py_str("MCU error during connect"),
        &py_dict(vec![("error", py_str("Timeout with MCU"))]),
    )
    .expect("rust connect error");
    check(
        "connect updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "connect logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- notify unknown message: no-op even with malformed details ---
    py.exec("printer.updates.clear()").expect("clear");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec("em._handle_notify_mcu_error('Something else', {})").expect("py unknown");
    let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
    let cfg = make_config(printer.clone());
    let em = PrinterMcuError::new(&cfg);
    em.handle_notify_mcu_error(&py_str("Something else"), &py_dict(Vec::new()))
        .expect("rust unknown");
    check(
        "unknown updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "unknown logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );

    // --- dispatch through registered event handlers (send_event) ---
    py.exec("printer.updates.clear()").expect("clear");
    py.exec("log_lines.clear()").expect("clear py log");
    captured.lock().clear();
    py.exec(
        "for cb in printer.handlers['klippy:notify_mcu_error']: cb('MCU error during connect', {'error': 'via event'})",
    )
    .expect("py dispatch");
    let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
    let cfg = make_config(printer.clone());
    let _em = PrinterMcuError::new(&cfg);
    printer
        .send_event(
            EV_KLIPPY_NOTIFY_MCU_ERROR,
            &[
                EventArg::Value(py_str("MCU error during connect")),
                EventArg::Value(py_dict(vec![("error", py_str("via event"))])),
            ],
        )
        .expect("rust dispatch");
    check(
        "dispatch updates",
        oracle_updates(&mut py),
        updates_value(printer.updates()),
    );
    check(
        "dispatch logs",
        py.eval("list(log_lines)"),
        rust_log_messages(&captured),
    );
}

#[test]
fn exception_paths_match_python() {
    let Some(mut py) = oracle() else { return };
    // Missing details key in MCU shutdown.
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("setup");
    check_exc(
        "shutdown missing mcu",
        py.eval("em._handle_analyze_shutdown('MCU shutdown', {'reason': 'x', 'event_type': 'shutdown'})"),
        {
            let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
            let cfg = make_config(printer);
            let em = PrinterMcuError::new(&cfg);
            em.handle_analyze_shutdown(
                &py_str("MCU shutdown"),
                &py_dict(vec![("reason", py_str("x")), ("event_type", py_str("shutdown"))]),
            )
        },
    );
    // Missing software_version.
    py.exec(
        "printer = FakePrinter({}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("setup2");
    check_exc(
        "protocol missing software_version",
        py.eval("em._handle_notify_mcu_error('Protocol error', {'error': 'x'})"),
        {
            let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
            let cfg = make_config(printer);
            let em = PrinterMcuError::new(&cfg);
            em.handle_notify_mcu_error(
                &py_str("Protocol error"),
                &py_dict(vec![("error", py_str("x"))]),
            )
        },
    );
    // Missing error key in connect path.
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))",
    )
    .expect("setup3");
    check_exc(
        "connect missing error",
        py.eval("em._handle_notify_mcu_error('MCU error during connect', {})"),
        {
            let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
            let cfg = make_config(printer);
            let em = PrinterMcuError::new(&cfg);
            em.handle_notify_mcu_error(&py_str("MCU error during connect"), &py_dict(Vec::new()))
        },
    );
    // Clarify callback raising propagates.
    py.exec(
        "printer = FakePrinter({'software_version': 'v1'}, [])\n\
         em = error_mcu.PrinterMCUError(FakeConfig(printer))\n\
         def _boom(msg, details): raise RuntimeError('boom')\n\
         em.add_clarify('x reason', _boom)",
    )
    .expect("setup4");
    check_exc(
        "clarify raises",
        py.eval("em._handle_analyze_shutdown('MCU shutdown', {'mcu': 'mcu', 'reason': 'x reason', 'event_type': 'shutdown'})"),
        {
            let printer = FakePrinter::new(py_dict(Vec::new()), Vec::new());
            let cfg = make_config(printer);
            let em = PrinterMcuError::new(&cfg);
            em.add_clarify(
                "x reason",
                Arc::new(|_, _| {
                    Err(KlippyError::Internal {
                        class_name: "RuntimeError".to_string(),
                        message: "boom".to_string(),
                    })
                }),
            );
            em.handle_analyze_shutdown(
                &py_str("MCU shutdown"),
                &py_dict(vec![
                    ("mcu", py_str("mcu")),
                    ("reason", py_str("x reason")),
                    ("event_type", py_str("shutdown")),
                ]),
            )
        },
    );
}

#[test]
fn module_factory_load_config() {
    let factory = extras::error_mcu::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let printer = FakePrinter::new(
        py_dict(vec![("software_version", py_str("v1"))]),
        Vec::new(),
    );
    let cfg = make_config(printer.clone());
    let load = factory.load_config.expect("load_config present");
    let obj = load(&cfg).expect("load_config ok").expect("object");
    assert!(obj.get_status(0.0).is_none());
    assert_eq!(printer.handler_names().len(), 2);
    // The `add_clarify` narrow interface is declared for other extras
    // (contract v2: `dyn PrinterMCUErrorApi` under the Python name).
    let api = klippy_api::object::interface::<dyn PrinterMCUErrorApi>(&obj)
        .expect("PrinterMCUError exposes PrinterMCUErrorApi");
    api.add_clarify(
        "check key",
        Arc::new(|_, _| Ok(Some("from api".to_string()))),
    );
}
