//! Parity tests for `extras::extruder_stepper` (port of
//! `klippy/extras/extruder_stepper.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterExtruderStepper`
//! is built on both sides with minimal Python fakes for the printer, the
//! config wrapper and the `kinematics.extruder.ExtruderStepper` (which owns
//! the stepper hardware, motion-queue sync and pressure advance), then fed
//! identical `new` / `handle_connect` / `get_status` / `find_past_position`
//! calls and the resulting values and errors are compared exactly (class and
//! message, floats by bits).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::extruder_stepper::{module_factory, PrinterExtruderStepper};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::interface;
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT,
};
use klippy_api::toolhead::{ExtruderStepper, ExtruderStepperApi};
use klippy_api::{EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import sys
import types

class CommandError(Exception):
    pass

class FakeExtruderStepper:
    def __init__(self, config):
        self.config = config
        self.sync_calls = []
        self.motion_queue = None
        self.sync_fail = None
    def sync_to_extruder(self, name):
        self.sync_calls.append(name)
        if self.sync_fail is not None:
            raise CommandError(self.sync_fail)
        self.motion_queue = name
    def find_past_position(self, print_time):
        return print_time * 2.0 + 1.0
    def get_status(self, eventtime):
        return {'pressure_advance': 0.03, 'smooth_time': 0.04,
                'motion_queue': self.motion_queue}

fake_extruder = types.ModuleType('kinematics.extruder')
fake_extruder.ExtruderStepper = FakeExtruderStepper
fake_kin = types.ModuleType('kinematics')
fake_kin.extruder = fake_extruder
sys.modules['kinematics'] = fake_kin
sys.modules['kinematics.extruder'] = fake_extruder

import extras.extruder_stepper as extruder_stepper_mod

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.handlers = {}
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options):
        self._printer = printer
        self._options = options
    def get_printer(self):
        return self._printer
    def get(self, option):
        if option not in self._options:
            raise configparser.Error(
                "No option '%s' in section: '%s'"
                % (option, 'extruder_stepper my_extra_stepper'))
        return self._options[option]
"#;

const SECTION_NAME: &str = "extruder_stepper my_extra_stepper";

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type HandlerLog = Arc<GCell<Vec<(String, EventCallback)>>>;

#[derive(Clone)]
struct FakePrinter {
    handlers: HandlerLog,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            handlers: Arc::new(GCell::new(Vec::new())),
        }
    }
    fn handler_count(&self, event: &str) -> usize {
        self.handlers
            .borrow()
            .iter()
            .filter(|(name, _)| name == event)
            .count()
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
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
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
        self.handlers
            .borrow_mut()
            .push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        let mut results = Vec::new();
        let mut handlers = self.handlers.borrow_mut();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                results.push(cb(params)?);
            }
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    options: GCell<IndexMap<String, String>>,
}

impl FakeConfig {
    fn new(printer: Printer, extruder: Option<&str>) -> FakeConfig {
        let mut options = IndexMap::new();
        if let Some(name) = extruder {
            options.insert("extruder".to_string(), name.to_string());
        }
        FakeConfig {
            printer,
            options: GCell::new(options),
        }
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        SECTION_NAME.to_string()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        let options = self.options.borrow();
        if let Some(value) = options.get(option) {
            return Ok(Some(value.clone()));
        }
        Err(KlippyError::config(format!(
            "No option '{option}' in section: '{SECTION_NAME}'"
        )))
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
        panic!("getsection is not used by extruder_stepper")
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

struct FakeExtruderStepper {
    sync_calls: GCell<Vec<Option<String>>>,
    motion_queue: GCell<Option<String>>,
    sync_fail: GCell<Option<String>>,
}

impl FakeExtruderStepper {
    fn new() -> FakeExtruderStepper {
        FakeExtruderStepper {
            sync_calls: GCell::new(Vec::new()),
            motion_queue: GCell::new(None),
            sync_fail: GCell::new(None),
        }
    }
}

impl ExtruderStepperApi for FakeExtruderStepper {
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("pressure_advance".to_string()),
            PyValue::Float(0.03),
        );
        status.insert(
            PyValue::Str("smooth_time".to_string()),
            PyValue::Float(0.04),
        );
        status.insert(
            PyValue::Str("motion_queue".to_string()),
            match &*self.motion_queue.borrow() {
                Some(name) => PyValue::Str(name.clone()),
                None => PyValue::None,
            },
        );
        PyValue::Dict(status)
    }
    fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        Ok(print_time * 2.0 + 1.0)
    }
    fn sync_to_extruder(&self, extruder_name: Option<&str>) -> KResult<()> {
        self.sync_calls
            .borrow_mut()
            .push(extruder_name.map(str::to_string));
        if let Some(message) = self.sync_fail.borrow().clone() {
            return Err(KlippyError::command(message));
        }
        *self.motion_queue.borrow_mut() = extruder_name.map(str::to_string);
        Ok(())
    }
    fn stepper(&self) -> klippy_api::stepper::Stepper {
        panic!("stepper is not used by extruder_stepper")
    }
    fn cmd_set_pressure_advance(&self, _gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("extruder_stepper fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping extruder_stepper oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let digits = n.0.to_string();
            match digits.parse::<i64>() {
                Ok(n) => Value::Int(n),
                Err(_) => Value::BigInt(digits),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(xs) => Value::List(xs.iter().map(to_value).collect()),
        PyValue::Tuple(xs) => Value::Tuple(xs.iter().map(to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (to_value(k), to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "Host".to_string(),
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
        unit: "extruder_stepper".to_string(),
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

fn check_value(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, expected, actual);
}

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned Ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

// ---------------------------------------------------------------------------
// Rig helpers.
// ---------------------------------------------------------------------------

struct Rig {
    printer: FakePrinter,
    stepper: Arc<FakeExtruderStepper>,
    result: KResult<PrinterExtruderStepper>,
}

fn make_rust(extruder: Option<&str>) -> Rig {
    let printer = FakePrinter::new();
    let printer_handle: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig::new(printer_handle, extruder));
    let stepper = Arc::new(FakeExtruderStepper::new());
    let handle: ExtruderStepper = stepper.clone();
    let result = PrinterExtruderStepper::new(&config, handle);
    Rig {
        printer,
        stepper,
        result,
    }
}

fn make_py(py: &mut pyoracle::Oracle, extruder: Option<&str>) -> pyoracle::OracleResult {
    let options = match extruder {
        Some(name) => format!("{{'extruder': '{name}'}}"),
        None => "{}".to_string(),
    };
    py.exec(&format!(
        "pp = FakePrinter(); cc = FakeConfig(pp, {options}); \
         obj = extruder_stepper_mod.load_config_prefix(cc)"
    ))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_shape_matches_python() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    assert!(factory.load_config.is_none(), "no load_config in Python");
    assert!(
        factory.load_config_prefix.is_some(),
        "load_config_prefix is the Python entry point"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "no add_early_printer_objects in Python"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "no add_printer_objects in Python"
    );
    let expected = py.eval("hasattr(extruder_stepper_mod, 'load_config_prefix')");
    check("has load_config_prefix", expected, Value::Bool(true));
    let expected = py.eval("hasattr(extruder_stepper_mod, 'load_config')");
    check("has no load_config", expected, Value::Bool(false));
    let expected = py.eval("hasattr(extruder_stepper_mod, 'add_printer_objects')");
    check(
        "has no add_printer_objects",
        expected,
        Value::Bool(false),
    );
}

#[test]
fn missing_extruder_option_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("pp = FakePrinter(); cc = FakeConfig(pp, {})")
        .expect("py setup");
    let expected = py.eval("extruder_stepper_mod.load_config_prefix(cc)");
    let rig = make_rust(None);
    check_err(
        "missing_extruder_option_raises_config_error",
        expected,
        rig.result.map(|_| ()),
    );
}

#[test]
fn handle_connect_syncs_to_configured_extruder() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    let expected = py.eval("obj.handle_connect()");
    let rig = make_rust(Some("extruder"));
    let rust = rig.result.expect("rust new");
    let actual = rust.handle_connect().map(|()| Value::Null);
    check_value(
        "handle_connect_syncs_to_configured_extruder",
        expected,
        actual,
    );
    let expected_calls = py.eval("obj.extruder_stepper.sync_calls");
    let calls: Vec<String> = rig
        .stepper
        .sync_calls
        .borrow()
        .iter()
        .map(|c| c.clone().unwrap_or_else(|| "<none>".to_string()))
        .collect();
    check(
        "handle_connect_syncs_to_configured_extruder calls",
        expected_calls,
        Value::List(calls.iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn connect_event_dispatches_sync() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    let expected = py.eval("pp.handlers['klippy:connect'][0]()");
    let rig = make_rust(Some("extruder"));
    let _rust = rig.result.expect("rust new");
    let printer_handle: Printer = Arc::new(rig.printer.clone());
    let delivered = printer_handle.send_event(EV_KLIPPY_CONNECT, &[]);
    assert!(delivered.is_ok(), "send_event must succeed");
    check(
        "connect_event_dispatches_sync returns None",
        expected,
        Value::Null,
    );
    let expected_calls = py.eval("obj.extruder_stepper.sync_calls");
    let calls: Vec<String> = rig
        .stepper
        .sync_calls
        .borrow()
        .iter()
        .map(|c| c.clone().unwrap_or_else(|| "<none>".to_string()))
        .collect();
    check(
        "connect_event_dispatches_sync calls",
        expected_calls,
        Value::List(calls.iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn connect_handler_registered_once() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    let expected = py.eval("len(pp.handlers.get('klippy:connect', []))");
    let rig = make_rust(Some("extruder"));
    let _ = rig.result.expect("rust new");
    check(
        "connect_handler_registered_once",
        expected,
        Value::int(rig.printer.handler_count(EV_KLIPPY_CONNECT) as i64),
    );
}

#[test]
fn empty_extruder_name_passed_through() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("")).expect("py setup objects");
    let expected = py.eval("obj.handle_connect()");
    let rig = make_rust(Some(""));
    let rust = rig.result.expect("rust new");
    let actual = rust.handle_connect().map(|()| Value::Null);
    check_value(
        "empty_extruder_name_passed_through",
        expected,
        actual,
    );
    let expected_calls = py.eval("obj.extruder_stepper.sync_calls");
    let calls: Vec<String> = rig
        .stepper
        .sync_calls
        .borrow()
        .iter()
        .map(|c| c.clone().unwrap_or_else(|| "<none>".to_string()))
        .collect();
    check(
        "empty_extruder_name_passed_through calls",
        expected_calls,
        Value::List(calls.iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn get_status_delegates_pre_and_post_sync() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    let rig = make_rust(Some("extruder"));
    let rust = rig.result.expect("rust new");
    let expected = py.eval("obj.get_status(1.5)");
    check(
        "get_status before sync",
        expected,
        to_value(&rust.get_status(1.5)),
    );
    py.eval("obj.handle_connect()").expect("py handle_connect");
    rust.handle_connect().expect("rust handle_connect");
    let expected = py.eval("obj.get_status(1.5)");
    check(
        "get_status after sync",
        expected,
        to_value(&rust.get_status(1.5)),
    );
    // The PrinterObject::get_status override delegates identically.
    let expected = py.eval("obj.get_status(2.5)");
    check(
        "printer_object get_status",
        expected,
        to_value(&PrinterObject::get_status(&rust, 2.5).unwrap().expect("get_status Some")),
    );
}

#[test]
fn find_past_position_delegates() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    let rig = make_rust(Some("extruder"));
    let rust = rig.result.expect("rust new");
    for t in [10.0, 123.456789, -5.0, 0.0, 1e9] {
        let expected = py.eval(&format!("obj.find_past_position({t:?})"));
        check_value(
            &format!("find_past_position_delegates {t:?}"),
            expected,
            rust.find_past_position(t).map(Value::float),
        );
    }
}

#[test]
fn sync_error_propagates_through_handle_connect() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    py.exec("obj.extruder_stepper.sync_fail = 'bad sync'")
        .expect("py set fail");
    let expected = py.eval("obj.handle_connect()");
    let rig = make_rust(Some("extruder"));
    let rust = rig.result.expect("rust new");
    *rig.stepper.sync_fail.borrow_mut() = Some("bad sync".to_string());
    check_err(
        "sync_error_propagates_through_handle_connect",
        expected,
        rust.handle_connect(),
    );
}

#[test]
fn sync_error_propagates_through_send_event() {
    let Some(mut py) = oracle() else { return };
    make_py(&mut py, Some("extruder")).expect("py setup objects");
    py.exec("obj.extruder_stepper.sync_fail = 'bad sync'")
        .expect("py set fail");
    let expected = py.eval("pp.handlers['klippy:connect'][0]()");
    let rig = make_rust(Some("extruder"));
    let _rust = rig.result.expect("rust new");
    *rig.stepper.sync_fail.borrow_mut() = Some("bad sync".to_string());
    let printer_handle: Printer = Arc::new(rig.printer.clone());
    let actual = printer_handle
        .send_event(EV_KLIPPY_CONNECT, &[])
        .map(|_| ());
    check_err(
        "sync_error_propagates_through_send_event",
        expected,
        actual,
    );
}

#[test]
fn load_config_prefix_pending_factory() {
    let Some(_py) = oracle() else { return };
    // Constructing the inner kinematics `ExtruderStepper` requires a factory
    // the current contract does not expose (see the module report); the
    // factory entry point exists with the Python shape.
    let factory = module_factory();
    let load_config_prefix = factory
        .load_config_prefix
        .expect("extruder_stepper exposes load_config_prefix");
    let printer: Printer = Arc::new(FakePrinter::new());
    let config: Config = Arc::new(FakeConfig::new(printer, Some("extruder")));
    let obj = load_config_prefix(&config).expect("load_config_prefix runs");
    assert!(
        obj.is_none(),
        "no object is built until the ExtruderStepper factory lands"
    );
}

#[test]
fn wrapper_declares_extruder_stepper_interface() {
    let Some(_py) = oracle() else { return };
    let rig = make_rust(Some("extruder"));
    let rust = rig.result.expect("rust new");
    let obj: ObjectRef = Arc::new(rust) as ObjectRef;
    assert!(
        interface::<dyn ExtruderStepperApi>(&obj).is_some(),
        "wrapper is duck-typed as an extruder stepper"
    );
}
