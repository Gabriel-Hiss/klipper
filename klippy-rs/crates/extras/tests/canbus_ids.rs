//! Parity tests for `extras::canbus_ids` (port of `klippy/extras/canbus_ids.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterCANBus` is built
//! on both sides with minimal Python fakes for the printer and the config
//! wrapper, then fed identical `add_uuid` / `get_nodeid` calls and the
//! resulting node ids and config errors are compared exactly
//! (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::canbus_ids::{PrinterCANBus, NODEID_FIRST};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::ObjectRef;
use klippy_api::{downcast_arc, EventArg};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.canbus_ids as canbus_ids

class FakePrinter:
    config_error = configparser.Error

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter;

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

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "canbus_ids".to_string()
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
        panic!("getsection is not used by canbus_ids")
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
            py.exec(SETUP).expect("canbus_ids fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping canbus_ids oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn fresh_pair(py: &mut pyoracle::Oracle) -> (PrinterCANBus, Config) {
    py.exec("p = FakePrinter(); c = FakeConfig(p); cb = canbus_ids.load_config(c)")
        .expect("py load_config");
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
    });
    let rust = PrinterCANBus::new(&config);
    (rust, config)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "canbus_ids".to_string(),
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

fn check_id(label: &str, expected: pyoracle::OracleResult, actual: KResult<i64>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), Value::int(actual));
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<i64>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(v) => panic!("{label}: rust returned {v} but call must fail"),
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
    // Both sides of canbus_ids raise configparser.Error, i.e. the
    // KlippyError::Config class "Error".
    assert_eq!(
        rust_err.class_name(),
        "Error",
        "{label}: canbus_ids errors are config errors"
    );
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn nodeid_first_matches_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("canbus_ids.NODEID_FIRST").expect("eval NODEID_FIRST");
    check(
        "nodeid_first_matches_python",
        Ok(expected),
        Value::int(NODEID_FIRST),
    );
}

#[test]
fn add_uuid_assigns_sequential_ids() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    for (uuid, iface) in [("uuid-a", "can0"), ("uuid-b", "can0"), ("uuid-c", "can1")] {
        let label = format!("add_uuid_assigns_sequential_ids {uuid}");
        let expr = format!("cb.add_uuid(c, '{uuid}', '{iface}')");
        let expected = py.eval(&expr);
        let actual = rust.add_uuid(&config, uuid, iface);
        check_id(&label, expected, actual);
    }
}

#[test]
fn get_nodeid_returns_assigned_ids() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    for uuid in ["uuid-a", "uuid-b"] {
        let expr = format!("cb.add_uuid(c, '{uuid}', 'can0')");
        let expected = py.eval(&expr);
        check_id("setup add", expected, rust.add_uuid(&config, uuid, "can0"));
    }
    for uuid in ["uuid-a", "uuid-b"] {
        let expr = format!("cb.get_nodeid('{uuid}')");
        let expected = py.eval(&expr);
        check_id(
            &format!("get_nodeid_returns_assigned_ids {uuid}"),
            expected,
            rust.get_nodeid(uuid),
        );
    }
}

#[test]
fn duplicate_uuid_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can0')");
    check_id("setup add", expected, rust.add_uuid(&config, "uuid-a", "can0"));
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can0')");
    check_err(
        "duplicate_uuid_raises_config_error",
        expected,
        rust.add_uuid(&config, "uuid-a", "can0"),
    );
    // The duplicate iface does not matter; a different iface still fails.
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can1')");
    check_err(
        "duplicate_uuid_raises_config_error other iface",
        expected,
        rust.add_uuid(&config, "uuid-a", "can1"),
    );
}

#[test]
fn unknown_uuid_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can0')");
    check_id("setup add", expected, rust.add_uuid(&config, "uuid-a", "can0"));
    let expected = py.eval("cb.get_nodeid('missing')");
    check_err(
        "unknown_uuid_raises_config_error",
        expected,
        rust.get_nodeid("missing"),
    );
    let expected = py.eval("cb.get_nodeid('')");
    check_err(
        "unknown_uuid_raises_config_error empty",
        expected,
        rust.get_nodeid(""),
    );
}

#[test]
fn failed_duplicate_keeps_next_id() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can0')");
    check_id("setup add", expected, rust.add_uuid(&config, "uuid-a", "can0"));
    let expected = py.eval("cb.add_uuid(c, 'uuid-a', 'can0')");
    check_err("failed duplicate", expected, rust.add_uuid(&config, "uuid-a", "can0"));
    // The second id is still 5: the failed call did not consume an id.
    let expected = py.eval("cb.add_uuid(c, 'uuid-b', 'can0')");
    check_id(
        "failed_duplicate_keeps_next_id",
        expected,
        rust.add_uuid(&config, "uuid-b", "can0"),
    );
}

#[test]
fn empty_and_special_uuids() {
    let Some(mut py) = oracle() else { return };
    let (rust, config) = fresh_pair(&mut py);
    let uuids = ["", "uuid with spaces", "uuid-%s-%r", "canbus\u{2603}"];
    for uuid in uuids {
        let py_uuid = format!("'{uuid}'");
        let expr = format!("cb.add_uuid(c, {py_uuid}, 'can0')");
        let expected = py.eval(&expr);
        check_id(
            &format!("empty_and_special_uuids add {uuid:?}"),
            expected,
            rust.add_uuid(&config, uuid, "can0"),
        );
        let expr = format!("cb.get_nodeid({py_uuid})");
        let expected = py.eval(&expr);
        check_id(
            &format!("empty_and_special_uuids get {uuid:?}"),
            expected,
            rust.get_nodeid(uuid),
        );
    }
    // Empty string is an ordinary key: re-adding it is a duplicate.
    let expected = py.eval("cb.add_uuid(c, '', 'can0')");
    check_err(
        "empty_and_special_uuids duplicate empty",
        expected,
        rust.add_uuid(&config, "", "can0"),
    );
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::canbus_ids::module_factory();
    let load_config = factory
        .load_config
        .expect("canbus_ids exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "canbus_ids has no load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "canbus_ids has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "canbus_ids has no add_printer_objects"
    );
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig { printer });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let rust: Arc<PrinterCANBus> = downcast_arc(&obj).expect("downcast to PrinterCANBus");
    // The object built through the factory behaves like the Python one.
    let expected = py.eval("canbus_ids.NODEID_FIRST");
    check("factory nodeid_first", expected, Value::int(NODEID_FIRST));
    py.exec("c2 = FakeConfig(FakePrinter()); cb2 = canbus_ids.load_config(c2)")
        .expect("py factory load_config");
    let expected = py.eval("cb2.add_uuid(c2, 'uuid-a', 'can0')");
    check_id(
        "factory add_uuid",
        expected,
        rust.add_uuid(&config, "uuid-a", "can0"),
    );
    let expected = py.eval("cb2.get_nodeid('uuid-a')");
    check_id(
        "factory get_nodeid",
        expected,
        rust.get_nodeid("uuid-a"),
    );
    let expected = py.eval("cb2.get_nodeid('nope')");
    check_err("factory unknown", expected, rust.get_nodeid("nope"));
}
