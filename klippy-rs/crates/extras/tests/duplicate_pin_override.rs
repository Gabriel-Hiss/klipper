//! Parity tests for `extras::duplicate_pin_override`
//! (port of `klippy/extras/duplicate_pin_override.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): `PrinterDupPinOverride` is
//! built on both sides with minimal Python fakes for the printer, the
//! config wrapper and the `pins` object (the real `pins.PrinterPins`,
//! wrapped to record `allow_multi_use_pin` calls), then fed the same
//! `pins` option value and the resulting calls, pin-sharing entries and
//! errors are compared exactly (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::duplicate_pin_override::PrinterDupPinOverride;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    Chip, LookupPinOpts, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::ObjectRef;
use klippy_api::{downcast_arc, EventArg, GCell};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import pins
import extras.duplicate_pin_override as dup

sentinel = object()

class RecordingPins(pins.PrinterPins):
    def __init__(self):
        super().__init__()
        self.calls = []
        self.register_chip('mcu', object())
        self.register_chip('extr1', object())
    def allow_multi_use_pin(self, pin_desc):
        self.calls.append(pin_desc)
        super().allow_multi_use_pin(pin_desc)

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, pins_obj, has_pins):
        self._pins = pins_obj
        self._has_pins = has_pins
    def lookup_object(self, name, default=sentinel):
        if name == 'pins' and self._has_pins:
            return self._pins
        if default is sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, pins_value):
        self._printer = printer
        self._pins = pins_value
        self.section = 'duplicate_pin_override'
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def getlist(self, option, default=sentinel, sep=',', count=None, note_valid=True):
        assert option == 'pins'
        if self._pins is None:
            raise configparser.Error("Option 'pins' in section 'duplicate_pin_override' must be specified")
        if len(self._pins.strip()) == 0:
            return ()
        return tuple(part.strip() for part in self._pins.split(sep))
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn pins_error(message: String) -> KlippyError {
    KlippyError::Other {
        class_name: "error".to_string(),
        message,
    }
}

pub struct FakePins {
    calls: GCell<Vec<String>>,
    chips: Vec<String>,
}

impl FakePins {
    fn new() -> FakePins {
        FakePins {
            calls: GCell::new(Vec::new()),
            chips: vec!["mcu".to_string(), "extr1".to_string()],
        }
    }

    fn calls(&self) -> Vec<String> {
        self.calls.cloned()
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by duplicate_pin_override");
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by duplicate_pin_override");
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by duplicate_pin_override");
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by duplicate_pin_override");
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used by duplicate_pin_override");
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by duplicate_pin_override");
    }
    /// `PrinterPins.allow_multi_use_pin` (`klippy/pins.py:132`), replicated
    /// so the fake raises the same `pins.error` messages as the real object.
    fn allow_multi_use_pin(&self, pin_desc: &str) -> KResult<()> {
        self.calls.borrow_mut().push(pin_desc.to_string());
        let desc = pin_desc.trim();
        let (chip_name, pin) = match desc.split_once(':') {
            Some((chip, pin)) => (chip.trim(), pin.trim()),
            None => ("mcu", desc),
        };
        if !self.chips.iter().any(|chip| chip == chip_name) {
            return Err(pins_error(format!("Unknown pin chip name '{chip_name}'")));
        }
        if pin.contains(['^', '~', '!', ':'])
            || pin.split_whitespace().collect::<String>() != pin
        {
            return Err(pins_error(format!(
                "Invalid pin description '{pin_desc}'\nFormat is: [chip_name:] pin_name"
            )));
        }
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

#[derive(Clone)]
struct FakePrinter {
    pins: Option<Arc<FakePins>>,
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
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if name == "pins" {
            if let Some(pins) = &self.pins {
                return Ok(Some(pins.clone() as ObjectRef));
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

struct FakeConfig {
    printer: Printer,
    pins_raw: Option<String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "duplicate_pin_override".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get is not used by duplicate_pin_override");
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by duplicate_pin_override");
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by duplicate_pin_override");
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by duplicate_pin_override");
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by duplicate_pin_override");
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by duplicate_pin_override");
    }
    /// `ConfigWrapper.getlist` (`klippy/configfile.py:107`): required
    /// option, comma-separated, each element stripped, empty value yields
    /// an empty list.
    fn getlist(&self, option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        assert_eq!(option, "pins", "duplicate_pin_override reads only 'pins'");
        match &self.pins_raw {
            None => Err(KlippyError::config(
                "Option 'pins' in section 'duplicate_pin_override' must be specified",
            )),
            Some(raw) => {
                if raw.trim().is_empty() {
                    Ok(Some(Vec::new()))
                } else {
                    Ok(Some(
                        raw.split(',').map(|part| part.trim().to_string()).collect(),
                    ))
                }
            }
        }
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by duplicate_pin_override");
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by duplicate_pin_override");
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by duplicate_pin_override");
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by duplicate_pin_override");
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by duplicate_pin_override");
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options is not used by duplicate_pin_override");
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("duplicate_pin_override fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping duplicate_pin_override oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

struct Harness {
    pins: Arc<FakePins>,
    config: Config,
}

/// Builds fresh Python (`p`, `c`) and Rust objects for one `pins` value.
/// `pins_raw == None` means the option is missing; `has_pins == false`
/// means the `pins` printer object does not exist.
fn fresh_pair(
    py: &mut pyoracle::Oracle,
    pins_raw: Option<&str>,
    has_pins: bool,
) -> Harness {
    let py_pins = match pins_raw {
        None => "None".to_string(),
        Some(raw) => format!("{raw:?}"),
    };
    let py_has_pins = if has_pins { "True" } else { "False" };
    py.exec(&format!(
        "p = RecordingPins(); pr = FakePrinter(p, {py_has_pins}); c = FakeConfig(pr, {py_pins})"
    ))
    .expect("py fresh pair");
    let pins = Arc::new(FakePins::new());
    let printer: Printer = if has_pins {
        Arc::new(FakePrinter {
            pins: Some(pins.clone()),
        })
    } else {
        Arc::new(FakePrinter { pins: None })
    };
    let config: Config = Arc::new(FakeConfig {
        printer,
        pins_raw: pins_raw.map(|raw| raw.to_string()),
    });
    Harness { pins, config }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "duplicate_pin_override".to_string(),
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

fn compare_load<T>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    match (expected, actual) {
        (Ok(_), Ok(_)) => {}
        (Ok(v), Err(r)) => {
            panic!("{label}: oracle returned {v:?} but rust raised {}: {}", r.class_name(), r.message())
        }
        (Err(e), Ok(_)) => {
            panic!("{label}: oracle raised {}: {} but rust call succeeded", e.class_name, e.message)
        }
        (Err(e), Err(r)) => {
            assert_eq!(
                r.class_name(),
                e.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                r.message(),
                e.message.as_str(),
                "{label}: exception message",
            );
        }
    }
}

fn check_calls(
    label: &str,
    py: &mut pyoracle::Oracle,
    harness: &Harness,
) {
    let expected = py.eval("p.calls").expect("py calls");
    let actual = Value::List(
        harness
            .pins
            .calls()
            .into_iter()
            .map(Value::Str)
            .collect(),
    );
    check(&format!("{label} calls"), Ok(expected), actual);
}

/// Runs `dup.load_config(c)` on the Python side and
/// `PrinterDupPinOverride::new(&config)` on the Rust side with the same
/// fresh fakes, then compares the outcome and the recorded pin calls.
fn run_case(
    label: &str,
    py: &mut pyoracle::Oracle,
    harness: &Harness,
) {
    let expected = py.eval("dup.load_config(c)");
    let actual = PrinterDupPinOverride::new(&harness.config);
    compare_load(label, expected, actual);
    check_calls(label, py, harness);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn single_pin() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA1"), true);
    run_case("single_pin", &mut py, &harness);
    let expected = py.eval("sorted(p.allow_multi_use_pins.keys())");
    check(
        "single_pin pins",
        expected,
        Value::List(vec![Value::str("mcu:PA1")]),
    );
}

#[test]
fn multiple_pins_in_order() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA1,mcu:PB2,extr1:PC3"), true);
    run_case("multiple_pins_in_order", &mut py, &harness);
}

#[test]
fn pin_without_chip_defaults_to_mcu() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("PA1"), true);
    run_case("pin_without_chip_defaults_to_mcu", &mut py, &harness);
}

#[test]
fn whitespace_is_stripped() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("  mcu:PA1 ,\t mcu:PB2  "), true);
    run_case("whitespace_is_stripped", &mut py, &harness);
}

#[test]
fn empty_value_allows_nothing() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some(""), true);
    run_case("empty_value_allows_nothing", &mut py, &harness);
}

#[test]
fn missing_option_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, None, true);
    run_case("missing_option_raises_config_error", &mut py, &harness);
}

#[test]
fn missing_pins_object_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA1"), false);
    run_case("missing_pins_object_raises_config_error", &mut py, &harness);
}

#[test]
fn unknown_chip_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("badchip:PA1"), true);
    run_case("unknown_chip_raises_pins_error", &mut py, &harness);
}

#[test]
fn invalid_pin_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA 1"), true);
    run_case("invalid_pin_raises_pins_error", &mut py, &harness);
}

#[test]
fn invert_char_is_invalid_without_can_invert() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("!PA1"), true);
    run_case("invert_char_is_invalid_without_can_invert", &mut py, &harness);
}

#[test]
fn first_error_aborts_remaining_pins() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA1,badchip:PB2,mcu:PC3"), true);
    run_case("first_error_aborts_remaining_pins", &mut py, &harness);
}

#[test]
fn duplicate_pins_are_allowed_twice() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:PA1,mcu:PA1"), true);
    run_case("duplicate_pins_are_allowed_twice", &mut py, &harness);
    let expected = py.eval("sorted(p.allow_multi_use_pins.keys())");
    check(
        "duplicate_pins_are_allowed_twice pins",
        expected,
        Value::List(vec![Value::str("mcu:PA1")]),
    );
}

#[test]
fn empty_pin_name_is_valid() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some("mcu:"), true);
    run_case("empty_pin_name_is_valid", &mut py, &harness);
}

#[test]
fn empty_chip_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    let harness = fresh_pair(&mut py, Some(":"), true);
    run_case("empty_chip_raises_pins_error", &mut py, &harness);
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::duplicate_pin_override::module_factory();
    let load_config = factory
        .load_config
        .expect("duplicate_pin_override exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "duplicate_pin_override has no load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "duplicate_pin_override has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "duplicate_pin_override has no add_printer_objects"
    );
    let pins = Arc::new(FakePins::new());
    let printer: Printer = Arc::new(FakePrinter {
        pins: Some(pins.clone()),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        pins_raw: Some("mcu:PA1,extr1:PB2".to_string()),
    });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let _: Arc<PrinterDupPinOverride> = downcast_arc(&obj).expect("downcast to PrinterDupPinOverride");
    assert_eq!(pins.calls(), vec!["mcu:PA1".to_string(), "extr1:PB2".to_string()]);
    py.exec("p2 = RecordingPins(); pr2 = FakePrinter(p2, True); c2 = FakeConfig(pr2, 'mcu:PA1,extr1:PB2'); dup.load_config(c2)")
        .expect("py factory load_config");
    let expected = py.eval("p2.calls");
    check(
        "factory calls",
        expected,
        Value::List(vec![Value::str("mcu:PA1"), Value::str("extr1:PB2")]),
    );
}
