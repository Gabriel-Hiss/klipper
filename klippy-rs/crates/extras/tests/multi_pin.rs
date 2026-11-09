//! Parity tests for `extras::multi_pin` (port of `klippy/extras/multi_pin.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterMultiPin` is
//! built on both sides with minimal Python fakes for the printer, the `pins`
//! object, the config wrapper and the MCU pins, then fed identical
//! `setup_pin` and fan-out calls; setup descriptions, fan-out sequences,
//! returned MCU names and errors (class and message) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.
//!
//! `setup_pin` returns the virtual pin itself (`self` in Python): the Rust side
//! returns a `Pin` wrapping the same `PrinterMultiPin`, and the returned pin
//! is driven through the pin traits to prove the fan-out. The empty `pins`
//! list case returns `self` too; see `setup_pin_with_empty_list`.

use extras::multi_pin::{module_factory, MultiPinApi, PrinterMultiPin};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, ChipApi, CommandQueue,
    CommandQueryWrapper, CommandWrapper, ConfigCallback, DigitalOutPin, DigitalOutPinApi,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, PinsApi, PwmPin, PwmPinApi,
    SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Canonical call-log formatting shared by both sides.
// ---------------------------------------------------------------------------

fn f_repr(x: f64) -> String {
    pycompat::text::float_repr(x)
}

fn b_repr(b: bool) -> &'static str {
    if b {
        "1"
    } else {
        "0"
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.multi_pin as multi_pin

CALLS = []

class error(Exception):
    pass

class FakeMcu:
    def __init__(self, name):
        self.name = name

class MCU_digital_out:
    def __init__(self, desc):
        self.desc = desc
        self.mcu = FakeMcu('mcu')
    def get_mcu(self):
        return self.mcu
    def setup_max_duration(self, v):
        CALLS.append('setup_max_duration:' + repr(v))
    def setup_start_value(self, start_value, shutdown_value):
        CALLS.append('setup_start_value:' + ('1' if start_value else '0') + ':' + ('1' if shutdown_value else '0'))
    def set_digital(self, print_time, value):
        CALLS.append('set_digital:' + repr(print_time) + ':' + ('1' if value else '0'))

class MCU_pwm:
    def __init__(self, desc):
        self.desc = desc
        self.mcu = FakeMcu('mcu')
    def get_mcu(self):
        return self.mcu
    def setup_max_duration(self, v):
        CALLS.append('setup_max_duration:' + repr(v))
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        CALLS.append('setup_cycle_time:' + repr(cycle_time))
    def setup_start_value(self, start_value, shutdown_value):
        CALLS.append('setup_start_value:' + repr(start_value) + ':' + repr(shutdown_value))
    def set_pwm(self, print_time, value):
        CALLS.append('set_pwm:' + repr(print_time) + ':' + repr(value))

class FakePins:
    error = error
    def __init__(self):
        self.chips = {}
        self.duplicate = False
        self.fail_desc = None
    def register_chip(self, chip_name, chip):
        if self.duplicate or chip_name in self.chips:
            raise self.error("Duplicate chip name '%s'" % (chip_name,))
        self.chips[chip_name] = chip
    def setup_pin(self, pin_type, pin_desc):
        CALLS.append('setup_pin:' + pin_type + ':' + pin_desc)
        if self.fail_desc is not None and pin_desc == self.fail_desc:
            raise self.error("Unknown pin chip name 'bogus'")
        if pin_type == 'pwm':
            return MCU_pwm(pin_desc)
        return MCU_digital_out(pin_desc)

_MISSING = object()

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
    def lookup_object(self, name, default=_MISSING):
        if name in self.objects:
            return self.objects[name]
        if default is not _MISSING:
            return default
        raise self.config_error("Unknown config object '%s'" % (name,))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, pins):
        self._printer = printer
        self._pins = pins
    def get_printer(self):
        return self._printer
    def getlist(self, option, **kwargs):
        if option == 'pins':
            if self._pins is None:
                raise configparser.Error("Required option 'pins' missing")
            return self._pins
        raise AssertionError('unexpected getlist ' + option)

def mk_pair(pins):
    printer = FakePrinter()
    pins_obj = FakePins()
    printer.objects['pins'] = pins_obj
    config = FakeConfig(printer, pins)
    mpin = multi_pin.load_config_prefix(config)
    return printer, pins_obj, config, mpin
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Calls = Arc<GCell<Vec<String>>>;

fn calls_joined(calls: &Calls) -> String {
    calls.borrow().join("|")
}

struct FakePins {
    calls: Calls,
    chips: GCell<Vec<String>>,
    duplicate: GCell<bool>,
    fail_desc: GCell<Option<String>>,
}

impl FakePins {
    fn new(calls: Calls) -> Arc<Self> {
        Arc::new(FakePins { calls, chips: GCell::new(Vec::new()), duplicate: GCell::new(false), fail_desc: GCell::new(None) })
    }

    fn chip_names(&self) -> Vec<String> {
        self.chips.borrow().clone()
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by multi_pin")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by multi_pin")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.calls.borrow_mut().push(format!("setup_pin:{pin_type}:{pin_desc}"));
        if self.fail_desc.borrow().as_deref() == Some(pin_desc) {
            return Err(KlippyError::Pins("Unknown pin chip name 'bogus'".to_string()));
        }
        if pin_type == "pwm" {
            Ok(Pin::Pwm(Arc::new(FakePwmPin { mcu: fake_mcu(), calls: self.calls.clone() }) as PwmPin))
        } else {
            Ok(Pin::DigitalOut(Arc::new(FakeDigitalOutPin {
                mcu: fake_mcu(),
                calls: self.calls.clone(),
            }) as DigitalOutPin))
        }
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { panic!("reset_pin_sharing is not used by multi_pin") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by multi_pin")
    }
    fn register_chip(&self, chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        if self.duplicate.get() || self.chips.borrow().iter().any(|c| c == chip_name) {
            return Err(KlippyError::Pins(format!("Duplicate chip name '{chip_name}'")));
        }
        self.chips.borrow_mut().push(chip_name.to_string());
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by multi_pin")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeDigitalOutPin {
    mcu: Mcu,
    calls: Calls,
}

impl DigitalOutPinApi for FakeDigitalOutPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.calls.borrow_mut().push(format!("setup_max_duration:{}", f_repr(max_duration)));
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        self.calls
            .borrow_mut()
            .push(format!("setup_start_value:{}:{}", b_repr(start_value), b_repr(shutdown_value)));
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        self.calls.borrow_mut().push(format!("set_digital:{}:{}", f_repr(print_time), b_repr(value)));
    }
}

struct FakePwmPin {
    mcu: Mcu,
    calls: Calls,
}

impl PwmPinApi for FakePwmPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.calls.borrow_mut().push(format!("setup_max_duration:{}", f_repr(max_duration)));
    }
    fn setup_cycle_time(&self, cycle_time: f64, _hardware_pwm: Option<bool>) -> klippy_api::KResult<()> {
        self.calls.borrow_mut().push(format!("setup_cycle_time:{}", f_repr(cycle_time)));
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        self.calls
            .borrow_mut()
            .push(format!("setup_start_value:{}:{}", f_repr(start_value), f_repr(shutdown_value)));
    }
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> klippy_api::KResult<()> {
        self.calls.borrow_mut().push(format!("set_pwm:{}:{}", f_repr(print_time), f_repr(value)));
        Ok(())
    }
}

struct FakeMcu {
    name: String,
}

fn fake_mcu() -> Mcu {
    Arc::new(FakeMcu { name: "mcu".to_string() })
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by multi_pin")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by multi_pin")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by multi_pin")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by multi_pin")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by multi_pin")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by multi_pin")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by multi_pin")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by multi_pin")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by multi_pin")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by multi_pin")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by multi_pin")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by multi_pin")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by multi_pin")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by multi_pin")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by multi_pin")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by multi_pin")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("get_enumerations is not used by multi_pin")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("get_constants is not used by multi_pin")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by multi_pin")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by multi_pin")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by multi_pin")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by multi_pin")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by multi_pin")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by multi_pin")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by multi_pin")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by multi_pin")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by multi_pin")
    }
}

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<GCell<Vec<(String, ObjectRef)>>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter { objects: Arc::new(GCell::new(Vec::new())) }
    }

    fn insert(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().push((name.to_string(), obj));
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.insert(name, obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        for (n, obj) in self.objects.borrow().iter() {
            if n == name {
                return Ok(Some(obj.clone()));
            }
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unknown config object '{name}'")))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.borrow().clone().into_iter().map(|(name, obj)| (name, Some(obj))).collect()
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
    pins: Option<Vec<String>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "multi_pin fanhall".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get is not used by multi_pin")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by multi_pin")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by multi_pin")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by multi_pin")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by multi_pin")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by multi_pin")
    }
    fn getlist(&self, option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        assert_eq!(option, "pins");
        match &self.pins {
            Some(pins) => Ok(Some(pins.clone())),
            None => Err(KlippyError::config("Required option 'pins' missing")),
        }
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by multi_pin")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by multi_pin")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by multi_pin")
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

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("multi_pin fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping multi_pin oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "multi_pin".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, rust raised an error"),
        Err(e) => e,
    };
    let rust_exc = match actual {
        Ok(()) => panic!("{label}: rust returned (), oracle raised {}: {}", oracle_exc.class_name, oracle_exc.message),
        Err(e) => e,
    };
    assert_eq!(rust_exc.class_name(), oracle_exc.class_name, "{label}: error class");
    assert_eq!(rust_exc.message(), oracle_exc.message, "{label}: error message");
}

fn rust_pair(pins: Vec<String>) -> (FakePrinter, Arc<FakePins>, Config, Calls) {
    let calls: Calls = Arc::new(GCell::new(Vec::new()));
    let pins_obj = FakePins::new(calls.clone());
    let printer = FakePrinter::new();
    printer.insert("pins", pins_obj.clone() as ObjectRef);
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig { printer: printer_arc, pins: Some(pins) });
    (printer, pins_obj, config, calls)
}

fn pin_params(pin: &str, invert: i64) -> PinParams {
    PinParams {
        chip: fake_mcu(),
        chip_name: "multi_pin".to_string(),
        pin: pin.to_string(),
        invert,
        pullup: 0,
        share_type: None,
    }
}

/// The `Pin` returned by `ChipApi::setup_pin` must wrap the virtual pin itself
/// (`self` in Python), in the variant matching the requested type; driving it
/// through the pin trait must fan out to every underlying pin.
fn as_pwm(pin: Pin) -> PwmPin {
    match pin {
        Pin::Pwm(p) => p,
        other => panic!("expected Pin::Pwm, got {}", pin_variant_name(&other)),
    }
}

fn as_digital_out(pin: Pin) -> DigitalOutPin {
    match pin {
        Pin::DigitalOut(p) => p,
        other => panic!("expected Pin::DigitalOut, got {}", pin_variant_name(&other)),
    }
}

fn pin_variant_name(pin: &Pin) -> &'static str {
    match pin {
        Pin::Endstop(_) => "Endstop",
        Pin::DigitalOut(_) => "DigitalOut",
        Pin::Pwm(_) => "Pwm",
        Pin::Adc(_) => "Adc",
    }
}

fn rust_calls(calls: &Calls) -> Value {
    Value::str(&calls_joined(calls))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn factory_has_only_load_config_prefix() {
    let factory = module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn load_config_prefix_registers_chip_and_stores_pins() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2', '!PB1'])")
        .expect("py constructs multi_pin");
    let (_printer, pins_rs, config, _calls) = rust_pair(vec!["PA1".into(), "PA2".into(), "!PB1".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs multi_pin");

    let actual_list = Value::List(mpin.pin_list().iter().map(|s| Value::str(s)).collect());
    check("pin_list", py.eval("mpin.pin_list"), actual_list);
    let actual_pin_type = match mpin.pin_type() {
        Some(t) => Value::str(&t),
        None => Value::Null,
    };
    check("pin_type", py.eval("mpin.pin_type"), actual_pin_type);
    let actual_chips = Value::List(pins_rs.chip_names().iter().map(|s| Value::str(s)).collect());
    check("chip_names", py.eval("sorted(pins_obj.chips)"), actual_chips);
    check_err("get_mcu_empty", py.eval("mpin.get_mcu()"), MultiPinApi::get_mcu(&*mpin).map(|_| ()));
}

#[test]
fn setup_pin_inverted_sets_up_every_pin() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    let (printer, _pins, config, calls) = rust_pair(vec!["PA1".into(), "PA2".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);

    py.exec("res = mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 1})").expect("py setup");
    check("py_setup_result", py.eval("res is mpin"), Value::Bool(true));
    py.eval("res.set_pwm(1.0, 0.5)").expect("py set_pwm on returned pin");
    let expected_calls = py.eval("'|'.join(CALLS)").expect("py calls");
    let returned = mpin.setup_pin("pwm", pin_params("fanhall", 1)).expect("rust setup_pin");
    as_pwm(returned).set_pwm(1.0, 0.5).expect("set_pwm");
    check("setup_calls", Ok(expected_calls), rust_calls(&calls));
    let actual_pin_type = Value::str(&mpin.pin_type().unwrap());
    check("pin_type", py.eval("mpin.pin_type"), actual_pin_type);
    check("mcu_count", py.eval("len(mpin.mcu_pins)"), Value::int(mpin.mcu_pin_count() as i64));
}

#[test]
fn setup_pin_not_inverted_is_not_prefixed() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    let (printer, _pins, config, calls) = rust_pair(vec!["PA1".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);

    py.exec("res = mpin.setup_pin('digital_out', {'pin': 'fanhall', 'invert': 0})").expect("py setup");
    check("py_setup_result", py.eval("res is mpin"), Value::Bool(true));
    py.eval("res.set_digital(2.0, True)").expect("py set_digital on returned pin");
    let expected_calls = py.eval("'|'.join(CALLS)").expect("py calls");
    let returned = mpin.setup_pin("digital_out", pin_params("fanhall", 0)).expect("rust setup");
    as_digital_out(returned).set_digital(2.0, true);
    check("setup_calls", Ok(expected_calls), rust_calls(&calls));
}

#[test]
fn setup_pin_unknown_virtual_pin_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    let (_p, _pins, config, _calls) = rust_pair(vec!["PA1".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    let actual = mpin.setup_pin("pwm", pin_params("zz", 0)).map(|_| ()).err().unwrap();
    check_err("unknown_pin", py.eval("mpin.setup_pin('pwm', {'pin': 'zz', 'invert': 0})"), Err(actual));
}

#[test]
fn setup_pin_twice_raises_pins_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    py.eval("mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 0})").expect("py first setup");
    let (printer, _pins, config, _calls) = rust_pair(vec!["PA1".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);
    mpin.setup_pin("pwm", pin_params("fanhall", 0)).expect("rust first setup");

    let actual = mpin.setup_pin("pwm", pin_params("fanhall", 0)).map(|_| ()).err().unwrap();
    check_err("twice", py.eval("mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 0})"), Err(actual));
}

#[test]
fn setup_pin_partial_failure_keeps_pin_type_and_empty_pins() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    py.exec("pins_obj.fail_desc = '!PA2'").expect("fail on second");
    let (printer, pins_rs, config, calls) = rust_pair(vec!["PA1".into(), "PA2".into()]);
    *pins_rs.fail_desc.borrow_mut() = Some("!PA2".to_string());
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);

    let actual = mpin.setup_pin("pwm", pin_params("fanhall", 1)).map(|_| ()).err().unwrap();
    check_err("partial_failure", py.eval("mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 1})"), Err(actual));
    let actual_pin_type = Value::str(&mpin.pin_type().unwrap());
    check("pin_type", py.eval("mpin.pin_type"), actual_pin_type);
    check("mcu_count", py.eval("len(mpin.mcu_pins)"), Value::int(mpin.mcu_pin_count() as i64));
    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("py calls")), rust_calls(&calls));
    // A retry now hits the "twice" error on both sides.
    let actual2 = mpin.setup_pin("pwm", pin_params("fanhall", 1)).map(|_| ()).err().unwrap();
    check_err("retry", py.eval("mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 1})"), Err(actual2));
}

#[test]
fn setup_pin_delegates_to_other_multi_pin() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter(); pins_obj = FakePins(); printer.objects['pins'] = pins_obj").expect("py printer");
    py.exec("a = multi_pin.load_config_prefix(FakeConfig(printer, ['A1']))").expect("py a");
    py.exec("b = multi_pin.load_config_prefix(FakeConfig(printer, ['B1', 'B2']))").expect("py b");
    py.exec("printer.objects['multi_pin a'] = a; printer.objects['multi_pin b'] = b").expect("py register");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("res = a.setup_pin('pwm', {'pin': 'b', 'invert': 0})").expect("py delegates");
    check("py_delegates_result", py.eval("res is b"), Value::Bool(true));
    py.eval("res.set_pwm(1.0, 0.5)").expect("py set_pwm on delegated pin");

    let (printer, _pins, _config, calls) = rust_pair(vec!["A1".into()]);
    let printer_arc: Printer = Arc::new(printer.clone());
    let config_a: Config = Arc::new(FakeConfig { printer: printer_arc.clone(), pins: Some(vec!["A1".into()]) });
    let config_b: Config = Arc::new(FakeConfig { printer: printer_arc, pins: Some(vec!["B1".into(), "B2".into()]) });
    let a = PrinterMultiPin::new(&config_a).expect("rust a");
    let b = PrinterMultiPin::new(&config_b).expect("rust b");
    printer.insert("multi_pin a", a.clone() as ObjectRef);
    printer.insert("multi_pin b", b.clone() as ObjectRef);
    let returned = a.setup_pin("pwm", pin_params("b", 0)).expect("rust delegates");
    as_pwm(returned).set_pwm(1.0, 0.5).expect("set_pwm");

    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("py calls")), rust_calls(&calls));
    let a_pin_type = match a.pin_type() { Some(t) => Value::str(&t), None => Value::Null };
    check("a_pin_type", py.eval("a.pin_type"), a_pin_type);
    check("b_pin_type", py.eval("b.pin_type"), Value::str(&b.pin_type().unwrap()));
    check("a_mcu_count", py.eval("len(a.mcu_pins)"), Value::int(a.mcu_pin_count() as i64));
    check("b_mcu_count", py.eval("len(b.mcu_pins)"), Value::int(b.mcu_pin_count() as i64));
}

#[test]
fn get_mcu_returns_first_underlying_mcu() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2'])").expect("py constructs");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    py.eval("mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 0})").expect("py setup");
    let (printer, _pins, config, _calls) = rust_pair(vec!["PA1".into(), "PA2".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);
    mpin.setup_pin("pwm", pin_params("fanhall", 0)).expect("rust setup");

    let actual_mcu = Value::str(&MultiPinApi::get_mcu(&*mpin).expect("rust get_mcu").get_name());
    check("get_mcu", py.eval("mpin.get_mcu().name"), actual_mcu);
    let actual_chip_mcu = Value::str(&ChipApi::get_mcu(&*mpin).expect("chip get_mcu").get_name());
    check("chip_get_mcu", Ok(actual_chip_mcu.clone()), actual_chip_mcu);
}

#[test]
fn fan_out_digital() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2'])").expect("py constructs");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    py.exec("res = mpin.setup_pin('digital_out', {'pin': 'fanhall', 'invert': 0})").expect("py setup");
    py.exec("CALLS.clear()").expect("clear");
    let (printer, _pins, config, calls) = rust_pair(vec!["PA1".into(), "PA2".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);
    let returned = mpin.setup_pin("digital_out", pin_params("fanhall", 0)).expect("rust setup");
    let pin = as_digital_out(returned);
    calls.borrow_mut().clear();

    py.eval("res.setup_max_duration(0.75)").expect("py max_duration");
    pin.setup_max_duration(0.75);
    py.eval("res.setup_start_value(0.0, 2.5)").expect("py start");
    pin.setup_start_value(false, true);
    py.eval("res.set_digital(12.5, True)").expect("py set_digital");
    pin.set_digital(12.5, true);
    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("py calls")), rust_calls(&calls));

    let actual = MultiPinApi::setup_cycle_time(&*mpin, 0.1, false).err().unwrap();
    check_err("cycle_time_missing", py.eval("mpin.setup_cycle_time(0.1)"), Err(actual));
    let actual = MultiPinApi::set_pwm(&*mpin, 1.0, 0.5).err().unwrap();
    check_err("set_pwm_missing", py.eval("mpin.set_pwm(1.0, 0.5)"), Err(actual));
    check("aligned", py.eval("mpin.next_aligned_print_time(3.25)"), Value::float(MultiPinApi::next_aligned_print_time(&*mpin, 3.25, 0.0)));
}

#[test]
fn fan_out_pwm() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1', 'PA2'])").expect("py constructs");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    py.exec("res = mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 0})").expect("py setup");
    py.exec("CALLS.clear()").expect("clear");
    let (printer, _pins, config, calls) = rust_pair(vec!["PA1".into(), "PA2".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);
    let returned = mpin.setup_pin("pwm", pin_params("fanhall", 0)).expect("rust setup");
    let pin = as_pwm(returned);
    calls.borrow_mut().clear();

    py.eval("res.setup_max_duration(0.75)").expect("py max_duration");
    pin.setup_max_duration(0.75);
    py.eval("res.setup_cycle_time(0.1)").expect("py cycle");
    pin.setup_cycle_time(0.1, None).expect("setup_cycle_time");
    py.eval("res.setup_start_value(0.25, 1.0)").expect("py start");
    pin.setup_start_value(0.25, 1.0);
    py.eval("res.set_pwm(12.5, 0.5)").expect("py set_pwm");
    pin.set_pwm(12.5, 0.5).expect("set_pwm");
    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("py calls")), rust_calls(&calls));

    let actual = MultiPinApi::set_digital(&*mpin, 1.0, true).err().unwrap();
    check_err("set_digital_missing", py.eval("mpin.set_digital(1.0, True)"), Err(actual));
    check("aligned", py.eval("res.next_aligned_print_time(3.25)"), Value::float(pin.next_aligned_print_time(3.25, None)));
}

#[test]
fn fan_out_before_setup_is_a_noop() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair(['PA1'])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    let (_p, _pins, config, calls) = rust_pair(vec!["PA1".into()]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");

    py.eval("mpin.setup_max_duration(1.0)").expect("py max_duration");
    MultiPinApi::setup_max_duration(&*mpin, 1.0).expect("rust max_duration");
    py.eval("mpin.set_digital(1.0, True)").expect("py set_digital");
    MultiPinApi::set_digital(&*mpin, 1.0, true).expect("rust set_digital");
    py.eval("mpin.set_pwm(1.0, 0.5)").expect("py set_pwm");
    MultiPinApi::set_pwm(&*mpin, 1.0, 0.5).expect("rust set_pwm");
    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("py calls")), rust_calls(&calls));
}

#[test]
fn duplicate_chip_registration_is_swallowed() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin_helper = mk_pair(['PA1'])").expect("py constructs");
    py.exec("pins_obj.duplicate = True").expect("duplicate");
    py.exec("printer2 = FakePrinter(); pins2 = FakePins(); pins2.duplicate = True; printer2.objects['pins'] = pins2").expect("py printer2");
    py.exec("mpin = multi_pin.load_config_prefix(FakeConfig(printer2, ['PA1']))").expect("py constructs despite duplicate");

    let (printer, pins_rs, _config, _calls) = rust_pair(vec!["PA1".into()]);
    pins_rs.duplicate.set(true);
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig { printer: printer_arc, pins: Some(vec!["PA1".into()]) });
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs despite duplicate");
    let actual_pin_list = Value::List(mpin.pin_list().iter().map(|s| Value::str(s)).collect());
    check("pin_list", py.eval("mpin.pin_list"), actual_pin_list);
}

#[test]
fn missing_pins_option_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter(); pins_obj = FakePins(); printer.objects['pins'] = pins_obj").expect("py printer");
    py.exec("config = FakeConfig(printer, None)").expect("py config");
    let (printer, _pins, _config, _calls) = rust_pair(vec!["PA1".into()]);
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig { printer: printer_arc, pins: None });
    let actual = PrinterMultiPin::new(&config).map(|_| ()).err().unwrap();
    check_err("missing_pins", py.eval("multi_pin.load_config_prefix(config)"), Err(actual));
}

#[test]
fn missing_pins_object_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter(); config = FakeConfig(printer, ['PA1'])").expect("py no pins");
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let config: Config = Arc::new(FakeConfig { printer: printer_arc, pins: Some(vec!["PA1".into()]) });
    let actual = PrinterMultiPin::new(&config).map(|_| ()).err().unwrap();
    check_err("missing_pins_object", py.eval("multi_pin.load_config_prefix(config)"), Err(actual));
}

#[test]
fn setup_pin_with_empty_list() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, pins_obj, config, mpin = mk_pair([])").expect("py constructs");
    py.exec("CALLS.clear()").expect("clear");
    py.exec("printer.objects['multi_pin fanhall'] = mpin").expect("register");
    let (printer, _pins, config, calls) = rust_pair(vec![]);
    let mpin = PrinterMultiPin::new(&config).expect("rust constructs");
    printer.insert("multi_pin fanhall", mpin.clone() as ObjectRef);

    // Python returns `self` without error even with an empty `pins` list.
    py.exec("res = mpin.setup_pin('pwm', {'pin': 'fanhall', 'invert': 0})").expect("py setup");
    check("py_setup_self", py.eval("res is mpin"), Value::Bool(true));
    py.eval("res.set_pwm(1.0, 0.5)").expect("py set_pwm on returned pin");
    let returned = mpin.setup_pin("pwm", pin_params("fanhall", 0)).expect("rust setup_pin");
    let pin = as_pwm(returned);
    pin.set_pwm(1.0, 0.5).expect("set_pwm");
    check("aligned", py.eval("res.next_aligned_print_time(3.25)"), Value::float(pin.next_aligned_print_time(3.25, None)));
    let actual_pin_type = Value::str(&mpin.pin_type().unwrap());
    check("pin_type", py.eval("mpin.pin_type"), actual_pin_type);
    check("calls", Ok(py.eval("'|'.join(CALLS)").expect("calls")), rust_calls(&calls));
    check_err("get_mcu", py.eval("mpin.get_mcu()"), MultiPinApi::get_mcu(&*mpin).map(|_| ()));
}
