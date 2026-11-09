//! Parity tests for `extras::heater_generic` (port of
//! `klippy/extras/heater_generic.py`).
//!
//! The module is three lines of Python: `load_config_prefix` loads the
//! `heaters` object and returns `pheaters.setup_heater(config)`, keeping the
//! `gcode_id=None` default. Everything is compared against the live
//! reference through `pyoracle` (CPython 3.8.2 + the frozen `klippy/`):
//! a fake printer whose `load_object('heaters')` returns a recording fake
//! on the Python side, and a fake `PrinterHeatersApi` whose
//! `setup_heater` returns a real `Heater` on the Rust side. The section
//! name passed to `load_object`, the config passed to `setup_heater`, the
//! `gcode_id` default, the returned heater identity, and both error paths
//! (the `heaters` load fails, `setup_heater` fails) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::heater_generic;
use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandHandler, GCmd, RegisterCommandOpts,
};
use klippy_api::mcu::{
    Chip, LookupPinOpts, Mcu, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, PwmPin,
    PwmPinApi,
};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, GCell};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.heater_generic as heater_generic

class _Sentinel:
    pass

_SENTINEL = _Sentinel()

class FakeHeater:
    def __init__(self, name):
        self.name = name

class FakeHeaters:
    def __init__(self):
        self.calls = []
        self.fail = None
    def setup_heater(self, config, gcode_id=None):
        self.calls.append((config.get_name(), gcode_id))
        if self.fail is not None:
            raise self.fail
        return FakeHeater(config.get_name())

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.load_calls = []
    def load_object(self, config, section, default=_SENTINEL):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        if default is _SENTINEL:
            raise self.config_error("Unable to load module '%s'" % (section,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name='heater_generic extruder1'):
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

struct DummyObj;

klippy_api::printer_object!(DummyObj);

struct FakePrinterState {
    objects: IndexMap<String, ObjectRef>,
    load_calls: Vec<String>,
    event_handlers: Vec<String>,
    reactor: Reactor,
}

struct FakePrinter {
    state: GCell<FakePrinterState>,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            state: GCell::new(FakePrinterState {
                objects: IndexMap::new(),
                load_calls: Vec::new(),
                event_handlers: Vec::new(),
                reactor: Reactor::new(false),
            }),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("debugoutput".to_string()), PyValue::None);
        PyValue::Dict(map)
    }
    fn get_reactor(&self) -> Reactor {
        self.state.borrow().reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        if st.objects.contains_key(name) {
            return Err(klippy_api::KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        st.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let st = self.state.borrow();
        if let Some(o) = st.objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(klippy_api::KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.state
            .borrow()
            .objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let mut st = self.state.borrow_mut();
        st.load_calls.push(section.to_string());
        if let Some(o) = st.objects.get(section).cloned() {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::Raise => Err(klippy_api::KlippyError::config(format!(
                "Unable to load module '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.state
            .borrow_mut()
            .event_handlers
            .push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str) -> Arc<Self> {
        let mut options = IndexMap::new();
        options.insert("min_temp".to_string(), "0.0".to_string());
        options.insert("max_temp".to_string(), "300.0".to_string());
        options.insert("heater_pin".to_string(), "PA0".to_string());
        options.insert("control".to_string(), "watermark".to_string());
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            options,
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let _ = option;
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if let Some(raw) = self.options.get(option) {
            let v: f64 = raw.parse().map_err(|_| {
                klippy_api::KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            })?;
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let _ = option;
        match opts.default {
            None => Err(klippy_api::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let key: PyValue = if let Some(raw) = self.options.get(option) {
            PyValue::Str(raw.clone())
        } else {
            match opts.default {
                None => {
                    return Err(klippy_api::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    )))
                }
                Some(d) => d.unwrap_or(PyValue::None),
            }
        };
        match choices {
            PyValue::Dict(m) => {
                if let Some(v) = m.get(&key) {
                    Ok(v.clone())
                } else {
                    Err(klippy_api::KlippyError::config(format!(
                        "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                        key.str(),
                        self.name
                    )))
                }
            }
            _ => Ok(PyValue::None),
        }
    }
    fn getlists(&self, _option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        Ok(opts
            .default
            .unwrap_or(Some(PyValue::None))
            .unwrap_or(PyValue::None))
    }
    fn getlist(&self, _option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(opts.default.unwrap())
    }
    fn getintlist(&self, _option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.unwrap())
    }
    fn getfloatlist(
        &self,
        _option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.unwrap())
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by heater_generic")
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

klippy_api::printer_object!(FakeConfig);

struct FakePwm;

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by heater_generic tests")
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: Option<bool>) -> klippy_api::KResult<()> { Ok(()) }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, _print_time: f64, _value: f64) -> klippy_api::KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakePwm; dyn PwmPinApi);

struct FakePins {
    pwm: PwmPin,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by heater_generic tests")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by heater_generic tests")
    }
    fn setup_pin(&self, pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        assert_eq!(pin_type, "pwm");
        Ok(Pin::Pwm(self.pwm.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("pin resolver is not used by heater_generic tests")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeGCode;

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
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("mutex is not used by heater_generic tests")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by heater_generic tests")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeSensor;

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        0.3
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

struct FakeHeaters {
    heater: Arc<Heater>,
    calls: GCell<Vec<(String, Option<String>)>>,
    fail: GCell<Option<String>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {}
    fn setup_heater(
        &self,
        config: &Config,
        gcode_id: Option<String>,
    ) -> KResult<Arc<Heater>> {
        self.calls
            .borrow_mut()
            .push((config.get_name(), gcode_id));
        if let Some(msg) = self.fail.borrow().clone() {
            return Err(klippy_api::KlippyError::config(msg));
        }
        Ok(self.heater.clone())
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        Err(klippy_api::KlippyError::config(format!(
            "Unknown heater '{heater_name}'"
        )))
    }
    fn setup_sensor(
        &self,
        _config: &Config,
    ) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by heater_generic tests")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn extras::heaters::SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(
        &self,
        _heater: &Arc<Heater>,
        _temp: f64,
        _wait: bool,
    ) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

// ---------------------------------------------------------------------------
// Fixture: a Rust printer with pins/gcode plus the `heaters` object, and
// the matching Python printer with the fake `heaters` object.
// ---------------------------------------------------------------------------

struct Fixture {
    printer: Arc<FakePrinter>,
    config: Config,
    heaters: Arc<FakeHeaters>,
}

fn build_fixture() -> Fixture {
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let pwm: PwmPin = Arc::new(FakePwm);
    {
        let mut st = printer.state.borrow_mut();
        st.objects
            .insert("pins".to_string(), Arc::new(FakePins { pwm }) as ObjectRef);
        st.objects
            .insert("gcode".to_string(), Arc::new(FakeGCode) as ObjectRef);
        // `Heater::new` loads these with `LookupDefault::Raise` for side
        // effects only; the returned objects are unused, so dummy objects
        // stand in for the `verify_heater` / `pid_calibrate` modules.
        st.objects.insert(
            "verify_heater extruder1".to_string(),
            Arc::new(DummyObj) as ObjectRef,
        );
        st.objects
            .insert("pid_calibrate".to_string(), Arc::new(DummyObj) as ObjectRef);
    }
    let config: Config = FakeConfig::new(printer_ref.clone(), "heater_generic extruder1");
    let sensor = Arc::new(FakeSensor);
    let heater = Heater::new(&config, sensor as Arc<dyn TemperatureSensorApi>)
        .expect("rust Heater::new");
    let heaters = Arc::new(FakeHeaters {
        heater,
        calls: GCell::new(Vec::new()),
        fail: GCell::new(None),
    });
    printer
        .state
        .borrow_mut()
        .objects
        .insert("heaters".to_string(), heaters.clone() as ObjectRef);
    // `Heater::new` above also went through `load_object` (for
    // `verify_heater` / `pid_calibrate`); clear the record so the tests
    // only see the calls `load_config_prefix` itself makes.
    printer.state.borrow_mut().load_calls.clear();
    Fixture {
        printer,
        config,
        heaters,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("heater_generic fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping heater_generic oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "heater_generic".to_string(),
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

fn check_str(label: &str, expected: pyoracle::OracleResult, actual: String) {
    check(label, expected, Value::str(&actual));
}

fn check_bool(label: &str, expected: pyoracle::OracleResult, actual: bool) {
    check(label, expected, Value::Bool(actual));
}

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<Option<ObjectRef>>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned an object but call must fail"),
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

fn load_config_prefix(fixture: &Fixture) -> KResult<Option<ObjectRef>> {
    let factory = heater_generic::module_factory();
    let load_config_prefix = factory
        .load_config_prefix
        .expect("heater_generic exposes load_config_prefix");
    load_config_prefix(&fixture.config)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn factory_shape_matches_python() {
    let factory = heater_generic::module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.heater_generic"), "entry points of extras.heater_generic");
}

#[test]
fn forwards_config_to_heaters_setup_heater() {
    let Some(mut py) = oracle() else { return };
    py.exec("ph = FakePrinter(); heaters = FakeHeaters(); ph.objects['heaters'] = heaters; pcfg = FakeConfig(ph); result = heater_generic.load_config_prefix(pcfg)")
        .expect("py setup");
    let fixture = build_fixture();
    let obj = load_config_prefix(&fixture)
        .expect("rust load_config_prefix")
        .expect("load_config_prefix returns an object");
    // The section passed to `load_object` is exactly "heaters".
    let expected_calls = py.eval("ph.load_calls");
    check(
        "load_object section",
        expected_calls,
        Value::List(
            fixture.printer.state.borrow().load_calls.iter().map(|s| Value::str(s)).collect(),
        ),
    );
    // `setup_heater` got the section config, exactly once, with the
    // `gcode_id=None` default.
    let expected_name = py.eval("heaters.calls[0][0]");
    let expected_count = py.eval("len(heaters.calls)");
    check(
        "setup_heater call count",
        expected_count,
        Value::int(fixture.heaters.calls.borrow().len() as i64),
    );
    let (recorded_name, recorded_gcode_id) = fixture.heaters.calls.borrow()[0].clone();
    check_str("setup_heater config name", expected_name, recorded_name);
    let expected_default = py.eval("heaters.calls[0][1] is None");
    check_bool("setup_heater gcode_id default", expected_default, recorded_gcode_id.is_none());
    // The returned object is the one `setup_heater` built.
    let expected_ret = py.eval("result.name");
    let rust_heater: Arc<Heater> = downcast_arc(&obj).expect("downcast to Heater");
    check_str("returned heater name", expected_ret, rust_heater.get_name());
}

#[test]
fn missing_heaters_module_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    // Same fakes, but the printer has no `heaters` object registered, so
    // `load_object('heaters')` raises exactly like the real printer
    // (`klippy.py:95-118`).
    py.exec("ph2 = FakePrinter(); pcfg2 = FakeConfig(ph2)")
        .expect("py setup");
    let expected = py.eval("heater_generic.load_config_prefix(pcfg2)");
    let fixture = build_fixture();
    fixture
        .printer
        .state
        .borrow_mut()
        .objects
        .shift_remove("heaters");
    let actual = load_config_prefix(&fixture);
    check_err("missing heaters module", expected, actual);
}

#[test]
fn setup_heater_error_propagates() {
    let Some(mut py) = oracle() else { return };
    py.exec("ph = FakePrinter(); heaters = FakeHeaters(); heaters.fail = configparser.Error(\"Heater extruder1 already registered\"); ph.objects['heaters'] = heaters; pcfg = FakeConfig(ph)")
        .expect("py setup");
    let expected = py.eval("heater_generic.load_config_prefix(pcfg)");
    let fixture = build_fixture();
    *fixture.heaters.fail.borrow_mut() = Some("Heater extruder1 already registered".to_string());
    let actual = load_config_prefix(&fixture);
    check_err("setup_heater error", expected, actual);
}
