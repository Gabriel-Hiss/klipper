//! Parity tests for `extras::thermistor` (port of
//! `klippy/extras/thermistor.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`; each test skips with a message when
//! `$KLIPPY_PYTHON` is unset). Both sides read their options through the real
//! config wrapper (`configfile.ConfigWrapper` / `klippy_core::configfile`)
//! over the same config text, and both run the real `heaters` object
//! (`heaters.PrinterHeaters` / `extras::heaters::PrinterHeaters`), so a
//! `[extruder] sensor_type: ...` resolves through `heaters.setup_sensor` and
//! loads `temperature_sensors.cfg` exactly as klippy does. Only the printer,
//! the pins, `query_adc`, `error_mcu` and `gcode` are fakes; the fake
//! `load_object` mirrors `klippy.Printer.load_object` (module lookup by the
//! section's first word, `load_config` / `load_config_prefix` on
//! `config.getsection(section)`).
//!
//! Compared per scenario, as one trace of `[label, result]` entries where a
//! result is `['ok', value]` or `['err', class, message]`: the `[thermistor]`
//! sections loaded, `setup_sensor`, the sensor factory names `heaters`
//! registered (the default sensors included), the pin and `query_adc`
//! registrations, the `error_mcu` clarify registration, the ADC sampling
//! arguments after `setup_minmax` (so `calc_adc` at the bounds), `adc_callback`
//! temperatures over an ADC grid that includes the `[0.00001, 0.99999]` clamp
//! edges and values outside them, `calc_adc` values over a temperature grid
//! (through `setup_minmax(t, t)`), the diagnostics clarify string, and the
//! log lines. Scenarios cover the default thermistors of
//! `temperature_sensors.cfg` (including the KE `EPCOS 100K B57560G104F`),
//! custom beta and three-point definitions (unsorted, a set that falls back
//! to beta with its warning), `pullup_resistor` / `inline_resistor`, config
//! errors, math errors while setting the coefficients, and the error paths of
//! `calc_temp` / `calc_adc` (an `inline_resistor` larger than the computed
//! resistance, `calc_adc` overflow). A separate test compares the parameters
//! `CustomThermistor` derives (option order, bounds, the `sorted(...)` of the
//! three points including nan values), another the `AdcConvert` error
//! mapping of the converter, and one `Heater.__init__` with a sensor whose
//! `setup_minmax` raises.
//!
//! Python exceptions that leave the ADC callback surface as panics in the
//! port (the serial dispatcher logs them like `serialhdl.py:65`); the traces
//! compare them as `Class: message`.

use extras::adc_temperature::AdcConvert;
use extras::error_mcu::{ClarifyCallback, PrinterMCUErrorApi};
use extras::heaters::{Heater, PrinterHeaters, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi};
use extras::query_adc::QueryAdcApi;
use extras::thermistor::{self, CustomThermistor, ThermistorParams};
use extras_adc_temperature::thermistor::Thermistor;
use indexmap::IndexMap;
use klippy_api::config::{Config, DeprecateGcodeOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, Pin, PinsApi};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer, PrinterApi};
use klippy_api::GCell;
use klippy_core::configfile::{ConfigFileReader, ConfigWrapper};
use pycompat::PyValue;
use recording::{assert_equal, Record, RecordKind, Value};
use std::cell::RefCell;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Arc, Once};

// ---------------------------------------------------------------------------
// Comparison helpers
// ---------------------------------------------------------------------------

fn dummy_record(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.thermistor".to_string(),
        scenario: "oracle".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: &Value, actual: &Value) {
    let rec = dummy_record(label);
    if let Err(d) = assert_equal(&rec, "ret", expected, actual) {
        panic!(
            "parity divergence in {label} at {path}: expected {expected}, actual {actual}",
            path = d.path,
            expected = d.expected,
            actual = d.actual
        );
    }
}

/// Compares two traces entry by entry (first divergence wins), then lengths.
fn check_trace(label: &str, expected: &Value, actual: &Value) {
    let (Value::List(expected_items), Value::List(actual_items)) = (expected, actual) else {
        panic!("{label}: a trace is a list: oracle {expected:?}, port {actual:?}");
    };
    for (i, (e, a)) in expected_items.iter().zip(actual_items.iter()).enumerate() {
        let entry = match e {
            Value::List(parts) => parts.first().map(Value::render).unwrap_or_default(),
            _ => String::new(),
        };
        check(&format!("{label} #{i} {entry}"), e, a);
    }
    assert_eq!(
        expected_items.len(),
        actual_items.len(),
        "{label}: trace length (oracle {}, port {})",
        expected.render(),
        actual.render()
    );
}

fn ok(value: Value) -> Value {
    Value::List(vec![Value::str("ok"), value])
}

fn err_result(class: &str, message: &str) -> Value {
    Value::List(vec![Value::str("err"), Value::str(class), Value::str(message)])
}

fn err_value(e: &KlippyError) -> Value {
    err_result(e.class_name(), e.message())
}

fn entry(label: &str, result: Value) -> Value {
    Value::List(vec![Value::str(label), result])
}

/// `capture(fn)` of the oracle prelude.
fn capture(f: impl FnOnce() -> KResult<Value>) -> Value {
    match f() {
        Ok(v) => ok(v),
        Err(e) => err_value(&e),
    }
}

/// Like [`capture`] for code that reports a Python exception as a panic with
/// the text `Class: message` (the ADC callback has no error channel).
fn capture_panic(f: impl FnOnce() -> Value) -> Value {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(v) => ok(v),
        Err(payload) => {
            let text = if let Some(s) = payload.downcast_ref::<String>() {
                s.clone()
            } else if let Some(s) = payload.downcast_ref::<&str>() {
                (*s).to_string()
            } else {
                panic!("panic payload is not a string")
            };
            let (class, message) = text.split_once(": ").expect("panic text is `Class: message`");
            err_result(class, message)
        }
    }
}

fn py_float(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x == f64::INFINITY {
        "float('inf')".to_string()
    } else if x == f64::NEG_INFINITY {
        "float('-inf')".to_string()
    } else {
        format!("{x:?}")
    }
}

fn py_floats(values: &[f64]) -> String {
    format!("[{}]", values.iter().map(|x| py_float(*x)).collect::<Vec<_>>().join(", "))
}

fn py_pairs(values: &[(f64, f64)]) -> String {
    format!(
        "[{}]",
        values.iter().map(|(a, b)| format!("({}, {})", py_float(*a), py_float(*b))).collect::<Vec<_>>().join(", ")
    )
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn start_oracle() -> Option<pyoracle::Oracle> {
    let oracle = pyoracle::Oracle::start();
    if oracle.is_none() {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
    }
    oracle
}

// ---------------------------------------------------------------------------
// Log capture: the port logs through `klippy_api::log`, the reference through
// the root logger; both end up as `LEVEL: message` lists.
// ---------------------------------------------------------------------------

thread_local! {
    static LOGS: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}
static LOG_SINK: Once = Once::new();

fn install_log_sink() {
    LOG_SINK.call_once(|| {
        let installed = klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.with(|logs| logs.borrow_mut().push(format!("{}: {}", rec.level.name(), rec.message)));
        }));
        assert!(installed, "another log sink was installed first");
    });
}

fn take_logs() -> Vec<String> {
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

fn logs_value() -> Value {
    Value::List(take_logs().iter().map(|l| Value::str(l)).collect())
}

// ---------------------------------------------------------------------------
// Rust fakes of the contract traits
// ---------------------------------------------------------------------------

/// The modules `temperature_sensors.cfg` and the scenarios load.
fn module_factory(name: &str) -> Option<ModuleFactory> {
    Some(match name {
        "adc_temperature" => extras::adc_temperature::module_factory(),
        "aht10" => extras::aht10::module_factory(),
        "bme280" => extras::bme280::module_factory(),
        "ds18b20" => extras::ds18b20::module_factory(),
        "htu21d" => extras::htu21d::module_factory(),
        "lm75" => extras::lm75::module_factory(),
        "sht3x" => extras::sht3x::module_factory(),
        "spi_temperature" => extras::spi_temperature::module_factory(),
        "temperature_combined" => extras::temperature_combined::module_factory(),
        "temperature_host" => extras::temperature_host::module_factory(),
        "temperature_mcu" => extras::temperature_mcu::module_factory(),
        "thermistor" => thermistor::module_factory(),
        _ => return None,
    })
}

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> reactor::Reactor {
        reactor::Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("not used")
    }
    fn is_shutdown(&self) -> bool {
        panic!("not used")
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("not used")
    }
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("not used")
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        panic!("not used")
    }
    /// `klippy.Printer.load_object`: a loaded object is returned as is; else
    /// the module named by the section's first word runs `load_config` (one
    /// word) or `load_config_prefix` (more) on `config.getsection(section)`.
    fn load_object(&self, config: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let loaded = self.objects.borrow().get(section).cloned();
        if let Some(obj) = loaded {
            return Ok(Some(obj));
        }
        let parts: Vec<&str> = section.split_whitespace().collect();
        let init = parts
            .first()
            .and_then(|module| module_factory(module))
            .and_then(|factory| if parts.len() == 1 { factory.load_config } else { factory.load_config_prefix });
        let Some(init) = init else {
            return match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!("Unable to load module '{section}'"))),
                LookupDefault::ReturnNone => Ok(None),
            };
        };
        let obj = init(&config.getsection(section))?;
        if let Some(obj) = &obj {
            self.objects.borrow_mut().insert(section.to_string(), obj.clone());
        }
        Ok(obj)
    }
    fn run(&self) -> Option<String> {
        panic!("not used")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("not used")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("not used")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("not used")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("not used")
    }
    fn request_exit(&self, _result: &str) {
        panic!("not used")
    }
}

struct FakeAdcPin {
    callback: GCell<Option<AdcCallback>>,
    sample_args: GCell<Option<(f64, AdcSampleOpts)>>,
    last_value: GCell<(f64, f64)>,
}

impl FakeAdcPin {
    fn new() -> Arc<Self> {
        Arc::new(FakeAdcPin { callback: GCell::new(None), sample_args: GCell::new(None), last_value: GCell::new((0., 0.)) })
    }

    /// Delivers `samples` to the callback the sensor installed.
    fn drive(&self, samples: Vec<(f64, f64)>) {
        let mut slot = self.callback.borrow_mut();
        let callback = slot.as_mut().expect("the sensor installed an ADC callback");
        callback(samples);
    }

    fn set_last(&self, value: (f64, f64)) {
        *self.last_value.borrow_mut() = value;
    }
}

impl AdcPinApi for FakeAdcPin {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("not used")
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        *self.sample_args.borrow_mut() = Some((report_time, opts));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        *self.last_value.borrow()
    }
}

struct FakePins {
    adcs: GCell<Vec<(String, Arc<FakeAdcPin>)>>,
    setup_calls: GCell<Vec<(String, String)>>,
}

impl FakePins {
    fn adc(&self, pin: &str) -> Arc<FakeAdcPin> {
        self.adcs.borrow().iter().rev().find(|(n, _)| n == pin).map(|(_, a)| Arc::clone(a)).expect("adc pin exists")
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<klippy_api::mcu::PinParams> {
        panic!("not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<klippy_api::mcu::PinParams> {
        panic!("not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setup_calls.borrow_mut().push((pin_type.to_string(), pin_desc.to_string()));
        let adc = FakeAdcPin::new();
        self.adcs.borrow_mut().push((pin_desc.to_string(), Arc::clone(&adc)));
        Ok(Pin::Adc(adc))
    }
    fn reset_pin_sharing(&self, _pin_params: klippy_api::mcu::PinParams) -> klippy_api::KResult<()> { panic!("not used") }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("not used")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeQueryAdc {
    registered: GCell<Vec<(String, AdcPin)>>,
}

impl QueryAdcApi for FakeQueryAdc {
    fn register_adc(&self, name: &str, adc: AdcPin) {
        self.registered.borrow_mut().push((name.to_string(), adc));
    }
}

klippy_api::printer_object!(FakeQueryAdc; dyn QueryAdcApi);

struct FakeErrorMcu {
    clarifies: GCell<Vec<(String, ClarifyCallback)>>,
}

impl PrinterMCUErrorApi for FakeErrorMcu {
    fn add_clarify(&self, msg: &str, callback: ClarifyCallback) {
        self.clarifies.borrow_mut().push((msg.to_string(), callback));
    }
}

klippy_api::printer_object!(FakeErrorMcu; dyn PrinterMCUErrorApi);

/// `PrinterHeaters` handling every call, recording the sensor factory names
/// in registration order (the real object keeps its factories private).
struct RecordingHeaters {
    inner: Arc<PrinterHeaters>,
    names: GCell<Vec<String>>,
}

impl PrinterHeatersApi for RecordingHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        let mut names = self.names.borrow_mut();
        if !names.iter().any(|n| n == sensor_type) {
            names.push(sensor_type.to_string());
        }
        drop(names);
        self.inner.add_sensor_factory(sensor_type, factory);
    }
    fn setup_heater(&self, config: &Config, gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        self.inner.setup_heater(config, gcode_id)
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.inner.get_all_heaters()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        self.inner.lookup_heater(heater_name)
    }
    fn setup_sensor(&self, config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        self.inner.setup_sensor(config)
    }
    fn register_sensor(&self, config: &Config, psensor: Arc<dyn SensorApi>, gcode_id: Option<String>) -> KResult<()> {
        self.inner.register_sensor(config, psensor, gcode_id)
    }
    fn register_monitor(&self, config: &Config) {
        self.inner.register_monitor(config);
    }
    fn turn_off_all_heaters(&self, print_time: f64) {
        self.inner.turn_off_all_heaters(print_time);
    }
    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        self.inner.set_temperature(heater, temp, wait)
    }
}

klippy_api::printer_object!(RecordingHeaters; dyn PrinterHeatersApi);

/// The `gcode` object `PrinterHeaters.__init__` registers its commands with.
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
        panic!("not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("not used")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("not used")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("not used")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("not used")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("not used")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("not used")
    }
    fn create_gcode_command(&self, _command: &str, _commandline: &str, _params: &IndexMap<String, String>) -> GCmd {
        panic!("not used")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("not used")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("not used")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("not used")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("not used")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

/// The `configfile` object: `read_config` as `PrinterConfig.read_config`
/// (real parse, a `'printer'` wrapper).
struct FakePrinterConfig {
    printer: Printer,
}

impl PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, filename: &str) -> KResult<Config> {
        let mut reader = ConfigFileReader::new();
        let text = reader.read_config_file(filename).map_err(|e| KlippyError::config(e.to_string()))?;
        let fileconfig =
            ConfigFileReader::build_fileconfig(&text, filename).map_err(|e| KlippyError::config(e.to_string()))?;
        Ok(Arc::new(ConfigWrapper::new(
            self.printer.clone(),
            Arc::new(fileconfig),
            Arc::new(GCell::new(IndexMap::new())),
            "printer".to_string(),
        )))
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("not used")
    }
    fn log_config(&self, _config: &Config) {
        panic!("not used")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("not used")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("not used")
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("not used")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {
        panic!("not used")
    }
    fn deprecate_mcu_code(&self, _mcu: &klippy_api::mcu::Mcu, _feature: &str, _msg: Option<String>) {
        panic!("not used")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("not used")
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> {
        panic!("not used")
    }
    fn remove_section(&self, _section: &str) {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);

/// The real `ConfigWrapper` over `text`, as `PrinterConfig` hands it to a
/// module.
fn make_root(printer: Printer, text: &str) -> Config {
    let fileconfig = ConfigFileReader::build_fileconfig(text, "printer.cfg").expect("parse config text");
    Arc::new(ConfigWrapper::new(
        printer,
        Arc::new(fileconfig),
        Arc::new(GCell::new(IndexMap::new())),
        "printer".to_string(),
    ))
}

/// A printer with the fakes and a real `PrinterHeaters` over `text`.
struct World {
    printer: Arc<FakePrinter>,
    pins: Arc<FakePins>,
    query: Arc<FakeQueryAdc>,
    error_mcu: Arc<FakeErrorMcu>,
    heaters: Arc<RecordingHeaters>,
    root: Config,
}

fn new_world(text: &str) -> World {
    install_log_sink();
    // `heaters` reads the bundled `temperature_sensors.cfg` from the
    // recorded klippy source directory.
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root");
    klippy_api::paths::set_klippy_dir(&repo.join("klippy").to_string_lossy());
    take_logs();
    let printer = Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()) });
    let printer_api: Printer = printer.clone();
    let pins = Arc::new(FakePins { adcs: GCell::new(Vec::new()), setup_calls: GCell::new(Vec::new()) });
    let query = Arc::new(FakeQueryAdc { registered: GCell::new(Vec::new()) });
    let error_mcu = Arc::new(FakeErrorMcu { clarifies: GCell::new(Vec::new()) });
    printer.add("pins", pins.clone() as ObjectRef);
    printer.add("query_adc", query.clone() as ObjectRef);
    printer.add("error_mcu", error_mcu.clone() as ObjectRef);
    printer.add("gcode", Arc::new(FakeGCode) as ObjectRef);
    printer.add("configfile", Arc::new(FakePrinterConfig { printer: printer_api.clone() }) as ObjectRef);
    let root = make_root(printer_api, text);
    let inner = PrinterHeaters::new(&root.getsection("heaters")).expect("heaters");
    let heaters = Arc::new(RecordingHeaters { inner, names: GCell::new(Vec::new()) });
    printer.add("heaters", heaters.clone() as ObjectRef);
    World { printer, pins, query, error_mcu, heaters, root }
}

// ---------------------------------------------------------------------------
// Python side: the same fakes around the real configfile and heaters
// ---------------------------------------------------------------------------

const ORACLE_SETUP: &str = r#"
import importlib
import logging
import configfile as _cf
import extras.heaters as _hmod
import extras.thermistor as _tmod

PY_LOGS = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        PY_LOGS.append("%s: %s" % (record.levelname, record.getMessage()))
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.WARNING)

_SENT = object()

def capture(fn):
    try:
        return ['ok', fn()]
    except Exception as e:
        return ['err', type(e).__name__, str(e)]

class FakeAdc:
    def __init__(self, pin):
        self.pin = pin
        self.callback = None
        self.sample_args = None
        self.last = (0., 0.)
    def setup_adc_callback(self, cb):
        self.callback = cb
    def setup_adc_sample(self, report_time, sample_time=0., sample_count=1,
                         batch_num=1, minval=0., maxval=1., range_check_count=0):
        self.sample_args = (report_time, sample_time, sample_count, batch_num,
                            minval, maxval, range_check_count)
    def get_last_value(self):
        return self.last
    def get_mcu(self):
        return None

class FakePins:
    def __init__(self):
        self.adcs = {}
        self.setup_calls = []
    def setup_pin(self, pin_type, pin_desc):
        self.setup_calls.append((pin_type, pin_desc))
        adc = FakeAdc(pin_desc)
        self.adcs[pin_desc] = adc
        return adc

class FakeQuery:
    def __init__(self):
        self.reg = []
    def register_adc(self, name, adc):
        self.reg.append(name)

class FakeErrorMcu:
    def __init__(self):
        self.clarifies = []
    def add_clarify(self, msg, cb):
        self.clarifies.append((msg, cb))

class FakeGCode:
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        pass
    def register_mux_command(self, cmd, key, value, func, desc=None):
        pass

class FakeConfigfile:
    def __init__(self, printer):
        self.printer = printer
    def read_config(self, filename):
        cfgrdr = _cf.ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        fileconfig = cfgrdr.build_fileconfig(data, filename)
        return _cf.ConfigWrapper(self.printer, fileconfig, {}, 'printer')

class FakePrinter:
    config_error = _cf.error
    def __init__(self):
        self.objects = {}
    def get_start_args(self):
        return {}
    def register_event_handler(self, event, cb):
        pass
    def lookup_object(self, name, default=_SENT):
        if name in self.objects:
            return self.objects[name]
        if default is _SENT:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_SENT):
        if section in self.objects:
            return self.objects[section]
        module_parts = section.split()
        try:
            module = importlib.import_module('extras.' + module_parts[0])
        except ImportError:
            module = None
        init_func = None
        if module is not None:
            init_func = getattr(module, 'load_config' if len(module_parts) == 1
                                else 'load_config_prefix', None)
        if init_func is None:
            if default is not _SENT:
                return default
            raise self.config_error("Unable to load module '%s'" % (section,))
        self.objects[section] = init_func(config.getsection(section))
        return self.objects[section]

def make_root(printer, text):
    fileconfig = _cf.ConfigFileReader().build_fileconfig(text, 'printer.cfg')
    return _cf.ConfigWrapper(printer, fileconfig, {}, 'printer')

def scenario(text, load, heater, pin, minmax, adcs, temps, lasts, heater_new):
    del PY_LOGS[:]
    printer = FakePrinter()
    pins = FakePins()
    query = FakeQuery()
    err_mcu = FakeErrorMcu()
    printer.objects.update({'pins': pins, 'query_adc': query, 'error_mcu': err_mcu,
                            'gcode': FakeGCode(), 'configfile': FakeConfigfile(printer)})
    root = make_root(printer, text)
    heaters = _hmod.load_config(root.getsection('heaters'))
    printer.objects['heaters'] = heaters
    out = []
    for sec in load:
        out.append(['load ' + sec, capture(lambda: (printer.load_object(root, sec), None)[1])])
    hcfg = root.getsection(heater)
    box = []
    def make():
        box.append(heaters.setup_sensor(hcfg))
    out.append(['setup_sensor', capture(make)])
    if not box:
        out.append(['logs', list(PY_LOGS)])
        return out
    sensor = box[0]
    out.append(['factories', list(heaters.sensor_factories.keys())])
    out.append(['pin_calls', [list(c) for c in pins.setup_calls]])
    out.append(['query_adc', list(query.reg)])
    out.append(['clarify_registered', [m for m, cb in err_mcu.clarifies]])
    adc = pins.adcs[pin]
    for mn, mx in minmax:
        out.append(['setup_minmax', capture(lambda: sensor.setup_minmax(mn, mx))])
        out.append(['sample_args', None if adc.sample_args is None else list(adc.sample_args)])
    seen = []
    sensor.setup_callback(lambda t, temp: seen.append([t, temp]))
    for v in adcs:
        del seen[:]
        out.append(['callback', capture(lambda: (adc.callback([(1.0, 0.5), (2.5, v)]), [list(x) for x in seen])[1])])
    for last in lasts:
        adc.last = (1.0, last)
        out.append(['clarify', capture(lambda: [cb('ADC out of range', {}) for m, cb in err_mcu.clarifies])])
    for t in temps:
        out.append(['calc_adc', capture(lambda: (sensor.setup_minmax(t, t), adc.sample_args[4])[1])])
    if heater_new:
        out.append(['heater_new', capture(lambda: (_hmod.Heater(hcfg, sensor), None)[1])])
    out.append(['logs', list(PY_LOGS)])
    return out

def custom_params(text, section):
    printer = FakePrinter()
    root = make_root(printer, text)
    def mk():
        c = _tmod.CustomThermistor(root.getsection(section))
        return [c.name, dict(c.params)]
    return capture(mk)

def conv_case(inline, t1, r1, beta, op, x):
    def run():
        th = _tmod.Thermistor(4700., inline)
        th.setup_coefficients_beta(t1, r1, beta)
        return getattr(th, op)(x)
    return capture(run)
"#;

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

/// A `CustomThermistor` parameter case: label, thermistor name, options.
type ParamsCase = (String, String, Vec<(String, String)>);

/// One heater scenario: the config text, the `[thermistor ...]` sections
/// loaded first, the heater section whose `sensor_type` is resolved, and the
/// inputs of each trace stage.
struct Scenario {
    label: String,
    text: String,
    load: Vec<String>,
    heater: &'static str,
    pin: &'static str,
    minmax: Vec<(f64, f64)>,
    adcs: Vec<f64>,
    temps: Vec<f64>,
    lasts: Vec<f64>,
    heater_new: bool,
}

/// ADC readings: below, at and above both clamp edges, and nan.
fn adc_grid() -> Vec<f64> {
    let mut v = vec![
        0.0, 0.000001, 0.0000099, 0.00001, 0.0001, 0.001, 0.01, 0.05, 0.1, 0.25, 0.5, 0.75, 0.9, 0.99, 0.999, 0.99999,
        0.999995, 1.0, -0.5, 2.0,
    ];
    v.push(f64::NAN);
    v
}

/// Temperatures around and below the Kelvin limit and far above any table.
fn temp_grid() -> Vec<f64> {
    vec![
        -1000.0, -300.0, -273.15, -273.14, -273.0, -270.0, -200.0, -50.0, 0.0, 25.0, 100.0, 150.0, 250.0, 300.0, 500.0,
        1000.0, 5000.0,
    ]
}

/// `setup_minmax` bounds, the last ones in range order so the clarify stage
/// sees `(0, 300)`.
fn minmax_grid() -> Vec<(f64, f64)> {
    vec![(300.0, 0.0), (10.0, 260.0), (-50.0, 500.0), (25.0, 25.0), (0.0, 300.0)]
}

/// ADC readings the clarify stage reads back as the last sample.
fn last_grid() -> Vec<f64> {
    vec![0.0001, 0.01, 0.5, 0.99999, 1.0, 2.0]
}

fn section_text(header: &str, opts: &[(&str, &str)]) -> String {
    let mut text = format!("[{header}]\n");
    for (key, value) in opts {
        text.push_str(&format!("{key}: {value}\n"));
    }
    text
}

fn custom_section(name: &str, opts: &[(&str, &str)]) -> String {
    section_text(&format!("thermistor {name}"), opts)
}

/// `[extruder]` with the options the scenarios share, then `extra`.
fn heater_section(sensor_type: &str, extra: &[(&str, &str)]) -> String {
    let mut opts = vec![("sensor_type", sensor_type), ("sensor_pin", "PA0"), ("min_temp", "0"), ("max_temp", "300")];
    opts.extend_from_slice(extra);
    section_text("extruder", &opts)
}

fn scenario(label: &str, text: String, load: &[&str]) -> Scenario {
    Scenario {
        label: label.to_string(),
        text,
        load: load.iter().map(|s| s.to_string()).collect(),
        heater: "extruder",
        pin: "PA0",
        minmax: minmax_grid(),
        adcs: adc_grid(),
        temps: temp_grid(),
        lasts: last_grid(),
        heater_new: false,
    }
}

const BETA_OPTS: &[(&str, &str)] = &[("temperature1", "25"), ("resistance1", "100000"), ("beta", "3950")];
const POINTS_UNSORTED_OPTS: &[(&str, &str)] = &[
    ("temperature1", "250"),
    ("resistance1", "226.15"),
    ("temperature2", "25"),
    ("resistance2", "100000"),
    ("temperature3", "150"),
    ("resistance3", "1641.9"),
];
/// Three points whose Steinhart-Hart `c3` is not positive: beta fallback.
const POINTS_BETA_FALLBACK_OPTS: &[(&str, &str)] = &[
    ("temperature1", "25"),
    ("resistance1", "100000"),
    ("temperature2", "100"),
    ("resistance2", "1500"),
    ("temperature3", "250"),
    ("resistance3", "100"),
];

/// The default thermistors of `temperature_sensors.cfg`, which the oracle's
/// `heaters.PrinterHeaters.load_config` registers.
const DEFAULT_THERMISTORS: &[&str] = &[
    "ATC Semitec 104GT-2",
    "ATC Semitec 104NT-4-R025H42G",
    "EPCOS 100K B57560G104F",
    "Generic 3950",
    "SliceEngineering 450",
    "TDK NTCG104LH104JT1",
    "Honeywell 100K 135-104LAG-J01",
    "NTC 100K MGB18-104F39050L32",
];

fn run_oracle_scenario(py: &mut pyoracle::Oracle, s: &Scenario) -> Value {
    let load: Vec<&str> = s.load.iter().map(String::as_str).collect();
    let call = format!(
        "RES = scenario({:?}, {:?}, {:?}, {:?}, {}, {}, {}, {}, {})",
        s.text,
        load,
        s.heater,
        s.pin,
        py_pairs(&s.minmax),
        py_floats(&s.adcs),
        py_floats(&s.temps),
        py_floats(&s.lasts),
        py_bool(s.heater_new),
    );
    py.exec(&call).unwrap_or_else(|e| panic!("{}: oracle scenario raised {}: {}", s.label, e.class_name, e.message));
    py.eval("RES").unwrap_or_else(|e| panic!("{}: oracle trace {}: {}", s.label, e.class_name, e.message))
}

fn sample_args_value(adc: &FakeAdcPin) -> Value {
    match &*adc.sample_args.borrow() {
        None => Value::Null,
        Some((report_time, opts)) => Value::List(vec![
            Value::float(*report_time),
            Value::float(opts.sample_time),
            Value::int(opts.sample_count),
            Value::int(opts.batch_num),
            Value::float(opts.minval),
            Value::float(opts.maxval),
            Value::int(opts.range_check_count),
        ]),
    }
}

fn strings_value(items: impl IntoIterator<Item = String>) -> Value {
    Value::List(items.into_iter().map(Value::Str).collect())
}

fn run_rust_scenario(s: &Scenario) -> Value {
    let w = new_world(&s.text);
    let mut out: Vec<Value> = Vec::new();
    for section in &s.load {
        let result = capture(|| {
            w.printer.load_object(&w.root, section, LookupDefault::Raise)?;
            Ok(Value::Null)
        });
        out.push(entry(&format!("load {section}"), result));
    }
    let hcfg = w.root.getsection(s.heater);
    let sensor = match w.heaters.setup_sensor(&hcfg) {
        Ok(sensor) => {
            out.push(entry("setup_sensor", ok(Value::Null)));
            sensor
        }
        Err(e) => {
            out.push(entry("setup_sensor", err_value(&e)));
            out.push(entry("logs", logs_value()));
            return Value::List(out);
        }
    };
    out.push(entry("factories", strings_value(w.heaters.names.borrow().iter().cloned())));
    out.push(entry(
        "pin_calls",
        Value::List(
            w.pins
                .setup_calls
                .borrow()
                .iter()
                .map(|(kind, desc)| Value::List(vec![Value::str(kind), Value::str(desc)]))
                .collect(),
        ),
    ));
    out.push(entry("query_adc", strings_value(w.query.registered.borrow().iter().map(|(n, _)| n.clone()))));
    out.push(entry(
        "clarify_registered",
        strings_value(w.error_mcu.clarifies.borrow().iter().map(|(m, _)| m.clone())),
    ));
    let adc = w.pins.adc(s.pin);
    for &(min_temp, max_temp) in &s.minmax {
        out.push(entry("setup_minmax", capture(|| sensor.setup_minmax(min_temp, max_temp).map(|()| Value::Null))));
        out.push(entry("sample_args", sample_args_value(&adc)));
    }
    let seen: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let sink = seen.clone();
    sensor.setup_callback(Box::new(move |read_time, temp| sink.borrow_mut().push((read_time, temp))));
    for &v in &s.adcs {
        seen.borrow_mut().clear();
        let result = capture_panic(|| {
            adc.drive(vec![(1.0, 0.5), (2.5, v)]);
            let temps: Vec<Value> = seen
                .borrow()
                .iter()
                .map(|&(t, temp)| Value::List(vec![Value::float(t), Value::float(temp)]))
                .collect();
            Value::List(temps)
        });
        out.push(entry("callback", result));
    }
    let clarify_callbacks: Vec<ClarifyCallback> =
        w.error_mcu.clarifies.borrow().iter().map(|(_, cb)| cb.clone()).collect();
    for &last in &s.lasts {
        adc.set_last((1.0, last));
        let result = capture(|| {
            let mut items = Vec::new();
            for cb in &clarify_callbacks {
                items.push(match cb("ADC out of range", &PyValue::Dict(IndexMap::new()))? {
                    Some(message) => Value::Str(message),
                    None => Value::Null,
                });
            }
            Ok(Value::List(items))
        });
        out.push(entry("clarify", result));
    }
    for &t in &s.temps {
        let result = capture(|| {
            sensor.setup_minmax(t, t)?;
            let sample = adc.sample_args.borrow();
            let (_, opts) = sample.as_ref().expect("setup_minmax set the sample arguments");
            Ok(Value::float(opts.minval))
        });
        out.push(entry("calc_adc", result));
    }
    if s.heater_new {
        out.push(entry("heater_new", capture(|| Heater::new(&hcfg, sensor.clone()).map(|_| Value::Null))));
    }
    out.push(entry("logs", logs_value()));
    Value::List(out)
}

fn compare_scenarios(py: &mut pyoracle::Oracle, scenarios: &[Scenario]) {
    for s in scenarios {
        let expected = run_oracle_scenario(py, s);
        let actual = run_rust_scenario(s);
        check_trace(&s.label, &expected, &actual);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

/// The KE config path: `[extruder] sensor_type: EPCOS 100K B57560G104F`
/// resolves through `heaters` (which loads `temperature_sensors.cfg`) to the
/// factory the `thermistor` module registered, and the same sensor type with
/// every other default thermistor.
#[test]
fn default_thermistors_through_heaters_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let mut scenarios = Vec::new();
    for name in DEFAULT_THERMISTORS {
        scenarios.push(scenario(&format!("default {name}"), heater_section(name, &[]), &[]));
    }
    compare_scenarios(&mut py, &scenarios);
}

/// The registered sensor factory names are the oracle's, in order: the
/// default sensors of every module `temperature_sensors.cfg` loads and the
/// thermistors, with nothing else.
#[test]
fn registered_factory_names_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let s = Scenario {
        minmax: Vec::new(),
        adcs: Vec::new(),
        temps: Vec::new(),
        lasts: Vec::new(),
        ..scenario("factory names", heater_section("EPCOS 100K B57560G104F", &[]), &[])
    };
    let expected = run_oracle_scenario(&mut py, &s);
    let actual = run_rust_scenario(&s);
    check_trace("factory names", &expected, &actual);
    // The trace holds the names: every default thermistor is among them.
    let Value::List(entries) = &actual else { panic!("trace is a list") };
    let names = entries
        .iter()
        .find_map(|e| match e {
            Value::List(parts) if parts.first() == Some(&Value::str("factories")) => Some(parts[1].clone()),
            _ => None,
        })
        .expect("factories entry");
    let Value::List(names) = names else { panic!("factories is a list") };
    for name in DEFAULT_THERMISTORS {
        assert!(names.contains(&Value::str(name)), "{name} is registered");
    }
}

/// Custom `[thermistor <name>]` sections and the `PrinterThermistor` options.
#[test]
fn custom_thermistors_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let mut scenarios = Vec::new();

    // Beta definition, and the same with the pullup / inline resistor options.
    scenarios.push(scenario(
        "beta",
        format!("{}{}", custom_section("beta one", BETA_OPTS), heater_section("beta one", &[])),
        &["thermistor beta one"],
    ));
    scenarios.push(scenario(
        "beta pullup inline",
        format!(
            "{}{}",
            custom_section("beta one", BETA_OPTS),
            heater_section("beta one", &[("pullup_resistor", "10000"), ("inline_resistor", "100")])
        ),
        &["thermistor beta one"],
    ));

    // Three points given out of order, sorted by temperature.
    scenarios.push(scenario(
        "points unsorted",
        format!("{}{}", custom_section("unsorted", POINTS_UNSORTED_OPTS), heater_section("unsorted", &[])),
        &["thermistor unsorted"],
    ));

    // Three points that fall back to beta: the warning names the heater.
    scenarios.push(scenario(
        "points beta fallback",
        format!("{}{}", custom_section("fallback", POINTS_BETA_FALLBACK_OPTS), heater_section("fallback", &[])),
        &["thermistor fallback"],
    ));

    // An inline resistor above the computed resistance: calc_temp raises for
    // small readings, so the callback, and the clarify message (the reference
    // raises NameError from its `except e:`), report errors.
    scenarios.push(scenario(
        "inline resistor 1000",
        heater_section("EPCOS 100K B57560G104F", &[("inline_resistor", "1000")]),
        &[],
    ));
    scenarios.push(scenario(
        "inline resistor huge",
        heater_section("EPCOS 100K B57560G104F", &[("inline_resistor", "100000000")]),
        &[],
    ));
    scenarios.push(scenario(
        "pullup",
        heater_section("Generic 3950", &[("pullup_resistor", "2200")]),
        &[],
    ));

    // `Heater.__init__` with a sensor whose `calc_adc` overflows at min_temp.
    let overflow_heater = section_text(
        "extruder",
        &[
            ("sensor_type", "beta one"),
            ("sensor_pin", "PA0"),
            ("min_temp", "-273"),
            ("max_temp", "300"),
        ],
    );
    let mut overflow = scenario(
        "heater init overflow",
        format!("{}{}", custom_section("beta one", BETA_OPTS), overflow_heater),
        &["thermistor beta one"],
    );
    overflow.heater_new = true;
    scenarios.push(overflow);

    compare_scenarios(&mut py, &scenarios);
}

/// Config errors and math errors while the sensor is created.
#[test]
fn thermistor_errors_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let mut scenarios = Vec::new();

    for (label, extra) in [
        ("pullup zero", ("pullup_resistor", "0")),
        ("pullup negative", ("pullup_resistor", "-4700")),
        ("pullup not a number", ("pullup_resistor", "abc")),
        ("inline negative", ("inline_resistor", "-1")),
        ("inline not a number", ("inline_resistor", "1k")),
    ] {
        scenarios.push(scenario(label, heater_section("EPCOS 100K B57560G104F", &[extra]), &[]));
    }
    // Missing sensor_pin: raised after the coefficients are set up.
    scenarios.push(scenario(
        "missing sensor_pin",
        section_text("extruder", &[("sensor_type", "EPCOS 100K B57560G104F"), ("min_temp", "0"), ("max_temp", "300")]),
        &[],
    ));
    // Missing sensor_pin after a beta fallback: the warning is logged first.
    scenarios.push(scenario(
        "missing sensor_pin after fallback",
        format!(
            "{}{}",
            custom_section("fallback", POINTS_BETA_FALLBACK_OPTS),
            section_text("extruder", &[("sensor_type", "fallback"), ("min_temp", "0"), ("max_temp", "300")])
        ),
        &["thermistor fallback"],
    ));
    scenarios.push(scenario("unknown sensor type", heater_section("No Such Thermistor", &[]), &[]));

    // A definition that fails to load registers nothing.
    scenarios.push(scenario(
        "load error",
        format!(
            "{}{}",
            custom_section("bad", &[("temperature1", "25")]),
            heater_section("bad", &[])
        ),
        &["thermistor bad"],
    ));

    // Math errors while setting the coefficients (raised by `setup_sensor`).
    for (label, opts) in [
        (
            "log of zero resistance",
            vec![("temperature1", "25"), ("resistance1", "0"), ("beta", "3950")],
        ),
        (
            "beta at absolute zero",
            vec![("temperature1", "-273.15"), ("resistance1", "100000"), ("beta", "3950")],
        ),
        (
            "equal resistances",
            vec![
                ("temperature1", "10"),
                ("resistance1", "1000"),
                ("temperature2", "20"),
                ("resistance2", "1000"),
                ("temperature3", "30"),
                ("resistance3", "1000"),
            ],
        ),
        (
            "equal temperatures",
            vec![
                ("temperature1", "25"),
                ("resistance1", "100000"),
                ("temperature2", "25"),
                ("resistance2", "1500"),
                ("temperature3", "250"),
                ("resistance3", "100"),
            ],
        ),
    ] {
        scenarios.push(scenario(
            label,
            format!("{}{}", custom_section("math", &opts), heater_section("math", &[])),
            &["thermistor math"],
        ));
    }

    compare_scenarios(&mut py, &scenarios);
}

/// `[thermistor <name>]` parameter parsing: name, option order, bounds, the
/// `sorted(...)` of three points (ties, descending order, nan) and the errors.
#[test]
fn custom_thermistor_params_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    let three = |t: [&str; 3], r: [&str; 3]| -> Vec<(String, String)> {
        let mut opts = Vec::new();
        for (i, (temperature, resistance)) in t.iter().zip(r.iter()).enumerate() {
            opts.push((format!("temperature{}", i + 1), temperature.to_string()));
            opts.push((format!("resistance{}", i + 1), resistance.to_string()));
        }
        opts
    };
    let owned = |opts: &[(&str, &str)]| -> Vec<(String, String)> {
        opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
    };
    let mut cases: Vec<ParamsCase> = vec![
        ("beta".into(), "beta one".into(), owned(BETA_OPTS)),
        (
            "beta ignores the other points".into(),
            "beta two".into(),
            owned(&[
                ("temperature1", "25"),
                ("resistance1", "100000"),
                ("beta", "4100"),
                ("temperature2", "bogus"),
            ]),
        ),
        (
            "beta bounds".into(),
            "edge".into(),
            owned(&[("temperature1", "-273.15"), ("resistance1", "0"), ("beta", "0.0001")]),
        ),
        ("points sorted".into(), "sorted".into(), three(["25", "150", "250"], ["100000", "1641.9", "226.15"])),
        ("points unsorted".into(), "unsorted".into(), owned(POINTS_UNSORTED_OPTS)),
        ("points descending".into(), "descending".into(), three(["300", "200", "100"], ["10", "20", "30"])),
        ("points middle first".into(), "mid".into(), three(["200", "100", "300"], ["10", "20", "30"])),
        ("points last smallest".into(), "last".into(), three(["200", "300", "100"], ["10", "20", "30"])),
        ("points tie on temperature".into(), "tie".into(), three(["25", "25", "10"], ["100", "50", "70"])),
        ("points tie on both".into(), "tie2".into(), three(["25", "25", "25"], ["100", "50", "75"])),
        ("points equal".into(), "same".into(), three(["25", "25", "25"], ["100", "100", "100"])),
        ("points beta fallback".into(), "fallback".into(), owned(POINTS_BETA_FALLBACK_OPTS)),
    ];
    // A nan in any position of the temperatures or of the tied resistances.
    for i in 0..3 {
        let mut temps = ["30", "10", "20"];
        temps[i] = "nan";
        cases.push((format!("nan temperature {}", i + 1), "nan".into(), three(temps, ["1", "2", "3"])));
        let mut res = ["3", "1", "2"];
        res[i] = "nan";
        cases.push((format!("nan resistance {}", i + 1), "nanr".into(), three(["25", "25", "25"], res)));
        let mut temps = ["10", "20", "30"];
        temps[i] = "nan";
        cases.push((format!("nan temperature ascending {}", i + 1), "nan2".into(), three(temps, ["1", "2", "3"])));
    }
    // Errors, in the reference's read order.
    let err_cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("missing temperature1", vec![("resistance1", "1"), ("beta", "1")]),
        ("missing resistance1", vec![("temperature1", "25"), ("beta", "1")]),
        ("temperature1 below minimum", vec![("temperature1", "-273.16"), ("resistance1", "1"), ("beta", "1")]),
        ("temperature1 unparsable", vec![("temperature1", "hot"), ("resistance1", "1"), ("beta", "1")]),
        ("resistance1 negative", vec![("temperature1", "25"), ("resistance1", "-1"), ("beta", "1")]),
        ("beta zero", vec![("temperature1", "25"), ("resistance1", "1"), ("beta", "0")]),
        ("beta negative", vec![("temperature1", "25"), ("resistance1", "1"), ("beta", "-5")]),
        ("beta unparsable", vec![("temperature1", "25"), ("resistance1", "1"), ("beta", "x")]),
        ("missing temperature2", vec![("temperature1", "25"), ("resistance1", "1")]),
        (
            "missing resistance2",
            vec![("temperature1", "25"), ("resistance1", "1"), ("temperature2", "50")],
        ),
        (
            "missing temperature3",
            vec![("temperature1", "25"), ("resistance1", "1"), ("temperature2", "50"), ("resistance2", "2")],
        ),
        (
            "temperature3 below minimum",
            vec![
                ("temperature1", "25"),
                ("resistance1", "1"),
                ("temperature2", "50"),
                ("resistance2", "2"),
                ("temperature3", "-300"),
                ("resistance3", "3"),
            ],
        ),
        (
            "resistance3 unparsable",
            vec![
                ("temperature1", "25"),
                ("resistance1", "1"),
                ("temperature2", "50"),
                ("resistance2", "2"),
                ("temperature3", "75"),
                ("resistance3", "ohm"),
            ],
        ),
        (
            "resistance2 negative",
            vec![
                ("temperature1", "25"),
                ("resistance1", "1"),
                ("temperature2", "50"),
                ("resistance2", "-2"),
                ("temperature3", "75"),
                ("resistance3", "3"),
            ],
        ),
    ];
    for (label, opts) in err_cases {
        cases.push((label.to_string(), "bad".to_string(), owned(&opts)));
    }

    for (label, name, opts) in &cases {
        let opts_ref: Vec<(&str, &str)> = opts.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        let text = custom_section(name, &opts_ref);
        let section = format!("thermistor {name}");
        let expected = py
            .eval(&format!("custom_params({text:?}, {section:?})"))
            .unwrap_or_else(|e| panic!("{label}: oracle raised {}: {}", e.class_name, e.message));
        install_log_sink();
        let printer = Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()) });
        let root = make_root(printer, &text);
        let actual = capture(|| {
            let custom = CustomThermistor::new(&root.getsection(&section))?;
            let params = match custom.params() {
                ThermistorParams::Beta { t1, r1, beta } => {
                    vec![("t1", *t1), ("r1", *r1), ("beta", *beta)]
                }
                ThermistorParams::Points { t1, r1, t2, r2, t3, r3 } => {
                    vec![("t1", *t1), ("r1", *r1), ("t2", *t2), ("r2", *r2), ("t3", *t3), ("r3", *r3)]
                }
            };
            Ok(Value::List(vec![
                Value::str(custom.name()),
                Value::Dict(params.into_iter().map(|(k, v)| (Value::str(k), Value::float(v))).collect()),
            ]))
        });
        check(&format!("params {label}"), &expected, &actual);
    }

    // A section with irregular spacing: the name re-joins the words.
    let spaced = "[thermistor   spaced   out  name]\ntemperature1: 25\nresistance1: 100000\nbeta: 3950\n";
    let expected = py.eval(&format!("custom_params({spaced:?}, 'thermistor   spaced   out  name')")).expect("oracle spaced");
    let printer = Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()) });
    let root = make_root(printer, spaced);
    let actual = capture(|| {
        let custom = CustomThermistor::new(&root.getsection("thermistor   spaced   out  name"))?;
        let ThermistorParams::Beta { t1, r1, beta } = *custom.params() else { panic!("beta definition") };
        Ok(Value::List(vec![
            Value::str(custom.name()),
            Value::Dict(vec![
                (Value::str("t1"), Value::float(t1)),
                (Value::str("r1"), Value::float(r1)),
                (Value::str("beta"), Value::float(beta)),
            ]),
        ]))
    });
    check("params spaced name", &expected, &actual);
}

/// The converter's exceptions reach the `AdcConvert` caller as the Python
/// exception class and message.
#[test]
fn adc_convert_errors_match_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let inf = f64::INFINITY;
    // (inline_resistor, t1, r1, beta, op, x)
    let cases: Vec<(f64, f64, f64, f64, &str, f64)> = vec![
        (0.0, 25.0, 100000.0, 3950.0, "calc_temp", 0.5),
        (0.0, 25.0, 100000.0, 3950.0, "calc_temp", 0.0),
        (1000.0, 25.0, 100000.0, 3950.0, "calc_temp", 0.0001),
        (1000.0, 25.0, 100000.0, 3950.0, "calc_temp", 0.5),
        (0.0, 25.0, 100000.0, 3950.0, "calc_adc", 100.0),
        (0.0, 25.0, 100000.0, 3950.0, "calc_adc", -273.0),
        (0.0, 25.0, 100000.0, 3950.0, "calc_adc", -273.15),
        (0.0, 25.0, 100000.0, 3950.0, "calc_adc", -300.0),
        (250.0, 25.0, 100000.0, 3950.0, "calc_adc", 100.0),
        (0.0, 25.0, 100000.0, inf, "calc_adc", 100.0),
        (0.0, 25.0, 0.0, 3950.0, "calc_adc", 100.0),
        (0.0, 25.0, -5.0, 3950.0, "calc_adc", 100.0),
        (0.0, 25.0, 100000.0, 0.0, "calc_adc", 100.0),
        (0.0, -273.15, 100000.0, 3950.0, "calc_temp", 0.5),
    ];
    for (inline, t1, r1, beta, op, x) in cases {
        let label = format!("{op}({x}) inline={inline} t1={t1} r1={r1} beta={beta}");
        let expected = py
            .eval(&format!(
                "conv_case({}, {}, {}, {}, {op:?}, {})",
                py_float(inline),
                py_float(t1),
                py_float(r1),
                py_float(beta),
                py_float(x)
            ))
            .unwrap_or_else(|e| panic!("{label}: oracle raised {}: {}", e.class_name, e.message));
        let actual = capture(|| {
            let mut th = Thermistor::new(4700.0, inline);
            th.setup_coefficients_beta(t1, r1, beta)
                .map_err(|e| KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg })?;
            let value = match op {
                "calc_temp" => AdcConvert::calc_temp(&th, x)?,
                _ => AdcConvert::calc_adc(&th, x)?,
            };
            Ok(Value::float(value))
        });
        check(&label, &expected, &actual);
    }
}

/// The module exposes `load_config_prefix` only, as the reference does.
#[test]
fn module_factory_matches_python() {
    let Some(mut py) = start_oracle() else { return };
    py.exec(ORACLE_SETUP).expect("oracle setup");
    let expected = py
        .eval("[hasattr(_tmod, 'load_config'), hasattr(_tmod, 'load_config_prefix')]")
        .expect("oracle hasattr");
    let factory = thermistor::module_factory();
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let actual = Value::List(vec![Value::Bool(factory.load_config.is_some()), Value::Bool(factory.load_config_prefix.is_some())]);
    check("module entry points", &expected, &actual);
}
