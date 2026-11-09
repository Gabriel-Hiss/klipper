//! Parity tests for `klippy/extras/adc_temperature.py`.
//!
//! The port is driven through fakes of the `klippy_api` contract traits
//! (`PrinterApi`, `ConfigApi`, `PinsApi`, `AdcPinApi`, `QueryAdcApi`,
//! `PrinterMCUErrorApi`, `PrinterHeatersApi`) with the same inputs given to
//! Python fakes running on the live reference through `pyoracle` (each test
//! skips when `$KLIPPY_PYTHON` is unset). Compared: `LinearInterpolate`
//! forward/reverse values and its `ValueError` paths (duplicate keys,
//! fewer than two samples, collinear slope de-duplication), `LinearVoltage`
//! / `LinearResistance` `calc_temp` / `calc_adc` over grids (custom
//! `adc_voltage` / `voltage_offset` / `pullup_resistor`, excluded
//! out-of-range voltage samples, the `[0.00001, 0.99999]` clamp including a
//! `nan` input), `calc_pt100` / `calc_ina826_pt100` tables, every default
//! voltage/resistance sensor, `PrinterADCtoTemperature` wiring
//! (`setup_adc_sample` arguments, `query_adc` registration, ADC callback
//! time shift, `setup_callback`), the diagnostics clarify strings, and
//! `load_config` / `load_config_prefix` factory registration and custom
//! voltage/resistance creation.

use extras::adc_temperature::{
    calc_ina826_pt100, calc_pt100, custom_sensor_name, load_config, load_config_prefix,
    CustomLinearResistance, CustomLinearVoltage, LinearInterpolate, LinearResistance, LinearVoltage,
    PrinterADCtoTemperature, AD595, REPORT_TIME, SAMPLE_COUNT, SAMPLE_TIME,
};
use extras::error_mcu::PrinterMCUErrorApi;
use extras::heaters::{PrinterHeatersApi, SensorFactory};
use extras::query_adc::QueryAdcApi;
use klippy_api::config::{
    ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts as CfgFloatOpts, GetIntOpts, GetListOpts,
    GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, Pin, PinsApi};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use pycompat::PyValue;
use recording::value::{FloatVal, Value};
use std::collections::BTreeMap;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Fakes of the contract traits
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<indexmap::IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
#[allow(clippy::new_ret_no_self)]
    fn new(objects: &[(&str, ObjectRef)]) -> Printer {
        Arc::new(FakePrinter { objects: GCell::new(objects.iter().map(|(n, o)| (n.to_string(), o.clone())).collect()) })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        panic!("not used")
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.borrow_mut();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!("Printer object '{name}' already created")));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
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
        self.objects.borrow().iter().map(|(n, o)| (n.clone(), o.clone())).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(&self, _config: &klippy_api::config::Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
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
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {
        panic!("not used")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("not used")
    }
    fn request_exit(&self, _result: &str) {
        panic!("not used")
    }
}

struct FakeConfig {
    printer: Printer,
    section: String,
    opts: BTreeMap<String, String>,
    floats: BTreeMap<String, f64>,
}

fn missing(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!("Option '{option}' in section '{section}' must be specified"))
}

impl FakeConfig {
    fn new_with_printer(printer: Printer, section: &str, opts: &[(&str, &str)], floats: &[(&str, f64)]) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            opts: opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            floats: floats.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
        })
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(missing(&self.section, option)),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("not used")
    }
    fn getfloat(&self, option: &str, opts: CfgFloatOpts) -> KResult<Option<f64>> {
        match self.floats.get(option).copied() {
            Some(v) => {
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(missing(&self.section, option)),
            },
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("not used")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("not used")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("not used")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("not used")
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("not used")
    }
    fn getsection(&self, _section: &str) -> klippy_api::config::Config {
        Arc::new(FakeConfig { printer: self.printer.clone(), section: self.section.clone(), opts: BTreeMap::new(), floats: BTreeMap::new() })
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<klippy_api::config::Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
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
    fn drive(&self, samples: Vec<(f64, f64)>) {
        let mut callback = self.callback.borrow_mut().take();
        if let Some(cb) = callback.as_mut() {
            cb(samples);
        }
        if callback.is_some() {
            *self.callback.borrow_mut() = callback;
        }
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
    fn new() -> Arc<Self> {
        Arc::new(FakePins { adcs: GCell::new(Vec::new()), setup_calls: GCell::new(Vec::new()) })
    }
    fn adc(&self, pin: &str) -> Arc<FakeAdcPin> {
        self.adcs.borrow().iter().find(|(n, _)| n == pin).map(|(_, a)| Arc::clone(a)).expect("adc pin exists")
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
    clarifies: GCell<Vec<(String, extras::error_mcu::ClarifyCallback)>>,
}

impl PrinterMCUErrorApi for FakeErrorMcu {
    fn add_clarify(&self, msg: &str, callback: extras::error_mcu::ClarifyCallback) {
        self.clarifies.borrow_mut().push((msg.to_string(), callback));
    }
}

klippy_api::printer_object!(FakeErrorMcu; dyn PrinterMCUErrorApi);

struct FakeHeaters {
    factories: GCell<Vec<(String, SensorFactory)>>,
}

impl FakeHeaters {
    fn new() -> Arc<Self> {
        Arc::new(FakeHeaters { factories: GCell::new(Vec::new()) })
    }
    fn factory(&self, name: &str) -> SensorFactory {
        self.factories.borrow().iter().find(|(n, _)| n == name).map(|(_, f)| f.clone()).expect("factory exists")
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.borrow_mut().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &klippy_api::config::Config, _gcode_id: Option<String>) -> KResult<Arc<extras::heaters::Heater>> {
        panic!("not used")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("not used")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<extras::heaters::Heater>> {
        panic!("not used")
    }
    fn setup_sensor(&self, _config: &klippy_api::config::Config) -> KResult<Arc<dyn extras::heaters::TemperatureSensorApi>> {
        panic!("not used")
    }
    fn register_sensor(
        &self,
        _config: &klippy_api::config::Config,
        _psensor: Arc<dyn extras::heaters::SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("not used")
    }
    fn register_monitor(&self, _config: &klippy_api::config::Config) {
        panic!("not used")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("not used")
    }
    fn set_temperature(&self, _heater: &Arc<extras::heaters::Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

/// A wired world: fakes behind one printer plus a section config.
struct World {
    pins: Arc<FakePins>,
    query: Arc<FakeQueryAdc>,
    error_mcu: Arc<FakeErrorMcu>,
    heaters: Arc<FakeHeaters>,
    config: klippy_api::config::Config,
}

fn world(section: &str, opts: &[(&str, &str)], floats: &[(&str, f64)]) -> World {
    let pins = FakePins::new();
    let query = Arc::new(FakeQueryAdc { registered: GCell::new(Vec::new()) });
    let error_mcu = Arc::new(FakeErrorMcu { clarifies: GCell::new(Vec::new()) });
    let heaters = FakeHeaters::new();
    let printer = FakePrinter::new(&[
        ("pins", pins.clone() as ObjectRef),
        ("query_adc", query.clone() as ObjectRef),
        ("error_mcu", error_mcu.clone() as ObjectRef),
        ("heaters", heaters.clone() as ObjectRef),
    ]);
    let config_obj = FakeConfig::new_with_printer(printer, section, opts, floats);
    let config: klippy_api::config::Config = config_obj;
    World { pins, query, error_mcu, heaters, config }
}

fn assert_float(label: &str, expected: f64, actual: f64) {
    assert_eq!(FloatVal::new(expected), FloatVal::new(actual), "{label}");
}

// ---------------------------------------------------------------------------
// Direct behaviour tests
// ---------------------------------------------------------------------------

#[test]
fn linear_interpolate_errors() {
    let err = LinearInterpolate::new(Vec::new()).expect_err("empty");
    assert_eq!(err.class_name(), "ValueError");
    assert_eq!(err.message(), "need at least two samples");

    let err = LinearInterpolate::new(vec![(1.0, 2.0)]).expect_err("single");
    assert_eq!(err.class_name(), "ValueError");
    assert_eq!(err.message(), "need at least two samples");

    let err = LinearInterpolate::new(vec![(1.0, 1.0), (1.0, 2.0)]).expect_err("duplicate");
    assert_eq!(err.class_name(), "ValueError");
    assert_eq!(err.message(), "duplicate value");

    // Collinear third point is de-duplicated (same slope).
    let li = LinearInterpolate::new(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)]).expect("collinear");
    assert_float("collinear midpoint", 1.5, li.interpolate(1.5));
    // Duplicate-slope continuation keeps the older segment origin: the
    // collinear (2.0, 2.0) sample is skipped without advancing last_key,
    // so the next segment runs from (1.0, 1.0) to (3.0, 4.0) with
    // gain 1.5 and offset -0.5, giving 2.5 * 1.5 - 0.5 = 3.25.
    let li = LinearInterpolate::new(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 4.0)]).expect("segments");
    assert_float("segment 1", 0.5, li.interpolate(0.5));
    assert_float("segment 2", 3.25, li.interpolate(2.5));
}

#[test]
fn custom_sensor_names() {
    let w = world("adc_temperature my sensor name", &[], &[]);
    assert_eq!(custom_sensor_name(&w.config), "my sensor name");
    let w = world("adc_temperature", &[], &[]);
    assert_eq!(custom_sensor_name(&w.config), "");
}

#[test]
fn custom_voltage_params() {
    let w = world(
        "adc_temperature mysensor",
        &[("sensor_pin", "P0")],
        &[
            ("temperature1", 0.0),
            ("voltage1", 0.0),
            ("temperature2", 100.0),
            ("voltage2", 1.0),
            ("temperature3", 200.0),
            ("voltage3", 2.0),
        ],
    );
    let custom = CustomLinearVoltage::new(&w.config).expect("custom voltage");
    assert_eq!(custom.name(), "mysensor");
    assert_eq!(custom.params(), &[(0.0, 0.0), (100.0, 1.0), (200.0, 2.0)]);
}

#[test]
fn custom_resistance_params() {
    let w = world(
        "adc_temperature mysensor",
        &[("sensor_pin", "P0")],
        &[
            ("temperature1", 0.0),
            ("resistance1", 100.0),
            ("temperature2", 200.0),
            ("resistance2", 200.0),
        ],
    );
    let custom = CustomLinearResistance::new(&w.config).expect("custom resistance");
    assert_eq!(custom.name(), "mysensor");
    assert_eq!(custom.samples(), &[(0.0, 100.0), (200.0, 200.0)]);
}

#[test]
fn load_config_registers_default_factories() {
    let w = world("heaters", &[], &[]);
    assert!(load_config(&w.config).expect("load_config").is_none());
    let names: Vec<String> = w.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["AD595", "AD597", "AD8494", "AD8495", "AD8496", "AD8497", "PT100 INA826", "PT1000"]);
    // Each factory builds a sensor with the report delta of the module.
    for (name, _) in w.heaters.factories.borrow().iter() {
        let sensor_world = world("heater_generic foo", &[("sensor_pin", "P0")], &[]);
        let factory = w.heaters.factory(name);
        let sensor = factory(&sensor_world.config).expect("factory builds sensor");
        assert_float("report delta", REPORT_TIME, sensor.get_report_time_delta());
    }
}

#[test]
fn load_config_prefix_selects_voltage_or_resistance() {
    // No resistance1: voltage mode.
    let w = world(
        "adc_temperature mysensor",
        &[("sensor_pin", "P0")],
        &[("temperature1", 0.0), ("voltage1", 0.0), ("temperature2", 100.0), ("voltage2", 1.0)],
    );
    assert!(load_config_prefix(&w.config).expect("load_config_prefix").is_none());
    let names: Vec<String> = w.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["mysensor"]);
    // resistance1 present: resistance mode.
    let w = world(
        "adc_temperature mysensor",
        &[("sensor_pin", "P0"), ("resistance1", "100")],
        &[("temperature1", 0.0), ("resistance1", 100.0), ("temperature2", 100.0), ("resistance2", 200.0)],
    );
    assert!(load_config_prefix(&w.config).expect("load_config_prefix").is_none());
    let names: Vec<String> = w.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["mysensor"]);
}

#[test]
fn printer_adc_wiring() {
    let w = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let lv = LinearVoltage::new(&w.config, AD595).expect("linear voltage");
    let sensor = PrinterADCtoTemperature::new(&w.config, Arc::new(lv)).expect("printer adc");
    // Pins set up once, as adc.
    assert_eq!(w.pins.setup_calls.borrow().clone(), [("adc".to_string(), "P1".to_string())]);
    // query_adc registration under the full section name.
    let names: Vec<String> = w.query.registered.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(names, ["heater_generic foo"]);
    // error_mcu clarify registered once.
    let clarifies = w.error_mcu.clarifies.borrow().clone();
    assert_eq!(clarifies.len(), 1);
    assert_eq!(clarifies[0].0, "ADC out of range");
    // Report delta constant.
    assert_float("report delta", REPORT_TIME, sensor.get_report_time_delta());
    // Callback path: samples[-1] wins, time shifted by 8 * 0.001.
    let received: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let received2 = Arc::clone(&received);
    sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
        received2.borrow_mut().push((read_time, temp));
    }));
    sensor.adc_callback(&[(1.0, 0.1), (1.5, 0.2)]).expect("adc_callback");
    let got = received.borrow().clone();
    assert_eq!(got.len(), 1);
    assert_float("callback time", 1.5 + SAMPLE_COUNT as f64 * SAMPLE_TIME, got[0].0);
    // setup_minmax programs sampling and diag bounds.
    sensor.setup_minmax(0.0, 300.0).expect("setup_minmax");
    let (report_time, opts) = w.pins.adc("P1").sample_args.borrow().clone().expect("sample args set");
    assert_float("sample report_time", REPORT_TIME, report_time);
    assert_float("sample_time", SAMPLE_TIME, opts.sample_time);
    assert_eq!(opts.sample_count, SAMPLE_COUNT);
    assert_eq!(opts.range_check_count, 4);
    assert!(opts.minval <= opts.maxval);
    // Empty sample batch is ignored (Python would index [-1] on an empty list).
    sensor.adc_callback(&[]).expect("adc_callback");
    assert_eq!(received.borrow().len(), 1);
}

#[test]
fn clarify_branches() {
    let w = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let lv = LinearVoltage::new(&w.config, AD595).expect("linear voltage");
    let sensor = PrinterADCtoTemperature::new(&w.config, Arc::new(lv)).expect("printer adc");
    let diag = sensor.diag_helper();
    let empty = PyValue::Dict(indexmap::IndexMap::new());
    // No bounds programmed yet.
    assert!(diag.clarify("ADC out of range", &empty).expect("clarify").is_none());
    sensor.setup_minmax(0.0, 300.0).expect("setup_minmax");
    // Zero timestamp: not yet sampled.
    w.pins.adc("P1").set_last((0.0, 0.5));
    assert!(diag.clarify("ADC out of range", &empty).expect("clarify").is_none());
    // Out-of-range timestamp and value: a message is produced.
    let (minval, maxval) = {
        let args = w.pins.adc("P1").sample_args.borrow().clone().expect("sample args");
        (args.1.minval, args.1.maxval)
    };
    w.pins.adc("P1").set_last((1.0, maxval + 0.5));
    let msg = diag.clarify("ADC out of range", &empty).expect("clarify").expect("message");
    assert!(msg.starts_with("Sensor 'heater_generic foo' temperature "), "{msg}");
    assert!(msg.contains(" not in range "), "{msg}");
    // The stored error_mcu callback reaches the same helper.
    let clarifies = w.error_mcu.clarifies.borrow().clone();
    let via = clarifies[0].1.as_ref()("ADC out of range", &empty).expect("clarify via callback").expect("message");
    assert_eq!(via, msg);
    // In range: no message.
    w.pins.adc("P1").set_last((1.0, (minval + maxval) / 2.));
    assert!(diag.clarify("ADC out of range", &empty).expect("clarify").is_none());
}

// ---------------------------------------------------------------------------
// Oracle comparison against the reference Python
// ---------------------------------------------------------------------------

const ORACLE_SETUP: &str = r#"
from extras.adc_temperature import (LinearInterpolate, LinearVoltage, LinearResistance, CustomLinearVoltage, CustomLinearResistance, PrinterADCtoTemperature, load_config, load_config_prefix, calc_pt100, calc_ina826_pt100, AD595, AD8494, AD8495)
_sentinel = object()
class Error(Exception): pass
class FakeAdc:
    def __init__(self, pin='P'):
        self.pin = pin; self.callback = None; self.sample_args = None; self.last = (0., 0.)
    def setup_adc_callback(self, cb): self.callback = cb
    def setup_adc_sample(self, report_time, sample_time, sample_count, minval=None, maxval=None, range_check_count=None):
        self.sample_args = (report_time, sample_time, sample_count, minval, maxval, range_check_count)
    def get_last_value(self): return self.last
    def get_mcu(self): return None
class FakePins:
    def __init__(self): self.adcs = {}
    def setup_pin(self, pin_type, pin_desc):
        adc = FakeAdc(pin_desc); self.adcs[pin_desc] = adc; return adc
class FakeQuery:
    def __init__(self): self.reg = {}
    def register_adc(self, name, adc): self.reg[name] = adc
class FakeErrorMcu:
    def __init__(self): self.clarifies = []
    def add_clarify(self, msg, cb): self.clarifies.append((msg, cb))
class FakeHeaters:
    def __init__(self): self.factories = {}
    def add_sensor_factory(self, sensor_type, factory): self.factories[sensor_type] = factory
class FakePrinter:
    def __init__(self, objs): self.objs = objs
    def lookup_object(self, name): return self.objs[name]
    def load_object(self, config, section): return self.objs[section]
class FakeConfig:
    def __init__(self, printer, name, opts=None, floats=None):
        self.printer = printer; self._name = name; self.opts = opts or {}; self.floats = floats or {}
    def get_printer(self): return self.printer
    def get_name(self): return self._name
    def get(self, name, default=_sentinel):
        if name in self.opts: return self.opts[name]
        if default is _sentinel: raise Error("Option '%s' in section '%s' must be specified" % (name, self.get_name()))
        return default
    def getfloat(self, name, default=_sentinel, above=None, minval=None, maxval=None, below=None):
        if name in self.floats: v = self.floats[name]
        elif default is _sentinel: raise Error("Option '%s' in section '%s' must be specified" % (name, self.get_name()))
        else: v = default
        if above is not None and v <= above: raise Error("Option '%s' in section '%s' must be above %s" % (name, self.get_name(), above))
        return v
    def error(self, msg): raise Error(msg)
_pins = FakePins(); _query = FakeQuery(); _err = FakeErrorMcu(); _heaters = FakeHeaters()
_printer = FakePrinter({'pins': _pins, 'query_adc': _query, 'error_mcu': _err, 'heaters': _heaters})
_cfg = FakeConfig(_printer, 'heater_generic foo', opts={'sensor_pin': 'P1'})
_lv = LinearVoltage(_cfg, AD595)
_sensor = PrinterADCtoTemperature(_cfg, _lv)
"#;

fn oracle_float(py: &mut pyoracle::Oracle, expr: &str) -> f64 {
    match py.eval(expr) {
        Ok(Value::Float(f)) => f.to_f64().expect("float hex decodes"),
        other => panic!("oracle: {expr} -> {other:?}"),
    }
}

fn vfloat(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("float hex decodes"),
        other => panic!("expected float, got {other:?}"),
    }
}

#[test]
fn oracle_interpolate_and_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    // LinearInterpolate over a non-monotonic sample set.
    let samples = [(0.1, 1.0), (0.5, 3.0), (0.9, 2.0)];
    let li = LinearInterpolate::new(samples.to_vec()).expect("linear interpolate");
    let keys = [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0];
    for key in keys {
        let expected = oracle_float(&mut py, &format!("LinearInterpolate([(0.1, 1.0), (0.5, 3.0), (0.9, 2.0)]).interpolate({key:?})"));
        assert_float(&format!("interpolate({key})"), expected, li.interpolate(key));
    }
    for value in [0.5, 1.0, 2.0, 2.5, 3.0, 3.5] {
        let expected = oracle_float(&mut py, &format!("LinearInterpolate([(0.1, 1.0), (0.5, 3.0), (0.9, 2.0)]).reverse_interpolate({value:?})"));
        assert_float(&format!("reverse({value})"), expected, li.reverse_interpolate(value));
    }

    // Collinear de-duplication: the skipped (2.0, 2.0) sample does not
    // advance last_key, so the next segment runs from (1.0, 1.0).
    let li2 = LinearInterpolate::new(vec![(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 4.0)]).expect("collinear segments");
    let expected = oracle_float(&mut py, "LinearInterpolate([(0.0, 0.0), (1.0, 1.0), (2.0, 2.0), (3.0, 4.0)]).interpolate(2.5)");
    assert_float("collinear segment 2 oracle", expected, li2.interpolate(2.5));

    // ValueError paths: class and message match.
    for (expr, message) in [
        ("LinearInterpolate([(1.0, 2.0)])", "need at least two samples"),
        ("LinearInterpolate([(1.0, 1.0), (1.0, 2.0)])", "duplicate value"),
        ("LinearInterpolate([])", "need at least two samples"),
    ] {
        match py.eval(expr) {
            Err(e) => {
                assert_eq!(e.class_name, "ValueError");
                assert_eq!(e.message, message);
            }
            other => panic!("expected oracle error for {expr}, got {other:?}"),
        }
    }

    // calc_pt100 / calc_ina826_pt100 pair lists match.
    let rs_pt100 = calc_pt100(100.0);
    let py_pt100 = match py.eval("calc_pt100()") {
        Ok(Value::List(items)) => items,
        other => panic!("calc_pt100 -> {other:?}"),
    };
    assert_eq!(py_pt100.len(), rs_pt100.len());
    for ((t, r), item) in rs_pt100.iter().zip(py_pt100.iter()) {
        let Value::Tuple(pair) = item else { panic!("pair {item:?}") };
        assert_float("pt100 t", *t, vfloat(&pair[0]));
        assert_float("pt100 r", *r, vfloat(&pair[1]));
    }
    let rs_ina = calc_ina826_pt100();
    let py_ina = match py.eval("calc_ina826_pt100()") {
        Ok(Value::List(items)) => items,
        other => panic!("calc_ina826_pt100 -> {other:?}"),
    };
    assert_eq!(py_ina.len(), rs_ina.len());
    for ((t, v), item) in rs_ina.iter().zip(py_ina.iter()) {
        let Value::Tuple(pair) = item else { panic!("pair {item:?}") };
        assert_float("ina t", *t, vfloat(&pair[0]));
        assert_float("ina v", *v, vfloat(&pair[1]));
    }
}

#[test]
fn oracle_voltage_and_resistance() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    // Built-in AD595 through the default options.
    let w = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let lv = LinearVoltage::new(&w.config, AD595).expect("linear voltage");
    for adc in [0.0, 0.1, 0.25, 0.5, 0.75, 1.0] {
        let expected = oracle_float(&mut py, &format!("LinearVoltage(_cfg, AD595).calc_temp({adc:?})"));
        assert_float(&format!("lv calc_temp({adc})"), expected, lv.calc_temp(adc));
    }
    for temp in [0.0, 25.0, 100.0, 200.0, 480.0] {
        let expected = oracle_float(&mut py, &format!("LinearVoltage(_cfg, AD595).calc_adc({temp:?})"));
        assert_float(&format!("lv calc_adc({temp})"), expected, lv.calc_adc(temp));
    }

    // Custom voltage options with an excluded out-of-range sample.
    let cw = world(
        "heater_generic foo",
        &[("sensor_pin", "P1")],
        &[("adc_voltage", 3.0), ("voltage_offset", 0.1)],
    );
    let params: &[(f64, f64)] = &[(0.0, 0.1), (100.0, 1.1), (200.0, 5.0), (-50.0, -1.0)];
    let lv2 = LinearVoltage::new(&cw.config, params).expect("custom voltage");
    py.exec(
        "_cv = FakeConfig(_printer, 'heater_generic foo', floats={'adc_voltage': 3.0, 'voltage_offset': 0.1})\n\
         _lv2 = LinearVoltage(_cv, [(0.0, 0.1), (100.0, 1.1), (200.0, 5.0), (-50.0, -1.0)])",
    )
    .expect("oracle custom voltage");
    for adc in [0.0, 0.33333333333333337, 0.5, 0.9999999999999999, 1.0] {
        let expected = oracle_float(&mut py, &format!("_lv2.calc_temp({adc:?})"));
        assert_float(&format!("lv2 calc_temp({adc})"), expected, lv2.calc_temp(adc));
    }
    for temp in [0.0, 50.0, 100.0] {
        let expected = oracle_float(&mut py, &format!("_lv2.calc_adc({temp:?})"));
        assert_float(&format!("lv2 calc_adc({temp})"), expected, lv2.calc_adc(temp));
    }

    // Single usable sample left after filtering: config error.
    let err = LinearVoltage::new(&cw.config, &[(0.0, 0.1), (100.0, 5.0)]).expect_err("too few samples");
    assert_eq!(err.class_name(), "Error");
    assert_eq!(err.message(), "adc_temperature need at least two samples in heater heater_generic foo");
    match py.eval("LinearVoltage(_cv, [(0.0, 0.1), (100.0, 5.0)])") {
        Err(e) => {
            assert_eq!(e.class_name, "Error");
            assert_eq!(e.message, "adc_temperature need at least two samples in heater heater_generic foo");
        }
        other => panic!("expected oracle error, got {other:?}"),
    }

    // PT1000 resistance through the default pullup, including the clamp.
    let rw = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let lr = LinearResistance::new(&rw.config, &calc_pt100(1000.0)).expect("linear resistance");
    py.exec("_lr = LinearResistance(_cfg, calc_pt100(1000.))").expect("oracle resistance");
    for adc in [0.00001, 0.01, 0.1, 0.5, 0.9, 0.99999] {
        let expected = oracle_float(&mut py, &format!("_lr.calc_temp({adc:?})"));
        assert_float(&format!("lr calc_temp({adc})"), expected, lr.calc_temp(adc));
    }
    // Outside the clamp and the nan input.
    for adc in [0.0, 1.0] {
        let expected = oracle_float(&mut py, &format!("_lr.calc_temp({adc:?})"));
        assert_float(&format!("lr clamped({adc})"), expected, lr.calc_temp(adc));
    }
    let expected_nan = oracle_float(&mut py, "_lr.calc_temp(float('nan'))");
    assert_float("lr calc_temp(nan)", expected_nan, lr.calc_temp(f64::NAN));
    for temp in [0.0, 100.0, 250.0, 480.0] {
        let expected = oracle_float(&mut py, &format!("_lr.calc_adc({temp:?})"));
        assert_float(&format!("lr calc_adc({temp})"), expected, lr.calc_adc(temp));
    }
}

#[test]
fn oracle_printer_wiring() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    let w = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let lv = LinearVoltage::new(&w.config, AD595).expect("linear voltage");
    let sensor = PrinterADCtoTemperature::new(&w.config, Arc::new(lv)).expect("printer adc");
    sensor.setup_minmax(0.0, 300.0).expect("setup_minmax");
    py.eval("_sensor.setup_minmax(0.0, 300.0)").expect("oracle setup_minmax");

    // setup_adc_sample arguments match, including the sorted ADC limits.
    let (report_time, opts) = w.pins.adc("P1").sample_args.borrow().clone().expect("sample args");
    assert_float("report_time", REPORT_TIME, report_time);
    assert_float("sample_time oracle", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[1]"), opts.sample_time);
    assert_float("minval oracle", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[3]"), opts.minval);
    assert_float("maxval oracle", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[4]"), opts.maxval);
    assert_eq!(opts.sample_count, SAMPLE_COUNT);
    assert_eq!(opts.range_check_count, 4);

    // ADC callback: last sample wins, time shifted by SAMPLE_COUNT * SAMPLE_TIME.
    let received: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let received2 = Arc::clone(&received);
    sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
        received2.borrow_mut().push((read_time, temp));
    }));
    py.exec("collected = []\n_sensor.setup_callback(lambda t, temp: collected.append((t, temp)))").expect("oracle callback");
    let batch: Vec<(f64, f64)> = vec![(1.0, 0.1), (1.5, 0.2)];
    py.eval("_pins.adcs['P1'].callback([(1.0, 0.1), (1.5, 0.2)])").expect("oracle drive");
    w.pins.adc("P1").drive(batch);
    let expected_collected: Vec<f64> = match py.eval("collected") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|item| match item {
                Value::Tuple(parts) => vfloat(&parts[1]),
                other => panic!("collected item {other:?}"),
            })
            .collect(),
        other => panic!("collected {other:?}"),
    };
    let got = received.borrow().clone();
    assert_eq!(got.len(), 1);
    assert_eq!(expected_collected.len(), 1);
    assert_float("collected temp", expected_collected[0], got[0].1);
    assert_float("collected time", oracle_float(&mut py, "collected[0][0]"), got[0].0);

    // Clarify messages match: zero timestamp, in range, out of range.
    let empty = PyValue::Dict(indexmap::IndexMap::new());
    let diag = sensor.diag_helper();
    py.exec("_pins.adcs['P1'].last = (0.0, 0.5)").expect("oracle last");
    w.pins.adc("P1").set_last((0.0, 0.5));
    assert!(diag.clarify("ADC out of range", &empty).expect("clarify").is_none());
    assert!(matches!(py.eval("_sensor.diag_helper._clarify_adc_range('ADC out of range', {})"), Ok(Value::Null)));

    let minval = opts.minval;
    let maxval = opts.maxval;
    py.exec(&format!("_pins.adcs['P1'].last = (1.0, {maxval:?})")).expect("oracle last");
    w.pins.adc("P1").set_last((1.0, maxval));
    assert!(diag.clarify("ADC out of range", &empty).expect("clarify").is_none());
    assert!(matches!(py.eval("_sensor.diag_helper._clarify_adc_range('ADC out of range', {})"), Ok(Value::Null)));

    let out_value = maxval + 0.5;
    py.exec(&format!("_pins.adcs['P1'].last = (1.0, {out_value:?})")).expect("oracle last");
    w.pins.adc("P1").set_last((1.0, out_value));
    let expected_msg = match py.eval("_sensor.diag_helper._clarify_adc_range('ADC out of range', {})") {
        Ok(Value::Str(s)) => s,
        other => panic!("clarify -> {other:?}"),
    };
    let actual_msg = diag.clarify("ADC out of range", &empty).expect("clarify").expect("message");
    assert_eq!(actual_msg, expected_msg);
    let _ = minval;
}

#[test]
fn oracle_load_config() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    // Default factories registered in table order.
    py.eval("load_config(_cfg)").expect("oracle load_config");
    let oracle_names = match py.eval("list(_heaters.factories.keys())") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Str(s) => s.clone(),
                other => panic!("name {other:?}"),
            })
            .collect::<Vec<String>>(),
        other => panic!("names -> {other:?}"),
    };
    let w = world("heaters", &[], &[]);
    assert!(load_config(&w.config).expect("load_config").is_none());
    let rust_names: Vec<String> = w.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(rust_names, oracle_names);

    // An AD595 factory-created sensor converts like the reference.
    let ad595_factory = w.heaters.factory("AD595");
    let rw = world("heater_generic foo", &[("sensor_pin", "P1")], &[]);
    let rust_sensor = ad595_factory(&rw.config).expect("ad595 sensor");
    rust_sensor.setup_minmax(0.0, 300.0).expect("setup_minmax");
    py.eval("_heaters.factories['AD595'](_cfg).setup_minmax(0.0, 300.0)").expect("oracle ad595 setup_minmax");
    let (_, rust_opts) = rw.pins.adc("P1").sample_args.borrow().clone().expect("sample args");
    assert_float("ad595 minval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[3]"), rust_opts.minval);
    assert_float("ad595 maxval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[4]"), rust_opts.maxval);
}

#[test]
fn oracle_load_config_prefix() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping oracle comparison: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(ORACLE_SETUP).expect("oracle setup");

    // Voltage mode (no resistance1).
    let opts = &[("sensor_pin", "P1")];
    let floats = &[
        ("temperature1", 0.0),
        ("voltage1", 0.0),
        ("temperature2", 100.0),
        ("voltage2", 1.0),
        ("temperature3", 200.0),
        ("voltage3", 2.0),
    ];
    py.exec(
        "_cv2 = FakeConfig(_printer, 'adc_temperature mysensor', opts={'sensor_pin': 'P1'}, floats={'temperature1': 0.0, 'voltage1': 0.0, 'temperature2': 100.0, 'voltage2': 1.0, 'temperature3': 200.0, 'voltage3': 2.0})\n\
         load_config_prefix(_cv2)",
    )
    .expect("oracle load_config_prefix voltage");
    let w = world("adc_temperature mysensor", opts, floats);
    assert!(load_config_prefix(&w.config).expect("load_config_prefix").is_none());
    let oracle_names = match py.eval("list(_heaters.factories.keys())") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Str(s) => s.clone(),
                other => panic!("name {other:?}"),
            })
            .collect::<Vec<String>>(),
        other => panic!("names -> {other:?}"),
    };
    let rust_names: Vec<String> = w.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(rust_names, oracle_names);
    assert_eq!(rust_names, ["mysensor"]);
    // The created sensor programs the same sampling limits.
    let factory = w.heaters.factory("mysensor");
    let rust_sensor = factory(&w.config).expect("voltage sensor");
    rust_sensor.setup_minmax(0.0, 200.0).expect("setup_minmax");
    py.eval("_heaters.factories['mysensor'](_cv2).setup_minmax(0.0, 200.0)").expect("oracle sensor setup_minmax");
    let (_, rust_opts) = w.pins.adc("P1").sample_args.borrow().clone().expect("sample args");
    assert_float("voltage minval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[3]"), rust_opts.minval);
    assert_float("voltage maxval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[4]"), rust_opts.maxval);

    // Resistance mode (resistance1 present).
    let opts_r = &[("sensor_pin", "P1"), ("resistance1", "100")];
    let floats_r = &[
        ("temperature1", 0.0),
        ("resistance1", 100.0),
        ("temperature2", 200.0),
        ("resistance2", 200.0),
    ];
    py.exec(
        "_cr2 = FakeConfig(_printer, 'adc_temperature mysensor', opts={'sensor_pin': 'P1', 'resistance1': '100'}, floats={'temperature1': 0.0, 'resistance1': 100.0, 'temperature2': 200.0, 'resistance2': 200.0})\n\
         load_config_prefix(_cr2)",
    )
    .expect("oracle load_config_prefix resistance");
    let rw = world("adc_temperature mysensor", opts_r, floats_r);
    assert!(load_config_prefix(&rw.config).expect("load_config_prefix").is_none());
    let rust_names: Vec<String> = rw.heaters.factories.borrow().iter().map(|(n, _)| n.clone()).collect();
    assert_eq!(rust_names, ["mysensor"]);
    let factory = rw.heaters.factory("mysensor");
    let rust_sensor = factory(&rw.config).expect("resistance sensor");
    rust_sensor.setup_minmax(0.0, 200.0).expect("setup_minmax");
    py.eval("_heaters.factories['mysensor'](_cr2).setup_minmax(0.0, 200.0)").expect("oracle sensor setup_minmax");
    let (_, rust_opts) = rw.pins.adc("P1").sample_args.borrow().clone().expect("sample args");
    assert_float("resistance minval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[3]"), rust_opts.minval);
    assert_float("resistance maxval", oracle_float(&mut py, "_pins.adcs['P1'].sample_args[4]"), rust_opts.maxval);
}
