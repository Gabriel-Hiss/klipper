//! Parity tests for `extras::temperature_sensor` (port of
//! `klippy/extras/temperature_sensor.py`).
//!
//! Every expected value comes from the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a fresh
//! `PrinterSensorGeneric` is built on both sides with minimal Python
//! fakes for the printer, config and `heaters` object, then fed
//! identical construction, callback and status calls and the results
//! are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so replay fixtures do not apply; expected values come
//! from the oracle alone.

use extras::heaters::{
    PrinterHeatersApi, SensorApi, TemperatureCallback, TemperatureSensorApi,
};
use extras::temperature_sensor::PrinterSensorGeneric;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell};
use pycompat::text::{float_from_string, float_str};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.temperature_sensor as ts

class _Sentinel:
    pass

_SENTINEL = _Sentinel()

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.load_calls = []
    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_SENTINEL):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        if default is _SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (section,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options=None):
        self._printer = printer
        self._name = name
        self.options = options or {}
        self.getfloat_calls = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=_SENTINEL):
        if option in self.options:
            return self.options[option]
        if default is _SENTINEL:
            raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return default
    def getfloat(self, option, default=_SENTINEL, minval=None, maxval=None, above=None, below=None):
        self.getfloat_calls.append((option, default if default is not _SENTINEL else '<required>', minval, maxval, above, below))
        if option not in self.options:
            if default is _SENTINEL:
                raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        raw = self.options[option]
        try:
            v = float(raw)
        except Exception:
            raise self.error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v

class FakeSensor:
    def __init__(self, report_delta=0.3):
        self.report_delta = report_delta
        self.minmax = []
        self.callback = None
    def setup_minmax(self, a, b):
        self.minmax.append((a, b))
    def setup_callback(self, cb):
        self.callback = cb
    def get_report_time_delta(self):
        return self.report_delta
    def fire(self, read_time, temp):
        self.callback(read_time, temp)

class FakeHeaters:
    def __init__(self, sensor):
        self.sensor = sensor
        self.setup_calls = []
        self.register_calls = []
    def setup_sensor(self, config):
        self.setup_calls.append(config.get_name())
        sensor_type = config.get('sensor_type')
        if sensor_type != 'fake_sensor':
            raise config.error("Unknown temperature sensor '%s'" % (sensor_type,))
        return self.sensor
    def register_sensor(self, config, psensor, gcode_id=None):
        self.register_calls.append((config.get_name(), gcode_id))

def build(options=None, name='temperature_sensor cabinet'):
    sensor = FakeSensor()
    heaters = FakeHeaters(sensor)
    printer = FakePrinter()
    printer.objects['heaters'] = heaters
    opts = {'sensor_type': 'fake_sensor'}
    if options:
        opts.update(options)
    config = FakeConfig(printer, name, opts)
    obj = ts.load_config_prefix(config)
    return (obj, sensor, heaters, printer, config)
def build_fakes(options=None, name='temperature_sensor cabinet'):
    sensor = FakeSensor()
    heaters = FakeHeaters(sensor)
    printer = FakePrinter()
    printer.objects['heaters'] = heaters
    opts = {'sensor_type': 'fake_sensor'}
    if options:
        opts.update(options)
    config = FakeConfig(printer, name, opts)
    return (sensor, heaters, printer, config)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinterState {
    objects: IndexMap<String, ObjectRef>,
    reactor: Reactor,
    load_calls: Vec<String>,
}

struct FakePrinter {
    state: GCell<FakePrinterState>,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            state: GCell::new(FakePrinterState {
                objects: IndexMap::new(),
                reactor: Reactor::new(false),
                load_calls: Vec::new(),
            }),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
        self.state.borrow_mut().objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.state.borrow().objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.state.borrow().objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _config: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.state.borrow_mut().load_calls.push(section.to_string());
        if let Some(o) = self.state.borrow().objects.get(section).cloned() {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{section}'"
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
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfigData {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
    getfloat_calls: GCell<Vec<String>>,
}

struct FakeConfig {
    inner: Arc<FakeConfigData>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: IndexMap<String, String>) -> Arc<Self> {
        Arc::new(FakeConfig {
            inner: Arc::new(FakeConfigData {
                printer,
                name: name.to_string(),
                options,
                getfloat_calls: GCell::new(Vec::new()),
            }),
        })
    }
}

fn fmt_bound_float(v: f64) -> String {
    float_str(v)
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.inner.printer.clone()
    }
    fn get_name(&self) -> String {
        self.inner.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.inner.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.inner.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, _option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match opts.default {
            None => Err(KlippyError::config("must be specified".to_string())),
            Some(d) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let fmt_opt = |o: Option<f64>| match o {
            Some(v) => fmt_bound_float(v),
            None => "None".to_string(),
        };
        let def_s = match opts.default {
            None => "<required>".to_string(),
            Some(d) => fmt_opt(d),
        };
        self.inner.getfloat_calls.borrow_mut().push(format!(
            "{option}|{def_s}|{}|{}|{}|{}",
            fmt_opt(opts.minval),
            fmt_opt(opts.maxval),
            fmt_opt(opts.above),
            fmt_opt(opts.below)
        ));
        if let Some(raw) = self.inner.options.get(option) {
            let v = float_from_string(raw).map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.inner.name
                ))
            })?;
            if let Some(minv) = opts.minval {
                if v < minv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have minimum of {}",
                        self.inner.name,
                        fmt_bound_float(minv)
                    )));
                }
            }
            if let Some(maxv) = opts.maxval {
                if v > maxv {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must have maximum of {}",
                        self.inner.name,
                        fmt_bound_float(maxv)
                    )));
                }
            }
            if let Some(ab) = opts.above {
                if v <= ab {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be above {}",
                        self.inner.name,
                        fmt_bound_float(ab)
                    )));
                }
            }
            if let Some(be) = opts.below {
                if v >= be {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be below {}",
                        self.inner.name,
                        fmt_bound_float(be)
                    )));
                }
            }
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.inner.name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match opts.default {
            None => Err(KlippyError::config("must be specified".to_string())),
            Some(d) => Ok(d),
        }
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        match opts.default {
            None => Err(KlippyError::config("must be specified".to_string())),
            Some(d) => Ok(d.unwrap_or(PyValue::None)),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(opts.default.unwrap())
    }
    fn getintlist(&self, _option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(opts.default.unwrap())
    }
    fn getfloatlist(&self, _option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(opts.default.unwrap())
    }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::new(self.inner.printer.clone(), section, IndexMap::new())
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

klippy_api::printer_object!(FakeConfig);

struct FakeSensor {
    report_delta: f64,
    minmax: GCell<Vec<(f64, f64)>>,
    callback: GCell<Option<TemperatureCallback>>,
}

impl FakeSensor {
    fn new() -> Arc<Self> {
        Arc::new(FakeSensor {
            report_delta: 0.3,
            minmax: GCell::new(Vec::new()),
            callback: GCell::new(None),
        })
    }

    fn fire(&self, read_time: f64, temp: f64) {
        let mut slot = self.callback.borrow_mut();
        match slot.as_mut() {
            Some(cb) => cb(read_time, temp),
            None => panic!("no callback registered"),
        }
    }
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        self.minmax.borrow_mut().push((min_temp, max_temp));
        Ok(())
    }

    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }

    fn get_report_time_delta(&self) -> f64 {
        self.report_delta
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

struct FakeHeaters {
    sensor: Arc<FakeSensor>,
    setup_calls: GCell<Vec<String>>,
    register_calls: GCell<Vec<String>>,
}

impl FakeHeaters {
    fn new(sensor: Arc<FakeSensor>) -> Arc<Self> {
        Arc::new(FakeHeaters {
            sensor,
            setup_calls: GCell::new(Vec::new()),
            register_calls: GCell::new(Vec::new()),
        })
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: extras::heaters::SensorFactory) {}
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<extras::heaters::Heater>> {
        Err(KlippyError::config("unused".to_string()))
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<extras::heaters::Heater>> {
        Err(KlippyError::config("unused".to_string()))
    }
    fn setup_sensor(&self, config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        self.setup_calls.borrow_mut().push(config.get_name());
        let sensor_type = config.get("sensor_type", GetStrOpts::default())?.unwrap();
        if sensor_type != "fake_sensor" {
            return Err(KlippyError::config(format!(
                "Unknown temperature sensor '{sensor_type}'"
            )));
        }
        Ok(self.sensor.clone() as Arc<dyn TemperatureSensorApi>)
    }
    fn register_sensor(&self, config: &Config, _psensor: Arc<dyn SensorApi>, _gcode_id: Option<String>) -> KResult<()> {
        self.register_calls.borrow_mut().push(config.get_name());
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<extras::heaters::Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        Err(KlippyError::config("unused".to_string()))
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("temperature_sensor fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping temperature_sensor oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "temperature_sensor".to_string(),
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

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: f64) {
    check(label, expected, Value::float(actual));
}

fn check_str(label: &str, expected: pyoracle::OracleResult, actual: String) {
    check(label, expected, Value::str(&actual));
}

fn check_bool(label: &str, expected: pyoracle::OracleResult, actual: bool) {
    check(label, expected, Value::Bool(actual));
}

fn check_err_val<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(_) => panic!("{label}: oracle returned a value but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned a value but call must fail"),
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

fn rust_opts(map: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut m = IndexMap::new();
    for (k, v) in map {
        m.insert(k.to_string(), v.to_string());
    }
    m
}

struct Pair {
    obj: Arc<PrinterSensorGeneric>,
    sensor: Arc<FakeSensor>,
    heaters: Arc<FakeHeaters>,
    printer: Arc<FakePrinter>,
    cfg: Arc<FakeConfig>,
}

fn fresh_pair(py: &mut pyoracle::Oracle, options: &[(&str, &str)], name: &str) -> Pair {
    let py_opts = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect::<Vec<_>>()
        .join(", ");
    let py_name = format!("'{}'", name);
    py.exec(&format!("obj, sensor, heaters, printer, config = build({{{py_opts}}}, {py_name})"))
        .expect("py build");
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor.clone());
    {
        let mut st = printer.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let mut opts = rust_opts(&[("sensor_type", "fake_sensor")]);
    for (k, v) in options {
        opts.insert(k.to_string(), v.to_string());
    }
    let cfg_concrete = FakeConfig::new(printer_ref, name, opts);
    let cfg: Config = cfg_concrete.clone();
    let obj = PrinterSensorGeneric::new(&cfg).expect("rust PrinterSensorGeneric::new");
    Pair {
        obj,
        sensor,
        heaters,
        printer,
        cfg: cfg_concrete,
    }
}


fn status_field(py_val: Value, key: &str) -> Value {
    match py_val {
        Value::Dict(pairs) => {
            for (k, v) in pairs {
                if let Value::Str(s) = k {
                    if s == key {
                        return v;
                    }
                }
            }
            panic!("missing key {key}");
        }
        other => panic!("status shape {other:?}"),
    }
}

fn rust_status_field(obj: &PrinterSensorGeneric, key: &str) -> f64 {
    let st = obj.get_status(0.).expect("status");
    match st {
        PyValue::Dict(m) => match m.get(&PyValue::Str(key.to_string())) {
            Some(PyValue::Float(f)) => *f,
            other => panic!("missing {key}: {other:?}"),
        },
        other => panic!("status shape {other:?}"),
    }
}

fn check_status(label: &str, py: &mut pyoracle::Oracle, obj: &PrinterSensorGeneric) {
    let expected = py.eval("obj.get_status(1000.0)").expect("py status");
    for key in ["temperature", "measured_min_temp", "measured_max_temp"] {
        let exp_field = status_field(expected.clone(), key);
        let actual = match exp_field {
            Value::Float(_) => rust_status_field(obj, key),
            other => panic!("status field not float: {other:?}"),
        };
        // expected.clone() is moved above; re-evaluate per key instead.
        check(&format!("{label} {key}"), Ok(exp_field), Value::float(actual));
    }
}

fn check_stats(label: &str, py: &mut pyoracle::Oracle, obj: &PrinterSensorGeneric) {
    let expected = py.eval("obj.stats(1000.0)").expect("py stats");
    let (active, msg) = obj.stats(1000.).expect("rust stats");
    match expected {
        Value::Tuple(items) => {
            check_bool(&format!("{label} active"), Ok(items[0].clone()), active);
            match &items[1] {
                Value::Str(_) => check_str(&format!("{label} msg"), Ok(items[1].clone()), msg),
                other => panic!("stats msg shape {other:?}"),
            }
        }
        other => panic!("stats shape {other:?}"),
    }
}

fn check_get_temp(label: &str, py: &mut pyoracle::Oracle, obj: &PrinterSensorGeneric) {
    let expected = py.eval("obj.get_temp(1000.0)").expect("py get_temp");
    let (cur, tgt) = SensorApi::get_temp(obj, 1000.);
    match expected {
        Value::Tuple(items) => {
            check_float(&format!("{label} cur"), Ok(items[0].clone()), cur);
            // Second element is the int 0 in Python (`0.`); compare as float bits.
            check_float(&format!("{label} tgt"), Ok(items[1].clone()), tgt);
        }
        other => panic!("get_temp shape {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_prefix() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::temperature_sensor::module_factory();
    assert!(factory.load_config.is_none(), "no load_config");
    assert!(factory.load_config_prefix.is_some(), "exposes load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no early objects");
    assert!(factory.add_printer_objects.is_none(), "no printer objects");
    let load_config_prefix = factory.load_config_prefix.expect("load_config_prefix");
    let _pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    let printer_arc = FakePrinter::new();
    let printer_ref: Printer = printer_arc.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor);
    {
        let mut st = printer_arc.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[("sensor_type", "fake_sensor")]),
    );
    let expected = py.eval("type(obj).__name__").expect("py type");
    let obj = load_config_prefix(&cfg).expect("factory load").expect("factory obj");
    let typed: Arc<PrinterSensorGeneric> =
        klippy_api::downcast_arc(&obj).expect("downcast");
    check_str("factory type", Ok(expected), "PrinterSensorGeneric".to_string());
    let _ = typed;
}

#[test]
fn defaults_minmax_and_registration() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    // Python recorded setup_minmax and register_sensor; Rust recorded the
    // same calls through its fakes.
    let py_minmax = py.eval("sensor.minmax").expect("py minmax");
    let rust_minmax = pair.sensor.minmax.borrow().clone();
    match py_minmax {
        Value::List(items) => {
            assert_eq!(items.len(), 1, "one setup_minmax call");
            match &items[0] {
                Value::Tuple(pair_vals) => {
                    check_float(
                        "minmax min",
                        Ok(pair_vals[0].clone()),
                        rust_minmax[0].0,
                    );
                    check_float(
                        "minmax max",
                        Ok(pair_vals[1].clone()),
                        rust_minmax[0].1,
                    );
                }
                other => panic!("minmax shape {other:?}"),
            }
        }
        other => panic!("minmax shape {other:?}"),
    }
    let expected_regs = py.eval("heaters.register_calls").expect("py regs");
    let rust_regs = pair.heaters.register_calls.borrow().clone();
    match expected_regs {
        Value::List(items) => {
            assert_eq!(items.len(), 1, "one register_sensor call");
            match &items[0] {
                Value::Tuple(vals) => {
                    // vals[0] is the section name string.
                    check_str(
                        "register name",
                        Ok(vals[0].clone()),
                        rust_regs[0].clone(),
                    );
                }
                other => panic!("register shape {other:?}"),
            }
        }
        other => panic!("regs shape {other:?}"),
    }
    assert_eq!(pair.heaters.setup_calls.borrow().len(), 1, "one setup_sensor call");
    assert_eq!(
        pair.printer.state.borrow().load_calls,
        vec!["heaters".to_string()],
        "printer.load_object(section='heaters')"
    );
    // getfloat call order and bounds: min_temp then max_temp, formatted
    // on the Python side with `str()` exactly as the Rust fake formats
    // them, then compared string-for-string against the oracle.
    let py_calls = py
        .eval("[str(c[0]) + '|' + str(c[1]) + '|' + str(c[2]) + '|' + str(c[3]) + '|' + str(c[4]) + '|' + str(c[5]) for c in config.getfloat_calls]")
        .expect("py calls");
    let rust_calls = pair.cfg.inner.getfloat_calls.borrow().clone();
    match py_calls {
        Value::List(items) => {
            assert_eq!(items.len(), 2, "two getfloat calls");
            assert_eq!(rust_calls.len(), 2, "two rust getfloat calls");
            for (item, rust_call) in items.iter().zip(rust_calls.iter()) {
                check_str("getfloat call", Ok(item.clone()), rust_call.clone());
            }
        }
        other => panic!("getfloat calls shape {other:?}"),
    }
}

#[test]
fn explicit_min_max() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(
        &mut py,
        &[("min_temp", "10.0"), ("max_temp", "120.0")],
        "temperature_sensor chamber",
    );
    let rust_minmax = pair.sensor.minmax.borrow().clone();
    assert_eq!(rust_minmax.len(), 1, "one setup_minmax call");
    let py_minmax = py.eval("sensor.minmax").expect("py minmax");
    match py_minmax {
        Value::List(items) => match &items[0] {
            Value::Tuple(vals) => {
                check_float("explicit min", Ok(vals[0].clone()), rust_minmax[0].0);
                check_float("explicit max", Ok(vals[1].clone()), rust_minmax[0].1);
            }
            other => panic!("minmax shape {other:?}"),
        },
        other => panic!("minmax shape {other:?}"),
    }
    // Name is the last token of the section name.
    check_stats("explicit stats name", &mut py, &pair.obj);
    let expected_msg = py.eval("obj.stats(1000.0)[1]").expect("py msg");
    let (_, msg) = pair.obj.stats(1000.).expect("rust stats");
    check_str("explicit stats msg", Ok(expected_msg), msg);
}

#[test]
fn initial_status_and_stats() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    check_get_temp("initial", &mut py, &pair.obj);
    check_status("initial", &mut py, &pair.obj);
    check_stats("initial", &mut py, &pair.obj);
}

#[test]
fn callback_tracks_min_max_and_zero_skip() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    for temp in [22.5_f64, 0.0, -5.25, 100.75, -0.0, 41.0] {
        // Interpolate as a Python float expression.
        let py_temp = format!("{:?}", temp);
        py.exec(&format!("sensor.fire(1000.0, {py_temp})"))
            .expect("py fire");
        pair.sensor.fire(1000., temp);
    }
    check_get_temp("after callbacks", &mut py, &pair.obj);
    check_status("after callbacks", &mut py, &pair.obj);
    check_stats("after callbacks", &mut py, &pair.obj);
}

#[test]
fn callback_negative_and_fractional_rounding() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    for temp in [25.456_f64, 25.454, 2.675, -12.345] {
        let py_temp = format!("{:?}", temp);
        py.exec(&format!("sensor.fire(2000.0, {py_temp})"))
            .expect("py fire");
        pair.sensor.fire(2000., temp);
    }
    check_status("rounding", &mut py, &pair.obj);
    check_stats("rounding", &mut py, &pair.obj);
}

#[test]
fn min_temp_below_bound_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor);
    {
        let mut st = printer.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[("sensor_type", "fake_sensor"), ("min_temp", "-300.0")]),
    );
    py.exec("sensor, heaters, printer, config = build_fakes({'min_temp': '-300.0'})")
        .expect("py build");
    let expected = py.eval("ts.load_config_prefix(config)");
    let actual = PrinterSensorGeneric::new(&cfg).map(|_| ());
    check_err_val("min_temp below bound", expected, actual);
}

#[test]
fn max_temp_not_above_min_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor);
    {
        let mut st = printer.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[
            ("sensor_type", "fake_sensor"),
            ("min_temp", "50.0"),
            ("max_temp", "50.0"),
        ]),
    );
    py.exec("sensor, heaters, printer, config = build_fakes({'min_temp': '50.0', 'max_temp': '50.0'})")
        .expect("py build");
    let expected = py.eval("ts.load_config_prefix(config)");
    let actual = PrinterSensorGeneric::new(&cfg).map(|_| ());
    check_err_val("max_temp not above min", expected, actual);
}

#[test]
fn unparseable_temp_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor);
    {
        let mut st = printer.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[("sensor_type", "fake_sensor"), ("min_temp", "banana")]),
    );
    py.exec("sensor, heaters, printer, config = build_fakes({'min_temp': 'banana'})")
        .expect("py build");
    let expected = py.eval("ts.load_config_prefix(config)");
    let actual = PrinterSensorGeneric::new(&cfg).map(|_| ());
    check_err_val("unparseable min_temp", expected, actual);
}

#[test]
fn unknown_sensor_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let sensor = FakeSensor::new();
    let heaters = FakeHeaters::new(sensor);
    {
        let mut st = printer.state.borrow_mut();
        st.objects.insert("heaters".to_string(), heaters.clone() as ObjectRef);
    }
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[("sensor_type", "nope_sensor")]),
    );
    py.exec("sensor, heaters, printer, config = build_fakes({'sensor_type': 'nope_sensor'})")
        .expect("py build");
    let expected = py.eval("ts.load_config_prefix(config)");
    let actual = PrinterSensorGeneric::new(&cfg).map(|_| ());
    check_err_val("unknown sensor", expected, actual);
}

#[test]
fn missing_heaters_errors() {
    let Some(mut py) = oracle() else { return };
    let printer = FakePrinter::new();
    let printer_ref: Printer = printer.clone();
    let cfg: Config = FakeConfig::new(
        printer_ref,
        "temperature_sensor cabinet",
        rust_opts(&[("sensor_type", "fake_sensor")]),
    );
    // Python: FakePrinter has no 'heaters', load_object raises.
    py.exec("obj, sensor, heaters, printer, config = build({})").expect("py build");
    py.exec("printer.objects.clear()").expect("py clear");
    let expected = py.eval("ts.load_config_prefix(config)");
    let actual = PrinterSensorGeneric::new(&cfg).map(|_| ());
    check_err_val("missing heaters", expected, actual);
}

#[test]
fn infinite_temp_updates_max() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    py.exec("sensor.fire(3000.0, float('inf'))").expect("py inf");
    pair.sensor.fire(3000., f64::INFINITY);
    check_status("inf", &mut py, &pair.obj);
    check_stats("inf", &mut py, &pair.obj);
    check_get_temp("inf", &mut py, &pair.obj);
}

#[test]
fn implements_sensor_api_interface() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[], "temperature_sensor cabinet");
    let obj_ref: ObjectRef = pair.obj.clone();
    let iface = klippy_api::interface::<dyn SensorApi>(&obj_ref);
    assert!(iface.is_some(), "object provides SensorApi");
    let expected = py.eval("obj.get_temp(1000.0)").expect("py get_temp");
    let via_iface = iface.expect("iface").get_temp(1000.);
    match expected {
        Value::Tuple(items) => {
            check_float("iface cur", Ok(items[0].clone()), via_iface.0);
            check_float("iface tgt", Ok(items[1].clone()), via_iface.1);
        }
        other => panic!("get_temp shape {other:?}"),
    }
}

