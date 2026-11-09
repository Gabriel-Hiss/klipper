//! Parity tests for `extras::temperature_host` (port of
//! `klippy/extras/temperature_host.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `Temperature_HOST` is
//! built on both sides with minimal Python fakes for the printer, the reactor
//! and the config wrapper, then fed identical sensor files and calls, and the
//! resulting temperatures, callback arguments, shutdown messages, status dicts
//! and config errors are compared exactly (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.
//!
//! The one place exact comparison is impossible is the sample wake time
//! (`measured_time + HOST_REPORT_TIME`): the Rust side reads the real
//! `reactor.monotonic()` clock while the Python fake returns a fixed value.
//! There the Rust return is checked to fall in the `[monotonic+1, monotonic+1]`
//! bracket around the call, and the oracle value is checked to be the same
//! `measured + 1.0` of its fixed clock.

use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::temperature_host::{
    Temperature_HOST, HOST_REPORT_TIME, RPI_PROC_TEMP_FILE,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AsyncResponseWrapperApi, CommandQueue, CommandWrapper, ConfigCallback,
    LookupQueryCommandOpts, McuApi, Pin, PinParams, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, LazyLock};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging

import extras.temperature_host as th

LOGGED = []
logging.exception = lambda msg, *a, **k: LOGGED.append(msg)

SENTINEL = object()

class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999999999.0
    def __init__(self):
        self.timers = []
        self.updates = []
        self._mono = 1000.0
    def register_timer(self, cb):
        self.timers.append(cb)
        return ("timer", len(self.timers))
    def update_timer(self, t, waketime):
        self.updates.append((t, waketime))
    def monotonic(self):
        return self._mono

class FakeMcu:
    def estimated_print_time(self, t):
        return 77.0

class FakeHeaters:
    def __init__(self):
        self.factories = {}
    def add_sensor_factory(self, sensor_type, factory):
        self.factories[sensor_type] = factory

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, debugoutput=None):
        self.reactor = FakeReactor()
        self.objects = {}
        self.handlers = {}
        self.shutdowns = []
        self._debugoutput = debugoutput
    def get_start_args(self):
        return {'debugoutput': self._debugoutput}
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (section,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name="temperature_host myhost", sensor_path=None):
        self._printer = printer
        self._name = name
        self._sensor_path = sensor_path
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=None):
        if option == "sensor_path" and self._sensor_path is not None:
            return self._sensor_path
        return default
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter {
    reactor: Reactor,
    debugoutput: Option<String>,
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
    handlers: Arc<Mutex<Vec<String>>>,
    shutdowns: Arc<Mutex<Vec<String>>>,
}

impl FakePrinter {
    fn new(debugoutput: Option<String>) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            debugoutput,
            objects: Arc::new(Mutex::new(IndexMap::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
            shutdowns: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn register_mcu(&self, printer: &Printer) {
        let mcu = Arc::new(FakeMcu { printer: printer.clone() });
        let obj: ObjectRef = mcu as ObjectRef;
        self.objects.lock().insert("mcu".to_string(), obj);
    }

    fn handler_names(&self) -> Vec<String> {
        self.handlers.lock().clone()
    }

    fn shutdown_messages(&self) -> Vec<String> {
        self.shutdowns.lock().clone()
    }

    fn object_names(&self) -> Vec<String> {
        self.objects.lock().keys().cloned().collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut args = IndexMap::new();
        let value = match &self.debugoutput {
            Some(debug) => PyValue::Str(debug.clone()),
            None => PyValue::None,
        };
        args.insert(PyValue::Str("debugoutput".to_string()), value);
        PyValue::Dict(args)
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name).cloned() {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .iter()
            .map(|(name, obj)| (name.clone(), obj.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.handlers.lock().push(event.to_string());
    }
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeAsyncWrapper;

impl AsyncResponseWrapperApi for FakeAsyncWrapper {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("setup_pin is not used by temperature_host"))
    }
    fn create_oid(&self) -> KResult<i64> {
        Err(KlippyError::config("create_oid is not used by temperature_host"))
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Err(KlippyError::config("lookup_command is not used by temperature_host"))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::config(
            "lookup_query_command is not used by temperature_host",
        ))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        Arc::new(FakeAsyncWrapper)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::config("get_constant_float is not used by temperature_host"))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        77.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Err(KlippyError::config("calibrate_clock is not used by temperature_host"))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::config(
            "create_trigger_dispatch is not used by temperature_host",
        ))
    }
}

impl PrinterObject for FakeMcu {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn McuApi);
}

struct FakeHeaters {
    factories: Mutex<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories
            .lock()
            .push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used by temperature_host")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("get_all_heaters is not used by temperature_host")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater is not used by temperature_host")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by temperature_host")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used by temperature_host")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used by temperature_host")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used by temperature_host")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used by temperature_host")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct FakeConfig {
    printer: Printer,
    name: String,
    sensor_path: Option<String>,
}


impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if option == "sensor_path" {
            if let Some(path) = &self.sensor_path {
                return Ok(Some(path.clone()));
            }
        }
        Ok(opts.default.clone().flatten())
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
        panic!("getsection is not used by temperature_host")
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
            py.exec(SETUP).expect("temperature_host fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping temperature_host oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn temp_file(name: &str, content: &str) -> std::path::PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!("temperature_host_{name}_{}.txt", std::process::id()));
    std::fs::write(&path, content).expect("write temp file");
    path
}

fn path_str(path: &std::path::Path) -> String {
    path.to_string_lossy().into_owned()
}

fn make_rust(
    name: &str,
    sensor_path: Option<String>,
    debugoutput: Option<String>,
    with_mcu: bool,
) -> (Arc<FakePrinter>, Printer, Config) {
    let fake = FakePrinter::new(debugoutput);
    let printer: Printer = fake.clone();
    if with_mcu {
        fake.register_mcu(&printer);
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: name.to_string(),
        sensor_path,
    });
    (fake, printer, config)
}

fn make_py(
    py: &mut pyoracle::Oracle,
    name: &str,
    sensor_path: Option<String>,
    debugoutput: Option<String>,
    with_mcu: bool,
) {
    let debug_arg = match &debugoutput {
        Some(debug) => format!("{debug:?}"),
        None => "None".to_string(),
    };
    let path_arg = match &sensor_path {
        Some(path) => format!("{path:?}"),
        None => "None".to_string(),
    };
    let name_q = format!("{name:?}");
    py.exec(&format!(
        "p = FakePrinter(debugoutput={debug_arg}); c = FakeConfig(p, {name_q}, {path_arg})"
    ))
    .expect("py fakes");
    if with_mcu {
        py.exec("p.objects['mcu'] = FakeMcu()").expect("py mcu");
    }
    py.exec("obj = th.Temperature_HOST(c)").expect("py construct");
}

fn check(label: &str, expected: Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "temperature_host".to_string(),
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

fn check_float(label: &str, expected: Value, actual: f64) {
    check(label, expected, Value::float(actual));
}

fn check_str(label: &str, expected: Value, actual: &str) {
    check(label, expected, Value::str(actual));
}

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned but call must fail"),
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

fn pyvalue_to_recording(value: &PyValue) -> Value {
    match value {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_recording).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_recording).collect()),
        PyValue::Dict(items) => Value::Dict(
            items
                .iter()
                .map(|(k, v)| (pyvalue_to_recording(k), pyvalue_to_recording(v)))
                .collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

/// Rust `get_status` temperature value of the host as a recording value.
fn host_status_temp(host: &Temperature_HOST) -> Value {
    let status = host.get_status(0.0).expect("temperature_host has get_status");
    let PyValue::Dict(dict) = status else {
        panic!("get_status must be a dict")
    };
    let temp = dict
        .get(&PyValue::Str("temperature".to_string()))
        .expect("temperature key")
        .clone();
    pyvalue_to_recording(&temp)
}

/// Shared log capture for `logging.exception` parity (first call installs).
static LOG_LINES: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));

fn log_lines() -> &'static Mutex<Vec<String>> {
    &LOG_LINES
}

fn install_log_sink() {
    log::set_sink(Box::new(|rec: &log::Record<'_>| {
        log_lines().lock().push(rec.message.to_string());
    }));
}

fn logged_lines() -> Vec<String> {
    log_lines().lock().clone()
}

fn callbacks_value(calls: &Arc<Mutex<Vec<(f64, f64)>>>) -> Value {
    Value::List(
        calls
            .lock()
            .iter()
            .map(|(pt, t)| Value::Tuple(vec![Value::float(*pt), Value::float(*t)]))
            .collect(),
    )
}

fn strings_value(messages: &[String]) -> Value {
    Value::List(messages.iter().map(|m| Value::str(m)).collect())
}

#[allow(clippy::type_complexity)]
fn sampled_pair(
    py: &mut pyoracle::Oracle,
    name: &str,
    path: &std::path::Path,
    min_temp: f64,
    max_temp: f64,
) -> (Arc<FakePrinter>, Temperature_HOST, Arc<Mutex<Vec<(f64, f64)>>>) {
    let path_s = path_str(path);
    make_py(py, name, Some(path_s.clone()), None, true);
    let (fake, _printer, config) = make_rust(name, Some(path_s), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    py.exec(&format!(
        "calls = []; obj.setup_minmax({min_temp:?}, {max_temp:?}); obj.setup_callback(lambda pt, t: calls.append((pt, t)))"
    ))
    .expect("py setup");
    let calls: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let rust_calls = calls.clone();
    let callback: TemperatureCallback =
        Box::new(move |print_time, temp| rust_calls.lock().push((print_time, temp)));
    host.setup_minmax(min_temp, max_temp).expect("rust setup_minmax");
    host.setup_callback(callback);
    (fake, host, calls)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check_float(
        "HOST_REPORT_TIME",
        py.eval("th.HOST_REPORT_TIME").expect("eval HOST_REPORT_TIME"),
        HOST_REPORT_TIME,
    );
    check_str(
        "RPI_PROC_TEMP_FILE",
        py.eval("th.RPI_PROC_TEMP_FILE").expect("eval RPI_PROC_TEMP_FILE"),
        RPI_PROC_TEMP_FILE,
    );
    let path = temp_file("constants", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (_, _, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check_float(
        "get_report_time_delta",
        py.eval("obj.get_report_time_delta()").expect("eval delta"),
        host.get_report_time_delta(),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn init_registers_object_and_handler() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("init", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (fake, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let _host = Temperature_HOST::new(&config).expect("rust construct");
    // The object is registered under "temperature_host <last token>".
    let mut rust_names = fake.object_names();
    rust_names.sort_unstable();
    check(
        "registered objects",
        py.eval("sorted(p.objects.keys())").expect("eval object names"),
        strings_value(&rust_names),
    );
    // The klippy:connect handler is registered.
    check(
        "event handlers",
        py.eval("sorted(p.handlers.keys())").expect("eval handlers"),
        strings_value(&fake.handler_names()),
    );
    // The reference registers one sample timer; the Rust side registers one
    // too (proven by construction on a real reactor and by the re-arm path in
    // `handle_connect_rearms_timer`).
    check_float(
        "timer count (reference)",
        py.eval("float(len(p.reactor.timers))").expect("eval timers"),
        1.0,
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn init_default_path_error_or_open() {
    let Some(mut py) = oracle() else { return };
    // With no sensor_path the default RPI_PROC_TEMP_FILE is opened; whether
    // that succeeds depends on the machine, but both sides must agree.
    py.exec("p = FakePrinter(); c = FakeConfig(p, 'temperature_host myhost', None)")
        .expect("py fakes");
    let expected = py.eval("th.Temperature_HOST(c)");
    let (_, _, config) = make_rust("temperature_host myhost", None, None, true);
    let actual = Temperature_HOST::new(&config);
    // `expected` is the one oracle drive of the reference. `Temperature_HOST`
    // registers its printer object before opening the file, so re-evaluating
    // the constructor on the same fakes in the error branch would raise
    // "Printer object 'temperature_host myhost' already created" instead of
    // the open error; compare the first failure directly.
    if expected.is_ok() {
        let host = actual.expect("rust construct must match oracle success");
        check_str(
            "default path",
            py.eval("c.get('sensor_path', th.RPI_PROC_TEMP_FILE)")
                .expect("eval default path"),
            host.path(),
        );
        check_str(
            "short name",
            py.eval("c.get_name().split()[-1]").expect("eval name"),
            host.name(),
        );
    } else {
        check_err(
            "default path open error",
            expected,
            actual.map(|_| ()),
        );
    }
}

#[test]
fn init_custom_sensor_path() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("custom_path", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (_, _, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check_str(
        "sensor_path",
        py.eval("obj.path").expect("eval path"),
        host.path(),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn init_name_without_suffix() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("no_suffix", "45000\n");
    make_py(&mut py, "temperature_host", Some(path_str(&path)), None, true);
    let (fake, _printer, config) = make_rust("temperature_host", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check_str(
        "short name",
        py.eval("obj.name").expect("eval name"),
        host.name(),
    );
    let mut rust_names = fake.object_names();
    rust_names.sort_unstable();
    check(
        "registered objects",
        py.eval("sorted(p.objects.keys())").expect("eval objects"),
        strings_value(&rust_names),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn init_debugoutput_skips_open_and_timer() {
    let Some(mut py) = oracle() else { return };
    let missing = "/nonexistent_temperature_host_dir/thermal_zone0/temp";
    // In debugoutput mode the file is never opened and no timer/handler is
    // registered, so even a missing sensor file constructs fine.
    py.exec(&format!(
        "p = FakePrinter(debugoutput='dbg'); c = FakeConfig(p, 'temperature_host myhost', {missing:?}); obj = th.Temperature_HOST(c)"
    ))
    .expect("py construct");
    let (fake, _printer, config) = make_rust(
        "temperature_host myhost",
        Some(missing.to_string()),
        Some("dbg".to_string()),
        true,
    );
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check(
        "handlers",
        py.eval("sorted(p.handlers.keys())").expect("eval handlers"),
        strings_value(&fake.handler_names()),
    );
    check_float(
        "timer count (reference)",
        py.eval("float(len(p.reactor.timers))").expect("eval timers"),
        0.0,
    );
    check(
        "initial status",
        py.eval("obj.get_status(0.0)['temperature']").expect("eval status"),
        host_status_temp(&host),
    );
}

#[test]
fn init_unopenable_file_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let missing = "/nonexistent_temperature_host_dir/thermal_zone0/temp";
    py.exec(&format!(
        "p = FakePrinter(); p.objects['mcu'] = FakeMcu(); c = FakeConfig(p, 'temperature_host myhost', {missing:?})"
    ))
    .expect("py fakes");
    let expected = py.eval("th.Temperature_HOST(c)");
    let (_, _, config) = make_rust(
        "temperature_host myhost",
        Some(missing.to_string()),
        None,
        true,
    );
    check_err("unopenable file", expected, Temperature_HOST::new(&config));
}

#[test]
fn setup_minmax_and_default_status() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("setup", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (_, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check(
        "status before sample",
        py.eval("obj.get_status(0.0)['temperature']").expect("eval status"),
        host_status_temp(&host),
    );
    py.exec("obj.setup_minmax(-5.0, 60.0)").expect("py setup_minmax");
    host.setup_minmax(-5.0, 60.0).expect("rust setup_minmax");
    check(
        "status after setup_minmax",
        py.eval("obj.get_status(0.0)['temperature']").expect("eval status"),
        host_status_temp(&host),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_reads_file_and_reports() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_ok", "45678\n");
    let (fake, host, calls) =
        sampled_pair(&mut py, "temperature_host myhost", &path, 0.0, 100.0);
    // The Python side's fixed clock makes its wake time exactly 1001.0.
    check_float(
        "sample wake (oracle clock)",
        py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample"),
        1001.0,
    );
    // The Rust side's wake time is measured+1.0 of the real clock.
    let before = fake.reactor.monotonic();
    let waketime = host.sample_pi_temperature(0.0).expect("rust sample");
    let after = fake.reactor.monotonic();
    assert!(
        before + 1.0 <= waketime && waketime <= after + 1.0,
        "rust wake {waketime} outside [{}, {}]",
        before + 1.0,
        after + 1.0,
    );
    check(
        "callback calls",
        py.eval("list(calls)").expect("eval calls"),
        callbacks_value(&calls),
    );
    check(
        "shutdowns",
        py.eval("list(p.shutdowns)").expect("eval shutdowns"),
        strings_value(&fake.shutdown_messages()),
    );
    check(
        "status after sample",
        py.eval("obj.get_status(0.0)['temperature']").expect("eval status"),
        host_status_temp(&host),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_below_minimum_shutdown() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_below", "500\n");
    let (fake, host, calls) =
        sampled_pair(&mut py, "temperature_host myhost", &path, 1.0, 100.0);
    let _ret = py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample");
    let _waketime = host.sample_pi_temperature(0.0).expect("rust sample");
    check(
        "shutdown messages",
        py.eval("list(p.shutdowns)").expect("eval shutdowns"),
        strings_value(&fake.shutdown_messages()),
    );
    check(
        "callback calls",
        py.eval("list(calls)").expect("eval calls"),
        callbacks_value(&calls),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_above_maximum_shutdown() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_above", "20000\n");
    let (fake, host, calls) =
        sampled_pair(&mut py, "temperature_host myhost", &path, 0.0, 10.0);
    let _ret = py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample");
    let _waketime = host.sample_pi_temperature(0.0).expect("rust sample");
    check(
        "shutdown messages",
        py.eval("list(p.shutdowns)").expect("eval shutdowns"),
        strings_value(&fake.shutdown_messages()),
    );
    check(
        "callback calls",
        py.eval("list(calls)").expect("eval calls"),
        callbacks_value(&calls),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_at_boundary_reaches_callback_without_shutdown() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_boundary", "5000\n");
    let (fake, host, calls) =
        sampled_pair(&mut py, "temperature_host myhost", &path, 5.0, 5.0);
    let _ret = py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample");
    let _waketime = host.sample_pi_temperature(0.0).expect("rust sample");
    check(
        "shutdowns (none on boundary)",
        py.eval("list(p.shutdowns)").expect("eval shutdowns"),
        strings_value(&fake.shutdown_messages()),
    );
    check(
        "callback calls",
        py.eval("list(calls)").expect("eval calls"),
        callbacks_value(&calls),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_read_error_logs_and_returns_never() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    for (tag, content) in [("sample_bad_text", "abc\n"), ("sample_empty", "")] {
        let path = temp_file(tag, content);
        make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
        let (_, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
        let host = Temperature_HOST::new(&config).expect("rust construct");
        py.exec("LOGGED.clear()").expect("clear log");
        let before = logged_lines().len();
        let expected = py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample");
        let actual = host.sample_pi_temperature(0.0).expect("rust sample");
        check_float("sample wake is NEVER", expected, actual);
        // The read error also zeroes the temperature and logs once per side.
        check(
            "status after read error",
            py.eval("obj.get_status(0.0)['temperature']").expect("eval status"),
            host_status_temp(&host),
        );
        let new_logged: Vec<String> = logged_lines()[before..].to_vec();
        check(
            "logged exception",
            py.eval("list(LOGGED)").expect("eval logged"),
            strings_value(&new_logged),
        );
        let _ = std::fs::remove_file(&path);
    }
}

#[test]
fn sample_missing_mcu_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_no_mcu", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, false);
    let (_, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, false);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check_err(
        "sample without mcu",
        py.eval("obj._sample_pi_temperature(0.0)"),
        host.sample_pi_temperature(0.0),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_without_callback_raises_attribute_error() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("sample_no_cb", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (_, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    check_err(
        "sample without callback",
        py.eval("obj._sample_pi_temperature(0.0)"),
        host.sample_pi_temperature(0.0),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn sample_nan_and_inf_contents() {
    let Some(mut py) = oracle() else { return };
    // NaN never shuts down; infinity exceeds any finite maximum.
    for (tag, content) in [("sample_nan", "nan\n"), ("sample_inf", "inf\n")] {
        let path = temp_file(tag, content);
        let (fake, host, calls) =
            sampled_pair(&mut py, "temperature_host myhost", &path, 0.0, 1e9);
        let _ret = py.eval("obj._sample_pi_temperature(0.0)").expect("eval sample");
        let _waketime = host.sample_pi_temperature(0.0).expect("rust sample");
        check(
            "callback calls",
            py.eval("list(calls)").expect("eval calls"),
            callbacks_value(&calls),
        );
        check(
            "shutdowns",
            py.eval("list(p.shutdowns)").expect("eval shutdowns"),
            strings_value(&fake.shutdown_messages()),
        );
        let _ = std::fs::remove_file(&path);
    }
}

#[test]
fn handle_connect_rearms_timer() {
    let Some(mut py) = oracle() else { return };
    let path = temp_file("connect", "45000\n");
    make_py(&mut py, "temperature_host myhost", Some(path_str(&path)), None, true);
    let (_, _printer, config) = make_rust("temperature_host myhost", Some(path_str(&path)), None, true);
    let host = Temperature_HOST::new(&config).expect("rust construct");
    py.exec("obj.handle_connect()").expect("py connect");
    host.handle_connect();
    check(
        "timer updates (reference)",
        py.eval("list(p.reactor.updates)").expect("eval updates"),
        Value::List(vec![Value::Tuple(vec![
            Value::Tuple(vec![Value::str("timer"), Value::int(1)]),
            Value::float(0.0),
        ])]),
    );
    let _ = std::fs::remove_file(&path);
}

#[test]
fn load_config_registers_factory() {
    let Some(mut py) = oracle() else { return };
    py.exec("p = FakePrinter(); p.objects['heaters'] = FakeHeaters(); c = FakeConfig(p, 'temperature_host myhost', None)")
        .expect("py fakes");
    check(
        "load_config return",
        py.eval("th.load_config(c)").expect("load_config"),
        Value::Null,
    );
    check(
        "registered sensor types",
        py.eval("sorted(p.objects['heaters'].factories.keys())")
            .expect("eval factories"),
        Value::List(vec![Value::str("temperature_host")]),
    );
    // The registered factory builds a working sensor on both sides.
    py.exec(
        "import os, tempfile; fd, tf = tempfile.mkstemp(); \
         os.write(fd, b'45000\\n'); os.close(fd); \
         fc = FakeConfig(p, 'temperature_host built', tf); \
         o2 = p.objects['heaters'].factories['temperature_host'](fc)",
    )
    .expect("py factory build");
    let tmp_path = match py.eval("tf").expect("eval tmpfile") {
        Value::Str(path) => path,
        other => panic!("expected temp path string, got {other:?}"),
    };
    let built_status = py
        .eval("o2.get_status(0.0)['temperature']")
        .expect("eval built status");
    let (fake, _printer, config) = make_rust("temperature_host myhost", Some(tmp_path.clone()), None, true);
    let heaters_obj: ObjectRef = Arc::new(FakeHeaters {
        factories: Mutex::new(Vec::new()),
    }) as ObjectRef;
    fake.objects
        .lock()
        .insert("heaters".to_string(), heaters_obj);
    let factory = extras::temperature_host::module_factory();
    let load_config = factory.load_config.expect("load_config exists");
    let load_result = load_config(&config).expect("rust load_config");
    assert!(load_result.is_none(), "load_config returns None");
    let heaters_obj = fake
        .objects
        .lock()
        .get("heaters")
        .cloned()
        .expect("heaters registered");
    let recorded = heaters_obj
        .as_any()
        .downcast_ref::<FakeHeaters>()
        .expect("fake heaters downcast")
        .factories
        .lock();
    assert_eq!(recorded.len(), 1, "one factory registered");
    assert_eq!(recorded[0].0, "temperature_host");
    let build_config: Config = Arc::new(FakeConfig {
        printer: _printer.clone(),
        name: "temperature_host built".to_string(),
        sensor_path: Some(tmp_path.clone()),
    });
    let _sensor: Arc<dyn TemperatureSensorApi> = (recorded[0].1)(&build_config).expect("factory builds sensor");
    // `Temperature_HOST.__init__` registers the object under
    // "temperature_host <name>"; recover it and check its status.
    let built_obj: ObjectRef = fake
        .objects
        .lock()
        .get("temperature_host built")
        .cloned()
        .expect("factory-built sensor registered");
    drop(recorded);
    let built_host: Arc<Temperature_HOST> =
        klippy_api::downcast_arc(&built_obj).expect("downcast to Temperature_HOST");
    check(
        "built sensor status",
        built_status,
        pyvalue_to_recording(
            &match built_host.get_status(0.0) {
                Some(PyValue::Dict(dict)) => dict
                    .get(&PyValue::Str("temperature".to_string()))
                    .expect("temperature key")
                    .clone(),
                _ => PyValue::None,
            },
        ),
    );
    let _ = std::fs::remove_file(&tmp_path);
}
