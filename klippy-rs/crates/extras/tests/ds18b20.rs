//! Parity tests for `extras::ds18b20` (port of `klippy/extras/ds18b20.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): both sides are built with minimal
//! Python / Rust fakes for printer, config, MCU and heaters, fed identical
//! inputs (config options, min/max, serial-response dicts), and the
//! resulting MCU config commands, temperature callbacks, log lines,
//! shutdown messages and errors are compared exactly (floats by bits
//! through `recording::assert_equal`, errors by class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply.

use extras::ds18b20::{
    module_factory, sensor_factory, DS18B20, DS18_MAX_CONSECUTIVE_ERRORS, DS18_MIN_REPORT_TIME,
    DS18_REPORT_TIME,
};
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueue, CommandWrapper, ConfigCallback, LookupQueryCommandOpts, McuApi, Pin, PinParams,
    SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use kos::sync::Mutex;
use std::sync::Arc;

use indexmap::IndexMap;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging
import extras.ds18b20 as ds18b20
import mcu as mcu_mod

class _Sentinel:
    pass
SENTINEL = _Sentinel()

log_lines = []
class _ListHandler(logging.Handler):
    def emit(self, record):
        try:
            log_lines.append(record.getMessage())
        except Exception:
            log_lines.append("LOG_FORMAT_ERROR")
logging.getLogger().addHandler(_ListHandler())
logging.getLogger().setLevel(logging.INFO)

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.async_shutdowns = []
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is not SENTINEL:
            return default
        raise configparser.Error("Unknown config object '%s'" % (name,))
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is not SENTINEL:
            return default
        raise configparser.Error("Unknown config object '%s'" % (section,))
    def invoke_async_shutdown(self, msg, details=None):
        if details is None:
            details = {}
        self.async_shutdowns.append(msg)
    def get_start_args(self):
        return {}
    def get_reactor(self):
        return None
    def is_shutdown(self):
        return False

class FakeMcu:
    def __init__(self, printer):
        self._printer = printer
        self.next_oid = 1
        self.cmds = []
        self.serial_cb = None
        self.serial_msg = None
        self.serial_oid = None
        self.config_cb = None
    def get_printer(self):
        return self._printer
    def create_oid(self):
        oid = self.next_oid
        self.next_oid += 1
        return oid
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_cb = cb
        self.serial_msg = msg
        self.serial_oid = oid
        return None
    def register_config_callback(self, cb):
        self.config_cb = cb
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.cmds.append((cmd, is_init))
    def get_query_slot(self, oid):
        return 5000 + oid * 7
    def seconds_to_clock(self, time):
        return int(time * 1000)
    def clock32_to_clock64(self, clock32):
        return clock32
    def clock_to_print_time(self, clock):
        return clock / 1000.0

def _fake_get_printer_mcu(printer, name):
    objname = "mcu" if name == "mcu" else "mcu " + name
    if objname in printer.objects:
        return printer.objects[objname]
    raise configparser.Error("Unknown config object '%s'" % (objname,))
mcu_mod.get_printer_mcu = _fake_get_printer_mcu

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, opts):
        self._printer = printer
        self._name = name
        self._opts = opts
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=SENTINEL, note_valid=True):
        if option not in self._opts:
            if default is not SENTINEL:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return self._opts[option]
    def getfloat(self, option, default=SENTINEL, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self._opts:
            if default is not SENTINEL:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        try:
            v = float(self._opts[option])
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v

class FakeHeaters:
    def __init__(self):
        self.sensor_factories = {}
    def add_sensor_factory(self, stype, factory):
        self.sensor_factories[stype] = factory

class FakeCallback:
    def __init__(self):
        self.calls = []
    def __call__(self, read_time, temp):
        self.calls.append((read_time, temp))
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    async_shutdowns: GCell<Vec<String>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter {
            objects: GCell::new(IndexMap::new()),
            async_shutdowns: GCell::new(Vec::new()),
        }
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
        let mut m = self.objects.borrow_mut();
        if m.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        m.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.borrow();
        match module {
            None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(module) => {
                let prefix = format!("{module} ");
                let mut out = Vec::new();
                if let Some(v) = objs.get(module) {
                    out.push((module.to_string(), v.clone()));
                }
                for (k, v) in objs.iter() {
                    if k.starts_with(&prefix) {
                        out.push((k.clone(), v.clone()));
                    }
                }
                out
            }
        }
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
        self.async_shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.async_shutdowns.borrow_mut().push(msg.to_string());
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    opts: IndexMap<String, String>,
}

impl FakeConfig {
    fn present(&self, option: &str) -> Option<String> {
        self.opts.get(option).cloned()
    }
    fn missing<T>(&self, option: &str) -> KResult<Option<T>> {
        Err(KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        )))
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
        if let Some(v) = self.present(option) {
            return Ok(Some(v));
        }
        match opts.default {
            None => self.missing(option),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.present(option) {
            Some(_) => panic!("FakeConfig getint is not used by ds18b20"),
            None => match opts.default {
                None => self.missing(option),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.present(option) {
            Some(s) => s,
            None => {
                return match opts.default {
                    None => self.missing(option),
                    Some(None) => Ok(None),
                    Some(Some(v)) => Ok(Some(v)),
                };
            }
        };
        let v = match pycompat::text::float_from_string(&raw) {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                )));
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    pycompat::text::float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    pycompat::text::float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    pycompat::text::float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    pycompat::text::float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.present(option) {
            Some(_) => panic!("FakeConfig getboolean is not used by ds18b20"),
            None => match opts.default {
                None => self.missing(option),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("FakeConfig getchoice is not used by ds18b20")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("FakeConfig getlists is not used by ds18b20")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("FakeConfig getlist is not used by ds18b20")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("FakeConfig getintlist is not used by ds18b20")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("FakeConfig getfloatlist is not used by ds18b20")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("FakeConfig getsection is not used by ds18b20")
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

struct FakeAsyncResp;
impl AsyncResponseWrapperApi for FakeAsyncResp {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
    oid_counter: GCell<i64>,
    cmds: GCell<Vec<(String, bool)>>,
    serial_cb: GCell<Option<SerialResponseCallback>>,
    serial_msg: GCell<String>,
    serial_oid: GCell<Option<i64>>,
    config_cb: GCell<Option<ConfigCallback>>,
}

impl FakeMcu {
    fn new(printer: Printer) -> Self {
        FakeMcu {
            printer,
            oid_counter: GCell::new(1),
            cmds: GCell::new(Vec::new()),
            serial_cb: GCell::new(None),
            serial_msg: GCell::new(String::new()),
            serial_oid: GCell::new(None),
            config_cb: GCell::new(None),
        }
    }
    fn cmds_snapshot(&self) -> Vec<(String, bool)> {
        self.cmds.borrow().clone()
    }
    fn cmds_joined(&self) -> String {
        self.cmds
            .borrow()
            .iter()
            .map(|(c, init)| format!("{}|{}", c, if *init { 1 } else { 0 }))
            .collect::<Vec<_>>()
            .join(";")
    }
    fn serial_info(&self) -> (String, Option<i64>) {
        (self.serial_msg.borrow().clone(), *self.serial_oid.borrow())
    }
    fn has_config_cb(&self) -> bool {
        self.config_cb.borrow().is_some()
    }
    fn invoke_config_cb(&self) -> KResult<()> {
        let mut cb = self.config_cb.replace(None);
        match cb.as_mut() {
            Some(f) => f(),
            None => panic!("FakeMcu has no config callback"),
        }
    }
    fn invoke_serial(&self, params: PyValue) {
        let cb = self.serial_cb.borrow().clone();
        match cb {
            Some(cb) => cb(params),
            None => panic!("FakeMcu has no serial callback"),
        }
    }
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
        Err(KlippyError::Mcu("FakeMcu setup_pin unused".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        let oid = self.oid_counter.get();
        self.oid_counter.set(oid + 1);
        Ok(oid)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.config_cb.replace(Some(cb));
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.cmds.borrow_mut().push((cmd.to_string(), opts.is_init));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        5000 + oid * 7
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        kos::cconv::f64_to_i64(time * 1000.0)
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
        Err(KlippyError::Mcu(
            "FakeMcu lookup_command unused".to_string(),
        ))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu(
            "FakeMcu lookup_query_command unused".to_string(),
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
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.serial_cb.replace(Some(cb));
        self.serial_msg.replace(msg.to_string());
        self.serial_oid.set(oid);
        Arc::new(FakeAsyncResp)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        Err(KlippyError::Mcu(format!("constant {name} not found")))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        kos::cconv::f64_to_i64(print_time * 1000.0)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Mcu(
            "FakeMcu create_trigger_dispatch unused".to_string(),
        ))
    }
}
klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeHeaters {
    factories: GCell<IndexMap<String, SensorFactory>>,
}

impl FakeHeaters {
    fn new() -> Self {
        FakeHeaters {
            factories: GCell::new(IndexMap::new()),
        }
    }
    fn has(&self, name: &str) -> bool {
        self.factories.borrow().contains_key(name)
    }
    fn factory(&self, name: &str) -> Option<SensorFactory> {
        self.factories.borrow().get(name).cloned()
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories
            .borrow_mut()
            .insert(sensor_type.to_string(), factory);
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("FakeHeaters setup_heater is not used by ds18b20")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("FakeHeaters get_all_heaters is not used by ds18b20")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("FakeHeaters lookup_heater is not used by ds18b20")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("FakeHeaters setup_sensor is not used by ds18b20")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("FakeHeaters register_sensor is not used by ds18b20")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("FakeHeaters register_monitor is not used by ds18b20")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("FakeHeaters turn_off_all_heaters is not used by ds18b20")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("FakeHeaters set_temperature is not used by ds18b20")
    }
}
klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

#[derive(Clone)]
struct Collector {
    calls: Arc<Mutex<Vec<(f64, f64)>>>,
}

impl Collector {
    fn new() -> Self {
        Collector {
            calls: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn record(&self, t: f64, temp: f64) {
        self.calls.lock().push((t, temp));
    }
    fn snapshot(&self) -> Vec<(f64, f64)> {
        self.calls.lock().clone()
    }
}

// Captured log lines of the Rust side on this test's thread; the sink is
// installed once per process and parallel tests keep their own lines.
thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_capture() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec: &klippy_api::log::Record<'_>| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn captured_logs() -> Vec<(String, String)> {
    LOGS.with(|logs| logs.borrow().clone())
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("ds18b20 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping ds18b20 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "ds18b20".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, err: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class",
    );
    assert_eq!(
        err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message",
    );
}

fn err<T>(r: KResult<T>) -> KlippyError {
    match r {
        Ok(_) => panic!("expected an error but got Ok"),
        Err(e) => e,
    }
}

fn params_dict(value: i64, next_clock: i64, fault: i64) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(
        PyValue::Str("value".to_string()),
        PyValue::Int(PyInt::from_i64(value)),
    );
    d.insert(
        PyValue::Str("next_clock".to_string()),
        PyValue::Int(PyInt::from_i64(next_clock)),
    );
    d.insert(
        PyValue::Str("fault".to_string()),
        PyValue::Int(PyInt::from_i64(fault)),
    );
    d.insert(
        PyValue::Str("oid".to_string()),
        PyValue::Int(PyInt::from_i64(1)),
    );
    PyValue::Dict(d)
}

fn temp_of_status(sensor: &DS18B20) -> f64 {
    let status = PrinterObject::get_status(sensor, 0.0).expect("ds18b20 get_status present");
    match status {
        PyValue::Dict(d) => match d.get(&PyValue::Str("temperature".to_string())) {
            Some(PyValue::Float(f)) => *f,
            other => panic!("temperature field is not a float: {other:?}"),
        },
        other => panic!("get_status is not a dict: {other:?}"),
    }
}

fn make_opts(pairs: &[(&str, &str)]) -> IndexMap<String, String> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

fn make_rust(
    name: &str,
    opts: &[(&str, &str)],
) -> (
    Arc<DS18B20>,
    Arc<FakeMcu>,
    Arc<FakePrinter>,
    Config,
    Arc<FakeHeaters>,
) {
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu.clone() as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters.clone() as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: name.to_string(),
        opts: make_opts(opts),
    });
    let sensor = DS18B20::new(&config).expect("ds18b20 new");
    (sensor, mcu, printer, config, heaters)
}

fn py_opts_expr(pairs: &[(&str, &str)]) -> String {
    let inner: Vec<String> = pairs.iter().map(|(k, v)| format!("'{k}': '{v}'")).collect();
    format!("{{{}}}", inner.join(", "))
}

fn fresh_pair(
    py: &mut pyoracle::Oracle,
    name: &str,
    opts: &[(&str, &str)],
) -> (
    Arc<DS18B20>,
    Arc<FakeMcu>,
    Arc<FakePrinter>,
    Config,
    Arc<FakeHeaters>,
    Collector,
) {
    let py_code = format!(
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, '{name}', {opts}); s = ds18b20.DS18B20(c); cb = FakeCallback(); s.setup_callback(cb);",
        name = name,
        opts = py_opts_expr(opts),
    );
    py.exec(&py_code).expect("py fresh sensor");
    let (sensor, mcu, printer, config, heaters) = make_rust(name, opts);
    let collector = Collector::new();
    let cc = collector.clone();
    sensor.setup_callback(Box::new(move |t: f64, temp: f64| cc.record(t, temp))
        as TemperatureCallback);
    (sensor, mcu, printer, config, heaters, collector)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check(
        "DS18_REPORT_TIME",
        py.eval("ds18b20.DS18_REPORT_TIME"),
        Value::float(DS18_REPORT_TIME),
    );
    check(
        "DS18_MIN_REPORT_TIME",
        py.eval("ds18b20.DS18_MIN_REPORT_TIME"),
        Value::float(DS18_MIN_REPORT_TIME),
    );
    check(
        "DS18_MAX_CONSECUTIVE_ERRORS",
        py.eval("ds18b20.DS18_MAX_CONSECUTIVE_ERRORS"),
        Value::int(DS18_MAX_CONSECUTIVE_ERRORS),
    );
}

#[test]
fn construction_registers_oid_serial_and_config() {
    let Some(mut py) = oracle() else { return };
    let (sensor, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "28-000004abc123"), ("sensor_mcu", "mcu")],
    );
    check("oid", py.eval("s.oid"), Value::int(sensor.oid()));
    check(
        "serial msg",
        py.eval("mc.serial_msg"),
        Value::str(&mcu.serial_info().0),
    );
    check(
        "serial oid",
        py.eval("mc.serial_oid"),
        Value::int(mcu.serial_info().1.unwrap()),
    );
    check(
        "has config cb",
        py.eval("mc.config_cb is not None"),
        Value::Bool(mcu.has_config_cb()),
    );
    check("name", py.eval("s.name"), Value::str(sensor.name()));
    let expected_bytes: Vec<u8> = b"28-000004abc123".to_vec();
    check(
        "sensor id repr",
        py.eval("bytes(s.sensor_id)"),
        Value::bytes(&expected_bytes),
    );
}

#[test]
fn construction_uses_report_time_default() {
    let Some(mut py) = oracle() else { return };
    let (sensor, _mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "2800000000000000"), ("sensor_mcu", "mcu")],
    );
    check(
        "report_time default",
        py.eval("s.report_time"),
        Value::float(sensor.get_report_time_delta()),
    );
    let (sensor2, _mcu2, _p2, _c2, _h2, _cb2) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[
            ("serial_no", "2800000000000000"),
            ("sensor_mcu", "mcu"),
            ("ds18_report_time", "2.5"),
        ],
    );
    check(
        "report_time custom",
        py.eval("s.report_time"),
        Value::float(sensor2.get_report_time_delta()),
    );
}

#[test]
fn build_config_default_minmax() {
    let Some(mut py) = oracle() else { return };
    let (_s, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "28-000004abc123"), ("sensor_mcu", "mcu")],
    );
    py.exec("mc.config_cb()").expect("py build_config");
    mcu.invoke_config_cb().expect("rust build_config");
    check(
        "build cmds default",
        py.eval("\";\".join([\"%s|%d\" % (c, 1 if i else 0) for (c, i) in mc.cmds])"),
        Value::str(&mcu.cmds_joined()),
    );
    assert_eq!(mcu.cmds_snapshot().len(), 2, "build emits two cmds");
    assert!(!mcu.cmds_snapshot()[0].1, "config cmd is not init");
    assert!(mcu.cmds_snapshot()[1].1, "query cmd is init");
}

#[test]
fn build_config_custom_minmax_and_report_time() {
    let Some(mut py) = oracle() else { return };
    let (sensor, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[
            ("serial_no", "28-000004abc123"),
            ("sensor_mcu", "mcu"),
            ("ds18_report_time", "2.5"),
        ],
    );
    py.exec("s.setup_minmax(10.5, 120.25)").expect("py minmax");
    sensor.setup_minmax(10.5, 120.25).expect("rust minmax");
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");
    check(
        "build cmds custom",
        py.eval("\";\".join([\"%s|%d\" % (c, 1 if i else 0) for (c, i) in mc.cmds])"),
        Value::str(&mcu.cmds_joined()),
    );
}

#[test]
fn build_config_truncates_scaled_temps_toward_zero() {
    let Some(mut py) = oracle() else { return };
    // Values chosen so `*1000` is fractional and `%d` truncates toward zero:
    // -123.4567 -> -123456 (toward zero), 12.9999 -> 12999; rounding would differ.
    let (sensor, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "abcdef"), ("sensor_mcu", "mcu")],
    );
    py.exec("s.setup_minmax(-123.4567, 12.9999)")
        .expect("py minmax");
    sensor.setup_minmax(-123.4567, 12.9999).expect("rust minmax");
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");
    check(
        "build cmds truncation",
        py.eval("\";\".join([\"%s|%d\" % (c, 1 if i else 0) for (c, i) in mc.cmds])"),
        Value::str(&mcu.cmds_joined()),
    );
}

#[test]
fn serial_id_hex_encoding() {
    let Some(mut py) = oracle() else { return };
    // Serial numbers are raw bytes of the UTF-8 encoding, hex-encoded
    // two lowercase digits per byte. Include a multi-byte character so the
    // byte (not char) encoding is exercised: "Aé" -> 41 c3 a9.
    let (sensor, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "A\u{e9}"), ("sensor_mcu", "mcu")],
    );
    check(
        "serial bytes",
        py.eval("bytes(s.sensor_id)"),
        Value::bytes(sensor.sensor_id()),
    );
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");
    check(
        "serial hex cmd",
        py.eval("\";\".join([\"%s|%d\" % (c, 1 if i else 0) for (c, i) in mc.cmds])"),
        Value::str(&mcu.cmds_joined()),
    );
    assert!(
        mcu.cmds_joined().contains("serial=41c3a9"),
        "encoded serial is hex of UTF-8 bytes"
    );
}

#[test]
fn handle_response_success() {
    let Some(mut py) = oracle() else { return };
    let (_sensor, mcu, _p, _c, _h, collector) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "test"), ("sensor_mcu", "mcu")],
    );
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");

    let cases = [
        (
            25000i64,
            10000i64,
            "25000 -> 25.0, clock 10000, report 3000 -> 7.0",
        ),
        (0i64, 3000i64, "zero value, clock == report -> 0.0"),
        (-12345i64, 50000i64, "negative value -12.345"),
        (1000000i64, 999999i64, "large value 1000.0"),
    ];
    for (value, next_clock, label) in cases {
        py.exec("cb.calls.clear()").expect("clear py calls");
        let before = collector.snapshot().len();
        let expr = format!(
            "mc.serial_cb({{'value': {value}, 'next_clock': {next_clock}, 'fault': 0, 'oid': 1}})"
        );
        py.exec(&expr).expect("py serial");
        mcu.invoke_serial(params_dict(value, next_clock, 0));
        let calls = collector.snapshot();
        assert_eq!(calls.len(), before + 1, "{label}: one callback");
        let (rt, temp) = calls[calls.len() - 1];
        let expected = py.eval("cb.calls[-1]");
        check(
            &format!("handle success {label}"),
            expected,
            Value::Tuple(vec![Value::float(rt), Value::float(temp)]),
        );
    }
}

#[test]
fn handle_response_fault_logs_and_skips_callback() {
    let Some(mut py) = oracle() else { return };
    install_log_capture();
    let (_s, mcu, _p, _c, _h, collector) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "test"), ("sensor_mcu", "mcu")],
    );
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");
    py.exec("log_lines.clear()").expect("clear py logs");

    let faults = [(25000i64, 42i64), (-500i64, 43i64), (12345i64, 1i64)];
    for (value, fault) in faults {
        py.exec("cb.calls.clear()").expect("clear py calls");
        collector.calls.lock().clear();
        let expr = format!(
            "mc.serial_cb({{'value': {value}, 'next_clock': 10000, 'fault': {fault}, 'oid': 1}})"
        );
        py.exec(&expr).expect("py serial fault");
        mcu.invoke_serial(params_dict(value, 10000, fault));
        // Callback never fires on fault.
        check(
            "fault keeps py callback empty",
            py.eval("len(cb.calls)"),
            Value::int(0),
        );
        assert!(
            collector.snapshot().is_empty(),
            "rust callback must not fire on fault"
        );
    }
    let py_logs = py.eval("\";\".join(log_lines)").expect("py logs");
    let rust_logs = captured_logs().iter().map(|(_, m)| m.clone()).collect::<Vec<_>>().join(";");
    check("fault log lines", Ok(py_logs), Value::str(&rust_logs));
}

#[test]
fn handle_response_missing_callback_is_attribute_error() {
    let Some(mut py) = oracle() else { return };
    // Build without setup_callback on both sides.
    let py_code =
        "p2 = FakePrinter(); mc2 = FakeMcu(p2); p2.objects['mcu'] = mc2; h2 = FakeHeaters(); p2.objects['heaters'] = h2; c2 = FakeConfig(p2, 'temperature_sensor my_sensor', {'serial_no': 'x', 'sensor_mcu': 'mcu'}); s2 = ds18b20.DS18B20(c2); mc2.config_cb()";
    py.exec(py_code).expect("py no-callback sensor");
    let (sensor, mcu, _pr, _cfg, _ht) = make_rust(
        "temperature_sensor my_sensor",
        &[("serial_no", "x"), ("sensor_mcu", "mcu")],
    );
    mcu.invoke_config_cb().expect("rust build");
    let py_expr = "s2._handle_ds18b20_response({'value': 25000, 'next_clock': 10000, 'fault': 0})";
    let expected = py.eval(py_expr);
    let actual = sensor.handle_response(&params_dict(25000, 10000, 0));
    check_err("missing callback", expected, err(actual));
}

#[test]
fn handle_response_missing_keys_are_key_errors() {
    let Some(mut py) = oracle() else { return };
    let (sensor, _mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "x"), ("sensor_mcu", "mcu")],
    );
    let cases: [(&str, PyValue); 3] = [
        ("{'next_clock': 10000, 'fault': 0}", {
            let mut m = IndexMap::new();
            m.insert(
                PyValue::Str("next_clock".to_string()),
                PyValue::Int(PyInt::from_i64(10000)),
            );
            m.insert(
                PyValue::Str("fault".to_string()),
                PyValue::Int(PyInt::from_i64(0)),
            );
            PyValue::Dict(m)
        }),
        ("{'value': 25000, 'next_clock': 10000}", {
            let mut m = IndexMap::new();
            m.insert(
                PyValue::Str("value".to_string()),
                PyValue::Int(PyInt::from_i64(25000)),
            );
            m.insert(
                PyValue::Str("next_clock".to_string()),
                PyValue::Int(PyInt::from_i64(10000)),
            );
            PyValue::Dict(m)
        }),
        ("{'value': 25000, 'fault': 0}", {
            let mut m = IndexMap::new();
            m.insert(
                PyValue::Str("value".to_string()),
                PyValue::Int(PyInt::from_i64(25000)),
            );
            m.insert(
                PyValue::Str("fault".to_string()),
                PyValue::Int(PyInt::from_i64(0)),
            );
            PyValue::Dict(m)
        }),
    ];
    for (dict, params) in &cases {
        let py_expr = format!("s._handle_ds18b20_response({dict})");
        let expected = py.eval(&py_expr);
        let actual = sensor.handle_response(params);
        check_err(&format!("missing key dict {dict}"), expected, err(actual));
    }
}

#[test]
fn get_status_temperature_stays_zero() {
    let Some(mut py) = oracle() else { return };
    let (sensor, mcu, _p, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "test"), ("sensor_mcu", "mcu")],
    );
    let expected = py.eval("s.get_status(0)['temperature']");
    check(
        "get_status initial",
        expected,
        Value::float(temp_of_status(&sensor)),
    );
    py.exec("mc.config_cb()").expect("py build");
    mcu.invoke_config_cb().expect("rust build");
    py.exec("mc.serial_cb({'value': 25000, 'next_clock': 10000, 'fault': 0})")
        .expect("py serial");
    mcu.invoke_serial(params_dict(25000, 10000, 0));
    // `self.temp` is never updated by the callback (reference bug
    // reproduced, not fixed), so status stays at 0.0 on both sides.
    let expected = py.eval("s.get_status(0)['temperature']");
    check(
        "get_status after callback",
        expected,
        Value::float(temp_of_status(&sensor)),
    );
}

#[test]
fn fault_invokes_async_shutdown() {
    let Some(mut py) = oracle() else { return };
    let (sensor, _mcu, printer, _c, _h, _cb) = fresh_pair(
        &mut py,
        "temperature_sensor my_sensor",
        &[("serial_no", "test"), ("sensor_mcu", "mcu")],
    );
    py.exec("s.fault('sensor offline')").expect("py fault");
    sensor.fault("sensor offline");
    check(
        "fault shutdown",
        py.eval("\";\".join(p.async_shutdowns)"),
        Value::str(&printer.async_shutdowns.borrow().join(";")),
    );
}

#[test]
fn missing_serial_no_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let py_code =
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'temperature_sensor my_sensor', {'sensor_mcu': 'mcu'}); ds18b20.DS18B20(c)";
    let expected = py.exec(py_code);
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "temperature_sensor my_sensor".to_string(),
        opts: make_opts(&[("sensor_mcu", "mcu")]),
    });
    check_err("missing serial_no", expected, err(DS18B20::new(&config)));
}

#[test]
fn missing_sensor_mcu_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let py_code =
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'temperature_sensor my_sensor', {'serial_no': 'abc'}); ds18b20.DS18B20(c)";
    let expected = py.exec(py_code);
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "temperature_sensor my_sensor".to_string(),
        opts: make_opts(&[("serial_no", "abc")]),
    });
    check_err("missing sensor_mcu", expected, err(DS18B20::new(&config)));
}

#[test]
fn unknown_sensor_mcu_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let py_code =
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'temperature_sensor my_sensor', {'serial_no': 'abc', 'sensor_mcu': 'nope'}); ds18b20.DS18B20(c)";
    let expected = py.exec(py_code);
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "temperature_sensor my_sensor".to_string(),
        opts: make_opts(&[("serial_no", "abc"), ("sensor_mcu", "nope")]),
    });
    check_err("unknown sensor_mcu", expected, err(DS18B20::new(&config)));
}

#[test]
fn report_time_below_minimum_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let py_code =
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'temperature_sensor my_sensor', {'serial_no': 'abc', 'sensor_mcu': 'mcu', 'ds18_report_time': '0.5'}); ds18b20.DS18B20(c)";
    let expected = py.exec(py_code);
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "temperature_sensor my_sensor".to_string(),
        opts: make_opts(&[
            ("serial_no", "abc"),
            ("sensor_mcu", "mcu"),
            ("ds18_report_time", "0.5"),
        ]),
    });
    check_err(
        "report_time too small",
        expected,
        err(DS18B20::new(&config)),
    );
}

#[test]
fn load_config_registers_factory_and_returns_none() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("ds18b20 exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "no load_config_prefix"
    );
    assert!(factory.add_early_printer_objects.is_none(), "no add_early");
    assert!(
        factory.add_printer_objects.is_none(),
        "no add_printer_objects"
    );

    let py_code =
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'ds18b20', {}); ds18b20.load_config(c)";
    check("py load returns None", py.exec(py_code), Value::Null);
    check(
        "factory registered",
        py.eval("'DS18B20' in h.sensor_factories"),
        Value::Bool(true),
    );

    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu.clone() as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters.clone() as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "ds18b20".to_string(),
        opts: IndexMap::new(),
    });
    let ret = load_config(&config).expect("load_config runs");
    assert!(
        ret.is_none(),
        "load_config returns no object (side effect only)"
    );
    assert!(heaters.has("DS18B20"), "rust factory registered");
}

#[test]
fn factory_builds_working_sensor() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'ds18b20', {}); ds18b20.load_config(c); c2 = FakeConfig(p, 'temperature_sensor made', {'serial_no': 'factory-sensor', 'sensor_mcu': 'mcu'}); sf = h.sensor_factories['DS18B20'](c2); cb2 = FakeCallback(); sf.setup_callback(cb2); mc.config_cb()",
    )
    .expect("py factory sensor");

    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu.clone() as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters.clone() as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "ds18b20".to_string(),
        opts: IndexMap::new(),
    });
    let ret = module_factory().load_config.expect("factory fn")(&config).expect("load");
    assert!(ret.is_none(), "load returns None");
    let factory = heaters.factory("DS18B20").expect("factory registered");
    let config2: Config = Arc::new(FakeConfig {
        printer: (printer.clone() as Printer),
        name: "temperature_sensor made".to_string(),
        opts: make_opts(&[("serial_no", "factory-sensor"), ("sensor_mcu", "mcu")]),
    });
    let sensor = factory(&config2).expect("factory sensor");
    let collector = Collector::new();
    let cc = collector.clone();
    sensor.setup_callback(Box::new(move |t: f64, temp: f64| cc.record(t, temp))
        as TemperatureCallback);
    mcu.invoke_config_cb().expect("rust build");
    check(
        "factory cmds",
        py.eval("\";\".join([\"%s|%d\" % (c, 1 if i else 0) for (c, i) in mc.cmds])"),
        Value::str(&mcu.cmds_joined()),
    );
}

#[test]
fn sensor_factory_fn_is_the_ds18b20_class() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "p = FakePrinter(); mc = FakeMcu(p); p.objects['mcu'] = mc; h = FakeHeaters(); p.objects['heaters'] = h; c = FakeConfig(p, 'temperature_sensor my_sensor', {'serial_no': 'zz', 'sensor_mcu': 'mcu'}); s = ds18b20.DS18B20(c)",
    )
    .expect("py ds18b20");
    let printer = Arc::new(FakePrinter::new());
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer));
    printer
        .add_object("mcu", mcu.clone() as ObjectRef)
        .expect("add mcu");
    let heaters = Arc::new(FakeHeaters::new());
    printer
        .add_object("heaters", heaters as ObjectRef)
        .expect("add heaters");
    let printer_trait = printer.clone() as Printer;
    let config: Config = Arc::new(FakeConfig {
        printer: printer_trait,
        name: "temperature_sensor my_sensor".to_string(),
        opts: make_opts(&[("serial_no", "zz"), ("sensor_mcu", "mcu")]),
    });
    let sensor = sensor_factory(&config).expect("factory direct");
    check(
        "factory report_time",
        py.eval("s.report_time"),
        Value::float(sensor.get_report_time_delta()),
    );
}
