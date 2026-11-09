//! Parity tests for `extras::bme280` (port of `klippy/extras/bme280.py`).
//!
//! Expected values come live from the frozen reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The Python side builds a real
//! `bme280.BME280` over a real `configfile.ConfigWrapper` (so the option
//! parsing, defaults and error texts come from `configfile.py`) with minimal
//! fakes of the printer, reactor, MCU and `bus.MCU_I2C_from_config`
//! (monkeypatched, like the reference imports it). The fake I2C device is a
//! 256 byte register file; its `i2c_write` encodes the message with the real
//! `msgproto.PT_buffer`, so an out-of-range byte raises the reference's own
//! `ValueError`. The Rust side uses the real bus constructor, resolves MCU and
//! pin services through their interfaces, and scripts its command transport
//! with the same register file.
//!
//! Compared against the oracle, per chip family (`BMP280`, `BME280`,
//! `BME680`, `BMP180`, `BMP388`, an unknown id) and over seeded random
//! calibration and measurement bytes: config options, defaults, errors and
//! log lines, registered objects and event handlers, the `klippy:connect`
//! chip probe (I2C reads and writes, errors from every failing read or
//! write), the parsed calibration `dig`, every `_sample_*` path (return
//! value, callbacks, shutdown messages, log records, zeroed state after read
//! failures, every response shape `bytearray(...)` accepts or rejects),
//! the compensation helpers by bits (including `ZeroDivisionError` and
//! `KeyError` cases), the gas heater helpers, `get_status`, and `load_config`.
//!
//! The Rust reactor clock cannot be scripted. The Python reactor is a virtual
//! clock seeded with the Rust clock at construction, and a sample's wake time
//! (`monotonic() + REPORT_TIME`) is checked as the Python value
//! `reactor.now + 0.8` on the reference side and as a value inside the bracket
//! of monotonic reads around the call on the port side. `NEVER` returns are
//! bit-compared. There are no recordings for this unit.

use extras::bme280::{
    get_signed_byte, get_signed_short, get_signed_short_msb, get_twos_complement,
    get_unsigned_short, get_unsigned_short_msb, module_factory, Bme280, BME280_CHIP_ADDR,
    BME280_I2C_SPEED, REPORT_TIME,
};
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log::{set_sink, Record as LogRecord};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueryWrapperApi,
    CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback, LookupQueryCommandOpts, Mcu,
    McuApi, Pin, PinParams, PinResolver, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{interface, EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT};
use klippy_api::GCell;
use pycompat::PyValue;
use reactor::{Reactor, NEVER};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::cell::RefCell;
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging
import configfile
import msgproto
import extras.bme280 as bme280
from configparser import RawConfigParser

class error(Exception):
    pass

class LogCapture(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.records = []
    def emit(self, record):
        self.records.append((record.levelname, record.getMessage(),
                             record.exc_info is not None))

capture = LogCapture()
logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger().addHandler(capture)

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self, now):
        self.now = now
        self.timers = []
        self.updates = []
    def monotonic(self):
        return self.now
    def pause(self, waketime):
        self.now = max(self.now, waketime)
        return self.now
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(waketime)
        return ('timer', len(self.timers))
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return 1234.5

class FakeI2C:
    def __init__(self, mcu):
        self.mcu = mcu
        self.i2c_address = 0x76
        self.mem = bytearray(256)
        self.reads = []
        self.writes = []
        self.fail_reads = {}
        self.reg_queue = {}
        self.fail_write_at = None
    def get_mcu(self):
        return self.mcu
    def i2c_read(self, write, read_len, minclock=0, reqclock=0, retry=True):
        self.reads.append((list(write), read_len))
        n = len(self.reads)
        reg = write[0]
        if n in self.fail_reads:
            item = self.fail_reads.pop(n)
        elif self.reg_queue.get(reg):
            item = self.reg_queue[reg].pop(0)
        else:
            item = {'response': bytes(self.mem[reg:reg + read_len])}
        if isinstance(item, Exception):
            raise item
        return item
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        data = list(data)
        msgproto.PT_buffer().encode([], data)
        self.writes.append(data)
        if self.fail_write_at == len(self.writes):
            raise error("i2c write failed")

class FakePrinter:
    def __init__(self, reactor, start_args, duplicate):
        self.reactor = reactor
        self.start_args = start_args
        self.duplicate = duplicate
        self.objects = []
        self.handlers = []
        self.shutdowns = []
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return self.start_args
    def add_object(self, name, obj):
        if self.duplicate:
            raise configfile.error("Printer object '%s' already created" % (name,))
        self.objects.append(name)
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)

from_config_calls = []
cbs = []

def build(section, options, start_args, duplicate, now):
    global printer, i2c, mcu, reactor, obj, cbs
    del capture.records[:]
    del from_config_calls[:]
    cbs = []
    reactor = FakeReactor(now)
    mcu = FakeMcu()
    i2c = FakeI2C(mcu)
    printer = FakePrinter(reactor, start_args, duplicate)
    fc = RawConfigParser()
    fc.add_section(section)
    for k, v in options.items():
        fc.set(section, k, v)
    wrapper = configfile.ConfigWrapper(printer, fc, {}, section)
    def fake_from_config(config, default_addr=None, default_speed=None, **kwargs):
        from_config_calls.append((default_addr, default_speed, sorted(kwargs.items())))
        config.get('i2c_mcu', 'mcu')
        i2c.i2c_speed = config.getint('i2c_speed', default_speed, minval=100000)
        i2c.i2c_address = config.getint('i2c_address', default_addr, minval=0, maxval=127)
        return i2c
    orig = bme280.bus.MCU_I2C_from_config
    bme280.bus.MCU_I2C_from_config = fake_from_config
    try:
        obj = bme280.BME280(wrapper)
    finally:
        bme280.bus.MCU_I2C_from_config = orig
    return None

def connect():
    for event, cb in list(printer.handlers):
        if event == 'klippy:connect':
            cb()

def set_callback():
    obj.setup_callback(lambda print_time, temp: cbs.append((print_time, temp)))

def set_mem(reg, data):
    i2c.mem[reg:reg + len(data)] = bytes(data)

def disarm():
    i2c.fail_reads.clear()
    i2c.reg_queue.clear()
    i2c.fail_write_at = None

def trace():
    result = ([(r[0], r[1]) for r in i2c.reads], list(i2c.writes),
              list(printer.shutdowns), list(cbs), list(capture.records),
              obj.get_status(0.),
              (obj.temp, obj.pressure, obj.humidity, obj.gas, obj.t_fine))
    del capture.records[:]
    return result

def info():
    dig = None
    if obj.dig is not None:
        dig = {k: float(v) for k, v in obj.dig.items()}
    return (obj.name, obj.chip_type, obj.iir_filter, obj.os_temp, obj.os_hum,
            obj.os_pres, obj.gas_heat_temp, obj.gas_heat_duration,
            obj.max_sample_time, dig, obj.sample_timer is not None)

def gas_time0():
    lgt = getattr(obj, 'last_gas_time', None)
    return None if lgt is None else float(lgt)

class FakeHeaters:
    def __init__(self):
        self.factories = []
    def add_sensor_factory(self, sensor_type, factory):
        self.factories.append((sensor_type, factory is bme280.BME280))

class FakeLoadPrinter:
    def __init__(self, heaters):
        self.heaters = heaters
        self.loaded = []
    def load_object(self, config, section, default=None):
        self.loaded.append(section)
        if section == 'heaters' and self.heaters is not None:
            return self.heaters
        raise configfile.error("Unknown config object '%s'" % (section,))

def load(section, with_heaters):
    global load_printer, load_heaters
    load_heaters = FakeHeaters() if with_heaters else None
    load_printer = FakeLoadPrinter(load_heaters)
    fc = RawConfigParser()
    fc.add_section(section)
    wrapper = configfile.ConfigWrapper(load_printer, fc, {}, section)
    return bme280.load_config(wrapper)
"#;

// ---------------------------------------------------------------------------
// Oracle helpers.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("SKIP: $KLIPPY_PYTHON is unset; pyoracle unavailable");
        return None;
    };
    py.exec(SETUP).expect("py SETUP");
    Some(py)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.bme280".to_string(),
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

/// Oracle value against a Rust result: values through `assert_equal`,
/// errors through exception class and message.
fn check_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(expected), Ok(actual)) => check(label, Ok(expected), actual),
        (Ok(v), Err(e)) => panic!(
            "{label}: oracle returned {}, port raised {}: {}",
            v.render(),
            e.class_name(),
            e.message()
        ),
        (Err(py_exc), Err(e)) => {
            assert_eq!(e.class_name(), py_exc.class_name, "{label}: exception class");
            assert_eq!(e.message(), py_exc.message, "{label}: exception message");
        }
        (Err(py_exc), Ok(actual)) => panic!(
            "{label}: oracle raised {}: {}, port returned {}",
            py_exc.class_name,
            py_exc.message,
            actual.render()
        ),
    }
}

fn value_float(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("recorded float"),
        _ => panic!("expected float value, got {}", v.render()),
    }
}

/// A Python number returned by the port: an int stays an int, a float is
/// compared by bits.
fn number_value(v: PyValue) -> Value {
    match v {
        PyValue::Int(n) => Value::int(i64::try_from(n.inner()).expect("int fits in i64")),
        PyValue::Float(f) => Value::float(f),
        other => panic!("expected an int or a float, got {other:?}"),
    }
}

fn ints(bytes: &[u8]) -> Value {
    Value::List(bytes.iter().map(|b| Value::int(*b as i64)).collect())
}

fn py_bytes_list(bytes: &[u8]) -> String {
    format!("[{}]", bytes.iter().map(|b| b.to_string()).collect::<Vec<_>>().join(", "))
}

fn py_dict(options: &[(&str, &str)]) -> String {
    let items: Vec<String> = options.iter().map(|(k, v)| format!("{k:?}: {v:?}")).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_start_args(args: &[(&str, Option<&str>)]) -> String {
    let items: Vec<String> = args
        .iter()
        .map(|(k, v)| match v {
            Some(v) => format!("{k:?}: {v:?}"),
            None => format!("{k:?}: None"),
        })
        .collect();
    format!("{{{}}}", items.join(", "))
}

// ---------------------------------------------------------------------------
// Log capture (per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOGS: RefCell<Vec<(String, String, bool)>> = const { RefCell::new(Vec::new()) };
}

fn install_sink() {
    let _ = set_sink(Box::new(|rec: &LogRecord<'_>| {
        LOGS.with(|logs| {
            logs.borrow_mut().push((
                rec.level.name().to_string(),
                rec.message.to_string(),
                rec.exc_text.is_some(),
            ))
        });
    }));
}

fn take_logs() -> Vec<(String, String, bool)> {
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

struct FakeConfig {
    section: String,
    printer: Printer,
    values: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, values: &[(&str, &str)]) -> Arc<FakeConfig> {
        let mut map = IndexMap::new();
        for (k, v) in values {
            map.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeConfig { section: section.to_string(), printer, values: map })
    }

    fn required(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{}' in section '{}' must be specified",
            option, self.section
        ))
    }
}

fn unused<T>(what: &str) -> KResult<T> {
    Err(KlippyError::Config(format!("FakeConfig.{what} is not used by bme280")))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.values.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            },
        }
    }
    /// `ConfigWrapper.getint` / `_get_wrapper` (`klippy/configfile.py:29-67`).
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.values.get(option) else {
            return match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            };
        };
        let v = raw.trim().parse::<i64>().map_err(|_| {
            KlippyError::Config(format!(
                "Unable to parse option '{}' in section '{}'",
                option, self.section
            ))
        })?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must have minimum of {}",
                    option, self.section, minval
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must have maximum of {}",
                    option, self.section, maxval
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        unused("getfloat")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        unused("getboolean")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        unused("getchoice")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unused("getlists")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unused("getlist")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unused("getintlist")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unused("getfloatlist")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("FakeConfig.getsection not exercised")
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

struct FakePrinter {
    reactor: Reactor,
    /// `printer.add_object` raises for an already created name.
    duplicate: bool,
    start_args: IndexMap<PyValue, PyValue>,
    objects: GCell<Vec<(String, ObjectRef)>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    shutdowns: GCell<Vec<String>>,
    heaters: GCell<Option<Arc<FakeHeaters>>>,
    loaded: GCell<Vec<String>>,
    services: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new(duplicate: bool, start_args: &[(&str, Option<&str>)]) -> Arc<FakePrinter> {
        let mut args = IndexMap::new();
        for (k, v) in start_args {
            let v = match v {
                Some(v) => PyValue::Str(v.to_string()),
                None => PyValue::None,
            };
            args.insert(PyValue::Str(k.to_string()), v);
        }
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            duplicate,
            start_args: args,
            objects: GCell::new(Vec::new()),
            handlers: GCell::new(Vec::new()),
            shutdowns: GCell::new(Vec::new()),
            heaters: GCell::new(None),
            loaded: GCell::new(Vec::new()),
            services: GCell::new(IndexMap::new()),
        })
    }

    fn object_names(&self) -> Vec<String> {
        self.objects.borrow().iter().map(|(n, _)| n.clone()).collect()
    }

    /// Event names of the handlers the module under test registered: the
    /// first one is the one `bus.MCU_I2C` registers, which the Python fake
    /// bus does not have.
    fn own_handler_events(&self) -> Vec<String> {
        let handlers = self.handlers.borrow();
        assert!(!handlers.is_empty(), "the bus registers its connect handler");
        handlers.iter().skip(1).map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(self.start_args.clone())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        if self.duplicate {
            return Err(KlippyError::Config(format!("Printer object '{name}' already created")));
        }
        self.objects.borrow_mut().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(Some(self.services.borrow().get(name).expect("registered service").clone()))
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.loaded.borrow_mut().push(section.to_string());
        let heaters = self.heaters.cloned();
        match heaters {
            Some(h) if section == "heaters" => Ok(Some(h as ObjectRef)),
            _ => Err(KlippyError::Config(format!("Unknown config object '{section}'"))),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut returns = Vec::new();
        let mut handlers = self.handlers.borrow_mut();
        for (ev, cb) in handlers.iter_mut() {
            if ev == event {
                returns.push(cb(params)?);
            }
        }
        Ok(returns)
    }
    fn request_exit(&self, _result: &str) {}
}

/// `printer.lookup_object('heaters')` stand-in: keeps `add_sensor_factory`.
struct FakeHeaters {
    factories: GCell<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.borrow_mut().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("FakeHeaters.setup_heater not exercised")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("FakeHeaters.get_all_heaters not exercised")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("FakeHeaters.lookup_heater not exercised")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("FakeHeaters.setup_sensor not exercised")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("FakeHeaters.register_sensor not exercised")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("FakeHeaters.register_monitor not exercised")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("FakeHeaters.turn_off_all_heaters not exercised")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("FakeHeaters.set_temperature not exercised")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

/// One scripted `i2c_read` outcome, rendered for both sides.
#[derive(Clone, Debug)]
enum Resp {
    /// `{'response': bytes(...)}`
    Bytes(Vec<u8>),
    /// `{'response': [..]}`
    List(Vec<i64>),
    /// `{'response': 'text'}`
    Str(&'static str),
    /// `{'response': n}`
    Int(i64),
    /// `{'response': x}` with a float
    Float(f64),
    /// `None`: `i2c_transfer` returns it for file output and after a shutdown.
    NoParams,
    /// `{}`: a reply without the `response` key.
    NoKey,
    /// The transport raises `error(msg)` (`mcu.error`).
    Err(&'static str),
}

impl Resp {
    fn py(&self) -> String {
        match self {
            Resp::Bytes(b) => format!("{{'response': bytes({})}}", py_bytes_list(b)),
            Resp::List(v) => format!(
                "{{'response': [{}]}}",
                v.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ")
            ),
            Resp::Str(s) => format!("{{'response': {s:?}}}"),
            Resp::Int(n) => format!("{{'response': {n}}}"),
            Resp::Float(f) => format!("{{'response': {f:?}}}"),
            Resp::NoParams => "None".to_string(),
            Resp::NoKey => "{}".to_string(),
            Resp::Err(msg) => format!("error({msg:?})"),
        }
    }

    fn rust(&self) -> KResult<PyValue> {
        let reply = |v: PyValue| {
            let mut map = IndexMap::new();
            map.insert(PyValue::Str("response".to_string()), v);
            Ok(PyValue::Dict(map))
        };
        match self {
            Resp::Bytes(b) => reply(PyValue::Bytes(b.clone())),
            Resp::List(v) => reply(PyValue::List(
                v.iter().map(|x| PyValue::Int(pycompat::PyInt::from_i64(*x))).collect(),
            )),
            Resp::Str(s) => reply(PyValue::Str(s.to_string())),
            Resp::Int(n) => reply(PyValue::Int(pycompat::PyInt::from_i64(*n))),
            Resp::Float(f) => reply(PyValue::Float(*f)),
            Resp::NoParams => Ok(PyValue::None),
            Resp::NoKey => Ok(PyValue::Dict(IndexMap::new())),
            Resp::Err(msg) => Err(KlippyError::Mcu(msg.to_string())),
        }
    }
}

/// The fake I2C device: the same register file and failure scripts as the
/// Python `FakeI2C`.
struct I2cState {
    mem: Vec<u8>,
    reads: Vec<(Vec<u8>, i64)>,
    writes: Vec<Vec<u8>>,
    fail_reads: BTreeMap<usize, Resp>,
    reg_queue: BTreeMap<u8, VecDeque<Resp>>,
    fail_write_at: Option<usize>,
}

struct FakeI2c {
    state: GCell<I2cState>,
}

impl CommandWrapperApi for FakeI2c {
    fn send(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("only acknowledged writes are used")
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        assert_eq!(opts, SendOpts::default());
        let PyValue::Bytes(bytes) = &data[1] else { panic!("write bytes") };
        let mut st = self.state.borrow_mut();
        st.writes.push(bytes.clone());
        if st.fail_write_at == Some(st.writes.len()) {
            return Err(KlippyError::Mcu("i2c write failed".to_string()));
        }
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        panic!("command tag is not used")
    }
}

impl CommandQueryWrapperApi for FakeI2c {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        assert_eq!(opts, QuerySendOpts::default());
        let PyValue::Bytes(bytes) = &data[1] else { panic!("read register bytes") };
        let PyValue::Int(len) = &data[2] else { panic!("read length") };
        let len = i64::try_from(len.inner()).expect("length fits");
        let mut st = self.state.borrow_mut();
        st.reads.push((bytes.clone(), len));
        let n = st.reads.len();
        let reg = bytes[0];
        if let Some(resp) = st.fail_reads.remove(&n) {
            return resp.rust();
        }
        if let Some(resp) = st.reg_queue.get_mut(&reg).and_then(|q| q.pop_front()) {
            return resp.rust();
        }
        let start = (reg as usize).min(st.mem.len());
        let end = (reg as usize + len as usize).min(st.mem.len());
        Resp::Bytes(st.mem[start..end].to_vec()).rust()
    }
    fn send_with_preface(
        &self,
        _cmd: CommandWrapper,
        _data: Vec<PyValue>,
        _query: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("read preface is not used")
    }
}

/// MCU transport fake, including the bus configuration callbacks.
struct FakeMcu {
    printer: Printer,
    transport: Arc<FakeI2c>,
    config_callbacks: GCell<Vec<ConfigCallback>>,
    config_cmds: GCell<Vec<String>>,
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
        panic!("FakeMcu.setup_pin not exercised")
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.config_callbacks.borrow_mut().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds.borrow_mut().push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        panic!("FakeMcu.request_move_queue_slot not exercised")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("FakeMcu.get_query_slot not exercised")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("FakeMcu.seconds_to_clock not exercised")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("FakeMcu.min_schedule_time not exercised")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("FakeMcu.max_nominal_duration not exercised")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Ok(self.transport.clone())
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(self.transport.clone())
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        Some(self.transport.clone())
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("FakeMcu.register_serial_response not exercised")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("FakeMcu.check_valid_response not exercised")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("FakeMcu.get_constants not exercised")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("FakeMcu.get_constant_float not exercised")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("FakeMcu.print_time_to_clock not exercised")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("FakeMcu.clock_to_print_time not exercised")
    }
    /// The Python `FakeMcu.estimated_print_time` returns the same constant.
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        1234.5
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("FakeMcu.clock32_to_clock64 not exercised")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("FakeMcu.calibrate_clock not exercised")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("FakeMcu.get_status not exercised")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("FakeMcu.stats not exercised")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("FakeMcu.create_trigger_dispatch not exercised")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins;

impl PinsApi for FakePins {
    fn parse_pin(&self, _desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("hardware bus does not parse pins")
    }
    fn lookup_pin(&self, _desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("hardware bus does not look up pins")
    }
    fn setup_pin(&self, _kind: &str, _desc: &str) -> KResult<Pin> {
        panic!("hardware bus does not set up pins")
    }
    fn reset_pin_sharing(&self, _params: PinParams) -> klippy_api::KResult<()> { panic!("hardware bus does not reset pin sharing") }
    fn get_pin_resolver(&self, _name: &str) -> KResult<PinResolver> {
        panic!("MCU has no bus enumeration")
    }
    fn register_chip(&self, _name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("hardware bus does not register chips")
    }
    fn allow_multi_use_pin(&self, _desc: &str) -> KResult<()> {
        panic!("hardware bus does not share pins")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakePrinterConfig;

impl PrinterConfigApi for FakePrinterConfig {
    fn get_printer(&self) -> Printer {
        panic!("config printer not used")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("config files not read")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("main config not read")
    }
    fn log_config(&self, _config: &Config) {
        panic!("config not logged")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("config validation not used")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("runtime warning not used")
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("option deprecation not used")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {
        panic!("gcode deprecation not used")
    }
    fn deprecate_mcu_code(&self, _mcu: &Mcu, feature: &str, _msg: Option<String>) {
        assert_eq!(feature, "i2c_transfer");
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("config status not used")
    }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> {
        panic!("config not saved")
    }
    fn remove_section(&self, _section: &str) {
        panic!("config sections not removed")
    }
}

klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);

// ---------------------------------------------------------------------------
// Rust-side construction and traces.
// ---------------------------------------------------------------------------

const SECTION: &str = "temperature_sensor foo";

/// The services and the config a `Bme280` is built from.
struct Env {
    printer: Arc<FakePrinter>,
    i2c: Arc<FakeI2c>,
    mcu: Arc<FakeMcu>,
    config: Config,
}

impl Env {
    fn new(
        section: &str,
        options: &[(&str, &str)],
        start_args: &[(&str, Option<&str>)],
        duplicate: bool,
    ) -> Env {
        install_sink();
        take_logs();
        let printer = FakePrinter::new(duplicate, start_args);
        let i2c = Arc::new(FakeI2c {
            state: GCell::new(I2cState {
                mem: vec![0; 256],
                reads: Vec::new(),
                writes: Vec::new(),
                fail_reads: BTreeMap::new(),
                reg_queue: BTreeMap::new(),
                fail_write_at: None,
            }),
        });
        let mcu = Arc::new(FakeMcu {
            printer: printer.clone(),
            transport: i2c.clone(),
            config_callbacks: GCell::new(Vec::new()),
            config_cmds: GCell::new(Vec::new()),
        });
        printer.services.borrow_mut().insert("mcu".into(), mcu.clone());
        printer.services.borrow_mut().insert("pins".into(), Arc::new(FakePins));
        printer.services.borrow_mut().insert("configfile".into(), Arc::new(FakePrinterConfig));
        let printer_api: Printer = printer.clone();
        let config: Config = FakeConfig::new(printer_api, section, options);
        Env { printer, i2c, mcu, config }
    }
}

struct Built {
    env: Env,
    cbs: Arc<GCell<Vec<(f64, f64)>>>,
    obj: Arc<Bme280>,
}

fn build_rust(env: Env) -> KResult<Built> {
    let obj = Bme280::new(&env.config)?;
    for mut callback in std::mem::take(&mut *env.mcu.config_callbacks.borrow_mut()) {
        callback()?;
    }
    Ok(Built { env, cbs: Arc::new(GCell::new(Vec::new())), obj })
}

/// Build both sides; the oracle result is returned for the caller to compare.
fn build_pair(
    py: &mut pyoracle::Oracle,
    section: &str,
    options: &[(&str, &str)],
    start_args: &[(&str, Option<&str>)],
    duplicate: bool,
) -> (pyoracle::OracleResult, KResult<Built>) {
    let dup = if duplicate { "True" } else { "False" };
    let now = reactor::monotonic();
    let expected = py.exec(&format!(
        "build({section:?}, {}, {}, {dup}, {now:?})",
        py_dict(options),
        py_start_args(start_args)
    ));
    let actual = build_rust(Env::new(section, options, start_args, duplicate));
    (expected, actual)
}

/// Build both sides and require success on both.
fn build_ok(py: &mut pyoracle::Oracle, options: &[(&str, &str)]) -> Built {
    let (expected, actual) = build_pair(py, SECTION, options, &[], false);
    expected.expect("py build");
    let b = match actual {
        Ok(b) => b,
        Err(e) => panic!("port build raised {}: {}", e.class_name(), e.message()),
    };
    check("construction trace", py.eval("trace()"), b.trace());
    b
}

impl Built {
    fn set_callback(&self, py: &mut pyoracle::Oracle) {
        py.exec("set_callback()").expect("py callback");
        let cbs = self.cbs.clone();
        self.obj.setup_callback(Box::new(move |print_time, temp| {
            cbs.borrow_mut().push((print_time, temp))
        }));
    }

    fn set_limits(&self, py: &mut pyoracle::Oracle, min_temp: f64, max_temp: f64) {
        py.exec(&format!("obj.setup_minmax({min_temp:?}, {max_temp:?})")).expect("py minmax");
        self.obj.setup_minmax(min_temp, max_temp).expect("rust minmax");
    }

    fn set_mem(&self, py: &mut pyoracle::Oracle, reg: usize, data: &[u8]) {
        py.exec(&format!("set_mem({reg}, {})", py_bytes_list(data))).expect("py mem");
        self.env.i2c.state.with_mut(|s| s.mem[reg..reg + data.len()].copy_from_slice(data));
    }

    fn apply_mem(&self, py: &mut pyoracle::Oracle, mem: &[(usize, Vec<u8>)]) {
        for (reg, data) in mem {
            self.set_mem(py, *reg, data);
        }
    }

    fn read_count(&self) -> usize {
        self.env.i2c.state.borrow().reads.len()
    }

    /// Make the `k`-th read from now on answer with `resp`.
    fn arm_read(&self, py: &mut pyoracle::Oracle, k: usize, resp: &Resp) {
        py.exec(&format!("i2c.fail_reads[len(i2c.reads) + {k}] = {}", resp.py())).expect("py arm");
        let at = self.read_count() + k;
        self.env.i2c.state.borrow_mut().fail_reads.insert(at, resp.clone());
    }

    /// Queue `resps` as the next answers to reads of register `reg`.
    fn queue_reg(&self, py: &mut pyoracle::Oracle, reg: u8, resps: &[Resp]) {
        let items: Vec<String> = resps.iter().map(|r| r.py()).collect();
        py.exec(&format!("i2c.reg_queue.setdefault({reg}, []).extend([{}])", items.join(", ")))
            .expect("py queue");
        self.env
            .i2c
            .state
            .borrow_mut()
            .reg_queue
            .entry(reg)
            .or_default()
            .extend(resps.iter().cloned());
    }

    /// Make the `k`-th write from now on fail.
    fn arm_write(&self, py: &mut pyoracle::Oracle, k: usize) {
        py.exec(&format!("i2c.fail_write_at = len(i2c.writes) + {k}")).expect("py arm write");
        let at = self.env.i2c.state.borrow().writes.len() + k;
        self.env.i2c.state.borrow_mut().fail_write_at = Some(at);
    }

    fn disarm(&self, py: &mut pyoracle::Oracle) {
        py.exec("disarm()").expect("py disarm");
        self.env.i2c.state.with_mut(|s| {
            s.fail_reads.clear();
            s.reg_queue.clear();
            s.fail_write_at = None;
        });
    }

    /// Mirror of the Python `trace()`.
    fn trace(&self) -> Value {
        let (reads, writes) = {
            let st = self.env.i2c.state.borrow();
            (
                Value::List(
                    st.reads
                        .iter()
                        .map(|(w, n)| Value::Tuple(vec![ints(w), Value::int(*n)]))
                        .collect(),
                ),
                Value::List(st.writes.iter().map(|w| ints(w)).collect()),
            )
        };
        let shutdowns =
            Value::List(self.env.printer.shutdowns.borrow().iter().map(|m| Value::str(m)).collect());
        let cbs = Value::List(
            self.cbs
                .borrow()
                .iter()
                .map(|(pt, t)| Value::Tuple(vec![Value::float(*pt), Value::float(*t)]))
                .collect(),
        );
        let logs = Value::List(
            take_logs()
                .into_iter()
                .map(|(level, msg, exc)| {
                    Value::Tuple(vec![Value::str(&level), Value::str(&msg), Value::Bool(exc)])
                })
                .collect(),
        );
        let state = Value::Tuple(vec![
            Value::float(self.obj.temp()),
            Value::float(self.obj.pressure()),
            Value::float(self.obj.humidity()),
            Value::float(self.obj.gas()),
            Value::float(self.obj.t_fine()),
        ]);
        Value::Tuple(vec![reads, writes, shutdowns, cbs, logs, self.status(), state])
    }

    fn status(&self) -> Value {
        use klippy_api::object::PrinterObject;
        let st = self.obj.get_status(0.).unwrap().expect("bme280 has get_status");
        let PyValue::Dict(map) = st else { panic!("status is a dict") };
        Value::Dict(
            map.iter()
                .map(|(k, v)| {
                    let PyValue::Str(k) = k else { panic!("status key is a str") };
                    let PyValue::Float(v) = v else { panic!("status value is a float") };
                    (Value::str(k), Value::float(*v))
                })
                .collect(),
        )
    }

    /// Mirror of the Python `info()`.
    fn info(&self) -> Value {
        let o = &self.obj;
        let dig = match o.dig() {
            None => Value::Null,
            Some(d) => Value::Dict(d.iter().map(|(k, v)| (Value::str(k), Value::float(*v))).collect()),
        };
        Value::Tuple(vec![
            Value::str(o.name()),
            Value::str(o.chip_type()),
            Value::int(o.iir_filter()),
            Value::int(o.os_temp()),
            Value::int(o.os_hum()),
            Value::int(o.os_pres()),
            Value::int(o.gas_heat_temp()),
            Value::int(o.gas_heat_duration()),
            o.max_sample_time().map_or(Value::Null, Value::float),
            dig,
            Value::Bool(o.has_sample_timer()),
        ])
    }

    /// Fire `klippy:connect` on both sides and compare the outcome, the I2C
    /// traffic, the logs and the chip state.
    fn connect(&self, py: &mut pyoracle::Oracle, label: &str) -> bool {
        let expected = py.exec("connect()");
        let actual = self.env.printer.send_event(EV_KLIPPY_CONNECT, &[]).map(|_| Value::Null);
        let ok = actual.is_ok();
        check_result(label, expected.map(|_| Value::Null), actual);
        check(&format!("{label} trace"), py.eval("trace()"), self.trace());
        check(&format!("{label} info"), py.eval("info()"), self.info());
        ok
    }

    /// Run `py_expr` on the reference and compare with the port's result,
    /// then compare the whole trace.
    fn compare(&self, py: &mut pyoracle::Oracle, label: &str, py_expr: &str, actual: KResult<Value>) {
        check_result(label, py.eval(py_expr), actual);
        check(&format!("{label} trace"), py.eval("trace()"), self.trace());
    }
}

// ---------------------------------------------------------------------------
// Sample steps.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum Sample {
    Bme280,
    Bmp388,
    Bme680,
    Bmp180,
}

impl Sample {
    fn py(self) -> &'static str {
        match self {
            Sample::Bme280 => "_sample_bme280",
            Sample::Bmp388 => "_sample_bmp388",
            Sample::Bme680 => "_sample_bme680",
            Sample::Bmp180 => "_sample_bmp180",
        }
    }

    fn call(self, o: &Bme280) -> KResult<f64> {
        match self {
            Sample::Bme280 => o.sample_bme280(1.0),
            Sample::Bmp388 => o.sample_bmp388(1.0),
            Sample::Bme680 => o.sample_bme680(1.0),
            Sample::Bmp180 => o.sample_bmp180(1.0),
        }
    }
}

/// One `_sample_*` call on both sides. `NEVER` is bit-compared; a finite wake
/// time is `monotonic() + REPORT_TIME` on each clock.
fn sample_step(py: &mut pyoracle::Oracle, b: &Built, label: &str, which: Sample) {
    let t0 = reactor::monotonic();
    let actual = which.call(&b.obj);
    let t1 = reactor::monotonic();
    let expected = py.eval(&format!("obj.{}(1.0)", which.py()));
    let py_now = value_float(&py.eval("reactor.now").expect("py clock"));
    match (expected, actual) {
        (Ok(p), Ok(r)) => {
            let p = value_float(&p);
            if p.to_bits() == NEVER.to_bits() {
                assert_eq!(r.to_bits(), NEVER.to_bits(), "{label}: NEVER return");
            } else {
                assert_eq!(
                    p.to_bits(),
                    (py_now + REPORT_TIME).to_bits(),
                    "{label}: reference wake time"
                );
                // `r` is `measured_time + REPORT_TIME` recomputed through f64
                // add/subtract, so allow a tiny rounding epsilon.
                let measured = r - REPORT_TIME;
                assert!(
                    measured >= t0 - 1e-6 && measured <= t1 + 1e-6,
                    "{label}: measured_time {measured} outside bracket [{t0}, {t1}]"
                );
            }
        }
        (Err(py_exc), Err(e)) => {
            assert_eq!(e.class_name(), py_exc.class_name, "{label}: exception class");
            assert_eq!(e.message(), py_exc.message, "{label}: exception message");
        }
        (Ok(p), Err(e)) => panic!(
            "{label}: oracle returned {}, port raised {}: {}",
            p.render(),
            e.class_name(),
            e.message()
        ),
        (Err(py_exc), Ok(r)) => panic!(
            "{label}: oracle raised {}: {}, port returned {r}",
            py_exc.class_name, py_exc.message
        ),
    }
    check(&format!("{label} trace"), py.eval("trace()"), b.trace());
}

// ---------------------------------------------------------------------------
// Deterministic fixtures.
// ---------------------------------------------------------------------------

struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Lcg {
        Lcg(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03)
    }

    fn byte(&mut self) -> u8 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 33) as u8
    }

    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.byte()).collect()
    }

    fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        items[self.byte() as usize % items.len()]
    }
}

#[derive(Clone, Copy, Debug)]
enum Chip {
    Bme280,
    Bmp280,
    Bme680,
    Bmp180,
    Bmp388,
}

/// Datasheet example calibration of the BMP280/BME280 (`0x88..0xA1`).
fn bmp280_cal() -> Vec<u8> {
    vec![
        0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, 0x7D, 0x8E, 0xC3, 0xD6, 0xD0, 0x0B, 0x27, 0x0B, 0x8C,
        0x00, 0xF9, 0xFF, 0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17, 0x00, 0x4B,
    ]
}

/// Datasheet example humidity calibration of the BME280 (`0xE1..0xF0`).
fn bme280_cal2() -> Vec<u8> {
    vec![0x6A, 0x01, 0x00, 0x13, 0x2B, 0x03, 0x1E, 0, 0, 0, 0, 0, 0, 0, 0, 0]
}

/// Datasheet example calibration of the BMP180 (`0xAA..0xBF`, MSB first).
fn bmp180_cal() -> Vec<u8> {
    vec![
        0x01, 0x98, 0xFF, 0xB8, 0xC7, 0xD1, 0x7F, 0xE5, 0x7F, 0xF5, 0x5A, 0x71, 0x18, 0x2E, 0x00,
        0x04, 0x80, 0x00, 0xDD, 0xF9, 0x0B, 0x34,
    ]
}

/// The register file a chip family needs for `_init_bmxx80`; seed 0 uses
/// datasheet calibration where one exists, the others random bytes.
fn chip_mem(chip: Chip, seed: u64) -> Vec<(usize, Vec<u8>)> {
    let mut r = Lcg::new(seed + 1);
    match chip {
        Chip::Bme280 | Chip::Bmp280 => {
            let id = if matches!(chip, Chip::Bme280) { 0x60 } else { 0x58 };
            let (cal1, cal2) =
                if seed == 0 { (bmp280_cal(), bme280_cal2()) } else { (r.bytes(26), r.bytes(16)) };
            let data = if seed == 0 {
                vec![0x65, 0xA2, 0xC0, 0x7E, 0xED, 0x00, 0x6D, 0x60]
            } else {
                r.bytes(8)
            };
            vec![(0xD0, vec![id]), (0x88, cal1), (0xE1, cal2), (0xF7, data)]
        }
        Chip::Bme680 => {
            let mut cal2 = r.bytes(16);
            if seed == 0 {
                // G1 = G2 = G3 = 0 keeps the initial heater resistance in range.
                cal2[10..14].fill(0);
            }
            let heat = if seed == 0 { vec![0, 0, 0] } else { r.bytes(3) };
            vec![
                (0xD0, vec![0x61]),
                (0x88, r.bytes(26)),
                (0xE1, cal2),
                (0x00, heat),
                (0x04, r.bytes(1)),
                (0x1D, vec![0x80]),
                (0x1F, r.bytes(8)),
                (0x2A, vec![r.byte(), 0x30 | (r.byte() & 0xCF)]),
            ]
        }
        Chip::Bmp180 => {
            let cal = if seed == 0 { bmp180_cal() } else { r.bytes(22) };
            let data = if seed == 0 { vec![0x6C, 0xFA, 0x00] } else { r.bytes(3) };
            vec![(0xD0, vec![0x55]), (0xAA, cal), (0xF6, data)]
        }
        Chip::Bmp388 => vec![
            (0xD0, vec![0x00]),
            (0x00, vec![0x50]),
            (0x31, r.bytes(21)),
            (0x03, vec![0x30]),
            (0x04, r.bytes(3)),
            (0x07, r.bytes(3)),
        ],
    }
}

/// Fresh measurement registers for the next sample (no register the
/// in-progress poll of the BME680 reads is left with bit 5 set).
fn sample_mem(chip: Chip, r: &mut Lcg) -> Vec<(usize, Vec<u8>)> {
    match chip {
        Chip::Bme280 => vec![(0xF7, r.bytes(8))],
        Chip::Bmp280 => vec![(0xF7, r.bytes(6))],
        Chip::Bme680 => {
            let status = r.pick(&[0x80u8, 0x80, 0x80, 0x00, 0x40, 0x90]);
            let gas_flags = r.pick(&[0x30u8, 0x20, 0x00, 0x10, 0x3F, 0x25, 0xE5]);
            vec![
                (0x1D, vec![status]),
                (0x1F, r.bytes(8)),
                (0x2A, vec![r.byte(), gas_flags]),
                (0x04, r.bytes(1)),
            ]
        }
        Chip::Bmp180 => vec![(0xF6, r.bytes(3))],
        Chip::Bmp388 => {
            let status = r.pick(&[0x00u8, 0x10, 0x20, 0x30, 0x70, 0xFF]);
            vec![(0x03, vec![status]), (0x04, r.bytes(3)), (0x07, r.bytes(3))]
        }
    }
}

fn sample_kind(chip: Chip) -> Sample {
    match chip {
        Chip::Bme280 | Chip::Bmp280 => Sample::Bme280,
        Chip::Bme680 => Sample::Bme680,
        Chip::Bmp180 => Sample::Bmp180,
        Chip::Bmp388 => Sample::Bmp388,
    }
}

/// Every response shape `bytearray(params['response'])` treats differently.
fn odd_responses() -> Vec<Resp> {
    vec![
        Resp::Err("MCU 'mcu' I2C request to addr 118 reports error NACK"),
        Resp::NoParams,
        Resp::NoKey,
        Resp::Bytes(vec![]),
        Resp::Bytes(vec![0x12]),
        Resp::Bytes(vec![0x12, 0x34, 0x56]),
        Resp::List(vec![1, 2, 3, 4, 5, 6, 7, 8]),
        Resp::List(vec![1, 2, 300]),
        Resp::List(vec![-1]),
        Resp::Str("abc"),
        Resp::Int(3),
        Resp::Int(-2),
        Resp::Float(1.5),
        Resp::Bytes(vec![0; 8]),
        Resp::Bytes(vec![0xFF; 8]),
    ]
}

// ---------------------------------------------------------------------------
// Tests: module-level helpers.
// ---------------------------------------------------------------------------

#[test]
fn helper_functions_match_python() {
    let Some(mut py) = oracle() else { return };
    for (val, bits) in [
        (0i64, 8i64),
        (127, 8),
        (128, 8),
        (255, 8),
        (-1, 8),
        (2047, 12),
        (2048, 12),
        (4095, 12),
        (32767, 16),
        (32768, 16),
        (65535, 16),
        (0, 16),
    ] {
        let label = format!("twos_complement({val},{bits})");
        let expected = py.eval(&format!("bme280.get_twos_complement({val}, {bits})"));
        check_result(&label, expected, Ok(Value::int(get_twos_complement(val, bits))));
    }
    for v in [0i64, 1, 127, 128, 200, 255] {
        let expected = py.eval(&format!("bme280.get_signed_byte({v})"));
        check_result(&format!("signed_byte({v})"), expected, Ok(Value::int(get_signed_byte(v))));
    }
    let pairs: [(u8, u8); 8] =
        [(1, 2), (255, 255), (0, 128), (7, 0), (0x80, 0x00), (0x7F, 0xFF), (0x12, 0xFE), (0, 0)];
    for (a, b) in pairs {
        let bits = [a, b];
        let arg = format!("bytearray([{a}, {b}])");
        let cases: [(&str, KResult<i64>); 4] = [
            ("get_unsigned_short", get_unsigned_short(&bits)),
            ("get_signed_short", get_signed_short(&bits)),
            ("get_unsigned_short_msb", get_unsigned_short_msb(&bits)),
            ("get_signed_short_msb", get_signed_short_msb(&bits)),
        ];
        for (name, actual) in cases {
            check_result(
                &format!("{name}({a},{b})"),
                py.eval(&format!("bme280.{name}({arg})")),
                actual.map(Value::int),
            );
        }
    }
    // Short inputs raise IndexError.
    for short in [&[][..], &[5u8][..]] {
        let arg = format!("bytearray({})", py_bytes_list(short));
        let cases: [(&str, KResult<i64>); 4] = [
            ("get_unsigned_short", get_unsigned_short(short)),
            ("get_signed_short", get_signed_short(short)),
            ("get_unsigned_short_msb", get_unsigned_short_msb(short)),
            ("get_signed_short_msb", get_signed_short_msb(short)),
        ];
        for (name, actual) in cases {
            check_result(
                &format!("{name} short {short:?}"),
                py.eval(&format!("bme280.{name}({arg})")),
                actual.map(Value::int),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Tests: construction.
// ---------------------------------------------------------------------------

#[test]
fn construction_and_config_match_python() {
    let Some(mut py) = oracle() else { return };
    let all = [
        ("bme280_iir_filter", "15"),
        ("bme280_oversample_temp", "4"),
        ("bme280_oversample_hum", "6"),
        ("bme280_oversample_pressure", "3"),
        ("bme280_gas_target_temp", "250"),
        ("bme280_gas_heat_duration", "200"),
    ];
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        (SECTION, vec![]),
        (SECTION, all.to_vec()),
        (SECTION, vec![("bme280_iir_filter", "0")]),
        (SECTION, vec![("bme280_iir_filter", "-1")]),
        (SECTION, vec![("bme280_iir_filter", "7")]),
        (SECTION, vec![("bme280_iir_filter", "8")]),
        (SECTION, vec![("bme280_iir_filter", "70")]),
        (SECTION, vec![("bme280_oversample_temp", "0")]),
        (SECTION, vec![("bme280_oversample_temp", "-5"), ("bme280_oversample_hum", "-1")]),
        (SECTION, vec![("bme280_oversample_pressure", "100"), ("bme280_oversample_hum", "65")]),
        (SECTION, vec![("bme280_oversample_pressure", "1000")]),
        (SECTION, vec![("bme280_gas_target_temp", "-20"), ("bme280_gas_heat_duration", "0")]),
        (SECTION, vec![("bme280_iir_filter", "abc")]),
        (SECTION, vec![("bme280_oversample_temp", "1.5")]),
        (SECTION, vec![("bme280_oversample_hum", "")]),
        (SECTION, vec![("bme280_oversample_pressure", "x")]),
        (SECTION, vec![("bme280_gas_target_temp", "hot")]),
        (SECTION, vec![("bme280_gas_heat_duration", "1e3")]),
        (SECTION, vec![("i2c_address", "119")]),
        (SECTION, vec![("i2c_address", "128")]),
        (SECTION, vec![("i2c_address", "-1")]),
        (SECTION, vec![("i2c_address", "high")]),
        (SECTION, vec![("i2c_speed", "400000")]),
        (SECTION, vec![("i2c_speed", "99999")]),
        ("bme280 temp1", vec![]),
        ("temperature_sensor  bar baz", vec![]),
        ("chamber", vec![]),
        (" ", vec![]),
    ];
    for (i, (section, options)) in cases.iter().enumerate() {
        let label = format!("config[{i}] {section:?} {options:?}");
        let (expected, actual) = build_pair(&mut py, section, options, &[], false);
        match (expected, actual) {
            (Ok(_), Ok(b)) => {
                check(&format!("{label} trace"), py.eval("trace()"), b.trace());
                check(&format!("{label} info"), py.eval("info()"), b.info());
                check(
                    &format!("{label} last_gas_time"),
                    py.eval("gas_time0()"),
                    b.obj.last_gas_time().map_or(Value::Null, Value::float),
                );
                check(
                    &format!("{label} registration"),
                    py.eval("(list(printer.objects), [e for e, _ in printer.handlers], obj.get_report_time_delta())"),
                    Value::Tuple(vec![
                        Value::List(
                            b.env.printer.object_names().iter().map(|n| Value::str(n)).collect(),
                        ),
                        Value::List(
                            b.env
                                .printer
                                .own_handler_events()
                                .iter()
                                .map(|e| Value::str(e))
                                .collect(),
                        ),
                        Value::float(TemperatureSensorApi::get_report_time_delta(&*b.obj)),
                    ]),
                );
                // The module hands `bus.MCU_I2C_from_config` its defaults; the
                // real bus turns them into the config commands.
                let calls = py.eval("from_config_calls").expect("py from_config calls");
                let (addr, speed) = from_config_defaults(&calls);
                assert_eq!(addr, BME280_CHIP_ADDR, "{label}: default address");
                assert_eq!(speed, BME280_I2C_SPEED, "{label}: default speed");
                let py_speed = value_int(&py.eval("i2c.i2c_speed").expect("py speed"));
                let cmds = b.env.mcu.config_cmds.borrow().clone();
                assert_eq!(cmds[0], "config_i2c oid=1", "{label}");
                let expected_addr = py.eval("i2c.i2c_address").expect("py address");
                let want_tail = format!(" rate={py_speed} address={}", value_int(&expected_addr));
                assert!(
                    cmds[1].starts_with("i2c_set_bus oid=1 i2c_bus=") && cmds[1].ends_with(&want_tail),
                    "{label}: bus config command {:?}",
                    cmds[1]
                );
            }
            (Ok(_), Err(e)) => panic!("{label}: port raised {}: {}", e.class_name(), e.message()),
            (Err(py_exc), Err(e)) => {
                assert_eq!(e.class_name(), py_exc.class_name, "{label}: error class");
                assert_eq!(e.message(), py_exc.message, "{label}: error message");
                // Logs written before the failure are part of the contract too.
                check(
                    &format!("{label} logs"),
                    py.eval("[(l, m, x) for (l, m, x) in capture.records]"),
                    Value::List(
                        take_logs()
                            .into_iter()
                            .map(|(l, m, x)| {
                                Value::Tuple(vec![Value::str(&l), Value::str(&m), Value::Bool(x)])
                            })
                            .collect(),
                    ),
                );
            }
            (Err(py_exc), Ok(_)) => {
                panic!("{label}: oracle raised {}: {}, port built", py_exc.class_name, py_exc.message)
            }
        }
    }
}

fn value_int(v: &Value) -> i64 {
    match v {
        Value::Int(n) => *n,
        _ => panic!("expected int value, got {}", v.render()),
    }
}

/// `(default_addr, default_speed)` of the single recorded
/// `MCU_I2C_from_config` call.
fn from_config_defaults(calls: &Value) -> (i64, i64) {
    let Value::List(calls) = calls else { panic!("calls is a list: {}", calls.render()) };
    assert_eq!(calls.len(), 1, "one MCU_I2C_from_config call");
    let Value::Tuple(call) = &calls[0] else { panic!("call is a tuple") };
    (value_int(&call[0]), value_int(&call[1]))
}

#[test]
fn debugoutput_and_duplicate_match_python() {
    let Some(mut py) = oracle() else { return };
    for (label, start_args) in [
        ("debugoutput set", vec![("debugoutput", Some("out.bin"))]),
        ("debugoutput none", vec![("debugoutput", None)]),
        ("other start arg", vec![("gcode", Some("x"))]),
    ] {
        let (expected, actual) = build_pair(&mut py, SECTION, &[], &start_args, false);
        expected.expect("py build");
        let b = actual.expect("port build");
        check(
            &format!("{label} registration"),
            py.eval("(list(printer.objects), [e for e, _ in printer.handlers])"),
            Value::Tuple(vec![
                Value::List(b.env.printer.object_names().iter().map(|n| Value::str(n)).collect()),
                Value::List(
                    b.env.printer.own_handler_events().iter().map(|e| Value::str(e)).collect(),
                ),
            ]),
        );
        check(
            &format!("{label} last_gas_time"),
            py.eval("gas_time0()"),
            b.obj.last_gas_time().map_or(Value::Null, Value::float),
        );
        check(&format!("{label} trace"), py.eval("trace()"), b.trace());
        // Without the connect handler `last_gas_time` does not exist.
        if b.obj.last_gas_time().is_none() {
            b.set_callback(&mut py);
            b.set_limits(&mut py, -100., 1000.);
            let expected = py.eval("obj._sample_bme680(0)");
            check_result(
                &format!("{label} sample_bme680"),
                expected,
                b.obj.sample_bme680(0.).map(Value::float),
            );
        }
    }

    let (expected, actual) = build_pair(&mut py, SECTION, &[], &[], true);
    check_result(
        "duplicate object",
        expected.map(|_| Value::Null),
        actual.map(|_| Value::Null),
    );
    check(
        "duplicate logs",
        py.eval("[(l, m, x) for (l, m, x) in capture.records]"),
        Value::List(
            take_logs()
                .into_iter()
                .map(|(l, m, x)| Value::Tuple(vec![Value::str(&l), Value::str(&m), Value::Bool(x)]))
                .collect(),
        ),
    );
}

// ---------------------------------------------------------------------------
// Tests: register access.
// ---------------------------------------------------------------------------

#[test]
fn register_access_matches_python() {
    let Some(mut py) = oracle() else { return };
    let b = build_ok(&mut py, &[]);
    b.set_mem(&mut py, 0x88, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
    b.set_mem(&mut py, 0xD0, &[0x60]);
    b.set_mem(&mut py, 0x00, &[0x50, 0x00]);

    // Reads through the default (BME280) register map, with every response shape.
    let mut i = 0;
    for resp in std::iter::once(None).chain(odd_responses().into_iter().map(Some)) {
        for (reg, len) in [("CAL_1", 4i64), ("STATUS", 1), ("PRESSURE_MSB", 8), ("NOPE", 1)] {
            let label = format!("read[{i}] {reg} {resp:?}");
            i += 1;
            if let (Some(resp), true) = (&resp, reg != "NOPE") {
                b.arm_read(&mut py, 1, resp);
            }
            b.compare(
                &mut py,
                &label,
                &format!("list(obj.read_register({reg:?}, {len}))"),
                b.obj.read_register(reg, len).map(|v| ints(&v)),
            );
            b.disarm(&mut py);
        }
    }

    // `read_id` and `read_bmp3_id`.
    for resp in std::iter::once(None).chain(odd_responses().into_iter().map(Some)) {
        let label = format!("read_id {resp:?}");
        if let Some(resp) = &resp {
            b.arm_read(&mut py, 1, resp);
        }
        b.compare(&mut py, &label, "obj.read_id()", b.obj.read_id().map(Value::int));
        b.disarm(&mut py);
        let label = format!("read_bmp3_id {resp:?}");
        if let Some(resp) = &resp {
            b.arm_read(&mut py, 1, resp);
        }
        b.compare(&mut py, &label, "obj.read_bmp3_id()", b.obj.read_bmp3_id().map(Value::int));
        b.disarm(&mut py);
    }

    // Writes: list and scalar forms, unknown register, bytes outside range(0, 256).
    let writes: &[(&str, &str, &[i64])] = &[
        ("CONFIG", "0x10", &[0x10]),
        ("CTRL_MEAS", "[0x27]", &[0x27]),
        ("CTRL_HUM", "255", &[255]),
        ("CAL_1", "[1, 2, 3]", &[1, 2, 3]),
        ("RESET", "[]", &[]),
        ("CONFIG", "256", &[256]),
        ("CONFIG", "-1", &[-1]),
        ("CONFIG", "[1, 300]", &[1, 300]),
        ("NOPE", "1", &[1]),
        ("PWR_CTRL", "[1]", &[1]),
    ];
    for (i, (reg, py_data, data)) in writes.iter().enumerate() {
        b.compare(
            &mut py,
            &format!("write[{i}] {reg}"),
            &format!("obj.write_register({reg:?}, {py_data})"),
            b.obj.write_register(reg, data).map(|_| Value::Null),
        );
    }
    // A failing transport write propagates.
    b.arm_write(&mut py, 1);
    b.compare(
        &mut py,
        "write transport failure",
        "obj.write_register('CONFIG', [1])",
        b.obj.write_register("CONFIG", &[1]).map(|_| Value::Null),
    );
    b.disarm(&mut py);
}

// ---------------------------------------------------------------------------
// Tests: one per chip family.
// ---------------------------------------------------------------------------

fn gas_ranges() -> Vec<i64> {
    let mut v: Vec<i64> = (0..16).collect();
    v.extend([-1, 16, 99]);
    v
}

/// Connect, then sample `rounds` times with random measurement registers and
/// a wide temperature window; then shutdown, callback and failure paths.
fn sample_flow(py: &mut pyoracle::Oracle, b: &Built, chip: Chip, seed: u64, rounds: usize) {
    let which = sample_kind(chip);
    let mut r = Lcg::new(seed + 1000);
    b.set_callback(py);
    b.set_limits(py, -1e12, 1e12);
    for round in 0..rounds {
        b.apply_mem(py, &sample_mem(chip, &mut r));
        sample_step(py, b, &format!("{chip:?} seed {seed} sample {round}"), which);
    }

    // Shutdown when the temperature leaves [min, max], in both directions.
    for (min_temp, max_temp) in [(0., 1.), (-1e12, -1e11), (1e11, 1e12)] {
        b.set_limits(py, min_temp, max_temp);
        b.apply_mem(py, &sample_mem(chip, &mut r));
        sample_step(py, b, &format!("{chip:?} limits {min_temp}..{max_temp}"), which);
    }
    b.set_limits(py, -1e12, 1e12);

    // Failing and odd responses to the first read of the sample.
    for (i, resp) in odd_responses().iter().enumerate() {
        b.apply_mem(py, &sample_mem(chip, &mut r));
        b.arm_read(py, 1, resp);
        sample_step(py, b, &format!("{chip:?} first read {i} {resp:?}"), which);
        b.disarm(py);
    }
    // Later reads of the sample fail: Err, short and odd.
    for k in 2..=4 {
        for resp in [
            Resp::Err("Timeout on wait for 'i2c_read_response' response"),
            Resp::Bytes(vec![0x80]),
            Resp::NoParams,
        ] {
            b.apply_mem(py, &sample_mem(chip, &mut r));
            b.arm_read(py, k, &resp);
            sample_step(py, b, &format!("{chip:?} read {k} {resp:?}"), which);
            b.disarm(py);
        }
    }
}

/// No callback installed: AttributeError after the shutdown check. It builds
/// a fresh pair, which replaces the Python globals `obj` and `i2c`, so a test
/// must not use an earlier `Built` after calling it.
fn no_callback_flow(py: &mut pyoracle::Oracle, chip: Chip, seed: u64) {
    let which = sample_kind(chip);
    let mut r = Lcg::new(seed + 2000);
    let (expected, actual) = build_pair(py, SECTION, &[], &[], false);
    expected.expect("py build no callback");
    let fresh = actual.expect("port build no callback");
    fresh.apply_mem(py, &chip_mem(chip, seed));
    fresh.connect(py, &format!("{chip:?} no-callback connect"));
    fresh.apply_mem(py, &sample_mem(chip, &mut r));
    sample_step(py, &fresh, &format!("{chip:?} no callback in range"), which);
    fresh.set_limits(py, 0., 1.);
    sample_step(py, &fresh, &format!("{chip:?} no callback out of range"), which);
}

/// Connect on a fresh pair of the chip family and compare everything.
fn connected(
    py: &mut pyoracle::Oracle,
    chip: Chip,
    seed: u64,
    options: &[(&str, &str)],
) -> (Built, bool) {
    let b = build_ok(py, options);
    b.apply_mem(py, &chip_mem(chip, seed));
    let ok = b.connect(py, &format!("{chip:?} seed {seed} {options:?} connect"));
    (b, ok)
}

/// [`connected`] with an explicit register file.
fn connected_mem(py: &mut pyoracle::Oracle, label: &str, mem: &[(usize, Vec<u8>)]) -> (Built, bool) {
    let b = build_ok(py, &[]);
    b.apply_mem(py, mem);
    let ok = b.connect(py, label);
    (b, ok)
}

/// Failures of every read and every write of `_init_bmxx80`.
fn init_failures(py: &mut pyoracle::Oracle, chip: Chip, reads: usize, writes: usize) {
    for k in 1..=reads {
        let b = build_ok(py, &[]);
        b.apply_mem(py, &chip_mem(chip, 0));
        b.arm_read(py, k, &Resp::Err("Timeout on wait for 'i2c_read_response' response"));
        b.connect(py, &format!("{chip:?} init read failure {k}"));
        let b = build_ok(py, &[]);
        b.apply_mem(py, &chip_mem(chip, 0));
        b.arm_read(py, k, &Resp::Bytes(vec![]));
        b.connect(py, &format!("{chip:?} init empty read {k}"));
    }
    for k in 1..=writes {
        let b = build_ok(py, &[]);
        b.apply_mem(py, &chip_mem(chip, 0));
        b.arm_write(py, k);
        b.connect(py, &format!("{chip:?} init write failure {k}"));
    }
}

#[test]
fn chip_bme280_matches_python() {
    let Some(mut py) = oracle() else { return };
    for seed in 0..3u64 {
        let (b, ok) = connected(&mut py, Chip::Bme280, seed, &[]);
        assert!(ok, "BME280 init succeeds");
        sample_flow(&mut py, &b, Chip::Bme280, seed, 8);

        // Compensation helpers over a raw value grid; each feeds `t_fine`.
        for raw in [0i64, 1, 16384, 262144, 500000, 519888, 0x7FFFF, 0x80000, 0xFFFFF] {
            b.compare(
                &mut py,
                &format!("temp {raw}"),
                &format!("obj._compensate_temp({raw})"),
                b.obj.compensate_temp(raw).map(Value::float),
            );
            b.compare(
                &mut py,
                &format!("pressure bme280 {raw}"),
                &format!("obj._compensate_pressure_bme280({raw})"),
                b.obj.compensate_pressure_bme280(raw).map(Value::float),
            );
            b.compare(
                &mut py,
                &format!("pressure bme680 {raw}"),
                &format!("obj._compensate_pressure_bme680({raw})"),
                b.obj.compensate_pressure_bme680(raw).map(Value::float),
            );
        }
        for raw in [0i64, 1, 20000, 27000, 40000, 65535] {
            b.compare(
                &mut py,
                &format!("humidity bme280 {raw}"),
                &format!("obj._compensate_humidity_bme280({raw})"),
                b.obj.compensate_humidity_bme280(raw).map(Value::float),
            );
            b.compare(
                &mut py,
                &format!("humidity bme680 {raw}"),
                &format!("obj._compensate_humidity_bme680({raw})"),
                b.obj.compensate_humidity_bme680(raw).map(Value::float),
            );
        }
        no_callback_flow(&mut py, Chip::Bme280, seed);
    }
    init_failures(&mut py, Chip::Bme280, 4, 5);
}

#[test]
fn chip_bmp280_matches_python() {
    let Some(mut py) = oracle() else { return };
    for seed in 0..3u64 {
        let (b, ok) = connected(&mut py, Chip::Bmp280, seed, &[]);
        assert!(ok, "BMP280 init succeeds");
        sample_flow(&mut py, &b, Chip::Bmp280, seed, 8);
        no_callback_flow(&mut py, Chip::Bmp280, seed);
    }
    init_failures(&mut py, Chip::Bmp280, 4, 4);
}

/// `max_standby_time` selects `t_sb`; sweep the oversampling options through
/// every threshold. `CTRL_MEAS` fails for large values after `CONFIG` was
/// written, which keeps the failure itself under comparison.
#[test]
fn standby_time_thresholds_match_python() {
    let Some(mut py) = oracle() else { return };
    // `max_standby_time = REPORT_TIME - max_sample_time`; with the default
    // temperature and pressure oversampling of 2 the humidity (BME280) or the
    // pressure (BMP280) oversampling is swept through the sums that land on
    // each `t_sb` threshold, including negative values for the `> 1` and
    // `> 4` branches.
    let sums: [i64; 8] = [-100, 6, 150, 250, 300, 330, 340, 400];
    let iirs = ["1", "0", "5", "9"];
    for (i, sum) in sums.iter().enumerate() {
        let hum = (sum - 4).to_string();
        let options = [
            ("bme280_oversample_hum", hum.as_str()),
            ("bme280_iir_filter", iirs[i % iirs.len()]),
        ];
        connected(&mut py, Chip::Bme280, 0, &options);
    }
    let sums: [i64; 8] = [-1400, -600, -100, 6, 150, 250, 300, 400];
    for (i, sum) in sums.iter().enumerate() {
        let pres = (sum - 2).to_string();
        let options = [
            ("bme280_oversample_pressure", pres.as_str()),
            ("bme280_iir_filter", iirs[i % iirs.len()]),
        ];
        connected(&mut py, Chip::Bmp280, 0, &options);
    }
    // A negative temperature oversampling drives the `> 1` branch of the BME280
    // through `max_sample_time` as well.
    connected(&mut py, Chip::Bme280, 0, &[("bme280_oversample_temp", "-120")]);
}

#[test]
fn chip_bme680_matches_python() {
    let Some(mut py) = oracle() else { return };
    for seed in 0..2u64 {
        let (b, ok) = connected(&mut py, Chip::Bme680, seed, &[]);
        if seed == 0 {
            assert!(ok, "BME680 init succeeds with in-range heater values");
        }

        // Gas run on the first sample (`last_gas_time` is 0), the in-progress
        // poll, the "no new data" return, and the temp/pressure skip values.
        b.set_callback(&mut py);
        b.set_limits(&mut py, -1e12, 1e12);
        b.queue_reg(&mut py, 0x1D, &[Resp::Bytes(vec![0x20]), Resp::Bytes(vec![0x20]), Resp::Bytes(vec![0x80])]);
        sample_step(&mut py, &b, "bme680 in-progress poll", Sample::Bme680);
        b.disarm(&mut py);
        let skip_raw = vec![0x01, 0x02, 0x03, 0x80, 0x00, 0x00, 0x07, 0x08];
        b.set_mem(&mut py, 0x1F, &skip_raw);
        sample_step(&mut py, &b, "bme680 temp 0x80000", Sample::Bme680);
        let skip_raw = vec![0x80, 0x00, 0x00, 0x04, 0x05, 0x06, 0x07, 0x08];
        b.set_mem(&mut py, 0x1F, &skip_raw);
        sample_step(&mut py, &b, "bme680 pressure 0x80000", Sample::Bme680);
        b.set_mem(&mut py, 0x1D, &[0x00]);
        sample_step(&mut py, &b, "bme680 no new data", Sample::Bme680);
        b.set_mem(&mut py, 0x1D, &[0x80]);

        // Heater helpers (they read the register file and use `self.temp`).
        let mut r = Lcg::new(seed + 500);
        for target in [-100i64, 0, 25, 320, 400, 1000, 100000] {
            b.set_mem(&mut py, 0x00, &r.bytes(3));
            b.compare(
                &mut py,
                &format!("heater resistance {target}"),
                &format!("obj._calc_gas_heater_resistance({target})"),
                b.obj.calc_gas_heater_resistance(target).map(Value::int),
            );
        }
        for dur in [-5i64, 0, 1, 63, 64, 65, 150, 255, 256, 1000, 4031, 4032, 5000] {
            b.compare(
                &mut py,
                &format!("heater duration {dur}"),
                &format!("obj._calc_gas_heater_duration({dur})"),
                Ok(Value::int(b.obj.calc_gas_heater_duration(dur))),
            );
        }

        // Compensation helpers over a raw value grid.
        for raw in [0i64, 1, 16384, 262144, 500000, 0x7FFFF, 0xFFFFF] {
            b.compare(
                &mut py,
                &format!("temp {raw}"),
                &format!("obj._compensate_temp({raw})"),
                b.obj.compensate_temp(raw).map(Value::float),
            );
            b.compare(
                &mut py,
                &format!("pressure bme680 {raw}"),
                &format!("obj._compensate_pressure_bme680({raw})"),
                b.obj.compensate_pressure_bme680(raw).map(Value::float),
            );
        }
        for raw in [0i64, 1, 20000, 40000, 65535] {
            b.compare(
                &mut py,
                &format!("humidity bme680 {raw}"),
                &format!("obj._compensate_humidity_bme680({raw})"),
                b.obj.compensate_humidity_bme680(raw).map(Value::float),
            );
        }
        for range in gas_ranges() {
            for raw in [-828i64, 0, 512, 1023, 8192] {
                b.set_mem(&mut py, 0x04, &r.bytes(1));
                b.compare(
                    &mut py,
                    &format!("gas raw {raw} range {range}"),
                    &format!("obj._compensate_gas({raw}, {range})"),
                    b.obj.compensate_gas(raw, range).map(Value::float),
                );
            }
        }
        // A zero divisor: range 0 has factor 1 and RANGE_SWITCHING_ERROR 0
        // gives var1 = 1340, so raw 512 - 1340 cancels it.
        b.set_mem(&mut py, 0x04, &[0]);
        b.compare(
            &mut py,
            "gas ZeroDivisionError",
            "obj._compensate_gas(-828, 0)",
            b.obj.compensate_gas(-828, 0).map(Value::float),
        );

        // The full sample flow, including the gas-run bookkeeping.
        sample_flow(&mut py, &b, Chip::Bme680, seed, 4);
        no_callback_flow(&mut py, Chip::Bme680, seed);
    }

    // Gas options: target temperature and heat duration feed the init writes.
    for options in [
        vec![("bme280_gas_target_temp", "200"), ("bme280_gas_heat_duration", "100")],
        vec![("bme280_gas_target_temp", "0"), ("bme280_gas_heat_duration", "4032")],
        vec![("bme280_gas_target_temp", "400"), ("bme280_gas_heat_duration", "63")],
        vec![("bme280_oversample_hum", "9"), ("bme280_iir_filter", "-2")],
        vec![("bme280_oversample_temp", "9"), ("bme280_oversample_pressure", "70")],
    ] {
        connected(&mut py, Chip::Bme680, 0, &options);
    }
    init_failures(&mut py, Chip::Bme680, 5, 8);
}

#[test]
fn chip_bmp180_matches_python() {
    let Some(mut py) = oracle() else { return };
    for seed in 0..3u64 {
        let (b, ok) = connected(&mut py, Chip::Bmp180, seed, &[]);
        assert!(ok, "BMP180 init succeeds");
        sample_flow(&mut py, &b, Chip::Bmp180, seed, 8);
        for raw in [0i64, 1, 8000, 27898, 40000, 65535] {
            b.compare(
                &mut py,
                &format!("bmp180 temp {raw}"),
                &format!("obj._compensate_temp_bmp180({raw})"),
                b.obj.compensate_temp_bmp180(raw).map(Value::float),
            );
            for praw in [0i64, 1, 23843, 65000, 400000] {
                b.compare(
                    &mut py,
                    &format!("bmp180 pressure {praw} after temp {raw}"),
                    &format!("obj._compensate_pressure_bmp180({praw})"),
                    b.obj.compensate_pressure_bmp180(praw).map(Value::float),
                );
            }
        }
        no_callback_flow(&mut py, Chip::Bmp180, seed);
    }

    // Oversampling bits: valid, and values whose shifts or writes fail.
    for os in ["0", "1", "2", "3", "4", "9", "-1", "70"] {
        let options = [("bme280_oversample_pressure", os)];
        let (b, ok) = connected(&mut py, Chip::Bmp180, 0, &options);
        assert!(ok, "BMP180 init does not use the oversampling");
        b.set_callback(&mut py);
        b.set_limits(&mut py, -1e12, 1e12);
        sample_step(&mut py, &b, &format!("bmp180 os_pres {os} sample"), Sample::Bmp180);
        for praw in [0i64, 1, 23843, 65000, 400000] {
            b.compare(
                &mut py,
                &format!("bmp180 os_pres {os} pressure {praw}"),
                &format!("obj._compensate_pressure_bmp180({praw})"),
                b.obj.compensate_pressure_bmp180(praw).map(Value::float),
            );
        }
    }

    // Zero divisors: MD = 0 with raw temperature = AC6 (x1 + MD == 0), and
    // AC4 = 0 (b4 == 0).
    let mut cal = bmp180_cal();
    cal[20] = 0;
    cal[21] = 0;
    let mem = [(0xD0usize, vec![0x55u8]), (0xAA, cal), (0xF6, vec![0x6C, 0xFA, 0x00])];
    let (b2, _) = connected_mem(&mut py, "bmp180 md=0 connect", &mem);
    b2.compare(
        &mut py,
        "bmp180 temp ZeroDivisionError",
        "obj._compensate_temp_bmp180(0x5A71)",
        b2.obj.compensate_temp_bmp180(0x5A71).map(Value::float),
    );
    let mut cal = bmp180_cal();
    cal[6] = 0;
    cal[7] = 0;
    let mem = [(0xD0usize, vec![0x55u8]), (0xAA, cal), (0xF6, vec![0x6C, 0xFA, 0x00])];
    let (b3, _) = connected_mem(&mut py, "bmp180 ac4=0 connect", &mem);
    b3.set_callback(&mut py);
    b3.set_limits(&mut py, -1e12, 1e12);
    sample_step(&mut py, &b3, "bmp180 ac4=0 sample", Sample::Bmp180);

    init_failures(&mut py, Chip::Bmp180, 3, 2);
}

#[test]
fn chip_bmp388_matches_python() {
    let Some(mut py) = oracle() else { return };
    for seed in 0..3u64 {
        let (b, ok) = connected(&mut py, Chip::Bmp388, seed, &[]);
        assert!(ok, "BMP388 init succeeds");
        sample_flow(&mut py, &b, Chip::Bmp388, seed, 10);
        // The temperature and pressure helpers can be called on their own.
        b.apply_mem(&mut py, &[(0x07, vec![0xAA, 0xBB, 0x0C]), (0x04, vec![0x11, 0x22, 0x33])]);
        b.compare(
            &mut py,
            "bmp388 temp",
            "obj._sample_bmp388_temp()",
            b.obj.sample_bmp388_temp().map(Value::float),
        );
        b.compare(
            &mut py,
            "bmp388 press",
            "obj._sample_bmp388_press()",
            b.obj.sample_bmp388_press().map(number_value),
        );
        no_callback_flow(&mut py, Chip::Bmp388, seed);
    }
    init_failures(&mut py, Chip::Bmp388, 5, 7);
}

// ---------------------------------------------------------------------------
// Tests: chip identification.
// ---------------------------------------------------------------------------

#[test]
fn chip_identification_matches_python() {
    let Some(mut py) = oracle() else { return };
    let cal = |r: &mut Lcg| vec![(0x88usize, r.bytes(26)), (0xE1, r.bytes(16))];
    let mut r = Lcg::new(5);
    // (0xD0 value, 0x00 value)
    for (id, id2) in [
        (0x99u8, 0u8),
        (0x00, 0x00),
        (0x00, 0x50),
        (0x00, 0x61),
        (0x00, 0x99),
        (0x58, 0x00),
        (0x61, 0x50),
        (0xFF, 0xFF),
    ] {
        let b = build_ok(&mut py, &[]);
        let mut mem = cal(&mut r);
        mem.push((0xD0, vec![id]));
        mem.push((0x00, vec![id2, 0, 0]));
        mem.push((0xF7, r.bytes(8)));
        b.apply_mem(&mut py, &mem);
        b.connect(&mut py, &format!("ids {id:#x} {id2:#x} connect"));
        b.set_callback(&mut py);
        b.set_limits(&mut py, -1e12, 1e12);
        for round in 0..2 {
            b.apply_mem(&mut py, &[(0xF7, r.bytes(8))]);
            sample_step(
                &mut py,
                &b,
                &format!("ids {id:#x} {id2:#x} sample {round}"),
                Sample::Bme280,
            );
        }
    }

    // The NVM copy poll: `STATUS` bit 0 stays set for a few reads.
    for polls in [1usize, 3] {
        let b = build_ok(&mut py, &[]);
        let mut mem = cal(&mut r);
        mem.push((0xD0, vec![0x60]));
        b.apply_mem(&mut py, &mem);
        let queue: Vec<Resp> = (0..polls).map(|_| Resp::Bytes(vec![0x01])).collect();
        b.queue_reg(&mut py, 0xF3, &queue);
        b.connect(&mut py, &format!("status poll {polls}"));
        b.disarm(&mut py);
    }
    // A poll answer that is not a byte string.
    for resp in odd_responses() {
        let b = build_ok(&mut py, &[]);
        let mut mem = cal(&mut r);
        mem.push((0xD0, vec![0x60]));
        b.apply_mem(&mut py, &mem);
        b.queue_reg(&mut py, 0xF3, std::slice::from_ref(&resp));
        b.connect(&mut py, &format!("status poll {resp:?}"));
        b.disarm(&mut py);
    }
}

// ---------------------------------------------------------------------------
// Tests: calls on an object whose chip was never probed.
// ---------------------------------------------------------------------------

#[test]
fn uninitialised_object_matches_python() {
    let Some(mut py) = oracle() else { return };
    let b = build_ok(&mut py, &[]);
    check("initial info", py.eval("info()"), b.info());
    check(
        "initial status",
        py.eval("obj.get_status(0.)"),
        b.status(),
    );
    // No calibration yet: `self.dig` is None.
    b.compare(&mut py, "temp", "obj._compensate_temp(5)", b.obj.compensate_temp(5).map(Value::float));
    b.compare(
        &mut py,
        "pressure bme280",
        "obj._compensate_pressure_bme280(5)",
        b.obj.compensate_pressure_bme280(5).map(Value::float),
    );
    b.compare(
        &mut py,
        "humidity bme680",
        "obj._compensate_humidity_bme680(5)",
        b.obj.compensate_humidity_bme680(5).map(Value::float),
    );
    b.compare(
        &mut py,
        "bmp180 temp",
        "obj._compensate_temp_bmp180(5)",
        b.obj.compensate_temp_bmp180(5).map(Value::float),
    );
    // Register names of other chip families are KeyErrors on the BME280 map.
    b.compare(
        &mut py,
        "gas",
        "obj._compensate_gas(1, 0)",
        b.obj.compensate_gas(1, 0).map(Value::float),
    );
    b.compare(
        &mut py,
        "heater resistance",
        "obj._calc_gas_heater_resistance(300)",
        b.obj.calc_gas_heater_resistance(300).map(Value::int),
    );
    b.set_callback(&mut py);
    b.set_limits(&mut py, -1e12, 1e12);
    sample_step(&mut py, &b, "bme680 sample on BME280 map", Sample::Bme680);
    sample_step(&mut py, &b, "bmp180 sample on BME280 map", Sample::Bmp180);
    // The default chip type is BMP280, whose sample reads six bytes; without
    // calibration the compensation raises.
    sample_step(&mut py, &b, "bme280 sample without dig", Sample::Bme280);
    // BMP388 sample on the BME280 map: STATUS reads 0, so only the callback fires.
    sample_step(&mut py, &b, "bmp388 sample on BME280 map", Sample::Bmp388);
    b.apply_mem(&mut py, &[(0xF3, vec![0x30])]);
    sample_step(&mut py, &b, "bmp388 sample status set", Sample::Bmp388);
    b.compare(
        &mut py,
        "bmp388 temp without dig",
        "obj._sample_bmp388_temp()",
        b.obj.sample_bmp388_temp().map(Value::float),
    );
    b.compare(
        &mut py,
        "bmp388 press without dig",
        "obj._sample_bmp388_press()",
        b.obj.sample_bmp388_press().map(number_value),
    );
}

// ---------------------------------------------------------------------------
// Tests: load_config.
// ---------------------------------------------------------------------------

#[test]
fn load_config_registers_sensor_factory_matches_python() {
    let Some(mut py) = oracle() else { return };
    for with_heaters in [true, false] {
        let label = format!("load_config with_heaters={with_heaters}");
        let flag = if with_heaters { "True" } else { "False" };
        let py_result = py.exec(&format!("load_result = load({SECTION:?}, {flag})"));
        let env = Env::new(SECTION, &[], &[], false);
        if with_heaters {
            *env.printer.heaters.borrow_mut() =
                Some(Arc::new(FakeHeaters { factories: GCell::new(Vec::new()) }));
        }
        let load_config = module_factory().load_config.expect("load_config entry");
        let rust_result = load_config(&env.config).map(|obj| {
            assert!(obj.is_none(), "{label}: load_config returns None");
            Value::Null
        });
        check_result(&label, py_result.map(|_| Value::Null), rust_result);
        check(
            &format!("{label} loaded"),
            py.eval("list(load_printer.loaded)"),
            Value::List(env.printer.loaded.borrow().iter().map(|s| Value::str(s)).collect()),
        );
        if !with_heaters {
            continue;
        }
        let heaters = env.printer.heaters.cloned().expect("heaters present");
        let factories: Vec<(String, SensorFactory)> = heaters.factories.cloned();
        check(
            &format!("{label} factories"),
            py.eval("[t for t, is_bme in load_heaters.factories]"),
            Value::List(factories.iter().map(|(t, _)| Value::str(t)).collect()),
        );
        check(
            &format!("{label} factory is BME280"),
            py.eval("all(is_bme for t, is_bme in load_heaters.factories)"),
            Value::Bool(true),
        );

        // The factory builds the sensor: a printer object `bme280 <name>` that
        // exposes the sensor interface.
        let (_, factory) = &factories[0];
        let sensor = factory(&env.config).expect("factory builds the sensor");
        assert_eq!(sensor.get_report_time_delta().to_bits(), REPORT_TIME.to_bits());
        let names = env.printer.object_names();
        assert_eq!(names, vec!["bme280 foo".to_string()]);
        let obj = env.printer.objects.borrow()[0].1.clone();
        assert!(
            interface::<dyn TemperatureSensorApi>(&obj).is_some(),
            "bme280 object declares the sensor interface"
        );
        let expected = py.exec("build('temperature_sensor foo', {}, {}, False, 5.0)");
        expected.expect("py build");
        check(
            "factory object name",
            py.eval("list(printer.objects)"),
            Value::List(names.iter().map(|n| Value::str(n)).collect()),
        );
    }
}

#[test]
fn module_factory_has_load_config_only() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.bme280"), "entry points of extras.bme280");
}
