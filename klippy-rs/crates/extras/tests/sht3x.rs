//! Parity tests for `extras::sht3x` (port of `klippy/extras/sht3x.py`).
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`). The Python side builds
//! the real `sht3x.SHT3X` on minimal fakes: the printer (objects, event
//! handlers, shutdowns, a fake reactor with a settable clock), the I2C device
//! (`bus.MCU_I2C_from_config` is replaced; it replays a scripted list of read
//! results and records every write and read), the MCU
//! (`estimated_print_time`) and `logging` (records level, message and the
//! exception text). The config is the real `configfile.ConfigWrapper`, so
//! option defaults and config error texts come from `configfile.py`.
//!
//! The Rust side drives the port through fakes of the contract traits
//! (`PrinterApi`, `ConfigApi`, `McuApi`, `bus::McuI2cApi`,
//! `PrinterHeatersApi`) with identical inputs. The Rust fake I2C is handed to
//! `Sht3x::from_config`, the way `bus.MCU_I2C_from_config` supplies the real
//! device. Compared exactly (floats by bits): the constructor results and
//! config errors, the registered printer object and event handler, the
//! return value of `_sample_sht3x`, `temp`, `humidity`, the `get_status`
//! dict, the `(print_time, temp)` sensor callback calls, shutdown messages,
//! log records, I2C writes and reads (data, length, `retry`), the raised
//! errors of `handle_connect`, the CRC-8 over the whole 16 bit range, the
//! command table, and the module entry points. The reactor pauses of the
//! reference are summed on the Python side and checked against the elapsed
//! time of the Rust call.
//!
//! Not observable through the fakes: the reactor timer registration and
//! `update_timer(sample_timer, NOW)` (the Rust reactor is the real one).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit.

use std::collections::VecDeque;
use std::sync::Arc;

use extras::bus::McuI2cApi;
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureSensorApi,
};
use extras::sht3x::{self, cmd, Sht3x, SHT3X_I2C_ADDR};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, QuerySendOpts, SendOpts,
    SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, interface, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys
import traceback
import configfile
import gcode as gcode_mod
import extras.sht3x as sht3x
from configparser import RawConfigParser

NEVER = 9999999999999999.

from mcu import error

writes = []
reads = []
logs = []
shutdowns = []
cbs = []
pauses = []
i2c_args = []

def reset_rec():
    del writes[:]
    del reads[:]
    del logs[:]
    del shutdowns[:]
    del cbs[:]
    del pauses[:]

class FakeLogging:
    def debug(self, msg, *a):
        logs.append(('DEBUG', msg, ''))
    def warning(self, msg, *a):
        logs.append(('WARNING', msg, ''))
    def exception(self, msg, *a):
        # The final traceback line `logging.exception` prints.
        e = sys.exc_info()[1]
        logs.append(('ERROR', msg, traceback.format_exception_only(type(e), e)[-1].strip()))
sht3x.logging = FakeLogging()

class FakeReactor:
    NOW = 0.
    NEVER = NEVER
    def __init__(self):
        self.t = 0.
        self.timers = []
        self.updates = []
    def monotonic(self):
        return self.t
    def pause(self, wake):
        pauses.append(wake - self.t)
        return wake
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return len(self.timers)
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))

class FakeMcu:
    def estimated_print_time(self, t):
        return t * 2.0 + 1.0

class FakeI2C:
    def __init__(self):
        self.script = []
        self.write_fail_at = None
        self.mcu = FakeMcu()
    def get_mcu(self):
        return self.mcu
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        if self.write_fail_at is not None and len(writes) == self.write_fail_at:
            raise gcode_mod.CommandError('i2c write failed')
        writes.append(list(data))
    def i2c_read(self, write, read_len, minclock=0, reqclock=0, retry=True):
        reads.append((list(write), read_len, retry))
        act = self.script.pop(0)
        if act[0] == 'ok':
            return {'response': bytes(act[1])}
        if act[0] == 'cmd':
            raise gcode_mod.CommandError(act[1])
        if act[0] == 'mcu':
            raise error(act[1])
        return {}

class FakeHeaters:
    def __init__(self):
        self.factories = []
    def add_sensor_factory(self, sensor_type, factory):
        self.factories.append((sensor_type, factory))

class FakePrinter:
    command_error = gcode_mod.CommandError
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is not configfile.sentinel:
            return default
        raise configfile.error("Unknown config object '%s'" % (name,))
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def invoke_shutdown(self, msg, details={}):
        shutdowns.append(msg)

p = None
fi = None
o = None

def wrapper_for(text, section):
    cfg = RawConfigParser()
    cfg.optionxform = str.lower
    cfg.read_string(text)
    return configfile.ConfigWrapper(p, cfg, {}, section)

def build(text, section):
    global p, fi, o
    reset_rec()
    del i2c_args[:]
    p = FakePrinter()
    fi = FakeI2C()
    wrapper = wrapper_for(text, section)
    orig = sht3x.bus.MCU_I2C_from_config
    def fake_from_config(config, default_addr=None, default_speed=100000, **kw):
        i2c_args.append((default_addr, default_speed))
        return fi
    sht3x.bus.MCU_I2C_from_config = fake_from_config
    try:
        o = sht3x.SHT3X(wrapper)
    finally:
        sht3x.bus.MCU_I2C_from_config = orig
    o.setup_callback(lambda print_time, temp: cbs.append((print_time, temp)))

def ctor_snap():
    return (o.name, o.report_time, o.deviceId, o.temp, o.humidity, o.min_temp,
            o.max_temp, float(o.get_report_time_delta()), o.get_status(0.),
            list(p.objects), list(p.handlers), list(i2c_args))

def snap(ret):
    return (ret, o.temp, o.humidity, o.get_status(0.), list(cbs),
            list(shutdowns), list(logs), list(reads), list(writes))

def try_call(f, *args):
    try:
        f(*args)
    except Exception as e:
        return ('exc', type(e).__name__, str(e))
    return ('ok', None)

def conn_snap():
    return (try_call(o.handle_connect), list(logs), list(reads), list(writes))

def mk(rt, rh, bt=False, bh=False):
    c = sht3x.SHT3X._crc8
    ct = c(None, rt) ^ (1 if bt else 0)
    ch = c(None, rh) ^ (1 if bh else 0)
    return [rt >> 8, rt & 255, ct, rh >> 8, rh & 255, ch]

def mkstatus(v, bad=False):
    return [v >> 8, v & 255, sht3x.SHT3X._crc8(None, v) ^ (1 if bad else 0)]
"#;

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("sht3x python fakes");
            Some(py)
        }
        None => {
            eprintln!("SKIP: $KLIPPY_PYTHON is unset; pyoracle unavailable");
            None
        }
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.sht3x".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn py_float(f: f64) -> String {
    if f.is_nan() {
        return "float('nan')".to_string();
    }
    if f.is_infinite() {
        return if f > 0.0 { "float('inf')".to_string() } else { "float('-inf')".to_string() };
    }
    let s = format!("{f:?}");
    if s.contains('.') || s.contains('e') || s.contains('E') {
        s
    } else {
        format!("{s}.0")
    }
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn float_of(value: &Value) -> f64 {
    match value {
        Value::Float(f) => f.to_f64().expect("float value"),
        other => panic!("expected a float, got {}", other.render()),
    }
}

fn int_list(bytes: &[u8]) -> Value {
    Value::List(bytes.iter().map(|b| Value::int(i64::from(*b))).collect())
}

fn int_vec(value: &Value) -> Vec<u8> {
    match value {
        Value::List(items) => items
            .iter()
            .map(|v| match v {
                Value::Int(i) => u8::try_from(*i).expect("byte"),
                other => panic!("expected an int, got {}", other.render()),
            })
            .collect(),
        other => panic!("expected a list, got {}", other.render()),
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::Str(s) => Value::str(s),
        PyValue::Float(f) => Value::float(*f),
        PyValue::None => Value::Null,
        PyValue::Dict(map) => Value::Dict(
            map.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect(),
        ),
        other => panic!("unexpected status value {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Rust-side log capture (lines kept per test thread; the sink is process wide).
// ---------------------------------------------------------------------------

type LogRecords = Vec<(String, String, String)>;

thread_local! {
    static LOGS: std::cell::RefCell<LogRecords> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        let installed = klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                    rec.exc_text.unwrap_or("").to_string(),
                ));
            });
        }));
        assert!(installed, "another log sink was installed first");
    });
}

fn serial() {
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn take_logs() -> Value {
    let records = LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()));
    Value::List(
        records
            .iter()
            .map(|(level, msg, exc)| {
                Value::Tuple(vec![Value::str(level), Value::str(msg), Value::str(exc)])
            })
            .collect(),
    )
}


// ---------------------------------------------------------------------------
// Rust-side fakes of the contract traits.
// ---------------------------------------------------------------------------

struct FakeHeaters {
    factories: kos::sync::Mutex<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.lock().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used by sht3x")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("get_all_heaters is not used by sht3x")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater is not used by sht3x")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by sht3x")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used by sht3x")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used by sht3x")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used by sht3x")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used by sht3x")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct FakePrinter {
    reactor: Reactor,
    objects: kos::sync::Mutex<Vec<(String, ObjectRef)>>,
    handlers: kos::sync::Mutex<Vec<(String, EventCallback)>>,
    shutdowns: kos::sync::Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            objects: kos::sync::Mutex::new(Vec::new()),
            handlers: kos::sync::Mutex::new(Vec::new()),
            shutdowns: kos::sync::Mutex::new(Vec::new()),
        })
    }

    fn object_names(&self) -> Value {
        Value::List(self.objects.lock().iter().map(|(n, _)| Value::str(n)).collect())
    }

    fn handler_names(&self) -> Value {
        let mut names: Vec<String> = Vec::new();
        for (event, _) in self.handlers.lock().iter() {
            if !names.contains(event) {
                names.push(event.clone());
            }
        }
        Value::List(names.iter().map(|n| Value::str(n)).collect())
    }

    fn shutdown_messages(&self) -> Value {
        Value::List(self.shutdowns.lock().iter().map(|m| Value::str(m)).collect())
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
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
        self.objects.lock().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some((_, obj)) = self.objects.lock().iter().find(|(n, _)| n == name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::Config(format!("Unknown config object '{name}'")))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by sht3x")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by sht3x")
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        // Handlers run outside the lock so they can use the printer.
        let mut handlers = std::mem::take(&mut *self.handlers.lock());
        let mut returns = Vec::new();
        let mut outcome = Ok(());
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                match cb(params) {
                    Ok(v) => returns.push(v),
                    Err(e) => {
                        outcome = Err(e);
                        break;
                    }
                }
            }
        }
        let mut slot = self.handlers.lock();
        handlers.append(&mut slot);
        *slot = handlers;
        outcome.map(|()| returns)
    }
    fn request_exit(&self, _result: &str) {}
}

/// `ConfigWrapper` for the two options the module reads, with the exact
/// `configfile.py:29-60` texts.
struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn config(printer: Printer, section: &str, options: &[(&str, &str)]) -> Config {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            options: options.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        })
    }

    fn required(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{}' in section '{}' must be specified",
            option, self.section
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Unable to parse option '{}' in section '{}'",
            option, self.section
        ))
    }
}

fn unused<T>(what: &str) -> KResult<T> {
    Err(KlippyError::Config(format!("FakeConfig.{what} is not used by sht3x")))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.options.get(option) else {
            return match opts.default {
                None => Err(self.required(option)),
                Some(d) => Ok(d),
            };
        };
        let v = raw.trim().parse::<i64>().map_err(|_| self.unparsable(option))?;
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
        panic!("FakeConfig.getsection is not used by sht3x")
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

/// MCU whose `estimated_print_time(t)` is `t * 2.0 + 1.0` and records `t`.
struct FakeMcu {
    printer: Printer,
    estimates: kos::sync::Mutex<Vec<f64>>,
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
        panic!("setup_pin is not used by sht3x")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by sht3x")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by sht3x")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by sht3x")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by sht3x")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by sht3x")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by sht3x")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by sht3x")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by sht3x")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by sht3x")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by sht3x")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by sht3x")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by sht3x")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by sht3x")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by sht3x")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("get_enumerations is not used by sht3x")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        panic!("get_constants is not used by sht3x")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by sht3x")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by sht3x")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by sht3x")
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.estimates.lock().push(eventtime);
        eventtime * 2.0 + 1.0
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by sht3x")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by sht3x")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by sht3x")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by sht3x")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by sht3x")
    }
}

/// One scripted `i2c_read` outcome.
#[derive(Clone)]
enum Act {
    /// `{'response': bytes}`.
    Ok(Vec<u8>),
    /// Raises the printer's `command_error`.
    Cmd(&'static str),
    /// Raises an exception of a different class (`mcu.error`'s role).
    Mcu(&'static str),
    /// Returns a dict without `'response'`.
    NoKey,
}

fn py_script(acts: &[Act]) -> String {
    let items: Vec<String> = acts
        .iter()
        .map(|a| match a {
            Act::Ok(bytes) => format!("('ok', {bytes:?})"),
            Act::Cmd(m) => format!("('cmd', {m:?})"),
            Act::Mcu(m) => format!("('mcu', {m:?})"),
            Act::NoKey => "('nokey',)".to_string(),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

#[derive(Default)]
struct FakeI2cState {
    script: VecDeque<Act>,
    write_fail_at: Option<usize>,
    writes: Vec<Vec<u8>>,
    reads: Vec<(Vec<u8>, i64, bool)>,
}

struct FakeI2c {
    mcu: Mcu,
    state: kos::sync::Mutex<FakeI2cState>,
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_i2c_address(&self) -> i64 {
        SHT3X_I2C_ADDR
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn build_config(&self) -> KResult<()> {
        Ok(())
    }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> {
        panic!("i2c_write_noack is not used by sht3x")
    }
    fn i2c_write(&self, data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        let mut st = self.state.lock();
        if st.write_fail_at == Some(st.writes.len()) {
            return Err(KlippyError::Command("i2c write failed".to_string()));
        }
        st.writes.push(data.to_vec());
        Ok(())
    }
    fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        let act = {
            let mut st = self.state.lock();
            st.reads.push((write.to_vec(), read_len, opts.retry));
            st.script.pop_front().expect("scripted i2c_read outcome")
        };
        match act {
            Act::Ok(bytes) => {
                let mut map = IndexMap::new();
                map.insert(PyValue::Str("response".to_string()), PyValue::Bytes(bytes));
                Ok(PyValue::Dict(map))
            }
            Act::Cmd(m) => Err(KlippyError::Command(m.to_string())),
            Act::Mcu(m) => Err(KlippyError::Mcu(m.to_string())),
            Act::NoKey => Ok(PyValue::Dict(IndexMap::new())),
        }
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("i2c_transfer is not used by sht3x")
    }
}

// ---------------------------------------------------------------------------
// A Rust sensor with its fakes, and the matching Python object.
// ---------------------------------------------------------------------------

struct Pair {
    printer: Arc<FakePrinter>,
    i2c: Arc<FakeI2c>,
    mcu: Arc<FakeMcu>,
    sensor: Arc<Sht3x>,
    calls: Arc<kos::sync::Mutex<Vec<(f64, f64)>>>,
}

const SECTION: &str = "temperature_sensor sx";

fn config_text(section: &str, options: &[(&str, &str)]) -> String {
    let mut text = format!("[{section}]\n");
    for (k, v) in options {
        text.push_str(&format!("{k}: {v}\n"));
    }
    text
}

fn build_rust(section: &str, options: &[(&str, &str)]) -> KResult<Pair> {
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let mcu = Arc::new(FakeMcu {
        printer: printer_api.clone(),
        estimates: kos::sync::Mutex::new(Vec::new()),
    });
    let i2c = Arc::new(FakeI2c {
        mcu: mcu.clone() as Mcu,
        state: kos::sync::Mutex::new(FakeI2cState::default()),
    });
    let config = FakeConfig::config(printer_api, section, options);
    let sensor = Sht3x::from_config(&config, i2c.clone())?;
    let calls = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let sink = calls.clone();
    sensor.setup_callback(Box::new(move |print_time, temp| {
        sink.lock().push((print_time, temp));
    }));
    Ok(Pair { printer, i2c, mcu, sensor, calls })
}

impl Pair {
    fn reset(&self) {
        self.printer.shutdowns.lock().clear();
        self.calls.lock().clear();
        self.mcu.estimates.lock().clear();
        let mut st = self.i2c.state.lock();
        st.writes.clear();
        st.reads.clear();
        drop(st);
        serial();
    }

    fn set_script(&self, acts: &[Act]) {
        self.i2c.state.lock().script = acts.iter().cloned().collect();
    }

    fn reads_value(&self) -> Value {
        Value::List(
            self.i2c
                .state
                .lock()
                .reads
                .iter()
                .map(|(w, n, retry)| {
                    Value::Tuple(vec![int_list(w), Value::int(*n), Value::Bool(*retry)])
                })
                .collect(),
        )
    }

    fn writes_value(&self) -> Value {
        Value::List(self.i2c.state.lock().writes.iter().map(|w| int_list(w)).collect())
    }

    fn calls_value(&self) -> Value {
        Value::List(
            self.calls
                .lock()
                .iter()
                .map(|(pt, t)| Value::Tuple(vec![Value::float(*pt), Value::float(*t)]))
                .collect(),
        )
    }

    fn status_value(&self) -> Value {
        use klippy_api::PrinterObject;
        pyvalue_to_value(&self.sensor.get_status(0.).unwrap().expect("get_status"))
    }

    /// Mirrors the Python `snap(ret)`.
    fn snap(&self, ret: Value) -> Value {
        Value::Tuple(vec![
            ret,
            Value::float(self.sensor.temp()),
            Value::float(self.sensor.humidity()),
            self.status_value(),
            self.calls_value(),
            self.printer.shutdown_messages(),
            take_logs(),
            self.reads_value(),
            self.writes_value(),
        ])
    }
}

/// Builds the Python object; both sides take the same config text.
fn build_py(py: &mut pyoracle::Oracle, section: &str, options: &[(&str, &str)]) -> pyoracle::OracleResult {
    py.exec(&format!("build({:?}, {:?})", config_text(section, options), section))
}

fn new_pair(py: &mut pyoracle::Oracle, options: &[(&str, &str)], min: f64, max: f64) -> Pair {
    let options: Vec<(&str, &str)> = [("sensor_type", "SHT3X")]
        .iter()
        .copied()
        .chain(options.iter().copied())
        .collect();
    build_py(py, SECTION, &options).expect("python build");
    let pair = build_rust(SECTION, &options).expect("rust build");
    py.exec(&format!("o.setup_minmax({}, {})", py_float(min), py_float(max)))
        .expect("python setup_minmax");
    pair.sensor.setup_minmax(min, max).expect("rust setup_minmax");
    pair
}

fn mk(py: &mut pyoracle::Oracle, rt: u32, rh: u32, bad_t: bool, bad_h: bool) -> Vec<u8> {
    let value = py
        .eval(&format!("mk({rt}, {rh}, {}, {})", py_bool(bad_t), py_bool(bad_h)))
        .expect("python mk");
    int_vec(&value)
}

fn mkstatus(py: &mut pyoracle::Oracle, v: u32, bad: bool) -> Vec<u8> {
    let value = py.eval(&format!("mkstatus({v}, {})", py_bool(bad))).expect("python mkstatus");
    int_vec(&value)
}

/// Runs one `_sample_sht3x` on both sides with the same scripted reads and
/// compares the full snapshot and the time spent in reactor pauses.
fn sample_step(py: &mut pyoracle::Oracle, pair: &Pair, label: &str, script: &[Act]) {
    pair.reset();
    pair.set_script(script);
    py.exec(&format!("reset_rec(); fi.script = {}", py_script(script))).expect("python script");
    let start = kos::time::monotonic();
    let ret = pair.sensor.sample_sht3x(0.);
    let elapsed = kos::time::monotonic() - start;
    // The Python clock stands still during the call; hand it the time the
    // Rust call measured so `print_time` and the return value derive from it.
    let measured = pair.mcu.estimates.lock().last().copied().unwrap_or(0.);
    py.exec(&format!("p.reactor.t = {}", py_float(measured))).expect("python clock");
    let expected = py.eval("snap(o._sample_sht3x(0.))");
    let actual = pair.snap(Value::float(ret));
    check(label, expected, actual);
    let slept = float_of(&py.eval("sum(pauses, 0.)").expect("python pauses"));
    assert!(
        elapsed >= slept - 0.05 && elapsed <= slept + 2.0,
        "{label}: slept {elapsed}s, reference pauses {slept}s"
    );
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_entry_points_and_command_table() {
    serial();
    let Some(mut py) = oracle() else { return };
    let factory = sht3x::module_factory();
    check(
        "entry points",
        py.eval(
            "(hasattr(sht3x, 'load_config'), hasattr(sht3x, 'load_config_prefix'), \
             hasattr(sht3x, 'add_printer_objects'), hasattr(sht3x, 'add_early_printer_objects'))",
        ),
        Value::Tuple(vec![
            Value::Bool(factory.load_config.is_some()),
            Value::Bool(factory.load_config_prefix.is_some()),
            Value::Bool(factory.add_printer_objects.is_some()),
            Value::Bool(factory.add_early_printer_objects.is_some()),
        ]),
    );
    check("i2c address", py.eval("sht3x.SHT3X_I2C_ADDR"), Value::int(SHT3X_I2C_ADDR));
    let table: [&[u8; 2]; 16] = [
        &cmd::MEASURE_STRETCH_ENABLED_HIGH_REP,
        &cmd::MEASURE_STRETCH_ENABLED_MED_REP,
        &cmd::MEASURE_STRETCH_ENABLED_LOW_REP,
        &cmd::MEASURE_STRETCH_DISABLED_HIGH_REP,
        &cmd::MEASURE_STRETCH_DISABLED_MED_REP,
        &cmd::MEASURE_STRETCH_DISABLED_LOW_REP,
        &cmd::PERIODIC_2HZ_HIGH_REP,
        &cmd::PERIODIC_2HZ_MED_REP,
        &cmd::PERIODIC_2HZ_LOW_REP,
        &cmd::OTHER_STATUS_READ,
        &cmd::OTHER_STATUS_CLEAN,
        &cmd::OTHER_SOFTRESET,
        &cmd::OTHER_HEATER_ENABLE,
        &cmd::OTHER_HEATER_DISABLE,
        &cmd::OTHER_FETCH,
        &cmd::OTHER_BREAK,
    ];
    check(
        "command table",
        py.eval(
            "(lambda c: [c['MEASURE']['STRETCH_ENABLED']['HIGH_REP'], c['MEASURE']['STRETCH_ENABLED']['MED_REP'], \
             c['MEASURE']['STRETCH_ENABLED']['LOW_REP'], c['MEASURE']['STRETCH_DISABLED']['HIGH_REP'], \
             c['MEASURE']['STRETCH_DISABLED']['MED_REP'], c['MEASURE']['STRETCH_DISABLED']['LOW_REP'], \
             c['PERIODIC']['2HZ']['HIGH_REP'], c['PERIODIC']['2HZ']['MED_REP'], c['PERIODIC']['2HZ']['LOW_REP'], \
             c['OTHER']['STATUS']['READ'], c['OTHER']['STATUS']['CLEAN'], c['OTHER']['SOFTRESET'], \
             c['OTHER']['HEATER']['ENABLE'], c['OTHER']['HEATER']['DISABLE'], c['OTHER']['FETCH'], \
             c['OTHER']['BREAK']])(sht3x.SHT3X_CMD)",
        ),
        Value::List(table.iter().map(|c| int_list(*c)).collect()),
    );
}

#[test]
fn crc8_matches_python_over_the_16_bit_range() {
    serial();
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("[sht3x.SHT3X._crc8(None, v) for v in range(65536)]");
    let actual = Value::List((0..65536i64).map(|v| Value::int(Sht3x::crc8(v))).collect());
    check("crc8 0..65535", expected, actual);
}

#[test]
fn constructor_and_config_match_python() {
    serial();
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &[(&str, &str)])] = &[
        (SECTION, &[("sensor_type", "SHT3X")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "3")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "1")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "120")]),
        ("temperature_sensor  wide  name", &[("sensor_type", "SHT3X")]),
        ("temperature_fan fan1", &[("sensor_type", "other")]),
        (SECTION, &[]),
        (SECTION, &[("sht3x_report_time", "2")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "0")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "-2")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "abc")]),
        (SECTION, &[("sensor_type", "SHT3X"), ("sht3x_report_time", "2.5")]),
        // The report time is validated before `sensor_type` is read.
        (SECTION, &[("sht3x_report_time", "0")]),
    ];
    for (i, (section, options)) in cases.iter().enumerate() {
        let label = format!("ctor[{i}] {section} {options:?}");
        let expected = build_py(&mut py, section, options);
        let actual = build_rust(section, options);
        match (expected, actual) {
            (Ok(_), Ok(pair)) => {
                let expected = py.eval("ctor_snap()");
                let sensor = &pair.sensor;
                let actual = Value::Tuple(vec![
                    Value::str(sensor.name()),
                    Value::int(sensor.report_time()),
                    Value::str(sensor.device_id()),
                    Value::float(sensor.temp()),
                    Value::float(sensor.humidity()),
                    Value::float(sensor.min_temp()),
                    Value::float(sensor.max_temp()),
                    Value::float(sensor.get_report_time_delta()),
                    pair.status_value(),
                    pair.printer.object_names(),
                    pair.printer.handler_names(),
                    // `Sht3x::new` asks `bus.MCU_I2C_from_config` for these.
                    Value::List(vec![Value::Tuple(vec![
                        Value::int(SHT3X_I2C_ADDR),
                        Value::int(100000),
                    ])]),
                ]);
                check(&label, expected, actual);
                // The registered object is the sensor itself and exposes the
                // sensor interface to `heaters`.
                let (name, obj) = pair.printer.objects.lock()[0].clone();
                let same = downcast_arc::<Sht3x>(&obj).expect("sht3x object");
                assert!(Arc::ptr_eq(&same, &pair.sensor), "{label}: registered object");
                assert!(
                    interface::<dyn TemperatureSensorApi>(&obj).is_some(),
                    "{label}: sensor interface"
                );
                assert_eq!(name, format!("sht3x {}", pair.sensor.name()), "{label}: object name");
            }
            (Err(py_exc), Err(e)) => {
                assert_eq!(e.class_name(), py_exc.class_name, "{label}: exception class");
                assert_eq!(e.message(), py_exc.message, "{label}: exception message");
            }
            (Ok(_), Err(e)) => {
                panic!("{label}: port raised {}: {}", e.class_name(), e.message())
            }
            (Err(py_exc), Ok(_)) => {
                panic!("{label}: oracle raised {}: {}", py_exc.class_name, py_exc.message)
            }
        }
    }
}

#[test]
fn sample_good_readings_match_python() {
    serial();
    let Some(mut py) = oracle() else { return };
    let raws: &[(u32, u32)] = &[
        (0, 0),
        (65535, 65535),
        (26214, 32768),
        (12345, 54321),
        (1, 1),
        (40000, 100),
        (65534, 2),
        (30000, 30000),
        (21845, 43690),
        (31000, 17000),
        (45000, 60000),
        (255, 256),
    ];
    for &report_time in &["1", "5"] {
        for (i, &(rt, rh)) in raws.iter().enumerate() {
            let pair = new_pair(&mut py, &[("sht3x_report_time", report_time)], -50., 150.);
            let good = mk(&mut py, rt, rh, false, false);
            sample_step(
                &mut py,
                &pair,
                &format!("good[{i}] rt={rt} rh={rh} report={report_time}"),
                &[Act::Ok(good)],
            );
        }
    }
}

#[test]
fn sample_checksum_errors_keep_previous_values() {
    serial();
    let Some(mut py) = oracle() else { return };
    let pair = new_pair(&mut py, &[], -50., 150.);
    let steps: &[(u32, u32, bool, bool)] = &[
        (30000, 30000, false, false),
        (40000, 50000, true, false),
        (41000, 51000, false, true),
        (42000, 52000, true, true),
        (43000, 53000, false, false),
        (0, 0, true, true),
    ];
    for (i, &(rt, rh, bt, bh)) in steps.iter().enumerate() {
        let bytes = mk(&mut py, rt, rh, bt, bh);
        sample_step(&mut py, &pair, &format!("crc step[{i}]"), &[Act::Ok(bytes)]);
    }
}

#[test]
fn sample_outside_range_invokes_shutdown_and_still_reports() {
    serial();
    let Some(mut py) = oracle() else { return };
    let ranges: &[(f64, f64)] = &[(20., 25.), (0., 50.), (-10.5, 12.25), (30., 10.), (22.5, 22.5)];
    let raws: &[(u32, u32)] = &[(0, 100), (65535, 200), (30000, 30000), (36000, 40000), (28000, 1)];
    for (ri, &(min, max)) in ranges.iter().enumerate() {
        let pair = new_pair(&mut py, &[], min, max);
        for (i, &(rt, rh)) in raws.iter().enumerate() {
            let bytes = mk(&mut py, rt, rh, false, false);
            sample_step(
                &mut py,
                &pair,
                &format!("range[{ri}] {min}:{max} raw[{i}]"),
                &[Act::Ok(bytes)],
            );
        }
    }
}

#[test]
fn sample_unset_limits_shut_down_like_python() {
    serial();
    let Some(mut py) = oracle() else { return };
    // `setup_minmax` never called: 0.0 on both sides.
    py.exec(&format!("build({:?}, {:?})", config_text(SECTION, &[("sensor_type", "SHT3X")]), SECTION))
        .expect("python build");
    let pair = build_rust(SECTION, &[("sensor_type", "SHT3X")]).expect("rust build");
    for (i, &(rt, rh)) in [(10000u32, 20000u32), (30000, 40000)].iter().enumerate() {
        let bytes = mk(&mut py, rt, rh, false, false);
        sample_step(&mut py, &pair, &format!("no limits[{i}]"), &[Act::Ok(bytes)]);
    }
}

#[test]
fn sample_retries_and_failures_match_python() {
    serial();
    let Some(mut py) = oracle() else { return };
    let good = mk(&mut py, 30000, 30000, false, false);
    let short3 = vec![1u8, 2, 3];
    let short4 = mk(&mut py, 20000, 20000, false, false)[..4].to_vec();
    let scenarios: Vec<(&str, Vec<Act>)> = vec![
        ("one retry", vec![Act::Cmd("nack 1"), Act::Ok(good.clone())]),
        (
            "four retries",
            vec![
                Act::Cmd("nack 1"),
                Act::Cmd("nack 2"),
                Act::Cmd("nack 3"),
                Act::Cmd("nack 4"),
                Act::Ok(good.clone()),
            ],
        ),
        (
            "retries exhausted",
            vec![
                Act::Cmd("nack 1"),
                Act::Cmd("nack 2"),
                Act::Cmd("nack 3"),
                Act::Cmd("nack 4"),
                Act::Cmd("nack 5"),
            ],
        ),
        ("other error class is not retried", vec![Act::Mcu("bus lost"), Act::Ok(good.clone())]),
        ("retry then other error", vec![Act::Cmd("nack 1"), Act::Mcu("bus lost")]),
        ("missing response key", vec![Act::NoKey]),
        ("response of 3 bytes", vec![Act::Ok(short3)]),
        ("response of 4 bytes", vec![Act::Ok(short4)]),
        ("empty response", vec![Act::Ok(Vec::new())]),
    ];
    for (name, script) in scenarios {
        let pair = new_pair(&mut py, &[], -50., 150.);
        // A good sample first so the reset to 0.0 after an error is visible.
        sample_step(&mut py, &pair, &format!("{name}: prime"), &[Act::Ok(good.clone())]);
        sample_step(&mut py, &pair, name, &script);
        // The timer stops on an error, but a direct call samples again.
        sample_step(&mut py, &pair, &format!("{name}: after"), &[Act::Ok(good.clone())]);
    }
}

#[test]
fn connect_sequence_matches_python() {
    serial();
    let Some(mut py) = oracle() else { return };
    let status_ok = mkstatus(&mut py, 0x8010, false);
    let status_bad = mkstatus(&mut py, 0x8010, true);
    let scenarios: Vec<(&str, Vec<Act>, Option<usize>)> = vec![
        ("status ok", vec![Act::Ok(status_ok.clone())], None),
        ("status checksum error", vec![Act::Ok(status_bad)], None),
        ("status of 2 bytes", vec![Act::Ok(vec![0x80, 0x10])], None),
        ("status without response", vec![Act::NoKey], None),
        ("status read command error", vec![Act::Cmd("nack status")], None),
        ("status read other error", vec![Act::Mcu("bus lost")], None),
        ("break write fails", vec![Act::Ok(status_ok.clone())], Some(0)),
        ("soft reset write fails", vec![Act::Ok(status_ok.clone())], Some(1)),
        ("periodic write fails", vec![Act::Ok(status_ok.clone())], Some(2)),
    ];
    for (name, script, fail_at) in scenarios {
        let pair = new_pair(&mut py, &[], -50., 150.);
        pair.reset();
        pair.set_script(&script);
        pair.i2c.state.lock().write_fail_at = fail_at;
        let fail_py = match fail_at {
            Some(n) => n.to_string(),
            None => "None".to_string(),
        };
        py.exec(&format!(
            "reset_rec(); fi.script = {}; fi.write_fail_at = {fail_py}",
            py_script(&script)
        ))
        .expect("python script");
        let start = kos::time::monotonic();
        let result = pair.printer.send_event("klippy:connect", &[]);
        let elapsed = kos::time::monotonic() - start;
        let outcome = match result {
            Ok(_) => Value::Tuple(vec![Value::str("ok"), Value::Null]),
            Err(e) => Value::Tuple(vec![
                Value::str("exc"),
                Value::str(e.class_name()),
                Value::str(e.message()),
            ]),
        };
        let actual = Value::Tuple(vec![
            outcome,
            take_logs(),
            pair.reads_value(),
            pair.writes_value(),
        ]);
        check(name, py.eval("conn_snap()"), actual);
        let slept = float_of(&py.eval("sum(pauses, 0.)").expect("python pauses"));
        assert!(
            elapsed >= slept - 0.05 && elapsed <= slept + 2.0,
            "{name}: slept {elapsed}s, reference pauses {slept}s"
        );
    }
}

#[test]
fn sensor_callback_is_replaceable_and_receives_each_sample() {
    serial();
    let Some(mut py) = oracle() else { return };
    let pair = new_pair(&mut py, &[], -50., 150.);
    let good = mk(&mut py, 30000, 30000, false, false);
    sample_step(&mut py, &pair, "first callback", &[Act::Ok(good.clone())]);
    // A second `setup_callback` replaces the first on both sides.
    py.exec("cbs2 = []; o.setup_callback(lambda pt, t: cbs2.append((pt, t)))").expect("py");
    let second: Arc<kos::sync::Mutex<Vec<(f64, f64)>>> = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let sink = second.clone();
    pair.sensor.setup_callback(Box::new(move |pt, t| sink.lock().push((pt, t))));
    pair.reset();
    pair.set_script(&[Act::Ok(good.clone())]);
    py.exec(&format!("reset_rec(); fi.script = {}", py_script(&[Act::Ok(good)]))).expect("py");
    pair.sensor.sample_sht3x(0.);
    let measured = pair.mcu.estimates.lock().last().copied().expect("estimate");
    py.exec(&format!("p.reactor.t = {}", py_float(measured))).expect("py clock");
    py.exec("o._sample_sht3x(0.)").expect("py sample");
    let expected = py.eval("([(pt, t) for pt, t in cbs2], list(cbs))");
    let actual = Value::Tuple(vec![
        Value::List(
            second
                .lock()
                .iter()
                .map(|(pt, t)| Value::Tuple(vec![Value::float(*pt), Value::float(*t)]))
                .collect(),
        ),
        pair.calls_value(),
    ]);
    check("replaced callback", expected, actual);
}

#[test]
fn load_config_registers_the_sensor_factory() {
    serial();
    let Some(mut py) = oracle() else { return };
    let sensor_cfg = [("sensor_type", "SHT3X")];

    // Without a `heaters` object the lookup raises on both sides.
    py.exec(&format!(
        "p = FakePrinter(); fi = FakeI2C(); w = wrapper_for({:?}, {SECTION:?})",
        config_text(SECTION, &sensor_cfg)
    ))
    .expect("python setup");
    let expected = py.eval("try_call(sht3x.load_config, w)");
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let config = FakeConfig::config(printer_api.clone(), SECTION, &sensor_cfg);
    let actual = match sht3x::load_config(&config) {
        Ok(None) => Value::Tuple(vec![Value::str("ok"), Value::Null]),
        Ok(Some(_)) => panic!("load_config returned an object"),
        Err(e) => Value::Tuple(vec![
            Value::str("exc"),
            Value::str(e.class_name()),
            Value::str(e.message()),
        ]),
    };
    check("load_config without heaters", expected, actual);

    // With `heaters`: one `SHT3X` factory is registered and `load_config`
    // returns None.
    py.exec("p.objects['heaters'] = FakeHeaters()").expect("python heaters");
    let expected = py.eval(
        "(sht3x.load_config(w), [t for t, f in p.objects['heaters'].factories], \
         [f is sht3x.SHT3X for t, f in p.objects['heaters'].factories])",
    );
    let heaters = Arc::new(FakeHeaters { factories: kos::sync::Mutex::new(Vec::new()) });
    printer.objects.lock().push(("heaters".to_string(), heaters.clone() as ObjectRef));
    let loaded = sht3x::load_config(&config).expect("rust load_config");
    assert!(loaded.is_none(), "load_config returns None");
    let names: Vec<Value> = heaters.factories.lock().iter().map(|(t, _)| Value::str(t)).collect();
    let actual = Value::Tuple(vec![
        Value::Null,
        Value::List(names.clone()),
        Value::List(names.iter().map(|_| Value::Bool(true)).collect()),
    ]);
    check("load_config registers SHT3X", expected, actual);

    // The registered factory builds `SHT3X(config)`: its first config access
    // is `MCU_I2C_from_config`, which fails the same way on a printer with
    // no `mcu` object.
    let expected = py.eval("try_call(p.objects['heaters'].factories[0][1], w)");
    let factory = heaters.factories.lock()[0].1.clone();
    let actual = match factory(&config) {
        Ok(_) => Value::Tuple(vec![Value::str("ok"), Value::Null]),
        Err(e) => Value::Tuple(vec![
            Value::str("exc"),
            Value::str(e.class_name()),
            Value::str(e.message()),
        ]),
    };
    check("factory without mcu", expected, actual);
}

#[test]
fn module_factory_wires_load_config() {
    let factory = sht3x::module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.sht3x"), "entry points of extras.sht3x");
}
