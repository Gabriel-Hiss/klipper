//! Parity tests for `klippy/extras/htu21d.py`.
//!
//! Every expected value comes from the frozen Python reference through the
//! `pyoracle` crate (set `KLIPPY_PYTHON` to the CPython 3.8.2 binary; the
//! oracle tests skip when it is unset). The oracle builds an `HTU21D` with
//! minimal Python fakes for the printer, config, reactor and I2C bus, and
//! the Rust side is driven through matching fakes of the `klippy_api`
//! traits plus a fake [`extras::bus::McuI2cApi`], with the same inputs.
//!
//! Compared against the reference:
//! - config reads and defaults (`htu21d_hold_master`, `htu21d_resolution`,
//!   `htu21d_report_time`, `sensor_type`), including the invalid-resolution
//!   and missing-option error texts;
//! - `_init_htu21d`: I2C write/read call sequence (including the line-156
//!   bug that passes the register value as `minclock`), log lines, and the
//!   short-response / bad-checksum paths;
//! - `_sample_htu21d`: temperature/humidity conversion for every chip and
//!   resolution, CRC error paths, humidity clamping, the (no-op)
//!   temperature compensation, the out-of-range shutdown message, the
//!   sensor callback arguments, and the exception path (temp/humidity
//!   reset, `NEVER` return);
//! - `_chekCRC8` over a grid of inputs;
//! - `load_config` registration order.
use extras::bus::McuI2cApi;
use extras::heaters::PrinterHeatersApi;
use extras::htu21d::{
    htu21d_devices, id_map, load_config, HTU21D, HTU21D_COMMANDS_FIRMWARE_READ,
    HTU21D_COMMANDS_HUMID, HTU21D_COMMANDS_HUMID_NH, HTU21D_COMMANDS_READ,
    HTU21D_COMMANDS_RESET, HTU21D_COMMANDS_SERIAL, HTU21D_COMMANDS_TEMP,
    HTU21D_COMMANDS_TEMP_NH, HTU21D_COMMANDS_WRITE, HTU21D_CRC8_POLYNOMINAL,
    HTU21D_I2C_ADDR, HTU21D_RESOLUTION_MASK, HTU21D_RESOLUTIONS,
    HTU21D_TEMP_COEFFICIENT,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log::{self, Record};
use klippy_api::mcu::{CommandQueue, Mcu, QuerySendOpts, SendOpts};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NEVER};
use recording::value::Value;
use std::sync::Arc;


// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        log::set_sink(Box::new(|rec: &Record<'_>| {
            let level = rec.level.name().to_string();
            LOGS.with(|logs| logs.borrow_mut().push((level, rec.message.to_string())));
        }));
    });
}

fn lock_logs() {
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn take_logs() -> Vec<(String, String)> {
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

/// The captured Rust log as the `py_logs()` list of `(levelname, message)`
/// tuples, so level and text compare against the reference together.
fn log_records_value() -> Value {
    Value::List(
        take_logs()
            .into_iter()
            .map(|(level, message)| {
                Value::Tuple(vec![Value::str(&level), Value::str(&message)])
            })
            .collect(),
    )
}

fn log_messages(records: &Value) -> Vec<String> {
    match records {
        Value::List(items) => items
            .iter()
            .map(|entry| match entry {
                Value::Tuple(p) => match &p[1] {
                    Value::Str(m) => m.clone(),
                    other => panic!("bad log message {other:?}"),
                },
                other => panic!("bad log entry {other:?}"),
            })
            .collect(),
        other => panic!("bad log records {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// Fakes.
// ---------------------------------------------------------------------------

struct LogFakePrinter {
    reactor: Reactor,
    objects: GCell<IndexMap<String, ObjectRef>>,
    shutdowns: GCell<Vec<String>>,
    handlers: GCell<Vec<(String, klippy_api::printer::EventCallback)>>,
}

impl LogFakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(LogFakePrinter {
            reactor: Reactor::new(false),
            objects: GCell::new(IndexMap::new()),
            shutdowns: GCell::new(Vec::new()),
            handlers: GCell::new(Vec::new()),
        })
    }
}

impl PrinterApi for LogFakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
        let mut objs = self.objects.borrow_mut();
        if objs.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(o) = self.objects.borrow().get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let objs = self.objects.borrow();
            match module {
                None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
                Some(m) => {
                    let prefix = format!("{m} ");
                    objs.iter()
                        .filter(|(k, _)| k.starts_with(&prefix) || *k == m)
                        .map(|(k, v)| (k.clone(), v.clone()))
                        .collect()
                }
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
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
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.handlers
            .borrow_mut()
            .push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        let mut handlers = self.handlers.borrow_mut();
        for (ev, cb) in handlers.iter_mut() {
            if ev == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}



struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, options: &[(&str, &str)]) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            options: options
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        })
    }
    fn raw(&self, option: &str) -> Option<&str> {
        self.options.get(option).map(|s| s.as_str())
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
        match self.raw(option) {
            Some(v) => Ok(Some(v.to_string())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            Some(v) => {
                let parsed: i64 = v.trim().parse().map_err(|_| {
                    KlippyError::Config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.name
                    ))
                })?;
                if let Some(minval) = opts.minval {
                    if parsed < minval {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.name
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if parsed > maxval {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.name
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            Some(v) => {
                let b = match v.trim().to_ascii_lowercase().as_str() {
                    "true" | "yes" | "on" | "1" => true,
                    "false" | "no" | "off" | "0" => false,
                    _ => {
                        return Err(KlippyError::Config(format!(
                            "Unable to parse option '{option}' in section '{}'",
                            self.name
                        )))
                    }
                };
                Ok(Some(b))
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::Config("unused".to_string()))
    }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::new(self.printer.clone(), section, &[])
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

/// Fake MCU: only `estimated_print_time` / `get_name` are used by htu21d.
struct FakeMcu {
    print_time: f64,
}

impl klippy_api::mcu::McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("unused")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: klippy_api::mcu::PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
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
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        klippy_api::mcu::CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("unused")
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
        Err(KlippyError::Mcu("unused".to_string()))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        self.print_time
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Mcu("unused".to_string()))
    }
}

#[derive(Debug, Clone, PartialEq)]
enum I2cCall {
    Write { data: Vec<u8>, minclock: i64, reqclock: i64, retry: bool },
    Read { write: Vec<u8>, read_len: i64, retry: bool },
}

/// Scripted fake I2C bus.
struct FakeI2c {
    mcu: Mcu,
    reads: GCell<Vec<Vec<u8>>>,
    calls: GCell<Vec<I2cCall>>,
    fail_next_read: GCell<Option<KlippyError>>,
    fail_next_write: GCell<Option<KlippyError>>,
}

impl FakeI2c {
    fn new(reads: Vec<Vec<u8>>) -> Arc<Self> {
        Arc::new(FakeI2c {
            mcu: Arc::new(FakeMcu { print_time: 5555.0 }),
            reads: GCell::new(reads),
            calls: GCell::new(Vec::new()),
            fail_next_read: GCell::new(None),
            fail_next_write: GCell::new(None),
        })
    }
    fn calls(&self) -> Vec<I2cCall> {
        self.calls.borrow().clone()
    }
}

impl McuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 { 0 }
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_i2c_address(&self) -> i64 { 0x40 }
    fn get_command_queue(&self) -> CommandQueue { CommandQueue(0) }
    fn build_config(&self) -> KResult<()> { Ok(()) }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> { Ok(()) }
    fn i2c_write(&self, data: &[u8], opts: QuerySendOpts) -> KResult<()> {
        self.calls.borrow_mut().push(I2cCall::Write {
            data: data.to_vec(),
            minclock: opts.minclock,
            reqclock: opts.reqclock,
            retry: opts.retry,
        });
        if let Some(e) = self.fail_next_write.replace(None) {
            return Err(e);
        }
        Ok(())
    }
    fn i2c_read(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        self.calls.borrow_mut().push(I2cCall::Read {
            write: write.to_vec(),
            read_len,
            retry: opts.retry,
        });
        if let Some(e) = self.fail_next_read.replace(None) {
            return Err(e);
        }
        let resp = self.reads.borrow_mut().remove(0);
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("response".to_string()),
            PyValue::List(
                resp.iter()
                    .map(|b| PyValue::Int(PyInt::from(*b as i64)))
                    .collect(),
            ),
        );
        Ok(PyValue::Dict(d))
    }
    fn i2c_transfer(&self, write: &[u8], read_len: i64, opts: QuerySendOpts) -> KResult<PyValue> {
        self.i2c_read(write, read_len, opts)
    }
}

/// Fake heaters registry for `load_config`.
struct FakeHeaters {
    factories: GCell<Vec<String>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(
        &self,
        sensor_type: &str,
        _factory: extras::heaters::SensorFactory,
    ) {
        self.factories.borrow_mut().push(sensor_type.to_string());
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<extras::heaters::Heater>> {
        panic!("unused setup_heater")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<extras::heaters::Heater>> {
        Err(KlippyError::Config("unused lookup_heater".to_string()))
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn extras::heaters::TemperatureSensorApi>> {
        panic!("unused setup_sensor")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn extras::heaters::SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("unused register_sensor")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("unused register_monitor")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("unused turn_off_all_heaters")
    }
    fn set_temperature(&self, _heater: &Arc<extras::heaters::Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("unused set_temperature")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);


fn rig(
    options: &[(&str, &str)],
    reads: Vec<Vec<u8>>,
) -> (Arc<LogFakePrinter>, Arc<FakeConfig>, Arc<FakeI2c>, Arc<HTU21D>) {
    let printer = LogFakePrinter::new();
    let config = FakeConfig::new(printer.clone(), "sensor test1", options);
    let i2c = FakeI2c::new(reads);
    let config_dyn: Config = config.clone();
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c.clone();
    let sensor = HTU21D::from_parts(&config_dyn, i2c_dyn).expect("construct HTU21D");
    (printer, config, i2c, sensor)
}

// ---------------------------------------------------------------------------
// Python oracle setup.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import sys, types, logging, configparser
LOG_CAPTURE = []
class _Capture(logging.Handler):
    def emit(self, record):
        LOG_CAPTURE.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_Capture())
logging.getLogger().setLevel(logging.DEBUG)

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = {}
    def monotonic(self):
        return 1000.0
    def pause(self, waketime):
        return waketime
    def register_timer(self, cb, waketime=None):
        t = object()
        self.timers[id(t)] = (cb, waketime)
        return t
    def update_timer(self, timer, waketime):
        cb, _ = self.timers[id(timer)]
        self.timers[id(timer)] = (cb, waketime)

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return 5555.0
    def get_name(self):
        return 'mcu'

class FakeI2C:
    def __init__(self):
        self.calls = []
        self.reads = []
        self.fail_read = None
        self.fail_write = None
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        self.calls.append(('write', list(data), minclock, reqclock, retry))
        if self.fail_write is not None:
            e, self.fail_write = self.fail_write, None
            raise e
        return None
    def i2c_read(self, write, read_len, retry=True):
        self.calls.append(('read', list(write), read_len, retry))
        if self.fail_read is not None:
            e, self.fail_read = self.fail_read, None
            raise e
        return {'response': list(self.reads.pop(0))}
    def get_mcu(self):
        return FakeMcu()

FAKE_I2C = FakeI2C()
_fake_bus = types.ModuleType('extras.bus')
def _mcu_i2c_from_config(config, default_addr=None, default_speed=100000, async_write_only=False):
    return FAKE_I2C
_fake_bus.MCU_I2C_from_config = _mcu_i2c_from_config
sys.modules['extras.bus'] = _fake_bus

class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.objects = {}
        self.shutdowns = []
        self.handlers = {}
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        if name in self.objects:
            raise configparser.Error(
                "Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)

_sentinel = object()
class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options):
        self.printer = printer
        self.name = name
        self.options = options
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def get(self, option, default=_sentinel):
        if option not in self.options:
            if default is _sentinel:
                raise configparser.Error(
                    "Option '%s' in section '%s' must be specified"
                    % (option, self.name))
            return default
        return self.options[option]
    def getboolean(self, option, default=False):
        if option not in self.options:
            return default
        v = self.options[option].strip().lower()
        if v in ('true', 'yes', 'on', '1'):
            return True
        if v in ('false', 'no', 'off', '0'):
            return False
        raise configparser.Error(
            "Unable to parse option '%s' in section '%s'" % (option, self.name))
    def getint(self, option, default=None, minval=None, maxval=None):
        if option not in self.options:
            return default
        v = int(self.options[option])
        if minval is not None and v < minval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self.name, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self.name, maxval))
        return v

import extras.htu21d as htu21d_mod
"#;

// Helpers to drive the Python side identically to the Rust rig.
const PY_HELPERS: &str = r"
def make_sensor(options):
    printer = FakePrinter()
    config = FakeConfig(printer, 'sensor test1', dict(options))
    FAKE_I2C.calls.clear()
    LOG_CAPTURE.clear()
    sensor = htu21d_mod.HTU21D(config)
    return printer, sensor
def queue_reads(*byte_lists):
    FAKE_I2C.reads = [list(b) for b in byte_lists]
def py_calls():
    return [tuple(c) for c in FAKE_I2C.calls]
def py_logs():
    return list(LOG_CAPTURE)
def py_status(sensor):
    return sensor.get_status(0.)
def crc8(data):
    return htu21d_mod.HTU21D._chekCRC8(None, data)
";

/// The checksum byte the reference computes for a 16-bit reading, used to
/// build the I2C responses so the fixtures come from the reference.
fn py_crc8(py: &mut pyoracle::Oracle, data: i64) -> i64 {
    match py.eval(&format!("crc8({data})")) {
        Ok(Value::Int(v)) => v,
        other => panic!("crc8({data}) -> {other:?}"),
    }
}

fn oracle_start() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(PY_SETUP).expect("oracle setup");
    py.exec(PY_HELPERS).expect("oracle helpers");
    Some(py)
}


/// Encode a Rust `PyValue` dict the way the recorder encodes the Python
/// dict, for `recording::assert_equal`-style comparison of status dicts.
fn pyvalue_to_recording(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::Float(recording::value::FloatVal::new(*f)),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => {
            Value::List(items.iter().map(pyvalue_to_recording).collect())
        }
        PyValue::Tuple(items) => {
            Value::Tuple(items.iter().map(pyvalue_to_recording).collect())
        }
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, val)| (pyvalue_to_recording(k), pyvalue_to_recording(val)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("0x{h:x}"),
        },
    }
}

/// `recording::assert_equal` compares floats by bits, because CPython's
/// `float.hex()` and the port's formatter spell the same value differently
/// (`0x1.4800000000000p+5` vs `0x1.48p+5`). Status dicts need the same rule.
fn values_match(expected: &Value, actual: &Value) -> bool {
    match (expected, actual) {
        (Value::Float(a), Value::Float(b)) => {
            let bits = |f: &recording::value::FloatVal| {
                recording::f64_from_hex(&f.hex).map(f64::to_bits)
            };
            bits(a).is_some() && bits(a) == bits(b)
        }
        (Value::List(a), Value::List(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| values_match(x, y))
        }
        (Value::Tuple(a), Value::Tuple(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| values_match(x, y))
        }
        (Value::Dict(a), Value::Dict(b)) => {
            a.len() == b.len()
                && a.iter().zip(b).all(|((ak, av), (bk, bv))| {
                    values_match(ak, bk) && values_match(av, bv)
                })
        }
        (a, b) => a == b,
    }
}

/// Encode one recorded I2C call the way the Python fake appends it:
/// `('write', data, minclock, reqclock, retry)` / `('read', write, len, retry)`.
fn i2c_call_value(call: &I2cCall) -> Value {
    match call {
        I2cCall::Write { data, minclock, reqclock, retry } => Value::Tuple(vec![
            Value::str("write"),
            Value::List(data.iter().map(|b| Value::int(*b as i64)).collect()),
            Value::int(*minclock),
            Value::int(*reqclock),
            Value::Bool(*retry),
        ]),
        I2cCall::Read { write, read_len, retry } => Value::Tuple(vec![
            Value::str("read"),
            Value::List(write.iter().map(|b| Value::int(*b as i64)).collect()),
            Value::int(*read_len),
            Value::Bool(*retry),
        ]),
    }
}

fn i2c_calls_value(calls: &[I2cCall]) -> Value {
    Value::List(calls.iter().map(i2c_call_value).collect())
}

fn assert_status_equal(label: &str, expected: pyoracle::OracleResult, actual: PyValue) {
    match expected {
        Ok(v) => {
            let a = pyvalue_to_recording(&actual);
            assert!(
                values_match(&v, &a),
                "{label}: status dict mismatch\n  expected: {}\n  actual:   {}",
                v.render(),
                a.render()
            );
        }
        Err(e) => panic!("{label}: oracle raised {e:?}, port returned {actual:?}"),
    }
}


fn assert_i64_equal(label: &str, expected: pyoracle::OracleResult, actual: i64) {
    match expected {
        Ok(Value::Int(v)) => assert_eq!(v, actual, "{label}"),
        Ok(v) => panic!("{label}: oracle returned {v:?}, expected an int"),
        Err(e) => panic!("{label}: oracle raised {e:?}, port returned {actual}"),
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn crc8_matches_python() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let (_p, _c, _i, sensor) = rig(&[("sensor_type", "HTU21D")], vec![]);
    let mut values: Vec<i64> = vec![0, 1, 0xFF, 0x100, 0x7FFF, 0x8000, 0xFFFF, 0x1234, 0xABCD];
    for hi in (0..=255).step_by(17) {
        for lo in (0..=255).step_by(31) {
            values.push(hi * 256 + lo);
        }
    }
    py.exec("_, sensor = make_sensor([('sensor_type', 'HTU21D')])")
        .expect("make python sensor");
    for v in values {
        let expected = py.eval(&format!("sensor._chekCRC8({v})"));
        let actual = sensor.chek_crc8(v);
        assert_i64_equal(&format!("chekCRC8({v})"), expected, actual);
    }
}

#[test]
fn module_constants_match_python() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let expected = py
        .eval(
            "(htu21d_mod.HTU21D_I2C_ADDR, htu21d_mod.HTU21D_COMMANDS, \
             htu21d_mod.HTU21D_RESOLUTION_MASK, htu21d_mod.HTU21D_RESOLUTIONS, \
             htu21d_mod.ID_MAP, htu21d_mod.HTU21D_DEVICES, \
             htu21d_mod.HTU21D_TEMP_COEFFICIENT, htu21d_mod.HTU21D_CRC8_POLYNOMINAL)",
        )
        .expect("python constants");
    let commands = Value::Dict(vec![
        (Value::str("HTU21D_TEMP"), Value::int(HTU21D_COMMANDS_TEMP)),
        (Value::str("HTU21D_HUMID"), Value::int(HTU21D_COMMANDS_HUMID)),
        (Value::str("HTU21D_TEMP_NH"), Value::int(HTU21D_COMMANDS_TEMP_NH)),
        (Value::str("HTU21D_HUMID_NH"), Value::int(HTU21D_COMMANDS_HUMID_NH)),
        (Value::str("WRITE"), Value::int(HTU21D_COMMANDS_WRITE)),
        (Value::str("READ"), Value::int(HTU21D_COMMANDS_READ)),
        (Value::str("RESET"), Value::int(HTU21D_COMMANDS_RESET)),
        (
            Value::str("SERIAL"),
            Value::List(HTU21D_COMMANDS_SERIAL.iter().map(|v| Value::int(*v)).collect()),
        ),
        (
            Value::str("FIRMWARE_READ"),
            Value::List(
                HTU21D_COMMANDS_FIRMWARE_READ.iter().map(|v| Value::int(*v)).collect(),
            ),
        ),
    ]);
    let resolutions = Value::Dict(
        HTU21D_RESOLUTIONS
            .iter()
            .map(|(k, v)| (Value::str(k), Value::int(*v)))
            .collect(),
    );
    let ids = Value::Dict(
        id_map()
            .iter()
            .map(|(k, v)| (Value::int(*k), Value::str(v)))
            .collect(),
    );
    let devices = Value::Dict(
        htu21d_devices()
            .iter()
            .map(|(chip, by_res)| {
                (
                    Value::str(chip),
                    Value::Dict(
                        by_res
                            .iter()
                            .map(|(res, times)| {
                                (
                                    Value::str(res),
                                    Value::List(
                                        times.iter().map(|t| Value::float(*t)).collect(),
                                    ),
                                )
                            })
                            .collect(),
                    ),
                )
            })
            .collect(),
    );
    let actual = Value::Tuple(vec![
        Value::int(HTU21D_I2C_ADDR),
        commands,
        Value::int(HTU21D_RESOLUTION_MASK),
        resolutions,
        ids,
        devices,
        Value::float(HTU21D_TEMP_COEFFICIENT),
        Value::int(HTU21D_CRC8_POLYNOMINAL),
    ]);
    assert!(
        values_match(&expected, &actual),
        "module constants mismatch\n  expected: {}\n  actual:   {}",
        expected.render(),
        actual.render()
    );
}

#[test]
fn config_defaults_and_errors() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec("printer, sensor = make_sensor([('sensor_type', 'HTU21D')])")
        .expect("python defaults");
    let (printer, _c, _i, rust) = rig(&[("sensor_type", "HTU21D")], vec![]);
    // `self.printer.add_object("htu21d " + self.name, self)` and the
    // `klippy:connect` registration (lines 106-108).
    let py_names = py.eval("sorted(printer.objects.keys())").unwrap();
    let rust_names: Vec<String> = printer.objects.borrow().keys().cloned().collect();
    assert_eq!(
        py_names,
        Value::List(rust_names.iter().map(|n| Value::str(n)).collect()),
        "printer object names"
    );
    let py_events = py.eval("sorted(printer.handlers.keys())").unwrap();
    let rust_events: Vec<String> = printer
        .handlers
        .borrow()
        .iter()
        .map(|(event, _)| event.clone())
        .collect();
    assert_eq!(
        py_events,
        Value::List(rust_events.iter().map(|e| Value::str(e)).collect()),
        "registered event handlers"
    );
    assert_eq!(rust.get_report_time_delta(), 30);
    let expected = py.eval("sensor.get_report_time_delta()");
    assert_i64_equal("report_time default", expected, rust.get_report_time_delta());
    py.exec(
        "printer, sensor = make_sensor([('sensor_type', 'SI7021'), \
         ('htu21d_hold_master', 'true'), ('htu21d_resolution', 'TEMP14_HUM12'), \
         ('htu21d_report_time', '45')])",
    )
    .expect("python explicit");
    let (_p2, _c2, _i2, rust2) = rig(
        &[
            ("sensor_type", "SI7021"),
            ("htu21d_hold_master", "true"),
            ("htu21d_resolution", "TEMP14_HUM12"),
            ("htu21d_report_time", "45"),
        ],
        vec![],
    );
    let expected = py.eval("sensor.get_report_time_delta()");
    assert_i64_equal("report_time explicit", expected, rust2.get_report_time_delta());
    assert_eq!(rust2.get_report_time_delta(), 45);

    py.exec("try:\n    make_sensor([('sensor_type', 'HTU21D'), ('htu21d_resolution', 'BOGUS')])\n    _err = ('ok', '')\nexcept Exception as e:\n    _err = (type(e).__name__, str(e))")
        .expect("python invalid resolution");
    let expected = py.eval("_err");
    let printer = LogFakePrinter::new();
    let config = FakeConfig::new(
        printer.clone(),
        "sensor test1",
        &[("sensor_type", "HTU21D"), ("htu21d_resolution", "BOGUS")],
    );
    let i2c = FakeI2c::new(vec![]);
    let config_dyn: Config = config.clone();
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c;
    let actual = HTU21D::from_parts(&config_dyn, i2c_dyn).map(|_| ());
    match (expected, actual) {
        (Ok(Value::Tuple(parts)), Err(e)) => {
            assert_eq!(parts.len(), 2);
            if let (Value::Str(cls), Value::Str(msg)) = (&parts[0], &parts[1]) {
                assert_eq!(e.class_name(), cls, "resolution error type");
                assert_eq!(e.message(), msg, "resolution error message");
            } else {
                panic!("unexpected oracle error tuple: {parts:?}");
            }
        }
        (e, a) => panic!("invalid resolution: expected={e:?} actual={a:?}"),
    }

    py.exec("try:\n    make_sensor([])\n    _err2 = ('ok', '')\nexcept Exception as e:\n    _err2 = (type(e).__name__, str(e))")
        .expect("python missing sensor_type");
    let expected = py.eval("_err2");
    let printer = LogFakePrinter::new();
    let config = FakeConfig::new(printer.clone(), "sensor test1", &[]);
    let i2c = FakeI2c::new(vec![]);
    let config_dyn: Config = config.clone();
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c;
    let actual = HTU21D::from_parts(&config_dyn, i2c_dyn).map(|_| ());
    match (expected, actual) {
        (Ok(Value::Tuple(parts)), Err(e)) => {
            assert_eq!(parts.len(), 2);
            if let (Value::Str(cls), Value::Str(msg)) = (&parts[0], &parts[1]) {
                assert_eq!(e.class_name(), cls, "missing sensor_type error type");
                assert_eq!(e.message(), msg, "missing sensor_type error message");
            } else {
                panic!("unexpected oracle error tuple: {parts:?}");
            }
        }
        (e, a) => panic!("missing sensor_type: expected={e:?} actual={a:?}"),
    }

    py.exec(
        "try:\n    make_sensor([('sensor_type', 'HTU21D'), ('htu21d_report_time', '3')])\n    _err3 = ('ok', '')\nexcept Exception as e:\n    _err3 = (type(e).__name__, str(e))",
    )
    .expect("python report_time minval");
    let expected = py.eval("_err3");
    let printer = LogFakePrinter::new();
    let config = FakeConfig::new(
        printer.clone(),
        "sensor test1",
        &[("sensor_type", "HTU21D"), ("htu21d_report_time", "3")],
    );
    let i2c = FakeI2c::new(vec![]);
    let config_dyn: Config = config.clone();
    let i2c_dyn: Arc<dyn McuI2cApi> = i2c;
    let actual = HTU21D::from_parts(&config_dyn, i2c_dyn).map(|_| ());
    match (expected, actual) {
        (Ok(Value::Tuple(parts)), Err(e)) => {
            assert_eq!(parts.len(), 2);
            if let (Value::Str(cls), Value::Str(msg)) = (&parts[0], &parts[1]) {
                assert_eq!(e.class_name(), cls, "report_time minval error type");
                assert_eq!(e.message(), msg, "report_time minval error message");
            } else {
                panic!("unexpected oracle error tuple: {parts:?}");
            }
        }
        (e, a) => panic!("report_time minval: expected={e:?} actual={a:?}"),
    }
}

#[test]
fn init_sequence_matches_python() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let crc = py_crc8(&mut py, 0x3200);
    let serial = vec![0x32u8, 0x00, crc as u8];
    let reg = vec![0x3Eu8];

    py.exec("printer, sensor = make_sensor([('sensor_type', 'HTU21D')])")
        .expect("python sensor");
    // Drive the registered `klippy:connect` handler, as the printer does.
    py.exec(&format!(
        "queue_reads({serial:?}, {reg:?})\nfor _cb in printer.handlers['klippy:connect']:\n    _cb()"
    ))
    .expect("python init");
    let py_calls_v = py.eval("py_calls()").unwrap();
    let py_logs_v = py.eval("py_logs()").unwrap();

    let (printer, _c, i2c, _rust) = rig(&[("sensor_type", "HTU21D")], vec![serial, reg]);
    printer
        .send_event("klippy:connect", &[])
        .expect("rust klippy:connect");
    let rust_calls = i2c.calls();
    let rust_logs = log_records_value();

    assert_eq!(py_calls_v, i2c_calls_value(&rust_calls), "init i2c calls");
    assert_eq!(rust_calls.len(), 4, "init call count");
    assert_eq!(rust_logs, py_logs_v, "init log records");
}

#[test]
fn init_unknown_device_and_bad_crc() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let serial = vec![0x77u8, 0x00, 0x00];
    let reg = vec![0x00u8];
    py.exec("printer, sensor = make_sensor([('sensor_type', 'HTU21D')])")
        .expect("python sensor");
    py.exec(&format!("queue_reads({serial:?}, {reg:?}); sensor._init_htu21d()"))
        .expect("python init unknown");
    let py_logs_v = py.eval("py_logs()").unwrap();

    let (_p, _c, _i, rust) = rig(&[("sensor_type", "HTU21D")], vec![serial, reg]);
    rust.init_htu21d().expect("rust init unknown");
    let rust_logs = log_records_value();
    assert_eq!(rust_logs, py_logs_v, "unknown-device log records");
    assert_eq!(
        log_messages(&rust_logs),
        vec![
            "htu21d: Reading deviceId !Checksum error!".to_string(),
            "htu21d: Unknown Device ID 0x77 ".to_string(),
            "htu21d: Found device Unknown. Forcing to type HTU21D as config."
                .to_string(),
            "htu21d: Setting resolution to TEMP12_HUM08 ".to_string(),
        ]
    );
}

#[test]
fn sample_conversion_matches_python() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let cases: Vec<(i64, i64)> = vec![
        (0x8000, 0x8002),
        (0xC000, 0xB002),
        (0x0000, 0x0002),
    ];
    let mut combos: Vec<(&str, &str)> = ["SI7013", "SI7020", "SI7021", "SHT21", "HTU21D"]
        .iter()
        .map(|d| (*d, "TEMP12_HUM08"))
        .collect();
    combos.extend(
        ["TEMP14_HUM12", "TEMP13_HUM10", "TEMP11_HUM11"]
            .iter()
            .map(|r| ("HTU21D", *r)),
    );
    for (device, resolution) in combos {
        for (raw_t, raw_h) in &cases {
                let ct = py_crc8(&mut py, *raw_t);
                let ch = py_crc8(&mut py, *raw_h);
                let temp_bytes = vec![(raw_t >> 8) as u8, *raw_t as u8, ct as u8];
                let hum_bytes = vec![(raw_h >> 8) as u8, *raw_h as u8, ch as u8];

                py.exec(&format!(
                    "CB = []\n\
                     printer, sensor = make_sensor([('sensor_type', '{device}'), ('htu21d_resolution', '{resolution}')])\n\
                     sensor.setup_minmax(-50.0, 150.0)\n\
                     sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
                     queue_reads({temp_bytes:?}, {hum_bytes:?})"
                ))
                .expect("python sample setup");
                let py_ret = py.eval("sensor._sample_htu21d(0.)");
                let py_status = py.eval("py_status(sensor)");
                let py_cb = py.eval("CB");
                let py_logs_v = py.eval("py_logs()").unwrap();
                let py_calls_v = py.eval("py_calls()").unwrap();

                let (_p2, _c2, i2c, rust) = rig(
                    &[
                        ("sensor_type", device),
                        ("htu21d_resolution", resolution),
                    ],
                    vec![temp_bytes.clone(), hum_bytes.clone()],
                );
                rust.setup_minmax(-50.0, 150.0);
                let cb_calls: Arc<kos::sync::Mutex<Vec<(f64, f64)>>> =
                    Arc::new(kos::sync::Mutex::new(Vec::new()));
                let cb_calls2 = cb_calls.clone();
                rust.setup_callback(Box::new(move |pt, t| {
                    cb_calls2.lock().push((pt, t));
                }));
                let reactor = _p2.get_reactor();
                let before = reactor.monotonic();
                let rust_ret = rust.sample(0.0);
                let after = reactor.monotonic();
                let rust_status = rust.status_dict();
                let rust_logs = log_records_value();
                let rust_calls = i2c.calls();

                let label = format!("{device}/{resolution}/t={raw_t:#x}/h={raw_h:#x}");
                assert_status_equal(&format!("{label} status"), py_status, rust_status);
                let cb = cb_calls.lock().clone();
                assert_eq!(cb.len(), 1, "{label} callback count");
                assert_eq!(cb[0].0.to_bits(), 5555.0f64.to_bits(), "{label} print_time");
                match py_cb {
                    Ok(Value::List(items)) => {
                        assert_eq!(items.len(), 1, "{label} python callback count");
                        if let Value::Tuple(p) = &items[0] {
                            if let (Value::Float(pt), Value::Float(t)) = (&p[0], &p[1]) {
                                assert_eq!(
                                    pt.to_f64().unwrap().to_bits(),
                                    cb[0].0.to_bits(),
                                    "{label} callback print_time"
                                );
                                assert_eq!(
                                    t.to_f64().unwrap().to_bits(),
                                    cb[0].1.to_bits(),
                                    "{label} callback temp"
                                );
                            } else {
                                panic!("{label}: bad python callback {items:?}");
                            }
                        } else {
                            panic!("{label}: bad python callback {items:?}");
                        }
                    }
                    other => panic!("{label}: bad python callback {other:?}"),
                }
                // The reference returns `measured_time + self.report_time`
                // (line 234); its fake reactor's monotonic is 1000.0 and the
                // default report time is 30.
                let py_ret_f = match py_ret {
                    Ok(Value::Float(f)) => f.to_f64().unwrap(),
                    other => panic!("{label}: bad python return {other:?}"),
                };
                assert_eq!(
                    py_ret_f.to_bits(),
                    1030.0f64.to_bits(),
                    "{label} python return (1000 + 30)"
                );
                assert!(
                    rust_ret >= before + 30.0 - 1e-6 && rust_ret <= after + 30.0 + 1e-6,
                    "{label} rust return is measured_time + report_time, got {rust_ret} \
                     (monotonic {before}..{after})"
                );
                assert_eq!(
                    py_calls_v,
                    i2c_calls_value(&rust_calls),
                    "{label} i2c calls"
                );
                assert_eq!(
                    rust_calls.len(),
                    4,
                    "{label} i2c call count"
                );
                assert_eq!(
                    rust_logs,
                    py_logs_v,
                    "{label} sample log records"
                );
        }
    }
}


#[test]
fn sample_crc_errors_and_clamping() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let temp_bytes = vec![0x80u8, 0x00, 0x00];
    let hum_bytes = vec![0x80u8, 0x02, 0x00];
    py.exec(&format!(
        "CB = []\n\
         printer, sensor = make_sensor([('sensor_type', 'HTU21D')])\n\
         sensor.setup_minmax(-50.0, 150.0)\n\
         sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
         queue_reads({temp_bytes:?}, {hum_bytes:?})"
    ))
    .expect("python crc setup");
    py.eval("sensor._sample_htu21d(0.)").expect("python crc sample");
    let py_status = py.eval("py_status(sensor)");
    let py_logs_v = py.eval("py_logs()").unwrap();

    let (_p, _c, _i, rust) =
        rig(&[("sensor_type", "HTU21D")], vec![temp_bytes.clone(), hum_bytes.clone()]);
    rust.setup_minmax(-50.0, 150.0);
    let cb_calls = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let cb2 = cb_calls.clone();
    rust.setup_callback(Box::new(move |pt, t| cb2.lock().push((pt, t))));
    rust.sample(0.0);
    assert_status_equal("crc errors status", py_status, rust.status_dict());
    assert_eq!(
        log_records_value(),
        py_logs_v,
        "crc error log records"
    );

    // The clamp assigns the int literals 0 and 100 (lines 209-214), so the
    // status humidity type changes too; the oracle decides both.
    for raw_h in [0x0002i64, 0xFFFDi64] {
        let raw_t = 0x8000i64;
        let ct = py_crc8(&mut py, raw_t);
        let ch = py_crc8(&mut py, raw_h);
        let tb = vec![(raw_t >> 8) as u8, raw_t as u8, ct as u8];
        let hb = vec![(raw_h >> 8) as u8, raw_h as u8, ch as u8];
        py.exec(&format!(
            "CB = []\n\
             printer, sensor = make_sensor([('sensor_type', 'HTU21D')])\n\
             sensor.setup_minmax(-50.0, 150.0)\n\
             sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
             queue_reads({tb:?}, {hb:?})"
        ))
        .expect("python clamp setup");
        py.eval("sensor._sample_htu21d(0.)").expect("python clamp sample");
        let py_status = py.eval("py_status(sensor)");

        let (_p2, _c2, _i2, rust) = rig(&[("sensor_type", "HTU21D")], vec![tb, hb]);
        rust.setup_minmax(-50.0, 150.0);
        rust.setup_callback(Box::new(|_, _| {}));
        rust.sample(0.0);
        assert_status_equal(
            &format!("humidity clamp h={raw_h:#x}"),
            py_status,
            rust.status_dict(),
        );
    }
}

#[test]
fn sample_shutdown_and_exception_paths() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let raw_t = 0xC000i64;
    let raw_h = 0x8002i64;
    let tb = vec![(raw_t >> 8) as u8, raw_t as u8, py_crc8(&mut py, raw_t) as u8];
    let hb = vec![(raw_h >> 8) as u8, raw_h as u8, py_crc8(&mut py, raw_h) as u8];
    py.exec(&format!(
        "CB = []\n\
         printer, sensor = make_sensor([('sensor_type', 'HTU21D')])\n\
         sensor.setup_minmax(0.0, 10.0)\n\
         sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
         queue_reads({tb:?}, {hb:?})"
    ))
    .expect("python shutdown setup");
    py.eval("sensor._sample_htu21d(0.)").expect("python shutdown sample");
    let py_shutdowns = py.eval("printer.shutdowns").unwrap();
    let py_logs_v = py.eval("py_logs()").unwrap();

    let (printer, _c, _i, rust) =
        rig(&[("sensor_type", "HTU21D")], vec![tb.clone(), hb.clone()]);
    rust.setup_minmax(0.0, 10.0);
    rust.setup_callback(Box::new(|_, _| {}));
    rust.sample(0.0);
    let rust_shutdowns = printer.shutdowns.borrow().clone();
    assert_eq!(
        log_records_value(),
        py_logs_v,
        "shutdown sample log records"
    );
    match py_shutdowns {
        Value::List(items) => {
            let expected: Vec<String> = items
                .iter()
                .map(|v| {
                    if let Value::Str(s) = v {
                        s.clone()
                    } else {
                        panic!("bad shutdown {v:?}")
                    }
                })
                .collect();
            assert_eq!(rust_shutdowns, expected, "shutdown messages");
            assert_eq!(rust_shutdowns.len(), 1, "one shutdown");
        }
        other => panic!("bad python shutdowns {other:?}"),
    }

    py.exec(
        "CB = []\n\
         printer, sensor = make_sensor([('sensor_type', 'HTU21D')])\n\
         sensor.setup_minmax(-50.0, 150.0)\n\
         sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
         FAKE_I2C.fail_read = Exception('boom')",
    )
    .expect("python exception setup");
    let py_ret = py.eval("sensor._sample_htu21d(0.)");
    let py_status = py.eval("py_status(sensor)");
    let py_logs_v = py.eval("py_logs()").unwrap();

    let (_printer2, _c2, i2c2, rust2) =
        rig(&[("sensor_type", "HTU21D")], vec![]);
    i2c2.fail_next_read
        .borrow_mut()
        .replace(KlippyError::Mcu("boom".to_string()));
    rust2.setup_minmax(-50.0, 150.0);
    rust2.setup_callback(Box::new(|_, _| {}));
    let rust_ret = rust2.sample(0.0);
    assert_eq!(
        rust_ret.to_bits(),
        NEVER.to_bits(),
        "exception path returns NEVER"
    );
    match py_ret {
        Ok(Value::Float(f)) => assert_eq!(
            f.to_f64().unwrap().to_bits(),
            NEVER.to_bits(),
            "python exception return"
        ),
        other => panic!("bad python exception return {other:?}"),
    }
    assert_status_equal("exception status", py_status, rust2.status_dict());
    assert_eq!(
        log_records_value(),
        py_logs_v,
        "exception log records"
    );
}

#[test]
fn hold_master_uses_hold_commands() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let raw_t = 0x8000i64;
    let raw_h = 0x8002i64;
    let tb = vec![(raw_t >> 8) as u8, raw_t as u8, py_crc8(&mut py, raw_t) as u8];
    let hb = vec![(raw_h >> 8) as u8, raw_h as u8, py_crc8(&mut py, raw_h) as u8];
    py.exec(&format!(
        "CB = []\n\
         printer, sensor = make_sensor([('sensor_type', 'HTU21D'), ('htu21d_hold_master', 'true')])\n\
         sensor.setup_minmax(-50.0, 150.0)\n\
         sensor.setup_callback(lambda pt, t: CB.append((pt, t)))\n\
         queue_reads({tb:?}, {hb:?})"
    ))
    .expect("python hold setup");
    py.eval("sensor._sample_htu21d(0.)").expect("python hold sample");
    let py_calls_v = py.eval("py_calls()").unwrap();

    let (_p2, _c2, i2c, rust) = rig(
        &[("sensor_type", "HTU21D"), ("htu21d_hold_master", "true")],
        vec![tb, hb],
    );
    rust.setup_minmax(-50.0, 150.0);
    rust.setup_callback(Box::new(|_, _| {}));
    rust.sample(0.0);
    let rust_calls = i2c.calls();
    assert_eq!(
        py_calls_v,
        i2c_calls_value(&rust_calls),
        "hold-master i2c calls"
    );
    assert_eq!(rust_calls.len(), 4, "hold-master call count");
}

#[test]
fn load_config_registers_all_devices() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(
        "class FakeHeaters:\n    def __init__(self):\n        self.factories = []\n    def add_sensor_factory(self, sensor_type, factory):\n        self.factories.append(sensor_type)\nheaters = FakeHeaters()\nprinter = FakePrinter()\nprinter.objects['heaters'] = heaters\nconfig = FakeConfig(printer, 'htu21d', {})\nret = htu21d_mod.load_config(config)\nregistered = list(heaters.factories)",
    )
    .expect("python load_config");
    let py_registered = py.eval("registered").unwrap();
    let py_ret = py.eval("ret").unwrap();

    let printer = LogFakePrinter::new();
    let heaters = Arc::new(FakeHeaters { factories: GCell::new(Vec::new()) });
    printer
        .add_object("heaters", heaters.clone())
        .expect("add heaters");
    let config = FakeConfig::new(printer.clone(), "htu21d", &[]);
    let config_dyn: Config = config.clone();
    let ret = load_config(&config_dyn).expect("rust load_config");
    assert_eq!(py_ret, Value::Null, "python load_config returns None");
    assert!(ret.is_none(), "rust load_config returns None");
    let rust_registered = heaters.factories.borrow().clone();
    match py_registered {
        Value::List(items) => {
            let expected: Vec<String> = items
                .iter()
                .map(|v| {
                    if let Value::Str(s) = v {
                        s.clone()
                    } else {
                        panic!("bad registration {v:?}")
                    }
                })
                .collect();
            assert_eq!(rust_registered, expected, "registered sensor types");
            assert_eq!(
                rust_registered,
                vec!["SI7013", "SI7020", "SI7021", "SHT21", "HTU21D"]
            );
        }
        other => panic!("bad python registrations {other:?}"),
    }
}

#[test]
fn load_config_missing_heaters_errors() {
    lock_logs();
    let Some(mut py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(
        "printer = FakePrinter()\nconfig = FakeConfig(printer, 'htu21d', {})\ntry:\n    htu21d_mod.load_config(config)\n    _err = ('ok', '')\nexcept Exception as e:\n    _err = (type(e).__name__, str(e))",
    )
    .expect("python load_config missing heaters");
    let expected = py.eval("_err");
    let printer = LogFakePrinter::new();
    let config = FakeConfig::new(printer.clone(), "htu21d", &[]);
    let config_dyn: Config = config.clone();
    let actual = load_config(&config_dyn).map(|_| ());
    match (expected, actual) {
        (Ok(Value::Tuple(parts)), Err(e)) => {
            if let (Value::Str(cls), Value::Str(msg)) = (&parts[0], &parts[1]) {
                assert_eq!(e.class_name(), cls, "missing heaters error type");
                assert_eq!(e.message(), msg, "missing heaters error message");
            } else {
                panic!("unexpected oracle error tuple: {parts:?}");
            }
        }
        (e, a) => panic!("missing heaters: expected={e:?} actual={a:?}"),
    }
}

#[test]
fn load_config_heaters_without_interface_errors() {
    lock_logs();
    let Some(py) = oracle_start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    struct PlainObject;
    impl PrinterObject for PlainObject {
        klippy_api::printer_object_base!();
    }
    let printer = LogFakePrinter::new();
    printer.add_object("heaters", Arc::new(PlainObject)).expect("add plain heaters");
    let config = FakeConfig::new(printer.clone(), "htu21d", &[]);
    let config_dyn: Config = config.clone();
    let actual = load_config(&config_dyn).map(|_| ());
    match actual {
        Err(e) => {
            assert_eq!(e.class_name(), "AttributeError", "plain heaters error type");
        }
        Ok(_) => panic!("plain heaters should error"),
    }
    let _ = py;
}
