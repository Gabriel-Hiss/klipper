//! Parity tests for `extras::ads1x1x` (port of `klippy/extras/ads1x1x.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `ADS1X1X_chip` is built
//! on both sides with minimal Python fakes for the printer, config wrapper,
//! pins and the `bus.MCU_I2C` handle, then fed identical config options and
//! driven through `setup_pin`, `calculate_sample_rate`,
//! `handle_report_time_update`, `sample`, `_handle_connect` and
//! `ADS1X1X_pin._process_sample`, comparing returned values (floats by
//! bits), raised errors (class and message), I2C writes, shutdowns and log
//! lines exactly.
//!
//! The `bus.MCU_I2C` handle is a Python fake on both sides; the Rust chip
//! constructor takes the same fake through
//! `ADS1X1X_chip::from_config(config, i2c)`, mirroring how
//! `bus.MCU_I2C_from_config` supplies the real handle in production.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use std::sync::{Arc, Mutex};

use extras::ads1x1x::{
    ADS101X_RESOLUTION, ADS101X_SAMPLES_PER_SECOND, ADS111X_RESOLUTION,
    ADS111X_SAMPLES_PER_SECOND, ADS1X1X_chip, ADS1X1X_pin, ChipCore, McuI2c,
    resolve_chip_address,
};
use extras::bus::McuI2cApi as BusMcuI2cApi;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AdcPinApi, AdcSampleOpts, AsyncResponseWrapper, CommandQueue,
    CommandWrapper, CommandQueryWrapper, ConfigCallback, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinsApi,
    QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import pins as pins_mod
import mcu as mcu_mod
import extras.bus as bus_mod
import extras.ads1x1x as ads1x1x

class FakeReactor:
    NOW = 0.0
    def __init__(self):
        self.t = 0.0
        self.timers = []
    def monotonic(self):
        return self.t
    def pause(self, wake):
        self.t = max(self.t, wake)
        return wake
    def register_timer(self, cb, wake):
        self.timers.append((cb, wake))
        return len(self.timers)
    def mutex(self):
        reactor = self
        class M:
            def __enter__(self):
                return self
            def __exit__(self, *a):
                return False
        return M()

class FakeMcu:
    def __init__(self):
        self.est_calls = 0
    def get_name(self):
        return 'mcu'
    def estimated_print_time(self, systime):
        self.est_calls += 1
        return 1234.5
fake_mcu = FakeMcu()

class FakeI2C:
    def __init__(self):
        self.writes = []
        self.reads = 0
        self.addrs = []
        self.config_value = 0x8000
        self.conversion = 0x4000
        self.not_ready = 0
        self.fail_write = None
        self.fail_read = None
    def i2c_write(self, data):
        if self.fail_write is not None:
            raise mcu_mod.error(self.fail_write)
        self.writes.append(list(data))
    def i2c_read(self, write, read_len):
        if self.fail_read is not None:
            raise mcu_mod.error(self.fail_read)
        self.reads += 1
        reg = write[0]
        if reg == 1:
            if self.not_ready > 0:
                self.not_ready -= 1
                v = 0
            else:
                v = self.config_value
        elif reg == 0:
            v = self.conversion
        else:
            v = 0
        return {'response': [(v >> 8) & 0xff, v & 0xff]}
    def get_mcu(self):
        return fake_mcu
fake_i2c = FakeI2C()

def fake_i2c_from_config(config, addr):
    fake_i2c.addrs.append(addr)
    return fake_i2c
bus_mod.MCU_I2C_from_config = fake_i2c_from_config

class FakeLogging:
    def __init__(self):
        self.records = []
    def warning(self, msg):
        self.records.append(('WARNING', msg))
    def exception(self, msg):
        self.records.append(('ERROR', msg))
fakelog = FakeLogging()
ads1x1x.logging = fakelog

class FakePins:
    def __init__(self):
        self.chips = []
    def register_chip(self, name, chip):
        name = name.strip()
        for (n, _) in self.chips:
            if n == name:
                raise pins_mod.error("Duplicate chip name '%s'" % (name,))
        self.chips.append((name, chip))

class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.pins = FakePins()
        self.objects = []
        self.handlers = []
        self.shutdowns = []
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        if name == 'pins':
            return self.pins
        return default
    def add_object(self, name, obj):
        self.objects.append(name)
    def register_event_handler(self, event, cb):
        self.handlers.append(event)
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)

_SENTINEL = object()

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options):
        self._printer = printer
        self._name = name
        self._options = dict(options)
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option, default=None):
        return self._options.get(option, default)
    def _raw(self, option, default):
        if option in self._options:
            return self._options[option]
        if default is not _SENTINEL:
            return default
        raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._name))
    def getchoice(self, option, choices, default=_SENTINEL):
        raw = self._raw(option, default)
        if raw not in choices:
            raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (raw, option, self._name))
        return choices[raw]
    def getfloat(self, option, default=_SENTINEL, above=None):
        raw = self._raw(option, default)
        try:
            v = float(raw)
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        return v

def new_pair(options):
    global p, c, chip
    p = FakePrinter()
    c = FakeConfig(p, 'ads1x1x temp', options)
    fake_i2c.writes = []
    fake_i2c.reads = 0
    fake_i2c.addrs = []
    fake_i2c.config_value = 0x8000
    fake_i2c.conversion = 0x4000
    fake_i2c.not_ready = 0
    fake_i2c.fail_write = None
    fake_i2c.fail_read = None
    fakelog.records = []
    chip = ads1x1x.load_config_prefix(c)
    return chip
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Default)]
struct FakePrinterState {
    objects: Vec<String>,
    handlers: Vec<String>,
    shutdowns: Vec<String>,
    chips: Vec<String>,
}

struct FakePins {
    state: Arc<Mutex<FakePrinterState>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by ads1x1x")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used by ads1x1x")
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("PinsApi::setup_pin is not used by ads1x1x")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by ads1x1x")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by ads1x1x")
    }
    fn register_chip(&self, chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        let name = chip_name.trim().to_string();
        let mut state = self.state.lock().unwrap();
        if state.chips.contains(&name) {
            return Err(KlippyError::Pins(format!("Duplicate chip name '{name}'")));
        }
        state.chips.push(name);
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by ads1x1x")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakePrinter {
    reactor: Reactor,
    state: Arc<Mutex<FakePrinterState>>,
    pins: ObjectRef,
}

impl FakePrinter {
    fn new() -> (FakePrinter, Arc<Mutex<FakePrinterState>>) {
        let state = Arc::new(Mutex::new(FakePrinterState::default()));
        let pins: ObjectRef = Arc::new(FakePins { state: state.clone() });
        let printer = FakePrinter { reactor: Reactor::new(false), state: state.clone(), pins };
        (printer, state)
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
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, _obj: ObjectRef) -> KResult<()> {
        self.state.lock().unwrap().objects.push(name.to_string());
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if name == "pins" {
            return Ok(Some(self.pins.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: name.to_string(),
            }),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by ads1x1x")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.state.lock().unwrap().shutdowns.push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by ads1x1x")
    }
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.state.lock().unwrap().handlers.push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: indexmap::IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer) -> Self {
        FakeConfig {
            printer,
            name: "ads1x1x temp".to_string(),
            options: indexmap::IndexMap::new(),
        }
    }
    fn with_option(mut self, key: &str, value: &str) -> Self {
        self.options.insert(key.to_string(), value.to_string());
        self
    }
    fn required_err(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option, self.name
        ))
    }
}

fn dict_get_str<'a>(dict: &'a indexmap::IndexMap<PyValue, PyValue>, key: &str) -> Option<&'a PyValue> {
    dict.get(&PyValue::Str(key.to_string()))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.required_err(option)),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by ads1x1x")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.options.get(option) {
            Some(v) => v.clone(),
            None => match opts.default {
                Some(d) => match d {
                    Some(v) => return Ok(Some(v)),
                    None => return Ok(None),
                },
                None => return Err(self.required_err(option)),
            },
        };
        let v: f64 = raw.parse().map_err(|_| {
            KlippyError::config(format!(
                "Unable to parse option '{}' in section '{}'",
                option, self.name
            ))
        })?;
        if let Some(above) = opts.above {
            if v <= above {
                let msg = pycompat::text::percent_format(
                    "Option '%s' in section '%s' must be above %s",
                    &[
                        pycompat::PctValue::Str(option.to_string()),
                        pycompat::PctValue::Str(self.name.clone()),
                        pycompat::PctValue::Float(above),
                    ],
                )
                .expect("percent format is valid");
                return Err(KlippyError::config(msg));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by ads1x1x")
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw = match self.options.get(option) {
            Some(v) => v.clone(),
            None => match opts.default {
                Some(d) => match d {
                    Some(PyValue::Str(s)) => s,
                    Some(_) => panic!("getchoice default is not a string key"),
                    None => return Err(self.required_err(option)),
                },
                None => return Err(self.required_err(option)),
            },
        };
        let PyValue::Dict(dict) = choices else {
            panic!("choices must be a dict");
        };
        match dict_get_str(dict, &raw) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::config(format!(
                "Choice '{raw}' for option '{option}' in section '{}' is not a valid choice",
                self.name
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by ads1x1x")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by ads1x1x")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by ads1x1x")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by ads1x1x")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by ads1x1x")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by ads1x1x")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// FakeMcu: only the methods ads1x1x calls are implemented.

struct FakeMcuState {
    est_calls: usize,
}

struct FakeMcu {
    printer: Printer,
    state: Arc<Mutex<FakeMcuState>>,
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
        panic!("McuApi::setup_pin is not used by ads1x1x")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("create_oid is not used by ads1x1x")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("register_config_callback is not used by ads1x1x")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("add_config_cmd is not used by ads1x1x")
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by ads1x1x")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by ads1x1x")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("seconds_to_clock is not used by ads1x1x")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by ads1x1x")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by ads1x1x")
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        panic!("lookup_command is not used by ads1x1x")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command is not used by ads1x1x")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by ads1x1x")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("alloc_command_queue is not used by ads1x1x")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by ads1x1x")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by ads1x1x")
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        panic!("get_enumerations is not used by ads1x1x")
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        panic!("get_constants is not used by ads1x1x")
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by ads1x1x")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by ads1x1x")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by ads1x1x")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        self.state.lock().unwrap().est_calls += 1;
        1234.5
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by ads1x1x")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by ads1x1x")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by ads1x1x")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by ads1x1x")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by ads1x1x")
    }
}

#[derive(Default)]
struct FakeI2cState {
    writes: Vec<Vec<u8>>,
    reads: usize,
    config_value: i64,
    conversion: i64,
    not_ready: usize,
    fail_write: Option<String>,
    fail_read: Option<String>,
}

struct FakeI2c {
    mcu: Mcu,
    state: Arc<Mutex<FakeI2cState>>,
}

fn i2c_error(msg: &str) -> KlippyError {
    KlippyError::Mcu(msg.to_string())
}

impl BusMcuI2cApi for FakeI2c {
    fn get_oid(&self) -> i64 {
        0
    }
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_i2c_address(&self) -> i64 {
        0x48
    }
    fn get_command_queue(&self) -> CommandQueue {
        panic!("get_command_queue is not used by ads1x1x")
    }
    fn build_config(&self) -> KResult<()> {
        panic!("build_config is not used by ads1x1x")
    }
    fn i2c_write_noack(&self, _data: &[u8], _opts: SendOpts) -> KResult<()> {
        panic!("i2c_write_noack is not used by ads1x1x")
    }
    fn i2c_write(&self, data: &[u8], _opts: QuerySendOpts) -> KResult<()> {
        let mut state = self.state.lock().unwrap();
        if let Some(msg) = &state.fail_write {
            return Err(i2c_error(msg));
        }
        state.writes.push(data.to_vec());
        Ok(())
    }
    fn i2c_read(&self, write: &[u8], _read_len: i64, _opts: QuerySendOpts) -> KResult<PyValue> {
        let mut state = self.state.lock().unwrap();
        if let Some(msg) = &state.fail_read {
            return Err(i2c_error(msg));
        }
        state.reads += 1;
        let reg = write[0];
        let value = if reg == 1 {
            if state.not_ready > 0 {
                state.not_ready -= 1;
                0
            } else {
                state.config_value
            }
        } else if reg == 0 {
            state.conversion
        } else {
            0
        };
        let mut response = indexmap::IndexMap::new();
        response.insert(
            PyValue::Str("response".to_string()),
            PyValue::List(vec![
                PyValue::Int(pycompat::PyInt::from_i64((value >> 8) & 0xff)),
                PyValue::Int(pycompat::PyInt::from_i64(value & 0xff)),
            ]),
        );
        Ok(PyValue::Dict(response))
    }
    fn i2c_transfer(
        &self,
        _write: &[u8],
        _read_len: i64,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("i2c_transfer is not used by ads1x1x")
    }
}

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_RECORDS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_RECORDS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn take_logs() -> Vec<(String, String)> {
    install_log_sink();
    LOG_RECORDS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("ads1x1x fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping ads1x1x oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}
#[test]
fn chip_address_resolution() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "resolve_addr = lambda c: ads1x1x.ADS1X1X_CHIP_ADDR['GND'] if c.get('address_pin', None) is None else c.getchoice('address_pin', ads1x1x.ADS1X1X_CHIP_ADDR)",
    )
    .expect("resolve helper");
    for opts in [
        vec![],
        vec![("address_pin", "GND")],
        vec![("address_pin", "VCC")],
        vec![("address_pin", "SDA")],
        vec![("address_pin", "SCL")],
    ] {
        let mut py_opts = vec![("chip", "ADS1115")];
        py_opts.extend(opts.iter().copied());
        let dict = py_opts
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!(
            "p = FakePrinter(); c = FakeConfig(p, 'ads1x1x temp', {{{dict}}})"
        ))
        .expect("py config");
        let expected = py.eval("resolve_addr(c)");
        let (printer, _) = FakePrinter::new();
        let printer: Printer = Arc::new(printer);
        let mut config = FakeConfig::new(printer);
        config = config.with_option("chip", "ADS1115");
        for (k, v) in &opts {
            config = config.with_option(k, v);
        }
        let config: Config = Arc::new(config);
        check_int(
            &format!("address {opts:?}"),
            expected,
            resolve_chip_address(&config).unwrap(),
        );
        // The same address reaches the Python bus factory as default addr.
        let dict2 = py_opts
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ");
        py.exec(&format!("new_pair({{{dict2}}})")).expect("py load_config_prefix");
        let expected_addr = py.eval("fake_i2c.addrs[0]");
        let (printer, _) = FakePrinter::new();
        let printer: Printer = Arc::new(printer);
        let mut config = FakeConfig::new(printer);
        for (k, v) in py_opts {
            config = config.with_option(k, v);
        }
        let config: Config = Arc::new(config);
        check_int(
            &format!("bus default addr {opts:?}"),
            expected_addr,
            resolve_chip_address(&config).unwrap(),
        );
    }
}
fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "ads1x1x".to_string(),
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

fn check_pair(label: &str, expected: pyoracle::OracleResult, actual: (i64, i64)) {
    check(label, expected, Value::Tuple(vec![Value::int(actual.0), Value::int(actual.1)]));
}

fn check_int(label: &str, expected: pyoracle::OracleResult, actual: i64) {
    check(label, expected, Value::int(actual));
}

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: f64) {
    check(label, expected, Value::float(actual));
}

fn check_err<T>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust returned Ok but call must fail"),
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

fn check_py_logs(label: &str, py: &mut pyoracle::Oracle) {
    let expected = py.eval("[[l, m] for (l, m) in fakelog.records]");
    let actual = Value::List(
        take_logs()
            .iter()
            .map(|(l, m)| Value::List(vec![Value::str(l), Value::str(m)]))
            .collect(),
    );
    check(label, expected, actual);
}

struct Pair {
    chip: Arc<ADS1X1X_chip>,
    printer_state: Arc<Mutex<FakePrinterState>>,
    i2c_state: Arc<Mutex<FakeI2cState>>,
}

fn fresh_pair(py: &mut pyoracle::Oracle, options: &[(&str, &str)]) -> Pair {
    let dict = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!("new_pair({{{dict}}})")).expect("py load_config_prefix");

    let (printer, printer_state) = FakePrinter::new();
    let printer: Printer = Arc::new(printer);
    let mut config = FakeConfig::new(printer.clone());
    for (k, v) in options {
        config = config.with_option(k, v);
    }
    let config: Config = Arc::new(config);
    let mcu_state = Arc::new(Mutex::new(FakeMcuState { est_calls: 0 }));
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer.clone(), state: mcu_state });
    let i2c_state = Arc::new(Mutex::new(FakeI2cState {
        config_value: 0x8000,
        conversion: 0x4000,
        ..Default::default()
    }));
    let i2c: McuI2c = Arc::new(FakeI2c { mcu, state: i2c_state.clone() });
    let chip = ADS1X1X_chip::from_config(&config, i2c).expect("rust from_config");
    Pair { chip, printer_state, i2c_state }
}

fn pin_params(mcu: Mcu, pin: &str) -> PinParams {
    PinParams {
        chip: mcu,
        chip_name: "temp".to_string(),
        pin: pin.to_string(),
        invert: 0,
        pullup: 0,
        share_type: None,
    }
}

fn pin_by_name(chip: &ADS1X1X_chip, name: &str) -> Arc<ADS1X1X_pin> {
    chip.core.pins.borrow().get(name).cloned().expect("pin exists")
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    let checks: &[(&str, Value)] = &[
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1013']", Value::int(3)),
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1014']", Value::int(4)),
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1015']", Value::int(5)),
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1113']", Value::int(13)),
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1114']", Value::int(14)),
        ("ads1x1x.ADS1X1X_CHIP_TYPE['ADS1115']", Value::int(15)),
        ("ads1x1x.ADS1X1X_CHIP_ADDR['GND']", Value::int(0x48)),
        ("ads1x1x.ADS1X1X_CHIP_ADDR['VCC']", Value::int(0x49)),
        ("ads1x1x.ADS1X1X_CHIP_ADDR['SDA']", Value::int(0x4a)),
        ("ads1x1x.ADS1X1X_CHIP_ADDR['SCL']", Value::int(0x4b)),
        ("ads1x1x.ADS1X1X_REG_POINTER_MASK", Value::int(0x03)),
        ("ads1x1x.ADS1X1X_REG_POINTER['CONVERSION']", Value::int(0x00)),
        ("ads1x1x.ADS1X1X_REG_POINTER['CONFIG']", Value::int(0x01)),
        ("ads1x1x.ADS1X1X_REG_POINTER['LO_THRESH']", Value::int(0x02)),
        ("ads1x1x.ADS1X1X_REG_POINTER['HI_THRESH']", Value::int(0x03)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['OS_MASK']", Value::int(0x8000)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['MULTIPLEXER_MASK']", Value::int(0x7000)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['PGA_MASK']", Value::int(0x0E00)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['MODE_MASK']", Value::int(0x0100)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['DATA_RATE_MASK']", Value::int(0x00E0)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['COMPARATOR_MODE_MASK']", Value::int(0x0010)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['COMPARATOR_POLARITY_MASK']", Value::int(0x0008)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['COMPARATOR_LATCHING_MASK']", Value::int(0x0004)),
        ("ads1x1x.ADS1X1X_REG_CONFIG['COMPARATOR_QUEUE_MASK']", Value::int(0x0003)),
        ("ads1x1x.ADS1X1X_OS['OS_IDLE']", Value::int(0x8000)),
        ("ads1x1x.ADS1X1X_OS['OS_SINGLE']", Value::int(0x8000)),
        ("ads1x1x.ADS1X1X_MUX['DIFF01']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_MUX['DIFF03']", Value::int(0x1000)),
        ("ads1x1x.ADS1X1X_MUX['DIFF13']", Value::int(0x2000)),
        ("ads1x1x.ADS1X1X_MUX['DIFF23']", Value::int(0x3000)),
        ("ads1x1x.ADS1X1X_MUX['AIN0']", Value::int(0x4000)),
        ("ads1x1x.ADS1X1X_MUX['AIN1']", Value::int(0x5000)),
        ("ads1x1x.ADS1X1X_MUX['AIN2']", Value::int(0x6000)),
        ("ads1x1x.ADS1X1X_MUX['AIN3']", Value::int(0x7000)),
        ("ads1x1x.ADS1X1X_PGA['6.144V']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_PGA['4.096V']", Value::int(0x0200)),
        ("ads1x1x.ADS1X1X_PGA['2.048V']", Value::int(0x0400)),
        ("ads1x1x.ADS1X1X_PGA['1.024V']", Value::int(0x0600)),
        ("ads1x1x.ADS1X1X_PGA['0.512V']", Value::int(0x0800)),
        ("ads1x1x.ADS1X1X_PGA['0.256V']", Value::int(0x0A00)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0000]", Value::float(6.144)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0200]", Value::float(4.096)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0400]", Value::float(2.048)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0600]", Value::float(1.024)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0800]", Value::float(0.512)),
        ("ads1x1x.ADS1X1X_PGA_VALUE[0x0A00]", Value::float(0.256)),
        ("ads1x1x.ADS111X_RESOLUTION", Value::float(32767.0)),
        ("ads1x1x.ADS101X_RESOLUTION", Value::float(2047.0)),
        ("ads1x1x.ADS1X1X_MODE['continuous']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_MODE['single']", Value::int(0x0100)),
        ("ads1x1x.ADS1X1X_COMPARATOR_MODE['TRADITIONAL']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_COMPARATOR_MODE['WINDOW']", Value::int(0x0010)),
        ("ads1x1x.ADS1X1X_COMPARATOR_POLARITY['ACTIVE_LO']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_COMPARATOR_POLARITY['ACTIVE_HI']", Value::int(0x0008)),
        ("ads1x1x.ADS1X1X_COMPARATOR_LATCHING['NON_LATCHING']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_COMPARATOR_LATCHING['LATCHING']", Value::int(0x0004)),
        ("ads1x1x.ADS1X1X_COMPARATOR_QUEUE['QUEUE_1']", Value::int(0x0000)),
        ("ads1x1x.ADS1X1X_COMPARATOR_QUEUE['QUEUE_2']", Value::int(0x0001)),
        ("ads1x1x.ADS1X1X_COMPARATOR_QUEUE['QUEUE_4']", Value::int(0x0002)),
        ("ads1x1x.ADS1X1X_COMPARATOR_QUEUE['QUEUE_NONE']", Value::int(0x0003)),
        ("ads1x1x.ADS1X1_OPERATIONS['SET_MUX']", Value::int(0)),
        ("ads1x1x.ADS1X1_OPERATIONS['READ_CONVERSION']", Value::int(1)),
    ];
    for (expr, actual) in checks {
        check(expr, py.eval(expr), actual.clone());
    }
    for (key, bits) in ADS101X_SAMPLES_PER_SECOND {
        let expr = format!("ads1x1x.ADS101X_SAMPLES_PER_SECOND['{key}']");
        check(&expr, py.eval(&expr), Value::int(bits));
    }
    for (key, bits) in ADS111X_SAMPLES_PER_SECOND {
        let expr = format!("ads1x1x.ADS111X_SAMPLES_PER_SECOND['{key}']");
        check(&expr, py.eval(&expr), Value::int(bits));
    }
    // PGA scalars are stored as value / resolution in the Python dicts.
    for (pga, scalar) in [(0x0000, 6.144), (0x0200, 4.096), (0x0400, 2.048),
                          (0x0600, 1.024), (0x0800, 0.512), (0x0A00, 0.256)] {
        let expr = format!("ads1x1x.ADS111X_PGA_SCALAR[{pga:#06x}]");
        check(&expr, py.eval(&expr), Value::float(scalar / ADS111X_RESOLUTION));
        let expr = format!("ads1x1x.ADS101X_PGA_SCALAR[{pga:#06x}]");
        check(&expr, py.eval(&expr), Value::float(scalar / ADS101X_RESOLUTION));
    }
    // isADS101X / isADS111X over every chip code.
    for code in [0, 3, 4, 5, 13, 14, 15, 99] {
        let expr = format!("ads1x1x.isADS101X({code})");
        check(&expr, py.eval(&expr), Value::Bool(extras::ads1x1x::is_ads101x(code)));
        let expr = format!("ads1x1x.isADS111X({code})");
        check(&expr, py.eval(&expr), Value::Bool(extras::ads1x1x::is_ads111x(code)));
    }
}

#[test]
fn chip_config_reads_and_defaults() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    check("name", py.eval("chip.name"), Value::str(pair.chip.name()));
    check_int("chip", py.eval("chip.chip"), pair.chip.chip());
    check_int("pga default bits", py.eval("chip.pga"), pair.chip.pga());
    check_float("adc_voltage default", py.eval("chip.adc_voltage"), pair.chip.adc_voltage());
    // add_object registration and pin-chip registration.
    let expected = py.eval("p.objects");
    check("printer objects", expected, Value::List(
        pair.printer_state.lock().unwrap().objects.iter().map(|s| Value::str(s)).collect(),
    ));
    let expected = py.eval("[n for (n, _) in p.pins.chips]");
    check("pin chips", expected, Value::List(
        pair.printer_state.lock().unwrap().chips.iter().map(|s| Value::str(s)).collect(),
    ));
    // The chip registers exactly one klippy:connect handler.
    let expected = py.eval("p.handlers");
    check("connect handlers", expected, Value::List(
        pair.printer_state.lock().unwrap().handlers.iter().map(|s| Value::str(s)).collect(),
    ));
    check("mcu name", py.eval("chip.mcu.get_name()"), Value::str(&pair.chip.core.mcu.get_name()));
}

#[test]
fn chip_mcu_identity() {
    // chip.mcu is the MCU behind the I2C handle.
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let expected = py.eval("chip.mcu.get_name()");
    check("mcu name", expected, Value::str(&pair.chip.core.mcu.get_name()));
}

#[test]
fn chip_choice_and_pga_options() {
    let Some(mut py) = oracle() else { return };
    for (name, code) in [
        ("ADS1013", 3), ("ADS1014", 4), ("ADS1015", 5),
        ("ADS1113", 13), ("ADS1114", 14), ("ADS1115", 15),
    ] {
        let pair = fresh_pair(&mut py, &[("chip", name)]);
        check_int(&format!("chip {name}"), py.eval("chip.chip"), pair.chip.chip());
        check_int(&format!("chip code {name}"), py.eval("chip.chip"), code);
    }
    for (pga, bits) in [
        ("6.144V", 0x0000), ("4.096V", 0x0200), ("2.048V", 0x0400),
        ("1.024V", 0x0600), ("0.512V", 0x0800), ("0.256V", 0x0A00),
    ] {
        let pair = fresh_pair(&mut py, &[("chip", "ADS1115"), ("pga", pga)]);
        check_int(&format!("pga {pga}"), py.eval("chip.pga"), pair.chip.pga());
        check_int(&format!("pga bits {pga}"), py.eval("chip.pga"), bits);
    }
    for (opt, val) in [("adc_voltage", "2.5"), ("adc_voltage", "5.0"), ("adc_voltage", "3.3")] {
        let pair = fresh_pair(&mut py, &[("chip", "ADS1115"), (opt, val)]);
        check_float(
            &format!("adc_voltage {val}"),
            py.eval("chip.adc_voltage"),
            pair.chip.adc_voltage(),
        );
    }
}


#[test]
fn chip_config_error_paths() {
    let Some(mut py) = oracle() else { return };
    let cases: &[(&str, &[(&str, &str)])] = &[
        ("missing chip", &[]),
        ("invalid chip", &[("chip", "ADS9999")]),
        ("invalid pga", &[("chip", "ADS1115"), ("pga", "9.0V")]),
        ("adc voltage zero", &[("chip", "ADS1115"), ("adc_voltage", "0")]),
        ("adc voltage negative", &[("chip", "ADS1115"), ("adc_voltage", "-1.5")]),
        ("adc voltage unparsable", &[("chip", "ADS1115"), ("adc_voltage", "abc")]),
    ];
    for (label, opts) in cases {
        let dict = opts.iter().map(|(k, v)| format!("'{k}': '{v}'")).collect::<Vec<_>>().join(", ");
        let expected = py.eval(&format!(
            "new_pair({{{dict}}})"
        ));
        let (printer, _) = FakePrinter::new();
        let printer: Printer = Arc::new(printer);
        let mut config = FakeConfig::new(printer.clone());
        for (k, v) in *opts {
            config = config.with_option(k, v);
        }
        let mcu: Mcu = Arc::new(FakeMcu { printer: printer.clone(), state: Arc::new(Mutex::new(FakeMcuState { est_calls: 0 })) });
        let i2c: McuI2c = Arc::new(FakeI2c { mcu, state: Arc::new(Mutex::new(FakeI2cState::default())) });
        let config: Config = Arc::new(config);
        let actual = ADS1X1X_chip::from_config(&config, i2c).map(|_| ());
        check_err(label, expected, actual);
    }
    // Duplicate pin-chip registration: a second chip with the same name fails.
    py.exec("new_pair({'chip': 'ADS1115'})").expect("py load_config_prefix");
    let expected = py.eval("p.pins.register_chip('temp', chip)");
    let (_, states) = FakePrinter::new();
    let pins_obj: ObjectRef = Arc::new(FakePins { state: states });
    let pins = downcast_arc::<FakePins>(&pins_obj).expect("pins downcast");
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    // pair.chip registered "temp" on its own printer; register that chip
    // once on this fresh pins object, then again to trigger the error.
    let first = pins.register_chip("temp", pair.chip.clone());
    assert!(first.is_ok(), "first register_chip must succeed");
    let second = pins.register_chip("temp", pair.chip.clone());
    check_err("duplicate chip registration", expected, second.map(|_| ()));
}

#[test]
fn setup_pin_computes_pcfg() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    for pin in ["DIFF01", "DIFF03", "DIFF13", "DIFF23", "AIN0", "AIN1", "AIN2", "AIN3"] {
        let expected = py.eval(&format!("chip.setup_pin('adc', {{'pin': '{pin}'}}).pcfg"));
        let mcu = pair.chip.core.mcu.clone();
        let actual = ChipCore::setup_pin_for_chip(
            &pair.chip.core,
            "adc",
            &pin_params(mcu, pin),
        )
        .expect("setup_pin");
        let pcfg = pin_by_name(&pair.chip, pin).pcfg.get();
        check_int(&format!("pcfg {pin}"), expected, pcfg);
        let _ = actual;
    }
    // Every pin registered a klippy:connect handler: 1 chip + 8 pins.
    assert_eq!(pair.printer_state.lock().unwrap().handlers.len(), 9);
    assert_eq!(pair.printer_state.lock().unwrap().handlers.iter().all(|h| h == "klippy:connect"), true);
}

#[test]
fn setup_pin_error_paths() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let mcu = pair.chip.core.mcu.clone();
    let expected = py.eval("chip.setup_pin('adc', {'pin': 'AIN9'})");
    check_err(
        "invalid pin",
        expected,
        ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu.clone(), "AIN9")),
    );
    let expected = py.eval("chip.setup_pin('pwm', {'pin': 'AIN0'})");
    check_err(
        "wrong type",
        expected,
        ChipCore::setup_pin_for_chip(&pair.chip.core, "pwm", &pin_params(mcu.clone(), "AIN0")),
    );
    // Duplicate pin: the second setup fails after creating the pin object.
    let expected_first = py.eval("chip.setup_pin('adc', {'pin': 'AIN1'}).pcfg");
    let first = ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu.clone(), "AIN1"));
    match first {
        Ok(_) => check_int("first AIN1 pcfg", expected_first, pin_by_name(&pair.chip, "AIN1").pcfg.get()),
        Err(e) => panic!("rust first setup failed: {e:?}"),
    }
    let expected = py.eval("chip.setup_pin('adc', {'pin': 'AIN1'})");
    check_err(
        "duplicate pin",
        expected,
        ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, "AIN1")),
    );
    // The failed duplicate still registered a connect handler in Python
    // (the pin object is created first); same on the Rust side.
    let expected = py.eval("p.handlers");
    check("handlers after duplicate", expected, Value::List(
        pair.printer_state.lock().unwrap().handlers.iter().map(|s| Value::str(s)).collect(),
    ));
}

#[test]
fn calculate_sample_rate_tables() {
    let Some(mut py) = oracle() else { return };
    // No pins: sample_rate is 0, so the lowest table entry wins.
    for (chip_name, expected) in [("ADS1115", (8, 0)), ("ADS1015", (128, 0))] {
        let pair = fresh_pair(&mut py, &[("chip", chip_name)]);
        check_pair(
            &format!("no pins {chip_name}"),
            py.eval("chip.calculate_sample_rate()"),
            pair.chip.core.calculate_sample_rate(),
        );
        check_pair(
            &format!("no pins expect {chip_name}"),
            py.eval("chip.calculate_sample_rate()"),
            expected,
        );
    }
    // rate = 1 / min_report_time * pin_count.
    let cases: &[(&str, &str, &[f64])] = &[
        ("one pin slow", "ADS1115", &[0.5]),
        ("two pins boundary", "ADS1115", &[0.5, 0.25]),
        ("two pins ADS101X", "ADS1015", &[0.1, 0.1]),
    ];
    for (label, chip_name, pins) in cases {
        let label = *label;
        let chip_name = *chip_name;
        let pins = *pins;
        let pair = fresh_pair(&mut py, &[("chip", chip_name)]);
        for (i, rt) in pins.iter().enumerate() {
            let pin = ["AIN0", "AIN1"][i];
            py.exec(&format!(
                "pp = chip.setup_pin('adc', {{'pin': '{pin}'}}); pp.setup_adc_sample({rt:?})"
            ))
            .expect("py setup pin");
            let mcu = pair.chip.core.mcu.clone();
            ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, pin))
                .expect("rust setup pin");
            let rpin = pin_by_name(&pair.chip, pin);
            klippy_api::mcu::AdcPinApi::setup_adc_sample(&*rpin, *rt, AdcSampleOpts::default());
        }
        check_pair(
            label,
            py.eval("chip.calculate_sample_rate()"),
            pair.chip.core.calculate_sample_rate(),
        );
    }
}

#[test]
fn sample_rate_exceeded_warns() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    for chip_name in ["ADS1115", "ADS1015"] {
        let pair = fresh_pair(&mut py, &[("chip", chip_name)]);
        py.exec(
            "pp = chip.setup_pin('adc', {'pin': 'AIN0'}); pp.setup_adc_sample(0.0001)",
        )
        .expect("py setup pin");
        let mcu = pair.chip.core.mcu.clone();
        ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, "AIN0"))
            .expect("rust setup pin");
        let rpin = pin_by_name(&pair.chip, "AIN0");
        klippy_api::mcu::AdcPinApi::setup_adc_sample(&*rpin, 0.0001, AdcSampleOpts::default());
        check_pair(
            &format!("exceeded {chip_name}"),
            py.eval("chip.calculate_sample_rate()"),
            pair.chip.core.calculate_sample_rate(),
        );
        check_py_logs(&format!("exceeded logs {chip_name}"), &mut py);
    }
}

#[test]
fn report_time_update_sets_data_rate_and_delay() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    py.exec(
        "pp = chip.setup_pin('adc', {'pin': 'AIN0'}); pp.setup_adc_sample(0.1)",
    )
    .expect("py setup pin");
    let mcu = pair.chip.core.mcu.clone();
    ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, "AIN0"))
        .expect("rust setup pin");
    let rpin = pin_by_name(&pair.chip, "AIN0");
    klippy_api::mcu::AdcPinApi::setup_adc_sample(&*rpin, 0.1, AdcSampleOpts::default());
    check_int("pcfg data rate", py.eval("pp.pcfg"), rpin.pcfg.get());
    check_float("delay", py.eval("chip.delay"), pair.chip.core.delay.get().expect("delay"));
    // A second pin with a faster report changes the shared rate.
    py.exec(
        "pp2 = chip.setup_pin('adc', {'pin': 'AIN1'}); pp2.setup_adc_sample(0.01)",
    )
    .expect("py setup pin 2");
    let mcu = pair.chip.core.mcu.clone();
    ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, "AIN1"))
        .expect("rust setup pin 2");
    let rpin2 = pin_by_name(&pair.chip, "AIN1");
    klippy_api::mcu::AdcPinApi::setup_adc_sample(&*rpin2, 0.01, AdcSampleOpts::default());
    check_int("pcfg1 after faster pin", py.eval("pp.pcfg"), rpin.pcfg.get());
    check_int("pcfg2 after faster pin", py.eval("pp2.pcfg"), rpin2.pcfg.get());
    check_float("delay after faster pin", py.eval("chip.delay"), pair.chip.core.delay.get().expect("delay"));
}

fn setup_one_pin(
    py: &mut pyoracle::Oracle,
    pair: &Pair,
    pin: &str,
    report_time: f64,
) -> Arc<ADS1X1X_pin> {
    py.exec(&format!(
        "pp = chip.setup_pin('adc', {{'pin': '{pin}'}}); pp.setup_adc_sample({report_time:?})"
    ))
    .expect("py setup pin");
    let mcu = pair.chip.core.mcu.clone();
    ChipCore::setup_pin_for_chip(&pair.chip.core, "adc", &pin_params(mcu, pin))
        .expect("rust setup pin");
    let rpin = pin_by_name(&pair.chip, pin);
    klippy_api::mcu::AdcPinApi::setup_adc_sample(&*rpin, report_time, AdcSampleOpts::default());
    rpin
}

fn check_i2c_writes(label: &str, py: &mut pyoracle::Oracle, pair: &Pair) {
    let expected = py.eval("fake_i2c.writes");
    let actual = Value::List(
        pair.i2c_state.lock().unwrap().writes.iter().map(|w| {
            Value::List(w.iter().map(|b| Value::int(*b as i64)).collect())
        }).collect(),
    );
    check(label, expected, actual);
}

#[test]
fn sample_ready_immediately() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
    // Python: chip.sample(pin) with conversion 0x4000.
    let expected = py.eval("chip.sample(pp)");
    let actual = pair.chip.core.sample(&rpin);
    match actual {
        Some(v) => check_int("sample ready", expected, v),
        None => panic!("rust sample returned None"),
    }
    check_i2c_writes("sample writes", &mut py, &pair);
    let expected = py.eval("fake_i2c.reads");
    check_int("read count", expected, pair.i2c_state.lock().unwrap().reads as i64);
    // is_ready reads the config register directly.
    let expected = py.eval("chip.is_ready()");
    check("is_ready", expected, Value::Bool(pair.chip.core.is_ready().expect("is_ready")));
}

#[test]
fn sample_waits_for_ready() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    // Report 0.1 picks 16 sps (delay 1/16), so two 1 ms polls fit inside
    // the conversion window; 0.002 would time out in Python itself.
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.1);
    py.exec("fake_i2c.not_ready = 2").expect("py arm not ready");
    pair.i2c_state.lock().unwrap().not_ready = 2;
    let expected = py.eval("chip.sample(pp)");
    let actual = pair.chip.core.sample(&rpin);
    match actual {
        Some(v) => check_int("sample after wait", expected, v),
        None => panic!("rust sample returned None"),
    }
    check_i2c_writes("writes after wait", &mut py, &pair);
    let expected = py.eval("fake_i2c.reads");
    check_int("reads after wait", expected, pair.i2c_state.lock().unwrap().reads as i64);
}

#[test]
fn sample_timeout_returns_none_and_warns() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
    py.exec("fake_i2c.not_ready = 1000000").expect("py arm never ready");
    pair.i2c_state.lock().unwrap().not_ready = 1_000_000;
    let expected = py.eval("chip.sample(pp)");
    let actual = pair.chip.core.sample(&rpin);
    match (expected, actual) {
        (Ok(v), None) => check("timeout none", Ok(v), Value::Null),
        (Ok(_), Some(v)) => panic!("rust sample returned {v} on timeout"),
        (Err(e), _) => panic!("oracle raised {}: {}", e.class_name, e.message),
    }
    check_py_logs("timeout logs", &mut py);
}

#[test]
fn sample_exception_is_logged() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
    py.exec("fake_i2c.fail_read = 'boom read'").expect("py arm read failure");
    pair.i2c_state.lock().unwrap().fail_read = Some("boom read".to_string());
    let expected = py.eval("chip.sample(pp)");
    let actual = pair.chip.core.sample(&rpin);
    match (expected, actual) {
        (Ok(v), None) => check("exception none", Ok(v), Value::Null),
        (Ok(_), Some(v)) => panic!("rust sample returned {v} on exception"),
        (Err(e), _) => panic!("oracle raised {}: {}", e.class_name, e.message),
    }
    check_py_logs("exception logs", &mut py);
    // A write failure is logged the same way.
    py.exec("fakelog.records = []").expect("py clear logs");
    py.exec("fake_i2c.fail_read = None; fake_i2c.fail_write = 'boom write'").expect("py arm write failure");
    pair.i2c_state.lock().unwrap().fail_read = None;
    pair.i2c_state.lock().unwrap().fail_write = Some("boom write".to_string());
    let expected = py.eval("chip.sample(pp)");
    let actual = pair.chip.core.sample(&rpin);
    match (expected, actual) {
        (Ok(v), None) => check("write exception none", Ok(v), Value::Null),
        (Ok(_), Some(v)) => panic!("rust sample returned {v} on write exception"),
        (Err(e), _) => panic!("oracle raised {}: {}", e.class_name, e.message),
    }
    check_py_logs("write exception logs", &mut py);
}

#[test]
fn chip_connect_resets_device() {
    let Some(mut py) = oracle() else { return };
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    py.exec("chip._handle_connect()").expect("py connect");
    pair.chip.core.handle_connect();
    check_i2c_writes("reset writes", &mut py, &pair);
    // A failed reset is logged, not raised.
    install_log_sink();
    take_logs();
    py.exec("fake_i2c.fail_write = 'no ack'").expect("py arm failure");
    pair.i2c_state.lock().unwrap().fail_write = Some("no ack".to_string());
    py.exec("chip._handle_connect()").expect("py connect fails");
    pair.chip.core.handle_connect();
    check_py_logs("connect failure logs", &mut py);
}

#[test]
fn process_sample_ads111x_and_ads101x() {
    let Some(mut py) = oracle() else { return };
    for (chip_name, resolution, conv) in [
        ("ADS1115", ADS111X_RESOLUTION, 0x4000),
        ("ADS1015", ADS101X_RESOLUTION, 0x4000),
    ] {
        let pair = fresh_pair(&mut py, &[("chip", chip_name)]);
        let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
        py.exec("got = []\npp.setup_adc_callback(lambda lst: got.append(list(lst)))\npp._handle_connect()\nfake_i2c.conversion = 0x4000")
            .expect("py process setup");
        pair.i2c_state.lock().unwrap().conversion = conv;
        let rcvd = Arc::new(Mutex::new(Vec::new()));
        let rcvd2 = rcvd.clone();
        rpin.setup_adc_callback(Box::new(move |samples| rcvd2.lock().unwrap().extend(samples)));
        rpin.handle_connect();
        let expected_ret = py.eval("pp._process_sample(5.0)");
        let actual_ret = rpin.process_sample(5.0);
        check_float(&format!("process ret {chip_name}"), expected_ret, actual_ret);
        let target: f64 = if chip_name == "ADS1015" {
            ((0x4000i64 >> 4) as f64) / resolution * (4.096 / 3.3)
        } else {
            (0x4000i64 as f64) / resolution * (4.096 / 3.3)
        };
        let got_expected = py.eval("[list(x) for x in got[0]]");
        let got_actual = Value::List(vec![Value::List(vec![
            Value::float(1234.5),
            Value::float(target),
        ])]);
        check(&format!("callback {chip_name}"), got_expected, got_actual);
        let expected_last = py.eval("list(pp.get_last_value())");
        check(
            &format!("last value {chip_name}"),
            expected_last,
            Value::List(vec![Value::float(1234.5), Value::float(target)]),
        );
        check_int(
            &format!("invalid count {chip_name}"),
            py.eval("pp.invalid_count"),
            rpin.invalid_count.get(),
        );
        let _ = rcvd;
    }
}

#[test]
fn process_sample_out_of_range_and_shutdown() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
    // range_check_count = 2 via setup_adc_sample defaults plus kwargs.
    py.exec(
        "got = []\npp.setup_adc_callback(lambda lst: got.append(list(lst)))\npp.setup_adc_sample(0.002, range_check_count=2)\npp._handle_connect()\nfake_i2c.conversion = 0x7fff",
    )
    .expect("py process setup");
    pair.i2c_state.lock().unwrap().conversion = 0x7fff;
    let rcvd = Arc::new(Mutex::new(Vec::new()));
    let rcvd2 = rcvd.clone();
    rpin.setup_adc_callback(Box::new(move |samples| rcvd2.lock().unwrap().extend(samples)));
    klippy_api::mcu::AdcPinApi::setup_adc_sample(
        &*rpin,
        0.002,
        AdcSampleOpts { sample_time: 0., sample_count: 1, batch_num: 1, minval: 0., maxval: 1., range_check_count: 2 },
    );
    rpin.handle_connect();
    for i in 0..3 {
        let expected_ret = py.eval("pp._process_sample(1.0)");
        let actual_ret = rpin.process_sample(1.0);
        check_float(&format!("out of range ret {i}"), expected_ret, actual_ret);
        check_int(
            &format!("invalid count {i}"),
            py.eval("pp.invalid_count"),
            rpin.invalid_count.get(),
        );
        let expected_shutdowns = py.eval("p.shutdowns");
        check(
            &format!("shutdowns {i}"),
            expected_shutdowns,
            Value::List(
                pair.printer_state.lock().unwrap().shutdowns.iter().map(|s| Value::str(s)).collect(),
            ),
        );
    }
    check_py_logs("out of range logs", &mut py);
    // A good sample resets the invalid count but publishes anyway.
    py.exec("fake_i2c.conversion = 0x4000").expect("py good conversion");
    pair.i2c_state.lock().unwrap().conversion = 0x4000;
    let expected_ret = py.eval("pp._process_sample(2.0)");
    let actual_ret = rpin.process_sample(2.0);
    check_float("good sample ret", expected_ret, actual_ret);
    check_int("invalid reset", py.eval("pp.invalid_count"), rpin.invalid_count.get());
}

#[test]
fn process_sample_none_sample() {
    let Some(mut py) = oracle() else { return };
    install_log_sink();
    take_logs();
    let pair = fresh_pair(&mut py, &[("chip", "ADS1115")]);
    let rpin = setup_one_pin(&mut py, &pair, "AIN0", 0.002);
    py.exec(
        "got = []\npp.setup_adc_callback(lambda lst: got.append(list(lst)))\npp._handle_connect()\nfake_i2c.not_ready = 1000000",
    )
    .expect("py process setup");
    pair.i2c_state.lock().unwrap().not_ready = 1_000_000;
    let rcvd = Arc::new(Mutex::new(Vec::new()));
    let rcvd2 = rcvd.clone();
    rpin.setup_adc_callback(Box::new(move |samples| rcvd2.lock().unwrap().extend(samples)));
    rpin.handle_connect();
    let expected_ret = py.eval("pp._process_sample(7.0)");
    let actual_ret = rpin.process_sample(7.0);
    check_float("none sample ret", expected_ret, actual_ret);
    check_int("none invalid", py.eval("pp.invalid_count"), rpin.invalid_count.get());
    check_int("none got count", py.eval("len(got)"), rcvd.lock().unwrap().len() as i64);
    let expected_last = py.eval("list(pp.get_last_value())");
    check("none last value", expected_last, Value::List(vec![Value::float(0.), Value::float(0.)]));
    check_py_logs("none sample logs", &mut py);
    check(
        "none shutdown",
        py.eval("p.shutdowns"),
        Value::List(
            pair.printer_state.lock().unwrap().shutdowns.iter().map(|s| Value::str(s)).collect(),
        ),
    );
}

#[test]
fn module_factory_entry_points() {
    let factory = extras::ads1x1x::module_factory();
    assert!(factory.load_config.is_none(), "ads1x1x has no load_config");
    assert!(
        factory.load_config_prefix.is_some(),
        "ads1x1x exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "ads1x1x has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "ads1x1x has no add_printer_objects"
    );
}
