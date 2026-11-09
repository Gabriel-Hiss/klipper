//! Parity tests for `extras::temperature_mcu`
//! (port of `klippy/extras/temperature_mcu.py`).
//!
//! Every expected value comes from `pyoracle` (CPython 3.8.2 with the frozen
//! `klippy/`): the reference `PrinterTemperatureMCU` is built over the real
//! `configfile.ConfigWrapper` (so option defaults, bounds and config error
//! texts are the reference ones) plus fakes for the printer, pins, MCU,
//! `query_adc`, `error_mcu` and `heaters`; the port is built over fakes of
//! the same contract traits with the same inputs. Both sides are driven
//! through `handle_mcu_identify` (directly and through the registered
//! `klippy:mcu_identify` handler), `setup_minmax`, the ADC callback and the
//! diagnostics clarify callback, and the resulting slope, base temperature,
//! `debug_read` traffic, `setup_adc_sample` arguments, log lines, exception
//! class/message and `query_adc`/`error_mcu` registrations are compared
//! exactly (floats by bits).
//!
//! `PrinterTemperatureMCU` implements the real `heaters::TemperatureSensorApi`
//! and registers its factory through the real `heaters::PrinterHeatersApi`, so
//! the test drives the same traits `heaters` uses.
//!
//! There are no recordings under `tests/fixtures/recordings/temperature_mcu/`,
//! so expected values come from the oracle alone.

use extras::error_mcu::{ClarifyCallback, PrinterMCUErrorApi};
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::query_adc::QueryAdcApi;
use extras::temperature_mcu::{
    module_factory, PrinterTemperatureMCU, RANGE_CHECK_COUNT, REPORT_TIME, SAMPLE_COUNT,
    SAMPLE_TIME,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::{
    AdcCallback, AdcPin, AdcPinApi, AdcSampleOpts, AddConfigCmdOpts, AsyncResponseWrapper, Chip,
    CommandQueue, CommandQueryWrapper, CommandQueryWrapperApi, CommandWrapper, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver,
    PinsApi, QuerySendOpts, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::text::float_str;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::cell::RefCell;
use std::sync::{Arc, Once};

// ---------------------------------------------------------------------------
// Python-side fakes and helpers, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging
import extras.temperature_mcu as temperature_mcu
from configfile import ConfigWrapper, sentinel

SENTINEL = sentinel

class FakePrinter:
    config_error = configparser.Error
    command_error = configparser.Error
    error = configparser.Error
    def __init__(self, start_args=None):
        self.objects = {}
        self.start_args = {'debugoutput': None} if start_args is None else start_args
        self.handlers = {}
    def get_start_args(self):
        return self.start_args
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)

class LoggingConfig(ConfigWrapper):
    def __init__(self, *args):
        ConfigWrapper.__init__(self, *args)
        self.reads = []
    def get(self, option, default=SENTINEL, note_valid=True):
        self.reads.append(('get', option))
        return ConfigWrapper.get(self, option, default, note_valid=note_valid)
    def getfloat(self, option, default=SENTINEL, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self.reads.append(('getfloat', option))
        return ConfigWrapper.getfloat(self, option, default, minval=minval,
                                      maxval=maxval, above=above, below=below,
                                      note_valid=note_valid)

class FakeMCU_ADC:
    def __init__(self, mcu):
        self._mcu = mcu
        self.cb = None
        self.samples = []
        self.last = (0., 0.)
    def get_mcu(self):
        return self._mcu
    def setup_adc_callback(self, cb):
        self.cb = cb
    def setup_adc_sample(self, report_time, sample_time=0., sample_count=1,
                         batch_num=1, minval=0., maxval=1., range_check_count=0):
        self.samples.append((report_time, sample_time, sample_count, batch_num,
                             minval, maxval, range_check_count))
    def get_last_value(self):
        return self.last

class FakeCmd:
    def __init__(self, values=None):
        self.values = dict(values or {})
        self.reads = []
    def send(self, data):
        self.reads.append(tuple(data))
        return {'val': self.values[data[1]]}

class FakeMCU:
    def __init__(self, name="mcu", mcu_type="samd21", values=None):
        self._name = name
        self._constants = {"MCU": mcu_type}
        self.cmd = FakeCmd(values)
        self.lookups = []
    def get_name(self):
        return self._name
    def get_constants(self):
        return dict(self._constants)
    def lookup_query_command(self, fmt, resp, **kw):
        self.lookups.append((fmt, resp))
        return self.cmd

class FakePins:
    def __init__(self, adc):
        self.adc = adc
        self.calls = []
    def setup_pin(self, pin_type, pin_desc):
        self.calls.append((pin_type, pin_desc))
        return self.adc

class FakeQueryADC:
    def __init__(self):
        self.registered = []
    def register_adc(self, name, adc):
        self.registered.append((name, adc))

class FakeErrorMCU:
    def __init__(self):
        self.clarifies = []
    def add_clarify(self, msg, cb):
        self.clarifies.append((msg, cb))

class FakeHeaters:
    def __init__(self):
        self.factories = {}
        self.order = []
    def add_sensor_factory(self, name, cls):
        self.factories[name] = cls
        self.order.append(name)

def build_sensor(floats=None, strs=None, mcu_type="samd21", mcu_name="mcu",
                 values=None, start_args=None):
    p = FakePrinter(start_args)
    mcu = FakeMCU(mcu_name, mcu_type, values)
    adc = FakeMCU_ADC(mcu)
    pins = FakePins(adc)
    query = FakeQueryADC()
    errmcu = FakeErrorMCU()
    heaters = FakeHeaters()
    p.objects.update({'pins': pins, 'query_adc': query, 'error_mcu': errmcu,
                      'heaters': heaters})
    fc = configparser.RawConfigParser()
    section = 'temperature_mcu'
    fc.add_section(section)
    for k, v in (strs or {}).items():
        fc.set(section, k, v)
    for k, v in (floats or {}).items():
        fc.set(section, k, repr(v))
    c = LoggingConfig(p, fc, {}, section)
    W = {'p': p, 'c': c, 'mcu': mcu, 'adc': adc, 'pins': pins, 'query': query,
         'errmcu': errmcu, 'heaters': heaters, 's': None}
    W['s'] = temperature_mcu.PrinterTemperatureMCU(c)
    return W

log_lines = []
class _Capture(logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)
"#;

// ---------------------------------------------------------------------------
// Rust-side log capture. The sink is process wide but writes the calling
// thread's buffer, so parallel tests do not see each other's records.
// ---------------------------------------------------------------------------

thread_local! {
    static LOGS: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        log::set_sink(Box::new(|rec| {
            LOGS.with(|lines| lines.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

fn take_logs() -> Vec<String> {
    LOGS.with(|lines| std::mem::take(&mut *lines.borrow_mut()))
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: kos::sync::Mutex<Vec<(String, ObjectRef)>>,
    handlers: kos::sync::Mutex<Vec<(String, EventCallback)>>,
    start_args: PyValue,
}

impl FakePrinter {
    fn new(start_args: PyValue) -> FakePrinter {
        FakePrinter {
            objects: kos::sync::Mutex::new(Vec::new()),
            handlers: kos::sync::Mutex::new(Vec::new()),
            start_args,
        }
    }

    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().push((name.to_string(), obj));
    }

    fn event_names(&self) -> Vec<String> {
        self.handlers.lock().iter().map(|(name, _)| name.clone()).collect()
    }

    /// Invokes every handler registered for `event`, like `send_event`.
    fn fire(&self, event: &str) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        for (name, handler) in handlers.iter_mut() {
            if name == event {
                handler(&[])?;
            }
        }
        Ok(())
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.start_args.clone()
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
        if self.objects.lock().iter().any(|(n, _)| n == name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self
            .objects
            .lock()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, o)| o.clone());
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        match module {
            None => objects.clone(),
            Some(module) => objects
                .iter()
                .filter(|(n, _)| n == module || n.starts_with(&format!("{module} ")))
                .cloned()
                .collect(),
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
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.fire(event)?;
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

/// `ConfigWrapper` (`klippy/configfile.py:18`) over an option map, with the
/// `_get_wrapper` texts for the two getters the module reads.
struct FakeConfig {
    printer: Printer,
    section: String,
    str_values: IndexMap<String, String>,
    float_values: IndexMap<String, f64>,
    reads: kos::sync::Mutex<Vec<(String, String)>>,
}

impl FakeConfig {
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
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
        self.reads.lock().push(("get".to_string(), option.to_string()));
        if let Some(v) = self.str_values.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            Some(Some(v)) => Ok(Some(v)),
            Some(None) => Ok(None),
            None => Err(self.missing(option)),
        }
    }
    fn getint(&self, option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(self.missing(option))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.lock().push(("getfloat".to_string(), option.to_string()));
        let v = match self.float_values.get(option).copied() {
            Some(v) => v,
            None => match opts.default {
                Some(Some(v)) => return Ok(Some(v)),
                Some(None) => return Ok(None),
                None => return Err(self.missing(option)),
            },
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_str(maxval)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(self.missing(option))
    }
    fn getchoice(&self, option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(self.missing(option))
    }
    fn getlists(&self, option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(self.missing(option))
    }
    fn getlist(&self, option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(self.missing(option))
    }
    fn getintlist(&self, option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(self.missing(option))
    }
    fn getfloatlist(&self, option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(self.missing(option))
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used")
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

/// `FakeCmd` from the Python setup: answers `debug_read` with `{'val': ...}`.
struct FakeDebugCmd {
    values: IndexMap<i64, i64>,
    reads: kos::sync::Mutex<Vec<(i64, i64)>>,
}

fn int_arg(value: &PyValue) -> i64 {
    match value {
        PyValue::Int(n) => i64::try_from(n.inner()).expect("debug_read argument fits i64"),
        other => panic!("debug_read argument must be an int, got {other:?}"),
    }
}

impl CommandQueryWrapperApi for FakeDebugCmd {
    fn send(&self, data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        let order = int_arg(&data[0]);
        let addr = int_arg(&data[1]);
        self.reads.lock().push((order, addr));
        let val = *self.values.get(&addr).expect("debug_read value for address");
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("val".to_string()), PyValue::Int(PyInt::from_i64(val)));
        Ok(PyValue::Dict(map))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used")
    }
}

struct FakeMcu {
    name: String,
    mcu_type: String,
    cmd: CommandQueryWrapper,
    lookups: kos::sync::Mutex<Vec<(String, String)>>,
    printer: kos::sync::Mutex<Option<Printer>>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.lock().clone().expect("printer is set")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("not used")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("not used")
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        panic!("not used")
    }
    fn request_move_queue_slot(&self) {
        panic!("not used")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("not used")
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        panic!("not used")
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("not used")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("not used")
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("not used")
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.lookups.lock().push((msgformat.to_string(), respformat.to_string()));
        Ok(self.cmd.clone())
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("not used")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        panic!("not used")
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("not used")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("not used")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        panic!("not used")
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        let mut constants = IndexMap::new();
        constants.insert("MCU".to_string(), self.mcu_type.clone());
        constants
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("not used")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("not used")
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("not used")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("not used")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("not used")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("not used")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("not used")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("not used")
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("not used")
    }
}

struct FakeAdcState {
    callback: Option<AdcCallback>,
    samples: Vec<(f64, AdcSampleOpts)>,
    last: (f64, f64),
}

struct FakeAdc {
    mcu: Mcu,
    state: kos::sync::Mutex<FakeAdcState>,
}

impl FakeAdc {
    fn new(mcu: Mcu) -> FakeAdc {
        FakeAdc {
            mcu,
            state: kos::sync::Mutex::new(FakeAdcState {
                callback: None,
                samples: Vec::new(),
                last: (0., 0.),
            }),
        }
    }

    fn has_callback(&self) -> bool {
        self.state.lock().callback.is_some()
    }

    fn samples(&self) -> Vec<(f64, AdcSampleOpts)> {
        self.state.lock().samples.clone()
    }

    fn set_last(&self, last: (f64, f64)) {
        self.state.lock().last = last;
    }

    fn fire_callback(&self, samples: Vec<(f64, f64)>) {
        let mut state = self.state.lock();
        if let Some(callback) = state.callback.as_mut() {
            callback(samples);
        }
    }
}

impl AdcPinApi for FakeAdc {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.state.lock().samples.push((report_time, opts));
    }
    fn setup_adc_callback(&self, callback: AdcCallback) {
        self.state.lock().callback = Some(callback);
    }
    fn get_last_value(&self) -> (f64, f64) {
        self.state.lock().last
    }
}

struct FakePins {
    adc: AdcPin,
    calls: kos::sync::Mutex<Vec<(String, String)>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used: {pin_desc}")
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin is not used: {pin_desc}")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.calls.lock().push((pin_type.to_string(), pin_desc.to_string()));
        Ok(Pin::Adc(self.adc.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("not used")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("not used")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("not used")
    }
}

struct FakeQueryAdc {
    registered: kos::sync::Mutex<Vec<String>>,
}

impl QueryAdcApi for FakeQueryAdc {
    fn register_adc(&self, name: &str, _mcu_adc: AdcPin) {
        self.registered.lock().push(name.to_string());
    }
}

struct FakeErrorMcu {
    clarifies: kos::sync::Mutex<Vec<(String, ClarifyCallback)>>,
}

impl PrinterMCUErrorApi for FakeErrorMcu {
    fn add_clarify(&self, message: &str, callback: ClarifyCallback) {
        self.clarifies.lock().push((message.to_string(), callback));
    }
}

/// The `heaters` object: records the sensor factory `load_config` registers
/// and serves it back, so the test can build a sensor through the same path
/// `heaters.setup_sensor` uses.
struct FakeHeaters {
    factories: kos::sync::Mutex<Vec<(String, SensorFactory)>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, factory: SensorFactory) {
        self.factories.lock().push((sensor_type.to_string(), factory));
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        panic!("get_all_heaters is not used")
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        panic!("lookup_heater is not used")
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);
klippy_api::printer_object!(FakeQueryAdc; dyn QueryAdcApi);
klippy_api::printer_object!(FakeErrorMcu; dyn PrinterMCUErrorApi);
klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct World {
    printer: Arc<FakePrinter>,
    config: Config,
    fake_config: Arc<FakeConfig>,
    fake_adc: Arc<FakeAdc>,
    fake_mcu: Arc<FakeMcu>,
    fake_cmd: Arc<FakeDebugCmd>,
    fake_pins: Arc<FakePins>,
    fake_query: Arc<FakeQueryAdc>,
    fake_err: Arc<FakeErrorMcu>,
    fake_heaters: Arc<FakeHeaters>,
}

fn debugoutput_args(debugoutput: Option<&str>) -> PyValue {
    let mut args = IndexMap::new();
    args.insert(
        PyValue::Str("debugoutput".to_string()),
        match debugoutput {
            Some(path) => PyValue::Str(path.to_string()),
            None => PyValue::None,
        },
    );
    PyValue::Dict(args)
}

fn build_world(
    float_values: &[(&str, f64)],
    str_values: &[(&str, &str)],
    mcu_type: &str,
    mcu_name: &str,
    values: &[(i64, i64)],
    debugoutput: Option<&str>,
) -> World {
    install_log_sink();
    let fake_printer = Arc::new(FakePrinter::new(debugoutput_args(debugoutput)));
    let printer: Printer = fake_printer.clone();
    let fake_cmd = Arc::new(FakeDebugCmd {
        values: values.iter().cloned().collect(),
        reads: kos::sync::Mutex::new(Vec::new()),
    });
    let cmd: CommandQueryWrapper = fake_cmd.clone();
    let fake_mcu = Arc::new(FakeMcu {
        name: mcu_name.to_string(),
        mcu_type: mcu_type.to_string(),
        cmd,
        lookups: kos::sync::Mutex::new(Vec::new()),
        printer: kos::sync::Mutex::new(Some(printer.clone())),
    });
    let mcu: Mcu = fake_mcu.clone();
    let fake_adc = Arc::new(FakeAdc::new(mcu));
    let adc: AdcPin = fake_adc.clone();
    let fake_pins = Arc::new(FakePins {
        adc,
        calls: kos::sync::Mutex::new(Vec::new()),
    });
    let fake_query = Arc::new(FakeQueryAdc { registered: kos::sync::Mutex::new(Vec::new()) });
    let fake_err = Arc::new(FakeErrorMcu { clarifies: kos::sync::Mutex::new(Vec::new()) });
    let fake_heaters = Arc::new(FakeHeaters { factories: kos::sync::Mutex::new(Vec::new()) });
    fake_printer.add("pins", fake_pins.clone());
    fake_printer.add("query_adc", fake_query.clone());
    fake_printer.add("error_mcu", fake_err.clone());
    fake_printer.add("heaters", fake_heaters.clone());
    let fake_config = Arc::new(FakeConfig {
        printer: printer.clone(),
        section: "temperature_mcu".to_string(),
        str_values: str_values
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
        float_values: float_values.iter().map(|(k, v)| (k.to_string(), *v)).collect(),
        reads: kos::sync::Mutex::new(Vec::new()),
    });
    let config: Config = fake_config.clone();
    World {
        printer: fake_printer,
        config,
        fake_config,
        fake_adc,
        fake_mcu,
        fake_cmd,
        fake_pins,
        fake_query,
        fake_err,
        fake_heaters,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("temperature_mcu fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping temperature_mcu oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run from the repo checkout"
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
        unit: "temperature_mcu".to_string(),
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

fn check_float_opt(label: &str, expected: pyoracle::OracleResult, actual: Option<f64>) {
    match actual {
        Some(v) => check_float(label, expected, v),
        None => check(label, expected, Value::Null),
    }
}

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but the port must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: the port returned Ok but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn py_float_dict(values: &[(&str, f64)]) -> String {
    let parts: Vec<String> = values.iter().map(|(k, v)| format!("{k:?}: {v:?}")).collect();
    format!("{{{}}}", parts.join(", "))
}

fn py_str_dict(values: &[(&str, &str)]) -> String {
    let parts: Vec<String> = values.iter().map(|(k, v)| format!("{k:?}: {v:?}")).collect();
    format!("{{{}}}", parts.join(", "))
}

fn py_values(values: &[(i64, i64)]) -> String {
    let parts: Vec<String> = values.iter().map(|(addr, val)| format!("{addr}: {val}")).collect();
    format!("{{{}}}", parts.join(", "))
}

fn rust_reads(world: &World) -> Value {
    let reads: Vec<Value> = world
        .fake_config
        .reads
        .lock()
        .iter()
        .map(|(kind, option)| Value::Tuple(vec![Value::str(kind), Value::str(option)]))
        .collect();
    Value::List(reads)
}

fn rust_pin_calls(world: &World) -> Value {
    let calls: Vec<Value> = world
        .fake_pins
        .calls
        .lock()
        .iter()
        .map(|(pin_type, pin_desc)| Value::Tuple(vec![Value::str(pin_type), Value::str(pin_desc)]))
        .collect();
    Value::List(calls)
}

fn sample_value(report_time: f64, opts: &AdcSampleOpts) -> Value {
    Value::Tuple(vec![
        Value::float(report_time),
        Value::float(opts.sample_time),
        Value::int(opts.sample_count),
        Value::int(opts.batch_num),
        Value::float(opts.minval),
        Value::float(opts.maxval),
        Value::int(opts.range_check_count),
    ])
}

fn rust_samples(world: &World) -> Value {
    let samples: Vec<Value> = world
        .fake_adc
        .samples()
        .iter()
        .map(|(report_time, opts)| sample_value(*report_time, opts))
        .collect();
    Value::List(samples)
}

fn rust_logs() -> Value {
    Value::List(take_logs().iter().map(|line| Value::str(line)).collect())
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check_float("SAMPLE_TIME", py.eval("temperature_mcu.SAMPLE_TIME"), SAMPLE_TIME);
    check("SAMPLE_COUNT", py.eval("temperature_mcu.SAMPLE_COUNT"), Value::int(SAMPLE_COUNT));
    check_float("REPORT_TIME", py.eval("temperature_mcu.REPORT_TIME"), REPORT_TIME);
    check(
        "RANGE_CHECK_COUNT",
        py.eval("temperature_mcu.RANGE_CHECK_COUNT"),
        Value::int(RANGE_CHECK_COUNT),
    );
}

#[test]
fn init_registers_pin_adc_diagnostics_and_handler() {
    let Some(mut py) = oracle() else { return };
    let floats = [("sensor_temperature1", 50.0), ("sensor_adc1", 0.25)];
    py.exec(&format!("W = build_sensor({})", py_float_dict(&floats))).expect("py build");
    let expected_reads = py.eval("W['c'].reads");
    let expected_pins = py.eval("W['pins'].calls");
    let expected_query = py.eval("[name for name, _adc in W['query'].registered]");
    let expected_clarify = py.eval("[msg for msg, _cb in W['errmcu'].clarifies]");
    let expected_handlers = py.eval("sorted(W['p'].handlers.keys())");
    let expected_samples = py.eval("W['adc'].samples");
    let expected_temp1 = py.eval("W['s'].temp1");
    let expected_adc1 = py.eval("W['s'].adc1");
    let expected_temp2 = py.eval("W['s'].temp2");
    let expected_base = py.eval("W['s'].base_temperature");
    let expected_slope = py.eval("W['s'].slope");

    let world = build_world(&floats, &[], "samd21", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    assert_eq!(rust.base_temperature(), None);
    assert_eq!(rust.slope(), None);
    check("init reads", expected_reads, rust_reads(&world));
    check("init pin", expected_pins, rust_pin_calls(&world));
    // The ADC pseudo pin of `sensor_mcu` is `ADC_TEMPERATURE`.
    assert_eq!(
        world.fake_pins.calls.lock().clone(),
        vec![("adc".to_string(), "mcu:ADC_TEMPERATURE".to_string())]
    );
    let registered = world.fake_query.registered.lock().clone();
    check(
        "init query_adc",
        expected_query,
        Value::List(registered.iter().map(|name| Value::str(name)).collect()),
    );
    assert_eq!(registered, vec!["temperature_mcu".to_string()]);
    let clarifies = world.fake_err.clarifies.lock().clone();
    check(
        "init error_mcu",
        expected_clarify,
        Value::List(clarifies.iter().map(|(msg, _cb)| Value::str(msg)).collect()),
    );
    check(
        "init handlers",
        expected_handlers,
        Value::List(
            world
                .printer
                .event_names()
                .iter()
                .map(|name| Value::str(name))
                .collect(),
        ),
    );
    assert!(world.fake_adc.has_callback());
    check("init samples", expected_samples, rust_samples(&world));
    check_float_opt("init temp1", expected_temp1, rust.temp1());
    check_float_opt("init adc1", expected_adc1, rust.adc1());
    check_float_opt("init temp2", expected_temp2, rust.temp2());
    check_float_opt("init base", expected_base, rust.base_temperature());
    check_float_opt("init slope", expected_slope, rust.slope());
}

#[test]
fn init_reads_both_temps_and_sensor_mcu() {
    let Some(mut py) = oracle() else { return };
    let floats = [
        ("sensor_temperature1", 50.0),
        ("sensor_adc1", 0.25),
        ("sensor_temperature2", 100.0),
        ("sensor_adc2", 0.75),
    ];
    let strs = [("sensor_mcu", "mcu2")];
    py.exec(&format!(
        "W = build_sensor({}, {}, mcu_name='mcu2')",
        py_float_dict(&floats),
        py_str_dict(&strs)
    ))
    .expect("py build");
    let expected_reads = py.eval("W['c'].reads");
    let expected_pins = py.eval("W['pins'].calls");
    let expected_temp2 = py.eval("W['s'].temp2");
    let expected_adc2 = py.eval("W['s'].adc2");

    let world = build_world(&floats, &strs, "samd21", "mcu2", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    check("both temps reads", expected_reads, rust_reads(&world));
    check("both temps pin", expected_pins, rust_pin_calls(&world));
    assert_eq!(
        world.fake_pins.calls.lock().clone(),
        vec![("adc".to_string(), "mcu2:ADC_TEMPERATURE".to_string())]
    );
    check_float_opt("both temps temp1", py.eval("W['s'].temp1"), rust.temp1());
    check_float_opt("both temps adc1", py.eval("W['s'].adc1"), rust.adc1());
    check_float_opt("both temps temp2", expected_temp2, rust.temp2());
    check_float_opt("both temps adc2", expected_adc2, rust.adc2());
}

#[test]
fn init_debugoutput_skips_handler_and_samples_with_defaults() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70', start_args={'debugoutput': 'out'})")
        .expect("py build");
    let expected_handlers = py.eval("sorted(W['p'].handlers.keys())");
    let expected_samples = py.eval("W['adc'].samples");
    let expected_delta = py.eval("W['s'].get_report_time_delta()");

    let world = build_world(&[], &[], "same70", "mcu", &[], Some("out"));
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    check(
        "debugoutput handlers",
        expected_handlers,
        Value::List(
            world.printer.event_names().iter().map(|name| Value::str(name)).collect(),
        ),
    );
    assert!(world.printer.event_names().is_empty());
    // The callback is registered before the debugoutput early return.
    assert!(world.fake_adc.has_callback());
    // `setup_adc_sample(REPORT_TIME, SAMPLE_TIME, SAMPLE_COUNT)`: the
    // remaining arguments keep their `klippy/mcu.py:570` defaults.
    check("debugoutput samples", expected_samples, rust_samples(&world));
    check_float("debugoutput delta", expected_delta, rust.get_report_time_delta());
    let samples = world.fake_adc.samples();
    assert_eq!(samples.len(), 1);
    assert_eq!(samples[0].0, REPORT_TIME);
}

#[test]
fn init_missing_adc1_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let floats = [("sensor_temperature1", 50.0)];
    let expected = py.eval(&format!("build_sensor({})", py_float_dict(&floats)));
    let world = build_world(&floats, &[], "samd21", "mcu", &[], None);
    check_err("missing adc1", expected, PrinterTemperatureMCU::new(&world.config).map(|_| ()));
}

#[test]
fn init_adc1_out_of_range_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    for adc in [1.5, -0.5] {
        let floats = [("sensor_temperature1", 50.0), ("sensor_adc1", adc)];
        let expected = py.eval(&format!("build_sensor({})", py_float_dict(&floats)));
        let world = build_world(&floats, &[], "samd21", "mcu", &[], None);
        check_err(
            &format!("adc1 out of range {adc:?}"),
            expected,
            PrinterTemperatureMCU::new(&world.config).map(|_| ()),
        );
    }
}

#[test]
fn init_missing_temperature2_keeps_adc2_unset() {
    let Some(mut py) = oracle() else { return };
    // `sensor_adc2` is only read when `sensor_temperature2` is present, so a
    // stray `sensor_adc2` stays unread.
    let floats = [("sensor_temperature1", 50.0), ("sensor_adc1", 0.25), ("sensor_adc2", 0.5)];
    py.exec(&format!("W = build_sensor({})", py_float_dict(&floats))).expect("py build");
    let expected_reads = py.eval("W['c'].reads");
    let expected_adc2 = py.eval("W['s'].adc2");
    let world = build_world(&floats, &[], "samd21", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    check("no temp2 reads", expected_reads, rust_reads(&world));
    check_float_opt("no temp2 adc2", expected_adc2, rust.adc2());
}

/// One full `handle_mcu_identify` run on both sides for one MCU type.
fn identify_case(
    label: &str,
    py: &mut pyoracle::Oracle,
    floats: &[(&str, f64)],
    mcu_type: &str,
    values: &[(i64, i64)],
) {
    py.exec(&format!(
        "W = build_sensor({}, mcu_type={mcu_type:?}, values={})",
        py_float_dict(floats),
        py_values(values)
    ))
    .expect("py build");
    py.exec("log_lines.clear()").expect("clear py logs");
    take_logs();
    let expected_result = py.eval("W['s'].handle_mcu_identify()");
    let expected_slope = py.eval("W['s'].slope");
    let expected_base = py.eval("W['s'].base_temperature");
    let expected_reads = py.eval("W['mcu'].cmd.reads");
    let expected_lookups = py.eval("W['mcu'].lookups");
    let expected_samples = py.eval("W['adc'].samples");
    let expected_log = py.eval("list(log_lines)");

    let world = build_world(floats, &[], mcu_type, "mcu", values, None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    let rust_result = rust.handle_mcu_identify();
    let ok = match (expected_result, rust_result) {
        (Ok(_), Ok(())) => true,
        (Err(e), Err(r)) => {
            assert_eq!(r.class_name(), e.class_name.as_str(), "{label}: error class");
            assert_eq!(r.message(), e.message.as_str(), "{label}: error message");
            false
        }
        (Ok(_), Err(r)) => panic!("{label}: port raised {}: {}", r.class_name(), r.message()),
        (Err(e), Ok(())) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let rust_reads_value: Vec<Value> = world
        .fake_cmd
        .reads
        .lock()
        .iter()
        .map(|(order, addr)| Value::Tuple(vec![Value::int(*order), Value::int(*addr)]))
        .collect();
    check(
        &format!("{label} reads"),
        expected_reads,
        Value::List(rust_reads_value),
    );
    let lookups: Vec<Value> = world
        .fake_mcu
        .lookups
        .lock()
        .iter()
        .map(|(fmt, resp)| Value::Tuple(vec![Value::str(fmt), Value::str(resp)]))
        .collect();
    check(&format!("{label} lookups"), expected_lookups, Value::List(lookups));
    check(&format!("{label} logs"), expected_log, rust_logs());
    if !ok {
        return;
    }
    check_float_opt(&format!("{label} slope"), expected_slope, rust.slope());
    check_float_opt(&format!("{label} base"), expected_base, rust.base_temperature());
    check(&format!("{label} samples"), expected_samples, rust_samples(&world));
}

#[test]
fn identify_constant_slope_mcus() {
    let Some(mut py) = oracle() else { return };
    for (label, mcu_type) in [
        ("rp2040", "rp2040"),
        ("sam3", "sam3x8e"),
        ("sam4", "sam4s8"),
        ("same70", "same70"),
        ("stm32f1", "stm32f1"),
        ("stm32f2", "stm32f2"),
    ] {
        identify_case(label, &mut py, &[], mcu_type, &[]);
    }
}

#[test]
fn identify_reading_mcus() {
    let Some(mut py) = oracle() else { return };
    // SAMD21/D51 calibration words: room 25.0C, hot 85.0C, 1.0V both
    // (`cal1`: room temp bits 0-7, hot temp bits 12-19; `cal2`: hot 1v
    // bits 0-7, room adc bits 8-19, hot adc bits 20-31).
    let cal1: i64 = 25 | (85 << 12);
    let cal2: i64 = (2048 << 8) | (4095 << 20);
    identify_case(
        "samd21",
        &mut py,
        &[],
        "samd21",
        &[(0x0080_6030, cal1), (0x0080_6034, cal2)],
    );
    identify_case(
        "samd51",
        &mut py,
        &[],
        "samd51",
        &[(0x0080_0100, cal1), (0x0080_0104, cal2)],
    );
    identify_case(
        "same5",
        &mut py,
        &[],
        "same5",
        &[(0x0080_0100, cal1), (0x0080_0104, cal2)],
    );
    identify_case(
        "stm32f4",
        &mut py,
        &[],
        "stm32f4",
        &[(0x1FFF_7A2C, 1372), (0x1FFF_7A2E, 2718)],
    );
    identify_case(
        "stm32f042",
        &mut py,
        &[],
        "stm32f042",
        &[(0x1FFF_F7B8, 1372), (0x1FFF_F7C2, 2718)],
    );
    identify_case(
        "stm32f072",
        &mut py,
        &[],
        "stm32f072",
        &[(0x1FFF_F7B8, 1372), (0x1FFF_F7C2, 2718)],
    );
    identify_case("stm32f070", &mut py, &[], "stm32f070", &[(0x1FFF_F7B8, 1372)]);
    identify_case(
        "stm32g0",
        &mut py,
        &[],
        "stm32g0",
        &[(0x1FFF_75A8, 1372), (0x1FFF_75CA, 2718)],
    );
    identify_case(
        "stm32g4",
        &mut py,
        &[],
        "stm32g4",
        &[(0x1FFF_75A8, 1372), (0x1FFF_75CA, 2718)],
    );
    identify_case(
        "stm32l4",
        &mut py,
        &[],
        "stm32l4",
        &[(0x1FFF_75A8, 1372), (0x1FFF_75CA, 2718)],
    );
    identify_case(
        "stm32h723",
        &mut py,
        &[],
        "stm32h723",
        &[(0x1FF1_E820, 1372), (0x1FF1_E840, 2718)],
    );
    identify_case(
        "stm32h7",
        &mut py,
        &[],
        "stm32h7",
        &[(0x1FF1_E820, 1372), (0x1FF1_E840, 2718)],
    );
}

#[test]
fn identify_unknown_mcu_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    identify_case("unknown gd32", &mut py, &[], "gd32f303", &[]);
    identify_case("unknown empty", &mut py, &[], "", &[]);
    identify_case("unknown rp3", &mut py, &[], "rp3", &[]);
    identify_case("unknown sam", &mut py, &[], "sam", &[]);
}

#[test]
fn identify_manual_override_recomputes_slope_and_base() {
    let Some(mut py) = oracle() else { return };
    identify_case(
        "override both",
        &mut py,
        &[
            ("sensor_temperature1", 50.0),
            ("sensor_adc1", 0.25),
            ("sensor_temperature2", 100.0),
            ("sensor_adc2", 0.75),
        ],
        "same70",
        &[],
    );
    identify_case(
        "override temp1 only",
        &mut py,
        &[("sensor_temperature1", 50.0), ("sensor_adc1", 0.25)],
        "same70",
        &[],
    );
}

#[test]
fn identify_after_setup_minmax_computes_adc_range() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.exec("W['s'].setup_minmax(10.0, 200.0)").expect("py setup_minmax");
    py.exec("log_lines.clear()").expect("clear py logs");
    take_logs();
    let expected_result = py.eval("W['s'].handle_mcu_identify()");
    let expected_samples = py.eval("W['adc'].samples");
    let expected_base = py.eval("W['s'].base_temperature");
    let expected_slope = py.eval("W['s'].slope");
    let expected_log = py.eval("list(log_lines)");

    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    rust.setup_minmax(10.0, 200.0);
    let rust_result = rust.handle_mcu_identify();
    assert!(expected_result.is_ok(), "oracle handle_mcu_identify must succeed");
    rust_result.expect("port handle_mcu_identify");
    check("minmax samples", expected_samples, rust_samples(&world));
    check_float_opt("minmax base", expected_base, rust.base_temperature());
    check_float_opt("minmax slope", expected_slope, rust.slope());
    check("minmax logs", expected_log, rust_logs());
    // One `setup_adc_sample` call, carrying the derived range.
    assert_eq!(world.fake_adc.samples().len(), 1);
}

#[test]
fn identify_from_registered_event_handler() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.exec("log_lines.clear()").expect("clear py logs");
    take_logs();
    let expected_result = py.eval(
        "[cb() for cb in W['p'].handlers['klippy:mcu_identify']]",
    );
    let expected_base = py.eval("W['s'].base_temperature");
    let expected_slope = py.eval("W['s'].slope");
    let expected_log = py.eval("list(log_lines)");

    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    world.printer.fire("klippy:mcu_identify").expect("port event");
    assert!(expected_result.is_ok(), "oracle handler dispatch must succeed");
    check_float_opt("event base", expected_base, rust.base_temperature());
    check_float_opt("event slope", expected_slope, rust.slope());
    check("event logs", expected_log, rust_logs());
}

#[test]
fn calc_methods_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.eval("W['s'].handle_mcu_identify()").expect("py identify");
    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    rust.handle_mcu_identify().expect("port identify");
    for adc in [0.0, 0.25, 0.5, 1.0] {
        check_float(
            &format!("calc_temp {adc:?}"),
            py.eval(&format!("W['s'].calc_temp({adc:?})")),
            rust.calc_temp(adc),
        );
    }
    for temp in [0.0, 25.0, 27.0, 200.0] {
        check_float(
            &format!("calc_adc {temp:?}"),
            py.eval(&format!("W['s'].calc_adc({temp:?})")),
            rust.calc_adc(temp),
        );
    }
    for (temp, adc) in [(25.0, 0.5), (27.0, 0.706 / 3.3), (30.0, 0.72 / 3.3)] {
        check_float(
            &format!("calc_base {temp:?}/{adc:?}"),
            py.eval(&format!("W['s'].calc_base({temp:?}, {adc:?})")),
            rust.calc_base(temp, adc),
        );
    }
}

#[test]
fn adc_callback_forwards_last_sample_with_offset() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.eval("W['s'].handle_mcu_identify()").expect("py identify");
    py.exec(
        "calls = []\nW['s'].setup_callback(lambda t, temp: calls.append((t, temp)))\nW['s'].adc_callback([(1.0, 0.1), (2.0, 0.5)])",
    )
    .expect("py adc callback");
    let expected = py.eval("calls");

    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    rust.handle_mcu_identify().expect("port identify");
    let captured: Arc<kos::sync::Mutex<Vec<(f64, f64)>>> =
        Arc::new(kos::sync::Mutex::new(Vec::new()));
    {
        let sink = captured.clone();
        let callback: TemperatureCallback =
            Box::new(move |time, temp| sink.lock().push((time, temp)));
        rust.setup_callback(callback);
    }
    // Drive the callback the port registered on the ADC pin, like the MCU
    // does when a sample batch arrives.
    world.fake_adc.fire_callback(vec![(1.0, 0.1), (2.0, 0.5)]);
    let calls: Vec<Value> = captured
        .lock()
        .iter()
        .map(|(time, temp)| Value::Tuple(vec![Value::float(*time), Value::float(*temp)]))
        .collect();
    check("adc_callback calls", expected, Value::List(calls));
}

#[test]
fn clarify_range_messages_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.eval("W['s'].handle_mcu_identify()").expect("py identify");
    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    rust.handle_mcu_identify().expect("port identify");

    // No sample yet (`last_read_time == 0`) -> None on both sides.
    let expected_none = py.eval("W['errmcu'].clarifies[0][1]('ADC out of range', {})");
    let clarify = world.fake_err.clarifies.lock()[0].1.clone();
    let actual_none = clarify("ADC out of range", &PyValue::None).expect("clarify");
    assert_eq!(actual_none, None);
    check("clarify no sample", expected_none, Value::Null);

    // Out-of-range last value -> the formatted sensor message.
    py.exec("W['adc'].last = (100.0, 5.0)").expect("py last value");
    world.fake_adc.set_last((100.0, 5.0));
    let expected = py.eval("W['errmcu'].clarifies[0][1]('ADC out of range', {})");
    let actual = clarify("ADC out of range", &PyValue::None).expect("clarify");
    check("clarify out of range", expected, Value::str(&actual.expect("message")));

    // In-range last value -> None.
    py.exec("W['adc'].last = (100.0, 0.5)").expect("py last value");
    world.fake_adc.set_last((100.0, 0.5));
    py.exec("W['s'].diag_helper.setup_diag_minmax(0.0, 300.0, 0.0, 1.0)").expect("py widen");
    rust.diag_helper().setup_diag_minmax(0.0, 300.0, 0.0, 1.0);
    let expected_in_range = py.eval("W['errmcu'].clarifies[0][1]('ADC out of range', {})");
    let actual_in_range = clarify("ADC out of range", &PyValue::None).expect("clarify");
    assert_eq!(actual_in_range, None);
    check("clarify in range", expected_in_range, Value::Null);
}

#[test]
fn load_config_registers_temperature_mcu_factory() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    py.eval("temperature_mcu.load_config(W['c'])").expect("py load_config");
    let expected_order = py.eval("W['heaters'].order");
    let expected_delta = py.eval("temperature_mcu.PrinterTemperatureMCU(W['c']).get_report_time_delta()");

    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let factory: ModuleFactory = module_factory();
    let result = factory.load_config.expect("load_config entry point")(&world.config);
    assert!(matches!(result, Ok(None)), "load_config returns None like Python");
    let factories = world.fake_heaters.factories.lock().clone();
    check(
        "load_config order",
        expected_order,
        Value::List(factories.iter().map(|(name, _)| Value::str(name)).collect()),
    );
    assert_eq!(factories.len(), 1);
    assert_eq!(factories[0].0, "temperature_mcu");
    // The registered factory builds a working sensor through the real
    // `heaters::TemperatureSensorApi` (as `heaters.setup_sensor` would).
    let sensor: Arc<dyn TemperatureSensorApi> = (factories[0].1)(&world.config).expect("factory");
    check_float("factory report delta", expected_delta, sensor.get_report_time_delta());
    sensor.setup_minmax(0.0, 100.0);
}

#[test]
fn read16_before_identify_raises_attribute_error() {
    let Some(mut py) = oracle() else { return };
    py.exec("W = build_sensor(mcu_type='same70')").expect("py build");
    let expected = py.eval("W['s'].read16(0x1FFF7A2C)");
    let world = build_world(&[], &[], "same70", "mcu", &[], None);
    let rust = PrinterTemperatureMCU::new(&world.config).expect("port new");
    check_err("read16 before identify", expected, rust.read16(0x1FFF_7A2C));
}
