//! Parity tests for `extras::lm75` (port of `klippy/extras/lm75.py`).
//!
//! Expected values come live from the frozen reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`; the tests skip with a message when
//! `$KLIPPY_PYTHON` is unset). The Python side builds a real `lm75.LM75` over
//! a real `configfile.ConfigWrapper` (so the `lm75_report_time` default,
//! bounds and error texts come from `configfile.py`) and minimal fakes of the
//! printer, reactor, MCU and `bus.MCU_I2C_from_config` (monkeypatched, like
//! the reference imports it). The Rust side uses the real bus constructor,
//! resolves MCU and pin services through their interfaces, and scripts its
//! command transport with the same responses as the Python I2C fake.
//!
//! Compared against the oracle: config results and errors, registered printer
//! objects and event handlers, the `bus.MCU_I2C_from_config` arguments,
//! `read_register` / `write_register` I2C traffic and errors,
//! `degrees_from_sample` over a byte grid, the `klippy:connect` chip-id probe
//! (I2C reads and the log line), every `_sample_lm75` path (in-range sample,
//! both shutdown bounds and their messages, read errors, short reads, the
//! missing-callback `AttributeError`; return value by bits, callback
//! `(print_time, temp)` pairs, shutdown messages, log records, `get_status`),
//! and `load_config` registering the `LM75` sensor factory.
//!
//! The Rust reactor clock cannot be scripted, so the port runs first, the
//! `eventtime` its fake MCU saw is fed to the Python reactor's `monotonic`,
//! and the Python run must then produce identical values.
//!
//! The timer registration (`NEVER`) and the `update_timer(NOW)` of
//! `handle_connect` act inside the reactor and are not observable through the
//! contract, so they are not compared. There are no recordings for this unit.

use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use extras::lm75::{module_factory, Lm75, LM75_CHIP_ADDR, LM75_I2C_SPEED};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts as ConfigGetFloatOpts, GetIntOpts,
    GetListsOpts, GetStrOpts,
};
use klippy_api::config::{DeprecateGcodeOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log::{set_sink, Record as LogRecord};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    CommandQueryWrapperApi, CommandWrapperApi, LookupQueryCommandOpts, Mcu, McuApi, Pin,
    PinParams, PinsApi, PinResolver, QuerySendOpts, SendOpts, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::GCell;
use pycompat::text::percent_format;
use pycompat::{PctValue, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::cell::RefCell;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Oracle helpers.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    let Some(py) = pyoracle::Oracle::start() else {
        eprintln!("SKIP: $KLIPPY_PYTHON is unset; pyoracle unavailable");
        return None;
    };
    Some(py)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.lm75".to_string(),
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

/// A Python float expression that round-trips `f` exactly.
fn py_float(f: f64) -> String {
    if f.is_nan() {
        return "float('nan')".to_string();
    }
    if f.is_infinite() {
        return if f > 0.0 { "float('inf')".to_string() } else { "float('-inf')".to_string() };
    }
    format!("{f:?}")
}

fn pct_str(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant lm75 test format")
}

fn ints(bytes: &[u8]) -> Value {
    Value::List(bytes.iter().map(|b| Value::int(*b as i64)).collect())
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
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(self.values.get(option).cloned().or_else(|| opts.default.flatten()))
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        assert!(!self.values.contains_key(option), "bus defaults only");
        Ok(opts.default.flatten())
    }
    /// `ConfigWrapper.getfloat` / `_get_wrapper` (`klippy/configfile.py:29-72`).
    fn getfloat(&self, option: &str, opts: ConfigGetFloatOpts) -> KResult<Option<f64>> {
        let section = self.section.clone();
        let Some(raw) = self.values.get(option).cloned() else {
            return match &opts.default {
                None => Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must be specified",
                    &[PctValue::Str(option.to_string()), PctValue::Str(section)],
                ))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(*v)),
            };
        };
        let v = raw.trim().parse::<f64>().map_err(|_| {
            KlippyError::Config(pct_str(
                "Unable to parse option '%s' in section '%s'",
                &[PctValue::Str(option.to_string()), PctValue::Str(section.clone())],
            ))
        })?;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must have minimum of %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(minval),
                    ],
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(pct_str(
                    "Option '%s' in section '%s' must have maximum of %s",
                    &[
                        PctValue::Str(option.to_string()),
                        PctValue::Str(section),
                        PctValue::Float(maxval),
                    ],
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("FakeConfig.getboolean not exercised")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("FakeConfig.getchoice not exercised")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("FakeConfig.getlists not exercised")
    }
    fn getlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        panic!("FakeConfig.getlist not exercised")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("FakeConfig.getintlist not exercised")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("FakeConfig.getfloatlist not exercised")
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
    objects: GCell<Vec<String>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    shutdowns: GCell<Vec<String>>,
    heaters: GCell<Option<Arc<FakeHeaters>>>,
    loaded: GCell<Vec<String>>,
    services: GCell<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new(duplicate: bool) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            duplicate,
            objects: GCell::new(Vec::new()),
            handlers: GCell::new(Vec::new()),
            shutdowns: GCell::new(Vec::new()),
            heaters: GCell::new(None),
            loaded: GCell::new(Vec::new()),
            services: GCell::new(IndexMap::new()),
        })
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
    fn add_object(&self, name: &str, _obj: ObjectRef) -> KResult<()> {
        if self.duplicate {
            return Err(KlippyError::Config(pct_str(
                "Printer object '%s' already created",
                &[PctValue::Str(name.to_string())],
            )));
        }
        self.objects.borrow_mut().push(name.to_string());
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(Some(self.services.borrow().get(name).expect("registered service").clone()))
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
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
            _ => Err(KlippyError::Config(pct_str(
                "Unknown config object '%s'",
                &[PctValue::Str(section.to_string())],
            ))),
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

/// `printer.lookup_object('heaters')` stand-in: records `add_sensor_factory`.
struct FakeHeaters {
    factories: GCell<Vec<String>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, sensor_type: &str, _factory: SensorFactory) {
        self.factories.borrow_mut().push(sensor_type.to_string());
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

/// MCU transport fake, including the bus configuration callbacks.
struct FakeMcu {
    last_eventtime: GCell<Option<f64>>,
    printer: Printer,
    transport: Arc<FakeI2c>,
    config_callbacks: GCell<Vec<klippy_api::mcu::ConfigCallback>>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        panic!("FakeMcu.get_name not exercised")
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
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_callbacks.borrow_mut().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
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
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        self.last_eventtime.set(Some(eventtime));
        eventtime
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

/// One scripted `i2c_read` outcome, rendered for both sides.
#[derive(Clone, Debug)]
enum Resp {
    /// `{'response': bytes(...)}`
    Bytes(Vec<u8>),
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
            Resp::Bytes(b) => format!(
                "{{'response': bytes([{}])}}",
                b.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ")
            ),
            Resp::NoParams => "None".to_string(),
            Resp::NoKey => "{}".to_string(),
            Resp::Err(msg) => format!("error({msg:?})"),
        }
    }

    fn rust(&self) -> KResult<PyValue> {
        match self {
            Resp::Bytes(b) => {
                let mut map = IndexMap::new();
                map.insert(PyValue::Str("response".to_string()), PyValue::Bytes(b.clone()));
                Ok(PyValue::Dict(map))
            }
            Resp::NoParams => Ok(PyValue::None),
            Resp::NoKey => Ok(PyValue::Dict(IndexMap::new())),
            Resp::Err(msg) => Err(KlippyError::Mcu(msg.to_string())),
        }
    }
}

struct FakeI2c {
    reads: GCell<Vec<(Vec<u8>, i64)>>,
    writes: GCell<Vec<Vec<u8>>>,
    script: GCell<Vec<KResult<PyValue>>>,
}

impl CommandWrapperApi for FakeI2c {
    fn send(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("only acknowledged writes are used")
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        assert_eq!(opts, SendOpts::default());
        let PyValue::Bytes(bytes) = &data[1] else { panic!("write bytes") };
        self.writes.borrow_mut().push(bytes.clone());
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
        self.reads.borrow_mut().push((bytes.clone(), i64::try_from(len.inner()).expect("length fits")));
        self.script.borrow_mut().remove(0)
    }
    fn send_with_preface(
        &self, _cmd: CommandWrapper, _data: Vec<PyValue>,
        _query: Vec<PyValue>, _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("read preface is not used")
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
    fn reset_pin_sharing(&self, _params: PinParams) {
        panic!("hardware bus does not reset pin sharing")
    }
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
    fn get_printer(&self) -> Printer { panic!("config printer not used") }
    fn read_config(&self, _filename: &str) -> KResult<Config> { panic!("config files not read") }
    fn read_main_config(&self) -> KResult<Config> { panic!("main config not read") }
    fn log_config(&self, _config: &Config) { panic!("config not logged") }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("config validation not used")
    }
    fn runtime_warning(&self, _msg: &str) { panic!("runtime warning not used") }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("option deprecation not used")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {
        panic!("gcode deprecation not used")
    }
    fn deprecate_mcu_code(&self, _mcu: &Mcu, feature: &str, _msg: Option<String>) {
        assert_eq!(feature, "i2c_transfer");
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { panic!("config status not used") }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> { panic!("config not saved") }
    fn remove_section(&self, _section: &str) { panic!("config sections not removed") }
}

klippy_api::printer_object!(FakePrinterConfig; dyn PrinterConfigApi);

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging
import configfile
import extras.lm75 as lm75
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
    def __init__(self):
        self.now = 0.5
        self.timers = []
        self.updates = []
    def monotonic(self):
        return self.now
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(waketime)
        return ('timer', len(self.timers))
    def update_timer(self, timer, waketime):
        self.updates.append((timer, waketime))

class FakeMcu:
    def estimated_print_time(self, eventtime):
        return eventtime

class FakeI2C:
    def __init__(self, mcu):
        self.mcu = mcu
        self.reads = []
        self.writes = []
        self.script = []
    def get_mcu(self):
        return self.mcu
    def i2c_read(self, write, read_len, minclock=0, reqclock=0, retry=True):
        self.reads.append((list(write), read_len))
        item = self.script.pop(0)
        if isinstance(item, Exception):
            raise item
        return item
    def i2c_write(self, data, minclock=0, reqclock=0, retry=True):
        self.writes.append(list(data))

class FakePrinter:
    def __init__(self, reactor, duplicate):
        self.reactor = reactor
        self.duplicate = duplicate
        self.objects = []
        self.handlers = []
        self.shutdowns = []
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        if self.duplicate:
            raise configfile.error("Printer object '%s' already created" % (name,))
        self.objects.append(name)
    def register_event_handler(self, event, cb):
        self.handlers.append(event)
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)

from_config_calls = []

def build(text, section, duplicate=False):
    global printer, i2c, mcu, reactor, obj, cbs
    del capture.records[:]
    del from_config_calls[:]
    cbs = []
    reactor = FakeReactor()
    mcu = FakeMcu()
    i2c = FakeI2C(mcu)
    printer = FakePrinter(reactor, duplicate)
    fc = RawConfigParser()
    fc.optionxform = str.lower
    fc.read_string(text)
    wrapper = configfile.ConfigWrapper(printer, fc, {}, section)
    def fake_from_config(config, default_addr=None, default_speed=None, **kwargs):
        from_config_calls.append((default_addr, default_speed))
        return i2c
    orig = lm75.bus.MCU_I2C_from_config
    lm75.bus.MCU_I2C_from_config = fake_from_config
    try:
        obj = lm75.LM75(wrapper)
    finally:
        lm75.bus.MCU_I2C_from_config = orig
    return None

def set_callback():
    obj.setup_callback(lambda print_time, temp: cbs.append((print_time, temp)))

def trace():
    result = ([(r[0], r[1]) for r in i2c.reads], list(i2c.writes),
              list(printer.shutdowns), list(cbs), list(capture.records),
              obj.get_status(0.))
    del capture.records[:]
    return result

class FakeHeaters:
    def __init__(self):
        self.factories = []
    def add_sensor_factory(self, sensor_type, factory):
        self.factories.append((sensor_type, factory is lm75.LM75))

class FakeLoadPrinter:
    def __init__(self, heaters):
        self.heaters = heaters
        self.loaded = []
    def load_object(self, config, section, default=None):
        self.loaded.append(section)
        if section == 'heaters' and self.heaters is not None:
            return self.heaters
        raise configfile.error("Unknown config object '%s'" % (section,))

def load(text, section, with_heaters):
    global load_printer, load_heaters
    load_heaters = FakeHeaters() if with_heaters else None
    load_printer = FakeLoadPrinter(load_heaters)
    fc = RawConfigParser()
    fc.optionxform = str.lower
    fc.read_string(text)
    wrapper = configfile.ConfigWrapper(load_printer, fc, {}, section)
    return lm75.load_config(wrapper)
"#;

// ---------------------------------------------------------------------------
// Rust-side construction and traces.
// ---------------------------------------------------------------------------

struct Built {
    printer: Arc<FakePrinter>,
    i2c: Arc<FakeI2c>,
    mcu: Arc<FakeMcu>,
    cbs: Arc<GCell<Vec<(f64, f64)>>>,
    obj: Arc<Lm75>,
}

fn parse_kv(text: &str) -> Vec<(&str, &str)> {
    let mut out = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('[') {
            continue;
        }
        if let Some((k, v)) = line.split_once('=') {
            out.push((k.trim(), v.trim()));
        }
    }
    out
}

fn build_rust(text: &str, section: &str, duplicate: bool) -> KResult<Built> {
    install_sink();
    take_logs();
    let printer = FakePrinter::new(duplicate);
    let i2c = Arc::new(FakeI2c {
        reads: GCell::new(Vec::new()),
        writes: GCell::new(Vec::new()),
        script: GCell::new(Vec::new()),
    });
    let mcu = Arc::new(FakeMcu {
        last_eventtime: GCell::new(None),
        printer: printer.clone(),
        transport: i2c.clone(),
        config_callbacks: GCell::new(Vec::new()),
    });
    printer.services.borrow_mut().insert("mcu".into(), mcu.clone());
    printer.services.borrow_mut().insert("pins".into(), Arc::new(FakePins));
    printer.services.borrow_mut().insert("configfile".into(), Arc::new(FakePrinterConfig));
    let printer_api: Printer = printer.clone();
    let config: Config = FakeConfig::new(printer_api, section, &parse_kv(text));
    let obj = Lm75::new(&config)?;
    for mut callback in std::mem::take(&mut *mcu.config_callbacks.borrow_mut()) {
        callback()?;
    }
    // The Python bus fake does not register its own connect handler.
    drop(printer.handlers.borrow_mut().remove(0));
    Ok(Built { printer, i2c, mcu, cbs: Arc::new(GCell::new(Vec::new())), obj })
}

impl Built {
    fn set_callback(&self) {
        let cbs = self.cbs.clone();
        let callback: TemperatureCallback =
            Box::new(move |print_time, temp| cbs.borrow_mut().push((print_time, temp)));
        self.obj.setup_callback(callback);
    }

    fn push(&self, resp: &Resp) {
        self.i2c.script.borrow_mut().push(resp.rust());
    }

    /// Mirror of the Python `trace()`.
    fn trace(&self) -> Value {
        let reads = Value::List(
            self.i2c
                .reads
                .borrow()
                .iter()
                .map(|(w, n)| Value::Tuple(vec![ints(w), Value::int(*n)]))
                .collect(),
        );
        let writes = Value::List(self.i2c.writes.borrow().iter().map(|w| ints(w)).collect());
        let shutdowns = Value::List(
            self.printer.shutdowns.borrow().iter().map(|m| Value::str(m)).collect(),
        );
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
        Value::Tuple(vec![reads, writes, shutdowns, cbs, logs, self.status()])
    }

    fn status(&self) -> Value {
        use klippy_api::object::PrinterObject;
        let st = self.obj.get_status(0.).expect("lm75 has get_status");
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
}

/// Build both sides from the same config text.
fn build_both(py: &mut pyoracle::Oracle, text: &str, section: &str) -> Built {
    py.exec(&format!("build({text:?}, {section:?})")).expect("py build");
    build_rust(text, section, false).expect("port build")
}

const CFG: &str = "[temperature_sensor foo]\nlm75_report_time = 1.25\n";
const SECTION: &str = "temperature_sensor foo";

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn config_and_registration_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let cases: &[(&str, &str)] = &[
        ("[temperature_sensor foo]\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 1.25\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 0.5\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 0.8\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 30\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 0.49\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = 0\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = -1\n", "temperature_sensor foo"),
        ("[temperature_sensor foo]\nlm75_report_time = abc\n", "temperature_sensor foo"),
        ("[temperature_sensor  bar baz]\nlm75_report_time = 2.5\n", "temperature_sensor  bar baz"),
        ("[chamber]\n", "chamber"),
    ];
    for (i, (text, section)) in cases.iter().enumerate() {
        let label = format!("config[{i}] {text:?}");
        let py_result = py.exec(&format!("build({text:?}, {section:?})"));
        let rust_result = build_rust(text, section, false);
        match (py_result, rust_result) {
            (Ok(_), Ok(b)) => {
                let expected = py
                    .eval("(obj.report_time, obj.get_report_time_delta(), obj.name)")
                    .expect("py params");
                let delta = TemperatureSensorApi::get_report_time_delta(&*b.obj);
                check(
                    &label,
                    Ok(expected),
                    Value::Tuple(vec![
                        Value::float(delta),
                        Value::float(delta),
                        Value::str(b.obj.name()),
                    ]),
                );
                let expected = py
                    .eval(
                        "(list(printer.objects), list(printer.handlers), \
                         list(from_config_calls), obj.get_status(0.))",
                    )
                    .expect("py registration");
                let actual = Value::Tuple(vec![
                    Value::List(
                        b.printer.objects.borrow().iter().map(|n| Value::str(n)).collect(),
                    ),
                    Value::List(
                        b.printer.handlers.borrow().iter().map(|(e, _)| Value::str(e)).collect(),
                    ),
                    Value::List(vec![Value::Tuple(vec![
                        Value::int(LM75_CHIP_ADDR),
                        Value::int(LM75_I2C_SPEED),
                    ])]),
                    b.status(),
                ]);
                check(&format!("{label} registration"), Ok(expected), actual);
            }
            (Ok(_), Err(e)) => panic!("{label}: port raised {}: {}", e.class_name(), e.message()),
            (Err(py_exc), Err(e)) => {
                assert_eq!(e.class_name(), py_exc.class_name, "{label}: error class");
                assert_eq!(e.message(), py_exc.message, "{label}: error message");
            }
            (Err(py_exc), Ok(_)) => {
                panic!("{label}: oracle raised {}: {}, port built", py_exc.class_name, py_exc.message)
            }
        }
    }
}

#[test]
fn duplicate_object_error_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let py_result = py.exec(&format!("build({CFG:?}, {SECTION:?}, True)"));
    let rust_result = build_rust(CFG, SECTION, true).map(|_| Value::Null);
    check_result("duplicate", py_result.map(|_| Value::Null), rust_result);
}

#[test]
fn degrees_from_sample_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    let his = [0u8, 1, 25, 26, 100, 127, 128, 200, 255];
    let los = [0u8, 1, 0x3f, 0x7f, 0x80, 0x81, 0xc0, 0xff];
    for &hi in &his {
        for &lo in &los {
            let label = format!("degrees [{hi}, {lo}]");
            let expected = py.eval(&format!("obj.degrees_from_sample(bytearray([{hi}, {lo}]))"));
            let actual = b.obj.degrees_from_sample(&[hi, lo]).map(Value::float);
            check_result(&label, expected, actual);
        }
    }
    // Extra bytes are ignored; short reads raise IndexError.
    let expected = py.eval("obj.degrees_from_sample(bytearray([10, 0x80, 99]))");
    check_result(
        "degrees long",
        expected,
        b.obj.degrees_from_sample(&[10, 0x80, 99]).map(Value::float),
    );
    for short in [&[][..], &[5][..]] {
        let items = short.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", ");
        let expected = py.eval(&format!("obj.degrees_from_sample(bytearray([{items}]))"));
        check_result(
            &format!("degrees short {short:?}"),
            expected,
            b.obj.degrees_from_sample(short).map(Value::float),
        );
    }
}

#[test]
fn read_and_write_register_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    let reads: &[(&str, i64, Resp)] = &[
        ("TEMP", 2, Resp::Bytes(vec![25, 0x80])),
        ("CONF", 1, Resp::Bytes(vec![0])),
        ("THYST", 2, Resp::Bytes(vec![75, 0])),
        ("TOS", 2, Resp::Bytes(vec![80, 0])),
        ("PRODID", 1, Resp::Bytes(vec![0xa1])),
        ("PRODID", 1, Resp::Bytes(vec![])),
        ("TEMP", 2, Resp::NoParams),
        ("TEMP", 2, Resp::NoKey),
        ("TEMP", 2, Resp::Err("MCU 'mcu' I2C request to addr 72 reports error NACK")),
        ("BOGUS", 1, Resp::Bytes(vec![1])),
    ];
    for (i, (reg, len, resp)) in reads.iter().enumerate() {
        let label = format!("read[{i}] {reg} {resp:?}");
        py.exec(&format!("i2c.script.append({})", resp.py())).expect("py script");
        b.push(resp);
        let expected = py.eval(&format!("list(obj.read_register({reg:?}, {len}))"));
        let actual = b.obj.read_register(reg, *len).map(|v| ints(&v));
        check_result(&label, expected, actual);
        check(&format!("{label} trace"), py.eval("trace()"), b.trace());
        // `BOGUS` raises before the transport is touched, so its response stays queued.
        if *reg == "BOGUS" {
            b.i2c.script.borrow_mut().clear();
            py.exec("del i2c.script[:]").expect("py clear");
        }
    }

    let writes: &[(&str, &str, &[u8])] = &[
        ("CONF", "0x01", &[0x01]),
        ("TOS", "[0x50, 0x00]", &[0x50, 0x00]),
        ("THYST", "[0x4b]", &[0x4b]),
        ("TEMP", "[]", &[]),
        ("BOGUS", "1", &[1]),
    ];
    for (i, (reg, py_data, data)) in writes.iter().enumerate() {
        let label = format!("write[{i}] {reg}");
        let expected = py.exec(&format!("obj.write_register({reg:?}, {py_data})"));
        let actual = b.obj.write_register(reg, data).map(|_| Value::Null);
        check_result(&label, expected.map(|_| Value::Null), actual);
        check(&format!("{label} trace"), py.eval("trace()"), b.trace());
    }
}

#[test]
fn connect_chip_id_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let cases: &[Resp] = &[
        Resp::Bytes(vec![0xa1]),
        Resp::Bytes(vec![0x20]),
        Resp::Bytes(vec![0x00]),
        Resp::Bytes(vec![0xff]),
        Resp::Bytes(vec![0xab, 0xcd]),
        Resp::Bytes(vec![]),
        Resp::NoParams,
        Resp::NoKey,
        Resp::Err("Timeout on wait for 'i2c_read_response' response"),
    ];
    for (i, resp) in cases.iter().enumerate() {
        let label = format!("connect[{i}] {resp:?}");
        let b = build_both(&mut py, CFG, SECTION);
        py.exec(&format!("i2c.script.append({})", resp.py())).expect("py script");
        b.push(resp);
        // Python fires the handler the printer saw registered.
        py.exec("obj.handle_connect()").expect("py connect");
        let results = b.printer.send_event("klippy:connect", &[]).expect("send klippy:connect");
        assert_eq!(results.len(), 1, "{label}: one klippy:connect handler");
        check(&label, py.eval("trace()"), b.trace());
    }
}

/// One `_sample_lm75` call on both sides, run on objects that already hold
/// their callback and temperature limits.
fn sample_step(py: &mut pyoracle::Oracle, b: &Built, label: &str, resp: &Resp) {
    py.exec(&format!("i2c.script.append({})", resp.py())).expect("py script");
    b.push(resp);
    b.mcu.last_eventtime.set(None);
    let actual = b.obj.sample_lm75(1.0);
    if let Some(measured) = b.mcu.last_eventtime.get() {
        py.exec(&format!("reactor.now = {}", py_float(measured))).expect("py clock");
    }
    let expected = py.eval("obj._sample_lm75(1.0)");
    check_result(label, expected, actual.map(Value::float));
    check(&format!("{label} trace"), py.eval("trace()"), b.trace());
}

#[test]
fn sample_paths_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    b.set_callback();
    py.exec("set_callback()").expect("py callback");
    b.obj.setup_minmax(10.0, 100.0).expect("rust minmax");
    py.exec("obj.setup_minmax(10.0, 100.0)").expect("py minmax");

    let steps: &[Resp] = &[
        Resp::Bytes(vec![25, 0x80]),
        Resp::Bytes(vec![5, 0x00]),
        Resp::Bytes(vec![9, 0x80]),
        Resp::Bytes(vec![101, 0x00]),
        Resp::Bytes(vec![100, 0x80]),
        Resp::Bytes(vec![10, 0x00]),
        Resp::Bytes(vec![100, 0x00]),
        Resp::Bytes(vec![255, 0xff]),
        Resp::Bytes(vec![0, 0x00]),
        Resp::Bytes(vec![30, 0x7f, 0xaa]),
        Resp::Err("MCU 'mcu' I2C request to addr 72 reports error NACK"),
        Resp::Bytes(vec![42, 0x80]),
        Resp::NoParams,
        Resp::Bytes(vec![43, 0x00]),
        Resp::NoKey,
        Resp::Bytes(vec![30]),
        Resp::Bytes(vec![]),
        Resp::Bytes(vec![50, 0x80]),
    ];
    for (i, resp) in steps.iter().enumerate() {
        sample_step(&mut py, &b, &format!("sample[{i}] {resp:?}"), resp);
    }
}

#[test]
fn sample_limits_before_setup_minmax_match_python() {
    // min_temp = max_temp = 0.0 until the heater calls setup_minmax, so any
    // positive temperature shuts down.
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    b.set_callback();
    py.exec("set_callback()").expect("py callback");
    sample_step(&mut py, &b, "default limits positive", &Resp::Bytes(vec![1, 0x80]));
    sample_step(&mut py, &b, "default limits zero", &Resp::Bytes(vec![0, 0x00]));
}

#[test]
fn sample_without_callback_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    b.obj.setup_minmax(10.0, 100.0).expect("rust minmax");
    py.exec("obj.setup_minmax(10.0, 100.0)").expect("py minmax");
    // In range, out of range (shutdown precedes the AttributeError), then a
    // read error, which returns before the callback is looked up.
    sample_step(&mut py, &b, "no callback in range", &Resp::Bytes(vec![25, 0x80]));
    sample_step(&mut py, &b, "no callback out of range", &Resp::Bytes(vec![150, 0x00]));
    sample_step(&mut py, &b, "no callback read error", &Resp::NoParams);
}

#[test]
fn setup_callback_replaces_previous_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    b.obj.setup_minmax(0.0, 120.0).expect("rust minmax");
    py.exec("obj.setup_minmax(0.0, 120.0)").expect("py minmax");
    b.set_callback();
    py.exec("set_callback()").expect("py callback");
    sample_step(&mut py, &b, "first callback", &Resp::Bytes(vec![20, 0x00]));
    // A second `setup_callback` replaces the first; both sides then report to
    // the new callback only.
    let second: Arc<GCell<Vec<(f64, f64)>>> = Arc::new(GCell::new(Vec::new()));
    let sink = second.clone();
    b.obj.setup_callback(Box::new(move |pt, t| sink.borrow_mut().push((pt, t))));
    py.exec("second = []\nobj.setup_callback(lambda pt, t: second.append((pt, t)))")
        .expect("py second callback");
    sample_step(&mut py, &b, "second callback", &Resp::Bytes(vec![21, 0x80]));
    let expected = py.eval("list(second)").expect("py second");
    let actual = Value::List(
        second
            .borrow()
            .iter()
            .map(|(pt, t)| Value::Tuple(vec![Value::float(*pt), Value::float(*t)]))
            .collect(),
    );
    check("second callback calls", Ok(expected), actual);
}

#[test]
fn status_rounding_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let b = build_both(&mut py, CFG, SECTION);
    b.set_callback();
    py.exec("set_callback()").expect("py callback");
    b.obj.setup_minmax(-300.0, 1000.0).expect("rust minmax");
    py.exec("obj.setup_minmax(-300.0, 1000.0)").expect("py minmax");
    check("status initial", py.eval("obj.get_status(0.)"), b.status());
    for (i, resp) in [
        Resp::Bytes(vec![0, 0x00]),
        Resp::Bytes(vec![255, 0x80]),
        Resp::Bytes(vec![37, 0xff]),
        Resp::Bytes(vec![22, 0x7f]),
    ]
    .iter()
    .enumerate()
    {
        sample_step(&mut py, &b, &format!("status[{i}]"), resp);
        check(&format!("status[{i}] dict"), py.eval("obj.get_status(123.)"), b.status());
    }
}

#[test]
fn load_config_registers_sensor_factory() {
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py SETUP");
    let text = "[temperature_sensor foo]\n";
    for with_heaters in [true, false] {
        let label = format!("load_config with_heaters={with_heaters}");
        let flag = if with_heaters { "True" } else { "False" };
        let py_result = py.exec(&format!("load_result = load({text:?}, {SECTION:?}, {flag})"));
        let printer = FakePrinter::new(false);
        if with_heaters {
            *printer.heaters.borrow_mut() =
                Some(Arc::new(FakeHeaters { factories: GCell::new(Vec::new()) }));
        }
        let printer_api: Printer = printer.clone();
        let config: Config = FakeConfig::new(printer_api, SECTION, &parse_kv(text));
        let load_config = module_factory().load_config.expect("load_config entry");
        let rust_result = load_config(&config).map(|obj| {
            assert!(obj.is_none(), "{label}: load_config returns None");
            Value::Null
        });
        check_result(&label, py_result.map(|_| Value::Null), rust_result);
        let expected = py.eval("list(load_printer.loaded)").expect("py loaded");
        let actual = Value::List(
            printer.loaded.borrow().iter().map(|s| Value::str(s)).collect(),
        );
        check(&format!("{label} loaded"), Ok(expected), actual);
        if with_heaters {
            let expected = py
                .eval("[t for t, is_lm75 in load_heaters.factories]")
                .expect("py factories");
            let factories = printer.heaters.cloned().expect("heaters present").factories.cloned();
            check(
                &format!("{label} factories"),
                Ok(expected),
                Value::List(factories.iter().map(|s| Value::str(s)).collect()),
            );
            let is_class = py
                .eval("all(is_lm75 for t, is_lm75 in load_heaters.factories)")
                .expect("py factory identity");
            check(&format!("{label} factory is LM75"), Ok(is_class), Value::Bool(true));
        }
    }
}

#[test]
fn module_factory_has_load_config_only() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}
