//! Parity tests for `klippy/extras/verify_heater.py`.
//!
//! The Rust port is driven through fakes of the contract traits
//! (`PrinterApi`, `ConfigApi`, `GCodeApi`, `PinsApi`, `McuApi`,
//! `TemperatureSensorApi`, `PrinterHeatersApi`) with the same inputs
//! given to the reference: the real `HeaterCheck` class is built in the
//! live CPython oracle (`pyoracle`) with equivalent Python fakes
//! (`PY_SETUP` below), and every expected value is an oracle
//! `eval`/`exec` result compared with `recording::assert_equal`
//! (floats by hex + repr spelling, errors by class and message,
//! return values, state attributes, log lines, shutdown messages).
//! The oracle is skipped when `$KLIPPY_PYTHON` is unset (outside the
//! oracle image); the remaining tests check structural invariants that
//! embed no reference numbers.
//!
//! The `heaters` fake returns a real `heaters::Heater` (the trait's
//! `lookup_heater` returns the concrete type); it is driven with
//! `set_temp` + `temperature_callback` so that `get_temp` yields the
//! scripted `(temp, target)` pair exactly like the Python fake.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test verify_heater
//! ```

use std::path::PathBuf;
use std::sync::Arc;

use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::verify_heater::{module_factory, HeaterCheck, HINT_THERMAL};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts as CfgGetFloatOpts,
    GetFloatListOpts, GetIntOpts as CfgGetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts,
    GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, Pin,
    PinParams, PinResolver, PinsApi, ParsePinOpts, PwmPin, PwmPinApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text as pytext;
use pycompat::PyValue;
use reactor::Reactor;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..")
}

fn klippy_extras_path() -> String {
    repo_root()
        .join("klippy")
        .join("extras")
        .to_string_lossy()
        .replace('\\', "/")
}

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// Tests assert presence of records, which is race-free with per-thread
// buffers.
// ---------------------------------------------------------------------------

type LogVec = Vec<(String, String)>;

thread_local! {
    static LOGS: std::cell::RefCell<LogVec> = const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn captured_logs() -> LogVec {
    install_log_sink();
    LOGS.with(|logs| logs.borrow().clone())
}

fn has_log(level: &str, msg: &str) -> bool {
    captured_logs().iter().any(|(l, m)| l == level && m == msg)
}

// ---------------------------------------------------------------------------
// Fakes of the contract traits.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct FakePrinterInner {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<EventCallback>>,
    shutdown_msgs: Vec<String>,
    load_calls: Vec<String>,
    debugoutput: Option<String>,
}

struct FakePrinter {
    reactor: Reactor,
    inner: GCell<FakePrinterInner>,
}

impl FakePrinter {
    fn new(debugoutput: Option<String>, reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            reactor,
            inner: GCell::new(FakePrinterInner {
                debugoutput,
                ..Default::default()
            }),
        })
    }

    fn register_object(&self, name: &str, obj: ObjectRef) {
        self.inner
            .borrow_mut()
            .objects
            .insert(name.to_string(), obj);
    }

    fn handler_keys(&self) -> Vec<String> {
        self.inner.borrow().handlers.keys().cloned().collect()
    }

    /// Dispatches to the registered handlers without holding the inner
    /// borrow (Python handlers may re-enter the printer). Handlers
    /// registered during the dispatch are not invoked by it.
    fn dispatch(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut handlers = {
            let mut inner = self.inner.borrow_mut();
            inner.handlers.shift_remove(event).unwrap_or_default()
        };
        let mut rets = Vec::new();
        let mut err: Option<KlippyError> = None;
        for cb in handlers.iter_mut() {
            match cb(params) {
                Ok(r) => rets.push(r),
                Err(e) => {
                    err = Some(e);
                    break;
                }
            }
        }
        // Re-register: originals (minus any the error path skipped keep
        // their place) — simply put the original list back ahead of any
        // handlers added during dispatch.
        {
            let mut inner = self.inner.borrow_mut();
            let mut added = inner.handlers.shift_remove(event).unwrap_or_default();
            handlers.append(&mut added);
            inner.handlers.insert(event.to_string(), handlers);
        }
        match err {
            Some(e) => Err(e),
            None => Ok(rets),
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut map = IndexMap::new();
        let v = match &self.inner.borrow().debugoutput {
            Some(s) => PyValue::Str(s.clone()),
            None => PyValue::None,
        };
        map.insert(PyValue::Str("debugoutput".to_string()), v);
        PyValue::Dict(map)
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.inner.borrow_mut();
        if inner.objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        inner.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let inner = self.inner.borrow();
        if let Some(o) = inner.objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{}'",
                name
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.inner
            .borrow()
            .objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let mut inner = self.inner.borrow_mut();
        inner.load_calls.push(section.to_string());
        if let Some(o) = inner.objects.get(section).cloned() {
            return Ok(Some(o));
        }
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.inner.borrow_mut().shutdown_msgs.push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.inner.borrow_mut().shutdown_msgs.push(msg.to_string());
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.inner
            .borrow_mut()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.dispatch(event, params)
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, options: Vec<(&str, &str)>) -> Self {
        let mut opts = IndexMap::new();
        for (k, v) in options {
            opts.insert(k.to_string(), v.to_string());
        }
        FakeConfig {
            printer,
            section: section.to_string(),
            options: opts,
        }
    }
}

fn parse_error(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Unable to parse option '{}' in section '{}'",
        option, section
    ))
}

fn required_error(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Option '{}' in section '{}' must be specified",
        option, section
    ))
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
                None => Err(required_error(&self.section, option)),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, option: &str, opts: CfgGetIntOpts) -> KResult<Option<i64>> {
        match self.options.get(option) {
            Some(s) => pytext::int_from_string(s, 10)
                .map(|n| n.inner().to_string().parse::<i64>().unwrap_or(0))
                .map(Some)
                .map_err(|_| parse_error(&self.section, option)),
            None => match opts.default {
                None => Err(required_error(&self.section, option)),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: CfgGetFloatOpts) -> KResult<Option<f64>> {
        match self.options.get(option) {
            Some(s) => {
                let v = pytext::float_from_string(s)
                    .map_err(|_| parse_error(&self.section, option))?;
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option,
                            self.section,
                            pytext::float_repr(minv)
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option,
                            self.section,
                            pytext::float_repr(maxv)
                        )));
                    }
                }
                if let Some(a) = opts.above {
                    if v <= a {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.section,
                            pytext::float_repr(a)
                        )));
                    }
                }
                if let Some(b) = opts.below {
                    if v >= b {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option,
                            self.section,
                            pytext::float_repr(b)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                None => Err(required_error(&self.section, option)),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let _ = (option, opts);
        panic!("getboolean not used")
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let raw = match self.options.get(option) {
            Some(s) => s.clone(),
            None => match opts.default {
                None => {
                    return Err(required_error(&self.section, option));
                }
                Some(inner) => match inner {
                    Some(v) => v.str(),
                    None => String::new(),
                },
            },
        };
        if let PyValue::Dict(d) = choices {
            for (k, v) in d.iter() {
                if k.str() == raw {
                    return Ok(v.clone());
                }
            }
        }
        Err(KlippyError::Config(format!(
            "Choice '{}' for option '{}' in section '{}' is not a valid choice",
            raw, option, self.section
        )))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists not used")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist not used")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist not used")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section not used")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections not used")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options not used")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeSensor;

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        1.
    }
}

struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        oid
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.
    }
    fn max_nominal_duration(&self) -> f64 {
        0.
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("lookup_command not used")
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command not used")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
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
        panic!("register_serial_response not used")
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
        Ok(0.)
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 0.))
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
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("trigger dispatch not used")
    }
}

klippy_api::printer_object!(FakeMcu);

struct FakePwm {
    mcu: Mcu,
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: Option<bool>) -> klippy_api::KResult<()> { Ok(()) }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, _print_time: f64, _value: f64) -> klippy_api::KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakePwm; dyn PwmPinApi);

struct FakePins {
    pwm: PwmPin,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin not used")
    }
    fn setup_pin(&self, pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        assert_eq!(pin_type, "pwm");
        Ok(Pin::Pwm(self.pwm.clone()))
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("pin resolver not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeGCode;

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("mutex not used")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("create_gcode_command not used")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeHeaters {
    heaters: GCell<IndexMap<String, Arc<Heater>>>,
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        panic!("add_sensor_factory not used")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater not used")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.heaters.borrow().keys().cloned().collect()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        match self.heaters.borrow().get(heater_name) {
            Some(h) => Ok(h.clone()),
            None => Err(KlippyError::config(format!("Unknown heater '{heater_name}'"))),
        }
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor not used")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor not used")
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(
        &self,
        _heater: &Arc<Heater>,
        _temp: f64,
        _wait: bool,
    ) -> KResult<()> {
        panic!("set_temperature not used")
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

// ---------------------------------------------------------------------------
// Fixture builders.
// ---------------------------------------------------------------------------

fn make_printer(debugoutput: Option<String>) -> (Arc<FakePrinter>, Printer) {
    // Records emitted before a sink is installed go to stderr and are
    // lost, so install it before the port can log anything.
    install_log_sink();
    let reactor = Reactor::new(false);
    let printer = FakePrinter::new(debugoutput, reactor);
    let api: Printer = printer.clone();
    (printer, api)
}

/// Printer with pins/gcode/heaters objects and a real `Heater` registered
/// under `heater_name`; `register_heaters` controls whether the `heaters`
/// object is visible to the `verify_heater` handle_connect lookup.
struct Env {
    printer: Arc<FakePrinter>,
    heater: Arc<Heater>,
}

fn build_env(heater_name: &str, debugoutput: Option<String>, register_heaters: bool) -> Env {
    let (printer, printer_api) = make_printer(debugoutput);
    let mcu: Mcu = Arc::new(FakeMcu);
    let pwm: Arc<FakePwm> = Arc::new(FakePwm { mcu });
    let pwm_pin: PwmPin = pwm;
    let pins_obj: ObjectRef = Arc::new(FakePins { pwm: pwm_pin });
    printer.register_object("pins", pins_obj);
    let gcode_obj: ObjectRef = Arc::new(FakeGCode);
    printer.register_object("gcode", gcode_obj);
    let heaters = Arc::new(FakeHeaters {
        heaters: GCell::new(IndexMap::new()),
    });
    if register_heaters {
        let heaters_obj: ObjectRef = heaters.clone();
        printer.register_object("heaters", heaters_obj);
    }
    // Real heater (the `PrinterHeatersApi::lookup_heater` return type).
    let hcfg: Config = Arc::new(FakeConfig::new(
        printer_api.clone(),
        heater_name,
        vec![
            ("min_temp", "-100"),
            ("max_temp", "300"),
            ("control", "watermark"),
            ("heater_pin", "PA1"),
        ],
    ));
    let sensor: Arc<dyn TemperatureSensorApi> = Arc::new(FakeSensor);
    let heater = Heater::new(&hcfg, sensor).expect("Heater::new");
    heaters
        .heaters
        .borrow_mut()
        .insert(heater_name.to_string(), heater.clone());
    Env { printer, heater }
}

/// Env with a `HeaterCheck` for `verify_heater <heater_name>` already
/// connected (via the `klippy:connect` event), so `check_event` can run.
struct Connected {
    env: Env,
    hc: HeaterCheck,
}

fn make_connected(
    section: &str,
    options: Vec<(&str, &str)>,
    debugoutput: Option<String>,
    heater_name: &str,
) -> Connected {
    let env = build_env(heater_name, debugoutput, true);
    let printer_api: Printer = env.printer.clone();
    let vcfg: Config = Arc::new(FakeConfig::new(
        printer_api.clone(),
        section,
        options,
    ));
    let hc = HeaterCheck::new(&vcfg).expect("HeaterCheck::new");
    env.printer
        .dispatch("klippy:connect", &[])
        .expect("klippy:connect");
    assert!(hc.has_heater(), "heater set after connect");
    assert!(hc.has_check_timer(), "timer armed after connect");
    Connected { env, hc }
}

/// One `check_event` step on the Rust side: set the target, feed the
/// temperature sample, then run the check.
fn step_rust(
    hc: &HeaterCheck,
    heater: &Arc<Heater>,
    eventtime: f64,
    temp: f64,
    target: f64,
) -> RValueGB {
    heater.set_temp(target).expect("set_temp in range");
    heater.temperature_callback(eventtime, temp).expect("temperature_callback");
    let ret = hc.check_event(eventtime);
    RValueGB::Tuple(vec![
        RValueGB::float(ret),
        RValueGB::Bool(hc.approaching_target()),
        RValueGB::Bool(hc.starting_approach()),
        RValueGB::float(hc.last_target()),
        RValueGB::float(hc.goal_temp()),
        RValueGB::float(hc.error()),
        RValueGB::float(hc.goal_systime()),
    ])
}

// ---------------------------------------------------------------------------
// Oracle scaffolding.
// ---------------------------------------------------------------------------

use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RValueGB};

fn dummy_record(function: &str) -> Record {
    Record {
        v: 0,
        unit: "verify_heater".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: function.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn cmp_oracle(py: &mut pyoracle::Oracle, label: &str, expr: &str, actual: RValueGB) {
    let expected = match py.eval(expr) {
        Ok(v) => v,
        Err(e) => panic!("oracle eval `{}` raised {}: {}", expr, e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&dummy_record(label), label, &expected, &actual) {
        panic!("parity mismatch at `{}` (expr `{}`):\n{}", label, expr, d);
    }
}

fn cmp_exc_oracle(
    py: &mut pyoracle::Oracle,
    label: &str,
    expr: &str,
    rust: &Result<(), KlippyError>,
) {
    match py.eval(expr) {
        Ok(_) => panic!("oracle eval `{}` did not raise", expr),
        Err(expected) => match rust {
            Ok(()) => panic!(
                "rust did not raise at `{}` (expected {}: {})",
                label, expected.class_name, expected.message
            ),
            Err(e) => {
                assert_eq!(e.class_name(), expected.class_name, "class at `{}`", label);
                assert_eq!(e.message(), expected.message, "message at `{}`", label);
            }
        },
    }
}

fn py_str_lit(s: &str) -> String {
    format!("{:?}", s)
}

/// Float literal for inline Python expressions (`500.0`, not `500`).
fn rf(f: f64) -> String {
    format!("{:?}", f)
}

fn assert_log(py: &mut pyoracle::Oracle, label: &str, level: &str, msg: &str) {
    assert!(has_log(level, msg), "rust log {} {:?}", level, msg);
    let expr = format!(
        "any((l == {} and m == {}) for (l, m) in LOG_LIST)",
        py_str_lit(level),
        py_str_lit(msg)
    );
    cmp_oracle(py, label, &expr, RValueGB::Bool(true));
}

const PY_SETUP: &str = r#"
import sys
sys.path.insert(0, '__KLIPPY_EXTRAS__')
import logging
LOG_LIST = []
class _LogHandler(logging.Handler):
    def emit(self, record):
        LOG_LIST.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_LogHandler())
logging.getLogger().setLevel(logging.DEBUG)
import verify_heater

class Error(Exception):
    pass

NEVER = 9999999999999999.

class FakeReactor:
    NEVER = NEVER
    NOW = 0.
    def __init__(self):
        self.timer_count = 0
        self.updates = []
    def register_timer(self, cb, waketime):
        self.timer_count += 1
        return ('timer', self.timer_count)
    def update_timer(self, t, waketime):
        self.updates.append(waketime)

class FakeHeater:
    def __init__(self):
        self.temp = 0.
        self.target = 0.
    def get_temp(self, eventtime):
        return (self.temp, self.target)

class FakeHeaters:
    def __init__(self):
        self.heaters = {}
    def lookup_heater(self, name):
        if name not in self.heaters:
            raise Error("Unknown heater '%s'" % (name,))
        return self.heaters[name]

_sentinel = object()

class FakePrinter:
    config_error = Error
    def __init__(self, debugoutput=None):
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
        self.start_args = {'debugoutput': debugoutput}
        self.shutdown_msgs = []
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def invoke_shutdown(self, msg, details=None):
        self.shutdown_msgs.append(msg)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    def __init__(self, printer, section, options):
        self.printer = printer
        self.section = section
        self.options = dict(options)
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def get(self, option, default=_sentinel):
        if option in self.options:
            return self.options[option]
        if default is _sentinel:
            raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return default
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self.options:
            if default is _sentinel:
                raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        try:
            v = float(self.options[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v

def run_step(hc, fh, eventtime, temp, target):
    fh.temp = temp
    fh.target = target
    r = hc.check_event(eventtime)
    return (r, hc.approaching_target, hc.starting_approach, hc.last_target, hc.goal_temp, hc.error, hc.goal_systime)

def build(section, options, debugoutput, heater_names):
    printer = FakePrinter(debugoutput)
    fh = FakeHeaters()
    for n in heater_names:
        fh.heaters[n] = FakeHeater()
    printer.objects['heaters'] = fh
    cfg = FakeConfig(printer, section, options)
    hc = verify_heater.HeaterCheck(cfg)
    return (printer, hc, fh)
def build_printer(section, options, debugoutput, heater_names):
    printer = FakePrinter(debugoutput)
    fh = FakeHeaters()
    for n in heater_names:
        fh.heaters[n] = FakeHeater()
    printer.objects['heaters'] = fh
    cfg = FakeConfig(printer, section, options)
    return (printer, cfg)
"#;

fn py_setup(py: &mut pyoracle::Oracle) {
    let code = PY_SETUP.replace("__KLIPPY_EXTRAS__", &klippy_extras_path());
    if let Err(e) = py.exec(&code) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
}

fn py_build(
    py: &mut pyoracle::Oracle,
    section: &str,
    options: &[(&str, &str)],
    debugoutput: Option<&str>,
    heater_names: &[&str],
) {
    let mut opts = String::from("{");
    let mut first = true;
    for (k, v) in options {
        if !first {
            opts.push_str(", ");
        }
        opts.push_str(&format!("{}: {}", py_str_lit(k), py_str_lit(v)));
        first = false;
    }
    opts.push('}');
    let dbg = match debugoutput {
        Some(d) => py_str_lit(d),
        None => "None".to_string(),
    };
    let mut names = String::from("[");
    let mut first = true;
    for n in heater_names {
        if !first {
            names.push_str(", ");
        }
        names.push_str(&py_str_lit(n));
        first = false;
    }
    names.push(']');
    let code = format!(
        "printer, hc, fh = build({}, {}, {}, {})",
        py_str_lit(section),
        opts,
        dbg,
        names
    );
    if let Err(e) = py.exec(&code) {
        panic!("oracle build: {}: {}", e.class_name, e.message);
    }
}
fn py_build_printer_cfg(
    py: &mut pyoracle::Oracle,
    section: &str,
    options: &[(&str, &str)],
    debugoutput: Option<&str>,
    heater_names: &[&str],
) {
    let mut opts = String::from("{");
    let mut first = true;
    for (k, v) in options {
        if !first {
            opts.push_str(", ");
        }
        opts.push_str(&format!("{}: {}", py_str_lit(k), py_str_lit(v)));
        first = false;
    }
    opts.push('}');
    let dbg = match debugoutput {
        Some(d) => py_str_lit(d),
        None => "None".to_string(),
    };
    let mut names = String::from("[");
    let mut first = true;
    for n in heater_names {
        if !first {
            names.push_str(", ");
        }
        names.push_str(&py_str_lit(n));
        first = false;
    }
    names.push(']');
    let code = format!(
        "printer, cfg = build_printer({}, {}, {}, {})",
        py_str_lit(section),
        opts,
        dbg,
        names
    );
    if let Err(e) = py.exec(&code) {
        panic!("oracle build_printer: {}: {}", e.class_name, e.message);
    }
}

fn py_step(py: &mut pyoracle::Oracle, heater_name: &str, eventtime: f64, temp: f64, target: f64) {
    let code = format!(
        "result = run_step(hc, fh.heaters[{}], {}, {}, {})",
        py_str_lit(heater_name),
        rf(eventtime),
        rf(temp),
        rf(target)
    );
    if let Err(e) = py.exec(&code) {
        panic!("oracle step failed: {}: {}", e.class_name, e.message);
    }
}

// ---------------------------------------------------------------------------
// Local structural tests (no oracle numbers embedded).
// ---------------------------------------------------------------------------

#[test]
fn factory_entry_point() {
    let (printer, printer_api) = make_printer(None);
    let f = module_factory();
    assert!(f.load_config.is_none(), "verify_heater has no load_config");
    assert!(f.load_config_prefix.is_some(), "load_config_prefix present");
    let cfg: Config = Arc::new(FakeConfig::new(
        printer_api.clone(),
        "verify_heater extruder",
        vec![],
    ));
    let obj = f
        .load_config_prefix
        .expect("load_config_prefix")(&cfg)
        .expect("load_config_prefix")
        .expect("Some object");
    let hc = klippy_api::downcast_arc::<HeaterCheck>(&obj).expect("object is HeaterCheck");
    assert_eq!(hc.heater_name(), "extruder");
    assert_eq!(hc.hysteresis(), 5.0);
    assert_eq!(hc.max_error(), 120.0);
    assert_eq!(hc.heating_gain(), 2.0);
    assert_eq!(hc.check_gain_time(), 20.0);
    assert!(!hc.approaching_target());
    assert!(!hc.starting_approach());
    assert_eq!(hc.last_target(), 0.0);
    assert_eq!(hc.goal_temp(), 0.0);
    assert_eq!(hc.error(), 0.0);
    assert_eq!(hc.goal_systime(), reactor::NEVER);
    assert!(!hc.has_heater());
    assert!(!hc.has_check_timer());
    let keys = printer.handler_keys();
    assert!(
        keys.iter().any(|k| k == "klippy:connect"),
        "klippy:connect handler registered"
    );
    assert!(
        keys.iter().any(|k| k == "klippy:shutdown"),
        "klippy:shutdown handler registered"
    );
}

// ---------------------------------------------------------------------------
// Oracle parity tests.
// ---------------------------------------------------------------------------

#[test]
fn oracle_construction_and_attrs() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    // Extruder defaults.
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
        let (printer, printer_api) = make_printer(None);
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        cmp_oracle(&mut py, "name", "hc.heater_name", RValueGB::str(hc.heater_name()));
        cmp_oracle(&mut py, "hysteresis", "hc.hysteresis", RValueGB::float(hc.hysteresis()));
        cmp_oracle(&mut py, "max_error", "hc.max_error", RValueGB::float(hc.max_error()));
        cmp_oracle(&mut py, "heating_gain", "hc.heating_gain", RValueGB::float(hc.heating_gain()));
        cmp_oracle(&mut py, "check_gain_time", "hc.check_gain_time", RValueGB::float(hc.check_gain_time()));
        cmp_oracle(&mut py, "goal_systime", "hc.goal_systime", RValueGB::float(hc.goal_systime()));
        cmp_oracle(&mut py, "last_target", "hc.last_target", RValueGB::float(hc.last_target()));
        cmp_oracle(&mut py, "error", "hc.error", RValueGB::float(hc.error()));
        let _ = printer;
    }
    // Bed defaults (60s gain time).
    {
        py_build(&mut py, "verify_heater heater_bed", &[], None, &["heater_bed"]);
        let (_printer, printer_api) = make_printer(None);
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater heater_bed",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        cmp_oracle(&mut py, "bed check_gain_time", "hc.check_gain_time", RValueGB::float(hc.check_gain_time()));
        cmp_oracle(&mut py, "bed heater_name", "hc.heater_name", RValueGB::str(hc.heater_name()));
    }
    // Custom options.
    {
        let opts = [
            ("hysteresis", "7.5"),
            ("max_error", "90"),
            ("heating_gain", "3"),
            ("check_gain_time", "45"),
        ];
        py_build(&mut py, "verify_heater extruder", &opts, None, &["extruder"]);
        let (_printer, printer_api) = make_printer(None);
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            opts.to_vec(),
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        cmp_oracle(&mut py, "custom hysteresis", "hc.hysteresis", RValueGB::float(hc.hysteresis()));
        cmp_oracle(&mut py, "custom max_error", "hc.max_error", RValueGB::float(hc.max_error()));
        cmp_oracle(&mut py, "custom heating_gain", "hc.heating_gain", RValueGB::float(hc.heating_gain()));
        cmp_oracle(&mut py, "custom check_gain_time", "hc.check_gain_time", RValueGB::float(hc.check_gain_time()));
    }
    // HINT_THERMAL text.
    cmp_oracle(&mut py, "hint", "verify_heater.HINT_THERMAL", RValueGB::str(HINT_THERMAL));
}

#[test]
fn oracle_construction_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    let cases: &[(&str, &[(&str, &str)])] = &[
        ("negative hysteresis", &[("hysteresis", "-1")]),
        ("zero heating_gain", &[("heating_gain", "0")]),
        ("small check_gain_time", &[("check_gain_time", "0.5")]),
        ("unparseable hysteresis", &[("hysteresis", "abc")]),
    ];
    for (label, opts) in cases {
        py_build_printer_cfg(&mut py, "verify_heater extruder", opts, None, &["extruder"]);
        let (_printer, printer_api) = make_printer(None);
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            opts.to_vec(),
        ));
        let rust = HeaterCheck::new(&cfg).map(|_| ());
        cmp_exc_oracle(&mut py, label, "verify_heater.HeaterCheck(cfg)", &rust);
    }
    // Single-token section name: IndexError.
    {
        py_build_printer_cfg(&mut py, "verify_heater", &[], None, &[]);
        let (_printer, printer_api) = make_printer(None);
        let cfg: Config = Arc::new(FakeConfig::new(printer_api, "verify_heater", vec![]));
        let rust = HeaterCheck::new(&cfg).map(|_| ());
        cmp_exc_oracle(&mut py, "single token section", "verify_heater.HeaterCheck(cfg)", &rust);
    }
}

#[test]
fn oracle_handle_connect() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    // Normal connect via the event (also exercises handler registration).
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
        let conn = make_connected("verify_heater extruder", vec![], None, "extruder");
        assert!(has_log("INFO", "Starting heater checks for extruder"));
        if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
            panic!("oracle connect: {}: {}", e.class_name, e.message);
        }
        cmp_oracle(
            &mut py,
            "connected heater",
            "(hc.heater is not None, hc.check_timer is not None)",
            RValueGB::Tuple(vec![RValueGB::Bool(conn.hc.has_heater()), RValueGB::Bool(conn.hc.has_check_timer())]),
        );
        cmp_oracle(&mut py, "timer_count", "printer.reactor.timer_count", RValueGB::Int(1));
        assert_log(&mut py, "connect log", "INFO", "Starting heater checks for extruder");
    }
    // debugoutput disables the check entirely.
    {
        py_build(&mut py, "verify_heater extruder", &[], Some("/tmp/dbg"), &["extruder"]);
        let env = build_env("extruder", Some("/tmp/dbg".to_string()), true);
        let printer_api: Printer = env.printer.clone();
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        env.printer.dispatch("klippy:connect", &[]).expect("connect");
        assert!(!hc.has_heater(), "no heater lookup with debugoutput");
        assert!(!hc.has_check_timer(), "no timer with debugoutput");
        if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
            panic!("oracle connect: {}: {}", e.class_name, e.message);
        }
        cmp_oracle(
            &mut py,
            "debugoutput state",
            "(hc.heater is not None, hc.check_timer is not None)",
            RValueGB::Tuple(vec![RValueGB::Bool(false), RValueGB::Bool(false)]),
        );
        cmp_oracle(&mut py, "debugoutput timer", "printer.reactor.timer_count", RValueGB::Int(0));
    }
}

#[test]
fn oracle_handle_connect_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    // Missing `heaters` object.
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
        if let Err(e) = py.exec("del printer.objects['heaters']") {
            panic!("oracle del: {}: {}", e.class_name, e.message);
        }
        let env = build_env("extruder", None, false);
        let printer_api: Printer = env.printer.clone();
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        let rust: Result<(), KlippyError> = hc.handle_connect();
        cmp_exc_oracle(&mut py, "missing heaters object", "hc.handle_connect()", &rust);
        assert!(!hc.has_heater());
        assert!(!hc.has_check_timer());
    }
    // Unknown heater name.
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["heater_bed"]);
        let env = build_env("heater_bed", None, true);
        let printer_api: Printer = env.printer.clone();
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        let rust: Result<(), KlippyError> = hc.handle_connect();
        cmp_exc_oracle(&mut py, "unknown heater", "hc.handle_connect()", &rust);
    }
}

#[test]
fn oracle_check_event_within() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
    if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
        panic!("oracle connect: {}: {}", e.class_name, e.message);
    }
    let conn = make_connected("verify_heater extruder", vec![], None, "extruder");

    // Within hysteresis band: error cleared.
    py_step(&mut py, "extruder", 1000., 197., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1000., 197., 200.);
    cmp_oracle(&mut py, "within band", "result", actual);
    // Target off: no approaching log (approaching is false).
    py_step(&mut py, "extruder", 1001., 100., 0.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1001., 100., 0.);
    cmp_oracle(&mut py, "target off", "result", actual);
    // Start approach.
    py_step(&mut py, "extruder", 1002., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1002., 25., 200.);
    cmp_oracle(&mut py, "approach start", "result", actual);
    assert_log(
        &mut py,
        "approach log",
        "INFO",
        "Heater extruder approaching new target of 200.000",
    );
    // Arrive within range while approaching: error preserved (above band).
    py_step(&mut py, "extruder", 1003., 230., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1003., 230., 200.);
    cmp_oracle(&mut py, "within range of target", "result", actual);
    assert_log(&mut py, "within log", "INFO", "Heater extruder within range of 200.000");
    // Within again while idle: error preserved, no log asserted.
    py_step(&mut py, "extruder", 1004., 210., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1004., 210., 200.);
    cmp_oracle(&mut py, "within preserve error", "result", actual);
}

#[test]
fn oracle_check_event_approach() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
    if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
        panic!("oracle connect: {}: {}", e.class_name, e.message);
    }
    let conn = make_connected("verify_heater extruder", vec![], None, "extruder");

    py_step(&mut py, "extruder", 1000., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1000., 25., 200.);
    cmp_oracle(&mut py, "start", "result", actual);
    // Still approaching, goal lowered by min().
    py_step(&mut py, "extruder", 1001., 23., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1001., 23., 200.);
    cmp_oracle(&mut py, "goal lowered", "result", actual);
    // Goal reached: error cleared, new goal.
    py_step(&mut py, "extruder", 1002., 30., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1002., 30., 200.);
    cmp_oracle(&mut py, "goal reached", "result", actual);
    // Systime passes: no longer approaching.
    py_step(&mut py, "extruder", 1023., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1023., 25., 200.);
    cmp_oracle(&mut py, "no longer approaching", "result", actual);
    assert_log(
        &mut py,
        "no longer log",
        "INFO",
        "Heater extruder no longer approaching target 200.000",
    );
    // New target while idle.
    py_step(&mut py, "extruder", 1024., 26., 210.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1024., 26., 210.);
    cmp_oracle(&mut py, "new target", "result", actual);
    assert_log(
        &mut py,
        "new target log",
        "INFO",
        "Heater extruder approaching new target of 210.000",
    );
    // Target change while approaching: only last_target moves.
    py_step(&mut py, "extruder", 1025., 26., 210.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1025., 26., 210.);
    cmp_oracle(&mut py, "same target approaching", "result", actual);
}

#[test]
fn oracle_check_event_fault() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
    if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
        panic!("oracle connect: {}: {}", e.class_name, e.message);
    }
    let conn = make_connected("verify_heater extruder", vec![], None, "extruder");

    py_step(&mut py, "extruder", 1000., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1000., 25., 200.);
    cmp_oracle(&mut py, "fault start", "result", actual);
    py_step(&mut py, "extruder", 1030., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1030., 25., 200.);
    cmp_oracle(&mut py, "fault no longer", "result", actual);
    py_step(&mut py, "extruder", 1031., 25., 200.);
    let actual = step_rust(&conn.hc, &conn.env.heater, 1031., 25., 200.);
    cmp_oracle(&mut py, "fault", "result", actual);
    assert_log(
        &mut py,
        "fault log",
        "ERROR",
        "Heater extruder not heating at expected rate",
    );
    let expected_shutdown = format!("Heater extruder not heating at expected rate{HINT_THERMAL}");
    cmp_oracle(
        &mut py,
        "shutdown message",
        "printer.shutdown_msgs[-1]",
        RValueGB::str(&expected_shutdown),
    );
    // Direct fault call also returns NEVER and appends the same message.
    let rust_ret = conn.hc.heater_fault();
    cmp_oracle(&mut py, "direct fault return", "hc.heater_fault()", RValueGB::float(rust_ret));
    cmp_oracle(&mut py, "shutdown count", "len(printer.shutdown_msgs)", RValueGB::Int(2));
}

#[test]
fn oracle_handle_shutdown() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    // Shutdown before connect: no timer update.
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
        let env = build_env("extruder", None, true);
        let printer_api: Printer = env.printer.clone();
        let cfg: Config = Arc::new(FakeConfig::new(
            printer_api,
            "verify_heater extruder",
            vec![],
        ));
        let hc = HeaterCheck::new(&cfg).expect("HeaterCheck::new");
        hc.handle_shutdown();
        if let Err(e) = py.exec("printer.send_event('klippy:shutdown')") {
            panic!("oracle shutdown: {}: {}", e.class_name, e.message);
        }
        cmp_oracle(&mut py, "shutdown before connect", "printer.reactor.updates", RValueGB::List(vec![]));
    }
    // Shutdown after connect: timer updated to NEVER.
    {
        py_build(&mut py, "verify_heater extruder", &[], None, &["extruder"]);
        let conn = make_connected("verify_heater extruder", vec![], None, "extruder");
        // The reference has a timer to update only after `klippy:connect`
        // has run; put it in the same state the port was driven to.
        if let Err(e) = py.exec("printer.send_event('klippy:connect')") {
            panic!("oracle connect: {}: {}", e.class_name, e.message);
        }
        conn.env.printer.dispatch("klippy:shutdown", &[]).expect("shutdown");
        if let Err(e) = py.exec("printer.send_event('klippy:shutdown')") {
            panic!("oracle shutdown: {}: {}", e.class_name, e.message);
        }
        // The contract exposes no timer-waketime accessor on `Reactor`,
        // so the update argument the reference recorded is compared
        // against the port's NEVER constant.
        cmp_oracle(
            &mut py,
            "shutdown after connect",
            "printer.reactor.updates",
            RValueGB::List(vec![RValueGB::float(reactor::NEVER)]),
        );
        assert!(conn.hc.has_check_timer(), "timer still registered after shutdown");
    }
}
