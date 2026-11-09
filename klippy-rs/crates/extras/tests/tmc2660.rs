//! Parity tests for `klippy/extras/tmc2660.py`.
//!
//! Expected values come from the frozen Python reference through the
//! `pyoracle` crate (`$KLIPPY_PYTHON`; the tests skip without it). The
//! oracle builds the reference objects with Python fakes that mirror the
//! Rust fakes below, and each comparison drives the matching Rust call and
//! asserts equality of the returned value (floats by bit pattern and repr
//! spelling, via `recording::Value`) or of the raised exception (Python
//! class and `str(exc)`).
//!
//! Covered:
//! - the register / field / formatter tables (both header units),
//!   including every `FieldFormatters` body over a value grid;
//! - `crate::tmc::FieldHelper` (used by the driver) (`get_field` /
//!   `set_field` / `set_config_field` / `lookup_register` /
//!   `pretty_format` / `get_reg_fields`, and every error path);
//! - the current math (`_calc_current_bits`, `_calc_current_from_bits`,
//!   `_calc_current`), including the inf / nan / zero-sense-resistor
//!   edges;
//! - `TMC2660CurrentHelper` config reads (`run_current`, `sense_resistor`,
//!   `idle_current_percent`), the `idle_timeout` handler registration,
//!   `get_current` / `set_current` / `_update_current`, and the deferred
//!   idle-timeout callbacks, compared byte for byte against the SPI
//!   sends recorded by the fake MCU;
//! - the `MCU_TMC2660_SPI` shim (`get_register` / `get_register_raw` /
//!   `set_register` / `get_tmc_frequency` / `get_mcu`), in both
//!   `debugoutput` modes, including the RDSEL re-selection sends and the
//!   error paths (`... is not in list`, unknown registers);
//! - `crate::bus::McuSpi` (`__init__` config strings, `build_config`
//!   including bus resolution and the `spi_set_sw_bus` pulse-ticks
//!   branch, `spi_send` in both pre/post-build modes, `spi_transfer`,
//!   `get_oid` / `get_mcu` / `get_command_queue` /
//!   `setup_shutdown_msg`), including `MCU_SPI_from_config`'s
//!   config-parsing errors;
//! - `TMC2660.setup_driver_config` (the CHOPCONF / SMARTEN / SGSCONF /
//!   DRVCONF driver-field block and the `driver_HEND + driver_HSTRT`
//!   error), and that `module_factory` advertises `load_config_prefix`;
//! - `load_config_prefix` / `TMC2660.__init__` end to end against a printer
//!   fake with `pins`, `gcode` and `stepper_enable` objects (the interfaces
//!   are resolved with `lookup_interface` / `load_interface`): the ordered
//!   log of every config option read (method, section, option, default,
//!   bounds), `lookup_pin` / `reset_pin_sharing` calls, `load_object`,
//!   event handler registrations, `register_mux_command` calls, the MCU
//!   config commands, the `FieldHelper` register values, `get_status`,
//!   `get_phase_offset` (through the declared `TMCCommandHelperApi`) and
//!   every raised error.
//!
//! Not driven here: `resolve_bus_name` branches reached only when the MCU
//! enumerates a bus.

use std::sync::{Arc, Mutex};

use extras::bus::{mcu_spi_from_config, McuSpi, McuSpiApi};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use extras::tmc::{FieldHelper, TMCCommandHelperApi};
use extras::tmc2660::*;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, Chip, CommandQueue, CommandQueryWrapper, CommandQueryWrapperApi,
    CommandWrapper, CommandWrapperApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, QuerySendOpts, SendOpts,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, PrinterApi};
use klippy_api::stepper::Stepper;
use pycompat::text::str_repr;
use pycompat::{PyInt, PyValue};
use pytext::configparser::RawConfigParser;
use reactor::Reactor;
use recording::value::{FloatVal, Value};

// ---------------------------------------------------------------------------
// Repository helpers.
// ---------------------------------------------------------------------------

fn py_f(x: f64) -> String {
    recording::value::py_float_repr(x)
}

/// A Python float expression for interpolation into oracle code: finite
/// values use the shortest repr; non-finite ones must be spelled
/// `float('inf')` / `float('-inf')` / `float('nan')`, since a bare `inf`
/// or `nan` is a `NameError` in Python.
fn py_num(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x.is_infinite() {
        if x.is_sign_positive() {
            "float('inf')".to_string()
        } else {
            "float('-inf')".to_string()
        }
    } else {
        py_f(x)
    }
}

// ---------------------------------------------------------------------------
// FakeConfigApi (mirrors `klippy/configfile.py` ConfigWrapper semantics).
// ---------------------------------------------------------------------------

struct FakeConfig {
    printer: Printer,
    name: String,
    parser: RawConfigParser,
    section: String,
    /// Ordered record of every option read, shared with the printer fake.
    log: Log,
}

type Log = Arc<Mutex<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(Mutex::new(Vec::new()))
}

fn push_log(log: &Log, line: String) {
    log.lock().unwrap().push(line);
}

/// `repr(default)` of a `default=` argument as the Python log prints it:
/// `required` stands for the `sentinel` default.
fn default_repr<T>(default: &Option<Option<T>>, repr: impl Fn(&T) -> String) -> String {
    match default {
        None => "required".to_string(),
        Some(None) => "None".to_string(),
        Some(Some(v)) => repr(v),
    }
}

fn bound_repr<T>(bound: &Option<T>, repr: impl Fn(&T) -> String) -> String {
    match bound {
        None => "None".to_string(),
        Some(v) => repr(v),
    }
}

impl FakeConfig {
    fn new(printer: Printer, ini: &str, section: &str) -> FakeConfig {
        FakeConfig::with_log(printer, ini, section, new_log())
    }

    fn with_log(printer: Printer, ini: &str, section: &str, log: Log) -> FakeConfig {
        let mut parser = RawConfigParser::new_klippy();
        parser.read_string(ini).expect("fake config INI must parse");
        FakeConfig { printer, name: section.to_string(), parser, section: section.to_string(), log }
    }

    /// One line per `_get_wrapper` call, in the format of the Python
    /// `LogConfig` in the end-to-end test.
    fn log_read(&self, kind: &str, option: &str, default: String, bounds: [String; 4]) {
        push_log(
            &self.log,
            format!(
                "{kind}|{}|{option}|{default}|{}|{}|{}|{}",
                self.section, bounds[0], bounds[1], bounds[2], bounds[3]
            ),
        );
    }

    fn config_error(&self, msg: String) -> KlippyError {
        KlippyError::Config(msg)
    }

    // `_get_wrapper` missing-option arm (configfile.py:31-38).
    fn missing<T>(&self, option: &str, default: Option<Option<T>>) -> KResult<Option<T>> {
        match default {
            Some(d) => Ok(d),
            None => Err(self.config_error(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
        }
    }

    // `_get_wrapper` parse-failure arm (configfile.py:39-45).
    fn parse_fail<T>(&self, option: &str) -> KResult<T> {
        Err(self.config_error(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        )))
    }

    fn check_bounds<T: PartialOrd + std::fmt::Display>(
        &self,
        option: &str,
        value: &T,
        minval: Option<&T>,
        maxval: Option<&T>,
        above: Option<&T>,
        below: Option<&T>,
    ) -> KResult<()> {
        if let Some(m) = minval {
            if value < m {
                return Err(self.config_error(format!(
                    "Option '{option}' in section '{}' must have minimum of {m}",
                    self.section
                )));
            }
        }
        if let Some(m) = maxval {
            if value > m {
                return Err(self.config_error(format!(
                    "Option '{option}' in section '{}' must have maximum of {m}",
                    self.section
                )));
            }
        }
        if let Some(m) = above {
            if value <= m {
                return Err(self.config_error(format!(
                    "Option '{option}' in section '{}' must be above {m}",
                    self.section
                )));
            }
        }
        if let Some(m) = below {
            if value >= m {
                return Err(self.config_error(format!(
                    "Option '{option}' in section '{}' must be below {m}",
                    self.section
                )));
            }
        }
        Ok(())
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
        self.log_read(
            "get",
            option,
            default_repr(&opts.default, |s| str_repr(s)),
            ["None".into(), "None".into(), "None".into(), "None".into()],
        );
        if !self.parser.has_option(&self.section, option) {
            return self.missing(option, opts.default);
        }
        match self.parser.get(&self.section, option) {
            Ok(Some(v)) => Ok(Some(v)),
            _ => self.parse_fail(option),
        }
    }

    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.log_read(
            "getint",
            option,
            default_repr(&opts.default, |v| v.to_string()),
            [
                bound_repr(&opts.minval, |v| v.to_string()),
                bound_repr(&opts.maxval, |v| v.to_string()),
                "None".into(),
                "None".into(),
            ],
        );
        if !self.parser.has_option(&self.section, option) {
            return self.missing(option, opts.default);
        }
        let v = match self.parser.getint(&self.section, option) {
            Ok(v) => v,
            Err(_) => return self.parse_fail(option),
        };
        self.check_bounds(
            option,
            &v,
            opts.minval.as_ref(),
            opts.maxval.as_ref(),
            None,
            None,
        )?;
        Ok(Some(v))
    }

    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.log_read(
            "getfloat",
            option,
            default_repr(&opts.default, |v| py_f(*v)),
            [
                bound_repr(&opts.minval, |v| py_f(*v)),
                bound_repr(&opts.maxval, |v| py_f(*v)),
                bound_repr(&opts.above, |v| py_f(*v)),
                bound_repr(&opts.below, |v| py_f(*v)),
            ],
        );
        if !self.parser.has_option(&self.section, option) {
            return self.missing(option, opts.default);
        }
        let v = match self.parser.getfloat(&self.section, option) {
            Ok(v) => v,
            Err(_) => return self.parse_fail(option),
        };
        self.check_bounds(
            option,
            &v,
            opts.minval.as_ref(),
            opts.maxval.as_ref(),
            opts.above.as_ref(),
            opts.below.as_ref(),
        )?;
        Ok(Some(v))
    }

    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.log_read(
            "getboolean",
            option,
            default_repr(&opts.default, |v| if *v { "True" } else { "False" }.to_string()),
            ["None".into(), "None".into(), "None".into(), "None".into()],
        );
        if !self.parser.has_option(&self.section, option) {
            return self.missing(option, opts.default);
        }
        match self.parser.getboolean(&self.section, option) {
            Ok(v) => Ok(Some(v)),
            Err(_) => self.parse_fail(option),
        }
    }

    /// `ConfigWrapper.getchoice` with int keys (`tmc2660` only reads the
    /// `microsteps` choice): the lookup goes through `getint`, which is the
    /// call the Python log records.
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let PyValue::Dict(map) = choices else { panic!("choices must be a dict") };
        assert!(opts.default.is_none(), "getchoice defaults are not used by tmc2660");
        let c = self
            .getint(option, GetIntOpts { default: None, ..Default::default() })?
            .expect("required option");
        match map.get(&PyValue::Int(PyInt::from_i64(c))) {
            Some(found) => Ok(found.clone()),
            None => Err(self.config_error(format!(
                "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
                self.section
            ))),
        }
    }

    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::Other { class_name: "NotImplementedError".into(), message: "getlists unused by tmc2660".into() })
    }

    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::Other { class_name: "NotImplementedError".into(), message: "getlist unused by tmc2660".into() })
    }

    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::Other { class_name: "NotImplementedError".into(), message: "getintlist unused by tmc2660".into() })
    }

    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::Other { class_name: "NotImplementedError".into(), message: "getfloatlist unused by tmc2660".into() })
    }

    fn getsection(&self, section: &str) -> Config {
        let mut fake =
            FakeConfig::with_log(self.printer.clone(), "", section, self.log.clone());
        fake.parser = self.parser.clone();
        Arc::new(fake)
    }

    fn has_section(&self, section: &str) -> bool {
        self.parser.has_section(section)
    }

    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.parser
            .sections()
            .into_iter()
            .filter(|s| s.starts_with(prefix))
            .map(|s| self.getsection(&s))
            .collect()
    }

    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        Ok(self.parser
            .options(&self.section)
            .unwrap_or_default()
            .into_iter()
            .filter(|o| o.starts_with(prefix))
            .collect())
    }

    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// FakePrinter (mirrors the `Printer` service).
// ---------------------------------------------------------------------------

struct FakePrinter {
    reactor: Reactor,
    start_args: PyValue,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>)>>,
    /// Ordered record of `load_object` and `register_event_handler` calls.
    log: Log,
}

impl FakePrinter {
    fn new(start_args: PyValue) -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            start_args,
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            log: new_log(),
        })
    }

    fn taken_handlers(
        &self,
    ) -> Vec<(String, Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>)> {
        self.handlers.lock().unwrap().drain(..).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.start_args.clone()
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
        let mut objects = self.objects.lock().unwrap();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(
                "Duplicate config object name".to_string(),
            ));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }

    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock().unwrap();
        if let Some(o) = objects.get(name) {
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
            let objects = self.objects.lock().unwrap();
            match module {
                None => objects.iter().map(|(n, o)| (n.clone(), o.clone())).collect(),
                Some(module) => objects
                    .iter()
                    .filter(|(n, _)| n.as_str() == module || n.starts_with(&format!("{} ", module)))
                    .map(|(n, o)| (n.clone(), o.clone()))
                    .collect(),
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
    }

    /// The Python fake of the end-to-end test resolves `load_object` through
    /// `lookup_object` (no config section is loaded) and logs the section.
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        push_log(&self.log, format!("load_object|{section}"));
        self.lookup_object(section, default)
    }

    fn run(&self) -> Option<String> {
        None
    }

    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}

    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}

    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}

    fn register_event_handler(&self, event: &str, callback: klippy_api::printer::EventCallback) {
        push_log(&self.log, format!("register_event_handler|{event}"));
        self.handlers.lock().unwrap().push((event.to_string(), callback));
    }

    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }

    fn request_exit(&self, _result: &str) {}
}

// ---------------------------------------------------------------------------
// FakeMcu.
// ---------------------------------------------------------------------------

struct FakeCommandWrapper {
    sends: Arc<Mutex<Vec<(Vec<PyValue>, SendOpts)>>>,
}

pub struct SendRecord(pub Vec<PyValue>, pub SendOpts);

impl CommandWrapperApi for FakeCommandWrapper {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.sends.lock().unwrap().push((data, opts));
        Ok(())
    }

    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.send(data, opts)
    }

    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeCommandQueryWrapper {
    sends: Arc<Mutex<Vec<(Vec<PyValue>, QuerySendOpts)>>>,
    response: Arc<Mutex<PyValue>>,
}

impl CommandQueryWrapperApi for FakeCommandQueryWrapper {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        self.sends.lock().unwrap().push((data, opts));
        Ok(self.response.lock().unwrap().clone())
    }

    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.sends.lock().unwrap().push((data, opts));
        Ok(self.response.lock().unwrap().clone())
    }
}

/// The returned `register_serial_response` wrapper.
struct Unregistered;

impl klippy_api::mcu::AsyncResponseWrapperApi for Unregistered {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
    next_oid: Mutex<i64>,
    next_cq: Mutex<i64>,
    config_cmds: Mutex<Vec<(String, AddConfigCmdOpts)>>,
    config_callbacks: Mutex<Vec<klippy_api::mcu::ConfigCallback>>,
    enumerations: IndexMap<String, IndexMap<String, i64>>,
    constants: IndexMap<String, String>,
    clock_rate: f64,
    transfer_response: Arc<Mutex<PyValue>>,
    has_sw_bus: bool,
    send_log: Arc<Mutex<Vec<(Vec<PyValue>, SendOpts)>>>,
    transfer_log: Arc<Mutex<Vec<(Vec<PyValue>, QuerySendOpts)>>>,
}

impl FakeMcu {
    fn new(printer: Printer, clock_rate: f64) -> FakeMcu {
        let response = PyValue::Dict(IndexMap::from([
            (
                PyValue::Str("response".into()),
                PyValue::Bytes(vec![0xAA, 0xBB, 0xCC]),
            ),
            (
                PyValue::Str("#receive_time".into()),
                PyValue::Float(1234.5),
            ),
        ]));
        FakeMcu {
            printer,
            next_oid: Mutex::new(0),
            next_cq: Mutex::new(0),
            config_cmds: Mutex::new(Vec::new()),
            config_callbacks: Mutex::new(Vec::new()),
            enumerations: IndexMap::new(),
            constants: IndexMap::new(),
            clock_rate,
            transfer_response: Arc::new(Mutex::new(response)),
            has_sw_bus: false,
            send_log: Arc::new(Mutex::new(Vec::new())),
            transfer_log: Arc::new(Mutex::new(Vec::new())),
        }
    }

    fn config_cmds(&self) -> Vec<(String, AddConfigCmdOpts)> {
        self.config_cmds.lock().unwrap().clone()
    }

    fn commands_run(&self) {
        let raven = self.config_callbacks.lock().unwrap().drain(..).collect::<Vec<_>>();
        for mut cb in raven {
            cb().expect("config callback must succeed");
        }
    }

    /// Replace the `lookup_query_command` response (the Python fakes pass
    /// the response bytes and `#receive_time` to the shim directly).
    fn set_transfer_response(&self, response: PyValue) {
        *self.transfer_response.lock().unwrap() = response;
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
        Err(KlippyError::Other {
            class_name: "NotImplementedError".into(),
            message: "setup_pin unused by tmc2660".into(),
        })
    }

    fn create_oid(&self) -> KResult<i64> {
        let mut next = self.next_oid.lock().unwrap();
        *next += 1;
        Ok(*next)
    }

    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_callbacks.lock().unwrap().push(cb);
        Ok(())
    }

    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.config_cmds.lock().unwrap().push((cmd.to_string(), opts));
        Ok(())
    }

    fn request_move_queue_slot(&self) {}

    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }

    fn seconds_to_clock(&self, time: f64) -> i64 {
        // mcu.py: `int(time * clock)`: Python `int()` truncates toward
        // zero; time is positive on every tested path.
        kos::cconv::f64_to_i64(time * self.clock_rate)
    }

    fn min_schedule_time(&self) -> f64 {
        0.100
    }

    fn max_nominal_duration(&self) -> f64 {
        3.0
    }

    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCommandWrapper { sends: self.send_log.clone() }))
    }

    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(Arc::new(FakeCommandQueryWrapper {
            sends: self.transfer_log.clone(),
            response: self.transfer_response.clone(),
        }))
    }

    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        if self.has_sw_bus && msgformat.starts_with("spi_set_sw_bus") {
            Some(Arc::new(FakeCommandWrapper { sends: self.send_log.clone() }))
        } else {
            None
        }
    }

    fn alloc_command_queue(&self) -> CommandQueue {
        let mut next = self.next_cq.lock().unwrap();
        *next += 1;
        CommandQueue(*next)
    }

    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> Arc<dyn klippy_api::mcu::AsyncResponseWrapperApi> {
        Arc::new(Unregistered)
    }

    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }

    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.enumerations.clone()
    }

    fn get_constants(&self) -> IndexMap<String, String> {
        self.constants.clone()
    }

    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        self.constants
            .get(name)
            .and_then(|v| v.parse().ok())
            .ok_or_else(|| KlippyError::Config(format!("Unknown constant '{name}'")))
    }

    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        // The shared rule used by the Python fake below: the clock of
        // `print_time` at the fake's clock rate.
        kos::cconv::f64_to_i64(print_time * self.clock_rate)
    }

    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.clock_rate
    }

    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
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
        Err(KlippyError::Other {
            class_name: "NotImplementedError".into(),
            message: "create_trigger_dispatch unused by tmc2660".into(),
        })
    }
}

// ---------------------------------------------------------------------------
// Recorded values for comparison with the oracle.
// ---------------------------------------------------------------------------

fn v_int(n: i64) -> Value {
    Value::Int(n)
}

fn v_float(f: f64) -> Value {
    let mut v = FloatVal::new(f);
    // `py_float_repr` spells NaN the Rust way ("NaN"); CPython's repr is "nan".
    if f.is_nan() {
        v.repr = "nan".to_string();
    }
    Value::Float(v)
}

fn v_str(s: impl Into<String>) -> Value {
    Value::Str(s.into())
}

fn v_none() -> Value {
    Value::Null
}

fn check_oracle(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    match expected {
        Ok(v) => assert_eq!(v, actual, "{label}"),
        Err(exc) => panic!("{label}: oracle raised {}: {}", exc.class_name, exc.message),
    }
}

/// `PyValue` -> recorded `Value`, for comparing a port result that is a
/// `PyValue` (e.g. the `get_register_raw` dict) with the oracle's answer.
fn pyv(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.0.to_string().parse::<i64>() {
            Ok(i) => Value::Int(i),
            Err(_) => Value::BigInt(n.0.to_string()),
        },
        PyValue::Float(f) => v_float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(pyv).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyv).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, val)| (pyv(k), pyv(val))).collect()),
        PyValue::Host(_) => panic!("host handle has no recorded form"),
    }
}

/// The recorded shape of `FakeSpi.spi_send` calls: `(data, minclock,
/// reqclock)` with `data` as a list of ints. The port's wrapper records
/// `[oid, bytes]`, so only the 3-byte frame is compared (the Python fake
/// records the frame alone, as `tmc2660` hands it to `spi_send`).
fn rust_spi_calls(sends: &[(Vec<PyValue>, SendOpts)]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|(data, opts)| {
                Value::Tuple(vec![
                    Value::List(extract_frame(data).iter().map(|b| v_int(*b as i64)).collect()),
                    v_int(opts.minclock),
                    v_int(opts.reqclock),
                ])
            })
            .collect(),
    )
}

/// The recorded shape of `FakeSpi.spi_transfer` calls.
fn rust_spi_transfers(transfers: &[(Vec<PyValue>, QuerySendOpts)]) -> Value {
    Value::List(
        transfers
            .iter()
            .map(|(data, opts)| {
                Value::Tuple(vec![
                    Value::List(extract_frame(data).iter().map(|b| v_int(*b as i64)).collect()),
                    v_int(opts.minclock),
                    v_int(opts.reqclock),
                ])
            })
            .collect(),
    )
}

/// The `{'response': ..., '#receive_time': ...}` dict a query command
/// returns.
fn spi_response(response: &[u8], receive_time: f64) -> PyValue {
    PyValue::Dict(IndexMap::from([
        (PyValue::Str("response".into()), PyValue::Bytes(response.to_vec())),
        (PyValue::Str("#receive_time".into()), PyValue::Float(receive_time)),
    ]))
}

fn check_oracle_err<T: std::fmt::Debug>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    match (expected, actual) {
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception type");
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
        (Err(exc), Ok(_v)) => panic!("{label}: oracle raised {}: {}, port returned", exc.class_name, exc.message),
        (Ok(v), Err(e)) => panic!("{label}: oracle returned {v:?}, port raised {}: {}", e.class_name(), e.message()),
        (Ok(_), Ok(_)) => panic!("{label}: both sides succeeded, expected a shared error"),
    }
}

fn check_error(label: &str, expected: &KlippyError, actual: &KlippyError) {
    assert_eq!(expected.class_name(), actual.class_name(), "{label} exception type");
    assert_eq!(expected.message(), actual.message(), "{label} exception message");
}


/// The `i64` inside an `Int` value (recorded oracle values).
fn as_int(v: &Value) -> i64 {
    match v {
        Value::Int(n) => *n,
        other => panic!("expected int, got {other:?}"),
    }
}
// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------


#[test]
fn module_tables_match_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660").expect("import extras.tmc2660");
    // Registers.
    check_oracle("Registers", py.eval("extras.tmc2660.Registers"),
        Value::Dict(registers().iter().map(|(k, v)| (v_str(k), v_int(*v))).collect()));
    // ReadRegisters.
    check_oracle("ReadRegisters", py.eval("extras.tmc2660.ReadRegisters"),
        Value::List(read_registers().iter().map(|s| v_str(s)).collect()));
    // SignedFields.
    check_oracle("SignedFields", py.eval("extras.tmc2660.SignedFields"),
        Value::List(vec![v_str("sgt")]));
    // MAX_CURRENT.
    check_oracle("MAX_CURRENT", py.eval("extras.tmc2660.MAX_CURRENT"), v_float(MAX_CURRENT));
    // Fields: every register and every field mask.
    let all = fields();
    for (reg, fmap) in &all {
        for (fname, mask_) in fmap {
            check_oracle(&format!("Fields[{reg:?}][{fname:?}]"),
                py.eval(&format!("extras.tmc2660.Fields[{reg:?}][{fname:?}]")),
                v_int(*mask_ as i64));
        }
    }
    // FieldFormatters: same keys, same outputs over a value grid.
    check_oracle("formatter keys", py.eval("sorted(extras.tmc2660.FieldFormatters)"),
        Value::List({
            let mut keys = field_formatters().keys().cloned().collect::<Vec<_>>();
            keys.sort();
            keys.iter().map(|k| v_str(k)).collect()
        }));
    for v in [-3i64, -1, 0, 1, 2, 3, 5, 15, 255, 0x100] {
        for (name, f) in field_formatters().iter().map(|(k, v)| (k.clone(), v)) {
            // `mres` computes `0x100 >> v`, which Python rejects for
            // negative `v` (`ValueError: negative shift count`); the
            // formatter has no error channel, so those inputs are out of
            // contract and skipped here.
            if name == "mres" && v < 0 {
                continue;
            }
            check_oracle(&format!("formatter {name}({v})"),
                py.eval(&format!("extras.tmc2660.FieldFormatters[{name:?}]({v})")),
                v_str(f(v)));
        }
    }
}

#[test]
fn field_helper_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc").expect("import reference");
    py.exec("fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)")
        .expect("build reference FieldHelper");
    let fh = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
    let all = fields();
    for (_reg, fmap) in fields().iter() {
        for (fname, _) in fmap {
            for v in [0i64, 1, 2, 5, 15, 31, 63, -1, -64] {
                let py_expr = format!("fh.set_field({fname:?}, {v})");
                match py.eval(&py_expr) {
                    Ok(exp) => {
                        let got = fh.set_field(&fname, v, None, None).expect("set_field");
                        check_oracle(&format!("set_field({fname}, {v})"), Ok(exp), v_int(got as i64));
                    }
                    Err(exc) => {
                        let got = fh.set_field(&fname, v, None, None);
                        assert!(got.is_err(), "set_field({fname}, {v}): oracle raised, port returned");
                        let e = got.unwrap_err();
                        assert_eq!(e.class_name(), exc.class_name);
                        assert_eq!(e.message(), exc.message);
                    }
                }
                check_oracle(&format!("get_field({fname})"),
                    py.eval(&format!("fh.get_field({fname:?})")),
                    match fh.get_field(&fname, None, None) {
                        Ok(n) => v_int(n),
                        Err(e) => panic!("get_field({fname}) raised {e:?}"),
                    });
            }
        }
    }
    // Unknown field and register paths.
    check_oracle_err("get_field unknown", py.eval("fh.get_field('nope')"), fh.get_field("nope", None, None));
    check_oracle_err("lookup unknown reg", py.eval("fh.get_field('cs', None, 'NOPE')"), fh.get_field("cs", None, Some("NOPE")));
    // lookup_register defaults.
    check_oracle("lookup_register cs", py.eval("fh.lookup_register('cs')"), v_str(fh.lookup_register("cs").unwrap()));
    check_oracle("lookup_register nope default", py.eval("fh.lookup_register('nope', 'fallback')"),
        v_str(fh.lookup_register("nope").unwrap_or_else(|| "fallback".to_string())));
    // explicit reg_value / reg_name.
    check_oracle("get_field explicit", py.eval("fh.get_field('mstep', 0x1234, 'READRSP@RDSEL0')"),
        v_int(fh.get_field("mstep", Some(0x1234), Some("READRSP@RDSEL0")).expect("explicit")));
    check_oracle("set_field explicit", py.eval("fh.set_field('mstep', 7, 0x55aa, 'READRSP@RDSEL0')"),
        v_int(fh.set_field("mstep", 7, Some(0x55aa), Some("READRSP@RDSEL0")).expect("explicit set") as i64));
    // signed sgt round trip.
    check_oracle("sgt set -64", py.eval("fh.set_field('sgt', -64)"), v_int(fh.set_field("sgt", -64, None, None).expect("sgt set") as i64));
    check_oracle("sgt get", py.eval("fh.get_field('sgt')"), v_int(fh.get_field("sgt", None, None).expect("sgt get")));
    check_oracle("sgt set 63", py.eval("fh.set_field('sgt', 63)"), v_int(fh.set_field("sgt", 63, None, None).expect("sgt set 63") as i64));
    check_oracle("sgt get 63", py.eval("fh.get_field('sgt')"), v_int(fh.get_field("sgt", None, None).expect("sgt get 63")));
    // pretty_format and get_reg_fields over the shared table state.
    // pretty_format with fixed register values (no register state).
    for (reg, _) in all.iter() {
        for rv in [0u64, 0x00FFFFFF, 0xDEADBEEF] {
            check_oracle(&format!("pretty_format {reg} {rv:08x}"),
                py.eval(&format!("fh.pretty_format({reg:?}, {rv})")),
                v_str(fh.pretty_format(reg, rv as i64)));
            // get_reg_fields: dict compare.
            let py_fields = py.eval(&format!("fh.get_reg_fields({reg:?}, {rv})"));
            let rust_fields = fh.get_reg_fields(reg, rv as i64);
            check_oracle(&format!("get_reg_fields {reg} {rv:08x}"), py_fields,
                Value::Dict(rust_fields.iter().map(|(k, v)| (v_str(k), v_int(*v))).collect()));
        }
    }
}

#[test]
fn current_math_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660").expect("import extras.tmc2660");
    // `_calc_current_bits(self, current, vsense)` needs `self` for the
    // sense resistor only; evaluate the body with an explicit sr here
    // via a bare function mirroring the Python formula.
    let currents = [0.0f64, 0.05, 0.1, 0.25, 0.5, 0.75, 1.0, 1.1, 1.5, 2.0, 2.4, 3.0, -1.0, 100.0];
    let resistances = [0.05f64, 0.075, 0.1, 0.15, 0.2, 0.0, -0.1, f64::INFINITY, f64::NEG_INFINITY, f64::NAN];
    py.exec("import math").expect("math");
    py.exec("def calc_bits(current, sr, vsense):\n    vref = 0.165 if vsense else 0.310\n    cs = int(32. * sr * current * math.sqrt(2.) / vref + .5) - 1\n    return max(0, min(31, cs))")
        .expect("calc_bits def");
    py.exec("def calc_from_bits(cs, sr, vsense):\n    vref = 0.165 if vsense else 0.310\n    return (cs + 1) * vref / (32. * sr * math.sqrt(2.))")
        .expect("calc_from_bits def");
    py.exec("def calc_cur(run_current, sr):\n    vsense = True\n    irun = calc_bits(run_current, sr, True)\n    if irun == 31:\n        cur = calc_from_bits(irun, sr, True)\n        if cur < run_current:\n            irun2 = calc_bits(run_current, sr, False)\n            cur2 = calc_from_bits(irun2, sr, False)\n            if abs(run_current - cur2) < abs(run_current - cur):\n                vsense, irun = False, irun2\n    return (vsense, irun)")
        .expect("calc_cur def");
    for &sr in &resistances {
        for &cur in &currents {
            for &vs in &[true, false] {
                let label = format!("bits({cur},{sr},{vs})");
                match (py.eval(&format!("calc_bits({}, {}, {})", py_num(cur), py_num(sr), if vs { "True" } else { "False" })), calc_current_bits(cur, sr, vs)) {
                    (Ok(e), Ok(a)) => assert_eq!(e, v_int(a), "{label}"),
                    (Err(exc), Err(e)) => { assert_eq!(e.class_name(), exc.class_name, "{label}"); assert_eq!(e.message(), exc.message, "{label}"); }
                    (Ok(v), Err(e)) => panic!("{label}: oracle {v:?}, port raised {}", e.message()),
                    (Err(exc), Ok(a)) => panic!("{label}: oracle raised {}: {}, port {a}", exc.class_name, exc.message),
                }
            }
            {
                let label = format!("calc({cur},{sr})");
                match (py.eval(&format!("calc_cur({}, {})", py_num(cur), py_num(sr))), calc_current(cur, sr)) {
                    (Ok(e), Ok((b, a))) => assert_eq!(e, Value::Tuple(vec![Value::Bool(b), v_int(a)]), "{label}"),
                    (Err(exc), Err(e)) => { assert_eq!(e.class_name(), exc.class_name, "{label}"); assert_eq!(e.message(), exc.message, "{label}"); }
                    (Ok(v), Err(e)) => panic!("{label}: oracle {v:?}, port raised {}", e.message()),
                    (Err(exc), Ok(a)) => panic!("{label}: oracle raised {}: {}, port {a:?}", exc.class_name, exc.message),
                }
            }
            for &cs in &[0i64, 1, 15, 30, 31] {
                for &vs in &[true, false] {
                    let label = format!("from_bits({cs},{sr},{vs})");
                    match (py.eval(&format!("calc_from_bits({}, {}, {})", cs, py_num(sr), if vs { "True" } else { "False" })), calc_current_from_bits(cs, sr, vs)) {
                        (Ok(e), Ok(a)) => assert_eq!(e, v_float(a), "{label}"),
                        (Err(exc), Err(e)) => { assert_eq!(e.class_name(), exc.class_name, "{label}"); assert_eq!(e.message(), exc.message, "{label}"); }
                        (Ok(v), Err(e)) => panic!("{label}: oracle {v:?}, port raised {}", e.message()),
                        (Err(exc), Ok(a)) => panic!("{label}: oracle raised {}: {}, port {a}", exc.class_name, exc.message),
                    }
                }
            }
        }
    }
}

// ====== TMC2660CurrentHelper ======

fn make_tmc_with_fakes(
    printer: Arc<FakePrinter>,
    print_time_clock: f64,
) -> (Arc<FakeMcu>, Arc<McuSpi>, Arc<McuTmc2660Spi>, Arc<FieldHelper>, Arc<FakePrinter>) {
    let fields = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
    let mcu = Arc::new(FakeMcu::new(printer.clone() as Printer, print_time_clock));
    let spi = McuSpi::new(mcu.clone() as Mcu, None, Some("PB1".to_string()), 0, 4000000, None, false)
        .expect("McuSpi::new");
    mcu.commands_run(); // build_config: resolves the send/transfer wrappers
    let tmc = McuTmc2660Spi::new_with_spi(printer.clone() as Printer, spi.clone(), registers(), fields.clone());
    (mcu, spi, tmc, fields, printer)
}

#[test]
fn current_helper_error_paths() {
    let cases = [
        ("run_current missing", "[tmc2660 stepper_x]\nsense_resistor = 0.1\n"),
        ("sense_resistor missing", "[tmc2660 stepper_x]\nrun_current = 0.5\n"),
        ("run_current below", "[tmc2660 stepper_x]\nrun_current = 0.01\nsense_resistor = 0.1\n"),
        ("run_current above", "[tmc2660 stepper_x]\nrun_current = 2.5\nsense_resistor = 0.1\n"),
        ("run_current unparseable", "[tmc2660 stepper_x]\nrun_current = abc\nsense_resistor = 0.1\n"),
        ("idle_current_percent above", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.1\nidle_current_percent = 101\n"),
        ("idle_current_percent below", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.1\nidle_current_percent = -1\n"),
    ];
    for (label, ini) in cases {
        let fake_printer = FakePrinter::new(PyValue::None);
        let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer, ini, "tmc2660 stepper_x"));
        let (_, _, tmc, _, _) = make_tmc_with_fakes(fake_printer.clone(), 1000000.0);
        let res = Tmc2660CurrentHelper::new(&config, tmc);
        assert!(res.is_err(), "{label}: expected a config error");
        let e = res.err().expect("expected a config error");
        // Error texts mirror configfile.ConfigWrapper (klippy/configfile.py).
        assert_eq!(e.class_name(), "Error", "{label}");
        assert!(e.message().starts_with("Option '") || e.message().starts_with("Unable to parse"), "{label}: unexpected message {}", e.message());
    }
    // Exact texts for the distinctive arms.
    let _printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(_printer.clone() as Printer,
        "[tmc2660 stepper_x]\nsense_resistor = 0.1\n", "tmc2660 stepper_x"));
    let (_, _, tmc, _, _) = make_tmc_with_fakes(_printer.clone(), 1000000.0);
    let e = Tmc2660CurrentHelper::new(&config, tmc).err().expect("must fail");
    assert_eq!(e.message(), "Option 'run_current' in section 'tmc2660 stepper_x' must be specified");

    let _printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(_printer.clone() as Printer,
        "[tmc2660 stepper_x]\nrun_current = 2.5\nsense_resistor = 0.1\n", "tmc2660 stepper_x"));
    let (_, _, tmc, _, _) = make_tmc_with_fakes(_printer.clone(), 1000000.0);
    let e = Tmc2660CurrentHelper::new(&config, tmc).err().expect("must fail");
    assert_eq!(e.message(), "Option 'run_current' in section 'tmc2660 stepper_x' must have maximum of 2.4");

    let _printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(_printer.clone() as Printer,
        "[tmc2660 stepper_x]\nrun_current = abc\nsense_resistor = 0.1\n", "tmc2660 stepper_x"));
    let (_, _, tmc, _, _) = make_tmc_with_fakes(_printer.clone(), 1000000.0);
    let e = Tmc2660CurrentHelper::new(&config, tmc).err().expect("must fail");
    check_error("unparseable", &KlippyError::Config(
        "Unable to parse option 'run_current' in section 'tmc2660 stepper_x'".into()), &e);
}

#[test]
fn current_helper_construction_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc, configparser, configfile").expect("import");
    py.exec(r#"
class P:
    def __init__(self): self.handlers = []
    def register_event_handler(self, event, cb): self.handlers.append(event)
class M:
    def __init__(self, fields): self.fields = fields; self.set_calls = []
    def get_fields(self): return self.fields
    def set_register(self, reg, val, print_time=None): self.set_calls.append((reg, val, print_time))
def make_helper(ini):
    p = P()
    raw = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    raw.read_string(ini)
    c = configfile.ConfigWrapper(p, raw, {}, 'tmc2660 stepper_x')
    fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)
    m = M(fh)
    h = extras.tmc2660.TMC2660CurrentHelper(c, m)
    return (h, p, m, fh)
"#).expect("python shim");
    let inis = [
        ("idle 100 default", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.075\n"),
        ("idle 50", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.075\nidle_current_percent = 50\n"),
        ("idle 0", "[tmc2660 stepper_x]\nrun_current = 2.4\nsense_resistor = 0.05\nidle_current_percent = 0\n"),
        ("idle 100 explicit", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.075\nidle_current_percent = 100\n"),
        ("run missing", "[tmc2660 stepper_x]\nsense_resistor = 0.1\n"),
        ("sr missing", "[tmc2660 stepper_x]\nrun_current = 0.5\n"),
        ("run above", "[tmc2660 stepper_x]\nrun_current = 2.5\nsense_resistor = 0.1\n"),
        ("run unparseable", "[tmc2660 stepper_x]\nrun_current = abc\nsense_resistor = 0.1\n"),
        ("idle outside", "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.1\nidle_current_percent = 101\n"),
    ];
    for (label, ini) in inis {
        let fake_printer = FakePrinter::new(PyValue::None);
        let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer, ini, "tmc2660 stepper_x"));
        let (_, _, tmc, fields, _) = make_tmc_with_fakes(fake_printer.clone(), 1000000.0);
        let rust_res = Tmc2660CurrentHelper::new(&config, tmc);
        // Record what the reference does with the SAME ini.
        let py_bind = format!("h, p, m, fh = make_helper({ini:?})");
        match py.exec(&py_bind) {
            Ok(_) => {
                let helper = rust_res.unwrap_or_else(|e| panic!("{label}: port raised {e:?}"));
                check_oracle(&format!("{label} name"), py.eval("h.name"), v_str(helper.name()));
                check_oracle(&format!("{label} current"), py.eval("h.current"), v_float(helper.current()));
                let (cur, _, _, mx) = helper.get_current();
                check_oracle(&format!("{label} get_current"), py.eval("(h.current, None, None, 2.4)"),
                    Value::Tuple(vec![v_float(cur), v_none(), v_none(), v_float(mx)]));
                check_oracle(&format!("{label} cs"), py.eval("fh.get_field('cs')"), v_int(fields.get_field("cs", None, None).expect("rust cs")));
                check_oracle(&format!("{label} vsense"), py.eval("fh.get_field('vsense')"), v_int(fields.get_field("vsense", None, None).expect("rust vsense")));
                let rust_handlers = fake_printer.handlers.lock().unwrap().iter().map(|(e, _)| e.clone()).collect::<Vec<_>>();
                check_oracle(&format!("{label} handlers"), py.eval("list(p.handlers)"),
                    Value::List(rust_handlers.iter().map(|e| v_str(e)).collect()));
            }
            Err(exc) => {
                let e = match rust_res { Err(e) => e, Ok(_) => panic!("{label}: port succeeded, oracle raised {exc:?}") };
                assert_eq!(e.class_name(), exc.class_name, "{label}");
                assert_eq!(e.message(), exc.message, "{label}");
            }
        }
    }
}

#[test]
fn current_helper_usage_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc, configparser, configfile").expect("import");
    py.exec(r#"
class P:
    def __init__(self): self.handlers = []
    def register_event_handler(self, event, cb): self.handlers.append(event)
class M:
    def __init__(self, fields): self.fields = fields; self.set_calls = []
    def get_fields(self): return self.fields
    def set_register(self, reg, val, print_time=None): self.set_calls.append((reg, val, print_time))
def make_helper(ini):
    p = P()
    raw = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    raw.read_string(ini)
    c = configfile.ConfigWrapper(p, raw, {}, 'tmc2660 stepper_x')
    fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)
    m = M(fh)
    h = extras.tmc2660.TMC2660CurrentHelper(c, m)
    return (h, p, m, fh)
"#).expect("python shim");
    let ini = "[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.075\n";
    py.exec(&format!("h, p, m, fh = make_helper({ini:?})")).expect("build py helper");
    let fake_printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer, ini, "tmc2660 stepper_x"));
    let (mcu, _spi, tmc, fields, _) = make_tmc_with_fakes(fake_printer.clone(), 1000000.0);
    let helper = Tmc2660CurrentHelper::new(&config, tmc).expect("helper constructs");
    // After construction the python side is a pristine reference for the
    // initial cs/vsense; the rust side matches via the previous test.
    // Now drive set_current on both and compare the resulting field
    // state plus the (reg, val, print_time) set_calls.
    for (label, (run, _hold, pt)) in [
        ("low", (0.2f64, 0.1f64, 123.5f64)),
        ("high vsense off", (2.3, 2.0, 124.5)),
        ("mid", (1.0, 1.0, 125.5)),
        ("tiny", (0.1, 0.0, 126.5)),
    ] {
        py.exec(&format!("m.set_calls.clear(); h.set_current({}, {}, {})", py_f(run), py_f(_hold), py_f(pt)))
            .expect("py set_current");
        let set_calls_before = mcu.send_log.lock().unwrap().len();
        helper.set_current(run, _hold, pt).expect("rust set_current");
        // Field state on both sides.
        check_oracle(&format!("{label} cs"), py.eval("fh.get_field('cs')"),
            v_int(fields.get_field("cs", None, None).expect("rust cs")));
        check_oracle(&format!("{label} vsense"), py.eval("fh.get_field('vsense')"),
            v_int(fields.get_field("vsense", None, None).expect("rust vsense")));
        check_oracle(&format!("{label} current"), py.eval("h.current"), v_float(helper.current()));
        // set_register triples: python recorded (reg, val, print_time);
        // rust sends are frames [((val>>16)|reg)&0xff, (val>>8)&0xff, val&0xff].
        // Compare per-call: decode val from the rust frames.
        let py_calls = py.eval("m.set_calls").expect("m.set_calls");
        let rust_sends = mcu.send_log.lock().unwrap()[set_calls_before..].to_vec();
        match py_calls {
            Value::List(calls) => {
                assert_eq!(calls.len(), rust_sends.len(), "{label}: set_register call count");
                for (call, (data, opts)) in calls.iter().zip(rust_sends.iter()) {
                    let Value::Tuple(pair) = call else { panic!("{label}: set_call shape") };
                    let Value::Str(reg) = &pair[0] else { panic!("{label}: set_call reg") };
                    let val = as_int(&pair[1]);
                    let reg_int: i64 = match reg.as_str() {
                        "SGCSCONF" => 0x0C,
                        "DRVCONF" => 0x0E,
                        other => panic!("{label}: unexpected register {other}"),
                    };
                    let expected_frame = spi_triple_frame(val, reg_int);
                    let got_frame = extract_frame(data);
                    assert_eq!(expected_frame, got_frame, "{label}: frame for {reg}");
                    // print_time -> minclock at 1.0 MHz fake rate.
                    let Some(print_time) = pair.get(2) else { panic!("{label}: set_call print_time") };
                    if let Value::Float(f) = print_time {
                        assert_eq!(opts.minclock, kos::cconv::f64_to_i64(f.to_f64().expect("float") * 1000000.0), "{label}: minclock");
                        assert_eq!(opts.reqclock, 0, "{label}: reqclock");
                    } else {
                        panic!("{label}: print_time not float: {print_time:?}");
                    }
                }
            }
            other => panic!("{label}: set_calls shape {other:?}"),
        }
    }
}

/// Decode the 3-byte frame out of a `CommandWrapper.send` data vector
/// `[oid, bytes]` (oid first).
fn extract_frame(data: &[PyValue]) -> [u8; 3] {
    assert_eq!(data.len(), 2, "send data has oid + bytes");
    match &data[1] {
        PyValue::Bytes(b) => [b[0], b[1], b[2]],
        other => panic!("expected bytes frame, got {other:?}"),
    }
}

/// The `set_register` 3-byte frame: `[((val >> 16) | reg) & 0xff, ...]`.
fn spi_triple_frame(val: i64, reg: i64) -> [u8; 3] {
    let v = val as u64;
    let r = reg as u64;
    [(((v >> 16) | r) & 0xff) as u8, ((v >> 8) & 0xff) as u8, (v & 0xff) as u8]
}

#[test]
fn idle_timeout_callbacks_match_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc, configparser, configfile").expect("import");
    py.exec(r#"
class R:
    def __init__(self): self.calls = []
    def register_callback(self, cb): self.calls.append(cb)
    def run_all(self):
        for cb in self.calls: cb(None)
class P:
    def __init__(self, reactor): self.handlers = {}; self.reactor = reactor
    def get_reactor(self): return self.reactor
    def register_event_handler(self, event, cb): self.handlers.setdefault(event, []).append(cb)
class M2:
    def __init__(self, fields): self.fields = fields; self.set_calls = []
    def get_fields(self): return self.fields
    def set_register(self, reg, val, print_time=None): self.set_calls.append((reg, val, print_time))
def make_helper2(ini, reactor):
    p = P(reactor)
    raw = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    raw.read_string(ini)
    c = configfile.ConfigWrapper(p, raw, {}, 'tmc2660 stepper_x')
    fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)
    m = M2(fh)
    h = extras.tmc2660.TMC2660CurrentHelper(c, m)
    return (h, p, m, fh)
"#).expect("python shim");
    let ini = "[tmc2660 stepper_x]\nrun_current = 1.0\nsense_resistor = 0.075\nidle_current_percent = 50\n";
    // Python side: invoke _handle_printing/_handle_ready and run the reactor.
    py.exec(&format!("r = R(); h, p, m, fh = make_helper2({ini:?}, r)")).expect("py build");
    let fake_printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer, ini, "tmc2660 stepper_x"));
    let (mcu, _spi, tmc, fields, _fp) = make_tmc_with_fakes(fake_printer.clone(), 1000000.0);
    let _helper = Tmc2660CurrentHelper::new(&config, tmc).expect("helper constructs");
    let rust_handlers = fake_printer.taken_handlers();
    let rust_names: Vec<String> = rust_handlers.iter().map(|(e, _)| e.clone()).collect();
    assert_eq!(rust_names, vec!["idle_timeout:printing".to_string(), "idle_timeout:ready".to_string()]);
    // Idempotent: call the python handlers the same way, then run callbacks.
    py.exec("h._handle_printing(100.0); h._handle_ready(200.0); r.run_all()").expect("py handlers");
    let expected_calls = py.eval("m.set_calls").expect("py calls");
    // Rust side: invoke captured handlers, then pump the reactor.
    let reactor = fake_printer.reactor.clone();
    let end_reactor = reactor.clone();
    reactor.register_timer(move |_| { end_reactor.end(); reactor::NEVER }, 1000.0);
    let mut iter = rust_handlers.into_iter();
    let (_, mut printing) = iter.next().expect("printing handler");
    let (_, mut ready) = iter.next().expect("ready handler");
    printing(&[EventArg::Value(PyValue::Float(100.0))]).expect("printing");
    ready(&[EventArg::Value(PyValue::Float(200.0))]).expect("ready");
    reactor.run();
    // Compare per-call (reg, val, print_time) against the python side.
    let Value::List(calls) = expected_calls else { panic!("py set_calls shape") };
    let rust_sends = mcu.send_log.lock().unwrap().clone();
    assert_eq!(calls.len(), rust_sends.len(), "idle callback call count");
    for (call, (data, opts)) in calls.iter().zip(rust_sends.iter()) {
        let Value::Tuple(pair) = call else { panic!("set_call shape") };
        let Value::Str(reg) = &pair[0] else { panic!("set_call reg") };
        let val = as_int(&pair[1]);
        let reg_int = match reg.as_str() { "SGCSCONF" => 0x0C, "DRVCONF" => 0x0E, other => panic!("{other}") };
        assert_eq!(spi_triple_frame(val, reg_int), extract_frame(data), "frame for {reg}");
        let Value::Float(f) = &pair[2] else { panic!("set_call time") };
        assert_eq!(opts.minclock, (f.to_f64().expect("float") * 1000000.0).trunc() as i64, "minclock");
    }
    // The python handlers do NOT touch vsense while already updated: both
    // callbacks end at the printed full/idle currents. Field state compare:
    check_oracle("idle cs", py.eval("fh.get_field('cs')"),
        v_int(fields.get_field("cs", None, None).expect("cs after idle")));
}

// ====== MCU_TMC2660_SPI ======

#[test]
fn spi_shim_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc").expect("import");
    py.exec(r#"
class FakeReactor:
    def mutex(self): return FakeMutex()
class FakeMutex:
    def __enter__(self): return None
    def __exit__(self, *a): return None
class FakePrinter3:
    def __init__(self, args): self.stored_args = args
    def get_start_args(self): return self.stored_args
class FakeMcuHandle:
    # `mcu.py` `print_time_to_clock`: int(print_time * clock), clock 1 MHz.
    def __init__(self): self.clock_calls = []
    def print_time_to_clock(self, print_time):
        self.clock_calls.append(print_time)
        return int(print_time * 1000000.0)
class FakeSpi:
    def __init__(self, resp):
        self.resp = resp; self.sends = []; self.transfers = []; self.mcu = FakeMcuHandle()
    def spi_send(self, data, minclock=0, reqclock=0): self.sends.append((list(data), minclock, reqclock))
    def spi_transfer(self, data, minclock=0, reqclock=0):
        self.transfers.append((list(data), minclock, reqclock))
        return {'response': self.resp, '#receive_time': 321.0}
    def get_mcu(self): return self.mcu
def make_spi(resp, start_args):
    fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)
    t = extras.tmc2660.MCU_TMC2660_SPI.__new__(extras.tmc2660.MCU_TMC2660_SPI)
    t.printer = FakePrinter3(start_args); t.mutex = FakeReactor().mutex(); t.spi = FakeSpi(resp)
    t.name_to_reg = extras.tmc2660.Registers; t.fields = fh
    return (t, fh)
"#).expect("python shim");
    for (label, py_args, start_args) in [
        ("no start args", "{}", PyValue::Dict(IndexMap::new())),
        ("debugoutput set", "{'debugoutput': '/dev/null'}", PyValue::Dict(IndexMap::from([(PyValue::Str("debugoutput".into()), PyValue::Str("/dev/null".into()))]))),
    ] {
        for reg in ["READRSP@RDSEL0", "READRSP@RDSEL1", "READRSP@RDSEL2"] {
            // Fresh object per register on both sides: `rdsel` starts at 0,
            // so the re-selection `spi_send` happens for RDSEL1 and RDSEL2.
            py.exec(&format!("t, fh = make_spi(bytes([0x11, 0x22, 0x33]), {py_args})"))
                .expect("py build");
            let raw_py = py.eval(&format!("t.get_register_raw({reg:?})"));
            let py_sends = py.eval("t.spi.sends").expect("py sends");
            let py_transfers = py.eval("t.spi.transfers").expect("py transfers");
            let py_rdsel = py.eval("fh.get_field('rdsel')");

            let fake_printer = FakePrinter::new(start_args.clone());
            let (mcu, _spi, tmc, fields, _) = make_tmc_with_fakes(fake_printer, 1000000.0);
            mcu.set_transfer_response(spi_response(&[0x11, 0x22, 0x33], 321.0));
            match tmc.get_register_raw(reg) {
                Ok(raw) => check_oracle(&format!("{label} {reg} raw"), raw_py, pyv(&raw)),
                Err(e) => panic!("{label} {reg} raw: port raised {}: {}", e.class_name(), e.message()),
            }
            let sends = mcu.send_log.lock().unwrap().clone();
            let transfers = mcu.transfer_log.lock().unwrap().clone();
            assert_eq!(py_sends, rust_spi_calls(&sends), "{label} {reg} sends");
            assert_eq!(py_transfers, rust_spi_transfers(&transfers), "{label} {reg} transfers");
            check_oracle(&format!("{label} {reg} rdsel"), py_rdsel,
                v_int(fields.get_field("rdsel", None, None).expect("rdsel")));
        }
    }
    // Unknown register name: same error class and message both sides.
    py.exec("t, fh = make_spi(b'\\x00\\x00\\x00', {})").expect("py build");
    let fake_printer = FakePrinter::new(PyValue::Dict(IndexMap::new()));
    let (_, _, tmc, _, _) = make_tmc_with_fakes(fake_printer, 1000000.0);
    check_oracle_err("not in list", py.eval("t.get_register_raw('NOPE')"), tmc.get_register_raw("NOPE"));
    // set_register frames and clocks, then get_register.
    py.exec("t, fh = make_spi(bytes([0x01, 0x02, 0x03]), {})").expect("py build");
    py.exec("t.set_register('SGCSCONF', 0x123456, 100.0); t.set_register('DRVCONF', 0x0000AB, None)").expect("py sets");
    let expected_sends = py.eval("t.spi.sends").expect("py sends");
    let expected_get = py.eval("t.get_register('READRSP@RDSEL0')");
    let fake_printer = FakePrinter::new(PyValue::Dict(IndexMap::new()));
    let (mcu, _, tmc, _, _) = make_tmc_with_fakes(fake_printer, 1000000.0);
    mcu.set_transfer_response(spi_response(&[0x01, 0x02, 0x03], 321.0));
    tmc.set_register("SGCSCONF", 0x123456, Some(100.0)).expect("rust set");
    tmc.set_register("DRVCONF", 0xAB, None).expect("rust set");
    let rust_sends = mcu.send_log.lock().unwrap().clone();
    assert_eq!(expected_sends, rust_spi_calls(&rust_sends), "set_register frames");
    check_oracle("get_register", expected_get,
        v_int(tmc.get_register("READRSP@RDSEL0").expect("get_register")));
    check_oracle("frequency", py.eval("t.get_tmc_frequency()"), v_none());
    assert!(tmc.get_tmc_frequency().is_none());
    // `get_mcu()` returns the MCU the SPI object holds (`self.spi.get_mcu()`).
    let via_shim: Mcu = tmc.get_mcu();
    let held_by_spi: Mcu = mcu.clone() as Mcu;
    assert!(Arc::ptr_eq(&via_shim, &held_by_spi), "get_mcu returns the spi's mcu");
}

// ====== crate::bus::McuSpi ======

#[test]
fn mcu_spi_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.bus, configparser").expect("import bus");
    py.exec(r#"
class FakeCommandWrapper:
    def __init__(self): self.sent = []
    def send(self, data, minclock=0, reqclock=0): self.sent.append((list(data), minclock, reqclock))
class FakeQueryWrapper(FakeCommandWrapper):
    def send(self, data, minclock=0, reqclock=0):
        self.sent.append((list(data), minclock, reqclock))
        return {'response': b'\xaa\xbb\xcc', '#receive_time': 99.0}
class FakeMcu2:
    def __init__(self):
        self.oid = 0; self.cq = -1; self.config_cmds = []; self.callbacks = []
        self.send_cmd = FakeCommandWrapper(); self.transfer_cmd = FakeQueryWrapper()
    def create_oid(self): self.oid += 1; return self.oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False): self.config_cmds.append((cmd, is_init, on_restart))
    def alloc_command_queue(self): self.cq += 1; return self.cq
    def register_config_callback(self, cb): self.callbacks.append(cb)
    def lookup_command(self, fmt, cq=None): return self.send_cmd
    def lookup_query_command(self, fmt, respfmt, oid=None, cq=None, is_async=False): return self.transfer_cmd
    def try_lookup_command(self, fmt): return FakeCommandWrapper() if 'spi_set_sw_bus' in fmt else None
    def seconds_to_clock(self, t): return int(t * 1000000.0)
    def get_enumerations(self): return {}
    def get_constants(self): return {}
    def get_printer(self): return self.printer
def make_spi(mcu, bus=None, pin='PB1', mode=0, speed=4000000, sw=None, cs_high=False):
    return extras.bus.MCU_SPI(mcu, bus, pin, mode, speed, sw, cs_high)
"#).expect("python shim");
    // __init__ config strings, both pin modes.
    py.exec("m = FakeMcu2()").expect("mcu");
    py.exec("spi = make_spi(m, pin='PB1')").expect("spi");
    check_oracle("ctor cmds", py.eval("m.config_cmds"),
        Value::List(vec![
            Value::Tuple(vec![v_str("config_spi oid=1 pin=PB1 cs_active_high=0"), Value::Bool(false), Value::Bool(false)]),
        ]));
    let _ = &py;
    let spi_test = {
        let fake_printer = FakePrinter::new(PyValue::None);
        let mcu = Arc::new(FakeMcu::new(fake_printer.clone() as Printer, 1000000.0));
        let spi = McuSpi::new(mcu.clone() as Mcu, None, Some("PB1".to_string()), 0, 4000000, None, false).expect("spi");
        assert_eq!(spi.get_oid(), 1);
        assert_eq!(spi.get_command_queue().0, 1);
        assert_eq!(mcu.config_cmds(), vec![
            ("config_spi oid=1 pin=PB1 cs_active_high=0".to_string(), AddConfigCmdOpts { is_init: false, on_restart: false }),
        ]);
        // pre-build spi_send goes to the config command list as hex.
        spi.spi_send(&[0x12, 0x34], SendOpts { minclock: 5, reqclock: 0 }).expect("pre-send");
        assert_eq!(mcu.config_cmds(), vec![
            ("config_spi oid=1 pin=PB1 cs_active_high=0".to_string(), AddConfigCmdOpts::default()),
            ("spi_send oid=1 data=1234".to_string(), AddConfigCmdOpts { is_init: true, on_restart: false }),
        ]);
        // build_config + post-build send/transfer.
        mcu.commands_run();
        spi.spi_send(&[0xDE, 0xAD], SendOpts { minclock: 7, reqclock: 0 }).expect("send");
        let sends = mcu.send_log.lock().unwrap();
        let PyValue::Bytes(frame) = &sends[0].0[1] else { panic!("frame bytes") };
        assert_eq!(frame, &vec![0xDEu8, 0xADu8]);
        assert_eq!(sends[0].1.minclock, 7);
        let resp = spi.spi_transfer(&[1, 2, 3], QuerySendOpts { minclock: 0, reqclock: 0, retry: true }).expect("transfer");
        assert_eq!(resp, PyValue::Dict(IndexMap::from([
            (PyValue::Str("response".into()), PyValue::Bytes(vec![0xAA, 0xBB, 0xCC])),
            (PyValue::Str("#receive_time".into()), PyValue::Float(1234.5)),
        ])));
        spi.setup_shutdown_msg(&[0xDE, 0xAD, 0xBE, 0xEF]).expect("shutdown");
        let cmds = mcu.config_cmds();
        assert_eq!(cmds.iter().find(|(c, _)| c.starts_with("config_spi_shutdown")).map(|(c, _)| c.as_str()),
            Some("config_spi_shutdown oid=2 spi_oid=1 shutdown_msg=deadbeef"));
        spi
    };
    let _ = spi_test;
    // build_config strings via oracle: run the same build in the shim fakes.
    py.exec("m2 = FakeMcu2(); s2 = make_spi(m2); s2.build_config()").expect("py build");
    check_oracle("build config cmds (py)", py.eval("m2.config_cmds"),
        Value::List(vec![
            Value::Tuple(vec![v_str("config_spi oid=1 pin=PB1 cs_active_high=0"), Value::Bool(false), Value::Bool(false)]),
            Value::Tuple(vec![v_str("spi_set_bus oid=1 spi_bus=0 mode=0 rate=4000000"), Value::Bool(false), Value::Bool(false)]),
        ]));
    // Rust build strings, same scenario.
    let fake_printer = FakePrinter::new(PyValue::None);
    let mcu = Arc::new(FakeMcu::new(fake_printer.clone() as Printer, 1000000.0));
    let spi = McuSpi::new(mcu.clone() as Mcu, None, Some("PB1".to_string()), 0, 4000000, None, false).expect("spi");
    mcu.commands_run();
    assert_eq!(mcu.config_cmds(), vec![
        ("config_spi oid=1 pin=PB1 cs_active_high=0".to_string(), AddConfigCmdOpts::default()),
        ("spi_set_bus oid=1 spi_bus=0 mode=0 rate=4000000".to_string(), AddConfigCmdOpts::default()),
    ]);
    assert_eq!(spi.get_oid(), 1);
    // Software-bus ticks path: `pulse_ticks = seconds_to_clock(1./speed)`
    // (bus.py:93), compared against the reference build.
    py.exec("m3 = FakeMcu2(); s3 = make_spi(m3, sw=['PBMISO', 'PBMS', 'PBSCLK']); s3.build_config()").expect("py sw build");
    let fake_printer = FakePrinter::new(PyValue::None);
    let mut mcu2 = FakeMcu::new(fake_printer.clone() as Printer, 1000000.0);
    mcu2.has_sw_bus = true;
    let mcu2 = Arc::new(mcu2);
    let spi = McuSpi::new(mcu2.clone() as Mcu, None, Some("PB1".to_string()), 0, 4000000,
        Some(["PBMISO".to_string(), "PBMS".to_string(), "PBSCLK".to_string()]), false).expect("sw spi");
    mcu2.commands_run();
    let rust_cmds: Vec<Value> = mcu2.config_cmds().iter().map(|(c, o)| {
        Value::Tuple(vec![v_str(c), Value::Bool(o.is_init), Value::Bool(o.on_restart)])
    }).collect();
    check_oracle("sw-bus build cmds", py.eval("m3.config_cmds"), Value::List(rust_cmds));
    let _ = spi;
}

#[test]
fn mcu_spi_from_config_errors() {
    // Without a `pins` printer object the lookup fails, matching
    // klippy.py:84's config_error.
    let fake_printer = FakePrinter::new(PyValue::None);
    let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer,
        "[tmc2660 stepper_x]\ncs_pin = PB1\n", "tmc2660 stepper_x"));
    let res = mcu_spi_from_config(&config, 0, "cs_pin", 4000000, None, false);
    let e = res.err().expect("must fail");
    assert_eq!(e.class_name(), "Error");
    assert_eq!(e.message(), "Unknown config object 'pins'");
}

// ====== TMC2660 driver-config block ======

#[test]
fn driver_config_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec("import extras.tmc2660, extras.tmc, configparser, configfile").expect("import");
    py.exec(r#"
def setup_driver(ini):
    raw = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    raw.read_string(ini)
    c = configfile.ConfigWrapper(None, raw, {}, 'tmc2660 stepper_x')
    fh = extras.tmc.FieldHelper(extras.tmc2660.Fields, extras.tmc2660.SignedFields, extras.tmc2660.FieldFormatters)
    scf = fh.set_config_field
    scf(c, "tbl", 2); scf(c, "rndtf", 0); scf(c, "hdec", 0); scf(c, "chm", 0)
    scf(c, "hend", 3); scf(c, "hstrt", 3); scf(c, "toff", 4)
    if not fh.get_field("chm"):
        if (fh.get_field("hstrt") + fh.get_field("hend")) > 15:
            raise configparser.Error("driver_HEND + driver_HSTRT must be <= 15")
    scf(c, "seimin", 0); scf(c, "sedn", 0); scf(c, "semax", 0); scf(c, "seup", 0); scf(c, "semin", 0)
    scf(c, "sfilt", 0); scf(c, "sgt", 0)
    scf(c, "slph", 0); scf(c, "slpl", 0); scf(c, "diss2g", 0); scf(c, "ts2g", 3)
    return fh
"#).expect("python shim");
    let cases = [
        ("defaults", "[tmc2660 stepper_x]\n"),
        ("tbl override", "[tmc2660 stepper_x]\ndriver_tbl = 3\n"),
        ("chm off hstrt/hend too big", "[tmc2660 stepper_x]\ndriver_hstrt = 8\ndriver_hend = 8\n"),
        ("chm on hstrt/hend too big", "[tmc2660 stepper_x]\ndriver_chm = 1\ndriver_hstrt = 8\ndriver_hend = 8\n"),
        ("boolean field", "[tmc2660 stepper_x]\ndriver_diss2g = 1\n"),
        ("signed field", "[tmc2660 stepper_x]\ndriver_sgt = -5\n"),
        ("signed field edge", "[tmc2660 stepper_x]\ndriver_sgt = -64\n"),
        ("sgt above max", "[tmc2660 stepper_x]\ndriver_sgt = 64\n"),
        ("tbl above max", "[tmc2660 stepper_x]\ndriver_tbl = 4\n"),
    ];
    for (label, ini) in cases {
        let fake_printer = FakePrinter::new(PyValue::None);
        let config: Config = Arc::new(FakeConfig::new(fake_printer.clone() as Printer, ini, "tmc2660 stepper_x"));
        let fields = FieldHelper::new(fields(), signed_fields(), field_formatters(), None);
        let rust_res = Tmc2660::setup_driver_config(&config, &fields);
        let py_expr = format!("setup_driver({ini:?})");
        match py.eval(&py_expr) {
            Ok(_) => {
                rust_res.unwrap_or_else(|e| panic!("{label}: port raised {e:?}"));
                py.exec(&format!("fh_last = setup_driver({ini:?})")).expect("bind");
                let names = ["tbl", "hdec", "hend", "hstrt", "toff", "cs", "vsense", "sgt", "sfilt", "ts2g"];
                for name in names {
                    check_oracle(&format!("{label} field {name}"),
                        py.eval(&format!("fh_last.get_field({name:?})")),
                        v_int(fields.get_field(name, None, None).expect("field")));
                }
            }
            Err(exc) => {
                let e = match rust_res { Err(e) => e, Ok(_) => panic!("{label}: port succeeded, oracle raised {exc:?}") };
                assert_eq!(e.class_name(), exc.class_name, "{label}");
                assert_eq!(e.message(), exc.message, "{label}");
            }
        }
    }
    // Factory surface.
    let f: ModuleFactory = module_factory();
    assert!(f.load_config_prefix.is_some());
    assert!(f.load_config.is_none());
}

/// `endstop_phase` and `angle` look the driver up as `tmc2660 <stepper>` and
/// call `get_phase_offset` on it, so the registered printer object must carry
/// the interface. Constructing a full `TMC2660` needs the `pins`, `gcode`,
/// `toolhead` and `stepper_enable` objects, so the property is asserted at
/// the type level instead.
#[test]
fn driver_object_provides_phase_offset() {
    fn assert_driver<T: TMCCommandHelperApi + klippy_api::PrinterObject>() {}
    assert_driver::<Tmc2660>();
}

// ====== TMC2660 constructed through load_config_prefix ======

/// A pins object whose `lookup_pin` hands out pins of one MCU and logs the
/// call like the Python fake of `SETUP_E2E`.
struct FakePins {
    log: Log,
    mcu: Mcu,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        fn py_bool(b: bool) -> &'static str {
            if b { "True" } else { "False" }
        }
        push_log(
            &self.log,
            format!(
                "lookup_pin|{}|{}|{}|{}",
                str_repr(pin_desc),
                py_bool(opts.can_invert),
                py_bool(opts.can_pullup),
                match &opts.share_type {
                    Some(s) => str_repr(s),
                    None => "None".to_string(),
                }
            ),
        );
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used")
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) -> klippy_api::KResult<()> {
        push_log(&self.log, format!("reset_pin_sharing|{}", str_repr(&pin_params.pin)));
        Ok(())
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

/// Logs every `register_mux_command`; nothing else of `gcode` is reached
/// while the driver is built.
struct FakeGcode {
    log: Log,
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("register_command is not used")
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        push_log(
            &self.log,
            format!(
                "register_mux_command|{cmd}|{key}|{}|{}",
                value.map_or_else(|| "None".to_string(), str_repr),
                desc.map_or_else(|| "None".to_string(), str_repr)
            ),
        );
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used")
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

/// `TMCCommandHelper.__init__` only stores `stepper_enable`.
struct FakeStepperEnable;

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        panic!("register_stepper is not used")
    }
    fn set_motors_enable(&self, _stepper_names: &[String], _enable: bool) -> KResult<bool> {
        panic!("set_motors_enable is not used")
    }
    fn motor_off(&self) -> KResult<()> {
        panic!("motor_off is not used")
    }
    fn lookup_enable(&self, _name: &str) -> KResult<Arc<EnableTracking>> {
        panic!("lookup_enable is not used")
    }
    fn get_steppers(&self) -> Vec<String> {
        panic!("get_steppers is not used")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used")
    }
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

const SETUP_E2E: &str = r#"
import configparser, configfile, extras.tmc2660

class E2eMcu:
    def __init__(self):
        self.oid = 0; self.cq = -1; self.cmds = []
    def create_oid(self):
        self.oid += 1
        return self.oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.cmds.append((cmd, is_init, on_restart))
    def alloc_command_queue(self):
        self.cq += 1
        return self.cq
    def register_config_callback(self, cb):
        pass

class E2ePins:
    def __init__(self, mcu, log):
        self.mcu = mcu; self.log = log
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        self.log.append("lookup_pin|%r|%r|%r|%r" % (pin_desc, can_invert, can_pullup, share_type))
        return {'chip': self.mcu, 'chip_name': 'mcu', 'pin': pin_desc, 'invert': 0,
                'pullup': 0, 'share_type': share_type}
    def reset_pin_sharing(self, pin_params):
        self.log.append("reset_pin_sharing|%r" % (pin_params['pin'],))

class E2eGcode:
    def __init__(self, log):
        self.log = log
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.log.append("register_mux_command|%s|%s|%r|%r" % (cmd, key, value, desc))

class E2eReactor:
    def mutex(self, is_locked=False):
        return object()

class E2ePrinter:
    def __init__(self, log):
        self.log = log
        self.mcu = E2eMcu()
        self.objects = {"pins": E2ePins(self.mcu, log), "gcode": E2eGcode(log),
                        "stepper_enable": object()}
        self.reactor = E2eReactor()
    def get_reactor(self):
        return self.reactor
    def get_start_args(self):
        return {}
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=configfile.sentinel):
        self.log.append("load_object|%s" % (section,))
        return self.lookup_object(section, default)
    def register_event_handler(self, event, cb):
        self.log.append("register_event_handler|%s" % (event,))

class LogConfig(configfile.ConfigWrapper):
    def __init__(self, printer, fileconfig, access_tracking, section, log):
        configfile.ConfigWrapper.__init__(self, printer, fileconfig, access_tracking, section)
        self.log = log
    def _get_wrapper(self, parser, option, default, minval=None, maxval=None,
                     above=None, below=None, note_valid=True):
        kind = parser.__name__
        if default is configfile.sentinel:
            shown = 'required'
        elif kind == 'getboolean' and default is not None:
            shown = repr(bool(default))
        else:
            shown = repr(default)
        self.log.append("%s|%s|%s|%s|%r|%r|%r|%r" % (
            kind, self.section, option, shown, minval, maxval, above, below))
        return configfile.ConfigWrapper._get_wrapper(
            self, parser, option, default, minval, maxval, above, below, note_valid)
    def getsection(self, section):
        return LogConfig(self.printer, self.fileconfig, self.access_tracking, section, self.log)

e2e_obj = None
def run_e2e(ini):
    global e2e_obj
    log = []
    p = E2ePrinter(log)
    raw = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    raw.read_string(ini)
    cfg = LogConfig(p, raw, {}, 'tmc2660 stepper_x', log)
    err = obj = None
    try:
        obj = extras.tmc2660.load_config_prefix(cfg)
    except Exception as e:
        err = (type(e).__name__, str(e))
    e2e_obj = obj
    status = None if obj is None else obj.get_status(0.)
    phase = None if obj is None else obj.get_phase_offset()
    return (err, log, p.mcu.cmds, status, phase)
"#;

fn phase_value(phase: (Option<i64>, i64)) -> Value {
    Value::Tuple(vec![phase.0.map_or(Value::Null, v_int), v_int(phase.1)])
}

/// Builds the driver with `load_config_prefix` (`via_factory`) or with
/// `Tmc2660::new`, against fakes mirroring `SETUP_E2E`. Returns the
/// `(error, log, mcu config commands, status, phase)` tuple of `run_e2e`
/// and, for the direct build, the `FieldHelper.registers` items.
fn rust_e2e(ini: &str, via_factory: bool) -> (Value, Option<Value>) {
    let printer = FakePrinter::new(PyValue::Dict(IndexMap::new()));
    let log = printer.log.clone();
    let fake_mcu = Arc::new(FakeMcu::new(printer.clone() as Printer, 1000000.0));
    let mcu: Mcu = fake_mcu.clone();
    let objects: Vec<(&str, ObjectRef)> = vec![
        ("pins", Arc::new(FakePins { log: log.clone(), mcu })),
        ("gcode", Arc::new(FakeGcode { log: log.clone() })),
        ("stepper_enable", Arc::new(FakeStepperEnable)),
    ];
    for (name, object) in objects {
        printer.add_object(name, object).expect("add object");
    }
    let config: Config = Arc::new(FakeConfig::with_log(
        printer.clone() as Printer,
        ini,
        "tmc2660 stepper_x",
        log.clone(),
    ));
    let mut registers = None;
    let error_value =
        |e: &KlippyError| Value::Tuple(vec![v_str(e.class_name()), v_str(e.message())]);
    let (err, status, phase) = if via_factory {
        let load = module_factory().load_config_prefix.expect("load_config_prefix");
        match load(&config) {
            Err(e) => (error_value(&e), Value::Null, Value::Null),
            Ok(obj) => {
                let obj = obj.expect("load_config_prefix returns the object");
                let status = obj.get_status(0.0).unwrap().expect("get_status");
                let helper = klippy_api::object::interface::<dyn TMCCommandHelperApi>(&obj)
                    .expect("the printer object declares TMCCommandHelperApi");
                (Value::Null, pyv(&status), phase_value(helper.get_phase_offset()))
            }
        }
    } else {
        match Tmc2660::new(&config) {
            Err(e) => (error_value(&e), Value::Null, Value::Null),
            Ok(t) => {
                registers = Some(Value::List(
                    t.fields()
                        .registers()
                        .iter()
                        .map(|(k, v)| Value::Tuple(vec![v_str(k), v_int(*v)]))
                        .collect(),
                ));
                (Value::Null, pyv(&t.get_status(0.0)), phase_value(t.get_phase_offset()))
            }
        }
    };
    let cmds = fake_mcu
        .config_cmds()
        .iter()
        .map(|(c, o)| Value::Tuple(vec![v_str(c), Value::Bool(o.is_init), Value::Bool(o.on_restart)]))
        .collect();
    let lines = log.lock().unwrap().iter().map(|l| v_str(l)).collect();
    (Value::Tuple(vec![err, Value::List(lines), Value::List(cmds), status, phase]), registers)
}

const E2E_STEPPER: &str = "[stepper_x]\nmicrosteps = 16\n\n";

#[test]
fn load_config_prefix_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping tmc2660 oracle: set KLIPPY_PYTHON to a CPython 3.8.2 binary");
        return;
    };
    py.exec(SETUP_E2E).expect("python fakes");
    let base = "[tmc2660 stepper_x]\ncs_pin = PB1\nrun_current = 0.5\nsense_resistor = 0.075\n";
    let cases: Vec<(&str, String)> = vec![
        ("defaults", format!("{E2E_STEPPER}{base}")),
        ("idle current", format!("{E2E_STEPPER}{base}idle_current_percent = 50\n")),
        ("idle current 100", format!("{E2E_STEPPER}{base}idle_current_percent = 100\n")),
        ("spi options", format!("{E2E_STEPPER}{base}spi_speed = 2000000\nspi_bus = spi1\n")),
        ("spi speed too low", format!("{E2E_STEPPER}{base}spi_speed = 1000\n")),
        (
            "cs_pin None",
            format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = None\nrun_current = 0.5\nsense_resistor = 0.075\n"),
        ),
        ("missing cs_pin", format!("{E2E_STEPPER}[tmc2660 stepper_x]\nrun_current = 0.5\nsense_resistor = 0.075\n")),
        ("missing run_current", format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = PB1\nsense_resistor = 0.075\n")),
        ("missing sense_resistor", format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = PB1\nrun_current = 0.5\n")),
        ("run_current too high", format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = PB1\nrun_current = 2.5\nsense_resistor = 0.075\n")),
        ("run_current too low", format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = PB1\nrun_current = 0.05\nsense_resistor = 0.075\n")),
        ("high current vsense", format!("{E2E_STEPPER}[tmc2660 stepper_x]\ncs_pin = PB1\nrun_current = 2.4\nsense_resistor = 0.05\n")),
        ("idle percent too high", format!("{E2E_STEPPER}{base}idle_current_percent = 101\n")),
        ("missing stepper section", base.to_string()),
        ("microsteps missing", format!("[stepper_x]\n\n{base}")),
        ("microsteps invalid", format!("[stepper_x]\nmicrosteps = 3\n\n{base}")),
        ("microsteps 256 no interpolate", format!("[stepper_x]\nmicrosteps = 256\n\n{base}interpolate = False\n")),
        (
            "driver fields",
            format!(
                "{E2E_STEPPER}{base}driver_tbl = 3\ndriver_rndtf = 1\ndriver_hdec = 2\ndriver_hend = 5\ndriver_hstrt = 4\ndriver_toff = 7\ndriver_seimin = 1\ndriver_sedn = 2\ndriver_semax = 9\ndriver_seup = 3\ndriver_semin = 6\ndriver_sfilt = 1\ndriver_sgt = -20\ndriver_slph = 2\ndriver_slpl = 1\ndriver_diss2g = 1\ndriver_ts2g = 1\n"
            ),
        ),
        ("chm constant toff", format!("{E2E_STEPPER}{base}driver_chm = 1\ndriver_hstrt = 7\ndriver_hend = 15\n")),
        ("hend + hstrt too big", format!("{E2E_STEPPER}{base}driver_hend = 8\ndriver_hstrt = 8\n")),
        ("sgt too low", format!("{E2E_STEPPER}{base}driver_sgt = -65\n")),
        ("tbl too high", format!("{E2E_STEPPER}{base}driver_tbl = 4\n")),
        ("diss2g unparseable", format!("{E2E_STEPPER}{base}driver_diss2g = maybe\n")),
    ];
    for (label, ini) in &cases {
        // The registry entry point: object, status, `get_phase_offset`
        // through the declared `TMCCommandHelperApi`, log and MCU commands.
        let expected = py.eval(&format!("run_e2e({ini:?})"));
        let (via_factory, _) = rust_e2e(ini, true);
        check_oracle(&format!("{label} (load_config_prefix)"), expected, via_factory);
        // `Tmc2660::new` is the constructor behind it and exposes the
        // `FieldHelper.registers` the Python object holds.
        let expected = py.eval(&format!("run_e2e({ini:?})"));
        let expected_regs = py
            .eval("None if e2e_obj is None else list(e2e_obj.fields.registers.items())")
            .expect("registers");
        let (direct, registers) = rust_e2e(ini, false);
        check_oracle(&format!("{label} (Tmc2660::new)"), expected, direct);
        match registers {
            Some(regs) => assert_eq!(expected_regs, regs, "{label} registers"),
            None => assert_eq!(expected_regs, Value::Null, "{label} registers"),
        }
    }
}
