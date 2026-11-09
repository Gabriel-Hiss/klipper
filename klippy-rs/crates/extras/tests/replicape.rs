//! Parity tests for `extras::replicape` (port of `klippy/extras/replicape.py`).
//!
//! Every expected value comes from the frozen reference through `pyoracle`.
//! A scenario (config options, MCU flags and a list of operations) runs once
//! against `replicape.load_config` in CPython and once against the port.
//! The reference side uses the real `configfile.ConfigWrapper`, the real
//! `pins.PrinterPins` (and its `PinResolver`) and the real `extras.bus.MCU_SPI`
//! over minimal fakes for the printer and the MCU. The port side uses the real
//! `klippy_core::pins::PrinterPins` and `extras::bus::McuSpi` over equivalent
//! fakes of the contract traits. Both sides append the same line format to one
//! event log, so the comparison covers in call order:
//!
//! - every MCU call the module makes (config commands with `is_init`,
//!   `alloc_command_queue`, `lookup_command`, `lookup_query_command`,
//!   `seconds_to_clock`, `print_time_to_clock`, `get_constant_float`,
//!   `request_move_queue_slot`, `setup_pin`) and every command send with its
//!   data and `minclock` / `reqclock`;
//! - every call on the enable pin and on the host PWM pin (floats by `repr`);
//! - the owned log line (`Ignoring pca9685 cycle time ...`);
//! - errors by class name and message (config, pins, MCU) and the result of
//!   each operation;
//! - the Replicape state (`enabled_channels`, `stepper_dacs`, `sr_disabled`,
//!   `sr_enabled`, `pins`) after the scenario.
//!
//! The reference's `register_async_callback` is faked as a queue that the
//! scenario drains at its end; the port's callbacks run on the real `Reactor`
//! (a timer asks the reactor to end after it processed them), so the
//! `note_pwm_enable` flow triggered by `set_pwm` is compared too.
//!
//! Not compared because the pin traits have no error channel: the
//! `pins.error` of `setup_cycle_time(hardware_pwm=True)` on a PCA9685 pin and
//! the `AttributeError` of `set_pwm` before the config callback ran.
//!
//! Integration owner runs: `cargo test -p extras --test replicape`.

use std::sync::{Arc, LazyLock};

use extras::replicape::{
    load_config, module_factory, step_config, Replicape, PIN_MIN_TIME, REPLICAPE_MAX_CURRENT,
    REPLICAPE_PCA9685_ADDRESS, REPLICAPE_PCA9685_BUS, REPLICAPE_PCA9685_CYCLE_TIME, SERVO_PINS,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, ChipApi, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback,
    DigitalOutPin, DigitalOutPinApi, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams,
    PinsApi, PwmPin, PwmPinApi, QuerySendOpts, SendOpts, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{downcast_arc, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::EventArg;
use klippy_core::pins::PrinterPins;
use pycompat::text::{float_from_string, float_repr};
use pycompat::PyValue;
use kos::sync::Mutex;
use pyoracle::Oracle;
use reactor::{Reactor, NEVER};
use recording::Value;

// ---------------------------------------------------------------------------
// Event log shared by every fake of the port side.
// ---------------------------------------------------------------------------

static LOG: LazyLock<Mutex<Vec<String>>> = LazyLock::new(|| Mutex::new(Vec::new()));

fn log_line(line: String) {
    LOG.lock().push(line);
}

fn install_log_sink() {
    // The sink is process wide; installing it twice is harmless.
    klippy_api::log::set_sink(Box::new(|rec| {
        log_line(format!("log:{}", rec.message));
    }));
}

fn b2i(b: bool) -> i32 {
    i32::from(b)
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}

/// The `norm()` of the Python fakes: ints by `str`, byte sequences as hex.
fn norm(v: &PyValue) -> String {
    match v {
        PyValue::Int(n) => n.0.to_string(),
        PyValue::Bytes(b) => format!("b:{}", hex(b)),
        other => panic!("unexpected command argument {}", other.repr()),
    }
}

// ---------------------------------------------------------------------------
// Port side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        panic!("get_start_args is not used by replicape")
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("get_state_message is not used by replicape")
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("update_error_msg is not used by replicape")
    }
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(obj) => Ok(Some(Arc::clone(obj))),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by replicape")
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by replicape")
    }
    fn run(&self) -> Option<String> {
        panic!("run is not used by replicape")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("set_rollover_info is not used by replicape")
    }
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        panic!("unexpected shutdown: {msg}")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by replicape")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {
        panic!("register_event_handler is not used by replicape")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("send_event is not used by replicape")
    }
    fn request_exit(&self, _result: &str) {
        panic!("request_exit is not used by replicape")
    }
}

klippy_api::printer_object!(FakePrinter);

/// `configfile.ConfigWrapper` over an option list: the getters the module
/// uses, with the messages of `ConfigWrapper._get_wrapper` / `getchoice`.
struct FakeConfig {
    printer: Printer,
    section: String,
    options: Vec<(String, String)>,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<&str> {
        self.options.iter().find(|(k, _)| k == option).map(|(_, v)| v.as_str())
    }
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
    }
    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        ))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.raw(option) {
            Some(v) => Ok(Some(v.to_string())),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(default) => Ok(default),
            },
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by replicape")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(default) => Ok(default),
            };
        };
        let v = float_from_string(raw).map_err(|_| self.unparsable(option))?;
        let section = &self.section;
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{section}' must have minimum of {}",
                    float_repr(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{section}' must have maximum of {}",
                    float_repr(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{section}' must be above {}",
                    float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{section}' must be below {}",
                    float_repr(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            Some(v) => match v.to_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Ok(Some(true)),
                "0" | "no" | "false" | "off" => Ok(Some(false)),
                _ => Err(self.unparsable(option)),
            },
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(default) => Ok(default),
            },
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let entries: Vec<(String, PyValue)> = match choices {
            PyValue::List(items) => items
                .iter()
                .map(|item| match item {
                    PyValue::Str(s) => (s.clone(), item.clone()),
                    other => panic!("unexpected choice {}", other.repr()),
                })
                .collect(),
            PyValue::Dict(d) => d
                .iter()
                .map(|(k, v)| match k {
                    PyValue::Str(s) => (s.clone(), v.clone()),
                    other => panic!("unexpected choice key {}", other.repr()),
                })
                .collect(),
            other => panic!("unexpected choices {}", other.repr()),
        };
        let c = match self.raw(option) {
            Some(v) => v.to_string(),
            None => match opts.default {
                None => return Err(self.missing(option)),
                Some(Some(PyValue::Str(s))) => s,
                Some(_) => panic!("unexpected getchoice default"),
            },
        };
        match entries.into_iter().find(|(k, _)| *k == c) {
            Some((_, v)) => Ok(v),
            None => Err(KlippyError::Config(format!(
                "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
                self.section
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by replicape")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by replicape")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by replicape")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by replicape")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by replicape")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by replicape")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by replicape")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options is not used by replicape")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate is not used by replicape")
    }
}

/// `MCU_digital_out` stand-in for the enable pin (and the ordinary MCU pins).
struct FakeDigitalOut {
    mcu: Mcu,
    pin: String,
}

impl DigitalOutPinApi for FakeDigitalOut {
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }
    fn setup_max_duration(&self, max_duration: f64) {
        log_line(format!("do:{}:setup_max_duration:{}", self.pin, float_repr(max_duration)));
    }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        log_line(format!(
            "do:{}:setup_start_value:{}:{}",
            self.pin,
            b2i(start_value),
            b2i(shutdown_value)
        ));
    }
    fn set_digital(&self, print_time: f64, value: bool) {
        log_line(format!(
            "do:{}:set_digital:{}:{}",
            self.pin,
            float_repr(print_time),
            b2i(value)
        ));
    }
}

/// `MCU_pwm` stand-in for the host PWM behind a servo pin.
struct FakeMcuPwm {
    mcu: Mcu,
    pin: String,
}

impl PwmPinApi for FakeMcuPwm {
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.mcu)
    }
    fn setup_max_duration(&self, max_duration: f64) {
        log_line(format!("pwm:{}:setup_max_duration:{}", self.pin, float_repr(max_duration)));
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        log_line(format!(
            "pwm:{}:setup_cycle_time:{}:{}",
            self.pin,
            float_repr(cycle_time),
            py_bool(hardware_pwm.unwrap_or(false))
        ));
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        log_line(format!(
            "pwm:{}:setup_start_value:{}:{}",
            self.pin,
            float_repr(start_value),
            float_repr(shutdown_value)
        ));
    }
    fn next_aligned_print_time(&self, _print_time: f64, _allow_early: Option<f64>) -> f64 {
        panic!("servo_pwm has no next_aligned_print_time")
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        log_line(format!(
            "pwm:{}:set_pwm:{}:{}",
            self.pin,
            float_repr(print_time),
            float_repr(value)
        ));
    }
}

struct FakeCommand {
    msgformat: String,
}

impl CommandWrapperApi for FakeCommand {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let data: Vec<String> = data.iter().map(norm).collect();
        log_line(format!(
            "send:{}:{}:{}:{}",
            self.msgformat,
            data.join(","),
            opts.minclock,
            opts.reqclock
        ));
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        panic!("send_wait_ack is not used by replicape")
    }
    fn get_command_tag(&self) -> i64 {
        panic!("get_command_tag is not used by replicape")
    }
}

struct FakeQueryCommand;

impl CommandQueryWrapperApi for FakeQueryCommand {
    fn send(&self, _data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        panic!("query send is not used by replicape")
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by replicape")
    }
}

struct FakeMcu {
    printer: Printer,
    name: String,
    fileoutput: bool,
    pca_max: Option<f64>,
    oid: Mutex<i64>,
    cq: Mutex<i64>,
    callbacks: Mutex<Vec<ConfigCallback>>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn is_fileoutput(&self) -> bool {
        self.fileoutput
    }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        log_line(format!("mcu:setup_pin:{pin_type}:{}:{}", pin_params.pin, pin_params.invert));
        let mcu: Mcu = Arc::clone(&pin_params.chip);
        let pin = pin_params.pin.clone();
        match pin_type {
            "pwm" => Ok(Pin::Pwm(Arc::new(FakeMcuPwm { mcu, pin }))),
            "digital_out" => Ok(Pin::DigitalOut(Arc::new(FakeDigitalOut { mcu, pin }))),
            other => panic!("pin type {other} is not used by replicape"),
        }
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut oid = self.oid.lock();
        *oid += 1;
        Ok(*oid)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        log_line(format!(
            "mcu:add_config_cmd:{cmd}:{}:{}",
            b2i(opts.is_init),
            b2i(opts.on_restart)
        ));
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        log_line("mcu:request_move_queue_slot".to_string());
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by replicape")
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        log_line(format!("mcu:seconds_to_clock:{}", float_repr(time)));
        (time * 1000000.) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by replicape")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by replicape")
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        let cq = cq.map_or("None".to_string(), |q| q.0.to_string());
        log_line(format!("mcu:lookup_command:{msgformat}:{cq}"));
        Ok(Arc::new(FakeCommand { msgformat: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        log_line(format!(
            "mcu:lookup_query_command:{msgformat}:{respformat}:{}:{}:{}",
            opts.oid.map_or("None".to_string(), |o| o.to_string()),
            opts.cq.map_or("None".to_string(), |q| q.0.to_string()),
            py_bool(opts.is_async)
        ));
        Ok(Arc::new(FakeQueryCommand))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by replicape")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut cq = self.cq.lock();
        *cq += 1;
        log_line(format!("mcu:alloc_command_queue:{}", *cq));
        CommandQueue(*cq)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by replicape")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by replicape")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        log_line(format!("mcu:get_constant_float:{name}"));
        match (name, self.pca_max) {
            ("PCA9685_MAX", Some(v)) => Ok(v),
            _ => Err(KlippyError::Mcu(format!("Firmware constant '{name}' not found"))),
        }
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        log_line(format!("mcu:print_time_to_clock:{}", float_repr(print_time)));
        (print_time * 1000000.) as i64
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by replicape")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by replicape")
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        panic!("clock32_to_clock64 is not used by replicape")
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by replicape")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by replicape")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by replicape")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by replicape")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

/// The chip registered under `mcu` (`pins.chips['mcu']` is the MCU itself in Python).
struct McuChip(Mcu);

impl ChipApi for McuChip {
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.0.setup_pin(pin_type, pin_params)
    }
    fn get_mcu(&self) -> Mcu {
        Arc::clone(&self.0)
    }
}

// ---------------------------------------------------------------------------
// Scenarios.
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Op {
    /// `ppins.setup_pin(ptype, desc)`, result kept as handle `h`.
    Pin { ptype: &'static str, desc: &'static str, h: usize },
    /// Run the MCU config callbacks in registration order.
    RunConfig,
    SetPwm(usize, f64, f64),
    SetDigital(usize, f64, bool),
    DoStart(usize, bool, bool),
    PwmStart(usize, f64, f64),
    MaxDuration(usize, f64),
    Cycle(usize, f64, bool),
    NextAligned(usize, f64),
    GetMcu(usize),
    NoteStart(i64, f64, f64),
    NoteEnable(f64, i64, bool),
    Reserve(&'static str, &'static str),
    State,
    /// Run the queued async callbacks, then log the state. Last operation only.
    Asyncs,
}

fn pyf(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x.is_infinite() {
        if x > 0. { "float('inf')" } else { "float('-inf')" }.to_string()
    } else {
        format!("{x:?}")
    }
}

fn pyb(b: bool) -> &'static str {
    py_bool(b)
}

impl Op {
    fn py(&self) -> String {
        match self {
            Op::Pin { ptype, desc, h } => format!("('pin', '{ptype}', '{desc}', {h})"),
            Op::RunConfig => "('run_config',)".to_string(),
            Op::SetPwm(h, t, v) => format!("('set_pwm', {h}, {}, {})", pyf(*t), pyf(*v)),
            Op::SetDigital(h, t, v) => format!("('set_digital', {h}, {}, {})", pyf(*t), pyb(*v)),
            Op::DoStart(h, a, b) => format!("('do_start', {h}, {}, {})", pyb(*a), pyb(*b)),
            Op::PwmStart(h, a, b) => format!("('pwm_start', {h}, {}, {})", pyf(*a), pyf(*b)),
            Op::MaxDuration(h, d) => format!("('max_duration', {h}, {})", pyf(*d)),
            Op::Cycle(h, ct, hw) => format!("('cycle', {h}, {}, {})", pyf(*ct), pyb(*hw)),
            Op::NextAligned(h, t) => format!("('next_aligned', {h}, {})", pyf(*t)),
            Op::GetMcu(h) => format!("('get_mcu', {h})"),
            Op::NoteStart(ch, a, b) => format!("('note_start', {ch}, {}, {})", pyf(*a), pyf(*b)),
            Op::NoteEnable(t, ch, e) => format!("('note_enable', {}, {ch}, {})", pyf(*t), pyb(*e)),
            Op::Reserve(pin, name) => format!("('reserve', '{pin}', '{name}')"),
            Op::State => "('state',)".to_string(),
            Op::Asyncs => "('asyncs',)".to_string(),
        }
    }

    fn kind(&self) -> &'static str {
        match self {
            Op::Pin { .. } => "pin",
            Op::RunConfig => "run_config",
            Op::SetPwm(..) => "set_pwm",
            Op::SetDigital(..) => "set_digital",
            Op::DoStart(..) => "do_start",
            Op::PwmStart(..) => "pwm_start",
            Op::MaxDuration(..) => "max_duration",
            Op::Cycle(..) => "cycle",
            Op::NextAligned(..) => "next_aligned",
            Op::GetMcu(..) => "get_mcu",
            Op::NoteStart(..) => "note_start",
            Op::NoteEnable(..) => "note_enable",
            Op::Reserve(..) => "reserve",
            Op::State => "state",
            Op::Asyncs => "asyncs",
        }
    }
}

#[derive(Clone)]
struct Case {
    name: &'static str,
    options: Vec<(String, String)>,
    fileoutput: bool,
    pca_max: Option<f64>,
    preregister_chip: bool,
    no_pins_object: bool,
    ops: Vec<Op>,
}

fn opt(key: &str, value: &str) -> (String, String) {
    (key.to_string(), value.to_string())
}

impl Case {
    fn new(name: &'static str) -> Case {
        Case {
            name,
            options: vec![opt("revision", "B3"), opt("host_mcu", "mcu")],
            fileoutput: true,
            pca_max: Some(4095.0),
            preregister_chip: false,
            no_pins_object: false,
            ops: Vec::new(),
        }
    }
    fn with(mut self, extra: &[(&str, &str)]) -> Case {
        for (k, v) in extra {
            self.options.push(opt(k, v));
        }
        self
    }
    fn without(mut self, key: &str) -> Case {
        self.options.retain(|(k, _)| k != key);
        self
    }
    fn ops(mut self, ops: Vec<Op>) -> Case {
        self.ops = ops;
        self
    }
    fn fileoutput(mut self, fileoutput: bool) -> Case {
        self.fileoutput = fileoutput;
        self
    }
    fn pca_max(mut self, pca_max: Option<f64>) -> Case {
        self.pca_max = pca_max;
        self
    }
}

fn pin(ptype: &'static str, desc: &'static str, h: usize) -> Op {
    Op::Pin { ptype, desc, h }
}

fn state_line(rep: &Replicape) -> String {
    let enabled: Vec<String> = rep
        .enabled_channels()
        .iter()
        .map(|(c, e)| format!("{c}={}", b2i(*e)))
        .collect();
    let dacs: Vec<String> =
        rep.stepper_dacs().iter().map(|(c, v)| format!("{c}={}", float_repr(*v))).collect();
    let pins: Vec<String> = rep
        .pin_map()
        .iter()
        .map(|(name, (class, channel))| format!("{name}:{class}:{channel}"))
        .collect();
    format!(
        "state:en={}:dac={}:dis={}:ena={}:pins={}",
        enabled.join(","),
        dacs.join(","),
        hex(&rep.sr_disabled()),
        hex(&rep.sr_enabled()),
        pins.join(",")
    )
}

enum Handle {
    Do(DigitalOutPin),
    Pwm(PwmPin),
}

/// Runs the async callbacks the port queued on the real reactor. A timer
/// queues one more async callback that ends the reactor: queued items are only
/// picked up on a pipe signal, and they run in registration order.
fn run_reactor(reactor: &Reactor) {
    let trigger = reactor.clone();
    reactor.register_timer(
        move |_eventtime| {
            let end = trigger.clone();
            trigger.register_async_callback(move |_| end.end(), 0.);
            NEVER
        },
        0.,
    );
    // Ends the run if the callbacks never arrive, instead of hanging the test.
    let watchdog = reactor.clone();
    let deadline = reactor.monotonic() + 30.;
    reactor.register_timer(
        move |_eventtime| {
            watchdog.end();
            NEVER
        },
        deadline,
    );
    reactor.run();
}

fn run_port(case: &Case) -> Vec<String> {
    LOG.lock().clear();
    let reactor = Reactor::new(false);
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(IndexMap::new()),
        reactor: reactor.clone(),
    });
    let printer_handle: Printer = printer.clone();
    let host = Arc::new(FakeMcu {
        printer: Arc::clone(&printer_handle),
        name: "mcu".to_string(),
        fileoutput: case.fileoutput,
        pca_max: case.pca_max,
        oid: Mutex::new(0),
        cq: Mutex::new(0),
        callbacks: Mutex::new(Vec::new()),
    });
    let host_handle: Mcu = host.clone();
    let ppins = Arc::new(PrinterPins::new());
    ppins.register_chip("mcu", Arc::new(McuChip(Arc::clone(&host_handle))) as Chip).unwrap();
    if !case.no_pins_object {
        printer.add("pins", ppins.clone());
    }
    printer.add("mcu", host.clone());
    if case.preregister_chip {
        ppins
            .register_chip("replicape", Arc::new(McuChip(Arc::clone(&host_handle))) as Chip)
            .unwrap();
    }
    let config: Config = Arc::new(FakeConfig {
        printer: Arc::clone(&printer_handle),
        section: "replicape".to_string(),
        options: case.options.clone(),
    });
    let rep = match load_config(&config) {
        Ok(Some(obj)) => {
            log_line("init:ok".to_string());
            downcast_arc::<Replicape>(&obj).expect("load_config returns the Replicape")
        }
        Ok(None) => panic!("load_config returned no object"),
        Err(e) => {
            log_line(format!("init:err:{}:{}", e.class_name(), e.message()));
            return LOG.lock().clone();
        }
    };

    let mut handles: IndexMap<usize, Handle> = IndexMap::new();
    for (i, op) in case.ops.iter().enumerate() {
        let kind = op.kind();
        let result: KResult<()> = match op {
            Op::Pin { ptype, desc, h } => match PinsApi::setup_pin(&*ppins, ptype, desc) {
                Ok(Pin::DigitalOut(d)) => {
                    log_line(format!("op{i}:pin:ok:Do"));
                    handles.insert(*h, Handle::Do(d));
                    Ok(())
                }
                Ok(Pin::Pwm(p)) => {
                    log_line(format!("op{i}:pin:ok:Pwm"));
                    handles.insert(*h, Handle::Pwm(p));
                    Ok(())
                }
                Ok(_) => panic!("replicape returned an unexpected pin kind"),
                Err(e) => Err(e),
            },
            Op::RunConfig => {
                let mut callbacks = std::mem::take(&mut *host.callbacks.lock());
                for (j, cb) in callbacks.iter_mut().enumerate() {
                    match cb() {
                        Ok(()) => log_line(format!("op{i}:cb{j}:ok")),
                        Err(e) => log_line(format!(
                            "op{i}:cb{j}:err:{}:{}",
                            e.class_name(),
                            e.message()
                        )),
                    }
                }
                *host.callbacks.lock() = callbacks;
                Ok(())
            }
            Op::SetPwm(h, t, v) => {
                match &handles[h] {
                    Handle::Pwm(p) => p.set_pwm(*t, *v),
                    Handle::Do(_) => panic!("set_pwm on a digital_out handle"),
                }
                Ok(())
            }
            Op::SetDigital(h, t, v) => {
                match &handles[h] {
                    Handle::Do(d) => d.set_digital(*t, *v),
                    Handle::Pwm(_) => panic!("set_digital on a pwm handle"),
                }
                Ok(())
            }
            Op::DoStart(h, a, b) => {
                match &handles[h] {
                    Handle::Do(d) => d.setup_start_value(*a, *b),
                    Handle::Pwm(_) => panic!("digital setup_start_value on a pwm handle"),
                }
                Ok(())
            }
            Op::PwmStart(h, a, b) => {
                match &handles[h] {
                    Handle::Pwm(p) => p.setup_start_value(*a, *b),
                    Handle::Do(_) => panic!("pwm setup_start_value on a digital_out handle"),
                }
                Ok(())
            }
            Op::MaxDuration(h, d) => {
                match &handles[h] {
                    Handle::Pwm(p) => p.setup_max_duration(*d),
                    Handle::Do(p) => p.setup_max_duration(*d),
                }
                Ok(())
            }
            Op::Cycle(h, ct, hw) => {
                match &handles[h] {
                    Handle::Pwm(p) => p.setup_cycle_time(*ct, Some(*hw)),
                    Handle::Do(_) => panic!("setup_cycle_time on a digital_out handle"),
                }
                Ok(())
            }
            Op::NextAligned(h, t) => {
                match &handles[h] {
                    Handle::Pwm(p) => log_line(format!(
                        "op{i}:next_aligned:{}",
                        float_repr(p.next_aligned_print_time(*t, None))
                    )),
                    Handle::Do(_) => panic!("next_aligned_print_time on a digital_out handle"),
                }
                Ok(())
            }
            Op::GetMcu(h) => {
                let mcu = match &handles[h] {
                    Handle::Pwm(p) => PwmPinApi::get_mcu(&**p),
                    Handle::Do(d) => DigitalOutPinApi::get_mcu(&**d),
                };
                log_line(format!("op{i}:get_mcu:{}", mcu.get_name()));
                Ok(())
            }
            Op::NoteStart(ch, a, b) => {
                rep.note_pwm_start_value(*ch, *a, *b);
                Ok(())
            }
            Op::NoteEnable(t, ch, e) => {
                rep.note_pwm_enable(*t, *ch, *e);
                Ok(())
            }
            Op::Reserve(pin, name) => ppins
                .get_pin_resolver("mcu")
                .and_then(|resolver| resolver.reserve_pin(pin, name)),
            Op::State => {
                log_line(state_line(&rep));
                Ok(())
            }
            Op::Asyncs => {
                assert_eq!(i, case.ops.len() - 1, "{}: Asyncs must be the last op", case.name);
                run_reactor(&reactor);
                log_line(state_line(&rep));
                Ok(())
            }
        };
        if let Err(e) = result {
            log_line(format!("op{i}:{kind}:err:{}:{}", e.class_name(), e.message()));
        }
    }
    LOG.lock().clone()
}

/// Python side: fakes plus the scenario driver, defined once per oracle.
const SETUP: &str = r#"
import configparser, logging
import configfile, pins, mcu
import extras.replicape as replicape

LOG = []
ASYNC = []

class LogHandler(logging.Handler):
    def emit(self, record):
        LOG.append('log:' + record.getMessage())

logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger().addHandler(LogHandler())

def norm(v):
    if isinstance(v, (list, tuple, bytes, bytearray)):
        return 'b:' + ''.join(['%02x' % (x,) for x in bytes(v)])
    return str(v)

class FakeCommand:
    def __init__(self, msgformat):
        self.msgformat = msgformat
    def send(self, data=(), minclock=0, reqclock=0):
        LOG.append('send:%s:%s:%d:%d' % (
            self.msgformat, ','.join([norm(d) for d in data]), minclock, reqclock))

class FakeDigitalOut:
    def __init__(self, host, pin):
        self.host = host
        self.pin = pin
    def get_mcu(self):
        return self.host
    def setup_max_duration(self, max_duration):
        LOG.append('do:%s:setup_max_duration:%s' % (self.pin, repr(max_duration)))
    def setup_start_value(self, start_value, shutdown_value):
        LOG.append('do:%s:setup_start_value:%d:%d' % (self.pin, start_value, shutdown_value))
    def set_digital(self, print_time, value):
        LOG.append('do:%s:set_digital:%s:%d' % (self.pin, repr(print_time), value))

class FakeMcuPwm:
    def __init__(self, host, pin):
        self.host = host
        self.pin = pin
    def get_mcu(self):
        return self.host
    def setup_max_duration(self, max_duration):
        LOG.append('pwm:%s:setup_max_duration:%s' % (self.pin, repr(max_duration)))
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        LOG.append('pwm:%s:setup_cycle_time:%s:%s' % (self.pin, repr(cycle_time), hardware_pwm))
    def setup_start_value(self, start_value, shutdown_value):
        LOG.append('pwm:%s:setup_start_value:%s:%s' % (
            self.pin, repr(start_value), repr(shutdown_value)))
    def set_pwm(self, print_time, value):
        LOG.append('pwm:%s:set_pwm:%s:%s' % (self.pin, repr(print_time), repr(value)))

class FakeReactor:
    def register_async_callback(self, callback, waketime=0.):
        ASYNC.append(callback)

_MISSING = object()

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=_MISSING):
        if name in self.objects:
            return self.objects[name]
        if default is not _MISSING:
            return default
        raise self.config_error("Unknown config object '%s'" % (name,))

class FakeMcu:
    def __init__(self, printer, name, fileoutput, pca_max):
        self.printer = printer
        self.name = name
        self.fileoutput = fileoutput
        self.pca_max = pca_max
        self.oid = 0
        self.cq = 0
        self.callbacks = []
    def get_name(self):
        return self.name
    def get_printer(self):
        return self.printer
    def is_fileoutput(self):
        return self.fileoutput
    def setup_pin(self, pin_type, pin_params):
        LOG.append('mcu:setup_pin:%s:%s:%d' % (pin_type, pin_params['pin'], pin_params['invert']))
        if pin_type == 'pwm':
            return FakeMcuPwm(self, pin_params['pin'])
        return FakeDigitalOut(self, pin_params['pin'])
    def create_oid(self):
        self.oid += 1
        return self.oid
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        LOG.append('mcu:add_config_cmd:%s:%d:%d' % (cmd, is_init, on_restart))
    def request_move_queue_slot(self):
        LOG.append('mcu:request_move_queue_slot')
    def seconds_to_clock(self, time):
        LOG.append('mcu:seconds_to_clock:%s' % (repr(time),))
        return int(time * 1000000.)
    def lookup_command(self, msgformat, cq=None):
        LOG.append('mcu:lookup_command:%s:%s' % (msgformat, cq))
        return FakeCommand(msgformat)
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        LOG.append('mcu:lookup_query_command:%s:%s:%s:%s:%s' % (
            msgformat, respformat, oid, cq, is_async))
        return object()
    def alloc_command_queue(self):
        self.cq += 1
        LOG.append('mcu:alloc_command_queue:%d' % (self.cq,))
        return self.cq
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
    def get_constant_float(self, name):
        LOG.append('mcu:get_constant_float:%s' % (name,))
        if name != 'PCA9685_MAX' or self.pca_max is None:
            raise mcu.error("Firmware constant '%s' not found" % (name,))
        return self.pca_max
    def print_time_to_clock(self, print_time):
        LOG.append('mcu:print_time_to_clock:%s' % (repr(print_time),))
        return int(print_time * 1000000.)

def state_line(rep):
    return 'state:en=%s:dac=%s:dis=%s:ena=%s:pins=%s' % (
        ','.join(['%d=%d' % (c, e) for c, e in rep.enabled_channels.items()]),
        ','.join(['%d=%s' % (c, repr(v)) for c, v in rep.stepper_dacs.items()]),
        ''.join(['%02x' % (x,) for x in rep.sr_disabled]),
        ''.join(['%02x' % (x,) for x in rep.sr_enabled]),
        ','.join(['%s:%s:%d' % (n, c.__name__, ch) for n, (c, ch) in rep.pins.items()]))

def drive(options, fileoutput, pca_max, preregister_chip, no_pins_object, ops):
    del LOG[:]
    del ASYNC[:]
    printer = FakePrinter()
    host = FakeMcu(printer, 'mcu', fileoutput, pca_max)
    ppins = pins.PrinterPins()
    ppins.register_chip('mcu', host)
    if not no_pins_object:
        printer.objects['pins'] = ppins
    printer.objects['mcu'] = host
    if preregister_chip:
        ppins.register_chip('replicape', host)
    fileconfig = configparser.RawConfigParser()
    fileconfig.add_section('replicape')
    for key, value in options:
        fileconfig.set('replicape', key, value)
    config = configfile.ConfigWrapper(printer, fileconfig, {}, 'replicape')
    try:
        rep = replicape.load_config(config)
    except Exception as e:
        LOG.append('init:err:%s:%s' % (type(e).__name__, e))
        return list(LOG)
    LOG.append('init:ok')
    handles = {}
    for i, op in enumerate(ops):
        kind = op[0]
        try:
            if kind == 'pin':
                obj = ppins.setup_pin(op[1], op[2])
                handles[op[3]] = obj
                variant = 'Pwm' if (isinstance(obj, replicape.servo_pwm) or op[1] == 'pwm') else 'Do'
                LOG.append('op%d:pin:ok:%s' % (i, variant))
            elif kind == 'run_config':
                for j, cb in enumerate(list(host.callbacks)):
                    try:
                        cb()
                        LOG.append('op%d:cb%d:ok' % (i, j))
                    except Exception as e:
                        LOG.append('op%d:cb%d:err:%s:%s' % (i, j, type(e).__name__, e))
            elif kind == 'set_pwm':
                handles[op[1]].set_pwm(op[2], op[3])
            elif kind == 'set_digital':
                handles[op[1]].set_digital(op[2], op[3])
            elif kind == 'do_start' or kind == 'pwm_start':
                handles[op[1]].setup_start_value(op[2], op[3])
            elif kind == 'max_duration':
                handles[op[1]].setup_max_duration(op[2])
            elif kind == 'cycle':
                handles[op[1]].setup_cycle_time(op[2], op[3])
            elif kind == 'next_aligned':
                LOG.append('op%d:next_aligned:%s' % (i, repr(handles[op[1]].next_aligned_print_time(op[2]))))
            elif kind == 'get_mcu':
                LOG.append('op%d:get_mcu:%s' % (i, handles[op[1]].get_mcu().get_name()))
            elif kind == 'note_start':
                rep.note_pwm_start_value(op[1], op[2], op[3])
            elif kind == 'note_enable':
                rep.note_pwm_enable(op[1], op[2], op[3])
            elif kind == 'reserve':
                ppins.get_pin_resolver('mcu').reserve_pin(op[1], op[2])
            elif kind == 'state':
                LOG.append(state_line(rep))
            elif kind == 'asyncs':
                assert i == len(ops) - 1
                while ASYNC:
                    ASYNC.pop(0)(0.)
                LOG.append(state_line(rep))
        except Exception as e:
            LOG.append('op%d:%s:err:%s:%s' % (i, kind, type(e).__name__, e))
    return list(LOG)
"#;

fn run_reference(py: &mut Oracle, case: &Case) -> Vec<String> {
    let options: Vec<String> =
        case.options.iter().map(|(k, v)| format!("('{k}', '{v}')")).collect();
    let ops: Vec<String> = case.ops.iter().map(Op::py).collect();
    let pca_max = case.pca_max.map_or("None".to_string(), pyf);
    let expr = format!(
        "drive([{}], {}, {}, {}, {}, [{}])",
        options.join(", "),
        pyb(case.fileoutput),
        pca_max,
        pyb(case.preregister_chip),
        pyb(case.no_pins_object),
        ops.join(", ")
    );
    match py.eval(&expr) {
        Ok(Value::List(lines)) => lines
            .into_iter()
            .map(|line| match line {
                Value::Str(s) => s,
                other => panic!("{}: unexpected log entry {other:?}", case.name),
            })
            .collect(),
        other => panic!("{}: oracle drive failed: {other:?}", case.name),
    }
}

// ---------------------------------------------------------------------------
// Scenario list.
// ---------------------------------------------------------------------------

fn stepper(name: char, mode: &str, current: &str, flags: &[(&str, &str)]) -> Vec<(String, String)> {
    let mut v = vec![
        opt(&format!("stepper_{name}_microstep_mode"), mode),
        opt(&format!("stepper_{name}_current"), current),
    ];
    for (k, value) in flags {
        v.push(opt(&format!("stepper_{name}_{k}"), value));
    }
    v
}

impl Case {
    fn steppers(mut self, steppers: Vec<Vec<(String, String)>>) -> Case {
        for s in steppers {
            self.options.extend(s);
        }
        self
    }
}

fn all_steppers_a(name: &'static str) -> Case {
    Case::new(name).steppers(vec![
        stepper('x', "1", "1.0", &[("chopper_off_time_high", "True")]),
        stepper('y', "2", "3.84", &[("chopper_hysteresis_high", "yes")]),
        stepper('z', "spread2", "0.5", &[("chopper_blank_time_high", "no")]),
        stepper(
            'e',
            "4",
            "2.25",
            &[
                ("chopper_off_time_high", "on"),
                ("chopper_hysteresis_high", "1"),
                ("chopper_blank_time_high", "false"),
            ],
        ),
        stepper('h', "16", "0.1", &[]),
    ])
}

fn all_steppers_b(name: &'static str) -> Case {
    Case::new(name).steppers(vec![
        stepper('x', "spread4", "1.75", &[("chopper_blank_time_high", "0")]),
        stepper('y', "spread16", "3.0", &[]),
        stepper('z', "stealth4", "0.25", &[("chopper_off_time_high", "TRUE")]),
        stepper('e', "stealth16", "1.2", &[("chopper_hysteresis_high", "Off")]),
        stepper('h', "disable", "9.9", &[]),
    ])
}

fn note_flow_ops(before_config: bool) -> Vec<Op> {
    let mut ops = Vec::new();
    if !before_config {
        ops.push(Op::RunConfig);
    }
    ops.extend([
        Op::NoteStart(5, 0.5, 0.),
        Op::NoteStart(11, 0., 1.),
        Op::NoteEnable(1.0, 11, true),
        Op::NoteEnable(1.01, 12, true),
        Op::NoteEnable(1.2, 11, false),
        Op::NoteEnable(1.3, 12, false),
        Op::NoteEnable(5.0, 5, true),
        Op::NoteEnable(5.0, 5, true),
        Op::NoteEnable(5.05, 5, false),
        Op::NoteEnable(9.0, 99, true),
        Op::NoteEnable(9.5, 99, false),
        Op::NoteEnable(10.0, 14, true),
        Op::NoteEnable(10.5, 14, false),
        Op::State,
    ]);
    ops
}

fn cases() -> Vec<Case> {
    let nan = f64::NAN;
    let mut cases = vec![
        Case::new("minimal").ops(vec![Op::State]),
        Case::new("enable_pin_custom").with(&[("enable_pin", "!gpio1_2")]).ops(vec![Op::State]),
        Case::new("enable_pin_unknown_chip")
            .with(&[("enable_pin", "nochip:gpio1_2")])
            .ops(vec![Op::State]),
        Case::new("enable_pin_invalid")
            .with(&[("enable_pin", "^gpio1_2")])
            .ops(vec![Op::State]),
        all_steppers_a("steppers_a").with(&[("standstill_power_down", "True")]).ops(vec![Op::State]),
        all_steppers_a("steppers_a_no_standstill").ops(vec![Op::State]),
        all_steppers_b("steppers_b").with(&[("standstill_power_down", "false")]).ops(vec![Op::State]),
        all_steppers_b("steppers_b_standstill")
            .with(&[("standstill_power_down", "1")])
            .ops(vec![Op::State]),
        Case::new("steppers_xy")
            .steppers(vec![
                stepper('x', "16", "1.0", &[]),
                stepper('y', "stealth16", "2.0", &[]),
            ])
            .ops(vec![Op::State]),
        Case::new("stepper_h_only")
            .steppers(vec![stepper('h', "spread4", "0.75", &[])])
            .with(&[("standstill_power_down", "yes")])
            .ops(vec![Op::State]),
        Case::new("stepper_e_only")
            .steppers(vec![stepper('e', "2", "1.5", &[("chopper_hysteresis_high", "yes")])])
            .ops(vec![Op::State]),
        Case::new("standstill_without_steppers")
            .with(&[("standstill_power_down", "True")])
            .ops(vec![Op::State]),
        Case::new("stepper_disabled_keeps_current_unread")
            .with(&[("stepper_x_microstep_mode", "disable"), ("stepper_x_current", "abc")])
            .ops(vec![Op::State]),
        Case::new("current_nan")
            .steppers(vec![stepper('x', "1", "nan", &[])])
            .ops(vec![Op::State]),
        // Config error paths.
        Case::new("missing_revision").without("revision"),
        Case::new("bad_revision").without("revision").with(&[("revision", "B4")]),
        Case::new("missing_host_mcu").without("host_mcu"),
        Case::new("unknown_host_mcu").without("host_mcu").with(&[("host_mcu", "nosuch")]),
        Case::new("bad_microstep").with(&[("stepper_x_microstep_mode", "bogus")]),
        Case::new("bad_microstep_after_ok_stepper")
            .steppers(vec![stepper('x', "1", "1.0", &[])])
            .with(&[("stepper_y_microstep_mode", "bogus")]),
        Case::new("bad_off_time_bool")
            .steppers(vec![stepper('x', "1", "1.0", &[("chopper_off_time_high", "maybe")])]),
        Case::new("bad_hysteresis_bool")
            .steppers(vec![stepper('x', "1", "1.0", &[("chopper_hysteresis_high", "2")])]),
        Case::new("bad_blank_time_bool")
            .steppers(vec![stepper('x', "1", "1.0", &[("chopper_blank_time_high", "")])]),
        Case::new("missing_current").with(&[("stepper_y_microstep_mode", "2")]),
        Case::new("current_zero").steppers(vec![stepper('x', "1", "0", &[])]),
        Case::new("current_negative").steppers(vec![stepper('x', "1", "-0.5", &[])]),
        Case::new("current_too_high").steppers(vec![stepper('z', "1", "3.85", &[])]),
        Case::new("current_inf").steppers(vec![stepper('z', "1", "inf", &[])]),
        Case::new("current_unparsable").steppers(vec![stepper('e', "1", "abc", &[])]),
        Case::new("standstill_bad_bool")
            .steppers(vec![stepper('x', "1", "1.0", &[])])
            .with(&[("standstill_power_down", "maybe")]),
        {
            let mut c = Case::new("no_pins_object");
            c.no_pins_object = true;
            c
        },
        {
            let mut c = Case::new("duplicate_chip");
            c.preregister_chip = true;
            c
        },
        Case::new("not_fileoutput_spi_bus").fileoutput(false).ops(vec![Op::State]),
    ];

    // Pin types and pin errors.
    cases.push(
        Case::new("pca_pin_types").ops(vec![
            pin("digital_out", "replicape:power_e", 0),
            pin("pwm", "replicape:power_h", 1),
            pin("digital_out", "!replicape:power_hotbed", 2),
            pin("endstop", "replicape:power_fan0", 3),
            pin("pwm", "replicape:power_fan0", 4),
            pin("pwm", "replicape:bogus", 5),
            pin("pwm", "replicape:power_fan1", 6),
            pin("digital_out", "replicape:power_fan2", 7),
            pin("pwm", "!replicape:power_fan3", 8),
            pin("pwm", "replicape: power_e", 9),
            pin("pwm", "replicape:pow er", 10),
            pin("adc", "replicape:power_fan2", 11),
            Op::GetMcu(0),
            Op::RunConfig,
            Op::State,
        ]),
    );
    cases.push(
        all_steppers_a("dac_pin_errors").ops(vec![
            pin("pwm", "replicape:stepper_x_enable", 0),
            pin("digital_out", "!replicape:stepper_y_enable", 1),
            pin("digital_out", "replicape:stepper_z_enable", 2),
            pin("digital_out", "replicape:stepper_e_enable", 3),
            Op::GetMcu(3),
            pin("endstop", "replicape:stepper_h_enable", 4),
            Op::State,
        ]),
    );
    cases.push(
        Case::new("dac_unconfigured_stepper").ops(vec![
            pin("digital_out", "replicape:stepper_x_enable", 0),
            Op::State,
        ]),
    );
    cases.push(
        Case::new("servo_pins").ops(vec![
            pin("pwm", "replicape:servo0", 0),
            pin("pwm", "!replicape:servo1", 1),
            Op::MaxDuration(0, 2.5),
            Op::Cycle(0, 0.02, false),
            Op::Cycle(1, 0.01, true),
            Op::PwmStart(0, 0.25, 0.5),
            Op::PwmStart(1, 1.0, 0.0),
            Op::GetMcu(0),
            Op::RunConfig,
            Op::SetPwm(0, 1.0, 0.75),
            Op::SetPwm(1, 2.0, 0.0),
            Op::State,
        ]),
    );
    cases.push(
        all_steppers_a("servo_with_steppers").ops(vec![
            pin("digital_out", "replicape:servo1", 0),
            Op::MaxDuration(0, 0.),
            Op::SetPwm(0, 1.5, 0.5),
            pin("pwm", "replicape:servo0", 1),
            Op::State,
        ]),
    );
    cases.push(
        Case::new("servo_reserved_conflict").ops(vec![
            Op::Reserve("gpio1_18", "other"),
            pin("pwm", "replicape:servo0", 0),
            pin("pwm", "replicape:servo1", 1),
            Op::Reserve("gpio3_17", "servo1"),
            Op::State,
        ]),
    );
    cases.push(
        Case::new("servo_without_pwmchip").fileoutput(false).ops(vec![
            pin("pwm", "replicape:servo1", 0),
            pin("pwm", "replicape:power_e", 1),
            Op::State,
        ]),
    );

    // PCA9685 command flow.
    cases.push(
        Case::new("pca_set_pwm").ops(vec![
            pin("pwm", "replicape:power_e", 0),
            pin("pwm", "!replicape:power_h", 1),
            pin("digital_out", "replicape:power_hotbed", 2),
            Op::MaxDuration(0, 1.5),
            Op::MaxDuration(2, 0.),
            Op::Cycle(0, REPLICAPE_PCA9685_CYCLE_TIME, false),
            Op::Cycle(0, 0.002, false),
            Op::Cycle(1, 0.0005, false),
            Op::Cycle(1, 0.001, false),
            Op::NextAligned(0, 3.25),
            Op::GetMcu(0),
            Op::GetMcu(2),
            Op::RunConfig,
            Op::SetPwm(0, 1.0, 0.0),
            Op::SetPwm(0, 1.5, 0.5),
            Op::SetPwm(0, 2.0, 1.0),
            Op::SetPwm(0, 2.5, -1.0),
            Op::SetPwm(0, 3.0, 2.0),
            Op::SetPwm(0, 3.5, 0.123456),
            Op::SetPwm(0, 4.0, 1.0 / 3.0),
            Op::SetPwm(0, 4.5, 0.5 / 4095.0),
            Op::SetPwm(0, 5.0, nan),
            Op::SetPwm(0, 5.5, f64::INFINITY),
            Op::SetPwm(0, 6.0, f64::NEG_INFINITY),
            Op::SetPwm(1, 1.0, 0.0),
            Op::SetPwm(1, 2.0, 1.0),
            Op::SetPwm(1, 3.0, 0.3),
            Op::SetPwm(1, 4.0, 1.5),
            Op::SetPwm(1, 5.0, -0.5),
            Op::SetPwm(1, 6.0, nan),
            Op::SetDigital(2, 7.0, true),
            Op::SetDigital(2, 7.5, false),
            Op::SetDigital(2, 8.0, true),
            Op::State,
            Op::Asyncs,
        ]),
    );
    cases.push(
        Case::new("pca_start_values").pca_max(Some(1000.5)).ops(vec![
            pin("pwm", "replicape:power_e", 0),
            pin("pwm", "!replicape:power_h", 1),
            pin("pwm", "replicape:power_hotbed", 2),
            pin("digital_out", "replicape:power_fan0", 3),
            pin("digital_out", "!replicape:power_fan1", 4),
            pin("digital_out", "!replicape:power_fan2", 5),
            pin("pwm", "replicape:power_fan3", 6),
            Op::PwmStart(0, 0.3, 0.7),
            Op::PwmStart(1, 0.3, 0.7),
            Op::PwmStart(2, -0.5, 1.5),
            Op::DoStart(3, true, false),
            Op::DoStart(4, true, true),
            Op::MaxDuration(5, 3.5),
            Op::MaxDuration(0, 0.),
            Op::PwmStart(6, nan, 0.0),
            Op::State,
            Op::RunConfig,
            Op::SetPwm(0, 1.0, 0.0),
            Op::SetPwm(0, 1.1, 0.5),
            Op::SetPwm(1, 1.2, 1.0),
            Op::SetPwm(2, 1.3, 0.0),
            Op::SetDigital(3, 1.4, true),
            Op::SetDigital(4, 1.5, false),
            Op::SetDigital(5, 1.6, false),
            Op::SetDigital(5, 1.7, true),
            Op::State,
            Op::Asyncs,
        ]),
    );
    cases.push(
        Case::new("pca_max_missing")
            .pca_max(None)
            .ops(vec![pin("pwm", "replicape:power_e", 0), Op::RunConfig, Op::State]),
    );
    cases.push(
        Case::new("pca_before_config_and_start_only").ops(vec![
            pin("pwm", "replicape:power_e", 0),
            Op::PwmStart(0, 1.0, 1.0),
            Op::MaxDuration(0, 7.25),
            Op::RunConfig,
            Op::SetPwm(0, 2.0, 1.0),
            Op::SetPwm(0, 2.1, 0.0),
            Op::Asyncs,
        ]),
    );

    // DAC enable pins.
    cases.push(
        all_steppers_a("dac_flow").with(&[("standstill_power_down", "True")]).ops(vec![
            pin("digital_out", "replicape:stepper_x_enable", 0),
            pin("digital_out", "replicape:stepper_y_enable", 1),
            pin("digital_out", "replicape:stepper_z_enable", 2),
            pin("digital_out", "replicape:stepper_e_enable", 3),
            pin("digital_out", "replicape:stepper_h_enable", 4),
            pin("pwm", "replicape:power_e", 5),
            Op::MaxDuration(0, 0.),
            Op::GetMcu(0),
            Op::RunConfig,
            Op::SetDigital(0, 1.0, true),
            Op::SetDigital(1, 1.02, true),
            Op::SetDigital(0, 1.5, false),
            Op::SetDigital(1, 1.55, false),
            Op::SetDigital(3, 2.0, true),
            Op::SetDigital(4, 2.01, true),
            Op::SetDigital(3, 2.2, false),
            Op::SetDigital(4, 2.3, false),
            Op::SetPwm(5, 3.0, 1.0),
            Op::SetPwm(5, 3.4, 0.0),
            Op::SetDigital(2, 4.0, true),
            Op::SetDigital(2, 4.05, false),
            Op::Asyncs,
        ]),
    );
    cases.push(
        Case::new("dac_tiny_current")
            .steppers(vec![stepper('x', "1", "0.0001", &[])])
            .ops(vec![
                pin("digital_out", "replicape:stepper_x_enable", 0),
                Op::RunConfig,
                Op::SetDigital(0, 1.0, true),
                Op::SetDigital(0, 2.0, false),
                Op::Asyncs,
            ]),
    );
    cases.push(
        Case::new("dac_tiny_pwm_max")
            .pca_max(Some(1.0))
            .steppers(vec![stepper('y', "2", "1.92", &[])])
            .ops(vec![
                pin("digital_out", "replicape:stepper_y_enable", 0),
                Op::RunConfig,
                Op::SetDigital(0, 1.0, true),
                Op::SetDigital(0, 2.0, false),
                Op::Asyncs,
            ]),
    );

    // note_pwm_start_value / note_pwm_enable flows.
    cases.push(
        Case::new("note_flows_before_config")
            .steppers(vec![
                stepper('x', "1", "1.0", &[]),
                stepper('y', "1", "1.0", &[]),
                stepper('e', "1", "1.0", &[]),
            ])
            .with(&[("standstill_power_down", "1")])
            .ops(note_flow_ops(true)),
    );
    cases.push(
        Case::new("note_flows_after_config")
            .steppers(vec![
                stepper('x', "1", "1.0", &[]),
                stepper('y', "1", "1.0", &[]),
                stepper('e', "1", "1.0", &[]),
            ])
            .ops(note_flow_ops(false)),
    );
    cases.push(
        Case::new("note_flows_no_steppers").ops(vec![
            Op::RunConfig,
            Op::NoteStart(5, 0.5, 0.5),
            Op::NoteEnable(1.0, 5, true),
            Op::NoteEnable(1.01, 3, true),
            Op::NoteEnable(1.02, 5, false),
            Op::NoteEnable(1.03, 3, false),
            Op::NoteEnable(1.04, 11, true),
            Op::State,
        ]),
    );
    cases
}

fn first_difference(expected: &[String], actual: &[String]) -> String {
    let n = expected.len().max(actual.len());
    for i in 0..n {
        let e = expected.get(i).map(String::as_str).unwrap_or("<missing>");
        let a = actual.get(i).map(String::as_str).unwrap_or("<missing>");
        if e != a {
            return format!("line {i}: reference `{e}` / port `{a}`");
        }
    }
    "no difference".to_string()
}

#[test]
fn replicape_parity() {
    let Some(mut py) = Oracle::start() else {
        eprintln!("skipping replicape_parity: KLIPPY_PYTHON is not set");
        return;
    };
    py.exec(SETUP).expect("oracle setup");
    install_log_sink();
    for case in cases() {
        let expected = run_reference(&mut py, &case);
        let actual = run_port(&case);
        assert_eq!(
            actual,
            expected,
            "{}: event log differs, first difference: {}\nreference:\n{}\nport:\n{}",
            case.name,
            first_difference(&expected, &actual),
            expected.join("\n"),
            actual.join("\n")
        );
    }
}

fn expect_float(py: &mut Oracle, expr: &str, want: f64) {
    match py.eval(expr) {
        Ok(Value::Float(f)) => assert_eq!(f.repr, float_repr(want), "{expr}"),
        other => panic!("{expr}: {other:?}"),
    }
}

fn expect_int(py: &mut Oracle, expr: &str, want: i64) {
    match py.eval(expr) {
        Ok(Value::Int(n)) => assert_eq!(n, want, "{expr}"),
        other => panic!("{expr}: {other:?}"),
    }
}

#[test]
fn replicape_constants_and_tables() {
    let Some(mut py) = Oracle::start() else {
        eprintln!("skipping replicape_constants_and_tables: KLIPPY_PYTHON is not set");
        return;
    };
    py.exec("import extras.replicape as replicape").expect("oracle import");
    expect_float(&mut py, "replicape.REPLICAPE_MAX_CURRENT", REPLICAPE_MAX_CURRENT);
    expect_int(&mut py, "replicape.REPLICAPE_PCA9685_BUS", REPLICAPE_PCA9685_BUS);
    expect_int(&mut py, "replicape.REPLICAPE_PCA9685_ADDRESS", REPLICAPE_PCA9685_ADDRESS);
    expect_float(&mut py, "replicape.REPLICAPE_PCA9685_CYCLE_TIME", REPLICAPE_PCA9685_CYCLE_TIME);
    expect_float(&mut py, "replicape.PIN_MIN_TIME", PIN_MIN_TIME);

    let servo: Vec<String> = SERVO_PINS
        .iter()
        .map(|(name, (a, b, c))| format!("('{name}', ('{a}', '{b}', '{c}'))"))
        .collect();
    let want = format!("[{}]", servo.join(", "));
    assert_eq!(
        py.eval("repr(sorted(replicape.SERVO_PINS.items()))"),
        Ok(Value::Str(want))
    );

    let steps: Vec<String> = step_config()
        .iter()
        .map(|(k, v)| format!("'{k}': {}", v.map_or("None".to_string(), |n| n.to_string())))
        .collect();
    let want = format!("{{{}}}", steps.join(", "));
    assert_eq!(py.eval("repr(replicape.ReplicapeStepConfig)"), Ok(Value::Str(want)));
}

#[test]
fn module_factory_entry_points() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}
