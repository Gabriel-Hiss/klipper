//! Parity tests for `extras::bltouch` (port of
//! `klippy/extras/bltouch.py`).
//!
//! Every expected value comes from the live `pyoracle` reference
//! (real `extras.bltouch` with fakes mirroring the Rust fakes below,
//! including fake `extras.probe` helpers) and
//! `recording::assert_equal` compares returned values, G-code
//! responses, PWM/endstop call logs, logs and raised errors. When
//! `Oracle::start()` returns `None` (outside the oracle image) the
//! test skips.
//!
//! Mocking note: the `probe` module helpers (`ProbeOffsetsHelper`,
//! `ProbeParameterHelper`, `DescendToEndstopHelper`,
//! `SampleAveragingHelper`, `ProbeCommandHelper`,
//! `HomingViaProbeHelper`) are faked on the Python side and are the
//! real `probe` port on the Rust side. `BLTouchProbe` timing/state
//! logic, G-code commands, output-mode commands and `PrinterBLTouch`
//! delegation are what is compared. The Python fakes mirror the
//! probe helpers' observable registrations (commands, event handlers)
//! so the comparison sees the same construction surface.

use extras::bltouch::{
    module_factory, BLTouchProbe, PrinterBLTouch, COMMANDS, ENDSTOP_REST_TIME,
    ENDSTOP_SAMPLE_COUNT, ENDSTOP_SAMPLE_TIME, MIN_CMD_TIME, RETRY_RESET_TIME, SIGNAL_PERIOD,
    TEST_TIME,
};
use extras::manual_probe::ProbeResult;
use extras::probe::{
    ProbeOffsetsHelper, ProbeOffsetsHelperApi, ProbeParameterHelper,
    ProbeParameterHelperApi, ProbeSessionApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts,
    GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GcmdFloatOpts, GetIntOpts as GcmdIntOpts,
    GetOpts as GcmdGetOpts, GetParser, GCodeCommandHandler, GCmd, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper, CommandWrapper,
    EndstopApi, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, PwmPinApi, PinsApi,
    TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{HomingTargetApi, Kinematics, ToolHeadApi};
use pycompat::text::{float_from_string, float_str, int_from_string, trunc_float_to_bigint};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, Value};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Record helpers
// ---------------------------------------------------------------------------

fn rec(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.bltouch.BLTouchProbe".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: recording::SideEffects::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    }
}

fn rec_printer(function: &str) -> Record {
    let mut r = rec(function);
    r.unit = "extras.bltouch.PrinterBLTouch".to_string();
    r
}

fn pyvalue_of(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.inner().to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::Float(recording::value::FloatVal::new(*f)),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_of).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_of).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_of(k), pyvalue_of(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("Host#{h}")),
    }
}

fn float_value(f: f64) -> Value {
    Value::Float(recording::value::FloatVal::new(f))
}

// Log capture (process-wide sink, mirrors other extras test harnesses).
static LOG_LINES: Mutex<Vec<String>> = Mutex::new(Vec::new());
static LOG_LOCK: Mutex<()> = Mutex::new(());
static LOG_INSTALLED: std::sync::OnceLock<()> = std::sync::OnceLock::new();

fn install_log_sink() {
    let _ = LOG_INSTALLED.set(());
    let _ = klippy_api::log::set_sink(Box::new(|rec| {
        LOG_LINES.lock().unwrap_or_else(|e| e.into_inner()).push(rec.message.to_string());
    }));
}

/// Serializes tests that capture logs through the process-wide sink.
fn with_log_capture<R>(f: impl FnOnce() -> R) -> (R, Vec<String>) {
    install_log_sink();
    let _guard = LOG_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    LOG_LINES.lock().unwrap_or_else(|e| e.into_inner()).clear();
    let result = f();
    let lines = LOG_LINES.lock().unwrap_or_else(|e| e.into_inner()).clone();
    (result, lines)
}

fn log_value(lines: &[String]) -> Value {
    Value::List(lines.iter().map(|s| Value::Str(s.clone())).collect())
}

// Serialize all tests: the reactor singleton plus the log sink are
// process-wide, and each Rust side is built while holding this lock.
fn test_lock() -> std::sync::MutexGuard<'static, ()> {
    static LOCK: std::sync::LazyLock<Mutex<()>> = std::sync::LazyLock::new(|| Mutex::new(()));
    LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

// ---------------------------------------------------------------------------
// Rust fakes
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
#[allow(dead_code)]
enum FakeOpt {
    Str(String),
    Bool(bool),
    Float(f64),
    Int(i64),
}

fn trunc_to_i64(f: f64) -> i64 {
    trunc_float_to_bigint(f)
        .inner()
        .to_string()
        .parse::<i64>()
        .unwrap_or(0)
}

fn required_err(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}
fn unable_err(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<IndexMap<String, Vec<EventCallback>>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(IndexMap::new()),
            reactor,
        })
    }
    fn handler_names(&self) -> Vec<String> {
        self.handlers
            .lock()
            .unwrap()
            .keys()
            .cloned()
            .collect()
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
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut g = self.objects.lock().unwrap();
        if g.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        g.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let g = self.objects.lock().unwrap();
        if let Some(o) = g.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .unwrap()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
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
        self.handlers
            .lock()
            .unwrap()
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let handlers = self.handlers.lock().unwrap().get(event).map(|v| v.len());
        let mut out = Vec::new();
        let mut g = self.handlers.lock().unwrap();
        if let Some(cbs) = g.get_mut(event) {
            for cb in cbs.iter_mut() {
                let r = cb(params)?;
                out.push(r);
            }
        }
        let _ = handlers;
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

klippy_api::printer_object!(FakePrinter);

struct FakeConfig {
    printer: Printer,
    name: String,
    options: IndexMap<String, FakeOpt>,
}

impl FakeConfig {
    fn lookup(&self, option: &str) -> Option<&FakeOpt> {
        self.options.get(option)
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
        match self.lookup(option) {
            Some(FakeOpt::Str(s)) => Ok(Some(s.clone())),
            Some(_) => Err(unable_err(option, &self.name)),
            None => match opts.default {
                None => Err(required_err(option, &self.name)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.lookup(option) {
            Some(FakeOpt::Int(v)) => {
                let v = *v;
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minv}",
                            self.name
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxv}",
                            self.name
                        )));
                    }
                }
                Ok(Some(v))
            }
            Some(_) => Err(unable_err(option, &self.name)),
            None => match opts.default {
                None => Err(required_err(option, &self.name)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.lookup(option) {
            Some(FakeOpt::Float(v)) => {
                let v = *v;
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.name,
                            float_str(minv)
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.name,
                            float_str(maxv)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.name,
                            float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.name,
                            float_str(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
            Some(_) => Err(unable_err(option, &self.name)),
            None => match opts.default {
                None => Err(required_err(option, &self.name)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.lookup(option) {
            Some(FakeOpt::Bool(v)) => Ok(Some(*v)),
            Some(_) => Err(unable_err(option, &self.name)),
            None => match opts.default {
                None => Err(required_err(option, &self.name)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw = match self.lookup(option) {
            Some(FakeOpt::Str(s)) => PyValue::Str(s.clone()),
            Some(_) => return Err(unable_err(option, &self.name)),
            None => match opts.default {
                None => return Err(required_err(option, &self.name)),
                Some(d) => d.unwrap_or(PyValue::None),
            },
        };
        let dict = match choices {
            PyValue::Dict(d) => d.clone(),
            PyValue::List(l) => l
                .iter()
                .map(|v| (v.clone(), v.clone()))
                .collect::<IndexMap<PyValue, PyValue>>(),
            _ => return Err(unable_err(option, &self.name)),
        };
        for (k, v) in &dict {
            if pycompat::value::py_equal(k, &raw) {
                return Ok(v.clone());
            }
        }
        return Err(KlippyError::config(format!(
            "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
            raw.str(),
            self.name
        )));
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        // `printer` (minimum_z_position lookup) and any other section:
        // return an empty-options view so defaults apply.
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            options: IndexMap::new(),
        })
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

#[derive(Default)]
struct SharedState {
    pwm: Vec<(f64, f64)>,
    pwm_setup: Vec<(String, f64, bool)>,
    home_starts: Vec<(f64, f64, i64, f64, bool)>,
    home_waits: Vec<f64>,
    home_wait_script: VecDeque<Result<f64, String>>,
    dwell: Vec<f64>,
    last_move_time: f64,
}

enum HomeWait {
    Ok(f64),
    Err(String),
}
impl HomeWait {
    fn into_result(self) -> Result<f64, String> {
        match self {
            HomeWait::Ok(v) => Ok(v),
            HomeWait::Err(s) => Err(s),
        }
    }
}

struct FakeMcu {
    printer: Printer,
    #[allow(dead_code)]
    shared: Arc<Mutex<SharedState>>,
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
        panic!("mcu setup_pin not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        trunc_to_i64(time * 1_000_000.0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.100
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("lookup_command not used")
    }
    fn lookup_query_command(&self, _m: &str, _r: &str, _o: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> {
        panic!("lookup_query_command not used")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(&self, _cb: klippy_api::mcu::SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> AsyncResponseWrapper {
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
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        trunc_to_i64(print_time * 1_000_000.0)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1_000_000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        10.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch not used")
    }
}

struct FakePwm {
    mcu: Mcu,
    shared: Arc<Mutex<SharedState>>,
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, max_duration: f64) {
        self.shared
            .lock()
            .unwrap()
            .pwm_setup
            .push(("max_duration".to_string(), max_duration, false));
    }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        self.shared
            .lock()
            .unwrap()
            .pwm_setup
            .push(("cycle_time".to_string(), cycle_time, hardware_pwm.unwrap_or(false)));
    }
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        self.shared.lock().unwrap().pwm.push((print_time, value));
    }
}

struct FakeEndstop {
    reactor: Reactor,
    mcu: Mcu,
    shared: Arc<Mutex<SharedState>>,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn add_stepper(&self, _stepper: Stepper) {}
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: Option<bool>,
    ) -> Completion<bool> {
        self.shared.lock().unwrap().home_starts.push((
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered.unwrap_or(true),
        ));
        let comp = self.reactor.completion();
        comp.complete(true);
        comp
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.shared.lock().unwrap().home_waits.push(home_end_time);
        let mut g = self.shared.lock().unwrap();
        match g.home_wait_script.pop_front() {
            Some(Ok(v)) => Ok(v),
            Some(Err(msg)) => Err(KlippyError::Command(msg)),
            None => Ok(0.0),
        }
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(0)
    }
}

struct FakePins {
    mcu: Mcu,
    shared: Arc<Mutex<SharedState>>,
    setups: Mutex<Vec<(String, String)>>,
    chips: Mutex<Vec<String>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used")
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        panic!("lookup_pin not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setups
            .lock()
            .unwrap()
            .push((pin_type.to_string(), pin_desc.to_string()));
        match pin_type {
            "pwm" => Ok(Pin::Pwm(Arc::new(FakePwm {
                mcu: self.mcu.clone(),
                shared: self.shared.clone(),
            }))),
            "endstop" => Ok(Pin::Endstop(Arc::new(FakeEndstop {
                reactor: self.reactor(),
                mcu: self.mcu.clone(),
                shared: self.shared.clone(),
            }))),
            _ => Err(KlippyError::config("unknown pin type")),
        }
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver not used")
    }
    fn register_chip(&self, chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        self.chips.lock().unwrap().push(chip_name.to_string());
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

impl FakePins {
    fn reactor(&self) -> Reactor {
        self.mcu.get_printer().get_reactor()
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeToolHead {
    shared: Arc<Mutex<SharedState>>,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0; 4]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics not used")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        self.shared.lock().unwrap().last_move_time
    }
    fn dwell(&self, delay: f64) {
        let mut g = self.shared.lock().unwrap();
        let d = if delay > 0.0 { delay } else { 0.0 };
        g.dwell.push(delay);
        g.last_move_time += d;
    }
    fn drip_move(&self, _newpos: Vec<f64>, _speed: f64, _drip_completion: &Completion<f64>) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq not used")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(&self, _a: Option<f64>, _b: Option<f64>, _c: Option<f64>, _d: Option<f64>) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _cb: klippy_api::toolhead::LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _t: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _e: klippy_api::toolhead::PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder not used")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _t: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory not used")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct GCodeState {
    commands: IndexMap<String, (Option<GCodeCommandHandler>, Option<String>)>,
    info: Vec<String>,
    raw: Vec<String>,
}

struct FakeGCode {
    state: Mutex<GCodeState>,
}

impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode {
            state: Mutex::new(GCodeState {
                commands: IndexMap::new(),
                info: Vec::new(),
                raw: Vec::new(),
            }),
        })
    }
    fn dispatch(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut g = self.state.lock().unwrap();
        let (handler, _) = g
            .commands
            .get_mut(cmd)
            .expect("command handler registered");
        let handler = handler.as_mut().expect("handler present");
        handler(gcmd)
    }
    fn info_list(&self) -> Vec<String> {
        self.state.lock().unwrap().info.clone()
    }
    fn command_keys(&self) -> Vec<String> {
        let mut k: Vec<String> = self.state.lock().unwrap().commands.keys().cloned().collect();
        k.sort();
        k
    }
    fn help_map(&self) -> Vec<(Value, Value)> {
        let g = self.state.lock().unwrap();
        g.commands
            .iter()
            .filter_map(|(k, (_, d))| d.clone().map(|v| (Value::Str(k.clone()), Value::Str(v))))
            .collect()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(&self, cmd: &str, func: Option<GCodeCommandHandler>, opts: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> {
        let mut g = self.state.lock().unwrap();
        if func.is_none() {
            return Ok(g.commands.shift_remove(cmd).and_then(|(h, _)| h));
        }
        if g.commands.contains_key(cmd) {
            return Err(KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        g.commands.insert(cmd.to_string(), (func, opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(&self, _cmd: &str, _key: &str, _value: Option<&str>, _func: GCodeCommandHandler, _desc: Option<&str>) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        let g = self.state.lock().unwrap();
        g.commands
            .iter()
            .filter_map(|(k, (_, d))| d.clone().map(|v| (k.clone(), v)))
            .collect()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
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
        panic!("get_mutex not used")
    }
    fn create_gcode_command(&self, command: &str, commandline: &str, params: &IndexMap<String, String>) -> GCmd {
        let fake = Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: Mutex::new(params.clone()),
            info: Mutex::new(Vec::new()),
        });
        let g: GCmd = fake.clone();
        g
    }
    fn respond_raw(&self, msg: &str) {
        self.state.lock().unwrap().raw.push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.state.lock().unwrap().info.push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: Mutex<IndexMap<String, String>>,
    info: Mutex<Vec<String>>,
}

impl FakeGcmd {
    fn new(command: &str, params: Vec<(String, String)>) -> Arc<Self> {
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: command.to_string(),
            params: Mutex::new(params.into_iter().collect()),
            info: Mutex::new(Vec::new()),
        })
    }
    fn missing(name: &str, commandline: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': missing {name}"))
    }
    fn unparsable(value: &str, commandline: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': unable to parse {value}"))
    }
    fn bound_str(v: &PyValue) -> String {
        match v {
            PyValue::Int(n) => n.inner().to_string(),
            PyValue::Float(f) => float_str(*f),
            other => other.str(),
        }
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GcmdGetOpts) -> KResult<PyValue> {
        let raw = match self.params.lock().unwrap().get(name) {
            None => match opts.default {
                None => return Err(Self::missing(name, &self.commandline)),
                Some(v) => return Ok(v),
            },
            Some(v) => v.clone(),
        };
        let parsed = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int => match int_from_string(&raw, 10) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(Self::unparsable(&raw, &self.commandline)),
            },
            GetParser::IntAutoBase => match int_from_string(&raw, 0) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(Self::unparsable(&raw, &self.commandline)),
            },
            GetParser::Float => match float_from_string(&raw) {
                Ok(f) => PyValue::Float(f),
                Err(_) => return Err(Self::unparsable(&raw, &self.commandline)),
            },
        };
        if let (Some(fv), Some(min)) = (parsed.to_f64(), opts.minval.as_ref().and_then(|v| v.to_f64())) {
            if fv < min {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    Self::bound_str(opts.minval.as_ref().unwrap())
                )));
            }
        }
        if let (Some(fv), Some(max)) = (parsed.to_f64(), opts.maxval.as_ref().and_then(|v| v.to_f64())) {
            if fv > max {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    Self::bound_str(opts.maxval.as_ref().unwrap())
                )));
            }
        }
        if let (Some(fv), Some(above)) = (parsed.to_f64(), opts.above.as_ref().and_then(|v| v.to_f64())) {
            if fv <= above {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    Self::bound_str(opts.above.as_ref().unwrap())
                )));
            }
        }
        if let (Some(fv), Some(below)) = (parsed.to_f64(), opts.below.as_ref().and_then(|v| v.to_f64())) {
            if fv >= below {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    Self::bound_str(opts.below.as_ref().unwrap())
                )));
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GcmdIntOpts) -> KResult<Option<i64>> {
        let v = self.get(
            name,
            GcmdGetOpts {
                default: opts
                    .default
                    .map(|o| o.map(|i| PyValue::Int(PyInt::from_i64(i))).unwrap_or(PyValue::None)),
                parser: GetParser::Int,
                minval: opts.minval.map(|i| PyValue::Int(PyInt::from_i64(i))),
                maxval: opts.maxval.map(|i| PyValue::Int(PyInt::from_i64(i))),
                above: None,
                below: None,
            },
        )?;
        match v {
            PyValue::Int(n) => n
                .inner()
                .to_string()
                .parse::<i64>()
                .map(Some)
                .map_err(|_| Self::unparsable("value out of i64 range", &self.commandline)),
            PyValue::None => Ok(None),
            _ => Ok(None),
        }
    }
    fn get_float(&self, name: &str, opts: GcmdFloatOpts) -> KResult<Option<f64>> {
        let v = self.get(
            name,
            GcmdGetOpts {
                default: opts.default.map(|o| o.map(PyValue::Float).unwrap_or(PyValue::None)),
                parser: GetParser::Float,
                minval: opts.minval.map(PyValue::Float),
                maxval: opts.maxval.map(PyValue::Float),
                above: opts.above.map(PyValue::Float),
                below: opts.below.map(PyValue::Float),
            },
        )?;
        match v {
            PyValue::Float(f) => Ok(Some(f)),
            PyValue::None => Ok(None),
            _ => Ok(None),
        }
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.lock().unwrap().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.info.lock().unwrap().push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.info.lock().unwrap().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

use pycompat::PyInt;

// ---------------------------------------------------------------------------
// Stack: the real `probe` helpers over the fakes above.
// ---------------------------------------------------------------------------

struct Stack {
    printer: Printer,
    printer_raw: Arc<FakePrinter>,
    config: Config,
    pins: Arc<FakePins>,
    gcode: Arc<FakeGCode>,
    shared: Arc<Mutex<SharedState>>,
}

fn make_stack(opts: Vec<(String, FakeOpt)>, home_wait: Vec<HomeWait>) -> Stack {
    let reactor = Reactor::new(false);
    let printer_raw = FakePrinter::new(reactor.clone());
    let printer: Printer = printer_raw.clone();
    let shared = Arc::new(Mutex::new(SharedState {
        home_wait_script: home_wait.into_iter().map(|h| h.into_result()).collect(),
        last_move_time: 0.0,
        ..Default::default()
    }));
    let mcu_raw = Arc::new(FakeMcu {
        printer: printer_raw.clone(),
        shared: Arc::clone(&shared),
    });
    let mcu: Mcu = mcu_raw.clone();
    let pins = Arc::new(FakePins {
        mcu: mcu.clone(),
        shared: Arc::clone(&shared),
        setups: Mutex::new(Vec::new()),
        chips: Mutex::new(Vec::new()),
    });
    let gcode = FakeGCode::new();
    let toolhead = Arc::new(FakeToolHead {
        shared: Arc::clone(&shared),
    });
    printer
        .add_object("pins", Arc::clone(&pins) as ObjectRef)
        .unwrap();
    printer
        .add_object("gcode", Arc::clone(&gcode) as ObjectRef)
        .unwrap();
    printer
        .add_object("toolhead", Arc::clone(&toolhead) as ObjectRef)
        .unwrap();
    let options: IndexMap<String, FakeOpt> = opts.into_iter().collect();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_raw.clone(),
        name: "bltouch".to_string(),
        options,
    });
    Stack {
        printer,
        printer_raw,
        config,
        pins,
        gcode,
        shared,
    }
}

fn make_probe(stack: &Stack) -> Arc<BLTouchProbe> {
    let offsets = ProbeOffsetsHelper::new(&stack.config).unwrap();
    let params = ProbeParameterHelper::new(&stack.config).unwrap();
    BLTouchProbe::new(
        &stack.config,
        offsets as Arc<dyn ProbeOffsetsHelperApi>,
        params as Arc<dyn ProbeParameterHelperApi>,
    )
    .unwrap()
}

// rust logs rendered as values
fn pwm_value(shared: &Arc<Mutex<SharedState>>) -> Value {
    Value::List(
        shared
            .lock()
            .unwrap()
            .pwm
            .iter()
            .map(|(t, v)| Value::Tuple(vec![float_value(*t), float_value(*v)]))
            .collect(),
    )
}
fn pwm_setup_value(shared: &Arc<Mutex<SharedState>>) -> Value {
    Value::List(
        shared
            .lock()
            .unwrap()
            .pwm_setup
            .iter()
            .map(|(name, v, hw)| {
                Value::Tuple(vec![
                    Value::Str(name.clone()),
                    float_value(*v),
                    Value::Bool(*hw),
                ])
            })
            .collect(),
    )
}
fn home_start_value(shared: &Arc<Mutex<SharedState>>) -> Value {
    Value::List(
        shared
            .lock()
            .unwrap()
            .home_starts
            .iter()
            .map(|(pt, st, sc, rt, trig)| {
                Value::Tuple(vec![
                    float_value(*pt),
                    float_value(*st),
                    Value::Int(*sc),
                    float_value(*rt),
                    Value::Bool(*trig),
                ])
            })
            .collect(),
    )
}
fn home_wait_value(shared: &Arc<Mutex<SharedState>>) -> Value {
    Value::List(
        shared
            .lock()
            .unwrap()
            .home_waits
            .iter()
            .map(|t| float_value(*t))
            .collect(),
    )
}
fn dwell_value(shared: &Arc<Mutex<SharedState>>) -> Value {
    Value::List(
        shared
            .lock()
            .unwrap()
            .dwell
            .iter()
            .map(|d| float_value(*d))
            .collect(),
    )
}
fn pins_setup_value(pins: &Arc<FakePins>) -> Value {
    Value::List(
        pins
            .setups
            .lock()
            .unwrap()
            .iter()
            .map(|(t, p)| Value::Tuple(vec![Value::Str(t.clone()), Value::Str(p.clone())]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Python oracle harness
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import logging
log_lines = []
class _Capture(logging.Handler):
    def emit(self, record):
        log_lines.append(record.getMessage())
_root = logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

import sys, types, collections
class Error(Exception):
    pass
ConfigError = Error
class CommandError(Exception):
    pass
class _Sentinel:
    pass
sentinel = _Sentinel()

class FakeReactor:
    def __init__(self):
        self.t = 100.0
    def monotonic(self):
        return self.t
    def register_callback(self, cb):
        return _Completion(None)
reactor = FakeReactor()

class PrinterFake:
    def __init__(self):
        self.objects = {}
        self.command_error = CommandError
        self.config_error = ConfigError
        self.handlers = {}
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is not sentinel:
            return default
        raise ConfigError("Unknown config object '%s'" % (name,))
    def add_object(self, name, obj):
        if name in self.objects:
            raise ConfigError("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def get_reactor(self):
        return reactor
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        for cb in list(self.handlers.get(event, [])):
            cb(*params)
        return []

class ConfigFake:
    def __init__(self, printer, options, name='bltouch'):
        self._printer = printer
        self.options = dict(options)
        self.name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.name
    def get(self, option, default=sentinel):
        if option in self.options:
            v = self.options[option]
            if not isinstance(v, str):
                raise ConfigError("Unable to parse option '%s' in section '%s'" % (option, self.name))
            return v
        if default is not sentinel:
            return default
        raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.name))
    def getboolean(self, option, default=sentinel):
        if option in self.options:
            return bool(self.options[option])
        if default is not sentinel:
            return default
        raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.name))
    def getfloat(self, option, default=sentinel, above=None, minval=None):
        if option in self.options:
            v = self.options[option]
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise ConfigError("Unable to parse option '%s' in section '%s'" % (option, self.name))
            v = float(v)
        else:
            if default is not sentinel:
                v = float(default)
            else:
                raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.name))
        if above is not None and v <= above:
            raise ConfigError("Option '%s' in section '%s' must be above %s" % (option, self.name, above))
        if minval is not None and v < minval:
            raise ConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self.name, minval))
        return v
    def getint(self, option, default=sentinel, minval=None):
        if option in self.options:
            v = self.options[option]
            if isinstance(v, bool) or not isinstance(v, int):
                raise ConfigError("Unable to parse option '%s' in section '%s'" % (option, self.name))
        else:
            if default is not sentinel:
                v = default
            else:
                raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.name))
        if minval is not None and v < minval:
            raise ConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self.name, minval))
        return v
    def getchoice(self, option, choices, default=sentinel):
        if type(choices) == list:
            choices = {i: i for i in choices}
        if option in self.options:
            c = self.options[option]
        elif default is not sentinel:
            c = default
        else:
            raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self.name))
        if c not in choices:
            raise ConfigError("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.name))
        return choices[c]

pwm_calls = []
pwm_setup_calls = []
endstop_home_calls = []
endstop_home_wait_calls = []
home_wait_script = []
toolhead_state = {'last': 0.0, 'dwells': []}
descend_clears = []
probe_calls = []

class FakeMcu:
    def estimated_print_time(self, curtime):
        return 10.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000000)
    def seconds_to_clock(self, seconds):
        return int(seconds * 1000000)
    def clock_to_print_time(self, clock):
        return clock / 1000000.0

class FakePwm:
    def __init__(self, mcu):
        self._mcu = mcu
    def setup_max_duration(self, max_duration):
        pwm_setup_calls.append(('max_duration', float(max_duration), False))
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        pwm_setup_calls.append(('cycle_time', float(cycle_time), bool(hardware_pwm)))
    def set_pwm(self, print_time, value):
        pwm_calls.append((float(print_time), float(value)))
    def get_mcu(self):
        return self._mcu

class _Completion:
    def __init__(self, result):
        self.result = result
    def wait(self, waketime=None):
        return self.result

class FakeEndstop:
    def __init__(self, mcu):
        self._mcu = mcu
    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        endstop_home_calls.append((float(print_time), float(sample_time), int(sample_count), float(rest_time), bool(triggered)))
        return _Completion(True)
    def home_wait(self, home_end_time):
        endstop_home_wait_calls.append(float(home_end_time))
        if not home_wait_script:
            return 0.0
        v = home_wait_script.pop(0)
        if isinstance(v, Exception):
            raise v
        return v
    def query_endstop(self, print_time):
        return 0
    def get_mcu(self):
        return self._mcu
    def add_stepper(self, stepper):
        pass
    def get_steppers(self):
        return []

class FakePins:
    def __init__(self, mcu):
        self._mcu = mcu
        self.setup_calls = []
        self.registered_chips = {}
    def setup_pin(self, pin_type, pin_desc):
        self.setup_calls.append((pin_type, pin_desc))
        if pin_type == 'pwm':
            return FakePwm(self._mcu)
        if pin_type == 'endstop':
            return FakeEndstop(self._mcu)
        raise ConfigError("unknown pin type")
    def register_chip(self, chip_name, chip):
        self.registered_chips[chip_name] = chip

class FakeToolHead:
    def get_last_move_time(self):
        return toolhead_state['last']
    def dwell(self, delay):
        d = delay if delay > 0.0 else 0.0
        toolhead_state['dwells'].append(float(delay))
        toolhead_state['last'] = toolhead_state['last'] + d
    def get_position(self):
        return [0.0, 0.0, 0.0, 0.0]

class FakeGCode:
    def __init__(self):
        self.commands = {}
        self.help = {}
        self.info = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            return self.commands.pop(cmd, None)
        if cmd in self.commands:
            raise ConfigError("gcode command %s already registered" % (cmd,))
        self.commands[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
    def respond_info(self, msg, log=True):
        self.info.append(msg)
    def respond_raw(self, msg):
        self.info.append(msg)
    def create_gcode_command(self, command, commandline, params):
        return GcmdFake(command, commandline, params)

class GcmdFake:
    def __init__(self, command, commandline, params=None):
        self.command = command
        self.commandline = commandline
        self.params = dict(params) if params else {}
        self.info = []
    def get(self, name, default=sentinel):
        if name in self.params:
            return self.params[name]
        if default is not sentinel:
            return default
        raise CommandError("gcmd.get('%s') requires a default" % (name,))
    def get_command(self):
        return self.command
    def get_commandline(self):
        return self.commandline
    def get_command_parameters(self):
        return dict(self.params)
    def respond_info(self, msg, log=True):
        self.info.append(msg)
    def respond_raw(self, msg):
        self.info.append(msg)

class FakeOffsets:
    def __init__(self, config):
        probe_calls.append('offsets')
        self.x = config.getfloat('x_offset', 0.)
        self.y = config.getfloat('y_offset', 0.)
        self.z = config.getfloat('z_offset')
    def get_offsets(self, gcmd=None):
        return (self.x, self.y, self.z)

class FakeParams:
    def __init__(self, config):
        probe_calls.append('params')
        self.speed = config.getfloat('speed', 5.0, above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.sample_count = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat('sample_retract_dist', 2., above=0.)
        self.samples_result = config.getchoice('samples_result', ['median', 'average'], 'average')
        self.samples_tolerance = config.getfloat('samples_tolerance', 0.100, minval=0.)
        self.samples_retries = config.getint('samples_tolerance_retries', 0, minval=0)
    def get_probe_params(self, gcmd=None):
        return {'probe_speed': self.speed, 'lift_speed': self.lift_speed, 'samples': self.sample_count, 'sample_retract_dist': self.sample_retract_dist, 'samples_tolerance': self.samples_tolerance, 'samples_tolerance_retries': self.samples_retries, 'samples_result': self.samples_result}

class FakeDescend:
    def __init__(self, config, mcu_probe, probe_offsets, param_helper, always_check_movement=False):
        probe_calls.append('descend(%r)' % (always_check_movement,))
        self.calls = 0
        self.pulled = []
        # Mirror `LookupZSteppers.__init__` (probe.py:199): the real helper
        # registers a `klippy:mcu_identify` handler.
        config.get_printer().register_event_handler('klippy:mcu_identify', lambda *a: None)
    def descend_until_trigger(self, gcmd):
        self.calls += 1
    def pull_trigger_positions(self):
        r = self.pulled[:]
        self.pulled = []
        return r
    def clear_trigger_positions(self):
        descend_clears.append(1)

class FakeSampling:
    def __init__(self, config, param_helper, start_session_cb):
        probe_calls.append('sampling')
        self.start_cb = start_session_cb
        self.hw = None
        self.starts = 0
        self.runs = 0
        # Mirror `SampleAveragingHelper.__init__` (probe.py:322): the real
        # helper registers a `gcode:command_error` handler.
        config.get_printer().register_event_handler('gcode:command_error', lambda *a: None)
    def start_probe_session(self, gcmd):
        if self.hw is not None:
            raise CommandError("Internal probe error - start/end probe session mismatch")
        self.hw = self.start_cb(gcmd)
        self.starts += 1
        return self
    def run_probe(self, gcmd):
        self.runs += 1
        return self.hw.run_probe(gcmd)
    def pull_probed_results(self):
        return self.hw.pull_probed_results()
    def end_probe_session(self):
        hw = self.hw
        self.hw = None
        return hw.end_probe_session()

class FakeCommandHelper:
    def __init__(self, config, probe, query_endstop=None, can_set_z_offset=True):
        probe_calls.append('command')
        self.name = config.get_name()
        self.probe = probe
        self.query_endstop = query_endstop
        # Mirror `ProbeCommandHelper.__init__` (probe.py:49-67): command
        # registrations and help strings, so command surface comparisons
        # line up with the real Rust probe port.
        g = config.get_printer().lookup_object('gcode')
        def _reg(cmd, desc):
            g.register_command(cmd, lambda gcmd: None, desc=desc)
        _reg('QUERY_PROBE', "Return the status of the z-probe")
        _reg('PROBE', "Probe Z-height at current XY position")
        if can_set_z_offset:
            _reg('PROBE_CALIBRATE', "Calibrate the probe's z_offset")
        _reg('PROBE_ACCURACY', "Probe Z-height accuracy at current XY position")
        if can_set_z_offset:
            _reg('Z_OFFSET_APPLY_PROBE', "Adjust the probe's z_offset")
    def get_status(self, eventtime):
        import gcode as gmod
        return {'name': self.name, 'last_query': False, 'last_probe_position': gmod.Coord((0., 0., 0.)), 'last_z_result': 0.}

class FakeHomingVia:
    def __init__(self, config, position_endstop, query_endstop_cb=None):
        probe_calls.append('homing(%r)' % (position_endstop,))
        self.position_endstop = position_endstop
        self.query_endstop_cb = query_endstop_cb
        config.get_printer().lookup_object('pins').register_chip('probe', self)
    def get_mcu(self):
        raise CommandError("Internal error. Can't home via HomingViaProbeHelper.")
    def add_stepper(self, stepper):
        pass
    def get_steppers(self):
        return []
    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        raise CommandError("Internal error. Can't home via HomingViaProbeHelper.")
    def home_wait(self, home_end_time):
        raise CommandError("Internal error. Can't home via HomingViaProbeHelper.")
    def query_endstop(self, print_time):
        if self.query_endstop_cb is None:
            return False
        return self.query_endstop_cb(print_time)
    def get_position_endstop(self):
        return self.position_endstop
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'z_virtual_endstop':
            raise ConfigError("Probe virtual endstop only useful as endstop pin")
        return self

_probe_fake = types.ModuleType('extras.probe')
_probe_fake.ProbeOffsetsHelper = FakeOffsets
_probe_fake.ProbeParameterHelper = FakeParams
_probe_fake.DescendToEndstopHelper = FakeDescend
_probe_fake.SampleAveragingHelper = FakeSampling
_probe_fake.ProbeCommandHelper = FakeCommandHelper
_probe_fake.HomingViaProbeHelper = FakeHomingVia
sys.modules['extras.probe'] = _probe_fake

import extras.bltouch as bt

def make_world(opts):
    printer = PrinterFake()
    mcu = FakeMcu()
    pins = FakePins(mcu)
    gcode = FakeGCode()
    toolhead = FakeToolHead()
    printer.objects['pins'] = pins
    printer.objects['gcode'] = gcode
    printer.objects['toolhead'] = toolhead
    config = ConfigFake(printer, opts)
    return printer, pins, gcode, toolhead, config
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(PY_SETUP).expect("bltouch fakes");
    Some(py)
}


// Convenience config options for BLTouchProbe-only tests (probe helpers
// are the real probe port on the Rust side, so the same values are
// read here as in the Python fakes).
fn base_opts(extra: Vec<(&str, FakeOpt)>) -> Vec<(String, FakeOpt)> {
    let mut v = vec![
        ("control_pin".to_string(), FakeOpt::Str("PA1".to_string())),
        ("sensor_pin".to_string(), FakeOpt::Str("PA2".to_string())),
        ("z_offset".to_string(), FakeOpt::Float(0.0)),
    ];
    v.extend(extra.into_iter().map(|(k, o)| (k.to_string(), o)));
    v
}

fn py_make_world(opts: Vec<(&str, &str)>) -> String {
    // Builds a Python options dict literal from (key, literal) pairs
    // where each value is already a Python literal expression.
    let inner = opts
        .iter()
        .map(|(k, lit)| format!("'{k}': {lit}"))
        .collect::<Vec<_>>()
        .join(", ");
    format!("printer, pins, gcode, toolhead, config = make_world({{{inner}}})")
}

// ---------------------------------------------------------------------------
// Module header / factory
// ---------------------------------------------------------------------------

#[test]
fn constants_match_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    let expected = py
        .eval("[bt.SIGNAL_PERIOD, bt.MIN_CMD_TIME, bt.TEST_TIME, bt.RETRY_RESET_TIME, bt.ENDSTOP_REST_TIME, bt.ENDSTOP_SAMPLE_TIME, bt.ENDSTOP_SAMPLE_COUNT]")
        .expect("oracle constants");
    let rust: Vec<Value> = vec![
        float_value(SIGNAL_PERIOD),
        float_value(MIN_CMD_TIME),
        float_value(TEST_TIME),
        float_value(RETRY_RESET_TIME),
        float_value(ENDSTOP_REST_TIME),
        float_value(ENDSTOP_SAMPLE_TIME),
        Value::Int(ENDSTOP_SAMPLE_COUNT),
    ];
    assert_equal(&rec("constants"), "path", &expected, &Value::List(rust)).unwrap();

    let expected_cmds = py.eval("sorted(bt.Commands.items())").expect("oracle commands");
    let mut cmds: Vec<(&str, f64)> = COMMANDS.to_vec();
    cmds.sort_by(|a, b| a.0.cmp(b.0));
    let rust_cmds: Vec<Value> = cmds
        .iter()
        .map(|(name, dur)| Value::Tuple(vec![Value::Str(name.to_string()), float_value(*dur)]))
        .collect();
    assert_equal(&rec("commands"), "path", &expected_cmds, &Value::List(rust_cmds)).unwrap();
}

#[test]
fn module_factory_slots_and_load_config() {
    let _guard = test_lock();
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    // `load_config` builds the real object and registers it as `probe`.
    let stack = make_stack(base_opts(vec![]), vec![]);
    let obj = (factory.load_config.unwrap())(&stack.config)
        .expect("load_config succeeds")
        .expect("load_config returns object");
    assert!(stack
        .printer
        .lookup_object("probe", LookupDefault::ReturnNone)
        .unwrap()
        .is_some());
    let _ = obj;
}

// ---------------------------------------------------------------------------
// Init group
// ---------------------------------------------------------------------------

#[test]
fn init_minimal_registrations() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))")
        .expect("py construct");
    let py_pins = py.eval("list(pins.setup_calls)").unwrap();
    let py_pwm_setup = py.eval("list(pwm_setup_calls)").unwrap();
    let py_cmds = py.eval("sorted(gcode.commands.keys())").unwrap();
    let py_help = py.eval("dict(gcode.help)").unwrap();
    let py_handlers = py.eval("sorted(printer.handlers.keys())").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let rust_pins = pins_setup_value(&stack.pins);
    let rust_pwm = pwm_setup_value(&stack.shared);
    let rust_cmds: Vec<Value> = stack
        .gcode
        .command_keys()
        .iter()
        .map(|s| Value::Str(s.clone()))
        .collect();
    assert_equal(&rec("init_pins"), "pins", &py_pins, &rust_pins).unwrap();
    assert_equal(&rec("init_pwm_setup"), "pwm_setup", &py_pwm_setup, &rust_pwm).unwrap();
    assert_equal(&rec("init_cmds"), "cmds", &py_cmds, &Value::List(rust_cmds)).unwrap();
    // Help dict compared separately (dict order-insensitive via Value::Dict).
    let rust_help_d: Vec<(Value, Value)> = stack.gcode.help_map().into_iter().collect();
    assert_equal(&rec("init_help"), "help", &py_help, &Value::Dict(rust_help_d)).unwrap();
    let rust_handlers: Vec<Value> = {
        let mut names = stack.printer_raw.handler_names();
        names.sort();
        names.iter().map(|s| Value::Str(s.clone())).collect()
    };
    assert_equal(&rec("init_handlers"), "handlers", &py_handlers, &Value::List(rust_handlers)).unwrap();
}

#[test]
fn init_pin_move_time_validation() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("pin_move_time", "0.0"),
    ]))
    .unwrap();
    let py_exc = match py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))") {
        Err(e) => e,
        Ok(_) => panic!("expected ConfigError"),
    };
    let stack = make_stack(
        base_opts(vec![("pin_move_time", FakeOpt::Float(0.0))]),
        vec![],
    );
    let err = match make_probe_expect_err(&stack) {
        Ok(_) => panic!("expect ConfigError"),
        Err(e) => e,
    };
    assert_equal(&rec("init_param"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(err.class_name().to_string())).unwrap();
    assert_equal(&rec("init_param"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(err.message().to_string())).unwrap();
}

fn make_probe_expect_err(stack: &Stack) -> KResult<Arc<BLTouchProbe>> {
    let offsets = ProbeOffsetsHelper::new(&stack.config)?;
    let params = ProbeParameterHelper::new(&stack.config)?;
    BLTouchProbe::new(
        &stack.config,
        offsets as Arc<dyn ProbeOffsetsHelperApi>,
        params as Arc<dyn ProbeParameterHelperApi>,
    )
}

#[test]
fn init_bad_output_mode() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("set_output_mode", "'6V'"),
    ]))
    .unwrap();
    let py_exc = match py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))") {
        Err(e) => e,
        Ok(_) => panic!("expected ConfigError"),
    };
    let stack = make_stack(
        base_opts(vec![("set_output_mode", FakeOpt::Str("6V".to_string()))]),
        vec![],
    );
    let err = match make_probe_expect_err(&stack) {
        Ok(_) => panic!("expect ConfigError"),
        Err(e) => e,
    };
    assert_equal(&rec("init_mode"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(err.class_name().to_string())).unwrap();
    assert_equal(&rec("init_mode"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(err.message().to_string())).unwrap();
}

#[test]
fn init_missing_control_pin() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    let py_exc = match py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))") {
        Err(e) => e,
        Ok(_) => panic!("expected ConfigError"),
    };
    let mut opts = base_opts(vec![]);
    opts.retain(|(k, _)| k != "control_pin");
    let stack = make_stack(opts, vec![]);
    let err = match make_probe_expect_err(&stack) {
        Ok(_) => panic!("expect ConfigError"),
        Err(e) => e,
    };
    assert_equal(&rec("init_missing"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(err.class_name().to_string())).unwrap();
    assert_equal(&rec("init_missing"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(err.message().to_string())).unwrap();
}

// ---------------------------------------------------------------------------
// Handle-connect group
// ---------------------------------------------------------------------------

#[test]
fn handle_connect_success_default() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.append(1.0); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_home = py.eval("list(endstop_home_calls)").unwrap();
    let py_waits = py.eval("list(endstop_home_wait_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![HomeWait::Ok(1.0)]);
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    assert_equal(&rec("connect_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("connect_home"), "home", &py_home, &home_start_value(&stack.shared)).unwrap();
    assert_equal(&rec("connect_waits"), "waits", &py_waits, &home_wait_value(&stack.shared)).unwrap();
    assert_equal(&rec("connect_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn handle_connect_output_mode_5v() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("set_output_mode", "'5V'"),
    ]))
    .unwrap();
    py.exec("home_wait_script.append(1.0); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(
        base_opts(vec![("set_output_mode", FakeOpt::Str("5V".to_string()))]),
        vec![HomeWait::Ok(1.0)],
    );
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    assert_equal(&rec("connect5v_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("connect5v_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn handle_connect_output_mode_od() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("set_output_mode", "'OD'"),
    ]))
    .unwrap();
    py.exec("home_wait_script.append(1.0); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(
        base_opts(vec![("set_output_mode", FakeOpt::Str("OD".to_string()))]),
        vec![HomeWait::Ok(1.0)],
    );
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    assert_equal(&rec("connectod_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("connectod_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn handle_connect_verify_retry_then_success() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.extend([0.0, 0.0, 1.0]); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_info = py.eval("list(gcode.info)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(
        base_opts(vec![]),
        vec![HomeWait::Ok(0.0), HomeWait::Ok(0.0), HomeWait::Ok(1.0)],
    );
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    let rust_info: Vec<String> = stack.gcode.info_list();
    assert_equal(&rec("retry_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("retry_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("retry_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn handle_connect_verify_fails_warns() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.extend([0.0, 0.0, 0.0]); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("res = printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_info = py.eval("list(gcode.info)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(
        base_opts(vec![]),
        vec![HomeWait::Ok(0.0), HomeWait::Ok(0.0), HomeWait::Ok(0.0)],
    );
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    let rust_info: Vec<String> = stack.gcode.info_list();
    assert_equal(&rec("fail_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("fail_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("fail_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn handle_connect_pin_up_not_triggered_false() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("pin_up_reports_not_triggered", "False"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); log_lines.clear()")
        .unwrap();
    py.exec("printer.send_event('klippy:connect')").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_home = py.eval("list(endstop_home_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(
        base_opts(vec![("pin_up_reports_not_triggered", FakeOpt::Bool(false))]),
        vec![],
    );
    let _blt = make_probe(&stack);
    let (rust_result, rust_logs) = with_log_capture(|| {
        stack.printer.send_event("klippy:connect", &[])
    });
    rust_result.unwrap();
    assert_equal(&rec("notrig_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("notrig_home"), "home", &py_home, &home_start_value(&stack.shared)).unwrap();
    assert_equal(&rec("notrig_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

// ---------------------------------------------------------------------------
// Verify-raise group (direct)
// ---------------------------------------------------------------------------

#[test]
fn verify_raise_probe_error_after_retries() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.extend([0.0, 0.0, 0.0]); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))")
        .unwrap();
    let py_exc = match py.exec("blt._verify_raise_probe()") {
        Err(e) => e,
        Ok(_) => panic!("expected CommandError"),
    };
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_info = py.eval("list(gcode.info)").unwrap();

    let stack = make_stack(
        base_opts(vec![]),
        vec![HomeWait::Ok(0.0), HomeWait::Ok(0.0), HomeWait::Ok(0.0)],
    );
    let blt = make_probe(&stack);
    let rust_err = blt.verify_raise_probe().expect_err("expect CommandError");
    assert_equal(&rec("verify_err"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(rust_err.class_name().to_string())).unwrap();
    assert_equal(&rec("verify_err"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(rust_err.message().to_string())).unwrap();
    let rust_info: Vec<String> = stack.gcode.info_list();
    assert_equal(&rec("verify_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("verify_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn verify_state_command_error_is_false() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.append(CommandError('mcu link lost')); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))")
        .unwrap();
    let py_ret = py.eval("blt._verify_state(True)").unwrap();

    let stack = make_stack(
        base_opts(vec![]),
        vec![HomeWait::Err("mcu link lost".to_string())],
    );
    let blt = make_probe(&stack);
    let rust_ret = blt.verify_state(true).unwrap();
    assert_equal(
        &rec("verify_state"),
        "ret",
        &py_ret,
        &Value::Bool(rust_ret),
    )
    .unwrap();
}

// ---------------------------------------------------------------------------
// BLTOUCH_DEBUG / BLTOUCH_STORE groups
// ---------------------------------------------------------------------------

#[test]
fn debug_command_no_arg_lists_commands() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_DEBUG', 'BLTOUCH_DEBUG', {}); gcode.commands['BLTOUCH_DEBUG'](gcmd)")
        .unwrap();
    let py_info = py.eval("list(gcmd.info)").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new("BLTOUCH_DEBUG", vec![]);
    let g: GCmd = Arc::clone(&fake) as GCmd;
    stack.gcode.dispatch("BLTOUCH_DEBUG", &g).unwrap();
    let rust_info = fake.info.lock().unwrap().clone();
    assert_equal(&rec("debug_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("debug_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn debug_command_invalid_name_lists_commands() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_DEBUG', 'BLTOUCH_DEBUG COMMAND=foo', {'COMMAND': 'foo'}); gcode.commands['BLTOUCH_DEBUG'](gcmd)")
        .unwrap();
    let py_info = py.eval("list(gcmd.info)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new(
        "BLTOUCH_DEBUG",
        vec![("COMMAND".to_string(), "foo".to_string())],
    );
    let g: GCmd = Arc::clone(&fake) as GCmd;
    stack.gcode.dispatch("BLTOUCH_DEBUG", &g).unwrap();
    let rust_info = fake.info.lock().unwrap().clone();
    assert_equal(&rec("debug_invalid"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
}

#[test]
fn debug_command_pin_up_default_duration() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("toolhead_state['last'] = 5.0; blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_DEBUG', 'BLTOUCH_DEBUG COMMAND=pin_up', {'COMMAND': 'pin_up'}); gcode.commands['BLTOUCH_DEBUG'](gcmd)")
        .unwrap();
    let py_info = py.eval("list(gcmd.info)").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_dwell = py.eval("list(toolhead_state['dwells'])").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    stack.shared.lock().unwrap().last_move_time = 5.0;
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new(
        "BLTOUCH_DEBUG",
        vec![("COMMAND".to_string(), "pin_up".to_string())],
    );
    let g: GCmd = Arc::clone(&fake) as GCmd;
    stack.gcode.dispatch("BLTOUCH_DEBUG", &g).unwrap();
    let rust_info = fake.info.lock().unwrap().clone();
    assert_equal(&rec("debug_up_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("debug_up_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("debug_up_dwell"), "dwell", &py_dwell, &dwell_value(&stack.shared)).unwrap();
}

#[test]
fn store_command_invalid_mode() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_STORE', 'BLTOUCH_STORE', {}); gcode.commands['BLTOUCH_STORE'](gcmd)")
        .unwrap();
    let py_info = py.eval("list(gcmd.info)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new("BLTOUCH_STORE", vec![]);
    let g: GCmd = Arc::clone(&fake) as GCmd;
    stack.gcode.dispatch("BLTOUCH_STORE", &g).unwrap();
    let rust_info = fake.info.lock().unwrap().clone();
    assert_equal(&rec("store_invalid"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
}

#[test]
fn store_command_5v_sequence() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_STORE', 'BLTOUCH_STORE MODE=5V', {'MODE': '5V'}); log_lines.clear(); gcode.commands['BLTOUCH_STORE'](gcmd)")
        .unwrap();
    let py_info = py.eval("list(gcmd.info)").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new(
        "BLTOUCH_STORE",
        vec![("MODE".to_string(), "5V".to_string())],
    );
    let g: GCmd = Arc::clone(&fake) as GCmd;
    let (rust_result, rust_logs) = with_log_capture(|| stack.gcode.dispatch("BLTOUCH_STORE", &g));
    rust_result.unwrap();
    let rust_info = fake.info.lock().unwrap().clone();
    assert_equal(&rec("store5v_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("store5v_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("store5v_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

#[test]
fn store_command_od_sequence() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('BLTOUCH_STORE', 'BLTOUCH_STORE MODE=OD', {'MODE': 'OD'}); log_lines.clear(); gcode.commands['BLTOUCH_STORE'](gcmd)")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_logs = py.eval("list(log_lines)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let _blt = make_probe(&stack);
    let fake = FakeGcmd::new(
        "BLTOUCH_STORE",
        vec![("MODE".to_string(), "OD".to_string())],
    );
    let g: GCmd = Arc::clone(&fake) as GCmd;
    let (rust_result, rust_logs) = with_log_capture(|| stack.gcode.dispatch("BLTOUCH_STORE", &g));
    rust_result.unwrap();
    assert_equal(&rec("storeod_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("storeod_logs"), "logs", &py_logs, &log_value(&rust_logs)).unwrap();
}

// ---------------------------------------------------------------------------
// Probe-prepare / session groups
// ---------------------------------------------------------------------------

#[test]
fn probe_prepare_stow_true_sensor_cycle() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.append(1.0); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); blt._probe_prepare(); blt._probe_prepare()")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_dwell = py.eval("list(toolhead_state['dwells'])").unwrap();
    let py_waits = py.eval("list(endstop_home_wait_calls)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![HomeWait::Ok(1.0)]);
    let blt = make_probe(&stack);
    blt.probe_prepare().unwrap();
    blt.probe_prepare().unwrap();
    assert_equal(&rec("prepare_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
    assert_equal(&rec("prepare_dwell"), "dwell", &py_dwell, &dwell_value(&stack.shared)).unwrap();
    assert_equal(&rec("prepare_waits"), "waits", &py_waits, &home_wait_value(&stack.shared)).unwrap();
}

#[test]
fn probe_prepare_stow_false_multi_cycle() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("stow_on_each_sample", "False"),
    ]))
    .unwrap();
    py.exec("home_wait_script.extend([1.0, 1.0, 1.0]); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); gcmd = GcmdFake('PROBE', 'PROBE', {}); blt.start_probe_session(gcmd); blt._probe_prepare(); blt._probe_prepare(); blt.end_probe_session()")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(
        base_opts(vec![("stow_on_each_sample", FakeOpt::Bool(false))]),
        vec![HomeWait::Ok(1.0), HomeWait::Ok(1.0), HomeWait::Ok(1.0)],
    );
    let blt = make_probe(&stack);
    let fake = FakeGcmd::new("PROBE", vec![]);
    let g: GCmd = Arc::clone(&fake) as GCmd;
    blt.start_probe_session(&g).unwrap();
    blt.probe_prepare().unwrap();
    blt.probe_prepare().unwrap();
    ProbeSessionApi::end_probe_session(&*blt).unwrap();
    assert_equal(&rec("multi_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn probe_prepare_no_sensor_check() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
        ("pin_up_touch_mode_reports_triggered", "False"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); blt._probe_prepare()")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(
        base_opts(vec![("pin_up_touch_mode_reports_triggered", FakeOpt::Bool(false))]),
        vec![],
    );
    let blt = make_probe(&stack);
    blt.probe_prepare().unwrap();
    assert_equal(&rec("prepare_nosensor"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn home_start_clamps_rest_time() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); blt.home_start(1.0, 0.001, 4, 0.01, triggered=False)")
        .unwrap();
    let py_home = py.eval("list(endstop_home_calls)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let blt = make_probe(&stack);
    let endstop: Arc<dyn EndstopApi> = Arc::clone(&blt) as Arc<dyn EndstopApi>;
    let _comp = endstop.home_start(1.0, 0.001, 4, 0.01, Some(false));
    assert_equal(&rec("home_start"), "home", &py_home, &home_start_value(&stack.shared)).unwrap();
}

#[test]
fn wait_for_trigger_raises_when_off() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config)); blt.home_start(1.0, 0.001, 4, 0.001, triggered=False); blt._wait_for_trigger(0.0)")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(base_opts(vec![]), vec![]);
    let blt = make_probe(&stack);
    let endstop: Arc<dyn EndstopApi> = Arc::clone(&blt) as Arc<dyn EndstopApi>;
    let _comp = endstop.home_start(1.0, 0.001, 4, 0.001, Some(false));
    blt.wait_for_trigger(0.0);
    assert_equal(&rec("wait_trig"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn probe_finish_without_home_is_attribute_error() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))")
        .unwrap();
    let py_exc = match py.exec("blt._probe_finish()") {
        Err(e) => e,
        Ok(_) => panic!("expected AttributeError"),
    };

    let stack = make_stack(base_opts(vec![]), vec![]);
    let blt = make_probe(&stack);
    let rust_err = blt.probe_finish().expect_err("expect AttributeError");
    assert_equal(&rec("finish_err"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(rust_err.class_name().to_string())).unwrap();
    assert_equal(&rec("finish_err"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(rust_err.message().to_string())).unwrap();
}

#[test]
fn test_sensor_fails_after_retries() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_make_world(vec![
        ("control_pin", "'PA1'"),
        ("sensor_pin", "'PA2'"),
        ("z_offset", "0.0"),
    ]))
    .unwrap();
    py.exec("home_wait_script.extend([0.0, 0.0, 0.0]); blt = bt.BLTouchProbe(config, FakeOffsets(config), FakeParams(config))")
        .unwrap();
    let py_exc = match py.exec("blt._test_sensor()") {
        Err(e) => e,
        Ok(_) => panic!("expected CommandError"),
    };
    let py_pwm = py.eval("list(pwm_calls)").unwrap();
    let py_info = py.eval("list(gcode.info)").unwrap();

    let stack = make_stack(
        base_opts(vec![]),
        vec![HomeWait::Ok(0.0), HomeWait::Ok(0.0), HomeWait::Ok(0.0)],
    );
    let blt = make_probe(&stack);
    let rust_err = blt.test_sensor().expect_err("expect CommandError");
    assert_equal(&rec("sensor_err"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(rust_err.class_name().to_string())).unwrap();
    assert_equal(&rec("sensor_err"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(rust_err.message().to_string())).unwrap();
    let rust_info: Vec<String> = stack.gcode.info_list();
    assert_equal(&rec("sensor_info"), "info", &py_info, &Value::List(rust_info.iter().map(|s| Value::Str(s.clone())).collect())).unwrap();
    assert_equal(&rec("sensor_pwm"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

// ---------------------------------------------------------------------------
// PrinterBLTouch
// ---------------------------------------------------------------------------

fn py_printer_setup(opts_literal: &str) -> String {
    format!(
        "printer, pins, gcode, toolhead, config = make_world({opts_literal}); printer_obj = bt.PrinterBLTouch(config)"
    )
}

#[test]
fn printer_construction_and_delegation() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_printer_setup(
        "{'control_pin': 'PA1', 'sensor_pin': 'PA2', 'x_offset': 1.0, 'y_offset': 2.0, 'z_offset': 3.0}",
    ))
    .unwrap();
    let py_offsets = py.eval("printer_obj.get_offsets()").unwrap();
    let py_params = py.eval("printer_obj.get_probe_params()").unwrap();
    let py_status = py.eval("printer_obj.get_status(0.0)").unwrap();
    let py_chips = py.eval("sorted(pins.registered_chips.keys())").unwrap();
    let py_pins = py.eval("list(pins.setup_calls)").unwrap();
    let py_cmds = py.eval("sorted(gcode.commands.keys())").unwrap();

    let stack = make_stack(
        vec![
            ("control_pin".to_string(), FakeOpt::Str("PA1".to_string())),
            ("sensor_pin".to_string(), FakeOpt::Str("PA2".to_string())),
            ("x_offset".to_string(), FakeOpt::Float(1.0)),
            ("y_offset".to_string(), FakeOpt::Float(2.0)),
            ("z_offset".to_string(), FakeOpt::Float(3.0)),
        ],
        vec![],
    );
    let printer_obj = PrinterBLTouch::new(&stack.config).unwrap();
    let (ox, oy, oz) = printer_obj.get_offsets(None);
    let rust_offsets_value = Value::Tuple(vec![float_value(ox), float_value(oy), float_value(oz)]);
    assert_equal(&rec_printer("printer_offsets"), "offsets", &py_offsets, &rust_offsets_value).unwrap();
    let rust_params = printer_obj.get_probe_params(None).unwrap();
    let mut params_dict = IndexMap::new();
    params_dict.insert(PyValue::Str("probe_speed".to_string()), PyValue::Float(rust_params.probe_speed));
    params_dict.insert(PyValue::Str("lift_speed".to_string()), PyValue::Float(rust_params.lift_speed));
    params_dict.insert(PyValue::Str("samples".to_string()), PyValue::Int(PyInt::from_i64(rust_params.samples)));
    params_dict.insert(PyValue::Str("sample_retract_dist".to_string()), PyValue::Float(rust_params.sample_retract_dist));
    params_dict.insert(PyValue::Str("samples_tolerance".to_string()), PyValue::Float(rust_params.samples_tolerance));
    params_dict.insert(PyValue::Str("samples_tolerance_retries".to_string()), PyValue::Int(PyInt::from_i64(rust_params.samples_tolerance_retries)));
    params_dict.insert(PyValue::Str("samples_result".to_string()), PyValue::Str(rust_params.samples_result.clone()));
    assert_equal(&rec_printer("printer_params"), "params", &py_params, &pyvalue_of(&PyValue::Dict(params_dict))).unwrap();
    let rust_status = printer_obj.get_status(0.0);
    assert_equal(&rec_printer("printer_status"), "status", &py_status, &pyvalue_of(&rust_status)).unwrap();
    let rust_chips: Vec<Value> = {
        let mut c = stack.pins.chips.lock().unwrap().clone();
        c.sort();
        c.iter().map(|s| Value::Str(s.clone())).collect()
    };
    assert_equal(&rec_printer("printer_chips"), "chips", &py_chips, &Value::List(rust_chips)).unwrap();
    assert_equal(&rec_printer("printer_pins"), "pins", &py_pins, &pins_setup_value(&stack.pins)).unwrap();
    let rust_cmds: Vec<Value> = stack
        .gcode
        .command_keys()
        .iter()
        .map(|s| Value::Str(s.clone()))
        .collect();
    assert_equal(&rec_printer("printer_cmds"), "cmds", &py_cmds, &Value::List(rust_cmds)).unwrap();
}

#[test]
fn printer_missing_z_offset_errors() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    let py_exc = match py.exec(&py_printer_setup(
        "{'control_pin': 'PA1', 'sensor_pin': 'PA2'}",
    )) {
        Err(e) => e,
        Ok(_) => panic!("expected ConfigError"),
    };
    let stack = make_stack(
        vec![
            ("control_pin".to_string(), FakeOpt::Str("PA1".to_string())),
            ("sensor_pin".to_string(), FakeOpt::Str("PA2".to_string())),
        ],
        vec![],
    );
    let err = match PrinterBLTouch::new(&stack.config) {
        Ok(_) => panic!("expect ConfigError"),
        Err(e) => e,
    };
    assert_equal(&rec_printer("printer_no_z"), "exc_class", &Value::Str(py_exc.class_name), &Value::Str(err.class_name().to_string())).unwrap();
    assert_equal(&rec_printer("printer_no_z"), "exc_msg", &Value::Str(py_exc.message), &Value::Str(err.message().to_string())).unwrap();
}

#[test]
fn printer_start_probe_session_roundtrip() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_printer_setup(
        "{'control_pin': 'PA1', 'sensor_pin': 'PA2', 'z_offset': 0.0, 'stow_on_each_sample': False}",
    ))
    .unwrap();
    py.exec("gcmd = GcmdFake('PROBE', 'PROBE', {}); sess = printer_obj.start_probe_session(gcmd)")
        .unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(
        vec![
            ("control_pin".to_string(), FakeOpt::Str("PA1".to_string())),
            ("sensor_pin".to_string(), FakeOpt::Str("PA2".to_string())),
            ("z_offset".to_string(), FakeOpt::Float(0.0)),
            ("stow_on_each_sample".to_string(), FakeOpt::Bool(false)),
        ],
        vec![],
    );
    let printer_obj = PrinterBLTouch::new(&stack.config).unwrap();
    let fake = FakeGcmd::new("PROBE", vec![]);
    let g: GCmd = Arc::clone(&fake) as GCmd;
    let sess = printer_obj.start_probe_session(&g).unwrap();
    drop(sess);
    assert_equal(&rec_printer("printer_session"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}

#[test]
fn printer_session_pull_and_end() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let _guard = test_lock();
    py.exec(&py_printer_setup(
        "{'control_pin': 'PA1', 'sensor_pin': 'PA2', 'z_offset': 0.0}",
    ))
    .unwrap();
    py.exec("gcmd = GcmdFake('PROBE', 'PROBE', {}); sess = printer_obj.start_probe_session(gcmd)")
        .unwrap();
    let py_pulled = py.eval("sess.pull_probed_results()").unwrap();
    py.exec("sess.end_probe_session()").unwrap();
    let py_pwm = py.eval("list(pwm_calls)").unwrap();

    let stack = make_stack(
        vec![
            ("control_pin".to_string(), FakeOpt::Str("PA1".to_string())),
            ("sensor_pin".to_string(), FakeOpt::Str("PA2".to_string())),
            ("z_offset".to_string(), FakeOpt::Float(0.0)),
        ],
        vec![],
    );
    let printer_obj = PrinterBLTouch::new(&stack.config).unwrap();
    let fake = FakeGcmd::new("PROBE", vec![]);
    let g: GCmd = Arc::clone(&fake) as GCmd;
    let sess = printer_obj.start_probe_session(&g).unwrap();
    let rust_pulled: Vec<ProbeResult> = sess.pull_probed_results().unwrap();
    sess.end_probe_session().unwrap();
    let rust_pulled_value = Value::List(
        rust_pulled
            .iter()
            .map(|p| Value::Tuple(vec![
                float_value(p.bed_x), float_value(p.bed_y), float_value(p.bed_z),
            ]))
            .collect(),
    );
    assert_equal(&rec_printer("printer_pull"), "pulled", &py_pulled, &rust_pulled_value).unwrap();
    assert_equal(&rec_printer("printer_end"), "pwm", &py_pwm, &pwm_value(&stack.shared)).unwrap();
}
