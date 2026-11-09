//! Parity tests for `extras::pid_calibrate` (port of
//! `klippy/extras/pid_calibrate.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a Python `ControlAutoTune` /
//! `PIDCalibrate` is built on minimal Python fakes for the printer, the
//! `heaters` object, the heater, the toolhead, `configfile` and the
//! `gcode` command, then driven with the same temperature samples as the
//! Rust side. Return values (floats by bits), raised errors (class and
//! message), log lines, G-code responses, `configfile.set` calls, MCU
//! PWM calls and the `write_file` bytes are compared exactly.
//!
//! The Rust side reaches the real `Heater` (`PrinterHeatersApi::
//! lookup_heater` returns the concrete `Arc<Heater>`), so rust worlds
//! build production heaters with `Heater::new`, wired to a recording
//! `FakePwm`. The Python heater fake mirrors the reference `Heater`
//! (`heaters.py:72-86,133-136`) for the members `pid_calibrate` uses
//! (`set_pwm` filtering, `alter_target` clamping, `set_control`
//! clearing the target) and records through the same MCU PWM shape.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test pid_calibrate
//! ```

use std::sync::Arc;
use kos::sync::Mutex;

use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorFactory, SensorApi, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::pid_calibrate::{
    module_factory, ControlAutoTune, PIDCalibrate, CMD_PID_CALIBRATE_HELP, TUNE_PID_DELTA,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetFloatListOpts, GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapperApi, Chip, CommandQueue,
    CommandQueryWrapper, CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts,
    Mcu, McuApi, Pin, PinParams, PinResolver, PinsApi, ParsePinOpts, PwmPin, PwmPinApi,
    TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    Rail, RailOpts, Stepper, StepperFactory, StepperFactoryApi, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, ExtruderStepper, HomingTargetApi, Kinematics, KinematicsApi, LookaheadCallback,
    MoveApi, PrinterExtruder, PrinterExtruderApi, ToolHeadApi,
};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::{float_from_string, int_from_string};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser, logging
import extras.pid_calibrate as pid_calibrate
import extras.heaters as heaters

class CommandError(Exception):
    pass

class _Sentinel: pass
sentinel = _Sentinel()

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self): self.objects = {}
    def add_object(self, name, obj): self.objects[name] = obj
    def lookup_object(self, name, default=sentinel):
        if name in self.objects: return self.objects[name]
        if default is sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    def __init__(self, printer): self._printer = printer
    def get_printer(self): return self._printer
    def get_name(self): return 'pid_calibrate'

class FakeControl:
    def __init__(self): self.calls = []
    def temperature_update(self, rt, temp, target):
        self.calls.append(('temperature_update', rt, temp, target))
    def check_busy(self, et, st, tt): return False

class FakeMcu:
    def estimated_print_time(self, eventtime): return eventtime

class FakeMcuPwm:
    def __init__(self): self.calls = []
    def setup_cycle_time(self, t): pass
    def setup_max_duration(self, t): pass
    def get_mcu(self): return FakeMcu()
    def set_pwm(self, t, v): self.calls.append(('set_pwm', t, v))

class FakeHeater:
    # Mirrors klippy/extras/heaters.py Heater for the members pid_calibrate
    # uses: set_pwm filtering (heaters.py:72), alter_target clamping
    # (heaters.py:133), set_control clearing the target (heaters.py:127).
    def __init__(self, name):
        self._name = name; self.min_temp = -100.; self.max_temp = 300.
        self.max_power = 1.; self.pwm_delay = 0.5; self.min_pwm_change = 0.05
        self.verify_mainthread_time = -999.; self.target_temp = 0.
        self.next_pwm_time = 0.; self.last_pwm_value = 0.
        self.control = self.initial_control = FakeControl()
        self.mcu_pwm = FakeMcuPwm()
        self.calls = []
    def get_name(self): return self._name
    def get_max_power(self): return self.max_power
    def get_pwm_delay(self): return self.pwm_delay
    def set_pwm(self, rt, v):
        if self.target_temp <= 0. or rt > self.verify_mainthread_time: v = 0.
        if ((rt < self.next_pwm_time or not self.last_pwm_value)
            and abs(v - self.last_pwm_value) < self.min_pwm_change):
            return
        pwm_time = rt + self.pwm_delay
        self.next_pwm_time = pwm_time + 3. - (3. * self.pwm_delay + 0.001)
        self.last_pwm_value = v
        self.mcu_pwm.set_pwm(pwm_time, v)
    def alter_target(self, t):
        if t: t = max(self.min_temp, min(self.max_temp, t))
        self.target_temp = t
        self.calls.append(('alter_target', t))
    def set_control(self, c):
        old = self.control; self.control = c; self.target_temp = 0.
        self.calls.append(('set_control',)); return old
    def set_temp(self, deg):
        if deg and (deg < self.min_temp or deg > self.max_temp):
            raise CommandError(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (deg, self.min_temp, self.max_temp))
        self.target_temp = deg
    def temperature_callback(self, rt, temp):
        self.control.temperature_update(rt, temp, self.target_temp)
    def stats(self, et):
        self.verify_mainthread_time = et + 5.
        return (self.target_temp != 0., '')

class FakeHeaters:
    def __init__(self, heater):
        self.heater = heater; self.calls = []; self.samples = []; self.fail = None
    def lookup_heater(self, name):
        self.calls.append(('lookup_heater', name))
        if name != self.heater.get_name():
            raise configparser.Error("Unknown heater '%s'" % (name,))
        return self.heater
    def set_temperature(self, heater, temp, wait):
        self.calls.append(('set_temperature', temp, wait))
        if self.fail == 'command':
            raise CommandError("fake command failure")
        heater.set_temp(temp)
        for (rt, t) in self.samples:
            heater.temperature_callback(rt, t)

class FakeToolhead:
    def __init__(self): self.calls = 0
    def get_last_move_time(self):
        self.calls += 1
        return 123.5

class FakeConfigFile:
    def __init__(self): self.calls = []
    def set(self, section, option, value): self.calls.append((section, option, value))

class FakeGCode:
    def __init__(self): self.registered = []
    def register_command(self, cmd, func, desc=None): self.registered.append((cmd, desc))

class FakeGCmd:
    error = CommandError
    def __init__(self, params, commandline):
        self.params = params; self.commandline = commandline
        self.responded = []
    def get_commandline(self): return self.commandline
    def get(self, name, default=sentinel):
        value = self.params.get(name)
        if value is None:
            if default is sentinel:
                raise CommandError("Error on '%s': missing %s" % (self.commandline, name))
            return default
        return value
    def get_int(self, name, default=sentinel):
        value = self.params.get(name)
        if value is None:
            if default is sentinel:
                raise CommandError("Error on '%s': missing %s" % (self.commandline, name))
            return default
        try: return int(value)
        except Exception:
            raise CommandError("Error on '%s': unable to parse %s" % (self.commandline, value))
    def get_float(self, name, default=sentinel):
        value = self.params.get(name)
        if value is None:
            if default is sentinel:
                raise CommandError("Error on '%s': missing %s" % (self.commandline, name))
            return default
        try: return float(value)
        except Exception:
            raise CommandError("Error on '%s': unable to parse %s" % (self.commandline, value))
    def respond_info(self, msg, log=True): self.responded.append((msg, log))

log_messages = []
class _Cap(logging.Handler):
    def emit(self, record): log_messages.append(record.getMessage())
logging.getLogger().addHandler(_Cap())
logging.getLogger().setLevel(logging.INFO)
"#;

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG_MESSAGES: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static LOG_INSTALLED: std::sync::Once = std::sync::Once::new();
    LOG_INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_MESSAGES.with(|logs| logs.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

/// Reset this thread's log buffer (the sink is installed once per process).
fn log_serial() {
    install_log_sink();
    LOG_MESSAGES.with(|logs| logs.borrow_mut().clear());
}

fn take_logs() -> Value {
    let logs = LOG_MESSAGES.with(|logs| std::mem::take(&mut *logs.borrow_mut()));
    Value::List(logs.iter().map(|s| Value::str(s)).collect())
}

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Arc::new(Mutex::new(IndexMap::new())),
        }
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
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
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
        match objects.get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
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
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

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
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by pid_calibrate")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.options.get(option) {
            Some(s) => {
                let v = pycompat::text::float_from_string(s)
                    .map_err(|_| parse_error(&self.section, option))?;
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option,
                            self.section,
                            pycompat::text::float_repr(minv)
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option,
                            self.section,
                            pycompat::text::float_repr(maxv)
                        )));
                    }
                }
                if let Some(a) = opts.above {
                    if v <= a {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.section,
                            pycompat::text::float_repr(a)
                        )));
                    }
                }
                if let Some(b) = opts.below {
                    if v >= b {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option,
                            self.section,
                            pycompat::text::float_repr(b)
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
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by pid_calibrate")
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
        panic!("getlists is not used by pid_calibrate")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by pid_calibrate")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by pid_calibrate")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by pid_calibrate")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by pid_calibrate")
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

struct FakeSensor;

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        0.5
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
        _cb: Arc<dyn Fn(PyValue) + Send + Sync + 'static>,
        _response: &str,
        _oid: Option<i64>,
    ) -> Arc<dyn AsyncResponseWrapperApi> {
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
    calls: GCell<Vec<Value>>,
}

impl FakePwm {
    fn new(mcu: Mcu) -> FakePwm {
        FakePwm {
            mcu,
            calls: GCell::new(Vec::new()),
        }
    }
    fn calls_value(&self) -> Value {
        Value::List(self.calls.borrow().clone())
    }
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: Option<bool>) {}
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        self.calls.borrow_mut().push(Value::Tuple(vec![
            Value::str("set_pwm"),
            Value::float(print_time),
            Value::float(value),
        ]));
    }
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
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
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

struct FakeHeatersState {
    calls: Vec<Value>,
    samples: Vec<(f64, f64)>,
    fail: Option<String>,
}

struct FakeHeaters {
    heaters: IndexMap<String, Arc<Heater>>,
    state: GCell<FakeHeatersState>,
}

impl FakeHeaters {
    fn new(heaters: IndexMap<String, Arc<Heater>>, samples: Vec<(f64, f64)>) -> FakeHeaters {
        FakeHeaters {
            heaters,
            state: GCell::new(FakeHeatersState {
                calls: Vec::new(),
                samples,
                fail: None,
            }),
        }
    }
    fn calls_value(&self) -> Value {
        Value::List(self.state.borrow().calls.clone())
    }
    fn set_fail(&self, fail: Option<&str>) {
        self.state.borrow_mut().fail = fail.map(str::to_string);
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        panic!("add_sensor_factory is not used by pid_calibrate")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used by pid_calibrate")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.heaters.keys().cloned().collect()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        self.state.borrow_mut().calls.push(Value::Tuple(vec![
            Value::str("lookup_heater"),
            Value::str(heater_name),
        ]));
        match self.heaters.get(heater_name) {
            Some(h) => Ok(h.clone()),
            None => Err(KlippyError::config(format!(
                "Unknown heater '{heater_name}'"
            ))),
        }
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by pid_calibrate")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used by pid_calibrate")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used by pid_calibrate")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used by pid_calibrate")
    }
    fn set_temperature(&self, heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        self.state.borrow_mut().calls.push(Value::Tuple(vec![
            Value::str("set_temperature"),
            Value::float(temp),
            Value::Bool(wait),
        ]));
        if self.state.borrow().fail.as_deref() == Some("command") {
            return Err(KlippyError::command("fake command failure"));
        }
        // `PrinterHeaters.set_temperature`: `heater.set_temp(temp)`, then
        // the temperature callback drives the current control.
        heater.set_temp(temp)?;
        for (read_time, temp_sample) in self.state.borrow().samples.clone() {
            heater.temperature_callback(read_time, temp_sample);
        }
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

struct DummyTrapq;

impl TrapqApi for DummyTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct DummyKinematics;

impl KinematicsApi for DummyKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(Vec::new())
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}

struct DummyFactory;

fn unused_err<T>() -> KResult<T> {
    Err(KlippyError::command("unused stepper factory in pid_calibrate tests"))
}

impl StepperFactoryApi for DummyFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        unused_err()
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unused_err()
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unused_err()
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        unused_err()
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        unused_err()
    }
}

struct DummyExtruder;

impl klippy_api::toolhead::ExtraAxisApi for DummyExtruder {
    fn get_name(&self) -> String {
        String::new()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        None
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn check_move(&self, _mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        Ok(())
    }
    fn calc_junction(
        &self,
        _prev_move: &dyn MoveApi,
        _mv: &dyn MoveApi,
        _ea_index: usize,
    ) -> f64 {
        0.
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for DummyExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::command("Extruder not configured"))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<ExtruderStepper> {
        None
    }
}

struct FakeToolhead {
    calls: GCell<i64>,
}

impl FakeToolhead {
    fn new() -> FakeToolhead {
        FakeToolhead { calls: GCell::new(0) }
    }
    fn calls(&self) -> i64 {
        self.calls.get()
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        Vec::new()
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(DummyKinematics)
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        self.calls.set(self.calls.get() + 1);
        Ok(123.5)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(DummyTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0., 0.)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0., 0., 0., 0.)
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0., 0., false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        Arc::new(DummyExtruder)
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(DummyFactory)
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakeConfigFile {
    printer: Printer,
    calls: GCell<Vec<Value>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::config("unused configfile in pid_calibrate tests"))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::config("unused configfile in pid_calibrate tests"))
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(
        &self,
        _mcu: &klippy_api::mcu::Mcu,
        _feature: &str,
        _msg: Option<String>,
    ) {
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.calls.borrow_mut().push(Value::Tuple(vec![
            Value::str(section),
            Value::str(option),
            Value::str(value),
        ]));
    }
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

struct FakeGCode {
    reactor: Reactor,
    registered: GCell<Vec<Value>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            reactor: Reactor::new(false),
            registered: GCell::new(Vec::new()),
        }
    }
    fn registered_value(&self) -> Value {
        Value::List(self.registered.borrow().clone())
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let desc = match &opts.desc {
            Some(desc) => Value::str(desc),
            None => Value::Null,
        };
        self.registered.borrow_mut().push(Value::Tuple(vec![
            Value::str(cmd),
            desc,
        ]));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
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
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd::new(params.clone(), commandline.to_string()))
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
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

struct FakeGCmd {
    params: IndexMap<String, String>,
    commandline: String,
    responded: GCell<Vec<Value>>,
}

impl FakeGCmd {
    fn new(params: IndexMap<String, String>, commandline: String) -> FakeGCmd {
        FakeGCmd {
            params,
            commandline,
            responded: GCell::new(Vec::new()),
        }
    }
    fn responded_value(&self) -> Value {
        Value::List(self.responded.borrow().clone())
    }
    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::command(format!(
            "Error on '{}': missing {name}",
            self.commandline
        ))
    }
    fn unable(&self, value: &str) -> KlippyError {
        KlippyError::command(format!(
            "Error on '{}': unable to parse {value}",
            self.commandline
        ))
    }
}

fn bound_text(bound: &PyValue) -> String {
    bound.str()
}

/// Numeric rendering of a `get` bound for the min/max/above/below checks.
fn bound_float(bound: &PyValue) -> Option<f64> {
    match bound {
        PyValue::Float(f) => Some(*f),
        PyValue::Int(n) => n.0.to_string().parse::<f64>().ok(),
        PyValue::Bool(b) => Some(if *b { 1. } else { 0. }),
        _ => None,
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.missing(name)),
                Some(default) => Ok(default),
            };
        };
        let value = value.clone();
        let parsed = match opts.parser {
            klippy_api::gcode::GetParser::Str => PyValue::Str(value.clone()),
            klippy_api::gcode::GetParser::Int => match int_from_string(&value, 10) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(self.unable(&value)),
            },
            klippy_api::gcode::GetParser::IntAutoBase => {
                match int_from_string(&value, 0) {
                    Ok(n) => PyValue::Int(n),
                    Err(_) => return Err(self.unable(&value)),
                }
            }
            klippy_api::gcode::GetParser::Float => match float_from_string(&value) {
                Ok(f) => PyValue::Float(f),
                Err(_) => return Err(self.unable(&value)),
            },
        };
        let Some(v) = bound_float(&parsed) else {
            return Ok(parsed);
        };
        if let Some(minval) = &opts.minval {
            if let Some(mn) = bound_float(minval) {
                if v < mn {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': {name} must have minimum of {}",
                        self.commandline,
                        bound_text(minval)
                    )));
                }
            }
        }
        if let Some(maxval) = &opts.maxval {
            if let Some(mx) = bound_float(maxval) {
                if v > mx {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': {name} must have maximum of {}",
                        self.commandline,
                        bound_text(maxval)
                    )));
                }
            }
        }
        if let Some(above) = &opts.above {
            if let Some(ab) = bound_float(above) {
                if v <= ab {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': {name} must be above {}",
                        self.commandline,
                        bound_text(above)
                    )));
                }
            }
        }
        if let Some(below) = &opts.below {
            if let Some(be) = bound_float(below) {
                if v >= be {
                    return Err(KlippyError::command(format!(
                        "Error on '{}': {name} must be below {}",
                        self.commandline,
                        bound_text(below)
                    )));
                }
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.missing(name)),
                Some(default) => Ok(default),
            };
        };
        let value = value.clone();
        let parsed = match int_from_string(&value, 10) {
            Ok(n) => match n.0.to_string().parse::<i64>() {
                Ok(v) => v,
                Err(_) => return Err(self.unable(&value)),
            },
            Err(_) => return Err(self.unable(&value)),
        };
        if let Some(minval) = opts.minval {
            if parsed < minval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {minval}",
                    self.commandline
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if parsed > maxval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {maxval}",
                    self.commandline
                )));
            }
        }
        Ok(Some(parsed))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.missing(name)),
                Some(default) => Ok(default),
            };
        };
        let value = value.clone();
        let parsed = match float_from_string(&value) {
            Ok(v) => v,
            Err(_) => return Err(self.unable(&value)),
        };
        if let Some(minval) = opts.minval {
            if parsed < minval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {minval}",
                    self.commandline
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if parsed > maxval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {maxval}",
                    self.commandline
                )));
            }
        }
        if let Some(above) = opts.above {
            if parsed <= above {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be above {above}",
                    self.commandline
                )));
            }
        }
        if let Some(below) = opts.below {
            if parsed >= below {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be below {below}",
                    self.commandline
                )));
            }
        }
        Ok(Some(parsed))
    }
    fn get_command(&self) -> String {
        "PID_CALIBRATE".to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.responded.borrow_mut().push(Value::Tuple(vec![
            Value::str(msg),
            Value::Bool(log),
        ]));
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pid_calibrate fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pid_calibrate oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: Value, actual: Value) {
    let record = Record {
        v: 1,
        unit: "extras.pid_calibrate".to_string(),
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
        Ok(_) => panic!("{label}: rust call returned but must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class",
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message",
    );
}

/// `(time, value)` style sample lists on either side.
fn samples_value(samples: &[(f64, f64)]) -> Value {
    Value::List(
        samples
            .iter()
            .map(|(a, b)| Value::Tuple(vec![Value::float(*a), Value::float(*b)]))
            .collect(),
    )
}

/// A `(Kp, Ki, Kd)` triple.
fn pid_value(pid: (f64, f64, f64)) -> Value {
    Value::Tuple(vec![
        Value::float(pid.0),
        Value::float(pid.1),
        Value::float(pid.2),
    ])
}

fn pid_result_value(actual: KResult<(f64, f64, f64)>) -> Value {
    match actual {
        Ok(pid) => pid_value(pid),
        Err(e) => panic!("rust calc_pid raised {}: {}", e.class_name(), e.message()),
    }
}

struct World {
    config: Config,
    heater: Arc<Heater>,
    pwm: Arc<FakePwm>,
    heaters: Arc<FakeHeaters>,
    toolhead: Arc<FakeToolhead>,
    configfile: Arc<FakeConfigFile>,
    gcode: Arc<FakeGCode>,
}

fn build_world(samples: Vec<(f64, f64)>) -> World {
    let fake_printer = FakePrinter::new();
    let printer: Printer = Arc::new(fake_printer.clone());
    let mcu: Mcu = Arc::new(FakeMcu);
    let pwm = Arc::new(FakePwm::new(mcu));
    let pwm_pin: PwmPin = pwm.clone();
    let pins_obj: ObjectRef = Arc::new(FakePins { pwm: pwm_pin });
    fake_printer.add("pins", pins_obj);
    let gcode = Arc::new(FakeGCode::new());
    fake_printer.add("gcode", gcode.clone() as ObjectRef);
    // Real heater: `PrinterHeatersApi::lookup_heater` returns `Arc<Heater>`.
    let hcfg: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "extruder",
        vec![
            ("min_temp", "-100"),
            ("max_temp", "300"),
            ("control", "watermark"),
            ("heater_pin", "PA1"),
        ],
    ));
    let sensor: Arc<dyn TemperatureSensorApi> = Arc::new(FakeSensor);
    let heater = Heater::new(&hcfg, sensor).expect("Heater::new");
    // Arm `verify_mainthread_time` so `set_pwm` passes values through,
    // mirroring `pheater.stats(200.0)` on the Python side.
    let _ = heater.stats(200.);
    let mut heaters_map = IndexMap::new();
    heaters_map.insert("extruder".to_string(), heater.clone());
    let heaters = Arc::new(FakeHeaters::new(heaters_map, samples));
    let toolhead = Arc::new(FakeToolhead::new());
    let configfile = Arc::new(FakeConfigFile {
        printer: printer.clone(),
        calls: GCell::new(Vec::new()),
    });
    fake_printer.add("heaters", heaters.clone() as ObjectRef);
    fake_printer.add("toolhead", toolhead.clone() as ObjectRef);
    fake_printer.add("configfile", configfile.clone() as ObjectRef);
    let config: Config = Arc::new(FakeConfig::new(printer.clone(), "pid_calibrate", vec![]));
    World {
        config,
        heater,
        pwm,
        heaters,
        toolhead,
        configfile,
        gcode,
    }
}

/// A world for direct `ControlAutoTune` drives, with the heater target
/// armed at the calibration target like `set_temperature` does.
fn direct_world() -> World {
    let world = build_world(Vec::new());
    world.heater.set_temp(200.).expect("set_temp in range");
    world
}

fn python_setup_samples(py: &mut pyoracle::Oracle, samples: &[(f64, f64)]) {
    let items: Vec<String> = samples
        .iter()
        .map(|(t, v)| format!("({t:?}, {v:?})"))
        .collect();
    let samples_py = format!("[{}]", items.join(", "));
    py.exec(&format!(
        "pp = FakePrinter(); pcfg = FakeConfig(pp)\n\
         pheater = FakeHeater('extruder'); pheater.stats(200.0); pheater.target_temp = 200.0\n\
         pheaters = FakeHeaters(pheater); pheaters.samples = {samples_py}\n\
         pth = FakeToolhead(); pcf = FakeConfigFile(); pgcode = FakeGCode()\n\
         pp.add_object('heaters', pheaters); pp.add_object('toolhead', pth)\n\
         pp.add_object('configfile', pcf); pp.add_object('gcode', pgcode)\n\
         ptune = pid_calibrate.ControlAutoTune(pheater, 200.0)"
    ))
    .expect("py world");
}

/// The fixed alternating sample schedule used by the end-to-end command
/// tests: twelve target crossings, ending with the heater off.
fn e2e_samples() -> Vec<(f64, f64)> {
    (0..12)
        .map(|i| {
            let temp = if i % 2 == 0 { 190. } else { 210. };
            (i as f64 * 10., temp)
        })
        .collect()
}

/// Drive `ptune` over the samples with `pheater.target_temp` as the live target.
fn python_drive(py: &mut pyoracle::Oracle, samples: &[(f64, f64)]) {
    for (t, v) in samples {
        py.exec(&format!(
            "ptune.temperature_update({t:?}, {v:?}, pheater.target_temp)"
        ))
        .expect("py temperature_update");
    }
}

/// Drive the Rust tuner over the samples with the heater target live.
fn rust_drive(tune: &ControlAutoTune, heater: &Heater, samples: &[(f64, f64)]) {
    for (t, v) in samples {
        tune.temperature_update(*t, *v, heater.target_temp())
            .expect("rust temperature_update");
    }
}

/// Compare tuner state after a drive.
fn check_tune_state(
    label: &str,
    py: &mut pyoracle::Oracle,
    tune: &ControlAutoTune,
    heater: &Heater,
    pwm: &FakePwm,
) {
    check(
        &format!("{label} heating"),
        py.eval("ptune.heating").expect("eval"),
        Value::Bool(tune.is_heating()),
    );
    check(
        &format!("{label} peak"),
        py.eval("[ptune.peak, ptune.peak_time]").expect("eval"),
        Value::List(vec![
            Value::float(tune.peak().0),
            Value::float(tune.peak().1),
        ]),
    );
    check(
        &format!("{label} peaks"),
        py.eval("ptune.peaks").expect("eval"),
        samples_value(&tune.peaks()),
    );
    check(
        &format!("{label} pwm samples"),
        py.eval("ptune.pwm_samples").expect("eval"),
        samples_value(&tune.pwm_samples()),
    );
    check(
        &format!("{label} temp samples"),
        py.eval("ptune.temp_samples").expect("eval"),
        samples_value(&tune.temp_samples()),
    );
    check(
        &format!("{label} last pwm"),
        py.eval("ptune.last_pwm").expect("eval"),
        Value::float(tune.last_pwm()),
    );
    check(
        &format!("{label} mcu pwm calls"),
        py.eval("pheater.mcu_pwm.calls").expect("eval"),
        pwm.calls_value(),
    );
    check(
        &format!("{label} heater target"),
        py.eval("pheater.target_temp").expect("eval"),
        Value::float(heater.target_temp()),
    );
}

fn tune_heater(world: &World) -> Arc<dyn HeaterApi> {
    world.heater.clone()
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn tune_pid_delta_matches_python() {
    let Some(mut py) = oracle() else { return };
    check(
        "TUNE_PID_DELTA",
        py.eval("pid_calibrate.TUNE_PID_DELTA").expect("eval"),
        Value::float(TUNE_PID_DELTA),
    );
}

#[test]
fn load_config_registers_command() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    py.exec(
        "pp = FakePrinter(); pcfg = FakeConfig(pp); pgcode = FakeGCode()\
         \npp.add_object('gcode', pgcode)\
         \nppid = pid_calibrate.PIDCalibrate(pcfg)",
    )
    .expect("py load");
    check(
        "registered command",
        py.eval("pgcode.registered").expect("eval"),
        Value::List(vec![Value::Tuple(vec![
            Value::str("PID_CALIBRATE"),
            Value::str(CMD_PID_CALIBRATE_HELP),
        ])]),
    );
    let world = build_world(Vec::new());
    let factory = module_factory();
    let load = factory.load_config.expect("load_config entry point");
    let obj = load(&world.config).expect("rust load_config");
    assert!(obj.is_some());
    check(
        "rust registered command",
        Value::List(vec![Value::Tuple(vec![
            Value::str("PID_CALIBRATE"),
            Value::str(CMD_PID_CALIBRATE_HELP),
        ])]),
        world.gcode.registered_value(),
    );
}

#[test]
fn initial_state_matches_python() {
    let Some(mut py) = oracle() else { return };
    let world = direct_world();
    let samples: Vec<(f64, f64)> = Vec::new();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    check(
        "heater_max_power",
        py.eval("ptune.heater_max_power").expect("eval"),
        Value::float(tune.heater_max_power()),
    );
    check(
        "calibrate_temp",
        py.eval("ptune.calibrate_temp").expect("eval"),
        Value::float(tune.calibrate_temp()),
    );
    check_tune_state("initial", &mut py, &tune, &world.heater, &world.pwm);
}

#[test]
fn set_pwm_records_only_changes() {
    let Some(mut py) = oracle() else { return };
    let world = direct_world();
    let samples: Vec<(f64, f64)> = Vec::new();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    let steps = [(1., 0.5), (2., 0.5), (3., 0.), (4., 0.), (5., 1.)];
    for (t, v) in steps {
        py.exec(&format!("ptune.set_pwm({t:?}, {v:?})")).expect("py set_pwm");
        tune.set_pwm(t, v);
    }
    check_tune_state("after set_pwm", &mut py, &tune, &world.heater, &world.pwm);
}

#[test]
fn temperature_update_sequence_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples = e2e_samples();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    for (i, (t, v)) in samples.iter().enumerate() {
        py.exec(&format!("ptune.temperature_update({t:?}, {v:?}, pheater.target_temp)"))
            .expect("py temperature_update");
        tune.temperature_update(*t, *v, world.heater.target_temp())
            .expect("rust temperature_update");
        let label = format!("step {i}");
        check(
            &format!("{label} heating"),
            py.eval("ptune.heating").expect("eval"),
            Value::Bool(tune.is_heating()),
        );
        check(
            &format!("{label} peaks"),
            py.eval("ptune.peaks").expect("eval"),
            samples_value(&tune.peaks()),
        );
        check(
            &format!("{label} peak"),
            py.eval("[ptune.peak, ptune.peak_time]").expect("eval"),
            Value::List(vec![
                Value::float(tune.peak().0),
                Value::float(tune.peak().1),
            ]),
        );
    }
    check_tune_state("after sequence", &mut py, &tune, &world.heater, &world.pwm);
}

#[test]
fn check_busy_threshold_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples = e2e_samples();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    // Before any peaks: busy, and the arguments do not matter.
    check(
        "busy before peaks",
        py.eval("ptune.check_busy(0., 0., 0.)").expect("eval"),
        Value::Bool(tune.check_busy(0., 0., 0.)),
    );
    rust_drive(&tune, &world.heater, &samples[..4]);
    python_drive(&mut py, &samples[..4]);
    check(
        "busy after 4 peaks",
        py.eval("ptune.check_busy(0., 0., 0.)").expect("eval"),
        Value::Bool(tune.check_busy(0., 0., 0.)),
    );
    rust_drive(&tune, &world.heater, &samples[4..]);
    python_drive(&mut py, &samples[4..]);
    check(
        "busy after 12 peaks",
        py.eval("ptune.check_busy(0., 0., 0.)").expect("eval"),
        Value::Bool(tune.check_busy(0., 0., 0.)),
    );
    check(
        "busy ignores args",
        py.eval("ptune.check_busy(9., 9., 9.)").expect("eval"),
        Value::Bool(tune.check_busy(9., 9., 9.)),
    );
}

#[test]
fn calc_pid_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples = e2e_samples();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    rust_drive(&tune, &world.heater, &samples);
    python_drive(&mut py, &samples);
    check(
        "calc_pid(5)",
        py.eval("ptune.calc_pid(5)").expect("eval"),
        pid_result_value(tune.calc_pid(5)),
    );
    check(
        "calc_pid(-1) wraps",
        py.eval("ptune.calc_pid(-1)").expect("eval"),
        pid_result_value(tune.calc_pid(-1)),
    );
    check(
        "calc_pid(0) wraps",
        py.eval("ptune.calc_pid(0)").expect("eval"),
        pid_result_value(tune.calc_pid(0)),
    );
    check(
        "calc_pid logs",
        py.eval("log_messages").expect("eval"),
        take_logs(),
    );
}

#[test]
fn calc_pid_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples = e2e_samples();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    rust_drive(&tune, &world.heater, &samples);
    python_drive(&mut py, &samples);
    check_err(
        "calc_pid(100)",
        py.eval("ptune.calc_pid(100)"),
        tune.calc_pid(100).map(|_| ()),
    );
    check_err(
        "calc_pid(-100)",
        py.eval("ptune.calc_pid(-100)"),
        tune.calc_pid(-100).map(|_| ()),
    );
}

/// Two consecutive peaks with the same temperature: the amplitude is zero
/// and Python raises `ZeroDivisionError` from the `Ku` division.
#[test]
fn calc_pid_zero_amplitude_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples: Vec<(f64, f64)> = (0..4).map(|i| (i as f64 * 10., 200.)).collect();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    // Drive with a constant target of 200.0, not the live heater target:
    // `alter_target` moves the live target to 195 after the second
    // crossing, so a live-target drive with all-200 samples records only
    // two peaks and never reaches `calc_pid`. With a constant target
    // every sample crosses and the fourth records a fourth peak whose
    // temperature equals the third, so the amplitude is zero.
    for (t, v) in &samples[..3] {
        py.exec(&format!(
            "ptune.temperature_update({t:?}, {v:?}, 200.0)"
        ))
        .expect("py temperature_update");
        tune.temperature_update(*t, *v, 200.)
            .expect("rust temperature_update");
    }
    // The fourth crossing records a fourth peak and raises in `calc_pid`.
    let py_fail = py.eval("ptune.temperature_update(30.0, 200.0, 200.0)");
    let rust_fail = tune.temperature_update(30., 200., 200.);
    check_err("zero amplitude temperature_update", py_fail, rust_fail.map(|_| ()));
    check_tune_state(
        "after zero amplitude rise",
        &mut py,
        &tune,
        &world.heater,
        &world.pwm,
    );
    check_err(
        "zero amplitude calc_pid",
        py.eval("ptune.calc_pid(3)"),
        tune.calc_pid(3).map(|_| ()),
    );
}

/// Equal peak times with different temperatures: `Tu` is zero and the
/// `Ki` division raises `ZeroDivisionError`.
#[test]
fn calc_pid_zero_tu_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples: Vec<(f64, f64)> = vec![
        (0., 190.),
        (0., 210.),
        (0., 190.),
    ];
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    rust_drive(&tune, &world.heater, &samples);
    python_drive(&mut py, &samples);
    let py_fail = py.eval("ptune.temperature_update(0.0, 215.0, pheater.target_temp)");
    let rust_fail = tune.temperature_update(0., 215., world.heater.target_temp());
    check_err("zero Tu temperature_update", py_fail, rust_fail.map(|_| ()));
    check_tune_state("after zero Tu rise", &mut py, &tune, &world.heater, &world.pwm);
}

#[test]
fn calc_final_pid_median_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let times = [0., 7., 30., 32., 90., 95., 120., 121., 200., 203., 260., 261.];
    let temps = [
        190., 210., 185., 215., 192., 208., 188., 212., 191., 209., 187., 211.,
    ];
    let samples: Vec<(f64, f64)> = times
        .iter()
        .zip(temps.iter())
        .map(|(t, v)| (*t, *v))
        .collect();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    rust_drive(&tune, &world.heater, &samples);
    python_drive(&mut py, &samples);
    check(
        "calc_final_pid",
        py.eval("ptune.calc_final_pid()").expect("eval"),
        pid_result_value(tune.calc_final_pid()),
    );
    // A fresh tuner with four peaks has an empty cycle list: IndexError.
    let samples4: Vec<(f64, f64)> = e2e_samples()[..4].to_vec();
    let world4 = direct_world();
    python_setup_samples(&mut py, &samples4);
    let tune4 = ControlAutoTune::new(tune_heater(&world4), 200.);
    rust_drive(&tune4, &world4.heater, &samples4);
    python_drive(&mut py, &samples4);
    check_err(
        "calc_final_pid empty",
        py.eval("ptune.calc_final_pid()"),
        tune4.calc_final_pid().map(|_| ()),
    );
}

const WRITE_PATH: &str = "/tmp/pid_calibrate_utest_heattest.txt";

#[test]
fn write_file_contents_match_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let _ = std::fs::remove_file(WRITE_PATH);
    let world = direct_world();
    let samples = e2e_samples();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    rust_drive(&tune, &world.heater, &samples);
    python_drive(&mut py, &samples);
    tune.write_file(WRITE_PATH).expect("rust write_file");
    let rust_bytes = std::fs::read(WRITE_PATH).expect("rust read back");
    py.exec(&format!("ptune.write_file('{WRITE_PATH}')"))
        .expect("py write_file");
    let py_bytes = py
        .eval(&format!("open('{WRITE_PATH}', 'rb').read()"))
        .expect("py read back");
    check(
        "write_file bytes",
        py_bytes,
        Value::bytes(&rust_bytes),
    );
    // An untouched tuner writes an empty file.
    let empty_world = direct_world();
    let empty_py_samples: Vec<(f64, f64)> = Vec::new();
    python_setup_samples(&mut py, &empty_py_samples);
    let empty_tune =
        ControlAutoTune::new(tune_heater(&empty_world), 200.);
    empty_tune
        .write_file(WRITE_PATH)
        .expect("rust empty write_file");
    let rust_bytes = std::fs::read(WRITE_PATH).expect("rust read back");
    py.exec(&format!("ptune.write_file('{WRITE_PATH}')"))
        .expect("py empty write_file");
    let py_bytes = py
        .eval(&format!("open('{WRITE_PATH}', 'rb').read()"))
        .expect("py read back");
    check("empty write_file bytes", py_bytes, Value::bytes(&rust_bytes));
}

#[test]
fn write_file_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let world = direct_world();
    let samples: Vec<(f64, f64)> = Vec::new();
    python_setup_samples(&mut py, &samples);
    let tune = ControlAutoTune::new(tune_heater(&world), 200.);
    let missing = "/tmp/pid_calibrate_no_such_dir/heattest.txt";
    check_err(
        "write_file missing directory",
        py.eval(&format!("ptune.write_file('{missing}')")),
        tune.write_file(missing).map(|_| ()),
    );
    check_err(
        "write_file is a directory",
        py.eval("ptune.write_file('/tmp')"),
        tune.write_file("/tmp").map(|_| ()),
    );
}

fn gcmd_params(pairs: &[(&str, &str)]) -> (IndexMap<String, String>, String) {
    let mut params = IndexMap::new();
    for (k, v) in pairs {
        params.insert((*k).to_string(), (*v).to_string());
    }
    let commandline = format!("PID_CALIBRATE {}", pairs
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" "));
    (params, commandline)
}

fn python_cmd(
    py: &mut pyoracle::Oracle,
    pairs: &[(&str, &str)],
) -> pyoracle::OracleResult {
    let (params, commandline) = gcmd_params(pairs);
    let items: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("('{k}', '{v}')"))
        .collect();
    py.exec(&format!(
        "pgcmd = FakeGCmd(dict([{input}]), '{commandline}')\n\
         ppid = pid_calibrate.PIDCalibrate(pcfg)",
        input = items.join(", "),
        commandline = commandline,
    ))
    .expect("py cmd setup");
    py.eval("ppid.cmd_PID_CALIBRATE(pgcmd)")
}

fn python_cmd_world(py: &mut pyoracle::Oracle, samples: &[(f64, f64)]) {
    let items: Vec<String> = samples
        .iter()
        .map(|(t, v)| format!("({t:?}, {v:?})"))
        .collect();
    let samples_py = format!("[{}]", items.join(", "));
    py.exec(&format!(
        "pp = FakePrinter(); pcfg = FakeConfig(pp)\n\
         pheater = FakeHeater('extruder'); pheater.stats(200.0)\n\
         pheaters = FakeHeaters(pheater); pheaters.samples = {samples_py}\n\
         pth = FakeToolhead(); pcf = FakeConfigFile(); pgcode = FakeGCode()\n\
         pp.add_object('heaters', pheaters); pp.add_object('toolhead', pth)\n\
         pp.add_object('configfile', pcf); pp.add_object('gcode', pgcode)",
        samples_py = samples_py,
    ))
    .expect("py cmd world");
}

#[test]
fn cmd_end_to_end_success_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let pairs = [("HEATER", "extruder"), ("TARGET", "200")];
    let expected = python_cmd(&mut py, &pairs);
    let world = build_world(samples);
    let (params, commandline) = gcmd_params(&pairs);
    let raw = Arc::new(FakeGCmd::new(params, commandline));
    let gcmd: GCmd = raw.clone();
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    match (&expected, &actual) {
        (Ok(v), Ok(())) => check("cmd end-to-end ok", v.clone(), Value::Null),
        (Ok(_), Err(e)) => panic!("cmd rust raised {}: {}", e.class_name(), e.message()),
        (Err(e), Err(_)) => panic!("oracle raised {}: {}", e.class_name, e.message),
        (Err(_), Ok(())) => panic!("rust returned but oracle raised"),
    }
    check(
        "mcu pwm calls",
        py.eval("pheater.mcu_pwm.calls").expect("eval"),
        world.pwm.calls_value(),
    );
    check(
        "heater target after restore",
        py.eval("pheater.target_temp").expect("eval"),
        Value::float(world.heater.target_temp()),
    );
    check(
        "heaters calls",
        py.eval("pheaters.calls").expect("eval"),
        world.heaters.calls_value(),
    );
    check(
        "responded",
        py.eval("pgcmd.responded").expect("eval"),
        raw.responded_value(),
    );
    check(
        "configfile calls",
        py.eval("pcf.calls").expect("eval"),
        Value::List(world.configfile.calls.borrow().clone()),
    );
    check(
        "toolhead calls",
        py.eval("pth.calls").expect("eval"),
        Value::int(world.toolhead.calls()),
    );
    check(
        "log messages",
        py.eval("log_messages").expect("eval"),
        take_logs(),
    );
}

#[test]
fn cmd_unknown_heater_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [("HEATER", "missing"), ("TARGET", "200")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let raw = Arc::new(FakeGCmd::new(params, commandline));
    let gcmd: GCmd = raw.clone();
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("unknown heater", expected, actual);
    check(
        "heater untouched",
        py.eval("pheater.mcu_pwm.calls").expect("eval"),
        world.pwm.calls_value(),
    );
    check(
        "responded empty",
        py.eval("pgcmd.responded").expect("eval"),
        raw.responded_value(),
    );
    check(
        "toolhead untouched",
        py.eval("pth.calls").expect("eval"),
        Value::int(world.toolhead.calls()),
    );
}

#[test]
fn cmd_missing_and_bad_params_match_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    // Missing HEATER (read first).
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [("TARGET", "200")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let gcmd: GCmd = Arc::new(FakeGCmd::new(params, commandline));
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("missing HEATER", expected, actual);
    // Missing TARGET.
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [("HEATER", "extruder")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let gcmd: GCmd = Arc::new(FakeGCmd::new(params, commandline));
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("missing TARGET", expected, actual);
    // Unparseable TARGET.
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [("HEATER", "extruder"), ("TARGET", "hot")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let gcmd: GCmd = Arc::new(FakeGCmd::new(params, commandline));
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("bad TARGET", expected, actual);
    // Unparseable WRITE_FILE.
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [
        ("HEATER", "extruder"),
        ("TARGET", "200"),
        ("WRITE_FILE", "yes"),
    ];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let gcmd: GCmd = Arc::new(FakeGCmd::new(params, commandline));
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("bad WRITE_FILE", expected, actual);
}

#[test]
fn cmd_set_temperature_error_restores_control() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    py.exec("pheaters.fail = 'command'").expect("py fail");
    let world = build_world(samples);
    world.heaters.set_fail(Some("command"));
    let pairs = [("HEATER", "extruder"), ("TARGET", "200")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let raw = Arc::new(FakeGCmd::new(params, commandline));
    let gcmd: GCmd = raw.clone();
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("set_temperature command error", expected, actual);
    // The restore leaves the heater with a zero target again.
    check(
        "target after error",
        py.eval("pheater.target_temp").expect("eval"),
        Value::float(world.heater.target_temp()),
    );
    // A temperature callback after the error reaches the restored
    // control, which never alters the target (the tuner would).
    py.exec("pheater.temperature_callback(0.0, -250.0)").expect("py callback");
    world.heater.temperature_callback(0., -250.);
    check(
        "target after restored-control callback",
        py.eval("pheater.target_temp").expect("eval"),
        Value::float(world.heater.target_temp()),
    );
    check(
        "mcu pwm calls",
        py.eval("pheater.mcu_pwm.calls").expect("eval"),
        world.pwm.calls_value(),
    );
    check(
        "responded empty",
        py.eval("pgcmd.responded").expect("eval"),
        raw.responded_value(),
    );
    check(
        "configfile empty",
        py.eval("pcf.calls").expect("eval"),
        Value::List(world.configfile.calls.borrow().clone()),
    );
}

#[test]
fn cmd_interrupted_when_busy_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    // Only four crossings: twelve peaks are never reached.
    let samples = e2e_samples()[..4].to_vec();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [("HEATER", "extruder"), ("TARGET", "200")];
    let expected = python_cmd(&mut py, &pairs);
    let (params, commandline) = gcmd_params(&pairs);
    let gcmd: GCmd = Arc::new(FakeGCmd::new(params, commandline));
    let pid = PIDCalibrate::new(&world.config);
    let actual = pid.cmd_pid_calibrate(&gcmd);
    check_err("interrupted", expected, actual);
    check(
        "target after interrupted run",
        py.eval("pheater.target_temp").expect("eval"),
        Value::float(world.heater.target_temp()),
    );
    check(
        "log messages",
        py.eval("log_messages").expect("eval"),
        take_logs(),
    );
}

#[test]
fn cmd_write_file_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_serial();
    let path = "/tmp/heattest.txt";
    let _ = std::fs::remove_file(path);
    let samples = e2e_samples();
    python_cmd_world(&mut py, &samples);
    let world = build_world(samples);
    let pairs = [
        ("HEATER", "extruder"),
        ("TARGET", "200"),
        ("WRITE_FILE", "1"),
    ];
    let expected = python_cmd(&mut py, &pairs);
    match &expected {
        Ok(_) => {}
        Err(e) => panic!("oracle raised {}: {}", e.class_name, e.message),
    }
    let rust_bytes = std::fs::read(path).expect("rust read back");
    let py_bytes = py
        .eval(&format!("open('{path}', 'rb').read()"))
        .expect("py read back");
    let _ = world;
    check("cmd write_file bytes", py_bytes, Value::bytes(&rust_bytes));
    let _ = std::fs::remove_file(path);
}
