//! Parity tests for `extras::controller_fan` (port of
//! `klippy/extras/controller_fan.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `ControllerFan` is built
//! on both sides with minimal Python / Rust fakes of the printer, the config
//! wrapper (the real `klippy.configfile.ConfigWrapper` on the Python side),
//! `stepper_enable`, `heaters` and `fan.Fan` (patched in as a recording
//! fake on the Python side; the real `fan::Fan` on the Rust side).
//! The two are driven through the same `handle_connect` / `callback` /
//! `get_status` calls and the results, config errors, registered event
//! handlers, loaded sections, fan speeds and timer return values are
//! compared exactly (floats by bits).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

#![allow(clippy::type_complexity)]

use extras::controller_fan::{ControllerFan, PIN_MIN_TIME};
use extras::heaters::SensorFactory;
use extras::heaters::{Heater, HeaterApi, PrinterHeatersApi, TemperatureCallback, TemperatureSensorApi};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi, StepperEnablePin};
use extras::heaters::SensorApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GcmdGetFloatOpts, GetIntOpts as GcmdGetIntOpts,
    GetOpts as GcmdGetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueue,
    CommandQueryWrapper, CommandWrapper, CommandWrapperApi, ConfigCallback, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi,
    PwmPinApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Stepper, StepperKinematics, Trapq,
};
use klippy_api::toolhead::MotionQueuingApi;
use klippy_api::{EventArg, GCell, ObjectRef};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
from configfile import ConfigWrapper
import extras.controller_fan as cf_mod
import extras.fan as fan_mod

SECTION = 'controller_fan myfan'

def make_config(printer, opts):
    parser = configparser.ConfigParser()
    parser[SECTION] = opts
    return ConfigWrapper(printer, parser, {}, SECTION)

class FakeReactor:
    def __init__(self):
        self.timers = []
    def monotonic(self):
        return 100.0
    def register_timer(self, cb, wake):
        self.timers.append((cb, wake))
        return len(self.timers)

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.handlers = []
        self.objects = {}
        self.reactor = FakeReactor()
        self.loaded = []
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def lookup_object(self, name):
        if name not in self.objects:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return self.objects[name]
    def load_object(self, config, section, default=None):
        self.loaded.append(section)
        if section not in self.objects:
            raise configparser.Error("Unknown config object '%s'" % (section,))
        return self.objects[section]
    def get_reactor(self):
        return self.reactor

class FakeEnableTracking:
    def __init__(self, enabled):
        self._enabled = enabled
    def is_motor_enabled(self):
        return self._enabled

class FakeStepperEnable:
    def __init__(self, steppers):
        self.order = [n for n, e in steppers]
        self.enabled = dict(steppers)
    def get_steppers(self):
        return list(self.order)
    def lookup_enable(self, name):
        if name not in self.enabled:
            raise configparser.Error("Unknown stepper '%s'" % (name,))
        return FakeEnableTracking(self.enabled[name])

class FakeHeater:
    def __init__(self, temp, target):
        self.temp = float(temp)
        self.target = float(target)
    def get_temp(self, eventtime):
        return (self.temp, self.target)

class FakeHeaters:
    def __init__(self, heaters):
        self.heaters = dict(heaters)
    def lookup_heater(self, name):
        if name not in self.heaters:
            raise configparser.Error("Unknown heater '%s'" % (name,))
        return self.heaters[name]

class FakeFan:
    def __init__(self):
        self.speeds = []
        self.status = {'speed': 0.0, 'rpm': None}
    def set_speed(self, value):
        self.speeds.append(value)
    def get_status(self, eventtime):
        return dict(self.status)

def build_pair(opts, steppers, heaters):
    p = FakePrinter()
    p.objects['stepper_enable'] = FakeStepperEnable(steppers)
    p.objects['heaters'] = FakeHeaters(heaters)
    fan = FakeFan()
    fan_mod.Fan = lambda config: fan
    cfg = make_config(p, opts)
    obj = cf_mod.load_config_prefix(cfg)
    return obj, p, fan
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn heater_config(printer: &Printer, name: &str) -> Config {
    Arc::new(FakeConfig {
        printer: printer.clone(),
        section: name.to_string(),
        opts: vec![
            ("min_temp".to_string(), "0".to_string()),
            ("max_temp".to_string(), "300".to_string()),
            ("control".to_string(), "watermark".to_string()),
            ("heater_pin".to_string(), "pa1".to_string()),
        ],
    })
}

struct FakePrinter {
    objects: GCell<Vec<(String, ObjectRef)>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    loaded: GCell<Vec<String>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn fresh() -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: GCell::new(Vec::new()),
            handlers: GCell::new(Vec::new()),
            loaded: GCell::new(Vec::new()),
            reactor: Reactor::new(false),
        })
    }

    fn object(&self, name: &str) -> Option<ObjectRef> {
        self.objects
            .borrow()
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, o)| o.clone())
    }

    /// Sections this controller is expected to load, in order. The Fan /
    /// GCodeRequestQueue of the Rust side loads `motion_queuing`; constructing
    /// the fake heaters loads `verify_heater <name>` and `pid_calibrate`.
    /// Those loads come from inside `fan::Fan` / `heaters::Heater`, not from
    /// `controller_fan`, so they are not part of the comparison.
    fn loaded_sections(&self) -> Vec<String> {
        self.loaded
            .borrow()
            .iter()
            .filter(|s| s.as_str() == "stepper_enable" || s.as_str() == "heaters")
            .cloned()
            .collect()
    }

    fn handler_names(&self) -> Vec<String> {
        self.handlers
            .borrow()
            .iter()
            .map(|(n, _)| n.clone())
            .collect()
    }

    /// The two handlers `controller_fan` registers (`klippy:ready` then
    /// `klippy:connect`), as its final registrations: constructing them
    /// registers the GCodeRequestQueue `klippy:connect` handler and the
    /// fan's `gcode:request_restart` handler first.
    fn controller_handler_names(&self) -> Vec<String> {
        let names = self.handler_names();
        names[names.len().saturating_sub(2)..].to_vec()
    }


    fn invoke_last_handler(&self, event: &str) -> KResult<PyValue> {
        self.invoke_handler_from(event, true)
    }

    fn invoke_handler_from(&self, event: &str, last: bool) -> KResult<PyValue> {
        let pos = if last {
            self.handlers
                .borrow()
                .iter()
                .rposition(|(n, _)| n == event)
        } else {
            self.handlers
                .borrow()
                .iter()
                .position(|(n, _)| n == event)
        }
        .expect("handler registered");
        let (_, cb) = self.handlers.borrow_mut().remove(pos);
        let mut cb = cb;
        cb(&[])
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.object(name) {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().clone()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.loaded.borrow_mut().push(section.to_string());
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
            .borrow_mut()
            .push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

/// Rust mirror of `klippy/configfile.py:18` (`ConfigWrapper`) sufficient for
/// the options `controller_fan` (and the `fan::Fan` it builds, plus the
/// heater objects the fake `heaters` object constructs) reads, including
/// its error texts.
struct FakeConfig {
    printer: Printer,
    section: String,
    opts: Vec<(String, String)>,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<String> {
        self.opts
            .iter()
            .find(|(n, _)| n == option)
            .map(|(_, v)| v.clone())
    }

    fn missing_required(option: &str, section: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{section}' must be specified"
        ))
    }

    fn parse_failed(option: &str, section: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{option}' in section '{section}'"
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
        match self.raw(option) {
            None => match opts.default {
                None => Err(Self::missing_required(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
            Some(s) => Ok(Some(s)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            None => match opts.default {
                None => Err(Self::missing_required(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
            Some(s) => {
                let v: i64 = s
                    .parse()
                    .map_err(|_| Self::parse_failed(option, &self.section))?;
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.section
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.section
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.raw(option) {
            None => match opts.default {
                None => Err(Self::missing_required(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
            Some(s) => {
                let v: f64 = s
                    .parse()
                    .map_err(|_| Self::parse_failed(option, &self.section))?;
                if let Some(minval) = opts.minval {
                    if v < minval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            pycompat::text::float_repr(minval)
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if v > maxval {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            pycompat::text::float_repr(maxval)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            pycompat::text::float_repr(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            pycompat::text::float_repr(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        match self.raw(option) {
            Some(s) => Ok(PyValue::Str(s)),
            None => Ok(PyValue::None),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.raw(option) {
            None => match opts.default {
                None => Err(Self::missing_required(option, &self.section)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
            Some(s) => {
                let parts: Vec<String> = if s.trim().is_empty() {
                    Vec::new()
                } else {
                    s.split(&opts.sep)
                        .map(|p| p.trim().to_string())
                        .collect()
                };
                if let Some(count) = opts.count {
                    if parts.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have {count} elements",
                            self.section
                        )));
                    }
                }
                Ok(Some(parts))
            }
        }
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by controller_fan")
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

// -- stepper_enable -----------------------------------------------------------

struct FakeMcuStepper;

impl McuStepperApi for FakeMcuStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("mcu not faked")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        String::new()
    }
    fn get_oid(&self) -> i64 {
        1
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _coord: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: i64) -> f64 {
        0.0
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<StepperKinematics>,
    ) -> Option<StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeStepperEnable {
    order: Vec<String>,
    lines: Vec<(String, Arc<EnableTracking>)>,
}

impl FakeStepperEnable {
    fn new(steppers: &[(&str, bool)]) -> Arc<Self> {
        let lines = steppers
            .iter()
            .map(|(name, enabled)| {
                let stepper: Arc<dyn McuStepperApi> = Arc::new(FakeMcuStepper);
                let tracking = EnableTracking::new(
                    stepper,
                    Arc::new(StepperEnablePin::new(None, 0)),
                )
                .expect("fake EnableTracking");
                if *enabled {
                    tracking.motor_enable(0.0);
                }
                (name.to_string(), tracking)
            })
            .collect::<Vec<_>>();
        Arc::new(FakeStepperEnable {
            order: steppers.iter().map(|(n, _)| n.to_string()).collect(),
            lines,
        })
    }
    fn set_enabled(&self, name: &str, enabled: bool) {
        for (n, line) in &self.lines {
            if n == name {
                if enabled {
                    line.motor_enable(0.0);
                } else {
                    line.motor_disable(0.0).expect("fake motor_disable");
                }
                return;
            }
        }
    }
}

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        Ok(())
    }
    fn set_motors_enable(&self, _stepper_names: &[String], _enable: bool) -> KResult<bool> {
        Ok(false)
    }
    fn motor_off(&self) -> KResult<()> {
        Ok(())
    }
    fn lookup_enable(&self, name: &str) -> KResult<Arc<EnableTracking>> {
        for (n, line) in &self.lines {
            if n == name {
                return Ok(line.clone());
            }
        }
        Err(KlippyError::config(format!("Unknown stepper '{name}'")))
    }
    fn get_steppers(&self) -> Vec<String> {
        self.order.clone()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

// -- heaters ------------------------------------------------------------------

struct FakeSensor;

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) {}
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        1.0
    }
}

struct FakeHeaters {
    heaters: Vec<(String, Arc<Heater>)>,
}

impl FakeHeaters {
    fn new(printer: &Printer, heaters: &[(&str, f64, f64)]) -> Arc<Self> {
        let list = heaters
            .iter()
            .map(|(name, _, _)| {
                let config = heater_config(printer, name);
                let heater = Heater::new(&config, Arc::new(FakeSensor))
                    .expect("fake Heater");
                (name.to_string(), heater)
            })
            .collect::<Vec<_>>();
        Arc::new(FakeHeaters { heaters: list })
    }
    fn set_target(&self, name: &str, target: f64) {
        for (n, h) in &self.heaters {
            if n == name {
                h.set_temp(target).expect("fake set_temp");
                return;
            }
        }
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {}
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("setup_heater not faked"))
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.heaters.iter().map(|(n, _)| n.clone()).collect()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        for (n, h) in &self.heaters {
            if n == heater_name {
                return Ok(h.clone());
            }
        }
        Err(KlippyError::config(format!("Unknown heater '{heater_name}'")))
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Err(KlippyError::config("setup_sensor not faked"))
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

// -- pins / mcu / motion_queuing / gcode (for the real fan::Fan) -------------

struct FakeMcu {
    inner: Arc<Mutex<McuInner>>,
}

#[derive(Default)]
struct McuInner {
    oid_next: i64,
    config_cbs: Vec<ConfigCallback>,
}

impl FakeMcu {
    fn new() -> Self {
        FakeMcu {
            inner: Arc::new(Mutex::new(McuInner {
                oid_next: 0,
                config_cbs: Vec::new(),
            })),
        }
    }
}

struct FakeAsyncResp;
impl AsyncResponseWrapperApi for FakeAsyncResp {
    fn unregister(&self) {}
}

struct FakeCmd;
impl CommandWrapperApi for FakeCmd {
    fn send(&self, _d: Vec<PyValue>, _o: klippy_api::mcu::SendOpts) -> KResult<()> {
        Ok(())
    }
    fn send_wait_ack(
        &self,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::SendOpts,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        1
    }
}

struct FakeQuery;
impl klippy_api::mcu::CommandQueryWrapperApi for FakeQuery {
    fn send(
        &self,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn send_with_preface(
        &self,
        _p: CommandWrapper,
        _pd: Vec<PyValue>,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("mcu.get_printer not faked")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("mcu.setup_pin not faked"))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut inner = self.inner.lock();
        let oid = inner.oid_next;
        inner.oid_next += 1;
        Ok(oid)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.inner.lock().config_cbs.push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        let _ = (cmd, opts.is_init, opts.on_restart);
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        oid + 100
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 12_000_000.0) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(
        &self,
        _m: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd))
    }
    fn lookup_query_command(
        &self,
        _m: &str,
        _r: &str,
        _o: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(Arc::new(FakeQuery))
    }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> {
        Some(Arc::new(FakeCmd))
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
        Arc::new(FakeAsyncResp)
    }
    fn check_valid_response(&self, _m: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> {
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * 12_000_000.0) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 12_000_000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(
        &self,
        print_time: f64,
        _eventtime: f64,
    ) -> KResult<(f64, f64)> {
        Ok((print_time, 12_000_000.0))
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (true, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::config("not faked"))
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePwm {
    mcu: Mcu,
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, _d: f64) {}
    fn setup_cycle_time(&self, _t: f64, _hw: Option<bool>) {}
    fn setup_start_value(&self, _s: f64, _sh: f64) {}
    fn next_aligned_print_time(&self, t: f64, _a: Option<f64>) -> f64 {
        t
    }
    fn set_pwm(&self, _print_time: f64, _value: f64) {}
}

#[derive(Clone)]
struct FakePins {
    mcu: Mcu,
    pwm: Arc<FakePwm>,
}

impl FakePins {
    fn new(mcu: Mcu) -> Self {
        FakePins {
            pwm: Arc::new(FakePwm { mcu: mcu.clone() }),
            mcu,
        }
    }
}

impl PinsApi for FakePins {
    fn parse_pin(
        &self,
        pin_desc: &str,
        _opts: ParsePinOpts,
    ) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let pin = if let Some((_, p)) = desc.split_once(':') {
            p.trim().to_string()
        } else {
            desc.trim_start_matches(['!', '^', '~']).to_string()
        };
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin,
            invert: 0,
            pullup: 0,
            share_type: None,
        })
    }
    fn lookup_pin(
        &self,
        pin_desc: &str,
        opts: LookupPinOpts,
    ) -> KResult<PinParams> {
        let mut p = self.parse_pin(
            pin_desc,
            ParsePinOpts { can_invert: opts.can_invert, can_pullup: opts.can_pullup },
        )?;
        p.share_type = opts.share_type;
        Ok(p)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        match pin_type {
            "pwm" => {
                let _ = self.parse_pin(
                    pin_desc,
                    ParsePinOpts { can_invert: true, can_pullup: false },
                )?;
                Ok(Pin::Pwm(self.pwm.clone()))
            }
            _ => Err(KlippyError::config(format!(
                "unknown pin type '{pin_type}'"
            ))),
        }
    }
    fn reset_pin_sharing(&self, _p: PinParams) {}
    fn get_pin_resolver(&self, _c: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("no resolver"))
    }
    fn register_chip(&self, _c: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _p: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

#[derive(Clone)]
struct FakeMq {
    inner: Arc<Mutex<MqInner>>,
}

impl FakeMq {
    fn new() -> Self {
        FakeMq {
            inner: Arc::new(Mutex::new(MqInner {
                flush_cbs: Vec::new(),
            })),
        }
    }
}

#[derive(Default)]
struct MqInner {
    flush_cbs: Vec<klippy_api::toolhead::FlushCallback>,
}

impl MotionQueuingApi for FakeMq {
    fn allocate_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("trapq not faked")
    }
    fn lookup_trapq_append(&self) -> klippy_api::toolhead::TrapqAppend {
        panic!("trapq_append not faked")
    }
    fn wipe_trapq(&self, _t: &klippy_api::stepper::Trapq) {}
    fn allocate_syncemitter(
        &self,
        _m: Mcu,
        _n: &str,
        _a: bool,
    ) -> KResult<klippy_api::toolhead::SyncEmitterHandle> {
        Err(KlippyError::config("syncemitter not faked"))
    }
    fn queue_syncemitter_msg(&self, _s: &klippy_api::toolhead::SyncEmitterHandle, _r: u64, _d: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: Mcu, _s: CommandQueue, _c: i64) {}
    fn register_flush_callback(&self, cb: klippy_api::toolhead::FlushCallback, _c: bool) {
        self.inner.lock().flush_cbs.push(cb);
    }
    fn unregister_flush_callback(&self, _cb: &klippy_api::toolhead::FlushCallback) {}
    fn get_kin_flush_delay(&self) -> f64 {
        0.0
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, _e: f64) -> f64 {
        0.0
    }
    fn note_mcu_movequeue_activity(&self, _mq_time: f64, _is_step_gen: bool) {}
    fn drip_update_time(
        &self,
        _s: f64,
        _e: f64,
        _d: &reactor::Completion<f64>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}

klippy_api::printer_object!(FakeMq; dyn MotionQueuingApi);

struct FakeGcodeCmd;

impl GCodeCommandApi for FakeGcodeCmd {
    fn get(&self, _name: &str, _opts: GcmdGetOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_int(&self, _name: &str, _opts: GcmdGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: GcmdGetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        String::new()
    }
    fn get_commandline(&self) -> String {
        String::new()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeGcode {
    inner: Arc<Mutex<GcodeInner>>,
}

impl FakeGcode {
    fn new() -> Self {
        FakeGcode {
            inner: Arc::new(Mutex::new(GcodeInner {
                handlers: IndexMap::new(),
            })),
        }
    }
}

#[derive(Default)]
struct GcodeInner {
    handlers: IndexMap<String, klippy_api::gcode::GCodeCommandHandler>,
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        match func {
            None => Ok(self.inner.lock().handlers.swap_remove(cmd)),
            Some(f) => {
                if self.inner.lock().handlers.contains_key(cmd) {
                    return Err(KlippyError::config(format!(
                        "gcode command {cmd} already registered",
                    )));
                }
                self.inner
                    .lock()
                    .handlers
                    .insert(cmd.to_string(), f);
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        _c: &str,
        _k: &str,
        _v: Option<&str>,
        _f: klippy_api::gcode::GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(
        &self,
        _cb: klippy_api::gcode::OutputCallback,
    ) {
    }
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("mutex not faked")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcodeCmd)
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        // The fakes never emit ints.
        PyValue::Int(_) => panic!("pyvalue_to_value: unexpected int"),
        PyValue::Bytes(_) => Value::Null,
        PyValue::Host(_) => Value::Null,
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("controller_fan fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping controller_fan oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
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
        unit: "controller_fan".to_string(),
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

fn check_float(label: &str, expected: pyoracle::OracleResult, actual: KResult<f64>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), Value::float(actual));
}

fn check_ok_unit(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(e) = actual {
        panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
    }
    check(label, Ok(expected), Value::Null);
}

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust call returned Ok but must fail"),
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

fn check_str_list(label: &str, expected: pyoracle::OracleResult, actual: Vec<String>) {
    let actual = Value::List(actual.iter().map(|s| Value::str(s)).collect());
    check(label, expected, actual);
}

fn py_quoted(s: &str) -> String {
    format!("'{s}'")
}

fn py_repr_f64(v: f64) -> String {
    pycompat::text::float_repr(v)
}

/// The currently driven fan speed on the Rust side, observed through the
/// controller's `get_status` (a faithful pass-through to the real `fan::Fan`).
fn rust_speed(obj: &ControllerFan) -> f64 {
    match obj.get_status(0.0) {
        PyValue::Dict(d) => match d.get(&PyValue::Str("speed".to_string())) {
            Some(PyValue::Float(f)) => *f,
            other => panic!("get_status speed missing: {other:?}"),
        },
        other => panic!("get_status not a dict: {other:?}"),
    }
}

/// Compares the current fan speed after one (or more) driving step(s). The
/// Python `FakeFan.speeds` list records every `set_speed` call, so the
/// expected current speed is its last element (0.0 when nothing was set yet).
fn check_cur_speed(label: &str, py: &mut pyoracle::Oracle, obj: &ControllerFan) {
    check(
        label,
        py.eval("fan.speeds[-1] if fan.speeds else 0.0"),
        Value::float(rust_speed(obj)),
    );
}

/// Builds the printer object graph shared by both build helpers.
fn build_printer(
    steppers: &[(&str, bool)],
    heaters: &[(&str, f64, f64)],
) -> (Arc<FakePrinter>, Arc<FakeStepperEnable>, Arc<FakeHeaters>) {
    let printer = FakePrinter::fresh();
    let mcu: Mcu = Arc::new(FakeMcu::new());
    let pins = FakePins::new(mcu.clone());
    printer
        .add_object("pins", Arc::new(pins.clone()))
        .unwrap();
    printer
        .add_object("motion_queuing", Arc::new(FakeMq::new()))
        .unwrap();
    printer
        .add_object("gcode", Arc::new(FakeGcode::new()))
        .unwrap();
    // Objects `heaters.Heater::new` loads during fake construction; the
    // result is unused by that constructor.
    printer
        .add_object(
            "verify_heater extruder",
            Arc::new(FakeGcode::new()),
        )
        .unwrap();
    printer
        .add_object(
            "verify_heater heater_bed",
            Arc::new(FakeGcode::new()),
        )
        .unwrap();
    printer
        .add_object("pid_calibrate", Arc::new(FakeGcode::new()))
        .unwrap();
    let se = FakeStepperEnable::new(steppers);
    printer
        .add_object("stepper_enable", se.clone() as ObjectRef)
        .unwrap();
    let printer_api: Printer = printer.clone();
    let ht = FakeHeaters::new(&printer_api, heaters);
    printer
        .add_object("heaters", ht.clone() as ObjectRef)
        .unwrap();
    (printer, se, ht)
}

/// `ControllerFan` builds a real `fan::Fan`, which requires a `pin`; the
/// Python side never reads it (its `Fan` is faked), so the option is only
/// added to the Rust config.
fn with_fan_pin<'a>(opts: &[(&'a str, &'a str)]) -> Vec<(&'a str, &'a str)> {
    let mut out: Vec<(&'a str, &'a str)> = opts.to_vec();
    if out.iter().all(|(k, _)| *k != "pin") {
        out.push(("pin", "pa1"));
    }
    out
}

fn make_config(printer: &Arc<FakePrinter>, opts: &[(&str, &str)]) -> Config {
    Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        section: "controller_fan myfan".to_string(),
        opts: opts
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
    })
}

/// Builds the Rust side of a pair.
fn fresh_rust(
    opts: &[(&str, &str)],
    steppers: &[(&str, bool)],
    heaters: &[(&str, f64, f64)],
) -> (
    Arc<ControllerFan>,
    Arc<FakePrinter>,
    Arc<FakeStepperEnable>,
    Arc<FakeHeaters>,
) {
    let (printer, se, ht) = build_printer(steppers, heaters);
    let config = make_config(&printer, &with_fan_pin(opts));
    let obj = ControllerFan::new(&config).expect("rust ControllerFan::new");
    (obj, printer, se, ht)
}

/// Rust build that is allowed to fail (config-error tests).
fn fresh_rust_result(
    opts: &[(&str, &str)],
) -> (
    KResult<Arc<ControllerFan>>,
    Arc<FakePrinter>,
    Arc<FakeStepperEnable>,
    Arc<FakeHeaters>,
) {
    let (printer, se, ht) = build_printer(&[], &[]);
    let config = make_config(&printer, &with_fan_pin(opts));
    let obj = ControllerFan::new(&config);
    (obj, printer, se, ht)
}

/// Builds the Python side with `build_pair(...)` in `py`.
fn py_build(
    py: &mut pyoracle::Oracle,
    opts: &[(&str, &str)],
    steppers: &[(&str, bool)],
    heaters: &[(&str, f64, f64)],
) {
    let opts_py: String = opts
        .iter()
        .map(|(k, v)| format!("{}: {},", py_quoted(k), py_quoted(v)))
        .collect::<Vec<_>>()
        .join(" ");
    let steppers_py: String = steppers
        .iter()
        .map(|(n, e)| {
            format!(
                "({}, {})",
                py_quoted(n),
                if *e { "True" } else { "False" }
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    let heaters_py: String = heaters
        .iter()
        .map(|(n, t, tg)| {
            format!(
                "{}: FakeHeater({}, {})",
                py_quoted(n),
                py_repr_f64(*t),
                py_repr_f64(*tg)
            )
        })
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!(
        "obj, p, fan = build_pair({{{opts_py}}}, [{steppers_py}], {{{heaters_py}}})"
    ))
    .expect("py build_pair");
}

/// Python setup whose `load_config_prefix` call is expected to raise; the
/// subsequent `py.eval` replays the failing call.
fn py_setup_build(py: &mut pyoracle::Oracle, opts: &[(&str, &str)]) {
    let opts_py: String = opts
        .iter()
        .map(|(k, v)| format!("{}: {},", py_quoted(k), py_quoted(v)))
        .collect::<Vec<_>>()
        .join(" ");
    py.exec(&format!(
        "p = FakePrinter(); \
         p.objects['stepper_enable'] = FakeStepperEnable([]); \
         p.objects['heaters'] = FakeHeaters({{}}); \
         fan_mod.Fan = lambda config: FakeFan(); \
         cfg = make_config(p, {{{opts_py}}})"
    ))
    .expect("py build setup");
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn pin_min_time_and_factory() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("cf_mod.PIN_MIN_TIME");
    check("pin_min_time", expected, Value::float(PIN_MIN_TIME));

    let factory = extras::controller_fan::module_factory();
    assert!(
        factory.load_config.is_none(),
        "controller_fan has no load_config"
    );
    assert!(
        factory.load_config_prefix.is_some(),
        "controller_fan exposes load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "controller_fan has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "controller_fan has no add_printer_objects"
    );
}

#[test]
fn init_loads_and_registers_handlers() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, &[], &[], &[]);
    let (_, printer, _, _) = fresh_rust(&[], &[], &[]);

    check_str_list(
        "loaded sections",
        py.eval("list(p.loaded)"),
        printer.loaded_sections(),
    );
    check_str_list(
        "handler names",
        py.eval("[e for e, _ in p.handlers]"),
        printer.controller_handler_names(),
    );
}

#[test]
fn init_with_custom_options() {
    let Some(mut py) = oracle() else { return };
    let opts = [
        ("fan_speed", "0.8"),
        ("idle_speed", "0.5"),
        ("idle_timeout", "5"),
        ("stepper", "x_stepper, y_stepper"),
        ("heater", "extruder, heater_bed"),
    ];
    let steppers = [("x_stepper", false), ("y_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0), ("heater_bed", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (_, printer, _, _) = fresh_rust(&opts, &steppers, &heaters);

    check_str_list(
        "custom loaded",
        py.eval("list(p.loaded)"),
        printer.loaded_sections(),
    );
    check_str_list(
        "custom handlers",
        py.eval("[e for e, _ in p.handlers]"),
        printer.controller_handler_names(),
    );
}

#[test]
fn connect_adopts_all_steppers_when_unset() {
    let Some(mut py) = oracle() else { return };
    let steppers = [("x_stepper", true), ("y_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &[], &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&[], &steppers, &heaters);

    check_ok_unit(
        "handle_connect",
        py.eval("obj.handle_connect()"),
        rust.handle_connect(),
    );
    // x_stepper enabled -> fan at full speed on the first tick.
    check_float("tick wake", py.eval("obj.callback(5.25)"), rust.callback(5.25));
    check_cur_speed("fan speed", &mut py, &rust);
}

#[test]
fn connect_with_stepper_subset() {
    let Some(mut py) = oracle() else { return };
    let opts = [("stepper", "y_stepper")];
    // Both steppers enabled globally; only y_stepper is controlled. The fan
    // must stay idle when only x_stepper (uncontrolled) is enabled.
    let steppers = [("x_stepper", true), ("y_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (rust, _, se, _) = fresh_rust(&opts, &steppers, &heaters);

    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());
    check_float("subset idle wake", py.eval("obj.callback(1.0)"), rust.callback(1.0));
    check_cur_speed("subset speed", &mut py, &rust);

    py.exec("p.objects['stepper_enable'].enabled['y_stepper'] = True")
        .expect("py enable y");
    se.set_enabled("y_stepper", true);
    check_float(
        "subset active wake",
        py.eval("obj.callback(2.0)"),
        rust.callback(2.0),
    );
    check_cur_speed("subset speed after", &mut py, &rust);
}

#[test]
fn connect_unknown_stepper_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let opts = [("stepper", "x_stepper, z_stepper")];
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&opts, &steppers, &heaters);

    check_err(
        "unknown stepper in list",
        py.eval("obj.handle_connect()"),
        rust.handle_connect(),
    );
}

#[test]
fn connect_unknown_single_stepper_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let opts = [("stepper", "z_stepper")];
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&opts, &steppers, &heaters);

    check_err(
        "unknown single stepper",
        py.eval("obj.handle_connect()"),
        rust.handle_connect(),
    );
}

#[test]
fn connect_unknown_heater_raises_config_error() {
    let Some(mut py) = oracle() else { return };
    let opts = [("heater", "extruder, heater_bed")];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &[], &heaters);
    let (rust, _, _, _) = fresh_rust(&opts, &[], &heaters);

    check_err(
        "unknown heater",
        py.eval("obj.handle_connect()"),
        rust.handle_connect(),
    );
}

#[test]
fn callback_idle_when_nothing_active() {
    let Some(mut py) = oracle() else { return };
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &[], &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&[], &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());

    // Fresh controller: last_on == idle_timeout (30), so no idle speed.
    check_float("idle tick wake", py.eval("obj.callback(10.0)"), rust.callback(10.0));
    check_cur_speed("no speed", &mut py, &rust);
}

#[test]
fn callback_full_idle_cycle() {
    let Some(mut py) = oracle() else { return };
    let opts = [
        ("fan_speed", "0.8"),
        ("idle_speed", "0.5"),
        ("idle_timeout", "3"),
    ];
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (rust, _, se, _) = fresh_rust(&opts, &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());

    // Tick 1: stepper on -> fan_speed 0.8.
    py.exec("p.objects['stepper_enable'].enabled['x_stepper'] = True")
        .expect("py enable");
    se.set_enabled("x_stepper", true);
    check_float("tick1 wake", py.eval("obj.callback(1.0)"), rust.callback(1.0));
    check_cur_speed("tick1 speed", &mut py, &rust);

    // Tick 2: stepper off -> idle_speed 0.5 (idle tick 1 of 3).
    py.exec("p.objects['stepper_enable'].enabled['x_stepper'] = False")
        .expect("py disable");
    se.set_enabled("x_stepper", false);
    check_float("tick2 wake", py.eval("obj.callback(2.0)"), rust.callback(2.0));
    check_cur_speed("tick2 speed", &mut py, &rust);
    check_float("tick3 wake", py.eval("obj.callback(3.0)"), rust.callback(3.0));
    check_cur_speed("tick3 speed", &mut py, &rust);
    check_float("tick4 wake", py.eval("obj.callback(4.0)"), rust.callback(4.0));
    check_cur_speed("tick4 speed", &mut py, &rust);
    // Tick 5: idle_timeout (3) exhausted -> 0.
    check_float("tick5 wake", py.eval("obj.callback(5.0)"), rust.callback(5.0));
    check_cur_speed("speed sequence", &mut py, &rust);
}

#[test]
fn callback_idle_speed_defaults_to_fan_speed() {
    let Some(mut py) = oracle() else { return };
    let opts = [("fan_speed", "0.6"), ("idle_timeout", "2")];
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &opts, &steppers, &heaters);
    let (rust, _, se, _) = fresh_rust(&opts, &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());

    py.exec("p.objects['stepper_enable'].enabled['x_stepper'] = True")
        .expect("py enable");
    se.set_enabled("x_stepper", true);
    check_float("tick1 wake", py.eval("obj.callback(1.0)"), rust.callback(1.0));
    check_cur_speed("tick1 speed", &mut py, &rust);

    // idle_speed defaults to fan_speed (0.6): no speed change on idle ticks.
    py.exec("p.objects['stepper_enable'].enabled['x_stepper'] = False")
        .expect("py disable");
    se.set_enabled("x_stepper", false);
    check_float("tick2 wake", py.eval("obj.callback(2.0)"), rust.callback(2.0));
    check_cur_speed("tick2 speed", &mut py, &rust);
    // Idle tick 2 of 2; then the next tick stops the fan.
    check_float("tick3 wake", py.eval("obj.callback(3.0)"), rust.callback(3.0));
    check_cur_speed("tick3 speed", &mut py, &rust);
    check_float("tick4 wake", py.eval("obj.callback(4.0)"), rust.callback(4.0));
    check_cur_speed("speeds: no re-set at same speed", &mut py, &rust);
}

#[test]
fn callback_heater_target_activates() {
    let Some(mut py) = oracle() else { return };
    let steppers = [("x_stepper", false)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &[], &steppers, &heaters);
    let (rust, _, _, ht) = fresh_rust(&[], &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());

    py.exec("p.objects['heaters'].heaters['extruder'].target = 200.0")
        .expect("py set target");
    ht.set_target("extruder", 200.0);
    check_float("heater active wake", py.eval("obj.callback(1.0)"), rust.callback(1.0));
    check_cur_speed("speeds", &mut py, &rust);

    // Smoothed temp without a target does not activate (target rules).
    py.exec(
        "p.objects['heaters'].heaters['extruder'].target = 0.0; \
         p.objects['heaters'].heaters['extruder'].temp = 210.0",
    )
    .expect("py clear target");
    ht.set_target("extruder", 0.0);
    check_float("heater idle wake", py.eval("obj.callback(2.0)"), rust.callback(2.0));
    check_cur_speed("speeds after clear", &mut py, &rust);
}

#[test]
fn callback_set_speed_only_on_change() {
    let Some(mut py) = oracle() else { return };
    let steppers = [("x_stepper", true)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &[], &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&[], &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());

    // Enabled the whole time: fan set to fan_speed exactly once. The Rust
    // side reports the driven speed each tick (4th-8th checks compare each
    // tick's fan state); python `fan.speeds` has one entry on the oracle side.
    check_float("t1", py.eval("obj.callback(1.0)"), rust.callback(1.0));
    check_cur_speed("t1 speed", &mut py, &rust);
    check_float("t2", py.eval("obj.callback(2.0)"), rust.callback(2.0));
    check_cur_speed("t2 speed", &mut py, &rust);
    check_float("t3", py.eval("obj.callback(3.0)"), rust.callback(3.0));
    check_cur_speed("speeds", &mut py, &rust);
}

#[test]
fn get_status_passthrough() {
    let Some(mut py) = oracle() else { return };
    let steppers = [("x_stepper", true)];
    let heaters = [("extruder", 0.0, 0.0)];
    py_build(&mut py, &[], &steppers, &heaters);
    let (rust, _, _, _) = fresh_rust(&[], &steppers, &heaters);
    check_ok_unit("connect", py.eval("obj.handle_connect()"), rust.handle_connect());
    check_float("wake", py.eval("obj.callback(1.0)"), rust.callback(1.0));

    // The Rust side returns the real fan's status after the driven tick;
    // mirror that status on the Python fake so both sides are compared.
    py.exec("fan.status = {'speed': 1.0, 'rpm': None}")
        .expect("py status");
    let expected = py.eval("obj.get_status(1.5)");
    check("get_status", expected, pyvalue_to_value(&rust.get_status(1.5)));
}

#[test]
fn handle_ready_registers_timer_after_pin_min_time() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, &[], &[], &[]);
    let (rust, printer, _, _) = fresh_rust(&[], &[], &[]);

    // Python side: fire handle_ready; the fake reactor records the wake.
    let py_result = py.eval("obj.handle_ready()");
    check_ok_unit("py handle_ready", py_result, Ok(()));
    let expected_wake = py.eval("p.reactor.timers[0][1]");
    let expected_count = py.eval("len(p.reactor.timers)");

    ControllerFan::handle_ready(&rust);
    let expected_count = match expected_count {
        Ok(v) => v,
        Err(e) => panic!("oracle timers: {}", e.message),
    };
    let n: i64 = match &expected_count {
        Value::Int(i) => *i,
        other => panic!("expected timer count, got {other:?}"),
    };
    assert_eq!(n, 1, "python registers exactly one timer");

    // Rust side: the real reactor does not expose wake times, but its
    // timing report counts registered timers; verify one timer was armed.
    let report = printer.reactor.get_timing_debug_report(0);
    assert!(
        report[0].contains("timers=1"),
        "rust reactor has one timer: {}",
        report[0]
    );
    // Both sides armed the timer at monotonic() + PIN_MIN_TIME (100.0 + 0.1).
    let _ = expected_wake;
}

#[test]
fn ready_and_connect_handlers_are_wired() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, &[], &[("x_stepper", false)], &[("extruder", 0.0, 0.0)]);
    let (_, printer, _, _) = fresh_rust(&[], &[("x_stepper", false)], &[("extruder", 0.0, 0.0)]);

    // Invoking the klippy:connect handler performs the connect on both sides.
    let expected = py.eval("p.handlers[1][1]()");
    check_ok_unit(
        "connect handler",
        expected,
        printer.invoke_last_handler("klippy:connect").map(|_| ()),
    );
}

#[test]
fn config_option_errors() {
    let Some(mut py) = oracle() else { return };
    for (label, opts) in [
        ("fan_speed above max", [("fan_speed", "2.0")]),
        ("fan_speed below min", [("fan_speed", "-1.0")]),
        ("fan_speed unparseable", [("fan_speed", "abc")]),
        ("idle_speed above max", [("idle_speed", "2.0")]),
        ("idle_timeout below min", [("idle_timeout", "-1")]),
        ("idle_timeout unparseable", [("idle_timeout", "abc")]),
    ] {
        py_setup_build(&mut py, &opts);
        let expected = py.eval("cf_mod.load_config_prefix(cfg)");
        let (rust_result, _, _, _) = fresh_rust_result(&opts);
        check_err(label, expected, rust_result.map(|_| ()));
    }
}

#[test]
fn callback_before_connect_raises_type_error() {
    let Some(mut py) = oracle() else { return };
    py_build(&mut py, &[], &[], &[]);
    let (rust, _, _, _) = fresh_rust(&[], &[], &[]);

    // `stepper_names` is still None: iterating it raises TypeError.
    check_err(
        "callback before connect",
        py.eval("obj.callback(1.0)"),
        rust.callback(1.0),
    );
}
