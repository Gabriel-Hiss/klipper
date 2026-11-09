//! Parity tests for `extras::homing_heaters` (port of
//! `klippy/extras/homing_heaters.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `HomingHeaters` is built
//! on both sides with minimal Python fakes for the printer, config wrapper,
//! heaters, toolhead/kinematics, steppers, endstops and homing moves, then
//! fed identical `handle_connect` / `handle_homing_move_begin` /
//! `handle_homing_move_end` calls and event dispatches. Returned values,
//! raised errors (class and message), the registered event names, and the
//! heater `set_temp` call sequences plus final targets are compared exactly.
//!
//! The Rust side reaches the real `Heater` objects from `extras::heaters`
//! (`PrinterHeatersApi::lookup_heater` returns `Arc<Heater>`), so rust worlds
//! build production `Heater`s via `Heater::new`. A real heater records no
//! set_temp call list; its calls are observed through `target_temp()`
//! snapshots before/after each phase ([`note`]) and final `target_temp`
//! comparisons.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::heaters::{
    Heater, HeaterApi, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback,
    TemperatureSensorApi,
};
use extras::homing_heaters::{module_factory, HomingHeaters};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, RegisterCommandOpts, GCmd};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, Endstop, EndstopApi, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, PinResolver, PinsApi, ParsePinOpts,
    PwmPin, PwmPinApi, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Stepper, StepperKinematics, Trapq,
};
use klippy_api::stepper::StepperFactory;
use klippy_api::toolhead::{
    ExtraAxis, HomingMoveApi, Kinematics, KinematicsApi, LookaheadCallback, MoveApi,
    PrinterExtruder, ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.homing_heaters as homing_heaters

RAISE = object()

class CommandError(Exception):
    pass

class FakeStepper:
    def __init__(self, name):
        self._name = name
    def get_name(self, short=False):
        if short and self._name.startswith('stepper'):
            return self._name[8:]
        return self._name

class FakeKinematics:
    def __init__(self, names):
        self.steppers = [FakeStepper(n) for n in names]
    def get_steppers(self):
        return list(self.steppers)

class FakeToolhead:
    def __init__(self, stepper_names):
        self.kin = FakeKinematics(stepper_names)
    def get_kinematics(self):
        return self.kin

class FakeEndstop:
    def __init__(self, stepper_names):
        self.steppers = [FakeStepper(n) for n in stepper_names]
    def get_steppers(self):
        return list(self.steppers)

class FakeHomingMove:
    def __init__(self, endstops):
        self.endstops = endstops
    def get_mcu_endstops(self):
        return list(self.endstops)

class FakeHeater:
    def __init__(self, name, target):
        self.name = name
        self.target = target
        self.temp = 25.0
        self.min_temp = -20.0
        self.max_temp = 300.0
        self.calls = []
    def get_temp(self, eventtime):
        return (self.temp, self.target)
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise CommandError("Requested temperature (%.1f) out of range (%.1f:%.1f)"
                               % (degrees, self.min_temp, self.max_temp))
        self.target = degrees
        self.calls.append(degrees)

class FakeHeaters:
    def __init__(self, targets):
        self.heaters = {}
        for name, target in targets.items():
            self.heaters[name] = FakeHeater(name, target)
    def get_all_heaters(self):
        return list(self.heaters.keys())
    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            raise configparser.Error("Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = []
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def lookup_object(self, name, default=RAISE):
        if name in self.objects:
            return self.objects[name]
        if default is RAISE:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=RAISE):
        if section in self.objects:
            return self.objects[section]
        if default is RAISE:
            raise configparser.Error("Unable to load module '%s'" % (section,))
        return default
    def send_event(self, event, *params):
        return [cb(*params) for e, cb in self.handlers if e == event]

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, lists):
        self._printer = printer
        self._lists = lists
    def get_printer(self):
        return self._printer
    def get_name(self):
        return 'homing_heaters'
    def getlist(self, option, default=None, sep=',', count=None, note_valid=True):
        if option in self._lists:
            return list(self._lists[option])
        return default

p = None
c = None
heaters_obj = None
toolhead_obj = None
hh = None

def make_hh(heaters=None, steppers=None, targets=None, kin_steppers=None):
    global p, c, heaters_obj, toolhead_obj, hh
    p = FakePrinter()
    targets = {} if targets is None else targets
    heaters_obj = FakeHeaters(targets)
    p.objects['heaters'] = heaters_obj
    toolhead_obj = FakeToolhead([] if kin_steppers is None else kin_steppers)
    p.objects['toolhead'] = toolhead_obj
    lists = {}
    if heaters is not None:
        lists['heaters'] = heaters
    if steppers is not None:
        lists['steppers'] = steppers
    c = FakeConfig(p, lists)
    hh = homing_heaters.load_config(c)
    return None
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeStepper {
    name: String,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by homing_heaters")
    }
    fn get_name(&self, short: Option<bool>) -> String {
        if short.unwrap_or(false) && self.name.starts_with("stepper") {
            return self.name[8..].to_string();
        }
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by homing_heaters")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians is not used by homing_heaters")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by homing_heaters")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration is not used by homing_heaters")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by homing_heaters")
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist is not used by homing_heaters")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by homing_heaters")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance is not used by homing_heaters")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by homing_heaters")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by homing_heaters")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by homing_heaters")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("set_position is not used by homing_heaters")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by homing_heaters")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        panic!("get_mcu_position is not used by homing_heaters")
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        panic!("get_past_mcu_position is not used by homing_heaters")
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: i64) -> f64 {
        panic!("mcu_to_commanded_position is not used by homing_heaters")
    }
    fn dump_steps(&self, _count: i64, _start_clock: i64, _end_clock: i64) -> (Vec<PullHistoryStep>, i64) {
        panic!("dump_steps is not used by homing_heaters")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics is not used by homing_heaters")
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics is not used by homing_heaters")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by homing_heaters")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq is not used by homing_heaters")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        panic!("set_trapq is not used by homing_heaters")
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        panic!("add_active_callback is not used by homing_heaters")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by homing_heaters")
    }
}

fn stepper(name: &str) -> Stepper {
    Arc::new(FakeStepper { name: name.to_string() })
}

struct FakeEndstop {
    steppers: Vec<Stepper>,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by homing_heaters")
    }
    fn add_stepper(&self, _stepper: Stepper) {
        panic!("add_stepper is not used by homing_heaters")
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        panic!("home_start is not used by homing_heaters")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        panic!("home_wait is not used by homing_heaters")
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("query_endstop is not used by homing_heaters")
    }
}

fn endstop(names: &[&str]) -> Endstop {
    Arc::new(FakeEndstop {
        steppers: names.iter().map(|n| stepper(n)).collect(),
    })
}

struct FakeKinematics {
    steppers: Vec<Stepper>,
}

impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        panic!("calc_position is not used by homing_heaters")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by homing_heaters")
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        panic!("check_move is not used by homing_heaters")
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("home is not used by homing_heaters")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing_heaters")
    }
    fn clear_homing_state(&self, _clear_axes: &str) {
        panic!("clear_homing_state is not used by homing_heaters")
    }
}

struct FakeToolhead {
    kinematics: Kinematics,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by homing_heaters")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by homing_heaters")
    }
    fn get_kinematics(&self) -> Kinematics {
        self.kinematics.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by homing_heaters")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by homing_heaters")
    }
    fn dwell(&self, _delay: f64) {
        panic!("dwell is not used by homing_heaters")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by homing_heaters")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by homing_heaters")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by homing_heaters")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by homing_heaters")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing_heaters")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by homing_heaters")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by homing_heaters")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by homing_heaters")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by homing_heaters")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by homing_heaters")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by homing_heaters")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by homing_heaters")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by homing_heaters")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by homing_heaters")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by homing_heaters")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by homing_heaters")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by homing_heaters")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by homing_heaters")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

struct FakeHomingMove {
    endstops: Vec<Endstop>,
}

impl HomingMoveApi for FakeHomingMove {
    fn get_mcu_endstops(&self) -> Vec<Endstop> {
        self.endstops.clone()
    }
    fn check_no_movement(&self) -> Option<String> {
        None
    }
}

struct FakeSensor {
    report_delta: f64,
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 {
        self.report_delta
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

struct FakeMcu {
    est: klippy_api::GCell<f64>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used in homing_heaters tests")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is on PinsApi, not McuApi in homing_heaters tests")
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
    fn seconds_to_clock(&self, time: f64) -> i64 {
        time as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
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
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        print_time as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        *self.est.borrow()
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

impl FakePwm {
    fn new(est: f64) -> Arc<Self> {
        let mcu: Arc<FakeMcu> = Arc::new(FakeMcu { est: klippy_api::GCell::new(est) });
        Arc::new(FakePwm { mcu: mcu as Mcu })
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
    fn set_pwm(&self, _print_time: f64, _value: f64) {}
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

struct FakeGCode {
    commands: klippy_api::GCell<Vec<String>>,
    mux: klippy_api::GCell<Vec<String>>,
    raw: klippy_api::GCell<Vec<String>>,
}

impl FakeGCode {
    fn new() -> Arc<Self> {
        Arc::new(FakeGCode {
            commands: klippy_api::GCell::new(Vec::new()),
            mux: klippy_api::GCell::new(Vec::new()),
            raw: klippy_api::GCell::new(Vec::new()),
        })
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        self.commands.borrow_mut().push(format!(
            "{cmd}:{}:{}",
            opts.when_not_ready,
            opts.desc.unwrap_or_default()
        ));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.borrow_mut().push(format!(
            "{cmd}:{key}:{}:{}",
            value.unwrap_or(""),
            desc.unwrap_or("")
        ));
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
    ) -> GCmd {
        panic!("create_gcode_command not used")
    }
    fn respond_raw(&self, msg: &str) {
        self.raw.borrow_mut().push(msg.to_string());
    }
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

struct FakePrinterHeaters {
    heaters: Mutex<IndexMap<String, Arc<Heater>>>,
}

impl FakePrinterHeaters {
    fn swap(&self, name: &str, heater: Arc<Heater>) {
        self.heaters.lock().insert(name.to_string(), heater);
    }
    fn get(&self, name: &str) -> Option<Arc<Heater>> {
        self.heaters.lock().get(name).cloned()
    }
}

impl PrinterHeatersApi for FakePrinterHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {
        panic!("add_sensor_factory is not used by homing_heaters")
    }
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        panic!("setup_heater is not used by homing_heaters")
    }
    fn get_all_heaters(&self) -> Vec<String> {
        self.heaters.lock().keys().cloned().collect()
    }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        match self.get(heater_name) {
            Some(h) => Ok(h),
            None => Err(KlippyError::config(format!(
                "Unknown heater '{heater_name}'"
            ))),
        }
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        panic!("setup_sensor is not used by homing_heaters")
    }
    fn register_sensor(
        &self,
        _config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        panic!("register_sensor is not used by homing_heaters")
    }
    fn register_monitor(&self, _config: &Config) {
        panic!("register_monitor is not used by homing_heaters")
    }
    fn turn_off_all_heaters(&self, _print_time: f64) {
        panic!("turn_off_all_heaters is not used by homing_heaters")
    }
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        panic!("set_temperature is not used by homing_heaters")
    }
}

klippy_api::printer_object!(FakePrinterHeaters; dyn PrinterHeatersApi);

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

const HOMING_EVENTS: [&str; 3] = [
    "klippy:connect",
    "homing:homing_move_begin",
    "homing:homing_move_end",
];

impl FakePrinter {
    fn events(&self) -> Vec<String> {
        // `Heater::new` also registers a `klippy:shutdown` handler per heater;
        // the parity check concerns the events HomingHeaters registers.
        self.handlers
            .lock()
            .iter()
            .map(|(e, _)| e.clone())
            .filter(|e| HOMING_EVENTS.contains(&e.as_str()))
            .collect()
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
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        match module {
            None => objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(prefix) => {
                let prefix = format!("{prefix} ");
                objects
                    .iter()
                    .filter(|(k, _)| k.starts_with(&prefix))
                    .map(|(k, v)| (k.clone(), v.clone()))
                    .collect()
            }
        }
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        // `Heater::new` loads `verify_heater ...` and `pid_calibrate` and
        // discards the result; missing sections count as loaded.
        if let Some(obj) = self.objects.lock().get(section).cloned() {
            return Ok(Some(obj));
        }
        Ok(None)
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
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        let taken = std::mem::take(&mut *self.handlers.lock());
        let mut results = Vec::new();
        let mut remaining = Vec::new();
        for (name, mut cb) in taken {
            if name == event {
                results.push(cb(params)?);
            }
            remaining.push((name, cb));
        }
        *self.handlers.lock() = remaining;
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    lists: IndexMap<String, Option<Vec<String>>>,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(
        printer: Printer,
        name: &str,
        lists: IndexMap<String, Option<Vec<String>>>,
        options: IndexMap<String, String>,
    ) -> Arc<Self> {
        Arc::new(FakeConfig {
            printer,
            name: name.to_string(),
            lists,
            options,
        })
    }
    fn required(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        ))
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
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(self.required(option)),
            Some(d) => Ok(d),
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by homing_heaters")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        if let Some(raw) = self.options.get(option) {
            let v: f64 = raw.parse().map_err(|_| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                ))
            })?;
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(self.required(option)),
            Some(d) => Ok(d),
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by homing_heaters")
    }
    fn getchoice(
        &self,
        option: &str,
        _choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        if let Some(v) = self.options.get(option) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            None => Err(self.required(option)),
            Some(d) => Ok(d.unwrap_or(PyValue::None)),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by homing_heaters")
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        if let Some(v) = self.lists.get(option) {
            return Ok(v.clone());
        }
        match opts.default {
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' is required",
                self.name
            ))),
        }
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by homing_heaters")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by homing_heaters")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by homing_heaters")
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

klippy_api::printer_object!(FakeConfig);

// ---------------------------------------------------------------------------
// World builders and comparison plumbing.
// ---------------------------------------------------------------------------

struct World {
    hh: Arc<HomingHeaters>,
    printer: Arc<FakePrinter>,
    heaters: Arc<FakePrinterHeaters>,
}

fn build_heater(printer: &Printer, name: &str, max_temp: f64, initial_target: f64) -> Arc<Heater> {
    let mut options = IndexMap::new();
    options.insert("min_temp".to_string(), "-20.0".to_string());
    options.insert("max_temp".to_string(), format!("{max_temp:?}"));
    options.insert("control".to_string(), "watermark".to_string());
    options.insert("heater_pin".to_string(), "PA0".to_string());
    let cfg: Config = FakeConfig::new(
        printer.clone(),
        &format!("heater_generic {name}"),
        IndexMap::new(),
        options,
    );
    let sensor = Arc::new(FakeSensor { report_delta: 0.3 });
    let heater = Heater::new(&cfg, sensor as Arc<dyn TemperatureSensorApi>)
        .expect("Heater::new");
    heater.set_temp(initial_target).expect("heater initial set_temp");
    heater
}

fn rust_world(
    heaters: Option<Vec<&str>>,
    steppers: Option<Vec<&str>>,
    targets: &[(&str, f64)],
    kin_steppers: &[&str],
) -> World {
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(IndexMap::new()),
        handlers: Mutex::new(Vec::new()),
    });
    let printer_typed: Printer = printer.clone();
    // `Heater::new` resolves `pins` and `gcode` printer objects.
    let pwm = FakePwm::new(1000.0);
    printer
        .add_object("pins", Arc::new(FakePins { pwm: pwm.clone() as PwmPin }) as ObjectRef)
        .unwrap();
    printer
        .add_object("gcode", FakeGCode::new() as ObjectRef)
        .unwrap();
    let mut heaters_map = IndexMap::new();
    for (name, target) in targets {
        // The Python fake assigns the initial target directly, bypassing range
        // validation; mirror that by widening max_temp above out-of-range
        // targets so the production Heater accepts them.
        let max_temp = if *target > 300.0 { 600.0 } else { 300.0 };
        let heater = build_heater(&printer_typed, name, max_temp, *target);
        heaters_map.insert(name.to_string(), heater);
    }
    let pheaters = Arc::new(FakePrinterHeaters {
        heaters: Mutex::new(heaters_map),
    });
    printer
        .add_object("heaters", Arc::clone(&pheaters) as ObjectRef)
        .unwrap();
    let toolhead = Arc::new(FakeToolhead {
        kinematics: Arc::new(FakeKinematics {
            steppers: kin_steppers.iter().map(|n| stepper(n)).collect(),
        }),
    });
    printer
        .add_object("toolhead", toolhead as ObjectRef)
        .unwrap();
    let mut lists = IndexMap::new();
    if let Some(h) = heaters {
        lists.insert("heaters".to_string(), Some(h.iter().map(|s| s.to_string()).collect()));
    }
    if let Some(s) = steppers {
        lists.insert("steppers".to_string(), Some(s.iter().map(|s| s.to_string()).collect()));
    }
    let config: Config = FakeConfig::new(
        Arc::clone(&printer) as Printer,
        "homing_heaters",
        lists,
        IndexMap::new(),
    );
    let hh = HomingHeaters::new(&config).expect("rust HomingHeaters::new");
    World { hh, printer, heaters: pheaters }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("homing_heaters fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping homing_heaters oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_str_list(items: &[&str]) -> String {
    let parts: Vec<String> = items.iter().map(|s| format!("'{s}'")).collect();
    format!("[{}]", parts.join(", "))
}

fn py_targets(targets: &[(&str, f64)]) -> String {
    let parts: Vec<String> = targets
        .iter()
        .map(|(n, v)| format!("'{n}': {v:.1}"))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn py_opt_list(items: &Option<Vec<&str>>) -> String {
    match items {
        Some(v) => py_str_list(v),
        None => "None".to_string(),
    }
}

fn py_world(
    py: &mut pyoracle::Oracle,
    heaters: Option<Vec<&str>>,
    steppers: Option<Vec<&str>>,
    targets: &[(&str, f64)],
    kin_steppers: &[&str],
) {
    let code = format!(
        "make_hh({}, {}, {}, {})",
        py_opt_list(&heaters),
        py_opt_list(&steppers),
        py_targets(targets),
        py_str_list(kin_steppers),
    );
    py.exec(&code).expect("py make_hh");
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "homing_heaters".to_string(),
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

fn check_unit(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => match v {
            Value::Null => None,
            _ => panic!("{label}: oracle returned {v:?}, expected None"),
        },
        Err(e) => Some(e),
    };
    match (oracle_exc, actual) {
        (None, Ok(())) => {}
        (Some(e), Err(rust)) => {
            assert_eq!(
                rust.class_name(),
                e.class_name.as_str(),
                "{label}: exception class",
            );
            assert_eq!(
                rust.message(),
                e.message.as_str(),
                "{label}: exception message",
            );
        }
        (None, Err(e)) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
        (Some(e), Ok(())) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    }
}

fn heater(world: &World, name: &str) -> Arc<Heater> {
    world
        .heaters
        .get(name)
        .expect("heater exists")
}

fn target_of(world: &World, name: &str) -> f64 {
    heater(world, name).target_temp()
}

// A real `Heater` records no `set_temp` call list, so the call sequence is
// observed through `target_temp()` snapshots: a snapshot is appended whenever
// a phase leaves the target different from before the phase, which is exactly
// the append condition of the Python fake call list.
fn note(log: &mut Vec<f64>, before: f64, after: f64) {
    if before != after {
        log.push(after);
    }
}

fn calls_value(log: &[f64]) -> Value {
    Value::List(log.iter().map(|v| Value::float(*v)).collect())
}

fn events_value(printer: &FakePrinter) -> Value {
    Value::List(printer.events().iter().map(|e| Value::str(e)).collect())
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn events_registered_match_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    let world = rust_world(None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    check(
        "events_registered_match_python",
        py.eval("[e for e, _ in p.handlers]"),
        events_value(&world.printer),
    );
}

#[test]
fn connect_defaults_to_all_heaters() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        None,
        None,
        &[("heater_bed", 200.0), ("extruder", 210.0)],
        &["stepper_x"],
    );
    let world = rust_world(
        None,
        None,
        &[("heater_bed", 200.0), ("extruder", 210.0)],
        &["stepper_x"],
    );
    check_unit(
        "connect_defaults_to_all_heaters",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    // With no steppers configured, every homing move is eligible.
    let begin_expected = py.eval(
        "hh.handle_homing_move_begin(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    let bed_before = target_of(&world, "heater_bed");
    let extruder_before = target_of(&world, "extruder");
    check_unit(
        "begin_disables_all_heaters",
        begin_expected,
        world.hh.handle_homing_move_begin(&*hmove),
    );
    let mut bed_log = Vec::new();
    note(&mut bed_log, bed_before, target_of(&world, "heater_bed"));
    let mut extruder_log = Vec::new();
    note(&mut extruder_log, extruder_before, target_of(&world, "extruder"));
    check(
        "heater_bed_disabled",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&bed_log),
    );
    check(
        "extruder_disabled",
        py.eval("heaters_obj.heaters['extruder'].calls"),
        calls_value(&extruder_log),
    );
    check(
        "heater_bed_target_is_zero",
        py.eval("heaters_obj.heaters['heater_bed'].target"),
        Value::float(target_of(&world, "heater_bed")),
    );
    let bed_before = target_of(&world, "heater_bed");
    let extruder_before = target_of(&world, "extruder");
    let end_expected = py.eval(
        "hh.handle_homing_move_end(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    check_unit(
        "end_restores_targets",
        end_expected,
        world.hh.handle_homing_move_end(&*hmove),
    );
    note(&mut bed_log, bed_before, target_of(&world, "heater_bed"));
    note(&mut extruder_log, extruder_before, target_of(&world, "extruder"));
    check(
        "heater_bed_calls_after_end",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&bed_log),
    );
    check(
        "extruder_calls_after_end",
        py.eval("heaters_obj.heaters['extruder'].calls"),
        calls_value(&extruder_log),
    );
    check(
        "heater_bed_target_restored",
        py.eval("heaters_obj.heaters['heater_bed'].target"),
        Value::float(target_of(&world, "heater_bed")),
    );
}

#[test]
fn connect_via_send_event() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    let world = rust_world(None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    py.exec("p.send_event('klippy:connect')").expect("py connect event");
    world
        .printer
        .send_event("klippy:connect", &[])
        .expect("rust connect event");
    py.exec("p.send_event('homing:homing_move_begin', FakeHomingMove([FakeEndstop(['stepper_x'])]))")
        .expect("py begin event");
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    let before = target_of(&world, "heater_bed");
    world
        .printer
        .send_event(
            "homing:homing_move_begin",
            &[EventArg::handle(hmove)],
        )
        .expect("rust begin event");
    let mut log = Vec::new();
    note(&mut log, before, target_of(&world, "heater_bed"));
    check(
        "heater_calls_via_events",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&log),
    );
}

#[test]
fn connect_with_explicit_valid_lists() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        Some(vec!["heater_bed"]),
        Some(vec!["stepper_x"]),
        &[("heater_bed", 200.0), ("extruder", 210.0)],
        &["stepper_x", "stepper_y"],
    );
    let world = rust_world(
        Some(vec!["heater_bed"]),
        Some(vec!["stepper_x"]),
        &[("heater_bed", 200.0), ("extruder", 210.0)],
        &["stepper_x", "stepper_y"],
    );
    check_unit(
        "connect_with_explicit_valid_lists",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    let begin_expected = py.eval(
        "hh.handle_homing_move_begin(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    let bed_before = target_of(&world, "heater_bed");
    let extruder_before = target_of(&world, "extruder");
    check_unit(
        "begin_with_explicit_lists",
        begin_expected,
        world.hh.handle_homing_move_begin(&*hmove),
    );
    let mut bed_log = Vec::new();
    note(&mut bed_log, bed_before, target_of(&world, "heater_bed"));
    let mut extruder_log = Vec::new();
    note(&mut extruder_log, extruder_before, target_of(&world, "extruder"));
    check(
        "listed_heater_disabled",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&bed_log),
    );
    check(
        "unlisted_heater_untouched",
        py.eval("heaters_obj.heaters['extruder'].calls"),
        calls_value(&extruder_log),
    );
}

#[test]
fn connect_unknown_heater_error() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        Some(vec!["heater_bed", "nope"]),
        None,
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    let world = rust_world(
        Some(vec!["heater_bed", "nope"]),
        None,
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    check_unit(
        "connect_unknown_heater_error",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
}

#[test]
fn connect_unknown_stepper_error() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        None,
        Some(vec!["stepper_x", "nope"]),
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    let world = rust_world(
        None,
        Some(vec!["stepper_x", "nope"]),
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    check_unit(
        "connect_unknown_stepper_error",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
}

#[test]
fn connect_without_steppers_option_skips_stepper_validation() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, None, None, &[("heater_bed", 200.0)], &[]);
    let world = rust_world(None, None, &[("heater_bed", 200.0)], &[]);
    check_unit(
        "connect_without_steppers_option_skips_stepper_validation",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
}

#[test]
fn ineligible_stepper_keeps_heaters_on() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        None,
        Some(vec!["stepper_x"]),
        &[("heater_bed", 200.0)],
        &["stepper_x", "stepper_y"],
    );
    let world = rust_world(
        None,
        Some(vec!["stepper_x"]),
        &[("heater_bed", 200.0)],
        &["stepper_x", "stepper_y"],
    );
    check_unit(
        "connect_ineligible",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    let begin_expected = py.eval(
        "hh.handle_homing_move_begin(FakeHomingMove([FakeEndstop(['stepper_y'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_y"])],
    });
    let mut log = Vec::new();
    let before = target_of(&world, "heater_bed");
    check_unit(
        "ineligible_begin",
        begin_expected,
        world.hh.handle_homing_move_begin(&*hmove),
    );
    note(&mut log, before, target_of(&world, "heater_bed"));
    let end_expected = py.eval(
        "hh.handle_homing_move_end(FakeHomingMove([FakeEndstop(['stepper_y'])]))",
    );
    let before = target_of(&world, "heater_bed");
    check_unit(
        "ineligible_end",
        end_expected,
        world.hh.handle_homing_move_end(&*hmove),
    );
    note(&mut log, before, target_of(&world, "heater_bed"));
    check(
        "heater_never_called",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&log),
    );
    check(
        "heater_target_unchanged",
        py.eval("heaters_obj.heaters['heater_bed'].target"),
        Value::float(target_of(&world, "heater_bed")),
    );
}

#[test]
fn end_without_begin_raises_key_error() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    let world = rust_world(None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    check_unit(
        "connect_end_without_begin",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    let end_expected = py.eval(
        "hh.handle_homing_move_end(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    check_unit(
        "end_without_begin_raises_key_error",
        end_expected,
        world.hh.handle_homing_move_end(&*hmove),
    );
}

#[test]
fn restore_out_of_range_target_propagates_command_error() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, None, None, &[("heater_bed", 500.0)], &["stepper_x"]);
    let world = rust_world(None, None, &[("heater_bed", 500.0)], &["stepper_x"]);
    check_unit(
        "connect_out_of_range",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    let begin_expected = py.eval(
        "hh.handle_homing_move_begin(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    let before = target_of(&world, "heater_bed");
    check_unit(
        "begin_out_of_range",
        begin_expected,
        world.hh.handle_homing_move_begin(&*hmove),
    );
    let mut log = Vec::new();
    note(&mut log, before, target_of(&world, "heater_bed"));
    // The Rust world starts with a max_temp of 600 so the saved target of
    // 500 is in range; swap in the max_temp of 300 heater the Python fake
    // uses before the restore, so the restore is out of range on both sides.
    let printer_typed: Printer = world.printer.clone();
    world.heaters.swap(
        "heater_bed",
        build_heater(&printer_typed, "heater_bed", 300.0, 0.0),
    );
    let before = target_of(&world, "heater_bed");
    let end_expected = py.eval(
        "hh.handle_homing_move_end(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    check_unit(
        "restore_out_of_range_target_propagates_command_error",
        end_expected,
        world.hh.handle_homing_move_end(&*hmove),
    );
    note(&mut log, before, target_of(&world, "heater_bed"));
    check(
        "heater_calls_out_of_range",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&log),
    );
}

#[test]
fn empty_heater_list_disables_nothing() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        Some(vec![]),
        None,
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    let world = rust_world(
        Some(vec![]),
        None,
        &[("heater_bed", 200.0)],
        &["stepper_x"],
    );
    check_unit(
        "connect_empty_heater_list",
        py.eval("hh.handle_connect()"),
        world.hh.handle_connect(),
    );
    let begin_expected = py.eval(
        "hh.handle_homing_move_begin(FakeHomingMove([FakeEndstop(['stepper_x'])]))",
    );
    let hmove: Arc<dyn HomingMoveApi> = Arc::new(FakeHomingMove {
        endstops: vec![endstop(&["stepper_x"])],
    });
    let before = target_of(&world, "heater_bed");
    check_unit(
        "begin_empty_heater_list",
        begin_expected,
        world.hh.handle_homing_move_begin(&*hmove),
    );
    let mut log = Vec::new();
    note(&mut log, before, target_of(&world, "heater_bed"));
    check(
        "heater_untouched_empty_list",
        py.eval("heaters_obj.heaters['heater_bed'].calls"),
        calls_value(&log),
    );
}

#[test]
fn module_factory_load_config_builds_object() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let world = rust_world(None, None, &[("heater_bed", 200.0)], &["stepper_x"]);
    let _ = world;
}
