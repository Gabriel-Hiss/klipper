//! Parity tests for `klippy/kinematics/winch.py`.
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a `WinchKinematics` is built on
//! both sides with minimal Python fakes for the stepper factory, the
//! toolhead and the config wrapper, then driven through construction,
//! `get_steppers`, `get_status`, `calc_position`, `set_position`, `home`,
//! `check_move` and `clear_homing_state` with identical inputs. Log lines,
//! returned values (floats by bits via `recording::assert_equal`) and
//! raised errors (class and message) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone. The Python fakes replace `stepper.PrinterStepper` with a
//! recorder; the Rust fakes implement the same recorders through the
//! `klippy_api` traits. `mathutil.trilateration` is the real reference
//! implementation on the Python side.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use kinematics::winch::{load_kinematics, module_factory, WinchKinematics};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::EventArg;
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Rail, RailOpts, Stepper,
    StepperFactory, StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, HomingStateApi, HomingStateHandle, Kinematics, KinematicsApi,
    LookaheadCallback, MoveApi, PrinterExtruderApi, ToolHead, ToolHeadApi,
};
use kos::sync::Mutex;
use pycompat::text::float_repr;
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::value::{Record, RecordKind, SideEffects, Value};
use recording::assert_equal;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import math
import gcode
import configparser
import stepper as stepper_mod
import kinematics.winch as winch_mod

call_log = []

class FakeStepper:
    def __init__(self, cfg):
        self.name = cfg.get_name()
        self.alloc = None
        self.trapq = None
        self.coords = []
        call_log.append('PrinterStepper:%s' % (self.name,))
    def get_name(self, short=False):
        return self.name
    def setup_itersolve(self, alloc, *params):
        self.alloc = (alloc, params)
        call_log.append('setup_itersolve:%s:%s:%r' % (self.name, alloc, params))
    def set_trapq(self, tq):
        self.trapq = tq
        call_log.append('set_trapq:%s' % (self.name,))
    def set_position(self, coord):
        # Mirror `stepper.PrinterStepper.set_position` (`stepper.py:158`):
        # the real stepper indexes `coord[0], coord[1], coord[2]`, so a
        # short coord raises IndexError before any state changes.
        vals = (coord[0], coord[1], coord[2])
        self.coords.append(vals)
        call_log.append('set_position:%s:%r' % (self.name, vals))

stepper_mod.PrinterStepper = FakeStepper

class FakeToolhead:
    Coord = gcode.Coord
    def get_trapq(self):
        return 'fake-trapq'

class _StepperCfg:
    def __init__(self, name, anchor):
        self._name = name
        self._anchor = anchor
    def get_name(self):
        return self._name
    def getfloat(self, opt):
        call_log.append('getfloat:%s:%s' % (self._name, opt))
        if self._anchor is None:
            raise configparser.Error(
                "Missing [%s] anchor option %s" % (self._name, opt))
        idx = {'anchor_x': 0, 'anchor_y': 1, 'anchor_z': 2}[opt]
        return self._anchor[idx]

class FakeConfig:
    def __init__(self, sections):
        self._sections = sections
        self._cfgs = dict((n, _StepperCfg(n, a)) for n, a in sections.items())
    def has_section(self, name):
        call_log.append('has_section:%s' % (name,))
        return name in self._sections
    def getsection(self, name):
        call_log.append('getsection:%s' % (name,))
        return self._cfgs[name]

class FakeHomingState:
    def __init__(self):
        self.axes = None
        self.pos = None
    def set_axes(self, axes):
        self.axes = list(axes)
        call_log.append('set_axes:%r' % (list(axes),))
    def get_axes(self):
        return list(self.axes)
    def set_homed_position(self, pos):
        self.pos = list(pos)
        call_log.append('set_homed_position:%r' % (list(pos),))

def make_kin(sections):
    global kin
    kin = winch_mod.load_kinematics(FakeToolhead(), FakeConfig(sections))
    return kin
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Log = Arc<Mutex<Vec<String>>>;

fn log_entry(log: &Log, entry: String) {
    log.lock().push(entry);
}

fn format_coord(c: [f64; 3]) -> String {
    format!(
        "({0}, {1}, {2})",
        float_repr(c[0]),
        float_repr(c[1]),
        float_repr(c[2])
    )
}

#[derive(Clone)]
struct FakePrinter;

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
    fn add_object(&self, _name: &str, _obj: klippy_api::ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        _name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(
        &self,
        _module: Option<&str>,
    ) -> Vec<(String, klippy_api::ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
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

fn fake_printer() -> Printer {
    Arc::new(FakePrinter)
}

/// One `[stepper_*]` section: `Some(anchor)` or `None` (missing anchor
/// options), mirroring the Python fakes.
struct FakeConfig {
    printer: Printer,
    sections: Arc<IndexMap<String, Option<[f64; 3]>>>,
    name: String,
    log: Log,
}

impl FakeConfig {
    fn new(sections: IndexMap<String, Option<[f64; 3]>>, log: Log) -> FakeConfig {
        FakeConfig {
            printer: fake_printer(),
            sections: Arc::new(sections),
            name: "printer".to_string(),
            log,
        }
    }

    fn section(&self, name: &str) -> FakeConfig {
        FakeConfig {
            printer: self.printer.clone(),
            sections: self.sections.clone(),
            name: name.to_string(),
            log: self.log.clone(),
        }
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Err(KlippyError::config("unsupported get in winch fake"))
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(KlippyError::config("unsupported getint in winch fake"))
    }
    fn getfloat(
        &self,
        option: &str,
        _opts: GetFloatOpts,
    ) -> KResult<Option<f64>> {
        log_entry(&self.log, format!("getfloat:{}:{}", self.name, option));
        let anchor = self.sections.get(&self.name).cloned().flatten();
        match anchor {
            Some(a) => Ok(Some(match option {
                "anchor_x" => a[0],
                "anchor_y" => a[1],
                "anchor_z" => a[2],
                other => {
                    return Err(KlippyError::config(format!(
                        "unknown option {other}"
                    )))
                }
            })),
            None => Err(KlippyError::config(format!(
                "Missing [{}] anchor option {}",
                self.name, option
            ))),
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(KlippyError::config("unsupported getboolean in winch fake"))
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::config("unsupported getchoice in winch fake"))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::config("unsupported getlists in winch fake"))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::config("unsupported getlist in winch fake"))
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::config("unsupported getintlist in winch fake"))
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::config("unsupported getfloatlist in winch fake"))
    }
    fn getsection(&self, section: &str) -> Config {
        log_entry(&self.log, format!("getsection:{}", section));
        Arc::new(self.section(section))
    }
    fn has_section(&self, section: &str) -> bool {
        log_entry(&self.log, format!("has_section:{}", section));
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct FakeStepper {
    name: String,
    log: Log,
    prefs: Mutex<Vec<[f64; 3]>>,
}

impl FakeStepper {
    fn new(name: String, log: Log) -> FakeStepper {
        log_entry(&log, format!("PrinterStepper:{}", name));
        FakeStepper {
            name,
            log,
            prefs: Mutex::new(Vec::new()),
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        Arc::new(FakeMcu { printer: fake_printer() })
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        0
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        let parts: Vec<String> = params
            .iter()
            .map(|p| match p {
                PyValue::Float(f) => float_repr(*f),
                other => other.repr(),
            })
            .collect();
        log_entry(
            &self.log,
            format!(
                "setup_itersolve:{}:{}:({})",
                self.name,
                alloc_func,
                parts.join(", ")
            ),
        );
    }
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
    fn set_position(&self, coord: [f64; 3]) {
        log_entry(
            &self.log,
            format!(
                "set_position:{}:{}",
                self.name,
                format_coord(coord)
            ),
        );
        self.prefs.lock().push(coord);
    }
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
        log_entry(&self.log, format!("set_trapq:{}", self.name));
        None
    }
    fn add_active_callback(
        &self,
        _cb: Box<dyn FnMut(f64) + Send>,
    ) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeFactory {
    log: Log,
    made: Mutex<Vec<Arc<FakeStepper>>>,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(
        &self,
        config: &Config,
        _units_in_radians: bool,
    ) -> KResult<Stepper> {
        let s = Arc::new(FakeStepper::new(config.get_name(), self.log.clone()));
        self.made.lock().push(s.clone());
        Ok(s)
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake factory lookup_rail".to_string(),
        })
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake factory lookup_multi_rail".to_string(),
        })
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake factory generic_rail".to_string(),
        })
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake factory parse_step_distance".to_string(),
        })
    }
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeAsyncResponse;

impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsyncResponse {
    fn unregister(&self) {}
}

struct FakeMcu {
    printer: Printer,
}

impl klippy_api::mcu::McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        true
    }
    fn setup_pin(
        &self,
        _pin_type: &str,
        _pin_params: klippy_api::mcu::PinParams,
    ) -> KResult<klippy_api::mcu::Pin> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake mcu setup_pin".to_string(),
        })
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(
        &self,
        _cb: klippy_api::mcu::ConfigCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(
        &self,
        _msgformat: &str,
        _cq: Option<klippy_api::mcu::CommandQueue>,
    ) -> KResult<klippy_api::mcu::CommandWrapper> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake mcu lookup_command".to_string(),
        })
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake mcu lookup_query_command".to_string(),
        })
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> klippy_api::mcu::CommandQueue {
        klippy_api::mcu::CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        Arc::new(FakeAsyncResponse)
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
        Err(KlippyError::config("winch fake mcu missing constant"))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, _clock32: i64) -> i64 {
        0
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "winch fake mcu create_trigger_dispatch".to_string(),
        })
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
        Err(klippy_api::toolhead::no_kinematics_attr("calc_position"))
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Err(klippy_api::toolhead::no_kinematics_attr("set_position"))
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Err(klippy_api::toolhead::no_kinematics_attr("check_move"))
    }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        Err(klippy_api::toolhead::no_kinematics_attr("home"))
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}

struct DummyExtruder;

impl ExtraAxisApi for DummyExtruder {
    fn get_name(&self) -> String {
        "extruder".to_string()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        Some("E".to_string())
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
        0.0
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for DummyExtruder {
    fn get_heater(&self) -> KResult<klippy_api::ObjectRef> {
        Err(KlippyError::command("Extruder not configured"))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

struct FakeToolHead {
    factory: StepperFactory,
    _trapq: Trapq,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0]
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
    fn get_last_move_time(&self) -> f64 {
        0.0
    }
    fn dwell(&self, _delay: f64) {}
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
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
    fn get_trapq(&self) -> Trapq {
        self._trapq.clone()
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(
        &self,
        _callback: LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(
        &self,
        _extruder: klippy_api::toolhead::PrinterExtruder,
        _extrude_pos: f64,
    ) {
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        Arc::new(DummyExtruder)
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

struct FakeHomingState {
    log: Log,
}

impl HomingStateApi for FakeHomingState {
    fn set_axes(&self, axes: Vec<i64>) {
        let parts: Vec<String> = axes.iter().map(|a| a.to_string()).collect();
        log_entry(&self.log, format!("set_axes:[{}]", parts.join(", ")));
    }
    fn get_axes(&self) -> Vec<i64> {
        vec![0, 1, 2]
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'x'".to_string(),
        })
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, pos: Vec<Option<f64>>) -> KResult<()> {
        let parts: Vec<String> = pos
            .iter()
            .map(|p| match p {
                Some(v) => float_repr(*v),
                None => "None".to_string(),
            })
            .collect();
        log_entry(
            &self.log,
            format!("set_homed_position:[{}]", parts.join(", ")),
        );
        Ok(())
    }
    fn home_rails(
        &self,
        _rails: Vec<Rail>,
        _forcepos: Vec<Option<f64>>,
        _movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        Ok(())
    }
}

struct DummyMove;

impl MoveApi for DummyMove {
    fn start_pos(&self) -> klippy_api::toolhead::Coord {
        klippy_api::toolhead::Coord(vec![0.0, 0.0, 0.0])
    }
    fn end_pos(&self) -> klippy_api::toolhead::Coord {
        klippy_api::toolhead::Coord(vec![0.0, 0.0, 0.0])
    }
    fn axes_d(&self) -> klippy_api::toolhead::Coord {
        klippy_api::toolhead::Coord(vec![0.0, 0.0, 0.0])
    }
    fn axes_r(&self) -> klippy_api::toolhead::Coord {
        klippy_api::toolhead::Coord(vec![0.0, 0.0, 0.0])
    }
    fn move_d(&self) -> f64 {
        0.0
    }
    fn move_error(&self, _msg: String) -> KlippyError {
        KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "winch fake move".to_string(),
        }
    }
    fn start_v(&self) -> f64 {
        0.0
    }
    fn cruise_v(&self) -> f64 {
        0.0
    }
    fn accel(&self) -> f64 {
        0.0
    }
    fn max_cruise_v2(&self) -> f64 {
        0.0
    }
    fn accel_t(&self) -> f64 {
        0.0
    }
    fn cruise_t(&self) -> f64 {
        0.0
    }
    fn decel_t(&self) -> f64 {
        0.0
    }
    fn is_kinematic_move(&self) -> bool {
        false
    }
    fn limit_speed(&self, _max_cruise_v: f64, _max_accel: f64) {}
}


// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("winch fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping winch oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "kinematics.winch".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record_for(label), "ret", &expected, &actual) {
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
        Ok(_) => panic!("{label}: rust call returned Ok but oracle raised"),
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

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(
            n.0.to_string().parse::<i64>().expect("test int fits in i64"),
        ),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(xs) => Value::List(xs.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(xs) => Value::Tuple(xs.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "host".to_string(),
            addr: format!("0x{h:x}"),
        },
    }
}


/// Sections for the first `n` steppers with distinct anchors.
fn sections_n(n: usize) -> IndexMap<String, Option<[f64; 3]>> {
    let mut s = IndexMap::new();
    for i in 0..n {
        let name = format!("stepper_{}", (b'a' + i as u8) as char);
        s.insert(
            name,
            Some([
                (i as f64) * 10.0,
                (i as f64) * 20.0 + 5.0,
                (i as f64) * 30.0 + 100.0,
            ]),
        );
    }
    s
}

/// Python expression building a sections dict.
fn py_sections_expr(sections: &IndexMap<String, Option<[f64; 3]>>) -> String {
    let parts: Vec<String> = sections
        .iter()
        .map(|(k, v)| match v {
            Some(a) => format!(
                "'{}': ({}, {}, {})",
                k,
                float_repr(a[0]),
                float_repr(a[1]),
                float_repr(a[2])
            ),
            None => format!("'{k}': None"),
        })
        .collect();
    format!("{{{}}}", parts.join(", "))
}

struct RustWorld {
    kin: WinchKinematics,
    log: Log,
}

fn rust_world(sections: IndexMap<String, Option<[f64; 3]>>) -> RustWorld {
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let factory = Arc::new(FakeFactory {
        log: log.clone(),
        made: Mutex::new(Vec::new()),
    });
    let toolhead: ToolHead = Arc::new(FakeToolHead {
        factory,
        _trapq: Arc::new(FakeTrapq),
    });
    let config: Config = Arc::new(FakeConfig::new(sections, log.clone()));
    let kin = WinchKinematics::new(&toolhead, &config).expect("rust build");
    RustWorld { kin, log }
}

fn clear_logs(py: &mut pyoracle::Oracle, log: &Log) {
    py.exec("call_log.clear()").expect("clear py log");
    log.lock().clear();
}

fn check_log(py: &mut pyoracle::Oracle, label: &str, log: &Log) {
    let expected = py.eval("list(call_log)").expect("read py log");
    let actual = Value::List(
        log.lock()
            .iter()
            .map(|s| Value::str(s))
            .collect(),
    );
    check(label, Ok(expected), actual);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_no_slots() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn construction_three_steppers() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    check_log(&mut py, "build log", &world.log);
    let expected = py
        .eval("kin.get_status(1.5)")
        .expect("py get_status");
    check(
        "get_status",
        Ok(expected),
        pyvalue_to_value(&world.kin.get_status(1.5)),
    );
    let expected = py.eval("len(kin.get_steppers())").expect("py count");
    check(
        "stepper count",
        Ok(expected),
        Value::int(world.kin.get_steppers().len() as i64),
    );
    let anchors: Vec<Value> = world
        .kin
        .anchors()
        .iter()
        .map(|a| {
            Value::List(vec![
                Value::float(a[0]),
                Value::float(a[1]),
                Value::float(a[2]),
            ])
        })
        .collect();
    let expected = py
        .eval("[list(a) for a in kin.anchors]")
        .expect("py anchors");
    check("anchors", Ok(expected), Value::List(anchors));
}

#[test]
fn construction_extra_steppers_until_missing() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(5);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    check_log(&mut py, "build log", &world.log);
    let expected = py
        .eval("[s.get_name() for s in kin.get_steppers()]")
        .expect("py names");
    let names: Vec<Value> = world
        .kin
        .get_steppers()
        .iter()
        .map(|s| Value::str(&s.get_name(None)))
        .collect();
    check("stepper names", Ok(expected), Value::List(names));
    let expected = py.eval("kin.get_status(0.0)").expect("py status");
    check(
        "get_status",
        Ok(expected),
        pyvalue_to_value(&world.kin.get_status(0.0)),
    );
}

#[test]
fn construction_uses_first_three_anchors_only_for_limits() {
    let Some(mut py) = oracle() else { return };
    let mut sections = sections_n(4);
    // The fourth anchor extends the limits even though only the first
    // three anchors matter for calc_position.
    sections.insert(
        "stepper_d".to_string(),
        Some([-100.0, 500.0, -50.0]),
    );
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let expected = py.eval("kin.get_status(0.0)").expect("py status");
    check(
        "get_status",
        Ok(expected),
        pyvalue_to_value(&world.kin.get_status(0.0)),
    );
}

#[test]
fn calc_position_nominal() {
    let Some(mut py) = oracle() else { return };
    let sections: IndexMap<String, Option<[f64; 3]>> =
        [("stepper_a", [0.0, 0.0, 0.0]),
         ("stepper_b", [100.0, 0.0, 0.0]),
         ("stepper_c", [0.0, 100.0, 0.0])]
        .into_iter()
        .map(|(k, a)| (k.to_string(), Some(a)))
        .collect();
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    // Target (30, 40, 50).
    let ra = (30.0f64 * 30.0 + 40.0 * 40.0 + 50.0 * 50.0).sqrt();
    let rb = (70.0f64 * 70.0 + 40.0 * 40.0 + 50.0 * 50.0).sqrt();
    let rc = (30.0f64 * 30.0 + 60.0 * 60.0 + 50.0 * 50.0).sqrt();
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), ra);
    pos.insert("stepper_b".to_string(), rb);
    pos.insert("stepper_c".to_string(), rc);
    let expected = py
        .eval(&format!(
            "kin.calc_position({{'stepper_a': {ra:?}, 'stepper_b': {rb:?}, 'stepper_c': {rc:?}}})"
        ))
        .expect("py calc_position");
    let actual = world
        .kin
        .calc_position(&pos)
        .expect("rust calc_position")
        .into_iter()
        .map(|v| Value::float(v.unwrap()))
        .collect();
    check("calc_position", Ok(expected), Value::List(actual));
}

#[test]
fn calc_position_generic_anchors() {
    let Some(mut py) = oracle() else { return };
    let sections: IndexMap<String, Option<[f64; 3]>> =
        [("stepper_a", [10.0, -20.0, 30.0]),
         ("stepper_b", [-40.0, 50.0, 60.0]),
         ("stepper_c", [70.0, 80.0, -90.0]),
         ("stepper_d", [0.0, 0.0, 0.0])]
        .into_iter()
        .map(|(k, a)| (k.to_string(), Some(a)))
        .collect();
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), 120.5);
    pos.insert("stepper_b".to_string(), 95.25);
    pos.insert("stepper_c".to_string(), 140.0);
    let expected = py
        .eval(
            "kin.calc_position({'stepper_a': 120.5, 'stepper_b': 95.25, 'stepper_c': 140.0})",
        )
        .expect("py calc_position");
    let actual = world
        .kin
        .calc_position(&pos)
        .expect("rust calc_position")
        .into_iter()
        .map(|v| Value::float(v.unwrap()))
        .collect();
    check("calc_position", Ok(expected), Value::List(actual));
}

#[test]
fn calc_position_missing_key() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), 10.0);
    pos.insert("stepper_c".to_string(), 10.0);
    let expected = py
        .eval("kin.calc_position({'stepper_a': 10.0, 'stepper_c': 10.0})")
        .expect_err("py must raise KeyError");
    check_err(
        "missing stepper_b",
        Err(expected),
        world.kin.calc_position(&pos),
    );
}

#[test]
fn calc_position_degenerate_anchors() {
    let Some(mut py) = oracle() else { return };
    // Collinear anchors: the ey direction vanishes (j == 0).
    let sections: IndexMap<String, Option<[f64; 3]>> =
        [("stepper_a", [0.0, 0.0, 0.0]),
         ("stepper_b", [10.0, 0.0, 0.0]),
         ("stepper_c", [20.0, 0.0, 0.0])]
        .into_iter()
        .map(|(k, a)| (k.to_string(), Some(a)))
        .collect();
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), 5.0);
    pos.insert("stepper_b".to_string(), 5.0);
    pos.insert("stepper_c".to_string(), 15.0);
    let expected = py
        .eval(
            "kin.calc_position({'stepper_a': 5.0, 'stepper_b': 5.0, 'stepper_c': 15.0})",
        )
        .expect_err("py must raise on collinear anchors");
    check_err(
        "collinear anchors",
        Err(expected),
        world.kin.calc_position(&pos),
    );
}

#[test]
fn calc_position_unreachable_point() {
    let Some(mut py) = oracle() else { return };
    let sections: IndexMap<String, Option<[f64; 3]>> =
        [("stepper_a", [0.0, 0.0, 0.0]),
         ("stepper_b", [100.0, 0.0, 0.0]),
         ("stepper_c", [0.0, 100.0, 0.0])]
        .into_iter()
        .map(|(k, a)| (k.to_string(), Some(a)))
        .collect();
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    // Distances too short to intersect: the final sqrt goes negative.
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), 1.0);
    pos.insert("stepper_b".to_string(), 1.0);
    pos.insert("stepper_c".to_string(), 1.0);
    let expected = py
        .eval(
            "kin.calc_position({'stepper_a': 1.0, 'stepper_b': 1.0, 'stepper_c': 1.0})",
        )
        .expect_err("py must raise math domain error");
    check_err(
        "unreachable point",
        Err(expected),
        world.kin.calc_position(&pos),
    );
}

#[test]
fn calc_position_huge_distance_overflow() {
    let Some(mut py) = oracle() else { return };
    // One anchor has a coordinate whose square overflows: the ** 2 in
    // `matrix_magsq` raises OverflowError before any division.
    let sections: IndexMap<String, Option<[f64; 3]>> =
        [("stepper_a", [0.0, 0.0, 0.0]),
         ("stepper_b", [1e200, 0.0, 0.0]),
         ("stepper_c", [0.0, 1e200, 0.0])]
        .into_iter()
        .map(|(k, a)| (k.to_string(), Some(a)))
        .collect();
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let mut pos = IndexMap::new();
    pos.insert("stepper_a".to_string(), 1.0);
    pos.insert("stepper_b".to_string(), 1.0);
    pos.insert("stepper_c".to_string(), 1.0);
    let expected = py
        .eval(
            "kin.calc_position({'stepper_a': 1.0, 'stepper_b': 1.0, 'stepper_c': 1.0})",
        )
        .expect_err("py must raise OverflowError");
    check_err(
        "huge distance",
        Err(expected),
        world.kin.calc_position(&pos),
    );
}

#[test]
fn set_position_updates_all_steppers() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(4);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    clear_logs(&mut py, &world.log);
    py.exec("kin.set_position([10.0, 20.0, 30.0], '')")
        .expect("py set_position");
    world
        .kin
        .set_position(vec![10.0, 20.0, 30.0], "")
        .expect("rust set_position");
    check_log(&mut py, "set_position log", &world.log);

}

#[test]
fn set_position_short_coord() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    let expected = py
        .eval("kin.set_position([1.0], '')")
        .expect_err("py must raise IndexError");
    check_err(
        "short coord",
        Err(expected),
        world.kin.set_position(vec![1.0], ""),
    );
}

#[test]
fn home_sets_axes_and_origin() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    clear_logs(&mut py, &world.log);
    py.exec("h = FakeHomingState(); kin.home(h)")
        .expect("py home");
    let homing: HomingStateHandle = Arc::new(FakeHomingState { log: world.log.clone() });
    world.kin.home(homing).expect("rust home");
    check_log(&mut py, "home log", &world.log);
}

#[test]
fn check_move_and_clear_homing_state_are_noops() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ))
    .expect("py build");
    let world = rust_world(sections);
    clear_logs(&mut py, &world.log);
    py.exec("kin.check_move(object()); kin.clear_homing_state('xyz')")
        .expect("py noops");
    world
        .kin
        .check_move(&DummyMove)
        .expect("rust check_move");
    world.kin.clear_homing_state("xyz");
    check_log(&mut py, "noop log", &world.log);
}

#[test]
fn missing_anchor_option_propagates() {
    let Some(mut py) = oracle() else { return };
    let mut sections = sections_n(3);
    sections.insert("stepper_b".to_string(), None);
    let _ = py.exec(&format!(
        "kin = make_kin({})",
        py_sections_expr(&sections)
    ));
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let factory = Arc::new(FakeFactory {
        log: log.clone(),
        made: Mutex::new(Vec::new()),
    });
    let toolhead: ToolHead = Arc::new(FakeToolHead {
        factory,
        _trapq: Arc::new(FakeTrapq),
    });
    let config: Config = Arc::new(FakeConfig::new(sections, log.clone()));
    let rust = WinchKinematics::new(&toolhead, &config);
    let py_err = py.eval("make_kin({'stepper_a': (0.0, 0.0, 0.0), 'stepper_b': None, 'stepper_c': (0.0, 0.0, 0.0)})");
    check_err("missing anchor option", py_err, rust);
}

#[test]
fn load_kinematics_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let sections = sections_n(3);
    py.exec(&format!(
        "kin2 = winch_mod.load_kinematics(FakeToolhead(), FakeConfig({}))",
        py_sections_expr(&sections)
    ))
    .expect("py load_kinematics");
    let log: Log = Arc::new(Mutex::new(Vec::new()));
    let factory = Arc::new(FakeFactory {
        log: log.clone(),
        made: Mutex::new(Vec::new()),
    });
    let toolhead: ToolHead = Arc::new(FakeToolHead {
        factory,
        _trapq: Arc::new(FakeTrapq),
    });
    let config: Config = Arc::new(FakeConfig::new(sections, log.clone()));
    let kin = load_kinematics(&toolhead, &config).expect("rust load_kinematics");
    let expected = py.eval("kin2.get_status(2.5)").expect("py status");
    check(
        "status via load_kinematics",
        Ok(expected),
        pyvalue_to_value(&kin.get_status(2.5)),
    );
}

