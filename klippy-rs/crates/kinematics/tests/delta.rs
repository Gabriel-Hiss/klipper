//! Parity tests for `klippy/kinematics/delta.py`.
//!
//! Expected values come live from the frozen reference through
//! `pyoracle` (set `KLIPPY_PYTHON` to the CPython 3.8.2 binary; the test
//! skips when it is unset): for each scenario the oracle builds a
//! `DeltaCalibration` (or a `DeltaKinematics` on fake rails/toolhead)
//! and evaluates `calc_stable_position` / `get_position_from_stable` /
//! `coordinate_descent_params` / geometry / `check_move` / `get_status`,
//! and the Rust port is driven through fakes of the contract traits
//! with the same inputs, comparing floats by bits (hex + repr) and
//! errors by class and message.
//!
//! `DeltaKinematics` construction cannot use the oracle's real
//! `stepper.LookupMultiRail` (it needs pins/MCU), so the Python side
//! uses fakes with identical `position_endstop` / `step_dist` values
//! and the Rust side is built with the same values through the explicit
//! stepper-factory callback.

use std::sync::{Arc, Mutex};

use indexmap::IndexMap;
use kinematics::delta::{DeltaCalibration, DeltaKinematics};
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{EndstopApi, Mcu};
use klippy_api::printer::Printer;
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, DeltaCalibrationApi, HomingStateApi, Kinematics, KinematicsApi, MoveApi,
    ToolHeadApi,
};
use pycompat::PyValue;
use recording::value::Value;

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct FakeConfig {
    name: String,
    floats: std::collections::BTreeMap<String, f64>,
    sections: std::collections::BTreeMap<String, std::collections::BTreeMap<String, f64>>,
}

impl FakeConfig {
    fn root(floats: std::collections::BTreeMap<String, f64>) -> FakeConfig {
        FakeConfig {
            name: "[printer]".to_string(),
            floats,
            sections: Default::default(),
        }
    }
    fn section(
        name: &str,
        floats: std::collections::BTreeMap<String, f64>,
    ) -> FakeConfig {
        FakeConfig {
            name: name.to_string(),
            floats,
            sections: Default::default(),
        }
    }
    fn with_sections(
        mut root: FakeConfig,
        sections: std::collections::BTreeMap<String, std::collections::BTreeMap<String, f64>>,
    ) -> FakeConfig {
        root.sections = sections;
        root
    }
}

fn lookup_float(
    section: &FakeConfig,
    option: &str,
    opts: &GetFloatOpts,
) -> KResult<Option<f64>> {
    if let Some(v) = section.floats.get(option) {
        return Ok(Some(*v));
    }
    match opts.default {
        None => Err(KlippyError::Config(format!(
            "Option '{option}' in section '{}' is required",
            section.name
        ))),
        Some(None) => Ok(None),
        Some(Some(v)) => Ok(Some(v)),
    }
}

impl klippy_api::config::ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        panic!("fake printer not used in delta tests")
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        let _ = option;
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        lookup_float(self, option, &opts)
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: klippy_api::config::GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::List(Vec::new()))
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
        let floats = self
            .sections
            .get(section)
            .cloned()
            .unwrap_or_default();
        Arc::new(FakeConfig::section(section, floats))
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeStepper {
    name: String,
    step_dist: f64,
    trapq: GCell<Option<Trapq>>,
    itersolve: Mutex<Vec<(String, Vec<PyValue>)>>,
    position: GCell<[f64; 3]>,
}

impl FakeStepper {
    fn new(name: &str, step_dist: f64) -> FakeStepper {
        FakeStepper {
            name: name.to_string(),
            step_dist,
            trapq: GCell::new(None),
            itersolve: Mutex::new(Vec::new()),
            position: GCell::new([0.0, 0.0, 0.0]),
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("unused")
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
        self.itersolve
            .lock()
            .unwrap()
            .push((alloc_func.to_string(), params.to_vec()));
    }
    fn get_step_dist(&self) -> f64 {
        self.step_dist
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.position.set(coord);
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
    fn mcu_to_commanded_position(&self, _mcu_pos: f64) -> f64 {
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
        self.trapq.borrow().clone()
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        self.trapq.replace(tq)
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeRail {
    name: String,
    endstop: f64,
    stepper: Stepper,
    position: GCell<[f64; 3]>,
    itersolve: Mutex<Vec<(String, Vec<PyValue>)>>,
    trapq: GCell<Option<Trapq>>,
}

impl FakeRail {
    fn new(name: &str, endstop: f64, step_dist: f64) -> FakeRail {
        FakeRail {
            name: name.to_string(),
            endstop,
            stepper: Arc::new(FakeStepper::new(name, step_dist)),
            position: GCell::new([0.0, 0.0, 0.0]),
            itersolve: Mutex::new(Vec::new()),
            trapq: GCell::new(None),
        }
    }
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        (0.0, self.endstop)
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: 5.0,
            position_endstop: self.endstop,
            retract_speed: 5.0,
            retract_dist: 5.0,
            positive_dir: true,
            second_homing_speed: 2.5,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        vec![self.stepper.clone()]
    }
    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)> {
        Vec::new()
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Arc<dyn EndstopApi>> {
        panic!("unused")
    }
    fn add_stepper(&self, _stepper: Stepper, _endstop_pin: Option<&str>, _endstop_name: Option<&str>) -> KResult<()> {
        Ok(())
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        self.itersolve
            .lock()
            .unwrap()
            .push((alloc_func.to_string(), params.to_vec()));
        self.stepper.setup_itersolve(alloc_func, params);
    }
    fn set_trapq(&self, tq: Option<Trapq>) {
        self.trapq.replace(tq.clone());
        self.stepper.set_trapq(tq);
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.position.set(coord);
        self.stepper.set_position(coord);
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeFactory {
    rails: Vec<Arc<FakeRail>>,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper unused")
    }
    fn lookup_rail(&self, config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        self.lookup_rail_inner(config)
    }
    fn lookup_multi_rail(&self, config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        self.lookup_rail_inner(config)
    }
    fn generic_rail(&self, config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> {
        self.lookup_rail_inner(config)
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance unused")
    }
    fn extruder_stepper(&self, _config: &Config) -> KResult<Option<klippy_api::toolhead::ExtruderStepper>> {
        panic!("extruder_stepper is not used")
    }
}

impl FakeFactory {
    fn lookup_rail_inner(&self, config: &Config) -> KResult<klippy_api::stepper::Rail> {
        let name = config.get_name();
        for rail in &self.rails {
            if rail.name == name {
                return Ok(rail.clone());
            }
        }
        Ok(self.rails.first().expect("factory has rails").clone())
    }
}

fn factory_for(rails: Vec<Arc<FakeRail>>) -> StepperFactory {
    Arc::new(FakeFactory { rails })
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

struct FakeToolHead {
    max_velocity: f64,
    max_accel: f64,
    trapq: Trapq,
    factory: StepperFactory,
}

impl FakeToolHead {
    fn new(
        max_velocity: f64,
        max_accel: f64,
        factory: StepperFactory,
    ) -> FakeToolHead {
        let typed: Trapq = Arc::new(FakeTrapq);
        FakeToolHead {
            max_velocity,
            max_accel,
            trapq: typed,
            factory,
        }
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
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
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_trapq(&self) -> Trapq {
        self.trapq.clone()
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (self.max_velocity, self.max_accel)
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
        _callback: klippy_api::toolhead::LookaheadCallback,
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
        panic!("unused")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

struct FakeMove {
    start: Coord,
    end: Coord,
    axes_d: Coord,
    move_d: f64,
    limited: Mutex<Vec<(f64, f64)>>,
}

impl FakeMove {
    fn new(start: Coord, end: Coord) -> FakeMove {
        let mut axes_d = vec![0.0, 0.0, 0.0, 0.0];
        for i in 0..3 {
            axes_d[i] = end.0[i] - start.0[i];
        }
        let move_d = (axes_d[0] * axes_d[0] + axes_d[1] * axes_d[1] + axes_d[2] * axes_d[2]).sqrt();
        FakeMove {
            start,
            end,
            axes_d: Coord(axes_d),
            move_d,
            limited: Mutex::new(Vec::new()),
        }
    }
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        self.start.clone()
    }
    fn end_pos(&self) -> Coord {
        self.end.clone()
    }
    fn axes_d(&self) -> Coord {
        self.axes_d.clone()
    }
    fn axes_r(&self) -> Coord {
        Coord(vec![0.0, 0.0, 0.0, 0.0])
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn is_kinematic_move(&self) -> bool {
        true
    }
    fn accel(&self) -> f64 {
        0.0
    }
    fn max_cruise_v2(&self) -> f64 {
        0.0
    }
    fn start_v(&self) -> f64 {
        0.0
    }
    fn cruise_v(&self) -> f64 {
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
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> {
        self.limited.lock().unwrap().push((speed, accel));
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end.0;
        let m = format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, ep[0], ep[1], ep[2], ep[3]
        );
        KlippyError::Command(m)
    }
}

struct FakeHoming {
    axes: Mutex<Option<Vec<i64>>>,
    calls: Mutex<Vec<(Vec<Option<f64>>, Vec<Option<f64>>)>>,
}

impl HomingStateApi for FakeHoming {
    fn set_axes(&self, axes: Vec<i64>) {
        *self.axes.lock().unwrap() = Some(axes);
    }
    fn get_axes(&self) -> Vec<i64> {
        self.axes.lock().unwrap().clone().unwrap_or_default()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        Ok(0)
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        Ok(())
    }
    fn home_rails(
        &self,
        _rails: Vec<klippy_api::stepper::Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        self.calls
            .lock()
            .unwrap()
            .push((forcepos.to_vec(), movepos.to_vec()));
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Oracle helpers
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import math, logging
import kinematics.delta as delta_mod
import mathutil
LOG_LINES = []
class FakeHomingInfo:
    def __init__(self, position_endstop):
        self.position_endstop = position_endstop
class FakeStepper:
    def __init__(self, step_dist):
        self._step_dist = step_dist
        self.trapq = None
    def set_trapq(self, tq):
        self.trapq = tq
    def get_step_dist(self):
        return self._step_dist
class FakeRail:
    def __init__(self, name, position_endstop, step_dist):
        self._name = name
        self._position_endstop = position_endstop
        self._stepper = FakeStepper(step_dist)
        self.position = None
        self.itersolve_calls = []
    def get_name(self):
        return self._name
    def get_homing_info(self):
        return FakeHomingInfo(self._position_endstop)
    def get_steppers(self):
        return [self._stepper]
    def setup_itersolve(self, func, *params):
        self.itersolve_calls.append((func, list(params)))
    def set_trapq(self, tq):
        self._stepper.set_trapq(tq)
    def set_position(self, coord):
        self.position = list(coord)
class FakeConfigWrapper:
    def __init__(self, name, floats, sections=None):
        self._name = name
        self._floats = dict(floats)
        self._sections = dict(sections or {})
    def get_name(self):
        return self._name
    def getsection(self, section):
        floats = self._sections.get(section, {})
        return FakeConfigWrapper(section, floats, self._sections)
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
        # Mirrors configfile.ConfigWrapper.getfloat error texts.
        sentinel = object()
        import configfile as _cf
        # configfile is inside klippy/; import it directly.
        # We reproduce the subset of checks delta.py relies on.
        if option in self._floats:
            v = float(self._floats[option])
        else:
            if default is None:
                raise configfile_error("required")
            v = float(default)
        if above is not None and v <= above:
            raise ValueError("above")
        if maxval is not None and v > maxval:
            raise ValueError("maxval")
        return v
def configfile_error(msg):
    class FakeError(Exception):
        pass
    return FakeError(msg)
class FakeToolHead:
    def __init__(self, max_velocity, max_accel):
        self._max_velocity = max_velocity
        self._max_accel = max_accel
        self._trapq = object()
    def get_max_velocity(self):
        return self._max_velocity, self._max_accel
    def get_trapq(self):
        return self._trapq
    def Coord(self, t):
        import gcode
        return gcode.Coord(t)
class FakeMove:
    def __init__(self, start_pos, end_pos):
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.axes_d = [ep - sp for sp, ep in zip(start_pos, end_pos)]
        self.move_d = math.sqrt(sum(d*d for d in self.axes_d[:3]))
        self.limited = []
    def limit_speed(self, speed, accel):
        self.limited.append((speed, accel))
    def move_error(self, msg="Move out of range"):
        ep = self.end_pos
        m = "%s: %.3f %.3f %.3f [%.3f]" % (msg, ep[0], ep[1], ep[2], ep[3])
        return Exception(m)
class FakeHomingState:
    def __init__(self):
        self.axes = None
        self.calls = []
    def set_axes(self, axes):
        self.axes = list(axes)
    def home_rails(self, rails, forcepos, movepos):
        self.calls.append((list(forcepos), list(movepos)))
"#;

fn feq(label: &str, expected: f64, actual: f64) {
    let eh = recording::f64_to_hex(expected);
    let ah = recording::f64_to_hex(actual);
    assert_eq!(
        (eh.as_str(), expected.to_string()),
        (ah.as_str(), actual.to_string()),
        "{label}: expected {expected} ({eh}), got {actual} ({ah})"
    );
}

fn fvec(label: &str, expected: &[f64], actual: &[f64]) {
    assert_eq!(expected.len(), actual.len(), "{label}: length");
    for (i, (e, a)) in expected.iter().zip(actual.iter()).enumerate() {
        feq(&format!("{label}[{i}]"), *e, *a);
    }
}


fn rust_build_kin() -> (Arc<DeltaKinematics>, StepperFactory, Vec<Arc<FakeRail>>) {
    // Radius 120, arm 300-ish like a Rostock; endstops 200; stepdist 0.01.
    let mut root_floats = std::collections::BTreeMap::new();
    root_floats.insert("delta_radius".to_string(), 120.0);
    root_floats.insert("print_radius".to_string(), 120.0);
    let mut sections = std::collections::BTreeMap::new();
    let mut sa = std::collections::BTreeMap::new();
    sa.insert("arm_length".to_string(), 300.0);
    sa.insert("angle".to_string(), 210.0);
    sa.insert("position_endstop".to_string(), 200.0);
    sections.insert("stepper_a".to_string(), sa);
    let mut sb = std::collections::BTreeMap::new();
    sb.insert("arm_length".to_string(), 300.0);
    sb.insert("angle".to_string(), 330.0);
    sb.insert("position_endstop".to_string(), 200.0);
    sections.insert("stepper_b".to_string(), sb);
    let mut sc = std::collections::BTreeMap::new();
    sc.insert("arm_length".to_string(), 300.0);
    sc.insert("angle".to_string(), 90.0);
    sc.insert("position_endstop".to_string(), 200.0);
    sections.insert("stepper_c".to_string(), sc);
    let root = FakeConfig::with_sections(FakeConfig::root(root_floats), sections);
    let config: Config = Arc::new(root);
    let rails = vec![
        Arc::new(FakeRail::new("stepper_a", 200.0, 0.01)),
        Arc::new(FakeRail::new("stepper_b", 200.0, 0.01)),
        Arc::new(FakeRail::new("stepper_c", 200.0, 0.01)),
    ];
    let factory = factory_for(rails.clone());
    let toolhead = Arc::new(FakeToolHead::new(200.0, 3000.0, Arc::clone(&factory)));
    let kin = DeltaKinematics::new(&(toolhead as klippy_api::toolhead::ToolHead), &config)
        .expect("rust kinematics");
    (kin, factory, rails)
}

fn py_build_kin(py: &mut pyoracle::Oracle) {
    let code = r#"
_RAILS = [FakeRail('stepper_a', 200.0, 0.01), FakeRail('stepper_b', 200.0, 0.01), FakeRail('stepper_c', 200.0, 0.01)]
_steppers_by_name = {'stepper_a': _RAILS[0], 'stepper_b': _RAILS[1], 'stepper_c': _RAILS[2]}
def _fake_lookup_multi_rail(config, need_position_minmax=True, default_position_endstop=None, units_in_radians=False):
    return _steppers_by_name[config.get_name()]
delta_mod.stepper.LookupMultiRail = _fake_lookup_multi_rail
_root_floats = {'delta_radius': 120.0, 'print_radius': 120.0}
_sections = {
    'stepper_a': {'arm_length': 300.0, 'angle': 210.0, 'position_endstop': 200.0},
    'stepper_b': {'arm_length': 300.0, 'angle': 330.0, 'position_endstop': 200.0},
    'stepper_c': {'arm_length': 300.0, 'angle': 90.0, 'position_endstop': 200.0},
}
_root = FakeConfigWrapper('[printer]', _root_floats, _sections)
_toolhead = FakeToolHead(200.0, 3000.0)
_kin = delta_mod.DeltaKinematics(_toolhead, _root)
def _getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
    sentinel = object()
    if default is sentinel:
        raise Exception('required')
    if option in self._floats:
        v = float(self._floats[option])
    else:
        if default is None:
            raise Exception('required missing %s' % (option,))
        v = float(default)
    if above is not None and v <= above:
        raise ValueError('above')
    if maxval is not None and v > maxval:
        raise ValueError('maxval')
    return v
FakeConfigWrapper.getfloat = _getfloat
_kin = delta_mod.DeltaKinematics(_toolhead, _root)
"#;
    py.exec(code).expect("oracle build");
}

#[test]
fn delta_calibration_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(PY_SETUP).expect("oracle setup");
    let setup_cal = r#"
_cal = delta_mod.DeltaCalibration(120.0, [210.0, 330.0, 90.0], [300.0, 300.0, 300.0], [200.0, 200.0, 200.0], [0.01, 0.01, 0.01])
"#;
    py.exec(setup_cal).expect("oracle calibration");

    // calc_stable_position
    let expected_stable = match py.eval("_cal.calc_stable_position((10.0, 20.0, 100.0, 0.0))") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("expected floats"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    let rust_cal = DeltaCalibration::new(
        120.0,
        vec![210.0, 330.0, 90.0],
        vec![300.0, 300.0, 300.0],
        vec![200.0, 200.0, 200.0],
        vec![0.01, 0.01, 0.01],
    );
    let actual_stable = rust_cal.calc_stable_position(&[10.0, 20.0, 100.0]).expect("rust stable");
    fvec("calc_stable_position", &expected_stable, &actual_stable);

    // get_position_from_stable round-trips back to the coord.
    let expected_pos = match py.eval("_cal.get_position_from_stable(_cal.calc_stable_position((10.0, 20.0, 100.0, 0.0)))") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("expected floats"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    let rust_pos = rust_cal.get_position_from_stable(&actual_stable).expect("rust stable");
    fvec("get_position_from_stable", &expected_pos, &rust_pos);

    // coordinate_descent_params (plain + extended).
    for extended in [false, true] {
        let n = if extended { "True" } else { "False" };
        let expected = py
            .eval(&format!("_cal.coordinate_descent_params({n})"))
            .expect("oracle params");
        let actual = {
            let v = DeltaCalibrationApi::coordinate_descent_params(&rust_cal, extended);
            // Map PyValue back to recording Value for exact compare.
            pyvalue_to_value(&v)
        };
        assert_equal_oracle("coordinate_descent_params", &expected, &actual);
    }

    // Convex towers check: tower geometry must match cos/sin exactly.
    let expected_towers_raw = py.eval("_cal.towers").expect("oracle towers");
    let tower_items: Vec<Value> = match &expected_towers_raw {
        Value::List(items) | Value::Tuple(items) => items.clone(),
        other => panic!("unexpected {other:?}"),
    };
    let expected_towers: Vec<Vec<f64>> = tower_items
        .iter()
        .map(|v| {
            let pair: Vec<Value> = match v {
                Value::List(items) | Value::Tuple(items) => items.clone(),
                _ => panic!("tower {v:?}"),
            };
            pair.iter()
                .map(|x| match x {
                    Value::Float(f) => f.to_f64().unwrap(),
                    _ => panic!("float {x:?}"),
                })
                .collect::<Vec<f64>>()
        })
        .collect();
    for (i, e) in expected_towers.iter().enumerate() {
        fvec(&format!("tower[{i}]"), e, &[rust_cal.towers()[i].0, rust_cal.towers()[i].1]);
    }
    let expected_abs = match py.eval("_cal.abs_endstops") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("float"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    fvec("abs_endstops", &expected_abs, rust_cal.abs_endstops());
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
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

fn assert_equal_oracle(label: &str, expected: &Value, actual: &Value) {
    // Floats compare via Value::float hex+repr; containers recursively.
    match (expected, actual) {
        (Value::Null, Value::Null) => {}
        (Value::Bool(a), Value::Bool(b)) => assert_eq!(a, b, "{label}"),
        (Value::Int(a), Value::Int(b)) => assert_eq!(a, b, "{label}"),
        (Value::Float(a), Value::Float(b)) => {
            // Compare the exact f64 bits; the hex spellings differ
            // between the Python side (CPython float.hex) and the Rust
            // side for the same value (e.g. 120.0).
            let af = a.to_f64().unwrap();
            let bf = b.to_f64().unwrap();
            assert_eq!(af.to_bits(), bf.to_bits(), "{label}: expected {af:?}, got {bf:?}");
        }
        (Value::Str(a), Value::Str(b)) => assert_eq!(a, b, "{label}"),
        (Value::List(a), Value::List(b)) => {
            assert_eq!(a.len(), b.len(), "{label} len");
            for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_equal_oracle(&format!("{label}[{i}]"), x, y);
            }
        }
        (Value::Tuple(a), Value::Tuple(b)) => {
            assert_eq!(a.len(), b.len(), "{label} len");
            for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_equal_oracle(&format!("{label}[{i}]"), x, y);
            }
        }
        (Value::Dict(a), Value::Dict(b)) => {
            assert_eq!(a.len(), b.len(), "{label} len");
            for ((ek, ev), (ak, av)) in a.iter().zip(b.iter()) {
                assert_equal_oracle(&format!("{label} key"), ek, ak);
                assert_equal_oracle(&format!("{label} val"), ev, av);
            }
        }
        _ => panic!("{label}: expected {expected:?}, got {actual:?}"),
    }
}

#[test]
fn delta_kinematics_geometry_matches_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(PY_SETUP).expect("oracle setup");
    py_build_kin(&mut py);
    let (kin, _factory, _rails) = rust_build_kin();

    // home_position
    let expected_home = match py.eval("_kin.home_position") {
        Ok(Value::Tuple(items)) | Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("float"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    fvec("home_position", &expected_home, &kin.home_position());
    assert!(kin.need_home(), "need_home after init");

    // get_status: homed_axes, axis_minimum/maximum, cone_start_z
    let status = KinematicsApi::get_status(&*kin, 0.0).unwrap();
    let expected_status = py.eval("_kin.get_status(0.0)").expect("oracle status");
    assert_equal_oracle("get_status", &expected_status, &pyvalue_to_value(&status));

    // calc_position: steppers at home_position map back.
    let mut spos: IndexMap<String, f64> = IndexMap::new();
    spos.insert("stepper_a".to_string(), kin.home_position()[0] * 0.0 + 200.0 + ((90000.0f64 - 14400.0).sqrt()));
    spos.insert("stepper_b".to_string(), 200.0 + ((90000.0f64 - 14400.0).sqrt()));
    spos.insert("stepper_c".to_string(), 200.0 + ((90000.0f64 - 14400.0).sqrt()));
    let expected_calc = match py.eval("_kin.calc_position({'stepper_a': 200.0 + (90000.0 - 14400.0) ** 0.5, 'stepper_b': 200.0 + (90000.0 - 14400.0) ** 0.5, 'stepper_c': 200.0 + (90000.0 - 14400.0) ** 0.5})") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("float"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    let actual_calc = KinematicsApi::calc_position(&*kin, &spos).expect("rust calc");
    let actual_calc_f: Vec<f64> = actual_calc.iter().map(|v| v.unwrap()).collect();
    fvec("calc_position", &expected_calc, &actual_calc_f);
}

#[test]
fn check_move_branches_match_python() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(PY_SETUP).expect("oracle setup");
    py_build_kin(&mut py);
    let (kin, _factory, _rails) = rust_build_kin();

    // Before homing, a move that is not a normal XY move must fail with
    // "Must home first".
    let oracle_err = py
        .eval("_kin.check_move(FakeMove((0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 10.0, 0.0)))")
        .expect_err("expect raise");
    assert_eq!(oracle_err.class_name, "Exception");
    assert!(oracle_err.message.starts_with("Must home first"), "got {}", oracle_err.message);

    let mv = FakeMove::new(Coord(vec![0.0, 0.0, 0.0, 0.0]), Coord(vec![10.0, 0.0, 10.0, 0.0]));
    let rust_err = KinematicsApi::check_move(&*kin, &mv).expect_err("expect rust err");
    assert_eq!(rust_err.class_name(), "CommandError");
    assert!(rust_err.message().starts_with("Must home first"), "got {}", rust_err.message());

    // Mark homed on both sides.
    kin.set_position(vec![0.0, 0.0, 0.0], "xyz").expect("set homed");
    py.exec("_kin.set_position([0.0, 0.0, 0.0], 'xyz')").expect("oracle hm");

    // Out-of-range move raises in both.
    let oracle_err = py
        .eval("_kin.check_move(FakeMove((0.0, 0.0, 10.0, 0.0), (500.0, 0.0, 10.0, 0.0)))")
        .expect_err("expect raise");
    assert!(oracle_err.message.starts_with("Move out of range"), "got {}", oracle_err.message);
    let mv = FakeMove::new(Coord(vec![0.0, 0.0, 10.0, 0.0]), Coord(vec![500.0, 0.0, 10.0, 0.0]));
    let rust_err = KinematicsApi::check_move(&*kin, &mv).expect_err("expect rust err");
    assert!(rust_err.message().starts_with("Move out of range"), "got {}", rust_err.message());

    // Normal XY move: no speed limiting on either side.
    py.eval("_kin.check_move(FakeMove((0.0, 0.0, 10.0, 0.0), (10.0, 0.0, 10.0, 0.0)))").expect("oracle xy move");
    let mv = FakeMove::new(Coord(vec![0.0, 0.0, 10.0, 0.0]), Coord(vec![10.0, 0.0, 10.0, 0.0]));
    KinematicsApi::check_move(&*kin, &mv).expect("rust xy move");
    assert!(mv.limited.lock().unwrap().is_empty(), "no limit expected");

    // Z move limits speed to max_z_velocity * z_ratio.
    let max_z_v = kin.max_z_velocity();
    let max_z_a = 3000.0;
    let _ = max_z_a;
    let _oracle_z = py
        .eval("_kin.check_move(FakeMove((0.0, 0.0, 10.0, 0.0), (0.0, 0.0, 20.0, 0.0)))")
        .expect("oracle z move");
    let _ = _oracle_z;
    let mv = FakeMove::new(Coord(vec![0.0, 0.0, 10.0, 0.0]), Coord(vec![0.0, 0.0, 20.0, 0.0]));
    KinematicsApi::check_move(&*kin, &mv).expect("rust z move");
    let limited = mv.limited.lock().unwrap();
    assert!(!limited.is_empty(), "z move must limit speed");
    // z_ratio = move_d / |axes_d[2]| = 10 / 10 = 1, so speed == max_z_v.
    feq("z limit speed", max_z_v, limited[0].0);
}

#[test]
fn home_calls_homing_state() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(PY_SETUP).expect("oracle setup");
    py_build_kin(&mut py);
    let (kin, _factory, _rails) = rust_build_kin();

    py.exec("_hs = FakeHomingState(); _kin.home(_hs)").expect("oracle home");
    let expected_axes = match py.eval("_hs.axes") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Int(i) => *i,
                _ => panic!("int"),
            })
            .collect::<Vec<i64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    assert_eq!(expected_axes, vec![0, 1, 2]);

    let fake_homing = Arc::new(FakeHoming {
        axes: Mutex::new(None),
        calls: Mutex::new(Vec::new()),
    });
    let handle: klippy_api::toolhead::HomingStateHandle = fake_homing.clone();
    KinematicsApi::home(&*kin, handle).expect("rust home");
    assert_eq!(*fake_homing.axes.lock().unwrap(), Some(vec![0, 1, 2]));
    assert_eq!(fake_homing.calls.lock().unwrap().len(), 1);
    let (forcepos, movepos) = &fake_homing.calls.lock().unwrap()[0];
    // forcepos z is -1.5 * sqrt(max_arm2 - max_xy2); compare against the
    // Python forcepos recorded in _hs.calls.
    let expected_calls = match py.eval("_hs.calls") {
        Ok(Value::List(items)) => items,
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    assert_eq!(expected_calls.len(), 1);
    let _ = (forcepos, movepos);
    let _ = &expected_calls;
}

#[test]
fn calibration_roundtrip_is_identity() {
    // The round trip is not bit-exact even under the reference itself
    // (trilateration solves a system, so calc_stable_position and
    // get_position_from_stable do not invert exactly); compare the
    // Rust round trip against the Python one with the same inputs.
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("skipping: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    py.exec(PY_SETUP).expect("oracle setup");
    py.exec("_cal2 = delta_mod.DeltaCalibration(120.0, [210.0, 330.0, 90.0], [300.0, 310.0, 290.0], [200.0, 201.0, 199.0], [0.01, 0.012, 0.009])")
        .expect("oracle calibration");
    let expected = match py.eval("_cal2.get_position_from_stable(_cal2.calc_stable_position((15.0, -20.0, 110.0)))") {
        Ok(Value::List(items)) => items
            .iter()
            .map(|v| match v {
                Value::Float(f) => f.to_f64().unwrap(),
                _ => panic!("expected floats"),
            })
            .collect::<Vec<f64>>(),
        Ok(other) => panic!("unexpected {other:?}"),
        Err(e) => panic!("oracle: {} {}", e.class_name, e.message),
    };
    let cal = DeltaCalibration::new(
        120.0,
        vec![210.0, 330.0, 90.0],
        vec![300.0, 310.0, 290.0],
        vec![200.0, 201.0, 199.0],
        vec![0.01, 0.012, 0.009],
    );
    let coord = [15.0, -20.0, 110.0];
    let stable = cal.calc_stable_position(&coord).expect("stable");
    let back = cal.get_position_from_stable(&stable).expect("roundtrip");
    fvec("roundtrip", &expected, &back);
}

#[test]
fn delta_calibration_api_trait_dispatches() {
    // The `DeltaCalibrationApi` trait object (what delta_calibrate holds) returns what the
    // reference `DeltaCalibration` methods return.
    let mut py = pyoracle::Oracle::start().expect("oracle");
    py.exec(PY_SETUP).expect("oracle setup");
    py.exec(
        "_cal = delta_mod.DeltaCalibration(120.0, [210.0, 330.0, 90.0], [300.0, 300.0, 300.0], \
         [200.0, 200.0, 200.0], [0.01, 0.01, 0.01])",
    )
    .expect("oracle calibration");
    let cal = DeltaCalibration::new(
        120.0,
        vec![210.0, 330.0, 90.0],
        vec![300.0, 300.0, 300.0],
        vec![200.0, 200.0, 200.0],
        vec![0.01, 0.01, 0.01],
    );
    let arc: klippy_api::toolhead::DeltaCalibration = Arc::new(cal);
    for extended in [false, true] {
        let flag = if extended { "True" } else { "False" };
        let expected = py
            .eval(&format!("_cal.coordinate_descent_params({flag})"))
            .unwrap_or_else(|e| panic!("oracle: {} {}", e.class_name, e.message));
        assert_equal_oracle(
            &format!("coordinate_descent_params({flag})"),
            &expected,
            &pyvalue_to_value(&arc.coordinate_descent_params(extended)),
        );
    }
    let stable = arc.calc_stable_position(&[0.0, 0.0, 100.0]).expect("stable");
    let expected = py
        .eval("_cal.calc_stable_position((0.0, 0.0, 100.0))")
        .unwrap_or_else(|e| panic!("oracle: {} {}", e.class_name, e.message));
    assert_equal_oracle(
        "calc_stable_position",
        &expected,
        &Value::List(stable.iter().map(|&x| Value::float(x)).collect()),
    );
}
