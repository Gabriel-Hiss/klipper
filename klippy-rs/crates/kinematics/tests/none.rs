//! Parity tests for `kinematics::none` (port of `klippy/kinematics/none.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `NoneKinematics` is built
//! on both sides with a minimal Python fake for the toolhead (only
//! `toolhead.Coord` is read by the reference) and the dummy `config` the
//! reference ignores, then `get_steppers`, `calc_position`, the no-op
//! methods and `get_status` are compared exactly (floats by bits via the
//! recording value format).
//!
//! One representational gap is bridged in the test: `calc_position`
//! returns `[0, 0, 0]` (Python ints) while the `KinematicsApi` contract
//! returns `Vec<Option<f64>>`, so the oracle's int list is asserted against
//! the int vector and the Rust return against `[Some(0.0), Some(0.0),
//! Some(0.0)]` (see `calc_position_is_zero`).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use kinematics::none::load_kinematics;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Rail, RailOpts, Stepper, StepperFactory, StepperFactoryApi, Trapq};
use klippy_api::toolhead::{
    Coord, ExtraAxis, HomingStateHandle, Kinematics, LookaheadCallback, MoveApi,
    PrinterExtruder, ToolHead, ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import gcode
import kinematics.none as none_mod

class FakeToolhead:
    Coord = gcode.Coord

kin = none_mod.load_kinematics(FakeToolhead(), None)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes (the port never calls any of these).
// ---------------------------------------------------------------------------

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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(&self, _name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
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
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "none".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by none kinematics")
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

struct FakeStepperFactory;

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper is not used by none kinematics")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail is not used by none kinematics")
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_multi_rail is not used by none kinematics")
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail is not used by none kinematics")
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance is not used by none kinematics")
    }
}

struct FakeToolhead;

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by none kinematics")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("toolhead set_position is not used by none kinematics")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by none kinematics")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by none kinematics")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        panic!("get_last_move_time is not used by none kinematics")
    }
    fn dwell(&self, _delay: f64) -> KResult<()> {
        panic!("dwell is not used by none kinematics")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by none kinematics")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by none kinematics")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by none kinematics")
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves is not used by none kinematics")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("toolhead get_status is not used by none kinematics")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by none kinematics")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by none kinematics")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by none kinematics")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by none kinematics")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by none kinematics")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by none kinematics")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by none kinematics")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by none kinematics")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by none kinematics")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by none kinematics")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by none kinematics")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by none kinematics")
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(FakeStepperFactory)
    }
}

struct FakeMove;

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        Coord(vec![0.0, 0.0, 0.0, 0.0])
    }
    fn end_pos(&self) -> Coord {
        Coord(vec![1.0, 2.0, 3.0, 0.0])
    }
    fn axes_d(&self) -> Coord {
        Coord(vec![1.0, 2.0, 3.0, 0.0])
    }
    fn axes_r(&self) -> Coord {
        Coord(vec![0.0, 0.0, 0.0, 0.0])
    }
    fn move_d(&self) -> f64 {
        0.0
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
    fn limit_speed(&self, _speed: f64, _accel: f64) -> KResult<()> { Ok(()) }
    fn move_error(&self, msg: String) -> KlippyError {
        KlippyError::command(msg)
    }
}

struct FakeHomingState;

impl klippy_api::toolhead::HomingStateApi for FakeHomingState {
    fn set_axes(&self, _axes: Vec<i64>) {}
    fn get_axes(&self) -> Vec<i64> {
        Vec::new()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        panic!("get_trigger_position is not used by none kinematics")
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        panic!("set_homed_position is not used by none kinematics")
    }
    fn home_rails(
        &self,
        _rails: Vec<Rail>,
        _forcepos: Vec<Option<f64>>,
        _movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        panic!("home_rails is not used by none kinematics")
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("none fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping none oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn fresh_kinematics() -> Kinematics {
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig { printer });
    let toolhead: ToolHead = Arc::new(FakeToolhead);
    load_kinematics(&toolhead, &config).expect("load_kinematics")
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "kinematics.none".to_string(),
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

fn pyvalue_to_rec(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_rec).collect()),
        PyValue::Tuple(l) => Value::Tuple(l.iter().map(pyvalue_to_rec).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, val)| (pyvalue_to_rec(k), pyvalue_to_rec(val)))
                .collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn get_steppers_is_empty() {
    let Some(mut py) = oracle() else { return };
    let kin = fresh_kinematics();
    check("get_steppers", py.eval("kin.get_steppers()"), Value::List(Vec::new()));
    assert!(kin.get_steppers().is_empty(), "none has no steppers");
}

#[test]
fn calc_position_is_zero() {
    let Some(mut py) = oracle() else { return };
    let kin = fresh_kinematics();
    // The reference returns the int list [0, 0, 0] for any input.
    check(
        "calc_position with entries",
        py.eval("kin.calc_position({'stepper_x': 5.0, 'stepper_y': -2.5, 'stepper_z': 0.125})"),
        Value::List(vec![Value::int(0), Value::int(0), Value::int(0)]),
    );
    check(
        "calc_position empty",
        py.eval("kin.calc_position({})"),
        Value::List(vec![Value::int(0), Value::int(0), Value::int(0)]),
    );
    // The contract returns Vec<Option<f64>>; int 0 maps to Some(0.0).
    let mut positions = indexmap::IndexMap::new();
    positions.insert("stepper_x".to_string(), 5.0);
    positions.insert("stepper_y".to_string(), -2.5);
    positions.insert("stepper_z".to_string(), 0.125);
    assert_eq!(
        kin.calc_position(&positions).expect("calc_position"),
        vec![Some(0.0), Some(0.0), Some(0.0)],
        "calc_position ignores its input"
    );
    assert_eq!(
        kin.calc_position(&indexmap::IndexMap::new())
            .expect("calc_position empty"),
        vec![Some(0.0), Some(0.0), Some(0.0)],
        "calc_position ignores its input"
    );
}

#[test]
fn noop_methods_return_none() {
    let Some(mut py) = oracle() else { return };
    let kin = fresh_kinematics();

    check(
        "set_position",
        py.eval("kin.set_position([1.0, 2.0, 3.0, 0.0], 'xyz')"),
        Value::Null,
    );
    kin.set_position(vec![1.0, 2.0, 3.0, 0.0], "xyz").expect("set_position");

    check("clear_homing_state", py.eval("kin.clear_homing_state('xy')"), Value::Null);
    kin.clear_homing_state("xy");

    check("home", py.eval("kin.home(object())"), Value::Null);
    kin.home(Arc::new(FakeHomingState) as HomingStateHandle).expect("home");

    check("check_move", py.eval("kin.check_move(object())"), Value::Null);
    kin.check_move(&FakeMove).expect("check_move");
}

#[test]
fn get_status_matches() {
    let Some(mut py) = oracle() else { return };
    let kin = fresh_kinematics();
    for eventtime in [0.0_f64, 123.5, -1.0] {
        check(
            &format!("get_status({eventtime})"),
            py.eval(&format!("kin.get_status({eventtime})")),
            pyvalue_to_rec(&kin.get_status(eventtime)),
        );
    }
    // The stored value is `gcode.Coord` `(0.0, 0.0, 0.0, 0)`: the fourth
    // element is the int padding zero (see the port's `get_status`).
    check(
        "axes_minmax padding shape",
        py.eval("kin.axes_minmax"),
        Value::Tuple(vec![
            Value::float(0.0),
            Value::float(0.0),
            Value::float(0.0),
            Value::int(0),
        ]),
    );
}

#[test]
fn get_calibration_is_missing() {
    let Some(mut py) = oracle() else { return };
    let kin = fresh_kinematics();
    // Only delta kinematics define `get_calibration`; both sides raise
    // AttributeError. The class matches; the message differs (Python names
    // the class, the contract uses a fixed phrasing), so it is checked
    // against the contract text, not the oracle text.
    let oracle_err = match py.eval("kin.get_calibration()") {
        Ok(v) => panic!("get_calibration must raise, got {v:?}"),
        Err(e) => e,
    };
    assert_eq!(oracle_err.class_name, "AttributeError", "oracle class");
    let err = match kin.get_calibration() {
        Ok(_) => panic!("get_calibration must raise"),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), "AttributeError", "rust class");
    assert_eq!(
        err.message(),
        "kinematics object has no attribute 'get_calibration'",
        "rust message"
    );
}
