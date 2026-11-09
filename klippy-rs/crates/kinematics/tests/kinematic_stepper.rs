//! Parity tests for `kinematics::kinematic_stepper` (port of
//! `klippy/kinematics/kinematic_stepper.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `KinematicStepper` is
//! built on both sides with minimal Python fakes for the `stepper` and
//! `chelper` modules (recording `setup_itersolve`,
//! `get_stepper_kinematics`, `set_position` and the chelper
//! `generic_cartesian_stepper_set_coeffs` calls) and the printer/config,
//! then fed identical `carriages` strings and update calls. Returned
//! values (coeffs, active axes, names, chelper coefficients) and raised
//! errors (class and message) are compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use indexmap::IndexMap;
use kinematics::kinematic_stepper::{module_factory, Carriage, CarriageApi, KinematicStepper};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Rail, RailOpts, Stepper, StepperFactoryApi,
    StepperKinematics, Trapq,
};
use klippy_api::ObjectRef;
use klippy_api::EventArg;
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of, KinAlloc};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import sys
import types

stepper_mod = types.ModuleType('stepper')
stepper_calls = []

class FakeStepperKinematics:
    pass

class FakeStepper:
    def __init__(self, config):
        self._name = config.get_name()
        self.sk = FakeStepperKinematics()
    def get_name(self, short=False):
        if short and self._name.startswith('stepper'):
            return self._name[8:]
        return self._name
    def setup_itersolve(self, alloc_func, *params):
        stepper_calls.append(('setup_itersolve', alloc_func, params))
    def get_stepper_kinematics(self):
        return self.sk
    def set_position(self, coord):
        stepper_calls.append(('set_position', tuple(coord)))

def PrinterStepper(config):
    return FakeStepper(config)

stepper_mod.PrinterStepper = PrinterStepper
sys.modules['stepper'] = stepper_mod

chelper_mod = types.ModuleType('chelper')
chelper_calls = []

class FakeFfiLib:
    def generic_cartesian_stepper_set_coeffs(self, sk, a_x, a_y, a_z):
        chelper_calls.append(('set_coeffs', a_x, a_y, a_z))

def _get_ffi():
    return (None, FakeFfiLib())

chelper_mod.get_ffi = _get_ffi
sys.modules['chelper'] = chelper_mod

import kinematics.kinematic_stepper as ks

class CommandError(Exception):
    pass

class ReportError(Exception):
    pass

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, carriages):
        self._printer = printer
        self._name = name
        self._carriages = carriages
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get(self, option):
        if option == 'carriages':
            return self._carriages
        raise KeyError(option)

add_calls = []

class FakeCarriage:
    def __init__(self, name, axis):
        self._name = name
        self._axis = axis
    def get_name(self):
        return self._name
    def get_axis(self):
        return self._axis
    def add_stepper(self, kin_stepper):
        add_calls.append(self._name)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
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
    fn lookup_object(
        &self,
        _name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
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
    name: String,
    carriages: Option<String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        if option == "carriages" {
            Ok(self.carriages.clone())
        } else {
            Err(KlippyError::Config(format!(
                "Option '{option}' is required"
            )))
        }
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
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
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
        panic!("getsection is not used by kinematic_stepper")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

/// Shared call log for the Rust fakes. A stepper entry is
/// `(kind, alloc_func, params)`; `params` are the `PyValue`s it was
/// called with.
#[derive(Default)]
struct FakeCalls {
    stepper: GCell<Vec<(String, String, Vec<PyValue>)>>,
    add_stepper: GCell<Vec<String>>,
}

impl FakeCalls {
    fn new() -> FakeCalls {
        FakeCalls::default()
    }
}

struct FakeStepper {
    name: String,
    calls: Arc<FakeCalls>,
    sk: GCell<Option<StepperKinematics>>,
}

impl FakeStepper {
    fn new(name: String, calls: Arc<FakeCalls>) -> FakeStepper {
        FakeStepper {
            name,
            calls,
            sk: GCell::new(None),
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by kinematic_stepper")
    }
    fn get_name(&self, short: Option<bool>) -> String {
        let name = &self.name;
        if short.unwrap_or(false) && name.starts_with("stepper") {
            return name[8..].to_string();
        }
        name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by kinematic_stepper")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians is not used by kinematic_stepper")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by kinematic_stepper")
    }
    fn setup_default_pulse_duration(&self, _a: f64, _b: bool) {
        panic!("setup_default_pulse_duration is not used by kinematic_stepper")
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        self.calls.stepper.borrow_mut().push((
            "setup_itersolve".to_string(),
            alloc_func.to_string(),
            params.to_vec(),
        ));
        self.sk.replace(Some(
            alloc_stepper_kinematics(alloc_func, params).expect("alloc stepper kinematics"),
        ));
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist is not used by kinematic_stepper")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by kinematic_stepper")
    }
    fn set_rotation_distance(&self, _d: f64) {
        panic!("set_rotation_distance is not used by kinematic_stepper")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by kinematic_stepper")
    }
    fn set_dir_inverted(&self, _b: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by kinematic_stepper")
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by kinematic_stepper")
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.calls.stepper.borrow_mut().push((
            "set_position".to_string(),
            String::new(),
            coord
                .iter()
                .map(|f| PyValue::Float(*f))
                .collect::<Vec<_>>(),
        ));
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by kinematic_stepper")
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        panic!("get_mcu_position is not used by kinematic_stepper")
    }
    fn get_past_mcu_position(&self, _t: f64) -> i64 {
        panic!("get_past_mcu_position is not used by kinematic_stepper")
    }
    fn mcu_to_commanded_position(&self, _p: i64) -> f64 {
        panic!("mcu_to_commanded_position is not used by kinematic_stepper")
    }
    fn dump_steps(&self, _a: i64, _b: i64, _c: i64) -> (Vec<PullHistoryStep>, i64) {
        panic!("dump_steps is not used by kinematic_stepper")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        let sk = self.sk.borrow().clone()?;
        let handle: StepperKinematics = sk;
        Some(handle)
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<StepperKinematics>,
    ) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics is not used by kinematic_stepper")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by kinematic_stepper")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq is not used by kinematic_stepper")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        panic!("set_trapq is not used by kinematic_stepper")
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        panic!("add_active_callback is not used by kinematic_stepper")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by kinematic_stepper")
    }
}

struct FakeFactory {
    calls: Arc<FakeCalls>,
    stepper: GCell<Option<Stepper>>,
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(
        &self,
        config: &Config,
        _units_in_radians: bool,
    ) -> KResult<Stepper> {
        let stepper: Stepper =
            Arc::new(FakeStepper::new(config.get_name(), self.calls.clone()));
        self.stepper.replace(Some(stepper.clone()));
        Ok(stepper)
    }
    fn lookup_rail(&self, _c: &Config, _o: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail is not used by kinematic_stepper")
    }
    fn lookup_multi_rail(&self, _c: &Config, _o: RailOpts) -> KResult<Rail> {
        panic!("lookup_multi_rail is not used by kinematic_stepper")
    }
    fn generic_rail(&self, _c: &Config, _o: RailOpts) -> KResult<Rail> {
        panic!("generic_rail is not used by kinematic_stepper")
    }
    fn parse_step_distance(
        &self,
        _c: &Config,
        _u: Option<bool>,
        _n: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance is not used by kinematic_stepper")
    }
}

struct FakeCarriage {
    name: String,
    axis: i64,
    calls: Arc<FakeCalls>,
}

impl CarriageApi for FakeCarriage {
    fn get_axis(&self) -> i64 {
        self.axis
    }
    fn add_stepper(&self, _kin_stepper: &KinematicStepper) -> KResult<()> {
        self.calls.add_stepper.borrow_mut().push(self.name.clone());
        Ok(())
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("kinematic_stepper fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping kinematic_stepper oracle tests: set KLIPPY_PYTHON to a \
                 CPython 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

const XYZS: &[(&str, i64)] = &[("x", 0), ("y", 1), ("z", 2)];

fn xyz_specs() -> Vec<(String, i64)> {
    XYZS.iter()
        .map(|(n, a)| (n.to_string(), *a))
        .collect()
}

/// The Python `carriages` dict in the oracle namespace.
fn py_carriages(py: &mut pyoracle::Oracle, specs: &[(String, i64)]) {
    let entries = specs
        .iter()
        .map(|(n, a)| format!("'{n}': FakeCarriage('{n}', {a})"))
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!("carriages = {{{entries}}}"))
        .expect("carriages dict");
}

/// Builds a fresh Python `ks_obj` in the oracle namespace.
fn py_pair(py: &mut pyoracle::Oracle, carriages_str: &str, specs: &[(String, i64)]) {
    py.exec("del stepper_calls[:]; del chelper_calls[:]; del add_calls[:]")
        .expect("clear calls");
    py_carriages(py, specs);
    py.exec(&format!(
        "p = FakePrinter(); c = FakeConfig(p, 'stepper_x', {carriages_str:?}); \
         ks_obj = ks.KinematicStepper(c, carriages)"
    ))
    .expect("py construction");
}

/// The Rust side under test, plus the shared fake call log.
struct Pair {
    stepper: KinematicStepper,
    calls: Arc<FakeCalls>,
}

fn rust_pair(carriages_str: Option<&str>, specs: &[(String, i64)]) -> KResult<Pair> {
    let calls = Arc::new(FakeCalls::new());
    let factory: Arc<dyn StepperFactoryApi> = Arc::new(FakeFactory {
        calls: calls.clone(),
        stepper: GCell::new(None),
    });
    let printer: Printer = Arc::new(FakePrinter);
    let config: Config = Arc::new(FakeConfig {
        printer,
        name: "stepper_x".to_string(),
        carriages: carriages_str.map(str::to_string),
    });
    let mut map: IndexMap<String, Carriage> = IndexMap::new();
    for (name, axis) in specs {
        let c: Carriage = Arc::new(FakeCarriage {
            name: name.clone(),
            axis: *axis,
            calls: calls.clone(),
        });
        map.insert(name.clone(), c);
    }
    KinematicStepper::new(&factory, &config, &map)
        .map(|stepper| Pair { stepper, calls })
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "kinematic_stepper".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust succeeded but call must fail"),
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

/// `Value` tuple of floats the oracle returns for `get_kin_coeffs()`, which
/// is a Python tuple. The recorder decodes a tuple as `Value::Tuple`, so
/// the actual value must be a `Tuple` too; `assert_equal` treats a `List`
/// as different from a `Tuple`.
fn floats(v: &[f64]) -> Value {
    Value::Tuple(v.iter().map(|f| Value::float(*f)).collect())
}

/// The `generic_cartesian_stepper_set_coeffs` calls the Rust side has made,
/// as `(set_coeffs, a_x, a_y, a_z)` tuples. The fake stepper's
/// `GenericCartesianStepper` is created by `setup_itersolve` and its
/// installed coefficients are what `KinematicStepper::update_kin_coeffs`
/// mutates through `set_coeffs`, so reading them back observes the last
/// (and, for these tests, only) push.
fn rust_chelper_calls(pair: &Pair) -> Value {
    let stepper = pair.stepper.get_stepper();
    let sk = stepper
        .get_stepper_kinematics()
        .expect("setup_itersolve installs the stepper kinematics");
    let c = kinematics_of(&sk)
        .expect("setup_itersolve must install a core handle")
        .with(|alloc| match alloc {
            KinAlloc::GenericCartesian(stepper) => [stepper.a.x, stepper.a.y, stepper.a.z],
            _ => panic!("setup_itersolve must install a GenericCartesian allocation"),
        });
    Value::List(vec![Value::Tuple(vec![
        Value::str("set_coeffs"),
        Value::float(c[0]),
        Value::float(c[1]),
        Value::float(c[2]),
    ])])
}

fn ints(v: &[i64]) -> Value {
    Value::List(v.iter().map(|i| Value::int(*i)).collect())
}

/// The recorded `setup_itersolve` calls, as `(alloc_func, params)` tuples.
fn itersolve_calls(calls: &FakeCalls) -> Value {
    let entries = calls.stepper.borrow();
    Value::List(
        entries
            .iter()
            .filter(|(kind, _, _)| kind == "setup_itersolve")
            .map(|(_, name, params)| {
                Value::Tuple(vec![
                    Value::str(name),
                    Value::List(
                        params
                            .iter()
                            .map(|p| match p {
                                PyValue::Float(f) => Value::float(*f),
                                other => panic!("unexpected param: {other:?}"),
                            })
                            .collect(),
                    ),
                ])
            })
            .collect(),
    )
}

/// The recorded `add_stepper` carriage names, in call order.
fn add_stepper_names(calls: &FakeCalls) -> Value {
    Value::List(
        calls
            .add_stepper
            .borrow()
            .iter()
            .map(|n| Value::str(n))
            .collect(),
    )
}

fn carriage_names(pair: &Pair) -> Value {
    Value::List(
        pair.stepper
            .get_carriages()
            .iter()
            .map(|c| {
                let f = c.as_any().downcast_ref::<FakeCarriage>().unwrap();
                Value::str(&f.name)
            })
            .collect(),
    )
}

/// The carriage map the Rust side was built with, rebuilt for updates.
fn carriage_map(specs: &[(String, i64)], calls: &Arc<FakeCalls>) -> IndexMap<String, Carriage> {
    let mut map: IndexMap<String, Carriage> = IndexMap::new();
    for (name, axis) in specs {
        let c: Carriage = Arc::new(FakeCarriage {
            name: name.clone(),
            axis: *axis,
            calls: calls.clone(),
        });
        map.insert(name.clone(), c);
    }
    map
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_no_entry_points() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn ctor_xyz_full_parity() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");

    check(
        "ctor_xyz coeffs",
        py.eval("ks_obj.get_kin_coeffs()"),
        floats(&pair.stepper.get_kin_coeffs()),
    );
    check(
        "ctor_xyz active",
        py.eval("ks_obj.get_active_axes()"),
        ints(&pair.stepper.get_active_axes()),
    );
    check(
        "ctor_xyz name",
        py.eval("ks_obj.get_name()"),
        Value::str(&pair.stepper.get_name(false)),
    );
    check(
        "ctor_xyz name_short",
        py.eval("ks_obj.get_name(True)"),
        Value::str(&pair.stepper.get_name(true)),
    );
    check(
        "ctor_xyz itersolve",
        py.eval(
            "[(alloc, list(params)) for kind, alloc, params in stepper_calls \
             if kind == 'setup_itersolve']",
        ),
        itersolve_calls(&pair.calls),
    );
    check(
        "ctor_xyz add_stepper",
        py.eval("add_calls"),
        add_stepper_names(&pair.calls),
    );
    check(
        "ctor_xyz carriages",
        py.eval("[c._name for c in ks_obj.get_carriages()]"),
        carriage_names(&pair),
    );
}

#[test]
fn ctor_scaled_and_signed_coeffs() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    for s in [
        "0.5*x-2*y+3*z", "-x", "+x", "2*x", "1e2*x", "-x+2*y-0.25*z", "inf*x", "z+y+x",
    ] {
        py_pair(&mut py, s, &specs);
        let pair = rust_pair(Some(s), &specs).expect("rust construction");
        check(
            &format!("scaled {s} coeffs"),
            py.eval("ks_obj.get_kin_coeffs()"),
            floats(&pair.stepper.get_kin_coeffs()),
        );
        check(
            &format!("scaled {s} active"),
            py.eval("ks_obj.get_active_axes()"),
            ints(&pair.stepper.get_active_axes()),
        );
        check(
            &format!("scaled {s} itersolve"),
            py.eval(
                "[(alloc, list(params)) for kind, alloc, params in stepper_calls \
                 if kind == 'setup_itersolve']",
            ),
            itersolve_calls(&pair.calls),
        );
        check(
            &format!("scaled {s} add_stepper"),
            py.eval("add_calls"),
            add_stepper_names(&pair.calls),
        );
    }
}

#[test]
fn ctor_whitespace_terms() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, " x+y", &specs);
    let pair = rust_pair(Some(" x+y"), &specs).expect("rust construction");
    check(
        "leading-space coeffs",
        py.eval("ks_obj.get_kin_coeffs()"),
        floats(&pair.stepper.get_kin_coeffs()),
    );
}

#[test]
fn ctor_error_cases() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_carriages(&mut py, &specs);
    for s in [
        "2*x*y*z", "q*x", "zz", "x+x", "x+x+y", "", "0*x", "0*x+0*y", "1.5x", "-", "x*y",
        "+", "x+", "x*", "*x", "x * y", "2*foo*bar",
    ] {
        let py_res = py.eval(&format!(
            "ks.KinematicStepper(FakeConfig(FakePrinter(), 'stepper_x', {s:?}), carriages)"
        ));
        let rust_res = rust_pair(Some(s), &specs).map(|_| ());
        check_err(&format!("ctor error {s:?}"), py_res, rust_res);
    }
}

#[test]
fn ctor_duplicate_axis_two_carriages() {
    let Some(mut py) = oracle() else { return };
    let specs = vec![
        ("a".to_string(), 0),
        ("b".to_string(), 0),
        ("c".to_string(), 1),
    ];
    py_carriages(&mut py, &specs);
    let py_res = py.eval(
        "ks.KinematicStepper(FakeConfig(FakePrinter(), 'stepper_x', 'a+b'), carriages)",
    );
    let rust_res = rust_pair(Some("a+b"), &specs).map(|_| ());
    check_err("duplicate axis a+b", py_res, rust_res);
}

#[test]
fn update_carriages_replaces_coeffs_and_chelper() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    let map = carriage_map(&specs, &pair.calls);
    py.exec("del chelper_calls[:]").expect("clear");
    py.exec("ks_obj.update_carriages('x+y', carriages, None)")
        .expect("py update");
    pair.stepper
        .update_carriages("x+y", &map, None)
        .expect("rust update");
    check(
        "update coeffs",
        py.eval("ks_obj.get_kin_coeffs()"),
        floats(&pair.stepper.get_kin_coeffs()),
    );
    check(
        "update carriages",
        py.eval("[c._name for c in ks_obj.get_carriages()]"),
        carriage_names(&pair),
    );
    check(
        "update chelper",
        py.eval("chelper_calls"),
        rust_chelper_calls(&pair),
    )
}

#[test]
fn update_carriages_empty_string_without_report_error() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    let map = carriage_map(&specs, &pair.calls);
    py.exec("del chelper_calls[:]").expect("clear");
    let py_res = py.eval("ks_obj.update_carriages('', carriages, None)");
    let rust_res = pair.stepper.update_carriages("", &map, None);
    match (&py_res, &rust_res) {
        (Ok(_), Ok(())) => {
            check(
                "update-empty coeffs",
                py.eval("ks_obj.get_kin_coeffs()"),
                floats(&pair.stepper.get_kin_coeffs()),
            );
            check(
                "update-empty chelper",
                py.eval("chelper_calls"),
                rust_chelper_calls(&pair),
            );
        }
        (Err(_), Err(_)) => check_err("update-empty", py_res, rust_res),
        _ => unreachable!(),
    }
}

#[test]
fn update_carriages_errors_are_command_errors() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    let map = carriage_map(&specs, &pair.calls);
    for s in ["zz", "q*x", "2*x*y"] {
        let py_res = py.eval(&format!("ks_obj.update_carriages({s:?}, carriages, None)"));
        let rust_res = pair.stepper.update_carriages(s, &map, None);
        check_err(&format!("update error {s:?}"), py_res, rust_res);
    }
    // With report_error=gcmd.error, parse errors are still CommandError.
    let py_res =
        py.eval("ks_obj.update_carriages('zz', carriages, FakePrinter.command_error)");
    let rust_res = pair
        .stepper
        .update_carriages("zz", &map, Some(&|msg: String| KlippyError::Command(msg)));
    check_err("update error with gcmd.error", py_res, rust_res);
    // Empty string with report_error -> the dedicated message.
    let py_res =
        py.eval("ks_obj.update_carriages('', carriages, FakePrinter.command_error)");
    let rust_res = pair
        .stepper
        .update_carriages("", &map, Some(&|msg: String| KlippyError::Command(msg)));
    check_err("update empty with gcmd.error", py_res, rust_res);
}

#[test]
fn update_carriages_uses_report_error_callable() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    let map = carriage_map(&specs, &pair.calls);
    let py_res = py.eval("ks_obj.update_carriages('zz', carriages, ReportError)");
    let rust_res = pair.stepper.update_carriages(
        "zz",
        &map,
        Some(&|msg: String| KlippyError::Other {
            class_name: "ReportError".to_string(),
            message: msg,
        }),
    );
    check_err("update error uses report_error", py_res, rust_res);
    let py_res =
        py.eval("ks_obj.update_carriages('', carriages, ReportError)");
    let rust_res = pair.stepper.update_carriages(
        "",
        &map,
        Some(&|msg: String| KlippyError::Other {
            class_name: "ReportError".to_string(),
            message: msg,
        }),
    );
    check_err(
        "update empty uses report_error",
        py_res,
        rust_res,
    );
}

#[test]
fn update_kin_coeffs_direct() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    py.exec("del chelper_calls[:]").expect("clear");
    py.exec("ks_obj.update_kin_coeffs([2.0, 3.0, 4.0])")
        .expect("py update coeffs");
    pair.stepper
        .update_kin_coeffs(vec![2.0, 3.0, 4.0])
        .expect("rust update coeffs");
    check(
        "update_kin_coeffs coeffs",
        py.eval("ks_obj.get_kin_coeffs()"),
        floats(&pair.stepper.get_kin_coeffs()),
    );
    check(
        "update_kin_coeffs active",
        py.eval("ks_obj.get_active_axes()"),
        ints(&pair.stepper.get_active_axes()),
    );
    check(
        "update_kin_coeffs chelper",
        py.eval("chelper_calls"),
        rust_chelper_calls(&pair),
    )
}

#[test]
fn update_kin_coeffs_short_list_is_index_error() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    let py_res = py.eval("ks_obj.update_kin_coeffs([1.0])");
    let rust_res = pair.stepper.update_kin_coeffs(vec![1.0]);
    check_err("update_kin_coeffs short", py_res, rust_res);
    // Python assigns kin_coeffs before the IndexError; the Rust port
    // does the same, so both sides now hold [1.0].
    check(
        "short coeffs state",
        py.eval("ks_obj.get_kin_coeffs()"),
        floats(&pair.stepper.get_kin_coeffs()),
    );
}

#[test]
fn set_position_forwards_coord() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    py.exec("del stepper_calls[:]").expect("clear");
    py.exec("ks_obj.set_position([1.0, 2.0, 3.0])")
        .expect("py set pos");
    pair.stepper.set_position([1.0, 2.0, 3.0]);
    check(
        "set_position calls",
        py.eval(
            "[list(entry[1]) for entry in stepper_calls if entry[0] == 'set_position']",
        ),
        Value::List(vec![Value::List(vec![
            Value::float(1.0),
            Value::float(2.0),
            Value::float(3.0),
        ])]),
    );
}

#[test]
fn get_stepper_name_matches() {
    let Some(mut py) = oracle() else { return };
    let specs = xyz_specs();
    py_pair(&mut py, "x+y+z", &specs);
    let pair = rust_pair(Some("x+y+z"), &specs).expect("rust construction");
    check(
        "stepper name",
        py.eval("ks_obj.get_stepper().get_name()"),
        Value::str(&pair.stepper.get_stepper().get_name(None)),
    );
    check(
        "stepper name short",
        py.eval("ks_obj.get_stepper().get_name(True)"),
        Value::str(&pair.stepper.get_stepper().get_name(Some(true))),
    );
}
