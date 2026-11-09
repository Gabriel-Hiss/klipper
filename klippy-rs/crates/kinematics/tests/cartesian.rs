//! Parity tests for `klippy/kinematics/cartesian.py` (`CartKinematics`).
//!
//! Every expected value comes from the frozen reference through `pyoracle`
//! (`$KLIPPY_PYTHON`); the test skips with a message outside the oracle
//! image. The Rust port is driven through fakes of the contract traits
//! (`PrinterApi`, `ConfigApi`, `ToolHeadApi`, `StepperFactoryApi`,
//! `RailApi`, `McuStepperApi`, `MoveApi`, `HomingStateApi`) with the same
//! inputs given to the Python fakes. Comparisons cover returned values
//! (floats by hex + repr), raised errors (class and message), `get_status`
//! dicts, config reads, `setup_itersolve` calls, `set_trapq` assignment,
//! `set_position`/`home_rails`/`limit_speed` side effects, and the init
//! sequence for with/without `[dual_carriage]`.
//!
//! The dual-carriage success path through `load_kinematics` is not driven
//! here: constructing `idex_modes.DualCarriages` needs the `idex_modes`
//! module (still a stub) plus its `chelper`/`gcode` collaborators. The
//! dual-carriage config reads (their error paths fail before the
//! constructor) and all dual-carriage behaviours through
//! `CartKinematics::new` with a fake dual-carriage object are covered, and
//! the python side installs a fake `DualCarriages` the same way.
//!
//! Run: `cargo test -p kinematics --test cartesian`.

use std::sync::Arc;

use kos::sync::Mutex;

use indexmap::IndexMap;
use kinematics::cartesian::{load_kinematics, CartKinematics};
use kinematics::idex_modes::{
    DualCarriagesApi, DualCarriagesRail, SavedDualCarriageState,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::Mcu;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, ExtraAxis, ExtraAxisApi, ExtruderStepper, HomingStateApi, HomingStateHandle,
    HomingTargetApi,
    KinematicsApi, MoveApi, PrinterExtruder, PrinterExtruderApi, ToolHead, ToolHeadApi,
};
use pycompat::PyValue;
use reactor::{Completion, Reactor};

// ---------------------------------------------------------------------------
// Conversion helpers
// ---------------------------------------------------------------------------

fn pyvalue_to_rec(v: &PyValue) -> recording::Value {
    match v {
        PyValue::None => recording::Value::Null,
        PyValue::Bool(b) => recording::Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => recording::Value::Int(i),
                Err(_) => recording::Value::BigInt(s),
            }
        }
        PyValue::Float(f) => recording::Value::float(*f),
        PyValue::Str(s) => recording::Value::Str(s.clone()),
        PyValue::Bytes(b) => recording::Value::Bytes(b.clone()),
        PyValue::List(l) => recording::Value::List(l.iter().map(pyvalue_to_rec).collect()),
        PyValue::Tuple(t) => recording::Value::Tuple(t.iter().map(pyvalue_to_rec).collect()),
        PyValue::Dict(d) => recording::Value::Dict(
            d.iter()
                .map(|(k, val)| (pyvalue_to_rec(k), pyvalue_to_rec(val)))
                .collect(),
        ),
        PyValue::Host(_) => recording::Value::Null,
    }
}



fn opts_floats_to_rec(v: &[Option<f64>]) -> recording::Value {
    recording::Value::List(
        v.iter()
            .map(|o| match o {
                Some(f) => recording::Value::float(*f),
                None => recording::Value::Null,
            })
            .collect(),
    )
}

fn check_value(label: &str, expected: recording::Value, actual: recording::Value) {
    let dummy = recording::Record {
        v: 1,
        unit: "kinematics.cartesian".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: Some(expected.clone()),
        exc: None,
        side_effects: recording::SideEffects::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    };
    if let Err(d) = recording::assert_equal(&dummy, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_error(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: Result<(), KlippyError>,
) {
    match (expected, actual) {
        (Ok(v), Ok(())) => panic!("{label}: oracle returned {v:?}, port returned Ok"),
        (Ok(v), Err(e)) => panic!(
            "{label}: oracle returned {v:?}, port raised {}: {}",
            e.class_name(),
            e.message()
        ),
        (Err(exc), Ok(())) => panic!(
            "{label}: oracle raised {}: {}, port returned Ok",
            exc.class_name, exc.message
        ),
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception type");
            assert_eq!(e.message(), exc.message, "{label} exception message");
        }
    }
}

/// Like `check_error` but compares only the exception type. Used where the
/// contract vocabulary deliberately differs from the Python class text
/// (`get_calibration` is absent from `CartKinematics`; the v2 trait default
/// phrases the `AttributeError` differently than CPython does).
fn check_error_class(label: &str, expected: pyoracle::OracleResult, actual: Result<(), KlippyError>) {
    match (expected, actual) {
        (Err(exc), Err(e)) => {
            assert_eq!(e.class_name(), exc.class_name, "{label} exception type");
        }
        (other, other_actual) => panic!("{label}: expected oracle error, got {other:?} vs {other_actual:?}"),
    }
}

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: Mutex::new(IndexMap::new()),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        panic!("get_reactor not used by cartesian fakes")
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut guard = self.objects.lock();
        if guard.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already exists"
            )));
        }
        guard.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let guard = self.objects.lock();
        match guard.get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Printer object '{name}' not found"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let guard = self.objects.lock();
        match module {
            None => guard
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
            Some(m) => guard
                .iter()
                .filter(|(k, _)| k.as_str() == m || k.starts_with(&format!("{m} ")))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
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
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

type SectionStore = Arc<Mutex<IndexMap<String, IndexMap<String, String>>>>;

fn store_from(specs: &[(&str, &[(&str, &str)])]) -> SectionStore {
    let mut map = IndexMap::new();
    for (section, opts) in specs {
        let mut inner = IndexMap::new();
        for (k, v) in *opts {
            inner.insert(k.to_string(), v.to_string());
        }
        map.insert(section.to_string(), inner);
    }
    Arc::new(Mutex::new(map))
}

struct FakeConfig {
    printer: Printer,
    section: String,
    store: SectionStore,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, store: SectionStore) -> Arc<FakeConfig> {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            store,
        })
    }
    fn raw(&self, option: &str) -> Option<String> {
        let guard = self.store.lock();
        guard.get(&self.section)?.get(option).cloned()
    }
    fn missing<T>(
        &self,
        option: &str,
        default: Option<Option<T>>,
    ) -> KResult<Option<T>> {
        match default {
            None => Err(KlippyError::Config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
            Some(opt) => Ok(opt),
        }
    }
    fn bound_str(v: f64) -> String {
        pycompat::text::float_str(v)
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
            None => self.missing(option, opts.default),
            Some(v) => Ok(Some(v)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            None => self.missing(option, opts.default),
            Some(raw) => {
                let v = raw
                    .trim()
                    .parse::<i64>()
                    .map_err(|_| KlippyError::Config(format!("Unable to parse option '{option}' in section '{}'", self.section)))?;
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {min}",
                            self.section
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {max}",
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
            None => self.missing(option, opts.default),
            Some(raw) => {
                let v = pycompat::text::float_from_string(raw.trim())
                    .map_err(|_| KlippyError::Config(format!("Unable to parse option '{option}' in section '{}'", self.section)))?;
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {}",
                            self.section,
                            Self::bound_str(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {}",
                            self.section,
                            Self::bound_str(max)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be above {}",
                            self.section,
                            Self::bound_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be below {}",
                            self.section,
                            Self::bound_str(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            None => self.missing(option, opts.default),
            Some(raw) => {
                let low = raw.trim().to_ascii_lowercase();
                let v = match low.as_str() {
                    "true" | "1" | "yes" | "on" => true,
                    "false" | "0" | "no" | "off" => false,
                    _ => {
                        return Err(KlippyError::Config(format!(
                            "Unable to parse option '{option}' in section '{}'",
                            self.section
                        )))
                    }
                };
                Ok(Some(v))
            }
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let value_map: IndexMap<String, PyValue> = match choices {
            PyValue::List(items) => items
                .iter()
                .map(|item| (item.str(), item.clone()))
                .collect(),
            PyValue::Dict(d) => {
                let mut m = IndexMap::new();
                for (k, v) in d {
                    m.insert(k.str(), v.clone());
                }
                m
            }
            other => {
                return Err(KlippyError::Config(format!(
                    "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                    other.str(),
                    self.section
                )))
            }
        };
        let choice_key = match self.raw(option) {
            None => match opts.default {
                None => {
                    return Err(KlippyError::Config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    )))
                }
                Some(None) => {
                    return Err(KlippyError::Config(format!(
                        "Choice 'None' for option '{option}' in section '{}' is not a valid choice",
                        self.section
                    )))
                }
                Some(Some(v)) => {
                    let key = v.str();
                    return Ok(value_map.get(&key).cloned().unwrap_or(v));
                }
            },
            Some(v) => v,
        };
        match value_map.get(&choice_key) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::Config(format!(
                "Choice '{choice_key}' for option '{option}' in section '{}' is not a valid choice",
                self.section
            ))),
        }
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match self.raw(option) {
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
                Some(opt) => Ok(opt.unwrap_or(PyValue::Tuple(Vec::new()))),
            },
            Some(raw) => {
                let parts: Vec<PyValue> = raw
                    .split(',')
                    .map(|p| p.trim().to_string())
                    .filter(|p| !p.is_empty())
                    .map(PyValue::Str)
                    .collect();
                Ok(PyValue::Tuple(parts))
            }
        }
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.raw(option) {
            None => self.missing(option, opts.default),
            Some(raw) => Ok(Some(
                raw.split(',')
                    .map(|p| p.trim().to_string())
                    .filter(|p| !p.is_empty())
                    .collect(),
            )),
        }
    }
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        match self.raw(option) {
            None => self.missing(option, opts.default),
            Some(raw) => Ok(Some(
                raw.split(',')
                    .filter(|p| !p.trim().is_empty())
                    .map(|p| p.trim().parse::<i64>().unwrap_or(0))
                    .collect(),
            )),
        }
    }
    fn getfloatlist(
        &self,
        option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        match self.raw(option) {
            None => self.missing(option, opts.default),
            Some(raw) => Ok(Some(
                raw.split(',')
                    .filter(|p| !p.trim().is_empty())
                    .map(|p| p.trim().parse::<f64>().unwrap_or(0.0))
                    .collect(),
            )),
        }
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            store: self.store.clone(),
        }) as Config
    }
    fn has_section(&self, section: &str) -> bool {
        let guard = self.store.lock();
        guard.contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        let guard = self.store.lock();
        guard
            .keys()
            .filter(|s| s.starts_with(prefix))
            .map(|s| Arc::new(FakeConfig {
                printer: self.printer.clone(),
                section: s.clone(),
                store: self.store.clone(),
            }) as Config)
            .collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> Vec<String> {
        let guard = self.store.lock();
        if let Some(inner) = guard.get(&self.section) {
            inner
                .keys()
                .filter(|k| k.starts_with(prefix))
                .cloned()
                .collect()
        } else {
            Vec::new()
        }
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct DummyExtruder;

impl ExtraAxisApi for DummyExtruder {
    fn get_name(&self) -> String {
        "extruder".to_string()
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
    fn calc_junction(&self, _prev_move: &dyn MoveApi, _mv: &dyn MoveApi, _ea_index: usize) -> f64 {
        0.0
    }
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for DummyExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::Command("Extruder not configured".to_string()))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<ExtruderStepper> {
        None
    }
}

struct FakeToolHead {
    max_velocity: f64,
    max_accel: f64,
    trapq: Trapq,
    stepper_factory: Arc<FakeStepperFactory>,
}

impl FakeToolHead {
    fn new(
        max_velocity: f64,
        max_accel: f64,
        stepper_factory: Arc<FakeStepperFactory>,
    ) -> Arc<FakeToolHead> {
        Arc::new(FakeToolHead {
            max_velocity,
            max_accel,
            trapq: Arc::new(FakeTrapq),
            stepper_factory,
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
        Ok(Vec::new())
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        Arc::new(DummyKinematics)
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
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
        (self.max_velocity, self.max_accel, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _callback: klippy_api::toolhead::LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
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
        self.stepper_factory.clone()
    }
}

struct FakeStepperFactory {
    rails: Mutex<IndexMap<String, Rail>>,
    lookups: Mutex<Vec<(String, RailOpts)>>,
}

impl FakeStepperFactory {
    fn new(entries: Vec<(String, Rail)>) -> Arc<FakeStepperFactory> {
        let mut m = IndexMap::new();
        for (k, v) in entries {
            m.insert(k, v);
        }
        Arc::new(FakeStepperFactory {
            rails: Mutex::new(m),
            lookups: Mutex::new(Vec::new()),
        })
    }
    fn lookups(&self) -> Vec<(String, RailOpts)> {
        self.lookups.lock().clone()
    }
}

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper not used by cartesian fakes")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail not used by cartesian fakes")
    }
    fn lookup_multi_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail> {
        let name = config.get_name();
        self.lookups.lock().push((name.clone(), opts));
        self.rails
            .lock()
            .get(&name)
            .cloned()
            .ok_or_else(|| KlippyError::Config(format!("no fake rail for section '{name}'")))
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail not used by cartesian fakes")
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance not used by cartesian fakes")
    }
}

struct FakeStepper {
    name: String,
    trapq_sets: Mutex<Vec<bool>>,
}

impl FakeStepper {
    fn new(name: &str) -> Arc<FakeStepper> {
        Arc::new(FakeStepper {
            name: name.to_string(),
            trapq_sets: Mutex::new(Vec::new()),
        })
    }
    fn did_trapq(&self) -> bool {
        !self.trapq_sets.lock().is_empty()
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu not used by cartesian fakes")
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
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        1.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (1.0, 1.0)
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
        sk: Option<StepperKinematics>,
    ) -> Option<StepperKinematics> {
        sk
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        self.trapq_sets.lock().push(tq.is_some());
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeRail {
    name: String,
    range: (f64, f64),
    homing_info: HomingInfo,
    stepper_fakes: Vec<Arc<FakeStepper>>,
    itersolve_calls: Mutex<Vec<(String, Vec<PyValue>)>>,
    set_position_calls: Mutex<Vec<[f64; 3]>>,
    set_trapq_calls: Mutex<Vec<Option<Trapq>>>,
}

impl FakeRail {
    fn new(
        name: &str,
        range: (f64, f64),
        homing_info: HomingInfo,
        stepper_names: &[&str],
    ) -> Arc<FakeRail> {
        let stepper_fakes: Vec<Arc<FakeStepper>> = stepper_names
            .iter()
            .map(|n| FakeStepper::new(n))
            .collect();
        Arc::new(FakeRail {
            name: name.to_string(),
            range,
            homing_info,
            stepper_fakes,
            itersolve_calls: Mutex::new(Vec::new()),
            set_position_calls: Mutex::new(Vec::new()),
            set_trapq_calls: Mutex::new(Vec::new()),
        })
    }
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        self.range
    }
    fn get_homing_info(&self) -> HomingInfo {
        self.homing_info
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.stepper_fakes
            .iter()
            .map(|s| s.clone() as Stepper)
            .collect()
    }
    fn get_endstops(
        &self,
    ) -> Vec<(Arc<dyn klippy_api::mcu::EndstopApi>, String)> {
        Vec::new()
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Arc<dyn klippy_api::mcu::EndstopApi>> {
        Err(KlippyError::Config(
            "lookup_endstop not used by cartesian fakes".to_string(),
        ))
    }
    fn add_stepper(
        &self,
        _stepper: Stepper,
        _endstop_pin: Option<&str>,
        _endstop_name: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        self.itersolve_calls
            .lock()
            .push((alloc_func.to_string(), params.to_vec()));
    }
    fn set_trapq(&self, tq: Option<Trapq>) {
        self.set_trapq_calls.lock().push(tq);
    }
    fn set_position(&self, coord: [f64; 3]) {
        self.set_position_calls.lock().push(coord);
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeMove {
    start_pos: Coord,
    end_pos: Coord,
    axes_d: Coord,
    axes_r: Coord,
    move_d_val: f64,
    limit_speed_calls: Mutex<Vec<(f64, f64)>>,
}

impl FakeMove {
    fn new(
        end_pos: [f64; 4],
        axes_d: [f64; 4],
        move_d: f64,
    ) -> Arc<FakeMove> {
        Arc::new(FakeMove {
            start_pos: Coord(vec![0.0, 0.0, 0.0, 0.0]),
            end_pos: Coord(end_pos.to_vec()),
            axes_d: Coord(axes_d.to_vec()),
            axes_r: Coord(vec![0.0, 0.0, 0.0, 0.0]),
            move_d_val: move_d,
            limit_speed_calls: Mutex::new(Vec::new()),
        })
    }
    fn limit_speed_calls(&self) -> Vec<(f64, f64)> {
        self.limit_speed_calls.lock().clone()
    }
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        self.start_pos.clone()
    }
    fn end_pos(&self) -> Coord {
        self.end_pos.clone()
    }
    fn axes_d(&self) -> Coord {
        self.axes_d.clone()
    }
    fn axes_r(&self) -> Coord {
        self.axes_r.clone()
    }
    fn move_d(&self) -> f64 {
        self.move_d_val
    }
    fn is_kinematic_move(&self) -> bool {
        false
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
        self.limit_speed_calls.lock().push((speed, accel));
        Ok(())
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos.0;
        let formatted = pycompat::text::percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                pycompat::text::PctValue::Str(msg),
                pycompat::text::PctValue::Float(ep[0]),
                pycompat::text::PctValue::Float(ep[1]),
                pycompat::text::PctValue::Float(ep[2]),
                pycompat::text::PctValue::Float(ep[3]),
            ],
        )
        .expect("move_error format");
        KlippyError::command(formatted)
    }
}

struct FakeHomingState {
    axes: Mutex<Vec<i64>>,
    home_rails_calls: Mutex<Vec<(Vec<String>, Vec<Option<f64>>, Vec<Option<f64>>)>>,
}

impl FakeHomingState {
    fn new(axes: Vec<i64>) -> Arc<FakeHomingState> {
        Arc::new(FakeHomingState {
            axes: Mutex::new(axes),
            home_rails_calls: Mutex::new(Vec::new()),
        })
    }
    fn calls(&self) -> Vec<(Vec<String>, Vec<Option<f64>>, Vec<Option<f64>>)> {
        self.home_rails_calls.lock().clone()
    }
}

impl HomingStateApi for FakeHomingState {
    fn set_axes(&self, axes: Vec<i64>) {
        *self.axes.lock() = axes;
    }
    fn get_axes(&self) -> Vec<i64> {
        self.axes.lock().clone()
    }
    fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: PyValue::Str(stepper_name.to_string()).repr(),
        })
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        Ok(())
    }
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        let names: Vec<String> = rails.iter().map(|r| r.get_name(None)).collect();
        self.home_rails_calls
            .lock()
            .push((names, forcepos, movepos));
        Ok(())
    }
}

struct FakeDualCarriage {
    primary: Mutex<Option<Rail>>,
    axis: i64,
    home_calls: Mutex<Vec<i64>>,
}

impl FakeDualCarriage {
    fn new(primary: Option<Rail>, axis: i64) -> Arc<FakeDualCarriage> {
        Arc::new(FakeDualCarriage {
            primary: Mutex::new(primary),
            axis,
            home_calls: Mutex::new(Vec::new()),
        })
    }
}

impl DualCarriagesApi for FakeDualCarriage {
    fn get_axes(&self) -> Vec<i64> {
        vec![self.axis]
    }
    fn get_primary_rail(&self, axis: i64) -> Option<Rail> {
        if axis != self.axis {
            return None;
        }
        self.primary.lock().clone()
    }
    fn get_dc_rail_wrapper(&self, _rail: &Rail) -> Option<Arc<DualCarriagesRail>> {
        None
    }
    fn get_transform(&self, _rail: &Rail) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn is_active(&self, _rail: &Rail) -> bool {
        false
    }
    fn home(&self, _homing_state: HomingStateHandle, axis: i64) -> KResult<()> {
        self.home_calls.lock().push(axis);
        Ok(())
    }
    fn save_dual_carriage_state(&self) -> KResult<SavedDualCarriageState> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "FakeDualCarriage does not track dual carriage state".to_string(),
        })
    }
    fn restore_dual_carriage_state(
        &self,
        _saved_state: &SavedDualCarriageState,
        _move_flag: i64,
        _move_speed: f64,
    ) -> KResult<()> {
        Err(KlippyError::Internal {
            class_name: "NotImplementedError".to_string(),
            message: "FakeDualCarriage does not track dual carriage state".to_string(),
        })
    }
}

// ---------------------------------------------------------------------------
// Shared builders (Rust + mirrors of the Python fakes below)
// ---------------------------------------------------------------------------

fn homing_info(
    speed: f64,
    position_endstop: f64,
    retract_speed: f64,
    retract_dist: f64,
    positive_dir: bool,
    second: f64,
) -> HomingInfo {
    HomingInfo {
        speed,
        position_endstop,
        retract_speed,
        retract_dist,
        positive_dir,
        second_homing_speed: second,
    }
}

struct World {
    config: Config,
    toolhead: ToolHead,
    stepper_factory: Arc<FakeStepperFactory>,
    rails: Vec<Arc<FakeRail>>,
}

fn build_world(
    printer: Printer,
    store: SectionStore,
    max_velocity: f64,
    max_accel: f64,
    rail_specs: Vec<(&str, &str, (f64, f64), HomingInfo, Vec<&str>)>,
) -> World {
    let config = FakeConfig::new(printer.clone(), "printer", store.clone()) as Config;
    let mut factory_entries = Vec::new();
    let mut rails = Vec::new();
    for (section, name, range, hi, steppers) in &rail_specs {
        let rail = FakeRail::new(name, *range, *hi, steppers);
        factory_entries.push((section.to_string(), rail.clone() as Rail));
        rails.push(rail);
    }
    let stepper_factory = FakeStepperFactory::new(factory_entries);
    let toolhead = FakeToolHead::new(max_velocity, max_accel, stepper_factory.clone()) as ToolHead;
    World {
        config,
        toolhead,
        stepper_factory,
        rails,
    }
}

/// Build a `CartKinematics` around the given rails and a fake dual-carriage
/// object, the way `load_kinematics` assembles it from a `[dual_carriage]`
/// config (the real `DualCarriages` constructor is covered separately).
#[allow(clippy::too_many_arguments)]
fn build_dc_kinematics(
    printer: Printer,
    rails: &[Arc<FakeRail>],
    dc_axis: i64,
    dc_primary: Option<Rail>,
) -> (Arc<CartKinematics>, Arc<FakeDualCarriage>) {
    let dyn_rails: Vec<Rail> = rails.iter().map(|r| r.clone() as Rail).collect();
    let ranges: Vec<(f64, f64)> = rails.iter().map(|r| r.get_range()).collect();
    let axes_min = Coord(vec![ranges[0].0, ranges[1].0, ranges[2].0, 0.0]);
    let axes_max = Coord(vec![ranges[0].1, ranges[1].1, ranges[2].1, 0.0]);
    let dc = FakeDualCarriage::new(dc_primary, dc_axis);
    let dc_module: Option<Arc<dyn DualCarriagesApi>> = Some(dc.clone());
    let kin = CartKinematics::new(
        printer,
        dyn_rails,
        Some(dc_axis),
        dc_module,
        axes_min,
        axes_max,
        50.0,
        500.0,
    );
    (kin, dc)
}

// ---------------------------------------------------------------------------
// Python fakes (executed in the oracle namespace)
// ---------------------------------------------------------------------------

const PY_FAKES: &str = r#"
import math, collections
import configparser

class CommandError(Exception):
    pass

class Error(configparser.Error):
    pass

class FakePrinter:
    def __init__(self):
        self.objects = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        try:
            return self.objects[name]
        except KeyError:
            if default is None:
                return None
            raise Error(name)
    def config_error(self, msg):
        return Error(msg)
    def command_error(self, msg):
        return CommandError(msg)

class FakeConfig:
    def __init__(self, printer, store, section):
        self.printer_obj = printer
        self.store = store  # {section: {opt: str}}
        self.section = section
    def get_printer(self):
        return self.printer_obj
    def get_name(self):
        return self.section
    def _missing(self, option, default, sentinel):
        if default is sentinel:
            raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return default
    def has_section(self, section):
        return section in self.store
    def getsection(self, section):
        return FakeConfig(self.printer_obj, self.store, section)
    def get(self, option, default=object(), note_valid=True):
        try:
            return self.store[self.section][option]
        except KeyError:
            return self._missing(option, default, FakeConfig._sentinel)
    def getfloat(self, option, default=object(), minval=None, maxval=None, above=None, below=None, note_valid=True):
        sentinel = FakeConfig._sentinel
        if self.section not in self.store or option not in self.store[self.section]:
            return self._missing(option, default, sentinel)
        raw = self.store[self.section][option]
        try:
            v = float(raw)
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v
    def getchoice(self, option, choices, default=object(), note_valid=True):
        sentinel = FakeConfig._sentinel
        if isinstance(choices, list):
            choices = {i: i for i in choices}
        if self.section not in self.store or option not in self.store[self.section]:
            return self._missing(option, default, sentinel)
        c = self.store[self.section][option]
        if c not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.section))
        return choices[c]
FakeConfig._sentinel = object()

class FakeStepper:
    def __init__(self, name):
        self.name = name
        self.trapq_sets = []
    def get_name(self, short=False):
        return self.name
    def set_trapq(self, tq):
        self.trapq_sets.append(tq)

class FakeRail:
    def __init__(self, name, pos_min, pos_max, hi):
        self.name = name
        self.position_min = pos_min
        self.position_max = pos_max
        self._hi = hi  # tuple(speed, position_endstop, retract_speed, retract_dist, positive_dir, second)
        self.steppers = [FakeStepper(name)]
        self.itersolve_calls = []
        self.set_position_calls = []
        self.set_trapq_calls = []
    def get_name(self, short=False):
        return self.name
    def get_range(self):
        return (self.position_min, self.position_max)
    def get_homing_info(self):
        return collections.namedtuple('homing_info', ['speed','position_endstop','retract_speed','retract_dist','positive_dir','second_homing_speed'])(*self._hi)
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return []
    def setup_itersolve(self, alloc_func, *params):
        self.itersolve_calls.append((alloc_func, params))
    def set_trapq(self, tq):
        for s in self.steppers:
            s.set_trapq(tq)
    def set_position(self, coord):
        self.set_position_calls.append(coord)

class FakeToolhead:
    def __init__(self, max_velocity, max_accel):
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.trapq = object()
    def get_max_velocity(self):
        return (self.max_velocity, self.max_accel)
    def get_trapq(self):
        return self.trapq

class FakeCoord(tuple):
    def __new__(cls, t):
        t = tuple(t)
        if len(t) < 4:
            t = t + (0,) * (4 - len(t))
        return super().__new__(cls, t)
    @property
    def x(self): return self[0]
    @property
    def y(self): return self[1]
    @property
    def z(self): return self[2]
    @property
    def e(self): return self[3]

class FakeMove:
    def __init__(self, end_pos, axes_d, move_d):
        self.end_pos = FakeCoord(end_pos)
        self.axes_d = FakeCoord(axes_d)
        self.move_d = move_d
        self.limit_speed_calls = []
    def limit_speed(self, speed, accel):
        self.limit_speed_calls.append((speed, accel))
    def move_error(self, msg='Move out of range'):
        ep = self.end_pos
        m = '%s: %.3f %.3f %.3f [%.3f]' % (msg, ep[0], ep[1], ep[2], ep[3])
        return CommandError(m)

class FakeHoming:
    def __init__(self, axes):
        self.axes = axes
        self.home_rails_calls = []
    def get_axes(self):
        return list(self.axes)
    def home_rails(self, rails, forcepos, movepos):
        names = [r.get_name() for r in rails]
        self.home_rails_calls.append((names, list(forcepos), list(movepos)))

class FakeDC:
    def __init__(self, primary_map, axis):
        self.primary_map = primary_map  # axis -> rail or None
        self.axis = axis
        self.home_calls = []
        self.create_calls = []
    def get_primary_rail(self, axis):
        return self.primary_map.get(axis)
    def home(self, homing_state, axis):
        self.home_calls.append(axis)

_rail_registry = {}
_dc_holder = {'dc': None}

_orig_lookup = None
_orig_dc = None
def install_fake_rails(rail_by_section):
    global _rail_registry, _orig_lookup, _orig_dc
    import stepper as _stepper
    _rail_registry = dict(rail_by_section)
    if _orig_lookup is None:
        _orig_lookup = _stepper.LookupMultiRail
    def fake_lookup(config, *a, **kw):
        return _rail_registry[config.get_name()]
    _stepper.LookupMultiRail = fake_lookup
    import kinematics.idex_modes as _idex
    if _orig_dc is None:
        _orig_dc = _idex.DualCarriages
    def fake_dc(printer, primary_rails, dual_rails, axes, safe_dist):
        holder = FakeDC({axes[0]: primary_rails[0]}, axes[0])
        holder.create_calls.append((primary_rails, dual_rails, axes, safe_dist))
        _dc_holder['dc'] = holder
        return holder
    _idex.DualCarriages = fake_dc

def install_fake_dc(mode):
    # mode: 'primary' (default above), 'dual' (dual rail is primary), 'none'
    import kinematics.idex_modes as _idex2
    def fake_dc_dual(printer, primary_rails, dual_rails, axes, safe_dist):
        holder = FakeDC({axes[0]: dual_rails[0]}, axes[0])
        holder.create_calls.append((primary_rails, dual_rails, axes, safe_dist))
        _dc_holder['dc'] = holder
        return holder
    def fake_dc_none(printer, primary_rails, dual_rails, axes, safe_dist):
        holder = FakeDC({axes[0]: None}, axes[0])
        holder.create_calls.append((primary_rails, dual_rails, axes, safe_dist))
        _dc_holder['dc'] = holder
        return holder
    if mode == 'dual':
        _idex2.DualCarriages = fake_dc_dual
    elif mode == 'none':
        _idex2.DualCarriages = fake_dc_none
    else:
        raise ValueError('unknown dc mode')
"#;

fn py_setup(py: &mut pyoracle::Oracle) {
    py.exec(PY_FAKES).expect("py fakes exec");
    py.exec(
        "import stepper, kinematics.cartesian, kinematics.idex_modes",
    )
    .expect("py imports");
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn init_defaults_and_explicit_limits() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping init_defaults_and_explicit_limits: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let printer = FakePrinter::new() as Printer;
    let store = store_from(&[
        ("printer", &[("max_z_velocity", "50"), ("max_z_accel", "500")]),
        ("stepper_x", &[("endstop_pin", "PA1")]),
        ("stepper_y", &[("endstop_pin", "PA2")]),
        ("stepper_z", &[("endstop_pin", "PA3")]),
    ]);
    let world = build_world(
        printer.clone(),
        store,
        200.0,
        1000.0,
        vec![
            ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
            ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
            ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
        ],
    );
    let kin = load_kinematics(&world.toolhead, &world.config).expect("rust init");

    // Rail lookups happened in order with default opts.
    let lookups = world.stepper_factory.lookups();
    assert_eq!(
        lookups
            .iter()
            .map(|(name, _)| name.clone())
            .collect::<Vec<_>>(),
        vec!["stepper_x", "stepper_y", "stepper_z"]
    );
    assert!(lookups.iter().all(|(_, opts)| *opts == RailOpts::default()), "lookup opts default");

    // setup_itersolve was called on each rail with the axis bytes; the
    // toolhead trapq was assigned to every stepper.
    for (idx, section) in ['x', 'y', 'z'].into_iter().enumerate() {
        let fake = &world.rails[idx];
        let calls = fake.itersolve_calls.lock().clone();
        assert_eq!(calls.len(), 1, "one setup_itersolve on {section}");
        assert_eq!(calls[0].0, "cartesian_stepper_alloc");
        assert_eq!(calls[0].1, vec![PyValue::Bytes(vec![section as u8])]);
        for stepper in &fake.stepper_fakes {
            assert!(stepper.did_trapq(), "stepper trapq assigned on {section}");
        }
    }

    // Python side with identical inputs.
    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'max_z_velocity': '50', 'max_z_accel': '500'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)",
    )
    .expect("py init");

    // get_status dict carries axes_min/max and the (empty) homed_axes.
    let expected_status = py.eval("kin_py.get_status(0.0)").expect("py status");
    check_value("get_status init", expected_status, pyvalue_to_rec(&kin.get_status(0.0)));

    // setup_itersolve order and args (alloc func + axis bytes), per rail.
    for (idx, section) in ['x', 'y', 'z'].into_iter().enumerate() {
        let expected_fn = py
            .eval(&format!(
                "rails_py['stepper_{section}'].itersolve_calls[0][0]"
            ))
            .expect("itersolve fn");
        check_value(
            &format!("itersolve fn {section}"),
            expected_fn,
            recording::Value::Str(world.rails[idx].itersolve_calls.lock()[0].0.clone()),
        );
        let expected_param = py
            .eval(&format!(
                "rails_py['stepper_{section}'].itersolve_calls[0][1][0]"
            ))
            .expect("itersolve param");
        check_value(
            &format!("itersolve param {section}"),
            expected_param,
            recording::Value::Bytes(vec![section as u8]),
        );
        let expected_trapq_len = py
            .eval(&format!(
                "len(rails_py['stepper_{section}'].steppers[0].trapq_sets)"
            ))
            .expect("trapq len");
        let actual_trapq_len = world.rails[idx].stepper_fakes[0].trapq_sets.lock().len() as i64;
        check_value(
            &format!("trapq len {section}"),
            expected_trapq_len,
            recording::Value::Int(actual_trapq_len),
        );
    }

    // max_z_velocity/accel are observable through check_move limit_speed on
    // a homed Z move (z_ratio = 10/10 = 1).
    kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("rust set_position");
    let mv_z = FakeMove::new([100.0, 100.0, 20.0, 0.0], [0.0, 0.0, 10.0, 0.0], 10.0);
    kin.check_move(mv_z.as_ref()).expect("rust z move");
    py.exec(
        "kin_py.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')\nmvz_py = FakeMove((100.0, 100.0, 20.0, 0.0), (0.0, 0.0, 10.0, 0.0), 10.0)\nkin_py.check_move(mvz_py)",
    )
    .expect("py z move");
    let expected_limit = py.eval("mvz_py.limit_speed_calls").expect("limit calls");
    check_value(
        "limit_speed explicit max_z",
        expected_limit,
        recording::Value::List(
            mv_z.limit_speed_calls()
                .iter()
                .map(|(s, a)| recording::Value::Tuple(vec![recording::Value::float(*s), recording::Value::float(*a)]))
                .collect(),
        ),
    );

    // Defaults: without explicit max_z_*, values fall back to the toolhead.
    let store2 = store_from(&[
        ("printer", &[("other", "1")]),
        ("stepper_x", &[("endstop_pin", "PA1")]),
        ("stepper_y", &[("endstop_pin", "PA2")]),
        ("stepper_z", &[("endstop_pin", "PA3")]),
    ]);
    let printer2 = FakePrinter::new() as Printer;
    let world2 = build_world(
        printer2,
        store2,
        200.0,
        1000.0,
        vec![
            ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
            ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
            ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
        ],
    );
    let kin2 = load_kinematics(&world2.toolhead, &world2.config).expect("rust init defaults");
    kin2.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("set 2");
    let mv_z2 = FakeMove::new([100.0, 100.0, 20.0, 0.0], [0.0, 0.0, 10.0, 0.0], 10.0);
    kin2.check_move(mv_z2.as_ref()).expect("rust z move defaults");
    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'other': '1'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nkin_py.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')\nmvz_py = FakeMove((100.0, 100.0, 20.0, 0.0), (0.0, 0.0, 10.0, 0.0), 10.0)\nkin_py.check_move(mvz_py)",
    )
    .expect("py init defaults");
    let expected_limit2 = py.eval("mvz_py.limit_speed_calls").expect("limit calls defaults");
    check_value(
        "limit_speed default max_z",
        expected_limit2,
        recording::Value::List(
            mv_z2.limit_speed_calls()
                .iter()
                .map(|(s, a)| recording::Value::Tuple(vec![recording::Value::float(*s), recording::Value::float(*a)]))
                .collect(),
        ),
    );
}

#[test]
fn max_z_config_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping max_z_config_errors: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let rails_spec = vec![
        ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
        ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
        ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
    ];
    for (printer_opts, py_store, label) in [
        (
            vec![("max_z_velocity", "300")],
            "{'printer': {'max_z_velocity': '300'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}",
            "max_z_velocity above max_velocity",
        ),
        (
            vec![("max_z_velocity", "0")],
            "{'printer': {'max_z_velocity': '0'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}",
            "max_z_velocity not above 0",
        ),
        (
            vec![("max_z_accel", "2000")],
            "{'printer': {'max_z_accel': '2000'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}",
            "max_z_accel above max_accel",
        ),
    ] {
        let store = store_from(&[
            ("printer", &printer_opts),
            ("stepper_x", &[("endstop_pin", "PA1")]),
            ("stepper_y", &[("endstop_pin", "PA2")]),
            ("stepper_z", &[("endstop_pin", "PA3")]),
        ]);
        let printer = FakePrinter::new() as Printer;
        let world = build_world(printer, store, 200.0, 1000.0, rails_spec.clone());
        let rust_err = load_kinematics(&world.toolhead, &world.config).map(|_| ());
        let expected = py.exec(&format!(
            "printer_py = FakePrinter()\nstore_py = {py_store}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {{'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)",
        ));
        check_error(label, expected, rust_err);
    }
}

#[test]
fn dual_carriage_config_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping dual_carriage_config_errors: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let rails_spec = vec![
        ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
        ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
        ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
        ("dual_carriage", "stepper_dual", (0.0, 220.0), hi, vec!["stepper_dual"]),
    ];
    for (dc_opts, py_dc_opts, label) in [
        (
            vec![("axis", "z")],
            "'dual_carriage': {'axis': 'z'}",
            "dual_carriage axis z invalid",
        ),
        (
            vec![("axis", "x"), ("safe_distance", "-5")],
            "'dual_carriage': {'axis': 'x', 'safe_distance': '-5'}",
            "dual_carriage safe_distance below min",
        ),
    ] {
        let store = store_from(&[
            ("printer", &[("max_z_velocity", "50")]),
            ("stepper_x", &[("endstop_pin", "PA1")]),
            ("stepper_y", &[("endstop_pin", "PA2")]),
            ("stepper_z", &[("endstop_pin", "PA3")]),
            ("dual_carriage", &dc_opts),
        ]);
        let printer = FakePrinter::new() as Printer;
        let world = build_world(printer, store, 200.0, 1000.0, rails_spec.clone());
        let rust_err = load_kinematics(&world.toolhead, &world.config).map(|_| ());
        let expected = py.exec(&format!(
            "printer_py = FakePrinter()\nstore_py = {{'printer': {{'max_z_velocity': '50'}}, 'stepper_x': {{'endstop_pin': 'PA1'}}, 'stepper_y': {{'endstop_pin': 'PA2'}}, 'stepper_z': {{'endstop_pin': 'PA3'}}, {py_dc_opts}}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {{'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'dual_carriage': FakeRail('stepper_dual', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)",
        ));
        check_error(label, expected, rust_err);
    }
}

#[test]
fn calc_position_and_get_steppers_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping calc_position_and_get_steppers_parity: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let printer = FakePrinter::new() as Printer;
    let store = store_from(&[
        ("printer", &[("other", "1")]),
        ("stepper_x", &[("endstop_pin", "PA1")]),
        ("stepper_y", &[("endstop_pin", "PA2")]),
        ("stepper_z", &[("endstop_pin", "PA3")]),
    ]);
    let world = build_world(
        printer,
        store,
        200.0,
        1000.0,
        vec![
            ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
            ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
            ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
        ],
    );
    let kin = load_kinematics(&world.toolhead, &world.config).expect("init");
    assert_eq!(kin.get_steppers().len(), 3);

    let mut positions = IndexMap::new();
    positions.insert("stepper_x".to_string(), 10.0);
    positions.insert("stepper_y".to_string(), 20.0);
    positions.insert("stepper_z".to_string(), 30.0);
    let rust_pos = kin.calc_position(&positions).expect("rust calc");

    // Missing stepper name raises KeyError on both sides.
    let mut sparse = IndexMap::new();
    sparse.insert("stepper_x".to_string(), 10.0);
    let rust_sparse = kin.calc_position(&sparse).map(|_| ());

    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'other': '1'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)",
    )
    .expect("py init");
    let expected = py
        .eval("kin_py.calc_position({'stepper_x': 10.0, 'stepper_y': 20.0, 'stepper_z': 30.0})")
        .expect("calc");
    check_value(
        "calc_position basic",
        expected,
        recording::Value::List(
            rust_pos
                .iter()
                .map(|o| match o {
                    Some(f) => recording::Value::float(*f),
                    None => recording::Value::Null,
                })
                .collect(),
        ),
    );
    let expected_steppers = py.eval("len(kin_py.get_steppers())").expect("steppers len");
    check_value(
        "get_steppers len",
        expected_steppers,
        recording::Value::Int(kin.get_steppers().len() as i64),
    );
    let expected_sparse = py.eval("kin_py.calc_position({'stepper_x': 10.0})");
    check_error("calc_position missing key", expected_sparse, rust_sparse);

    // Cartesian has no get_calibration (only delta kinematics do); both
    // sides raise AttributeError (message text is contract vocabulary).
    let rust_cal = kin.get_calibration().map(|_| ());
    let expected_cal = py.eval("kin_py.get_calibration()");
    check_error_class("get_calibration absent", expected_cal, rust_cal);
}

#[test]
fn calc_position_with_dual_carriage_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping calc_position_with_dual_carriage_parity: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    for (axis_str, axis_idx, mode) in [
        ("x", 0i64, "dual"),
        ("y", 1i64, "dual"),
        ("x", 0i64, "none"),
    ] {
        let printer = FakePrinter::new() as Printer;
        // The dual rail gets an inner range so set_position limits in the
        // 'dual' mode differ from the primary rail's range.
        let dc_rail = FakeRail::new("stepper_dual", (10.0, 210.0), hi, &["stepper_dual"]);
        let rails = vec![
            FakeRail::new("stepper_x", (0.0, 220.0), hi, &["stepper_x"]),
            FakeRail::new("stepper_y", (0.0, 220.0), hi, &["stepper_y"]),
            FakeRail::new("stepper_z", (0.0, 250.0), hi, &["stepper_z"]),
            dc_rail.clone(),
        ];
        let dc_primary: Option<Rail> = match mode {
            "dual" => Some(dc_rail.clone() as Rail),
            _ => None,
        };
        let (kin, _dc) = build_dc_kinematics(printer.clone(), &rails, axis_idx, dc_primary);

        let mut positions = IndexMap::new();
        positions.insert("stepper_x".to_string(), 10.0);
        positions.insert("stepper_y".to_string(), 20.0);
        positions.insert("stepper_z".to_string(), 30.0);
        positions.insert("stepper_dual".to_string(), 99.0);
        let rust_pos = kin.calc_position(&positions).expect("rust calc dc");

        // set_position limits with the dual-carriage axis homed.
        kin.set_position(vec![1.0, 2.0, 3.0, 4.0], axis_str).expect("rust set_position dc");
        let rust_limits = kin.limits();

        py.exec(&format!(
            "printer_py = FakePrinter()\nstore_py = {{'printer': {{'max_z_velocity': '50'}}, 'stepper_x': {{'endstop_pin': 'PA1'}}, 'stepper_y': {{'endstop_pin': 'PA2'}}, 'stepper_z': {{'endstop_pin': 'PA3'}}, 'dual_carriage': {{'axis': '{axis_str}', 'safe_distance': '10'}}}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {{'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'dual_carriage': FakeRail('stepper_dual', 10.0, 210.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}}\ninstall_fake_rails(rails_py)\ninstall_fake_dc('{mode}')\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nkin_py.set_position([1.0, 2.0, 3.0, 4.0], '{axis_str}')",
        ))
        .expect("py dc init");
        let label = format!("calc_position dc {axis_str} {mode}");
        let expected = py
            .eval("kin_py.calc_position({'stepper_x': 10.0, 'stepper_y': 20.0, 'stepper_z': 30.0, 'stepper_dual': 99.0})")
            .expect("calc dc");
        check_value(
            &label,
            expected,
            recording::Value::List(
                rust_pos
                    .iter()
                    .map(|o| match o {
                        Some(f) => recording::Value::float(*f),
                        None => recording::Value::Null,
                    })
                    .collect(),
            ),
        );
        // Two rail names are loaded (axis rail + dual rail): rails len 4.
        let expected_len = py.eval("len(kin_py.rails)").expect("rails len dc");
        check_value(
            "rails len dc",
            expected_len,
            recording::Value::Int(kin.rails().len() as i64),
        );
        let expected_axis = py.eval("kin_py.dual_carriage_axis").expect("dc axis");
        check_value(
            "dual_carriage_axis dc",
            expected_axis,
            recording::Value::Int(kin.dual_carriage_axis().unwrap()),
        );
        let expected_limits = py.eval("kin_py.limits").expect("py limits dc");
        check_value(
            "limits dc",
            expected_limits,
            recording::Value::List(
                rust_limits
                    .iter()
                    .map(|(l, h)| recording::Value::Tuple(vec![recording::Value::float(*l), recording::Value::float(*h)]))
                    .collect(),
            ),
        );
    }
}

#[test]
fn set_position_and_limits_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping set_position_and_limits_parity: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let printer = FakePrinter::new() as Printer;
    let rails = [
        FakeRail::new("stepper_x", (0.0, 220.0), hi, &["stepper_x"]),
        FakeRail::new("stepper_y", (5.0, 225.0), hi, &["stepper_y"]),
        FakeRail::new("stepper_z", (10.0, 230.0), hi, &["stepper_z"]),
    ];
    let dyn_rails: Vec<Rail> = rails.iter().map(|r| r.clone() as Rail).collect();
    let kin = CartKinematics::new(
        printer,
        dyn_rails,
        None,
        None,
        Coord(vec![0.0, 5.0, 10.0, 0.0]),
        Coord(vec![220.0, 225.0, 230.0, 0.0]),
        50.0,
        500.0,
    );

    for axes in ["", "x", "y", "z", "xyz", "yx", "zx"] {
        kin.set_position(vec![1.0, 2.0, 3.0, 4.0], axes).expect("set_position");
    }

    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'other': '1'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 5.0, 225.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 10.0, 230.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nfor axes_ in ['', 'x', 'y', 'z', 'xyz', 'yx', 'zx']: kin_py.set_position([1.0, 2.0, 3.0, 4.0], axes_)",
    )
    .expect("py init");
    let expected_limits = py.eval("kin_py.limits").expect("py limits");
    check_value(
        "limits after set xyz",
        expected_limits,
        recording::Value::List(
            kin.limits()
                .iter()
                .map(|(l, h)| recording::Value::Tuple(vec![recording::Value::float(*l), recording::Value::float(*h)]))
                .collect(),
        ),
    );
    // rail.set_position received the truncated (x, y, z) coord on every rail.
    for (idx, section) in ['x', 'y', 'z'].into_iter().enumerate() {
        let expected_calls = py
            .eval(&format!(
                "[[c[0], c[1], c[2]] for c in rails_py['stepper_{section}'].set_position_calls]"
            ))
            .expect("set_position calls");
        let rust_calls: Vec<recording::Value> = rails[idx]
            .set_position_calls
            .lock()
            .iter()
            .map(|c| recording::Value::List(c.iter().map(|v| recording::Value::float(*v)).collect()))
            .collect();
        check_value(
            &format!("set_position calls {section}"),
            expected_calls,
            recording::Value::List(rust_calls),
        );
    }

    // Invalid axis char raises ValueError (Python 'substring not found').
    let rust_err = kin.set_position(vec![1.0, 2.0, 3.0, 4.0], "a");
    let expected_err = py.eval("kin_py.set_position([1.0, 2.0, 3.0, 4.0], 'a')");
    check_error("set_position invalid axis", expected_err, rust_err);

    // clear_homing_state / update_limits state transitions, checked through
    // the remaining limits and through check_move behaviour.
    kin.clear_homing_state("y");
    py.exec("kin_py.clear_homing_state('y')").expect("py clear");
    let expected_limits = py.eval("kin_py.limits").expect("py limits clear");
    check_value(
        "limits after clear y",
        expected_limits,
        recording::Value::List(
            kin.limits()
                .iter()
                .map(|(l, h)| recording::Value::Tuple(vec![recording::Value::float(*l), recording::Value::float(*h)]))
                .collect(),
        ),
    );

    // update_limits on an un-homed axis is a no-op (still "Must home").
    kin.update_limits(1, (0.0, 100.0)).expect("update unhomed");
    py.exec("kin_py.update_limits(1, (0.0, 100.0))").expect("py update unhomed");
    let mv_unhomed = FakeMove::new([100.0, 999.0, 10.0, 0.0], [0.0, 900.0, 0.0, 0.0], 900.0);
    let rust_unhomed = kin.check_move(mv_unhomed.as_ref()).map(|_| ());
    py.exec("mv_py = FakeMove((100.0, 999.0, 10.0, 0.0), (0.0, 900.0, 0.0, 0.0), 900.0)").expect("py mv");
    let expected_unhomed = py.eval("kin_py.check_move(mv_py)");
    check_error("update_limits unhomed", expected_unhomed, rust_unhomed);

    // After re-homing, update_limits applies; moves past the new range fail.
    kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("re-home");
    kin.update_limits(1, (0.0, 100.0)).expect("update homed");
    py.exec("kin_py.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')\nkin_py.update_limits(1, (0.0, 100.0))").expect("py re-home+update");
    let mv_out = FakeMove::new([0.0, 150.0, 10.0, 0.0], [0.0, 100.0, 0.0, 0.0], 100.0);
    let rust_out = kin.check_move(mv_out.as_ref()).map(|_| ());
    py.exec("mv_py = FakeMove((0.0, 150.0, 10.0, 0.0), (0.0, 100.0, 0.0, 0.0), 100.0)").expect("py mv out");
    let expected_out = py.eval("kin_py.check_move(mv_py)");
    check_error("update_limits homed", expected_out, rust_out);
}

#[test]
fn home_axis_and_home_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping home_axis_and_home_parity: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi_pos = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let hi_neg = homing_info(80.0, 220.0, 5.0, 2.0, false, 4.0);
    let printer = FakePrinter::new() as Printer;
    let store = store_from(&[
        ("printer", &[("other", "1")]),
        ("stepper_x", &[("endstop_pin", "PA1")]),
        ("stepper_y", &[("endstop_pin", "PA2")]),
        ("stepper_z", &[("endstop_pin", "PA3")]),
    ]);
    let world = build_world(
        printer,
        store,
        200.0,
        1000.0,
        vec![
            ("stepper_x", "stepper_x", (0.0, 220.0), hi_pos, vec!["stepper_x"]),
            ("stepper_y", "stepper_y", (0.0, 220.0), hi_neg, vec!["stepper_y"]),
            ("stepper_z", "stepper_z", (0.0, 250.0), hi_pos, vec!["stepper_z"]),
        ],
    );
    let kin = load_kinematics(&world.toolhead, &world.config).expect("init");
    let homing = FakeHomingState::new(vec![0, 1, 2]);
    let homing_handle: HomingStateHandle = homing.clone();
    kin.home(homing_handle).expect("rust home");
    let calls = homing.calls();
    assert_eq!(calls.len(), 3);

    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'other': '1'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (80.0, 220.0, 5.0, 2.0, False, 4.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nhoming_py = FakeHoming([0, 1, 2])\nkin_py.home(homing_py)",
    )
    .expect("py home");
    for (idx, call) in calls.iter().enumerate().take(3) {
        let expected_names = py
            .eval(&format!("homing_py.home_rails_calls[{idx}][0]"))
            .expect("py names");
        check_value(
            &format!("home rails[{idx}]"),
            expected_names,
            recording::Value::List(call.0.iter().map(|n| recording::Value::Str(n.clone())).collect()),
        );
        let expected_force = py
            .eval(&format!("homing_py.home_rails_calls[{idx}][1]"))
            .expect("py force");
        check_value(
            &format!("home force[{idx}]"),
            expected_force,
            opts_floats_to_rec(&call.1),
        );
        let expected_home = py
            .eval(&format!("homing_py.home_rails_calls[{idx}][2]"))
            .expect("py homepos");
        check_value(
            &format!("home move[{idx}]"),
            expected_home,
            opts_floats_to_rec(&call.2),
        );
    }

    // Direct home_axis call for axis 1 with its own homing state.
    let homing2 = FakeHomingState::new(vec![]);
    kin.home_axis(homing2.clone(), 1, world.rails[1].clone() as Rail).expect("rust home_axis");
    py.exec("kin_py.home_axis(homing_py, 1, rails_py['stepper_y'])").expect("py home_axis");
    let expected_force = py.eval("homing_py.home_rails_calls[3][1]").expect("direct force");
    check_value(
        "home_axis force",
        expected_force,
        opts_floats_to_rec(&homing2.calls()[0].1),
    );

    // Dual-carriage home delegates axis 0 to the dual-carriage object.
    let printer2 = FakePrinter::new() as Printer;
    let dc_rail = FakeRail::new("stepper_dual", (10.0, 210.0), hi_pos, &["stepper_dual"]);
    let rails2 = vec![
        FakeRail::new("stepper_x", (0.0, 220.0), hi_pos, &["stepper_x"]),
        FakeRail::new("stepper_y", (0.0, 220.0), hi_pos, &["stepper_y"]),
        FakeRail::new("stepper_z", (0.0, 250.0), hi_pos, &["stepper_z"]),
        dc_rail.clone(),
    ];
    let (kin2, dc) = build_dc_kinematics(printer2, &rails2, 0, Some(dc_rail.clone() as Rail));
    let homing3 = FakeHomingState::new(vec![0, 1, 2]);
    kin2.home(homing3.clone()).expect("rust home dc");
    let dc_home_calls = dc.home_calls.lock().clone();
    let calls2 = homing3.calls();
    assert_eq!(calls2.len(), 2, "only y/z homed via rails");
    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'max_z_velocity': '50'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}, 'dual_carriage': {'axis': 'x', 'safe_distance': '10'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'dual_carriage': FakeRail('stepper_dual', 10.0, 210.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nhoming_py = FakeHoming([0, 1, 2])\nkin_py.home(homing_py)",
    )
    .expect("py home dc");
    let expected_dc = py.eval("_dc_holder['dc'].home_calls").expect("dc home calls");
    check_value(
        "dc home calls",
        expected_dc,
        recording::Value::List(dc_home_calls.iter().map(|a| recording::Value::Int(*a)).collect()),
    );
    let expected_rails = py.eval("homing_py.home_rails_calls").expect("py rails dc");
    check_value(
        "home rails dc",
        expected_rails,
        recording::Value::List(
            calls2
                .iter()
                .map(|(names, force, move_)| {
                    recording::Value::Tuple(vec![
                        recording::Value::List(names.iter().map(|n| recording::Value::Str(n.clone())).collect()),
                        opts_floats_to_rec(force),
                        opts_floats_to_rec(move_),
                    ])
                })
                .collect(),
        ),
    );
}

#[test]
fn check_move_and_status_parity() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping check_move_and_status_parity: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
        return;
    };
    py_setup(&mut py);

    let hi = homing_info(100.0, 0.0, 10.0, 1.0, true, 5.0);
    let printer = FakePrinter::new() as Printer;
    let store = store_from(&[
        ("printer", &[("max_z_velocity", "10"), ("max_z_accel", "100")]),
        ("stepper_x", &[("endstop_pin", "PA1")]),
        ("stepper_y", &[("endstop_pin", "PA2")]),
        ("stepper_z", &[("endstop_pin", "PA3")]),
    ]);
    let world = build_world(
        printer,
        store,
        200.0,
        1000.0,
        vec![
            ("stepper_x", "stepper_x", (0.0, 220.0), hi, vec!["stepper_x"]),
            ("stepper_y", "stepper_y", (0.0, 220.0), hi, vec!["stepper_y"]),
            ("stepper_z", "stepper_z", (0.0, 250.0), hi, vec!["stepper_z"]),
        ],
    );
    let kin = load_kinematics(&world.toolhead, &world.config).expect("init");
    kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("home all");

    // XY move inside: no error, no limit_speed.
    let mv_inside = FakeMove::new([100.0, 100.0, 10.0, 0.0], [50.0, 50.0, 0.0, 0.0], 70.71067811865476);
    assert!(kin.check_move(mv_inside.as_ref()).is_ok());
    assert!(mv_inside.limit_speed_calls().is_empty());

    // Z move: limit_speed with z_ratio = move_d / |axes_d[2]|.
    let mv_z = FakeMove::new([100.0, 100.0, 20.0, 0.0], [0.0, 0.0, 10.0, 0.0], 10.0);
    assert!(kin.check_move(mv_z.as_ref()).is_ok());
    let calls = mv_z.limit_speed_calls();
    assert_eq!(calls.len(), 1);

    // XY outside homed: bare Move out of range.
    let mv_out = FakeMove::new([999.0, 100.0, 10.0, 0.0], [900.0, 0.0, 0.0, 0.0], 900.0);
    let rust_out = kin.check_move(mv_out.as_ref()).map(|_| ());

    // Un-homed axis: Must home axis first (after clear).
    kin.clear_homing_state("x");
    let mv_unhomed = FakeMove::new([999.0, 100.0, 10.0, 10.0], [900.0, 0.0, 0.0, 0.0], 900.0);
    let rust_unhomed = kin.check_move(mv_unhomed.as_ref()).map(|_| ());
    kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("re-home");

    // get_status: all homed -> "xyz".
    let status = kin.get_status(0.0);

    py.exec(
        "printer_py = FakePrinter()\nstore_py = {'printer': {'max_z_velocity': '10', 'max_z_accel': '100'}, 'stepper_x': {'endstop_pin': 'PA1'}, 'stepper_y': {'endstop_pin': 'PA2'}, 'stepper_z': {'endstop_pin': 'PA3'}}\nconfig_py = FakeConfig(printer_py, store_py, 'printer')\ntoolhead_py = FakeToolhead(200.0, 1000.0)\ntoolhead_py.Coord = FakeCoord\nrails_py = {'stepper_x': FakeRail('stepper_x', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_y': FakeRail('stepper_y', 0.0, 220.0, (100.0, 0.0, 10.0, 1.0, True, 5.0)), 'stepper_z': FakeRail('stepper_z', 0.0, 250.0, (100.0, 0.0, 10.0, 1.0, True, 5.0))}\ninstall_fake_rails(rails_py)\nimport importlib, kinematics.cartesian\nimportlib.reload(kinematics.cartesian)\nkin_py = kinematics.cartesian.load_kinematics(toolhead_py, config_py)\nkin_py.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')",
    )
    .expect("py init");

    let expected_status = py.eval("kin_py.get_status(0.0)").expect("py status");
    check_value("get_status xyz", expected_status, pyvalue_to_rec(&status));

    py.exec("mv_py = FakeMove((999.0, 100.0, 10.0, 0.0), (900.0, 0.0, 0.0, 0.0), 900.0)").expect("py mv");
    let expected_out = py.eval("kin_py.check_move(mv_py)");
    check_error("check_move out of range", expected_out, rust_out);

    py.exec("kin_py.clear_homing_state('x')\nmv_py = FakeMove((999.0, 100.0, 10.0, 10.0), (900.0, 0.0, 0.0, 0.0), 900.0)").expect("py clear");
    let expected_unhomed = py.eval("kin_py.check_move(mv_py)");
    check_error("check_move unhomed", expected_unhomed, rust_unhomed);
    py.exec("kin_py.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')").expect("py re-home");

    // XY inside move: no error, no limit_speed on the reference either.
    py.exec("mv_py = FakeMove((100.0, 100.0, 10.0, 0.0), (50.0, 50.0, 0.0, 0.0), 70.71067811865476)\nkin_py.check_move(mv_py)").expect("py inside");
    let expected_calls = py.eval("mv_py.limit_speed_calls").expect("no limit calls");
    check_value("no limit_speed xy", expected_calls, recording::Value::List(Vec::new()));

    // Z move limit_speed args.
    py.exec("mvz_py = FakeMove((100.0, 100.0, 20.0, 0.0), (0.0, 0.0, 10.0, 0.0), 10.0)\nkin_py.check_move(mvz_py)").expect("py z move");
    let expected_limit = py.eval("mvz_py.limit_speed_calls").expect("limit calls");
    check_value(
        "limit_speed z",
        expected_limit,
        recording::Value::List(
            calls
                .iter()
                .map(|(s, a)| recording::Value::Tuple(vec![recording::Value::float(*s), recording::Value::float(*a)]))
                .collect(),
        ),
    );
}
