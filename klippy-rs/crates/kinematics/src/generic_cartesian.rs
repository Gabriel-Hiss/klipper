//! Port of the `generic_cartesian` module (`klippy/kinematics/generic_cartesian.py`).
//!
//! Implements `MainCarriage`, `ExtraCarriage`, `DualCarriage`,
//! `GenericCartesianKinematics` and `load_kinematics` with the Python
//! semantics faithfully reproduced, including error texts, config/G-code
//! contracts and float operation order.
//!
//! The kinematic stepper (`KinematicStepper`), the dual-carriage controller
//! (`DualCarriages`) and the matrix helpers (`mathutil`) come from their
//! owners (`crate::kinematic_stepper`, `crate::idex_modes`,
//! `klippy_core::mathutil`); this file owns only the carriage classes
//! and the generic-cartesian kinematics itself.

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::{PyInt, PyValue};

use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCodeApi, GetIntOpts, GetOpts, RegisterCommandOpts};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::{Rail, RailOpts, Stepper, StepperFactory};
use klippy_api::toolhead::{
    HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_api::lookup_interface;
use klippy_core::mathutil::{
    gaussian_solve, mat_mat_mul, mat_transp, matrix_dot, pseudo_inverse, Matrix,
};

use crate::idex_modes::{DualCarriages, DualCarriagesApi, PRIMARY};
use crate::kinematic_stepper::{Carriage as KCarriage, CarriageApi, KinematicStepper};

/// `VALID_AXES = ['x', 'y', 'z']` (`generic_cartesian.py:12`).
pub const VALID_AXES: [&str; 3] = ["x", "y", "z"];

/// `cmd_SET_STEPPER_CARRIAGES_help = "Set stepper carriages"` (`:355`).
pub const CMD_SET_STEPPER_CARRIAGES_HELP: &str = "Set stepper carriages";

fn config_err(msg: String) -> KlippyError {
    KlippyError::Config(msg)
}

fn command_err(msg: String) -> KlippyError {
    KlippyError::Command(msg)
}

fn internal_err(class_name: &str, msg: String) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: msg,
    }
}

fn get_required_str(config: &Config, option: &str) -> KResult<String> {
    let v = config.get(option, GetStrOpts { default: None, note_valid: true })?;
    match v {
        Some(s) => Ok(s),
        None => Err(config_err(format!(
            "Option '{}' in section '{}' must be specified",
            option,
            config.get_name()
        ))),
    }
}

fn get_optional_str(config: &Config, option: &str) -> KResult<Option<String>> {
    config.get(
        option,
        GetStrOpts {
            default: Some(None),
            note_valid: true,
        },
    )
}

fn valid_axes_choices() -> PyValue {
    PyValue::List(VALID_AXES.iter().map(|s| PyValue::Str(s.to_string())).collect())
}

fn valid_axes_none_choices() -> PyValue {
    let mut v: Vec<PyValue> = VALID_AXES.iter().map(|s| PyValue::Str(s.to_string())).collect();
    v.push(PyValue::None);
    PyValue::List(v)
}

fn get_axis_choice(
    config: &Config,
    default: Option<Option<String>>,
) -> KResult<String> {
    let opts = GetChoiceOpts {
        default: default.map(|d| d.map(PyValue::Str)),
        note_valid: true,
    };
    let v = config.getchoice("axis", &valid_axes_choices(), opts)?;
    match v {
        PyValue::Str(s) => Ok(s),
        other => Err(config_err(format!(
            "Choice '{}' for option 'axis' in section '{}' is not a valid choice",
            other.repr(),
            config.get_name()
        ))),
    }
}

fn get_axis_choice_optional_none(config: &Config) -> KResult<Option<String>> {
    let opts = GetChoiceOpts {
        default: Some(None),
        note_valid: true,
    };
    let v = config.getchoice("axis", &valid_axes_none_choices(), opts)?;
    match v {
        PyValue::Str(s) => Ok(Some(s)),
        PyValue::None => Ok(None),
        other => Err(config_err(format!(
            "Choice '{}' for option 'axis' in section '{}' is not a valid choice",
            other.repr(),
            config.get_name()
        ))),
    }
}

fn get_axis_required(config: &Config) -> KResult<String> {
    get_axis_choice(config, None)
}

fn axis_index(name: &str) -> i64 {
    (name.as_bytes()[0] as i64) - ('x' as i64)
}

// ---------------------------------------------------------------------------
// Carriage handles: `KinematicStepper` reaches carriages through
// [`CarriageApi`]; downcast helpers recover the own-module concrete type
// for the operations the trait does not name.
// ---------------------------------------------------------------------------

fn as_main(c: &KCarriage) -> Option<&MainCarriage> {
    c.as_any().downcast_ref::<MainCarriage>()
}

fn as_dual(c: &KCarriage) -> Option<&DualCarriage> {
    c.as_any().downcast_ref::<DualCarriage>()
}

fn as_extra(c: &KCarriage) -> Option<&ExtraCarriage> {
    c.as_any().downcast_ref::<ExtraCarriage>()
}

fn carriage_name(c: &KCarriage) -> String {
    if let Some(m) = as_main(c) {
        return m.get_name();
    }
    if let Some(d) = as_dual(c) {
        return d.get_name();
    }
    if let Some(e) = as_extra(c) {
        return e.get_name();
    }
    String::new()
}

fn carriage_rail(c: &KCarriage) -> Rail {
    if let Some(m) = as_main(c) {
        return m.get_rail();
    }
    if let Some(d) = as_dual(c) {
        return d.get_rail();
    }
    as_extra(c).expect("unknown carriage").get_rail()
}

/// `carriage.axis_name` as Python stores it. A `DualCarriage` configured
/// with `primary_carriage` and no `axis` keeps `None`.
fn carriage_axis_name(c: &KCarriage) -> Option<String> {
    if let Some(m) = as_main(c) {
        return Some(m.get_axis_name());
    }
    as_dual(c).and_then(|d| d.axis_name.clone())
}

fn py_str_opt(v: &Option<String>) -> String {
    match v {
        Some(s) => s.clone(),
        None => "None".to_string(),
    }
}

fn carriage_dual_carriage(c: &KCarriage) -> Option<KCarriage> {
    if let Some(m) = as_main(c) {
        return m.get_dual_carriage().map(|d| {
            let a: Arc<DualCarriage> = d;
            let x: KCarriage = a;
            x
        });
    }
    as_dual(c).and_then(|d| d.get_dual_carriage())
}


fn carriage_set_dual_carriage(c: &KCarriage, dc: Arc<DualCarriage>) {
    if let Some(m) = as_main(c) {
        m.set_dual_carriage(dc);
    } else if let Some(d) = as_dual(c) {
        d.set_dual_carriage(dc);
    }
}

fn carriage_is_active(c: &KCarriage) -> KResult<bool> {
    if let Some(m) = as_main(c) {
        return m.is_active();
    }
    if let Some(d) = as_dual(c) {
        return d.is_active();
    }
    // `ExtraCarriage` never appears in `self.carriages`; Python would
    // raise `AttributeError` for a missing `is_active`.
    Err(internal_err(
        "AttributeError",
        "ExtraCarriage object has no attribute 'is_active'".to_string(),
    ))
}

// ---------------------------------------------------------------------------
// Carriages.
// ---------------------------------------------------------------------------

/// `MainCarriage` (`generic_cartesian.py:14`).
pub struct MainCarriage {
    rail: Rail,
    axis_name: String,
    axis: i64,
    printer: Printer,
    dual_carriage: GCell<Option<Arc<DualCarriage>>>,
}

impl MainCarriage {
    pub fn new(config: &Config, factory: &StepperFactory) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let rail = factory.generic_rail(config, RailOpts::default())?;
        let carriage_name = rail.get_name(Some(true));
        let axis_name = if VALID_AXES.contains(&carriage_name.as_str()) {
            get_axis_choice(config, Some(Some(carriage_name.clone())))?
        } else {
            get_axis_required(config)?
        };
        let axis = axis_index(&axis_name);
        Ok(Arc::new(MainCarriage {
            rail,
            axis_name,
            axis,
            printer,
            dual_carriage: GCell::new(None),
        }))
    }

    pub fn get_name(&self) -> String {
        self.rail.get_name(Some(true))
    }
    pub fn get_axis_name(&self) -> String {
        self.axis_name.clone()
    }
    pub fn get_axis(&self) -> i64 {
        self.axis
    }
    pub fn get_rail(&self) -> Rail {
        self.rail.clone()
    }
    pub fn is_active(&self) -> KResult<bool> {
        if self.dual_carriage.borrow().is_none() {
            return Ok(true);
        }
        let dc_module = lookup_interface::<dyn DualCarriagesApi>(
            &*self.printer,
            "dual_carriage",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        Ok(dc_module.is_active(&self.rail))
    }
    pub fn set_dual_carriage(&self, carriage: Arc<DualCarriage>) {
        *self.dual_carriage.borrow_mut() = Some(carriage);
    }
    pub fn get_dual_carriage(&self) -> Option<Arc<DualCarriage>> {
        self.dual_carriage.borrow().clone()
    }
}

impl CarriageApi for MainCarriage {
    fn get_axis(&self) -> i64 {
        MainCarriage::get_axis(self)
    }
    fn add_stepper(&self, kin_stepper: &KinematicStepper) -> KResult<()> {
        self.rail.add_stepper(kin_stepper.get_stepper(), None, None)
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

/// `DualCarriage` (`generic_cartesian.py:56`).
pub struct DualCarriage {
    printer: Printer,
    rail: Rail,
    primary_carriage_name: Option<String>,
    axis_name: Option<String>,
    axis: GCell<i64>,
    safe_dist: Option<f64>,
    primary_carriage: GCell<Option<KCarriage>>,
    dual_carriage: GCell<Option<Arc<DualCarriage>>>,
}

impl DualCarriage {
    pub fn new(config: &Config, factory: &StepperFactory) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let rail = factory.generic_rail(config, RailOpts::default())?;
        let primary_carriage_name = get_optional_str(config, "primary_carriage")?;
        let (axis_name, axis, safe_dist) = if primary_carriage_name.is_none() {
            let an = get_axis_required(config)?;
            let ax = axis_index(&an);
            (Some(an), ax, None)
        } else {
            let an = get_axis_choice_optional_none(config)?;
            let ax = match &an {
                Some(s) => axis_index(s),
                None => 0,
            };
            let sd = config.getfloat(
                "safe_distance",
                GetFloatOpts {
                    default: Some(None),
                    minval: Some(0.0),
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?;
            (an, ax, sd)
        };
        Ok(Arc::new(DualCarriage {
            printer,
            rail,
            primary_carriage_name,
            axis_name,
            axis: GCell::new(axis),
            safe_dist,
            primary_carriage: GCell::new(None),
            dual_carriage: GCell::new(None),
        }))
    }

    pub fn resolve_primary_carriage(
        self: &Arc<Self>,
        carriages: &IndexMap<String, KCarriage>,
    ) -> KResult<()> {
        let Some(pname) = self.primary_carriage_name.as_ref() else {
            return Ok(());
        };
        let Some(primary) = carriages.get(pname).cloned() else {
            return Err(config_err(format!(
                "primary_carriage = '{}' for '{}' is not a valid choice",
                pname,
                self.get_name()
            )));
        };
        *self.primary_carriage.borrow_mut() = Some(primary.clone());
        let primary_axis_name = carriage_axis_name(&primary);
        let axis_name = self.axis_name.clone().or_else(|| primary_axis_name.clone());
        if axis_name != primary_axis_name {
            return Err(config_err(format!(
                "Mismatching axes between carriage '{}' (axis={}) and dual_carriage '{}' (axis={})",
                carriage_name(&primary),
                py_str_opt(&primary_axis_name),
                self.get_name(),
                py_str_opt(&axis_name)
            )));
        }
        let Some(axis_name) = axis_name else {
            return Err(internal_err(
                "TypeError",
                "ord() expected string of length 1, but NoneType found".to_string(),
            ));
        };
        self.axis.set(axis_index(&axis_name));
        if let Some(existing) = carriage_dual_carriage(&primary) {
            return Err(config_err(format!(
                "Multiple dual carriages ('{}', '{}') for carriage '{}'",
                carriage_name(&existing),
                self.get_name(),
                carriage_name(&primary)
            )));
        }
        carriage_set_dual_carriage(&primary, Arc::clone(self));
        let axis = primary.get_axis();
        self.axis.set(axis);
        if axis > 1 {
            return Err(config_err(format!(
                "Invalid axis '{}' for dual_carriage '{}'",
                "xyz".chars().nth(axis as usize).unwrap_or('?'),
                self.get_name()
            )));
        }
        Ok(())
    }

    pub fn get_name(&self) -> String {
        self.rail.get_name(Some(true))
    }
    pub fn get_axis(&self) -> i64 {
        self.axis.get()
    }
    pub fn get_rail(&self) -> Rail {
        self.rail.clone()
    }
    pub fn get_safe_dist(&self) -> Option<f64> {
        self.safe_dist
    }
    pub fn get_dc_module(&self) -> KResult<Arc<dyn DualCarriagesApi>> {
        match lookup_interface::<dyn DualCarriagesApi>(
            &*self.printer,
            "dual_carriage",
            LookupDefault::Raise,
        )? {
            Some(m) => Ok(m),
            None => Err(config_err(
                "Unable to lookup object 'dual_carriage'".to_string(),
            )),
        }
    }
    pub fn is_active(&self) -> KResult<bool> {
        let dc_module = self.get_dc_module()?;
        Ok(dc_module.is_active(&self.rail))
    }
    pub fn set_dual_carriage(&self, carriage: Arc<DualCarriage>) {
        *self.dual_carriage.borrow_mut() = Some(carriage);
    }
    pub fn get_dual_carriage(&self) -> Option<KCarriage> {
        if let Some(d) = self.dual_carriage.borrow().clone() {
            let a: Arc<DualCarriage> = d;
            return Some(a);
        }
        self.primary_carriage.borrow().clone()
    }
    pub fn get_primary_carriage(&self) -> Option<KCarriage> {
        self.primary_carriage.borrow().clone()
    }
}

impl CarriageApi for DualCarriage {
    fn get_axis(&self) -> i64 {
        DualCarriage::get_axis(self)
    }
    fn add_stepper(&self, kin_stepper: &KinematicStepper) -> KResult<()> {
        self.rail.add_stepper(kin_stepper.get_stepper(), None, None)
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

/// `ExtraCarriage` (`generic_cartesian.py:41`).
pub struct ExtraCarriage {
    name: String,
    primary_carriage: KCarriage,
    endstop_pin: String,
}

impl ExtraCarriage {
    pub fn new(
        config: &Config,
        carriages: &IndexMap<String, KCarriage>,
    ) -> KResult<Arc<Self>> {
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let pname = get_required_str(config, "primary_carriage")?;
        let primary_carriage = match carriages.get(&pname) {
            Some(c) => c.clone(),
            None => {
                return Err(config_err(format!(
                    "Choice '{}' for option 'primary_carriage' in section '{}' is not a valid choice",
                    pname,
                    config.get_name()
                )));
            }
        };
        let endstop_pin = get_required_str(config, "endstop_pin")?;
        Ok(Arc::new(ExtraCarriage {
            name,
            primary_carriage,
            endstop_pin,
        }))
    }

    pub fn get_name(&self) -> String {
        self.name.clone()
    }
    pub fn get_axis(&self) -> i64 {
        self.primary_carriage.get_axis()
    }
    pub fn get_rail(&self) -> Rail {
        carriage_rail(&self.primary_carriage)
    }
}

impl CarriageApi for ExtraCarriage {
    fn get_axis(&self) -> i64 {
        ExtraCarriage::get_axis(self)
    }
    fn add_stepper(&self, kin_stepper: &KinematicStepper) -> KResult<()> {
        carriage_rail(&self.primary_carriage).add_stepper(
            kin_stepper.get_stepper(),
            Some(&self.endstop_pin),
            Some(&self.name),
        )
    }
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// ---------------------------------------------------------------------------
// GenericCartesianKinematics (`generic_cartesian.py:119`).
// ---------------------------------------------------------------------------

pub struct GenericCartesianKinematics {
    printer: Printer,
    carriages: IndexMap<String, KCarriage>,
    primary_carriages: Vec<Arc<MainCarriage>>,
    dc_carriages: Vec<Arc<DualCarriage>>,
    kin_steppers: Vec<KinematicStepper>,
    all_carriages: IndexMap<String, KCarriage>,
    dc_module: Option<Arc<DualCarriages>>,
    limits: GCell<Vec<(f64, f64)>>,
    max_z_velocity: f64,
    max_z_accel: f64,
}

impl GenericCartesianKinematics {
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let factory = toolhead.stepper_factory();

        // ---- _load_kinematics ----
        let mut primary_carriages: Vec<Arc<MainCarriage>> = Vec::new();
        for mcconfig in config.get_prefix_sections("carriage ") {
            primary_carriages.push(MainCarriage::new(&mcconfig, &factory)?);
        }
        for (axis, axis_name) in (0i64..3).zip(VALID_AXES.iter()) {
            let dups: Vec<String> = primary_carriages
                .iter()
                .filter(|pc| pc.get_axis() == axis)
                .map(|pc| pc.get_name())
                .collect();
            if dups.len() > 1 {
                return Err(config_err(format!(
                    "Axis '{}' is set for multiple primary carriages ({})",
                    axis_name,
                    dups.join(", ")
                )));
            } else if dups.is_empty() {
                return Err(config_err(format!(
                    "No carriage defined for axis '{}'",
                    axis_name
                )));
            }
        }
        let mut dc_carriages: Vec<Arc<DualCarriage>> = Vec::new();
        for dcconfig in config.get_prefix_sections("dual_carriage ") {
            dc_carriages.push(DualCarriage::new(&dcconfig, &factory)?);
        }
        let mut carriages_map: IndexMap<String, KCarriage> = IndexMap::new();
        for c in primary_carriages
            .iter()
            .map(|m| {
                let a: Arc<MainCarriage> = Arc::clone(m);
                let x: KCarriage = a;
                x
            })
            .chain(dc_carriages.iter().map(|d| {
                let a: Arc<DualCarriage> = Arc::clone(d);
                let x: KCarriage = a;
                x
            }))
        {
            let name = carriage_name(&c);
            if carriages_map.contains_key(&name) {
                return Err(config_err(format!("Redefinition of carriage {}", name)));
            }
            carriages_map.insert(name, c);
        }
        for dc in &dc_carriages {
            dc.resolve_primary_carriage(&carriages_map)?;
        }
        let carriages_snapshot = carriages_map.clone();
        let mut all_map: IndexMap<String, KCarriage> = carriages_map.clone();
        let mut ec_carriages: Vec<Arc<ExtraCarriage>> = Vec::new();
        for ecconfig in config.get_prefix_sections("extra_carriage ") {
            ec_carriages.push(ExtraCarriage::new(&ecconfig, &carriages_map)?);
        }
        for ec in &ec_carriages {
            let name = ec.get_name();
            if all_map.contains_key(&name) {
                return Err(config_err(format!("Redefinition of carriage {}", name)));
            }
            let a: Arc<ExtraCarriage> = Arc::clone(ec);
            let x: KCarriage = a;
            all_map.insert(name, x);
        }
        let mut kin_steppers: Vec<KinematicStepper> = Vec::new();
        for sc in config.get_prefix_sections("stepper ") {
            kin_steppers.push(KinematicStepper::new(&factory, &sc, &all_map)?);
        }

        // ---- _check_carriages_references / _check_multi_mcu_homing ----
        {
            let mut remaining = all_map.clone();
            for s in &kin_steppers {
                for c in s.get_carriages() {
                    remaining.shift_remove(&carriage_name(&c));
                }
            }
            if !remaining.is_empty() {
                let names: Vec<String> = remaining.keys().cloned().collect();
                return Err(config_err(format!(
                    "Carriage(s) {} must be referenced by some stepper(s)",
                    names.join(", ")
                )));
            }
        }
        {
            for carriage in carriages_snapshot.values() {
                let rail = carriage_rail(carriage);
                let rail_steppers = rail.get_steppers();
                for es in rail.get_endstops() {
                    let endstop_steppers = es.0.get_steppers();
                    let mut mcus: Vec<Mcu> = Vec::new();
                    for s in &endstop_steppers {
                        if rail_steppers.iter().any(|rs| Arc::ptr_eq(rs, s)) {
                            let m = s.get_mcu();
                            if !mcus.iter().any(|mm| Arc::ptr_eq(mm, &m)) {
                                mcus.push(m);
                            }
                        }
                    }
                    if mcus.len() > 1 {
                        return Err(config_err(format!(
                            "Multi-mcu homing not supported on multi-mcu shared carriage {}",
                            es.1
                        )));
                    }
                }
            }
        }

        // ---- set_trapq ----
        let trapq = toolhead.get_trapq();
        for s in &kin_steppers {
            s.get_stepper().set_trapq(Some(trapq.clone()));
        }

        // ---- dc_module setup ----
        let mut dc_module: Option<Arc<DualCarriages>> = None;
        let zero_pos = [0.0f64, 0.0];
        if !dc_carriages.is_empty() {
            let dc_axes: Vec<i64> = {
                let mut a: Vec<i64> = dc_carriages.iter().map(|dc| dc.get_axis()).collect();
                a.sort();
                a.dedup();
                a
            };
            let mut pcs: Vec<KCarriage> = Vec::new();
            for pc in &primary_carriages {
                if dc_axes.contains(&pc.get_axis()) {
                    let a: Arc<MainCarriage> = Arc::clone(pc);
                    pcs.push(a);
                }
            }
            for dc in &dc_carriages {
                if dc.get_primary_carriage().is_none() {
                    let a: Arc<DualCarriage> = Arc::clone(dc);
                    pcs.push(a);
                }
            }
            let dcs: Vec<Option<KCarriage>> =
                pcs.iter().map(|pc| carriage_dual_carriage(pc)).collect();
            let primary_rails: Vec<Rail> =
                pcs.iter().map(|pc| carriage_rail(pc)).collect();
            let dual_rails: Vec<Option<Rail>> = dcs
                .iter()
                .map(|dc| dc.as_ref().map(|d| carriage_rail(d)))
                .collect();
            let axes: Vec<i64> = pcs.iter().map(|pc| pc.get_axis()).collect();
            let safe_dist: Vec<Option<f64>> = dcs
                .iter()
                .map(|dc| match dc {
                    Some(d) => as_dual(d).and_then(|dd| dd.get_safe_dist()),
                    None => None,
                })
                .collect();
            let module = DualCarriages::new(
                printer.clone(),
                primary_rails,
                dual_rails,
                axes,
                safe_dist,
            )?;
            dc_module = Some(module.clone());

            let pairs: Vec<(KCarriage, Option<KCarriage>)> =
                pcs.iter().cloned().zip(dcs.iter().cloned()).collect();
            let total = 1usize << pairs.len();
            for mask in 0..total {
                let mut combo: Vec<Option<KCarriage>> = Vec::with_capacity(pairs.len());
                for (idx, (pc, dc)) in pairs.iter().enumerate() {
                    // `itertools.product` varies the last pair fastest.
                    let bit = (mask >> (pairs.len() - 1 - idx)) & 1;
                    if bit == 0 {
                        combo.push(Some(pc.clone()));
                    } else {
                        combo.push(dc.clone());
                    }
                }
                for c_opt in &combo {
                    let c = match c_opt {
                        Some(c) => c.clone(),
                        None => continue,
                    };
                    let wrapper = match module.get_dc_rail_wrapper(&carriage_rail(&c)) {
                        Some(w) => w,
                        None => {
                            return Err(internal_err(
                                "AttributeError",
                                "'NoneType' object has no attribute 'activate'".to_string(),
                            ))
                        }
                    };
                    wrapper.activate(PRIMARY, &zero_pos, None)?;
                    if let Some(dc) = carriage_dual_carriage(&c) {
                        let w2 = match module.get_dc_rail_wrapper(&carriage_rail(&dc)) {
                            Some(w) => w,
                            None => {
                                return Err(internal_err(
                                    "AttributeError",
                                    "'NoneType' object has no attribute 'inactivate'"
                                        .to_string(),
                                ))
                            }
                        };
                        w2.inactivate(&zero_pos)?;
                    }
                }
                check_kinematics_with_module(
                    &kin_steppers,
                    &carriages_snapshot,
                    &Some(module.clone()),
                    &|m| config_err(m),
                )?;
            }
            for dc in &dc_carriages {
                let wrapper = match module.get_dc_rail_wrapper(&dc.get_rail()) {
                    Some(w) => w,
                    None => {
                        return Err(internal_err(
                            "AttributeError",
                            "'NoneType' object has no attribute 'inactivate'".to_string(),
                        ))
                    }
                };
                wrapper.inactivate(&zero_pos)?;
            }
            for pc in &primary_carriages {
                if !dc_axes.contains(&pc.get_axis()) {
                    continue;
                }
                let wrapper = match module.get_dc_rail_wrapper(&pc.get_rail()) {
                    Some(w) => w,
                    None => {
                        return Err(internal_err(
                            "AttributeError",
                            "'NoneType' object has no attribute 'activate'".to_string(),
                        ))
                    }
                };
                wrapper.activate(PRIMARY, &zero_pos, None)?;
            }
        }

        // `__init__` runs this check inside the DC loop above and here for the
        // single-carriage case, before the `max_z_*` options are read.
        if dc_module.is_none() {
            check_kinematics_with_module(
                &kin_steppers,
                &carriages_snapshot,
                &dc_module,
                &|m| config_err(m),
            )?;
        }

        // ---- boundary checks setup ----
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config.getfloat(
            "max_z_velocity",
            GetFloatOpts {
                default: Some(Some(max_velocity)),
                minval: None,
                maxval: Some(max_velocity),
                above: Some(0.0),
                below: None,
                note_valid: true,
                ..Default::default()
            },
        )?.unwrap_or(max_velocity);
        let max_z_accel = config.getfloat(
            "max_z_accel",
            GetFloatOpts {
                default: Some(Some(max_accel)),
                minval: None,
                maxval: Some(max_accel),
                above: Some(0.0),
                below: None,
                note_valid: true,
                ..Default::default()
            },
        )?.unwrap_or(max_accel);

        let kin = Arc::new(GenericCartesianKinematics {
            printer: printer.clone(),
            carriages: carriages_snapshot,
            primary_carriages,
            dc_carriages,
            kin_steppers,
            all_carriages: all_map,
            dc_module,
            limits: GCell::new(vec![(1.0, -1.0); 3]),
            max_z_velocity,
            max_z_accel,
        });

        // ---- register gcode ----
        {
            let gcode = lookup_interface::<dyn GCodeApi>(
                &*printer,
                "gcode",
                LookupDefault::Raise,
            )?
            .expect("LookupDefault::Raise never yields None");
            let kin_for_cmd = Arc::clone(&kin);
            let handler = Box::new(move |gcmd: &klippy_api::gcode::GCmd| {
                kin_for_cmd.cmd_set_stepper_carriages(gcmd)
            });
            gcode.register_command(
                "SET_STEPPER_CARRIAGES",
                Some(handler),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_SET_STEPPER_CARRIAGES_HELP.to_string()),
                },
            )?;
        }

        Ok(kin)
    }

    pub fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    pub fn get_carriages(&self) -> &IndexMap<String, KCarriage> {
        &self.carriages
    }
    pub fn get_all_carriages(&self) -> &IndexMap<String, KCarriage> {
        &self.all_carriages
    }
    pub fn get_primary_carriages(&self) -> &[Arc<MainCarriage>] {
        &self.primary_carriages
    }
    pub fn get_dc_carriages(&self) -> &[Arc<DualCarriage>] {
        &self.dc_carriages
    }
    pub fn get_kin_steppers(&self) -> &[KinematicStepper] {
        &self.kin_steppers
    }
    pub fn get_dc_module(&self) -> Option<&Arc<DualCarriages>> {
        self.dc_module.as_ref()
    }
    pub fn get_max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }
    pub fn get_max_z_accel(&self) -> f64 {
        self.max_z_accel
    }
    pub fn get_limits(&self) -> Vec<(f64, f64)> {
        self.limits.cloned()
    }

    fn _get_kinematics_coeffs(
        kin_steppers: &[KinematicStepper],
        carriages: &IndexMap<String, KCarriage>,
        dc_module: &Option<Arc<DualCarriages>>,
    ) -> KResult<(Matrix, Vec<f64>)> {
        let mut matr: IndexMap<String, Vec<f64>> = IndexMap::new();
        let mut offs: IndexMap<String, Vec<f64>> = IndexMap::new();
        for s in kin_steppers {
            let name = s.get_name(false);
            matr.insert(name.clone(), s.get_kin_coeffs().to_vec());
            offs.insert(name, vec![0.0; 3]);
        }
        if dc_module.is_none() {
            let m: Matrix = kin_steppers
                .iter()
                .map(|s| matr.get(&s.get_name(false)).unwrap().clone())
                .collect();
            let o: Vec<f64> = kin_steppers.iter().map(|_| 0.0).collect();
            return Ok((m, o));
        }
        let module = dc_module.as_ref().unwrap();
        let axes = module.get_axes();
        let orig_matr = matr.clone();
        for c in carriages.values() {
            let axis = c.get_axis();
            if axes.contains(&axis) {
                let (m, o) = module.get_transform(&carriage_rail(c));
                for s in carriage_rail(c).get_steppers() {
                    let name = s.get_name(Some(false));
                    let entry = match matr.get_mut(&name) {
                        Some(e) => e,
                        None => {
                            return Err(internal_err(
                                "KeyError",
                                pycompat::value::repr_key(&name),
                            ));
                        }
                    };
                    entry[axis as usize] *= m;
                    let off_entry = offs.get_mut(&name).unwrap();
                    off_entry[axis as usize] += o;
                }
            }
        }
        let m: Matrix = kin_steppers
            .iter()
            .map(|s| matr.get(&s.get_name(false)).unwrap().clone())
            .collect();
        let o: Vec<f64> = kin_steppers
            .iter()
            .map(|s| {
                let orig = orig_matr.get(&s.get_name(false)).unwrap();
                let off = offs.get(&s.get_name(false)).unwrap();
                let a = [orig[0], orig[1], orig[2]];
                let b = [off[0], off[1], off[2]];
                matrix_dot(&a, &b)
            })
            .collect();
        Ok((m, o))
    }

    pub fn _check_kinematics(&self, make_error: &dyn Fn(String) -> KlippyError) -> KResult<()> {
        check_kinematics_with_module(
            &self.kin_steppers,
            &self.carriages,
            &self.dc_module,
            make_error,
        )
    }

    pub fn calc_position_impl(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let (matr, offs) = Self::_get_kinematics_coeffs(
            &self.kin_steppers,
            &self.carriages,
            &self.dc_module,
        )?;
        let mut spos: Vec<f64> = Vec::with_capacity(self.kin_steppers.len());
        for s in &self.kin_steppers {
            let name = s.get_name(false);
            match stepper_positions.get(&name) {
                Some(v) => spos.push(*v),
                None => {
                    return Err(internal_err("KeyError", pycompat::value::repr_key(&name)));
                }
            }
        }
        let pinv = match pseudo_inverse(&matr)? {
            Some(p) => p,
            None => {
                return Err(internal_err(
                    "TypeError",
                    "'NoneType' object is not iterable".to_string(),
                ));
            }
        };
        let row: Matrix = vec![
            spos
                .iter()
                .zip(offs.iter())
                .map(|(sp, o)| sp - o)
                .collect(),
        ];
        let pinv_t = mat_transp(&pinv)?;
        let prod = mat_mat_mul(&row, &pinv_t)?.ok_or_else(|| {
            internal_err("TypeError", "'NoneType' object is not iterable".to_string())
        })?;
        let mut pos = prod[0].clone();
        for i in 0..pinv.len() {
            if pinv[i].iter().all(|x| *x == 0.0) {
                pos[i] = f64::NAN;
            }
        }
        let mut out: Vec<Option<f64>> = Vec::with_capacity(3);
        for i in 0..pos.len().min(3) {
            if pinv[i].iter().all(|x| *x == 0.0) {
                out.push(None);
            } else {
                out.push(Some(pos[i]));
            }
        }
        while out.len() < 3 {
            out.push(None);
        }
        Ok(out[..3].to_vec())
    }

    pub fn update_limits_impl(&self, i: i64, range: (f64, f64)) -> KResult<()> {
        let mut limits = self.limits.borrow_mut();
        let len = limits.len() as i64;
        let idx = if i < 0 { i + len } else { i };
        if idx < 0 || idx >= len {
            return Err(internal_err(
                "IndexError",
                "list index out of range".to_string(),
            ));
        }
        let idx = idx as usize;
        let (l, h) = limits[idx];
        if l <= h {
            limits[idx] = range;
        }
        Ok(())
    }

    pub fn set_position_impl(
        &self,
        newpos: Vec<f64>,
        homing_axes: &str,
    ) -> KResult<()> {
        for s in &self.kin_steppers {
            s.set_position([newpos[0], newpos[1], newpos[2]]);
        }
        for axis_name in homing_axes.chars() {
            let axis = match axis_name {
                'x' => 0i64,
                'y' => 1i64,
                'z' => 2i64,
                _ => {
                    return Err(internal_err(
                        "ValueError",
                        "substring not found".to_string(),
                    ));
                }
            };
            for c in self.carriages.values() {
                if c.get_axis() == axis && carriage_is_active(c)? {
                    let range = carriage_rail(c).get_range();
                    self.limits.borrow_mut()[axis as usize] = range;
                    break;
                }
            }
        }
        Ok(())
    }

    pub fn clear_homing_state_impl(&self, clear_axes: &str) {
        let mut limits = self.limits.borrow_mut();
        for (axis, axis_name) in (0usize..3).zip("xyz".chars()) {
            if clear_axes.contains(axis_name) {
                limits[axis] = (1.0, -1.0);
            }
        }
    }

    pub fn home_axis_impl(
        &self,
        homing_state: &HomingStateHandle,
        axis: i64,
        rail: &Rail,
    ) -> KResult<()> {
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos: Vec<Option<f64>> = vec![None; 4];
        homepos[axis as usize] = Some(hi.position_endstop);
        let mut forcepos = homepos.clone();
        if hi.positive_dir {
            forcepos[axis as usize] = Some(
                hi.position_endstop - 1.5 * (hi.position_endstop - position_min),
            );
        } else {
            forcepos[axis as usize] =
                Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
        }
        homing_state.home_rails(vec![rail.clone()], forcepos, homepos)
    }

    pub fn home_impl(&self, homing_state: &HomingStateHandle) -> KResult<()> {
        self._check_kinematics(&|m| command_err(m))?;
        let mut primary_map: IndexMap<i64, Arc<MainCarriage>> = IndexMap::new();
        for c in &self.primary_carriages {
            primary_map.insert(c.get_axis(), Arc::clone(c));
        }
        for axis in homing_state.get_axes() {
            if let Some(module) = &self.dc_module {
                if module.get_axes().contains(&axis) {
                    module.home(homing_state.clone(), axis)?;
                    continue;
                }
            }
            let carriage = match primary_map.get(&axis) {
                Some(c) => c.clone(),
                None => {
                    return Err(internal_err(
                        "KeyError",
                        axis.to_string(),
                    ));
                }
            };
            self.home_axis_impl(homing_state, axis, &carriage.get_rail())?;
        }
        Ok(())
    }

    fn _check_endstops(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let limits = self.limits.cloned();
        for (i, (lo, hi)) in limits.iter().enumerate().take(3) {
            let d = axes_d.0.get(i).copied().unwrap_or(0.0);
            if d != 0.0 {
                let e = end_pos.0.get(i).copied().unwrap_or(0.0);
                if e < *lo || e > *hi {
                    if lo > hi {
                        return Err(mv.move_error("Must home axis first".to_string()));
                    }
                    return Err(mv.move_error("Move out of range".to_string()));
                }
            }
        }
        Ok(())
    }

    pub fn check_move_impl(&self, mv: &dyn MoveApi) -> KResult<()> {
        let limits = self.limits.cloned();
        let end_pos = mv.end_pos();
        let xpos = end_pos.0.first().copied().unwrap_or(0.0);
        let ypos = end_pos.0.get(1).copied().unwrap_or(0.0);
        if xpos < limits[0].0
            || xpos > limits[0].1
            || ypos < limits[1].0
            || ypos > limits[1].1
        {
            self._check_endstops(mv)?;
        }
        let axes_d = mv.axes_d();
        let zd = axes_d.0.get(2).copied().unwrap_or(0.0);
        if zd == 0.0 {
            return Ok(());
        }
        self._check_endstops(mv)?;
        let z_ratio = mv.move_d() / zd.abs();
        mv.limit_speed(
            self.max_z_velocity * z_ratio,
            self.max_z_accel * z_ratio,
        )?;
        Ok(())
    }

    pub fn get_status_impl(&self, _eventtime: f64) -> PyValue {
        let limits = self.limits.cloned();
        let mut homed = String::new();
        for (a, (l, h)) in "xyz".chars().zip(limits.iter()) {
            if *l <= *h {
                homed.push(a);
            }
        }
        let mut ranges_min = Vec::with_capacity(3);
        let mut ranges_max = Vec::with_capacity(3);
        for axis in 0i64..3 {
            let mut min_v = f64::INFINITY;
            let mut max_v = f64::NEG_INFINITY;
            let mut first = true;
            for c in self.carriages.values() {
                if c.get_axis() == axis {
                    let (lo, hi) = carriage_rail(c).get_range();
                    if first {
                        min_v = lo;
                        max_v = hi;
                        first = false;
                    } else {
                        min_v = min_v.min(lo);
                        max_v = max_v.max(hi);
                    }
                }
            }
            ranges_min.push(min_v);
            ranges_max.push(max_v);
        }
        fn coord_tuple(v: &[f64]) -> PyValue {
            let mut t: Vec<PyValue> = v.iter().map(|x| PyValue::Float(*x)).collect();
            while t.len() < 4 {
                t.push(PyValue::Int(PyInt::from_i64(0)));
            }
            PyValue::Tuple(t)
        }
        let mut dict: IndexMap<PyValue, PyValue> = IndexMap::new();
        dict.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str(homed));
        dict.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_tuple(&ranges_min),
        );
        dict.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_tuple(&ranges_max),
        );
        PyValue::Dict(dict)
    }

    pub fn cmd_set_stepper_carriages(
        &self,
        gcmd: &klippy_api::gcode::GCmd,
    ) -> KResult<()> {
        let toolhead = match lookup_interface::<dyn klippy_api::toolhead::ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )? {
            Some(t) => t,
            None => {
                return Err(command_err("Unable to lookup object 'toolhead'".to_string()));
            }
        };
        toolhead.flush_step_generation()?;
        let stepper_name = match gcmd.get("STEPPER", GetOpts {
            default: None,
            parser: klippy_api::gcode::GetParser::Str,
            minval: None,
            maxval: None,
            above: None,
            below: None,
        })? {
            PyValue::Str(s) => s,
            _ => String::new(),
        };
        let steppers: Vec<&KinematicStepper> = self
            .kin_steppers
            .iter()
            .filter(|s| s.get_name(false) == stepper_name || s.get_name(true) == stepper_name)
            .collect();
        if steppers.len() != 1 {
            return Err(command_err(format!(
                "Invalid STEPPER '{}' specified",
                stepper_name
            )));
        }
        let stepper = steppers[0];
        let carriages_raw = match gcmd.get("CARRIAGES", GetOpts {
            default: None,
            parser: klippy_api::gcode::GetParser::Str,
            minval: None,
            maxval: None,
            above: None,
            below: None,
        })? {
            PyValue::Str(s) => s,
            _ => String::new(),
        };
        let carriages_str = carriages_raw.to_lowercase();
        let disable = gcmd.get_int(
            "DISABLE_CHECKS",
            GetIntOpts {
                default: Some(Some(0)),
                minval: None,
                maxval: None,
            },
        )?.unwrap_or(0);
        let validate = disable == 0;
        let old_carriages = stepper.get_carriages();
        let old_kin_coeffs = stepper.get_kin_coeffs();
        stepper.update_carriages(
            &carriages_str,
            &self.all_carriages,
            Some(&|m| command_err(m)),
        )?;
        let new_carriages = stepper.get_carriages();
        let same = old_carriages.len() == new_carriages.len()
            && old_carriages
                .iter()
                .zip(new_carriages.iter())
                .all(|(a, b)| Arc::ptr_eq(a, b));
        if !same {
            stepper.update_kin_coeffs(old_kin_coeffs)?;
            return Err(command_err(
                "SET_STEPPER_CARRIAGES cannot add or remove carriages that the stepper controls"
                    .to_string(),
            ));
        }
        let pos = toolhead.get_position();
        stepper.set_position([pos[0], pos[1], pos[2]]);
        if !validate {
            return Ok(());
        }
        if let Some(module) = &self.dc_module {
            let state = module.save_dual_carriage_state()?;
            let pcs: Vec<Option<KCarriage>> = self
                .dc_carriages
                .iter()
                .map(|dc| dc.get_dual_carriage())
                .collect();
            let dcs: Vec<Arc<DualCarriage>> = self
                .dc_carriages
                .iter()
                .map(Arc::clone)
                .collect();
            let pairs: Vec<(Option<KCarriage>, Arc<DualCarriage>)> =
                pcs.iter().cloned().zip(dcs.iter().cloned()).collect();
            let total = 1usize << pairs.len();
            for mask in 0..total {
                let mut combo: Vec<Option<KCarriage>> = Vec::with_capacity(pairs.len());
                for (idx, (pc, dc)) in pairs.iter().enumerate() {
                    // `itertools.product` varies the last pair fastest.
                    let bit = (mask >> (pairs.len() - 1 - idx)) & 1;
                    if bit == 0 {
                        combo.push(pc.clone());
                    } else {
                        let a: Arc<DualCarriage> = Arc::clone(dc);
                        combo.push(Some(a));
                    }
                }
                for c_opt in &combo {
                    let c = match c_opt {
                        Some(c) => c.clone(),
                        None => {
                            return Err(internal_err(
                                "AttributeError",
                                "'NoneType' object has no attribute 'get_rail'".to_string(),
                            ))
                        }
                    };
                    let wrapper = match module.get_dc_rail_wrapper(&carriage_rail(&c)) {
                        Some(w) => w,
                        None => {
                            return Err(internal_err(
                                "AttributeError",
                                "'NoneType' object has no attribute 'activate'".to_string(),
                            ))
                        }
                    };
                    wrapper.activate(PRIMARY, &pos, None)?;
                    let dc = match carriage_dual_carriage(&c) {
                        Some(d) => d,
                        None => {
                            return Err(internal_err(
                                "AttributeError",
                                "'NoneType' object has no attribute 'get_rail'".to_string(),
                            ))
                        }
                    };
                    let w2 = match module.get_dc_rail_wrapper(&carriage_rail(&dc)) {
                        Some(w) => w,
                        None => {
                            return Err(internal_err(
                                "AttributeError",
                                "'NoneType' object has no attribute 'inactivate'".to_string(),
                            ))
                        }
                    };
                    w2.inactivate(&pos)?;
                }
                self._check_kinematics(&|m| command_err(m))?;
            }
            module.restore_dual_carriage_state(&state, 0, 0.0)?;
        } else {
            self._check_kinematics(&|m| command_err(m))?;
        }
        Ok(())
    }
}

impl KinematicsApi for GenericCartesianKinematics {
    fn class_name(&self) -> &'static str {
        "GenericCartesianKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.kin_steppers
            .iter()
            .map(|s| s.get_stepper())
            .collect()
    }
    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        self.calc_position_impl(stepper_positions)
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.set_position_impl(newpos, homing_axes)
    }
    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        self.check_move_impl(mv)
    }
    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        self.home_impl(&homing_state)
    }
    fn get_status(&self, eventtime: f64) -> KResult<PyValue> {
        Ok(self.get_status_impl(eventtime))
    }
    fn clear_homing_state(&self, clear_axes: &str) -> KResult<()> {
        self.clear_homing_state_impl(clear_axes);
        Ok(())
    }
    fn home_axis(&self, homing_state: HomingStateHandle, axis: i64, rail: Rail) -> KResult<()> {
        self.home_axis_impl(&homing_state, axis, &rail)
    }
    fn update_limits(&self, axis: i64, range: (f64, f64)) -> KResult<()> {
        self.update_limits_impl(axis, range)
    }
}

pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    let kin = GenericCartesianKinematics::new(toolhead, config)?;
    let k: Kinematics = kin;
    Ok(k)
}

/// Kinematics modules are not loaded through `printer.load_object` (the
/// toolhead imports `kinematics.<name>` directly, `klippy/toolhead.py:242`),
/// so there is no `ModuleFactory` entry to expose.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

fn check_kinematics_with_module(
    kin_steppers: &[KinematicStepper],
    carriages: &IndexMap<String, KCarriage>,
    dc_module: &Option<Arc<DualCarriages>>,
    make_error: &dyn Fn(String) -> KlippyError,
) -> KResult<()> {
    let (matr, _offs) =
        GenericCartesianKinematics::_get_kinematics_coeffs(kin_steppers, carriages, dc_module)?;
    let mt = mat_transp(&matr)?;
    let mtm = match mat_mat_mul(&mt, &matr)? {
        Some(v) => v,
        None => {
            return Err(make_error(
                "Verify configured stepper(s) and their 'carriages' specifications, the current configuration does not allow independent movements of all printer axes.".to_string(),
            ));
        }
    };
    let rhs: Matrix = (0..mtm.len()).map(|_| Vec::new()).collect();
    if gaussian_solve(&mtm, &rhs, false)?.is_none() {
        return Err(make_error(
            "Verify configured stepper(s) and their 'carriages' specifications, the current configuration does not allow independent movements of all printer axes.".to_string(),
        ));
    }
    Ok(())
}
