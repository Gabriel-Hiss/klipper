//! Port of `klippy/kinematics/hybrid_corexy.py`.
//!
//! `HybridCoreXYKinematics` (the Markforged kinematics, `hybrid_corexy.py:12`)
//! drives X with a single `corexy_stepper_alloc` `'-'` stepper, Y and Z with
//! `cartesian_stepper_alloc` rails, and shares the Y endstop with the X
//! stepper. A `[dual_carriage]` section appends a fourth rail allocated with
//! `corexy_stepper_alloc` `'+'` and hands it to
//! `idex_modes.DualCarriages` (`hybrid_corexy.py:26-39`).
//!
//! Rails come from the toolhead's stepper factory (`stepper.LookupRail` /
//! `stepper.LookupMultiRail`), so the module reads config through the factory
//! exactly where Python does, and homing goes through
//! `klippy_api::toolhead::HomingStateApi` directly.
//!
//! `self.dc_module` holds the concrete `idex_modes.DualCarriages` object, as
//! Python does: `calc_position` reads `dc_module.get_status()['carriage_1']`
//! (`hybrid_corexy.py:53-54`), and `DualCarriagesApi` does not expose
//! `get_status` yet (reported contract gap).

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::{PyInt, PyValue};

use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts};
use klippy_api::gcell::GCell;
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{
    Coord, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_api::{KResult, KlippyError};

use crate::idex_modes::{DualCarriages, PRIMARY};

/// `IndexError("list index out of range")`, as CPython raises it.
fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `seq[i]` with Python list semantics (negative indices wrap,
/// out-of-range indices raise `IndexError`).
fn list_index(len: usize, i: i64) -> KResult<usize> {
    let len = len as i64;
    let idx = if i < 0 { i + len } else { i };
    if idx < 0 || idx >= len {
        return Err(index_error());
    }
    Ok(idx as usize)
}

/// `values[i]` with Python list semantics.
fn pos_at(values: &[f64], i: i64) -> KResult<f64> {
    let idx = list_index(values.len(), i)?;
    Ok(values[idx])
}

/// `KeyError(name)` with `str(exc)` being the key's `repr`.
fn key_error(name: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: PyValue::Str(name.to_string()).repr(),
    }
}

fn value_error(message: String) -> KlippyError {
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message,
    }
}

fn type_error(message: String) -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message,
    }
}

/// `status[key]` on the status dict, raising the Python `KeyError` /
/// `TypeError` a non-dict or missing key produces (`hybrid_corexy.py:53-54`).
fn status_field(status: &PyValue, key: &str) -> KResult<PyValue> {
    match status {
        PyValue::Dict(dict) => dict
            .get(&PyValue::Str(key.to_string()))
            .cloned()
            .ok_or_else(|| key_error(key)),
        other => Err(type_error(format!(
            "'{}' object is not subscriptable",
            pycompat::value::type_name(other)
        ))),
    }
}

/// `toolhead.Coord([...])` (`klippy/gcode.py:12`): pads to 4 elements with
/// the int `0`, so the stored `axes_min`/`axes_max` carry a zero `e`.
fn coord_from_3(v: [f64; 3]) -> Coord {
    Coord(vec![v[0], v[1], v[2], 0.0])
}

/// The `Coord` as the Python tuple `get_status` exposes: three floats and
/// the int `0` padding (`klippy/gcode.py:15`).
fn coord_status_value(coord: &Coord) -> PyValue {
    PyValue::Tuple(vec![
        PyValue::Float(coord.0[0]),
        PyValue::Float(coord.0[1]),
        PyValue::Float(coord.0[2]),
        PyValue::Int(PyInt::from_i64(0)),
    ])
}

/// `self.rails[1].get_endstops()[0][0]` of the shared Y endstop.
fn first_endstop(rail: &Rail) -> KResult<klippy_api::mcu::Endstop> {
    let mut endstops = rail.get_endstops();
    if endstops.is_empty() {
        return Err(index_error());
    }
    Ok(endstops.remove(0).0)
}

/// `rail.get_steppers()[0]` (`hybrid_corexy.py:20,32`).
fn first_stepper(rail: &Rail) -> KResult<Stepper> {
    let mut steppers = rail.get_steppers();
    if steppers.is_empty() {
        return Err(index_error());
    }
    Ok(steppers.remove(0))
}

/// Every stepper of every rail, in rail order (`hybrid_corexy.py:40`).
fn steppers_of(rails: &[Rail]) -> Vec<Stepper> {
    let mut out = Vec::new();
    for rail in rails {
        out.extend(rail.get_steppers());
    }
    out
}

/// `HybridCoreXYKinematics` (`klippy/kinematics/hybrid_corexy.py:12`).
pub struct HybridCoreXYKinematics {
    rails: Vec<Rail>,
    dc_module: Option<Arc<DualCarriages>>,
    axes_min: Coord,
    axes_max: Coord,
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: GCell<[(f64, f64); 3]>,
}

impl HybridCoreXYKinematics {
    /// The object `HybridCoreXYKinematics.__init__` produces once
    /// [`load_kinematics`] has resolved the rails, the optional dual
    /// carriage and the limits.
    fn new(
        rails: Vec<Rail>,
        dc_module: Option<Arc<DualCarriages>>,
        axes_min: Coord,
        axes_max: Coord,
        max_z_velocity: f64,
        max_z_accel: f64,
    ) -> Self {
        HybridCoreXYKinematics {
            rails,
            dc_module,
            axes_min,
            axes_max,
            max_z_velocity,
            max_z_accel,
            limits: GCell::new([(1.0f64, -1.0f64); 3]),
        }
    }

    /// `self.rails` (X, Y, Z, plus the dual-carriage rail when configured).
    pub fn rails(&self) -> &[Rail] {
        &self.rails
    }

    /// `self.dc_module`, set only when a `[dual_carriage]` section exists.
    pub fn dc_module(&self) -> Option<&Arc<DualCarriages>> {
        self.dc_module.as_ref()
    }

    /// `self.axes_min` (the three range lower bounds plus the `0` padding).
    pub fn axes_min(&self) -> &Coord {
        &self.axes_min
    }

    /// `self.axes_max`.
    pub fn axes_max(&self) -> &Coord {
        &self.axes_max
    }

    /// `self.max_z_velocity`.
    pub fn max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }

    /// `self.max_z_accel`.
    pub fn max_z_accel(&self) -> f64 {
        self.max_z_accel
    }

    /// Snapshot of `self.limits` (`[(1.0, -1.0)] * 3` while un-homed).
    pub fn limits(&self) -> [(f64, f64); 3] {
        self.limits.get()
    }

    /// `get_steppers` (`hybrid_corexy.py:41`).
    pub fn get_steppers(&self) -> Vec<Stepper> {
        steppers_of(&self.rails)
    }

    /// `calc_position` (`hybrid_corexy.py:42`). The dual carriage, when its
    /// second carriage is the active primary, places X on that carriage:
    /// `pos[3] - pos[1]` instead of `pos[0] + pos[1]`.
    pub fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut pos = Vec::with_capacity(self.rails.len());
        for rail in &self.rails {
            let name = rail.get_name(None);
            let value = *stepper_positions.get(&name).ok_or_else(|| key_error(&name))?;
            pos.push(value);
        }
        let dc_primary = match &self.dc_module {
            Some(dc) => {
                let value = status_field(&dc.get_status(0.0), "carriage_1")?;
                matches!(&value, PyValue::Str(mode) if mode == PRIMARY)
            }
            None => false,
        };
        if dc_primary {
            return Ok(vec![
                Some(pos_at(&pos, 3)? - pos_at(&pos, 1)?),
                Some(pos_at(&pos, 1)?),
                Some(pos_at(&pos, 2)?),
            ]);
        }
        Ok(vec![
            Some(pos_at(&pos, 0)? + pos_at(&pos, 1)?),
            Some(pos_at(&pos, 1)?),
            Some(pos_at(&pos, 2)?),
        ])
    }

    /// `update_limits` (`hybrid_corexy.py:56`): only an already-homed axis
    /// takes the new range.
    pub fn update_limits(&self, i: i64, range: (f64, f64)) -> KResult<()> {
        let idx = list_index(3, i)?;
        let (l, h) = self.limits.get()[idx];
        if l <= h {
            self.limits.with_mut(|limits| limits[idx] = range);
        }
        Ok(())
    }

    /// `set_position` (`hybrid_corexy.py:64`).
    pub fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(index_error());
        }
        let coord: [f64; 3] = [newpos[0], newpos[1], newpos[2]];
        for rail in &self.rails {
            rail.set_position(coord);
        }
        for axis_name in homing_axes.chars() {
            let axis = match "xyz".find(axis_name) {
                Some(axis) => axis as i64,
                None => return Err(value_error("substring not found".to_string())),
            };
            let mut rail = None;
            if self.dc_module.is_some() && axis == 0 {
                rail = self
                    .dc_module
                    .as_ref()
                    .and_then(|dc| dc.get_primary_rail(axis));
            }
            let rail = match rail {
                Some(rail) => rail,
                None => {
                    let idx = list_index(self.rails.len(), axis)?;
                    self.rails[idx].clone()
                }
            };
            let idx = list_index(3, axis)?;
            self.limits.with_mut(|limits| limits[idx] = rail.get_range());
        }
        Ok(())
    }

    /// `clear_homing_state` (`hybrid_corexy.py:76`).
    pub fn clear_homing_state(&self, clear_axes: &str) {
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                self.limits.with_mut(|limits| limits[axis] = (1.0, -1.0));
            }
        }
    }

    /// `home_axis` (`hybrid_corexy.py:81`): force position 1.5 ranges away
    /// from the endstop on the homing side, then home the rail.
    pub fn home_axis(
        &self,
        homing_state: &HomingStateHandle,
        axis: i64,
        rail: &Rail,
    ) -> KResult<()> {
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let slot = list_index(4, axis)?;
        let mut homepos: Vec<Option<f64>> = vec![None, None, None, None];
        homepos[slot] = Some(hi.position_endstop);
        let mut forcepos = homepos.clone();
        if hi.positive_dir {
            forcepos[slot] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
        } else {
            forcepos[slot] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
        }
        homing_state.home_rails(vec![rail.clone()], forcepos, homepos)
    }

    /// `home` (`hybrid_corexy.py:92`): axis 0 belongs to the dual carriage
    /// when one is configured.
    pub fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        for axis in homing_state.get_axes() {
            if self.dc_module.is_some() && axis == 0 {
                if let Some(dc) = &self.dc_module {
                    dc.home(homing_state.clone(), axis)?;
                }
                continue;
            }
            let idx = list_index(self.rails.len(), axis)?;
            let rail = self.rails[idx].clone();
            self.home_axis(&homing_state, axis, &rail)?;
        }
        Ok(())
    }

    /// `_check_endstops` (`hybrid_corexy.py:98`).
    fn check_endstops(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let limits = self.limits.get();
        for (i, &(lo, hi)) in limits.iter().enumerate() {
            let axis_d = axes_d.0.get(i).copied().ok_or_else(index_error)?;
            if axis_d != 0.0 {
                let end = end_pos.0.get(i).copied().ok_or_else(index_error)?;
                if end < lo || end > hi {
                    if lo > hi {
                        return Err(mv.move_error("Must home axis first".to_string()));
                    }
                    return Err(mv.move_error("Move out of range".to_string()));
                }
            }
        }
        Ok(())
    }

    /// `check_move` (`hybrid_corexy.py:109`).
    pub fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let limits = self.limits.get();
        let end_pos = mv.end_pos();
        if end_pos.0.len() < 2 {
            return Err(value_error(format!(
                "not enough values to unpack (expected 2, got {})",
                end_pos.0.len()
            )));
        }
        let xpos = end_pos.0[0];
        let ypos = end_pos.0[1];
        if xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1 {
            self.check_endstops(mv)?;
        }
        let axes_d = mv.axes_d();
        let zd = axes_d.0.get(2).copied().ok_or_else(index_error)?;
        if zd == 0.0 {
            // Normal XY move - use defaults.
            return Ok(());
        }
        // Move with Z - update velocity and accel for slower Z axis.
        self.check_endstops(mv)?;
        let z_ratio = mv.move_d() / zd.abs();
        mv.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)?;
        Ok(())
    }

    /// `get_status` (`hybrid_corexy.py:120`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let limits = self.limits.get();
        let mut axes = String::new();
        for (axis, (l, h)) in "xyz".chars().zip(limits.iter()) {
            if l <= h {
                axes.push(axis);
            }
        }
        let mut status: IndexMap<PyValue, PyValue> = IndexMap::new();
        status.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str(axes));
        status.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_status_value(&self.axes_min),
        );
        status.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_status_value(&self.axes_max),
        );
        PyValue::Dict(status)
    }
}

impl KinematicsApi for HybridCoreXYKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.get_steppers()
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        self.calc_position(stepper_positions)
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.set_position(newpos, homing_axes)
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        self.check_move(mv)
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        self.home(homing_state)
    }

    fn get_status(&self, eventtime: f64) -> PyValue {
        self.get_status(eventtime)
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        self.clear_homing_state(clear_axes);
    }

    fn home_axis(
        &self,
        homing_state: HomingStateHandle,
        axis: i64,
        rail: Rail,
    ) -> KResult<()> {
        self.home_axis(&homing_state, axis, &rail)
    }

    fn update_limits(&self, axis: i64, range: (f64, f64)) -> KResult<()> {
        self.update_limits(axis, range)
    }
}

impl klippy_api::PrinterObject for HybridCoreXYKinematics {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn KinematicsApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// The `HybridCoreXYKinematics(toolhead, config)` constructor
/// (`hybrid_corexy.py:13`).
///
/// Builds the X rail with `stepper.LookupRail` and Y/Z with
/// `stepper.LookupMultiRail`, cross-links the X stepper onto the shared Y
/// endstop, sets up the iterators, records the axis ranges, optionally
/// appends the `[dual_carriage]` rail and its `DualCarriages` controller,
/// assigns the toolhead trapq to every stepper, and reads the Z limits.
pub fn build(toolhead: &ToolHead, config: &Config) -> KResult<Arc<HybridCoreXYKinematics>> {
    let printer = config.get_printer();
    let factory = toolhead.stepper_factory();

    let mut rails = vec![
        factory.lookup_rail(&config.getsection("stepper_x"), RailOpts::default())?,
        factory.lookup_multi_rail(&config.getsection("stepper_y"), RailOpts::default())?,
        factory.lookup_multi_rail(&config.getsection("stepper_z"), RailOpts::default())?,
    ];
    // The X stepper hangs off the Y endstop (`hybrid_corexy.py:20`).
    let y_endstop = first_endstop(&rails[1])?;
    y_endstop.add_stepper(first_stepper(&rails[0])?);

    rails[0].setup_itersolve("corexy_stepper_alloc", &[PyValue::Bytes(vec![b'-'])]);
    rails[1].setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'y'])]);
    rails[2].setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'z'])]);

    let ranges = [rails[0].get_range(), rails[1].get_range(), rails[2].get_range()];
    let axes_min = coord_from_3([ranges[0].0, ranges[1].0, ranges[2].0]);
    let axes_max = coord_from_3([ranges[0].1, ranges[1].1, ranges[2].1]);

    let mut dc_module = None;
    if config.has_section("dual_carriage") {
        let dc_config = config.getsection("dual_carriage");
        // Dummy for cartesian config users (`hybrid_corexy.py:29`).
        dc_config.getchoice(
            "axis",
            &PyValue::List(vec![PyValue::Str("x".to_string())]),
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("x".to_string()))),
                note_valid: true,
            },
        )?;
        rails.push(factory.lookup_rail(&dc_config, RailOpts::default())?);
        let dc_endstop = first_endstop(&rails[1])?;
        dc_endstop.add_stepper(first_stepper(&rails[3])?);
        rails[3].setup_itersolve("corexy_stepper_alloc", &[PyValue::Bytes(vec![b'+'])]);
        let safe_distance = dc_config.getfloat(
            "safe_distance",
            GetFloatOpts {
                default: Some(None),
                minval: Some(0.0),
                ..Default::default()
            },
        )?;
        dc_module = Some(DualCarriages::new(
            printer,
            vec![rails[0].clone()],
            vec![Some(rails[3].clone())],
            vec![0],
            vec![safe_distance],
        )?);
    }

    let trapq = toolhead.get_trapq();
    for stepper in steppers_of(&rails) {
        stepper.set_trapq(Some(trapq.clone()));
    }

    let (max_velocity, max_accel) = toolhead.get_max_velocity();
    let max_z_velocity = config
        .getfloat(
            "max_z_velocity",
            GetFloatOpts {
                default: Some(Some(max_velocity)),
                above: Some(0.0),
                maxval: Some(max_velocity),
                ..Default::default()
            },
        )?
        .unwrap_or(max_velocity);
    let max_z_accel = config
        .getfloat(
            "max_z_accel",
            GetFloatOpts {
                default: Some(Some(max_accel)),
                above: Some(0.0),
                maxval: Some(max_accel),
                ..Default::default()
            },
        )?
        .unwrap_or(max_accel);

    Ok(Arc::new(HybridCoreXYKinematics::new(
        rails,
        dc_module,
        axes_min,
        axes_max,
        max_z_velocity,
        max_z_accel,
    )))
}

/// `load_kinematics(toolhead, config)` (`hybrid_corexy.py:128-129`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    let kin = build(toolhead, config)?;
    Ok(kin as Kinematics)
}

/// Kinematics modules are not printer sections (the toolhead imports
/// `kinematics.<name>` by the `[printer] kinematics:` name,
/// `klippy/toolhead.py:242`); the Python module has no factory entry point,
/// so the factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
