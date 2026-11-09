//! Port of `klippy/kinematics/hybrid_corexz.py`.
//!
//! The hybrid-corexz kinematic is also known as Markforged kinematics.
//! Implements `HybridCoreXZKinematics` (`kinematics/hybrid_corexz.py:11-127`)
//! and `load_kinematics` (`kinematics/hybrid_corexz.py:128-129`). The module
//! loads steppers for `stepper_x/y/z` (plus an optional fourth rail for
//! `dual_carriage`), wires the Z endstop onto the X stepper, sets up the
//! CoreXZ iterator on X (and on the dual-carriage rail when configured)
//! and the Cartesian iterator on Y and Z, records the axis ranges and Z
//! limits, and implements position, homing and move checks with the same
//! order of operations and float arithmetic as the Python.
//!
//! Rails come from the toolhead's stepper factory
//! (`ToolHeadApi::stepper_factory`, the `stepper.LookupRail` /
//! `stepper.LookupMultiRail` calls), and the dual-carriage object is
//! constructed by `crate::idex_modes::DualCarriages::new` (the
//! `idex_modes.DualCarriages` call). Homing goes through
//! `klippy_api::toolhead::HomingStateApi` directly.

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::{PyInt, PyValue};
use klippy_api::config::{Config, GetChoiceOpts, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{Coord, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead};

use crate::idex_modes::{DualCarriages, DualCarriagesApi};

/// `axes_min`/`axes_max` as exposed by `get_status`: `gcode.Coord` pads the
/// three range bounds with the int `0` (`klippy/gcode.py:12`), so the
/// fourth tuple element is an int, not a float (`hybrid_corexz.py:24-25,123-124`).
fn coord_status_value(coord: &Coord) -> PyValue {
    PyValue::Tuple(vec![
        PyValue::Float(coord.0[0]),
        PyValue::Float(coord.0[1]),
        PyValue::Float(coord.0[2]),
        PyValue::Int(PyInt::from_i64(0)),
    ])
}

/// `gcode.Coord(t)` (`klippy/gcode.py:12`): tuples are padded to 4 elements
/// with `0.0`. `hybrid_corexz.py:24-25` builds `Coord` from three range
/// bounds, so the stored `axes_min`/`axes_max` have a zero `e` component.
fn coord_from_3(v: [f64; 3]) -> Coord {
    Coord(vec![v[0], v[1], v[2], 0.0])
}

/// `IndexError("list index out of range")` for an out-of-range list index,
/// exactly as CPython raises it.
fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `seq[i]` index with Python list semantics (negative indices wrap,
/// out-of-range indices raise `IndexError`).
fn list_index(len: usize, i: i64) -> KResult<usize> {
    let len = len as i64;
    let idx = if i < 0 { i + len } else { i };
    if idx < 0 || idx >= len {
        return Err(index_error());
    }
    Ok(idx as usize)
}

/// `HybridCoreXZKinematics` (`klippy/kinematics/hybrid_corexz.py:11`).
pub struct HybridCoreXZKinematics {
    printer: Printer,
    rails: Vec<Rail>,
    dc_module: Option<Arc<dyn DualCarriagesApi>>,
    axes_min: Coord,
    axes_max: Coord,
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: GCell<[(f64, f64); 3]>,
}

impl HybridCoreXZKinematics {
    /// Direct construction from already-resolved parts. Tests build the
    /// kinematics through fakes with this constructor; [`load_kinematics`]
    /// calls it after reading the config and creating the rails.
    pub fn new(
        printer: Printer,
        rails: Vec<Rail>,
        dc_module: Option<Arc<dyn DualCarriagesApi>>,
        axes_min: Coord,
        axes_max: Coord,
        max_z_velocity: f64,
        max_z_accel: f64,
    ) -> Arc<HybridCoreXZKinematics> {
        Arc::new(HybridCoreXZKinematics {
            printer,
            rails,
            dc_module,
            axes_min,
            axes_max,
            max_z_velocity,
            max_z_accel,
            limits: GCell::new([(1.0, -1.0); 3]),
        })
    }

    /// `printer` stored on the object (`config.get_printer()`).
    pub fn printer(&self) -> &Printer {
        &self.printer
    }

    /// Axis rails in order: X, Y, Z, plus the dual-carriage rail when
    /// configured (`hybrid_corexz.py:15-31`).
    pub fn rails(&self) -> &[Rail] {
        &self.rails
    }

    /// `dc_module`, when a `[dual_carriage]` section is configured.
    pub fn dc_module(&self) -> Option<&Arc<dyn DualCarriagesApi>> {
        self.dc_module.as_ref()
    }

    /// `axes_min` (`gcode.Coord` of the rail lower bounds, 4 elements).
    pub fn axes_min(&self) -> &Coord {
        &self.axes_min
    }

    /// `axes_max` (`gcode.Coord` of the rail upper bounds, 4 elements).
    pub fn axes_max(&self) -> &Coord {
        &self.axes_max
    }

    /// `max_z_velocity` (config `max_z_velocity`, default `max_velocity`).
    pub fn max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }

    /// `max_z_accel` (config `max_z_accel`, default `max_accel`).
    pub fn max_z_accel(&self) -> f64 {
        self.max_z_accel
    }

    /// Snapshot of `limits` (`[(1.0, -1.0)] * 3` when un-homed).
    pub fn limits(&self) -> [(f64, f64); 3] {
        self.limits.get()
    }

    /// `_check_endstops(move)` (`hybrid_corexz.py:97-105`).
    fn check_endstops(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let limits = self.limits.get();
        for (i, &(lo, hi)) in limits.iter().enumerate() {
            let axis_d = axes_d
                .0
                .get(i)
                .copied()
                .ok_or_else(index_error)?;
            if axis_d != 0.0 {
                let ep = end_pos.0.get(i).copied().ok_or_else(index_error)?;
                if ep < lo || ep > hi {
                    if lo > hi {
                        return Err(mv.move_error("Must home axis first".to_string()));
                    }
                    return Err(mv.move_error("Move out of range".to_string()));
                }
            }
        }
        Ok(())
    }

    /// `home_axis` logic shared by [`KinematicsApi::home_axis`] and
    /// [`KinematicsApi::home`]: compute the force/home positions and call
    /// `homing_state.home_rails([rail], forcepos, homepos)`.
    fn home_axis_impl(
        &self,
        homing_state: &HomingStateHandle,
        axis: i64,
        rail: &Rail,
    ) -> KResult<()> {
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        // `homepos[axis] = hi.position_endstop` (`hybrid_corexz.py:84`).
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
}

impl KinematicsApi for HybridCoreXZKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        let mut out = Vec::new();
        for rail in &self.rails {
            for stepper in rail.get_steppers() {
                out.push(stepper);
            }
        }
        out
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut pos = Vec::with_capacity(self.rails.len());
        for rail in &self.rails {
            let name = rail.get_name(None);
            let p = stepper_positions.get(&name).ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: PyValue::Str(name.clone()).repr(),
            })?;
            pos.push(*p);
        }
        // `self.dc_module.get_status()['carriage_1'] == 'PRIMARY'`
        // (`hybrid_corexz.py:53-54`): `carriage_1` is the dual rail's mode.
        let carriage1_primary = match (&self.dc_module, self.rails.get(3)) {
            (Some(dc), Some(dual_rail)) => match dc.get_dc_rail_wrapper(dual_rail) {
                Some(wrapper) => wrapper.mode() == "PRIMARY",
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: "'carriage_1'".to_string(),
                    })
                }
            },
            _ => false,
        };
        if carriage1_primary {
            Ok(vec![
                Some(pos[3] - pos[2]),
                Some(pos[1]),
                Some(pos[2]),
            ])
        } else {
            Ok(vec![
                Some(pos[0] + pos[2]),
                Some(pos[1]),
                Some(pos[2]),
            ])
        }
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(index_error());
        }
        let coord = [newpos[0], newpos[1], newpos[2]];
        for rail in &self.rails {
            rail.set_position(coord);
        }
        for axis_name in homing_axes.chars() {
            let axis = "xyz".find(axis_name).ok_or_else(|| KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "substring not found".to_string(),
            })? as i64;
            let idx = axis as usize;
            let rail: Rail = if let Some(dc) = &self.dc_module {
                if axis == 0 {
                    dc.get_primary_rail(0)
                        .unwrap_or_else(|| self.rails[idx].clone())
                } else {
                    self.rails[idx].clone()
                }
            } else {
                self.rails[idx].clone()
            };
            let range = rail.get_range();
            self.limits.with_mut(|lim: &mut [(f64, f64); 3]| {
                lim[idx] = range;
            });
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        if end_pos.0.len() < 2 {
            // `xpos, ypos = move.end_pos[:2]` (`hybrid_corexz.py:108`).
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: format!(
                    "not enough values to unpack (expected 2, got {})",
                    end_pos.0.len()
                ),
            });
        }
        let xpos = end_pos.0[0];
        let ypos = end_pos.0[1];
        let limits = self.limits.get();
        if xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1 {
            self.check_endstops(mv)?;
        }
        let axes_d = mv.axes_d();
        if axes_d.0.len() < 3 {
            return Err(index_error());
        }
        if axes_d.0[2] == 0.0 {
            return Ok(());
        }
        self.check_endstops(mv)?;
        let z_ratio = mv.move_d() / axes_d.0[2].abs();
        mv.limit_speed(
            self.max_z_velocity * z_ratio,
            self.max_z_accel * z_ratio,
        );
        Ok(())
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        let axes = homing_state.get_axes();
        for axis in axes {
            if self.dc_module.is_some() && axis == 0 {
                self.dc_module
                    .as_ref()
                    .expect("dc_module checked present above")
                    .home(homing_state.clone(), axis)?;
                continue;
            }
            let idx = list_index(self.rails.len(), axis)?;
            let rail = self.rails[idx].clone();
            self.home_axis_impl(&homing_state, axis, &rail)?;
        }
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let limits = self.limits.get();
        let mut axes = String::new();
        for (axis, (l, h)) in "xyz".chars().zip(limits.iter()) {
            if l <= h {
                axes.push(axis);
            }
        }
        let mut dict = IndexMap::new();
        dict.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str(axes));
        dict.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_status_value(&self.axes_min),
        );
        dict.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_status_value(&self.axes_max),
        );
        PyValue::Dict(dict)
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        let limits = self.limits.get();
        let mut updated = limits;
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                updated[axis] = (1.0, -1.0);
            }
        }
        self.limits.set(updated);
    }

    fn home_axis(
        &self,
        homing_state: HomingStateHandle,
        axis: i64,
        rail: Rail,
    ) -> KResult<()> {
        self.home_axis_impl(&homing_state, axis, &rail)
    }

    fn update_limits(&self, axis: i64, range: (f64, f64)) -> KResult<()> {
        // `self.limits[i]` with Python index semantics (`hybrid_corexz.py:58`).
        let idx = list_index(3, axis)?;
        let (l, h) = self.limits.get()[idx];
        // Only update limits if this axis was already homed (`hybrid_corexz.py:60-62`).
        if l <= h {
            self.limits.with_mut(|lim: &mut [(f64, f64); 3]| {
                lim[idx] = range;
            });
        }
        Ok(())
    }
}

impl klippy_api::PrinterObject for HybridCoreXZKinematics {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn KinematicsApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(KinematicsApi::get_status(self, eventtime))
    }
}

/// `load_kinematics(toolhead, config)` (`hybrid_corexz.py:128-129`).
///
/// Creates the X rail through the stepper factory's `lookup_rail` and the
/// Y/Z rails through `lookup_multi_rail` (`hybrid_corexz.py:15-17`), wires
/// the X stepper onto the Z endstop (`hybrid_corexz.py:18-19`), sets up the
/// CoreXZ allocator on X and the Cartesian allocator on Y and Z
/// (`hybrid_corexz.py:20-22`), reads the axis ranges and Z limits,
/// optionally creates the dual-carriage rail and constructs the
/// dual-carriage object (`hybrid_corexz.py:27-39`), assigns the toolhead
/// trapq to every stepper (`hybrid_corexz.py:40-41`), and returns the
/// kinematics object itself (`hybrid_corexz.py:128-129`).
pub fn load_kinematics(
    toolhead: &ToolHead,
    config: &Config,
) -> KResult<Kinematics> {
    let printer = config.get_printer();
    let factory = toolhead.stepper_factory();

    let mut rails: Vec<Rail> = Vec::new();
    rails.push(factory.lookup_rail(&config.getsection("stepper_x"), RailOpts::default())?);
    rails.push(factory.lookup_multi_rail(&config.getsection("stepper_y"), RailOpts::default())?);
    rails.push(factory.lookup_multi_rail(&config.getsection("stepper_z"), RailOpts::default())?);

    {
        let endstops = rails[2].get_endstops();
        let entry = endstops.first().ok_or_else(index_error)?;
        let steppers = rails[0].get_steppers();
        let stepper = steppers.first().ok_or_else(index_error)?;
        entry.0.add_stepper(stepper.clone());
    }

    rails[0].setup_itersolve(
        "corexz_stepper_alloc",
        &[PyValue::Bytes(vec![b'-'])],
    );
    rails[1].setup_itersolve(
        "cartesian_stepper_alloc",
        &[PyValue::Bytes(vec![b'y'])],
    );
    rails[2].setup_itersolve(
        "cartesian_stepper_alloc",
        &[PyValue::Bytes(vec![b'z'])],
    );

    let ranges: Vec<(f64, f64)> = rails.iter().map(|rail| rail.get_range()).collect();
    let axes_min = coord_from_3([ranges[0].0, ranges[1].0, ranges[2].0]);
    let axes_max = coord_from_3([ranges[0].1, ranges[1].1, ranges[2].1]);

    let mut dc_module: Option<Arc<dyn DualCarriagesApi>> = None;
    if config.has_section("dual_carriage") {
        let dc_config = config.getsection("dual_carriage");
        // Dummy for cartesian config users (`hybrid_corexz.py:31`).
        let choices = PyValue::List(vec![PyValue::Str("x".to_string())]);
        let _ = dc_config.getchoice(
            "axis",
            &choices,
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("x".to_string()))),
                note_valid: true,
            },
        )?;
        let dc_rail = factory.lookup_rail(&dc_config, RailOpts::default())?;
        rails.push(dc_rail);
        {
            let endstops = rails[2].get_endstops();
            let entry = endstops.first().ok_or_else(index_error)?;
            let steppers = rails[3].get_steppers();
            let stepper = steppers.first().ok_or_else(index_error)?;
            entry.0.add_stepper(stepper.clone());
        }
        rails[3].setup_itersolve(
            "corexz_stepper_alloc",
            &[PyValue::Bytes(vec![b'+'])],
        );
        let safe_dist = dc_config.getfloat(
            "safe_distance",
            GetFloatOpts {
                default: Some(None),
                minval: Some(0.0),
                maxval: None,
                above: None,
                below: None,
                note_valid: true,
            },
        )?;
        let dc: Arc<dyn DualCarriagesApi> = DualCarriages::new(
            printer.clone(),
            vec![rails[0].clone()],
            vec![Some(rails[3].clone())],
            vec![0],
            vec![safe_dist],
        )?;
        dc_module = Some(dc);
    }

    let trapq = toolhead.get_trapq();
    for stepper in rails.iter().flat_map(|rail| rail.get_steppers()) {
        stepper.set_trapq(Some(trapq.clone()));
    }

    let (max_velocity, max_accel) = toolhead.get_max_velocity();
    let max_z_velocity = config
        .getfloat(
            "max_z_velocity",
            GetFloatOpts {
                default: Some(Some(max_velocity)),
                minval: None,
                maxval: Some(max_velocity),
                above: Some(0.0),
                below: None,
                note_valid: true,
            },
        )?
        .unwrap_or(max_velocity);
    let max_z_accel = config
        .getfloat(
            "max_z_accel",
            GetFloatOpts {
                default: Some(Some(max_accel)),
                minval: None,
                maxval: Some(max_accel),
                above: Some(0.0),
                below: None,
                note_valid: true,
            },
        )?
        .unwrap_or(max_accel);

    Ok(HybridCoreXZKinematics::new(
        printer,
        rails,
        dc_module,
        axes_min,
        axes_max,
        max_z_velocity,
        max_z_accel,
    ))
}

/// Kinematics modules are not loaded through `printer.load_object` (the
/// toolhead imports `kinematics.<name>` directly, `klippy/toolhead.py:242`),
/// so there is no `ModuleFactory` entry to expose. This function exists
/// for interface parity with other modules and returns the empty factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
