//! Port of the `delta` module.
//!
//! `DeltaKinematics` / `DeltaCalibration` reproduce
//! `klippy/kinematics/delta.py` line by line: configuration order, boundary
//! checks, tower geometry, trilateration, move limiting and calibration
//! math. Floating point follows the Python source operation for
//! operation: transcendentals go through `pycompat::libm` (system libm,
//! matching CPython 3.8.2 on glibc 2.29), `math.radians` is
//! `a * pi / 180` in that order, and log lines are formatted with
//! `pycompat::text::percent_format` so `%.2f` matches CPython exactly.
//! Python `** 2` on floats matches IEEE `x * x` bit for bit (both are
//! correctly rounded), so squares use multiplication.
//!
//! The module has no printer object and no G-code commands:
//! [`module_factory`] is empty, mirroring `clocksync`. `toolhead.py`
//! constructs the kinematics directly via `load_kinematics`.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, PrinterConfig};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{
    Coord, DeltaCalibrationApi, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_api::lookup_interface;

use pycompat::libm;
use pycompat::PyValue;
use pycompat::text::{percent_format, PctValue, PyError};

/// `SLOW_RATIO = 3.` (`delta.py:10`).
pub const SLOW_RATIO: f64 = 3.0;

/// `math.radians` (`Modules/mathmodule.c`): `x * pi / 180`, in that order.
fn radians(x: f64) -> f64 {
    x * std::f64::consts::PI / 180.0
}

/// `math.sqrt` with CPython's domain check (`ValueError`).
fn math_sqrt(x: f64) -> Result<f64, PyError> {
    if x < 0.0 {
        return Err(PyError::new("ValueError", "math domain error".to_string()));
    }
    Ok(libm::sqrt(x))
}

/// `a ** 2` (see module docs).
fn sq(x: f64) -> f64 {
    x * x
}

fn key_error(name: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: pycompat::text::str_repr(name),
    }
}

fn index_error_list() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

fn py_error(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

fn py_sqrt(x: f64) -> KResult<f64> {
    math_sqrt(x).map_err(py_error)
}


/// Build a `(x, y, z)` coord padded to four floats like `gcode.Coord`.
fn coord3(x: f64, y: f64, z: f64) -> Coord {
    Coord(vec![x, y, z, 0.0])
}


/// Optional `getfloat` with a concrete default.
fn default_float(config: &Config, option: &str, default: f64) -> KResult<f64> {
    let opts = GetFloatOpts {
        default: Some(Some(default)),
        ..GetFloatOpts::default()
    };
    match config.getfloat(option, opts)? {
        Some(v) => Ok(v),
        None => Ok(default),
    }
}

/// `getfloat` with bounds (`above`, `maxval`), used for
/// `max_z_velocity` / `max_z_accel`.
fn bounded_float(
    config: &Config,
    option: &str,
    default: f64,
    above: f64,
    maxval: f64,
) -> KResult<f64> {
    let opts = GetFloatOpts {
        default: Some(Some(default)),
        above: Some(above),
        maxval: Some(maxval),
        ..GetFloatOpts::default()
    };
    match config.getfloat(option, opts)? {
        Some(v) => Ok(v),
        None => Ok(default),
    }
}

/// Format one of the two `%.2f` log lines of `__init__` through
/// `pycompat::text::percent_format` (CPython `%` semantics).
fn percent2(fmt: &str, v0: f64, v1: f64) -> String {
    percent_format(
        fmt,
        &[PctValue::Float(v0), PctValue::Float(v1)],
    )
    .expect("valid percent format with float args")
}

fn percent3(fmt: &str, v0: f64, v1: f64, v2: f64) -> String {
    percent_format(
        fmt,
        &[PctValue::Float(v0), PctValue::Float(v1), PctValue::Float(v2)],
    )
    .expect("valid percent format with float args")
}

/// Mutable `DeltaKinematics` state (Python instance dict subset).
struct State {
    need_home: bool,
    limit_xy2: f64,
}

#[allow(dead_code)]
pub struct DeltaKinematics {
    rails: Vec<Rail>,
    toolhead: ToolHead,
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    radius: f64,
    arm_lengths: Vec<f64>,
    arm2: Vec<f64>,
    abs_endstops: Vec<f64>,
    angles: Vec<f64>,
    towers: Vec<(f64, f64)>,
    home_position: [f64; 3],
    max_z: f64,
    min_z: f64,
    limit_z: f64,
    min_arm_length: f64,
    min_arm2: f64,
    slow_xy2: f64,
    very_slow_xy2: f64,
    max_xy2: f64,
    axes_min: Coord,
    axes_max: Coord,
    state: GCell<State>,
}

impl DeltaKinematics {
    /// `DeltaKinematics(toolhead, config)` (`delta.py:13-90`).
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<Arc<DeltaKinematics>> {
        // Setup tower rails (delta.py:15-25).
        let stepper_configs: Vec<Config> = ['a', 'b', 'c']
            .iter()
            .map(|a| config.getsection(&format!("stepper_{a}")))
            .collect();
        let factory = toolhead.stepper_factory();
        let base_opts = RailOpts {
            need_position_minmax: false,
            default_position_endstop: None,
            units_in_radians: false,
        };
        let rail_a = factory.lookup_multi_rail(&stepper_configs[0], base_opts)?;
        let a_endstop = rail_a.get_homing_info().position_endstop;
        let opts_b = RailOpts {
            need_position_minmax: false,
            default_position_endstop: Some(a_endstop),
            units_in_radians: false,
        };
        let rail_b = factory.lookup_multi_rail(&stepper_configs[1], opts_b)?;
        let rail_c = factory.lookup_multi_rail(&stepper_configs[2], opts_b)?;
        let rails: Vec<Rail> = vec![rail_a, rail_b, rail_c];

        // Setup max velocity (delta.py:27-32).
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = bounded_float(config, "max_z_velocity", max_velocity, 0.0, max_velocity)?;
        let max_z_accel = bounded_float(config, "max_z_accel", max_accel, 0.0, max_accel)?;

        // Read radius and arm lengths (delta.py:34-43).
        let radius = {
            let opts = GetFloatOpts {
                default: None,
                above: Some(0.0),
                ..GetFloatOpts::default()
            };
            match config.getfloat("delta_radius", opts)? {
                Some(v) => v,
                None => {
                    return Err(KlippyError::Config(
                        "missing required option 'delta_radius'".to_string(),
                    ))
                }
            }
        };
        let print_radius = {
            let opts = GetFloatOpts {
                default: Some(Some(radius)),
                above: Some(0.0),
                ..GetFloatOpts::default()
            };
            match config.getfloat("print_radius", opts)? {
                Some(v) => v,
                None => radius,
            }
        };
        let arm_length_a = {
            let opts = GetFloatOpts {
                default: None,
                above: Some(radius),
                ..GetFloatOpts::default()
            };
            match stepper_configs[0].getfloat("arm_length", opts)? {
                Some(v) => v,
                None => {
                    return Err(KlippyError::Config(
                        "missing required option 'arm_length'".to_string(),
                    ))
                }
            }
        };
        let mut arm_lengths: Vec<f64> = Vec::with_capacity(3);
        for sconfig in &stepper_configs {
            let opts = GetFloatOpts {
                default: Some(Some(arm_length_a)),
                above: Some(radius),
                ..GetFloatOpts::default()
            };
            let v = match sconfig.getfloat("arm_length", opts)? {
                Some(v) => v,
                None => arm_length_a,
            };
            arm_lengths.push(v);
        }
        let arm2: Vec<f64> = arm_lengths.iter().map(|a| sq(*a)).collect();
        let mut abs_endstops: Vec<f64> = Vec::with_capacity(3);
        for (rail, a2) in rails.iter().zip(arm2.iter()) {
            let ep = rail.get_homing_info().position_endstop;
            let extra = math_sqrt(a2 - sq(radius)).map_err(py_error)?;
            abs_endstops.push(ep + extra);
        }

        // Tower locations (delta.py:45-50).
        let default_angles = [210.0, 330.0, 90.0];
        let mut angles: Vec<f64> = Vec::with_capacity(3);
        for (i, sconfig) in stepper_configs.iter().enumerate() {
            let v = default_float(sconfig, "angle", default_angles[i])?;
            angles.push(v);
        }
        let towers: Vec<(f64, f64)> = angles
            .iter()
            .map(|a| (libm::cos(radians(*a)) * radius, libm::sin(radians(*a)) * radius))
            .collect();

        // setup_itersolve (delta.py:51-52).
        for ((rail, a2), tower) in rails.iter().zip(arm2.iter()).zip(towers.iter()) {
            rail.setup_itersolve(
                "delta_stepper_alloc",
                &[PyValue::Float(*a2), PyValue::Float(tower.0), PyValue::Float(tower.1)],
            );
        }
        {
            let trapq = toolhead.get_trapq();
            for rail in &rails {
                rail.set_trapq(Some(Arc::clone(&trapq)));
            }
        }
        let sphere_coords: [[f64; 3]; 3] = [
            [towers[0].0, towers[0].1, abs_endstops[0]],
            [towers[1].0, towers[1].1, abs_endstops[1]],
            [towers[2].0, towers[2].1, abs_endstops[2]],
        ];
        let home_position =
            klippy_core::mathutil::trilateration(&sphere_coords, &[arm2[0], arm2[1], arm2[2]])?;
        let max_z = rails
            .iter()
            .map(|r| r.get_homing_info().position_endstop)
            .fold(f64::INFINITY, f64::min);
        let min_z = {
            let opts = GetFloatOpts {
                default: Some(Some(0.0)),
                maxval: Some(max_z),
                ints: klippy_api::config::IntArgs::DEFAULT,
                ..GetFloatOpts::default()
            };
            config.getfloat("minimum_z_position", opts)?.unwrap_or(0.0)
        };
        let limit_z = abs_endstops
            .iter()
            .zip(arm_lengths.iter())
            .map(|(ep, arm)| ep - arm)
            .fold(f64::INFINITY, f64::min);
        let min_arm_length = arm_lengths
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let min_arm2 = sq(min_arm_length);

        log::info(&percent2(
            "Delta max build height %.2fmm (radius tapered above %.2fmm)",
            max_z,
            limit_z,
        ));

        let half_min_step_dist = {
            let mut m = f64::INFINITY;
            for rail in &rails {
                let steppers: Vec<Stepper> = rail.get_steppers();
                let d = steppers[0].get_step_dist();
                if d < m {
                    m = d;
                }
            }
            m * 0.5
        };
        let ratio_to_xy = |ratio: f64| -> KResult<f64> {
            let inside = sq(min_arm_length) / (sq(ratio) + 1.0) - sq(half_min_step_dist);
            let v = math_sqrt(inside).map_err(py_error)?;
            Ok(ratio * v + half_min_step_dist - radius)
        };
        let slow_xy2 = sq(ratio_to_xy(SLOW_RATIO)?);
        let very_slow_xy2 = sq(ratio_to_xy(2.0 * SLOW_RATIO)?);
        let max_xy2 = sq(print_radius.min(min_arm_length - radius).min(ratio_to_xy(4.0 * SLOW_RATIO)?));
        let max_xy = math_sqrt(max_xy2).map_err(py_error)?;
        log::info(&percent3(
            "Delta max build radius %.2fmm (moves slowed past %.2fmm and %.2fmm)",
            max_xy,
            math_sqrt(slow_xy2).map_err(py_error)?,
            math_sqrt(very_slow_xy2).map_err(py_error)?,
        ));

        let axes_min = coord3(-max_xy, -max_xy, min_z);
        let axes_max = coord3(max_xy, max_xy, max_z);

        let kin = DeltaKinematics {
            rails,
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            radius,
            toolhead: Arc::clone(toolhead),
            arm_lengths,
            arm2,
            abs_endstops,
            angles,
            towers,
            home_position,
            max_z,
            min_z,
            limit_z,
            min_arm_length,
            min_arm2,
            slow_xy2,
            very_slow_xy2,
            max_xy2,
            axes_min,
            axes_max,
            state: GCell::new(State {
                need_home: true,
                limit_xy2: -1.0,
            }),
        };
        let kin = Arc::new(kin);
        kin.set_position(vec![0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }

    /// `get_steppers` (`delta.py:91-92`).
    pub fn steppers(&self) -> Vec<Stepper> {
        let mut out = Vec::new();
        for rail in &self.rails {
            for s in rail.get_steppers() {
                out.push(s);
            }
        }
        out
    }

    fn actuator_to_cartesian(&self, spos: &[f64; 3]) -> KResult<[f64; 3]> {
        let sphere_coords: [[f64; 3]; 3] = [
            [self.towers[0].0, self.towers[0].1, spos[0]],
            [self.towers[1].0, self.towers[1].1, spos[1]],
            [self.towers[2].0, self.towers[2].1, spos[2]],
        ];
        klippy_core::mathutil::trilateration(&sphere_coords, &[self.arm2[0], self.arm2[1], self.arm2[2]])
    }

    /// `home_position` (delta.py:58-59).
    pub fn home_position(&self) -> [f64; 3] {
        self.home_position
    }

    /// `need_home` state (delta.py:56, 102-109).
    pub fn need_home(&self) -> bool {
        self.state.borrow().need_home
    }

    /// `limit_xy2` state (delta.py:57, 102-109, 152).
    pub fn limit_xy2(&self) -> f64 {
        self.state.borrow().limit_xy2
    }

    /// Total Z velocity limit bookkeeping kept for parity with
    /// `max_z_velocity` (unused by later delta.py code).
    pub fn max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }

    /// `axes_min` / `axes_max` (`delta.py:88-89`).
    pub fn axes_min(&self) -> &Coord {
        &self.axes_min
    }

    /// `axes_max` (`delta.py:88-89`).
    pub fn axes_max(&self) -> &Coord {
        &self.axes_max
    }

    /// `max_xy2` / `slow_xy2` / `very_slow_xy2` / `limit_z` / `min_z` /
    /// `max_z` geometry (delta.py:60-88).
    pub fn geometry(&self) -> (f64, f64, f64, f64, f64, f64) {
        (
            self.max_xy2,
            self.slow_xy2,
            self.very_slow_xy2,
            self.limit_z,
            self.min_z,
            self.max_z,
        )
    }
}

impl KinematicsApi for DeltaKinematics {
    fn class_name(&self) -> &'static str {
        "DeltaKinematics"
    }

    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        let mut spos = [0.0f64; 3];
        for (i, rail) in self.rails.iter().enumerate() {
            let name = rail.get_name(None);
            spos[i] = *stepper_positions.get(&name).ok_or_else(|| key_error(&name))?;
        }
        let pos = self.actuator_to_cartesian(&spos)?;
        Ok(vec![Some(pos[0]), Some(pos[1]), Some(pos[2])])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(index_error_list());
        }
        let coord = [newpos[0], newpos[1], newpos[2]];
        for rail in &self.rails {
            rail.set_position(coord);
        }
        let mut st = self.state.borrow_mut();
        st.limit_xy2 = -1.0;
        if homing_axes == "xyz" {
            st.need_home = false;
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let end_xy2 = sq(end_pos.0[0]) + sq(end_pos.0[1]);
        {
            let st = self.state.borrow();
            if end_xy2 <= st.limit_xy2 && axes_d.0[2] == 0.0 {
                return Ok(());
            }
            if st.need_home {
                return Err(mv.move_error("Must home first".to_string()));
            }
        }

        let end_z = end_pos.0[2];
        let mut limit_xy2 = self.max_xy2;
        if end_z > self.limit_z {
            let above_z_limit = end_z - self.limit_z;
            let allowed_radius = self.radius
                - math_sqrt(self.min_arm2 - sq(self.min_arm_length - above_z_limit))
                    .map_err(py_error)?;
            limit_xy2 = limit_xy2.min(sq(allowed_radius));
        }
        if end_xy2 > limit_xy2 || end_z > self.max_z || end_z < self.min_z {
            let at_home_xy = end_pos.0[0] == self.home_position[0] && end_pos.0[1] == self.home_position[1];
            if !at_home_xy || end_z < self.min_z || end_z > self.home_position[2] {
                return Err(mv.move_error("Move out of range".to_string()));
            }
            limit_xy2 = -1.0;
        }
        if axes_d.0[2] != 0.0 {
            let z_ratio = mv.move_d() / axes_d.0[2].abs();
            mv.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)?;
            limit_xy2 = -1.0;
        }
        let start_pos = mv.start_pos();
        let extreme_xy2 = end_xy2.max(sq(start_pos.0[0]) + sq(start_pos.0[1]));
        if extreme_xy2 > self.slow_xy2 {
            let mut r = 0.5;
            if extreme_xy2 > self.very_slow_xy2 {
                r = 0.25;
            }
            mv.limit_speed(self.max_velocity * r, self.max_accel * r)?;
            limit_xy2 = -1.0;
        }
        let slow = self.slow_xy2;
        self.state.borrow_mut().limit_xy2 = limit_xy2.min(slow);
        Ok(())
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        homing_state.set_axes(vec![0, 1, 2]);
        let force_z = -1.5
            * math_sqrt(self.arm2.iter().cloned().fold(0.0f64, f64::max) - self.max_xy2)
                .map_err(py_error)?;
        let forcepos: Vec<Option<f64>> = vec![
            Some(self.home_position[0]),
            Some(self.home_position[1]),
            Some(force_z),
        ];
        let movepos: Vec<Option<f64>> = vec![
            Some(self.home_position[0]),
            Some(self.home_position[1]),
            Some(self.home_position[2]),
        ];
        homing_state.home_rails(self.rails.clone(), forcepos, movepos)
    }


    fn get_status(&self, _eventtime: f64) -> KResult<PyValue> {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        let homed = if self.state.borrow().need_home {
            PyValue::Str(String::new())
        } else {
            PyValue::Str("xyz".to_string())
        };
        d.insert(PyValue::Str("homed_axes".to_string()), homed);
        let axmin = vec![
            PyValue::Float(self.axes_min.0[0]),
            PyValue::Float(self.axes_min.0[1]),
            PyValue::Float(self.axes_min.0[2]),
            PyValue::Int(pycompat::PyInt::from_i64(0)),
        ];
        let axmax = vec![
            PyValue::Float(self.axes_max.0[0]),
            PyValue::Float(self.axes_max.0[1]),
            PyValue::Float(self.axes_max.0[2]),
            PyValue::Int(pycompat::PyInt::from_i64(0)),
        ];
        d.insert(
            PyValue::Str("axis_minimum".to_string()),
            PyValue::Tuple(axmin),
        );
        d.insert(
            PyValue::Str("axis_maximum".to_string()),
            PyValue::Tuple(axmax),
        );
        d.insert(
            PyValue::Str("cone_start_z".to_string()),
            PyValue::Float(self.limit_z),
        );
        Ok(PyValue::Dict(d))
    }

    fn clear_homing_state(&self, clear_axes: &str) -> KResult<()> {
        // Clearing homing state for each axis individually is not implemented
        // (delta.py:106-109): any non-empty clear_axes resets.
        if !clear_axes.is_empty() {
            let mut st = self.state.borrow_mut();
            st.limit_xy2 = -1.0;
            st.need_home = true;
        }
        Ok(())
    }

    fn get_calibration(&self) -> KResult<klippy_api::toolhead::DeltaCalibration> {
        let endstops: Vec<f64> = self
            .rails
            .iter()
            .map(|r| r.get_homing_info().position_endstop)
            .collect();
        let stepdists: Vec<f64> = self
            .rails
            .iter()
            .map(|r| r.get_steppers()[0].get_step_dist())
            .collect();
        let cal = DeltaCalibration::new(
            self.radius,
            self.angles.clone(),
            self.arm_lengths.clone(),
            endstops,
            stepdists,
        );
        Ok(Arc::new(cal))
    }
}

/// `DeltaCalibration` (`delta.py:169-238`).
#[derive(Clone, Debug)]
pub struct DeltaCalibration {
    radius: f64,
    angles: Vec<f64>,
    arms: Vec<f64>,
    endstops: Vec<f64>,
    stepdists: Vec<f64>,
    towers: Vec<(f64, f64)>,
    abs_endstops: Vec<f64>,
}

impl DeltaCalibration {
    /// `DeltaCalibration(radius, angles, arms, endstops, stepdists)`.
    pub fn new(
        radius: f64,
        angles: Vec<f64>,
        arms: Vec<f64>,
        endstops: Vec<f64>,
        stepdists: Vec<f64>,
    ) -> DeltaCalibration {
        let towers: Vec<(f64, f64)> = angles
            .iter()
            .map(|a| (libm::cos(radians(*a)) * radius, libm::sin(radians(*a)) * radius))
            .collect();
        let radius2 = sq(radius);
        let mut abs_endstops: Vec<f64> = Vec::with_capacity(3);
        for (e, a) in endstops.iter().zip(arms.iter()) {
            let extra = libm::sqrt(sq(*a) - radius2);
            abs_endstops.push(e + extra);
        }
        DeltaCalibration {
            radius,
            angles,
            arms,
            endstops,
            stepdists,
            towers,
            abs_endstops,
        }
    }

    /// `radius` (delta.py:171).
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// `angles` (delta.py:172).
    pub fn angles(&self) -> &[f64] {
        &self.angles
    }

    /// `arms` (delta.py:173).
    pub fn arms(&self) -> &[f64] {
        &self.arms
    }

    /// `endstops` (delta.py:174).
    pub fn endstops(&self) -> &[f64] {
        &self.endstops
    }

    /// `stepdists` (delta.py:175).
    pub fn stepdists(&self) -> &[f64] {
        &self.stepdists
    }

    /// `towers` (delta.py:178-179).
    pub fn towers(&self) -> &[(f64, f64)] {
        &self.towers
    }

    /// `abs_endstops` (delta.py:182-183).
    pub fn abs_endstops(&self) -> &[f64] {
        &self.abs_endstops
    }

    fn coordinate_descent_params_inner(
        &self,
        is_extended: bool,
    ) -> (Vec<String>, Vec<(String, f64)>) {
        let mut adj_params: Vec<String> = vec![
            "radius".to_string(),
            "angle_a".to_string(),
            "angle_b".to_string(),
            "endstop_a".to_string(),
            "endstop_b".to_string(),
            "endstop_c".to_string(),
        ];
        if is_extended {
            adj_params.push("arm_a".to_string());
            adj_params.push("arm_b".to_string());
            adj_params.push("arm_c".to_string());
        }
        let mut params: Vec<(String, f64)> = vec![("radius".to_string(), self.radius)];
        for (i, axis) in ['a', 'b', 'c'].iter().enumerate() {
            params.push((format!("angle_{axis}"), self.angles[i]));
            params.push((format!("arm_{axis}"), self.arms[i]));
            params.push((format!("endstop_{axis}"), self.endstops[i]));
            params.push((format!("stepdist_{axis}"), self.stepdists[i]));
        }
        (adj_params, params)
    }

    /// Create a new calibration object from coordinate-descent params
    /// (`new_calibration`, delta.py:197-204).
    pub fn new_calibration(&self, params: &PyValue) -> KResult<DeltaCalibration> {
        let get = |k: &str| -> KResult<f64> {
            match params {
                PyValue::Dict(d) => d
                    .get(&PyValue::Str(k.to_string()))
                    .and_then(|v| match v {
                        PyValue::Float(f) => Some(*f),
                        _ => None,
                    })
                    .ok_or_else(|| key_error(k)),
                _ => Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "params must be a dict".to_string(),
                }),
            }
        };
        let radius = get("radius")?;
        let mut angles = Vec::with_capacity(3);
        let mut arms = Vec::with_capacity(3);
        let mut endstops = Vec::with_capacity(3);
        let mut stepdists = Vec::with_capacity(3);
        for axis in ['a', 'b', 'c'] {
            angles.push(get(&format!("angle_{axis}"))?);
            arms.push(get(&format!("arm_{axis}"))?);
            endstops.push(get(&format!("endstop_{axis}"))?);
            stepdists.push(get(&format!("stepdist_{axis}"))?);
        }
        Ok(DeltaCalibration::new(
            radius, angles, arms, endstops, stepdists,
        ))
    }


    /// `get_position_from_stable` (`delta.py:205-211`).
    pub fn get_position_from_stable(&self, stable_position: &[f64]) -> KResult<Vec<f64>> {
        if stable_position.len() < 3 {
            return Err(index_error_list());
        }
        let sphere_coords: [[f64; 3]; 3] = [
            [
                self.towers[0].0,
                self.towers[0].1,
                self.abs_endstops[0] - stable_position[0] * self.stepdists[0],
            ],
            [
                self.towers[1].0,
                self.towers[1].1,
                self.abs_endstops[1] - stable_position[1] * self.stepdists[1],
            ],
            [
                self.towers[2].0,
                self.towers[2].1,
                self.abs_endstops[2] - stable_position[2] * self.stepdists[2],
            ],
        ];
        let pos = klippy_core::mathutil::trilateration(
            &sphere_coords,
            &[sq(self.arms[0]), sq(self.arms[1]), sq(self.arms[2])],
        )?;
        Ok(vec![pos[0], pos[1], pos[2]])
    }

    /// `calc_stable_position` (`delta.py:212-219`).
    pub fn calc_stable_position(&self, coord: &[f64]) -> KResult<Vec<f64>> {
        if coord.len() < 3 {
            return Err(index_error_list());
        }
        let mut steppos: Vec<f64> = Vec::with_capacity(3);
        for (t, a) in self.towers.iter().zip(self.arms.iter()) {
            let dx = t.0 - coord[0];
            let dy = t.1 - coord[1];
            let v = py_sqrt(sq(*a) - sq(dx) - sq(dy))? + coord[2];
            steppos.push(v);
        }
        let mut out: Vec<f64> = Vec::with_capacity(3);
        for ((sd, ep), sp) in self
            .stepdists
            .iter()
            .zip(self.abs_endstops.iter())
            .zip(steppos.iter())
        {
            out.push((ep - sp) / sd);
        }
        Ok(out)
    }

    /// `save_state` (`delta.py:220-238`).
    ///
    /// `configfile` and `gcode` are the `PrinterConfig` /
    /// `GCode` objects (`configfile.set`, `printer.lookup_object("gcode")`
    /// `respond_info`); the frozen `DeltaCalibrationApi` trait does not
    /// expose this method, so it lives here as an inherent method.
    pub fn save_state(&self, configfile: &PrinterConfig) -> KResult<()> {
        configfile.set("printer", "delta_radius", &fmt6(self.radius))?;
        for (i, axis) in ['a', 'b', 'c'].iter().enumerate() {
            configfile.set(
                &format!("stepper_{axis}"),
                "angle",
                &fmt6(self.angles[i]),
            )?;
            configfile.set(
                &format!("stepper_{axis}"),
                "arm_length",
                &fmt6(self.arms[i]),
            )?;
            configfile.set(
                &format!("stepper_{axis}"),
                "position_endstop",
                &fmt6(self.endstops[i]),
            )?;
        }
        let msg = percent_format(
            "stepper_a: position_endstop: %.6f angle: %.6f arm_length: %.6f\nstepper_b: position_endstop: %.6f angle: %.6f arm_length: %.6f\nstepper_c: position_endstop: %.6f angle: %.6f arm_length: %.6f\ndelta_radius: %.6f",
            &[
                PctValue::Float(self.endstops[0]),
                PctValue::Float(self.angles[0]),
                PctValue::Float(self.arms[0]),
                PctValue::Float(self.endstops[1]),
                PctValue::Float(self.angles[1]),
                PctValue::Float(self.arms[1]),
                PctValue::Float(self.endstops[2]),
                PctValue::Float(self.angles[2]),
                PctValue::Float(self.arms[2]),
                PctValue::Float(self.radius),
            ],
        )
        .expect("valid percent format with float args");
        let printer = configfile.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        gcode.respond_info(&msg, true);
        Ok(())
    }
}

fn fmt6(v: f64) -> String {
    percent_format("%.6f", &[PctValue::Float(v)])
        .expect("valid percent format with float args")
}

impl DeltaCalibrationApi for DeltaCalibration {
    fn coordinate_descent_params(&self, is_extended: bool) -> PyValue {
        let (adj_params, params) = self.coordinate_descent_params_inner(is_extended);
        let adj: Vec<PyValue> = adj_params.into_iter().map(PyValue::Str).collect();
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in params {
            d.insert(PyValue::Str(k), PyValue::Float(v));
        }
        PyValue::Tuple(vec![PyValue::Tuple(adj), PyValue::Dict(d)])
    }

    fn new_calibration(&self, params: &PyValue) -> KResult<klippy_api::toolhead::DeltaCalibration> {
        Ok(Arc::new(DeltaCalibration::new_calibration(self, params)?))
    }

    fn get_position_from_stable(&self, stable_position: &[f64]) -> KResult<Vec<f64>> {
        DeltaCalibration::get_position_from_stable(self, stable_position)
    }

    fn calc_stable_position(&self, coord: &[f64]) -> KResult<Vec<f64>> {
        DeltaCalibration::calc_stable_position(self, coord)
    }

    fn save_state(&self, configfile: &PrinterConfig) -> KResult<()> {
        DeltaCalibration::save_state(self, configfile)
    }
}



/// `load_kinematics(toolhead, config)` (`delta.py:240-241`).
///
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    Ok(DeltaKinematics::new(toolhead, config)?)
}

/// The frozen `delta.py` has no `load_config` / `load_config_prefix` /
/// `add_printer_objects`: `toolhead.py` builds the kinematics directly
/// via `load_kinematics` (see `clocksync` for the same shape).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
