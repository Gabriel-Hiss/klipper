//! Port of `klippy/kinematics/polar.py`.

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::{libm, PyInt, PyValue};

use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{
    Coord, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_api::{GCell, KResult, KlippyError};

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

fn index_error_tuple() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "tuple index out of range".to_string(),
    }
}

fn zero_div_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "ZeroDivisionError".to_string(),
        message: "float division by zero".to_string(),
    }
}

fn math_domain_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "math domain error".to_string(),
    }
}

fn py_div(a: f64, b: f64) -> KResult<f64> {
    if b == 0.0 {
        return Err(zero_div_error());
    }
    Ok(a / b)
}

fn py_sqrt(x: f64) -> KResult<f64> {
    if x < 0.0 {
        return Err(math_domain_error());
    }
    Ok(libm::sqrt(x))
}

/// `distance_to_center(p1, p2)` (`polar.py:10`).
pub fn distance_to_center(p1: &[f64], p2: &[f64]) -> KResult<f64> {
    if p1.len() < 2 || p2.len() < 2 {
        return Err(index_error_tuple());
    }
    let ab_x = p2[0] - p1[0];
    let ab_y = p2[1] - p1[1];
    let ap_x = -p1[0];
    let ap_y = -p1[1];
    let dot = ab_x * ap_x + ab_y * ap_y;
    let ab_length = py_sqrt(libm::pow(ab_x, 2.0) + libm::pow(ab_y, 2.0))?;
    if dot <= 0.0 {
        return Ok(py_sqrt(libm::pow(ap_x, 2.0) + libm::pow(ap_y, 2.0))?);
    } else if dot >= libm::pow(ab_length, 2.0) {
        return Ok(py_sqrt(libm::pow(p2[0], 2.0) + libm::pow(p2[1], 2.0))?);
    }
    let num = libm::fabs(ab_x * ap_y - ab_y * ap_x);
    py_div(num, ab_length)
}

/// `PolarKinematics` (`polar.py:29`).
pub struct PolarKinematics {
    rails: [Rail; 2],
    steppers: Vec<Stepper>,
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    v_rad_max: f64,
    limit_z: GCell<(f64, f64)>,
    limit_xy2: GCell<f64>,
    axes_min: Coord,
    axes_max: Coord,
}

impl PolarKinematics {
    /// `PolarKinematics.__init__(toolhead, config)` (`polar.py:30`).
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<Arc<PolarKinematics>> {
        let factory = toolhead.stepper_factory();
        let stepper_bed = factory.printer_stepper(&config.getsection("stepper_bed"), true)?;
        let rail_arm = factory.lookup_rail(&config.getsection("stepper_arm"), RailOpts::default())?;
        let rail_z = factory.lookup_multi_rail(
            &config.getsection("stepper_z"),
            RailOpts::default(),
        )?;
        stepper_bed.setup_itersolve(
            "polar_stepper_alloc",
            &[PyValue::Bytes(b"a".to_vec())],
        );
        rail_arm.setup_itersolve(
            "polar_stepper_alloc",
            &[PyValue::Bytes(b"r".to_vec())],
        );
        rail_z.setup_itersolve(
            "cartesian_stepper_alloc",
            &[PyValue::Bytes(b"z".to_vec())],
        );
        let rails: [Rail; 2] = [rail_arm, rail_z];
        let mut steppers: Vec<Stepper> = Vec::new();
        steppers.push(Arc::clone(&stepper_bed));
        for rail in rails.iter() {
            for s in rail.get_steppers() {
                steppers.push(s);
            }
        }
        let trapq = toolhead.get_trapq();
        for s in &steppers {
            s.set_trapq(Some(Arc::clone(&trapq)));
        }
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = config
            .getfloat(
                "max_z_velocity",
                GetFloatOpts {
                    default: Some(Some(max_velocity)),
                    maxval: Some(max_velocity),
                    above: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(max_velocity);
        let max_z_accel = config
            .getfloat(
                "max_z_accel",
                GetFloatOpts {
                    default: Some(Some(max_accel)),
                    maxval: Some(max_accel),
                    above: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(max_accel);
        let v_rad_max = config
            .getfloat(
                "max_angular_velocity",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    above: Some(0.0),
                    ints: klippy_api::config::IntArgs::DEFAULT,
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.0);
        let max_xy = rails[0].get_range().1;
        let (min_z, max_z) = rails[1].get_range();
        Ok(Arc::new(PolarKinematics {
            rails,
            steppers,
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            v_rad_max,
            limit_z: GCell::new((1.0, -1.0)),
            limit_xy2: GCell::new(-1.0),
            axes_min: Coord(vec![-max_xy, -max_xy, min_z, 0.0]),
            axes_max: Coord(vec![max_xy, max_xy, max_z, 0.0]),
        }))
    }

    fn home_axis_inner(
        &self,
        homing_state: &HomingStateHandle,
        axis: i64,
        rail: &Rail,
    ) -> KResult<()> {
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos: Vec<Option<f64>> = vec![None, None, None, None];
        homepos[axis as usize] = Some(hi.position_endstop);
        if axis == 0 {
            homepos[1] = Some(0.0);
        }
        let mut forcepos = homepos.clone();
        if hi.positive_dir {
            let delta = hi.position_endstop - position_min;
            forcepos[axis as usize] = Some(hi.position_endstop - delta);
        } else {
            let delta = position_max - hi.position_endstop;
            forcepos[axis as usize] = Some(hi.position_endstop + delta);
        }
        homing_state.home_rails(vec![Arc::clone(rail)], forcepos, homepos)
    }

    /// Current `limit_xy2`.
    pub fn limit_xy2(&self) -> f64 {
        self.limit_xy2.get()
    }

    /// Current `limit_z`.
    pub fn limit_z(&self) -> (f64, f64) {
        self.limit_z.get()
    }

    /// `axes_min`.
    pub fn axes_min(&self) -> Coord {
        self.axes_min.clone()
    }

    /// `axes_max`.
    pub fn axes_max(&self) -> Coord {
        self.axes_max.clone()
    }

    /// Stepper list copy.
    pub fn steppers_snapshot(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }

    /// Rail range snapshot.
    pub fn rail_ranges(&self) -> Vec<(f64, f64)> {
        self.rails.iter().map(|r| r.get_range()).collect()
    }

    /// `max_velocity`.
    pub fn max_velocity(&self) -> f64 {
        self.max_velocity
    }

    /// `max_accel`.
    pub fn max_accel(&self) -> f64 {
        self.max_accel
    }

    /// `max_z_velocity`.
    pub fn max_z_velocity(&self) -> f64 {
        self.max_z_velocity
    }

    /// `max_z_accel`.
    pub fn max_z_accel(&self) -> f64 {
        self.max_z_accel
    }

    /// `v_rad_max`.
    pub fn v_rad_max(&self) -> f64 {
        self.v_rad_max
    }
}

impl KinematicsApi for PolarKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let bed_name = self.steppers[0].get_name(None);
        let bed_angle = *stepper_positions
            .get(&bed_name)
            .ok_or_else(|| key_error(&bed_name))?;
        let arm_name = self.rails[0].get_name(None);
        let arm_pos = *stepper_positions
            .get(&arm_name)
            .ok_or_else(|| key_error(&arm_name))?;
        let z_name = self.rails[1].get_name(None);
        let z_pos = *stepper_positions
            .get(&z_name)
            .ok_or_else(|| key_error(&z_name))?;
        Ok(vec![
            Some(libm::cos(bed_angle) * arm_pos),
            Some(libm::sin(bed_angle) * arm_pos),
            Some(z_pos),
        ])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(index_error_list());
        }
        let coord = [newpos[0], newpos[1], newpos[2]];
        for s in &self.steppers {
            s.set_position(coord);
        }
        if homing_axes.contains('z') {
            self.limit_z.set(self.rails[1].get_range());
        }
        if homing_axes.contains('x') && homing_axes.contains('y') {
            let m = self.rails[0].get_range().1;
            self.limit_xy2.set(libm::pow(m, 2.0));
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        if clear_axes.contains('x') || clear_axes.contains('y') {
            self.limit_xy2.set(-1.0);
        }
        if clear_axes.contains('z') {
            self.limit_z.set((1.0, -1.0));
        }
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        let homing_axes = homing_state.get_axes();
        let home_xy = homing_axes.contains(&0) || homing_axes.contains(&1);
        let home_z = homing_axes.contains(&2);
        let mut updated_axes: Vec<i64> = Vec::new();
        if home_xy {
            updated_axes = vec![0, 1];
        }
        if home_z {
            updated_axes.push(2);
        }
        homing_state.set_axes(updated_axes);
        if home_xy {
            self.home_axis_inner(&homing_state, 0, &self.rails[0])?;
        }
        if home_z {
            self.home_axis_inner(&homing_state, 2, &self.rails[1])?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        if end_pos.0.len() < 2 {
            return Err(index_error_tuple());
        }
        let xy2 = libm::pow(end_pos.0[0], 2.0) + libm::pow(end_pos.0[1], 2.0);
        let limit_xy2 = self.limit_xy2.get();
        if xy2 > limit_xy2 {
            if limit_xy2 < 0.0 {
                return Err(mv.move_error("Must home axis first".to_string()));
            }
            return Err(mv.move_error("Move out of range".to_string()));
        }
        let axes_d = mv.axes_d();
        if axes_d.0.len() < 3 {
            return Err(index_error_tuple());
        }
        if axes_d.0[2] != 0.0 {
            if end_pos.0.len() < 3 {
                return Err(index_error_tuple());
            }
            let limit_z = self.limit_z.get();
            if end_pos.0[2] < limit_z.0 || end_pos.0[2] > limit_z.1 {
                if limit_z.0 > limit_z.1 {
                    return Err(mv.move_error("Must home axis first".to_string()));
                }
                return Err(mv.move_error("Move out of range".to_string()));
            }
            let move_d = mv.move_d();
            let denom = libm::fabs(axes_d.0[2]);
            let z_ratio = py_div(move_d, denom)?;
            mv.limit_speed(
                self.max_z_velocity * z_ratio,
                self.max_z_accel * z_ratio,
            )?;
        }
        if axes_d.0[0] != 0.0 || axes_d.0[1] != 0.0 {
            if self.v_rad_max == 0.0 {
                return Ok(());
            }
            let start_pos = mv.start_pos();
            let min_dist = distance_to_center(&start_pos.0, &end_pos.0)?;
            if min_dist == 0.0 {
                return Ok(());
            }
            let max_cruise_v2 = mv.max_cruise_v2();
            let v_angular = py_div(py_sqrt(max_cruise_v2)?, min_dist)?;
            if self.v_rad_max < v_angular {
                let scale_radius = py_div(self.v_rad_max, v_angular)?;
                mv.limit_speed(
                    self.max_velocity * scale_radius,
                    self.max_accel * scale_radius,
                )?;
            }
        }
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let limit_xy2 = self.limit_xy2.get();
        let limit_z = self.limit_z.get();
        let xy_home = if limit_xy2 >= 0.0 { "xy" } else { "" };
        let z_home = if limit_z.0 <= limit_z.1 { "z" } else { "" };
        let mut dict = IndexMap::new();
        dict.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(format!("{xy_home}{z_home}")),
        );
        dict.insert(
            PyValue::Str("axis_minimum".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(self.axes_min.0[0]),
                PyValue::Float(self.axes_min.0[1]),
                PyValue::Float(self.axes_min.0[2]),
                PyValue::Int(PyInt::from_i64(0)),
            ]),
        );
        dict.insert(
            PyValue::Str("axis_maximum".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(self.axes_max.0[0]),
                PyValue::Float(self.axes_max.0[1]),
                PyValue::Float(self.axes_max.0[2]),
                PyValue::Int(PyInt::from_i64(0)),
            ]),
        );
        PyValue::Dict(dict)
    }
}

/// `load_kinematics(toolhead, config)` (`polar.py:150`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    Ok(PolarKinematics::new(toolhead, config)?)
}

/// Module factory. `polar` is loaded through `load_kinematics`, not a printer section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
