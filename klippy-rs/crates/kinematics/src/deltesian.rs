//! Port of `klippy/kinematics/deltesian.py`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead};
use klippy_api::GCell;
use pycompat::text::PyError;
use pycompat::{PctValue, PyInt, PyValue};
use std::sync::Arc;

/// `SLOW_RATIO` (`klippy/kinematics/deltesian.py:10`).
pub const SLOW_RATIO: f64 = 3.0;
/// `MIN_ANGLE` (`klippy/kinematics/deltesian.py:13`).
pub const MIN_ANGLE: f64 = 5.0;

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.to_string(),
    }
}

fn from_py(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

fn index_error_list() -> KlippyError {
    internal("IndexError", "list index out of range")
}

fn index_error_tuple() -> KlippyError {
    internal("IndexError", "tuple index out of range")
}

/// `math.sqrt`: `ValueError` for negative input, NaN passes through.
fn py_sqrt(x: f64) -> KResult<f64> {
    if x < 0.0 {
        return Err(internal("ValueError", "math domain error"));
    }
    Ok(kos::libm::sqrt(x))
}

/// `a / b` on floats: `ZeroDivisionError` for `b == 0.0`.
fn py_div(a: f64, b: f64) -> KResult<f64> {
    pycompat::text::float_true_div(a, b).map_err(from_py)
}

/// `a ** 2` on a float: libm `pow`, `OverflowError` with the errno text on overflow.
fn py_pow2(a: f64) -> KResult<f64> {
    let r = kos::libm::pow(a, 2.0);
    if r.is_infinite() && a.is_finite() {
        return Err(internal("OverflowError", "(34, 'Numerical result out of range')"));
    }
    Ok(r)
}

/// `math.ceil` / `math.floor` result as an integral float (the Python result is an int).
fn float_to_integral(r: f64) -> KResult<f64> {
    if r.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer"));
    }
    if r.is_infinite() {
        return Err(internal("OverflowError", "cannot convert float infinity to integer"));
    }
    // `-0.0 + 0.0` is `0.0`: Python's int has no negative zero.
    Ok(r + 0.0)
}

fn py_ceil(x: f64) -> KResult<f64> {
    float_to_integral(kos::libm::ceil(x))
}

fn py_floor(x: f64) -> KResult<f64> {
    float_to_integral(kos::libm::floor(x))
}

/// Builtin `min(a, b)`: the first argument unless the second compares smaller.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

fn int_value(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

/// `klippy/kinematics/deltesian.py:15` `DeltesianKinematics`.
pub struct DeltesianKinematics {
    rails: [Rail; 3],
    arm_x: [f64; 2],
    arm2: [f64; 2],
    limits: [(f64, f64); 3],
    abs_endstop: [f64; 2],
    home_z: f64,
    slow_x2: Option<f64>,
    very_slow_x2: Option<f64>,
    max_z_velocity: f64,
    max_z_accel: f64,
    axes_min: Vec<PyValue>,
    axes_max: Vec<PyValue>,
    homed_axis: GCell<[bool; 3]>,
}

impl DeltesianKinematics {
    /// `DeltesianKinematics.__init__(toolhead, config)`
    /// (`klippy/kinematics/deltesian.py:16`).
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<Arc<DeltesianKinematics>> {
        let factory = toolhead.stepper_factory();
        let stepper_configs: Vec<Config> = ["left", "right", "y"]
            .iter()
            .map(|s| config.getsection(&format!("stepper_{s}")))
            .collect();
        let rail0 = factory.lookup_rail(
            &stepper_configs[0],
            RailOpts {
                need_position_minmax: false,
                ..RailOpts::default()
            },
        )?;
        let def_pos_es = rail0.get_homing_info().position_endstop;
        let rail1 = factory.lookup_rail(
            &stepper_configs[1],
            RailOpts {
                need_position_minmax: false,
                default_position_endstop: Some(def_pos_es),
                ..RailOpts::default()
            },
        )?;
        let rail2 = factory.lookup_multi_rail(&stepper_configs[2], RailOpts::default())?;
        let rails: [Rail; 3] = [rail0, rail1, rail2];

        let arm_x0 = required_float(
            &stepper_configs[0],
            "arm_x_length",
            GetFloatOpts {
                above: Some(0.0),
                ..Default::default()
            },
        )?;
        let arm_x1 = required_float(
            &stepper_configs[1],
            "arm_x_length",
            GetFloatOpts {
                default: Some(Some(arm_x0)),
                above: Some(0.0),
                ..Default::default()
            },
        )?;
        let arm_x = [arm_x0, arm_x1];
        let arm0 = required_float(
            &stepper_configs[0],
            "arm_length",
            GetFloatOpts {
                above: Some(arm_x[0]),
                ..Default::default()
            },
        )?;
        let arm1 = required_float(
            &stepper_configs[1],
            "arm_length",
            GetFloatOpts {
                default: Some(Some(arm0)),
                above: Some(arm_x[1]),
                ..Default::default()
            },
        )?;
        let arm = [arm0, arm1];
        let arm2 = [py_pow2(arm[0])?, py_pow2(arm[1])?];
        rails[0].setup_itersolve(
            "deltesian_stepper_alloc",
            &[PyValue::Float(arm2[0]), PyValue::Float(-arm_x[0])],
        );
        rails[1].setup_itersolve(
            "deltesian_stepper_alloc",
            &[PyValue::Float(arm2[1]), PyValue::Float(arm_x[1])],
        );
        rails[2].setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(b"y".to_vec())]);
        for rail in rails.iter() {
            for s in rail.get_steppers() {
                s.set_trapq(Some(toolhead.get_trapq()));
            }
        }

        let mut limits: [(f64, f64); 3] = [(1.0, -1.0); 3];
        // X axis limits
        let min_angle = required_float(
            config,
            "min_angle",
            GetFloatOpts {
                default: Some(Some(MIN_ANGLE)),
                minval: Some(0.0),
                maxval: Some(90.0),
                ..Default::default()
            },
        )?;
        // math.radians(x) is x * (pi / 180)
        let cos_angle = kos::libm::cos(min_angle * (std::f64::consts::PI / 180.0));
        let x_kin_min = py_ceil(-py_min(arm_x[0], cos_angle * arm[1] - arm_x[1]))?;
        let x_kin_max = py_floor(py_min(arm_x[1], cos_angle * arm[0] - arm_x[0]))?;
        let x_kin_range = py_min(
            py_min(x_kin_max - x_kin_min, x_kin_max * 2.0),
            -x_kin_min * 2.0,
        );
        // `x_kin_range` is an int in Python (`ceil`/`floor` results).
        let print_width = config.getfloat(
            "print_width",
            GetFloatOpts {
                default: Some(None),
                minval: Some(0.0),
                maxval: Some(x_kin_range),
                ints: klippy_api::config::IntArgs::MAXVAL,
                ..Default::default()
            },
        )?;
        // `ceil`/`floor` produce ints, so the default X limits are ints in `axis_minimum`.
        let mut x_limits_are_ints = true;
        limits[0] = match print_width {
            Some(w) if w != 0.0 => {
                x_limits_are_ints = false;
                (-w * 0.5, w * 0.5)
            }
            _ => (x_kin_min, x_kin_max),
        };
        // Y axis limits
        limits[1] = rails[2].get_range();
        // Z axis limits
        let pmax = [
            rails[0].get_homing_info().position_endstop,
            rails[1].get_homing_info().position_endstop,
        ];
        let mut abs_endstop = [0.0f64; 2];
        for i in 0..2 {
            abs_endstop[i] = pmax[i] + py_sqrt(arm2[i] - py_pow2(arm_x[i])?)?;
        }
        let home_z = actuator_to_cartesian(&arm_x, &arm2, &abs_endstop)?.1;
        let z_max_lo = pillars_z_max(&arm_x, &arm2, &abs_endstop, limits[0].0)?;
        let z_max_hi = pillars_z_max(&arm_x, &arm2, &abs_endstop, limits[0].1)?;
        let z_max = py_min(z_max_lo, z_max_hi);
        // The Python default is the int 0: tracked and returned as an `int`,
        // so `axis_minimum` shows `0`. The getter returns `f64`; an untracked
        // presence check tells the default apart.
        let z_min_present = config
            .get("minimum_z_position", klippy_api::config::GetStrOpts { default: Some(None), note_valid: false })?
            .is_some();
        let z_min = config.getfloat(
            "minimum_z_position",
            GetFloatOpts {
                default: Some(Some(0.0)),
                maxval: Some(z_max),
                ints: klippy_api::config::IntArgs::DEFAULT,
                ..Default::default()
            },
        )?;
        let z_min_cfg = if z_min_present { z_min } else { None };
        limits[2] = (z_min_cfg.unwrap_or(0.0), z_max);

        // Limit the speed/accel if is is at the extreme values of the x axis
        let slow_ratio = required_float(
            config,
            "slow_ratio",
            GetFloatOpts {
                default: Some(Some(SLOW_RATIO)),
                minval: Some(0.0),
                ..Default::default()
            },
        )?;
        let mut slow_x2: Option<f64> = None;
        let mut very_slow_x2: Option<f64> = None;
        if slow_ratio > 0.0 {
            let sr2 = py_pow2(slow_ratio)?;
            let mut slow = [0.0f64; 2];
            for i in 0..2 {
                slow[i] = py_sqrt(py_div(sr2 * arm2[i], sr2 + 1.0)?)? - arm_x[i];
            }
            let s2 = py_pow2(py_min(slow[0], slow[1]))?;
            let mut very_slow = [0.0f64; 2];
            for i in 0..2 {
                very_slow[i] = py_sqrt(py_div(2.0 * sr2 * arm2[i], 2.0 * sr2 + 1.0)?)? - arm_x[i];
            }
            let vs2 = py_pow2(py_min(very_slow[0], very_slow[1]))?;
            slow_x2 = Some(s2);
            very_slow_x2 = Some(vs2);
            let msg = pycompat::text::percent_format(
                "Deltesian kinematics: moves slowed past %.2fmm and %.2fmm",
                &[
                    PctValue::Float(py_sqrt(s2)?),
                    PctValue::Float(py_sqrt(vs2)?),
                ],
            )
            .map_err(from_py)?;
            klippy_api::log::info(&msg);
        }

        // Setup boundary checks
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_z_velocity = required_float(
            config,
            "max_z_velocity",
            GetFloatOpts {
                default: Some(Some(max_velocity)),
                above: Some(0.0),
                maxval: Some(max_velocity),
                ..Default::default()
            },
        )?;
        let max_z_accel = required_float(
            config,
            "max_z_accel",
            GetFloatOpts {
                default: Some(Some(max_accel)),
                above: Some(0.0),
                maxval: Some(max_accel),
                ..Default::default()
            },
        )?;
        let x_value = |v: f64| {
            if x_limits_are_ints {
                PyValue::Int(pycompat::text::trunc_float_to_bigint(v))
            } else {
                PyValue::Float(v)
            }
        };
        let z_min_value = match z_min_cfg {
            Some(v) => PyValue::Float(v),
            None => int_value(0),
        };
        // `toolhead.Coord` pads with the int 0 up to four entries.
        let axes_min = vec![
            x_value(limits[0].0),
            PyValue::Float(limits[1].0),
            z_min_value,
            int_value(0),
        ];
        let axes_max = vec![
            x_value(limits[0].1),
            PyValue::Float(limits[1].1),
            PyValue::Float(limits[2].1),
            int_value(0),
        ];

        let kin = Arc::new(DeltesianKinematics {
            rails,
            arm_x,
            arm2,
            limits,
            abs_endstop,
            home_z,
            slow_x2,
            very_slow_x2,
            max_z_velocity,
            max_z_accel,
            axes_min,
            axes_max,
            homed_axis: GCell::new([false; 3]),
        });
        kin.set_position(vec![0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }

    fn pillars_z(&self, x: f64) -> KResult<f64> {
        pillars_z_max(&self.arm_x, &self.arm2, &self.abs_endstop, x)
    }
}

/// `config.getfloat(option, default, ...)` where the default is a float: the value is always present.
fn required_float(config: &Config, option: &str, opts: GetFloatOpts) -> KResult<f64> {
    match config.getfloat(option, opts)? {
        Some(v) => Ok(v),
        None => Err(KlippyError::Config(format!(
            "Missing option {option} in section {}",
            config.get_name()
        ))),
    }
}

/// `_actuator_to_cartesian` (`klippy/kinematics/deltesian.py:92`).
fn actuator_to_cartesian(arm_x: &[f64; 2], arm2: &[f64; 2], sp: &[f64; 2]) -> KResult<(f64, f64)> {
    let dx = pycompat::text::py_sum_floats(arm_x);
    let dz = sp[1] - sp[0];
    let pivots = py_sqrt(py_pow2(dx)? + py_pow2(dz)?)?;
    // Trilateration w/ reference frame along left to right pivots
    let xt = py_div(py_pow2(pivots)? + arm2[0] - arm2[1], 2.0 * pivots)?;
    let zt = py_sqrt(arm2[0] - py_pow2(xt)?)?;
    // Rotation and translation of the reference frame
    let x = py_div(xt * dx, pivots)? + py_div(zt * dz, pivots)? - arm_x[0];
    let z = py_div(xt * dz, pivots)? - py_div(zt * dx, pivots)? + sp[0];
    Ok((x, z))
}

/// `_pillars_z_max` (`klippy/kinematics/deltesian.py:103`).
fn pillars_z_max(
    arm_x: &[f64; 2],
    arm2: &[f64; 2],
    abs_endstop: &[f64; 2],
    x: f64,
) -> KResult<f64> {
    let dz0 = py_sqrt(arm2[0] - py_pow2(arm_x[0] + x)?)?;
    let dz1 = py_sqrt(arm2[1] - py_pow2(arm_x[1] - x)?)?;
    Ok(py_min(abs_endstop[0] - dz0, abs_endstop[1] - dz1))
}

impl KinematicsApi for DeltesianKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        let mut out = Vec::new();
        for rail in self.rails.iter() {
            out.extend(rail.get_steppers());
        }
        out
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut sp = [0.0f64; 3];
        for (i, rail) in self.rails.iter().enumerate() {
            let name = rail.get_name(None);
            match stepper_positions.get(&name) {
                Some(v) => sp[i] = *v,
                None => {
                    return Err(internal("KeyError", &pycompat::text::str_repr(&name)));
                }
            }
        }
        let (x, z) = actuator_to_cartesian(&self.arm_x, &self.arm2, &[sp[0], sp[1]])?;
        Ok(vec![Some(x), Some(sp[2]), Some(z)])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(index_error_list());
        }
        let coord = [newpos[0], newpos[1], newpos[2]];
        for rail in self.rails.iter() {
            rail.set_position(coord);
        }
        for c in homing_axes.chars() {
            let axis = match c {
                'x' => 0,
                'y' => 1,
                'z' => 2,
                _ => return Err(internal("ValueError", "substring not found")),
            };
            self.homed_axis.borrow_mut()[axis] = true;
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        let mut homed = self.homed_axis.borrow_mut();
        for (axis, name) in ["x", "y", "z"].iter().enumerate() {
            if clear_axes.contains(name) {
                homed[axis] = false;
            }
        }
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        let homing_axes = homing_state.get_axes();
        let home_xz = homing_axes.contains(&0) || homing_axes.contains(&2);
        let home_y = homing_axes.contains(&1);
        let forceaxes: Vec<i64> = if home_xz && home_y {
            vec![0, 1, 2]
        } else if home_xz {
            vec![0, 2]
        } else if home_y {
            vec![1]
        } else {
            Vec::new()
        };
        homing_state.set_axes(forceaxes);
        let mut homepos: Vec<Option<f64>> = vec![None; 4];
        if home_xz {
            homing_state.set_axes(if home_y { vec![0, 1, 2] } else { vec![0, 2] });
            homepos[0] = Some(0.0);
            homepos[2] = Some(self.home_z);
            let mut forcepos = homepos.clone();
            let dz2 = [
                self.arm2[0] - py_pow2(self.arm_x[0])?,
                self.arm2[1] - py_pow2(self.arm_x[1])?,
            ];
            let dz2_max = if dz2[1] > dz2[0] { dz2[1] } else { dz2[0] };
            forcepos[2] = Some(-1.5 * py_sqrt(dz2_max)?);
            homing_state.home_rails(
                vec![self.rails[0].clone(), self.rails[1].clone()],
                forcepos,
                homepos.clone(),
            )?;
        }
        if home_y {
            let (position_min, position_max) = self.rails[2].get_range();
            let hi = self.rails[2].get_homing_info();
            homepos[1] = Some(hi.position_endstop);
            let mut forcepos = homepos.clone();
            if hi.positive_dir {
                forcepos[1] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
            } else {
                forcepos[1] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
            }
            homing_state.home_rails(vec![self.rails[2].clone()], forcepos, homepos)?;
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let mut limits = self.limits;
        let spos = mv.start_pos();
        let epos = mv.end_pos();
        let axes_d = mv.axes_d();
        if spos.0.len() < 3 || epos.0.len() < 3 {
            return Err(index_error_tuple());
        }
        if axes_d.0.len() < 3 {
            return Err(index_error_list());
        }
        let mut homing_move = false;
        if epos.0[0] == 0.0 && epos.0[2] == self.home_z && axes_d.0[1] == 0.0 {
            // Identify a homing move
            homing_move = true;
        } else if epos.0[2] > limits[2].1 {
            // Moves at the very top - adapt limits depending on the X position
            limits[2].1 = self.pillars_z(epos.0[0])?;
        }
        for (i, limit) in limits.iter().enumerate() {
            if axes_d.0[i] != 0.0 {
                if !self.homed_axis.borrow()[i] {
                    return Err(mv.move_error("Must home axis first".to_string()));
                }
                // Move out of range - verify not a homing move
                if (epos.0[i] < limit.0 || epos.0[i] > limit.1) && !homing_move {
                    return Err(mv.move_error("Move out of range".to_string()));
                }
            }
        }
        if axes_d.0[2] != 0.0 {
            let z_ratio = py_div(mv.move_d(), kos::libm::fabs(axes_d.0[2]))?;
            mv.limit_speed(self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)?;
        }
        // Limit the speed/accel if is is at the extreme values of the x axis
        let slow = self.slow_x2.unwrap_or(0.0);
        let very_slow = self.very_slow_x2.unwrap_or(0.0);
        if axes_d.0[0] != 0.0 && slow != 0.0 && very_slow != 0.0 {
            let move_x2 = {
                let a = py_pow2(spos.0[0])?;
                let b = py_pow2(epos.0[0])?;
                if b > a {
                    b
                } else {
                    a
                }
            };
            if move_x2 > very_slow || move_x2 > slow {
                // The reference reads `self.max_velocity`, which the class never defines.
                return Err(internal(
                    "AttributeError",
                    "'DeltesianKinematics' object has no attribute 'max_velocity'",
                ));
            }
        }
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let homed = self.homed_axis.borrow();
        let mut axes = String::new();
        for (i, &b) in homed.iter().enumerate() {
            if b {
                axes.push_str(["x", "y", "z"][i]);
            }
        }
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str(axes));
        d.insert(
            PyValue::Str("axis_minimum".to_string()),
            PyValue::Tuple(self.axes_min.clone()),
        );
        d.insert(
            PyValue::Str("axis_maximum".to_string()),
            PyValue::Tuple(self.axes_max.clone()),
        );
        PyValue::Dict(d)
    }
}

/// `load_kinematics(toolhead, config)` (`klippy/kinematics/deltesian.py:183`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    Ok(DeltesianKinematics::new(toolhead, config)?)
}

/// Kinematics modules are loaded through `load_kinematics`, not by a printer section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
