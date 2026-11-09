//! Port of `klippy/kinematics/corexz.py`.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Rail, RailOpts};
use klippy_api::toolhead::{
    Coord, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use pycompat::PyValue;

/// CoreXZ kinematics (`klippy/kinematics/corexz.py:9`).
pub struct CoreXZKinematics {
    rails: [Rail; 3],
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: GCell<[(f64, f64); 3]>,
    axes_min: Coord,
    axes_max: Coord,
}

impl CoreXZKinematics {
    /// `klippy/kinematics/corexz.py:10` `__init__(toolhead, config)`.
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<CoreXZKinematics> {
        let factory = toolhead.stepper_factory();
        let mut rails = Vec::with_capacity(3);
        for n in ['x', 'y', 'z'] {
            let section = config.getsection(&format!("stepper_{n}"));
            rails.push(factory.lookup_multi_rail(&section, RailOpts::default())?);
        }
        let rails: [Rail; 3] = rails.try_into().unwrap_or_else(|_| unreachable!("corexz: three rails pushed above"));

        for s in rails[0].get_steppers() {
            let endstop_z = rails[2].get_endstops();
            let endstop_z = endstop_z.first().map(|(e, _)| e.clone());
            let endstop_z = match endstop_z {
                Some(e) => e,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list index out of range".to_string(),
                    })
                }
            };
            endstop_z.add_stepper(s);
        }

        for s in rails[2].get_steppers() {
            let endstop_x = rails[0].get_endstops();
            let endstop_x = match endstop_x.first().map(|(e, _)| e.clone()) {
                Some(e) => e,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list index out of range".to_string(),
                    })
                }
            };
            endstop_x.add_stepper(s);
        }

        rails[0].setup_itersolve("corexz_stepper_alloc", &[PyValue::Bytes(vec![b'+'])]);
        rails[1].setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'y'])]);
        rails[2].setup_itersolve("corexz_stepper_alloc", &[PyValue::Bytes(vec![b'-'])]);

        for rail in &rails {
            for s in rail.get_steppers() {
                // Python evaluates `toolhead.get_trapq()` once per stepper.
                let _ = s.set_trapq(Some(toolhead.get_trapq()));
            }
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

        let ranges: Vec<(f64, f64)> = rails.iter().map(|r| r.get_range()).collect();
        let axes_min = Coord(vec![ranges[0].0, ranges[1].0, ranges[2].0, 0.0]);
        let axes_max = Coord(vec![ranges[0].1, ranges[1].1, ranges[2].1, 0.0]);

        Ok(CoreXZKinematics {
            rails,
            max_z_velocity,
            max_z_accel,
            limits: GCell::new([(1.0, -1.0); 3]),
            axes_min,
            axes_max,
        })
    }

    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        let mut out = Vec::new();
        for rail in &self.rails {
            out.extend(rail.get_steppers());
        }
        out
    }

    fn _check_endstops(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let limits = self.limits.cloned();
        for i in 0..3 {
            let axes_i = axes_d.0.get(i).copied().unwrap_or(0.0);
            if axes_i != 0.0 {
                let ep = end_pos.0.get(i).copied().unwrap_or(f64::NAN);
                if ep < limits[i].0 || ep > limits[i].1 {
                    if limits[i].0 > limits[i].1 {
                        return Err(mv.move_error("Must home axis first".to_string()));
                    }
                    return Err(mv.move_error("Move out of range".to_string()));
                }
            }
        }
        Ok(())
    }
}

impl KinematicsApi for CoreXZKinematics {
    fn class_name(&self) -> &'static str {
        "CoreXZKinematics"
    }

    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        CoreXZKinematics::get_steppers(self)
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut pos = Vec::with_capacity(3);
        for rail in &self.rails {
            let name = rail.get_name(None);
            let v = stepper_positions.get(&name).copied().ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: format!("'{name}'"),
            })?;
            pos.push(v);
        }
        Ok(vec![
            Some(0.5 * (pos[0] + pos[2])),
            Some(pos[1]),
            Some(0.5 * (pos[0] - pos[2])),
        ])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        let coord: [f64; 3] = if newpos.len() >= 3 {
            [newpos[0], newpos[1], newpos[2]]
        } else {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        };
        for (i, rail) in self.rails.iter().enumerate() {
            rail.set_position(coord);
            let axis_char = ["x", "y", "z"][i];
            if homing_axes.contains(axis_char) {
                let range = rail.get_range();
                self.limits.with_mut(|limits| limits[i] = range);
            }
        }
        Ok(())
    }

    fn clear_homing_state(&self, clear_axes: &str) -> KResult<()> {
        for (axis, axis_name) in ["x", "y", "z"].iter().enumerate() {
            if clear_axes.contains(axis_name) {
                self.limits.with_mut(|limits| limits[axis] = (1.0, -1.0));
            }
        }
        Ok(())
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        for axis in homing_state.get_axes() {
            let rail = if axis >= 0 && axis < 3 {
                self.rails[axis as usize].clone()
            } else if axis >= -3 && axis < 0 {
                self.rails[(axis + 3) as usize].clone()
            } else {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                });
            };
            let (position_min, position_max) = rail.get_range();
            let hi = rail.get_homing_info();
            let mut homepos: Vec<Option<f64>> = vec![None, None, None, None];
            // Python `homepos[axis]` on a length-4 list wraps negative
            // axes (e.g. `homepos[-1]` is index 3).
            let pidx = if axis < 0 { (4 + axis) as usize } else { axis as usize };
            homepos[pidx] = Some(hi.position_endstop);
            let mut forcepos = homepos.clone();
            if hi.positive_dir {
                let f = 1.5 * (hi.position_endstop - position_min);
                forcepos[pidx] = Some(hi.position_endstop - f);
            } else {
                let f = 1.5 * (position_max - hi.position_endstop);
                forcepos[pidx] = Some(hi.position_endstop + f);
            }
            homing_state.home_rails(vec![rail], forcepos, homepos)?;
        }
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> KResult<PyValue> {
        let limits = self.limits.cloned();
        let mut axes = String::new();
        for (a, (l, h)) in ["x", "y", "z"].iter().zip(limits.iter()) {
            if l <= h {
                axes.push_str(a);
            }
        }
        let mut dict = IndexMap::new();
        dict.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(axes),
        );
        let mut amin_vals: Vec<PyValue> = self.axes_min
            .0
            .iter()
            .map(|v| PyValue::Float(*v))
            .collect();
        if amin_vals.len() >= 4 {
            amin_vals[3] = PyValue::Int(pycompat::PyInt::from_i64(0));
        }
        dict.insert(
            PyValue::Str("axis_minimum".to_string()),
            PyValue::Tuple(amin_vals),
        );
        let mut amax_vals: Vec<PyValue> = self.axes_max
            .0
            .iter()
            .map(|v| PyValue::Float(*v))
            .collect();
        if amax_vals.len() >= 4 {
            amax_vals[3] = PyValue::Int(pycompat::PyInt::from_i64(0));
        }
        dict.insert(
            PyValue::Str("axis_maximum".to_string()),
            PyValue::Tuple(amax_vals),
        );
        Ok(PyValue::Dict(dict))
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let limits = self.limits.cloned();
        let end_pos = mv.end_pos();
        let xpos = end_pos.0.get(0).copied().unwrap_or(f64::NAN);
        let ypos = end_pos.0.get(1).copied().unwrap_or(f64::NAN);
        if xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1
        {
            self._check_endstops(mv)?;
        }
        let axes_d = mv.axes_d();
        let z = axes_d.0.get(2).copied().unwrap_or(0.0);
        if z == 0.0 {
            return Ok(());
        }
        self._check_endstops(mv)?;
        let z_ratio = mv.move_d() / z.abs();
        mv.limit_speed(
            self.max_z_velocity * z_ratio,
            self.max_z_accel * z_ratio,
        )?;
        Ok(())
    }
}

/// `klippy/kinematics/corexz.py:94` `load_kinematics(toolhead, config)`.
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    Ok(Arc::new(CoreXZKinematics::new(toolhead, config)?))
}

/// Module factory. Kinematics modules are loaded by the toolhead through
/// `load_kinematics`, not by the printer, so no printer-object entry points exist.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
