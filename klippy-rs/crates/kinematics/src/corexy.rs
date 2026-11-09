//! Port of `klippy/kinematics/corexy.py`.

use indexmap::IndexMap;
use std::sync::Arc;

use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::gcell::GCell;
use klippy_api::object::PrinterObject;
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Rail, RailOpts, Stepper};
use klippy_api::toolhead::{
    Coord, HomingStateApi, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_api::{KResult, KlippyError};
use pycompat::PyValue;

/// `toolhead.Coord` (`klippy/gcode.py:12`): pad to 4 elements with `0`.
fn to_coord(min: f64, mid: f64, max: f64) -> Coord {
    Coord(vec![min, mid, max, 0.0])
}

/// `Coord` as the Python `Coord` tuple for `get_status`.
///
/// Python pads with the int `0` (`klippy/gcode.py:15`), so the 4th element
/// is an int, the first three are floats.
fn coord_to_pyvalue(coord: &Coord) -> PyValue {
    let mut items = Vec::with_capacity(4);
    for (i, v) in coord.0.iter().enumerate() {
        if i < 3 {
            items.push(PyValue::Float(*v));
        }
    }
    while items.len() < 3 {
        items.push(PyValue::Float(0.0));
    }
    items.push(PyValue::Int(pycompat::PyInt::from_i64(0)));
    PyValue::Tuple(items)
}

/// Index error matching Python `list index out of range`.
fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `CoreXYKinematics` (`klippy/kinematics/corexy.py:9`).
pub struct CoreXYKinematics {
    rails: [Rail; 3],
    max_z_velocity: f64,
    max_z_accel: f64,
    limits: GCell<[(f64, f64); 3]>,
    axes_min: Coord,
    axes_max: Coord,
}

impl CoreXYKinematics {
    /// `CoreXYKinematics.__init__` with pre-built rails (`klippy/kinematics/corexy.py:10`).
    ///
    /// Cross-links X/Y steppers with the shared endstops, calls
    /// `setup_itersolve`, `set_trapq`, reads `max_z_*` config options and
    /// records axis ranges.
    pub fn new_with_rails(
        rails: [Rail; 3],
        toolhead: &ToolHead,
        config: &Config,
    ) -> KResult<Self> {
        // Cross-link X/Y steppers with the shared endstops.
        for s in rails[1].get_steppers() {
            let endstops = rails[0].get_endstops();
            let entry = endstops.first().ok_or_else(index_error)?;
            entry.0.add_stepper(s);
        }
        for s in rails[0].get_steppers() {
            let endstops = rails[1].get_endstops();
            let entry = endstops.first().ok_or_else(index_error)?;
            entry.0.add_stepper(s);
        }
        rails[0].setup_itersolve("corexy_stepper_alloc", &[PyValue::Bytes(vec![b'+'])]);
        rails[1].setup_itersolve("corexy_stepper_alloc", &[PyValue::Bytes(vec![b'-'])]);
        rails[2].setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'z'])]);
        let trapq = toolhead.get_trapq();
        for s in Self::steppers_of(&rails) {
            s.set_trapq(Some(trapq.clone()));
        }
        // Boundary checks.
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
        let limits = GCell::new([(1.0f64, -1.0f64); 3]);
        let ranges = [rails[0].get_range(), rails[1].get_range(), rails[2].get_range()];
        let axes_min = to_coord(ranges[0].0, ranges[1].0, ranges[2].0);
        let axes_max = to_coord(ranges[0].1, ranges[1].1, ranges[2].1);
        Ok(CoreXYKinematics {
            rails,
            max_z_velocity,
            max_z_accel,
            limits,
            axes_min,
            axes_max,
        })
    }

    fn steppers_of(rails: &[Rail; 3]) -> Vec<Stepper> {
        let mut out = Vec::new();
        for rail in rails.iter() {
            out.extend(rail.get_steppers());
        }
        out
    }

    /// `get_steppers` (`klippy/kinematics/corexy.py:33`).
    pub fn get_steppers(&self) -> Vec<Stepper> {
        Self::steppers_of(&self.rails)
    }

    /// One axis homing step of `home` (`klippy/kinematics/corexy.py:47`).
    fn home_one_axis(
        &self,
        homing_state: &dyn HomingStateApi,
        axis: usize,
        rail: &Rail,
    ) -> KResult<()> {
        let (position_min, position_max) = rail.get_range();
        let hi = rail.get_homing_info();
        let mut homepos: [Option<f64>; 4] = [None; 4];
        homepos[axis] = Some(hi.position_endstop);
        let mut forcepos = homepos;
        if hi.positive_dir {
            forcepos[axis] = Some(hi.position_endstop - 1.5 * (hi.position_endstop - position_min));
        } else {
            forcepos[axis] = Some(hi.position_endstop + 1.5 * (position_max - hi.position_endstop));
        }
        homing_state.home_rails(vec![rail.clone()], forcepos.to_vec(), homepos.to_vec())
    }

    /// `_check_endstops` (`klippy/kinematics/corexy.py:63`).
    fn check_endstops(&self, mv: &dyn MoveApi) -> KResult<()> {
        let end_pos = mv.end_pos();
        let axes_d = mv.axes_d();
        let limits = self.limits.cloned();
        for (i, (lo, hi)) in limits.iter().enumerate() {
            let end_i = end_pos.0.get(i).copied().unwrap_or(0.0);
            let d = axes_d.0.get(i).copied().unwrap_or(0.0);
            if d != 0.0 && (end_i < *lo || end_i > *hi) {
                if lo > hi {
                    return Err(mv.move_error("Must home axis first".to_string()));
                }
                return Err(mv.move_error("Move out of range".to_string()));
            }
        }
        Ok(())
    }

    /// `check_move` (`klippy/kinematics/corexy.py:72`).
    pub fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        let limits = self.limits.cloned();
        let end_pos = mv.end_pos();
        let xpos = end_pos.0.first().copied().unwrap_or(0.0);
        let ypos = end_pos.0.get(1).copied().unwrap_or(0.0);
        if xpos < limits[0].0 || xpos > limits[0].1 || ypos < limits[1].0 || ypos > limits[1].1 {
            self.check_endstops(mv)?;
        }
        let axes_d = mv.axes_d();
        let zd = axes_d.0.get(2).copied().unwrap_or(0.0);
        if zd == 0.0 {
            // Normal XY move - use defaults.
            return Ok(());
        }
        // Move with Z - update velocity and accel for slower Z axis.
        self.check_endstops(mv)?;
        let z_ratio = mv.move_d() / zd.abs();
        mv.limit_speed(
            self.max_z_velocity * z_ratio,
            self.max_z_accel * z_ratio,
        )?;
        Ok(())
    }

    /// `get_status` (`klippy/kinematics/corexy.py:86`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let limits = self.limits.cloned();
        let axes: String = ("xyz")
            .chars()
            .zip(limits.iter())
            .filter(|(_, (l, h))| l <= h)
            .map(|(a, _)| a)
            .collect();
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(axes),
        );
        d.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_to_pyvalue(&self.axes_min),
        );
        d.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_to_pyvalue(&self.axes_max),
        );
        PyValue::Dict(d)
    }
}

impl KinematicsApi for CoreXYKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.get_steppers()
    }

    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        let mut pos: [f64; 3] = [0.0; 3];
        for (i, rail) in self.rails.iter().enumerate() {
            let name = rail.get_name(None);
            pos[i] = *stepper_positions.get(&name).ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: format!("'{name}'"),
            })?;
        }
        // `0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]`.
        Ok(vec![
            Some(0.5 * (pos[0] + pos[1])),
            Some(0.5 * (pos[0] - pos[1])),
            Some(pos[2]),
        ])
    }

    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        let coord: [f64; 3] = [
            newpos.first().copied().unwrap_or(0.0),
            newpos.get(1).copied().unwrap_or(0.0),
            newpos.get(2).copied().unwrap_or(0.0),
        ];
        for (i, rail) in self.rails.iter().enumerate() {
            rail.set_position(coord);
            let letter = "xyz".as_bytes()[i] as char;
            if homing_axes.contains(letter) {
                let range = rail.get_range();
                self.limits.with_mut(|limits| limits[i] = range);
            }
        }
        Ok(())
    }

    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        self.check_move(mv)
    }

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        for axis in homing_state.get_axes() {
            // Python `self.rails[axis]` wraps negative indices.
            let idx = usize::try_from(if axis < 0 { 3 + axis } else { axis })
                .map_err(|_| index_error())?;
            let rail = self.rails.get(idx).ok_or_else(index_error)?;
            self.home_one_axis(&*homing_state, idx, rail)?;
        }
        Ok(())
    }

    fn get_status(&self, eventtime: f64) -> PyValue {
        self.get_status(eventtime)
    }

    fn clear_homing_state(&self, clear_axes: &str) {
        for (axis, axis_name) in "xyz".chars().enumerate() {
            if clear_axes.contains(axis_name) {
                self.limits.with_mut(|limits| limits[axis] = (1.0, -1.0));
            }
        }
    }
}

impl PrinterObject for CoreXYKinematics {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn KinematicsApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `load_kinematics` (`klippy/kinematics/corexy.py:94`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    let factory = toolhead.stepper_factory();
    let rails = [
        factory.lookup_multi_rail(&config.getsection("stepper_x"), RailOpts::default())?,
        factory.lookup_multi_rail(&config.getsection("stepper_y"), RailOpts::default())?,
        factory.lookup_multi_rail(&config.getsection("stepper_z"), RailOpts::default())?,
    ];
    let kin = CoreXYKinematics::new_with_rails(rails, toolhead, config)?;
    Ok(Arc::new(kin))
}

/// Kinematics modules are not printer sections; there is no factory entry
/// point in the Python module (`load_kinematics` only). The factory is empty.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
