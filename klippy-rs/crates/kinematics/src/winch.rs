//! Port of the `winch` kinematics module (`klippy/kinematics/winch.py`).
//!
//! Unlike the other kinematics modules there is no `load_config` entry
//! point here: the toolhead reaches this module by name through
//! [`load_kinematics`] (the Python `load_kinematics(toolhead, config)`
//! function), not through the printer [`ModuleFactory`] table. The
//! factory below exists for uniformity and has every slot empty.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{
    HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead,
};
use klippy_core::mathutil::trilateration;
use pycompat::{PyInt, PyValue};

/// `WinchKinematics` (`klippy/kinematics/winch.py:8`).
pub struct WinchKinematics {
    steppers: Vec<Stepper>,
    anchors: Vec<[f64; 3]>,
    axes_min: [f64; 4],
    axes_max: [f64; 4],
}

impl WinchKinematics {
    /// `WinchKinematics.__init__(toolhead, config)` (`winch.py:9`).
    pub fn new(toolhead: &ToolHead, config: &Config) -> KResult<WinchKinematics> {
        let mut steppers: Vec<Stepper> = Vec::new();
        let mut anchors: Vec<[f64; 3]> = Vec::new();
        for i in 0..26i32 {
            let name = format!("stepper_{}", (b'a' + i as u8) as char);
            if i >= 3 && !config.has_section(&name) {
                break;
            }
            let stepper_config = config.getsection(&name);
            let s = toolhead
                .stepper_factory()
                .printer_stepper(&stepper_config, false)?;
            let ax = anchor_float(&stepper_config, "anchor_x")?;
            let ay = anchor_float(&stepper_config, "anchor_y")?;
            let az = anchor_float(&stepper_config, "anchor_z")?;
            let a = [ax, ay, az];
            s.setup_itersolve(
                "winch_stepper_alloc",
                &[PyValue::Float(a[0]), PyValue::Float(a[1]), PyValue::Float(a[2])],
            );
            s.set_trapq(Some(toolhead.get_trapq()));
            steppers.push(s);
            anchors.push(a);
        }
        if anchors.is_empty() {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "min() arg is an empty sequence".to_string(),
            });
        }
        let mut axes_min = [0.0f64; 4];
        let mut axes_max = [0.0f64; 4];
        for axis in 0..3 {
            let mut lo = anchors[0][axis];
            let mut hi = anchors[0][axis];
            for a in &anchors[1..] {
                if a[axis] < lo {
                    lo = a[axis];
                }
                if hi < a[axis] {
                    hi = a[axis];
                }
            }
            axes_min[axis] = lo;
            axes_max[axis] = hi;
        }
        let kin = WinchKinematics {
            steppers,
            anchors,
            axes_min,
            axes_max,
        };
        kin.set_position(vec![0.0, 0.0, 0.0], "")?;
        Ok(kin)
    }

    /// The anchor positions `stepper_a`, `stepper_b`, ... (`self.anchors`).
    pub fn anchors(&self) -> &[[f64; 3]] {
        &self.anchors
    }

    /// The (padded) minimum anchor position (`self.axes_min`).
    pub fn axes_min(&self) -> [f64; 4] {
        self.axes_min
    }

    /// The (padded) maximum anchor position (`self.axes_max`).
    pub fn axes_max(&self) -> [f64; 4] {
        self.axes_max
    }
}

fn anchor_float(config: &Config, option: &str) -> KResult<f64> {
    match config.getfloat(option, GetFloatOpts::default())? {
        Some(v) => Ok(v),
        None => Err(KlippyError::Config(format!(
            "Missing option '{option}' in section '{name}'",
            name = config.get_name()
        ))),
    }
}

/// A [`Coord`] as a status tuple: three floats plus the int `0` the
/// Python `Coord` constructor pads with (`gcode.py:15`).
fn coord_value(coord: [f64; 4]) -> PyValue {
    PyValue::Tuple(vec![
        PyValue::Float(coord[0]),
        PyValue::Float(coord[1]),
        PyValue::Float(coord[2]),
        PyValue::Int(PyInt::from_i64(0)),
    ])
}

impl KinematicsApi for WinchKinematics {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }

    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let mut pos = [0.0f64; 3];
        for (idx, rail) in self.steppers.iter().take(3).enumerate() {
            let name = rail.get_name(None);
            match stepper_positions.get(&name) {
                Some(&v) => pos[idx] = v,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: pycompat::value::repr_key(&name),
                    })
                }
            }
        }
        let radius2 = [pos[0] * pos[0], pos[1] * pos[1], pos[2] * pos[2]];
        let sphere_coords = [self.anchors[0], self.anchors[1], self.anchors[2]];
        let out = trilateration(&sphere_coords, &radius2)?;
        Ok(vec![Some(out[0]), Some(out[1]), Some(out[2])])
    }

    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        if newpos.len() < 3 {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        }
        let coord = [newpos[0], newpos[1], newpos[2]];
        for s in &self.steppers {
            s.set_position(coord);
        }
        Ok(())
    }

    fn clear_homing_state(&self, _clear_axes: &str) {}

    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        homing_state.set_axes(vec![0, 1, 2]);
        homing_state.set_homed_position(vec![Some(0.0), Some(0.0), Some(0.0)])
    }

    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut dict = IndexMap::new();
        dict.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str("xyz".to_string()),
        );
        dict.insert(
            PyValue::Str("axis_minimum".to_string()),
            coord_value(self.axes_min),
        );
        dict.insert(
            PyValue::Str("axis_maximum".to_string()),
            coord_value(self.axes_max),
        );
        PyValue::Dict(dict)
    }
}

/// `load_kinematics(toolhead, config)` (`winch.py:56`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    Ok(std::sync::Arc::new(WinchKinematics::new(toolhead, config)?) as Kinematics)
}

/// Module entry point. The Python module only exposes `load_kinematics`,
/// so no printer object is ever registered through this factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
