//! Port of the `none` module (`klippy/kinematics/none.py`).
//!
//! Dummy kinematics for developer testing: no steppers, no homing,
//! position reads as `[0, 0, 0]`.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{Coord, HomingStateHandle, Kinematics, KinematicsApi, MoveApi, ToolHead};
use pycompat::{PyInt, PyValue};

/// `klippy/kinematics/none.py:7` `NoneKinematics`.
pub struct NoneKinematics {
    axes_minmax: Coord,
}

impl NoneKinematics {
    /// `NoneKinematics.__init__(toolhead, config)` (`klippy/kinematics/none.py:8`).
    /// Only `toolhead.Coord` is used; `config` is ignored.
    pub fn new(_toolhead: &ToolHead, _config: &Config) -> NoneKinematics {
        // `toolhead.Coord((0., 0., 0.))` pads the 3-tuple to length 4
        // (`klippy/gcode.py:15`).
        NoneKinematics {
            axes_minmax: Coord(vec![0.0, 0.0, 0.0, 0.0]),
        }
    }
}

impl KinematicsApi for NoneKinematics {
    fn class_name(&self) -> &'static str {
        "NoneKinematics"
    }

    /// `get_steppers` (`klippy/kinematics/none.py:10`).
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }

    /// `calc_position` (`klippy/kinematics/none.py:12`).
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(vec![Some(0.0), Some(0.0), Some(0.0)])
    }

    /// `set_position` (`klippy/kinematics/none.py:14`).
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }

    /// `check_move` (`klippy/kinematics/none.py:20`).
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }

    /// `home` (`klippy/kinematics/none.py:18`).
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        Ok(())
    }

    /// `clear_homing_state` (`klippy/kinematics/none.py:16`).
    fn clear_homing_state(&self, _clear_axes: &str) -> KResult<()> {
        Ok(())
    }

    /// `get_status` (`klippy/kinematics/none.py:22`).
    fn get_status(&self, _eventtime: f64) -> KResult<PyValue> {
        // `Coord((0., 0., 0.))` pads with `(0,)` (the int zero,
        // `klippy/gcode.py:16`), so the stored value is
        // `(0.0, 0.0, 0.0, 0)`: the fourth element is an int.
        let minmax = PyValue::Tuple(vec![
            PyValue::Float(self.axes_minmax.0[0]),
            PyValue::Float(self.axes_minmax.0[1]),
            PyValue::Float(self.axes_minmax.0[2]),
            PyValue::Int(PyInt::from_i64(0)),
        ]);
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(String::new()),
        );
        status.insert(PyValue::Str("axis_minimum".to_string()), minmax.clone());
        status.insert(PyValue::Str("axis_maximum".to_string()), minmax);
        Ok(PyValue::Dict(status))
    }
}

/// `load_kinematics(toolhead, config)` (`klippy/kinematics/none.py:29`).
pub fn load_kinematics(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics> {
    let kin: Kinematics = Arc::new(NoneKinematics::new(toolhead, config));
    Ok(kin)
}
