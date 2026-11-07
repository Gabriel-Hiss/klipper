//! Contract of the `toolhead`, kinematics and `motion_queuing` services.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::Completion;

use crate::KResult;
use crate::mcu::{CommandQueue, Mcu};
use crate::stepper::{RailApi, Stepper};

/// `gcode.Coord` (a tuple of at least 4 floats; `x`, `y`, `z`, then the
/// extruder/extra-axis position).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Coord(pub Vec<f64>);

/// Opaque `trapq` handle from the ported `chelper_motion` crate.
pub type TrapqHandle = Arc<dyn Any + Send + Sync>;

/// Opaque `syncemitter` handle from the ported `chelper_motion` crate.
pub type SyncEmitterHandle = Arc<dyn Any + Send + Sync>;

/// Opaque homing-state handle (`extras/homing.py` `HomingState`).
pub type HomingStateHandle = Arc<dyn Any + Send + Sync>;

/// Opaque extra-axis handle (`kinematics/extruder.py` `ExtraAxis`).
pub type ExtraAxisHandle = Arc<dyn Any + Send + Sync>;

/// Opaque extruder handle (`kinematics/extruder.py` `PrinterExtruder`).
pub type PrinterExtruderHandle = Arc<dyn Any + Send + Sync>;

/// `trapq_append` from `motion_queuing.lookup_trapq_append`:
/// `(trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, axes_rx, axes_ry,
/// axes_rz, start_v, cruise_v, accel)`.
pub type TrapqAppend = Arc<
    dyn Fn(
            TrapqHandle,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
            f64,
        ) + Send
        + Sync,
>;

/// Flush callback `callback(flush_time, step_gen_time)`.
pub type FlushCallback = Arc<dyn Fn(f64, f64) + Send + Sync>;

/// The `ToolHead` object (`klippy/toolhead.py`).
pub trait ToolHeadApi: Send + Sync + 'static {
    /// `klippy/toolhead.py:383`
    fn get_position(&self) -> Vec<f64>;

    /// `klippy/toolhead.py:385`
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()>;

    /// `klippy/toolhead.py:397`
    /// Pauses the reactor.
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()>;

    /// `klippy/toolhead.py:414`
    /// Pauses the reactor.
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()>;

    /// `klippy/toolhead.py:421`
    /// Pauses the reactor.
    fn dwell(&self, delay: f64);

    /// `klippy/toolhead.py:424`
    /// Pauses the reactor.
    fn wait_moves(&self);

    /// `klippy/toolhead.py:320`
    fn get_last_move_time(&self) -> f64;

    /// `klippy/toolhead.py:317`
    fn flush_step_generation(&self) -> KResult<()>;

    /// `klippy/toolhead.py:476`
    /// Pauses the reactor.
    fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &Completion<f64>,
    ) -> KResult<()>;

    /// `klippy/toolhead.py:497`
    fn get_status(&self, eventtime: f64) -> PyValue;

    /// `klippy/toolhead.py:522`
    fn get_kinematics(&self) -> Kinematics;

    /// `klippy/toolhead.py:524`
    fn get_trapq(&self) -> TrapqHandle;

    /// `klippy/toolhead.py:534`
    fn get_max_velocity(&self) -> (f64, f64);

    /// `klippy/toolhead.py:539`
    fn set_max_velocities(
        &self,
        max_velocity: Option<f64>,
        max_accel: Option<f64>,
        square_corner_velocity: Option<f64>,
        min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64);

    /// `klippy/toolhead.py:526`
    fn register_lookahead_callback(&self, callback: Box<dyn FnMut(f64) + Send>);

    /// `klippy/toolhead.py:490`
    fn check_busy(&self, eventtime: f64) -> (f64, f64, bool);

    /// `klippy/toolhead.py:391`
    fn limit_next_junction_speed(&self, speed: f64);

    /// `klippy/toolhead.py:440`
    fn set_extruder(&self, extruder: PrinterExtruderHandle, extrude_pos: f64);

    /// `klippy/toolhead.py:443`
    fn get_extruder(&self) -> PrinterExtruderHandle;

    /// `klippy/toolhead.py:446`
    fn add_extra_axis(&self, ea: ExtraAxisHandle, axis_pos: f64);

    /// `klippy/toolhead.py:454`
    fn remove_extra_axis(&self, ea: ExtraAxisHandle);

    /// `klippy/toolhead.py:462`
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxisHandle>>;

    /// `klippy/toolhead.py:494`
    fn stats(&self, eventtime: f64) -> (bool, String);
}

/// The `ToolHead` service handle.
pub type ToolHead = Arc<dyn ToolHeadApi>;

/// A move under construction (`toolhead.py` `Move`), handed to
/// [`KinematicsApi::check_move`]; `limit_speed` mutates it in place.
pub trait MoveApi: Send + Sync + 'static {
    /// `klippy/toolhead.py:19`
    fn start_pos(&self) -> Coord;

    /// `klippy/toolhead.py:19`
    fn end_pos(&self) -> Coord;

    /// `klippy/toolhead.py:19`
    fn axes_d(&self) -> Coord;

    /// `klippy/toolhead.py:19`
    fn axes_r(&self) -> Coord;

    /// `klippy/toolhead.py:19`
    fn move_d(&self) -> f64;

    /// `klippy/toolhead.py:19`
    fn max_cruise_v2(&self) -> f64;

    /// `klippy/toolhead.py:32`
    fn limit_speed(&self, speed: f64, accel: f64);

    /// `klippy/toolhead.py:64`
    fn move_error(&self, msg: String) -> crate::KlippyError;
}

/// Delta calibration (`klippy/kinematics/delta.py` `DeltaCalibration`).
pub trait DeltaCalibrationApi: Send + Sync + 'static {
    /// `klippy/kinematics/delta.py:184`
    fn coordinate_descent_params(&self, is_extended: bool) -> PyValue;

    /// `klippy/kinematics/delta.py:212`
    fn calc_stable_position(&self, coord: &Coord) -> Coord;
}

/// Delta calibration handle.
pub type DeltaCalibration = Arc<dyn DeltaCalibrationApi>;

/// The kinematics interface returned by `ToolHead.get_kinematics`
/// (implemented by every `kinematics/*.py` module and by
/// `manual_stepper`'s duck type).
pub trait KinematicsApi: Send + Sync + 'static {
    /// `klippy/kinematics/cartesian.py:46`
    fn get_steppers(&self) -> Vec<Stepper>;

    /// `klippy/kinematics/cartesian.py:48`
    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>)
        -> Vec<Option<f64>>;

    /// `klippy/kinematics/cartesian.py:62`
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:106`
    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:90`
    fn home(&self, homing_state: HomingStateHandle) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:120`
    fn get_status(&self, eventtime: f64) -> PyValue;

    /// `klippy/kinematics/cartesian.py:73`
    fn clear_homing_state(&self, clear_axes: &str);

    /// `klippy/kinematics/cartesian.py:77`
    fn home_axis(
        &self,
        homing_state: HomingStateHandle,
        axis: i64,
        rail: Arc<dyn RailApi>,
    ) -> KResult<()>;

    /// `klippy/kinematics/delta.py:160`
    fn get_calibration(&self) -> Option<DeltaCalibration>;
}

/// The kinematics service handle.
pub type Kinematics = Arc<dyn KinematicsApi>;

/// `PrinterMotionQueuing` (`klippy/extras/motion_queuing.py`).
pub trait MotionQueuingApi: Send + Sync + 'static {
    /// `klippy/extras/motion_queuing.py:133`
    fn allocate_trapq(&self) -> TrapqHandle;

    /// `klippy/extras/motion_queuing.py:141`
    fn lookup_trapq_append(&self) -> TrapqAppend;

    /// `klippy/extras/motion_queuing.py:138`
    fn wipe_trapq(&self, trapq: TrapqHandle);

    /// `klippy/extras/motion_queuing.py:153`
    fn allocate_syncemitter(
        &self,
        mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> SyncEmitterHandle;

    /// `klippy/extras/motion_queuing.py:163`
    fn setup_mcu_movequeue(&self, mcu: Mcu, serialqueue: CommandQueue, move_count: i64);

    /// `klippy/extras/motion_queuing.py:182`
    fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool);

    /// `klippy/extras/motion_queuing.py:187`
    fn unregister_flush_callback(&self, callback: &FlushCallback);

    /// `klippy/extras/motion_queuing.py:193`
    fn get_kin_flush_delay(&self) -> f64;

    /// `klippy/extras/motion_queuing.py:195`
    fn check_step_generation_scan_windows(&self);

    /// `klippy/extras/motion_queuing.py:365`
    fn flush_all_steps(&self) -> KResult<()>;

    /// `klippy/extras/motion_queuing.py:369`
    fn calc_step_gen_restart(&self, est_print_time: f64) -> f64;

    /// `klippy/extras/motion_queuing.py:468`
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool);

    /// `klippy/extras/motion_queuing.py:477`
    fn drip_update_time(
        &self,
        start_time: f64,
        end_time: f64,
        drip_completion: &Completion<f64>,
    );

    /// `klippy/extras/motion_queuing.py:503`
    fn check_drip_timing(&self) -> Option<f64>;
}

/// The `motion_queuing` service handle.
pub type MotionQueuing = Arc<dyn MotionQueuingApi>;
