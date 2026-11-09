//! Contract of the `toolhead`, kinematics, homing and `motion_queuing` services.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::Completion;

use crate::config::Config;
use crate::gcode::GCmd;
use crate::mcu::{Endstop, Mcu};
use crate::object::ObjectRef;
use crate::stepper::{PullHistoryStep, Rail, Stepper, StepperFactory, StepperKinematics, Trapq};
use crate::KResult;
use chelper_serial::serialqueue::SerialQueue;

/// `gcode.Coord` (a tuple of at least 4 floats; `x`, `y`, `z`, then the
/// extruder/extra-axis position).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Coord(pub Vec<f64>);

/// A `syncemitter` (`klippy/chelper/steppersync.c` `struct syncemitter`): the
/// message queue of one stepper (or of a non-stepper user such as `pwm_tool`).
///
/// `stepper.py` calls `syncemitter_get_stepcompress` once and then the
/// `stepcompress_*` calls on it (`stepper.py:50-52,130,152,181,188,206,222`),
/// plus `syncemitter_set_stepper_kinematics` (`stepper.py:200`);
/// `motion_queuing.py` calls `syncemitter_get_stepper_kinematics`
/// (`motion_queuing.py:199`); `stepper.py:210` and `pwm_tool.py:110` call
/// `syncemitter_queue_msg`. The trait covers exactly those calls. The
/// implementation lives on the newtype that owns the ported C object
/// (`core::motion_handles::CoreSyncEmitter`), never as a second trait.
pub trait SyncEmitterApi: Send + Sync + 'static {
    /// Upcast for the owning crate's downcast.
    fn as_any(&self) -> &dyn Any;
    /// `syncemitter_queue_msg` (`steppersync.c:73`).
    fn queue_msg(&self, req_clock: u64, data: &[u32]);
    /// `syncemitter_set_stepper_kinematics` (`steppersync.c:58`); `None` clears it.
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>);
    /// `syncemitter_get_stepper_kinematics` (`steppersync.c:66`).
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics>;
    /// `stepcompress_fill` (`stepper.py:130`).
    fn stepcompress_fill(
        &self,
        oid: u32,
        max_error: u32,
        queue_step_msgtag: i32,
        set_next_step_dir_msgtag: i32,
    );
    /// `stepcompress_set_invert_sdir` (`stepper.py:52,152`).
    fn stepcompress_set_invert_sdir(&self, invert: u32);
    /// `stepcompress_reset` (`stepper.py:206`).
    fn stepcompress_reset(&self, last_step_clock: u64) -> i32;
    /// `stepcompress_set_last_position` (`stepper.py:222`).
    fn stepcompress_set_last_position(&self, clock: u64, last_position: i64) -> i32;
    /// `stepcompress_find_past_position` (`stepper.py:181`).
    fn stepcompress_find_past_position(&self, clock: u64) -> i64;
    /// `stepcompress_extract_old` (`stepper.py:188`).
    fn stepcompress_extract_old(
        &self,
        max: i32,
        start_clock: u64,
        end_clock: u64,
    ) -> Vec<PullHistoryStep>;
}

/// Opaque `syncemitter` handle from `motion_queuing.allocate_syncemitter`;
/// `stepper` and `pwm_tool` consume it through [`SyncEmitterApi`].
pub type SyncEmitterHandle = Arc<dyn SyncEmitterApi>;

/// `trapq_append` from `motion_queuing.lookup_trapq_append`:
/// `(trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, axes_rx, axes_ry,
/// axes_rz, start_v, cruise_v, accel)`.
pub type TrapqAppend = Arc<
    dyn Fn(
            &Trapq,
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
pub type FlushCallback = Arc<dyn Fn(f64, f64) -> KResult<()> + Send + Sync>;

/// Lookahead callback `callback(print_time)` (`toolhead.py:526`). Python lets its
/// exceptions propagate out of the flush that runs it.
pub type LookaheadCallback = Box<dyn FnMut(f64) -> KResult<()> + Send>;

/// `load_kinematics(toolhead, config)` of a `kinematics/*.py` module. The core toolhead
/// receives a table of these (it cannot depend on the `kinematics` crate).
pub type LoadKinematics = fn(toolhead: &ToolHead, config: &Config) -> KResult<Kinematics>;

/// The `homing` printer object (`klippy/extras/homing.py` `PrinterHoming`), as used by
/// probes, `manual_stepper` and `homing_override`.
pub trait PrinterHomingApi: Send + Sync + 'static {
    /// `klippy/extras/homing.py:307` `manual_home(toolhead, endstops, pos, speed,
    /// probe_pos, triggered, check_triggered)`.
    /// Pauses the reactor.
    #[allow(clippy::too_many_arguments)]
    fn manual_home(
        &self,
        toolhead: HomingTarget,
        endstops: Vec<(Endstop, String)>,
        pos: Vec<Option<f64>>,
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    ) -> KResult<Vec<f64>>;

    /// `klippy/extras/homing.py:323` `probing_move(mcu_probe, pos, speed, check_movement)`.
    /// Pauses the reactor.
    fn probing_move(
        &self,
        mcu_probe: Endstop,
        pos: Vec<Option<f64>>,
        speed: f64,
        check_movement: bool,
    ) -> KResult<Vec<f64>>;

    /// `klippy/extras/homing.py:320` `check_probe_first_home(gcmd)`: the command is
    /// `G28` and `HOME_ATTEMPT_NUM` is the string `'1'`.
    fn check_probe_first_home(&self, gcmd: &GCmd) -> KResult<bool>;
}

/// The homing-request state (`klippy/extras/homing.py:164` `Homing`) handed to
/// `kin.home()` and to the `homing:home_rails_begin/end` handlers.
pub trait HomingStateApi: Send + Sync + 'static {
    /// `klippy/extras/homing.py:171`
    fn set_axes(&self, axes: Vec<i64>);
    /// `klippy/extras/homing.py:173`
    fn get_axes(&self) -> Vec<i64>;
    /// `klippy/extras/homing.py:175` — missing name is a `KeyError`.
    fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64>;
    /// `klippy/extras/homing.py:177`
    fn set_stepper_adjustment(&self, stepper_name: &str, adjustment: f64);
    /// `klippy/extras/homing.py:186`
    fn set_homed_position(&self, pos: Vec<Option<f64>>) -> KResult<()>;
    /// `klippy/extras/homing.py:286` `home_rails(rails, forcepos, movepos)`.
    /// Pauses the reactor.
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()>;
}

/// `Homing` handle.
pub type HomingStateHandle = Arc<dyn HomingStateApi>;

/// A homing/probing move (`klippy/extras/homing.py:45` `HomingMove`), the payload of
/// `homing:homing_move_begin/end`.
pub trait HomingMoveApi: Send + Sync + 'static {
    /// `klippy/extras/homing.py:53`
    fn get_mcu_endstops(&self) -> Vec<Endstop>;
    /// `klippy/extras/homing.py:155` — name of a stepper that did not move, if any.
    fn check_no_movement(&self) -> Option<String>;
}

/// `HomingMove` handle.
pub type HomingMoveHandle = Arc<dyn HomingMoveApi>;

/// The "toolhead" a `HomingMove` drives: the real toolhead, or `manual_stepper`'s duck
/// type. [`ToolHeadApi`] extends it, so a `ToolHead` upcasts to a [`HomingTarget`].
pub trait HomingTargetApi: Send + Sync + 'static {
    /// `klippy/toolhead.py:383`
    fn get_position(&self) -> Vec<f64>;
    /// `klippy/toolhead.py:385`
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()>;
    /// `klippy/toolhead.py:522`
    fn get_kinematics(&self) -> Kinematics;
    /// `klippy/toolhead.py:317`
    fn flush_step_generation(&self) -> KResult<()>;
    /// `klippy/toolhead.py:320`
    fn get_last_move_time(&self) -> KResult<f64>;
    /// `klippy/toolhead.py:421`
    /// Pauses the reactor.
    fn dwell(&self, delay: f64) -> KResult<()>;
    /// `klippy/toolhead.py:476`
    /// Pauses the reactor.
    fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()>;
}

/// Homing-target handle.
pub type HomingTarget = Arc<dyn HomingTargetApi>;

/// An entry of `toolhead.extra_axes` (`DummyExtruder`, `PrinterExtruder`,
/// `ManualStepper` registered as a G-Code axis). `ea_index` is the move-coordinate index.
pub trait ExtraAxisApi: Send + Sync + 'static {
    /// `klippy/kinematics/extruder.py:198`
    fn get_name(&self) -> String;
    /// `klippy/kinematics/extruder.py:204` — `None` for a manual stepper not bound to an axis.
    fn get_axis_gcode_id(&self) -> Option<String>;
    /// `klippy/kinematics/extruder.py:202` — `None` for `DummyExtruder`.
    fn get_trapq(&self) -> Option<Trapq>;
    /// `klippy/kinematics/extruder.py:208`
    fn check_move(&self, mv: &dyn MoveApi, ea_index: usize) -> KResult<()>;
    /// `klippy/kinematics/extruder.py:236` — `(instant_corner_v / abs(diff_r))**2` raises
    /// `OverflowError` when the square overflows.
    fn calc_junction(&self, prev_move: &dyn MoveApi, mv: &dyn MoveApi, ea_index: usize)
        -> KResult<f64>;
    /// `klippy/kinematics/extruder.py:241` — no error channel: the Python implementations
    /// only do float arithmetic and a `trapq_append` cffi call, neither of which raises.
    fn process_move(&self, print_time: f64, mv: &dyn MoveApi, ea_index: usize);
}

/// Extra-axis handle.
pub type ExtraAxis = Arc<dyn ExtraAxisApi>;

/// `toolhead.get_extruder()` (`PrinterExtruder` or `DummyExtruder`).
pub trait PrinterExtruderApi: ExtraAxisApi {
    /// `klippy/kinematics/extruder.py:200` — the `heaters.Heater`; callers take its
    /// interface (`dyn extras::heaters::HeaterApi`). `DummyExtruder` raises
    /// `command_error("Extruder not configured")`.
    fn get_heater(&self) -> KResult<ObjectRef>;
    /// `klippy/kinematics/extruder.py:256`
    fn find_past_position(&self, print_time: f64) -> KResult<f64>;
    /// `klippy/kinematics/extruder.py:206` — `None` for `DummyExtruder` (no `stats`).
    fn stats(&self, eventtime: f64) -> Option<(bool, String)>;
    /// The `ExtruderStepper` driving this extruder (`extruder_stepper` attribute).
    fn extruder_stepper(&self) -> Option<ExtruderStepper>;
}

/// Extruder handle.
pub type PrinterExtruder = Arc<dyn PrinterExtruderApi>;

/// `kinematics/extruder.py:9` `ExtruderStepper`, also wrapped by `extras/extruder_stepper.py`.
pub trait ExtruderStepperApi: Send + Sync + 'static {
    /// `klippy/kinematics/extruder.py:43`
    fn get_status(&self, eventtime: f64) -> PyValue;
    /// `klippy/kinematics/extruder.py:47`
    fn find_past_position(&self, print_time: f64) -> KResult<f64>;
    /// `klippy/kinematics/extruder.py:50` — `None`/`""` desyncs.
    fn sync_to_extruder(&self, extruder_name: Option<&str>) -> KResult<()>;
    /// `extruder_stepper.stepper` (`klippy/kinematics/extruder.py:94`).
    fn stepper(&self) -> crate::stepper::Stepper;
    /// `extruder_stepper.cmd_SET_PRESSURE_ADVANCE(gcmd)`
    /// (`klippy/kinematics/extruder.py:97`).
    fn cmd_set_pressure_advance(&self, gcmd: &GCmd) -> KResult<()>;
}

/// `ExtruderStepper` handle.
pub type ExtruderStepper = Arc<dyn ExtruderStepperApi>;

/// The `ToolHead` object (`klippy/toolhead.py`).
pub trait ToolHeadApi: HomingTargetApi {
    /// `klippy/toolhead.py:397`
    /// Pauses the reactor.
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()>;

    /// `klippy/toolhead.py:414`
    /// Pauses the reactor.
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()>;

    /// `klippy/toolhead.py:424`
    /// Pauses the reactor.
    fn wait_moves(&self) -> KResult<()>;

    /// `klippy/toolhead.py:497` (`self.kin.get_status` may raise).
    fn get_status(&self, eventtime: f64) -> KResult<PyValue>;

    /// `klippy/toolhead.py:524`
    fn get_trapq(&self) -> Trapq;

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

    /// `klippy/toolhead.py:526` — runs `callback` immediately when no move is queued,
    /// returning its error.
    fn register_lookahead_callback(&self, callback: LookaheadCallback) -> KResult<()>;

    /// `klippy/toolhead.py:490`
    fn check_busy(&self, eventtime: f64) -> (f64, f64, bool);

    /// `klippy/toolhead.py:391`
    fn limit_next_junction_speed(&self, speed: f64);

    /// `klippy/toolhead.py:440`
    fn set_extruder(&self, extruder: PrinterExtruder, extrude_pos: f64);

    /// `klippy/toolhead.py:443`
    fn get_extruder(&self) -> PrinterExtruder;

    /// `klippy/toolhead.py:446`
    fn add_extra_axis(&self, ea: ExtraAxis, axis_pos: f64) -> KResult<()>;

    /// `klippy/toolhead.py:454` — identity comparison (`Arc::ptr_eq`).
    fn remove_extra_axis(&self, ea: &ExtraAxis) -> KResult<()>;

    /// `klippy/toolhead.py:462` — `[None, None, None] + extra_axes`.
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>>;

    /// `klippy/toolhead.py:494`
    fn stats(&self, eventtime: f64) -> (bool, String);

    /// The `stepper` module functions (`PrinterStepper`, `LookupRail`, ...). Not a Python
    /// method: kinematics and extras that `import stepper` reach the core factory here.
    fn stepper_factory(&self) -> StepperFactory;
}

/// The `ToolHead` service handle.
pub type ToolHead = Arc<dyn ToolHeadApi>;

/// A move under construction (`toolhead.py:14` `Move`), handed to
/// [`KinematicsApi::check_move`] and the extra axes; `limit_speed` mutates it in place.
/// The timing fields are valid once the lookahead has planned the move.
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
    fn is_kinematic_move(&self) -> bool;
    /// `klippy/toolhead.py:19`
    fn accel(&self) -> f64;
    /// `klippy/toolhead.py:19`
    fn max_cruise_v2(&self) -> f64;
    /// `klippy/toolhead.py:91` (set by `set_junction`)
    fn start_v(&self) -> f64;
    /// `klippy/toolhead.py:91`
    fn cruise_v(&self) -> f64;
    /// `klippy/toolhead.py:91`
    fn accel_t(&self) -> f64;
    /// `klippy/toolhead.py:91`
    fn cruise_t(&self) -> f64;
    /// `klippy/toolhead.py:91`
    fn decel_t(&self) -> f64;
    /// `klippy/toolhead.py:32`
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()>;
    /// `klippy/toolhead.py:64` (`msg` defaults to `"Move out of range"`).
    fn move_error(&self, msg: String) -> crate::KlippyError;
}

/// Delta calibration (`klippy/kinematics/delta.py` and `rotary_delta.py`
/// `*DeltaCalibration`), returned by `kin.get_calibration()`.
pub trait DeltaCalibrationApi: Send + Sync + 'static {
    /// `klippy/kinematics/delta.py:184` — `(adj_params, params)` as a Python tuple.
    fn coordinate_descent_params(&self, is_extended: bool) -> PyValue;
    /// `klippy/kinematics/delta.py:197` — `params` is the dict from
    /// `coordinate_descent_params`.
    fn new_calibration(&self, params: &PyValue) -> KResult<DeltaCalibration>;
    /// `klippy/kinematics/delta.py:205` (trilateration may raise).
    fn get_position_from_stable(&self, stable_position: &[f64]) -> KResult<Vec<f64>>;
    /// `klippy/kinematics/delta.py:212` (`math.sqrt` may raise).
    fn calc_stable_position(&self, coord: &[f64]) -> KResult<Vec<f64>>;
    /// `klippy/kinematics/delta.py:220` — writes `configfile` and responds on `gcode`.
    fn save_state(&self, configfile: &crate::config::PrinterConfig) -> KResult<()>;
}

/// Delta calibration handle.
pub type DeltaCalibration = Arc<dyn DeltaCalibrationApi>;

/// The kinematics interface returned by `ToolHead.get_kinematics` (implemented by every
/// `kinematics/*.py` module and by `manual_stepper`'s duck type). Methods a Python class
/// lacks keep the default, which raises the `AttributeError` Python would.
pub trait KinematicsApi: Send + Sync + 'static {
    /// `type(kin).__name__`, named by the `AttributeError` of a missing method.
    fn class_name(&self) -> &'static str;

    /// `klippy/kinematics/cartesian.py:46`
    fn get_steppers(&self) -> Vec<Stepper>;

    /// `klippy/kinematics/cartesian.py:48` — a missing stepper name is a `KeyError`.
    fn calc_position(&self, stepper_positions: &IndexMap<String, f64>)
        -> KResult<Vec<Option<f64>>>;

    /// `klippy/kinematics/cartesian.py:62`
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:106`
    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:90`
    fn home(&self, homing_state: HomingStateHandle) -> KResult<()>;

    /// `klippy/kinematics/cartesian.py:120` (`manual_stepper` has none).
    fn get_status(&self, _eventtime: f64) -> KResult<PyValue> {
        Err(no_kinematics_attr(self.class_name(), "get_status"))
    }

    /// `klippy/kinematics/cartesian.py:73` (`manual_stepper` has none).
    fn clear_homing_state(&self, _clear_axes: &str) -> KResult<()> {
        Err(no_kinematics_attr(self.class_name(), "clear_homing_state"))
    }

    /// `klippy/kinematics/cartesian.py:77` (idex_modes calls it on cartesian and
    /// hybrid kinematics only).
    fn home_axis(&self, _homing_state: HomingStateHandle, _axis: i64, _rail: Rail) -> KResult<()> {
        Err(no_kinematics_attr(self.class_name(), "home_axis"))
    }

    /// `klippy/kinematics/cartesian.py:68` `update_limits(i, range)` (idex_modes).
    fn update_limits(&self, _axis: i64, _range: (f64, f64)) -> KResult<()> {
        Err(no_kinematics_attr(self.class_name(), "update_limits"))
    }

    /// `klippy/kinematics/delta.py:160` — only the delta kinematics define it.
    fn get_calibration(&self) -> KResult<DeltaCalibration> {
        Err(no_kinematics_attr(self.class_name(), "get_calibration"))
    }
}

/// `AttributeError` for a method the Python class `class_name` does not define.
pub fn no_kinematics_attr(class_name: &str, name: &str) -> crate::KlippyError {
    crate::KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{}' object has no attribute '{}'", class_name, name),
    }
}

/// The kinematics service handle.
pub type Kinematics = Arc<dyn KinematicsApi>;

/// `PrinterMotionQueuing` (`klippy/extras/motion_queuing.py`).
pub trait MotionQueuingApi: Send + Sync + 'static {
    /// `klippy/extras/motion_queuing.py:133`
    fn allocate_trapq(&self) -> Trapq;

    /// `klippy/extras/motion_queuing.py:141`
    fn lookup_trapq_append(&self) -> TrapqAppend;

    /// `klippy/extras/motion_queuing.py:138`
    fn wipe_trapq(&self, trapq: &Trapq);

    /// `klippy/extras/motion_queuing.py:153` — an empty name is an `IndexError`.
    fn allocate_syncemitter(
        &self,
        mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle>;

    /// `syncemitter_queue_msg(se, req_clock, data, len)` (`klippy/chelper/steppersync.c:73`),
    /// which `stepper.py:210` and `pwm_tool.py:110` call through cffi on a syncemitter from
    /// [`allocate_syncemitter`](Self::allocate_syncemitter). `data` holds the `uint32_t`
    /// message words (cffi converts each Python int to `uint32_t`).
    fn queue_syncemitter_msg(&self, syncemitter: &SyncEmitterHandle, req_clock: u64, data: &[u32]);

    /// `klippy/extras/motion_queuing.py:163` (`motion_queuing.py:163-169`):
    /// the `serialqueue` argument is `mcu._serial.get_serialqueue()`
    /// (`mcu.py:1226`), resolved by the caller. `None` (no serial session
    /// yet) leaves the sync unconfigured, mirroring the C NULL dereference
    /// guard of the port (the reference would crash; the port queues).
    fn setup_mcu_movequeue(&self, mcu: Mcu, serialqueue: Option<std::sync::Arc<SerialQueue>>, move_count: i64);

    /// `klippy/extras/motion_queuing.py:182`
    fn register_flush_callback(&self, callback: FlushCallback, can_add_trapq: bool);
    /// `register_flush_callback` with the resolved `debug_callback_name`
    /// inputs (`motion_queuing.py:31`): `name` is the `__qualname__` /
    /// `__name__` / type-name equivalent of the callback object (`None` for
    /// the `cb is None` case) and `obj_name` the `__self__._name` when the
    /// callback is a bound method. `None` name yields `"-"`.
    /// Production callers pass the same name Python would resolve
    /// (`ToolHead._handle_step_flush` reports `_handle_step_flush`,
    /// `MCU_queued_pwm._flush_notification` reports
    /// `_flush_notification`, `GCodeRequestQueue._flush_notification`
    /// reports `_flush_notification`); closures without one pass `None`.
    /// The default keeps `register_flush_callback` behaviour (name `None`).
    fn register_flush_callback_named(
        &self,
        callback: FlushCallback,
        can_add_trapq: bool,
        name: Option<&str>,
        obj_name: Option<&str>,
    ) {
        let _ = obj_name;
        self.register_flush_callback(callback, can_add_trapq);
        let _ = name;
    }
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
        drip_completion: &Completion<bool>,
    ) -> KResult<()>;

    /// `klippy/extras/motion_queuing.py:503`
    fn check_drip_timing(&self) -> Option<f64>;
}

/// The `motion_queuing` service handle.
pub type MotionQueuing = Arc<dyn MotionQueuingApi>;
