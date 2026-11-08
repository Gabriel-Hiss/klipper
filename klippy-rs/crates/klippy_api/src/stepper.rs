//! Contract of the `stepper` service.
//! Port of `klippy/stepper.py` (`MCU_stepper`, `GenericPrinterRail`)
//! and the `PrinterStepper` / `LookupRail` / `LookupMultiRail` /
//! `parse_step_distance` helpers externals call.
//!
//! Handles to steppers and rails are `Arc<dyn ...Api>`. State lives in
//! `GCell`s inside implementations; methods take `&self`.
//!
//! Opaque chelper handles (`stepper_kinematics`, `trapq`) are traits
//! ([`StepperKinematicsApi`], [`TrapqApi`]) whose only method is `as_any`: the
//! owning crate (`chelper_motion` users in core and `motion_queuing`) downcasts
//! back to its concrete type; everyone else passes the handle along. Identity
//! (`trapq is other`) is `Arc::ptr_eq`.

use std::any::Any;
use std::sync::Arc;

use pycompat::PyValue;

use crate::config::Config;
use crate::error::KResult;
use crate::mcu::{EndstopApi, Mcu};

/// One entry of [`McuStepperApi::dump_steps`].
///
/// `struct pull_history_steps` (`klippy/stepper.py:185`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PullHistoryStep {
    /// `klippy/stepper.py:185` `first_clock`.
    pub first_clock: i64,
    /// `klippy/stepper.py:185` `last_clock`.
    pub last_clock: i64,
    /// `klippy/stepper.py:185` `start_position`.
    pub start_position: i64,
    /// `klippy/stepper.py:185` `interval`.
    pub interval: i64,
    /// `klippy/stepper.py:185` `step_count`.
    pub step_count: i64,
    /// `klippy/stepper.py:185` `add`.
    pub add: i64,
}

/// Opaque chelper stepper-kinematics handle (`stepper.py:191`).
pub trait StepperKinematicsApi: Send + Sync + 'static {
    /// Upcast for the owning crate's downcast.
    fn as_any(&self) -> &dyn Any;
}

/// Opaque chelper stepper-kinematics handle.
pub type StepperKinematics = Arc<dyn StepperKinematicsApi>;

/// Opaque chelper trapq handle (`stepper.py:228`, `toolhead.get_trapq()`,
/// `motion_queuing.allocate_trapq()` all hand out the same type).
pub trait TrapqApi: Send + Sync + 'static {
    /// Upcast for the owning crate's downcast.
    fn as_any(&self) -> &dyn Any;
}

/// Opaque chelper trapq handle.
pub type Trapq = Arc<dyn TrapqApi>;

/// Shared `MCU_stepper` handle.
pub type Stepper = Arc<dyn McuStepperApi>;

/// Shared `GenericPrinterRail` handle.
pub type Rail = Arc<dyn RailApi>;

/// Port of `MCU_stepper` (`klippy/stepper.py:22`).
pub trait McuStepperApi: Send + Sync + 'static {
    /// `klippy/stepper.py:58`
    fn get_mcu(&self) -> Mcu;
    /// `klippy/stepper.py:60`
    fn get_name(&self, short: Option<bool>) -> String;
    /// `klippy/stepper.py:132`
    fn get_oid(&self) -> i64;
    /// `klippy/stepper.py:65`
    fn units_in_radians(&self) -> bool;
    /// `klippy/stepper.py:68`
    fn get_pulse_duration(&self) -> (Option<f64>, bool);
    /// `klippy/stepper.py:70`
    fn setup_default_pulse_duration(&self, pulse_duration: f64, step_both_edge: bool);
    /// `klippy/stepper.py:74`
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]);
    /// `klippy/stepper.py:134`
    fn get_step_dist(&self) -> f64;
    /// `klippy/stepper.py:135`
    fn get_rotation_distance(&self) -> (f64, f64);
    /// `klippy/stepper.py:138`
    fn set_rotation_distance(&self, rotation_dist: f64);
    /// `klippy/stepper.py:144`
    fn get_dir_inverted(&self) -> (bool, bool);
    /// `klippy/stepper.py:146`
    fn set_dir_inverted(&self, invert_dir: bool) -> KResult<()>;
    /// `klippy/stepper.py:154`
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64;
    /// `klippy/stepper.py:158`
    fn set_position(&self, coord: [f64; 3]);
    /// `klippy/stepper.py:164`
    fn get_commanded_position(&self) -> f64;
    /// `klippy/stepper.py:167`
    fn get_mcu_position(&self, cmd_pos: Option<f64>) -> i64;
    /// `klippy/stepper.py:178`
    fn get_past_mcu_position(&self, print_time: f64) -> i64;
    /// `klippy/stepper.py:183`
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64;
    /// `klippy/stepper.py:185`
    fn dump_steps(&self, count: i64, start_clock: i64, end_clock: i64) -> (Vec<PullHistoryStep>, i64);
    /// `klippy/stepper.py:191`
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics>;
    /// `klippy/stepper.py:193`
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics>;
    /// `klippy/stepper.py:204`
    /// Pauses the reactor.
    fn note_homing_end(&self) -> KResult<()>;
    /// `klippy/stepper.py:228`
    fn get_trapq(&self) -> Option<Trapq>;
    /// `klippy/stepper.py:230`
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq>;
    /// `klippy/stepper.py:239`
    fn add_active_callback(&self, cb: Box<dyn FnMut(f64) + Send>) -> KResult<()>;
    /// `klippy/stepper.py:260`
    fn is_active_axis(&self, axis: char) -> bool;
}

/// Port of the `homing_info` namedtuple (`klippy/stepper.py:398`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HomingInfo {
    /// `klippy/stepper.py:398` `speed`.
    pub speed: f64,
    /// `klippy/stepper.py:398` `position_endstop`.
    pub position_endstop: f64,
    /// `klippy/stepper.py:398` `retract_speed`.
    pub retract_speed: f64,
    /// `klippy/stepper.py:398` `retract_dist`.
    pub retract_dist: f64,
    /// `klippy/stepper.py:398` `positive_dir`.
    pub positive_dir: bool,
    /// `klippy/stepper.py:398` `second_homing_speed`.
    pub second_homing_speed: f64,
}

/// Keyword arguments of the rail factories (`GenericPrinterRail`,
/// `LookupRail`, `LookupMultiRail`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RailOpts {
    /// `klippy/stepper.py:328` `need_position_minmax`.
    pub need_position_minmax: bool,
    /// `klippy/stepper.py:328` `default_position_endstop`.
    pub default_position_endstop: Option<f64>,
    /// `klippy/stepper.py:328` `units_in_radians`.
    pub units_in_radians: bool,
}

impl Default for RailOpts {
    /// Defaults of `klippy/stepper.py:328`.
    fn default() -> RailOpts {
        RailOpts {
            need_position_minmax: true,
            default_position_endstop: None,
            units_in_radians: false,
        }
    }
}

/// Port of `GenericPrinterRail` (`klippy/stepper.py:327`).
pub trait RailApi: Send + Sync + 'static {
    /// `klippy/stepper.py:388`
    fn get_name(&self, short: Option<bool>) -> String;
    /// `klippy/stepper.py:395`
    fn get_range(&self) -> (f64, f64);
    /// `klippy/stepper.py:397`
    fn get_homing_info(&self) -> HomingInfo;
    /// `klippy/stepper.py:405`
    fn get_steppers(&self) -> Vec<Stepper>;
    /// `klippy/stepper.py:407`
    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)>;
    /// `klippy/stepper.py:409`
    fn lookup_endstop(&self, endstop_pin: Option<&str>, name: &str) -> KResult<Arc<dyn EndstopApi>>;
    /// `klippy/stepper.py:434`
    fn add_stepper(&self, stepper: Stepper, endstop_pin: Option<&str>, endstop_name: Option<&str>) -> KResult<()>;
    /// `klippy/stepper.py:445`
    fn add_stepper_from_config(&self, config: &Config) -> KResult<()>;
    /// `klippy/stepper.py:448`
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]);
    /// `klippy/stepper.py:451`
    fn set_trapq(&self, tq: Option<Trapq>);
    /// `klippy/stepper.py:454`
    fn set_position(&self, coord: [f64; 3]);
    /// `klippy/stepper.py:436`
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64;
    /// `klippy/stepper.py:435`
    fn get_commanded_position(&self) -> f64;
}

/// Stepper/rail factory helpers (`klippy/stepper.py:266-476`), the module
/// functions Python code reaches with `import stepper`. The core implements
/// it; consumers get it from `ToolHeadApi::stepper_factory()` (fakes in tests).
///
/// `PrinterStepper`, `LookupRail`, `LookupMultiRail` and
/// `GenericPrinterRail` are the only externally used constructors;
/// `parse_step_distance` is the only externally used parse helper.
/// `parse_gear_ratio` (`klippy/stepper.py:287`) has no external caller
/// and is not exposed. Constructors read config and raise its errors.
pub trait StepperFactoryApi: Send + Sync + 'static {
    /// `klippy/stepper.py:266`
    fn printer_stepper(&self, config: &Config, units_in_radians: bool) -> KResult<Stepper>;
    /// `klippy/stepper.py:458`
    fn lookup_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail>;
    /// `klippy/stepper.py:466`
    fn lookup_multi_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail>;
    /// `klippy/stepper.py:328`
    fn generic_rail(&self, config: &Config, opts: RailOpts) -> KResult<Rail>;
    /// `klippy/stepper.py:297` — `(rotation_dist, steps_per_rotation)`.
    fn parse_step_distance(
        &self,
        config: &Config,
        units_in_radians: Option<bool>,
        note_valid: bool,
    ) -> KResult<(f64, f64)>;
}

/// The stepper factory handle.
pub type StepperFactory = Arc<dyn StepperFactoryApi>;
