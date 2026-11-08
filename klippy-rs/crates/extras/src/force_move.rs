//! Port of the `force_move` module (`klippy/extras/force_move.py`).
//!
//! Utility for manually moving a stepper for diagnostic purposes: `STEPPER_BUZZ`
//! oscillates a stepper, `FORCE_MOVE` performs a single manual move, and
//! `SET_KINEMATIC_POSITION` forces a low-level kinematic position.

use crate::stepper_enable::PrinterStepperEnableApi;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::{Stepper, StepperKinematics, StepperRegistryApi, Trapq};
use klippy_api::toolhead::{MotionQueuingApi, ToolHeadApi, TrapqAppend};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef};
use klippy_core::motion_handles::alloc_stepper_kinematics;
use pycompat::text::{float_pow, float_true_div, percent_format, PctValue, PyError};
use pycompat::PyValue;
use std::sync::{Arc, Weak};

/// `BUZZ_DISTANCE` (`klippy/extras/force_move.py:9`).
pub const BUZZ_DISTANCE: f64 = 1.0;

/// `BUZZ_VELOCITY` (`klippy/extras/force_move.py:10`).
pub const BUZZ_VELOCITY: f64 = BUZZ_DISTANCE / 0.250;

/// `BUZZ_RADIANS_DISTANCE` (`klippy/extras/force_move.py:11`):
/// `math.radians(1.)`, i.e. `1.0 * (pi / 180.)` as CPython `math.radians` computes it.
pub const BUZZ_RADIANS_DISTANCE: f64 = std::f64::consts::PI / 180.0;

/// `BUZZ_RADIANS_VELOCITY` (`klippy/extras/force_move.py:12`).
pub const BUZZ_RADIANS_VELOCITY: f64 = BUZZ_RADIANS_DISTANCE / 0.250;

/// `cmd_STEPPER_BUZZ_help` (`klippy/extras/force_move.py:92`).
pub const CMD_STEPPER_BUZZ_HELP: &str = "Oscillate a given stepper to help id it";

/// `cmd_FORCE_MOVE_help` (`klippy/extras/force_move.py:107`).
pub const CMD_FORCE_MOVE_HELP: &str = "Manually move a stepper; invalidates kinematics";

/// `cmd_SET_KINEMATIC_POSITION_help` (`klippy/extras/force_move.py:117`).
pub const CMD_SET_KINEMATIC_POSITION_HELP: &str = "Force a low-level kinematic position";

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

/// `calc_move_time(dist, speed, accel)` (`klippy/extras/force_move.py:14-28`).
/// Returns `(axis_r, accel_t, cruise_t, cruise_v)`. A Python division by a zero
/// speed raises `ZeroDivisionError: float division by zero`; it is reproduced
/// via [`float_true_div`].
pub fn calc_move_time(dist: f64, speed: f64, accel: f64) -> KResult<(f64, f64, f64, f64)> {
    let mut axis_r = 1.0;
    let mut dist = dist;
    let mut speed = speed;
    if dist < 0.0 {
        axis_r = -1.0;
        dist = -dist;
    }
    if accel == 0.0 || dist == 0.0 {
        let cruise_t = float_true_div(dist, speed).map_err(map_py_error)?;
        return Ok((axis_r, 0.0, cruise_t, speed));
    }
    let max_cruise_v2 = dist * accel;
    if max_cruise_v2 < float_pow(speed, 2.0).map_err(map_py_error)? {
        if max_cruise_v2 < 0.0 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "math domain error".to_string(),
            });
        }
        speed = kos::libm::sqrt(max_cruise_v2);
    }
    let accel_t = float_true_div(speed, accel).map_err(map_py_error)?;
    let accel_decel_d = accel_t * speed;
    let cruise_t = float_true_div(dist - accel_decel_d, speed).map_err(map_py_error)?;
    Ok((axis_r, accel_t, cruise_t, speed))
}

/// `fmt % args` for a constant module format string (the `logging` call sites).
fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant force_move format")
}

/// `gcmd.get(name, default)` with the default `str` parser, as a `String`.
fn str_param(gcmd: &GCmd, name: &str, opts: GetOpts) -> KResult<String> {
    Ok(gcmd.get(name, opts)?.str())
}

/// `gcmd.get_float(name, ...)` for a required float: the Rust getter raises on
/// a missing parameter already, so `None` is unreachable; the fallback repeats
/// the message Python raises for a missing parameter (`gcode.py:70`).
fn required_float(gcmd: &GCmd, name: &str, opts: GetFloatOpts) -> KResult<f64> {
    match gcmd.get_float(name, opts)? {
        Some(v) => Ok(v),
        None => Err(KlippyError::command(format!(
            "Error on '{}': missing {}",
            gcmd.get_commandline(),
            name
        ))),
    }
}

/// `gcmd.get_float(name, default)` with a numeric default: `None` is
/// unreachable (the default always yields a value), and the fallback is that
/// same default.
fn default_float(gcmd: &GCmd, name: &str, default: f64) -> KResult<f64> {
    let v = gcmd.get_float(
        name,
        GetFloatOpts { default: Some(Some(default)), ..Default::default() },
    )?;
    Ok(v.unwrap_or(default))
}

/// `"".join([a for a in "xyz" if a in homed])` (`klippy/extras/force_move.py:126,132`).
fn filter_axes(homed: &str) -> String {
    let mut axes = String::new();
    for ax in ['x', 'y', 'z'] {
        if homed.contains(ax) {
            axes.push(ax);
        }
    }
    axes
}

/// `klippy/extras/force_move.py:30` `ForceMove`.
pub struct ForceMove {
    printer: Printer,
    steppers: GCell<IndexMap<String, Stepper>>,
    motion_queuing: Arc<dyn MotionQueuingApi>,
    trapq: Trapq,
    trapq_append: TrapqAppend,
    stepper_kinematics: StepperKinematics,
    enable_force_move: bool,
    self_ref: GCell<Option<Weak<ForceMove>>>,
}

impl ForceMove {
    /// `ForceMove.__init__(config)` (`klippy/extras/force_move.py:31-47`).
    pub fn new(config: &Config) -> KResult<Arc<ForceMove>> {
        let printer = config.get_printer();
        let motion_queuing =
            load_interface::<dyn MotionQueuingApi>(&*printer, config, "motion_queuing", LookupDefault::Raise)?
                .expect("motion_queuing load with Raise never yields None");
        let trapq = motion_queuing.allocate_trapq();
        let trapq_append = motion_queuing.lookup_trapq_append();
        let stepper_kinematics = alloc_stepper_kinematics(
            "cartesian_stepper_alloc",
            &[PyValue::Bytes(b"x".to_vec())],
        )?;
        let enable_force_move = config
            .getboolean(
                "enable_force_move",
                GetBoolOpts { default: Some(Some(false)), ..Default::default() },
            )?
            .unwrap_or(false);
        let this = Arc::new(ForceMove {
            printer,
            steppers: GCell::new(IndexMap::new()),
            motion_queuing,
            trapq,
            trapq_append,
            stepper_kinematics,
            enable_force_move,
            self_ref: GCell::new(None),
        });
        *this.self_ref.borrow_mut() = Some(Arc::downgrade(&this));
        if enable_force_move {
            let gcode = lookup_interface::<dyn GCodeApi>(&*this.printer, "gcode", LookupDefault::Raise)?
                .expect("gcode lookup with Raise never yields None");
            let handler_self = Arc::clone(&this);
            gcode.register_command(
                "SET_KINEMATIC_POSITION",
                Some(Box::new(move |gcmd: &GCmd| handler_self.cmd_set_kinematic_position(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_SET_KINEMATIC_POSITION_HELP.to_string()),
                },
            )?;
        }
        Ok(this)
    }

    /// `stepper_kinematics` attribute (`klippy/extras/force_move.py:39`).
    pub fn stepper_kinematics(&self) -> StepperKinematics {
        self.stepper_kinematics.clone()
    }

    /// `trapq` attribute (`klippy/extras/force_move.py:36`).
    pub fn trapq(&self) -> Trapq {
        self.trapq.clone()
    }

    /// `_enable_force_move` attribute (`klippy/extras/force_move.py:42`).
    pub fn enable_force_move(&self) -> bool {
        self.enable_force_move
    }

    /// Registered stepper names, in registration order (`steppers` keys).
    pub fn stepper_names(&self) -> Vec<String> {
        self.steppers.borrow().keys().cloned().collect()
    }

    /// `register_stepper(config, mcu_stepper)` (`klippy/extras/force_move.py:48-59`).
    pub fn register_stepper(self: &Arc<Self>, config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        Self::register_stepper_impl(self, config, mcu_stepper)
    }

    fn register_stepper_impl(this: &Arc<Self>, _config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        let name = mcu_stepper.get_name(None);
        this.steppers.borrow_mut().insert(name.clone(), mcu_stepper);
        let gcode = lookup_interface::<dyn GCodeApi>(&*this.printer, "gcode", LookupDefault::Raise)?
            .expect("gcode lookup with Raise never yields None");
        let buzz_self = Arc::clone(this);
        gcode.register_mux_command(
            "STEPPER_BUZZ",
            "STEPPER",
            Some(&name),
            Box::new(move |gcmd: &GCmd| buzz_self.cmd_stepper_buzz(gcmd)),
            Some(CMD_STEPPER_BUZZ_HELP),
        )?;
        if this.enable_force_move {
            let move_self = Arc::clone(this);
            gcode.register_mux_command(
                "FORCE_MOVE",
                "STEPPER",
                Some(&name),
                Box::new(move |gcmd: &GCmd| move_self.cmd_force_move(gcmd)),
                Some(CMD_FORCE_MOVE_HELP),
            )?;
        }
        Ok(())
    }

    /// `lookup_stepper(name)` (`klippy/extras/force_move.py:60-63`).
    pub fn lookup_stepper(&self, name: &str) -> KResult<Stepper> {
        match self.steppers.borrow().get(name) {
            Some(stepper) => Ok(stepper.clone()),
            None => Err(KlippyError::config(pct_format(
                "Unknown stepper %s",
                &[PctValue::Str(name.to_string())],
            ))),
        }
    }

    fn stepper_enable(&self) -> KResult<Arc<dyn PrinterStepperEnableApi>> {
        lookup_interface::<dyn PrinterStepperEnableApi>(&*self.printer, "stepper_enable", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "stepper_enable lookup with Raise never yields None".to_string(),
            })
    }

    fn toolhead(&self) -> KResult<Arc<dyn ToolHeadApi>> {
        lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "toolhead lookup with Raise never yields None".to_string(),
            })
    }

    /// `_force_enable(stepper)` (`klippy/extras/force_move.py:64-68`).
    fn force_enable(&self, stepper: &Stepper) -> KResult<bool> {
        let stepper_name = stepper.get_name(None);
        let stepper_enable = self.stepper_enable()?;
        stepper_enable.set_motors_enable(&[stepper_name], true)
    }

    /// `_restore_enable(stepper, did_enable)` (`klippy/extras/force_move.py:69-74`).
    fn restore_enable(&self, stepper: &Stepper, did_enable: bool) -> KResult<()> {
        if !did_enable {
            return Ok(());
        }
        let stepper_name = stepper.get_name(None);
        let stepper_enable = self.stepper_enable()?;
        stepper_enable.set_motors_enable(&[stepper_name], false)?;
        Ok(())
    }

    /// `manual_move(stepper, dist, speed, accel=0.)` (`klippy/extras/force_move.py:75-91`).
    pub fn manual_move(&self, stepper: &Stepper, dist: f64, speed: f64, accel: f64) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let prev_sk = stepper.set_stepper_kinematics(Some(self.stepper_kinematics.clone()));
        let prev_trapq = stepper.set_trapq(Some(self.trapq.clone()));
        stepper.set_position([0.0, 0.0, 0.0]);
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel)?;
        let print_time = toolhead.get_last_move_time();
        (*self.trapq_append)(
            &self.trapq,
            print_time,
            accel_t,
            cruise_t,
            accel_t,
            0.0,
            0.0,
            0.0,
            axis_r,
            0.0,
            0.0,
            0.0,
            cruise_v,
            accel,
        );
        let print_time = print_time + accel_t + cruise_t + accel_t;
        self.motion_queuing.note_mcu_movequeue_activity(print_time, true);
        toolhead.dwell(accel_t + cruise_t + accel_t);
        toolhead.flush_step_generation()?;
        stepper.set_trapq(prev_trapq);
        stepper.set_stepper_kinematics(prev_sk);
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }

    /// `cmd_STEPPER_BUZZ(gcmd)` (`klippy/extras/force_move.py:92-106`).
    pub fn cmd_stepper_buzz(&self, gcmd: &GCmd) -> KResult<()> {
        let stepper = self.lookup_stepper(&str_param(gcmd, "STEPPER", GetOpts::default())?)?;
        let msg = pct_format("Stepper buzz %s", &[PctValue::Str(stepper.get_name(None))]);
        log::info(&msg);
        let did_enable = self.force_enable(&stepper)?;
        let toolhead = self.toolhead()?;
        let (dist, speed) = if stepper.units_in_radians() {
            (BUZZ_RADIANS_DISTANCE, BUZZ_RADIANS_VELOCITY)
        } else {
            (BUZZ_DISTANCE, BUZZ_VELOCITY)
        };
        for _ in 0..10 {
            self.manual_move(&stepper, dist, speed, 0.0)?;
            toolhead.dwell(0.050);
            self.manual_move(&stepper, -dist, speed, 0.0)?;
            toolhead.dwell(0.450);
        }
        self.restore_enable(&stepper, did_enable)
    }

    /// `cmd_FORCE_MOVE(gcmd)` (`klippy/extras/force_move.py:107-116`).
    pub fn cmd_force_move(&self, gcmd: &GCmd) -> KResult<()> {
        let stepper = self.lookup_stepper(&str_param(gcmd, "STEPPER", GetOpts::default())?)?;
        let distance = required_float(gcmd, "DISTANCE", GetFloatOpts::default())?;
        let speed = required_float(
            gcmd,
            "VELOCITY",
            GetFloatOpts { above: Some(0.0), ..Default::default() },
        )?;
        let accel = gcmd
            .get_float(
                "ACCEL",
                GetFloatOpts { default: Some(Some(0.0)), minval: Some(0.0), ..Default::default() },
            )?
            .unwrap_or(0.0);
        let msg = pct_format(
            "FORCE_MOVE %s distance=%.3f velocity=%.3f accel=%.3f",
            &[
                PctValue::Str(stepper.get_name(None)),
                PctValue::Float(distance),
                PctValue::Float(speed),
                PctValue::Float(accel),
            ],
        );
        log::info(&msg);
        self.force_enable(&stepper)?;
        self.manual_move(&stepper, distance, speed, accel)
    }

    /// `cmd_SET_KINEMATIC_POSITION(gcmd)` (`klippy/extras/force_move.py:117-137`).
    pub fn cmd_set_kinematic_position(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.get_last_move_time();
        let curpos = toolhead.get_position();
        let x = default_float(gcmd, "X", curpos[0])?;
        let y = default_float(gcmd, "Y", curpos[1])?;
        let z = default_float(gcmd, "Z", curpos[2])?;
        let set_homed = str_param(
            gcmd,
            "SET_HOMED",
            GetOpts { default: Some(PyValue::Str("xyz".to_string())), ..Default::default() },
        )?
        .to_lowercase();
        let set_homed_axes = filter_axes(&set_homed);
        let clear_homed = {
            let clear_homed_opt = gcmd.get(
                "CLEAR_HOMED",
                GetOpts { default: Some(PyValue::None), ..Default::default() },
            )?;
            if matches!(clear_homed_opt, PyValue::None) {
                // "CLEAR" is an alias for "CLEAR_HOMED"; should deprecate
                str_param(
                    gcmd,
                    "CLEAR",
                    GetOpts { default: Some(PyValue::Str(String::new())), ..Default::default() },
                )?
                .to_lowercase()
            } else {
                clear_homed_opt.str().to_lowercase()
            }
        };
        let clear_homed_axes = filter_axes(&clear_homed);
        let msg = pct_format(
            "SET_KINEMATIC_POSITION pos=%.3f,%.3f,%.3f set_homed=%s clear_homed=%s",
            &[
                PctValue::Float(x),
                PctValue::Float(y),
                PctValue::Float(z),
                PctValue::Str(set_homed_axes.clone()),
                PctValue::Str(clear_homed_axes.clone()),
            ],
        );
        log::info(&msg);
        toolhead.set_position(vec![x, y, z], &set_homed_axes)?;
        toolhead.get_kinematics().clear_homing_state(&clear_homed_axes);
        Ok(())
    }
}

impl StepperRegistryApi for ForceMove {
    fn register_stepper(&self, config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        let this = self
            .self_ref
            .borrow()
            .as_ref()
            .and_then(|weak| weak.upgrade())
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "ForceMove self reference missing".to_string(),
            })?;
        Self::register_stepper_impl(&this, config, mcu_stepper)
    }
}

klippy_api::printer_object!(ForceMove; dyn StepperRegistryApi);

/// `load_config(config)` (`klippy/extras/force_move.py:139-140`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ForceMove::new(config)? as ObjectRef))
}

/// Module entry point for the `force_move` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
