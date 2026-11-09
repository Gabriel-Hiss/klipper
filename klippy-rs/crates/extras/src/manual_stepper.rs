//! Port of the `manual_stepper` module (`klippy/extras/manual_stepper.py`).

use crate::force_move::calc_move_time as force_calc_move_time;
use crate::stepper_enable::PrinterStepperEnableApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, DeprecateGcodeOpts, GetFloatOpts as ConfigGetFloatOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeMoveApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts,
    GetOpts as GGetOpts, GCmd,
};
use klippy_api::mcu::Endstop;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::{Rail, RailOpts, Stepper, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, HomingTarget, HomingTargetApi, Kinematics, KinematicsApi,
    MotionQueuingApi, MoveApi, PrinterHomingApi, ToolHead, ToolHeadApi, TrapqAppend,
};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef};
use pycompat::text::{float_true_div, PyError};
use pycompat::PyValue;
use reactor::Completion;
use std::sync::{Arc, Weak};

/// Help string (`klippy/extras/manual_stepper.py:93`).
pub const CMD_MANUAL_STEPPER_HELP: &str = "Command a manually configured stepper";

/// Port of `force_move.calc_move_time` (`klippy/extras/force_move.py:15`).
/// Delegates to the `force_move` port so the Python error semantics
/// (zero division, math domain) are shared.
pub fn calc_move_time(dist: f64, speed: f64, accel: f64) -> KResult<(f64, f64, f64, f64)> {
    force_calc_move_time(dist, speed, accel)
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

/// The rail object: a `GenericPrinterRail` when the stepper can home,
/// otherwise the single `MCU_stepper` from `PrinterStepper`.
enum RailOrStepper {
    Rail(Rail),
    Stepper(Stepper),
}

impl RailOrStepper {
    fn get_name(&self, short: Option<bool>) -> String {
        match self {
            RailOrStepper::Rail(rail) => rail.get_name(short),
            RailOrStepper::Stepper(stepper) => stepper.get_name(short),
        }
    }
    fn set_position(&self, coord: [f64; 3]) {
        match self {
            RailOrStepper::Rail(rail) => rail.set_position(coord),
            RailOrStepper::Stepper(stepper) => stepper.set_position(coord),
        }
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        match self {
            RailOrStepper::Rail(rail) => rail.setup_itersolve(alloc_func, params),
            RailOrStepper::Stepper(stepper) => stepper.setup_itersolve(alloc_func, params),
        }
    }
    fn set_trapq(&self, trapq: Trapq) {
        let tq = Some(Arc::clone(&trapq));
        match self {
            RailOrStepper::Rail(rail) => rail.set_trapq(tq),
            RailOrStepper::Stepper(stepper) => { let _ = stepper.set_trapq(tq); }
        }
    }
    fn get_endstops(&self) -> Vec<(Endstop, String)> {
        match self {
            RailOrStepper::Rail(rail) => rail.get_endstops(),
            RailOrStepper::Stepper(_) => Vec::new(),
        }
    }
}

struct State {
    velocity: f64,
    accel: f64,
    homing_accel: f64,
    next_cmd_time: f64,
    commanded_pos: f64,
    pos_min: Option<f64>,
    pos_max: Option<f64>,
    can_home: bool,
    axis_gcode_id: Option<String>,
    instant_corner_v: f64,
    gaxis_limit_velocity: f64,
    gaxis_limit_accel: f64,
}

/// Port of `ManualStepper` (`klippy/extras/manual_stepper.py:10`).
pub struct ManualStepper {
    printer: Printer,
    name: String,
    rail: RailOrStepper,
    steppers: Vec<Stepper>,
    motion_queuing: Arc<dyn MotionQueuingApi>,
    trapq: Trapq,
    trapq_append: TrapqAppend,
    state: GCell<State>,
    self_ref: GCell<Option<Weak<ManualStepper>>>,
}

klippy_api::printer_object!(ManualStepper; dyn ExtraAxisApi, dyn HomingTargetApi, dyn KinematicsApi);

impl ManualStepper {
    /// Port of `ManualStepper.__init__` (`klippy/extras/manual_stepper.py:11`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let can_home = config
            .get(
                "endstop_pin",
                GetStrOpts { default: Some(None), note_valid: true },
            )?
            .is_some();
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*printer, "toolhead", LookupDefault::Raise)?
            .expect("toolhead lookup with Raise never yields None");
        let factory = toolhead.stepper_factory();
        let (rail, steppers): (RailOrStepper, Vec<Stepper>) = if can_home {
            let rail = factory.lookup_rail(
                config,
                RailOpts {
                    need_position_minmax: false,
                    default_position_endstop: Some(0.0),
                    units_in_radians: false,
                },
            )?;
            let steppers = rail.get_steppers();
            (RailOrStepper::Rail(rail), steppers)
        } else {
            let stepper = factory.printer_stepper(config, false)?;
            (RailOrStepper::Stepper(Arc::clone(&stepper)), vec![stepper])
        };
        let velocity = config
            .getfloat(
                "velocity",
                ConfigGetFloatOpts {
                    default: Some(Some(5.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let accel = config
            .getfloat(
                "accel",
                ConfigGetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let pos_min = config.getfloat(
            "position_min",
            ConfigGetFloatOpts { default: Some(None), ..Default::default() },
        )?;
        let pos_max = config.getfloat(
            "position_max",
            ConfigGetFloatOpts { default: Some(None), ..Default::default() },
        )?;
        let motion_queuing =
            load_interface::<dyn MotionQueuingApi>(&*printer, config, "motion_queuing", LookupDefault::Raise)?
                .expect("motion_queuing load with Raise never yields None");
        let trapq = motion_queuing.allocate_trapq();
        let trapq_append = motion_queuing.lookup_trapq_append();
        rail.setup_itersolve("cartesian_stepper_alloc", &[PyValue::Bytes(b"x".to_vec())]);
        rail.set_trapq(Arc::clone(&trapq));
        let stepper_name = match name.split_whitespace().nth(1) {
            Some(s) => s.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                });
            }
        };
        let this = Arc::new(ManualStepper {
            printer: Arc::clone(&printer),
            name: name.clone(),
            rail,
            steppers,
            motion_queuing,
            trapq,
            trapq_append,
            state: GCell::new(State {
                velocity,
                accel,
                homing_accel: accel,
                next_cmd_time: 0.0,
                commanded_pos: 0.0,
                pos_min,
                pos_max,
                can_home,
                axis_gcode_id: None,
                instant_corner_v: 0.0,
                gaxis_limit_velocity: 0.0,
                gaxis_limit_accel: 0.0,
            }),
            self_ref: GCell::new(None),
        });
        *this.self_ref.borrow_mut() = Some(Arc::downgrade(&this));
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("gcode lookup with Raise never yields None");
        let handler_self = Arc::clone(&this);
        gcode.register_mux_command(
            "MANUAL_STEPPER",
            "STEPPER",
            Some(&stepper_name),
            Box::new(move |gcmd: &GCmd| handler_self.cmd_manual_stepper(gcmd)),
            Some(CMD_MANUAL_STEPPER_HELP),
        )?;
        Ok(this)
    }

    fn arc_self(&self) -> Arc<Self> {
        let weak = self.self_ref.borrow().clone().expect("ManualStepper self_ref set");
        weak.upgrade().expect("ManualStepper alive while referenced")
    }

    fn toolhead(&self) -> KResult<ToolHead> {
        lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "toolhead lookup with Raise never yields None".to_string(),
            })
    }

    fn stepper_enable(&self) -> KResult<Arc<dyn PrinterStepperEnableApi>> {
        lookup_interface::<dyn PrinterStepperEnableApi>(&*self.printer, "stepper_enable", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "stepper_enable lookup with Raise never yields None".to_string(),
            })
    }

    fn homing(&self) -> KResult<Arc<dyn PrinterHomingApi>> {
        lookup_interface::<dyn PrinterHomingApi>(&*self.printer, "homing", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "homing lookup with Raise never yields None".to_string(),
            })
    }

    fn configfile(&self) -> KResult<Arc<dyn PrinterConfigApi>> {
        lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::Internal {
                class_name: "AssertionError".to_string(),
                message: "configfile lookup with Raise never yields None".to_string(),
            })
    }

    /// `get_name`.
    pub fn get_name(&self) -> String {
        self.name.clone()
    }

    /// Current commanded position (test observable).
    pub fn commanded_pos(&self) -> f64 {
        self.state.borrow().commanded_pos
    }

    /// Current next command time (test observable).
    pub fn next_cmd_time(&self) -> f64 {
        self.state.borrow().next_cmd_time
    }

    /// Configured velocity (test observable).
    pub fn velocity(&self) -> f64 {
        self.state.borrow().velocity
    }

    /// Configured accel (test observable).
    pub fn accel(&self) -> f64 {
        self.state.borrow().accel
    }

    /// Whether the stepper can home (test observable).
    pub fn can_home(&self) -> bool {
        self.state.borrow().can_home
    }

    /// Position minimum (test observable).
    pub fn pos_min(&self) -> Option<f64> {
        self.state.borrow().pos_min
    }

    /// Position maximum (test observable).
    pub fn pos_max(&self) -> Option<f64> {
        self.state.borrow().pos_max
    }

    /// `get_axis_gcode_id` (`klippy/extras/manual_stepper.py:198`).
    pub fn get_axis_gcode_id(&self) -> Option<String> {
        self.state.borrow().axis_gcode_id.clone()
    }

    /// `get_trapq` (`klippy/extras/manual_stepper.py:200`).
    pub fn get_trapq(&self) -> Trapq {
        Arc::clone(&self.trapq)
    }

    /// `get_steppers` (`klippy/extras/manual_stepper.py:228`).
    pub fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }

    /// `get_position` (`klippy/extras/manual_stepper.py:206-207`).
    pub fn get_position(&self) -> Vec<f64> {
        vec![self.state.borrow().commanded_pos, 0.0, 0.0, 0.0]
    }

    /// `set_position` (`klippy/extras/manual_stepper.py:208-209`).
    pub fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        self.do_set_position(newpos[0])
    }

    /// `get_last_move_time` (`klippy/extras/manual_stepper.py:210-212`).
    pub fn get_last_move_time(&self) -> KResult<f64> {
        self.sync_print_time()?;
        Ok(self.state.borrow().next_cmd_time)
    }

    /// `dwell` (`klippy/extras/manual_stepper.py:213-214`).
    pub fn dwell(&self, delay: f64) -> KResult<()> {
        let cur = self.state.borrow().next_cmd_time;
        self.state.borrow_mut().next_cmd_time = cur + delay.max(0.0);
        Ok(())
    }

    /// `flush_step_generation` (`klippy/extras/manual_stepper.py:203-205`).
    pub fn flush_step_generation(&self) -> KResult<()> {
        self.toolhead()?.flush_step_generation()
    }

    /// `drip_move` (`klippy/extras/manual_stepper.py:215-225`).
    pub fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        self.sync_print_time()?;
        let start_time = self.state.borrow().next_cmd_time;
        let homing_accel = self.state.borrow().homing_accel;
        let end_time = self.submit_move(start_time, newpos[0], speed, homing_accel)?;
        self.motion_queuing.drip_update_time(start_time, end_time, drip_completion)?;
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }

    /// `calc_position` (`klippy/extras/manual_stepper.py:230-231`).
    pub fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let name = self.rail.get_name(None);
        match stepper_positions.get(&name) {
            Some(v) => Ok(vec![Some(*v), Some(0.0), Some(0.0)]),
            None => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: format!("'{name}'"),
            }),
        }
    }

    fn sync_print_time(&self) -> KResult<()> {
        let print_time = self.toolhead()?.get_last_move_time()?;
        let next = self.state.borrow().next_cmd_time;
        if next > print_time {
            self.toolhead()?.dwell(next - print_time)?;
        } else {
            self.state.borrow_mut().next_cmd_time = print_time;
        }
        Ok(())
    }

    /// `do_enable` (`klippy/extras/manual_stepper.py:54-57`).
    pub fn do_enable(&self, enable: i64) -> KResult<bool> {
        let names: Vec<String> =
            self.steppers.iter().map(|s| s.get_name(None)).collect();
        self.stepper_enable()?.set_motors_enable(&names, enable != 0)
    }

    /// `do_set_position` (`klippy/extras/manual_stepper.py:58-62`).
    pub fn do_set_position(&self, setpos: f64) -> KResult<()> {
        self.toolhead()?.flush_step_generation()?;
        self.state.borrow_mut().commanded_pos = setpos;
        self.rail.set_position([setpos, 0.0, 0.0]);
        Ok(())
    }

    /// `_submit_move` (`klippy/extras/manual_stepper.py:63-73`).
    pub fn submit_move(
        &self,
        movetime: f64,
        movepos: f64,
        speed: f64,
        accel: f64,
    ) -> KResult<f64> {
        let cp = self.state.borrow().commanded_pos;
        let dist = movepos - cp;
        let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(dist, speed, accel)?;
        (self.trapq_append)(
            &self.trapq,
            movetime,
            accel_t,
            cruise_t,
            accel_t,
            cp,
            0.0,
            0.0,
            axis_r,
            0.0,
            0.0,
            0.0,
            cruise_v,
            accel,
        );
        self.state.borrow_mut().commanded_pos = movepos;
        Ok(movetime + accel_t + cruise_t + accel_t)
    }

    /// `do_move` (`klippy/extras/manual_stepper.py:74-80`).
    pub fn do_move(
        &self,
        movepos: f64,
        speed: f64,
        accel: f64,
        sync: bool,
    ) -> KResult<()> {
        self.sync_print_time()?;
        let next = self.state.borrow().next_cmd_time;
        let new_next = self.submit_move(next, movepos, speed, accel)?;
        self.state.borrow_mut().next_cmd_time = new_next;
        let t = self.state.borrow().next_cmd_time;
        self.motion_queuing.note_mcu_movequeue_activity(t, true);
        if sync {
            self.sync_print_time()?;
        }
        Ok(())
    }

    /// `do_homing_move` (`klippy/extras/manual_stepper.py:81-92`).
    pub fn do_homing_move(
        &self,
        movepos: f64,
        speed: f64,
        accel: f64,
        probe_pos: bool,
        triggered: bool,
        check_trigger: bool,
    ) -> KResult<()> {
        if !self.state.borrow().can_home {
            return Err(KlippyError::Command(
                "No endstop for this manual stepper".to_string(),
            ));
        }
        self.state.borrow_mut().homing_accel = accel;
        let pos: Vec<Option<f64>> = vec![Some(movepos), Some(0.0), Some(0.0), Some(0.0)];
        let endstops = self.rail.get_endstops();
        let self_target: HomingTarget = self.arc_self();
        self.homing()?.manual_home(
            self_target,
            endstops,
            pos,
            speed,
            probe_pos,
            triggered,
            check_trigger,
        )?;
        self.sync_print_time()?;
        Ok(())
    }

    /// `cmd_MANUAL_STEPPER` (`klippy/extras/manual_stepper.py:94-137`).
    pub fn cmd_manual_stepper(&self, gcmd: &GCmd) -> KResult<()> {
        let gcode_axis_val = gcmd.get(
            "GCODE_AXIS",
            GGetOpts { default: Some(PyValue::None), ..Default::default() },
        )?;
        if !matches!(gcode_axis_val, PyValue::None) {
            return self.command_with_gcode_axis(gcmd);
        }
        if self.state.borrow().axis_gcode_id.is_some() {
            return Err(KlippyError::Command(
                "Must unregister from gcode axis first".to_string(),
            ));
        }
        if let Some(enable) = gcmd.get_int(
            "ENABLE",
            GGetIntOpts { default: Some(None), ..Default::default() },
        )? {
            self.do_enable(enable)?;
        }
        if let Some(setpos) = gcmd.get_float(
            "SET_POSITION",
            GGetFloatOpts { default: Some(None), ..Default::default() },
        )? {
            self.do_set_position(setpos)?;
        }
        let velocity = self.state.borrow().velocity;
        let accel0 = self.state.borrow().accel;
        let speed = gcmd
            .get_float(
                "SPEED",
                GGetFloatOpts {
                    default: Some(Some(velocity)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let accel = gcmd
            .get_float(
                "ACCEL",
                GGetFloatOpts {
                    default: Some(Some(accel0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let homing_val = gcmd.get(
            "STOP_ON_ENDSTOP",
            GGetOpts { default: Some(PyValue::None), ..Default::default() },
        )?;
        if !matches!(homing_val, PyValue::None) {
            let mut homing_move = match &homing_val {
                PyValue::Str(s) => s.clone(),
                _ => homing_val.str(),
            };
            let old_map: Option<(&str, &str)> = match homing_move.as_str() {
                "-2" => Some(("-2", "try_inverted_home")),
                "-1" => Some(("-1", "inverted_home")),
                "1" => Some(("1", "home")),
                "2" => Some(("2", "try_home")),
                _ => None,
            };
            if let Some((_, mapped)) = old_map {
                self.configfile()?.deprecate_gcode(
                    "MANUAL_STEPPER",
                    DeprecateGcodeOpts {
                        param: Some("STOP_ON_ENDSTOP".to_string()),
                        value: Some(PyValue::Str(homing_move.clone())),
                        msg: None,
                    },
                );
                homing_move = mapped.to_string();
            }
            let is_try = homing_move.starts_with("try_");
            if is_try {
                homing_move = homing_move[4..].to_string();
            }
            let is_inverted = homing_move.starts_with("inverted_");
            if is_inverted {
                homing_move = homing_move[9..].to_string();
            }
            if homing_move != "probe" && homing_move != "home" {
                return Err(KlippyError::Command(
                    "Unknown STOP_ON_ENDSTOP request".to_string(),
                ));
            }
            let is_probe = homing_move == "probe";
            let movepos = gcmd
                .get_float("MOVE", GGetFloatOpts::default())?
                .unwrap();
            let (pos_min, pos_max) = {
                let st = self.state.borrow();
                (st.pos_min, st.pos_max)
            };
            if (pos_min.is_some() && movepos < pos_min.unwrap())
                || (pos_max.is_some() && movepos > pos_max.unwrap())
            {
                return Err(KlippyError::Command("Move out of range".to_string()));
            }
            self.do_homing_move(movepos, speed, accel, is_probe, !is_inverted, !is_try)?;
        } else if let Some(movepos) = gcmd.get_float(
            "MOVE",
            GGetFloatOpts { default: Some(None), ..Default::default() },
        )? {
            let (pos_min, pos_max) = {
                let st = self.state.borrow();
                (st.pos_min, st.pos_max)
            };
            if (pos_min.is_some() && movepos < pos_min.unwrap())
                || (pos_max.is_some() && movepos > pos_max.unwrap())
            {
                return Err(KlippyError::Command("Move out of range".to_string()));
            }
            let sync = gcmd
                .get_int(
                    "SYNC",
                    GGetIntOpts { default: Some(Some(1)), ..Default::default() },
                )?
                .unwrap();
            self.do_move(movepos, speed, accel, sync != 0)?;
        } else if let Some(sync) = gcmd.get_int(
            "SYNC",
            GGetIntOpts { default: Some(Some(0)), ..Default::default() },
        )? {
            if sync != 0 {
                self.sync_print_time()?;
            }
        }
        Ok(())
    }

    /// `command_with_gcode_axis` (`klippy/extras/manual_stepper.py:139-167`).
    pub fn command_with_gcode_axis(&self, gcmd: &GCmd) -> KResult<()> {
        // Python looks up gcode_move first (raises if missing).
        let _ = lookup_interface::<dyn GCodeMoveApi>(&*self.printer, "gcode_move", LookupDefault::Raise)?
            .expect("gcode_move lookup with Raise never yields None");
        let toolhead = self.toolhead()?;
        let gcode_axis_val = gcmd.get("GCODE_AXIS", GGetOpts::default())?;
        let gcode_axis = match &gcode_axis_val {
            PyValue::Str(s) => s.to_uppercase(),
            _ => gcode_axis_val.str().to_uppercase(),
        };
        let instant_corner_v = gcmd
            .get_float(
                "INSTANTANEOUS_CORNER_VELOCITY",
                GGetFloatOpts {
                    default: Some(Some(1.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let limit_velocity = gcmd
            .get_float(
                "LIMIT_VELOCITY",
                GGetFloatOpts {
                    default: Some(Some(999999.9)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        let limit_accel = gcmd
            .get_float(
                "LIMIT_ACCEL",
                GGetFloatOpts {
                    default: Some(Some(999999.9)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap();
        if self.state.borrow().axis_gcode_id.is_some() {
            if !gcode_axis.is_empty() {
                return Err(KlippyError::Command(
                    "Must unregister axis first".to_string(),
                ));
            }
            let ea: ExtraAxis = self.arc_self();
            toolhead.remove_extra_axis(&ea)?;
            self.state.borrow_mut().axis_gcode_id = None;
            return Ok(());
        }
        let is_upper_single = gcode_axis.len() == 1
            && gcode_axis
                .chars()
                .next()
                .map(|c| c.is_uppercase())
                .unwrap_or(false);
        if gcode_axis.len() != 1 || !is_upper_single || "XYZEFN".contains(&gcode_axis) {
            if gcode_axis.is_empty() {
                return Ok(());
            }
            return Err(KlippyError::Command("Not a valid GCODE_AXIS".to_string()));
        }
        for ea in toolhead.get_extra_axes().into_iter().flatten() {
            if ea.get_axis_gcode_id().as_deref() == Some(gcode_axis.as_str()) {
                    return Err(KlippyError::Command(format!(
                        "Axis '{gcode_axis}' already registered"
                    )));
                }
        }
        let commanded = self.state.borrow().commanded_pos;
        self.state.borrow_mut().axis_gcode_id = Some(gcode_axis);
        self.state.borrow_mut().instant_corner_v = instant_corner_v;
        self.state.borrow_mut().gaxis_limit_velocity = limit_velocity;
        self.state.borrow_mut().gaxis_limit_accel = limit_accel;
        let ea: ExtraAxis = self.arc_self();
        toolhead.add_extra_axis(ea, commanded)?;
        Ok(())
    }

    /// `process_move` (`klippy/extras/manual_stepper.py:168-179`).
    pub fn process_move_impl(
        &self,
        print_time: f64,
        move_: &dyn MoveApi,
        ea_index: usize,
    ) {
        let axis_r = move_.axes_r().0[ea_index];
        let start_pos = move_.start_pos().0[ea_index];
        let accel = move_.accel() * axis_r;
        let start_v = move_.start_v() * axis_r;
        let cruise_v = move_.cruise_v() * axis_r;
        (self.trapq_append)(
            &self.trapq,
            print_time,
            move_.accel_t(),
            move_.cruise_t(),
            move_.decel_t(),
            start_pos,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            start_v,
            cruise_v,
            accel,
        );
        self.state.borrow_mut().commanded_pos = move_.end_pos().0[ea_index];
    }

    /// `check_move` (`klippy/extras/manual_stepper.py:180-192`).
    pub fn check_move_impl(&self, move_: &dyn MoveApi, ea_index: usize) -> KResult<()> {
        let movepos = move_.end_pos().0[ea_index];
        let (pos_min, pos_max, gaxis_v, gaxis_a, accel0) = {
            let st = self.state.borrow();
            (st.pos_min, st.pos_max, st.gaxis_limit_velocity, st.gaxis_limit_accel, st.accel)
        };
        if (pos_min.is_some() && movepos < pos_min.unwrap())
            || (pos_max.is_some() && movepos > pos_max.unwrap())
        {
            return Err(move_.move_error("Move out of range".to_string()));
        }
        let axis_ratio = float_true_div(move_.move_d(), move_.axes_d().0[ea_index].abs())
            .map_err(map_py_error)?;
        let limit_velocity = gaxis_v * axis_ratio;
        let mut limit_accel = gaxis_a * axis_ratio;
        if !move_.is_kinematic_move() && accel0 != 0.0 {
            limit_accel = limit_accel.min(accel0 * axis_ratio);
        }
        move_.limit_speed(limit_velocity, limit_accel)?;
        Ok(())
    }

    /// `calc_junction` (`klippy/extras/manual_stepper.py:193-197`).
    pub fn calc_junction_impl(
        &self,
        prev_move: &dyn MoveApi,
        move_: &dyn MoveApi,
        ea_index: usize,
    ) -> f64 {
        let diff_r = move_.axes_r().0[ea_index] - prev_move.axes_r().0[ea_index];
        if diff_r != 0.0 {
            let icv = self.state.borrow().instant_corner_v;
            let base = icv / diff_r.abs();
            return base * base;
        }
        move_.max_cruise_v2()
    }
}

impl ExtraAxisApi for ManualStepper {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        self.state.borrow().axis_gcode_id.clone()
    }
    fn get_trapq(&self) -> Option<Trapq> {
        Some(Arc::clone(&self.trapq))
    }
    fn check_move(&self, mv: &dyn MoveApi, ea_index: usize) -> KResult<()> {
        self.check_move_impl(mv, ea_index)
    }
    fn calc_junction(&self, prev_move: &dyn MoveApi, mv: &dyn MoveApi, ea_index: usize) -> f64 {
        self.calc_junction_impl(prev_move, mv, ea_index)
    }
    fn process_move(&self, print_time: f64, mv: &dyn MoveApi, ea_index: usize) {
        self.process_move_impl(print_time, mv, ea_index)
    }
}

impl HomingTargetApi for ManualStepper {
    fn get_position(&self) -> Vec<f64> {
        vec![self.state.borrow().commanded_pos, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.do_set_position(newpos[0])?;
        let _ = homing_axes;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        self.arc_self()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.toolhead()?.flush_step_generation()
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        ManualStepper::get_last_move_time(self)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        ManualStepper::dwell(self, delay)
    }
    fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        self.sync_print_time()?;
        let start_time = self.state.borrow().next_cmd_time;
        let homing_accel = self.state.borrow().homing_accel;
        let end_time = self.submit_move(start_time, newpos[0], speed, homing_accel)?;
        self.motion_queuing.drip_update_time(start_time, end_time, drip_completion)?;
        self.motion_queuing.wipe_trapq(&self.trapq);
        Ok(())
    }
}

impl KinematicsApi for ManualStepper {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let name = self.rail.get_name(None);
        match stepper_positions.get(&name) {
            Some(v) => Ok(vec![Some(*v), Some(0.0), Some(0.0)]),
            None => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: format!("'{name}'"),
            }),
        }
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.do_set_position(newpos[0])?;
        let _ = homing_axes;
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Err(klippy_api::toolhead::no_kinematics_attr("check_move"))
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Err(klippy_api::toolhead::no_kinematics_attr("home"))
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn clear_homing_state(&self, _clear_axes: &str) {
        // Python ManualStepper has no clear_homing_state; default
        // trait version is not provided, so do nothing? The trait
        // requires this method (no default), so provide no-op.
        // Actually KinematicsApi::clear_homing_state has no default;
        // the Python object lacks the attribute, which would be an
        // AttributeError if called. Mirror the core's missing-attr
        // behaviour is impossible here without error return; keep
        // no-op and never rely on it.
    }
}

/// `load_config_prefix` (`klippy/extras/manual_stepper.py:233-234`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = ManualStepper::new(config)?;
    Ok(Some(obj as ObjectRef))
}

/// Module factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
