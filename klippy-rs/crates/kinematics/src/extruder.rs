//! Port of the `extruder` module (`klippy/kinematics/extruder.py`).
//!
//! Tracks the hotend heater, the extrusion motion queue and an optional
//! extruder stepper (`ExtruderStepper`) with pressure advance, plus a
//! dummy extruder used when the printer has none.
//!
//! The C side of pressure advance (`kin_extruder.c`:
//! `extruder_stepper_alloc` / `extruder_set_pressure_advance`) lives in the
//! `chelper_motion` `kin_extruder` unit, reached through the frozen
//! motion handles: [`alloc_stepper_kinematics`](klippy_core::motion_handles::alloc_stepper_kinematics)
//! at construction and [`kinematics_of(...).with(...)`](klippy_core::motion_handles::kinematics_of)
//! for the kind-specific call, exactly as the reference reaches it through
//! `ffi_lib`. The `calc_position` integration itself stays with the C unit.
//!
//! `heaters.Heater` / `heaters.PrinterHeaters` are owned by the `extras`
//! `heaters` module and used through `extras::heaters`. `toolhead`,
//! `gcode`, `motion_queuing`, the stepper factory and the `Move` object use
//! the frozen `klippy_api` contracts.

use extras::heaters::{Heater, HeaterApi, PrinterHeatersApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCodeApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GCmd};
use klippy_api::log;
use klippy_api::object::{
    downcast_arc, load_interface, lookup_interface, require_interface, ObjectRef, PrinterObject,
};
use klippy_api::printer::{
    EV_EXTRUDER_ACTIVATE_EXTRUDER, EV_KLIPPY_CONNECT, LookupDefault, ModuleFactory, Printer,
};
use klippy_api::toolhead::{
    ExtraAxisApi, ExtruderStepperApi, MotionQueuing, MotionQueuingApi, MoveApi,
    PrinterExtruder as ThExtruder, PrinterExtruderApi, ToolHeadApi,
};
use klippy_api::stepper::{Stepper, StepperKinematics, Trapq};
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of, KinAlloc};
use pycompat::PyValue;
use pycompat::text::{float_pow, float_true_div, percent_format, PctValue, PyError};
use std::sync::Arc;

/// `heaters.PrinterHeaters` handle.
pub type PrinterHeaters = Arc<dyn PrinterHeatersApi>;

// ---------------------------------------------------------------------------
// Helpers mirroring Python formatting / arithmetic.
// ---------------------------------------------------------------------------

fn fmt(fmt_str: &str, args: &[PctValue]) -> String {
    percent_format(fmt_str, args).expect("constant extruder format")
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg }
}

fn div_f64(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(map_py_error)
}

fn pow_f64(a: f64, b: f64) -> KResult<f64> {
    float_pow(a, b).map_err(map_py_error)
}

/// Python `math.pi` (exactly rounded to double).
const PI: f64 = std::f64::consts::PI;

/// Value of a required config float. The frozen config contract raises
/// before returning `None` for a required option, so `None` is a
/// contract violation; map it to the configfile error text either way.
fn required_f64(config: &Config, option: &str, v: Option<f64>) -> KResult<f64> {
    v.ok_or_else(|| {
        KlippyError::config(fmt(
            "Option '%s' in section '%s' must be specified",
            &[PctValue::Str(option.to_string()), PctValue::Str(config.get_name())],
        ))
    })
}

/// Config `getfloat` whose option always carries a default.
fn default_f64(v: Option<f64>, default: f64) -> f64 {
    v.unwrap_or(default)
}

fn dict(pairs: Vec<(&str, PyValue)>) -> PyValue {
    let mut map = IndexMap::new();
    for (k, v) in pairs {
        map.insert(PyValue::Str(k.to_string()), v);
    }
    PyValue::Dict(map)
}


fn same_trapq(a: &Option<Trapq>, b: &Trapq) -> bool {
    match a {
        Some(x) => Arc::ptr_eq(x, b),
        None => false,
    }
}
/// `sk_extruder.extruder_set_pressure_advance(print_time, pa, smooth_time)`
/// through the frozen motion handle. The handle is always the
/// `extruder_stepper_alloc` allocation made in `ExtruderStepper::new`, so
/// any other variant (or a foreign handle) is unreachable.
fn set_extruder_pa(sk: &StepperKinematics, print_time: f64, pressure_advance: f64, smooth_time: f64) -> KResult<()> {
    let handle = kinematics_of(sk)?;
    handle.with(|alloc| {
        match alloc {
            KinAlloc::Extruder(es) => es.set_pressure_advance(print_time, pressure_advance, smooth_time),
            _ => panic!("non-extruder stepper kinematics"),
        }
    });
    Ok(())
}


// ---------------------------------------------------------------------------
// ExtruderStepper
// ---------------------------------------------------------------------------

struct StepperState {
    pressure_advance: f64,
    pressure_advance_smooth_time: f64,
    motion_queue: Option<String>,
}

/// Port of `ExtruderStepper` (`klippy/kinematics/extruder.py:9`).
pub struct ExtruderStepper {
    printer: Printer,
    name: String,
    state: GCell<StepperState>,
    config_pa: f64,
    config_smooth_time: f64,
    stepper: Stepper,
    sk: StepperKinematics,
}

impl ExtruderStepper {
    /// `ExtruderStepper.__init__(config)`.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer: Printer = config.get_printer();
        let name = config.get_name().split_whitespace().next_back().unwrap_or("").to_string();
        let config_pa = default_f64(
            config.getfloat(
                "pressure_advance",
                GetFloatOpts { default: Some(Some(0.0)), minval: Some(0.0), ..Default::default() },
            )?,
            0.0,
        );
        let config_smooth_time = default_f64(
            config.getfloat(
                "pressure_advance_smooth_time",
                GetFloatOpts { default: Some(Some(0.040)), above: Some(0.0), maxval: Some(0.200), ..Default::default() },
            )?,
            0.040,
        );
        // `stepper.PrinterStepper(config)` via the toolhead factory.
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*printer, "toolhead", LookupDefault::Raise)?
            .expect("Raise yields the toolhead");
        let stepper = toolhead.stepper_factory().printer_stepper(config, false)?;
        // `extruder_stepper_alloc()` through the frozen motion handles,
        // then `set_stepper_kinematics`.
        let sk = alloc_stepper_kinematics("extruder_stepper_alloc", &[])?;
        stepper.set_stepper_kinematics(Some(sk.clone()));
        let me = Arc::new(Self {
            printer: printer.clone(),
            name: name.clone(),
            state: GCell::new(StepperState {
                pressure_advance: 0.0,
                pressure_advance_smooth_time: 0.0,
                motion_queue: None,
            }),
            config_pa,
            config_smooth_time,
            stepper,
            sk,
        });
        // Register the connect handler (`klippy:connect`).
        {
            let me_cb = me.clone();
            printer.register_event_handler(EV_KLIPPY_CONNECT, Box::new(move |_args| {
                me_cb.handle_connect()?;
                Ok(PyValue::None)
            }));
        }
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise yields gcode");
        if me.name == "extruder" {
            let me_cb = me.clone();
            gcode.register_mux_command(
                "SET_PRESSURE_ADVANCE",
                "EXTRUDER",
                None,
                Box::new(move |gcmd| me_cb.cmd_default_set_pressure_advance(gcmd)),
                Some(Self::CMD_SET_PRESSURE_ADVANCE_HELP),
            )?;
        }
        {
            let me_cb = me.clone();
            gcode.register_mux_command(
                "SET_PRESSURE_ADVANCE",
                "EXTRUDER",
                Some(&me.name),
                Box::new(move |gcmd| me_cb.cmd_set_pressure_advance(gcmd)),
                Some(Self::CMD_SET_PRESSURE_ADVANCE_HELP),
            )?;
        }
        {
            let me_cb = me.clone();
            gcode.register_mux_command(
                "SET_EXTRUDER_ROTATION_DISTANCE",
                "EXTRUDER",
                Some(&me.name),
                Box::new(move |gcmd| me_cb.cmd_set_e_rotation_distance(gcmd)),
                Some(Self::CMD_SET_E_ROTATION_DISTANCE_HELP),
            )?;
        }
        {
            let me_cb = me.clone();
            gcode.register_mux_command(
                "SYNC_EXTRUDER_MOTION",
                "EXTRUDER",
                Some(&me.name),
                Box::new(move |gcmd| me_cb.cmd_sync_extruder_motion(gcmd)),
                Some(Self::CMD_SYNC_EXTRUDER_MOTION_HELP),
            )?;
        }
        Ok(me)
    }

    /// `cmd_SET_PRESSURE_ADVANCE_help` (`extruder.py:89`).
    pub const CMD_SET_PRESSURE_ADVANCE_HELP: &'static str = "Set pressure advance parameters";
    /// `cmd_SET_E_ROTATION_DISTANCE_help` (`extruder.py:110`).
    pub const CMD_SET_E_ROTATION_DISTANCE_HELP: &'static str = "Set extruder rotation distance";
    /// `cmd_SYNC_EXTRUDER_MOTION_help` (`extruder.py:132`).
    pub const CMD_SYNC_EXTRUDER_MOTION_HELP: &'static str = "Set extruder stepper motion queue";

    /// Stepper name (`config.get_name().split()[-1]`).
    pub fn name(&self) -> &str {
        &self.name
    }
    /// Current `pressure_advance`.
    pub fn pressure_advance(&self) -> f64 {
        self.state.borrow().pressure_advance
    }
    /// Current `pressure_advance_smooth_time`.
    pub fn pressure_advance_smooth_time(&self) -> f64 {
        self.state.borrow().pressure_advance_smooth_time
    }
    /// Config `pressure_advance`.
    pub fn config_pa(&self) -> f64 {
        self.config_pa
    }
    /// Config `pressure_advance_smooth_time`.
    pub fn config_smooth_time(&self) -> f64 {
        self.config_smooth_time
    }
    /// The underlying `PrinterStepper`.
    pub fn stepper(&self) -> Stepper {
        self.stepper.clone()
    }
    /// The pressure-advance kinematics allocation handle.
    pub fn kinematics(&self) -> StepperKinematics {
        self.sk.clone()
    }
    /// `motion_queue` attribute.
    pub fn motion_queue(&self) -> Option<String> {
        self.state.borrow().motion_queue.clone()
    }

    /// `_handle_connect` (`extruder.py:41`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.set_pressure_advance_inner(self.config_pa, self.config_smooth_time)
    }

    fn toolhead(&self) -> KResult<Arc<dyn ToolHeadApi>> {
        lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::config("Unknown config object 'toolhead'"))
    }

    fn motion_queuing(&self) -> KResult<MotionQueuing> {
        lookup_interface::<dyn MotionQueuingApi>(&*self.printer, "motion_queuing", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::config("Unknown config object 'motion_queuing'"))
    }

    /// `_set_pressure_advance` (`extruder.py:67`).
    pub fn set_pressure_advance_inner(&self, pressure_advance: f64, smooth_time: f64) -> KResult<()> {
        let old_smooth_time = {
            let st = self.state.borrow();
            if st.pressure_advance == 0.0 {
                0.0
            } else {
                st.pressure_advance_smooth_time
            }
        };
        let new_smooth_time = if pressure_advance == 0.0 { 0.0 } else { smooth_time };
        let toolhead = self.toolhead()?;
        let sk = self.sk.clone();
        if new_smooth_time != old_smooth_time {
            toolhead.flush_step_generation()?;
            set_extruder_pa(&sk, 0.0, pressure_advance, new_smooth_time)?;
            let mq = self.motion_queuing()?;
            mq.check_step_generation_scan_windows();
        } else {
            toolhead.register_lookahead_callback(Box::new(move |print_time| {
                set_extruder_pa(&sk, print_time, pressure_advance, new_smooth_time)
            }))?;
        }
        {
            let mut st = self.state.borrow_mut();
            st.pressure_advance = pressure_advance;
            st.pressure_advance_smooth_time = smooth_time;
        }
        Ok(())
    }

    /// `sync_to_extruder` (`extruder.py:50`).
    pub fn sync_to_extruder_inner(&self, extruder_name: Option<&str>) -> KResult<()> {
        let toolhead = self.toolhead()?;
        toolhead.flush_step_generation()?;
        let mq = self.motion_queuing()?;
        match extruder_name {
            None => {
                self.stepper.set_trapq(None);
                self.state.borrow_mut().motion_queue = None;
                mq.check_step_generation_scan_windows();
                return Ok(());
            }
            Some("") => {
                self.stepper.set_trapq(None);
                self.state.borrow_mut().motion_queue = None;
                mq.check_step_generation_scan_windows();
                return Ok(());
            }
            Some(_) => {}
        }
        let extruder_name = extruder_name.unwrap();
        let extruder_obj = self
            .printer
            .lookup_object(extruder_name, LookupDefault::ReturnNone)?;
        let extruder = extruder_obj
            .as_ref()
            .and_then(downcast_arc::<PrinterExtruder>);
        let extruder = match extruder {
            Some(e) => e,
            None => {
                return Err(KlippyError::command(fmt(
                    "'%s' is not a valid extruder.",
                    &[PctValue::Str(extruder_name.to_string())],
                )))
            }
        };
        self.stepper.set_position([extruder.last_position(), 0.0, 0.0]);
        self.stepper.set_trapq(Some(extruder.trapq()));
        self.state.borrow_mut().motion_queue = Some(extruder_name.to_string());
        mq.check_step_generation_scan_windows();
        Ok(())
    }

    /// `cmd_default_SET_PRESSURE_ADVANCE` (`extruder.py:90`).
    pub fn cmd_default_set_pressure_advance(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let active = toolhead.get_extruder();
        // Python reads `extruder.extruder_stepper` directly: a `DummyExtruder`
        // has no such attribute (`AttributeError`), while a real extruder
        // without a stepper yields the command error below. The core dummy
        // is the only active extruder with an empty name and no trapq.
        let stepper = match active.extruder_stepper() {
            Some(s) => s,
            None if active.get_name().is_empty() && ExtraAxisApi::get_trapq(&*active).is_none() => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'DummyExtruder' object has no attribute 'extruder_stepper'".to_string(),
                })
            }
            None => return Err(KlippyError::command("Active extruder does not have a stepper")),
        };
        let strapq = stepper.stepper().get_trapq();
        let trapq = ExtraAxisApi::get_trapq(&*active).expect("active extruder has a trapq");
        if !same_trapq(&strapq, &trapq) {
            return Err(KlippyError::command("Unable to infer active extruder stepper"));
        }
        stepper.cmd_set_pressure_advance(gcmd)
    }

    /// `cmd_SET_PRESSURE_ADVANCE` (`extruder.py:98`).
    pub fn cmd_set_pressure_advance(&self, gcmd: &GCmd) -> KResult<()> {
        let cur_pa = self.pressure_advance();
        let cur_smooth = self.pressure_advance_smooth_time();
        let pressure_advance = gcmd
            .get_float(
                "ADVANCE",
                GGetFloatOpts { default: Some(Some(cur_pa)), minval: Some(0.0), ..Default::default() },
            )?
            .unwrap_or(cur_pa);
        let smooth_time = gcmd
            .get_float(
                "SMOOTH_TIME",
                GGetFloatOpts { default: Some(Some(cur_smooth)), minval: Some(0.0), maxval: Some(0.200), ..Default::default() },
            )?
            .unwrap_or(cur_smooth);
        self.set_pressure_advance_inner(pressure_advance, smooth_time)?;
        let msg = fmt(
            "pressure_advance: %.6f\npressure_advance_smooth_time: %.6f",
            &[PctValue::Float(pressure_advance), PctValue::Float(smooth_time)],
        );
        let rollover = fmt("%s: %s", &[PctValue::Str(self.name.clone()), PctValue::Str(msg.clone())]);
        self.printer.set_rollover_info(&self.name, Some(&rollover), true);
        gcmd.respond_info(&msg, false);
        Ok(())
    }

    /// `cmd_SET_E_ROTATION_DISTANCE` (`extruder.py:111`).
    pub fn cmd_set_e_rotation_distance(&self, gcmd: &GCmd) -> KResult<()> {
        let rotation_dist = gcmd.get_float(
            "DISTANCE",
            GGetFloatOpts { default: Some(None), ..Default::default() },
        )?;
        let mut rotation_dist = match rotation_dist {
            Some(rotation_dist) => {
                if rotation_dist == 0.0 {
                    return Err(KlippyError::command("Rotation distance can not be zero"));
                }
                let (_invert_dir, orig_invert_dir) = self.stepper.get_dir_inverted();
                let mut next_invert_dir = orig_invert_dir;
                let mut rotation_dist = rotation_dist;
                if rotation_dist < 0.0 {
                    next_invert_dir = !orig_invert_dir;
                    rotation_dist = -rotation_dist;
                }
                let toolhead = self.toolhead()?;
                toolhead.flush_step_generation()?;
                self.stepper.set_rotation_distance(rotation_dist);
                self.stepper.set_dir_inverted(next_invert_dir)?;
                rotation_dist
            }
            None => {
                let (rd, _spr) = self.stepper.get_rotation_distance();
                rd
            }
        };
        let (invert_dir, orig_invert_dir) = self.stepper.get_dir_inverted();
        if invert_dir != orig_invert_dir {
            rotation_dist = -rotation_dist;
        }
        let msg = fmt(
            "Extruder '%s' rotation distance set to %0.6f",
            &[PctValue::Str(self.name.clone()), PctValue::Float(rotation_dist)],
        );
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    /// `cmd_SYNC_EXTRUDER_MOTION` (`extruder.py:133`).
    pub fn cmd_sync_extruder_motion(&self, gcmd: &GCmd) -> KResult<()> {
        let ename_value = gcmd.get("MOTION_QUEUE", GGetOpts::default())?;
        let (ename_string, sync_arg) = match &ename_value {
            PyValue::None => ("None".to_string(), None),
            PyValue::Str(s) => {
                let s = s.clone();
                let sync = if s.is_empty() { None } else { Some(s.clone()) };
                (s, sync)
            }
            other => {
                let s = other.str();
                let sync = if s.is_empty() { None } else { Some(s.clone()) };
                (s, sync)
            }
        };
        self.sync_to_extruder_inner(sync_arg.as_deref())?;
        let msg = fmt(
            "Extruder '%s' now syncing with '%s'",
            &[PctValue::Str(self.name.clone()), PctValue::Str(ename_string)],
        );
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

impl ExtruderStepperApi for ExtruderStepper {
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let st = self.state.borrow();
        dict(vec![
            ("pressure_advance", PyValue::Float(st.pressure_advance)),
            ("smooth_time", PyValue::Float(st.pressure_advance_smooth_time)),
            (
                "motion_queue",
                match &st.motion_queue {
                    Some(q) => PyValue::Str(q.clone()),
                    None => PyValue::None,
                },
            ),
        ])
    }
    fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        let mcu_pos = self.stepper.get_past_mcu_position(print_time);
        Ok(self.stepper.mcu_to_commanded_position(mcu_pos))
    }
    fn sync_to_extruder(&self, extruder_name: Option<&str>) -> KResult<()> {
        self.sync_to_extruder_inner(extruder_name)
    }
    fn stepper(&self) -> Stepper {
        self.stepper.clone()
    }
    fn cmd_set_pressure_advance(&self, gcmd: &GCmd) -> KResult<()> {
        self.cmd_set_pressure_advance(gcmd)
    }
}

impl PrinterObject for ExtruderStepper {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ExtruderStepperApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(ExtruderStepperApi::get_status(self, eventtime))
    }
}

// ---------------------------------------------------------------------------
// PrinterExtruder
// ---------------------------------------------------------------------------

/// Port of `PrinterExtruder` (`klippy/kinematics/extruder.py:140`).
pub struct PrinterExtruder {
    printer: Printer,
    name: String,
    last_position: GCell<f64>,
    heater: Arc<Heater>,
    nozzle_diameter: f64,
    filament_area: f64,
    max_extrude_ratio: f64,
    max_e_velocity: f64,
    max_e_accel: f64,
    max_e_dist: f64,
    instant_corner_v: f64,
    trapq: Trapq,
    trapq_append: klippy_api::toolhead::TrapqAppend,
    extruder_stepper: Option<Arc<ExtruderStepper>>,
}

impl PrinterExtruder {
    /// `PrinterExtruder.__init__(config, extruder_num)`.
    pub fn new(config: &Config, extruder_num: usize) -> KResult<Arc<Self>> {
        let printer: Printer = config.get_printer();
        let name = config.get_name();
        // Setup hotend heater.
        let pheaters = load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
            .expect("Raise yields heaters");
        let gcode_id = fmt("T%d", &[PctValue::Int(pycompat::PyInt::from_i64(extruder_num as i64))]);
        let heater = pheaters.setup_heater(config, Some(gcode_id))?;
        // Setup kinematic checks.
        let nozzle_diameter = required_f64(
            config,
            "nozzle_diameter",
            config.getfloat("nozzle_diameter", GetFloatOpts { above: Some(0.0), ..Default::default() })?,
        )?;
        let filament_diameter = required_f64(
            config,
            "filament_diameter",
            config.getfloat("filament_diameter", GetFloatOpts { minval: Some(nozzle_diameter), ..Default::default() })?,
        )?;
        let filament_area = PI * pow_f64(filament_diameter * 0.5, 2.0)?;
        let def_max_cross_section = 4.0 * pow_f64(nozzle_diameter, 2.0)?;
        let def_max_extrude_ratio = div_f64(def_max_cross_section, filament_area)?;
        let max_cross_section = default_f64(
            config.getfloat(
                "max_extrude_cross_section",
                GetFloatOpts { default: Some(Some(def_max_cross_section)), above: Some(0.0), ..Default::default() },
            )?,
            def_max_cross_section,
        );
        let max_extrude_ratio = div_f64(max_cross_section, filament_area)?;
        log::info(&fmt(
            "Extruder max_extrude_ratio=%.6f",
            &[PctValue::Float(max_extrude_ratio)],
        ));
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*printer, "toolhead", LookupDefault::Raise)?
            .expect("Raise yields toolhead");
        let (max_velocity, max_accel) = toolhead.get_max_velocity();
        let max_e_velocity = default_f64(
            config.getfloat(
                "max_extrude_only_velocity",
                GetFloatOpts { default: Some(Some(max_velocity * def_max_extrude_ratio)), above: Some(0.0), ..Default::default() },
            )?,
            max_velocity * def_max_extrude_ratio,
        );
        let max_e_accel = default_f64(
            config.getfloat(
                "max_extrude_only_accel",
                GetFloatOpts { default: Some(Some(max_accel * def_max_extrude_ratio)), above: Some(0.0), ..Default::default() },
            )?,
            max_accel * def_max_extrude_ratio,
        );
        let max_e_dist = default_f64(
            config.getfloat(
                "max_extrude_only_distance",
                GetFloatOpts { default: Some(Some(50.0)), minval: Some(0.0), ..Default::default() },
            )?,
            50.0,
        );
        let instant_corner_v = default_f64(
            config.getfloat(
                "instantaneous_corner_velocity",
                GetFloatOpts { default: Some(Some(1.0)), minval: Some(0.0), ..Default::default() },
            )?,
            1.0,
        );
        // Setup extruder trapq.
        let motion_queuing = load_interface::<dyn MotionQueuingApi>(&*printer, config, "motion_queuing", LookupDefault::Raise)?
            .expect("Raise yields motion_queuing");
        let trapq = motion_queuing.allocate_trapq();
        let trapq_append = motion_queuing.lookup_trapq_append();
        // Setup extruder stepper when a stepper option is present.
        let has_stepper = config.get("step_pin", GetStrOpts { default: Some(None), ..Default::default() })?.is_some()
            || config.get("dir_pin", GetStrOpts { default: Some(None), ..Default::default() })?.is_some()
            || config.get("rotation_distance", GetStrOpts { default: Some(None), ..Default::default() })?.is_some();
        let extruder_stepper = if has_stepper {
            let es = ExtruderStepper::new(config)?;
            es.stepper.set_trapq(Some(trapq.clone()));
            Some(es)
        } else {
            None
        };
        let me = Arc::new(Self {
            printer: printer.clone(),
            name: name.clone(),
            last_position: GCell::new(0.0),
            heater,
            nozzle_diameter,
            filament_area,
            max_extrude_ratio,
            max_e_velocity,
            max_e_accel,
            max_e_dist,
            instant_corner_v,
            trapq,
            trapq_append,
            extruder_stepper,
        });
        // Register commands.
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise yields gcode");
        if me.name == "extruder" {
            let me_cb = me.clone();
            let me_handle: ThExtruder = me_cb.clone();
            toolhead.set_extruder(me_handle, 0.0);
            let me_m104 = me.clone();
            gcode.register_command("M104", Some(Box::new(move |gcmd| me_m104.cmd_m104(gcmd, false))), Default::default())?;
            let me_m109 = me.clone();
            gcode.register_command("M109", Some(Box::new(move |gcmd| me_m109.cmd_m104(gcmd, true))), Default::default())?;
        }
        {
            let me_cb = me.clone();
            gcode.register_mux_command(
                "ACTIVATE_EXTRUDER",
                "EXTRUDER",
                Some(&me.name),
                Box::new(move |gcmd| me_cb.cmd_activate_extruder(gcmd)),
                Some(PrinterExtruder::CMD_ACTIVATE_EXTRUDER_HELP),
            )?;
        }
        Ok(me)
    }

    /// `cmd_ACTIVATE_EXTRUDER_help` (`extruder.py:280`).
    pub const CMD_ACTIVATE_EXTRUDER_HELP: &'static str = "Change the active extruder";

    /// Extruder name.
    pub fn name(&self) -> &str {
        &self.name
    }
    /// `last_position`.
    pub fn last_position(&self) -> f64 {
        self.last_position.get()
    }
    /// `nozzle_diameter`.
    pub fn nozzle_diameter(&self) -> f64 {
        self.nozzle_diameter
    }
    /// `filament_area`.
    pub fn filament_area(&self) -> f64 {
        self.filament_area
    }
    /// `max_extrude_ratio`.
    pub fn max_extrude_ratio(&self) -> f64 {
        self.max_extrude_ratio
    }
    /// `max_e_velocity`.
    pub fn max_e_velocity(&self) -> f64 {
        self.max_e_velocity
    }
    /// `max_e_accel`.
    pub fn max_e_accel(&self) -> f64 {
        self.max_e_accel
    }
    /// `max_e_dist`.
    pub fn max_e_dist(&self) -> f64 {
        self.max_e_dist
    }
    /// `instant_corner_v`.
    pub fn instant_corner_v(&self) -> f64 {
        self.instant_corner_v
    }
    /// `trapq`.
    pub fn trapq(&self) -> Trapq {
        self.trapq.clone()
    }
    /// The attached extruder stepper, if any (concrete).
    pub fn extruder_stepper_concrete(&self) -> Option<Arc<ExtruderStepper>> {
        self.extruder_stepper.clone()
    }
    /// `extruder_stepper` attribute as the core handle.
    pub fn extruder_stepper_handle(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        self.extruder_stepper.as_ref().map(|s| s.clone() as Arc<dyn ExtruderStepperApi>)
    }

    fn toolhead(&self) -> KResult<Arc<dyn ToolHeadApi>> {
        lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .ok_or_else(|| KlippyError::config("Unknown config object 'toolhead'"))
    }

    fn printers_heaters(&self) -> KResult<PrinterHeaters> {
        let obj = self.printer.lookup_object("heaters", LookupDefault::Raise)?;
        let obj = obj.ok_or_else(|| KlippyError::config("Unknown config object 'heaters'"))?;
        require_interface::<dyn PrinterHeatersApi>(&obj, "heaters")
    }

    /// `get_status` (`extruder.py:192`).
    pub fn status(&self, eventtime: f64) -> PyValue {
        let mut sts = HeaterApi::get_status(&*self.heater, eventtime);
        let can_extrude = HeaterApi::can_extrude(&*self.heater);
        if let PyValue::Dict(map) = &mut sts {
            map.insert(PyValue::Str("can_extrude".to_string()), PyValue::Bool(can_extrude));
            if let Some(stepper) = &self.extruder_stepper {
                let stepper_status = ExtruderStepperApi::get_status(&**stepper, eventtime);
                if let PyValue::Dict(stepper_map) = stepper_status {
                    for (k, v) in stepper_map {
                        map.insert(k, v);
                    }
                }
            }
        } else {
            // Python assigns into the dict the heater returned and raises
            // TypeError for anything else; the heater contract is a dict.
            unreachable!("heaters.Heater.get_status must return a dict")
        }
        sts
    }

    /// `cmd_M104` (`extruder.py:260`); `cmd_M109` is `(gcmd, wait=True)`.
    pub fn cmd_m104(&self, gcmd: &GCmd, wait: bool) -> KResult<()> {
        let temp = gcmd
            .get_float("S", GGetFloatOpts { default: Some(Some(0.0)), ..Default::default() })?
            .unwrap_or(0.0);
        let index = gcmd.get_int("T", GGetIntOpts { default: Some(None), minval: Some(0), ..Default::default() })?;
        let heater: ObjectRef = match index {
            Some(index) => {
                let section = if index == 0 {
                    "extruder".to_string()
                } else {
                    fmt("extruder%d", &[PctValue::Int(pycompat::PyInt::from_i64(index))])
                };
                let extruder_obj = self.printer.lookup_object(&section, LookupDefault::ReturnNone)?;
                match extruder_obj {
                    Some(obj) => {
                        let extruder = require_interface::<dyn PrinterExtruderApi>(&obj, &section)?;
                        extruder.get_heater()?
                    }
                    None => {
                        if temp <= 0.0 {
                            return Ok(());
                        }
                        return Err(KlippyError::command("Extruder not configured"));
                    }
                }
            }
            None => {
                let toolhead = self.toolhead()?;
                toolhead.get_extruder().get_heater()?
            }
        };
        let pheaters = self.printers_heaters()?;
        let heater_concrete = downcast_arc::<Heater>(&heater).ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "printer object 'heater' does not provide Heater".to_string(),
        })?;
        pheaters.set_temperature(&heater_concrete, temp, wait)
    }

    /// `cmd_ACTIVATE_EXTRUDER` (`extruder.py:281`).
    pub fn cmd_activate_extruder(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let toolhead = self.toolhead()?;
        let active = toolhead.get_extruder();
        let self_handle: ThExtruder = self_self_handle(self);
        if std::ptr::addr_eq(Arc::as_ptr(&active), Arc::as_ptr(&self_handle)) {
            gcmd.respond_info(&fmt("Extruder %s already active", &[PctValue::Str(self.name.clone())]), true);
            return Ok(());
        }
        gcmd.respond_info(&fmt("Activating extruder %s", &[PctValue::Str(self.name.clone())]), true);
        toolhead.flush_step_generation()?;
        toolhead.set_extruder(self_handle, self.last_position());
        self.printer.send_event(EV_EXTRUDER_ACTIVATE_EXTRUDER, &[])?;
        Ok(())
    }
}

/// The `Arc<dyn PrinterExtruderApi>` handle of an `Arc<PrinterExtruder>`.
fn self_self_handle(me: &Arc<PrinterExtruder>) -> ThExtruder {
    me.clone() as ThExtruder
}

impl ExtraAxisApi for PrinterExtruder {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        Some("E".to_string())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        Some(self.trapq.clone())
    }
    /// `check_move` (`extruder.py:208`).
    fn check_move(&self, mv: &dyn MoveApi, ea_index: usize) -> KResult<()> {
        if !HeaterApi::can_extrude(&*self.heater) {
            return Err(KlippyError::command(
                "Extrude below minimum temp\nSee the 'min_extrude_temp' config option for details",
            ));
        }
        let axes_r = mv.axes_r();
        let axes_d = mv.axes_d();
        let axis_r = axes_r.0[ea_index];
        let axis_d = axes_d.0[ea_index];
        if (axes_d.0[0] == 0.0 && axes_d.0[1] == 0.0) || axis_r < 0.0 {
            if axis_d.abs() > self.max_e_dist {
                return Err(KlippyError::command(fmt(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\nSee the 'max_extrude_only_distance' config option for details",
                    &[PctValue::Float(axis_d), PctValue::Float(self.max_e_dist)],
                )));
            }
            let inv_extrude_r = div_f64(1.0, axis_r.abs())?;
            mv.limit_speed(self.max_e_velocity * inv_extrude_r, self.max_e_accel * inv_extrude_r);
        } else if axis_r > self.max_extrude_ratio {
            if axis_d <= self.nozzle_diameter * self.max_extrude_ratio {
                return Ok(());
            }
            let area = axis_r * self.filament_area;
            log::debug(&fmt(
                "Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                &[
                    PctValue::Float(axis_r),
                    PctValue::Float(self.max_extrude_ratio),
                    PctValue::Float(area),
                    PctValue::Float(mv.move_d()),
                ],
            ));
            return Err(KlippyError::command(fmt(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\nSee the 'max_extrude_cross_section' config option for details",
                &[PctValue::Float(area), PctValue::Float(self.max_extrude_ratio * self.filament_area)],
            )));
        }
        Ok(())
    }
    /// `calc_junction` (`extruder.py:236`).
    fn calc_junction(&self, prev_move: &dyn MoveApi, mv: &dyn MoveApi, ea_index: usize) -> f64 {
        let diff_r = mv.axes_r().0[ea_index] - prev_move.axes_r().0[ea_index];
        if diff_r != 0.0 {
            // `diff_r != 0` matches Python truthiness (`nan` is truthy).
            // `abs(diff_r)` is never zero here, so the Python division never
            // raises; the unwrap arms only mirror `x / 0.0` (`inf`) and a
            // domain error in `** 2` (`nan`) for inputs that cannot occur.
            let ratio = if diff_r.is_nan() {
                f64::NAN
            } else {
                div_f64(self.instant_corner_v, diff_r.abs()).unwrap_or(f64::INFINITY)
            };
            pow_f64(ratio, 2.0).unwrap_or(f64::NAN)
        } else {
            mv.max_cruise_v2()
        }
    }
    /// `process_move` (`extruder.py:241`).
    fn process_move(&self, print_time: f64, mv: &dyn MoveApi, ea_index: usize) {
        let axis_r = mv.axes_r().0[ea_index];
        let accel = mv.accel() * axis_r;
        let start_v = mv.start_v() * axis_r;
        let cruise_v = mv.cruise_v() * axis_r;
        let can_pressure_advance = axis_r > 0.0 && (mv.axes_d().0[0] != 0.0 || mv.axes_d().0[1] != 0.0);
        (self.trapq_append)(
            &self.trapq,
            print_time,
            mv.accel_t(),
            mv.cruise_t(),
            mv.decel_t(),
            mv.start_pos().0[ea_index],
            0.0,
            0.0,
            1.0,
            if can_pressure_advance { 1.0 } else { 0.0 },
            0.0,
            start_v,
            cruise_v,
            accel,
        );
        self.last_position.set(mv.end_pos().0[ea_index]);
    }
}

impl PrinterExtruderApi for PrinterExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        let heater: ObjectRef = self.heater.clone();
        Ok(heater)
    }
    fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        match &self.extruder_stepper {
            None => Ok(0.0),
            Some(stepper) => stepper.find_past_position(print_time),
        }
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(HeaterApi::stats(&*self.heater, eventtime))
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        self.extruder_stepper_handle()
    }
}

impl PrinterObject for PrinterExtruder {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterExtruderApi, dyn ExtraAxisApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.status(eventtime))
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(HeaterApi::stats(&*self.heater, eventtime))
    }
}

// ---------------------------------------------------------------------------
// DummyExtruder
// ---------------------------------------------------------------------------

/// Port of `DummyExtruder` (`klippy/kinematics/extruder.py:292`).
pub struct DummyExtruder {
    #[allow(dead_code)]
    printer: Printer,
}

impl DummyExtruder {
    /// `DummyExtruder.__init__(printer)`.
    pub fn new(printer: &Printer) -> Arc<Self> {
        Arc::new(Self { printer: printer.clone() })
    }
}

impl ExtraAxisApi for DummyExtruder {
    fn get_name(&self) -> String {
        String::new()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        Some("E".to_string())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    /// `check_move` (`extruder.py:295`).
    fn check_move(&self, mv: &dyn MoveApi, _ea_index: usize) -> KResult<()> {
        Err(mv.move_error("Extrude when no extruder present".to_string()))
    }
    /// `calc_junction` (`extruder.py:299`).
    fn calc_junction(&self, _prev_move: &dyn MoveApi, mv: &dyn MoveApi, _ea_index: usize) -> f64 {
        mv.max_cruise_v2()
    }
    /// `DummyExtruder` has no `process_move` in Python (`AttributeError`
    /// on use); the toolhead's `check_move` always raises first, so this
    /// is unreachable in the frozen control flow.
    fn process_move(&self, _print_time: f64, _mv: &dyn MoveApi, _ea_index: usize) {}
}

impl PrinterExtruderApi for DummyExtruder {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::command("Extruder not configured"))
    }
    fn find_past_position(&self, _print_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

impl PrinterObject for DummyExtruder {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterExtruderApi, dyn ExtraAxisApi);
}

// ---------------------------------------------------------------------------
// Module entry points
// ---------------------------------------------------------------------------

/// `add_printer_objects(config)` (`extruder.py:310`).
pub fn add_printer_objects(config: &Config) -> KResult<()> {
    let printer: Printer = config.get_printer();
    for i in 0..99usize {
        let section = if i == 0 {
            "extruder".to_string()
        } else {
            fmt("extruder%d", &[PctValue::Int(pycompat::PyInt::from_i64(i as i64))])
        };
        if !config.has_section(&section) {
            break;
        }
        let pe = PrinterExtruder::new(&config.getsection(&section), i)?;
        let obj: ObjectRef = pe;
        printer.add_object(&section, obj)?;
    }
    Ok(())
}

/// Frozen module factory: this module only exposes
/// `add_printer_objects` in Python.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: Some(add_printer_objects),
    }
}

