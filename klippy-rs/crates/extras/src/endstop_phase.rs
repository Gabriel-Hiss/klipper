//! Port of the `endstop_phase` module (`klippy/extras/endstop_phase.py`).
//!
//! Endstop accuracy improvement via stepper phase tracking. `[endstop_phase <stepper>]`
//! sections ([`EndstopPhase`]) adjust the homed position by the stepper driver phase at
//! trigger time; the `[endstop_phase]` object ([`EndstopPhases`]) tracks the phase of every
//! homed stepper and implements `ENDSTOP_PHASE_CALIBRATE`.

use crate::tmc::TMCCommandHelperApi;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetIntListOpts, GetIntOpts, GetStrOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCode, GCodeApi, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_HOMING_HOME_RAILS_END, EV_KLIPPY_CONNECT,
};
use klippy_api::stepper::{Rail, Stepper};
use klippy_api::toolhead::{HomingStateApi, ToolHeadApi};
use klippy_api::{downcast_arc, lookup_interface, EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::libm;
use pycompat::text::{float_true_div, trunc_float_to_bigint, PyError};
use pycompat::value::type_name;
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `TRINAMIC_DRIVERS` (`klippy/extras/endstop_phase.py:9`).
pub const TRINAMIC_DRIVERS: [&str; 6] = [
    "tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160",
];

/// `cmd_ENDSTOP_PHASE_CALIBRATE_help` (`klippy/extras/endstop_phase.py:166`).
pub const CMD_ENDSTOP_PHASE_CALIBRATE_HELP: &str = "Calibrate stepper phase";

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.to_string(),
    }
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

fn index_error() -> KlippyError {
    internal("IndexError", "list index out of range")
}

/// Python `a % b` for ints (result takes the sign of `b`).
fn py_mod(a: i64, b: i64) -> KResult<i64> {
    if b == 0 {
        return Err(internal("ZeroDivisionError", "integer division or modulo by zero"));
    }
    let r = a.wrapping_rem(b);
    if r != 0 && ((r < 0) != (b < 0)) {
        Ok(r.wrapping_add(b))
    } else {
        Ok(r)
    }
}

/// Python `int(f)` / `math.ceil(f)` argument check: non-finite floats raise.
fn finite_to_int(f: f64) -> KResult<i64> {
    if f.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer"));
    }
    if f.is_infinite() {
        return Err(internal("OverflowError", "cannot convert float infinity to integer"));
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    Ok(digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX }))
}

/// `int(f)` as a float again (`int * float` converts the int back).
fn int_of_float_as_float(f: f64) -> KResult<f64> {
    if f.is_nan() {
        return Err(internal("ValueError", "cannot convert float NaN to integer"));
    }
    if f.is_infinite() {
        return Err(internal("OverflowError", "cannot convert float infinity to integer"));
    }
    trunc_float_to_bigint(f)
        .to_f64_exact()
        .ok_or_else(|| internal("OverflowError", "int too large to convert to float"))
}

/// `self.printer.get_start_args().get('debugoutput') is not None`.
fn debugoutput_is_set(printer: &Printer) -> KResult<bool> {
    match printer.get_start_args() {
        PyValue::Dict(d) => Ok(match d.get(&PyValue::Str("debugoutput".to_string())) {
            None | Some(PyValue::None) => false,
            Some(_) => true,
        }),
        other => Err(internal(
            "AttributeError",
            &format!("'{}' object has no attribute 'get'", type_name(&other)),
        )),
    }
}

fn py_int_value(v: Option<i64>) -> PyValue {
    match v {
        Some(n) => PyValue::Int(PyInt::from_i64(n)),
        None => PyValue::None,
    }
}

/// The `(homing_state, rails)` payload of `homing:home_rails_end`.
fn home_rails_args(args: &[EventArg]) -> KResult<(Arc<dyn HomingStateApi>, Vec<Rail>)> {
    if args.len() != 2 {
        return Err(internal(
            "TypeError",
            &format!(
                "handle_home_rails_end() takes 3 positional arguments but {} were given",
                args.len() + 1
            ),
        ));
    }
    let homing_state = args[0]
        .as_handle::<dyn HomingStateApi>()
        .ok_or_else(|| internal("TypeError", "homing:home_rails_end payload is not a Homing"))?;
    let rails = args[1]
        .as_handle_list::<dyn klippy_api::stepper::RailApi>()
        .ok_or_else(|| internal("TypeError", "homing:home_rails_end payload is not a list of rails"))?;
    Ok((homing_state, rails))
}

#[derive(Default)]
struct PhaseCalcState {
    phases: Option<i64>,
    tmc_module: Option<Arc<dyn TMCCommandHelperApi>>,
    phase_history: Option<Vec<i64>>,
    last_phase: Option<i64>,
    last_mcu_position: Option<i64>,
    is_primary: bool,
    stats_only: bool,
}

/// `PhaseCalc` (`klippy/extras/endstop_phase.py:13`): calculates the trigger phase of a
/// stepper motor.
pub struct PhaseCalc {
    printer: Printer,
    name: String,
    state: GCell<PhaseCalcState>,
}

impl PhaseCalc {
    /// `PhaseCalc.__init__(printer, name, phases=None)` (`endstop_phase.py:14-21`).
    pub fn new(printer: Printer, name: &str, phases: Option<i64>) -> Arc<PhaseCalc> {
        Arc::new(PhaseCalc {
            printer,
            name: name.to_string(),
            state: GCell::new(PhaseCalcState { phases, ..Default::default() }),
        })
    }

    /// `name` attribute.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `phases` attribute.
    pub fn phases(&self) -> Option<i64> {
        self.state.borrow().phases
    }

    /// `tmc_module is not None`.
    pub fn has_tmc_module(&self) -> bool {
        self.state.borrow().tmc_module.is_some()
    }

    /// `phase_history` attribute.
    pub fn phase_history(&self) -> Option<Vec<i64>> {
        self.state.borrow().phase_history.clone()
    }

    /// `last_phase` attribute.
    pub fn last_phase(&self) -> Option<i64> {
        self.state.borrow().last_phase
    }

    /// `last_mcu_position` attribute.
    pub fn last_mcu_position(&self) -> Option<i64> {
        self.state.borrow().last_mcu_position
    }

    /// `is_primary` attribute.
    pub fn is_primary(&self) -> bool {
        self.state.borrow().is_primary
    }

    /// `stats_only` attribute.
    pub fn stats_only(&self) -> bool {
        self.state.borrow().stats_only
    }

    /// `PhaseCalc.lookup_tmc()` (`endstop_phase.py:22-32`).
    pub fn lookup_tmc(&self) -> KResult<()> {
        for driver in TRINAMIC_DRIVERS {
            let driver_name = format!("{} {}", driver, self.name);
            let module = lookup_interface::<dyn TMCCommandHelperApi>(
                &*self.printer,
                &driver_name,
                LookupDefault::ReturnNone,
            )?;
            if let Some(module) = module {
                self.state.borrow_mut().tmc_module = Some(Arc::clone(&module));
                let phases_unset = self.state.borrow().phases.is_none();
                if phases_unset {
                    let (_phase_offset, phases) = module.get_phase_offset();
                    self.state.borrow_mut().phases = Some(phases);
                }
                break;
            }
        }
        let phases = self.state.borrow().phases;
        if let Some(phases) = phases {
            let count = usize::try_from(phases).unwrap_or(0);
            self.state.borrow_mut().phase_history = Some(vec![0; count]);
        }
        Ok(())
    }

    /// `PhaseCalc.convert_phase(driver_phase, driver_phases)` (`endstop_phase.py:33-35`).
    pub fn convert_phase(&self, driver_phase: i64, driver_phases: i64) -> KResult<i64> {
        let phases = self.state.borrow().phases.ok_or_else(|| {
            internal(
                "TypeError",
                "unsupported operand type(s) for *: 'float' and 'NoneType'",
            )
        })?;
        let ratio = float_true_div(driver_phase as f64, driver_phases as f64).map_err(map_py_error)?;
        let scaled = ratio * (phases as f64) + 0.5;
        if scaled.is_nan() {
            return Err(internal("ValueError", "cannot convert float NaN to integer"));
        }
        if scaled.is_infinite() {
            return Err(internal("OverflowError", "cannot convert float infinity to integer"));
        }
        let truncated = trunc_float_to_bigint(scaled);
        if phases == 0 {
            return Err(internal("ZeroDivisionError", "integer division or modulo by zero"));
        }
        let remainder = pycompat::int::int_mod(&truncated, &PyInt::from_i64(phases));
        remainder
            .0
            .to_string()
            .parse::<i64>()
            .map_err(|_| internal("OverflowError", "Python int too large to convert to C long"))
    }

    /// `PhaseCalc.calc_phase(stepper, trig_mcu_pos)` (`endstop_phase.py:36-49`).
    pub fn calc_phase(&self, _stepper: &Stepper, trig_mcu_pos: i64) -> KResult<i64> {
        let mut mcu_phase_offset: i64 = 0;
        let tmc_module = self.state.borrow().tmc_module.clone();
        if let Some(tmc_module) = tmc_module {
            let (offset, _phases) = tmc_module.get_phase_offset();
            match offset {
                Some(offset) => mcu_phase_offset = offset,
                None => {
                    if !debugoutput_is_set(&self.printer)? {
                        return Err(KlippyError::command(format!(
                            "Stepper {} phase unknown",
                            self.name
                        )));
                    }
                    mcu_phase_offset = 0;
                }
            }
        }
        let phases = self.state.borrow().phases.ok_or_else(|| {
            internal(
                "TypeError",
                "unsupported operand type(s) for %: 'int' and 'NoneType'",
            )
        })?;
        let phase = py_mod(trig_mcu_pos.wrapping_add(mcu_phase_offset), phases)?;
        let mut state = self.state.borrow_mut();
        let history = state
            .phase_history
            .as_mut()
            .ok_or_else(|| internal("TypeError", "'NoneType' object is not subscriptable"))?;
        let len = history.len() as i64;
        let index = if phase < 0 { phase + len } else { phase };
        if index < 0 || index >= len {
            return Err(index_error());
        }
        history[index as usize] += 1;
        state.last_phase = Some(phase);
        state.last_mcu_position = Some(trig_mcu_pos);
        Ok(phase)
    }
}

#[derive(Default)]
struct EndstopSettings {
    endstop_phase: Option<i64>,
    endstop_align_zero: bool,
    endstop_accuracy: Option<f64>,
    endstop_phase_accuracy: i64,
}

/// `EndstopPhase` (`klippy/extras/endstop_phase.py:52`): adjusted endstop trigger
/// positions of one stepper (`[endstop_phase <stepper>]`).
pub struct EndstopPhase {
    name: String,
    step_dist: f64,
    phases: i64,
    phase_calc: Arc<PhaseCalc>,
    settings: GCell<EndstopSettings>,
}

impl EndstopPhase {
    /// `EndstopPhase.__init__(config)` (`endstop_phase.py:53-93`).
    pub fn new(config: &Config) -> KResult<Arc<EndstopPhase>> {
        let printer = config.get_printer();
        let section_name = config.get_name();
        let name = section_name
            .split_whitespace()
            .skip(1)
            .collect::<Vec<_>>()
            .join(" ");
        // Obtain step_distance and microsteps from stepper config section
        let sconfig = config.getsection(&name);
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*printer, "toolhead", LookupDefault::Raise)?
            .ok_or_else(|| internal("AssertionError", "toolhead lookup with Raise never yields None"))?;
        let (rotation_dist, steps_per_rotation) =
            toolhead.stepper_factory().parse_step_distance(&sconfig, None, false)?;
        let step_dist = float_true_div(rotation_dist, steps_per_rotation).map_err(map_py_error)?;
        let microsteps = sconfig
            .getint("microsteps", GetIntOpts { note_valid: false, ..Default::default() })?
            .ok_or_else(|| internal("AssertionError", "required microsteps option yielded None"))?;
        let phases = microsteps.wrapping_mul(4);
        let phase_calc = PhaseCalc::new(Arc::clone(&printer), &name, Some(phases));
        let this = Arc::new(EndstopPhase {
            name,
            step_dist,
            phases,
            phase_calc,
            settings: GCell::new(EndstopSettings::default()),
        });
        // Register event handlers
        let connect_calc = Arc::clone(&this.phase_calc);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args: &[EventArg]| {
                connect_calc.lookup_tmc()?;
                Ok(PyValue::None)
            }),
        );
        let handler_self = Arc::clone(&this);
        printer.register_event_handler(
            EV_HOMING_HOME_RAILS_END,
            Box::new(move |args: &[EventArg]| {
                let (homing_state, rails) = home_rails_args(args)?;
                handler_self.handle_home_rails_end(&*homing_state, &rails)?;
                Ok(PyValue::None)
            }),
        );
        printer.load_object(config, "endstop_phase", LookupDefault::Raise)?;
        // Read config
        let trigger_phase = config.get(
            "trigger_phase",
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        let mut endstop_phase: Option<i64> = None;
        if let Some(trigger_phase) = trigger_phase {
            let values = config
                .getintlist(
                    "trigger_phase",
                    GetIntListOpts {
                        default: None,
                        sep: "/".to_string(),
                        count: Some(2),
                        note_valid: true,
                        ..Default::default()
                    },
                )?
                .unwrap_or_default();
            let (p, ps) = match values.as_slice() {
                [p, ps] => (*p, *ps),
                other => {
                    return Err(internal(
                        "ValueError",
                        &if other.len() < 2 {
                            format!("not enough values to unpack (expected 2, got {})", other.len())
                        } else {
                            "too many values to unpack (expected 2)".to_string()
                        },
                    ))
                }
            };
            if p >= ps {
                return Err(KlippyError::config(format!(
                    "Invalid trigger_phase '{}'",
                    trigger_phase
                )));
            }
            endstop_phase = Some(this.phase_calc.convert_phase(p, ps)?);
        }
        let endstop_align_zero = config
            .getboolean(
                "endstop_align_zero",
                GetBoolOpts { default: Some(Some(false)), ..Default::default() },
            )?
            .unwrap_or(false);
        let endstop_accuracy = config.getfloat(
            "endstop_accuracy",
            GetFloatOpts { default: Some(None), above: Some(0.0), ..Default::default() },
        )?;
        // Determine endstop accuracy
        let half_phases = phases.div_euclid(2);
        let mut endstop_phase_accuracy = match endstop_accuracy {
            None => half_phases - 1,
            Some(accuracy) => {
                let ratio = if endstop_phase.is_some() {
                    float_true_div(accuracy * 0.5, step_dist).map_err(map_py_error)?
                } else {
                    float_true_div(accuracy, step_dist).map_err(map_py_error)?
                };
                finite_to_int(libm::ceil(ratio))?
            }
        };
        if endstop_phase_accuracy >= half_phases {
            return Err(KlippyError::config(format!(
                "Endstop for {} is not accurate enough for stepper phase adjustment",
                this.name
            )));
        }
        if debugoutput_is_set(&printer)? {
            endstop_phase_accuracy = phases;
        }
        *this.settings.borrow_mut() = EndstopSettings {
            endstop_phase,
            endstop_align_zero,
            endstop_accuracy,
            endstop_phase_accuracy,
        };
        Ok(this)
    }

    /// `name` attribute.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `step_dist` attribute.
    pub fn step_dist(&self) -> f64 {
        self.step_dist
    }

    /// `phases` attribute.
    pub fn phases(&self) -> i64 {
        self.phases
    }

    /// `phase_calc` attribute.
    pub fn phase_calc(&self) -> Arc<PhaseCalc> {
        Arc::clone(&self.phase_calc)
    }

    /// `endstop_phase` attribute.
    pub fn endstop_phase(&self) -> Option<i64> {
        self.settings.borrow().endstop_phase
    }

    /// `endstop_align_zero` attribute.
    pub fn endstop_align_zero(&self) -> bool {
        self.settings.borrow().endstop_align_zero
    }

    /// `endstop_accuracy` attribute.
    pub fn endstop_accuracy(&self) -> Option<f64> {
        self.settings.borrow().endstop_accuracy
    }

    /// `endstop_phase_accuracy` attribute.
    pub fn endstop_phase_accuracy(&self) -> i64 {
        self.settings.borrow().endstop_phase_accuracy
    }

    /// `EndstopPhase.align_endstop(rail)` (`endstop_phase.py:94-104`).
    pub fn align_endstop(&self, rail: &Rail) -> KResult<f64> {
        let (align_zero, endstop_phase) = {
            let settings = self.settings.borrow();
            (settings.endstop_align_zero, settings.endstop_phase)
        };
        let endstop_phase = match endstop_phase {
            Some(phase) if align_zero => phase,
            _ => return Ok(0.0),
        };
        // Adjust the endstop position so 0.0 is always at a full step
        let microsteps = self.phases.div_euclid(4);
        let half_microsteps = microsteps.div_euclid(2);
        let phase_offset = ((py_mod(endstop_phase + half_microsteps, microsteps)? - half_microsteps)
            as f64)
            * self.step_dist;
        let full_step = (microsteps as f64) * self.step_dist;
        let pe = rail.get_homing_info().position_endstop;
        let steps = float_true_div(pe, full_step).map_err(map_py_error)? + 0.5;
        Ok(int_of_float_as_float(steps)? * full_step - pe + phase_offset)
    }

    /// `EndstopPhase.get_homed_offset(stepper, trig_mcu_pos)` (`endstop_phase.py:105-118`).
    pub fn get_homed_offset(&self, stepper: &Stepper, trig_mcu_pos: i64) -> KResult<f64> {
        let phase = self.phase_calc.calc_phase(stepper, trig_mcu_pos)?;
        let endstop_phase = self.settings.borrow().endstop_phase;
        let endstop_phase = match endstop_phase {
            None => {
                log::info(&format!("Setting {} endstop phase to {}", self.name, phase));
                self.settings.borrow_mut().endstop_phase = Some(phase);
                return Ok(0.0);
            }
            Some(endstop_phase) => endstop_phase,
        };
        let accuracy = self.settings.borrow().endstop_phase_accuracy;
        let mut delta = py_mod(phase.wrapping_sub(endstop_phase), self.phases)?;
        if delta >= self.phases.wrapping_sub(accuracy) {
            delta = delta.wrapping_sub(self.phases);
        } else if delta > accuracy {
            return Err(KlippyError::command(format!(
                "Endstop {} incorrect phase (got {} vs {})",
                self.name, phase, endstop_phase
            )));
        }
        Ok((delta as f64) * self.step_dist)
    }

    /// `EndstopPhase.handle_home_rails_end(homing_state, rails)` (`endstop_phase.py:119-127`).
    pub fn handle_home_rails_end(
        &self,
        homing_state: &dyn HomingStateApi,
        rails: &[Rail],
    ) -> KResult<()> {
        for rail in rails {
            let endstops = rail.get_endstops();
            let (endstop, _endstop_name) = endstops.first().ok_or_else(index_error)?;
            let steppers = endstop.get_steppers();
            let stepper = steppers.first().ok_or_else(index_error)?;
            if stepper.get_name(None) == self.name {
                let trig_mcu_pos = homing_state.get_trigger_position(&self.name)?;
                let align = self.align_endstop(rail)?;
                let offset = self.get_homed_offset(stepper, trig_mcu_pos)?;
                homing_state.set_stepper_adjustment(&self.name, align + offset);
                return Ok(());
            }
        }
        Ok(())
    }
}

klippy_api::printer_object!(EndstopPhase);

/// `EndstopPhases` (`klippy/extras/endstop_phase.py:130`): `ENDSTOP_PHASE_CALIBRATE`.
pub struct EndstopPhases {
    printer: Printer,
    gcode: GCode,
    tracking: GCell<IndexMap<String, Arc<PhaseCalc>>>,
}

impl EndstopPhases {
    /// `EndstopPhases.__init__(config)` (`endstop_phase.py:131-141`).
    pub fn new(config: &Config) -> KResult<Arc<EndstopPhases>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .ok_or_else(|| internal("AssertionError", "gcode lookup with Raise never yields None"))?;
        let this = Arc::new(EndstopPhases {
            printer: Arc::clone(&printer),
            gcode: Arc::clone(&gcode),
            tracking: GCell::new(IndexMap::new()),
        });
        // Register handlers
        let handler_self = Arc::clone(&this);
        printer.register_event_handler(
            EV_HOMING_HOME_RAILS_END,
            Box::new(move |args: &[EventArg]| {
                let (homing_state, rails) = home_rails_args(args)?;
                handler_self.handle_home_rails_end(&*homing_state, &rails)?;
                Ok(PyValue::None)
            }),
        );
        let cmd_self = Arc::clone(&this);
        gcode.register_command(
            "ENDSTOP_PHASE_CALIBRATE",
            Some(Box::new(move |gcmd: &GCmd| cmd_self.cmd_endstop_phase_calibrate(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_ENDSTOP_PHASE_CALIBRATE_HELP.to_string()),
            },
        )?;
        Ok(this)
    }

    /// The tracked `PhaseCalc` of a stepper (`tracking[stepper_name]`).
    pub fn tracked(&self, stepper_name: &str) -> Option<Arc<PhaseCalc>> {
        self.tracking.borrow().get(stepper_name).cloned()
    }

    /// Names in `tracking`, in insertion order.
    pub fn tracked_names(&self) -> Vec<String> {
        self.tracking.borrow().keys().cloned().collect()
    }

    /// `EndstopPhases.update_stepper(stepper, trig_mcu_pos, is_primary)`
    /// (`endstop_phase.py:142-163`).
    pub fn update_stepper(
        &self,
        stepper: &Stepper,
        trig_mcu_pos: i64,
        is_primary: bool,
    ) -> KResult<()> {
        let stepper_name = stepper.get_name(None);
        let existing = self.tracking.borrow().get(&stepper_name).cloned();
        let phase_calc = match existing {
            Some(phase_calc) => phase_calc,
            None => {
                // Check if stepper has an endstop_phase config section defined
                let mod_name = format!("endstop_phase {}", stepper_name);
                let module = self.printer.lookup_object(&mod_name, LookupDefault::ReturnNone)?;
                let phase_calc = match module {
                    Some(module) => downcast_arc::<EndstopPhase>(&module)
                        .ok_or_else(|| {
                            internal("AttributeError", "'object' object has no attribute 'phase_calc'")
                        })?
                        .phase_calc(),
                    None => {
                        // Create new PhaseCalc tracker
                        let phase_calc =
                            PhaseCalc::new(Arc::clone(&self.printer), &stepper_name, None);
                        phase_calc.state.borrow_mut().stats_only = true;
                        phase_calc.lookup_tmc()?;
                        phase_calc
                    }
                };
                self.tracking
                    .borrow_mut()
                    .insert(stepper_name.clone(), Arc::clone(&phase_calc));
                phase_calc
            }
        };
        if phase_calc.state.borrow().phase_history.is_none() {
            return Ok(());
        }
        if is_primary {
            phase_calc.state.borrow_mut().is_primary = true;
        }
        if phase_calc.stats_only() {
            phase_calc.calc_phase(stepper, trig_mcu_pos)?;
        }
        Ok(())
    }

    /// `EndstopPhases.handle_home_rails_end(homing_state, rails)` (`endstop_phase.py:164-170`).
    pub fn handle_home_rails_end(
        &self,
        homing_state: &dyn HomingStateApi,
        rails: &[Rail],
    ) -> KResult<()> {
        for rail in rails {
            let mut is_primary = true;
            for stepper in rail.get_steppers() {
                let sname = stepper.get_name(None);
                let trig_mcu_pos = homing_state.get_trigger_position(&sname)?;
                self.update_stepper(&stepper, trig_mcu_pos, is_primary)?;
                is_primary = false;
            }
        }
        Ok(())
    }

    /// `EndstopPhases.cmd_ENDSTOP_PHASE_CALIBRATE(gcmd)` (`endstop_phase.py:172-189`).
    pub fn cmd_endstop_phase_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let stepper_name = gcmd.get(
            "STEPPER",
            GetOpts { default: Some(PyValue::None), ..Default::default() },
        )?;
        let stepper_name = match stepper_name {
            PyValue::None => {
                self.report_stats()?;
                return Ok(());
            }
            other => other.str(),
        };
        let phase_calc = self.tracking.borrow().get(&stepper_name).cloned();
        let phase_calc = match phase_calc {
            Some(phase_calc) if phase_calc.state.borrow().phase_history.is_some() => phase_calc,
            _ => {
                return Err(KlippyError::command(format!(
                    "Stats not available for stepper {}",
                    stepper_name
                )))
            }
        };
        let (endstop_phase, phases) = self.generate_stats(&stepper_name, &phase_calc)?;
        if !phase_calc.is_primary() {
            return Ok(());
        }
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .ok_or_else(|| internal("AssertionError", "configfile lookup with Raise never yields None"))?;
        let section = format!("endstop_phase {}", stepper_name);
        configfile.remove_section(&section);
        configfile.set(&section, "trigger_phase", &format!("{}/{}", endstop_phase, phases))?;
        gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config\n\
             file with these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    /// `EndstopPhases.generate_stats(stepper_name, phase_calc)` (`endstop_phase.py:190-208`).
    pub fn generate_stats(&self, stepper_name: &str, phase_calc: &PhaseCalc) -> KResult<(i64, i64)> {
        let phase_history = phase_calc
            .phase_history()
            .ok_or_else(|| internal("TypeError", "unsupported operand type(s) for +: 'NoneType' and 'NoneType'"))?;
        let mut wph = phase_history.clone();
        wph.extend_from_slice(&phase_history);
        let phases = phase_history.len() as i64;
        let half_phases = phases / 2;
        let mut res: Vec<(i64, i64)> = Vec::new();
        for i in 0..phases {
            let phase = i + half_phases;
            let mut cost: i64 = 0;
            for j in i..(i + phases) {
                cost = cost.wrapping_add(wph[j as usize].wrapping_mul((j - phase).abs()));
            }
            res.push((cost, phase));
        }
        res.sort();
        let best = res.first().ok_or_else(index_error)?.1;
        let mut found: Vec<i64> = Vec::new();
        for j in (best - half_phases)..(best + half_phases) {
            let count = if j < 0 {
                wph.get((wph.len() as i64 + j) as usize)
            } else {
                wph.get(j as usize)
            };
            if *count.ok_or_else(index_error)? != 0 {
                found.push(j);
            }
        }
        let best_phase = py_mod(best, phases)?;
        let lo = py_mod(*found.first().ok_or_else(index_error)?, phases)?;
        let hi = py_mod(*found.last().ok_or_else(index_error)?, phases)?;
        self.gcode.respond_info(
            &format!(
                "{}: trigger_phase={}/{} (range {} to {})",
                stepper_name, best_phase, phases, lo, hi
            ),
            true,
        );
        Ok((best_phase, phases))
    }

    /// `EndstopPhases.report_stats()` (`endstop_phase.py:209-217`).
    pub fn report_stats(&self) -> KResult<()> {
        let entries: Vec<(String, Arc<PhaseCalc>)> = self
            .tracking
            .borrow()
            .iter()
            .map(|(name, calc)| (name.clone(), Arc::clone(calc)))
            .collect();
        if entries.is_empty() {
            self.gcode
                .respond_info("No steppers found. (Be sure to home at least once.)", true);
            return Ok(());
        }
        let mut names: Vec<&String> = entries.iter().map(|(name, _)| name).collect();
        names.sort();
        for stepper_name in names {
            let phase_calc = entries
                .iter()
                .find(|(name, _)| name == stepper_name)
                .map(|(_, calc)| Arc::clone(calc))
                .ok_or_else(|| internal("KeyError", "tracking entry vanished"))?;
            if !phase_calc.is_primary() {
                continue;
            }
            self.generate_stats(stepper_name, &phase_calc)?;
        }
        Ok(())
    }
}

impl PrinterObject for EndstopPhases {
    klippy_api::printer_object_base!();

    /// `EndstopPhases.get_status(eventtime)` (`endstop_phase.py:218-225`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut last_home: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (name, phase_calc) in self.tracking.borrow().iter() {
            if phase_calc.state.borrow().phase_history.is_none() {
                continue;
            }
            let mut entry: IndexMap<PyValue, PyValue> = IndexMap::new();
            entry.insert(PyValue::Str("phase".to_string()), py_int_value(phase_calc.last_phase()));
            entry.insert(PyValue::Str("phases".to_string()), py_int_value(phase_calc.phases()));
            entry.insert(
                PyValue::Str("mcu_position".to_string()),
                py_int_value(phase_calc.last_mcu_position()),
            );
            last_home.insert(PyValue::Str(name.clone()), PyValue::Dict(entry));
        }
        let mut status: IndexMap<PyValue, PyValue> = IndexMap::new();
        status.insert(PyValue::Str("last_home".to_string()), PyValue::Dict(last_home));
        Some(PyValue::Dict(status))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/endstop_phase.py:227`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(EndstopPhase::new(config)? as ObjectRef))
}

/// `load_config(config)` (`klippy/extras/endstop_phase.py:230`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(EndstopPhases::new(config)? as ObjectRef))
}

/// Module entry points of `endstop_phase`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
