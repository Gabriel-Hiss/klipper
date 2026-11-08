//! Port of the `stepper_enable` module (`klippy/extras/stepper_enable.py`).
//!
//! Tracks enable pins for stepper motor drivers. Each stepper gets an
//! [`EnableTracking`] that enables the (possibly shared) enable pin on
//! first motion and disables it on demand. The G-code commands `M18`,
//! `M84` and `SET_STEPPER_ENABLE` turn the motors off or toggle an
//! individual stepper.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_GCODE_REQUEST_RESTART,
    EV_STEPPER_ENABLE_MOTOR_OFF,
};
use klippy_api::stepper::{Stepper, StepperRegistryApi};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::gcode::{GetIntOpts, GetOpts, GCmd, GCodeApi, RegisterCommandOpts};
use klippy_api::mcu::{DigitalOutPin, LookupPinOpts, Pin, PinsApi};
use klippy_api::{GCell, lookup_interface, EventArg, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::{Arc, Weak};

/// `DISABLE_STALL_TIME` (`klippy/extras/stepper_enable.py:8`).
pub const DISABLE_STALL_TIME: f64 = 0.100;

/// `PrinterStepperEnable.cmd_SET_STEPPER_ENABLE_help`
/// (`klippy/extras/stepper_enable.py:130`).
pub const CMD_SET_STEPPER_ENABLE_HELP: &str = "Enable/disable individual stepper by name";

/// `StepperEnablePin` (`klippy/extras/stepper_enable.py:11`): an enable
/// pin shared between steppers, enabled while `enable_count > 0`.
pub struct StepperEnablePin {
    mcu_enable: Option<DigitalOutPin>,
    enable_count: GCell<i64>,
    is_dedicated: GCell<bool>,
}

impl StepperEnablePin {
    /// `StepperEnablePin.__init__(mcu_enable, enable_count)`
    /// (`klippy/extras/stepper_enable.py:12`).
    pub fn new(mcu_enable: Option<DigitalOutPin>, enable_count: i64) -> StepperEnablePin {
        StepperEnablePin {
            mcu_enable,
            enable_count: GCell::new(enable_count),
            is_dedicated: GCell::new(true),
        }
    }

    /// `StepperEnablePin.set_enable(print_time)`
    /// (`klippy/extras/stepper_enable.py:16`).
    pub fn set_enable(&self, print_time: f64) {
        if self.enable_count.get() == 0 {
            if let Some(pin) = &self.mcu_enable {
                pin.set_digital(print_time, true);
            }
        }
        self.enable_count.set(self.enable_count.get() + 1);
    }

    /// `StepperEnablePin.set_disable(print_time)`
    /// (`klippy/extras/stepper_enable.py:20`).
    pub fn set_disable(&self, print_time: f64) {
        self.enable_count.set(self.enable_count.get() - 1);
        if self.enable_count.get() == 0 {
            if let Some(pin) = &self.mcu_enable {
                pin.set_digital(print_time, false);
            }
        }
    }

    /// `StepperEnablePin.is_dedicated` (`klippy/extras/stepper_enable.py:15`).
    pub fn is_dedicated(&self) -> bool {
        self.is_dedicated.get()
    }

    fn set_dedicated(&self, value: bool) {
        self.is_dedicated.set(value);
    }

    /// `StepperEnablePin.enable_count` (`klippy/extras/stepper_enable.py:14`).
    pub fn enable_count(&self) -> i64 {
        self.enable_count.get()
    }
}
/// `EnableTracking` state callback (`tmc.py:470`):
/// `callback(print_time, is_enable)`.
pub type StateCallback = Box<dyn FnMut(f64, bool) + Send>;

/// `EnableTracking` (`klippy/extras/stepper_enable.py:45`): per-stepper
/// enable state, with state callbacks and a stepper active callback that
/// turns the motor on at the first motion.
pub struct EnableTracking {
    stepper: Stepper,
    enable: Arc<StepperEnablePin>,
    callbacks: GCell<Vec<StateCallback>>,
    is_enabled: GCell<bool>,
    self_weak: GCell<Option<Weak<EnableTracking>>>,
}

impl EnableTracking {
    /// `EnableTracking.__init__(stepper, enable)`
    /// (`klippy/extras/stepper_enable.py:46`).
    pub fn new(stepper: Stepper, enable: Arc<StepperEnablePin>) -> KResult<Arc<EnableTracking>> {
        let tracking = Arc::new(EnableTracking {
            stepper,
            enable,
            callbacks: GCell::new(Vec::new()),
            is_enabled: GCell::new(false),
            self_weak: GCell::new(None),
        });
        tracking.self_weak.with_mut(|weak| *weak = Some(Arc::downgrade(&tracking)));
        tracking.register_active_callback()?;
        Ok(tracking)
    }

    fn register_active_callback(&self) -> KResult<()> {
        let weak = self.self_weak.borrow().clone().expect("EnableTracking built by new()");
        self.stepper.add_active_callback(Box::new(move |print_time| {
            if let Some(tracking) = weak.upgrade() {
                tracking.motor_enable(print_time);
            }
        }))
    }

    /// `EnableTracking.register_state_callback(callback)`
    /// (`klippy/extras/stepper_enable.py:52`).
    pub fn register_state_callback(&self, callback: StateCallback) {
        self.callbacks.borrow_mut().push(callback);
    }

    /// `for cb in self.callbacks: cb(print_time, enabled)`. Python iterates
    /// the live list, so a callback registered during the pass runs in the
    /// same pass; the borrow is never held across a callback.
    fn run_state_callbacks(&self, print_time: f64, enabled: bool) {
        let mut callbacks = std::mem::take(&mut *self.callbacks.borrow_mut());
        let mut next = 0;
        loop {
            while next < callbacks.len() {
                (callbacks[next])(print_time, enabled);
                next += 1;
            }
            let added: Vec<StateCallback> = self.callbacks.borrow_mut().drain(..).collect();
            if added.is_empty() {
                break;
            }
            callbacks.extend(added);
        }
        *self.callbacks.borrow_mut() = callbacks;
    }

    /// `EnableTracking.motor_enable(print_time)`
    /// (`klippy/extras/stepper_enable.py:54`).
    pub fn motor_enable(&self, print_time: f64) {
        if !self.is_enabled.get() {
            self.run_state_callbacks(print_time, true);
            self.enable.set_enable(print_time);
            self.is_enabled.set(true);
        }
    }

    /// `EnableTracking.motor_disable(print_time)`
    /// (`klippy/extras/stepper_enable.py:60`).
    pub fn motor_disable(&self, print_time: f64) -> KResult<()> {
        if self.is_enabled.get() {
            self.run_state_callbacks(print_time, false);
            self.enable.set_disable(print_time);
            self.is_enabled.set(false);
            // Enable stepper on future stepper movement
            self.register_active_callback()?;
        }
        Ok(())
    }

    /// `EnableTracking.is_motor_enabled()`
    /// (`klippy/extras/stepper_enable.py:68`).
    pub fn is_motor_enabled(&self) -> bool {
        self.is_enabled.get()
    }

    /// `EnableTracking.has_dedicated_enable()`
    /// (`klippy/extras/stepper_enable.py:70`).
    pub fn has_dedicated_enable(&self) -> bool {
        self.enable.is_dedicated()
    }
}

/// The `EnableTracking` state-callback contract consumed by `tmc`
/// (`register_state_callback`, `is_motor_enabled`, `has_dedicated_enable`).
pub trait EnableTrackingApi: Send + Sync + 'static {
    /// `EnableTracking.register_state_callback(callback)`
    /// (`klippy/extras/stepper_enable.py:52`).
    fn register_state_callback(&self, callback: StateCallback);
    /// `EnableTracking.is_motor_enabled()` (`klippy/extras/stepper_enable.py:68`).
    fn is_motor_enabled(&self) -> bool;
    /// `EnableTracking.has_dedicated_enable()` (`klippy/extras/stepper_enable.py:70`).
    fn has_dedicated_enable(&self) -> bool;
}

/// The `PrinterStepperEnable` service (`klippy/extras/stepper_enable.py:74`):
/// the consumer-facing contract used by `stepper.py` (`register_stepper`),
/// `force_move` / `manual_stepper` (`set_motors_enable`), `homing`
/// (`motor_off`), `tmc` (`lookup_enable`) and `controller_fan`.
pub trait PrinterStepperEnableApi: Send + Sync + 'static {
    /// `PrinterStepperEnable.register_stepper(config, mcu_stepper)`
    /// (`klippy/extras/stepper_enable.py:84`).
    fn register_stepper(&self, config: &Config, stepper: &Stepper) -> KResult<()>;
    /// `PrinterStepperEnable.set_motors_enable(stepper_names, enable)`
    /// (`klippy/extras/stepper_enable.py:92`).
    fn set_motors_enable(&self, stepper_names: &[String], enable: bool) -> KResult<bool>;
    /// `PrinterStepperEnable.motor_off()` (`klippy/extras/stepper_enable.py:116`).
    fn motor_off(&self) -> KResult<()>;
    /// `PrinterStepperEnable.lookup_enable(name)`
    /// (`klippy/extras/stepper_enable.py:139`).
    fn lookup_enable(&self, name: &str) -> KResult<Arc<EnableTracking>>;
    /// `PrinterStepperEnable.get_steppers()`
    /// (`klippy/extras/stepper_enable.py:143`).
    fn get_steppers(&self) -> Vec<String>;
    /// `PrinterStepperEnable.get_status(eventtime)`
    /// (`klippy/extras/stepper_enable.py:121`).
    fn get_status(&self, eventtime: f64) -> PyValue;
}

/// `PrinterStepperEnable` (`klippy/extras/stepper_enable.py:74`).
pub struct PrinterStepperEnable {
    printer: Printer,
    enable_lines: GCell<IndexMap<String, Arc<EnableTracking>>>,
    shared_pins: GCell<IndexMap<String, Arc<StepperEnablePin>>>,
    self_weak: GCell<Option<Weak<PrinterStepperEnable>>>,
}

impl PrinterStepperEnable {
    /// `PrinterStepperEnable.__init__(config)`
    /// (`klippy/extras/stepper_enable.py:75`).
    pub fn new(config: &Config) -> KResult<Arc<PrinterStepperEnable>> {
        let printer = config.get_printer();
        let this = Arc::new(PrinterStepperEnable {
            printer: printer.clone(),
            enable_lines: GCell::new(IndexMap::new()),
            shared_pins: GCell::new(IndexMap::new()),
            self_weak: GCell::new(None),
        });
        this.self_weak.with_mut(|weak| *weak = Some(Arc::downgrade(&this)));
        printer.register_event_handler(EV_GCODE_REQUEST_RESTART, {
            let handler = this.clone();
            Box::new(move |_params: &[EventArg]| {
                handler.motor_off()?;
                Ok(PyValue::None)
            })
        });
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("'gcode' missing raises");
        let m18 = this.clone();
        gcode.register_command(
            "M18",
            Some(Box::new(move |gcmd: &GCmd| m18.cmd_m18(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        let m84 = this.clone();
        gcode.register_command(
            "M84",
            Some(Box::new(move |gcmd: &GCmd| m84.cmd_m18(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        Ok(this)
    }

    /// `PrinterStepperEnable.register_stepper(config, mcu_stepper)`
    /// (`klippy/extras/stepper_enable.py:84`).
    pub fn register_stepper(&self, config: &Config, stepper: &Stepper) -> KResult<()> {
        let name = stepper.get_name(None);
        let gcode = lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
            .expect("'gcode' missing raises");
        let tracker = self.self_weak.borrow().clone().and_then(|weak| weak.upgrade())
            .expect("PrinterStepperEnable built by new()");
        gcode.register_mux_command(
            "SET_STEPPER_ENABLE",
            "STEPPER",
            Some(&name),
            Box::new(move |gcmd: &GCmd| tracker.cmd_set_stepper_enable(gcmd)),
            Some(CMD_SET_STEPPER_ENABLE_HELP),
        )?;
        let enable_pin = config.get(
            "enable_pin",
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        let enable = self.setup_enable_pin(enable_pin.as_deref())?;
        let tracking = EnableTracking::new(stepper.clone(), enable)?;
        self.enable_lines.borrow_mut().insert(name, tracking);
        Ok(())
    }

    /// `setup_enable_pin(printer, pin)` (`klippy/extras/stepper_enable.py:25`).
    ///
    /// The Python module stores the shared pin in `pin_params['class']`;
    /// [`PinParams`] has no `class` slot, so the shared pins are kept in
    /// [`PrinterStepperEnable::shared_pins`] keyed by
    /// `"<chip_name>:<pin>"`, the same key `pins.py` uses for pin sharing.
    fn setup_enable_pin(&self, pin: Option<&str>) -> KResult<Arc<StepperEnablePin>> {
        let Some(pin) = pin else {
            // No enable line (stepper always enabled)
            let enable = StepperEnablePin::new(None, 9999);
            enable.set_dedicated(false);
            return Ok(Arc::new(enable));
        };
        let pins = lookup_interface::<dyn PinsApi>(&*self.printer, "pins", LookupDefault::Raise)?
            .expect("'pins' missing raises");
        let pin_params = pins.lookup_pin(
            pin,
            LookupPinOpts {
                can_invert: true,
                can_pullup: false,
                share_type: Some("stepper_enable".to_string()),
            },
        )?;
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        let cached = {
            let shared = self.shared_pins.borrow();
            shared.get(&share_name).cloned()
        };
        if let Some(enable) = cached {
            // Shared enable line
            enable.set_dedicated(false);
            return Ok(enable);
        }
        let chip = pin_params.chip.clone();
        let pin_obj = chip.setup_pin("digital_out", pin_params)?;
        let Pin::DigitalOut(mcu_enable) = pin_obj else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "setup_pin('digital_out') did not return a digital output pin".to_string(),
            });
        };
        mcu_enable.setup_max_duration(0.);
        let enable = Arc::new(StepperEnablePin::new(Some(mcu_enable), 0));
        self.shared_pins.borrow_mut().insert(share_name, enable.clone());
        Ok(enable)
    }

    /// `PrinterStepperEnable.set_motors_enable(stepper_names, enable)`
    /// (`klippy/extras/stepper_enable.py:92`).
    pub fn set_motors_enable(&self, stepper_names: &[String], enable: bool) -> KResult<bool> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("'toolhead' missing raises");
        // Flush steps to ensure all auto enable callbacks invoked
        toolhead.flush_step_generation()?;
        let mut print_time: Option<f64> = None;
        let mut did_change = false;
        for stepper_name in stepper_names {
            let tracking = {
                let lines = self.enable_lines.borrow();
                lines.get(stepper_name).cloned()
            };
            let tracking = tracking.ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: pycompat::value::repr_key(stepper_name),
            })?;
            if tracking.is_motor_enabled() == enable {
                continue;
            }
            if print_time.is_none() {
                // Dwell for sufficient delay from any previous auto enable
                if !enable {
                    toolhead.dwell(DISABLE_STALL_TIME);
                }
                print_time = Some(toolhead.get_last_move_time());
            }
            let print_time = print_time.expect("set above");
            if enable {
                tracking.motor_enable(print_time);
            } else {
                tracking.motor_disable(print_time)?;
            }
            did_change = true;
        }
        // Dwell to ensure sufficient delay prior to a future auto enable
        if did_change && !enable {
            toolhead.dwell(DISABLE_STALL_TIME);
        }
        Ok(did_change)
    }

    /// `PrinterStepperEnable.motor_off()` (`klippy/extras/stepper_enable.py:116`).
    pub fn motor_off(&self) -> KResult<()> {
        self.set_motors_enable(&self.get_steppers(), false)?;
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("'toolhead' missing raises");
        toolhead.get_kinematics().clear_homing_state("xyz");
        self.printer.send_event(EV_STEPPER_ENABLE_MOTOR_OFF, &[])?;
        Ok(())
    }

    /// `PrinterStepperEnable.get_status(eventtime)`
    /// (`klippy/extras/stepper_enable.py:121`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut steppers = IndexMap::new();
        for (name, tracking) in self.enable_lines.borrow().iter() {
            steppers.insert(
                PyValue::Str(name.clone()),
                PyValue::Bool(tracking.is_motor_enabled()),
            );
        }
        let mut out = IndexMap::new();
        out.insert(PyValue::Str("steppers".to_string()), PyValue::Dict(steppers));
        PyValue::Dict(out)
    }

    /// `PrinterStepperEnable._handle_request_restart(print_time)`
    /// (`klippy/extras/stepper_enable.py:125`).
    pub fn handle_request_restart(&self, _print_time: f64) -> KResult<()> {
        self.motor_off()
    }

    /// `PrinterStepperEnable.cmd_M18(gcmd)` (`klippy/extras/stepper_enable.py:127`).
    pub fn cmd_m18(&self, _gcmd: &GCmd) -> KResult<()> {
        // Turn off motors
        self.motor_off()
    }

    /// `PrinterStepperEnable.cmd_SET_STEPPER_ENABLE(gcmd)`
    /// (`klippy/extras/stepper_enable.py:131`).
    pub fn cmd_set_stepper_enable(&self, gcmd: &GCmd) -> KResult<()> {
        let stepper_name = gcmd.get("STEPPER", GetOpts::default())?.str();
        let stepper_enable = gcmd
            .get_int("ENABLE", GetIntOpts { default: Some(Some(1)), ..Default::default() })?
            .unwrap_or(1);
        self.set_motors_enable(std::slice::from_ref(&stepper_name), stepper_enable != 0)?;
        if stepper_enable != 0 {
            klippy_api::log::info(
                &percent_format(
                    "%s has been manually enabled",
                    &[PctValue::Str(stepper_name.clone())],
                ).map_err(|e| KlippyError::Internal {
                    class_name: e.kind.to_string(),
                    message: e.display(),
                })?,
            );
        } else {
            klippy_api::log::info(
                &percent_format(
                    "%s has been manually disabled",
                    &[PctValue::Str(stepper_name.clone())],
                ).map_err(|e| KlippyError::Internal {
                    class_name: e.kind.to_string(),
                    message: e.display(),
                })?,
            );
        }
        Ok(())
    }

    /// `PrinterStepperEnable.lookup_enable(name)`
    /// (`klippy/extras/stepper_enable.py:139`).
    pub fn lookup_enable(&self, name: &str) -> KResult<Arc<EnableTracking>> {
        let lines = self.enable_lines.borrow();
        if let Some(tracking) = lines.get(name) {
            return Ok(tracking.clone());
        }
        Err(KlippyError::Config(format!("Unknown stepper '{name}'")))
    }

    /// `PrinterStepperEnable.get_steppers()`
    /// (`klippy/extras/stepper_enable.py:143`).
    pub fn get_steppers(&self) -> Vec<String> {
        self.enable_lines.borrow().keys().cloned().collect()
    }
}

impl EnableTrackingApi for EnableTracking {
    fn register_state_callback(&self, callback: StateCallback) {
        EnableTracking::register_state_callback(self, callback)
    }

    fn is_motor_enabled(&self) -> bool {
        EnableTracking::is_motor_enabled(self)
    }

    fn has_dedicated_enable(&self) -> bool {
        EnableTracking::has_dedicated_enable(self)
    }
}

impl PrinterObject for PrinterStepperEnable {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterStepperEnableApi, dyn StepperRegistryApi);

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(PrinterStepperEnableApi::get_status(self, eventtime))
    }
}

impl PrinterStepperEnableApi for PrinterStepperEnable {
    fn register_stepper(&self, config: &Config, stepper: &Stepper) -> KResult<()> {
        PrinterStepperEnable::register_stepper(self, config, stepper)
    }

    fn set_motors_enable(&self, stepper_names: &[String], enable: bool) -> KResult<bool> {
        PrinterStepperEnable::set_motors_enable(self, stepper_names, enable)
    }

    fn motor_off(&self) -> KResult<()> {
        PrinterStepperEnable::motor_off(self)
    }

    fn lookup_enable(&self, name: &str) -> KResult<Arc<EnableTracking>> {
        PrinterStepperEnable::lookup_enable(self, name)
    }

    fn get_steppers(&self) -> Vec<String> {
        PrinterStepperEnable::get_steppers(self)
    }

    fn get_status(&self, eventtime: f64) -> PyValue {
        PrinterStepperEnable::get_status(self, eventtime)
    }
}

impl StepperRegistryApi for PrinterStepperEnable {
    fn register_stepper(&self, config: &Config, mcu_stepper: Stepper) -> KResult<()> {
        PrinterStepperEnable::register_stepper(self, config, &mcu_stepper)
    }
}

/// `load_config(config)` (`klippy/extras/stepper_enable.py:146`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterStepperEnable::new(config)? as ObjectRef))
}

/// Module entry point for the `stepper_enable` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
