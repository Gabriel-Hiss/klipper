//! Port of the `extruder_stepper` module (`klippy/extras/extruder_stepper.py`).
//!
//! Thin wrapper around a kinematics `ExtruderStepper` for a
//! `[extruder_stepper <name>]` section: it owns the target extruder name
//! (`extruder` option) and delegates `sync_to_extruder`, `find_past_position`
//! and `get_status` to the inner stepper. The inner stepper itself (stepper
//! pins, pressure advance, chelper kinematics, G-code commands) is owned by
//! the `kinematics` crate (`kinematics/extruder.py`); it is not constructed
//! here because `extras` cannot depend on `kinematics` (the reverse
//! dependency already exists). The constructor therefore receives the
//! already built [`ExtruderStepper`] handle, exactly like the Python
//! `extruder.ExtruderStepper(config)` call it replaces.

use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{EV_KLIPPY_CONNECT, ModuleFactory};
use klippy_api::toolhead::{ExtruderStepper, ExtruderStepperApi};
use klippy_api::{EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use std::sync::Arc;

/// `klippy/extras/extruder_stepper.py:PrinterExtruderStepper`.
pub struct PrinterExtruderStepper {
    extruder_stepper: ExtruderStepper,
    extruder_name: String,
}

impl PrinterExtruderStepper {
    /// `PrinterExtruderStepper.__init__(config)`
    /// (`klippy/extras/extruder_stepper.py:10`).
    ///
    /// Python constructs the inner `extruder.ExtruderStepper(config)` first
    /// and reads the `extruder` option second; here the caller builds the
    /// inner stepper before the call, so the option read still follows the
    /// construction. The `klippy:connect` handler syncs the inner stepper
    /// to the named extruder, exactly like `handle_connect`.
    pub fn new(config: &Config, extruder_stepper: ExtruderStepper) -> KResult<Self> {
        let printer = config.get_printer();
        let extruder_name = config
            .get("extruder", GetStrOpts::default())?
            .ok_or_else(|| {
                KlippyError::config(format!(
                    "Option 'extruder' in section '{}' must be specified",
                    config.get_name()
                ))
            })?;
        let stepper = Arc::clone(&extruder_stepper);
        let name = extruder_name.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_: &[EventArg]| {
                // `handle_connect` (`klippy/extras/extruder_stepper.py:16`).
                stepper.sync_to_extruder(Some(&name))?;
                Ok(PyValue::None)
            }),
        );
        Ok(PrinterExtruderStepper {
            extruder_stepper,
            extruder_name,
        })
    }

    /// `handle_connect` (`klippy/extras/extruder_stepper.py:16`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.extruder_stepper
            .sync_to_extruder(Some(&self.extruder_name))
    }

    /// `find_past_position(print_time)`
    /// (`klippy/extras/extruder_stepper.py:18`).
    pub fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        self.extruder_stepper.find_past_position(print_time)
    }

    /// `get_status(eventtime)` (`klippy/extras/extruder_stepper.py:20`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.extruder_stepper.get_status(eventtime)
    }
}

/// The wrapper is duck-typed as an extruder stepper (`find_past_position` /
/// `get_status` callers, e.g. the filament sensors, reach it through the
/// stepper name), so it declares `dyn ExtruderStepperApi` like the
/// `ExtruderStepper` it wraps.
impl ExtruderStepperApi for PrinterExtruderStepper {
    fn get_status(&self, eventtime: f64) -> PyValue {
        self.extruder_stepper.get_status(eventtime)
    }

    fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        self.extruder_stepper.find_past_position(print_time)
    }

    fn sync_to_extruder(&self, extruder_name: Option<&str>) -> KResult<()> {
        self.extruder_stepper.sync_to_extruder(extruder_name)
    }

    fn stepper(&self) -> klippy_api::stepper::Stepper {
        self.extruder_stepper.stepper()
    }

    fn cmd_set_pressure_advance(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.extruder_stepper.cmd_set_pressure_advance(gcmd)
    }
}

impl PrinterObject for PrinterExtruderStepper {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ExtruderStepperApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.extruder_stepper.get_status(eventtime)))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/extruder_stepper.py:23`).
///
/// Genuine construction of the inner `kinematics::extruder::ExtruderStepper`
/// requires a factory that the current contract does not expose (the
/// `extras` crate cannot depend on `kinematics`); see the report for the
/// exact trait addition needed. The wrapper itself is built by
/// [`PrinterExtruderStepper::new`] wherever the handle is available (the
/// core assembly, and the parity test).
fn load_config_prefix(_config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(None)
}

/// Module entry point for the `extruder_stepper` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
