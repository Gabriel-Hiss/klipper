//! Port of the `static_digital_output` module (`klippy/extras/static_digital_output.py`).
//!
//! Sets a fixed digital-output level for each listed pin by queueing an MCU
//! config command at load time.
use klippy_api::config::{Config, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{AddConfigCmdOpts, LookupPinOpts, PinsApi};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{lookup_interface, ObjectRef};

use pycompat::text::{percent_format, PctValue};
use std::sync::Arc;

/// `klippy/extras/static_digital_output.py:PrinterStaticDigitalOut`.
pub struct PrinterStaticDigitalOut;

impl PrinterStaticDigitalOut {
    /// `PrinterStaticDigitalOut.__init__(config)` (`klippy/extras/static_digital_output.py:8`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_list = config
            .getlist("pins", GetListOpts::default())?
            .expect("getlist with required default never yields None");
        for pin_desc in &pin_list {
            let pin_params = ppins.lookup_pin(
                pin_desc,
                LookupPinOpts {
                    can_invert: true,
                    can_pullup: false,
                    share_type: None,
                },
            )?;
            let cmd = percent_format(
                "set_digital_out pin=%s value=%d",
                &[
                    PctValue::Str(pin_params.pin.clone()),
                    PctValue::Bool(pin_params.invert == 0),
                ],
            )
            .map_err(|e| KlippyError::Internal {
                class_name: e.kind.to_string(),
                message: e.msg,
            })?;
            pin_params
                .chip
                .add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        }
        Ok(PrinterStaticDigitalOut)
    }
}

klippy_api::printer_object!(PrinterStaticDigitalOut);

/// `load_config_prefix(config)` (`klippy/extras/static_digital_output.py:18`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterStaticDigitalOut::new(config)?) as ObjectRef))
}

/// Module entry point for `[static_digital_output <name>]` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
