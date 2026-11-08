//! Port of the `duplicate_pin_override` module (`klippy/extras/duplicate_pin_override.py`).
//!
//! The `[duplicate_pin_override]` section lists pins (the required `pins`
//! option) that may be used more than once in the config. Loading the
//! object records each listed pin on the `pins` printer object via
//! `allow_multi_use_pin`, in list order.

use klippy_api::config::{Config, GetListOpts};
use klippy_api::error::KResult;
use klippy_api::lookup_interface;
use klippy_api::mcu::PinsApi;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::ObjectRef;
use std::sync::Arc;

/// `klippy/extras/duplicate_pin_override.py:PrinterDupPinOverride`.
pub struct PrinterDupPinOverride;

impl PrinterDupPinOverride {
    /// `PrinterDupPinOverride.__init__(config)`
    /// (`klippy/extras/duplicate_pin_override.py:8`).
    ///
    /// Looks up the `pins` object (a missing object raises a config
    /// error, like `printer.lookup_object('pins')` with the default
    /// sentinel), reads the required `pins` list, then calls
    /// `allow_multi_use_pin` for each entry in order. The first error
    /// from either step aborts the rest, matching the Python loop.
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let pins = config
            .getlist("pins", GetListOpts::default())?
            .expect("required 'pins' option never yields None");
        for pin_desc in &pins {
            ppins.allow_multi_use_pin(pin_desc)?;
        }
        Ok(PrinterDupPinOverride)
    }
}

klippy_api::printer_object!(PrinterDupPinOverride);

/// `load_config(config)` (`klippy/extras/duplicate_pin_override.py:14`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(
        Arc::new(PrinterDupPinOverride::new(config)?) as ObjectRef,
    ))
}

/// Module entry point for the `duplicate_pin_override` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
