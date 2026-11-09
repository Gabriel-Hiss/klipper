//! Port of the `heater_generic` module (`klippy/extras/heater_generic.py`).
//!
//! Source: `klippy/extras/heater_generic.py` (all of it).
//!
//! A `[heater_generic <name>]` section carries no options of its own: the
//! `heaters` printer object builds the `Heater` from that same config, so
//! this module only forwards the config to it. The `Heater` object is
//! returned and stored under the full section name, like any other
//! `load_config_prefix` result.

use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::object::{load_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory};

use crate::heaters::PrinterHeatersApi;

/// `load_config_prefix(config)` (`klippy/extras/heater_generic.py:7-9`).
///
/// `pheaters = config.get_printer().load_object(config, 'heaters')` and
/// `return pheaters.setup_heater(config)`; `setup_heater` keeps its
/// `gcode_id=None` default (`heaters.py:275`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(
        &*printer,
        config,
        "heaters",
        LookupDefault::Raise,
    )?
    .expect("Raise never yields None");
    let heater = pheaters.setup_heater(config, None)?;
    Ok(Some(heater as ObjectRef))
}

/// Module entry point for the `heater_generic` section name. Only
/// `load_config_prefix` exists in Python.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
