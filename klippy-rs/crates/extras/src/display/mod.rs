//! Port of `klippy/extras/display/__init__.py`: the `display` package entry
//! points. `[display]` loads the primary display; `[display <name>]` loads an
//! auxiliary one and needs the primary section.
pub mod aip31068_spi;
pub mod display;
pub mod font8x14;
pub mod hd44780;
pub mod hd44780_spi;
pub mod menu;
pub mod menu_keys;
pub mod st7920;
pub mod uc1701;

use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::ModuleFactory;
use klippy_api::ObjectRef;

/// `load_config(config)` (`__init__.py:8-9`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    display::load_config(config)
}

/// `load_config_prefix(config)` (`__init__.py:11-21`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    if !config.has_section("display") {
        return Err(KlippyError::config(
            "A primary [display] section must be defined in printer.cfg \
             to use auxiliary displays"
                .to_string(),
        ));
    }
    let full_name = config.get_name();
    // `config.get_name().split()[-1]`
    let name = full_name
        .split(display::python_whitespace)
        .rfind(|part| !part.is_empty())
        .expect("a load_config_prefix section name starts with 'display'");
    if name == "display" {
        return Err(KlippyError::config(
            "Section name [display display] is not valid. \
             Please choose a different postfix."
                .to_string(),
        ));
    }
    display::load_config(config)
}

/// Module entry point for the `display` package.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
