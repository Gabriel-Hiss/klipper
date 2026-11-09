//! Port of `klippy/extras/lis3dh.py`.
//!
//! The Python module only defines the two loading entry points; both
//! build a `lis2dw::LIS2DW` sensor configured for an LIS3DH chip
//! (`lis2dw::LIS3DH_TYPE`).

use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::printer::ModuleFactory;
use klippy_api::ObjectRef;

use crate::lis2dw::{LIS2DW, LIS3DH_TYPE};

/// `LIS2DW(config, lis2dw.LIS3DH_TYPE)` (`klippy/extras/lis3dh.py:8`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = LIS2DW::new(config, LIS3DH_TYPE)?;
    Ok(Some(dev))
}

/// `LIS2DW(config, lis2dw.LIS3DH_TYPE)` (`klippy/extras/lis3dh.py:11`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let dev: ObjectRef = LIS2DW::new(config, LIS3DH_TYPE)?;
    Ok(Some(dev))
}

/// The Python module exposes exactly these two entry points; it has no
/// `add_early_printer_objects` or `add_printer_objects`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
