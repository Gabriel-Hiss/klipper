//! Port of the `canbus_ids` module (`klippy/extras/canbus_ids.py`).
//!
//! Tracks CAN bus UUIDs mapped to node ids. The first node is
//! [`NODEID_FIRST`]; each subsequent UUID gets the next integer.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{ModuleFactory, Printer};
use klippy_api::{GCell, ObjectRef};
use klippy_core::mcu::CanbusIdsApi;
use std::sync::Arc;

/// First node id assigned by `PrinterCANBus`.
pub const NODEID_FIRST: i64 = 4;

/// `klippy/extras/canbus_ids.py:PrinterCANBus`.
#[allow(dead_code)]
pub struct PrinterCANBus {
    printer: Printer,
    ids: GCell<IndexMap<String, i64>>,
}

impl PrinterCANBus {
    /// `PrinterCANBus.__init__(config)` (`klippy/extras/canbus_ids.py:10`).
    pub fn new(config: &Config) -> Self {
        PrinterCANBus {
            printer: config.get_printer(),
            ids: GCell::new(IndexMap::new()),
        }
    }
}

impl CanbusIdsApi for PrinterCANBus {
    /// `PrinterCANBus.add_uuid(config, canbus_uuid, canbus_iface)`
    /// (`klippy/extras/canbus_ids.py:13`).
    fn add_uuid(&self, _config: &Config, canbus_uuid: &str, _canbus_iface: &str) -> KResult<i64> {
        let mut ids = self.ids.borrow_mut();
        if ids.contains_key(canbus_uuid) {
            return Err(KlippyError::config("Duplicate canbus_uuid"));
        }
        let new_id = ids.len() as i64 + NODEID_FIRST;
        ids.insert(canbus_uuid.to_string(), new_id);
        Ok(new_id)
    }

    /// `PrinterCANBus.get_nodeid(canbus_uuid)`
    /// (`klippy/extras/canbus_ids.py:19`).
    fn get_nodeid(&self, canbus_uuid: &str) -> KResult<i64> {
        let ids = self.ids.borrow();
        if let Some(&node_id) = ids.get(canbus_uuid) {
            return Ok(node_id);
        }
        Err(KlippyError::config(format!("Unknown canbus_uuid {canbus_uuid}")))
    }
}

klippy_api::printer_object!(PrinterCANBus; dyn CanbusIdsApi);

/// `load_config(config)` (`klippy/extras/canbus_ids.py:25`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterCANBus::new(config)) as ObjectRef))
}

/// Module entry point for the `canbus_ids` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
