//! Port of the `board_pins` module (`klippy/extras/board_pins.py`).

use klippy_api::config::{Config, GetListOpts, GetListsOpts, ListParser};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::PinsApi;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{lookup_interface, ObjectRef};
use pycompat::PyValue;
use std::sync::Arc;

/// `klippy/extras/board_pins.py:PrinterBoardAliases`.
#[allow(dead_code)]
pub struct PrinterBoardAliases;

impl PrinterBoardAliases {
    /// `PrinterBoardAliases.__init__(config)` (`klippy/extras/board_pins.py:8`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let mcu_names = config
            .getlist(
                "mcu",
                GetListOpts {
                    default: Some(Some(vec!["mcu".to_string()])),
                    ..Default::default()
                },
            )?
            .unwrap_or_else(|| vec!["mcu".to_string()]);
        let mut pin_resolvers = Vec::new();
        for name in &mcu_names {
            pin_resolvers.push(ppins.get_pin_resolver(name)?);
        }
        let mut options = vec!["aliases".to_string()];
        options.extend(config.get_prefix_options("aliases_")?);
        for opt in &options {
            let aliases = config.getlists(
                opt,
                GetListsOpts {
                    seps: vec!["=".to_string(), ",".to_string()],
                    count: Some(2),
                    parser: ListParser::Str,
                    ..Default::default()
                },
            )?;
            for (name, value) in parse_aliases(&aliases)? {
                if value.starts_with('<') && value.ends_with('>') {
                    for pin_resolver in &pin_resolvers {
                        pin_resolver.reserve_pin(&name, &value)?;
                    }
                } else {
                    for pin_resolver in &pin_resolvers {
                        pin_resolver.alias_pin(&name, &value)?;
                    }
                }
            }
        }
        Ok(PrinterBoardAliases)
    }
}

klippy_api::printer_object!(PrinterBoardAliases);

fn parse_aliases(value: &PyValue) -> KResult<Vec<(String, String)>> {
    let outer = match value {
        PyValue::Tuple(items) | PyValue::List(items) => items,
        _ => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "cannot unpack non-iterable object".to_string(),
            });
        }
    };
    let mut result = Vec::with_capacity(outer.len());
    for item in outer {
        let pair = match item {
            PyValue::Tuple(pair) | PyValue::List(pair) => pair,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "cannot unpack non-iterable object".to_string(),
                });
            }
        };
        if pair.len() != 2 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: if pair.len() > 2 {
                    "too many values to unpack (expected 2)".to_string()
                } else {
                    format!(
                        "not enough values to unpack (expected 2, got {})",
                        pair.len()
                    )
                },
            });
        }
        let name = match &pair[0] {
            PyValue::Str(s) => s.clone(),
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "cannot unpack non-iterable object".to_string(),
                });
            }
        };
        let pin = match &pair[1] {
            PyValue::Str(s) => s.clone(),
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "cannot unpack non-iterable object".to_string(),
                });
            }
        };
        result.push((name, pin));
    }
    Ok(result)
}

/// `load_config(config)` (`klippy/extras/board_pins.py:23`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterBoardAliases::new(config)?) as ObjectRef))
}

/// `load_config_prefix(config)` (`klippy/extras/board_pins.py:26`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterBoardAliases::new(config)?) as ObjectRef))
}

/// Module entry point for the `board_pins` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
