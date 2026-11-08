//! Port of the `respond` module (`klippy/extras/respond.py`).
//!
//! Adds the `M118` and `RESPOND` G-code commands for sending messages to
//! the host. `M118` echoes the raw parameters with the configured default
//! prefix; `RESPOND` selects the prefix with `TYPE` (defaulting to the
//! configured default prefix), allows an explicit `PREFIX` override, and
//! echoes `MSG` with a space unless the type is `echo_no_space`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetStrOpts};
use klippy_api::error::KResult;
use klippy_api::gcode::{GCmd, GCodeApi, GetOpts, RegisterCommandOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, ObjectRef};
use pycompat::PyValue;
use std::sync::Arc;

/// `respond_types` (`klippy/extras/respond.py:7`): the `TYPE` values that
/// use a space between the prefix and the message.
fn respond_types() -> IndexMap<PyValue, PyValue> {
    let mut map = IndexMap::new();
    map.insert(PyValue::Str("echo".to_string()), PyValue::Str("echo:".to_string()));
    map.insert(PyValue::Str("command".to_string()), PyValue::Str("//".to_string()));
    map.insert(PyValue::Str("error".to_string()), PyValue::Str("!!".to_string()));
    map
}

/// `respond_types_no_space` (`klippy/extras/respond.py:13`): the `TYPE`
/// values whose prefix is echoed without a space.
fn respond_types_no_space() -> IndexMap<PyValue, PyValue> {
    let mut map = IndexMap::new();
    map.insert(
        PyValue::Str("echo_no_space".to_string()),
        PyValue::Str("echo:".to_string()),
    );
    map
}

/// `HostResponder.cmd_RESPOND_help` (`klippy/extras/respond.py:31`).
pub const CMD_RESPOND_HELP: &str = "Echo the message prepended with a prefix";

/// `klippy/extras/respond.py:HostResponder`.
#[allow(dead_code)]
pub struct HostResponder {
    printer: Printer,
    default_prefix: String,
}

impl HostResponder {
    /// `HostResponder.__init__(config)` (`klippy/extras/respond.py:18`).
    ///
    /// Returns the shared object: the two G-code handlers registered below
    /// capture a clone of it, so construction hands out an `Arc`.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        // `self.reactor = self.printer.get_reactor()`; the reactor is never
        // used again in the Python module, so the call is made for its
        // lookup side effect and the handle is dropped.
        let _ = printer.get_reactor();
        let choices = respond_types();
        let default_prefix_value = config.getchoice(
            "default_type",
            &PyValue::Dict(choices),
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("echo".to_string()))),
                note_valid: true,
            },
        )?;
        let default_prefix_value = match default_prefix_value {
            PyValue::Str(s) => s,
            // `getchoice` only returns the mapped strings; a host object
            // would be a config error in Python, not a valid prefix.
            // This branch cannot be reached through a faithful `getchoice`.
            other => other.str(),
        };
        let default_prefix = config.get(
            "default_prefix",
            GetStrOpts {
                default: Some(Some(default_prefix_value)),
                note_valid: true,
            },
        )?
        .unwrap_or_default();
        let this = Arc::new(HostResponder {
            printer,
            default_prefix,
        });
        let gcode = lookup_interface::<dyn GCodeApi>(&*this.printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        {
            let this = this.clone();
            gcode.register_command(
                "M118",
                Some(Box::new(move |gcmd: &GCmd| this.cmd_m118(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: true,
                    desc: None,
                },
            )?;
        }
        {
            let this = this.clone();
            gcode.register_command(
                "RESPOND",
                Some(Box::new(move |gcmd: &GCmd| this.cmd_respond(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: true,
                    desc: Some(CMD_RESPOND_HELP.to_string()),
                },
            )?;
        }
        Ok(this)
    }

    /// The computed `default_prefix` (after `default_type` and the
    /// `default_prefix` config override).
    pub fn default_prefix(&self) -> &str {
        &self.default_prefix
    }

    /// `cmd_M118` (`klippy/extras/respond.py:28`).
    pub fn cmd_m118(&self, gcmd: &GCmd) -> KResult<()> {
        let msg = gcmd.get_raw_command_parameters();
        gcmd.respond_raw(&format!("{} {}", self.default_prefix, msg));
        Ok(())
    }

    /// `cmd_RESPOND` (`klippy/extras/respond.py:32`).
    pub fn cmd_respond(&self, gcmd: &GCmd) -> KResult<()> {
        let mut no_space = false;
        let respond_type_value = gcmd.get(
            "TYPE",
            GetOpts {
                default: Some(PyValue::None),
                parser: klippy_api::gcode::GetParser::Str,
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        let mut prefix = self.default_prefix.clone();
        if !matches!(respond_type_value, PyValue::None) {
            let respond_type = match respond_type_value {
                PyValue::Str(s) => s.to_lowercase(),
                // `parser=str` always yields a string when present.
                other => other.str().to_lowercase(),
            };
            let types = respond_types();
            if let Some(PyValue::Str(p)) = types.get(&PyValue::Str(respond_type.clone())) {
                prefix = p.clone();
            } else {
                let no_space_types = respond_types_no_space();
                if let Some(PyValue::Str(p)) =
                    no_space_types.get(&PyValue::Str(respond_type.clone()))
                {
                    prefix = p.clone();
                    no_space = true;
                } else {
                    return Err(klippy_api::error::KlippyError::command(format!(
                        "RESPOND TYPE '{respond_type}' is invalid. Must be one of \
                         'echo', 'command', or 'error'"
                    )));
                }
            }
        }
        let prefix_value = gcmd.get(
            "PREFIX",
            GetOpts {
                default: Some(PyValue::Str(prefix.clone())),
                parser: klippy_api::gcode::GetParser::Str,
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        let prefix = match prefix_value {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let msg_value = gcmd.get(
            "MSG",
            GetOpts {
                default: Some(PyValue::Str(String::new())),
                parser: klippy_api::gcode::GetParser::Str,
                minval: None,
                maxval: None,
                above: None,
                below: None,
            },
        )?;
        let msg = match msg_value {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        if no_space {
            gcmd.respond_raw(&format!("{prefix}{msg}"));
        } else {
            gcmd.respond_raw(&format!("{prefix} {msg}"));
        }
        Ok(())
    }
}

klippy_api::printer_object!(HostResponder);

/// `load_config(config)` (`klippy/extras/respond.py:54`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(HostResponder::new(config)? as ObjectRef))
}

/// Module entry point for the `respond` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
