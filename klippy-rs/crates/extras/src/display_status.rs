//! Port of the `display_status` module (`klippy/extras/display_status.py`).
//!
//! Handles `M73` / `M117` / `SET_DISPLAY_TEXT` display progress messages.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GetFloatOpts, GetOpts, RegisterCommandOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::value::repr_key;
use pycompat::PyValue;
use std::sync::Arc;

/// `M73_TIMEOUT` (`display_status.py:8`).
pub const M73_TIMEOUT: f64 = 5.;

/// `cmd_SET_DISPLAY_TEXT_help` (`display_status.py:45`).
pub const SET_DISPLAY_TEXT_HELP: &str = "Set or clear the display message";

#[derive(Debug, Clone, Default)]
struct InnerState {
    expire_progress: f64,
    progress: Option<f64>,
    message: Option<String>,
}

/// `klippy/extras/display_status.py:DisplayStatus`.
pub struct DisplayStatus {
    printer: Printer,
    state: Arc<GCell<InnerState>>,
}

fn do_m73(printer: &Printer, state: &GCell<InnerState>, gcmd: &GCmd) -> KResult<()> {
    let progress = gcmd.get_float(
        "P",
        GetFloatOpts {
            default: Some(None),
            ..Default::default()
        },
    )?;
    if let Some(p) = progress {
        // `min(1., max(0., progress))`: the first argument wins ties, so
        // `-0.0` and NaN become `0.0` (`f64::max` may keep `-0.0`).
        let low = if p / 100. > 0. { p / 100. } else { 0. };
        let clamped = if low < 1. { low } else { 1. };
        let expire = printer.get_reactor().monotonic() + M73_TIMEOUT;
        state.with_mut(|s| {
            s.progress = Some(clamped);
            s.expire_progress = expire;
        });
    }
    Ok(())
}

fn do_m117(state: &GCell<InnerState>, gcmd: &GCmd) -> KResult<()> {
    let raw = gcmd.get_raw_command_parameters();
    let msg = match raw.is_empty() {
        true => None,
        false => Some(raw),
    };
    state.with_mut(|s| s.message = msg);
    Ok(())
}

fn do_set_display_text(state: &GCell<InnerState>, gcmd: &GCmd) -> KResult<()> {
    let msg = gcmd.get(
        "MSG",
        GetOpts {
            default: Some(PyValue::None),
            ..Default::default()
        },
    )?;
    let message = match msg {
        PyValue::None => None,
        PyValue::Str(s) => Some(s),
        other => Some(other.str()),
    };
    state.with_mut(|s| s.message = message);
    Ok(())
}

impl DisplayStatus {
    /// `DisplayStatus.__init__(config)` (`display_status.py:11-21`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn klippy_api::gcode::GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let state: Arc<GCell<InnerState>> = Arc::new(GCell::new(InnerState::default()));

        let s1 = Arc::clone(&state);
        let p1 = printer.clone();
        gcode.register_command(
            "M73",
            Some(Box::new(move |gcmd| do_m73(&p1, &s1, gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: None,
            },
        )?;

        let s2 = Arc::clone(&state);
        gcode.register_command(
            "M117",
            Some(Box::new(move |gcmd| do_m117(&s2, gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: None,
            },
        )?;

        let s3 = Arc::clone(&state);
        gcode.register_command(
            "SET_DISPLAY_TEXT",
            Some(Box::new(move |gcmd| do_set_display_text(&s3, gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(SET_DISPLAY_TEXT_HELP.to_string()),
            },
        )?;

        Ok(DisplayStatus { printer, state })
    }

    /// `DisplayStatus.cmd_M73` (`display_status.py:35-41`).
    pub fn cmd_m73(&self, gcmd: &GCmd) -> KResult<()> {
        do_m73(&self.printer, &self.state, gcmd)
    }

    /// `DisplayStatus.cmd_M117` (`display_status.py:42-44`).
    pub fn cmd_m117(&self, gcmd: &GCmd) -> KResult<()> {
        do_m117(&self.state, gcmd)
    }

    /// `DisplayStatus.cmd_SET_DISPLAY_TEXT` (`display_status.py:46-47`).
    pub fn cmd_set_display_text(&self, gcmd: &GCmd) -> KResult<()> {
        do_set_display_text(&self.state, gcmd)
    }

    /// Fallible `get_status` body (`display_status.py:22-34`).
    ///
    /// [`PrinterObject::get_status`] cannot propagate errors; this is the
    /// faithful path tests drive, and the trait wrapper below returns `None`
    /// when it fails.
    pub fn get_status_value(&self, eventtime: f64) -> KResult<PyValue> {
        let (mut progress, expire, message) = {
            let s = self.state.borrow();
            (s.progress, s.expire_progress, s.message.clone())
        };
        if progress.is_some() {
            if eventtime > expire {
                let obj = self
                    .printer
                    .lookup_object("idle_timeout", LookupDefault::Raise)?
                    .expect("Raise never yields None");
                let status = obj.get_status(eventtime).ok_or_else(|| {
                    KlippyError::Internal {
                        class_name: "AttributeError".to_string(),
                        message: "'idle_timeout' object has no attribute 'get_status'"
                            .to_string(),
                    }
                })?;
                let st = dict_get(&status, "state")?;
                let state_str = match st {
                    PyValue::Str(s) => s.as_str(),
                    other => return Err(type_error("state", "str", other)),
                };
                if state_str != "Printing" {
                    progress = None;
                    self.state.with_mut(|s| s.progress = None);
                }
            }
        }
        let final_progress = match progress {
            Some(p) => p,
            None => {
                let mut fp = 0.;
                if let Some(sd) = self
                    .printer
                    .lookup_object("virtual_sdcard", LookupDefault::ReturnNone)?
                {
                    let st = sd.get_status(eventtime).ok_or_else(|| {
                        KlippyError::Internal {
                            class_name: "AttributeError".to_string(),
                            message: "'virtual_sdcard' object has no attribute 'get_status'"
                                .to_string(),
                        }
                    })?;
                    let v = dict_get(&st, "progress")?;
                    fp = match v {
                        PyValue::Float(f) => *f,
                        PyValue::Int(n) => n.to_f64_exact().unwrap_or(f64::NAN),
                        PyValue::Bool(b) => {
                            if *b {
                                1.
                            } else {
                                0.
                            }
                        }
                        other => return Err(type_error("progress", "float", other)),
                    };
                }
                fp
            }
        };
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("progress".to_string()),
            PyValue::Float(final_progress),
        );
        d.insert(
            PyValue::Str("message".to_string()),
            match message {
                Some(m) => PyValue::Str(m),
                None => PyValue::None,
            },
        );
        Ok(PyValue::Dict(d))
    }
}

fn dict_get<'a>(v: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match v {
        PyValue::Dict(d) => {
            let k = PyValue::Str(key.to_string());
            d.get(&k).ok_or_else(|| KlippyError::Other {
                class_name: "KeyError".to_string(),
                message: repr_key(key),
            })
        }
        PyValue::None => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "'NoneType' object is not subscriptable".to_string(),
        }),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", py_value_type_name(other)),
        }),
    }
}

fn type_error(_key: &str, expected: &str, actual: &PyValue) -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: format!(
            "'{}' object cannot be interpreted as a {}",
            py_value_type_name(actual),
            expected
        ),
    }
}

fn py_value_type_name(v: &PyValue) -> &'static str {
    match v {
        PyValue::None => "NoneType",
        PyValue::Bool(_) => "bool",
        PyValue::Int(_) => "int",
        PyValue::Float(_) => "float",
        PyValue::Str(_) => "str",
        PyValue::Bytes(_) => "bytes",
        PyValue::List(_) => "list",
        PyValue::Tuple(_) => "tuple",
        PyValue::Dict(_) => "dict",
        PyValue::Host(_) => "object",
    }
}

impl PrinterObject for DisplayStatus {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        self.get_status_value(eventtime).ok()
    }
}

/// `load_config(config)` (`display_status.py:49-50`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(DisplayStatus::new(config)?) as ObjectRef))
}

/// Module entry point for the `display_status` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
