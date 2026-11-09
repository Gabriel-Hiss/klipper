//! Port of `klippy/extras/gcode_button.py`.
//!
//! A `gcode_button` section binds a hardware button to a press/release
//! G-code template pair, with `QUERY_BUTTON` reporting its state.

use std::sync::Arc;

use klippy_api::config::{Config, GetFloatListOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi, GCmd};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::PyValue;

use crate::buttons::{ButtonState, DebounceCallback, PrinterButtonsApi};
use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};

/// `GCodeButton.cmd_QUERY_BUTTON_help` (`klippy/extras/gcode_button.py:33`).
pub const CMD_QUERY_BUTTON_HELP: &str = "Report on the state of a button";

/// Python `str.strip()` whitespace set: `char::is_whitespace` plus the
/// `\x1c`..=`\x1f` separators CPython strips as well
/// (`Objects/unicodeobject.c` `_PyUnicode_IsWhitespace`).
fn py_strip(s: &str) -> &str {
    s.trim_matches(|c: char| c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c))
}

/// Python truthiness of a debounced button state
/// (`klippy/extras/gcode_button.py:40,51` `if not state`): `None`
/// (`ButtonState::Empty`) and `False`/`0` are falsy.
fn state_truthy(state: ButtonState) -> bool {
    match state {
        ButtonState::Empty => false,
        ButtonState::Analog(pressed) => pressed,
        ButtonState::Digital(state) => state != 0,
    }
}

/// Shared mutable state of a `GCodeButton`, referenced by the button
/// callback and the `QUERY_BUTTON` handler (the Python instance is
/// referenced from `__init__` and the bound methods the same way).
struct GCodeButtonInner {
    name: String,
    gcode: GCell<Option<GCode>>,
    last_state: GCell<ButtonState>,
    press_template: GCell<Option<Arc<TemplateWrapper>>>,
    release_template: GCell<Option<Arc<TemplateWrapper>>>,
}

impl GCodeButtonInner {
    /// `button_callback` (`klippy/extras/gcode_button.py:37`).
    fn button_callback(&self, _eventtime: f64, state: ButtonState) {
        self.last_state.set(state);
        let template = if state_truthy(state) {
            self.press_template.cloned()
        } else {
            self.release_template.cloned()
        };
        // The callback can only fire after construction, when both
        // templates are loaded; a missing template means the callback
        // fired in the middle of the constructor, which cannot happen
        // in the Python original (registration happens before any
        // button event can be dispatched).
        let Some(template) = template else { return };
        let commands = match template.render(None) {
            Ok(commands) => commands,
            Err(e) => {
                log::exception(
                    "Script running error",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                return;
            }
        };
        if py_strip(&commands).is_empty() {
            return;
        }
        let Some(gcode) = self.gcode.cloned() else { return };
        if let Err(e) = gcode.run_script(&commands) {
            log::exception(
                "Script running error",
                &format!("{}: {}", e.class_name(), e.message()),
            );
        }
    }

    /// `cmd_QUERY_BUTTON` (`klippy/extras/gcode_button.py:34`).
    fn cmd_query_button(&self, gcmd: &GCmd) -> KResult<()> {
        gcmd.respond_info(
            &format!("{}: {}", self.name, self.state_name()),
            true,
        );
        Ok(())
    }

    /// `get_status()['state']` (`klippy/extras/gcode_button.py:50`).
    fn state_name(&self) -> &'static str {
        if state_truthy(self.last_state.get()) {
            "PRESSED"
        } else {
            "RELEASED"
        }
    }

    fn status(&self) -> PyValue {
        let mut d = indexmap::IndexMap::new();
        d.insert(
            PyValue::Str("state".to_string()),
            PyValue::Str(self.state_name().to_string()),
        );
        PyValue::Dict(d)
    }
}

/// A `gcode_button` printer object (`klippy/extras/gcode_button.py`
/// `GCodeButton`).
pub struct GCodeButton {
    inner: Arc<GCodeButtonInner>,
    pin: String,
}

impl GCodeButton {
    /// `GCodeButton.__init__` (`klippy/extras/gcode_button.py:10`):
    /// reads the section config, registers the button with the
    /// `buttons` object, loads the press/release templates from the
    /// `gcode_macro` object, and registers the `QUERY_BUTTON` mux
    /// command with the `gcode` object, in that order.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer: Printer = config.get_printer();
        let name = config
            .get_name()
            .rsplit(' ')
            .next()
            .unwrap_or("")
            .to_string();
        let pin = match config.get("pin", GetStrOpts::default())? {
            Some(pin) => pin,
            None => {
                return Err(KlippyError::config(format!(
                    "Option 'pin' in section '{}' must be specified",
                    config.get_name()
                )));
            }
        };
        // The inner state exists before the button is registered so
        // the registered callback and the `QUERY_BUTTON` handler can
        // both reach it; the templates are filled once loaded. The
        // `gcode` handle is resolved after the templates, as in the
        // Python original, so a template failure precedes a missing
        // `gcode` object.
        let inner = Arc::new(GCodeButtonInner {
            name,
            gcode: GCell::new(None),
            last_state: GCell::new(ButtonState::Digital(0)),
            press_template: GCell::new(None),
            release_template: GCell::new(None),
        });
        let callback_inner = inner.clone();
        let callback: DebounceCallback =
            Arc::new(move |eventtime: f64, state: ButtonState| {
                callback_inner.button_callback(eventtime, state);
            });
        let buttons = match load_interface::<dyn PrinterButtonsApi>(
            &*printer,
            config,
            "buttons",
            LookupDefault::Raise,
        )? {
            Some(buttons) => buttons,
            None => {
                return Err(KlippyError::config("Unknown config object 'buttons'"));
            }
        };
        if config
            .get(
                "analog_range",
                GetStrOpts { default: Some(None), ..Default::default() },
            )?
            .is_none()
        {
            buttons.register_debounce_button(&pin, callback, config)?;
        } else {
            let range = config
                .getfloatlist(
                    "analog_range",
                    GetFloatListOpts { count: Some(2), ..Default::default() },
                )?
                .unwrap_or_default();
            let (amin, amax) = match range.as_slice() {
                [amin, amax] => (*amin, *amax),
                _ => {
                    return Err(KlippyError::Internal {
                        class_name: "ValueError".to_string(),
                        message: format!(
                            "not enough values to unpack (expected 2, got {})",
                            range.len()
                        ),
                    });
                }
            };
            let pullup = config
                .getfloat(
                    "analog_pullup_resistor",
                    GetFloatOpts {
                        default: Some(Some(4700.0)),
                        above: Some(0.0),
                        ..Default::default()
                    },
                )?
                .unwrap_or(4700.0);
            buttons.register_debounce_adc_button(&pin, amin, amax, pullup, callback, config)?;
        }
        let gcode_macro = match load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )? {
            Some(gcode_macro) => gcode_macro,
            None => {
                return Err(KlippyError::config("Unknown config object 'gcode_macro'"));
            }
        };
        let press_template = gcode_macro.load_template(config, "press_gcode", None)?;
        *inner.press_template.borrow_mut() = Some(Arc::new(press_template));
        let release_template = gcode_macro.load_template(config, "release_gcode", Some(""))?;
        *inner.release_template.borrow_mut() = Some(Arc::new(release_template));
        let gcode = match lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)? {
            Some(gcode) => gcode,
            None => {
                return Err(KlippyError::config("Unknown config object 'gcode'"));
            }
        };
        let handler_inner = inner.clone();
        gcode.register_mux_command(
            "QUERY_BUTTON",
            "BUTTON",
            Some(&inner.name),
            Box::new(move |gcmd: &GCmd| handler_inner.cmd_query_button(gcmd)),
            Some(CMD_QUERY_BUTTON_HELP),
        )?;
        *inner.gcode.borrow_mut() = Some(gcode);
        Ok(Arc::new(GCodeButton { inner, pin }))
    }

    /// `GCodeButton.name` (`klippy/extras/gcode_button.py:12`).
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// `GCodeButton.pin` (`klippy/extras/gcode_button.py:13`).
    pub fn pin(&self) -> &str {
        &self.pin
    }

    /// The loaded press template (`GCodeButton.press_template`,
    /// `klippy/extras/gcode_button.py:25`), for tests and introspection.
    pub fn press_template(&self) -> Option<Arc<TemplateWrapper>> {
        self.inner.press_template.cloned()
    }

    /// The loaded release template (`GCodeButton.release_template`,
    /// `klippy/extras/gcode_button.py:26`), for tests and introspection.
    pub fn release_template(&self) -> Option<Arc<TemplateWrapper>> {
        self.inner.release_template.cloned()
    }

    /// `GCodeButton.button_callback`
    /// (`klippy/extras/gcode_button.py:37`).
    pub fn button_callback(&self, eventtime: f64, state: ButtonState) {
        self.inner.button_callback(eventtime, state);
    }

    /// `GCodeButton.cmd_QUERY_BUTTON`
    /// (`klippy/extras/gcode_button.py:34`).
    pub fn cmd_query_button(&self, gcmd: &GCmd) -> KResult<()> {
        self.inner.cmd_query_button(gcmd)
    }

    /// `GCodeButton.get_status`
    /// (`klippy/extras/gcode_button.py:50`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        self.inner.status()
    }
}

impl PrinterObject for GCodeButton {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `load_config_prefix` (`klippy/extras/gcode_button.py:55`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(GCodeButton::new(config)?))
}

/// The module factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..Default::default() }
}
