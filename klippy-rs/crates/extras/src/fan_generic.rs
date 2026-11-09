//! Port of `klippy/extras/fan_generic.py`.
//!
//! A `[fan_generic NAME]` section builds a real [`Fan`] (from `fan.py`, with
//! `default_shutdown_speed=0.`), the shared template evaluator (from
//! `output_pin.py`) and registers the mux command `SET_FAN_SPEED FAN=<name>`.
//! Status is the fan status. `_template_update` converts the rendered text
//! with `float()`, logs `fan_generic template render error` on `ValueError`
//! and sets the speed to `0.`.

use std::sync::Arc;

use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCode, GCodeApi, GCodeCommandHandler, GetFloatOpts, GetOpts};
use klippy_api::lookup_interface;
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};

use crate::fan::Fan;
use crate::output_pin::{lookup_template_eval, PrinterTemplateEvaluator, TemplateCallback};

/// `PrinterFanGeneric.cmd_SET_FAN_SPEED_help` (`klippy/extras/fan_generic.py:10`).
pub const CMD_SET_FAN_SPEED_HELP: &str = "Sets the speed of a fan";

/// Port of `PrinterFanGeneric` (`klippy/extras/fan_generic.py:9`).
pub struct PrinterFanGeneric {
    printer: Printer,
    fan: Arc<Fan>,
    template_eval: Arc<PrinterTemplateEvaluator>,
    fan_name: String,
}

impl PrinterFanGeneric {
    /// `PrinterFanGeneric.__init__(config)` (`klippy/extras/fan_generic.py:11`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let fan = Fan::new(config, 0.)?;
        let fan_name = fan_name_from_section_name(&config.get_name())?;
        let template_eval = lookup_template_eval(config)?;

        let this = Arc::new(PrinterFanGeneric {
            printer,
            fan,
            template_eval,
            fan_name: fan_name.clone(),
        });
        let gcode: GCode =
            lookup_interface::<dyn GCodeApi>(&*this.printer, "gcode", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        let owner = Arc::clone(&this);
        let handler: GCodeCommandHandler = Box::new(move |gcmd| owner.cmd_set_fan_speed(gcmd));
        gcode.register_mux_command(
            "SET_FAN_SPEED",
            "FAN",
            Some(&fan_name),
            handler,
            Some(CMD_SET_FAN_SPEED_HELP),
        )?;
        Ok(this)
    }

    /// The wrapped `fan.Fan` (`self.fan`).
    pub fn fan(&self) -> &Arc<Fan> {
        &self.fan
    }

    /// `config.get_name().split()[-1]` (`klippy/extras/fan_generic.py:14`).
    pub fn fan_name(&self) -> &str {
        &self.fan_name
    }

    /// `PrinterFanGeneric.get_status(eventtime)` (`klippy/extras/fan_generic.py:25`).
    pub fn get_status(&self, eventtime: f64) -> pycompat::PyValue {
        self.fan.get_status(eventtime)
    }

    /// `PrinterFanGeneric._template_update(text)`
    /// (`klippy/extras/fan_generic.py:27`): `float(text)`; on `ValueError`
    /// log `fan_generic template render error` and use `0.`.
    pub fn template_update(&self, text: &str) -> KResult<()> {
        let value = match pycompat::text::float_from_string(text) {
            Ok(value) => value,
            Err(e) => {
                // `logging.exception("fan_generic template render error")`:
                // the error level record carries the exception display.
                let traceback = format!("ValueError: {}", e.display());
                klippy_api::log::exception("fan_generic template render error", &traceback);
                0.
            }
        };
        self.fan.set_speed(value, None)
    }

    /// `PrinterFanGeneric.cmd_SET_FAN_SPEED(gcmd)`
    /// (`klippy/extras/fan_generic.py:34`).
    pub fn cmd_set_fan_speed(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let speed = gcmd.get_float(
            "SPEED",
            GetFloatOpts {
                default: Some(None),
                minval: Some(0.),
                ..GetFloatOpts::default()
            },
        )?;
        let template = match gcmd.get(
            "TEMPLATE",
            GetOpts {
                default: Some(pycompat::PyValue::None),
                ..GetOpts::default()
            },
        )? {
            pycompat::PyValue::None => None,
            other => Some(other.str()),
        };
        if speed.is_none() == template.is_none() {
            return Err(KlippyError::command(
                "SET_FAN_SPEED must specify SPEED or TEMPLATE",
            ));
        }
        if template.is_some() {
            let owner = Arc::clone(self);
            let callback: TemplateCallback = Arc::new(move |text: &str| {
                let _ = owner.template_update(text);
            });
            return self.template_eval.set_template(gcmd, callback, None);
        }
        let speed = speed.expect("template is None implies SPEED was given");
        self.fan.set_speed_from_command(speed)
    }
}

/// `config.get_name().split()[-1]` (`klippy/extras/fan_generic.py:14`):
/// the last whitespace-separated token of the section name. A name without
/// tokens raises the Python `IndexError` (`list index out of range`).
fn fan_name_from_section_name(name: &str) -> KResult<String> {
    name.split_whitespace()
        .last()
        .map(|token| token.to_string())
        .ok_or_else(|| KlippyError::Internal {
            class_name: "IndexError".to_string(),
            message: "list index out of range".to_string(),
        })
}

impl PrinterObject for PrinterFanGeneric {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    /// `PrinterFanGeneric.get_status` (`klippy/extras/fan_generic.py:25`).
    fn get_status(&self, eventtime: f64) -> Option<pycompat::PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/fan_generic.py:45`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = PrinterFanGeneric::new(config)?;
    Ok(Some(obj as ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        ..ModuleFactory::default()
    }
}
