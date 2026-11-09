//! Port of the `homing_override` module (`klippy/extras/homing_override.py`).
//!
//! Intercepts `G28`: with no axis parameter, or when an axis listed in the
//! `axes` option is present, the forced position (if configured) is applied
//! and the configured `gcode` template is run through `gcode_macro`; every
//! other `G28` and every call made while already inside the override script
//! delegates to the previously registered `G28` handler.

use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras_macros_templates::gcode_macro::Value;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::KResult;
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, GetOpts, RegisterCommandOpts};
use klippy_api::object::{load_interface, lookup_interface};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{GCell, ObjectRef};
use pycompat::PyValue;
use std::sync::Arc;

/// `klippy/extras/homing_override.py:HomingOverride`.
pub struct HomingOverride {
    printer: Printer,
    start_pos: [Option<f64>; 3],
    axes: String,
    template: TemplateWrapper,
    in_script: GCell<bool>,
    prev_g28: GCell<Option<GCodeCommandHandler>>,
}


impl HomingOverride {
    /// `start_pos` attribute (`homing_override.py:10`).
    pub fn start_pos(&self) -> [Option<f64>; 3] {
        self.start_pos
    }

    /// `axes` attribute (`homing_override.py:12`).
    pub fn axes(&self) -> &str {
        &self.axes
    }

    /// `HomingOverride.__init__(config)`
    /// (`klippy/extras/homing_override.py:8`). Returns the shared object:
    /// the `G28` closure captures its `Arc`.
    pub fn new(config: &Config) -> KResult<Arc<HomingOverride>> {
        let printer = config.get_printer();
        let start_pos: [Option<f64>; 3] = [
            config.getfloat(
                "set_position_x",
                GetFloatOpts { default: Some(None), ..Default::default() },
            )?,
            config.getfloat(
                "set_position_y",
                GetFloatOpts { default: Some(None), ..Default::default() },
            )?,
            config.getfloat(
                "set_position_z",
                GetFloatOpts { default: Some(None), ..Default::default() },
            )?,
        ];
        let axes = config
            .get("axes", GetStrOpts { default: Some(Some("XYZ".to_string())), ..Default::default() })?
            .expect("default XYZ is never None")
            .to_uppercase();
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let template = gcode_macro.load_template(config, "gcode", None)?;
        printer.load_object(config, "homing", LookupDefault::Raise)?;
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let prev_g28 = gcode.register_command("G28", None, RegisterCommandOpts::default())?;
        let ho = Arc::new(HomingOverride {
            printer,
            start_pos,
            axes,
            template,
            in_script: GCell::new(false),
            prev_g28: GCell::new(prev_g28),
        });
        let ho_dispatch = ho.clone();
        gcode.register_command(
            "G28",
            Some(Box::new(move |gcmd: &GCmd| ho_dispatch.cmd_g28(gcmd))),
            RegisterCommandOpts::default(),
        )?;
        Ok(ho)
    }

    /// `HomingOverride.cmd_G28(gcmd)`
    /// (`klippy/extras/homing_override.py:20`).
    pub fn cmd_g28(&self, gcmd: &GCmd) -> KResult<()> {
        if self.in_script.get() {
            // Was called recursively - invoke the real G28 command.
            return self.call_prev_g28(gcmd);
        }

        // If no axis is given as parameter we assume the override.
        let mut no_axis = true;
        for axis in "XYZ".chars() {
            let axis_name = axis.to_string();
            if has_param(gcmd, &axis_name)? {
                no_axis = false;
                break;
            }
        }

        let override_homing = if no_axis {
            true
        } else {
            // Check if we home an axis which needs the override.
            let mut override_homing = false;
            for axis in self.axes.chars() {
                let axis_name = axis.to_string();
                if has_param(gcmd, &axis_name)? {
                    override_homing = true;
                }
            }
            override_homing
        };

        if !override_homing {
            return self.call_prev_g28(gcmd);
        }

        // Calculate forced position (if configured).
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let mut pos = toolhead.get_position();
        let mut homing_axes = String::new();
        for (axis, loc) in self.start_pos.iter().enumerate() {
            if let Some(loc) = loc {
                if pos.len() <= axis {
                    return Err(klippy_api::KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list assignment index out of range".to_string(),
                    });
                }
                pos[axis] = *loc;
                homing_axes.push("xyz".chars().nth(axis).expect("axis < 3"));
            }
        }
        toolhead.set_position(pos, &homing_axes)?;

        // Perform homing.
        let mut context = self.template.create_template_context(None);
        let mut params: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in gcmd.get_command_parameters() {
            params.insert(PyValue::Str(k), PyValue::Str(v));
        }
        context.insert("params".to_string(), Value::py(PyValue::Dict(params)));
        context.insert(
            "rawparams".to_string(),
            Value::py(PyValue::Str(gcmd.get_raw_command_parameters())),
        );
        self.in_script.set(true);
        let res = self.template.run_gcode_from_command(Some(context));
        self.in_script.set(false);
        res
    }

    /// `self.prev_G28(gcmd)`. The handler is taken out of the cell for the
    /// call because the real `G28` pauses the reactor.
    fn call_prev_g28(&self, gcmd: &GCmd) -> KResult<()> {
        let taken = self.prev_g28.replace(None);
        match taken {
            Some(mut handler) => {
                let res = handler(gcmd);
                self.prev_g28.replace(Some(handler));
                res
            }
            None => Err(klippy_api::KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not callable".to_string(),
            }),
        }
    }
}

/// `gcmd.get(axis, None) is not None` (`homing_override.py:29,39`).
fn has_param(gcmd: &GCmd, axis: &str) -> KResult<bool> {
    let value = gcmd.get(
        axis,
        GetOpts { default: Some(PyValue::None), ..Default::default() },
    )?;
    Ok(matches!(value, PyValue::Str(_)))
}

klippy_api::printer_object!(HomingOverride);

/// `load_config(config)` (`klippy/extras/homing_override.py:65`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(HomingOverride::new(config)? as ObjectRef))
}

/// Module entry point for the `homing_override` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
