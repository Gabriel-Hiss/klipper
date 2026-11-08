//! Port of `klippy/extras/query_adc.py`.
//!
//! `QueryADC` is a registry of named ADC pins plus the `QUERY_ADC`
//! G-code command that reports the last sampled value of one of them,
//! optionally including the resistance it implies against a pullup.
//! The module has no config options and no status fields.
//!
//! Floating point matches the Python source operation for operation:
//! `%.6f` / `%.3f` / `%.0f` rendering goes through
//! `pycompat::text::percent_format`, and the resistance division
//! through `float_true_div`, so every message matches CPython
//! byte for byte.

use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts, GetOpts, RegisterCommandOpts};
use klippy_api::gcell::GCell;
use klippy_api::mcu::AdcPin;
use klippy_api::object::ObjectRef;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::lookup_interface;
use pycompat::text::{float_true_div, percent_format, PctValue, PyError};
use pycompat::PyValue;
use std::sync::Arc;

/// `cmd_QUERY_ADC_help` (`klippy/extras/query_adc.py:16`).
pub const CMD_QUERY_ADC_HELP: &str = "Report the last value of an analog pin";

/// Map a CPython formatting/division error onto the internal-error
/// class Python would raise it as (uncaught in `cmd_QUERY_ADC`).
fn py_internal(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

/// The `QueryADC` printer object (`class QueryADC`, lines 7-32).
///
/// `printer` mirrors `self.printer`. The adc registry mirrors
/// `self.adc`, a Python dict: `register_adc` overwrites an existing
/// name in place (insertion order of other names is unchanged),
/// which `IndexMap::insert` reproduces.
pub struct QueryAdc {
    /// `self.printer` (`query_adc.py:9`).
    pub printer: Printer,
    adc: GCell<IndexMap<String, AdcPin>>,
}

/// The interface other modules call on the `query_adc` object:
/// `register_adc(name, mcu_adc)`, used by `adc_scaled.py:18,59`,
/// `adc_temperature.py:52`, and `buttons.py:114`.
pub trait QueryAdcApi: Send + Sync + 'static {
    /// `register_adc(self, name, mcu_adc)` (`query_adc.py:14-15`).
    fn register_adc(&self, name: &str, mcu_adc: AdcPin);
}

impl QueryAdcApi for QueryAdc {
    fn register_adc(&self, name: &str, mcu_adc: AdcPin) {
        QueryAdc::register_adc(self, name, mcu_adc)
    }
}

klippy_api::printer_object!(QueryAdc; dyn QueryAdcApi);

impl QueryAdc {
    /// `QueryADC.__init__(config)` (lines 8-13): the `gcode` object is
    /// looked up with [`LookupDefault::Raise`] (the Python sentinel
    /// default) and asked for its `dyn GCodeApi` interface through
    /// [`lookup_interface`]. Registers `QUERY_ADC` with
    /// `desc=cmd_QUERY_ADC_help`.
    pub fn new(printer: Printer) -> KResult<Arc<Self>> {
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("lookup_interface with Raise returns Some or raises");
        let obj = Arc::new(QueryAdc {
            printer,
            adc: GCell::new(IndexMap::new()),
        });
        let handler_obj = Arc::clone(&obj);
        gcode.register_command(
            "QUERY_ADC",
            Some(Box::new(move |gcmd: &GCmd| handler_obj.cmd_query_adc(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_QUERY_ADC_HELP.to_string()),
            },
        )?;
        Ok(obj)
    }

    /// `register_adc(self, name, mcu_adc)` (lines 14-15).
    pub fn register_adc(&self, name: &str, mcu_adc: AdcPin) {
        self.adc.borrow_mut().insert(name.to_string(), mcu_adc);
    }

    /// `cmd_QUERY_ADC(self, gcmd)` (lines 17-32).
    pub fn cmd_query_adc(&self, gcmd: &GCmd) -> KResult<()> {
        let name_value = gcmd.get(
            "NAME",
            GetOpts {
                default: Some(PyValue::None),
                ..Default::default()
            },
        )?;
        // `gcmd.get('NAME', None)` with the default `str` parser:
        // absent returns None; present applies `str()` to the value.
        let name: Option<String> = match name_value {
            PyValue::None => None,
            PyValue::Str(s) => Some(s),
            other => Some(other.str()),
        };
        let pin: Option<AdcPin> = match &name {
            Some(n) => self.adc.borrow().get(n).cloned(),
            None => None,
        };
        let Some(pin) = pin else {
            let mut names: Vec<String> = self.adc.borrow().keys().cloned().collect();
            names.sort();
            let mut objs: Vec<String> = Vec::with_capacity(names.len());
            for n in &names {
                objs.push(
                    percent_format("\"%s\"", &[PctValue::Str(n.clone())]).map_err(py_internal)?,
                );
            }
            let msg = percent_format(
                "Available ADC objects: %s",
                &[PctValue::Str(objs.join(", "))],
            )
            .map_err(py_internal)?;
            gcmd.respond_info(&msg, true);
            return Ok(());
        };
        let (timestamp, value) = pin.get_last_value();
        let mut msg = percent_format(
            "ADC object \"%s\" has value %.6f (timestamp %.3f)",
            &[
                PctValue::Str(name.expect("pin found implies a name")),
                PctValue::Float(value),
                PctValue::Float(timestamp),
            ],
        )
        .map_err(py_internal)?;
        let pullup = gcmd.get_float(
            "PULLUP",
            GetFloatOpts {
                default: Some(None),
                above: Some(0.),
                ..Default::default()
            },
        )?;
        if let Some(pullup) = pullup {
            // `v = max(.00001, min(.99999, value))`. Rust's
            // `f64::max` / `min` (NaN propagates the other operand,
            // zero-sign ties) agree with CPython's comparison-based
            // `max` / `min` for these finite nonzero bounds.
            let v = 0.00001_f64.max(0.99999_f64.min(value));
            let r = float_true_div(pullup * v, 1.0 - v).map_err(py_internal)?;
            msg.push_str(
                &percent_format(
                    "\n resistance %.3f (with %.0f pullup)",
                    &[PctValue::Float(r), PctValue::Float(pullup)],
                )
                .map_err(py_internal)?,
            );
        }
        gcmd.respond_info(&msg, true);
        Ok(())
    }
}

/// `load_config(config)` (lines 34-35).
///
/// `QueryADC.__init__` does `config.get_printer()` followed by
/// `printer.lookup_object('gcode')` with the sentinel default, so
/// the lookup uses [`LookupDefault::Raise`] and the `gcode` object
/// is asked for its `dyn GCodeApi` interface through
/// [`lookup_interface`]. A missing `gcode` object raises exactly
/// `lookup_object('gcode')`; a present object without the
/// interface is an `AttributeError` from the lookup itself.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let obj = QueryAdc::new(printer)?;
    let obj: ObjectRef = obj;
    Ok(Some(obj))
}

/// Module entry points (`load_config` is the only one the Python
/// module has).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
