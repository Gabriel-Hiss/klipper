//! Port of the `mcp4018` module (`klippy/extras/mcp4018.py`): MCP4018
//! digipot support. A single I2C device is configured by a section of
//! the form `[mcp4018 NAME]`; the wiper value selects the tap on the
//! 127-step resistor string and is written as one byte at
//! `klippy:connect` and on demand through the mux command
//! `SET_DIGIPOT DIGIPOT=<name> WIPER=<v>`.
//!
//! The I2C transport is the `bus` module's [`McuI2cApi`]
//! (`klippy/extras/bus.py:161` `MCU_I2C`); scalar values are read
//! via the `klippy_api` config and gcode contracts. The value math
//! mirrors `klippy/extras/mcp4018.py:25-27,29-34` exactly,
//! including the float operation order (`value * 127. / scale + .5`)
//! and Python `int()` truncation.

use crate::bus::{self, McuI2cApi};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi, GetFloatOpts as GcmdGetFloatOpts, GCmd};
use klippy_api::lookup_interface;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// Help text registered for the mux command (`klippy/extras/mcp4018.py:28`).
pub const CMD_SET_DIGIPOT_HELP: &str = "Set digipot value";

/// The configuration values read by `mcp4018.__init__`
/// (`klippy/extras/mcp4018.py:12-18`).
#[derive(Clone, Debug, PartialEq)]
pub struct Mcp4018Params {
    /// `config.get_name().split()[1]`: the section's second token.
    pub name: String,
    /// `config.getfloat('scale', 1., above=0.)`.
    pub scale: f64,
    /// `config.getfloat('wiper', minval=0., maxval=scale)`.
    pub wiper: f64,
}

/// Port of the `mcp4018` class (`klippy/extras/mcp4018.py:11`).
pub struct Mcp4018 {
    i2c: Arc<dyn McuI2cApi>,
    name: String,
    scale: f64,
    start_value: f64,
}

klippy_api::printer_object!(Mcp4018);

fn py_int_to_byte_error() -> KlippyError {
    // `msgproto.PT_buffer.encode` (`klippy/msgproto.py:76-78`) runs
    // `out.extend(bytearray(v))`; CPython raises for an out-of-range
    // byte value.
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "bytearray must be in range(0, 256)".to_string(),
    }
}

/// `int(value * 127. / scale + .5)` as one I2C byte
/// (`klippy/extras/mcp4018.py:26`).
///
/// The float evaluation order is CPython's left-to-right order:
/// `((value * 127.0) / scale) + 0.5`. `int()` truncates toward zero
/// (`pycompat::trunc_float_to_bigint`); `int(nan)` raises
/// `ValueError` and `int(inf)` raises `OverflowError` with the exact
/// CPython texts (`Objects/abstract.c:PyNumber_Long`).
fn dac_byte(value: f64, scale: f64) -> KResult<u8> {
    let f = value * 127.0 / scale + 0.5;
    if f.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if f.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    let n: i64 = digits.parse().map_err(|_| py_int_to_byte_error())?;
    u8::try_from(n).map_err(|_| py_int_to_byte_error())
}

impl Mcp4018 {
    /// Read the `scale`/`wiper` config options and the digipot name
    /// (`klippy/extras/mcp4018.py:12-18`).
    pub fn read_config(config: &Config) -> KResult<Mcp4018Params> {
        // `config.getfloat('scale', 1., above=0.)` (line 12).
        let scale_opts = GetFloatOpts {
            default: Some(Some(1.0)),
            minval: None,
            maxval: None,
            above: Some(0.0),
            below: None,
            note_valid: true,
            ..Default::default()
        };
        let scale = config.getfloat("scale", scale_opts)?.unwrap_or(1.0);
        // `config.getfloat('wiper', minval=0., maxval=self.scale)` (line 13).
        let wiper_opts = GetFloatOpts {
            default: None,
            minval: Some(0.0),
            maxval: Some(scale),
            above: None,
            below: None,
            note_valid: true,
            ..Default::default()
        };
        let wiper = match config.getfloat("wiper", wiper_opts)? {
            Some(v) => v,
            None => {
                return Err(KlippyError::Config(format!(
                    "Option 'wiper' in section '{}' must be specified",
                    config.get_name()
                )))
            }
        };
        let section = config.get_name();
        let name = section
            .split_whitespace()
            .nth(1)
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?
            .to_string();
        Ok(Mcp4018Params { name, scale, wiper })
    }

    /// Create the printer object and register its callbacks:
    /// `klippy:connect` -> `handle_connect` (lines 15-16) and the
    /// mux command `SET_DIGIPOT` with the digipot name (lines
    /// 19-22).
    pub fn new(
        printer: &Printer,
        gcode: &GCode,
        i2c: Arc<dyn McuI2cApi>,
        name: String,
        scale: f64,
        start_value: f64,
    ) -> KResult<Arc<Mcp4018>> {
        let this = Arc::new(Mcp4018 {
            i2c,
            name,
            scale,
            start_value,
        });
        let conn = Arc::clone(&this);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                conn.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        let cmd = Arc::clone(&this);
        gcode.register_mux_command(
            "SET_DIGIPOT",
            "DIGIPOT",
            Some(this.name.as_str()),
            Box::new(move |gcmd: &GCmd| cmd.cmd_set_digipot(gcmd)),
            Some(CMD_SET_DIGIPOT_HELP),
        )?;
        Ok(this)
    }

    /// Production construction mirroring `mcp4018.__init__` plus the
    /// `load_config_prefix` glue (the whole of
    /// `klippy/extras/mcp4018.py`): create the I2C device with the
    /// default address, read the config, and register the callbacks.
    /// `printer.lookup_object('gcode')` is lowered to
    /// [`lookup_interface`] for `dyn GCodeApi`, matching
    /// `klippy/extras/mcp4018.py:19`.
    pub fn from_config(config: &Config) -> KResult<Arc<Mcp4018>> {
        let printer = config.get_printer();
        let i2c: Arc<dyn McuI2cApi> =
            bus::mcu_i2c_from_config(config, Some(0x2f), 100000, false)?;
        let params = Self::read_config(config)?;
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("lookup_interface with Raise returns Some or raises");
        Mcp4018::new(&printer, &gcode, i2c, params.name, params.scale, params.wiper)
    }

    /// `handle_connect` (lines 23-24).
    pub fn handle_connect(&self) -> KResult<()> {
        self.set_dac(self.start_value)
    }

    /// `set_dac` (lines 25-27).
    pub fn set_dac(&self, value: f64) -> KResult<()> {
        let val = dac_byte(value, self.scale)?;
        self.i2c.i2c_write(&[val], QuerySendOpts::default())
    }

    /// `cmd_SET_DIGIPOT` (lines 29-34).
    pub fn cmd_set_digipot(&self, gcmd: &GCmd) -> KResult<()> {
        let wiper = match gcmd.get_float(
            "WIPER",
            GcmdGetFloatOpts {
                default: None,
                minval: Some(0.0),
                maxval: Some(self.scale),
                above: None,
                below: None,
                ..Default::default()
            },
        )? {
            Some(v) => v,
            None => {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': missing WIPER",
                    gcmd.get_commandline()
                )))
            }
        };
        self.set_dac(wiper)?;
        let msg = percent_format(
            "New value for DIGIPOT = %s, wiper = %.2f",
            &[PctValue::Str(self.name.clone()), PctValue::Float(wiper)],
        )
        .map_err(|e| KlippyError::Internal {
            class_name: e.kind.to_string(),
            message: e.msg,
        })?;
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    /// `self.name` (line 18).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.scale` (line 12).
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// `self.start_value` (line 13).
    pub fn start_value(&self) -> f64 {
        self.start_value
    }
}

/// `load_config_prefix(config)` (`klippy/extras/mcp4018.py:35-36`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Mcp4018::from_config(config)? as ObjectRef))
}

/// The frozen `klippy/extras/mcp4018.py` module has only a
/// `load_config_prefix` entry point.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
