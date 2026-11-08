//! Port of the `mcp4451` module (`klippy/extras/mcp4451.py`).
//!
//! The MCP4451 is an I2C quad digipot. Construction configures two
//! control registers and then any configured wiper registers, all at
//! config time. There is no G-code or status output.

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::ModuleFactory;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::{printer_object, ObjectRef};
use pycompat::text::trunc_float_to_bigint;
use std::sync::Arc;

/// Wiper registers, matching Python `WiperRegisters`
/// (`klippy/extras/mcp4451.py:8`).
pub const WIPER_REGISTERS: [i64; 4] = [0x00, 0x01, 0x06, 0x07];

/// `klippy/extras/mcp4451.py:mcp4451`.
pub struct Mcp4451 {
    i2c: Arc<dyn McuI2cApi>,
}

impl Mcp4451 {
    /// `mcp4451.__init__(config)` (`klippy/extras/mcp4451.py:11`), with the
    /// I2C device injected. In the printer the device comes from
    /// `bus.MCU_I2C_from_config` (see `load_config_prefix`); tests drive
    /// this constructor with a fake device.
    pub fn new(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Self> {
        let i2c_addr = i2c.get_i2c_address();
        if !(44..=47).contains(&i2c_addr) {
            return Err(KlippyError::config(
                "mcp4451 address must be between 44 and 47",
            ));
        }
        let scale = config
            .getfloat(
                "scale",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(1.0);
        let obj = Mcp4451 { i2c };
        obj.set_register(0x04, 0xff)?;
        obj.set_register(0x0a, 0xff)?;
        for (i, &reg) in WIPER_REGISTERS.iter().enumerate() {
            let option = format!("wiper_{i}");
            let val = config.getfloat(
                &option,
                GetFloatOpts {
                    default: Some(None),
                    minval: Some(0.0),
                    maxval: Some(scale),
                    ..Default::default()
                },
            )?;
            if let Some(val) = val {
                let value = wiper_value(val, scale)?;
                obj.set_register(reg, value)?;
            }
        }
        Ok(obj)
    }

    /// `mcp4451.set_register(reg, value)` (`klippy/extras/mcp4451.py:26`).
    /// The byte range check reproduces the packing the Python bus layer
    /// performs when sending (`PT_string.encode` -> `bytearray(v)`),
    /// which raises `ValueError("byte must be in range(0, 256)")` for
    /// values outside 0..=255. The `__init__` path only sends in-range
    /// values (0xff and wiper values <= 255); larger arguments reach the
    /// same error as in the reference.
    pub fn set_register(&self, reg: i64, value: i64) -> KResult<()> {
        let first = (reg << 4) | ((value >> 8) & 0x03);
        let bytes = bytes_from_ints(&[first, value])?;
        self.i2c.i2c_write(&bytes, QuerySendOpts::default())
    }
}

/// CPython `bytearray(v)` packing of the ints `set_register` sends.
fn bytes_from_ints(vals: &[i64]) -> KResult<Vec<u8>> {
    let mut out = Vec::with_capacity(vals.len());
    for &v in vals {
        if !(0..=255).contains(&v) {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "byte must be in range(0, 256)".to_string(),
            });
        }
        out.push(v as u8);
    }
    Ok(out)
}

printer_object!(Mcp4451);

/// Python `int(val * 255. / scale + .5)` (`klippy/extras/mcp4451.py:24`),
/// including the errors the conversion raises for NaN/infinite results.
fn wiper_value(val: f64, scale: f64) -> KResult<i64> {
    let f = val * 255.0 / scale + 0.5;
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
    Ok(digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX }))
}

/// `load_config_prefix(config)` (`klippy/extras/mcp4451.py:29`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let i2c: Arc<dyn McuI2cApi> = mcu_i2c_from_config(config, None, 100000, false)?;
    Ok(Some(Arc::new(Mcp4451::new(config, i2c)?) as ObjectRef))
}

/// Module entry point; `[mcp4451 NAME]` sections dispatch here.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
