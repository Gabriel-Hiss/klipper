//! Port of `klippy/extras/mcp4728.py`.
//!
//! The MCP4728 is a 4-channel 12-bit I2C DAC. The Python class reads
//! `scale` (default 1.0, must be above 0) and optional `channel_a` ..
//! `channel_d` values (each between 0 and `scale`), and configures each
//! present channel at construction time with `int(val * 4095. / scale + .5)`
//! through `set_dac`, which emits three bytes per channel over the I2C bus
//! handle (`bus.MCU_I2C_from_config(config, default_addr=0x60)`).
//!
//! The I2C handle is the `bus` module's [`McuI2c`] (`crate::bus::McuI2cApi`):
//! [`Mcp4728::new`] builds it with `bus::mcu_i2c_from_config(config,
//! Some(0x60), 100000, false)`, the Python defaults (`mcp4728.py:10`,
//! `bus.py:302`). [`Mcp4728::from_i2c`] takes the handle directly so tests
//! (and any caller that already built the bus handle) can inject a fake.
//!
//! Floating point follows the Python source line by line: the scaling is
//! `val * 4095.0 / scale + 0.5` in that order, and `int()` truncates toward
//! zero through `pycompat::text::trunc_float_to_bigint`. Non-finite
//! arguments (possible because `scale` may be `nan`/`inf` and the range
//! checks pass them) raise the CPython `int()` errors verbatim.
//!
//! `set_dac` passes the three bytes to `i2c_write` with the Python
//! defaults (`minclock=0, reqclock=0, retry=True`,
//! `QuerySendOpts::default()`); queueing until `klippy:connect` is the
//! `bus` handle's own behaviour.
//!
//! The Python module exposes only `load_config_prefix`; the Python class
//! has no `get_status`.

use std::sync::Arc;

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::QuerySendOpts;
use klippy_api::printer::ModuleFactory;
use klippy_api::ObjectRef;
use pycompat::text::trunc_float_to_bigint;

/// The MCP4728 DAC (`klippy/extras/mcp4728.py:8`).
///
/// The Python class has no `get_status`; it is not part of the printer
/// object's status surface.
pub struct Mcp4728 {
    i2c: Arc<dyn McuI2cApi>,
}

klippy_api::printer_object!(Mcp4728);

impl Mcp4728 {
    /// `mcp4728(config)` (`klippy/extras/mcp4728.py:9`).
    ///
    /// Builds the bus handle first (`bus.MCU_I2C_from_config(config,`
    /// `default_addr=0x60)`, Python defaults `default_speed=100000`,
    /// `async_write_only=False`), so bus options are read before `scale`,
    /// exactly as in Python.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let i2c = mcu_i2c_from_config(config, Some(0x60), 100000, false)?;
        Ok(Arc::new(Self::from_i2c(config, i2c)?))
    }

    /// `mcp4728.__init__` with an already-built bus handle: reads `scale`
    /// and the optional channels and configures each present channel.
    pub fn from_i2c(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Self> {
        let scale = match config.getfloat(
            "scale",
            GetFloatOpts {
                default: Some(Some(1.0)),
                above: Some(0.0),
                ..Default::default()
            },
        )? {
            Some(v) => v,
            None => unreachable!("getfloat with a `Some` default never returns None"),
        };
        let mcp = Mcp4728 { i2c };
        for (i, name) in ['a', 'b', 'c', 'd'].iter().enumerate() {
            let option = format!("channel_{name}");
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
                mcp.set_dac(i as i64, dac_value(val, scale)?)?;
            }
        }
        Ok(mcp)
    }

    /// `set_dac(self, dac, value)` (`klippy/extras/mcp4728.py:18`).
    pub fn set_dac(&self, dac: i64, value: i64) -> KResult<()> {
        self.i2c.i2c_write(
            &[
                (0x40 | (dac << 1)) as u8,
                (((value >> 8) & 0x0f) | 0x80) as u8,
                (value & 0xff) as u8,
            ],
            QuerySendOpts::default(),
        )
    }
}

/// `int(val * 4095. / scale + .5)` (`klippy/extras/mcp4728.py:17`) with
/// CPython `int()` semantics: truncation toward zero and the exact
/// `int(nan)` / `int(inf)` error texts.
fn dac_value(val: f64, scale: f64) -> KResult<i64> {
    let scaled = val * 4095.0 / scale + 0.5;
    if scaled.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if scaled.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    Ok(trunc_float_to_bigint(scaled)
        .0
        .to_string()
        .parse::<i64>()
        .unwrap_or(if scaled < 0.0 { i64::MIN } else { i64::MAX }))
}

/// `load_config_prefix` (`klippy/extras/mcp4728.py:22`):
/// `return mcp4728(config)`.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Mcp4728::new(config)? as ObjectRef))
}

/// Module factory for the `[mcp4728 <name>]` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
