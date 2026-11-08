//! Port of the `ad5206` module (`klippy/extras/ad5206.py`).
//!
//! AD5206 is a six-channel digital potentiometer on an MCU SPI bus.
//! The Python object has no G-code commands, no `get_status`, no logs and
//! no events: it only reads its config section and programs any configured
//! `channel_N` registers at load time. The unit contract is therefore:
//!
//! - Config `scale` (`float`, default `1.`, requires `> 0`).
//! - Config `channel_1` .. `channel_6` (`float`, optional, `0. <= v <= scale`).
//! - One `spi_send([reg, value])` per configured channel, in channel order.
//!   `value` is `int(val * 256. / scale + .5)` (Python `int()` truncation).
//!
//! SPI construction and transport are delegated to `crate::bus`:
//! `bus.MCU_SPI_from_config(config, 0, pin_option="enable_pin",
//! default_speed=25000000)` (`klippy/extras/bus.py:124`) and
//! `spi_send(data, minclock=0, reqclock=0)` (`klippy/extras/bus.py:105`)
//! through the owner trait [`crate::bus::McuSpiApi`].
//!
//! Byte conversion: the Rust bus takes `&[u8]`, so each int is truncated
//! with `as u8` (wrapping modulo 256), matching the C byte domain.
//! Note `channel_value` itself still returns the full Python int: for
//! `val == scale` it is `256`, which the bus cannot represent as a
//! single byte. Python at load time takes the `"%02x"` hex path in
//! `bus.py:108` (emitting three hex digits for 256) and after config
//! takes the msgproto `bytearray` path (raising `ValueError`). The
//! `&[u8]` bus contract can reproduce neither; `256 as u8 == 0` is
//! the truncation applied here. The parity tests drive the reference
//! through the same byte boundary and keep SPI cases in range,
//! while the `channel_value` unit still asserts the `256` formula.

use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::SendOpts;
use klippy_api::object::ObjectRef;
use klippy_api::printer::ModuleFactory;
use klippy_api::printer_object;
use pycompat::text::trunc_float_to_bigint;
use crate::bus::{mcu_spi_from_config, McuSpiApi};
use std::sync::Arc;

/// AD5206 object (`klippy/extras/ad5206.py:8`).
pub struct Ad5206 {
    spi: Arc<dyn McuSpiApi>,
}

printer_object!(Ad5206);

impl Ad5206 {
    /// `ad5206.__init__(config)` with an explicit SPI handle
    /// (`klippy/extras/ad5206.py:9`). The SPI handle is constructed
    /// first, exactly as in Python, then `scale` and `channel_1..6`
    /// are read and programmed.
    pub fn new(config: &Config, spi: Arc<dyn McuSpiApi>) -> KResult<Arc<Self>> {
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
        for i in 0..6 {
            let name = format!("channel_{}", i + 1);
            let val = config.getfloat(
                &name,
                GetFloatOpts {
                    default: Some(None),
                    minval: Some(0.0),
                    maxval: Some(scale),
                    ..Default::default()
                },
            )?;
            if let Some(val) = val {
                let value = Self::channel_value(val, scale)?;
                Self::set_register_on(&spi, i as i64, value)?;
            }
        }
        Ok(Arc::new(Ad5206 { spi }))
    }

    /// Program one register (`set_register`,
    /// `klippy/extras/ad5206.py:18`): `spi_send([reg, value])`.
    pub fn set_register(&self, reg: i64, value: i64) -> KResult<()> {
        Self::set_register_on(&self.spi, reg, value)
    }

    fn set_register_on(spi: &Arc<dyn McuSpiApi>, reg: i64, value: i64) -> KResult<()> {
        spi.spi_send(&[reg as u8, value as u8], SendOpts::default())
    }

    /// `int(val * 256. / scale + .5)` (`klippy/extras/ad5206.py:17`).
    ///
    /// Float operations evaluate left to right as in Python:
    /// `(val * 256.) / scale + .5`. `int()` is Python truncation
    /// toward zero; `nan` raises `ValueError: cannot convert float NaN
    /// to integer` and `inf` raises `OverflowError: cannot convert
    /// float infinity to integer` (only reachable with `nan`/`inf`
    /// config values that pass the `minval`/`maxval`/`above` checks,
    /// e.g. a `nan` channel value).
    pub fn channel_value(val: f64, scale: f64) -> KResult<i64> {
        let v = val * 256.0 / scale + 0.5;
        let n = py_int_trunc(v)?;
        let s = n.0.to_string();
        s.parse::<i64>().map_err(|_| {
            KlippyError::Internal { class_name: "OverflowError".to_string(), message: format!("cannot convert float {v} to integer") }
        })
    }
}

fn py_int_trunc(f: f64) -> KResult<pycompat::PyInt> {
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
    Ok(trunc_float_to_bigint(f))
}

/// `load_config_prefix(config)` (`klippy/extras/ad5206.py:21`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let spi = mcu_spi_from_config(config, 0, "enable_pin", 25_000_000, None, false)?;
    Ok(Some(Ad5206::new(config, spi)?))
}

/// `module_factory()` — only `load_config_prefix` exists in Python.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: None, load_config_prefix: Some(load_config_prefix), add_early_printer_objects: None, add_printer_objects: None }
}
