//! Port of the `dac084S085` module (`klippy/extras/dac084S085.py`).
//!
//! SPI DAC DAC084S085 (4 channels, 8 bit). Section
//! `[dac084S085 name]` loads through `load_config_prefix`; the
//! constructor programs the channels given in config:
//! `scale` (default 1.0, above 0) and `channel_A`..`channel_D`
//! (default `None`, minval 0, maxval scale).

use crate::bus::{mcu_spi_from_config, McuSpiApi};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::SendOpts;
use klippy_api::printer::ModuleFactory;
use klippy_api::ObjectRef;
use std::sync::Arc;

/// DAC084S085 channel letters, in `enumerate("ABCD")` order.
const CHANNELS: [char; 4] = ['A', 'B', 'C', 'D'];

/// `klippy/extras/dac084S085.py:dac084S085`.
pub struct Dac084S085 {
    spi: Arc<dyn McuSpiApi>,
}

impl Dac084S085 {
    /// `dac084S085.__init__(config)` (`klippy/extras/dac084S085.py:10-18`).
    /// Resolves the SPI bus exactly like the reference: mode 1,
    /// `enable_pin`, 10 MHz default.
    pub fn new(config: &Config) -> KResult<Self> {
        let spi = mcu_spi_from_config(config, 1, "enable_pin", 10_000_000, None, false)?;
        Self::from_spi(config, spi)
    }

    /// Constructor with the SPI bus supplied by the caller.
    /// `load_config_prefix` and tests use this; the bus is still the
    /// one built by `MCU_SPI_from_config` in production.
    pub fn from_spi(config: &Config, spi: Arc<dyn McuSpiApi>) -> KResult<Self> {
        let scale = config.getfloat(
            "scale",
            GetFloatOpts {
                default: Some(Some(1.0)),
                above: Some(0.0),
                ..GetFloatOpts::default()
            },
        )?
        .expect("scale has a default");
        let dac = Dac084S085 { spi };
        for (chan, name) in CHANNELS.iter().enumerate() {
            let option = format!("channel_{name}");
            let val = config.getfloat(
                &option,
                GetFloatOpts {
                    default: Some(None),
                    minval: Some(0.0),
                    maxval: Some(scale),
                    ..GetFloatOpts::default()
                },
            )?;
            if let Some(val) = val {
                // `int(val * 255. / scale)` (`klippy/extras/dac084S085.py:18`).
                let f = val * 255.0 / scale;
                let value = py_int_trunc(f)?;
                dac.set_register(chan as i64, value)?;
            }
        }
        Ok(dac)
    }

    /// `dac084S085.set_register(chan, value)`
    /// (`klippy/extras/dac084S085.py:19-22`).
    pub fn set_register(&self, chan: i64, value: i64) -> KResult<()> {
        let b1 = (chan << 6) | (1 << 4) | ((value >> 4) & 0x0f);
        let b2 = (value << 4) & 0xf0;
        self.spi.spi_send(&[b1 as u8, b2 as u8], SendOpts::default())
    }
}

klippy_api::printer_object!(Dac084S085);

/// Python `int(float)` truncation toward zero
/// (`Objects/abstract.c:PyNumber_Long`), with the CPython
/// non-finite texts; values here are bounded to `0..=255` by
/// the config checks above.
fn py_int_trunc(f: f64) -> KResult<i64> {
    if f.is_nan() {
        return Err(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if f.is_infinite() {
        return Err(KlippyError::Other {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    // Exact truncation via pycompat; the decimal text round-trips
    // to i64 for the bounded range (see `core::clocksync::py_trunc_i64`).
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits.parse::<i64>().map_err(|_| KlippyError::Other {
        class_name: "OverflowError".to_string(),
        message: "cannot convert float infinity to integer".to_string(),
    })
}

/// `load_config_prefix(config)` (`klippy/extras/dac084S085.py:24`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(Dac084S085::new(config)?) as ObjectRef))
}

/// Module entry point for the `dac084S085` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
