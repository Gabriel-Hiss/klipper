//! Port of `klippy/extras/pca9533.py` (PCA9533 LED driver over I2C).

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use crate::led::{LEDHelper, LedColor, LedUpdateFunc};
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::GCell;
use pycompat::text::trunc_float_to_bigint;
use pycompat::value::PyValue;
use std::sync::Arc;

/// `BACKGROUND_PRIORITY_CLOCK` (`klippy/extras/pca9533.py:9`).
pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fff_ffff_0000_0000;
/// `PCA9533_PWM0` (`klippy/extras/pca9533.py:11`).
pub const PCA9533_PWM0: i64 = 0b010;
/// `PCA9533_PWM1` (`klippy/extras/pca9533.py:12`).
pub const PCA9533_PWM1: i64 = 0b100;
/// `PCA9533_PLS0` (`klippy/extras/pca9533.py:13`).
pub const PCA9533_PLS0: i64 = 0b101;

/// Port of the `PCA9533` class (`klippy/extras/pca9533.py:15`).
pub struct PCA9533 {
    /// `self.printer = printer = config.get_printer()` (`pca9533.py:17`); the
    /// attribute is kept for parity even though nothing reads it afterwards.
    #[allow(dead_code)]
    printer: Printer,
    i2c: Arc<dyn McuI2cApi>,
    led_helper: GCell<Option<Arc<LEDHelper>>>,
}

impl PCA9533 {
    /// `PCA9533.__init__(config)` (`klippy/extras/pca9533.py:16`).
    ///
    /// Builds the I2C device (`bus.MCU_I2C_from_config(config, default_addr=98)`)
    /// and the LED helper (`led.LEDHelper(config, self.update_leds, 1)`), then
    /// registers the `klippy:connect` handler.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let i2c: Arc<dyn McuI2cApi> = mcu_i2c_from_config(config, Some(98), 100000, false)?;
        PCA9533::from_parts(config, i2c)
    }

    /// `PCA9533.__init__(config)` with an already-built I2C handle. The
    /// `update` callback is the `self.update_leds` bound method: it keeps the
    /// object alive while the helper transmits.
    pub fn from_parts(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let this = Arc::new(Self { printer: printer.clone(), i2c, led_helper: GCell::new(None) });
        let update: LedUpdateFunc = {
            let this = Arc::clone(&this);
            Arc::new(move |led_state: &[LedColor], print_time: Option<f64>| {
                let state: Vec<PyValue> = led_state
                    .iter()
                    .map(|c| PyValue::Tuple(c.iter().map(|v| PyValue::Float(*v)).collect()))
                    .collect();
                this.update_leds(&PyValue::List(state), print_time)
            })
        };
        let led_helper = LEDHelper::new(config, update, 1)?;
        this.led_helper.replace(Some(led_helper));
        let handler = Arc::clone(&this);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                handler.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// The `self.led_helper` attribute.
    fn helper(&self) -> Arc<LEDHelper> {
        self.led_helper.cloned().expect("PCA9533 LEDHelper accessed before construction finished")
    }

    /// `PCA9533.handle_connect()` (`klippy/extras/pca9533.py:21`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.i2c.i2c_write(&[PCA9533_PWM0 as u8, 85], QuerySendOpts::default())?;
        self.i2c.i2c_write(&[PCA9533_PWM1 as u8, 170], QuerySendOpts::default())?;
        let status = self.helper().get_status(0.0);
        let color_data = dict_get(&status, "color_data")?;
        self.update_leds(color_data, None)
    }

    /// `PCA9533.update_leds(led_state, print_time)`
    /// (`klippy/extras/pca9533.py:25`). `led_state` is the `color_data`
    /// list (`[ (red, green, blue, white), ... ]`); only the first tuple
    /// is used, exactly as in Python.
    pub fn update_leds(&self, led_state: &PyValue, print_time: Option<f64>) -> KResult<()> {
        let first = list_get(led_state, 0)?;
        let parts = unpack4(&first)?;
        let rmap: [i64; 5] = [0, 2, 3, 1, 1];
        let mut channels = [0i64; 4];
        for (i, v) in parts.iter().enumerate() {
            let f = numeric_to_f64(v)?;
            let f = f * 4.0;
            let index = py_int_from_float(f)?;
            channels[i] = rmap_index(&rmap, index)?;
        }
        let (red, green, blue, white) = (channels[0], channels[1], channels[2], channels[3]);
        let ls0 = (white << 6) | (blue << 4) | (green << 2) | red;
        let minclock = match print_time {
            Some(t) => self.i2c.get_mcu().print_time_to_clock(t),
            None => 0,
        };
        self.i2c.i2c_write(
            &[PCA9533_PLS0 as u8, ls0 as u8],
            QuerySendOpts { minclock, reqclock: BACKGROUND_PRIORITY_CLOCK, retry: true },
        )
    }

    /// `PCA9533.get_status(eventtime)` (`klippy/extras/pca9533.py:34`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.helper().get_status(eventtime)
    }
}

impl PrinterObject for PCA9533 {
    klippy_api::printer_object_base!();

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status(eventtime)))
    }
}

fn list_get(value: &PyValue, index: usize) -> KResult<PyValue> {
    match value {
        PyValue::List(l) => l
            .get(index)
            .cloned()
            .ok_or_else(|| KlippyError::Other { class_name: "IndexError".to_string(), message: "list index out of range".to_string() }),
        PyValue::Tuple(t) => t
            .get(index)
            .cloned()
            .ok_or_else(|| KlippyError::Other { class_name: "IndexError".to_string(), message: "tuple index out of range".to_string() }),
        PyValue::Str(s) => {
            if index == 0 {
                match s.chars().next() {
                    Some(c) => {
                        let mut ch = String::new();
                        ch.push(c);
                        Ok(PyValue::Str(ch))
                    }
                    None => Err(KlippyError::Other { class_name: "IndexError".to_string(), message: "string index out of range".to_string() }),
                }
            } else {
                Err(KlippyError::Other { class_name: "IndexError".to_string(), message: "string index out of range".to_string() })
            }
        }
        PyValue::Bytes(b) => b
            .get(index)
            .map(|v| PyValue::Int(pycompat::PyInt::from_i64(*v as i64)))
            .ok_or_else(|| KlippyError::Other { class_name: "IndexError".to_string(), message: "index out of range".to_string() }),
        other => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", pycompat::value::type_name(other)),
        }),
    }
}

fn dict_get<'a>(value: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match value {
        PyValue::Dict(d) => {
            for (k, v) in d.iter() {
                if let PyValue::Str(s) = k {
                    if s == key {
                        return Ok(v);
                    }
                }
            }
            Err(KlippyError::Other { class_name: "KeyError".to_string(), message: format!("'{key}'") })
        }
        PyValue::List(_) => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "list indices must be integers or slices, not str".to_string(),
        }),
        other => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: format!("'{}' object is not subscriptable", pycompat::value::type_name(other)),
        }),
    }
}


fn unpack4(first: &PyValue) -> KResult<Vec<PyValue>> {
    match first {
        PyValue::List(l) => {
            if l.len() < 4 {
                Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: format!("not enough values to unpack (expected 4, got {})", l.len()),
                })
            } else if l.len() > 4 {
                Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: "too many values to unpack (expected 4)".to_string(),
                })
            } else {
                Ok(l.clone())
            }
        }
        PyValue::Tuple(t) => {
            if t.len() < 4 {
                Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: format!("not enough values to unpack (expected 4, got {})", t.len()),
                })
            } else if t.len() > 4 {
                Err(KlippyError::Other {
                    class_name: "ValueError".to_string(),
                    message: "too many values to unpack (expected 4)".to_string(),
                })
            } else {
                Ok(t.clone())
            }
        }
        other => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: format!("cannot unpack non-iterable {} object", pycompat::value::type_name(other)),
        }),
    }
}

fn numeric_to_f64(value: &PyValue) -> KResult<f64> {
    match value {
        PyValue::Float(f) => Ok(*f),
        PyValue::Int(_) | PyValue::Bool(_) => match value.to_f64() {
            Some(f) => Ok(f),
            None => Err(KlippyError::Other {
                class_name: "OverflowError".to_string(),
                message: "int too large to convert to float".to_string(),
            }),
        },
        PyValue::Str(_) | PyValue::Bytes(_) | PyValue::List(_) | PyValue::Tuple(_) => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: "can't multiply sequence by non-int of type 'float'".to_string(),
        }),
        other => Err(KlippyError::Other {
            class_name: "TypeError".to_string(),
            message: format!("unsupported operand type(s) for *: '{}' and 'float'", pycompat::value::type_name(other)),
        }),
    }
}

/// `int(v)` truncation (`Objects/longobject.c` via `floatobject.c`): `nan`
/// raises `ValueError`, infinities raise `OverflowError`.
fn py_int_from_float(f: f64) -> KResult<pycompat::PyInt> {
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
    Ok(trunc_float_to_bigint(f))
}

/// `rmap[n]` with Python negative-index wrap (`Objects/listobject.c`).
fn rmap_index(rmap: &[i64; 5], n: pycompat::PyInt) -> KResult<i64> {
    let text = n.0.to_string();
    let index: i64 = match text.parse() {
        Ok(v) => v,
        Err(_) => {
            return Err(KlippyError::Other { class_name: "IndexError".to_string(), message: "list index out of range".to_string() });
        }
    };
    if index >= 0 {
        match rmap.get(index as usize) {
            Some(v) => Ok(*v),
            None => Err(KlippyError::Other { class_name: "IndexError".to_string(), message: "list index out of range".to_string() }),
        }
    } else {
        let wrapped = index + 5;
        if !(0..5).contains(&wrapped) {
            Err(KlippyError::Other { class_name: "IndexError".to_string(), message: "list index out of range".to_string() })
        } else {
            Ok(rmap[wrapped as usize])
        }
    }
}

/// `load_config_prefix(config)` (`klippy/extras/pca9533.py:37`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let i2c: Arc<dyn McuI2cApi> = mcu_i2c_from_config(config, Some(98), 100000, false)?;
    Ok(Some(PCA9533::from_parts(config, i2c)? as ObjectRef))
}

/// The frozen `klippy/extras/pca9533.py` module exposes only
/// `load_config_prefix` (no `load_config`, `add_early_printer_objects` or
/// `add_printer_objects`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..Default::default() }
}
