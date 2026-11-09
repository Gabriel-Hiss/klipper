//! Port of the `pca9632` module (`klippy/extras/pca9632.py`).
//!
//! Drives a PCA9632 I2C LED driver: four PWM registers plus a LEDOUT
//! enable register, with a cache of previously written registers so
//! unchanged values are not retransmitted. Colours come from an
//! [`led::LEDHelper`](crate::led::LEDHelper) (the `SET_LED` mux
//! commands); the device is initialised on `klippy:connect`.
//!
//! Dependencies are reached exactly as the reference does: the I2C
//! bus handle is built by [`bus::mcu_i2c_from_config`](crate::bus::mcu_i2c_from_config)
//! (the owner's factory, `pca9632.py:28`) and held as the owner's
//! [`McuI2cApi`](crate::bus::McuI2cApi) trait, and the LED helper is
//! built by [`led::LEDHelper::new`](crate::led::LEDHelper::new) and
//! held concrete, the same way `led::PrinterPWMLED` holds it.
//!
//! Parity notes (Python bugs reproduced):
//! - `update_leds` does not clamp: a colour component above 1.0
//!   produces a PWM value above 255. The value is passed as a Python
//!   `int` to `i2c_write`, where `bytearray([reg, val])`
//!   (`klippy/msgproto.py:78`) raises
//!   `ValueError: bytes must be in range(0, 256)`; [`PCA9632::reg_write`]
//!   reproduces that error at the same call site.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};

use crate::bus::McuI2cApi;
use crate::led::{LedColor, LedUpdateFunc, LEDHelper};

/// `BACKGROUND_PRIORITY_CLOCK` (`klippy/extras/pca9632.py:8`).
pub const BACKGROUND_PRIORITY_CLOCK: i64 = 0x7fffffff00000000;
/// `PCA9632_MODE1` (`klippy/extras/pca9632.py:11`).
pub const PCA9632_MODE1: i64 = 0x00;
/// `PCA9632_MODE2` (`klippy/extras/pca9632.py:12`).
pub const PCA9632_MODE2: i64 = 0x01;
/// `PCA9632_PWM0` (`klippy/extras/pca9632.py:13`).
pub const PCA9632_PWM0: i64 = 0x02;
/// `PCA9632_PWM1` (`klippy/extras/pca9632.py:14`).
pub const PCA9632_PWM1: i64 = 0x03;
/// `PCA9632_PWM2` (`klippy/extras/pca9632.py:15`).
pub const PCA9632_PWM2: i64 = 0x04;
/// `PCA9632_PWM3` (`klippy/extras/pca9632.py:16`).
pub const PCA9632_PWM3: i64 = 0x05;
/// `PCA9632_LEDOUT` (`klippy/extras/pca9632.py:17`).
pub const PCA9632_LEDOUT: i64 = 0x08;

/// `LED_PWM` (`klippy/extras/pca9632.py:19`).
pub const LED_PWM: i64 = 0x02;
/// `PCA9632_LED0` (`klippy/extras/pca9632.py:20`).
pub const PCA9632_LED0: i64 = 0x00;
/// `PCA9632_LED1` (`klippy/extras/pca9632.py:21`).
pub const PCA9632_LED1: i64 = 0x02;
/// `PCA9632_LED2` (`klippy/extras/pca9632.py:22`).
pub const PCA9632_LED2: i64 = 0x04;
/// `PCA9632_LED3` (`klippy/extras/pca9632.py:23`).
pub const PCA9632_LED3: i64 = 0x06;

/// The `PCA9632` object (`klippy/extras/pca9632.py:25`).
pub struct PCA9632 {
    printer: Printer,
    i2c: Arc<dyn McuI2cApi>,
    /// `self.color_map`: position of each written channel in `RGBW`.
    color_map: [usize; 4],
    /// `self.prev_regs`: register cache (`reg` -> value).
    prev_regs: GCell<IndexMap<i64, PyInt>>,
    /// `self.led_helper`; attached after `Arc` wrap so the
    /// `LEDHelper` update callback can capture the object.
    led: GCell<Option<Arc<LEDHelper>>>,
}

impl PCA9632 {
    /// `PCA9632(config)` (`klippy/extras/pca9632.py:26`): resolves
    /// the I2C bus inside, as the reference does (factory first,
    /// then the `color_order` check, then the LED helper).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        // `bus.MCU_I2C_from_config(config, default_addr=98)` (pca9632.py:28).
        let i2c = crate::bus::mcu_i2c_from_config(config, Some(98), 100000, false)?;
        Self::from_i2c(config, i2c)
    }

    /// `PCA9632.__init__` over an already-built I2C handle (the
    /// handle `bus.MCU_I2C_from_config` would return); production
    /// callers build it with
    /// [`mcu_i2c_from_config`](crate::bus::mcu_i2c_from_config)
    /// in [`new`](Self::new), which also gives tests the fake-bus
    /// seam. The LED helper creation, which needs the finished
    /// object for its update callback, happens here, in Python
    /// order: printer, `color_order` read and validation, register
    /// cache, `led.LEDHelper`, connect handler.
    pub fn from_i2c(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let color_order = config
            .get(
                "color_order",
                GetStrOpts { default: Some(Some("RGBW".to_string())), note_valid: true },
            )?
            .unwrap_or_else(|| "RGBW".to_string());
        // `if sorted(color_order) != sorted("RGBW")` (pca9632.py:30).
        if sorted_chars(&color_order) != sorted_chars("RGBW") {
            let msg = percent_format("Invalid color_order '%s'", &[PctValue::Str(color_order)])
                .map_err(|e| KlippyError::Internal {
                    class_name: e.kind.to_string(),
                    message: e.msg,
                })?;
            return Err(KlippyError::Config(msg));
        }
        // `["RGBW".index(c) for c in color_order]` (pca9632.py:32).
        // The sorted check above guarantees every `c` is in `"RGBW"`,
        // so the error arms are unreachable.
        let mut pending: Vec<usize> = Vec::new();
        for c in color_order.chars() {
            match "RGBW".find(c) {
                Some(i) => pending.push(i),
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "ValueError".to_string(),
                        message: "substring not found".to_string(),
                    })
                }
            }
        }
        let color_map: [usize; 4] = match <[usize; 4]>::try_from(pending) {
            Ok(a) => a,
            Err(_) => {
                return Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "substring not found".to_string(),
                })
            }
        };
        let this = Arc::new(PCA9632 {
            printer,
            i2c,
            color_map,
            prev_regs: GCell::new(IndexMap::new()),
            led: GCell::new(None),
        });
        // `led.LEDHelper(config, self.update_leds, 1)` (pca9632.py:34).
        // `self.update_leds` is a bound method: it keeps the object alive.
        let led_update: LedUpdateFunc = {
            let me = Arc::clone(&this);
            Arc::new(move |led_state, print_time| me.update_leds(led_state, print_time))
        };
        this.led.replace(Some(LEDHelper::new(config, led_update, 1)?));
        // `printer.register_event_handler("klippy:connect", ...)` (pca9632.py:35).
        register_connect_handler(&this);
        Ok(this)
    }

    /// The `LEDHelper` of this object.
    pub fn led_helper(&self) -> Arc<LEDHelper> {
        self.led
            .borrow()
            .clone()
            .expect("led_helper is set before construction returns")
    }

    /// `reg_write` (pca9632.py:36).
    pub fn reg_write(&self, reg: i64, val: PyInt, minclock: i64) -> KResult<()> {
        if self.prev_regs.borrow().get(&reg) == Some(&val) {
            return Ok(());
        }
        self.prev_regs.borrow_mut().insert(reg, val.clone());
        // `self.i2c.i2c_write([reg, val], ...)`; the ints become bytes
        // in the MCU encoder (msgproto.py:78).
        let data = [py_int_to_u8(&PyInt::from_i64(reg))?, py_int_to_u8(&val)?];
        self.i2c.i2c_write(
            &data,
            QuerySendOpts {
                minclock,
                reqclock: BACKGROUND_PRIORITY_CLOCK,
                retry: true,
            },
        )
    }

    /// `handle_connect` (pca9632.py:42).
    pub fn handle_connect(&self) -> KResult<()> {
        self.reg_write(PCA9632_MODE1, PyInt::from_i64(0x00), 0)?;
        // `MODE2 (DIMMING, INVERT, CHANGE ON STOP,TOTEM)` (pca9632.py:45).
        self.reg_write(PCA9632_MODE2, PyInt::from_i64(0x15), 0)?;
        // `self.led_helper.get_status()['color_data']` (pca9632.py:48).
        let color_data = self.led_helper().color_data();
        self.update_leds(&color_data, None)
    }

    /// `update_leds` (pca9632.py:49). `led_state[0]` holds the
    /// `(r, g, b, w)` floats; `print_time` is the print time or
    /// `None`.
    pub fn update_leds(&self, led_state: &[LedColor], print_time: Option<f64>) -> KResult<()> {
        let mut minclock: i64 = 0;
        if let Some(print_time) = print_time {
            minclock = self.i2c.get_mcu().print_time_to_clock(print_time);
        }
        // `color = [int(v * 255. + .5) for v in led_state[0]]` (pca9632.py:54).
        let row = match led_state.first() {
            Some(r) => r,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let color: Vec<PyInt> = row
            .iter()
            .map(|&v| py_int_from_color_component(v))
            .collect::<KResult<Vec<PyInt>>>()?;
        // `led0, led1, led2, led3 = [color[idx] for idx in ...]` (pca9632.py:55).
        let mut chan: [PyInt; 4] = [
            PyInt::from_i64(0),
            PyInt::from_i64(0),
            PyInt::from_i64(0),
            PyInt::from_i64(0),
        ];
        for (i, &idx) in self.color_map.iter().enumerate() {
            chan[i] = match color.get(idx) {
                Some(v) => v.clone(),
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list index out of range".to_string(),
                    })
                }
            };
        }
        let led0 = chan[0].clone();
        let led1 = chan[1].clone();
        let led2 = chan[2].clone();
        let led3 = chan[3].clone();
        self.reg_write(PCA9632_PWM0, led0.clone(), minclock)?;
        self.reg_write(PCA9632_PWM1, led1.clone(), minclock)?;
        self.reg_write(PCA9632_PWM2, led2.clone(), minclock)?;
        self.reg_write(PCA9632_PWM3, led3.clone(), minclock)?;
        // LEDOUT (pca9632.py:61-64).
        let mut ledout: i64 = 0;
        if !led0.is_zero() {
            ledout |= LED_PWM << PCA9632_LED0;
        }
        if !led1.is_zero() {
            ledout |= LED_PWM << PCA9632_LED1;
        }
        if !led2.is_zero() {
            ledout |= LED_PWM << PCA9632_LED2;
        }
        if !led3.is_zero() {
            ledout |= LED_PWM << PCA9632_LED3;
        }
        self.reg_write(PCA9632_LEDOUT, PyInt::from_i64(ledout), minclock)
    }
}

/// `printer.register_event_handler("klippy:connect",
/// self.handle_connect)` (pca9632.py:35).
fn register_connect_handler(obj: &Arc<PCA9632>) {
    let me = Arc::clone(obj);
    obj.printer.register_event_handler(
        EV_KLIPPY_CONNECT,
        Box::new(move |_: &[klippy_api::EventArg]| {
            me.handle_connect()?;
            Ok(PyValue::None)
        }),
    );
}

impl PrinterObject for PCA9632 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    /// `get_status` (pca9632.py:66): delegates to the LED helper.
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.led_helper().get_status(eventtime)))
    }
}

/// `load_config_prefix` (`klippy/extras/pca9632.py:69`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PCA9632::new(config)?))
}

/// Module factory (`klippy/extras/pca9632.py:69` `load_config_prefix`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..ModuleFactory::default() }
}

/// Sorted list of chars, compared as multi-sets
/// (`sorted(color_order) != sorted("RGBW")`, pca9632.py:30).
fn sorted_chars(s: &str) -> Vec<char> {
    let mut v: Vec<char> = s.chars().collect();
    v.sort_unstable();
    v
}

/// One element of `bytearray([reg, val])` (msgproto.py:78): an int in
/// `0..=255`, else `ValueError: bytes must be in range(0, 256)`.
fn py_int_to_u8(v: &PyInt) -> KResult<u8> {
    if v.sign() < 0 {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "bytes must be in range(0, 256)".to_string(),
        });
    }
    let n: u64 = v.inner().to_string().parse::<u64>().map_err(|_| KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "bytes must be in range(0, 256)".to_string(),
    })?;
    u8::try_from(n).map_err(|_| KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "bytes must be in range(0, 256)".to_string(),
    })
}

/// `int(v * 255. + .5)` (pca9632.py:54): the float ops evaluate in
/// source order, and `int()` truncates toward zero. CPython `int()` on
/// a non-finite float raises (`int(inf)`, `int(nan)`), so those cases
/// are errors here rather than calls into the finite-only helper.
fn py_int_from_color_component(v: f64) -> KResult<PyInt> {
    if v.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if v.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    Ok(trunc_float_to_bigint(v * 255.0 + 0.5))
}
