//! Port of the `static_pwm_clock` module (`klippy/extras/static_pwm_clock.py`).
//!
//! `PrinterClockOutputPin` exposes a GPU pin as a clock: it configures a
//! PWM pin with a fixed cycle time and a 50% start value. The only
//! deferred work is `_build_config` validation, registered as an MCU
//! config callback: it checks that the requested frequency divides the
//! MCU clock without remainder and warns when the resulting pulse width
//! is outside `[40%, 60%]`.

use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::{Mcu, Pin, Pins, PwmPin};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, ObjectRef};
use pycompat::text::{as_float, float_floor_div, float_true_div, percent_format, trunc_float_to_bigint, PctValue, PyError};
use pycompat::PyInt;
use std::sync::Arc;

/// `PrinterClockOutputPin` (`klippy/extras/static_pwm_clock.py:9`).
///
/// The Python class keeps `name`, `printer`, `mcu_pin`, `mcu` and
/// `frequency` as plain attributes. `build_error` is the Rust-only
/// mirror of the deferred `_build_config` error so tests can observe
/// the callback outcome; the closure also returns the error through
/// `ConfigCallback` exactly like the Python raise.
pub struct PrinterClockOutputPin {
    name: String,
    printer: Printer,
    mcu: Mcu,
    mcu_pin: PwmPin,
    frequency: f64,
    build_error: GCell<Option<KlippyError>>,
}

klippy_api::printer_object!(PrinterClockOutputPin);

impl PrinterClockOutputPin {
    /// `PrinterClockOutputPin.__init__` (`klippy/extras/static_pwm_clock.py:10`).
    ///
    /// Follows the Python steps in order: read the section name, look up
    /// `pins`, set up the `pwm` pin, read `frequency`, configure the pin,
    /// then register `_build_config` on the pin's MCU.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let name = config.get_name();
        let printer = config.get_printer();
        let ppins: Pins = match lookup_interface::<dyn klippy_api::mcu::PinsApi>(
            &*printer,
            "pins",
            LookupDefault::Raise,
        )? {
            Some(p) => p,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "KeyError".to_string(),
                    message: "'pins'".to_string(),
                });
            }
        };
        let pin_desc = match config.get("pin", GetStrOpts::default())? {
            Some(v) => v,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'pin' option is required but returned None".to_string(),
                });
            }
        };
        let pin = ppins.setup_pin("pwm", &pin_desc)?;
        let mcu_pin = match pin {
            Pin::Pwm(p) => p,
            Pin::DigitalOut(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_digital_out' object has no attribute 'setup_cycle_time'"
                        .to_string(),
                });
            }
            Pin::Endstop(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_endstop' object has no attribute 'setup_cycle_time'"
                        .to_string(),
                });
            }
            Pin::Adc(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_adc' object has no attribute 'setup_cycle_time'".to_string(),
                });
            }
        };
        let mcu = mcu_pin.get_mcu();
        let frequency = match config.getfloat(
            "frequency",
            GetFloatOpts {
                default: Some(Some(100.0)),
                maxval: Some(520_000_000.0),
                above: Some(1.0 / 0.3),
                ..GetFloatOpts::default()
            },
        )? {
            Some(v) => v,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'frequency' option is required but returned None".to_string(),
                });
            }
        };
        mcu_pin.setup_cycle_time(1.0 / frequency, Some(true));
        mcu_pin.setup_max_duration(0.0);
        mcu_pin.setup_start_value(0.5, 0.5);
        let this = Arc::new(Self {
            name,
            printer,
            mcu,
            mcu_pin,
            frequency,
            build_error: GCell::new(None),
        });
        let callback_this = Arc::clone(&this);
        this.mcu.register_config_callback(Box::new(move || {
            let result = callback_this.build_config();
            if let Err(e) = &result {
                let _ = callback_this.build_error.replace(Some(e.clone()));
            }
            result
        }))?;
        Ok(this)
    }

    /// `self.name`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.frequency`.
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// `self.mcu`.
    pub fn mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `self.mcu_pin`.
    pub fn mcu_pin(&self) -> PwmPin {
        self.mcu_pin.clone()
    }

    /// `self.printer`.
    pub fn printer(&self) -> Printer {
        self.printer.clone()
    }

    /// The error recorded by the deferred `_build_config` callback, if any.
    pub fn build_error(&self) -> Option<KlippyError> {
        self.build_error.borrow().clone()
    }

    /// `_build_config` (`klippy/extras/static_pwm_clock.py:22`).
    pub fn build_config(&self) -> KResult<()> {
        let mcu_freq = self.mcu.seconds_to_clock(1.0);
        let cycle_ticks = py_int_floor_div(mcu_freq, self.frequency)?;
        let cycle_f = py_int_to_float(&cycle_ticks)?;
        let mcu_freq_rev = py_int_trunc(cycle_f * self.frequency)?;
        if int_cmp_py(&mcu_freq_rev, PyInt::from_i64(mcu_freq)) != std::cmp::Ordering::Equal {
            let msg = pct_format(
                "\n            # Frequency output must be without remainder, %i != %i\n            [%s]\n            frequency = %f\n            ",
                &[
                    PctValue::Int(PyInt::from_i64(mcu_freq)),
                    PctValue::Int(mcu_freq_rev),
                    PctValue::Str(self.name.clone()),
                    PctValue::Float(self.frequency),
                ],
            );
            return Err(KlippyError::Config(msg));
        }
        let value = py_int_trunc(0.5 * cycle_f)?;
        if cycle_ticks.is_zero() {
            return Err(KlippyError::Internal {
                class_name: "ZeroDivisionError".to_string(),
                message: "division by zero".to_string(),
            });
        }
        let value_f = py_int_to_float(&value)?;
        let ratio = match float_true_div(value_f, cycle_f) {
            Ok(v) => v,
            Err(e) => return Err(map_py_error(e)),
        };
        if ratio < 0.4 {
            let msg = pct_format("[%s] pulse width < 40%%", &[PctValue::Str(self.name.clone())]);
            log::warning(&msg);
        }
        if ratio > 0.6 {
            let msg = pct_format("[%s] pulse width > 60%%", &[PctValue::Str(self.name.clone())]);
            log::warning(&msg);
        }
        Ok(())
    }
}

/// `load_config_prefix` (`klippy/extras/static_pwm_clock.py:40`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = PrinterClockOutputPin::new(config)?;
    Ok(Some(obj as ObjectRef))
}

/// Module entry points matching the Python module namespace.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}

fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant static_pwm_clock format")
}

fn map_py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// `int(float)` with CPython overflow/NaN texts.
fn py_int_trunc(f: f64) -> KResult<PyInt> {
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

/// Python `int` to `float` conversion (`float(x)` on an int).
fn py_int_to_float(v: &PyInt) -> KResult<f64> {
    as_float(&PctValue::Int(v.clone())).map_err(map_py_error)
}

/// `int(mcu_freq // frequency)`: float floor division of the int operand,
/// then `int()`.
fn py_int_floor_div(mcu_freq: i64, frequency: f64) -> KResult<PyInt> {
    let d = float_floor_div(mcu_freq as f64, frequency).map_err(map_py_error)?;
    py_int_trunc(d)
}

fn int_cmp_py(a: &PyInt, b: PyInt) -> std::cmp::Ordering {
    pycompat::int::int_cmp(a, &b)
}

