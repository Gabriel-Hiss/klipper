//! Port of the `multi_pin` module (`klippy/extras/multi_pin.py`).
//!
//! A virtual chip (`[multi_pin <name>]`) that fans pin setup and output
//! calls out to a list of real pins from the `pins` option.

use klippy_api::config::{Config, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AdcCallback, AdcPinApi, AdcSampleOpts, ChipApi, DigitalOutPinApi, EndstopApi, Mcu, Pin, PinsApi,
    PwmPinApi,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::Stepper;
use klippy_api::{downcast_arc, lookup_interface, require_interface, GCell, ObjectRef};
use reactor::Completion;
use std::sync::Arc;

/// The duck-typed pin object a `[multi_pin <name>]` returns to consumers
/// (`klippy/extras/multi_pin.py:7`): the `PrinterMultiPin` itself in Python,
/// which fans pin calls out to every underlying MCU pin.
pub trait MultiPinApi: Send + Sync + 'static {
    /// `get_mcu` (`klippy/extras/multi_pin.py:35`).
    fn get_mcu(&self) -> KResult<Mcu>;
    /// `setup_max_duration` (`klippy/extras/multi_pin.py:37`).
    fn setup_max_duration(&self, max_duration: f64) -> KResult<()>;
    /// `setup_start_value` (`klippy/extras/multi_pin.py:40`).
    ///
    /// Python forwards positionally; a digital output coerces by truthiness.
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) -> KResult<()>;
    /// `setup_cycle_time` (`klippy/extras/multi_pin.py:43`).
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()>;
    /// `set_digital` (`klippy/extras/multi_pin.py:46`).
    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()>;
    /// `next_aligned_print_time` (`klippy/extras/multi_pin.py:49`).
    ///
    /// The default `allow_early=0.` is modelled as an explicit argument.
    fn next_aligned_print_time(&self, print_time: f64, allow_early: f64) -> f64;
    /// `set_pwm` (`klippy/extras/multi_pin.py:51`).
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()>;
}

/// Python class name of a stored pin, for `AttributeError` messages.
fn pin_class_name(pin: &Pin) -> &'static str {
    match pin {
        Pin::Endstop(_) => "MCU_endstop",
        Pin::DigitalOut(_) => "MCU_digital_out",
        Pin::Pwm(_) => "MCU_pwm",
        Pin::Adc(_) => "MCU_adc",
    }
}

fn missing_attr(pin: &Pin, method: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{}' object has no attribute '{}'", pin_class_name(pin), method),
    }
}

fn index_error() -> KlippyError {
    KlippyError::Internal { class_name: "IndexError".to_string(), message: "list index out of range".to_string() }
}

/// `klippy/extras/multi_pin.py:PrinterMultiPin`.
pub struct PrinterMultiPin {
    printer: Printer,
    pin_type: GCell<Option<String>>,
    pin_list: GCell<Vec<String>>,
    mcu_pins: GCell<Vec<Pin>>,
}

impl PrinterMultiPin {
    /// `PrinterMultiPin.__init__(config)` (`klippy/extras/multi_pin.py:8`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let multi = Arc::new(PrinterMultiPin {
            printer: printer.clone(),
            pin_type: GCell::new(None),
            pin_list: GCell::new(Vec::new()),
            mcu_pins: GCell::new(Vec::new()),
        });
        let chip: Arc<dyn ChipApi> = multi.clone();
        if let Err(err) = pins.register_chip("multi_pin", chip) {
            // Python: `except ppins.error: pass` (klippy/extras/multi_pin.py:11).
            match err {
                KlippyError::Other { ref class_name, .. } if class_name == "error" => {}
                err => return Err(err),
            }
        }
        let pin_list = config
            .getlist("pins", GetListOpts::default())?
            .expect("getlist with required option never yields None");
        *multi.pin_list.borrow_mut() = pin_list;
        Ok(multi)
    }

    /// `self.pin_type` (None until `setup_pin`).
    pub fn pin_type(&self) -> Option<String> {
        self.pin_type.borrow().clone()
    }

    /// `self.pin_list` (the `pins` config option).
    pub fn pin_list(&self) -> Vec<String> {
        self.pin_list.borrow().clone()
    }

    /// Number of underlying pins created by `setup_pin`.
    pub fn mcu_pin_count(&self) -> usize {
        self.mcu_pins.borrow().len()
    }

    fn mcu_pins_snapshot(&self) -> Vec<Pin> {
        self.mcu_pins.borrow().clone()
    }

    /// `self.mcu_pins[0].get_mcu()`, panicking with the Python `IndexError`
    /// text when no pin is set up (the pin-trait `get_mcu` cannot fail).
    fn first_mcu_or_panic(&self) -> Mcu {
        match self.mcu_pins_snapshot().first() {
            Some(pin) => match pin {
                Pin::Endstop(p) => p.get_mcu(),
                Pin::DigitalOut(p) => p.get_mcu(),
                Pin::Pwm(p) => p.get_mcu(),
                Pin::Adc(p) => p.get_mcu(),
            },
            None => panic!("list index out of range"),
        }
    }

    /// `setup_max_duration` (`klippy/extras/multi_pin.py:37`).
    fn fan_setup_max_duration(&self, max_duration: f64) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::DigitalOut(p) => p.setup_max_duration(max_duration),
                Pin::Pwm(p) => p.setup_max_duration(max_duration),
                Pin::Endstop(_) | Pin::Adc(_) => return Err(missing_attr(&pin, "setup_max_duration")),
            }
        }
        Ok(())
    }

    /// `setup_start_value` with float arguments (`klippy/extras/multi_pin.py:40`):
    /// Python forwards the same values to every underlying pin, and a digital
    /// output coerces them by truthiness (`not not value`).
    fn fan_setup_start_value_float(&self, start_value: f64, shutdown_value: f64) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::DigitalOut(p) => p.setup_start_value(start_value != 0.0, shutdown_value != 0.0),
                Pin::Pwm(p) => p.setup_start_value(start_value, shutdown_value),
                Pin::Endstop(_) | Pin::Adc(_) => return Err(missing_attr(&pin, "setup_start_value")),
            }
        }
        Ok(())
    }

    /// `setup_start_value` with bool arguments, as a `MCU_digital_out` consumer
    /// calls it: bools reach PWM pins as 1.0/0.0, exactly the values Python
    /// passes on to `MCU_pwm.setup_start_value`.
    fn fan_setup_start_value_bool(&self, start_value: bool, shutdown_value: bool) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::DigitalOut(p) => p.setup_start_value(start_value, shutdown_value),
                Pin::Pwm(p) => p.setup_start_value(f64::from(start_value), f64::from(shutdown_value)),
                Pin::Endstop(_) | Pin::Adc(_) => return Err(missing_attr(&pin, "setup_start_value")),
            }
        }
        Ok(())
    }

    /// `setup_cycle_time` (`klippy/extras/multi_pin.py:43`).
    fn fan_setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::Pwm(p) => p.setup_cycle_time(cycle_time, Some(hardware_pwm)),
                Pin::Endstop(_) | Pin::DigitalOut(_) | Pin::Adc(_) => {
                    return Err(missing_attr(&pin, "setup_cycle_time"))
                }
            }
        }
        Ok(())
    }

    /// `set_digital` (`klippy/extras/multi_pin.py:46`).
    fn fan_set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::DigitalOut(p) => p.set_digital(print_time, value),
                Pin::Endstop(_) | Pin::Pwm(_) | Pin::Adc(_) => return Err(missing_attr(&pin, "set_digital")),
            }
        }
        Ok(())
    }

    /// `set_pwm` (`klippy/extras/multi_pin.py:51`).
    fn fan_set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        for pin in self.mcu_pins_snapshot() {
            match &pin {
                Pin::Pwm(p) => p.set_pwm(print_time, value),
                Pin::Endstop(_) | Pin::DigitalOut(_) | Pin::Adc(_) => return Err(missing_attr(&pin, "set_pwm")),
            }
        }
        Ok(())
    }
}

impl MultiPinApi for PrinterMultiPin {
    fn get_mcu(&self) -> KResult<Mcu> {
        match self.mcu_pins_snapshot().first() {
            Some(pin) => Ok(match pin {
                Pin::Endstop(p) => p.get_mcu(),
                Pin::DigitalOut(p) => p.get_mcu(),
                Pin::Pwm(p) => p.get_mcu(),
                Pin::Adc(p) => p.get_mcu(),
            }),
            None => Err(index_error()),
        }
    }

    fn setup_max_duration(&self, max_duration: f64) -> KResult<()> {
        self.fan_setup_max_duration(max_duration)
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) -> KResult<()> {
        self.fan_setup_start_value_float(start_value, shutdown_value)
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) -> KResult<()> {
        self.fan_setup_cycle_time(cycle_time, hardware_pwm)
    }

    fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        self.fan_set_digital(print_time, value)
    }

    fn next_aligned_print_time(&self, print_time: f64, _allow_early: f64) -> f64 {
        print_time
    }

    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        self.fan_set_pwm(print_time, value)
    }
}

/// Unwrap a fan-out call for a pin trait whose methods cannot fail: Python raises
/// `AttributeError` on an underlying pin without the method, which surfaces here
/// as a panic with the same message text.
fn unwrap_fan(result: KResult<()>) {
    if let Err(err) = result {
        panic!("{}", err.message());
    }
}

/// `MCU_digital_out` view of `self` (`klippy/extras/multi_pin.py:34`): `setup_pin`
/// returns `self`, so the returned pin fans calls out like the virtual pin.
impl DigitalOutPinApi for PrinterMultiPin {
    fn get_mcu(&self) -> Mcu {
        self.first_mcu_or_panic()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        unwrap_fan(self.fan_setup_max_duration(max_duration));
    }

    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        unwrap_fan(self.fan_setup_start_value_bool(start_value, shutdown_value));
    }

    fn set_digital(&self, print_time: f64, value: bool) {
        unwrap_fan(self.fan_set_digital(print_time, value));
    }
}

/// `MCU_pwm` view of `self` (`klippy/extras/multi_pin.py:34`).
impl PwmPinApi for PrinterMultiPin {
    fn get_mcu(&self) -> Mcu {
        self.first_mcu_or_panic()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        unwrap_fan(self.fan_setup_max_duration(max_duration));
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        unwrap_fan(self.fan_setup_cycle_time(cycle_time, hardware_pwm.unwrap_or(false)));
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        unwrap_fan(self.fan_setup_start_value_float(start_value, shutdown_value));
    }

    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 {
        print_time
    }

    fn set_pwm(&self, print_time: f64, value: f64) {
        unwrap_fan(self.fan_set_pwm(print_time, value));
    }
}

/// `MCU_endstop` view of `self` (`klippy/extras/multi_pin.py:34`): Python has no
/// endstop methods on `PrinterMultiPin`, so every endstop call raises
/// `AttributeError` there; `get_mcu` exists and fans out like the others.
impl EndstopApi for PrinterMultiPin {
    fn get_mcu(&self) -> Mcu {
        self.first_mcu_or_panic()
    }

    fn add_stepper(&self, _stepper: Stepper) {
        panic!("'PrinterMultiPin' object has no attribute 'add_stepper'");
    }

    fn get_steppers(&self) -> Vec<Stepper> {
        panic!("'PrinterMultiPin' object has no attribute 'get_steppers'");
    }

    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        panic!("'PrinterMultiPin' object has no attribute 'home_start'");
    }

    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        panic!("'PrinterMultiPin' object has no attribute 'home_wait'");
    }

    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("'PrinterMultiPin' object has no attribute 'query_endstop'");
    }
}

/// `MCU_adc` view of `self` (`klippy/extras/multi_pin.py:34`): like the endstop
/// view, only `get_mcu` exists on the Python object.
impl AdcPinApi for PrinterMultiPin {
    fn get_mcu(&self) -> Mcu {
        self.first_mcu_or_panic()
    }

    fn setup_adc_sample(&self, _report_time: f64, _opts: AdcSampleOpts) {
        panic!("'PrinterMultiPin' object has no attribute 'setup_adc_sample'");
    }

    fn setup_adc_callback(&self, _callback: AdcCallback) {
        panic!("'PrinterMultiPin' object has no attribute 'setup_adc_callback'");
    }

    fn get_last_value(&self) -> (f64, f64) {
        panic!("'PrinterMultiPin' object has no attribute 'get_last_value'");
    }
}

impl ChipApi for PrinterMultiPin {
    fn setup_pin(&self, pin_type: &str, pin_params: klippy_api::mcu::PinParams) -> KResult<Pin> {
        let pins = lookup_interface::<dyn PinsApi>(&*self.printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let pin_name = pin_params.pin.clone();
        let key = format!("multi_pin {pin_name}");
        let self_arc: Arc<Self> = match self.printer.lookup_object(&key, LookupDefault::ReturnNone)? {
            None => {
                return Err(KlippyError::Other {
                    class_name: "error".to_string(),
                    message: format!("multi_pin {pin_name} not configured"),
                });
            }
            Some(obj) => {
                if let Some(other) = downcast_arc::<PrinterMultiPin>(&obj) {
                    // `pin is self`: same allocation as this object.
                    if !std::ptr::eq(Arc::as_ptr(&other), self) {
                        return other.setup_pin(pin_type, pin_params);
                    }
                    other
                } else {
                    let chip = require_interface::<dyn ChipApi>(&obj, &key)?;
                    return chip.setup_pin(pin_type, pin_params);
                }
            }
        };
        if self.pin_type.borrow().is_some() {
            return Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!("Can't setup multi_pin {pin_name} twice"),
            });
        }
        *self.pin_type.borrow_mut() = Some(pin_type.to_string());
        let invert = if pin_params.invert != 0 { "!" } else { "" };
        let pin_list = self.pin_list.borrow().clone();
        let mut created = Vec::new();
        for pin_desc in pin_list {
            created.push(pins.setup_pin(pin_type, &format!("{invert}{pin_desc}"))?);
        }
        *self.mcu_pins.borrow_mut() = created;
        // Python returns `self` (`klippy/extras/multi_pin.py:34`): the virtual
        // pin itself, fanning every call out to the underlying pins. Wrap this
        // object in the `Pin` variant matching the requested type. With no
        // underlying pins Python still returns `self`.
        Ok(match pin_type {
            "pwm" => Pin::Pwm(self_arc),
            "digital_out" => Pin::DigitalOut(self_arc),
            "endstop" => Pin::Endstop(self_arc),
            "adc" => Pin::Adc(self_arc),
            other => {
                return Err(KlippyError::Other {
                    class_name: "error".to_string(),
                    message: format!("pin type {other} not supported on mcu"),
                });
            }
        })
    }

    fn get_mcu(&self) -> Mcu {
        // Python `self.mcu_pins[0]` raises `IndexError: list index out of range` on an
        // empty list; the trait cannot fail, so the exception text is carried as a panic.
        self.first_mcu_or_panic()
    }
}

klippy_api::printer_object!(PrinterMultiPin; dyn ChipApi, dyn MultiPinApi);

/// `load_config_prefix(config)` (`klippy/extras/multi_pin.py:55`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterMultiPin::new(config)? as ObjectRef))
}

/// Module entry point for the `multi_pin <name>` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
