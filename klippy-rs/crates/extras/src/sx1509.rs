//! Port of the `sx1509` module (`klippy/extras/sx1509.py`).

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use indexmap::IndexMap;
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    ChipApi, DigitalOutPinApi, Mcu, Pin, PinParams, PinsApi, PwmPinApi, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::{lookup_interface, GCell, ObjectRef};
use pycompat::PyValue;
use std::sync::{Arc, Weak};

pub const REG_RESET: i64 = 0x7D;
pub const REG_CLOCK: i64 = 0x1E;
pub const REG_MISC: i64 = 0x1F;
pub const REG_DIR: i64 = 0x0E;
pub const REG_DATA: i64 = 0x10;
pub const REG_PULLUP: i64 = 0x06;
pub const REG_PULLDOWN: i64 = 0x08;
pub const REG_INPUT_DISABLE: i64 = 0x00;
pub const REG_ANALOG_DRIVER_ENABLE: i64 = 0x20;

pub const REG_I_ON: [i64; 16] = [
    0x2A, 0x2D, 0x30, 0x33, 0x36, 0x3B, 0x40, 0x45, 0x4A, 0x4D, 0x50, 0x53, 0x56, 0x5B, 0x5F, 0x65,
];

fn pins_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Pins(msg.into())
}

fn py_error(kind: &'static str, msg: String) -> KlippyError {
    KlippyError::Other { class_name: kind.to_string(), message: msg }
}

fn py_pin_prefix(pin: &str) -> String {
    pin.chars().take(4).collect()
}

fn parse_sxpin(pin: &str) -> KResult<i64> {
    let parts: Vec<&str> = pin.split('_').collect();
    if parts.len() < 2 {
        return Err(py_error("IndexError", "list index out of range".to_string()));
    }
    let s = parts[1];
    match pycompat::text::int_from_string(s, 10) {
        Ok(n) => {
            let digits = n.0.to_string();
            match digits.parse::<i64>() {
                Ok(v) => Ok(v),
                Err(_) => {
                    if digits.starts_with('-') {
                        Ok(i64::MIN)
                    } else {
                        Ok(i64::MAX)
                    }
                }
            }
        }
        Err(e) => Err(py_error(e.kind, e.msg)),
    }
}

fn bitmask_for(sxpin: i64) -> KResult<i64> {
    if sxpin < 0 {
        return Err(py_error("ValueError", "negative shift count".to_string()));
    }
    if sxpin >= 63 {
        return Ok(0);
    }
    Ok(1i64 << sxpin)
}

fn py_int_255(value: f64) -> i64 {
    let prod = 255.0 * value;
    if !prod.is_finite() {
        return 0;
    }
    let digits = pycompat::text::trunc_float_to_bigint(prod).0.to_string();
    digits.parse::<i64>().unwrap_or(if prod < 0.0 { i64::MIN } else { i64::MAX })
}

/// `SX1509` (`klippy/extras/sx1509.py:24`).
pub struct SX1509 {
    printer: Printer,
    i2c: Arc<dyn McuI2cApi>,
    mcu: Mcu,
    oid: i64,
    last_clock: GCell<i64>,
    reg_dict: GCell<IndexMap<i64, i64>>,
    reg_i_on_dict: GCell<IndexMap<i64, i64>>,
    self_ref: GCell<Option<Weak<SX1509>>>,
}

impl SX1509 {
    fn default_reg_dict() -> IndexMap<i64, i64> {
        let mut m = IndexMap::new();
        m.insert(REG_DIR, 0xFFFF);
        m.insert(REG_DATA, 0);
        m.insert(REG_PULLUP, 0);
        m.insert(REG_PULLDOWN, 0);
        m.insert(REG_INPUT_DISABLE, 0);
        m.insert(REG_ANALOG_DRIVER_ENABLE, 0);
        m
    }

    fn default_reg_i_on_dict() -> IndexMap<i64, i64> {
        let mut m = IndexMap::new();
        for reg in REG_I_ON {
            m.insert(reg, 0);
        }
        m
    }

    pub fn from_parts(printer: Printer, i2c: Arc<dyn McuI2cApi>, mcu: Mcu, oid: i64) -> Arc<SX1509> {
        let sx = Arc::new(SX1509 {
            printer,
            i2c,
            mcu,
            oid,
            last_clock: GCell::new(0),
            reg_dict: GCell::new(Self::default_reg_dict()),
            reg_i_on_dict: GCell::new(Self::default_reg_i_on_dict()),
            self_ref: GCell::new(None),
        });
        sx.self_ref.with_mut(|v| *v = Some(Arc::downgrade(&sx)));
        sx
    }

    pub fn new(config: &Config) -> KResult<Arc<SX1509>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .nth(1)
            .ok_or_else(|| py_error("IndexError", "list index out of range".to_string()))?
            .to_string();
        let i2c: Arc<dyn McuI2cApi> = mcu_i2c_from_config(config, None, 400000, true)?;
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let mcu = i2c.get_mcu();
        let oid = i2c.get_oid();
        let sx = Self::from_parts(printer.clone(), i2c, mcu, oid);
        pins.register_chip(&format!("sx1509_{name}"), sx.clone() as klippy_api::mcu::Chip)?;
        let weak: Weak<SX1509> = Arc::downgrade(&sx);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                if let Some(s) = weak.upgrade() {
                    s.handle_connect().map(|_| PyValue::None)
                } else {
                    Ok(PyValue::None)
                }
            }),
        );
        Ok(sx)
    }

    pub fn handle_connect(&self) -> KResult<()> {
        self.i2c.i2c_write(&[REG_RESET as u8, 0x12], QuerySendOpts::default())?;
        self.i2c.i2c_write(&[REG_RESET as u8, 0x34], QuerySendOpts::default())?;
        self.i2c.i2c_write(&[REG_CLOCK as u8, 1 << 6], QuerySendOpts::default())?;
        self.i2c.i2c_write(&[REG_MISC as u8, 1 << 4], QuerySendOpts::default())?;
        let reactor = self.printer.get_reactor();
        let word_regs: Vec<i64> = self.reg_dict.borrow().keys().cloned().collect();
        for reg in word_regs {
            let curtime = reactor.monotonic();
            let printtime = self.mcu.estimated_print_time(curtime);
            self.send_register(reg, printtime);
        }
        let byte_regs: Vec<i64> = self.reg_i_on_dict.borrow().keys().cloned().collect();
        for reg in byte_regs {
            let curtime = reactor.monotonic();
            let printtime = self.mcu.estimated_print_time(curtime);
            self.send_register(reg, printtime);
        }
        Ok(())
    }

    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    pub fn get_oid(&self) -> i64 {
        self.oid
    }

    pub fn clear_bits_in_register(&self, reg: i64, bitmask: i64) {
        let mut dict = self.reg_dict.borrow_mut();
        if dict.contains_key(&reg) {
            let cur = *dict.get(&reg).unwrap();
            dict.insert(reg, cur & !bitmask);
            return;
        }
        drop(dict);
        let mut i_on = self.reg_i_on_dict.borrow_mut();
        if let Some(&cur) = i_on.get(&reg) {
            i_on.insert(reg, cur & !bitmask);
        }
    }

    pub fn set_bits_in_register(&self, reg: i64, bitmask: i64) {
        let mut dict = self.reg_dict.borrow_mut();
        if dict.contains_key(&reg) {
            let cur = *dict.get(&reg).unwrap();
            dict.insert(reg, cur | bitmask);
            return;
        }
        drop(dict);
        let mut i_on = self.reg_i_on_dict.borrow_mut();
        if let Some(&cur) = i_on.get(&reg) {
            i_on.insert(reg, cur | bitmask);
        }
    }

    pub fn set_register(&self, reg: i64, value: i64) {
        {
            let mut dict = self.reg_dict.borrow_mut();
            if dict.contains_key(&reg) {
                dict.insert(reg, value);
                return;
            }
        }
        let mut i_on = self.reg_i_on_dict.borrow_mut();
        if i_on.contains_key(&reg) {
            i_on.insert(reg, value);
        }
    }

    pub fn send_register(&self, reg: i64, print_time: f64) {
        let data: Vec<u8> = {
            let dict = self.reg_dict.borrow();
            if let Some(&val) = dict.get(&reg) {
                vec![(reg & 0xFF) as u8, ((val >> 8) & 0xFF) as u8, (val & 0xFF) as u8]
            } else {
                let i_on = self.reg_i_on_dict.borrow();
                if let Some(&val) = i_on.get(&reg) {
                    vec![(reg & 0xFF) as u8, (val & 0xFF) as u8]
                } else {
                    vec![(reg & 0xFF) as u8]
                }
            }
        };
        let last_clock = self.last_clock.get();
        let clock = self.mcu.print_time_to_clock(print_time);
        let _ = self.i2c.i2c_write_noack(&data, SendOpts { minclock: last_clock, reqclock: clock });
        self.last_clock.set(clock);
    }

    pub fn setup_pin_internal(
        self: &Arc<SX1509>,
        pin_type: &str,
        pin_params: PinParams,
    ) -> KResult<Pin> {
        let prefix = py_pin_prefix(&pin_params.pin);
        if pin_type == "digital_out" && prefix == "PIN_" {
            let out = Sx1509DigitalOut::new(self.clone(), pin_params)?;
            return Ok(Pin::DigitalOut(Arc::new(out)));
        }
        if pin_type == "pwm" && prefix == "PIN_" {
            let pwm = Sx1509Pwm::new(self.clone(), pin_params)?;
            return Ok(Pin::Pwm(Arc::new(pwm)));
        }
        Err(pins_error(format!(
            "Wrong pin or incompatible type: {prefix} with type {pin_type}! "
        )))
    }

    pub fn chip_setup_pin(
        self: &Arc<SX1509>,
        pin_type: &str,
        pin_params: PinParams,
    ) -> KResult<Pin> {
        self.setup_pin_internal(pin_type, pin_params)
    }

    pub fn reg_dict_snapshot(&self) -> Vec<(i64, i64)> {
        self.reg_dict.borrow().iter().map(|(k, v)| (*k, *v)).collect()
    }

    pub fn reg_i_on_dict_snapshot(&self) -> Vec<(i64, i64)> {
        self.reg_i_on_dict.borrow().iter().map(|(k, v)| (*k, *v)).collect()
    }

    pub fn last_clock(&self) -> i64 {
        self.last_clock.get()
    }
}

klippy_api::printer_object!(SX1509; dyn klippy_api::mcu::ChipApi);

impl ChipApi for SX1509 {
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        if let Some(weak) = self.self_ref.borrow().as_ref() {
            if let Some(arc) = weak.upgrade() {
                return arc.setup_pin_internal(pin_type, pin_params);
            }
        }
        Err(pins_error("SX1509 setup performed before construction completed"))
    }

    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
}

/// `SX1509_digital_out` (`klippy/extras/sx1509.py:100`).
pub struct Sx1509DigitalOut {
    sx1509: Arc<SX1509>,
    mcu: Mcu,
    sxpin: i64,
    bitmask: i64,
    pin: String,
    invert: i64,
    max_duration: Arc<GCell<f64>>,
    start_value: GCell<i64>,
    shutdown_value: GCell<i64>,
}

impl Sx1509DigitalOut {
    pub fn new(sx1509: Arc<SX1509>, pin_params: PinParams) -> KResult<Sx1509DigitalOut> {
        let mcu = sx1509.get_mcu();
        let sxpin = parse_sxpin(&pin_params.pin)?;
        let bitmask = bitmask_for(sxpin)?;
        let pin = pin_params.pin.clone();
        let invert = pin_params.invert;
        let max_duration = Arc::new(GCell::new(2.0f64));
        let shared = max_duration.clone();
        mcu.register_config_callback(Box::new(move || {
            if shared.get() != 0.0 {
                return Err(pins_error("SX1509 pins are not suitable for heaters"));
            }
            Ok(())
        }))?;
        let out = Sx1509DigitalOut {
            sx1509,
            mcu,
            sxpin,
            bitmask,
            pin,
            invert,
            max_duration,
            start_value: GCell::new(invert),
            shutdown_value: GCell::new(invert),
        };
        out.sx1509.clear_bits_in_register(REG_DIR, out.bitmask);
        Ok(out)
    }

    pub fn set_pwm(&self, print_time: f64, value: f64) {
        self.set_digital(print_time, value >= 0.5);
    }

    pub fn start_value(&self) -> i64 {
        self.start_value.get()
    }

    pub fn shutdown_value(&self) -> i64 {
        self.shutdown_value.get()
    }

    pub fn max_duration(&self) -> f64 {
        self.max_duration.get()
    }

    pub fn sxpin(&self) -> i64 {
        self.sxpin
    }

    pub fn bitmask(&self) -> i64 {
        self.bitmask
    }

    pub fn pin_name(&self) -> &str {
        &self.pin
    }
}

impl DigitalOutPinApi for Sx1509DigitalOut {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        let _ = shutdown_value;
        let start_val = (if start_value { 1 } else { 0 }) ^ self.invert;
        self.start_value.set(start_val);
        self.shutdown_value.set(self.invert);
        if start_val != 0 {
            self.sx1509.set_bits_in_register(REG_DATA, self.bitmask);
        } else {
            self.sx1509.clear_bits_in_register(REG_DATA, self.bitmask);
        }
    }

    fn set_digital(&self, print_time: f64, value: bool) {
        if (if value { 1 } else { 0 }) ^ self.invert != 0 {
            self.sx1509.set_bits_in_register(REG_DATA, self.bitmask);
        } else {
            self.sx1509.clear_bits_in_register(REG_DATA, self.bitmask);
        }
        self.sx1509.send_register(REG_DATA, print_time);
    }
}

/// `SX1509_pwm` (`klippy/extras/sx1509.py:139`).
pub struct Sx1509Pwm {
    sx1509: Arc<SX1509>,
    mcu: Mcu,
    sxpin: i64,
    bitmask: i64,
    i_on_reg: i64,
    pin: String,
    invert: i64,
    max_duration: Arc<GCell<f64>>,
    hardware_pwm: Arc<GCell<bool>>,
    cycle_time: GCell<f64>,
    start_value: GCell<f64>,
    shutdown_value: GCell<f64>,
}

impl Sx1509Pwm {
    pub fn new(sx1509: Arc<SX1509>, pin_params: PinParams) -> KResult<Sx1509Pwm> {
        let mcu = sx1509.get_mcu();
        let sxpin = parse_sxpin(&pin_params.pin)?;
        let bitmask = bitmask_for(sxpin)?;
        if !(0..16).contains(&sxpin) {
            return Err(py_error("IndexError", "list index out of range".to_string()));
        }
        let i_on_reg = REG_I_ON[sxpin as usize];
        let pin = pin_params.pin.clone();
        let invert = pin_params.invert;
        let max_duration = Arc::new(GCell::new(2.0f64));
        let hardware_pwm = Arc::new(GCell::new(false));
        let shared_max = max_duration.clone();
        let shared_hw = hardware_pwm.clone();
        mcu.register_config_callback(Box::new(move || {
            if !shared_hw.get() {
                return Err(pins_error("SX1509_pwm must have hardware_pwm enabled"));
            }
            if shared_max.get() != 0.0 {
                return Err(pins_error("SX1509 pins are not suitable for heaters"));
            }
            Ok(())
        }))?;
        let start = invert as f64;
        let pwm = Sx1509Pwm {
            sx1509,
            mcu,
            sxpin,
            bitmask,
            i_on_reg,
            pin,
            invert,
            max_duration,
            hardware_pwm,
            cycle_time: GCell::new(0.0),
            start_value: GCell::new(start),
            shutdown_value: GCell::new(start),
        };
        pwm.sx1509.set_bits_in_register(REG_INPUT_DISABLE, pwm.bitmask);
        pwm.sx1509.clear_bits_in_register(REG_PULLUP, pwm.bitmask);
        pwm.sx1509.clear_bits_in_register(REG_DIR, pwm.bitmask);
        pwm.sx1509
            .set_bits_in_register(REG_ANALOG_DRIVER_ENABLE, pwm.bitmask);
        pwm.sx1509.clear_bits_in_register(REG_DATA, pwm.bitmask);
        Ok(pwm)
    }

    pub fn start_value(&self) -> f64 {
        self.start_value.get()
    }

    pub fn shutdown_value(&self) -> f64 {
        self.shutdown_value.get()
    }

    pub fn cycle_time(&self) -> f64 {
        self.cycle_time.get()
    }

    pub fn hardware_pwm(&self) -> bool {
        self.hardware_pwm.get()
    }

    pub fn sxpin(&self) -> i64 {
        self.sxpin
    }

    pub fn bitmask(&self) -> i64 {
        self.bitmask
    }

    pub fn i_on_reg(&self) -> i64 {
        self.i_on_reg
    }
}

impl PwmPinApi for Sx1509Pwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    fn setup_max_duration(&self, max_duration: f64) {
        self.max_duration.set(max_duration);
    }

    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        self.cycle_time.set(cycle_time);
        self.hardware_pwm.set(hardware_pwm.unwrap_or(false));
    }

    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let mut s = start_value;
        let mut sd = shutdown_value;
        if self.invert != 0 {
            s = 1.0 - s;
            sd = 1.0 - sd;
        }
        let s = s.min(1.0).max(0.0);
        let sd = sd.min(1.0).max(0.0);
        self.start_value.set(s);
        self.shutdown_value.set(sd);
        let n = py_int_255(s);
        self.sx1509.set_register(self.i_on_reg, (!n) & 0xFF);
    }

    fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64 {
        let _ = allow_early;
        print_time
    }

    fn set_pwm(&self, print_time: f64, value: f64) {
        let n = py_int_255(value);
        if self.invert == 0 {
            self.sx1509.set_register(self.i_on_reg, !n);
        } else {
            self.sx1509.set_register(self.i_on_reg, n & 0xFF);
        }
        self.sx1509.send_register(self.i_on_reg, print_time);
    }
}

fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let sx = SX1509::new(config)?;
    Ok(Some(sx as ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
