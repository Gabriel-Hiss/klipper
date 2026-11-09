//! Port of the `lm75` module (`klippy/extras/lm75.py`).
//!
//! Support for I2C based LM75/LM75A temperature sensors. The module
//! registers the `LM75` sensor factory with `heaters`; the object the
//! factory builds is a sensor chip ([`TemperatureSensorApi`]) that samples
//! the chip from a reactor timer and reports through the callback the
//! heater installed with `setup_callback`.

use crate::bus::{self, McuI2cApi};
use crate::heaters::{
    PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{Mcu, QuerySendOpts};
use klippy_api::object::{load_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::{EventArg, GCell};
use pycompat::text::{percent_format, round_py_nd, str_repr, PctValue};
use pycompat::value::type_name;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::{Arc, Weak};

/// `LM75_CHIP_ADDR` (`lm75.py:9`).
pub const LM75_CHIP_ADDR: i64 = 0x48;
/// `LM75_I2C_SPEED` (`lm75.py:10`).
pub const LM75_I2C_SPEED: i64 = 100000;
/// `LM75_REGS` (`lm75.py:11`).
pub const LM75_REGS: [(&str, u8); 5] = [
    ("TEMP", 0x00),
    ("CONF", 0x01),
    ("THYST", 0x02),
    ("TOS", 0x03),
    ("PRODID", 0x07),
];
/// `LM75_REPORT_TIME` (`lm75.py:18`).
pub const LM75_REPORT_TIME: f64 = 0.8;
/// `LM75_MIN_REPORT_TIME` (`lm75.py:22`).
pub const LM75_MIN_REPORT_TIME: f64 = 0.5;

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message }
}

/// `f"{class}: {message}"`, the last line of a Python traceback.
fn exc_text(e: &KlippyError) -> String {
    e.exception_line()
}

struct State {
    temp: f64,
    min_temp: f64,
    max_temp: f64,
}

/// `LM75` (`lm75.py:24`).
pub struct Lm75 {
    printer: Printer,
    name: String,
    reactor: Reactor,
    i2c: Arc<dyn McuI2cApi>,
    mcu: Mcu,
    report_time: f64,
    state: GCell<State>,
    /// `self._callback`; unset until `setup_callback` (Python has no attribute).
    callback: GCell<Option<TemperatureCallback>>,
    sample_timer: GCell<Option<Timer>>,
}

impl Lm75 {
    /// `LM75.__init__` (`lm75.py:25-38`).
    pub fn new(config: &Config) -> KResult<Arc<Lm75>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .ok_or_else(|| internal("IndexError", "list index out of range".to_string()))?
            .to_string();
        let reactor = printer.get_reactor();
        let i2c: Arc<dyn McuI2cApi> =
            bus::mcu_i2c_from_config(config, Some(LM75_CHIP_ADDR), LM75_I2C_SPEED, false)?;
        let mcu = i2c.get_mcu();
        let report_time = config
            .getfloat(
                "lm75_report_time",
                GetFloatOpts {
                    default: Some(Some(LM75_REPORT_TIME)),
                    minval: Some(LM75_MIN_REPORT_TIME),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("lm75_report_time has a default");
        let this = Arc::new(Lm75 {
            printer: printer.clone(),
            name,
            reactor: reactor.clone(),
            i2c,
            mcu,
            report_time,
            state: GCell::new(State { temp: 0.0, min_temp: 0.0, max_temp: 0.0 }),
            callback: GCell::new(None),
            sample_timer: GCell::new(None),
        });
        let weak: Weak<Lm75> = Arc::downgrade(&this);
        let timer = reactor.register_timer(
            move |eventtime| match weak.upgrade() {
                Some(this) => this
                    .sample_lm75(eventtime)
                    .unwrap_or_else(|e| e.raise()),
                None => NEVER,
            },
            NEVER,
        );
        *this.sample_timer.borrow_mut() = Some(timer);
        printer.add_object(&format!("lm75 {}", this.name), this.clone() as ObjectRef)?;
        let connect = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args: &[EventArg]| {
                connect.handle_connect();
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// `self.name` (`lm75.py:27`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `LM75.handle_connect` (`lm75.py:40-42`).
    pub fn handle_connect(&self) {
        self.init_lm75();
        let timer = self.sample_timer.borrow();
        self.reactor
            .update_timer(timer.as_ref().expect("timer registered in __init__"), NOW);
    }

    /// `LM75.degrees_from_sample` (`lm75.py:54-57`). `x` is the bytearray
    /// read from the chip; a short read raises `IndexError`.
    pub fn degrees_from_sample(&self, x: &[u8]) -> KResult<f64> {
        let (Some(&hi), Some(&lo)) = (x.first(), x.get(1)) else {
            return Err(internal("IndexError", "bytearray index out of range".to_string()));
        };
        Ok(hi as f64 + (lo >> 7) as f64 * 0.5)
    }

    /// `LM75._init_lm75` (`lm75.py:59-66`): errors are ignored.
    fn init_lm75(&self) {
        let prodid = match self.read_register("PRODID", 1) {
            Ok(data) => match data.first() {
                Some(&b) => b,
                None => return,
            },
            Err(_) => return,
        };
        let msg = percent_format(
            "lm75: Chip ID %#x",
            &[PctValue::Int(PyInt::from_i64(prodid as i64))],
        )
        .expect("constant format");
        klippy_api::log::info(&msg);
    }

    /// `LM75._sample_lm75` (`lm75.py:68-84`). The only error is the
    /// `AttributeError` Python raises when no `setup_callback` happened.
    pub fn sample_lm75(&self, _eventtime: f64) -> KResult<f64> {
        let sampled = self
            .read_register("TEMP", 2)
            .and_then(|sample| self.degrees_from_sample(&sample));
        match sampled {
            Ok(temp) => self.state.borrow_mut().temp = temp,
            Err(e) => {
                klippy_api::log::exception("lm75: Error reading data", &exc_text(&e));
                self.state.borrow_mut().temp = 0.0;
                return Ok(NEVER);
            }
        }

        let (temp, min_temp, max_temp) = {
            let st = self.state.borrow();
            (st.temp, st.min_temp, st.max_temp)
        };
        if temp < min_temp || temp > max_temp {
            let msg = percent_format(
                "LM75 temperature %0.1f outside range of %0.1f:%.01f",
                &[PctValue::Float(temp), PctValue::Float(min_temp), PctValue::Float(max_temp)],
            )
            .expect("constant format");
            self.printer.invoke_shutdown(&msg, PyValue::Dict(IndexMap::new()));
        }

        let measured_time = self.reactor.monotonic();
        // The callback may call back into the printer; keep no borrow while it runs.
        let callback = self.callback.borrow_mut().take();
        let Some(mut callback) = callback else {
            return Err(internal(
                "AttributeError",
                "'LM75' object has no attribute '_callback'".to_string(),
            ));
        };
        callback(self.mcu.estimated_print_time(measured_time), temp);
        {
            let mut slot = self.callback.borrow_mut();
            if slot.is_none() {
                *slot = Some(callback);
            }
        }
        Ok(measured_time + self.report_time)
    }

    /// `LM75.read_register` (`lm75.py:86-90`): returns the bytearray contents.
    pub fn read_register(&self, reg_name: &str, read_len: i64) -> KResult<Vec<u8>> {
        let reg = lookup_reg(reg_name)?;
        let params = self.i2c.i2c_read(&[reg], read_len, QuerySendOpts::default())?;
        response_bytes(&params)
    }

    /// `LM75.write_register` (`lm75.py:92-97`). Python wraps a scalar in a
    /// list; here `data` is already that list.
    pub fn write_register(&self, reg_name: &str, data: &[u8]) -> KResult<()> {
        let reg = lookup_reg(reg_name)?;
        let mut buf = Vec::with_capacity(data.len() + 1);
        buf.push(reg);
        buf.extend_from_slice(data);
        self.i2c.i2c_write(&buf, QuerySendOpts::default())
    }
}

fn lookup_reg(reg_name: &str) -> KResult<u8> {
    LM75_REGS
        .iter()
        .find(|(name, _)| *name == reg_name)
        .map(|(_, reg)| *reg)
        .ok_or_else(|| internal("KeyError", str_repr(reg_name)))
}

/// `bytearray(params['response'])` (`lm75.py:90`).
fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let PyValue::Dict(dict) = params else {
        return Err(internal(
            "TypeError",
            format!("'{}' object is not subscriptable", type_name(params)),
        ));
    };
    match dict.get(&PyValue::Str("response".to_string())) {
        None => Err(internal("KeyError", str_repr("response"))),
        Some(PyValue::Bytes(b)) => Ok(b.clone()),
        Some(other) => Err(internal(
            "TypeError",
            format!("cannot convert '{}' object to bytearray", type_name(other)),
        )),
    }
}

impl TemperatureSensorApi for Lm75 {
    /// `LM75.setup_minmax` (`lm75.py:44-46`).
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        st.min_temp = min_temp;
        st.max_temp = max_temp;
        Ok(())
    }

    /// `LM75.setup_callback` (`lm75.py:48-49`).
    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }

    /// `LM75.get_report_time_delta` (`lm75.py:51-52`).
    fn get_report_time_delta(&self) -> f64 {
        self.report_time
    }
}

impl PrinterObject for Lm75 {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);

    /// `LM75.get_status` (`lm75.py:99-102`).
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let temp = self.state.borrow().temp;
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(temp, 2)),
        );
        Ok(Some(PyValue::Dict(map)))
    }
}

/// `load_config(config)` (`lm75.py:105-108`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters =
        load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
    let factory: SensorFactory = Arc::new(|config: &Config| {
        Ok(Lm75::new(config)? as Arc<dyn TemperatureSensorApi>)
    });
    pheaters.add_sensor_factory("LM75", factory);
    Ok(None)
}

/// Module entry point for the `lm75` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
