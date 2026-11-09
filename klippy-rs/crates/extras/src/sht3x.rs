//! Port of the `sht3x` module (`klippy/extras/sht3x.py`): SHT3X i2c based
//! temperature and humidity sensor.
//!
//! The module registers the `SHT3X` sensor factory with `heaters` in
//! `load_config`; each sensor owns an I2C device (`bus.MCU_I2C`, the `bus`
//! port's [`McuI2cApi`]), a sampling timer, and the `sht3x <name>` printer
//! object whose status reports the last temperature and humidity.

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::{EventArg, GCell};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;

/// `SHT3X_I2C_ADDR` (`klippy/extras/sht3x.py:15`).
pub const SHT3X_I2C_ADDR: i64 = 0x44;

/// `SHT3X_CMD` (`klippy/extras/sht3x.py:17-50`), flattened into constants
/// named after the nested dict keys.
pub mod cmd {
    /// `SHT3X_CMD['MEASURE']['STRETCH_ENABLED']['HIGH_REP']`.
    pub const MEASURE_STRETCH_ENABLED_HIGH_REP: [u8; 2] = [0x2c, 0x06];
    /// `SHT3X_CMD['MEASURE']['STRETCH_ENABLED']['MED_REP']`.
    pub const MEASURE_STRETCH_ENABLED_MED_REP: [u8; 2] = [0x2c, 0x0D];
    /// `SHT3X_CMD['MEASURE']['STRETCH_ENABLED']['LOW_REP']`.
    pub const MEASURE_STRETCH_ENABLED_LOW_REP: [u8; 2] = [0x2c, 0x10];
    /// `SHT3X_CMD['MEASURE']['STRETCH_DISABLED']['HIGH_REP']`.
    pub const MEASURE_STRETCH_DISABLED_HIGH_REP: [u8; 2] = [0x24, 0x00];
    /// `SHT3X_CMD['MEASURE']['STRETCH_DISABLED']['MED_REP']`.
    pub const MEASURE_STRETCH_DISABLED_MED_REP: [u8; 2] = [0x24, 0x0B];
    /// `SHT3X_CMD['MEASURE']['STRETCH_DISABLED']['LOW_REP']`.
    pub const MEASURE_STRETCH_DISABLED_LOW_REP: [u8; 2] = [0x24, 0x16];
    /// `SHT3X_CMD['PERIODIC']['2HZ']['HIGH_REP']`.
    pub const PERIODIC_2HZ_HIGH_REP: [u8; 2] = [0x22, 0x36];
    /// `SHT3X_CMD['PERIODIC']['2HZ']['MED_REP']`.
    pub const PERIODIC_2HZ_MED_REP: [u8; 2] = [0x22, 0x20];
    /// `SHT3X_CMD['PERIODIC']['2HZ']['LOW_REP']`.
    pub const PERIODIC_2HZ_LOW_REP: [u8; 2] = [0x22, 0x2B];
    /// `SHT3X_CMD['OTHER']['STATUS']['READ']`.
    pub const OTHER_STATUS_READ: [u8; 2] = [0xF3, 0x2D];
    /// `SHT3X_CMD['OTHER']['STATUS']['CLEAN']`.
    pub const OTHER_STATUS_CLEAN: [u8; 2] = [0x30, 0x41];
    /// `SHT3X_CMD['OTHER']['SOFTRESET']`.
    pub const OTHER_SOFTRESET: [u8; 2] = [0x30, 0xA2];
    /// `SHT3X_CMD['OTHER']['HEATER']['ENABLE']`.
    pub const OTHER_HEATER_ENABLE: [u8; 2] = [0x30, 0x6D];
    /// `SHT3X_CMD['OTHER']['HEATER']['DISABLE']`.
    pub const OTHER_HEATER_DISABLE: [u8; 2] = [0x30, 0x66];
    /// `SHT3X_CMD['OTHER']['FETCH']`.
    pub const OTHER_FETCH: [u8; 2] = [0xE0, 0x00];
    /// `SHT3X_CMD['OTHER']['BREAK']`.
    pub const OTHER_BREAK: [u8; 2] = [0x30, 0x93];
}

fn fmt(format: &str, args: &[PctValue]) -> String {
    percent_format(format, args).unwrap_or_default()
}

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.to_string() }
}

/// `bytearray(params['response'])`.
fn response_bytes(params: &PyValue) -> KResult<Vec<u8>> {
    let PyValue::Dict(map) = params else {
        return Err(internal("TypeError", "object is not subscriptable"));
    };
    for (k, v) in map {
        if let PyValue::Str(s) = k {
            if s == "response" {
                return match v {
                    PyValue::Bytes(b) => Ok(b.clone()),
                    _ => Err(internal("TypeError", "cannot convert object to bytearray")),
                };
            }
        }
    }
    Err(internal("KeyError", "'response'"))
}

/// `response[index]` on a `bytearray`.
fn byte_at(response: &[u8], index: usize) -> KResult<i64> {
    match response.get(index) {
        Some(b) => Ok(i64::from(*b)),
        None => Err(internal("IndexError", "bytearray index out of range")),
    }
}

struct State {
    temp: f64,
    min_temp: f64,
    max_temp: f64,
    humidity: f64,
}

/// `SHT3X` (`klippy/extras/sht3x.py:52`).
pub struct Sht3x {
    printer: Printer,
    name: String,
    reactor: Reactor,
    i2c: Arc<dyn McuI2cApi>,
    report_time: i64,
    device_id: String,
    state: GCell<State>,
    sample_timer: Timer,
    callback: GCell<Option<TemperatureCallback>>,
}

impl Sht3x {
    /// `SHT3X.__init__` with the I2C device `bus.MCU_I2C_from_config` created
    /// (`klippy/extras/sht3x.py:53-66`).
    pub fn from_config(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<Sht3x>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let reactor = printer.get_reactor();
        let report_time = config
            .getint(
                "sht3x_report_time",
                GetIntOpts { default: Some(Some(1)), minval: Some(1), maxval: None, note_valid: true },
            )?
            .expect("default");
        let device_id = config.get("sensor_type", GetStrOpts::default())?.expect("required");
        let sht3x = Arc::new_cyclic(|weak: &std::sync::Weak<Sht3x>| {
            let weak = weak.clone();
            let sample_timer = reactor.register_timer(
                move |eventtime| match weak.upgrade() {
                    Some(sensor) => sensor.sample_sht3x(eventtime),
                    None => NEVER,
                },
                NEVER,
            );
            Sht3x {
                printer: printer.clone(),
                name,
                reactor: reactor.clone(),
                i2c,
                report_time,
                device_id,
                state: GCell::new(State { temp: 0., min_temp: 0., max_temp: 0., humidity: 0. }),
                sample_timer,
                callback: GCell::new(None),
            }
        });
        printer.add_object(&format!("sht3x {}", sht3x.name), sht3x.clone() as ObjectRef)?;
        let handler = sht3x.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args: &[EventArg]| {
                handler.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(sht3x)
    }

    /// The sensor as `heaters` sensor factories build it: `SHT3X(config)`.
    pub fn new(config: &Config) -> KResult<Arc<Sht3x>> {
        let i2c: Arc<dyn McuI2cApi> =
            mcu_i2c_from_config(config, Some(SHT3X_I2C_ADDR), 100000, false)?;
        Sht3x::from_config(config, i2c)
    }

    /// `handle_connect` (`klippy/extras/sht3x.py:67`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.init_sht3x()?;
        self.reactor.update_timer(&self.sample_timer, NOW);
        Ok(())
    }

    /// `_init_sht3x` (`klippy/extras/sht3x.py:81`).
    pub fn init_sht3x(&self) -> KResult<()> {
        // Device soft reset
        self.i2c.i2c_write(&cmd::OTHER_BREAK, QuerySendOpts::default())?;
        // Break takes ~ 1ms
        self.reactor.pause(self.reactor.monotonic() + 0.0015);
        self.i2c.i2c_write(&cmd::OTHER_SOFTRESET, QuerySendOpts::default())?;
        // Wait <=1.5ms after reset
        self.reactor.pause(self.reactor.monotonic() + 0.0015);

        let status = self.i2c.i2c_read(&cmd::OTHER_STATUS_READ, 3, QuerySendOpts::default())?;
        let response = response_bytes(&status)?;
        let mut status = byte_at(&response, 0)? << 8;
        status |= byte_at(&response, 1)?;
        let checksum = byte_at(&response, 2)?;

        if Sht3x::crc8(status) != checksum {
            log::warning("sht3x: Reading status - checksum error!");
        }

        // Enable periodic mode
        self.i2c.i2c_write(&cmd::PERIODIC_2HZ_HIGH_REP, QuerySendOpts::default())?;
        // Wait <=15.5ms for first measurement
        self.reactor.pause(self.reactor.monotonic() + 0.0155);
        Ok(())
    }

    /// The reading part of `_sample_sht3x`'s `try` block
    /// (`klippy/extras/sht3x.py:109-141`).
    fn read_measurement(&self) -> KResult<()> {
        let mut retries = 5;
        let mut params: Option<PyValue> = None;
        let mut error: Option<KlippyError> = None;
        while retries > 0 && params.is_none() {
            let opts = QuerySendOpts { retry: false, ..QuerySendOpts::default() };
            match self.i2c.i2c_read(&cmd::OTHER_FETCH, 6, opts) {
                Ok(p) => params = Some(p),
                Err(e @ KlippyError::Command(_)) => {
                    error = Some(e);
                    self.reactor.pause(self.reactor.monotonic() + 0.5);
                    retries -= 1;
                }
                Err(e) => return Err(e),
            }
        }
        let Some(params) = params else {
            return Err(error.expect("retries are exhausted only after an error"));
        };

        let response = response_bytes(&params)?;
        let mut rtemp = byte_at(&response, 0)? << 8;
        rtemp |= byte_at(&response, 1)?;
        if Sht3x::crc8(rtemp) != byte_at(&response, 2)? {
            log::warning("sht3x: Checksum error on Temperature reading!");
        } else {
            let temp = -45. + (175 * rtemp) as f64 / 65535.;
            self.state.borrow_mut().temp = temp;
            log::debug(&fmt("sht3x: Temperature %.2f ", &[PctValue::Float(temp)]));
        }

        let mut rhumid = byte_at(&response, 3)? << 8;
        rhumid |= byte_at(&response, 4)?;
        if Sht3x::crc8(rhumid) != byte_at(&response, 5)? {
            log::warning("sht3x: Checksum error on Humidity reading!");
        } else {
            let humidity = (100 * rhumid) as f64 / 65535.;
            self.state.borrow_mut().humidity = humidity;
            log::debug(&fmt("sht3x: Humidity %.2f ", &[PctValue::Float(humidity)]));
        }
        Ok(())
    }

    /// `_sample_sht3x` (`klippy/extras/sht3x.py:106`).
    pub fn sample_sht3x(&self, _eventtime: f64) -> f64 {
        if let Err(e) = self.read_measurement() {
            log::exception(
                "sht3x: Error reading data",
                &format!("{}: {}", e.class_name(), e.message()),
            );
            let mut st = self.state.borrow_mut();
            st.temp = 0.;
            st.humidity = 0.;
            return NEVER;
        }

        let (temp, min_temp, max_temp) = {
            let st = self.state.borrow();
            (st.temp, st.min_temp, st.max_temp)
        };
        if temp < min_temp || temp > max_temp {
            self.printer.invoke_shutdown(
                &fmt(
                    "sht3x: temperature %0.1f outside range of %0.1f:%.01f",
                    &[PctValue::Float(temp), PctValue::Float(min_temp), PctValue::Float(max_temp)],
                ),
                PyValue::Dict(IndexMap::new()),
            );
        }

        let measured_time = self.reactor.monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        // Taken out of the cell while it runs so the callback can re-enter the sensor.
        let taken = self.callback.borrow_mut().take();
        let Some(mut callback) = taken else {
            panic!("AttributeError: 'SHT3X' object has no attribute '_callback'");
        };
        callback(print_time, temp);
        let mut slot = self.callback.borrow_mut();
        if slot.is_none() {
            *slot = Some(callback);
        }
        drop(slot);
        measured_time + self.report_time as f64
    }

    /// `_crc8` (`klippy/extras/sht3x.py:158`): CRC-8 with polynomial
    /// x^8 + x^5 + x^4 + 1 over the two bytes of a 16 bit value. The Python
    /// does not mask the register between shifts and neither does this.
    pub fn crc8(data: i64) -> i64 {
        let polynomial: i64 = 0x31;
        let mut crc: i64 = 0xFF;
        let data_bytes = [(data >> 8) & 0xFF, data & 0xFF];
        for byte in data_bytes {
            crc ^= byte;
            for _ in 0..8 {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ polynomial;
                } else {
                    crc <<= 1;
                }
            }
        }
        crc & 0xFF
    }

    /// `self.name` (`klippy/extras/sht3x.py:55`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.deviceId` (`klippy/extras/sht3x.py:61`).
    pub fn device_id(&self) -> &str {
        &self.device_id
    }

    /// `self.report_time` (`klippy/extras/sht3x.py:60`).
    pub fn report_time(&self) -> i64 {
        self.report_time
    }

    /// `self.temp`.
    pub fn temp(&self) -> f64 {
        self.state.borrow().temp
    }

    /// `self.humidity`.
    pub fn humidity(&self) -> f64 {
        self.state.borrow().humidity
    }

    /// `self.min_temp`.
    pub fn min_temp(&self) -> f64 {
        self.state.borrow().min_temp
    }

    /// `self.max_temp`.
    pub fn max_temp(&self) -> f64 {
        self.state.borrow().max_temp
    }
}

impl TemperatureSensorApi for Sht3x {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        let mut st = self.state.borrow_mut();
        st.min_temp = min_temp;
        st.max_temp = max_temp;
        Ok(())
    }

    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }

    fn get_report_time_delta(&self) -> f64 {
        self.report_time as f64
    }
}

impl PrinterObject for Sht3x {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);

    /// `get_status` (`klippy/extras/sht3x.py:172`).
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let (temp, humidity) = {
            let st = self.state.borrow();
            (st.temp, st.humidity)
        };
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("temperature".to_string()), PyValue::Float(round_py_nd(temp, 2)));
        map.insert(PyValue::Str("humidity".to_string()), PyValue::Float(round_py_nd(humidity, 1)));
        Some(PyValue::Dict(map))
    }
}

/// `load_config` (`klippy/extras/sht3x.py:178`): registers the sensor factory.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheater =
        lookup_interface::<dyn PrinterHeatersApi>(&*printer, "heaters", LookupDefault::Raise)?
            .expect("lookup_interface with Raise returns Some or raises");
    let factory: SensorFactory =
        Arc::new(|config: &Config| Ok(Sht3x::new(config)? as Arc<dyn TemperatureSensorApi>));
    pheater.add_sensor_factory("SHT3X", factory);
    Ok(None)
}

/// Module entry points of `klippy/extras/sht3x.py`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
