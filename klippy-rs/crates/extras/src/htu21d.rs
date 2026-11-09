//! Port of `klippy/extras/htu21d.py`.
//!
//! HTU21D(F)/Si7013/Si7020/Si7021/SHT21 I2C temperature (and humidity)
//! sensor. The sensor is constructed by the `heaters` module through the
//! factory registered by [`load_config`]; it is also a printer object named
//! `"htu21d <name>"` whose status is `{temperature, humidity}`.
//!
//! Python bugs are reproduced, not fixed:
//! - `i2c_write([WRITE], registerData)` passes the register value as the
//!   `minclock` argument of `MCU_I2C.i2c_write`, not as data (line 156).
//! - The humidity temperature-compensation assignment is split across two
//!   lines, so the second line is a discarded expression statement and the
//!   humidity is never compensated (lines 218-219).

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::lookup_interface;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::{Arc, Weak};

/// `HTU21D_I2C_ADDR = 0x40` (line 22).
pub const HTU21D_I2C_ADDR: i64 = 0x40;
/// Default I2C speed passed to `bus.MCU_I2C_from_config` (line 95).
pub const HTU21D_DEFAULT_SPEED: i64 = 100000;

/// `HTU21D_COMMANDS` (lines 24-35), in definition order.
pub const HTU21D_COMMANDS_TEMP: i64 = 0xE3;
/// `HTU21D_COMMANDS['HTU21D_HUMID']` (line 26).
pub const HTU21D_COMMANDS_HUMID: i64 = 0xE5;
/// `HTU21D_COMMANDS['HTU21D_TEMP_NH']` (line 27).
pub const HTU21D_COMMANDS_TEMP_NH: i64 = 0xF3;
/// `HTU21D_COMMANDS['HTU21D_HUMID_NH']` (line 28).
pub const HTU21D_COMMANDS_HUMID_NH: i64 = 0xF5;
/// `HTU21D_COMMANDS['WRITE']` (line 29).
pub const HTU21D_COMMANDS_WRITE: i64 = 0xE6;
/// `HTU21D_COMMANDS['READ']` (line 30).
pub const HTU21D_COMMANDS_READ: i64 = 0xE7;
/// `HTU21D_COMMANDS['RESET']` (line 31).
pub const HTU21D_COMMANDS_RESET: i64 = 0xFE;
/// `HTU21D_COMMANDS['SERIAL']` (line 32).
pub const HTU21D_COMMANDS_SERIAL: [i64; 4] = [0xFA, 0x0F, 0xFC, 0xC9];
/// `HTU21D_COMMANDS['FIRMWARE_READ']` (line 33).
pub const HTU21D_COMMANDS_FIRMWARE_READ: [i64; 2] = [0x84, 0xB8];

/// `HTU21D_RESOLUTION_MASK = 0x7E` (line 37).
pub const HTU21D_RESOLUTION_MASK: i64 = 0x7E;

/// `HTU21D_RESOLUTIONS` (lines 38-43) in definition order. The order is
/// observable in the "Invalid HTU21D Resolution" error message.
pub const HTU21D_RESOLUTIONS: [(&str, i64); 4] = [
    ("TEMP14_HUM12", 0),
    ("TEMP13_HUM10", 0x80),
    ("TEMP12_HUM08", 0x01),
    ("TEMP11_HUM11", 0x81),
];

/// `ID_MAP` (lines 45-52).
pub fn id_map() -> IndexMap<i64, &'static str> {
    let mut m = IndexMap::new();
    m.insert(0x0D, "SI7013");
    m.insert(0x14, "SI7020");
    m.insert(0x15, "SI7021");
    m.insert(0x31, "SHT21");
    m.insert(0x01, "SHT21");
    m.insert(0x32, "HTU21D");
    m
}

/// Conversion times per chip and resolution (lines 57-83):
/// `<CHIP> -> <RESOLUTION> -> [temp_time, humidity_time]`.
/// Insertion order matches `HTU21D_DEVICES` (SI7013, SI7020, SI7021, SHT21,
/// HTU21D); `load_config` iterates it in this order.
pub fn htu21d_devices() -> IndexMap<&'static str, IndexMap<&'static str, [f64; 2]>> {
    fn entry(
        temp14: [f64; 2],
        temp13: [f64; 2],
        temp12: [f64; 2],
        temp11: [f64; 2],
    ) -> IndexMap<&'static str, [f64; 2]> {
        let mut m = IndexMap::new();
        m.insert("TEMP14_HUM12", temp14);
        m.insert("TEMP13_HUM10", temp13);
        m.insert("TEMP12_HUM08", temp12);
        m.insert("TEMP11_HUM11", temp11);
        m
    }
    let mut m = IndexMap::new();
    m.insert("SI7013", entry([0.11, 0.12], [0.7, 0.5], [0.4, 0.4], [0.3, 0.7]));
    m.insert("SI7020", entry([0.11, 0.12], [0.7, 0.5], [0.4, 0.4], [0.3, 0.7]));
    m.insert("SI7021", entry([0.11, 0.12], [0.7, 0.5], [0.4, 0.4], [0.3, 0.7]));
    m.insert("SHT21", entry([0.85, 0.29], [0.43, 0.9], [0.22, 0.4], [0.11, 0.15]));
    m.insert("HTU21D", entry([0.50, 0.16], [0.25, 0.5], [0.13, 0.3], [0.12, 0.8]));
    m
}

/// `HTU21D_TEMP_COEFFICIENT = -0.15` (line 86).
pub const HTU21D_TEMP_COEFFICIENT: f64 = -0.15;
/// `HTU21D_CRC8_POLYNOMINAL = 0x13100` (line 88).
pub const HTU21D_CRC8_POLYNOMINAL: i64 = 0x13100;


// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("fixed htu21d format string is valid")
}

fn pct_int(fmt: &str, value: i64) -> String {
    pct(fmt, &[PctValue::Int(PyInt::from(value))])
}

fn pct_float(fmt: &str, value: f64) -> String {
    pct(fmt, &[PctValue::Float(value)])
}

/// Extract `params['response']` as bytes, mirroring
/// `bytearray(params['response'])` (lines 133, 153, 174, 195).
fn response_bytes(params: &PyValue) -> Result<Vec<u8>, KlippyError> {
    let dict = match params {
        PyValue::Dict(d) => d,
        other => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!(
                    "bytearray() argument must be an iterable, not {}",
                    type_name_of(other)
                ),
            });
        }
    };
    let key = PyValue::Str("response".to_string());
    let resp = match dict.get(&key) {
        Some(v) => v,
        None => {
            return Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: "'response'".to_string(),
            });
        }
    };
    match resp {
        PyValue::List(items) => {
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                match item {
                    PyValue::Int(n) => {
                        let v = n.0.to_string().parse::<i64>().unwrap_or(-1);
                        if !(0..=255).contains(&v) {
                            return Err(KlippyError::Internal {
                                class_name: "ValueError".to_string(),
                                message: format!("byte must be in range(0, 256): {v}"),
                            });
                        }
                        out.push(v as u8);
                    }
                    other => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".to_string(),
                            message: format!(
                                "'{}' object cannot be interpreted as an integer",
                                type_name_of(other)
                            ),
                        });
                    }
                }
            }
            Ok(out)
        }
        PyValue::Bytes(b) => Ok(b.clone()),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "bytearray() argument must be an iterable, not {}",
                type_name_of(other)
            ),
        }),
    }
}

fn type_name_of(v: &PyValue) -> &'static str {
    match v {
        PyValue::None => "NoneType",
        PyValue::Bool(_) => "bool",
        PyValue::Int(_) => "int",
        PyValue::Float(_) => "float",
        PyValue::Str(_) => "str",
        PyValue::Bytes(_) => "bytes",
        PyValue::List(_) => "list",
        PyValue::Tuple(_) => "tuple",
        PyValue::Dict(_) => "dict",
        PyValue::Host(_) => "object",
    }
}

/// `response[i]` on a `bytearray`; out-of-range mirrors Python's
/// `IndexError: bytearray index out of range`.
fn byte_at(response: &[u8], i: usize) -> Result<i64, KlippyError> {
    response.get(i).map(|b| *b as i64).ok_or_else(|| KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "bytearray index out of range".to_string(),
    })
}

/// `HTU21D_DEVICES[device_id][resolution]`; a missing key mirrors Python's
/// `KeyError: '<key>'` (repr of the key).
#[allow(clippy::type_complexity)]
const CONVERSION_TIMES: [(&str, [(&str, [f64; 2]); 4]); 5] = [
    ("SI7013", [("TEMP14_HUM12", [0.11, 0.12]), ("TEMP13_HUM10", [0.7, 0.5]), ("TEMP12_HUM08", [0.4, 0.4]), ("TEMP11_HUM11", [0.3, 0.7])]),
    ("SI7020", [("TEMP14_HUM12", [0.11, 0.12]), ("TEMP13_HUM10", [0.7, 0.5]), ("TEMP12_HUM08", [0.4, 0.4]), ("TEMP11_HUM11", [0.3, 0.7])]),
    ("SI7021", [("TEMP14_HUM12", [0.11, 0.12]), ("TEMP13_HUM10", [0.7, 0.5]), ("TEMP12_HUM08", [0.4, 0.4]), ("TEMP11_HUM11", [0.3, 0.7])]),
    ("SHT21", [("TEMP14_HUM12", [0.85, 0.29]), ("TEMP13_HUM10", [0.43, 0.9]), ("TEMP12_HUM08", [0.22, 0.4]), ("TEMP11_HUM11", [0.11, 0.15])]),
    ("HTU21D", [("TEMP14_HUM12", [0.50, 0.16]), ("TEMP13_HUM10", [0.25, 0.5]), ("TEMP12_HUM08", [0.13, 0.3]), ("TEMP11_HUM11", [0.12, 0.8])]),
];

fn conversion_times(device_id: &str, resolution: &str) -> Result<[f64; 2], KlippyError> {
    let by_res = CONVERSION_TIMES.iter().find(|(chip, _)| *chip == device_id).ok_or_else(|| KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: pycompat::text::str_repr(device_id),
    })?;
    by_res.1.iter().find(|(res, _)| *res == resolution).map(|(_, t)| *t).ok_or_else(|| KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: pycompat::text::str_repr(resolution),
    })
}

fn resolution_value(resolution: &str) -> Option<i64> {
    HTU21D_RESOLUTIONS.iter().find(|(k, _)| *k == resolution).map(|(_, v)| *v)
}

// ---------------------------------------------------------------------------
// HTU21D sensor object.
// ---------------------------------------------------------------------------

/// The humidity in the type Python keeps it in. The clamp on lines 209-214
/// assigns the ints `0` and `100`, the conversion assigns a float, and
/// `get_status` returns whichever type the attribute holds.
#[derive(Clone, Copy)]
enum Humidity {
    Int(i64),
    Float(f64),
}

impl Humidity {
    fn as_f64(self) -> f64 {
        match self {
            Humidity::Int(v) => v as f64,
            Humidity::Float(v) => v,
        }
    }

    fn to_py(self) -> PyValue {
        match self {
            Humidity::Int(v) => PyValue::Int(PyInt::from(v)),
            Humidity::Float(v) => PyValue::Float(v),
        }
    }
}

struct State {
    temp: f64,
    humidity: Humidity,
    min_temp: f64,
    max_temp: f64,
    callback: Option<TemperatureCallback>,
}

/// `HTU21D` (`klippy/extras/htu21d.py:90`).
pub struct HTU21D {
    printer: Printer,
    name: String,
    reactor: Reactor,
    i2c: Arc<dyn McuI2cApi>,
    hold_master_mode: bool,
    resolution: String,
    report_time: i64,
    device_id: String,
    state: GCell<State>,
    sample_timer: GCell<Option<Timer>>,
}

impl HTU21D {
    /// `HTU21D(config)` (lines 91-108). The I2C bus comes from
    /// `bus.MCU_I2C_from_config(config, default_addr=0x40,
    /// default_speed=100000)` (lines 95-96), which is resolved inside
    /// this constructor as the reference does.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let i2c: Arc<dyn McuI2cApi> =
            mcu_i2c_from_config(config, Some(HTU21D_I2C_ADDR), HTU21D_DEFAULT_SPEED, false)?;
        Self::from_parts(config, i2c)
    }

    /// Construct with an already built I2C bus. Used by tests to inject a
    /// fake `McuI2cApi`; production construction goes through [`HTU21D::new`].
    pub fn from_parts(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let section = config.get_name();
        // config.get_name().split()[-1] (line 93).
        let name = section
            .split_whitespace()
            .last()
            .map(|s| s.to_string())
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?;
        let reactor = printer.get_reactor();
        let hold_master_mode = config
            .getboolean(
                "htu21d_hold_master",
                GetBoolOpts { default: Some(Some(false)), note_valid: true },
            )?
            .unwrap_or(false);
        let resolution = config
            .get(
                "htu21d_resolution",
                GetStrOpts { default: Some(Some("TEMP12_HUM08".to_string())), note_valid: true },
            )?
            .unwrap_or_else(|| "TEMP12_HUM08".to_string());
        let report_time = config
            .getint(
                "htu21d_report_time",
                GetIntOpts {
                    default: Some(Some(30)),
                    minval: Some(5),
                    maxval: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(30);
        if resolution_value(&resolution).is_none() {
            let valid = HTU21D_RESOLUTIONS
                .iter()
                .map(|(k, _)| *k)
                .collect::<Vec<_>>()
                .join("|");
            return Err(KlippyError::Config(pct(
                "Invalid HTU21D Resolution. Valid are %s",
                &[PctValue::Str(valid)],
            )));
        }
        let device_id = config
            .get("sensor_type", GetStrOpts::default())?
            .ok_or_else(|| {
                KlippyError::Config(format!(
                    "Option 'sensor_type' in section '{section}' must be specified"
                ))
            })?;

        let obj = Arc::new(HTU21D {
            printer,
            name,
            reactor: reactor.clone(),
            i2c,
            hold_master_mode,
            resolution,
            report_time,
            device_id,
            state: GCell::new(State {
                temp: 0.0,
                humidity: Humidity::Float(0.0),
                min_temp: 0.0,
                max_temp: 0.0,
                callback: None,
            }),
            sample_timer: GCell::new(None),
        });
        // self.sample_timer = self.reactor.register_timer(self._sample_htu21d)
        // (line 105).
        let weak: Weak<HTU21D> = Arc::downgrade(&obj);
        let timer = reactor.register_timer(
            move |eventtime| match weak.upgrade() {
                Some(o) => o.sample(eventtime),
                None => NEVER,
            },
            NEVER,
        );
        *obj.sample_timer.borrow_mut() = Some(timer);
        // self.printer.add_object("htu21d " + self.name, self) (line 106).
        let object_name = format!("htu21d {}", obj.name);
        let object_ref: ObjectRef = obj.clone();
        obj.printer.add_object(&object_name, object_ref)?;
        // self.printer.register_event_handler("klippy:connect",
        //     self.handle_connect) (line 107-108).
        let weak2: Weak<HTU21D> = Arc::downgrade(&obj);
        obj.printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                if let Some(o) = weak2.upgrade() {
                    o.handle_connect()?;
                }
                Ok(PyValue::None)
            }),
        );
        Ok(obj)
    }

    /// `handle_connect` (lines 110-112).
    pub fn handle_connect(&self) -> KResult<()> {
        self.init_htu21d()?;
        if let Some(timer) = self.sample_timer.borrow().as_ref() {
            self.reactor.update_timer(timer, NOW);
        }
        Ok(())
    }

    /// `setup_minmax(min_temp, max_temp)` (lines 114-116).
    pub fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        let mut s = self.state.borrow_mut();
        s.min_temp = min_temp;
        s.max_temp = max_temp;
    }

    /// `setup_callback(cb)` (lines 118-119).
    pub fn setup_callback(&self, cb: TemperatureCallback) {
        self.state.borrow_mut().callback = Some(cb);
    }

    /// `get_report_time_delta()` (lines 121-122).
    pub fn get_report_time_delta(&self) -> i64 {
        self.report_time
    }

    /// `_init_htu21d()` (lines 124-157).
    /// Pauses the reactor.
    pub fn init_htu21d(&self) -> KResult<()> {
        // Device Soft Reset (line 126).
        self.i2c.i2c_write(&[HTU21D_COMMANDS_RESET as u8], QuerySendOpts::default())?;
        // Wait 15ms after reset (line 128).
        self.reactor.pause(self.reactor.monotonic() + 0.15);
        // Read ChipId (lines 131-132).
        let params = self.i2c.i2c_read(
            &[HTU21D_COMMANDS_SERIAL[2] as u8, HTU21D_COMMANDS_SERIAL[3] as u8],
            3,
            QuerySendOpts::default(),
        )?;
        let response = response_bytes(&params)?;
        let mut rdev_id = (byte_at(&response, 0)? << 8) | byte_at(&response, 1)?;
        let checksum = byte_at(&response, 2)?;
        if self.chek_crc8(rdev_id) != checksum {
            log::warning("htu21d: Reading deviceId !Checksum error!");
        }
        rdev_id >>= 8;
        let guess_dev =
            id_map().get(&rdev_id).copied().unwrap_or("Unknown").to_string();
        if guess_dev == self.device_id {
            log::info(&pct(
                "htu21d: Found Device Type %s",
                &[PctValue::Str(guess_dev.clone())],
            ));
        } else {
            log::warning(&pct_int("htu21d: Unknown Device ID %#x ", rdev_id));
        }
        if self.device_id != guess_dev {
            log::warning(&pct(
                "htu21d: Found device %s. Forcing to type %s as config.",
                &[PctValue::Str(guess_dev), PctValue::Str(self.device_id.clone())],
            ));
        }
        // Set Resolution (lines 152-157). Note the Python bug on line 156:
        // `registerData` is passed as `minclock`, not as bus data.
        let params = self.i2c.i2c_read(&[HTU21D_COMMANDS_READ as u8], 1, QuerySendOpts::default())?;
        let response = response_bytes(&params)?;
        let mut register_data = byte_at(&response, 0)? & HTU21D_RESOLUTION_MASK;
        register_data |= resolution_value(&self.resolution)
            .expect("resolution validated in new");
        self.i2c.i2c_write(
            &[HTU21D_COMMANDS_WRITE as u8],
            QuerySendOpts { minclock: register_data, reqclock: 0, retry: true },
        )?;
        log::info(&pct(
            "htu21d: Setting resolution to %s ",
            &[PctValue::Str(self.resolution.clone())],
        ));
        Ok(())
    }

    /// `_sample_htu21d(eventtime)` (lines 159-234).
    /// Pauses the reactor.
    pub fn sample(&self, _eventtime: f64) -> f64 {
        match self.sample_inner() {
            Ok(t) => t,
            Err(e) => {
                // except Exception: (line 221-224).
                log::exception(
                    "htu21d: Error reading data",
                    &format!("{}: {}", e.class_name(), e.message()),
                );
                let mut s = self.state.borrow_mut();
                s.temp = 0.0;
                s.humidity = Humidity::Float(0.0);
                NEVER
            }
        }
    }

    fn sample_inner(&self) -> KResult<f64> {
        // Read Temperature (lines 162-183).
        if self.hold_master_mode {
            self.i2c.i2c_write(&[HTU21D_COMMANDS_TEMP as u8], QuerySendOpts::default())?;
        } else {
            self.i2c.i2c_write(&[HTU21D_COMMANDS_TEMP_NH as u8], QuerySendOpts::default())?;
        }
        // Wait (lines 168-169).
        let times = conversion_times(&self.device_id, &self.resolution)?;
        self.reactor.pause(self.reactor.monotonic() + times[0]);
        let params = self.i2c.i2c_read(&[], 3, QuerySendOpts::default())?;
        let response = response_bytes(&params)?;
        let rtemp = (byte_at(&response, 0)? << 8) | byte_at(&response, 1)?;
        if self.chek_crc8(rtemp) != byte_at(&response, 2)? {
            log::warning("htu21d: Checksum error on Temperature reading!");
        } else {
            let temp = 0.002681 * (rtemp as f64) - 46.85;
            self.state.borrow_mut().temp = temp;
            log::debug(&pct_float("htu21d: Temperature %.2f ", temp));
        }
        // Read Humidity (lines 186-220).
        if self.hold_master_mode {
            self.i2c.i2c_write(&[HTU21D_COMMANDS_HUMID as u8], QuerySendOpts::default())?;
        } else {
            self.i2c.i2c_write(&[HTU21D_COMMANDS_HUMID_NH as u8], QuerySendOpts::default())?;
        }
        // Wait (lines 192-193).
        let times = conversion_times(&self.device_id, &self.resolution)?;
        self.reactor.pause(self.reactor.monotonic() + times[1]);
        let params = self.i2c.i2c_read(&[], 3, QuerySendOpts::default())?;
        let response = response_bytes(&params)?;
        let mut rhumid = (byte_at(&response, 0)? << 8) | byte_at(&response, 1)?;
        if self.chek_crc8(rhumid) != byte_at(&response, 2)? {
            log::warning("htu21d: Checksum error on Humidity reading!");
        } else {
            // clear status bits (lines 205-206).
            rhumid ^= 0x02;
            let mut humidity = Humidity::Float(0.001907 * (rhumid as f64) - 6.0);
            // Lines 209-214: the clamp assigns the int literals.
            if humidity.as_f64() < 0.0 {
                humidity = Humidity::Int(0);
            } else if humidity.as_f64() > 100.0 {
                humidity = Humidity::Int(100);
            }
            // Only for HTU21D & SHT21 (lines 215-219). The Python
            // assignment is split across lines 218-219, so the compensation
            // expression is evaluated and discarded; humidity is unchanged.
            let temp = self.state.borrow().temp;
            if (self.device_id == "SHT21" || self.device_id == "HTU21D")
                && temp > 0.0
                && temp < 80.0
            {
                log::debug("htu21d: Do temp compensation..");
                let _discarded = (25.0 - temp) * HTU21D_TEMP_COEFFICIENT;
            }
            self.state.borrow_mut().humidity = humidity;
            log::debug(&pct_float("htu21d: Humidity %.2f ", humidity.as_f64()));
        }
        // Lines 226-229.
        let (temp, min_temp, max_temp) = {
            let s = self.state.borrow();
            (s.temp, s.min_temp, s.max_temp)
        };
        if temp < min_temp || temp > max_temp {
            self.printer.invoke_shutdown(
                &pct(
                    "HTU21D temperature %0.1f outside range of %0.1f:%.01f",
                    &[
                        PctValue::Float(temp),
                        PctValue::Float(min_temp),
                        PctValue::Float(max_temp),
                    ],
                ),
                PyValue::Dict(IndexMap::new()),
            );
        }
        // Lines 231-234.
        let measured_time = self.reactor.monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        let cb_taken = self.state.borrow_mut().callback.take();
        if let Some(mut cb) = cb_taken {
            cb(print_time, temp);
            self.state.borrow_mut().callback = Some(cb);
        }
        Ok(measured_time + self.report_time as f64)
    }

    /// `_chekCRC8(data)` (lines 236-243).
    pub fn chek_crc8(&self, mut data: i64) -> i64 {
        for _bit in 0..16 {
            if (data & 0x8000) != 0 {
                data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
            } else {
                data <<= 1;
            }
        }
        data >>= 8;
        data
    }

    /// `get_status(eventtime)` (lines 245-249).
    pub fn status_dict(&self) -> PyValue {
        let (temp, humidity) = {
            let s = self.state.borrow();
            (s.temp, s.humidity)
        };
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(temp, 2)),
        );
        d.insert(PyValue::Str("humidity".to_string()), humidity.to_py());
        PyValue::Dict(d)
    }
}

impl TemperatureSensorApi for HTU21D {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        HTU21D::setup_minmax(self, min_temp, max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TemperatureCallback) {
        HTU21D::setup_callback(self, cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_time as f64
    }
}

impl PrinterObject for HTU21D {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status_dict())
    }
}

// ---------------------------------------------------------------------------
// load_config.
// ---------------------------------------------------------------------------

/// Register all `HTU21D_DEVICES` sensor types with a heater registry
/// (`load_config` body, lines 252-256).
pub fn register_sensor_factories(registry: &dyn PrinterHeatersApi) {
    for stype in htu21d_devices().keys() {
        let factory: SensorFactory = Arc::new(|config: &Config| -> KResult<Arc<dyn TemperatureSensorApi>> {
            let obj = HTU21D::new(config)?;
            Ok(obj as Arc<dyn TemperatureSensorApi>)
        });
        registry.add_sensor_factory(stype, factory);
    }
}

/// `load_config(config)` (lines 252-256).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let heaters = lookup_interface::<dyn PrinterHeatersApi>(&*printer, "heaters", LookupDefault::Raise)?
        .expect("lookup_interface with Raise returns Some or raises");
    register_sensor_factories(&*heaters);
    Ok(None)
}

/// Module factory (`load_config` entry point).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
