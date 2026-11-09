// Port of `klippy/extras/aht10.py` (AHTxx family I2C temperature and
// humidity sensors).
//
// The module owns the sensor state machine: init with soft reset when the
// device stays busy, measurement polling with a bounded retry loop, and
// periodic sampling through the reactor timer with out-of-range shutdown.
//
// The I2C transport is the `bus` module's [`McuI2cApi`]
// (`klippy/extras/bus.py:161` `MCU_I2C`); the bus handle is built with
// `bus::mcu_i2c_from_config(config, Some(0x38), 100000, false)` (the Python
// `MCU_I2C_from_config(config, default_addr=0x38, default_speed=100000)`
// defaults, `aht10.py:38-39`) before the injected-handle constructor runs.
// Sensor factories register with the `heaters` object's
// [`PrinterHeatersApi`] (`aht10.py:194-203`); the object itself implements
// [`TemperatureSensorApi`], the duck type `heaters` calls on sensors.
//
// The Python subclasses `AHT1x`/`AHT2x`/`AHT3x` become the [`AhtModel`]
// carried by [`AHTBase`]; the constructors [`aht1x`], [`aht2x`] and
// [`aht3x`] mirror the Python classes by name.

use crate::bus::{mcu_i2c_from_config, McuI2cApi};
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use klippy_api::config::{Config, GetIntOpts};
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::mcu::QuerySendOpts;
use klippy_api::object::{lookup_interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{printer_interfaces, printer_object_base};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Timer, NEVER, NOW};
use std::sync::Arc;

pub const I2C_ADDR: i64 = 0x38;
pub const CMD_MEASURE: [u8; 3] = [0xAC, 0x33, 0x00];
pub const CMD_RESET: [u8; 1] = [0xBA];
pub const CMD_INIT_AHT1X: [u8; 3] = [0xE1, 0x08, 0x00];
pub const CMD_INIT_AHT2X: [u8; 3] = [0xBE, 0x08, 0x00];
pub const STATUS_BUSY: u8 = 0x80;
pub const STATUS_CALIBRATED: u8 = 0x08;
pub const MAX_BUSY_CYCLES: i64 = 5;


/// Which sensor variant this instance models (`AHT1x`, `AHT2x`, `AHT3x`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AhtModel {
    Aht1x,
    Aht2x,
    Aht3x,
}

impl AhtModel {
    /// Python `model` class attribute.
    pub fn name(self) -> &'static str {
        match self {
            AhtModel::Aht1x => "aht1x",
            AhtModel::Aht2x => "aht2x",
            AhtModel::Aht3x => "aht3x",
        }
    }

    fn upper_name(self) -> &'static str {
        match self {
            AhtModel::Aht1x => "AHT1X",
            AhtModel::Aht2x => "AHT2X",
            AhtModel::Aht3x => "AHT3X",
        }
    }
}

fn pct(format: &str, args: &[PctValue]) -> String {
    percent_format(format, args).unwrap_or_else(|e| e.display())
}

fn s(value: &str) -> PctValue {
    PctValue::Str(value.to_string())
}

fn f(value: f64) -> PctValue {
    PctValue::Float(value)
}

fn n(value: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(value))
}
/// Parse an `i2c_read` result (`bus.py` `{'response': bytes}` or `None`).
/// A `None` payload (`PyValue::None`) maps to `None`; a malformed payload
/// raises the `TypeError`/`KeyError` Python would raise, so the caller's
/// `except Exception` path logs it exactly as Python does.
fn response_bytes(params: &PyValue) -> KResult<Option<Vec<u8>>> {
    let PyValue::Dict(d) = params else {
        return if matches!(params, PyValue::None) {
            Ok(None)
        } else {
            Err(klippy_api::error::KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "i2c_read params must be a dict".to_string(),
            })
        };
    };
    let response = d
        .get(&PyValue::Str("response".to_string()))
        .ok_or_else(|| klippy_api::error::KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'response'".to_string(),
        })?;
    match response {
        PyValue::Bytes(b) => Ok(Some(b.clone())),
        PyValue::List(items) => {
            let mut out = Vec::with_capacity(items.len());
            for v in items {
                let PyValue::Int(n) = v else {
                    return Err(klippy_api::error::KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: "i2c response must be ints".to_string(),
                    });
                };
                let n: i64 = n
                    .inner()
                    .to_string()
                    .parse()
                    .map_err(|_| klippy_api::error::KlippyError::Internal {
                        class_name: "ValueError".to_string(),
                        message: "int value out of range".to_string(),
                    })?;
                out.push(u8::try_from(n).map_err(|_| {
                    klippy_api::error::KlippyError::Internal {
                        class_name: "ValueError".to_string(),
                        message: "byte must be in range(0, 256)".to_string(),
                    }
                })?);
            }
            Ok(Some(out))
        }
        _ => Err(klippy_api::error::KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "i2c response must be bytes".to_string(),
        }),
    }
}

struct State {
    temp: f64,
    min_temp: f64,
    max_temp: f64,
    humidity: i64,
    /// Python keeps `humidity` a float (`0.`) until the first successful
    /// measurement assigns `int(...)`; the status dict must reflect that.
    humidity_is_float: bool,
    is_calibrated: bool,
    init_sent: bool,
    callback: Option<TemperatureCallback>,
}

/// Port of the Python `AHTBase` class (`klippy/extras/aht10.py:31`).
///
/// The Python subclasses `AHT1x`/`AHT2x`/`AHT3x` become the [`AhtModel`]
/// carried by this one type; the constructors [`aht1x`], [`aht2x`] and
/// [`aht3x`] mirror the Python classes by name.
pub struct AHTBase {
    printer: Printer,
    reactor: reactor::Reactor,
    i2c: Arc<dyn McuI2cApi>,
    name: String,
    model: AhtModel,
    report_time: i64,
    state: GCell<State>,
    sample_timer: GCell<Option<Timer>>,
}

impl AHTBase {
    /// Port of `AHTBase.__init__` (`aht10.py:34`).
    ///
    /// Python resolves the I2C bus first through
    /// `bus.MCU_I2C_from_config(config, default_addr=0x38,
    /// default_speed=100000)`; production callers (the sensor factories
    /// in [`load_config`]) build the bus with
    /// [`mcu_i2c_from_config`] and inject it here, which also gives tests
    /// the fake-bus seam. The `aht10_report_time` option is read with the
    /// same default (30) and minimum (5).
    pub fn new(config: &Config, i2c: Arc<dyn McuI2cApi>, model: AhtModel) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name().split_whitespace().last().unwrap_or("").to_string();
        let reactor = printer.get_reactor();
        let report_time = config
            .getint(
                "aht10_report_time",
                GetIntOpts {
                    default: Some(Some(30)),
                    minval: Some(5),
                    maxval: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(30);
        let obj = Arc::new(AHTBase {
            printer: printer.clone(),
            reactor: reactor.clone(),
            i2c,
            name,
            model,
            report_time,
            state: GCell::new(State {
                temp: 0.,
                min_temp: 0.,
                max_temp: 0.,
                humidity: 0,
                humidity_is_float: true,
                is_calibrated: false,
                init_sent: false,
                callback: None,
            }),
            sample_timer: GCell::new(None),
        });
        // `self.reactor.register_timer(self._sample_aht)`: the bound method
        // keeps the Python instance alive; a weak reference mirrors that
        // lifetime (the printer registry holds the strong reference).
        let weak = Arc::downgrade(&obj);
        let timer = reactor.register_timer(
            move |eventtime| weak.upgrade().map(|o| o.sample_aht(eventtime)).unwrap_or(NEVER),
            NEVER,
        );
        *obj.sample_timer.borrow_mut() = Some(timer);
        let obj_ref: ObjectRef = obj.clone();
        printer.add_object(&format!("aht10 {}", obj.name), obj_ref)?;
        let weak = Arc::downgrade(&obj);
        printer.register_event_handler(
            klippy_api::printer::EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                if let Some(o) = weak.upgrade() {
                    o.handle_connect()?;
                }
                Ok(PyValue::None)
            }),
        );
        Ok(obj)
    }

    /// Port of `handle_connect` (`aht10.py:51`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.init_sensor()?;
        let timer = self.sample_timer.borrow();
        if let Some(timer) = timer.as_ref() {
            self.reactor.update_timer(timer, NOW);
        }
        Ok(())
    }

    /// Port of `_send_init` (`aht10.py:55`), specialised per subclass.
    fn send_init(&self) -> KResult<()> {
        match self.model {
            AhtModel::Aht1x => {
                self.i2c.i2c_write(&CMD_INIT_AHT1X, QuerySendOpts::default())?;
                self.reactor.pause(self.reactor.monotonic() + 0.040);
            }
            AhtModel::Aht2x => {
                self.i2c.i2c_write(&CMD_INIT_AHT2X, QuerySendOpts::default())?;
                self.reactor.pause(self.reactor.monotonic() + 0.100);
            }
            AhtModel::Aht3x => {
                // Wait for auto-calibration at power-on.
                self.reactor.pause(self.reactor.monotonic() + 0.100);
            }
        }
        Ok(())
    }

    /// Port of `_init_sensor` (`aht10.py:58`).
    pub fn init_sensor(&self) -> KResult<()> {
        self.send_init()?;
        self.state.with_mut(|state| state.init_sent = true);
        if self.make_measurement() {
            let state = self.state.borrow();
            if !state.is_calibrated {
                log::warning(&pct(
                    "%s %s: not calibrated, possible OTP fault",
                    &[s(self.model.name()), s(&self.name)],
                ));
            }
            log::info(&pct(
                "%s %s: successfully initialized, initial temp: %.3f, humidity: %.3f",
                &[
                    s(self.model.name()),
                    s(&self.name),
                    f(state.temp),
                    if state.humidity_is_float {
                        f(state.humidity as f64)
                    } else {
                        n(state.humidity)
                    },
                ],
            ));
        }
        Ok(())
    }

    /// Port of `_soft_reset` (`aht10.py:70`).
    fn soft_reset(&self) -> KResult<()> {
        log::info(&pct(
            "%s %s: performing soft reset",
            &[s(self.model.name()), s(&self.name)],
        ));
        self.i2c.i2c_write(&CMD_RESET, QuerySendOpts::default())?;
        self.reactor.pause(self.reactor.monotonic() + 0.020);
        Ok(())
    }

    /// Port of `_make_measurement` (`aht10.py:75`).
    pub fn make_measurement(&self) -> bool {
        if !self.state.borrow().init_sent {
            return false;
        }
        let result = (|| -> KResult<(Option<Vec<u8>>, bool)> {
            let mut data: Option<Vec<u8>> = None;
            let mut is_busy = true;
            let mut cycles: i64 = 0;
            while is_busy {
                // Check if we're constantly busy. If so, send soft-reset
                // and issue warning.
                if is_busy && cycles > MAX_BUSY_CYCLES {
                    log::warning(&pct(
                        "%s %s: device reported busy after %d cycles, resetting device",
                        &[s(self.model.name()), s(&self.name), n(MAX_BUSY_CYCLES)],
                    ));
                    self.soft_reset()?;
                    data = None;
                    break;
                }
                cycles += 1;
                // Write command for updating temperature+status bit.
                self.i2c.i2c_write(&CMD_MEASURE, QuerySendOpts::default())?;
                // Wait 110ms after first read, 75ms minimum.
                self.reactor.pause(self.reactor.monotonic() + 0.110);
                // Read 6 bytes of data.
                match response_bytes(&self.i2c.i2c_read(&[], 6, QuerySendOpts::default())?)? {
                    None => {
                        log::warning(&pct(
                            "%s %s: received data from i2c_read is None",
                            &[s(self.model.name()), s(&self.name)],
                        ));
                        continue;
                    }
                    Some(bytes) => {
                        if bytes.len() < 6 {
                            log::warning(&pct(
                                "%s %s: received bytes less than expected: got %d, need 6",
                                &[
                                    s(self.model.name()),
                                    s(&self.name),
                                    n(bytes.len() as i64),
                                ],
                            ));
                            continue;
                        }
                        self.state
                            .with_mut(|state| state.is_calibrated = bytes[0] & STATUS_CALIBRATED != 0);
                        is_busy = bytes[0] & STATUS_BUSY != 0;
                        data = Some(bytes);
                    }
                }
            }
            Ok((data, is_busy))
        })();
        let (data, is_busy) = match result {
            Err(e) => {
                log::exception(
                    &pct(
                        "%s %s: exception encountered reading data: %s",
                        &[
                            s(self.model.name()),
                            s(&self.name),
                            PctValue::Str(e.message().to_string()),
                        ],
                    ),
                    e.message(),
                );
                return false;
            }
            Ok(value) => value,
        };
        if is_busy {
            return false;
        }
        // `data` is always present here in Python: leaving the loop
        // without the busy-reset path means a full 6-byte read was stored.
        let data = match data {
            Some(data) => data,
            None => return false,
        };
        // Parse temperature: 20 bits starting at data[3] (low nibble).
        let temp_raw = ((data[3] & 0x0F) as u32) << 16 | ((data[4] as u32) << 8) | data[5] as u32;
        let temp = ((temp_raw as f64) * 200.0 / 1048576.0) - 50.0;
        // Parse humidity: 20 bits starting at data[1].
        let hum_raw = (((data[1] as u32) << 16) | ((data[2] as u32) << 8) | data[3] as u32) >> 4;
        // `int(hum_raw * 100 / 1048576)`: the product is below 2^27, so the
        // float division by 2^20 is exact and truncation equals integer
        // division (Python `int()` truncates toward zero; the value is
        // non-negative here).
        let humidity = (hum_raw * 100 / 1048576) as i64;
        // Clamp humidity.
        let humidity = humidity.clamp(0, 100);
        self.state.with_mut(|state| {
            state.temp = temp;
            state.humidity = humidity;
            state.humidity_is_float = false;
        });
        true
    }

    /// Port of `_sample_aht` (`aht10.py:142`). Returns the next waketime.
    pub fn sample_aht(&self, _eventtime: f64) -> f64 {
        if !self.make_measurement() {
            self.state.with_mut(|state| {
                state.temp = 0.;
                state.humidity = 0;
                state.humidity_is_float = true;
            });
            return NEVER;
        }
        let (temp, min_temp, max_temp) = {
            let state = self.state.borrow();
            (state.temp, state.min_temp, state.max_temp)
        };
        if temp < min_temp || temp > max_temp {
            self.printer.invoke_shutdown(
                &pct(
                    "%s temperature %.1f outside range of %.1f:%.1f",
                    &[
                        s(self.model.upper_name()),
                        f(temp),
                        f(min_temp),
                        f(max_temp),
                    ],
                ),
                PyValue::Dict(indexmap::IndexMap::new()),
            );
        }
        let measured_time = self.reactor.monotonic();
        let print_time = self.i2c.get_mcu().estimated_print_time(measured_time);
        // Take the callback out so it is not invoked while borrowed.
        let mut callback = self.state.with_mut(|state| state.callback.take());
        if let Some(callback) = callback.as_mut() {
            callback(print_time, temp);
        }
        self.state.with_mut(|state| state.callback = callback);
        measured_time + self.report_time as f64
    }

    /// Port of `setup_minmax` (`aht10.py:157`).
    pub fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        self.state.with_mut(|state| {
            state.min_temp = min_temp;
            state.max_temp = max_temp;
        });
    }

    /// Port of `setup_callback` (`aht10.py:161`): `cb(print_time, temp)`.
    pub fn setup_callback(&self, cb: TemperatureCallback) {
        self.state.with_mut(|state| state.callback = Some(cb));
    }

    /// Port of `get_report_time_delta` (`aht10.py:164`).
    pub fn get_report_time_delta(&self) -> i64 {
        self.report_time
    }

    /// Port of `get_status` (`aht10.py:167`). `eventtime` is unused, as in
    /// Python (the argument is accepted but never read).
    pub fn status(&self) -> PyValue {
        let state = self.state.borrow();
        let mut map = indexmap::IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(state.temp, 2)),
        );
        map.insert(
            PyValue::Str("humidity".to_string()),
            if state.humidity_is_float {
                PyValue::Float(state.humidity as f64)
            } else {
                PyValue::Int(PyInt::from_i64(state.humidity))
            },
        );
        PyValue::Dict(map)
    }
}

impl TemperatureSensorApi for AHTBase {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        AHTBase::setup_minmax(self, min_temp, max_temp);
        Ok(())
    }
    fn setup_callback(&self, cb: TemperatureCallback) {
        AHTBase::setup_callback(self, cb);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_time as f64
    }
}

impl PrinterObject for AHTBase {
    printer_object_base!();
    printer_interfaces!(dyn TemperatureSensorApi);

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status()))
    }
}

/// Port of the `AHT1x` class (`aht10.py:173`).
pub fn aht1x(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<AHTBase>> {
    AHTBase::new(config, i2c, AhtModel::Aht1x)
}

/// Port of the `AHT2x` class (`aht10.py:180`).
pub fn aht2x(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<AHTBase>> {
    AHTBase::new(config, i2c, AhtModel::Aht2x)
}

/// Port of the `AHT3x` class (`aht10.py:187`).
pub fn aht3x(config: &Config, i2c: Arc<dyn McuI2cApi>) -> KResult<Arc<AHTBase>> {
    AHTBase::new(config, i2c, AhtModel::Aht3x)
}

/// Sensor factory for one model (`heaters.py:273` `SensorFactory`): the
/// Python class constructor `AHT1x(config)` etc. Like `AHTBase.__init__`
/// (`aht10.py:38-39`), the bus handle is built from the config before the
/// sensor is constructed.
fn sensor_factory(model: AhtModel) -> SensorFactory {
    Arc::new(move |config: &Config| -> KResult<Arc<dyn TemperatureSensorApi>> {
        let i2c: Arc<dyn McuI2cApi> =
            mcu_i2c_from_config(config, Some(I2C_ADDR), 100000, false)?;
        Ok(AHTBase::new(config, i2c, model)?)
    })
}

/// Port of `load_config` (`aht10.py:194`): register the sensor factories
/// with the `heaters` object (raising `config_error` when it is missing,
/// exactly like the Python `lookup_object("heaters")`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheater =
        lookup_interface::<dyn PrinterHeatersApi>(&*printer, "heaters", LookupDefault::Raise)?
            .expect("lookup_interface with Raise returns Some or raises");
    // Backwards compatibility alias.
    pheater.add_sensor_factory("AHT10", sensor_factory(AhtModel::Aht1x));
    pheater.add_sensor_factory("AHT1X", sensor_factory(AhtModel::Aht1x));
    pheater.add_sensor_factory("AHT2X", sensor_factory(AhtModel::Aht2x));
    pheater.add_sensor_factory("AHT3X", sensor_factory(AhtModel::Aht3x));
    Ok(None)
}

/// Module entry points matching the Python module (`aht10.py:194`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
