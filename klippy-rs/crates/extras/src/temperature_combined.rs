//! Port of `klippy/extras/temperature_combined.py`.
//!
//! Combines several temperature sensors into one virtual sensor used by a
//! heater. The Python class is `PrinterSensorCombined`; in Rust it is
//! [`CombinedSensor`]. The module registers itself with the heaters object
//! through `load_config` and is otherwise constructed by the heaters
//! `sensor_factory` mechanism.
//!
//! Every branch, message text, float operation order and rounding follows
//! `temperature_combined.py` line by line. Python bugs are reproduced:
//! `if temp:` is a truthiness check (0.0 leaves `last_temp` unchanged), and
//! `get_temp` always returns `(last_temp, 0.)` regardless of `eventtime`.

use indexmap::IndexMap;
use klippy_api::config::{GetChoiceOpts, GetFloatOpts, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::McuApi;
use klippy_api::object::{load_interface, lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT, EV_KLIPPY_READY};
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use reactor::Reactor;
use reactor::Timer;
use std::sync::Arc;

use crate::heaters::{PrinterHeatersApi, SensorApi, TemperatureCallback, TemperatureSensorApi};

/// `REPORT_TIME = 0.300` (`temperature_combined.py:8`).
pub const REPORT_TIME: f64 = 0.300;

/// `mean(values)` (`temperature_combined.py:174`).
///
/// ```text
/// if not values: return
/// return sum(values) / len(values)
/// ```
///
/// Empty input returns `None`, matching Python's bare `return`.
pub fn mean(values: &[f64]) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    let mut acc = 0.0f64;
    for &x in values {
        acc += x;
    }
    Some(acc / values.len() as f64)
}

/// Combination algorithm selected by `combination_method` in config.
/// Mirrors the Python dict `{'min': min, 'max': max, 'mean': mean}`; the
/// caller-visible names are `min`, `max` and `mean`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CombinationMethod {
    Min,
    Max,
    Mean,
}

impl CombinationMethod {
    fn apply(&self, values: &[f64]) -> Option<f64> {
        match self {
            // Python `min` / `max` on an empty sequence raise ValueError;
            // callers only pass non-empty lists, and `mean` returns None
            // for empty. A defensive None here keeps the Rust side
            // unreachable-on-path rather than panicking.
            CombinationMethod::Min => values.iter().cloned().fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.min(b)))),
            CombinationMethod::Max => values.iter().cloned().fold(None, |a: Option<f64>, b| Some(a.map_or(b, |a| a.max(b)))),
            CombinationMethod::Mean => mean(values),
        }
    }
}

/// Mutable state mirroring the Python instance dict. All fields the Python
/// code sets in `__init__` plus those mutated later live here so a single
/// `GCell` protects the whole object (Python has one dict).
struct State {
    /// `self.sensor` (the combined sensor passes itself as the `sensor`
    /// for heaters; `self.sensor = self`). Modelled as a flag that the
    /// sensor is itself.
    sensor_is_self: bool,
    last_temp: f64,
    min_temp: f64,
    max_temp: f64,
    humidity: Option<f64>,
    pressure: Option<f64>,
    gas: Option<f64>,
    sensors: Vec<ObjectRef>,
    /// `self.temperature_callback`, set by `setup_callback`. The heaters
    /// duck type passes `Box<dyn FnMut>`; Python calls it as
    /// `callback(print_time, temp)`.
    temperature_callback: Option<TemperatureCallback>,
    timer: Option<Timer>,
}

impl Default for State {
    fn default() -> Self {
        State {
            sensor_is_self: true,
            last_temp: 0.0,
            min_temp: 0.0,
            max_temp: 0.0,
            humidity: None,
            pressure: None,
            gas: None,
            sensors: Vec::new(),
            temperature_callback: None,
            timer: None,
        }
    }
}

/// The combined temperature sensor (`PrinterSensorCombined`,
/// `temperature_combined.py:11`).
pub struct CombinedSensor {
    printer: Printer,
    reactor: Reactor,
    /// `self.name`, the last token of the section name.
    name: String,
    sensor_names: Vec<String>,
    max_deviation: f64,
    apply_mode: CombinationMethod,
    state: GCell<State>,
}

impl CombinedSensor {
    /// `PrinterSensorCombined.__init__(config)` (`temperature_combined.py:12`).
    pub fn new(config: &klippy_api::config::Config) -> KResult<CombinedSensor> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let name = config
            .get_name()
            .split_whitespace()
            .next_back()
            .unwrap_or("")
            .to_string();
        let sensor_names = config.getlist("sensor_list", GetListOpts::default())?.ok_or_else(|| {
            KlippyError::config(format!("Option 'sensor_list' in section '{}' must be specified", config.get_name()))
        })?;
        let max_deviation = config
            .getfloat(
                "maximum_deviation",
                GetFloatOpts { above: Some(0.0), ..GetFloatOpts::default() },
            )?
            .ok_or_else(|| {
                KlippyError::config(format!("Option 'maximum_deviation' in section '{}' must be specified", config.get_name()))
            })?;
        let apply_mode = {
            let choices = PyValue::Dict(
                [
                    (PyValue::Str("min".to_string()), PyValue::Str("min".to_string())),
                    (PyValue::Str("max".to_string()), PyValue::Str("max".to_string())),
                    (PyValue::Str("mean".to_string()), PyValue::Str("mean".to_string())),
                ]
                .into_iter()
                .collect::<IndexMap<_, _>>(),
            );
            let picked = config.getchoice("combination_method", &choices, GetChoiceOpts::default());
            match picked? {
                PyValue::Str(s) if s == "min" => CombinationMethod::Min,
                PyValue::Str(s) if s == "max" => CombinationMethod::Max,
                PyValue::Str(s) if s == "mean" => CombinationMethod::Mean,
                _ => CombinationMethod::Mean,
            }
        };

        Ok(CombinedSensor {
            printer,
            reactor,
            name,
            sensor_names,
            max_deviation,
            apply_mode,
            state: GCell::new(State::default()),
        })
    }

    /// Registers the timer and event handlers, as the Python `__init__`
    /// does, once `self` is inside an `Arc`.
    ///
    /// Python registers `register_timer(self._temperature_update_event)`
    /// (with the default `waketime=NEVER`, never fires until `_handle_ready`
    /// arms it) and the `klippy:connect` / `klippy:ready` handlers.
    pub fn init(self: &Arc<Self>) -> KResult<()> {
        let timer_self = Arc::clone(self);
        let reactor = self.reactor.clone();
        let timer = reactor.register_timer(
            move |eventtime| timer_self.temperature_update_event(eventtime),
            reactor::NEVER,
        );
        self.state.with_mut(|s| s.timer = Some(timer));
        let connect_self = Arc::clone(self);
        self.printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_| {
                connect_self.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        let ready_self = Arc::clone(self);
        self.printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_| {
                ready_self.handle_ready();
                Ok(PyValue::None)
            }),
        );
        Ok(())
    }

    /// `printer.add_object("temperature_combined " + self.name, self)` for an
    /// already-`Arc`-wrapped sensor.
    pub fn add_object(self: &Arc<Self>) -> KResult<()> {
        let key = format!("temperature_combined {}", self.name);
        self.printer.add_object(&key, Arc::clone(self) as ObjectRef)
    }

    /// `_handle_connect` (`temperature_combined.py:40`).
    pub fn handle_connect(&self) -> KResult<()> {
        for sensor_name in &self.sensor_names {
            let sensor = self
                .printer
                .lookup_object(sensor_name, LookupDefault::Raise)?
                .expect("lookup_object with Raise never returns None");
            let status = sensor.get_status(self.reactor.monotonic());
            let Some(status) = status else {
                return Err(KlippyError::config(format!("'{sensor_name}' does not have a status.")));
            };
            let Some(temp) = dict_get_str(&status, "temperature") else {
                return Err(KlippyError::config(format!("'{sensor_name}' does not report a temperature.")));
            };
            if matches!(temp, PyValue::None) {
                return Err(KlippyError::config(format!(
                    "Temperature monitor '{sensor_name}' is not supported"
                )));
            }
            self.state.with_mut(|s| s.sensors.push(sensor));
        }
        Ok(())
    }

    /// `_handle_ready` (`temperature_combined.py:62`).
    pub fn handle_ready(&self) {
        let wake = self.reactor.monotonic() + 1.0;
        self.state.with_mut(|s| {
            if let Some(timer) = &s.timer {
                self.reactor.update_timer(timer, wake);
            }
        });
    }

    /// `setup_minmax(min_temp, max_temp)` (`temperature_combined.py:69`).
    pub fn setup_minmax(&self, min_temp: f64, max_temp: f64) {
        self.state.with_mut(|s| {
            s.min_temp = min_temp;
            s.max_temp = max_temp;
        });
    }

    /// `setup_callback(temperature_callback)` (`temperature_combined.py:73`).
    pub fn setup_callback(&self, temperature_callback: TemperatureCallback) {
        self.state.with_mut(|s| s.temperature_callback = Some(temperature_callback));
    }

    /// `get_report_time_delta()` (`temperature_combined.py:76`).
    pub fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }

    /// `get_temp(eventtime)` (`temperature_combined.py:97`): always
    /// `(self.last_temp, 0.)`, `eventtime` unused.
    pub fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (self.state.borrow().last_temp, 0.0)
    }

    /// `update_temp(eventtime)` (`temperature_combined.py:79`).
    pub fn update_temp(&self, eventtime: f64) {
        let sensors = self.state.borrow().sensors.clone();
        let mut values: Vec<f64> = Vec::with_capacity(sensors.len());
        for sensor in &sensors {
            let status = sensor.get_status(eventtime);
            let Some(status) = status else {
                // `sensor_status['temperature']` on a non-dict raises
                // TypeError in Python; the reference sensors always return
                // a dict with the key, so this is unreachable in practice.
                continue;
            };
            if let Some(PyValue::Float(t)) = dict_get_str(&status, "temperature") {
                values.push(t);
            } else if let Some(v) = dict_get_str(&status, "temperature") {
                if let Some(fv) = v.to_f64() {
                    values.push(fv);
                }
            }
        }
        if values.is_empty() {
            return;
        }
        let mut max_v = values[0];
        let mut min_v = values[0];
        for &v in &values[1..] {
            if v > max_v {
                max_v = v;
            }
            if v < min_v {
                min_v = v;
            }
        }
        if (max_v - min_v) > self.max_deviation {
            let msg = percent_format(
                "COMBINED SENSOR maximum deviation exceeded limit of %0.1f, max sensor value %0.1f, min sensor value %0.1f.",
                &[
                    PctValue::Float(self.max_deviation),
                    PctValue::Float(max_v),
                    PctValue::Float(min_v),
                ],
            )
            .unwrap_or_default();
            self.printer.invoke_shutdown(&msg, PyValue::None);
        }
        if let Some(temp) = self.apply_mode.apply(&values) {
            // `if temp:` — truthiness: 0.0 (and -0.0) is falsy.
            if temp != 0.0 {
                self.state.with_mut(|s| s.last_temp = temp);
            }
        }
    }

    /// `update_additional(eventtime)` (`temperature_combined.py:100`).
    pub fn update_additional(&self, eventtime: f64) {
        let sensors = self.state.borrow().sensors.clone();
        let mut values_humidity: Vec<f64> = Vec::new();
        let mut values_pressure: Vec<f64> = Vec::new();
        let mut values_gas: Vec<f64> = Vec::new();
        for sensor in &sensors {
            let Some(status) = sensor.get_status(eventtime) else { continue };
            if let Some(v) = dict_get_str(&status, "humidity") {
                if let Some(f) = v.to_f64() {
                    values_humidity.push(f);
                }
            }
            if let Some(v) = dict_get_str(&status, "pressure") {
                if let Some(f) = v.to_f64() {
                    values_pressure.push(f);
                }
            }
            if let Some(v) = dict_get_str(&status, "gas") {
                if let Some(f) = v.to_f64() {
                    values_gas.push(f);
                }
            }
        }
        if !values_humidity.is_empty() {
            if let Some(humidity) = self.apply_mode.apply(&values_humidity) {
                if humidity != 0.0 {
                    self.state.with_mut(|s| s.humidity = Some(humidity));
                }
            }
        }
        if !values_pressure.is_empty() {
            if let Some(pressure) = self.apply_mode.apply(&values_pressure) {
                if pressure != 0.0 {
                    self.state.with_mut(|s| s.pressure = Some(pressure));
                }
            }
        }
        if !values_gas.is_empty() {
            if let Some(gas) = self.apply_mode.apply(&values_gas) {
                if gas != 0.0 {
                    self.state.with_mut(|s| s.gas = Some(gas));
                }
            }
        }
    }

    /// `get_status(eventtime)` (`temperature_combined.py:134`).
    pub fn status_dict(&self) -> PyValue {
        let (last_temp, humidity, pressure, gas) = self.state.with_mut(|s| {
            (s.last_temp, s.humidity, s.pressure, s.gas)
        });
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(last_temp, 2)),
        );
        if let Some(h) = humidity {
            map.insert(PyValue::Str("humidity".to_string()), PyValue::Float(h));
        }
        if let Some(p) = pressure {
            map.insert(PyValue::Str("pressure".to_string()), PyValue::Float(p));
        }
        if let Some(g) = gas {
            map.insert(PyValue::Str("gas".to_string()), PyValue::Float(g));
        }
        PyValue::Dict(map)
    }

    /// `_temperature_update_event(eventtime)` (`temperature_combined.py:146`).
    ///
    /// Returns the next waketime (`measured_time + REPORT_TIME`), matching
    /// the Python `return measured_time + REPORT_TIME`.
    pub fn temperature_update_event(&self, eventtime: f64) -> f64 {
        self.update_temp(eventtime);
        self.update_additional(eventtime);

        let (last_temp, min_temp, max_temp) = self.state.with_mut(|s| {
            (s.last_temp, s.min_temp, s.max_temp)
        });
        if last_temp < min_temp {
            let msg = percent_format(
                "COMBINED SENSOR temperature %0.1f below minimum temperature of %0.1f.",
                &[PctValue::Float(last_temp), PctValue::Float(min_temp)],
            )
            .unwrap_or_default();
            self.printer.invoke_shutdown(&msg, PyValue::None);
        }
        if last_temp > max_temp {
            let msg = percent_format(
                "COMBINED SENSOR temperature %0.1f above maximum temperature of %0.1f.",
                &[PctValue::Float(last_temp), PctValue::Float(max_temp)],
            )
            .unwrap_or_default();
            self.printer.invoke_shutdown(&msg, PyValue::None);
        }

        // `temperature_combined.py:165-169`: the mcu lookup and the
        // callback are unconditional; with a real printer both are
        // present by construction (the heaters call `setup_callback`
        // before the timer is armed), so a missing one panics with the
        // lookup/callback site — the Python `AttributeError` path.
        let mcu: Arc<dyn McuApi> = match lookup_interface::<dyn McuApi>(
            &*self.printer,
            "mcu",
            LookupDefault::Raise,
        ) {
            Ok(Some(mcu)) => mcu,
            _ => panic!("'mcu' object not found for 'temperature_combined {}'", self.name),
        };
        let measured_time = self.reactor.monotonic();
        let print_time = mcu.estimated_print_time(measured_time);
        // Take the callback out so it runs without holding the state
        // borrow (GCell rule 2); put it back afterwards.
        let mut cb = self.state.with_mut(|s| s.temperature_callback.take());
        let cb_fn = cb
            .as_mut()
            .unwrap_or_else(|| panic!("'temperature_callback' not set for 'temperature_combined {}'", self.name));
        cb_fn(print_time, last_temp);
        self.state.with_mut(|s| s.temperature_callback = cb);
        measured_time + REPORT_TIME
    }

    /// `self.sensor` flag (`self.sensor = self` in `__init__`).
    pub fn sensor_is_self(&self) -> bool {
        self.state.borrow().sensor_is_self
    }

    /// `self.name`, the last token of the section name.
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// The heaters sensor-chip view (`heaters.py:30-32`): only the three
/// methods `Heater` calls on the object a sensor factory returns.
impl TemperatureSensorApi for CombinedSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        CombinedSensor::setup_minmax(self, min_temp, max_temp);
        Ok(())
    }
    fn setup_callback(&self, callback: TemperatureCallback) {
        CombinedSensor::setup_callback(self, callback)
    }
    fn get_report_time_delta(&self) -> f64 {
        CombinedSensor::get_report_time_delta(self)
    }
}

/// The `register_sensor` / `temperature_sensor` view (`heaters.py:336,384`):
/// `get_temp(eventtime)` -> `(current, target)`.
impl SensorApi for CombinedSensor {
    fn get_temp(&self, eventtime: f64) -> (f64, f64) {
        CombinedSensor::get_temp(self, eventtime)
    }
}

impl PrinterObject for CombinedSensor {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn TemperatureSensorApi, dyn SensorApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status_dict())
    }
}
/// `PrinterSensorCombined` alias for the Python class name.
pub type PrinterSensorCombined = CombinedSensor;

/// Convenience constructor returning an `Arc` already registered with the
/// printer, timers and event handlers armed: `PrinterSensorCombined(config)`
/// plus `add_object` / `register_timer` / `register_event_handler` exactly as
/// the Python `__init__` does them.
pub fn printer_sensor_combined(config: &klippy_api::config::Config) -> KResult<Arc<CombinedSensor>> {
    let sensor = Arc::new(CombinedSensor::new(config)?);
    sensor.add_object()?;
    sensor.init()?;
    Ok(sensor)
}

fn dict_get_str(v: &PyValue, key: &str) -> Option<PyValue> {
    match v {
        PyValue::Dict(d) => d.get(&PyValue::Str(key.to_string())).cloned(),
        _ => None,
    }
}

/// `load_config(config)` (`temperature_combined.py:180`):
///
/// ```python
/// pheaters = config.get_printer().load_object(config, "heaters")
/// pheaters.add_sensor_factory("temperature_combined", PrinterSensorCombined)
/// ```
pub fn load_config(config: &klippy_api::config::Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    pheaters.add_sensor_factory(
        "temperature_combined",
        Arc::new(|cfg: &klippy_api::config::Config| {
            printer_sensor_combined(cfg).map(|arc| arc as Arc<dyn TemperatureSensorApi>)
        }),
    );
    Ok(None)
}

/// Module factory for the printer loader (this module's `load_config`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: Some(load_config), load_config_prefix: None, add_early_printer_objects: None, add_printer_objects: None }
}
