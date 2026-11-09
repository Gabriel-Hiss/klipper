//! Port of the `temperature_sensor` module (`klippy/extras/temperature_sensor.py`).
//!
//! Adapts a `heaters` sensor chip to the `SensorApi` registered with
//! `heaters` (`PrinterHeatersApi::register_sensor`), tracking the last
//! sample plus the measured min/max.

use crate::heaters::{PrinterHeatersApi, TemperatureSensorApi};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::KResult;
use klippy_api::object::{load_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::gcell::SCell;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// `klippy/extras/temperature_sensor.py:7`.
pub const KELVIN_TO_CELSIUS: f64 = -273.15;

/// `PrinterSensorGeneric` (`klippy/extras/temperature_sensor.py:9`).
pub struct PrinterSensorGeneric {
    name: String,
    last_temp: SCell<f64>,
    measured_min: SCell<f64>,
    measured_max: SCell<f64>,
    _sensor: Arc<dyn TemperatureSensorApi>,
}

impl PrinterSensorGeneric {
    /// `PrinterSensorGeneric.__init__(config)`
    /// (`klippy/extras/temperature_sensor.py:10-24`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let pheaters = load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
            .expect("Raise yields Some");
        let sensor = pheaters.setup_sensor(config)?;

        let mut min_opts = GetFloatOpts::default();
        min_opts.default = Some(Some(KELVIN_TO_CELSIUS));
        min_opts.minval = Some(KELVIN_TO_CELSIUS);
        let min_temp = config.getfloat("min_temp", min_opts)?.unwrap();

        let mut max_opts = GetFloatOpts::default();
        max_opts.default = Some(Some(99999999.9));
        max_opts.above = Some(min_temp);
        let max_temp = config.getfloat("max_temp", max_opts)?.unwrap();

        let obj = Arc::new(PrinterSensorGeneric {
            name,
            last_temp: SCell::new(0.),
            measured_min: SCell::new(99999999.),
            measured_max: SCell::new(0.),
            _sensor: sensor.clone(),
        });

        sensor.setup_minmax(min_temp, max_temp)?;
        {
            let cb_obj = Arc::downgrade(&obj);
            sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
                let Some(cb_obj) = cb_obj.upgrade() else { return };
                cb_obj.temperature_callback(read_time, temp);
            }));
        }
        pheaters.register_sensor(config, obj.clone() as Arc<dyn crate::heaters::SensorApi>, None)?;
        Ok(obj)
    }

    /// `PrinterSensorGeneric.temperature_callback(read_time, temp)`
    /// (`klippy/extras/temperature_sensor.py:25-29`).
    pub fn temperature_callback(&self, _read_time: f64, temp: f64) {
        self.last_temp.set(temp);
        if temp != 0. {
            let cur_min = self.measured_min.get();
            let cur_max = self.measured_max.get();
            self.measured_min.set(f64::min(cur_min, temp));
            self.measured_max.set(f64::max(cur_max, temp));
        }
    }
}

impl crate::heaters::SensorApi for PrinterSensorGeneric {
    /// `PrinterSensorGeneric.get_temp(eventtime)`
    /// (`klippy/extras/temperature_sensor.py:30-31`).
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        (self.last_temp.get(), 0.)
    }
}

impl PrinterObject for PrinterSensorGeneric {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn crate::heaters::SensorApi);

    /// `PrinterSensorGeneric.get_status(eventtime)`
    /// (`klippy/extras/temperature_sensor.py:34-39`).
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(round_py_nd(self.last_temp.get(), 2)),
        );
        map.insert(
            PyValue::Str("measured_min_temp".to_string()),
            PyValue::Float(round_py_nd(self.measured_min.get(), 2)),
        );
        map.insert(
            PyValue::Str("measured_max_temp".to_string()),
            PyValue::Float(round_py_nd(self.measured_max.get(), 2)),
        );
        Ok(Some(PyValue::Dict(map)))
    }

    /// `PrinterSensorGeneric.stats(eventtime)`
    /// (`klippy/extras/temperature_sensor.py:32-33`).
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        let msg = match percent_format(
            "%s: temp=%.1f",
            &[PctValue::Str(self.name.clone()), PctValue::Float(self.last_temp.get())],
        ) {
            Ok(s) => s,
            Err(_) => String::new(),
        };
        Some((false, msg))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/temperature_sensor.py:41-42`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterSensorGeneric::new(config)? as ObjectRef))
}

/// Module entry point for the `temperature_sensor <name>` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
