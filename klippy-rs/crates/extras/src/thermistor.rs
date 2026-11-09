//! Port of `klippy/extras/thermistor.py`.
//!
//! Thermistors are ADC converters for [`PrinterADCtoTemperature`]. The
//! Steinhart-Hart / beta math is [`Thermistor`] from the
//! `extras_adc_temperature` crate; this module adds the config layer:
//! [`printer_thermistor`] (`PrinterThermistor`, the per-sensor
//! `pullup_resistor` / `inline_resistor` options and the coefficient
//! setup), [`CustomThermistor`] (the `[thermistor <name>]` sections,
//! including the default ones in `temperature_sensors.cfg`) and
//! [`load_config_prefix`], which registers each custom definition as a
//! sensor factory with `heaters`.
//!
//! The converter's math raises `ValueError` / `ZeroDivisionError` /
//! `OverflowError` like the reference; [`AdcConvert`] carries those as
//! [`KlippyError::Internal`] values with the Python class name.

use crate::adc_temperature::{custom_sensor_name, AdcConvert, PrinterADCtoTemperature};
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureSensorApi};
use extras_adc_temperature::thermistor::{Thermistor, KELVIN_TO_CELSIUS};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{load_interface, ObjectRef};
use pycompat::text::PyError;
use std::sync::Arc;

/// The exception a `Thermistor` method raised, as the uncaught Python
/// exception it is.
fn py_error(e: PyError) -> KlippyError {
    KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg }
}

impl AdcConvert for Thermistor {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        Thermistor::calc_temp(self, adc).map_err(py_error)
    }

    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        Thermistor::calc_adc(self, temp).map_err(py_error)
    }
}

/// Coefficient parameters of a custom thermistor (`CustomThermistor.params`,
/// lines 93 and 100-101): either `t1`, `r1` and `beta`, or three
/// temperature/resistance points sorted by temperature.
#[derive(Clone, Debug)]
pub enum ThermistorParams {
    /// `{'t1': t1, 'r1': r1, 'beta': beta}`.
    Beta { t1: f64, r1: f64, beta: f64 },
    /// `{'t1': t1, 'r1': r1, 't2': t2, 'r2': r2, 't3': t3, 'r3': r3}`.
    Points { t1: f64, r1: f64, t2: f64, r2: f64, t3: f64, r3: f64 },
}

/// `PrinterThermistor(config, params)` (lines 72-83): an ADC converter with
/// a thermistor.
///
/// A three-point set that does not give a positive `c3` falls back to beta
/// and logs `Using thermistor beta %.3f in heater %s` (the heater being
/// the section name of `config`).
pub fn printer_thermistor(config: &Config, params: &ThermistorParams) -> KResult<Arc<PrinterADCtoTemperature>> {
    let pullup = config
        .getfloat(
            "pullup_resistor",
            GetFloatOpts { default: Some(Some(4700.)), above: Some(0.), ..GetFloatOpts::default() },
        )?
        .expect("pullup_resistor has a default");
    let inline_resistor = config
        .getfloat(
            "inline_resistor",
            GetFloatOpts { default: Some(Some(0.)), minval: Some(0.), ..GetFloatOpts::default() },
        )?
        .expect("inline_resistor has a default");
    let mut thermistor = Thermistor::new(pullup, inline_resistor);
    match *params {
        ThermistorParams::Beta { t1, r1, beta } => {
            thermistor.setup_coefficients_beta(t1, r1, beta).map_err(py_error)?;
        }
        ThermistorParams::Points { t1, r1, t2, r2, t3, r3 } => {
            let warnings = thermistor
                .setup_coefficients(t1, r1, t2, r2, t3, r3, &config.get_name())
                .map_err(py_error)?;
            for line in &warnings {
                log::warning(line);
            }
        }
    }
    PrinterADCtoTemperature::new(config, Arc::new(thermistor))
}

/// `a < b` for the `(temperature, resistance)` tuples `sorted` compares
/// (`unsafe_tuple_compare` and `unsafe_float_compare` in CPython's
/// `Objects/listobject.c`): the first position whose floats are not `==`
/// decides with `<`, so a nan never orders before or after anything.
fn point_lt(a: &(f64, f64), b: &(f64, f64)) -> bool {
    if a.0 != b.0 {
        a.0 < b.0
    } else if a.1 != b.1 {
        a.1 < b.1
    } else {
        false
    }
}

/// `sorted([(t1, r1), (t2, r2), (t3, r3)])` as CPython 3.8 runs it for three
/// elements: `count_run` takes the leading ascending or strictly descending
/// run (reversing a descending one), then `binarysort` inserts the rest.
/// The comparison is not a total order when a value is nan, so the result
/// depends on this exact sequence of comparisons.
fn sorted_points(points: [(f64, f64); 3]) -> [(f64, f64); 3] {
    let mut a = points;
    let descending = point_lt(&a[1], &a[0]);
    let mut run = 2;
    while run < a.len() {
        let lt = point_lt(&a[run], &a[run - 1]);
        if lt != descending {
            break;
        }
        run += 1;
    }
    if descending {
        a[..run].reverse();
    }
    for start in run..a.len() {
        let pivot = a[start];
        let (mut l, mut r) = (0, start);
        loop {
            let p = l + ((r - l) >> 1);
            if point_lt(&pivot, &a[p]) {
                r = p;
            } else {
                l = p + 1;
            }
            if l >= r {
                break;
            }
        }
        for p in (l + 1..=start).rev() {
            a[p] = a[p - 1];
        }
        a[l] = pivot;
    }
    a
}

/// Required `config.getfloat(option, minval=minval)`.
fn required_float(config: &Config, option: &str, minval: f64) -> KResult<f64> {
    Ok(config
        .getfloat(option, GetFloatOpts { minval: Some(minval), ..GetFloatOpts::default() })?
        .expect("a required float option is present or raises"))
}

/// Custom defined thermistors from the config file (`class CustomThermistor`,
/// lines 86-103).
#[derive(Clone, Debug)]
pub struct CustomThermistor {
    name: String,
    params: ThermistorParams,
}

impl CustomThermistor {
    /// `CustomThermistor.__init__(config)` (lines 87-101). The options are
    /// read in the reference order; `temperature2`/`3` and `resistance2`/`3`
    /// are only read when `beta` is absent.
    pub fn new(config: &Config) -> KResult<Self> {
        let name = custom_sensor_name(config);
        let t1 = required_float(config, "temperature1", KELVIN_TO_CELSIUS)?;
        let r1 = required_float(config, "resistance1", 0.)?;
        let beta = config.getfloat(
            "beta",
            GetFloatOpts { default: Some(None), above: Some(0.), ..GetFloatOpts::default() },
        )?;
        if let Some(beta) = beta {
            return Ok(CustomThermistor { name, params: ThermistorParams::Beta { t1, r1, beta } });
        }
        let t2 = required_float(config, "temperature2", KELVIN_TO_CELSIUS)?;
        let r2 = required_float(config, "resistance2", 0.)?;
        let t3 = required_float(config, "temperature3", KELVIN_TO_CELSIUS)?;
        let r3 = required_float(config, "resistance3", 0.)?;
        let [(t1, r1), (t2, r2), (t3, r3)] = sorted_points([(t1, r1), (t2, r2), (t3, r3)]);
        Ok(CustomThermistor { name, params: ThermistorParams::Points { t1, r1, t2, r2, t3, r3 } })
    }

    /// `self.name` (line 88): the sensor type the section registers.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.params` (lines 93 and 100-101).
    pub fn params(&self) -> &ThermistorParams {
        &self.params
    }

    /// `create(self, config)` (lines 102-103).
    pub fn create(&self, config: &Config) -> KResult<Arc<PrinterADCtoTemperature>> {
        printer_thermistor(config, &self.params)
    }
}

/// `load_config_prefix(config)` (lines 105-108): registers the
/// `[thermistor <name>]` sensor with `heaters`. Returns no printer object.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let thermistor = Arc::new(CustomThermistor::new(config)?);
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    let name = thermistor.name().to_string();
    let factory: SensorFactory = Arc::new(move |config: &Config| {
        let sensor: Arc<dyn TemperatureSensorApi> = thermistor.create(config)?;
        Ok(sensor)
    });
    pheaters.add_sensor_factory(&name, factory);
    Ok(None)
}

/// Module entry point for the `thermistor` sections: the reference has
/// `load_config_prefix` only.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config_prefix: Some(load_config_prefix), ..ModuleFactory::default() }
}
