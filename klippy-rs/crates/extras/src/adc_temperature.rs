//! Port of `klippy/extras/adc_temperature.py`.
//!
//! ADC-to-temperature converters based on linear interpolation of
//! calibration tables: [`LinearInterpolate`] over `(key, value)` samples,
//! [`LinearVoltage`] for voltage tables and [`LinearResistance`] for
//! resistance tables. [`PrinterADCtoTemperature`] is the sensor chip the
//! `heaters` module uses: it wires an ADC pin to `-- calc_temp`/`calc_adc`
//! converters and the diagnostics helper ([`HelperTemperatureDiagnostics`])
//! that registers the ADC with `query_adc` and explains out-of-range
//! shutdowns via `error_mcu`. [`CustomLinearVoltage`] /
//! [`CustomLinearResistance`] parse `[adc_temperature <name>]` sections and
//! [`load_config`] / [`load_config_prefix`] register the sensor factories
//! with `heaters`.
//!
//! Float operations follow the Python source line by line, with no
//! reassociation or multiply-add: division uses plain `/` in the places
//! where the Python denominator cannot be zero (checked by the key-order
//! guard in [`LinearInterpolate::new`], the clamp in
//! [`LinearResistance::calc_temp`], `pullup > 0` in
//! [`LinearResistance::calc_adc`], and `adc_voltage > 0` in
//! [`LinearVoltage::new`]). A flat interpolation segment (equal values at
//! two keys) would raise `ZeroDivisionError` in Python from
//! `reverse_interpolate`; no built-in table has one.
//!
//! Thermistors (the `thermistor` module) convert through the same
//! [`AdcConvert`] interface.

use crate::error_mcu::PrinterMCUErrorApi;
use crate::heaters::{PrinterHeatersApi, SensorFactory, TemperatureCallback, TemperatureSensorApi};
use crate::query_adc::QueryAdcApi;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::SCell;
use klippy_api::log;
use klippy_api::mcu::{AdcPin, AdcSampleOpts, Pin, PinsApi};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{load_interface, lookup_interface, ObjectRef};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::cmp::Ordering;
use std::sync::Arc;

/// `SAMPLE_TIME` (`adc_temperature.py:13`).
pub const SAMPLE_TIME: f64 = 0.001;
/// `SAMPLE_COUNT` (`adc_temperature.py:14`).
pub const SAMPLE_COUNT: i64 = 8;
/// `REPORT_TIME` (`adc_temperature.py:15`).
pub const REPORT_TIME: f64 = 0.300;
/// `RANGE_CHECK_COUNT` (`adc_temperature.py:16`).
pub const RANGE_CHECK_COUNT: i64 = 4;

/// `SAMPLE_COUNT * SAMPLE_TIME` (`adc_temperature.py:34`).
const SAMPLE_DELAY: f64 = SAMPLE_COUNT as f64 * SAMPLE_TIME;

/// Internal Python exception helper (`ValueError`).
fn value_error(message: &str) -> KlippyError {
    KlippyError::Internal { class_name: "ValueError".to_string(), message: message.to_string() }
}

/// `config.error(...)` text (`LinearVoltage`/`LinearResistance`, lines
/// 140-141 and 172-173): `adc_temperature <str(e)> in heater <name>`.
fn config_error_for(config: &Config, detail: &KlippyError) -> KlippyError {
    KlippyError::Config(
        percent_format("adc_temperature %s in heater %s", &[PctValue::Str(detail.message().to_string()), PctValue::Str(config.get_name())])
            .expect("percent_format with string args"),
    )
}

// ---------------------------------------------------------------------------
// Linear interpolation  (lines 81-117)
// ---------------------------------------------------------------------------

/// Helper for linear interpolation (`class LinearInterpolate`,
/// lines 81-117). `samples` are `(key, value)` pairs.
#[derive(Clone, Debug)]
pub struct LinearInterpolate {
    keys: Vec<f64>,
    slopes: Vec<(f64, f64)>,
}

impl LinearInterpolate {
    /// `LinearInterpolate.__init__(samples)` (lines 82-104).
    pub fn new(samples: Vec<(f64, f64)>) -> KResult<Self> {
        // `sorted(samples)`: lexicographic on (key, value).
        let mut sorted = samples;
        sorted.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(Ordering::Equal)
                .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
        });
        let mut keys: Vec<f64> = Vec::new();
        let mut slopes: Vec<(f64, f64)> = Vec::new();
        let mut last_key: Option<f64> = None;
        let mut last_value = 0.0;
        for (key, value) in sorted {
            let Some(last) = last_key else {
                last_key = Some(key);
                last_value = value;
                continue;
            };
            if key <= last {
                return Err(value_error("duplicate value"));
            }
            let gain = (value - last_value) / (key - last);
            let offset = last_value - last * gain;
            if slopes.last() == Some(&(gain, offset)) {
                continue;
            }
            last_value = value;
            last_key = Some(key);
            keys.push(key);
            slopes.push((gain, offset));
        }
        if keys.is_empty() {
            return Err(value_error("need at least two samples"));
        }
        keys.push(9999999999999.0);
        let last = slopes[slopes.len() - 1];
        slopes.push(last);
        Ok(LinearInterpolate { keys, slopes })
    }

    /// `interpolate(self, key)` (lines 105-108): `bisect` (right).
    pub fn interpolate(&self, key: f64) -> f64 {
        let pos = self.keys.partition_point(|&k| k <= key);
        let (gain, offset) = self.slopes[pos];
        key * gain + offset
    }

    /// `reverse_interpolate(self, value)` (lines 109-117).
    pub fn reverse_interpolate(&self, value: f64) -> f64 {
        let values: Vec<f64> = self
            .keys
            .iter()
            .zip(self.slopes.iter())
            .map(|(key, (gain, offset))| key * gain + offset)
            .collect();
        let last = values.len() - 1;
        let idx = if values[0] < values[last - 1] {
            values.iter().position(|&v| v >= value).unwrap_or(last)
        } else {
            values.iter().position(|&v| v <= value).unwrap_or(last)
        };
        let (gain, offset) = self.slopes[idx];
        (value - offset) / gain
    }
}

// ---------------------------------------------------------------------------
// ADC converter interface (the `adc_convert` duck type)
// ---------------------------------------------------------------------------

/// ADC-to-temperature converter used by [`PrinterADCtoTemperature`]
/// (`adc_convert` in `adc_temperature.py:20`). It is implemented by
/// [`LinearVoltage`] and [`LinearResistance`] in this module, by the
/// `thermistor` module's converter and by `temperature_mcu`. Both methods
/// return the exception the Python converter raises (`ValueError: math
/// domain error`, `ZeroDivisionError`, ...).
pub trait AdcConvert: Send + Sync + 'static {
    /// `calc_temp(adc)` (calibration ADC -> temperature).
    fn calc_temp(&self, adc: f64) -> KResult<f64>;
    /// `calc_adc(temp)` (temperature -> calibration ADC).
    fn calc_adc(&self, temp: f64) -> KResult<f64>;
}

// ---------------------------------------------------------------------------
// Linear voltage converter  (lines 125-143)
// ---------------------------------------------------------------------------

/// Linear voltage to temperature converter (`class LinearVoltage`,
/// lines 125-143). `params` are `(temp, volt)` pairs.
#[derive(Debug)]
pub struct LinearVoltage {
    li: LinearInterpolate,
}

impl LinearVoltage {
    /// `LinearVoltage.__init__(config, params)` (lines 126-143).
    pub fn new(config: &Config, params: &[(f64, f64)]) -> KResult<Self> {
        let adc_voltage = config
            .getfloat(
                "adc_voltage",
                GetFloatOpts { default: Some(Some(5.)), above: Some(0.), ..GetFloatOpts::default() },
            )?
            .unwrap_or(5.);
        let voltage_offset =
            config
                .getfloat("voltage_offset", GetFloatOpts { default: Some(Some(0.0)), ..GetFloatOpts::default() })?
                .unwrap_or(0.0);
        let mut samples: Vec<(f64, f64)> = Vec::new();
        for (temp, volt) in params.iter().copied() {
            let adc = (volt - voltage_offset) / adc_voltage;
            // Written as two comparisons, not a range contains: Python
            // `adc < 0. or adc > 1.` keeps a nan sample (both comparisons
            // are false), while `RangeInclusive::contains` would drop it.
            #[allow(clippy::manual_range_contains)]
            let excluded = adc < 0. || adc > 1.;
            if excluded {
                if let Ok(msg) = percent_format(
                    "Ignoring adc sample %.3f/%.3f in heater %s",
                    &[PctValue::Float(temp), PctValue::Float(volt), PctValue::Str(config.get_name())],
                ) {
                    log::warning(&msg);
                }
                continue;
            }
            samples.push((adc, temp));
        }
        let li = match LinearInterpolate::new(samples) {
            Ok(li) => li,
            Err(e) => return Err(config_error_for(config, &e)),
        };
        Ok(LinearVoltage { li })
    }

    /// `calc_temp(adc)` (`li.interpolate`, line 142).
    pub fn calc_temp(&self, adc: f64) -> f64 {
        self.li.interpolate(adc)
    }

    /// `calc_adc(temp)` (`li.reverse_interpolate`, line 143).
    pub fn calc_adc(&self, temp: f64) -> f64 {
        self.li.reverse_interpolate(temp)
    }
}

impl AdcConvert for LinearVoltage {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        Ok(self.calc_temp(adc))
    }
    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        Ok(self.calc_adc(temp))
    }
}

// ---------------------------------------------------------------------------
// Linear resistance converter  (lines 166-182)
// ---------------------------------------------------------------------------

/// Linear resistance to temperature converter (`class LinearResistance`,
/// lines 166-182). `samples` are `(temp, resistance)` pairs.
#[derive(Debug)]
pub struct LinearResistance {
    pullup: f64,
    li: LinearInterpolate,
}

impl LinearResistance {
    /// `LinearResistance.__init__(config, samples)` (lines 167-173).
    pub fn new(config: &Config, samples: &[(f64, f64)]) -> KResult<Self> {
        let pullup = config
            .getfloat(
                "pullup_resistor",
                GetFloatOpts { default: Some(Some(4700.)), above: Some(0.), ..GetFloatOpts::default() },
            )?
            .unwrap_or(4700.);
        let swapped: Vec<(f64, f64)> = samples.iter().map(|(t, r)| (*r, *t)).collect();
        let li = match LinearInterpolate::new(swapped) {
            Ok(li) => li,
            Err(e) => return Err(config_error_for(config, &e)),
        };
        Ok(LinearResistance { pullup, li })
    }

    /// `pullup_resistor` (`self.pullup`, line 168).
    pub fn pullup(&self) -> f64 {
        self.pullup
    }

    /// `calc_temp(self, adc)` (lines 174-178).
    pub fn calc_temp(&self, adc: f64) -> f64 {
        let adc = 0.00001f64.max(0.99999f64.min(adc));
        let r = self.pullup * adc / (1.0 - adc);
        self.li.interpolate(r)
    }

    /// `calc_adc(self, temp)` (lines 179-182).
    pub fn calc_adc(&self, temp: f64) -> f64 {
        let r = self.li.reverse_interpolate(temp);
        r / (self.pullup + r)
    }
}

impl AdcConvert for LinearResistance {
    fn calc_temp(&self, adc: f64) -> KResult<f64> {
        Ok(self.calc_temp(adc))
    }
    fn calc_adc(&self, temp: f64) -> KResult<f64> {
        Ok(self.calc_adc(temp))
    }
}

// ---------------------------------------------------------------------------
// Diagnostics helper  (lines 44-73)
// ---------------------------------------------------------------------------

/// Tool to register with `query_adc` and report extra info on ADC range
/// errors (`class HelperTemperatureDiagnostics`, lines 44-73).
pub struct HelperTemperatureDiagnostics {
    name: String,
    mcu_adc: AdcPin,
    adc_convert: Arc<dyn AdcConvert>,
    // (min_temp, max_temp, min_adc, max_adc); `None` until `setup_diag_minmax`.
    bounds: SCell<Option<(f64, f64, f64, f64)>>,
}

impl HelperTemperatureDiagnostics {
    /// `HelperTemperatureDiagnostics.__init__(config, mcu_adc, calc_temp_cb)`
    /// (lines 45-54).
    pub fn new(config: &Config, mcu_adc: AdcPin, adc_convert: Arc<dyn AdcConvert>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let this = Arc::new(HelperTemperatureDiagnostics {
            name: name.clone(),
            mcu_adc: mcu_adc.clone(),
            adc_convert,
            bounds: SCell::new(None),
        });
        let query_adc = load_interface::<dyn QueryAdcApi>(&*printer, config, "query_adc", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        query_adc.register_adc(&name, mcu_adc);
        let error_mcu = load_interface::<dyn PrinterMCUErrorApi>(&*printer, config, "error_mcu", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let owner = Arc::clone(&this);
        error_mcu.add_clarify(
            "ADC out of range",
            Arc::new(move |msg, details| owner.clarify(msg, details)),
        );
        Ok(this)
    }

    /// Section name (`self.name`, line 47).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `setup_diag_minmax` (lines 55-57).
    pub fn setup_diag_minmax(&self, min_temp: f64, max_temp: f64, min_adc: f64, max_adc: f64) {
        *self.bounds.borrow_mut() = Some((min_temp, max_temp, min_adc, max_adc));
    }

    /// `_clarify_adc_range(self, msg, details)` (lines 58-73). `msg` and
    /// `details` are ignored, as in Python.
    ///
    /// The reference wraps `calc_temp_cb` in `try ... except e:`. When the
    /// callback raises, Python evaluates the handler expression `e`, which
    /// names no variable in the module, so the exception that leaves
    /// `_clarify_adc_range` is `NameError: name 'e' is not defined` and the
    /// `logging.exception` line is never reached.
    pub fn clarify(&self, _msg: &str, _details: &PyValue) -> KResult<Option<String>> {
        let Some((min_temp, max_temp, min_adc, max_adc)) = *self.bounds.borrow() else {
            return Ok(None);
        };
        let (last_read_time, last_value) = self.mcu_adc.get_last_value();
        if last_read_time == 0.0 {
            return Ok(None);
        }
        if last_value >= min_adc && last_value <= max_adc {
            return Ok(None);
        }
        let last_temp = match self.adc_convert.calc_temp(last_value) {
            Ok(temp) => temp,
            Err(_) => {
                return Err(KlippyError::Internal {
                    class_name: "NameError".to_string(),
                    message: "name 'e' is not defined".to_string(),
                });
            }
        };
        let tempstr = percent_format("%.3f", &[PctValue::Float(last_temp)]).expect("percent_format with float");
        let text = percent_format(
            "Sensor '%s' temperature %s not in range %.3f:%.3f",
            &[PctValue::Str(self.name.clone()), PctValue::Str(tempstr), PctValue::Float(min_temp), PctValue::Float(max_temp)],
        )
        .expect("percent_format with string/float args");
        Ok(Some(text))
    }
}

// ---------------------------------------------------------------------------
// Printer ADC to temperature  (lines 19-41)
// ---------------------------------------------------------------------------

struct Inner {
    adc_convert: Arc<dyn AdcConvert>,
    mcu_adc: AdcPin,
    temperature_callback: SCell<Option<TemperatureCallback>>,
}

impl Inner {
    /// `adc_callback(self, samples)` (lines 31-34). An empty batch raises
    /// `IndexError` from `samples[-1]`.
    fn adc_callback(&self, samples: &[(f64, f64)]) -> KResult<()> {
        let Some(&(read_time, read_value)) = samples.last() else {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        };
        let temp = self.adc_convert.calc_temp(read_value)?;
        let mut callback = self.temperature_callback.take_callback();
        if let Some(cb) = callback.as_mut() {
            cb(read_time + SAMPLE_DELAY, temp);
        }
        Ok(())
    }
}

/// An exception escaping the ADC callback. The reference calls it from
/// `MCU_adc._handle_analog_in_state`, which `SerialReader._bg_thread` runs
/// inside `try: ... except: logging.exception("%sException in serial
/// callback")` (`serialhdl.py:60-66`): the exception is logged and the
/// reader keeps going. The `AdcCallback` contract has no error channel, and
/// the serial dispatcher guards every handler with the same log line, so a
/// panic with the exception's class and message reproduces that.
fn raise_in_serial_callback(err: KlippyError) -> ! {
    err.raise()
}

/// Interface between MCU adc and heater temperature callbacks
/// (`class PrinterADCtoTemperature`, lines 19-41).
pub struct PrinterADCtoTemperature {
    inner: Arc<Inner>,
    diag: Arc<HelperTemperatureDiagnostics>,
}

impl PrinterADCtoTemperature {
    /// `PrinterADCtoTemperature.__init__(config, adc_convert)`
    /// (lines 20-26).
    pub fn new(config: &Config, adc_convert: Arc<dyn AdcConvert>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let sensor_pin = config
            .get("sensor_pin", GetStrOpts::default())?
            .ok_or_else(|| {
                KlippyError::Config(format!(
                    "Option 'sensor_pin' in section '{}' must be specified",
                    config.get_name()
                ))
            })?;
        let mcu_adc = match ppins.setup_pin("adc", &sensor_pin)? {
            Pin::Adc(adc) => adc,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "pins.setup_pin('adc', ...) did not return an ADC pin".to_string(),
                });
            }
        };
        let inner = Arc::new(Inner { adc_convert: adc_convert.clone(), mcu_adc: mcu_adc.clone(), temperature_callback: SCell::new(None) });
        let callback_inner = Arc::downgrade(&inner);
        mcu_adc.setup_adc_callback(Box::new(move |samples| {
            let Some(callback_inner) = callback_inner.upgrade() else { return };
            if let Err(err) = callback_inner.adc_callback(&samples) {
                raise_in_serial_callback(err);
            }
        }));
        let diag = HelperTemperatureDiagnostics::new(config, mcu_adc, adc_convert)?;
        Ok(Arc::new(PrinterADCtoTemperature { inner, diag }))
    }

    /// `adc_callback(self, samples)` (lines 31-34). Returns the exception
    /// the converter raises; the registered ADC callback reports it as an
    /// exception in the serial callback.
    pub fn adc_callback(&self, samples: &[(f64, f64)]) -> KResult<()> {
        self.inner.adc_callback(samples)
    }

    /// `setup_callback(self, temperature_callback)` (lines 27-28).
    pub fn setup_callback(&self, temperature_callback: TemperatureCallback) {
        *self.inner.temperature_callback.borrow_mut() = Some(temperature_callback);
    }

    /// `get_report_time_delta(self)` (lines 29-30).
    pub fn get_report_time_delta(&self) -> f64 {
        REPORT_TIME
    }

    /// `setup_minmax(self, min_temp, max_temp)` (lines 35-41). A converter
    /// exception from `calc_adc` propagates before the ADC is configured.
    pub fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        let a = self.inner.adc_convert.calc_adc(min_temp)?;
        let b = self.inner.adc_convert.calc_adc(max_temp)?;
        let (min_adc, max_adc) = if b < a { (b, a) } else { (a, b) };
        self.inner.mcu_adc.setup_adc_sample(
            REPORT_TIME,
            AdcSampleOpts {
                sample_time: SAMPLE_TIME,
                sample_count: SAMPLE_COUNT,
                batch_num: 1,
                minval: min_adc,
                maxval: max_adc,
                range_check_count: RANGE_CHECK_COUNT,
            },
        );
        self.diag.setup_diag_minmax(min_temp, max_temp, min_adc, max_adc);
        Ok(())
    }

    /// The ADC pin set up from `sensor_pin` (`self.mcu_adc`).
    pub fn mcu_adc(&self) -> AdcPin {
        self.inner.mcu_adc.clone()
    }

    /// The diagnostics helper (`self.diag_helper`).
    pub fn diag_helper(&self) -> Arc<HelperTemperatureDiagnostics> {
        self.diag.clone()
    }
}

impl TemperatureSensorApi for PrinterADCtoTemperature {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        PrinterADCtoTemperature::setup_minmax(self, min_temp, max_temp)
    }
    fn setup_callback(&self, callback: TemperatureCallback) {
        self.setup_callback(callback);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.get_report_time_delta()
    }
}

// ---------------------------------------------------------------------------
// Custom sensors  (lines 145-158 and 184-197)
// ---------------------------------------------------------------------------

/// Section name without the module token
/// (`" ".join(config.get_name().split()[1:])`, lines 148 and 187).
pub fn custom_sensor_name(config: &Config) -> String {
    config
        .get_name()
        .split_whitespace()
        .skip(1)
        .collect::<Vec<_>>()
        .join(" ")
}

/// `config.getfloat("temperature%d" % i, None)` (lines 151 and 190):
/// `None` when the option is missing.
fn option_float(config: &Config, option: &str) -> KResult<Option<f64>> {
    config.getfloat(option, GetFloatOpts { default: Some(None), ..GetFloatOpts::default() })
}

/// Required `config.getfloat(option)`: missing raises the config error.
fn required_float(config: &Config, option: String) -> KResult<f64> {
    config
        .getfloat(&option, GetFloatOpts::default())?
        .ok_or_else(|| KlippyError::Config(format!("Option '{option}' in section '{}' must be specified", config.get_name())))
}

/// Custom defined voltage sensors (`class CustomLinearVoltage`,
/// lines 145-158).
#[derive(Debug)]
pub struct CustomLinearVoltage {
    name: String,
    params: Vec<(f64, f64)>,
}

impl CustomLinearVoltage {
    /// `CustomLinearVoltage.__init__(config)` (lines 147-155).
    pub fn new(config: &Config) -> KResult<Self> {
        let name = custom_sensor_name(config);
        let mut params = Vec::new();
        for i in 1..1000 {
            let Some(t) = option_float(config, &format!("temperature{i}"))? else { break };
            let v = required_float(config, format!("voltage{i}"))?;
            params.push((t, v));
        }
        Ok(CustomLinearVoltage { name, params })
    }

    /// `self.name` (line 148).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.params` (line 149).
    pub fn params(&self) -> &[(f64, f64)] {
        &self.params
    }

    /// `create(self, config)` (lines 156-158).
    pub fn create(&self, config: &Config) -> KResult<Arc<PrinterADCtoTemperature>> {
        let lv = LinearVoltage::new(config, &self.params)?;
        PrinterADCtoTemperature::new(config, Arc::new(lv))
    }
}

/// Custom defined resistance sensors (`class CustomLinearResistance`,
/// lines 184-197).
#[derive(Debug)]
pub struct CustomLinearResistance {
    name: String,
    samples: Vec<(f64, f64)>,
}

impl CustomLinearResistance {
    /// `CustomLinearResistance.__init__(config)` (lines 186-194).
    pub fn new(config: &Config) -> KResult<Self> {
        let name = custom_sensor_name(config);
        let mut samples = Vec::new();
        for i in 1..1000 {
            let Some(t) = option_float(config, &format!("temperature{i}"))? else { break };
            let r = required_float(config, format!("resistance{i}"))?;
            samples.push((t, r));
        }
        Ok(CustomLinearResistance { name, samples })
    }

    /// `self.name` (line 187).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `self.samples` (line 188).
    pub fn samples(&self) -> &[(f64, f64)] {
        &self.samples
    }

    /// `create(self, config)` (lines 195-197).
    pub fn create(&self, config: &Config) -> KResult<Arc<PrinterADCtoTemperature>> {
        let lr = LinearResistance::new(config, &self.samples)?;
        PrinterADCtoTemperature::new(config, Arc::new(lr))
    }
}

// ---------------------------------------------------------------------------
// Default sensors  (lines 204-316)
// ---------------------------------------------------------------------------

/// `AD595` (`adc_temperature.py:204-211`).
pub const AD595: &[(f64, f64)] = &[
    (0.0, 0.0027),
    (10.0, 0.101),
    (20.0, 0.200),
    (25.0, 0.250),
    (30.0, 0.300),
    (40.0, 0.401),
    (50.0, 0.503),
    (60.0, 0.605),
    (80.0, 0.810),
    (100.0, 1.015),
    (120.0, 1.219),
    (140.0, 1.420),
    (160.0, 1.620),
    (180.0, 1.817),
    (200.0, 2.015),
    (220.0, 2.213),
    (240.0, 2.413),
    (260.0, 2.614),
    (280.0, 2.817),
    (300.0, 3.022),
    (320.0, 3.227),
    (340.0, 3.434),
    (360.0, 3.641),
    (380.0, 3.849),
    (400.0, 4.057),
    (420.0, 4.266),
    (440.0, 4.476),
    (460.0, 4.686),
    (480.0, 4.896),
];

/// `AD597` (`adc_temperature.py:213-220`).
pub const AD597: &[(f64, f64)] = &[
    (0.0, 0.0),
    (10.0, 0.097),
    (20.0, 0.196),
    (25.0, 0.245),
    (30.0, 0.295),
    (40.0, 0.395),
    (50.0, 0.496),
    (60.0, 0.598),
    (80.0, 0.802),
    (100.0, 1.005),
    (120.0, 1.207),
    (140.0, 1.407),
    (160.0, 1.605),
    (180.0, 1.801),
    (200.0, 1.997),
    (220.0, 2.194),
    (240.0, 2.392),
    (260.0, 2.592),
    (280.0, 2.794),
    (300.0, 2.996),
    (320.0, 3.201),
    (340.0, 3.406),
    (360.0, 3.611),
    (380.0, 3.817),
    (400.0, 4.024),
    (420.0, 4.232),
    (440.0, 4.440),
    (460.0, 4.649),
    (480.0, 4.857),
    (500.0, 5.066),
];

/// `AD8494` (`adc_temperature.py:222-238`).
pub const AD8494: &[(f64, f64)] = &[
    (-180.0, -0.714),
    (-160.0, -0.658),
    (-140.0, -0.594),
    (-120.0, -0.523),
    (-100.0, -0.446),
    (-80.0, -0.365),
    (-60.0, -0.278),
    (-40.0, -0.188),
    (-20.0, -0.095),
    (0.0, 0.002),
    (20.0, 0.1),
    (25.0, 0.125),
    (40.0, 0.201),
    (60.0, 0.303),
    (80.0, 0.406),
    (100.0, 0.511),
    (120.0, 0.617),
    (140.0, 0.723),
    (160.0, 0.829),
    (180.0, 0.937),
    (200.0, 1.044),
    (220.0, 1.151),
    (240.0, 1.259),
    (260.0, 1.366),
    (280.0, 1.473),
    (300.0, 1.58),
    (320.0, 1.687),
    (340.0, 1.794),
    (360.0, 1.901),
    (380.0, 2.008),
    (400.0, 2.114),
    (420.0, 2.221),
    (440.0, 2.328),
    (460.0, 2.435),
    (480.0, 2.542),
    (500.0, 2.65),
    (520.0, 2.759),
    (540.0, 2.868),
    (560.0, 2.979),
    (580.0, 3.09),
    (600.0, 3.203),
    (620.0, 3.316),
    (640.0, 3.431),
    (660.0, 3.548),
    (680.0, 3.666),
    (700.0, 3.786),
    (720.0, 3.906),
    (740.0, 4.029),
    (760.0, 4.152),
    (780.0, 4.276),
    (800.0, 4.401),
    (820.0, 4.526),
    (840.0, 4.65),
    (860.0, 4.774),
    (880.0, 4.897),
    (900.0, 5.018),
    (920.0, 5.138),
    (940.0, 5.257),
    (960.0, 5.374),
    (980.0, 5.49),
    (1000.0, 5.606),
    (1020.0, 5.72),
    (1040.0, 5.833),
    (1060.0, 5.946),
    (1080.0, 6.058),
    (1100.0, 6.17),
    (1120.0, 6.282),
    (1140.0, 6.394),
    (1160.0, 6.505),
    (1180.0, 6.616),
    (1200.0, 6.727),
];

/// `AD8495` (`adc_temperature.py:240-258`).
pub const AD8495: &[(f64, f64)] = &[
    (-260.0, -0.786),
    (-240.0, -0.774),
    (-220.0, -0.751),
    (-200.0, -0.719),
    (-180.0, -0.677),
    (-160.0, -0.627),
    (-140.0, -0.569),
    (-120.0, -0.504),
    (-100.0, -0.432),
    (-80.0, -0.355),
    (-60.0, -0.272),
    (-40.0, -0.184),
    (-20.0, -0.093),
    (0.0, 0.003),
    (20.0, 0.1),
    (25.0, 0.125),
    (40.0, 0.2),
    (60.0, 0.301),
    (80.0, 0.402),
    (100.0, 0.504),
    (120.0, 0.605),
    (140.0, 0.705),
    (160.0, 0.803),
    (180.0, 0.901),
    (200.0, 0.999),
    (220.0, 1.097),
    (240.0, 1.196),
    (260.0, 1.295),
    (280.0, 1.396),
    (300.0, 1.497),
    (320.0, 1.599),
    (340.0, 1.701),
    (360.0, 1.803),
    (380.0, 1.906),
    (400.0, 2.01),
    (420.0, 2.113),
    (440.0, 2.217),
    (460.0, 2.321),
    (480.0, 2.425),
    (500.0, 2.529),
    (520.0, 2.634),
    (540.0, 2.738),
    (560.0, 2.843),
    (580.0, 2.947),
    (600.0, 3.051),
    (620.0, 3.155),
    (640.0, 3.259),
    (660.0, 3.362),
    (680.0, 3.465),
    (700.0, 3.568),
    (720.0, 3.67),
    (740.0, 3.772),
    (760.0, 3.874),
    (780.0, 3.975),
    (800.0, 4.076),
    (820.0, 4.176),
    (840.0, 4.275),
    (860.0, 4.374),
    (880.0, 4.473),
    (900.0, 4.571),
    (920.0, 4.669),
    (940.0, 4.766),
    (960.0, 4.863),
    (980.0, 4.959),
    (1000.0, 5.055),
    (1020.0, 5.15),
    (1040.0, 5.245),
    (1060.0, 5.339),
    (1080.0, 5.432),
    (1100.0, 5.525),
    (1120.0, 5.617),
    (1140.0, 5.709),
    (1160.0, 5.8),
    (1180.0, 5.891),
    (1200.0, 5.98),
    (1220.0, 6.069),
    (1240.0, 6.158),
    (1260.0, 6.245),
    (1280.0, 6.332),
    (1300.0, 6.418),
    (1320.0, 6.503),
    (1340.0, 6.587),
    (1360.0, 6.671),
    (1380.0, 6.754),
];

/// `AD8496` (`adc_temperature.py:260-276`).
pub const AD8496: &[(f64, f64)] = &[
    (-180.0, -0.642),
    (-160.0, -0.59),
    (-140.0, -0.53),
    (-120.0, -0.464),
    (-100.0, -0.392),
    (-80.0, -0.315),
    (-60.0, -0.235),
    (-40.0, -0.15),
    (-20.0, -0.063),
    (0.0, 0.027),
    (20.0, 0.119),
    (25.0, 0.142),
    (40.0, 0.213),
    (60.0, 0.308),
    (80.0, 0.405),
    (100.0, 0.503),
    (120.0, 0.601),
    (140.0, 0.701),
    (160.0, 0.8),
    (180.0, 0.9),
    (200.0, 1.001),
    (220.0, 1.101),
    (240.0, 1.201),
    (260.0, 1.302),
    (280.0, 1.402),
    (300.0, 1.502),
    (320.0, 1.602),
    (340.0, 1.702),
    (360.0, 1.801),
    (380.0, 1.901),
    (400.0, 2.001),
    (420.0, 2.1),
    (440.0, 2.2),
    (460.0, 2.3),
    (480.0, 2.401),
    (500.0, 2.502),
    (520.0, 2.603),
    (540.0, 2.705),
    (560.0, 2.808),
    (580.0, 2.912),
    (600.0, 3.017),
    (620.0, 3.124),
    (640.0, 3.231),
    (660.0, 3.34),
    (680.0, 3.451),
    (700.0, 3.562),
    (720.0, 3.675),
    (740.0, 3.789),
    (760.0, 3.904),
    (780.0, 4.02),
    (800.0, 4.137),
    (820.0, 4.254),
    (840.0, 4.37),
    (860.0, 4.486),
    (880.0, 4.6),
    (900.0, 4.714),
    (920.0, 4.826),
    (940.0, 4.937),
    (960.0, 5.047),
    (980.0, 5.155),
    (1000.0, 5.263),
    (1020.0, 5.369),
    (1040.0, 5.475),
    (1060.0, 5.581),
    (1080.0, 5.686),
    (1100.0, 5.79),
    (1120.0, 5.895),
    (1140.0, 5.999),
    (1160.0, 6.103),
    (1180.0, 6.207),
    (1200.0, 6.311),
];

/// `AD8497` (`adc_temperature.py:278-297`).
pub const AD8497: &[(f64, f64)] = &[
    (-260.0, -0.785),
    (-240.0, -0.773),
    (-220.0, -0.751),
    (-200.0, -0.718),
    (-180.0, -0.676),
    (-160.0, -0.626),
    (-140.0, -0.568),
    (-120.0, -0.503),
    (-100.0, -0.432),
    (-80.0, -0.354),
    (-60.0, -0.271),
    (-40.0, -0.184),
    (-20.0, -0.092),
    (0.0, 0.003),
    (20.0, 0.101),
    (25.0, 0.126),
    (40.0, 0.2),
    (60.0, 0.301),
    (80.0, 0.403),
    (100.0, 0.505),
    (120.0, 0.605),
    (140.0, 0.705),
    (160.0, 0.804),
    (180.0, 0.902),
    (200.0, 0.999),
    (220.0, 1.097),
    (240.0, 1.196),
    (260.0, 1.296),
    (280.0, 1.396),
    (300.0, 1.498),
    (320.0, 1.599),
    (340.0, 1.701),
    (360.0, 1.804),
    (380.0, 1.907),
    (400.0, 2.01),
    (420.0, 2.114),
    (440.0, 2.218),
    (460.0, 2.322),
    (480.0, 2.426),
    (500.0, 2.53),
    (520.0, 2.634),
    (540.0, 2.739),
    (560.0, 2.843),
    (580.0, 2.948),
    (600.0, 3.052),
    (620.0, 3.156),
    (640.0, 3.259),
    (660.0, 3.363),
    (680.0, 3.466),
    (700.0, 3.569),
    (720.0, 3.671),
    (740.0, 3.773),
    (760.0, 3.874),
    (780.0, 3.976),
    (800.0, 4.076),
    (820.0, 4.176),
    (840.0, 4.276),
    (860.0, 4.375),
    (880.0, 4.474),
    (900.0, 4.572),
    (920.0, 4.67),
    (940.0, 4.767),
    (960.0, 4.863),
    (980.0, 4.96),
    (1000.0, 5.055),
    (1020.0, 5.151),
    (1040.0, 5.245),
    (1060.0, 5.339),
    (1080.0, 5.433),
    (1100.0, 5.526),
    (1120.0, 5.618),
    (1140.0, 5.71),
    (1160.0, 5.801),
    (1180.0, 5.891),
    (1200.0, 5.981),
    (1220.0, 6.07),
    (1240.0, 6.158),
    (1260.0, 6.246),
    (1280.0, 6.332),
    (1300.0, 6.418),
    (1320.0, 6.503),
    (1340.0, 6.588),
    (1360.0, 6.671),
    (1380.0, 6.754),
];

/// `calc_pt100(base=100.)` (lines 299-302): PT100/PT1000 resistances from
/// the Callendar-Van Dusen formula (`A`, `B` below).
pub fn calc_pt100(base: f64) -> Vec<(f64, f64)> {
    const A: f64 = 3.9083e-3;
    const B: f64 = -5.775e-7;
    (0..500)
        .step_by(10)
        .map(|i| {
            let t = i as f64;
            (t, base * (1. + A * t + B * t * t))
        })
        .collect()
}

/// `calc_ina826_pt100()` (lines 304-306): 4400 ohm pullup, 10x gain to 5 V.
pub fn calc_ina826_pt100() -> Vec<(f64, f64)> {
    calc_pt100(100.).iter().map(|(t, r)| (*t, 10. * 5. * r / (4400. + r))).collect()
}

/// `DefaultVoltageSensors` (lines 308-312) as `(sensor_type, params)`.
pub fn default_voltage_sensors() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![
        ("AD595", AD595.to_vec()),
        ("AD597", AD597.to_vec()),
        ("AD8494", AD8494.to_vec()),
        ("AD8495", AD8495.to_vec()),
        ("AD8496", AD8496.to_vec()),
        ("AD8497", AD8497.to_vec()),
        ("PT100 INA826", calc_ina826_pt100()),
    ]
}

/// `DefaultResistanceSensors` (lines 314-316) as `(sensor_type, params)`.
pub fn default_resistance_sensors() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![("PT1000", calc_pt100(1000.))]
}

// ---------------------------------------------------------------------------
// Module entry points  (lines 318-337)
// ---------------------------------------------------------------------------

/// `load_config(config)` (lines 318-329): registers the default sensors
/// with `heaters`. Returns no printer object.
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let printer = config.get_printer();
    let pheaters = load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    for (sensor_type, params) in default_voltage_sensors() {
        let factory: SensorFactory = Arc::new(move |config: &Config| {
            let lv = LinearVoltage::new(config, &params)?;
            let sensor: Arc<dyn TemperatureSensorApi> = PrinterADCtoTemperature::new(config, Arc::new(lv))?;
            Ok(sensor)
        });
        pheaters.add_sensor_factory(sensor_type, factory);
    }
    for (sensor_type, params) in default_resistance_sensors() {
        let factory: SensorFactory = Arc::new(move |config: &Config| {
            let lr = LinearResistance::new(config, &params)?;
            let sensor: Arc<dyn TemperatureSensorApi> = PrinterADCtoTemperature::new(config, Arc::new(lr))?;
            Ok(sensor)
        });
        pheaters.add_sensor_factory(sensor_type, factory);
    }
    Ok(None)
}

/// `load_config_prefix(config)` (lines 331-337): registers the custom
/// `[adc_temperature <name>]` sensor with `heaters`. Returns no printer
/// object.
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let has_resistance = config
        .get(
            "resistance1",
            GetStrOpts { default: Some(None), note_valid: true },
        )?
        .is_some();
    let printer = config.get_printer();
    if has_resistance {
        let custom = Arc::new(CustomLinearResistance::new(config)?);
        let pheaters =
            load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        let owner = Arc::clone(&custom);
        let name = custom.name().to_string();
        let factory: SensorFactory = Arc::new(move |config: &Config| {
            let sensor: Arc<dyn TemperatureSensorApi> = owner.create(config)?;
            Ok(sensor)
        });
        pheaters.add_sensor_factory(&name, factory);
    } else {
        let custom = Arc::new(CustomLinearVoltage::new(config)?);
        let pheaters =
            load_interface::<dyn PrinterHeatersApi>(&*printer, config, "heaters", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
        let owner = Arc::clone(&custom);
        let name = custom.name().to_string();
        let factory: SensorFactory = Arc::new(move |config: &Config| {
            let sensor: Arc<dyn TemperatureSensorApi> = owner.create(config)?;
            Ok(sensor)
        });
        pheaters.add_sensor_factory(&name, factory);
    }
    Ok(None)
}

/// Module entry point for the `adc_temperature` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: Some(load_config), load_config_prefix: Some(load_config_prefix), ..ModuleFactory::default() }
}
