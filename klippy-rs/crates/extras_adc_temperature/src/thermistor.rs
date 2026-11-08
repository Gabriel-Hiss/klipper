//! Port of `klippy/extras/thermistor.py`.
//!
//! Implements the `Thermistor` Steinhart-Hart / beta converter, the
//! `[thermistor <name>]` custom-sensor config parsing (`CustomThermistor`)
//! and the per-sensor config parsing (`PrinterThermistor`). The
//! `adc_temperature` side it plugs into (`PrinterADCtoTemperature`) is not
//! part of this module; only the converter interface used by it is
//! reproduced: `calc_temp` (ADC to temperature) and `calc_adc`
//! (temperature to ADC, used by `setup_minmax` to compute ADC limits).
//!
//! Floating point follows the Python source line by line: transcendentals
//! call glibc libm through `pycompat::libm`, operation order matches the
//! source, and no fused multiply-add is used.

use pycompat::libm;
use pycompat::text::{float_pow, float_repr, percent_format, PctValue, PyError};
use pytext::configparser::RawConfigParser;

/// `KELVIN_TO_CELSIUS = -273.15` (thermistor.py line 9).
pub const KELVIN_TO_CELSIUS: f64 = -273.15;

/// `math.log` result with CPython's domain check: `math.log` raises
fn math_log(x: f64) -> Result<f64, PyError> {
    if x <= 0.0 {
        return Err(PyError::new("ValueError", "math domain error".to_string()));
    }
    Ok(libm::log(x))
}

/// `math.sqrt` with CPython's domain check.
fn math_sqrt(x: f64) -> Result<f64, PyError> {
    if x < 0.0 {
        return Err(PyError::new("ValueError", "math domain error".to_string()));
    }
    Ok(libm::sqrt(x))
}

/// `math.pow` with CPython's domain check: a negative base with a
/// non-integer exponent raises `ValueError: math domain error`.
fn math_pow(x: f64, y: f64) -> Result<f64, PyError> {
    if x < 0.0 && x.is_finite() && y.is_finite() && y.fract() != 0.0 {
        return Err(PyError::new("ValueError", "math domain error".to_string()));
    }
    Ok(libm::pow(x, y))
}

/// `math.exp` with CPython's range check: finite input whose result
/// overflows raises `OverflowError: math range error`.
fn math_exp(x: f64) -> Result<f64, PyError> {
    let r = libm::exp(x);
    if r.is_infinite() && x.is_finite() {
        return Err(PyError::new("OverflowError", "math range error".to_string()));
    }
    Ok(r)
}

/// `a / b` for floats with CPython's zero check:
/// `ZeroDivisionError: float division by zero`.
fn float_div(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new(
            "ZeroDivisionError",
            "float division by zero".to_string(),
        ));
    }
    Ok(a / b)
}

/// Steinhart-Hart thermistor converter (`class Thermistor`, lines 12-69).
#[derive(Debug)]
pub struct Thermistor {
    pullup: f64,
    inline_resistor: f64,
    c1: f64,
    c2: f64,
    c3: f64,
}

impl Thermistor {
    /// `Thermistor(pullup, inline_resistor)`; coefficients start at 0.
    pub fn new(pullup: f64, inline_resistor: f64) -> Thermistor {
        Thermistor {
            pullup,
            inline_resistor,
            c1: 0.0,
            c2: 0.0,
            c3: 0.0,
        }
    }

    /// Steinhart-Hart coefficient `c1`.
    pub fn c1(&self) -> f64 {
        self.c1
    }

    /// Steinhart-Hart coefficient `c2`.
    pub fn c2(&self) -> f64 {
        self.c2
    }

    /// Steinhart-Hart coefficient `c3`.
    pub fn c3(&self) -> f64 {
        self.c3
    }

    /// `setup_coefficients(t1, r1, t2, r2, t3, r3, name="")`.
    /// Returns the `logging.warning` lines emitted (only the beta fallback
    /// logs), so the caller can reproduce the log output.
    #[allow(clippy::too_many_arguments)]
    pub fn setup_coefficients(
        &mut self,
        t1: f64,
        r1: f64,
        t2: f64,
        r2: f64,
        t3: f64,
        r3: f64,
        name: &str,
    ) -> Result<Vec<String>, PyError> {
        let mut log_lines = Vec::new();
        let inv_t1 = float_div(1.0, t1 - KELVIN_TO_CELSIUS)?;
        let inv_t2 = float_div(1.0, t2 - KELVIN_TO_CELSIUS)?;
        let inv_t3 = float_div(1.0, t3 - KELVIN_TO_CELSIUS)?;
        let ln_r1 = math_log(r1)?;
        let ln_r2 = math_log(r2)?;
        let ln_r3 = math_log(r3)?;
        let ln3_r1 = float_pow(ln_r1, 3.0)?;
        let ln3_r2 = float_pow(ln_r2, 3.0)?;
        let ln3_r3 = float_pow(ln_r3, 3.0)?;

        let inv_t12 = inv_t1 - inv_t2;
        let inv_t13 = inv_t1 - inv_t3;
        let ln_r12 = ln_r1 - ln_r2;
        let ln_r13 = ln_r1 - ln_r3;
        let ln3_r12 = ln3_r1 - ln3_r2;
        let ln3_r13 = ln3_r1 - ln3_r3;

        self.c3 = float_div(
            inv_t12 - float_div(inv_t13 * ln_r12, ln_r13)?,
            ln3_r12 - float_div(ln3_r13 * ln_r12, ln_r13)?,
        )?;
        if self.c3 <= 0.0 {
            let beta = float_div(ln_r13, inv_t13)?;
            let warning = percent_format(
                "Using thermistor beta %.3f in heater %s",
                &[PctValue::Float(beta), PctValue::Str(name.to_string())],
            )?;
            log_lines.push(warning);
            self.setup_coefficients_beta(t1, r1, beta)?;
            return Ok(log_lines);
        }
        self.c2 = float_div(inv_t12 - self.c3 * ln3_r12, ln_r12)?;
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1;
        Ok(log_lines)
    }

    /// `setup_coefficients_beta(t1, r1, beta)`.
    pub fn setup_coefficients_beta(&mut self, t1: f64, r1: f64, beta: f64) -> Result<(), PyError> {
        let inv_t1 = float_div(1.0, t1 - KELVIN_TO_CELSIUS)?;
        let ln_r1 = math_log(r1)?;
        self.c3 = 0.0;
        self.c2 = float_div(1.0, beta)?;
        self.c1 = inv_t1 - self.c2 * ln_r1;
        Ok(())
    }

    /// `calc_temp(adc)`: ADC reading to temperature in Celsius.
    pub fn calc_temp(&self, adc: f64) -> Result<f64, PyError> {
        // `adc = max(.00001, min(.99999, adc))` with CPython comparison
        // semantics: comparisons with NaN are false, so a NaN input
        // clamps to 0.99999.
        let adc = if adc < 0.99999 { adc } else { 0.99999 };
        let adc = if adc > 0.00001 { adc } else { 0.00001 };
        let r = self.pullup * adc / (1.0 - adc);
        let ln_r = math_log(r - self.inline_resistor)?;
        let inv_t = self.c1 + self.c2 * ln_r + self.c3 * float_pow(ln_r, 3.0)?;
        Ok(float_div(1.0, inv_t)? + KELVIN_TO_CELSIUS)
    }

    /// `calc_adc(temp)`: temperature in Celsius to ADC reading.
    pub fn calc_adc(&self, temp: f64) -> Result<f64, PyError> {
        if temp <= KELVIN_TO_CELSIUS {
            return Ok(1.0);
        }
        let inv_t = float_div(1.0, temp - KELVIN_TO_CELSIUS)?;
        let ln_r = if self.c3 != 0.0 {
            // Solve for ln_r using Cardano's formula.
            let y = (self.c1 - inv_t) / (2.0 * self.c3);
            let x = math_sqrt(float_pow(self.c2 / (3.0 * self.c3), 3.0)? + float_pow(y, 2.0)?)?;
            let third = 1.0 / 3.0;
            math_pow(x - y, third)? - math_pow(x + y, third)?
        } else {
            float_div(inv_t - self.c1, self.c2)?
        };
        let r = math_exp(ln_r)? + self.inline_resistor;
        float_div(r, self.pullup + r)
    }
}

/// Coefficient parameters parsed from a `[thermistor <name>]` section
/// (`CustomThermistor.params`).
#[derive(Debug)]
pub struct ThermistorParams {
    /// `t1` in Celsius.
    pub t1: f64,
    /// `r1` in ohms.
    pub r1: f64,
    /// `t2` in Celsius (absent for beta-defined sensors).
    pub t2: f64,
    /// `r2` in ohms (absent for beta-defined sensors).
    pub r2: f64,
    /// `t3` in Celsius (absent for beta-defined sensors).
    pub t3: f64,
    /// `r3` in ohms (absent for beta-defined sensors).
    pub r3: f64,
    /// `beta`, present for beta-defined sensors.
    pub beta: Option<f64>,
}

/// A custom thermistor definition (`class CustomThermistor`).
#[derive(Debug)]
pub struct CustomThermistor {
    /// Sensor name: the section name without the `thermistor` prefix.
    pub name: String,
    /// Coefficient parameters.
    pub params: ThermistorParams,
}

/// `config.getfloat` with `ConfigWrapper._get_wrapper` semantics
/// (klippy/configfile.py lines 29-60): defaults, and the exact error texts
/// for a missing option, an unparsable value, and range violations.
/// Returns the error text to raise as a `configparser.Error` message.
fn getfloat(
    parser: &RawConfigParser,
    section: &str,
    option: &str,
    default: Option<f64>,
    minval: Option<f64>,
    above: Option<f64>,
) -> Result<f64, String> {
    let value = match parser.get(section, option) {
        Ok(Some(s)) => s,
        Ok(None) => {
            return match default {
                Some(d) => Ok(d),
                None => Err(format!(
                    "Option '{option}' in section '{section}' must be specified"
                )),
            }
        }
        Err(e) => return Err(e.to_string()),
    };
    let v = match pycompat::text::float_from_string(&value) {
        Ok(v) => v,
        Err(_) => {
            return Err(format!(
                "Unable to parse option '{option}' in section '{section}'"
            ))
        }
    };
    if let Some(minval) = minval {
        if v < minval {
            return Err(format!(
                "Option '{option}' in section '{section}' must have minimum of {}",
                float_repr(minval)
            ));
        }
    }
    if let Some(above) = above {
        if v <= above {
            return Err(format!(
                "Option '{option}' in section '{section}' must be above {}",
                float_repr(above)
            ));
        }
    }
    Ok(v)
}

/// `config.getfloat("beta", None, above=0.)` variant: a missing option
/// yields `None` instead of the default.
fn getfloat_opt(
    parser: &RawConfigParser,
    section: &str,
    option: &str,
    above: f64,
) -> Result<Option<f64>, String> {
    match parser.get(section, option) {
        Ok(Some(value)) => {
            let v = match pycompat::text::float_from_string(&value) {
                Ok(v) => v,
                Err(_) => {
                    return Err(format!(
                        "Unable to parse option '{option}' in section '{section}'"
                    ))
                }
            };
            if v <= above {
                return Err(format!(
                    "Option '{option}' in section '{section}' must be above {}",
                    float_repr(above)
                ));
            }
            Ok(Some(v))
        }
        Ok(None) => Ok(None),
        Err(e) => Err(e.to_string()),
    }
}

/// `CustomThermistor(config)` for section `section` (lines 86-103).
/// The name is the section name with the first whitespace-separated token
/// (`thermistor`) dropped and the rest re-joined with single spaces
/// (`" ".join(config.get_name().split()[1:])`).
pub fn custom_thermistor(parser: &RawConfigParser, section: &str) -> Result<CustomThermistor, String> {
    let name = {
        let parts: Vec<&str> = section.split_whitespace().collect();
        if parts.len() > 1 {
            parts[1..].join(" ")
        } else {
            String::new()
        }
    };
    let t1 = getfloat(parser, section, "temperature1", None, Some(KELVIN_TO_CELSIUS), None)?;
    let r1 = getfloat(parser, section, "resistance1", None, Some(0.0), None)?;
    if let Some(beta) = getfloat_opt(parser, section, "beta", 0.0)? {
        return Ok(CustomThermistor {
            name,
            params: ThermistorParams {
                t1,
                r1,
                t2: 0.0,
                r2: 0.0,
                t3: 0.0,
                r3: 0.0,
                beta: Some(beta),
            },
        });
    }
    let t2 = getfloat(parser, section, "temperature2", None, Some(KELVIN_TO_CELSIUS), None)?;
    let r2 = getfloat(parser, section, "resistance2", None, Some(0.0), None)?;
    let t3 = getfloat(parser, section, "temperature3", None, Some(KELVIN_TO_CELSIUS), None)?;
    let r3 = getfloat(parser, section, "resistance3", None, Some(0.0), None)?;
    // `(t1, r1), (t2, r2), (t3, r3) = sorted([(t1, r1), (t2, r2), (t3, r3)])`:
    // sort by temperature ascending; Python's tuple ordering compares the
    // resistance as a tie-break.
    let mut pts = [(t1, r1), (t2, r2), (t3, r3)];
    pts.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    });
    Ok(CustomThermistor {
        name,
        params: ThermistorParams {
            t1: pts[0].0,
            r1: pts[0].1,
            t2: pts[1].0,
            r2: pts[1].1,
            t3: pts[2].0,
            r3: pts[2].1,
            beta: None,
        },
    })
}

/// Result of `PrinterThermistor(config, params)` (lines 72-83): the
/// converter that `adc_temperature.PrinterADCtoTemperature` wraps, plus any
/// `logging.warning` lines emitted while setting the coefficients.
#[derive(Debug)]
pub struct PrinterThermistor {
    /// The `Thermistor` converter.
    pub thermistor: Thermistor,
    /// `logging.warning` lines from coefficient setup, in order.
    pub log_lines: Vec<String>,
}

/// `PrinterThermistor(config, params)`: reads `pullup_resistor` (default
/// 4700, above 0) and `inline_resistor` (default 0, minimum 0) from the
/// per-sensor section, then sets the coefficients from `params`.
/// Config-level errors are returned as the `configparser.Error` message
/// text; math errors keep their `PyError` form.
pub fn printer_thermistor(
    parser: &RawConfigParser,
    section: &str,
    params: &ThermistorParams,
) -> Result<PrinterThermistor, PyError> {
    let to_pyerr = |msg: String| -> PyError { PyError::new("Error", msg) };
    let pullup = getfloat(parser, section, "pullup_resistor", Some(4700.0), None, Some(0.0))
        .map_err(to_pyerr)?;
    let inline_resistor = getfloat(parser, section, "inline_resistor", Some(0.0), Some(0.0), None)
        .map_err(to_pyerr)?;
    let mut thermistor = Thermistor::new(pullup, inline_resistor);
    let log_lines = match params.beta {
        Some(beta) => {
            thermistor.setup_coefficients_beta(params.t1, params.r1, beta)?;
            Vec::new()
        }
        None => thermistor.setup_coefficients(
            params.t1,
            params.r1,
            params.t2,
            params.r2,
            params.t3,
            params.r3,
            section,
        )?,
    };
    Ok(PrinterThermistor {
        thermistor,
        log_lines,
    })
}
