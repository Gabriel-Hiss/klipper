//! Port of the math of `klippy/extras/thermistor.py`.
//!
//! Implements the `Thermistor` Steinhart-Hart / beta converter: `calc_temp`
//! (ADC to temperature) and `calc_adc` (temperature to ADC, used by
//! `setup_minmax` to compute ADC limits). The config side
//! (`PrinterThermistor`, `CustomThermistor`, `load_config_prefix`) is the
//! `thermistor` module of the `extras` crate, which plugs this converter
//! into `adc_temperature.PrinterADCtoTemperature`.
//!
//! Floating point follows the Python source line by line: transcendentals
//! call glibc libm through `pycompat::libm`, operation order matches the
//! source, and no fused multiply-add is used.

use pycompat::libm;
use pycompat::text::{float_pow, percent_format, PctValue, PyError};

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
