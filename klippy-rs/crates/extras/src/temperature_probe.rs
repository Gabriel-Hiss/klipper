//! Port of `klippy/extras/temperature_probe.py`.
//!
//! Parity notes (see final report for the full list):
//!
//! - `heaters` is consumed through the traits its own module owns
//!   ([`PrinterHeatersApi`], [`TemperatureSensorApi`], [`SensorApi`]) and
//!   obtained with `load_interface`; the temperature sensor chip and the
//!   registered sensor are two different duck types there.
//! - `manual_probe.ManualProbeHelper` and `verify_no_manual_probe` come
//!   from `crate::manual_probe`.
//! - `probe` is consumed through `crate::probe::PrinterProbeApi` (its session
//!   is `crate::probe::ProbeSessionApi`) and `probe_eddy_current` through
//!   `PrinterEddyProbeApi` / `DriftCompensationApi` from
//!   `crate::probe_eddy_current`, all obtained with `lookup_interface` /
//!   `load_interface`.
//! - Core services (`gcode`, `toolhead`, `configfile`) are obtained with
//!   `lookup_interface` / `load_interface` (contract v2).
//! - `int()` truncations and `round()` go through `pycompat`; all `%`
//!   formatting goes through `pycompat::text::percent_format`.
use crate::heaters::{PrinterHeatersApi, SensorApi, TemperatureSensorApi};
use crate::manual_probe::{ManualProbeHelper, ProbeResult, verify_no_manual_probe};
use crate::probe::PrinterProbeApi;
use crate::probe_eddy_current::{DriftCompensationApi, PrinterEddyProbeApi};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetFloatListOpts, GetFloatOpts, GetListsOpts, ListParser,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandApi, GetFloatOpts as CmdFloatOpts, GetIntOpts as CmdIntOpts,
    GetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::object::{
    load_interface, lookup_interface, require_interface, ObjectRef, PrinterObject,
};
use klippy_api::printer::{LookupDefault, Printer};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::config::{PrinterConfig, PrinterConfigApi};
use pycompat::int::PyInt;
use pycompat::text::{percent_format, round_py_nd, PctValue};
use pycompat::value::PyValue;
use std::sync::{Arc, Weak};

/// `KELVIN_TO_CELSIUS` (`temperature_probe.py:10`).
pub const KELVIN_TO_CELSIUS: f64 = -273.15;
/// `DRIFT_SAMPLE_COUNT` (`temperature_probe.py:477`).
pub const DRIFT_SAMPLE_COUNT: usize = 9;

/// `%`-format helper: `fmt % args` with the `pycompat` formatter.
fn pf(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("temperature_probe format string")
}

fn fval(v: f64) -> PctValue {
    PctValue::Float(v)
}
fn ival(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}
fn sval(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}
/// `name.split(None, 1)[-1]` (`temperature_probe.py:111,348,585`).
fn short_name_of(name: &str) -> &str {
    let trimmed = name.trim_start();
    match trimmed.find(|c: char| c.is_whitespace()) {
        Some(idx) => trimmed[idx..].trim_start(),
        None => trimmed,
    }
}
/// Numeric value of a Python int/float kept as a [`PyValue`] (fields the
/// reference stores as Python values whose int/float type is observable).
fn pyval_f64(v: &PyValue) -> f64 {
    match v {
        PyValue::Float(f) => *f,
        PyValue::Int(n) => n.0.to_string().parse().unwrap_or(0.0),
        _ => 0.0,
    }
}
/// Python `v += x` for an int/float `v` and a float `x`: always a float.
fn pyval_add_f64(v: &PyValue, x: f64) -> PyValue {
    PyValue::Float(pyval_f64(v) + x)
}
/// Python numeric truthiness (`if not self.enabled`).
fn pyval_truthy(v: &PyValue) -> bool {
    match v {
        PyValue::Bool(b) => *b,
        PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap_or(0) != 0,
        PyValue::Float(f) => *f != 0.0,
        PyValue::None => false,
        _ => true,
    }
}

/// Python `min(a, b)`: `a` unless `b < a` (differs from `f64::min` for NaN).
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// Python `max(a, b)`: `a` unless `b > a` (differs from `f64::max` for NaN).
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `sum(values)`: starts from the int `0`, so an all `-0.0` list sums
/// to `0.0` (Rust's float `Sum` starts from `-0.0`).
fn py_sum(values: &[f64]) -> f64 {
    values.iter().fold(0.0, |acc, &v| acc + v)
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

// ---------------------------------------------------------------------------
// Polynomial2d (temperature_probe.py:16-57); the least-squares fit uses
// core::mathutil solve_linear_equations (mathutil.py:216).
// ---------------------------------------------------------------------------

use klippy_core::mathutil::solve_linear_equations as core_solve_linear_equations;

// ---------------------------------------------------------------------------

/// `Polynomial2d` (`temperature_probe.py:16`).
#[derive(Clone, Debug)]
pub struct Polynomial2d {
    a: f64,
    b: f64,
    c: f64,
}

impl Polynomial2d {
    pub fn new(a: f64, b: f64, c: f64) -> Self {
        Polynomial2d { a, b, c }
    }

    /// `__call__`: `c*x*x + b*x + a` in that evaluation order.
    pub fn call(&self, xval: f64) -> f64 {
        self.c * xval * xval + self.b * xval + self.a
    }

    /// `get_coefs()` (`temperature_probe.py:25`).
    pub fn get_coefs(&self) -> (f64, f64, f64) {
        (self.a, self.b, self.c)
    }

    /// `__str__` (`temperature_probe.py:28`).
    pub fn py_str(&self) -> String {
        pf("%f, %f, %f", &[fval(self.a), fval(self.b), fval(self.c)])
    }

    /// `__repr__` (`temperature_probe.py:31`).
    ///
    /// The reference converts a coefficient to `int` when
    /// `round(coef, 8) == int(coef)` (lines 35-36); the converted
    /// value is what the `abs(coef) < 1e-10` skip test then sees,
    /// so e.g. `1e-9` becomes `0` and is skipped even though
    /// `abs(1e-9) >= 1e-10`.
    pub fn py_repr(&self) -> KResult<String> {
        let mut parts: Vec<String> = vec!["y(x) =".to_string()];
        let deg = 2i64;
        for (i, coef) in [self.c, self.b, self.a].iter().enumerate() {
            let mut coef = *coef;
            // `int(coef)` raises for NaN and infinity.
            if !coef.is_finite() {
                py_int_f64(coef)?;
            }
            if round_py_nd(coef, 8) == coef.trunc() {
                coef = coef.trunc();
            }
            if coef.abs() < 1e-10 {
                continue;
            }
            let cur_deg = deg - i as i64;
            let x_str = if cur_deg > 1 {
                pf("x^%d", &[ival(cur_deg)])
            } else {
                "x".repeat(if cur_deg < 0 { 0 } else { cur_deg as usize })
            };
            if parts.len() == 1 {
                parts.push(pf("%f%s", &[fval(coef), sval(&x_str)]));
            } else {
                let sym = if coef < 0.0 { "-" } else { "+" };
                parts.push(pf("%s %f%s", &[sval(sym), fval(coef.abs()), sval(&x_str)]));
            }
        }
        Ok(parts.join(" "))
    }

    /// `Polynomial2d.fit` (`temperature_probe.py:48`): least squares fit of
    /// `a + b*x + c*x*x = y`. A singular system makes
    /// `mathutil.solve_linear_equations` return `None`, and the reference
    /// then fails on `res[0][0]`.
    pub fn fit(coords: &[(f64, f64)]) -> KResult<Self> {
        let mut eqs: Vec<Vec<f64>> = Vec::with_capacity(coords.len());
        let mut ans: Vec<Vec<f64>> = Vec::with_capacity(coords.len());
        for &(x, y) in coords {
            eqs.push(vec![1.0, x, x * x]);
            ans.push(vec![y]);
        }
        let res = core_solve_linear_equations(&eqs, &ans, false)?.ok_or_else(|| {
            KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not subscriptable".to_string(),
            }
        })?;
        Ok(Polynomial2d::new(res[0][0], res[1][0], res[2][0]))
    }
}

/// One bulk sample `(ptime, freq, dummy_z)` (`ldc1612.py:191`).
type BulkSample = (f64, f64, f64);
/// Samples collected per z position during one `collect_sample` call.
type SampleBatches = Vec<Vec<BulkSample>>;

fn find_gcode(printer: &Printer) -> KResult<GCode> {
    Ok(lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
fn find_toolhead(printer: &Printer) -> KResult<ToolHead> {
    Ok(lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
fn find_printer_config(printer: &Printer) -> KResult<PrinterConfig> {
    Ok(lookup_interface::<dyn PrinterConfigApi>(&**printer, "configfile", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
/// `printer.lookup_object("probe")` (temperature_probe.py:260) with the
/// `probe` interface; the object is kept for `probe.get_status(None)`.
fn find_probe(printer: &Printer) -> KResult<(ObjectRef, Arc<dyn PrinterProbeApi>)> {
    let obj = printer
        .lookup_object("probe", LookupDefault::Raise)?
        .expect("Raise never yields None");
    let probe = require_interface::<dyn PrinterProbeApi>(&obj, "probe")?;
    Ok((obj, probe))
}
/// `printer.load_object(config, "heaters")` (temperature_probe.py:87).
fn find_heaters(printer: &Printer, config: &Config) -> KResult<Arc<dyn PrinterHeatersApi>> {
    Ok(load_interface::<dyn PrinterHeatersApi>(&**printer, config, "heaters", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
fn find_eddy_probe_by_name(
    printer: &Printer,
    sect_name: &str,
) -> KResult<Arc<dyn PrinterEddyProbeApi>> {
    Ok(lookup_interface::<dyn PrinterEddyProbeApi>(&**printer, sect_name, LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
/// `config.getlists` decode helper: `drift_calibration` rows of 3 floats.
fn decode_drift_calibration(dc: &PyValue) -> KResult<Option<Vec<[f64; 3]>>> {
    match dc {
        PyValue::None => Ok(None),
        PyValue::Tuple(rows) => {
            let mut out = Vec::with_capacity(rows.len());
            for row in rows {
                let items = match row {
                    PyValue::Tuple(items) => items,
                    _ => {
                        return Err(KlippyError::Config(
                            "Invalid polynomial in drift calibration".to_string(),
                        ))
                    }
                };
                if items.len() != 3 {
                    return Err(KlippyError::Config(
                        "Invalid polynomial in drift calibration".to_string(),
                    ));
                }
                let mut coefs = [0.0f64; 3];
                for (k, v) in items.iter().enumerate() {
                    coefs[k] = match v {
                        PyValue::Float(f) => *f,
                        _ => {
                            return Err(KlippyError::Config(
                                "Invalid polynomial in drift calibration".to_string(),
                            ))
                        }
                    };
                }
                out.push(coefs);
            }
            Ok(Some(out))
        }
        _ => Err(KlippyError::Config(
            "Invalid polynomial in drift calibration".to_string(),
        )),
    }
}
/// Truncate `x` with Python `int()` and convert to `i64`.
fn trunc_i64(x: f64) -> i64 {
    pycompat::text::trunc_float_to_bigint(x)
        .0
        .to_string()
        .parse::<i64>()
        .unwrap_or(i64::MAX)
}

/// Python `int(x)` for a float: non-finite values raise before truncating, and
/// the errors are not catchable by the config or command handlers.
fn py_int_f64(x: f64) -> KResult<i64> {
    if x.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if x.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    Ok(trunc_i64(x))
}

/// Extruder name for `toolhead.get_extruder().get_name()`
/// (temperature_probe.py:270).
fn extruder_name(toolhead: &ToolHead) -> KResult<String> {
    Ok(toolhead.get_extruder().get_name())
}

// ---------------------------------------------------------------------------
// EddyDriftCompensation (temperature_probe.py:479-711)
// ---------------------------------------------------------------------------

struct EddyState {
    calibration_samples: Option<Vec<Vec<(f64, f64)>>>,
    /// Python `self.enabled`: starts as the bool `has_dc`, becomes the int
    /// from `ENABLE` after `set_enabled`, and `False` (bool) after
    /// `start_calibration`.
    enabled: PyValue,
}

/// `EddyDriftCompensation` (`temperature_probe.py:479`).
pub struct EddyDriftCompensation {
    printer: Printer,
    temp_sensor: Weak<dyn SensorApi>,
    name: String,
    cal_temp: GCell<f64>,
    drift_calibration: Option<Vec<Polynomial2d>>,
    max_valid_temp: f64,
    min_freq: GCell<f64>,
    state: GCell<EddyState>,
}

impl EddyDriftCompensation {
    pub fn new(
        config: &Config,
        temp_sensor: Weak<dyn SensorApi>,
    ) -> KResult<Arc<EddyDriftCompensation>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let cal_temp = config
            .getfloat(
                "calibration_temp",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);
        let max_valid_temp = config
            .getfloat(
                "max_validation_temp",
                GetFloatOpts {
                    default: Some(Some(60.0)),
                    ..Default::default()
                },
            )?
            .unwrap_or(60.0);
        let dc_min_temp = config
            .getfloat(
                "drift_calibration_min_temp",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);
        let dc = config.getlists(
            "drift_calibration",
            GetListsOpts {
                default: Some(None),
                seps: vec![",".to_string(), "\n".to_string()],
                parser: ListParser::Float,
                ..Default::default()
            },
        )?;
        let (drift_calibration, min_freq) = match decode_drift_calibration(&dc)? {
            None => {
                log::info(&pf(
                    "%s: No drift calibration configured, disabling temperature drift compensation",
                    &[sval(&name)],
                ));
                (None, 999999999999.0)
            }
            Some(coefs_list) => {
                let cals: Vec<Polynomial2d> = coefs_list
                    .iter()
                    .map(|c| Polynomial2d::new(c[0], c[1], c[2]))
                    .collect();
                Self::check_calibration_static(
                    &cals,
                    dc_min_temp,
                    max_valid_temp,
                    &name,
                    true,
                )?;
                let low_poly = cals.last().ok_or_else(index_error)?.clone();
                let mut min_freq = low_poly.call(0.0);
                for temp in 1..121 {
                    min_freq = py_min(min_freq, low_poly.call(temp as f64));
                }
                let mut reprs = Vec::with_capacity(cals.len());
                for p in &cals {
                    reprs.push(p.py_repr()?);
                }
                let cal_str = reprs.join("\n");
                log::info(&pf(
                    "%s: loaded temperature drift calibration. Min Temp: %.2f, Min Freq: %.6f\n%s",
                    &[
                        sval(&name),
                        fval(dc_min_temp),
                        fval(min_freq),
                        sval(&cal_str),
                    ],
                ));
                (Some(cals), min_freq)
            }
        };
        let has_dc = drift_calibration.is_some();
        let obj = Arc::new(EddyDriftCompensation {
            printer,
            temp_sensor,
            name: name.clone(),
            cal_temp: GCell::new(cal_temp),
            drift_calibration,
            max_valid_temp,
            min_freq: GCell::new(min_freq),
            state: GCell::new(EddyState {
                calibration_samples: None,
                enabled: PyValue::Bool(has_dc),
            }),
        });
        if obj.cal_temp.get() < 1e-6 && has_dc {
            obj.state.borrow_mut().enabled = PyValue::Bool(false);
            log::info(&pf(
                "%s: No temperature saved for eddy probe calibration, disabling temperature drift compensation.",
                &[sval(&obj.name)],
            ));
        }
        Ok(obj)
    }

    /// `_check_calibration` (temperature_probe.py:657) with the
    /// `config.error` variant at config-load time (`is_config`).
    fn check_calibration_static(
        calibration: &[Polynomial2d],
        start_temp: f64,
        end_temp: f64,
        name: &str,
        is_config: bool,
    ) -> KResult<()> {
        let (start, end) = (py_int_f64(start_temp)?, py_int_f64(end_temp)? + 1);
        let err = |msg: String| -> KlippyError {
            if is_config {
                KlippyError::Config(msg)
            } else {
                KlippyError::Command(msg)
            }
        };
        for temp in start..end {
            let first = calibration.first().ok_or_else(index_error)?;
            let mut last_freq = first.call(temp as f64);
            for (i, poly) in calibration[1..].iter().enumerate() {
                let next_freq = poly.call(temp as f64);
                if next_freq >= last_freq {
                    return Err(err(pf(
                        "%s: invalid calibration detected, curve at index %d overlaps previous curve at temp %dC.",
                        &[sval(name), ival((i + 1) as i64), ival(temp)],
                    )));
                }
                last_freq = next_freq;
            }
        }
        Ok(())
    }

    /// `_check_calibration` with the default `printer.command_error`.
    pub fn check_calibration(
        &self,
        calibration: &[Polynomial2d],
        start_temp: f64,
        end_temp: f64,
    ) -> KResult<()> {
        Self::check_calibration_static(calibration, start_temp, end_temp, &self.name, false)
    }

    /// `is_enabled` (temperature_probe.py:526): returns the raw Python
    /// `self.enabled` (bool until `set_enabled` stores the int).
    pub fn is_enabled(&self) -> PyValue {
        self.state.borrow().enabled.clone()
    }

    fn enabled_flag(&self) -> bool {
        pyval_truthy(&self.state.borrow().enabled)
    }

    /// `set_enabled` (temperature_probe.py:529).
    pub fn set_enabled(&self, gcmd: &dyn GCodeCommandApi) -> KResult<()> {
        let enabled = gcmd
            .get_int("ENABLE", CmdIntOpts::default())?
            .expect("a required parameter is never None");
        if enabled != 0 {
            if self.drift_calibration.is_none() {
                return Err(KlippyError::Command(
                    "No drift calibration configured, cannot enable temperature drift compensation"
                        .to_string(),
                ));
            }
            if self.cal_temp.get() < 1e-6 {
                return Err(KlippyError::Command(
                    "Z Calibration temperature not configured, cannot enable temperature drift compensation"
                        .to_string(),
                ));
            }
        }
        self.state.borrow_mut().enabled = PyValue::Int(PyInt::from_i64(enabled));
        Ok(())
    }

    /// `collect_sample` (temperature_probe.py:559).
    pub fn collect_sample(
        &self,
        mpresult: &ProbeResult,
        tool_zero_z: f64,
        speeds: (f64, f64, f64),
    ) -> KResult<f64> {
        {
            let mut st = self.state.borrow_mut();
            if st.calibration_samples.is_none() {
                st.calibration_samples = Some((0..DRIFT_SAMPLE_COUNT).map(|_| Vec::new()).collect());
            }
        }
        let toolhead = find_toolhead(&self.printer)?;
        let mut cur_pos = toolhead.get_position();
        let (lift_speed, probe_speed, _move_speed) = speeds;

        // Scratch buffers shared with the bulk callback (FnMut closure).
        let move_times_cell: Arc<GCell<Vec<(usize, f64, f64)>>> =
            Arc::new(GCell::new(Vec::new()));
        let temps_cell: Arc<GCell<Vec<f64>>> = Arc::new(GCell::new(vec![0.0; DRIFT_SAMPLE_COUNT]));
        let samples_cell: Arc<GCell<SampleBatches>> =
            Arc::new(GCell::new((0..DRIFT_SAMPLE_COUNT).map(|_| Vec::new()).collect()));
        {
            let mt = Arc::clone(&move_times_cell);
            let t_cell = Arc::clone(&temps_cell);
            let s_cell = Arc::clone(&samples_cell);
            let weak = self.temp_sensor.clone();
            let cb = move |msg: PyValue| -> KResult<bool> {
                // `msg["data"]`: list of `(ptime, freq, dummy_z)` samples
                // (`ldc1612.py:191`, `bulk_sensor.py:71`).
                let data = match &msg {
                    PyValue::Dict(d) => d.iter().find_map(|(k, v)| match k {
                        PyValue::Str(key) if key == "data" => Some(v),
                        _ => None,
                    }),
                    _ => None,
                };
                let samples = match data {
                    Some(PyValue::List(l)) | Some(PyValue::Tuple(l)) => l.as_slice(),
                    _ => return Ok(true),
                };
                if mt.borrow().is_empty() {
                    return Ok(true);
                }
                let cur_temp = weak
                    .upgrade()
                    .map(|ts| ts.get_temp(0.0).0)
                    .unwrap_or(0.0);
                for sample in samples {
                    let s_list = match sample {
                        PyValue::List(l) | PyValue::Tuple(l) => l,
                        _ => continue,
                    };
                    if s_list.len() < 3 {
                        continue;
                    }
                    let ptime = sample_f64(&s_list[0]).unwrap_or(0.0);
                    {
                        let mut mt_b = mt.borrow_mut();
                        while !mt_b.is_empty() && ptime > mt_b[0].2 {
                            let (idx, _, _) = mt_b.remove(0);
                            if mt_b.is_empty() {
                                return Ok(idx >= DRIFT_SAMPLE_COUNT - 1);
                            }
                        }
                        if mt_b.is_empty() {
                            return Ok(true);
                        }
                        let (idx, start_time, _) = mt_b[0];
                        if ptime < start_time {
                            continue;
                        }
                        t_cell.borrow_mut()[idx] = cur_temp;
                        let freq = sample_f64(&s_list[1]).unwrap_or(0.0);
                        let z = sample_f64(&s_list[2]).unwrap_or(0.0);
                        s_cell.borrow_mut()[idx].push((ptime, freq, z));
                    }
                }
                Ok(true)
            };
            let sect_name = format!("probe_eddy_current {}", short_name_of(&self.name));
            find_eddy_probe_by_name(&self.printer, &sect_name)?.add_client(Box::new(cb))?;
        }

        for i in 0..DRIFT_SAMPLE_COUNT {
            if i == 0 {
                cur_pos[2] = tool_zero_z + 0.05;
            } else {
                cur_pos[2] += 1.0;
                toolhead.manual_move(vec_of_opt(&cur_pos), lift_speed)?;
                cur_pos[2] -= 0.5;
            }
            toolhead.manual_move(vec_of_opt(&cur_pos), probe_speed)?;
            let start = toolhead.get_last_move_time()? + 0.05;
            let end = start + 0.1;
            move_times_cell.borrow_mut().push((i, start, end));
            toolhead.dwell(0.2)?;
        }
        toolhead.wait_moves()?;
        // Wait for sample collection to finish (reactor.pause loop).
        let reactor = self.printer.get_reactor();
        let mut evttime = reactor.monotonic();
        while !move_times_cell.borrow().is_empty() {
            evttime = reactor.pause(evttime + 0.1);
        }
        let temps = temps_cell.cloned();
        let probe_samples = samples_cell.cloned();
        let sample_temp = py_sum(&temps) / temps.len() as f64;
        {
            let mut st = self.state.borrow_mut();
            let calib = st
                .calibration_samples
                .get_or_insert_with(|| (0..DRIFT_SAMPLE_COUNT).map(|_| Vec::new()).collect());
            for (i, data) in probe_samples.iter().enumerate() {
                if data.is_empty() {
                    return Err(KlippyError::Internal {
                        class_name: "ZeroDivisionError".to_string(),
                        message: "division by zero".to_string(),
                    });
                }
                let freqs: Vec<f64> = data.iter().map(|d| d.1).collect();
                let zvals: Vec<f64> = data.iter().map(|d| d.2).collect();
                let avg_freq = py_sum(&freqs) / freqs.len() as f64;
                let avg_z = py_sum(&zvals) / zvals.len() as f64;
                let kin_z = i as f64 * 0.5 + 0.05 + mpresult.bed_z;
                log::info(&pf(
                    "Probe Values at Temp %.2fC, Z %.4fmm: Avg Freq = %.6f, Avg Measured Z = %.6f",
                    &[fval(sample_temp), fval(kin_z), fval(avg_freq), fval(avg_z)],
                ));
                calib[i].push((sample_temp, avg_freq));
            }
        }
        Ok(sample_temp)
    }

    /// `start_calibration` (temperature_probe.py:622).
    pub fn start_calibration(&self) {
        let mut st = self.state.borrow_mut();
        st.enabled = PyValue::Bool(false);
        st.calibration_samples = Some((0..DRIFT_SAMPLE_COUNT).map(|_| Vec::new()).collect());
    }

    /// `finish_calibration` (temperature_probe.py:626).
    pub fn finish_calibration(&self, success: bool) -> KResult<()> {
        let cal_samples = self.state.borrow_mut().calibration_samples.take();
        if !success {
            return Ok(());
        }
        // `gcode` is looked up before `len(cal_samples)` raises on a missing
        // calibration-sample list.
        let gcode = find_gcode(&self.printer)?;
        let cal_samples = cal_samples.ok_or_else(|| KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "object of type 'NoneType' has no len()".to_string(),
        })?;
        if cal_samples.len() < 3 {
            return Err(KlippyError::Command(
                "calibration error, not enough samples".to_string(),
            ));
        }
        let (min_temp, _) = cal_samples
            .first()
            .and_then(|s| s.first())
            .copied()
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?;
        let (max_temp, _) = cal_samples
            .last()
            .and_then(|s| s.first())
            .copied()
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?;
        let mut polynomials: Vec<Polynomial2d> = Vec::with_capacity(cal_samples.len());
        for (i, coords) in cal_samples.iter().enumerate() {
            let height = 0.05 + i as f64 * 0.5;
            let poly = Polynomial2d::fit(coords)?;
            log::info(&pf(
                "Polynomial at Z=%.2f: %s",
                &[fval(height), sval(&poly.py_repr()?)],
            ));
            polynomials.push(poly);
        }
        let end_vld_temp = py_max(self.max_valid_temp, max_temp);
        self.check_calibration(&polynomials, min_temp, end_vld_temp)?;
        let coef_cfg = format!(
            "\n{}",
            polynomials.iter().map(|p| p.py_str()).collect::<Vec<_>>().join("\n")
        );
        let configfile = find_printer_config(&self.printer)?;
        configfile.set(&self.name, "drift_calibration", &coef_cfg);
        configfile.set(
            &self.name,
            "drift_calibration_min_temp",
            &pycompat::text::float_str(min_temp),
        );
        gcode.respond_info(
            &pf(
                "%s: generated %d 2D polynomials\nThe SAVE_CONFIG command will update the printer config file and restart the printer.",
                &[sval(&self.name), ival(polynomials.len() as i64)],
            ),
            true,
        );
        Ok(())
    }

    /// `_calc_freq` (temperature_probe.py:692). When the loop reaches a
    /// polynomial, `freq < high_freq` and `freq >= low_freq`, so
    /// `high_freq - low_freq` is never zero.
    fn calc_freq(&self, freq: f64, origin_temp: f64, dest_temp: f64) -> f64 {
        let dc = match &self.drift_calibration {
            Some(d) => d,
            None => return freq,
        };
        let mut high_freq: Option<f64>;
        let mut low_freq: Option<f64> = None;
        for (pos, poly) in dc.iter().enumerate() {
            high_freq = low_freq;
            let low = poly.call(origin_temp);
            low_freq = Some(low);
            if freq >= low {
                let Some(high) = high_freq else {
                    // Frequency above max calibration value
                    let err = poly.call(dest_temp) - low;
                    return freq + err;
                };
                let t = py_min(1.0, py_max(0.0, (freq - low) / (high - low)));
                let low_tgt_freq = poly.call(dest_temp);
                let high_tgt_freq = dc[pos - 1].call(dest_temp);
                return (1.0 - t) * low_tgt_freq + t * high_tgt_freq;
            }
        }
        // Frequency below minimum, no correction
        freq
    }

    /// `self.temp_sensor.get_temp()[0]` (temperature_probe.py:711).
    fn temperature(&self) -> f64 {
        self.temp_sensor
            .upgrade()
            .map(|ts| ts.get_temp(0.0).0)
            .unwrap_or(0.0)
    }
}

impl DriftCompensationApi for EddyDriftCompensation {
    /// `get_temperature` (temperature_probe.py:710).
    fn get_temperature(&self) -> KResult<f64> {
        Ok(self.temperature())
    }

    /// `note_z_calibration_start` (temperature_probe.py:544).
    fn note_z_calibration_start(&self) -> KResult<()> {
        let t = self.temperature();
        self.cal_temp.set(t);
        Ok(())
    }

    /// `note_z_calibration_finish` (temperature_probe.py:547).
    fn note_z_calibration_finish(&self) -> KResult<()> {
        let cal_temp = (self.cal_temp.get() + self.temperature()) / 2.0;
        self.cal_temp.set(cal_temp);
        let configfile = find_printer_config(&self.printer)?;
        configfile.set(&self.name, "calibration_temp", &pf("%.6f ", &[fval(cal_temp)]));
        let gcode = find_gcode(&self.printer)?;
        gcode.respond_info(
            &pf(
                "%s: Z Calibration Temperature set to %.2f. The SAVE_CONFIG command will update the printer config file and restart the printer.",
                &[sval(&self.name), fval(cal_temp)],
            ),
            true,
        );
        Ok(())
    }

    /// `adjust_freq` (temperature_probe.py:674): from the current temperature
    /// toward the calibration temperature.
    fn adjust_freq(&self, freq: f64, origin_temp: Option<f64>) -> KResult<f64> {
        if !self.enabled_flag() || freq < self.min_freq.get() {
            return Ok(freq);
        }
        let origin_temp = origin_temp.unwrap_or_else(|| self.temperature());
        Ok(self.calc_freq(freq, origin_temp, self.cal_temp.get()))
    }

    /// `unadjust_freq` (temperature_probe.py:683): given a frequency sampled at
    /// the calibration temperature, the offset frequency at `dest_temp`.
    fn unadjust_freq(&self, freq: f64, dest_temp: Option<f64>) -> KResult<f64> {
        if !self.enabled_flag() || freq < self.min_freq.get() {
            return Ok(freq);
        }
        let dest_temp = dest_temp.unwrap_or_else(|| self.temperature());
        Ok(self.calc_freq(freq, self.cal_temp.get(), dest_temp))
    }
}

fn sample_f64(v: &PyValue) -> Option<f64> {
    match v {
        PyValue::Float(f) => Some(*f),
        PyValue::Int(n) => n.0.to_string().parse().ok(),
        _ => None,
    }
}

fn vec_of_opt(pos: &[f64]) -> Vec<Option<f64>> {
    pos.iter().map(|&x| Some(x)).collect()
}

// ---------------------------------------------------------------------------
// TemperatureProbe (temperature_probe.py:59-468)
// ---------------------------------------------------------------------------

struct CalState {
    last_temp_read_time: f64,
    last_measurement: (f64, f64, f64), // (smoothed, min, max)
    method: String,
    next_auto_temp: f64,
    /// Python `self.target_temp`: int `0` until `cmd_CALIBRATE` stores the
    /// float from `get_float`, int `0` again after finalize.
    target_temp: PyValue,
    expected_count: i64,
    sample_count: i64,
    in_calibration: bool,
    step: f64,
    last_zero_pos: Option<f64>,
    /// Python `self.total_expansion`: int `0` until the first `+= z_diff`.
    total_expansion: PyValue,
    start_pos: Vec<f64>,
}

/// `TemperatureProbe` (`temperature_probe.py:59`).
pub struct TemperatureProbe {
    name: String,
    printer: Printer,
    gcode: GCode,
    speed: Option<f64>,
    horizontal_move_z: f64,
    resting_z: f64,
    cal_pos: Option<Vec<f64>>,
    cal_bed_temp: Option<f64>,
    cal_extruder_temp: Option<f64>,
    cal_extruder_z: f64,
    inv_smooth_time: f64,
    sensor: Arc<dyn TemperatureSensorApi>,
    cal_helper: GCell<Option<Arc<EddyDriftCompensation>>>,
    state: GCell<CalState>,
}

impl TemperatureProbe {
    /// `__init__` logic minus wiring that needs the final `Arc<Self>`
    /// (sensor callback, mux commands, drift helper). `load_config_prefix`
    /// finishes the wiring. Returns the sensor chip so the caller can
    /// install the callback on the same object.
    fn new_internal(
        config: &Config,
        printer: Printer,
        gcode: GCode,
    ) -> KResult<Self> {
        let name = config.get_name();
        let speed = config.getfloat(
            "speed",
            GetFloatOpts {
                default: Some(None),
                above: Some(0.0),
                ..Default::default()
            },
        )?;
        let horizontal_move_z = config
            .getfloat(
                "horizontal_move_z",
                GetFloatOpts {
                    default: Some(Some(2.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(2.0);
        let resting_z = config
            .getfloat(
                "resting_z",
                GetFloatOpts {
                    default: Some(Some(0.4)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.4);
        let cal_pos = config.getfloatlist(
            "calibration_position",
            GetFloatListOpts {
                default: Some(None),
                count: Some(3),
                ..Default::default()
            },
        )?;
        let cal_bed_temp = config.getfloat(
            "calibration_bed_temp",
            GetFloatOpts {
                default: Some(None),
                above: Some(50.0),
                ..Default::default()
            },
        )?;
        let cal_extruder_temp = config.getfloat(
            "calibration_extruder_temp",
            GetFloatOpts {
                default: Some(None),
                above: Some(50.0),
                ..Default::default()
            },
        )?;
        let cal_extruder_z = config
            .getfloat(
                "extruder_heating_z",
                GetFloatOpts {
                    default: Some(Some(50.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(50.0);
        let smooth_time = config
            .getfloat(
                "smooth_time",
                GetFloatOpts {
                    default: Some(Some(2.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(2.0);
        let inv_smooth_time = 1.0 / smooth_time;
        let min_temp = config
            .getfloat(
                "min_temp",
                GetFloatOpts {
                    default: Some(Some(KELVIN_TO_CELSIUS)),
                    minval: Some(KELVIN_TO_CELSIUS),
                    ..Default::default()
                },
            )?
            .unwrap_or(KELVIN_TO_CELSIUS);
        let max_temp = config
            .getfloat(
                "max_temp",
                GetFloatOpts {
                    default: Some(Some(99999999.9)),
                    above: Some(min_temp),
                    ..Default::default()
                },
            )?
            .unwrap_or(99999999.9);
        let pheaters = find_heaters(&printer, config)?;
        let sensor = pheaters.setup_sensor(config)?;
        sensor.setup_minmax(min_temp, max_temp)?;
        let probe = TemperatureProbe {
            name,
            printer,
            gcode,
            speed,
            horizontal_move_z,
            resting_z,
            cal_pos,
            cal_bed_temp,
            cal_extruder_temp,
            cal_extruder_z,
            inv_smooth_time,
            sensor: Arc::clone(&sensor),
            cal_helper: GCell::new(None),
            state: GCell::new(CalState {
                last_temp_read_time: 0.0,
                last_measurement: (0.0, 99999999.0, 0.0),
                method: "manual".to_string(),
                next_auto_temp: 99999999.0,
                target_temp: PyValue::Int(PyInt::from_i64(0)),
                expected_count: 0,
                sample_count: 0,
                in_calibration: false,
                step: 2.0,
                last_zero_pos: None,
                total_expansion: PyValue::Int(PyInt::from_i64(0)),
                start_pos: Vec::new(),
            }),
        };
        Ok(probe)
    }

    /// `load_config_prefix` (`temperature_probe.py:714`): full construction
    /// including the printer-dependent wiring.
    pub fn load_config_prefix(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode = find_gcode(&printer)?;
        let probe = Self::new_internal(config, printer.clone(), gcode)?;
        let probe_arc = Arc::new(probe);

        // Setup temperature sensor callback + registration.
        let sensor = Arc::clone(&probe_arc.sensor);
        {
            let arc_for_cb = Arc::clone(&probe_arc);
            sensor.setup_callback(Box::new(move |read_time: f64, temp: f64| {
                arc_for_cb.temp_callback(read_time, temp);
            }));
        }
        let pheaters = find_heaters(&printer, config)?;
        let registered: Arc<dyn SensorApi> = probe_arc.clone();
        pheaters.register_sensor(config, registered, None)?;

        // Register GCode Commands.
        let pname = short_name_of(&probe_arc.name).to_string();
        probe_arc.register_calibrate_mux(&pname)?;
        probe_arc.register_enable_mux(&pname)?;

        // Register Drift Compensation Helper with probe.
        let full_probe_name = pf("probe_eddy_current %s", &[sval(&pname)]);
        if config.has_section(&full_probe_name) {
            let pprobe = load_interface::<dyn PrinterEddyProbeApi>(
                &*printer, config, &full_probe_name, LookupDefault::Raise,
            )?
            .expect("Raise never yields None");
            let weak: Weak<dyn SensorApi> = {
                let as_trait: Arc<dyn SensorApi> = probe_arc.clone();
                Arc::downgrade(&as_trait)
            };
            let cal_helper = EddyDriftCompensation::new(config, weak)?;
            *probe_arc.cal_helper.borrow_mut() = Some(Arc::clone(&cal_helper));
            pprobe.register_drift_compensation(cal_helper);
            log::info(&pf(
                "%s: registered drift compensation with probe [%s]",
                &[sval(&probe_arc.name), sval(&full_probe_name)],
            ));
        } else {
            log::info(&pf(
                "%s: No probe named %s configured, thermal drift compensation disabled.",
                &[sval(&probe_arc.name), sval(&pname)],
            ));
        }
        Ok(probe_arc)
    }

    fn register_calibrate_mux(self: &Arc<Self>, pname: &str) -> KResult<()> {
        let this = Arc::clone(self);
        self.gcode.register_mux_command(
            "TEMPERATURE_PROBE_CALIBRATE",
            "PROBE",
            Some(pname),
            Box::new(move |gcmd: &GCmd| this.cmd_calibrate(gcmd)),
            Some("Calibrate probe temperature drift compensation"),
        )
    }

    fn register_enable_mux(self: &Arc<Self>, pname: &str) -> KResult<()> {
        let this = Arc::clone(self);
        self.gcode.register_mux_command(
            "TEMPERATURE_PROBE_ENABLE",
            "PROBE",
            Some(pname),
            Box::new(move |gcmd: &GCmd| this.cmd_enable(gcmd)),
            Some("Set adjustment factor applied to drift correction"),
        )
    }

    // -- internals ----------------------------------------------------------

    /// `_temp_callback` (temperature_probe.py:140).
    pub fn temp_callback(self: &Arc<Self>, read_time: f64, temp: f64) {
        let should_kick = {
            let mut st = self.state.borrow_mut();
            let (smoothed_temp, measured_min, measured_max) = st.last_measurement;
            let time_diff = read_time - st.last_temp_read_time;
            st.last_temp_read_time = read_time;
            let temp_diff = temp - smoothed_temp;
            let adj_time = py_min(time_diff * self.inv_smooth_time, 1.0);
            let smoothed_temp = smoothed_temp + temp_diff * adj_time;
            let measured_min = py_min(measured_min, smoothed_temp);
            let measured_max = py_max(measured_max, smoothed_temp);
            st.last_measurement = (smoothed_temp, measured_min, measured_max);
            st.in_calibration && smoothed_temp >= st.next_auto_temp
        };
        if should_kick {
            let this = Arc::clone(self);
            self.printer.get_reactor().register_async_callback(
                move |_eventtime: f64| {
                    this.check_kick_next()
                        .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message()));
                },
                0.0,
            );
        }
    }

    /// `_check_kick_next` (temperature_probe.py:155).
    pub fn check_kick_next(&self) -> KResult<()> {
        let do_run = {
            let mut st = self.state.borrow_mut();
            let smoothed_temp = st.last_measurement.0;
            if st.in_calibration && smoothed_temp >= st.next_auto_temp {
                st.next_auto_temp = 99999999.0;
                true
            } else {
                false
            }
        };
        if do_run {
            self.gcode.run_script("TEMPERATURE_PROBE_NEXT")?;
        }
        Ok(())
    }

    /// `get_temp` (temperature_probe.py:161).
    pub fn get_temp(&self) -> (f64, PyValue) {
        let st = self.state.borrow();
        (st.last_measurement.0, st.target_temp.clone())
    }

    /// `_get_probe` (temperature_probe.py:259): `lookup_object("probe")`
    /// raises `config_error` when missing (the `None` check in the
    /// reference is unreachable).
    pub fn get_probe(&self) -> KResult<(ObjectRef, Arc<dyn PrinterProbeApi>)> {
        find_probe(&self.printer)
    }

    fn get_speeds(&self) -> KResult<(f64, f64, f64)> {
        let (_, probe) = self.get_probe()?;
        let pparams = probe.get_probe_params(None)?;
        let (probe_speed, lift_speed) = (pparams.probe_speed, pparams.lift_speed);
        let move_speed = self.speed.unwrap_or_else(|| py_max(probe_speed, lift_speed));
        Ok((lift_speed, probe_speed, move_speed))
    }

    /// `_collect_sample` (temperature_probe.py:164).
    pub fn collect_sample(
        self: &Arc<Self>,
        mpresult: &ProbeResult,
        tool_zero_z: f64,
    ) -> KResult<f64> {
        let (_, probe) = self.get_probe()?;
        let (x_offset, y_offset, _) = probe.get_offsets(None)?;
        let speeds = self.get_speeds()?;
        let (lift_speed, _probe_speed, move_speed) = speeds;
        let toolhead = find_toolhead(&self.printer)?;
        let mut cur_pos = toolhead.get_position();
        cur_pos[2] += self.horizontal_move_z;
        toolhead.manual_move(vec_of_opt(&cur_pos), lift_speed)?;
        cur_pos[0] -= x_offset;
        cur_pos[1] -= y_offset;
        toolhead.manual_move(vec_of_opt(&cur_pos), move_speed)?;
        let cal_helper = self.cal_helper.borrow().clone().ok_or_else(|| {
            KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'collect_sample'".to_string(),
            }
        })?;
        cal_helper.collect_sample(mpresult, tool_zero_z, speeds)
    }

    /// `_prepare_next_sample` (temperature_probe.py:179).
    pub fn prepare_next_sample(self: &Arc<Self>, last_temp: f64, tool_zero_z: f64) -> KResult<()> {
        let this = Arc::clone(self);
        self.gcode.register_command(
            "ABORT",
            Some(Box::new(move |gcmd: &GCmd| this.cmd_abort(gcmd))),
            RegisterCommandOpts {
                desc: Some("Abort Probe Drift Calibration".to_string()),
                ..Default::default()
            },
        )?;
        let probe_speed = self.get_speeds()?.1;
        let toolhead = find_toolhead(&self.printer)?;
        let mut cur_pos = toolhead.get_position();
        cur_pos[2] = tool_zero_z + self.resting_z;
        toolhead.manual_move(vec_of_opt(&cur_pos), probe_speed)?;
        let (cnt, exp_cnt) = {
            let st = self.state.borrow();
            (st.sample_count, st.expected_count)
        };
        let next_auto_temp = last_temp + self.state.borrow().step;
        self.state.borrow_mut().next_auto_temp = next_auto_temp;
        self.gcode.respond_info(
            &pf(
                "%s: collected sample %d/%d at temp %.2fC, next sample scheduled at temp %.2fC",
                &[
                    sval(&self.name),
                    ival(cnt),
                    ival(exp_cnt),
                    fval(last_temp),
                    fval(next_auto_temp),
                ],
            ),
            true,
        );
        Ok(())
    }

    /// `_manual_probe_finalize` (temperature_probe.py:200).
    pub fn manual_probe_finalize(
        self: &Arc<Self>,
        mpresult: Option<ProbeResult>,
    ) -> KResult<()> {
        let Some(mpresult) = mpresult else {
            self.finalize_drift_cal(false, None)?;
            return Ok(());
        };
        let last_zero_pos = self.state.borrow().last_zero_pos;
        if let Some(last_zero) = last_zero_pos {
            let z_diff = last_zero - mpresult.bed_z;
            let total = {
                let mut st = self.state.borrow_mut();
                st.total_expansion = pyval_add_f64(&st.total_expansion, z_diff);
                pyval_f64(&st.total_expansion)
            };
            log::info(&pf(
                "Estimated Total Thermal Expansion: %.6f",
                &[fval(total)],
            ));
        }
        self.state.borrow_mut().last_zero_pos = Some(mpresult.bed_z);
        let toolhead = find_toolhead(&self.printer)?;
        let tool_zero_z = toolhead.get_position()[2];
        let last_temp = match self.collect_sample(&mpresult, tool_zero_z) {
            Ok(v) => v,
            Err(e) => {
                self.finalize_drift_cal(false, None)?;
                return Err(e);
            }
        };
        self.state.borrow_mut().sample_count += 1;
        if last_temp >= pyval_f64(&self.state.borrow().target_temp) {
            self.finalize_drift_cal(true, None)?;
        } else {
            match self.prepare_next_sample(last_temp, tool_zero_z) {
                Ok(()) => {
                    if self.state.borrow().sample_count == 1 {
                        self.set_bed_temp(self.cal_bed_temp)?;
                    }
                }
                Err(e) => {
                    self.finalize_drift_cal(false, None)?;
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// `_finalize_drift_cal` (temperature_probe.py:233).
    pub fn finalize_drift_cal(&self, success: bool, msg: Option<String>) -> KResult<()> {
        {
            let mut st = self.state.borrow_mut();
            st.next_auto_temp = 99999999.0;
            st.target_temp = PyValue::Int(PyInt::from_i64(0));
            st.expected_count = 0;
            st.sample_count = 0;
            st.step = 2.0;
            st.in_calibration = false;
            st.last_zero_pos = None;
            st.total_expansion = PyValue::Int(PyInt::from_i64(0));
            st.start_pos = Vec::new();
        }
        self.gcode.register_command("ABORT", None, Default::default())?;
        self.gcode.register_command("TEMPERATURE_PROBE_NEXT", None, Default::default())?;
        self.gcode.register_command("TEMPERATURE_PROBE_COMPLETE", None, Default::default())?;
        self.set_extruder_temp(0.0, false)?;
        self.set_bed_temp(Some(0.0))?;
        let mut success = success;
        let mut msg = msg;
        let helper = self.cal_helper.borrow().clone();
        if let Some(helper) = helper {
            if let Err(e) = helper.finish_calibration(success) {
                if matches!(e, KlippyError::Command(_)) {
                    success = false;
                    msg = Some(e.message().to_string());
                } else {
                    return Err(e);
                }
            }
        }
        if !success {
            let msg =
                msg.unwrap_or_else(|| pf("%s: calibration aborted", &[sval(&self.name)]));
            self.gcode.respond_info(&msg, true);
        }
        Ok(())
    }

    fn set_extruder_temp(&self, temp: f64, wait: bool) -> KResult<()> {
        if self.cal_extruder_temp.is_none() {
            return Ok(());
        }
        let toolhead = find_toolhead(&self.printer)?;
        let extr_name = extruder_name(&toolhead)?;
        self.gcode.run_script_from_command(&pf(
            "SET_HEATER_TEMPERATURE HEATER=%s TARGET=%f",
            &[sval(&extr_name), fval(temp)],
        ))?;
        if wait {
            self.gcode.run_script_from_command(&pf(
                "TEMPERATURE_WAIT SENSOR=%s MINIMUM=%f",
                &[sval(&extr_name), fval(temp)],
            ))?;
        }
        Ok(())
    }

    fn set_bed_temp(&self, temp: Option<f64>) -> KResult<()> {
        if self.cal_bed_temp.is_none() {
            return Ok(());
        }
        let Some(temp) = temp else { return Ok(()) };
        self.gcode.run_script_from_command(&pf(
            "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=%f",
            &[fval(temp)],
        ))
    }

    /// `_check_homed` (temperature_probe.py:289).
    pub fn check_homed(&self) -> KResult<()> {
        let toolhead = find_toolhead(&self.printer)?;
        let reactor = self.printer.get_reactor();
        let status = toolhead.get_status(reactor.monotonic());
        let homed = dict_str(&status, "homed_axes").ok_or_else(|| KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'homed_axes'".to_string(),
        })?;
        for axis in ['x', 'y', 'z'] {
            if !homed.contains(axis) {
                return Err(KlippyError::Command(
                    "Printer must be homed before calibration".to_string(),
                ));
            }
        }
        Ok(())
    }

    fn move_to_start(&self) -> KResult<()> {
        let toolhead = find_toolhead(&self.printer)?;
        let mut cur_pos = toolhead.get_position();
        let move_speed = self.get_speeds()?.2;
        if let Some(cal_pos) = self.cal_pos.clone() {
            if self.cal_extruder_temp.is_some() {
                cur_pos[2] = self.cal_extruder_z;
                toolhead.manual_move(vec_of_opt(&cur_pos), move_speed)?;
            }
            toolhead.manual_move(vec_of_opt(&cal_pos[..2]), move_speed)?;
            if let Some(t) = self.cal_extruder_temp {
                self.set_extruder_temp(t, true)?;
            }
            toolhead.manual_move(vec_of_opt(&cal_pos), move_speed)?;
        } else if let Some(t) = self.cal_extruder_temp {
            cur_pos[2] = self.cal_extruder_z;
            toolhead.manual_move(vec_of_opt(&cur_pos), move_speed)?;
            self.set_extruder_temp(t, true)?;
        }
        Ok(())
    }

    /// `_auto_probe` (temperature_probe.py:327).
    pub fn auto_probe(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let mut fo_params: IndexMap<String, String> = gcmd.get_command_parameters();
        let method = self.state.borrow().method.clone();
        fo_params.insert("METHOD".to_string(), method);
        let gcode = find_gcode(&self.printer)?;
        let fo_gcmd = gcode.create_gcode_command("PROBE", "PROBE", &fo_params);
        let (_, pprobe) = self.get_probe()?;
        let probe_session = pprobe.start_probe_session(&fo_gcmd)?;
        probe_session.run_probe(&fo_gcmd)?;
        let pos = probe_session
            .pull_probed_results()?
            .into_iter()
            .next()
            .ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })?;
        probe_session.end_probe_session()?;
        self.manual_probe_finalize(Some(pos))
    }

    /// `cmd_TEMPERATURE_PROBE_CALIBRATE` (temperature_probe.py:338).
    pub fn cmd_calibrate(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let method_raw = gcmd.get(
            "METHOD",
            GetOpts {
                default: Some(PyValue::Str("manual".to_string())),
                ..Default::default()
            },
        )?;
        let method = match method_raw {
            PyValue::Str(s) => s.to_lowercase(),
            other => other.str().to_lowercase(),
        };
        self.state.borrow_mut().method = method;
        if self.cal_helper.borrow().is_none() {
            return Err(KlippyError::Command(pf(
                "No calibration helper registered for [%s]",
                &[sval(&self.name)],
            )));
        }
        self.check_homed()?;
        let (probe, _) = self.get_probe()?;
        let probe_name = probe_status_name(&probe)?;
        let short_name = short_name_of(&probe_name).to_string();
        let my_short = short_name_of(&self.name).to_string();
        if short_name != my_short {
            return Err(KlippyError::Command(pf(
                "[%s] not linked to registered probe [%s].",
                &[sval(&self.name), sval(&probe_name)],
            )));
        }
        verify_no_manual_probe(&self.printer)?;
        if self.state.borrow().in_calibration {
            return Err(KlippyError::Command(
                "Already in probe drift calibration. Use TEMPERATURE_PROBE_COMPLETE or ABORT to exit."
                    .to_string(),
            ));
        }
        let cur_temp = self.state.borrow().last_measurement.0;
        let target_temp = gcmd
            .get_float(
                "TARGET",
                CmdFloatOpts {
                    above: Some(cur_temp),
                    ..Default::default()
                },
            )?
            .expect("a required parameter is never None");
        let step = gcmd
            .get_float(
                "STEP",
                CmdFloatOpts {
                    default: Some(Some(2.0)),
                    minval: Some(1.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(2.0);
        let expected_count = py_int_f64((target_temp - cur_temp) / step + 0.5)?;
        if expected_count < 3 {
            return Err(KlippyError::Command(pf(
                "Invalid STEP and/or TARGET parameters resulted in too few expected samples: %d",
                &[ival(expected_count)],
            )));
        }
        let this_next = Arc::clone(self);
        let reg_next = self.gcode.register_command(
            "TEMPERATURE_PROBE_NEXT",
            Some(Box::new(move |gcmd: &GCmd| this_next.cmd_next(gcmd))),
            RegisterCommandOpts {
                desc: Some("Sample next probe drift temperature".to_string()),
                ..Default::default()
            },
        );
        if let Err(e) = reg_next {
            return Err(match e {
                KlippyError::Config(_) => KlippyError::Command(
                    "Auxiliary Probe Drift Commands already registered. Use TEMPERATURE_PROBE_COMPLETE or ABORT to exit."
                        .to_string(),
                ),
                other => other,
            });
        }
        let this_complete = Arc::clone(self);
        let reg_complete = self.gcode.register_command(
            "TEMPERATURE_PROBE_COMPLETE",
            Some(Box::new(move |gcmd: &GCmd| this_complete.cmd_complete(gcmd))),
            RegisterCommandOpts {
                desc: Some("Sample next probe drift temperature".to_string()),
                ..Default::default()
            },
        );
        if let Err(e) = reg_complete {
            return Err(match e {
                KlippyError::Config(_) => KlippyError::Command(
                    "Auxiliary Probe Drift Commands already registered. Use TEMPERATURE_PROBE_COMPLETE or ABORT to exit."
                        .to_string(),
                ),
                other => other,
            });
        }
        {
            let mut st = self.state.borrow_mut();
            st.in_calibration = true;
        }
        if let Some(helper) = self.cal_helper.borrow().clone() {
            helper.start_calibration();
        }
        {
            let mut st = self.state.borrow_mut();
            st.target_temp = PyValue::Float(target_temp);
            st.step = step;
            st.sample_count = 0;
            st.expected_count = expected_count;
        }
        if let Err(e) = self.move_to_start() {
            if matches!(e, KlippyError::Command(_)) {
                self.finalize_drift_cal(false, Some("Error during initial move".to_string()))?;
            }
            return Err(e);
        }
        let toolhead = find_toolhead(&self.printer)?;
        let start_pos_full = toolhead.get_position();
        self.state.borrow_mut().start_pos = start_pos_full[..2].to_vec();
        if self.state.borrow().method == "tap" {
            return self.auto_probe(gcmd);
        }
        let this = Arc::clone(self);
        let cb: crate::manual_probe::FinalizeCallback =
            Box::new(move |res| this.manual_probe_finalize(res));
        ManualProbeHelper::new(&self.printer, gcmd, cb)?;
        Ok(())
    }

    /// `cmd_TEMPERATURE_PROBE_NEXT` (temperature_probe.py:410).
    pub fn cmd_next(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        verify_no_manual_probe(&self.printer)?;
        self.state.borrow_mut().next_auto_temp = 99999999.0;
        let toolhead = find_toolhead(&self.printer)?;
        let mut curpos = toolhead.get_position();
        let start_z = curpos[2];
        let (lift_speed, probe_speed, move_speed) = self.get_speeds()?;
        curpos[2] += self.horizontal_move_z;
        toolhead.manual_move(vec_of_opt(&curpos), lift_speed)?;
        let start_pos = self.state.borrow().start_pos.clone();
        if start_pos.len() < 2 {
            return Err(index_error());
        }
        curpos[0] = start_pos[0];
        curpos[1] = start_pos[1];
        toolhead.manual_move(vec_of_opt(&curpos), move_speed)?;
        curpos[2] = start_z;
        toolhead.manual_move(vec_of_opt(&curpos), probe_speed)?;
        self.gcode.register_command("ABORT", None, Default::default())?;
        if self.state.borrow().method == "tap" {
            return self.auto_probe(gcmd);
        }
        let this = Arc::clone(self);
        let cb: crate::manual_probe::FinalizeCallback =
            Box::new(move |res| this.manual_probe_finalize(res));
        ManualProbeHelper::new(&self.printer, gcmd, cb)?;
        Ok(())
    }

    /// `cmd_TEMPERATURE_PROBE_COMPLETE` (temperature_probe.py:435).
    pub fn cmd_complete(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let _ = gcmd;
        verify_no_manual_probe(&self.printer)?;
        let ok = self.state.borrow().sample_count >= 3;
        self.finalize_drift_cal(ok, None)?;
        Ok(())
    }

    /// `cmd_TEMPERATURE_PROBE_ABORT` (temperature_probe.py:440).
    pub fn cmd_abort(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let _ = gcmd;
        self.finalize_drift_cal(false, None)?;
        Ok(())
    }

    /// `cmd_TEMPERATURE_PROBE_ENABLE` (temperature_probe.py:446).
    pub fn cmd_enable(&self, gcmd: &GCmd) -> KResult<()> {
        if let Some(helper) = self.cal_helper.borrow().clone() {
            helper.set_enabled(&**gcmd)?;
        }
        Ok(())
    }

    /// `is_in_calibration` (temperature_probe.py:450).
    pub fn is_in_calibration(&self) -> bool {
        self.state.borrow().in_calibration
    }

    /// `get_status` (temperature_probe.py:453).
    pub fn status_dict(&self) -> PyValue {
        let st = self.state.borrow();
        let (smoothed_temp, measured_min, measured_max) = st.last_measurement;
        let dcomp_enabled = self
            .cal_helper
            .borrow()
            .as_ref()
            .map(|h| h.is_enabled())
            .unwrap_or(PyValue::Bool(false));
        let mut dict = IndexMap::new();
        dict.insert(
            PyValue::Str("temperature".to_string()),
            PyValue::Float(smoothed_temp),
        );
        dict.insert(
            PyValue::Str("measured_min_temp".to_string()),
            PyValue::Float(round_py_nd(measured_min, 2)),
        );
        dict.insert(
            PyValue::Str("measured_max_temp".to_string()),
            PyValue::Float(round_py_nd(measured_max, 2)),
        );
        dict.insert(
            PyValue::Str("in_calibration".to_string()),
            PyValue::Bool(st.in_calibration),
        );
        dict.insert(
            PyValue::Str("estimated_expansion".to_string()),
            st.total_expansion.clone(),
        );
        dict.insert(
            PyValue::Str("compensation_enabled".to_string()),
            dcomp_enabled,
        );
        PyValue::Dict(dict)
    }

    /// `stats` (temperature_probe.py:467).
    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        (
            false,
            pf("%s: temp=%.1f", &[
                sval(&self.name),
                fval(self.state.borrow().last_measurement.0),
            ]),
        )
    }

    pub fn cal_helper(&self) -> Option<Arc<EddyDriftCompensation>> {
        self.cal_helper.borrow().clone()
    }
}

impl SensorApi for TemperatureProbe {
    /// `get_temp(eventtime=None)` (temperature_probe.py:161).
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) {
        let (current, target) = TemperatureProbe::get_temp(self);
        (current, pyval_f64(&target))
    }
}

/// `probe.get_status(None)["name"]` (temperature_probe.py:347).
fn probe_status_name(probe: &ObjectRef) -> KResult<String> {
    let status = probe.get_status(0.0).ok_or_else(|| KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: "'probe' object has no attribute 'get_status'".to_string(),
    })?;
    dict_str(&status, "name").ok_or_else(|| KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: "'name'".to_string(),
    })
}

impl PrinterObject for TemperatureProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn SensorApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status_dict())
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(TemperatureProbe::stats(self, eventtime))
    }
}

fn dict_str(v: &PyValue, key: &str) -> Option<String> {
    match v {
        PyValue::Dict(d) => {
            for (k, val) in d.iter() {
                if let PyValue::Str(ks) = k {
                    if ks == key {
                        if let PyValue::Str(s) = val {
                            return Some(s.clone());
                        }
                    }
                }
            }
            None
        }
        _ => None,
    }
}

/// `load_config_prefix` (`temperature_probe.py:714`).
pub fn load_config_prefix(config: &Config) -> KResult<Arc<TemperatureProbe>> {
    TemperatureProbe::load_config_prefix(config)
}

/// Module factory (`load_config_prefix` entry point).
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory {
        load_config_prefix: Some(|config: &Config| {
            TemperatureProbe::load_config_prefix(config)
                .map(|p| Some(p as Arc<dyn PrinterObject>))
        }),
        ..Default::default()
    }
}
