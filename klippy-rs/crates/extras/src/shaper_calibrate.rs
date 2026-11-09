//! Port of the `shaper_calibrate` module (`klippy/extras/shaper_calibrate.py`).
//!
//! Pure numeric calibration of input shapers from accelerometer PSD data,
//! plus persistence (`save_params` / `save_calibration_data`). There are no
//! G-code commands, config options or status fields (see
//! `parity/inventory/units.json`).
//!
//! NumPy operations go through [`pycompat::numpy`], which reproduces the
//! NumPy 1.16.4 kernels (pairwise/buffered sums, `kaiser`, `fftpack_lite`
//! `rfft`, `rfftfreq`, `arange`, `interp`, `maximum`) bit for bit; the
//! element-wise math is the glibc `libm` NumPy calls. Python scalar
//! operations (`**`, builtin `sum`/`min`/`max`) keep their CPython form
//! (`pow`, left-to-right sums, "replace only when strictly greater/less").

use std::fs::OpenOptions;
use std::io::Write;

use klippy_api::config::PrinterConfigApi;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::GCodeApi;
use klippy_api::printer::Printer;

use pycompat::libm as lm;
use pycompat::numpy::{self, Complex64, RfftPlan};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue, PyError};

use crate::shaper_defs::{
    get_shaper_cfg, init_shaper, DEFAULT_DAMPING_RATIO, SHAPER_VIBRATION_REDUCTION,
};

// ---------------------------------------------------------------------------
// Header constants (`shaper_calibrate.py:9-16`, `shaper_defs.py:9-10`).
// ---------------------------------------------------------------------------

/// `MIN_FREQ = 5.`
pub const MIN_FREQ: f64 = 5.0;
/// `MAX_FREQ = 200.`
pub const MAX_FREQ: f64 = 200.0;
/// `WINDOW_T_SEC = 0.5`
pub const WINDOW_T_SEC: f64 = 0.5;
/// `MAX_SHAPER_FREQ = 150.`
pub const MAX_SHAPER_FREQ: f64 = 150.0;
/// `TEST_DAMPING_RATIOS=[0.075, 0.1, 0.15]`
pub const TEST_DAMPING_RATIOS: [f64; 3] = [0.075, 0.1, 0.15];
/// `AUTOTUNE_SHAPERS = ['zv', 'mzv', 'ei', '2hump_ei', '3hump_ei']`
pub const AUTOTUNE_SHAPERS: [&str; 5] = ["zv", "mzv", "ei", "2hump_ei", "3hump_ei"];
/// An input shaper: `(A, T)` — amplitudes and delays.
pub type Shaper = (Vec<f64>, Vec<f64>);

fn type_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Other { class_name: "TypeError".to_string(), message: msg.into() }
}

fn value_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Other { class_name: "ValueError".to_string(), message: msg.into() }
}

// ---------------------------------------------------------------------------
// `CalibrationData` / `CalibrationResult`.
// ---------------------------------------------------------------------------

/// One accelerometer sample: `(time, x, y, z)`.
pub type Sample = [f64; 4];

/// `CalibrationData` (`shaper_calibrate.py:22-57`).
#[derive(Debug, Clone)]
pub struct CalibrationData {
    /// Dataset name.
    pub name: String,
    /// Frequency bins, one per PSD point.
    pub freq_bins: Vec<f64>,
    /// Sum of the X/Y/Z periodograms.
    pub psd_sum: Vec<f64>,
    /// X periodogram.
    pub psd_x: Vec<f64>,
    /// Y periodogram.
    pub psd_y: Vec<f64>,
    /// Z periodogram.
    pub psd_z: Vec<f64>,
    /// Whether `normalize_to_frequencies` has run.
    pub normalized: bool,
    /// Extra datasets (normalized together with this one).
    pub data_sets: Vec<CalibrationData>,
}

impl CalibrationData {
    /// `CalibrationData(name, freq_bins, psd_sum, psd_x, psd_y, psd_z)`
    pub fn new(
        name: String,
        freq_bins: Vec<f64>,
        psd_sum: Vec<f64>,
        psd_x: Vec<f64>,
        psd_y: Vec<f64>,
        psd_z: Vec<f64>,
    ) -> Self {
        CalibrationData {
            name,
            freq_bins,
            psd_sum,
            psd_x,
            psd_y,
            psd_z,
            normalized: false,
            data_sets: Vec::new(),
        }
    }

    /// `add_data`
    pub fn add_data(&mut self, mut other: CalibrationData) {
        // `self.data_sets.extend(other.get_datasets())`.
        let mut children = std::mem::take(&mut other.data_sets);
        self.data_sets.push(other);
        self.data_sets.append(&mut children);
    }

    /// `set_numpy` (no-op in the port: NumPy functionality is native).
    pub fn set_numpy(&mut self) {}

    /// `normalize_to_frequencies`
    pub fn normalize_to_frequencies(&mut self) {
        if !self.normalized {
            for psd_idx in 0..4 {
                let psd_len = match psd_idx { 0 => self.psd_sum.len(), 1 => self.psd_x.len(), 2 => self.psd_y.len(), _ => self.psd_z.len() };
                if psd_len == 0 { continue; }
                for i in 0..psd_len {
                    let denom = self.freq_bins[i] + 0.1;
                    match psd_idx { 0 => self.psd_sum[i] /= denom, 1 => self.psd_x[i] /= denom, 2 => self.psd_y[i] /= denom, _ => self.psd_z[i] /= denom }
                }
                for i in 0..psd_len {
                    let f = self.freq_bins[i];
                    if f < 2.0 * MIN_FREQ {
                        let x = 2.0 * MIN_FREQ / (f + 0.1);
                        let factor = lm::exp(-(x * x) + 1.0);
                        match psd_idx { 0 => self.psd_sum[i] *= factor, 1 => self.psd_x[i] *= factor, 2 => self.psd_y[i] *= factor, _ => self.psd_z[i] *= factor }
                    }
                }
            }
            self.normalized = true;
        }
        for other in self.data_sets.iter_mut() {
            other.normalize_to_frequencies();
        }
    }

    /// `get_psd(axis='all')`
    pub fn get_psd(&self, axis: &str) -> Option<&Vec<f64>> {
        match axis {
            "x" => Some(&self.psd_x),
            "y" => Some(&self.psd_y),
            "z" => Some(&self.psd_z),
            "all" => Some(&self.psd_sum),
            _ => None,
        }
    }

    /// `get_datasets`
    pub fn get_datasets(&self) -> Vec<&CalibrationData> {
        let mut out = vec![self];
        for d in &self.data_sets {
            out.extend(d.get_datasets());
        }
        out
    }
}

/// `CalibrationResult` namedtuple (`shaper_calibrate.py:60-63`).
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Shaper name.
    pub name: String,
    /// Shaper frequency that produced this result.
    pub freq: f64,
    /// Evaluation frequency bins.
    pub freq_bins: Vec<f64>,
    /// Shaper response per frequency bin.
    pub vals: Vec<f64>,
    /// Remaining vibrations fraction.
    pub vibrs: f64,
    /// Estimated smoothing.
    pub smoothing: f64,
    /// Score balancing vibrations against smoothing.
    pub score: f64,
    /// Maximum projected acceleration.
    pub max_accel: f64,
}

/// `shaper_freqs` for `fit_shaper` / `find_best_shaper`: a range tuple
/// `(start, end, step)` or an explicit frequency list. `None` (or an empty
/// list) means `(None, None, None)`.
#[derive(Debug, Clone)]
pub enum ShaperFreqs {
    /// Tuple form `(start, end, step)`.
    Range(Option<f64>, Option<f64>, Option<f64>),
    /// Explicit frequency list.
    List(Vec<f64>),
}

// ---------------------------------------------------------------------------
// `ShaperCalibrate`.
// ---------------------------------------------------------------------------

/// `ShaperCalibrate` (`shaper_calibrate.py:65-493`).
pub struct ShaperCalibrate {
    printer: Option<Printer>,
    /// `NumPy` is unconditionally available in the port; `__init__` would
    /// raise `command_error("Failed to import `numpy` ...")` on the host
    /// interpreter without NumPy.
    pub has_numpy: bool,
}

impl std::fmt::Debug for ShaperCalibrate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ShaperCalibrate")
            .field("has_numpy", &self.has_numpy)
            .finish()
    }
}

klippy_api::printer_object!(ShaperCalibrate);

impl ShaperCalibrate {
    /// `ShaperCalibrate(printer)`; `printer=None` mirrors the `printer=None`
    /// test hook (`self.error = Exception` instead of `command_error`).
    pub fn new(printer: Option<Printer>) -> Self {
        ShaperCalibrate { printer, has_numpy: true }
    }

    fn command_error(&self, msg: String) -> KlippyError {
        if self.printer.is_some() {
            KlippyError::command(msg)
        } else {
            KlippyError::Other { class_name: "Exception".to_string(), message: msg }
        }
    }

    /// `background_process_exec`: the port runs the closure inline. Errors
    /// are wrapped as `command_error("Error in remote calculation: %s")`
    /// when a printer is present, mirroring the multiprocessing wrapper.
    pub fn background_process_exec<T>(
        &self,
        f: impl FnOnce() -> KResult<T>,
    ) -> KResult<T> {
        if self.printer.is_none() {
            return f();
        }
        match f() {
            Ok(v) => Ok(v),
            Err(e) => {
                let rendered = percent_format("Error in remote calculation: %s", &[PctValue::Str(e.message().to_string())])
                    .unwrap_or_else(|_| format!("Error in remote calculation: {}", e.message()));
                Err(KlippyError::command(rendered))
            }
        }
    }

    /// `_split_into_windows(x, window_size, overlap)`
    /// Returns the same `(window_size, n_windows)` layout as the reference
    /// `as_strided` view: element `[i][j]` is `x[j * step + i]`.
    pub fn _split_into_windows(&self, x: &[f64], window_size: usize, overlap: usize) -> Vec<Vec<f64>> {
        let step = window_size - overlap;
        let n_windows = x.len().saturating_sub(overlap) / step;
        let mut out = vec![vec![0.0; n_windows]; window_size];
        for i in 0..window_size {
            for j in 0..n_windows {
                out[i][j] = x[j * step + i];
            }
        }
        out
    }

    /// `_psd(x, fs, nfft)` returning `(freqs, psd)`: Welch's method with a
    /// Kaiser window over windows overlapping by half.
    ///
    /// NumPy order: `np.mean(x, axis=0)` sums each window pairwise, the
    /// `rfft` along axis 0 leaves the result transposed, so the final
    /// `result.real.mean(axis=-1)` adds the windows one after the other.
    pub fn _psd(&self, x: &[f64], fs: f64, nfft: usize) -> KResult<(Vec<f64>, Vec<f64>)> {
        let window = numpy::kaiser(nfft, 6.0);
        // Compensation for windowing loss: `1.0 / (window**2).sum()`.
        let window_sq: Vec<f64> = window.iter().map(|w| w * w).collect();
        let scale = 1.0 / numpy::add_reduce(&window_sq);
        let overlap = nfft / 2;
        // `_split_into_windows`: `(x.shape[-1] - overlap) // step` with a
        // strided view of that many windows.
        let step = nfft - overlap;
        if step == 0 {
            return Err(KlippyError::Other {
                class_name: "ZeroDivisionError".to_string(),
                message: "integer division or modulo by zero".to_string(),
            });
        }
        if x.len() < overlap {
            return Err(value_error("negative dimensions are not allowed"));
        }
        let n_windows = (x.len() - overlap) / step;
        let plan = RfftPlan::new(nfft).map_err(py_error)?;
        let bins = nfft / 2 + 1;
        // `result *= scale / fs` and `result[1:-1,:] *= 2.` multiply by the
        // complex scalars `(scale/fs, 0)` and `(2, 0)`.
        let factor = Complex64::new(scale / fs, 0.0);
        let two = Complex64::new(2.0, 0.0);
        let mut sums = vec![0.0; bins];
        let mut detrended = vec![0.0; nfft];
        for j in 0..n_windows {
            let segment = &x[j * step..j * step + nfft];
            // First detrend, then apply the windowing function.
            let mean = numpy::mean(segment);
            for (d, (&w, &v)) in detrended.iter_mut().zip(window.iter().zip(segment)) {
                *d = w * (v - mean);
            }
            for (k, c) in plan.rfft(&detrended).into_iter().enumerate() {
                let power = c.conj() * c * factor;
                let power = if k >= 1 && k + 1 < bins { power * two } else { power };
                sums[k] += power.re;
            }
        }
        let psd = sums.into_iter().map(|s| s / n_windows as f64).collect();
        Ok((numpy::rfftfreq(nfft, 1. / fs), psd))
    }

    /// `calc_freq_response(name, raw_values)`: `None` mirrors
    /// `raw_values is None`, `Some(samples)` mirrors a `get_samples()` list
    /// or a NumPy array (an empty list also returns `None`).
    pub fn calc_freq_response(
        &self,
        name: &str,
        raw_values: Option<&[Sample]>,
    ) -> KResult<Option<CalibrationData>> {
        let Some(data) = raw_values else { return Ok(None) };
        if data.is_empty() {
            return Ok(None);
        }
        let n = data.len();
        let t = data[n - 1][0] - data[0][0];
        let sampling_freq = n as f64 / t;
        // Round up to the nearest power of 2 for faster FFT:
        // `M = 1 << int(SAMPLING_FREQ * WINDOW_T_SEC - 1).bit_length()`.
        let v = sampling_freq * WINDOW_T_SEC - 1.0;
        if v.is_nan() {
            return Err(value_error("cannot convert float NaN to integer"));
        }
        if v.is_infinite() {
            return Err(KlippyError::Other {
                class_name: "OverflowError".to_string(),
                message: "cannot convert float infinity to integer".to_string(),
            });
        }
        let bits = trunc_float_to_bigint(v).0.bits();
        // `if N <= M: return None` (an `M` beyond `usize` is larger than any `N`).
        if bits >= u64::from(usize::BITS - 1) || n <= 1usize << bits {
            return Ok(None);
        }
        let m = 1usize << bits;
        let column = |idx: usize| -> Vec<f64> { data.iter().map(|s| s[idx]).collect() };
        let (fx, px) = self._psd(&column(1), sampling_freq, m)?;
        let (_fy, py) = self._psd(&column(2), sampling_freq, m)?;
        let (_fz, pz) = self._psd(&column(3), sampling_freq, m)?;
        let psd_sum: Vec<f64> = px.iter().zip(&py).zip(&pz).map(|((a, b), c)| a + b + c).collect();
        Ok(Some(CalibrationData::new(name.to_string(), fx, psd_sum, px, py, pz)))
    }

    /// `process_accelerometer_data`
    pub fn process_accelerometer_data(&self, name: &str, data: Option<&[Sample]>) -> KResult<CalibrationData> {
        let calibration_data = self.background_process_exec(|| self.calc_freq_response(name, data))?;
        // The None check below mirrors the reference
        // `raise self.error("Internal error ...")`.
        match calibration_data {
            Some(mut d) => {
                d.set_numpy();
                Ok(d)
            }
            None => Err(self.command_error(format!(
                "Internal error processing accelerometer data {data:?}"
            ))),
        }
    }

    /// `_estimate_shaper(shaper, test_damping_ratio, test_freqs)`
    pub fn _estimate_shaper(&self, shaper: &Shaper, test_damping_ratio: f64, test_freqs: &[f64]) -> KResult<Vec<f64>> {
        let (a, t) = shaper;
        let inv_d = 1. / numpy::add_reduce(a);
        let Some(&t_last) = t.last() else {
            return Err(KlippyError::Other {
                class_name: "IndexError".to_string(),
                message: "index -1 is out of bounds for axis 0 with size 0".to_string(),
            });
        };
        let damped = lm::sqrt(1. - lm::pow(test_damping_ratio, 2.0));
        // One row of `W`, `S` and `C` per test frequency; `S.sum(axis=1)` sums
        // each row along the inner loop.
        let mut s_row = vec![0.0; a.len()];
        let mut c_row = vec![0.0; a.len()];
        let mut out = Vec::with_capacity(test_freqs.len());
        for &tf in test_freqs {
            let omega = 2. * std::f64::consts::PI * tf;
            let damping = test_damping_ratio * omega;
            let omega_d = omega * damped;
            for (j, (&aj, &tj)) in a.iter().zip(t).enumerate() {
                let w = aj * lm::exp(-damping * (t_last - tj));
                s_row[j] = w * lm::sin(omega_d * tj);
                c_row[j] = w * lm::cos(omega_d * tj);
            }
            let s_sum = numpy::add_reduce(&s_row);
            let c_sum = numpy::add_reduce(&c_row);
            out.push(lm::sqrt(s_sum * s_sum + c_sum * c_sum) * inv_d);
        }
        Ok(out)
    }

    /// `_estimate_remaining_vibrations`
    pub fn _estimate_remaining_vibrations(
        &self,
        shaper: &Shaper,
        test_damping_ratio: f64,
        freq_bins: &[f64],
        psd: &[f64],
    ) -> KResult<(f64, Vec<f64>)> {
        let vals = self._estimate_shaper(shaper, test_damping_ratio, freq_bins)?;
        // The input shaper can only reduce the amplitude of vibrations by
        // SHAPER_VIBRATION_REDUCTION times, so all vibrations below that
        // threshold can be ignored.
        let vibr_threshold = numpy::max_reduce(psd).map_err(py_error)? / SHAPER_VIBRATION_REDUCTION;
        let remaining: Vec<f64> = vals
            .iter()
            .zip(psd)
            .map(|(&v, &p)| numpy::maximum(v * p - vibr_threshold, 0.))
            .collect();
        let all: Vec<f64> = psd.iter().map(|&p| numpy::maximum(p - vibr_threshold, 0.)).collect();
        Ok((numpy::add_reduce(&remaining) / numpy::add_reduce(&all), vals))
    }

    /// `_get_shaper_smoothing(shaper, accel=5000, scv=5.)`
    pub fn _get_shaper_smoothing(&self, shaper: &Shaper, accel: f64, scv: f64) -> f64 {
        let half_accel = accel * 0.5;
        let (a, t) = shaper;
        // Builtin `sum` over Python floats: left to right from 0.
        let inv_d = 1.0 / numpy::sequential_sum(a.iter().copied());
        let n = t.len();
        // Calculate input shaper shift.
        let ts = numpy::sequential_sum((0..n).map(|i| a[i] * t[i])) * inv_d;
        // Calculate offset for 90 and 180 degrees turn.
        let mut offset_90 = 0.0;
        let mut offset_180 = 0.0;
        for i in 0..n {
            if t[i] >= ts {
                // Calculate offset for one of the axes.
                offset_90 += a[i] * (scv + half_accel * (t[i] - ts)) * (t[i] - ts);
            }
            offset_180 += a[i] * half_accel * lm::pow(t[i] - ts, 2.0);
        }
        offset_90 *= inv_d * lm::sqrt(2.0);
        offset_180 *= inv_d;
        py_max(offset_90, offset_180)
    }

    /// `find_shaper_max_accel`
    pub fn find_shaper_max_accel(&self, shaper: &Shaper, scv: f64) -> f64 {
        const TARGET_SMOOTHING: f64 = 0.12;
        self._bisect(|test_accel: f64| self._get_shaper_smoothing(shaper, test_accel, scv) <= TARGET_SMOOTHING)
    }

    /// `_bisect(func)`
    pub fn _bisect<F: Fn(f64) -> bool>(&self, func: F) -> f64 {
        let mut left = 1.0;
        let mut right = 1.0;
        if !func(1e-9) {
            return 0.0;
        }
        while !func(left) {
            right = left;
            left *= 0.5;
        }
        if right == left {
            while func(right) {
                right *= 2.0;
            }
        }
        while right - left > 1e-8 {
            let middle = (left + right) * 0.5;
            if func(middle) {
                left = middle;
            } else {
                right = middle;
            }
        }
        left
    }

    /// `fit_shaper`
    #[allow(clippy::too_many_arguments)]
    pub fn fit_shaper(
        &self,
        shaper_name: &str,
        calibration_data: &CalibrationData,
        shaper_freqs: Option<&ShaperFreqs>,
        damping_ratio: Option<f64>,
        scv: Option<f64>,
        max_smoothing: Option<f64>,
        max_vibrations: Option<f64>,
        test_damping_ratios: Option<&[f64]>,
        max_freq: Option<f64>,
    ) -> KResult<Vec<CalibrationResult>> {
        let damping_ratio = or_default(damping_ratio, DEFAULT_DAMPING_RATIO);
        let default_drs: Vec<f64> = test_damping_ratios
            .filter(|s| !s.is_empty())
            .map(|s| s.to_vec())
            .unwrap_or_else(|| TEST_DAMPING_RATIOS.to_vec());
        let test_freqs: Vec<f64> = match shaper_freqs {
            Some(ShaperFreqs::List(freqs)) if !freqs.is_empty() => freqs.clone(),
            other => {
                // `if not shaper_freqs: shaper_freqs = (None, None, None)`.
                let (start, end, step) = match other {
                    Some(ShaperFreqs::Range(start, end, step)) => (*start, *end, *step),
                    _ => (None, None, None),
                };
                let freq_end = or_default(end, MAX_SHAPER_FREQ);
                // `shaper_freqs[0] or get_shaper_cfg(shaper_name).min_freq`:
                // the config is only looked up for a falsy start.
                let start_val = match start {
                    Some(v) if v != 0.0 => v,
                    _ => match get_shaper_cfg(shaper_name) {
                        Some(cfg) => cfg.min_freq,
                        None => {
                            return Err(KlippyError::Other {
                                class_name: "AttributeError".to_string(),
                                message: "'NoneType' object has no attribute 'min_freq'".to_string(),
                            })
                        }
                    },
                };
                let freq_start = py_min(start_val, freq_end - 1e-7);
                let freq_step = or_default(step, 0.2);
                numpy::arange(freq_start, freq_end, freq_step).map_err(py_error)?
            }
        };
        let max_freq = py_max(
            or_default(max_freq, MAX_FREQ),
            numpy::max_reduce(&test_freqs).map_err(py_error)?,
        );
        let mut min_freq = max_freq;
        for data in calibration_data.get_datasets() {
            min_freq = py_min(min_freq, numpy::min_reduce(&data.freq_bins).map_err(py_error)?);
        }
        let mut best_res: Option<CalibrationResult> = None;
        let mut results: Vec<CalibrationResult> = Vec::new();
        for &test_freq in test_freqs.iter().rev() {
            let mut shaper_vibrations = 0.0;
            let shaper = match init_shaper(shaper_name, test_freq, damping_ratio, None)? {
                Some(s) => s,
                None => {
                    return Err(type_error("cannot unpack non-iterable NoneType object"));
                }
            };
            // `scv=None` raises in `_get_shaper_smoothing` on the first
            // `scv + half_accel * ...` (every shaper has an impulse at or
            // after its shift).
            let Some(scv_val) = scv else {
                return Err(type_error("unsupported operand type(s) for +: 'NoneType' and 'float'"));
            };
            let shaper_smoothing = self._get_shaper_smoothing(&shaper, 5000.0, scv_val);
            if let Some(best) = &best_res {
                if matches!(max_smoothing, Some(ms) if ms != 0.0 && shaper_smoothing > ms) {
                    let mut out = vec![best.clone()];
                    out.extend(results);
                    return Ok(out);
                }
            }
            let max_accel = self.find_shaper_max_accel(&shaper, scv_val);
            let mut all_shaper_vals: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
            for data in calibration_data.get_datasets() {
                let mut freq_bins: Vec<f64> = Vec::new();
                let mut psd: Vec<f64> = Vec::new();
                for (&f, &p) in data.freq_bins.iter().zip(&data.psd_sum) {
                    if f <= max_freq {
                        freq_bins.push(f);
                        psd.push(p);
                    }
                }
                let mut shaper_vals = vec![0.0; freq_bins.len()];
                // Exact damping ratio of the printer is unknown, pessimizing
                // remaining vibrations over possible damping values.
                for &dr in &default_drs {
                    let (vibrations, vals) =
                        self._estimate_remaining_vibrations(&shaper, dr, &freq_bins, &psd)?;
                    for (slot, &v) in shaper_vals.iter_mut().zip(&vals) {
                        *slot = numpy::maximum(*slot, v);
                    }
                    if vibrations > shaper_vibrations {
                        shaper_vibrations = vibrations;
                    }
                }
                all_shaper_vals.push((freq_bins, shaper_vals));
            }
            // The score trying to minimize vibrations, but also accounting
            // the growth of smoothing.
            let shaper_score = shaper_smoothing
                * (lm::pow(shaper_vibrations, 1.5) + shaper_vibrations * 0.2 + 0.01);
            let shaper_freq_bins = numpy::arange(min_freq, max_freq, 0.2).map_err(py_error)?;
            let mut shaper_vals = vec![0.0; shaper_freq_bins.len()];
            for (freq_bins, vals) in &all_shaper_vals {
                let interp_vals = numpy::interp(&shaper_freq_bins, freq_bins, vals).map_err(py_error)?;
                for (slot, &v) in shaper_vals.iter_mut().zip(&interp_vals) {
                    *slot = numpy::maximum(*slot, v);
                }
            }
            let result = CalibrationResult {
                name: shaper_name.to_string(),
                freq: test_freq,
                freq_bins: shaper_freq_bins,
                vals: shaper_vals,
                vibrs: shaper_vibrations,
                smoothing: shaper_smoothing,
                score: shaper_score,
                max_accel,
            };
            if best_res.as_ref().is_none_or(|b| b.vibrs > result.vibrs) {
                // The current frequency is better for the shaper.
                best_res = Some(result.clone());
            }
            results.push(result);
        }
        // Try to find an 'optimal' shapper configuration: the one that is not
        // much worse than the 'best' one, but gives much less smoothing.
        let Some(best) = best_res else {
            // Unreachable with a non-empty `test_freqs`; Python would fail on
            // `best_res.vibrs`.
            return Err(KlippyError::Other {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'vibrs'".to_string(),
            });
        };
        let mut selected = best.clone();
        for res in results.iter().rev() {
            if res.vibrs < best.vibrs * 1.1 + 0.0005
                && (res.score < selected.score || max_vibrations.is_some_and(|mv| best.vibrs > mv))
            {
                selected = res.clone();
            }
        }
        let mut out = vec![selected];
        out.extend(results);
        Ok(out)
    }

    /// `find_best_shaper`
    #[allow(clippy::too_many_arguments)]
    pub fn find_best_shaper(
        &self,
        calibration_data: &CalibrationData,
        shapers: Option<&[&str]>,
        damping_ratio: Option<f64>,
        scv: Option<f64>,
        shaper_freqs: Option<&ShaperFreqs>,
        max_smoothing: Option<f64>,
        max_vibrations: Option<f64>,
        test_damping_ratios: Option<&[f64]>,
        max_freq: Option<f64>,
        logger: Option<&dyn Fn(&str)>,
    ) -> KResult<(CalibrationResult, Vec<CalibrationResult>)> {
        let mut best_shaper: Option<CalibrationResult> = None;
        let mut all_shapers: Vec<CalibrationResult> = Vec::new();
        let default_shapers: Vec<&str> = AUTOTUNE_SHAPERS.to_vec();
        let shapers = match shapers {
            None | Some([]) => &default_shapers[..],
            Some(s) => s,
        };
        for &shaper_name in shapers {
            let shaper_cfg = get_shaper_cfg(shaper_name);
            if shaper_cfg.is_none() {
                if let Some(log) = logger {
                    log(&join_percent("Unknown shaper %s", &[PctValue::Str(shaper_name.to_string())]));
                }
                continue;
            }
            let cfg = shaper_cfg.unwrap();
            let init_res = init_shaper(shaper_name, cfg.min_freq, DEFAULT_DAMPING_RATIO, None);
            if let Err(e) = init_res {
                if let Some(log) = logger {
                    log(&join_percent(
                        "Cannot initialize shaper %s: %s",
                        &[PctValue::Str(shaper_name.to_string()), PctValue::Str(e.message().to_string())],
                    ));
                    continue;
                }
            }
            let fit_results = self.background_process_exec(|| {
                self.fit_shaper(
                    shaper_name,
                    calibration_data,
                    shaper_freqs,
                    damping_ratio,
                    scv,
                    max_smoothing,
                    max_vibrations,
                    test_damping_ratios,
                    max_freq,
                )
            })?;
            let mut shaper = fit_results[0].clone();
            let results = &fit_results[1..];
            let take = match &best_shaper {
                None => true,
                Some(best) => {
                    shaper.score * 1.2 < best.score
                        || (shaper.score * 1.05 < best.score && shaper.smoothing * 1.1 < best.smoothing)
                        || (max_vibrations.is_some()
                            && best.vibrs > max_vibrations.unwrap()
                            && shaper.vibrs < best.vibrs)
                }
            };
            if take {
                best_shaper = Some(shaper.clone());
            }
            for s in results.iter().rev() {
                let best = best_shaper.as_ref().unwrap();
                if s.vibrs < best.vibrs && s.smoothing < best.smoothing {
                    best_shaper = Some(s.clone());
                    shaper = s.clone();
                }
            }
            if let Some(log) = logger {
                log(&join_percent(
                    "Fitted shaper '%s' frequency = %.1f Hz (vibrations = %.1f%%, smoothing ~= %.3f)",
                    &[
                        PctValue::Str(shaper.name.clone()),
                        PctValue::Float(shaper.freq),
                        PctValue::Float(shaper.vibrs * 100.0),
                        PctValue::Float(shaper.smoothing),
                    ],
                ));
                // `round(shaper.max_accel / 100.) * 100.`
                let rounded = pycompat::text::round_py(shaper.max_accel / 100.0)
                    .map_err(py_error)?
                    .to_f64_exact()
                    .filter(|v| v.is_finite())
                    .ok_or_else(|| KlippyError::Other {
                        class_name: "OverflowError".to_string(),
                        message: "int too large to convert to float".to_string(),
                    })?
                    * 100.0;
                log(&join_percent(
                    "To avoid too much smoothing with '%s', suggested max_accel <= %.0f mm/sec^2",
                    &[PctValue::Str(shaper.name.clone()), PctValue::Float(rounded)],
                ));
            }
            all_shapers.push(shaper);
        }
        let mut best = match best_shaper {
            Some(b) => b,
            None => {
                return Err(KlippyError::Other {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'name'".to_string(),
                })
            }
        };
        if best.name == "zv" {
            for tuned in &all_shapers {
                if tuned.name != "zv" && tuned.vibrs * 1.1 < best.vibrs {
                    best = tuned.clone();
                    break;
                }
            }
        }
        Ok((best, all_shapers))
    }

    /// `save_params`
    pub fn save_params(
        &self,
        configfile: &dyn PrinterConfigApi,
        axis: &str,
        shaper_name: &str,
        shaper_freq: f64,
    ) -> KResult<()> {
        if axis == "xy" {
            self.save_params(configfile, "x", shaper_name, shaper_freq)?;
            self.save_params(configfile, "y", shaper_name, shaper_freq)?;
        } else {
            configfile.set("input_shaper", &format!("shaper_type_{axis}"), shaper_name)?;
            configfile.set(
                "input_shaper",
                &format!("shaper_freq_{axis}"),
                &join_percent("%.1f", &[PctValue::Float(shaper_freq)]),
            )?;
        }
        Ok(())
    }

    /// `apply_params`
    pub fn apply_params(
        &self,
        input_shaper: &dyn crate::input_shaper::InputShaperApi,
        axis: &str,
        shaper_name: &str,
        shaper_freq: f64,
    ) -> KResult<()> {
        if axis == "xy" {
            self.apply_params(input_shaper, "x", shaper_name, shaper_freq)?;
            self.apply_params(input_shaper, "y", shaper_name, shaper_freq)?;
            return Ok(());
        }
        let Some(printer) = &self.printer else {
            return Err(KlippyError::Other {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'lookup_object'".to_string(),
            });
        };
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(
            &**printer,
            "gcode",
            klippy_api::printer::LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let axis_u = axis.to_uppercase();
        let mut params = indexmap::IndexMap::new();
        params.insert(format!("SHAPER_TYPE_{axis_u}"), shaper_name.to_string());
        params.insert(format!("SHAPER_FREQ_{axis_u}"), format!("{shaper_freq:?}"));
        let gcmd = gcode.create_gcode_command("SET_INPUT_SHAPER", "SET_INPUT_SHAPER", &params);
        input_shaper.cmd_set_input_shaper(&gcmd)
    }

    /// `_escape_for_csv`
    pub fn _escape_for_csv(&self, name: &str) -> String {
        let name = name.replace('"', "\"\"");
        if name.contains(',') {
            format!("\"{name}\"")
        } else {
            name
        }
    }

    /// `save_calibration_data`
    pub fn save_calibration_data(
        &self,
        output: &str,
        calibration_data: &CalibrationData,
        shapers: Option<&[CalibrationResult]>,
        max_freq: Option<f64>,
    ) -> KResult<()> {
        // Python `if shapers:` — an empty list is falsy.
        let shapers = shapers.filter(|s| !s.is_empty());
        let datasets = calibration_data.get_datasets();
        let max_freq = or_default(max_freq, MAX_FREQ);
        // Only `IOError` is turned into the "Error writing" error; NumPy
        // errors from `arange`/`interp` propagate as they are.
        let (freq_bins, psd_data_to_write): (Vec<f64>, Vec<Vec<f64>>) = if datasets.len() > 1 {
            let freq_bins = if let Some(shapers) = shapers {
                shapers[0].freq_bins.clone()
            } else {
                let mut min_freq = max_freq;
                for data in &datasets {
                    min_freq = py_min(min_freq, numpy::min_reduce(&data.freq_bins).map_err(py_error)?);
                }
                numpy::arange(min_freq, max_freq, 0.2).map_err(py_error)?
            };
            let mut psd_data = Vec::new();
            for data in &datasets {
                psd_data.push(numpy::interp(&freq_bins, &data.freq_bins, &data.psd_sum).map_err(py_error)?);
            }
            (freq_bins, psd_data)
        } else {
            (
                calibration_data.freq_bins.clone(),
                vec![
                    calibration_data.psd_x.clone(),
                    calibration_data.psd_y.clone(),
                    calibration_data.psd_z.clone(),
                    calibration_data.psd_sum.clone(),
                ],
            )
        };
        let run = || -> std::io::Result<()> {
            let mut file = OpenOptions::new().write(true).create(true).truncate(true).open(output)?;
            file.write_all(b"freq,")?;
            if datasets.len() > 1 {
                let header: Vec<String> = datasets.iter().map(|d| self._escape_for_csv(&d.name)).collect();
                file.write_all(header.join(",").as_bytes())?;
            } else {
                file.write_all(b"psd_x,psd_y,psd_z,psd_xyz")?;
            }
            if let Some(shapers) = shapers {
                file.write_all(b",shapers:")?;
                for shaper in shapers {
                    let line = join_percent(",%s(%.1f)", &[PctValue::Str(shaper.name.clone()), PctValue::Float(shaper.freq)]);
                    file.write_all(line.as_bytes())?;
                }
            }
            file.write_all(b"\n")?;
            let num_freqs = freq_bins.len();
            for i in 0..num_freqs {
                if freq_bins[i] >= max_freq {
                    break;
                }
                file.write_all(join_percent("%.1f", &[PctValue::Float(freq_bins[i])]).as_bytes())?;
                for psd in &psd_data_to_write {
                    file.write_all(join_percent(",%.3e", &[PctValue::Float(psd[i])]).as_bytes())?;
                }
                if let Some(shapers) = shapers {
                    file.write_all(b",")?;
                    for shaper in shapers {
                        file.write_all(join_percent(",%.3f", &[PctValue::Float(shaper.vals[i])]).as_bytes())?;
                    }
                }
                file.write_all(b"\n")?;
            }
            file.flush()
        };
        match run() {
            Ok(()) => Ok(()),
            Err(e) => Err(self.command_error(format!("Error writing to file '{output}': {e}"))),
        }
    }
}

// ---------------------------------------------------------------------------
// Free helpers.
// ---------------------------------------------------------------------------

fn join_percent(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|_| fmt.to_string())
}

/// A NumPy error (`ValueError`, `ZeroDivisionError`, ...) as the same
/// Python exception.
fn py_error(e: PyError) -> KlippyError {
    KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
}

/// Builtin `max(a, b)`: `b` replaces `a` only when `b > a`.
fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Builtin `min(a, b)`: `b` replaces `a` only when `b < a`.
fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `value or default` for an optional float argument (`None` and `0.0`
/// are falsy).
fn or_default(value: Option<f64>, default: f64) -> f64 {
    match value {
        Some(v) if v != 0.0 => v,
        _ => default,
    }
}

/// `module_factory()`.
pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    use klippy_api::printer::ModuleFactory;
    use klippy_api::config::Config;
    use klippy_api::object::ObjectRef;
    fn load_config(_config: &Config) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    ModuleFactory { load_config: Some(load_config), ..Default::default() }
}
