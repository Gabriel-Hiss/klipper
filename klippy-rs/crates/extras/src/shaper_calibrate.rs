//! Port of the `shaper_calibrate` module (`klippy/extras/shaper_calibrate.py`).
//!
//! Pure numeric calibration of input shapers from accelerometer PSD data,
//! plus persistence (`save_params` / `save_calibration_data`). There are no
//! G-code commands, config options or status fields (see
//! `parity/inventory/units.json`).
//!
//! NumPy operations are reimplemented with plain `Vec<f64>` in the same
//! arithmetic order as the Python code so results match NumPy to
//! float-analytic precision. `_psd` uses a naive DFT instead of NumPy's FFT
//! (mathematically identical `|X|^2` power, different rounding at the
//! 1e-12 level; see the report).

use std::fs::OpenOptions;
use std::io::Write;

use klippy_api::config::PrinterConfigApi;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi};
use klippy_api::printer::Printer;

use pycompat::libm as lm;
use pycompat::text::{percent_format, PctValue};

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

    /// `_psd(x, fs, nfft)` returning `(freqs, psd)`.
    pub fn _psd(&self, x: &[f64], fs: f64, nfft: usize) -> (Vec<f64>, Vec<f64>) {
        let window = kaiser(nfft, 6.0);
        let mut denom = 0.0;
        for &w in &window {
            denom += w * w;
        }
        let scale = 1.0 / denom;
        let overlap = nfft / 2;
        // Reference layout is `(nfft, n_windows)`; each column is one window.
        let windows = self._split_into_windows(x, nfft, overlap);
        let n_windows = if windows.is_empty() { 0 } else { windows[0].len() };
        let bins = nfft / 2 + 1;
        let mut psd = vec![0.0; bins];
        for j in 0..n_windows {
            let mean = {
                let mut s = 0.0;
                for i in 0..nfft {
                    s += windows[i][j];
                }
                s / nfft as f64
            };
            let detrended: Vec<f64> = (0..nfft).map(|i| window[i] * (windows[i][j] - mean)).collect();
            // Naive DFT == FFT up to rounding; `|X|^2` is the periodogram.
            for (k, slot) in psd.iter_mut().enumerate() {
                let mut re = 0.0;
                let mut im = 0.0;
                for (n, &v) in detrended.iter().enumerate() {
                    let theta = 2.0 * std::f64::consts::PI * k as f64 * n as f64 / nfft as f64;
                    re += v * lm::cos(theta);
                    im -= v * lm::sin(theta);
                }
                let mut power = (re * re + im * im) * scale / fs;
                if k != 0 && k != nfft / 2 {
                    power *= 2.0;
                }
                *slot += power;
            }
        }
        if n_windows > 0 {
            for p in psd.iter_mut() {
                *p /= n_windows as f64;
            }
        }
        let mut freqs = Vec::with_capacity(bins);
        for k in 0..bins {
            freqs.push(k as f64 * fs / nfft as f64);
        }
        (freqs, psd)
    }

    /// `calc_freq_response(name, raw_values)`: `None` mirrors
    /// `raw_values is None`, `Some(samples)` mirrors a `get_samples()` list
    /// or a NumPy array (empty list also returns `None`).
    pub fn calc_freq_response(&self, name: &str, raw_values: Option<&[Sample]>) -> Option<CalibrationData> {
        let data = raw_values?;
        if data.is_empty() {
            return None;
        }
        let n = data.len() as f64;
        let t = data[data.len() - 1][0] - data[0][0];
        let sampling_freq = n / t;
        let m = {
            // `1 << int(f * WINDOW_T_SEC - 1).bit_length()`
            let v = (sampling_freq * WINDOW_T_SEC - 1.0) as i64;
            let bits = if v == 0 {
                0u32
            } else {
                64 - v.unsigned_abs().leading_zeros()
            };
            1usize << bits
        };
        if (data.len() as f64) <= m as f64 {
            return None;
        }
        let col = |idx: usize| -> Vec<f64> { data.iter().map(|s| s[idx]).collect() };
        let (fx, px) = self._psd(&col(1), sampling_freq, m);
        let (_fy, py) = self._psd(&col(2), sampling_freq, m);
        let (_fz, pz) = self._psd(&col(3), sampling_freq, m);
        let psd_sum: Vec<f64> = px.iter().zip(py.iter()).zip(pz.iter()).map(|((a, b), c)| a + b + c).collect();
        Some(CalibrationData::new(name.to_string(), fx, psd_sum, px, py, pz))
    }

    /// `process_accelerometer_data`
    pub fn process_accelerometer_data(&self, name: &str, data: Option<&[Sample]>) -> KResult<CalibrationData> {
        let calibration_data = self.background_process_exec(|| Ok(self.calc_freq_response(name, data)))?;
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
    pub fn _estimate_shaper(&self, shaper: &Shaper, test_damping_ratio: f64, test_freqs: &[f64]) -> Vec<f64> {
        let (a, t) = shaper;
        let sum_a: f64 = a.iter().sum();
        let inv_d = 1.0 / sum_a;
        let mut out = vec![0.0; test_freqs.len()];
        for (i, &tf) in test_freqs.iter().enumerate() {
            let omega = 2.0 * std::f64::consts::PI * tf;
            let damping = test_damping_ratio * omega;
            let omega_d = omega * lm::sqrt(1.0 - test_damping_ratio * test_damping_ratio);
            let t_last = t[t.len() - 1];
            let mut s_sum = 0.0;
            let mut c_sum = 0.0;
            for j in 0..a.len() {
                let w = a[j] * lm::exp(-damping * (t_last - t[j]));
                s_sum += w * lm::sin(omega_d * t[j]);
                c_sum += w * lm::cos(omega_d * t[j]);
            }
            out[i] = lm::sqrt(s_sum * s_sum + c_sum * c_sum) * inv_d;
        }
        out
    }

    /// `_estimate_remaining_vibrations`
    pub fn _estimate_remaining_vibrations(
        &self,
        shaper: &Shaper,
        test_damping_ratio: f64,
        freq_bins: &[f64],
        psd: &[f64],
    ) -> (f64, Vec<f64>) {
        let vals = self._estimate_shaper(shaper, test_damping_ratio, freq_bins);
        let psd_max = psd.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b));
        let vibr_threshold = psd_max / SHAPER_VIBRATION_REDUCTION;
        let mut remaining = 0.0;
        let mut all = 0.0;
        for i in 0..psd.len() {
            remaining += (vals[i] * psd[i] - vibr_threshold).max(0.0);
            all += (psd[i] - vibr_threshold).max(0.0);
        }
        (remaining / all, vals)
    }

    /// `_get_shaper_smoothing(shaper, accel=5000, scv=5.)`
    pub fn _get_shaper_smoothing(&self, shaper: &Shaper, accel: f64, scv: f64) -> f64 {
        let half_accel = accel * 0.5;
        let (a, t) = shaper;
        let sum_a: f64 = a.iter().sum();
        let inv_d = 1.0 / sum_a;
        let n = t.len();
        let mut ts_num = 0.0;
        for i in 0..n {
            ts_num += a[i] * t[i];
        }
        let ts = ts_num * inv_d;
        let mut offset_90 = 0.0;
        let mut offset_180 = 0.0;
        for i in 0..n {
            if t[i] >= ts {
                offset_90 += a[i] * (scv + half_accel * (t[i] - ts)) * (t[i] - ts);
            }
            offset_180 += a[i] * half_accel * (t[i] - ts) * (t[i] - ts);
        }
        offset_90 *= inv_d * lm::sqrt(2.0);
        offset_180 *= inv_d;
        offset_90.max(offset_180)
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
        let damping_ratio = match damping_ratio {
            None | Some(0.0) => DEFAULT_DAMPING_RATIO,
            Some(v) => v,
        };
        let default_drs: Vec<f64> = test_damping_ratios
            .filter(|s| !s.is_empty())
            .map(|s| s.to_vec())
            .unwrap_or_else(|| TEST_DAMPING_RATIOS.to_vec());
        let test_freqs: Vec<f64> = match shaper_freqs {
            None => {
                let cfg = match get_shaper_cfg(shaper_name) {
                    Some(c) => c,
                    None => {
                        return Err(KlippyError::Other {
                            class_name: "AttributeError".to_string(),
                            message: "'NoneType' object has no attribute 'min_freq'".to_string(),
                        })
                    }
                };
                let min_freq = cfg.min_freq;
                let freq_end = MAX_SHAPER_FREQ;
                let freq_start = min_freq.min(freq_end - 1e-7);
                arange(freq_start, freq_end, 0.2)
            }
            Some(ShaperFreqs::Range(start, end, step)) => {
                let cfg = match get_shaper_cfg(shaper_name) {
                    Some(c) => c,
                    None => {
                        return Err(KlippyError::Other {
                            class_name: "AttributeError".to_string(),
                            message: "'NoneType' object has no attribute 'min_freq'".to_string(),
                        })
                    }
                };
                let freq_end = match end {
                    None | Some(0.0) => MAX_SHAPER_FREQ,
                    Some(v) => *v,
                };
                // `start or min_freq`: None and 0.0 are falsy.
                let start_val = match start {
                    None | Some(0.0) => cfg.min_freq,
                    Some(v) => *v,
                };
                let freq_start = start_val.min(freq_end - 1e-7);
                let freq_step = match step {
                    None | Some(0.0) => 0.2,
                    Some(v) => *v,
                };
                arange(freq_start, freq_end, freq_step)
            }
            Some(ShaperFreqs::List(freqs)) => {
                if freqs.is_empty() {
                    // `if not shaper_freqs` maps [] to the default tuple.
                    let cfg = match get_shaper_cfg(shaper_name) {
                        Some(c) => c,
                        None => {
                            return Err(KlippyError::Other {
                                class_name: "AttributeError".to_string(),
                                message: "'NoneType' object has no attribute 'min_freq'".to_string(),
                            })
                        }
                    };
                    let min_freq = cfg.min_freq;
                    let freq_end = MAX_SHAPER_FREQ;
                    let freq_start = min_freq.min(freq_end - 1e-7);
                    arange(freq_start, freq_end, 0.2)
                } else {
                    freqs.clone()
                }
            }
        };
        if test_freqs.is_empty() {
            // Python: `np.arange(...)` empty then `test_freqs.max()` raises
            // `ValueError: zero-size array to reduction operation maximum...`.
            return Err(value_error(
                "zero-size array to reduction operation maximum which has no identity".to_string(),
            ));
        }
        let max_test = test_freqs.iter().fold(f64::NEG_INFINITY, |a, b| a.max(*b));
        let max_freq = {
            let base = match max_freq {
                None | Some(0.0) => MAX_FREQ,
                Some(v) => v,
            };
            base.max(max_test)
        };
        let mut min_freq = max_freq;
        for data in calibration_data.get_datasets() {
            let data_min = data.freq_bins.iter().fold(f64::INFINITY, |a, b| a.min(*b));
            if data_min < min_freq {
                min_freq = data_min;
            }
        }
        let scv_val = match scv {
            Some(v) => v,
            None => {
                // `scv=None` reaches `_get_shaper_smoothing` and raises
                // `TypeError` on the first `scv + half_accel * ...` when the
                // shaper is non-empty.
                return Err(type_error(
                    "unsupported operand type(s) for +: 'NoneType' and 'float'".to_string(),
                ));
            }
        };
        let mut best_res: Option<CalibrationResult> = None;
        let mut results: Vec<CalibrationResult> = Vec::new();
        for &test_freq in test_freqs.iter().rev() {
            let mut shaper_vibrations = 0.0;
            let shaper = match init_shaper(shaper_name, test_freq, damping_ratio, None)? {
                Some(s) => s,
                None => {
                    return Err(KlippyError::Other {
                        class_name: "TypeError".to_string(),
                        message: "cannot unpack non-iterable NoneType object".to_string(),
                    })
                }
            };
            let shaper_smoothing = self._get_shaper_smoothing(&shaper, 5000.0, scv_val);
            if let (Some(ms), true) = (max_smoothing, best_res.is_some()) {
                if ms != 0.0 && shaper_smoothing > ms {
                    let mut out = vec![best_res.unwrap()];
                    out.extend(results);
                    return Ok(out);
                }
            }
            let max_accel = self.find_shaper_max_accel(&shaper, scv_val);
            let mut all_shaper_vals: Vec<(Vec<f64>, Vec<f64>)> = Vec::new();
            for data in calibration_data.get_datasets() {
                let mut freq_bins: Vec<f64> = Vec::new();
                let mut psd: Vec<f64> = Vec::new();
                for (i, &f) in data.freq_bins.iter().enumerate() {
                    if f <= max_freq {
                        freq_bins.push(f);
                        psd.push(data.psd_sum[i]);
                    }
                }
                let mut shaper_vals = vec![0.0; freq_bins.len()];
                for &dr in &default_drs {
                    let (vibrations, vals) =
                        self._estimate_remaining_vibrations(&shaper, dr, &freq_bins, &psd);
                    for i in 0..shaper_vals.len() {
                        if vals[i] > shaper_vals[i] {
                            shaper_vals[i] = vals[i];
                        }
                    }
                    if vibrations > shaper_vibrations {
                        shaper_vibrations = vibrations;
                    }
                }
                all_shaper_vals.push((freq_bins, shaper_vals));
            }
            let shaper_score = shaper_smoothing
                * (lm::pow(shaper_vibrations, 1.5) + shaper_vibrations * 0.2 + 0.01);
            let shaper_freq_bins = arange(min_freq, max_freq, 0.2);
            let mut shaper_vals = vec![0.0; shaper_freq_bins.len()];
            for (freq_bins, vals) in &all_shaper_vals {
                let interp_vals = np_interp(&shaper_freq_bins, freq_bins, vals);
                for i in 0..shaper_vals.len() {
                    if interp_vals[i] > shaper_vals[i] {
                        shaper_vals[i] = interp_vals[i];
                    }
                }
            }
            results.push(CalibrationResult {
                name: shaper_name.to_string(),
                freq: test_freq,
                freq_bins: shaper_freq_bins,
                vals: shaper_vals,
                vibrs: shaper_vibrations,
                smoothing: shaper_smoothing,
                score: shaper_score,
                max_accel,
            });
            let better = match &best_res {
                None => true,
                Some(b) => b.vibrs > results[results.len() - 1].vibrs,
            };
            if better {
                best_res = Some(results[results.len() - 1].clone());
            }
        }
        // Try to find an 'optimal' shapper configuration.
        let best = best_res.clone().unwrap_or_else(|| {
            // `test_freqs` empty mirrors Python's `best_res = None` crash at
            // `selected = best_res` followed by `best_res.vibrs`; report the
            // `AttributeError` the interpreter raises for `None.vibrs`.
            CalibrationResult {
                name: shaper_name.to_string(),
                freq: f64::NAN,
                freq_bins: Vec::new(),
                vals: Vec::new(),
                vibrs: f64::NAN,
                smoothing: f64::NAN,
                score: f64::NAN,
                max_accel: f64::NAN,
            }
        });
        if best_res.is_none() {
            return Err(KlippyError::Other {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'vibrs'".to_string(),
            });
        }
        let mut selected = best.clone();
        for res in results.iter().rev() {
            if res.vibrs < best.vibrs * 1.1 + 0.0005
                && (res.score < selected.score || max_vibrations.is_some() && best.vibrs > max_vibrations.unwrap())
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
                let rounded = {
                    let x = shaper.max_accel / 100.0;
                    match pycompat::text::round_py(x) {
                        Ok(v) => v.to_f64_exact().unwrap_or(x) * 100.0,
                        Err(_) => x * 100.0,
                    }
                };
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
        let run = || -> std::io::Result<()> {
            // Python `if shapers:` — an empty list is falsy.
            let shapers = shapers.filter(|s| !s.is_empty());
            let datasets = calibration_data.get_datasets();
            let max_freq = match max_freq {
                None | Some(0.0) => MAX_FREQ,
                Some(v) => v,
            };
            let (freq_bins, psd_data_to_write): (Vec<f64>, Vec<Vec<f64>>) = if datasets.len() > 1 {
                let freq_bins = if let Some(shapers) = shapers {
                    shapers[0].freq_bins.clone()
                } else {
                    let mut min_freq = max_freq;
                    for data in &datasets {
                        let dmin = data.freq_bins.iter().fold(f64::INFINITY, |a, b| a.min(*b));
                        if dmin < min_freq {
                            min_freq = dmin;
                        }
                    }
                    arange(min_freq, max_freq, 0.2)
                };
                let mut psd_data = Vec::new();
                for data in &datasets {
                    psd_data.push(np_interp(&freq_bins, &data.freq_bins, &data.psd_sum));
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

/// Modified Bessel `I0(x)` by power series (matches the analytic value to
/// ~1e-15; NumPy's `kaiser` uses the same series internally).
fn bessel_i0(x: f64) -> f64 {
    let x2_over_4 = x * x / 4.0;
    let mut sum = 1.0;
    let mut term = 1.0;
    for k in 1..200 {
        term *= x2_over_4 / (k as f64 * k as f64);
        sum += term;
        if term < 1e-18 * sum {
            break;
        }
    }
    sum
}

/// `np.kaiser(M, beta)`.
fn kaiser(m: usize, beta: f64) -> Vec<f64> {
    if m == 1 {
        return vec![1.0];
    }
    let denom = bessel_i0(beta);
    let mut out = Vec::with_capacity(m);
    for n in 0..m {
        let r = 2.0 * n as f64 / (m - 1) as f64 - 1.0;
        let arg = beta * lm::sqrt(1.0 - r * r);
        out.push(bessel_i0(arg) / denom);
    }
    out
}

/// `np.arange(start, stop, step)`.
fn arange(start: f64, stop: f64, step: f64) -> Vec<f64> {
    if step <= 0.0 || start >= stop {
        return Vec::new();
    }
    let n = ((stop - start) / step).ceil() as usize;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        out.push(start + i as f64 * step);
    }
    out
}

/// `np.interp(x_new, xp, fp)` with endpoint clamping.
fn np_interp(x_new: &[f64], xp: &[f64], fp: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(x_new.len());
    for &x in x_new {
        if x <= xp[0] {
            out.push(fp[0]);
            continue;
        }
        if x >= xp[xp.len() - 1] {
            out.push(fp[fp.len() - 1]);
            continue;
        }
        // Linear search: the input sizes here are small (<= 1000 bins).
        let mut j = 0;
        while j + 1 < xp.len() && xp[j + 1] < x {
            j += 1;
        }
        let x0 = xp[j];
        let x1 = xp[j + 1];
        let y0 = fp[j];
        let y1 = fp[j + 1];
        if (x1 - x0) == 0.0 {
            out.push(y0);
        } else {
            out.push(y0 + (x - x0) * (y1 - y0) / (x1 - x0));
        }
    }
    out
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
