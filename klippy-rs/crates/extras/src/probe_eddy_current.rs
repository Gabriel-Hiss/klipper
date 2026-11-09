//! Port of the `probe_eddy_current` module (`klippy/extras/probe_eddy_current.py`).
//!
//! Eddy-current Z probes: frequency-to-height calibration, `probe` sessions
//! for `descend`, `tap` and `scan`/`rapid_scan` methods, and the tap
//! least-squares analysis.
//!
//! The sensor named by `sensor_type` is the `ldc1612` module's `LDC1612`,
//! driven through its owner's [`LDC1612Api`] and [`TriggerAnalogSensor`].
//! Drift compensation is registered through [`DriftCompensationApi`] by
//! `temperature_probe`.

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetChoiceOpts, GetFloatOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeMoveApi, GetFloatOpts as GcmdFloatOpts, GetIntOpts as GcmdIntOpts,
    GetOpts as GcmdGetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::Endstop;
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_GCODE_COMMAND_ERROR, EV_PROBE_UPDATE_RESULTS,
};
use klippy_api::toolhead::{PrinterHomingApi, ToolHeadApi};
use klippy_api::{lookup_interface, EventArg, GCell, ObjectRef, PrinterObject};
use klippy_core::mathutil::{gaussian_solve, mat_mat_mul, mat_transp, solve_linear_equations, Matrix};
use pycompat::text::{
    float_from_string, float_pow, float_true_div, percent_format, round_py_nd, str_repr,
    trunc_float_to_bigint, PctValue, PyError,
};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, Weak};

use crate::bulk_sensor::ClientCallback;
use crate::ldc1612::{LDC1612Api, LDC1612};
use crate::manual_probe::{create_probe_result, FinalizeCallback, ManualProbeHelper, ProbeResult};
use crate::probe::{
    lookup_minimum_z, PrinterProbeApi, ProbeCommandHelper, ProbeParameterHelper,
    ProbeParameterHelperApi, ProbeParams, ProbeResultList, ProbeSessionApi, SampleAveragingHelper,
};
use crate::trigger_analog::{
    CfgErrorKind, DigitalFilter, MCU_SosFilter, MCU_trigger_analog, TriggerAnalogSensor,
};

/// `OUT_OF_RANGE` (`klippy/extras/probe_eddy_current.py:15`).
pub const OUT_OF_RANGE: f64 = 99.9;
/// `MAX_VALID_RAW_VALUE` (`klippy/extras/probe_eddy_current.py:709`).
pub const MAX_VALID_RAW_VALUE: i64 = 0x03ffffff;
/// `sys.float_info.max` (`klippy/extras/probe_eddy_current.py:610,653,667`).
pub const FLOAT_MAX: f64 = f64::MAX;

/// One sensor measurement (`samp_time`, `freq`, `z`) as `ldc1612` decodes it
/// (`klippy/extras/ldc1612.py:190`).
pub type Sample = (f64, f64, f64);
/// A batch of sensor measurements (`probe_eddy_current.py:462`).
pub type Measures = Vec<Sample>;

/// A Python `dict` keyed by a float, with CPython's ordering and update rules:
/// insertion order is preserved and assigning an existing key keeps its
/// position. `IndexMap` cannot key on `f64` (`Eq + Hash`), so the key is the
/// bit pattern with `-0.0` folded onto `0.0` (Python treats those as equal).
pub struct F64Map<V> {
    entries: IndexMap<u64, (f64, V)>,
}

impl<V> Default for F64Map<V> {
    fn default() -> Self {
        F64Map::new()
    }
}

impl<V> F64Map<V> {
    /// An empty map.
    pub fn new() -> F64Map<V> {
        F64Map {
            entries: IndexMap::new(),
        }
    }

    fn key(key: f64) -> u64 {
        if key == 0.0 {
            0.0f64.to_bits()
        } else {
            key.to_bits()
        }
    }

    /// `d[key] = value`: replaces the value in place, keeping the position.
    pub fn insert(&mut self, key: f64, value: V) {
        self.entries.insert(Self::key(key), (key, value));
    }

    /// `len(d)`
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// `not d`
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// `d.items()`
    pub fn iter(&self) -> impl Iterator<Item = (f64, &V)> {
        self.entries.values().map(|(k, v)| (*k, v))
    }

    /// `sorted(d.items())`
    pub fn iter_sorted(&self) -> impl Iterator<Item = (f64, &V)> {
        let mut items: Vec<(f64, &V)> = self.iter().collect();
        items.sort_by(|a, b| a.0.total_cmp(&b.0));
        items.into_iter()
    }
}

/// `calc_freqs` output: per Z position, `(freq_avg, mad_hz, count)`
/// (`klippy/extras/probe_eddy_current.py:198`).
pub type CalibrationPositions = F64Map<(f64, f64, i64)>;

impl F64Map<Vec<f64>> {
    /// `d.setdefault(key, []).append(value)`
    /// (`klippy/extras/probe_eddy_current.py:181`).
    pub fn push(&mut self, key: f64, value: f64) {
        match self.entries.get_mut(&F64Map::<Vec<f64>>::key(key)) {
            Some(entry) => entry.1.push(value),
            None => {
                self.entries
                    .insert(F64Map::<Vec<f64>>::key(key), (key, vec![value]));
            }
        }
    }
}
/// One `validate_calibration_data` entry: `(pos, freq_avg, mad_hz, mad_mm)`
/// (`klippy/extras/probe_eddy_current.py:240`).
pub type FilteredPair = (f64, f64, f64, f64);
/// `TapBestFit.find_best_fit` result
/// (`klippy/extras/probe_eddy_current.py:702`).
pub type TapCoeffs = [f64; 5];
/// `EddyTap.get_last_tap_info` value: `(status, depress_dist, coeffs)`
/// (`klippy/extras/probe_eddy_current.py:862,876`).
pub type LastTap = (String, f64, TapCoeffs);

/// Analysis callback of [`EddyGatherSamples::add_probe_request`]
/// (`probe_eddy_current.py:503`): Python's `cb(measures, *args)`.
pub type ProbeRequestCallback = Arc<dyn Fn(Measures) -> KResult<ProbeResult> + Send + Sync>;

/// The drift compensation interface (`klippy/extras/probe_eddy_current.py:53`).
///
/// `temperature_probe` registers its own object through
/// [`EddyCalibrationApi::register_drift_compensation`]. Its methods can raise
/// (`get_temperature` reads a sensor, `note_z_calibration_finish` writes the
/// config), so every method is fallible.
pub trait DriftCompensationApi: Send + Sync + 'static {
    /// `get_temperature()`
    fn get_temperature(&self) -> KResult<f64>;
    /// `note_z_calibration_start()`
    fn note_z_calibration_start(&self) -> KResult<()>;
    /// `note_z_calibration_finish()`
    fn note_z_calibration_finish(&self) -> KResult<()>;
    /// `adjust_freq(freq, temp=None)`
    fn adjust_freq(&self, freq: f64, temp: Option<f64>) -> KResult<f64>;
    /// `unadjust_freq(freq, temp=None)`
    fn unadjust_freq(&self, freq: f64, temp: Option<f64>) -> KResult<f64>;
}

/// `DummyDriftCompensation` (`klippy/extras/probe_eddy_current.py:18`).
pub struct DummyDriftCompensation;

impl DriftCompensationApi for DummyDriftCompensation {
    fn get_temperature(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn note_z_calibration_start(&self) -> KResult<()> {
        Ok(())
    }
    fn note_z_calibration_finish(&self) -> KResult<()> {
        Ok(())
    }
    fn adjust_freq(&self, freq: f64, _temp: Option<f64>) -> KResult<f64> {
        Ok(freq)
    }
    fn unadjust_freq(&self, freq: f64, _temp: Option<f64>) -> KResult<f64> {
        Ok(freq)
    }
}

/// `EddyCalibration` (`klippy/extras/probe_eddy_current.py:31`).
///
/// The class is reached by `ldc1612` (`apply_calibration`) and by
/// `temperature_probe` (`register_drift_compensation`, `get_calibration`), so
/// both go through this trait.
pub trait EddyCalibrationApi: Send + Sync + 'static {
    /// `get_printer()`
    fn get_printer(&self) -> Printer;
    /// `note_z_calibration_start()`
    fn note_z_calibration_start(&self) -> KResult<()>;
    /// `note_z_calibration_finish()`
    fn note_z_calibration_finish(&self) -> KResult<()>;
    /// `register_drift_compensation(comp)`
    fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>);
    /// `verify_calibrated()`
    fn verify_calibrated(&self) -> KResult<()>;
    /// `get_calibration()`
    fn get_calibration(&self) -> (Vec<f64>, Vec<f64>);
    /// `apply_calibration(samples)` replaces each entry in place.
    fn apply_calibration(&self, samples: &mut Measures) -> KResult<()>;
    /// `freq_to_height(freq)`
    fn freq_to_height(&self, freq: f64) -> KResult<f64>;
    /// `height_to_freq(height)`
    fn height_to_freq(&self, height: f64) -> KResult<f64>;
}

/// `PrinterEddyProbe` as other modules reach it by section name
/// (`probe_eddy_current.py:146`, `temperature_probe.py:127,586`).
pub trait PrinterEddyProbeApi: Send + Sync + 'static {
    /// `add_client(cb)`
    fn add_client(&self, cb: ClientCallback) -> KResult<()>;
    /// `register_drift_compensation(comp)`
    fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>);
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

/// `EddyCalibration` (`klippy/extras/probe_eddy_current.py:31`).
pub struct EddyCalibration {
    printer: Printer,
    drift_comp: GCell<Arc<dyn DriftCompensationApi>>,
    cal_freqs: GCell<Vec<f64>>,
    cal_zpos: GCell<Vec<f64>>,
}

impl EddyCalibration {
    /// `EddyCalibration.__init__(config)`
    /// (`klippy/extras/probe_eddy_current.py:32`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let this = Arc::new(EddyCalibration {
            printer,
            drift_comp: GCell::new(Arc::new(DummyDriftCompensation) as Arc<dyn DriftCompensationApi>),
            cal_freqs: GCell::new(Vec::new()),
            cal_zpos: GCell::new(Vec::new()),
        });
        let cal = config.get(
            "calibrate",
            GetStrOpts {
                default: Some(None),
                note_valid: true,
            },
        )?;
        if let Some(cal) = cal {
            let pairs = parse_calibration(&cal)?;
            this.load_calibration(&pairs);
        }
        Ok(this)
    }

    /// `_load_calibration(cal)` (`klippy/extras/probe_eddy_current.py:43`):
    /// `cal` holds `(pos, freq)` pairs.
    pub fn load_calibration(&self, cal: &[(f64, f64)]) {
        // `sorted([(c[1], c[0]) for c in cal])`: by frequency, then position.
        let mut pairs: Vec<(f64, f64)> = cal.iter().map(|c| (c.1, c.0)).collect();
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
        *self.cal_freqs.borrow_mut() = pairs.iter().map(|p| p.0).collect();
        *self.cal_zpos.borrow_mut() = pairs.iter().map(|p| p.1).collect();
    }

    /// `get_printer()` (`klippy/extras/probe_eddy_current.py:47`).
    pub fn get_printer(&self) -> Printer {
        self.printer.clone()
    }

    /// `note_z_calibration_start()` (`klippy/extras/probe_eddy_current.py:49`).
    pub fn note_z_calibration_start(&self) -> KResult<()> {
        self.drift_comp.cloned().note_z_calibration_start()
    }

    /// `note_z_calibration_finish()` (`klippy/extras/probe_eddy_current.py:51`).
    pub fn note_z_calibration_finish(&self) -> KResult<()> {
        self.drift_comp.cloned().note_z_calibration_finish()
    }

    /// `register_drift_compensation(comp)`
    /// (`klippy/extras/probe_eddy_current.py:53`).
    pub fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>) {
        *self.drift_comp.borrow_mut() = comp;
    }

    /// `verify_calibrated()` (`klippy/extras/probe_eddy_current.py:55`).
    pub fn verify_calibrated(&self) -> KResult<()> {
        if self.cal_freqs.borrow().len() <= 2 {
            return Err(KlippyError::command(
                "Must calibrate probe_eddy_current first",
            ));
        }
        Ok(())
    }

    /// `get_calibration()` (`klippy/extras/probe_eddy_current.py:59`).
    pub fn get_calibration(&self) -> (Vec<f64>, Vec<f64>) {
        (self.cal_freqs.cloned(), self.cal_zpos.cloned())
    }

    /// `apply_calibration(samples)` (`klippy/extras/probe_eddy_current.py:61`).
    pub fn apply_calibration(&self, samples: &mut Measures) -> KResult<()> {
        let drift = self.drift_comp.cloned();
        let cur_temp = drift.get_temperature()?;
        let cal_freqs = self.cal_freqs.cloned();
        let cal_zpos = self.cal_zpos.cloned();
        for sample in samples.iter_mut() {
            let (samp_time, freq, _) = *sample;
            let adj_freq = drift.adjust_freq(freq, Some(cur_temp))?;
            let pos = bisect_right(&cal_freqs, adj_freq);
            let zpos = if pos >= cal_zpos.len() {
                -OUT_OF_RANGE
            } else if pos == 0 {
                OUT_OF_RANGE
            } else {
                // XXX - could further optimize and avoid div by zero
                let this_freq = cal_freqs[pos];
                let prev_freq = cal_freqs[pos - 1];
                let this_zpos = cal_zpos[pos];
                let prev_zpos = cal_zpos[pos - 1];
                let gain =
                    float_true_div(this_zpos - prev_zpos, this_freq - prev_freq).map_err(py_error)?;
                let offset = prev_zpos - prev_freq * gain;
                adj_freq * gain + offset
            };
            *sample = (samp_time, freq, round_py_nd(zpos, 6));
        }
        Ok(())
    }

    /// `freq_to_height(freq)` (`klippy/extras/probe_eddy_current.py:80`).
    pub fn freq_to_height(&self, freq: f64) -> KResult<f64> {
        let mut dummy_sample: Measures = vec![(0.0, freq, 0.0)];
        self.apply_calibration(&mut dummy_sample)?;
        Ok(dummy_sample[0].2)
    }

    /// `height_to_freq(height)` (`klippy/extras/probe_eddy_current.py:84`).
    pub fn height_to_freq(&self, height: f64) -> KResult<f64> {
        // XXX - could optimize lookup
        let mut rev_zpos = self.cal_zpos.cloned();
        rev_zpos.reverse();
        let mut rev_freqs = self.cal_freqs.cloned();
        rev_freqs.reverse();
        let pos = bisect_right(&rev_zpos, height);
        if pos == 0 || pos >= rev_zpos.len() {
            return Err(KlippyError::command("Invalid probe_eddy_current height"));
        }
        let this_freq = rev_freqs[pos];
        let prev_freq = rev_freqs[pos - 1];
        let this_zpos = rev_zpos[pos];
        let prev_zpos = rev_zpos[pos - 1];
        let gain = float_true_div(this_freq - prev_freq, this_zpos - prev_zpos).map_err(py_error)?;
        let offset = prev_freq - prev_zpos * gain;
        let freq = height * gain + offset;
        self.drift_comp.cloned().unadjust_freq(freq, None)
    }
}

impl EddyCalibrationApi for EddyCalibration {
    fn get_printer(&self) -> Printer {
        EddyCalibration::get_printer(self)
    }
    fn note_z_calibration_start(&self) -> KResult<()> {
        EddyCalibration::note_z_calibration_start(self)
    }
    fn note_z_calibration_finish(&self) -> KResult<()> {
        EddyCalibration::note_z_calibration_finish(self)
    }
    fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>) {
        EddyCalibration::register_drift_compensation(self, comp)
    }
    fn verify_calibrated(&self) -> KResult<()> {
        EddyCalibration::verify_calibrated(self)
    }
    fn get_calibration(&self) -> (Vec<f64>, Vec<f64>) {
        EddyCalibration::get_calibration(self)
    }
    fn apply_calibration(&self, samples: &mut Measures) -> KResult<()> {
        EddyCalibration::apply_calibration(self, samples)
    }
    fn freq_to_height(&self, freq: f64) -> KResult<f64> {
        EddyCalibration::freq_to_height(self, freq)
    }
    fn height_to_freq(&self, height: f64) -> KResult<f64> {
        EddyCalibration::height_to_freq(self, height)
    }
}

/// `EddyCalibrationTool.cmd_EDDY_CALIBRATE_help`
/// (`klippy/extras/probe_eddy_current.py:295`).
pub const CMD_EDDY_CALIBRATE_HELP: &str = "Calibrate eddy current probe";
/// `EddyCalibrationTool.cmd_Z_OFFSET_APPLY_PROBE_help`
/// (`klippy/extras/probe_eddy_current.py:315`).
pub const CMD_Z_OFFSET_APPLY_PROBE_HELP: &str = "Adjust the probe's z_offset";
/// `EddyTapCalibration.cmd_TAP_CALIBRATE_help`
/// (`klippy/extras/probe_eddy_current.py:401`).
pub const CMD_TAP_CALIBRATE_HELP: &str = "Calibrate tap_threshold for 'tap' probing";

/// `EddyCalibrationTool` (`klippy/extras/probe_eddy_current.py:102`).
pub struct EddyCalibrationTool {
    printer: Printer,
    name: String,
    calibration: Arc<dyn EddyCalibrationApi>,
    probe_speed: GCell<f64>,
}

impl EddyCalibrationTool {
    /// `EddyCalibrationTool.__init__(config, calibration)`
    /// (`klippy/extras/probe_eddy_current.py:103`).
    pub fn new(config: &Config, calibration: Arc<dyn EddyCalibrationApi>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let cname = name.split_whitespace().last().unwrap_or_default().to_string();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = Arc::new(EddyCalibrationTool {
            printer: printer.clone(),
            name,
            calibration,
            probe_speed: GCell::new(0.0),
        });
        let this2 = Arc::clone(&this);
        gcode.register_mux_command(
            "PROBE_EDDY_CURRENT_CALIBRATE",
            "CHIP",
            Some(cname.as_str()),
            Box::new(move |gcmd| Self::cmd_eddy_calibrate(&this2, gcmd)),
            Some(CMD_EDDY_CALIBRATE_HELP),
        )?;
        let this3 = Arc::clone(&this);
        gcode.register_command(
            "Z_OFFSET_APPLY_PROBE",
            Some(Box::new(move |gcmd| this3.cmd_z_offset_apply_probe(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_Z_OFFSET_APPLY_PROBE_HELP.to_string()),
            },
        )?;
        Ok(this)
    }

    /// `_save_calibration(z_freq_pairs)`
    /// (`klippy/extras/probe_eddy_current.py:118`).
    pub fn save_calibration(&self, z_freq_pairs: &[(f64, f64)]) -> KResult<()> {
        let gcode =
            lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
                .expect("Raise never yields None");
        gcode.respond_info(
            "The SAVE_CONFIG command will update the printer config file\nand restart the printer.",
            true,
        );
        // Save results
        let mut cal_contents: Vec<String> = Vec::new();
        for (i, pair) in z_freq_pairs.iter().enumerate() {
            if i % 3 == 0 {
                cal_contents.push("\n".to_string());
            }
            cal_contents.push(format_f(
                "%.6f:%.3f",
                &[PctValue::Float(pair.0), PctValue::Float(pair.1)],
            )?);
            cal_contents.push(",".to_string());
        }
        cal_contents.pop().ok_or_else(pop_from_empty)?;
        let configfile =
            lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
                .expect("Raise never yields None");
        configfile.set(&self.name, "calibrate", &cal_contents.join(""))?;
        Ok(())
    }

    /// `do_calibration_moves(move_speed)`
    /// (`klippy/extras/probe_eddy_current.py:134`).
    pub fn do_calibration_moves(&self, move_speed: f64) -> KResult<F64Map<Vec<f64>>> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let kin = toolhead.get_kinematics();
        // Start data collection
        let msgs = Arc::new(GCell::new(Vec::<PyValue>::new()));
        let is_finished = Arc::new(GCell::new(false));
        let msgs_cb = Arc::clone(&msgs);
        let finished_cb = Arc::clone(&is_finished);
        let handle_batch: ClientCallback = Box::new(move |msg: PyValue| -> KResult<bool> {
            if finished_cb.get() {
                return Ok(false);
            }
            msgs_cb.borrow_mut().push(msg);
            Ok(true)
        });
        let probe_client = lookup_interface::<dyn PrinterEddyProbeApi>(
            &*self.printer,
            &self.name,
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        probe_client.add_client(handle_batch)?;
        toolhead.dwell(1.0)?;
        self.calibration.note_z_calibration_start()?;
        // Move to each 40um position
        let max_z = 4.0;
        let samp_dist = 0.040;
        let count = py_trunc_i64(float_true_div(max_z, samp_dist).map_err(py_error)?)?;
        let start_pos = toolhead.get_position();
        let mut times: Vec<(f64, f64, f64)> = Vec::new();
        for i in 0..=count {
            let zpos = (i as f64) * samp_dist;
            // Move to next position (always descending to reduce backlash)
            let mut hop_pos = start_pos.clone();
            hop_pos[2] += zpos + 0.500;
            toolhead.manual_move(opt_vec(&hop_pos), move_speed)?;
            let mut next_pos = start_pos.clone();
            next_pos[2] += zpos;
            toolhead.manual_move(opt_vec(&next_pos), move_speed)?;
            // Note sample timing
            let start_query_time = toolhead.get_last_move_time()? + 0.050;
            let end_query_time = start_query_time + 0.100;
            toolhead.dwell(0.200)?;
            // Find Z position based on actual commanded stepper position
            toolhead.flush_step_generation()?;
            let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
            for stepper in kin.get_steppers() {
                kin_spos.insert(stepper.get_name(Some(false)), stepper.get_commanded_position());
            }
            let kin_pos = kin.calc_position(&kin_spos)?;
            let kin_z = opt_f64(kin_pos.get(2).copied().flatten())?;
            times.push((start_query_time, end_query_time, kin_z));
        }
        toolhead.dwell(1.0)?;
        toolhead.wait_moves()?;
        self.calibration.note_z_calibration_finish()?;
        // Finish data collection
        is_finished.set(true);
        // Correlate query responses
        let mut cal: F64Map<Vec<f64>> = F64Map::new();
        let mut step = 0usize;
        for msg in msgs.cloned() {
            for (query_time, freq, _old_z) in msg_data(&msg)? {
                // Add to step tracking
                while step < times.len() && query_time > times[step].1 {
                    step += 1;
                }
                if step < times.len() && query_time >= times[step].0 {
                    cal.push(times[step].2, freq);
                }
            }
        }
        if cal.len() != times.len() {
            return Err(KlippyError::command(
                "Failed calibration - incomplete sensor data",
            ));
        }
        Ok(cal)
    }

    /// `_median(values)` (`klippy/extras/probe_eddy_current.py:192`).
    pub fn median(&self, values: &[f64]) -> KResult<f64> {
        let mut values = values.to_vec();
        // Python sorts with float comparisons; NaN ordering is unspecified
        // there, so the total order is used.
        values.sort_by(|a, b| a.total_cmp(b));
        let n = values.len();
        if n.is_multiple_of(2) {
            let left = (n / 2).checked_sub(1).ok_or_else(index_error)?;
            let a = *values.get(left).ok_or_else(index_error)?;
            let b = *values.get(n / 2).ok_or_else(index_error)?;
            float_true_div(a + b, 2.0).map_err(py_error)
        } else {
            Ok(*values.get(n / 2).ok_or_else(index_error)?)
        }
    }

    /// `calc_freqs(meas)` (`klippy/extras/probe_eddy_current.py:198`).
    pub fn calc_freqs(&self, meas: &F64Map<Vec<f64>>) -> KResult<CalibrationPositions> {
        let mut positions: CalibrationPositions = F64Map::new();
        for (pos, freqs) in meas.iter() {
            let count = freqs.len() as i64;
            let freq_avg = float_true_div(py_sum(freqs), count as f64).map_err(py_error)?;
            let mads: Vec<f64> = freqs.iter().map(|f| (f - freq_avg).abs()).collect();
            let mad = self.median(&mads)?;
            positions.insert(pos, (freq_avg, mad, count));
        }
        Ok(positions)
    }

    /// `validate_calibration_data(positions)`
    /// (`klippy/extras/probe_eddy_current.py:207`).
    pub fn validate_calibration_data(
        &self,
        positions: &CalibrationPositions,
    ) -> KResult<Vec<FilteredPair>> {
        let gcode =
            lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let mut last_freq = 40000000.0;
        let mut last_pos = 0.0;
        let mut last_mad = 0.0;
        let mut filtered: Vec<FilteredPair> = Vec::new();
        let mut mad_hz_total = 0.0;
        let mut mad_mm_total = 0.0;
        let mut samples_count: i64 = 0;
        for (pos, (freq_avg, mad_hz, count)) in positions.iter_sorted() {
            let (freq_avg, mad_hz, count) = (*freq_avg, *mad_hz, *count);
            if freq_avg > last_freq {
                gcode.respond_info(
                    &format_f(
                        "Frequency stops decreasing at step %.3f",
                        &[PctValue::Float(pos)],
                    )?,
                    true,
                );
                break;
            }
            let diff_mad = libm_sqrt(
                float_pow(last_mad, 2.0).map_err(py_error)?
                    + float_pow(mad_hz, 2.0).map_err(py_error)?,
            );
            // Calculate if samples have a significant difference
            let freq_diff = last_freq - freq_avg;
            last_freq = freq_avg;
            if freq_diff < 2.5 * diff_mad {
                gcode.respond_info(
                    &format_f(
                        "Frequency too noisy at step %.3f -> %.3f",
                        &[PctValue::Float(last_pos), PctValue::Float(pos)],
                    )?,
                    true,
                );
                gcode.respond_info(
                    &format_f(
                        "Frequency diff: %.3f, MAD_Hz: %.3f -> MAD_Hz: %.3f",
                        &[
                            PctValue::Float(freq_diff),
                            PctValue::Float(last_mad),
                            PctValue::Float(mad_hz),
                        ],
                    )?,
                    true,
                );
                break;
            }
            last_mad = mad_hz;
            let delta_dist = pos - last_pos;
            last_pos = pos;
            // MAD is Median Absolute Deviation to Frequency avg ~ delta_hz_1
            // Signal is delta_hz_2 / delta_dist
            // SNR ~= delta_hz_1 / (delta_hz_2 / delta_mm) = d_1 * d_mm / d_2
            let mad_mm = float_true_div(mad_hz * delta_dist, freq_diff).map_err(py_error)?;
            filtered.push((pos, freq_avg, mad_hz, mad_mm));
            mad_hz_total += mad_hz;
            mad_mm_total += mad_mm;
            samples_count += count;
        }
        let count = filtered.len() as f64;
        let avg_mad = float_true_div(mad_hz_total, count).map_err(py_error)?;
        let avg_mad_mm = float_true_div(mad_mm_total, count).map_err(py_error)?;
        gcode.respond_info(
            &format_f(
                "probe_eddy_current: noise %.6fmm, MAD_Hz=%.3f in %d queries\n",
                &[
                    PctValue::Float(avg_mad_mm),
                    PctValue::Float(avg_mad),
                    PctValue::Int(PyInt::from_i64(samples_count)),
                ],
            )?,
            true,
        );
        let freq_list: Vec<f64> = filtered.iter().map(|e| e.1).collect();
        let freq_diff = py_max(&freq_list)? - py_min(&freq_list)?;
        gcode.respond_info(
            &format_f("Total frequency range: %.3f Hz\n", &[PctValue::Float(freq_diff)])?,
            true,
        );
        let mut points: Vec<f64> = vec![0.25, 0.5, 1.0, 2.0, 3.0];
        for (pos, _, mad_hz, mad_mm) in &filtered {
            if !points.is_empty() && points[0] <= *pos {
                points.remove(0);
                gcode.respond_info(
                    &format_f(
                        "z: %.3f # noise %.6fmm, MAD_Hz=%.3f\n",
                        &[
                            PctValue::Float(*pos),
                            PctValue::Float(*mad_mm),
                            PctValue::Float(*mad_hz),
                        ],
                    )?,
                    true,
                );
            }
        }
        Ok(filtered)
    }

    /// `post_manual_probe(mpresult)`
    /// (`klippy/extras/probe_eddy_current.py:260`).
    pub fn post_manual_probe(&self, mpresult: Option<ProbeResult>) -> KResult<()> {
        let Some(mpresult) = mpresult else {
            // Manual Probe was aborted
            return Ok(());
        };
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let probe_speed = self.probe_speed.get();
        let mut curpos = vec![mpresult.bed_x, mpresult.bed_y, mpresult.bed_z];
        // Move away from the bed
        let probe_calibrate_z = curpos[2];
        curpos[2] += 5.0;
        toolhead.manual_move(opt_vec(&curpos), probe_speed)?;
        // Move sensor over nozzle position
        let (x_offset, y_offset, _z_offset) = self.probe_offsets()?;
        curpos[0] -= x_offset;
        curpos[1] -= y_offset;
        toolhead.manual_move(opt_vec(&curpos), probe_speed)?;
        // Descend back to bed
        curpos[2] -= 5.0 - 0.050;
        toolhead.manual_move(opt_vec(&curpos), probe_speed)?;
        // Perform calibration movement and capture
        let cal = self.do_calibration_moves(probe_speed)?;
        // Calculate each sample position average and variance
        let raw_positions = self.calc_freqs(&cal)?;
        // Fix Z position offset
        let mut positions: CalibrationPositions = F64Map::new();
        for (k, v) in raw_positions.iter() {
            positions.insert(k - probe_calibrate_z, *v);
        }
        let filtered = self.validate_calibration_data(&positions)?;
        if filtered.len() <= 8 {
            return Err(KlippyError::command(
                "Failed calibration - No usable data",
            ));
        }
        let z_freq_pairs: Vec<(f64, f64)> = filtered
            .iter()
            .map(|(pos, freq, _, _)| (*pos, *freq))
            .collect();
        self.save_calibration(&z_freq_pairs)
    }

    /// `self.printer.lookup_object("probe").get_offsets()`
    /// (`klippy/extras/probe_eddy_current.py:272`).
    pub fn probe_offsets(&self) -> KResult<(f64, f64, f64)> {
        let pprobe =
            lookup_interface::<dyn PrinterProbeApi>(&*self.printer, "probe", LookupDefault::Raise)?
                .expect("Raise never yields None");
        pprobe.get_offsets(None)
    }

    /// `cmd_EDDY_CALIBRATE(gcmd)` (`klippy/extras/probe_eddy_current.py:296`).
    pub fn cmd_eddy_calibrate(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        this.probe_speed.set(
            gcmd.get_float(
                "PROBE_SPEED",
                GcmdFloatOpts {
                    default: Some(Some(5.0)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given"),
        );
        // Start manual probe
        let callback: FinalizeCallback = {
            let this = Arc::clone(this);
            Box::new(move |mpresult| this.post_manual_probe(mpresult))
        };
        ManualProbeHelper::new(&this.printer, gcmd, callback)?;
        Ok(())
    }

    /// `_save_tap_z_offset(gcmd, homing_z)`
    /// (`klippy/extras/probe_eddy_current.py:302`).
    pub fn save_tap_z_offset(&self, gcmd: &GCmd, homing_z: f64) -> KResult<()> {
        let eventtime = self.printer.get_reactor().monotonic();
        let configfile =
            lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let cstatus = configfile.get_status(Some(eventtime));
        let csettings = py_dict_get_or(&cstatus, "settings")
            .and_then(|settings| py_dict_get_or(settings, &self.name))
            .cloned()
            .unwrap_or(PyValue::Dict(IndexMap::new()));
        let tap_z_offset = match py_dict_get_or(&csettings, "tap_z_offset") {
            Some(v) => pyval_float(v)?,
            None => 0.0,
        };
        let new_calibrate = tap_z_offset - homing_z;
        gcmd.respond_info(
            &format_f(
                "%s: tap_z_offset: %.3f\n\
                 The SAVE_CONFIG command will update the printer config file\n\
                 with the above and restart the printer.",
                &[
                    PctValue::Str(self.name.clone()),
                    PctValue::Float(new_calibrate),
                ],
            )?,
            true,
        );
        configfile.set(
            &self.name,
            "tap_z_offset",
            &format_f("%.3f", &[PctValue::Float(new_calibrate)])?,
        )?;
        Ok(())
    }

    /// `cmd_Z_OFFSET_APPLY_PROBE(gcmd)`
    /// (`klippy/extras/probe_eddy_current.py:316`).
    pub fn cmd_z_offset_apply_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let gcode_move =
            lookup_interface::<dyn GCodeMoveApi>(&*self.printer, "gcode_move", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let status = gcode_move.get_status(None);
        let offset = py_dict_get(&status, "homing_origin")
            .and_then(|origin| pyval_float(&coord_z(origin)?))?;
        if offset == 0.0 {
            gcmd.respond_info("Nothing to do: Z Offset is 0", true);
            return Ok(());
        }
        let method = gcmd
            .get(
                "METHOD",
                GcmdGetOpts {
                    default: Some(PyValue::Str(String::new())),
                    ..GcmdGetOpts::default()
                },
            )?
            .str()
            .to_lowercase();
        if method == "tap" {
            return self.save_tap_z_offset(gcmd, offset);
        }
        let (cal_freqs, cal_zpos) = self.calibration.get_calibration();
        let mut z_freq_pairs: Vec<(f64, f64)> = cal_zpos
            .iter()
            .map(|z| z - offset)
            .zip(cal_freqs.iter().copied())
            .collect();
        z_freq_pairs.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
        self.save_calibration(&z_freq_pairs)
    }
}

/// `EddyTapCalibration` (`klippy/extras/probe_eddy_current.py:332`).
pub struct EddyTapCalibration {
    printer: Printer,
    name: String,
    calibration: Arc<dyn EddyCalibrationApi>,
    eddy_tap: Arc<EddyTap>,
    refine_tap_threshold: GCell<Option<f64>>,
}

impl EddyTapCalibration {
    /// `EddyTapCalibration.__init__(config, calibration, eddy_tap)`
    /// (`klippy/extras/probe_eddy_current.py:333`).
    pub fn new(
        config: &Config,
        calibration: Arc<dyn EddyCalibrationApi>,
        eddy_tap: Arc<EddyTap>,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = Arc::new(EddyTapCalibration {
            printer: printer.clone(),
            name,
            calibration,
            eddy_tap,
            refine_tap_threshold: GCell::new(None),
        });
        let this2 = Arc::clone(&this);
        gcode.register_command(
            "PROBE_EDDY_CURRENT_TAP_CALIBRATE",
            Some(Box::new(move |gcmd| this2.cmd_tap_calibrate(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_TAP_CALIBRATE_HELP.to_string()),
            },
        )?;
        Ok(this)
    }

    /// `_analyze_main_calibration()`
    /// (`klippy/extras/probe_eddy_current.py:343`).
    pub fn analyze_main_calibration(&self) -> KResult<Option<Matrix>> {
        let (freqs, zpos) = self.calibration.get_calibration();
        if freqs.len() < 2 {
            return Ok(None);
        }
        // Find best fit for: freq = c0 + c1*z + c2*z*z
        let mut eqs: Matrix = Vec::new();
        let mut ans: Matrix = Vec::new();
        for (freq, z) in freqs.iter().zip(zpos.iter()) {
            if *z <= 0.750 {
                ans.push(vec![*freq]);
                eqs.push(vec![1.0, *z, *z * *z]);
            }
        }
        solve_linear_equations(&eqs, &ans, false)
    }

    /// `_describe_main_calibration(coeffs)`
    /// (`klippy/extras/probe_eddy_current.py:355`).
    pub fn describe_main_calibration(&self, coeffs: &Option<Matrix>) -> KResult<Vec<String>> {
        let Some(coeffs) = coeffs else {
            return Ok(vec![
                "Main calibration data not available.".to_string(),
                String::new(),
            ]);
        };
        let msg = format_f(
            "Calibration: f=%.3f s=%.3f q=%.3f",
            &[
                PctValue::Float(m_get(coeffs, 0, 0)?),
                PctValue::Float(m_get(coeffs, 1, 0)?),
                PctValue::Float(m_get(coeffs, 2, 0)?),
            ],
        )?;
        Ok(vec![msg, String::new()])
    }

    /// `_describe_last_tap(last_tap)`
    /// (`klippy/extras/probe_eddy_current.py:361`).
    pub fn describe_last_tap(&self, last_tap: &Option<LastTap>) -> KResult<Vec<String>> {
        let Some((status, depress_dist, coeffs)) = last_tap else {
            return Ok(vec!["Run tap probe for last tap analysis.".to_string()]);
        };
        let [z_contact, freq_contact, depress_slope, slope, slope2] = *coeffs;
        let contact_slope_delta = depress_slope - slope;
        let m1 = format_f(
            "Last tap: z=%.6f f=%.3f s=%.3f q=%.3f",
            &[
                PctValue::Float(z_contact),
                PctValue::Float(freq_contact),
                PctValue::Float(slope),
                PctValue::Float(slope2),
            ],
        )?;
        let m2 = format_f(
            "  depress_dist=%.6f depress_slope=%.3f",
            &[
                PctValue::Float(*depress_dist),
                PctValue::Float(depress_slope),
            ],
        )?;
        let m3 = format_f(
            "  contact_slope_delta=%.3f",
            &[PctValue::Float(contact_slope_delta)],
        )?;
        let mut msgs = vec![m1, m2, m3];
        if status != "success" {
            msgs.extend(["".to_string(), "Warning! Last tap did not succeed.".to_string()]);
        }
        Ok(msgs)
    }

    /// `_try_tap(gcmd, tap_threshold, samples=1)`
    /// (`klippy/extras/probe_eddy_current.py:376`).
    pub fn try_tap(&self, gcmd: &GCmd, tap_threshold: f64, samples: i64) -> KResult<()> {
        // Create dummy gcmd with SAMPLES=1
        let mut fo_params = gcmd.get_command_parameters();
        fo_params.insert("METHOD".to_string(), "tap".to_string());
        let threshold_str = format_f("%.3f", &[PctValue::Float(tap_threshold)])?;
        fo_params.insert("TAP_THRESHOLD".to_string(), threshold_str.clone());
        let samples_str = samples.to_string();
        fo_params.insert("SAMPLES".to_string(), samples_str.clone());
        let gcode =
            lookup_interface::<dyn GCodeApi>(&*self.printer, "gcode", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let fo_gcmd = gcode.create_gcode_command("", "", &fo_params);
        gcmd.respond_info(
            &format_f(
                "Tap probing with TAP_THRESHOLD=%s SAMPLES=%s",
                &[
                    PctValue::Str(threshold_str),
                    PctValue::Str(samples_str),
                ],
            )?,
            true,
        );
        // Run "tap" probe
        let probe =
            lookup_interface::<dyn PrinterProbeApi>(&*self.printer, "probe", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let probe_session = probe.start_probe_session(&fo_gcmd)?;
        probe_session.run_probe(&fo_gcmd)?;
        let positions = probe_session.pull_probed_results()?;
        probe_session.end_probe_session()?;
        let first = positions.first().ok_or_else(index_error)?;
        gcmd.respond_info(
            &format_f("Tap probing reports z=%.6f", &[PctValue::Float(first.bed_z)])?,
            true,
        );
        Ok(())
    }

    /// `_save_tap_threshold(gcmd, tap_threshold)`
    /// (`klippy/extras/probe_eddy_current.py:393`).
    pub fn save_tap_threshold(&self, gcmd: &GCmd, tap_threshold: f64) -> KResult<()> {
        let configfile =
            lookup_interface::<dyn PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
                .expect("Raise never yields None");
        gcmd.respond_info(
            &format_f(
                "%s: tap_threshold: %.3f\n\
                 The SAVE_CONFIG command will update the printer config file\n\
                 with the above and restart the printer.",
                &[
                    PctValue::Str(self.name.clone()),
                    PctValue::Float(tap_threshold),
                ],
            )?,
            true,
        );
        configfile.set(
            &self.name,
            "tap_threshold",
            &format_f("%.3f", &[PctValue::Float(tap_threshold)])?,
        )?;
        Ok(())
    }

    /// `cmd_TAP_CALIBRATE(gcmd)`
    /// (`klippy/extras/probe_eddy_current.py:402`).
    pub fn cmd_tap_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let mc_coeffs = self.analyze_main_calibration()?;
        let last_tap = self.eddy_tap.get_last_tap_info();
        let tap_test = gcmd.get(
            "TAP",
            GcmdGetOpts {
                default: Some(PyValue::None),
                ..GcmdGetOpts::default()
            },
        )?;
        match &tap_test {
            PyValue::None => {
                // Provide technical information
                let mut msgs = self.describe_main_calibration(&mc_coeffs)?;
                msgs.extend(self.describe_last_tap(&last_tap)?);
                gcmd.respond_info(&msgs.join("\n"), true);
            }
            PyValue::Str(tap_test) if tap_test == "guess" => {
                // Attempt tap based on main calibration
                self.refine_tap_threshold.set(None);
                let Some(coeffs) = &mc_coeffs else {
                    return Err(KlippyError::command(
                        "Must complete PROBE_EDDY_CURRENT_CALIBRATE first",
                    ));
                };
                self.try_tap(gcmd, m_get(coeffs, 1, 0)? * -0.10, 1)?;
            }
            PyValue::Str(tap_test) if tap_test == "refine" => {
                // Attempt tap based on change in slope observed during last tap
                self.refine_tap_threshold.set(None);
                match &last_tap {
                    Some((status, _, coeffs)) if status == "success" => {
                        let [_, _, depress_slope, slope, _] = *coeffs;
                        let contact_slope_delta = depress_slope - slope;
                        let try_tap_threshold = contact_slope_delta * 0.20;
                        self.try_tap(gcmd, try_tap_threshold, 1)?;
                        self.refine_tap_threshold.set(Some(try_tap_threshold));
                    }
                    _ => {
                        return Err(KlippyError::command("Must complete valid 'tap' probe first"));
                    }
                }
            }
            PyValue::Str(tap_test) if tap_test == "verify" => {
                // Retry tap several times to verify it is stable
                let Some(refine) = self.refine_tap_threshold.get() else {
                    return Err(KlippyError::command("Must complete valid 'refine' step first"));
                };
                self.try_tap(gcmd, refine, 5)?;
                self.save_tap_threshold(gcmd, refine)?;
            }
            _ => {
                return Err(KlippyError::command("Please provide a valid TAP parameter"));
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Measurement collection
// ---------------------------------------------------------------------------

/// `EddyGatherSamples` (`klippy/extras/probe_eddy_current.py:444`).
pub struct EddyGatherSamples {
    printer: Printer,
    sensor_helper: Arc<dyn LDC1612Api>,
    sensor_messages: GCell<Vec<PyValue>>,
    need_stop: GCell<bool>,
    probe_requests: GCell<Vec<(ProbeRequestCallback, f64, f64)>>,
    analysis_results: GCell<Vec<(Option<ProbeResult>, Option<String>)>>,
}

impl EddyGatherSamples {
    /// `EddyGatherSamples.__init__(printer, sensor_helper)`
    /// (`klippy/extras/probe_eddy_current.py:445`).
    pub fn new(printer: Printer, sensor_helper: Arc<dyn LDC1612Api>) -> KResult<Arc<Self>> {
        let this = Arc::new(EddyGatherSamples {
            printer,
            sensor_helper: Arc::clone(&sensor_helper),
            sensor_messages: GCell::new(Vec::new()),
            need_stop: GCell::new(false),
            probe_requests: GCell::new(Vec::new()),
            analysis_results: GCell::new(Vec::new()),
        });
        // Start samples
        let this2 = Arc::clone(&this);
        sensor_helper.add_client(Box::new(move |msg| this2.add_sensor_message(msg)))?;
        Ok(this)
    }

    /// `_add_sensor_message(msg)` (`klippy/extras/probe_eddy_current.py:457`).
    pub fn add_sensor_message(&self, msg: PyValue) -> KResult<bool> {
        if self.need_stop.get() {
            self.sensor_messages.borrow_mut().clear();
            return Ok(false);
        }
        self.sensor_messages.borrow_mut().push(msg);
        self.check_sensor_messages()?;
        Ok(true)
    }

    /// `finish()` (`klippy/extras/probe_eddy_current.py:464`).
    pub fn finish(&self) {
        self.need_stop.set(true);
    }

    /// `_pull_measurements(start_time, end_time)`
    /// (`klippy/extras/probe_eddy_current.py:466`).
    pub fn pull_measurements(&self, start_time: f64, end_time: f64) -> KResult<Measures> {
        // Extract measurements from sensor messages for given time range
        let mut measures: Measures = Vec::new();
        let mut msg_num = 0usize;
        let mut discard_msgs = 0usize;
        while msg_num < self.sensor_messages.borrow().len() {
            let data = {
                let messages = self.sensor_messages.borrow();
                msg_data(&messages[msg_num])?
            };
            msg_num += 1;
            if data.first().ok_or_else(index_error)?.0 > end_time {
                break;
            }
            if data.last().ok_or_else(index_error)?.0 < start_time {
                discard_msgs = msg_num;
                continue;
            }
            for measure in data {
                let time = measure.0;
                if time < start_time {
                    continue;
                }
                if time > end_time {
                    break;
                }
                measures.push(measure);
            }
        }
        self.sensor_messages.borrow_mut().drain(0..discard_msgs);
        Ok(measures)
    }

    /// `_check_sensor_messages()` (`klippy/extras/probe_eddy_current.py:488`).
    pub fn check_sensor_messages(&self) -> KResult<()> {
        while !self.sensor_messages.borrow().is_empty() && !self.probe_requests.borrow().is_empty() {
            let (cb, start_time, end_time) = self.probe_requests.borrow()[0].clone();
            let last_time = {
                let messages = self.sensor_messages.borrow();
                let data = msg_data(&messages[messages.len() - 1])?;
                data.last().ok_or_else(index_error)?.0
            };
            if last_time < end_time {
                break;
            }
            let measures = self.pull_measurements(start_time, end_time)?;
            // Call the analysis callback to process the measurements. A
            // `command_error` is deferred to `pull_probed()`.
            let (res, errmsg) = match cb(measures) {
                Ok(res) => (Some(res), None),
                Err(KlippyError::Command(msg)) => (None, Some(msg)),
                Err(e) => return Err(e),
            };
            self.analysis_results.borrow_mut().push((res, errmsg));
            self.probe_requests.borrow_mut().remove(0);
        }
        Ok(())
    }

    /// `add_probe_request(cb, start_time, end_time, *args)`
    /// (`klippy/extras/probe_eddy_current.py:503`).
    pub fn add_probe_request(
        &self,
        cb: ProbeRequestCallback,
        start_time: f64,
        end_time: f64,
    ) -> KResult<()> {
        self.probe_requests
            .borrow_mut()
            .push((cb, start_time, end_time));
        self.check_sensor_messages()
    }

    /// `_await_sensor_messages()` (`klippy/extras/probe_eddy_current.py:507`).
    pub fn await_sensor_messages(&self) -> KResult<()> {
        // Make sure enough samples have been collected
        let reactor = self.printer.get_reactor();
        let mcu = self.sensor_helper.get_mcu();
        while !self.probe_requests.borrow().is_empty() {
            let (_, _, end_time) = self.probe_requests.borrow()[0].clone();
            let systime = reactor.monotonic();
            let est_print_time = mcu.estimated_print_time(systime);
            if est_print_time > end_time + 1.0 {
                return Err(KlippyError::command("probe_eddy_current sensor outage"));
            }
            if mcu.is_fileoutput() {
                // In debugging mode - just create dummy response
                let dummy_pr = create_probe_result(&[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
                self.analysis_results
                    .borrow_mut()
                    .push((Some(dummy_pr), None));
                self.probe_requests.borrow_mut().remove(0);
                continue;
            }
            reactor.pause(systime + 0.010);
        }
        Ok(())
    }

    /// `pull_probed()` (`klippy/extras/probe_eddy_current.py:525`).
    pub fn pull_probed(&self) -> KResult<Vec<ProbeResult>> {
        self.await_sensor_messages()?;
        let mut results: Vec<ProbeResult> = Vec::new();
        {
            let pending = self.analysis_results.borrow();
            for (res, errmsg) in pending.iter() {
                if let Some(errmsg) = errmsg {
                    return Err(KlippyError::command(errmsg.clone()));
                }
                results.push(res.expect("every entry is either a result or an error"));
            }
        }
        self.analysis_results.borrow_mut().clear();
        Ok(results)
    }
}

/// `probe_results_from_avg(measures, toolhead_pos, calibration, offsets)`
/// (`klippy/extras/probe_eddy_current.py:536`).
pub fn probe_results_from_avg(
    measures: &Measures,
    toolhead_pos: &[f64],
    calibration: &dyn EddyCalibrationApi,
    offsets: (f64, f64, f64),
) -> KResult<ProbeResult> {
    if measures.is_empty() {
        return Err(KlippyError::command(
            "Unable to obtain probe_eddy_current sensor readings",
        ));
    }
    // Determine average of measurements
    let freq_sum = measures.iter().fold(0.0, |sum, measure| sum + measure.1);
    let freq_avg = float_true_div(freq_sum, measures.len() as f64).map_err(py_error)?;
    // Determine height associated with frequency
    let sensor_z = calibration.freq_to_height(freq_avg)?;
    if sensor_z <= -OUT_OF_RANGE || sensor_z >= OUT_OF_RANGE {
        return Err(KlippyError::command(
            "probe_eddy_current sensor not in valid range",
        ));
    }
    Ok(create_probe_result(
        toolhead_pos,
        [offsets.0, offsets.1, sensor_z],
    ))
}

// ---------------------------------------------------------------------------
// Data fitting for "tap"
// ---------------------------------------------------------------------------

/// `TapBestFit` (`klippy/extras/probe_eddy_current.py:562`).
pub struct TapBestFit {
    least_squares_cache: GCell<IndexMap<usize, [f64; 11]>>,
}

impl Default for TapBestFit {
    fn default() -> Self {
        TapBestFit::new()
    }
}

impl TapBestFit {
    /// `TapBestFit.__init__()` (`klippy/extras/probe_eddy_current.py:563`).
    pub fn new() -> TapBestFit {
        TapBestFit {
            least_squares_cache: GCell::new(IndexMap::new()),
        }
    }

    /// `_build_ls_matrix(samples, est_z_contact)`
    /// (`klippy/extras/probe_eddy_current.py:565`): the reference version of
    /// the optimized matrix below; Python keeps it as documentation.
    pub fn build_ls_matrix(
        &self,
        samples: &[(f64, f64)],
        est_z_contact: f64,
    ) -> KResult<(Matrix, Matrix)> {
        let len_samples = samples.len();
        let mut eqs: Matrix = vec![vec![0.0; 4]; len_samples];
        let mut ans: Matrix = vec![vec![0.0]; len_samples];
        for (i, (step_z, sensor_freq)) in samples.iter().enumerate() {
            ans[i][0] = *sensor_freq;
            let eq = &mut eqs[i];
            eq[0] = 1.0;
            if *step_z <= est_z_contact {
                // 1*c0 + (z-ezc)*c1 + ezc*c2 + ezc*ezc*c3 = freq
                eq[1] = *step_z - est_z_contact;
                eq[2] = est_z_contact;
                eq[3] = est_z_contact * est_z_contact;
            } else {
                // 1*c0 + 0*c1 + z*c2 + z*z*c3 = freq
                eq[1] = 0.0;
                eq[2] = *step_z;
                eq[3] = *step_z * *step_z;
            }
        }
        let eqst = mat_transp(&eqs)?;
        let eqst_eqs = mat_mat_mul(&eqst, &eqs)?.ok_or_else(dim_type_error)?;
        let eqst_ans = mat_mat_mul(&eqst, &ans)?.ok_or_else(dim_type_error)?;
        Ok((eqst_eqs, eqst_ans))
    }

    /// `_build_sums(samples, num_le)` (`klippy/extras/probe_eddy_current.py:588`).
    pub fn build_sums(&self, samples: &[(f64, f64)], num_le: usize) -> KResult<[f64; 11]> {
        let mut sum_le_z = 0.0;
        let mut sum_le_z2 = 0.0;
        let mut sum_le_freq = 0.0;
        let mut sum_le_freq_z = 0.0;
        for (z, freq) in samples[..num_le].iter() {
            sum_le_z += *z;
            sum_le_z2 += float_pow(*z, 2.0).map_err(py_error)?;
            sum_le_freq += *freq;
            sum_le_freq_z += *freq * *z;
        }
        let mut sum_gt_z = 0.0;
        let mut sum_gt_z2 = 0.0;
        let mut sum_gt_z3 = 0.0;
        let mut sum_gt_z4 = 0.0;
        let mut sum_gt_freq = 0.0;
        let mut sum_gt_freq_z = 0.0;
        let mut sum_gt_freq_z2 = 0.0;
        for (z, freq) in samples[num_le..].iter() {
            sum_gt_z += *z;
            sum_gt_z2 += float_pow(*z, 2.0).map_err(py_error)?;
            sum_gt_z3 += float_pow(*z, 3.0).map_err(py_error)?;
            sum_gt_z4 += float_pow(*z, 4.0).map_err(py_error)?;
            sum_gt_freq += *freq;
            sum_gt_freq_z += *freq * *z;
            sum_gt_freq_z2 += *freq * float_pow(*z, 2.0).map_err(py_error)?;
        }
        Ok([
            sum_le_z,
            sum_le_z2,
            sum_le_freq,
            sum_le_freq_z,
            sum_gt_z,
            sum_gt_z2,
            sum_gt_z3,
            sum_gt_z4,
            sum_gt_freq,
            sum_gt_freq_z,
            sum_gt_freq_z2,
        ])
    }

    /// `_build_ls_matrix_opt(samples, est_z_contact)`
    /// (`klippy/extras/probe_eddy_current.py:608`).
    pub fn build_ls_matrix_opt(
        &self,
        samples: &[(f64, f64)],
        est_z_contact: f64,
    ) -> KResult<(Matrix, Matrix)> {
        // This function is an optimized version of _build_ls_matrix().
        // `bisect.bisect(samples, (est_z_contact, sys.float_info.max))`.
        let num_le = tuple_bisect_right(samples, est_z_contact);
        // Check for previously calculated raw freq/z counters
        let cached = self.least_squares_cache.borrow().get(&num_le).copied();
        let sums = match cached {
            Some(sums) => sums,
            None => {
                let sums = self.build_sums(samples, num_le)?;
                self.least_squares_cache.borrow_mut().insert(num_le, sums);
                sums
            }
        };
        let [
            sum_le_z,
            sum_le_z2,
            sum_le_freq,
            sum_le_freq_z,
            sum_gt_z,
            sum_gt_z2,
            sum_gt_z3,
            sum_gt_z4,
            sum_gt_freq,
            sum_gt_freq_z,
            sum_gt_freq_z2,
        ] = sums;
        let num_samples = samples.len() as f64;
        let num_le = num_le as f64;
        let ezc = est_z_contact;
        let ezc2 = float_pow(ezc, 2.0).map_err(py_error)?;
        let ezc3 = float_pow(ezc, 3.0).map_err(py_error)?;
        let ezc4 = float_pow(ezc, 4.0).map_err(py_error)?;
        // Build matrices for least squares evaluation
        let mut eqst_eqs: Matrix = vec![vec![0.0; 4]; 4];
        eqst_eqs[0][0] = num_samples;
        eqst_eqs[1][1] = sum_le_z2 - 2.0 * ezc * sum_le_z + num_le * ezc2;
        eqst_eqs[2][2] = sum_gt_z2 + num_le * ezc2;
        eqst_eqs[3][3] = sum_gt_z4 + num_le * ezc4;
        eqst_eqs[0][1] = sum_le_z - num_le * ezc;
        eqst_eqs[1][0] = eqst_eqs[0][1];
        eqst_eqs[0][2] = sum_gt_z + num_le * ezc;
        eqst_eqs[2][0] = eqst_eqs[0][2];
        eqst_eqs[0][3] = sum_gt_z2 + num_le * ezc2;
        eqst_eqs[3][0] = eqst_eqs[0][3];
        eqst_eqs[2][3] = sum_gt_z3 + num_le * ezc3;
        eqst_eqs[3][2] = eqst_eqs[2][3];
        eqst_eqs[2][1] = ezc * eqst_eqs[0][1];
        eqst_eqs[1][2] = eqst_eqs[2][1];
        eqst_eqs[3][1] = ezc2 * eqst_eqs[0][1];
        eqst_eqs[1][3] = eqst_eqs[3][1];
        let mut eqst_ans: Matrix = vec![vec![0.0]; 4];
        eqst_ans[0][0] = sum_le_freq + sum_gt_freq;
        eqst_ans[1][0] = sum_le_freq_z - ezc * sum_le_freq;
        eqst_ans[2][0] = sum_gt_freq_z + ezc * sum_le_freq;
        eqst_ans[3][0] = sum_gt_freq_z2 + ezc2 * sum_le_freq;
        Ok((eqst_eqs, eqst_ans))
    }

    /// `_calc_least_squares(samples, est_z_contact)`
    /// (`klippy/extras/probe_eddy_current.py:642`).
    pub fn calc_least_squares(
        &self,
        samples: &[(f64, f64)],
        est_z_contact: f64,
    ) -> KResult<(f64, Matrix)> {
        let (eqst_eqs, eqst_ans) = self.build_ls_matrix_opt(samples, est_z_contact)?;
        let mut coeffs = gaussian_solve(&eqst_eqs, &eqst_ans, false)?;
        if let Some(c) = &coeffs {
            if c[3][0] < 0.0 {
                // z**2 factor can't be negative - retry using only linear
                let alt_eqst_eqs: Matrix = eqst_eqs[..3].iter().map(|row| row[..3].to_vec()).collect();
                let alt_eqst_ans: Matrix = eqst_ans[..3].to_vec();
                coeffs = gaussian_solve(&alt_eqst_eqs, &alt_eqst_ans, false)?;
                if let Some(mut c) = coeffs {
                    c.push(vec![0.0]);
                    coeffs = Some(c);
                }
            }
        }
        let Some(coeffs) = coeffs else {
            return Ok((FLOAT_MAX, vec![vec![0.0]; 4]));
        };
        let mut rel_err = 0.0;
        for (c, a) in coeffs.iter().zip(eqst_ans.iter()) {
            rel_err += c[0] * a[0];
        }
        Ok((-rel_err, coeffs))
    }

    /// `find_best_fit(data)` (`klippy/extras/probe_eddy_current.py:656`).
    /// `data` entries are `(sensor_freq, toolhead_pos)` pairs.
    pub fn find_best_fit(&self, data: &[(f64, Vec<f64>)]) -> KResult<TapCoeffs> {
        self.least_squares_cache.borrow_mut().clear();
        // Change base of freq/z measurements to improve numerical stability
        let first = data.first().ok_or_else(index_error)?;
        let last = data.last().ok_or_else(index_error)?;
        let base_z = 0.5
            * (*first.1.get(2).ok_or_else(index_error)?
                + *last.1.get(2).ok_or_else(index_error)?);
        let base_freq = 0.5 * (first.0 + last.0);
        let samples: Vec<(f64, f64)> = data
            .iter()
            .map(|d| Ok((*d.1.get(2).ok_or_else(index_error)? - base_z, d.0 - base_freq)))
            .collect::<KResult<_>>()?;
        // Run least squares with various z values to reduce residual error
        let mut min_z = samples[0].0;
        let mut best_z = samples[0].0;
        let mut max_z = samples[samples.len() - 1].0;
        let mut best_err = FLOAT_MAX;
        let mut best_coeffs: Option<Matrix> = None;
        while max_z - min_z > 0.000050 {
            // Select z value to check
            let mid_z = (min_z + max_z) * 0.5;
            let guess_z = if best_z < mid_z {
                (best_z + max_z) * 0.5
            } else {
                (min_z + best_z) * 0.5
            };
            // Calculate least squares error for given z
            let (guess_err, coeffs) = self.calc_least_squares(&samples, guess_z)?;
            // Update search bounds
            if guess_err < best_err {
                if guess_z > best_z {
                    min_z = best_z;
                } else {
                    max_z = best_z;
                }
                best_z = guess_z;
                best_err = guess_err;
                best_coeffs = Some(coeffs);
            } else if guess_z > best_z {
                max_z = guess_z;
            } else {
                min_z = guess_z;
            }
        }
        self.least_squares_cache.borrow_mut().clear();
        // Return to original freq/z measurement base. Python's initial
        // `best_coeffs = [0., 0., 0., 0.]` is a flat list, so
        // `[v[0] for v in best_coeffs]` raises `TypeError` when no guess won.
        let Some(best_coeffs) = best_coeffs else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'float' object is not subscriptable".to_string(),
            });
        };
        let bc: Vec<f64> = best_coeffs.iter().map(|row| row[0]).collect();
        let z_contact = base_z + best_z;
        let freq_contact = base_freq + bc[0] + best_z * bc[2] + best_z * best_z * bc[3];
        let depress_slope = bc[1];
        let slope = bc[2] + 2.0 * best_z * bc[3];
        let slope2 = bc[3];
        Ok([
            z_contact,
            freq_contact,
            depress_slope,
            slope,
            slope2,
        ])
    }
}

// ---------------------------------------------------------------------------
// Probe sessions
// ---------------------------------------------------------------------------

/// `EddyDescend` (`klippy/extras/probe_eddy_current.py:712`).
pub struct EddyDescend {
    printer: Printer,
    sensor_helper: Arc<dyn LDC1612Api>,
    calibration: Arc<dyn EddyCalibrationApi>,
    probe_offsets: Arc<EddyProbeOffsets>,
    param_helper: Arc<EddyParameterHelper>,
    trigger_analog: Arc<MCU_trigger_analog>,
    descend_z: f64,
    z_min_position: f64,
    gather: GCell<Option<Arc<EddyGatherSamples>>>,
}

impl EddyDescend {
    /// `EddyDescend.__init__(config, sensor_helper, calibration, probe_offsets,
    /// param_helper, trigger_analog)`
    /// (`klippy/extras/probe_eddy_current.py:713`).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        config: &Config,
        sensor_helper: Arc<dyn LDC1612Api>,
        calibration: Arc<dyn EddyCalibrationApi>,
        probe_offsets: Arc<EddyProbeOffsets>,
        param_helper: Arc<EddyParameterHelper>,
        trigger_analog: Arc<MCU_trigger_analog>,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let has_z_offset = config
            .get(
                "z_offset",
                GetStrOpts {
                    default: Some(None),
                    note_valid: false,
                },
            )?
            .is_some();
        let has_descend_z = has_z_offset && config
            .get(
                "descend_z",
                GetStrOpts {
                    default: Some(None),
                    note_valid: false,
                },
            )?
            .is_some();
        let descend_z = if has_z_offset && !has_descend_z {
            config.deprecate("z_offset", None)?;
            config
                .getfloat(
                    "z_offset",
                    GetFloatOpts {
                        default: None,
                        above: Some(0.0),
                        ..GetFloatOpts::default()
                    },
                )?
                .expect("a required option")
        } else {
            config
                .getfloat(
                    "descend_z",
                    GetFloatOpts {
                        default: None,
                        above: Some(0.0),
                        ..GetFloatOpts::default()
                    },
                )?
                .expect("a required option")
        };
        let z_min_position = lookup_minimum_z(config)?;
        // The helper registers the `probe:z_virtual_endstop` chip; Python
        // keeps no reference to it either.
        crate::probe::HomingViaProbeHelper::new(config, descend_z, None)?;
        Ok(Arc::new(EddyDescend {
            printer,
            sensor_helper,
            calibration,
            probe_offsets,
            param_helper,
            trigger_analog,
            descend_z,
            z_min_position,
            gather: GCell::new(None),
        }))
    }

    /// `self._descend_z` (`klippy/extras/probe_eddy_current.py:727`).
    pub fn descend_z(&self) -> f64 {
        self.descend_z
    }

    /// `_prep_trigger_analog()` (`klippy/extras/probe_eddy_current.py:728`).
    pub fn prep_trigger_analog(&self) -> KResult<()> {
        let sos_filter = self
            .trigger_analog
            .get_sos_filter()
            .ok_or_else(|| none_has_no_attr("set_filter_design"))?;
        sos_filter.set_filter_design(None);
        sos_filter.set_offset_scale(0, 1.0, false);
        self.trigger_analog.set_raw_range(0, MAX_VALID_RAW_VALUE);
        let trigger_freq = self.calibration.height_to_freq(self.descend_z)?;
        let conv_freq = self.sensor_helper.convert_frequency_to_raw(trigger_freq)?;
        self.trigger_analog
            .set_trigger("gt".to_string(), PyValue::Int(PyInt::from_i64(conv_freq)));
        Ok(())
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe_eddy_current.py:739`).
    pub fn start_probe_session(self: &Arc<Self>, _gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        self.calibration.verify_calibrated()?;
        self.prep_trigger_analog()?;
        let gather = EddyGatherSamples::new(self.printer.clone(), Arc::clone(&self.sensor_helper))?;
        *self.gather.borrow_mut() = Some(gather);
        let session: Arc<dyn ProbeSessionApi> = self.clone();
        Ok(session)
    }

    /// `run_probe(gcmd)` (`klippy/extras/probe_eddy_current.py:744`).
    pub fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let mut pos = toolhead.get_position();
        pos[2] = self.z_min_position;
        let params = self.param_helper.get_probe_params(Some(gcmd))?;
        let speed = params.probe_speed;
        // Perform probing move
        let phoming =
            lookup_interface::<dyn PrinterHomingApi>(&*self.printer, "homing", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let check_movement = !phoming.check_probe_first_home(gcmd)?;
        let mcu_probe: Endstop = self.trigger_analog.clone();
        phoming.probing_move(mcu_probe, opt_vec(&pos), speed, check_movement)?;
        // Extract samples
        let start_time = self.trigger_analog.get_last_trigger_time() + 0.050;
        let end_time = start_time + 0.100;
        let toolhead_pos = toolhead.get_position();
        let offsets = self.probe_offsets.get_offsets(None);
        let calibration = Arc::clone(&self.calibration);
        let cb: ProbeRequestCallback = Arc::new(move |measures| {
            probe_results_from_avg(&measures, &toolhead_pos, &*calibration, offsets)
        });
        self.gather_or_error()?
            .add_probe_request(cb, start_time, end_time)
    }

    /// `pull_probed_results()` (`klippy/extras/probe_eddy_current.py:762`).
    pub fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        self.gather_or_error()?.pull_probed()
    }

    /// `end_probe_session()` (`klippy/extras/probe_eddy_current.py:764`).
    pub fn end_probe_session(&self) -> KResult<()> {
        let gather = self.gather.cloned().ok_or_else(|| none_has_no_attr("finish"))?;
        gather.finish();
        self.gather.replace(None);
        Ok(())
    }

    fn gather_or_error(&self) -> KResult<Arc<EddyGatherSamples>> {
        self.gather
            .cloned()
            .ok_or_else(|| none_has_no_attr("pull_probed"))
    }
}

impl ProbeSessionApi for EddyDescend {
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        EddyDescend::run_probe(self, gcmd)
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        EddyDescend::pull_probed_results(self)
    }
    fn end_probe_session(&self) -> KResult<()> {
        EddyDescend::end_probe_session(self)
    }
}

/// `EddyTap` (`klippy/extras/probe_eddy_current.py:769`).
pub struct EddyTap {
    printer: Printer,
    sensor_helper: Arc<dyn LDC1612Api>,
    param_helper: Arc<EddyParameterHelper>,
    trigger_analog: Arc<MCU_trigger_analog>,
    z_min_position: f64,
    gather: GCell<Option<Arc<EddyGatherSamples>>>,
    filter_design: GCell<Option<Arc<DigitalFilter>>>,
    tap_z_offset: f64,
    tap_threshold: f64,
    current_tap_threshold: GCell<f64>,
    last_tap: GCell<Option<LastTap>>,
    self_ref: GCell<Option<Weak<Self>>>,
}

impl EddyTap {
    /// `EddyTap.__init__(config, sensor_helper, param_helper, trigger_analog)`
    /// (`klippy/extras/probe_eddy_current.py:770`).
    pub fn new(
        config: &Config,
        sensor_helper: Arc<dyn LDC1612Api>,
        param_helper: Arc<EddyParameterHelper>,
        trigger_analog: Arc<MCU_trigger_analog>,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let z_min_position = lookup_minimum_z(config)?;
        let tap_z_offset = config
            .getfloat(
                "tap_z_offset",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let tap_threshold = config
            .getfloat(
                "tap_threshold",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    above: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let this = Arc::new(EddyTap {
            printer,
            sensor_helper: Arc::clone(&sensor_helper),
            param_helper,
            trigger_analog,
            z_min_position,
            gather: GCell::new(None),
            filter_design: GCell::new(None),
            tap_z_offset,
            tap_threshold,
            current_tap_threshold: GCell::new(0.0),
            last_tap: GCell::new(None),
            self_ref: GCell::new(None),
        });
        *this.self_ref.borrow_mut() = Some(Arc::downgrade(&this));
        this.setup_tap()?;
        Ok(this)
    }

    /// `_setup_tap()` (`klippy/extras/probe_eddy_current.py:784`).
    pub fn setup_tap(&self) -> KResult<()> {
        // Create sos filter "design"
        let sps = self.sensor_helper.get_samples_per_second();
        let design = DigitalFilter::new(sps, CfgErrorKind::Config);
        design.add_lowpass(25.0, 4)?;
        design.add_derivative();
        let filter_size = design.get_size() as i64;
        *self.filter_design.borrow_mut() = Some(Arc::new(design));
        // Create SOS filter
        let cmd_queue = self.trigger_analog.get_dispatch().get_command_queue();
        let mcu = self.sensor_helper.get_mcu();
        let sos_filter = MCU_SosFilter::new(mcu, cmd_queue, filter_size)?;
        self.trigger_analog.setup_sos_filter(sos_filter);
        Ok(())
    }

    /// `_prep_trigger_analog_tap(gcmd)`
    /// (`klippy/extras/probe_eddy_current.py:798`).
    pub fn prep_trigger_analog_tap(&self, gcmd: &GCmd) -> KResult<()> {
        let tap_threshold = gcmd
            .get_float(
                "TAP_THRESHOLD",
                GcmdFloatOpts {
                    default: Some(Some(self.tap_threshold)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        if tap_threshold == 0.0 {
            return Err(KlippyError::command("Tap not configured"));
        }
        let params = self.param_helper.get_probe_params(Some(gcmd))?;
        // Setup mcu filter (scale internal values to milli-hz)
        let sos_filter = self
            .trigger_analog
            .get_sos_filter()
            .ok_or_else(|| none_has_no_attr("set_filter_design"))?;
        sos_filter.set_filter_design(self.filter_design.cloned());
        const FRAC_HZ: f64 = 1000.0;
        let s = FRAC_HZ * self.sensor_helper.convert_raw_to_frequency(1.0);
        sos_filter.set_offset_scale(0, s, true);
        self.trigger_analog.set_raw_range(0, MAX_VALID_RAW_VALUE);
        // Set mcu trigger to tap_threshold
        let sps = self.sensor_helper.get_samples_per_second();
        let adj_thresh =
            float_true_div(tap_threshold * params.probe_speed, sps).map_err(py_error)?;
        let samp_thresh = py_trunc_i64(FRAC_HZ * adj_thresh + 0.5)?;
        self.trigger_analog.set_trigger(
            "diff_peak_gt".to_string(),
            PyValue::Int(PyInt::from_i64(samp_thresh)),
        );
        self.current_tap_threshold.set(tap_threshold);
        Ok(())
    }

    /// `_validate_samples_time(measures, start_time, end_time)`
    /// (`klippy/extras/probe_eddy_current.py:818`).
    pub fn validate_samples_time(
        &self,
        measures: &Measures,
        start_time: f64,
        end_time: f64,
    ) -> KResult<()> {
        if end_time - start_time < 0.100 {
            return Err(KlippyError::command(
                "Tap detected too close to start of move",
            ));
        }
        let timestamps: Vec<f64> = measures.iter().map(|m| m.0).collect();
        if timestamps.len() < 2 {
            return Err(KlippyError::command(
                "Unable to obtain probe_eddy_current sensor readings",
            ));
        }
        let mut ts = vec![start_time];
        ts.extend(timestamps.iter().copied());
        ts.push(end_time);
        let tdiffs: Vec<f64> = (1..ts.len()).map(|i| ts[i] - ts[i - 1]).collect();
        let tmax = py_max(&tdiffs)?;
        let tmin = py_min(&tdiffs[1..tdiffs.len() - 1])?;
        let cycle_time =
            float_true_div(1.0, self.sensor_helper.get_samples_per_second()).map_err(py_error)?;
        if tmax > cycle_time * 1.25 {
            return Err(KlippyError::command(format_f(
                "Eddy: Gaps in the data: %.3f > %.3f",
                &[PctValue::Float(tmax), PctValue::Float(cycle_time * 1.25)],
            )?));
        }
        if tmin < cycle_time * 0.75 {
            return Err(KlippyError::command(format_f(
                "Eddy: CLKIN frequency too low: %.3f < %.3f",
                &[PctValue::Float(tmin), PctValue::Float(cycle_time * 0.75)],
            )?));
        }
        Ok(())
    }

    /// `_lookup_toolhead_pos(pos_time)`
    /// (`klippy/extras/probe_eddy_current.py:832`).
    pub fn lookup_toolhead_pos(&self, pos_time: f64) -> KResult<Vec<f64>> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let kin = toolhead.get_kinematics();
        let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
        for stepper in kin.get_steppers() {
            let mcu_pos = stepper.get_past_mcu_position(pos_time);
            kin_spos.insert(
                stepper.get_name(Some(false)),
                stepper.mcu_to_commanded_position(mcu_pos as f64),
            );
        }
        let kin_pos = kin.calc_position(&kin_spos)?;
        kin_pos.iter().map(|v| opt_f64(*v)).collect()
    }

    /// `_error_detect(msg)` (`klippy/extras/probe_eddy_current.py:838`).
    pub fn error_detect(&self, msg: &str) -> KlippyError {
        // `"Unable to detect tap: %s" % (msg,)`: `%s` of a `str` is the string.
        KlippyError::command(format!("Unable to detect tap: {msg}"))
    }

    /// `_analyze_pullback(measures, start_time, end_time)`
    /// (`klippy/extras/probe_eddy_current.py:845`).
    pub fn analyze_pullback(
        &self,
        measures: Measures,
        start_time: f64,
        end_time: f64,
    ) -> KResult<ProbeResult> {
        *self.last_tap.borrow_mut() = None;
        let reactor = self.printer.get_reactor();
        self.validate_samples_time(&measures, start_time, end_time)?;
        // Correlate measurements to toolhead position at time of measurement
        let mut data: Vec<(f64, Vec<f64>)> = Vec::with_capacity(measures.len());
        for (samp_time, sensor_freq, _sensor_z) in measures.iter() {
            let pos = self.lookup_toolhead_pos(*samp_time)?;
            data.push((*sensor_freq, pos));
        }
        reactor.pause(0.0);
        let min_z = data[0].1[2];
        let max_z = data[data.len() - 1].1[2];
        if max_z - min_z < 0.350 {
            let msg = format_f(
                "insufficient lift (%.6f vs %.6f)",
                &[PctValue::Float(max_z - min_z), PctValue::Float(0.350)],
            )?;
            return Err(self.error_detect(&msg));
        }
        // Find best fit for extracted measurements
        let tap_fit = TapBestFit::new();
        let coeffs = tap_fit.find_best_fit(&data)?;
        let [z_contact, _freq_contact, depress_slope, slope, slope2] = coeffs;
        *self.last_tap.borrow_mut() = Some(("fail".to_string(), z_contact - min_z, coeffs));
        reactor.pause(0.0);
        let contact_slope_delta = depress_slope - slope;
        if contact_slope_delta < self.current_tap_threshold.get() {
            let msg = format_f(
                "insufficient slope delta (%.6f vs %.6f)",
                &[
                    PctValue::Float(contact_slope_delta),
                    PctValue::Float(self.current_tap_threshold.get()),
                ],
            )?;
            return Err(self.error_detect(&msg));
        }
        if slope >= 0.0 || slope2 < 0.0 {
            let msg = format_f(
                "invalid free air slope (s=%.6f s2=%.6f)",
                &[PctValue::Float(slope), PctValue::Float(slope2)],
            )?;
            return Err(self.error_detect(&msg));
        }
        if z_contact - min_z < 0.030 || z_contact - min_z > 0.250 {
            let msg = format_f(
                "invalid depress distance (%.6f vs %.6f:%.6f)",
                &[
                    PctValue::Float(z_contact - min_z),
                    PctValue::Float(0.030),
                    PctValue::Float(0.250),
                ],
            )?;
            return Err(self.error_detect(&msg));
        }
        *self.last_tap.borrow_mut() = Some(("success".to_string(), z_contact - min_z, coeffs));
        // Report probe position
        let mut trig_idx = data.len() - 1;
        while trig_idx > 0 && data[trig_idx - 1].1[2] > z_contact {
            trig_idx -= 1;
        }
        let trig_pos = &data[trig_idx].1;
        let adj_z_contact = z_contact - self.tap_z_offset;
        Ok(ProbeResult {
            bed_x: trig_pos[0],
            bed_y: trig_pos[1],
            bed_z: adj_z_contact,
            test_x: trig_pos[0],
            test_y: trig_pos[1],
            test_z: trig_pos[2],
        })
    }

    /// `get_last_tap_info()` (`klippy/extras/probe_eddy_current.py:885`).
    pub fn get_last_tap_info(&self) -> Option<LastTap> {
        self.last_tap.cloned()
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe_eddy_current.py:888`).
    pub fn start_probe_session(self: &Arc<Self>, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        self.prep_trigger_analog_tap(gcmd)?;
        let gather = EddyGatherSamples::new(self.printer.clone(), Arc::clone(&self.sensor_helper))?;
        *self.gather.borrow_mut() = Some(gather);
        let session: Arc<dyn ProbeSessionApi> = self.clone();
        Ok(session)
    }

    /// `run_probe(gcmd)` (`klippy/extras/probe_eddy_current.py:892`).
    pub fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let mut pos = toolhead.get_position();
        pos[2] = self.z_min_position;
        let params = self.param_helper.get_probe_params(Some(gcmd))?;
        let speed = params.probe_speed;
        let lift_speed = params.lift_speed;
        let lift_dist = gcmd
            .get_float(
                "SAMPLE_RETRACT_DIST",
                GcmdFloatOpts {
                    default: Some(Some(4.0)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        // Perform probing move
        let phoming =
            lookup_interface::<dyn PrinterHomingApi>(&*self.printer, "homing", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let mcu_probe: Endstop = self.trigger_analog.clone();
        phoming.probing_move(mcu_probe, opt_vec(&pos), speed, true)?;
        // Perform lifting move
        let mut haltpos = toolhead.get_position();
        haltpos[2] += lift_dist;
        let retract_start_time = toolhead.get_last_move_time()?;
        toolhead.manual_move(opt_vec(&haltpos), lift_speed)?;
        // Extract retract samples
        let start_time = retract_start_time - 0.010;
        let end_time = retract_start_time + 0.150;
        let this = self_ref_upgrade(&self.self_ref)?;
        let cb: ProbeRequestCallback = Arc::new(move |measures| {
            this.analyze_pullback(measures, start_time, end_time)
        });
        self.gather_or_error()?
            .add_probe_request(cb, start_time, end_time)
    }

    /// `pull_probed_results()` (`klippy/extras/probe_eddy_current.py:913`).
    pub fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        self.gather_or_error()?.pull_probed()
    }

    /// `end_probe_session()` (`klippy/extras/probe_eddy_current.py:915`).
    pub fn end_probe_session(&self) -> KResult<()> {
        let gather = self.gather.cloned().ok_or_else(|| none_has_no_attr("finish"))?;
        gather.finish();
        self.gather.replace(None);
        Ok(())
    }

    fn gather_or_error(&self) -> KResult<Arc<EddyGatherSamples>> {
        self.gather
            .cloned()
            .ok_or_else(|| none_has_no_attr("pull_probed"))
    }
}

impl ProbeSessionApi for EddyTap {
    /// Pauses the reactor.
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        EddyTap::run_probe(self, gcmd)
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        EddyTap::pull_probed_results(self)
    }
    fn end_probe_session(&self) -> KResult<()> {
        EddyTap::end_probe_session(self)
    }
}

/// `EddyScanningProbe` (`klippy/extras/probe_eddy_current.py:920`).
pub struct EddyScanningProbe {
    printer: Printer,
    sensor_helper: Arc<dyn LDC1612Api>,
    calibration: Arc<dyn EddyCalibrationApi>,
    offsets: (f64, f64, f64),
    gather: GCell<Option<Arc<EddyGatherSamples>>>,
    sample_time_delay: f64,
    sample_time: GCell<f64>,
    is_rapid: GCell<bool>,
    self_ref: GCell<Option<Weak<Self>>>,
}

impl EddyScanningProbe {
    /// `EddyScanningProbe.__init__(config, sensor_helper, calibration,
    /// probe_offsets)` (`klippy/extras/probe_eddy_current.py:921`).
    pub fn new(
        config: &Config,
        sensor_helper: Arc<dyn LDC1612Api>,
        calibration: Arc<dyn EddyCalibrationApi>,
        probe_offsets: &EddyProbeOffsets,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let this = Arc::new(EddyScanningProbe {
            printer: printer.clone(),
            sensor_helper,
            calibration,
            offsets: probe_offsets.get_offsets(None),
            gather: GCell::new(None),
            sample_time_delay: 0.050,
            sample_time: GCell::new(0.0),
            is_rapid: GCell::new(false),
            self_ref: GCell::new(None),
        });
        *this.self_ref.borrow_mut() = Some(Arc::downgrade(&this));
        let this2 = Arc::clone(&this);
        printer.register_event_handler(
            EV_GCODE_COMMAND_ERROR,
            Box::new(move |_args| {
                this2.handle_command_error()?;
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// `_handle_command_error()` (`klippy/extras/probe_eddy_current.py:932`).
    pub fn handle_command_error(&self) -> KResult<()> {
        if self.gather.cloned().is_some() {
            self.end_probe_session()?;
        }
        Ok(())
    }

    /// `_lookup_toolhead_pos(pos_time)`
    /// (`klippy/extras/probe_eddy_current.py:936`).
    pub fn lookup_toolhead_pos(&self, pos_time: f64) -> KResult<Vec<f64>> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let kin = toolhead.get_kinematics();
        let mut kin_spos: IndexMap<String, f64> = IndexMap::new();
        for stepper in kin.get_steppers() {
            let mcu_pos = stepper.get_past_mcu_position(pos_time);
            kin_spos.insert(
                stepper.get_name(Some(false)),
                stepper.mcu_to_commanded_position(mcu_pos as f64),
            );
        }
        let kin_pos = kin.calc_position(&kin_spos)?;
        kin_pos.iter().map(|v| opt_f64(*v)).collect()
    }

    /// `_analyze_scan(measures, pos_time)`
    /// (`klippy/extras/probe_eddy_current.py:942`).
    pub fn analyze_scan(&self, measures: Measures, pos_time: f64) -> KResult<ProbeResult> {
        let toolhead_pos = self.lookup_toolhead_pos(pos_time)?;
        probe_results_from_avg(&measures, &toolhead_pos, &*self.calibration, self.offsets)
    }

    /// `_rapid_lookahead_cb(printtime)`
    /// (`klippy/extras/probe_eddy_current.py:946`).
    pub fn rapid_lookahead_cb(&self, printtime: f64) -> KResult<()> {
        let sample_time = self.sample_time.get();
        let start_time = printtime - sample_time / 2.0;
        let end_time = start_time + sample_time;
        let this = self_ref_upgrade(&self.self_ref)?;
        let cb: ProbeRequestCallback =
            Arc::new(move |measures| this.analyze_scan(measures, printtime));
        self.gather_or_error()?
            .add_probe_request(cb, start_time, end_time)
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe_eddy_current.py:952`).
    pub fn start_probe_session(self: &Arc<Self>, gcmd: &GCmd) -> KResult<Arc<Self>> {
        self.calibration.verify_calibrated()?;
        let gather = EddyGatherSamples::new(self.printer.clone(), Arc::clone(&self.sensor_helper))?;
        *self.gather.borrow_mut() = Some(gather);
        let sample_time = gcmd
            .get_float(
                "SAMPLE_TIME",
                GcmdFloatOpts {
                    default: Some(Some(0.100)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        self.sample_time.set(sample_time);
        let method = gcmd
            .get(
                "METHOD",
                GcmdGetOpts {
                    default: Some(PyValue::Str("scan".to_string())),
                    ..GcmdGetOpts::default()
                },
            )?
            .str()
            .to_lowercase();
        self.is_rapid.set(method == "rapid_scan");
        Ok(self.clone())
    }

    /// `run_probe(gcmd)` (`klippy/extras/probe_eddy_current.py:958`).
    pub fn run_probe(&self, _gcmd: &GCmd) -> KResult<()> {
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        if self.is_rapid.get() {
            let this = self_ref_upgrade(&self.self_ref)?;
            toolhead.register_lookahead_callback(Box::new(move |printtime| {
                this.rapid_lookahead_cb(printtime)
            }))?;
            return Ok(());
        }
        let sample_time = self.sample_time.get();
        let printtime = toolhead.get_last_move_time()?;
        toolhead.dwell(self.sample_time_delay + sample_time)?;
        let start_time = printtime + self.sample_time_delay;
        let end_time = start_time + sample_time;
        let this = self_ref_upgrade(&self.self_ref)?;
        let cb: ProbeRequestCallback =
            Arc::new(move |measures| this.analyze_scan(measures, start_time));
        self.gather_or_error()?
            .add_probe_request(cb, start_time, end_time)
    }

    /// `pull_probed_results()` (`klippy/extras/probe_eddy_current.py:969`).
    pub fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        if self.is_rapid.get() {
            // Flush lookahead (so all lookahead callbacks are invoked)
            let toolhead = lookup_interface::<dyn ToolHeadApi>(
                &*self.printer,
                "toolhead",
                LookupDefault::Raise,
            )?
            .expect("Raise never yields None");
            toolhead.get_last_move_time()?;
        }
        let results = self.gather_or_error()?.pull_probed()?;
        // Allow axis_twist_compensation to update results
        let results = Arc::new(ProbeResultList::new(results));
        self.printer.send_event(
            EV_PROBE_UPDATE_RESULTS,
            &[EventArg::handle(Arc::clone(&results))],
        )?;
        Ok(results.cloned())
    }

    /// `end_probe_session()` (`klippy/extras/probe_eddy_current.py:978`).
    pub fn end_probe_session(&self) -> KResult<()> {
        let gather = self.gather.cloned().ok_or_else(|| none_has_no_attr("finish"))?;
        gather.finish();
        self.gather.replace(None);
        Ok(())
    }

    fn gather_or_error(&self) -> KResult<Arc<EddyGatherSamples>> {
        self.gather
            .cloned()
            .ok_or_else(|| none_has_no_attr("pull_probed"))
    }
}

impl ProbeSessionApi for EddyScanningProbe {
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        EddyScanningProbe::run_probe(self, gcmd)
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        EddyScanningProbe::pull_probed_results(self)
    }
    fn end_probe_session(&self) -> KResult<()> {
        EddyScanningProbe::end_probe_session(self)
    }
}

// ---------------------------------------------------------------------------
// Main probe interface
// ---------------------------------------------------------------------------

/// Eddy specific `ProbeOffsets` class (does not store `z_offset`)
/// (`klippy/extras/probe_eddy_current.py:988`).
pub struct EddyProbeOffsets {
    /// `x_offset`
    pub x_offset: f64,
    /// `y_offset`
    pub y_offset: f64,
}

impl EddyProbeOffsets {
    /// `EddyProbeOffsets.__init__(config)`
    /// (`klippy/extras/probe_eddy_current.py:989`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let x_offset = config
            .getfloat(
                "x_offset",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let y_offset = config
            .getfloat(
                "y_offset",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    ..GetFloatOpts::default()
                },
            )?
            .expect("a default was given");
        Ok(Arc::new(EddyProbeOffsets { x_offset, y_offset }))
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/probe_eddy_current.py:992`).
    pub fn get_offsets(&self, _gcmd: Option<&GCmd>) -> (f64, f64, f64) {
        (self.x_offset, self.y_offset, 0.0)
    }
}

/// Wrapper around `ProbeParameterHelper`
/// (`klippy/extras/probe_eddy_current.py:996`).
pub struct EddyParameterHelper {
    param_helper: Arc<ProbeParameterHelper>,
}

impl EddyParameterHelper {
    /// `EddyParameterHelper.__init__(config)`
    /// (`klippy/extras/probe_eddy_current.py:997`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        Ok(Arc::new(EddyParameterHelper {
            param_helper: ProbeParameterHelper::new(config)?,
        }))
    }

    /// `get_probe_params(gcmd=None)`
    /// (`klippy/extras/probe_eddy_current.py:999`).
    pub fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        let method = match gcmd {
            Some(gcmd) => Some(
                gcmd.get(
                    "METHOD",
                    GcmdGetOpts {
                        default: Some(PyValue::Str(String::new())),
                        ..GcmdGetOpts::default()
                    },
                )?
                .str()
                .to_lowercase(),
            ),
            None => None,
        };
        if !matches!(
            method.as_deref(),
            Some("scan") | Some("rapid_scan") | Some("tap")
        ) {
            return self.param_helper.get_probe_params(gcmd);
        }
        let gcmd = gcmd.expect("a scan/tap method comes from a command");
        let probe_speed = gcmd
            .get_float(
                "PROBE_SPEED",
                GcmdFloatOpts {
                    default: Some(Some(5.0)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let lift_speed = gcmd
            .get_float(
                "LIFT_SPEED",
                GcmdFloatOpts {
                    default: Some(Some(5.0)),
                    above: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let samples = gcmd
            .get_int(
                "SAMPLES",
                GcmdIntOpts {
                    default: Some(Some(1)),
                    minval: Some(1),
                    ..GcmdIntOpts::default()
                },
            )?
            .expect("a default was given");
        let samp_retract_dist = 0.0;
        let samp_tolerance = gcmd
            .get_float(
                "SAMPLES_TOLERANCE",
                GcmdFloatOpts {
                    default: Some(Some(0.100)),
                    minval: Some(0.0),
                    ..GcmdFloatOpts::default()
                },
            )?
            .expect("a default was given");
        let samp_retries = gcmd
            .get_int(
                "SAMPLES_TOLERANCE_RETRIES",
                GcmdIntOpts {
                    default: Some(Some(0)),
                    minval: Some(0),
                    ..GcmdIntOpts::default()
                },
            )?
            .expect("a default was given");
        let samples_result = gcmd
            .get(
                "SAMPLES_RESULT",
                GcmdGetOpts {
                    default: Some(PyValue::Str("average".to_string())),
                    ..GcmdGetOpts::default()
                },
            )?
            .str();
        Ok(ProbeParams {
            probe_speed,
            lift_speed,
            samples,
            sample_retract_dist: samp_retract_dist,
            samples_tolerance: samp_tolerance,
            samples_tolerance_retries: samp_retries,
            samples_result,
        })
    }
}

impl ProbeParameterHelperApi for EddyParameterHelper {
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        EddyParameterHelper::get_probe_params(self, gcmd)
    }
}

/// Main "printer object" (`klippy/extras/probe_eddy_current.py:1021`).
pub struct PrinterEddyProbe {
    calibration: Arc<EddyCalibration>,
    sensor_helper: Arc<dyn LDC1612Api>,
    probe_offsets: Arc<EddyProbeOffsets>,
    param_helper: Arc<EddyParameterHelper>,
    eddy_descend_session: Arc<SampleAveragingHelper>,
    eddy_tap_session: Arc<SampleAveragingHelper>,
    eddy_scan: Arc<EddyScanningProbe>,
    cmd_helper: GCell<Option<Arc<ProbeCommandHelper>>>,
}

impl PrinterEddyProbe {
    /// `PrinterEddyProbe.__init__(config)`
    /// (`klippy/extras/probe_eddy_current.py:1022`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let calibration = EddyCalibration::new(config)?;
        let calibration_api: Arc<dyn EddyCalibrationApi> = calibration.clone();
        EddyCalibrationTool::new(config, Arc::clone(&calibration_api))?;
        // Sensor type
        let sensor = create_sensor(config, Arc::clone(&calibration_api))?;
        // Create trigger_analog interface
        let trigger_sensor: Arc<dyn TriggerAnalogSensor> = sensor.clone();
        let trig_analog = MCU_trigger_analog::new(trigger_sensor)?;
        {
            let dispatch = trig_analog.get_dispatch();
            let add_stepper: crate::probe::AddStepperCb =
                Box::new(move |stepper| dispatch.add_stepper(stepper));
            crate::probe::LookupZSteppers::new(config, add_stepper);
        }
        // Basic probe requests
        let probe_offsets = EddyProbeOffsets::new(config)?;
        let param_helper = EddyParameterHelper::new(config)?;
        let param_api: Arc<dyn ProbeParameterHelperApi> = param_helper.clone();
        let eddy_descend = EddyDescend::new(
            config,
            Arc::clone(&sensor),
            Arc::clone(&calibration_api),
            Arc::clone(&probe_offsets),
            Arc::clone(&param_helper),
            Arc::clone(&trig_analog),
        )?;
        let eddy_descend_session = SampleAveragingHelper::new(
            config,
            Arc::clone(&param_api),
            {
                let eddy_descend = Arc::clone(&eddy_descend);
                Box::new(move |gcmd| eddy_descend.start_probe_session(gcmd))
            },
        );
        // Probing via "tap" interface
        let eddy_tap = EddyTap::new(
            config,
            Arc::clone(&sensor),
            Arc::clone(&param_helper),
            Arc::clone(&trig_analog),
        )?;
        EddyTapCalibration::new(config, Arc::clone(&calibration_api), Arc::clone(&eddy_tap))?;
        let eddy_tap_session = SampleAveragingHelper::new(
            config,
            Arc::clone(&param_api),
            {
                let eddy_tap = Arc::clone(&eddy_tap);
                Box::new(move |gcmd| eddy_tap.start_probe_session(gcmd))
            },
        );
        // Probing via "scan" and "rapid_scan" requests
        let eddy_scan = EddyScanningProbe::new(
            config,
            Arc::clone(&sensor),
            Arc::clone(&calibration_api),
            &probe_offsets,
        )?;
        let this = Arc::new(PrinterEddyProbe {
            calibration,
            sensor_helper: sensor,
            probe_offsets,
            param_helper,
            eddy_descend_session,
            eddy_tap_session,
            eddy_scan,
            cmd_helper: GCell::new(None),
        });
        // Register with main probe interface
        let probe_weak = Arc::downgrade(&(Arc::clone(&this) as Arc<dyn PrinterProbeApi>));
        *this.cmd_helper.borrow_mut() = Some(ProbeCommandHelper::new(config, probe_weak, None, false)?);
        printer.add_object("probe", Arc::clone(&this) as ObjectRef)?;
        Ok(this)
    }

    /// `add_client(cb)` (`klippy/extras/probe_eddy_current.py:1054`).
    pub fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        self.sensor_helper.add_client(cb)
    }

    /// `get_probe_params(gcmd=None)`
    /// (`klippy/extras/probe_eddy_current.py:1056`).
    pub fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/probe_eddy_current.py:1058`).
    pub fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        if let Some(gcmd) = gcmd {
            let method = gcmd
                .get(
                    "METHOD",
                    GcmdGetOpts {
                        default: Some(PyValue::Str(String::new())),
                        ..GcmdGetOpts::default()
                    },
                )?
                .str()
                .to_lowercase();
            if method == "tap" {
                return Ok((0.0, 0.0, 0.0));
            }
        }
        Ok(self.probe_offsets.get_offsets(gcmd))
    }

    /// `get_status(eventtime)` (`klippy/extras/probe_eddy_current.py:1062`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.cmd_helper
            .cloned()
            .expect("the command helper is built in the constructor")
            .get_status(eventtime)
    }

    /// `start_probe_session(gcmd)`
    /// (`klippy/extras/probe_eddy_current.py:1064`).
    pub fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        let method = gcmd
            .get(
                "METHOD",
                GcmdGetOpts {
                    default: Some(PyValue::Str("automatic".to_string())),
                    ..GcmdGetOpts::default()
                },
            )?
            .str()
            .to_lowercase();
        if method == "scan" || method == "rapid_scan" {
            let session: Arc<dyn ProbeSessionApi> = self.eddy_scan.start_probe_session(gcmd)?;
            return Ok(session);
        }
        if method == "tap" {
            let session: Arc<dyn ProbeSessionApi> =
                self.eddy_tap_session.start_probe_session(gcmd)?;
            return Ok(session);
        }
        let session: Arc<dyn ProbeSessionApi> =
            self.eddy_descend_session.start_probe_session(gcmd)?;
        Ok(session)
    }

    /// `register_drift_compensation(comp)`
    /// (`klippy/extras/probe_eddy_current.py:1071`).
    pub fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>) {
        self.calibration.register_drift_compensation(comp)
    }
}

impl PrinterProbeApi for PrinterEddyProbe {
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        PrinterEddyProbe::get_probe_params(self, gcmd)
    }
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        PrinterEddyProbe::get_offsets(self, gcmd)
    }
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        PrinterEddyProbe::start_probe_session(self, gcmd)
    }
}

impl PrinterEddyProbeApi for PrinterEddyProbe {
    fn add_client(&self, cb: ClientCallback) -> KResult<()> {
        PrinterEddyProbe::add_client(self, cb)
    }
    fn register_drift_compensation(&self, comp: Arc<dyn DriftCompensationApi>) {
        PrinterEddyProbe::register_drift_compensation(self, comp)
    }
}

impl PrinterObject for PrinterEddyProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi, dyn PrinterEddyProbeApi);

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(PrinterEddyProbe::get_status(self, eventtime)))
    }
}

/// `sensors[sensor_type](config, self.calibration)`
/// (`klippy/extras/probe_eddy_current.py:1027-1029`). `ldc1612` is the only
/// entry of `sensors`, so the validated choice always builds an `LDC1612`.
fn create_sensor(
    config: &Config,
    calibration: Arc<dyn EddyCalibrationApi>,
) -> KResult<Arc<dyn LDC1612Api>> {
    let mut choices: IndexMap<PyValue, PyValue> = IndexMap::new();
    choices.insert(
        PyValue::Str("ldc1612".to_string()),
        PyValue::Str("ldc1612".to_string()),
    );
    config.getchoice(
        "sensor_type",
        &PyValue::Dict(choices),
        GetChoiceOpts {
            default: None,
            note_valid: true,
        },
    )?;
    let sensor: Arc<dyn LDC1612Api> = LDC1612::new(config, Some(calibration))?;
    Ok(sensor)
}

/// `load_config_prefix(config)` (`klippy/extras/probe_eddy_current.py:1074`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterEddyProbe::new(config)? as ObjectRef))
}

/// Module entry point for the `probe_eddy_current` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}

// ---------------------------------------------------------------------------
// Helpers mirroring CPython semantics
// ---------------------------------------------------------------------------

/// The `calibrate` option parse of `EddyCalibration.__init__`
/// (`klippy/extras/probe_eddy_current.py:38-40`): `(pos, freq)` pairs.
fn parse_calibration(cal: &str) -> KResult<Vec<(f64, f64)>> {
    let mut pairs: Vec<(f64, f64)> = Vec::new();
    for raw in cal.split(',') {
        // `d.strip().split(':', 1)`: `float('')` raises `ValueError` first,
        // and a single field then raises `IndexError` at `c[1]`.
        let line = raw.trim();
        match line.split_once(':') {
            Some((pos_str, freq_str)) => {
                let pos = float_from_string(pos_str).map_err(py_error)?;
                let freq = float_from_string(freq_str).map_err(py_error)?;
                pairs.push((pos, freq));
            }
            None => {
                float_from_string(line).map_err(py_error)?;
                return Err(index_error());
            }
        }
    }
    Ok(pairs)
}

/// `percent_format` with `PyError` mapped to the Python exception class.
fn format_f(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(py_error)
}

/// Map a `pycompat` error to the Python exception class raised uncaught.
fn py_error(e: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

fn pop_from_empty() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "pop from empty list".to_string(),
    }
}

fn dim_type_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: "'NoneType' object is not iterable".to_string(),
    }
}

/// `<NoneType>.<attr>` of a failed duck-typed call.
fn none_has_no_attr(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'NoneType' object has no attribute '{attr}'"),
    }
}

/// Python `int(x)` truncation toward zero for finite `x`.
fn py_trunc_i64(f: f64) -> KResult<i64> {
    if f.is_nan() {
        return Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: "cannot convert float NaN to integer".to_string(),
        });
    }
    if f.is_infinite() {
        return Err(KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "cannot convert float infinity to integer".to_string(),
        });
    }
    trunc_float_to_bigint(f)
        .0
        .to_string()
        .parse::<i64>()
        .map_err(|_| KlippyError::Internal {
            class_name: "OverflowError".to_string(),
            message: "Python int too large to convert to C long".to_string(),
        })
}

/// `bisect.bisect` (`bisect_right`): insertion point after any entries `<= x`.
fn bisect_right(a: &[f64], x: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = a.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        if x < a[mid] {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    lo
}

/// `bisect.bisect(samples, (est_z_contact, sys.float_info.max))`: a sample
/// counts as `<=` the key when `(z, freq) <= (est, max)`.
fn tuple_bisect_right(samples: &[(f64, f64)], est_z_contact: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = samples.len();
    while lo < hi {
        let mid = (lo + hi) / 2;
        let (z, freq) = samples[mid];
        if est_z_contact < z || (est_z_contact == z && FLOAT_MAX < freq) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    lo
}

/// `sum(floats)`: left-to-right accumulation starting from the int `0`.
fn py_sum(values: &[f64]) -> f64 {
    let mut acc = 0.0f64;
    for v in values {
        acc += *v;
    }
    acc
}

/// `max(values)`: the first maximum wins, like CPython's `max`.
fn py_max(values: &[f64]) -> KResult<f64> {
    let mut iter = values.iter().copied();
    let mut best = iter.next().ok_or_else(|| KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "max() arg is an empty sequence".to_string(),
    })?;
    for v in iter {
        if v > best {
            best = v;
        }
    }
    Ok(best)
}

/// `min(values)`: the first minimum wins, like CPython's `min`.
fn py_min(values: &[f64]) -> KResult<f64> {
    let mut iter = values.iter().copied();
    let mut best = iter.next().ok_or_else(|| KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "min() arg is an empty sequence".to_string(),
    })?;
    for v in iter {
        if v < best {
            best = v;
        }
    }
    Ok(best)
}

/// `math.sqrt` (libm, correctly rounded).
fn libm_sqrt(x: f64) -> f64 {
    x.sqrt()
}

/// A `kin.calc_position` entry: a `None` raises `TypeError` at the first
/// arithmetic use in Python.
fn opt_f64(v: Option<f64>) -> KResult<f64> {
    v.ok_or_else(|| KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: "unsupported operand type(s) for *: 'NoneType' and 'float'".to_string(),
    })
}

/// Move coordinates as the `Vec<Option<f64>>` of `toolhead.manual_move`.
fn opt_vec(pos: &[f64]) -> Vec<Option<f64>> {
    pos.iter().map(|v| Some(*v)).collect()
}

/// `Arc` of a `Weak` self-reference (`self_ref`), or the `AttributeError` a
/// `None` would raise.
fn self_ref_upgrade<T>(cell: &GCell<Option<Weak<T>>>) -> KResult<Arc<T>> {
    cell.borrow()
        .as_ref()
        .and_then(Weak::upgrade)
        .ok_or_else(|| none_has_no_attr("add_probe_request"))
}

fn pyval_float(v: &PyValue) -> KResult<f64> {
    match v {
        PyValue::Float(f) => Ok(*f),
        PyValue::Int(n) => n
            .inner()
            .to_string()
            .parse::<f64>()
            .map_err(|_| KlippyError::Internal {
                class_name: "OverflowError".to_string(),
                message: "int too large to convert to float".to_string(),
            }),
        PyValue::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        PyValue::Str(s) => float_from_string(s).map_err(py_error),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "float() argument must be a string or a number, not '{}'",
                pycompat::value::type_name(other)
            ),
        }),
    }
}

/// `value[key]` of a Python dict.
fn py_dict_get<'a>(value: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    py_dict_get_or(value, key).ok_or_else(|| KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: str_repr(key),
    })
}

/// `value.get(key)` of a Python dict.
fn py_dict_get_or<'a>(value: &'a PyValue, key: &str) -> Option<&'a PyValue> {
    if let PyValue::Dict(map) = value {
        for (k, v) in map {
            if k.str() == key {
                return Some(v);
            }
        }
    }
    None
}

/// `coord.z` of a `gcode.Coord` (a 4-tuple).
fn coord_z(origin: &PyValue) -> KResult<PyValue> {
    match origin {
        PyValue::Tuple(items) | PyValue::List(items) => {
            items.get(2).cloned().ok_or_else(index_error)
        }
        other => Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: format!(
                "'{}' object has no attribute 'z'",
                pycompat::value::type_name(other)
            ),
        }),
    }
}

/// `msg['data']` as `(samp_time, freq, z)` triples (`ldc1612.py:190`).
fn msg_data(msg: &PyValue) -> KResult<Measures> {
    let data = py_dict_get(msg, "data")?;
    let list = match data {
        PyValue::List(list) | PyValue::Tuple(list) => list,
        other => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!("'{}' object is not iterable", pycompat::value::type_name(other)),
            });
        }
    };
    let mut out = Vec::with_capacity(list.len());
    for item in list {
        let triple = match item {
            PyValue::List(t) | PyValue::Tuple(t) => t,
            other => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "cannot unpack non-iterable {} object",
                        pycompat::value::type_name(other)
                    ),
                });
            }
        };
        if triple.len() != 3 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: if triple.len() < 3 {
                    format!("not enough values to unpack (expected 3, got {})", triple.len())
                } else {
                    "too many values to unpack (expected 3)".to_string()
                },
            });
        }
        out.push((
            pyval_float(&triple[0])?,
            pyval_float(&triple[1])?,
            pyval_float(&triple[2])?,
        ));
    }
    Ok(out)
}

fn m_get(m: &Matrix, i: usize, j: usize) -> KResult<f64> {
    m.get(i)
        .and_then(|row| row.get(j))
        .copied()
        .ok_or_else(index_error)
}
