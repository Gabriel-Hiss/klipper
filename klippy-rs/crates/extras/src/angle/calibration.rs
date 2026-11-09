//! `AngleCalibration` port (`klippy/extras/angle.py:18-266`).
//!
//! The 64-bucket linear-interpolation calibration table, the
//! numpy-backed `load_calibration` least-squares solve (in-tree
//! one-sided Jacobi SVD, see [`lstsq_min_norm`]), and the
//! stepper-coupled `ANGLE_CALIBRATE` move sequence.
//!
//! Duck-typed dependencies are the owner traits
//! ([`crate::force_move::ForceMoveApi`],
//! [`crate::tmc::TMCCommandHelperApi`], the module's own
//! [`super::AngleApi`]) plus the `klippy_api` core traits for
//! `configfile` and `toolhead`.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetStrOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler};
use klippy_api::object::EventArg;
use klippy_api::printer::{
    LookupDefault, Printer, EV_KLIPPY_CONNECT, EV_STEPPER_SYNC_MCU_POSITION,
};
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, require_interface, ObjectRef};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

use super::{
    dict_get, float_div, float_mod as fmod, fmt_const, index_error, key_error,
    name_last_token, pct_float, pct_int, py_trunc_i64, pyvalue_to_f64, pyvalue_to_i64,
    AngleApi, ANGLE_BITS, CALIBRATION_BITS, TRINAMIC_DRIVERS,
};
use crate::bulk_sensor::ClientCallback;
use crate::force_move::ForceMoveApi;
use crate::tmc::TMCCommandHelperApi;

/// `fcal` / `rcal`: step index to the angles sampled at that step.
pub type CalibrationMaps = (IndexMap<i64, Vec<i64>>, IndexMap<i64, Vec<i64>>);

/// `cmd_ANGLE_CALIBRATE_help` (angle.py:226).
pub const ANGLE_CALIBRATE_HELP: &str = "Calibrate angle sensor to stepper motor";

/// `class AngleCalibration` (`angle.py:18-266`).
pub struct AngleCalibration {
    printer: Printer,
    name: String,
    stepper_name: Option<String>,
    tmc_module: GCell<Option<ObjectRef>>,
    mcu_stepper: GCell<Option<Stepper>>,
    mcu_pos_offset: GCell<Option<f64>>,
    angle_phase_offset: GCell<f64>,
    calibration_reversed: GCell<bool>,
    calibration: GCell<Vec<i64>>,
    has_table: bool,
}

impl AngleCalibration {
    /// `AngleCalibration.__init__(config)` (lines 19-51).
    pub fn new(config: &Config) -> KResult<Arc<AngleCalibration>> {
        let printer: Printer = config.get_printer();
        let name: String = config.get_name();
        let stepper_name: Option<String> = config.get(
            "stepper",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        let this = Arc::new(AngleCalibration {
            printer,
            name,
            stepper_name: stepper_name.clone(),
            tmc_module: GCell::new(None),
            mcu_stepper: GCell::new(None),
            mcu_pos_offset: GCell::new(None),
            angle_phase_offset: GCell::new(0.0),
            calibration_reversed: GCell::new(false),
            calibration: GCell::new(Vec::new()),
            has_table: stepper_name.is_some(),
        });
        let Some(stepper) = stepper_name else {
            // No calibration: the Python returns early (line 24).
            return Ok(this);
        };
        let sconfig = config.getsection(&stepper);
        // Validated for presence (line 31); the value itself is read
        // live from `configfile` settings by `get_microsteps`.
        sconfig.getint(
            "microsteps",
            GetIntOpts { default: None, minval: None, maxval: None, note_valid: false },
        )?;
        let cal: Option<String> = config.get(
            "calibrate",
            GetStrOpts { default: Some(None), note_valid: true },
        )?;
        if let Some(cal_str) = cal {
            let angles: Vec<f64> = cal_str
                .split(',')
                .map(str::trim)
                .filter(|d| !d.is_empty())
                .map(parse_config_float)
                .collect::<KResult<Vec<f64>>>()?;
            this.load_calibration(&angles)?;
        }
        {
            let cb = Arc::clone(&this);
            this.printer.register_event_handler(
                EV_STEPPER_SYNC_MCU_POSITION,
                Box::new(move |args: &[EventArg]| {
                    cb.handle_sync_mcu_pos(args);
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let cb = Arc::clone(&this);
            this.printer.register_event_handler(
                EV_KLIPPY_CONNECT,
                Box::new(move |_args: &[EventArg]| {
                    cb.connect()?;
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let cname = name_last_token(&this.name);
            let gcode = lookup_interface::<dyn GCodeApi>(
                &*this.printer,
                "gcode",
                LookupDefault::Raise,
            )?
            .expect("LookupDefault::Raise never yields None");
            let cb = Arc::clone(&this);
            let handler: GCodeCommandHandler = Box::new(move |gcmd: &GCmd| cb.cmd_angle_calibrate(gcmd));
            gcode.register_mux_command(
                "ANGLE_CALIBRATE",
                "CHIP",
                Some(&cname),
                handler,
                Some(ANGLE_CALIBRATE_HELP),
            )?;
        }
        Ok(this)
    }

    /// `handle_sync_mcu_pos` (lines 52-54).
    pub fn handle_sync_mcu_pos(&self, args: &[EventArg]) {
        let Some(arg) = args.first() else { return };
        let stepper = arg
            .as_handle::<dyn klippy_api::stepper::McuStepperApi>()
            .or_else(|| stepper_from_object(arg));
        let Some(stepper) = stepper else { return };
        if Some(stepper.get_name(None)) == self.stepper_name {
            self.mcu_pos_offset.replace(None);
        }
    }

    /// `calc_mcu_pos_offset` (lines 55-73).
    pub fn calc_mcu_pos_offset(&self, sample: (f64, i64)) -> KResult<()> {
        let (mcu_phase_offset, phases) = self.tmc_phase_offset()?;
        let Some(mcu_phase_offset) = mcu_phase_offset else {
            return Ok(());
        };
        let (angle_time, angle_pos) = sample;
        let mcu_stepper: Stepper = self
            .mcu_stepper
            .borrow()
            .clone()
            .ok_or_else(|| super::no_attribute_error("AngleCalibration", "mcu_stepper"))?;
        let mcu_pos: i64 = mcu_stepper.get_past_mcu_position(angle_time);
        let (microsteps, full_steps) = self.get_microsteps()?;
        // `angle_to_mcu_pos = full_steps * microsteps / float(1<<ANGLE_BITS)`
        let angle_to_mcu_pos: f64 =
            (full_steps * microsteps) as f64 / ((1i64 << ANGLE_BITS) as f64);
        let angle_mpos: f64 = (angle_pos as f64) * angle_to_mcu_pos;
        // `phase_diff = ((angle_mpos + off * k) - (mcu_pos + phase)) % phases`
        let left: f64 =
            angle_mpos + self.angle_phase_offset.get() * angle_to_mcu_pos;
        let right: f64 = (mcu_pos + mcu_phase_offset) as f64;
        let mut phase_diff: f64 = fmod(left - right, phases as f64)?;
        // `if phase_diff > phases//2: phase_diff -= phases`
        if phase_diff > (phases / 2) as f64 {
            phase_diff -= phases as f64;
        }
        // `mcu_pos_offset = mcu_pos - (angle_mpos - phase_diff)`
        let offset: f64 = (mcu_pos as f64) - (angle_mpos - phase_diff);
        self.mcu_pos_offset.replace(Some(offset));
        Ok(())
    }

    /// `apply_calibration` (lines 74-98).
    pub fn apply_calibration(
        &self,
        samples: &mut [(f64, i64)],
    ) -> KResult<Option<f64>> {
        if !self.has_table {
            // Early-`__init__`-return object: `self.calibration`
            // never existed (Python AttributeError).
            return Err(super::no_attribute_error("AngleCalibration", "calibration"));
        }
        let calibration: Vec<i64> = self.calibration.borrow().clone();
        if calibration.is_empty() {
            return Ok(None);
        }
        let calibration_reversed: bool = self.calibration_reversed.get();
        let interp_bits: i64 = ANGLE_BITS - CALIBRATION_BITS;
        let interp_mask: i64 = (1i64 << interp_bits) - 1;
        let interp_round: i64 = 1i64 << (interp_bits - 1);
        for entry in samples.iter_mut() {
            let angle = entry.1;
            let bucket = ((angle & 0xffff) >> interp_bits) as usize;
            let cal1 = calibration[bucket];
            let cal2 = calibration[bucket + 1];
            let adj = (angle & interp_mask) * (cal2 - cal1);
            let adj = cal1 + ((adj + interp_round) >> interp_bits);
            let mut angle_diff = (adj - angle) & 0xffff;
            angle_diff -= (angle_diff & 0x8000) << 1;
            let mut new_angle = angle + angle_diff;
            if calibration_reversed {
                new_angle = -new_angle;
            }
            entry.1 = new_angle;
        }
        if self.mcu_pos_offset.borrow().is_none() {
            if samples.is_empty() {
                return Err(index_error());
            }
            let sample = samples[0];
            self.calc_mcu_pos_offset(sample)?;
            if self.mcu_pos_offset.borrow().is_none() {
                return Ok(None);
            }
        }
        let offset: f64 = self.mcu_pos_offset.get().expect("offset set above");
        let mcu_stepper: Stepper = self
            .mcu_stepper
            .borrow()
            .clone()
            .ok_or_else(|| super::no_attribute_error("AngleCalibration", "mcu_stepper"))?;
        Ok(Some(mcu_to_commanded_position(&mcu_stepper, offset)))
    }

    /// `load_calibration` with the `numpy.linalg.lstsq` solve
    /// replaced by [`lstsq_min_norm`] (lines 99-130).
    pub fn load_calibration(&self, angles: &[f64]) -> KResult<()> {
        let angle_max: i64 = 1i64 << ANGLE_BITS;
        let calibration_count: i64 = 1i64 << CALIBRATION_BITS;
        let bucket_size: i64 = angle_max / calibration_count;
        let full_steps: i64 = angles.len() as i64;
        let nominal_step: f64 = float_div(angle_max as f64, full_steps as f64)?;
        let (min_idx, _) = first_min_index(angles);
        self.angle_phase_offset
            .replace(((min_idx as i64) & 3) as f64 * nominal_step);
        if angles.len() < 2 {
            return Err(index_error());
        }
        let reversed = angles[angles.len() - 2] > angles[angles.len() - 1];
        self.calibration_reversed.replace(reversed);
        let mut angles = angles.to_vec();
        if reversed {
            angles.reverse();
        }
        let (first_step, _) = first_min_index(&angles);
        let angles: Vec<f64> = angles[first_step..]
            .iter()
            .chain(angles[..first_step].iter())
            .copied()
            .collect();
        let m = full_steps as usize;
        let n = calibration_count as usize;
        let mut eqs: Vec<Vec<f64>> = vec![vec![0.0; n]; m];
        let mut ans: Vec<f64> = vec![0.0; m];
        for (step, angle) in angles.iter().enumerate() {
            let int_angle = int_mod65536(angle + 0.5);
            let bucket: i64 = int_angle / bucket_size;
            let bucket_start: i64 = bucket * bucket_size;
            let ang_diff: f64 = angle - bucket_start as f64;
            let ang_diff_per: f64 = float_div(ang_diff, bucket_size as f64)?;
            let eq = &mut eqs[step];
            eq[bucket as usize] = 1.0 - ang_diff_per;
            eq[((bucket + 1) % calibration_count) as usize] = ang_diff_per;
            ans[step] = (step as f64) * nominal_step;
            if bucket + 1 >= calibration_count {
                ans[step] -= ang_diff_per * (angle_max as f64);
            }
        }
        let sol: Vec<f64> = lstsq_min_norm(&eqs, &ans);
        let isol: Vec<i64> = sol
            .iter()
            .map(|s| py_trunc_i64(s + 0.5))
            .collect();
        let mut calib = isol.clone();
        let first = *isol.first().unwrap_or(&0);
        calib.push(first + angle_max);
        self.calibration.replace(calib);
        Ok(())
    }

    /// `lookup_tmc` (lines 131-138).
    pub fn lookup_tmc(&self) -> KResult<ObjectRef> {
        let stepper = self.stepper_name.as_deref().unwrap_or("");
        for driver in TRINAMIC_DRIVERS {
            let driver_name = format!("{driver} {stepper}");
            if let Some(module) =
                self.printer.lookup_object(&driver_name, LookupDefault::ReturnNone)?
            {
                return Ok(module);
            }
        }
        Err(KlippyError::command(format!(
            "Unable to find TMC driver for {stepper}"
        )))
    }

    /// `connect` (lines 139-142).
    pub fn connect(&self) -> KResult<()> {
        let tmc = self.lookup_tmc()?;
        self.tmc_module.replace(Some(tmc));
        let force_move = lookup_interface::<dyn ForceMoveApi>(
            &*self.printer,
            "force_move",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let stepper_name: String = self.stepper_name.clone().unwrap_or_default();
        let stepper = force_move.lookup_stepper(Some(&stepper_name))?;
        self.mcu_stepper.replace(Some(stepper));
        Ok(())
    }

    fn tmc_phase_offset(&self) -> KResult<(Option<i64>, i64)> {
        let module = self
            .tmc_module
            .borrow()
            .clone()
            .ok_or_else(|| super::no_attribute_error("AngleCalibration", "tmc_module"))?;
        let driver = require_interface::<dyn TMCCommandHelperApi>(&module, "tmc")?;
        Ok(driver.get_phase_offset())
    }

    /// `get_microsteps` (lines 143-149).
    pub fn get_microsteps(&self) -> KResult<(i64, i64)> {
        let stepper_name: String = self.stepper_name.clone().unwrap_or_default();
        get_microsteps_from_configfile(&self.printer, &stepper_name)
    }

    /// `get_stepper_phase` (lines 150-156).
    pub fn get_stepper_phase(&self) -> KResult<i64> {
        let (mcu_phase_offset, phases) = self.tmc_phase_offset()?;
        let Some(mcu_phase_offset) = mcu_phase_offset else {
            let stepper = self.stepper_name.as_deref().unwrap_or("");
            return Err(KlippyError::command(format!(
                "Driver phase not known for {stepper}"
            )));
        };
        let mcu_stepper: Stepper = self
            .mcu_stepper
            .borrow()
            .clone()
            .ok_or_else(|| super::no_attribute_error("AngleCalibration", "mcu_stepper"))?;
        let mcu_pos: i64 = mcu_stepper.get_mcu_position(None);
        Ok((mcu_pos + mcu_phase_offset).rem_euclid(phases))
    }

    /// `do_calibration_moves` (lines 157-215).
    pub fn do_calibration_moves(&self) -> KResult<CalibrationMaps> {
        let force_move = lookup_interface::<dyn ForceMoveApi>(
            &*self.printer,
            "force_move",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        // Start data collection (lines 159-167).
        let msgs_cell: Arc<GCell<Vec<PyValue>>> = Arc::new(GCell::new(Vec::new()));
        let finished_cell: Arc<GCell<bool>> = Arc::new(GCell::new(false));
        {
            let angle = lookup_interface::<dyn AngleApi>(
                &*self.printer,
                &self.name,
                LookupDefault::Raise,
            )?
            .expect("LookupDefault::Raise never yields None");
            let msgs = Arc::clone(&msgs_cell);
            let finished = Arc::clone(&finished_cell);
            let handle_batch: ClientCallback = Box::new(move |msg: PyValue| {
                if finished.get() {
                    return Ok(false);
                }
                msgs.with_mut(|m| m.push(msg));
                Ok(true)
            });
            angle.add_client(handle_batch)?;
        }
        // Move stepper several turns (to allow internal sensor calibration).
        let (microsteps, full_steps) = self.get_microsteps()?;
        let mcu_stepper: Stepper = self
            .mcu_stepper
            .borrow()
            .clone()
            .ok_or_else(|| super::no_attribute_error("AngleCalibration", "mcu_stepper"))?;
        let step_dist: f64 = mcu_stepper.get_step_dist();
        let full_step_dist: f64 = step_dist * (microsteps as f64);
        let rotation_dist: f64 = (full_steps as f64) * full_step_dist;
        let align_dist: f64 = step_dist * (self.get_stepper_phase()? as f64);
        let move_time: f64 = 0.010;
        let move_speed: f64 = float_div(full_step_dist, move_time)?;
        let manual_move = |dist: f64| -> KResult<()> {
            force_move.manual_move(&mcu_stepper, dist, move_speed, 0.0)
        };
        manual_move(-(rotation_dist + align_dist))?;
        manual_move(2.0 * rotation_dist)?;
        manual_move(-2.0 * rotation_dist)?;
        manual_move(0.5 * rotation_dist - full_step_dist)?;
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let mut times: Vec<(f64, f64)> = Vec::new();
        let mut samp_dist: f64 = full_step_dist;
        for i in 0..(2 * full_steps) {
            manual_move(samp_dist)?;
            let start_query_time: f64 = toolhead.get_last_move_time()? + 0.050;
            let end_query_time: f64 = start_query_time + 0.050;
            times.push((start_query_time, end_query_time));
            toolhead.dwell(0.150)?;
            if i == full_steps - 1 {
                manual_move(0.5 * rotation_dist)?;
                manual_move(-0.5 * rotation_dist + samp_dist)?;
                samp_dist = -samp_dist;
            }
        }
        manual_move(0.5 * rotation_dist + align_dist)?;
        toolhead.wait_moves()?;
        finished_cell.replace(true);
        let msgs: Vec<PyValue> = msgs_cell.with_mut(std::mem::take);
        let mut cal: Vec<Option<Vec<i64>>> = vec![None; times.len()];
        let mut step: usize = 0;
        for msg in &msgs {
            let data_val = dict_get(msg, "data")?;
            let PyValue::List(entries) = data_val else {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'PyValue' object is not iterable".to_string(),
                });
            };
            for entry in entries {
                let PyValue::Tuple(pair) = entry else {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: "cannot unpack non-iterable PyValue object".to_string(),
                    });
                };
                if pair.len() != 2 {
                    return Err(index_error());
                }
                let query_time: f64 =
                    pyvalue_to_f64(&pair[0]).unwrap_or(f64::NAN);
                let pos: i64 = pyvalue_to_i64(&pair[1]).unwrap_or(0);
                while step < times.len() && query_time > times[step].1 {
                    step += 1;
                }
                if step < times.len() && query_time >= times[step].0 {
                    cal[step]
                        .get_or_insert_with(Vec::new)
                        .push(pos);
                }
            }
        }
        let filled: usize = cal.iter().filter(|c| c.is_some()).count();
        if filled != times.len() {
            return Err(KlippyError::command(
                "Failed calibration - incomplete sensor data",
            ));
        }
        let mut fcal = IndexMap::new();
        for i in 0..full_steps {
            fcal.insert(i, cal[i as usize].take().unwrap_or_default());
        }
        let mut rcal = IndexMap::new();
        for i in 0..full_steps {
            rcal.insert(full_steps - i - 1, cal[(i + full_steps) as usize].take().unwrap_or_default());
        }
        Ok((fcal, rcal))
    }

    /// `calc_angles` (lines 216-225).
    pub fn calc_angles(
        &self,
        meas: &IndexMap<i64, Vec<i64>>,
    ) -> KResult<(IndexMap<i64, f64>, f64, i64)> {
        let mut angles = IndexMap::new();
        let mut total_count: i64 = 0;
        let mut total_variance: f64 = 0.0;
        for (step, data) in meas.iter() {
            let count: i64 = data.len() as i64;
            let sum: i64 = data.iter().sum();
            let angle_avg: f64 = float_div(sum as f64, count as f64)?;
            angles.insert(*step, angle_avg);
            total_count += count;
            // `sum([(d - angle_avg)**2 for d in data])` accumulates on its
            // own before joining `total_variance`; `**` calls libm `pow`.
            let mut step_variance: f64 = 0.0;
            for d in data {
                step_variance += pycompat::libm::pow((*d as f64) - angle_avg, 2.0);
            }
            total_variance += step_variance;
        }
        if total_count == 0 {
            // `0 / 0` on two ints.
            return Err(KlippyError::Internal {
                class_name: "ZeroDivisionError".to_string(),
                message: "division by zero".to_string(),
            });
        }
        let std: f64 =
            pycompat::libm::sqrt(float_div(total_variance, total_count as f64)?);
        Ok((angles, std, total_count))
    }

    /// `cmd_ANGLE_CALIBRATE` (lines 227-265).
    pub fn cmd_angle_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let old_calibration: Vec<i64> = self.calibration.borrow().clone();
        self.calibration.replace(Vec::new());
        let moves = self.do_calibration_moves();
        self.calibration.replace(old_calibration);
        let (fcal, rcal) = moves?;
        let (_microsteps, full_steps) = self.get_microsteps()?;
        let (fangles, fstd, _ftotal) = self.calc_angles(&fcal)?;
        let (rangles, rstd, _rtotal) = self.calc_angles(&rcal)?;
        if has_duplicate_floats(&fangles) || has_duplicate_floats(&rangles) {
            return Err(KlippyError::command(
                "Failed calibration - sensor not updating for each step",
            ));
        }
        let mut merged = IndexMap::new();
        for i in 0..full_steps {
            let mut combined: Vec<i64> = fcal.get(&i).cloned().unwrap_or_default();
            combined.extend(rcal.get(&i).cloned().unwrap_or_default());
            merged.insert(i, combined);
        }
        let (angles, std, total) = self.calc_angles(&merged)?;
        gcmd.respond_info(
            &fmt_const(
                "angle: stddev=%.3f (%.3f forward / %.3f reverse) in %d queries",
                &[pct_float(std), pct_float(fstd), pct_float(rstd), pct_int(total)],
            ),
            true,
        );
        let mut anglist: Vec<f64> = Vec::new();
        for i in 0..full_steps {
            let a = angles.get(&i).copied().unwrap_or(0.0);
            anglist.push(fmod(a, 0xffff as f64)?);
        }
        let first_ang: f64 = if *angles.get(&0).ok_or_else(|| key_error("0"))?
            > *angles.get(&1).ok_or_else(|| key_error("1"))?
        {
            anglist
                .iter()
                .copied()
                .fold(f64::MIN, f64::max)
        } else {
            anglist
                .iter()
                .copied()
                .fold(f64::MAX, f64::min)
        };
        let first_phase: i64 =
            anglist
                .iter()
                .position(|x| *x == first_ang)
                .map(|i| i as i64)
                .unwrap_or(0)
                & !3i64;
        let first_phase = first_phase as usize;
        let mut rotated: Vec<f64> = Vec::new();
        rotated.extend_from_slice(&anglist[first_phase..]);
        rotated.extend_from_slice(&anglist[..first_phase]);
        let mut cal_contents: Vec<String> = Vec::new();
        for (i, angle) in rotated.iter().enumerate() {
            if i % 8 == 0 {
                cal_contents.push("\n".to_string());
            }
            cal_contents.push(fmt_const("%.1f", &[pct_float(*angle)]));
            cal_contents.push(",".to_string());
        }
        cal_contents.pop();
        let joined = cal_contents.join("");
        let obj = self
            .printer
            .lookup_object("configfile", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let configfile =
            require_interface::<dyn PrinterConfigApi>(&obj, "configfile")?;
        configfile.remove_section(&self.name);
        configfile.set(&self.name, "calibrate", &joined)?;
        Ok(())
    }

    /// `get_name` parity: the full section name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Test-visible state snapshot.
    pub fn calibration_table(&self) -> Vec<i64> {
        self.calibration.borrow().clone()
    }
    /// Test-visible phase offset.
    pub fn angle_phase_offset(&self) -> f64 {
        self.angle_phase_offset.get()
    }
    /// Test-visible reversed flag.
    pub fn calibration_reversed(&self) -> bool {
        self.calibration_reversed.get()
    }
    /// Test-visible mcu position offset.
    pub fn mcu_pos_offset(&self) -> Option<f64> {
        self.mcu_pos_offset.get()
    }
}

fn parse_config_float(s: &str) -> KResult<f64> {
    pycompat::text::float_from_string(s).map_err(super::map_py_error)
}

fn stepper_from_object(arg: &EventArg) -> Option<Stepper> {
    match arg {
        EventArg::Object(obj) => {
            klippy_api::interface::<dyn klippy_api::stepper::McuStepperApi>(obj)
        }
        _ => None,
    }
}

fn first_min_index(values: &[f64]) -> (usize, f64) {
    if values.is_empty() {
        return (0, 0.0);
    }
    let mut best = 0usize;
    for i in 1..values.len() {
        if values[i] < values[best] {
            best = i;
        }
    }
    (best, values[best])
}

/// `int(angle + .5) % 65536` with Python modulo semantics.
fn int_mod65536(angle_plus_half: f64) -> i64 {
    let t = super::trunc_bigint(angle_plus_half);
    let r = pycompat::int::int_mod(&t, &PyInt::from_i64(1i64 << ANGLE_BITS));
    r.0.to_string().parse::<i64>().unwrap_or(0)
}

/// `configfile.get_status(None)['settings'][stepper]` microsteps /
/// full-steps read (angle.py:143-149, shared with `HelperMT6826S`).
pub fn get_microsteps_from_configfile(
    printer: &Printer,
    stepper_name: &str,
) -> KResult<(i64, i64)> {
    let obj = printer
        .lookup_object("configfile", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    let configfile = require_interface::<dyn PrinterConfigApi>(&obj, "configfile")?;
    let status = configfile.get_status(None);
    let settings = dict_get(&status, "settings")?;
    let stconfig = match settings {
        PyValue::Dict(m) => m
            .get(&PyValue::Str(stepper_name.to_string()))
            .cloned()
            .unwrap_or(PyValue::Dict(IndexMap::new())),
        _ => PyValue::Dict(IndexMap::new()),
    };
    let microsteps = match settings_get(&stconfig, "microsteps")? {
        Some(v) => pyvalue_to_i64(v).unwrap_or(0),
        None => return Err(key_error("microsteps")),
    };
    let full_steps = match settings_get(&stconfig, "full_steps_per_rotation")? {
        Some(v) => pyvalue_to_i64(v).unwrap_or(0),
        None => return Err(key_error("full_steps_per_rotation")),
    };
    Ok((microsteps, full_steps))
}

fn settings_get<'a>(stconfig: &'a PyValue, key: &str) -> KResult<Option<&'a PyValue>> {
    match stconfig {
        PyValue::Dict(m) => Ok(m.get(&PyValue::Str(key.to_string()))),
        _ => Err(key_error(key)),
    }
}

fn has_duplicate_floats(map: &IndexMap<i64, f64>) -> bool {
    let mut values: Vec<f64> = map.values().copied().collect();
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    for w in values.windows(2) {
        if w[0] == w[1] {
            return true;
        }
    }
    false
}

/// `mcu_stepper.mcu_to_commanded_position(offset)` where the Python
/// passes a `float` offset but the contract takes `i64`.
///
/// `mcu_to_commanded_position(x) = x * step_dist - mcu_position_offset`,
/// so `mcu_to_commanded_position(0)` is `-mcu_position_offset` exactly and
/// `offset * step_dist - mcu_position_offset` is rebuilt in the Python
/// operation order. Taking `f64` in `McuStepperApi::mcu_to_commanded_position`
/// removes the need for this helper.
pub fn mcu_to_commanded_position(stepper: &Stepper, offset: f64) -> f64 {
    let neg_mcu_offset: f64 = stepper.mcu_to_commanded_position(0);
    offset * stepper.get_step_dist() - (0.0 - neg_mcu_offset)
}

// ---------------------------------------------------------------------------
// Minimum-norm least squares (numpy.linalg.lstsq with rcond=None)
// ---------------------------------------------------------------------------

/// One-sided Jacobi SVD minimum-norm least-squares solution of
/// `A x ~= b` (`numpy.linalg.lstsq(A, b, rcond=None)[0]`).
///
/// With `rcond=None` numpy uses `cond = max(M, N) * eps` (`eps` the
/// float64 machine epsilon); singular values at or below
/// `cond * sigma_max` are cut. `A` is `m x n` row-major.
pub fn lstsq_min_norm(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let m = a.len();
    let n = if m == 0 { 0 } else { a[0].len() };
    if n == 0 {
        return Vec::new();
    }
    // One-sided Jacobi orthogonalizes the columns of `w` (starting as
    // a copy of `a`), accumulating rotations in `v` (`V`).
    let mut w: Vec<Vec<f64>> = a.to_vec();
    let mut v: Vec<Vec<f64>> = vec![vec![0.0; n]; n];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    // Sweep until all column-pair correlations vanish.
    for _sweep in 0..100 {
        let mut converged = true;
        for p in 0..n {
            for q in (p + 1)..n {
                let (alpha, beta, gamma) = column_stats(&w, p, q);
                if alpha == 0.0 || beta == 0.0 {
                    continue;
                }
                if gamma.abs() <= 1e-15 * (alpha * beta).sqrt() {
                    continue;
                }
                converged = false;
                let zeta = (beta - alpha) / (2.0 * gamma);
                let t = zeta.signum() / (zeta.abs() + (1.0 + zeta * zeta).sqrt());
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = c * t;
                for row in w.iter_mut() {
                    let wp = row[p];
                    let wq = row[q];
                    row[p] = c * wp - s * wq;
                    row[q] = s * wp + c * wq;
                }
                for row in v.iter_mut() {
                    let vp = row[p];
                    let vq = row[q];
                    row[p] = c * vp - s * vq;
                    row[q] = s * vp + c * vq;
                }
            }
        }
        if converged {
            break;
        }
    }
    let mut sigma: Vec<f64> = vec![0.0; n];
    for j in 0..n {
        let mut acc = 0.0;
        for row in &w {
            acc += row[j] * row[j];
        }
        sigma[j] = acc.sqrt();
    }
    let sigma_max = sigma.iter().copied().fold(0.0, f64::max);
    // `rcond=None`: `cond = max(M, N) * eps`.
    let cutoff = (m.max(n) as f64) * f64::EPSILON * sigma_max;
    let mut x = vec![0.0f64; n];
    for j in 0..n {
        let s = sigma[j];
        if s <= cutoff {
            continue;
        }
        // u_j = w_j / sigma; coeff = (u_j . b) / sigma.
        let mut utb = 0.0;
        for (i, row) in w.iter().enumerate() {
            utb += (row[j] / s) * b[i];
        }
        let coeff = utb / s;
        for i in 0..n {
            x[i] += coeff * v[i][j];
        }
    }
    x
}

fn column_stats(w: &[Vec<f64>], p: usize, q: usize) -> (f64, f64, f64) {
    let mut alpha = 0.0;
    let mut beta = 0.0;
    let mut gamma = 0.0;
    for row in w {
        alpha += row[p] * row[p];
        beta += row[q] * row[q];
        gamma += row[p] * row[q];
    }
    (alpha, beta, gamma)
}

#[allow(dead_code)]
fn _unused_globals() -> PyValue {
    PyValue::None
}
