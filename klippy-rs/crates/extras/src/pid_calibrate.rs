//! Port of the `pid_calibrate` module (`klippy/extras/pid_calibrate.py`).
//!
//! PID autotuning for heaters: `PID_CALIBRATE` replaces the heater's control
//! object with a [`ControlAutoTune`], bang-bangs the heater around the
//! target while [`ControlAutoTune`] records peaks, then computes
//! Ziegler-Nichols PID parameters with the Astrom-Hagglund method.
//!
//! The heater-side duck types this module calls into are the `heaters`
//! module's contract traits, imported from [`crate::heaters`]:
//! [`HeaterApi`] (`heaters.py:21`), [`PrinterHeatersApi`]
//! (`heaters.py:242`) and [`ControlApi`] (`heaters.py:166,192`, the
//! algorithm interface `Heater.control` implements). `ControlAutoTune`
//! implements [`ControlApi`] so a real [`crate::heaters::Heater`] can
//! run it through `set_control`. The inherent
//! [`ControlAutoTune::temperature_update`] keeps the reference's
//! `KResult`, so the `ZeroDivisionError`/`IndexError` that
//! [`ControlAutoTune::calc_pid`] raises is observable; the
//! [`ControlApi`] implementation cannot propagate it through the
//! heater's `temperature_callback` (the trait returns `()`), matching
//! the frozen contract. [`PID_PARAM_BASE`] is imported from
//! `crate::heaters` (`heaters.py:16`).

use std::io::Write as _;
use std::sync::Arc;

use crate::heaters::{ControlApi, HeaterApi, PID_PARAM_BASE, PrinterHeatersApi};
use klippy_api::config::{Config, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts, GetIntOpts, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, GCell, ObjectRef};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;

/// `TUNE_PID_DELTA` (`klippy/extras/pid_calibrate.py:53`).
pub const TUNE_PID_DELTA: f64 = 5.0;

/// Help text of the `PID_CALIBRATE` command.
pub const CMD_PID_CALIBRATE_HELP: &str = "Run PID calibration test";

/// File written when `PID_CALIBRATE` is given `WRITE_FILE=1`.
const HEAT_TEST_FILE: &str = "/tmp/heattest.txt";

// ---------------------------------------------------------------------------
// Formatting / error helpers.
// ---------------------------------------------------------------------------

/// `fmt % args` with CPython semantics; the `%f`/`%.3f` uses in this
/// module never fail, an error is reported rather than panicking.
fn pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}
/// CPython `OSError` text for a failed `open`/`write` on `filename`:
/// `[Errno <errno>] <strerror>: '<filename>'`, with the `OSError`
/// subclass CPython maps the errno to.
fn os_error(filename: &str, e: &std::io::Error) -> KlippyError {
    match e.raw_os_error() {
        Some(errno) => {
            let text = e.to_string();
            let text = text
                .strip_suffix(&format!(" (os error {errno})"))
                .unwrap_or(text.as_str());
            let class_name = match errno {
                2 => "FileNotFoundError",
                13 => "PermissionError",
                20 => "NotADirectoryError",
                21 => "IsADirectoryError",
                _ => "OSError",
            };
            KlippyError::Internal {
                class_name: class_name.to_string(),
                message: format!("[Errno {errno}] {text}: '{filename}'"),
            }
        }
        None => KlippyError::Internal {
            class_name: "OSError".to_string(),
            message: format!("{e}: '{filename}'"),
        },
    }
}

fn zero_div_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "ZeroDivisionError".to_string(),
        message: "float division by zero".to_string(),
    }
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

// ---------------------------------------------------------------------------
// `ControlAutoTune` (`klippy/extras/pid_calibrate.py:55`).
// ---------------------------------------------------------------------------

struct TuneState {
    heating: bool,
    peak: f64,
    peak_time: f64,
    peaks: Vec<(f64, f64)>,
    last_pwm: f64,
    pwm_samples: Vec<(f64, f64)>,
    temp_samples: Vec<(f64, f64)>,
}

/// The tuner: while installed as the heater's control it bang-bangs the
/// heater around the calibration target and records temperature peaks.
pub struct ControlAutoTune {
    heater: Arc<dyn HeaterApi>,
    heater_max_power: f64,
    calibrate_temp: f64,
    state: GCell<TuneState>,
}

impl ControlAutoTune {
    /// `ControlAutoTune.__init__(heater, target)` (`pid_calibrate.py:56`).
    pub fn new(heater: Arc<dyn HeaterApi>, target: f64) -> ControlAutoTune {
        let heater_max_power = heater.get_max_power();
        ControlAutoTune {
            heater,
            heater_max_power,
            calibrate_temp: target,
            state: GCell::new(TuneState {
                heating: false,
                peak: 0.,
                peak_time: 0.,
                peaks: Vec::new(),
                last_pwm: 0.,
                pwm_samples: Vec::new(),
                temp_samples: Vec::new(),
            }),
        }
    }

    /// `ControlAutoTune.set_pwm` (`pid_calibrate.py:71`): record the PWM
    /// change (offset by the heater PWM delay) and drive the heater.
    pub fn set_pwm(&self, read_time: f64, value: f64) {
        {
            let mut s = self.state.borrow_mut();
            if value != s.last_pwm {
                let delay = self.heater.get_pwm_delay();
                s.pwm_samples.push((read_time + delay, value));
                s.last_pwm = value;
            }
        }
        self.heater.set_pwm(read_time, value);
    }

    /// `ControlAutoTune.temperature_update` (`pid_calibrate.py:77`).
    pub fn temperature_update(
        &self,
        read_time: f64,
        temp: f64,
        target_temp: f64,
    ) -> KResult<()> {
        self.state
            .with_mut(|s| s.temp_samples.push((read_time, temp)));
        // Crossed the target: flip the bang-bang direction, record the
        // peak and retarget the heater.
        let new_target = {
            let mut s = self.state.borrow_mut();
            if s.heating && temp >= target_temp {
                s.heating = false;
                Some(self.calibrate_temp - TUNE_PID_DELTA)
            } else if !s.heating && temp <= target_temp {
                s.heating = true;
                Some(self.calibrate_temp)
            } else {
                None
            }
        };
        if let Some(new_target) = new_target {
            self.check_peaks()?;
            self.heater.alter_target(new_target);
        }
        if self.state.borrow().heating {
            self.set_pwm(read_time, self.heater_max_power);
            self.state.with_mut(|s| {
                if temp < s.peak {
                    s.peak = temp;
                    s.peak_time = read_time;
                }
            });
        } else {
            self.set_pwm(read_time, 0.);
            self.state.with_mut(|s| {
                if temp > s.peak {
                    s.peak = temp;
                    s.peak_time = read_time;
                }
            });
        }
        Ok(())
    }

    /// `ControlAutoTune.check_busy` (`pid_calibrate.py:100`).
    pub fn check_busy(&self, _eventtime: f64, _smoothed_temp: f64, _target_temp: f64) -> bool {
        let s = self.state.borrow();
        s.heating || s.peaks.len() < 12
    }

    /// `ControlAutoTune.check_peaks` (`pid_calibrate.py:105`).
    pub fn check_peaks(&self) -> KResult<()> {
        let pos = {
            let mut s = self.state.borrow_mut();
            let peak = (s.peak, s.peak_time);
            s.peaks.push(peak);
            if s.heating {
                s.peak = 9999999.;
            } else {
                s.peak = -9999999.;
            }
            if s.peaks.len() < 4 {
                None
            } else {
                Some((s.peaks.len() - 1) as isize)
            }
        };
        if let Some(pos) = pos {
            self.calc_pid(pos)?;
        }
        Ok(())
    }

    /// `ControlAutoTune.calc_pid` (`pid_calibrate.py:114`).
    ///
    /// `pos` follows Python indexing: a negative index counts from the end
    /// of the peak list, and an out-of-range index raises `IndexError`,
    /// exactly as `self.peaks[pos]` does in the reference.
    pub fn calc_pid(&self, pos: isize) -> KResult<(f64, f64, f64)> {
        let (temp_diff, time_diff) = {
            let peaks = self.state.borrow().peaks.clone();
            let n = peaks.len() as isize;
            let at = |idx: isize| -> KResult<(f64, f64)> {
                let i = if idx < 0 { idx + n } else { idx };
                if i < 0 || i >= n {
                    return Err(index_error());
                }
                Ok(peaks[i as usize])
            };
            let temp_diff = at(pos)?.0 - at(pos - 1)?.0;
            let time_diff = at(pos)?.1 - at(pos - 2)?.1;
            (temp_diff, time_diff)
        };
        // Astrom-Hagglund estimate of Ku and Tu.
        let amplitude = 0.5 * temp_diff.abs();
        let denominator = std::f64::consts::PI * amplitude;
        if denominator == 0.0 {
            return Err(zero_div_error());
        }
        let ku = (4. * self.heater_max_power) / denominator;
        let tu = time_diff;
        // Ziegler-Nichols PID parameters.
        let ti = 0.5 * tu;
        let td = 0.125 * tu;
        let kp = 0.6 * ku * PID_PARAM_BASE;
        if ti == 0.0 {
            return Err(zero_div_error());
        }
        let ki = kp / ti;
        let kd = kp * td;
        let msg = pct(
            "Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
            &[
                PctValue::Float(temp_diff),
                PctValue::Float(self.heater_max_power),
                PctValue::Float(ku),
                PctValue::Float(tu),
                PctValue::Float(kp),
                PctValue::Float(ki),
                PctValue::Float(kd),
            ],
        )?;
        log::info(&msg);
        Ok((kp, ki, kd))
    }

    /// `ControlAutoTune.calc_final_pid` (`pid_calibrate.py:130`): the PID
    /// parameters of the median cycle time. With fewer than five peaks the
    /// cycle list is empty and Python raises `IndexError`.
    pub fn calc_final_pid(&self) -> KResult<(f64, f64, f64)> {
        let mut cycle_times: Vec<(f64, usize)> = {
            let s = self.state.borrow();
            (4..s.peaks.len())
                .map(|pos| (s.peaks[pos].1 - s.peaks[pos - 2].1, pos))
                .collect()
        };
        // Python `sorted` compares tuples with `<` and is stable.
        cycle_times.sort_by(|a, b| {
            if a.0 < b.0 {
                std::cmp::Ordering::Less
            } else if b.0 < a.0 {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });
        let midpoint = match cycle_times.get(cycle_times.len() / 2) {
            Some(&(_, pos)) => pos as isize,
            None => return Err(index_error()),
        };
        self.calc_pid(midpoint)
    }

    /// `ControlAutoTune.write_file` (`pid_calibrate.py:136`): the PWM and
    /// temperature samples, joined with newlines and no trailing newline.
    pub fn write_file(&self, filename: &str) -> KResult<()> {
        let body = {
            let s = self.state.borrow();
            let mut lines: Vec<String> = Vec::with_capacity(
                s.pwm_samples.len() + s.temp_samples.len(),
            );
            for (time, value) in &s.pwm_samples {
                lines.push(pct(
                    "pwm: %.3f %.3f",
                    &[PctValue::Float(*time), PctValue::Float(*value)],
                )?);
            }
            for (time, temp) in &s.temp_samples {
                lines.push(pct(
                    "%.3f %.3f",
                    &[PctValue::Float(*time), PctValue::Float(*temp)],
                )?);
            }
            lines.join("\n")
        };
        let mut file = std::fs::OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(true)
            .open(filename)
            .map_err(|e| os_error(filename, &e))?;
        file.write_all(body.as_bytes())
            .and_then(|()| file.flush())
            .map_err(|e| os_error(filename, &e))?;
        Ok(())
    }

    /// Whether the heater is currently being driven on.
    pub fn is_heating(&self) -> bool {
        self.state.borrow().heating
    }

    /// The current peak and the time it was observed.
    pub fn peak(&self) -> (f64, f64) {
        let s = self.state.borrow();
        (s.peak, s.peak_time)
    }

    /// The recorded `(temp, time)` peaks.
    pub fn peaks(&self) -> Vec<(f64, f64)> {
        self.state.borrow().peaks.clone()
    }

    /// The recorded `(time, value)` PWM changes.
    pub fn pwm_samples(&self) -> Vec<(f64, f64)> {
        self.state.borrow().pwm_samples.clone()
    }

    /// The recorded `(time, temp)` temperature samples.
    pub fn temp_samples(&self) -> Vec<(f64, f64)> {
        self.state.borrow().temp_samples.clone()
    }

    /// The last PWM value handed to [`ControlAutoTune::set_pwm`].
    pub fn last_pwm(&self) -> f64 {
        self.state.borrow().last_pwm
    }

    /// The heater's maximum power, captured at construction.
    pub fn heater_max_power(&self) -> f64 {
        self.heater_max_power
    }

    /// The calibration target.
    pub fn calibrate_temp(&self) -> f64 {
        self.calibrate_temp
    }
}

/// The tuner installed as the heater's control (`heaters.py:166` `ControlApi`
/// duck type). `temperature_update` cannot return the `KResult` of
/// [`ControlAutoTune::temperature_update`], so an error raised inside
/// `calc_pid` while the heater drives the tuner is dropped here.
impl ControlApi for ControlAutoTune {
    fn temperature_update(&self, read_time: f64, temp: f64, target_temp: f64) {
        let _ = ControlAutoTune::temperature_update(self, read_time, temp, target_temp);
    }

    fn check_busy(&self, eventtime: f64, smoothed_temp: f64, target_temp: f64) -> bool {
        ControlAutoTune::check_busy(self, eventtime, smoothed_temp, target_temp)
    }
}

// ---------------------------------------------------------------------------
// `PIDCalibrate` (`klippy/extras/pid_calibrate.py:9`).
// ---------------------------------------------------------------------------

/// The printer object behind the `PID_CALIBRATE` command.
pub struct PIDCalibrate {
    printer: Printer,
}

/// `gcmd.get('HEATER')` (required `str` parameter).
fn required_str(gcmd: &GCmd, name: &str) -> KResult<String> {
    match gcmd.get(name, GetOpts::default())? {
        PyValue::Str(s) => Ok(s),
        other => Ok(other.str()),
    }
}

/// `gcmd.get_float(name)` (required parameter).
fn required_float(gcmd: &GCmd, name: &str) -> KResult<f64> {
    match gcmd.get_float(name, GetFloatOpts::default())? {
        Some(value) => Ok(value),
        None => Err(KlippyError::command(format!(
            "Error on '{}': missing {}",
            gcmd.get_commandline(),
            name
        ))),
    }
}

/// `gcmd.get_int(name, default)` with an explicit default.
fn int_with_default(gcmd: &GCmd, name: &str, default: i64) -> KResult<i64> {
    let opts = GetIntOpts {
        default: Some(Some(default)),
        ..GetIntOpts::default()
    };
    match gcmd.get_int(name, opts)? {
        Some(value) => Ok(value),
        None => Err(KlippyError::command(format!(
            "Error on '{}': missing {}",
            gcmd.get_commandline(),
            name
        ))),
    }
}

impl PIDCalibrate {
    /// `PIDCalibrate.__init__(config)` (`pid_calibrate.py:10`); the
    /// `PID_CALIBRATE` registration itself happens in [`load_config`],
    /// which is the path the Python `__init__` is reached through.
    pub fn new(config: &Config) -> PIDCalibrate {
        PIDCalibrate {
            printer: config.get_printer(),
        }
    }

    /// `PIDCalibrate.cmd_PID_CALIBRATE` (`pid_calibrate.py:16`).
    pub fn cmd_pid_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        let heater_name = required_str(gcmd, "HEATER")?;
        let target = required_float(gcmd, "TARGET")?;
        let write_file = int_with_default(gcmd, "WRITE_FILE", 0)?;
        let pheaters = lookup_interface::<dyn PrinterHeatersApi>(
            &*self.printer,
            "heaters",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        // `pheaters.lookup_heater` returns the concrete `Arc<Heater>`;
        // `set_temperature` takes it back.
        let heater = match pheaters.lookup_heater(&heater_name) {
            Ok(heater) => heater,
            // `except printer.config_error as e: raise gcmd.error(str(e))`
            Err(KlippyError::Config(message)) => return Err(KlippyError::command(message)),
            Err(e) => return Err(e),
        };
        // The heater must see the last move time before tuning starts.
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let _ = toolhead.get_last_move_time()?;
        let heater_api: Arc<dyn HeaterApi> = heater.clone();
        let calibrate = Arc::new(ControlAutoTune::new(heater_api, target));
        let old_control = heater.set_control(calibrate.clone() as Arc<dyn ControlApi>);
        if let Err(e) = pheaters.set_temperature(&heater, target, true) {
            // `except printer.command_error: heater.set_control(old_control); raise`
            // A non-command error also propagates, without the restore,
            // mirroring the reference's narrow `except`.
            if matches!(e, KlippyError::Command(_)) {
                heater.set_control(old_control);
            }
            return Err(e);
        }
        heater.set_control(old_control);
        if write_file != 0 {
            calibrate.write_file(HEAT_TEST_FILE)?;
        }
        if calibrate.check_busy(0., 0., 0.) {
            return Err(KlippyError::command("pid_calibrate interrupted"));
        }
        let (kp, ki, kd) = calibrate.calc_final_pid()?;
        let msg = pct(
            "Autotune: final: Kp=%f Ki=%f Kd=%f",
            &[
                PctValue::Float(kp),
                PctValue::Float(ki),
                PctValue::Float(kd),
            ],
        )?;
        log::info(&msg);
        let msg = pct(
            "PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n\
             The SAVE_CONFIG command will update the printer config file\n\
             with these parameters and restart the printer.",
            &[
                PctValue::Float(kp),
                PctValue::Float(ki),
                PctValue::Float(kd),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        // Store results for SAVE_CONFIG.
        let cfgname = heater.get_name();
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        configfile.set(&cfgname, "control", "pid");
        configfile.set(&cfgname, "pid_Kp", &pct("%.3f", &[PctValue::Float(kp)])?);
        configfile.set(&cfgname, "pid_Ki", &pct("%.3f", &[PctValue::Float(ki)])?);
        configfile.set(&cfgname, "pid_Kd", &pct("%.3f", &[PctValue::Float(kd)])?);
        Ok(())
    }
}

klippy_api::printer_object!(PIDCalibrate);

/// `load_config(config)` (`pid_calibrate.py:144`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = Arc::new(PIDCalibrate::new(config));
    let gcode = lookup_interface::<dyn GCodeApi>(
        &*obj.printer,
        "gcode",
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None");
    let handler_obj = obj.clone();
    gcode.register_command(
        "PID_CALIBRATE",
        Some(Box::new(move |gcmd: &GCmd| handler_obj.cmd_pid_calibrate(gcmd))),
        RegisterCommandOpts {
            when_not_ready: false,
            desc: Some(CMD_PID_CALIBRATE_HELP.to_string()),
        },
    )?;
    Ok(Some(obj as ObjectRef))
}

/// Module entry point for the `pid_calibrate` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
