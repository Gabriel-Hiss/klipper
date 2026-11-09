//! Port of the `resonance_tester` module (`klippy/extras/resonance_tester.py`).
//!
//! Resonance test generators, the `TestAxis` direction helper, the test
//! executor and the `ResonanceTester` printer object with the
//! `MEASURE_AXES_NOISE`, `TEST_RESONANCES` and `SHAPER_CALIBRATE` commands.

use crate::adxl345::{AccelChipApi, AccelQueryHelper};
use crate::input_shaper::InputShaperApi;
use crate::shaper_calibrate::{CalibrationData, CalibrationResult, ShaperCalibrate};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetFloatOpts, GetListsOpts, GetStrOpts, ListParser, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts,
    GetOpts as GGetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{GCell, ObjectRef};
use pycompat::numpy;
use pycompat::text::{float_from_string, percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// `%`-formatting helpers (`"..." % (...)` in the reference).
// ---------------------------------------------------------------------------

fn pf(v: f64) -> PctValue {
    PctValue::Float(v)
}

fn ps(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}

fn fmt_pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|e| e.display())
}

fn fmt1(fmt: &str, a: PctValue) -> String {
    fmt_pct(fmt, &[a])
}

// ---------------------------------------------------------------------------
// Errors the reference raises through Python machinery.
// ---------------------------------------------------------------------------

fn attribute_error(type_name: &str, attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'{type_name}' object has no attribute '{attr}'"),
    }
}

fn key_error(key: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: format!("'{key}'"),
    }
}

/// `a > b` where `b` may be `None` (`TypeError` in Python).
fn py_gt(a: f64, b: Option<f64>) -> KResult<bool> {
    match b {
        Some(b) => Ok(a > b),
        None => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "'>' not supported between instances of 'float' and 'NoneType'".to_string(),
        }),
    }
}

/// `float(v)` for the numeric values the status dicts carry.
fn py_float(v: &PyValue) -> Option<f64> {
    match v {
        PyValue::Float(f) => Some(*f),
        PyValue::Int(n) => n.0.to_string().parse::<f64>().ok(),
        PyValue::Bool(b) => Some(if *b { 1. } else { 0. }),
        _ => None,
    }
}

fn dict_get<'a>(d: &'a IndexMap<PyValue, PyValue>, key: &str) -> Option<&'a PyValue> {
    d.get(&PyValue::Str(key.to_string()))
}

// ---------------------------------------------------------------------------
// `TestAxis` (`klippy/extras/resonance_tester.py:9`).
// ---------------------------------------------------------------------------

/// One entry of a generated test sequence: `(time, accel, freq)`.
pub type TestSeqItem = (f64, f64, f64);

/// `TestAxis` (`resonance_tester.py:9-37`): a vibration direction.
#[derive(Clone, Debug, PartialEq)]
pub struct TestAxis {
    name: String,
    vib_dir: Vec<f64>,
}

impl TestAxis {
    /// `TestAxis(axis='x'|'y'|'z')`: the unit direction of one axis.
    pub fn new_axis(axis: &str) -> KResult<TestAxis> {
        // `ord(axis)` raises TypeError for a string that is not one character;
        // `[...][ord(axis) - ord('x')]` indexes a three-entry list, so the
        // offsets -3..=2 are valid (negative ones wrap) and anything else is
        // an IndexError.
        let mut chars = axis.chars();
        let c = match (chars.next(), chars.next()) {
            (Some(c), None) => c,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "ord() expected a character, but string of length {} found",
                        axis.chars().count()
                    ),
                })
            }
        };
        let offset = c as i64 - 'x' as i64;
        let index = if offset < 0 { offset + 3 } else { offset };
        let vib_dir = match index {
            0 => vec![1., 0., 0.],
            1 => vec![0., 1., 0.],
            2 => vec![0., 0., 1.],
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        Ok(TestAxis { name: axis.to_string(), vib_dir })
    }

    /// `TestAxis(vib_dir=(x, y, z))`, normalised to unit length.
    pub fn new_vib_dir(vib_dir: &[f64]) -> KResult<TestAxis> {
        let name = fmt_pct(
            "axis=%.3f,%.3f,%.3f",
            &[
                pf(vib_dir[0]),
                pf(vib_dir[1]),
                pf(if vib_dir.len() == 3 { vib_dir[2] } else { 0. }),
            ],
        );
        let mut sum = 0f64;
        for &d in vib_dir {
            sum += d * d;
        }
        let s = pycompat::libm::sqrt(sum);
        if s == 0.0 {
            return Err(KlippyError::Internal {
                class_name: "ZeroDivisionError".to_string(),
                message: "float division by zero".to_string(),
            });
        }
        Ok(TestAxis { name, vib_dir: vib_dir.iter().map(|&d| d / s).collect() })
    }

    /// `matches(chip_axis)`.
    pub fn matches(&self, chip_axis: &str) -> bool {
        if self.vib_dir[0] != 0.0 && chip_axis.contains('x') {
            return true;
        }
        if self.vib_dir[1] != 0.0 && chip_axis.contains('y') {
            return true;
        }
        if self.vib_dir[2] != 0.0 && chip_axis.contains('z') {
            return true;
        }
        false
    }

    /// `get_dir()`.
    pub fn get_dir(&self) -> &[f64] {
        &self.vib_dir
    }

    /// `get_name()`.
    pub fn get_name(&self) -> &str {
        &self.name
    }

    /// `get_point(l)`.
    pub fn get_point(&self, l: f64) -> Vec<f64> {
        self.vib_dir.iter().map(|&d| d * l).collect()
    }
}

fn parse_float_field(raw: &str) -> Result<f64, ()> {
    float_from_string(raw.trim()).map_err(|_| ())
}

/// `_parse_axis(gcmd, raw_axis)` (`resonance_tester.py:39-54`).
pub fn parse_axis(
    _gcmd: &dyn GCodeCommandApi,
    raw_axis: Option<&str>,
) -> KResult<Option<TestAxis>> {
    let raw = match raw_axis {
        None => return Ok(None),
        Some(s) => s,
    };
    let lowered = raw.to_lowercase();
    if lowered == "x" || lowered == "y" || lowered == "z" {
        return Ok(Some(TestAxis::new_axis(&lowered)?));
    }
    let dirs: Vec<&str> = lowered.split(',').collect();
    if dirs.len() != 2 && dirs.len() != 3 {
        return Err(KlippyError::Command(fmt1(
            "Invalid format of axis '%s'",
            ps(&lowered),
        )));
    }
    let dir_x = parse_float_field(dirs[0]).map_err(|_| {
        KlippyError::Command(fmt1("Unable to parse axis direction '%s'", ps(&lowered)))
    })?;
    let dir_y = parse_float_field(dirs[1]).map_err(|_| {
        KlippyError::Command(fmt1("Unable to parse axis direction '%s'", ps(&lowered)))
    })?;
    let dir_z = if dirs.len() == 3 {
        parse_float_field(dirs[2]).map_err(|_| {
            KlippyError::Command(fmt1("Unable to parse axis direction '%s'", ps(&lowered)))
        })?
    } else {
        0.
    };
    Ok(Some(TestAxis::new_vib_dir(&[dir_x, dir_y, dir_z])?))
}

// ---------------------------------------------------------------------------
// `VibrationPulseTestGenerator` (`resonance_tester.py:57-96`).
// ---------------------------------------------------------------------------

/// `VibrationPulseTestGenerator` (`resonance_tester.py:57`).
#[derive(Clone, Debug, PartialEq)]
pub struct VibrationPulseTestGenerator {
    pub min_freq: f64,
    pub max_freq: f64,
    pub max_freq_z: f64,
    pub accel_per_hz: f64,
    pub accel_per_hz_z: f64,
    pub hz_per_sec: f64,
    freq_start: f64,
    freq_end: f64,
    test_accel_per_hz: f64,
    test_hz_per_sec: f64,
}

impl VibrationPulseTestGenerator {
    /// `__init__(config)`.
    pub fn new(config: &Config) -> KResult<Self> {
        let min_freq = config
            .getfloat(
                "min_freq",
                GetFloatOpts { default: Some(Some(5.)), minval: Some(1.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let max_freq = config
            .getfloat(
                "max_freq",
                GetFloatOpts {
                    default: Some(Some(135.)),
                    minval: Some(min_freq),
                    maxval: Some(300.),
                    ..Default::default()
                },
            )?
            .expect("getfloat with a default returns a value");
        let max_freq_z = config
            .getfloat(
                "max_freq_z",
                GetFloatOpts {
                    default: Some(Some(100.)),
                    minval: Some(min_freq),
                    maxval: Some(300.),
                    ..Default::default()
                },
            )?
            .expect("getfloat with a default returns a value");
        let accel_per_hz = config
            .getfloat(
                "accel_per_hz",
                GetFloatOpts { default: Some(Some(60.)), above: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let accel_per_hz_z = config
            .getfloat(
                "accel_per_hz_z",
                GetFloatOpts { default: Some(Some(15.)), above: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let hz_per_sec = config
            .getfloat(
                "hz_per_sec",
                GetFloatOpts {
                    default: Some(Some(1.)),
                    minval: Some(0.1),
                    maxval: Some(2.),
                    ..Default::default()
                },
            )?
            .expect("getfloat with a default returns a value");
        Ok(VibrationPulseTestGenerator {
            min_freq,
            max_freq,
            max_freq_z,
            accel_per_hz,
            accel_per_hz_z,
            hz_per_sec,
            freq_start: min_freq,
            freq_end: max_freq,
            test_accel_per_hz: accel_per_hz,
            test_hz_per_sec: hz_per_sec,
        })
    }

    /// `prepare_test(gcmd, is_z)`.
    pub fn prepare_test(&mut self, gcmd: &dyn GCodeCommandApi, is_z: bool) -> KResult<()> {
        self.freq_start = gcmd
            .get_float(
                "FREQ_START",
                GGetFloatOpts {
                    default: Some(Some(self.min_freq)),
                    minval: Some(1.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        let freq_end_default = if is_z { self.max_freq_z } else { self.max_freq };
        self.freq_end = gcmd
            .get_float(
                "FREQ_END",
                GGetFloatOpts {
                    default: Some(Some(freq_end_default)),
                    minval: Some(self.freq_start),
                    maxval: Some(300.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        let accel_default = if is_z { self.accel_per_hz_z } else { self.accel_per_hz };
        self.test_accel_per_hz = gcmd
            .get_float(
                "ACCEL_PER_HZ",
                GGetFloatOpts {
                    default: Some(Some(accel_default)),
                    above: Some(0.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        self.test_hz_per_sec = gcmd
            .get_float(
                "HZ_PER_SEC",
                GGetFloatOpts {
                    default: Some(Some(self.hz_per_sec)),
                    above: Some(0.),
                    maxval: Some(2.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        Ok(())
    }

    /// `gen_test()`.
    pub fn gen_test(&self) -> Vec<TestSeqItem> {
        let mut freq = self.freq_start;
        let mut res: Vec<TestSeqItem> = Vec::new();
        let mut sign = 1f64;
        let mut time = 0f64;
        while freq <= self.freq_end + 0.000001 {
            let t_seg = 0.25 / freq;
            let accel = self.test_accel_per_hz * freq;
            time += t_seg;
            res.push((time, sign * accel, freq));
            time += t_seg;
            res.push((time, -sign * accel, freq));
            freq += 2. * t_seg * self.test_hz_per_sec;
            sign = -sign;
        }
        res
    }

    /// `get_max_freq()`.
    pub fn get_max_freq(&self) -> f64 {
        self.freq_end
    }
}

// ---------------------------------------------------------------------------
// `SweepingVibrationsTestGenerator` (`resonance_tester.py:97-152`).
// ---------------------------------------------------------------------------

/// `SweepingVibrationsTestGenerator` (`resonance_tester.py:97`).
#[derive(Clone, Debug, PartialEq)]
pub struct SweepingVibrationsTestGenerator {
    pub vibration_generator: VibrationPulseTestGenerator,
    pub sweeping_accel: f64,
    pub sweeping_accel_z: f64,
    pub sweeping_period: f64,
    test_sweeping_accel: f64,
    test_sweeping_period: f64,
}

impl SweepingVibrationsTestGenerator {
    /// `__init__(config)`.
    pub fn new(config: &Config) -> KResult<Self> {
        let vibration_generator = VibrationPulseTestGenerator::new(config)?;
        let sweeping_accel = config
            .getfloat(
                "sweeping_accel",
                GetFloatOpts { default: Some(Some(400.)), above: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let sweeping_accel_z = config
            .getfloat(
                "sweeping_accel_z",
                GetFloatOpts { default: Some(Some(50.)), above: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let sweeping_period = config
            .getfloat(
                "sweeping_period",
                GetFloatOpts { default: Some(Some(1.2)), minval: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        Ok(SweepingVibrationsTestGenerator {
            vibration_generator,
            sweeping_accel,
            sweeping_accel_z,
            sweeping_period,
            test_sweeping_accel: sweeping_accel,
            test_sweeping_period: sweeping_period,
        })
    }

    /// `prepare_test(gcmd, is_z)`.
    pub fn prepare_test(&mut self, gcmd: &dyn GCodeCommandApi, is_z: bool) -> KResult<()> {
        self.vibration_generator.prepare_test(gcmd, is_z)?;
        let accel_default = if is_z { self.sweeping_accel_z } else { self.sweeping_accel };
        self.test_sweeping_accel = gcmd
            .get_float(
                "SWEEPING_ACCEL",
                GGetFloatOpts {
                    default: Some(Some(accel_default)),
                    above: Some(0.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        self.test_sweeping_period = gcmd
            .get_float(
                "SWEEPING_PERIOD",
                GGetFloatOpts {
                    default: Some(Some(self.sweeping_period)),
                    minval: Some(0.),
                    ..Default::default()
                },
            )?
            .expect("get_float with a default returns a value");
        Ok(())
    }

    /// `gen_test()`.
    pub fn gen_test(&self) -> Vec<TestSeqItem> {
        let test_seq = self.vibration_generator.gen_test();
        let mut accel_fraction = pycompat::libm::sqrt(2.0) * 0.125;
        let (mut t_rem, sweeping_accel) = if self.test_sweeping_period != 0. {
            (self.test_sweeping_period * accel_fraction, self.test_sweeping_accel)
        } else {
            (f64::INFINITY, 0.)
        };
        let mut res: Vec<TestSeqItem> = Vec::new();
        let mut last_t = 0f64;
        let mut sig = 1f64;
        accel_fraction += 0.25;
        for &(next_t, accel, freq) in test_seq.iter() {
            let mut t_seg = next_t - last_t;
            while t_rem <= t_seg {
                last_t += t_rem;
                res.push((last_t, accel + sweeping_accel * sig, freq));
                t_seg -= t_rem;
                t_rem = self.test_sweeping_period * accel_fraction;
                accel_fraction = 0.5;
                sig = -sig;
            }
            t_rem -= t_seg;
            res.push((next_t, accel + sweeping_accel * sig, freq));
            last_t = next_t;
        }
        res
    }

    /// `get_max_freq()`.
    pub fn get_max_freq(&self) -> f64 {
        self.vibration_generator.get_max_freq()
    }
}

// ---------------------------------------------------------------------------
// `lookup_z_limits` (`resonance_tester.py:142-151`).
// ---------------------------------------------------------------------------

/// `lookup_z_limits(configfile)`.
///
/// Both limits stay `Option<f64>`: the reference compares them against the
/// test maxima only afterwards, where a missing value is a `TypeError`.
pub fn lookup_z_limits(configfile: &dyn PrinterConfigApi) -> KResult<(Option<f64>, Option<f64>)> {
    let status = configfile.get_status(None);
    let status_dict = match &status {
        PyValue::Dict(d) => d,
        _ => return Err(attribute_error(pycompat::value::type_name(&status), "get")),
    };
    let settings = dict_get(status_dict, "settings").ok_or_else(|| key_error("settings"))?;
    let settings_dict = match settings {
        PyValue::Dict(d) => d,
        other => return Err(attribute_error(pycompat::value::type_name(other), "get")),
    };
    // `sconfig.get('printer')` returns None when the section was never read.
    let printer_dict = match dict_get(settings_dict, "printer") {
        Some(PyValue::Dict(d)) => d,
        Some(other) => return Err(attribute_error(pycompat::value::type_name(other), "get")),
        None => return Err(attribute_error("NoneType", "get")),
    };
    let get_opt = |key: &str| dict_get(printer_dict, key).and_then(py_float);
    let max_z_velocity = match get_opt("max_z_velocity") {
        Some(v) => Some(v),
        None => get_opt("max_velocity"),
    };
    let max_z_accel = match get_opt("max_z_accel") {
        Some(v) => Some(v),
        None => get_opt("max_accel"),
    };
    Ok((max_z_velocity, max_z_accel))
}

// ---------------------------------------------------------------------------
// `ResonanceTestExecutor` (`resonance_tester.py:153-261`).
// ---------------------------------------------------------------------------

/// `ResonanceTestExecutor` (`resonance_tester.py:153`).
pub struct ResonanceTestExecutor {
    printer: Printer,
    gcode: Arc<dyn GCodeApi>,
}

impl ResonanceTestExecutor {
    /// `__init__(config)`.
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        Ok(ResonanceTestExecutor { printer, gcode })
    }

    /// `run_test(test_seq, axis, gcmd)`.
    pub fn run_test(
        &self,
        test_seq: &[TestSeqItem],
        axis: &TestAxis,
        gcmd: &dyn GCodeCommandApi,
    ) -> KResult<()> {
        let reactor = self.printer.get_reactor();
        let configfile = klippy_api::lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let tpos = toolhead.get_position();
        let (mut x, mut y, mut z) = (tpos[0], tpos[1], tpos[2]);
        // Override maximum acceleration and acceleration to
        // deceleration based on the maximum test frequency
        let systime = reactor.monotonic();
        let toolhead_info = toolhead.get_status(systime);
        let info_dict = match &toolhead_info {
            PyValue::Dict(d) => d,
            other => return Err(attribute_error(pycompat::value::type_name(other), "get")),
        };
        let status_float = |key: &str| -> KResult<f64> {
            match dict_get(info_dict, key).and_then(py_float) {
                Some(v) => Ok(v),
                None => Err(key_error(key)),
            }
        };
        let old_max_velocity = status_float("max_velocity")?;
        let old_max_accel = status_float("max_accel")?;
        let old_minimum_cruise_ratio = status_float("minimum_cruise_ratio")?;
        // `max([abs(a) for _, a, _ in test_seq])` over an empty sequence.
        if test_seq.is_empty() {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "max() arg is an empty sequence".to_string(),
            });
        }
        let max_accel = test_seq
            .iter()
            .map(|&(_, a, _)| a.abs())
            .fold(f64::NEG_INFINITY, f64::max);
        let mut max_velocity = 0f64;
        let mut last_v = 0f64;
        let mut last_t = 0f64;
        for &(next_t, accel, _freq) in test_seq {
            let v = last_v + accel * (next_t - last_t);
            max_velocity = max_velocity.max(v.abs());
            last_t = next_t;
            last_v = v;
        }
        if axis.get_dir()[2] != 0. {
            let (max_z_velocity, max_z_accel) = lookup_z_limits(&*configfile)?;
            let mut error_msg = String::new();
            if py_gt(max_velocity, max_z_velocity)? {
                error_msg = fmt1(
                    "Insufficient maximum Z velocity for these test parameters, increase at least to %.f mm/s for the resonance test.",
                    pf(max_velocity + 0.5),
                );
            }
            if py_gt(max_accel, max_z_accel)? {
                if !error_msg.is_empty() {
                    error_msg += "\n";
                }
                error_msg += &fmt1(
                    "Insufficient maximum Z acceleration for these test parameters, increase at least to %.f mm/s^2 for the resonance test.",
                    pf(max_accel + 0.5),
                );
            }
            if !error_msg.is_empty() {
                return Err(KlippyError::Command(error_msg));
            }
        }
        self.gcode.run_script_from_command(&fmt_pct(
            "SET_VELOCITY_LIMIT VELOCITY=%.f ACCEL=%.f MINIMUM_CRUISE_RATIO=0",
            &[pf(max_velocity + 0.5), pf(max_accel + 0.5)],
        ))?;
        let input_shaper_obj =
            self.printer.lookup_object("input_shaper", LookupDefault::ReturnNone)?;
        let mut disabled_shaper: Option<Arc<dyn InputShaperApi>> = None;
        if let Some(obj) = &input_shaper_obj {
            if gcmd
                .get_int(
                    "INPUT_SHAPING",
                    GGetIntOpts { default: Some(Some(0)), ..Default::default() },
                )?
                .unwrap_or(0)
                == 0
            {
                let shaper =
                    klippy_api::require_interface::<dyn InputShaperApi>(obj, "input_shaper")?;
                shaper.disable_shaping()?;
                gcmd.respond_info("Disabled [input_shaper] for resonance testing", true);
                disabled_shaper = Some(shaper);
            }
        }
        let mut last_v = 0f64;
        let mut last_t = 0f64;
        let mut last_freq = 0f64;
        for &(next_t, accel, freq) in test_seq {
            let t_seg = next_t - last_t;
            let abs_last_v = last_v.abs();
            let last_v2 = last_v * last_v;
            let (v, abs_v, half_inv_accel, d) = if accel.abs() < 0.000001 {
                let v = last_v;
                let abs_v = abs_last_v;
                if abs_v < 0.000001 {
                    toolhead.dwell(t_seg)?;
                    last_t = next_t;
                    last_freq = freq;
                    continue;
                }
                (v, abs_v, 0., v * t_seg)
            } else {
                toolhead.set_max_velocities(None, Some(accel.abs()), None, None);
                let v = last_v + accel * t_seg;
                let mut abs_v = v.abs();
                let v = if abs_v < 0.000001 {
                    abs_v = 0.;
                    0.
                } else {
                    v
                };
                let half_inv_accel = 0.5 / accel;
                (v, abs_v, half_inv_accel, (v * v - last_v2) * half_inv_accel)
            };
            let d_xyz = axis.get_point(d);
            let (nx, ny, nz) = (x + d_xyz[0], y + d_xyz[1], z + d_xyz[2]);
            toolhead.limit_next_junction_speed(abs_last_v);
            if v * last_v < 0. {
                // The move first goes to a complete stop, then changes direction
                let d_decel = -last_v2 * half_inv_accel;
                let decel_xyz = axis.get_point(d_decel);
                let mut mv = vec![x + decel_xyz[0], y + decel_xyz[1], z + decel_xyz[2]];
                mv.extend_from_slice(&tpos[3..]);
                toolhead.r#move(mv, abs_last_v)?;
                let mut mv = vec![nx, ny, nz];
                mv.extend_from_slice(&tpos[3..]);
                toolhead.r#move(mv, abs_v)?;
            } else {
                let mut mv = vec![nx, ny, nz];
                mv.extend_from_slice(&tpos[3..]);
                toolhead.r#move(mv, abs_v.max(abs_last_v))?;
            }
            if pycompat::libm::floor(freq) > pycompat::libm::floor(last_freq) {
                gcmd.respond_info(&fmt1("Testing frequency %.0f Hz", pf(freq)), true);
                reactor.pause(reactor.monotonic() + 0.01);
            }
            x = nx;
            y = ny;
            z = nz;
            last_t = next_t;
            last_v = v;
            last_freq = freq;
        }
        if last_v != 0. {
            // Decelerate to a stop, continuing in the direction of motion
            let d_decel = 0.5 * last_v * last_v.abs() / old_max_accel;
            let decel_xyz = axis.get_point(d_decel);
            toolhead.set_max_velocities(None, Some(old_max_accel), None, None);
            let mut mv = vec![x + decel_xyz[0], y + decel_xyz[1], z + decel_xyz[2]];
            mv.extend_from_slice(&tpos[3..]);
            toolhead.r#move(mv, last_v.abs())?;
        }
        // Restore the original acceleration values
        self.gcode.run_script_from_command(&fmt_pct(
            "SET_VELOCITY_LIMIT VELOCITY=%.3f ACCEL=%.3f MINIMUM_CRUISE_RATIO=%.3f",
            &[pf(old_max_velocity), pf(old_max_accel), pf(old_minimum_cruise_ratio)],
        ))?;
        // Restore input shaper if it was disabled for resonance testing
        if let Some(shaper) = disabled_shaper {
            shaper.enable_shaping()?;
            gcmd.respond_info("Re-enabled [input_shaper]", true);
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// `ResonanceTester` (`resonance_tester.py:262-551`).
// ---------------------------------------------------------------------------

/// The `test_point` argument of `_run_test`.
///
/// The reference passes the raw `POINT` value, so an empty `POINT=` string is
/// falsy (the probe points are used) while `test_point is not None` still
/// holds, and the "Probing point" response is printed.
#[derive(Clone, Copy, Debug, Default)]
pub struct TestPointArg<'a> {
    /// The parsed coordinates, `None` when the value was falsy.
    pub point: Option<&'a [f64]>,
    /// `test_point is not None`.
    pub given: bool,
}

fn decode_probe_points(value: &PyValue) -> Vec<Vec<f64>> {
    // `getlists(..., parser=float, count=3)` yields a tuple of 3-float tuples.
    let mut out = Vec::new();
    if let PyValue::List(items) | PyValue::Tuple(items) = value {
        for item in items {
            match item {
                PyValue::List(vals) | PyValue::Tuple(vals) => {
                    out.push(vals.iter().filter_map(py_float).collect());
                }
                _ => out.push(Vec::new()),
            }
        }
    }
    out
}

/// `time.strftime("%Y%m%d_%H%M%S")`, the default `NAME` suffix.
fn name_suffix_default() -> String {
    let datestr = klippy_core::configfile::datestr_from_wall_time(kos::time::wall_time());
    datestr.strip_prefix('-').unwrap_or(&datestr).to_string()
}

/// `gcmd.get(name, default)` with `parser=str`: `default = None` keeps the
/// parameter required (`sentinel`), `Some(PyValue::None)` returns `None`.
fn get_string_opt(
    gcmd: &dyn GCodeCommandApi,
    name: &str,
    default: Option<PyValue>,
) -> KResult<Option<String>> {
    let value = gcmd.get(name, GGetOpts { default, ..Default::default() })?;
    match value {
        PyValue::None => Ok(None),
        PyValue::Str(s) => Ok(Some(s)),
        other => Ok(Some(other.str())),
    }
}

/// `ResonanceTester` (`resonance_tester.py:262`).
pub struct ResonanceTester {
    printer: Printer,
    move_speed: f64,
    generator: GCell<SweepingVibrationsTestGenerator>,
    executor: ResonanceTestExecutor,
    accel_chip_names: Vec<(String, String)>,
    max_smoothing: Option<f64>,
    probe_points: Vec<Vec<f64>>,
    accel_chips: GCell<Vec<(String, ObjectRef)>>,
}

klippy_api::printer_object!(ResonanceTester);

impl ResonanceTester {
    /// `__init__(config)`.
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let move_speed = config
            .getfloat(
                "move_speed",
                GetFloatOpts { default: Some(Some(50.)), above: Some(0.), ..Default::default() },
            )?
            .expect("getfloat with a default returns a value");
        let generator = SweepingVibrationsTestGenerator::new(config)?;
        let executor = ResonanceTestExecutor::new(config)?;
        let accel_chip_x = config.get(
            "accel_chip_x",
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        let required_chip = |name: &str| -> KResult<String> {
            Ok(config
                .get(name, GetStrOpts::default())?
                .expect("a required option raises instead of returning None"))
        };
        let optional_chip_z = || -> KResult<String> {
            Ok(config
                .get(
                    "accel_chip_z",
                    GetStrOpts { default: Some(Some(String::new())), ..Default::default() },
                )?
                .unwrap_or_default())
        };
        let mut accel_chip_names: Vec<(String, String)> = if accel_chip_x.as_deref().unwrap_or("").is_empty() {
            vec![
                ("xy".to_string(), required_chip("accel_chip")?.trim().to_string()),
                ("z".to_string(), optional_chip_z()?.trim().to_string()),
            ]
        } else {
            let accel_chip_x = accel_chip_x.expect("the empty case was handled above");
            vec![
                ("x".to_string(), accel_chip_x.trim().to_string()),
                ("y".to_string(), required_chip("accel_chip_y")?.trim().to_string()),
                ("z".to_string(), optional_chip_z()?.trim().to_string()),
            ]
        };
        // Group chips by their axes, exactly like `itertools.groupby` over a
        // stable sort by chip name.
        accel_chip_names.sort_by(|a, b| a.1.cmp(&b.1));
        let mut grouped: Vec<(String, String)> = Vec::new();
        let mut i = 0;
        while i < accel_chip_names.len() {
            let chip_name = accel_chip_names[i].1.clone();
            let mut axes: Vec<String> = Vec::new();
            while i < accel_chip_names.len() && accel_chip_names[i].1 == chip_name {
                axes.push(accel_chip_names[i].0.clone());
                i += 1;
            }
            axes.sort();
            grouped.push((axes.join(""), chip_name));
        }
        let accel_chip_names = grouped;
        let max_smoothing = config.getfloat(
            "max_smoothing",
            GetFloatOpts { default: Some(None), minval: Some(0.05), ..Default::default() },
        )?;
        let probe_points = config.getlists(
            "probe_points",
            GetListsOpts {
                default: None,
                seps: vec![",".to_string(), "\n".to_string()],
                count: Some(3),
                parser: ListParser::Float,
                ..Default::default()
            },
        )?;
        let probe_points = decode_probe_points(&probe_points);
        Ok(ResonanceTester {
            printer,
            move_speed,
            generator: GCell::new(generator),
            executor,
            accel_chip_names,
            max_smoothing,
            probe_points,
            accel_chips: GCell::new(Vec::new()),
        })
    }

    /// The `gcode.register_command` + `klippy:connect` half of `__init__`.
    pub fn register_commands(self: &Arc<Self>) -> KResult<()> {
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(
            &*self.printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let this = Arc::clone(self);
        gcode.register_command(
            "MEASURE_AXES_NOISE",
            Some(Box::new(move |gcmd: &klippy_api::gcode::GCmd| {
                this.cmd_measure_axes_noise(&**gcmd)
            })),
            RegisterCommandOpts {
                desc: Some(Self::cmd_measure_axes_noise_help().to_string()),
                ..Default::default()
            },
        )?;
        let this = Arc::clone(self);
        gcode.register_command(
            "TEST_RESONANCES",
            Some(Box::new(move |gcmd: &klippy_api::gcode::GCmd| {
                this.cmd_test_resonances(&**gcmd)
            })),
            RegisterCommandOpts {
                desc: Some(Self::cmd_test_resonances_help().to_string()),
                ..Default::default()
            },
        )?;
        let this = Arc::clone(self);
        gcode.register_command(
            "SHAPER_CALIBRATE",
            Some(Box::new(move |gcmd: &klippy_api::gcode::GCmd| {
                this.cmd_shaper_calibrate(&**gcmd)
            })),
            RegisterCommandOpts {
                desc: Some(Self::cmd_shaper_calibrate_help().to_string()),
                ..Default::default()
            },
        )?;
        let this = Arc::clone(self);
        self.printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_| {
                this.connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(())
    }

    /// `cmd_MEASURE_AXES_NOISE_help`.
    pub fn cmd_measure_axes_noise_help() -> &'static str {
        "Measures noise of all enabled accelerometer chips"
    }

    /// `cmd_TEST_RESONANCES_help`.
    pub fn cmd_test_resonances_help() -> &'static str {
        "Runs the resonance test for a specified axis"
    }

    /// `cmd_SHAPER_CALIBRATE_help`.
    pub fn cmd_shaper_calibrate_help() -> &'static str {
        "Similar to TEST_RESONANCES but suggest input shaper config"
    }

    /// `connect()`.
    pub fn connect(&self) -> KResult<()> {
        let mut chips: Vec<(String, ObjectRef)> = Vec::new();
        for (chip_axis, chip_name) in self.accel_chip_names.iter() {
            if chip_name.is_empty() {
                continue;
            }
            let chip = self
                .printer
                .lookup_object(chip_name, LookupDefault::Raise)?
                .expect("Raise never yields None");
            if klippy_api::interface::<dyn AccelChipApi>(&chip).is_none() {
                return Err(KlippyError::Config(fmt1(
                    "'%s' is not an accelerometer",
                    ps(chip_name),
                )));
            }
            chips.push((chip_axis.clone(), chip));
        }
        *self.accel_chips.borrow_mut() = chips;
        Ok(())
    }

    fn make_helper(&self) -> ShaperCalibrate {
        ShaperCalibrate::new(Some(Arc::clone(&self.printer)))
    }

    fn chip_snapshot(&self) -> Vec<(String, ObjectRef)> {
        self.accel_chips.cloned()
    }

    fn start_client(
        chip: &ObjectRef,
    ) -> KResult<(Arc<dyn AccelChipApi>, Arc<AccelQueryHelper>)> {
        let api = klippy_api::require_interface::<dyn AccelChipApi>(chip, "accel_chip")?;
        let client = api.start_internal_client()?;
        Ok((api, client))
    }

    /// `is_valid_name_suffix(name_suffix)`.
    pub fn is_valid_name_suffix(&self, name_suffix: &str) -> bool {
        // `name_suffix.replace('-', '').replace('_', '').isalnum()`
        let stripped: String =
            name_suffix.chars().filter(|&c| c != '-' && c != '_').collect();
        !stripped.is_empty() && stripped.chars().all(|c| c.is_alphanumeric())
    }

    /// `get_filename(base, name_suffix, axis=None, point=None, chip_name=None)`.
    pub fn get_filename(
        &self,
        base: &str,
        name_suffix: &str,
        axis: Option<&TestAxis>,
        point: Option<&[f64]>,
        chip_name: Option<&str>,
    ) -> String {
        let mut name = base.to_string();
        if let Some(a) = axis {
            name += "_";
            name += a.get_name();
        }
        if let Some(cn) = chip_name.filter(|s| !s.is_empty()) {
            name += "_";
            name += &cn.replace(' ', "_");
        }
        if let Some(p) = point {
            name += &fmt_pct("_%.3f_%.3f_%.3f", &[pf(p[0]), pf(p[1]), pf(p[2])]);
        }
        name += "_";
        name += name_suffix;
        // `os.path.join("/tmp", name + ".csv")`
        format!("/tmp/{}.csv", name)
    }

    /// `save_calibration_data(...)`.
    #[allow(clippy::too_many_arguments)]
    pub fn save_calibration_data(
        &self,
        base_name: &str,
        name_suffix: &str,
        shaper_calibrate: &ShaperCalibrate,
        axis: &TestAxis,
        calibration_data: &CalibrationData,
        all_shapers: Option<&[CalibrationResult]>,
        point: Option<&[f64]>,
        max_freq: Option<f64>,
    ) -> KResult<String> {
        let output = self.get_filename(base_name, name_suffix, Some(axis), point, None);
        shaper_calibrate.save_calibration_data(&output, calibration_data, all_shapers, max_freq)?;
        Ok(output)
    }

    /// `_get_max_calibration_freq()`.
    pub fn get_max_calibration_freq(&self) -> f64 {
        1.5 * self.generator.borrow().get_max_freq()
    }

    /// `_parse_chips(accel_chips)`.
    pub fn parse_chips(&self, accel_chips: &str) -> KResult<Vec<ObjectRef>> {
        let mut parsed_chips: Vec<ObjectRef> = Vec::new();
        for chip_name in accel_chips.split(',') {
            let chip = self.printer.lookup_object(chip_name.trim(), LookupDefault::ReturnNone)?;
            let chip = match chip {
                None => {
                    return Err(KlippyError::Command(fmt1(
                        "Name '%s' is not valid for CHIPS parameter",
                        ps(chip_name),
                    )))
                }
                Some(c) => c,
            };
            if klippy_api::interface::<dyn AccelChipApi>(&chip).is_none() {
                return Err(KlippyError::Command(fmt1(
                    "'%s' is not an accelerometer",
                    ps(chip_name),
                )));
            }
            parsed_chips.push(chip);
        }
        Ok(parsed_chips)
    }

    /// `_run_test(gcmd, axes, helper, name_suffix, raw_name_suffix=None,
    /// accel_chips=None, test_point=None)`.
    ///
    /// The reference keys `calibration_data` by `TestAxis` identity; the port
    /// returns one slot per entry of `axes` in the same order.
    #[allow(clippy::too_many_arguments)]
    pub fn run_test(
        &self,
        gcmd: &dyn GCodeCommandApi,
        axes: &[TestAxis],
        helper: Option<&ShaperCalibrate>,
        name_suffix: &str,
        raw_name_suffix: Option<&str>,
        accel_chips: Option<&[ObjectRef]>,
        test_point: TestPointArg<'_>,
    ) -> KResult<Vec<Option<CalibrationData>>> {
        let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let mut calibration_data: Vec<Option<CalibrationData>> =
            axes.iter().map(|_| None).collect();
        let has_z: Vec<bool> = axes.iter().map(|a| a.get_dir()[2] != 0.).collect();
        if has_z.iter().all(|&b| b) != has_z.iter().any(|&b| b) {
            return Err(KlippyError::Command(
                "Cannot test Z axis together with other axes".to_string(),
            ));
        }
        self.generator.borrow_mut().prepare_test(gcmd, has_z.iter().all(|&b| b))?;
        let fallback_points = self.probe_points.clone();
        let test_points: Vec<&[f64]> = match test_point.point {
            Some(p) => vec![p],
            None => fallback_points.iter().map(|v| v.as_slice()).collect(),
        };
        let chips_snapshot = self.chip_snapshot();
        for point in test_points.iter() {
            toolhead.manual_move(point.iter().map(|&v| Some(v)).collect(), self.move_speed)?;
            if test_points.len() > 1 || test_point.given {
                gcmd.respond_info(
                    &fmt_pct(
                        "Probing point (%.3f, %.3f, %.3f)",
                        &[pf(point[0]), pf(point[1]), pf(point[2])],
                    ),
                    true,
                );
            }
            let point_arg = if test_points.len() > 1 { Some(*point) } else { None };
            for (axis_idx, axis) in axes.iter().enumerate() {
                toolhead.wait_moves()?;
                toolhead.dwell(0.500)?;
                if axes.len() > 1 {
                    gcmd.respond_info(&fmt1("Testing axis %s", ps(axis.get_name())), true);
                }

                // `(chip_axis, aclient, chip.name)`
                let mut raw_values: Vec<(String, Arc<AccelQueryHelper>, String)> = Vec::new();
                match accel_chips {
                    None => {
                        for (chip_axis, chip) in chips_snapshot.iter() {
                            if axis.matches(chip_axis) {
                                let (api, client) = Self::start_client(chip)?;
                                raw_values.push((chip_axis.clone(), client, api.name()));
                            }
                        }
                    }
                    Some(chips) => {
                        for chip in chips {
                            let (api, client) = Self::start_client(chip)?;
                            raw_values.push((axis.get_name().to_string(), client, api.name()));
                        }
                    }
                }
                if raw_values.is_empty() {
                    return Err(KlippyError::Command(fmt1(
                        "No accelerometers specified that can measure resonances over axis '%s'",
                        ps(axis.get_name()),
                    )));
                }
                let with_chip_name = accel_chips.is_some() || raw_values.len() > 1;

                // Generate moves
                let test_seq = self.generator.borrow().gen_test();
                self.executor.run_test(&test_seq, axis, gcmd)?;
                for (_chip_axis, aclient, chip_name) in raw_values.iter() {
                    aclient.finish_measurements()?;
                    if let Some(raw_suffix) = raw_name_suffix {
                        let raw_name = self.get_filename(
                            "raw_data",
                            raw_suffix,
                            Some(axis),
                            point_arg,
                            if with_chip_name { Some(chip_name.as_str()) } else { None },
                        );
                        aclient.write_to_file(&raw_name);
                        gcmd.respond_info(
                            &fmt1(
                                "Writing raw accelerometer data to %s file",
                                ps(&raw_name),
                            ),
                            true,
                        );
                    }
                }
                let Some(helper) = helper else { continue };
                for (_chip_axis, aclient, chip_name) in raw_values.iter() {
                    if !aclient.has_valid_samples()? {
                        return Err(KlippyError::Command(fmt1(
                            "accelerometer '%s' measured no data",
                            ps(chip_name),
                        )));
                    }
                    let name = self.get_filename(
                        "resonances",
                        name_suffix,
                        Some(axis),
                        point_arg,
                        if with_chip_name { Some(chip_name.as_str()) } else { None },
                    );
                    let samples: Vec<[f64; 4]> = aclient
                        .get_samples()?
                        .into_iter()
                        .map(|(t, x, y, z)| [t, x, y, z])
                        .collect();
                    let new_data =
                        helper.process_accelerometer_data(&name, Some(&samples))?;
                    match &mut calibration_data[axis_idx] {
                        slot @ None => *slot = Some(new_data),
                        Some(existing) => existing.add_data(new_data),
                    }
                }
            }
        }
        Ok(calibration_data)
    }

    /// `cmd_TEST_RESONANCES(gcmd)`.
    pub fn cmd_test_resonances(&self, gcmd: &dyn GCodeCommandApi) -> KResult<()> {
        // Parse parameters
        let raw_axis = get_string_opt(gcmd, "AXIS", None)?
            .expect("AXIS is a required parameter and get raises without it");
        let axis = parse_axis(gcmd, Some(&raw_axis.to_lowercase()))?
            .expect("a present AXIS always parses to a TestAxis");
        let chips_str = get_string_opt(gcmd, "CHIPS", Some(PyValue::None))?;
        let test_point_str = get_string_opt(gcmd, "POINT", Some(PyValue::None))?;

        let mut test_point: Option<Vec<f64>> = None;
        if let Some(raw) = &test_point_str {
            if !raw.is_empty() {
                let test_coords: Vec<&str> = raw.split(',').collect();
                if test_coords.len() != 3 {
                    return Err(KlippyError::Command(
                        "Invalid POINT parameter, must be 'x,y,z'".to_string(),
                    ));
                }
                let mut parsed = Vec::new();
                for p in test_coords {
                    match parse_float_field(p) {
                        Ok(v) => parsed.push(v),
                        Err(_) => {
                            return Err(KlippyError::Command(
                                "Invalid POINT parameter, must be 'x,y,z' where x, y and z are valid floating point numbers"
                                    .to_string(),
                            ))
                        }
                    }
                }
                test_point = Some(parsed);
            }
        }
        let point_arg = TestPointArg {
            point: test_point.as_deref(),
            given: test_point_str.is_some(),
        };

        let accel_chips = match &chips_str {
            Some(s) if !s.is_empty() => Some(self.parse_chips(s)?),
            _ => None,
        };

        let outputs: Vec<String> = get_string_opt(
            gcmd,
            "OUTPUT",
            Some(PyValue::Str("resonances".to_string())),
        )?
        .unwrap_or_else(|| "resonances".to_string())
        .to_lowercase()
        .split(',')
        .map(|s| s.to_string())
        .collect();
        for output in outputs.iter() {
            if output != "resonances" && output != "raw_data" {
                return Err(KlippyError::Command(fmt1(
                    "Unsupported output '%s', only 'resonances' and 'raw_data' are supported",
                    ps(output),
                )));
            }
        }
        if outputs.is_empty() {
            // `str.split` never yields an empty list; kept for the reference shape.
            return Err(KlippyError::Command(
                "No output specified, at least one of 'resonances' or 'raw_data' must be set in OUTPUT parameter"
                    .to_string(),
            ));
        }
        let name_suffix =
            get_string_opt(gcmd, "NAME", Some(PyValue::None))?.unwrap_or_else(name_suffix_default);
        if !self.is_valid_name_suffix(&name_suffix) {
            return Err(KlippyError::Command("Invalid NAME parameter".to_string()));
        }
        let csv_output = outputs.iter().any(|o| o == "resonances");
        let raw_output = outputs.iter().any(|o| o == "raw_data");

        // Setup calculation of resonances
        let helper = if csv_output { Some(self.make_helper()) } else { None };

        let mut results = self.run_test(
            gcmd,
            std::slice::from_ref(&axis),
            helper.as_ref(),
            &name_suffix,
            if raw_output { Some(&name_suffix) } else { None },
            accel_chips.as_deref(),
            point_arg,
        )?;
        // `self._run_test(...)[axis]`
        let data = results.remove(0);
        if csv_output {
            let helper = helper.expect("csv output builds the helper");
            let data = match data.as_ref() {
                Some(d) => d,
                None => return Err(attribute_error("NoneType", "get_datasets")),
            };
            let csv_name = self.save_calibration_data(
                "resonances",
                &name_suffix,
                &helper,
                &axis,
                data,
                None,
                test_point.as_deref(),
                Some(self.get_max_calibration_freq()),
            )?;
            gcmd.respond_info(&fmt1("Resonances data written to %s file", ps(&csv_name)), true);
        }
        Ok(())
    }

    /// `cmd_SHAPER_CALIBRATE(gcmd)`.
    pub fn cmd_shaper_calibrate(&self, gcmd: &dyn GCodeCommandApi) -> KResult<()> {
        // Parse parameters
        let axis = get_string_opt(gcmd, "AXIS", Some(PyValue::None))?;
        let calibrate_axes: Vec<TestAxis> = match axis {
            None => vec![TestAxis::new_axis("x")?, TestAxis::new_axis("y")?],
            Some(raw) if raw.is_empty() => vec![TestAxis::new_axis("x")?, TestAxis::new_axis("y")?],
            Some(raw) => {
                let lowered = raw.to_lowercase();
                // Python: `axis.lower() not in 'xyz'` (a substring test).
                if !"xyz".contains(lowered.as_str()) {
                    return Err(KlippyError::Command(fmt1("Unsupported axis '%s'", ps(&raw))));
                }
                vec![TestAxis::new_axis(&lowered)?]
            }
        };
        let chips_str = get_string_opt(gcmd, "CHIPS", Some(PyValue::None))?;
        let accel_chips = match &chips_str {
            Some(s) if !s.is_empty() => Some(self.parse_chips(s)?),
            _ => None,
        };

        let max_smoothing = gcmd.get_float(
            "MAX_SMOOTHING",
            GGetFloatOpts {
                default: Some(self.max_smoothing),
                minval: Some(0.05),
                ..Default::default()
            },
        )?;

        let name_suffix =
            get_string_opt(gcmd, "NAME", Some(PyValue::None))?.unwrap_or_else(name_suffix_default);
        if !self.is_valid_name_suffix(&name_suffix) {
            return Err(KlippyError::Command("Invalid NAME parameter".to_string()));
        }

        let input_shaper = self.printer.lookup_object("input_shaper", LookupDefault::ReturnNone)?;

        // Setup shaper calibration
        let helper = self.make_helper();

        let mut calibration_data = self.run_test(
            gcmd,
            &calibrate_axes,
            Some(&helper),
            &name_suffix,
            None,
            accel_chips.as_deref(),
            TestPointArg::default(),
        )?;

        let configfile = klippy_api::lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        for (idx, axis) in calibrate_axes.iter().enumerate() {
            let axis_name = axis.get_name();
            gcmd.respond_info(
                &fmt1(
                    "Calculating the best input shaper parameters for %s axis",
                    ps(axis_name),
                ),
                true,
            );
            let data = match calibration_data[idx].as_mut() {
                Some(d) => d,
                None => return Err(attribute_error("NoneType", "normalize_to_frequencies")),
            };
            data.normalize_to_frequencies();
            let systime = self.printer.get_reactor().monotonic();
            let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
                &*self.printer,
                "toolhead",
                LookupDefault::Raise,
            )?
            .expect("Raise never yields None");
            let toolhead_info = toolhead.get_status(systime);
            let info_dict = match &toolhead_info {
                PyValue::Dict(d) => d,
                other => return Err(attribute_error(pycompat::value::type_name(other), "get")),
            };
            let scv = match dict_get(info_dict, "square_corner_velocity").and_then(py_float) {
                Some(v) => v,
                None => return Err(key_error("square_corner_velocity")),
            };
            let max_freq = self.get_max_calibration_freq();
            let (best_shaper, all_shapers) = helper.find_best_shaper(
                data,
                None,
                None,
                Some(scv),
                None,
                max_smoothing,
                None,
                None,
                Some(max_freq),
                Some(&|msg: &str| gcmd.respond_info(msg, true)),
            )?;
            gcmd.respond_info(
                &fmt_pct(
                    "Recommended shaper_type_%s = %s, shaper_freq_%s = %.1f Hz",
                    &[
                        ps(axis_name),
                        ps(&best_shaper.name),
                        ps(axis_name),
                        pf(best_shaper.freq),
                    ],
                ),
                true,
            );
            if let Some(obj) = &input_shaper {
                let shaper =
                    klippy_api::require_interface::<dyn InputShaperApi>(obj, "input_shaper")?;
                helper.apply_params(
                    &*shaper,
                    axis_name,
                    &best_shaper.name,
                    best_shaper.freq,
                )?;
            }
            helper.save_params(&*configfile, axis_name, &best_shaper.name, best_shaper.freq)?;
            let csv_name = self.save_calibration_data(
                "calibration_data",
                &name_suffix,
                &helper,
                axis,
                data,
                Some(&all_shapers),
                None,
                Some(max_freq),
            )?;
            gcmd.respond_info(
                &fmt1("Shaper calibration data written to %s file", ps(&csv_name)),
                true,
            );
        }
        gcmd.respond_info(
            "The SAVE_CONFIG command will update the printer config file\nwith these parameters and restart the printer.",
            true,
        );
        Ok(())
    }

    /// `cmd_MEASURE_AXES_NOISE(gcmd)`.
    pub fn cmd_measure_axes_noise(&self, gcmd: &dyn GCodeCommandApi) -> KResult<()> {
        let meas_time = gcmd
            .get_float(
                "MEAS_TIME",
                GGetFloatOpts { default: Some(Some(2.)), above: Some(0.), ..Default::default() },
            )?
            .expect("get_float with a default returns a value");
        let mut raw_values: Vec<(String, Arc<AccelQueryHelper>)> = Vec::new();
        for (chip_axis, chip) in self.chip_snapshot().iter() {
            let (_api, client) = Self::start_client(chip)?;
            raw_values.push((chip_axis.clone(), client));
        }
        klippy_api::lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None")
        .dwell(meas_time)?;
        for (_chip_axis, aclient) in raw_values.iter() {
            aclient.finish_measurements()?;
        }
        let helper = self.make_helper();
        for (chip_axis, aclient) in raw_values.iter() {
            if !aclient.has_valid_samples()? {
                return Err(KlippyError::Command(fmt1(
                    "%s-axis accelerometer measured no data",
                    ps(chip_axis),
                )));
            }
            let samples: Vec<[f64; 4]> = aclient
                .get_samples()?
                .into_iter()
                .map(|(t, x, y, z)| [t, x, y, z])
                .collect();
            let data = helper.process_accelerometer_data("", Some(&samples))?;
            // `ndarray.mean()` of each PSD.
            gcmd.respond_info(
                &fmt_pct(
                    "Axes noise for %s-axis accelerometer: %.6f (x), %.6f (y), %.6f (z)",
                    &[
                        ps(chip_axis),
                        pf(numpy::mean(&data.psd_x)),
                        pf(numpy::mean(&data.psd_y)),
                        pf(numpy::mean(&data.psd_z)),
                    ],
                ),
                true,
            );
        }
        Ok(())
    }

    /// `self.accel_chips`: the `(axes, chip object)` pairs resolved by
    /// [`ResonanceTester::connect`].
    pub fn accel_chips(&self) -> Vec<(String, ObjectRef)> {
        self.accel_chips.cloned()
    }

    /// `self.accel_chip_names`: `(axes, chip_name)` pairs, grouped by chip.
    pub fn accel_chip_names(&self) -> &[(String, String)] {
        &self.accel_chip_names
    }

    /// `self.move_speed`.
    pub fn move_speed(&self) -> f64 {
        self.move_speed
    }

    /// `self.max_smoothing`.
    pub fn max_smoothing(&self) -> Option<f64> {
        self.max_smoothing
    }

    /// `self.probe_points`.
    pub fn probe_points(&self) -> &[Vec<f64>] {
        &self.probe_points
    }

    /// `self.generator.get_max_freq()` through the mutable generator.
    pub fn generator(&self) -> std::cell::Ref<'_, SweepingVibrationsTestGenerator> {
        self.generator.borrow()
    }
}

/// `load_config(config)` (`resonance_tester.py:552`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = Arc::new(ResonanceTester::new(config)?);
    obj.register_commands()?;
    Ok(Some(obj as ObjectRef))
}

/// Module entry point for the `resonance_tester` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory { load_config: Some(load_config), ..Default::default() }
}
