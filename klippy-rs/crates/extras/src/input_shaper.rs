//! Port of the `input_shaper` module (`klippy/extras/input_shaper.py`).
//!
//! Configures per-axis kinematic input shapers and exposes the
//! `SET_INPUT_SHAPER` command. The shaper pulse amplitudes and times
//! computed from the configured type/frequency/damping are handed to
//! the stepper kinematics through the `input_shaper_*` chelper entry
//! points (`klippy/chelper/kin_shaper.c`): the wrapper is allocated with
//! `klippy_core::motion_handles::alloc_stepper_kinematics` and the
//! kind-specific calls go through `StepperKinematicsHandle::with` on
//! [`klippy_core::motion_handles::KinAlloc::InputShaper`].
//!
//! The shaper math itself is `crate::shaper_defs`
//! (`klippy/extras/shaper_defs.py`), which also owns the `mathutil`
//! matrix helpers it depends on.

use crate::shaper_defs;
use chelper_motion::kin_shaper;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts as GGetFloatOpts, GetOpts, GetParser,
    RegisterCommandOpts};
use klippy_api::printer::{EV_DUAL_CARRIAGE_UPDATE_KINEMATICS, EV_KLIPPY_CONNECT, LookupDefault,
    ModuleFactory, Printer};
use klippy_api::stepper::{Stepper, StepperKinematics};
use klippy_api::toolhead::{MotionQueuingApi, ToolHead, ToolHeadApi};
use klippy_api::{GCell, lookup_interface, printer_object};
use klippy_core::motion_handles::{KinAlloc, alloc_stepper_kinematics, kinematics_of};
use pycompat::PyValue;
use std::sync::Arc;

/// `cmd_SET_INPUT_SHAPER_help` (`klippy/extras/input_shaper.py:199`).
pub const CMD_SET_INPUT_SHAPER_HELP: &str = "Set cartesian parameters for input shaper";


fn dual_carriage_error() -> KlippyError {
    KlippyError::config(
        "Input shaper parameters cannot be configured via [input_shaper] section with \
         dual_carriage(s) enabled. Refer to Klipper documentation on how to configure \
         input shaper for dual_carriage(s).",
    )
}

/// Which exception class a `get_shaper` validation failure is raised as
/// (`config.error` or `gcmd.error` in the reference).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrKind {
    Config,
    Command,
}



// ---------------------------------------------------------------------------
// Input-shaper stepper kinematics: the `input_shaper_*` chelper entry
// points (`klippy/chelper/kin_shaper.c`), reached through the contract
// handles (`klippy_core::motion_handles`).
// ---------------------------------------------------------------------------

fn wrong_kind() -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: "stepper kinematics is not an input_shaper_stepper".to_string(),
    }
}

/// `input_shaper_alloc()` (`kin_shaper.c:287`). The caller keeps the
/// wrapper (and the original it wraps) alive, exactly like the Python
/// `input_shaper_stepper_kinematics` / `orig_stepper_kinematics` lists.
pub fn input_shaper_alloc() -> KResult<StepperKinematics> {
    alloc_stepper_kinematics("input_shaper_alloc", &[])
}

/// `input_shaper_set_sk()` (`kin_shaper.c:243`): attach the original
/// stepper kinematics to the wrapper. Returns `0` on success and `-1`
/// when the original has no active axis (the wrapper is left untouched).
pub fn input_shaper_set_sk(
    is_sk: &StepperKinematics,
    orig_sk: Option<&StepperKinematics>,
) -> KResult<i32> {
    let Some(orig) = orig_sk else {
        // `stepper.get_stepper_kinematics()` without an installed
        // kinematics: the C entry would dereference the null pointer;
        // treat as a failed wrap so the caller restores and skips.
        return Ok(-1);
    };
    let orig_ptr = kinematics_of(orig)?.sk_ptr();
    kinematics_of(is_sk)?.with(|alloc| match alloc {
        KinAlloc::InputShaper(is) => {
            // SAFETY: the caller keeps the original allocation alive for
            // as long as the wrapper exists (`orig_stepper_kinematics`).
            Ok(unsafe { kin_shaper::input_shaper_set_sk(is, orig_ptr) })
        }
        _ => Err(wrong_kind()),
    })
}

/// `input_shaper_set_shaper_params()` (`kin_shaper.c:269`): configure one
/// axis' pulse train. Returns `0` on success, `-1` for an unknown axis or
/// invalid pulses; an inactive axis is ignored with status `0`.
pub fn input_shaper_set_shaper_params(
    is_sk: &StepperKinematics,
    axis: u8,
    n: i32,
    a: &[f64],
    t: &[f64],
) -> KResult<i32> {
    kinematics_of(is_sk)?.with(|alloc| match alloc {
        KinAlloc::InputShaper(is) => {
            Ok(kin_shaper::input_shaper_set_shaper_params(is, axis, n, a, t))
        }
        _ => Err(wrong_kind()),
    })
}

/// `input_shaper_update_sk()` (`kin_shaper.c:226`): re-read the wrapped
/// kinematics' active flags and refresh the step-generation windows.
pub fn input_shaper_update_sk(is_sk: &StepperKinematics) -> KResult<()> {
    kinematics_of(is_sk)?.with(|alloc| match alloc {
        KinAlloc::InputShaper(is) => {
            kin_shaper::input_shaper_update_sk(is);
            Ok(())
        }
        _ => Err(wrong_kind()),
    })
}

/// Snapshot of the three pulse sets for tests.
pub fn input_shaper_pulses(
    is_sk: &StepperKinematics,
) -> Option<[chelper_motion::kin_shaper::ShaperPulses; 3]> {
    let handle = kinematics_of(is_sk).ok()?;
    handle.with(|alloc| match alloc {
        KinAlloc::InputShaper(is) => Some([
            *is.shaper_pulses(0),
            *is.shaper_pulses(1),
            *is.shaper_pulses(2),
        ]),
        _ => None,
    })
}

// ---------------------------------------------------------------------------
// `InputShaperParams` (`input_shaper.py:11`).
// ---------------------------------------------------------------------------

/// Per-axis resolved parameters.
#[derive(Debug)]
pub struct InputShaperParams {
    axis: String,
    shaper_type: String,
    damping_ratio: f64,
    shaper_freq: f64,
}

impl InputShaperParams {
    /// `InputShaperParams.__init__(axis, config)` (`input_shaper.py:12`).
    pub fn new(axis: &str, config: &Config) -> KResult<Self> {
        let shaper_type = config
            .get(
                "shaper_type",
                GetStrOpts { default: Some(Some("mzv".to_string())), ..Default::default() },
            )?
            .unwrap_or_else(|| "mzv".to_string());
        let shaper_type = config
            .get(
                &format!("shaper_type_{axis}"),
                GetStrOpts {
                    default: Some(Some(shaper_type.clone())),
                    ..Default::default()
                },
            )?
            .unwrap_or_else(|| shaper_type.clone());
        let sconfig = match shaper_defs::get_shaper_cfg(&shaper_type) {
            Some(cfg) => cfg,
            None => {
                return Err(KlippyError::config(format!("Unsupported shaper type: {shaper_type}")));
            }
        };
        let damping_ratio = config
            .getfloat(
                &format!("damping_ratio_{axis}"),
                GetFloatOpts {
                    default: Some(Some(shaper_defs::DEFAULT_DAMPING_RATIO)),
                    minval: Some(0.),
                    maxval: Some(sconfig.max_damping_ratio),
                    ..Default::default()
                },
            )?
            .unwrap_or(shaper_defs::DEFAULT_DAMPING_RATIO);
        let shaper_freq = config
            .getfloat(
                &format!("shaper_freq_{axis}"),
                GetFloatOpts {
                    default: Some(Some(0.)),
                    minval: Some(0.),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.);
        let params = InputShaperParams {
            axis: axis.to_string(),
            shaper_type,
            damping_ratio,
            shaper_freq,
        };
        params.get_shaper(None, None, None, Some(ErrKind::Config))?;
        Ok(params)
    }

    /// `InputShaperParams.update(gcmd)` (`input_shaper.py:27`).
    pub fn update(&mut self, gcmd: &GCmd) -> KResult<()> {
        let axis = self.axis.to_uppercase();
        let st_value = gcmd.get(
            "SHAPER_TYPE",
            GetOpts { default: Some(PyValue::None), parser: GetParser::Str, ..Default::default() },
        )?;
        let shaper_type = match st_value {
            PyValue::None => {
                let fallback = gcmd.get(
                    &format!("SHAPER_TYPE_{axis}"),
                    GetOpts {
                        default: Some(PyValue::Str(self.shaper_type.clone())),
                        parser: GetParser::Str,
                        ..Default::default()
                    },
                )?;
                match fallback {
                    PyValue::Str(s) => s,
                    other => other.str(),
                }
            }
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let lower = shaper_type.to_lowercase();
        let sconfig = match shaper_defs::get_shaper_cfg(&lower) {
            Some(cfg) => cfg,
            None => {
                return Err(KlippyError::command(format!("Unsupported shaper type: {shaper_type}")));
            }
        };
        let damping_ratio = gcmd
            .get_float(
                &format!("DAMPING_RATIO_{axis}"),
                GGetFloatOpts {
                    default: Some(Some(self.damping_ratio)),
                    minval: Some(0.),
                    ..Default::default()
                },
            )?
            .unwrap_or(self.damping_ratio);
        if damping_ratio > sconfig.max_damping_ratio {
            return Err(KlippyError::command(format!(
                "Too high value of damping_ratio={:.3} for shaper {shaper_type} on axis {axis}",
                damping_ratio
            )));
        }
        let shaper_freq = gcmd
            .get_float(
                &format!("SHAPER_FREQ_{axis}"),
                GGetFloatOpts {
                    default: Some(Some(self.shaper_freq)),
                    minval: Some(0.),
                    ..Default::default()
                },
            )?
            .unwrap_or(self.shaper_freq);
        self.get_shaper(Some(&lower), Some(shaper_freq), Some(damping_ratio), Some(ErrKind::Command))?;
        self.damping_ratio = damping_ratio;
        self.shaper_type = lower;
        self.shaper_freq = shaper_freq;
        Ok(())
    }

    /// `InputShaperParams.get_shaper(...)` (`input_shaper.py:49`).
    ///
    /// The `or` fallbacks are reproduced exactly: a zero frequency or
    /// damping (or an empty type string) falls back to the configured
    /// value, and a configured zero frequency always yields the empty
    /// shaper regardless of the arguments.
    pub fn get_shaper(
        &self,
        shaper_type: Option<&str>,
        shaper_freq: Option<f64>,
        damping_ratio: Option<f64>,
        err: Option<ErrKind>,
    ) -> KResult<(i64, Vec<f64>, Vec<f64>)> {
        if self.shaper_freq == 0. {
            let (a, t) = shaper_defs::get_none_shaper();
            return Ok((0, a, t));
        }
        let st = match shaper_type {
            Some(s) if !s.is_empty() => s.to_string(),
            _ => self.shaper_type.clone(),
        };
        let freq = match shaper_freq {
            Some(v) if v != 0. => v,
            _ => self.shaper_freq,
        };
        let damping = match damping_ratio {
            Some(v) if v != 0. => v,
            _ => self.damping_ratio,
        };
        let config_factory = |m: String| KlippyError::config(m);
        let command_factory = |m: String| KlippyError::command(m);
        let err_factory: Option<&dyn Fn(String) -> KlippyError> = match err {
            Some(ErrKind::Config) => Some(&config_factory),
            Some(ErrKind::Command) => Some(&command_factory),
            None => None,
        };
        let (a, t) = match shaper_defs::init_shaper(&st, freq, damping, err_factory)? {
            Some(pair) => pair,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "cannot unpack non-iterable NoneType object".to_string(),
                });
            }
        };
        Ok((a.len() as i64, a, t))
    }

    /// `InputShaperParams.get_status()` (`input_shaper.py:59`).
    pub fn get_status(&self) -> indexmap::IndexMap<String, String> {
        let mut out: indexmap::IndexMap<String, String> = indexmap::IndexMap::new();
        out.insert("shaper_type".to_string(), self.shaper_type.clone());
        out.insert(
            "shaper_freq".to_string(),
            pycompat::text::percent_format("%.3f", &[pycompat::text::PctValue::Float(self.shaper_freq)])
                .unwrap_or_default(),
        );
        out.insert(
            "damping_ratio".to_string(),
            pycompat::text::percent_format(
                "%.6f",
                &[pycompat::text::PctValue::Float(self.damping_ratio)],
            )
            .unwrap_or_default(),
        );
        out
    }

    pub fn axis(&self) -> &str {
        &self.axis
    }

    pub fn shaper_type(&self) -> &str {
        &self.shaper_type
    }

    pub fn damping_ratio(&self) -> f64 {
        self.damping_ratio
    }

    pub fn shaper_freq(&self) -> f64 {
        self.shaper_freq
    }
}

// ---------------------------------------------------------------------------
// `AxisInputShaper` (`input_shaper.py:65`).
// ---------------------------------------------------------------------------

/// One axis' shaper state.
pub struct AxisInputShaper {
    axis: String,
    params: GCell<InputShaperParams>,
    /// `(n, A, T)`.
    shaper: GCell<(i64, Vec<f64>, Vec<f64>)>,
    /// `(n, A, T)` saved while shaping is disabled.
    saved: GCell<Option<(i64, Vec<f64>, Vec<f64>)>>,
}

impl AxisInputShaper {
    /// `AxisInputShaper.__init__(axis, config)` (`input_shaper.py:66`).
    pub fn new(axis: &str, config: &Config) -> KResult<Self> {
        let params = InputShaperParams::new(axis, config)?;
        let shaper = params.get_shaper(None, None, None, None)?;
        Ok(AxisInputShaper {
            axis: axis.to_string(),
            params: GCell::new(params),
            shaper: GCell::new(shaper),
            saved: GCell::new(None),
        })
    }

    /// `get_name` (`input_shaper.py:71`).
    pub fn get_name(&self) -> String {
        format!("shaper_{}", self.axis)
    }

    /// `get_shaper` (`input_shaper.py:73`).
    pub fn get_shaper(&self) -> (i64, Vec<f64>, Vec<f64>) {
        self.shaper.borrow().clone()
    }

    /// `update(gcmd)` (`input_shaper.py:75`).
    pub fn update(&self, gcmd: &GCmd) -> KResult<()> {
        self.params.borrow_mut().update(gcmd)?;
        let shaper = self.params.borrow().get_shaper(None, None, None, None)?;
        *self.shaper.borrow_mut() = shaper;
        Ok(())
    }

    /// `set_shaper_kinematics(sk)` (`input_shaper.py:78`).
    pub fn set_shaper_kinematics(&self, sk: &StepperKinematics) -> KResult<bool> {
        let (n, a, t) = self.get_shaper();
        let axis = self.axis.as_bytes().first().copied().unwrap_or(b'x');
        let success = input_shaper_set_shaper_params(sk, axis, n as i32, &a, &t)? == 0;
        if !success {
            self.disable_shaping();
            let (n, a, t) = self.get_shaper();
            input_shaper_set_shaper_params(sk, axis, n as i32, &a, &t)?;
        }
        Ok(success)
    }

    /// `is_enabled` (`input_shaper.py:87`).
    pub fn is_enabled(&self) -> bool {
        self.get_shaper().0 > 0
    }

    /// `disable_shaping` (`input_shaper.py:89`).
    pub fn disable_shaping(&self) {
        let current = self.get_shaper();
        let mut saved = self.saved.borrow_mut();
        if saved.is_none() && current.0 != 0 {
            *saved = Some(current);
        }
        drop(saved);
        let (a, t) = shaper_defs::get_none_shaper();
        *self.shaper.borrow_mut() = (a.len() as i64, a, t);
    }

    /// `enable_shaping` (`input_shaper.py:94`).
    pub fn enable_shaping(&self) {
        if self.saved.borrow().is_none() {
            return;
        }
        let saved = self.saved.replace(None);
        *self.shaper.borrow_mut() = saved.expect("checked above");
    }

    /// `report(gcmd)` (`input_shaper.py:100`).
    pub fn report(&self, gcmd: &GCmd) {
        let status = self.params.borrow().get_status();
        let info = status
            .iter()
            .map(|(key, value)| format!("{key}_{}:{value}", self.axis))
            .collect::<Vec<_>>()
            .join(" ");
        gcmd.respond_info(&info, true);
    }

    pub fn axis(&self) -> &str {
        &self.axis
    }

    pub fn params_snapshot(&self) -> (String, f64, f64) {
        let params = self.params.borrow();
        (
            params.shaper_type().to_string(),
            params.shaper_freq(),
            params.damping_ratio(),
        )
    }

    pub fn get_status(&self) -> indexmap::IndexMap<String, String> {
        self.params.borrow().get_status()
    }
}

// ---------------------------------------------------------------------------
// `InputShaper` (`input_shaper.py:105`).
// ---------------------------------------------------------------------------

/// The printer object behind the `[input_shaper]` section.
pub struct InputShaper {
    printer: Printer,
    toolhead: GCell<Option<ToolHead>>,
    shapers: Vec<Arc<AxisInputShaper>>,
    /// `input_shaper_stepper_kinematics` (kept alive).
    is_sks: GCell<Vec<StepperKinematics>>,
    /// `orig_stepper_kinematics` (kept alive).
    orig_sks: GCell<Vec<Option<StepperKinematics>>>,
}

/// The cross-module surface `resonance_tester` and `shaper_calibrate`
/// duck-type against (`disable_shaping`, `enable_shaping`,
/// `cmd_SET_INPUT_SHAPER`).
pub trait InputShaperApi: Send + Sync + 'static {
    /// `disable_shaping()` (`input_shaper.py:191`).
    fn disable_shaping(&self) -> KResult<()>;
    /// `enable_shaping()` (`input_shaper.py:195`).
    fn enable_shaping(&self) -> KResult<()>;
    /// `cmd_SET_INPUT_SHAPER(gcmd)` (`input_shaper.py:200`).
    fn cmd_set_input_shaper(&self, gcmd: &GCmd) -> KResult<()>;
}

impl InputShaper {
    /// `InputShaper.__init__(config)` (`input_shaper.py:106`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        printer.register_event_handler(EV_KLIPPY_CONNECT, {
            let printer = printer.clone();
            Box::new(move |_args| {
                let obj = printer.lookup_object("input_shaper", LookupDefault::Raise)?;
                let obj = obj.expect("Raise never yields None");
                let me = klippy_api::downcast_arc::<InputShaper>(&obj)
                    .expect("input_shaper object type");
                me.connect()?;
                Ok(PyValue::None)
            })
        });
        printer.register_event_handler(EV_DUAL_CARRIAGE_UPDATE_KINEMATICS, {
            let printer = printer.clone();
            Box::new(move |_args| {
                let obj = printer.lookup_object("input_shaper", LookupDefault::Raise)?;
                let obj = obj.expect("Raise never yields None");
                let me = klippy_api::downcast_arc::<InputShaper>(&obj)
                    .expect("input_shaper object type");
                me.update_kinematics()?;
                Ok(PyValue::None)
            })
        });
        let mut shapers = Vec::new();
        for axis in ["x", "y", "z"] {
            shapers.push(Arc::new(AxisInputShaper::new(axis, config)?));
        }
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        gcode.register_command(
            "SET_INPUT_SHAPER",
            Some(Box::new({
                let printer = printer.clone();
                move |gcmd| {
                    let obj = printer.lookup_object("input_shaper", LookupDefault::Raise)?;
                    let obj = obj.expect("Raise never yields None");
                    let me = klippy_api::downcast_arc::<InputShaper>(&obj)
                        .expect("input_shaper object type");
                    me.cmd_set_input_shaper(gcmd)
                }
            })),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_SET_INPUT_SHAPER_HELP.to_string()),
            },
        )?;
        Ok(InputShaper {
            printer,
            toolhead: GCell::new(None),
            shapers,
            is_sks: GCell::new(Vec::new()),
            orig_sks: GCell::new(Vec::new()),
        })
    }

    /// `connect()` (`input_shaper.py:124`).
    pub fn connect(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("Raise never yields None");
        *self.toolhead.borrow_mut() = Some(toolhead);
        let dual = self.printer.lookup_object("dual_carriage", LookupDefault::ReturnNone)?;
        if dual.is_some() {
            if self.shapers.iter().any(|s| s.is_enabled()) {
                return Err(dual_carriage_error());
            }
            return Ok(());
        }
        self.update_input_shaping(true)
    }

    /// `_get_input_shaper_stepper_kinematics(stepper)`
    /// (`input_shaper.py:138`).
    fn get_is_sk(&self, stepper: &Stepper) -> KResult<Option<StepperKinematics>> {
        let sk = stepper.get_stepper_kinematics();
        {
            let is_sks = self.is_sks.borrow();
            if let Some(s) = &sk {
                if is_sks.iter().any(|c| Arc::ptr_eq(c, s)) {
                    return Ok(Some(s.clone()));
                }
            }
        }
        let is_sk = input_shaper_alloc()?;
        stepper.set_stepper_kinematics(Some(is_sk.clone()));
        if input_shaper_set_sk(&is_sk, sk.as_ref())? < 0 {
            stepper.set_stepper_kinematics(sk);
            return Ok(None);
        }
        self.orig_sks.borrow_mut().push(sk);
        self.is_sks.borrow_mut().push(is_sk.clone());
        Ok(Some(is_sk))
    }

    /// `_update_kinematics()` (`input_shaper.py:153`).
    pub fn update_kinematics(&self) -> KResult<()> {
        let toolhead = match self.toolhead.borrow().clone() {
            Some(toolhead) => toolhead,
            None => return Ok(()),
        };
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        for stepper in kin.get_steppers() {
            if stepper.get_trapq().is_none() {
                continue;
            }
            let Some(is_sk) = self.get_is_sk(&stepper)? else {
                continue;
            };
            input_shaper_update_sk(&is_sk)?;
        }
        let motion_queuing =
            lookup_interface::<dyn MotionQueuingApi>(&*self.printer, "motion_queuing", LookupDefault::Raise)?
                .expect("Raise never yields None");
        motion_queuing.check_step_generation_scan_windows();
        Ok(())
    }

    /// `_update_input_shaping(error=...)` (`input_shaper.py:169`).
    ///
    /// `use_config_error` selects the `printer.config_error` class
    /// used on init, otherwise `printer.command_error`.
    pub fn update_input_shaping(&self, use_config_error: bool) -> KResult<()> {
        let toolhead = match self.toolhead.borrow().clone() {
            Some(toolhead) => toolhead,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'flush_step_generation'"
                        .to_string(),
                });
            }
        };
        toolhead.flush_step_generation()?;
        let kin = toolhead.get_kinematics();
        let mut failed: Vec<String> = Vec::new();
        for stepper in kin.get_steppers() {
            if stepper.get_trapq().is_none() {
                continue;
            }
            let Some(is_sk) = self.get_is_sk(&stepper)? else {
                continue;
            };
            for shaper in &self.shapers {
                if failed.contains(&shaper.get_name()) {
                    continue;
                }
                if !shaper.set_shaper_kinematics(&is_sk)? {
                    failed.push(shaper.get_name());
                }
            }
        }
        let motion_queuing =
            lookup_interface::<dyn MotionQueuingApi>(&*self.printer, "motion_queuing", LookupDefault::Raise)?
                .expect("Raise never yields None");
        motion_queuing.check_step_generation_scan_windows();
        if !failed.is_empty() {
            let msg = format!(
                "Failed to configure shaper(s) {} with given parameters",
                failed.join(", ")
            );
            return Err(if use_config_error {
                KlippyError::config(msg)
            } else {
                KlippyError::command(msg)
            });
        }
        Ok(())
    }

    /// `disable_shaping()` (`input_shaper.py:191`).
    pub fn disable_shaping(&self) -> KResult<()> {
        for shaper in &self.shapers {
            shaper.disable_shaping();
        }
        self.update_input_shaping(false)
    }

    /// `enable_shaping()` (`input_shaper.py:195`).
    pub fn enable_shaping(&self) -> KResult<()> {
        for shaper in &self.shapers {
            shaper.enable_shaping();
        }
        self.update_input_shaping(false)
    }

    /// `cmd_SET_INPUT_SHAPER(gcmd)` (`input_shaper.py:200`).
    pub fn cmd_set_input_shaper(&self, gcmd: &GCmd) -> KResult<()> {
        if !gcmd.get_command_parameters().is_empty() {
            for shaper in &self.shapers {
                shaper.update(gcmd)?;
            }
            self.update_input_shaping(false)?;
        }
        for (ind, shaper) in self.shapers.iter().enumerate() {
            if ind < 2 || shaper.is_enabled() {
                shaper.report(gcmd);
            }
        }
        Ok(())
    }

    /// `get_shapers()` (`input_shaper.py:122`).
    pub fn get_shapers(&self) -> Vec<Arc<AxisInputShaper>> {
        self.shapers.clone()
    }
}

impl InputShaperApi for InputShaper {
    fn disable_shaping(&self) -> KResult<()> {
        InputShaper::disable_shaping(self)
    }

    fn enable_shaping(&self) -> KResult<()> {
        InputShaper::enable_shaping(self)
    }

    fn cmd_set_input_shaper(&self, gcmd: &GCmd) -> KResult<()> {
        InputShaper::cmd_set_input_shaper(self, gcmd)
    }
}

printer_object!(InputShaper; dyn InputShaperApi);

/// `load_config(config)` (`input_shaper.py:209`).
fn load_config(config: &Config) -> KResult<Option<klippy_api::ObjectRef>> {
    Ok(Some(Arc::new(InputShaper::new(config)?) as klippy_api::ObjectRef))
}

/// Module entry point for the `input_shaper` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
