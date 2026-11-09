// Port of `klippy/extras/screws_tilt_adjust.py`.
use crate::manual_probe::ProbeResult;
use crate::probe::{ProbeFinalizeCallback, ProbePointsHelper};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetFloatListOpts, GetStrOpts};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GCmd, GetFloatOpts as GcmdFloatOpts, GetOpts,
    RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{lookup_interface, GCell, KlippyError, KResult, ObjectRef, PrinterObject};
use pycompat::text::{
    percent_format, round_py_nd, str_format, trunc_float_to_bigint, PctValue, PyError,
};
use pycompat::{PyInt, PyValue};
use std::sync::{Arc, OnceLock, Weak};

fn internal(class_name: &str, message: String) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message,
    }
}

/// `percent_format`/`str_format` cannot fail on the literals used here; the
/// mapping only keeps the class Python would raise.
fn fmt_err(e: PyError) -> KlippyError {
    internal(e.kind, e.msg)
}

fn index_error() -> KlippyError {
    internal("IndexError", "list index out of range".to_string())
}

/// The `int` a `getchoice` over an all-`int` mapping returns.
fn choice_int(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => i64::try_from(&n.0).map_err(|_| {
            internal(
                "OverflowError",
                "Python int too large to convert to C long".to_string(),
            )
        }),
        other => Err(internal(
            "TypeError",
            format!(
                "int() argument must be a string, a bytes-like object or a number, not '{}'",
                pycompat::value::type_name(other)
            ),
        )),
    }
}

/// `math.trunc(adjust)` (`screws_tilt_adjust.py:110`).
fn py_trunc(f: f64) -> KResult<PyInt> {
    if f.is_nan() {
        return Err(internal(
            "ValueError",
            "cannot convert float NaN to integer".to_string(),
        ));
    }
    if f.is_infinite() {
        return Err(internal(
            "OverflowError",
            "cannot convert float infinity to integer".to_string(),
        ));
    }
    Ok(trunc_float_to_bigint(f))
}

/// `threads_factor.get(self.thread, 0.5)` (`screws_tilt_adjust.py:91-92`).
fn thread_factor(thread: i64) -> f64 {
    match thread {
        0 | 1 => 0.5,
        2 | 3 => 0.7,
        4 | 5 => 0.8,
        6 | 7 => 1.0,
        _ => 0.5,
    }
}

/// One value of `self.results` (`screws_tilt_adjust.py:105-107`).
struct ScrewResult {
    z: f64,
    sign: String,
    adjust: String,
    is_base: bool,
}

/// `ScrewsTiltAdjust` (`klippy/extras/screws_tilt_adjust.py:10`).
pub struct ScrewsTiltAdjust {
    gcode: GCode,
    screws: Vec<(Vec<f64>, String)>,
    thread: i64,
    probe_helper: Arc<ProbePointsHelper>,
    max_diff: GCell<Option<f64>>,
    max_diff_error: GCell<bool>,
    results: GCell<IndexMap<String, ScrewResult>>,
    /// Python creates the attribute only in `cmd_SCREWS_TILT_CALCULATE`, so
    /// the outer `None` below means "no attribute yet" (`probe_finalize` then
    /// raises `AttributeError`), exactly like the reference.
    direction: GCell<Option<Option<String>>>,
}

impl ScrewsTiltAdjust {
    /// `ScrewsTiltAdjust.__init__` (`klippy/extras/screws_tilt_adjust.py:11`).
    pub fn new(config: &Config) -> KResult<Arc<ScrewsTiltAdjust>> {
        let mut screws: Vec<(Vec<f64>, String)> = Vec::new();
        for i in 0..99usize {
            let prefix = format!("screw{}", i + 1);
            let present = config.get(
                &prefix,
                GetStrOpts {
                    default: Some(None),
                    note_valid: true,
                },
            )?;
            if present.is_none() {
                break;
            }
            let coord = config
                .getfloatlist(
                    &prefix,
                    GetFloatListOpts {
                        default: None,
                        sep: ",".to_string(),
                        count: Some(2),
                        note_valid: true,
                        ..Default::default()
                    },
                )?
                .expect("a present option yields the parsed list");
            let default_name = percent_format(
                "screw at %.3f,%.3f",
                &[PctValue::Float(coord[0]), PctValue::Float(coord[1])],
            )
            .map_err(fmt_err)?;
            let name = config
                .get(
                    &format!("{prefix}_name"),
                    GetStrOpts {
                        default: Some(Some(default_name)),
                        note_valid: true,
                    },
                )?
                .expect("the default name is always returned");
            screws.push((coord, name));
        }
        if screws.len() < 3 {
            return Err(KlippyError::config(
                "screws_tilt_adjust: Must have at least three screws",
            ));
        }
        let thread_names = [
            "CW-M3", "CCW-M3", "CW-M4", "CCW-M4", "CW-M5", "CCW-M5", "CW-M6", "CCW-M6",
        ];
        let mut choices = IndexMap::new();
        for (idx, name) in thread_names.iter().enumerate() {
            choices.insert(
                PyValue::Str(name.to_string()),
                PyValue::Int(PyInt::from_i64(idx as i64)),
            );
        }
        let thread = choice_int(&config.getchoice(
            "screw_thread",
            &PyValue::Dict(choices),
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("CW-M3".to_string()))),
                note_valid: true,
            },
        )?)?;
        // `probe.ProbePointsHelper(config, self.probe_finalize, ...)` reaches
        // back into the object that is still being built, so the callback
        // reads the object through this slot (`self.probe_finalize` in
        // Python). The slot is filled before any command can run.
        let self_slot: Arc<OnceLock<Weak<ScrewsTiltAdjust>>> = Arc::new(OnceLock::new());
        let finalize = {
            let slot = Arc::clone(&self_slot);
            Box::new(move |positions: &[ProbeResult]| -> KResult<PyValue> {
                slot.get()
                    .expect("the helper is created inside the constructor")
                    .upgrade()
                    .expect("the probe helper is owned by the object it calls back")
                    .probe_finalize(positions)?;
                Ok(PyValue::None)
            }) as ProbeFinalizeCallback
        };
        let points: Vec<Vec<f64>> = screws.iter().map(|(c, _)| c.clone()).collect();
        let probe_helper = ProbePointsHelper::new(config, finalize, Some(points))?;
        probe_helper.minimum_points(3)?;
        let gcode = lookup_interface::<dyn GCodeApi>(
            &*config.get_printer(),
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let this = Arc::new(ScrewsTiltAdjust {
            gcode: gcode.clone(),
            screws,
            thread,
            probe_helper,
            max_diff: GCell::new(None),
            max_diff_error: GCell::new(false),
            results: GCell::new(IndexMap::new()),
            direction: GCell::new(None),
        });
        let _ = self_slot.set(Arc::downgrade(&this));
        let me = Arc::clone(&this);
        gcode.register_command(
            "SCREWS_TILT_CALCULATE",
            Some(Box::new(move |gcmd: &GCmd| me.cmd_screws_tilt_calculate(gcmd))
                as GCodeCommandHandler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(
                    "Tool to help adjust bed leveling screws by calculating the number of turns to level it."
                        .to_string(),
                ),
            },
        )?;
        Ok(this)
    }

    /// `cmd_SCREWS_TILT_CALCULATE` (`klippy/extras/screws_tilt_adjust.py:51`).
    pub fn cmd_screws_tilt_calculate(&self, gcmd: &GCmd) -> KResult<()> {
        self.max_diff.set(gcmd.get_float(
            "MAX_DEVIATION",
            GcmdFloatOpts {
                default: Some(None),
                ..Default::default()
            },
        )?);
        let direction = match gcmd.get(
            "DIRECTION",
            GetOpts {
                default: Some(PyValue::None),
                ..Default::default()
            },
        )? {
            PyValue::None => None,
            PyValue::Str(s) => {
                let upper = s.to_uppercase();
                if upper != "CW" && upper != "CCW" {
                    let msg = percent_format(
                        "Error on '%s': DIRECTION must be either CW or CCW",
                        &[PctValue::Str(gcmd.get_commandline())],
                    )
                    .map_err(fmt_err)?;
                    return Err(KlippyError::command(msg));
                }
                Some(upper)
            }
            other => {
                return Err(internal(
                    "AttributeError",
                    format!(
                        "'{}' object has no attribute 'upper'",
                        pycompat::value::type_name(&other)
                    ),
                ))
            }
        };
        self.direction.replace(Some(direction));
        self.probe_helper.start_probe(gcmd)
    }

    /// `probe_finalize` (`klippy/extras/screws_tilt_adjust.py:66`).
    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<()> {
        self.results.replace(IndexMap::new());
        self.max_diff_error.set(false);
        let thread = self.thread;
        let is_clockwise_thread = (thread & 1) == 0;
        let direction = self.direction.cloned().ok_or_else(|| {
            internal(
                "AttributeError",
                "'ScrewsTiltAdjust' object has no attribute 'direction'".to_string(),
            )
        })?;
        let mut screw_diff: Vec<f64> = Vec::new();
        let (i_base, z_base) = match &direction {
            Some(dir) => {
                let use_max = (is_clockwise_thread && dir == "CW")
                    || (!is_clockwise_thread && dir == "CCW");
                let Some(first) = positions.first() else {
                    return Err(internal(
                        "ValueError",
                        format!(
                            "{}() arg is an empty sequence",
                            if use_max { "max" } else { "min" }
                        ),
                    ));
                };
                // `max`/`min` over `enumerate(...)` keep the first extreme.
                let mut best = (0usize, first.bed_z);
                for (i, pos) in positions.iter().enumerate() {
                    let better = if use_max {
                        pos.bed_z > best.1
                    } else {
                        pos.bed_z < best.1
                    };
                    if better {
                        best = (i, pos.bed_z);
                    }
                }
                best
            }
            None => (0, positions.first().ok_or_else(index_error)?.bed_z),
        };
        self.gcode.respond_info(
            "01:20 means 1 full turn and 20 minutes, CW=clockwise, CCW=counter-clockwise",
            true,
        );
        for (i, (coord, name)) in self.screws.iter().enumerate() {
            let z = positions.get(i).ok_or_else(index_error)?.bed_z;
            if i == i_base {
                let msg = percent_format(
                    "%s : x=%.1f, y=%.1f, z=%.5f",
                    &[
                        PctValue::Str(format!("{name} (base)")),
                        PctValue::Float(coord[0]),
                        PctValue::Float(coord[1]),
                        PctValue::Float(z),
                    ],
                )
                .map_err(fmt_err)?;
                self.gcode.respond_info(&msg, true);
                let sign = if is_clockwise_thread { "CW" } else { "CCW" };
                self.results.borrow_mut().insert(
                    format!("screw{}", i + 1),
                    ScrewResult {
                        z,
                        sign: sign.to_string(),
                        adjust: "00:00".to_string(),
                        is_base: true,
                    },
                );
            } else {
                let diff = z_base - z;
                screw_diff.push(diff.abs());
                let adjust = if diff.abs() < 0.001 {
                    0.0
                } else {
                    diff / thread_factor(thread)
                };
                let sign = if is_clockwise_thread {
                    if adjust >= 0.0 {
                        "CW"
                    } else {
                        "CCW"
                    }
                } else if adjust >= 0.0 {
                    "CCW"
                } else {
                    "CW"
                };
                let adjust = adjust.abs();
                let full_turns = py_trunc(adjust)?;
                let decimal_part = adjust - full_turns.to_f64_exact().ok_or_else(|| {
                    internal(
                        "OverflowError",
                        "int too large to convert to float".to_string(),
                    )
                })?;
                let minutes = round_py_nd(decimal_part * 60.0, 0);
                let adjust_s = percent_format(
                    "%02d:%02d",
                    &[
                        PctValue::Int(full_turns.clone()),
                        PctValue::Float(minutes),
                    ],
                )
                .map_err(fmt_err)?;
                let msg = percent_format(
                    "%s : x=%.1f, y=%.1f, z=%.5f : adjust %s %02d:%02d",
                    &[
                        PctValue::Str(name.clone()),
                        PctValue::Float(coord[0]),
                        PctValue::Float(coord[1]),
                        PctValue::Float(z),
                        PctValue::Str(sign.to_string()),
                        PctValue::Int(full_turns),
                        PctValue::Float(minutes),
                    ],
                )
                .map_err(fmt_err)?;
                self.gcode.respond_info(&msg, true);
                self.results.borrow_mut().insert(
                    format!("screw{}", i + 1),
                    ScrewResult {
                        z,
                        sign: sign.to_string(),
                        adjust: adjust_s,
                        is_base: false,
                    },
                );
            }
        }
        if let Some(max_diff) = self.max_diff.get() {
            // `if self.max_diff and any(...)`: 0.0 is falsy, NaN is truthy.
            if max_diff != 0.0 && screw_diff.iter().any(|d| *d > max_diff) {
                self.max_diff_error.set(true);
                let msg = str_format(
                    "bed level exceeds configured limits ({}mm)! Adjust screws and restart print.",
                    &[PctValue::Float(max_diff)],
                    &[],
                )
                .map_err(fmt_err)?;
                return Err(KlippyError::command(msg));
            }
        }
        Ok(())
    }

    /// `get_status` (`klippy/extras/screws_tilt_adjust.py:61`).
    pub fn get_status_value(&self) -> PyValue {
        let mut results_map = IndexMap::new();
        for (k, v) in self.results.borrow().iter() {
            let mut inner = IndexMap::new();
            inner.insert(PyValue::Str("z".to_string()), PyValue::Float(v.z));
            inner.insert(
                PyValue::Str("sign".to_string()),
                PyValue::Str(v.sign.clone()),
            );
            inner.insert(
                PyValue::Str("adjust".to_string()),
                PyValue::Str(v.adjust.clone()),
            );
            inner.insert(
                PyValue::Str("is_base".to_string()),
                PyValue::Bool(v.is_base),
            );
            results_map.insert(PyValue::Str(k.clone()), PyValue::Dict(inner));
        }
        let mut out = IndexMap::new();
        out.insert(
            PyValue::Str("error".to_string()),
            PyValue::Bool(self.max_diff_error.get()),
        );
        out.insert(
            PyValue::Str("max_deviation".to_string()),
            match self.max_diff.get() {
                Some(f) => PyValue::Float(f),
                None => PyValue::None,
            },
        );
        out.insert(
            PyValue::Str("results".to_string()),
            PyValue::Dict(results_map),
        );
        PyValue::Dict(out)
    }
}

impl PrinterObject for ScrewsTiltAdjust {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.get_status_value())
    }
}

/// `load_config(config)` (`klippy/extras/screws_tilt_adjust.py:129`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = ScrewsTiltAdjust::new(config)?;
    Ok(Some(obj as ObjectRef))
}

pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
