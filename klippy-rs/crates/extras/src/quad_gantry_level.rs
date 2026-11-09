//! Port of the `quad_gantry_level` module (`klippy/extras/quad_gantry_level.py`).
//!
//! Mechanical conforming of a 4-Z-stepper moving gantry to the bed.
//! Owns `QUAD_GANTRY_LEVEL`, reads `[quad_gantry_level]`
//! `max_adjust`, `horizontal_move_z` and `gantry_corners`, and logs
//! `quad_gantry_level f1: ...` / `af: ...` via `logging.info`.
//!
//! Helpers come from their owners: `z_tilt` (`RetryHelper`,
//! `ZAdjustStatus`, `ZAdjustHelper`) and `probe` (`ProbePointsHelper`)
//! are constructed directly in [`QuadGantryLevel::new`], exactly like
//! the reference constructor. Nothing is copied from those modules and
//! nothing is injected through setters or constructors.

use crate::manual_probe::ProbeResult;
use crate::probe::{ProbeFinalizeCallback, ProbePointsHelper};
use crate::z_tilt::{RetryHelper, ZAdjustHelper, ZAdjustStatus};
use klippy_api::config::{Config, GetFloatOpts, GetListsOpts, ListParser};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi, GCmd, RegisterCommandOpts};
use klippy_api::log as klog;
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{lookup_interface, ObjectRef, PrinterObject};
use pycompat::text::{float_true_div, percent_format, py_sum_floats, PctValue, PyError};
use pycompat::{PyValue, PyInt};
use std::sync::{Arc, OnceLock, Weak};

fn py_err(err: PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: err.kind.to_string(),
        message: err.msg,
    }
}

fn pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(py_err)
}

fn pct_str(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|_| fmt.to_string())
}

fn float_div(a: f64, b: f64) -> KResult<f64> {
    float_true_div(a, b).map_err(py_err)
}

/// Python `max` with `>` comparison (iteration order matters for NaN).
pub fn py_max(values: &[f64]) -> f64 {
    // Python raises ValueError on empty sequence: "max() arg is an empty sequence".
    // Callers never pass an empty slice (z_positions has 4 items).
    if values.is_empty() {
        return f64::NAN;
    }
    let mut max = values[0];
    for &v in &values[1..] {
        if v > max {
            max = v;
        }
    }
    max
}

/// `quad_gantry_level.linefit` (`quad_gantry_level.py:115`).
pub fn linefit(p1: [f64; 2], p2: [f64; 2]) -> KResult<(f64, f64)> {
    if p1[1] == p2[1] {
        return Ok((0.0, p1[1]));
    }
    let m = float_div(p2[1] - p1[1], p2[0] - p1[0])?;
    let b = p1[1] - m * p1[0];
    Ok((m, b))
}

/// `quad_gantry_level.plot` (`quad_gantry_level.py:122`).
pub fn plot(fit: (f64, f64), x: f64) -> f64 {
    fit.0 * x + fit.1
}

// ---------------------------------------------------------------------------
// QuadGantryLevel (quad_gantry_level.py:25).
// ---------------------------------------------------------------------------

/// `QuadGantryLevel` (`klippy/extras/quad_gantry_level.py:25`).
pub struct QuadGantryLevel {
    gcode: GCode,
    max_adjust: f64,
    horizontal_move_z: f64,
    gantry_corners: Vec<[f64; 2]>,
    retry_helper: RetryHelper,
    probe_helper: Arc<ProbePointsHelper>,
    z_status: Arc<ZAdjustStatus>,
    z_helper: Arc<ZAdjustHelper>,
}

fn parse_float_pairs(value: &PyValue) -> Vec<[f64; 2]> {
    match value {
        PyValue::Tuple(items) | PyValue::List(items) => items
            .iter()
            .filter_map(|item| match item {
                PyValue::Tuple(inner) | PyValue::List(inner) if inner.len() == 2 => match (&inner[0], &inner[1]) {
                    (PyValue::Float(a), PyValue::Float(b)) => Some([*a, *b]),
                    _ => None,
                },
                _ => None,
            })
            .collect(),
        _ => Vec::new(),
    }
}

fn tuple_repr(m: f64, b: f64) -> String {
    // Python `linefit` returns int `0` for a flat line, so repr is
    // `(0, <b>)`, not `(0.0, <b>)`. A computed slope is zero only in
    // that flat case, so comparison by value is exact here.
    let m_val = if m == 0.0 {
        PyValue::Int(PyInt::from_i64(0))
    } else {
        PyValue::Float(m)
    };
    PyValue::Tuple(vec![m_val, PyValue::Float(b)]).repr()
}

impl QuadGantryLevel {
    /// `QuadGantryLevel.__init__(config)` (`quad_gantry_level.py:26`).
    ///
    /// Helper construction follows the reference order exactly, so
    /// config errors surface in the same sequence: `RetryHelper`,
    /// `max_adjust`, `horizontal_move_z`, `ProbePointsHelper` (which
    /// reads `points`, `horizontal_move_z` and `speed` itself),
    /// the 4-probe-point check, `ZAdjustStatus`, `ZAdjustHelper`,
    /// `gantry_corners` and its length check, then the `gcode`
    /// lookup and command registration.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let retry_helper = RetryHelper::new(config, "Possibly Z motor numbering is wrong")?;
        let max_adjust = config
            .getfloat("max_adjust", GetFloatOpts {
                default: Some(Some(4.0)),
                minval: None,
                maxval: None,
                above: Some(0.0),
                below: None,
                ints: klippy_api::config::IntArgs::DEFAULT | klippy_api::config::IntArgs::ABOVE,
                note_valid: true,
            })?
            .unwrap_or(4.0);
        let horizontal_move_z = config
            .getfloat("horizontal_move_z", GetFloatOpts {
                default: Some(Some(5.0)),
                minval: None,
                maxval: None,
                above: None,
                below: None,
                note_valid: true,
                ..Default::default()
            })?
            .unwrap_or(5.0);
        // The probe helper takes this object's `probe_finalize` as its
        // finalize callback, like the bound method in the reference.
        // The object is built first and the helper installed after;
        // the callback reaches the object through a `Weak`, filled in
        // once the object exists, so no reference cycle forms.
        let self_slot: Arc<OnceLock<Weak<QuadGantryLevel>>> = Arc::new(OnceLock::new());
        let callback: ProbeFinalizeCallback = {
            let slot = Arc::clone(&self_slot);
            Box::new(move |positions: &[ProbeResult]| -> KResult<PyValue> {
                let this = slot
                    .get()
                    .expect("the helper is created inside the constructor")
                    .upgrade()
                    .expect("the probe helper is owned by the object it calls back");
                Ok(PyValue::Str(this.probe_finalize(positions)?))
            })
        };
        let probe_helper = ProbePointsHelper::new(config, callback, None)?;
        if probe_helper.probe_points().len() != 4 {
            return Err(KlippyError::config(
                "Need exactly 4 probe points for quad_gantry_level",
            ));
        }
        let z_status = ZAdjustStatus::new(&printer);
        let z_helper = ZAdjustHelper::new(config, 4);
        let gantry_corners_val = config.getlists("gantry_corners", GetListsOpts {
            default: None,
            seps: vec![",".to_string(), "\n".to_string()],
            count: Some(2),
            parser: ListParser::Float,
            note_valid: true,
        })?;
        let gantry_corners = parse_float_pairs(&gantry_corners_val);
        if gantry_corners.len() < 2 {
            return Err(KlippyError::config(
                "quad_gantry_level requires at least two gantry_corners",
            ));
        }
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = Arc::new(QuadGantryLevel {
            gcode: gcode.clone(),
            max_adjust,
            horizontal_move_z,
            gantry_corners,
            retry_helper,
            probe_helper,
            z_status,
            z_helper,
        });
        let _ = self_slot.set(Arc::downgrade(&this));
        let cell = Arc::clone(&this);
        gcode.register_command(
            "QUAD_GANTRY_LEVEL",
            Some(Box::new(move |gcmd: &GCmd| cell.cmd_quad_gantry_level(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(
                    "Conform a moving, twistable gantry to the shape of a stationary bed"
                        .to_string(),
                ),
            },
        )?;
        Ok(this)
    }

    pub fn max_adjust(&self) -> f64 {
        self.max_adjust
    }

    pub fn horizontal_move_z(&self) -> f64 {
        self.horizontal_move_z
    }

    pub fn gantry_corners(&self) -> Vec<[f64; 2]> {
        self.gantry_corners.clone()
    }

    /// `cmd_QUAD_GANTRY_LEVEL` (`quad_gantry_level.py:50`).
    pub fn cmd_quad_gantry_level(&self, gcmd: &GCmd) -> KResult<()> {
        self.z_status.reset();
        self.retry_helper.start(gcmd)?;
        self.probe_helper.start_probe(gcmd)
    }

    /// `probe_finalize` (`quad_gantry_level.py:54`).
    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<String> {
        // `positions[i]` in the reference is a plain list index: the
        // points message is emitted first, then the first out-of-range
        // access raises IndexError.
        fn at(positions: &[ProbeResult], index: usize) -> KResult<&ProbeResult> {
            positions.get(index).ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            })
        }
        let corners = &self.gantry_corners;
        let z_positions: Vec<f64> = positions.iter().map(|p| self.horizontal_move_z - p.bed_z).collect();
        let mut parts: Vec<String> = Vec::new();
        for (z_id, z_position) in z_positions.iter().enumerate() {
            parts.push(pct_str("%s: %.6f", &[
                PctValue::Int((z_id as i64).into()),
                PctValue::Float(*z_position),
            ]));
        }
        let points_message = pct("Gantry-relative probe points:\n%s\n", &[
            PctValue::Str(parts.join(" ")),
        ])?;
        self.gcode.respond_info(&points_message, true);

        let ppx0 = [at(positions, 0)?.bed_x, z_positions[0]];
        let ppx3 = [at(positions, 3)?.bed_x, z_positions[3]];
        let slope_x_pp03 = linefit(ppx0, ppx3)?;
        let ppx1 = [at(positions, 1)?.bed_x, z_positions[1]];
        let ppx2 = [at(positions, 2)?.bed_x, z_positions[2]];
        let slope_x_pp12 = linefit(ppx1, ppx2)?;
        klog::info(&pct_str("quad_gantry_level f1: %s, f2: %s", &[
            PctValue::Str(tuple_repr(slope_x_pp03.0, slope_x_pp03.1)),
            PctValue::Str(tuple_repr(slope_x_pp12.0, slope_x_pp12.1)),
        ]));

        let a1 = [at(positions, 0)?.bed_y, plot(slope_x_pp03, corners[0][0])];
        let a2 = [at(positions, 1)?.bed_y, plot(slope_x_pp12, corners[0][0])];
        let slope_y_s01 = linefit(a1, a2)?;
        let b1 = [at(positions, 0)?.bed_y, plot(slope_x_pp03, corners[1][0])];
        let b2 = [at(positions, 1)?.bed_y, plot(slope_x_pp12, corners[1][0])];
        let slope_y_s23 = linefit(b1, b2)?;
        klog::info(&pct_str("quad_gantry_level af: %s, bf: %s", &[
            PctValue::Str(tuple_repr(slope_y_s01.0, slope_y_s01.1)),
            PctValue::Str(tuple_repr(slope_y_s23.0, slope_y_s23.1)),
        ]));

        let z_height = [
            plot(slope_y_s01, corners[0][1]),
            plot(slope_y_s01, corners[1][1]),
            plot(slope_y_s23, corners[1][1]),
            plot(slope_y_s23, corners[0][1]),
        ];
        let labels = ["z", "z1", "z2", "z3"];
        let mut apos_parts: Vec<String> = Vec::new();
        for i in 0..4 {
            apos_parts.push(pct_str("%s: %06f", &[
                PctValue::Str(labels[i].to_string()),
                PctValue::Float(z_height[i]),
            ]));
        }
        self.gcode.respond_info(
            &format!("Actuator Positions:\n{}", apos_parts.join(" ")),
            true,
        );

        let z_ave = float_div(py_sum_floats(&z_height), 4.0)?;
        self.gcode.respond_info(&pct("Average: %0.6f", &[PctValue::Float(z_ave)])?, true);
        let z_adjust: Vec<f64> = z_height.iter().map(|&z| z_ave - z).collect();

        let adjust_max = py_max(&z_adjust);
        if adjust_max > self.max_adjust {
            let message = pct(
                "Aborting quad_gantry_level required adjustment %0.6f is greater than max_adjust %0.6f",
                &[PctValue::Float(adjust_max), PctValue::Float(self.max_adjust)],
            )?;
            return Err(KlippyError::Command(message));
        }

        let speed = self.probe_helper.get_lift_speed();
        self.z_helper.adjust_steppers(&z_adjust, speed)?;
        let retry = self.retry_helper.check_retry(&z_positions)?;
        Ok(self.z_status.check_retry_result(&retry))
    }

    /// `get_status` (`quad_gantry_level.py:124`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.z_status.get_status(eventtime)
    }
}

impl PrinterObject for QuadGantryLevel {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(QuadGantryLevel::get_status(self, eventtime)))
    }
}

/// `load_config` (`quad_gantry_level.py:127`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(QuadGantryLevel::new(config)? as ObjectRef))
}

/// Module entry point for the `quad_gantry_level` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
