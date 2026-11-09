//! Port of the `bed_tilt` module (`klippy/extras/bed_tilt.py`).
//!
//! The module registers itself as the `gcode_move` move transform and adjusts
//! the toolhead Z position by `x * x_adjust + y * y_adjust + z_adjust`.
//! When `points` is configured it also registers `BED_TILT_CALIBRATE`, which
//! probes the configured points and solves for the adjustments that best fit
//! `a + b * bed_x + c * bed_y = bed_z`.
//!
//! * `mathutil.solve_linear_equations` is called from
//!   `klippy_core::mathutil` (`klippy/mathutil.py:216`); the Python call
//!   `solve_linear_equations(eqs, ans)` uses the default
//!   `allow_underdetermined=False`.

use crate::manual_probe::ProbeResult;
use crate::probe::ProbePointsHelper;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GCodeMoveApi, MoveTransform, MoveTransformApi,
    RegisterCommandOpts, GCmd,
};
use klippy_api::log;
use klippy_api::{load_interface, lookup_interface};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::{GCell, ObjectRef};
use pycompat::text::{float_repr, percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// Help text for `BED_TILT_CALIBRATE` (`klippy/extras/bed_tilt.py:58`).
pub const CMD_BED_TILT_CALIBRATE_HELP: &str = "Bed tilt calibration script";

/// `str(positions)` for the `Calculating bed_tilt with: %s` log line
/// (`bed_tilt.py:62`). Positions are `probe_result` namedtuples, so the
/// repr is `probe_result(bed_x=..., ...)` with CPython float reprs.
pub fn probe_positions_repr(positions: &[ProbeResult]) -> String {
    let items: Vec<String> = positions
        .iter()
        .map(|pos| {
            format!(
                "probe_result(bed_x={}, bed_y={}, bed_z={}, test_x={}, test_y={}, test_z={})",
                float_repr(pos.bed_x),
                float_repr(pos.bed_y),
                float_repr(pos.bed_z),
                float_repr(pos.test_x),
                float_repr(pos.test_y),
                float_repr(pos.test_z),
            )
        })
        .collect();
    format!("[{}]", items.join(", "))
}


fn percent_format_or_err(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}


// ---------------------------------------------------------------------------
// BedTilt - `klippy/extras/bed_tilt.py:10-44`.
// ---------------------------------------------------------------------------

/// `BedTilt` (`bed_tilt.py:10-23`).
pub struct BedTilt {
    printer: Printer,
    x_adjust: GCell<f64>,
    y_adjust: GCell<f64>,
    z_adjust: GCell<f64>,
    toolhead: GCell<Option<ToolHead>>,
    calibrate: GCell<Option<Arc<BedTiltCalibrate>>>,
}

impl BedTilt {
    /// `BedTilt(config)` (`bed_tilt.py:11-23`). The real Python constructor
    /// needs the live `printer` object, so the returned value is the
    /// `Arc<BedTilt>` that is also what `load_config` stores.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let bed_tilt = Arc::new(Self {
            printer: printer.clone(),
            x_adjust: GCell::new(0.0),
            y_adjust: GCell::new(0.0),
            z_adjust: GCell::new(0.0),
            toolhead: GCell::new(None),
            calibrate: GCell::new(None),
        });
        // Register move transform handler (Python registers the bound
        // method before reading any option).
        {
            let bt = Arc::clone(&bed_tilt);
            printer.register_event_handler(
                "klippy:connect",
                Box::new(move |_params| {
                    bt.handle_connect()?;
                    Ok(PyValue::None)
                }),
            );
        }
        let default_zero = Some(Some(0.0_f64));
        let x_adjust = config
            .getfloat(
                "x_adjust",
                GetFloatOpts { default: default_zero, ..Default::default() },
            )?
            .unwrap_or(0.0);
        let y_adjust = config
            .getfloat(
                "y_adjust",
                GetFloatOpts { default: default_zero, ..Default::default() },
            )?
            .unwrap_or(0.0);
        let z_adjust = config
            .getfloat(
                "z_adjust",
                GetFloatOpts { default: default_zero, ..Default::default() },
            )?
            .unwrap_or(0.0);
        bed_tilt.x_adjust.set(x_adjust);
        bed_tilt.y_adjust.set(y_adjust);
        bed_tilt.z_adjust.set(z_adjust);
        // `if config.get('points', None) is not None` (`bed_tilt.py:18`).
        let has_points = config
            .get(
                "points",
                GetStrOpts { default: Some(None), ..Default::default() },
            )?
            .is_some();
        if has_points {
            let calibrate = BedTiltCalibrate::new(
                config,
                printer.clone(),
                Arc::clone(&bed_tilt),
            )?;
            *bed_tilt.calibrate.borrow_mut() = Some(calibrate);
        }
        // Register move transform with g-code class (`bed_tilt.py:22-23`).
        let gcode_move = load_interface::<dyn GCodeMoveApi>(&*printer, config, "gcode_move", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let transform: Arc<dyn MoveTransformApi> = Arc::clone(&bed_tilt) as Arc<dyn MoveTransformApi>;
        gcode_move.set_move_transform(MoveTransform::Custom(transform), false)?;
        Ok(bed_tilt)
    }

    /// Current `x_adjust` (the Python `self.x_adjust` attribute).
    pub fn x_adjust(&self) -> f64 {
        self.x_adjust.get()
    }

    /// Current `y_adjust` (the Python `self.y_adjust` attribute).
    pub fn y_adjust(&self) -> f64 {
        self.y_adjust.get()
    }

    /// Current `z_adjust` (the Python `self.z_adjust` attribute).
    pub fn z_adjust(&self) -> f64 {
        self.z_adjust.get()
    }

    /// The calibrator, when `points` was configured (`None` otherwise).
    pub fn calibrate(&self) -> Option<Arc<BedTiltCalibrate>> {
        self.calibrate.borrow().clone()
    }

    /// `BedTilt.handle_connect` (`bed_tilt.py:24-25`).
    pub fn handle_connect(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("Raise never yields None");
        *self.toolhead.borrow_mut() = Some(toolhead);
        Ok(())
    }

    /// `BedTilt.update_adjust` (`bed_tilt.py:35-44`).
    pub fn update_adjust(
        &self,
        x_adjust: f64,
        y_adjust: f64,
        z_adjust: f64,
    ) -> KResult<()> {
        self.x_adjust.set(x_adjust);
        self.y_adjust.set(y_adjust);
        self.z_adjust.set(z_adjust);
        let gcode_move = lookup_interface::<dyn GCodeMoveApi>(&*self.printer, "gcode_move", LookupDefault::Raise)?
            .expect("Raise never yields None");
        gcode_move.reset_last_position();
        let configfile = lookup_interface::<dyn klippy_api::config::PrinterConfigApi>(&*self.printer, "configfile", LookupDefault::Raise)?
            .expect("Raise never yields None");
        for (option, value) in [
            ("x_adjust", x_adjust),
            ("y_adjust", y_adjust),
            ("z_adjust", z_adjust),
        ] {
            let text = percent_format_or_err("%.6f", &[PctValue::Float(value)])?;
            configfile.set("bed_tilt", option, &text);
        }
        Ok(())
    }

    fn adjustments(&self) -> (f64, f64, f64) {
        (self.x_adjust.get(), self.y_adjust.get(), self.z_adjust.get())
    }
}

impl MoveTransformApi for BedTilt {
    /// `BedTilt.get_position` (`bed_tilt.py:26-30`).
    fn get_position(&self) -> KResult<Vec<f64>> {
        let toolhead = match self.toolhead.borrow().clone() {
            Some(toolhead) => toolhead,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message:
                        "'NoneType' object has no attribute 'get_position'".to_string(),
                })
            }
        };
        let pos = toolhead.get_position();
        if pos.len() < 3 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: format!(
                    "not enough values to unpack (expected 3, got {})",
                    pos.len()
                ),
            });
        }
        let (x_adjust, y_adjust, z_adjust) = self.adjustments();
        let mut out = pos.clone();
        // `z -= x * self.x_adjust + y * self.y_adjust + self.z_adjust`
        out[2] -= pos[0] * x_adjust + pos[1] * y_adjust + z_adjust;
        Ok(out)
    }

    /// `BedTilt.move` (`bed_tilt.py:31-34`).
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let toolhead = match self.toolhead.borrow().clone() {
            Some(toolhead) => toolhead,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message:
                        "'NoneType' object has no attribute 'move'".to_string(),
                })
            }
        };
        if newpos.len() < 3 {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: format!(
                    "not enough values to unpack (expected 3, got {})",
                    newpos.len()
                ),
            });
        }
        let (x_adjust, y_adjust, z_adjust) = self.adjustments();
        let mut out = newpos.clone();
        out[2] += newpos[0] * x_adjust + newpos[1] * y_adjust + z_adjust;
        toolhead.r#move(out, speed)
    }
}

klippy_api::printer_object!(BedTilt; dyn MoveTransformApi);

// ---------------------------------------------------------------------------
// BedTiltCalibrate - `bed_tilt.py:47-84`.
// ---------------------------------------------------------------------------

struct CalibrateInner {
    printer: Printer,
    bedtilt: std::sync::Weak<BedTilt>,
    /// The `gcode` object, set after `minimum_points` exactly like Python.
    gcode: GCell<Option<GCode>>,
}

impl CalibrateInner {
    /// `BedTiltCalibrate.probe_finalize` (`bed_tilt.py:61-84`).
    fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<PyValue> {
        // `logging.info("Calculating bed_tilt with: %s", positions)`.
        log::info(&percent_format_or_err(
            "Calculating bed_tilt with: %s",
            &[PctValue::Str(probe_positions_repr(positions))],
        )?);
        // Find best fit for `a + b*bed_x + c*bed_y = bed_z`.
        let mut eqs: Vec<Vec<f64>> = Vec::with_capacity(positions.len());
        let mut ans: Vec<Vec<f64>> = Vec::with_capacity(positions.len());
        for pos in positions {
            eqs.push(vec![1.0, pos.bed_x, pos.bed_y]);
            ans.push(vec![pos.bed_z]);
        }
        let res = match klippy_core::mathutil::solve_linear_equations(&eqs, &ans, false)? {
            Some(res) => res,
            None => return Err(KlippyError::command("Unable to calculate tilt")),
        };
        let z_adjust = res[0][0];
        let x_adjust = res[1][0];
        let y_adjust = res[2][0];
        // Update current bed_tilt calculations.
        let bedtilt = self.bedtilt.upgrade().ok_or_else(|| KlippyError::Internal {
            class_name: "ReferenceError".to_string(),
            message: "bed_tilt object has been freed".to_string(),
        })?;
        bedtilt.update_adjust(x_adjust, y_adjust, z_adjust)?;
        // Log and report results.
        let msg = percent_format_or_err(
            "x_adjust: %.6f y_adjust: %.6f z_adjust: %.6f",
            &[
                PctValue::Float(x_adjust),
                PctValue::Float(y_adjust),
                PctValue::Float(z_adjust),
            ],
        )?;
        let rollover =
            percent_format_or_err("bed_tilt: %s", &[PctValue::Str(msg.clone())])?;
        self.printer
            .set_rollover_info("bed_tilt", Some(&rollover), true);
        let gcode = match self.gcode.borrow().clone() {
            Some(gcode) => gcode,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'respond_info'"
                        .to_string(),
                })
            }
        };
        let respond = percent_format_or_err(
            "%s\nThe above parameters have been applied to the current\nsession. The SAVE_CONFIG command will update the printer\nconfig file and restart the printer.",
            &[PctValue::Str(msg)],
        )?;
        gcode.respond_info(&respond, true);
        Ok(PyValue::None)
    }
}

/// `BedTiltCalibrate` (`bed_tilt.py:47-84`).
pub struct BedTiltCalibrate {
    inner: Arc<CalibrateInner>,
    probe_helper: Arc<ProbePointsHelper>,
}

impl BedTiltCalibrate {
    /// `BedTiltCalibrate(config, bedtilt)` (`bed_tilt.py:48-57`). The probe
    /// helper is owned by the `probe` module
    /// (`crate::probe::ProbePointsHelper`).
    pub fn new(
        config: &Config,
        printer: Printer,
        bedtilt: Arc<BedTilt>,
    ) -> KResult<Arc<Self>> {
        let inner = Arc::new(CalibrateInner {
            printer: printer.clone(),
            bedtilt: Arc::downgrade(&bedtilt),
            gcode: GCell::new(None),
        });
        let weak = Arc::downgrade(&inner);
        let probe_helper = ProbePointsHelper::new(
            config,
            Box::new(move |positions| {
                let inner = weak
                    .upgrade()
                    .ok_or_else(|| KlippyError::Internal {
                        class_name: "ReferenceError".to_string(),
                        message: "bed_tilt calibrator has been freed".to_string(),
                    })?;
                inner.probe_finalize(positions)
            }),
            None,
        )?;
        probe_helper.minimum_points(3)?;
        // Register BED_TILT_CALIBRATE command.
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        *inner.gcode.borrow_mut() = Some(gcode.clone());
        let calibrate = Arc::new(Self {
            inner,
            probe_helper,
        });
        {
            let calib = Arc::clone(&calibrate);
            let handler: GCodeCommandHandler = Box::new(move |gcmd: &GCmd| {
                calib.cmd_bed_tilt_calibrate(gcmd)
            });
            gcode.register_command(
                "BED_TILT_CALIBRATE",
                Some(handler),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_BED_TILT_CALIBRATE_HELP.to_string()),
                },
            )?;
        }
        Ok(calibrate)
    }

    /// `BedTiltCalibrate.cmd_BED_TILT_CALIBRATE` (`bed_tilt.py:59-60`).
    pub fn cmd_bed_tilt_calibrate(&self, gcmd: &GCmd) -> KResult<()> {
        self.probe_helper.start_probe(gcmd)
    }

    /// `BedTiltCalibrate.probe_finalize` (`bed_tilt.py:61-84`) through the
    /// stored finalize path.
    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<PyValue> {
        self.inner.probe_finalize(positions)
    }
}


// `load_config(config)` (`bed_tilt.py:86-87`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = BedTilt::new(config)?;
    Ok(Some(obj as ObjectRef))
}

/// Module entry point for the `bed_tilt` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
