//! Port of the `z_tilt` module (`klippy/extras/z_tilt.py`).
//!
//! Mechanical bed tilt calibration for printers with multiple Z steppers:
//! the `[z_tilt]` (and prefix) section probes the bed, fits a plane
//! (`a + b*bed_x + c*bed_y = bed_z`) with least squares and moves each Z
//! stepper so the gantry sits level. [`module_factory`] exposes
//! `load_config`, matching the Python module.

use std::sync::Arc;

use crate::manual_probe::ProbeResult;
use crate::probe::ProbePointsHelper;
use klippy_api::config::{
    Config, GetFloatOpts, GetIntOpts, GetListsOpts, ListParser,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GetFloatOpts as CmdGetFloatOpts,
    GetIntOpts as CmdGetIntOpts, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT,
    EV_STEPPER_ENABLE_MOTOR_OFF,
};
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::{lookup_interface, GCell};
use klippy_core::mathutil::{solve_linear_equations, Matrix};
use pycompat::text::{float_repr, percent_format, round_py_nd, PctValue};
use pycompat::PyValue;

const VALUE_LABEL: &str = "Probed points range";

fn pct(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant z_tilt format")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(pycompat::PyInt::from_i64(v))
}

// ---------------------------------------------------------------------------
// `ZAdjustStatus` (`klippy/extras/z_tilt.py:69-83`).
// ---------------------------------------------------------------------------

/// Tracks whether the Z tilt has been applied since the steppers last
/// disabled (`stepper_enable:motor_off` clears it).
pub struct ZAdjustStatus {
    applied: GCell<bool>,
}

impl ZAdjustStatus {
    /// `ZAdjustStatus.__init__`: registers `stepper_enable:motor_off`.
    pub fn new(printer: &Printer) -> Arc<Self> {
        let status = Arc::new(ZAdjustStatus {
            applied: GCell::new(false),
        });
        let handler_status = Arc::clone(&status);
        printer.register_event_handler(
            EV_STEPPER_ENABLE_MOTOR_OFF,
            Box::new(move |_params: &[EventArg]| {
                handler_status.motor_off();
                Ok(PyValue::None)
            }),
        );
        status
    }

    /// `ZAdjustStatus.check_retry_result` (`z_tilt.py:74`).
    pub fn check_retry_result(&self, retry_result: &str) -> String {
        if retry_result == "done" {
            self.applied.set(true);
        }
        retry_result.to_string()
    }

    /// `ZAdjustStatus.reset` (`z_tilt.py:78`).
    pub fn reset(&self) {
        self.applied.set(false);
    }

    /// `ZAdjustStatus.get_status` (`z_tilt.py:80`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut dict = indexmap::IndexMap::new();
        dict.insert(
            PyValue::Str("applied".to_string()),
            PyValue::Bool(self.applied.get()),
        );
        PyValue::Dict(dict)
    }

    /// `ZAdjustStatus._motor_off` (`z_tilt.py:82`): the `motor_off`
    /// event forgets the applied state.
    pub fn motor_off(&self) {
        self.reset();
    }
}

// ---------------------------------------------------------------------------
// `RetryHelper` (`klippy/extras/z_tilt.py:85-125`).
// ---------------------------------------------------------------------------

/// `RetryHelper` attributes. Python creates them one by one in `start`, so
/// every field is `None` until assigned and reading it early raises the same
/// `AttributeError` as the reference.
#[derive(Default)]
struct RetryState {
    max_retries: Option<i64>,
    retry_tolerance: Option<f64>,
    current_retry: Option<i64>,
    previous: Option<Option<f64>>,
    increasing: Option<i64>,
}

/// `AttributeError` for an attribute `start` has not assigned yet.
fn unstarted(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'RetryHelper' object has no attribute '{attr}'"),
    }
}

/// Repeated probing until the probed points stop varying.
pub struct RetryHelper {
    gcode: GCode,
    default_max_retries: i64,
    default_retry_tolerance: f64,
    error_msg_extra: String,
    state: GCell<RetryState>,
}

impl RetryHelper {
    /// `RetryHelper.__init__(config, error_msg_extra="")` (`z_tilt.py:86`);
    /// `quad_gantry_level` passes its own text.
    pub fn new(config: &Config, error_msg_extra: &str) -> KResult<Self> {
        let printer = config.get_printer();
        let gcode: GCode = lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let int_opts = GetIntOpts {
            default: Some(Some(0)),
            minval: Some(0),
            ..Default::default()
        };
        let default_max_retries = config.getint("retries", int_opts)?.unwrap_or(0);
        let float_opts = GetFloatOpts {
            default: Some(Some(0.0)),
            above: Some(0.0),
            ..Default::default()
        };
        let default_retry_tolerance = config
            .getfloat("retry_tolerance", float_opts)?
            .unwrap_or(0.0);
        Ok(RetryHelper {
            gcode,
            default_max_retries,
            default_retry_tolerance,
            error_msg_extra: error_msg_extra.to_string(),
            state: GCell::new(RetryState::default()),
        })
    }

    /// `RetryHelper.start` (`z_tilt.py:93`). Each attribute is assigned in
    /// the reference's order, so a parameter error leaves the earlier ones
    /// set.
    pub fn start(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let int_opts = CmdGetIntOpts {
            default: Some(Some(self.default_max_retries)),
            minval: Some(0),
            maxval: Some(30),
        };
        let max_retries = gcmd
            .get_int("RETRIES", int_opts)?
            .unwrap_or(self.default_max_retries);
        self.state.borrow_mut().max_retries = Some(max_retries);
        let float_opts = CmdGetFloatOpts {
            default: Some(Some(self.default_retry_tolerance)),
            minval: Some(0.0),
            maxval: Some(1.0),
            ..Default::default()
        };
        let retry_tolerance = gcmd
            .get_float("RETRY_TOLERANCE", float_opts)?
            .unwrap_or(self.default_retry_tolerance);
        let mut state = self.state.borrow_mut();
        state.retry_tolerance = Some(retry_tolerance);
        state.current_retry = Some(0);
        state.previous = Some(None);
        state.increasing = Some(0);
        Ok(())
    }

    /// `RetryHelper.check_increase` (`z_tilt.py:102`).
    pub fn check_increase(&self, error: f64) -> KResult<bool> {
        let mut state = self.state.borrow_mut();
        let previous = state.previous.ok_or_else(|| unstarted("previous"))?;
        // `if self.previous and error > self.previous + 0.0000001`:
        // a stored previous counts only when Python-truthy, i.e. a
        // non-zero float (None and 0.0 / -0.0 are falsy; NaN is truthy).
        let increasing = state.increasing.ok_or_else(|| unstarted("increasing"))?;
        let mut increasing = increasing;
        if matches!(previous, Some(p) if p != 0.0) && error > previous.unwrap() + 0.0000001 {
            increasing += 1;
        } else if increasing > 0 {
            increasing -= 1;
        }
        state.previous = Some(Some(error));
        state.increasing = Some(increasing);
        Ok(increasing > 1)
    }

    /// `RetryHelper.check_retry` (`z_tilt.py:109`).
    pub fn check_retry(&self, z_positions: &[f64]) -> KResult<String> {
        let max_retries = self
            .state
            .borrow()
            .max_retries
            .ok_or_else(|| unstarted("max_retries"))?;
        if max_retries == 0 {
            return Ok("done".to_string());
        }
        if z_positions.is_empty() {
            return Err(KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: "max() arg is an empty sequence".to_string(),
            });
        }
        let max_z = z_positions
            .iter()
            .fold(None::<f64>, |acc, v| match acc {
                None => Some(*v),
                Some(a) => Some(if *v > a { *v } else { a }),
            });
        let min_z = z_positions
            .iter()
            .fold(None::<f64>, |acc, v| match acc {
                None => Some(*v),
                Some(a) => Some(if *v < a { *v } else { a }),
            });
        let error = round_py_nd(max_z.unwrap() - min_z.unwrap(), 6);
        // `respond_info("Retries: %d/%d %s: ..." % (self.current_retry,
        // self.max_retries, ..., self.retry_tolerance))`: the arguments are
        // read left to right.
        let (current_retry, retry_tolerance) = {
            let state = self.state.borrow();
            let current_retry = state
                .current_retry
                .ok_or_else(|| unstarted("current_retry"))?;
            let retry_tolerance = state
                .retry_tolerance
                .ok_or_else(|| unstarted("retry_tolerance"))?;
            (current_retry, retry_tolerance)
        };
        self.gcode.respond_info(
            &pct(
                "Retries: %d/%d %s: %0.6f tolerance: %0.6f",
                &[
                    pct_int(current_retry),
                    pct_int(max_retries),
                    PctValue::Str(VALUE_LABEL.to_string()),
                    PctValue::Float(error),
                    PctValue::Float(retry_tolerance),
                ],
            ),
            true,
        );
        if self.check_increase(error)? {
            return Err(KlippyError::Command(pct(
                "Retries aborting: %s is increasing. %s",
                &[
                    PctValue::Str(VALUE_LABEL.to_string()),
                    PctValue::Str(self.error_msg_extra.clone()),
                ],
            )));
        }
        if error <= retry_tolerance {
            return Ok("done".to_string());
        }
        let current_retry = {
            let mut state = self.state.borrow_mut();
            let current_retry = state
                .current_retry
                .ok_or_else(|| unstarted("current_retry"))?;
            let next = current_retry + 1;
            state.current_retry = Some(next);
            next
        };
        if current_retry > max_retries {
            return Err(KlippyError::Command("Too many retries".to_string()));
        }
        Ok("retry".to_string())
    }
}

// ---------------------------------------------------------------------------
// `ZAdjustHelper` (`klippy/extras/z_tilt.py:10-67`).
// ---------------------------------------------------------------------------

/// Levels the Z steppers by moving each one in turn.
pub struct ZAdjustHelper {
    printer: Printer,
    name: String,
    z_count: usize,
    z_steppers: GCell<Vec<Stepper>>,
}

impl ZAdjustHelper {
    /// `ZAdjustHelper.__init__` (`z_tilt.py:11`).
    pub fn new(config: &Config, z_count: usize) -> Arc<Self> {
        let printer = config.get_printer();
        let helper = Arc::new(ZAdjustHelper {
            printer: printer.clone(),
            name: config.get_name(),
            z_count,
            z_steppers: GCell::new(Vec::new()),
        });
        let connect_helper = Arc::clone(&helper);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params: &[EventArg]| {
                connect_helper.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        helper
    }

    /// `ZAdjustHelper.handle_connect` (`z_tilt.py:18`).
    pub fn handle_connect(&self) -> KResult<()> {
        let toolhead: ToolHead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let kinematics = toolhead.get_kinematics();
        let steppers: Vec<Stepper> = kinematics
            .get_steppers()
            .into_iter()
            .filter(|s| s.is_active_axis('z'))
            .collect();
        if steppers.len() != self.z_count {
            return Err(KlippyError::Config(pct(
                "%s z_positions needs exactly %d items",
                &[
                    PctValue::Str(self.name.clone()),
                    pct_int(steppers.len() as i64),
                ],
            )));
        }
        if steppers.len() < 2 {
            return Err(KlippyError::Config(pct(
                "%s requires multiple z steppers",
                &[PctValue::Str(self.name.clone())],
            )));
        }
        self.z_steppers.replace(steppers);
        Ok(())
    }

    /// `ZAdjustHelper.adjust_steppers` (`z_tilt.py:29`).
    pub fn adjust_steppers(&self, adjustments: &[f64], speed: f64) -> KResult<()> {
        let toolhead: ToolHead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let gcode: GCode = lookup_interface::<dyn GCodeApi>(
            &*self.printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let z_steppers = self.z_steppers.cloned();
        let mut curpos = toolhead.get_position();
        // Report on movements
        let mut stepstrs = Vec::with_capacity(z_steppers.len());
        for (s, a) in z_steppers.iter().zip(adjustments.iter()) {
            stepstrs.push(pct(
                "%s = %.6f",
                &[PctValue::Str(s.get_name(None)), PctValue::Float(*a)],
            ));
        }
        gcode.respond_info(
            &pct(
                "Making the following Z adjustments:\n%s",
                &[PctValue::Str(stepstrs.join("\n"))],
            ),
            true,
        );
        // Disable Z stepper movements
        toolhead.flush_step_generation()?;
        for s in &z_steppers {
            s.set_trapq(None);
        }
        // Move each z stepper (sorted from lowest to highest) until they match
        let mut positions: Vec<(f64, Stepper)> = adjustments
            .iter()
            .zip(z_steppers.iter())
            .map(|(a, s)| (-a, s.clone()))
            .collect();
        positions.sort_by(|a, b| {
            a.0.partial_cmp(&b.0)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        if positions.is_empty() {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        }
        let (first_stepper_offset, _first_stepper) = positions[0].clone();
        let z_low = curpos[2] - first_stepper_offset;
        for i in 0..positions.len() - 1 {
            let (_stepper_offset, stepper) = positions[i].clone();
            let (next_stepper_offset, _next_stepper) = positions[i + 1].clone();
            toolhead.flush_step_generation()?;
            stepper.set_trapq(Some(toolhead.get_trapq()));
            curpos[2] = z_low + next_stepper_offset;
            if let Err(e) = toolhead
                .r#move(curpos.clone(), speed)
                .and_then(|()| toolhead.set_position(curpos.clone(), ""))
            {
                log::exception("ZAdjustHelper adjust_steppers", e.message());
                toolhead.flush_step_generation()?;
                for s in &z_steppers {
                    s.set_trapq(Some(toolhead.get_trapq()));
                }
                return Err(e);
            }
        }
        // Z should now be level - do final cleanup
        let (_last_stepper_offset, last_stepper) = positions[positions.len() - 1].clone();
        toolhead.flush_step_generation()?;
        last_stepper.set_trapq(Some(toolhead.get_trapq()));
        curpos[2] += first_stepper_offset;
        toolhead.set_position(curpos, "")?;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// `ZTilt` (`klippy/extras/z_tilt.py:127-173`).
// ---------------------------------------------------------------------------

struct ZShared {
    z_positions: Vec<(f64, f64)>,
    status: GCell<Option<Arc<ZAdjustStatus>>>,
    retry: RetryHelper,
    z_helper: GCell<Option<Arc<ZAdjustHelper>>>,
    /// The probe helper, created after the finalize callback that
    /// captures the shared state.
    probe: GCell<Option<Arc<ProbePointsHelper>>>,
}

impl ZShared {
    fn status_installed(&self) -> Arc<ZAdjustStatus> {
        self.status
            .borrow()
            .as_ref()
            .map(Arc::clone)
            .expect("status installed at construction")
    }

    fn helper_installed(&self) -> Arc<ZAdjustHelper> {
        self.z_helper
            .borrow()
            .as_ref()
            .map(Arc::clone)
            .expect("z helper installed at construction")
    }

    fn probe_installed(&self) -> Arc<ProbePointsHelper> {
        self.probe
            .borrow()
            .as_ref()
            .map(Arc::clone)
            .expect("probe helper installed at construction")
    }
}

/// The `z_tilt` printer object.
pub struct ZTilt {
    shared: Arc<ZShared>,
}

impl ZTilt {
    /// `ZTilt.__init__` (`z_tilt.py:128`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let lists_opts = GetListsOpts {
            seps: vec![",".to_string(), "\n".to_string()],
            count: Some(2),
            parser: ListParser::Float,
            ..Default::default()
        };
        let raw = config.getlists("z_positions", lists_opts)?;
        let z_positions = parse_z_positions(&raw);
        let retry = RetryHelper::new(config, "")?;
        let z_count = z_positions.len();
        // Build the shared state with placeholders so the finalize
        // callback can capture it, then create the probe helper first
        // (Python order: `ProbePointsHelper` + `minimum_points` before
        // `ZAdjustStatus` and `ZAdjustHelper`, z_tilt.py:132-136), so
        // a probe config error leaves no registrations behind.
        let shared = Arc::new(ZShared {
            z_positions,
            status: GCell::new(None),
            retry,
            z_helper: GCell::new(None),
            probe: GCell::new(None),
        });
        let cb_shared = Arc::clone(&shared);
        let probe_helper = ProbePointsHelper::new(
            config,
            Box::new(move |positions: &[ProbeResult]| {
                Ok(PyValue::Str(probe_finalize(&cb_shared, positions)?))
            }),
            None,
        )?;
        probe_helper.minimum_points(2)?;
        *shared.probe.borrow_mut() = Some(probe_helper);
        let status = ZAdjustStatus::new(&printer);
        let z_helper = ZAdjustHelper::new(config, z_count);
        *shared.status.borrow_mut() = Some(status);
        *shared.z_helper.borrow_mut() = Some(z_helper);
        let ztilt = Arc::new(ZTilt { shared });
        // Register Z_TILT_ADJUST command (`z_tilt.py:138-140`).
        let gcode: GCode = lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let handler_ztilt = Arc::clone(&ztilt);
        let handler: GCodeCommandHandler = Box::new(move |gcmd| {
            handler_ztilt.cmd_z_tilt_adjust(gcmd)
        });
        let opts = RegisterCommandOpts {
            desc: Some("Adjust the Z tilt".to_string()),
            ..Default::default()
        };
        gcode.register_command("Z_TILT_ADJUST", Some(handler), opts)?;
        Ok(ztilt)
    }

    /// `ZTilt.cmd_Z_TILT_ADJUST` (`z_tilt.py:142`).
    pub fn cmd_z_tilt_adjust(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.shared.status_installed().reset();
        self.shared.retry.start(gcmd)?;
        let probe = self.shared.probe_installed();
        probe.start_probe(gcmd)
    }

    /// `ZTilt.probe_finalize` (`z_tilt.py:146`); also the callback
    /// handed to the probe helper.
    pub fn probe_finalize(&self, positions: &[ProbeResult]) -> KResult<String> {
        probe_finalize(&self.shared, positions)
    }

    /// The `retry_helper` attribute (`z_tilt.py:131`).
    pub fn retry_helper(&self) -> &RetryHelper {
        &self.shared.retry
    }

    /// `ZTilt.get_status` (`z_tilt.py:169`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.shared.status_installed().get_status(eventtime)
    }
}

impl PrinterObject for ZTilt {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `ZTilt.probe_finalize` body operating on the shared state, so the
/// probe callback can capture it independently of the object handle.
fn probe_finalize(shared: &ZShared, positions: &[ProbeResult]) -> KResult<String> {
    log::info(&pct(
        "Calculating bed tilt with: %s",
        &[PctValue::Str(probe_result_list_repr(positions))],
    ));
    // Find best fit for: a + b*bed_x + c*bed_y = bed_z
    let mut eqs: Matrix = Vec::with_capacity(positions.len());
    let mut ans: Matrix = Vec::with_capacity(positions.len());
    for pos in positions {
        eqs.push(vec![1.0, pos.bed_x, pos.bed_y]);
        ans.push(vec![pos.bed_z]);
    }
    // An empty probe list fails inside `solve_linear_equations` with the
    // reference's `IndexError` (mathutil indexes `eqs[0]`).
    let res = solve_linear_equations(&eqs, &ans, true)?.ok_or(KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: "'NoneType' object is not subscriptable".to_string(),
    })?;
    let z_adjust = res[0][0];
    let x_adjust = res[1][0];
    let y_adjust = res[2][0];
    // Apply results
    log::info(&pct(
        "Calculated bed tilt parameters: x_adjust=%.6f y_adjust=%.6f z_adjust=%.6f",
        &[
            PctValue::Float(x_adjust),
            PctValue::Float(y_adjust),
            PctValue::Float(z_adjust),
        ],
    ));
    let probe = shared.probe_installed();
    let speed = probe.get_lift_speed();
    let mut adjustments = Vec::with_capacity(shared.z_positions.len());
    for (x, y) in &shared.z_positions {
        adjustments.push(x * x_adjust + y * y_adjust + z_adjust);
    }
    shared
        .helper_installed()
        .adjust_steppers(&adjustments, speed)?;
    let retry_result = shared.retry.check_retry(
        &positions.iter().map(|p| p.bed_z).collect::<Vec<_>>(),
    )?;
    Ok(shared.status_installed().check_retry_result(&retry_result))
}

/// Python `repr(list)` of probe results (namedtuples `probe_result`).
fn probe_result_list_repr(positions: &[ProbeResult]) -> String {
    let mut items = Vec::with_capacity(positions.len());
    for p in positions {
        items.push(format!(
            "probe_result(bed_x={}, bed_y={}, bed_z={}, test_x={}, test_y={}, test_z={})",
            float_repr(p.bed_x),
            float_repr(p.bed_y),
            float_repr(p.bed_z),
            float_repr(p.test_x),
            float_repr(p.test_y),
            float_repr(p.test_z),
        ));
    }
    format!("[{}]", items.join(", "))
}

/// Extract `((x, y), ...)` from `getlists` output (already validated as
/// float pairs by the config layer). The config contract documents a tuple,
/// but accept a list as well so the value's container type cannot silently
/// drop every position.
fn parse_z_positions(raw: &PyValue) -> Vec<(f64, f64)> {
    let rows = match raw {
        PyValue::Tuple(rows) | PyValue::List(rows) => rows,
        _ => return Vec::new(),
    };
    let mut out = Vec::with_capacity(rows.len());
    for row in rows {
        let pair = match row {
            PyValue::Tuple(pair) | PyValue::List(pair) => pair,
            _ => continue,
        };
        if pair.len() != 2 {
            continue;
        }
        let coords = pair
            .iter()
            .map(|v| match v {
                PyValue::Float(f) => Some(*f),
                PyValue::Int(n) => n.0.to_string().parse::<f64>().ok(),
                _ => None,
            })
            .collect::<Option<Vec<f64>>>();
        if let Some(coords) = coords {
            out.push((coords[0], coords[1]));
        }
    }
    out
}

/// `load_config` (`z_tilt.py:172`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let ztilt = ZTilt::new(config)?;
    Ok(Some(ztilt as ObjectRef))
}

/// The module factory published for the klippy loader.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
