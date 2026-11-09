//! Port of the `axis_twist_compensation` module
//! (`klippy/extras/axis_twist_compensation.py`).
//!
//! Corrects the probed Z position for twist of the X and/or Y axis. The
//! compensation tables are applied to every probe result through the
//! `probe:update_results` event, and `AXIS_TWIST_COMPENSATION_CALIBRATE` walks
//! the operator through measuring them with the probe and a manual (nozzle)
//! probe at each calibration point.
//!
//! The probe object (`probe.PrinterProbe` / `probe_eddy_current`) is reached
//! through `crate::probe::PrinterProbeApi` and `crate::probe::run_single_probe`.

use std::sync::Arc;

use klippy_api::config::{Config, GetFloatOpts, GetListsOpts, ListParser, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCode, GCodeApi, GCodeCommandHandler, GetIntOpts as GcmdIntOpts, GetOpts as GcmdGetOpts,
    RegisterCommandOpts,
};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT, EV_PROBE_UPDATE_RESULTS,
};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, GCell, ObjectRef};
use pycompat::text::{float_str, float_true_div, percent_format, str_format, PctValue, PyError};
use pycompat::PyValue;

use crate::bed_mesh::{constrain, lerp};
use crate::manual_probe::{verify_no_manual_probe, FinalizeCallback, ManualProbeHelper, ProbeResult};
use crate::probe::{run_single_probe, PrinterProbeApi, ProbeResultList};

/// `DEFAULT_SAMPLE_COUNT` (`klippy/extras/axis_twist_compensation.py:11`).
pub const DEFAULT_SAMPLE_COUNT: i64 = 3;
/// `DEFAULT_SPEED` (`klippy/extras/axis_twist_compensation.py:12`).
pub const DEFAULT_SPEED: f64 = 50.;
/// `DEFAULT_HORIZONTAL_MOVE_Z` (`klippy/extras/axis_twist_compensation.py:13`).
pub const DEFAULT_HORIZONTAL_MOVE_Z: f64 = 5.;

/// `AxisTwistCompensation.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE_help`
/// (`klippy/extras/axis_twist_compensation.py:147`), the exact triple-quoted text.
const CMD_AXIS_TWIST_COMPENSATION_CALIBRATE_HELP: &str = "\n    Performs the x twist calibration wizard\n    Measure z probe offset at n points along the x axis,\n    and calculate x twist compensation\n    ";

/// Error text of the X axis config check
/// (`klippy/extras/axis_twist_compensation.py:177`).
const X_AXIS_REQUIRES: &str = "AXIS_TWIST_COMPENSATION for X axis requires\n                    calibrate_start_x, calibrate_end_x and calibrate_y\n                    to be defined\n                    ";
/// Error text of the Y axis config check
/// (`klippy/extras/axis_twist_compensation.py:204`).
const Y_AXIS_REQUIRES: &str = "AXIS_TWIST_COMPENSATION for Y axis requires\n                    calibrate_start_y, calibrate_end_y and calibrate_x\n                    to be defined\n                    ";

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn py_err(e: PyError) -> KlippyError {
    internal(e.kind, e.msg)
}

fn format_py(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(py_err)
}

/// Python truthiness of a float: `0.0` is false.
fn truthy(v: f64) -> bool {
    v != 0.0
}

/// `a - b` where either operand may be `None`; Python raises `TypeError`
/// naming the operand types.
fn sub_optional(a: Option<f64>, b: Option<f64>) -> KResult<f64> {
    match (a, b) {
        (Some(a), Some(b)) => Ok(a - b),
        _ => {
            let name = |v: Option<f64>| if v.is_some() { "float" } else { "NoneType" };
            Err(internal(
                "TypeError",
                format!("unsupported operand type(s) for -: '{}' and '{}'", name(a), name(b)),
            ))
        }
    }
}

/// `config.getfloat(option, default=default)` for an option with an explicit
/// default (`None` or a value).
fn config_float(
    config: &Config,
    option: &str,
    default: Option<f64>,
) -> KResult<Option<f64>> {
    config.getfloat(
        option,
        GetFloatOpts { default: Some(default), ..GetFloatOpts::default() },
    )
}

/// `config.getlists(option, default=[], parser=float)`.
fn config_float_lists(config: &Config, option: &str) -> KResult<Vec<f64>> {
    let value = config.getlists(
        option,
        GetListsOpts {
            default: Some(Some(PyValue::List(Vec::new()))),
            parser: ListParser::Float,
            ..GetListsOpts::default()
        },
    )?;
    match value {
        PyValue::List(items) | PyValue::Tuple(items) => items
            .iter()
            .map(|item| match item {
                PyValue::Float(f) => Ok(*f),
                other => Err(internal(
                    "TypeError",
                    format!("float expected, got {}", pycompat::value::type_name(other)),
                )),
            })
            .collect(),
        other => Err(internal(
            "TypeError",
            format!("'{}' object is not iterable", pycompat::value::type_name(&other)),
        )),
    }
}

/// Compensation tables and their ranges, rewritten by the calibration.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CompensationState {
    /// `z_compensations`
    pub z_compensations: Vec<f64>,
    /// `compensation_start_x`
    pub compensation_start_x: Option<f64>,
    /// `compensation_end_x`
    pub compensation_end_x: Option<f64>,
    /// `zy_compensations`
    pub zy_compensations: Vec<f64>,
    /// `compensation_start_y`
    pub compensation_start_y: Option<f64>,
    /// `compensation_end_y`
    pub compensation_end_y: Option<f64>,
}

/// `AxisTwistCompensation` (`klippy/extras/axis_twist_compensation.py:16`).
pub struct AxisTwistCompensation {
    printer: Printer,
    /// `horizontal_move_z`
    pub horizontal_move_z: f64,
    /// `speed`
    pub speed: f64,
    /// `calibrate_start_x`
    pub calibrate_start_x: Option<f64>,
    /// `calibrate_end_x`
    pub calibrate_end_x: Option<f64>,
    /// `calibrate_y`
    pub calibrate_y: Option<f64>,
    /// `calibrate_start_y`
    pub calibrate_start_y: Option<f64>,
    /// `calibrate_end_y`
    pub calibrate_end_y: Option<f64>,
    /// `calibrate_x`
    pub calibrate_x: Option<f64>,
    /// `z_compensations`, `zy_compensations` and their `compensation_*` ranges.
    pub state: GCell<CompensationState>,
}

impl AxisTwistCompensation {
    /// `AxisTwistCompensation.__init__(config)`
    /// (`klippy/extras/axis_twist_compensation.py:17`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        // get printer
        let printer = config.get_printer();
        lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");

        // get values from [axis_twist_compensation] section in printer .cfg
        let horizontal_move_z = config_float(
            config,
            "horizontal_move_z",
            Some(DEFAULT_HORIZONTAL_MOVE_Z),
        )?
        .unwrap_or(DEFAULT_HORIZONTAL_MOVE_Z);
        let speed = config_float(config, "speed", Some(DEFAULT_SPEED))?.unwrap_or(DEFAULT_SPEED);
        let calibrate_start_x = config_float(config, "calibrate_start_x", None)?;
        let calibrate_end_x = config_float(config, "calibrate_end_x", None)?;
        let calibrate_y = config_float(config, "calibrate_y", None)?;
        let z_compensations = config_float_lists(config, "z_compensations")?;
        let compensation_start_x = config_float(config, "compensation_start_x", None)?;
        let compensation_end_x = config_float(config, "compensation_end_x", None)?;

        let calibrate_start_y = config_float(config, "calibrate_start_y", None)?;
        let calibrate_end_y = config_float(config, "calibrate_end_y", None)?;
        let calibrate_x = config_float(config, "calibrate_x", None)?;
        let compensation_start_y = config_float(config, "compensation_start_y", None)?;
        let compensation_end_y = config_float(config, "compensation_end_y", None)?;
        let zy_compensations = config_float_lists(config, "zy_compensations")?;

        let this = Arc::new(AxisTwistCompensation {
            printer,
            horizontal_move_z,
            speed,
            calibrate_start_x,
            calibrate_end_x,
            calibrate_y,
            calibrate_start_y,
            calibrate_end_y,
            calibrate_x,
            state: GCell::new(CompensationState {
                z_compensations,
                compensation_start_x,
                compensation_end_x,
                zy_compensations,
                compensation_start_y,
                compensation_end_y,
            }),
        });

        // setup calibrater
        Calibrater::new(&this, config)?;
        // register events
        let handler = Arc::clone(&this);
        this.printer.register_event_handler(
            EV_PROBE_UPDATE_RESULTS,
            Box::new(move |args| {
                let Some(first) = args.first() else {
                    return Err(internal(
                        "TypeError",
                        "_update_z_compensation_value() missing 1 required positional argument: 'poslist'",
                    ));
                };
                let Some(poslist) = first.as_handle::<ProbeResultList>() else {
                    return Err(internal(
                        "TypeError",
                        "_update_z_compensation_value() expected the probe result list",
                    ));
                };
                handler.update_z_compensation_value(&poslist)?;
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// `AxisTwistCompensation._update_z_compensation_value(poslist)`
    /// (`klippy/extras/axis_twist_compensation.py:54`): replaces every entry of
    /// the list in place.
    pub fn update_z_compensation_value(&self, poslist: &ProbeResultList) -> KResult<()> {
        let state = self.state.borrow();
        let len = poslist.borrow().len();
        for i in 0..len {
            let pos = poslist.borrow()[i];
            let mut zo = 0.;
            if !state.z_compensations.is_empty() {
                zo += self.get_interpolated_z_compensation(
                    pos.test_x,
                    &state.z_compensations,
                    state.compensation_start_x,
                    state.compensation_end_x,
                )?;
            }

            if !state.zy_compensations.is_empty() {
                zo += self.get_interpolated_z_compensation(
                    pos.test_y,
                    &state.zy_compensations,
                    state.compensation_start_y,
                    state.compensation_end_y,
                )?;
            }

            poslist.borrow_mut()[i] = ProbeResult {
                bed_x: pos.bed_x,
                bed_y: pos.bed_y,
                bed_z: pos.bed_z + zo,
                test_x: pos.test_x,
                test_y: pos.test_y,
                test_z: pos.test_z,
            };
        }
        Ok(())
    }

    /// `AxisTwistCompensation._get_interpolated_z_compensation`
    /// (`klippy/extras/axis_twist_compensation.py:76`).
    fn get_interpolated_z_compensation(
        &self,
        coord: f64,
        z_compensations: &[f64],
        comp_start: Option<f64>,
        comp_end: Option<f64>,
    ) -> KResult<f64> {
        let sample_count = z_compensations.len();
        let span = sub_optional(comp_end, comp_start)?;
        let spacing = float_true_div(span, (sample_count as f64) - 1.0).map_err(py_err)?;
        // `comp_start` is a float here: the subtraction above raised otherwise.
        let start = comp_start.unwrap_or(0.0);
        let mut interpolate_t = float_true_div(coord - start, spacing).map_err(py_err)?;
        // `int(math.floor(interpolate_t))`
        let floored = kos::libm::floor(interpolate_t);
        if floored.is_nan() {
            return Err(internal("ValueError", "cannot convert float NaN to integer"));
        }
        if floored.is_infinite() {
            return Err(internal("OverflowError", "cannot convert float infinity to integer"));
        }
        // The constrained index is an integer in `0..=sample_count - 2`, so the
        // conversion below is exact.
        let constrained = constrain(floored, 0.0, (sample_count as f64) - 2.0);
        let interpolate_i = kos::cconv::f64_to_i64(constrained);
        interpolate_t -= interpolate_i as f64;
        let index = interpolate_i as usize;
        Ok(lerp(interpolate_t, z_compensations[index], z_compensations[index + 1]))
    }

    /// `AxisTwistCompensation.clear_compensations(axis=None)`
    /// (`klippy/extras/axis_twist_compensation.py:94`).
    pub fn clear_compensations(&self, axis: Option<&str>) {
        let mut state = self.state.borrow_mut();
        match axis {
            None => {
                state.z_compensations = Vec::new();
                state.zy_compensations = Vec::new();
            }
            Some("X") => state.z_compensations = Vec::new(),
            Some("Y") => state.zy_compensations = Vec::new(),
            Some(_) => {}
        }
    }
}

klippy_api::printer_object!(AxisTwistCompensation);

/// `Calibrater` (`klippy/extras/axis_twist_compensation.py:103`).
pub struct Calibrater {
    compensation: Arc<AxisTwistCompensation>,
    printer: Printer,
    gcode: GCode,
    probe: GCell<Option<Arc<dyn PrinterProbeApi>>>,
    lift_speed: GCell<Option<f64>>,
    speed: f64,
    horizontal_move_z: f64,
    x_start_point: (Option<f64>, Option<f64>),
    x_end_point: (Option<f64>, Option<f64>),
    y_start_point: (Option<f64>, Option<f64>),
    y_end_point: (Option<f64>, Option<f64>),
    results: GCell<Vec<f64>>,
    current_point_index: GCell<usize>,
    gcmd: GCell<Option<GCmd>>,
    configname: String,
    current_axis: GCell<String>,
    current_measured_z: GCell<f64>,
}

impl Calibrater {
    /// `Calibrater.__init__(compensation, config)`
    /// (`klippy/extras/axis_twist_compensation.py:104`).
    pub fn new(compensation: &Arc<AxisTwistCompensation>, config: &Config) -> KResult<Arc<Self>> {
        // setup self attributes
        let printer = Arc::clone(&compensation.printer);
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let this = Arc::new(Calibrater {
            compensation: Arc::clone(compensation),
            printer,
            gcode,
            // probe settings are set to none, until they are available
            probe: GCell::new(None),
            lift_speed: GCell::new(None),
            speed: compensation.speed,
            horizontal_move_z: compensation.horizontal_move_z,
            x_start_point: (compensation.calibrate_start_x, compensation.calibrate_y),
            x_end_point: (compensation.calibrate_end_x, compensation.calibrate_y),
            y_start_point: (compensation.calibrate_x, compensation.calibrate_start_y),
            y_end_point: (compensation.calibrate_x, compensation.calibrate_end_y),
            results: GCell::new(Vec::new()),
            current_point_index: GCell::new(0),
            gcmd: GCell::new(None),
            configname: config.get_name(),
            current_axis: GCell::new(String::new()),
            current_measured_z: GCell::new(0.0),
        });
        let on_connect = Arc::clone(&this);
        this.printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_args| {
                on_connect.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        // register gcode handlers
        this.register_gcode_handlers()?;
        Ok(this)
    }

    /// `Calibrater._handle_connect` (`klippy/extras/axis_twist_compensation.py:132`).
    fn handle_connect(&self) -> KResult<()> {
        let probe = lookup_interface::<dyn PrinterProbeApi>(
            &*self.printer,
            "probe",
            LookupDefault::ReturnNone,
        )?;
        *self.probe.borrow_mut() = probe.clone();
        let Some(probe) = probe else {
            return Err(KlippyError::config(
                "AXIS_TWIST_COMPENSATION requires [probe] to be defined",
            ));
        };
        let params = probe.get_probe_params(None)?;
        *self.lift_speed.borrow_mut() = Some(params.lift_speed);
        Ok(())
    }

    /// `Calibrater._register_gcode_handlers`
    /// (`klippy/extras/axis_twist_compensation.py:139`).
    fn register_gcode_handlers(self: &Arc<Self>) -> KResult<()> {
        let this = Arc::clone(self);
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd| this.cmd_axis_twist_compensation_calibrate(gcmd));
        self.gcode.register_command(
            "AXIS_TWIST_COMPENSATION_CALIBRATE",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_AXIS_TWIST_COMPENSATION_CALIBRATE_HELP.to_string()),
            },
        )?;
        Ok(())
    }

    /// `self.probe` as a call target: `None` before `klippy:connect` is
    /// Python's `AttributeError` on `NoneType`.
    fn probe_for(&self, attribute: &str) -> KResult<Arc<dyn PrinterProbeApi>> {
        self.probe.cloned().ok_or_else(|| {
            internal(
                "AttributeError",
                format!("'NoneType' object has no attribute '{attribute}'"),
            )
        })
    }

    /// `self.gcmd` as a call target (`None` until the command ran).
    fn gcmd(&self) -> KResult<GCmd> {
        self.gcmd.cloned().ok_or_else(|| {
            internal("AttributeError", "'NoneType' object has no attribute 'respond_info'")
        })
    }

    /// `Calibrater.cmd_AXIS_TWIST_COMPENSATION_CALIBRATE(gcmd)`
    /// (`klippy/extras/axis_twist_compensation.py:153`).
    pub fn cmd_axis_twist_compensation_calibrate(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        *self.gcmd.borrow_mut() = Some(Arc::clone(gcmd));
        let (probe_x_offset, probe_y_offset, _) =
            self.probe_for("get_offsets")?.get_offsets(Some(gcmd))?;
        let sample_count = gcmd
            .get_int(
                "SAMPLE_COUNT",
                GcmdIntOpts { default: Some(Some(DEFAULT_SAMPLE_COUNT)), ..GcmdIntOpts::default() },
            )?
            .unwrap_or(DEFAULT_SAMPLE_COUNT);
        let axis = match gcmd.get(
            "AXIS",
            GcmdGetOpts { default: Some(PyValue::Str("X".to_string())), ..GcmdGetOpts::default() },
        )? {
            PyValue::Str(axis) => axis,
            other => other.str(),
        };

        // check for valid sample_count
        if sample_count < 2 {
            return Err(KlippyError::command("SAMPLE_COUNT to probe must be at least 2"));
        }

        // calculate the points to put the probe at, returned as a list of tuples
        let mut nozzle_points: Vec<(f64, f64)> = Vec::new();
        let interval_dist;

        if axis == "X" {
            self.compensation.clear_compensations(Some("X"));

            let (start_x, end_x, start_y) =
                match (self.x_start_point.0, self.x_end_point.0, self.x_start_point.1) {
                    (Some(a), Some(b), Some(c)) if truthy(a) && truthy(b) && truthy(c) => (a, b, c),
                    _ => return Err(KlippyError::command(X_AXIS_REQUIRES)),
                };

            let x_axis_range = end_x - start_x;
            interval_dist = x_axis_range / ((sample_count - 1) as f64);

            for i in 0..sample_count {
                let x = start_x + (i as f64) * interval_dist;
                let y = start_y;
                nozzle_points.push((x, y));
            }
        } else if axis == "Y" {
            self.compensation.clear_compensations(Some("Y"));

            let (start_x, _, start_y) =
                match (self.y_start_point.0, self.y_end_point.0, self.y_start_point.1) {
                    (Some(a), Some(b), Some(c)) if truthy(a) && truthy(b) && truthy(c) => (a, b, c),
                    _ => return Err(KlippyError::command(Y_AXIS_REQUIRES)),
                };

            // `calibrate_end_y` is not part of the check above, so it may be
            // `None` here and the subtraction raises like Python's.
            let y_axis_range = sub_optional(self.y_end_point.1, Some(start_y))?;
            interval_dist = y_axis_range / ((sample_count - 1) as f64);

            for i in 0..sample_count {
                let x = start_x;
                let y = start_y + (i as f64) * interval_dist;
                nozzle_points.push((x, y));
            }
        } else {
            return Err(KlippyError::command(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Invalid axis.",
            ));
        }

        let probe_points =
            self.calculate_probe_points(&nozzle_points, probe_x_offset, probe_y_offset);

        // verify no other manual probe is in progress
        verify_no_manual_probe(&self.printer)?;

        // begin calibration
        self.current_point_index.set(0);
        *self.results.borrow_mut() = Vec::new();
        *self.current_axis.borrow_mut() = axis;
        self.calibration(&Arc::new(probe_points), &Arc::new(nozzle_points), interval_dist)
    }

    /// `Calibrater._calculate_probe_points`
    /// (`klippy/extras/axis_twist_compensation.py:238`).
    fn calculate_probe_points(
        &self,
        nozzle_points: &[(f64, f64)],
        probe_x_offset: f64,
        probe_y_offset: f64,
    ) -> Vec<(f64, f64)> {
        // calculate the points to put the nozzle at
        // returned as a list of tuples
        nozzle_points
            .iter()
            .map(|point| (point.0 - probe_x_offset, point.1 - probe_y_offset))
            .collect()
    }

    /// `Calibrater._move_helper(target_coordinates, override_speed=None)`
    /// (`klippy/extras/axis_twist_compensation.py:249`).
    fn move_helper(
        &self,
        target_coordinates: &[Option<f64>],
        override_speed: Option<f64>,
    ) -> KResult<()> {
        // pad target coordinates
        let target: Vec<Option<f64>> = if target_coordinates.len() == 2 {
            vec![target_coordinates[0], target_coordinates[1], None]
        } else {
            target_coordinates.to_vec()
        };
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let speed = if target[2].is_none() { Some(self.speed) } else { self.lift_speed.get() };
        let speed = if override_speed.is_some() { override_speed } else { speed };
        let Some(speed) = speed else {
            // `lift_speed` stays `None` until `klippy:connect`; `Move.__init__`
            // then fails in `min(speed, toolhead.max_velocity)`.
            return Err(internal(
                "TypeError",
                "'<' not supported between instances of 'float' and 'NoneType'",
            ));
        };
        toolhead.manual_move(target, speed)
    }

    /// `Calibrater._calibration(probe_points, nozzle_points, interval)`
    /// (`klippy/extras/axis_twist_compensation.py:259`).
    fn calibration(
        self: &Arc<Self>,
        probe_points: &Arc<Vec<(f64, f64)>>,
        nozzle_points: &Arc<Vec<(f64, f64)>>,
        interval: f64,
    ) -> KResult<()> {
        // begin the calibration process
        let gcmd = self.gcmd()?;
        let index = self.current_point_index.get();
        gcmd.respond_info(
            &format!(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Probing point {} of {}",
                index + 1,
                probe_points.len()
            ),
            true,
        );

        // horizontal_move_z (to prevent probe trigger or hitting bed)
        self.move_helper(&[None, None, Some(self.horizontal_move_z)], None)?;

        // move to point to probe
        self.move_helper(&[Some(probe_points[index].0), Some(probe_points[index].1), None], None)?;

        // probe the point
        let probe = self.probe_for("start_probe_session")?;
        let pos = run_single_probe(&probe, &gcmd)?;
        self.current_measured_z.set(pos.bed_z);

        // horizontal_move_z (to prevent probe trigger or hitting bed)
        self.move_helper(&[None, None, Some(self.horizontal_move_z)], None)?;

        // move the nozzle over the probe point
        self.move_helper(&[Some(nozzle_points[index].0), Some(nozzle_points[index].1)], None)?;

        // start the manual (nozzle) probe
        ManualProbeHelper::new(
            &self.printer,
            &gcmd,
            self.manual_probe_callback_factory(probe_points, nozzle_points, interval),
        )?;
        Ok(())
    }

    /// `Calibrater._manual_probe_callback_factory`
    /// (`klippy/extras/axis_twist_compensation.py:289`).
    fn manual_probe_callback_factory(
        self: &Arc<Self>,
        probe_points: &Arc<Vec<(f64, f64)>>,
        nozzle_points: &Arc<Vec<(f64, f64)>>,
        interval: f64,
    ) -> FinalizeCallback {
        // returns a callback function for the manual probe
        let is_end = self.current_point_index.get() as i64 == probe_points.len() as i64 - 1;
        let this = Arc::clone(self);
        let probe_points = Arc::clone(probe_points);
        let nozzle_points = Arc::clone(nozzle_points);
        Box::new(move |mpresult| {
            let Some(mpresult) = mpresult else {
                // probe was cancelled
                this.gcmd()?.respond_info(
                    "AXIS_TWIST_COMPENSATION_CALIBRATE: Probe cancelled, calibration aborted",
                    true,
                );
                return Ok(());
            };
            let z_offset = this.current_measured_z.get() - mpresult.bed_z;
            this.results.borrow_mut().push(z_offset);
            if is_end {
                // end of calibration
                this.finalize_calibration()
            } else {
                // move to next point
                this.current_point_index.set(this.current_point_index.get() + 1);
                this.calibration(&probe_points, &nozzle_points, interval)
            }
        })
    }

    /// `Calibrater._finalize_calibration`
    /// (`klippy/extras/axis_twist_compensation.py:312`).
    fn finalize_calibration(&self) -> KResult<()> {
        // finalize the calibration process
        // calculate average of results
        let measured = self.results.cloned();
        let avg = pycompat::text::py_sum_floats(&measured) / (measured.len() as f64);
        // subtract average from each result
        // so that they are independent of z_offset
        let results: Vec<f64> = measured.iter().map(|x| avg - x).collect();
        *self.results.borrow_mut() = results.clone();
        // save the config
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let mut formatted = Vec::with_capacity(results.len());
        for x in &results {
            formatted.push(str_format("{:.6f}", &[PctValue::Float(*x)], &[]).map_err(py_err)?);
        }
        let values_as_str = formatted.join(", ");

        let axis = self.current_axis.cloned();
        if axis == "X" {
            let start = self.x_start_point.0;
            let end = self.x_end_point.0;
            configfile.set(&self.configname, "z_compensations", &values_as_str)?;
            configfile.set(&self.configname, "compensation_start_x", &optional_str(start))?;
            configfile.set(&self.configname, "compensation_end_x", &optional_str(end))?;

            let mut state = self.compensation.state.borrow_mut();
            state.z_compensations = results.clone();
            state.compensation_start_x = start;
            state.compensation_end_x = end;
        } else if axis == "Y" {
            let start = self.y_start_point.1;
            let end = self.y_end_point.1;
            configfile.set(&self.configname, "zy_compensations", &values_as_str)?;
            configfile.set(&self.configname, "compensation_start_y", &optional_str(start))?;
            configfile.set(&self.configname, "compensation_end_y", &optional_str(end))?;

            let mut state = self.compensation.state.borrow_mut();
            state.zy_compensations = results.clone();
            state.compensation_start_y = start;
            state.compensation_end_y = end;
        }

        self.gcode.respond_info(
            concat!(
                "AXIS_TWIST_COMPENSATION state has been saved for the current session.  ",
                "The SAVE_CONFIG command will update the printer config file and restart the printer."
            ),
            true,
        );
        // output result
        let offsets = PyValue::List(results.iter().map(|x| PyValue::Float(*x)).collect()).str();
        self.gcmd()?.respond_info(
            &format_py(
                "AXIS_TWIST_COMPENSATION_CALIBRATE: Calibration complete, offsets: %s, mean z_offset: %f",
                &[PctValue::Str(offsets), PctValue::Float(avg)],
            )?,
            true,
        );
        Ok(())
    }
}

/// `str(value)` of an `Optional[float]` as `configfile.set` stores it.
fn optional_str(value: Option<f64>) -> String {
    match value {
        Some(f) => float_str(f),
        None => "None".to_string(),
    }
}

/// `load_config(config)` (`klippy/extras/axis_twist_compensation.py:360`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(AxisTwistCompensation::new(config)? as ObjectRef))
}

/// Module entry point for the `axis_twist_compensation` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
