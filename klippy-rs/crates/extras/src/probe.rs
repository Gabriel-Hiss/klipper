//! Port of the `probe` module (`klippy/extras/probe.py`).
//!
//! Z-probe support: the probing G-code commands (`ProbeCommandHelper`), multi-sample
//! averaging (`SampleAveragingHelper`), probing by homing to an endstop
//! (`DescendToEndstopHelper`, `ProbeEndstopWrapper`), the `probe:z_virtual_endstop`
//! pseudo-pin (`HomingViaProbeHelper`) and the point-by-point probing helper used by the
//! leveling modules (`ProbePointsHelper`).
//!
//! The Python duck types other modules rely on are traits owned by this module:
//! [`PrinterProbeApi`] (the `probe` printer object), [`ProbeSessionApi`] (the object
//! `start_probe_session` returns), [`ProbeParameterHelperApi`] and [`ProbeOffsetsHelperApi`].
//! `get_status` of the probe objects is the `PrinterObject::get_status` of the registered
//! object.

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListsOpts, GetStrOpts,
    ListParser, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCode, GCodeApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts as GcmdFloatOpts,
    GetIntOpts as GcmdIntOpts, GetOpts as GcmdGetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    get_printer_mcu, Chip, ChipApi, Endstop, EndstopApi, Mcu, Pin, PinParams, PinsApi,
};
use klippy_api::printer::{
    EventCallback, LookupDefault, ModuleFactory, Printer, EV_GCODE_COMMAND_ERROR,
    EV_KLIPPY_MCU_IDENTIFY, EV_PROBE_UPDATE_RESULTS,
};
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::{PrinterHomingApi, ToolHeadApi};
use klippy_api::{
    load_interface, lookup_interface, EventArg, GCell, ObjectRef, PrinterObject,
};
use pycompat::text::{float_pow, percent_format};
use pycompat::{PctValue, PyInt, PyValue};
use reactor::Completion;
use std::sync::{Arc, Weak};

use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use crate::manual_probe::{
    create_probe_result, lookup_z_endstop_config, verify_no_manual_probe, FinalizeCallback,
    ManualProbeHelper, ProbeResult,
};

/// `HINT_TIMEOUT` (`klippy/extras/probe.py:10`).
pub const HINT_TIMEOUT: &str = "\nIf the probe did not move far enough to trigger, then\nconsider reducing the Z axis minimum position so the probe\ncan travel further (the Z minimum position can be negative).\n";

/// Payload of the `probe:update_results` event (`klippy/extras/probe.py:365`): the sender
/// sends one `EventArg::handle(Arc<ProbeResultList>)` and reads entry 0 back after the
/// event; handlers replace entries in place.
pub type ProbeResultList = GCell<Vec<ProbeResult>>;

/// Query callback of a probe (`mcu_probe.query_endstop`): `cb(print_time)`.
pub type QueryEndstopCb = Arc<dyn Fn(f64) -> KResult<i64> + Send + Sync>;

/// The `start_session_cb` handed to [`SampleAveragingHelper`]: `cb(gcmd)` returns the
/// hardware probe session.
pub type StartSessionCb = Box<dyn Fn(&GCmd) -> KResult<Arc<dyn ProbeSessionApi>> + Send + Sync>;

/// The `add_stepper_cb` handed to [`LookupZSteppers`] (`mcu_probe.add_stepper`).
pub type AddStepperCb = Box<dyn FnMut(Stepper) -> KResult<()> + Send>;

/// The `finalize_callback` of [`ProbePointsHelper`]: `cb(results)`. The helper treats the
/// string `"retry"` as "probe again" and any other value as done.
pub type ProbeFinalizeCallback = Box<dyn Fn(&[ProbeResult]) -> KResult<PyValue> + Send + Sync>;

/// The dict `get_probe_params` returns (`klippy/extras/probe.py:309`).
#[derive(Clone, Debug, PartialEq)]
pub struct ProbeParams {
    /// `probe_speed`
    pub probe_speed: f64,
    /// `lift_speed`
    pub lift_speed: f64,
    /// `samples`
    pub samples: i64,
    /// `sample_retract_dist`
    pub sample_retract_dist: f64,
    /// `samples_tolerance`
    pub samples_tolerance: f64,
    /// `samples_tolerance_retries`
    pub samples_tolerance_retries: i64,
    /// `samples_result`
    pub samples_result: String,
}

/// The object `start_probe_session` returns (`SampleAveragingHelper`,
/// `ProbeEndstopWrapper`, the BLTouch and eddy probe sessions).
pub trait ProbeSessionApi: Send + Sync + 'static {
    /// `run_probe(gcmd)`. Pauses the reactor.
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()>;
    /// `pull_probed_results()`
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>>;
    /// `end_probe_session()`. Pauses the reactor.
    fn end_probe_session(&self) -> KResult<()>;
}

/// The `probe` printer object (`PrinterProbe` and the probe classes of `bltouch`,
/// `smart_effector`, `load_cell_probe`, `probe_eddy_current`). Its `get_status` is the
/// object's `PrinterObject::get_status`.
pub trait PrinterProbeApi: Send + Sync + 'static {
    /// `get_probe_params(gcmd=None)`
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams>;
    /// `get_offsets(gcmd=None)`
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)>;
    /// `start_probe_session(gcmd)`
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>>;
}

/// Duck type of the `param_helper` argument of [`DescendToEndstopHelper`] and
/// [`SampleAveragingHelper`].
pub trait ProbeParameterHelperApi: Send + Sync + 'static {
    /// `get_probe_params(gcmd=None)`
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams>;
}

/// Duck type of the `probe_offsets` argument of [`DescendToEndstopHelper`].
pub trait ProbeOffsetsHelperApi: Send + Sync + 'static {
    /// `get_offsets(gcmd=None)`
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> (f64, f64, f64);
}

// ---------------------------------------------------------------------------
// Small helpers.
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message: message.to_string(),
    }
}

fn index_error() -> KlippyError {
    internal("IndexError", "list index out of range")
}

/// `pins.error` (`klippy/pins.py:8`).
fn pins_error(message: &str) -> KlippyError {
    KlippyError::Pins(message.to_string())
}

fn fmt(format: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(format, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn count(n: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(n))
}

/// A getter whose default is always supplied cannot return `None`.
fn defaulted<T>(value: Option<T>) -> T {
    value.expect("getter with a default always returns a value")
}

fn lookup<T>(printer: &Printer, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    Ok(lookup_interface::<T>(&**printer, name, LookupDefault::Raise)?
        .expect("Raise never yields None"))
}

fn config_float(
    config: &Config,
    option: &str,
    default: Option<Option<f64>>,
    minval: Option<f64>,
    above: Option<f64>,
    note_valid: bool,
) -> KResult<Option<f64>> {
    config.getfloat(
        option,
        GetFloatOpts {
            default,
            minval,
            above,
            note_valid,
            ..GetFloatOpts::default()
        },
    )
}

fn gcmd_float(
    gcmd: &GCmd,
    name: &str,
    default: f64,
    minval: Option<f64>,
    above: Option<f64>,
) -> KResult<f64> {
    Ok(defaulted(gcmd.get_float(
        name,
        GcmdFloatOpts {
            default: Some(Some(default)),
            minval,
            above,
            ..GcmdFloatOpts::default()
        },
    )?))
}

fn gcmd_int(gcmd: &GCmd, name: &str, default: i64, minval: Option<i64>) -> KResult<i64> {
    Ok(defaulted(gcmd.get_int(
        name,
        GcmdIntOpts {
            default: Some(Some(default)),
            minval,
            maxval: None,
        },
    )?))
}

fn gcmd_str(gcmd: &GCmd, name: &str, default: &str) -> KResult<String> {
    let value = gcmd.get(
        name,
        GcmdGetOpts {
            default: Some(PyValue::Str(default.to_string())),
            ..GcmdGetOpts::default()
        },
    )?;
    Ok(match value {
        PyValue::Str(s) => s,
        other => other.str(),
    })
}

fn dict_get<'a>(value: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    if let PyValue::Dict(map) = value {
        for (k, v) in map {
            if let PyValue::Str(name) = k {
                if name == key {
                    return Ok(v);
                }
            }
        }
    }
    Err(internal("KeyError", &pycompat::text::str_repr(key)))
}

/// `[0.0, 0.0, 0.0, 0]`-padded `gcode.Coord((x, y, z))` (`klippy/gcode.py:12`): the fourth
/// entry is the int 0.
fn coord_value(x: f64, y: f64, z: f64) -> PyValue {
    PyValue::Tuple(vec![
        PyValue::Float(x),
        PyValue::Float(y),
        PyValue::Float(z),
        PyValue::Int(PyInt::from_i64(0)),
    ])
}

fn as_move(coord: &[f64]) -> Vec<Option<f64>> {
    coord.iter().map(|v| Some(*v)).collect()
}

/// `toolhead.manual_move(coord, speed)` for a fully specified coordinate.
fn manual_move(printer: &Printer, coord: &[f64], speed: f64) -> KResult<()> {
    let toolhead = lookup::<dyn ToolHeadApi>(printer, "toolhead")?;
    toolhead.manual_move(as_move(coord), speed)
}

fn first3(pos: &[f64]) -> &[f64] {
    &pos[..pos.len().min(3)]
}

fn register(
    gcode: &GCode,
    cmd: &str,
    func: GCodeCommandHandler,
    desc: &str,
) -> KResult<()> {
    gcode.register_command(
        cmd,
        Some(func),
        RegisterCommandOpts {
            when_not_ready: false,
            desc: Some(desc.to_string()),
        },
    )?;
    Ok(())
}

/// Left fold of `max()` over the values (`>` comparison like CPython).
fn py_max(values: &[f64]) -> KResult<f64> {
    let mut iter = values.iter();
    let mut best = *iter
        .next()
        .ok_or_else(|| internal("ValueError", "max() arg is an empty sequence"))?;
    for v in iter {
        if *v > best {
            best = *v;
        }
    }
    Ok(best)
}

/// Left fold of `min()` over the values (`<` comparison like CPython).
fn py_min(values: &[f64]) -> KResult<f64> {
    let mut iter = values.iter();
    let mut best = *iter
        .next()
        .ok_or_else(|| internal("ValueError", "min() arg is an empty sequence"))?;
    for v in iter {
        if *v < best {
            best = *v;
        }
    }
    Ok(best)
}

/// `calc_probe_z_average(positions, method='average')` (`klippy/extras/probe.py:17`).
pub fn calc_probe_z_average(positions: &[ProbeResult], method: &str) -> KResult<ProbeResult> {
    if method != "median" {
        // Use mean average.
        if positions.is_empty() {
            return Err(internal("ZeroDivisionError", "float division by zero"));
        }
        let inv_count = 1. / (positions.len() as f64);
        let mean = |field: fn(&ProbeResult) -> f64| -> f64 {
            let mut total = 0.0_f64;
            for pos in positions {
                total += field(pos);
            }
            total * inv_count
        };
        return Ok(ProbeResult {
            bed_x: mean(|p| p.bed_x),
            bed_y: mean(|p| p.bed_y),
            bed_z: mean(|p| p.bed_z),
            test_x: mean(|p| p.test_x),
            test_y: mean(|p| p.test_y),
            test_z: mean(|p| p.test_z),
        });
    }
    // Use median: `sorted(positions, key=lambda p: p.bed_z)` is a stable sort by `<`.
    let mut z_sorted: Vec<ProbeResult> = positions.to_vec();
    for i in 1..z_sorted.len() {
        let mut j = i;
        while j > 0 && z_sorted[j].bed_z < z_sorted[j - 1].bed_z {
            z_sorted.swap(j, j - 1);
            j -= 1;
        }
    }
    let middle = positions.len() / 2;
    if positions.len() & 1 == 1 {
        // odd number of samples
        return Ok(z_sorted[middle]);
    }
    // even number of samples
    if z_sorted.is_empty() {
        return calc_probe_z_average(&[], "average");
    }
    calc_probe_z_average(&z_sorted[middle - 1..middle + 1], "average")
}

/// `run_single_probe(probe, gcmd)` (`klippy/extras/probe.py:532`).
pub fn run_single_probe(probe: &Arc<dyn PrinterProbeApi>, gcmd: &GCmd) -> KResult<ProbeResult> {
    let probe_session = probe.start_probe_session(gcmd)?;
    probe_session.run_probe(gcmd)?;
    let pos = *probe_session
        .pull_probed_results()?
        .first()
        .ok_or_else(index_error)?;
    probe_session.end_probe_session()?;
    Ok(pos)
}

// ---------------------------------------------------------------------------
// ProbeCommandHelper
// ---------------------------------------------------------------------------

/// `probe_calibrate_info`: the initial probe result and the probe offsets at that time.
type CalibrateInfo = (ProbeResult, (f64, f64, f64));

/// Helper to implement common probing commands (`klippy/extras/probe.py:39`).
pub struct ProbeCommandHelper {
    printer: Printer,
    gcode: GCode,
    probe: Weak<dyn PrinterProbeApi>,
    query_endstop: Option<QueryEndstopCb>,
    name: String,
    last_state: GCell<PyValue>,
    last_probe_position: GCell<(f64, f64, f64)>,
    last_z_result: GCell<f64>,
    probe_calibrate_info: GCell<Option<CalibrateInfo>>,
}

impl ProbeCommandHelper {
    /// `ProbeCommandHelper.__init__(config, probe, query_endstop=None,
    /// can_set_z_offset=True)` (`klippy/extras/probe.py:40`). `probe` is the (still
    /// constructing) owner object, so it is held weakly.
    pub fn new(
        config: &Config,
        probe: Weak<dyn PrinterProbeApi>,
        query_endstop: Option<QueryEndstopCb>,
        can_set_z_offset: bool,
    ) -> KResult<Arc<ProbeCommandHelper>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let gcode = lookup::<dyn GCodeApi>(&printer, "gcode")?;
        let this = Arc::new(ProbeCommandHelper {
            printer,
            gcode,
            probe,
            query_endstop,
            name,
            last_state: GCell::new(PyValue::Bool(false)),
            last_probe_position: GCell::new((0., 0., 0.)),
            last_z_result: GCell::new(0.),
            probe_calibrate_info: GCell::new(None),
        });
        // QUERY_PROBE command
        register(
            &this.gcode,
            "QUERY_PROBE",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_query_probe(gcmd))
            },
            Self::cmd_query_probe_help(),
        )?;
        // PROBE command
        register(
            &this.gcode,
            "PROBE",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_probe(gcmd))
            },
            Self::cmd_probe_help(),
        )?;
        // PROBE_CALIBRATE command
        if can_set_z_offset {
            register(
                &this.gcode,
                "PROBE_CALIBRATE",
                {
                    let this = Arc::clone(&this);
                    Box::new(move |gcmd| ProbeCommandHelper::cmd_probe_calibrate(&this, gcmd))
                },
                Self::cmd_probe_calibrate_help(),
            )?;
        }
        // Other commands
        register(
            &this.gcode,
            "PROBE_ACCURACY",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_probe_accuracy(gcmd))
            },
            Self::cmd_probe_accuracy_help(),
        )?;
        if can_set_z_offset {
            register(
                &this.gcode,
                "Z_OFFSET_APPLY_PROBE",
                {
                    let this = Arc::clone(&this);
                    Box::new(move |gcmd| this.cmd_z_offset_apply_probe(gcmd))
                },
                Self::cmd_z_offset_apply_probe_help(),
            )?;
        }
        Ok(this)
    }

    /// `cmd_QUERY_PROBE_help` (`klippy/extras/probe.py:75`).
    pub fn cmd_query_probe_help() -> &'static str {
        "Return the status of the z-probe"
    }

    /// `cmd_PROBE_help` (`klippy/extras/probe.py:84`).
    pub fn cmd_probe_help() -> &'static str {
        "Probe Z-height at current XY position"
    }

    /// `cmd_PROBE_CALIBRATE_help` (`klippy/extras/probe.py:106`).
    pub fn cmd_probe_calibrate_help() -> &'static str {
        "Calibrate the probe's z_offset"
    }

    /// `cmd_PROBE_ACCURACY_help` (`klippy/extras/probe.py:124`).
    pub fn cmd_probe_accuracy_help() -> &'static str {
        "Probe Z-height accuracy at current XY position"
    }

    /// `cmd_Z_OFFSET_APPLY_PROBE_help` (`klippy/extras/probe.py:170`).
    pub fn cmd_z_offset_apply_probe_help() -> &'static str {
        "Adjust the probe's z_offset"
    }

    fn probe(&self) -> Arc<dyn PrinterProbeApi> {
        self.probe
            .upgrade()
            .expect("the probe object outlives its command helper")
    }

    /// `get_status(eventtime)` (`klippy/extras/probe.py:70`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let (x, y, z) = self.last_probe_position.get();
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("name".to_string()),
            PyValue::Str(self.name.clone()),
        );
        status.insert(
            PyValue::Str("last_query".to_string()),
            self.last_state.cloned(),
        );
        status.insert(
            PyValue::Str("last_probe_position".to_string()),
            coord_value(x, y, z),
        );
        status.insert(
            PyValue::Str("last_z_result".to_string()),
            PyValue::Float(self.last_z_result.get()),
        );
        PyValue::Dict(status)
    }

    /// `cmd_QUERY_PROBE(gcmd)` (`klippy/extras/probe.py:76`). Pauses the reactor.
    pub fn cmd_query_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let Some(query_endstop) = &self.query_endstop else {
            return Err(KlippyError::command("Probe does not support QUERY_PROBE"));
        };
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let print_time = toolhead.get_last_move_time()?;
        let res = query_endstop(print_time)?;
        self.last_state.replace(PyValue::Int(PyInt::from_i64(res)));
        let state = if res != 0 { "TRIGGERED" } else { "open" };
        gcmd.respond_info(&format!("probe: {state}"), true);
        Ok(())
    }

    /// `cmd_PROBE(gcmd)` (`klippy/extras/probe.py:85`). Pauses the reactor.
    pub fn cmd_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let probe = self.probe();
        let pos = run_single_probe(&probe, gcmd)?;
        let msg = fmt(
            "Result: at %.3f,%.3f estimate contact at z=%.6f",
            &[
                PctValue::Float(pos.bed_x),
                PctValue::Float(pos.bed_y),
                PctValue::Float(pos.bed_z),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        self.last_probe_position
            .set((pos.bed_x, pos.bed_y, pos.bed_z));
        let (_x_offset, _y_offset, z_offset) = probe.get_offsets(Some(gcmd))?;
        self.last_z_result.set(pos.bed_z + z_offset); // Deprecated
        Ok(())
    }

    /// `probe_calibrate_finalize(mpresult)` (`klippy/extras/probe.py:94`).
    pub fn probe_calibrate_finalize(&self, mpresult: Option<ProbeResult>) -> KResult<()> {
        let Some(mpresult) = mpresult else {
            return Ok(());
        };
        let Some((ppos, offsets)) = self.probe_calibrate_info.cloned() else {
            return Err(internal(
                "TypeError",
                "cannot unpack non-iterable NoneType object",
            ));
        };
        let z_offset = offsets.2 - mpresult.bed_z + ppos.bed_z;
        let msg = fmt(
            "%s: z_offset: %.3f\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
            &[PctValue::Str(self.name.clone()), PctValue::Float(z_offset)],
        )?;
        self.gcode.respond_info(&msg, true);
        let configfile = lookup::<dyn PrinterConfigApi>(&self.printer, "configfile")?;
        configfile.set(&self.name, "z_offset", &fmt("%.3f", &[PctValue::Float(z_offset)])?);
        Ok(())
    }

    /// `cmd_PROBE_CALIBRATE(gcmd)` (`klippy/extras/probe.py:107`). Pauses the reactor.
    pub fn cmd_probe_calibrate(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        verify_no_manual_probe(&this.printer)?;
        let probe = this.probe();
        let params = probe.get_probe_params(Some(gcmd))?;
        // Perform initial probe
        let ppos = run_single_probe(&probe, gcmd)?;
        // Move away from the bed
        let toolhead = lookup::<dyn ToolHeadApi>(&this.printer, "toolhead")?;
        let mut curpos = toolhead.get_position();
        curpos[2] += 5.;
        manual_move(&this.printer, &curpos, params.lift_speed)?;
        // Move the nozzle over the probe point
        curpos[0] = ppos.bed_x;
        curpos[1] = ppos.bed_y;
        manual_move(&this.printer, &curpos, params.probe_speed)?;
        // Start manual probe
        let offsets = probe.get_offsets(Some(gcmd))?;
        this.probe_calibrate_info.replace(Some((ppos, offsets)));
        let callback: FinalizeCallback = Box::new({
            let this = Arc::clone(this);
            move |mpresult| this.probe_calibrate_finalize(mpresult)
        });
        ManualProbeHelper::new(&this.printer, gcmd, callback)?;
        Ok(())
    }

    /// `cmd_PROBE_ACCURACY(gcmd)` (`klippy/extras/probe.py:125`). Pauses the reactor.
    pub fn cmd_probe_accuracy(&self, gcmd: &GCmd) -> KResult<()> {
        let probe = self.probe();
        let params = probe.get_probe_params(Some(gcmd))?;
        let sample_count = gcmd_int(gcmd, "SAMPLES", 10, Some(1))?;
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let start_pos = toolhead.get_position();
        let msg = fmt(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f (samples=%d retract=%.3f speed=%.1f lift_speed=%.1f)\n",
            &[
                PctValue::Float(start_pos[0]),
                PctValue::Float(start_pos[1]),
                PctValue::Float(start_pos[2]),
                count(sample_count),
                PctValue::Float(params.sample_retract_dist),
                PctValue::Float(params.probe_speed),
                PctValue::Float(params.lift_speed),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        // Create dummy gcmd with SAMPLES=1
        let mut fo_params = gcmd.get_command_parameters();
        fo_params.insert("SAMPLES".to_string(), "1".to_string());
        let gcode = lookup::<dyn GCodeApi>(&self.printer, "gcode")?;
        let fo_gcmd = gcode.create_gcode_command("", "", &fo_params);
        // Probe bed sample_count times
        let probe_session = probe.start_probe_session(&fo_gcmd)?;
        let mut probe_num = 0;
        while probe_num < sample_count {
            // Probe position
            probe_session.run_probe(&fo_gcmd)?;
            probe_num += 1;
            // Retract
            let lift_z = toolhead.get_position()[2] + params.sample_retract_dist;
            let liftpos = [start_pos[0], start_pos[1], lift_z];
            manual_move(&self.printer, &liftpos, params.lift_speed)?;
        }
        let positions = probe_session.pull_probed_results()?;
        probe_session.end_probe_session()?;
        // Calculate maximum, minimum and average values
        let z_values: Vec<f64> = positions.iter().map(|p| p.bed_z).collect();
        let max_value = py_max(&z_values)?;
        let min_value = py_min(&z_values)?;
        let range_value = max_value - min_value;
        let avg_value = calc_probe_z_average(&positions, "average")?.bed_z;
        let median = calc_probe_z_average(&positions, "median")?.bed_z;
        // calculate the standard deviation
        let mut deviation_sum = 0.0_f64;
        for position in &positions {
            deviation_sum += float_pow(position.bed_z - avg_value, 2.).map_err(py_error)?;
        }
        let sigma = float_pow(deviation_sum / (positions.len() as f64), 0.5).map_err(py_error)?;
        // Show information
        let msg = fmt(
            "probe accuracy results: maximum %.6f, minimum %.6f, range %.6f, average %.6f, median %.6f, standard deviation %.6f",
            &[
                PctValue::Float(max_value),
                PctValue::Float(min_value),
                PctValue::Float(range_value),
                PctValue::Float(avg_value),
                PctValue::Float(median),
                PctValue::Float(sigma),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    /// `cmd_Z_OFFSET_APPLY_PROBE(gcmd)` (`klippy/extras/probe.py:171`).
    pub fn cmd_z_offset_apply_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let gcode_move = lookup::<dyn GCodeMoveApi>(&self.printer, "gcode_move")?;
        let status = gcode_move.get_status(None);
        let offset = match dict_get(&status, "homing_origin")? {
            PyValue::Tuple(items) | PyValue::List(items) => match items.get(2) {
                Some(PyValue::Float(z)) => *z,
                Some(PyValue::Int(n)) => n.0.to_string().parse::<f64>().unwrap_or(0.),
                _ => return Err(index_error()),
            },
            _ => return Err(internal("AttributeError", "homing_origin has no attribute 'z'")),
        };
        if offset == 0. {
            gcmd.respond_info("Nothing to do: Z Offset is 0", true);
            return Ok(());
        }
        let z_offset = self.probe().get_offsets(Some(gcmd))?.2;
        let new_calibrate = z_offset - offset;
        let msg = fmt(
            "%s: z_offset: %.3f\nThe SAVE_CONFIG command will update the printer config file\nwith the above and restart the printer.",
            &[PctValue::Str(self.name.clone()), PctValue::Float(new_calibrate)],
        )?;
        gcmd.respond_info(&msg, true);
        let configfile = lookup::<dyn PrinterConfigApi>(&self.printer, "configfile")?;
        configfile.set(
            &self.name,
            "z_offset",
            &fmt("%.3f", &[PctValue::Float(new_calibrate)])?,
        );
        Ok(())
    }
}

fn py_error(e: pycompat::PyError) -> KlippyError {
    KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    }
}

// ---------------------------------------------------------------------------
// LookupMinimumZ, LookupZSteppers, HomingViaProbeHelper
// ---------------------------------------------------------------------------

/// `lookup_minimum_z(config)` (`klippy/extras/probe.py:188`).
pub fn lookup_minimum_z(config: &Config) -> KResult<f64> {
    if let Some(zconfig) = lookup_z_endstop_config(config)? {
        return Ok(defaulted(config_float(
            &zconfig,
            "position_min",
            Some(Some(0.)),
            None,
            None,
            false,
        )?));
    }
    let pconfig = config.getsection("printer");
    Ok(defaulted(config_float(
        &pconfig,
        "minimum_z_position",
        Some(Some(0.)),
        None,
        None,
        false,
    )?))
}

/// Helper to lookup all the Z axis steppers (`klippy/extras/probe.py:196`).
pub struct LookupZSteppers {
    printer: Printer,
}

impl LookupZSteppers {
    /// `LookupZSteppers.__init__(config, add_stepper_cb)`: registers the
    /// `klippy:mcu_identify` handler that hands every Z stepper to `add_stepper_cb`.
    pub fn new(config: &Config, mut add_stepper_cb: AddStepperCb) -> LookupZSteppers {
        let printer = config.get_printer();
        let handler_printer = Arc::clone(&printer);
        let handler: EventCallback = Box::new(move |_args| {
            // `_handle_mcu_identify`
            let toolhead = lookup::<dyn ToolHeadApi>(&handler_printer, "toolhead")?;
            let kin = toolhead.get_kinematics();
            for stepper in kin.get_steppers() {
                if stepper.is_active_axis('z') {
                    add_stepper_cb(stepper)?;
                }
            }
            Ok(PyValue::None)
        });
        printer.register_event_handler(EV_KLIPPY_MCU_IDENTIFY, handler);
        LookupZSteppers { printer }
    }

    /// The printer the handler is registered on.
    pub fn printer(&self) -> &Printer {
        &self.printer
    }
}

/// Support homing via probe using the `probe:z_virtual_endstop` pseudo-pin
/// (`klippy/extras/probe.py:209`).
pub struct HomingViaProbeHelper {
    printer: Printer,
    position_endstop: f64,
    query_endstop_cb: Option<QueryEndstopCb>,
    self_ref: Weak<HomingViaProbeHelper>,
}

const HOMING_VIA_PROBE_ERROR: &str = "Internal error. Can't home via HomingViaProbeHelper.";

impl HomingViaProbeHelper {
    /// `HomingViaProbeHelper.__init__(config, position_endstop, query_endstop_cb=None)`.
    pub fn new(
        config: &Config,
        position_endstop: f64,
        query_endstop_cb: Option<QueryEndstopCb>,
    ) -> KResult<Arc<HomingViaProbeHelper>> {
        let printer = config.get_printer();
        let this = Arc::new_cyclic(|weak| HomingViaProbeHelper {
            printer: Arc::clone(&printer),
            position_endstop,
            query_endstop_cb,
            self_ref: weak.clone(),
        });
        // Register z_virtual_endstop pin
        let pins = lookup::<dyn PinsApi>(&printer, "pins")?;
        pins.register_chip("probe", Arc::clone(&this) as Chip)?;
        Ok(this)
    }

    /// `get_position_endstop()`
    pub fn get_position_endstop(&self) -> f64 {
        self.position_endstop
    }
}

impl ChipApi for HomingViaProbeHelper {
    /// `setup_pin(pin_type, pin_params)` (`klippy/extras/probe.py:238`).
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        if pin_type != "endstop" || pin_params.pin != "z_virtual_endstop" {
            return Err(pins_error(
                "Probe virtual endstop only useful as endstop pin",
            ));
        }
        if pin_params.invert != 0 || pin_params.pullup != 0 {
            return Err(pins_error("Can not pullup/invert probe virtual endstop"));
        }
        let this = self
            .self_ref
            .upgrade()
            .expect("the registered chip is alive while it is called");
        Ok(Pin::Endstop(this as Endstop))
    }

    /// The pin parameters of this chip carry an MCU (`PrinterPins.parse_pin` asks every
    /// chip for it); the Python class has none and raises on `get_mcu`, which the
    /// contract cannot express, so the main MCU stands in.
    fn get_mcu(&self) -> Mcu {
        get_printer_mcu(&self.printer, "mcu").expect("the main mcu object exists")
    }
}

impl EndstopApi for HomingViaProbeHelper {
    /// `get_mcu()` raises `command_error` in Python; the contract returns no error.
    fn get_mcu(&self) -> Mcu {
        panic!("{HOMING_VIA_PROBE_ERROR}")
    }

    fn add_stepper(&self, _stepper: Stepper) {}

    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }

    /// `home_start(...)` raises `command_error` in Python; the contract returns no error.
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        panic!("{HOMING_VIA_PROBE_ERROR}")
    }

    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        Err(KlippyError::command(HOMING_VIA_PROBE_ERROR))
    }

    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        match &self.query_endstop_cb {
            None => Ok(0),
            Some(cb) => cb(print_time),
        }
    }

    fn get_position_endstop(&self) -> Option<f64> {
        Some(self.position_endstop)
    }
}

// ---------------------------------------------------------------------------
// DescendToEndstopHelper
// ---------------------------------------------------------------------------

/// Support for probes that trigger via `mcu_endstop` (`klippy/extras/probe.py:246`).
pub struct DescendToEndstopHelper {
    printer: Printer,
    mcu_probe: Endstop,
    probe_offsets: Arc<dyn ProbeOffsetsHelperApi>,
    param_helper: Arc<dyn ProbeParameterHelperApi>,
    always_check_movement: bool,
    z_min_position: f64,
    results: GCell<Vec<ProbeResult>>,
    _z_steppers: LookupZSteppers,
}

impl DescendToEndstopHelper {
    /// `DescendToEndstopHelper.__init__(config, mcu_probe, probe_offsets, param_helper,
    /// always_check_movement=False)`.
    pub fn new(
        config: &Config,
        mcu_probe: Endstop,
        probe_offsets: Arc<dyn ProbeOffsetsHelperApi>,
        param_helper: Arc<dyn ProbeParameterHelperApi>,
        always_check_movement: bool,
    ) -> KResult<DescendToEndstopHelper> {
        let printer = config.get_printer();
        let z_min_position = lookup_minimum_z(config)?;
        let z_steppers = LookupZSteppers::new(config, {
            let mcu_probe = Arc::clone(&mcu_probe);
            Box::new(move |stepper| {
                mcu_probe.add_stepper(stepper);
                Ok(())
            })
        });
        Ok(DescendToEndstopHelper {
            printer,
            mcu_probe,
            probe_offsets,
            param_helper,
            always_check_movement,
            z_min_position,
            results: GCell::new(Vec::new()),
            _z_steppers: z_steppers,
        })
    }

    /// `descend_until_trigger(gcmd)` (`klippy/extras/probe.py:257`). Pauses the reactor.
    pub fn descend_until_trigger(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let mut pos = toolhead.get_position();
        pos[2] = self.z_min_position;
        let speed = self.param_helper.get_probe_params(Some(gcmd))?.probe_speed;
        let phoming = lookup::<dyn PrinterHomingApi>(&self.printer, "homing")?;
        let check_movement =
            self.always_check_movement || !phoming.check_probe_first_home(gcmd)?;
        let ppos = phoming.probing_move(
            Arc::clone(&self.mcu_probe),
            as_move(&pos),
            speed,
            check_movement,
        )?;
        let (x_offset, y_offset, z_offset) = self.probe_offsets.get_offsets(None);
        let res = create_probe_result(&ppos, [x_offset, y_offset, z_offset]);
        self.results.borrow_mut().push(res);
        Ok(())
    }

    /// `pull_trigger_positions()` (`klippy/extras/probe.py:270`).
    pub fn pull_trigger_positions(&self) -> Vec<ProbeResult> {
        self.results.replace(Vec::new())
    }

    /// `clear_trigger_positions()` (`klippy/extras/probe.py:274`).
    pub fn clear_trigger_positions(&self) {
        self.pull_trigger_positions();
    }
}

// ---------------------------------------------------------------------------
// ProbeParameterHelper
// ---------------------------------------------------------------------------

/// Helper to read multi-sample parameters from config (`klippy/extras/probe.py:278`).
pub struct ProbeParameterHelper {
    dummy_gcode_cmd: GCmd,
    speed: f64,
    lift_speed: f64,
    sample_count: i64,
    sample_retract_dist: f64,
    samples_result: String,
    samples_tolerance: f64,
    samples_retries: i64,
}

impl ProbeParameterHelper {
    /// `ProbeParameterHelper.__init__(config)`.
    pub fn new(config: &Config) -> KResult<Arc<ProbeParameterHelper>> {
        let gcode = lookup::<dyn GCodeApi>(&config.get_printer(), "gcode")?;
        let dummy_gcode_cmd = gcode.create_gcode_command("", "", &IndexMap::new());
        // Configurable probing speeds
        let speed = defaulted(config_float(config, "speed", Some(Some(5.0)), None, Some(0.), true)?);
        let lift_speed = defaulted(config_float(
            config,
            "lift_speed",
            Some(Some(speed)),
            None,
            Some(0.),
            true,
        )?);
        // Multi-sample support (for improved accuracy)
        let sample_count = defaulted(config.getint(
            "samples",
            GetIntOpts {
                default: Some(Some(1)),
                minval: Some(1),
                ..GetIntOpts::default()
            },
        )?);
        let sample_retract_dist = defaulted(config_float(
            config,
            "sample_retract_dist",
            Some(Some(2.)),
            None,
            Some(0.),
            true,
        )?);
        let atypes = PyValue::List(vec![
            PyValue::Str("median".to_string()),
            PyValue::Str("average".to_string()),
        ]);
        let samples_result = match config.getchoice(
            "samples_result",
            &atypes,
            GetChoiceOpts {
                default: Some(Some(PyValue::Str("average".to_string()))),
                note_valid: true,
            },
        )? {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        let samples_tolerance = defaulted(config_float(
            config,
            "samples_tolerance",
            Some(Some(0.100)),
            Some(0.),
            None,
            true,
        )?);
        let samples_retries = defaulted(config.getint(
            "samples_tolerance_retries",
            GetIntOpts {
                default: Some(Some(0)),
                minval: Some(0),
                ..GetIntOpts::default()
            },
        )?);
        Ok(Arc::new(ProbeParameterHelper {
            dummy_gcode_cmd,
            speed,
            lift_speed,
            sample_count,
            sample_retract_dist,
            samples_result,
            samples_tolerance,
            samples_retries,
        }))
    }

    /// `get_probe_params(gcmd=None)` (`klippy/extras/probe.py:296`).
    pub fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        let gcmd = gcmd.unwrap_or(&self.dummy_gcode_cmd);
        let probe_speed = gcmd_float(gcmd, "PROBE_SPEED", self.speed, None, Some(0.))?;
        let lift_speed = gcmd_float(gcmd, "LIFT_SPEED", self.lift_speed, None, Some(0.))?;
        let samples = gcmd_int(gcmd, "SAMPLES", self.sample_count, Some(1))?;
        let sample_retract_dist = gcmd_float(
            gcmd,
            "SAMPLE_RETRACT_DIST",
            self.sample_retract_dist,
            None,
            Some(0.),
        )?;
        let samples_tolerance = gcmd_float(
            gcmd,
            "SAMPLES_TOLERANCE",
            self.samples_tolerance,
            Some(0.),
            None,
        )?;
        let samples_retries = gcmd_int(
            gcmd,
            "SAMPLES_TOLERANCE_RETRIES",
            self.samples_retries,
            Some(0),
        )?;
        let samples_result = gcmd_str(gcmd, "SAMPLES_RESULT", &self.samples_result)?;
        Ok(ProbeParams {
            probe_speed,
            lift_speed,
            samples,
            sample_retract_dist,
            samples_tolerance,
            samples_tolerance_retries: samples_retries,
            samples_result,
        })
    }
}

impl ProbeParameterHelperApi for ProbeParameterHelper {
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        ProbeParameterHelper::get_probe_params(self, gcmd)
    }
}

// ---------------------------------------------------------------------------
// SampleAveragingHelper
// ---------------------------------------------------------------------------

/// Helper to track multiple probe attempts in a single command
/// (`klippy/extras/probe.py:318`).
pub struct SampleAveragingHelper {
    printer: Printer,
    param_helper: Arc<dyn ProbeParameterHelperApi>,
    start_session_cb: StartSessionCb,
    // Session state
    hw_probe_session: GCell<Option<Arc<dyn ProbeSessionApi>>>,
    results: GCell<Vec<ProbeResult>>,
    self_ref: Weak<SampleAveragingHelper>,
}

impl SampleAveragingHelper {
    /// `SampleAveragingHelper.__init__(config, param_helper, start_session_cb)`.
    pub fn new(
        config: &Config,
        param_helper: Arc<dyn ProbeParameterHelperApi>,
        start_session_cb: StartSessionCb,
    ) -> Arc<SampleAveragingHelper> {
        let printer = config.get_printer();
        let this = Arc::new_cyclic(|weak| SampleAveragingHelper {
            printer: Arc::clone(&printer),
            param_helper,
            start_session_cb,
            hw_probe_session: GCell::new(None),
            results: GCell::new(Vec::new()),
            self_ref: weak.clone(),
        });
        // Register event handlers
        let handler: EventCallback = Box::new({
            let this = Arc::clone(&this);
            move |_args| {
                this.handle_command_error();
                Ok(PyValue::None)
            }
        });
        printer.register_event_handler(EV_GCODE_COMMAND_ERROR, handler);
        this
    }

    /// `_handle_command_error()` (`klippy/extras/probe.py:329`).
    fn handle_command_error(&self) {
        let active = self.hw_probe_session.borrow().is_some();
        if active {
            if let Err(e) = ProbeSessionApi::end_probe_session(self) {
                let text = format!("{}: {}", e.class_name(), e.message());
                klippy_api::log::exception("Multi-probe end", &text);
            }
        }
    }

    fn probe_state_error(&self) -> KlippyError {
        KlippyError::command("Internal probe error - start/end probe session mismatch")
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe.py:338`): returns this helper.
    pub fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<SampleAveragingHelper>> {
        if self.hw_probe_session.borrow().is_some() {
            return Err(self.probe_state_error());
        }
        let session = (self.start_session_cb)(gcmd)?;
        self.hw_probe_session.replace(Some(session));
        self.results.replace(Vec::new());
        Ok(self
            .self_ref
            .upgrade()
            .expect("the helper is alive while it is called"))
    }

    /// `_probe(gcmd)` (`klippy/extras/probe.py:351`). Pauses the reactor.
    fn probe(&self, gcmd: &GCmd) -> KResult<ProbeResult> {
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let curtime = self.printer.get_reactor().monotonic();
        let status = toolhead.get_status(curtime);
        match dict_get(&status, "homed_axes")? {
            PyValue::Str(homed) => {
                if !homed.contains('z') {
                    return Err(KlippyError::command("Must home before probe"));
                }
            }
            other => {
                return Err(internal(
                    "TypeError",
                    &format!(
                        "argument of type '{}' is not iterable",
                        pycompat::value::type_name(other)
                    ),
                ));
            }
        }
        let attempt = || -> KResult<ProbeResult> {
            let session = self.hw_probe_session.cloned().ok_or_else(|| {
                internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'run_probe'",
                )
            })?;
            session.run_probe(gcmd)?;
            let session = self.hw_probe_session.cloned().ok_or_else(|| {
                internal(
                    "AttributeError",
                    "'NoneType' object has no attribute 'pull_probed_results'",
                )
            })?;
            session
                .pull_probed_results()?
                .first()
                .copied()
                .ok_or_else(index_error)
        };
        let epos = match attempt() {
            Ok(epos) => epos,
            Err(KlippyError::Command(mut reason)) => {
                if reason.contains("Timeout during endstop homing") {
                    reason.push_str(HINT_TIMEOUT);
                }
                return Err(KlippyError::command(reason));
            }
            Err(e) => return Err(e),
        };
        // Allow axis_twist_compensation to update results
        let results: Arc<ProbeResultList> = Arc::new(GCell::new(vec![epos]));
        self.printer.send_event(
            EV_PROBE_UPDATE_RESULTS,
            &[EventArg::handle(Arc::clone(&results))],
        )?;
        let epos = *results.borrow().first().ok_or_else(index_error)?;
        // Report results
        if gcmd.get_command() != "G28" {
            let gcode = lookup::<dyn GCodeApi>(&self.printer, "gcode")?;
            let msg = fmt(
                "probe: at %.3f,%.3f bed will contact at z=%.6f",
                &[
                    PctValue::Float(epos.bed_x),
                    PctValue::Float(epos.bed_y),
                    PctValue::Float(epos.bed_z),
                ],
            )?;
            gcode.respond_info(&msg, true);
        }
        Ok(epos)
    }
}

impl ProbeSessionApi for SampleAveragingHelper {
    /// `run_probe(gcmd)` (`klippy/extras/probe.py:374`).
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        if self.hw_probe_session.borrow().is_none() {
            return Err(self.probe_state_error());
        }
        let params = self.param_helper.get_probe_params(Some(gcmd))?;
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let position = toolhead.get_position();
        let probexy = [position[0], position[1]];
        let mut retries = 0;
        let mut positions: Vec<ProbeResult> = Vec::new();
        let sample_count = params.samples;
        while (positions.len() as i64) < sample_count {
            // Probe position
            let pos = self.probe(gcmd)?;
            positions.push(pos);
            // Check samples tolerance
            let z_positions: Vec<f64> = positions.iter().map(|p| p.bed_z).collect();
            if py_max(&z_positions)? - py_min(&z_positions)? > params.samples_tolerance {
                if retries >= params.samples_tolerance_retries {
                    return Err(KlippyError::command("Probe samples exceed samples_tolerance"));
                }
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...", true);
                retries += 1;
                positions = Vec::new();
            }
            // Retract
            if (positions.len() as i64) < sample_count {
                let cur_z = toolhead.get_position()[2];
                toolhead.manual_move(
                    vec![
                        Some(probexy[0]),
                        Some(probexy[1]),
                        Some(cur_z + params.sample_retract_dist),
                    ],
                    params.lift_speed,
                )?;
            }
        }
        // Calculate result
        let epos = calc_probe_z_average(&positions, &params.samples_result)?;
        self.results.borrow_mut().push(epos);
        Ok(())
    }

    /// `pull_probed_results()` (`klippy/extras/probe.py:404`).
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.results.replace(Vec::new()))
    }

    /// `end_probe_session()` (`klippy/extras/probe.py:344`).
    fn end_probe_session(&self) -> KResult<()> {
        let Some(hw_probe_session) = self.hw_probe_session.cloned() else {
            return Err(self.probe_state_error());
        };
        self.results.replace(Vec::new());
        self.hw_probe_session.replace(None);
        hw_probe_session.end_probe_session()
    }
}

// ---------------------------------------------------------------------------
// ProbeOffsetsHelper
// ---------------------------------------------------------------------------

/// Helper to read the xyz probe offsets from the config (`klippy/extras/probe.py:410`).
pub struct ProbeOffsetsHelper {
    x_offset: f64,
    y_offset: f64,
    z_offset: f64,
}

impl ProbeOffsetsHelper {
    /// `ProbeOffsetsHelper.__init__(config)`.
    pub fn new(config: &Config) -> KResult<Arc<ProbeOffsetsHelper>> {
        let x_offset = defaulted(config_float(config, "x_offset", Some(Some(0.)), None, None, true)?);
        let y_offset = defaulted(config_float(config, "y_offset", Some(Some(0.)), None, None, true)?);
        let z_offset = defaulted(config_float(config, "z_offset", None, None, None, true)?);
        Ok(Arc::new(ProbeOffsetsHelper {
            x_offset,
            y_offset,
            z_offset,
        }))
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/probe.py:415`).
    pub fn get_offsets(&self, _gcmd: Option<&GCmd>) -> (f64, f64, f64) {
        (self.x_offset, self.y_offset, self.z_offset)
    }
}

impl ProbeOffsetsHelperApi for ProbeOffsetsHelper {
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> (f64, f64, f64) {
        ProbeOffsetsHelper::get_offsets(self, gcmd)
    }
}

// ---------------------------------------------------------------------------
// ProbePointsHelper
// ---------------------------------------------------------------------------

/// Helper code that can probe a series of points and report the position at each point
/// (`klippy/extras/probe.py:425`).
pub struct ProbePointsHelper {
    printer: Printer,
    finalize_callback: ProbeFinalizeCallback,
    probe_points: GCell<Vec<Vec<f64>>>,
    name: String,
    gcode: GCode,
    default_horizontal_move_z: f64,
    speed: f64,
    use_offsets: GCell<bool>,
    horizontal_move_z: GCell<Option<f64>>,
    // Internal probing state
    lift_speed: GCell<f64>,
    probe_offsets: GCell<(f64, f64, f64)>,
    manual_results: GCell<Vec<ProbeResult>>,
    self_ref: Weak<ProbePointsHelper>,
}

fn parse_points(value: &PyValue) -> KResult<Vec<Vec<f64>>> {
    fn number(v: &PyValue) -> KResult<f64> {
        match v {
            PyValue::Float(f) => Ok(*f),
            PyValue::Int(n) => Ok(n.0.to_string().parse::<f64>().unwrap_or(0.)),
            _ => Err(internal("TypeError", "probe points must be floats")),
        }
    }
    match value {
        PyValue::Tuple(points) | PyValue::List(points) => points
            .iter()
            .map(|point| match point {
                PyValue::Tuple(coords) | PyValue::List(coords) => {
                    coords.iter().map(number).collect()
                }
                _ => Err(internal("TypeError", "probe point must be a sequence")),
            })
            .collect(),
        _ => Err(internal("TypeError", "probe points must be a sequence")),
    }
}

impl ProbePointsHelper {
    /// `ProbePointsHelper.__init__(config, finalize_callback, default_points=None)`.
    pub fn new(
        config: &Config,
        finalize_callback: ProbeFinalizeCallback,
        default_points: Option<Vec<Vec<f64>>>,
    ) -> KResult<Arc<ProbePointsHelper>> {
        let printer = config.get_printer();
        let mut probe_points = default_points.clone();
        let name = config.get_name();
        let gcode = lookup::<dyn GCodeApi>(&printer, "gcode")?;
        // Read config settings
        if default_points.is_none()
            || config
                .get(
                    "points",
                    GetStrOpts {
                        default: Some(None),
                        note_valid: true,
                    },
                )?
                .is_some()
        {
            let points = config.getlists(
                "points",
                GetListsOpts {
                    seps: vec![",".to_string(), "\n".to_string()],
                    count: Some(2),
                    parser: ListParser::Float,
                    ..GetListsOpts::default()
                },
            )?;
            probe_points = Some(parse_points(&points)?);
        }
        let def_move_z = defaulted(config_float(
            config,
            "horizontal_move_z",
            Some(Some(5.)),
            None,
            None,
            true,
        )?);
        let speed = defaulted(config_float(
            config,
            "speed",
            Some(Some(50.)),
            None,
            Some(0.),
            true,
        )?);
        Ok(Arc::new_cyclic(|weak| ProbePointsHelper {
            printer,
            finalize_callback,
            probe_points: GCell::new(probe_points.unwrap_or_default()),
            name,
            gcode,
            default_horizontal_move_z: def_move_z,
            speed,
            use_offsets: GCell::new(false),
            horizontal_move_z: GCell::new(None),
            lift_speed: GCell::new(speed),
            probe_offsets: GCell::new((0., 0., 0.)),
            manual_results: GCell::new(Vec::new()),
            self_ref: weak.clone(),
        }))
    }

    /// `probe_points` (the attribute other modules read).
    pub fn probe_points(&self) -> Vec<Vec<f64>> {
        self.probe_points.cloned()
    }

    /// `minimum_points(n)` (`klippy/extras/probe.py:444`).
    pub fn minimum_points(&self, n: i64) -> KResult<()> {
        if (self.probe_points.borrow().len() as i64) < n {
            return Err(KlippyError::config(fmt(
                "Need at least %d probe points for %s",
                &[count(n), PctValue::Str(self.name.clone())],
            )?));
        }
        Ok(())
    }

    /// `update_probe_points(points, min_points)` (`klippy/extras/probe.py:448`).
    pub fn update_probe_points(&self, points: Vec<Vec<f64>>, min_points: i64) -> KResult<()> {
        self.probe_points.replace(points);
        self.minimum_points(min_points)
    }

    /// `use_xy_offsets(use_offsets)` (`klippy/extras/probe.py:451`).
    pub fn use_xy_offsets(&self, use_offsets: bool) {
        self.use_offsets.set(use_offsets);
    }

    /// `get_lift_speed()` (`klippy/extras/probe.py:453`).
    pub fn get_lift_speed(&self) -> f64 {
        self.lift_speed.get()
    }

    fn horizontal_move_z(&self) -> KResult<f64> {
        self.horizontal_move_z.get().ok_or_else(|| {
            internal(
                "AttributeError",
                "'ProbePointsHelper' object has no attribute 'horizontal_move_z'",
            )
        })
    }

    /// `_raise_tool(is_first=False)` (`klippy/extras/probe.py:457`). Pauses the reactor.
    fn raise_tool(&self, is_first: bool) -> KResult<()> {
        let mut speed = self.lift_speed.get();
        if is_first {
            // Use full speed to first probe position
            speed = self.speed;
        }
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        toolhead.manual_move(vec![None, None, Some(self.horizontal_move_z()?)], speed)
    }

    /// `_invoke_callback(results)` (`klippy/extras/probe.py:463`).
    fn invoke_callback(&self, results: &[ProbeResult]) -> KResult<bool> {
        // Flush lookahead queue
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        toolhead.get_last_move_time()?;
        // Invoke callback
        let res = (self.finalize_callback)(results)?;
        Ok(!matches!(&res, PyValue::Str(s) if s == "retry"))
    }

    /// `_move_next(probe_num)` (`klippy/extras/probe.py:470`). Pauses the reactor.
    fn move_next(&self, probe_num: usize) -> KResult<()> {
        // Move to next XY probe point
        let mut nextpos = self
            .probe_points
            .borrow()
            .get(probe_num)
            .cloned()
            .ok_or_else(index_error)?;
        if self.use_offsets.get() {
            let offsets = self.probe_offsets.get();
            nextpos[0] -= offsets.0;
            nextpos[1] -= offsets.1;
        }
        manual_move(&self.printer, &nextpos, self.speed)
    }

    /// `start_probe(gcmd)` (`klippy/extras/probe.py:477`). Pauses the reactor.
    pub fn start_probe(&self, gcmd: &GCmd) -> KResult<()> {
        verify_no_manual_probe(&self.printer)?;
        // Lookup objects
        let probe = lookup_interface::<dyn PrinterProbeApi>(
            &*self.printer,
            "probe",
            LookupDefault::ReturnNone,
        )?;
        let method = gcmd_str(gcmd, "METHOD", "automatic")?.to_lowercase();
        let def_move_z = self.default_horizontal_move_z;
        self.horizontal_move_z
            .set(Some(gcmd_float(gcmd, "HORIZONTAL_MOVE_Z", def_move_z, None, None)?));
        let probe = match probe {
            Some(probe) if method != "manual" => probe,
            _ => {
                // Manual probe
                self.lift_speed.set(self.speed);
                self.probe_offsets.set((0., 0., 0.));
                self.manual_results.replace(Vec::new());
                return self.manual_probe_start();
            }
        };
        // Perform automatic probing
        self.lift_speed
            .set(probe.get_probe_params(Some(gcmd))?.lift_speed);
        let probe_offsets = probe.get_offsets(Some(gcmd))?;
        self.probe_offsets.set(probe_offsets);
        if self.horizontal_move_z()? < probe_offsets.2 {
            return Err(KlippyError::command(
                "horizontal_move_z can't be less than probe's z_offset",
            ));
        }
        let probe_session = probe.start_probe_session(gcmd)?;
        let mut probe_num: usize = 0;
        loop {
            self.raise_tool(probe_num == 0)?;
            if probe_num >= self.probe_points.borrow().len() {
                let results = probe_session.pull_probed_results()?;
                let done = self.invoke_callback(&results)?;
                if done {
                    break;
                }
                // Caller wants a "retry" - restart probing
                probe_num = 0;
            }
            self.move_next(probe_num)?;
            probe_session.run_probe(gcmd)?;
            probe_num += 1;
        }
        probe_session.end_probe_session()
    }

    /// `_manual_probe_start()` (`klippy/extras/probe.py:513`). Pauses the reactor.
    fn manual_probe_start(&self) -> KResult<()> {
        let first = self.manual_results.borrow().is_empty();
        self.raise_tool(first)?;
        if self.manual_results.borrow().len() >= self.probe_points.borrow().len() {
            let results = self.manual_results.cloned();
            let done = self.invoke_callback(&results)?;
            if done {
                return Ok(());
            }
            // Caller wants a "retry" - clear results and restart probing
            self.manual_results.replace(Vec::new());
        }
        let next = self.manual_results.borrow().len();
        self.move_next(next)?;
        let gcmd = self.gcode.create_gcode_command("", "", &IndexMap::new());
        let this = self
            .self_ref
            .upgrade()
            .expect("the helper is alive while it is called");
        let callback: FinalizeCallback = Box::new(move |mpresult| this.manual_probe_finalize(mpresult));
        ManualProbeHelper::new(&self.printer, &gcmd, callback)?;
        Ok(())
    }

    /// `_manual_probe_finalize(mpresult)` (`klippy/extras/probe.py:525`).
    fn manual_probe_finalize(&self, mpresult: Option<ProbeResult>) -> KResult<()> {
        let Some(mpresult) = mpresult else {
            return Ok(());
        };
        self.manual_results.borrow_mut().push(mpresult);
        self.manual_probe_start()
    }
}

// ---------------------------------------------------------------------------
// ProbeEndstopWrapper
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Multi {
    Off,
    First,
    On,
}

/// Endstop wrapper that enables probe specific features (`klippy/extras/probe.py:545`).
pub struct ProbeEndstopWrapper {
    printer: Printer,
    stow_on_each_sample: bool,
    activate_gcode: TemplateWrapper,
    deactivate_gcode: TemplateWrapper,
    mcu_endstop: Endstop,
    homing_helper: DescendToEndstopHelper,
    // multi probes state
    multi: GCell<Multi>,
    self_ref: Weak<ProbeEndstopWrapper>,
}

impl ProbeEndstopWrapper {
    /// `ProbeEndstopWrapper.__init__(config, probe_offsets, param_helper)`.
    pub fn new(
        config: &Config,
        probe_offsets: Arc<dyn ProbeOffsetsHelperApi>,
        param_helper: Arc<dyn ProbeParameterHelperApi>,
    ) -> KResult<Arc<ProbeEndstopWrapper>> {
        let printer = config.get_printer();
        let stow_on_each_sample = defaulted(config.getboolean(
            "deactivate_on_each_sample",
            GetBoolOpts {
                default: Some(Some(true)),
                note_valid: true,
            },
        )?);
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let activate_gcode = gcode_macro.load_template(config, "activate_gcode", Some(""))?;
        let deactivate_gcode = gcode_macro.load_template(config, "deactivate_gcode", Some(""))?;
        // Create an "endstop" object to handle the probe pin
        let ppins = lookup::<dyn PinsApi>(&printer, "pins")?;
        let pin_desc = defaulted(config.get("pin", GetStrOpts::default())?);
        let mcu_endstop = match ppins.setup_pin("endstop", &pin_desc)? {
            Pin::Endstop(endstop) => endstop,
            _ => {
                return Err(internal(
                    "AttributeError",
                    "pin object has no attribute 'query_endstop'",
                ))
            }
        };
        // Probing via homing to endstop
        let homing_helper = DescendToEndstopHelper::new(
            config,
            Arc::clone(&mcu_endstop),
            probe_offsets,
            param_helper,
            false,
        )?;
        Ok(Arc::new_cyclic(|weak| ProbeEndstopWrapper {
            printer,
            stow_on_each_sample,
            activate_gcode,
            deactivate_gcode,
            mcu_endstop,
            homing_helper,
            multi: GCell::new(Multi::Off),
            self_ref: weak.clone(),
        }))
    }

    /// `query_endstop` (`mcu_endstop.query_endstop`).
    pub fn query_endstop(&self) -> QueryEndstopCb {
        let mcu_endstop = Arc::clone(&self.mcu_endstop);
        Arc::new(move |print_time| mcu_endstop.query_endstop(print_time))
    }

    /// `_raise_probe()` (`klippy/extras/probe.py:564`). Pauses the reactor.
    fn raise_probe(&self) -> KResult<()> {
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let start_pos = toolhead.get_position();
        self.deactivate_gcode.run_gcode_from_command(None)?;
        if first3(&toolhead.get_position()) != first3(&start_pos) {
            return Err(KlippyError::command(
                "Toolhead moved during probe deactivate_gcode script",
            ));
        }
        Ok(())
    }

    /// `_lower_probe()` (`klippy/extras/probe.py:571`). Pauses the reactor.
    fn lower_probe(&self) -> KResult<()> {
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let start_pos = toolhead.get_position();
        self.activate_gcode.run_gcode_from_command(None)?;
        if first3(&toolhead.get_position()) != first3(&start_pos) {
            return Err(KlippyError::command(
                "Toolhead moved during probe activate_gcode script",
            ));
        }
        Ok(())
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe.py:578`): returns this wrapper.
    pub fn start_probe_session(&self, _gcmd: &GCmd) -> KResult<Arc<ProbeEndstopWrapper>> {
        self.homing_helper.clear_trigger_positions();
        if !self.stow_on_each_sample {
            self.multi.set(Multi::First);
        }
        Ok(self
            .self_ref
            .upgrade()
            .expect("the wrapper is alive while it is called"))
    }

    /// `_probe_prepare()` (`klippy/extras/probe.py:583`).
    fn probe_prepare(&self) -> KResult<()> {
        let multi = self.multi.get();
        if multi == Multi::Off || multi == Multi::First {
            self.lower_probe()?;
            if multi == Multi::First {
                self.multi.set(Multi::On);
            }
        }
        Ok(())
    }

    /// `_probe_finish()` (`klippy/extras/probe.py:588`).
    fn probe_finish(&self) -> KResult<()> {
        if self.multi.get() == Multi::Off {
            self.raise_probe()?;
        }
        Ok(())
    }
}

impl ProbeSessionApi for ProbeEndstopWrapper {
    /// `run_probe(gcmd)` (`klippy/extras/probe.py:591`).
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        self.probe_prepare()?;
        match self.homing_helper.descend_until_trigger(gcmd) {
            Err(e @ KlippyError::Command(_)) => {
                self.probe_finish()?;
                return Err(e);
            }
            other => other?,
        }
        self.probe_finish()
    }

    /// `pull_probed_results()` (`klippy/extras/probe.py:599`).
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.homing_helper.pull_trigger_positions())
    }

    /// `end_probe_session()` (`klippy/extras/probe.py:601`).
    fn end_probe_session(&self) -> KResult<()> {
        self.homing_helper.clear_trigger_positions();
        if !self.stow_on_each_sample {
            self.raise_probe()?;
            self.multi.set(Multi::Off);
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// PrinterProbe
// ---------------------------------------------------------------------------

/// Main external probe interface (`klippy/extras/probe.py:608`).
pub struct PrinterProbe {
    probe_offsets: Arc<ProbeOffsetsHelper>,
    param_helper: Arc<ProbeParameterHelper>,
    mcu_probe: Arc<ProbeEndstopWrapper>,
    probe_session: Arc<SampleAveragingHelper>,
    cmd_helper: GCell<Option<Arc<ProbeCommandHelper>>>,
}

impl PrinterProbe {
    /// `PrinterProbe.__init__(config)`.
    pub fn new(config: &Config) -> KResult<Arc<PrinterProbe>> {
        let probe_offsets = ProbeOffsetsHelper::new(config)?;
        let param_helper = ProbeParameterHelper::new(config)?;
        let mcu_probe = ProbeEndstopWrapper::new(
            config,
            Arc::clone(&probe_offsets) as Arc<dyn ProbeOffsetsHelperApi>,
            Arc::clone(&param_helper) as Arc<dyn ProbeParameterHelperApi>,
        )?;
        let probe_session = SampleAveragingHelper::new(
            config,
            Arc::clone(&param_helper) as Arc<dyn ProbeParameterHelperApi>,
            {
                let mcu_probe = Arc::clone(&mcu_probe);
                Box::new(move |gcmd| {
                    Ok(mcu_probe.start_probe_session(gcmd)? as Arc<dyn ProbeSessionApi>)
                })
            },
        );
        let query_endstop = mcu_probe.query_endstop();
        let this = Arc::new(PrinterProbe {
            probe_offsets,
            param_helper,
            mcu_probe,
            probe_session,
            cmd_helper: GCell::new(None),
        });
        let weak_this: Weak<PrinterProbe> = Arc::downgrade(&this);
        let weak: Weak<dyn PrinterProbeApi> = weak_this;
        let cmd_helper =
            ProbeCommandHelper::new(config, weak, Some(Arc::clone(&query_endstop)), true)?;
        this.cmd_helper.replace(Some(cmd_helper));
        HomingViaProbeHelper::new(
            config,
            this.probe_offsets.get_offsets(None).2,
            Some(query_endstop),
        )?;
        Ok(this)
    }

    /// The `mcu_probe` attribute.
    pub fn mcu_probe(&self) -> &Arc<ProbeEndstopWrapper> {
        &self.mcu_probe
    }
}

impl PrinterProbeApi for PrinterProbe {
    /// `get_probe_params(gcmd=None)` (`klippy/extras/probe.py:621`).
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/probe.py:623`).
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        Ok(self.probe_offsets.get_offsets(gcmd))
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/probe.py:627`).
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        Ok(self.probe_session.start_probe_session(gcmd)? as Arc<dyn ProbeSessionApi>)
    }
}

impl PrinterObject for PrinterProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi);

    /// `get_status(eventtime)` (`klippy/extras/probe.py:625`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        let helper = self.cmd_helper.cloned()?;
        Some(helper.get_status(eventtime))
    }
}

/// `load_config(config)` (`klippy/extras/probe.py:630`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterProbe::new(config)? as ObjectRef))
}

/// Module entry point for the `probe` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
