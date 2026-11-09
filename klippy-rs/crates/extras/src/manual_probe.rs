//! Port of the `manual_probe` module (`klippy/extras/manual_probe.py`).
//!
//! Manual Z probing: the operator moves the nozzle with `TESTZ` and
//! confirms with `ACCEPT`, or aborts with `ABORT`.

use indexmap::IndexMap;
use klippy_api::config::{
    Config, GetFloatOpts, GetStrOpts, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GCodeCommandHandler, GCodeMove,
    GCodeMoveApi, GetFloatOpts as GcmdFloatOpts, GetOpts as GcmdGetOpts,
    RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{ToolHead, ToolHeadApi};
use klippy_api::{GCell, lookup_interface, load_interface, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// `Z_BOB_MINIMUM` (`klippy/extras/manual_probe.py:160`).
pub const Z_BOB_MINIMUM: f64 = 0.500;
/// `BISECT_MAX` (`klippy/extras/manual_probe.py:161`).
pub const BISECT_MAX: f64 = 0.200;

/// `ProbeResult` (`klippy/extras/manual_probe.py:14`): the probe result tuple.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProbeResult {
    /// `bed_x`
    pub bed_x: f64,
    /// `bed_y`
    pub bed_y: f64,
    /// `bed_z`
    pub bed_z: f64,
    /// `test_x`
    pub test_x: f64,
    /// `test_y`
    pub test_y: f64,
    /// `test_z`
    pub test_z: f64,
}

/// `create_probe_result(test_pos, offsets=(0., 0., 0.))`
/// (`klippy/extras/manual_probe.py:18`).
pub fn create_probe_result(test_pos: &[f64], offsets: [f64; 3]) -> ProbeResult {
    let [x_offset, y_offset, z_offset] = offsets;
    ProbeResult {
        bed_x: test_pos[0] + x_offset,
        bed_y: test_pos[1] + y_offset,
        bed_z: test_pos[2] - z_offset,
        test_x: test_pos[0],
        test_y: test_pos[1],
        test_z: test_pos[2],
    }
}

/// `lookup_z_endstop_config(config)` (`klippy/extras/manual_probe.py:25`).
pub fn lookup_z_endstop_config(config: &Config) -> KResult<Option<Config>> {
    if config.has_section("stepper_z") {
        return Ok(Some(config.getsection("stepper_z")));
    }
    for cconfig in config.get_prefix_sections("carriage ") {
        let carriage_name = cconfig
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or_default()
            .trim()
            .to_string();
        let axis_name = cconfig.get(
            "axis",
            GetStrOpts {
                default: Some(Some(carriage_name.clone())),
                note_valid: false,
            },
        )?;
        if axis_name.as_deref() == Some("z") {
            return Ok(Some(cconfig));
        }
    }
    Ok(None)
}

/// Status snapshot of `ManualProbe.status` (`klippy/extras/manual_probe.py:85`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManualProbeStatus {
    /// `is_active`
    pub is_active: bool,
    /// `z_position`
    pub z_position: Option<f64>,
    /// `z_position_lower`
    pub z_position_lower: Option<f64>,
    /// `z_position_upper`
    pub z_position_upper: Option<f64>,
}

impl ManualProbeStatus {
    fn initial() -> Self {
        ManualProbeStatus {
            is_active: false,
            z_position: None,
            z_position_lower: None,
            z_position_upper: None,
        }
    }

    fn to_pyvalue(self) -> PyValue {
        fn opt(v: Option<f64>) -> PyValue {
            match v {
                Some(f) => PyValue::Float(f),
                None => PyValue::None,
            }
        }
        let mut map = IndexMap::new();
        map.insert(PyValue::Str("is_active".to_string()), PyValue::Bool(self.is_active));
        map.insert(PyValue::Str("z_position".to_string()), opt(self.z_position));
        map.insert(PyValue::Str("z_position_lower".to_string()), opt(self.z_position_lower));
        map.insert(PyValue::Str("z_position_upper".to_string()), opt(self.z_position_upper));
        PyValue::Dict(map)
    }
}

/// Duck-typed status surface of the `manual_probe` printer object
/// (`klippy/extras/manual_probe.py:84,231,285`), as used by
/// [`ManualProbeHelper`] and other probing modules.
pub trait ManualProbeApi: Send + Sync + 'static {
    /// `ManualProbe.reset_status()` (`klippy/extras/manual_probe.py:84`).
    fn reset_status(&self);
    /// Write `ManualProbe.status` (`klippy/extras/manual_probe.py:231`).
    fn set_status(
        &self,
        is_active: bool,
        z_position: Option<f64>,
        z_position_lower: Option<f64>,
        z_position_upper: Option<f64>,
    );
}

/// `ManualProbe` (`klippy/extras/manual_probe.py:35`).
pub struct ManualProbe {
    printer: Printer,
    gcode: GCode,
    gcode_move: GCodeMove,
    z_position_endstop: Option<f64>,
    z_endstop_config_name: Option<String>,
    a_position_endstop: Option<f64>,
    b_position_endstop: Option<f64>,
    c_position_endstop: Option<f64>,
    status: GCell<ManualProbeStatus>,
}

impl ManualProbe {
    /// `ManualProbe.__init__(config)` (`klippy/extras/manual_probe.py:36`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let gcode_move =
            load_interface::<dyn GCodeMoveApi>(&*printer, config, "gcode_move", LookupDefault::Raise)?
                .expect("Raise never yields None");
        // Endstop value for cartesian printers with separate Z axis.
        let zconfig = lookup_z_endstop_config(config)?;
        let (z_position_endstop, z_endstop_config_name) = match &zconfig {
            Some(zconfig) => {
                let endstop = zconfig.getfloat(
                    "position_endstop",
                    GetFloatOpts {
                        default: Some(None),
                        note_valid: false,
                        ..GetFloatOpts::default()
                    },
                )?;
                (endstop, Some(zconfig.get_name()))
            }
            None => (None, None),
        };
        // Endstop values for linear delta printers with vertical A,B,C towers.
        let a_position_endstop = config.getsection("stepper_a").getfloat(
            "position_endstop",
            GetFloatOpts {
                default: Some(None),
                note_valid: false,
                ..GetFloatOpts::default()
            },
        )?;
        let b_position_endstop = config.getsection("stepper_b").getfloat(
            "position_endstop",
            GetFloatOpts {
                default: Some(None),
                note_valid: false,
                ..GetFloatOpts::default()
            },
        )?;
        let c_position_endstop = config.getsection("stepper_c").getfloat(
            "position_endstop",
            GetFloatOpts {
                default: Some(None),
                note_valid: false,
                ..GetFloatOpts::default()
            },
        )?;
        let this = Arc::new(ManualProbe {
            printer,
            gcode,
            gcode_move,
            z_position_endstop,
            z_endstop_config_name,
            a_position_endstop,
            b_position_endstop,
            c_position_endstop,
            status: GCell::new(ManualProbeStatus::initial()),
        });
        register(
            &this.gcode,
            "MANUAL_PROBE",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| {
                    let cb: FinalizeCallback = Box::new({
                        let this = Arc::clone(&this);
                        move |res| this.manual_probe_finalize(res)
                    });
                    ManualProbeHelper::new(&this.printer, gcmd, cb)?;
                    Ok(())
                })
            },
            Some(ManualProbe::cmd_manual_probe_help().to_string()),
        )?;
        // Conditionally register appropriate commands depending on printer.
        // Cartesian printers with separate Z axis.
        if this.z_position_endstop.is_some() {
            register(
                &this.gcode,
                "Z_ENDSTOP_CALIBRATE",
                {
                    let this = Arc::clone(&this);
                    Box::new(move |gcmd| {
                        let cb: FinalizeCallback = Box::new({
                            let this = Arc::clone(&this);
                            move |res| this.z_endstop_finalize(res)
                        });
                        ManualProbeHelper::new(&this.printer, gcmd, cb)?;
                        Ok(())
                    })
                },
                Some(ManualProbe::cmd_z_endstop_calibrate_help().to_string()),
            )?;
            register(
                &this.gcode,
                "Z_OFFSET_APPLY_ENDSTOP",
                {
                    let this = Arc::clone(&this);
                    Box::new(move |gcmd| this.cmd_z_offset_apply_endstop(gcmd))
                },
                Some(ManualProbe::cmd_z_offset_apply_endstop_help().to_string()),
            )?;
        }
        // Linear delta printers with A,B,C towers.
        if config.getsection("printer").get("kinematics", GetStrOpts::default())?.as_deref()
            == Some("delta")
        {
            register(
                &this.gcode,
                "Z_OFFSET_APPLY_ENDSTOP",
                {
                    let this = Arc::clone(&this);
                    Box::new(move |gcmd| this.cmd_z_offset_apply_delta_endstops(gcmd))
                },
                Some(ManualProbe::cmd_z_offset_apply_endstop_help().to_string()),
            )?;
        }
        this.reset_status();
        Ok(this)
    }

    /// `ManualProbe.cmd_MANUAL_PROBE_help` (`klippy/extras/manual_probe.py:93`).
    pub fn cmd_manual_probe_help() -> &'static str {
        "Start manual probe helper script"
    }

    /// `ManualProbe.cmd_Z_ENDSTOP_CALIBRATE_help`
    /// (`klippy/extras/manual_probe.py:108`).
    pub fn cmd_z_endstop_calibrate_help() -> &'static str {
        "Calibrate a Z endstop"
    }

    /// `ManualProbe.cmd_Z_OFFSET_APPLY_ENDSTOP_help`
    /// (`klippy/extras/manual_probe.py:148`).
    pub fn cmd_z_offset_apply_endstop_help() -> &'static str {
        "Adjust the z endstop_position"
    }

    /// `ManualProbe.manual_probe_finalize(mpresult)`
    /// (`klippy/extras/manual_probe.py:81`).
    pub fn manual_probe_finalize(&self, mpresult: Option<ProbeResult>) -> KResult<()> {
        if let Some(mpresult) = mpresult {
            let msg = format_f(
                "Z position is %.3f",
                &[PctValue::Float(mpresult.bed_z)],
            )?;
            self.gcode.respond_info(&msg, true);
        }
        Ok(())
    }

    /// `ManualProbe.z_endstop_finalize(mpresult)`
    /// (`klippy/extras/manual_probe.py:96`).
    pub fn z_endstop_finalize(&self, mpresult: Option<ProbeResult>) -> KResult<()> {
        let Some(mpresult) = mpresult else { return Ok(()) };
        let z_pos = self.z_position_endstop.unwrap() - mpresult.bed_z;
        let msg = format_f(
            "%s: position_endstop: %.3f\n\
             The SAVE_CONFIG command will update the printer config file\n\
             with the above and restart the printer.",
            &[
                PctValue::Str(self.z_endstop_config_name.clone().unwrap_or_default()),
                PctValue::Float(z_pos),
            ],
        )?;
        self.gcode.respond_info(&msg, true);
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let value = format_f("%.3f", &[PctValue::Float(z_pos)])?;
        configfile.set(
            self.z_endstop_config_name.clone().unwrap_or_default().as_str(),
            "position_endstop",
            &value,
        )?;
        Ok(())
    }

    /// `ManualProbe.cmd_Z_OFFSET_APPLY_ENDSTOP(gcmd)`
    /// (`klippy/extras/manual_probe.py:111`).
    pub fn cmd_z_offset_apply_endstop(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let offset = self.homing_origin_z()?;
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        if offset == 0.0 {
            self.gcode.respond_info("Nothing to do: Z Offset is 0", true);
        } else {
            let new_calibrate = self.z_position_endstop.unwrap() - offset;
            let msg = format_f(
                "%s: position_endstop: %.3f\n\
                 The SAVE_CONFIG command will update the printer config file\n\
                 with the above and restart the printer.",
                &[
                    PctValue::Str(self.z_endstop_config_name.clone().unwrap_or_default()),
                    PctValue::Float(new_calibrate),
                ],
            )?;
            self.gcode.respond_info(&msg, true);
            let value = format_f("%.3f", &[PctValue::Float(new_calibrate)])?;
            configfile.set(
                self.z_endstop_config_name.clone().unwrap_or_default().as_str(),
                "position_endstop",
                &value,
            )?;
        }
        let _ = gcmd;
        Ok(())
    }

    /// `ManualProbe.cmd_Z_OFFSET_APPLY_DELTA_ENDSTOPS(gcmd)`
    /// (`klippy/extras/manual_probe.py:125`).
    pub fn cmd_z_offset_apply_delta_endstops(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let offset = self.homing_origin_z()?;
        let configfile = lookup_interface::<dyn PrinterConfigApi>(
            &*self.printer,
            "configfile",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        if offset == 0.0 {
            self.gcode.respond_info("Nothing to do: Z Offset is 0", true);
        } else {
            let new_a_calibrate = self.a_position_endstop.unwrap() - offset;
            let new_b_calibrate = self.b_position_endstop.unwrap() - offset;
            let new_c_calibrate = self.c_position_endstop.unwrap() - offset;
            let msg = format_f(
                "stepper_a: position_endstop: %.3f\n\
                 stepper_b: position_endstop: %.3f\n\
                 stepper_c: position_endstop: %.3f\n\
                 The SAVE_CONFIG command will update the printer config file\n\
                 with the above and restart the printer.",
                &[
                    PctValue::Float(new_a_calibrate),
                    PctValue::Float(new_b_calibrate),
                    PctValue::Float(new_c_calibrate),
                ],
            )?;
            self.gcode.respond_info(&msg, true);
            configfile.set(
                "stepper_a",
                "position_endstop",
                &format_f("%.3f", &[PctValue::Float(new_a_calibrate)])?,
            )?;
            configfile.set(
                "stepper_b",
                "position_endstop",
                &format_f("%.3f", &[PctValue::Float(new_b_calibrate)])?,
            )?;
            configfile.set(
                "stepper_c",
                "position_endstop",
                &format_f("%.3f", &[PctValue::Float(new_c_calibrate)])?,
            )?;
        }
        let _ = gcmd;
        Ok(())
    }

    /// `get_status()['homing_origin'].z` (`klippy/extras/manual_probe.py:112`).
    fn homing_origin_z(&self) -> KResult<f64> {
        let status = self.gcode_move.get_status(None);
        let origin = py_dict_get(&status, "homing_origin")?;
        match origin {
            PyValue::Tuple(items) | PyValue::List(items) => match items.get(2) {
                Some(PyValue::Float(z)) => Ok(*z),
                Some(PyValue::Int(n)) => {
                    let s = n.inner().to_string();
                    s.parse::<f64>().map_err(|_| {
                        KlippyError::Internal {
                            class_name: "TypeError".to_string(),
                            message: "unsupported operand type(s) for +: 'NoneType' and 'float'".to_string(),
                        }
                    })
                }
                _ => Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "unsupported operand type(s) for +: 'NoneType' and 'float'".to_string(),
                }),
            },
            _ => Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "unsupported operand type(s) for +: 'NoneType' and 'float'".to_string(),
            }),
        }
    }

    /// Current `status` snapshot (`klippy/extras/manual_probe.py:92`).
    pub fn status_snapshot(&self) -> ManualProbeStatus {
        self.status.cloned()
    }

    /// `z_position_endstop` (`klippy/extras/manual_probe.py:46`).
    pub fn z_position_endstop(&self) -> Option<f64> {
        self.z_position_endstop
    }

    /// `z_endstop_config_name` (`klippy/extras/manual_probe.py:48`).
    pub fn z_endstop_config_name(&self) -> Option<&str> {
        self.z_endstop_config_name.as_deref()
    }
}

impl ManualProbeApi for ManualProbe {
    fn reset_status(&self) {
        *self.status.borrow_mut() = ManualProbeStatus::initial();
    }

    fn set_status(
        &self,
        is_active: bool,
        z_position: Option<f64>,
        z_position_lower: Option<f64>,
        z_position_upper: Option<f64>,
    ) {
        *self.status.borrow_mut() = ManualProbeStatus {
            is_active,
            z_position,
            z_position_lower,
            z_position_upper,
        };
    }
}

impl PrinterObject for ManualProbe {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ManualProbeApi);

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status.cloned().to_pyvalue()))
    }
}

/// `verify_no_manual_probe(printer)` (`klippy/extras/manual_probe.py:151`).
pub fn verify_no_manual_probe(printer: &Printer) -> KResult<()> {
    let gcode = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("Raise never yields None");
    match register(
        &gcode,
        "ACCEPT",
        Box::new(|_gcmd| Ok(())),
        None,
    ) {
        Err(KlippyError::Config(_)) => {
            return Err(KlippyError::command(
                "Already in a manual Z probe. Use ABORT to abort it.",
            ));
        }
        Err(e) => return Err(e),
        Ok(()) => {}
    }
    unregister(&gcode, "ACCEPT")?;
    Ok(())
}

/// Finalize callback of `ManualProbeHelper` (`klippy/extras/manual_probe.py:165`).
pub type FinalizeCallback =
    Box<dyn FnMut(Option<ProbeResult>) -> KResult<()> + Send>;

struct HelperState {
    past_positions: Vec<f64>,
    last_toolhead_pos: Option<Vec<f64>>,
    last_kinematics_pos: Option<Vec<Option<f64>>>,
}

/// `ManualProbeHelper` (`klippy/extras/manual_probe.py:164`).
pub struct ManualProbeHelper {
    #[allow(dead_code)]
    printer: Printer,
    finalize_callback: GCell<Option<FinalizeCallback>>,
    gcode: GCode,
    toolhead: ToolHead,
    manual_probe: Arc<dyn ManualProbeApi>,
    speed: f64,
    start_position: GCell<Vec<f64>>,
    state: GCell<HelperState>,
}

impl ManualProbeHelper {
    /// `ManualProbeHelper.__init__(printer, gcmd, finalize_callback)`
    /// (`klippy/extras/manual_probe.py:165`).
    pub fn new(
        printer: &Printer,
        gcmd: &klippy_api::gcode::GCmd,
        finalize_callback: FinalizeCallback,
    ) -> KResult<Arc<Self>> {
        let gcode = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let toolhead =
            lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let manual_probe = lookup_interface::<dyn ManualProbeApi>(
            &**printer,
            "manual_probe",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let speed = gcmd
            .get_float(
                "SPEED",
                GcmdFloatOpts {
                    default: Some(Some(5.0)),
                    ..GcmdFloatOpts::default()
                },
            )?
            .unwrap();
        let this = Arc::new(ManualProbeHelper {
            printer: Arc::clone(printer),
            finalize_callback: GCell::new(Some(finalize_callback)),
            gcode,
            toolhead,
            manual_probe,
            speed,
            start_position: GCell::new(Vec::new()),
            state: GCell::new(HelperState {
                past_positions: Vec::new(),
                last_toolhead_pos: None,
                last_kinematics_pos: None,
            }),
        });
        // Register commands.
        verify_no_manual_probe(printer)?;
        register(
            &this.gcode,
            "ACCEPT",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_accept(gcmd))
            },
            Some(Self::cmd_accept_help().to_string()),
        )?;
        register(
            &this.gcode,
            "NEXT",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_accept(gcmd))
            },
            None,
        )?;
        register(
            &this.gcode,
            "ABORT",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_abort(gcmd))
            },
            Some(Self::cmd_abort_help().to_string()),
        )?;
        register(
            &this.gcode,
            "TESTZ",
            {
                let this = Arc::clone(&this);
                Box::new(move |gcmd| this.cmd_testz(gcmd))
            },
            Some(Self::cmd_testz_help().to_string()),
        )?;
        this.gcode.respond_info(
            "Starting manual Z probe. Use TESTZ to adjust position.\n\
             Finish with ACCEPT or ABORT command.",
            true,
        );
        *this.start_position.borrow_mut() = this.toolhead.get_position();
        this.report_z_status(false, None)?;
        Ok(this)
    }

    /// `ManualProbeHelper.cmd_ACCEPT_help` (`klippy/extras/manual_probe.py:240`).
    pub fn cmd_accept_help() -> &'static str {
        "Accept the current Z position"
    }

    /// `ManualProbeHelper.cmd_ABORT_help` (`klippy/extras/manual_probe.py:251`).
    pub fn cmd_abort_help() -> &'static str {
        "Abort manual Z probing tool"
    }

    /// `ManualProbeHelper.cmd_TESTZ_help` (`klippy/extras/manual_probe.py:254`).
    pub fn cmd_testz_help() -> &'static str {
        "Move to new Z height"
    }

    /// `ManualProbeHelper.get_kinematics_pos()`
    /// (`klippy/extras/manual_probe.py:188`).
    pub fn get_kinematics_pos(&self) -> KResult<Vec<Option<f64>>> {
        let toolhead_pos = self.toolhead.get_position();
        {
            let state = self.state.borrow();
            if state.last_toolhead_pos.as_ref() == Some(&toolhead_pos) {
                return Ok(state.last_kinematics_pos.clone().unwrap_or_default());
            }
        }
        self.toolhead.flush_step_generation()?;
        let kin = self.toolhead.get_kinematics();
        let mut kin_spos = IndexMap::new();
        for s in kin.get_steppers() {
            kin_spos.insert(s.get_name(None), s.get_commanded_position());
        }
        let kin_pos = kin.calc_position(&kin_spos)?;
        {
            let mut state = self.state.borrow_mut();
            state.last_toolhead_pos = Some(toolhead_pos);
            state.last_kinematics_pos = Some(kin_pos.clone());
        }
        Ok(kin_pos)
    }

    /// `ManualProbeHelper.move_z(z_pos)` (`klippy/extras/manual_probe.py:200`).
    pub fn move_z(&self, z_pos: f64) -> KResult<()> {
        let curpos = self.toolhead.get_position();
        let z_bob_pos = z_pos + Z_BOB_MINIMUM;
        let result = if curpos[2] < z_bob_pos {
            self.toolhead
                .manual_move(vec![None, None, Some(z_bob_pos)], self.speed)
                .and_then(|()| self.toolhead.manual_move(vec![None, None, Some(z_pos)], self.speed))
        } else {
            self.toolhead.manual_move(vec![None, None, Some(z_pos)], self.speed)
        };
        match result {
            Err(KlippyError::Command(_)) => {
                self.finalize(false)?;
                result
            }
            other => other,
        }
    }

    /// `ManualProbeHelper.report_z_status(warn_no_change=False, prev_pos=None)`
    /// (`klippy/extras/manual_probe.py:210`).
    pub fn report_z_status(
        &self,
        warn_no_change: bool,
        prev_pos: Option<f64>,
    ) -> KResult<()> {
        // Get position.
        let kin_pos = self.get_kinematics_pos()?;
        let z_pos = kin_z(&kin_pos)?;
        if warn_no_change && prev_pos == Some(z_pos) {
            self.gcode.respond_info(
                "WARNING: No change in position (reached stepper resolution)",
                true,
            );
        }
        // Find recent positions that were tested.
        let (prev_pos_val, next_pos_val, prev_str, next_str) = {
            let pp = &self.state.borrow().past_positions;
            let mut next_pos = pp.partition_point(|v| *v < z_pos);
            let prev_pos = next_pos as i64 - 1;
            if next_pos < pp.len() && pp[next_pos] == z_pos {
                next_pos += 1;
            }
            let prev_pos_val = if prev_pos >= 0 {
                Some(pp[prev_pos as usize])
            } else {
                None
            };
            let next_pos_val = if next_pos < pp.len() {
                Some(pp[next_pos])
            } else {
                None
            };
            let prev_str = match prev_pos_val {
                Some(v) => format_f("%.3f", &[PctValue::Float(v)])?,
                None => "??????".to_string(),
            };
            let next_str = match next_pos_val {
                Some(v) => format_f("%.3f", &[PctValue::Float(v)])?,
                None => "??????".to_string(),
            };
            (prev_pos_val, next_pos_val, prev_str, next_str)
        };
        self.manual_probe.set_status(true, Some(z_pos), prev_pos_val, next_pos_val);
        // Find recent positions.
        let msg = format_f(
            "Z position: %s --> %.3f <-- %s",
            &[PctValue::Str(prev_str), PctValue::Float(z_pos), PctValue::Str(next_str)],
        )?;
        self.gcode.respond_info(&msg, true);
        Ok(())
    }

    /// `ManualProbeHelper.cmd_ACCEPT(gcmd)` (`klippy/extras/manual_probe.py:241`).
    pub fn cmd_accept(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let pos = self.toolhead.get_position();
        let start_pos = self.start_position.borrow().clone();
        if pos[..2] != start_pos[..2] || pos[2] >= start_pos[2] {
            gcmd.respond_info(
                "Manual probe failed! Use TESTZ commands to position the\n\
                 nozzle prior to running ACCEPT.",
                true,
            );
            self.finalize(false)?;
            return Ok(());
        }
        self.finalize(true)
    }

    /// `ManualProbeHelper.cmd_ABORT(gcmd)` (`klippy/extras/manual_probe.py:252`).
    pub fn cmd_abort(&self, _gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.finalize(false)
    }

    /// `ManualProbeHelper.cmd_TESTZ(gcmd)` (`klippy/extras/manual_probe.py:255`).
    pub fn cmd_testz(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        // Store current position for later reference.
        let kin_pos = self.get_kinematics_pos()?;
        let z_pos = kin_z(&kin_pos)?;
        let insert_pos = {
            let mut state = self.state.borrow_mut();
            let pp = &mut state.past_positions;
            let insert_pos = pp.partition_point(|v| *v < z_pos);
            if insert_pos >= pp.len() || pp[insert_pos] != z_pos {
                pp.insert(insert_pos, z_pos);
            }
            insert_pos
        };
        // Determine next position to move to.
        let req = gcmd.get("Z", GcmdGetOpts::default())?;
        let req_str = match &req {
            PyValue::Str(s) => s.clone(),
            other => other.str(),
        };
        let next_z_pos = if req_str == "+" || req_str == "++" {
            let mut check_z = 9999999999999.9;
            {
                let pp = &self.state.borrow().past_positions;
                if insert_pos < pp.len() - 1 {
                    check_z = pp[insert_pos + 1];
                }
            }
            if req_str == "+" {
                check_z = (check_z + z_pos) / 2.0;
            }
            check_z.min(z_pos + BISECT_MAX)
        } else if req_str == "-" || req_str == "--" {
            let mut check_z = -9999999999999.9;
            {
                let pp = &self.state.borrow().past_positions;
                if insert_pos > 0 {
                    check_z = pp[insert_pos - 1];
                }
            }
            if req_str == "-" {
                check_z = (check_z + z_pos) / 2.0;
            }
            check_z.max(z_pos - BISECT_MAX)
        } else {
            let z = gcmd
                .get_float(
                    "Z",
                    GcmdFloatOpts {
                        default: None,
                        ..GcmdFloatOpts::default()
                    },
                )?
                .unwrap();
            z_pos + z
        };
        // Move to given position and report it.
        self.move_z(next_z_pos)?;
        self.report_z_status(next_z_pos != z_pos, Some(z_pos))
    }

    /// `ManualProbeHelper.finalize(success)` (`klippy/extras/manual_probe.py:284`).
    pub fn finalize(&self, success: bool) -> KResult<()> {
        self.manual_probe.reset_status();
        unregister(&self.gcode, "ACCEPT")?;
        unregister(&self.gcode, "NEXT")?;
        unregister(&self.gcode, "ABORT")?;
        unregister(&self.gcode, "TESTZ")?;
        let mpresult = if success {
            let kin_pos = self.get_kinematics_pos()?;
            let mut floats = Vec::with_capacity(kin_pos.len());
            for v in &kin_pos {
                match v {
                    Some(f) => floats.push(*f),
                    None => {
                        return Err(KlippyError::Internal {
                            class_name: "TypeError".to_string(),
                            message: "unsupported operand type(s) for +: 'NoneType' and 'float'"
                                .to_string(),
                        });
                    }
                }
            }
            Some(create_probe_result(&floats, [0.0, 0.0, 0.0]))
        } else {
            None
        };
        let cb = self.finalize_callback.borrow_mut().take();
        if let Some(mut cb) = cb {
            cb(mpresult)?;
        }
        Ok(())
    }

    /// Stored `speed` (`klippy/extras/manual_probe.py:171`).
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Stored `start_position` (`klippy/extras/manual_probe.py:186`).
    pub fn start_position(&self) -> Vec<f64> {
        self.start_position.borrow().clone()
    }

    /// `past_positions` (`klippy/extras/manual_probe.py:172`).
    pub fn past_positions(&self) -> Vec<f64> {
        self.state.borrow().past_positions.clone()
    }
}

fn kin_z(kin_pos: &[Option<f64>]) -> KResult<f64> {
    match kin_pos.get(2) {
        Some(Some(z)) => Ok(*z),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "unsupported operand type(s) for +: 'NoneType' and 'float'".to_string(),
        }),
    }
}

fn py_dict_get<'a>(value: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match value {
        PyValue::Dict(map) => {
            for (k, v) in map {
                if k.str() == key {
                    return Ok(v);
                }
            }
            Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: pycompat::text::str_repr(key),
            })
        }
        _ => Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: pycompat::text::str_repr(key),
        }),
    }
}

fn format_f(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn register(
    gcode: &GCode,
    cmd: &str,
    func: GCodeCommandHandler,
    desc: Option<String>,
) -> KResult<()> {
    gcode.register_command(
        cmd,
        Some(func),
        RegisterCommandOpts {
            when_not_ready: false,
            desc,
        },
    )?;
    Ok(())
}

fn unregister(gcode: &GCode, cmd: &str) -> KResult<()> {
    gcode.register_command(
        cmd,
        None,
        RegisterCommandOpts {
            when_not_ready: false,
            desc: None,
        },
    )?;
    Ok(())
}

/// `load_config(config)` (`klippy/extras/manual_probe.py:296`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ManualProbe::new(config)? as ObjectRef))
}

/// Module entry point for the `manual_probe` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
