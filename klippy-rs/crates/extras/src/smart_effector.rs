//! Port of `klippy/extras/smart_effector.py`.
//!
//! The SmartEffector daughter board is driven by bit-banging a control
//! pin at 1000 baud over the motion queue (`ControlPinHelper`). On top of
//! that, `SmartEffectorProbe` layers probing with accel control and the
//! `SET_SMART_EFFECTOR` / `RESET_SMART_EFFECTOR` commands, and
//! `PrinterSmartEffector` wires the `probe.py` helpers together and
//! registers itself as the `probe` printer object.
//!
//! Probe helpers (`ProbeEndstopWrapper`, `ProbeOffsetsHelper`,
//! `ProbeParameterHelper`, `SampleAveragingHelper`, `ProbeCommandHelper`,
//! `HomingViaProbeHelper` and the `PrinterProbeApi` / `ProbeSessionApi`
//! traits) belong to `crate::probe` (`klippy/extras/probe.py`) and are
//! imported from there.

use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCode, GCodeApi, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{AddConfigCmdOpts, CommandWrapper, LookupPinOpts, PinsApi, SendOpts};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, ObjectRef, PrinterObject};
use klippy_api::GCell;
use pycompat::int::PyInt;
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;
use std::sync::Weak;

use crate::manual_probe::ProbeResult;
use crate::probe::{
    HomingViaProbeHelper, PrinterProbeApi, ProbeCommandHelper, ProbeEndstopWrapper,
    ProbeOffsetsHelper, ProbeOffsetsHelperApi, ProbeParameterHelper,
    ProbeParameterHelperApi, ProbeSessionApi, ProbeParams, QueryEndstopCb,
    SampleAveragingHelper,
};

/// SmartEffector control signaling speed (`klippy/extras/smart_effector.py:12`).
pub const BITS_PER_SECOND: f64 = 1000.0;

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_str(s: &str) -> PctValue {
    PctValue::Str(s.to_string())
}

fn pct_float(v: f64) -> PctValue {
    PctValue::Float(v)
}

fn fmt(f: &str, args: &[PctValue]) -> String {
    percent_format(f, args).expect("constant smart_effector format")
}

/// Faithful dict lookup with Python's `KeyError` text on a missing key.
fn py_dict_get<'a>(value: &'a PyValue, key: &str) -> KResult<&'a PyValue> {
    match value {
        PyValue::Dict(d) => d
            .get(&PyValue::Str(key.to_string()))
            .ok_or_else(|| KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: pycompat::text::str_repr(key),
            }),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "'{}' object is not subscriptable",
                pycompat::value::type_name(value)
            ),
        }),
    }
}

// ---------------------------------------------------------------------------
// ControlPinHelper (smart_effector.py:14-49)
// ---------------------------------------------------------------------------

/// Bit-bangs the SmartEffector control line through the motion queue.
pub struct ControlPinHelper {
    mcu: klippy_api::mcu::Mcu,
    pin: String,
    start_value: i64,
    invert: i64,
    oid: GCell<Option<i64>>,
    set_cmd: GCell<Option<CommandWrapper>>,
}

impl ControlPinHelper {
    /// `ControlPinHelper(pin_params)` (`klippy/extras/smart_effector.py:15`).
    pub fn new(pin_params: &klippy_api::mcu::PinParams) -> KResult<Arc<Self>> {
        let mcu = pin_params.chip.clone();
        let helper = Arc::new(ControlPinHelper {
            mcu: mcu.clone(),
            pin: pin_params.pin.clone(),
            start_value: pin_params.invert,
            invert: pin_params.invert,
            oid: GCell::new(None),
            set_cmd: GCell::new(None),
        });
        let built = helper.clone();
        mcu.register_config_callback(Box::new(move || built.build_config()))?;
        Ok(helper)
    }

    /// `_build_config` (`klippy/extras/smart_effector.py:22`).
    pub fn build_config(&self) -> KResult<()> {
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid()?;
        let cmd = fmt(
            "config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
            &[
                pct_int(oid),
                pct_str(&self.pin),
                pct_int(self.start_value),
                pct_int(self.start_value),
                pct_int(0),
            ],
        );
        self.mcu.add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let cq = self.mcu.alloc_command_queue();
        let set_cmd = self
            .mcu
            .lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cq))?;
        *self.oid.borrow_mut() = Some(oid);
        *self.set_cmd.borrow_mut() = Some(set_cmd);
        Ok(())
    }

    /// `write_bits` (`klippy/extras/smart_effector.py:32`).
    pub fn write_bits(&self, start_time: f64, bit_stream: &[i64]) -> KResult<f64> {
        let bit_step = 1.0 / BITS_PER_SECOND;
        let mut last_value = self.start_value;
        let mut bit_time = start_time;
        // Python references `value` after the loop; an empty stream raises
        // UnboundLocalError (a NameError subclass). Reproduce it.
        let mut value = 0i64;
        let mut has_value = false;
        for &b in bit_stream {
            value = ((if b != 0 { 1 } else { 0 }) ^ self.invert) & 1;
            has_value = true;
            if value != last_value {
                self.send_one(bit_time, value)?;
                last_value = value;
            }
            bit_time += bit_step;
        }
        if !has_value {
            return Err(KlippyError::Internal {
                class_name: "UnboundLocalError".to_string(),
                message: "local variable 'value' referenced before assignment".to_string(),
            });
        }
        // After the last bit, the signal must go back to its start value.
        if value != self.start_value {
            self.send_one(bit_time, self.start_value)?;
            bit_time += bit_step;
        }
        Ok(bit_time)
    }

    fn send_one(&self, bit_time: f64, value: i64) -> KResult<()> {
        let oid = self.oid.borrow().ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'NoneType' object has no attribute 'send'".to_string(),
        })?;
        let cmd = self.set_cmd.borrow().clone().ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'NoneType' object has no attribute 'send'".to_string(),
        })?;
        let clock = self.mcu.print_time_to_clock(bit_time);
        cmd.send(
            vec![
                PyValue::Int(PyInt::from_i64(oid)),
                PyValue::Int(PyInt::from_i64(clock)),
                PyValue::Int(PyInt::from_i64(value)),
            ],
            SendOpts::default(),
        )
    }
}
// ---------------------------------------------------------------------------
// SmartEffectorProbe (smart_effector.py:51-157)
// ---------------------------------------------------------------------------

/// Probing with SmartEffector accel control and sensitivity programming.
///
/// The Python object is its own probe session: `start_probe_session`
/// returns `self`, and `run_probe` / `pull_probed_results` /
/// `end_probe_session` delegate to the stored wrapper session.
pub struct SmartEffectorProbe {
    printer: Printer,
    gcode: GCode,
    probe_wrapper: Arc<ProbeEndstopWrapper>,
    probe_session: GCell<Option<Arc<ProbeEndstopWrapper>>>,
    probe_accel: GCell<f64>,
    recovery_time: GCell<f64>,
    control_pin: Option<Arc<ControlPinHelper>>,
    old_max_accel: GCell<f64>,
}

impl SmartEffectorProbe {
    /// `SmartEffectorProbe(config, probe_offsets, param_helper)`
    /// (`klippy/extras/smart_effector.py:51`).
    pub fn new(
        config: &Config,
        probe_offsets: Arc<ProbeOffsetsHelper>,
        param_helper: Arc<ProbeParameterHelper>,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let probe_accel = config
            .getfloat(
                "probe_accel",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("getfloat with default never yields None");
        let recovery_time = config
            .getfloat(
                "recovery_time",
                GetFloatOpts {
                    default: Some(Some(0.4)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("getfloat with default never yields None");
        let probe_wrapper = ProbeEndstopWrapper::new(
            config,
            probe_offsets as Arc<dyn ProbeOffsetsHelperApi>,
            param_helper as Arc<dyn ProbeParameterHelperApi>,
        )?;
        let control_pin_str = config
            .get(
                "control_pin",
                GetStrOpts {
                    default: Some(None),
                    ..Default::default()
                },
            )?;
        let control_pin = match control_pin_str.as_deref() {
            Some(s) if !s.is_empty() => {
                let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
                    .expect("LookupDefault::Raise never yields None");
                let pin_params = pins.lookup_pin(
                    s,
                    LookupPinOpts {
                        can_invert: true,
                        ..Default::default()
                    },
                )?;
                Some(ControlPinHelper::new(&pin_params)?)
            }
            _ => None,
        };
        let probe = SmartEffectorProbe {
            printer: printer.clone(),
            gcode: gcode.clone(),
            probe_wrapper,
            probe_session: GCell::new(None),
            probe_accel: GCell::new(probe_accel),
            recovery_time: GCell::new(recovery_time),
            control_pin,
            old_max_accel: GCell::new(0.0),
        };
        let this = Arc::new(probe);
        // `RESET_SMART_EFFECTOR` is registered only with a control pin
        // (smart_effector.py:63-69); `SET_SMART_EFFECTOR` always (line 72).
        if this.control_pin.is_some() {
            let me = this.clone();
            this.gcode.register_command(
                "RESET_SMART_EFFECTOR",
                Some(Box::new(move |gcmd| me.cmd_reset_smart_effector(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some("Reset SmartEffector settings (sensitivity)".to_string()),
                },
            )?;
        }
        let me = this.clone();
        this.gcode.register_command(
            "SET_SMART_EFFECTOR",
            Some(Box::new(move |gcmd| me.cmd_set_smart_effector(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Set SmartEffector parameters".to_string()),
            },
        )?;
        Ok(this)
    }

    /// `_probe_prepare` (`klippy/extras/smart_effector.py:75`).
    fn probe_prepare(&self) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        if self.probe_accel.get() != 0.0 {
            let systime = self.printer.get_reactor().monotonic();
            let status = toolhead.get_status(systime);
            let max_accel = py_dict_get(&status, "max_accel")?.to_f64().unwrap_or(0.0);
            self.old_max_accel.set(max_accel);
            self.gcode
                .run_script_from_command(&fmt("M204 S%.3f", &[pct_float(self.probe_accel.get())]))?;
        }
        if self.recovery_time.get() != 0.0 {
            toolhead.dwell(self.recovery_time.get());
        }
        Ok(())
    }

    /// `_probe_finish` (`klippy/extras/smart_effector.py:85`).
    fn probe_finish(&self) -> KResult<()> {
        if self.probe_accel.get() != 0.0 {
            self.gcode.run_script_from_command(&fmt(
                "M204 S%.3f",
                &[pct_float(self.old_max_accel.get())],
            ))?;
        }
        Ok(())
    }

    /// `query_endstop` (`klippy/extras/smart_effector.py:59`, bound method).
    pub fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        (self.probe_wrapper.query_endstop())(print_time)
    }

    /// The `probe_wrapper.query_endstop` callback for the probe helpers.
    pub fn query_endstop_cb(&self) -> QueryEndstopCb {
        self.probe_wrapper.query_endstop()
    }

    /// `start_probe_session` (`klippy/extras/smart_effector.py:89`).
    pub fn start_probe_session(self: &Arc<Self>, gcmd: &klippy_api::gcode::GCmd) -> KResult<Arc<Self>> {
        let session = self.probe_wrapper.start_probe_session(gcmd)?;
        *self.probe_session.borrow_mut() = Some(session);
        Ok(self.clone())
    }

    /// `run_probe` (`klippy/extras/smart_effector.py:92`).
    pub fn run_probe(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.probe_prepare()?;
        let session = self.probe_session.borrow().clone().ok_or_else(|| {
            KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'run_probe'".to_string(),
            }
        })?;
        let result = session.run_probe(gcmd);
        match result {
            Err(e) if matches!(e, KlippyError::Command(_)) => {
                let _ = self.probe_finish();
                Err(e)
            }
            Err(e) => Err(e),
            Ok(()) => self.probe_finish(),
        }
    }

    /// `pull_probed_results` (`klippy/extras/smart_effector.py:100`).
    pub fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        match self.probe_session.borrow().clone() {
            Some(s) => s.pull_probed_results(),
            None => Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'pull_probed_results'".to_string(),
            }),
        }
    }

    /// `end_probe_session` (`klippy/extras/smart_effector.py:102`).
    pub fn end_probe_session(&self) -> KResult<()> {
        match self.probe_session.replace(None) {
            Some(s) => s.end_probe_session(),
            None => Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'end_probe_session'".to_string(),
            }),
        }
    }

    /// `_send_command` (`klippy/extras/smart_effector.py:105`).
    pub fn send_command(&self, buf: &[i64]) -> KResult<()> {
        // Each byte is sent to the SmartEffector as
        // [0 0 1 0 b7 b6 b5 b4 !b4 b3 b2 b1 b0 !b0]
        let mut bit_stream: Vec<i64> = Vec::with_capacity(buf.len() * 14);
        for &b0 in buf {
            let b = b0 & 0xFF;
            bit_stream.extend_from_slice(&[0, 0, 1, 0]);
            bit_stream.push(b & 0x80);
            bit_stream.push(b & 0x40);
            bit_stream.push(b & 0x20);
            bit_stream.push(b & 0x10);
            bit_stream.push((!b) & 0x10);
            bit_stream.push(b & 0x08);
            bit_stream.push(b & 0x04);
            bit_stream.push(b & 0x02);
            bit_stream.push(b & 0x01);
            bit_stream.push((!b) & 0x01);
        }
        // Wait for previous actions to finish.
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        toolhead.wait_moves();
        let start_time = toolhead.get_last_move_time();
        // Write generated bits to the control pin.
        let pin = self.control_pin.clone().ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'NoneType' object has no attribute 'write_bits'".to_string(),
        })?;
        let end_time = pin.write_bits(start_time, &bit_stream)?;
        // Dwell to make sure no subsequent actions are queued together
        // with the SmartEffector programming.
        toolhead.dwell(end_time - start_time);
        toolhead.wait_moves();
        Ok(())
    }

    /// `cmd_SET_SMART_EFFECTOR` (`klippy/extras/smart_effector.py:127`).
    pub fn cmd_set_smart_effector(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let sensitivity = gcmd.get_int(
            "SENSITIVITY",
            GGetIntOpts {
                default: Some(None),
                minval: Some(0),
                maxval: Some(255),
            },
        )?;
        let mut respond_info: Vec<String> = Vec::new();
        if let Some(sens) = sensitivity {
            if self.control_pin.is_some() {
                self.send_command(&[105, sens, 255 - sens])?;
                respond_info.push(fmt("sensitivity: %d", &[pct_int(sens)]));
            } else {
                return Err(KlippyError::Command(
                    "control_pin must be set in [smart_effector] for sensitivity programming"
                        .to_string(),
                ));
            }
        }
        let accel = gcmd
            .get_float(
                "ACCEL",
                GGetFloatOpts {
                    default: Some(Some(self.probe_accel.get())),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("get_float with default never yields None");
        self.probe_accel.set(accel);
        let recovery = gcmd
            .get_float(
                "RECOVERY_TIME",
                GGetFloatOpts {
                    default: Some(Some(self.recovery_time.get())),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("get_float with default never yields None");
        self.recovery_time.set(recovery);
        if self.probe_accel.get() != 0.0 {
            respond_info.push(fmt(
                "probing accelartion: %.3f",
                &[pct_float(self.probe_accel.get())],
            ));
        } else {
            respond_info.push("probing acceleration control disabled".to_string());
        }
        if self.recovery_time.get() != 0.0 {
            respond_info.push(fmt(
                "probe recovery time: %.3f",
                &[pct_float(self.recovery_time.get())],
            ));
        } else {
            respond_info.push("probe recovery time disabled".to_string());
        }
        gcmd.respond_info(&format!("SmartEffector:\n{}", respond_info.join("\n")), true);
        Ok(())
    }

    /// `cmd_RESET_SMART_EFFECTOR` (`klippy/extras/smart_effector.py:153`).
    pub fn cmd_reset_smart_effector(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.send_command(&[131, 131])?;
        gcmd.respond_info("SmartEffector sensitivity was reset", true);
        Ok(())
    }

    pub fn probe_accel(&self) -> f64 {
        self.probe_accel.get()
    }

    pub fn recovery_time(&self) -> f64 {
        self.recovery_time.get()
    }
}

impl ProbeSessionApi for SmartEffectorProbe {
    fn run_probe(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        self.run_probe(gcmd)
    }

    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        self.pull_probed_results()
    }

    fn end_probe_session(&self) -> KResult<()> {
        self.end_probe_session()
    }
}

// ---------------------------------------------------------------------------
// PrinterSmartEffector (smart_effector.py:158-183)
// ---------------------------------------------------------------------------

/// The `probe` printer object for the `[smart_effector]` section.
///
/// It implements `crate::probe::PrinterProbeApi`, the trait
/// `ProbeCommandHelper` duck-types on for the `probe` object.
/// `PrinterSmartEffector` (`klippy/extras/smart_effector.py:158`).
pub struct PrinterSmartEffector {
    probe_offsets: Arc<ProbeOffsetsHelper>,
    param_helper: Arc<ProbeParameterHelper>,
    _mcu_probe: Arc<SmartEffectorProbe>,
    probe_session: Arc<SampleAveragingHelper>,
    cmd_helper: GCell<Option<Arc<ProbeCommandHelper>>>,
}

impl PrinterSmartEffector {
    /// `PrinterSmartEffector(config)` (`klippy/extras/smart_effector.py:159`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let probe_offsets = ProbeOffsetsHelper::new(config)?;
        let param_helper = ProbeParameterHelper::new(config)?;
        let mcu_probe = SmartEffectorProbe::new(
            config,
            Arc::clone(&probe_offsets),
            Arc::clone(&param_helper),
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
        // The Python object wires `query_endstop` bound methods and
        // `self` by reference; the Arc cycle stays alive for the
        // printer's lifetime like the Python reference cycle.
        let this = Arc::new(PrinterSmartEffector {
            probe_offsets,
            param_helper,
            _mcu_probe: mcu_probe.clone(),
            probe_session,
            cmd_helper: GCell::new(None),
        });
        let query_endstop = mcu_probe.query_endstop_cb();
        let weak_this: Weak<PrinterSmartEffector> = Arc::downgrade(&this);
        let weak: Weak<dyn PrinterProbeApi> = weak_this;
        let cmd_helper = ProbeCommandHelper::new(config, weak, Some(Arc::clone(&query_endstop)), true)?;
        this.cmd_helper.replace(Some(cmd_helper));
        let z_offset = this.probe_offsets.get_offsets(None).2;
        HomingViaProbeHelper::new(config, z_offset, Some(query_endstop))?;
        Ok(this)
    }

    /// `get_probe_params(gcmd=None)` (`klippy/extras/smart_effector.py:171`).
    pub fn get_probe_params(&self, gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/smart_effector.py:173`).
    pub fn get_offsets(&self, gcmd: Option<&klippy_api::gcode::GCmd>) -> (f64, f64, f64) {
        self.probe_offsets.get_offsets(gcmd)
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/smart_effector.py:177`).
    pub fn start_probe_session(
        &self,
        gcmd: &klippy_api::gcode::GCmd,
    ) -> KResult<Arc<SampleAveragingHelper>> {
        self.probe_session.start_probe_session(gcmd)
    }
}

impl PrinterProbeApi for PrinterSmartEffector {
    fn get_probe_params(&self, gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    fn get_offsets(&self, gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<(f64, f64, f64)> {
        Ok(self.probe_offsets.get_offsets(gcmd))
    }

    fn start_probe_session(
        &self,
        gcmd: &klippy_api::gcode::GCmd,
    ) -> KResult<Arc<dyn ProbeSessionApi>> {
        Ok(self.probe_session.start_probe_session(gcmd)? as Arc<dyn ProbeSessionApi>)
    }
}

impl PrinterObject for PrinterSmartEffector {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi);

    /// `get_status` (`klippy/extras/smart_effector.py:175`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        let helper = self.cmd_helper.cloned()?;
        Some(helper.get_status(eventtime))
    }
}

/// `load_config(config)` (`klippy/extras/smart_effector.py:180`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let smart_effector = PrinterSmartEffector::new(config)?;
    config.get_printer().add_object("probe", smart_effector.clone())?;
    Ok(Some(smart_effector as ObjectRef))
}

/// Module entry point for the `smart_effector` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
