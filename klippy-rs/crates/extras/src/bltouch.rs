//! Port of the `bltouch` module (`klippy/extras/bltouch.py`).
//!
//! The BLTouch probe drives the control pin with short PWM pulses
//! (`Commands`) and reads the sensor pin as an MCU endstop. Probe
//! session bookkeeping comes from the `probe` module helpers
//! (`ProbeOffsetsHelper`, `ProbeParameterHelper`,
//! `DescendToEndstopHelper`, `SampleAveragingHelper`,
//! `ProbeCommandHelper`, `HomingViaProbeHelper`,
//! `klippy/extras/probe.py`), used directly through their concrete
//! types.

use std::sync::{Arc, Weak};

use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCode, GCodeApi, GCodeCommandHandler, GCmd, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::mcu::{Endstop, EndstopApi, Mcu, Pin, PinsApi, PwmPin};
use klippy_api::object::ObjectRef;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::stepper::Stepper;
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, EventArg, PrinterObject};
use pycompat::text::{percent_format, str_repr, trunc_float_to_bigint, PctValue};
use pycompat::PyValue;
use reactor::{Completion, Reactor, NEVER, NOW};

use crate::manual_probe::ProbeResult;
use crate::probe::{
    DescendToEndstopHelper, HomingViaProbeHelper, PrinterProbeApi, ProbeCommandHelper,
    ProbeOffsetsHelper, ProbeOffsetsHelperApi, ProbeParameterHelper, ProbeParameterHelperApi,
    ProbeSessionApi, QueryEndstopCb,
};

// ---------------------------------------------------------------------------
// Constants (`klippy/extras/bltouch.py:9-23`)
// ---------------------------------------------------------------------------

pub const SIGNAL_PERIOD: f64 = 0.020;
pub const MIN_CMD_TIME: f64 = 5.0 * SIGNAL_PERIOD;
pub const TEST_TIME: f64 = 5.0 * 60.0;
pub const RETRY_RESET_TIME: f64 = 1.0;
pub const ENDSTOP_REST_TIME: f64 = 0.001;
pub const ENDSTOP_SAMPLE_TIME: f64 = 0.000015;
pub const ENDSTOP_SAMPLE_COUNT: i64 = 4;

pub const COMMANDS: &[(&str, f64)] = &[
    ("pin_down", 0.000650),
    ("touch_mode", 0.001165),
    ("pin_up", 0.001475),
    ("self_test", 0.001780),
    ("reset", 0.002190),
    ("set_5V_output_mode", 0.001988),
    ("set_OD_output_mode", 0.002091),
    ("output_mode_store", 0.001884),
];

pub const CMD_BLTOUCH_DEBUG_HELP: &str = "Send a command to the bltouch for debugging";
pub const CMD_BLTOUCH_STORE_HELP: &str = "Store an output mode in the BLTouch EEPROM";

fn command_duration(cmd: &str) -> Option<f64> {
    COMMANDS.iter().find(|(name, _)| *name == cmd).map(|(_, v)| *v)
}

fn sorted_command_names() -> Vec<String> {
    let mut names: Vec<String> = COMMANDS.iter().map(|(name, _)| (*name).to_string()).collect();
    names.sort_unstable();
    names
}

fn py_max(a: f64, b: f64) -> f64 {
    if b > a { b } else { a }
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

fn pct_str(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}

fn fmt_percent(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant bltouch format")
}

/// `int(float)` as `f64` for the `_send_cmd` pulse count.
fn float_to_f64(f: f64) -> f64 {
    trunc_float_to_bigint(f).to_f64_exact().unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// BLTouchProbe (`klippy/extras/bltouch.py:26-280`)
// ---------------------------------------------------------------------------

fn command_error(message: &str) -> KlippyError {
    KlippyError::Command(message.to_string())
}

fn key_error(cmd: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "KeyError".to_string(),
        message: str_repr(cmd),
    }
}

fn lookup_toolhead(printer: &Printer) -> KResult<Arc<dyn ToolHeadApi>> {
    let toolhead = lookup_interface::<dyn ToolHeadApi>(&**printer, "toolhead", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    Ok(toolhead)
}

fn lookup_gcode(printer: &Printer) -> KResult<GCode> {
    let g = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    Ok(g)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Multi {
    Off,
    First,
    On,
}

/// `klippy/extras/bltouch.py:26` `BLTouchProbe`.
pub struct BLTouchProbe {
    printer: Printer,
    gcode: GCode,
    stow_on_each_sample: bool,
    probe_touch_mode: bool,
    mcu_pwm: PwmPin,
    mcu_endstop: Endstop,
    output_mode: Option<String>,
    pin_up_not_triggered: bool,
    pin_up_touch_triggered: bool,
    pin_move_time: f64,
    next_cmd_time: GCell<f64>,
    action_end_time: GCell<f64>,
    finish_home_complete: GCell<Option<Completion<bool>>>,
    wait_trigger_complete: GCell<Option<Completion<()>>>,
    next_test_time: GCell<f64>,
    multi: GCell<Multi>,
    homing_helper: GCell<Option<DescendToEndstopHelper>>,
    reactor: Reactor,
    self_ref: Weak<BLTouchProbe>,
}

impl BLTouchProbe {
    /// `BLTouchProbe(config, probe_offsets, param_helper)`
    /// (`klippy/extras/bltouch.py:27-74`).
    pub fn new(
        config: &Config,
        probe_offsets: Arc<dyn ProbeOffsetsHelperApi>,
        param_helper: Arc<dyn ProbeParameterHelperApi>,
    ) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let stow_on_each_sample = config
            .getboolean(
                "stow_on_each_sample",
                GetBoolOpts { default: Some(Some(true)), note_valid: true },
            )?
            .expect("stow_on_each_sample has a default");
        let probe_touch_mode = config
            .getboolean(
                "probe_with_touch_mode",
                GetBoolOpts { default: Some(Some(false)), note_valid: true },
            )?
            .expect("probe_with_touch_mode has a default");
        let ppins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let control_pin = config
            .get("control_pin", GetStrOpts::default())?
            .expect("control_pin required");
        let mcu_pwm = match ppins.setup_pin("pwm", &control_pin)? {
            Pin::Pwm(pin) => pin,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "setup_pin('pwm') did not return a PWM pin".to_string(),
                })
            }
        };
        mcu_pwm.setup_max_duration(0.0);
        mcu_pwm.setup_cycle_time(SIGNAL_PERIOD, None);
        let sensor_pin = config
            .get("sensor_pin", GetStrOpts::default())?
            .expect("sensor_pin required");
        let mcu_endstop = match ppins.setup_pin("endstop", &sensor_pin)? {
            Pin::Endstop(pin) => pin,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "setup_pin('endstop') did not return an endstop pin".to_string(),
                })
            }
        };
        let mut omode_choices = indexmap::IndexMap::new();
        omode_choices.insert(PyValue::Str("5V".to_string()), PyValue::Str("5V".to_string()));
        omode_choices.insert(PyValue::Str("OD".to_string()), PyValue::Str("OD".to_string()));
        omode_choices.insert(PyValue::None, PyValue::None);
        let output_mode = match config.getchoice(
            "set_output_mode",
            &PyValue::Dict(omode_choices),
            klippy_api::config::GetChoiceOpts { default: Some(None), note_valid: true },
        )? {
            PyValue::Str(mode) => Some(mode),
            PyValue::None => None,
            _ => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "set_output_mode did not return a string".to_string(),
                })
            }
        };
        let pin_up_not_triggered = config
            .getboolean(
                "pin_up_reports_not_triggered",
                GetBoolOpts { default: Some(Some(true)), note_valid: true },
            )?
            .expect("pin_up_reports_not_triggered has a default");
        let pin_up_touch_triggered = config
            .getboolean(
                "pin_up_touch_mode_reports_triggered",
                GetBoolOpts { default: Some(Some(true)), note_valid: true },
            )?
            .expect("pin_up_touch_mode_reports_triggered has a default");
        let pin_move_time = config
            .getfloat(
                "pin_move_time",
                GetFloatOpts {
                    default: Some(Some(0.680)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                },
            )?
            .expect("pin_move_time has a default");
        let gcode = lookup_gcode(&printer)?;
        let reactor = printer.get_reactor();
        let this = Arc::new_cyclic(|weak| BLTouchProbe {
            printer: printer.clone(),
            gcode: gcode.clone(),
            stow_on_each_sample,
            probe_touch_mode,
            mcu_pwm: mcu_pwm.clone(),
            mcu_endstop: mcu_endstop.clone(),
            output_mode: output_mode.clone(),
            pin_up_not_triggered,
            pin_up_touch_triggered,
            pin_move_time,
            next_cmd_time: GCell::new(0.0),
            action_end_time: GCell::new(0.0),
            finish_home_complete: GCell::new(None),
            wait_trigger_complete: GCell::new(None),
            next_test_time: GCell::new(0.0),
            multi: GCell::new(Multi::Off),
            homing_helper: GCell::new(None),
            reactor: reactor.clone(),
            self_ref: weak.clone(),
        });
        // Probing via homing to endstop (`always_check_movement=True`).
        let helper = DescendToEndstopHelper::new(
            config,
            Arc::clone(&this) as Endstop,
            probe_offsets,
            param_helper,
            true,
        )?;
        this.homing_helper.replace(Some(helper));
        {
            let this_cb = Arc::clone(&this);
            let handler: GCodeCommandHandler = Box::new(move |gcmd: &GCmd| {
                this_cb.cmd_bltouch_debug(gcmd)
            });
            gcode.register_command(
                "BLTOUCH_DEBUG",
                Some(handler),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_BLTOUCH_DEBUG_HELP.to_string()),
                },
            )?;
            let this_store = Arc::clone(&this);
            let store_handler: GCodeCommandHandler =
                Box::new(move |gcmd: &GCmd| this_store.cmd_bltouch_store(gcmd));
            gcode.register_command(
                "BLTOUCH_STORE",
                Some(store_handler),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(CMD_BLTOUCH_STORE_HELP.to_string()),
                },
            )?;
            let connect_weak = Weak::clone(&this.self_ref);
            printer.register_event_handler(
                EV_KLIPPY_CONNECT,
                Box::new(move |_params: &[EventArg]| {
                    let Some(this) = connect_weak.upgrade() else {
                        return Ok(PyValue::None);
                    };
                    this.handle_connect()?;
                    Ok(PyValue::None)
                }),
            );
        }
        Ok(this)
    }

    fn with_homing<R>(&self, method: &str, f: impl FnOnce(&DescendToEndstopHelper) -> R) -> KResult<R> {
        let result = self.homing_helper.borrow().as_ref().map(f);
        result.ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: format!("'NoneType' object has no attribute '{}'", method),
        })
    }

    /// `query_endstop` (`mcu_endstop.query_endstop`).
    pub fn query_endstop(&self) -> QueryEndstopCb {
        let mcu_endstop = Arc::clone(&self.mcu_endstop);
        Arc::new(move |print_time| mcu_endstop.query_endstop(print_time))
    }

    /// `_handle_connect` (`klippy/extras/bltouch.py:75-83`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.sync_mcu_print_time();
        self.next_cmd_time.with_mut(|t| *t += 0.200);
        self.set_output_mode(self.output_mode.as_deref())?;
        match self.raise_probe().and_then(|()| self.verify_raise_probe()) {
            Ok(()) => Ok(()),
            Err(e) if matches!(e, KlippyError::Command(_)) => {
                log::warning(&fmt_percent(
                    "BLTouch raise probe error: %s",
                    &[PctValue::Str(e.message().to_string())],
                ));
                Ok(())
            }
            Err(e) => Err(e),
        }
    }

    /// `_sync_mcu_print_time` (`klippy/extras/bltouch.py:84-87`).
    pub fn sync_mcu_print_time(&self) {
        let curtime = self.reactor.monotonic();
        let est_time = self.mcu_pwm.get_mcu().estimated_print_time(curtime);
        let next = self.next_cmd_time.get();
        self.next_cmd_time.set(py_max(next, est_time + MIN_CMD_TIME));
    }

    /// `_sync_print_time` (`klippy/extras/bltouch.py:88-94`).
    pub fn sync_print_time(&self) -> KResult<()> {
        let toolhead = lookup_toolhead(&self.printer)?;
        let print_time = toolhead.get_last_move_time()?;
        let next = self.next_cmd_time.get();
        if next > print_time {
            toolhead.dwell(next - print_time)?;
        } else {
            self.next_cmd_time.set(print_time);
        }
        Ok(())
    }

    /// `_send_cmd` (`klippy/extras/bltouch.py:95-107`).
    pub fn send_cmd(&self, cmd: &str) -> KResult<()> {
        self.send_cmd_with_duration(cmd, MIN_CMD_TIME)
    }

    /// `_send_cmd(cmd, duration)` with an explicit duration.
    pub fn send_cmd_with_duration(&self, cmd: &str, duration: f64) -> KResult<()> {
        let Some(cmd_value) = command_duration(cmd) else {
            return Err(key_error(cmd));
        };
        let mcu = self.mcu_pwm.get_mcu();
        let next = self.next_cmd_time.get();
        let mut cmd_clock = mcu.print_time_to_clock(next);
        let pulse = float_to_f64((duration - MIN_CMD_TIME) / SIGNAL_PERIOD) * SIGNAL_PERIOD;
        cmd_clock += mcu.seconds_to_clock(py_max(MIN_CMD_TIME, pulse));
        let end_time = mcu.clock_to_print_time(cmd_clock as f64);
        self.mcu_pwm.set_pwm(next, cmd_value / SIGNAL_PERIOD);
        self.mcu_pwm.set_pwm(end_time, 0.0);
        let action_end = next + duration;
        self.action_end_time.set(action_end);
        self.next_cmd_time.set(py_max(action_end, end_time + MIN_CMD_TIME));
        Ok(())
    }

    /// `_verify_state` (`klippy/extras/bltouch.py:108-118`).
    pub fn verify_state(&self, triggered: bool) -> KResult<bool> {
        let action_end = self.action_end_time.get();
        self.mcu_endstop.home_start(
            action_end,
            ENDSTOP_SAMPLE_TIME,
            ENDSTOP_SAMPLE_COUNT,
            ENDSTOP_REST_TIME,
            Some(triggered),
        );
        match self.mcu_endstop.home_wait(action_end + 0.100) {
            Ok(trigger_time) => Ok(trigger_time > 0.0),
            Err(KlippyError::Command(_)) => Ok(false),
            Err(e) => Err(e),
        }
    }

    /// `_raise_probe` (`klippy/extras/bltouch.py:119-123`).
    pub fn raise_probe(&self) -> KResult<()> {
        self.sync_mcu_print_time();
        if !self.pin_up_not_triggered {
            self.send_cmd("reset")?;
        }
        self.send_cmd_with_duration("pin_up", self.pin_move_time)
    }

    /// `_verify_raise_probe` (`klippy/extras/bltouch.py:124-140`).
    pub fn verify_raise_probe(&self) -> KResult<()> {
        if !self.pin_up_not_triggered {
            return Ok(());
        }
        for retry in 0..3 {
            let success = self.verify_state(false)?;
            if success {
                return Ok(());
            }
            if retry >= 2 {
                return Err(command_error("BLTouch failed to raise probe"));
            }
            self.gcode.respond_info(
                "Failed to verify BLTouch probe is raised; retrying.",
                true,
            );
            self.sync_mcu_print_time();
            self.send_cmd_with_duration("reset", RETRY_RESET_TIME)?;
            self.send_cmd_with_duration("pin_up", self.pin_move_time)?;
        }
        Ok(())
    }

    /// `_lower_probe` (`klippy/extras/bltouch.py:141-146`).
    pub fn lower_probe(&self) -> KResult<()> {
        self.test_sensor()?;
        self.sync_print_time()?;
        self.send_cmd_with_duration("pin_down", self.pin_move_time)?;
        if self.probe_touch_mode {
            self.send_cmd("touch_mode")?;
        }
        Ok(())
    }

    /// `_test_sensor` (`klippy/extras/bltouch.py:147-171`).
    pub fn test_sensor(&self) -> KResult<()> {
        if !self.pin_up_touch_triggered {
            return Ok(());
        }
        let toolhead = lookup_toolhead(&self.printer)?;
        let print_time = toolhead.get_last_move_time()?;
        let next_test = self.next_test_time.get();
        if print_time < next_test {
            self.next_test_time.set(print_time + TEST_TIME);
            return Ok(());
        }
        self.sync_print_time()?;
        for retry in 0..3 {
            self.send_cmd_with_duration("pin_up", self.pin_move_time)?;
            self.send_cmd("touch_mode")?;
            let success = self.verify_state(true)?;
            self.sync_print_time()?;
            if success {
                self.next_test_time.set(print_time + TEST_TIME);
                return Ok(());
            }
            let msg = "BLTouch failed to verify sensor state";
            if retry >= 2 {
                return Err(command_error(msg));
            }
            self.gcode.respond_info(&fmt_percent("%s; retrying.", &[pct_str(msg)]), true);
            self.send_cmd_with_duration("reset", RETRY_RESET_TIME)?;
        }
        Ok(())
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/bltouch.py:173-177`):
    /// returns the probe itself.
    pub fn start_probe_session(&self, _gcmd: &GCmd) -> KResult<Arc<Self>> {
        self.with_homing("clear_trigger_positions", |helper| helper.clear_trigger_positions())?;
        if !self.stow_on_each_sample {
            self.multi.set(Multi::First);
        }
        Ok(self
            .self_ref
            .upgrade()
            .expect("BLTouchProbe outlives its session calls"))
    }

    /// `_probe_prepare` (`klippy/extras/bltouch.py:178-183`).
    pub fn probe_prepare(&self) -> KResult<()> {
        let multi = self.multi.get();
        if multi == Multi::Off || multi == Multi::First {
            self.lower_probe()?;
            if self.multi.get() == Multi::First {
                self.multi.set(Multi::On);
            }
        }
        self.sync_print_time()
    }

    /// `home_start` (`klippy/extras/bltouch.py:184-192`) as `EndstopApi`.
    /// The reactor callback registration lives in
    /// [`EndstopApi::home_start`] below.
    fn home_start_impl(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: bool,
    ) -> Completion<bool> {
        let rest = py_min(rest_time, ENDSTOP_REST_TIME);
        let finish = self.mcu_endstop.home_start(
            print_time,
            sample_time,
            sample_count,
            rest,
            Some(triggered),
        );
        self.finish_home_complete.replace(Some(finish.clone()));
        let weak = Weak::clone(&self.self_ref);
        let reactor = self.reactor.clone();
        let wait = reactor.register_callback(
            move |eventtime| {
                if let Some(this) = weak.upgrade() {
                    this.wait_for_trigger(eventtime);
                }
            },
            NOW,
        );
        self.wait_trigger_complete.replace(Some(wait));
        finish
    }

    /// `_wait_for_trigger` (`klippy/extras/bltouch.py:193-196`).
    pub fn wait_for_trigger(&self, _eventtime: f64) {
        if let Some(finish) = self.finish_home_complete.cloned() {
            finish.wait(NEVER);
        }
        if self.multi.get() == Multi::Off {
            let _ = self.raise_probe();
        }
    }

    /// `_probe_finish` (`klippy/extras/bltouch.py:197-201`).
    pub fn probe_finish(&self) -> KResult<()> {
        let Some(wait) = self.wait_trigger_complete.cloned() else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'wait'".to_string(),
            });
        };
        wait.wait(NEVER);
        if self.multi.get() == Multi::Off {
            self.verify_raise_probe()?;
        }
        self.sync_print_time()
    }

    /// `_set_output_mode` (`klippy/extras/bltouch.py:221-234`).
    pub fn set_output_mode(&self, mode: Option<&str>) -> KResult<()> {
        let Some(mode) = mode else {
            return Ok(());
        };
        log::info(&fmt_percent("BLTouch set output mode: %s", &[pct_str(mode)]));
        self.sync_mcu_print_time();
        if mode == "5V" {
            self.send_cmd("set_5V_output_mode")?;
        }
        if mode == "OD" {
            self.send_cmd("set_OD_output_mode")?;
        }
        Ok(())
    }

    /// `_store_output_mode` (`klippy/extras/bltouch.py:235-257`).
    pub fn store_output_mode(&self, mode: &str) -> KResult<()> {
        log::info(&fmt_percent("BLTouch store output mode: %s", &[pct_str(mode)]));
        self.sync_print_time()?;
        self.send_cmd("pin_down")?;
        if mode == "5V" {
            self.send_cmd("set_5V_output_mode")?;
        } else {
            self.send_cmd("set_OD_output_mode")?;
        }
        self.send_cmd("output_mode_store")?;
        if mode == "5V" {
            self.send_cmd("set_5V_output_mode")?;
        } else {
            self.send_cmd("set_OD_output_mode")?;
        }
        self.send_cmd("pin_up")
    }

    /// `cmd_BLTOUCH_DEBUG` (`klippy/extras/bltouch.py:259-268`).
    pub fn cmd_bltouch_debug(&self, gcmd: &GCmd) -> KResult<()> {
        let cmd_value = gcmd.get(
            "COMMAND",
            GetOpts { default: Some(PyValue::None), ..GetOpts::default() },
        )?;
        let cmd = match cmd_value {
            PyValue::None => None,
            PyValue::Str(s) => Some(s),
            other => Some(other.str()),
        };
        if cmd.as_ref().is_none_or(|c| command_duration(c).is_none()) {
            let names = sorted_command_names().join(", ");
            gcmd.respond_info(
                &fmt_percent("BLTouch commands: %s", &[pct_str(&names)]),
                true,
            );
            return Ok(());
        }
        let cmd = cmd.expect("checked above");
        gcmd.respond_info(
            &fmt_percent("Sending BLTOUCH_DEBUG COMMAND=%s", &[pct_str(&cmd)]),
            true,
        );
        self.sync_print_time()?;
        self.send_cmd_with_duration(&cmd, self.pin_move_time)?;
        self.sync_print_time()
    }

    /// `cmd_BLTOUCH_STORE` (`klippy/extras/bltouch.py:270-278`).
    pub fn cmd_bltouch_store(&self, gcmd: &GCmd) -> KResult<()> {
        let cmd_value = gcmd.get(
            "MODE",
            GetOpts { default: Some(PyValue::None), ..GetOpts::default() },
        )?;
        let cmd = match cmd_value {
            PyValue::None => None,
            PyValue::Str(s) => Some(s),
            other => Some(other.str()),
        };
        if !cmd.as_ref().map(|c| c == "5V" || c == "OD").unwrap_or(false) {
            gcmd.respond_info("BLTouch output modes: 5V, OD", true);
            return Ok(());
        }
        let cmd = cmd.expect("checked above");
        gcmd.respond_info(
            &fmt_percent("Storing BLTouch output mode: %s", &[pct_str(&cmd)]),
            true,
        );
        self.sync_print_time()?;
        self.store_output_mode(&cmd)?;
        self.sync_print_time()
    }
}


impl EndstopApi for BLTouchProbe {
    fn get_mcu(&self) -> Mcu {
        self.mcu_endstop.get_mcu()
    }
    fn add_stepper(&self, stepper: Stepper) {
        self.mcu_endstop.add_stepper(stepper);
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.mcu_endstop.get_steppers()
    }
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: Option<bool>,
    ) -> Completion<bool> {
        self.home_start_impl(
            print_time,
            sample_time,
            sample_count,
            rest_time,
            triggered.unwrap_or(true),
        )
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.mcu_endstop.home_wait(home_end_time)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        self.mcu_endstop.query_endstop(print_time)
    }
}

impl ProbeSessionApi for BLTouchProbe {
    /// `run_probe(gcmd)` (`klippy/extras/bltouch.py:202-209`).
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        self.probe_prepare()?;
        let descend = self.with_homing("descend_until_trigger", |helper| helper.descend_until_trigger(gcmd))?;
        match descend {
            Ok(()) => self.probe_finish(),
            Err(e @ KlippyError::Command(_)) => {
                self.probe_finish()?;
                Err(e)
            }
            Err(e) => Err(e),
        }
    }
    /// `pull_probed_results()` (`klippy/extras/bltouch.py:210-211`).
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        self.with_homing("pull_trigger_positions", |helper| helper.pull_trigger_positions())
    }
    /// `end_probe_session()` (`klippy/extras/bltouch.py:212-219`).
    fn end_probe_session(&self) -> KResult<()> {
        self.with_homing("clear_trigger_positions", |helper| helper.clear_trigger_positions())?;
        if !self.stow_on_each_sample {
            self.sync_print_time()?;
            self.raise_probe()?;
            self.verify_raise_probe()?;
            self.sync_print_time()?;
            self.multi.set(Multi::Off);
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// PrinterBLTouch (`klippy/extras/bltouch.py:281-306`)
// ---------------------------------------------------------------------------

/// `klippy/extras/bltouch.py:281` `PrinterBLTouch`.
pub struct PrinterBLTouch {
    probe_offsets: Arc<ProbeOffsetsHelper>,
    param_helper: Arc<ProbeParameterHelper>,
    mcu_probe: Arc<BLTouchProbe>,
    probe_session: Arc<crate::probe::SampleAveragingHelper>,
    cmd_helper: GCell<Option<Arc<ProbeCommandHelper>>>,
}

impl PrinterBLTouch {
    /// `PrinterBLTouch(config)` (`klippy/extras/bltouch.py:282-293`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let probe_offsets = ProbeOffsetsHelper::new(config)?;
        let param_helper = ProbeParameterHelper::new(config)?;
        let mcu_probe = BLTouchProbe::new(
            config,
            Arc::clone(&probe_offsets) as Arc<dyn ProbeOffsetsHelperApi>,
            Arc::clone(&param_helper) as Arc<dyn ProbeParameterHelperApi>,
        )?;
        let probe_session = crate::probe::SampleAveragingHelper::new(
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
        let this = Arc::new(PrinterBLTouch {
            probe_offsets,
            param_helper,
            mcu_probe,
            probe_session,
            cmd_helper: GCell::new(None),
        });
        let weak_this: Weak<PrinterBLTouch> = Arc::downgrade(&this);
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

    /// `get_probe_params(gcmd=None)`.
    pub fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<crate::probe::ProbeParams> {
        self.param_helper.get_probe_params(gcmd)
    }

    /// `get_offsets(gcmd=None)`.
    pub fn get_offsets(&self, gcmd: Option<&GCmd>) -> (f64, f64, f64) {
        self.probe_offsets.get_offsets(gcmd)
    }

    /// `get_status(eventtime)`.
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        let helper = self
            .cmd_helper
            .cloned()
            .expect("cmd_helper set before get_status");
        helper.get_status(eventtime)
    }

    /// `start_probe_session(gcmd)`.
    pub fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        Ok(self.probe_session.start_probe_session(gcmd)? as Arc<dyn ProbeSessionApi>)
    }

    /// The `mcu_probe` attribute (for tests).
    pub fn mcu_probe(&self) -> &Arc<BLTouchProbe> {
        &self.mcu_probe
    }
}

impl PrinterProbeApi for PrinterBLTouch {
    /// `get_probe_params(gcmd=None)` (`klippy/extras/bltouch.py:294`).
    fn get_probe_params(&self, gcmd: Option<&GCmd>) -> KResult<crate::probe::ProbeParams> {
        PrinterBLTouch::get_probe_params(self, gcmd)
    }

    /// `get_offsets(gcmd=None)` (`klippy/extras/bltouch.py:296`).
    fn get_offsets(&self, gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        Ok(PrinterBLTouch::get_offsets(self, gcmd))
    }

    /// `start_probe_session(gcmd)` (`klippy/extras/bltouch.py:300`).
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        PrinterBLTouch::start_probe_session(self, gcmd)
    }
}

impl PrinterObject for PrinterBLTouch {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterProbeApi);

    /// `get_status(eventtime)` (`klippy/extras/bltouch.py:298`).
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(PrinterBLTouch::get_status(self, eventtime))
    }
}

/// `load_config(config)` (`klippy/extras/bltouch.py:303-306`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let blt = PrinterBLTouch::new(config)?;
    let obj_ref: ObjectRef = Arc::clone(&blt) as ObjectRef;
    config.get_printer().add_object("probe", obj_ref.clone())?;
    Ok(Some(obj_ref))
}

/// Module entry point for the `bltouch` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
