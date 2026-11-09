//! Port of the `pwm_tool` module (`klippy/extras/pwm_tool.py`).
//!
//! Queued PWM GPIO output: [`McuQueuedPwm`] builds the MCU config
//! commands (hardware `config_pwm_out`/`queue_pwm_out` or software
//! `config_digital_out`/`set_digital_out_pwm_cycle`/`queue_digital_out`)
//! and pushes updates through a `motion_queuing` syncemitter.
//! [`PrinterOutputPin`] reads the section config and registers `SET_PIN`.

use std::sync::Arc;

use klippy_api::config::{
    Config, GetBoolOpts, GetFloatOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GetFloatOpts as GcodeGetFloatOpts, GCodeApi, GCmd};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{AddConfigCmdOpts, LookupPinOpts, Mcu, PinParams, PinsApi};
use klippy_api::object::load_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{FlushCallback, MotionQueuingApi, SyncEmitterHandle, ToolHeadApi};
use klippy_api::{lookup_interface, ObjectRef, PrinterObject};
use pycompat::PyValue;

// ---------------------------------------------------------------------------
// Python numeric helpers (max/min, int(float), == for int/float mixes).
// ---------------------------------------------------------------------------

fn py_max(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

fn py_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

fn clamp_0_1(v: f64) -> f64 {
    py_max(0.0, py_min(1.0, v))
}

/// Python `int(float)` truncation. Callers only use finite values; the
/// debug assert mirrors `pycompat::text::trunc_float_to_bigint`.
fn float_to_int(f: f64) -> i64 {
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits.parse::<i64>().expect("pwm_tool int value out of i64 range")
}

fn int_eq_float(i: i64, f: f64) -> bool {
    pycompat::value::py_equal(
        &PyValue::Int(pycompat::int::PyInt::from_i64(i)),
        &PyValue::Float(f),
    )
}

/// Default value: hardware stores `shutdown * pwm_max` (float), software
/// stores `int(shutdown >= 0.5) * cycle_ticks` (int). Kept distinct so the
/// `last_value == default_value` comparisons match CPython's int/float
/// equality exactly.
#[derive(Clone, Copy, Debug)]
enum DefaultValue {
    Int(i64),
    Float(f64),
}

impl DefaultValue {
    fn equals_int(&self, v: i64) -> bool {
        match *self {
            DefaultValue::Int(d) => d == v,
            DefaultValue::Float(f) => int_eq_float(v, f),
        }
    }
}

/// Formatter for a `pycompat` `%`-format argument that is a plain integer.
fn pct_int(v: i64) -> pycompat::text::PctValue {
    pycompat::text::PctValue::Int(pycompat::int::PyInt::from_i64(v))
}

fn format_or_internal(fmt: &str, args: &[pycompat::text::PctValue]) -> KResult<String> {
    pycompat::text::percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

// ---------------------------------------------------------------------------
// MCU_queued_pwm
// ---------------------------------------------------------------------------

struct QueuedPwmState {
    hardware_pwm: bool,
    cycle_time: f64,
    max_duration: f64,
    oid: i64,
    start_value: f64,
    shutdown_value: f64,
    last_clock: i64,
    last_value: i64,
    default_value: DefaultValue,
    duration_ticks: i64,
    pwm_max: f64,
    set_cmd_tag: Option<i64>,
    toolhead: Option<klippy_api::toolhead::ToolHead>,
}

/// `klippy/extras/pwm_tool.py:11` `MCU_queued_pwm`.
pub struct McuQueuedPwm {
    mcu: Mcu,
    motion_queuing: klippy_api::toolhead::MotionQueuing,
    syncemitter: SyncEmitterHandle,
    pin: String,
    invert: bool,
    state: GCell<QueuedPwmState>,
}

impl McuQueuedPwm {
    /// `MCU_queued_pwm.__init__(config, pin_params)` (`klippy/extras/pwm_tool.py:12`).
    pub fn new(config: &Config, pin_params: PinParams) -> KResult<Arc<Self>> {
        let mcu = pin_params.chip.clone();
        let pin = pin_params.pin.clone();
        let invert = pin_params.invert != 0;
        let oid = mcu.create_oid()?;
        let printer = mcu.get_printer();
        let motion_queuing = load_interface::<dyn MotionQueuingApi>(
            &*printer,
            config,
            "motion_queuing",
            LookupDefault::Raise,
        )?
        .expect("motion_queuing loaded");
        let syncemitter =
            motion_queuing.allocate_syncemitter(mcu.clone(), &config.get_name(), false)?;
        let start0 = if invert { 1.0 } else { 0.0 };
        let state = GCell::new(QueuedPwmState {
            hardware_pwm: false,
            cycle_time: 0.100,
            max_duration: 2.0,
            oid,
            start_value: start0,
            shutdown_value: start0,
            last_clock: 0,
            last_value: 0,
            default_value: DefaultValue::Int(0),
            duration_ticks: 0,
            pwm_max: 0.0,
            set_cmd_tag: None,
            toolhead: None,
        });
        let this = Arc::new(Self {
            mcu: mcu.clone(),
            motion_queuing,
            syncemitter,
            pin,
            invert,
            state,
        });
        let for_config = Arc::clone(&this);
        mcu.register_config_callback(Box::new(move || for_config.build_config()))?;
        let for_connect = Arc::clone(&this);
        let printer_for_connect = printer.clone();
        printer.register_event_handler(
            "klippy:connect",
            Box::new(move |_params: &[klippy_api::EventArg]| {
                let toolhead = lookup_interface::<dyn ToolHeadApi>(
                    &*printer_for_connect,
                    "toolhead",
                    LookupDefault::Raise,
                )?
                .expect("toolhead present");
                for_connect.state.borrow_mut().toolhead = Some(toolhead);
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// `get_mcu` (`klippy/extras/pwm_tool.py:36`).
    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `setup_max_duration` (`klippy/extras/pwm_tool.py:38`).
    pub fn setup_max_duration(&self, max_duration: f64) {
        self.state.borrow_mut().max_duration = max_duration;
    }

    /// `setup_cycle_time` (`klippy/extras/pwm_tool.py:40`).
    pub fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: bool) {
        let mut s = self.state.borrow_mut();
        s.cycle_time = cycle_time;
        s.hardware_pwm = hardware_pwm;
    }

    /// `setup_start_value` (`klippy/extras/pwm_tool.py:43`).
    pub fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let invert = self.invert;
        let mut start_value = start_value;
        let mut shutdown_value = shutdown_value;
        if invert {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        let mut s = self.state.borrow_mut();
        s.start_value = clamp_0_1(start_value);
        s.shutdown_value = clamp_0_1(shutdown_value);
    }

    /// `_build_config` (`klippy/extras/pwm_tool.py:49`).
    pub fn build_config(self: &Arc<Self>) -> KResult<()> {
        let (max_duration, start_value, shutdown_value, cycle_time, hardware_pwm) = {
            let s = self.state.borrow();
            (
                s.max_duration,
                s.start_value,
                s.shutdown_value,
                s.cycle_time,
                s.hardware_pwm,
            )
        };
        if max_duration != 0.0 && start_value != shutdown_value {
            return Err(KlippyError::config(
                "Pin with max duration must have start value equal to shutdown value",
            ));
        }
        let cmd_queue = self.mcu.alloc_command_queue();
        let printer = self.mcu.get_printer();
        let curtime = printer.get_reactor().monotonic();
        let printtime = self.mcu.estimated_print_time(curtime);
        let new_last_clock = self.mcu.print_time_to_clock(printtime + 0.200);
        self.state.borrow_mut().last_clock = new_last_clock;
        let cycle_ticks = self.mcu.seconds_to_clock(cycle_time);
        if cycle_ticks >= 1 << 31 {
            return Err(KlippyError::config("PWM pin cycle time too large"));
        }
        let duration_ticks = self.mcu.seconds_to_clock(max_duration);
        if duration_ticks >= 1 << 31 {
            return Err(KlippyError::config("PWM pin max duration too large"));
        }
        self.state.borrow_mut().duration_ticks = duration_ticks;
        if duration_ticks != 0 {
            let mq = self.motion_queuing.clone();
            let this = Arc::clone(self);
            let cb: FlushCallback =
                Arc::new(move |must_flush_time: f64, max_step_gen_time: f64| {
                    this.flush_notification(must_flush_time, max_step_gen_time);
                    Ok(())
                });
            // `register_flush_callback(self._flush_notification)`
            // (`pwm_tool.py:66-67`): the reported name is the bound
            // method's `__qualname__` (`motion_queuing.py:35-42`).
            mq.register_flush_callback_named(cb, false, Some("_flush_notification"), None);
        }
        let oid = self.state.borrow().oid;
        if hardware_pwm {
            let pwm_max = self.mcu.get_constant_float("PWM_MAX")?;
            let default_value = shutdown_value * pwm_max;
            {
                let mut s = self.state.borrow_mut();
                s.pwm_max = pwm_max;
                s.default_value = DefaultValue::Float(default_value);
            }
            let value_arg = start_value * pwm_max;
            let cmd = format_or_internal(
                "config_pwm_out oid=%d pin=%s cycle_ticks=%d value=%d default_value=%d max_duration=%d",
                &[
                    pct_int(oid),
                    pycompat::text::PctValue::Str(self.pin.clone()),
                    pct_int(cycle_ticks),
                    pycompat::text::PctValue::Float(value_arg),
                    pycompat::text::PctValue::Float(default_value),
                    pct_int(duration_ticks),
                ],
            )?;
            self.mcu
                .add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
            let last_value = float_to_int(start_value * pwm_max + 0.5);
            let last_clock = self.state.borrow().last_clock;
            self.state.borrow_mut().last_value = last_value;
            let cmd = format_or_internal(
                "queue_pwm_out oid=%d clock=%d value=%d",
                &[pct_int(oid), pct_int(last_clock), pct_int(last_value)],
            )?;
            self.mcu.add_config_cmd(
                &cmd,
                AddConfigCmdOpts {
                    is_init: false,
                    on_restart: true,
                },
            )?;
            let tag = self
                .mcu
                .lookup_command("queue_pwm_out oid=%c clock=%u value=%hu", Some(cmd_queue))?
                .get_command_tag();
            self.state.borrow_mut().set_cmd_tag = Some(tag);
            return Ok(());
        }
        // Software PWM.
        if shutdown_value != 0.0 && shutdown_value != 1.0 {
            return Err(KlippyError::config(
                "shutdown value must be 0.0 or 1.0 on soft pwm",
            ));
        }
        let start_bit: i64 = if start_value >= 1.0 { 1 } else { 0 };
        let shutdown_bit: i64 = if shutdown_value >= 0.5 { 1 } else { 0 };
        let cmd = format_or_internal(
            "config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
            &[
                pct_int(oid),
                pycompat::text::PctValue::Str(self.pin.clone()),
                pct_int(start_bit),
                pct_int(shutdown_bit),
                pct_int(duration_ticks),
            ],
        )?;
        self.mcu
            .add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let default_value = DefaultValue::Int(
            if shutdown_value >= 0.5 { 1 } else { 0 } * cycle_ticks,
        );
        {
            let mut s = self.state.borrow_mut();
            s.default_value = default_value;
        }
        let cmd = format_or_internal(
            "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d",
            &[pct_int(oid), pct_int(cycle_ticks)],
        )?;
        self.mcu
            .add_config_cmd(&cmd, AddConfigCmdOpts::default())?;
        let pwm_max = cycle_ticks as f64;
        let last_value = float_to_int(start_value * pwm_max + 0.5);
        {
            let mut s = self.state.borrow_mut();
            s.pwm_max = pwm_max;
            s.last_value = last_value;
        }
        let last_clock = self.state.borrow().last_clock;
        let cmd = format_or_internal(
            "queue_digital_out oid=%d clock=%d on_ticks=%d",
            &[pct_int(oid), pct_int(last_clock), pct_int(last_value)],
        )?;
        self.mcu.add_config_cmd(
            &cmd,
            AddConfigCmdOpts {
                is_init: true,
                on_restart: false,
            },
        )?;
        let tag = self
            .mcu
            .lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?
            .get_command_tag();
        self.state.borrow_mut().set_cmd_tag = Some(tag);
        Ok(())
    }

    /// `_send_update` (`klippy/extras/pwm_tool.py:106`).
    pub fn send_update(&self, clock: i64, val: i64) {
        let (set_cmd_tag, oid, duration_ticks, default_value) = {
            let s = self.state.borrow();
            (s.set_cmd_tag.unwrap_or(0), s.oid, s.duration_ticks, s.default_value)
        };
        let clock = {
            let mut s = self.state.borrow_mut();
            s.last_clock = if s.last_clock > clock {
                s.last_clock
            } else {
                clock
            };
            s.last_clock
        };
        self.state.borrow_mut().last_value = val;
        let clock32 = clock & 0xffff_ffff;
        // `syncemitter_queue_msg(se, clock, data, len)` (pwm_tool.py:110),
        // with data = (tag, oid, clock & 0xffffffff, val); cffi converts
        // each Python int to `uint32_t`.
        self.motion_queuing.queue_syncemitter_msg(
            &self.syncemitter,
            clock as u64,
            &[
                set_cmd_tag as u32,
                oid as u32,
                clock32 as u32,
                val as u32,
            ],
        );
        let mut wakeclock = clock;
        let last_value = self.state.borrow().last_value;
        if !default_value.equals_int(last_value) {
            wakeclock = wakeclock.wrapping_add(duration_ticks);
        }
        let wake_print_time = self.mcu.clock_to_print_time(wakeclock as f64);
        // Avoid holding a state borrow across this call (GCell rule 2);
        // every value was copied out above.
        self.motion_queuing
            .note_mcu_movequeue_activity(wake_print_time, false);
    }

    /// `_gen_intermediate_updates` (`klippy/extras/pwm_tool.py:119`).
    pub fn gen_intermediate_updates(&self, clock: i64) {
        let (last_value, default_value, duration_ticks) = {
            let s = self.state.borrow();
            (s.last_value, s.default_value, s.duration_ticks)
        };
        if default_value.equals_int(last_value) {
            return;
        }
        // Re-read the clock on each step: send_update advances last_clock.
        loop {
            let last_clock_now = self.state.borrow().last_clock;
            if clock < last_clock_now + duration_ticks {
                break;
            }
            let val_now = self.state.borrow().last_value;
            self.send_update(last_clock_now + duration_ticks, val_now);
        }
    }

    /// `set_pwm` (`klippy/extras/pwm_tool.py:125`).
    pub fn set_pwm(&self, print_time: f64, value: f64) {
        let clock = self.mcu.print_time_to_clock(print_time);
        let invert = self.invert;
        let pwm_max = self.state.borrow().pwm_max;
        let mut value = value;
        if invert {
            value = 1.0 - value;
        }
        let v = float_to_int(clamp_0_1(value) * pwm_max + 0.5);
        let duration_ticks = self.state.borrow().duration_ticks;
        if duration_ticks != 0 {
            self.gen_intermediate_updates(clock - 1);
        }
        self.send_update(clock, v);
    }

    /// `_flush_notification` (`klippy/extras/pwm_tool.py:133`).
    pub fn flush_notification(&self, must_flush_time: f64, _max_step_gen_time: f64) {
        let clock = self.mcu.print_time_to_clock(must_flush_time);
        self.gen_intermediate_updates(clock);
    }

    /// Current state snapshot helpers (observation only).
    pub fn last_clock(&self) -> i64 {
        self.state.borrow().last_clock
    }
    pub fn last_value(&self) -> i64 {
        self.state.borrow().last_value
    }
    pub fn duration_ticks(&self) -> i64 {
        self.state.borrow().duration_ticks
    }
    pub fn set_cmd_tag(&self) -> Option<i64> {
        self.state.borrow().set_cmd_tag
    }
}

// ---------------------------------------------------------------------------
// PrinterOutputPin
// ---------------------------------------------------------------------------

struct PinState {
    last_print_time: f64,
    last_value: f64,
}

/// `klippy/extras/pwm_tool.py:137` `PrinterOutputPin`.
pub struct PrinterOutputPin {
    printer: Printer,
    mcu_pin: Arc<McuQueuedPwm>,
    scale: f64,
    #[allow(dead_code)]
    shutdown_value: f64,
    state: GCell<PinState>,
}

impl PrinterOutputPin {
    /// `PrinterOutputPin.__init__(config)` (`klippy/extras/pwm_tool.py:138`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let pins = lookup_interface::<dyn PinsApi>(&*printer, "pins", LookupDefault::Raise)?
            .expect("pins present");
        let pin_desc = config
            .get(
                "pin",
                GetStrOpts {
                    default: None,
                    note_valid: true,
                },
            )?
            .expect("pin required");
        let pin_params = pins.lookup_pin(
            &pin_desc,
            LookupPinOpts {
                can_invert: true,
                can_pullup: false,
                share_type: None,
            },
        )?;
        let mcu_pin = McuQueuedPwm::new(config, pin_params)?;
        let max_duration = mcu_pin.get_mcu().max_nominal_duration();
        let cycle_time = config
            .getfloat(
                "cycle_time",
                GetFloatOpts {
                    default: Some(Some(0.100)),
                    minval: None,
                    maxval: Some(max_duration),
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("cycle_time has a default");
        let hardware_pwm = config
            .getboolean(
                "hardware_pwm",
                GetBoolOpts {
                    default: Some(Some(false)),
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("hardware_pwm has a default");
        mcu_pin.setup_cycle_time(cycle_time, hardware_pwm);
        let scale = config
            .getfloat(
                "scale",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("scale has a default");
        let maximum_mcu_duration = config
            .getfloat(
                "maximum_mcu_duration",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.500),
                    maxval: Some(max_duration),
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("maximum_mcu_duration has a default");
        mcu_pin.setup_max_duration(maximum_mcu_duration);
        let last_value_raw = config
            .getfloat(
                "value",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    maxval: Some(scale),
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("value has a default");
        let shutdown_raw = config
            .getfloat(
                "shutdown_value",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    maxval: Some(scale),
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .expect("shutdown_value has a default");
        let last_value = last_value_raw / scale;
        let shutdown_value = shutdown_raw / scale;
        mcu_pin.setup_start_value(last_value, shutdown_value);
        let section_name = config.get_name();
        let pin_name = match section_name.split_whitespace().nth(1) {
            Some(n) => n.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        // `[pwm_tool <name>]`: config.get_name().split()[1] splits on any
        // whitespace run; split_whitespace matches for ASCII section names.
        let this = Arc::new(Self {
            printer: printer.clone(),
            mcu_pin,
            scale,
            shutdown_value,
            state: GCell::new(PinState {
                last_print_time: 0.0,
                last_value,
            }),
        });
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("gcode present");
        let for_cmd = Arc::clone(&this);
        gcode.register_mux_command(
            "SET_PIN",
            "PIN",
            Some(&pin_name),
            Box::new(move |gcmd: &GCmd| Self::cmd_set_pin(&for_cmd, gcmd)),
            Some("Set the value of an output pin"),
        )?;
        Ok(this)
    }

    /// Queued PWM helper (test/observation surface).
    pub fn mcu_pin(&self) -> Arc<McuQueuedPwm> {
        Arc::clone(&self.mcu_pin)
    }

    /// `get_status` (`klippy/extras/pwm_tool.py:167`).
    pub fn status_value(&self) -> f64 {
        self.state.borrow().last_value
    }

    /// `_set_pin` (`klippy/extras/pwm_tool.py:169`).
    pub fn set_pin(&self, print_time: f64, value: f64) {
        let (last_value, last_print_time) = {
            let s = self.state.borrow();
            (s.last_value, s.last_print_time)
        };
        if value == last_value {
            return;
        }
        let print_time = py_max(print_time, last_print_time);
        self.mcu_pin.set_pwm(print_time, value);
        let mut s = self.state.borrow_mut();
        s.last_value = value;
        s.last_print_time = print_time;
    }

    /// `cmd_SET_PIN` (`klippy/extras/pwm_tool.py:177`).
    pub fn cmd_set_pin(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let scale = this.scale;
        let scaled = gcmd
            .get_float(
                "VALUE",
                GcodeGetFloatOpts {
                    default: None,
                    minval: Some(0.0),
                    maxval: Some(scale),
                    above: None,
                    below: None,
                    ..Default::default()
                },
            )?
            .expect("VALUE is required");
        let value = scaled / scale;
        let toolhead = lookup_interface::<dyn ToolHeadApi>(
            &*this.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("toolhead present");
        let this2 = Arc::clone(this);
        toolhead.register_lookahead_callback(Box::new(move |print_time: f64| {
            this2.set_pin(print_time, value);
            Ok(())
        }))
    }
}

impl PrinterObject for PrinterOutputPin {
    klippy_api::printer_object_base!();

    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let mut map = indexmap::IndexMap::new();
        map.insert(
            PyValue::Str("value".to_string()),
            PyValue::Float(self.state.borrow().last_value),
        );
        Ok(Some(PyValue::Dict(map)))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/pwm_tool.py:186`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterOutputPin::new(config)? as ObjectRef))
}

/// Module entry point for the `pwm_tool` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
