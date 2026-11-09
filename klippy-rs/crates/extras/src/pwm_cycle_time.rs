//! Port of the `pwm_cycle_time` module (`klippy/extras/pwm_cycle_time.py`).
//!
//! Software PWM output pins with a variable cycle time: `MCU_pwm_cycle`
//! drives the MCU digital-out commands, and `PrinterOutputPWMCycle`
//! exposes the pin through the `SET_PIN` mux command.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, GetFloatOpts as GcmdGetFloatOpts, GCmd};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandWrapper, LookupPinOpts, Mcu, PinParams, SendOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::{LookaheadCallback, ToolHeadApi};
use klippy_api::{GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, trunc_float_to_bigint, PctValue};
use pycompat::{PyInt, PyValue};
use std::sync::Arc;

/// `1<<31`, the `cycle_ticks` limit (`klippy/extras/pwm_cycle_time.py`).
const MAX_CYCLE_TICKS: i64 = 1 << 31;

/// Python `max` with two arguments (first maximal item wins).
fn py_max2(a: f64, b: f64) -> f64 {
    if b > a {
        b
    } else {
        a
    }
}

/// Python `min` with two arguments (first minimal item wins).
fn py_min2(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `max(0., min(1., v))` (`klippy/extras/pwm_cycle_time.py`).
fn clamp_unit(v: f64) -> f64 {
    py_max2(0., py_min2(1., v))
}

/// `int(float)`: truncation toward zero (`klippy/extras/pwm_cycle_time.py`).
///
/// `trunc_float_to_bigint` covers exact truncation; the i64 conversion
/// clamps for values outside i64 range, which real cycle times never
/// reach (cycle_ticks < 2^31).
fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = trunc_float_to_bigint(f).0.to_string();
    digits
        .parse::<i64>()
        .unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

fn pct_format(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant pwm_cycle_time format")
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

// ---------------------------------------------------------------------------
// MCU_pwm_cycle
// ---------------------------------------------------------------------------

/// Mutable state of `MCU_pwm_cycle`, one `GCell` mirroring the Python
/// instance dict (`klippy/extras/pwm_cycle_time.py:8-21`).
struct McuPwmCycleState {
    oid: i64,
    last_clock: i64,
    cycle_ticks: i64,
    set_cmd: Option<CommandWrapper>,
    set_cycle_ticks: Option<CommandWrapper>,
}

impl McuPwmCycleState {
    fn new() -> McuPwmCycleState {
        McuPwmCycleState {
            oid: 0,
            last_clock: 0,
            cycle_ticks: 0,
            set_cmd: None,
            set_cycle_ticks: None,
        }
    }
}

/// `klippy/extras/pwm_cycle_time.py:MCU_pwm_cycle`.
pub struct McuPwmCycle {
    mcu: Mcu,
    cycle_time: f64,
    pin: String,
    invert: bool,
    start_value: f64,
    shutdown_value: f64,
    state: GCell<McuPwmCycleState>,
}

impl McuPwmCycle {
    /// `MCU_pwm_cycle.__init__` (`klippy/extras/pwm_cycle_time.py:8`).
    pub fn new(
        pin_params: &PinParams,
        cycle_time: f64,
        start_value: f64,
        shutdown_value: f64,
    ) -> KResult<Arc<McuPwmCycle>> {
        let invert = pin_params.invert != 0;
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if invert {
            start_value = 1. - start_value;
            shutdown_value = 1. - shutdown_value;
        }
        let mcu = pin_params.chip.clone();
        let pin = pin_params.pin.clone();
        let start_value = clamp_unit(start_value);
        let shutdown_value = clamp_unit(shutdown_value);
        let mcu_pwm_cycle = Arc::new(McuPwmCycle {
            mcu: mcu.clone(),
            cycle_time,
            pin,
            invert,
            start_value,
            shutdown_value,
            state: GCell::new(McuPwmCycleState::new()),
        });
        let mcu_pwm_cycle_clone = mcu_pwm_cycle.clone();
        mcu.register_config_callback(Box::new(move || mcu_pwm_cycle_clone.build_config()))?;
        Ok(mcu_pwm_cycle)
    }

    /// `get_mcu` (`klippy/extras/pwm_cycle_time.py:22`).
    pub fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }

    /// `_build_config` (`klippy/extras/pwm_cycle_time.py:24`).
    pub fn build_config(&self) -> KResult<()> {
        let cmd_queue = self.mcu.alloc_command_queue();
        let curtime = self.mcu.get_printer().get_reactor().monotonic();
        let printtime = self.mcu.estimated_print_time(curtime);
        let last_clock = self.mcu.print_time_to_clock(printtime + 0.200);
        self.state.borrow_mut().last_clock = last_clock;
        let cycle_ticks = self.mcu.seconds_to_clock(self.cycle_time);
        if !(self.shutdown_value == 0. || self.shutdown_value == 1.) {
            return Err(KlippyError::config(
                "shutdown value must be 0.0 or 1.0 on soft pwm",
            ));
        }
        if cycle_ticks >= MAX_CYCLE_TICKS {
            return Err(KlippyError::config("PWM pin cycle time too large"));
        }
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid()?;
        self.state.borrow_mut().oid = oid;
        self.mcu.add_config_cmd(
            &pct_format(
                "config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
                &[
                    PctValue::Int(PyInt::from_i64(oid)),
                    PctValue::Str(self.pin.clone()),
                    PctValue::Bool(self.start_value >= 1.),
                    PctValue::Bool(self.shutdown_value >= 0.5),
                    PctValue::Int(PyInt::from_i64(0)),
                ],
            ),
            AddConfigCmdOpts::default(),
        )?;
        self.mcu.add_config_cmd(
            &pct_format(
                "set_digital_out_pwm_cycle oid=%d cycle_ticks=%d",
                &[
                    PctValue::Int(PyInt::from_i64(oid)),
                    PctValue::Int(PyInt::from_i64(cycle_ticks)),
                ],
            ),
            AddConfigCmdOpts::default(),
        )?;
        self.state.borrow_mut().cycle_ticks = cycle_ticks;
        let svalue = py_trunc_i64(self.start_value * (cycle_ticks as f64) + 0.5);
        let last_clock = self.state.borrow().last_clock;
        self.mcu.add_config_cmd(
            &pct_format(
                "queue_digital_out oid=%d clock=%d on_ticks=%d",
                &[
                    PctValue::Int(PyInt::from_i64(oid)),
                    PctValue::Int(PyInt::from_i64(last_clock)),
                    PctValue::Int(PyInt::from_i64(svalue)),
                ],
            ),
            AddConfigCmdOpts { is_init: true, on_restart: false },
        )?;
        let set_cmd = self
            .mcu
            .lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?;
        let set_cycle_ticks = self.mcu.lookup_command(
            "set_digital_out_pwm_cycle oid=%c cycle_ticks=%u",
            Some(cmd_queue),
        )?;
        let mut state = self.state.borrow_mut();
        state.set_cmd = Some(set_cmd);
        state.set_cycle_ticks = Some(set_cycle_ticks);
        Ok(())
    }

    /// `set_pwm_cycle` (`klippy/extras/pwm_cycle_time.py:55`).
    pub fn set_pwm_cycle(&self, print_time: f64, value: f64, cycle_time: f64) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let minclock = self.state.borrow().last_clock;
        let cycle_ticks = self.mcu.seconds_to_clock(cycle_time);
        let current_ticks = self.state.borrow().cycle_ticks;
        if cycle_ticks != current_ticks {
            if cycle_ticks >= MAX_CYCLE_TICKS {
                return Err(KlippyError::command("PWM cycle time too large"));
            }
            let set_cycle_ticks = self
                .state
                .borrow()
                .set_cycle_ticks
                .clone()
                .expect("set_cycle_ticks set by _build_config");
            let oid = self.state.borrow().oid;
            set_cycle_ticks.send(
                vec![py_int(oid), py_int(cycle_ticks)],
                SendOpts { minclock, reqclock: clock },
            )?;
            self.state.borrow_mut().cycle_ticks = cycle_ticks;
        }
        let mut value = value;
        if self.invert {
            value = 1. - value;
        }
        let cycle_ticks = self.state.borrow().cycle_ticks;
        let v = py_trunc_i64(clamp_unit(value) * (cycle_ticks as f64) + 0.5);
        let set_cmd = self
            .state
            .borrow()
            .set_cmd
            .clone()
            .expect("set_cmd set by _build_config");
        let oid = self.state.borrow().oid;
        let last_clock = self.state.borrow().last_clock;
        set_cmd.send(
            vec![py_int(oid), py_int(clock), py_int(v)],
            SendOpts { minclock: last_clock, reqclock: clock },
        )?;
        self.state.borrow_mut().last_clock = clock;
        Ok(())
    }

    /// `oid`, exposed for the parity tests.
    pub fn oid(&self) -> i64 {
        self.state.borrow().oid
    }

    /// `last_clock`, exposed for the parity tests.
    pub fn last_clock(&self) -> i64 {
        self.state.borrow().last_clock
    }

    /// `cycle_ticks`, exposed for the parity tests.
    pub fn cycle_ticks(&self) -> i64 {
        self.state.borrow().cycle_ticks
    }
}

klippy_api::printer_object!(McuPwmCycle);

// ---------------------------------------------------------------------------
// PrinterOutputPWMCycle
// ---------------------------------------------------------------------------

/// Mutable state of `PrinterOutputPWMCycle`, the shared part of the
/// Python instance dict (`klippy/extras/pwm_cycle_time.py:77-93`).
/// Shared through `Arc` so the lookahead callback can run `_set_pin`.
struct OutputState {
    last_print_time: GCell<f64>,
    last_value: GCell<f64>,
    last_cycle_time: GCell<f64>,
}

impl OutputState {
    fn set_pin(
        &self,
        mcu_pin: &Arc<McuPwmCycle>,
        print_time: f64,
        value: f64,
        cycle_time: f64,
    ) -> KResult<()> {
        if value == self.last_value.get() && cycle_time == self.last_cycle_time.get() {
            return Ok(());
        }
        let min_sched_time = mcu_pin.get_mcu().min_schedule_time();
        let print_time = py_max2(print_time, self.last_print_time.get() + min_sched_time);
        mcu_pin.set_pwm_cycle(print_time, value, cycle_time)?;
        self.last_value.set(value);
        self.last_cycle_time.set(cycle_time);
        self.last_print_time.set(print_time);
        Ok(())
    }
}

/// `config.getfloat(...)` with a numeric default and bounds.
fn config_float(
    config: &Config,
    option: &str,
    default: f64,
    minval: Option<f64>,
    maxval: Option<f64>,
    above: Option<f64>,
) -> KResult<f64> {
    let opts = GetFloatOpts {
        default: Some(Some(default)),
        minval,
        maxval,
        above,
        ..GetFloatOpts::default()
    };
    Ok(config.getfloat(option, opts)?.unwrap_or(default))
}

/// `klippy/extras/pwm_cycle_time.py:PrinterOutputPWMCycle`.
pub struct PrinterOutputPWMCycle {
    printer: Printer,
    scale: f64,
    shutdown_value: f64,
    mcu_pin: Arc<McuPwmCycle>,
    default_cycle_time: f64,
    state: Arc<OutputState>,
}

impl PrinterOutputPWMCycle {
    /// `PrinterOutputPWMCycle.__init__` (`klippy/extras/pwm_cycle_time.py:76`).
    pub fn new(config: &Config) -> KResult<Arc<PrinterOutputPWMCycle>> {
        let printer = config.get_printer();
        let scale = config_float(config, "scale", 1., None, None, Some(0.))?;
        let last_value = config_float(config, "value", 0., Some(0.), Some(scale), None)? / scale;
        let shutdown_value =
            config_float(config, "shutdown_value", 0., Some(0.), Some(scale), None)? / scale;
        let pins = klippy_api::lookup_interface::<dyn klippy_api::mcu::PinsApi>(
            &*printer,
            "pins",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let pin_desc = config
            .get("pin", GetStrOpts::default())?
            .expect("get('pin') without default always returns Some");
        let pin_params = pins.lookup_pin(
            &pin_desc,
            LookupPinOpts { can_invert: true, ..LookupPinOpts::default() },
        )?;
        let max_duration = pin_params.chip.max_nominal_duration();
        let cycle_time =
            config_float(config, "cycle_time", 0.100, None, Some(max_duration), Some(0.))?;
        let mcu_pin = McuPwmCycle::new(&pin_params, cycle_time, last_value, shutdown_value)?;
        let s = Arc::new(PrinterOutputPWMCycle {
            printer: printer.clone(),
            scale,
            shutdown_value,
            mcu_pin,
            default_cycle_time: cycle_time,
            state: Arc::new(OutputState {
                last_print_time: GCell::new(0.),
                last_value: GCell::new(last_value),
                last_cycle_time: GCell::new(cycle_time),
            }),
        });
        let pin_name = config.get_name().split_whitespace().collect::<Vec<_>>()[1].to_string();
        let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let s_for_cmd = s.clone();
        let cmd_set_pin: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| s_for_cmd.cmd_set_pin(gcmd));
        gcode.register_mux_command(
            "SET_PIN",
            "PIN",
            Some(&pin_name),
            cmd_set_pin,
            Some("Set the value of an output pin"),
        )?;
        Ok(s)
    }

    /// `_set_pin` (`klippy/extras/pwm_cycle_time.py:102`).
    pub fn set_pin(&self, print_time: f64, value: f64, cycle_time: f64) -> KResult<()> {
        self.state.set_pin(&self.mcu_pin, print_time, value, cycle_time)
    }

    /// `cmd_SET_PIN` (`klippy/extras/pwm_cycle_time.py:112`).
    pub fn cmd_set_pin(&self, gcmd: &GCmd) -> KResult<()> {
        let value = gcmd
            .get_float(
                "VALUE",
                GcmdGetFloatOpts {
                    minval: Some(0.),
                    maxval: Some(self.scale),
                    ..GcmdGetFloatOpts::default()
                },
            )?
            .expect("get_float without default always returns Some")
            / self.scale;
        let max_duration = self.mcu_pin.get_mcu().max_nominal_duration();
        let cycle_time = gcmd
            .get_float(
                "CYCLE_TIME",
                GcmdGetFloatOpts {
                    default: Some(Some(self.default_cycle_time)),
                    above: Some(0.),
                    maxval: Some(max_duration),
                    ..GcmdGetFloatOpts::default()
                },
            )?
            .unwrap_or(self.default_cycle_time);
        let toolhead = klippy_api::lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let state = self.state.clone();
        let mcu_pin = self.mcu_pin.clone();
        let callback: LookaheadCallback = Box::new(move |print_time: f64| {
            state.set_pin(&mcu_pin, print_time, value, cycle_time)
        });
        toolhead.register_lookahead_callback(callback)
    }

    /// `get_status` (`klippy/extras/pwm_cycle_time.py:100`).
    pub fn status_value(&self) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("value".to_string()),
            PyValue::Float(self.state.last_value.get()),
        );
        PyValue::Dict(d)
    }

    /// `scale`, exposed for the parity tests.
    pub fn scale(&self) -> f64 {
        self.scale
    }

    /// `shutdown_value`, exposed for the parity tests.
    pub fn shutdown_value(&self) -> f64 {
        self.shutdown_value
    }

    /// `mcu_pin`, exposed for the parity tests.
    pub fn mcu_pin(&self) -> Arc<McuPwmCycle> {
        self.mcu_pin.clone()
    }

    /// `default_cycle_time`, exposed for the parity tests.
    pub fn default_cycle_time(&self) -> f64 {
        self.default_cycle_time
    }

    /// `last_value`, exposed for the parity tests.
    pub fn last_value(&self) -> f64 {
        self.state.last_value.get()
    }

    /// `last_cycle_time`, exposed for the parity tests.
    pub fn last_cycle_time(&self) -> f64 {
        self.state.last_cycle_time.get()
    }

    /// `last_print_time`, exposed for the parity tests.
    pub fn last_print_time(&self) -> f64 {
        self.state.last_print_time.get()
    }
}

impl PrinterObject for PrinterOutputPWMCycle {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status_value()))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/pwm_cycle_time.py:124`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(PrinterOutputPWMCycle::new(config)? as ObjectRef))
}

/// Module entry point for the `pwm_cycle_time` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
