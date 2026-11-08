//! Port of `klippy/extras/buttons.py`: button state tracking.
//!
//! Covers digital pins (`McuButtons`, `config_buttons`/`buttons_add`/
//! `buttons_query`/`buttons_ack`/`buttons_state`), ADC buttons
//! (`McuAdcButtons`), rotary encoders (`RotaryEncoder`, full- and
//! half-step), debounced buttons (`DebounceButton`) and the button
//! registration surface (`PrinterButtons`, loaded as `[buttons]` via
//! the factory's `load_config`).
//!
//! Python bugs are reproduced (see `ARCHITECTURE.md`): the 8-bit ack
//! unwrap, the 8-pins-per-MCU replacement rule, the step-table shapes
//! and the strict/relaxed inequality edges in the ADC ladder.
//!
//! Cross-module dependencies, reached through interfaces:
//! - `pins` object (`klippy_api::mcu::PinsApi` via `lookup_interface`):
//!   `lookup_pin`/`setup_pin`.
//! - `query_adc` object (`crate::query_adc::QueryAdcApi`, owned by the
//!   `query_adc` module): `register_adc`.

use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{
    AdcSampleOpts, AddConfigCmdOpts, CommandWrapper, LookupPinOpts, Mcu, Pin, PinParams,
    PinsApi, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::{lookup_interface, ObjectRef};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, NOW};

use crate::query_adc::QueryAdcApi;

/// MCU query period, `buttons.py:13` `QUERY_TIME`.
pub const QUERY_TIME: f64 = 0.002;
/// Retransmit count, `buttons.py:14` `RETRANSMIT_COUNT`.
pub const RETRANSMIT_COUNT: i64 = 50;
/// ADC reporting period, `buttons.py:90` `ADC_REPORT_TIME`.
pub const ADC_REPORT_TIME: f64 = 0.010;
/// ADC debounce window, `buttons.py:91` `ADC_DEBOUNCE_TIME`.
pub const ADC_DEBOUNCE_TIME: f64 = 0.025;
/// ADC sampling period, `buttons.py:92` `ADC_SAMPLE_TIME`.
pub const ADC_SAMPLE_TIME: f64 = 0.001;
/// ADC sample count, `buttons.py:93` `ADC_SAMPLE_COUNT`.
pub const ADC_SAMPLE_COUNT: i64 = 6;
/// ADC batch count, `buttons.py:94` `ADC_BATCH_COUNT`.
pub const ADC_BATCH_COUNT: i64 = 5;

/// Digital button callback `(eventtime, state)` over one or more pins
/// (`register_buttons`).
pub type ButtonCallback = Arc<dyn Fn(f64, i64) + Send + Sync + 'static>;
/// ADC button callback `(eventtime, pressed)`.
pub type AdcButtonCallback = Arc<dyn Fn(f64, bool) + Send + Sync + 'static>;
/// Single-event callback `(eventtime)` (push/encoder destinations).
pub type PushCallback = Arc<dyn Fn(f64) + Send + Sync + 'static>;
/// `DebounceButton` action `(eventtime, state)`.
pub type DebounceCallback = Arc<dyn Fn(f64, ButtonState) + Send + Sync + 'static>;

/// A debounced button's state (`None` in Python before any event).
#[derive(Clone, Copy, Debug)]
pub enum ButtonState {
    /// Python `None` (before the first event).
    Empty,
    /// ADC pressed/released.
    Analog(bool),
    /// Digital pin bit state.
    Digital(i64),
}

impl ButtonState {
    /// The number Python would compare this against (`None` has none).
    fn number(&self) -> Option<i64> {
        match self {
            ButtonState::Empty => None,
            ButtonState::Analog(b) => Some(if *b { 1 } else { 0 }),
            ButtonState::Digital(i) => Some(*i),
        }
    }

    /// Human-readable form for test recordings.
    pub fn repr(&self) -> String {
        match self {
            ButtonState::Empty => "None".to_string(),
            ButtonState::Analog(b) => if *b { "True".to_string() } else { "False".to_string() },
            ButtonState::Digital(i) => i.to_string(),
        }
    }
}

/// Python equality: `None == None`, `False`/`0` and `True`/`1` compare by
/// value, across the analog/digital representations.
impl PartialEq for ButtonState {
    fn eq(&self, other: &Self) -> bool {
        match (self.number(), other.number()) {
            (Some(a), Some(b)) => a == b,
            (None, None) => true,
            _ => false,
        }
    }
}

impl Eq for ButtonState {}

// ---------------------------------------------------------------------------
// Small value helpers.
// ---------------------------------------------------------------------------

/// Pull the integer out of a `PyValue` (`msgproto` decodes `%c`/`%d` as
/// `int`; `bool`/`float` follow `int()` semantics).
fn pyvalue_i64(v: &PyValue) -> i64 {
    let n = match v {
        PyValue::Int(n) => n.clone(),
        PyValue::Bool(b) => PyInt::from_i64(if *b { 1 } else { 0 }),
        PyValue::Float(f) => pycompat::text::trunc_float_to_bigint(*f),
        _ => return 0,
    };
    let s = n.0.to_string();
    s.parse::<i64>()
        .unwrap_or(if s.starts_with('-') { i64::MIN } else { i64::MAX })
}

fn dict_get<'a>(params: &'a PyValue, key: &str) -> Option<&'a PyValue> {
    match params {
        PyValue::Dict(d) => d.get(&PyValue::Str(key.to_string())),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// MCU_buttons (digital pins)
// ---------------------------------------------------------------------------

struct McuButtonsInner {
    /// `(pin, pull_up)` per button, `pin_list`.
    pin_list: Vec<(String, i64)>,
    /// `(mask, shift, callback)` per `setup_buttons` call.
    callbacks: Vec<(i64, i64, ButtonCallback)>,
    /// XOR mask for `invert` bits, `invert`.
    invert: i64,
    /// Last full byte seen after XOR, `last_button`.
    last_button: i64,
    /// Last acked count, `ack_count`.
    ack_count: i64,
    /// `buttons_ack` command, `ack_cmd`.
    ack_cmd: Option<CommandWrapper>,
    /// `oid`.
    oid: i64,
}

/// `MCU_buttons` (`buttons.py:16`): one batch of digital button pins on
/// one MCU.
pub struct McuButtons {
    reactor: Reactor,
    mcu: Mcu,
    inner: GCell<McuButtonsInner>,
}

impl McuButtons {
    /// `MCU_buttons(printer, mcu)`; registers `build_config` with the MCU.
    pub fn new(printer: &Printer, mcu: Mcu) -> KResult<Arc<Self>> {
        let this = Arc::new(McuButtons {
            reactor: printer.get_reactor(),
            mcu: mcu.clone(),
            inner: GCell::new(McuButtonsInner {
                pin_list: Vec::new(),
                callbacks: Vec::new(),
                invert: 0,
                last_button: 0,
                ack_count: 0,
                ack_cmd: None,
                oid: 0,
            }),
        });
        let me = Arc::clone(&this);
        mcu.register_config_callback(Box::new(move || me.build_config()))?;
        Ok(this)
    }

    /// Number of registered pins (`len(pin_list)`), for the
    /// `register_buttons` 8-pins-per-MCU rule.
    fn pin_count(&self) -> usize {
        self.inner.borrow().pin_list.len()
    }

    /// `setup_buttons(pins, callback)` where `pins` are `pin_params`
    /// dicts from `PrinterPins.lookup_pin`.
    pub fn setup_buttons(&self, pins: Vec<PinParams>, callback: ButtonCallback) {
        let mut inner = self.inner.borrow_mut();
        let mut mask = 0i64;
        let shift = inner.pin_list.len() as i64;
        for pin_params in &pins {
            if pin_params.invert != 0 {
                inner.invert |= 1i64 << inner.pin_list.len();
            }
            mask |= 1i64 << inner.pin_list.len();
            inner.pin_list.push((pin_params.pin.clone(), pin_params.pullup));
        }
        inner.callbacks.push((mask, shift, callback));
    }

    /// `build_config()`: emits the `config_buttons`/`buttons_add`/
    /// `buttons_query` commands and registers the `buttons_state`
    /// serial response. No-op when no pins were registered.
    pub fn build_config(self: &Arc<Self>) -> KResult<()> {
        let (pin_list, invert) = {
            let inner = self.inner.borrow();
            if inner.pin_list.is_empty() {
                return Ok(());
            }
            (inner.pin_list.clone(), inner.invert)
        };
        let oid = self.mcu.create_oid()?;
        self.inner.borrow_mut().oid = oid;
        self.mcu.add_config_cmd(
            &format!("config_buttons oid={} button_count={}", oid, pin_list.len()),
            AddConfigCmdOpts::default(),
        )?;

        for (i, (pin, pull_up)) in pin_list.iter().enumerate() {
            self.mcu.add_config_cmd(
                &format!("buttons_add oid={} pos={} pin={} pull_up={}", oid, i, pin, pull_up),
                AddConfigCmdOpts { is_init: true, on_restart: false },
            )?;
        }
        let cmd_queue = self.mcu.alloc_command_queue();
        let ack_cmd =
            self.mcu.lookup_command("buttons_ack oid=%c count=%c", Some(cmd_queue))?;
        self.inner.borrow_mut().ack_cmd = Some(ack_cmd);
        let clock = self.mcu.get_query_slot(oid);
        let rest_ticks = self.mcu.seconds_to_clock(QUERY_TIME);
        self.mcu.add_config_cmd(
            &format!(
                "buttons_query oid={} clock={} rest_ticks={} retransmit_count={} invert={}",
                oid, clock, rest_ticks, RETRANSMIT_COUNT, invert
            ),
            AddConfigCmdOpts { is_init: true, on_restart: false },
        )?;
        let me = Arc::clone(self);
        let cb: SerialResponseCallback = Arc::new(move |params: PyValue| {
            me.handle_buttons_state(&params);
        });
        let _ = self
            .mcu
            .register_serial_response(cb, "buttons_state oid=%c ack_count=%c state=%*s", Some(oid));
        Ok(())
    }

    /// `handle_buttons_state(params)`: unwrap the ack counter, ack the
    /// MCU, and dispatch per-button callbacks.
    pub fn handle_buttons_state(&self, params: &PyValue) {
        let ack_in = dict_get(params, "ack_count").map(pyvalue_i64).unwrap_or(0);
        let state_bytes: Vec<u8> = match dict_get(params, "state") {
            Some(PyValue::Bytes(b)) => b.clone(),
            _ => Vec::new(),
        };
        let btime = dict_get(params, "#receive_time")
            .and_then(|v| v.to_f64())
            .unwrap_or(0.0);
        let (oid, ack_cmd, invert, last_button, ack_count, callbacks) = {
            let inner = self.inner.borrow();
            (
                inner.oid,
                inner.ack_cmd.clone(),
                inner.invert,
                inner.last_button,
                inner.ack_count,
                inner.callbacks.clone(),
            )
        };
        // Expand the message ack_count from 8-bit.
        let ack_diff = (ack_in - ack_count) & 0xff;
        let ack_diff = ack_diff - ((ack_diff & 0x80) << 1);
        let msg_ack_count = ack_count + ack_diff;
        let new_count = msg_ack_count + state_bytes.len() as i64 - ack_count;
        if new_count <= 0 {
            return;
        }
        // `buttons[-new_count:]` (whole list when new_count > len).
        let start =
            state_bytes.len().saturating_sub(new_count as usize);
        let new_buttons = state_bytes[start..].to_vec();
        // Send ack to MCU.
        let cmd = match ack_cmd {
            Some(cmd) => cmd,
            None => {
                self.log_serial_callback_error();
                return;
            }
        };
        let sent = cmd.send(
            vec![PyValue::Int(PyInt::from_i64(oid)), PyValue::Int(PyInt::from_i64(new_count))],
            SendOpts::default(),
        );
        if sent.is_err() {
            // Python's serial callback wrapper logs the exception and
            // aborts the handler: no ack count update, no callbacks.
            self.log_serial_callback_error();
            return;
        }
        // Xor each byte, fixed-width; transitions and callback dispatch.
        let mut last = last_button;
        let mut events: Vec<(ButtonCallback, i64, f64)> = Vec::new();
        for raw in new_buttons {
            let button = (raw as i64) ^ invert;
            let changed = button ^ last;
            last = button;
            for (mask, shift, callback) in &callbacks {
                if changed & mask != 0 {
                    let state = (button & mask) >> shift;
                    events.push((Arc::clone(callback), state, btime));
                }
            }
        }
        {
            let mut inner = self.inner.borrow_mut();
            inner.ack_count = ack_count + new_count;
            inner.last_button = last;
        }
        for (callback, state, bt) in events {
            self.reactor
                .register_async_callback(move |_et| callback(bt, state), NOW);
        }
    }

    fn log_serial_callback_error(&self) {
        // `serialhdl.py:63-65` logs "%sException in serial callback"
        // (warn_prefix = "mcu '<name>': "); the port has no Python
        // traceback, the normalizer table covers traceback bodies.
        klippy_api::log::error(&format!(
            "mcu '{}': Exception in serial callback",
            self.mcu.get_name()
        ));
    }
}

// ---------------------------------------------------------------------------
// MCU_ADC_buttons
// ---------------------------------------------------------------------------

struct McuAdcInner {
    /// `(min_value, max_value, callback)` per `setup_button` call.
    buttons: Vec<(f64, f64, AdcButtonCallback)>,
    last_button: Option<usize>,
    last_pressed: Option<usize>,
    last_debouncetime: f64,
    pullup: f64,
    pin: String,
    min_value: f64,
    max_value: f64,
}

/// `MCU_ADC_buttons` (`buttons.py:96`): strips ADC samples into button
/// presses.
pub struct McuAdcButtons {
    reactor: Reactor,
    inner: GCell<McuAdcInner>,
}

impl McuAdcButtons {
    /// `MCU_ADC_buttons(printer, pin, pullup)`.
    pub fn new(printer: &Printer, pin: &str, pullup: f64) -> KResult<Arc<Self>> {
        let ppins = lookup_interface::<dyn PinsApi>(&**printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let pin_obj = ppins.setup_pin("adc", pin)?;
        let mcu_adc = match pin_obj {
            Pin::Adc(adc) => adc,
            Pin::Endstop(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_endstop' object has no attribute 'setup_adc_sample'"
                        .to_string(),
                });
            }
            Pin::DigitalOut(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_digital_out' object has no attribute 'setup_adc_sample'"
                        .to_string(),
                });
            }
            Pin::Pwm(_) => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'MCU_pwm' object has no attribute 'setup_adc_sample'".to_string(),
                });
            }
        };
        mcu_adc.setup_adc_sample(
            ADC_REPORT_TIME,
            AdcSampleOpts {
                sample_time: ADC_SAMPLE_TIME,
                sample_count: ADC_SAMPLE_COUNT,
                batch_num: ADC_BATCH_COUNT,
                ..AdcSampleOpts::default()
            },
        );
        let me = Arc::new(McuAdcButtons {
            reactor: printer.get_reactor(),
            inner: GCell::new(McuAdcInner {
                buttons: Vec::new(),
                last_button: None,
                last_pressed: None,
                last_debouncetime: 0.,
                pullup,
                pin: pin.to_string(),
                min_value: 999999999999.9,
                max_value: 0.,
            }),
        });
        let cb_me = Arc::clone(&me);
        mcu_adc.setup_adc_callback(Box::new(move |samples| cb_me.adc_callback(samples)));
        let query_adc =
            lookup_interface::<dyn QueryAdcApi>(&**printer, "query_adc", LookupDefault::Raise)?
                .expect("Raise never yields None");
        let pin = me.inner.borrow().pin.clone();
        query_adc.register_adc(&format!("adc_button:{}", pin.trim()), mcu_adc.clone());
        Ok(me)
    }

    /// `setup_button(min_value, max_value, callback)`.
    pub fn setup_button(&self, min_value: f64, max_value: f64, callback: AdcButtonCallback) {
        let mut inner = self.inner.borrow_mut();
        inner.min_value = inner.min_value.min(min_value);
        inner.max_value = inner.max_value.max(max_value);
        inner.buttons.push((min_value, max_value, callback));
    }

    /// `adc_callback(samples)`.
    pub fn adc_callback(&self, samples: Vec<(f64, f64)>) {
        for (read_time, read_value) in samples {
            self.adc_sample(read_time, read_value);
        }
    }

    fn adc_sample(&self, read_time: f64, read_value: f64) {
        let events: Vec<(AdcButtonCallback, bool)> = {
            let mut inner = self.inner.borrow_mut();
            #[allow(clippy::manual_clamp)]
            let adc = read_value.min(0.99999).max(0.00001);
            let value = inner.pullup * adc / (1.0 - adc);
            // Determine button pressed.
            let mut btn: Option<usize> = None;
            if inner.min_value <= value && value <= inner.max_value {
                for (i, (min_v, max_v, _)) in inner.buttons.iter().enumerate() {
                    if *min_v < value && value < *max_v {
                        btn = Some(i);
                        break;
                    }
                }
            }
            // If the button changed, due to noise or pressing:
            if btn != inner.last_button {
                // reset the debouncing timer
                inner.last_debouncetime = read_time;
            }
            // button debounce check & new button pressed
            let mut events: Vec<(AdcButtonCallback, bool)> = Vec::new();
            if (read_time - inner.last_debouncetime) >= ADC_DEBOUNCE_TIME
                && inner.last_button == btn
                && inner.last_pressed != btn
            {
                // release last_pressed
                if let Some(pressed) = inner.last_pressed {
                    events.push((inner.buttons[pressed].2.clone(), false));
                    inner.last_pressed = None;
                }
                if let Some(b) = btn {
                    events.push((inner.buttons[b].2.clone(), true));
                    inner.last_pressed = Some(b);
                }
            }
            inner.last_button = btn;
            events
        };
        for (callback, state) in events {
            self.reactor.register_async_callback(move |et| callback(et, state), NOW);
        }
    }

    /// `call_button(button, state)`.
    pub fn call_button(&self, button: usize, state: bool) {
        let callback = self.inner.borrow().buttons[button].2.clone();
        self.reactor.register_async_callback(move |eventtime| callback(eventtime, state), NOW);
    }
}

// ---------------------------------------------------------------------------
// Rotary encoders
// ---------------------------------------------------------------------------

const R_START: i64 = 0x0;
const R_DIR_CW: i64 = 0x10;
const R_DIR_CCW: i64 = 0x20;
const R_DIR_MSK: i64 = 0x30;

/// `FullStepRotaryEncoder.ENCODER_STATES` (`buttons.py:192-218`).
const FULL_STEP_STATES: [[i64; 4]; 7] = [
    // R_START
    [R_START, 0x2, 0x4, R_START],
    // R_CW_FINAL
    [0x3, R_START, 0x1, R_START | R_DIR_CW],
    // R_CW_BEGIN
    [0x3, 0x2, R_START, R_START],
    // R_CW_NEXT
    [0x3, 0x2, 0x1, R_START],
    // R_CCW_BEGIN
    [0x6, R_START, 0x4, R_START],
    // R_CCW_FINAL
    [0x6, 0x5, R_START, R_START | R_DIR_CCW],
    // R_CCW_NEXT
    [0x6, 0x5, 0x4, R_START],
];

/// `HalfStepRotaryEncoder.ENCODER_STATES` (`buttons.py:228-250`).
const HALF_STEP_STATES: [[i64; 4]; 6] = [
    // R_START (00)
    [0x3, 0x2, 0x1, R_START],
    // R_CCW_BEGIN
    [0x3 | R_DIR_CCW, R_START, 0x1, R_START],
    // R_CW_BEGIN
    [0x3 | R_DIR_CW, 0x2, R_START, R_START],
    // R_START_M (11)
    [0x3, 0x5, 0x4, R_START],
    // R_CW_BEGIN_M
    [0x3, 0x3, 0x4, R_START | R_DIR_CW],
    // R_CCW_BEGIN_M
    [0x3, 0x5, 0x3, R_START | R_DIR_CCW],
];

/// `BaseRotaryEncoder` (`buttons.py:165`): either encoder flavor,
/// selected by state table.
pub struct RotaryEncoder {
    cw_callback: PushCallback,
    ccw_callback: PushCallback,
    encoder_state: GCell<i64>,
    states: &'static [[i64; 4]],
}

impl RotaryEncoder {
    /// `FullStepRotaryEncoder(cw_callback, ccw_callback)`.
    pub fn full_step(cw_callback: PushCallback, ccw_callback: PushCallback) -> Self {
        RotaryEncoder {
            cw_callback,
            ccw_callback,
            encoder_state: GCell::new(R_START),
            states: &FULL_STEP_STATES,
        }
    }

    /// `HalfStepRotaryEncoder(cw_callback, ccw_callback)`.
    pub fn half_step(cw_callback: PushCallback, ccw_callback: PushCallback) -> Self {
        RotaryEncoder {
            cw_callback,
            ccw_callback,
            encoder_state: GCell::new(R_START),
            states: &HALF_STEP_STATES,
        }
    }

    /// `encoder_callback(eventtime, state)`.
    pub fn encoder_callback(&self, eventtime: f64, state: i64) {
        let es = self.states[(self.encoder_state.get() & 0xf) as usize][(state & 0x3) as usize];
        self.encoder_state.set(es);
        if es & R_DIR_MSK == R_DIR_CW {
            (self.cw_callback)(eventtime);
        } else if es & R_DIR_MSK == R_DIR_CCW {
            (self.ccw_callback)(eventtime);
        }
    }
}

// ---------------------------------------------------------------------------
// DebounceButton
// ---------------------------------------------------------------------------

struct DebounceInner {
    logical_state: ButtonState,
    physical_state: ButtonState,
    latest_eventtime: Option<f64>,
}

/// `DebounceButton` (`buttons.py:252`): releases a button action only
/// after `debounce_delay` of stable state.
#[derive(Clone)]
pub struct DebounceButton {
    reactor: Reactor,
    debounce_delay: f64,
    button_action: DebounceCallback,
    inner: Arc<GCell<DebounceInner>>,
}

impl DebounceButton {
    /// `DebounceButton(config, button_action)`.
    pub fn new(config: &Config, button_action: DebounceCallback) -> KResult<Self> {
        let printer = config.get_printer();
        let delay = config
            .getfloat(
                "debounce_delay",
                GetFloatOpts {
                    default: Some(Some(0.)),
                    minval: Some(0.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.);
        Ok(DebounceButton {
            reactor: printer.get_reactor(),
            debounce_delay: delay,
            button_action,
            inner: Arc::new(GCell::new(DebounceInner {
                logical_state: ButtonState::Empty,
                physical_state: ButtonState::Empty,
                latest_eventtime: None,
            })),
        })
    }

    /// `button_handler(eventtime, state)`.
    pub fn button_handler(&self, eventtime: f64, state: ButtonState) {
        let trigger_time = eventtime + self.debounce_delay;
        let equal = {
            let mut inner = self.inner.borrow_mut();
            inner.physical_state = state;
            inner.latest_eventtime = Some(eventtime);
            inner.logical_state == inner.physical_state
        };
        // if there would be no state transition, ignore the event
        if equal {
            return;
        }
        let db = self.clone();
        self.reactor
            .register_callback(move |et| db.debounce_event(et), trigger_time);
    }

    /// `_debounce_event(eventtime)`.
    pub fn debounce_event(&self, eventtime: f64) {
        let (logical, physical, latest) = {
            let inner = self.inner.borrow();
            (inner.logical_state, inner.physical_state, inner.latest_eventtime)
        };
        // if there would be no state transition, ignore the event
        if logical == physical {
            return;
        }
        let latest = match latest {
            Some(t) => t,
            // Python raises TypeError here (`<` against `None`); only
            // reachable by calling this before any button event.
            None => return,
        };
        // if there were more recent events, they supersede this one
        if (eventtime - self.debounce_delay) < latest {
            return;
        }
        // enact state transition and trigger action
        self.inner.borrow_mut().logical_state = physical;
        (self.button_action)(latest, physical);
    }
}

// ---------------------------------------------------------------------------
// PrinterButtons
// ---------------------------------------------------------------------------

/// `PrinterButtons` (`buttons.py:284`): `[buttons]` section object used
/// to register buttons with the printer.
pub struct PrinterButtons {
    printer: Printer,
    mcu_buttons: GCell<IndexMap<String, Arc<McuButtons>>>,
    adc_buttons: GCell<IndexMap<String, Arc<McuAdcButtons>>>,
}

/// The interface other modules call on the `buttons` object
/// (`PrinterButtons`, `buttons.py:284`): `register_adc_button`,
/// `register_debounce_button`, `register_debounce_adc_button`,
/// `register_adc_button_push`, `register_buttons`,
/// `register_rotary_encoder` and `register_button_push`, used by
/// `gcode_button.py`, `display/menu_keys.py`,
/// `filament_motion_sensor.py` and `filament_switch_sensor.py`.
pub trait PrinterButtonsApi: Send + Sync + 'static {
    /// `register_adc_button(self, pin, min_val, max_val, pullup, callback)`
    /// (`buttons.py:290`).
    fn register_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: AdcButtonCallback,
    ) -> KResult<()>;
    /// `register_debounce_button(self, pin, callback, config)`
    /// (`buttons.py:296`).
    fn register_debounce_button(
        &self,
        pin: &str,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()>;
    /// `register_debounce_adc_button(self, pin, min_val, max_val, pullup,
    /// callback, config)` (`buttons.py:299`).
    fn register_debounce_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()>;
    /// `register_adc_button_push(self, pin, min_val, max_val, pullup,
    /// callback)` (`buttons.py:304`).
    fn register_adc_button_push(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: PushCallback,
    ) -> KResult<()>;
    /// `register_buttons(self, pins, callback)` (`buttons.py:309`).
    fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) -> KResult<()>;
    /// `register_rotary_encoder(self, pin1, pin2, cw_callback, ccw_callback,
    /// steps_per_detent)` (`buttons.py:328`).
    fn register_rotary_encoder(
        &self,
        pin1: &str,
        pin2: &str,
        cw_callback: PushCallback,
        ccw_callback: PushCallback,
        steps_per_detent: i64,
    ) -> KResult<()>;
    /// `register_button_push(self, pin, callback)` (`buttons.py:338`).
    fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()>;
}

impl PrinterButtonsApi for PrinterButtons {
    fn register_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: AdcButtonCallback,
    ) -> KResult<()> {
        PrinterButtons::register_adc_button(self, pin, min_val, max_val, pullup, callback)
    }
    fn register_debounce_button(
        &self,
        pin: &str,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()> {
        PrinterButtons::register_debounce_button(self, pin, callback, config)
    }
    fn register_debounce_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()> {
        PrinterButtons::register_debounce_adc_button(
            self, pin, min_val, max_val, pullup, callback, config,
        )
    }
    fn register_adc_button_push(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: PushCallback,
    ) -> KResult<()> {
        PrinterButtons::register_adc_button_push(self, pin, min_val, max_val, pullup, callback)
    }
    fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) -> KResult<()> {
        PrinterButtons::register_buttons(self, pins, callback)
    }
    fn register_rotary_encoder(
        &self,
        pin1: &str,
        pin2: &str,
        cw_callback: PushCallback,
        ccw_callback: PushCallback,
        steps_per_detent: i64,
    ) -> KResult<()> {
        PrinterButtons::register_rotary_encoder(
            self, pin1, pin2, cw_callback, ccw_callback, steps_per_detent,
        )
    }
    fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()> {
        PrinterButtons::register_button_push(self, pin, callback)
    }
}

klippy_api::printer_object!(PrinterButtons; dyn PrinterButtonsApi);

impl PrinterButtons {
    /// `PrinterButtons(config)`.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let _ = printer.load_object(config, "query_adc", LookupDefault::Raise)?;
        Ok(Arc::new(PrinterButtons {
            printer,
            mcu_buttons: GCell::new(IndexMap::new()),
            adc_buttons: GCell::new(IndexMap::new()),
        }))
    }

    /// `register_adc_button(pin, min_val, max_val, pullup, callback)`.
    pub fn register_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: AdcButtonCallback,
    ) -> KResult<()> {
        let buttons = {
            let mut map = self.adc_buttons.borrow_mut();
            match map.get(pin) {
                Some(existing) => Arc::clone(existing),
                None => {
                    let created = McuAdcButtons::new(&self.printer, pin, pullup)?;
                    map.insert(pin.to_string(), Arc::clone(&created));
                    created
                }
            }
        };
        buttons.setup_button(min_val, max_val, callback);
        Ok(())
    }

    /// `register_debounce_button(pin, callback, config)`.
    pub fn register_debounce_button(
        &self,
        pin: &str,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()> {
        let debounce = DebounceButton::new(config, callback)?;
        let db = debounce.clone();
        let handler: ButtonCallback =
            Arc::new(move |et, state| db.button_handler(et, ButtonState::Digital(state)));
        self.register_buttons(vec![pin.to_string()], handler)
    }

    /// `register_debounce_adc_button(pin, min_val, max_val, pullup, callback, config)`.
    pub fn register_debounce_adc_button(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()> {
        let debounce = DebounceButton::new(config, callback)?;
        let db = debounce.clone();
        let handler: AdcButtonCallback =
            Arc::new(move |et, state| db.button_handler(et, ButtonState::Analog(state)));
        self.register_adc_button(pin, min_val, max_val, pullup, handler)
    }

    /// `register_adc_button_push(pin, min_val, max_val, pullup, callback)`.
    pub fn register_adc_button_push(
        &self,
        pin: &str,
        min_val: f64,
        max_val: f64,
        pullup: f64,
        callback: PushCallback,
    ) -> KResult<()> {
        let cb = Arc::clone(&callback);
        let helper: AdcButtonCallback = Arc::new(move |et, state| {
            if state {
                cb(et);
            }
        });
        self.register_adc_button(pin, min_val, max_val, pullup, helper)
    }

    /// `register_buttons(pins, callback)`.
    pub fn register_buttons(&self, pins: Vec<String>, callback: ButtonCallback) -> KResult<()> {
        // Parse pins
        let ppins = lookup_interface::<dyn PinsApi>(&*self.printer, "pins", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let mut mcu: Option<Mcu> = None;
        let mut mcu_name: Option<String> = None;
        let mut pin_params_list: Vec<PinParams> = Vec::new();
        for pin in pins {
            let pin_params = ppins.lookup_pin(
                &pin,
                LookupPinOpts { can_invert: true, can_pullup: true, share_type: None },
            )?;
            if let Some(expected) = &mcu {
                if !Arc::ptr_eq(expected, &pin_params.chip) {
                    return Err(KlippyError::Config(
                        "button pins must be on same mcu".to_string(),
                    ));
                }
            }
            mcu = Some(pin_params.chip.clone());
            mcu_name = Some(pin_params.chip_name.clone());
            pin_params_list.push(pin_params);
        }
        // `register_buttons([])` would construct `MCU_buttons(printer, None)`
        // in Python and raise AttributeError on the config callback.
        let mcu = match mcu {
            Some(mcu) => mcu,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'register_config_callback'"
                        .to_string(),
                });
            }
        };
        let mcu_name = mcu_name.unwrap_or_default();
        // Register pins and callback with the appropriate MCU
        let mcu_buttons = {
            let mut map = self.mcu_buttons.borrow_mut();
            let reuse = map
                .get(&mcu_name)
                .map(|existing| existing.pin_count() + pin_params_list.len() <= 8)
                .unwrap_or(false);
            if reuse {
                Arc::clone(map.get(&mcu_name).unwrap())
            } else {
                let created = McuButtons::new(&self.printer, mcu)?;
                map.insert(mcu_name.clone(), Arc::clone(&created));
                created
            }
        };
        mcu_buttons.setup_buttons(pin_params_list, callback);
        Ok(())
    }

    /// `register_rotary_encoder(pin1, pin2, cw_callback, ccw_callback, steps_per_detent)`.
    pub fn register_rotary_encoder(
        &self,
        pin1: &str,
        pin2: &str,
        cw_callback: PushCallback,
        ccw_callback: PushCallback,
        steps_per_detent: i64,
    ) -> KResult<()> {
        let encoder = if steps_per_detent == 2 {
            RotaryEncoder::half_step(cw_callback, ccw_callback)
        } else if steps_per_detent == 4 {
            RotaryEncoder::full_step(cw_callback, ccw_callback)
        } else {
            return Err(KlippyError::Config(format!(
                "{} steps per detent not supported",
                steps_per_detent
            )));
        };
        let re = Arc::new(encoder);
        let cb: ButtonCallback = Arc::new(move |et, state| re.encoder_callback(et, state));
        self.register_buttons(vec![pin1.to_string(), pin2.to_string()], cb)
    }

    /// `register_button_push(pin, callback)`.
    pub fn register_button_push(&self, pin: &str, callback: PushCallback) -> KResult<()> {
        let cb = Arc::clone(&callback);
        let helper: ButtonCallback = Arc::new(move |et, state| {
            if state != 0 {
                cb(et);
            }
        });
        self.register_buttons(vec![pin.to_string()], helper)
    }
}

// ---------------------------------------------------------------------------
// Module factory
// ---------------------------------------------------------------------------

/// `load_config(config)` (`buttons.py:344`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let buttons = PrinterButtons::new(config)?;
    Ok(Some(buttons as ObjectRef))
}

/// The module entry points (`load_config` only) for section `buttons`.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
