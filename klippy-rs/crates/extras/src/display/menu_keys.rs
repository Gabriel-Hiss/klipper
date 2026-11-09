//! Port of `klippy/extras/display/menu_keys.py` (display menu button
//! press tracking, rotary encoder fast-rate, click/long-press and the
//! kill button).
//!
//! Python has no `load_config` / `load_config_prefix` in this module:
//! `menu.py` constructs [`MenuKeys`] directly and passes its own
//! `key_event` as the menu callback, so [`module_factory`] returns a
//! factory with every entry point unset, like the Python module.
//!
//! # Buttons dependency
//!
//! [`MenuKeys`] resolves the `buttons` printer object through
//! [`klippy_api::load_interface`] to
//! [`crate::buttons::PrinterButtonsApi`] exactly like Python's
//! `printer.load_object(config, "buttons")`, and reuses that handle for
//! every registration (Python re-looks-up the same registry object
//! inside each `register_button` call; the observable call order is
//! unchanged).
//!
//! Callback arities mirror Python bound methods:
//! - encoder callbacks take `(eventtime)` ([`PushCallback`]);
//! - push-button callbacks take `(eventtime)` ([`PushCallback`]);
//! - the click button callback takes `(eventtime, state)`; the digital
//!   [`ButtonCallback`] state (`i64`) is truth-tested (`state != 0`)
//!   before reaching [`MenuKeys::click_callback`], which takes `bool`
//!   and is also the [`AdcButtonCallback`] state.
//!
//! `TIMER_DELAY` is defined by the Python module but never used there;
//! it is kept for parity.

use std::sync::Arc;

use klippy_api::config::{Config, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::load_interface;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER};

use crate::buttons::{AdcButtonCallback, ButtonCallback, PrinterButtonsApi, PushCallback};

/// `LONG_PRESS_DURATION = 0.800` (`menu_keys.py:9`).
pub const LONG_PRESS_DURATION: f64 = 0.8;

/// `TIMER_DELAY = .200` (`menu_keys.py:10`); defined but unused by the
/// Python module.
pub const TIMER_DELAY: f64 = 0.2;

/// Menu key callback `callback(key, eventtime)` (`menu.py::key_event`).
pub type MenuKeyCallback = Arc<dyn Fn(&str, f64) + Send + Sync>;

/// A button callback in either of the arities `buttons.py` uses.
pub enum AnyButtonCallback {
    /// `(eventtime)`: `back`/`up`/`down`/`kill` push buttons.
    Push(PushCallback),
    /// `(eventtime, state)`: the click button.
    State(AdcButtonCallback),
}

/// Module factory: the Python module exposes no `load_config`-family
/// entry points, so every field stays unset.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}

/// `MenuKeys` (`menu_keys.py:12`).
pub struct MenuKeys {
    printer: Printer,
    reactor: Reactor,
    callback: MenuKeyCallback,
    buttons: Arc<dyn PrinterButtonsApi>,
    encoder_fast_rate: GCell<f64>,
    last_encoder_cw_eventtime: GCell<f64>,
    last_encoder_ccw_eventtime: GCell<f64>,
    is_short_click: GCell<bool>,
    click_timer: GCell<Option<Timer>>,
}

fn arity_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: "button callback arity does not match push_only".to_string(),
    }
}

fn pyvalue_to_i64(value: &PyValue) -> KResult<i64> {
    match value {
        PyValue::Int(n) => n
            .0
            .to_string()
            .parse::<i64>()
            .map_err(|_| KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "expected int".to_string(),
            }),
        _ => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "expected int".to_string(),
        }),
    }
}

impl MenuKeys {
    /// `MenuKeys(config, callback)` (`menu_keys.py:13`): loads the
    /// `buttons` object through the [`PrinterButtonsApi`] interface
    /// and registers every button.
    pub fn new(config: &Config, callback: MenuKeyCallback) -> KResult<Arc<MenuKeys>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let buttons = load_interface::<dyn PrinterButtonsApi>(
            &*printer,
            config,
            "buttons",
            LookupDefault::Raise,
        )?
        .ok_or_else(|| klippy_api::missing_interface::<dyn PrinterButtonsApi>("buttons"))?;
        // The `Arc` exists before any registration so callbacks can
        // capture it; fields the Python constructor assigns later are
        // set in the same order as the Python statements below.
        let keys = Arc::new(MenuKeys {
            printer,
            reactor: reactor.clone(),
            callback,
            buttons: Arc::clone(&buttons),
            encoder_fast_rate: GCell::new(0.0),
            last_encoder_cw_eventtime: GCell::new(0.0),
            last_encoder_ccw_eventtime: GCell::new(0.0),
            is_short_click: GCell::new(false),
            click_timer: GCell::new(None),
        });
        // Register rotary encoder (`menu_keys.py:18-30`).
        let encoder_pins = config.get(
            "encoder_pins",
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        let steps = config.getchoice(
            "encoder_steps_per_detent",
            &PyValue::List(vec![
                PyValue::Int(PyInt::from_i64(2)),
                PyValue::Int(PyInt::from_i64(4)),
            ]),
            GetChoiceOpts {
                default: Some(Some(PyValue::Int(PyInt::from_i64(4)))),
                ..Default::default()
            },
        )?;
        let steps_per_detent = pyvalue_to_i64(&steps)?;
        if let Some(encoder_pins) = encoder_pins {
            let parts: Vec<&str> = encoder_pins.split(',').collect();
            if parts.len() != 2 {
                return Err(KlippyError::config("Unable to parse encoder_pins"));
            }
            let pin1 = parts[0].trim();
            let pin2 = parts[1].trim();
            let keys_cw = Arc::clone(&keys);
            let cw: PushCallback = Arc::new(move |et| keys_cw.encoder_cw_callback(et));
            let keys_ccw = Arc::clone(&keys);
            let ccw: PushCallback = Arc::new(move |et| keys_ccw.encoder_ccw_callback(et));
            buttons.register_rotary_encoder(pin1, pin2, cw, ccw, steps_per_detent)?;
        }
        // `encoder_fast_rate` (`menu_keys.py:31-32`).
        let fast_rate = config
            .getfloat(
                "encoder_fast_rate",
                GetFloatOpts {
                    default: Some(Some(0.03)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.03);
        keys.encoder_fast_rate.set(fast_rate);
        // Click timer (`menu_keys.py:36-37`); `register_timer` defaults
        // to `NEVER` in Python (`reactor.py:226`).
        let keys_timer = Arc::clone(&keys);
        let timer = reactor.register_timer(
            move |et| keys_timer.long_click_event(et),
            NEVER,
        );
        keys.click_timer.replace(Some(timer));
        // Buttons (`menu_keys.py:38-43`).
        let keys_click = Arc::clone(&keys);
        keys.register_button(
            config,
            "click_pin",
            AnyButtonCallback::State(Arc::new(move |et, st| {
                keys_click.click_callback(et, st)
            })),
            false,
        )?;
        let keys_back = Arc::clone(&keys);
        keys.register_button(
            config,
            "back_pin",
            AnyButtonCallback::Push(Arc::new(move |et| keys_back.back_callback(et))),
            true,
        )?;
        let keys_up = Arc::clone(&keys);
        keys.register_button(
            config,
            "up_pin",
            AnyButtonCallback::Push(Arc::new(move |et| keys_up.up_callback(et))),
            true,
        )?;
        let keys_down = Arc::clone(&keys);
        keys.register_button(
            config,
            "down_pin",
            AnyButtonCallback::Push(Arc::new(move |et| keys_down.down_callback(et))),
            true,
        )?;
        let keys_kill = Arc::clone(&keys);
        keys.register_button(
            config,
            "kill_pin",
            AnyButtonCallback::Push(Arc::new(move |et| keys_kill.kill_callback(et))),
            true,
        )?;
        Ok(keys)
    }

    /// `register_button(config, name, callback, push_only=True)`
    /// (`menu_keys.py:45-61`).
    pub fn register_button(
        &self,
        config: &Config,
        name: &str,
        callback: AnyButtonCallback,
        push_only: bool,
    ) -> KResult<()> {
        let pin = config.get(
            name,
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        let Some(pin) = pin else {
            return Ok(());
        };
        let analog_name = format!("analog_range_{name}");
        let analog = config.get(
            &analog_name,
            GetStrOpts { default: Some(None), ..Default::default() },
        )?;
        if analog.is_none() {
            return match (push_only, callback) {
                (true, AnyButtonCallback::Push(cb)) => {
                    self.buttons.register_button_push(&pin, cb)
                }
                (false, AnyButtonCallback::State(cb)) => {
                    let digital: ButtonCallback =
                        Arc::new(move |et, st: i64| cb(et, st != 0));
                    self.buttons.register_buttons(vec![pin], digital)
                }
                _ => Err(arity_error()),
            };
        }
        let range = config.getfloatlist(
            &analog_name,
            GetFloatListOpts { default: None, count: Some(2), ..Default::default() },
        )?;
        let Some(range) = range else {
            return Err(KlippyError::config(format!(
                "Option '{analog_name}' in section '{}' must have 2 elements",
                config.get_name()
            )));
        };
        if range.len() != 2 {
            return Err(KlippyError::config(format!(
                "Option '{analog_name}' in section '{}' must have 2 elements",
                config.get_name()
            )));
        }
        let (amin, amax) = (range[0], range[1]);
        let pullup = config
            .getfloat(
                "analog_pullup_resistor",
                GetFloatOpts {
                    default: Some(Some(4700.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(4700.0);
        match (push_only, callback) {
            (true, AnyButtonCallback::Push(cb)) => {
                self.buttons.register_adc_button_push(&pin, amin, amax, pullup, cb)
            }
            (false, AnyButtonCallback::State(cb)) => {
                self.buttons.register_adc_button(&pin, amin, amax, pullup, cb)
            }
            _ => Err(arity_error()),
        }
    }

    /// `encoder_cw_callback(eventtime)` (`menu_keys.py:64-71`).
    pub fn encoder_cw_callback(&self, eventtime: f64) {
        let fast_rate =
            (eventtime - self.last_encoder_cw_eventtime.get())
                <= self.encoder_fast_rate.get();
        self.last_encoder_cw_eventtime.set(eventtime);
        if fast_rate {
            (self.callback)("fast_up", eventtime);
        } else {
            (self.callback)("up", eventtime);
        }
    }

    /// `encoder_ccw_callback(eventtime)` (`menu_keys.py:73-80`).
    pub fn encoder_ccw_callback(&self, eventtime: f64) {
        let fast_rate =
            (eventtime - self.last_encoder_ccw_eventtime.get())
                <= self.encoder_fast_rate.get();
        self.last_encoder_ccw_eventtime.set(eventtime);
        if fast_rate {
            (self.callback)("fast_down", eventtime);
        } else {
            (self.callback)("down", eventtime);
        }
    }

    /// `long_click_event(eventtime)` (`menu_keys.py:83-86`); returns the
    /// reactor `NEVER` waketime like the Python timer callback.
    pub fn long_click_event(&self, eventtime: f64) -> f64 {
        self.is_short_click.set(false);
        (self.callback)("long_click", eventtime);
        NEVER
    }

    /// `click_callback(eventtime, state)` (`menu_keys.py:88-95`). The
    /// release branch intentionally keeps `is_short_click` set, matching
    /// Python (only `long_click_event` clears it).
    pub fn click_callback(&self, eventtime: f64, state: bool) {
        if state {
            self.is_short_click.set(true);
            let waketime = eventtime + LONG_PRESS_DURATION;
            let reactor = self.reactor.clone();
            self.click_timer.with_mut(|slot| {
                if let Some(timer) = slot.as_ref() {
                    reactor.update_timer(timer, waketime);
                }
            });
        } else if self.is_short_click.get() {
            let reactor = self.reactor.clone();
            self.click_timer.with_mut(|slot| {
                if let Some(timer) = slot.as_ref() {
                    reactor.update_timer(timer, NEVER);
                }
            });
            (self.callback)("click", eventtime);
        }
    }

    /// `back_callback(eventtime)` (`menu_keys.py:98-99`).
    pub fn back_callback(&self, eventtime: f64) {
        (self.callback)("back", eventtime);
    }

    /// `up_callback(eventtime)` (`menu_keys.py:101-102`).
    pub fn up_callback(&self, eventtime: f64) {
        (self.callback)("up", eventtime);
    }

    /// `down_callback(eventtime)` (`menu_keys.py:104-105`).
    pub fn down_callback(&self, eventtime: f64) {
        (self.callback)("down", eventtime);
    }

    /// `kill_callback(eventtime)` (`menu_keys.py:107-108`).
    pub fn kill_callback(&self, _eventtime: f64) {
        self.printer
            .invoke_shutdown("Shutdown due to kill button!", PyValue::Dict(Default::default()));
    }
}
