//! Port of the `controller_fan` module (`klippy/extras/controller_fan.py`).
//!
//! A `[controller_fan <name>]` section drives a `fan.Fan` whenever any of
//! the configured steppers is enabled or any of the configured heaters has
//! a non-zero target temperature. After activity stops the fan runs at
//! `idle_speed` for `idle_timeout` seconds and then stops.
//!
//! The objects this module is duck-typed against come from their owning
//! modules: `crate::fan::Fan` (`klippy/extras/fan.py:8`),
//! `crate::stepper_enable::PrinterStepperEnable` / `EnableTracking`
//! (`klippy/extras/stepper_enable.py:74` / `:45`), and
//! `crate::heaters::PrinterHeaters` / `Heater` (`klippy/extras/heaters.py`).

use klippy_api::config::{Config, GetFloatOpts, GetIntOpts, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT, EV_KLIPPY_READY,
};
use klippy_api::{
    load_interface, lookup_interface, EventArg, GCell, ObjectRef, PrinterObject,
};
use pycompat::PyValue;
use std::sync::Arc;

use crate::fan::Fan;
use crate::heaters::{Heater, HeaterApi, PrinterHeatersApi};
use crate::stepper_enable::PrinterStepperEnableApi;

/// `PIN_MIN_TIME` (`klippy/extras/controller_fan.py:8`).
pub const PIN_MIN_TIME: f64 = 0.100;

/// `ControllerFan` (`klippy/extras/controller_fan.py:10`).
pub struct ControllerFan {
    printer: Printer,
    /// `stepper_names`: `None` means "all steppers" (config `stepper`
    /// unset); `handle_connect` replaces it with the full list.
    stepper_names: GCell<Option<Vec<String>>>,
    stepper_enable: Arc<dyn PrinterStepperEnableApi>,
    /// `heaters`: filled by `handle_connect`.
    heaters: GCell<Vec<Arc<Heater>>>,
    heater_names: Vec<String>,
    fan: Arc<Fan>,
    fan_speed: f64,
    idle_speed: f64,
    idle_timeout: i64,
    last_on: GCell<i64>,
    last_speed: GCell<f64>,
}

impl ControllerFan {
    /// `ControllerFan.__init__(config)` (`klippy/extras/controller_fan.py:11`).
    ///
    /// The `klippy:ready` and `klippy:connect` handlers are registered with
    /// the printer (in that order) before this returns, exactly as in the
    /// Python constructor. Returns the ready-to-register object.
    pub fn new(config: &Config) -> KResult<Arc<ControllerFan>> {
        let printer = config.get_printer();
        // `config.getlist("stepper", None)` (controller_fan.py:16).
        let stepper_names = config.getlist(
            "stepper",
            GetListOpts { default: Some(None), ..Default::default() },
        )?;
        // `self.printer.load_object(config, 'stepper_enable')` (:17).
        let stepper_enable = load_interface::<dyn PrinterStepperEnableApi>(
            &*printer,
            config,
            "stepper_enable",
            LookupDefault::Raise,
        )?
        .expect("stepper_enable lookup with Raise never returns None");
        // `self.printer.load_object(config, 'heaters')` (:18): ensures the
        // object exists; the result is discarded here (re-looked-up at
        // `klippy:connect`, like the Python code).
        let _ = load_interface::<dyn PrinterHeatersApi>(
            &*printer,
            config,
            "heaters",
            LookupDefault::Raise,
        )?;
        // `self.fan = fan.Fan(config)` (:20) precedes these reads.
        let fan = Fan::new(config, 0.0)?;
        // `config.getfloat('fan_speed', default=1., minval=0., maxval=1.)` (:21-22).
        let fan_speed = config
            .getfloat(
                "fan_speed",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    minval: Some(0.0),
                    maxval: Some(1.0),
                    ..Default::default()
                },
            )?
            .expect("fan_speed has a default");
        // `config.getfloat('idle_speed', default=self.fan_speed, ...)` (:23-24).
        let idle_speed = config
            .getfloat(
                "idle_speed",
                GetFloatOpts {
                    default: Some(Some(fan_speed)),
                    minval: Some(0.0),
                    maxval: Some(1.0),
                    ..Default::default()
                },
            )?
            .expect("idle_speed has a default");
        // `config.getint("idle_timeout", default=30, minval=0)` (:25).
        let idle_timeout = config
            .getint(
                "idle_timeout",
                GetIntOpts {
                    default: Some(Some(30)),
                    minval: Some(0),
                    ..Default::default()
                },
            )?
            .expect("idle_timeout has a default");
        // `config.getlist("heater", ("extruder",))` (:26).
        let heater_names = config
            .getlist(
                "heater",
                GetListOpts {
                    default: Some(Some(vec!["extruder".to_string()])),
                    ..Default::default()
                },
            )?
            .expect("heater has a default");
        let this = Arc::new(ControllerFan {
            printer: printer.clone(),
            stepper_names: GCell::new(stepper_names),
            stepper_enable,
            heaters: GCell::new(Vec::new()),
            heater_names,
            fan,
            fan_speed,
            idle_speed,
            idle_timeout,
            // `self.last_on = self.idle_timeout` (:27) and
            // `self.last_speed = 0.` (:28).
            last_on: GCell::new(idle_timeout),
            last_speed: GCell::new(0.0),
        });
        let ready = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_: &[EventArg]| {
                ControllerFan::handle_ready(&ready);
                Ok(PyValue::None)
            }),
        );
        let connect = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_: &[EventArg]| {
                connect.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        Ok(this)
    }

    /// `handle_connect()` (`klippy/extras/controller_fan.py:29`).
    ///
    /// Resolves the heaters for the configured heater names, then (when the
    /// config `stepper` list was unset) adopts every stepper known to
    /// `stepper_enable`. A configured stepper name unknown to
    /// `stepper_enable` raises `printer.config_error`.
    pub fn handle_connect(&self) -> KResult<()> {
        // Heater lookup (:31-32).
        let pheaters = lookup_interface::<dyn PrinterHeatersApi>(
            &*self.printer,
            "heaters",
            LookupDefault::Raise,
        )?
        .expect("heaters lookup with Raise never returns None");
        let mut heaters = Vec::with_capacity(self.heater_names.len());
        for name in &self.heater_names {
            heaters.push(pheaters.lookup_heater(name)?);
        }
        *self.heaters.borrow_mut() = heaters;
        // Stepper lookup (:34-42).
        let all_steppers = self.stepper_enable.get_steppers();
        if self.stepper_names.borrow().is_none() {
            *self.stepper_names.borrow_mut() = Some(all_steppers);
            return Ok(());
        }
        let unknown = {
            let names = self.stepper_names.borrow();
            names
                .as_ref()
                .expect("stepper_names is Some here")
                .iter()
                .any(|x| !all_steppers.contains(x))
        };
        if unknown {
            let names = self.stepper_names.borrow();
            let names = names.as_ref().expect("stepper_names is Some here");
            // `"... unknown: %s (valid steppers are: %s)" % (self.stepper_names,
            // ", ".join(all_steppers))` (:39-42): `%s` of the `getlist`
            // tuple of names (`klippy/configfile.py:102` returns a tuple).
            let names_tuple = PyValue::Tuple(
                names.iter().map(|n| PyValue::Str(n.clone())).collect(),
            );
            return Err(KlippyError::config(format!(
                "One or more of these steppers are unknown: {} (valid steppers are: {})",
                names_tuple.str(),
                all_steppers.join(", "),
            )));
        }
        Ok(())
    }

    /// `handle_ready()` (`klippy/extras/controller_fan.py:43`): starts the
    /// 1 Hz fan timer after `PIN_MIN_TIME` so the pins have settled.
    ///
    /// Takes the `Arc` because the timer closure must own the controller
    /// for the rest of the run: `register_timer` requires a `'static`
    /// callback, so a bare `&self` cannot be used.
    pub fn handle_ready(this: &Arc<ControllerFan>) {
        let reactor = this.printer.get_reactor();
        let wake = reactor.monotonic() + PIN_MIN_TIME;
        let timer_this = this.clone();
        reactor.register_timer(
            move |eventtime| {
                // The timer dispatch cannot propagate the error; a failing
                // controller timer aborts the run in both worlds (Python
                // raises out of `reactor.run()`, here the dispatch thread
                // panics with the same class and message).
                match timer_this.callback(eventtime) {
                    Ok(next) => next,
                    Err(e) => panic!(
                        "controller_fan callback: {}: {}",
                        e.class_name(),
                        e.message()
                    ),
                }
            },
            wake,
        );
    }

    /// `get_status(eventtime)` (`klippy/extras/controller_fan.py:46`):
    /// straight pass-through to the fan.
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.fan.get_status(eventtime)
    }
    pub fn callback(&self, eventtime: f64) -> KResult<f64> {
        let mut speed = 0.0;
        let mut active = false;
        // `stepper_enable.lookup_enable(name).is_motor_enabled()` for every
        // configured name: `|=` evaluates every operand. (`for name in
        // self.stepper_names` raises TypeError before `handle_connect`,
        // exactly like the Python original.)
        let names = self.stepper_names.borrow().clone();
        let names = match names {
            Some(names) => names,
            None => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "'NoneType' object is not iterable".to_string(),
                });
            }
        };
        for name in &names {
            active |= self.stepper_enable.lookup_enable(name)?.is_motor_enabled();
        }
        // `_, target_temp = heater.get_temp(eventtime)` for every heater;
        // `if target_temp:` means non-zero (also true for NaN).
        let heaters = self.heaters.borrow().clone();
        for heater in &heaters {
            let (_, target_temp) = heater.get_temp(eventtime);
            if target_temp != 0.0 {
                active = true;
            }
        }
        if active {
            self.last_on.set(0);
            speed = self.fan_speed;
        } else if self.last_on.get() < self.idle_timeout {
            speed = self.idle_speed;
            self.last_on.set(self.last_on.get() + 1);
        }
        if speed != self.last_speed.get() {
            self.last_speed.set(speed);
            self.fan.set_speed(speed, None)?;
        }
        Ok(eventtime + 1.0)
    }
}

impl PrinterObject for ControllerFan {
    klippy_api::printer_object_base!();

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(ControllerFan::get_status(self, eventtime)))
    }
}

/// `load_config_prefix(config)` (`klippy/extras/controller_fan.py:68`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(ControllerFan::new(config)? as ObjectRef))
}

/// Module entry point for the `[controller_fan <name>]` section. The Python
/// module defines `load_config_prefix` only (`controller_fan.py:68`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
