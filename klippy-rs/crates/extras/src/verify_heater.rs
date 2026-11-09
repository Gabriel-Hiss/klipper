//! Port of the `verify_heater` module.
//!
//! Source: `klippy/extras/verify_heater.py` (all of it).
//!
//! A `verify_heater <heater>` section watches the named heater's
//! temperature. On `klippy:connect` (skipped entirely when the printer
//! started with a `debugoutput` file) it looks the heater up in the
//! `heaters` object and arms a once-per-second check timer. The check
//! invokes a printer shutdown when the heater fails to approach its
//! target at the expected rate or fails to maintain it.

use crate::heaters::{HeaterApi, PrinterHeatersApi};
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log;
use klippy_api::object::{lookup_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT, EV_KLIPPY_SHUTDOWN};
use klippy_api::GCell;
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER, NOW};
use std::sync::Arc;

/// Hint appended to the fault shutdown message (`verify_heater.py:8-11`).
pub const HINT_THERMAL: &str = "\nSee the 'verify_heater' section in docs/Config_Reference.md\nfor the parameters that control this check.\n";

/// Mutable per-instance state (`verify_heater.py:30-33`).
struct State {
    approaching_target: bool,
    starting_approach: bool,
    last_target: f64,
    goal_temp: f64,
    error: f64,
    goal_systime: f64,
}

/// Shared inner state of [`HeaterCheck`]. The event handlers and the
/// reactor timer callback each need an owned handle to the same state,
/// so it lives in an `Arc` (all shared state is in `GCell`s, as the
/// reactor runs one greenlet at a time).
struct Inner {
    printer: Printer,
    reactor: Reactor,
    heater_name: String,
    hysteresis: f64,
    max_error: f64,
    heating_gain: f64,
    check_gain_time: f64,
    /// `self.heater` (`verify_heater.py:21`); set by `handle_connect`.
    heater: GCell<Option<Arc<dyn HeaterApi>>>,
    state: GCell<State>,
    /// `self.check_timer` (`verify_heater.py:33`).
    check_timer: GCell<Option<Timer>>,
}

/// The `verify_heater` printer object (`HeaterCheck`, `verify_heater.py:13`).
///
/// The Python object is stored directly in the printer registry under
/// the full section name; the `klippy:connect` / `klippy:shutdown`
/// handlers and the reactor timer all need an owned handle to the
/// same state, so state lives in an `Arc<Inner>`.
pub struct HeaterCheck {
    inner: Arc<Inner>,
}

klippy_api::printer_object!(HeaterCheck);

/// `dict.get('debugoutput') is None` (`verify_heater.py:35`): the
/// Python `start_args` dict has no `debugoutput` entry, or the entry
/// is `None`.
fn start_arg_none(args: &PyValue, key: &str) -> bool {
    let value = match args {
        PyValue::Dict(map) => map.get(&PyValue::Str(key.to_string())),
        _ => None,
    };
    matches!(value, None | Some(PyValue::None))
}

/// `config.getfloat(option, default, ...)` (`configfile.py:68`); the
/// Python call always yields a value because every option here has a
/// default.
fn get_float(
    config: &Config,
    option: &str,
    default: f64,
    minval: Option<f64>,
    above: Option<f64>,
) -> KResult<f64> {
    let opts = GetFloatOpts {
        default: Some(Some(default)),
        minval,
        maxval: None,
        above,
        below: None,
        note_valid: true,
        ..Default::default()
    };
    Ok(config.getfloat(option, opts)?.unwrap_or(default))
}

fn percent(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant verify_heater format")
}

impl HeaterCheck {
    /// `HeaterCheck(config)` (`verify_heater.py:14-33`).
    pub fn new(config: &Config) -> KResult<HeaterCheck> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        // `config.get_name().split()[1]` (`verify_heater.py:20`).
        let heater_name = match config.get_name().split_whitespace().nth(1) {
            Some(n) => n.to_string(),
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let hysteresis = get_float(config, "hysteresis", 5., Some(0.), None)?;
        let max_error = get_float(config, "max_error", 120., Some(0.), None)?;
        let heating_gain = get_float(config, "heating_gain", 2., None, Some(0.))?;
        // `default_gain_time` (`verify_heater.py:25-27`).
        let default_gain_time = if heater_name == "heater_bed" { 60. } else { 20. };
        let check_gain_time = get_float(config, "check_gain_time", default_gain_time, Some(1.), None)?;
        let inner = Arc::new(Inner {
            printer: printer.clone(),
            reactor,
            heater_name,
            hysteresis,
            max_error,
            heating_gain,
            check_gain_time,
            heater: GCell::new(None),
            state: GCell::new(State {
                approaching_target: false,
                starting_approach: false,
                last_target: 0.,
                goal_temp: 0.,
                error: 0.,
                goal_systime: NEVER,
            }),
            check_timer: GCell::new(None),
        });
        let conn_inner = Arc::clone(&inner);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                handle_connect(&conn_inner)?;
                Ok(PyValue::None)
            }),
        );
        let shutdown_inner = Arc::clone(&inner);
        printer.register_event_handler(
            EV_KLIPPY_SHUTDOWN,
            Box::new(move |_params| {
                handle_shutdown(&shutdown_inner);
                Ok(PyValue::None)
            }),
        );
        Ok(HeaterCheck { inner })
    }

    /// `self.heater_name` (`verify_heater.py:20`).
    pub fn heater_name(&self) -> &str {
        &self.inner.heater_name
    }

    /// `self.hysteresis` (`verify_heater.py:22`).
    pub fn hysteresis(&self) -> f64 {
        self.inner.hysteresis
    }

    /// `self.max_error` (`verify_heater.py:23`).
    pub fn max_error(&self) -> f64 {
        self.inner.max_error
    }

    /// `self.heating_gain` (`verify_heater.py:24`).
    pub fn heating_gain(&self) -> f64 {
        self.inner.heating_gain
    }

    /// `self.check_gain_time` (`verify_heater.py:28-29`).
    pub fn check_gain_time(&self) -> f64 {
        self.inner.check_gain_time
    }

    /// `self.approaching_target` (`verify_heater.py:30`).
    pub fn approaching_target(&self) -> bool {
        self.inner.state.borrow().approaching_target
    }

    /// `self.starting_approach` (`verify_heater.py:30`).
    pub fn starting_approach(&self) -> bool {
        self.inner.state.borrow().starting_approach
    }

    /// `self.last_target` (`verify_heater.py:31`).
    pub fn last_target(&self) -> f64 {
        self.inner.state.borrow().last_target
    }

    /// `self.goal_temp` (`verify_heater.py:31`).
    pub fn goal_temp(&self) -> f64 {
        self.inner.state.borrow().goal_temp
    }

    /// `self.error` (`verify_heater.py:31`).
    pub fn error(&self) -> f64 {
        self.inner.state.borrow().error
    }

    /// `self.goal_systime` (`verify_heater.py:32`).
    pub fn goal_systime(&self) -> f64 {
        self.inner.state.borrow().goal_systime
    }

    /// `self.heater is not None` (`verify_heater.py:21`).
    pub fn has_heater(&self) -> bool {
        self.inner.heater.borrow().is_some()
    }

    /// `self.check_timer is not None` (`verify_heater.py:33`).
    pub fn has_check_timer(&self) -> bool {
        self.inner.check_timer.borrow().is_some()
    }

    /// `self.handle_connect()` (`verify_heater.py:34-42`); also the
    /// `klippy:connect` event handler body.
    pub fn handle_connect(&self) -> KResult<()> {
        handle_connect(&self.inner)
    }

    /// `self.handle_shutdown()` (`verify_heater.py:43-46`); also the
    /// `klippy:shutdown` event handler body.
    pub fn handle_shutdown(&self) {
        handle_shutdown(&self.inner)
    }

    /// `self.check_event(eventtime)` (`verify_heater.py:47-85`); the
    /// reactor timer callback.
    pub fn check_event(&self, eventtime: f64) -> f64 {
        check_event(&self.inner, eventtime)
    }

    /// `self.heater_fault()` (`verify_heater.py:86-90`).
    pub fn heater_fault(&self) -> f64 {
        heater_fault(&self.inner)
    }
}

fn handle_connect(inner: &Arc<Inner>) -> KResult<()> {
    // `verify_heater.py:34-42`.
    if !start_arg_none(&inner.printer.get_start_args(), "debugoutput") {
        // Disable verify_heater if outputting to a debug file.
        return Ok(());
    }
    let pheaters = lookup_interface::<dyn PrinterHeatersApi>(&*inner.printer, "heaters", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None");
    let heater: Arc<dyn HeaterApi> = pheaters.lookup_heater(&inner.heater_name)?;
    *inner.heater.borrow_mut() = Some(heater);
    log::info(&percent(
        "Starting heater checks for %s",
        &[PctValue::Str(inner.heater_name.clone())],
    ));
    let timer_inner = Arc::clone(inner);
    let timer = inner
        .reactor
        .register_timer(move |eventtime| check_event(&timer_inner, eventtime), NOW);
    *inner.check_timer.borrow_mut() = Some(timer);
    Ok(())
}

fn handle_shutdown(inner: &Arc<Inner>) {
    // `verify_heater.py:43-46`.
    let timer = inner.check_timer.borrow();
    if let Some(timer) = timer.as_ref() {
        inner.reactor.update_timer(timer, NEVER);
    }
}

fn check_event(inner: &Arc<Inner>, eventtime: f64) -> f64 {
    // `verify_heater.py:47-85`.
    let (temp, target) = {
        let heater = inner.heater.borrow().clone();
        match heater {
            Some(h) => h.get_temp(eventtime),
            // `self.heater.get_temp` on a never-connected instance
            // raises `AttributeError: 'NoneType' object has no
            // attribute 'get_temp'` in Python.
            None => panic!("AttributeError: 'NoneType' object has no attribute 'get_temp'"),
        }
    };
    let hysteresis = inner.hysteresis;
    let heating_gain = inner.heating_gain;
    let check_gain_time = inner.check_gain_time;
    let max_error = inner.max_error;
    let heater_name = inner.heater_name.clone();
    let fault = {
        let mut s = inner.state.borrow_mut();
        if temp >= target - hysteresis || target <= 0. {
            // Temperature near target - reset checks.
            if s.approaching_target && target != 0. {
                log::info(&percent(
                    "Heater %s within range of %.3f",
                    &[PctValue::Str(heater_name.clone()), PctValue::Float(target)],
                ));
            }
            s.approaching_target = false;
            s.starting_approach = false;
            if temp <= target + hysteresis {
                s.error = 0.;
            }
            s.last_target = target;
            return eventtime + 1.;
        }
        s.error += (target - hysteresis) - temp;
        let mut fault = false;
        if !s.approaching_target {
            if target != s.last_target {
                // Target changed - reset checks.
                log::info(&percent(
                    "Heater %s approaching new target of %.3f",
                    &[PctValue::Str(heater_name.clone()), PctValue::Float(target)],
                ));
                s.approaching_target = true;
                s.starting_approach = true;
                s.goal_temp = temp + heating_gain;
                s.goal_systime = eventtime + check_gain_time;
            } else if s.error >= max_error {
                // Failure due to inability to maintain target temperature.
                fault = true;
            }
        } else if temp >= s.goal_temp {
            // Temperature approaching target - reset checks.
            s.starting_approach = false;
            s.error = 0.;
            s.goal_temp = temp + heating_gain;
            s.goal_systime = eventtime + check_gain_time;
        } else if eventtime >= s.goal_systime {
            // Temperature is no longer approaching target.
            s.approaching_target = false;
            log::info(&percent(
                "Heater %s no longer approaching target %.3f",
                &[PctValue::Str(heater_name.clone()), PctValue::Float(target)],
            ));
        } else if s.starting_approach {
            // `min(goal_temp, temp + heating_gain)` (`verify_heater.py:83`);
            // spelled with `<` so NaN behaves like Python's `min`.
            let new_goal = temp + heating_gain;
            if new_goal < s.goal_temp {
                s.goal_temp = new_goal;
            }
        }
        if !fault {
            s.last_target = target;
        }
        fault
    };
    if fault {
        return heater_fault(inner);
    }
    eventtime + 1.
}

fn heater_fault(inner: &Arc<Inner>) -> f64 {
    // `verify_heater.py:86-90`.
    let msg = percent(
        "Heater %s not heating at expected rate",
        &[PctValue::Str(inner.heater_name.clone())],
    );
    log::error(&msg);
    let shutdown_msg = {
        let mut m = msg;
        m.push_str(HINT_THERMAL);
        m
    };
    inner
        .printer
        .invoke_shutdown(&shutdown_msg, PyValue::Dict(indexmap::IndexMap::new()));
    NEVER
}

/// `load_config_prefix(config)` (`verify_heater.py:92-93`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = HeaterCheck::new(config)?;
    Ok(Some(Arc::new(obj) as ObjectRef))
}

/// Module factory for section loading.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
