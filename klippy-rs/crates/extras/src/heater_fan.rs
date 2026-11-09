//! Port of the `heater_fan` module (`klippy/extras/heater_fan.py`).
//!
//! A `[heater_fan NAME]` section drives a [`Fan`](crate::fan::Fan) whose
//! speed follows the heaters named by `heater`: a 1 s reactor timer calls
//! `heater.get_temp(eventtime)` for each resolved heater and turns the fan
//! to `fan_speed` while any heater has a non-zero target or is hotter than
//! `heater_temp`, and back to `0.` otherwise. `get_status` delegates to the
//! fan status.
//!
//! The `Fan` is constructed with `default_shutdown_speed=1.` so a shutdown
//! leaves the fan running (`fan.py` `Fan(config, default_shutdown_speed=1.)`).

use klippy_api::config::{Config, GetFloatOpts, GetListOpts};
use klippy_api::error::KResult;
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, EV_KLIPPY_READY};
use klippy_api::{lookup_interface, GCell};
use pycompat::PyValue;
use std::sync::Arc;

use crate::fan::Fan;
use crate::heaters::{Heater, HeaterApi, PrinterHeatersApi};
/// `PIN_MIN_TIME` (`heater_fan.py:8`).
pub const PIN_MIN_TIME: f64 = 0.100;

struct State {
    /// `self.heater_names` (`heater_fan.py:15`).
    heater_names: Vec<String>,
    /// `self.heater_temp` (`heater_fan.py:16`).
    heater_temp: f64,
    /// `self.fan_speed` (`heater_fan.py:19`).
    fan_speed: f64,
    /// `self.last_speed` (`heater_fan.py:20,34-35`).
    last_speed: f64,
    /// `self.heaters` (`heater_fan.py:17,23`).
    heaters: Vec<Arc<Heater>>,
}

struct Inner {
    printer: klippy_api::printer::Printer,
    fan: Arc<Fan>,
    state: GCell<State>,
}

/// `PrinterHeaterFan` (`heater_fan.py:10`).
pub struct PrinterHeaterFan {
    inner: Arc<Inner>,
}

impl PrinterHeaterFan {
    /// `PrinterHeaterFan.__init__(config)` (`heater_fan.py:11-20`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        printer.load_object(config, "heaters", LookupDefault::Raise)?;
        let heater_names = config
            .getlist(
                "heater",
                GetListOpts {
                    default: Some(Some(vec!["extruder".to_string()])),
                    ..GetListOpts::default()
                },
            )?
            .expect("getlist with a default never yields None");
        let heater_temp = config
            .getfloat(
                "heater_temp",
                GetFloatOpts {
                    default: Some(Some(50.0)),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(50.0);
        let fan = Fan::new(config, 1.0)?;
        let fan_speed = config
            .getfloat(
                "fan_speed",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    minval: Some(0.0),
                    maxval: Some(1.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(1.0);
        let inner = Arc::new(Inner {
            printer: printer.clone(),
            fan,
            state: GCell::new(State {
                heater_names,
                heater_temp,
                fan_speed,
                last_speed: 0.0,
                heaters: Vec::new(),
            }),
        });
        let ready_inner = Arc::clone(&inner);
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params| {
                handle_ready(&ready_inner)?;
                Ok(PyValue::None)
            }),
        );
        Ok(PrinterHeaterFan { inner })
    }

    /// `self.heater_names` (`heater_fan.py:15`).
    pub fn heater_names(&self) -> Vec<String> {
        self.inner.state.borrow().heater_names.clone()
    }

    /// `self.heater_temp` (`heater_fan.py:16`).
    pub fn heater_temp(&self) -> f64 {
        self.inner.state.borrow().heater_temp
    }

    /// `self.fan_speed` (`heater_fan.py:19`).
    pub fn fan_speed(&self) -> f64 {
        self.inner.state.borrow().fan_speed
    }

    /// `self.last_speed` (`heater_fan.py:20,34-35`).
    pub fn last_speed(&self) -> f64 {
        self.inner.state.borrow().last_speed
    }

    /// The wrapped fan (`self.fan`, `heater_fan.py:18`).
    pub fn fan(&self) -> Arc<Fan> {
        self.inner.fan.clone()
    }

    /// `self.handle_ready()` (`heater_fan.py:21-25`).
    pub fn handle_ready(&self) -> KResult<()> {
        handle_ready(&self.inner)
    }

    /// `self.get_status(eventtime)` (`heater_fan.py:26-27`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.inner.fan.get_status(eventtime)
    }

    /// `self.callback(eventtime)` (`heater_fan.py:28-37`); the timer body.
    pub fn callback(&self, eventtime: f64) -> f64 {
        callback(&self.inner, eventtime)
    }
}

fn handle_ready(inner: &Arc<Inner>) -> KResult<()> {
    // `heater_fan.py:21-25`.
    let pheaters =
        lookup_interface::<dyn PrinterHeatersApi>(&*inner.printer, "heaters", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
    let heater_names = inner.state.borrow().heater_names.clone();
    let mut heaters = Vec::with_capacity(heater_names.len());
    for name in &heater_names {
        heaters.push(pheaters.lookup_heater(name)?);
    }
    inner.state.borrow_mut().heaters = heaters;
    let reactor = inner.printer.get_reactor();
    let inner_cb = Arc::clone(inner);
    let _timer = reactor.register_timer(
        move |eventtime| callback(&inner_cb, eventtime),
        reactor.monotonic() + PIN_MIN_TIME,
    );
    Ok(())
}

fn callback(inner: &Arc<Inner>, eventtime: f64) -> f64 {
    // `heater_fan.py:28-37`. The GCell borrow is dropped before
    // `fan.set_speed` (which may pause the reactor), per the GCell rules.
    let (heaters, heater_temp, fan_speed, last_speed) = {
        let st = inner.state.borrow();
        (
            st.heaters.clone(),
            st.heater_temp,
            st.fan_speed,
            st.last_speed,
        )
    };
    let mut speed = 0.0;
    for heater in &heaters {
        let (current_temp, target_temp) = heater.get_temp(eventtime);
        if target_temp != 0.0 || current_temp > heater_temp {
            speed = fan_speed;
        }
    }
    if speed != last_speed {
        inner.state.borrow_mut().last_speed = speed;
        // `Fan::set_speed` currently cannot fail (`send_async_request`
        // always returns `Ok`), so there is no error path to mirror.
        let _ = inner.fan.set_speed(speed, None);
    }
    eventtime + 1.0
}

impl PrinterObject for PrinterHeaterFan {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(eventtime))
    }
}

/// `load_config_prefix(config)` (`heater_fan.py:39-40`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = PrinterHeaterFan::new(config)?;
    Ok(Some(Arc::new(obj) as ObjectRef))
}

/// The frozen `heater_fan` module factory.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
