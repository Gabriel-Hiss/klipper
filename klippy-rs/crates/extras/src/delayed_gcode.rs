//! Port of the `delayed_gcode` module.
//!
//! Source: `klippy/extras/delayed_gcode.py` (all of it).
//!
//! A `delayed_gcode` section names a Jinja `gcode` template and an
//! `initial_duration`. The template runs once the reactor timer fires
//! after `initial_duration` from `klippy:ready`, and `UPDATE_DELAYED_GCODE`
//! re-arms the timer (or, when called from inside the running script,
//! arranges a repeat).

use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GetFloatOpts as GGetFloatOpts, GCode, GCodeApi, GCmd};
use klippy_api::log;
use klippy_api::object::{load_interface, lookup_interface, ObjectRef};
use klippy_api::printer::{EV_KLIPPY_READY, LookupDefault, ModuleFactory, Printer};
use klippy_api::{printer_object, GCell};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER};

use std::sync::Arc;

use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};

/// Help text registered for the `UPDATE_DELAYED_GCODE` command
/// (`delayed_gcode.py:42`).
pub const CMD_HELP: &str = "Update the duration of a delayed_gcode";

/// Help text alias matching the Python class attribute name
/// (`cmd_UPDATE_DELAYED_GCODE_help`, `delayed_gcode.py:42`).
pub const CMD_UPDATE_DELAYED_GCODE_HELP: &str = CMD_HELP;

struct State {
    duration: f64,
    inside_timer: bool,
    repeat: bool,
    timer_handler: Option<Timer>,
    // Mirror of `timer_handler.waketime` (`reactor.py:227-232,269-271`):
    // the value passed to `register_timer` / `update_timer`, then the
    // timer event's return value. Exposed for tests.
    timer_waketime: f64,
}

struct Inner {
    name: String,
    reactor: Reactor,
    gcode: GCode,
    template: TemplateWrapper,
    state: GCell<State>,
}

/// The `DelayedGcode` printer object (`delayed_gcode.py:9`).
///
/// The Python object is stored directly in the printer registry; the
/// reactor timer, the `klippy:ready` handler and the mux command handler
/// all need an owned handle to the same state, so state lives in an
/// `Arc<Inner>` (all shared state is in the inner `GCell`, as the
/// reactor runs one greenlet at a time).
pub struct DelayedGcode {
    inner: Arc<Inner>,
}

printer_object!(DelayedGcode);

impl DelayedGcode {
    /// `DelayedGcode(config)` (`delayed_gcode.py:10-24`).
    pub fn new(config: &Config) -> KResult<DelayedGcode> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let name = match config.get_name().split_whitespace().nth(1) {
            Some(n) => n.to_string(),
            // `config.get_name().split()[1]` on a single-token name
            // raises `IndexError("list index out of range")` in
            // Python (`delayed_gcode.py:13`).
            None => {
                return Err(KlippyError::Internal {
                    class_name: "IndexError".to_string(),
                    message: "list index out of range".to_string(),
                })
            }
        };
        let gcode = lookup_gcode(&printer)?;

        // `gcode_macro = self.printer.load_object(config, 'gcode_macro')`
        // and `gcode_macro.load_template(config, 'gcode')`
        // (`delayed_gcode.py:15-16`).
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let template = gcode_macro.load_template(config, "gcode", None)?;
        let duration = config
            .getfloat(
                "initial_duration",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);

        let inner = Arc::new(Inner {
            name,
            reactor,
            gcode,
            template,
            state: GCell::new(State {
                duration,
                inside_timer: false,
                repeat: false,
                timer_handler: None,
                timer_waketime: NEVER,
            }),
        });
        let obj = DelayedGcode { inner: Arc::clone(&inner) };

        // `printer.register_event_handler("klippy:ready", self._handle_ready)`
        // (`delayed_gcode.py:20`). The Python handler takes no
        // parameters; the event carries none either.
        let inner_ev = Arc::clone(&inner);
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params| {
                handle_ready(&inner_ev);
                Ok(PyValue::None)
            }),
        );

        // `self.gcode.register_mux_command("UPDATE_DELAYED_GCODE", "ID",
        // self.name, self.cmd_UPDATE_DELAYED_GCODE, desc=T)` (`delayed_gcode.py:21-24`).
        let inner_cmd = Arc::clone(&inner);
        let mux_name = inner.name.clone();
        inner.gcode.register_mux_command(
            "UPDATE_DELAYED_GCODE",
            "ID",
            Some(&mux_name),
            Box::new(move |gcmd: &GCmd| cmd_update_delayed_gcode(&inner_cmd, gcmd)),
            Some(CMD_HELP),
        )?;

        Ok(obj)
    }

    /// Name of the delayed_gcode (`self.name`, `delayed_gcode.py:13`).
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// Current duration (`self.duration`, `delayed_gcode.py:17,44`).
    pub fn duration(&self) -> f64 {
        self.inner.state.borrow().duration
    }

    /// `self._handle_ready()` (`delayed_gcode.py:25-30`); also the
    /// `klippy:ready` event handler body.
    pub fn handle_ready(&self) {
        handle_ready(&self.inner);
    }

    /// `self._gcode_timer_event(eventtime)` (`delayed_gcode.py:31-41`).
    pub fn gcode_timer_event(&self, eventtime: f64) -> f64 {
        gcode_timer_event(&self.inner, eventtime)
    }

    /// `self.cmd_UPDATE_DELAYED_GCODE(gcmd)` (`delayed_gcode.py:43-51`).
    pub fn cmd_update_delayed_gcode(&self, gcmd: &GCmd) -> KResult<()> {
        cmd_update_delayed_gcode(&self.inner, gcmd)
    }

    /// `self.inside_timer` (`delayed_gcode.py:19,32,40`).
    pub fn inside_timer(&self) -> bool {
        self.inner.state.borrow().inside_timer
    }

    /// `self.repeat` (`delayed_gcode.py:19,38,40,46`).
    pub fn repeat(&self) -> bool {
        self.inner.state.borrow().repeat
    }

    /// `timer_handler.waketime` mirror (`reactor.py:227,271`).
    pub fn timer_waketime(&self) -> f64 {
        self.inner.state.borrow().timer_waketime
    }
}

fn lookup_gcode(printer: &Printer) -> KResult<GCode> {
    // `printer.lookup_object('gcode')` (`delayed_gcode.py:14`).
    let iface = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?;
    Ok(iface.expect("Raise never yields None"))
}

fn handle_ready(inner: &Arc<Inner>) {
    // `delayed_gcode.py:25-30`.
    let duration = inner.state.borrow().duration;
    let waketime = if duration != 0.0 {
        inner.reactor.monotonic() + duration
    } else {
        NEVER
    };
    let inner_cb = Arc::clone(inner);
    let timer = inner
        .reactor
        .register_timer(move |eventtime| gcode_timer_event(&inner_cb, eventtime), waketime);
    let mut s = inner.state.borrow_mut();
    s.timer_handler = Some(timer);
    s.timer_waketime = waketime;
}

fn gcode_timer_event(inner: &Arc<Inner>, eventtime: f64) -> f64 {
    // `delayed_gcode.py:31-41`.
    inner.state.borrow_mut().inside_timer = true;
    let outcome = inner
        .template
        .render(None)
        .and_then(|script| inner.gcode.run_script(&script));
    if let Err(e) = outcome {
        log::exception("Script running error", &e.message().to_string());
    }
    let (repeat, duration) = {
        let s = inner.state.borrow();
        (s.repeat, s.duration)
    };
    let nextwake = if repeat {
        eventtime + duration
    } else {
        NEVER
    };
    {
        let mut s = inner.state.borrow_mut();
        s.inside_timer = false;
        s.repeat = false;
        if s.timer_handler.is_some() {
            s.timer_waketime = nextwake;
        }
    }
    nextwake
}

fn cmd_update_delayed_gcode(inner: &Arc<Inner>, gcmd: &GCmd) -> KResult<()> {
    // `delayed_gcode.py:43-51`.
    let duration = gcmd
        .get_float(
            "DURATION",
            GGetFloatOpts {
                default: None,
                minval: Some(0.0),
                maxval: None,
                above: None,
                below: None,
                ..Default::default()
            },
        )?
        .expect("required DURATION parameter");
    inner.state.borrow_mut().duration = duration;
    let inside = inner.state.borrow().inside_timer;
    if inside {
        inner.state.borrow_mut().repeat = duration != 0.0;
    } else {
        let waketime = if duration != 0.0 {
            inner.reactor.monotonic() + duration
        } else {
            NEVER
        };
        let timer = {
            let mut s = inner.state.borrow_mut();
            s.timer_handler.take()
        };
        let timer = match timer {
            Some(t) => t,
            // Before `klippy:ready` the handler is still `None` and
            // Python's `reactor.update_timer(None, waketime)` raises
            // `AttributeError: 'NoneType' object has no attribute
            // 'timer_is_running'`.
            None => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: "'NoneType' object has no attribute 'timer_is_running'"
                        .to_string(),
                })
            }
        };
        inner.reactor.update_timer(&timer, waketime);
        let mut s = inner.state.borrow_mut();
        s.timer_handler = Some(timer);
        s.timer_waketime = waketime;
    }
    Ok(())
}

/// `load_config_prefix(config)` (`delayed_gcode.py:53-54`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = DelayedGcode::new(config)?;
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
