//! Port of the `idle_timeout` module (`klippy/extras/idle_timeout.py`).
//!
//! Disables the printer (idle G-code, then motors off) after it has been
//! motionless for `idle_timeout` seconds. The lifecycle is a state machine
//! driven by a reactor timer: `Idle` (construct state) -> `Printing`
//! (on `toolhead:sync_print_time`) -> `Ready` (motion caught up) ->
//! `Idle` (timeout elapsed, idle G-code run). See the module reference for
//! the Python counterpart of every method.

use std::sync::Arc;

use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts as ConfigGetFloatOpts};
use klippy_api::gcode::{
    GCode, GetFloatOpts as GCodeGetFloatOpts, RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::printer::{
    EV_IDLE_TIMEOUT_IDLE, EV_IDLE_TIMEOUT_PRINTING, EV_IDLE_TIMEOUT_READY, EV_KLIPPY_READY,
    EV_TOOLHEAD_SYNC_PRINT_TIME, LookupDefault, ModuleFactory, Printer,
};
use klippy_api::toolhead::ToolHead;
use klippy_api::object::load_interface;
use klippy_api::{lookup_interface, EventArg, KlippyError, KResult, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER};

/// `DEFAULT_IDLE_GCODE` (`idle_timeout.py:8-13`): the `gcode` template
/// used when the `[idle_timeout]` section does not define `gcode`.
pub const DEFAULT_IDLE_GCODE: &str = "\n{% if 'heaters' in printer %}\n   TURN_OFF_HEATERS\n{% endif %}\nM84\n";

/// `PIN_MIN_TIME` (`idle_timeout.py:15`).
pub const PIN_MIN_TIME: f64 = 0.100;

/// `READY_TIMEOUT` (`idle_timeout.py:16`).
pub const READY_TIMEOUT: f64 = 0.500;

/// `cmd_SET_IDLE_TIMEOUT_help` (`idle_timeout.py:108`).
pub const CMD_SET_IDLE_TIMEOUT_HELP: &str = "Set the idle timeout in seconds";

/// `self.gcode = printer.lookup_object('gcode')` (`idle_timeout.py:22`),
/// via the `dyn GCodeApi` interface of the `gcode` object.
fn gcode_of(printer: &Printer) -> KResult<GCode> {
    Ok(lookup_interface::<dyn klippy_api::gcode::GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .expect("LookupDefault::Raise never yields None"))
}

/// `self.toolhead = printer.lookup_object('toolhead')`
/// (`idle_timeout.py:42`), via the `dyn ToolHeadApi` interface.
fn toolhead_of(printer: &Printer) -> KResult<ToolHead> {
    Ok(lookup_interface::<dyn klippy_api::toolhead::ToolHeadApi>(
        &**printer,
        "toolhead",
        LookupDefault::Raise,
    )?
    .expect("LookupDefault::Raise never yields None"))
}

/// Float value of `args[i]`, for event parameters that are floats
/// (`toolhead:sync_print_time` carries floats only).
fn event_f64(args: &[EventArg], i: usize) -> f64 {
    match args.get(i) {
        Some(EventArg::Value(PyValue::Float(f))) => *f,
        Some(EventArg::Value(PyValue::Int(n))) => {
            n.inner().to_string().parse::<f64>().unwrap_or(0.0)
        }
        _ => 0.0,
    }
}

/// Port of the `IdleTimeout` class (`klippy/extras/idle_timeout.py:18`).
pub struct IdleTimeout {
    printer: Printer,
    reactor: Reactor,
    gcode: GCode,
    idle_gcode: TemplateWrapper,
    /// `self.toolhead = None` until `handle_ready` (`idle_timeout.py:23,42`).
    toolhead: GCell<Option<ToolHead>>,
    /// `self.timeout_timer = None` until `handle_ready`
    /// (`idle_timeout.py:23,43`).
    timeout_timer: GCell<Option<Timer>>,
    /// `self.idle_timeout` (`idle_timeout.py:25,111`).
    idle_timeout: GCell<f64>,
    /// `self.state` (`idle_timeout.py:32`): `"Idle" | "Ready" | "Printing"`.
    state: GCell<String>,
    /// `self.last_print_start_systime` (`idle_timeout.py:33,103`).
    last_print_start_systime: GCell<f64>,
}

impl IdleTimeout {
    /// `IdleTimeout.__init__` (`idle_timeout.py:19-33`), then wires the
    /// `klippy:ready` handler (`idle_timeout.py:24`) and the
    /// `SET_IDLE_TIMEOUT` command (`idle_timeout.py:29-31`), in the same
    /// order as the Python.
    pub fn new(config: &Config) -> KResult<Arc<IdleTimeout>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let gcode = gcode_of(&printer)?;

        let idle_timeout = config
            .getfloat(
                "timeout",
                ConfigGetFloatOpts {
                    default: Some(Some(600.0)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(600.0);
        // `gcode_macro = self.printer.load_object(config, 'gcode_macro')`
        // and `gcode_macro.load_template(config, 'gcode', DEFAULT_IDLE_GCODE)`
        // (`idle_timeout.py:26-28`).
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let idle_gcode = gcode_macro.load_template(config, "gcode", Some(DEFAULT_IDLE_GCODE))?;

        let this = Arc::new(IdleTimeout {
            printer: printer.clone(),
            reactor,
            gcode,
            idle_gcode,
            toolhead: GCell::new(None),
            timeout_timer: GCell::new(None),
            idle_timeout: GCell::new(idle_timeout),
            state: GCell::new("Idle".to_string()),
            last_print_start_systime: GCell::new(0.0),
        });

        let this_clone = this.clone();
        printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params: &[EventArg]| {
                this_clone.handle_ready(this_clone.clone())?;
                Ok(PyValue::None)
            }),
        );

        let cmd_this = this.clone();
        this.gcode.register_command(
            "SET_IDLE_TIMEOUT",
            Some(Box::new(move |gcmd| cmd_this.cmd_set_idle_timeout(gcmd))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_SET_IDLE_TIMEOUT_HELP.to_string()),
            },
        )?;
        Ok(this)
    }

    /// Current `idle_timeout` value (`self.idle_timeout`).
    pub fn idle_timeout(&self) -> f64 {
        self.idle_timeout.get()
    }

    /// Current state (`self.state`).
    pub fn state(&self) -> String {
        self.state.cloned()
    }

    /// `self.last_print_start_systime`.
    pub fn last_print_start_systime(&self) -> f64 {
        self.last_print_start_systime.get()
    }

    /// `get_status(eventtime)` (`idle_timeout.py:34-40`).
    fn get_status_py(&self, eventtime: f64) -> PyValue {
        let printing_time = if self.state.borrow().as_str() == "Printing" {
            eventtime - self.last_print_start_systime.get()
        } else {
            0.0
        };
        let mut s: IndexMap<PyValue, PyValue> = IndexMap::new();
        s.insert(PyValue::Str("state".to_string()), PyValue::Str(self.state.cloned()));
        s.insert(PyValue::Str("printing_time".to_string()), PyValue::Float(printing_time));
        s.insert(PyValue::Str("idle_timeout".to_string()), PyValue::Float(self.idle_timeout.get()));
        PyValue::Dict(s)
    }

    /// `handle_ready` (`idle_timeout.py:41-45`): look up the toolhead,
    /// start the timeout timer, and subscribe to `toolhead:sync_print_time`.
    pub fn handle_ready(&self, this: Arc<IdleTimeout>) -> KResult<()> {
        let toolhead = toolhead_of(&self.printer)?;
        *self.toolhead.borrow_mut() = Some(toolhead);
        let obj_clone = this.clone();
        // A timer callback raising propagates out of the reactor run loop.
        let timer = self.reactor.register_timer(
            move |eventtime| {
                obj_clone
                    .timeout_handler(eventtime)
                    .unwrap_or_else(|e| panic!("{}: {}", e.class_name(), e.message()))
            },
            NEVER,
        );
        *self.timeout_timer.borrow_mut() = Some(timer);
        self.printer.register_event_handler(
            EV_TOOLHEAD_SYNC_PRINT_TIME,
            Box::new(move |params: &[EventArg]| {
                this.handle_sync_print_time(params)?;
                Ok(PyValue::None)
            }),
        );
        Ok(())
    }

    /// `transition_idle_state` (`idle_timeout.py:46-58`).
    pub fn transition_idle_state(&self, eventtime: f64) -> KResult<f64> {
        *self.state.borrow_mut() = "Printing".to_string();

        let result = self
            .idle_gcode
            .render(None)
            .and_then(|script| self.gcode.run_script(&script));
        if let Err(e) = result {
            klippy_api::log::exception(
                "idle timeout gcode execution",
                &format!("{}: {}", e.class_name(), e.message()),
            );
            *self.state.borrow_mut() = "Ready".to_string();
            return Ok(eventtime + 1.0);
        }
        let toolhead = match self.toolhead.borrow().clone() {
            Some(t) => t,
            None => {
                // Unreachable: the timer only exists after `handle_ready`
                // has stored the toolhead.
                *self.state.borrow_mut() = "Idle".to_string();
                return Ok(NEVER);
            }
        };
        let print_time = toolhead.get_last_move_time()?;
        *self.state.borrow_mut() = "Idle".to_string();
        self.printer
            .send_event(EV_IDLE_TIMEOUT_IDLE, &[EventArg::Value(PyValue::Float(print_time))])?;
        Ok(NEVER)
    }

    /// `check_idle_timeout` (`idle_timeout.py:59-74`).
    pub fn check_idle_timeout(&self, eventtime: f64) -> KResult<f64> {
        let toolhead = match self.toolhead.borrow().clone() {
            Some(t) => t,
            None => {
                // Unreachable: see `transition_idle_state`.
                return Ok(NEVER);
            }
        };
        let (print_time, est_print_time, lookahead_empty) = toolhead.check_busy(eventtime);
        let idle_time = est_print_time - print_time;
        let idle_timeout = self.idle_timeout.get();
        if !lookahead_empty || idle_time < 1.0 {
            // Toolhead is busy
            return Ok(eventtime + idle_timeout);
        }
        if idle_time < idle_timeout {
            // Wait for idle timeout
            return Ok(eventtime + idle_timeout - idle_time);
        }
        if self.gcode.get_mutex().test() {
            // Gcode class busy
            return Ok(eventtime + 1.0);
        }
        // Idle timeout has elapsed
        self.transition_idle_state(eventtime)
    }

    /// `timeout_handler` (`idle_timeout.py:75-97`).
    pub fn timeout_handler(&self, eventtime: f64) -> KResult<f64> {
        if self.printer.is_shutdown() {
            return Ok(NEVER);
        }
        if self.state.borrow().as_str() == "Ready" {
            return self.check_idle_timeout(eventtime);
        }
        // Check if need to transition to "ready" state
        let toolhead = match self.toolhead.borrow().clone() {
            Some(t) => t,
            None => {
                // Unreachable: see `transition_idle_state`.
                return Ok(NEVER);
            }
        };
        let (print_time, est_print_time, lookahead_empty) = toolhead.check_busy(eventtime);
        let buffer_time = (2.0_f64).min(print_time - est_print_time);
        if !lookahead_empty {
            // Toolhead is busy
            return Ok(eventtime + READY_TIMEOUT + buffer_time.max(0.0));
        }
        if buffer_time > -READY_TIMEOUT {
            // Wait for ready timeout
            return Ok(eventtime + READY_TIMEOUT + buffer_time);
        }
        if self.gcode.get_mutex().test() {
            // Gcode class busy
            return Ok(eventtime + READY_TIMEOUT);
        }
        // Transition to "ready" state
        *self.state.borrow_mut() = "Ready".to_string();
        let t = est_print_time + PIN_MIN_TIME;
        self.printer.send_event(EV_IDLE_TIMEOUT_READY, &[EventArg::Value(PyValue::Float(t))])?;
        Ok(eventtime + self.idle_timeout.get())
    }

    /// `handle_sync_print_time` (`idle_timeout.py:98-107`).
    ///
    /// The event sender (`toolhead.py:267`) passes
    /// `(curtime, est_print_time, print_time)` while the handler declares
    /// `(curtime, print_time, est_print_time)`; parameters are bound
    /// positionally exactly as in Python (and used under the handler's
    /// names) — the misordering is part of the behavior.
    pub fn handle_sync_print_time(&self, params: &[EventArg]) -> KResult<()> {
        let curtime = event_f64(params, 0);
        let print_time = event_f64(params, 1);
        let est_print_time = event_f64(params, 2);
        if self.state.borrow().as_str() == "Printing" {
            return Ok(());
        }
        // Transition to "printing" state
        *self.state.borrow_mut() = "Printing".to_string();
        self.last_print_start_systime.set(curtime);
        let check_time = READY_TIMEOUT + print_time - est_print_time;
        if let Some(timer) = self.timeout_timer.borrow().as_ref() {
            self.reactor.update_timer(timer, curtime + check_time);
        }
        self.printer
            .send_event(EV_IDLE_TIMEOUT_PRINTING, &[EventArg::Value(PyValue::Float(est_print_time + PIN_MIN_TIME))])?;
        Ok(())
    }

    /// `cmd_SET_IDLE_TIMEOUT` (`idle_timeout.py:109-115`).
    pub fn cmd_set_idle_timeout(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let current = self.idle_timeout.get();
        let timeout = gcmd
            .get_float(
                "TIMEOUT",
                GCodeGetFloatOpts {
                    default: Some(Some(current)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    ..Default::default()
                },
            )?
            .unwrap_or(current);
        self.idle_timeout.set(timeout);
        let msg = percent_format("idle_timeout: Timeout set to %.2f s", &[PctValue::Float(timeout)])
            .map_err(|e| KlippyError::Internal {
                class_name: e.kind.to_string(),
                message: e.msg.clone(),
            })?;
        gcmd.respond_info(&msg, true);
        if self.state.borrow().as_str() == "Ready" {
            let checktime = self.reactor.monotonic() + timeout;
            let cell = self.timeout_timer.borrow();
            if let Some(timer) = cell.as_ref() {
                self.reactor.update_timer(timer, checktime);
            }
        }
        Ok(())
    }
}

impl PrinterObject for IdleTimeout {
    klippy_api::printer_object_base!();

    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(self.get_status_py(eventtime))
    }
}

/// `load_config(config)` (`idle_timeout.py:117-118`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let this = IdleTimeout::new(config)?;
    Ok(Some(this as ObjectRef))
}

/// Module factory entry points (`module_factory` convention).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
