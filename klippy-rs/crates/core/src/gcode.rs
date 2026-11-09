//! Port of the `gcode` module (`klippy/gcode.py`).
//!
//! Covers [`GCodeDispatch`] (all of `klippy/gcode.py:99-389`),
//! [`GCodeIO`] (`klippy/gcode.py:390-487`), the [`GCodeCommand`]
//! handle handed to every handler (`klippy/gcode.py:24-96`), and
//! [`add_early_printer_objects`] / [`module_factory`] (`klippy/gcode.py:488-490`).
//! The line grammar and the handler-table rules live in
//! [`crate::gcode_parse`]; this module owns the live printer,
//! reactor and response collaborators.
//!
//! Mapping notes, where the frozen `klippy_api` contract forces a shape
//! different from the Python:
//! - `GCodeDispatch` keeps its mutable state in a shared [`DispatchInner`]
//!   (`Arc`-shared) so event callbacks and the [`GCodeCommand`]s it hands
//!   out can reach the same output list without a self-referential borrow.
//! - Handler tables store `Option<GCodeCommandHandler>` slots. The slot is
//!   taken out while the handler runs (so an `FnMut` handler never borrows
//!   the table it may re-register into) and put back afterwards. The one
//!   deviation: `register_command(..., None)` called from inside the very
//!   handler that is running returns `None` instead of the running handler;
//!   the final table state still matches Python.
//! - `register_mux_command` stores a placeholder handler for the mux
//!   command and dispatches through the mux table at call time. The Python
//!   mux dispatcher closure is recreated by this indirection; observable
//!   behaviour (extended-parameter parsing, dispatch, errors) is unchanged.
//! - The Python `dict` repr of the handlers in the mux duplicate message
//!   shows `<function anonymous>` because the Rust handler registration has
//!   no function name to render.
//! - `GCodeIO` skips fd registration when `gcode_fd` is missing from
//!   `start_args`; the real startup (`klippy/klippy.py:356-367`) always
//!   sets it.

use std::collections::VecDeque;
use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts, GetIntOpts, GetOpts,
    GetParser, OutputCallback, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_GCODE_COMMAND_ERROR,
    EV_GCODE_REQUEST_RESTART, EV_KLIPPY_ANALYZE_SHUTDOWN, EV_KLIPPY_DISCONNECT,
    EV_KLIPPY_READY, EV_KLIPPY_SHUTDOWN,
};
use klippy_api::toolhead::ToolHeadApi;
use pycompat::text::{float_from_string, int_from_string, percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use pycompat::value::py_rich_compare;
use pytext::pyre;
use reactor::{FdHandler, Reactor};

use crate::gcode_parse::{self, CommandError as ParseCommandError};

/// Canonical `Coord` (`klippy/gcode.py:12`); one Python class shared by
/// `gcode`, `gcode_move` and `toolhead`,/canonicalised in `klippy_api`.
pub type Coord = klippy_api::toolhead::Coord;

/// `klippy/gcode.py` `CommandError`; in Rust `KlippyError::Command` carries
/// it. The parse layer keeps its own [`ParseCommandError`] type.
pub use crate::gcode_parse::CommandError;

/// The `m112_r` pattern (`klippy/gcode.py:432`).
const M112_PATTERN: &str = "^(?:[nN][0-9]+)?\\s*[mM]112(?:\\s|$)";

// ---------------------------------------------------------------------------
// Small bridges to `pycompat` / the printer service.
// ---------------------------------------------------------------------------

fn pct(fmt: &str, args: &[PctValue]) -> String {
    match percent_format(fmt, args) {
        Ok(s) => s,
        Err(e) => panic!("constant gcode format {fmt:?} failed: {}", e.msg),
    }
}

fn pct_str(v: &str) -> PctValue {
    PctValue::Str(v.to_string())
}

fn pct_float(v: f64) -> PctValue {
    PctValue::Float(v)
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

/// The `request_restart('exit')` log line (`klippy/gcode.py:357`).
pub fn format_exiting(print_time: f64) -> String {
    pct("Exiting (print time %.3fs)", &[pct_float(print_time)])
}

/// The `analyze_shutdown` dump (`klippy/gcode.py:420-425`): one
/// `Read %f: <repr>` entry per buffered input block, headed by the block
/// count. `entries` mirrors `GCodeIO.input_log`.
pub fn format_input_dump(entries: &[(f64, String)]) -> String {
    let mut out: Vec<String> =
        vec![pct("Dumping gcode input %d blocks", &[pct_int(entries.len() as i64)])];
    for (eventtime, data) in entries {
        out.push(pct(
            "Read %f: %s",
            &[pct_float(*eventtime), pct_str(&pycompat::text::str_repr(data))],
        ));
    }
    out.join("\n")
}

fn command_error(message: String) -> KlippyError {
    KlippyError::Command(message)
}

fn map_parse_command_error(e: ParseCommandError) -> KlippyError {
    KlippyError::Command(e.0)
}

fn config_error(message: String) -> KlippyError {
    KlippyError::Config(message)
}

/// `printer.get_start_args().get(key)`; `None` when absent or not a dict
/// (the Python code passes a dict; a non-dict raises `AttributeError`,
/// which a real startup never produces).
fn start_arg(args: &PyValue, key: &str) -> Option<PyValue> {
    match args {
        PyValue::Dict(m) => m.get(&PyValue::Str(key.to_string())).cloned(),
        _ => None,
    }
}

fn start_arg_truthy(args: &PyValue, key: &str) -> bool {
    match start_arg(args, key) {
        Some(v) => v.truthy(),
        None => false,
    }
}

fn start_arg_i32(args: &PyValue, key: &str) -> Option<i32> {
    match start_arg(args, key) {
        Some(PyValue::Int(n)) => n.0.to_string().parse::<i64>().ok().map(|v| {
            if v > i32::MAX as i64 {
                i32::MAX
            } else if v < i32::MIN as i64 {
                i32::MIN
            } else {
                v as i32
            }
        }),
        _ => None,
    }
}

/// `printer.get_start_args().get("software_version")` rendered with `%s`
/// (`None` for a missing key, exactly as Python formats `None`).
fn start_arg_as_str(args: &PyValue, key: &str) -> String {
    match start_arg(args, key) {
        Some(v) => v.str(),
        None => "None".to_string(),
    }
}

/// Extended-command name check (`klippy/gcode.py:145-149`). `gcode_parse`
/// keeps the identical helper private, so it is restated here for the live
/// registration path.
fn validate_extended_name(cmd: &str) -> Result<(), KlippyError> {
    let upper = cmd.chars().flat_map(|c| c.to_uppercase()).collect::<String>();
    let sanitized: String = cmd.chars().map(|c| if c == '_' { 'A' } else { c }).collect();
    let alnum = !sanitized.is_empty() && sanitized.chars().all(|c| c.is_alphanumeric());
    let first_bad = matches!(cmd.chars().next(), Some(c) if c.is_ascii_digit());
    let second_bad = matches!(cmd.chars().nth(1), Some(c) if c.is_ascii_digit());
    if upper != cmd || !alnum || first_bad || second_bad {
        return Err(config_error(format!(
            "Can't register '{cmd}' as it is an invalid name"
        )));
    }
    Ok(())
}

/// Python `dict` repr of the mux values map in the duplicate error
/// (`klippy/gcode.py:170-172`). Handler names are unavailable in the Rust
/// registration, so each renders as `<function anonymous>`.
fn mux_dict_repr(values: &IndexMap<Option<String>, Option<GCodeCommandHandler>>) -> String {
    let parts: Vec<String> = values
        .keys()
        .map(|k| match k {
            Some(s) => format!("'{s}': <function anonymous>"),
            None => "None: <function anonymous>".to_string(),
        })
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn m112_matches(line: &str) -> bool {
    pyre::compile(M112_PATTERN)
        .ok()
        .and_then(|re| re.match_str(line))
        .is_some()
}

// ---------------------------------------------------------------------------
// Output sharing between the dispatcher and every `GCodeCommand` it makes.
// ---------------------------------------------------------------------------

struct Outputs {
    callbacks: GCell<Vec<OutputCallback>>,
}

impl Outputs {
    fn new() -> Outputs {
        Outputs { callbacks: GCell::new(Vec::new()) }
    }

    fn respond_raw(&self, msg: &str) {
        let mut cbs = self.callbacks.borrow_mut();
        for cb in cbs.iter_mut() {
            cb(msg.to_string());
        }
    }

    /// `GCodeDispatch.respond_info` (`klippy/gcode.py:250-254`).
    fn respond_info(&self, msg: &str, log_enabled: bool) {
        if log_enabled {
            log::info(msg);
        }
        let lines: Vec<&str> = msg.trim().split('\n').map(|l| l.trim()).collect();
        self.respond_raw(&format!("// {}", lines.join("\n// ")));
    }
}

// ---------------------------------------------------------------------------
// `GCodeDispatch` state.
// ---------------------------------------------------------------------------

struct MuxState {
    key: String,
    values: IndexMap<Option<String>, Option<GCodeCommandHandler>>,
}

struct DispatchInner {
    printer: Printer,
    is_fileinput: bool,
    is_printer_ready: GCell<bool>,
    reactor: Reactor,
    /// `self.mutex = printer.get_reactor().mutex()` (`klippy/gcode.py:111`).
    mutex: reactor::ReactorMutex,
    outputs: Outputs,
    ready: GCell<IndexMap<String, Option<GCodeCommandHandler>>>,
    /// `when_not_ready` registrations, in registration order. The Python
    /// `base_gcode_handlers` dict holds the same handler objects; this list
    /// holds the same membership and order with one storage location.
    base_order: GCell<Vec<String>>,
    help: GCell<IndexMap<String, String>>,
    mux: GCell<IndexMap<String, MuxState>>,
}

impl DispatchInner {
    fn respond_raw(&self, msg: &str) {
        self.outputs.respond_raw(msg);
    }

    fn respond_info(&self, msg: &str, log_enabled: bool) {
        self.outputs.respond_info(msg, log_enabled);
    }

    /// `_respond_state` (`klippy/gcode.py:263-264`).
    fn respond_state(&self, state: &str) {
        let msg = pct("Klipper state: %s", &[pct_str(state)]);
        self.respond_info(&msg, false);
    }

    /// `_respond_error` (`klippy/gcode.py:255-262`).
    fn respond_error(&self, msg: &str) {
        log::warning(msg);
        let lines: Vec<&str> = msg.trim().split('\n').collect();
        if lines.len() > 1 {
            self.respond_info(&lines.join("\n"), false);
        }
        let line0 = lines.first().map(|s| s.trim()).unwrap_or("");
        self.respond_raw(&pct("!! %s", &[pct_str(line0)]));
        if self.is_fileinput {
            self.printer.request_exit("error_exit");
        }
    }

    fn state_message(&self) -> String {
        self.printer.get_state_message().0
    }

    /// `request_restart` (`klippy/gcode.py:352-361`).
    fn do_request_restart(&self, result: &str) -> KResult<()> {
        if self.is_printer_ready.get() {
            let toolhead = match klippy_api::lookup_interface::<dyn ToolHeadApi>(
                &*self.printer,
                "toolhead",
                LookupDefault::Raise,
            )? {
                Some(t) => t,
                None => {
                    return Err(config_error(
                        "Unknown config object 'toolhead'".to_string(),
                    ))
                }
            };
            let print_time = toolhead.get_last_move_time()?;
            if result == "exit" {
                log::info(&format_exiting(print_time));
            }
            self.printer.send_event(
                EV_GCODE_REQUEST_RESTART,
                &[EventArg::Value(PyValue::Float(print_time))],
            )?;
            toolhead.dwell(0.500)?;
            toolhead.wait_moves()?;
        }
        self.printer.request_exit(result);
        Ok(())
    }

    fn on_ready(&self) {
        self.is_printer_ready.set(true);
        self.respond_state("Ready");
    }

    fn on_shutdown(&self) {
        if !self.is_printer_ready.get() {
            return;
        }
        self.is_printer_ready.set(false);
        self.respond_state("Shutdown");
    }

    fn on_disconnect(&self) {
        self.respond_state("Disconnect");
    }

    fn is_active(&self, cmd: &str) -> bool {
        if self.is_printer_ready.get() {
            self.ready.borrow().contains_key(cmd)
        } else {
            self.base_order.borrow().iter().any(|c| c == cmd)
                && self.ready.borrow().contains_key(cmd)
        }
    }

    fn has_handler_active(&self, cmd: &str) -> bool {
        // The mux placeholder is registered with `when_not_ready=False`, so
        // it only exists in the ready table (`klippy/gcode.py:162`); when
        // the printer is not ready, mux commands fall through to
        // `cmd_default` exactly like any other non-base command.
        self.is_active(cmd)
    }
    fn active_commands(&self) -> Vec<String> {
        if self.is_printer_ready.get() {
            self.ready.borrow().keys().cloned().collect()
        } else {
            self.base_order.borrow().clone()
        }
    }

    /// Shared insert for `register_command` and the placeholder the mux
    /// registration needs. Returns the removed handler for `func=None`.
    fn insert_command(
        &self,
        cmd: &str,
        handler: Option<GCodeCommandHandler>,
        when_not_ready: bool,
        desc: Option<&str>,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut ready = self.ready.borrow_mut();
        match handler {
            None => {
                let old = ready.shift_remove(cmd);
                drop(ready);
                let mut base = self.base_order.borrow_mut();
                if let Some(pos) = base.iter().position(|c| c == cmd) {
                    base.remove(pos);
                }
                // Help entries persist after unregister, as Python keeps
                // `gcode_help` untouched.
                Ok(old.flatten())
            }
            Some(func) => {
                if ready.contains_key(cmd) {
                    return Err(config_error(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                if !gcode_parse::is_traditional_gcode(cmd) {
                    validate_extended_name(cmd)?;
                }
                ready.insert(cmd.to_string(), Some(func));
                drop(ready);
                if when_not_ready {
                    self.base_order.borrow_mut().push(cmd.to_string());
                }
                if let Some(d) = desc {
                    self.help.borrow_mut().insert(cmd.to_string(), d.to_string());
                }
                Ok(None)
            }
        }
    }

    /// Take the active handler for `cmd`; the slot stays in place holding
    /// `None` so insertion order is preserved while it runs.
    fn take_active(&self, cmd: &str) -> Option<GCodeCommandHandler> {
        let mut ready = self.ready.borrow_mut();
        match ready.get_mut(cmd) {
            Some(slot) => slot.take(),
            None => None,
        }
    }

    /// Restore a handler taken by [`DispatchInner::take_active`], unless the
    /// slot was unregistered (or replaced) while it ran.
    fn restore_active(&self, cmd: &str, handler: GCodeCommandHandler) {
        let mut ready = self.ready.borrow_mut();
        if let Some(slot) = ready.get_mut(cmd) {
            if slot.is_none() {
                let _ = slot.insert(handler);
            }
        }
    }


    /// `_get_extended_params` on the concrete command; the error is the
    /// same `Malformed command` text the Python raises.
    fn apply_extended(concrete: &GCodeCommand) -> Result<(), KlippyError> {
        let mut inner = concrete.inner.borrow_mut();
        match inner.apply_extended_params() {
            Ok(()) => Ok(()),
            Err(e) => Err(command_error(e.0)),
        }
    }

    /// Invoke the active handler (or mux dispatcher) for `cmd`, with
    /// extended-parameter preparation exactly as the Python wrapper does.
    /// `handler_exists` must have been checked by the caller when the
    /// caller needs to distinguish `cmd_default`.
    fn call_active(&self, cmd: &str, concrete: &Arc<GCodeCommand>) -> KResult<()> {
        let is_mux = self.mux.borrow().contains_key(cmd);
        if !gcode_parse::is_traditional_gcode(cmd) {
            Self::apply_extended(concrete)?;
        }
        if is_mux {
            return self.call_mux(cmd, concrete);
        }
        let mut handler = match self.take_active(cmd) {
            Some(h) => h,
            None => return Ok(()),
        };
        let gcmd: GCmd = Arc::clone(concrete) as GCmd;
        let result = handler(&gcmd);
        self.restore_active(cmd, handler);
        result
    }

    fn call_mux(&self, cmd: &str, concrete: &Arc<GCodeCommand>) -> KResult<()> {
        // Clone the key and whether a `None` arm exists, then drop the
        // borrow before parameter access.
        let (key, has_none) = {
            let mux = self.mux.borrow();
            match mux.get(cmd) {
                Some(m) => (m.key.clone(), m.values.contains_key(&None)),
                None => return Ok(()),
            }
        };
        let key_param: Option<String> = if has_none {
            concrete
                .inner
                .borrow()
                .get_str_opt(&key, Some(None))
                .map_err(map_parse_command_error)?
        } else {
            Some(
                concrete
                    .inner
                    .borrow()
                    .get_str(&key, None)
                    .map_err(map_parse_command_error)?,
            )
        };
        // Validate membership and clone the value keys for the error text.
        let value_keys: Vec<Option<String>> = {
            let mux = self.mux.borrow();
            match mux.get(cmd) {
                Some(m) => m.values.keys().cloned().collect(),
                None => Vec::new(),
            }
        };
        if !value_keys.iter().any(|v| v == &key_param) {
            let shown = key_param.as_deref().unwrap_or("None");
            let mut keys: Vec<String> = Vec::new();
            let mut guess = String::new();
            for v in &value_keys {
                let Some(vs) = v else { continue };
                keys.push(format!("'{vs}'"));
                if let Some(kp) = &key_param {
                    if !kp.is_empty() && vs.contains(kp.as_str()) {
                        guess = format!(". Did you mean '{vs}'?");
                    }
                }
            }
            if guess.is_empty() {
                guess = format!(". Options: {}", keys.join(", "));
            }
            return Err(command_error(format!(
                "The value '{shown}' is not valid for {key}{guess}"
            )));
        }
        let mut handler = {
            let mut mux = self.mux.borrow_mut();
            match mux.get_mut(cmd) {
                Some(m) => match m.values.get_mut(&key_param) {
                    Some(slot) => match slot.take() {
                        Some(h) => h,
                        None => return Ok(()),
                    },
                    None => return Ok(()),
                },
                None => return Ok(()),
            }
        };
        let gcmd: GCmd = Arc::clone(concrete) as GCmd;
        let result = handler(&gcmd);
        // Restore unless the slot was removed while running.
        let mut mux = self.mux.borrow_mut();
        if let Some(m) = mux.get_mut(cmd) {
            if let Some(slot) = m.values.get_mut(&key_param) {
                if slot.is_none() {
                    let _ = slot.insert(handler);
                }
            }
        }
        result
    }

    /// `cmd_default` (`klippy/gcode.py:283-316`).
    fn cmd_default(&self, concrete: &Arc<GCodeCommand>) -> KResult<()> {
        let cmd = concrete.inner.borrow().get_command().to_string();
        if cmd == "M105" {
            concrete.ack_inner(Some("T:0"));
            return Ok(());
        }
        if cmd == "M21" {
            return Ok(());
        }
        if !self.is_printer_ready.get() {
            return Err(command_error(self.state_message()));
        }
        if cmd.is_empty() {
            let commandline = concrete.inner.borrow().get_commandline().to_string();
            if !commandline.is_empty() {
                log::debug(&commandline);
            }
            return Ok(());
        }
        if cmd.contains(' ') {
            let real = cmd.split_whitespace().next().unwrap_or("").to_string();
            if ["M117", "M118", "M23"].contains(&real.as_str()) && self.has_handler_active(&real)
            {
                concrete.inner.borrow_mut().set_command(real.clone());
                return self.call_active(&real, concrete);
            }
            // Falls through to the unknown-command response.
        } else if cmd == "M140" || cmd == "M104" {
            let s = concrete
                .inner
                .borrow()
                .get_float("S", Some(0.0), None, None, None, None)
                .map_err(map_parse_command_error)?;
            if s == 0.0 {
                return Ok(());
            }
        } else if cmd == "M107" {
            return Ok(());
        } else if cmd == "M106" {
            let s = concrete
                .inner
                .borrow()
                .get_float("S", Some(1.0), None, None, None, None)
                .map_err(map_parse_command_error)?;
            if s == 0.0 || self.is_fileinput {
                return Ok(());
            }
        }
        let msg = format!("Unknown command:\"{cmd}\"");
        concrete.respond_info(&msg, true);
        Ok(())
    }

    /// One line of `_process_commands` (handler invocation and the error
    /// handling around it). The returned `Err` leaves the per-command ack
    /// decision to the caller, mirroring Python's `raise` before `ack`.
    fn invoke_handler(
        &self,
        concrete: &Arc<GCodeCommand>,
        cmd: &str,
    ) -> KResult<()> {
        if self.has_handler_active(cmd) {
            self.call_active(cmd, concrete)
        } else {
            self.cmd_default(concrete)
        }
    }
}


/// Build the `get_status` payload (`klippy/gcode.py:176-183`), derived on
/// demand from the active tables (the Python rebuilds it eagerly; the
/// observable value is identical after every rebuild point).
fn build_status(inner: &DispatchInner) -> PyValue {
    let commands = inner.active_commands();
    let help = inner.help.borrow();
    let mut status_commands = IndexMap::new();
    for cmd in &commands {
        let mut entry = IndexMap::new();
        if let Some(desc) = help.get(cmd) {
            entry.insert(PyValue::Str("help".to_string()), PyValue::Str(desc.clone()));
        }
        status_commands.insert(PyValue::Str(cmd.clone()), PyValue::Dict(entry));
    }
    let mut status = IndexMap::new();
    status.insert(
        PyValue::Str("commands".to_string()),
        PyValue::Dict(status_commands),
    );
    PyValue::Dict(status)
}

// ---------------------------------------------------------------------------
// `GCodeDispatch`.
// ---------------------------------------------------------------------------

/// Port of `GCodeDispatch` (`klippy/gcode.py:99`).
pub struct GCodeDispatch {
    printer: Printer,
    inner: Arc<DispatchInner>,
}

impl GCodeDispatch {
    /// `GCodeDispatch(printer)` (`klippy/gcode.py:102-124`).
    pub fn new(printer: Printer) -> Self {
        let reactor = printer.get_reactor();
        let args = printer.get_start_args();
        let is_fileinput = start_arg_truthy(&args, "debuginput");
        let inner = Arc::new(DispatchInner {
            printer: printer.clone(),
            is_fileinput,
            is_printer_ready: GCell::new(false),
            mutex: reactor.mutex(false),
            reactor,
            outputs: Outputs::new(),
            ready: GCell::new(IndexMap::new()),
            base_order: GCell::new(Vec::new()),
            help: GCell::new(IndexMap::new()),
            mux: GCell::new(IndexMap::new()),
        });

        // Event handlers, in the Python `__init__` order.
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_READY,
                Box::new(move |_| {
                    i.on_ready();
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_SHUTDOWN,
                Box::new(move |_| {
                    i.on_shutdown();
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_DISCONNECT,
                Box::new(move |_| {
                    i.on_disconnect();
                    Ok(PyValue::None)
                }),
            );
        }

        let printer_m112 = printer.clone();
        let _ = inner.insert_command("M110", Some(Box::new(|_| Ok(()))), true, None);
        let _ = inner.insert_command(
            "M112",
            Some(Box::new(move |_| {
                printer_m112.invoke_shutdown(
                    "Shutdown due to M112 command",
                    PyValue::Dict(IndexMap::new()),
                );
                Ok(())
            })),
            true,
            None,
        );
        let printer_m115 = printer.clone();
        let _ = inner.insert_command(
            "M115",
            Some(Box::new(move |gcmd: &GCmd| {
                let args = printer_m115.get_start_args();
                let version = start_arg_as_str(&args, "software_version");
                let msg = pct(
                    "FIRMWARE_NAME:Klipper FIRMWARE_VERSION:%s",
                    &[pct_str(&version)],
                );
                let did_ack = gcmd.ack(Some(&msg));
                if !did_ack {
                    gcmd.respond_info(&msg, true);
                }
                Ok(())
            })),
            true,
            None,
        );
        {
            let i = Arc::clone(&inner);
            let _ = inner.insert_command(
                "RESTART",
                Some(Box::new(move |_| i.do_request_restart("restart"))),
                true,
                Some("Reload config file and restart host software"),
            );
        }
        {
            let i = Arc::clone(&inner);
            let _ = inner.insert_command(
                "FIRMWARE_RESTART",
                Some(Box::new(move |_| i.do_request_restart("firmware_restart"))),
                true,
                Some("Restart firmware, host, and reload config"),
            );
        }
        let _ = inner.insert_command(
            "ECHO",
            Some(Box::new(|gcmd: &GCmd| {
                gcmd.respond_info(&gcmd.get_commandline(), false);
                Ok(())
            })),
            true,
            None,
        );
        {
            let i = Arc::clone(&inner);
            let _ = inner.insert_command(
                "STATUS",
                Some(Box::new(move |_| {
                    if i.is_printer_ready.get() {
                        i.respond_state("Ready");
                        return Ok(());
                    }
                    let msg = i.state_message();
                    let msg = format!("{}\nKlipper state: Not ready", msg.trim_end());
                    Err(command_error(msg))
                })),
                true,
                Some("Report the printer status"),
            );
        }
        {
            let i = Arc::clone(&inner);
            let _ = inner.insert_command(
                "HELP",
                Some(Box::new(move |gcmd: &GCmd| {
                    let mut cmdhelp: Vec<String> = Vec::new();
                    if !i.is_printer_ready.get() {
                        cmdhelp.push(
                            "Printer is not ready - not all commands available.".to_string(),
                        );
                    }
                    cmdhelp.push("Available extended commands:".to_string());
                    let msgs = {
                        let help = i.help.borrow();
                        let mut cmds = i.active_commands();
                        cmds.sort();
                        let mut out = Vec::new();
                        for c in &cmds {
                            if let Some(desc) = help.get(c) {
                                out.push(pct("%-10s: %s", &[pct_str(c), pct_str(desc)]));
                            }
                        }
                        out
                    };
                    cmdhelp.extend(msgs);
                    gcmd.respond_info(&cmdhelp.join("\n"), false);
                    Ok(())
                })),
                true,
                Some("Report the list of available extended G-Code commands"),
            );
        }

        GCodeDispatch { printer, inner }
    }

}

impl PrinterObject for GCodeDispatch {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);

    fn teardown(&self) {
        let ready = self.inner.ready.replace(IndexMap::new());
        let mux = self.inner.mux.replace(IndexMap::new());
        let outputs = self.inner.outputs.callbacks.replace(Vec::new());
        self.inner.base_order.borrow_mut().clear();
        self.inner.help.borrow_mut().clear();
        drop((ready, mux, outputs));
    }

    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(build_status(&self.inner)))
    }
}

impl GCodeApi for GCodeDispatch {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.inner
            .insert_command(cmd, func, opts.when_not_ready, opts.desc.as_deref())
    }

    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        // First sight of the command: register the (placeholder) dispatcher
        // exactly as Python registers `lambda gcmd: self._cmd_mux(cmd, gcmd)`.
        {
            let mux = self.inner.mux.borrow();
            if !mux.contains_key(cmd) {
                drop(mux);
                self.inner
                    .insert_command(cmd, Some(Box::new(|_| Ok(()))), false, desc)?;
                let mut mux = self.inner.mux.borrow_mut();
                mux.insert(
                    cmd.to_string(),
                    MuxState { key: key.to_string(), values: IndexMap::new() },
                );
            }
        }
        let mut mux = self.inner.mux.borrow_mut();
        let state = mux.get_mut(cmd).expect("mux entry just inserted");
        if state.key != key {
            return Err(config_error(format!(
                "mux command {cmd} {key} {} may have only one key ({})",
                value.unwrap_or("None"),
                state.key
            )));
        }
        let value_key: Option<String> = value.map(|v| v.to_string());
        if state.values.contains_key(&value_key) {
            let repr = mux_dict_repr(&state.values);
            return Err(config_error(format!(
                "mux command {cmd} {key} {} already registered ({repr})",
                value.unwrap_or("None")
            )));
        }
        let _ = state.values.insert(value_key, Some(func));
        Ok(())
    }

    fn get_command_help(&self) -> IndexMap<String, String> {
        self.inner.help.borrow().clone()
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        build_status(&self.inner)
    }

    fn register_output_handler(&self, callback: OutputCallback) {
        self.inner.outputs.callbacks.borrow_mut().push(callback);
    }

    fn request_restart(&self, result: &str) -> KResult<()> {
        self.inner.do_request_restart(result)
    }

    fn run_script(&self, script: &str) -> KResult<()> {
        let commands: Vec<String> =
            script.split('\n').map(|s| s.to_string()).collect();
        self.inner.mutex.lock();
        let result = self.process_commands(&commands, false);
        self.inner.mutex.unlock();
        result
    }

    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        let commands: Vec<String> =
            script.split('\n').map(|s| s.to_string()).collect();
        self.process_commands(&commands, false)
    }

    fn get_mutex(&self) -> reactor::ReactorMutex {
        self.inner.mutex.clone()
    }

    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        let concrete = Arc::new(GCodeCommand {
            inner: GCell::new(crate::gcode_parse::GCodeCommand::new(
                command.to_string(),
                commandline.to_string(),
                params.clone(),
                false,
            )),
            shared: Arc::clone(&self.inner),
        });
        concrete
    }

    fn respond_raw(&self, msg: &str) {
        self.inner.respond_raw(msg);
    }

    fn respond_info(&self, msg: &str, log: bool) {
        self.inner.respond_info(msg, log);
    }

    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        gcode_parse::is_traditional_gcode(cmd)
    }

    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        self.printer.invoke_shutdown(
            "Shutdown due to M112 command",
            PyValue::Dict(IndexMap::new()),
        );
        Ok(())
    }

    fn process_commands(&self, commands: &[String], need_ack: bool) -> KResult<()> {
        for line in commands {
            let parsed = gcode_parse::parse_line(line);
            let cmd_for_msg = parsed.cmd.clone();
            let concrete = Arc::new(GCodeCommand {
                inner: GCell::new(crate::gcode_parse::GCodeCommand::from_parsed(
                    &parsed, need_ack,
                )),
                shared: Arc::clone(&self.inner),
            });
            let result = self.inner.invoke_handler(&concrete, &cmd_for_msg);
            match result {
                Ok(()) => {
                    concrete.ack_inner(None);
                }
                Err(KlippyError::Command(msg)) => {
                    self.inner.respond_error(&msg);
                    self.printer.send_event(EV_GCODE_COMMAND_ERROR, &[])?;
                    if !need_ack {
                        return Err(KlippyError::Command(msg));
                    }
                    concrete.ack_inner(None);
                }
                Err(other) => {
                    let msg = pct(
                        "Internal error on command:\"%s\"",
                        &[pct_str(&cmd_for_msg)],
                    );
                    log::exception(&msg, &other.exception_line());
                    self.printer
                        .invoke_shutdown(&msg, PyValue::Dict(IndexMap::new()));
                    self.inner.respond_error(&msg);
                    if !need_ack {
                        return Err(other);
                    }
                    concrete.ack_inner(None);
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// `GCodeCommand` (the `gcmd` handed to every handler).
// ---------------------------------------------------------------------------

/// Concrete [`GCodeCommandApi`] (`klippy/gcode.py:24`).
pub struct GCodeCommand {
    inner: GCell<crate::gcode_parse::GCodeCommand>,
    shared: Arc<DispatchInner>,
}

impl GCodeCommand {
    /// `ack` core: emits `ok` through the dispatcher output list.
    fn ack_inner(&self, msg: Option<&str>) -> bool {
        let out = self.inner.borrow_mut().ack(msg);
        match out {
            Some(ok) => {
                self.shared.respond_raw(&ok);
                true
            }
            None => false,
        }
    }

    fn respond_info(&self, msg: &str, log_enabled: bool) {
        self.shared.respond_info(msg, log_enabled);
    }

    fn commandline(&self) -> String {
        self.inner.borrow().get_commandline().to_string()
    }

    fn bound_compare_lt(&self, parsed: &PyValue, bound: Option<PyValue>) -> KResult<bool> {
        match bound {
            None => Ok(false),
            Some(b) => match py_rich_compare("<", parsed, &b) {
                Ok(v) => Ok(v),
                Err(text) => Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: text,
                }),
            },
        }
    }
}

impl GCodeCommandApi for GCodeCommand {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let (commandline, value_opt) = {
            let inner = self.inner.borrow();
            let v = inner.get_command_parameters().get(name).cloned();
            (inner.get_commandline().to_string(), v)
        };
        let Some(value) = value_opt else {
            return match opts.default {
                None => Err(command_error(format!(
                    "Error on '{commandline}': missing {name}"
                ))),
                Some(d) => Ok(d),
            };
        };
        let unable = || {
            command_error(format!(
                "Error on '{commandline}': unable to parse {value}"
            ))
        };
        let parsed: PyValue = match opts.parser {
            GetParser::Str => PyValue::Str(value.clone()),
            GetParser::Int => match int_from_string(&value, 10) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(unable()),
            },
            GetParser::IntAutoBase => match int_from_string(&value, 0) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(unable()),
            },
            GetParser::Float => match float_from_string(&value) {
                Ok(f) => PyValue::Float(f),
                Err(_) => return Err(unable()),
            },
        };
        // Bounds (`klippy/gcode.py:78-89`). A `TypeError` here is what the
        // Python would raise from the comparison itself, so it is internal.
        if self.bound_compare_lt(&parsed, opts.minval.clone())? {
            return Err(command_error(format!(
                "Error on '{commandline}': {name} must have minimum of {}",
                opts.minval.map(|v| v.str()).unwrap_or_default()
            )));
        }
        let cmp_gt = match opts.maxval.clone() {
            None => false,
            Some(b) => match py_rich_compare(">", &parsed, &b) {
                Ok(v) => v,
                Err(text) => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: text,
                    })
                }
            },
        };
        if cmp_gt {
            return Err(command_error(format!(
                "Error on '{commandline}': {name} must have maximum of {}",
                opts.maxval.map(|v| v.str()).unwrap_or_default()
            )));
        }
        let cmp_le = match opts.above.clone() {
            None => false,
            Some(b) => match py_rich_compare("<=", &parsed, &b) {
                Ok(v) => v,
                Err(text) => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: text,
                    })
                }
            },
        };
        if cmp_le {
            return Err(command_error(format!(
                "Error on '{commandline}': {name} must be above {}",
                opts.above.map(|v| v.str()).unwrap_or_default()
            )));
        }
        let cmp_ge = match opts.below.clone() {
            None => false,
            Some(b) => match py_rich_compare(">=", &parsed, &b) {
                Ok(v) => v,
                Err(text) => {
                    return Err(KlippyError::Internal {
                        class_name: "TypeError".to_string(),
                        message: text,
                    })
                }
            },
        };
        if cmp_ge {
            return Err(command_error(format!(
                "Error on '{commandline}': {name} must be below {}",
                opts.below.map(|v| v.str()).unwrap_or_default()
            )));
        }
        Ok(parsed)
    }

    fn get_int(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.inner
            .borrow()
            .get_int_opt(name, opts.default, opts.minval, opts.maxval)
            .map_err(map_parse_command_error)
    }

    fn get_float(&self, name: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.inner
            .borrow()
            .get_float_opt(name, opts.default, opts.minval, opts.maxval, opts.above, opts.below, opts.ints)
            .map_err(map_parse_command_error)
    }

    fn get_command(&self) -> String {
        self.inner.borrow().get_command().to_string()
    }

    fn get_commandline(&self) -> String {
        self.commandline()
    }

    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.inner.borrow().get_command_parameters().clone()
    }

    fn get_raw_command_parameters(&self) -> String {
        self.inner.borrow().get_raw_command_parameters()
    }

    fn respond_info(&self, msg: &str, log: bool) {
        self.respond_info(msg, log);
    }

    fn respond_raw(&self, msg: &str) {
        self.shared.respond_raw(msg);
    }

    fn ack(&self, msg: Option<&str>) -> bool {
        self.ack_inner(msg)
    }
}

// ---------------------------------------------------------------------------
// `GCodeIO`.
// ---------------------------------------------------------------------------

struct IoInner {
    printer: Printer,
    gcode: klippy_api::gcode::GCode,
    reactor: Reactor,
    /// `self.gcode_mutex = self.gcode.get_mutex()` (`klippy/gcode.py:394`).
    gcode_mutex: reactor::ReactorMutex,
    is_fileinput: bool,
    fd: Option<i32>,
    is_printer_ready: GCell<bool>,
    is_processing_data: GCell<bool>,
    pipe_is_active: GCell<bool>,
    fd_handle: GCell<Option<FdHandler>>,
    partial_input: GCell<String>,
    pending_commands: GCell<Vec<String>>,
    bytes_read: GCell<i64>,
    input_log: GCell<VecDeque<(f64, String)>>,
}

impl IoInner {
    /// `_respond_raw` (`klippy/gcode.py:478-484`).
    fn respond_raw(&self, msg: &str) {
        if !self.pipe_is_active.get() {
            return;
        }
        let Some(fd) = self.fd else { return };
        let mut bytes = (msg.to_string() + "\n").into_bytes();
        // `os.write` writes once; a short write keeps its prefix in Python
        // too (the return value is ignored).
        match kos::fd::write(fd, &bytes) {
            Ok(_) => {}
            Err(e) => {
                log::exception("Write g-code response", &format!("{e:?}"));
                self.pipe_is_active.set(false);
                let _ = &mut bytes;
            }
        }
    }

    fn handle_ready(&self, this: Arc<IoInner>) {
        self.is_printer_ready.set(true);
        if self.is_fileinput && this.fd_handle.borrow().is_none() {
            if let Some(fd) = self.fd {
                let arc = Arc::clone(&this);
                let handle = self
                    .reactor
                    .register_fd(fd, move |et| Arc::clone(&arc).process_data(et), None::<fn(f64)>);
                self.fd_handle.replace(Some(handle));
            }
        }
    }

    fn handle_shutdown(&self) {
        if !self.is_printer_ready.get() {
            return;
        }
        self.is_printer_ready.set(false);
        if self.is_fileinput {
            self.printer.request_exit("error_exit");
        }
    }

    fn handle_analyze_shutdown(&self) {
        let entries: Vec<(f64, String)> =
            self.input_log.borrow().iter().cloned().collect();
        log::info(&format_input_dump(&entries));
    }

    /// `_process_data` (`klippy/gcode.py:433-477`).
    fn process_data(self: Arc<Self>, eventtime: f64) {
        let Some(fd) = self.fd else { return };
        let mut buf = [0u8; 4096];
        let data: String = match kos::fd::read(fd, &mut buf) {
            Ok(n) => match String::from_utf8(buf[..n].to_vec()) {
                Ok(s) => s,
                Err(e) => {
                    log::exception("Read g-code", &e.to_string());
                    return;
                }
            },
            Err(e) => {
                let err = crate::webhooks::os_error(&e, None);
                log::exception(
                    "Read g-code",
                    &format!(
                        "Traceback (most recent call last):\n  File \"gcode.py\", line 0, in _process_data\n{}",
                        err.exception_line()
                    ),
                );
                return;
            }
        };
        {
            let mut log_copy = self.input_log.borrow_mut();
            log_copy.push_back((eventtime, data.clone()));
            while log_copy.len() > 50 {
                let _ = log_copy.pop_front();
            }
        }
        let char_count = data.chars().count() as i64;
        self.bytes_read.set(self.bytes_read.get() + char_count);
        // Split by '\n', re-join the partial first line.
        let mut lines: Vec<String> =
            data.split('\n').map(|s| s.to_string()).collect();
        let old_partial = self.partial_input.replace(String::new());
        if let Some(first) = lines.first_mut() {
            *first = old_partial + first;
        }
        let tail = lines.pop().unwrap_or_default();
        self.partial_input.replace(tail);
        self.pending_commands.borrow_mut().extend(lines.clone());
        self.pipe_is_active.set(true);
        // Debug file input EOF.
        if data.is_empty() && self.is_fileinput {
            if !self.is_processing_data.get() {
                self.unregister_fd();
                self.gcode.request_restart("exit")
                    .unwrap_or_else(|e| e.raise());
            }
            self.pending_commands.borrow_mut().push(String::new());
        }
        if self.pending_commands.borrow().len() < 20 {
            for line in &lines {
                if m112_matches(line) {
                    let _ = self.gcode.cmd_m112(None);
                }
            }
        }
        if self.is_processing_data.get() {
            if self.pending_commands.borrow().len() >= 20 {
                self.unregister_fd();
            }
            return;
        }
        self.is_processing_data.set(true);
        loop {
            let batch: Vec<String> =
                std::mem::take(&mut *self.pending_commands.borrow_mut());
            if batch.is_empty() {
                break;
            }
            self.gcode_mutex.lock();
            let result = self.gcode.process_commands(&batch, true);
            self.gcode_mutex.unlock();
            result.unwrap_or_else(|e| e.raise());
        }
        self.is_processing_data.set(false);
        if self.fd_handle.borrow().is_none() {
            let arc = Arc::clone(&self);
            let handle = self
                .reactor
                .register_fd(fd, move |et| Arc::clone(&arc).process_data(et), None::<fn(f64)>);
            self.fd_handle.replace(Some(handle));
        }
    }

    fn unregister_fd(&self) {
        let taken = self.fd_handle.replace(None);
        if let Some(handle) = taken {
            self.reactor.unregister_fd(&handle);
        }
    }

    fn register_fd(&self, this: Arc<IoInner>) {
        if let Some(fd) = self.fd {
            let arc = this;
            let handle = self
                .reactor
                .register_fd(fd, move |et| Arc::clone(&arc).process_data(et), None::<fn(f64)>);
            self.fd_handle.replace(Some(handle));
        }
    }

    fn stats(&self) -> (bool, String) {
        (false, pct("gcodein=%d", &[pct_int(self.bytes_read.get())]))
    }
}

/// Port of `GCodeIO` (`klippy/gcode.py:390`).
pub struct GCodeIO {
    inner: Arc<IoInner>,
}

impl GCodeIO {
    /// `GCodeIO(printer)` (`klippy/gcode.py:391-414`). The `gcode` object
    /// is looked up from the printer (raising `config_error` like Python).
    pub fn new(printer: Printer) -> KResult<GCodeIO> {
        let gcode = match klippy_api::lookup_interface::<dyn GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )? {
            Some(g) => g,
            None => {
                return Err(config_error(
                    "Unknown config object 'gcode'".to_string(),
                ))
            }
        };
        Ok(Self::with_gcode(printer, gcode))
    }

    /// Construct with an already-looked-up `gcode` (tests use this to skip
    /// the printer object store).
    pub fn with_gcode(printer: Printer, gcode: klippy_api::gcode::GCode) -> GCodeIO {
        let reactor = printer.get_reactor();
        let args = printer.get_start_args();
        let is_fileinput = start_arg_truthy(&args, "debuginput");
        let fd = start_arg_i32(&args, "gcode_fd");
        let inner = Arc::new(IoInner {
            printer: printer.clone(),
            gcode: gcode.clone(),
            reactor: reactor.clone(),
            gcode_mutex: gcode.get_mutex(),
            is_fileinput,
            fd,
            is_printer_ready: GCell::new(false),
            is_processing_data: GCell::new(false),
            pipe_is_active: GCell::new(true),
            fd_handle: GCell::new(None),
            partial_input: GCell::new(String::new()),
            pending_commands: GCell::new(Vec::new()),
            bytes_read: GCell::new(0),
            input_log: GCell::new(VecDeque::new()),
        });
        if !is_fileinput {
            let i = Arc::clone(&inner);
            gcode.register_output_handler(Box::new(move |msg| {
                i.respond_raw(&msg);
            }));
            inner.register_fd(Arc::clone(&inner));
        }
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_READY,
                Box::new(move |_| {
                    i.handle_ready(Arc::clone(&i));
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_SHUTDOWN,
                Box::new(move |_| {
                    i.handle_shutdown();
                    Ok(PyValue::None)
                }),
            );
        }
        {
            let i = Arc::clone(&inner);
            printer.register_event_handler(
                EV_KLIPPY_ANALYZE_SHUTDOWN,
                Box::new(move |_| {
                    i.handle_analyze_shutdown();
                    Ok(PyValue::None)
                }),
            );
        }
        GCodeIO { inner }
    }

    /// `_process_data` entry point, for the reactor callback and tests.
    pub fn process_data(&self, eventtime: f64) {
        Arc::clone(&self.inner).process_data(eventtime);
    }

    /// Bytes read so far (`stats` numerator).
    pub fn bytes_read(&self) -> i64 {
        self.inner.bytes_read.get()
    }

    /// Currently pending complete lines.
    pub fn pending_commands(&self) -> Vec<String> {
        self.inner.pending_commands.borrow().clone()
    }

    /// Whether writes are still attempted (`pipe_is_active`).
    pub fn pipe_is_active(&self) -> bool {
        self.inner.pipe_is_active.get()
    }

    /// `_respond_raw` entry point, for tests.
    pub fn respond_raw(&self, msg: &str) {
        self.inner.respond_raw(msg);
    }
}

impl PrinterObject for GCodeIO {
    klippy_api::printer_object_base!();

    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        Some(self.inner.stats())
    }
}

// ---------------------------------------------------------------------------
// Module entry points.
// ---------------------------------------------------------------------------

/// `add_early_printer_objects(printer)` (`klippy/gcode.py:488-490`).
pub fn add_early_printer_objects(printer: &Printer) -> KResult<()> {
    let dispatch = GCodeDispatch::new(Arc::clone(printer));
    let dispatch_ref: ObjectRef = Arc::new(dispatch);
    printer.add_object("gcode", dispatch_ref)?;
    let io = GCodeIO::new(Arc::clone(printer))?;
    printer.add_object("gcode_io", Arc::new(io))?;
    Ok(())
}

/// Module factory for this module (`klippy/gcode.py` has no `load_config`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: None,
        add_early_printer_objects: Some(add_early_printer_objects),
        add_printer_objects: None,
    }
}

