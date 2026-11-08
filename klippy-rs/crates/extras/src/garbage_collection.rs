// Port of the `garbage_collection` module (`klippy/extras/garbage_collection.py`).
//! Garbage collection optimization
//!
//! Copyright (C) 2025  Branden Cash <ammmze@gmail.com>
//!
//! This file may be distributed under the terms of the GNU GPLv3 license.

use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::log;
use klippy_api::printer::{ModuleFactory, Printer, EV_KLIPPY_DISCONNECT, EV_KLIPPY_READY};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use std::sync::Arc;

/// Event names exactly as the reference registers them.
/// The printer sends both without parameters (`klippy/klippy.py:165,201`).
pub const READY_EVENT: &str = EV_KLIPPY_READY;
pub const DISCONNECT_EVENT: &str = EV_KLIPPY_DISCONNECT;

/// Debug log emitted by `_handle_ready` (`klippy/extras/garbage_collection.py:21`).
pub const READY_LOG: &str = "Running full garbage collection and freezing";
/// Debug log emitted by `_handle_disconnect` (`klippy/extras/garbage_collection.py:27`).
pub const DISCONNECT_LOG: &str = "Unfreezing garbage collection";

/// Whether `gc.freeze()` / `gc.unfreeze()` are available.
///
/// The reference feature check is `hasattr(gc, 'freeze') and
/// hasattr(gc, 'unfreeze')` (`gc` exists since Python 3.7, so on the
/// reference interpreter always true). Rust has no Python `gc` object;
/// the port is built for interpreters where `gc.freeze`/`gc.unfreeze`
/// exist, so this is always true and both handlers are always
/// registered. On a pre-3.7 interpreter the reference would register
/// nothing; that case does not occur on this target.
fn can_freeze() -> bool {
    true
}

/// Mirrors Python `gc.collect(n)`. Rust has no Python reference collector
/// on a Python heap, nothing here is heap-scan observable; the call order
/// `collect(0); collect(1); collect(2); freeze()` is preserved so any future
/// mapping keeps the same sequence. Intentionally a no-op.
fn gc_collect(_generation: i32) {}

/// Mirrors `gc.freeze()`: move all objects to a permanent generation and
/// exclude them from future collections. Rust objects are not Python
/// GC objects, so no equivalent operation exists; the observable effect
/// (event wiring plus the debug log above) is preserved. Intentionally
/// a no-op that records the frozen flag on the module state instead.
fn gc_freeze() {}

/// Mirrors `gc.unfreeze()`. See `gc_freeze`.
fn gc_unfreeze() {}

/// Shared state so registered event callbacks update the same object
/// Python's bound method would (per-printer, in-process).
pub struct GarbageCollectionState {
    frozen: GCell<bool>,
}

impl GarbageCollectionState {
    fn new() -> Self {
        GarbageCollectionState { frozen: GCell::new(false) }
    }
    fn is_frozen(&self) -> bool {
        self.frozen.get()
    }
    fn set_frozen(&self, frozen: bool) {
        self.frozen.set(frozen);
    }
}

/// `_handle_ready` (`klippy/extras/garbage_collection.py:20`). Logs, then
/// `gc.collect(0)`, `gc.collect(1)`, `gc.collect(2)`, `gc.freeze()`.
fn run_handle_ready(state: &GarbageCollectionState) -> KResult<PyValue> {
    log::debug(READY_LOG);
    for n in 0..3 {
        gc_collect(n);
    }
    gc_freeze();
    state.set_frozen(true);
    Ok(PyValue::None)
}

/// `_handle_disconnect` (`klippy/extras/garbage_collection.py:26`). Logs,
/// then `gc.unfreeze()`.
fn run_handle_disconnect(state: &GarbageCollectionState) -> KResult<PyValue> {
    log::debug(DISCONNECT_LOG);
    gc_unfreeze();
    state.set_frozen(false);
    Ok(PyValue::None)
}

/// Python bound-method arity check: `_handle_ready` / `_handle_disconnect`
/// take no arguments beyond `self`. `klippy:ready` and
/// `klippy:disconnect` are always sent without parameters, so this never
/// fires from the printer; it reproduces CPython's `TypeError` for any
/// other caller exactly.
fn check_no_args(method: &str, params: &[EventArg]) -> KResult<()> {
    let given = params.len() + 1;
    if given != 1 {
        return Err(klippy_api::KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "{method}() takes 1 positional argument but {given} were given"
            ),
        });
    }
    Ok(())
}

/// `klippy/extras/garbage_collection.py` class.
pub struct GarbageCollection {
    printer: Printer,
    state: Arc<GarbageCollectionState>,
    registered: bool,
}

impl GarbageCollection {
    /// `GarbageCollection.__init__(config)` (`klippy/extras/garbage_collection.py:10`):
    /// stores `config.get_printer()`; when `gc.freeze`/`gc.unfreeze` exist,
    /// registers `_handle_ready` on `klippy:ready` and `_handle_disconnect`
    /// on `klippy:disconnect`.
    pub fn new(config: &Config) -> Self {
        let printer = config.get_printer();
        let can_freeze = can_freeze();
        let state = Arc::new(GarbageCollectionState::new());
        if can_freeze {
            let ready_state = state.clone();
            printer.register_event_handler(
                READY_EVENT,
                Box::new(move |params: &[EventArg]| {
                    check_no_args("_handle_ready", params)?;
                    run_handle_ready(&ready_state)
                }),
            );
            let disconnect_state = state.clone();
            printer.register_event_handler(
                DISCONNECT_EVENT,
                Box::new(move |params: &[EventArg]| {
                    check_no_args("_handle_disconnect", params)?;
                    run_handle_disconnect(&disconnect_state)
                }),
            );
        }
        GarbageCollection { printer, state, registered: can_freeze }
    }

    /// `_handle_ready()` direct invocation (as the event callback calls it).
    pub fn handle_ready(&self) -> KResult<PyValue> {
        run_handle_ready(&self.state)
    }

    /// `_handle_disconnect()` direct invocation.
    pub fn handle_disconnect(&self) -> KResult<PyValue> {
        run_handle_disconnect(&self.state)
    }

    /// Whether the module registered its event handlers (feature check).
    pub fn is_registered(&self) -> bool {
        self.registered
    }

    /// Whether the frozen flag is set (after ready, before disconnect).
    pub fn is_frozen(&self) -> bool {
        self.state.is_frozen()
    }

    /// The printer the module stored (`self.printer`).
    pub fn printer(&self) -> &Printer {
        &self.printer
    }
}

klippy_api::printer_object!(GarbageCollection);

/// `load_config(config)` (`klippy/extras/garbage_collection.py:30`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(GarbageCollection::new(config)) as ObjectRef))
}

/// Module entry point for the `garbage_collection` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
