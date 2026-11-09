//! Port of the `sdcard_loop` module (`klippy/extras/sdcard_loop.py`).
//!
//! Implements SD-card file looping: `SDCARD_LOOP_BEGIN COUNT=` pushes the
//! current file position, `SDCARD_LOOP_END` seeks back to it while
//! iterations remain, and `SDCARD_LOOP_DESIST` clears the stack when the
//! command comes from outside the SD file. No config options or status
//! fields are read by this module.
//!
//! The `virtual_sdcard` dependency is reached through the owner's
//! [`VirtualSDApi`] (`crate::virtual_sdcard`) and `gcode` through
//! [`GCodeApi`]; both are resolved inside [`SDCardLoop::new`] exactly as the
//! reference `__init__` does (`printer.load_object(config,
//! "virtual_sdcard")`, `printer.lookup_object('gcode')`).

use std::sync::Arc;

use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCmd, GetIntOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::object::{load_interface, lookup_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{printer_object, GCell};
use crate::virtual_sdcard::VirtualSDApi;

/// `SDCARD_LOOP_BEGIN` help text (`klippy/extras/sdcard_loop.py:24`).
pub const BEGIN_HELP: &str = "Begins a looped section in the SD file.";
/// `SDCARD_LOOP_END` help text (`klippy/extras/sdcard_loop.py:29`).
pub const END_HELP: &str = "Ends a looped section in the SD file.";
/// `SDCARD_LOOP_DESIST` help text (`klippy/extras/sdcard_loop.py:33`).
pub const DESIST_HELP: &str = "Stops iterating the current loop stack.";

/// `SDCardLoop` (`klippy/extras/sdcard_loop.py:9`).
///
/// Mutable state (`loop_stack`, a list of `(count, position)` tuples) lives
/// in a [`GCell`] mirroring the Python instance list; the `sdcard` handle
/// mirrors `self.sdcard`. Python also stores `self.gcode`
/// (`sdcard_loop.py:13`) but never uses it after registration, so the port
/// drops it (no observable state).
pub struct SDCardLoop {
    /// `self.sdcard` (`sdcard_loop.py:12`).
    sdcard: Arc<dyn VirtualSDApi>,
    /// `self.loop_stack` (`sdcard_loop.py:23`): `(count, position)` pairs.
    loop_stack: GCell<Vec<(i64, i64)>>,
}

printer_object!(SDCardLoop);

impl SDCardLoop {
    /// `SDCardLoop.__init__` (`sdcard_loop.py:10-23`): loads
    /// `virtual_sdcard`, looks up `gcode`, registers the three G-code
    /// commands, and returns the shared object.
    ///
    /// # Errors
    /// Propagates `KlippyError::Config` when `virtual_sdcard`/`gcode` is
    /// missing (or lacks the interface) and from `register_command`
    /// (duplicate or invalid command name).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let sdcard = load_interface::<dyn VirtualSDApi>(
            &*printer,
            config,
            "virtual_sdcard",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let me = Arc::new(SDCardLoop {
            sdcard,
            loop_stack: GCell::new(Vec::new()),
        });
        {
            let m = me.clone();
            gcode.register_command(
                "SDCARD_LOOP_BEGIN",
                Some(Box::new(move |gcmd: &GCmd| m.cmd_SDCARD_LOOP_BEGIN(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(BEGIN_HELP.to_string()),
                },
            )?;
        }
        {
            let m = me.clone();
            gcode.register_command(
                "SDCARD_LOOP_END",
                Some(Box::new(move |gcmd: &GCmd| m.cmd_SDCARD_LOOP_END(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(END_HELP.to_string()),
                },
            )?;
        }
        {
            let m = me.clone();
            gcode.register_command(
                "SDCARD_LOOP_DESIST",
                Some(Box::new(move |gcmd: &GCmd| m.cmd_SDCARD_LOOP_DESIST(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(DESIST_HELP.to_string()),
                },
            )?;
        }
        Ok(me)
    }

    /// `cmd_SDCARD_LOOP_BEGIN` (`sdcard_loop.py:25-28`).
    pub fn cmd_SDCARD_LOOP_BEGIN(&self, gcmd: &GCmd) -> KResult<()> {
        let count = gcmd
            .get_int(
                "COUNT",
                GetIntOpts {
                    default: None,
                    minval: Some(0),
                    maxval: None,
                },
            )?
            .expect("get_int with required default returns Some");
        if !self.loop_begin(count) {
            return Err(KlippyError::command("Only permitted in SD file."));
        }
        Ok(())
    }

    /// `cmd_SDCARD_LOOP_END` (`sdcard_loop.py:30-32`).
    pub fn cmd_SDCARD_LOOP_END(&self, _gcmd: &GCmd) -> KResult<()> {
        if !self.loop_end() {
            return Err(KlippyError::command("Only permitted in SD file."));
        }
        Ok(())
    }

    /// `cmd_SDCARD_LOOP_DESIST` (`sdcard_loop.py:34-36`).
    pub fn cmd_SDCARD_LOOP_DESIST(&self, _gcmd: &GCmd) -> KResult<()> {
        if !self.loop_desist() {
            return Err(KlippyError::command(
                "Only permitted outside of a SD file.",
            ));
        }
        Ok(())
    }

    /// `loop_begin` (`sdcard_loop.py:37-42`).
    ///
    /// Pushes `(count, get_file_position())` when the command comes from
    /// the SD file, otherwise returns `false`.
    pub fn loop_begin(&self, count: i64) -> bool {
        if !self.sdcard.is_cmd_from_sd() {
            return false;
        }
        let position = self.sdcard.get_file_position();
        self.loop_stack.borrow_mut().push((count, position));
        true
    }

    /// `loop_end` (`sdcard_loop.py:43-63`).
    ///
    /// Empty stack is a no-op success. `count == 0` re-arms the same entry
    /// (infinite loop); `count == 1` drops it (last repeat); otherwise the
    /// entry is re-pushed decremented. The file position is seeked back
    /// for every case except `count == 1` and the empty stack.
    pub fn loop_end(&self) -> bool {
        if !self.sdcard.is_cmd_from_sd() {
            return false;
        }
        let (count, position) = match self.loop_stack.borrow_mut().pop() {
            Some(entry) => entry,
            None => return true,
        };
        if count == 0 {
            self.sdcard.set_file_position(position);
            self.loop_stack.borrow_mut().push((count, position));
        } else if count == 1 {
            // Last repeat: nothing to do.
        } else {
            self.sdcard.set_file_position(position);
            self.loop_stack.borrow_mut().push((count - 1, position));
        }
        true
    }

    /// `loop_desist` (`sdcard_loop.py:64-70`).
    ///
    /// Outside an SD file, logs and clears the stack; inside one, fails.
    pub fn loop_desist(&self) -> bool {
        if self.sdcard.is_cmd_from_sd() {
            return false;
        }
        log::info("Desisting existing SD loops");
        self.loop_stack.borrow_mut().clear();
        true
    }

    /// Snapshot of `loop_stack` for tests. The Python attribute is a plain
    /// list, so inspection is the faithful observable.
    pub fn loop_stack_snapshot(&self) -> Vec<(i64, i64)> {
        self.loop_stack.cloned()
    }
}

/// `load_config` (`sdcard_loop.py:72-73`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(SDCardLoop::new(config)? as ObjectRef))
}

/// Module factory for the printer (`load_config` only, matching
/// `sdcard_loop.py` which has no other entry points).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
