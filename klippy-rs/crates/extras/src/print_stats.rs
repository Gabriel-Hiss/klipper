//! Port of the `print_stats` module (`klippy/extras/print_stats.py`).
//!
//! `PrintStats` tracks virtual-SD-card print statistics: filament usage
//! (derived from `gcode_move` E positions), pause accumulation, total /
//! print / init durations, the printer state machine (`standby`,
//! `printing`, `paused`, `complete`, `error`, `cancelled`) and the
//! slicer layer info set via `SET_PRINT_STATS_INFO`.
//!
//! Durations come from `reactor.monotonic()`; `get_status(eventtime)`
//! behaviour matches Python exactly, including the in-place updates
//! (`_update_filament_usage`, `total_duration`, `init_duration`) the
//! Python call performs.

use std::sync::Arc;

use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCodeApi, GCodeMoveApi, GetIntOpts, RegisterCommandOpts};
use klippy_api::object::{load_interface, lookup_interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use pycompat::PyValue;
use pycompat::value::type_name;
use reactor::Reactor;

/// `cmd_SET_PRINT_STATS_INFO_help` (`print_stats.py:72`): the two
/// adjacent Python string literals, concatenated.
const CMD_HELP: &str = "Pass slicer info like layer act and total to klipper";

/// The `print_stats` printer-object service: `virtual_sdcard.py` calls
/// these methods on the `print_stats` object.
pub trait PrintStatsApi: Send + Sync + 'static {
    fn set_current_file(&self, filename: &str);
    fn note_start(&self);
    fn note_pause(&self);
    fn note_complete(&self);
    fn note_error(&self, message: &str);
    fn note_cancel(&self);
    fn reset(&self);
}

/// Mutable state of [`PrintStats`] (all `self.*` fields of the Python
/// class, except the services).
#[derive(Clone)]
struct Inner {
    filename: String,
    error_message: String,
    state: String,
    prev_pause_duration: f64,
    last_epos: f64,
    filament_used: f64,
    total_duration: f64,
    print_start_time: Option<f64>,
    last_pause_time: Option<f64>,
    init_duration: f64,
    info_total_layer: Option<i64>,
    info_current_layer: Option<i64>,
}

impl Default for Inner {
    fn default() -> Inner {
        Inner {
            filename: String::new(),
            error_message: String::new(),
            state: String::new(),
            prev_pause_duration: 0.,
            last_epos: 0.,
            filament_used: 0.,
            total_duration: 0.,
            print_start_time: None,
            last_pause_time: None,
            init_duration: 0.,
            info_total_layer: None,
            info_current_layer: None,
        }
    }
}

/// Port of the `PrintStats` class (`klippy/extras/print_stats.py:7`).
pub struct PrintStats {
    inner: GCell<Inner>,
    gcode_move: Arc<dyn GCodeMoveApi>,
    reactor: Reactor,
}

impl PrintStats {
    /// Port of `__init__` (`print_stats.py:8`): resolves `gcode_move`
    /// via `load_interface` and `gcode` via `lookup_interface`, takes
    /// the reactor, runs `reset()`, then registers the
    /// `SET_PRINT_STATS_INFO` command (`print_stats.py:15`) and the
    /// `extruder:activate_extruder` handler (`print_stats.py:18`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let gcode_move = load_interface::<dyn GCodeMoveApi>(
            &*printer,
            config,
            "gcode_move",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let reactor = printer.get_reactor();
        let this = Arc::new(PrintStats {
            inner: GCell::new(Inner::default()),
            gcode_move,
            reactor,
        });
        this.reset();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        this.register(&printer, &gcode)?;
        Ok(this)
    }

    /// Registers `SET_PRINT_STATS_INFO` (`print_stats.py:15`) and the
    /// `extruder:activate_extruder` handler (`print_stats.py:18`),
    /// in that order, as Python `__init__` does. Needs `&Arc<Self>`
    /// so the closures can own the object they call back into.
    fn register(self: &Arc<Self>, printer: &Printer, gcode: &Arc<dyn GCodeApi>) -> KResult<()> {
        let for_command = Arc::clone(self);
        gcode.register_command(
            "SET_PRINT_STATS_INFO",
            Some(Box::new(move |gcmd| {
                for_command.cmd_set_print_stats_info(gcmd)
            })),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_HELP.to_string()),
            },
        )?;
        let for_event = Arc::clone(self);
        printer.register_event_handler(
            "extruder:activate_extruder",
            Box::new(move |_args: &[EventArg]| {
                for_event.handle_activate_extruder();
                Ok(PyValue::None)
            }),
        );
        Ok(())
    }

    /// Port of `_handle_activate_extruder` (`print_stats.py:20`).
    fn handle_activate_extruder(&self) {
        let status = self.gcode_move.get_status(None);
        let e = dict_position_e(&status);
        self.inner.with_mut(|g| g.last_epos = e);
    }

    /// Port of `_update_filament_usage` (`print_stats.py:23`).
    fn update_filament_usage(&self, eventtime: f64) {
        let last_epos = self.inner.borrow().last_epos;
        let status = self.gcode_move.get_status(Some(eventtime));
        let cur_epos = dict_position_e(&status);
        let extrude_factor = dict_float(&status, "extrude_factor");
        self.inner.with_mut(|g| {
            g.filament_used += (cur_epos - last_epos) / extrude_factor;
            g.last_epos = cur_epos;
        });
    }

    /// Port of `set_current_file` (`print_stats.py:29`).
    pub fn set_current_file(&self, filename: &str) {
        self.reset();
        self.inner.with_mut(|g| g.filename = filename.to_string());
    }

    /// Port of `note_start` (`print_stats.py:32`).
    pub fn note_start(&self) {
        let curtime = self.reactor.monotonic();
        self.inner.with_mut(|g| {
            if g.print_start_time.is_none() {
                g.print_start_time = Some(curtime);
            } else if let Some(last_pause) = g.last_pause_time {
                g.prev_pause_duration += curtime - last_pause;
                g.last_pause_time = None;
            }
        });
        let status = self.gcode_move.get_status(Some(curtime));
        let e = dict_position_e(&status);
        self.inner.with_mut(|g| {
            g.last_epos = e;
            g.state = "printing".to_string();
            g.error_message.clear();
        });
    }

    /// Port of `note_pause` (`print_stats.py:46`).
    pub fn note_pause(&self) {
        if self.inner.borrow().last_pause_time.is_none() {
            let curtime = self.reactor.monotonic();
            self.inner.with_mut(|g| g.last_pause_time = Some(curtime));
            self.update_filament_usage(curtime);
        }
        self.inner.with_mut(|g| {
            if g.state != "error" {
                g.state = "paused".to_string();
            }
        });
    }

    /// Port of `note_complete` (`print_stats.py:54`).
    pub fn note_complete(&self) {
        self.note_finish("complete", "");
    }

    /// Port of `note_error` (`print_stats.py:56`).
    pub fn note_error(&self, message: &str) {
        self.note_finish("error", message);
    }

    /// Port of `note_cancel` (`print_stats.py:58`).
    pub fn note_cancel(&self) {
        self.note_finish("cancelled", "");
    }

    /// Port of `_note_finish` (`print_stats.py:60`).
    fn note_finish(&self, state: &str, error_message: &str) {
        let start = self.inner.borrow().print_start_time;
        let Some(print_start_time) = start else { return };
        let eventtime = self.reactor.monotonic();
        self.inner.with_mut(|g| {
            g.state = state.to_string();
            g.error_message = error_message.to_string();
            g.total_duration = eventtime - print_start_time;
            if g.filament_used < 0.0000001 {
                g.init_duration = g.total_duration - g.prev_pause_duration;
            }
            g.print_start_time = None;
        });
    }

    /// Port of `cmd_SET_PRINT_STATS_INFO` (`print_stats.py:74`).
    pub fn cmd_set_print_stats_info(&self, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let (info_total, info_current) = {
            let g = self.inner.borrow();
            (g.info_total_layer, g.info_current_layer)
        };
        let total_layer = gcmd.get_int(
            "TOTAL_LAYER",
            GetIntOpts {
                default: Some(info_total),
                minval: Some(0),
                maxval: None,
            },
        )?;
        let current_layer = gcmd.get_int(
            "CURRENT_LAYER",
            GetIntOpts {
                default: Some(info_current),
                minval: Some(0),
                maxval: None,
            },
        )?;
        self.inner.with_mut(|g| {
            if total_layer == Some(0) {
                g.info_total_layer = None;
                g.info_current_layer = None;
            } else if total_layer != g.info_total_layer {
                g.info_total_layer = total_layer;
                g.info_current_layer = Some(0);
            }
            if let (Some(total), Some(current)) = (g.info_total_layer, current_layer) {
                if Some(current) != g.info_current_layer {
                    g.info_current_layer = Some(current.min(total));
                }
            }
        });
        Ok(())
    }

    /// Port of `reset` (`print_stats.py:90`).
    pub fn reset(&self) {
        self.inner.with_mut(|g| {
            g.filename.clear();
            g.error_message.clear();
            g.state = "standby".to_string();
            g.prev_pause_duration = 0.;
            g.last_epos = 0.;
            g.filament_used = 0.;
            g.total_duration = 0.;
            g.print_start_time = None;
            g.last_pause_time = None;
            g.init_duration = 0.;
            g.info_total_layer = None;
            g.info_current_layer = None;
        });
    }

    /// Port of `get_status` (`print_stats.py:99`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        let (prev_pause_duration, print_start_time, last_pause_time) = {
            let g = self.inner.borrow();
            (
                g.prev_pause_duration,
                g.print_start_time,
                g.last_pause_time,
            )
        };
        let mut time_paused = prev_pause_duration;
        if let Some(print_start_time) = print_start_time {
            if let Some(last_pause_time) = last_pause_time {
                time_paused += eventtime - last_pause_time;
            } else {
                self.update_filament_usage(eventtime);
            }
            let total_duration = eventtime - print_start_time;
            let filament_used = self.inner.borrow().filament_used;
            self.inner.with_mut(|g| {
                g.total_duration = total_duration;
                if filament_used < 0.0000001 {
                    g.init_duration = total_duration - time_paused;
                }
            });
        }
        let (total_duration, init_duration, filament_used, filename, state, message) = {
            let g = self.inner.borrow();
            (
                g.total_duration,
                g.init_duration,
                g.filament_used,
                g.filename.clone(),
                g.state.clone(),
                g.error_message.clone(),
            )
        };
        let print_duration = total_duration - init_duration - time_paused;
        let (info_total_layer, info_current_layer) = {
            let g = self.inner.borrow();
            (g.info_total_layer, g.info_current_layer)
        };
        let mut info = indexmap::IndexMap::new();
        info.insert(
            PyValue::Str("total_layer".to_string()),
            info_layer(info_total_layer),
        );
        info.insert(
            PyValue::Str("current_layer".to_string()),
            info_layer(info_current_layer),
        );
        let mut status = indexmap::IndexMap::new();
        status.insert(PyValue::Str("filename".to_string()), PyValue::Str(filename));
        status.insert(
            PyValue::Str("total_duration".to_string()),
            PyValue::Float(total_duration),
        );
        status.insert(
            PyValue::Str("print_duration".to_string()),
            PyValue::Float(print_duration),
        );
        status.insert(
            PyValue::Str("filament_used".to_string()),
            PyValue::Float(filament_used),
        );
        status.insert(PyValue::Str("state".to_string()), PyValue::Str(state));
        status.insert(PyValue::Str("message".to_string()), PyValue::Str(message));
        status.insert(PyValue::Str("info".to_string()), PyValue::Dict(info));
        PyValue::Dict(status)
    }
}

impl PrintStatsApi for PrintStats {
    fn set_current_file(&self, filename: &str) {
        PrintStats::set_current_file(self, filename);
    }

    fn note_start(&self) {
        PrintStats::note_start(self);
    }

    fn note_pause(&self) {
        PrintStats::note_pause(self);
    }

    fn note_complete(&self) {
        PrintStats::note_complete(self);
    }

    fn note_error(&self, message: &str) {
        PrintStats::note_error(self, message);
    }

    fn note_cancel(&self) {
        PrintStats::note_cancel(self);
    }

    fn reset(&self) {
        PrintStats::reset(self);
    }
}

impl PrinterObject for PrintStats {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrintStatsApi);
    fn get_status(&self, eventtime: f64) -> Option<PyValue> {
        Some(PrintStats::get_status(self, eventtime))
    }
}

/// Layers are `int` or `None` in the status dict (`print_stats.py:120`).
fn info_layer(layer: Option<i64>) -> PyValue {
    match layer {
        Some(v) => PyValue::Int(pycompat::PyInt::from_i64(v)),
        None => PyValue::None,
    }
}

/// `status[key]` from a `gcode_move` status dict. Panics with the
/// Python exception text when the key is absent or the value is not a
/// dict: the input cannot occur in the ported system (the
/// `GCodeMoveApi::get_status` contract always return the dict), so
/// the failure — like Python's unhandled raise — is a loud bug
/// signal (compare `GCell` borrow panics).
fn dict_get<'a>(status: &'a PyValue, key: &str) -> &'a PyValue {
    match status {
        PyValue::Dict(m) => match m.get(&PyValue::Str(key.to_string())) {
            Some(v) => v,
            None => panic!("KeyError: {}", pycompat::value::repr_key(key)),
        },
        other => panic!(
            "TypeError: '{}' object is not subscriptable",
            type_name(other)
        ),
    }
}

/// `status['position'].e`: the fourth element of the position tuple
/// (`gcode.Coord` index 3, `gcode.py:21`).
fn dict_position_e(status: &PyValue) -> f64 {
    let position = dict_get(status, "position");
    match position {
        PyValue::Tuple(items) => match items.get(3) {
            Some(v) => val_float(v),
            None => panic!("IndexError: tuple index out of range"),
        },
        other => panic!(
            "AttributeError: '{}' object has no attribute 'e'",
            type_name(other)
        ),
    }
}

fn dict_float(status: &PyValue, key: &str) -> f64 {
    val_float(dict_get(status, key))
}

/// A numeric status value as `f64`. Panics with the Python
/// `TypeError` text for non-numeric values (see [`dict_get`]).
fn val_float(v: &PyValue) -> f64 {
    match v.to_f64() {
        Some(f) => f,
        None => panic!(
            "TypeError: unsupported operand type(s) for -: '{}' and 'float'",
            type_name(v)
        ),
    }
}

/// Port of `load_config` (`print_stats.py:124`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = PrintStats::new(config)?;
    Ok(Some(obj as ObjectRef))
}

/// Module factory (`print_stats.py:124`): section `print_stats`
/// dispatches to [`load_config`] on a single-token section name.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
