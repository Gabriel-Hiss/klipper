//! Contract of the `printer` service (`klippy/klippy.py:25`).
//!
//! Source inventory: `local://contract/printer.md`.
//! All module/event/function names below are the literal Python strings so
//! fakes and the core agree byte for byte.
//!
//! Notes:
//! - `printer.config_error` / `printer.command_error` are not trait
//!   methods; they are `KlippyError::Config` / `KlippyError::Command`
//!   (`klippy/klippy.py:26-27`).
//! - Non core-object event payloads (`HomingMove`, `MCUStepper`,
//!   `PrinterRail`, `LoadCell`) travel as `EventArg::Any` /
//!   `EventArg::List`, not as the canonical
//!   `crate::{mcu,stepper,toolhead}` handles. The printer never
//!   downcasts them.
//! - Private members `_set_state`, `_read_config`, `_connect`,
//!   `_debug_shutdown`, `_debug_host_state` are declared nowhere here:
//!   the inventory shows no external caller, so only the core uses them.

use crate::error::KResult;
use crate::config::Config;
use crate::object::{EventArg, ObjectRef};
use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::Reactor;
use std::sync::Arc;

/// Shared handle to the printer service (`Arc<dyn PrinterApi>`).
pub type Printer = Arc<dyn PrinterApi>;

/// Event callback: `cb(*params) -> Any` in Python. The return value is
/// collected by `send_event`; handlers that raise `command_error` abort
/// the dispatch and propagate.
pub type EventCallback = Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>;

/// How `lookup_object` / `load_object` treats a missing name. Python
/// passes either the `configfile.sentinel` (raise `config_error`) or
/// `None` (return `None`); `default` is never a live object.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LookupDefault {
    /// `default=sentinel`: missing name raises `KlippyError::Config`.
    #[default]
    Raise,
    /// `default=None`: missing name yields `None`.
    ReturnNone,
}

/// Decoded view of `start_args` (`klippy/klippy.py:356-404`). The
/// Python dict is mutable and shared by reference; this struct is a
/// snapshot for typed readers. `get_start_args` itself stays `PyValue`.
#[derive(Debug, Clone, Default)]
pub struct StartArgs {
    /// `config_file` (`klippy/klippy.py:356`).
    pub config_file: String,
    /// `apiserver` (`klippy/klippy.py:356`).
    pub apiserver: Option<String>,
    /// `start_reason` (`klippy/klippy.py:357`).
    pub start_reason: String,
    /// `debuginput` (`klippy/klippy.py:363`).
    pub debuginput: Option<String>,
    /// `gcode_fd` (`klippy/klippy.py:365`).
    pub gcode_fd: Option<i64>,
    /// `debugoutput` (`klippy/klippy.py:369`).
    pub debugoutput: Option<String>,
    /// `dictionary` (`klippy/klippy.py:370`).
    pub dictionary: Option<String>,
    /// `dictionary_<mcu>` entries (`klippy/klippy.py:370`).
    pub dictionaries: IndexMap<String, String>,
    /// `log_file` (`klippy/klippy.py:373`).
    pub log_file: Option<String>,
    /// `software_version` (`klippy/klippy.py:401`).
    pub software_version: String,
    /// `cpu_info` (`klippy/klippy.py:402`).
    pub cpu_info: Option<String>,
    /// `device` (`klippy/klippy.py:403`).
    pub device: Option<String>,
    /// `linux_version` (`klippy/klippy.py:404`).
    pub linux_version: Option<String>,
}

/// The `Printer` object (`klippy/klippy.py:25`).
pub trait PrinterApi: Send + Sync + 'static {
    /// `klippy/klippy.py:44` `get_start_args()` — start_args dict as `PyValue`.
    fn get_start_args(&self) -> PyValue;
    /// `klippy/klippy.py:46` `get_reactor()` — main reactor handle.
    fn get_reactor(&self) -> Reactor;
    /// `klippy/klippy.py:48` `get_state_message()` — `(state_message, category)` with category `ready|startup|shutdown|error`.
    fn get_state_message(&self) -> (String, String);
    /// `klippy/klippy.py:58` `is_shutdown()` — `in_shutdown_state`.
    fn is_shutdown(&self) -> bool;
    /// `klippy/klippy.py:68` `update_error_msg(oldmsg, newmsg)` — conditional state rewrite (error_mcu handlers only).
    fn update_error_msg(&self, old_msg: &str, new_msg: &str);
    /// `klippy/klippy.py:75` `add_object(name, obj)` — duplicate name raises `KlippyError::Config`.
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()>;
    /// `klippy/klippy.py:80` `lookup_object(name, default)` — missing name follows `default`.
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>>;
    /// `klippy/klippy.py:86` `lookup_objects(module=None)` — all entries, or exact match plus `module+' '` prefixed entries.
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)>;
    /// Pauses the reactor.
    /// `klippy/klippy.py:95` `load_object(config, section, default)` — runs the module factory; may block on MCU.
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>>;
    /// Pauses the reactor.
    /// `klippy/klippy.py:175` `run()` — main loop; returns `run_result` (`exit|error_exit|firmware_restart|restart`).
    fn run(&self) -> Option<String>;
    /// `klippy/klippy.py:206` `set_rollover_info(name, info, log=True)` — `info=None` clears the marker.
    fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool);
    /// `klippy/klippy.py:259` `invoke_shutdown(msg, details={})` — idempotent; fires `klippy:shutdown` then `klippy:analyze_shutdown`.
    fn invoke_shutdown(&self, msg: &str, details: PyValue);
    /// `klippy/klippy.py:276` `invoke_async_shutdown(msg, details={})` — defer via reactor async callback.
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue);
    /// `klippy/klippy.py:279` `register_event_handler(event, callback)` — append order is invocation order.
    fn register_event_handler(&self, event: &str, callback: EventCallback);
    /// `klippy/klippy.py:281` `send_event(event, *params)` — collect handler returns; handler exceptions propagate.
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>>;
    /// `klippy/klippy.py:283` `request_exit(result)` — first call wins; ends the reactor.
    fn request_exit(&self, result: &str);
}

/// Module-loading convention (`klippy/klippy.py:95-118`).
/// Single-token section dispatches to `load_config`; multi-token
/// (space-separated) section dispatches to `load_config_prefix`. The
/// returned object is stored under the full section string and may be
/// `None` (side-effect only). `add_early_printer_objects` takes the
/// printer directly before config (`klippy/klippy.py:42`); the other
/// three take the config wrapper.
#[derive(Debug, Clone, Copy, Default)]
pub struct ModuleFactory {
    /// `load_config(config) -> Any` — single-token sections (`extras/<module>.py:load_config`).
    pub load_config: Option<fn(&Config) -> KResult<Option<ObjectRef>>>,
    /// `load_config_prefix(config) -> Any` — multi-token sections (`extras/<module>.py:load_config_prefix`).
    pub load_config_prefix: Option<fn(&Config) -> KResult<Option<ObjectRef>>>,
    /// `add_early_printer_objects(printer)` — `[gcode, webhooks]` at construction (`klippy/klippy.py:42`, `klippy/gcode.py:488`, `klippy/webhooks.py:574`).
    pub add_early_printer_objects: Option<fn(&Printer) -> KResult<()>>,
    /// `add_printer_objects(config)` — `pins`, `mcu`, `toolhead` in `_read_config` (`klippy/klippy.py:125-130`).
    pub add_printer_objects: Option<fn(&Config) -> KResult<()>>,
}

// ---------------------------------------------------------------------------
// Event names. Params are documented as the handler signature; the sender
// order is noted where it differs from the handler declaration.
// ---------------------------------------------------------------------------

/// `klippy:connect` — params: none (`cb()`). Sender `klippy/klippy.py:137`; handlers e.g. `klippy/mcu.py:1145`.
pub const EV_KLIPPY_CONNECT: &str = "klippy:connect";
/// `klippy:ready` — params: none (`cb()`). Sender `klippy/klippy.py:165`.
pub const EV_KLIPPY_READY: &str = "klippy:ready";
/// `klippy:shutdown` — params: none (`cb()`). Sender `klippy/klippy.py:266`.
pub const EV_KLIPPY_SHUTDOWN: &str = "klippy:shutdown";
/// `klippy:analyze_shutdown` — params: `(msg: str, details: dict)`. Sender `klippy/klippy.py:270`.
pub const EV_KLIPPY_ANALYZE_SHUTDOWN: &str = "klippy:analyze_shutdown";
/// `klippy:disconnect` — params: none (`cb()`). Sender `klippy/klippy.py:201`.
pub const EV_KLIPPY_DISCONNECT: &str = "klippy:disconnect";
/// `klippy:mcu_identify` — params: none (`cb()`). Sender `klippy/klippy.py:136`.
pub const EV_KLIPPY_MCU_IDENTIFY: &str = "klippy:mcu_identify";
/// `klippy:notify_mcu_error` — params: `(msg: str, details: dict)` with `details["error"]: str`. Sender `klippy/klippy.py:149`.
pub const EV_KLIPPY_NOTIFY_MCU_ERROR: &str = "klippy:notify_mcu_error";
/// `klippy:firmware_restart` — params: none (`cb()`). Sender `klippy/klippy.py:200`.
pub const EV_KLIPPY_FIRMWARE_RESTART: &str = "klippy:firmware_restart";
/// `gcode:command_error` — params: none (`cb()`). Sender `klippy/gcode.py:226`.
pub const EV_GCODE_COMMAND_ERROR: &str = "gcode:command_error";
/// `gcode:request_restart` — params: `(print_time: float)`. Sender `klippy/gcode.py:358`.
pub const EV_GCODE_REQUEST_RESTART: &str = "gcode:request_restart";
/// `toolhead:sync_print_time` — params: `(curtime: float, est_print_time: float, print_time: float)` sender order at `klippy/toolhead.py:267`; the handler declares `(curtime, print_time, est_print_time)` — keep both orders.
pub const EV_TOOLHEAD_SYNC_PRINT_TIME: &str = "toolhead:sync_print_time";
/// `toolhead:set_position` — params: none (`cb()`). Sender `klippy/toolhead.py:390`.
pub const EV_TOOLHEAD_SET_POSITION: &str = "toolhead:set_position";
/// `toolhead:manual_move` — params: none (`cb()`). Sender `klippy/toolhead.py:416`.
pub const EV_TOOLHEAD_MANUAL_MOVE: &str = "toolhead:manual_move";
/// `toolhead:update_extra_axes` — params: none (`cb()`). Sender `klippy/toolhead.py:446`.
pub const EV_TOOLHEAD_UPDATE_EXTRA_AXES: &str = "toolhead:update_extra_axes";
/// `stepper:sync_mcu_position` — params: `(stepper: McuStepper)` (as `EventArg`). Sender `klippy/stepper.py:227`.
pub const EV_STEPPER_SYNC_MCU_POSITION: &str = "stepper:sync_mcu_position";
/// `stepper:set_dir_inverted` — params: `(stepper: McuStepper)` (as `EventArg`). Sender `klippy/stepper.py:153`.
pub const EV_STEPPER_SET_DIR_INVERTED: &str = "stepper:set_dir_inverted";
/// `homing:homing_move_begin` — params: `(move: HomingMove)` (as `EventArg`). Sender `klippy/homing.py:80`.
pub const EV_HOMING_MOVE_BEGIN: &str = "homing:homing_move_begin";
/// `homing:homing_move_end` — params: `(move: HomingMove)`; handlers may raise `command_error`. Sender `klippy/homing.py:148`.
pub const EV_HOMING_MOVE_END: &str = "homing:homing_move_end";
/// `homing:home_rails_begin` — params: `(homing: Homing, rails: list)` (as `EventArg`). Sender `klippy/homing.py:210`.
pub const EV_HOMING_HOME_RAILS_BEGIN: &str = "homing:home_rails_begin";
/// `homing:home_rails_end` — params: `(homing: Homing, rails: list)` (as `EventArg`). Sender `klippy/homing.py:234`.
pub const EV_HOMING_HOME_RAILS_END: &str = "homing:home_rails_end";
/// `idle_timeout:idle` — params: `(print_time: float)`. Sender `klippy/extras/idle_timeout.py:57`.
pub const EV_IDLE_TIMEOUT_IDLE: &str = "idle_timeout:idle";
/// `idle_timeout:ready` — params: `(time: float)`. Sender `klippy/extras/idle_timeout.py:95`.
pub const EV_IDLE_TIMEOUT_READY: &str = "idle_timeout:ready";
/// `idle_timeout:printing` — params: `(time: float)`. Sender `klippy/extras/idle_timeout.py:106`.
pub const EV_IDLE_TIMEOUT_PRINTING: &str = "idle_timeout:printing";
/// `probe:update_results` — params: `(results: list)` (as `EventArg::List`, mutated in place). Sender `klippy/probe.py:366`.
pub const EV_PROBE_UPDATE_RESULTS: &str = "probe:update_results";
/// `load_cell:calibrate` — params: `(sensor: LoadCell)` (as `EventArg`). Sender `klippy/load_cell.py:404`.
pub const EV_LOAD_CELL_CALIBRATE: &str = "load_cell:calibrate";
/// `load_cell:tare` — params: `(sensor: LoadCell)` (as `EventArg`). Sender `klippy/load_cell.py:406`.
pub const EV_LOAD_CELL_TARE: &str = "load_cell:tare";
/// `stepper_enable:motor_off` — params: none (`cb()`). Sender `klippy/stepper_enable.py:120`.
pub const EV_STEPPER_ENABLE_MOTOR_OFF: &str = "stepper_enable:motor_off";
/// `virtual_sdcard:reset_file` — params: none (`cb()`). Sender `klippy/virtual_sdcard.py:291`.
pub const EV_VIRTUAL_SDCARD_RESET_FILE: &str = "virtual_sdcard:reset_file";
/// `extruder:activate_extruder` — params: none (`cb()`). Sender `klippy/kinematics/extruder.py:289`.
pub const EV_EXTRUDER_ACTIVATE_EXTRUDER: &str = "extruder:activate_extruder";
/// `dual_carriage:update_kinematics` — params: none (`cb()`). Sender `klippy/kinematics/idex_modes.py:383`.
pub const EV_DUAL_CARRIAGE_UPDATE_KINEMATICS: &str = "dual_carriage:update_kinematics";
/// `menu:init` — params: `(manager)` (as `EventArg`). Sender `klippy/extras/display/menu.py:722`.
pub const EV_MENU_INIT: &str = "menu:init";
/// `menu:begin` — params: `(manager)` (as `EventArg`). Sender `klippy/extras/display/menu.py:754`.
pub const EV_MENU_BEGIN: &str = "menu:begin";
/// `menu:exit` — params: `(manager)` (as `EventArg`). Sender `klippy/extras/display/menu.py:913`.
pub const EV_MENU_EXIT: &str = "menu:exit";
/// `menu:populate` — params: `(container)` (as `EventArg`). Sender `klippy/extras/display/menu.py:346`.
pub const EV_MENU_POPULATE: &str = "menu:populate";
/// Prefix for dynamic `menu:<ns>:<event>` names (`klippy/extras/display/menu.py:180`); no const is possible per namespace.
pub const MENU_EVENT_PREFIX: &str = "menu:";
