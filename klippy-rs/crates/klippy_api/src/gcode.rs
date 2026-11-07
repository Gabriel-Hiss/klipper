//! Contract of the `gcode`, `gcode_command` and `gcode_move` services.
//!
//! Covers `GCodeDispatch` (`klippy/gcode.py:99`, object `'gcode'`),
//! `GCodeCommand` (`klippy/gcode.py:24`, the `gcmd` passed to every
//! registered handler) and `GCodeMove` (`klippy/extras/gcode_move.py:8`,
//! object `'gcode_move'`). The ported `core::gcode` module implements
//! [`GCodeApi`], the ported `core::gcode_parse` module implements
//! [`GCodeCommandApi`] and the ported `core::gcode_move` module
//! implements [`GCodeMoveApi`].
//!
//! Source inventory: `local://contract/gcode.md`.
//! All command and parameter names below are the literal Python strings
//! so handlers and fakes agree byte for byte.
//!
//! Mapping notes (faithfulness trade-offs, see report):
//! - `gcode.error` / `gcmd.error` (`gcode.py:25,100`) are the
//!   `CommandError` class itself (`raise gcmd.error(msg)` and
//!   `except gcmd.error`); in Rust they are [`KlippyError::Command`]
//!   and need no method, like `printer.config_error` /
//!   `printer.command_error` (`printer.rs`).
//! - `gcode.Coord`, `gcode_move.Coord` and `toolhead.Coord` are one
//!   Python class (`gcode.py:12`); the canonical Rust type is
//!   [`Coord`] (`crate::toolhead::Coord`), re-exported here so
//!   `crate::gcode::Coord` and `crate::toolhead::Coord` are the same
//!   type.
//! - `gcmd.get`'s `parser` callable is spelled
//!   [`GetParser`] (the tree only ever passes `str`, `int`,
//!   `float` and `lambda x: int(x, 0)`); `get` returns
//!   [`PyValue`] so every parser shares one return type, while
//!   `get_int` / `get_float` stay typed like the other getters
//!   (`config.rs`).
//! - `register_command`'s `func=None` (unregister) returns the
//!   removed handler (`homing_override.py:18`, `gcode_macro.py:164`);
//!   the `'dummy'` placeholder strings (`load_cell.py:166`,
//!   `manual_probe.py:154`) are a no-op handler in Rust.
//! - `set_move_transform` accepts any object with
//!   `move(newpos, speed)` and `get_position()`
//!   (`gcode_move.py:91-92`): the `ToolHead` itself or a transform
//!   wrapper (`bed_mesh`, `bed_tilt`, `exclude_object`,
//!   `skew_correction`, `tuning_tower`, `z_thermal_adjust`); see
//!   [`MoveTransform`] / [`MoveTransformApi`].

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::PyValue;
use reactor::ReactorMutex;

use crate::error::KResult;
use crate::toolhead::ToolHead;

pub use crate::toolhead::Coord;

/// `GCodeDispatch` handle (`Arc<dyn GCodeApi>`).
pub type GCode = Arc<dyn GCodeApi>;

/// `GCodeCommand` handle (`Arc<dyn GCodeCommandApi>`); the `gcmd`
/// passed to every handler.
pub type GCmd = Arc<dyn GCodeCommandApi>;

/// `GCodeMove` handle (`Arc<dyn GCodeMoveApi>`).
pub type GCodeMove = Arc<dyn GCodeMoveApi>;

/// Handler registered for one command: `def cmd_X(self, gcmd)`
/// (`gcode.py:133`); stored and called by the dispatcher.
pub type GCodeCommandHandler = Box<dyn FnMut(&GCmd) -> KResult<()> + Send>;

/// Output callback `cb(msg: str) -> None` (`gcode.py:184`); stored
/// and called many times by `respond_raw`.
pub type OutputCallback = Box<dyn FnMut(String) + Send>;

/// `gcmd.get`'s `parser` (`gcode.py:65`): the only callables the
/// tree ever passes are `str`, `int`, `float` and
/// `lambda x: int(x, 0)` (`adxl345.py:181`, `angle.py:404`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum GetParser {
    /// `str` (the Python default).
    #[default]
    Str,
    /// `int` (base 10, as `get_int` uses).
    Int,
    /// `lambda x: int(x, 0)` (auto base: `0x` hex, `0o` octal).
    IntAutoBase,
    /// `float` (as `get_float` uses).
    Float,
}

/// Options for [`GCodeCommandApi::get`] (`klippy/gcode.py:65`).
#[derive(Clone, Debug)]
pub struct GetOpts {
    /// `default` (`None` = required/sentinel raises; `Some(PyValue::None)`
    /// returns `None` when absent, see module docs).
    pub default: Option<PyValue>,
    /// `parser`.
    pub parser: GetParser,
    /// `minval` (compared after parsing).
    pub minval: Option<PyValue>,
    /// `maxval` (compared after parsing).
    pub maxval: Option<PyValue>,
    /// `above` (exclusive lower bound, `v <= above` raises).
    pub above: Option<PyValue>,
    /// `below` (exclusive upper bound, `v >= below` raises).
    pub below: Option<PyValue>,
}

impl Default for GetOpts {
    fn default() -> Self {
        GetOpts {
            default: None,
            parser: GetParser::Str,
            minval: None,
            maxval: None,
            above: None,
            below: None,
        }
    }
}

/// Options for [`GCodeCommandApi::get_int`] (`klippy/gcode.py:91`).
#[derive(Clone, Copy, Debug, Default)]
pub struct GetIntOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<i64>>,
    /// `minval`.
    pub minval: Option<i64>,
    /// `maxval`.
    pub maxval: Option<i64>,
}

/// Options for [`GCodeCommandApi::get_float`] (`klippy/gcode.py:93`).
#[derive(Clone, Copy, Debug, Default)]
pub struct GetFloatOpts {
    /// `default` (`None` = required, see module docs).
    pub default: Option<Option<f64>>,
    /// `minval`.
    pub minval: Option<f64>,
    /// `maxval`.
    pub maxval: Option<f64>,
    /// `above` (exclusive lower bound, `v <= above` raises).
    pub above: Option<f64>,
    /// `below` (exclusive upper bound, `v >= below` raises).
    pub below: Option<f64>,
}

/// Options for [`GCodeApi::register_command`] (`klippy/gcode.py:133`).
#[derive(Clone, Debug, Default)]
pub struct RegisterCommandOpts {
    /// `when_not_ready` (register also before `klippy:ready`).
    pub when_not_ready: bool,
    /// `desc` (help string; `None` stores no help entry).
    pub desc: Option<String>,
}

/// A move transform for [`GCodeMoveApi::set_move_transform`]
/// (`klippy/extras/gcode_move.py:83`): the `ToolHead` itself or
/// a duck-typed wrapper with `move(newpos, speed)` /
/// `get_position()` (`bed_mesh.BedMesh`, `bed_tilt`,
/// `exclude_object`, `skew_correction`, `tuning_tower`,
/// `z_thermal_adjust`).
#[derive(Clone)]
pub enum MoveTransform {
    /// The `ToolHead` object (`toolhead.move` / `toolhead.get_position`).
    ToolHead(ToolHead),
    /// A transform wrapper (`transform.move` / `transform.get_position`).
    Custom(Arc<dyn MoveTransformApi>),
}

/// A move-transform wrapper (`gcode_move.py:91-92` `transform.move`
/// / `transform.get_position`).
pub trait MoveTransformApi: Send + Sync + 'static {
    /// `ToolHead.move` / `transform.move` (`klippy/toolhead.py:397`).
    /// Pauses the reactor.
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()>;

    /// `ToolHead.get_position` / `transform.get_position`
    /// (`klippy/toolhead.py:383`).
    fn get_position(&self) -> Vec<f64>;
}

/// A move-transform wrapper handle.
pub type MoveTransformHandle = Arc<dyn MoveTransformApi>;

/// The `GCodeDispatch` service (`klippy/gcode.py:99`).
pub trait GCodeApi: Send + Sync + 'static {
    /// `klippy/gcode.py:133` `register_command` — `func=None` unregisters and returns the removed handler.
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>>;

    /// `klippy/gcode.py:158` `register_mux_command`
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()>;

    /// `klippy/gcode.py:174` `get_command_help`
    fn get_command_help(&self) -> IndexMap<String, String>;

    /// `klippy/gcode.py:176` `get_status`
    fn get_status(&self, eventtime: f64) -> PyValue;

    /// `klippy/gcode.py:184` `register_output_handler`
    fn register_output_handler(&self, callback: OutputCallback);

    /// `klippy/gcode.py:352` `request_restart`
    /// Pauses the reactor.
    fn request_restart(&self, result: &str) -> KResult<()>;

    /// `klippy/gcode.py:239` `run_script`
    /// Pauses the reactor.
    fn run_script(&self, script: &str) -> KResult<()>;

    /// `klippy/gcode.py:237` `run_script_from_command`
    /// Pauses the reactor.
    fn run_script_from_command(&self, script: &str) -> KResult<()>;

    /// `klippy/gcode.py:242` `get_mutex`
    fn get_mutex(&self) -> ReactorMutex;

    /// `klippy/gcode.py:244` `create_gcode_command`
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd;

    /// `klippy/gcode.py:247` `respond_raw`
    fn respond_raw(&self, msg: &str);

    /// `klippy/gcode.py:250` `respond_info`
    fn respond_info(&self, msg: &str, log: bool);

    /// `klippy/gcode.py:125` `is_traditional_gcode`
    fn is_traditional_gcode(&self, cmd: &str) -> bool;

    /// `klippy/gcode.py:341` `cmd_M112` — invoked with no `gcmd` (`None`) from `GCodeIO`
    fn cmd_m112(&self, gcmd: Option<&GCmd>) -> KResult<()>;

    /// `klippy/gcode.py:202` `_process_commands` — invoked by `GCodeIO` with `need_ack=True`
    /// Pauses the reactor.
    fn process_commands(&self, commands: &[String], need_ack: bool) -> KResult<()>;
}

/// The `GCodeCommand` service (`klippy/gcode.py:24`): the `gcmd`
/// handed to every handler.
pub trait GCodeCommandApi: Send + Sync + 'static {
    /// `klippy/gcode.py:65` `get`
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue>;

    /// `klippy/gcode.py:91` `get_int`
    fn get_int(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>>;

    /// `klippy/gcode.py:93` `get_float`
    fn get_float(&self, name: &str, opts: GetFloatOpts) -> KResult<Option<f64>>;

    /// `klippy/gcode.py:34` `get_command`
    fn get_command(&self) -> String;

    /// `klippy/gcode.py:36` `get_commandline`
    fn get_commandline(&self) -> String;

    /// `klippy/gcode.py:38` `get_command_parameters`
    fn get_command_parameters(&self) -> IndexMap<String, String>;

    /// `klippy/gcode.py:40` `get_raw_command_parameters`
    fn get_raw_command_parameters(&self) -> String;

    /// `klippy/gcode.py:38` `respond_info` (bound `gcode.respond_info`)
    fn respond_info(&self, msg: &str, log: bool);

    /// `klippy/gcode.py:37` `respond_raw` (bound `gcode.respond_raw`)
    fn respond_raw(&self, msg: &str);

    /// `klippy/gcode.py:54` `ack`
    fn ack(&self, msg: Option<&str>) -> bool;
}

/// The `GCodeMove` service (`klippy/extras/gcode_move.py:8`).
pub trait GCodeMoveApi: Send + Sync + 'static {
    /// `klippy/extras/gcode_move.py:102` `get_status`
    fn get_status(&self, eventtime: Option<f64>) -> PyValue;

    /// `klippy/extras/gcode_move.py:83` `set_move_transform`
    fn set_move_transform(&self, transform: MoveTransform, force: bool) -> KResult<MoveTransform>;

    /// `klippy/extras/gcode_move.py:115` `reset_last_position`
    fn reset_last_position(&self);

    /// `klippy/extras/gcode_move.py:134` `cmd_G1` — also registered as `G0`; invoked directly by `gcode_arcs.py:179`
    /// Pauses the reactor.
    fn cmd_g1(&self, gcmd: &GCmd) -> KResult<()>;
}
