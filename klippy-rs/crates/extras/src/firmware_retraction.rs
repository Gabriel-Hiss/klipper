//! Port of `klippy/extras/firmware_retraction.py`.
//!
//! Marlin/Smoothie/Reprap style firmware retraction via `G10`/`G11` plus
//! `SET_RETRACTION` / `GET_RETRACTION`. The port reproduces every branch,
//! message text, float operation and `%` format of the Python module.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCode, GCodeApi, GetFloatOpts as GGetFloatOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::object::{lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use pycompat::text::{percent_format, PctValue, PyError};
use pycompat::PyValue;
use std::sync::Arc;

/// `SET_RETRACTION` help line (`klippy/extras/firmware_retraction.py:33`).
pub const SET_RETRACTION_HELP: &str = "Set firmware retraction parameters";
/// `GET_RETRACTION` help line (`klippy/extras/firmware_retraction.py:46`).
pub const GET_RETRACTION_HELP: &str = "Report firmware retraction parameters";

/// Format string for the `G10` script (`klippy/extras/firmware_retraction.py:55`).
const G10_FORMAT: &str = "SAVE_GCODE_STATE NAME=_retract_state\n\
                           G91\n\
                           G1 E-%.5f F%d\n\
                           RESTORE_GCODE_STATE NAME=_retract_state";
/// Format string for the `G11` script (`klippy/extras/firmware_retraction.py:65`).
const G11_FORMAT: &str = "SAVE_GCODE_STATE NAME=_retract_state\n\
                           G91\n\
                           G1 E%.5f F%d\n\
                           RESTORE_GCODE_STATE NAME=_retract_state";
/// Format string for `GET_RETRACTION`
/// (`klippy/extras/firmware_retraction.py:48`).
const GET_RETRACTION_FORMAT: &str =
    "RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f";

/// %-format helper: formatting failures are uncaught Python errors.
fn percent(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e: PyError| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

struct State {
    retract_length: f64,
    retract_speed: f64,
    unretract_extra_length: f64,
    unretract_speed: f64,
    unretract_length: f64,
    is_retracted: bool,
}

/// `FirmwareRetraction` (`klippy/extras/firmware_retraction.py:7`).
///
/// `self.printer` in Python is never read after `__init__` (only `gcode`
/// is used), so the port keeps the `GCode` handle and drops the printer.
pub struct FirmwareRetraction {
    gcode: GCode,
    state: GCell<State>,
}

/// Resolve the `gcode` object (`klippy/extras/firmware_retraction.py:18`).
///
/// Python uses `printer.lookup_object('gcode')` with the sentinel default,
/// which raises `config_error` (`"Unknown config object 'gcode'"`) when
/// the object is missing; the `Raise` default reproduces that.
fn lookup_gcode(printer: &Printer) -> KResult<GCode> {
    let gcode = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?
        .ok_or_else(|| KlippyError::config("Unknown config object 'gcode'"))?;
    Ok(gcode)
}

impl FirmwareRetraction {
    /// `FirmwareRetraction.__init__` (`klippy/extras/firmware_retraction.py:8`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let retract_length = config
            .getfloat(
                "retract_length",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);
        let retract_speed = config
            .getfloat(
                "retract_speed",
                GetFloatOpts {
                    default: Some(Some(20.0)),
                    minval: Some(1.0),
                    ints: klippy_api::config::IntArgs::MINVAL,
                    ..Default::default()
                },
            )?
            .unwrap_or(20.0);
        let unretract_extra_length = config
            .getfloat(
                "unretract_extra_length",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);
        let unretract_speed = config
            .getfloat(
                "unretract_speed",
                GetFloatOpts {
                    default: Some(Some(10.0)),
                    minval: Some(1.0),
                    ints: klippy_api::config::IntArgs::MINVAL,
                    ..Default::default()
                },
            )?
            .unwrap_or(10.0);
        let gcode = lookup_gcode(&printer)?;
        let obj = Arc::new(Self {
            gcode,
            state: GCell::new(State {
                retract_length,
                retract_speed,
                unretract_extra_length,
                unretract_speed,
                unretract_length: retract_length + unretract_extra_length,
                is_retracted: false,
            }),
        });
        {
            let me = Arc::clone(&obj);
            obj.gcode.register_command(
                "SET_RETRACTION",
                Some(Box::new(move |gcmd| me.cmd_set_retraction(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(SET_RETRACTION_HELP.to_string()),
                },
            )?;
        }
        {
            let me = Arc::clone(&obj);
            obj.gcode.register_command(
                "GET_RETRACTION",
                Some(Box::new(move |gcmd| me.cmd_get_retraction(gcmd))),
                RegisterCommandOpts {
                    when_not_ready: false,
                    desc: Some(GET_RETRACTION_HELP.to_string()),
                },
            )?;
        }
        {
            let me = Arc::clone(&obj);
            obj.gcode.register_command(
                "G10",
                Some(Box::new(move |gcmd| me.cmd_g10(gcmd))),
                RegisterCommandOpts::default(),
            )?;
        }
        {
            let me = Arc::clone(&obj);
            obj.gcode.register_command(
                "G11",
                Some(Box::new(move |gcmd| me.cmd_g11(gcmd))),
                RegisterCommandOpts::default(),
            )?;
        }
        Ok(obj)
    }

    /// `cmd_SET_RETRACTION` (`klippy/extras/firmware_retraction.py:34`).
    ///
    /// Python assigns each parameter in order, so a failure on a later
    /// parameter leaves earlier assignments in place; the recomputation
    /// of `unretract_length` and the reset of `is_retracted` only happen
    /// afterwards.
    pub fn cmd_set_retraction(&self, gcmd: &GCmd) -> KResult<()> {
        let cur = self.state.borrow().retract_length;
        let value = gcmd
            .get_float(
                "RETRACT_LENGTH",
                GGetFloatOpts {
                    default: Some(Some(cur)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(cur);
        self.state.borrow_mut().retract_length = value;
        let cur = self.state.borrow().retract_speed;
        let value = gcmd
            .get_float(
                "RETRACT_SPEED",
                GGetFloatOpts {
                    default: Some(Some(cur)),
                    minval: Some(1.0),
                    ints: klippy_api::config::IntArgs::MINVAL,
                    ..Default::default()
                },
            )?
            .unwrap_or(cur);
        self.state.borrow_mut().retract_speed = value;
        let cur = self.state.borrow().unretract_extra_length;
        let value = gcmd
            .get_float(
                "UNRETRACT_EXTRA_LENGTH",
                GGetFloatOpts {
                    default: Some(Some(cur)),
                    minval: Some(0.0),
                    ..Default::default()
                },
            )?
            .unwrap_or(cur);
        self.state.borrow_mut().unretract_extra_length = value;
        let cur = self.state.borrow().unretract_speed;
        let value = gcmd
            .get_float(
                "UNRETRACT_SPEED",
                GGetFloatOpts {
                    default: Some(Some(cur)),
                    minval: Some(1.0),
                    ints: klippy_api::config::IntArgs::MINVAL,
                    ..Default::default()
                },
            )?
            .unwrap_or(cur);
        self.state.borrow_mut().unretract_speed = value;
        self.state.with_mut(|s| {
            s.unretract_length = s.retract_length + s.unretract_extra_length;
            s.is_retracted = false;
        });
        Ok(())
    }

    /// `cmd_GET_RETRACTION` (`klippy/extras/firmware_retraction.py:47`).
    pub fn cmd_get_retraction(&self, gcmd: &GCmd) -> KResult<()> {
        let (retract_length, retract_speed, unretract_extra_length, unretract_speed) = {
            let s = self.state.borrow();
            (
                s.retract_length,
                s.retract_speed,
                s.unretract_extra_length,
                s.unretract_speed,
            )
        };
        let msg = percent(
            GET_RETRACTION_FORMAT,
            &[
                PctValue::Float(retract_length),
                PctValue::Float(retract_speed),
                PctValue::Float(unretract_extra_length),
                PctValue::Float(unretract_speed),
            ],
        )?;
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    /// `cmd_G10` (`klippy/extras/firmware_retraction.py:53`).
    ///
    /// The borrow is dropped before `run_script_from_command` (which
    /// pauses the reactor); the flag is set only when the script succeeds.
    pub fn cmd_g10(&self, _gcmd: &GCmd) -> KResult<()> {
        let (is_retracted, retract_length, retract_speed) = {
            let s = self.state.borrow();
            (s.is_retracted, s.retract_length, s.retract_speed)
        };
        if !is_retracted {
            let script = percent(
                G10_FORMAT,
                &[
                    PctValue::Float(retract_length),
                    PctValue::Float(retract_speed * 60.0),
                ],
            )?;
            self.gcode.run_script_from_command(&script)?;
            self.state.borrow_mut().is_retracted = true;
        }
        Ok(())
    }

    /// `cmd_G11` (`klippy/extras/firmware_retraction.py:63`).
    pub fn cmd_g11(&self, _gcmd: &GCmd) -> KResult<()> {
        let (is_retracted, unretract_length, unretract_speed) = {
            let s = self.state.borrow();
            (s.is_retracted, s.unretract_length, s.unretract_speed)
        };
        if is_retracted {
            let script = percent(
                G11_FORMAT,
                &[
                    PctValue::Float(unretract_length),
                    PctValue::Float(unretract_speed * 60.0),
                ],
            )?;
            self.gcode.run_script_from_command(&script)?;
            self.state.borrow_mut().is_retracted = false;
        }
        Ok(())
    }
}

impl PrinterObject for FirmwareRetraction {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }

    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }

    /// `get_status` (`klippy/extras/firmware_retraction.py:26`).
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let s = self.state.borrow();
        let mut map = IndexMap::new();
        map.insert(
            PyValue::Str("retract_length".to_string()),
            PyValue::Float(s.retract_length),
        );
        map.insert(
            PyValue::Str("retract_speed".to_string()),
            PyValue::Float(s.retract_speed),
        );
        map.insert(
            PyValue::Str("unretract_extra_length".to_string()),
            PyValue::Float(s.unretract_extra_length),
        );
        map.insert(
            PyValue::Str("unretract_speed".to_string()),
            PyValue::Float(s.unretract_speed),
        );
        Ok(Some(PyValue::Dict(map)))
    }
}

/// `load_config` (`klippy/extras/firmware_retraction.py:73`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(FirmwareRetraction::new(config)? as ObjectRef))
}

/// Module factory (only `load_config` exists in Python).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        ..Default::default()
    }
}
