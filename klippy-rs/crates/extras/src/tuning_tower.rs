//! Port of the `tuning_tower` module (`klippy/extras/tuning_tower.py`).
//!
//! `TuningTower` is a move transform that re-emits a G-code command
//! (`COMMAND PARAMETER=<value>`, or without `=` for traditional G-code
//! names) whenever an extruding move reaches a new Z height. The emitted
//! value follows `calc_value`: a linear `START + Z * FACTOR` (optionally
//! quantized to `BAND`), or `START + STEP_DELTA * floor(Z / STEP_HEIGHT)`
//! for the step mode, shifted by `SKIP`. Starting twice ends the previous
//! test first; an extrusion move more than [`CANCEL_Z_DELTA`] below the
//! last handled Z ends the test.

use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCode, GCodeCommandHandler, GCodeMove, GetFloatOpts, GetOpts, GetParser,
    MoveTransform, MoveTransformApi, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory};
use klippy_api::{load_interface, lookup_interface, GCell, ObjectRef};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// `CANCEL_Z_DELTA` (`klippy/extras/tuning_tower.py:8`).
pub const CANCEL_Z_DELTA: f64 = 2.0;

/// Help text for `TUNING_TOWER` (`klippy/extras/tuning_tower.py:23`).
pub const CMD_TUNING_TOWER_HELP: &str = "Tool to adjust a parameter at each Z height";

/// G-code command registered by the module (`klippy/extras/tuning_tower.py:21`).
pub const TUNING_TOWER_CMD: &str = "TUNING_TOWER";

#[derive(Clone)]
struct State {
    normal_transform: Option<MoveTransform>,
    last_position: Vec<f64>,
    last_z: f64,
    start: f64,
    factor: f64,
    band: f64,
    // Python creates these attributes only inside `cmd_TUNING_TOWER`
    // (`tuning_tower.py:33-35`); `__init__` never sets them, so reading
    // them before the first command raises `AttributeError`.
    step_delta: Option<f64>,
    step_height: Option<f64>,
    skip: Option<f64>,
    last_command_value: Option<f64>,
    command_fmt: String,
}

impl Default for State {
    fn default() -> Self {
        State {
            normal_transform: None,
            last_position: vec![0.0, 0.0, 0.0, 0.0],
            last_z: 0.0,
            start: 0.0,
            factor: 0.0,
            band: 0.0,
            step_delta: None,
            step_height: None,
            skip: None,
            last_command_value: None,
            command_fmt: String::new(),
        }
    }
}

/// `klippy/extras/tuning_tower.py:TuningTower`.
pub struct TuningTower {
    gcode: GCode,
    gcode_move: GCodeMove,
    state: GCell<State>,
}

fn missing_attr(name: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'TuningTower' object has no attribute '{name}'"),
    }
}

fn none_move_attr() -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: "'NoneType' object has no attribute 'move'".to_string(),
    }
}

fn none_get_position_attr() -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: "'NoneType' object has no attribute 'get_position'".to_string(),
    }
}

fn list_index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}


fn fmt_percent(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(|e| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

/// `gcmd.get(name)` with no default (`klippy/gcode.py:65`).
fn required_str(gcmd: &GCmd, name: &str) -> KResult<String> {
    let value = gcmd.get(
        name,
        GetOpts {
            default: None,
            parser: GetParser::Str,
            minval: None,
            maxval: None,
            above: None,
            below: None,
        },
    )?;
    match value {
        PyValue::Str(s) => Ok(s),
        PyValue::None => Err(KlippyError::command(format!(
            "Error on '{}': missing {}",
            gcmd.get_commandline(),
            name
        ))),
        other => Ok(other.str()),
    }
}

/// `gcmd.get_float(name, default, minval)` as used by `cmd_TUNING_TOWER`.
fn float_param(
    gcmd: &GCmd,
    name: &str,
    default: f64,
    minval: Option<f64>,
) -> KResult<f64> {
    let value = gcmd.get_float(
        name,
        GetFloatOpts {
            default: Some(Some(default)),
            minval,
            maxval: None,
            above: None,
            below: None,
        },
    )?;
    Ok(value.unwrap_or(default))
}

/// `gcode_move.get_status()['gcode_position'].z`.
fn extract_gcode_z(status: &PyValue) -> KResult<f64> {
    let PyValue::Dict(entries) = status else {
        return Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!(
                "'{}' object is not subscriptable",
                pycompat::value::type_name(status)
            ),
        });
    };
    let key = PyValue::Str("gcode_position".to_string());
    let Some(value) = entries.get(&key) else {
        return Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'gcode_position'".to_string(),
        });
    };
    let items = match value {
        PyValue::Tuple(items) => items,
        PyValue::List(items) => items,
        other => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!(
                    "'{}' object is not subscriptable",
                    pycompat::value::type_name(other)
                ),
            });
        }
    };
    // `gcode.Coord` pads missing entries with the int `0`
    // (`klippy/gcode.py:14-17`), so `.z` of a short position is `0`.
    let Some(z) = items.get(2) else {
        return Ok(0.0);
    };
    z.to_f64().ok_or_else(|| KlippyError::Internal {
        class_name: "TypeError".to_string(),
        message: format!(
            "must be real number, not {}",
            pycompat::value::type_name(z)
        ),
    })
}

impl TuningTower {
    /// `TuningTower.__init__(config)` (`klippy/extras/tuning_tower.py:11`):
    /// loads `gcode_move`, looks up `gcode` and registers `TUNING_TOWER`.
    pub fn new(config: &Config) -> KResult<Arc<TuningTower>> {
        let printer = config.get_printer();
        let gcode_move =
            load_interface::<dyn klippy_api::gcode::GCodeMoveApi>(
                &*printer,
                config,
                "gcode_move",
                LookupDefault::Raise,
            )?
            .expect("LookupDefault::Raise never yields None");
        let gcode = lookup_interface::<dyn klippy_api::gcode::GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let tower = Arc::new(TuningTower {
            gcode: gcode.clone(),
            gcode_move,
            state: GCell::new(State::default()),
        });
        let handler = Arc::clone(&tower);
        let func: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| handler.cmd_tuning_tower(gcmd));
        gcode.register_command(
            TUNING_TOWER_CMD,
            Some(func),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_TUNING_TOWER_HELP.to_string()),
            },
        )?;
        Ok(tower)
    }

    /// `TuningTower.cmd_TUNING_TOWER` (`klippy/extras/tuning_tower.py:24`).
    ///
    /// The receiver is `&Arc<Self>` because installing the transform
    /// needs a `MoveTransform::Custom` handle to this object.
    pub fn cmd_tuning_tower(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        if self.is_active() {
            self.end_test()?;
        }
        let command = required_str(gcmd, "COMMAND")?;
        let parameter = required_str(gcmd, "PARAMETER")?;
        // Python assigns each attribute right after parsing it
        // (`tuning_tower.py:30-35`), so a parse failure partway leaves
        // earlier attributes updated and later ones untouched.
        let start = float_param(gcmd, "START", 0.0, None)?;
        self.state.borrow_mut().start = start;
        let factor = float_param(gcmd, "FACTOR", 0.0, None)?;
        self.state.borrow_mut().factor = factor;
        let band = float_param(gcmd, "BAND", 0.0, Some(0.0))?;
        self.state.borrow_mut().band = band;
        let step_delta = float_param(gcmd, "STEP_DELTA", 0.0, None)?;
        self.state.borrow_mut().step_delta = Some(step_delta);
        let step_height = float_param(gcmd, "STEP_HEIGHT", 0.0, Some(0.0))?;
        self.state.borrow_mut().step_height = Some(step_height);
        let skip = float_param(gcmd, "SKIP", 0.0, Some(0.0))?;
        self.state.borrow_mut().skip = Some(skip);
        if factor != 0.0 && (step_height != 0.0 || step_delta != 0.0) {
            return Err(KlippyError::command(
                "Cannot specify both FACTOR and STEP_DELTA/STEP_HEIGHT",
            ));
        }
        if (step_delta != 0.0) != (step_height != 0.0) {
            return Err(KlippyError::command(
                "Must specify both STEP_DELTA and STEP_HEIGHT",
            ));
        }
        let command_fmt = if self.gcode.is_traditional_gcode(&command) {
            format!("{command} {parameter}%.9f")
        } else {
            format!("{command} {parameter}=%.9f")
        };
        self.state.borrow_mut().command_fmt = command_fmt;
        let this: Arc<TuningTower> = Arc::clone(self);
        let handle: klippy_api::gcode::MoveTransformHandle = this;
        let prev = self
            .gcode_move
            .set_move_transform(MoveTransform::Custom(handle), true)?;
        {
            let mut state = self.state.borrow_mut();
            state.normal_transform = prev;
            state.last_z = -99999999.9;
            state.last_command_value = None;
        }
        self.get_position()?;
        let mut parts: Vec<String> = Vec::new();
        parts.push(fmt_percent(
            "start=%.6f",
            &[PctValue::Float(start)],
        )?);
        if factor != 0.0 {
            parts.push(fmt_percent(
                "factor=%.6f",
                &[PctValue::Float(factor)],
            )?);
            if band != 0.0 {
                parts.push(fmt_percent("band=%.6f", &[PctValue::Float(band)])?);
            }
        } else {
            parts.push(fmt_percent(
                "step_delta=%.6f",
                &[PctValue::Float(step_delta)],
            )?);
            parts.push(fmt_percent(
                "step_height=%.6f",
                &[PctValue::Float(step_height)],
            )?);
        }
        if skip != 0.0 {
            parts.push(fmt_percent("skip=%.6f", &[PctValue::Float(skip)])?);
        }
        gcmd.respond_info(
            &format!("Starting tuning test ({})", parts.join(" ")),
            true,
        );
        Ok(())
    }

    /// `TuningTower.calc_value` (`klippy/extras/tuning_tower.py:68`).
    ///
    /// Before the first `TUNING_TOWER` command, `step_delta`,
    /// `step_height` and `skip` do not exist as attributes, so the
    /// Python reads raise `AttributeError` (starting with `skip`,
    /// the first one `calc_value` touches).
    pub fn calc_value(&self, z: f64) -> KResult<f64> {
        let (skip, step_height, step_delta, band, start, factor) = {
            let state = self.state.borrow();
            (
                state.skip,
                state.step_height,
                state.step_delta,
                state.band,
                state.start,
                state.factor,
            )
        };
        let skip = skip.ok_or_else(|| missing_attr("skip"))?;
        let mut z = z;
        if skip != 0.0 {
            z = (z - skip).max(0.0);
        }
        let step_height = step_height.ok_or_else(|| missing_attr("step_height"))?;
        if step_height != 0.0 {
            let step_delta = step_delta.ok_or_else(|| missing_attr("step_delta"))?;
            return Ok(start + step_delta * (z / step_height).floor());
        }
        if band != 0.0 {
            z = ((z / band).floor() + 0.5) * band;
        }
        Ok(start + z * factor)
    }

    /// `TuningTower.get_position` (`klippy/extras/tuning_tower.py:64`).
    pub fn get_position(&self) -> KResult<Vec<f64>> {
        let normal = {
            let state = self.state.borrow();
            state.normal_transform.clone()
        }
        .ok_or_else(none_get_position_attr)?;
        let pos = match &normal {
            MoveTransform::ToolHead(toolhead) => toolhead.get_position(),
            MoveTransform::Custom(custom) => custom.get_position()?,
        };
        self.state.borrow_mut().last_position = pos.clone();
        Ok(pos)
    }

    /// `TuningTower.move` (`klippy/extras/tuning_tower.py:77`), also the
    /// [`MoveTransformApi::move`] body.
    pub fn move_pos(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        // `self.normal_transform` may be `None` (tower inactive); the
        // reference still evaluates the trigger condition first and
        // only raises the `AttributeError` when forwarding the move
        // (`tuning_tower.py:78-97`).
        let normal = {
            let state = self.state.borrow();
            state.normal_transform.clone()
        };
        if newpos.len() < 4 {
            return Err(list_index_error());
        }
        let (last_position, last_z) = {
            let state = self.state.borrow();
            (state.last_position.clone(), state.last_z)
        };
        if last_position.len() < 4 {
            return Err(list_index_error());
        }
        let extruding = newpos[3] > last_position[3];
        let z_changed = newpos[2] != last_z;
        let xyz_changed = newpos[..3] != last_position[..3];
        if extruding && z_changed && xyz_changed {
            let z = newpos[2];
            if z < last_z - CANCEL_Z_DELTA {
                self.end_test()?;
            } else {
                let status = self.gcode_move.get_status(None);
                let gcode_z = extract_gcode_z(&status)?;
                let newval = self.calc_value(gcode_z)?;
                let command_fmt = self.state.borrow().command_fmt.clone();
                let changed = {
                    let mut state = self.state.borrow_mut();
                    state.last_z = z;
                    let changed = state.last_command_value != Some(newval);
                    if changed {
                        state.last_command_value = Some(newval);
                    }
                    changed
                };
                if changed {
                    let script =
                        fmt_percent(&command_fmt, &[PctValue::Float(newval)])?;
                    self.gcode.run_script_from_command(&script)?;
                }
            }
        }
        self.state.borrow_mut().last_position = newpos.clone();
        match &normal {
            Some(MoveTransform::ToolHead(toolhead)) => toolhead.r#move(newpos, speed),
            Some(MoveTransform::Custom(custom)) => custom.r#move(newpos, speed),
            None => Err(none_move_attr()),
        }
    }
    /// `TuningTower.end_test` (`klippy/extras/tuning_tower.py:98`).
    pub fn end_test(&self) -> KResult<()> {
        self.gcode.respond_info("Ending tuning test mode", true);
        // The reference restores `self.normal_transform` (possibly
        // `None`) even when the tower is inactive. The
        // `MoveTransform` enum cannot express `None`, so skip the
        // restore call when there is nothing to restore.
        let normal = {
            let state = self.state.borrow();
            state.normal_transform.clone()
        };
        if let Some(normal) = normal {
            self.gcode_move.set_move_transform(normal, true)?;
        }
        self.state.borrow_mut().normal_transform = None;
        Ok(())
    }

    /// `TuningTower.is_active` (`klippy/extras/tuning_tower.py:102`).
    pub fn is_active(&self) -> bool {
        self.state.borrow().normal_transform.is_some()
    }

    pub fn start(&self) -> f64 {
        self.state.borrow().start
    }

    pub fn factor(&self) -> f64 {
        self.state.borrow().factor
    }

    pub fn band(&self) -> f64 {
        self.state.borrow().band
    }

    pub fn step_delta(&self) -> Option<f64> {
        self.state.borrow().step_delta
    }

    pub fn step_height(&self) -> Option<f64> {
        self.state.borrow().step_height
    }

    pub fn skip(&self) -> Option<f64> {
        self.state.borrow().skip
    }

    pub fn last_z(&self) -> f64 {
        self.state.borrow().last_z
    }

    pub fn last_command_value(&self) -> Option<f64> {
        self.state.borrow().last_command_value
    }

    pub fn command_fmt(&self) -> String {
        self.state.borrow().command_fmt.clone()
    }

    pub fn last_position(&self) -> Vec<f64> {
        self.state.borrow().last_position.clone()
    }
}

impl MoveTransformApi for TuningTower {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.move_pos(newpos, speed)
    }

    fn get_position(&self) -> KResult<Vec<f64>> {
        TuningTower::get_position(self)
    }
}

klippy_api::printer_object!(TuningTower; dyn MoveTransformApi);

/// `load_config(config)` (`klippy/extras/tuning_tower.py:105`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(TuningTower::new(config)? as ObjectRef))
}

/// Module entry point for the `tuning_tower` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
