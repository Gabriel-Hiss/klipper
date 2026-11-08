//! Port of `safe_z_home` (`klippy/extras/safe_z_home.py`).

use indexmap::IndexMap;
use klippy_api::config::{Config, GetBoolOpts, GetFloatListOpts, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCode, GCodeApi, GCodeCommandHandler, GetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, GCell, ObjectRef};
use pycompat::PyValue;
use pycompat::value::type_name;
use std::sync::Arc;
pub use crate::manual_probe::lookup_z_endstop_config;

/// `klippy/extras/safe_z_home.py:SafeZHoming`.
pub struct SafeZHoming {
    printer: Printer,
    gcode: GCode,
    prev_g28: GCell<Option<GCodeCommandHandler>>,
    home_x_pos: f64,
    home_y_pos: f64,
    z_hop: f64,
    z_hop_speed: f64,
    max_z: f64,
    speed: f64,
    move_to_previous: bool,
}

klippy_api::printer_object!(SafeZHoming);

impl SafeZHoming {
    /// `SafeZHoming.__init__(config)` (`klippy/extras/safe_z_home.py:9`).
    pub fn new(config: &Config) -> KResult<Arc<SafeZHoming>> {
        let printer = config.get_printer();
        let home_xy = match config.getfloatlist(
            "home_xy_position",
            GetFloatListOpts {
                default: None,
                count: Some(2),
                ..GetFloatListOpts::default()
            },
        )? {
            Some(v) => v,
            None => {
                return Err(KlippyError::config(format!(
                    "Option 'home_xy_position' in section '{}' must be specified",
                    config.get_name()
                )))
            }
        };
        let (home_x_pos, home_y_pos) = match home_xy.as_slice() {
            [x, y] => (*x, *y),
            _ => {
                let msg = if home_xy.len() < 2 {
                    format!(
                        "not enough values to unpack (expected 2, got {})",
                        home_xy.len()
                    )
                } else {
                    format!("too many values to unpack (expected 2)")
                };
                return Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: msg,
                });
            }
        };
        let z_hop = get_float_default(config, "z_hop", 0.0, None)?;
        let z_hop_speed = get_float_default(config, "z_hop_speed", 15.0, Some(0.0))?;
        let zconfig = match lookup_z_endstop_config(config)? {
            Some(c) => c,
            None => {
                return Err(KlippyError::config(
                    "Missing Z endstop config for safe_z_homing",
                ))
            }
        };
        let max_z = match zconfig.getfloat(
            "position_max",
            GetFloatOpts {
                default: None,
                note_valid: false,
                ..GetFloatOpts::default()
            },
        )? {
            Some(v) => v,
            None => {
                return Err(KlippyError::config(format!(
                    "Option 'position_max' in section '{}' must be specified",
                    zconfig.get_name()
                )))
            }
        };
        let speed = get_float_default(config, "speed", 50.0, Some(0.0))?;
        let move_to_previous = match config.getboolean(
            "move_to_previous",
            GetBoolOpts {
                default: Some(Some(false)),
                note_valid: true,
            },
        )? {
            Some(v) => v,
            None => false,
        };
        // `printer.load_object(config, 'homing')` — side effect only; the
        // returned object is unused by the Python code.
        printer.load_object(config, "homing", LookupDefault::Raise)?;
        let gcode =
            match lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)? {
                Some(g) => g,
                None => return Err(KlippyError::config("Unknown config object 'gcode'")),
            };
        let this = Arc::new(SafeZHoming {
            printer,
            gcode: gcode.clone(),
            prev_g28: GCell::new(None),
            home_x_pos,
            home_y_pos,
            z_hop,
            z_hop_speed,
            max_z,
            speed,
            move_to_previous,
        });
        let prev = gcode.register_command("G28", None, RegisterCommandOpts::default())?;
        this.prev_g28.replace(prev);
        let this_for_handler = this.clone();
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd: &GCmd| this_for_handler.cmd_g28(gcmd));
        gcode.register_command("G28", Some(handler), RegisterCommandOpts::default())?;
        if config.has_section("homing_override") {
            return Err(KlippyError::config(
                "homing_override and safe_z_homing cannot be used simultaneously",
            ));
        }
        Ok(this)
    }

    /// `SafeZHoming.home_x_pos` (`klippy/extras/safe_z_home.py:12`).
    pub fn home_x_pos(&self) -> f64 {
        self.home_x_pos
    }

    /// `SafeZHoming.home_y_pos` (`klippy/extras/safe_z_home.py:12`).
    pub fn home_y_pos(&self) -> f64 {
        self.home_y_pos
    }

    /// `SafeZHoming.z_hop` (`klippy/extras/safe_z_home.py:13`).
    pub fn z_hop(&self) -> f64 {
        self.z_hop
    }

    /// `SafeZHoming.z_hop_speed` (`klippy/extras/safe_z_home.py:14`).
    pub fn z_hop_speed(&self) -> f64 {
        self.z_hop_speed
    }

    /// `SafeZHoming.max_z` (`klippy/extras/safe_z_home.py:18`).
    pub fn max_z(&self) -> f64 {
        self.max_z
    }

    /// `SafeZHoming.speed` (`klippy/extras/safe_z_home.py:19`).
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// `SafeZHoming.move_to_previous` (`klippy/extras/safe_z_home.py:20`).
    pub fn move_to_previous(&self) -> bool {
        self.move_to_previous
    }

    /// Whether the previous `G28` handler is present.
    pub fn has_prev_g28(&self) -> bool {
        self.prev_g28.borrow().is_some()
    }

    fn call_prev(&self, gcmd: &GCmd) -> KResult<()> {
        let taken = {
            let mut guard = self.prev_g28.borrow_mut();
            guard.take()
        };
        match taken {
            Some(mut h) => {
                let r = h(gcmd);
                let mut guard = self.prev_g28.borrow_mut();
                *guard = Some(h);
                r
            }
            None => Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not callable".to_string(),
            }),
        }
    }

    /// `SafeZHoming.cmd_G28(gcmd)` (`klippy/extras/safe_z_home.py:30`).
    pub fn cmd_g28(&self, gcmd: &GCmd) -> KResult<()> {
        let toolhead = match lookup_interface::<dyn ToolHeadApi>(
            &*self.printer,
            "toolhead",
            LookupDefault::Raise,
        )? {
            Some(t) => t,
            None => return Err(KlippyError::config("Unknown config object 'toolhead'")),
        };
        // Perform Z Hop if necessary.
        if self.z_hop != 0.0 {
            let curtime = self.printer.get_reactor().monotonic();
            let kin_status = toolhead.get_kinematics().get_status(curtime);
            let mut pos = toolhead.get_position();
            if !axis_homed(&kin_status, 'z')? {
                if pos.len() < 3 {
                    return Err(KlippyError::Internal {
                        class_name: "IndexError".to_string(),
                        message: "list assignment index out of range".to_string(),
                    });
                }
                pos[2] = 0.0;
                toolhead.set_position(pos, "z")?;
                toolhead.manual_move(vec![None, None, Some(self.z_hop)], self.z_hop_speed)?;
                toolhead.get_kinematics().clear_homing_state("z");
            } else if pos.get(2).ok_or_else(index_error)? < &self.z_hop {
                toolhead.manual_move(vec![None, None, Some(self.z_hop)], self.z_hop_speed)?;
            }
        }
        // Determine which axes we need to home.
        let need_x_val = gcmd.get("X", none_default())?;
        let need_y_val = gcmd.get("Y", none_default())?;
        let need_z_val = gcmd.get("Z", none_default())?;
        let (mut need_x, mut need_y, mut need_z) = (
            !matches!(need_x_val, PyValue::None),
            !matches!(need_y_val, PyValue::None),
            !matches!(need_z_val, PyValue::None),
        );
        if !need_x && !need_y && !need_z {
            need_x = true;
            need_y = true;
            need_z = true;
        }
        // Home XY axes if necessary.
        let mut new_params = IndexMap::new();
        if need_x {
            new_params.insert("X".to_string(), "0".to_string());
        }
        if need_y {
            new_params.insert("Y".to_string(), "0".to_string());
        }
        if !new_params.is_empty() {
            let g28_gcmd = self.gcode.create_gcode_command("G28", "G28", &new_params);
            self.call_prev(&g28_gcmd)?;
        }
        // Home Z axis if necessary.
        if need_z {
            let curtime = self.printer.get_reactor().monotonic();
            let kin_status = toolhead.get_kinematics().get_status(curtime);
            if !axis_homed(&kin_status, 'x')? || !axis_homed(&kin_status, 'y')? {
                return Err(KlippyError::command("Must home X and Y axes first"));
            }
            // Move to safe XY homing position.
            let prevpos = toolhead.get_position();
            toolhead.manual_move(
                vec![Some(self.home_x_pos), Some(self.home_y_pos)],
                self.speed,
            )?;
            // Home Z.
            let params = IndexMap::from([("Z".to_string(), "0".to_string())]);
            let g28_gcmd = self.gcode.create_gcode_command("G28", "G28", &params);
            self.call_prev(&g28_gcmd)?;
            // Perform Z Hop again for pressure-based probes.
            if self.z_hop != 0.0 {
                let pos = toolhead.get_position();
                if pos.get(2).ok_or_else(index_error)? < &self.z_hop {
                    toolhead.manual_move(vec![None, None, Some(self.z_hop)], self.z_hop_speed)?;
                }
            }
            // Move XY back to previous positions.
            if self.move_to_previous {
                let xy: Vec<Option<f64>> = prevpos.iter().take(2).map(|v| Some(*v)).collect();
                toolhead.manual_move(xy, self.speed)?;
            }
        }
        Ok(())
    }
}

fn get_float_default(
    config: &Config,
    option: &str,
    default: f64,
    above: Option<f64>,
) -> KResult<f64> {
    match config.getfloat(
        option,
        GetFloatOpts {
            default: Some(Some(default)),
            above,
            ..GetFloatOpts::default()
        },
    )? {
        Some(v) => Ok(v),
        None => Ok(default),
    }
}

fn none_default() -> GetOpts {
    GetOpts {
        default: Some(PyValue::None),
        parser: GetParser::Str,
        minval: None,
        maxval: None,
        above: None,
        below: None,
    }
}

fn axis_homed(status: &PyValue, axis: char) -> KResult<bool> {
    let homed = match status {
        PyValue::Dict(d) => {
            let mut found: Option<&PyValue> = None;
            for (k, v) in d.iter() {
                if matches!(k, PyValue::Str(s) if s == "homed_axes") {
                    found = Some(v);
                    break;
                }
            }
            match found {
                Some(v) => v,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: "'homed_axes'".to_string(),
                    })
                }
            }
        }
        PyValue::List(_) => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "list indices must be integers or slices, not str".to_string(),
            })
        }
        PyValue::Tuple(_) => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "tuple indices must be integers or slices, not str".to_string(),
            })
        }
        PyValue::Str(_) | PyValue::Bytes(_) => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "string indices must be integers".to_string(),
            })
        }
        other => {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!("'{}' object is not subscriptable", type_name(other)),
            })
        }
    };
    match homed {
        PyValue::Str(s) => Ok(s.contains(axis)),
        PyValue::List(items) | PyValue::Tuple(items) => Ok({
            let needle = axis.to_string();
            items.iter().any(|x| matches!(x, PyValue::Str(s) if *s == needle))
        }),
        PyValue::Dict(d) => Ok({
            let needle = axis.to_string();
            d.keys()
                .any(|k| matches!(k, PyValue::Str(s) if *s == needle))
        }),
        PyValue::Bytes(_) => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "a bytes-like object is required, not 'str'".to_string(),
        }),
        other => Err(KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: format!("argument of type '{}' is not iterable", type_name(other)),
        }),
    }
}
fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

/// `load_config(config)` (`klippy/extras/safe_z_home.py:92`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(SafeZHoming::new(config)? as ObjectRef))
}

/// Module entry point for the `safe_z_home` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
