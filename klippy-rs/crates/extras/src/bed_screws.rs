//! Port of the `bed_screws` module (`klippy/extras/bed_screws.py`).
//!
//! Helps the user adjust bed leveling screws one by one. The state machine
//! starts with `BED_SCREWS_ADJUST`, registers the point commands `ACCEPT`,
//! `ADJUSTED` and `ABORT` for the screw currently under adjustment, and
//! walks through every `screwN` (and every `screwN_fine_adjust` once the
//! coarse pass completes) until all are accepted.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatListOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeCommandHandler, GCodeApi, RegisterCommandOpts, GCmd};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

/// Mutable state (`klippy/extras/bed_screws.py:40` `reset`).
#[derive(Default, Clone, Copy)]
struct State {
    /// Current pass name (`'adjust'` or `'fine'`), or `None` when inactive.
    state: Option<&'static str>,
    /// Index of the screw currently under adjustment.
    current_screw: i64,
    /// Screws accepted in the current pass.
    accepted_screws: i64,
}

fn percent_err(e: pycompat::text::PyError) -> KlippyError {
    KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg }
}

/// `klippy/extras/bed_screws.py:BedScrews`.
pub struct BedScrews {
    printer: Printer,
    gcode: GCode,
    /// Coarse pass: `(coordinate, name)` per `screwN`.
    adjust: Vec<(Vec<f64>, String)>,
    /// Fine pass: `(coordinate, name)` per `screwN_fine_adjust`.
    fine: Vec<(Vec<f64>, String)>,
    /// `len(screws)`; a pass completes once this many are accepted.
    number_of_screws: i64,
    /// `speed`.
    speed: f64,
    /// `probe_speed` (the lift speed).
    lift_speed: f64,
    /// `horizontal_move_z`.
    horizontal_move_z: f64,
    /// `probe_height`.
    probe_z: f64,
    state: GCell<State>,
}

impl BedScrews {
    /// `BedScrews.__init__(config)` (`klippy/extras/bed_screws.py:8`).
    ///
    /// Returns the object as an `Arc` because the registered G-code handlers
    /// capture it.
    pub fn new(config: &Config) -> KResult<Arc<BedScrews>> {
        let printer = config.get_printer();
        let mut screws: Vec<(Vec<f64>, String)> = Vec::new();
        let mut fine_adjust: Vec<(Vec<f64>, String)> = Vec::new();
        for i in 0..99 {
            let prefix = format!("screw{}", i + 1);
            let present = config.get(
                &prefix,
                GetStrOpts { default: Some(None), note_valid: true },
            )?;
            if present.is_none() {
                break;
            }
            let screw_coord = match config.getfloatlist(
                &prefix,
                GetFloatListOpts { default: None, sep: ",".to_string(), count: Some(2), note_valid: true, ..Default::default() },
            )? {
                Some(coord) => coord,
                None => break,
            };
            let default_name = percent_format(
                "screw at %.3f,%.3f",
                &[PctValue::Float(screw_coord[0]), PctValue::Float(screw_coord[1])],
            )
            .map_err(percent_err)?;
            let screw_name = config
                .get(
                    &format!("{prefix}_name"),
                    GetStrOpts {
                        default: Some(Some(default_name.clone())),
                        note_valid: true,
                    },
                )?
                .unwrap_or(default_name);
            screws.push((screw_coord, screw_name.clone()));
            let pfa = format!("{prefix}_fine_adjust");
            let has_fine = config
                .get(&pfa, GetStrOpts { default: Some(None), note_valid: true })?
                .is_some();
            if has_fine {
                if let Some(fine_coord) = config.getfloatlist(
                    &pfa,
                    GetFloatListOpts { default: None, sep: ",".to_string(), count: Some(2), note_valid: true, ..Default::default() },
                )? {
                    fine_adjust.push((fine_coord, screw_name));
                }
            }
        }
        if screws.len() < 3 {
            return Err(KlippyError::config("bed_screws: Must have at least three screws"));
        }
        let number_of_screws = screws.len() as i64;
        let speed = config
            .getfloat(
                "speed",
                GetFloatOpts {
                    default: Some(Some(50.0)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(50.0);
        let lift_speed = config
            .getfloat(
                "probe_speed",
                GetFloatOpts {
                    default: Some(Some(5.0)),
                    minval: None,
                    maxval: None,
                    above: Some(0.0),
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(5.0);
        let horizontal_move_z = config
            .getfloat(
                "horizontal_move_z",
                GetFloatOpts {
                    default: Some(Some(5.0)),
                    minval: None,
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(5.0);
        let probe_z = config
            .getfloat(
                "probe_height",
                GetFloatOpts {
                    default: Some(Some(0.0)),
                    minval: None,
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                    ..Default::default()
                },
            )?
            .unwrap_or(0.0);
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let bed_screws = Arc::new(BedScrews {
            printer,
            gcode,
            adjust: screws,
            fine: fine_adjust,
            number_of_screws,
            speed,
            lift_speed,
            horizontal_move_z,
            probe_z,
            state: GCell::new(State::default()),
        });
        bed_screws.register_adjust_command()?;
        Ok(bed_screws)
    }

    fn register_adjust_command(self: &Arc<Self>) -> KResult<()> {
        let this = Arc::clone(self);
        self.gcode.register_command(
            "BED_SCREWS_ADJUST",
            Some(Box::new(move |gcmd: &GCmd| this.cmd_bed_screws_adjust(gcmd)) as GCodeCommandHandler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Tool to help adjust bed leveling screws".to_string()),
            },
        )?;
        Ok(())
    }

    /// `BedScrews.reset()` (`klippy/extras/bed_screws.py:40`).
    pub fn reset(&self) {
        self.state.replace(State::default());
    }

    /// `BedScrews.move()` (`klippy/extras/bed_screws.py:44`).
    fn r#move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        match toolhead.manual_move(coord, speed) {
            Ok(()) => Ok(()),
            Err(e @ KlippyError::Command(_)) => {
                self.unregister_commands()?;
                self.reset();
                Err(e)
            }
            Err(e) => Err(e),
        }
    }

    /// `BedScrews.move_to_screw()` (`klippy/extras/bed_screws.py:51`).
    fn move_to_screw(self: &Arc<Self>, state: &'static str, screw: usize) -> KResult<()> {
        // Move up, over, and then down.
        self.r#move(vec![None, None, Some(self.horizontal_move_z)], self.lift_speed)?;
        let (coord, name) = {
            let list = match state {
                "adjust" => &self.adjust,
                "fine" => &self.fine,
                _ => {
                    return Err(KlippyError::Other {
                        class_name: "KeyError".to_string(),
                        message: format!("'{state}'"),
                    });
                }
            };
            let (coord, name) = &list[screw];
            (coord.clone(), name.clone())
        };
        self.r#move(
            vec![Some(coord[0]), Some(coord[1]), Some(self.horizontal_move_z)],
            self.speed,
        )?;
        self.r#move(vec![Some(coord[0]), Some(coord[1]), Some(self.probe_z)], self.lift_speed)?;
        // Update state.
        self.state.with_mut(|s| {
            s.state = Some(state);
            s.current_screw = screw as i64;
        });
        self.gcode.respond_info(
            &percent_format(
                "Adjust %s. Then run ACCEPT, ADJUSTED, or ABORT\n\
                 Use ADJUSTED if a significant screw adjustment is made",
                &[PctValue::Str(name)],
            )
            .map_err(percent_err)?,
            true,
        );
        // Register commands.
        let accept = Arc::clone(self);
        self.gcode.register_command(
            "ACCEPT",
            Some(Box::new(move |gcmd: &GCmd| accept.cmd_accept(gcmd)) as GCodeCommandHandler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Accept bed screw position".to_string()),
            },
        )?;
        let adjusted = Arc::clone(self);
        self.gcode.register_command(
            "ADJUSTED",
            Some(Box::new(move |gcmd: &GCmd| adjusted.cmd_adjusted(gcmd)) as GCodeCommandHandler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Accept bed screw position after notable adjustment".to_string()),
            },
        )?;
        let abort = Arc::clone(self);
        self.gcode.register_command(
            "ABORT",
            Some(Box::new(move |gcmd: &GCmd| abort.cmd_abort(gcmd)) as GCodeCommandHandler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("Abort bed screws tool".to_string()),
            },
        )?;
        Ok(())
    }

    /// `BedScrews.unregister_commands()` (`klippy/extras/bed_screws.py:70`).
    pub fn unregister_commands(&self) -> KResult<()> {
        for cmd in ["ACCEPT", "ADJUSTED", "ABORT"] {
            self.gcode.register_command(cmd, None, RegisterCommandOpts::default())?;
        }
        Ok(())
    }

    /// `BedScrews.get_status()` (`klippy/extras/bed_screws.py:74`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        let state = self.state.cloned();
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("is_active".to_string()),
            PyValue::Bool(state.state.is_some()),
        );
        status.insert(
            PyValue::Str("state".to_string()),
            match state.state {
                Some(name) => PyValue::Str(name.to_string()),
                None => PyValue::None,
            },
        );
        status.insert(
            PyValue::Str("current_screw".to_string()),
            PyValue::Int(pycompat::PyInt::from_i64(state.current_screw)),
        );
        status.insert(
            PyValue::Str("accepted_screws".to_string()),
            PyValue::Int(pycompat::PyInt::from_i64(state.accepted_screws)),
        );
        PyValue::Dict(status)
    }

    /// `BedScrews.cmd_BED_SCREWS_ADJUST()` (`klippy/extras/bed_screws.py:82`).
    pub fn cmd_bed_screws_adjust(self: &Arc<Self>, _gcmd: &GCmd) -> KResult<()> {
        if self.state.borrow().state.is_some() {
            return Err(KlippyError::Command(
                "Already in bed_screws helper; use ABORT to exit".to_string(),
            ));
        }
        // reset accepted screws
        self.state.with_mut(|s| s.accepted_screws = 0);
        self.r#move(vec![None, None, Some(self.horizontal_move_z)], self.speed)?;
        self.move_to_screw("adjust", 0)
    }

    /// `BedScrews.cmd_ACCEPT()` (`klippy/extras/bed_screws.py:90`).
    pub fn cmd_accept(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        self.unregister_commands()?;
        self.state.with_mut(|s| s.accepted_screws += 1);
        let state = self.state.cloned();
        let Some(state_name) = state.state else {
            return Err(KlippyError::Other {
                class_name: "KeyError".to_string(),
                message: "None".to_string(),
            });
        };
        let len = match state_name {
            "adjust" => self.adjust.len() as i64,
            "fine" => self.fine.len() as i64,
            _ => {
                return Err(KlippyError::Other {
                    class_name: "KeyError".to_string(),
                    message: format!("'{state_name}'"),
                });
            }
        };
        if state.current_screw + 1 < len && state.accepted_screws < self.number_of_screws {
            // Continue with next screw.
            self.move_to_screw(state_name, (state.current_screw + 1) as usize)?;
            return Ok(());
        }
        if state.accepted_screws < self.number_of_screws {
            // Retry coarse adjustments.
            self.move_to_screw("adjust", 0)?;
            return Ok(());
        }
        if state_name == "adjust" && !self.fine.is_empty() {
            // Reset accepted screws for fine adjustment.
            self.state.with_mut(|s| s.accepted_screws = 0);
            // Perform fine screw adjustments.
            self.move_to_screw("fine", 0)?;
            return Ok(());
        }
        // Done.
        self.reset();
        self.r#move(vec![None, None, Some(self.horizontal_move_z)], self.lift_speed)?;
        gcmd.respond_info("Bed screws tool completed successfully", true);
        Ok(())
    }

    /// `BedScrews.cmd_ADJUSTED()` (`klippy/extras/bed_screws.py:113`).
    pub fn cmd_adjusted(self: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        self.unregister_commands()?;
        self.state.with_mut(|s| s.accepted_screws = -1);
        self.cmd_accept(gcmd)
    }

    /// `BedScrews.cmd_ABORT()` (`klippy/extras/bed_screws.py:118`).
    pub fn cmd_abort(&self, _gcmd: &GCmd) -> KResult<()> {
        self.unregister_commands()?;
        self.reset();
        Ok(())
    }
}

impl PrinterObject for BedScrews {
    klippy_api::printer_object_base!();

    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.get_status(_eventtime))
    }
}

/// `load_config(config)` (`klippy/extras/bed_screws.py:122`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(BedScrews::new(config)? as ObjectRef))
}

/// Module entry point for the `bed_screws` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
