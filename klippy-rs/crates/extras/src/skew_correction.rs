//! Port of `klippy/extras/skew_correction.py` (Marlin skew correction).
//!
//! The object is installed as a move transform on `gcode_move`: every
//! commanded position is skewed before being passed down the transform
//! chain, and the current position is unskewed on the way back out.

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts as ConfigGetFloatOpts, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GetFloatOpts, GetIntOpts, GetOpts, MoveTransform, MoveTransformApi, RegisterCommandOpts,
    GCodeApi, GCodeMoveApi, GCmd, GCodeCommandHandler,
};
use klippy_api::object::{lookup_interface, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use pycompat::text::{float_from_string, float_str, percent_format, PctValue, PyError};
use pycompat::PyValue;
use std::sync::Arc;

const CMD_GET_CURRENT_SKEW_HELP: &str = "Report current printer skew";
const CMD_CALC_MEASURED_SKEW_HELP: &str = "Calculate skew from measured print";
const CMD_SET_SKEW_HELP: &str = "Set skew based on lengths of measured object";
const CMD_SKEW_PROFILE_HELP: &str = "Profile management for skew_correction";

fn lookup_gcode(printer: &Printer) -> KResult<Arc<dyn GCodeApi>> {
    let gcode = lookup_interface::<dyn GCodeApi>(&**printer, "gcode", LookupDefault::Raise)?;
    Ok(gcode.expect("LookupDefault::Raise never yields None"))
}

fn lookup_gcode_move(printer: &Printer) -> KResult<Arc<dyn GCodeMoveApi>> {
    let gcode_move =
        lookup_interface::<dyn GCodeMoveApi>(&**printer, "gcode_move", LookupDefault::Raise)?;
    Ok(gcode_move.expect("LookupDefault::Raise never yields None"))
}

fn lookup_configfile(printer: &Printer) -> KResult<Arc<dyn PrinterConfigApi>> {
    let configfile =
        lookup_interface::<dyn PrinterConfigApi>(&**printer, "configfile", LookupDefault::Raise)?;
    Ok(configfile.expect("LookupDefault::Raise never yields None"))
}

/// One stored skew profile (`xy_skew` / `xz_skew` / `yz_skew`).
#[derive(Clone, Copy, Debug, Default)]
pub struct SkewProfile {
    /// `xy_skew`.
    pub xy_skew: f64,
    /// `xz_skew`.
    pub xz_skew: f64,
    /// `yz_skew`.
    pub yz_skew: f64,
}

#[derive(Clone, Default)]
struct SkewState {
    current_profile_name: String,
    xy_factor: f64,
    xz_factor: f64,
    yz_factor: f64,
    skew_profiles: IndexMap<String, SkewProfile>,
    next_transform: Option<MoveTransform>,
}

/// The `skew_correction` printer object (`PrinterSkew`).
pub struct PrinterSkew {
    printer: Printer,
    name: String,
    state: GCell<SkewState>,
}

/// `calc_skew_factor(ac, bd, ad)` (module function of the reference).
///
/// CPython `math.sqrt` / `math.acos` raise `ValueError` on domain errors
/// and float division by zero raises `ZeroDivisionError`; both are
/// reproduced exactly (the underlying C `libm` calls would return NaN).
pub fn calc_skew_factor(ac: f64, bd: f64, ad: f64) -> KResult<f64> {
    let inner = 2.0 * ac * ac + 2.0 * bd * bd - 4.0 * ad * ad;
    if inner < 0.0 {
        return Err(math_domain_error());
    }
    let side = kos::libm::sqrt(inner) / 2.0;
    let denom = 2.0 * side * ad;
    if denom == 0.0 {
        return Err(KlippyError::Internal {
            class_name: "ZeroDivisionError".to_string(),
            message: "float division by zero".to_string(),
        });
    }
    let arg = (ac * ac - side * side - ad * ad) / denom;
    if arg < -1.0 || arg > 1.0 {
        return Err(math_domain_error());
    }
    Ok(kos::libm::tan(std::f64::consts::PI / 2.0 - kos::libm::acos(arg)))
}

fn math_domain_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "math domain error".to_string(),
    }
}

fn index_error() -> KlippyError {
    KlippyError::Internal {
        class_name: "IndexError".to_string(),
        message: "list index out of range".to_string(),
    }
}

fn none_attr_error(attr: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: "AttributeError".to_string(),
        message: format!("'NoneType' object has no attribute '{attr}'"),
    }
}

/// `math.degrees(factor)` (`factor * (180.0 / pi)` as in CPython).
fn degrees(factor: f64) -> f64 {
    factor * (180.0 / std::f64::consts::PI)
}

/// Applies `percent_format` and maps any formatting error the way an
/// unhandled CPython exception would surface.
fn fmt(fmt_str: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt_str, args).map_err(|e: PyError| KlippyError::Internal {
        class_name: e.kind.to_string(),
        message: e.msg,
    })
}

fn req_config_float(config: &Config, option: &str) -> KResult<f64> {
    match config.getfloat(option, ConfigGetFloatOpts::default())? {
        Some(v) => Ok(v),
        None => Err(KlippyError::config(format!(
            "Option '{}' in section '{}' must be specified",
            option,
            config.get_name()
        ))),
    }
}

fn opt_gcmd_str(gcmd: &GCmd, name: &str) -> KResult<Option<String>> {
    let mut opts = GetOpts::default();
    opts.default = Some(PyValue::None);
    match gcmd.get(name, opts)? {
        PyValue::None => Ok(None),
        v => Ok(Some(v.str())),
    }
}

fn req_gcmd_float(gcmd: &GCmd, name: &str, above: Option<f64>) -> KResult<f64> {
    let opts = GetFloatOpts {
        default: None,
        above,
        ..Default::default()
    };
    match gcmd.get_float(name, opts)? {
        Some(v) => Ok(v),
        None => Err(KlippyError::command(format!(
            "Error on '{}': missing {}",
            gcmd.get_commandline(),
            name
        ))),
    }
}

impl PrinterSkew {
    /// `PrinterSkew.__init__` (via `load_config`).
    pub fn new(config: &Config) -> KResult<Arc<PrinterSkew>> {
        let printer = config.get_printer();
        let name = config.get_name();
        let mut state = SkewState::default();
        load_storage(config, &name, &mut state.skew_profiles)?;
        let obj = Arc::new(PrinterSkew {
            printer: printer.clone(),
            name,
            state: GCell::new(state),
        });

        let connect_obj = Arc::clone(&obj);
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                connect_obj.handle_connect()?;
                Ok(PyValue::None)
            }),
        );

        let gcode = lookup_gcode(&printer)?;
        let h = Arc::clone(&obj);
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd| h.cmd_get_current_skew(gcmd));
        gcode.register_command(
            "GET_CURRENT_SKEW",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_GET_CURRENT_SKEW_HELP.to_string()),
            },
        )?;
        let h = Arc::clone(&obj);
        let handler: GCodeCommandHandler =
            Box::new(move |gcmd| h.cmd_calc_measured_skew(gcmd));
        gcode.register_command(
            "CALC_MEASURED_SKEW",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_CALC_MEASURED_SKEW_HELP.to_string()),
            },
        )?;
        let h = Arc::clone(&obj);
        let handler: GCodeCommandHandler = Box::new(move |gcmd| h.cmd_set_skew(gcmd));
        gcode.register_command(
            "SET_SKEW",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_SET_SKEW_HELP.to_string()),
            },
        )?;
        let h = Arc::clone(&obj);
        let handler: GCodeCommandHandler = Box::new(move |gcmd| h.cmd_skew_profile(gcmd));
        gcode.register_command(
            "SKEW_PROFILE",
            Some(handler),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some(CMD_SKEW_PROFILE_HELP.to_string()),
            },
        )?;
        Ok(obj)
    }
    fn handle_connect(self: &Arc<Self>) -> KResult<()> {
        let gcode_move = lookup_gcode_move(&self.printer)?;
        let cloned: Arc<PrinterSkew> = Arc::clone(self);
        let handle: Arc<dyn MoveTransformApi> = cloned;
        let prev = gcode_move.set_move_transform(MoveTransform::Custom(handle), true)?;
        self.state.borrow_mut().next_transform = prev;
        Ok(())
    }

    /// `calc_skew` (move-transform direction, commanded -> skewed).
    pub fn calc_skew(&self, pos: &[f64]) -> KResult<Vec<f64>> {
        let s = self.state.borrow();
        if pos.len() < 3 {
            return Err(index_error());
        }
        let skewed_x =
            pos[0] - pos[1] * s.xy_factor - pos[2] * (s.xz_factor - (s.xy_factor * s.yz_factor));
        let skewed_y = pos[1] - pos[2] * s.yz_factor;
        let mut out = vec![skewed_x, skewed_y];
        out.extend_from_slice(&pos[2..]);
        Ok(out)
    }

    /// `calc_unskew` (skewed -> commanded direction).
    pub fn calc_unskew(&self, pos: &[f64]) -> KResult<Vec<f64>> {
        let s = self.state.borrow();
        if pos.len() < 3 {
            return Err(index_error());
        }
        let skewed_x = pos[0] + pos[1] * s.xy_factor + pos[2] * s.xz_factor;
        let skewed_y = pos[1] + pos[2] * s.yz_factor;
        let mut out = vec![skewed_x, skewed_y];
        out.extend_from_slice(&pos[2..]);
        Ok(out)
    }

    /// `get_position`: unskew the next transform's position. Before
    /// `klippy:connect` the reference raises `AttributeError`
    /// (`'NoneType' object has no attribute 'get_position'`).
    pub fn get_position(&self) -> KResult<Vec<f64>> {
        let next = self.state.borrow().next_transform.clone();
        match next {
            Some(MoveTransform::ToolHead(th)) => self.calc_unskew(&th.get_position()),
            Some(MoveTransform::Custom(c)) => {
                let pos = c.get_position()?;
                self.calc_unskew(&pos)
            }
            None => Err(none_attr_error("get_position")),
        }
    }

    /// `_update_skew` (Python-private; public so tests can drive it).
    pub fn update_skew(&self, xy_factor: f64, xz_factor: f64, yz_factor: f64) -> KResult<()> {
        {
            let mut s = self.state.borrow_mut();
            s.xy_factor = xy_factor;
            s.xz_factor = xz_factor;
            s.yz_factor = yz_factor;
        }
        lookup_gcode_move(&self.printer)?.reset_last_position()?;
        Ok(())
    }

    /// Current XY skew factor (`xy_factor`).
    pub fn xy_factor(&self) -> f64 {
        self.state.borrow().xy_factor
    }

    /// Current XZ skew factor (`xz_factor`).
    pub fn xz_factor(&self) -> f64 {
        self.state.borrow().xz_factor
    }

    /// Current YZ skew factor (`yz_factor`).
    pub fn yz_factor(&self) -> f64 {
        self.state.borrow().yz_factor
    }

    /// `current_profile_name`.
    pub fn current_profile_name(&self) -> String {
        self.state.borrow().current_profile_name.clone()
    }

    /// Snapshot of `skew_profiles` in insertion order.
    pub fn profiles_snapshot(&self) -> Vec<(String, SkewProfile)> {
        self.state
            .borrow()
            .skew_profiles
            .iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect()
    }

    /// `cmd_GET_CURRENT_SKEW`.
    pub fn cmd_get_current_skew(&self, gcmd: &GCmd) -> KResult<()> {
        let (xy, xz, yz) = {
            let s = self.state.borrow();
            (s.xy_factor, s.xz_factor, s.yz_factor)
        };
        let mut out = "Current Printer Skew:".to_string();
        for (plane, fac) in [("XY", xy), ("XZ", xz), ("YZ", yz)] {
            out.push('\n');
            out.push_str(plane);
            out.push_str(&fmt(
                " Skew: %.6f radians, %.2f degrees",
                &[PctValue::Float(fac), PctValue::Float(degrees(fac))],
            )?);
        }
        gcmd.respond_info(&out, true);
        Ok(())
    }

    /// `cmd_CALC_MEASURED_SKEW`.
    pub fn cmd_calc_measured_skew(&self, gcmd: &GCmd) -> KResult<()> {
        let ac = req_gcmd_float(gcmd, "AC", Some(0.0))?;
        let bd = req_gcmd_float(gcmd, "BD", Some(0.0))?;
        let ad = req_gcmd_float(gcmd, "AD", Some(0.0))?;
        let factor = calc_skew_factor(ac, bd, ad)?;
        let out = fmt(
            "Calculated Skew: %.6f radians, %.2f degrees",
            &[PctValue::Float(factor), PctValue::Float(degrees(factor))],
        )?;
        gcmd.respond_info(&out, true);
        Ok(())
    }

    /// `cmd_SET_SKEW`.
    pub fn cmd_set_skew(&self, gcmd: &GCmd) -> KResult<()> {
        let opts = GetIntOpts {
            default: Some(Some(0)),
            ..Default::default()
        };
        if gcmd.get_int("CLEAR", opts)?.unwrap_or(0) != 0 {
            self.update_skew(0.0, 0.0, 0.0)?;
            return Ok(());
        }
        for plane in ["XY", "XZ", "YZ"] {
            let Some(lengths) = opt_gcmd_str(gcmd, plane)? else {
                continue;
            };
            let parsed: Vec<f64> = match parse_lengths(&lengths) {
                Ok(v) => v,
                Err(()) => {
                    return Err(KlippyError::command(fmt(
                        "skew_correction: improperly formatted entry for plane [%s]\n%s",
                        &[
                            PctValue::Str(plane.to_string()),
                            PctValue::Str(gcmd.get_commandline()),
                        ],
                    )?));
                }
            };
            let factor = calc_skew_factor(parsed[0], parsed[1], parsed[2])?;
            let mut s = self.state.borrow_mut();
            match plane {
                "XY" => s.xy_factor = factor,
                "XZ" => s.xz_factor = factor,
                _ => s.yz_factor = factor,
            }
        }
        Ok(())
    }

    /// `cmd_SKEW_PROFILE`.
    pub fn cmd_skew_profile(&self, gcmd: &GCmd) -> KResult<()> {
        if let Some(name) = opt_gcmd_str(gcmd, "LOAD")? {
            self.state.borrow_mut().current_profile_name = name.clone();
            let prof = self.state.borrow().skew_profiles.get(&name).copied();
            let Some(prof) = prof else {
                let out = fmt(
                    "skew_correction:  Load failed, unknown profile [%s]",
                    &[PctValue::Str(name)],
                )?;
                gcmd.respond_info(&out, true);
                return Ok(());
            };
            self.update_skew(prof.xy_skew, prof.xz_skew, prof.yz_skew)?;
        } else if let Some(name) = opt_gcmd_str(gcmd, "SAVE")? {
            let configfile = lookup_configfile(&self.printer)?;
            let cfg_name = format!("{} {}", self.name, name);
            let (xy, xz, yz) = {
                let s = self.state.borrow();
                (s.xy_factor, s.xz_factor, s.yz_factor)
            };
            configfile.set(&cfg_name, "xy_skew", &float_str(xy))?;
            configfile.set(&cfg_name, "xz_skew", &float_str(xz))?;
            configfile.set(&cfg_name, "yz_skew", &float_str(yz))?;
            self.state.borrow_mut().skew_profiles.insert(
                name.clone(),
                SkewProfile {
                    xy_skew: xy,
                    xz_skew: xz,
                    yz_skew: yz,
                },
            );
            let out = fmt(
                "Skew Correction state has been saved to profile [%s]\n\
                 for the current session.  The SAVE_CONFIG command will\n\
                 update the printer config file and restart the printer.",
                &[PctValue::Str(name)],
            )?;
            gcmd.respond_info(&out, true);
        } else if let Some(name) = opt_gcmd_str(gcmd, "REMOVE")? {
            let present = self.state.borrow().skew_profiles.contains_key(&name);
            if present {
                let configfile = lookup_configfile(&self.printer)?;
                configfile.remove_section(&format!("skew_correction {name}"));
                self.state.borrow_mut().skew_profiles.shift_remove(&name);
                let out = fmt(
                    "Profile [%s] removed from storage for this session.\n\
                     The SAVE_CONFIG command will update the printer\n\
                     configuration and restart the printer",
                    &[PctValue::Str(name)],
                )?;
                gcmd.respond_info(&out, true);
            } else {
                let out = fmt(
                    "skew_correction: No profile named [%s] to remove",
                    &[PctValue::Str(name)],
                )?;
                gcmd.respond_info(&out, true);
            }
        }
        Ok(())
    }
}

/// Parses one `SET_SKEW` plane value: `strip().split(",", 2)` then
/// `float(part.strip())` for each part; anything but exactly three
/// numbers is improperly formatted.
fn parse_lengths(value: &str) -> Result<Vec<f64>, ()> {
    let parts: Vec<&str> = value.trim().splitn(3, ',').collect();
    let mut lengths = Vec::with_capacity(parts.len());
    for part in parts {
        match float_from_string(part.trim()) {
            Ok(v) => lengths.push(v),
            Err(_) => return Err(()),
        }
    }
    if lengths.len() != 3 {
        return Err(());
    }
    Ok(lengths)
}

fn load_storage(
    config: &Config,
    name: &str,
    profiles: &mut IndexMap<String, SkewProfile>,
) -> KResult<()> {
    for profile in config.get_prefix_sections(name) {
        let profile_name_full = profile.get_name();
        if profile_name_full == name {
            continue;
        }
        let profile_name = profile_name_full
            .splitn(2, ' ')
            .nth(1)
            .ok_or_else(index_error)?;
        profiles.insert(
            profile_name.to_string(),
            SkewProfile {
                xy_skew: req_config_float(&profile, "xy_skew")?,
                xz_skew: req_config_float(&profile, "xz_skew")?,
                yz_skew: req_config_float(&profile, "yz_skew")?,
            },
        );
    }
    Ok(())
}

impl MoveTransformApi for PrinterSkew {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let corrected_pos = self.calc_skew(&newpos)?;
        let next = self.state.borrow().next_transform.clone();
        match next {
            Some(MoveTransform::ToolHead(th)) => th.r#move(corrected_pos, speed),
            Some(MoveTransform::Custom(c)) => c.r#move(corrected_pos, speed),
            None => Err(none_attr_error("move")),
        }
    }

    fn get_position(&self) -> KResult<Vec<f64>> {
        PrinterSkew::get_position(self)
    }
}

impl PrinterObject for PrinterSkew {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn MoveTransformApi);

    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut dict = IndexMap::new();
        dict.insert(
            PyValue::Str("current_profile_name".to_string()),
            PyValue::Str(self.current_profile_name()),
        );
        Some(PyValue::Dict(dict))
    }
}

/// `load_config(config)` entry point (`load_config` in the reference).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = PrinterSkew::new(config)?;
    Ok(Some(obj))
}

/// Module factory (only `load_config` is implemented by the reference).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
