//! Port of `klippy/kinematics/kinematic_stepper.py`.
//!
//! `KinematicStepper` binds one `stepper.PrinterStepper` to the generic
//! cartesian kinematics: it parses the `carriages` config string into
//! per-axis coefficients plus the referenced carriages, installs the
//! `generic_cartesian_stepper_alloc` chelper stepper kinematics on the
//! stepper, and registers the stepper with each referenced carriage.
//!
//! `parse_carriages_string` and the `KinematicStepper` methods mirror the
//! Python line by line, including the error texts and their classes
//! (`config.error` at construction, `printer.command_error` /
//! `gcmd.error` on update).
//!
//! Contracts this module builds on (see the report):
//! - `stepper.PrinterStepper` comes from `toolhead.stepper_factory()`,
//!   which is why the constructor takes the factory.
//! - A carriage (`MainCarriage` / `ExtraCarriage` / `DualCarriage` of
//!   `generic_cartesian.py`) is reached through [`CarriageApi`]; that
//!   module owns the classes and implements the trait.
//! - The chelper call `generic_cartesian_stepper_set_coeffs` acts on the
//!   opaque stepper-kinematics handle; [`KinematicStepper`] reaches the
//!   `chelper_motion::kin_generic::GenericCartesianStepper` through
//!   `klippy_core::motion_handles::kinematics_of` and matches
//!   [`KinAlloc::GenericCartesian`](klippy_core::motion_handles::KinAlloc::GenericCartesian).
//!   It defines no copy of that struct.
//!
use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::printer::ModuleFactory;
use klippy_api::stepper::{Stepper, StepperFactory, StepperKinematics};
use klippy_core::motion_handles::{kinematics_of, KinAlloc};
use pycompat::text::float_from_string;
use pycompat::PyValue;
use pytext::pyre;
use std::sync::Arc;

/// A carriage that `KinematicStepper` references by name.
///
/// Port of the duck type used by `parse_carriages_string` and
/// `KinematicStepper.__init__` (`klippy/kinematics/generic_cartesian.py`:
/// `MainCarriage`, `ExtraCarriage`, `DualCarriage`). The owning module
/// (`generic_cartesian`) implements this trait.
pub trait CarriageApi: Send + Sync + 'static {
    /// `carriage.get_axis()` (`generic_cartesian.py:26,48,98`): the
    /// 0-based axis index (`ord(axis_name) - ord('x')`).
    fn get_axis(&self) -> i64;
    /// `carriage.add_stepper(kin_stepper)`
    /// (`generic_cartesian.py:30,52,116`). The implementation extracts
    /// `kin_stepper.get_stepper()` and adds it to its rail.
    fn add_stepper(&self, kin_stepper: &KinematicStepper) -> KResult<()>;
    /// Upcast for the owning crate's downcast.
    fn as_any(&self) -> &dyn std::any::Any;
}

/// A carriage handle (`Arc<dyn CarriageApi>`).
pub type Carriage = Arc<dyn CarriageApi>;


/// Parses `carriages_str` into per-axis coefficients and carriages.
///
/// Port of `parse_carriages_string`
/// (`klippy/kinematics/kinematic_stepper.py:10-44`). `parse_error`
/// builds the exception the Python `parse_error` callable would raise
/// (`config.error` at construction, `printer.command_error` or
/// `gcmd.error` on update).
pub fn parse_carriages_string(
    carriages_str: &str,
    printer_carriages: &IndexMap<String, Carriage>,
    parse_error: &dyn Fn(String) -> KlippyError,
) -> KResult<(Vec<f64>, Vec<Carriage>)> {
    let pat = pyre::compile("[+-]").expect("supported re pattern");
    let chars: Vec<char> = carriages_str.chars().collect();
    let mut nxt = 0usize;
    let mut coeffs = vec![0f64; 3];
    let mut ref_carriages: Vec<Carriage> = Vec::new();
    while nxt < chars.len() {
        let end = pat
            .search_at(carriages_str, nxt + 1)
            .map(|(start, _)| start)
            .unwrap_or(chars.len());
        let term: String = chars[nxt..end]
            .iter()
            .collect::<String>()
            .trim_matches(|c: char| c.is_whitespace())
            .to_string();
        // `term.split('*')` keeps empty pieces, like Python.
        let mut term_lst: Vec<String> = term.split('*').map(str::to_string).collect();
        if !(1..=2).contains(&term_lst.len()) {
            return Err(parse_error(format!(
                "Invalid term '{term}' in '{carriages_str}'"
            )));
        }
        let coeff = if term_lst.len() == 2 {
            match float_from_string(&term_lst[0]) {
                Ok(v) => v,
                Err(_) => {
                    return Err(parse_error(format!(
                        "Invalid float '{}'",
                        term_lst[0]
                    )))
                }
            }
        } else {
            let coeff = if term_lst[0].starts_with('-') {
                -1.0
            } else {
                1.0
            };
            if term_lst[0].starts_with('-') || term_lst[0].starts_with('+') {
                term_lst[0] = term_lst[0][1..].to_string();
            }
            coeff
        };
        let c = term_lst.last().expect("non-empty split").clone();
        let Some(carriage) = printer_carriages.get(&c) else {
            return Err(parse_error(format!(
                "Invalid '{c}' carriage referenced in '{carriages_str}'"
            )));
        };
        let j = carriage.get_axis();
        // Python integer indexing: negative indices wrap around, out of
        // range raises IndexError.
        let idx = py_index(j, coeffs.len())?;
        if coeffs[idx] != 0.0 {
            let axis = "xyz".chars().nth(idx).expect("axis in range");
            return Err(parse_error(format!(
                "Axis '{axis}' was referenced multiple times by carriages in '{carriages_str}'"
            )));
        }
        coeffs[idx] = coeff;
        ref_carriages.push(carriage.clone());
        nxt = end;
    }
    Ok((coeffs, ref_carriages))
}

/// Python `list[i]` / `str[i]` indexing with negative wraparound.
fn py_index(i: i64, len: usize) -> KResult<usize> {
    let len = len as i64;
    let idx = if i < 0 { i + len } else { i };
    if idx < 0 || idx >= len {
        return Err(KlippyError::Other {
            class_name: "IndexError".to_string(),
            message: "list index out of range".to_string(),
        });
    }
    Ok(idx as usize)
}

/// A generic-cartesian stepper bound to its carriages.
///
/// Port of `KinematicStepper`
/// (`klippy/kinematics/kinematic_stepper.py:46-92`). It is not a printer
/// object; `GenericCartesianKinematics` holds it and drives it through
/// `set_position`, `get_kin_coeffs`, `update_carriages` and
/// `update_kin_coeffs`.
pub struct KinematicStepper {
    stepper: Stepper,
    stepper_sk: Option<StepperKinematics>,
    kin_coeffs: GCell<Vec<f64>>,
    carriages: GCell<Vec<Carriage>>,
}

impl KinematicStepper {
    /// `KinematicStepper(config, printer_carriages)`
    /// (`kinematic_stepper.py:47-62`). The stepper factory stands in for
    /// the `import stepper` the Python module performs.
    pub fn new(
        stepper_factory: &StepperFactory,
        config: &Config,
        printer_carriages: &IndexMap<String, Carriage>,
    ) -> KResult<Self> {
        let stepper = stepper_factory.printer_stepper(config, false)?;
        let carriages_str = config.get("carriages", GetStrOpts::default())?;
        let carriages_str = carriages_str.expect("required option 'carriages'");
        let (kin_coeffs, carriages) =
            parse_carriages_string(&carriages_str, printer_carriages, &KlippyError::Config)?;
        if !kin_coeffs.iter().any(|c| *c != 0.0) {
            return Err(KlippyError::Config(format!(
                "'{}' must provide a valid 'carriages' configuration",
                stepper.get_name(None)
            )));
        }
        stepper.setup_itersolve(
            "generic_cartesian_stepper_alloc",
            &[
                PyValue::Float(kin_coeffs[0]),
                PyValue::Float(kin_coeffs[1]),
                PyValue::Float(kin_coeffs[2]),
            ],
        );
        let obj = KinematicStepper {
            stepper_sk: stepper.get_stepper_kinematics(),
            kin_coeffs: GCell::new(kin_coeffs),
            carriages: GCell::new(carriages.clone()),
            stepper,
        };
        for sc in &carriages {
            sc.add_stepper(&obj)?;
        }
        Ok(obj)
    }

    /// `get_name(short=False)`
    /// (`kinematic_stepper.py:63-67`).
    pub fn get_name(&self, short: bool) -> String {
        let name = self.stepper.get_name(Some(short));
        if short && name.starts_with("stepper ") {
            return name[8..].to_string();
        }
        name
    }

    /// `get_stepper()` (`kinematic_stepper.py:68-69`).
    pub fn get_stepper(&self) -> Stepper {
        self.stepper.clone()
    }

    /// `get_kin_coeffs()` (`kinematic_stepper.py:70-71`): the
    /// coefficients as a tuple, here a `Vec<f64>`.
    pub fn get_kin_coeffs(&self) -> Vec<f64> {
        self.kin_coeffs.cloned()
    }

    /// `get_active_axes()` (`kinematic_stepper.py:72-73`): indices of
    /// the axes whose coefficient is truthy (`NaN` counts as active,
    /// `-0.0` does not, as Python float truthiness does).
    pub fn get_active_axes(&self) -> Vec<i64> {
        self.kin_coeffs
            .borrow()
            .iter()
            .enumerate()
            .filter(|(_, c)| **c != 0.0)
            .map(|(i, _)| i as i64)
            .collect()
    }

    /// `get_carriages()` (`kinematic_stepper.py:74-75`).
    pub fn get_carriages(&self) -> Vec<Carriage> {
        self.carriages.cloned()
    }

    /// `update_kin_coeffs(kin_coeffs)`
    /// (`kinematic_stepper.py:76-80`): stores the coefficients and
    /// pushes them into the chelper stepper kinematics
    /// (`generic_cartesian_stepper_set_coeffs`).
    pub fn update_kin_coeffs(&self, kin_coeffs: Vec<f64>) -> KResult<()> {
        self.kin_coeffs.replace(kin_coeffs.clone());
        let c0 = kin_coeffs[py_index(0, kin_coeffs.len())?];
        let c1 = kin_coeffs[py_index(1, kin_coeffs.len())?];
        let c2 = kin_coeffs[py_index(2, kin_coeffs.len())?];
        if let Some(sk) = &self.stepper_sk {
            let handle = kinematics_of(sk)?;
            let ok = handle.with(|alloc| match alloc {
                KinAlloc::GenericCartesian(stepper) => {
                    stepper.set_coeffs(c0, c1, c2);
                    true
                }
                _ => false,
            });
            if !ok {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "stepper kinematics is not a generic_cartesian_stepper"
                        .to_string(),
                });
            }
        }
        Ok(())
    }

    /// `update_carriages(carriages_str, printer_carriages, report_error)`
    /// (`kinematic_stepper.py:81-90`). `report_error` is the caller
    /// supplied error factory (`gcmd.error` in
    /// `generic_cartesian.py:370`); when it is `None` the parse errors
    /// are `printer.command_error`.
    pub fn update_carriages(
        &self,
        carriages_str: &str,
        printer_carriages: &IndexMap<String, Carriage>,
        report_error: Option<&dyn Fn(String) -> KlippyError>,
    ) -> KResult<()> {
        let default_error = |msg: String| -> KlippyError { KlippyError::Command(msg) };
        let parse_error: &dyn Fn(String) -> KlippyError = match report_error {
            Some(f) => f,
            None => &default_error,
        };
        let (kin_coeffs, carriages) =
            parse_carriages_string(carriages_str, printer_carriages, parse_error)?;
        if let Some(f) = report_error {
            if !kin_coeffs.iter().any(|c| *c != 0.0) {
                return Err(f(format!(
                    "A valid string that references at least one carriage \
                     must be provided for '{}'",
                    self.get_name(false)
                )));
            }
        }
        self.carriages.replace(carriages);
        self.update_kin_coeffs(kin_coeffs)
    }

    /// `set_position(coord)` (`kinematic_stepper.py:91-92`).
    pub fn set_position(&self, coord: [f64; 3]) {
        self.stepper.set_position(coord);
    }
}

/// This module is not a printer object module: the Python file defines
/// no `load_config` / `load_config_prefix` / `add_printer_objects`
/// entry points (`GenericCartesianKinematics` constructs
/// `KinematicStepper` directly), so the factory has no entry points.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
