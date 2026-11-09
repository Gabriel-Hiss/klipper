//! Port of the `heater_bed` module (`klippy/extras/heater_bed.py`).
//!
//! The bed heater is a plain [`crate::heaters`] `Heater` registered under
//! gcode id `B`: [`PrinterHeaterBed`] registers `M140` (set, no wait) and
//! `M190` (set and wait) and delegates `get_status` / `stats` to that
//! heater (`self.get_status = self.heater.get_status`).

use crate::heaters::{Heater, HeaterApi, PrinterHeatersApi};
use klippy_api::config::Config;
use klippy_api::error::KResult;
use klippy_api::gcode::{GCmd, GCodeApi, GetFloatOpts, RegisterCommandOpts};
use klippy_api::object::{load_interface, lookup_interface, ObjectRef};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::PrinterObject;
use pycompat::PyValue;
use std::sync::Arc;

/// `klippy/extras/heater_bed.py:PrinterHeaterBed`.
pub struct PrinterHeaterBed {
    printer: Printer,
    heater: Arc<Heater>,
}

/// Shared body of `cmd_M140(gcmd, wait)` (`heater_bed.py:18`): the `heaters`
/// object is looked up fresh on every command, exactly like Python.
fn cmd_set_bed_temperature(
    printer: &Printer,
    heater: &Arc<Heater>,
    gcmd: &GCmd,
    wait: bool,
) -> KResult<()> {
    let temp = gcmd.get_float(
        "S",
        GetFloatOpts {
            default: Some(Some(0.0)),
            ..Default::default()
        },
    )?
    .unwrap_or(0.0);
    let pheaters = lookup_interface::<dyn PrinterHeatersApi>(
        &**printer,
        "heaters",
        LookupDefault::Raise,
    )?
    .expect("Raise never yields None");
    pheaters.set_temperature(heater, temp, wait)
}

impl PrinterHeaterBed {
    /// `PrinterHeaterBed.__init__(config)` (`heater_bed.py:8`).
    pub fn new(config: &Config) -> KResult<Self> {
        let printer = config.get_printer();
        let pheaters = load_interface::<dyn PrinterHeatersApi>(
            &*printer,
            config,
            "heaters",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let heater = pheaters.setup_heater(config, Some("B".to_string()))?;
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let heater_m140 = heater.clone();
        let printer_m140 = printer.clone();
        gcode.register_command(
            "M140",
            Some(Box::new(move |gcmd: &GCmd| {
                cmd_set_bed_temperature(&printer_m140, &heater_m140, gcmd, false)
            })),
            RegisterCommandOpts::default(),
        )?;
        let heater_m190 = heater.clone();
        let printer_m190 = printer.clone();
        gcode.register_command(
            "M190",
            Some(Box::new(move |gcmd: &GCmd| {
                cmd_set_bed_temperature(&printer_m190, &heater_m190, gcmd, true)
            })),
            RegisterCommandOpts::default(),
        )?;
        Ok(PrinterHeaterBed { printer, heater })
    }

    /// `cmd_M140(gcmd, wait=False)` (`heater_bed.py:18`).
    pub fn cmd_M140(&self, gcmd: &GCmd, wait: bool) -> KResult<()> {
        cmd_set_bed_temperature(&self.printer, &self.heater, gcmd, wait)
    }

    /// `cmd_M190(gcmd)` (`heater_bed.py:23`): set the bed and wait.
    pub fn cmd_M190(&self, gcmd: &GCmd) -> KResult<()> {
        self.cmd_M140(gcmd, true)
    }
}

// `self.get_status = self.heater.get_status` / `self.stats = self.heater.stats`
// (`heater_bed.py:12-13`): delegation, not a snapshot. `Heater` also has these
// as `PrinterObject` methods, so the trait paths are named to stay unambiguous.
impl PrinterObject for PrinterHeaterBed {
    klippy_api::printer_object_base!();

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(HeaterApi::get_status(&*self.heater, eventtime)))
    }

    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(HeaterApi::stats(&*self.heater, eventtime))
    }
}

/// `load_config(config)` (`heater_bed.py:27`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PrinterHeaterBed::new(config)?) as ObjectRef))
}

/// Module entry point for the `heater_bed` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
