//! Port of the `homing_heaters` module (`klippy/extras/homing_heaters.py`).
//!
//! Temporarily disables configured heaters around homing moves when the move
//! involves configured "flaky" steppers.
//!
//! The `heaters` module owns `PrinterHeaters` and `Heater` options; this
//! module reaches them through the `crate::heaters` interfaces
//! ([`PrinterHeatersApi`] and [`HeaterApi`]).

use crate::heaters::{HeaterApi, PrinterHeatersApi};

use indexmap::IndexMap;
use klippy_api::config::{Config, GetListOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::Endstop;
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_HOMING_MOVE_BEGIN, EV_HOMING_MOVE_END,
    EV_KLIPPY_CONNECT,
};
use klippy_api::toolhead::{HomingMoveApi, HomingMoveHandle, ToolHeadApi};
use klippy_api::{load_interface, lookup_interface, EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use std::sync::Arc;


/// `klippy/extras/homing_heaters.py:HomingHeaters`.
pub struct HomingHeaters {
    printer: Printer,
    disable_heaters: GCell<Option<Vec<String>>>,
    flaky_steppers: GCell<Option<Vec<String>>>,
    pheaters: Arc<dyn PrinterHeatersApi>,
    target_save: GCell<IndexMap<String, f64>>,
}

impl HomingHeaters {
    /// `HomingHeaters.__init__(config)` (`homing_heaters.py:10`).
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let heaters_opts = GetListOpts {
            default: Some(None),
            ..Default::default()
        };
        let disable_heaters = config.getlist("heaters", heaters_opts)?;
        let steppers_opts = GetListOpts {
            default: Some(None),
            ..Default::default()
        };
        let flaky_steppers = config.getlist("steppers", steppers_opts)?;
        let pheaters = load_interface::<dyn PrinterHeatersApi>(
            &*printer,
            config,
            "heaters",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let obj = Arc::new(HomingHeaters {
            printer: printer.clone(),
            disable_heaters: GCell::new(disable_heaters),
            flaky_steppers: GCell::new(flaky_steppers),
            pheaters,
            target_save: GCell::new(IndexMap::new()),
        });
        let this = obj.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_| {
                this.handle_connect()?;
                Ok(PyValue::None)
            }),
        );
        let this = obj.clone();
        printer.register_event_handler(
            EV_HOMING_MOVE_BEGIN,
            Box::new(move |params| {
                let hmove = homing_move(params)?;
                this.handle_homing_move_begin(&*hmove)?;
                Ok(PyValue::None)
            }),
        );
        let this = obj.clone();
        printer.register_event_handler(
            EV_HOMING_MOVE_END,
            Box::new(move |params| {
                let hmove = homing_move(params)?;
                this.handle_homing_move_end(&*hmove)?;
                Ok(PyValue::None)
            }),
        );
        Ok(obj)
    }

    /// `HomingHeaters.handle_connect()` (`homing_heaters.py:23`).
    pub fn handle_connect(&self) -> KResult<()> {
        // heaters to disable
        let all_heaters = self.pheaters.get_all_heaters();
        match self.disable_heaters.cloned() {
            None => {
                *self.disable_heaters.borrow_mut() = Some(all_heaters.clone());
            }
            Some(disable) => {
                if !disable.iter().all(|x| all_heaters.contains(x)) {
                    return Err(KlippyError::config(format!(
                        "One or more of these heaters are unknown: {}",
                        list_str(&disable)
                    )));
                }
            }
        }
        // steppers valid?
        let toolhead = lookup_interface::<dyn ToolHeadApi>(&*self.printer, "toolhead", LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let kin = toolhead.get_kinematics();
        let all_steppers: Vec<String> =
            kin.get_steppers().iter().map(|s| s.get_name(None)).collect();
        let flaky = self.flaky_steppers.cloned();
        let Some(flaky) = flaky else {
            return Ok(());
        };
        if !flaky.iter().all(|x| all_steppers.contains(x)) {
            return Err(KlippyError::config(format!(
                "One or more of these steppers are unknown: {}",
                list_str(&flaky)
            )));
        }
        Ok(())
    }

    /// `HomingHeaters.check_eligible(endstops)` (`homing_heaters.py:42`).
    pub fn check_eligible(&self, endstops: &[Endstop]) -> bool {
        let Some(flaky) = self.flaky_steppers.cloned() else {
            return true;
        };
        let steppers_being_homed: Vec<String> = endstops
            .iter()
            .flat_map(|es| es.get_steppers())
            .map(|s| s.get_name(None))
            .collect();
        steppers_being_homed.iter().any(|x| flaky.contains(x))
    }

    /// `HomingHeaters.handle_homing_move_begin(hmove)` (`homing_heaters.py:49`).
    pub fn handle_homing_move_begin(&self, hmove: &dyn HomingMoveApi) -> KResult<()> {
        if !self.check_eligible(&hmove.get_mcu_endstops()) {
            return Ok(());
        }
        let Some(disable) = self.disable_heaters.cloned() else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not iterable".to_string(),
            });
        };
        for heater_name in disable {
            let heater = self.pheaters.lookup_heater(&heater_name)?;
            let target = heater.get_temp(0.0).1;
            self.target_save
                .borrow_mut()
                .insert(heater_name.clone(), target);
            heater.set_temp(0.0)?;
        }
        Ok(())
    }

    /// `HomingHeaters.handle_homing_move_end(hmove)` (`homing_heaters.py:56`).
    pub fn handle_homing_move_end(&self, hmove: &dyn HomingMoveApi) -> KResult<()> {
        if !self.check_eligible(&hmove.get_mcu_endstops()) {
            return Ok(());
        }
        let Some(disable) = self.disable_heaters.cloned() else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not iterable".to_string(),
            });
        };
        for heater_name in &disable {
            let heater = self.pheaters.lookup_heater(heater_name)?;
            let target = match self.target_save.borrow().get(heater_name) {
                Some(&t) => t,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: pycompat::text::str_repr(heater_name),
                    });
                }
            };
            heater.set_temp(target)?;
        }
        Ok(())
    }
}

/// `"%s" % (names,)` for a list of strings: the Python `str(list)`, i.e.
/// the `repr` of each element joined into a list literal.
fn list_str(names: &[String]) -> String {
    PyValue::List(names.iter().map(|s| PyValue::Str(s.clone())).collect()).str()
}

/// Recover the `HomingMove` payload of a `homing:homing_move_*` event.
fn homing_move(params: &[EventArg]) -> KResult<HomingMoveHandle> {
    params
        .first()
        .and_then(|a| a.as_handle::<dyn HomingMoveApi>())
        .ok_or_else(|| KlippyError::Internal {
            class_name: "TypeError".to_string(),
            message: "expected HomingMove event payload".to_string(),
        })
}

klippy_api::printer_object!(HomingHeaters);

/// `load_config(config)` (`homing_heaters.py:63`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(HomingHeaters::new(config)? as ObjectRef))
}

/// Module entry point for the `homing_heaters` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
