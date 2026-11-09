//! Port of the `filament_motion_sensor` module
//! (`klippy/extras/filament_motion_sensor.py`).
//!
//! An encoder-based filament sensor. Button events from `switch_pin`
//! are treated as filament motion; a timer checks the extruder
//! position against the position it last saw moving.

use std::sync::Arc;

use klippy_api::config::{Config, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu;
use klippy_api::object::{
    interface, load_interface, missing_interface, EventArg, ObjectRef, PrinterObject,
};
use klippy_api::printer::{
    LookupDefault, ModuleFactory, Printer, EV_IDLE_TIMEOUT_IDLE, EV_IDLE_TIMEOUT_PRINTING,
    EV_IDLE_TIMEOUT_READY, EV_KLIPPY_READY,
};
use klippy_api::toolhead::{
    ExtruderStepper, ExtruderStepperApi, PrinterExtruder, PrinterExtruderApi,
};
use klippy_api::GCell;
use pycompat::PyValue;
use reactor::{Reactor, Timer, NEVER, NOW};

use crate::buttons::{ButtonCallback, PrinterButtonsApi};
use crate::filament_switch_sensor::RunoutHelper;

/// `klippy/extras/filament_motion_sensor.py:CHECK_RUNOUT_TIMEOUT`.
pub const CHECK_RUNOUT_TIMEOUT: f64 = 0.250;

/// The extruder-like object `filament_motion_sensor` checks for motion
/// (`klippy/extras/filament_motion_sensor.py:46,62`). The config
/// `extruder` name can point at a `PrinterExtruder` or an
/// `ExtruderStepper`; Python resolves them by duck typing, so the port
/// keeps both shapes behind one dispatch.
#[derive(Clone)]
enum ExtruderRef {
    PrinterExtruder(PrinterExtruder),
    ExtruderStepper(ExtruderStepper),
}

impl ExtruderRef {
    fn find_past_position(&self, print_time: f64) -> KResult<f64> {
        match self {
            ExtruderRef::PrinterExtruder(extruder) => {
                extruder.find_past_position(print_time)
            }
            ExtruderRef::ExtruderStepper(stepper) => {
                stepper.find_past_position(print_time)
            }
        }
    }
}

/// Mutable state of the sensor after construction; the config and
/// helper are set during construction and never change afterwards.
struct SensorState {
    /// `self.runout_helper` (`filament_motion_sensor.py:24`).
    runout_helper: Option<RunoutHelper>,
    /// `self.extruder` (`filament_motion_sensor.py:46`); not yet
    /// resolved until `klippy:ready`.
    extruder: Option<ExtruderRef>,
    /// `self.estimated_print_time` (`filament_motion_sensor.py:47`);
    /// the `mcu` object's `estimated_print_time` bound method, not yet
    /// resolved until `klippy:ready`.
    mcu: Option<mcu::Mcu>,
    /// `self.filament_runout_pos` (`filament_motion_sensor.py:29`).
    filament_runout_pos: Option<f64>,
    /// `self._extruder_pos_update_timer` (`filament_motion_sensor.py:50`).
    extruder_pos_update_timer: Option<Timer>,
}

/// The `EncoderSensor` object
/// (`klippy/extras/filament_motion_sensor.py:11`).
pub struct EncoderSensor {
    printer: Printer,
    reactor: Reactor,
    /// `self.extruder_name` (`filament_motion_sensor.py:16`).
    extruder_name: String,
    /// `self.detection_length` (`filament_motion_sensor.py:17`).
    detection_length: f64,
    state: GCell<SensorState>,
}

impl EncoderSensor {
    /// `EncoderSensor.__init__(config)`
    /// (`klippy/extras/filament_motion_sensor.py:12`).
    ///
    /// Two-phase: the button registration (`__init__` line 21) and the
    /// event handlers (lines 31-38) capture the sensor as an `Arc`,
    /// so the object is created first and then initialised on the
    /// returned `Arc`. The initialisation order matches the Python
    /// `__init__` body, so config errors and the buttons load keep
    /// their precedence.
    pub fn new(config: &Config) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let switch_pin = config
            .get("switch_pin", GetStrOpts::default())?
            .expect("required option never returns None");
        let extruder_name = config
            .get("extruder", GetStrOpts::default())?
            .expect("required option never returns None");
        let detection_length = config
            .getfloat(
                "detection_length",
                GetFloatOpts {
                    default: Some(Some(7.0)),
                    above: Some(0.0),
                    ..Default::default()
                },
            )?
            .expect("default option never returns None");
        let reactor = printer.get_reactor();
        let sensor = Arc::new(EncoderSensor {
            printer,
            reactor,
            extruder_name,
            detection_length,
            state: GCell::new(SensorState {
                runout_helper: None,
                extruder: None,
                mcu: None,
                filament_runout_pos: None,
                extruder_pos_update_timer: None,
            }),
        });
        sensor.init(config, &switch_pin)?;
        Ok(sensor)
    }

    /// Body of `__init__` that needs the `Arc`: buttons (lines 20-21),
    /// the runout helper (line 24) and the event handlers (lines
    /// 31-38), in that order.
    fn init(self: &Arc<Self>, config: &Config, switch_pin: &str) -> KResult<()> {
        let buttons = load_interface::<dyn PrinterButtonsApi>(
            &*self.printer,
            config,
            "buttons",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let sensor = Arc::clone(self);
        let callback: ButtonCallback = Arc::new(move |eventtime: f64, state: i64| {
            let _ = sensor.encoder_event(eventtime, state != 0);
        });
        buttons.register_buttons(vec![switch_pin.to_string()], callback)?;
        let runout_helper = RunoutHelper::new(config)?;
        self.state.borrow_mut().runout_helper = Some(runout_helper);

        let ready = Arc::clone(self);
        self.printer.register_event_handler(
            EV_KLIPPY_READY,
            Box::new(move |_params: &[EventArg]| {
                ready.handle_ready()?;
                Ok(PyValue::None)
            }),
        );
        let printing = Arc::clone(self);
        self.printer.register_event_handler(
            EV_IDLE_TIMEOUT_PRINTING,
            Box::new(move |_params: &[EventArg]| {
                printing.handle_printing()?;
                Ok(PyValue::None)
            }),
        );
        let not_printing = Arc::clone(self);
        self.printer.register_event_handler(
            EV_IDLE_TIMEOUT_READY,
            Box::new(move |_params: &[EventArg]| {
                not_printing.handle_not_printing()?;
                Ok(PyValue::None)
            }),
        );
        let idle = Arc::clone(self);
        self.printer.register_event_handler(
            EV_IDLE_TIMEOUT_IDLE,
            Box::new(move |_params: &[EventArg]| {
                idle.handle_not_printing()?;
                Ok(PyValue::None)
            }),
        );
        Ok(())
    }

    /// `self._update_filament_runout_pos(eventtime=None)`
    /// (`filament_motion_sensor.py:39`).
    pub fn update_filament_runout_pos(&self, eventtime: Option<f64>) -> KResult<()> {
        let eventtime = eventtime.unwrap_or_else(|| self.reactor.monotonic());
        let extruder_pos = self.get_extruder_pos(Some(eventtime))?;
        self.state.borrow_mut().filament_runout_pos =
            Some(extruder_pos + self.detection_length);
        Ok(())
    }

    /// `self._get_extruder_pos(eventtime=None)`
    /// (`filament_motion_sensor.py:58`).
    pub fn get_extruder_pos(&self, eventtime: Option<f64>) -> KResult<f64> {
        let eventtime = eventtime.unwrap_or_else(|| self.reactor.monotonic());
        // Python calls `self.estimated_print_time(eventtime)` first; when
        // it is still None that is a TypeError, and when the call
        // succeeds `self.extruder.find_past_position` may still raise an
        // AttributeError. Reproduce that order.
        let mcu = self.state.borrow().mcu.clone();
        let Some(mcu) = mcu else {
            return Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: "'NoneType' object is not callable".to_string(),
            });
        };
        let print_time = mcu.estimated_print_time(eventtime);
        let extruder = self.state.borrow().extruder.clone();
        let Some(extruder) = extruder else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'find_past_position'"
                    .to_string(),
            });
        };
        extruder.find_past_position(print_time)
    }

    /// `self._handle_ready()` (`filament_motion_sensor.py:45`).
    pub fn handle_ready(self: &Arc<Self>) -> KResult<()> {
        // Python resolves the extruder object and the `mcu` object
        // (lines 46-48) before any `find_past_position` call inside
        // `_update_filament_runout_pos` (line 49), so a missing `mcu`
        // raises `config_error` before an extruder missing the
        // interface raises `AttributeError`.
        let obj = self
            .printer
            .lookup_object(&self.extruder_name, LookupDefault::Raise)?
            .expect("LookupDefault::Raise never yields None");
        let mcu = mcu::get_printer_mcu(&self.printer, "mcu")?;
        let extruder = resolve_extruder(&obj, &self.extruder_name)?;
        {
            let mut state = self.state.borrow_mut();
            state.extruder = Some(extruder);
            state.mcu = Some(mcu);
        }
        self.update_filament_runout_pos(None)?;
        let timer_sensor = Arc::clone(self);
        let timer = self.reactor.register_timer(
            move |eventtime: f64| timer_sensor.extruder_pos_update_event(eventtime),
            NEVER,
        );
        self.state.borrow_mut().extruder_pos_update_timer = Some(timer);
        Ok(())
    }

    /// `self._handle_printing(print_time)`
    /// (`filament_motion_sensor.py:52`).
    pub fn handle_printing(&self) -> KResult<()> {
        // `print_time` is unused by the Python handler.
        // The state borrow is held through `update_timer`, which does
        // not pause the reactor and never calls back into the sensor.
        let state = self.state.borrow();
        if state.extruder_pos_update_timer.is_none() {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message:
                    "'EncoderSensor' object has no attribute '_extruder_pos_update_timer'"
                        .to_string(),
            });
        }
        let timer = state.extruder_pos_update_timer.as_ref().unwrap();
        self.reactor.update_timer(timer, NOW);
        Ok(())
    }

    /// `self._handle_not_printing(print_time)`
    /// (`filament_motion_sensor.py:55`).
    pub fn handle_not_printing(&self) -> KResult<()> {
        // `print_time` is unused by the Python handler.
        // The state borrow is held through `update_timer`, which does
        // not pause the reactor and never calls back into the sensor.
        let state = self.state.borrow();
        if state.extruder_pos_update_timer.is_none() {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message:
                    "'EncoderSensor' object has no attribute '_extruder_pos_update_timer'"
                        .to_string(),
            });
        }
        let timer = state.extruder_pos_update_timer.as_ref().unwrap();
        self.reactor.update_timer(timer, NEVER);
        Ok(())
    }

    /// `self._extruder_pos_update_event(eventtime)`
    /// (`filament_motion_sensor.py:63`).
    pub fn extruder_pos_update_event(&self, eventtime: f64) -> f64 {
        // `find_past_position` and `estimated_print_time` have no error
        // path for a ready printer; when Python raises there the
        // exception is uncaught in the reactor dispatch, so the Rust
        // equivalent is an explicit panic.
        let extruder_pos = self
            .get_extruder_pos(Some(eventtime))
            .expect("find_past_position cannot fail on a ready printer");
        let filament_runout_pos = self
            .state
            .borrow()
            .filament_runout_pos
            .expect("filament_runout_pos is set by _handle_ready before the timer runs");
        self.state
            .borrow()
            .runout_helper
            .as_ref()
            .expect("runout_helper is set during __init__")
            .note_filament_present(eventtime, extruder_pos < filament_runout_pos)
            .expect("note_filament_present cannot fail on a ready printer");
        eventtime + CHECK_RUNOUT_TIMEOUT
    }

    /// `self.encoder_event(eventtime, state)`
    /// (`filament_motion_sensor.py:69`).
    pub fn encoder_event(&self, eventtime: f64, _state: bool) -> KResult<()> {
        if self.state.borrow().extruder.is_none() {
            return Ok(());
        }
        self.update_filament_runout_pos(Some(eventtime))?;
        // Filament is always assumed to be present on an encoder event.
        self.state
            .borrow()
            .runout_helper
            .as_ref()
            .expect("runout_helper is set during __init__")
            .note_filament_present(eventtime, true)?;
        Ok(())
    }

    /// The sensor's `get_status` is the runout helper's method
    /// (`filament_motion_sensor.py:25`).
    pub fn get_sensor_status(&self, eventtime: f64) -> PyValue {
        self.state
            .borrow()
            .runout_helper
            .as_ref()
            .expect("runout_helper is set during __init__")
            .get_status(eventtime)
    }
}

impl PrinterObject for EncoderSensor {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!();

    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_sensor_status(eventtime)))
    }
}

/// `printer.lookup_object(self.extruder_name)` resolved to the extruder
/// shape with `find_past_position` (`filament_motion_sensor.py:46,62`).
fn resolve_extruder(obj: &ObjectRef, name: &str) -> KResult<ExtruderRef> {
    if let Some(extruder) = interface::<dyn PrinterExtruderApi>(obj) {
        return Ok(ExtruderRef::PrinterExtruder(extruder));
    }
    if let Some(stepper) = interface::<dyn ExtruderStepperApi>(obj) {
        return Ok(ExtruderRef::ExtruderStepper(stepper));
    }
    Err(missing_interface::<dyn PrinterExtruderApi>(name))
}

/// `load_config_prefix(config)` (`filament_motion_sensor.py:76`).
fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(EncoderSensor::new(config)? as ObjectRef))
}

/// Module entry point for the `filament_motion_sensor` sections.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
