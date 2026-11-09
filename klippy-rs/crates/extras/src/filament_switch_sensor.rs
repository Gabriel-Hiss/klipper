//! Port of `klippy/extras/filament_switch_sensor.py`.
//!
//! The module holds `RunoutHelper` (the shared runout/insert state machine
//! used by the filament sensors) and `SwitchSensor` (loaded by
//! `load_config_prefix`). Behaviour follows the Python reference line by
//! line: config reads, the `klippy:ready` settle time, `QUERY_FILAMENT_SENSOR`
//! / `SET_FILAMENT_SENSOR` mux commands, `run_script` composition and log
//! lines.
//!
//! Objects reached by printer name are resolved with `lookup_interface` /
//! `load_interface` against the v2 contracts: `gcode` is the core
//! `dyn GCodeApi` service, `buttons` is `crate::buttons::PrinterButtonsApi`,
//! `pause_resume` is `crate::pause_resume::PauseResumeApi`, and templates
//! come from `crate::gcode_macro::PrinterGCodeMacroApi` exactly as the
//! reference loads `gcode_macro`.

use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use reactor::Reactor;

use crate::buttons::{ButtonState, DebounceCallback, PrinterButtonsApi};
use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use crate::pause_resume::PauseResumeApi;
use klippy_api::config::{Config, GetBoolOpts, GetFloatOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeApi, GCodeCommandHandler, GCmd, GetIntOpts};
use klippy_api::gcell::GCell;
use klippy_api::log;
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer};
use klippy_api::{load_interface, lookup_interface, ObjectRef, PrinterObject};

/// `cmd_QUERY_FILAMENT_SENSOR_help` (`filament_switch_sensor.py:97`).
pub const QUERY_HELP: &str = "Query the status of the Filament Sensor";
/// `cmd_SET_FILAMENT_SENSOR_help` (`filament_switch_sensor.py:104`).
pub const SET_HELP: &str = "Sets the filament sensor on/off";
/// Fixed settle time in `_handle_ready` (`filament_switch_sensor.py:44`).
const READY_SETTLE: f64 = 2.;


// ----------------------------------------------------------------------
// RunoutHelper (`filament_switch_sensor.py:9`).
// ----------------------------------------------------------------------

struct Inner {
    name: String,
    printer: Printer,
    reactor: Reactor,
    gcode: GCode,
    runout_pause: bool,
    runout_gcode: Option<TemplateWrapper>,
    insert_gcode: Option<TemplateWrapper>,
    pause_delay: f64,
    event_delay: f64,
    min_event_systime: GCell<f64>,
    filament_present: GCell<bool>,
    sensor_enabled: GCell<bool>,
}

/// Shared runout helper used by every filament sensor.
pub struct RunoutHelper {
    inner: Arc<Inner>,
}

impl RunoutHelper {
    /// `RunoutHelper.__init__` (`filament_switch_sensor.py:10-42`).
    pub fn new(config: &Config) -> KResult<RunoutHelper> {
        let name = config
            .get_name()
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let runout_pause = config
            .getboolean(
                "pause_on_runout",
                GetBoolOpts { default: Some(Some(true)), ..GetBoolOpts::default() },
            )?
            .unwrap_or(true);
        if runout_pause {
            printer.load_object(config, "pause_resume", LookupDefault::Raise)?;
        }
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let has_runout = config
            .get("runout_gcode", GetStrOpts { default: Some(None), ..GetStrOpts::default() })?
            .is_some();
        let runout_gcode = if runout_pause || has_runout {
            Some(gcode_macro.load_template(config, "runout_gcode", Some(""))?)
        } else {
            None
        };
        let has_insert = config
            .get("insert_gcode", GetStrOpts { default: Some(None), ..GetStrOpts::default() })?
            .is_some();
        let insert_gcode = if has_insert {
            Some(gcode_macro.load_template(config, "insert_gcode", None)?)
        } else {
            None
        };
        let pause_delay = config
            .getfloat(
                "pause_delay",
                GetFloatOpts {
                    default: Some(Some(0.5)),
                    above: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.5);
        let event_delay = config
            .getfloat(
                "event_delay",
                GetFloatOpts {
                    default: Some(Some(3.0)),
                    minval: Some(0.0),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(3.0);
        let inner = Arc::new(Inner {
            name,
            printer: printer.clone(),
            reactor,
            gcode,
            runout_pause,
            runout_gcode,
            insert_gcode,
            pause_delay,
            event_delay,
            min_event_systime: GCell::new(reactor::NEVER),
            filament_present: GCell::new(false),
            sensor_enabled: GCell::new(true),
        });
        {
            let inner = inner.clone();
            let cb: EventCallback = Box::new(move |_| {
                inner.handle_ready();
                Ok(PyValue::None)
            });
            printer.register_event_handler("klippy:ready", cb);
        }
        {
            let inner = inner.clone();
            let name = inner.name.clone();
            let handler_inner = inner.clone();
            let handler: GCodeCommandHandler = Box::new(move |gcmd| {
                handler_inner.cmd_query_filament_sensor(gcmd)
            });
            inner.gcode.register_mux_command(
                "QUERY_FILAMENT_SENSOR",
                "SENSOR",
                Some(&name),
                handler,
                Some(QUERY_HELP),
            )?;
        }
        {
            let inner = inner.clone();
            let name = inner.name.clone();
            let handler_inner = inner.clone();
            let handler: GCodeCommandHandler = Box::new(move |gcmd| {
                handler_inner.cmd_set_filament_sensor(gcmd)
            });
            inner.gcode.register_mux_command(
                "SET_FILAMENT_SENSOR",
                "SENSOR",
                Some(&name),
                handler,
                Some(SET_HELP),
            )?;
        }
        Ok(RunoutHelper { inner })
    }

    /// Number of the helper (`self.name`).
    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// `filament_present` state.
    pub fn filament_present(&self) -> bool {
        self.inner.filament_present.get()
    }

    /// `sensor_enabled` state.
    pub fn sensor_enabled(&self) -> bool {
        self.inner.sensor_enabled.get()
    }

    /// Sets `sensor_enabled` (`cmd_SET_FILAMENT_SENSOR` assignment).
    pub fn set_sensor_enabled(&self, enabled: bool) {
        self.inner.sensor_enabled.set(enabled);
    }

    /// `note_filament_present` (`filament_switch_sensor.py:63-92`).
    pub fn note_filament_present(
        &self,
        eventtime: f64,
        is_filament_present: bool,
    ) -> KResult<()> {
        self.inner
            .note_filament_present(&self.inner, eventtime, is_filament_present)
    }

    /// Direct `_runout_event_handler` call (`filament_switch_sensor.py:45`).
    pub fn runout_event_handler(&self, eventtime: f64) {
        self.inner.runout_event_handler(eventtime);
    }

    /// Direct `_insert_event_handler` call (`filament_switch_sensor.py:55`).
    pub fn insert_event_handler(&self, eventtime: f64) {
        self.inner.insert_event_handler(eventtime);
    }

    /// `cmd_QUERY_FILAMENT_SENSOR` (`filament_switch_sensor.py:98`).
    pub fn cmd_query_filament_sensor(&self, gcmd: &GCmd) -> KResult<()> {
        self.inner.cmd_query_filament_sensor(gcmd)
    }

    /// `cmd_SET_FILAMENT_SENSOR` (`filament_switch_sensor.py:105`).
    pub fn cmd_set_filament_sensor(&self, gcmd: &GCmd) -> KResult<()> {
        self.inner.cmd_set_filament_sensor(gcmd)
    }

    /// `get_status` (`filament_switch_sensor.py:93`).
    pub fn get_status(&self, _eventtime: f64) -> PyValue {
        self.inner.get_status()
    }
}

impl Inner {
    fn handle_ready(&self) {
        let v = self.reactor.monotonic() + READY_SETTLE;
        self.min_event_systime.set(v);
    }

    fn runout_event_handler(&self, eventtime: f64) {
        let mut pause_prefix = "";
        if self.runout_pause {
            match lookup_interface::<dyn PauseResumeApi>(
                &*self.printer,
                "pause_resume",
                LookupDefault::Raise,
            ) {
                Ok(Some(pause)) => {
                    if let Err(e) = pause.send_pause_command() {
                        log::exception(&format!("{e}"), &e.exception_line());
                    }
                }
                Ok(None) => {}
                Err(e) => {
                    log::exception(&format!("{e}"), &e.exception_line());
                }
            }
            pause_prefix = "PAUSE\n";
            // `self.printer.get_reactor().pause(eventtime + pause_delay)`
            self.printer.get_reactor().pause(eventtime + self.pause_delay);
        }
        self.exec_gcode(pause_prefix, self.runout_gcode.as_ref());
    }

    fn insert_event_handler(&self, eventtime: f64) {
        let _ = eventtime;
        self.exec_gcode("", self.insert_gcode.as_ref());
    }

    fn exec_gcode(&self, prefix: &str, template: Option<&TemplateWrapper>) {
        let rendered = match template {
            Some(t) => t.render(None),
            None => Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'render'".to_string(),
            }),
        };
        let result = match rendered {
            Ok(script) => self.gcode.run_script(&format!("{prefix}{script}\nM400")),
            Err(e) => Err(e),
        };
        if let Err(e) = result {
            log::exception("Script running error", &e.exception_line());
        }
        let v = self.reactor.monotonic() + self.event_delay;
        self.min_event_systime.set(v);
    }

    /// `idle_timeout.get_status(now)["state"] == "Printing"`
    /// (`filament_switch_sensor.py:75-76`).
    fn is_printing(&self, now: f64) -> KResult<bool> {
        let obj = self
            .printer
            .lookup_object("idle_timeout", LookupDefault::Raise)?
            .expect("Raise never yields None");
        let status = obj.get_status(now)?.ok_or_else(|| KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "'idle_timeout' object has no attribute 'get_status'".to_string(),
        })?;
        match status {
            PyValue::Dict(d) => match d.get(&PyValue::Str("state".to_string())) {
                Some(PyValue::Str(s)) => Ok(s == "Printing"),
                Some(_) => Ok(false),
                None => Err(KlippyError::Internal {
                    class_name: "KeyError".to_string(),
                    message: "'state'".to_string(),
                }),
            },
            other => Err(KlippyError::Internal {
                class_name: "TypeError".to_string(),
                message: format!(
                    "'{}' object is not subscriptable",
                    pycompat::value::type_name(&other)
                ),
            }),
        }
    }

    fn note_filament_present(
        &self,
        self_arc: &Arc<Inner>,
        eventtime: f64,
        is_filament_present: bool,
    ) -> KResult<()> {
        if is_filament_present == self.filament_present.get() {
            return Ok(());
        }
        self.filament_present.set(is_filament_present);
        if eventtime < self.min_event_systime.get() || !self.sensor_enabled.get() {
            return Ok(());
        }
        let now = self.reactor.monotonic();
        let is_printing = self.is_printing(now)?;
        if is_filament_present {
            if !is_printing && self.insert_gcode.is_some() {
                // insert detected
                self.min_event_systime.set(reactor::NEVER);
                let msg = percent_format(
                    "Filament Sensor %s: insert event detected, Time %.2f",
                    &[PctValue::Str(self.name.clone()), PctValue::Float(now)],
                )
                .unwrap_or_else(|_| {
                    format!("Filament Sensor {}: insert event detected, Time {now}", self.name)
                });
                log::info(&msg);
                let inner = self_arc.clone();
                let reactor = inner.reactor.clone();
                reactor.register_callback(
                    move |et| {
                        inner.insert_event_handler(et);
                    },
                    reactor::NOW,
                );
            }
        } else if is_printing && self.runout_gcode.is_some() {
            // runout detected
            self.min_event_systime.set(reactor::NEVER);
            let msg = percent_format(
                "Filament Sensor %s: runout event detected, Time %.2f",
                &[PctValue::Str(self.name.clone()), PctValue::Float(now)],
            )
            .unwrap_or_else(|_| {
                format!("Filament Sensor {}: runout event detected, Time {now}", self.name)
            });
            log::info(&msg);
            let inner = self_arc.clone();
            let reactor = inner.reactor.clone();
            reactor.register_callback(
                move |et| {
                    inner.runout_event_handler(et);
                },
                reactor::NOW,
            );
        }
        Ok(())
    }

    fn cmd_query_filament_sensor(&self, gcmd: &GCmd) -> KResult<()> {
        let msg = if self.filament_present.get() {
            percent_format(
                "Filament Sensor %s: filament detected",
                &[PctValue::Str(self.name.clone())],
            )
            .unwrap_or_else(|_| format!("Filament Sensor {}: filament detected", self.name))
        } else {
            percent_format(
                "Filament Sensor %s: filament not detected",
                &[PctValue::Str(self.name.clone())],
            )
            .unwrap_or_else(|_| {
                format!("Filament Sensor {}: filament not detected", self.name)
            })
        };
        gcmd.respond_info(&msg, true);
        Ok(())
    }

    fn cmd_set_filament_sensor(&self, gcmd: &GCmd) -> KResult<()> {
        let value = gcmd
            .get_int(
                "ENABLE",
                GetIntOpts {
                    default: Some(Some(1)),
                    ..GetIntOpts::default()
                },
            )?
            .unwrap_or(1);
        self.sensor_enabled.set(value != 0);
        Ok(())
    }

    fn get_status(&self) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("filament_detected".to_string()),
            PyValue::Bool(self.filament_present.get()),
        );
        d.insert(
            PyValue::Str("enabled".to_string()),
            PyValue::Bool(self.sensor_enabled.get()),
        );
        PyValue::Dict(d)
    }
}

// ----------------------------------------------------------------------
// SwitchSensor (`filament_switch_sensor.py:108`).
// ----------------------------------------------------------------------

/// Switch-based filament sensor (`load_config_prefix`).
pub struct SwitchSensor {
    runout_helper: Arc<RunoutHelper>,
}

impl SwitchSensor {
    /// `SwitchSensor.__init__` (`filament_switch_sensor.py:109-116`).
    pub fn new(config: &Config) -> KResult<SwitchSensor> {
        let printer = config.get_printer();
        let buttons = load_interface::<dyn PrinterButtonsApi>(
            &*printer,
            config,
            "buttons",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let switch_pin = config
            .get(
                "switch_pin",
                GetStrOpts { default: None, ..GetStrOpts::default() },
            )?
            .expect("required option returns a value");
        let helper = Arc::new(RunoutHelper::new(config)?);
        {
            let h = helper.clone();
            let cb: DebounceCallback = Arc::new(move |eventtime, state| {
                let present = match state {
                    ButtonState::Empty => false,
                    ButtonState::Analog(pressed) => pressed,
                    ButtonState::Digital(value) => value != 0,
                };
                let _ = h.note_filament_present(eventtime, present);
            });
            buttons.register_debounce_button(&switch_pin, cb, config)?;
        }
        Ok(SwitchSensor { runout_helper: helper })
    }

    /// The shared runout helper.
    pub fn runout_helper(&self) -> &Arc<RunoutHelper> {
        &self.runout_helper
    }

    /// `SwitchSensor._button_handler` (`filament_switch_sensor.py:117`).
    pub fn button_handler(&self, eventtime: f64, state: bool) {
        let _ = self.runout_helper.note_filament_present(eventtime, state);
    }

    /// `get_status` (delegates to `runout_helper.get_status`).
    pub fn get_status(&self, eventtime: f64) -> PyValue {
        self.runout_helper.get_status(eventtime)
    }
}

impl PrinterObject for SwitchSensor {
    klippy_api::printer_object_base!();
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.get_status(eventtime)))
    }
}

/// `load_config_prefix` (`filament_switch_sensor.py:120`).
pub fn load_config_prefix(config: &Config) -> KResult<Option<ObjectRef>> {
    let sensor = SwitchSensor::new(config)?;
    Ok(Some(Arc::new(sensor) as ObjectRef))
}

/// Module entry point for the printer loader.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: None,
        load_config_prefix: Some(load_config_prefix),
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
