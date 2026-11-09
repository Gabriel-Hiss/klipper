//! Port of the `pause_resume` module (`klippy/extras/pause_resume.py`).

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCode, GCodeCommandHandler, GCmd, RegisterCommandOpts};
use klippy_api::mcu::McuApi;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_CONNECT};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback, WebRequest};
use klippy_api::{lookup_interface, GCell, ObjectRef, PrinterObject};
use pycompat::text::{percent_format, PctValue};
use pycompat::PyValue;
use std::sync::Arc;

use crate::virtual_sdcard::VirtualSDApi;

/// The `pause_resume` object service (`klippy/extras/pause_resume.py`):
/// the methods other extras duck-type against
/// (`filament_switch_sensor.py:51`, `palette2.py:248`).
pub trait PauseResumeApi: Send + Sync + 'static {
    /// `PauseResume.is_sd_active()` (`klippy/extras/pause_resume.py:56`).
    fn is_sd_active(&self) -> bool;
    /// `PauseResume.send_pause_command()` (`klippy/extras/pause_resume.py:58`).
    fn send_pause_command(&self) -> KResult<()>;
    /// `PauseResume.send_resume_command()` (`klippy/extras/pause_resume.py:79`).
    fn send_resume_command(&self) -> KResult<()>;
    /// `PauseResume.is_paused` (`klippy/extras/pause_resume.py:13`).
    fn is_paused(&self) -> bool;
}

/// Mutable `PauseResume` state (`klippy/extras/pause_resume.py:12-15`).
struct PauseState {
    is_paused: bool,
    sd_paused: bool,
    pause_command_sent: bool,
    v_sd: Option<Arc<dyn VirtualSDApi>>,
}

/// State shared with the registered G-code/webhook/event handlers.
struct Inner {
    printer: Printer,
    gcode: GCode,
    recover_velocity: f64,
    state: GCell<PauseState>,
}

impl Inner {
    fn is_paused(&self) -> bool {
        self.state.borrow().is_paused
    }

    fn is_sd_active(&self) -> bool {
        let v_sd = self.state.borrow().v_sd.clone();
        match v_sd {
            Some(v_sd) => v_sd.is_active(),
            None => false,
        }
    }

    fn handle_connect(&self) -> KResult<()> {
        let v_sd = lookup_interface::<dyn VirtualSDApi>(
            &*self.printer,
            "virtual_sdcard",
            LookupDefault::ReturnNone,
        )?;
        self.state.borrow_mut().v_sd = v_sd;
        Ok(())
    }

    fn send_pause_command(&self) -> KResult<()> {
        if self.state.borrow().pause_command_sent {
            return Ok(());
        }
        if self.is_sd_active() {
            self.state.borrow_mut().sd_paused = true;
            let v_sd = self.state.borrow().v_sd.clone();
            match v_sd {
                Some(v_sd) => v_sd.do_pause(),
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "AttributeError".to_string(),
                        message: "'NoneType' object has no attribute 'do_pause'"
                            .to_string(),
                    });
                }
            }
        } else {
            self.state.borrow_mut().sd_paused = false;
            self.gcode.respond_info("action:paused", true);
        }
        self.state.borrow_mut().pause_command_sent = true;
        Ok(())
    }

    fn send_resume_command(&self) -> KResult<()> {
        if self.state.borrow().sd_paused {
            let v_sd = self.state.borrow().v_sd.clone();
            match v_sd {
                Some(v_sd) => v_sd.do_resume()?,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "AttributeError".to_string(),
                        message: "'NoneType' object has no attribute 'do_resume'"
                            .to_string(),
                    });
                }
            }
            self.state.borrow_mut().sd_paused = false;
        } else {
            self.gcode.respond_info("action:resumed", true);
        }
        self.state.borrow_mut().pause_command_sent = false;
        Ok(())
    }

    fn cmd_pause(&self, gcmd: &GCmd) -> KResult<()> {
        if self.is_paused() {
            gcmd.respond_info("Print already paused", true);
            return Ok(());
        }
        self.send_pause_command()?;
        self.gcode.run_script_from_command("SAVE_GCODE_STATE NAME=PAUSE_STATE")?;
        self.state.borrow_mut().is_paused = true;
        Ok(())
    }

    fn cmd_resume(&self, gcmd: &GCmd) -> KResult<()> {
        if !self.is_paused() {
            gcmd.respond_info("Print is not paused, resume aborted", true);
            return Ok(());
        }
        let velocity = gcmd
            .get_float(
                "VELOCITY",
                klippy_api::gcode::GetFloatOpts {
                    default: Some(Some(self.recover_velocity)),
                    ..Default::default()
                },
            )?
            .unwrap_or(self.recover_velocity);
        let script = percent_format(
            "RESTORE_GCODE_STATE NAME=PAUSE_STATE MOVE=1 MOVE_SPEED=%.4f",
            &[PctValue::Float(velocity)],
        )
        .map_err(|e| KlippyError::Internal {
            class_name: e.kind.to_string(),
            message: e.msg.clone(),
        })?;
        self.gcode.run_script_from_command(&script)?;
        self.send_resume_command()?;
        self.state.borrow_mut().is_paused = false;
        Ok(())
    }

    fn cmd_clear_pause(&self) {
        let mut state = self.state.borrow_mut();
        state.is_paused = false;
        state.pause_command_sent = false;
    }

    fn cmd_cancel_print(&self, gcmd: &GCmd) -> KResult<()> {
        if self.is_sd_active() || self.state.borrow().sd_paused {
            let v_sd = self.state.borrow().v_sd.clone();
            match v_sd {
                Some(v_sd) => v_sd.do_cancel()?,
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "AttributeError".to_string(),
                        message: "'NoneType' object has no attribute 'do_cancel'"
                            .to_string(),
                    });
                }
            }
        } else {
            gcmd.respond_info("action:cancel", true);
        }
        self.cmd_clear_pause();
        Ok(())
    }

    fn get_status(&self) -> PyValue {
        let mut status = IndexMap::new();
        status.insert(
            PyValue::Str("is_paused".to_string()),
            PyValue::Bool(self.is_paused()),
        );
        PyValue::Dict(status)
    }
}

/// `PauseResume` (`klippy/extras/pause_resume.py:7`).
pub struct PauseResume {
    inner: Arc<Inner>,
}

impl PauseResume {
    /// `PauseResume.__init__(config)` (`klippy/extras/pause_resume.py:8`).
    pub fn new(config: &Config) -> KResult<PauseResume> {
        let printer = config.get_printer();
        let gcode = lookup_interface::<dyn klippy_api::gcode::GCodeApi>(
            &*printer,
            "gcode",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let recover_velocity = config
            .getfloat(
                "recover_velocity",
                GetFloatOpts {
                    default: Some(Some(50.)),
                    ..Default::default()
                },
            )?
            .unwrap_or(50.);
        let inner = Arc::new(Inner {
            printer: printer.clone(),
            gcode: gcode.clone(),
            recover_velocity,
            state: GCell::new(PauseState {
                is_paused: false,
                sd_paused: false,
                pause_command_sent: false,
                v_sd: None,
            }),
        });

        let connect_inner = inner.clone();
        printer.register_event_handler(
            EV_KLIPPY_CONNECT,
            Box::new(move |_params| {
                connect_inner.handle_connect()?;
                Ok(PyValue::None)
            }),
        );

        let register = |cmd: &str, desc: &str, handler: GCodeCommandHandler| -> KResult<()> {
            gcode.register_command(
                cmd,
                Some(handler),
                RegisterCommandOpts { when_not_ready: false, desc: Some(desc.to_string()) },
            )?;
            Ok(())
        };
        let pause_inner = inner.clone();
        register(
            "PAUSE",
            "Pauses the current print",
            Box::new(move |gcmd| pause_inner.cmd_pause(gcmd)),
        )?;
        let resume_inner = inner.clone();
        register(
            "RESUME",
            "Resumes the print from a pause",
            Box::new(move |gcmd| resume_inner.cmd_resume(gcmd)),
        )?;
        let clear_inner = inner.clone();
        register(
            "CLEAR_PAUSE",
            "Clears the current paused state without resuming the print",
            Box::new(move |_gcmd| {
                clear_inner.cmd_clear_pause();
                Ok(())
            }),
        )?;
        let cancel_inner = inner.clone();
        register(
            "CANCEL_PRINT",
            "Cancel the current print",
            Box::new(move |gcmd| cancel_inner.cmd_cancel_print(gcmd)),
        )?;

        let webhooks = lookup_interface::<dyn WebHooksApi>(
            &*printer,
            "webhooks",
            LookupDefault::Raise,
        )?
        .expect("LookupDefault::Raise never yields None");
        let cancel_gcode = gcode.clone();
        let register_endpoint = |path: &str, cb: WebHooksCallback| -> KResult<()> {
            webhooks.register_endpoint(path, cb)
        };
        register_endpoint(
            "pause_resume/cancel",
            Arc::new(move |_request: WebRequest| cancel_gcode.run_script("CANCEL_PRINT")),
        )?;
        let pause_gcode = gcode.clone();
        register_endpoint(
            "pause_resume/pause",
            Arc::new(move |_request: WebRequest| pause_gcode.run_script("PAUSE")),
        )?;
        let resume_gcode = gcode.clone();
        register_endpoint(
            "pause_resume/resume",
            Arc::new(move |_request: WebRequest| resume_gcode.run_script("RESUME")),
        )?;
        register_endpoint(
            "pause_resume/check_continue_print_state",
            Arc::new(|request: WebRequest| {
                let mut data = IndexMap::new();
                data.insert(PyValue::Str("file_state".to_string()), PyValue::Bool(false));
                data.insert(PyValue::Str("eeprom_state".to_string()), PyValue::Bool(false));
                request.send(PyValue::Dict(data))
            }),
        )?;
        let bootloader_printer = printer.clone();
        register_endpoint(
            "getBootLoaderVersion",
            Arc::new(move |request: WebRequest| {
                let mcu = lookup_interface::<dyn McuApi>(
                    &*bootloader_printer,
                    "mcu",
                    LookupDefault::Raise,
                )?
                .expect("LookupDefault::Raise never yields None");
                let version = mcu
                    .get_constants()
                    .get("software_version")
                    .cloned()
                    .unwrap_or_default();
                let mut data = IndexMap::new();
                data.insert(PyValue::Str("software_version".to_string()), PyValue::Str(version));
                request.send(PyValue::Dict(data))
            }),
        )?;

        Ok(PauseResume { inner })
    }

    /// `PauseResume.recover_velocity` (`klippy/extras/pause_resume.py:11`).
    pub fn recover_velocity(&self) -> f64 {
        self.inner.recover_velocity
    }

    /// `PauseResume.is_paused` (`klippy/extras/pause_resume.py:13`).
    pub fn is_paused(&self) -> bool {
        self.inner.is_paused()
    }

    /// `PauseResume.is_sd_active()` (`klippy/extras/pause_resume.py:56`).
    pub fn is_sd_active(&self) -> bool {
        self.inner.is_sd_active()
    }

    /// `PauseResume.send_pause_command()` (`klippy/extras/pause_resume.py:58`).
    pub fn send_pause_command(&self) -> KResult<()> {
        self.inner.send_pause_command()
    }

    /// `PauseResume.send_resume_command()` (`klippy/extras/pause_resume.py:79`).
    pub fn send_resume_command(&self) -> KResult<()> {
        self.inner.send_resume_command()
    }

    /// `PauseResume.handle_connect()` (`klippy/extras/pause_resume.py:38`).
    pub fn handle_connect(&self) -> KResult<()> {
        self.inner.handle_connect()
    }
}

impl PauseResumeApi for PauseResume {
    fn is_sd_active(&self) -> bool {
        self.inner.is_sd_active()
    }
    fn send_pause_command(&self) -> KResult<()> {
        self.inner.send_pause_command()
    }
    fn send_resume_command(&self) -> KResult<()> {
        self.inner.send_resume_command()
    }
    fn is_paused(&self) -> bool {
        self.inner.is_paused()
    }
}

impl PrinterObject for PauseResume {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PauseResumeApi);
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.inner.get_status()))
    }
}

/// `load_config(config)` (`klippy/extras/pause_resume.py:110`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    Ok(Some(Arc::new(PauseResume::new(config)?) as ObjectRef))
}

/// Module entry point for the `pause_resume` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
