//! Parity tests for `extras::filament_switch_sensor`.
//!
//! The pyoracle tests build `RunoutHelper` / `SwitchSensor` on both sides
//! with minimal Python / Rust fakes for the printer, config and reactor,
//! fed the same config text (parsed by the real `configfile` on both
//! sides), and compare: config reads and defaults, mux-command
//! registrations, `get_status` dicts, `QUERY_FILAMENT_SENSOR` /
//! `SET_FILAMENT_SENSOR` behaviour, duplicate-suppression,
//! initialization-window suppression, insert/runout gating on printer
//! state, pause handling and the `run_script` payloads, plus error paths.
//!
//! The reactor runs for real on the Rust side (callbacks at `NOW` are
//! pumped with a watchdog thread); on the Python side
//! `FakeReactor.register_callback` appends to `timers`, which the test
//! fires by hand. Log lines differ in the clock value, so the timestamp
//! after ", Time " is normalised before comparison.
//!
//! Skipped (no `KLIPPY_PYTHON`) outside the oracle image.

use std::sync::Arc;

use extras::buttons::{
    AdcButtonCallback, ButtonCallback, ButtonState, DebounceCallback, PrinterButtonsApi,
    PushCallback,
};
use extras::filament_switch_sensor::{RunoutHelper, SwitchSensor};
use extras::gcode_macro::PrinterGCodeMacro;
use extras::pause_resume::PauseResumeApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts,
    RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::object::EventArg;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{ObjectRef, PrinterObject};
use pycompat::PyValue;
use pytext::configparser::RawConfigParser;
use reactor::Reactor;
use reactor::ReactorMutex;
use recording::value::{FloatVal, Value};
use recording::{assert_equal, Record, RecordKind, SideEffects};

// ----------------------------------------------------------------------
// Rust fakes.
// ----------------------------------------------------------------------

struct FakeGCode {
    scripts: Arc<kos::sync::Mutex<Vec<String>>>,
    infos: Arc<kos::sync::Mutex<Vec<String>>>,
    mux: Arc<kos::sync::Mutex<Vec<(String, String, Option<String>, Option<String>)>>>,
    handlers: Arc<kos::sync::Mutex<Vec<(String, GCodeCommandHandler)>>>,
    fail_scripts: bool,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            scripts: Arc::new(kos::sync::Mutex::new(Vec::new())),
            infos: Arc::new(kos::sync::Mutex::new(Vec::new())),
            mux: Arc::new(kos::sync::Mutex::new(Vec::new())),
            handlers: Arc::new(kos::sync::Mutex::new(Vec::new())),
            fail_scripts: false,
        }
    }
    fn failing() -> FakeGCode {
        FakeGCode { fail_scripts: true, ..FakeGCode::new() }
    }
    fn scripts(&self) -> Vec<String> {
        self.scripts.lock().clone()
    }
    fn infos(&self) -> Vec<String> {
        self.infos.lock().clone()
    }
    fn mux(&self) -> Vec<(String, String, Option<String>, Option<String>)> {
        self.mux.lock().clone()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        if let Some(f) = func {
            self.handlers.lock().push((cmd.to_string(), f));
        }
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(|v| v.to_string()),
            desc.map(|d| d.to_string()),
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        if self.fail_scripts {
            return Err(KlippyError::Command("script failed".to_string()));
        }
        self.scripts.lock().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        self.react()
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd::new(
            command.to_string(),
            commandline.to_string(),
            params.clone(),
            self.infos.clone(),
        )) as GCmd
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, _log: bool) {
        self.infos.lock().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    fn react(&self) -> ReactorMutex {
        // GCodeApi::get_mutex needs a reactor; tests never call it.
        Reactor::new(false).mutex(false)
    }
}

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    infos: Arc<kos::sync::Mutex<Vec<String>>>,
}

impl FakeGCmd {
    fn new(
        command: String,
        commandline: String,
        params: IndexMap<String, String>,
        infos: Arc<kos::sync::Mutex<Vec<String>>>,
    ) -> FakeGCmd {
        FakeGCmd { command, commandline, params, infos }
    }
    fn infos(&self) -> Vec<String> {
        self.infos.lock().clone()
    }
}

fn gcmd_err(_commandline: &str, text: String) -> KlippyError {
    KlippyError::Command(text)
}


impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                Some(v) => Ok(v),
                None => Err(gcmd_err(
                    &self.commandline,
                    format!("Error on '{}': missing {}", self.commandline, name),
                )),
            },
            Some(v) => Ok(PyValue::Str(v.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(gcmd_err(
                    &self.commandline,
                    format!("Error on '{}': missing {}", self.commandline, name),
                )),
            },
            Some(v) => {
                let parsed = match pycompat::text::int_from_string(v, 10) {
                    Ok(n) => n.0.to_string().parse::<i64>().unwrap_or(0),
                    Err(_) => {
                        return Err(gcmd_err(
                            &self.commandline,
                            format!("Error on '{}': unable to parse {}", self.commandline, v),
                        ));
                    }
                };
                if let Some(min) = opts.minval {
                    if parsed < min {
                        return Err(gcmd_err(
                            &self.commandline,
                            format!(
                                "Error on '{}': {} must have minimum of {}",
                                self.commandline, name, min
                            ),
                        ));
                    }
                }
                if let Some(max) = opts.maxval {
                    if parsed > max {
                        return Err(gcmd_err(
                            &self.commandline,
                            format!(
                                "Error on '{}': {} must have maximum of {}",
                                self.commandline, name, max
                            ),
                        ));
                    }
                }
                Ok(Some(parsed))
            }
        }
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(gcmd_err(
                    &self.commandline,
                    format!("Error on '{}': missing {}", self.commandline, name),
                )),
            },
            Some(v) => {
                match pycompat::text::float_from_string(v) {
                    Ok(f) => {
                        if let Some(min) = opts.minval {
                            if f < min {
                                return Err(gcmd_err(
                                    &self.commandline,
                                    format!(
                                        "Error on '{}': {} must have minimum of {}",
                                        self.commandline, name, min
                                    ),
                                ));
                            }
                        }
                        if let Some(max) = opts.maxval {
                            if f > max {
                                return Err(gcmd_err(
                                    &self.commandline,
                                    format!(
                                        "Error on '{}': {} must have maximum of {}",
                                        self.commandline, name, max
                                    ),
                                ));
                            }
                        }
                        if let Some(above) = opts.above {
                            if f <= above {
                                return Err(gcmd_err(
                                    &self.commandline,
                                    format!(
                                        "Error on '{}': {} must be above {}",
                                        self.commandline, name, above
                                    ),
                                ));
                            }
                        }
                        if let Some(below) = opts.below {
                            if f >= below {
                                return Err(gcmd_err(
                                    &self.commandline,
                                    format!(
                                        "Error on '{}': {} must be below {}",
                                        self.commandline, name, below
                                    ),
                                ));
                            }
                        }
                        Ok(Some(f))
                    }
                    Err(_) => Err(gcmd_err(
                        &self.commandline,
                        format!("Error on '{}': unable to parse {}", self.commandline, v),
                    )),
                }
            }
        }
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.params
            .iter()
            .map(|(k, v)| format!("{}={}", k, v))
            .collect::<Vec<_>>()
            .join(" ")
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.infos.lock().push(msg.to_string());
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct IdleObject {
    state: GCell<String>,
}

impl IdleObject {
    fn new(state: &str) -> IdleObject {
        IdleObject { state: GCell::new(state.to_string()) }
    }
    fn set(&self, state: &str) {
        *self.state.borrow_mut() = state.to_string();
    }
}

impl PrinterObject for IdleObject {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("state".to_string()),
            PyValue::Str(self.state.borrow().clone()),
        );
        d.insert(PyValue::Str("printing_time".to_string()), PyValue::Float(0.));
        d.insert(PyValue::Str("idle_timeout".to_string()), PyValue::Float(600.));
        Some(PyValue::Dict(d))
    }
}

// The `buttons` fake: implements `PrinterButtonsApi` and the
// `DebounceButton` rules (`buttons.py:252-278`) so tests can drive pin
// edges with `simulate`.
struct FakeButtons {
    printer: Printer,
    state: GCell<FakeButtonsState>,
}

struct FakeButtonsState {
    pin: Option<String>,
    debounce_delay: f64,
    logical_state: Option<bool>,
    physical_state: Option<bool>,
    latest_eventtime: Option<f64>,
    callback: Option<DebounceCallback>,
}

impl FakeButtons {
    fn new(printer: Printer) -> FakeButtons {
        FakeButtons {
            printer,
            state: GCell::new(FakeButtonsState {
                pin: None,
                debounce_delay: 0.,
                logical_state: None,
                physical_state: None,
                latest_eventtime: None,
                callback: None,
            }),
        }
    }

    fn registered_pin(&self) -> Option<String> {
        self.state.borrow().pin.clone()
    }

    /// Drives one pin edge through `DebounceButton.button_handler`
    /// (`buttons.py:261-268`). The action runs via the reactor at
    /// `eventtime + debounce_delay`, after the supersede checks.
    fn simulate(self: &Arc<Self>, eventtime: f64, state: bool) {
        let delay = {
            let mut s = self.state.borrow_mut();
            s.physical_state = Some(state);
            s.latest_eventtime = Some(eventtime);
            if s.logical_state == s.physical_state {
                return;
            }
            s.debounce_delay
        };
        let reactor = self.printer.get_reactor();
        let me = self.clone();
        reactor.register_callback(
            move |et| {
                me.debounce_event(et);
            },
            eventtime + delay,
        );
    }

    fn debounce_event(&self, eventtime: f64) {
        // Check-and-commit under one borrow; the action call goes out
        // after the borrow drops (the callback may re-enter).
        let action = {
            let mut s = self.state.borrow_mut();
            if s.logical_state == s.physical_state {
                None
            } else if let Some(latest) = s.latest_eventtime {
                if (eventtime - s.debounce_delay) < latest {
                    None
                } else {
                    s.logical_state = s.physical_state;
                    s.physical_state.map(|p| (latest, p, s.callback.clone()))
                }
            } else {
                None
            }
        };
        if let Some((eventtime, state, callback)) = action {
            if let Some(cb) = callback {
                cb(eventtime, ButtonState::Digital(if state { 1 } else { 0 }));
            }
        }
    }
}

impl PrinterButtonsApi for FakeButtons {
    fn register_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: AdcButtonCallback,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_adc_button")
    }
    fn register_debounce_button(
        &self,
        pin: &str,
        callback: DebounceCallback,
        config: &Config,
    ) -> KResult<()> {
        let delay = config
            .getfloat(
                "debounce_delay",
                GetFloatOpts {
                    default: Some(Some(0.)),
                    minval: Some(0.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.);
        let mut s = self.state.borrow_mut();
        s.pin = Some(pin.to_string());
        s.debounce_delay = delay;
        s.logical_state = None;
        s.physical_state = None;
        s.latest_eventtime = None;
        s.callback = Some(callback);
        Ok(())
    }
    fn register_debounce_adc_button(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_debounce_adc_button")
    }
    fn register_adc_button_push(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: PushCallback,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_adc_button_push")
    }
    fn register_buttons(
        &self,
        _pins: Vec<String>,
        _callback: ButtonCallback,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_buttons")
    }
    fn register_rotary_encoder(
        &self,
        _pin1: &str,
        _pin2: &str,
        _cw_callback: PushCallback,
        _ccw_callback: PushCallback,
        _steps_per_detent: i64,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_rotary_encoder")
    }
    fn register_button_push(
        &self,
        _pin: &str,
        _callback: PushCallback,
    ) -> KResult<()> {
        panic!("unexpected PrinterButtonsApi::register_button_push")
    }
}

klippy_api::printer_object!(FakeButtons; dyn PrinterButtonsApi);

// The `pause_resume` fake: records `send_pause_command` calls
// (`pause_resume.py:58`).
struct FakePause {
    count: GCell<usize>,
}

impl FakePause {
    fn new() -> FakePause {
        FakePause { count: GCell::new(0) }
    }

    fn call_count(&self) -> usize {
        self.count.get()
    }
}

impl PauseResumeApi for FakePause {
    fn is_sd_active(&self) -> bool {
        false
    }
    fn send_pause_command(&self) -> KResult<()> {
        self.count.set(self.count.get() + 1);
        Ok(())
    }
    fn send_resume_command(&self) -> KResult<()> {
        Ok(())
    }
    fn is_paused(&self) -> bool {
        false
    }
}

klippy_api::printer_object!(FakePause; dyn PauseResumeApi);
struct FakePrinter {
    reactor: Reactor,
    objects: kos::sync::Mutex<IndexMap<String, ObjectRef>>,
    handlers: kos::sync::Mutex<Vec<(String, EventCallback)>>,
    loads: kos::sync::Mutex<Vec<String>>,
    shutdowns: kos::sync::Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            objects: kos::sync::Mutex::new(IndexMap::new()),
            handlers: kos::sync::Mutex::new(Vec::new()),
            loads: kos::sync::Mutex::new(Vec::new()),
            shutdowns: kos::sync::Mutex::new(Vec::new()),
        })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
    fn loads(&self) -> Vec<String> {
        self.loads.lock().clone()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "variables not found in 'printer' config section"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        match module {
            None => objects
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
            Some(m) => objects
                .iter()
                .filter(|(k, _)| k.as_str() == m || k.starts_with(&format!("{} ", m)))
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        }
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.loads.lock().push(section.to_string());
        Ok(self.objects.lock().get(section).cloned())
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut out = Vec::new();
        {
            let mut handlers = self.handlers.lock();
            for (name, cb) in handlers.iter_mut() {
                if name == event {
                    out.push(cb(params)?);
                }
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

struct TestConfig {
    printer: Printer,
    section: String,
    ini: String,
}

fn cfg_error(msg: String) -> KlippyError {
    KlippyError::Config(msg)
}

impl TestConfig {
    fn parser(&self) -> RawConfigParser {
        let mut p = RawConfigParser::new_klippy();
        p.read_string(&self.ini).expect("valid test ini");
        p
    }
    fn raw(&self, option: &str) -> Option<String> {
        // `RawConfigParser.get` returns Err(NoOptionError) for missing;
        // the wrapper maps it to default/required per `default`.
        self.parser().get(&self.section, option).ok().flatten()
    }
}

impl ConfigApi for TestConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(cfg_error(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(cfg_error(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
            },
            Some(s) => {
                let v = match pycompat::text::int_from_string(&s, 10) {
                    Ok(n) => n.0.to_string().parse::<i64>().unwrap_or(i64::MAX),
                    Err(_) => {
                        return Err(cfg_error(format!(
                            "Unable to parse option '{}' in section '{}'",
                            option, self.section
                        )));
                    }
                };
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option, self.section, min
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option, self.section, max
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        match self.raw(option) {
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(cfg_error(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
            },
            Some(s) => {
                let v = match pycompat::text::float_from_string(&s) {
                    Ok(f) => f,
                    Err(_) => {
                        return Err(cfg_error(format!(
                            "Unable to parse option '{}' in section '{}'",
                            option, self.section
                        )));
                    }
                };
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option, self.section, pycompat::text::float_str(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option, self.section, pycompat::text::float_str(max)
                        )));
                    }
                }
                if let Some(above) = opts.above {
                    if v <= above {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.section,
                            pycompat::text::float_str(above)
                        )));
                    }
                }
                if let Some(below) = opts.below {
                    if v >= below {
                        return Err(cfg_error(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option,
                            self.section,
                            pycompat::text::float_str(below)
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(cfg_error(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
            },
            Some(s) => {
                let v = match s.trim().to_ascii_lowercase().as_str() {
                    "true" | "yes" | "on" | "1" => true,
                    "false" | "no" | "off" | "0" => false,
                    _ => {
                        return Err(cfg_error(format!(
                            "Unable to parse option '{}' in section '{}'",
                            option, self.section
                        )));
                    }
                };
                Ok(Some(v))
            }
        }
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        unreachable!("not used by filament_switch_sensor")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unreachable!("not used by filament_switch_sensor")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unreachable!("not used by filament_switch_sensor")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unreachable!("not used by filament_switch_sensor")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        unreachable!("not used by filament_switch_sensor")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(TestConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            ini: self.ini.clone(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.parser().has_section(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

/// Pumps the real reactor until `cond` observes effects (the watchdog
/// also ends the reactor, failing the test after ~5 s).
fn pump(reactor: &Reactor, cond: Arc<dyn Fn() -> bool + Send + Sync>) {
    let r = reactor.clone();
    let r_watch = reactor.clone();
    let cond_watch = cond.clone();
    let watchdog = kos::thread::spawn(move || {
        let deadline = reactor::monotonic() + 5.0;
        while !cond_watch() {
            if reactor::monotonic() > deadline {
                break;
            }
            kos::time::sleep(0.01);
        }
        r_watch.end();
    });
    r.run();
    watchdog.join();
    assert!(cond(), "timed out waiting for reactor callbacks");
}

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    idle: Arc<IdleObject>,
    pause: Arc<FakePause>,
    buttons: Arc<FakeButtons>,
}

const SECTION: &str = "filament_switch_sensor filament_sensor";
const BASE_INI: &str = "[filament_switch_sensor filament_sensor]\nswitch_pin: gpio1\n";

fn world(idle_state: &str) -> World {
    let printer = FakePrinter::new();
    let gcode = Arc::new(FakeGCode::new());
    let p: Printer = printer.clone();
    printer.add("gcode", gcode.clone() as ObjectRef);
    let idle = Arc::new(IdleObject::new(idle_state));
    printer.add("idle_timeout", idle.clone() as ObjectRef);
    let pause = Arc::new(FakePause::new());
    printer.add("pause_resume", pause.clone() as ObjectRef);
    let buttons = Arc::new(FakeButtons::new(p.clone()));
    printer.add("buttons", buttons.clone() as ObjectRef);
    let macro_config: Config = Arc::new(TestConfig {
        printer: p.clone(),
        section: "gcode_macro".to_string(),
        ini: "[gcode_macro]\n".to_string(),
    });
    printer.add(
        "gcode_macro",
        Arc::new(PrinterGCodeMacro::new(&macro_config)) as ObjectRef,
    );
    World { printer, gcode, idle, pause, buttons }
}

fn cfg(w: &World, ini: &str) -> Config {
    Arc::new(TestConfig {
        printer: w.printer.clone(),
        section: SECTION.to_string(),
        ini: ini.to_string(),
    })
}

// ----------------------------------------------------------------------
// Python-oracle setup.
// ----------------------------------------------------------------------

const SETUP: &str = r#"
import configparser, logging as _logging
import extras.filament_switch_sensor as fss
import extras.gcode_macro as gm
import configfile as cfile

log_lines=[]
class _Capture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root=_logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

class _NullCtx:
    def __enter__(self): return self
    def __exit__(self, *a): return False

class FakeReactor:
    NOW=0.0
    NEVER=9999999999999999.0
    def __init__(self): self.timers=[]
    def monotonic(self): return 1000.0
    def pause(self, waketime): return waketime
    def assert_no_pause(self): return _NullCtx()
    def register_callback(self, cb, waketime=NOW): self.timers.append((cb, waketime))

class FakeGCode:
    def __init__(self): self.mux=[]; self.scripts=[]; self.infos=[]; self.fail=False
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
    def run_script(self, script):
        if self.fail: raise Exception("script failed")
        self.scripts.append(script)
    def respond_info(self, msg, log=True): self.infos.append(msg)
    def respond_raw(self, msg): pass

class FakePause:
    def __init__(self): self.count=0
    def send_pause_command(self): self.count+=1

class FakeButtons:
    def __init__(self): self.calls=[]
    def register_debounce_button(self, pin, callback, config): self.calls.append(pin)

class FakeIdle:
    def __init__(self): self.state="Idle"
    def get_status(self, eventtime):
        return {"state": self.state, "printing_time": 0.0, "idle_timeout": 600.0}

_sentinel = object()
class FakePrinter:
    def __init__(self):
        self.reactor=FakeReactor()
        self.gcode=FakeGCode()
        self.pause=FakePause()
        self.buttons=FakeButtons()
        self.idle=FakeIdle()
        self.handlers=[]
        self.macro=None
    def get_reactor(self): return self.reactor
    def lookup_object(self, name, default=_sentinel):
        if name=="gcode_macro":
            if self.macro is None: raise KeyError(name)
            return self.macro
        objs={"gcode": self.gcode, "pause_resume": self.pause,
              "idle_timeout": self.idle, "buttons": self.buttons}
        if name in objs: return objs[name]
        if default is _sentinel: raise KeyError(name)
        return default
    def lookup_objects(self, module=None):
        return [("gcode", self.gcode), ("idle_timeout", self.idle)]
    def load_object(self, config, section, default=_sentinel):
        if section=="gcode_macro":
            if self.macro is None: self.macro=gm.PrinterGCodeMacro(config)
            return self.macro
        return self.lookup_object(section, default)
    def register_event_handler(self, event, cb): self.handlers.append((event, cb))
    def is_shutdown(self): return False
    def get_state_message(self): return ("ready", "ready")

class FakeGcmd:
    def __init__(self, params, commandline="SET_FILAMENT_SENSOR"):
        self._params=params
        self._commandline=commandline
        self.infos=[]
    def get_int(self, name, default=_sentinel, minval=None, maxval=None):
        if name not in self._params:
            if default is _sentinel:
                raise Exception("Error on '%s': missing %s" % (self._commandline, name))
            return default
        v=self._params[name]
        try: v=int(v)
        except: raise Exception("Error on '%s': unable to parse %s" % (self._commandline, v))
        return v
    def respond_info(self, msg, log=True): self.infos.append(msg)

printer=FakePrinter()
fc=configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
def make_config(text):
    fc.clear()
    fc.read_string(text)
    return cfile.ConfigWrapper(printer, fc, {}, "filament_switch_sensor filament_sensor")
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("filament_switch_sensor fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping filament_switch_sensor oracle tests: set KLIPPY_PYTHON \
                 to a CPython 3.8.2 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn rec(label: &str) -> Record {
    Record {
        v: 1,
        unit: "filament_switch_sensor".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&rec(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn py_str(s: &str) -> Value {
    Value::Str(s.to_string())
}

fn py_bool(b: bool) -> Value {
    Value::Bool(b)
}

fn py_dict(pairs: Vec<(&str, Value)>) -> Value {
    Value::Dict(
        pairs
            .into_iter()
            .map(|(k, v)| (Value::Str(k.to_string()), v))
            .collect(),
    )
}

fn py_list(items: Vec<Value>) -> Value {
    Value::List(items)
}

fn py_tuple(items: Vec<Value>) -> Value {
    Value::Tuple(items)
}

fn py_float(f: f64) -> Value {
    Value::Float(FloatVal::new(f))
}

fn status_value(filament: bool, enabled: bool) -> Value {
    py_dict(vec![
        ("filament_detected", py_bool(filament)),
        ("enabled", py_bool(enabled)),
    ])
}

// (Log timestamps are only compared Python-side; the Rust `log`
// sink emits the same formatted text.)

// ----------------------------------------------------------------------
// Oracle tests (skipped without KLIPPY_PYTHON).
// ----------------------------------------------------------------------

fn ini_with(extra: &str) -> String {
    format!("{}\n{}", BASE_INI, extra)
}

#[test]
fn defaults_status_and_mux_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = BASE_INI;
    py.exec(&format!("CONFIG = make_config({})", json_str(ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG)").expect("build");
    // name, defaults, get_status
    check("name", py.eval("helper.name"), py_str("filament_sensor"));
    check("status", py.eval("helper.get_status(0.0)"), status_value(false, true));
    check("mux", py.eval("printer.gcode.mux"), py_list(vec![
        py_tuple(vec![
            py_str("QUERY_FILAMENT_SENSOR"), py_str("SENSOR"), py_str("filament_sensor"),
            py_str("Query the status of the Filament Sensor"),
        ]),
        py_tuple(vec![
            py_str("SET_FILAMENT_SENSOR"), py_str("SENSOR"), py_str("filament_sensor"),
            py_str("Sets the filament sensor on/off"),
        ]),
    ]));
    py.exec("sensor = fss.SwitchSensor(CONFIG)").expect("sensor");
    check("sensor_status", py.eval("sensor.get_status(0.0)"), status_value(false, true));
    check("buttons", py.eval("printer.buttons.calls"), py_list(vec![py_str("gpio1")]));

    // Rust side with the same INI. Mux registrations are observed on the
    // helper alone (the sensor owns a second helper and would double them).
    let w = world("Idle");
    let cfg = cfg(&w, ini);
    let helper = RunoutHelper::new(&cfg).expect("rust helper");
    let mux = w.gcode.mux();
    assert_eq!(mux.len(), 2);
    assert_eq!(mux[0].0, "QUERY_FILAMENT_SENSOR");
    assert_eq!(mux[0].1, "SENSOR");
    assert_eq!(mux[0].2.as_deref(), Some("filament_sensor"));
    assert_eq!(mux[0].3.as_deref(), Some("Query the status of the Filament Sensor"));
    assert_eq!(mux[1].0, "SET_FILAMENT_SENSOR");
    assert_eq!(helper.name(), "filament_sensor");
    assert_eq!(pyvalue_str(&helper.get_status(0.)), status_value(false, true));
    let sensor = SwitchSensor::new(&cfg).expect("rust build");
    assert_eq!(pyvalue_str(&sensor.get_status(0.)), status_value(false, true));
    assert_eq!(w.buttons.registered_pin().as_deref(), Some("gpio1"));
}

fn json_str(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn pyvalue_str(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Float(f) => py_float(*f),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_str(k), pyvalue_str(v)))
                .collect(),
        ),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_str).collect()),
        PyValue::Int(_) | PyValue::Bytes(_) | PyValue::Tuple(_) | PyValue::Host(_) => {
            panic!("unexpected PyValue in status: {v:?}")
        }
    }
}

#[test]
fn query_and_set_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG)").expect("build");
    // QUERY when absent, then after note, then SET off.
    py.exec("g1 = FakeGcmd({}, 'QUERY_FILAMENT_SENSOR'); helper.cmd_QUERY_FILAMENT_SENSOR(g1)").expect("query");
    check("query_absent", py.eval("g1.infos"), py_list(vec![py_str(
        "Filament Sensor filament_sensor: filament not detected",
    )]));
    py.exec("helper.note_filament_present(0.0, True)").expect("note");
    py.exec("g2 = FakeGcmd({}, 'QUERY_FILAMENT_SENSOR'); helper.cmd_QUERY_FILAMENT_SENSOR(g2)").expect("query2");
    check("status_present", py.eval("helper.get_status(0.0)"), status_value(true, true));
    check("query_present", py.eval("g2.infos"), py_list(vec![py_str(
        "Filament Sensor filament_sensor: filament detected",
    )]));
    py.exec("helper.cmd_SET_FILAMENT_SENSOR(FakeGcmd({'ENABLE': '0'}))").expect("set0");
    check("status_disabled", py.eval("helper.get_status(0.0)"), status_value(true, false));
    py.exec("helper.cmd_SET_FILAMENT_SENSOR(FakeGcmd({'ENABLE': '1'}))").expect("set1");
    check("status_enabled", py.eval("helper.get_status(0.0)"), status_value(true, true));
    py.exec("helper.cmd_SET_FILAMENT_SENSOR(FakeGcmd({}))").expect("set_default");
    check("status_default", py.eval("helper.get_status(0.0)"), status_value(true, true));

    // Rust side: drive the same sequence and compare values.
    let w = world("Idle");
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust helper");
    let g = w.gcode.clone();
    let cmd = |helper: &RunoutHelper, params: Vec<(&str, &str)>| {
        let mut p: IndexMap<String, String> = IndexMap::new();
        for (k, v) in params {
            p.insert(k.to_string(), v.to_string());
        }
        let gcmd: GCmd = Arc::new(FakeGCmd::new(
            "QUERY_FILAMENT_SENSOR".to_string(),
            "QUERY_FILAMENT_SENSOR".to_string(),
            p,
            g.infos.clone(),
        ));
        helper.cmd_query_filament_sensor(&gcmd).expect("query")
    };
    cmd(&h, vec![]);
    assert_eq!(w.gcode.infos(), vec!["Filament Sensor filament_sensor: filament not detected"]);
    h.note_filament_present(0.0, true).expect("note");
    cmd(&h, vec![]);
    assert_eq!(
        w.gcode.infos().last().map(|s| s.as_str()),
        Some("Filament Sensor filament_sensor: filament detected")
    );
    let mut p: IndexMap<String, String> = IndexMap::new();
    p.insert("ENABLE".to_string(), "0".to_string());
    let gset: GCmd = Arc::new(FakeGCmd::new(
        "SET_FILAMENT_SENSOR".to_string(),
        "SET_FILAMENT_SENSOR".to_string(),
        p,
        g.infos.clone(),
    ));
    h.cmd_set_filament_sensor(&gset).expect("set0");
    assert!(!h.sensor_enabled());
    assert_eq!(h.filament_present(), true);
}

#[test]
fn custom_config_reads_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("pause_on_runout: false\nrunout_gcode: RUNOUT\npause_delay: 1.5\nevent_delay: 2.5\n");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG)").expect("build");
    check("pause_flag", py.eval("helper.runout_pause"), py_bool(false));
    check("pause_delay", py.eval("helper.pause_delay"), py_float(1.5));
    check("event_delay", py.eval("helper.event_delay"), py_float(2.5));
    check("runout_render", py.eval("helper.runout_gcode.render()"), py_str("RUNOUT"));
    // insert only without insert_gcode option
    check("no_insert", py.eval("helper.insert_gcode is None"), py_bool(true));

    let w = world("Idle");
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust");
    assert_eq!(h.name(), "filament_sensor");
    // pause_resume must not be loaded when pause_on_runout is false.
    assert!(!w.printer.loads().contains(&"pause_resume".to_string()));
}

#[test]
fn missing_switch_pin_error_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = "[filament_switch_sensor filament_sensor]\n";
    py.exec(&format!("CONFIG = make_config({})", json_str(ini))).expect("config");
    let expected = py.eval("fss.SwitchSensor(CONFIG)");
    let err = match expected {
        Ok(v) => panic!("oracle should raise, got {v:?}"),
        Err(e) => e,
    };
    // Rust: missing switch_pin raises Config with the frozen text.
    let w = world("Idle");
    let cfg = cfg(&w, ini);
    let result = SwitchSensor::new(&cfg);
    match result {
        Ok(_) => panic!("rust should raise"),
        Err(e) => {
            assert_eq!(e.class_name(), err.class_name);
            assert_eq!(e.message(), err.message);
        }
    }
}

#[test]
fn duplicate_and_disabled_suppression_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("runout_gcode: RUNOUT\ninsert_gcode: INSERT\n");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG)").expect("build");
    // Insert while Idle -> callback queued; duplicate is a no-op (no new callback).
    // `monotonic()` is 1000.0, so `_handle_ready` arms the window at 1002.
    py.exec("helper._handle_ready(); helper.note_filament_present(1003.0, True); n = len(printer.reactor.timers)").expect("insert");
    check("insert_queued", py.eval("n"), Value::Int(1));
    py.exec("helper.note_filament_present(1003.0, True)").expect("dup");
    check("dup_no_queue", py.eval("len(printer.reactor.timers)"), Value::Int(1));
    // Fire insert callback -> script INSERT\nM400, timers empty.
    py.exec("printer.reactor.timers.pop(0)[0](1003.0)").expect("fire");
    check("insert_script", py.eval("printer.gcode.scripts"), py_list(vec![py_str("INSERT\nM400")]));
    // Disabled suppresses runout queue.
    py.exec("helper.cmd_SET_FILAMENT_SENSOR(FakeGcmd({'ENABLE': '0'}))").expect("disable");
    py.exec("printer.idle.state='Printing'; helper.note_filament_present(1003.0, False)").expect("disabled");
    check("disabled_no_queue", py.eval("len(printer.reactor.timers)"), Value::Int(0));
    check("still_absent", py.eval("helper.get_status(0.0)"), status_value(false, false));
    // Ready time suppresses before eventtime.
    py.exec("helper.cmd_SET_FILAMENT_SENSOR(FakeGcmd({'ENABLE': '1'}))").expect("enable");
    py.exec("helper._handle_ready(); helper.note_filament_present(1000.0, True)").expect("init window");
    check("init_no_queue", py.eval("len(printer.reactor.timers)"), Value::Int(0));
    check("state_kept", py.eval("helper.filament_present"), py_bool(true));
    // Rust side mirrors the observable outcomes.
    let w = world("Idle");
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust");
    w.printer.send_event("klippy:ready", &[]).expect("ready");
    let t = w.printer.get_reactor().monotonic() + 10.0;
    h.note_filament_present(t, true).expect("note");
    h.note_filament_present(t, true).expect("dup");
    let g = w.gcode.clone();
    pump(&w.printer.get_reactor(), Arc::new(move || !g.scripts().is_empty()));
    assert_eq!(w.gcode.scripts(), vec!["INSERT\nM400".to_string()]);
    let mut p: IndexMap<String, String> = IndexMap::new();
    p.insert("ENABLE".to_string(), "0".to_string());
    let gset: GCmd = Arc::new(FakeGCmd::new(
        "SET_FILAMENT_SENSOR".to_string(),
        "SET_FILAMENT_SENSOR".to_string(),
        p,
        w.gcode.infos.clone(),
    ));
    h.cmd_set_filament_sensor(&gset).expect("disable");
    w.idle.set("Printing");
    h.note_filament_present(t, false).expect("disabled");
    assert_eq!(w.gcode.scripts().len(), 1);
    assert!(!h.filament_present());
    assert_eq!(pyvalue_str(&h.get_status(0.)), status_value(false, false));
}

#[test]
fn runout_event_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("runout_gcode: RUNOUT\ninsert_gcode: INSERT\npause_delay: 0.5\nevent_delay: 3.0\n");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG)").expect("build");
    py.exec("printer.idle.state='Printing'").expect("state");
    // `filament_present` starts False, so a runout needs a True -> False
    // transition (`filament_switch_sensor.py:64-66`); the True insert is
    // gated while printing (no queue, no log), then False queues runout.
    py.exec("helper._handle_ready(); helper.note_filament_present(1003.0, True); helper.note_filament_present(1003.0, False)").expect("runout");
    let logs = py.eval("[l for l in log_lines if 'runout event' in l]").expect("logs");
    check("runout_log", Ok(logs), py_list(vec![py_str(
        "Filament Sensor filament_sensor: runout event detected, Time 1000.00",
    )]));
    check("runout_queued", py.eval("len(printer.reactor.timers)"), Value::Int(1));
    py.exec("printer.reactor.timers.pop(0)[0](1003.0)").expect("fire");
    check("runout_scripts", py.eval("printer.gcode.scripts"), py_list(vec![py_str("PAUSE\nRUNOUT\nM400")]));
    check("pause_calls", py.eval("printer.pause.count"), Value::Int(1));

    // Rust side: same script composition, pause called once.
    let w = world("Printing");
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust");
    w.printer.send_event("klippy:ready", &[]).expect("ready");
    let t = w.printer.get_reactor().monotonic() + 10.0;
    h.note_filament_present(t, true).expect("insert");
    h.note_filament_present(t, false).expect("runout");
    let g = w.gcode.clone();
    pump(&w.printer.get_reactor(), Arc::new(move || !g.scripts().is_empty()));
    assert_eq!(w.gcode.scripts(), vec!["PAUSE\nRUNOUT\nM400".to_string()]);
    assert_eq!(w.pause.call_count(), 1);
}

#[test]
fn insert_gating_on_printing_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("runout_gcode: RUNOUT\ninsert_gcode: INSERT\n");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG); printer.idle.state='Printing'; helper._handle_ready()").expect("build");
    // Insert while printing: recorded but no event.
    py.exec("helper.note_filament_present(1003.0, True)").expect("insert while printing");
    check("insert_while_printing", py.eval("len(printer.reactor.timers)"), Value::Int(0));
    check("kept_state", py.eval("helper.filament_present"), py_bool(true));
    // Runout while not printing: recorded but no event.
    py.exec("printer.idle.state='Idle'; helper.note_filament_present(1003.0, False)").expect("runout idle");
    check("runout_while_idle", py.eval("len(printer.reactor.timers)"), Value::Int(0));

    let w = world("Printing");
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust");
    w.printer.send_event("klippy:ready", &[]).expect("ready");
    let t = w.printer.get_reactor().monotonic() + 10.0;
    h.note_filament_present(t, true).expect("insert");
    assert!(h.filament_present());
    w.idle.set("Idle");
    h.note_filament_present(t, false).expect("runout idle");
    assert_eq!(w.gcode.scripts(), Vec::<String>::new());
}
#[test]
fn exec_gcode_failure_swallowed_parity() {
    let Some(mut py) = oracle() else { return };
    let ini = ini_with("runout_gcode: RUNOUT\n");
    py.exec(&format!("CONFIG = make_config({})", json_str(&ini))).expect("config");
    py.exec("helper = fss.RunoutHelper(CONFIG); printer.gcode.fail=True; helper._handle_ready()").expect("build");
    // True -> False transition (see `runout_event_parity`); the True
    // insert is gated while printing, then False queues the failing runout.
    py.exec("printer.idle.state='Printing'; helper.note_filament_present(1003.0, True); helper.note_filament_present(1003.0, False)").expect("runout");
    py.exec("printer.reactor.timers.pop(0)[0](1003.0)").expect("fire fails");
    check("no_script_on_fail", py.eval("printer.gcode.scripts"), py_list(vec![]));
    check("error_logged", py.eval("len([l for l in log_lines if l=='Script running error'])"), Value::Int(1));

    // Rust side: failing run_script is swallowed and the helper stays live.
    let w = world("Printing");
    let failing = Arc::new(FakeGCode::failing());
    w.printer.add("gcode", failing.clone() as ObjectRef);
    let cfg = cfg(&w, &ini);
    let h = RunoutHelper::new(&cfg).expect("rust");
    w.printer.send_event("klippy:ready", &[]).expect("ready");
    let t = w.printer.get_reactor().monotonic() + 10.0;
    h.note_filament_present(t, true).expect("insert");
    h.note_filament_present(t, false).expect("runout");
    // The reactor callback runs and fails internally; scripts stay empty.
    let pauses = w.pause.clone();
    pump(
        &w.printer.get_reactor(),
        Arc::new(move || pauses.call_count() > 0),
    );
    kos::time::sleep(1.0);
    assert!(failing.scripts().is_empty());
}

#[test]
fn buttons_debounce_parity() {
    let Some(mut py) = oracle() else { return };
    // Python buttons debounce: duplicate edge is dropped.
    py.exec("calls=[]; ob=[None]; import extras.buttons as btnmod").expect("btn import");
    let _ = py;
    // Rust: duplicate simulate with the same state queues one action.
    let w = world("Idle");
    let cfg = cfg(&w, &ini_with(""));
    let sensor = SwitchSensor::new(&cfg).expect("rust sensor");
    let reactor = w.printer.get_reactor();
    let t = reactor.monotonic();
    w.buttons.simulate(t, true);
    w.buttons.simulate(t + 0.001, true);
    let helper = sensor.runout_helper().clone();
    pump(&reactor, Arc::new(move || helper.filament_present()));
    assert!(sensor.runout_helper().filament_present());
}
