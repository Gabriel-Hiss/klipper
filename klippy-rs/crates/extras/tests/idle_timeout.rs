//! Parity tests for `klippy/extras/idle_timeout.py`.
//!
//! The test drives the Rust port through fakes of the contract traits
//! (`PrinterApi`, `ConfigApi`, `GCodeApi`, `GCodeCommandApi`,
//! `ToolHeadApi`) with the same inputs given to the reference: the real
//! `IdleTimeout` class is built in the live CPython oracle (`pyoracle`)
//! with equivalent Python fakes (`PY_SETUP` below), and every expected
//! value is an oracle `eval`/`exec` result compared with
//! `recording::assert_equal` (floats by hex + repr spelling, errors by
//! class and message, event args, gcode scripts, log lines). The oracle
//! is skipped when `$KLIPPY_PYTHON` is unset (outside the oracle
//! image); the remaining tests check structural invariants that embed
//! no reference numbers.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test idle_timeout
//! ```

use std::path::PathBuf;
use std::sync::{Arc, LazyLock, Mutex};

use extras::gcode_macro::PrinterGCodeMacro;
use extras::idle_timeout::{IdleTimeout, module_factory};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts as CfgGetFloatOpts,
    GetFloatListOpts, GetIntOpts as CfgGetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts,
    GetStrOpts,
};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GCmdFloatOpts,
    GetIntOpts as GCmdIntOpts, GetOpts as GCmdGetOpts, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Rail, StepperFactory, Trapq, TrapqApi};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, KinematicsApi, MoveApi, PrinterExtruder,
    ToolHead, ToolHeadApi,
};
use klippy_api::{EventArg, KlippyError, KResult, ObjectRef, PrinterObject};
use pycompat::PyValue;
use pycompat::text as pytext;
use reactor::{Reactor, ReactorMutex, NEVER};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..")
}

fn klippy_extras_path() -> String {
    repo_root()
        .join("klippy")
        .join("extras")
        .to_string_lossy()
        .replace('\\', "/")
}

// ---------------------------------------------------------------------------
// Log capture (process-wide sink, installed once; tests assert presence
// and relative order of records which is race-free).
// ---------------------------------------------------------------------------

static LOGS: LazyLock<Arc<Mutex<Vec<(String, String)>>>> = LazyLock::new(|| {
    let logs = Arc::new(Mutex::new(Vec::new()));
    let logs2 = logs.clone();
    klippy_api::log::set_sink(Box::new(move |rec| {
        logs2
            .lock()
            .unwrap()
            .push((rec.level.name().to_string(), rec.message.to_string()));
    }));
    logs
});

fn install_log_sink() -> Arc<Mutex<Vec<(String, String)>>> {
    LOGS.clone()
}

/// Captured records as `(level, message)` with trailing newlines trimmed
/// (CPython traceback pieces carry a trailing newline that the Rust port
/// does not; the normalizer covers traceback bodies).
fn captured_logs() -> Vec<(String, String)> {
    install_log_sink()
        .lock()
        .unwrap()
        .iter()
        .map(|(lvl, msg)| (lvl.clone(), msg.trim_end_matches('\n').to_string()))
        .collect()
}

fn has_log(level: &str, msg: &str) -> bool {
    captured_logs().iter().any(|(l, m)| l == level && m == msg)
}

// ---------------------------------------------------------------------------
// Fakes of the contract traits.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct FakePrinterInner {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<EventCallback>>,
    events: Vec<(String, Vec<EventArg>)>,
    shutdown: bool,
    shutdown_msgs: Vec<String>,
}

struct FakePrinter {
    reactor: Reactor,
    inner: Mutex<FakePrinterInner>,
}

impl FakePrinter {
    fn new() -> (Arc<FakePrinter>, Printer) {
        let p = Arc::new(FakePrinter {
            reactor: Reactor::new(false),
            inner: Mutex::new(FakePrinterInner::default()),
        });
        let printer: Printer = p.clone();
        (p, printer)
    }

    fn register_object(&self, name: &str, obj: ObjectRef) {
        self.inner
            .lock()
            .unwrap()
            .objects
            .insert(name.to_string(), obj);
    }

    /// Dispatches to the registered handlers without holding the inner
    /// lock (Python handlers may re-enter the printer). Handlers
    /// registered during the dispatch are not invoked by it.
    fn dispatch(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.inner
            .lock()
            .unwrap()
            .events
            .push((event.to_string(), params.to_vec()));
        let mut handlers = {
            let mut inner = self.inner.lock().unwrap();
            inner.handlers.shift_remove(event).unwrap_or_default()
        };
        let mut rets = Vec::new();
        let mut err: Option<KlippyError> = None;
        for cb in handlers.iter_mut() {
            match cb(params) {
                Ok(r) => rets.push(r),
                Err(e) => {
                    err = Some(e);
                    break;
                }
            }
        }
        // Re-register: originals (minus any the error path skipped keep
        // their place) — simply put the original list back ahead of any
        // handlers added during dispatch.
        {
            let mut inner = self.inner.lock().unwrap();
            let mut added = inner.handlers.shift_remove(event).unwrap_or_default();
            let mut merged = Vec::new();
            merged.append(&mut handlers);
            merged.append(&mut added);
            inner.handlers.insert(event.to_string(), merged);
        }
        match err {
            Some(e) => Err(e),
            None => Ok(rets),
        }
    }

    fn run_handlers(&self, event: &str, args: &[EventArg]) -> KResult<()> {
        self.dispatch(event, args)?;
        Ok(())
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
        self.inner.lock().unwrap().shutdown
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.inner.lock().unwrap();
        if inner.objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        inner.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let inner = self.inner.lock().unwrap();
        if let Some(o) = inner.objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{}'",
                name
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let inner = self.inner.lock().unwrap();
        inner
            .objects
            .iter()
            .filter(|(k, _)| match module {
                None => true,
                Some(m) => k.as_str() == m || k.starts_with(&format!("{} ", m)),
            })
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let mut inner = self.inner.lock().unwrap();
        if let Some(o) = inner.objects.get(section) {
            return Ok(Some(o.clone()));
        }
        if section == "gcode_macro" {
            let obj: ObjectRef = Arc::new(PrinterGCodeMacro::new(config));
            inner.objects.insert(section.to_string(), obj.clone());
            return Ok(Some(obj));
        }
        Err(KlippyError::Config(format!(
            "Unable to load module '{}'",
            section
        )))
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        let mut inner = self.inner.lock().unwrap();
        inner.shutdown = true;
        inner.shutdown_msgs.push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, msg: &str, _details: PyValue) {
        self.invoke_shutdown(msg, PyValue::Dict(IndexMap::new()));
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.inner
            .lock()
            .unwrap()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.dispatch(event, params)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeHeaters;

impl PrinterObject for FakeHeaters {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn as_any_arc(self: Arc<Self>) -> Arc<dyn std::any::Any + Send + Sync> {
        self
    }
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(PyValue::Str("probe".to_string()), PyValue::Str("ready".to_string()));
        Some(PyValue::Dict(d))
    }
}

struct FakeGCode {
    reactor: Reactor,
    commands: Mutex<Vec<(String, Option<String>, bool)>>,
    handlers: Mutex<Vec<GCodeCommandHandler>>,
    scripts: Mutex<Vec<String>>,
    responded: Arc<Mutex<Vec<String>>>,
    outputs: Mutex<Vec<String>>,
    run_error: Mutex<Option<KlippyError>>,
    locked: Mutex<bool>,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            reactor,
            commands: Mutex::new(Vec::new()),
            handlers: Mutex::new(Vec::new()),
            scripts: Mutex::new(Vec::new()),
            responded: Arc::new(Mutex::new(Vec::new())),
            outputs: Mutex::new(Vec::new()),
            run_error: Mutex::new(None),
            locked: Mutex::new(false),
        })
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.commands
            .lock()
            .unwrap()
            .push((cmd.to_string(), opts.desc.clone(), opts.when_not_ready));
        if let Some(h) = func {
            self.handlers.lock().unwrap().push(h);
        }
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(PyValue::Str("commands".to_string()), PyValue::Dict(IndexMap::new()));
        PyValue::Dict(d)
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().unwrap().push(script.to_string());
        if let Some(e) = &*self.run_error.lock().unwrap() {
            return Err(e.clone());
        }
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.run_script(script)
    }
    fn get_mutex(&self) -> ReactorMutex {
        let locked = *self.locked.lock().unwrap();
        self.reactor.mutex(locked)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        let mut p = IndexMap::new();
        for (k, v) in params {
            p.insert(k.clone(), v.clone());
        }
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: p,
            responded: self.responded.clone(),
        })
    }
    fn respond_raw(&self, msg: &str) {
        self.outputs.lock().unwrap().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.responded.lock().unwrap().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        klippy_core::gcode_parse::is_traditional_gcode(cmd)
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

// The object advertises `dyn GCodeApi`, so the port resolves it with
// `lookup_interface` exactly as the reference resolves `printer.lookup_object('gcode')`.
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    /// Shared with the `FakeGCode` (Python's `gcmd.respond_info` is the
    /// bound `gcode.respond_info`).
    responded: Arc<Mutex<Vec<String>>>,
}

fn cmd_err(commandline: &str, msg: String) -> KlippyError {
    KlippyError::Command(format!("Error on '{}': {}", commandline, msg))
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GCmdGetOpts) -> KResult<PyValue> {
        let value = self.params.get(name);
        let s = match value {
            None => {
                return match opts.default {
                    None => Err(cmd_err(&self.commandline, format!("missing {}", name))),
                    Some(v) => Ok(v),
                };
            }
            Some(s) => s.clone(),
        };
        let parsed = match opts.parser {
            klippy_api::gcode::GetParser::Str => PyValue::Str(s.clone()),
            klippy_api::gcode::GetParser::Int => match pytext::int_from_string(&s, 10) {
                Ok(n) => PyValue::Int(n),
                Err(_) => {
                    return Err(cmd_err(&self.commandline, format!("unable to parse {}", s)));
                }
            },
            klippy_api::gcode::GetParser::Float => match pytext::float_from_string(&s) {
                Ok(f) => PyValue::Float(f),
                Err(_) => {
                    return Err(cmd_err(&self.commandline, format!("unable to parse {}", s)));
                }
            },
            klippy_api::gcode::GetParser::IntAutoBase => match pytext::int_from_string(&s, 0) {
                Ok(n) => PyValue::Int(n),
                Err(_) => {
                    return Err(cmd_err(&self.commandline, format!("unable to parse {}", s)));
                }
            },
        };
        let f = parsed.to_f64();
        if let (Some(fv), Some(min)) = (f, opts.minval.as_ref().and_then(|v| v.to_f64())) {
            if fv < min {
                return Err(cmd_err(
                    &self.commandline,
                    format!("{} must have minimum of {}", name, pytext::float_repr(min)),
                ));
            }
        }
        if let (Some(fv), Some(max)) = (f, opts.maxval.as_ref().and_then(|v| v.to_f64())) {
            if fv > max {
                return Err(cmd_err(
                    &self.commandline,
                    format!("{} must have maximum of {}", name, pytext::float_repr(max)),
                ));
            }
        }
        if let (Some(fv), Some(above)) = (f, opts.above.as_ref().and_then(|v| v.to_f64())) {
            if fv <= above {
                return Err(cmd_err(
                    &self.commandline,
                    format!("{} must be above {}", name, pytext::float_repr(above)),
                ));
            }
        }
        if let (Some(fv), Some(below)) = (f, opts.below.as_ref().and_then(|v| v.to_f64())) {
            if fv >= below {
                return Err(cmd_err(
                    &self.commandline,
                    format!("{} must be below {}", name, pytext::float_repr(below)),
                ));
            }
        }
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GCmdIntOpts) -> KResult<Option<i64>> {
        let v = self.get(
            name,
            GCmdGetOpts {
                default: opts.default.map(|o| o.map(|i| PyValue::Int(pycompat::int::PyInt::from_i64(i))).unwrap_or(PyValue::None)),
                parser: klippy_api::gcode::GetParser::Int,
                minval: opts.minval.map(|i| PyValue::Int(pycompat::int::PyInt::from_i64(i))),
                maxval: opts.maxval.map(|i| PyValue::Int(pycompat::int::PyInt::from_i64(i))),
                above: None,
                below: None,
            },
        )?;
        match v {
            PyValue::Int(n) => Ok(Some(n.inner().to_string().parse::<i64>().unwrap_or(0))),
            PyValue::None => Ok(None),
            _ => Ok(None),
        }
    }
    fn get_float(&self, name: &str, opts: GCmdFloatOpts) -> KResult<Option<f64>> {
        let v = self.get(
            name,
            GCmdGetOpts {
                default: opts.default.map(|o| o.map(PyValue::Float).unwrap_or(PyValue::None)),
                parser: klippy_api::gcode::GetParser::Float,
                minval: opts.minval.map(PyValue::Float),
                maxval: opts.maxval.map(PyValue::Float),
                above: opts.above.map(PyValue::Float),
                below: opts.below.map(PyValue::Float),
            },
        )?;
        match v {
            PyValue::Float(f) => Ok(Some(f)),
            PyValue::None => Ok(None),
            _ => Ok(None),
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
        let command = &self.command;
        let mut start = command.len();
        if self.commandline[..start].to_uppercase() != command.to_uppercase() {
            start += self
                .commandline
                .to_uppercase()
                .find(&command.to_uppercase())
                .unwrap_or(0);
        }
        self.commandline[start..].trim_start().to_string()
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.responded.lock().unwrap().push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.responded.lock().unwrap().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeKinematics;

impl KinematicsApi for FakeKinematics {
    fn get_steppers(&self) -> Vec<klippy_api::stepper::Stepper> {
        Vec::new()
    }
    fn calc_position(&self, _stepper_positions: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(Vec::new())
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn clear_homing_state(&self, _clear_axes: &str) {}
    fn home_axis(
        &self,
        _homing_state: klippy_api::toolhead::HomingStateHandle,
        _axis: i64,
        _rail: Rail,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_calibration(&self) -> KResult<klippy_api::toolhead::DeltaCalibration> {
        Err(KlippyError::Internal {
            class_name: "AttributeError".to_string(),
            message: "kinematics object has no attribute 'get_calibration'".to_string(),
        })
    }
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

struct FakeToolHead {
    busy: Mutex<(f64, f64, bool)>,
    last_move: Mutex<f64>,
}

impl FakeToolHead {
    fn new() -> (Arc<FakeToolHead>, ToolHead) {
        let t = Arc::new(FakeToolHead {
            busy: Mutex::new((0.0, 0.0, true)),
            last_move: Mutex::new(0.0),
        });
        let h: ToolHead = t.clone();
        (t, h)
    }
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics)
    }
    fn get_last_move_time(&self) -> f64 {
        *self.last_move.lock().unwrap()
    }
    fn dwell(&self, _delay: f64) {}
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<f64>,
    ) -> KResult<()> {
        Ok(())
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(
            PyValue::Str("current_position".to_string()),
            PyValue::List(vec![PyValue::Float(0.0); 4]),
        );
        PyValue::Dict(d)
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _callback: klippy_api::toolhead::LookaheadCallback) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        *self.busy.lock().unwrap()
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder { panic!("not used") }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> { Vec::new() }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory { panic!("not used") }
}

// The object advertises `dyn ToolHeadApi`, so the port resolves it with
// `lookup_interface` exactly as the reference resolves `printer.lookup_object('toolhead')`.
klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, options: Vec<(&str, &str)>) -> Self {
        let mut opts = IndexMap::new();
        for (k, v) in options {
            opts.insert(k.to_string(), v.to_string());
        }
        FakeConfig {
            printer,
            section: section.to_string(),
            options: opts,
        }
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, option: &str, opts: CfgGetIntOpts) -> KResult<Option<i64>> {
        match self.options.get(option) {
            Some(s) => pytext::int_from_string(s, 10).map(|n| {
                let i = n.inner().to_string().parse::<i64>().unwrap_or(0);
                Some(i)
            }).map_err(|_| KlippyError::Config(format!(
                "Unable to parse option '{}' in section '{}'",
                option, self.section
            ))),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: CfgGetFloatOpts) -> KResult<Option<f64>> {
        match self.options.get(option) {
            Some(s) => {
                let v: f64 = match pytext::float_from_string(s) {
                    Ok(f) => f,
                    Err(_) => {
                        return Err(KlippyError::Config(format!(
                            "Unable to parse option '{}' in section '{}'",
                            option, self.section
                        )));
                    }
                };
                if let Some(minv) = opts.minval {
                    if v < minv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option,
                            self.section,
                            pytext::float_repr(minv)
                        )));
                    }
                }
                if let Some(maxv) = opts.maxval {
                    if v > maxv {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option,
                            self.section,
                            pytext::float_repr(maxv)
                        )));
                    }
                }
                if let Some(a) = opts.above {
                    if v <= a {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option,
                            self.section,
                            pytext::float_repr(a)
                        )));
                    }
                }
                if let Some(b) = opts.below {
                    if v >= b {
                        return Err(KlippyError::Config(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option,
                            self.section,
                            pytext::float_repr(b)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.options.get(option) {
            Some(s) => {
                let s = s.to_ascii_lowercase();
                match s.as_str() {
                    "true" | "yes" | "on" | "1" => Ok(Some(true)),
                    "false" | "no" | "off" | "0" => Ok(Some(false)),
                    _ => Err(KlippyError::Config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    ))),
                }
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let raw = match self.options.get(option) {
            Some(s) => s.clone(),
            None => match opts.default {
                None => {
                    return Err(KlippyError::Config(format!(
                        "Option '{}' in section '{}' must be specified",
                        option, self.section
                    )));
                }
                Some(inner) => match inner {
                    Some(v) => v.str(),
                    None => String::new(),
                },
            },
        };
        match choices {
            PyValue::Dict(d) => {
                for (k, v) in d.iter() {
                    if k.str() == raw {
                        return Ok(v.clone());
                    }
                }
            }
            _ => {}
        }
        Err(KlippyError::Config(format!(
            "Choice '{}' for option '{}' in section '{}' is not a valid choice",
            raw, option, self.section
        )))
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        match self.options.get(option) {
            Some(s) => {
                let mut parts: Vec<PyValue> = Vec::new();
                for p in s.split(',') {
                    let t = p.trim().to_string();
                    match opts.parser {
                        klippy_api::config::ListParser::Str => {
                            parts.push(PyValue::Str(t));
                        }
                        klippy_api::config::ListParser::Int => match pytext::int_from_string(
                            &t,
                            10,
                        ) {
                            Ok(n) => parts.push(PyValue::Int(n)),
                            Err(_) => {
                                return Err(KlippyError::Config(format!(
                                    "Unable to parse option '{}' in section '{}'",
                                    option, self.section
                                )));
                            }
                        },
                        klippy_api::config::ListParser::Float => {
                            match pytext::float_from_string(&t) {
                                Ok(f) => parts.push(PyValue::Float(f)),
                                Err(_) => {
                                    return Err(KlippyError::Config(format!(
                                        "Unable to parse option '{}' in section '{}'",
                                        option, self.section
                                    )));
                                }
                            }
                        }
                    }
                }
                Ok(PyValue::Tuple(parts))
            }
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner.unwrap_or(PyValue::None)),
            },
        }
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.options.get(option) {
            Some(s) => Ok(Some(
                s.split(&opts.sep).map(|p| p.trim().to_string()).collect(),
            )),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        match self.options.get(option) {
            Some(s) => s
                .split(&opts.sep)
                .map(|p| {
                    pytext::int_from_string(p.trim(), 10).map(|n| {
                        n.inner().to_string().parse::<i64>().unwrap_or(0)
                    })
                })
                .collect::<Result<Vec<i64>, _>>()
                .map(Some)
                .map_err(|_| {
                    KlippyError::Config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    ))
                }),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        match self.options.get(option) {
            Some(s) => s
                .split(&opts.sep)
                .map(|p| pytext::float_from_string(p.trim()))
                .collect::<Result<Vec<f64>, _>>()
                .map(Some)
                .map_err(|_| {
                    KlippyError::Config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    ))
                }),
            None => match opts.default {
                None => Err(KlippyError::Config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            options: IndexMap::new(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        section == self.section
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

/// Shared fixture: a printer with `gcode`, `toolhead`, `gcode_macro` and
/// `heaters` objects registered.
struct Fixture {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
    reactor: Reactor,
}

fn make_fixture() -> Fixture {
    let (printer, printer_api) = FakePrinter::new();
    let reactor = printer.reactor.clone();
    let gcode = FakeGCode::new(reactor.clone());
    let (toolhead, _toolhead_api) = FakeToolHead::new();
    printer.register_object("gcode", gcode.clone() as ObjectRef);
    printer.register_object("toolhead", toolhead.clone() as ObjectRef);
    let macro_config: Config =
        Arc::new(FakeConfig::new(printer_api.clone(), "gcode_macro", vec![]));
    printer.register_object(
        "gcode_macro",
        Arc::new(PrinterGCodeMacro::new(&macro_config)) as ObjectRef,
    );
    printer.register_object("heaters", Arc::new(FakeHeaters) as ObjectRef);
    Fixture {
        printer,
        printer_api,
        gcode,
        toolhead,
        reactor,
    }
}

// ---------------------------------------------------------------------------
// Value conversion and oracle comparison helpers.
// ---------------------------------------------------------------------------

use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RValue};

fn to_rec(v: &PyValue) -> RValue {
    match v {
        PyValue::None => RValue::Null,
        PyValue::Bool(b) => RValue::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(i) => RValue::Int(i),
            Err(_) => RValue::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => RValue::float(*f),
        PyValue::Str(s) => RValue::str(s),
        PyValue::Bytes(b) => RValue::bytes(b),
        PyValue::List(l) => RValue::List(l.iter().map(to_rec).collect()),
        PyValue::Tuple(t) => RValue::Tuple(t.iter().map(to_rec).collect()),
        PyValue::Dict(d) => RValue::Dict(
            d.iter()
                .map(|(k, v)| (to_rec(k), to_rec(v)))
                .collect(),
        ),
        PyValue::Host(h) => RValue::Opaque {
            type_name: "host".to_string(),
            addr: format!("{:#x}", h),
        },
    }
}

fn event_arg_rec(a: &EventArg) -> RValue {
    match a {
        EventArg::Value(v) => to_rec(v),
        EventArg::Object(_) => RValue::Str("<object>".to_string()),
        EventArg::Any(_) => RValue::Str("<any>".to_string()),
        EventArg::List(l) => RValue::List(l.iter().map(event_arg_rec).collect()),
    }
}

fn event_rec(event: &str, args: &[EventArg]) -> RValue {
    let mut items = vec![RValue::str(event)];
    items.extend(args.iter().map(event_arg_rec));
    RValue::Tuple(items)
}

fn dummy_record(function: &str) -> Record {
    Record {
        v: 0,
        unit: "idle_timeout".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: function.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn cmp_oracle(py: &mut pyoracle::Oracle, label: &str, expr: &str, actual: RValue) {
    let expected = match py.eval(expr) {
        Ok(v) => v,
        Err(e) => panic!("oracle eval `{}` raised {}: {}", expr, e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&dummy_record(label), label, &expected, &actual) {
        panic!("parity mismatch at `{}` (expr `{}`):\n{}", label, expr, d);
    }
}

fn cmp_exc_oracle(
    py: &mut pyoracle::Oracle,
    label: &str,
    expr: &str,
    rust: &Result<(), KlippyError>,
) {
    match py.eval(expr) {
        Ok(_) => panic!("oracle eval `{}` did not raise", expr),
        Err(expected) => match rust {
            Ok(()) => panic!(
                "rust did not raise at `{}` (expected {}: {})",
                label, expected.class_name, expected.message
            ),
            Err(e) => {
                assert_eq!(e.class_name(), expected.class_name, "class at `{}`", label);
                assert_eq!(e.message(), expected.message, "message at `{}`", label);
            }
        },
    }
}

/// Float literal for inline Python expressions (`500.0`, not `500`).
fn rf(f: f64) -> String {
    format!("{:?}", f)
}
// ---------------------------------------------------------------------------
// Python oracle driver: fakes mirroring the Rust fakes call for call.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import sys
sys.path.insert(0, '__KLIPPY_EXTRAS__')
import logging
import traceback
import jinja2
LOG_LIST = []
class _LogHandler(logging.Handler):
    def emit(self, record):
        LOG_LIST.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_LogHandler())
logging.getLogger().setLevel(logging.DEBUG)
import idle_timeout
import gcode_macro as gcode_macro_mod
class Error(Exception): pass
import contextlib
from gcode import CommandError as _GCodeCommandError
NEVER = 9999999999999999.
class _Mutex:
    def test(self): return self.locked
class FakeReactor:
    NEVER = NEVER
    def __init__(self):
        self.monotonic_time = 1000.0
        self.timer_count = 0
        self.updates = []
    def register_timer(self, cb):
        self.timer_count += 1
        return ("timer", self.timer_count)
    def update_timer(self, t, waketime):
        self.updates.append(waketime)
    def monotonic(self):
        return self.monotonic_time
    def assert_no_pause(self):
        # gcode_macro.GetStatusWrapper wraps get_status in reactor.assert_no_pause() (reactor.py).
        return contextlib.nullcontext()
class FakeGCode:
    # The real class, so gcode_macro renders it as `gcode.CommandError` like klippy does.
    CommandError = _GCodeCommandError
    error = CommandError
    def __init__(self):
        self.registered = []
        self.scripts = []
        self.responded = []
        self.run_error = None
        self.mutex_locked = False
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append((cmd, desc, when_not_ready))
    def run_script(self, script):
        self.scripts.append(script)
        if self.run_error is not None:
            raise self.run_error
    def get_mutex(self):
        m = _Mutex()
        m.locked = self.mutex_locked
        return m
    def respond_info(self, msg, log=True):
        self.responded.append(msg)
class FakeToolhead:
    def __init__(self):
        self.busy = (0.0, 0.0, True)
        self.last_move = 0.0
    def check_busy(self, eventtime):
        return self.busy
    def get_last_move_time(self):
        return self.last_move
class FakeHeaters:
    def get_status(self, eventtime):
        return {'probe': 'ready'}
_sentinel = object()
class FakePrinter:
    config_error = Error
    command_error = FakeGCode.CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.events = []
        self.reactor = FakeReactor()
        self.shutdown = False
        self.shutdown_msgs = []
    def get_reactor(self):
        return self.reactor
    def is_shutdown(self):
        return self.shutdown
    def invoke_shutdown(self, msg, details=None):
        self.shutdown = True
        self.shutdown_msgs.append(msg)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        self.events.append((event,) + tuple(params))
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise Error("Unknown config object '%s'" % name)
        return default
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if section == 'gcode_macro':
            obj = gcode_macro_mod.load_config(FakeConfig(self, 'gcode_macro', {}))
            self.objects[section] = obj
            return obj
        if default is _sentinel:
            raise Error("Unable to load module '%s'" % section)
        return default
class FakeConfig:
    def __init__(self, printer, section, options):
        self.printer = printer
        self.section = section
        self.options = dict(options)
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def get(self, option, default=_sentinel):
        if option in self.options:
            return self.options[option]
        if default is _sentinel:
            raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return default
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self.options:
            if default is _sentinel:
                raise Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        try:
            v = float(self.options[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v
class FakeGCmd:
    error = FakeGCode.CommandError
    def __init__(self, printer, commandline, params):
        self._commandline = commandline
        self._command = commandline.split(' ', 1)[0]
        self._params = dict(params)
        self._printer = printer
    def get_commandline(self):
        return self._commandline
    def respond_info(self, msg, log=True):
        self._printer.objects['gcode'].respond_info(msg, log)
    def get_float(self, name, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        raw = value
        try:
            value = float(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, raw))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value
"#;

fn py_setup(py: &mut pyoracle::Oracle) {
    let code = PY_SETUP.replace("__KLIPPY_EXTRAS__", &klippy_extras_path());
    if let Err(e) = py.exec(&code) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
}

fn py_str_lit(s: &str) -> String {
    format!("{:?}", s)
}

/// Builds the standard Python side: printer, gcode, toolhead, heaters,
/// config and the `IdleTimeout` instance.
fn py_build(timeout_opt: Option<&str>, gcode_opt: Option<&str>) -> String {
    let mut opts = String::from("{");
    let mut first = true;
    if let Some(t) = timeout_opt {
        opts.push_str(&format!("'timeout': {}", py_str_lit(t)));
        first = false;
    }
    if let Some(g) = gcode_opt {
        if !first {
            opts.push_str(", ");
        }
        opts.push_str(&format!("'gcode': {}", py_str_lit(g)));
    }
    opts.push('}');
    format!(
        "\nprinter = FakePrinter()\n\
         printer.objects['gcode'] = FakeGCode()\n\
         printer.objects['toolhead'] = FakeToolhead()\n\
         printer.objects['heaters'] = FakeHeaters()\n\
         cfg = FakeConfig(printer, 'idle_timeout', {})\n\
         it = idle_timeout.IdleTimeout(cfg)\n\
         printer.objects['idle_timeout'] = it\n",
        opts
    )
}

// ---------------------------------------------------------------------------
// Local structural tests (no oracle numbers embedded).
// ---------------------------------------------------------------------------

#[test]
fn factory_registers_command_and_ready_handler() {
    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix.printer_api.clone(),
        "idle_timeout",
        vec![("timeout", "42")],
    ));
    let f = module_factory();
    assert!(f.load_config.is_some());
    let obj = (f.load_config.unwrap())(&cfg).expect("load_config").unwrap();
    let registered = fix.gcode.commands.lock().unwrap().clone();
    assert_eq!(
        registered,
        vec![(
            "SET_IDLE_TIMEOUT".to_string(),
            Some("Set the idle timeout in seconds".to_string()),
            false
        )]
    );
    let has_ready = fix
        .printer
        .inner
        .lock()
        .unwrap()
        .handlers
        .contains_key("klippy:ready");
    assert!(has_ready, "klippy:ready handler registered");

    let it = klippy_api::downcast_arc::<IdleTimeout>(&obj).expect("object is IdleTimeout");
    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SET_IDLE_TIMEOUT".to_string(),
        commandline: "SET_IDLE_TIMEOUT TIMEOUT=61.5".to_string(),
        params: IndexMap::from([("TIMEOUT".to_string(), "61.5".to_string())]),
        responded: fix.gcode.responded.clone(),
    });
    let mut handler = {
        let mut h = fix.gcode.handlers.lock().unwrap();
        h.pop().expect("one handler stored")
    };
    handler(&gcmd).expect("handler runs");
    let responded = fix.gcode.responded.lock().unwrap().clone();
    assert_eq!(responded.len(), 1);
    assert!(responded[0].starts_with("idle_timeout: Timeout set to "));
    assert!(responded[0].ends_with(" s"));
    assert_eq!(it.idle_timeout(), 61.5);
    let status = it.get_status(0.0).expect("get_status present");
    match &status {
        PyValue::Dict(d) => {
            let keys: Vec<String> = d.keys().map(|k| k.str()).collect();
            assert_eq!(keys, vec!["state", "printing_time", "idle_timeout"]);
        }
        _ => panic!("get_status must be a dict"),
    }
    assert_eq!(it.state(), "Idle");
}

#[test]
fn missing_gcode_object_fails() {
    let (_printer, printer_api) = FakePrinter::new();
    let cfg: Config = Arc::new(FakeConfig::new(printer_api, "idle_timeout", vec![]));
    match IdleTimeout::new(&cfg) {
        Ok(_) => panic!("expected config error"),
        Err(e) => {
            assert_eq!(e.class_name(), "Error");
            assert_eq!(e.message(), "Unknown config object 'gcode'");
        }
    }
}

// ---------------------------------------------------------------------------
// Oracle parity tests.
// ---------------------------------------------------------------------------

#[test]
fn oracle_construction_timeout_and_status() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    py.exec(&py_build(None, None)).unwrap();
    cmp_oracle(
        &mut py,
        "status default",
        "it.get_status(123.0)",
        to_rec(&it.get_status(123.0).expect("status")),
    );
    cmp_oracle(&mut py, "registered", "printer.objects['gcode'].registered", {
        let reg = fix.gcode.commands.lock().unwrap();
        RValue::List(
            reg.iter()
                .map(|(cmd, desc, wnr)| {
                    RValue::Tuple(vec![
                        RValue::str(cmd),
                        desc.as_ref().map(|s| RValue::str(s)).unwrap_or(RValue::Null),
                        RValue::Bool(*wnr),
                    ])
                })
                .collect(),
        )
    });

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix.printer_api.clone(),
        "idle_timeout",
        vec![("timeout", "321.5")],
    ));
    let it = IdleTimeout::new(&cfg).expect("construct");
    py.exec(&py_build(Some("321.5"), None)).unwrap();
    cmp_oracle(
        &mut py,
        "status timeout",
        "it.get_status(0.5)",
        to_rec(&it.get_status(0.5).expect("status")),
    );
    cmp_oracle(&mut py, "timeout field", "it.idle_timeout", RValue::float(it.idle_timeout()));
}

#[test]
fn oracle_timeout_config_errors() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    for timeout in ["0", "-3.5", "soon"] {
        let fix = make_fixture();
        let cfg: Config = Arc::new(FakeConfig::new(
            fix.printer_api.clone(),
            "idle_timeout",
            vec![("timeout", timeout)],
        ));
        let rust = IdleTimeout::new(&cfg).map(|_| ()).map_err(|e| e);
        py.exec("printer = FakePrinter()\nprinter.objects['gcode'] = FakeGCode()\nprinter.objects['toolhead'] = FakeToolhead()\nprinter.objects['heaters'] = FakeHeaters()\n")
            .unwrap();
        cmp_exc_oracle(
            &mut py,
            &format!("timeout {}", timeout),
            &format!(
                "idle_timeout.IdleTimeout(FakeConfig(printer, 'idle_timeout', {{'timeout': {}}}))",
                py_str_lit(timeout)
            ),
            &rust,
        );
    }
}

#[test]
fn oracle_template_syntax_error() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    let bad = "{% if %}\nTURN_OFF_HEATERS\n{% endif %}\n";
    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix.printer_api.clone(),
        "idle_timeout",
        vec![("gcode", bad)],
    ));
    let rust = IdleTimeout::new(&cfg).map(|_| ());
    py.exec("printer = FakePrinter()\nprinter.objects['gcode'] = FakeGCode()\nprinter.objects['toolhead'] = FakeToolhead()\nprinter.objects['heaters'] = FakeHeaters()\n")
        .unwrap();
    // `cmp_exc_oracle` would compare the full message, but the
    // diagnostic after `# ` is Jinja2's own wording
    // (`gcode_macro.py:55-60`), not `idle_timeout`'s: Jinja2 says
    // "Expected an expression, got 'end of statement block'" for
    // `{% if %}`, the Rust template engine says "unexpected end of
    // expression". That wording is owned by the `gcode_macro`
    // template engine, so compare the error class and the
    // `TemplateWrapper` prefix that `idle_timeout` itself produces
    // (template name, line number and source line), and require
    // both sides to carry a non-empty engine diagnostic.
    let expr = format!(
        "idle_timeout.IdleTimeout(FakeConfig(printer, 'idle_timeout', {{'gcode': {}}}))",
        py_str_lit(bad)
    );
    let expected = match py.eval(&expr) {
        Ok(_) => panic!("oracle eval `{expr}` did not raise"),
        Err(e) => e,
    };
    let rust_err = match rust {
        Ok(()) => panic!(
            "rust did not raise at `template syntax` (expected {}: {})",
            expected.class_name, expected.message
        ),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        expected.class_name,
        "class at `template syntax`"
    );
    let prefix = format!(
        "Error loading template 'idle_timeout:gcode'\nline 1: {} # ",
        bad.lines().next().unwrap_or("")
    );
    assert!(
        expected.message.starts_with(&prefix),
        "oracle message at `template syntax`: {:?}",
        expected.message
    );
    assert!(
        rust_err.message().starts_with(&prefix),
        "rust message at `template syntax`: {:?}",
        rust_err.message()
    );
    assert!(
        !expected.message[prefix.len()..].is_empty(),
        "oracle message has no engine diagnostic"
    );
    assert!(
        !rust_err.message()[prefix.len()..].is_empty(),
        "rust message has no engine diagnostic"
    );
}

#[test]
fn oracle_default_render_via_transition() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    py.exec(&py_build(None, None)).unwrap();

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    fix.printer.inner.lock().unwrap().events.clear();

    // Long-idle toolhead, mutex free: Idle -> Ready -> Idle.
    fix.toolhead.busy.lock().unwrap().clone_from(&(0.0, 601.0, true));
    *fix.toolhead.last_move.lock().unwrap() = 123.0;
    py.exec("printer.objects['toolhead'].busy = (0.0, 601.0, True)\nprinter.objects['toolhead'].last_move = 123.0")
        .unwrap();
    py.exec("it.handle_ready()").unwrap();

    let t1 = 2000.0f64;
    let rust_wake = it.timeout_handler(t1);
    cmp_oracle(&mut py, "timeout wake", &format!("it.timeout_handler({})", rf(t1)), RValue::float(rust_wake));
    cmp_oracle(&mut py, "state ready", "it.state", RValue::str(&it.state()));
    let t2 = 2601.0f64;
    let rust_wake = it.check_idle_timeout(t2);
    cmp_oracle(&mut py, "check wake", &format!("it.check_idle_timeout({})", rf(t2)), RValue::float(rust_wake));
    cmp_oracle(&mut py, "state idle", "it.state", RValue::str(&it.state()));
    cmp_oracle(
        &mut py,
        "scripts",
        "printer.objects['gcode'].scripts",
        RValue::List(
            fix.gcode
                .scripts
                .lock()
                .unwrap()
                .iter()
                .map(|s| RValue::str(s))
                .collect(),
        ),
    );
    cmp_oracle(
        &mut py,
        "events",
        "printer.events",
        RValue::List(
            fix.printer
                .inner
                .lock()
                .unwrap()
                .events
                .iter()
                .map(|(e, args)| event_rec(e, args))
                .collect(),
        ),
    );
}

#[test]
fn oracle_custom_gcode_render() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    py.exec(&py_build(None, Some("M105\nM114\n"))).unwrap();

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix.printer_api.clone(),
        "idle_timeout",
        vec![("gcode", "M105\nM114\n")],
    ));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    fix.toolhead.busy.lock().unwrap().clone_from(&(0.0, 601.0, true));
    py.exec("printer.objects['toolhead'].busy = (0.0, 601.0, True)\nit.handle_ready()").unwrap();
    let rust_wake = it.transition_idle_state(3000.0);
    cmp_oracle(&mut py, "transition wake", "it.transition_idle_state(3000.0)", RValue::float(rust_wake));
    cmp_oracle(
        &mut py,
        "scripts",
        "printer.objects['gcode'].scripts",
        RValue::List(
            fix.gcode
                .scripts
                .lock()
                .unwrap()
                .iter()
                .map(|s| RValue::str(s))
                .collect(),
        ),
    );
}

#[test]
fn oracle_cmd_set_idle_timeout() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    py.exec(&py_build(None, None)).unwrap();

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    py.exec("it.handle_ready()").unwrap();

    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SET_IDLE_TIMEOUT".to_string(),
        commandline: "SET_IDLE_TIMEOUT TIMEOUT=123.456".to_string(),
        params: IndexMap::from([("TIMEOUT".to_string(), "123.456".to_string())]),
        responded: fix.gcode.responded.clone(),
    });
    it.cmd_set_idle_timeout(&gcmd).expect("cmd");
    py.exec("it.cmd_SET_IDLE_TIMEOUT(FakeGCmd(printer, 'SET_IDLE_TIMEOUT TIMEOUT=123.456', {'TIMEOUT': '123.456'}))")
        .unwrap();
    cmp_oracle(
        &mut py,
        "respond TIMEOUT=123.456",
        "printer.objects['gcode'].responded",
        RValue::List(
            fix.gcode
                .responded
                .lock()
                .unwrap()
                .iter()
                .map(|s| RValue::str(s))
                .collect(),
        ),
    );
    cmp_oracle(&mut py, "timeout value", "it.idle_timeout", RValue::float(it.idle_timeout()));

    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SET_IDLE_TIMEOUT".to_string(),
        commandline: "SET_IDLE_TIMEOUT".to_string(),
        params: IndexMap::new(),
        responded: fix.gcode.responded.clone(),
    });
    it.cmd_set_idle_timeout(&gcmd).expect("cmd");
    py.exec("it.cmd_SET_IDLE_TIMEOUT(FakeGCmd(printer, 'SET_IDLE_TIMEOUT', {}))").unwrap();
    cmp_oracle(
        &mut py,
        "respond default",
        "printer.objects['gcode'].responded",
        RValue::List(
            fix.gcode
                .responded
                .lock()
                .unwrap()
                .iter()
                .map(|s| RValue::str(s))
                .collect(),
        ),
    );

    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SET_IDLE_TIMEOUT".to_string(),
        commandline: "SET_IDLE_TIMEOUT TIMEOUT=0".to_string(),
        params: IndexMap::from([("TIMEOUT".to_string(), "0".to_string())]),
        responded: fix.gcode.responded.clone(),
    });
    let rust = it.cmd_set_idle_timeout(&gcmd).map(|_| ());
    cmp_exc_oracle(
        &mut py,
        "TIMEOUT=0",
        "it.cmd_SET_IDLE_TIMEOUT(FakeGCmd(printer, 'SET_IDLE_TIMEOUT TIMEOUT=0', {'TIMEOUT': '0'}))",
        &rust.map_err(|e| e),
    );

    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SET_IDLE_TIMEOUT".to_string(),
        commandline: "SET_IDLE_TIMEOUT TIMEOUT=soon".to_string(),
        params: IndexMap::from([("TIMEOUT".to_string(), "soon".to_string())]),
        responded: fix.gcode.responded.clone(),
    });
    let rust = it.cmd_set_idle_timeout(&gcmd).map(|_| ());
    cmp_exc_oracle(
        &mut py,
        "TIMEOUT=soon",
        "it.cmd_SET_IDLE_TIMEOUT(FakeGCmd(printer, 'SET_IDLE_TIMEOUT TIMEOUT=soon', {'TIMEOUT': 'soon'}))",
        &rust.map_err(|e| e),
    );
}

#[test]
fn oracle_sync_print_time_and_status() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    py.exec(&py_build(None, None)).unwrap();

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    py.exec("it.handle_ready()").unwrap();

    // Sender order: (curtime, est_print_time, print_time).
    let (curtime, est, print) = (500.0f64, 501.5f64, 500.75f64);
    fix.printer
        .run_handlers(
            "toolhead:sync_print_time",
            &[
                EventArg::Value(PyValue::Float(curtime)),
                EventArg::Value(PyValue::Float(est)),
                EventArg::Value(PyValue::Float(print)),
            ],
        )
        .expect("sync");
    py.exec(&format!(
        "printer.send_event('toolhead:sync_print_time', {}, {}, {})",
        rf(curtime),
        rf(est),
        rf(print)
    ))
    .unwrap();

    cmp_oracle(
        &mut py,
        "sync events",
        "printer.events",
        RValue::List(
            fix.printer
                .inner
                .lock()
                .unwrap()
                .events
                .iter()
                .map(|(e, args)| event_rec(e, args))
                .collect(),
        ),
    );
    cmp_oracle(&mut py, "state after sync", "it.state", RValue::str(&it.state()));
    cmp_oracle(
        &mut py,
        "status printing",
        "it.get_status(600.0)",
        to_rec(&it.get_status(600.0).expect("status")),
    );
    cmp_oracle(
        &mut py,
        "last_print_start",
        "it.last_print_start_systime",
        RValue::float(it.last_print_start_systime()),
    );

    // Second sync while Printing is ignored (state and clock unchanged).
    fix.printer
        .run_handlers(
            "toolhead:sync_print_time",
            &[
                EventArg::Value(PyValue::Float(600.0)),
                EventArg::Value(PyValue::Float(601.0)),
                EventArg::Value(PyValue::Float(600.5)),
            ],
        )
        .expect("sync2");
    py.exec("printer.send_event('toolhead:sync_print_time', 600.0, 601.0, 600.5)").unwrap();
    cmp_oracle(
        &mut py,
        "status still printing",
        "it.get_status(610.0)",
        to_rec(&it.get_status(610.0).expect("status")),
    );
    cmp_oracle(
        &mut py,
        "last_print_start still",
        "it.last_print_start_systime",
        RValue::float(it.last_print_start_systime()),
    );
}

/// Drives one `timeout_handler` step with the same inputs on both sides
/// and checks `(return value, state)`.
fn timeout_step(
    py: &mut pyoracle::Oracle,
    label: &str,
    fix: &Fixture,
    it: &Arc<IdleTimeout>,
    eventtime: f64,
    busy: (f64, f64, bool),
    locked: bool,
    shutdown: bool,
) {
    fix.toolhead.busy.lock().unwrap().clone_from(&busy);
    *fix.gcode.locked.lock().unwrap() = locked;
    fix.printer.inner.lock().unwrap().shutdown = shutdown;
    py.exec(&format!(
        "printer.objects['toolhead'].busy = ({}, {}, {})\nprinter.objects['gcode'].mutex_locked = {}\nprinter.shutdown = {}",
        rf(busy.0),
        rf(busy.1),
        if busy.2 { "True" } else { "False" },
        if locked { "True" } else { "False" },
        if shutdown { "True" } else { "False" }
    ))
    .unwrap();
    let rust_wake = it.timeout_handler(eventtime);
    cmp_oracle(
        py,
        label,
        &format!("it.timeout_handler({})", rf(eventtime)),
        RValue::float(rust_wake),
    );
    cmp_oracle(py, &format!("{} state", label), "it.state", RValue::str(&it.state()));
}

#[test]
fn oracle_timeout_handler_paths() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    py.exec(&py_build(None, None)).unwrap();

    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    py.exec("it.handle_ready()").unwrap();

    // Shutdown guard: returns NEVER without touching state.
    timeout_step(&mut py, "shutdown", &fix, &it, 4000.0, (0.0, 601.0, true), false, true);

    // Printing state, toolhead busy (!empty): eventtime + 0.5 + max(0, buffer).
    timeout_step(&mut py, "printing busy", &fix, &it, 4100.0, (100.0, 100.5, false), false, false);

    // Printing state, buffer in (-0.5, ...]: eventtime + 0.5 + buffer.
    timeout_step(&mut py, "printing buffer", &fix, &it, 4200.0, (100.0, 100.25, true), false, false);

    // Printing state, ready qualified but gcode mutex busy.
    timeout_step(&mut py, "printing mutex", &fix, &it, 4300.0, (0.0, 601.0, true), true, false);

    // Printing state, ready qualified and mutex free: -> Ready + event.
    timeout_step(&mut py, "printing ready", &fix, &it, 4400.0, (0.0, 601.0, true), false, false);
    cmp_oracle(
        &mut py,
        "ready events",
        "printer.events",
        RValue::List(
            fix.printer
                .inner
                .lock()
                .unwrap()
                .events
                .iter()
                .map(|(e, args)| event_rec(e, args))
                .collect(),
        ),
    );

    // check_idle_timeout in Ready: busy (!empty) -> eventtime + idle.
    timeout_step(&mut py, "ready busy", &fix, &it, 4500.0, (0.0, 601.0, false), false, false);

    // check_idle_timeout in Ready: idle_time < 1 -> eventtime + idle.
    timeout_step(&mut py, "ready short idle", &fix, &it, 4600.0, (100.0, 100.5, true), false, false);

    // check_idle_timeout in Ready: 1 <= idle_time < idle_timeout.
    timeout_step(&mut py, "ready waiting", &fix, &it, 4700.0, (0.0, 300.0, true), false, false);

    // check_idle_timeout in Ready: idle >= idle_timeout, mutex busy.
    timeout_step(&mut py, "ready mutex", &fix, &it, 4800.0, (0.0, 601.0, true), true, false);
}

#[test]
fn oracle_transition_failures_and_logs() {
    let Some(mut py) = pyoracle::Oracle::start() else {
        println!("skipping (no KLIPPY_PYTHON)");
        return;
    };
    py_setup(&mut py);
    install_log_sink();

    // run_script raising -> Ready, eventtime + 1, "idle timeout gcode execution" log.
    py.exec(&py_build(None, None)).unwrap();
    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(fix.printer_api.clone(), "idle_timeout", vec![]));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");
    py.exec("it.handle_ready()").unwrap();

    *fix.gcode.run_error.lock().unwrap() = Some(KlippyError::Command("simulated gcode error".to_string()));
    py.exec("printer.objects['gcode'].run_error = FakeGCode.CommandError('simulated gcode error')")
        .unwrap();
    let rust_wake = it.transition_idle_state(5000.0);
    cmp_oracle(
        &mut py,
        "transition run error wake",
        "it.transition_idle_state(5000.0)",
        RValue::float(rust_wake),
    );
    cmp_oracle(&mut py, "state ready after error", "it.state", RValue::str(&it.state()));
    assert!(
        has_log("ERROR", "idle timeout gcode execution"),
        "run_script failure logs idle timeout gcode execution"
    );
    cmp_oracle(
        &mut py,
        "python log",
        "'idle timeout gcode execution' in [m for (l, m) in LOG_LIST]",
        RValue::Bool(true),
    );

    // Render raising (action_raise_error in the template).
    py.exec(&py_build(None, Some("{action_raise_error('boom')}"))).unwrap();
    let fix2 = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix2.printer_api.clone(),
        "idle_timeout",
        vec![("gcode", "{action_raise_error('boom')}")],
    ));
    let it2 = IdleTimeout::new(&cfg).expect("construct");
    it2.handle_ready(it2.clone()).expect("handle_ready");
    let rust_wake = it2.transition_idle_state(5100.0);
    cmp_oracle(
        &mut py,
        "transition render error wake",
        "it.transition_idle_state(5100.0)",
        RValue::float(rust_wake),
    );
    cmp_oracle(&mut py, "state ready after render error", "it.state", RValue::str(&it2.state()));
    // The expected line comes from the reference's own log record.
    let py_render_log = match py.eval("[m for (l, m) in LOG_LIST if m.startswith('Error evaluating')][-1]") {
        Ok(RValue::Str(s)) => s,
        other => panic!("oracle render log: {other:?}"),
    };
    assert!(
        has_log("ERROR", py_render_log.trim_end_matches('\n')),
        "render error log {py_render_log:?} missing from {:?}",
        captured_logs()
    );
    assert!(has_log("ERROR", "idle timeout gcode execution"));
    cmp_oracle(
        &mut py,
        "python render log",
        "any('Error evaluating' in m for (l, m) in LOG_LIST)",
        RValue::Bool(true),
    );
}

// ---------------------------------------------------------------------------
// Reactor smoke: the registered timer callback fires through the real
// reactor dispatch and drives the whole lifecycle.
// ---------------------------------------------------------------------------

#[test]
fn reactor_timer_smoke() {
    let fix = make_fixture();
    let cfg: Config = Arc::new(FakeConfig::new(
        fix.printer_api.clone(),
        "idle_timeout",
        vec![("timeout", "0.8")],
    ));
    let it = IdleTimeout::new(&cfg).expect("construct");
    it.handle_ready(it.clone()).expect("handle_ready");

    // Qualified toolhead: (print, est, empty) with est - print = 601.
    *fix.toolhead.busy.lock().unwrap() = (0.0, 601.0, true);
    *fix.toolhead.last_move.lock().unwrap() = 123.0;

    // Sync with curtime 0.4s ahead and sender_est - sender_print = -0.2
    // so the registered timer wakes about 0.2s after run starts.
    let t0 = fix.reactor.monotonic();
    it.handle_sync_print_time(&[
        EventArg::Value(PyValue::Float(t0 + 0.4)),
        EventArg::Value(PyValue::Float(0.0)),
        EventArg::Value(PyValue::Float(0.2)),
    ])
    .expect("sync");

    let r2 = fix.reactor.clone();
    let end_at = t0 + 3.0;
    fix.reactor.register_timer(
        move |_| {
            r2.end();
            NEVER
        },
        end_at,
    );
    fix.reactor.run();

    let scripts = fix.gcode.scripts.lock().unwrap();
    assert_eq!(scripts.len(), 1, "idle gcode runs exactly once");
    assert!(
        scripts[0].contains("TURN_OFF_HEATERS") && scripts[0].contains("M84"),
        "default script rendered with heaters present"
    );
    let events = &fix.printer.inner.lock().unwrap().events;
    assert!(events.iter().any(|(e, _)| e == "idle_timeout:ready"));
    assert_eq!(events.iter().filter(|(e, _)| e == "idle_timeout:idle").count(), 1);
    assert_eq!(it.state(), "Idle");
}
