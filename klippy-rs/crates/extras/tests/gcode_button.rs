//! Parity tests for `klippy/extras/gcode_button.py`.
//!
//! Everything is compared against the live reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`): a fresh
//! `GCodeButton` is built on both sides with minimal Python/Rust
//! fakes for the printer, config and the `buttons`, `gcode_macro`
//! and `gcode` objects, fed identical config sections, and the
//! observed behaviour is compared exactly:
//!
//! - the button registration (`register_debounce_button` /
//!   `register_debounce_adc_button` arguments, including the
//!   `analog_range` / `analog_pullup_resistor` defaults),
//! - the template loads (template names and sources read from the
//!   `press_gcode` / `release_gcode` options, including the
//!   required/missing-option errors),
//! - the `QUERY_BUTTON` mux registration and its `respond_info`
//!   output before and after press/release events,
//! - `get_status` and the `run_script` calls after button events
//!   (including the whitespace skip and the render/run error logs),
//! - every construction error path (missing `pin`, missing
//!   `press_gcode`, bad `analog_range` / `analog_pullup_resistor`,
//!   and registering before template loading).
//!
//! The `gcode_macro` object is the real one on both sides
//! (`extras.gcode_macro.PrinterGCodeMacro` and
//! `extras::gcode_macro::PrinterGCodeMacro`), so template rendering
//! is the real Jinja engine on both sides; the templates used here
//! stay inside the subset the Rust engine reproduces (plain G-code,
//! whitespace-only expressions and `action_raise_error`).
//!
//! The Python side uses the real `configfile.ConfigWrapper`, so all
//! config error texts are the reference ones; the Rust `FakeConfig`
//! mirrors `klippy/configfile.py` for the same getters.

use extras::buttons::{
    AdcButtonCallback, ButtonCallback, ButtonState, DebounceCallback, PrinterButtonsApi,
    PushCallback,
};
use extras::gcode_button::{GCodeButton, CMD_QUERY_BUTTON_HELP};
use extras::gcode_macro::PrinterGCodeMacro;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GCmdFloatOpts,
    GetIntOpts as GCmdIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use kos::sync::Mutex;
use pycompat::text::{float_from_string, float_repr};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::BTreeMap;
use std::sync::Arc;

const SETUP: &str = r#"
import extras.gcode_button, extras.gcode_macro
import configfile, configparser
import logging as _logging

log_lines = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_lines.append((record.levelname, record.getMessage()))
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

class CommandError(Exception):
    pass
CommandError.__module__ = "gcode"

class Error(configparser.Error):
    pass

class FakeButtons:
    def __init__(self, fail=None):
        self.regs = []
        self.callbacks = []
        self.fail = fail
    def register_debounce_button(self, pin, callback, config):
        if self.fail is not None:
            raise Error(self.fail)
        self.regs.append(('debounce', pin))
        self.callbacks.append(callback)
    def register_debounce_adc_button(self, pin, amin, amax, pullup, callback, config):
        if self.fail is not None:
            raise Error(self.fail)
        self.regs.append(('adc', pin, amin, amax, pullup))
        self.callbacks.append(callback)

class FakeGCodeCmd:
    def __init__(self):
        self.infos = []
    def respond_info(self, msg, log=True):
        self.infos.append((msg, log))

class FakeGCode:
    error = CommandError
    def __init__(self, run_error=None):
        self.mux = []
        self.scripts = []
        self.run_error = run_error
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc, func))
    def run_script(self, script):
        if self.run_error is not None:
            raise CommandError(self.run_error)
        self.scripts.append(script)

_sentinel = object()
class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, buttons, gcode):
        self.objects = {'buttons': buttons, 'gcode': gcode}
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        if default is _sentinel:
            raise configparser.Error("Unable to load module '%s'" % (section,))
        return default
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

H = {}
def h_init(section, config_text, buttons_fail, run_error):
    global H
    log_lines.clear()
    buttons = FakeButtons(buttons_fail)
    gcode = FakeGCode(run_error)
    printer = FakePrinter(buttons, gcode)
    fc = configparser.RawConfigParser()
    fc.read_string(config_text)
    cfg = configfile.ConfigWrapper(printer, fc, {}, section)
    printer.objects['gcode_macro'] = extras.gcode_macro.PrinterGCodeMacro(cfg)
    try:
        btn = extras.gcode_button.GCodeButton(cfg)
    except Exception as e:
        H = {'err': (type(e).__name__, str(e)), 'btn': None, 'buttons': buttons, 'gcode': gcode, 'cfg': cfg}
        return H['err']
    H = {'btn': btn, 'buttons': buttons, 'gcode': gcode, 'cfg': cfg}
    return 'ok'
def h_event(eventtime, state):
    H['buttons'].callbacks[0](eventtime, state)
    return H['btn'].get_status()
def h_query():
    gcmd = FakeGCodeCmd()
    H['gcode'].mux[0][4](gcmd)
    return gcmd.infos
def h_status():
    return H['btn'].get_status()
def h_obs():
    btn = H.get('btn')
    templates = []
    if btn is not None:
        templates = [[btn.press_template.name, H['cfg'].get('press_gcode')],
                     [btn.release_template.name, H['cfg'].get('release_gcode', '')]]
    return {'buttons': H['buttons'].regs, 'templates': templates,
            'mux': [[m[0], m[1], m[2], m[3]] for m in H['gcode'].mux],
            'scripts': H['gcode'].scripts, 'logs': list(log_lines)}
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fresh_reactor() -> Reactor {
    Reactor::new(false)
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("gcode_button fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping gcode_button oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// Captured log lines of the Rust side on this test's thread; the sink is
// installed once per process and parallel tests keep their own lines.
thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                ));
            });
        }));
    });
}

fn log_capture() {
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
}

fn captured_logs() -> Vec<(String, String)> {
    LOGS.with(|logs| logs.borrow().clone())
}

#[derive(Clone, Debug)]
enum ButtonReg {
    Digital { pin: String },
    Adc { pin: String, amin: f64, amax: f64, pullup: f64 },
}

struct FakeButtons {
    regs: Mutex<Vec<ButtonReg>>,
    callback: Mutex<Option<DebounceCallback>>,
    fail: Option<KlippyError>,
}

impl FakeButtons {
    fn new(fail: Option<KlippyError>) -> FakeButtons {
        FakeButtons { regs: Mutex::new(Vec::new()), callback: Mutex::new(None), fail }
    }

    fn fire(&self, eventtime: f64, state: ButtonState) {
        let mut guard = self.callback.lock();
        let cb = guard.as_mut().expect("button callback registered");
        cb(eventtime, state);
    }

    fn regs_value(&self) -> Value {
        Value::List(
            self.regs
                .lock()
                .iter()
                .map(|r| match r {
                    ButtonReg::Digital { pin } => {
                        Value::Tuple(vec![Value::str("debounce"), Value::str(pin)])
                    }
                    ButtonReg::Adc { pin, amin, amax, pullup } => Value::Tuple(vec![
                        Value::str("adc"),
                        Value::str(pin),
                        Value::float(*amin),
                        Value::float(*amax),
                        Value::float(*pullup),
                    ]),
                })
                .collect(),
        )
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
        panic!("unused")
    }

    fn register_debounce_button(
        &self,
        pin: &str,
        callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        if let Some(e) = &self.fail {
            return Err(e.clone());
        }
        self.regs.lock().push(ButtonReg::Digital { pin: pin.to_string() });
        *self.callback.lock() = Some(callback);
        Ok(())
    }

    fn register_debounce_adc_button(
        &self,
        pin: &str,
        amin: f64,
        amax: f64,
        pullup: f64,
        callback: DebounceCallback,
        _config: &Config,
    ) -> KResult<()> {
        if let Some(e) = &self.fail {
            return Err(e.clone());
        }
        self.regs.lock().push(ButtonReg::Adc {
            pin: pin.to_string(),
            amin,
            amax,
            pullup,
        });
        *self.callback.lock() = Some(callback);
        Ok(())
    }

    fn register_adc_button_push(
        &self,
        _pin: &str,
        _min_val: f64,
        _max_val: f64,
        _pullup: f64,
        _callback: PushCallback,
    ) -> KResult<()> {
        panic!("unused")
    }

    fn register_buttons(&self, _pins: Vec<String>, _callback: ButtonCallback) -> KResult<()> {
        panic!("unused")
    }

    fn register_rotary_encoder(
        &self,
        _pin1: &str,
        _pin2: &str,
        _cw_callback: PushCallback,
        _ccw_callback: PushCallback,
        _steps_per_detent: i64,
    ) -> KResult<()> {
        panic!("unused")
    }

    fn register_button_push(&self, _pin: &str, _callback: PushCallback) -> KResult<()> {
        panic!("unused")
    }
}

klippy_api::printer_object!(FakeButtons; dyn PrinterButtonsApi);

struct FakeGCmd {
    infos: Mutex<Vec<(String, bool)>>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<PyValue> {
        panic!("unused")
    }
    fn get_int(&self, _name: &str, _opts: GCmdIntOpts) -> KResult<Option<i64>> {
        panic!("unused")
    }
    fn get_float(&self, _name: &str, _opts: GCmdFloatOpts) -> KResult<Option<f64>> {
        panic!("unused")
    }
    fn get_command(&self) -> String {
        panic!("unused")
    }
    fn get_commandline(&self) -> String {
        panic!("unused")
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        panic!("unused")
    }
    fn get_raw_command_parameters(&self) -> String {
        panic!("unused")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("unused")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("unused")
    }
}

struct FakeGCode {
    mux: Mutex<Vec<(String, String, Option<String>, Option<String>)>>,
    handler: Mutex<Option<GCodeCommandHandler>>,
    scripts: Mutex<Vec<String>>,
    run_error: Option<KlippyError>,
    reactor: Reactor,
}

impl FakeGCode {
    fn mux_value(&self) -> Value {
        Value::List(
            self.mux
                .lock()
                .iter()
                .map(|(cmd, key, value, desc)| {
                    let value = match value {
                        None => Value::Null,
                        Some(v) => Value::str(v),
                    };
                    let desc = match desc {
                        None => Value::Null,
                        Some(v) => Value::str(v),
                    };
                    Value::List(vec![Value::str(cmd), Value::str(key), value, desc])
                })
                .collect(),
        )
    }

    fn scripts_value(&self) -> Value {
        Value::List(self.scripts.lock().iter().map(|s| Value::str(s)).collect())
    }

}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("unused")
    }

    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(|s| s.to_string()),
            desc.map(|s| s.to_string()),
        ));
        *self.handler.lock() = Some(func);
        Ok(())
    }

    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("unused")
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("unused")
    }

    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("unused")
    }

    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("unused")
    }

    fn run_script(&self, script: &str) -> KResult<()> {
        if let Some(e) = &self.run_error {
            return Err(e.clone());
        }
        self.scripts.lock().push(script.to_string());
        Ok(())
    }

    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("unused")
    }

    fn get_mutex(&self) -> reactor::ReactorMutex {
        self.reactor.mutex(false)
    }

    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("unused")
    }

    fn respond_raw(&self, _msg: &str) {
        panic!("unused")
    }

    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("unused")
    }

    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("unused")
    }

    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("unused")
    }

    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("unused")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<BTreeMap<String, ObjectRef>>>,
}

impl Default for FakePrinter {
    fn default() -> FakePrinter {
        FakePrinter { objects: Arc::new(Mutex::new(BTreeMap::new())) }
    }
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut map = self.objects.lock();
        if map.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        map.insert(name.to_string(), obj);
        Ok(())
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        fresh_reactor()
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("unused")
    }
    fn is_shutdown(&self) -> bool {
        panic!("unused")
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("unused")
    }
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj)
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(
        &self,
        _module: Option<&str>,
    ) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(section) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unable to load module '{section}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn run(&self) -> Option<String> {
        panic!("unused")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("unused")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("unused")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("unused")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {
        panic!("unused")
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("unused")
    }
    fn request_exit(&self, _result: &str) {
        panic!("unused")
    }
}

/// `ConfigApi` mirror of `klippy/configfile.py` `ConfigWrapper`, for
/// the getters `gcode_button` uses.
struct FakeConfig {
    printer: Printer,
    name: String,
    options: Vec<(String, String)>,
}

impl FakeConfig {
    fn raw(&self, option: &str) -> Option<String> {
        self.options
            .iter()
            .rev()
            .find(|(k, _)| k == option)
            .map(|(_, v)| v.clone())
    }

    fn parse_error(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.name
        ))
    }

    fn specified_error(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        ))
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }

    fn get_name(&self) -> String {
        self.name.clone()
    }

    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.raw(option) {
            return Ok(Some(v));
        }
        match opts.default {
            None => Err(self.specified_error(option)),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }

    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("unused")
    }

    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(self.specified_error(option)),
                    Some(None) => Ok(None),
                    Some(Some(v)) => Ok(Some(v)),
                }
            }
        };
        let v = float_from_string(&raw).map_err(|_| self.parse_error(option))?;
        if let Some(b) = opts.minval {
            if v < b {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    float_repr(b)
                )));
            }
        }
        if let Some(b) = opts.maxval {
            if v > b {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    float_repr(b)
                )));
            }
        }
        if let Some(b) = opts.above {
            if v <= b {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    float_repr(b)
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    float_repr(b)
                )));
            }
        }
        Ok(Some(v))
    }

    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("unused")
    }

    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("unused")
    }

    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("unused")
    }

    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("unused")
    }

    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("unused")
    }

    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let raw = match self.raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(self.specified_error(option)),
                    Some(None) => Ok(None),
                    Some(Some(v)) => Ok(Some(v)),
                }
            }
        };
        let mut out = Vec::new();
        if !raw.trim().is_empty() {
            for part in raw.split(opts.sep.as_str()) {
                let part = part.trim();
                out.push(float_from_string(part).map_err(|_| self.parse_error(option))?);
            }
        }
        if let Some(n) = opts.count {
            if out.len() != n {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have {n} elements",
                    self.name
                )));
            }
        }
        Ok(Some(out))
    }

    fn getsection(&self, _section: &str) -> Config {
        panic!("unused")
    }

    fn has_section(&self, _section: &str) -> bool {
        panic!("unused")
    }

    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("unused")
    }

    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("unused")
    }

    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("unused")
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn py_string(s: &str) -> String {
    format!("{s:?}")
}

fn config_text(section: &str, options: &[(&str, &str)]) -> String {
    let mut text = format!("[{section}]\n");
    for (key, value) in options {
        text.push_str(&format!("{key} = {value}\n"));
    }
    text
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "gcode_button".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// `h_init` returns `'ok'` on success and a `(class, message)` tuple
/// when the reference constructor raises, so both outcomes arrive as
/// `Ok` from the oracle and are matched against the port's `KResult`.
fn check_init(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: &KResult<Arc<GCodeButton>>,
) {
    match expected {
        Ok(Value::Str(s)) if s == "ok" => match actual {
            Ok(_) => {}
            Err(e) => panic!(
                "{label}: oracle built the button, port raised {}: {}",
                e.class_name(),
                e.message()
            ),
        },
        Ok(Value::Tuple(t)) if t.len() == 2 => {
            let (class, message) = match (&t[0], &t[1]) {
                (Value::Str(class), Value::Str(message)) => (class, message),
                _ => panic!("{label}: unexpected oracle error tuple: {t:?}"),
            };
            match actual {
                Ok(_) => panic!(
                    "{label}: oracle raised {class}: {message}, port built the button"
                ),
                Err(e) => {
                    assert_eq!(e.class_name(), *class, "{label} exception class");
                    assert_eq!(e.message(), *message, "{label} exception message");
                }
            }
        }
        Ok(v) => panic!("{label}: oracle returned {v:?}"),
        Err(exc) => {
            panic!("{label}: oracle raised {}: {}", exc.class_name, exc.message)
        }
    }
}

fn status_value(py: &PyValue) -> Value {
    match py {
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (Value::str(&k.str()), Value::str(&v.str())))
                .collect(),
        ),
        other => panic!("unexpected status value: {other:?}"),
    }
}

fn templates_value(btn: Option<&GCodeButton>) -> Value {
    let Some(btn) = btn else { return Value::List(Vec::new()) };
    let press = btn.press_template().expect("press template loaded");
    let release = btn.release_template().expect("release template loaded");
    Value::List(vec![
        Value::List(vec![Value::str(press.name()), Value::str(press.source())]),
        Value::List(vec![Value::str(release.name()), Value::str(release.source())]),
    ])
}

fn obs_value(
    buttons: &FakeButtons,
    btn: Option<&GCodeButton>,
    gcode: &FakeGCode,
    logs: &[(String, String)],
) -> Value {
    Value::Dict(vec![
        (Value::str("buttons"), buttons.regs_value()),
        (Value::str("templates"), templates_value(btn)),
        (Value::str("mux"), gcode.mux_value()),
        (Value::str("scripts"), gcode.scripts_value()),
        (
            Value::str("logs"),
            Value::List(
                logs.iter()
                    .map(|(level, msg)| Value::Tuple(vec![Value::str(level), Value::str(msg)]))
                    .collect(),
            ),
        ),
    ])
}

struct Fixture {
    config: Config,
    buttons: Arc<FakeButtons>,
    gcode: Arc<FakeGCode>,
}

fn fixture(
    section: &str,
    options: &[(&str, &str)],
    buttons_fail: Option<&str>,
    run_error: Option<&str>,
) -> Fixture {
    let printer_obj = FakePrinter::default();
    let printer: Printer = Arc::new(printer_obj.clone());
    let buttons = Arc::new(FakeButtons::new(buttons_fail.map(|m| KlippyError::Config(m.to_string()))));
    let gcode = Arc::new(FakeGCode {
        mux: Mutex::new(Vec::new()),
        handler: Mutex::new(None),
        scripts: Mutex::new(Vec::new()),
        run_error: run_error.map(|m| KlippyError::Command(m.to_string())),
        reactor: fresh_reactor(),
    });
    let buttons_ref: ObjectRef = buttons.clone();
    printer_obj.add("buttons", buttons_ref).expect("add buttons");
    let gcode_ref: ObjectRef = gcode.clone();
    printer_obj.add("gcode", gcode_ref).expect("add gcode");
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        name: section.to_string(),
        options: options
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect(),
    });
    let gmacro = Arc::new(PrinterGCodeMacro::new(&config));
    let gmacro_ref: ObjectRef = gmacro;
    printer_obj.add("gcode_macro", gmacro_ref).expect("add gcode_macro");
    Fixture { config, buttons, gcode }
}

fn py_init(
    py: &mut pyoracle::Oracle,
    section: &str,
    options: &[(&str, &str)],
    buttons_fail: Option<&str>,
    run_error: Option<&str>,
) -> pyoracle::OracleResult {
    let fail = match buttons_fail {
        None => "None".to_string(),
        Some(m) => py_string(m),
    };
    let run = match run_error {
        None => "None".to_string(),
        Some(m) => py_string(m),
    };
    py.eval(&format!(
        "h_init({}, {}, {}, {})",
        py_string(section),
        py_string(&config_text(section, options)),
        fail,
        run
    ))
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn gcode_button_matches_python() {
    let Some(mut py) = oracle() else { return };
    log_capture();
    assert!(extras::gcode_button::module_factory()
        .load_config_prefix
        .is_some());
    assert_eq!(CMD_QUERY_BUTTON_HELP, "Report on the state of a button");

    // Digital button: construction, QUERY_BUTTON, press/release events.
    {
        let label = "digital button";
        let section = "gcode_button ab";
        let options = [("pin", "gpio12"), ("press_gcode", "G28"), ("release_gcode", "G29")];
        let fx = fixture(section, &options, None, None);
        log_capture();
        let expected = py_init(&mut py, section, &options, None, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        assert_eq!(btn.name(), "ab");
        assert_eq!(btn.pin(), "gpio12");
        check(
            &format!("{label} obs"),
            py.eval("h_obs()"),
            obs_value(&fx.buttons, Some(&btn), &fx.gcode, &captured_logs()),
        );
        check(
            &format!("{label} query before"),
            py.eval("h_query()"),
            {
                query_gcmd(&fx.gcode, &Arc::new(FakeGCmd { infos: Mutex::new(Vec::new()) }))
            },
        );
        for (et, state) in [(12.5f64, 1i64), (13.5, 0)] {
            check(
                &format!("{label} event {state}"),
                py.eval(&format!("h_event({et:?}, {state})")),
                {
                    fx.buttons.fire(et, ButtonState::Digital(state));
                    status_value(&btn.get_status(0.0))
                },
            );
        }
        check(
            &format!("{label} query after"),
            py.eval("h_query()"),
            {
                query_gcmd(&fx.gcode, &Arc::new(FakeGCmd { infos: Mutex::new(Vec::new()) }))
            },
        );
        check(
            &format!("{label} status"),
            py.eval("h_status()"),
            status_value(&btn.get_status(0.0)),
        );
        check(
            &format!("{label} scripts"),
            py.eval("H['gcode'].scripts"),
            fx.gcode.scripts_value(),
        );
    }

    // ADC button: default pullup, explicit pullup, absent release_gcode.
    {
        let label = "adc button";
        let section = "gcode_button adc";
        let options = [
            ("pin", "gpio13"),
            ("analog_range", "0.1, 0.9"),
            ("press_gcode", "G30"),
        ];
        let fx = fixture(section, &options, None, None);
        log_capture();
        let expected = py_init(&mut py, section, &options, None, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        check(
            &format!("{label} obs"),
            py.eval("h_obs()"),
            obs_value(&fx.buttons, Some(&btn), &fx.gcode, &captured_logs()),
        );
        check(
            &format!("{label} press"),
            py.eval("h_event(20.0, 1)"),
            {
                fx.buttons.fire(20.0, ButtonState::Analog(true));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} release"),
            py.eval("h_event(21.0, 0)"),
            {
                fx.buttons.fire(21.0, ButtonState::Analog(false));
                status_value(&btn.get_status(0.0))
            },
        );
    }

    // Explicit pullup and multi-word name.
    {
        let label = "adc explicit pullup";
        let section = "gcode_button door sensor";
        let options = [
            ("pin", "gpio14"),
            ("analog_range", "0.25, 1.5"),
            ("analog_pullup_resistor", "2200"),
            ("press_gcode", "G31"),
            ("release_gcode", "G32"),
        ];
        let fx = fixture(section, &options, None, None);
        log_capture();
        let expected = py_init(&mut py, section, &options, None, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        assert_eq!(btn.name(), "sensor");
        check(
            &format!("{label} obs"),
            py.eval("h_obs()"),
            obs_value(&fx.buttons, Some(&btn), &fx.gcode, &captured_logs()),
        );
        check(
            &format!("{label} query"),
            py.eval("h_query()"),
            {
                query_gcmd(&fx.gcode, &Arc::new(FakeGCmd { infos: Mutex::new(Vec::new()) }))
            },
        );
    }

    // Whitespace-only scripts are skipped; nonzero states are pressed;
    // a `None` state is released.
    {
        let label = "whitespace scripts";
        let section = "gcode_button ws";
        let options = [
            ("pin", "gpio15"),
            ("press_gcode", "{ '   ' }"),
            ("release_gcode", "{ ' ' }"),
        ];
        let fx = fixture(section, &options, None, None);
        log_capture();
        let expected = py_init(&mut py, section, &options, None, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        check(
            &format!("{label} press"),
            py.eval("h_event(30.0, 7)"),
            {
                fx.buttons.fire(30.0, ButtonState::Digital(7));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} release"),
            py.eval("h_event(31.0, 0)"),
            {
                fx.buttons.fire(31.0, ButtonState::Digital(0));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} none state"),
            py.eval("h_event(32.0, None)"),
            {
                fx.buttons.fire(32.0, ButtonState::Empty);
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} scripts"),
            py.eval("H['gcode'].scripts"),
            fx.gcode.scripts_value(),
        );
    }

    // Render error: logged, state still recorded, nothing run.
    {
        let label = "render error";
        let section = "gcode_button re";
        let options = [
            ("pin", "gpio16"),
            ("press_gcode", "{ action_raise_error('render boom') }"),
            ("release_gcode", "G33"),
        ];
        let fx = fixture(section, &options, None, None);
        log_capture();
        let expected = py_init(&mut py, section, &options, None, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        check(
            &format!("{label} press"),
            py.eval("h_event(40.0, 1)"),
            {
                fx.buttons.fire(40.0, ButtonState::Digital(1));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} logs"),
            py.eval("list(log_lines)"),
            Value::List(
                captured_logs()
                    .iter()
                    .map(|(level, msg)| Value::Tuple(vec![Value::str(level), Value::str(msg)]))
                    .collect(),
            ),
        );
        check(
            &format!("{label} release"),
            py.eval("h_event(41.0, 0)"),
            {
                fx.buttons.fire(41.0, ButtonState::Digital(0));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} scripts"),
            py.eval("H['gcode'].scripts"),
            fx.gcode.scripts_value(),
        );
    }

    // run_script error: logged, state still recorded.
    {
        let label = "run error";
        let section = "gcode_button rn";
        let options = [("pin", "gpio17"), ("press_gcode", "G34"), ("release_gcode", "G35")];
        let fx = fixture(section, &options, None, Some("script failed"));
        log_capture();
        let expected = py_init(&mut py, section, &options, None, Some("script failed"));
        let actual = GCodeButton::new(&fx.config);
        check_init(label, expected, &actual);
        let btn = actual.expect("button builds");
        check(
            &format!("{label} press"),
            py.eval("h_event(50.0, 1)"),
            {
                fx.buttons.fire(50.0, ButtonState::Digital(1));
                status_value(&btn.get_status(0.0))
            },
        );
        check(
            &format!("{label} logs"),
            py.eval("list(log_lines)"),
            Value::List(
                captured_logs()
                    .iter()
                    .map(|(level, msg)| Value::Tuple(vec![Value::str(level), Value::str(msg)]))
                    .collect(),
            ),
        );
    }

    // Construction error paths: init error class/message plus the
    // partial registrations made before the failure.
    let error_cases: Vec<(&str, &str, Vec<(&str, &str)>, Option<&str>)> = vec![
        (
            "missing pin",
            "gcode_button np",
            vec![("press_gcode", "G28")],
            None,
        ),
        (
            "missing press_gcode",
            "gcode_button mp",
            vec![("pin", "gpio18")],
            None,
        ),
        (
            "analog_range wrong count",
            "gcode_button ac",
            vec![("pin", "gpio19"), ("analog_range", "1,2,3"), ("press_gcode", "x")],
            None,
        ),
        (
            "analog_range unparseable",
            "gcode_button au",
            vec![("pin", "gpio19"), ("analog_range", "abc"), ("press_gcode", "x")],
            None,
        ),
        (
            "pullup zero",
            "gcode_button pz",
            vec![
                ("pin", "gpio19"),
                ("analog_range", "0, 1"),
                ("analog_pullup_resistor", "0"),
                ("press_gcode", "x"),
            ],
            None,
        ),
        (
            "pullup negative",
            "gcode_button pn",
            vec![
                ("pin", "gpio19"),
                ("analog_range", "0, 1"),
                ("analog_pullup_resistor", "-5"),
                ("press_gcode", "x"),
            ],
            None,
        ),
        (
            "pullup unparseable",
            "gcode_button pu",
            vec![
                ("pin", "gpio19"),
                ("analog_range", "0, 1"),
                ("analog_pullup_resistor", "abc"),
                ("press_gcode", "x"),
            ],
            None,
        ),
        (
            "button register fails",
            "gcode_button bf",
            vec![("pin", "gpio20"), ("press_gcode", "x")],
            Some("btn bad"),
        ),
    ];
    for (label, section, options, buttons_fail) in &error_cases {
        let fx = fixture(section, options, *buttons_fail, None);
        log_capture();
        let expected = py_init(&mut py, section, options, *buttons_fail, None);
        let actual = GCodeButton::new(&fx.config);
        check_init(&format!("{label} init"), expected, &actual);
        assert!(actual.is_err(), "{label}: construction must fail");
        check(
            &format!("{label} obs"),
            py.eval("h_obs()"),
            obs_value(&fx.buttons, None, &fx.gcode, &captured_logs()),
        );
    }
}

fn query_gcmd(gcode: &FakeGCode, gcmd: &Arc<FakeGCmd>) -> Value {
    let mut handler = gcode.handler.lock().take().expect("mux handler registered");
    let api: GCmd = gcmd.clone();
    handler(&api).expect("QUERY_BUTTON handler");
    *gcode.handler.lock() = Some(handler);
    Value::List(
        gcmd.infos
            .lock()
            .iter()
            .map(|(msg, log)| Value::Tuple(vec![Value::str(msg), Value::Bool(*log)]))
            .collect(),
    )
}
