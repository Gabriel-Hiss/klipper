//! Parity tests for `extras::display_status` (`klippy/extras/display_status.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `DisplayStatus` is built
//! on both sides with minimal Python fakes, then fed identical `M73` /
//! `M117` / `SET_DISPLAY_TEXT` calls and `get_status` queries. Progress
//! values, message storage, expiry, SD-card fallback and error paths are
//! compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so expected values come from the oracle alone.

use extras::display_status::{DisplayStatus, module_factory, M73_TIMEOUT, SET_DISPLAY_TEXT_HELP};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GetFloatOpts as GCCmdGetFloatOpts, GetIntOpts as GCCmdGetIntOpts,
    GetOpts as GCCmdGetOpts, GCodeCommandHandler, OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.display_status as display_status

_sentinel = object()

class CommandError(Exception):
    pass

class FakeReactor:
    def __init__(self):
        self.t = 1000.
    def monotonic(self):
        return self.t

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.commands[cmd] = (func, desc)

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {'gcode': FakeGCode()}
        self._reactor = FakeReactor()
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def get_reactor(self):
        return self._reactor

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

class FakeGCmd:
    error = CommandError
    def __init__(self, params=None, raw=""):
        self._params = dict(params or {})
        self._raw = raw
        self._commandline = "M73 " + raw
    def get_float(self, name, default=None):
        value = self._params.get(name)
        if value is None:
            return default
        try:
            return float(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
    def get_raw_command_parameters(self):
        return self._raw
    def get(self, name, default=None):
        return self._params.get(name, default)

class FakeTimeout:
    def __init__(self, state):
        self._state = state
    def get_status(self, eventtime):
        return {'state': self._state}

class FakeSd:
    def __init__(self, prog):
        self._p = prog
    def get_status(self, eventtime):
        return {'progress': self._p}
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinterInner {
    objects: IndexMap<String, ObjectRef>,
    reactor: Reactor,
}

#[derive(Clone)]
struct FakePrinter {
    inner: FakePrinterInner,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.inner.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        match self.inner.objects.get(name) {
            Some(o) => Ok(Some(o.clone())),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "display_status".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by display_status")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

/// Minimal status object (stand-in for `idle_timeout` / `virtual_sdcard`).
struct FakeStatusObj {
    status: PyValue,
}

impl PrinterObject for FakeStatusObj {
    klippy_api::printer_object_base!();
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status.clone()))
    }
}

struct FakeGCode {
    commands: std::sync::Mutex<IndexMap<String, Option<String>>>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Self {
        Self {
            commands: std::sync::Mutex::new(IndexMap::new()),
            reactor,
        }
    }
    fn registered(&self) -> IndexMap<String, Option<String>> {
        self.commands.lock().unwrap().clone()
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.commands
            .lock()
            .unwrap()
            .insert(cmd.to_string(), opts.desc);
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
        PyValue::None
    }
    fn register_output_handler(&self, _callback: OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
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
        panic!("create_gcode_command is not used by display_status")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCmd {
    params: IndexMap<String, String>,
    raw: String,
    commandline: String,
}

impl FakeGCmd {
    fn m73(p: Option<&str>) -> Self {
        let mut params = IndexMap::new();
        if let Some(v) = p {
            params.insert("P".to_string(), v.to_string());
        }
        let raw = p.map_or_else(String::new, |v| format!("P{v}"));
        Self {
            params,
            raw: raw.clone(),
            commandline: format!("M73 {raw}"),
        }
    }
    fn m117(raw: &str) -> Self {
        Self {
            params: IndexMap::new(),
            raw: raw.to_string(),
            commandline: format!("M117 {raw}"),
        }
    }
    fn set_text(msg: Option<&str>) -> Self {
        let mut params = IndexMap::new();
        if let Some(v) = msg {
            params.insert("MSG".to_string(), v.to_string());
        }
        let raw = msg.map_or_else(String::new, |v| format!("MSG={v}"));
        Self {
            params,
            raw: raw.clone(),
            commandline: format!("SET_DISPLAY_TEXT {raw}"),
        }
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GCCmdGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_int(&self, _name: &str, _opts: GCCmdGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, name: &str, opts: GCCmdGetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            Some(v) => match v.parse::<f64>() {
                Ok(f) => Ok(Some(f)),
                Err(_) => Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {v}",
                    self.commandline
                ))),
            },
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::command(format!(
                    "Error on '{}': missing {name}",
                    self.commandline
                ))),
            },
        }
    }
    fn get_command(&self) -> String {
        self.commandline.split(' ').next().unwrap_or("").to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.raw.clone()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("display_status fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping display_status oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            if let Some(i) = n.to_f64_exact() {
                let _ = i;
            }
            // Test values are small; use string round-trip for exactness.
            let s = n.0.to_string();
            if let Ok(i) = s.parse::<i64>() {
                Value::Int(i)
            } else {
                Value::BigInt(s)
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "object".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "display_status".to_string(),
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

fn check_status(label: &str, expected: Value, actual: KResult<PyValue>) {
    let actual = match actual {
        Ok(v) => v,
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    };
    check(label, Ok(expected), pyvalue_to_value(&actual));
}

fn check_cmd(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Ok(v), Ok(())) => {
            // cmd_* returns None in Python.
            check(label, Ok(v), Value::Null);
        }
        (Err(e), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), e.class_name.as_str(), "{label}: class");
            assert_eq!(rust_err.message(), e.message.as_str(), "{label}: message");
        }
        (Ok(v), Err(rust_err)) => {
            panic!("{label}: oracle returned {v:?} but rust raised {}: {}", rust_err.class_name(), rust_err.message());
        }
        (Err(e), Ok(())) => {
            panic!("{label}: oracle raised {}: {} but rust succeeded", e.class_name, e.message);
        }
    }
}

fn fresh_pair(py: &mut pyoracle::Oracle) -> (DisplayStatus, Config, Arc<FakeGCode>) {
    py.exec("p = FakePrinter(); c = FakeConfig(p); ds = display_status.load_config(c)")
        .expect("py load_config");
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    objs.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs, reactor },
    });
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
    });
    let rust = DisplayStatus::new(&config).expect("rust new");
    (rust, config, gcode)
}

fn set_py_objects(
    py: &mut pyoracle::Oracle,
    rust_printer: &FakePrinter,
    idle_state: Option<&str>,
    sd_progress: Option<f64>,
) {
    let idle_code = match idle_state {
        Some(s) => format!(
            "p.objects['idle_timeout'] = FakeTimeout({:?});",
            s
        ),
        None => String::new(),
    };
    let sd_code = match sd_progress {
        Some(v) => format!("p.objects['virtual_sdcard'] = FakeSd({v});"),
        None => String::new(),
    };
    py.exec(&format!("{idle_code}{sd_code}"))
        .expect("py set objects");
    let _ = rust_printer;
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("display_status.M73_TIMEOUT").expect("eval M73_TIMEOUT");
    check("M73_TIMEOUT", Ok(expected), Value::float(M73_TIMEOUT));
    let expected = py
        .eval("display_status.DisplayStatus.cmd_SET_DISPLAY_TEXT_help")
        .expect("eval help");
    check(
        "SET_DISPLAY_TEXT help",
        Ok(expected),
        Value::Str(SET_DISPLAY_TEXT_HELP.to_string()),
    );
}

#[test]
fn initial_status_has_zero_progress_and_no_message() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    let expected = py.eval("ds.get_status(0.)").expect("py get_status");
    check_status("initial get_status", expected, rust.get_status_value(0.));
}

#[test]
fn m73_sets_progress_clamped() {
    let Some(mut py) = oracle() else { return };
    for (label, p) in [
        ("m73 0", "0"),
        ("m73 25", "25"),
        ("m73 50", "50"),
        ("m73 100", "100"),
        ("m73 -10 clamps to 0", "-10"),
        ("m73 150 clamps to 1", "150"),
        ("m73 33.333", "33.333"),
        ("m73 -0 stores +0", "-0"),
        ("m73 nan stores 0", "nan"),
    ] {
        let (rust, _, _) = fresh_pair(&mut py);
        let gcmd = FakeGCmd::m73(Some(p));
        let gcmd_arc: GCmd = Arc::new(gcmd);
        let rust_r = rust.cmd_m73(&gcmd_arc);
        let expected = py.eval(&format!("ds.cmd_M73(FakeGCmd({{'P': '{p}'}}, 'P{p}'))"));
        check_cmd(&format!("{label} cmd"), expected, rust_r);
        let expected = py.eval("ds.get_status(0.)").expect("py get_status after M73");
        check_status(&format!("{label} status"), expected, rust.get_status_value(0.));
    }
}

#[test]
fn m73_without_p_leaves_status_unchanged() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    let gcmd = FakeGCmd::m73(None);
    let gcmd_arc: GCmd = Arc::new(gcmd);
    let rust_r = rust.cmd_m73(&gcmd_arc);
    let expected = py.eval("ds.cmd_M73(FakeGCmd({}))");
    check_cmd("m73 no P cmd", expected, rust_r);
    let expected = py.eval("ds.get_status(0.)").expect("py get_status after no-P M73");
    check_status("m73 no P status", expected, rust.get_status_value(0.));
}

#[test]
fn m73_invalid_float_raises_command_error() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    let gcmd = FakeGCmd::m73(Some("abc"));
    let gcmd_arc: GCmd = Arc::new(gcmd);
    let rust_r = rust.cmd_m73(&gcmd_arc);
    let expected = py.eval("ds.cmd_M73(FakeGCmd({'P': 'abc'}, 'Pabc'))");
    check_cmd("m73 invalid P cmd", expected, rust_r);
}

#[test]
fn m117_sets_and_clears_message() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    for (label, raw) in [
        ("m117 hello", "Hello World"),
        ("m117 empty clears", ""),
        ("m117 heating", "Heating..."),
    ] {
        let gcmd = FakeGCmd::m117(raw);
        let gcmd_arc: GCmd = Arc::new(gcmd);
        let rust_r = rust.cmd_m117(&gcmd_arc);
        // Python FakeGCmd raw handling mirrors gcmd.get_raw_command_parameters().
        let raw_esc = raw.replace('\'', "\\'");
        let expected = py.eval(&format!("ds.cmd_M117(FakeGCmd({{}}, '{raw_esc}'))"));
        check_cmd(&format!("{label} cmd"), expected, rust_r);
        let expected = py.eval("ds.get_status(0.)").expect("py get_status after M117");
        check_status(&format!("{label} status"), expected, rust.get_status_value(0.));
    }
}

#[test]
fn set_display_text_msg_and_missing() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    for (label, msg) in [
        ("set text hello", Some("Hello")),
        ("set text empty", Some("")),
        ("set text missing clears", None),
    ] {
        let gcmd = FakeGCmd::set_text(msg);
        let gcmd_arc: GCmd = Arc::new(gcmd);
        let rust_r = rust.cmd_set_display_text(&gcmd_arc);
        let py_expr = match msg {
            Some(m) => format!("ds.cmd_SET_DISPLAY_TEXT(FakeGCmd({{'MSG': {m:?}}}))"),
            None => "ds.cmd_SET_DISPLAY_TEXT(FakeGCmd({}))".to_string(),
        };
        let expected = py.eval(&py_expr);
        check_cmd(&format!("{label} cmd"), expected, rust_r);
        let expected = py.eval("ds.get_status(0.)").expect("py get_status after SET_DISPLAY_TEXT");
        check_status(&format!("{label} status"), expected, rust.get_status_value(0.));
    }
}

#[test]
fn get_status_sdcard_fallback() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    // No M73: without sdcard progress is 0.
    let expected = py.eval("ds.get_status(0.)").expect("py status no sd");
    check_status("sd fallback default", expected, rust.get_status_value(0.));
    // With sdcard progress 0.7 both sides fall back to it.
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    let g = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("progress".to_string()), PyValue::Float(0.7));
            d
        }),
    });
    objs.insert("virtual_sdcard".to_string(), g as ObjectRef);
    // Replace printer objects by mutating the fresh pair's printer is not
    // possible; instead this test only exercises the no-sdcard path plus the
    // oracle sdcard path is covered by the expiry test below with a fresh
    // printer pair. Keep the oracle sdcard path check here.
    set_py_objects(&mut py, &FakePrinter { inner: FakePrinterInner { objects: objs, reactor: Reactor::new(false) } }, None, Some(0.7));
    let expected = py.eval("ds.get_status(0.)").expect("py status with sd");
    // Rust side needs a printer carrying the sdcard; rebuild a pair where the
    // printer already knows virtual_sdcard.
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let sd_obj: ObjectRef = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("progress".to_string()), PyValue::Float(0.7));
            d
        }),
    });
    let mut objs2: IndexMap<String, ObjectRef> = IndexMap::new();
    objs2.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    objs2.insert("virtual_sdcard".to_string(), sd_obj);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs2, reactor },
    });
    let config: Config = Arc::new(FakeConfig { printer });
    let rust2 = DisplayStatus::new(&config).expect("rust2 new");
    check_status("sd fallback 0.7", expected, rust2.get_status_value(0.));
}

#[test]
fn expiry_clears_progress_when_not_printing() {
    let Some(mut py) = oracle() else { return };
    let (_, _, _) = fresh_pair(&mut py);
    // Python: M73 at t=1000 expires at 1005; get_status(1e12) clears if not Printing.
    py.exec("p.objects['idle_timeout'] = FakeTimeout('Ready'); p.objects['virtual_sdcard'] = FakeSd(0.3)")
        .expect("py objects");
    let expected = py.eval("ds.cmd_M73(FakeGCmd({'P': '80'}))");
    check_cmd("expiry setup M73", expected, Ok::<(), KlippyError>(()));
    let expected = py.eval("ds.get_status(1000000000000.)").expect("py expired status");
    // Rust: fresh printer pair with idle Ready and sd 0.3.
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let idle: ObjectRef = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("state".to_string()), PyValue::Str("Ready".to_string()));
            d
        }),
    });
    let sd: ObjectRef = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("progress".to_string()), PyValue::Float(0.3));
            d
        }),
    });
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    objs.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    objs.insert("idle_timeout".to_string(), idle);
    objs.insert("virtual_sdcard".to_string(), sd);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs, reactor },
    });
    let config: Config = Arc::new(FakeConfig { printer });
    let rust = DisplayStatus::new(&config).expect("rust new");
    let g: GCmd = Arc::new(FakeGCmd::m73(Some("80")));
    rust.cmd_m73(&g).expect("rust M73");
    check_status("expiry clears when Ready", expected, rust.get_status_value(1e12));
}

#[test]
fn expiry_keeps_progress_when_printing() {
    let Some(mut py) = oracle() else { return };
    let (_, _, _) = fresh_pair(&mut py);
    py.exec("p.objects['idle_timeout'] = FakeTimeout('Printing')")
        .expect("py idle printing");
    py.eval("ds.cmd_M73(FakeGCmd({'P': '80'}))").expect("py M73");
    let expected = py.eval("ds.get_status(1000000000000.)").expect("py printing expired status");
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let idle: ObjectRef = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("state".to_string()), PyValue::Str("Printing".to_string()));
            d
        }),
    });
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    objs.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    objs.insert("idle_timeout".to_string(), idle);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs, reactor },
    });
    let config: Config = Arc::new(FakeConfig { printer });
    let rust = DisplayStatus::new(&config).expect("rust new");
    let g: GCmd = Arc::new(FakeGCmd::m73(Some("80")));
    rust.cmd_m73(&g).expect("rust M73");
    check_status("expiry keeps when Printing", expected, rust.get_status_value(1e12));
}

#[test]
fn get_status_before_expiry_ignores_idle_state() {
    let Some(mut py) = oracle() else { return };
    let (_, _, _) = fresh_pair(&mut py);
    py.exec("p.objects['idle_timeout'] = FakeTimeout('Ready')")
        .expect("py idle ready before expiry");
    py.eval("ds.cmd_M73(FakeGCmd({'P': '80'}))").expect("py M73 before expiry");
    let expected = py.eval("ds.get_status(0.)").expect("py before-expiry status");
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let idle: ObjectRef = Arc::new(FakeStatusObj {
        status: PyValue::Dict({
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("state".to_string()), PyValue::Str("Ready".to_string()));
            d
        }),
    });
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    objs.insert("gcode".to_string(), gcode.clone() as ObjectRef);
    objs.insert("idle_timeout".to_string(), idle);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs, reactor },
    });
    let config: Config = Arc::new(FakeConfig { printer });
    let rust = DisplayStatus::new(&config).expect("rust new");
    let g: GCmd = Arc::new(FakeGCmd::m73(Some("80")));
    rust.cmd_m73(&g).expect("rust M73");
    check_status("before expiry ignores Ready", expected, rust.get_status_value(0.));
}

#[test]
fn factory_registers_three_commands_with_help() {
    let Some(mut py) = oracle() else { return };
    let (rust, config, gcode) = fresh_pair(&mut py);
    let _ = (rust, config);
    let registered = gcode.registered();
    assert_eq!(registered.len(), 3, "three commands registered");
    assert!(registered.contains_key("M73"));
    assert!(registered.contains_key("M117"));
    assert!(registered.contains_key("SET_DISPLAY_TEXT"));
    assert_eq!(
        registered.get("SET_DISPLAY_TEXT"),
        Some(&Some("Set or clear the display message".to_string()))
    );
    let expected = py.eval("sorted(p.objects['gcode'].commands.keys())")
        .expect("py registered");
    check(
        "factory command names",
        Ok(expected),
        Value::List(vec![
            Value::Str("M117".to_string()),
            Value::Str("M73".to_string()),
            Value::Str("SET_DISPLAY_TEXT".to_string()),
        ]),
    );
    let expected = py
        .eval("p.objects['gcode'].commands['SET_DISPLAY_TEXT'][1]")
        .expect("py desc");
    check(
        "factory SET_DISPLAY_TEXT desc",
        Ok(expected),
        Value::Str("Set or clear the display message".to_string()),
    );
}

#[test]
fn module_factory_load_config_builds_object() {
    let Some(mut py) = oracle() else { return };
    let (rust, config, _) = fresh_pair(&mut py);
    // module_factory returns a load_config that builds a working object; the
    // already-built `rust` proves the same constructor path. Verify the factory
    // shape and that a second build also reports the initial status.
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    let built = (factory.load_config.unwrap())(&config).expect("factory build");
    let built = built.expect("factory obj");
    let status = built.get_status(0.).unwrap().expect("factory status");
    let value = pyvalue_to_value(&status);
    let expected = py.eval("ds.get_status(0.)").expect("py initial");
    // Both start fresh, so statuses match.
    check("factory status", Ok(expected), value);
    let _ = rust;
}

#[test]
fn trait_get_status_matches_inherent() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    // PrinterObject::get_status returns Some(dict) with same progress/message.
    let via_trait = PrinterObject::get_status(&rust, 0.).unwrap().expect("trait status");
    let expected = py.eval("ds.get_status(0.)").expect("py status");
    check("trait get_status", Ok(expected), pyvalue_to_value(&via_trait));
    // downcast_arc to the concrete type works for the factory object.
    let reactor = Reactor::new(false);
    let gcode = Arc::new(FakeGCode::new(reactor.clone()));
    let mut objs: IndexMap<String, ObjectRef> = IndexMap::new();
    objs.insert("gcode".to_string(), gcode as ObjectRef);
    let printer: Printer = Arc::new(FakePrinter {
        inner: FakePrinterInner { objects: objs, reactor },
    });
    let config: Config = Arc::new(FakeConfig { printer });
    let obj = (module_factory().load_config.unwrap())(&config)
        .expect("factory")
        .expect("obj");
    let ds = downcast_arc::<DisplayStatus>(&obj).expect("downcast");
    let expected = py.eval("ds.get_status(0.)").expect("py status2");
    check_status("downcast status", expected, ds.get_status_value(0.));
}

/// An expired M73 progress with no `idle_timeout` object: Python's
/// `get_status` raises the `lookup_object` config error, and the trait
/// `get_status` returns the same class and text.
#[test]
fn trait_get_status_raises_without_idle_timeout() {
    let Some(mut py) = oracle() else { return };
    let (rust, _, _) = fresh_pair(&mut py);
    py.eval("ds.cmd_M73(FakeGCmd({'P': '80'}))").expect("py M73");
    let expected = py
        .eval("ds.get_status(1000000000000.)")
        .expect_err("py get_status raises without idle_timeout");
    let g: GCmd = Arc::new(FakeGCmd::m73(Some("80")));
    rust.cmd_m73(&g).expect("rust M73");
    let actual = PrinterObject::get_status(&rust, 1e12).expect_err("rust get_status raises");
    assert_eq!(actual.class_name(), expected.class_name, "class");
    assert_eq!(actual.message(), expected.message, "message");
    py.exec(
        "import traceback\n\
         try:\n    ds.get_status(1000000000000.)\n\
         except Exception as e:\n    _tb_line = traceback.format_exception_only(type(e), e)[-1].rstrip()",
    )
    .expect("py traceback line");
    let line = py.eval("_tb_line").expect("py traceback line value");
    check("traceback line", Ok(line), Value::Str(actual.exception_line()));
}
