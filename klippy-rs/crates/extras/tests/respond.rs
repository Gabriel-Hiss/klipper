//! Parity tests for `extras::respond` (port of `klippy/extras/respond.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `HostResponder` is built
//! on both sides with minimal Python/Rust fakes for the printer, config,
//! `gcode` dispatch and `gcmd`, then fed identical `M118` / `RESPOND`
//! calls and the resulting `respond_raw` outputs, returned errors
//! (class and message), registered commands and config errors are
//! compared exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use extras::respond::{HostResponder, CMD_RESPOND_HELP};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler,
    GetFloatOpts as GcodeGetFloatOpts, GetIntOpts as GcodeGetIntOpts, GetOpts,
    OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.respond as respond

class CommandError(Exception):
    pass

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, gcode=None):
        self._gcode = gcode
    def get_reactor(self):
        return None
    def lookup_object(self, name):
        if name == 'gcode' and self._gcode is not None:
            return self._gcode
        raise configparser.Error("Unknown config object '%s'" % (name,))

class FakeGCode:
    def __init__(self):
        self.registered = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd in self.registered:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.registered[cmd] = (func, when_not_ready, desc)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options=None, section='respond'):
        self._printer = printer
        self._options = dict(options) if options else {}
        self._section = section
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._section
    def get(self, option, default=None):
        return self._options.get(option, default)
    def getchoice(self, option, choices, default=None):
        c = self._options.get(option, default)
        if c not in choices:
            raise configparser.Error(
                "Choice '%s' for option '%s' in section '%s' is not a valid choice"
                % (c, option, self._section))
        return choices[c]

class FakeGCmd:
    error = CommandError
    def __init__(self, params=None, raw='', commandline=''):
        self._params = dict(params) if params else {}
        self._raw = raw
        self._commandline = commandline
        self.responses = []
    def get_raw_command_parameters(self):
        return self._raw
    def get(self, name, default=None):
        return self._params.get(name, default)
    def respond_raw(self, msg):
        self.responses.append(msg)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter {
    gcode: Option<ObjectRef>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
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
        if name == "gcode" {
            if let Some(gcode) = &self.gcode {
                return Ok(Some(gcode.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(klippy_api::error::KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
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

struct FakeGCodeState {
    registered: IndexMap<String, (bool, Option<String>)>,
    handlers: IndexMap<String, GCodeCommandHandler>,
}

struct FakeGCode {
    state: GCell<FakeGCodeState>,
}

impl FakeGCode {
    fn new() -> Self {
        FakeGCode {
            state: GCell::new(FakeGCodeState {
                registered: IndexMap::new(),
                handlers: IndexMap::new(),
            }),
        }
    }

    fn registered_summary(&self) -> Value {
        let state = self.state.borrow();
        Value::List(
            state
                .registered
                .iter()
                .map(|(cmd, (when_not_ready, desc))| {
                    Value::Tuple(vec![
                        Value::Str(cmd.clone()),
                        Value::Bool(*when_not_ready),
                        desc.as_ref().map_or(Value::Null, |d| Value::Str(d.clone())),
                    ])
                })
                .collect(),
        )
    }

    fn invoke(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handler = self
            .state
            .borrow_mut()
            .handlers
            .shift_remove(cmd)
            .expect("handler registered");
        let result = handler(gcmd);
        self.state.borrow_mut().handlers.insert(cmd.to_string(), handler);
        result
    }
}
impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut state = self.state.borrow_mut();
        if state.registered.contains_key(cmd) {
            return Err(klippy_api::error::KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        state
            .registered
            .insert(cmd.to_string(), (opts.when_not_ready, opts.desc.clone()));
        if let Some(handler) = func {
            state.handlers.insert(cmd.to_string(), handler);
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
        self.state
            .borrow()
            .registered
            .iter()
            .filter_map(|(cmd, (_, desc))| desc.as_ref().map(|d| (cmd.clone(), d.clone())))
            .collect()
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
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd::new(IndexMap::new(), String::new(), String::new()))
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

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
    section: String,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.options.get(option) {
            return Ok(Some(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
            Some(Some(v)) => Ok(Some(v)),
            Some(None) => Ok(None),
        }
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
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let c = if let Some(v) = self.options.get(option) {
            PyValue::Str(v.clone())
        } else {
            match opts.default {
                None => {
                    return Err(klippy_api::error::KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    )))
                }
                Some(Some(v)) => v,
                Some(None) => PyValue::None,
            }
        };
        let PyValue::Dict(map) = choices else {
            return Err(klippy_api::error::KlippyError::config("choices is not a dict"));
        };
        if let Some(v) = map.get(&c) {
            return Ok(v.clone());
        }
        Err(klippy_api::error::KlippyError::config(format!(
            "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
            c.str(),
            self.section
        )))
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
        panic!("getsection is not used by respond")
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

struct FakeGCmd {
    params: IndexMap<String, String>,
    raw: String,
    commandline: String,
    responses: GCell<Vec<String>>,
}

impl FakeGCmd {
    fn new(params: IndexMap<String, String>, raw: String, commandline: String) -> Self {
        FakeGCmd {
            params,
            raw,
            commandline,
            responses: GCell::new(Vec::new()),
        }
    }
    fn responses_value(&self) -> Value {
        Value::List(
            self.responses
                .borrow()
                .iter()
                .map(|s| Value::Str(s.clone()))
                .collect(),
        )
    }
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        if let Some(v) = self.params.get(name) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::command(format!(
                "Error on '{}': missing {name}",
                self.commandline
            ))),
            Some(v) => Ok(v),
        }
    }
    fn get_int(&self, _name: &str, _opts: GcodeGetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: GcodeGetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        "RESPOND".to_string()
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
    fn respond_raw(&self, msg: &str) {
        self.responses.borrow_mut().push(msg.to_string());
    }
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
            py.exec(SETUP).expect("respond fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping respond oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_dict(options: &IndexMap<String, String>) -> String {
    let parts: Vec<String> = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn rust_side(options: &IndexMap<String, String>) -> (Arc<HostResponder>, Arc<FakeGCode>, Config) {
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode::new());
    let gcode_obj: ObjectRef = gcode.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter {
        gcode: Some(gcode_obj),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: options.clone(),
        section: "respond".to_string(),
    });
    let rust: Arc<HostResponder> = HostResponder::new(&config).expect("rust HostResponder::new");
    (rust, gcode, config)
}

fn fresh_pair(
    py: &mut pyoracle::Oracle,
    options: &IndexMap<String, String>,
) -> (Arc<HostResponder>, Arc<FakeGCode>, Config) {
    py.exec(&format!(
        "g = FakeGCode(); p = FakePrinter(g); c = FakeConfig(p, {}); hr = respond.load_config(c)",
        py_dict(options)
    ))
    .expect("py load_config");
    rust_side(options)
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "respond".to_string(),
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

fn check_responses(
    py: &mut pyoracle::Oracle,
    label: &str,
    params: &IndexMap<String, String>,
    raw: &str,
    commandline: &str,
    rust: &Arc<HostResponder>,
    is_m118: bool,
) {
    let method = if is_m118 { "cmd_M118" } else { "cmd_RESPOND" };
    py.exec(&format!(
        "gcmd = FakeGCmd({}, {:?}, {:?}); hr.{method}(gcmd)",
        py_dict(params),
        raw,
        commandline
    ))
    .expect("py command");
    let expected = py.eval("gcmd.responses");
    let rust_gcmd = Arc::new(FakeGCmd::new(
        params.clone(),
        raw.to_string(),
        commandline.to_string(),
    ));
    let gcmd_trait: GCmd = rust_gcmd.clone();
    let actual_result = if is_m118 {
        rust.cmd_m118(&gcmd_trait)
    } else {
        rust.cmd_respond(&gcmd_trait)
    };
    if let Err(e) = actual_result {
        panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
    }
    check(label, expected, rust_gcmd.responses_value());
}

fn check_command_error(
    py: &mut pyoracle::Oracle,
    label: &str,
    params: &IndexMap<String, String>,
    rust: &Arc<HostResponder>,
) {
    py.exec(&format!(
        "gcmd = FakeGCmd({}, '', 'RESPOND')",
        py_dict(params)
    ))
    .expect("py gcmd");
    let expected = py.eval("hr.cmd_RESPOND(gcmd)");
    let rust_gcmd: GCmd = Arc::new(FakeGCmd::new(
        params.clone(),
        String::new(),
        "RESPOND".to_string(),
    ));
    let actual = rust.cmd_respond(&rust_gcmd);
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        rust_err.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        rust_err.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
    assert_eq!(
        rust_err.class_name(),
        "CommandError",
        "{label}: RESPOND type errors are command errors"
    );
}

fn params_of(pairs: &[(&str, &str)]) -> IndexMap<String, String> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect()
}

fn no_options() -> IndexMap<String, String> {
    IndexMap::new()
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn registers_m118_and_respond() {
    let Some(mut py) = oracle() else { return };
    let (rust, gcode, _config) = fresh_pair(&mut py, &no_options());
    let _ = rust;
    let expected = py.eval(
        "[(k, v[1], v[2]) for k, v in g.registered.items()]",
    );
    check(
        "registers_m118_and_respond",
        expected,
        gcode.registered_summary(),
    );
    let expected = py.eval("hr.cmd_RESPOND_help");
    check(
        "cmd_RESPOND_help",
        Ok(expected.expect("oracle help")),
        Value::Str(CMD_RESPOND_HELP.to_string()),
    );
}

#[test]
fn default_prefix_from_default_type() {
    let Some(mut py) = oracle() else { return };
    for (dtype, expected_prefix) in [("echo", "echo:"), ("command", "//"), ("error", "!!")] {
        let mut opts = IndexMap::new();
        opts.insert("default_type".to_string(), dtype.to_string());
        let (rust, _g, _c) = fresh_pair(&mut py, &opts);
        let expected = py.eval("hr.default_prefix");
        check(
            &format!("default_prefix default_type={dtype}"),
            expected,
            Value::Str(expected_prefix.to_string()),
        );
        assert_eq!(rust.default_prefix(), expected_prefix);
    }
    // No options at all: default_type defaults to 'echo'.
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    let expected = py.eval("hr.default_prefix");
    check(
        "default_prefix no options",
        expected,
        Value::Str(rust.default_prefix().to_string()),
    );
    assert_eq!(rust.default_prefix(), "echo:");
}

#[test]
fn default_prefix_override() {
    let Some(mut py) = oracle() else { return };
    let mut opts = IndexMap::new();
    opts.insert("default_type".to_string(), "command".to_string());
    opts.insert("default_prefix".to_string(), ">>".to_string());
    let (rust, _g, _c) = fresh_pair(&mut py, &opts);
    let expected = py.eval("hr.default_prefix");
    check(
        "default_prefix_override",
        expected,
        Value::Str(rust.default_prefix().to_string()),
    );
    assert_eq!(rust.default_prefix(), ">>");

    let mut opts = IndexMap::new();
    opts.insert("default_prefix".to_string(), String::new());
    let (rust, _g, _c) = fresh_pair(&mut py, &opts);
    let expected = py.eval("hr.default_prefix");
    check(
        "default_prefix empty override",
        expected,
        Value::Str(rust.default_prefix().to_string()),
    );
    assert_eq!(rust.default_prefix(), "");
}

#[test]
fn m118_uses_default_prefix_and_raw_params() {
    let Some(mut py) = oracle() else { return };
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    for (label, raw) in [
        ("m118 hello", "hello world"),
        ("m118 empty", ""),
        ("m118 spaced", "  spaced  "),
        ("m118 percent", "%s %d %%"),
        ("m118 unicode", "héllo ☃"),
    ] {
        check_responses(
            &mut py,
            label,
            &no_options(),
            raw,
            &format!("M118 {raw}"),
            &rust,
            true,
        );
    }
    // With default_type=command, M118 uses '//'.
    let mut opts = IndexMap::new();
    opts.insert("default_type".to_string(), "command".to_string());
    let (rust, _g, _c) = fresh_pair(&mut py, &opts);
    check_responses(
        &mut py,
        "m118 default_type command",
        &no_options(),
        "hi",
        "M118 hi",
        &rust,
        true,
    );
}

#[test]
fn respond_defaults() {
    let Some(mut py) = oracle() else { return };
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    for (label, params) in [
        ("respond msg only", params_of(&[("MSG", "hi")])),
        ("respond no params", params_of(&[])),
        ("respond empty msg", params_of(&[("MSG", "")])),
        ("respond msg with spaces", params_of(&[("MSG", "a b c")])),
    ] {
        check_responses(&mut py, label, &params, "", "RESPOND", &rust, false);
    }
}

#[test]
fn respond_types() {
    let Some(mut py) = oracle() else { return };
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    for (label, params) in [
        ("respond type echo", params_of(&[("TYPE", "echo"), ("MSG", "hi")])),
        ("respond type command", params_of(&[("TYPE", "command"), ("MSG", "hi")])),
        ("respond type error", params_of(&[("TYPE", "error"), ("MSG", "hi")])),
        ("respond type ECHO upper", params_of(&[("TYPE", "ECHO"), ("MSG", "hi")])),
        ("respond type EcHo mixed", params_of(&[("TYPE", "EcHo"), ("MSG", "hi")])),
        (
            "respond type ERROR upper",
            params_of(&[("TYPE", "ERROR"), ("MSG", "hi")]),
        ),
        (
            "respond type echo_no_space",
            params_of(&[("TYPE", "echo_no_space"), ("MSG", "hi")]),
        ),
        (
            "respond type ECHO_NO_SPACE upper",
            params_of(&[("TYPE", "ECHO_NO_SPACE"), ("MSG", "hi")]),
        ),
        (
            "respond type echo_no_space empty msg",
            params_of(&[("TYPE", "echo_no_space"), ("MSG", "")]),
        ),
    ] {
        check_responses(&mut py, label, &params, "", "RESPOND", &rust, false);
    }
}

#[test]
fn respond_prefix_override() {
    let Some(mut py) = oracle() else { return };
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    for (label, params) in [
        (
            "respond prefix override",
            params_of(&[("PREFIX", ">>"), ("MSG", "hi")]),
        ),
        (
            "respond prefix empty override",
            params_of(&[("PREFIX", ""), ("MSG", "hi")]),
        ),
        (
            "respond type then prefix override",
            params_of(&[("TYPE", "command"), ("PREFIX", ">>"), ("MSG", "hi")]),
        ),
        (
            "respond echo_no_space then prefix override keeps no space",
            params_of(&[("TYPE", "echo_no_space"), ("PREFIX", ">>"), ("MSG", "hi")]),
        ),
        (
            "respond prefix override no msg",
            params_of(&[("PREFIX", ">>")]),
        ),
    ] {
        check_responses(&mut py, label, &params, "", "RESPOND", &rust, false);
    }
}

#[test]
fn respond_invalid_type() {
    let Some(mut py) = oracle() else { return };
    let (rust, _g, _c) = fresh_pair(&mut py, &no_options());
    for (label, params) in [
        ("respond invalid foo", params_of(&[("TYPE", "foo"), ("MSG", "hi")])),
        ("respond invalid FOO upper lowered in msg", params_of(&[("TYPE", "FOO")])),
        ("respond invalid empty", params_of(&[("TYPE", ""), ("MSG", "hi")])),
        ("respond invalid echoo", params_of(&[("TYPE", "echoo")])),
    ] {
        check_command_error(&mut py, label, &params, &rust);
    }
}

#[test]
fn stored_handlers_match_direct_calls() {
    let Some(mut py) = oracle() else { return };
    let (rust, gcode, _c) = fresh_pair(&mut py, &no_options());
    let _ = rust;
    // RESPOND through the handler stored by register_command.
    py.exec("gcmd = FakeGCmd({'MSG': 'via handler'}, '', 'RESPOND'); g.registered['RESPOND'][0](gcmd)")
        .expect("py stored RESPOND");
    let expected = py.eval("gcmd.responses");
    let rust_gcmd = Arc::new(FakeGCmd::new(
        params_of(&[("MSG", "via handler")]),
        String::new(),
        "RESPOND".to_string(),
    ));
    let gcmd_trait: GCmd = rust_gcmd.clone();
    gcode
        .invoke("RESPOND", &gcmd_trait)
        .expect("rust stored RESPOND");
    check("stored RESPOND handler", expected, rust_gcmd.responses_value());

    // M118 through the stored handler.
    py.exec("gcmd = FakeGCmd({}, 'raw text', 'M118 raw text'); g.registered['M118'][0](gcmd)")
        .expect("py stored M118");
    let expected = py.eval("gcmd.responses");
    let rust_gcmd = Arc::new(FakeGCmd::new(
        IndexMap::new(),
        "raw text".to_string(),
        "M118 raw text".to_string(),
    ));
    let gcmd_trait: GCmd = rust_gcmd.clone();
    gcode.invoke("M118", &gcmd_trait).expect("rust stored M118");
    check("stored M118 handler", expected, rust_gcmd.responses_value());
}

#[test]
fn invalid_default_type_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let mut opts = IndexMap::new();
    opts.insert("default_type".to_string(), "bogus".to_string());
    let expected = py.exec(&format!(
        "respond.load_config(FakeConfig(FakePrinter(FakeGCode()), {}))",
        py_dict(&opts)
    ));
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode::new());
    let gcode_obj: ObjectRef = gcode.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter {
        gcode: Some(gcode_obj),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: opts,
        section: "respond".to_string(),
    });
    let actual = HostResponder::new(&config);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("rust returned Ok but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str());
    assert_eq!(rust_err.message(), oracle_exc.message.as_str());
}

#[test]
fn missing_gcode_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.exec(
        "respond.load_config(FakeConfig(FakePrinter(None), {}))",
    );
    let printer: Printer = Arc::new(FakePrinter { gcode: None });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: IndexMap::new(),
        section: "respond".to_string(),
    });
    let actual = HostResponder::new(&config);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("rust returned Ok but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str());
    assert_eq!(rust_err.message(), oracle_exc.message.as_str());
}

#[test]
fn duplicate_registration_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let expected = py.exec(
        "g2 = FakeGCode(); g2.register_command('M118', lambda gcmd: None); \
         respond.load_config(FakeConfig(FakePrinter(g2), {}))",
    );
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode::new());
    gcode
        .register_command(
            "M118",
            Some(Box::new(|_gcmd: &GCmd| Ok(()))),
            RegisterCommandOpts {
                when_not_ready: true,
                desc: None,
            },
        )
        .expect("pre-register M118");
    let gcode_obj: ObjectRef = gcode.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter {
        gcode: Some(gcode_obj),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: IndexMap::new(),
        section: "respond".to_string(),
    });
    let actual = HostResponder::new(&config);
    let oracle_exc = match expected {
        Ok(v) => panic!("oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("rust returned Ok but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str());
    assert_eq!(rust_err.message(), oracle_exc.message.as_str());
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::respond::module_factory();
    let load_config = factory
        .load_config
        .expect("respond exposes load_config");
    assert!(factory.load_config_prefix.is_none(), "respond has no load_config_prefix");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "respond has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "respond has no add_printer_objects"
    );
    let gcode: Arc<FakeGCode> = Arc::new(FakeGCode::new());
    let gcode_obj: ObjectRef = gcode.clone() as ObjectRef;
    let printer: Printer = Arc::new(FakePrinter {
        gcode: Some(gcode_obj),
    });
    let config: Config = Arc::new(FakeConfig {
        printer,
        options: IndexMap::new(),
        section: "respond".to_string(),
    });
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let rust: Arc<HostResponder> = downcast_arc(&obj).expect("downcast to HostResponder");
    py.exec("g = FakeGCode(); p = FakePrinter(g); c = FakeConfig(p, {}); hr = respond.load_config(c)")
        .expect("py factory load_config");
    let expected = py.eval("hr.default_prefix");
    check(
        "factory default_prefix",
        expected,
        Value::Str(rust.default_prefix().to_string()),
    );
    check_responses(
        &mut py,
        "factory RESPOND",
        &params_of(&[("MSG", "hi")]),
        "",
        "RESPOND",
        &rust,
        false,
    );
}
