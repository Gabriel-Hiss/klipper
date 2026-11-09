//! Parity tests for `extras::save_variables` (port of
//! `klippy/extras/save_variables.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `SaveVariables` is built
//! on both sides with minimal Python fakes for the printer, the config
//! wrapper and the gcode dispatch, then fed identical `SAVE_VARIABLE`
//! invocations. Returned errors (class and message), `get_status` values,
//! the variable file contents and the command registration are compared
//! exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::save_variables::SaveVariables;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetIntOpts as GGetIntOpts, GetOpts, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, ObjectRef, PrinterObject};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import gcode
import extras.save_variables as save_variables

class _Sentinel:
    pass
_SENTINEL = _Sentinel()

class FakePrinter:
    command_error = gcode.CommandError
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
    def lookup_object(self, name, default=_SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is _SENTINEL:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        self.objects[name] = obj

class FakeDispatch:
    def __init__(self):
        self.registered = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered[cmd] = (func, when_not_ready, desc)
    def respond_info(self, msg, log=True):
        pass
    def respond_raw(self, msg):
        pass

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, filename=_SENTINEL):
        self._printer = printer
        self._filename = filename
    def get_printer(self):
        return self._printer
    def get(self, option, default=_SENTINEL):
        if option == 'filename' and self._filename is not _SENTINEL:
            return self._filename
        if default is _SENTINEL:
            raise self.error("Option '%s' in section '%s' must be specified" % (option, 'save_variables'))
        return default
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: kos::sync::Mutex<indexmap::IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter { objects: kos::sync::Mutex::new(indexmap::IndexMap::new()) })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unknown config object '{name}'")))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
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
    filename: Option<String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "save_variables".to_string()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if option == "filename" {
            if let Some(name) = &self.filename {
                return Ok(Some(name.clone()));
            }
        }
        match opts.default {
            Some(d) => Ok(d),
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section 'save_variables' must be specified"
            ))),
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
        panic!("getsection is not used by save_variables")
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

/// Fake `GCodeDispatch` (`dyn GCodeApi`): records registrations and lets the
/// test drive the registered handler with a fake command.
struct FakeGCode {
    reactor: Reactor,
    registered: kos::sync::Mutex<Vec<(String, bool, Option<String>)>>,
    handler: kos::sync::Mutex<Option<GCodeCommandHandler>>,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            reactor: Reactor::new(false),
            registered: kos::sync::Mutex::new(Vec::new()),
            handler: kos::sync::Mutex::new(None),
        })
    }
    fn registrations(&self) -> Vec<(String, bool, Option<String>)> {
        self.registered.lock().clone()
    }
    fn invoke(&self, gcmd: &GCmd) -> KResult<()> {
        let mut handler = self.handler.lock();
        match handler.as_mut() {
            Some(h) => h(gcmd),
            None => panic!("SAVE_VARIABLE handler is registered"),
        }
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.registered.lock().push((cmd.to_string(), opts.when_not_ready, opts.desc.clone()));
        *self.handler.lock() = func;
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
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
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
        self.reactor.mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by save_variables")
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

/// Fake `GCodeCommand` (`dyn GCodeCommandApi`) replicating the parameter
/// errors of `klippy/gcode.py:65`.
struct FakeGCodeCommand {
    commandline: String,
    params: indexmap::IndexMap<String, String>,
}

klippy_api::printer_object!(FakeGCodeCommand);

impl FakeGCodeCommand {
    fn missing(&self, name: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{}': missing {name}", self.commandline))
    }
    fn unparsable(&self, value: &str) -> KlippyError {
        KlippyError::command(format!(
            "Error on '{}': unable to parse {value}",
            self.commandline
        ))
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                Some(v) => Ok(v),
                None => Err(self.missing(name)),
            };
        };
        match opts.parser {
            klippy_api::gcode::GetParser::Str => Ok(PyValue::Str(value.clone())),
            klippy_api::gcode::GetParser::Int => pycompat::text::int_from_string(value, 10)
                .map(PyValue::Int)
                .map_err(|_| self.unparsable(value)),
            klippy_api::gcode::GetParser::IntAutoBase => {
                pycompat::text::int_from_string(value, 0)
                    .map(PyValue::Int)
                    .map_err(|_| self.unparsable(value))
            }
            klippy_api::gcode::GetParser::Float => pycompat::text::float_from_string(value)
                .map(PyValue::Float)
                .map_err(|_| self.unparsable(value)),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let value = match self.params.get(name) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    Some(d) => Ok(d),
                    None => Err(self.missing(name)),
                };
            }
        };
        pycompat::text::int_from_string(&value, 10)
            .map(|n| n.inner().to_string().parse::<i64>().unwrap_or(i64::MIN))
            .map(Some)
            .map_err(|_| self.unparsable(&value))
    }
    fn get_float(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetFloatOpts,
    ) -> KResult<Option<f64>> {
        let value = match self.params.get(name) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    Some(d) => Ok(d),
                    None => Err(self.missing(name)),
                };
            }
        };
        pycompat::text::float_from_string(&value)
            .map(Some)
            .map_err(|_| self.unparsable(&value))
    }
    fn get_command(&self) -> String {
        "SAVE_VARIABLE".to_string()
    }
    fn get_commandline(&self) -> String {
        self.commandline.clone()
    }
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
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
            py.exec(SETUP).expect("save_variables fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping save_variables oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_repr_str(s: &str) -> String {
    format!("'{}'", s.replace('\\', "\\\\").replace('\'', "\\'"))
}

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("save_variables_{tag}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("create temp dir");
    dir
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let digits = n.inner().to_string();
            match digits.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(digits),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (to_value(k), to_value(v))).collect(),
        ),
        PyValue::Host(_) => panic!("host value cannot be compared"),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "save_variables".to_string(),
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

fn check_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Ok(v), Ok(())) => {
            // Python returns None from cmd_SAVE_VARIABLE.
            check(label, Ok(v), Value::Null);
        }
        (Ok(v), Err(e)) => {
            panic!(
                "{label}: oracle returned {v:?} but rust call raised {}: {}",
                e.class_name(),
                e.message()
            )
        }
        (Err(e), Ok(())) => {
            panic!("{label}: rust call succeeded but oracle raised {}: {}", e.class_name, e.message)
        }
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label}: exception class");
            assert_eq!(err.message(), exc.message.as_str(), "{label}: exception message");
        }
    }
}
/// Construct the port through `module_factory().load_config` exactly like
/// the Python side drives `save_variables.load_config`, so the
/// `SAVE_VARIABLE` registration happens on both sides.
fn make_rust(filename: &str) -> (Arc<SaveVariables>, Arc<FakeGCode>) {
    let dispatch: Arc<FakeGCode> = FakeGCode::new();
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", dispatch.clone() as ObjectRef);
    let printer_api: Printer = printer.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_api,
        filename: Some(filename.to_string()),
    });
    let factory = extras::save_variables::module_factory();
    let load_config = factory.load_config.expect("load_config present");
    let obj: ObjectRef = load_config(&config).expect("rust load_config").expect("object");
    let rust: Arc<SaveVariables> = downcast_arc(&obj).expect("downcast to SaveVariables");
    (rust, dispatch)
}

fn new_pair(py: &mut pyoracle::Oracle, filename: &str) -> (Arc<SaveVariables>, Arc<FakeGCode>) {
    let (rust, dispatch) = make_rust(filename);
    py.exec(&format!(
        "dispatch = FakeDispatch(); printer = FakePrinter(); printer.objects['gcode'] = dispatch; \
         config = FakeConfig(printer, {}); sv = save_variables.load_config(config)",
        py_repr_str(filename)
    ))
    .expect("py load_config");
    (rust, dispatch)
}

fn save_command(
    gcode: &Arc<FakeGCode>,
    py: &mut pyoracle::Oracle,
    var_name: Option<&str>,
    value: Option<&str>,
) -> (pyoracle::OracleResult, KResult<()>) {
    let mut params = indexmap::IndexMap::new();
    if let Some(n) = var_name {
        params.insert("VARIABLE".to_string(), n.to_string());
    }
    if let Some(v) = value {
        params.insert("VALUE".to_string(), v.to_string());
    }
    let gcmd: GCmd = Arc::new(FakeGCodeCommand {
        commandline: "SAVE_VARIABLE".to_string(),
        params: params.clone(),
    });
    let rust_result = gcode.invoke(&gcmd);
    let mut dict = String::from("{");
    for (i, (k, v)) in params.iter().enumerate() {
        if i > 0 {
            dict.push_str(", ");
        }
        dict.push_str(&format!("{}: {}", py_repr_str(k), py_repr_str(v)));
    }
    dict.push('}');
    py.exec(&format!(
        "gcmd = gcode.GCodeCommand(dispatch, 'SAVE_VARIABLE', 'SAVE_VARIABLE', {dict}, False)"
    ))
    .expect("py make gcmd");
    let expected = py.eval("dispatch.registered['SAVE_VARIABLE'][0](gcmd)");
    (expected, rust_result)
}

fn status_check(py: &mut pyoracle::Oracle, rust: &SaveVariables, label: &str) {
    let expected = py.eval("sv.get_status(0)");
    let actual = rust.get_status(0.0).expect("rust get_status");
    check(label, expected, to_value(&actual));
}

fn file_check(py: &mut pyoracle::Oracle, filename: &str, label: &str) {
    let expected = py.eval(&format!("open({}, 'r').read()", py_repr_str(filename)));
    let bytes = std::fs::read(filename).expect("read variable file");
    let text = String::from_utf8(bytes).expect("variable file is utf8");
    check(label, expected, to_value(&PyValue::Str(text)));
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn missing_file_is_created_empty() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("missing_file");
    let filename = dir.join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8 path").to_string();
    let (rust, _dispatch) = new_pair(&mut py, &filename_str);
    status_check(&mut py, &rust, "missing_file_is_created_empty status");
    file_check(&mut py, &filename_str, "missing_file_is_created_empty file");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn save_and_reload_values() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("save_reload");
    // Two files: one per side, so the oracle and the port never interleave.
    let py_path = dir.join("py.cfg");
    let rust_path = dir.join("rust.cfg");
    let py_filename = py_path.to_str().expect("utf8").to_string();
    let rust_filename = rust_path.to_str().expect("utf8").to_string();
    let (rust, dispatch) = make_rust(&rust_filename);
    py.exec(&format!(
        "dispatch = FakeDispatch(); printer = FakePrinter(); printer.objects['gcode'] = dispatch; \
         config = FakeConfig(printer, {}); sv = save_variables.load_config(config)",
        py_repr_str(&py_filename)
    ))
    .expect("py load_config");

    let cases: &[(&str, &str)] = &[
        ("my_int", "42"),
        ("my_float", "1.5"),
        ("my_str", "'hello'"),
        ("my_bool", "True"),
        ("my_none", "None"),
        ("my_list", "[1, 2.5, 'x']"),
        ("my_tuple", "(1, 2)"),
        ("my_dict", "{'a': 1}"),
        ("big_int", "123456789012345678901234567890"),
        ("big_float", "1e+16"),
        ("unicode_str", "'München'"),
    ];
    for (name, literal) in cases {
        let label = format!("save_and_reload save {name}");
        let gcmd: GCmd = Arc::new(FakeGCodeCommand {
            commandline: "SAVE_VARIABLE".to_string(),
            params: indexmap::IndexMap::from_iter([
                ("VARIABLE".to_string(), name.to_string()),
                ("VALUE".to_string(), literal.to_string()),
            ]),
        });
        let rust_result = dispatch.invoke(&gcmd);
        py.exec(&format!(
            "gcmd = gcode.GCodeCommand(dispatch, 'SAVE_VARIABLE', 'SAVE_VARIABLE', \
             {{'VARIABLE': {}, 'VALUE': {}}}, False)",
            py_repr_str(name),
            py_repr_str(literal)
        ))
        .expect("py make gcmd");
        let expected = py.eval("dispatch.registered['SAVE_VARIABLE'][0](gcmd)");
        check_result(&label, expected, rust_result);
        status_check(&mut py, &rust, &format!("{label} status"));
        file_check(&mut py, &py_filename, &format!("{label} file"));
        let py_text = std::fs::read_to_string(&py_filename).expect("py file");
        let rust_text = std::fs::read_to_string(&rust_filename).expect("rust file");
        assert_eq!(py_text, rust_text, "{label}: file text differs");
    }
    let (rust_reloaded, _dispatch2) = make_rust(&rust_filename);
    // A fresh object on each side reloads the same values from its file.
    py.exec(&format!(
        "config2 = FakeConfig(printer, {}); sv2 = save_variables.load_config(config2)",
        py_repr_str(&py_filename)
    ))
    .expect("py reload");
    let expected = py.eval("sv2.get_status(0)");
    check(
        "save_and_reload reload status",
        expected,
        to_value(&rust_reloaded.get_status(0.0).expect("get_status")),
    );
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn uppercase_variable_is_rejected() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("uppercase");
    let filename = dir.join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8").to_string();
    let (rust, dispatch) = new_pair(&mut py, &filename_str);
    let (expected, actual) = save_command(&dispatch, &mut py, Some("MyVar"), Some("42"));
    check_result("uppercase_variable_is_rejected", expected, actual);
    status_check(&mut py, &rust, "uppercase_variable_is_rejected status");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn unparseable_value_is_rejected() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("unparseable");
    // Separate files so a rejected save cannot leak state across the sides.
    let py_filename = dir.join("py.cfg");
    let rust_filename = dir.join("rust.cfg");
    let py_filename_str = py_filename.to_str().expect("utf8").to_string();
    let rust_filename_str = rust_filename.to_str().expect("utf8").to_string();
    py.exec(&format!(
        "dispatch = FakeDispatch(); printer = FakePrinter(); printer.objects['gcode'] = dispatch; \
         config = FakeConfig(printer, {}); sv = save_variables.load_config(config)",
        py_repr_str(&py_filename_str)
    ))
    .expect("py load_config");
    let (rust, dispatch) = make_rust(&rust_filename_str);
    // Bare names reach `ast.literal_eval`'s `ValueError` branch (malformed
    // node), so both sides raise the "Unable to parse ... as a literal"
    // command error.
    for bad in ["not_a_literal", "undefined_name", "foo"] {
        let label = format!("unparseable_value_is_rejected {bad}");
        let (expected, actual) =
            save_command(&dispatch, &mut py, Some("my_var"), Some(bad));
        check_result(&label, expected, actual);
    }
    status_check(&mut py, &rust, "unparseable_value_is_rejected status");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn missing_parameter_is_rejected() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("missing_param");
    let filename = dir.join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8").to_string();
    let (_rust, dispatch) = new_pair(&mut py, &filename_str);
    // No VARIABLE.
    let (expected, actual) = save_command(&dispatch, &mut py, None, Some("42"));
    check_result("missing_parameter no VARIABLE", expected, actual);
    // No VALUE.
    let (expected, actual) = save_command(&dispatch, &mut py, Some("my_var"), None);
    check_result("missing_parameter no VALUE", expected, actual);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn filename_option_is_required() {
    let Some(mut py) = oracle() else { return };
    py.exec("p2 = FakePrinter(); p2.objects['gcode'] = FakeDispatch(); c2 = FakeConfig(p2)")
        .expect("py setup no filename");
    let expected = py.eval("save_variables.load_config(c2)");
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", FakeGCode::new() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: None,
    });
    let actual = SaveVariables::new(&config);
    let label = "filename_option_is_required";
    match (expected, actual) {
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label} class");
            assert_eq!(err.message(), exc.message.as_str(), "{label} message");
        }
        _ => panic!("{label}: expected oracle error and rust error"),
    }
}

#[test]
fn unparseable_existing_file_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("bad_existing");
    for (tag, content) in [
        ("no_header", "just some text\n"),
        ("bad_value", "[Variables]\nfoo = not_a_literal\n"),
        ("dup_option", "[Variables]\nfoo = 1\nfoo = 2\n"),
        ("interpolation_missing", "[Variables]\nfoo = '%(missing)s'\n"),
    ] {
        let py_filename = dir.join(format!("py_{tag}.cfg"));
        let rust_filename = dir.join(format!("rust_{tag}.cfg"));
        std::fs::write(&py_filename, content).expect("write py file");
        std::fs::write(&rust_filename, content).expect("write rust file");
        let py_path = py_filename.to_str().expect("utf8").to_string();
        let rust_path = rust_filename.to_str().expect("utf8").to_string();
        let expected = py.eval(&format!(
            "(lambda: FakePrinter(), save_variables.SaveVariables(FakeConfig(FakePrinter(), {})))()",
            py_repr_str(&py_path)
        ));
        let printer: Arc<FakePrinter> = FakePrinter::new();
        printer.add("gcode", FakeGCode::new() as ObjectRef);
        let config: Config = Arc::new(FakeConfig {
            printer: printer.clone() as Printer,
            filename: Some(rust_path),
        });
        let actual = SaveVariables::new(&config);
        let label = format!("unparseable_existing_file_is_config_error {tag}");
        match (expected, actual) {
            (Err(exc), Err(err)) => {
                assert_eq!(err.class_name(), exc.class_name.as_str(), "{label} class");
                assert_eq!(err.message(), exc.message.as_str(), "{label} message");
            }
            _ => panic!("{label}: expected oracle error and rust error"),
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn existing_file_is_loaded() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("existing_file");
    let content = "[Variables]\nalpha = 42\nbeta = 'text'\ngamma = [1, 2]\n";
    let py_filename = dir.join("py.cfg");
    let rust_filename = dir.join("rust.cfg");
    std::fs::write(&py_filename, content).expect("write py file");
    std::fs::write(&rust_filename, content).expect("write rust file");
    let py_path = py_filename.to_str().expect("utf8").to_string();
    let rust_path = rust_filename.to_str().expect("utf8").to_string();
    py.exec(&format!(
        "p = FakePrinter(); p.objects['gcode'] = FakeDispatch(); \
         c = FakeConfig(p, {}); sv = save_variables.load_config(c)",
        py_repr_str(&py_path)
    ))
    .expect("py construct");
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", FakeGCode::new() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: Some(rust_path),
    });
    let rust: SaveVariables = SaveVariables::new(&config).expect("rust new");
    status_check(&mut py, &rust, "existing_file_is_loaded status");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn missing_parent_directory_is_os_error() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("no_parent");
    let filename = dir.join("no_such_dir").join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8").to_string();
    py.exec(&format!(
        "p = FakePrinter(); p.objects['gcode'] = FakeDispatch(); \
         c = FakeConfig(p, {})",
        py_repr_str(&filename_str)
    ))
    .expect("py setup");
    let expected = py.eval("save_variables.load_config(c)");
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", FakeGCode::new() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: Some(filename_str),
    });
    let actual = SaveVariables::new(&config);
    let label = "missing_parent_directory_is_os_error";
    match (expected, actual) {
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label} class");
            assert_eq!(err.message(), exc.message.as_str(), "{label} message");
        }
        _ => panic!("{label}: expected oracle error and rust error"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn unknown_gcode_object_is_config_error() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("no_gcode");
    let filename = dir.join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8").to_string();
    py.exec(&format!(
        "p = FakePrinter(); c = FakeConfig(p, {})",
        py_repr_str(&filename_str)
    ))
    .expect("py setup");
    let expected = py.eval("save_variables.load_config(c)");
    let printer: Arc<FakePrinter> = FakePrinter::new();
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: Some(filename_str),
    });
    let factory = extras::save_variables::module_factory();
    let load_config = factory.load_config.expect("load_config present");
    let actual = load_config(&config);
    let label = "unknown_gcode_object_is_config_error";
    match (expected, actual) {
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label} class");
            assert_eq!(err.message(), exc.message.as_str(), "{label} message");
        }
        _ => panic!("{label}: expected oracle error and rust error"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn tilde_filename_expands() {
    let Some(mut py) = oracle() else { return };
    // Same process environment on both sides (`HOME` is inherited), so the
    // expanded path must agree; the file lands in the real home directory.
    let tag = format!("save_variables_home_{}.cfg", std::process::id());
    let filename_str = format!("~/{tag}");
    py.exec(&format!(
        "p = FakePrinter(); p.objects['gcode'] = FakeDispatch(); \
         c = FakeConfig(p, {}); sv = save_variables.load_config(c)",
        py_repr_str(&filename_str)
    ))
    .expect("py construct");
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", FakeGCode::new() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: Some(filename_str),
    });
    let rust: SaveVariables = SaveVariables::new(&config).expect("rust new");
    status_check(&mut py, &rust, "tilde_filename_expands status");
    // Both created the same file in the home directory.
    let home = std::env::var("HOME").expect("HOME set");
    let path = std::path::Path::new(&home).join(&tag);
    assert!(path.exists(), "expanded file must exist");
    let _ = std::fs::remove_file(&path);
}

#[test]
fn interpolation_syntax_on_save_is_internal_error() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("interp_save");
    let py_filename = dir.join("py.cfg");
    let rust_filename = dir.join("rust.cfg");
    let py_path = py_filename.to_str().expect("utf8").to_string();
    let rust_path = rust_filename.to_str().expect("utf8").to_string();
    py.exec(&format!(
        "dispatch = FakeDispatch(); printer = FakePrinter(); printer.objects['gcode'] = dispatch; \
         config = FakeConfig(printer, {}); sv = save_variables.load_config(config)",
        py_repr_str(&py_path)
    ))
    .expect("py load_config");
    let (rust, dispatch) = make_rust(&rust_path);
    // A string value whose repr contains `%` trips ConfigParser.set's
    // interpolation validation (`ValueError` in Python, uncaught there).
    let label = "interpolation_syntax_on_save_is_internal_error";
    let (expected, actual) = save_command(&dispatch, &mut py, Some("pct"), Some("'100%'"));
    match (expected, actual) {
        (Err(exc), Err(err)) => {
            assert_eq!(err.class_name(), exc.class_name.as_str(), "{label} class");
            assert_eq!(err.message(), exc.message.as_str(), "{label} message");
        }
        _ => panic!("{label}: expected oracle error and rust error"),
    }
    // State is unchanged on both sides.
    status_check(&mut py, &rust, "interpolation_syntax_on_save_is_internal_error status");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn factory_registers_save_variable() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("factory");
    let filename = dir.join("variables.cfg");
    let filename_str = filename.to_str().expect("utf8").to_string();
    let factory = extras::save_variables::module_factory();
    let load_config = factory.load_config.expect("load_config present");
    assert!(factory.load_config_prefix.is_none(), "no load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none(), "no add_early_printer_objects");
    assert!(factory.add_printer_objects.is_none(), "no add_printer_objects");
    let dispatch = FakeGCode::new();
    let printer: Arc<FakePrinter> = FakePrinter::new();
    printer.add("gcode", dispatch.clone() as ObjectRef);
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone() as Printer,
        filename: Some(filename_str.clone()),
    });
    let obj: ObjectRef = load_config(&config).expect("factory load_config").expect("object");
    let rust: Arc<SaveVariables> = downcast_arc(&obj).expect("downcast to SaveVariables");
    // Registration matches the Python registration (cmd, when_not_ready, desc).
    py.exec(&format!(
        "dispatch = FakeDispatch(); printer = FakePrinter(); printer.objects['gcode'] = dispatch; \
         config = FakeConfig(printer, {}); sv = save_variables.load_config(config)",
        py_repr_str(&filename_str)
    ))
    .expect("py load_config");
    let expected = py.eval(
        "sorted('%s|%s|%r' % (k, v[1], v[2]) for k, v in dispatch.registered.items())",
    );
    let actual = dispatch
        .registrations()
        .iter()
        .map(|(cmd, when_not_ready, desc)| {
            Value::str(&format!(
                "{cmd}|{}|{}",
                if *when_not_ready { "True" } else { "False" },
                match desc {
                    None => "None".to_string(),
                    Some(d) => py_repr_str(d),
                }
            ))
        })
        .collect::<Vec<_>>();
    check("factory_registers_save_variable registration", expected, Value::List(actual));
    status_check(&mut py, &rust, "factory_registers_save_variable status");
    let _ = std::fs::remove_dir_all(&dir);
}
