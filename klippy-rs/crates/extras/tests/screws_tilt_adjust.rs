//! Parity tests for `klippy/extras/screws_tilt_adjust.py`.

use extras::manual_probe::ProbeResult;
use extras::screws_tilt_adjust::ScrewsTiltAdjust;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, OutputCallback,
    RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import configparser
import extras.screws_tilt_adjust as screws_tilt_adjust
import extras.manual_probe as manual_probe

class CommandError(Exception):
    pass

class sentinel:
    pass

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.registrations = []
        self.messages = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registrations.append((cmd, func is not None, desc))
        return None
    def respond_info(self, msg, log=True):
        self.messages.append((msg, log))

class FakeGCmd:
    error = CommandError
    def __init__(self, commandline, params):
        self._commandline = commandline
        self._params = dict(params)
    def get(self, name, default=sentinel):
        if name in self._params:
            return self._params[name]
        if default is sentinel:
            raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
        return default
    def get_float(self, name, default=sentinel, minval=None, maxval=None, above=None, below=None):
        if name not in self._params:
            if default is sentinel:
                raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
            return default
        value = float(self._params[name])
        if minval is not None and value < minval:
            raise CommandError("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise CommandError("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise CommandError("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise CommandError("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value
    def get_commandline(self):
        return self._commandline

class FakePrinter:
    command_error = CommandError
    def __init__(self, gcode=None):
        self.objects = {}
        if gcode is not None:
            self.objects['gcode'] = gcode
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, options, section='screws_tilt_adjust'):
        self._printer = printer
        self.options = dict(options)
        self.section = section
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def get(self, option, default=sentinel, note_valid=True):
        if option in self.options:
            return self.options[option]
        if default is sentinel:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return default
    def getfloat(self, option, default=sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self.options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        value = float(self.options[option])
        if above is not None and value <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        return value
    def getfloatlist(self, option, default=sentinel, sep=',', count=None, note_valid=True):
        if option not in self.options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            return default
        value = self.options[option]
        parts = [p.strip() for p in value.split(sep)] if value.strip() else []
        res = [float(p) for p in parts]
        if count is not None and len(res) != count:
            raise configparser.Error("Option '%s' in section '%s' must have %d elements" % (option, self.section, count))
        return tuple(res)
    def getchoice(self, option, choices, default=sentinel, note_valid=True):
        if option not in self.options:
            if default is sentinel:
                raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
            key = default
        else:
            key = self.options[option]
        if key in choices:
            return choices[key]
        raise configparser.Error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (key, option, self.section))
    def getlists(self, option, seps=(',',), pos_only=None, parser=str, func=None, count=None, default=None):
        return ()
"#;

#[derive(Default)]
struct Logs {
    registrations: Vec<(String, bool, Option<String>)>,
    messages: Vec<(String, bool)>,
}

struct FakeGCode {
    logs: GCell<Logs>,
}

impl FakeGCode {
    fn new() -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            logs: GCell::new(Logs::default()),
        })
    }
}

impl klippy_api::PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(PyValue::Dict(IndexMap::new()))
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.logs.borrow_mut().registrations.push((
            cmd.to_string(),
            func.is_some(),
            opts.desc.clone(),
        ));
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
        panic!("register_mux_command unused")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
        panic!("get_mutex unused")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
            logs: None,
        })
    }
    fn respond_raw(&self, msg: &str) {
        self.logs.borrow_mut().messages.push((msg.to_string(), true));
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.logs.borrow_mut().messages.push((msg.to_string(), log));
    }
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
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    logs: Option<Arc<FakeGCode>>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        if let Some(v) = self.params.get(name) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            Some(v) => Ok(v),
            None => Err(KlippyError::command(format!(
                "Error on '{}': missing {name}",
                self.commandline
            ))),
        }
    }
    fn get_int(&self, _name: &str, _opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int unused")
    }
    fn get_float(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetFloatOpts,
    ) -> KResult<Option<f64>> {
        if let Some(v) = self.params.get(name) {
            return v.parse::<f64>().map(Some).map_err(|_| {
                KlippyError::command(format!(
                    "Error on '{}': unable to parse {name}",
                    self.commandline
                ))
            });
        }
        match opts.default {
            Some(v) => Ok(v),
            None => Err(KlippyError::command(format!(
                "Error on '{}': missing {name}",
                self.commandline
            ))),
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
        self.commandline.clone()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        if let Some(g) = &self.logs {
            g.logs.borrow_mut().messages.push((msg.to_string(), log));
        }
    }
    fn respond_raw(&self, msg: &str) {
        if let Some(g) = &self.logs {
            g.logs.borrow_mut().messages.push((msg.to_string(), true));
        }
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
}

klippy_api::printer_object!(FakePrinter);

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
        self.objects.borrow_mut().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
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
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
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
            Some(v) => Ok(v),
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.section
            ))),
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(opts.default.unwrap_or(None))
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
        let def = opts.default.clone().unwrap_or(None).unwrap_or(PyValue::None);
        let key = match self.options.get(option) {
            Some(raw) => PyValue::Str(raw.clone()),
            None => def,
        };
        if let PyValue::Dict(d) = choices {
            for (k, v) in d.iter() {
                if *k == key {
                    return Ok(v.clone());
                }
            }
        }
        let shown = match &key {
            PyValue::Str(s) => s.clone(),
            PyValue::None => "None".to_string(),
            other => format!("{other:?}"),
        };
        Err(KlippyError::config(format!(
            "Choice '{shown}' for option '{option}' in section '{}' is not a valid choice",
            self.section
        )))
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::Tuple(Vec::new()))
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let raw = match self.options.get(option) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    Some(v) => Ok(v),
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                };
            }
        };
        let parts: Vec<&str> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(&opts.sep).map(str::trim).collect()
        };
        let values: Vec<f64> = parts
            .iter()
            .map(|p| {
                p.parse::<f64>().map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{option}' in section '{}'",
                        self.section
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        if let Some(count) = opts.count {
            if values.len() != count {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.section
                )));
            }
        }
        Ok(Some(values))
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection unused")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(_) => Value::Null,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "screws_tilt_adjust".to_string(),
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

fn check_call(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match expected {
        Ok(_) => {
            if let Err(e) = actual {
                panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
            }
            check(label, Ok(Value::Null), Value::Null);
        }
        Err(e) => {
            let rust_err = match actual {
                Ok(()) => panic!("{label}: rust returned Ok but python must fail"),
                Err(e) => e,
            };
            assert_eq!(
                rust_err.class_name(),
                e.class_name.as_str(),
                "{label}: exception class"
            );
            assert_eq!(
                rust_err.message(),
                e.message.as_str(),
                "{label}: exception message"
            );
        }
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("screws fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping screws_tilt_adjust oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary"
            );
            None
        }
    }
}

fn dict_literal(options: &[(&str, &str)]) -> String {
    let pairs: Vec<String> = options
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", pairs.join(", "))
}

fn base_options<'a>(extra: &'a [(&'a str, &'a str)]) -> Vec<(&'a str, &'a str)> {
    let mut v = vec![
        ("screw1", "0,0"),
        ("screw2", "200,0"),
        ("screw3", "0,200"),
        ("screw4", "200,200"),
    ];
    v.extend(extra.iter().copied());
    v
}

struct World {
    rust: KResult<Arc<ScrewsTiltAdjust>>,
    gcode: Arc<FakeGCode>,
}

fn build_rust(options: &[(&str, &str)]) -> World {
    let gcode = FakeGCode::new();
    let printer: Printer = Arc::new(FakePrinter {
        objects: GCell::new(IndexMap::new()),
    });
    printer
        .add_object("gcode", gcode.clone() as ObjectRef)
        .expect("add gcode");
    let map: IndexMap<String, String> = options
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let config: Config = Arc::new(FakeConfig {
        printer,
        section: "screws_tilt_adjust".to_string(),
        options: map,
    });
    let rust = ScrewsTiltAdjust::new(&config);
    World { rust, gcode }
}

fn build_py(py: &mut pyoracle::Oracle, options: &[(&str, &str)]) -> pyoracle::OracleResult {
    let code = format!(
        "g = FakeGCode()\np = FakePrinter(g)\nc = FakeConfig(p, {})\nobj = screws_tilt_adjust.load_config(c)",
        dict_literal(options)
    );
    py.exec(&code)
}

fn rust_positions(zs: &[f64]) -> Vec<ProbeResult> {
    zs.iter()
        .map(|&z| ProbeResult {
            bed_x: 0.0,
            bed_y: 0.0,
            bed_z: z,
            test_x: 0.0,
            test_y: 0.0,
            test_z: 0.0,
        })
        .collect()
}

fn py_positions_expr(zs: &[f64]) -> String {
    let parts: Vec<String> = zs
        .iter()
        .map(|&z| format!("manual_probe.ProbeResult(0.0, 0.0, {z:?}, 0.0, 0.0, 0.0)"))
        .collect();
    format!("[{}]", parts.join(", "))
}

fn params_expr(params: &[(&str, &str)]) -> String {
    let pairs: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", pairs.join(", "))
}

fn status_value(rust: &Arc<ScrewsTiltAdjust>) -> Value {
    pyvalue_to_value(&rust.get_status_value())
}

fn messages_value(gcode: &Arc<FakeGCode>) -> Value {
    Value::List(
        gcode
            .logs
            .borrow()
            .messages
            .iter()
            .map(|(msg, log)| Value::Tuple(vec![Value::str(msg), Value::Bool(*log)]))
            .collect(),
    )
}

fn registrations_value(gcode: &Arc<FakeGCode>) -> Value {
    Value::List(
        gcode
            .logs
            .borrow()
            .registrations
            .iter()
            .map(|(cmd, registered, desc)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::Bool(*registered),
                    match desc {
                        Some(d) => Value::str(d),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

fn rust_cmd(rust: &Arc<ScrewsTiltAdjust>, commandline: &str, params: &[(&str, &str)]) -> KResult<()>
{
    let map: IndexMap<String, String> = params
        .iter()
        .map(|(k, v)| (k.to_string(), v.to_string()))
        .collect();
    let gcmd: GCmd = Arc::new(FakeGCmd {
        command: "SCREWS_TILT_CALCULATE".to_string(),
        commandline: commandline.to_string(),
        params: map,
        logs: None,
    });
    rust.cmd_screws_tilt_calculate(&gcmd)
}

/// Runs `SCREWS_TILT_CALCULATE` on both sides with the given parameters.
///
/// With this fake printer the command stops inside
/// `ProbePointsHelper.start_probe` (`lookup_object('toolhead')`), which is past
/// the `MAX_DEVIATION`/`DIRECTION` assignments that `probe_finalize` reads, so
/// both sides fail the same way while the attributes stay set.
fn run_cmd(
    label: &str,
    py: &mut pyoracle::Oracle,
    rust_obj: &Arc<ScrewsTiltAdjust>,
    params: &[(&str, &str)],
) {
    let suffix: String = params
        .iter()
        .map(|(k, v)| format!(" {k}={v}"))
        .collect();
    let cmdline = format!("SCREWS_TILT_CALCULATE{suffix}");
    let py_code = format!(
        "gc = FakeGCmd({cmdline:?}, {})\nobj.cmd_SCREWS_TILT_CALCULATE(gc)",
        params_expr(params)
    );
    let expected = py.exec(&py_code);
    let actual = rust_cmd(rust_obj, &cmdline, params);
    check_call(label, expected, actual);
}

fn finalize_case(label: &str, py: &mut pyoracle::Oracle, thread: Option<&str>, zs: &[f64]) {
    let extra: Vec<(&str, &str)> = match thread {
        Some(t) => vec![("screw_thread", t)],
        None => Vec::new(),
    };
    let options = base_options(&extra);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(py, &options);
    if let Err(e) = py_result {
        panic!("{label}: py load_config raised {}: {}", e.class_name, e.message);
    }
    // `probe_finalize` reads `self.direction`, which the command assigns; a
    // real finalize always follows a `SCREWS_TILT_CALCULATE`.
    run_cmd(&format!("{label} cmd"), py, &rust_obj, &[]);
    let py_code = format!("obj.probe_finalize({})", py_positions_expr(zs));
    let expected = py.eval(&py_code);
    let actual = rust_obj.probe_finalize(&rust_positions(zs));
    check_call(label, expected, actual);
    check(
        &format!("{label} status"),
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
    check(
        &format!("{label} messages"),
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
}

#[test]
fn finalize_default_threads() {
    let Some(mut py) = oracle() else { return };
    finalize_case("finalize CW-M3 default base", &mut py, None, &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize CCW-M3", &mut py, Some("CCW-M3"), &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize CW-M4", &mut py, Some("CW-M4"), &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize CCW-M4", &mut py, Some("CCW-M4"), &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize CW-M5", &mut py, Some("CW-M5"), &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize CW-M6", &mut py, Some("CW-M6"), &[1.0, 1.5, 0.5, 1.0]);
    finalize_case("finalize tiny diff", &mut py, None, &[1.0, 1.0001, 1.001, 0.9995]);
    finalize_case("finalize negative adjust", &mut py, None, &[1.0, 0.0, 2.5, 1.0]);
    finalize_case("finalize many turns", &mut py, None, &[1.0, 21.0, 1.0, 1.0]);
    finalize_case("finalize half minute", &mut py, None, &[1.0, 1.25, 1.0, 1.0]);
}

#[test]
fn config_needs_three_screws() {
    let Some(mut py) = oracle() else { return };
    let options = vec![
        ("screw1", "0,0"),
        ("screw2", "200,0"),
    ];
    let world = build_rust(&options);
    let rust_res = match world.rust {
        Ok(_) => Ok(()),
        Err(e) => Err(e),
    };
    let expected = build_py(&mut py, &options);
    check_call("three screws", expected, rust_res);
}

#[test]
fn load_registers_command() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let _rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    check(
        "registrations",
        py.eval("g.registrations"),
        registrations_value(&world.gcode),
    );
}

#[test]
fn cmd_rejects_bad_direction() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    // The message quotes `gcmd.get_commandline()`.
    run_cmd(
        "bad direction",
        &mut py,
        &rust_obj,
        &[("DIRECTION", "DOWN")],
    );
}

fn direction_finalize_case(
    label: &str,
    py: &mut pyoracle::Oracle,
    thread: Option<&str>,
    direction: &str,
    zs: &[f64],
) {
    let extra: Vec<(&str, &str)> = match thread {
        Some(t) => vec![("screw_thread", t)],
        None => Vec::new(),
    };
    let options = base_options(&extra);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(py, &options);
    if let Err(e) = py_result {
        panic!("{label}: py load_config raised {}: {}", e.class_name, e.message);
    }
    run_cmd(
        &format!("{label} cmd"),
        py,
        &rust_obj,
        &[("DIRECTION", direction)],
    );
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(zs));
    check_call(&format!("{label} finalize"), fin_expected, fin_actual);
    check(
        &format!("{label} status"),
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
    check(
        &format!("{label} messages"),
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
}

#[test]
fn finalize_with_direction() {
    let Some(mut py) = oracle() else { return };
    direction_finalize_case("CW with CW thread", &mut py, None, "CW", &[1.0, 1.2, 2.0, 1.5]);
    direction_finalize_case("CCW with CW thread", &mut py, None, "CCW", &[1.0, 1.2, 2.0, 1.5]);
    direction_finalize_case("CW with CCW thread", &mut py, Some("CCW-M3"), "CW", &[1.0, 1.2, 2.0, 1.5]);
    direction_finalize_case("CCW with CCW thread", &mut py, Some("CCW-M3"), "CCW", &[1.0, 1.2, 2.0, 1.5]);
    direction_finalize_case("lowercase cw", &mut py, None, "cw", &[1.0, 1.2, 2.0, 1.5]);
}

#[test]
fn max_deviation_raises() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    let zs = [1.0, 1.5, 1.0, 1.0];
    run_cmd(
        "max deviation cmd",
        &mut py,
        &rust_obj,
        &[("MAX_DEVIATION", "0.1")],
    );
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
    check_call("max deviation finalize", fin_expected, fin_actual);
    check(
        "max deviation status",
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
    check(
        "max deviation messages",
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
}

#[test]
fn max_deviation_zero_and_none_do_not_raise() {
    let Some(mut py) = oracle() else { return };
    {
        let options = base_options(&[]);
        let world = build_rust(&options);
        let rust_obj = world.rust.expect("rust load_config");
        let py_result = build_py(&mut py, &options);
        if let Err(e) = py_result {
            panic!("py load_config raised {}: {}", e.class_name, e.message);
        }
        let zs = [1.0, 2.0, 1.0, 1.0];
        run_cmd(
            "zero cmd",
            &mut py,
            &rust_obj,
            &[("MAX_DEVIATION", "0")],
        );
        let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
        let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
        check_call("zero finalize", fin_expected, fin_actual);
        check(
            "zero status",
            py.eval("obj.get_status(0)"),
            status_value(&rust_obj),
        );
    }
    {
        let options = base_options(&[]);
        let world = build_rust(&options);
        let rust_obj = world.rust.expect("rust load_config");
        let py_result = build_py(&mut py, &options);
        if let Err(e) = py_result {
            panic!("py load_config raised {}: {}", e.class_name, e.message);
        }
        let zs = [1.0, 2.0, 1.0, 1.0];
        // No MAX_DEVIATION: the command sets both attributes to None.
        run_cmd("none cmd", &mut py, &rust_obj, &[]);
        let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
        let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
        check_call("none finalize", fin_expected, fin_actual);
        check(
            "none status",
            py.eval("obj.get_status(0)"),
            status_value(&rust_obj),
        );
    }
}

#[test]
fn custom_screw_names() {
    let Some(mut py) = oracle() else { return };
    let options = vec![
        ("screw1", "10,20"),
        ("screw1_name", "front-left"),
        ("screw2", "200,0"),
        ("screw3", "0,200"),
    ];
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    let zs = [1.0, 1.5, 0.5];
    run_cmd("custom names cmd", &mut py, &rust_obj, &[]);
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
    check_call("custom names finalize", fin_expected, fin_actual);
    check(
        "custom names messages",
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
    check(
        "custom names status",
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
}

#[test]
fn invalid_thread_rejected() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[("screw_thread", "BOGUS")]);
    let world = build_rust(&options);
    let rust_res = match world.rust {
        Ok(_) => Ok(()),
        Err(e) => Err(e),
    };
    let expected = build_py(&mut py, &options);
    check_call("invalid thread", expected, rust_res);
}

#[test]
fn initial_status() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    check(
        "initial status",
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
}

#[test]
fn float_spelling_in_positions() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    let zs = [0.5, -0.25, 1e-05, 1.0];
    run_cmd("float spelling cmd", &mut py, &rust_obj, &[]);
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
    check_call("float spelling finalize", fin_expected, fin_actual);
    check(
        "float spelling messages",
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
}

#[test]
fn finalize_before_command_raises() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    // Python assigns `self.direction` only in the command, so finalizing first
    // raises AttributeError.
    let zs = [1.0, 1.5, 0.5, 1.0];
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
    check_call("finalize before command", fin_expected, fin_actual);
    check(
        "finalize before command status",
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
    check(
        "finalize before command messages",
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
}

#[test]
fn finalize_short_positions_raises() {
    let Some(mut py) = oracle() else { return };
    let options = base_options(&[]);
    let world = build_rust(&options);
    let rust_obj = world.rust.expect("rust load_config");
    let py_result = build_py(&mut py, &options);
    if let Err(e) = py_result {
        panic!("py load_config raised {}: {}", e.class_name, e.message);
    }
    run_cmd("short positions cmd", &mut py, &rust_obj, &[]);
    // `positions` is indexed by screw number: one result missing raises
    // IndexError after the earlier screws were reported.
    let zs = [1.0, 1.5, 0.5];
    let fin_expected = py.eval(&format!("obj.probe_finalize({})", py_positions_expr(&zs)));
    let fin_actual = rust_obj.probe_finalize(&rust_positions(&zs));
    check_call("short positions", fin_expected, fin_actual);
    check(
        "short positions messages",
        py.eval("g.messages"),
        messages_value(&world.gcode),
    );
    check(
        "short positions status",
        py.eval("obj.get_status(0)"),
        status_value(&rust_obj),
    );
}

#[test]
fn finalize_empty_positions_raises() {
    let Some(mut py) = oracle() else { return };
    for direction in ["CW", "CCW"] {
        let options = base_options(&[]);
        let world = build_rust(&options);
        let rust_obj = world.rust.expect("rust load_config");
        let py_result = build_py(&mut py, &options);
        if let Err(e) = py_result {
            panic!("py load_config raised {}: {}", e.class_name, e.message);
        }
        let label = format!("empty positions {direction}");
        run_cmd(
            &format!("{label} cmd"),
            &mut py,
            &rust_obj,
            &[("DIRECTION", direction)],
        );
        let fin_expected = py.eval("obj.probe_finalize([])");
        let fin_actual = rust_obj.probe_finalize(&[]);
        check_call(&label, fin_expected, fin_actual);
        check(
            &format!("{label} status"),
            py.eval("obj.get_status(0)"),
            status_value(&rust_obj),
        );
    }
}
