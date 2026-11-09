//! Parity tests for `extras::gcode_arcs` (port of `klippy/extras/gcode_arcs.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `ArcSupport` is built
//! on both sides with minimal Python fakes for the printer, the `gcode`
//! dispatcher and the `gcode_move` objects, then fed identical G2/G3/G17/
//! G18/G19 commands. The synthetic G1 parameter sequences (floats by bits),
//! the plane selection, the registered commands, the `resolution` config
//! read and every `CommandError` are compared exactly (class and message).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::gcode_arcs::{ArcSupport, ARC_PLANE_X_Y, ARC_PLANE_X_Z, ARC_PLANE_Y_Z};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetOpts as GCmdGetOpts,
    GetFloatOpts as GCmdFloatOpts, GetIntOpts as GCmdIntOpts, GCmd, GCodeCommandHandler,
    GCodeMoveApi, MoveTransform, OutputCallback, RegisterCommandOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use pycompat::PyValue;
use pycompat::text::{float_from_string, float_str};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import gcode as gcode_mod
import extras.gcode_arcs as gcode_arcs

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.load_calls = []
        self.lookup_calls = []
    def lookup_object(self, name, default=None):
        self.lookup_calls.append(name)
        if name in self.objects:
            return self.objects[name]
        raise configparser.Error("Unknown config object '%s'" % (name,))
    def load_object(self, config, section, default=None):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        raise configparser.Error("Unable to load module '%s'" % (section,))

class FakeGCodeMove:
    def __init__(self):
        self.position = (0., 0., 0., 0.)
        self.absolute_coord = True
        self.absolute_extrude = True
        self.g1 = []
    def get_status(self, eventtime=None):
        return {'absolute_coordinates': self.absolute_coord,
                'absolute_extrude': self.absolute_extrude,
                'gcode_position': gcode_mod.Coord(self.position)}
    def cmd_G1(self, gcmd):
        self.g1.append((gcmd.get_command(), dict(gcmd.get_command_parameters())))

class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.registered = []
        self.created = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if cmd in self.handlers:
            raise configparser.Error("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        self.registered.append(cmd)
        return None
    def create_gcode_command(self, command, commandline, params):
        gcmd = gcode_mod.GCodeCommand(self, command, commandline, params, False)
        self.created.append((command, commandline, dict(params)))
        return gcmd
    def respond_info(self, msg, log=True):
        pass
    def respond_raw(self, msg):
        pass

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, resolution=None, section='gcode_arcs'):
        self._printer = printer
        self._resolution = resolution
        self._section = section
        self.getfloat_calls = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._section
    def getfloat(self, option, default=None, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self.getfloat_calls.append((option, default, above))
        if option == 'resolution' and self._resolution is not None:
            v = self._resolution
        elif default is not None:
            return default
        else:
            raise configparser.Error(
                "Option '%s' in section '%s' must be specified" % (option, self._section))
        if above is not None and v <= above:
            raise configparser.Error(
                "Option '%s' in section '%s' must be above %s" % (option, self._section, above))
        return v
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct FakePrinterInner {
    load_calls: Vec<String>,
    lookup_calls: Vec<String>,
}

struct FakePrinter {
    objects: IndexMap<String, ObjectRef>,
    inner: GCell<FakePrinterInner>,
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
    fn add_object(&self, name: &str, _obj: ObjectRef) -> KResult<()> {
        if self.objects.contains_key(name) {
            return Err(KlippyError::Config(format!("Printer object '{name}' already created")));
        }
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.inner.borrow_mut().lookup_calls.push(name.to_string());
        if let Some(obj) = self.objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .iter()
            .map(|(name, obj)| (name.clone(), obj.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.inner.borrow_mut().load_calls.push(section.to_string());
        if let Some(obj) = self.objects.get(section) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unable to load module '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
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
    section: String,
    resolution: Option<f64>,
    calls: GCell<Vec<(String, Option<f64>, Option<f64>)>>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let default = opts.default.expect("recorded default");
        self.calls.borrow_mut().push((option.to_string(), default, opts.above));
        let value = if option == "resolution" {
            match self.resolution {
                Some(v) => v,
                None => match default {
                    Some(v) => return Ok(Some(v)),
                    None => {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must be specified",
                            self.section
                        )))
                    }
                },
            }
        } else {
            match default {
                Some(v) => return Ok(Some(v)),
                None => {
                    return Err(KlippyError::Config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    )))
                }
            }
        };
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_str(above)
                )));
            }
        }
        Ok(Some(value))
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
        panic!("getsection is not used by gcode_arcs")
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

struct FakeGCodeCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCodeCommand {
    fn new(command: &str, commandline: &str, params: &[(&str, &str)]) -> Self {
        FakeGCodeCommand {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params
                .iter()
                .map(|(k, v)| (k.to_string(), v.to_string()))
                .collect(),
        }
    }
    fn missing(name: &str, commandline: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': missing {name}"))
    }
    fn unparsable(value: &str, commandline: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{commandline}': unable to parse {value}"))
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GCmdGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(Self::missing(name, &self.commandline)),
                Some(v) => Ok(v),
            },
            Some(value) => Ok(PyValue::Str(value.clone())),
        }
    }
    fn get_int(&self, name: &str, opts: GCmdIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            None => match opts.default {
                None => Err(Self::missing(name, &self.commandline)),
                Some(v) => Ok(v),
            },
            Some(value) => match value.parse::<i64>() {
                Ok(v) => Ok(Some(v)),
                Err(_) => Err(Self::unparsable(value, &self.commandline)),
            },
        }
    }
    fn get_float(&self, name: &str, opts: GCmdFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(Self::missing(name, &self.commandline)),
                Some(v) => Ok(v),
            };
        };
        let value = float_from_string(raw)
            .map_err(|_| Self::unparsable(raw, &self.commandline))?;
        if let Some(minval) = opts.minval {
            if value < minval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    float_str(minval)
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if value > maxval {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    float_str(maxval)
                )));
            }
        }
        if let Some(above) = opts.above {
            if value <= above {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if value >= below {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    float_str(below)
                )));
            }
        }
        Ok(Some(value))
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
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeGCode {
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
    registered: GCell<Vec<String>>,
    created: GCell<Vec<(String, String, Vec<(String, String)>)>>,
}

impl FakeGCode {
    fn new() -> Self {
        FakeGCode {
            handlers: GCell::new(IndexMap::new()),
            registered: GCell::new(Vec::new()),
            created: GCell::new(Vec::new()),
        }
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let Some(func) = func else {
            let old = self.handlers.borrow_mut().shift_remove(cmd);
            return Ok(old);
        };
        if self.handlers.borrow().contains_key(cmd) {
            return Err(KlippyError::Config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        self.handlers.borrow_mut().insert(cmd.to_string(), func);
        self.registered.borrow_mut().push(cmd.to_string());
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        let _ = cmd;
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
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        self.created.borrow_mut().push((
            command.to_string(),
            commandline.to_string(),
            params.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
        ));
        Arc::new(FakeGCodeCommand {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        let Some(first) = cmd.split_whitespace().next() else {
            return false;
        };
        let mut bytes = first.chars();
        let letter = bytes.next();
        let rest = bytes.as_str();
        matches!(letter, Some(c) if c.is_uppercase())
            && !rest.is_empty()
            && rest.chars().all(|c| c.is_ascii_digit())
            && first.parse::<f64>().is_ok()
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGCodeMove {
    position: GCell<Vec<f64>>,
    absolute_coordinates: bool,
    absolute_extrude: bool,
    g1: GCell<Vec<(String, Vec<(String, String)>)>>,
}

impl FakeGCodeMove {
    fn new(position: [f64; 4], absolute_coordinates: bool, absolute_extrude: bool) -> Self {
        FakeGCodeMove {
            position: GCell::new(position.to_vec()),
            absolute_coordinates,
            absolute_extrude,
            g1: GCell::new(Vec::new()),
        }
    }
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut dict: IndexMap<PyValue, PyValue> = IndexMap::new();
        dict.insert(
            PyValue::Str("absolute_coordinates".to_string()),
            PyValue::Bool(self.absolute_coordinates),
        );
        dict.insert(
            PyValue::Str("absolute_extrude".to_string()),
            PyValue::Bool(self.absolute_extrude),
        );
        let position = self
            .position
            .borrow()
            .iter()
            .map(|&x| PyValue::Float(x))
            .collect::<Vec<_>>();
        dict.insert(
            PyValue::Str("gcode_position".to_string()),
            PyValue::Tuple(position),
        );
        PyValue::Dict(dict)
    }
    fn set_move_transform(
        &self,
        _transform: MoveTransform,
        _force: bool,
    ) -> KResult<Option<MoveTransform>> {
        Ok(None)
    }
    fn reset_last_position(&self) -> KResult<()> { Ok(()) }
    fn cmd_g1(&self, gcmd: &GCmd) -> KResult<()> {
        self.g1.borrow_mut().push((
            gcmd.get_command(),
            gcmd.get_command_parameters()
                .iter()
                .map(|(k, v)| (k.clone(), v.clone()))
                .collect(),
        ));
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

// ---------------------------------------------------------------------------
// Rigs and comparison plumbing.
// ---------------------------------------------------------------------------

#[allow(dead_code)]
struct Rig {
    created: KResult<Arc<ArcSupport>>,
    fake_gmove: Arc<FakeGCodeMove>,
    fake_gcode: Arc<FakeGCode>,
    fake_config: Arc<FakeConfig>,
    config: Config,
}

fn rig_parts(
    resolution: Option<f64>,
    position: [f64; 4],
    absolute_coordinates: bool,
    absolute_extrude: bool,
    with_gmove: bool,
    with_gcode: bool,
) -> (Arc<FakeGCodeMove>, Arc<FakeGCode>, Arc<FakeConfig>, Config) {
    let fake_gmove = Arc::new(FakeGCodeMove::new(position, absolute_coordinates, absolute_extrude));
    let fake_gcode = Arc::new(FakeGCode::new());
    let mut objects: IndexMap<String, ObjectRef> = IndexMap::new();
    if with_gmove {
        objects.insert("gcode_move".to_string(), fake_gmove.clone() as ObjectRef);
    }
    if with_gcode {
        objects.insert("gcode".to_string(), fake_gcode.clone() as ObjectRef);
    }
    let printer: Printer = Arc::new(FakePrinter {
        objects,
        inner: GCell::new(FakePrinterInner::default()),
    });
    let fake_config = Arc::new(FakeConfig {
        printer,
        section: "gcode_arcs".to_string(),
        resolution,
        calls: GCell::new(Vec::new()),
    });
    let config: Config = fake_config.clone();
    (fake_gmove, fake_gcode, fake_config, config)
}

fn rig(
    resolution: Option<f64>,
    position: [f64; 4],
    absolute_coordinates: bool,
    absolute_extrude: bool,
    with_gmove: bool,
    with_gcode: bool,
) -> Rig {
    let (fake_gmove, fake_gcode, fake_config, config) =
        rig_parts(resolution, position, absolute_coordinates, absolute_extrude, with_gmove, with_gcode);
    let created = ArcSupport::new(&config);
    Rig {
        created,
        fake_gmove,
        fake_gcode,
        fake_config,
        config,
    }
}

fn created_clone(rig: &Rig) -> KResult<Arc<ArcSupport>> {
    rig.created.clone()
}


fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("gcode_arcs fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping gcode_arcs oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_position_expr(position: [f64; 4]) -> String {
    let parts: Vec<String> = position.iter().map(|&x| float_str(x)).collect();
    format!("({})", parts.join(", "))
}

fn py_setup(
    py: &mut pyoracle::Oracle,
    resolution: Option<f64>,
    position: [f64; 4],
    absolute_coordinates: bool,
    absolute_extrude: bool,
    with_gmove: bool,
    with_gcode: bool,
) {
    let resolution_expr = match resolution {
        Some(v) => float_str(v),
        None => "None".to_string(),
    };
    let mut objects = String::new();
    if with_gmove {
        objects.push_str("p.objects['gcode_move'] = gm;");
    }
    if with_gcode {
        objects.push_str("p.objects['gcode'] = gc;");
    }
    py.exec(&format!(
        "p = FakePrinter(); gm = FakeGCodeMove(); gc = FakeGCode(); {objects} \
         gm.position = {}; gm.absolute_coord = {}; gm.absolute_extrude = {}; \
         cfg = FakeConfig(p, resolution={resolution_expr});",
        py_position_expr(position),
        if absolute_coordinates { "True" } else { "False" },
        if absolute_extrude { "True" } else { "False" },
    ))
    .expect("py setup");
}

fn py_load(py: &mut pyoracle::Oracle) -> pyoracle::OracleResult {
    py.exec("afs = gcode_arcs.load_config(cfg)")
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "gcode_arcs".to_string(),
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

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(_) => panic!("{label}: rust call succeeded but must fail"),
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
}

fn check_ok<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) -> T {
    match actual {
        Ok(v) => {
            match expected {
                Ok(_) => v,
                Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
            }
        }
        Err(e) => panic!("{label}: rust raised {}: {}", e.class_name(), e.message()),
    }
}

fn g1_value(calls: &[(String, Vec<(String, String)>)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(command, params)| {
                Value::Tuple(vec![
                    Value::str(command),
                    Value::Dict(
                        params
                            .iter()
                            .map(|(k, v)| {
                                let f = float_from_string(v).expect("float_str parses");
                                (Value::str(k), Value::float(f))
                            })
                            .collect(),
                    ),
                ])
            })
            .collect(),
    )
}

/// Runs one dispatch of `cmd` (with `params` as raw strings) on both sides
/// and checks either the generated G1 sequence or the raised error.
fn run_dispatch(
    py: &mut pyoracle::Oracle,
    label: &str,
    rig: &Rig,
    cmd: &str,
    params: &[(&str, &str)],
) {
    let created = rig.created.as_ref().expect("rig support");
    let commandline = format!(
        "{cmd} {}",
        params
            .iter()
            .map(|(k, v)| format!("{k}{v}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    let params_expr = format!(
        "{{{} }}",
        params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let py_method = cmd.to_uppercase();
    let expected = py.exec(&format!(
        "g = gcode_mod.GCodeCommand(gc, '{cmd}', '{commandline}', {params_expr}, False); \
         afs.cmd_{py_method}(g)"
    ));
    let gcmd: GCmd = Arc::new(FakeGCodeCommand::new(cmd, &commandline, params));
    let actual = match py_method.as_str() {
        "G2" => created.cmd_g2(&gcmd),
        "G3" => created.cmd_g3(&gcmd),
        "G17" => created.cmd_g17(&gcmd),
        "G18" => created.cmd_g18(&gcmd),
        "G19" => created.cmd_g19(&gcmd),
        _ => panic!("unknown command {cmd}"),
    };
    match expected {
        Ok(_) => {
            check_ok(label, Ok(Value::Null), actual);
            let g1_expected = py.eval("gm.g1").expect("eval g1");
            let g1_actual = g1_value(&rig.fake_gmove.g1.borrow());
            check(&format!("{label} g1"), Ok(g1_expected), g1_actual);
        }
        Err(exc) => {
            let rust_err = match actual {
                Ok(_) => panic!("{label}: rust call succeeded but must fail"),
                Err(e) => e,
            };
            assert_eq!(rust_err.class_name(), exc.class_name.as_str(), "{label}: class");
            assert_eq!(rust_err.message(), exc.message.as_str(), "{label}: message");
            let empty_py = py.eval("gm.g1").expect("eval g1");
            check(&format!("{label} empty g1"), Ok(empty_py), Value::List(Vec::new()));
            assert!(
                rig.fake_gmove.g1.borrow().is_empty(),
                "{label}: rust emitted G1s on error"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_constants_match_python() {
    let Some(mut py) = oracle() else { return };
    for (name, value) in [
        ("ARC_PLANE_X_Y", ARC_PLANE_X_Y),
        ("ARC_PLANE_X_Z", ARC_PLANE_X_Z),
        ("ARC_PLANE_Y_Z", ARC_PLANE_Y_Z),
        ("X_AXIS", 0),
        ("Y_AXIS", 1),
        ("Z_AXIS", 2),
        ("E_AXIS", 3),
    ] {
        let expected = py.eval(&format!("gcode_arcs.{name}")).expect("eval const");
        check(&format!("const {name}"), Ok(expected), Value::int(value as i64));
    }
}

#[test]
fn init_reads_resolution_once() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [0., 0., 0., 0.], true, true, true, true);
    let expected_load = py_load(&mut py);
    let r = rig(None, [0., 0., 0., 0.], true, true, true, true);
    check_ok("init default resolution", expected_load, created_clone(&r));
    let calls = r.fake_config.calls.borrow();
    let expected = py.eval("cfg.getfloat_calls");
    let actual = Value::List(
        calls
            .iter()
            .map(|(option, default, above)| {
                Value::Tuple(vec![
                    Value::str(option),
                    Value::float(default.expect("default recorded")),
                    Value::float(above.expect("above recorded")),
                ])
            })
            .collect(),
    );
    check("init getfloat calls", expected, actual);
}

#[test]
fn init_registers_five_commands_in_order() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [0., 0., 0., 0.], true, true, true, true);
    let expected_load = py_load(&mut py);
    let r = rig(None, [0., 0., 0., 0.], true, true, true, true);
    check_ok("init register", expected_load, created_clone(&r));
    let expected = py.eval("gc.registered").expect("eval registered");
    let actual = Value::List(
        r.fake_gcode
            .registered
            .borrow()
            .iter()
            .map(|name| Value::str(name))
            .collect(),
    );
    check("init registered order", Ok(expected), actual);
}

#[test]
fn init_missing_gcode_move_errors() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [0., 0., 0., 0.], true, true, false, true);
    let expected = py_load(&mut py);
    let r = rig(None, [0., 0., 0., 0.], true, true, false, true);
    check_err("missing gcode_move", expected, created_clone(&r));
}

#[test]
fn init_missing_gcode_errors() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [0., 0., 0., 0.], true, true, true, false);
    let expected = py_load(&mut py);
    let r = rig(None, [0., 0., 0., 0.], true, true, true, false);
    check_err("missing gcode", expected, created_clone(&r));
}

#[test]
fn init_duplicate_command_errors() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [0., 0., 0., 0.], true, true, true, true);
    py.exec("gc.register_command('G2', lambda gcmd: None)")
        .expect("pre-register G2");
    let expected = py_load(&mut py);
    let (fake_gmove, fake_gcode, _fake_config, config) =
        rig_parts(None, [0., 0., 0., 0.], true, true, true, true);
    let _ = fake_gmove;
    let pre = fake_gcode.register_command(
        "G2",
        Some(Box::new(|_gcmd: &GCmd| Ok(()))),
        RegisterCommandOpts::default(),
    );
    assert!(pre.is_ok());
    let created = ArcSupport::new(&config);
    check_err("duplicate G2", expected, created);
}

#[test]
fn init_resolution_above_zero_errors() {
    let Some(mut py) = oracle() else { return };
    for value in [0., -1., -0.5] {
        py_setup(&mut py, Some(value), [0., 0., 0., 0.], true, true, true, true);
        let expected = py_load(&mut py);
        let r = rig(Some(value), [0., 0., 0., 0.], true, true, true, true);
        check_err(&format!("resolution {value}"), expected, created_clone(&r));
    }
}

#[test]
fn init_resolution_error_wins_over_missing_objects() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, Some(0.), [0., 0., 0., 0.], true, true, true, true);
    let expected = py_load(&mut py);
    let r = rig(Some(0.), [0., 0., 0., 0.], true, true, false, true);
    check_err("resolution before load", expected, created_clone(&r));
}

#[test]
fn load_config_factory_builds_working_object() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::gcode_arcs::module_factory();
    let load_config = factory
        .load_config
        .expect("gcode_arcs exposes load_config");
    assert!(
        factory.load_config_prefix.is_none(),
        "gcode_arcs has no load_config_prefix"
    );
    assert!(
        factory.add_early_printer_objects.is_none(),
        "gcode_arcs has no add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "gcode_arcs has no add_printer_objects"
    );
    let (fake_gmove, fake_gcode, _fake_config, config) =
        rig_parts(None, [10., 0., 0., 0.], true, true, true, true);
    let obj: ObjectRef = load_config(&config)
        .expect("factory load_config")
        .expect("factory load_config returns an object");
    let support: Arc<ArcSupport> = downcast_arc(&obj).expect("downcast to ArcSupport");
    assert_eq!(support.current_plane(), ARC_PLANE_X_Y);
    // The factory-built object dispatches through the registered G2 handler.
    py_setup(&mut py, None, [10., 0., 0., 0.], true, true, true, true);
    let expected = py_load(&mut py);
    check_ok("factory load", expected, Ok(()));
    let expected = py.exec(
        "g = gcode_mod.GCodeCommand(gc, 'G2', 'G2 X0 Y10 I-10 J0', {'X': '0', 'Y': '10', 'I': '-10', 'J': '0'}, False); afs.cmd_G2(g)",
    );
    let gcmd: GCmd = Arc::new(FakeGCodeCommand::new(
        "G2",
        "G2 X0 Y10 I-10 J0",
        &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
    ));
    let result = {
        let mut handlers = fake_gcode.handlers.borrow_mut();
        let handler = handlers.get_mut("G2").expect("G2 handler registered");
        handler(&gcmd)
    };
    check_ok("factory registered G2", expected, result);
    let expected = py.eval("gm.g1").expect("eval g1");
    let actual = g1_value(&fake_gmove.g1.borrow());
    check("factory g1", Ok(expected), actual);
}

#[test]
fn arc_scenarios_match_oracle() {
    let Some(mut py) = oracle() else { return };
    struct Scenario {
        label: &'static str,
        resolution: Option<f64>,
        position: [f64; 4],
        absolute_coordinates: bool,
        absolute_extrude: bool,
        setup_cmds: &'static [&'static str],
        cmd: &'static str,
        params: &'static [(&'static str, &'static str)],
    }
    let scenarios = [
        Scenario {
            label: "xy clockwise quarter",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "xy counterclockwise quarter",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "xy full circle",
            resolution: None,
            position: [10., 0., 5., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "10"), ("Y", "0"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "xy helical",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("X", "0"), ("Y", "10"), ("Z", "5"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "xy default target params",
            resolution: None,
            position: [10., 0., 2., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("I", "-10")],
        },
        Scenario {
            label: "xz plane",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G18"],
            cmd: "G2",
            params: &[("X", "10"), ("Z", "10"), ("I", "-10"), ("K", "0")],
        },
        Scenario {
            label: "xz plane only IK",
            resolution: None,
            position: [10., 7., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G18"],
            cmd: "G3",
            params: &[("Z", "10"), ("I", "-10"), ("K", "0")],
        },
        Scenario {
            label: "yz plane",
            resolution: None,
            position: [0., 10., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G19"],
            cmd: "G2",
            params: &[("Y", "0"), ("Z", "10"), ("J", "-10"), ("K", "0")],
        },
        Scenario {
            label: "yz plane only JK",
            resolution: None,
            position: [3., 10., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G19"],
            cmd: "G3",
            params: &[("Z", "10"), ("J", "-10")],
        },
        Scenario {
            label: "plane reselect",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G18", "G19", "G17"],
            cmd: "G2",
            params: &[("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "extrude absolute",
            resolution: None,
            position: [10., 0., 0., 2.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("Y", "10"), ("I", "-10"), ("J", "0"), ("E", "7.5")],
        },
        Scenario {
            label: "extrude relative",
            resolution: None,
            position: [10., 0., 0., 2.],
            absolute_coordinates: true,
            absolute_extrude: false,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("Y", "10"), ("I", "-10"), ("J", "0"), ("E", "5.5")],
        },
        Scenario {
            label: "feedrate",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("Y", "10"), ("I", "-10"), ("J", "0"), ("F", "1500")],
        },
        Scenario {
            label: "extrude and feedrate",
            resolution: None,
            position: [10., 0., 0., 1.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("Y", "10"), ("I", "-10"), ("J", "0"), ("E", "4"), ("F", "900")],
        },
        Scenario {
            label: "tiny arc single segment",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "10.1"), ("Y", "0.1"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "fine resolution",
            resolution: Some(0.25),
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "coarse resolution",
            resolution: Some(5.),
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "relative mode rejected",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: false,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "R move rejected",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("R", "10")],
        },
        Scenario {
            label: "R zero rejected",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0"), ("R", "0")],
        },
        Scenario {
            label: "missing IJ rejected",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10")],
        },
        Scenario {
            label: "missing IK rejected in xz",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &["G18"],
            cmd: "G2",
            params: &[("X", "0"), ("Z", "10"), ("J", "0")],
        },
        Scenario {
            label: "unparsable X",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "abc"), ("Y", "10"), ("I", "-10")],
        },
        Scenario {
            label: "unparsable R",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("R", "abc")],
        },
        Scenario {
            label: "unparsable K ignored in xy",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0"), ("K", "abc")],
        },
        Scenario {
            label: "nan target",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "nan"), ("Y", "10"), ("I", "-10"), ("J", "0")],
        },
        Scenario {
            label: "zero I with J",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G3",
            params: &[("Y", "20"), ("I", "0"), ("J", "10")],
        },
        Scenario {
            label: "half circle cw",
            resolution: None,
            position: [10., 0., 0., 0.],
            absolute_coordinates: true,
            absolute_extrude: true,
            setup_cmds: &[],
            cmd: "G2",
            params: &[("X", "-10"), ("Y", "0"), ("I", "-10"), ("J", "0")],
        },
    ];
    for scenario in &scenarios {
        let label = scenario.label;
        py_setup(
            &mut py,
            scenario.resolution,
            scenario.position,
            scenario.absolute_coordinates,
            scenario.absolute_extrude,
            true,
            true,
        );
        let expected = py_load(&mut py);
        let r = rig(
            scenario.resolution,
            scenario.position,
            scenario.absolute_coordinates,
            scenario.absolute_extrude,
            true,
            true,
        );
        check_ok(&format!("{label} load"), expected, created_clone(&r));
        for setup_cmd in scenario.setup_cmds {
            let py_method = setup_cmd.to_uppercase();
            let expected = py.exec(&format!(
                "g = gcode_mod.GCodeCommand(gc, '{setup_cmd}', '{setup_cmd}', {{}}, False); \
                 afs.cmd_{py_method}(g)"
            ));
            let gcmd: GCmd = Arc::new(FakeGCodeCommand::new(setup_cmd, setup_cmd, &[]));
            let actual = match setup_cmd.to_uppercase().as_str() {
                "G17" => rig_cmd(&r, gcmd, "G17"),
                "G18" => rig_cmd(&r, gcmd, "G18"),
                "G19" => rig_cmd(&r, gcmd, "G19"),
                _ => panic!("unknown setup command {setup_cmd}"),
            };
            check_ok(&format!("{label} plane {setup_cmd}"), expected, actual);
        }
        run_dispatch(&mut py, label, &r, scenario.cmd, scenario.params);
    }
    // Plane value after the last reselect on the python side.
    let expected = py.eval("afs.plane").expect("eval plane");
    let r = rig(None, [10., 0., 0., 0.], true, true, true, true);
    let support = check_ok("plane rig", Ok(Value::Null), created_clone(&r));
    check(
        "plane final value",
        Ok(expected),
        Value::int(support.current_plane() as i64),
    );
}

#[test]
fn registered_handler_matches_direct_call() {
    let Some(mut py) = oracle() else { return };
    py_setup(&mut py, None, [10., 0., 0., 0.], true, true, true, true);
    let expected = py_load(&mut py);
    let r = rig(None, [10., 0., 0., 0.], true, true, true, true);
    let support = check_ok("handler load", expected, r.created);
    let expected = py.exec(
        "g = gcode_mod.GCodeCommand(gc, 'G3', 'G3 X0 Y10 I-10 J0', {'X': '0', 'Y': '10', 'I': '-10', 'J': '0'}, False); afs.cmd_G3(g)",
    );
    let gcmd: GCmd = Arc::new(FakeGCodeCommand::new(
        "G3",
        "G3 X0 Y10 I-10 J0",
        &[("X", "0"), ("Y", "10"), ("I", "-10"), ("J", "0")],
    ));
    let result = {
        let mut handlers = r.fake_gcode.handlers.borrow_mut();
        let handler = handlers.get_mut("G3").expect("G3 handler registered");
        handler(&gcmd)
    };
    check_ok("handler G3", expected, result);
    let expected = py.eval("gm.g1").expect("eval g1");
    let actual = g1_value(&r.fake_gmove.g1.borrow());
    check("handler g1", Ok(expected), actual);
    let _ = support;
}

fn rig_cmd(r: &Rig, gcmd: GCmd, cmd: &str) -> KResult<()> {
    let support = r.created.as_ref().expect("rig support");
    match cmd {
        "G17" => support.cmd_g17(&gcmd),
        "G18" => support.cmd_g18(&gcmd),
        "G19" => support.cmd_g19(&gcmd),
        _ => panic!("unknown command {cmd}"),
    }
}
