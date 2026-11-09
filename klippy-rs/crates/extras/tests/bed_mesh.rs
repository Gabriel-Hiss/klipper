//! Parity tests for `extras::bed_mesh` (port of `klippy/extras/bed_mesh.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): the Python `ZMesh`,
//! `ProbeManager` point generation, `MoveSplitter` and `parse_config_pair`
//! are built with minimal Python fakes, then fed the same inputs as the
//! Rust port and the result values (floats by bits via `assert_equal`)
//! plus the error class and exact message text are compared.
//!
//! The oracle-backed cases skip with a message when `$KLIPPY_PYTHON` is
//! unset (outside the oracle image), exactly like the canbus_ids tests.

use extras::bed_mesh::{
    constrain, isclose, lerp, parse_config_pair as rust_parse_config_pair, within, MoveSplitter,
    ProbeFinalize, ProbeManager, ZMesh,
};
use extras::manual_probe::ProbeResult;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcell::GCell;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetOpts, RegisterCommandOpts, GCmd,
};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import math, collections, configparser

class FakeReactor:
    def monotonic(self):
        return 100.0

class FakePrinter:
    config_error = configparser.Error
    command_error = Exception
    def __init__(self):
        self.objects = {}
        self.reactor = FakeReactor()
    def register_event_handler(self, *a, **k):
        pass
    def lookup_object(self, name, default=None):
        if name not in self.objects:
            return default
        return self.objects[name]
    def load_object(self, config, section, default=None):
        return default
    def get_reactor(self):
        return self.reactor

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, values=None):
        self._printer = printer
        self.values = values or {}
        self.name = "bed_mesh"
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.name
    def get(self, option, default=None):
        if option in self.values:
            return self.values[option]
        if default is None:
            raise configparser.Error("Option %s not found in section bed_mesh" % (option,))
        return default
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None):
        v = self.values.get(option)
        if v is None:
            if default is None:
                raise configparser.Error("Option %s not found in section bed_mesh" % (option,))
            return default
        v = float(v)
        if above is not None and v <= above:
            raise configparser.Error("Option %s must be above %s" % (option, above))
        if minval is not None and v < minval:
            raise configparser.Error("Option %s must be >= %s" % (option, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option %s must be <= %s" % (option, maxval))
        return v
    def getint(self, option, default=None, minval=None, maxval=None):
        v = self.values.get(option)
        if v is None:
            if default is None:
                raise configparser.Error("Option %s not found in section bed_mesh" % (option,))
            return default
        v = int(v)
        if minval is not None and v < minval:
            raise configparser.Error("Option %s must be >= %s" % (option, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option %s must be <= %s" % (option, maxval))
        return v
    def getfloatlist(self, option, default=None, count=None):
        v = self.values.get(option)
        if v is None:
            if default is None:
                raise configparser.Error("Option %s not found in section bed_mesh" % (option,))
            return default
        out = [float(x.strip()) for x in str(v).split(',')]
        if count is not None and len(out) != count:
            raise configparser.Error("Option %s requires %d values" % (option, count))
        return out
    def getintlist(self, option, default=None, count=None):
        v = self.values.get(option)
        if v is None:
            if default is None:
                raise configparser.Error("Option %s not found in section bed_mesh" % (option,))
            return default
        out = [int(x.strip()) for x in str(v).split(',')]
        if count is not None and len(out) != count:
            raise configparser.Error("Option %s requires %d values" % (option, count))
        return out
    def getlists(self, option, default=None, seps=None, parser=str, count=None):
        raise configparser.Error("unused")
    def getsection(self, section):
        return self
    def get_prefix_sections(self, prefix):
        return []
"#;

#[derive(Clone)]
struct FakePrinter {
    objects: std::sync::Arc<std::collections::BTreeMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> Self {
        let gcode: ObjectRef = Arc::new(FakeGCode {
            reactor: Reactor::new(false),
            registered: GCell::new(Vec::new()),
            handlers: GCell::new(Vec::new()),
            responded: GCell::new(Vec::new()),
        });
        let mut objects = std::collections::BTreeMap::new();
        objects.insert("gcode".to_string(), gcode);
        FakePrinter { objects: std::sync::Arc::new(objects) }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, _n: &str, _o: ObjectRef) -> KResult<()> { Ok(()) }
    fn lookup_object(&self, n: &str, d: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.get(n).cloned() {
            Some(o) => Ok(Some(o)),
            None => match d {
                LookupDefault::Raise => Err(klippy_api::error::KlippyError::config(format!("lookup_object: {n} not found"))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, ObjectRef)> { Vec::new() }
    fn load_object(&self, _c: &Config, _s: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> { Ok(None) }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _c: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _r: &str) {}
}

struct FakeGCode {
    reactor: Reactor,
    registered: GCell<Vec<(String, Option<String>)>>,
    handlers: GCell<Vec<(String, GCodeCommandHandler)>>,
    responded: GCell<Vec<String>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(&self, cmd: &str, func: Option<GCodeCommandHandler>, opts: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> {
        self.registered.borrow_mut().push((cmd.to_string(), opts.desc));
        if let Some(handler) = func {
            self.handlers.borrow_mut().push((cmd.to_string(), handler));
        }
        Ok(None)
    }
    fn register_mux_command(&self, _cmd: &str, _key: &str, _value: Option<&str>, _func: GCodeCommandHandler, _desc: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _eventtime: f64) -> PyValue { PyValue::None }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> reactor::ReactorMutex { self.reactor.mutex(false) }
    fn create_gcode_command(&self, command: &str, commandline: &str, params: &IndexMap<String, String>) -> GCmd {
        Arc::new(FakeGcmd { command: command.to_string(), commandline: commandline.to_string(), params: GCell::new(params.clone()) })
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, msg: &str, log: bool) {
        if log { klippy_api::log::info(msg); }
        self.responded.borrow_mut().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool { false }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: GCell<IndexMap<String, String>>,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, _opts: GetOpts) -> KResult<PyValue> {
        Ok(match self.params.borrow().get(name) {
            Some(v) => PyValue::Str(v.clone()),
            None => PyValue::None,
        })
    }
    fn get_int(&self, name: &str, opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        match self.params.borrow().get(name) {
            Some(v) => Ok(Some(v.parse().unwrap_or(0))),
            None => match opts.default {
                None => Err(klippy_api::error::KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_float(&self, name: &str, opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        match self.params.borrow().get(name) {
            Some(v) => Ok(Some(v.parse().unwrap_or(0.0))),
            None => match opts.default {
                None => Err(klippy_api::error::KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_command(&self) -> String { self.command.clone() }
    fn get_commandline(&self) -> String { self.commandline.clone() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.borrow().clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool { true }
}

struct FakeConfig {
    printer: Printer,
    values: std::collections::BTreeMap<String, String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { "bed_mesh".to_string() }
    fn get(&self, option: &str, opts: GetStrOpts) -> klippy_api::error::KResult<Option<String>> {
        if let Some(v) = self.values.get(option) { return Ok(Some(v.clone())); }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!("Option {option} not found in section bed_mesh"))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> klippy_api::error::KResult<Option<i64>> {
        if let Some(v) = self.values.get(option) { return Ok(Some(v.parse::<i64>().unwrap())); }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!("Option {option} not found in section bed_mesh"))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> klippy_api::error::KResult<Option<f64>> {
        if let Some(v) = self.values.get(option) { return Ok(Some(v.parse::<f64>().unwrap())); }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!("Option {option} not found in section bed_mesh"))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getboolean(&self, _option: &str, opts: GetBoolOpts) -> klippy_api::error::KResult<Option<bool>> {
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config("required")),
            Some(v) => Ok(v),
        }
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> klippy_api::error::KResult<PyValue> { Ok(PyValue::None) }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> klippy_api::error::KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> klippy_api::error::KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> klippy_api::error::KResult<Option<Vec<i64>>> {
        if let Some(v) = self.values.get(option) {
            return Ok(Some(v.split(',').map(|s| s.trim().parse::<i64>().unwrap()).collect()));
        }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!("Option {option} not found in section bed_mesh"))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> klippy_api::error::KResult<Option<Vec<f64>>> {
        if let Some(v) = self.values.get(option) {
            return Ok(Some(v.split(',').map(|s| s.trim().parse::<f64>().unwrap()).collect()));
        }
        match opts.default {
            None => Err(klippy_api::error::KlippyError::config(format!("Option {option} not found in section bed_mesh"))),
            Some(None) => Ok(None),
            Some(Some(v)) => Ok(Some(v)),
        }
    }
    fn getsection(&self, _s: &str) -> Config { panic!("unused") }
    fn has_section(&self, _s: &str) -> bool { false }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _p: &str) -> Vec<String> { Vec::new() }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) {}
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bed_mesh fakes");
            Some(py)
        }
        None => {
            println!("skipping bed_mesh oracle tests: set KLIPPY_PYTHON for CPython 3.8.2");
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "bed_mesh".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: klippy_api::error::KResult<()>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust succeeded but must fail"),
        Err(e) => e,
    };
    assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

fn matrix_value(m: &Vec<Vec<f64>>) -> Value {
    Value::List(
        m.iter()
            .map(|r| Value::List(r.iter().map(|v| Value::float(*v)).collect()))
            .collect(),
    )
}

fn points_value(pts: &Vec<(f64, f64)>) -> Value {
    Value::List(
        pts.iter()
            .map(|p| Value::Tuple(vec![Value::float(p.0), Value::float(p.1)]))
            .collect(),
    )
}

#[test]
fn helpers_match_python_semantics() {
    assert!(isclose(1.0, 1.0));
    assert!(isclose(0.0, 0.0));
    assert!(!isclose(1.0, 1.0001));
    assert!(within((5.0, 5.0), (0.0, 0.0), (10.0, 10.0), 0.0));
    assert!(!within((-1.0, 5.0), (0.0, 0.0), (10.0, 10.0), 0.0));
    assert!(within((-1.0, 5.0), (0.0, 0.0), (10.0, 10.0), 1.0));
    assert_eq!(constrain(5.0, 0.0, 10.0), 5.0);
    assert_eq!(constrain(-3.0, 0.0, 10.0), 0.0);
    assert_eq!(constrain(99.0, 0.0, 10.0), 10.0);
    assert!((lerp(0.0, 10.0, 20.0) - 10.0).abs() < 1e-12);
    assert!((lerp(0.5, 10.0, 20.0) - 15.0).abs() < 1e-12);
    assert!((lerp(1.0, 10.0, 20.0) - 20.0).abs() < 1e-12);
}

const BASE_MATRIX: [[f64; 3]; 3] = [[0.1, 0.2, 0.15], [0.25, 0.3, 0.2], [0.05, 0.12, 0.18]];

fn zmesh_params(algo: &str, pps: i64, tension: f64) -> indexmap::IndexMap<String, PyValue> {
    let mut p = indexmap::IndexMap::new();
    p.insert("min_x".into(), PyValue::Float(0.0));
    p.insert("max_x".into(), PyValue::Float(100.0));
    p.insert("min_y".into(), PyValue::Float(0.0));
    p.insert("max_y".into(), PyValue::Float(100.0));
    p.insert("x_count".into(), PyValue::Int(3i64.into()));
    p.insert("y_count".into(), PyValue::Int(3i64.into()));
    p.insert("mesh_x_pps".into(), PyValue::Int(pps.into()));
    p.insert("mesh_y_pps".into(), PyValue::Int(pps.into()));
    p.insert("algo".into(), PyValue::Str(algo.into()));
    p.insert("tension".into(), PyValue::Float(tension));
    p
}

fn base_matrix() -> Vec<Vec<f64>> {
    BASE_MATRIX.iter().map(|r| r.to_vec()).collect()
}

#[test]
fn zmesh_direct_calc_z_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec("import extras.bed_mesh as bm\nparams = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=3, y_count=3, mesh_x_pps=0, mesh_y_pps=0, algo='direct', tension=0.2)\nzm = bm.ZMesh(params, None)\nzm.build_mesh([[0.1, 0.2, 0.15],[0.25, 0.3, 0.2],[0.05, 0.12, 0.18]])\n")
        .expect("build");
    for (x, y) in [(0.0, 0.0), (50.0, 50.0), (100.0, 100.0), (25.0, 75.0), (200.0, 200.0), (-5.0, -5.0)] {
        let expected = py.eval(&format!("zm.calc_z({x}, {y})")).expect("oracle calc_z");
        let zm = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
        zm.build_mesh(base_matrix()).unwrap();
        check(&format!("direct calc_z {x},{y}"), Ok(expected), Value::float(zm.calc_z(x, y)));
    }
    let expected = py.eval("zm.get_probed_matrix()").expect("oracle probed");
    let zm = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    zm.build_mesh(base_matrix()).unwrap();
    check("direct probed_matrix", Ok(expected), matrix_value(&zm.get_probed_matrix()));
}

#[test]
fn zmesh_lagrange_and_bicubic_match_oracle() {
    let Some(mut py) = oracle() else { return };
    let matrix = vec![vec![0.1,0.2,0.15,0.22], vec![0.25,0.3,0.2,0.11], vec![0.05,0.12,0.18,0.19], vec![0.21,0.09,0.14,0.17]];
    py.exec("import extras.bed_mesh as bm\nparams = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=4, y_count=4, mesh_x_pps=2, mesh_y_pps=2, algo='lagrange', tension=0.2)\nzm = bm.ZMesh(params, None)\nzm.build_mesh([[0.1,0.2,0.15,0.22],[0.25,0.3,0.2,0.11],[0.05,0.12,0.18,0.19],[0.21,0.09,0.14,0.17]])\n")
        .expect("lagrange build");
    let expected = py.eval("zm.get_mesh_matrix()").expect("oracle mesh");
    let mut p = zmesh_params("lagrange", 2, 0.2);
    p.insert("x_count".into(), PyValue::Int(4i64.into()));
    p.insert("y_count".into(), PyValue::Int(4i64.into()));
    let zm = ZMesh::new(p, None);
    zm.build_mesh(matrix.clone()).unwrap();
    check("lagrange mesh_matrix", Ok(expected), matrix_value(&zm.get_mesh_matrix()));

    py.exec("params = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=4, y_count=4, mesh_x_pps=2, mesh_y_pps=2, algo='bicubic', tension=0.2)\nzm2 = bm.ZMesh(params, None)\nzm2.build_mesh([[0.1,0.2,0.15,0.22],[0.25,0.3,0.2,0.11],[0.05,0.12,0.18,0.19],[0.21,0.09,0.14,0.17]])\n")
        .expect("bicubic build");
    let expected = py.eval("zm2.get_mesh_matrix()").expect("oracle mesh2");
    let mut p2 = zmesh_params("bicubic", 2, 0.2);
    p2.insert("x_count".into(), PyValue::Int(4i64.into()));
    p2.insert("y_count".into(), PyValue::Int(4i64.into()));
    let zm2 = ZMesh::new(p2, None);
    zm2.build_mesh(matrix).unwrap();
    check("bicubic mesh_matrix", Ok(expected), matrix_value(&zm2.get_mesh_matrix()));
    let expected = py.eval("zm2.calc_z(33.3, 66.6)").expect("oracle calc");
    check("bicubic calc_z", Ok(expected), Value::float(zm2.calc_z(33.3, 66.6)));
}

#[test]
fn zmesh_no_mesh_zero_and_profile_name() {
    let zm = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    assert_eq!(zm.calc_z(10.0, 10.0), 0.0);
    assert_eq!(zm.get_z_average(), 0.0);
    assert_eq!(zm.get_z_range(), (0.0, 0.0));
    assert_eq!(zm.get_mesh_matrix(), vec![vec![]]);
    assert_eq!(zm.get_probed_matrix(), vec![vec![]]);
    assert!(zm.get_profile_name().starts_with("adaptive-"));
    let zm2 = ZMesh::new(zmesh_params("direct", 0, 0.2), Some("mine"));
    assert_eq!(zm2.get_profile_name(), "mine");
}

#[test]
fn zmesh_set_zero_reference_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec("import extras.bed_mesh as bm\nparams = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=3, y_count=3, mesh_x_pps=0, mesh_y_pps=0, algo='direct', tension=0.2)\nzm = bm.ZMesh(params, None)\nzm.build_mesh([[0.1,0.2,0.15],[0.25,0.3,0.2],[0.05,0.12,0.18]])\nzm.set_zero_reference(50, 50)\n")
        .expect("build+ref");
    let expected = py.eval("zm.get_probed_matrix()").expect("oracle");
    let zm = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    zm.build_mesh(base_matrix()).unwrap();
    zm.set_zero_reference(50.0, 50.0);
    check("set_zero_reference probed", Ok(expected), matrix_value(&zm.get_probed_matrix()));
}

fn config_with(values: Vec<(&str, &str)>) -> Config {
    let printer: Printer = Arc::new(FakePrinter::new());
    let mut map = std::collections::BTreeMap::new();
    for (k, v) in values { map.insert(k.to_string(), v.to_string()); }
    Arc::new(FakeConfig { printer, values: map })
}

#[test]
fn probe_manager_generate_points_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        r#"
import extras.bed_mesh as bm
import configparser
class Cfg:
    error = configparser.Error
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None):
        if default is None:
            raise configparser.Error("required")
        return default
    def getfloatlist(self, option, default=None, count=None):
        return default
class Helper:
    def __init__(self):
        self.pts = []
    def use_xy_offsets(self, u):
        pass
    def update_probe_points(self, points, min_points):
        self.pts = [list(p) for p in points]
    def start_probe(self, gcmd):
        pass
cfg = Cfg(FakePrinter())
orig = dict(radius=None, origin=None, mesh_min=(0.0, 0.0), mesh_max=(100.0, 100.0))
mc = dict(x_count=4, y_count=4, mesh_x_pps=2, mesh_y_pps=2, algo='lagrange', tension=0.2)
orig2 = dict(orig)
orig2['mesh_min'] = (0.0, 0.0)
orig2['mesh_max'] = (100.0, 100.0)
# ProbeManager.__init__ needs probe.ProbePointsHelper; stub it by pre-patching class
import extras.probe as probe_mod
class FakeHelper:
    def __init__(self, config, finalize_callback, default_points=None):
        self.printer = config.get_printer()
        self.finalize_callback = finalize_callback
        self.pts = []
    def use_xy_offsets(self, u):
        pass
    def update_probe_points(self, points, min_points):
        self.pts = [list(p) for p in points]
    def start_probe(self, gcmd):
        pass
probe_mod.ProbePointsHelper = FakeHelper
pm = bm.ProbeManager(cfg, orig, lambda pts: None)
pm.probe_helper = FakeHelper(cfg, lambda pts: None, None)
pm.generate_points(mc, (0.0, 0.0), (100.0, 100.0), None, None)
"#
    )
    .expect("py ProbeManager");
    let expected = py.eval("pm.base_points").expect("oracle points");

    let cfg = config_with(vec![("scan_overshoot", "0")]);
    let mut orig: indexmap::IndexMap<String, PyValue> = indexmap::IndexMap::new();
    orig.insert("radius".into(), PyValue::None);
    orig.insert("origin".into(), PyValue::None);
    orig.insert("mesh_min".into(), PyValue::Tuple(vec![PyValue::Float(0.0), PyValue::Float(0.0)]));
    orig.insert("mesh_max".into(), PyValue::Tuple(vec![PyValue::Float(100.0), PyValue::Float(100.0)]));
    let mut mc: indexmap::IndexMap<String, PyValue> = indexmap::IndexMap::new();
    mc.insert("x_count".into(), PyValue::Int(4i64.into()));
    mc.insert("y_count".into(), PyValue::Int(4i64.into()));
    mc.insert("mesh_x_pps".into(), PyValue::Int(2i64.into()));
    mc.insert("mesh_y_pps".into(), PyValue::Int(2i64.into()));
    mc.insert("algo".into(), PyValue::Str("lagrange".into()));
    mc.insert("tension".into(), PyValue::Float(0.2));
    let dummy: ProbeFinalize = Arc::new(|_: &[ProbeResult]| Ok(()));
    let pm = ProbeManager::new(&cfg, orig, dummy).unwrap();
    pm.generate_points(&mc, (0.0, 0.0), (100.0, 100.0), None, None, "automatic").unwrap();
    check("generate_points base_points", Ok(expected), points_value(&pm.get_base_points()));
}

#[test]
fn move_splitter_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        r#"
import extras.bed_mesh as bm
class Cfg:
    error = configparser.Error
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None):
        return default
params = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=3, y_count=3, mesh_x_pps=0, mesh_y_pps=0, algo='direct', tension=0.2)
zm = bm.ZMesh(params, None)
zm.build_mesh([[0.0, 0.5, 1.0],[0.0, 0.5, 1.0],[0.0, 0.5, 1.0]])
cfg = Cfg()
ms = bm.MoveSplitter(cfg, None)
ms.initialize(zm, 0.25)
ms.build_move((0.0, 0.0, 0.0, 0.0), (100.0, 0.0, 0.0, 0.0), 1.0)
splits = []
while not ms.traverse_complete:
    sm = ms.split()
    splits.append(list(sm) if sm else None)
"#,
    )
    .expect("splitter setup");
    let expected = py.eval("splits").expect("oracle splits");

    let cfg = config_with(vec![("split_delta_z", "0.025"), ("move_check_distance", "5")]);
    let ms = MoveSplitter::new(&cfg).unwrap();
    let zm = Arc::new(ZMesh::new(zmesh_params("direct", 0, 0.2), None));
    zm.build_mesh(vec![vec![0.0, 0.5, 1.0], vec![0.0, 0.5, 1.0], vec![0.0, 0.5, 1.0]]).unwrap();
    ms.initialize(Some(zm), 0.25);
    ms.build_move(vec![0.0, 0.0, 0.0, 0.0], vec![100.0, 0.0, 0.0, 0.0], 1.0).unwrap();
    let mut got: Vec<Vec<f64>> = Vec::new();
    loop {
        match ms.split().unwrap() {
            Some(p) => got.push(p),
            None => break,
        }
        if ms.traverse_done() { break; }
    }
    let got_value = Value::List(got.iter().map(|r| Value::List(r.iter().map(|v| Value::float(*v)).collect())).collect());
    check("move_splitter splits", Ok(expected), got_value);
}

#[test]
fn parse_config_pair_errors_match_oracle() {
    let Some(mut py) = oracle() else { return };
    // malformed length (3 values)
    let cfg = config_with(vec![("probe_count", "3,4,5")]);
    let rust = rust_parse_config_pair(&cfg, "probe_count", 3, Some(3), None).map(|_| ());
    py.exec("import extras.bed_mesh as bm\nc = FakeConfig(FakePrinter(), {'probe_count': '3,4,5'})")
        .expect("cfg");
    let exp = py.eval("bm.parse_config_pair(c, 'probe_count', 3, 3)");
    check_err("parse_config_pair malformed", exp, rust);
    // minval breach
    let cfg = config_with(vec![("probe_count", "2")]);
    let rust = rust_parse_config_pair(&cfg, "probe_count", 3, Some(3), None).map(|_| ());
    py.exec("c = FakeConfig(FakePrinter(), {'probe_count': '2'})").expect("cfg2");
    let exp = py.eval("bm.parse_config_pair(c, 'probe_count', 3, 3)");
    check_err("parse_config_pair minval", exp, rust);
    // single value duplicates
    let cfg = config_with(vec![("probe_count", "4")]);
    let (a, b) = rust_parse_config_pair(&cfg, "probe_count", 3, Some(3), None).unwrap();
    py.exec("c = FakeConfig(FakePrinter(), {'probe_count': '4'})").expect("cfg3");
    let exp = py.eval("bm.parse_config_pair(c, 'probe_count', 3, 3)");
    check("parse_config_pair single", exp, Value::Tuple(vec![Value::Int(a), Value::Int(b)]));
}
