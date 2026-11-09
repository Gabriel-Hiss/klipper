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
    constrain, isclose, lerp, parse_config_pair as rust_parse_config_pair, within, BedMesh,
    BedMeshCalibrate, MoveSplitter, ProbeFinalize, ProbeManager, ZMesh,
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
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetOpts, MoveTransform,
    RegisterCommandOpts, GCmd,
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
        objects.insert("gcode_move".to_string(), Arc::new(FakeGCodeMove) as ObjectRef);
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
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, Option<ObjectRef>)> { Vec::new() }
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
        Arc::new(FakeGcmd { command: command.to_string(), commandline: commandline.to_string(), params: GCell::new(params.clone()), out: GCell::new(Vec::new()) })
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

/// `gcode_move` stand-in: `BedMesh.set_mesh` only calls `reset_last_position`.
struct FakeGCodeMove;

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn set_move_transform(&self, _transform: Option<MoveTransform>, _force: bool) -> KResult<Option<MoveTransform>> { Ok(None) }
    fn reset_last_position(&self) -> KResult<()> { Ok(()) }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeGcmd {
    command: String,
    commandline: String,
    params: GCell<IndexMap<String, String>>,
    /// Every response as `(kind, msg, log)`, like the Python fake's `out`.
    out: GCell<Vec<(String, String, bool)>>,
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
    fn respond_info(&self, msg: &str, log: bool) {
        self.out.borrow_mut().push(("info".to_string(), msg.to_string(), log));
    }
    fn respond_raw(&self, msg: &str) {
        self.out.borrow_mut().push(("raw".to_string(), msg.to_string(), false));
    }
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
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
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
    let mut py = oracle().expect("oracle");
    py.exec("import extras.bed_mesh as bm").expect("import bed_mesh");
    for (a, b) in [(1.0, 1.0), (0.0, 0.0), (1.0, 1.0001)] {
        check(&format!("isclose({a}, {b})"), py.eval(&format!("bm.isclose({a:?}, {b:?})")), Value::Bool(isclose(a, b)));
    }
    for (coord, tol) in [((5.0, 5.0), 0.0), ((-1.0, 5.0), 0.0), ((-1.0, 5.0), 1.0)] {
        check(
            &format!("within({coord:?}, tol={tol})"),
            py.eval(&format!("bm.within({coord:?}, (0.0, 0.0), (10.0, 10.0), {tol:?})")),
            Value::Bool(within(coord, (0.0, 0.0), (10.0, 10.0), tol)),
        );
    }
    for val in [5.0, -3.0, 99.0] {
        check(
            &format!("constrain({val})"),
            py.eval(&format!("bm.constrain({val:?}, 0.0, 10.0)")),
            Value::float(constrain(val, 0.0, 10.0)),
        );
    }
    for t in [0.0, 0.5, 1.0] {
        check(&format!("lerp({t})"), py.eval(&format!("bm.lerp({t:?}, 10.0, 20.0)")), Value::float(lerp(t, 10.0, 20.0)));
    }
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

/// The `params` dict of `zmesh_params(algo, pps, tension)` as Python source.
fn py_zmesh_params(algo: &str, pps: i64, tension: f64) -> String {
    format!(
        "dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=3, y_count=3, \
         mesh_x_pps={pps}, mesh_y_pps={pps}, algo={algo:?}, tension={tension:?})"
    )
}

/// `"adaptive-%X" % (id(self),)` split into the literal prefix and the hex digits.
fn split_adaptive_name(name: &str) -> (String, String) {
    let digits = name.len() - name.trim_end_matches(|c: char| c.is_ascii_hexdigit()).len();
    let (prefix, hex) = name.split_at(name.len() - digits);
    (prefix.to_string(), hex.to_string())
}

#[test]
fn zmesh_no_mesh_zero_and_profile_name() {
    let mut py = oracle().expect("oracle");
    py.exec(&format!("import extras.bed_mesh as bm\nzm = bm.ZMesh({}, None)", py_zmesh_params("direct", 0, 0.2)))
        .expect("python ZMesh");
    let zm = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    check("calc_z without mesh", py.eval("zm.calc_z(10.0, 10.0)"), Value::float(zm.calc_z(10.0, 10.0)));
    check("get_z_average without mesh", py.eval("zm.get_z_average()"), Value::float(zm.get_z_average()));
    let (lo, hi) = zm.get_z_range();
    check("get_z_range without mesh", py.eval("zm.get_z_range()"), Value::Tuple(vec![Value::float(lo), Value::float(hi)]));
    check("get_mesh_matrix without mesh", py.eval("zm.get_mesh_matrix()"), matrix_value(&zm.get_mesh_matrix()));
    check("get_probed_matrix without mesh", py.eval("zm.get_probed_matrix()"), matrix_value(&zm.get_probed_matrix()));
    // The generated name embeds the object id: compare the literal prefix.
    let py_name = match py.eval("zm.get_profile_name()") {
        Ok(Value::Str(s)) => s,
        other => panic!("python profile name: {other:?}"),
    };
    assert_eq!(split_adaptive_name(&zm.get_profile_name()).0, split_adaptive_name(&py_name).0, "generated name prefix");
    py.exec(&format!("zm2 = bm.ZMesh({}, 'mine')", py_zmesh_params("direct", 0, 0.2))).expect("python named ZMesh");
    let zm2 = ZMesh::new(zmesh_params("direct", 0, 0.2), Some("mine"));
    check("named profile", py.eval("zm2.get_profile_name()"), Value::str(&zm2.get_profile_name()));
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

/// Python fakes for building a real `BedMeshCalibrate`: config getters that
/// return the default for any missing option (the Rust fake's `Some(None)`
/// behavior), a probe-points helper stub, and a recording gcmd.
const CALIBRATE_SETUP: &str = r#"
import types
import extras.bed_mesh as bm
import extras.probe as probe_mod
class StubPointsHelper:
    def __init__(self, config, finalize_callback, default_points=None):
        pass
    def use_xy_offsets(self, use_offsets):
        pass
    def update_probe_points(self, points, min_points):
        pass
probe_mod.ProbePointsHelper = StubPointsHelper
class StubGCode:
    def register_command(self, *args, **kwargs):
        pass
class DefaultsConfig(FakeConfig):
    def get(self, option, default=None):
        return self.values.get(option, default)
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None):
        v = self.values.get(option)
        return default if v is None else float(v)
    def getint(self, option, default=None, minval=None, maxval=None):
        v = self.values.get(option)
        return default if v is None else int(v)
    def getfloatlist(self, option, default=None, count=None):
        v = self.values.get(option)
        return default if v is None else [float(x) for x in v.split(',')]
    def getintlist(self, option, default=None, count=None):
        v = self.values.get(option)
        return default if v is None else [int(x) for x in v.split(',')]
class RecGcmd:
    error = Exception
    def __init__(self, params):
        self.params = params
        self.out = []
    def get_command_parameters(self):
        return dict(self.params)
    def get(self, name, default=None):
        return self.params.get(name, default)
    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self.params[name]) if name in self.params else default
    def get_float(self, name, default=None, minval=None, maxval=None, above=None, below=None):
        return float(self.params[name]) if name in self.params else default
    def respond_info(self, msg, log=True):
        self.out.append(('info', msg, log))
    def respond_raw(self, msg):
        self.out.append(('raw', msg, False))
printer = FakePrinter()
printer.objects['gcode'] = StubGCode()
cfg = DefaultsConfig(printer, {'mesh_min': '10,10', 'mesh_max': '210,210', 'probe_count': '5,5'})
"#;

const CALIBRATE_VALUES: [(&str, &str); 3] = [("mesh_min", "10,10"), ("mesh_max", "210,210"), ("probe_count", "5,5")];

fn rec_gcmd(params: &[(&str, &str)]) -> Arc<FakeGcmd> {
    Arc::new(FakeGcmd {
        command: "BED_MESH_OUTPUT".to_string(),
        commandline: "BED_MESH_OUTPUT".to_string(),
        params: GCell::new(params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()),
        out: GCell::new(Vec::new()),
    })
}

fn rec_out(g: &FakeGcmd) -> Value {
    Value::List(
        g.out
            .borrow()
            .iter()
            .map(|(kind, msg, log)| Value::Tuple(vec![Value::Str(kind.clone()), Value::Str(msg.clone()), Value::Bool(*log)]))
            .collect(),
    )
}

fn py_params(params: &[(&str, &str)]) -> String {
    let items: Vec<String> = params.iter().map(|(k, v)| format!("{k:?}: {v:?}")).collect();
    format!("{{{}}}", items.join(", "))
}

/// `BedMeshCalibrate.update_config` resets `mesh_config` from `orig_config`
/// key by key; the port used to keep a `RefCell` borrow of `mesh_config`
/// alive across that loop and panicked ("already borrowed") on the first
/// call. Each call is followed by the generated points, compared with the
/// Python object fed the same commands.
#[test]
fn calibrate_update_config_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(CALIBRATE_SETUP).expect("calibrate fakes");
    py.exec("bmc = bm.BedMeshCalibrate(cfg, None)").expect("py BedMeshCalibrate");
    let bmc = BedMeshCalibrate::new(&config_with(CALIBRATE_VALUES.to_vec())).unwrap();
    let steps: [&[(&str, &str)]; 4] = [
        &[("PROBE_COUNT", "4,3"), ("MESH_MIN", "20,15")],
        &[],
        &[("MESH_PPS", "3,1"), ("ALGORITHM", " Bicubic ")],
        &[("MESH_MAX", "200,180")],
    ];
    for (n, params) in steps.iter().enumerate() {
        py.exec(&format!(
            "g = RecGcmd({})\nbmc.update_config(g)\npts = []\nbmc.print_generated_points(pts.append)",
            py_params(params)
        ))
        .expect("py update_config");
        let expected = py.eval("pts").expect("py points");
        let g = rec_gcmd(params);
        bmc.update_config(&(g.clone() as GCmd)).unwrap();
        let pts = std::cell::RefCell::new(Vec::new());
        bmc.print_generated_points(&|m: &str| pts.borrow_mut().push(Value::Str(m.to_string())), false);
        check(&format!("update_config step {n}"), Ok(expected), Value::List(pts.into_inner()));
    }
}

/// `BED_MESH_OUTPUT` answers through `gcmd.respond_info`, which logs by
/// default: the probed matrix and the PGP point listing (any nonzero PGP)
/// must reach the log like Python's, and the mesh goes out raw.
#[test]
fn bed_mesh_output_responses_match_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(CALIBRATE_SETUP).expect("calibrate fakes");
    py.exec(
        "params = dict(min_x=0.0, max_x=100.0, min_y=0.0, max_y=100.0, x_count=3, y_count=3, mesh_x_pps=2, mesh_y_pps=2, algo='lagrange', tension=0.2)\n\
         ns = types.SimpleNamespace(z_mesh=None, bmc=bm.BedMeshCalibrate(cfg, None), horizontal_move_z=5.0)\n\
         def output(p):\n    g = RecGcmd(p)\n    bm.BedMesh.cmd_BED_MESH_OUTPUT(ns, g)\n    return g.out\n",
    )
    .expect("py BED_MESH_OUTPUT setup");
    let bmesh = BedMesh::new(&config_with(CALIBRATE_VALUES.to_vec())).unwrap();
    let run = |params: &[(&str, &str)], label: &str, py: &mut pyoracle::Oracle| {
        let expected = py.eval(&format!("output({})", py_params(params))).expect("py output");
        let g = rec_gcmd(params);
        bmesh.cmd_bed_mesh_output(&(g.clone() as GCmd)).unwrap();
        check(label, Ok(expected), rec_out(&g));
    };
    run(&[], "no mesh", &mut py);
    py.exec("zm = bm.ZMesh(params, 'default')\nzm.build_mesh([[0.1, 0.2, 0.15], [0.25, 0.3, 0.2], [0.05, 0.12, 0.18]])\nns.z_mesh = zm")
        .expect("py mesh");
    let zm = ZMesh::new(zmesh_params("lagrange", 2, 0.2), Some("default"));
    zm.build_mesh(base_matrix()).unwrap();
    bmesh.set_mesh(Some(Arc::new(zm))).unwrap();
    run(&[], "probed mesh", &mut py);
    run(&[("PGP", "1")], "PGP=1", &mut py);
    run(&[("PGP", "2")], "PGP=2", &mut py);
}

/// Python side of the adaptive-mesh runs: a fresh printer per case with an
/// optional `exclude_object` whose status lists the JSON-decoded objects,
/// then `dump_calibration(gcmd)` (which runs `update_config`, hence
/// `set_adaptive_mesh`). `mesh_min`/`mesh_max`/`origin` become tuples: the
/// rectangular adaptive path stores lists, which only Python code can tell
/// apart (webhooks serialize both as JSON arrays).
const ADAPTIVE_SETUP: &str = r#"
import json
class CommandError(Exception):
    pass
class AdaptiveGcmd(RecGcmd):
    error = CommandError
class FakeExclude:
    def __init__(self, objects):
        self.objects = objects
    def get_status(self, eventtime=None):
        return {"objects": self.objects, "excluded_objects": [], "current_object": None}
def adaptive_run(values, objects_json, params):
    p = FakePrinter()
    p.objects['gcode'] = StubGCode()
    if objects_json is not None:
        p.objects['exclude_object'] = FakeExclude(json.loads(objects_json))
    bmc = bm.BedMeshCalibrate(DefaultsConfig(p, values), None)
    g = AdaptiveGcmd(params)
    try:
        dump = bmc.dump_calibration(g)
    except Exception as e:
        return ('err', g.out, type(e).__name__, str(e))
    cfg = dict(dump['config'])
    for k in ('mesh_min', 'mesh_max', 'origin'):
        if isinstance(cfg[k], list):
            cfg[k] = tuple(cfg[k])
    return ('ok', g.out, cfg, dump['points'])
"#;

/// `exclude_object` stand-in whose status carries a fixed object list.
struct FakeExclude {
    objects: PyValue,
}

impl klippy_api::PrinterObject for FakeExclude {
    klippy_api::printer_object_base!();
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let mut status = IndexMap::new();
        status.insert(PyValue::Str("objects".into()), self.objects.clone());
        status.insert(PyValue::Str("excluded_objects".into()), PyValue::List(Vec::new()));
        status.insert(PyValue::Str("current_object".into()), PyValue::None);
        Ok(Some(PyValue::Dict(status)))
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().expect("int fits in i64")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect()),
        other => panic!("unexpected value {other:?}"),
    }
}

/// `pycompat::json::loads` rejects the `NaN` token CPython's `json.loads`
/// accepts; the Rust side parses this stand-in number and swaps it back.
const NAN_STANDIN: &str = "-1.25e-300";

fn with_nan(v: PyValue) -> PyValue {
    match v {
        PyValue::Float(f) if f.to_bits() == (-1.25e-300f64).to_bits() => PyValue::Float(f64::NAN),
        PyValue::List(l) => PyValue::List(l.into_iter().map(with_nan).collect()),
        PyValue::Dict(d) => PyValue::Dict(d.into_iter().map(|(k, v)| (k, with_nan(v))).collect()),
        other => other,
    }
}

/// `key=value` pairs (config options or command parameters).
type Params<'a> = &'a [(&'a str, &'a str)];

/// The Rust run mirroring `adaptive_run`.
fn rust_adaptive_run(values: &[(&str, &str)], objects_json: Option<&str>, params: &[(&str, &str)]) -> Value {
    let mut printer = FakePrinter::new();
    if let Some(json) = objects_json {
        let objects = with_nan(pycompat::json::loads(&json.replace("NaN", NAN_STANDIN)).expect("objects json"));
        let mut objs = (*printer.objects).clone();
        objs.insert("exclude_object".to_string(), Arc::new(FakeExclude { objects }) as ObjectRef);
        printer.objects = Arc::new(objs);
    }
    let printer: Printer = Arc::new(printer);
    let config: Config = Arc::new(FakeConfig {
        printer,
        values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
    });
    let bmc = BedMeshCalibrate::new(&config).unwrap();
    let g = rec_gcmd(params);
    match bmc.dump_calibration(Some(&(g.clone() as GCmd))) {
        Ok(PyValue::Dict(dump)) => {
            let get = |k: &str| dump.get(&PyValue::Str(k.into())).expect("dump key");
            Value::Tuple(vec![Value::str("ok"), rec_out(&g), pyvalue_to_value(get("config")), pyvalue_to_value(get("points"))])
        }
        Ok(other) => panic!("dump_calibration returned {other:?}"),
        Err(e) => Value::Tuple(vec![Value::str("err"), rec_out(&g), Value::str(e.class_name()), Value::str(e.message())]),
    }
}

/// `BED_MESH_CALIBRATE ADAPTIVE=1` (`set_adaptive_mesh`, bed_mesh.py:457-566),
/// as the KE's `START_PRINT` runs it: the responses ("Found N objects", the
/// bounds as tuples and lists, ratio, probe counts), the resulting mesh
/// configuration and probe points, and the exceptions malformed objects raise.
#[test]
fn adaptive_mesh_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(CALIBRATE_SETUP).expect("calibrate fakes");
    py.exec(ADAPTIVE_SETUP).expect("adaptive fakes");
    let ke: &[(&str, &str)] = &[("mesh_min", "10,10"), ("mesh_max", "210,210"), ("probe_count", "5,5"), ("adaptive_margin", "0")];
    let ke_margin: &[(&str, &str)] = &[("mesh_min", "10,10"), ("mesh_max", "210,210"), ("probe_count", "5,5"), ("adaptive_margin", "7.5")];
    let bicubic: &[(&str, &str)] = &[("mesh_min", "0,0"), ("mesh_max", "200,200"), ("probe_count", "9,9"), ("algorithm", "bicubic")];
    let round: &[(&str, &str)] = &[("mesh_radius", "100"), ("round_probe_count", "7"), ("algorithm", "bicubic")];
    let one = r#"[{"name": "PART_1", "center": [110.0, 110.0], "polygon": [[100.0, 100.0], [120.0, 100.0], [120.0, 120.0], [100.0, 120.0]]}]"#;
    let several = r#"[{"name": "A", "polygon": [[30, 40.5], [60.25, 40.5], [60.25, 90]]},
        {"name": "B", "polygon": [[150.125, 120], [180, 160.75]]},
        {"name": "C", "polygon": [[95, 15.5], [105.5, 25]]}]"#;
    let edge = r#"[{"name": "EDGE", "polygon": [[5, 3], [208, 212.5]]}]"#;
    let narrow_x = r#"[{"name": "STRIP", "polygon": [[90, 0], [100, 200]]}]"#;
    let wide = r#"[{"name": "WIDE", "polygon": [[20, 30], [180, 150]]}]"#;
    let centered = r#"[{"name": "MID", "polygon": [[-20, -10], [25, 15]]}]"#;
    let offcenter = r#"[{"name": "FAR", "polygon": [[40, 40], [95, 95]]}]"#;
    let tiny = r#"[{"name": "DOT", "polygon": [[110, 110], [110.5, 110.25]]}]"#;
    let no_polygon = r#"[{"name": "A", "polygon": [[10, 10], [20, 20]]}, {"name": "B"}]"#;
    let empty_polygons = r#"[{"name": "A", "polygon": []}]"#;
    let short_point = r#"[{"name": "A", "polygon": [[10, 10], [20]]}]"#;
    let nan_point = r#"[{"name": "A", "polygon": [[NaN, 100], [120, 120]]}]"#;
    let adaptive: &[(&str, &str)] = &[("ADAPTIVE", "1")];
    let cases: Vec<(&str, Params, Option<&str>, Params)> = vec![
        ("no exclude_object", ke, None, adaptive),
        ("zero objects", ke, Some("[]"), adaptive),
        ("ADAPTIVE=0", ke, Some(one), &[("ADAPTIVE", "0")]),
        ("one object", ke, Some(one), adaptive),
        ("ADAPTIVE=2 is truthy", ke, Some(one), &[("ADAPTIVE", "2")]),
        ("several objects", ke, Some(several), adaptive),
        ("config margin", ke_margin, Some(several), adaptive),
        ("ADAPTIVE_MARGIN override", ke_margin, Some(several), &[("ADAPTIVE", "1"), ("ADAPTIVE_MARGIN", "12.25")]),
        ("margin clamped to mesh bounds", ke, Some(edge), &[("ADAPTIVE", "1"), ("ADAPTIVE_MARGIN", "50")]),
        ("tiny area keeps minimum size", ke, Some(tiny), adaptive),
        ("with PROBE_COUNT and MESH_MIN", ke, Some(several), &[("ADAPTIVE", "1"), ("PROBE_COUNT", "6,4"), ("MESH_MIN", "20,20")]),
        ("min 4 probes when the other axis exceeds 6", bicubic, Some(narrow_x), adaptive),
        ("no clamp above 6 with 4+ on both", bicubic, Some(wide), adaptive),
        ("min 3 probes", bicubic, Some(tiny), adaptive),
        ("round bed adapts", round, Some(centered), adaptive),
        ("round bed keeps full mesh", round, Some(offcenter), adaptive),
        ("object without polygon", ke, Some(no_polygon), adaptive),
        ("empty polygons", ke, Some(empty_polygons), adaptive),
        ("short point", ke, Some(short_point), adaptive),
        ("NaN coordinate", ke, Some(nan_point), adaptive),
    ];
    for (label, values, objects, params) in cases {
        let objects_py = match objects {
            Some(json) => format!("{json:?}"),
            None => "None".to_string(),
        };
        let expected = py.eval(&format!("adaptive_run({}, {objects_py}, {})", py_params(values), py_params(params)));
        check(label, expected, rust_adaptive_run(values, objects, params));
    }
}

/// `ZMesh(params, None)` names the mesh `"adaptive-%X" % id(self)`: uppercase
/// hex without a `0x` prefix (the shape the parity normalizer matches), and
/// distinct for distinct meshes.
#[test]
fn adaptive_profile_name_shape() {
    // Python names an unnamed mesh `"adaptive-%X" % (id(self),)`: the id differs per run, so
    // the comparison is on the shape of two live meshes' names on each side.
    let mut py = oracle().expect("oracle");
    py.exec(&format!(
        "import extras.bed_mesh as bm\na = bm.ZMesh({p}, None)\nb = bm.ZMesh({p}, None)",
        p = py_zmesh_params("direct", 0, 0.2)
    ))
    .expect("python ZMesh pair");
    let shape = |a: &str, b: &str| {
        let (pa, ha) = split_adaptive_name(a);
        let (pb, hb) = split_adaptive_name(b);
        let upper_hex = |h: &str| !h.is_empty() && h.chars().all(|c| c.is_ascii_digit() || c.is_ascii_uppercase());
        let no_leading_zero = |h: &str| !h.starts_with('0');
        Value::Tuple(vec![
            Value::str(&pa),
            Value::Bool(pa == pb),
            Value::Bool(upper_hex(&ha) && upper_hex(&hb)),
            Value::Bool(no_leading_zero(&ha) && no_leading_zero(&hb)),
            Value::Bool(a != b),
        ])
    };
    let py_names = match py.eval("(a.get_profile_name(), b.get_profile_name())") {
        Ok(Value::Tuple(items)) => items
            .into_iter()
            .map(|v| match v {
                Value::Str(s) => s,
                other => panic!("python profile name: {other:?}"),
            })
            .collect::<Vec<String>>(),
        other => panic!("python profile names: {other:?}"),
    };
    let a = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    let b = ZMesh::new(zmesh_params("direct", 0, 0.2), None);
    check(
        "adaptive name shape",
        Ok(shape(&py_names[0], &py_names[1])),
        shape(&a.get_profile_name(), &b.get_profile_name()),
    );
}
