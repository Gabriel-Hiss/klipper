//! Parity tests for `extras::print_stats` (port of
//! `klippy/extras/print_stats.py`).
//!
//! A fresh [`PrintStats`] is built on both sides with minimal fakes
//! for the printer, config and `gcode_move`, fed identical inputs,
//! and the results are compared bit-for-bit through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). Scenarios cover the
//! `SET_PRINT_STATS_INFO` command (including its error paths), the
//! `extruder:activate_extruder` event wiring, filament accounting,
//! the state machine (`note_start` / `note_pause` / finish), layer
//! tracking and `get_status` (Python `get_status` updates the object
//! in place, and so does the port).
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit; expected values come from the oracle alone.
//!
//! One exclusion, as in `core/tests/clocksync.rs`: fields derived
//! from `reactor.monotonic()` cannot match bit-for-bit, because the
//! Rust side reads the real clock while the Python fake returns a
//! fixed value. In scenarios where a call to `note_start` (or a
//! finish) actually reads the clock, `total_duration` and
//! `print_duration` are excluded from the comparison and the
//! remaining fields (`filename`, `filament_used`, `state`,
//! `message`, `info`) are compared bit-for-bit. The duration
//! arithmetic is still compared exactly in standby scenarios (both
//! sides report `0.0`), and `filament_used` — the quantity the
//! durations gate on — is always compared.

use extras::print_stats::PrintStats;
use indexmap::IndexMap;
use klippy_api::config::{Config, ConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts, GetIntOpts, MoveTransform, OutputCallback, RegisterCommandOpts};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

fn internal(class: &str, msg: &str) -> KlippyError {
    KlippyError::Internal {
        class_name: class.to_string(),
        message: msg.to_string(),
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import gcode as _gcode
import extras.print_stats as print_stats

class FakeReactor:
    def __init__(self):
        self.t = 1000.0
    def monotonic(self):
        return self.t

class FakeGCodeMove:
    def __init__(self):
        self.e = 0.0
        self.extrude_factor = 1.0
        self.records = []
    def get_status(self, eventtime=None):
        self.records.append(eventtime)
        return {'extrude_factor': self.extrude_factor,
                'position': _gcode.Coord([0., 0., 0., self.e])}

class FakeGCode:
    def __init__(self):
        self.cmds = []
    def respond_info(self, msg, log=True): pass
    def respond_raw(self, msg): pass
    def register_command(self, cmd, func, desc=None, when_not_ready=False):
        self.cmds.append((cmd, desc))

class FakePrinter:
    def __init__(self, gm, gc, reactor):
        self.gm = gm
        self.gc = gc
        self.reactor = reactor
        self.handlers = []
    def load_object(self, config, section, default=None):
        assert section == 'gcode_move', section
        return self.gm
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=None):
        assert name == 'gcode', name
        return self.gc
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

def _fresh():
    gm = FakeGCodeMove()
    gc = FakeGCode()
    reactor = FakeReactor()
    printer = FakePrinter(gm, gc, reactor)
    stats = print_stats.load_config(FakeConfig(printer))
    return stats, gm, gc, printer, reactor

def _cmd(params, line):
    cmd = _gcode.GCodeCommand(gc, 'SET_PRINT_STATS_INFO', line, params, False)
    return stats.cmd_SET_PRINT_STATS_INFO(cmd)
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("print_stats fakes");
            py.exec("stats, gm, gc, printer, reactor = _fresh()")
                .expect("print_stats construct");
            Some(py)
        }
        None => {
            println!(
                "skipping print_stats oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fresh_reactor() -> Reactor {
    Reactor::new(false)
}

/// `GCodeMove`-service fake: steerable E position / extrude factor,
/// with the received event-times recorded. Returns the dict shape
/// of `GCodeMove.get_status` (`print_stats` reads `position.e` =
/// element 3 and `extrude_factor`).
struct FakeGCodeMove {
    inner: Mutex<FakeGmState>,
}

struct FakeGmState {
    e: f64,
    extrude_factor: f64,
    records: Vec<Option<f64>>,
}

impl FakeGCodeMove {
    fn new() -> FakeGCodeMove {
        FakeGCodeMove {
            inner: Mutex::new(FakeGmState {
                e: 0.,
                extrude_factor: 1.,
                records: Vec::new(),
            }),
        }
    }

    fn set_e(&self, e: f64) {
        self.inner.lock().e = e;
    }

    fn set_extrude_factor(&self, f: f64) {
        self.inner.lock().extrude_factor = f;
    }

    fn records(&self) -> Vec<Option<f64>> {
        self.inner.lock().records.clone()
    }

    fn status(&self, eventtime: Option<f64>) -> PyValue {
        let mut g = self.inner.lock();
        g.records.push(eventtime);
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("extrude_factor".to_string()),
            PyValue::Float(g.extrude_factor),
        );
        d.insert(
            PyValue::Str("position".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(0.),
                PyValue::Float(0.),
                PyValue::Float(0.),
                PyValue::Float(g.e),
            ]),
        );
        PyValue::Dict(d)
    }
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        self.status(eventtime)
    }

    fn set_move_transform(
        &self,
        _transform: MoveTransform,
        _force: bool,
    ) -> KResult<Option<MoveTransform>> {
        Err(internal(
            "FakeGCodeMove",
            "set_move_transform is not used in print_stats tests",
        ))
    }

    fn reset_last_position(&self) -> KResult<()> { Ok(()) }

    fn cmd_g1(&self, _gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        Err(internal(
            "FakeGCodeMove",
            "cmd_g1 is not used in print_stats tests",
        ))
    }
}

impl PrinterObject for FakeGCodeMove {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeMoveApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(self.status(None))
    }
}

/// `GCodeDispatch` fake: records `(cmd, desc)` registrations and
/// keeps the handlers so the test can fire them.
struct FakeGCode {
    reactor: Reactor,
    registered: Mutex<Vec<(String, Option<String>, GCodeCommandHandler)>>,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> FakeGCode {
        FakeGCode {
            reactor,
            registered: Mutex::new(Vec::new()),
        }
    }

    fn registered_names(&self) -> Vec<(String, Option<String>)> {
        self.registered
            .lock()
            .iter()
            .map(|(c, d, _)| (c.clone(), d.clone()))
            .collect()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut g = self.registered.lock();
        if func.is_none() {
            if let Some(pos) = g.iter().position(|(c, _, _)| c == cmd) {
                let (_, _, old) = g.remove(pos);
                return Ok(Some(old));
            }
            return Ok(None);
        }
        g.push((cmd.to_string(), opts.desc, func.unwrap()));
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
        Err(internal(
            "FakeGCode",
            "register_mux_command is not used in print_stats tests",
        ))
    }

    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }

    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }

    fn register_output_handler(&self, _callback: OutputCallback) {}

    fn request_restart(&self, _result: &str) -> KResult<()> {
        Err(internal(
            "FakeGCode",
            "request_restart is not used in print_stats tests",
        ))
    }

    fn run_script(&self, _script: &str) -> KResult<()> {
        Err(internal(
            "FakeGCode",
            "run_script is not used in print_stats tests",
        ))
    }

    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Err(internal(
            "FakeGCode",
            "run_script_from_command is not used in print_stats tests",
        ))
    }

    fn get_mutex(&self) -> ReactorMutex {
        self.reactor.mutex(false)
    }

    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        })
    }

    fn respond_raw(&self, _msg: &str) {}

    fn respond_info(&self, _msg: &str, _log: bool) {}

    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        klippy_core::gcode_parse::is_traditional_gcode(cmd)
    }

    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Err(internal(
            "FakeGCode",
            "cmd_m112 is not used in print_stats tests",
        ))
    }

    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Err(internal(
            "FakeGCode",
            "process_commands is not used in print_stats tests",
        ))
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
    fn get_status(&self, _eventtime: f64) -> Option<PyValue> {
        Some(PyValue::Dict(IndexMap::new()))
    }
}

/// `GCodeCommand` fake with `gcode.py:65-92` (`get` / `get_int`)
/// semantics, including the exact error texts.
struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCmd {
    fn new(line: &str, pairs: &[(&str, &str)]) -> FakeGCmd {
        let mut params = IndexMap::new();
        for (k, v) in pairs {
            params.insert(k.to_string(), v.to_string());
        }
        FakeGCmd {
            command: "SET_PRINT_STATS_INFO".to_string(),
            commandline: line.to_string(),
            params,
        }
    }
}

fn gcmd_handle(cmd: FakeGCmd) -> klippy_api::gcode::GCmd {
    Arc::new(cmd)
}

impl GCodeCommandApi for FakeGCmd {
    fn get(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetOpts,
    ) -> KResult<PyValue> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(v) => Ok(v),
            };
        };
        let parsed = match opts.parser {
            klippy_api::gcode::GetParser::Str => PyValue::Str(value.clone()),
            klippy_api::gcode::GetParser::Int
            | klippy_api::gcode::GetParser::IntAutoBase => {
                match pycompat::text::int_from_string(value, 10) {
                    Ok(n) => PyValue::Int(n),
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {}",
                            self.commandline, value
                        )))
                    }
                }
            }
            klippy_api::gcode::GetParser::Float => {
                match pycompat::text::float_from_string(value) {
                    Ok(f) => PyValue::Float(f),
                    Err(_) => {
                        return Err(KlippyError::Command(format!(
                            "Error on '{}': unable to parse {}",
                            self.commandline, value
                        )))
                    }
                }
            }
        };
        let minv = opts.minval.as_ref().and_then(|m| m.to_f64());
        if let (Some(v), Some(m)) = (parsed.to_f64(), minv) {
            if v < m {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    opts.minval.unwrap().str()
                )));
            }
        }
        let maxv = opts.maxval.as_ref().and_then(|m| m.to_f64());
        if let (Some(v), Some(m)) = (parsed.to_f64(), maxv) {
            if v > m {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    opts.maxval.unwrap().str()
                )));
            }
        }
        Ok(parsed)
    }

    fn get_int(&self, name: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(KlippyError::Command(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            };
        };
        let n = match pycompat::text::int_from_string(value, 10) {
            Ok(n) => n,
            Err(_) => {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, value
                )))
            }
        };
        let v: i64 = match n.0.to_string().parse() {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, value
                )))
            }
        };
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline, name, minval
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Command(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline, name, maxval
                )));
            }
        }
        Ok(Some(v))
    }

    fn get_float(&self, _name: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Err(internal(
            "FakeGCmd",
            "get_float is not used in print_stats tests",
        ))
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
        // `get_raw_command_parameters` (gcode.py:40) without the
        // line-number / checksum handling, which print_stats never
        // exercises.
        self.commandline
            .strip_prefix(&self.command)
            .map(str::to_string)
            .unwrap_or_default()
    }

    fn respond_info(&self, _msg: &str, _log: bool) {}

    fn respond_raw(&self, _msg: &str) {}

    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

/// `Printer` fake: hands the print-stats fakes to `load_config`.
struct FakePrinter {
    reactor: Reactor,
    gm: Arc<FakeGCodeMove>,
    gc: Arc<FakeGCode>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
}

impl FakePrinter {
    fn new(reactor: Reactor, gm: Arc<FakeGCodeMove>, gc: Arc<FakeGCode>) -> FakePrinter {
        FakePrinter {
            reactor,
            gm,
            gc,
            handlers: Mutex::new(Vec::new()),
        }
    }

    fn handler_names(&self) -> Vec<String> {
        self.handlers
            .lock()
            .iter()
            .map(|(n, _)| n.clone())
            .collect()
    }

    fn fire_handler(&self, name: &str) -> KResult<PyValue> {
        let mut g = self.handlers.lock();
        let (_, cb) = g.iter_mut().find(|(n, _)| n == name).unwrap();
        cb(&[])
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

    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        Ok(())
    }

    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if name == "gcode" {
            Ok(Some(self.gc.clone()))
        } else {
            match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            }
        }
    }

    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        vec![
            ("gcode".to_string(), self.gc.clone()),
            ("gcode_move".to_string(), self.gm.clone()),
        ]
    }

    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if section == "gcode_move" {
            Ok(Some(self.gm.clone()))
        } else {
            match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unable to load module '{section}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            }
        }
    }

    fn run(&self) -> Option<String> {
        None
    }

    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}

    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}

    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}

    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .lock()
            .push((event.to_string(), callback));
    }

    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
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
        "print_stats".to_string()
    }
    fn get(&self, _o: &str, _x: klippy_api::config::GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _o: &str, _x: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _o: &str, _x: klippy_api::config::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _o: &str, _x: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _x: klippy_api::config::GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _o: &str, _x: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _o: &str, _x: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _o: &str, _x: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _o: &str, _x: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection unused in print_stats tests")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}


// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

/// A `PyValue` as a recording [`Value`] (see `parity/record/FORMAT.md`).
fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.0.to_string().parse::<i64>() {
            Ok(i) => Value::Int(i),
            Err(_) => Value::BigInt(n.0.to_string()),
        },
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => {
            Value::Tuple(items.iter().map(pyvalue_to_value).collect())
        }
        PyValue::Dict(pairs) => Value::Dict(
            pairs
                .iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => panic!("host value {h} in status dict"),
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "print_stats".to_string(),
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

/// Compares an oracle-raised exception with a Rust error (class and
/// message, the user-visible contract).
fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match (expected, actual) {
        (Err(e), Err(ke)) => {
            assert_eq!(e.class_name, ke.class_name(), "{label}: class");
            assert_eq!(e.message, ke.message(), "{label}: message");
        }
        (Ok(v), Ok(())) => panic!("{label}: oracle returned {v:?}, expected a raise"),
        (Err(e), Ok(())) => {
            panic!(
                "{label}: oracle raised {}: {}, Rust returned ok",
                e.class_name, e.message
            )
        }
        (Ok(v), Err(ke)) => panic!("{label}: oracle returned {v:?}, Rust raised {ke:?}"),
    }
}

/// `status` without the clock-derived fields (`total_duration`,
/// `print_duration`): the real clock on the Rust side and the fixed
/// fake on the Python side make those two bit-different (see the
/// module docs).
fn status_without_durations(status: Value) -> Value {
    let Value::Dict(pairs) = status else {
        panic!("status is not a dict")
    };
    Value::Dict(
        pairs
            .iter()
            .filter(|(k, _)| {
                !matches!(k, Value::Str(s) if s == "total_duration" || s == "print_duration")
            })
            .cloned()
            .collect(),
    )
}

/// Builds a Rust-side `PrintStats` the way `load_config` does
/// (`new` + `register`), returning the shared handles so the test
/// can steer the fakes.
struct RustPair {
    stats: Arc<PrintStats>,
    gm: Arc<FakeGCodeMove>,
    gc: Arc<FakeGCode>,
    printer: Arc<FakePrinter>,
}

fn rust_pair() -> RustPair {
    let reactor = fresh_reactor();
    let gm = Arc::new(FakeGCodeMove::new());
    let gc = Arc::new(FakeGCode::new(reactor.clone()));
    let printer = Arc::new(FakePrinter::new(reactor.clone(), gm.clone(), gc.clone()));
    let printer_iface: Printer = printer.clone();
    let config: Config = Arc::new(FakeConfig {
        printer: printer_iface.clone(),
    });
    let stats = PrintStats::new(&config).expect("load_config");
    RustPair {
        stats,
        gm,
        gc,
        printer,
    }
}

/// `stats.get_status(at)` on the Python side, as a recording value.
fn py_get_status(py: &mut pyoracle::Oracle, at: f64) -> Value {
    let expr = format!("stats.get_status({at:?})");
    match py.eval(&expr) {
        Ok(v) => v,
        Err(e) => panic!("oracle get_status raised {}: {}", e.class_name, e.message),
    }
}

/// Compares the time-independent subset of both sides' statuses,
/// queried at the same `at`.
fn check_status_subset(
    label: &str,
    py: &mut pyoracle::Oracle,
    rust: &PrintStats,
    at: f64,
) {
    check(
        label,
        Ok(status_without_durations(py_get_status(py, at))),
        status_without_durations(pyvalue_to_value(&rust.get_status(at))),
    );
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_dispatches_to_load_config() {
    let factory = extras::print_stats::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn fresh_state_and_registration_match_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    check(
        "fresh get_status",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
    rust.stats.set_current_file("test.gcode");
    py.exec("stats.set_current_file('test.gcode')")
        .expect("py set_current_file");
    check(
        "get_status after set_current_file",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
    check(
        "registered commands",
        py.eval("gc.cmds"),
        Value::List(
            rust.gc
                .registered_names()
                .into_iter()
                .map(|(cmd, desc)| {
                    Value::Tuple(vec![
                        Value::str(&cmd),
                        match desc {
                            Some(d) => Value::str(&d),
                            None => Value::Null,
                        },
                    ])
                })
                .collect(),
        ),
    );
    check(
        "registered handlers",
        py.eval("[e for (e, _c) in printer.handlers]"),
        Value::List(
            rust.printer
                .handler_names()
                .into_iter()
                .map(|n| Value::str(&n))
                .collect(),
        ),
    );
}

#[test]
fn layer_commands_match_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    // Fresh defaults: both layers fall back to the current info.
    let cmd = FakeGCmd::new("SET_PRINT_STATS_INFO", &[]);
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({}, 'SET_PRINT_STATS_INFO')")
        .expect("py cmd no params");
    check(
        "no params on fresh",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );

    // New total resets the current layer to 0, then applies current.
    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=3",
        &[("TOTAL_LAYER", "10"), ("CURRENT_LAYER", "3")],
    );
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({'TOTAL_LAYER': '10', 'CURRENT_LAYER': '3'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=3')")
        .expect("py cmd total+current");
    check(
        "total=10 current=3",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );

    // Unchanged total keeps the layer, then current advances.
    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=7",
        &[("TOTAL_LAYER", "10"), ("CURRENT_LAYER", "7")],
    );
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({'TOTAL_LAYER': '10', 'CURRENT_LAYER': '7'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=10 CURRENT_LAYER=7')")
        .expect("py cmd unchanged total");
    check(
        "total=10 current=7",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
    // Current above total clamps to total; total=0 clears both.
    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=5 CURRENT_LAYER=99",
        &[("TOTAL_LAYER", "5"), ("CURRENT_LAYER", "99")],
    );
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({'TOTAL_LAYER': '5', 'CURRENT_LAYER': '99'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=5 CURRENT_LAYER=99')")
        .expect("py cmd clamp");
    check(
        "clamped to total",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );

    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=0",
        &[("TOTAL_LAYER", "0")],
    );
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({'TOTAL_LAYER': '0'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=0')")
        .expect("py cmd zero total");
    check(
        "total=0 clears",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
}

#[test]
fn layer_command_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=-1",
        &[("TOTAL_LAYER", "-1")],
    );
    check_err(
        "negative total_layer",
        py.eval("_cmd({'TOTAL_LAYER': '-1'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=-1')"),
        rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)),
    );

    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO CURRENT_LAYER=slow",
        &[("CURRENT_LAYER", "slow")],
    );
    check_err(
        "unparsable current_layer",
        py.eval("_cmd({'CURRENT_LAYER': 'slow'}, 'SET_PRINT_STATS_INFO CURRENT_LAYER=slow')"),
        rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)),
    );

    // Failed parses leave the info untouched.
    check(
        "info untouched after errors",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
}

#[test]
fn filament_pause_and_activate_match_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    // The activate handler re-syncs last_epos without touching
    // the filament total.
    rust.gm.set_e(10.0);
    py.exec("gm.e = 10.0").expect("py set e");
    rust.printer
        .fire_handler("extruder:activate_extruder")
        .expect("rust event");
    py.exec("printer.handlers[0][1]()").expect("py event");

    // First pause accumulates the delta over the new E position,
    // scaled by the extrude factor.
    rust.gm.set_e(15.0);
    rust.gm.set_extrude_factor(2.5);
    py.exec("gm.e = 15.0; gm.extrude_factor = 2.5")
        .expect("py set e extrude");
    rust.stats.note_pause();
    py.exec("stats.note_pause()").expect("py note_pause");
    check(
        "filament after first pause",
        py.eval("stats.get_status(100.0)"),
        pyvalue_to_value(&rust.stats.get_status(100.0)),
    );

    // A second pause is a no-op for the filament (last_pause_time
    // is set); only the state is re-applied.
    rust.gm.set_e(40.0);
    py.exec("gm.e = 40.0").expect("py set e2");
    rust.stats.note_pause();
    py.exec("stats.note_pause()").expect("py note_pause again");
    check(
        "filament after second pause",
        py.eval("stats.get_status(200.0)"),
        pyvalue_to_value(&rust.stats.get_status(200.0)),
    );
}

#[test]
fn start_pause_complete_subset_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    rust.gm.set_e(1.0);
    py.exec("gm.e = 1.0; reactor.t = 1000.0").expect("py set e");
    rust.stats.note_start();
    py.exec("stats.note_start()").expect("py note_start");
    check_status_subset("printing subset", &mut py, &rust.stats, 43.0);

    // Extrusion during printing is accumulated by get_status.
    rust.gm.set_e(6.0);
    py.exec("gm.e = 6.0").expect("py set e");
    check_status_subset("filament while printing", &mut py, &rust.stats, 44.0);

    // Pause freezes the accumulation.
    rust.stats.note_pause();
    py.exec("reactor.t = 1010.0; stats.note_pause()")
        .expect("py note_pause");
    rust.gm.set_e(11.0);
    py.exec("gm.e = 11.0").expect("py set e2");
    check_status_subset("paused subset", &mut py, &rust.stats, 45.0);

    rust.stats.note_complete();
    py.exec("stats.note_complete()").expect("py note_complete");
    check_status_subset("complete subset", &mut py, &rust.stats, 46.0);
}

#[test]
fn finish_before_start_and_error_match_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    // Finishing before a start is a no-op (print_start_time is None).
    rust.stats.note_complete();
    py.exec("stats.note_complete()").expect("py finish");
    rust.stats.note_error("ignored");
    py.exec("stats.note_error('ignored')").expect("py error");
    check(
        "standby survives early finishes",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );

    rust.stats.note_start();
    py.exec("stats.note_start()").expect("py start");
    rust.stats.note_error("printer jammed");
    py.exec("stats.note_error('printer jammed')").expect("py note_error");
    check_status_subset("error subset", &mut py, &rust.stats, 47.0);

    // Pausing during the error state keeps the state.
    rust.stats.note_pause();
    py.exec("stats.note_pause()").expect("py pause");
    check_status_subset("error survives pause", &mut py, &rust.stats, 48.0);

    // Finish cleared print_start_time, so the cancel is a no-op too
    // and the error message survives.
    rust.stats.note_cancel();
    py.exec("stats.note_cancel()").expect("py cancel");
    check_status_subset("cancel after finish is no-op", &mut py, &rust.stats, 49.0);
}

#[test]
fn reset_via_set_current_file_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    let cmd = FakeGCmd::new(
        "SET_PRINT_STATS_INFO TOTAL_LAYER=9 CURRENT_LAYER=4",
        &[("TOTAL_LAYER", "9"), ("CURRENT_LAYER", "4")],
    );
    rust.stats.cmd_set_print_stats_info(&gcmd_handle(cmd)).unwrap();
    py.exec("_cmd({'TOTAL_LAYER': '9', 'CURRENT_LAYER': '4'}, 'SET_PRINT_STATS_INFO TOTAL_LAYER=9 CURRENT_LAYER=4')")
        .expect("py cmd");
    rust.stats.note_start();
    py.exec("stats.note_start()").expect("py start");

    rust.stats.set_current_file("next.gcode");
    py.exec("stats.set_current_file('next.gcode')")
        .expect("py set_current_file");
    check(
        "reset by set_current_file",
        py.eval("stats.get_status(42.0)"),
        pyvalue_to_value(&rust.stats.get_status(42.0)),
    );
}

#[test]
fn status_query_count_matches_python() {
    let Some(mut py) = oracle() else { return };
    let rust = rust_pair();

    rust.stats.note_pause();
    py.exec("stats.note_pause()").expect("py pause");
    rust.stats.note_start();
    py.exec("stats.note_start()").expect("py start");
    rust.stats.get_status(12.0);
    py.exec("stats.get_status(12.0)").expect("py get_status");
    check(
        "gcode_move get_status count",
        py.eval("len(gm.records)"),
        Value::int(rust.gm.records().len() as i64),
    );
}
