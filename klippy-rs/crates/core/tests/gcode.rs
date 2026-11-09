//! Parity tests for `klippy_core::gcode` (ports of `klippy/gcode.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a Python `GCodeDispatch` /
//! `GCodeIO` is built on minimal fakes for the printer, reactor and
//! toolhead, fed identical inputs, and the outputs, shutdowns, exits,
//! toolhead calls, handler tables, status dicts and (for IO) byte counts
//! and pending lines are compared with the Rust port bit for bit (floats
//! by bits via the recording value format). When the oracle binary is
//! unavailable the oracle tests skip with a message.
//!
//! The one place Python cannot be byte-stable is the mux duplicate
//! message, which embeds a `dict` repr with `<function ... at 0x...>`
//! addresses; that check compares the message prefix instead. The Rust
//! registration has no function name, so it renders `<function
//! anonymous>` there (documented in the module).

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCmd, GetFloatOpts, GetIntOpts, GetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_ANALYZE_SHUTDOWN,
    EV_KLIPPY_SHUTDOWN,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingTargetApi, Kinematics, PrinterExtruder, ToolHeadApi,
};
use klippy_core::gcode::{format_exiting, format_input_dump, GCodeDispatch, GCodeIO};
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import gcode as gmod
import logging as _lg
LOGS = []
class _C(_lg.Handler):
    def emit(self, r): LOGS.append(r.getMessage())
_root = _lg.getLogger()
_root.handlers = [_C()]
_root.setLevel(0)
class Error(Exception): pass
class FakeReactor:
    def __init__(self): self.fds = []
    def mutex(self): return FakeMutex()
    def register_fd(self, fd, cb): self.fds.append(fd); return ('fd', fd)
    def unregister_fd(self, h): pass
class FakeMutex:
    def __init__(self): self.locked = False
    def __enter__(self): self.locked = True; return self
    def __exit__(self, *a): self.locked = False
class FakeToolHead:
    def __init__(self, t): self.print_time = float(t); self.dwell_calls = []; self.wait_calls = 0
    def get_last_move_time(self): return self.print_time
    def dwell(self, t): self.dwell_calls.append(float(t))
    def wait_moves(self): self.wait_calls += 1
class FakePrinter:
    config_error = Error
    def __init__(self, start_args, state_msg):
        self.start_args = start_args
        self.reactor = FakeReactor()
        self.objects = {}
        self.handlers = {}
        self.sent = []
        self.shutdowns = []
        self.analyzes = []
        self.exits = []
        self.state_msg = state_msg
    def get_start_args(self): return self.start_args
    def get_reactor(self): return self.reactor
    def get_state_message(self): return (self.state_msg, 'ready' if self.state_msg == 'Printer is ready' else 'startup')
    def register_event_handler(self, event, cb): self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        self.sent.append((event, list(params)))
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def invoke_shutdown(self, msg, details=None):
        self.shutdowns.append(msg)
        for cb in list(self.handlers.get('klippy:shutdown', [])):
            try: cb()
            except Exception: _lg.exception('Exception during shutdown handler')
        self.analyzes.append((msg, details))
        for cb in list(self.handlers.get('klippy:analyze_shutdown', [])):
            try: cb(msg, {} if details is None else details)
            except Exception: _lg.exception('Exception in analyze_shutdown handler')
    def request_exit(self, result):
        if not self.exits: self.exits.append(result)
    def lookup_object(self, name, default=None):
        if name in self.objects: return self.objects[name]
        if default is None: raise Error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        if name in self.objects: raise Error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
P = None; D = None; OUT = None; TH = None; IO = None
def new_world(start_args, state_msg, print_time):
    global P, D, OUT, TH, IO
    OUT = []
    TH = FakeToolHead(print_time)
    P = FakePrinter(dict(start_args), state_msg)
    P.objects['toolhead'] = TH
    D = gmod.GCodeDispatch(P)
    P.objects['gcode'] = D
    D.register_output_handler(OUT.append)
    IO = None
    return None
def outputs(): return list(OUT)
def shutdowns(): return list(P.shutdowns)
def exits(): return list(P.exits)
def count_sent(name): return len([s for s in P.sent if s[0] == name])
def tool_calls(): return (list(TH.dwell_calls), TH.wait_calls)
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(PY_SETUP).expect("gcode fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping gcode oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn render_arg(arg: &EventArg) -> String {
    match arg {
        EventArg::Value(v) => v.str(),
        EventArg::Object(_) => "<object>".to_string(),
        EventArg::Any(_) => "<object>".to_string(),
        EventArg::List(items) => {
            let parts: Vec<String> = items.iter().map(render_arg).collect();
            format!("[{}]", parts.join(", "))
        }
    }
}

pub struct FakePrinter {
    reactor: Reactor,
    start_args: PyValue,
    state_message: Mutex<(String, String)>,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    event_handlers: Mutex<Vec<(String, Vec<EventCallback>)>>,
    sent_events: Mutex<Vec<String>>,
    shutdowns: Mutex<Vec<String>>,
    exits: Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new(start_args: PyValue, state: (&str, &str)) -> FakePrinter {
        FakePrinter {
            reactor: Reactor::new(false),
            start_args,
            state_message: Mutex::new((state.0.to_string(), state.1.to_string())),
            objects: Mutex::new(IndexMap::new()),
            event_handlers: Mutex::new(Vec::new()),
            sent_events: Mutex::new(Vec::new()),
            shutdowns: Mutex::new(Vec::new()),
            exits: Mutex::new(Vec::new()),
        }
    }


    fn sent_count(&self, event: &str) -> i64 {
        self.sent_events
            .lock()
            .iter()
            .filter(|s| s.starts_with(&format!("{event}:")))
            .count() as i64
    }

    fn shutdowns(&self) -> Vec<String> {
        self.shutdowns.lock().clone()
    }

    fn exits(&self) -> Vec<String> {
        self.exits.lock().clone()
    }

    fn fire(&self, event: &str, params: &[EventArg]) {
        let mut handlers = self.event_handlers.lock();
        for (name, cbs) in handlers.iter_mut() {
            if name == event {
                for cb in cbs.iter_mut() {
                    // Shutdown firing swallows handler exceptions in Python.
                    let _ = cb(params);
                }
            }
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.start_args.clone()
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        self.state_message.lock().clone()
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
        match objects.get(name).cloned() {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::Config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objects = self.objects.lock();
        match module {
            None => objects
                .iter()
                .map(|(n, o)| (n.clone(), Arc::clone(o)))
                .collect(),
            Some(m) => {
                let prefix = format!("{m} ");
                let mut prefixed: Vec<(String, ObjectRef)> = objects
                    .iter()
                    .filter(|(n, _)| n.starts_with(&prefix))
                    .map(|(n, o)| (n.clone(), Arc::clone(o)))
                    .collect();
                if let Some(o) = objects.get(m) {
                    let mut out = vec![(m.to_string(), Arc::clone(o))];
                    out.append(&mut prefixed);
                    out
                } else {
                    prefixed
                }
            }
        }
    }
    fn load_object(
        &self,
        _config: &klippy_api::config::Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Err(KlippyError::Config(format!(
            "Unable to load module '{section}'"
        )))
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, details: PyValue) {
        self.shutdowns.lock().push(msg.to_string());
        self.fire(EV_KLIPPY_SHUTDOWN, &[]);
        self.fire(
            EV_KLIPPY_ANALYZE_SHUTDOWN,
            &[
                EventArg::Value(PyValue::Str(msg.to_string())),
                EventArg::Value(details),
            ],
        );
    }
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        self.invoke_shutdown(msg, details);
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        let mut handlers = self.event_handlers.lock();
        match handlers.iter_mut().find(|(n, _)| n == event) {
            Some((_, cbs)) => cbs.push(callback),
            None => handlers.push((event.to_string(), vec![callback])),
        }
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let rendered: Vec<String> = params.iter().map(render_arg).collect();
        self.sent_events.lock().push(format!("{event}:{:?}", rendered));
        let mut handlers = self.event_handlers.lock();
        let mut results = Vec::new();
        for (name, cbs) in handlers.iter_mut() {
            if name == event {
                for cb in cbs.iter_mut() {
                    results.push(cb(params)?);
                }
            }
        }
        Ok(results)
    }
    fn request_exit(&self, result: &str) {
        let mut exits = self.exits.lock();
        if exits.is_empty() {
            exits.push(result.to_string());
        }
    }
}

fn start_args(debuginput: bool, gcode_fd: Option<i32>, software_version: Option<&str>) -> PyValue {
    let mut m = IndexMap::new();
    m.insert(
        PyValue::Str("config_file".to_string()),
        PyValue::Str("test.cfg".to_string()),
    );
    if debuginput {
        m.insert(
            PyValue::Str("debuginput".to_string()),
            PyValue::Str("/tmp/gcode-in".to_string()),
        );
    }
    if let Some(fd) = gcode_fd {
        m.insert(
            PyValue::Str("gcode_fd".to_string()),
            PyValue::Int(PyInt::from_i64(fd as i64)),
        );
    }
    if let Some(v) = software_version {
        m.insert(
            PyValue::Str("software_version".to_string()),
            PyValue::Str(v.to_string()),
        );
    }
    PyValue::Dict(m)
}

pub struct FakeToolHead {
    print_time: f64,
    dwell_calls: Mutex<Vec<f64>>,
    wait_calls: Mutex<i64>,
}

impl FakeToolHead {
    fn new(print_time: f64) -> FakeToolHead {
        FakeToolHead {
            print_time,
            dwell_calls: Mutex::new(Vec::new()),
            wait_calls: Mutex::new(0),
        }
    }
    fn tool_calls(&self) -> (Vec<f64>, i64) {
        (self.dwell_calls.lock().clone(), *self.wait_calls.lock())
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("fake toolhead: get_kinematics")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(self.print_time)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        self.dwell_calls.lock().push(delay);
        Ok(())
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
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
    fn wait_moves(&self) -> KResult<()> {
        *self.wait_calls.lock() += 1;
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("fake toolhead: get_trapq")
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
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("fake toolhead: set_extruder")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("fake toolhead: get_extruder")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("fake toolhead: add_extra_axis")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("fake toolhead: remove_extra_axis")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("fake toolhead: stepper_factory")
    }
}

struct World {
    printer: Printer,
    fake: Arc<FakePrinter>,
    dispatch: Arc<GCodeDispatch>,
    outputs: Arc<Mutex<Vec<String>>>,
    toolhead: Arc<FakeToolHead>,
}

fn build_world(args: PyValue, state_msg: &str, print_time: f64) -> World {
    let category = if state_msg == "Printer is ready" {
        "ready"
    } else {
        "startup"
    };
    let fake = Arc::new(FakePrinter::new(args, (state_msg, category)));
    let printer: Printer = Arc::clone(&fake) as Printer;
    let toolhead = Arc::new(FakeToolHead::new(print_time));
    printer
        .add_object("toolhead", Arc::clone(&toolhead) as ObjectRef)
        .expect("toolhead object");
    let dispatch = Arc::new(GCodeDispatch::new(printer.clone()));
    printer
        .add_object("gcode", Arc::clone(&dispatch) as ObjectRef)
        .expect("gcode object");
    let outputs = Arc::new(Mutex::new(Vec::new()));
    {
        let out = Arc::clone(&outputs);
        dispatch.register_output_handler(Box::new(move |msg| {
            out.lock().push(msg);
        }));
    }
    World { printer, fake, dispatch, outputs, toolhead }
}

fn make_world(debuginput: bool, state_msg: &str, print_time: f64) -> World {
    build_world(start_args(debuginput, None, Some("v0.12.0-123")), state_msg, print_time)
}

impl World {
    fn outputs(&self) -> Vec<String> {
        self.outputs.lock().clone()
    }

    fn outputs_value(&self) -> Value {
        Value::List(self.outputs().iter().map(|s| Value::str(s)).collect())
    }

    fn process(&self, lines: &[&str], need_ack: bool) -> KResult<()> {
        let lines: Vec<String> = lines.iter().map(|s| s.to_string()).collect();
        self.dispatch.process_commands(&lines, need_ack)
    }

    fn fire_ready(&self) -> KResult<Vec<PyValue>> {
        self.printer.send_event("klippy:ready", &[])
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "gcode".to_string(),
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

fn check_error(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let (exp_class, exp_msg) = match expected {
        Err(e) => (e.class_name, e.message),
        Ok(v) => panic!("{label}: oracle did not raise, got {}", v.render()),
    };
    match actual {
        Err(e) => {
            assert_eq!(
                e.class_name(),
                exp_class.as_str(),
                "{label}: error class for {exp_msg:?}"
            );
            assert_eq!(e.message(), exp_msg.as_str(), "{label}: error message");
        }
        Ok(()) => panic!("{label}: oracle raised {exp_class}: {exp_msg:?}, Rust returned Ok"),
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, val)| (pyvalue_to_value(k), pyvalue_to_value(val)))
                .collect(),
        ),
        PyValue::Host(h) => Value::int(*h as i64),
    }
}

fn error_lines(value: Value) -> Vec<Value> {
    match value {
        Value::List(items) => items
            .into_iter()
            .filter(|v| matches!(v, Value::Str(s) if s.starts_with("!!")))
            .collect(),
        other => panic!("unexpected outputs {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// M115.
// ---------------------------------------------------------------------------

#[test]
fn m115_version_with_and_without_ack() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v0.12.0-123'}, 'Printer is ready', 12.345)",
    )
    .expect("world");
    py.exec("D._process_commands(['M115'], need_ack=True)").expect("py M115");
    py.exec("D._process_commands(['M115'], need_ack=False)").expect("py M115 no ack");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "Printer is ready", 12.345);
    w.process(&["M115"], true).expect("M115 ack");
    w.process(&["M115"], false).expect("M115 no ack");
    check("m115 outputs", Ok(expected), w.outputs_value());
}

#[test]
fn m115_missing_version_is_none() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in'}, \
         'Printer is ready', 0.0)",
    )
    .expect("world without version");
    py.exec("D._process_commands(['M115'], need_ack=True)").expect("py M115");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = build_world(start_args(true, None, None), "Printer is ready", 0.0);
    w.process(&["M115"], true).expect("M115");
    check("m115 none version", Ok(expected), w.outputs_value());
}

// ---------------------------------------------------------------------------
// ECHO / STATUS / HELP, ready and not ready, status dicts.
// ---------------------------------------------------------------------------

#[test]
fn echo_status_help_across_ready_toggle() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v0.12.0-123'}, 'Printer is starting', 3.0)",
    )
    .expect("world");
    py.exec("D._process_commands(['ECHO hello world', 'ECHO  spaced'], need_ack=True)")
        .expect("py echo not ready");
    py.exec("D._process_commands(['STATUS'], need_ack=True)").expect("py status err");
    py.exec("D._process_commands(['HELP'], need_ack=True)").expect("py help not ready");
    py.exec("P.send_event('klippy:ready')").expect("py ready");
    py.exec("D._process_commands(['STATUS', 'HELP'], need_ack=True)").expect("py ready cmds");
    py.exec("P.invoke_shutdown('shutdown-msg')").expect("py shutdown");
    py.exec("D._process_commands(['STATUS'], need_ack=True)").expect("py status after");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "Printer is starting", 3.0);
    w.process(&["ECHO hello world", "ECHO  spaced"], true).expect("rust echo");
    w.process(&["STATUS"], true).expect("rust status err handled");
    w.process(&["HELP"], true).expect("rust help");
    w.fire_ready().expect("rust ready");
    w.process(&["STATUS", "HELP"], true).expect("rust ready cmds");
    w.printer.invoke_shutdown("shutdown-msg", PyValue::Dict(IndexMap::new()));
    w.process(&["STATUS"], true).expect("rust status after");
    check("echo/status/help outputs", Ok(expected), w.outputs_value());
}

#[test]
fn status_dict_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_command('M900', lambda gcmd: None, when_not_ready=True, desc='desc900')",
    )
    .expect("register");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D.register_command('MY_MACRO', lambda gcmd: None, desc='macro help')")
        .expect("register macro");
    let expected_status = py.eval("D.get_status(0.0)").expect("py status");
    let expected_help = py.eval("D.get_command_help()").expect("py help");

    let w = make_world(true, "s", 0.0);
    w.dispatch
        .register_command(
            "M900",
            Some(Box::new(|_| Ok(()))),
            RegisterCommandOpts { when_not_ready: true, desc: Some("desc900".to_string()) },
        )
        .expect("rust register");
    w.fire_ready().expect("rust ready");
    w.dispatch
        .register_command(
            "MY_MACRO",
            Some(Box::new(|_| Ok(()))),
            RegisterCommandOpts {
                when_not_ready: false,
                desc: Some("macro help".to_string()),
            },
        )
        .expect("rust register macro");
    check(
        "get_status",
        Ok(expected_status),
        pyvalue_to_value(&GCodeApi::get_status(&*w.dispatch, 0.0)),
    );
    check(
        "get_command_help",
        Ok(expected_help),
        pyvalue_to_value(&PyValue::Dict(
            w.dispatch
                .get_command_help()
                .into_iter()
                .map(|(k, v)| (PyValue::Str(k), PyValue::Str(v)))
                .collect(),
        )),
    );
}

// ---------------------------------------------------------------------------
// M112 / RESTART / FIRMWARE_RESTART.
// ---------------------------------------------------------------------------

#[test]
fn restart_paths_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is ready', 42.5)",
    )
    .expect("world");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D.cmd_M112(None)").expect("py M112");
    py.exec("D.request_restart('restart')").expect("py restart");
    py.exec("D.request_restart('firmware_restart')").expect("py fw");
    let py_shutdowns = py.eval("shutdowns()").expect("py shutdowns");
    let py_tool = py.eval("tool_calls()").expect("py tool");
    let py_restart_count = py.eval("count_sent('gcode:request_restart')").expect("py count");

    let w = make_world(true, "Printer is ready", 42.5);
    w.fire_ready().expect("rust ready");
    w.dispatch.cmd_m112(None).expect("rust M112");
    w.dispatch.request_restart("restart").expect("rust restart");
    w.dispatch.request_restart("firmware_restart").expect("rust fw");
    check(
        "restart shutdowns",
        Ok(py_shutdowns),
        Value::List(w.fake.shutdowns().iter().map(|s| Value::str(s)).collect()),
    );
    let (dwell, waits) = w.toolhead.tool_calls();
    check(
        "restart tool calls",
        Ok(py_tool),
        Value::Tuple(vec![
            Value::List(dwell.iter().map(|f| Value::float(*f)).collect()),
            Value::int(waits),
        ]),
    );
    check(
        "request_restart events",
        Ok(py_restart_count),
        Value::int(w.fake.sent_count("gcode:request_restart")),
    );
    check("exiting log line", Ok(py
        .eval("'Exiting (print time %.3fs)' % (42.5,)").expect("py log")), Value::str(&format_exiting(42.5)));
}

// ---------------------------------------------------------------------------
// `cmd_default`.
// ---------------------------------------------------------------------------

#[test]
fn default_branches_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is starting', 0.0)",
    )
    .expect("world");
    py.exec("D._process_commands(['M105', 'M21', 'G28'], need_ack=True)")
        .expect("py not ready");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec(
        "D._process_commands(['', 'M104 S0', 'M104 S12.5', 'M140 S0.0', 'M107', \
         'M106 S0', 'M106 S255', 'G28', 'M999 X1'], need_ack=True)",
    )
    .expect("py ready defaults");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "Printer is starting", 0.0);
    w.process(&["M105", "M21", "G28"], true).expect("not ready");
    w.fire_ready().expect("ready");
    w.process(
        &["", "M104 S0", "M104 S12.5", "M140 S0.0", "M107", "M106 S0",
          "M106 S255", "G28", "M999 X1"],
        true,
    )
    .expect("ready defaults");
    check("default branches", Ok(expected), w.outputs_value());
}

#[test]
fn default_m106_respects_fileinput() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'software_version': 'v1'}, \
         'Printer is ready', 0.0)",
    )
    .expect("world without debuginput");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['M106 S100'], need_ack=True)").expect("py m106");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(false, "Printer is ready", 0.0);
    w.fire_ready().expect("ready");
    w.process(&["M106 S100"], true).expect("rust m106");
    check("m106 tty", Ok(expected), w.outputs_value());
}

#[test]
fn default_m117_redispatch_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'ready', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_command('M117', \
         lambda g: g.respond_info('custom117:' + g.get_command(), False))",
    )
    .expect("register M117");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['M117 hello', 'M118 world', 'M23 x'], need_ack=True)")
        .expect("py redispatched");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "ready", 0.0);
    w.dispatch
        .register_command(
            "M117",
            Some(Box::new(|gcmd: &GCmd| {
                gcmd.respond_info(&format!("custom117:{}", gcmd.get_command()), false);
                Ok(())
            })),
            RegisterCommandOpts::default(),
        )
        .expect("rust register M117");
    w.fire_ready().expect("ready");
    w.process(&["M117 hello", "M118 world", "M23 x"], true)
        .expect("rust redispatched");
    check("m117 redispatch", Ok(expected), w.outputs_value());
}

// ---------------------------------------------------------------------------
// Registration and mux.
// ---------------------------------------------------------------------------

#[test]
fn registration_errors_and_unregister_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    let py_dup = py.exec("D.register_command('M110', lambda g: None)");
    let w = make_world(true, "s", 0.0);
    let rs_dup = w.dispatch.register_command(
        "M110",
        Some(Box::new(|_| Ok(()))),
        RegisterCommandOpts::default(),
    );
    check_error("duplicate M110", py_dup, rs_dup.map(|_| ()));

    for name in ["foo", "1ABC", "M1FOO"] {
        let Some(mut py2) = oracle() else { return };
        py2.exec(
            "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
             'software_version': 'v1'}, 's', 0.0)",
        )
        .expect("world");
        let expected = py2.exec(&format!("D.register_command('{name}', lambda g: None)"));
        let w2 = make_world(true, "s", 0.0);
        let actual = w2.dispatch.register_command(
            name,
            Some(Box::new(|_| Ok(()))),
            RegisterCommandOpts::default(),
        );
        check_error(&format!("invalid {name}"), expected, actual.map(|_| ()));
    }

    // Unregister removes the handler: a later call falls to `cmd_default`.
    let old = w
        .dispatch
        .register_command("M110", None, RegisterCommandOpts::default())
        .expect("unregister");
    assert!(old.is_some(), "unregister returns the old handler");
    let none_again = w
        .dispatch
        .register_command("M110", None, RegisterCommandOpts::default())
        .expect("unregister again");
    assert!(none_again.is_none(), "second unregister finds nothing");
}

#[test]
fn mux_dispatch_and_errors_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_mux_command('MUT', 'K', 'a', lambda g: OUT.append('mux a'))",
    )
    .expect("mux a");
    py.exec(
        "D.register_mux_command('MUT', 'K', 'beta', lambda g: OUT.append('mux beta'))",
    )
    .expect("mux beta");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['MUT K=a', 'MUT K=bet', 'MUT K=zz'], need_ack=True)")
        .expect("py mux invalid paths");
    let py_key = py.exec("D.register_mux_command('MUT', 'J', 'x', lambda g: None)");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "s", 0.0);
    w.dispatch
        .register_mux_command(
            "MUT",
            "K",
            Some("a"),
            Box::new(|g: &GCmd| {
                g.respond_raw("mux a");
                Ok(())
            }),
            None,
        )
        .expect("mux a");
    w.dispatch
        .register_mux_command(
            "MUT",
            "K",
            Some("beta"),
            Box::new(|g: &GCmd| {
                g.respond_raw("mux beta");
                Ok(())
            }),
            None,
        )
        .expect("mux beta");
    w.fire_ready().expect("ready");
    w.process(&["MUT K=a", "MUT K=bet", "MUT K=zz"], true)
        .expect("rust mux invalid paths");
    let rs_key = w.dispatch.register_mux_command(
        "MUT",
        "J",
        Some("x"),
        Box::new(|_| Ok(())),
        None,
    );
    check_error("distinct mux key", py_key, rs_key);
    check(
        "mux invalid lines",
        Ok(Value::List(error_lines(expected))),
        Value::List(error_lines(w.outputs_value())),
    );

    let py_dup =
        py.exec("D.register_mux_command('MUT', 'K', 'a', lambda g: None)");
    let rs_dup = w.dispatch.register_mux_command(
        "MUT",
        "K",
        Some("a"),
        Box::new(|_| Ok(())),
        None,
    );
    let py_msg = match py_dup {
        Err(e) => e.message,
        Ok(_) => panic!("py duplicate mux did not raise"),
    };
    let rs_msg = match rs_dup {
        Err(e) => e.message().to_string(),
        Ok(()) => panic!("rust duplicate mux did not raise"),
    };
    let prefix = "mux command MUT K a already registered (";
    assert!(
        py_msg.starts_with(prefix) && rs_msg.starts_with(prefix),
        "duplicate mux prefix: py={py_msg:?} rs={rs_msg:?}"
    );
}

// ---------------------------------------------------------------------------
// Extended parameters and typed getters.
// ---------------------------------------------------------------------------

#[test]
fn extended_params_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_command('FOO', \
         lambda g: OUT.append(sorted(g.get_command_parameters().items())))",
    )
    .expect("register FOO");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['FOO a=1 B=\"two words\"', 'FOO x'], need_ack=True)")
        .expect("py extended + malformed");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "s", 0.0);
    w.dispatch
        .register_command(
            "FOO",
            Some(Box::new(|gcmd: &GCmd| {
                let mut params: Vec<(String, String)> =
                    gcmd.get_command_parameters().into_iter().collect();
                params.sort();
                gcmd.respond_raw(&format!("{params:?}"));
                Ok(())
            })),
            RegisterCommandOpts::default(),
        )
        .expect("register FOO");
    w.fire_ready().expect("ready");
    w.process(&["FOO a=1 B=\"two words\"", "FOO x"], true)
        .expect("rust extended");
    check(
        "malformed lines",
        Ok(Value::List(error_lines(expected))),
        Value::List(error_lines(w.outputs_value())),
    );
}

#[test]
fn typed_getters_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_command('M999', \
         lambda g: (OUT.append(('gi', g.get_int('X', 2))), \
                    OUT.append(('gf', g.get_float('Y', 1.5))), \
                    OUT.append(('gb', g.get('X', None, parser=lambda x: int(x, 0))))))",
    )
    .expect("register");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['M999 X0x10 Y2.5'], need_ack=True)").expect("py getters");
    py.exec("D._process_commands(['M999'], need_ack=True)").expect("py missing");
    py.exec("D._process_commands(['M999 Xabc Y1'], need_ack=True)").expect("py parse err");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "s", 0.0);
    w.dispatch
        .register_command(
            "M999",
            Some(Box::new(|gcmd: &GCmd| {
                let gi = gcmd.get_int(
                    "X",
                    GetIntOpts { default: Some(Some(2)), minval: None, maxval: None },
                )?;
                gcmd.respond_raw(&format!("gi:{gi:?}"));
                let gf = gcmd.get_float(
                    "Y",
                    GetFloatOpts {
                        default: Some(Some(1.5)),
                        minval: None,
                        maxval: None,
                        above: None,
                        below: None,
                    },
                )?;
                gcmd.respond_raw(&format!("gf:{gf:?}"));
                let gb = gcmd.get(
                    "X",
                    GetOpts {
                        default: Some(PyValue::None),
                        parser: GetParser::IntAutoBase,
                        minval: None,
                        maxval: None,
                        above: None,
                        below: None,
                    },
                )?;
                gcmd.respond_raw(&format!("gb:{}", gb.str()));
                Ok(())
            })),
            RegisterCommandOpts::default(),
        )
        .expect("register");
    w.fire_ready().expect("ready");
    w.process(&["M999 X0x10 Y2.5"], true).expect("getters");
    w.process(&["M999"], true).expect("missing");
    w.process(&["M999 Xabc Y1"], true).expect("parse err");
    check(
        "getter error lines",
        Ok(Value::List(error_lines(expected))),
        Value::List(error_lines(w.outputs_value())),
    );
}

// ---------------------------------------------------------------------------
// Command vs internal error handling, `need_ack` behaviour.
// ---------------------------------------------------------------------------

#[test]
fn command_and_internal_error_paths_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "D.register_command('BOOM', lambda g: (_ for _ in ()).throw(ValueError('boom')))",
    )
    .expect("boom");
    py.exec(
        "D.register_command('EERR', lambda g: (_ for _ in ()).throw(gmod.CommandError('bad thing')))",
    )
    .expect("err");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D._process_commands(['EERR'], need_ack=True)").expect("py command err");
    py.exec("D._process_commands(['BOOM'], need_ack=True)").expect("py internal err");
    let py_outputs = py.eval("outputs()").expect("py outputs");
    let py_shutdowns = py.eval("shutdowns()").expect("py shutdowns");
    let py_err_count = py.eval("count_sent('gcode:command_error')").expect("py count");

    let w = make_world(true, "s", 0.0);
    w.dispatch
        .register_command(
            "EERR",
            Some(Box::new(|_| Err(KlippyError::Command("bad thing".to_string())))),
            RegisterCommandOpts::default(),
        )
        .expect("EERR");
    w.dispatch
        .register_command(
            "BOOM",
            Some(Box::new(|_| {
                Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "boom".to_string(),
                })
            })),
            RegisterCommandOpts::default(),
        )
        .expect("BOOM");
    w.fire_ready().expect("ready");
    w.process(&["EERR"], true).expect("rust command err handled");
    w.process(&["BOOM"], true).expect("rust internal handled");
    check("error outputs", Ok(py_outputs), w.outputs_value());
    check(
        "internal shutdowns",
        Ok(py_shutdowns),
        Value::List(w.fake.shutdowns().iter().map(|s| Value::str(s)).collect()),
    );
    check(
        "command_error events",
        Ok(py_err_count),
        Value::int(w.fake.sent_count("gcode:command_error")),
    );

    let py_reraise = py.exec("D._process_commands(['BOOM'], need_ack=False)");
    let rs_reraise = w.process(&["BOOM"], false);
    check_error("internal reraise", py_reraise, rs_reraise);
}

// ---------------------------------------------------------------------------
// `run_script` (mutex) and multi-line scripts.
// ---------------------------------------------------------------------------

#[test]
fn run_script_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec("D.run_script('ECHO MSG=one\\nECHO MSG=two')").expect("py run");
    let expected = py.eval("outputs()").expect("py outputs");

    let w = make_world(true, "s", 0.0);
    w.fire_ready().expect("ready");
    w.dispatch.run_script("ECHO MSG=one\nECHO MSG=two").expect("rust run");
    check("run_script outputs", Ok(expected), w.outputs_value());
}

// ---------------------------------------------------------------------------
// `GCodeIO`.
// ---------------------------------------------------------------------------

fn pipe_with(data: &[u8]) -> (i32, i32) {
    let (r, w) = kos::fd::pipe().expect("pipe");
    kos::fd::write_all(w, data).expect("write pipe");
    (r, w)
}

fn io_world(fd: i32) -> World {
    build_world(start_args(true, Some(fd), Some("v1")), "Printer is ready", 7.0)
}

fn pyvalue_stats(io: &GCodeIO) -> Value {
    let obj: &dyn PrinterObject = io;
    let (flag, text) = obj.stats(0.0).expect("stats");
    Value::Tuple(vec![Value::Bool(flag), Value::str(&text)])
}

#[test]
fn io_partial_lines_and_stats_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is ready', 7.0)",
    )
    .expect("world");
    py.exec(
        "P.send_event('klippy:ready')",
    )
    .expect("ready");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); _os.write(_w, b'M11'); \
         P.start_args['gcode_fd'] = _r; IO = gmod.GCodeIO(P); \
         IO._process_data(10.0)",
    )
    .expect("py io partial 1");
    py.exec("_os.write(_w, b'5\\nECHO hi\\npart'); IO._process_data(11.5)")
        .expect("py io partial 2");
    let py_stats = py.eval("IO.stats(0.0)").expect("py stats");
    let py_outputs = py.eval("outputs()").expect("py outputs");
    let py_bytes = py.eval("IO.bytes_read").expect("py bytes");
    let py_pending = py.eval("list(IO.pending_commands)").expect("py pending");

    let (r, w) = pipe_with(b"M11");
    let world = io_world(r);
    world.fire_ready().expect("rust ready");
    let io = GCodeIO::new(world.printer.clone()).expect("rust IO");
    io.process_data(10.0);
    kos::fd::write_all(w, b"5\nECHO hi\npart").expect("write more");
    io.process_data(11.5);
    kos::fd::close(w);
    kos::fd::close(r);
    check("io stats", Ok(py_stats), pyvalue_stats(&io));
    check("io outputs", Ok(py_outputs), world.outputs_value());
    check("io bytes", Ok(py_bytes), Value::int(io.bytes_read()));
    check(
        "io pending",
        Ok(py_pending),
        Value::List(io.pending_commands().iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn io_eof_requests_exit_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is ready', 7.0)",
    )
    .expect("world");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); _os.close(_w); \
         P.start_args['gcode_fd'] = _r; IO = gmod.GCodeIO(P); \
         IO._process_data(20.0)",
    )
    .expect("py io eof");
    let py_exits = py.eval("exits()").expect("py exits");
    let py_tool = py.eval("tool_calls()").expect("py tool");
    let py_outputs = py.eval("outputs()").expect("py outputs");

    let (r, w) = kos::fd::pipe().expect("pipe");
    kos::fd::close(w);
    let world = io_world(r);
    world.fire_ready().expect("rust ready");
    let io = GCodeIO::new(world.printer.clone()).expect("rust IO");
    io.process_data(20.0);
    kos::fd::close(r);
    check(
        "eof exits",
        Ok(py_exits),
        Value::List(world.fake.exits().iter().map(|s| Value::str(s)).collect()),
    );
    let (dwell, waits) = world.toolhead.tool_calls();
    check(
        "eof tool",
        Ok(py_tool),
        Value::Tuple(vec![
            Value::List(dwell.iter().map(|f| Value::float(*f)).collect()),
            Value::int(waits),
        ]),
    );
    check("eof outputs", Ok(py_outputs), world.outputs_value());
}

#[test]
fn io_decode_error_is_ignored_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is ready', 7.0)",
    )
    .expect("world");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); _os.write(_w, b'\\xff'); \
         P.start_args['gcode_fd'] = _r; IO = gmod.GCodeIO(P); \
         IO._process_data(30.0)",
    )
    .expect("py io decode");
    let py_stats = py.eval("IO.stats(0.0)").expect("py stats");
    let py_pending = py.eval("list(IO.pending_commands)").expect("py pending");

    let (r, w) = pipe_with(&[0xff]);
    let world = io_world(r);
    world.fire_ready().expect("rust ready");
    let io = GCodeIO::new(world.printer.clone()).expect("rust IO");
    io.process_data(30.0);
    kos::fd::close(w);
    kos::fd::close(r);
    check("decode stats", Ok(py_stats), pyvalue_stats(&io));
    check(
        "decode pending",
        Ok(py_pending),
        Value::List(io.pending_commands().iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn io_m112_out_of_order_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 'Printer is ready', 7.0)",
    )
    .expect("world");
    py.exec("P.send_event('klippy:ready')").expect("ready");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); _os.write(_w, b'G1 X1\\nM112\\n'); \
         P.start_args['gcode_fd'] = _r; IO = gmod.GCodeIO(P); IO._process_data(40.0)",
    )
    .expect("py io m112");
    let py_shutdowns = py.eval("shutdowns()").expect("py shutdowns");
    let py_outputs = py.eval("outputs()").expect("py outputs");

    let (r, w) = pipe_with(b"G1 X1\nM112\n");
    let world = io_world(r);
    world.fire_ready().expect("rust ready");
    let io = GCodeIO::new(world.printer.clone()).expect("rust IO");
    io.process_data(40.0);
    kos::fd::close(w);
    kos::fd::close(r);
    check(
        "io m112 shutdowns",
        Ok(py_shutdowns),
        Value::List(world.fake.shutdowns().iter().map(|s| Value::str(s)).collect()),
    );
    check("io m112 outputs", Ok(py_outputs), world.outputs_value());
}

#[test]
fn io_tty_respond_raw_write_error_deactivates() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'software_version': 'v1'}, \
         'Printer is ready', 7.0)",
    )
    .expect("world without debuginput");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); P.start_args['gcode_fd'] = _r; \
         IO = gmod.GCodeIO(P); IO._respond_raw('hello')",
    )
    .expect("py write err");
    let py_active = py.eval("IO.pipe_is_active").expect("py active");

    let (r, w) = kos::fd::pipe().expect("pipe");
    let fake = Arc::new(FakePrinter::new(
        start_args(false, Some(r), Some("v1")),
        ("Printer is ready", "ready"),
    ));
    let printer: Printer = Arc::clone(&fake) as Printer;
    let toolhead = Arc::new(FakeToolHead::new(7.0));
    printer
        .add_object("toolhead", Arc::clone(&toolhead) as ObjectRef)
        .expect("toolhead");
    let dispatch = Arc::new(GCodeDispatch::new(printer.clone()));
    printer
        .add_object("gcode", Arc::clone(&dispatch) as ObjectRef)
        .expect("gcode");
    let io = GCodeIO::new(printer).expect("rust IO");
    io.respond_raw("hello");
    check(
        "write err active flag",
        Ok(py_active),
        Value::Bool(io.pipe_is_active()),
    );
    kos::fd::close(w);
    kos::fd::close(r);
}

// ---------------------------------------------------------------------------
// Log-line helpers.
// ---------------------------------------------------------------------------

#[test]
fn log_line_helpers_match() {
    let Some(mut py) = oracle() else { return };
    let py_exiting = py
        .eval("'Exiting (print time %.3fs)' % (12.345,)")
        .expect("py exiting");
    check("exiting", Ok(py_exiting), Value::str(&format_exiting(12.345)));
    let Some(mut py) = oracle() else { return };
    py.exec(
        "new_world({'config_file': 'test.cfg', 'debuginput': '/tmp/gcode-in', \
         'software_version': 'v1'}, 's', 0.0)",
    )
    .expect("world");
    py.exec(
        "import os as _os; _r, _w = _os.pipe(); P.start_args['gcode_fd'] = _r; \
         IO = gmod.GCodeIO(P); IO.input_log.append((10.0, 'M115\\n')); \
         IO.input_log.append((11.5, 'part')); IO._handle_analyze_shutdown('x', {})",
    )
    .expect("py dump");
    let expected = py.eval("LOGS[-1]").expect("py dump text");
    let entries = vec![
        (10.0, "M115\n".to_string()),
        (11.5, "part".to_string()),
    ];
    check("input dump", Ok(expected), Value::str(&format_input_dump(&entries)));
}

// ---------------------------------------------------------------------------
// Module factory, object wiring and error class names.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_and_add_early() {
    let factory = klippy_core::gcode::module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_some());
    assert!(factory.add_printer_objects.is_none());
    let w = make_world(true, "s", 0.0);
    let names: Vec<String> = w
        .printer
        .lookup_objects(None)
        .into_iter()
        .map(|(n, _)| n)
        .collect();
    assert!(names.contains(&"toolhead".to_string()));
    assert!(names.contains(&"gcode".to_string()));
    let f = factory.add_early_printer_objects.expect("factory fn");
    let _ = f;
}

#[test]
fn command_error_class_name_matches() {
    let Some(mut py) = oracle() else { return };
    let py_type = py.eval("gmod.CommandError.__name__").expect("py class");
    check(
        "CommandError name",
        Ok(py_type),
        Value::str(KlippyError::Command("x".to_string()).class_name()),
    );
}
