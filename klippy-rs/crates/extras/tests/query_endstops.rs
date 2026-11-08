//! Parity tests for `extras::query_endstops` (port of
//! `klippy/extras/query_endstops.py`).
//!
//! Every expected value comes from the live `pyoracle` reference with
//! fakes mirroring the Rust fakes below; `recording::assert_equal`
//! compares the return value, G-code responses, webhook payloads and
//! `get_status` dicts. When `Oracle::start()` returns `None` (outside the
//! oracle image) the test skips.

use extras::query_endstops::module_factory;
use extras::query_endstops::QueryEndstops;
use indexmap::IndexMap;
use klippy_api::config::{Config, ConfigApi, GetChoiceOpts, GetListsOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts, GetIntOpts, GetOpts, GCodeCommandHandler, GCmd,
    RegisterCommandOpts,
};
use klippy_api::mcu::{Endstop, EndstopApi, Mcu};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{QueryEndstopsApi, Stepper, StepperFactory, Trapq};
use klippy_api::toolhead::{ExtraAxis, HomingTargetApi, Kinematics, PrinterExtruder, ToolHeadApi};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback, WebRequest, WebRequestApi};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Record wrapper for assert_equal.
// ---------------------------------------------------------------------------

fn rec(function: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.query_endstops.QueryEndstops".to_string(),
        scenario: "live".to_string(),
        seq: 0,
        function: function.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: recording::SideEffects::default(),
        object_id: None,
        kind: recording::RecordKind::Python,
    }
}

fn pyvalue_of(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.clone().into_bigint().to_string().parse().unwrap()),
        PyValue::Float(f) => Value::Float(recording::value::FloatVal::new(*f)),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_of).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_of).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_of(k), pyvalue_of(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("Host#{h}")),
    }
}

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

#[derive(Default, Clone)]
struct InnerPrinter {
    objects: IndexMap<String, ObjectRef>,
}

struct FakePrinter {
    inner: Mutex<InnerPrinter>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakePrinter {
            inner: Mutex::new(InnerPrinter::default()),
            reactor,
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
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
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut g = self.inner.lock().unwrap();
        if g.objects.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        g.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let g = self.inner.lock().unwrap();
        if let Some(o) = g.objects.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{}'",
                name
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.inner
            .lock()
            .unwrap()
            .objects
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
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
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
        "query_endstops".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, _option: &str, _opts: klippy_api::config::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, _option: &str, _opts: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: klippy_api::config::GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used by query_endstops")
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

#[derive(Default)]
struct GCodeState {
    commands: IndexMap<String, (Option<GCodeCommandHandler>, Option<String>)>,
    respond_raw: Vec<String>,
}

struct FakeGCode {
    state: Mutex<GCodeState>,
    reactor: Reactor,
}

impl FakeGCode {
    fn new(reactor: Reactor) -> Arc<Self> {
        Arc::new(FakeGCode {
            state: Mutex::new(GCodeState::default()),
            reactor,
        })
    }

    fn dispatch(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut g = self.state.lock().unwrap();
        let (handler, _) = g
            .commands
            .get_mut(cmd)
            .expect("command handler registered");
        let handler = handler.as_mut().expect("handler present");
        handler(gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut g = self.state.lock().unwrap();
        if func.is_none() {
            return Ok(g.commands.shift_remove(cmd).and_then(|(h, _)| h));
        }
        if g.commands.contains_key(cmd) {
            return Err(KlippyError::Config(format!(
                "gcode command {} already registered",
                cmd
            )));
        }
        g.commands
            .insert(cmd.to_string(), (func, opts.desc.clone()));
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
        let g = self.state.lock().unwrap();
        g.commands
            .iter()
            .filter_map(|(k, (_, d))| d.clone().map(|v| (k.clone(), v)))
            .collect()
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
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command not used here")
    }
    fn respond_raw(&self, msg: &str) {
        self.state
            .lock()
            .unwrap()
            .respond_raw
            .push(msg.to_string());
    }
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

#[derive(Default)]
struct WebHooksState {
    endpoints: Vec<(String, WebHooksCallback)>,
}

struct FakeWebHooks {
    state: Mutex<WebHooksState>,
}

impl WebHooksApi for FakeWebHooks {
    fn register_endpoint(&self, path: &str, cb: WebHooksCallback) -> KResult<()> {
        self.state
            .lock()
            .unwrap()
            .endpoints
            .push((path.to_string(), cb));
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        _path: &str,
        _key: &str,
        _value: &str,
        _cb: WebHooksCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

impl FakeWebHooks {
    fn dispatch(&self, path: &str, req: WebRequest) -> KResult<()> {
        let g = self.state.lock().unwrap();
        let (_, cb) = g
            .endpoints
            .iter()
            .find(|(p, _)| p == path)
            .expect("endpoint registered");
        cb(req)
    }
}

struct FakeToolHead {
    last_move_time: f64,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0; 4]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics not used")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        self.last_move_time
    }
    fn dwell(&self, _delay: f64) {}
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<f64>,
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
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq not used")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _a: Option<f64>,
        _b: Option<f64>,
        _c: Option<f64>,
        _d: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(
        &self,
        _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _t: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder not used")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _t: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory not used")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakeWebHooks; dyn WebHooksApi);
klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeEndstop {
    value: i64,
    fail: bool,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu not used")
    }
    fn add_stepper(&self, _stepper: Stepper) {}
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn home_start(
        &self,
        _pt: f64,
        _st: f64,
        _sc: i64,
        _rt: f64,
        _trig: Option<bool>,
    ) -> Completion<bool> {
        panic!("home_start not used")
    }
    fn home_wait(&self, _t: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        if self.fail {
            return Err(KlippyError::Command("mcu error".to_string()));
        }
        Ok(self.value)
    }
}

#[derive(Default)]
struct GCmdState {
    respond_raw: Vec<String>,
}

struct FakeGcmd {
    state: Mutex<GCmdState>,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_int(&self, _name: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn get_float(&self, _name: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn get_command(&self) -> String {
        "QUERY_ENDSTOPS".to_string()
    }
    fn get_commandline(&self) -> String {
        "QUERY_ENDSTOPS".to_string()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, msg: &str) {
        self.state
            .lock()
            .unwrap()
            .respond_raw
            .push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

struct FakeWebRequest {
    sent: Mutex<Option<PyValue>>,
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> klippy_api::webhooks::ClientConnection {
        panic!("client_conn not used")
    }
    fn id(&self) -> Option<PyValue> {
        None
    }
    fn method(&self) -> String {
        "query_endstops/status".to_string()
    }
    fn params(&self) -> PyValue {
        PyValue::None
    }
    fn response(&self) -> Option<PyValue> {
        self.sent.lock().unwrap().clone()
    }
    fn is_error(&self) -> bool {
        false
    }
    fn get_client_connection(&self) -> klippy_api::webhooks::ClientConnection {
        panic!("get_client_connection not used")
    }
    fn get(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_str(&self, _item: &str, _default: Option<String>) -> KResult<String> {
        Ok(String::new())
    }
    fn get_int(&self, _item: &str, _default: Option<i64>) -> KResult<i64> {
        Ok(0)
    }
    fn get_float(&self, _item: &str, _default: Option<f64>) -> KResult<f64> {
        Ok(0.0)
    }
    fn get_dict(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_method(&self) -> String {
        self.method()
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        *self.sent.lock().unwrap() = Some(data);
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        self.sent.lock().unwrap().clone()
    }
}


struct Stack {
    qe: Arc<QueryEndstops>,
    gcode: Arc<FakeGCode>,
    webhooks: Arc<FakeWebHooks>,
}

fn make_stack(print_time: f64) -> Stack {
    let reactor = Reactor::new(false);
    let printer: Printer = FakePrinter::new(reactor.clone());
    let gcode: Arc<FakeGCode> = FakeGCode::new(reactor.clone());
    let webhooks: Arc<FakeWebHooks> = Arc::new(FakeWebHooks {
        state: Mutex::new(WebHooksState::default()),
    });
    let toolhead: Arc<FakeToolHead> = Arc::new(FakeToolHead { last_move_time: print_time });
    let printer2 = printer.clone();
    printer
        .add_object("gcode", gcode.clone() as ObjectRef)
        .unwrap();
    printer
        .add_object("webhooks", webhooks.clone() as ObjectRef)
        .unwrap();
    printer
        .add_object("toolhead", toolhead.clone() as ObjectRef)
        .unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer2 });
    let qe = QueryEndstops::new(&config).unwrap();
    Stack { qe, gcode, webhooks }
}

fn endstop(value: i64) -> Endstop {
    Arc::new(FakeEndstop { value, fail: false })
}

fn endstop_fail() -> Endstop {
    Arc::new(FakeEndstop { value: 0, fail: true })
}

// ---------------------------------------------------------------------------
// Python oracle harness.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import extras.query_endstops as qemod
class ConfigError(Exception):
    pass
class CommandError(Exception):
    pass
class _Sentinel:
    pass
sentinel = _Sentinel()
class PrinterFake:
    def __init__(self):
        self.objects = {}
        self.config_error = ConfigError
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is not sentinel:
            return default
        raise ConfigError("Unknown config object '%s'" % (name,))
class ConfigFake:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer
class WebHooksFake:
    def __init__(self):
        self.endpoints = {}
    def register_endpoint(self, path, cb):
        self.endpoints[path] = cb
class GCodeFake:
    def __init__(self):
        self.commands = {}
        self.help = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            return self.commands.pop(cmd, None)
        if cmd in self.commands:
            raise ConfigError("gcode command %s already registered" % (cmd,))
        self.commands[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
    def get_mutex(self):
        import contextlib
        return contextlib.nullcontext()
class ToolHeadFake:
    def __init__(self, t):
        self.t = t
    def get_last_move_time(self):
        return self.t
class EndstopFake:
    def __init__(self, v):
        self.v = v
    def query_endstop(self, print_time):
        return self.v
class FailEndstop:
    def query_endstop(self, print_time):
        raise CommandError('mcu error')
class GcmdFake:
    def __init__(self):
        self.raw = []
    def respond_raw(self, msg):
        self.raw.append(msg)
class WebRequestFake:
    def __init__(self):
        self.sent = None
    def send(self, data):
        if self.sent is not None:
            raise Exception("Multiple calls to send")
        self.sent = data
printer = PrinterFake()
webhooks = WebHooksFake()
gcode = GCodeFake()
toolhead = ToolHeadFake(7.5)
printer.objects['webhooks'] = webhooks
printer.objects['gcode'] = gcode
printer.objects['toolhead'] = toolhead
config = ConfigFake(printer)
qe = qemod.QueryEndstops(config)
"#;

/// Starts the oracle and execs the Python fakes, so each test drives the
/// reference through the same setup (`qe`, endstop/gcmd/web-request fakes).
fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(PY_SETUP).expect("query_endstops fakes");
    Some(py)
}

#[test]
fn module_factory_loads() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    let ep = py
        .eval("sorted(webhooks.endpoints.keys())")
        .unwrap();
    let cmds = py.eval("sorted(gcode.commands.keys())").unwrap();
    let help = py.eval("gcode.help").unwrap();
    // Rust side: construction registers the same names.
    let st = make_stack(7.5);
    let rust_ep: Vec<Value> = {
        let g = st.webhooks.state.lock().unwrap();
        g.endpoints
            .iter()
            .map(|(s, _)| Value::Str(s.clone()))
            .collect()
    };
    assert_equal(&rec("module_factory"), "path", &ep, &Value::List(rust_ep)).unwrap();
    let cmd_names = {
        let g = st.gcode.state.lock().unwrap();
        let mut names: Vec<String> = g.commands.keys().cloned().collect();
        names.sort();
        names
    };
    let rust_cmds: Vec<Value> = cmd_names
        .iter()
        .map(|s| Value::Str(s.clone()))
        .collect();
    assert_equal(
        &rec("module_factory"),
        "path",
        &cmds,
        &Value::List(rust_cmds),
    )
    .unwrap();
    let rust_help: Vec<(Value, Value)> = st
        .gcode
        .get_command_help()
        .iter()
        .map(|(k, v)| (Value::Str(k.clone()), Value::Str(v.clone())))
        .collect();
    assert_equal(
        &rec("module_factory"),
        "path",
        &help,
        &Value::Dict(rust_help),
    )
    .unwrap();
}

#[test]
fn get_status_empty() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    let expected = py
        .eval("qe.get_status(0.0)")
        .expect("oracle get_status");
    let st = make_stack(7.5);
    let actual = st.qe.get_status(0.0);
    assert_equal(&rec("get_status"), "path", &expected, &pyvalue_of(&actual)).unwrap();
}

#[test]
fn query_endstops_commands_and_status() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    // Seed Python endstops: index 0 open, index 1 TRIGGERED.
    py.exec("ex = EndstopFake(0); ez = EndstopFake(1); qe.register_endstop(ex, 'x'); qe.register_endstop(ez, 'z')").unwrap();
    // QUERY_ENDSTOPS response through the registered handler, via a
    // capturing gcmd.
    py.exec("pygcmd = GcmdFake(); gcode.commands['QUERY_ENDSTOPS'](pygcmd)").unwrap();
    let py_msg = py.eval("pygcmd.raw[0]").unwrap();
    let py_status = py
        .eval("qe.get_status(0.0)")
        .expect("oracle get_status after query");

    let st2 = make_stack(7.5);
    st2.qe.register_endstop(endstop(0), "x");
    st2.qe.register_endstop(endstop(1), "z");
    let fake = Arc::new(FakeGcmd { state: Mutex::new(GCmdState::default()) });
    let trait_gcmd: GCmd = fake.clone();
    st2.gcode.dispatch("QUERY_ENDSTOPS", &trait_gcmd).unwrap();
    let captured = fake.state.lock().unwrap().respond_raw.clone();
    assert_equal(&rec("cmd"), "path", &py_msg, &Value::Str(captured[0].clone())).unwrap();
    let status = st2.qe.get_status(0.0);
    assert_equal(&rec("get_status"), "path", &py_status, &pyvalue_of(&status)).unwrap();
}

#[test]
fn m119_alias_matches_query() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    py.exec("ex = EndstopFake(0); ez = EndstopFake(1); qe.register_endstop(ex, 'x'); qe.register_endstop(ez, 'z')").unwrap();
    // The reference registers the same method for M119; invoke it through
    // the registered `gcode.commands` entry on both sides.
    py.exec("pygcmd = GcmdFake(); gcode.commands['M119'](pygcmd)").unwrap();
    let expected = py.eval("pygcmd.raw[0]").unwrap();
    let st = make_stack(7.5);
    st.qe.register_endstop(endstop(0), "x");
    st.qe.register_endstop(endstop(1), "z");
    let fake = Arc::new(FakeGcmd { state: Mutex::new(GCmdState::default()) });
    let trait_gcmd: GCmd = fake.clone();
    st.gcode.dispatch("M119", &trait_gcmd).unwrap();
    let captured = fake.state.lock().unwrap().respond_raw.clone();
    assert_equal(&rec("m119"), "path", &expected, &Value::Str(captured[0].clone())).unwrap();
}

#[test]
fn web_request_payload() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    py.exec("ex = EndstopFake(0); ez = EndstopFake(1); qe.register_endstop(ex, 'x'); qe.register_endstop(ez, 'z')").unwrap();
    py.exec("pyreq = WebRequestFake(); webhooks.endpoints['query_endstops/status'](pyreq)").unwrap();
    let expected = py.eval("pyreq.sent").expect("oracle web payload");

    let st = make_stack(7.5);
    st.qe.register_endstop(endstop(0), "x");
    st.qe.register_endstop(endstop(1), "z");
    let fake_concrete = Arc::new(FakeWebRequest {
        sent: Mutex::new(None),
    });
    let fake: WebRequest = fake_concrete.clone();
    st.webhooks.dispatch("query_endstops/status", fake).unwrap();
    let got = fake_concrete.sent.lock().unwrap().clone().unwrap();
    assert_equal(&rec("web"), "path", &expected, &pyvalue_of(&got)).unwrap();
}
#[test]
fn empty_query_responds_empty() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    py.exec("pygcmd = GcmdFake(); gcode.commands['QUERY_ENDSTOPS'](pygcmd)").unwrap();
    let expected = py.eval("pygcmd.raw[0]").unwrap();
    let st = make_stack(7.5);
    let fake = Arc::new(FakeGcmd { state: Mutex::new(GCmdState::default()) });
    let trait_gcmd: GCmd = fake.clone();
    st.gcode.dispatch("QUERY_ENDSTOPS", &trait_gcmd).unwrap();
    let captured = fake.state.lock().unwrap().respond_raw.clone();
    assert_equal(&rec("empty"), "path", &expected, &Value::Str(captured[0].clone())).unwrap();
}

#[test]
fn query_endstop_error_preserves_last_state() {
    let Some(mut py) = oracle() else {
        eprintln!("pyoracle skipped: oracle unavailable");
        return;
    };
    // One good endstop, a successful query, then a failing endstop: the
    // next query raises CommandError('mcu error'), emits no response and
    // leaves last_state at the previous successful state.
    py.exec("qe.register_endstop(EndstopFake(1), 'x')").unwrap();
    py.exec("pygcmd = GcmdFake(); gcode.commands['QUERY_ENDSTOPS'](pygcmd)").unwrap();
    py.exec("qe.register_endstop(FailEndstop(), 'z')").unwrap();
    let py_exc = match py.exec("pygcmd2 = GcmdFake(); gcode.commands['QUERY_ENDSTOPS'](pygcmd2)") {
        Err(e) => e,
        Ok(_) => panic!("expected CommandError from failing endstop"),
    };
    let py_raw_len = py.eval("len(pygcmd2.raw)").unwrap();
    let py_status = py
        .eval("qe.get_status(0.0)")
        .expect("oracle get_status after error");

    let st = make_stack(7.5);
    st.qe.register_endstop(endstop(1), "x");
    let fake1 = Arc::new(FakeGcmd { state: Mutex::new(GCmdState::default()) });
    let g1: GCmd = fake1.clone();
    st.gcode.dispatch("QUERY_ENDSTOPS", &g1).unwrap();
    st.qe.register_endstop(endstop_fail(), "z");
    let fake2 = Arc::new(FakeGcmd { state: Mutex::new(GCmdState::default()) });
    let g2: GCmd = fake2.clone();
    let rust_err = st.gcode.dispatch("QUERY_ENDSTOPS", &g2).unwrap_err();
    let captured2 = fake2.state.lock().unwrap().respond_raw.clone();
    assert!(
        captured2.is_empty(),
        "no response emitted on endstop error"
    );
    assert_equal(
        &rec("error"),
        "path",
        &Value::Str(py_exc.class_name.clone()),
        &Value::Str(rust_err.class_name().to_string()),
    )
    .unwrap();
    assert_equal(
        &rec("error"),
        "path",
        &Value::Str(py_exc.message.clone()),
        &Value::Str(rust_err.message().to_string()),
    )
    .unwrap();
    assert_equal(
        &rec("empty"),
        "path",
        &py_raw_len,
        &Value::Int(captured2.len() as i64),
    )
    .unwrap();
    let status = st.qe.get_status(0.0);
    assert_equal(&rec("get_status"), "path", &py_status, &pyvalue_of(&status)).unwrap();
}
