//! Parity tests for `extras::trigger_analog` (port of `klippy/extras/trigger_analog.py`).
//!
//! Expected values come live from the frozen reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The real `trigger_analog` module
//! is imported (with the real `mcu.MCU_trsync` constants) and driven with
//! minimal Python fakes for the MCU, the printer, the sensor and the
//! `TriggerDispatch`. Each fake appends a line to a shared log for every call
//! the module makes (command lookups, `add_config_cmd`, every command sent
//! with its arguments, MCU conversions, dispatch calls); every driven step also
//! logs its return value or the raised exception class and message. The Rust
//! port is driven with the same steps through Rust fakes that render the same
//! lines, and the two logs are compared line by line.
//!
//! Covered: `to_fixed_32` and `calc_frac_bits` (ties, overflow, NaN and
//! infinity), `DigitalFilter` (pre-generated table hits and misses, the
//! SciPy-less error paths of all three error factories, the `signal`
//! NameError in `setup_initial_state`, `pre_gen_filt` and its global table
//! reset), `MCU_SosFilter` (construction, config lookups, section/state/
//! offset-scale/active commands, change tracking, every `ValueError`),
//! `MCU_trigger_analog` (construction, `_build_config`, `_reset_filter`,
//! `_clear_home`, `home_start`, `home_wait` with every stop reason, error map
//! and sensor errors, file output, missing commands before `_build_config`).
//!
//! The SciPy-less checks are skipped when the oracle interpreter has SciPy
//! (the printer does not).

use extras::trigger_analog::*;
use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback,
    EndstopApi, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, QuerySendOpts, SendOpts,
    SerialResponseCallback, TriggerDispatch, TriggerDispatchApi,
};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::Stepper;
use pycompat::text::{float_repr, str_repr};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor};
use recording::value::FloatVal;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python side.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, configparser
# mcu.py imports modules that need a serial port or a C compiler; the
# module under test only needs the MCU_trsync constants.
for _name in ('serialhdl', 'chelper', 'clocksync'):
    try:
        __import__(_name)
    except Exception:
        sys.modules[_name] = types.ModuleType(_name)
import mcu as mcu_mod
import extras.trigger_analog as ta_mod
try:
    import scipy.signal
    HAS_SCIPY = True
except Exception:
    HAS_SCIPY = False

class CommandError(Exception):
    pass

def render(args):
    return '[' + ', '.join([repr(a) for a in args]) + ']'

class World:
    def __init__(self):
        self.log = []
        self.query_response = {'homing_clock': 2000}
        self.enums = {'trigger_analog_error:': {'SOME_ERR': 3, 'SENSOR_SPECIFIC': 100, 'OTHER': 7}}
        self.fileout = False
        self.stop = 1
        self.sps = 400.0
        self.sensor_raise = False
        self.next_oid = 10
W = World()

class Cmd:
    def __init__(self, fmt):
        self.fmt = fmt
    def send(self, args, minclock=0, reqclock=0):
        W.log.append('send %s %s minclock=%d reqclock=%d' % (self.fmt, render(args), minclock, reqclock))

class QueryCmd:
    def __init__(self, fmt):
        self.fmt = fmt
    def send(self, args, minclock=0, reqclock=0, retry=True):
        W.log.append('query %s %s' % (self.fmt, render(args)))
        return dict(W.query_response)

class Printer:
    command_error = CommandError

class Mcu:
    def get_printer(self):
        return PRINTER
    def create_oid(self):
        W.next_oid += 1
        W.log.append('create_oid -> %d' % (W.next_oid,))
        return W.next_oid
    def register_config_callback(self, cb):
        W.log.append('register_config_callback')
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        W.log.append('add_config_cmd %s is_init=%r on_restart=%r' % (cmd, is_init, on_restart))
    def lookup_command(self, msgformat, cq=None):
        W.log.append('lookup_command %s cq=%r' % (msgformat, cq))
        return Cmd(msgformat)
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        W.log.append('lookup_query_command %s | %s oid=%r cq=%r is_async=%r' % (msgformat, respformat, oid, cq, is_async))
        return QueryCmd(msgformat)
    def get_enumerations(self):
        W.log.append('get_enumerations')
        return W.enums
    def is_fileoutput(self):
        W.log.append('is_fileoutput')
        return W.fileout
    def clock32_to_clock64(self, clock32):
        W.log.append('clock32_to_clock64 %r' % (clock32,))
        return clock32 + 1000
    def clock_to_print_time(self, clock):
        W.log.append('clock_to_print_time %r' % (clock,))
        return clock / 12000000.0
    def print_time_to_clock(self, print_time):
        W.log.append('print_time_to_clock %r' % (print_time,))
        return int(print_time * 12000000.0)
    def seconds_to_clock(self, time):
        W.log.append('seconds_to_clock %r' % (time,))
        return int(time * 12000000.0)

class Sensor:
    def get_mcu(self):
        return MCU
    def setup_trigger_analog(self, oid):
        W.log.append('sensor setup_trigger_analog %r' % (oid,))
    def get_samples_per_second(self):
        W.log.append('sensor get_samples_per_second')
        return W.sps
    def lookup_sensor_error(self, error_code):
        W.log.append('sensor lookup_sensor_error %r' % (error_code,))
        if W.sensor_raise:
            raise TypeError('not all arguments converted during string formatting')
        return {0: 'SENSOR_ERR_ZERO', 2: 'SENSOR_ERR_TWO'}.get(error_code, 'Unknown sensor error %d' % (error_code,))

class FakeDispatch:
    def __init__(self, mcu):
        W.log.append('dispatch new')
        self._oid = mcu.create_oid()
    def get_oid(self):
        return self._oid
    def get_command_queue(self):
        return 42
    def get_steppers(self):
        return []
    def start(self, print_time):
        W.log.append('dispatch start %r' % (print_time,))
        return 'completion'
    def wait_end(self, end_time):
        W.log.append('dispatch wait_end %r' % (end_time,))
    def stop(self):
        W.log.append('dispatch stop')
        return W.stop

mcu_mod.TriggerDispatch = FakeDispatch
PRINTER = Printer()
MCU = Mcu()
SENSOR = Sensor()

def step(label, fn):
    try:
        r = fn()
    except Exception as e:
        W.log.append('%s -> raised %s: %s' % (label, type(e).__name__, e))
    else:
        W.log.append('%s -> %r' % (label, r))
"#;

/// Python float literal (`{x}` would print `10` for `10.0`).
fn py_f(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v == f64::INFINITY {
        "float('inf')".to_string()
    } else if v == f64::NEG_INFINITY {
        "float('-inf')".to_string()
    } else {
        format!("{v:?}")
    }
}

fn py_list(v: &[f64]) -> String {
    format!("[{}]", v.iter().map(|x| py_f(*x)).collect::<Vec<_>>().join(", "))
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

// ---------------------------------------------------------------------------
// Rust side fakes. Every call renders the same log line as the Python fake.
// ---------------------------------------------------------------------------

struct World {
    log: Mutex<Vec<String>>,
    query_response: Mutex<PyValue>,
    enums: Mutex<IndexMap<String, IndexMap<String, i64>>>,
    fileout: AtomicBool,
    stop: AtomicI64,
    sps: Mutex<f64>,
    sensor_raise: AtomicBool,
    next_oid: AtomicI64,
}

fn dict_one(key: &str, value: i64) -> PyValue {
    PyValue::Dict(IndexMap::from([(
        PyValue::Str(key.to_string()),
        PyValue::Int(PyInt::from_i64(value)),
    )]))
}

fn default_enums() -> IndexMap<String, IndexMap<String, i64>> {
    IndexMap::from([(
        "trigger_analog_error:".to_string(),
        IndexMap::from([
            ("SOME_ERR".to_string(), 3),
            ("SENSOR_SPECIFIC".to_string(), 100),
            ("OTHER".to_string(), 7),
        ]),
    )])
}

impl World {
    fn new() -> Arc<World> {
        Arc::new(World {
            log: Mutex::new(Vec::new()),
            query_response: Mutex::new(dict_one("homing_clock", 2000)),
            enums: Mutex::new(default_enums()),
            fileout: AtomicBool::new(false),
            stop: AtomicI64::new(1),
            sps: Mutex::new(400.0),
            sensor_raise: AtomicBool::new(false),
            next_oid: AtomicI64::new(10),
        })
    }
    fn log(&self, line: impl Into<String>) {
        self.log.lock().unwrap().push(line.into());
    }
    fn lines(&self) -> Vec<String> {
        self.log.lock().unwrap().clone()
    }
}

/// Python `repr` of the argument kinds the module sends.
fn r_val(v: &PyValue) -> String {
    match v {
        PyValue::Int(n) => n.inner().to_string(),
        PyValue::Float(f) => float_repr(*f),
        PyValue::Bool(b) => py_bool(*b).to_string(),
        PyValue::Str(s) => str_repr(s),
        PyValue::None => "None".to_string(),
        other => panic!("unexpected command argument {other:?}"),
    }
}

fn r_args(args: &[PyValue]) -> String {
    format!("[{}]", args.iter().map(r_val).collect::<Vec<_>>().join(", "))
}

fn r_floats(v: &[f64]) -> String {
    format!("[{}]", v.iter().map(|x| float_repr(*x)).collect::<Vec<_>>().join(", "))
}

fn r_sections(v: &[Vec<f64>]) -> String {
    format!("[{}]", v.iter().map(|s| r_floats(s)).collect::<Vec<_>>().join(", "))
}

fn r_opt_i64(v: Option<i64>) -> String {
    v.map(|x| x.to_string()).unwrap_or_else(|| "None".to_string())
}

fn step<T>(w: &World, label: &str, r: KResult<T>, render: impl FnOnce(T) -> String) {
    match r {
        Ok(v) => w.log(format!("{label} -> {}", render(v))),
        Err(e) => w.log(format!("{label} -> raised {}: {}", e.class_name(), e.message())),
    }
}

fn none(_: ()) -> String {
    "None".to_string()
}

struct FakeCmd {
    fmt: String,
    w: Arc<World>,
}

impl FakeCmd {
    fn record(&self, data: &[PyValue], opts: SendOpts) {
        self.w.log(format!(
            "send {} {} minclock={} reqclock={}",
            self.fmt,
            r_args(data),
            opts.minclock,
            opts.reqclock
        ));
    }
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.record(&data, opts);
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.record(&data, opts);
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQueryCmd {
    fmt: String,
    w: Arc<World>,
}

impl CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, _opts: QuerySendOpts) -> KResult<PyValue> {
        self.w.log(format!("query {} {}", self.fmt, r_args(&data)));
        Ok(self.w.query_response.lock().unwrap().clone())
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.send(data, opts)
    }
}

struct FakePrinterObj;

impl PrinterApi for FakePrinterObj {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".into())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: klippy_api::object::ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        _name: &str,
        _default: LookupDefault,
    ) -> KResult<Option<klippy_api::object::ObjectRef>> {
        Ok(None)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, klippy_api::object::ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &klippy_api::config::Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<klippy_api::object::ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[klippy_api::object::EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeMcu {
    w: Arc<World>,
    printer: Printer,
}

impl FakeMcu {
    fn new(w: Arc<World>) -> Arc<FakeMcu> {
        Arc::new(FakeMcu { w, printer: Arc::new(FakePrinterObj) })
    }
}

fn r_cq(cq: Option<CommandQueue>) -> String {
    r_opt_i64(cq.map(|c| c.0))
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".into()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        self.w.log("is_fileoutput");
        self.w.fileout.load(Ordering::SeqCst)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin not used".into()))
    }
    fn create_oid(&self) -> KResult<i64> {
        let n = self.w.next_oid.fetch_add(1, Ordering::SeqCst) + 1;
        self.w.log(format!("create_oid -> {n}"));
        Ok(n)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        self.w.log("register_config_callback");
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.w.log(format!(
            "add_config_cmd {cmd} is_init={} on_restart={}",
            py_bool(opts.is_init),
            py_bool(opts.on_restart)
        ));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        self.w.log(format!("seconds_to_clock {}", float_repr(time)));
        (time * 12000000.0) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        self.w.log(format!("lookup_command {msgformat} cq={}", r_cq(cq)));
        Ok(Arc::new(FakeCmd { fmt: msgformat.to_string(), w: self.w.clone() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.w.log(format!(
            "lookup_query_command {msgformat} | {respformat} oid={} cq={} is_async={}",
            r_opt_i64(opts.oid),
            r_cq(opts.cq),
            py_bool(opts.is_async)
        ));
        Ok(Arc::new(FakeQueryCmd { fmt: msgformat.to_string(), w: self.w.clone() }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(42)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        struct Noop;
        impl AsyncResponseWrapperApi for Noop {
            fn unregister(&self) {}
        }
        Arc::new(Noop)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.w.log("get_enumerations");
        self.w.enums.lock().unwrap().clone()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(KlippyError::Mcu("no constants".into()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        self.w.log(format!("print_time_to_clock {}", float_repr(print_time)));
        (print_time * 12000000.0) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        // The reference passes the int `clock32_to_clock64` returned.
        self.w.log(format!("clock_to_print_time {}", clock as i64));
        clock / 12000000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        self.w.log(format!("clock32_to_clock64 {clock32}"));
        clock32 + 1000
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        self.w.log("dispatch new");
        let oid = self.create_oid()?;
        Ok(Arc::new(FakeDispatchObj { oid, w: self.w.clone() }))
    }
}

struct FakeDispatchObj {
    oid: i64,
    w: Arc<World>,
}

impl TriggerDispatchApi for FakeDispatchObj {
    fn get_oid(&self) -> i64 {
        self.oid
    }
    fn get_command_queue(&self) -> CommandQueue {
        CommandQueue(42)
    }
    fn add_stepper(&self, _stepper: Stepper) -> KResult<()> {
        Ok(())
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn start(&self, print_time: f64) -> Completion<bool> {
        self.w.log(format!("dispatch start {}", float_repr(print_time)));
        Completion::new(Reactor::new(false))
    }
    fn wait_end(&self, end_time: f64) {
        self.w.log(format!("dispatch wait_end {}", float_repr(end_time)));
    }
    fn stop(&self) -> KResult<i64> {
        self.w.log("dispatch stop");
        Ok(self.w.stop.load(Ordering::SeqCst))
    }
}

struct FakeSensor {
    w: Arc<World>,
    mcu: Mcu,
}

impl TriggerAnalogSensor for FakeSensor {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_trigger_analog(&self, trigger_analog_oid: i64) -> KResult<()> {
        self.w.log(format!("sensor setup_trigger_analog {trigger_analog_oid}"));
        Ok(())
    }
    fn get_samples_per_second(&self) -> f64 {
        self.w.log("sensor get_samples_per_second");
        *self.w.sps.lock().unwrap()
    }
    fn lookup_sensor_error(&self, error_code: i64) -> KResult<String> {
        self.w.log(format!("sensor lookup_sensor_error {error_code}"));
        if self.w.sensor_raise.load(Ordering::SeqCst) {
            return Err(KlippyError::Other {
                class_name: "TypeError".to_string(),
                message: "not all arguments converted during string formatting".to_string(),
            });
        }
        Ok(match error_code {
            0 => "SENSOR_ERR_ZERO".to_string(),
            2 => "SENSOR_ERR_TWO".to_string(),
            _ => format!("Unknown sensor error {error_code}"),
        })
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

/// The tests share the process-global `GeneratedSOS` table (`pre_gen_filt`
/// clears it), so they run one at a time.
static LOCK: Mutex<()> = Mutex::new(());

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(SETUP).expect("trigger_analog fakes");
    Some(py)
}

fn has_scipy(py: &mut pyoracle::Oracle) -> bool {
    match py.eval("HAS_SCIPY") {
        Ok(Value::Bool(b)) => b,
        other => panic!("HAS_SCIPY: {other:?}"),
    }
}

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "trigger_analog".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "test".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record_for(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

/// Compares the oracle's `W.log` with the Rust fakes' log.
fn compare_logs(py: &mut pyoracle::Oracle, label: &str, rust: Vec<String>) {
    let expected = py.eval("W.log");
    check(label, expected, Value::List(rust.into_iter().map(Value::Str).collect()));
}

/// Compares a Python expression with a Rust result (value or error).
fn compare_expr(py: &mut pyoracle::Oracle, label: &str, expr: &str, actual: KResult<Value>) {
    let expected = py.eval(expr);
    match (expected, actual) {
        (Ok(e), Ok(a)) => check(label, Ok(e), a),
        (Err(e), Err(a)) => {
            assert_eq!(a.class_name(), e.class_name, "{label}: class");
            assert_eq!(a.message(), e.message, "{label}: message");
        }
        (Ok(e), Err(a)) => panic!("{label}: oracle returned {e:?}, port raised {a:?}"),
        (Err(e), Ok(a)) => panic!("{label}: oracle raised {}: {}, port returned {a:?}", e.class_name, e.message),
    }
}

// ---------------------------------------------------------------------------
// Fixed point helpers.
// ---------------------------------------------------------------------------

#[test]
fn fixed_point_helpers_match_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let nan = f64::NAN;
    let cases: Vec<(f64, i64)> = vec![
        (1.5, 4),
        (-2.25, 8),
        (0.0009334986129548442, 20),
        (100.0, 0),
        (-0.5, 1),
        (0.5, 0),
        (1.5, 0),
        (2.5, 0),
        (3.5, 0),
        (-0.5, 0),
        (-1.5, 0),
        (-2.5, 0),
        (0.25, 1),
        (0.75, 1),
        (2147483647.0, 0),
        (-2147483648.0, 0),
        (2147483647.5, 0),
        (2147483648.0, 0),
        (-2147483648.5, 0),
        (-2147483649.0, 0),
        (1e20, 20),
        (1e300, 31),
        (1.7e308, 31),
        (0.9999999999, 31),
        (-1.0, 31),
        (nan, 0),
        (f64::INFINITY, 0),
        (f64::NEG_INFINITY, 4),
    ];
    for (v, fb) in cases {
        let label = format!("to_fixed_32({}, {fb})", float_repr(v));
        let expr = format!("ta_mod.to_fixed_32({}, {fb})", py_f(v));
        compare_expr(&mut py, &label, &expr, to_fixed_32(v, fb).map(Value::Int));
    }
    for (v, fb) in [(5i64, 3i64), (-5, 3), (2147483647, 7), (2147483648, 7), (-2147483648, 0), (-2147483649, 0)] {
        let label = format!("assert_is_int32({v}, {fb})");
        let expr = format!("ta_mod.assert_is_int32({v}, {fb})");
        compare_expr(&mut py, &label, &expr, assert_is_int32(v, fb).map(Value::Int));
    }
    let lists: Vec<Vec<f64>> = vec![
        vec![],
        vec![1.0, 2.0],
        vec![1.0, 0.5],
        vec![0.001, -1.365],
        vec![1000.0],
        vec![0.1],
        vec![0.0],
        vec![-0.0],
        vec![0.5],
        vec![-3.75, 2.5],
        vec![1e-12],
        vec![0.9999999999],
        vec![3.9999999999, 0.5],
        vec![-3.9999999999, 0.5],
        vec![2147483647.9],
        vec![2147483648.5, 0.25],
        vec![1e30, 0.5],
        vec![1e30],
        vec![1.0, nan],
        vec![nan, 0.5],
        vec![0.5, f64::INFINITY],
        vec![f64::NEG_INFINITY],
        vec![0.0009334986129548442, 0.0018669972259096883, 0.0009334986129548442, 1.0, -1.3651172372392975, 0.4775922500725171],
        vec![1.0, 2.0, 1.0, 1.0, -1.6117270964574348, 0.7445208382054344],
    ];
    for vals in lists {
        let label = format!("calc_frac_bits({vals:?})");
        let expr = format!("ta_mod.calc_frac_bits({})", py_list(&vals));
        compare_expr(&mut py, &label, &expr, calc_frac_bits(&vals).map(Value::Int));
    }
}

// ---------------------------------------------------------------------------
// DigitalFilter.
// ---------------------------------------------------------------------------

#[test]
fn digital_filter_matches_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    let scipy = has_scipy(&mut py);
    let w = World::new();
    let mut code = String::from(
        "Err = configparser.Error
df = ta_mod.DigitalFilter(400.0, Err)
step('add_lowpass(25, 4)', lambda: df.add_lowpass(25.0, 4))
step('sections', lambda: [list(s) for s in df.get_filter_sections()])
step('add_derivative', df.add_derivative)
step('get_size', df.get_size)
step('initial_state', df.get_initial_state)
step('sample_frequency', lambda: df.sample_frequency)
step('add_lowpass(0, 4)', lambda: df.add_lowpass(0.0, 4))
step('add_highpass(0, 2)', lambda: df.add_highpass(0.0, 2))
step('setup_initial_state', df.setup_initial_state)
step('initial_state after setup', df.get_initial_state)
df800 = ta_mod.DigitalFilter(800.0, Err)
step('800/50 lowpass', lambda: df800.add_lowpass(50.0, 4))
step('800 sections', lambda: [list(s) for s in df800.get_filter_sections()])
empty = ta_mod.DigitalFilter(400.0, Err)
step('empty setup_initial_state', empty.setup_initial_state)
step('empty initial_state', empty.get_initial_state)
step('empty size', empty.get_size)
step('empty sections', empty.get_filter_sections)
",
    );
    if !scipy {
        code.push_str(
            "step('add_highpass(10, 2)', lambda: df.add_highpass(10.0, 2))
step('add_lowpass(25, 3)', lambda: df.add_lowpass(25.0, 3))
step('add_notch', lambda: df.add_notch(50.0, 2.0))
step('get_scipy_signal', df.get_scipy_signal)
cmd = ta_mod.DigitalFilter(400.0, CommandError)
step('command add_highpass', lambda: cmd.add_highpass(10.0, 2))
step('command add_notch', lambda: cmd.add_notch(50.0, 2.0))
imp = ta_mod.DigitalFilter(400.0, ImportError)
step('import add_lowpass(25, 3)', lambda: imp.add_lowpass(25.0, 3))
step('800/50 highpass', lambda: df800.add_highpass(50.0, 4))
step('pre_gen_filt', lambda: ta_mod.pre_gen_filt('lowpass', 400, 25, 4))
step('lowpass after pre_gen_filt', lambda: ta_mod.DigitalFilter(400.0, Err).add_lowpass(25.0, 4))
",
        );
    }
    py.exec(&code).expect("python digital filter scenario");

    let df = DigitalFilter::new(400.0, CfgErrorKind::Config);
    step(&w, "add_lowpass(25, 4)", df.add_lowpass(25.0, 4), none);
    step(&w, "sections", Ok(df.get_filter_sections()), |s| r_sections(&s));
    df.add_derivative();
    w.log("add_derivative -> None");
    step(&w, "get_size", Ok(df.get_size() as i64), |n| n.to_string());
    step(&w, "initial_state", Ok(df.get_initial_state()), |s| r_sections(&s));
    step(&w, "sample_frequency", Ok(df.sample_frequency.get()), float_repr);
    step(&w, "add_lowpass(0, 4)", df.add_lowpass(0.0, 4), none);
    step(&w, "add_highpass(0, 2)", df.add_highpass(0.0, 2), none);
    step(&w, "setup_initial_state", df.setup_initial_state(), none);
    step(&w, "initial_state after setup", Ok(df.get_initial_state()), |s| r_sections(&s));
    let df800 = DigitalFilter::new(800.0, CfgErrorKind::Config);
    step(&w, "800/50 lowpass", df800.add_lowpass(50.0, 4), none);
    step(&w, "800 sections", Ok(df800.get_filter_sections()), |s| r_sections(&s));
    let empty = DigitalFilter::new(400.0, CfgErrorKind::Config);
    step(&w, "empty setup_initial_state", empty.setup_initial_state(), none);
    step(&w, "empty initial_state", Ok(empty.get_initial_state()), |s| r_sections(&s));
    step(&w, "empty size", Ok(empty.get_size() as i64), |n| n.to_string());
    step(&w, "empty sections", Ok(empty.get_filter_sections()), |s| r_sections(&s));
    if !scipy {
        step(&w, "add_highpass(10, 2)", df.add_highpass(10.0, 2), none);
        step(&w, "add_lowpass(25, 3)", df.add_lowpass(25.0, 3), none);
        step(&w, "add_notch", df.add_notch(50.0, 2.0), none);
        step(&w, "get_scipy_signal", df.get_scipy_signal(), none);
        let cmd = DigitalFilter::new(400.0, CfgErrorKind::Command);
        step(&w, "command add_highpass", cmd.add_highpass(10.0, 2), none);
        step(&w, "command add_notch", cmd.add_notch(50.0, 2.0), none);
        let imp = DigitalFilter::new_import_error(400.0);
        step(&w, "import add_lowpass(25, 3)", imp.add_lowpass(25.0, 3), none);
        step(&w, "800/50 highpass", df800.add_highpass(50.0, 4), none);
        step(&w, "pre_gen_filt", pre_gen_filt("lowpass", 400.0, 25.0, 4), |_| "None".to_string());
        let after = DigitalFilter::new(400.0, CfgErrorKind::Config);
        step(&w, "lowpass after pre_gen_filt", after.add_lowpass(25.0, 4), none);
        reset_generated_sos();
    }
    compare_logs(&mut py, "digital filter", w.lines());
}

// ---------------------------------------------------------------------------
// MCU_SosFilter.
// ---------------------------------------------------------------------------

enum DStep {
    Lowpass(f64, i64),
    Derivative,
    Raw(Vec<f64>),
    State(Vec<Vec<f64>>),
}

enum SOp {
    Design(Vec<DStep>),
    NoDesign,
    Start(f64),
    Scale(i64, f64, bool),
    Reset,
}

fn py_sos_op(op: &SOp) -> String {
    match op {
        SOp::Design(steps) => {
            let mut s = String::from("D = ta_mod.DigitalFilter(400.0, configparser.Error)\n");
            for st in steps {
                match st {
                    DStep::Lowpass(f, o) => {
                        s += &format!("step('lowpass', lambda: D.add_lowpass({}, {o}))\n", py_f(*f));
                    }
                    DStep::Derivative => s += "D.add_derivative()\n",
                    DStep::Raw(v) => s += &format!("D.filter_sections.append({})\n", py_list(v)),
                    DStep::State(v) => {
                        let rows: Vec<String> = v.iter().map(|r| py_list(r)).collect();
                        s += &format!("D.initial_state = [{}]\n", rows.join(", "));
                    }
                }
            }
            s + "sos.set_filter_design(D)\n"
        }
        SOp::NoDesign => "sos.set_filter_design(None)\n".to_string(),
        SOp::Start(v) => format!("sos.set_start_state({})\n", py_f(*v)),
        SOp::Scale(off, scale, auto) => {
            format!("sos.set_offset_scale({off}, {}, {})\n", py_f(*scale), py_bool(*auto))
        }
        SOp::Reset => "step('reset_filter', sos.reset_filter)\n".to_string(),
    }
}

fn rust_sos_op(w: &World, sos: &Arc<MCU_SosFilter>, op: &SOp) {
    match op {
        SOp::Design(steps) => {
            let d = DigitalFilter::new(400.0, CfgErrorKind::Config);
            for st in steps {
                match st {
                    DStep::Lowpass(f, o) => step(w, "lowpass", d.add_lowpass(*f, *o), none),
                    DStep::Derivative => d.add_derivative(),
                    DStep::Raw(v) => d.filter_sections.borrow_mut().push(v.clone()),
                    DStep::State(v) => *d.initial_state.borrow_mut() = Some(v.clone()),
                }
            }
            sos.set_filter_design(Some(Arc::new(d)));
        }
        SOp::NoDesign => sos.set_filter_design(None),
        SOp::Start(v) => sos.set_start_state(*v),
        SOp::Scale(off, scale, auto) => sos.set_offset_scale(*off, *scale, *auto),
        SOp::Reset => step(w, "reset_filter", sos.reset_filter(), none),
    }
}

fn run_sos(py: &mut pyoracle::Oracle, label: &str, max_sections: i64, ops: &[SOp]) {
    let mut code = format!(
        "W = World()\nsos = ta_mod.MCU_SosFilter(MCU, 42, {max_sections})\nsos._build_config()\n"
    );
    for op in ops {
        code += &py_sos_op(op);
    }
    py.exec(&code).unwrap_or_else(|e| panic!("{label}: python raised {}: {}", e.class_name, e.message));

    let w = World::new();
    let mcu = FakeMcu::new(w.clone());
    let mcu_handle: Mcu = mcu;
    let sos = MCU_SosFilter::new(mcu_handle, CommandQueue(42), max_sections).expect("MCU_SosFilter::new");
    sos.build_config().expect("sos build_config");
    for op in ops {
        rust_sos_op(&w, &sos, op);
    }
    compare_logs(py, label, w.lines());
}

fn lowpass() -> DStep {
    DStep::Lowpass(25.0, 4)
}

#[test]
fn sos_filter_matches_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    // Change tracking: coefficients and offset/scale are only resent when
    // they changed, the state and the activation always.
    run_sos(
        &mut py,
        "tracking",
        4,
        &[
            SOp::Design(vec![lowpass(), DStep::Derivative]),
            SOp::Start(3.0),
            SOp::Scale(-12, 0.5, false),
            SOp::Reset,
            SOp::Reset,
            SOp::Scale(-12, 0.5, true),
            SOp::Reset,
            SOp::Scale(-12, 0.5, false),
            SOp::Reset,
            SOp::Scale(7, 3.3, false),
            SOp::Reset,
            SOp::Design(vec![lowpass()]),
            SOp::Reset,
            SOp::NoDesign,
            SOp::Reset,
        ],
    );
    // Rounding of the state and of the scale.
    run_sos(
        &mut py,
        "state and scale rounding",
        4,
        &[
            SOp::Design(vec![lowpass(), DStep::State(vec![vec![0.5, -0.25], vec![1.5, 2.0]])]),
            SOp::Start(3.0),
            SOp::Scale(0, 1.0, false),
            SOp::Reset,
            SOp::Scale(0, 1e-5, false),
            SOp::Reset,
            SOp::Scale(0, 0.75, false),
            SOp::Reset,
            SOp::Scale(0, 2.5e9, false),
            SOp::Reset,
            SOp::Scale(0, 3.9999999999, false),
            SOp::Reset,
            SOp::Scale(0, f64::NAN, false),
            SOp::Reset,
            SOp::Scale(0, f64::INFINITY, false),
            SOp::Reset,
            SOp::Start(f64::NAN),
            SOp::Scale(1, 0.5, false),
            SOp::Reset,
        ],
    );
    // Section limits.
    run_sos(
        &mut py,
        "max sections 2",
        2,
        &[
            SOp::Design(vec![lowpass(), DStep::Derivative]),
            SOp::Reset,
            SOp::Design(vec![lowpass()]),
            SOp::Reset,
        ],
    );
    run_sos(
        &mut py,
        "max sections 0",
        0,
        &[SOp::NoDesign, SOp::Reset, SOp::Design(vec![lowpass()]), SOp::Reset],
    );
    // Malformed designs.
    run_sos(
        &mut py,
        "malformed designs",
        4,
        &[
            SOp::Design(vec![lowpass(), DStep::State(vec![vec![0.0, 0.0]])]),
            SOp::Reset,
            SOp::Design(vec![lowpass(), DStep::State(vec![vec![0.0, 0.0, 0.0], vec![0.0, 0.0]])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![1.0, 2.0, 3.0])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![1.0, 0.0, 0.0, 2.0, 0.0, 0.0])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![1.0, 0.0, 0.0, 1.0, 0.0, 0.0])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![1.5, 0.25, 0.125, 1.0, -1.75, 0.9])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![3.9999999999, 0.5, 0.25, 1.0, 0.5, 0.1])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![f64::NAN, 0.5, 0.25, 1.0, 0.5, 0.1])]),
            SOp::Reset,
            SOp::Design(vec![DStep::Raw(vec![5000000000.0, 0.5, 0.25, 1.0, 0.5, 0.1])]),
            SOp::Reset,
        ],
    );
}

// ---------------------------------------------------------------------------
// MCU_trigger_analog.
// ---------------------------------------------------------------------------

enum TV {
    Int(i64),
    Float(f64),
}

enum TOp {
    New,
    NewWithSos(i64),
    Build,
    Oids,
    Range(i64, i64),
    Trigger(&'static str, TV),
    Query(Option<i64>),
    Stop(i64),
    File(bool),
    Sps(f64),
    Raise(bool),
    ResetFilter,
    ClearHome,
    HomeStart(f64),
    HomeStartEndstop(f64),
    HomeWait(f64),
    Steppers,
    QueryEndstop,
}

fn py_trigger_op(op: &TOp) -> String {
    match op {
        TOp::New => "ta = ta_mod.MCU_trigger_analog(SENSOR)\n".to_string(),
        TOp::NewWithSos(n) => format!(
            "ta.setup_sos_filter(ta_mod.MCU_SosFilter(MCU, ta.get_dispatch().get_command_queue(), {n}))\n"
        ),
        TOp::Build => "step('build', lambda: (ta._build_config(), ta._sos_filter._build_config())[0])\n"
            .to_string(),
        TOp::Oids => "step('oids', lambda: [ta.get_oid(), ta.get_dispatch().get_oid(), ta.get_sos_filter().get_oid()])\n"
            .to_string(),
        TOp::Range(a, b) => format!("ta.set_raw_range({a}, {b})\n"),
        TOp::Trigger(kind, TV::Int(v)) => format!("ta.set_trigger('{kind}', {v})\n"),
        TOp::Trigger(kind, TV::Float(v)) => format!("ta.set_trigger('{kind}', {})\n", py_f(*v)),
        TOp::Query(Some(c)) => format!("W.query_response = {{'homing_clock': {c}}}\n"),
        TOp::Query(None) => "W.query_response = {}\n".to_string(),
        TOp::Stop(n) => format!("W.stop = {n}\n"),
        TOp::File(b) => format!("W.fileout = {}\n", py_bool(*b)),
        TOp::Sps(v) => format!("W.sps = {}\n", py_f(*v)),
        TOp::Raise(b) => format!("W.sensor_raise = {}\n", py_bool(*b)),
        TOp::ResetFilter => "step('_reset_filter', ta._reset_filter)\n".to_string(),
        TOp::ClearHome => "step('_clear_home', ta._clear_home)\n".to_string(),
        TOp::HomeStart(t) | TOp::HomeStartEndstop(t) => {
            format!("step('home_start', lambda: ta.home_start({}, 0., 0, 0.))\n", py_f(*t))
        }
        TOp::HomeWait(t) => format!(
            "step('home_wait', lambda: ta.home_wait({}))\nstep('last_trigger_time', ta.get_last_trigger_time)\n",
            py_f(*t)
        ),
        TOp::Steppers => "step('get_steppers', lambda: len(ta.get_steppers()))\n".to_string(),
        TOp::QueryEndstop => "step('query_endstop', lambda: ta.query_endstop(1.0))\n".to_string(),
    }
}

struct Rig {
    w: Arc<World>,
    mcu: Mcu,
    sensor: Arc<dyn TriggerAnalogSensor>,
    ta: Option<Arc<MCU_trigger_analog>>,
}

fn completion(_: Completion<bool>) -> String {
    "'completion'".to_string()
}

fn rust_trigger_op(rig: &mut Rig, op: &TOp) {
    let w = rig.w.clone();
    match op {
        TOp::New => {
            rig.ta = Some(MCU_trigger_analog::new(rig.sensor.clone()).expect("MCU_trigger_analog::new"));
            return;
        }
        TOp::Query(Some(c)) => {
            *w.query_response.lock().unwrap() = dict_one("homing_clock", *c);
            return;
        }
        TOp::Query(None) => {
            *w.query_response.lock().unwrap() = PyValue::Dict(IndexMap::new());
            return;
        }
        TOp::Stop(n) => {
            w.stop.store(*n, Ordering::SeqCst);
            return;
        }
        TOp::File(b) => {
            w.fileout.store(*b, Ordering::SeqCst);
            return;
        }
        TOp::Sps(v) => {
            *w.sps.lock().unwrap() = *v;
            return;
        }
        TOp::Raise(b) => {
            w.sensor_raise.store(*b, Ordering::SeqCst);
            return;
        }
        _ => {}
    }
    let ta = rig.ta.clone().expect("New must come first");
    match op {
        TOp::NewWithSos(n) => {
            let cq = ta.get_dispatch().get_command_queue();
            let sos = MCU_SosFilter::new(rig.mcu.clone(), cq, *n).expect("MCU_SosFilter::new");
            ta.setup_sos_filter(sos);
        }
        TOp::Build => {
            let r = ta.build_config().and_then(|()| {
                ta.get_sos_filter()
                    .ok_or_else(|| KlippyError::Other {
                        class_name: "AttributeError".to_string(),
                        message: "'NoneType' object has no attribute '_build_config'".to_string(),
                    })?
                    .build_config()
            });
            step(&w, "build", r, none);
        }
        TOp::Oids => {
            let sos = ta.get_sos_filter().expect("sos filter");
            let oids = vec![ta.get_oid(), ta.get_dispatch().get_oid(), sos.get_oid()];
            step(&w, "oids", Ok(oids), |o| {
                format!("[{}]", o.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "))
            });
        }
        TOp::Range(a, b) => ta.set_raw_range(*a, *b),
        TOp::Trigger(kind, TV::Int(v)) => {
            ta.set_trigger(kind.to_string(), PyValue::Int(PyInt::from_i64(*v)))
        }
        TOp::Trigger(kind, TV::Float(v)) => ta.set_trigger(kind.to_string(), PyValue::Float(*v)),
        TOp::ResetFilter => step(&w, "_reset_filter", ta.reset_filter(), none),
        TOp::ClearHome => step(&w, "_clear_home", ta.clear_home(), float_repr),
        TOp::HomeStart(t) => step(&w, "home_start", ta.home_start(*t), completion),
        TOp::HomeStartEndstop(t) => {
            // The same call through the endstop contract the probes use.
            let endstop: Arc<dyn EndstopApi> = ta.clone();
            let c = endstop.home_start(*t, 0.0, 0, 0.0, None);
            w.log(format!("home_start -> {}", completion(c)));
        }
        TOp::HomeWait(t) => {
            step(&w, "home_wait", ta.home_wait(*t), float_repr);
            step(&w, "last_trigger_time", Ok(ta.get_last_trigger_time()), float_repr);
        }
        TOp::Steppers => step(&w, "get_steppers", Ok(ta.get_steppers().len() as i64), |n| n.to_string()),
        TOp::QueryEndstop => {
            let endstop: Arc<dyn EndstopApi> = ta.clone();
            step(&w, "query_endstop", endstop.query_endstop(1.0), |v| v.to_string());
        }
        TOp::New
        | TOp::Query(_)
        | TOp::Stop(_)
        | TOp::File(_)
        | TOp::Sps(_)
        | TOp::Raise(_) => unreachable!("handled above"),
    }
}

fn run_trigger(py: &mut pyoracle::Oracle, label: &str, enums: bool, ops: &[TOp]) {
    let mut code = String::from("W = World()\n");
    if !enums {
        code += "W.enums = {}\n";
    }
    for op in ops {
        code += &py_trigger_op(op);
    }
    py.exec(&code).unwrap_or_else(|e| panic!("{label}: python raised {}: {}", e.class_name, e.message));

    let w = World::new();
    if !enums {
        *w.enums.lock().unwrap() = IndexMap::new();
    }
    let mcu: Mcu = FakeMcu::new(w.clone());
    let sensor: Arc<dyn TriggerAnalogSensor> = Arc::new(FakeSensor { w: w.clone(), mcu: mcu.clone() });
    let mut rig = Rig { w: w.clone(), mcu, sensor, ta: None };
    for op in ops {
        rust_trigger_op(&mut rig, op);
    }
    compare_logs(py, label, w.lines());
}

#[test]
fn trigger_homing_flow_matches_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    use TOp::*;
    run_trigger(
        &mut py,
        "homing flow",
        true,
        &[
            New,
            Build,
            Oids,
            Steppers,
            Range(0, 1000),
            Trigger("abs_ge", TV::Int(75)),
            HomeStart(1.5),
            HomeStart(1.5),
            // 75 == 75.0: the change check sees no change.
            Trigger("abs_ge", TV::Float(75.0)),
            HomeStart(2.0),
            Trigger("gt", TV::Int(5)),
            Range(-5, 10),
            HomeStart(2.25),
            HomeStartEndstop(2.5),
            Stop(1),
            HomeWait(3.0),
            Stop(2),
            HomeStart(2.5),
            HomeWait(3.5),
            Stop(3),
            HomeWait(4.0),
            Stop(4),
            HomeWait(4.5),
            Stop(5),
            HomeWait(5.0),
            Stop(8),
            HomeWait(5.5),
            Stop(12),
            HomeWait(6.0),
            Stop(105),
            HomeWait(6.5),
            Stop(107),
            HomeWait(7.0),
            Stop(106),
            HomeWait(7.5),
            Raise(true),
            HomeWait(8.0),
            Raise(false),
            Stop(1),
            File(true),
            HomeWait(8.5),
            File(false),
            Query(None),
            HomeWait(9.0),
            ClearHome,
            Query(Some(4000000000)),
            ClearHome,
            HomeWait(9.5),
            QueryEndstop,
        ],
    );
    // Without the `trigger_analog_error:` enumeration every error code is a
    // sensor error.
    run_trigger(
        &mut py,
        "no error enumeration",
        false,
        &[New, Build, Stop(5), HomeWait(1.0), Stop(8), HomeWait(2.0), Stop(4), HomeWait(3.0)],
    );
    // The caller installs its own SOS filter before `_build_config`.
    run_trigger(
        &mut py,
        "caller sos filter",
        true,
        &[
            New,
            NewWithSos(4),
            Build,
            Oids,
            Trigger("diff_peak_gt", TV::Int(1234)),
            Range(0, 16777215),
            HomeStart(0.5),
            HomeStart(0.75),
        ],
    );
    // Zero sample rate.
    run_trigger(&mut py, "zero sps", true, &[New, Build, Sps(0.0), HomeStart(1.0), Sps(12.0), HomeStart(1.0)]);
}

#[test]
fn trigger_before_build_config_matches_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    use TOp::*;
    run_trigger(
        &mut py,
        "commands before _build_config",
        true,
        &[New, HomeStart(1.0), ResetFilter, ClearHome, HomeWait(2.0), Range(1, 2), ResetFilter],
    );
}

#[test]
fn module_factory_is_empty() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn constants_match_python() {
    let _guard = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let Some(mut py) = oracle() else {
        println!("skipping pyoracle: set KLIPPY_PYTHON to run against the reference");
        return;
    };
    check(
        "REASON_TRIGGER_ANALOG",
        py.eval("ta_mod.MCU_trigger_analog.REASON_TRIGGER_ANALOG"),
        Value::Int(REASON_TRIGGER_ANALOG),
    );
    check("MONITOR_MAX", py.eval("ta_mod.MCU_trigger_analog.MONITOR_MAX"), Value::Int(MONITOR_MAX));
    check("MAX_INT32", py.eval("ta_mod.MAX_INT32"), Value::Int(MAX_INT32));
    check("MIN_INT32", py.eval("ta_mod.MIN_INT32"), Value::Int(MIN_INT32));
    let generated = py.eval("[list(s) for s in ta_mod.GeneratedSOS[('lowpass', 400.0/25.0, 4)]]");
    let ours = DigitalFilter::new(400.0, CfgErrorKind::Config);
    ours.add_lowpass(25.0, 4).expect("table hit");
    let actual: Vec<Value> = ours
        .get_filter_sections()
        .iter()
        .map(|s| Value::List(s.iter().map(|f| Value::Float(FloatVal::new(*f))).collect()))
        .collect();
    check("GeneratedSOS table", generated, Value::List(actual));
}
