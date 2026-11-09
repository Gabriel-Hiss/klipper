//! Parity tests for `extras::motion_report` (port of `klippy/extras/motion_report.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`). A `PrinterMotionReport` is built
//! on both sides with minimal Python/Rust fakes for the printer, config,
//! `gcode`, `webhooks`, `motion_queuing`, `toolhead`, `mcu` and
//! `MCU_stepper`, then fed identical stepper-history / trapq inputs.
//! Returned values, error classes/messages, batch payload dicts,
//! `get_status` dicts, batch streaming through the real
//! `bulk_sensor.BatchBulkHelper`, and shutdown log lines are compared.
//!
//! - trapq handles are core `TrapqHandle` instances wrapping the chelper
//!   trapq, exactly as `motion_queuing.allocate_trapq` produces, and
//!   Python drives the same C trapq through `chelper`.
//! - Coordinates use the real `gcode.Coord`, not a stub class.

use extras::motion_report::{
    DumpStepper, DumpTrapQ, PrinterMotionReport, NEVER_TIME, STATUS_REFRESH_TIME,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, GCmd, OutputCallback, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, McuApi, Pin, PinParams, SerialResponseCallback,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, Stepper, StepperKinematics, Trapq,
};
use klippy_api::stepper::StepperFactory;
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, Kinematics, MotionQueuingApi, MoveApi, PrinterExtruder, ToolHeadApi, TrapqAppend, FlushCallback, SyncEmitterHandle,
};
use klippy_api::log::{self, Record as LogRecord};
use klippy_api::webhooks::{ClientConnectionApi, WebHooksApi, WebHooksCallback, WebRequest, WebRequestApi};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, LazyLock, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser, logging
import gcode
from gcode import Coord
import extras.motion_report as motion_report
from extras.motion_report import DumpStepper, DumpTrapQ, PrinterMotionReport

LOG_CAPTURE = []
class _LogCapture(logging.Handler):
    def emit(self, record):
        LOG_CAPTURE.append((record.levelname, record.getMessage()))
logging.getLogger().addHandler(_LogCapture())
logging.getLogger().setLevel(logging.DEBUG)
def py_logs():
    return list(LOG_CAPTURE)
def py_clear_logs():
    LOG_CAPTURE.clear()

class FakeReactor:
    def monotonic(self):
        return 1000.0
    def register_timer(self, cb, wake):
        return None
    def unregister_timer(self, timer):
        pass

class FakeGcode:
    Coord = Coord
    def register_mux_command(self, *a, **k):
        pass

class FakeWebhooks:
    def __init__(self):
        self.mux = []
    def register_mux_endpoint(self, path, key, value, cb):
        self.mux.append((path, key, value, cb))

class FakeClientConnection:
    def __init__(self, closed=False):
        self._closed = closed
        self.sent = []
    def is_closed(self):
        return self._closed
    def send(self, msg):
        self.sent.append(msg)

class FakeWebRequest:
    def __init__(self, params=None, conn=None):
        self.params = params if params is not None else {}
        self.conn = conn if conn is not None else FakeClientConnection()
        self.sent = []
    def get_client_connection(self):
        return self.conn
    def get_dict(self, item, default=None):
        return self.params.get(item, default)
    def send(self, data):
        self.sent.append(data)

class FakeMotionQueuing:
    def __init__(self, drip=None):
        self._drip = drip
    def check_drip_timing(self):
        return self._drip

class FakeMcu:
    def __init__(self, name='mcu'):
        self._name = name
    def get_name(self):
        return self._name
    def clock_to_print_time(self, clock):
        return clock / 1000.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000.0)
    def estimated_print_time(self, eventtime):
        return eventtime

class FakeMcuStepper:
    def __init__(self, name, mcu, steps=(), step_dist=0.01):
        self._name = name
        self._mcu = mcu
        self._steps = list(steps)
        self._step_dist = step_dist
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        return self._name
    def get_step_dist(self):
        return self._step_dist
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist
    def dump_steps(self, count, start_clock, end_clock):
        batch = self._steps[:128]
        self._steps = self._steps[128:]
        return (batch, len(batch))

class FakeStep:
    def __init__(self, first_clock, last_clock, start_position, interval, step_count, add):
        self.first_clock=first_clock; self.last_clock=last_clock; self.start_position=start_position; self.interval=interval; self.step_count=step_count; self.add=add

class FakeToolhead:
    Coord = Coord
    def __init__(self, trapq=None, extra_axes=None):
        self._trapq = trapq
        self._extra_axes = extra_axes if extra_axes is not None else [None, None, None]
    def get_trapq(self):
        return self._trapq
    def get_extra_axes(self):
        return list(self._extra_axes)

class FakeExtruder:
    def __init__(self, trapq=None, name='extruder'):
        self._trapq = trapq
        self._name = name
    def get_name(self):
        return self._name
    def get_trapq(self):
        return self._trapq

sentinel = object()
class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self._reactor = FakeReactor()
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        prefix = module + ' '
        objs = [(n, self.objects[n]) for n in self.objects if n.startswith(prefix)]
        if module in self.objects:
            return [(module, self.objects[module])] + objs
        return objs
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def get_reactor(self):
        return self._reactor

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name='motion_report'):
        self._printer = printer
        self._name = name
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
    handlers: Arc<Mutex<IndexMap<String, Vec<EventCallback>>>>,
}

impl FakePrinter {
    fn new() -> (Self, Printer) {
        let fake = Self {
            objects: Arc::new(Mutex::new(IndexMap::new())),
            handlers: Arc::new(Mutex::new(IndexMap::new())),
        };
        let printer: Printer = Arc::new(fake.clone());
        (fake, printer)
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut m = self.objects.lock().unwrap();
        if m.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        m.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        if let Some(o) = m.get(name) { return Ok(Some(o.clone())); }
        drop(m);
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let m = self.objects.lock().unwrap();
            match module {
                None => m.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
                Some(modname) => {
                    let prefix = format!("{modname} ");
                    let objs: Vec<(String, ObjectRef)> = m.iter()
                        .filter(|(n, _)| n.starts_with(&prefix))
                        .map(|(k, v)| (k.clone(), v.clone()))
                        .collect();
                    if let Some(exact) = m.get(modname) {
                        let mut out = vec![(modname.to_string(), exact.clone())];
                        out.extend(objs);
                        out
                    } else { objs }
                }
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(&self, _c: &Config, _s: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> { Ok(None) }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, cb: EventCallback) {
        self.handlers.lock().unwrap().entry(event.to_string()).or_default().push(cb);
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _r: &str) {}
}

struct FakeConfig { printer: Printer }
impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { "motion_report".to_string() }
    fn get(&self, _o: &str, _opts: GetStrOpts) -> KResult<Option<String>> { Ok(None) }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, _o: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> { Ok(None) }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, _s: &str) -> Config { panic!("getsection unused in motion_report tests") }
    fn has_section(&self, _s: &str) -> bool { false }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// GCode / WebHooks / MotionQueuing / Mcu / Stepper / Toolhead fakes.
struct FakeGcode;
impl GCodeApi for FakeGcode {
    fn register_command(&self, _c: &str, _f: Option<GCodeCommandHandler>, _o: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> { Ok(None) }
    fn register_mux_command(&self, _c: &str, _k: &str, _v: Option<&str>, _f: GCodeCommandHandler, _d: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _e: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn register_output_handler(&self, _c: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { Reactor::new(false).mutex(false) }
    fn create_gcode_command(&self, _c: &str, _cl: &str, _p: &IndexMap<String, String>) -> GCmd { panic!("unused") }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool { false }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> { Ok(()) }
}
klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

struct FakeWebhooks { mux: Mutex<Vec<(String, String, String, WebHooksCallback)>> }
impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _p: &str, _c: WebHooksCallback) -> KResult<()> { Ok(()) }
    fn register_mux_endpoint(&self, path: &str, key: &str, value: &str, c: WebHooksCallback) -> KResult<()> {
        self.mux.lock().unwrap().push((path.to_string(), key.to_string(), value.to_string(), c));
        Ok(())
    }
    fn call_remote_method(&self, _m: &str, _p: &IndexMap<String, PyValue>) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
}
klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

#[derive(Default)]
struct ConnState {
    closed: bool,
    sent: Vec<PyValue>,
}

struct FakeClientConn {
    state: Arc<Mutex<ConnState>>,
}

impl ClientConnectionApi for FakeClientConn {
    fn uid(&self) -> u64 { panic!("uid is not used by motion_report") }
    fn dump_request_log(&self) { panic!("dump_request_log is not used by motion_report") }
    fn set_client_info(&self, _client_info: PyValue, _state_msg: Option<String>) {
        panic!("set_client_info is not used by motion_report")
    }
    fn close(&self) { panic!("close is not used by motion_report") }
    fn is_closed(&self) -> bool { self.state.lock().unwrap().closed }
    fn send(&self, data: PyValue) { self.state.lock().unwrap().sent.push(data); }
}

struct FakeWebRequest {
    conn: klippy_api::webhooks::ClientConnection,
    params: PyValue,
    sent: Arc<Mutex<Vec<PyValue>>>,
}

impl FakeWebRequest {
    fn build(params: PyValue, conn_closed: bool) -> (WebRequest, Arc<Mutex<ConnState>>, Arc<Mutex<Vec<PyValue>>>) {
        let state = Arc::new(Mutex::new(ConnState { closed: conn_closed, sent: Vec::new() }));
        let sent = Arc::new(Mutex::new(Vec::new()));
        let obj: WebRequest = Arc::new(FakeWebRequest {
            conn: Arc::new(FakeClientConn { state: state.clone() }),
            params,
            sent: sent.clone(),
        });
        (obj, state, sent)
    }
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> klippy_api::webhooks::ClientConnection { panic!("client_conn is not used by motion_report") }
    fn id(&self) -> Option<PyValue> { panic!("id is not used by motion_report") }
    fn method(&self) -> String { panic!("method is not used by motion_report") }
    fn params(&self) -> PyValue { panic!("params is not used by motion_report") }
    fn response(&self) -> Option<PyValue> { panic!("response is not used by motion_report") }
    fn is_error(&self) -> bool { panic!("is_error is not used by motion_report") }
    fn get_client_connection(&self) -> klippy_api::webhooks::ClientConnection { self.conn.clone() }
    fn get(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> { panic!("get is not used by motion_report") }
    fn get_str(&self, _item: &str, _default: Option<String>) -> KResult<String> { panic!("get_str is not used by motion_report") }
    fn get_int(&self, _item: &str, _default: Option<i64>) -> KResult<i64> { panic!("get_int is not used by motion_report") }
    fn get_float(&self, _item: &str, _default: Option<f64>) -> KResult<f64> { panic!("get_float is not used by motion_report") }
    fn get_dict(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        if let PyValue::Dict(dict) = &self.params {
            let key = PyValue::Str(item.to_string());
            if let Some(v) = dict.get(&key) { return Ok(v.clone()); }
        }
        Ok(default.unwrap_or(PyValue::None))
    }
    fn get_method(&self) -> String { panic!("get_method is not used by motion_report") }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        self.sent.lock().unwrap().push(data);
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> { panic!("finish is not used by motion_report") }
}

// Log capture: the sink is installed once per process and records into a
// per-thread buffer, so tests running in parallel only see their own lines.
thread_local! {
    static LOGS: std::cell::RefCell<Vec<(String, String)>> = const { std::cell::RefCell::new(Vec::new()) };
}
static LOG_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

fn install_log_sink() {
    log::set_sink(Box::new(|rec: &LogRecord<'_>| {
        LOGS.with(|logs| logs.borrow_mut().push((rec.level.name().to_string(), rec.message.to_string())));
    }));
}

fn lock_logs() -> std::sync::MutexGuard<'static, ()> {
    let guard = LOG_LOCK.lock().unwrap();
    install_log_sink();
    LOGS.with(|logs| logs.borrow_mut().clear());
    guard
}

fn take_logs() -> Vec<(String, String)> {
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

fn logs_value() -> Value {
    Value::List(
        take_logs()
            .into_iter()
            .map(|(level, message)| {
                Value::Tuple(vec![Value::str(&level), Value::str(&message)])
            })
            .collect(),
    )
}

struct FakeMotionQueuing { drip: Option<f64> }
impl MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> Trapq { panic!("unused") }
    fn lookup_trapq_append(&self) -> TrapqAppend { panic!("unused") }
    fn wipe_trapq(&self, _t: &Trapq) {}
    fn allocate_syncemitter(&self, _m: klippy_api::mcu::Mcu, _n: &str, _a: bool) -> KResult<SyncEmitterHandle> { panic!("unused") }
    fn queue_syncemitter_msg(&self, _s: &SyncEmitterHandle, _c: u64, _d: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: klippy_api::mcu::Mcu, _s: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _c: i64) {}
    fn register_flush_callback(&self, _c: FlushCallback, _a: bool) {}
    fn unregister_flush_callback(&self, _c: &FlushCallback) {}
    fn register_flush_callback_named(&self, _c: FlushCallback, _a: bool, _n: Option<&str>, _o: Option<&str>) {
    }
    fn get_kin_flush_delay(&self) -> f64 { 0.0 }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> { Ok(()) }
    fn calc_step_gen_restart(&self, _e: f64) -> f64 { 0.0 }
    fn note_mcu_movequeue_activity(&self, _t: f64, _s: bool) {}
    fn drip_update_time(&self, _s: f64, _e: f64, _c: &Completion<bool>) -> KResult<()> { Ok(()) }
    fn check_drip_timing(&self) -> Option<f64> { self.drip }
}
klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

struct FakeMcu { name: String, printer: Printer }
impl McuApi for FakeMcu {
    fn get_name(&self) -> String { self.name.clone() }
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn is_fileoutput(&self) -> bool { false }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> { panic!("unused") }
    fn create_oid(&self) -> KResult<i64> { Ok(1) }
    fn register_config_callback(&self, _c: ConfigCallback) -> KResult<()> { Ok(()) }
    fn add_config_cmd(&self, _c: &str, _o: AddConfigCmdOpts) -> KResult<()> { Ok(()) }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _o: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, time: f64) -> i64 { (time * 1000.0) as i64 }
    fn min_schedule_time(&self) -> f64 { 0.0 }
    fn max_nominal_duration(&self) -> f64 { 0.0 }
    fn lookup_command(&self, _m: &str, _q: Option<CommandQueue>) -> KResult<CommandWrapper> { panic!("unused") }
    fn lookup_query_command(&self, _m: &str, _r: &str, _o: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> { panic!("unused") }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(0) }
    fn register_serial_response(&self, _c: SerialResponseCallback, _m: &str, _o: Option<i64>) -> AsyncResponseWrapper { panic!("unused") }
    fn check_valid_response(&self, _m: &str) -> bool { true }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> { Ok(0.0) }
    fn print_time_to_clock(&self, print_time: f64) -> i64 { (print_time * 1000.0) as i64 }
    fn clock_to_print_time(&self, clock: f64) -> f64 { clock / 1000.0 }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { eventtime }
    fn clock32_to_clock64(&self, c: i64) -> i64 { c }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> { Ok((0.0, 0.0)) }
    fn get_status(&self, _e: Option<f64>) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> { panic!("unused") }
}
klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeStepper {
    name: String,
    mcu: klippy_api::mcu::Mcu,
    batches: Mutex<Vec<(Vec<PullHistoryStep>, i64)>>,
    step_dist: f64,
}
impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { self.mcu.clone() }
    fn get_name(&self, _s: Option<bool>) -> String { self.name.clone() }
    fn get_oid(&self) -> i64 { 1 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _p: f64, _s: bool) {}
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 { self.step_dist }
    fn get_rotation_distance(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) { (false, false) }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 { 0.0 }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 { 0.0 }
    fn get_mcu_position(&self, _p: Option<f64>) -> i64 { 0 }
    fn get_past_mcu_position(&self, _p: f64) -> i64 { 0 }
    fn mcu_to_commanded_position(&self, mcu_pos: f64) -> f64 { mcu_pos * self.step_dist }
    fn dump_steps(&self, _count: i64, _s: i64, _e: i64) -> (Vec<PullHistoryStep>, i64) {
        let mut b = self.batches.lock().unwrap();
        if b.is_empty() { return (Vec::new(), 0); }
        b.remove(0)
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> { None }
    fn set_stepper_kinematics(&self, _s: Option<StepperKinematics>) -> Option<StepperKinematics> { None }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<Trapq> { None }
    fn set_trapq(&self, _t: Option<Trapq>) -> Option<Trapq> { None }
    fn add_active_callback(&self, _c: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _a: char) -> bool { false }
}
klippy_api::printer_object!(FakeStepper; dyn McuStepperApi);



#[allow(dead_code)]
struct FakeToolhead {
    trapq: Trapq,
    extra_axes: Vec<Option<ExtraAxis>>,
}
impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> { vec![0.0, 0.0, 0.0, 0.0] }
    fn set_position(&self, _p: Vec<f64>, _h: &str) -> KResult<()> { Ok(()) }
    fn get_kinematics(&self) -> Kinematics { panic!("unused") }
    fn flush_step_generation(&self) -> KResult<()> { Ok(()) }
    fn get_last_move_time(&self) -> KResult<f64> { Ok(0.0) }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _p: Vec<f64>, _s: f64, _c: &Completion<bool>) -> KResult<()> { Ok(()) }
}
impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _p: Vec<f64>, _s: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> { Ok(PyValue::Dict(IndexMap::new())) }
    fn get_trapq(&self) -> Trapq { self.trapq.clone() }
    fn get_max_velocity(&self) -> (f64, f64) { (0.0, 0.0) }
    fn set_max_velocities(&self, _a: Option<f64>, _b: Option<f64>, _c: Option<f64>, _d: Option<f64>) -> (f64, f64, f64, f64) { (0.0, 0.0, 0.0, 0.0) }
    fn register_lookahead_callback(&self, _c: klippy_api::toolhead::LookaheadCallback) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder { panic!("unused") }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> { self.extra_axes.clone() }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> StepperFactory { panic!("unused") }
}
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

#[allow(dead_code)]
struct FakeExtruder { name: String, trapq: Option<Trapq> }
impl ExtraAxisApi for FakeExtruder {
    fn get_name(&self) -> String { self.name.clone() }
    fn get_axis_gcode_id(&self) -> Option<String> { Some("E".to_string()) }
    fn get_trapq(&self) -> Option<Trapq> { self.trapq.clone() }
    fn check_move(&self, _m: &dyn MoveApi, _i: usize) -> KResult<()> { Ok(()) }
    fn calc_junction(&self, _p: &dyn MoveApi, _m: &dyn MoveApi, _i: usize) -> klippy_api::KResult<f64> { Ok(0.0) }
    fn process_move(&self, _p: f64, _m: &dyn MoveApi, _i: usize) {}
}
klippy_api::printer_object!(FakeExtruder; dyn ExtraAxisApi);

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => { py.exec(SETUP).expect("motion_report fakes"); Some(py) }
        None => { println!("skipping motion_report oracle tests: set KLIPPY_PYTHON"); None }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected { Ok(v) => v, Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message) };
    let record = Record { v: 1, unit: "motion_report".to_string(), scenario: label.to_string(), seq: 0, function: "test".to_string(), args: Vec::new(), ret: None, exc: None, side_effects: SideEffects::default(), object_id: None, kind: RecordKind::Python };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual) { panic!("{label}: {d}"); }
}

fn pyvalue_to_record(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap()),
        PyValue::Float(f) => Value::Float(recording::value::FloatVal::new(*f)),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_record).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_record).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, vv)| (pyvalue_to_record(k), pyvalue_to_record(vv))).collect()),
        PyValue::Host(h) => Value::Opaque { type_name: "host".into(), addr: format!("0x{h:x}") },
    }
}

fn hist_step(first: i64, last: i64, start: i64, interval: i64, count: i64, add: i64) -> PullHistoryStep {
    PullHistoryStep { first_clock: first, last_clock: last, start_position: start, interval, step_count: count, add }
}

fn pyvalues_to_record(v: &[PyValue]) -> Value {
    Value::List(v.iter().map(pyvalue_to_record).collect())
}

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    let err = match actual {
        Ok(_) => panic!("{label}: rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(err.message(), oracle_exc.message.as_str(), "{label}: exception message");
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check("NEVER_TIME", py.eval("motion_report.NEVER_TIME"), Value::float(NEVER_TIME));
    check("STATUS_REFRESH_TIME", py.eval("motion_report.STATUS_REFRESH_TIME"), Value::float(STATUS_REFRESH_TIME));
}

#[test]
fn initial_status_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("p = FakePrinter(); p.add_object('gcode', FakeGcode()); p.add_object('webhooks', FakeWebhooks()); p.add_object('motion_queuing', FakeMotionQueuing()); c = FakeConfig(p); r = motion_report.load_config(c)").expect("py load_config");
    let expected = py.eval("r.last_status");
    // Rust side
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let report = PrinterMotionReport::new(&config).expect("rust load_config");
    let rs_status = report.get_status_value(0.0).expect("status");
    check("initial_status", expected.clone(), pyvalue_to_record(&rs_status));
    // Factory returns an object.
    let factory = extras::motion_report::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
}

#[test]
fn get_step_queue_reversal_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
class FakeStep:
    def __init__(self, first_clock, last_clock, start_position, interval, step_count, add):
        self.first_clock=first_clock; self.last_clock=last_clock; self.start_position=start_position; self.interval=interval; self.step_count=step_count; self.add=add
class FakeStepperScripted:
    def __init__(self):
        self.calls = 0
        self._batches = [([FakeStep(500,510,40,4,1,1), FakeStep(400,410,30,3,1,1)], 2), ([FakeStep(300,310,20,2,1,1), FakeStep(200,210,10,1,1,1), FakeStep(100,110,0,0,1,1)], 3)]
    def dump_steps(self, count, start_clock, end_clock):
        if self.calls >= len(self._batches):
            return ([], 0)
        b, c = self._batches[self.calls]
        self.calls += 1
        return (b, c)
    def get_mcu(self):
        return FakeMcu()
    def get_name(self, short=False):
        return 'stepper_x'
ds = DumpStepper.__new__(DumpStepper)
ds.printer = None
ds.mcu_stepper = FakeStepperScripted()
ds.last_batch_clock = 0
flat, batches = ds.get_step_queue(0, 10000)
res_first = flat[0].first_clock
res_len = len(flat)
"#).expect("py get_step_queue");
    let expected_first = py.eval("res_first");
    let expected_len = py.eval("res_len");
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    let mcu: klippy_api::mcu::Mcu = Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() });
    let stepper: Stepper = Arc::new(FakeStepper {
        name: "stepper_x".into(), mcu: mcu.clone(),
        batches: Mutex::new(vec![
            (vec![hist_step(500, 510, 40, 4, 1, 1), hist_step(400, 410, 30, 3, 1, 1)], 2),
            (vec![hist_step(300, 310, 20, 2, 1, 1), hist_step(200, 210, 10, 1, 1, 1), hist_step(100, 110, 0, 0, 1, 1)], 3),
        ]),
        step_dist: 0.01,
    });
    let ds = DumpStepper::try_new(printer.clone(), stepper).expect("dump stepper");
    let (flat, _batches) = ds.get_step_queue(0, 10000);
    check("get_step_queue_first", expected_first, Value::int(flat[0].first_clock));
    check("get_step_queue_len", expected_len, Value::int(flat.len() as i64));
}

#[test]
fn trapq_position_matches_python() {
    let Some(mut py) = oracle() else { return };
    // Drive the reference `DumpTrapQ.get_trapq_position` against real
    // chelper with two moves. At print_time 3.5 the reference
    // (`trapq_extract_old`, max 1) returns the newest move, not the oldest.
    py.exec(r#"
import chelper
_ffi_main, _ffi_lib = chelper.get_ffi()
_tq = _ffi_lib.trapq_alloc()
_ffi_lib.trapq_append(_tq, 1.0, 2.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 5.0, 5.0, 2.0)
_ffi_lib.trapq_append(_tq, 3.0, 2.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, 1.0, 0.0, 4.0, 4.0, 1.0)
_dt = DumpTrapQ.__new__(DumpTrapQ)
_dt.trapq = _tq
_pos, _vel = _dt.get_trapq_position(3.5)
py_pos = _pos
py_vel = _vel
"#).expect("py trapq position");
    let exp_pos = py.eval("py_pos");
    let exp_vel = py.eval("py_vel");
    let handle: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
    {
        let h = klippy_core::motion_handles::trapq_of(&handle).expect("trapq_of");
        let mut ct = h.shared().lock();
        ct.append(1.0, 2.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 5.0, 5.0, 2.0);
        ct.append(3.0, 2.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, 1.0, 0.0, 4.0, 4.0, 1.0);
    }
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let dt = DumpTrapQ::try_new(printer.clone(), "toolhead".into(), Some(handle)).expect("dump trapq");
    let (pos, vel) = dt.get_trapq_position(3.5);
    let pos = pos.expect("position");
    let vel = vel.expect("velocity");
    check("trapq_pos", exp_pos.clone(), Value::Tuple(vec![Value::float(pos[0]), Value::float(pos[1]), Value::float(pos[2])]));
    check("trapq_vel", exp_vel.clone(), Value::float(vel));
}

#[test]
fn factory_loads_working_object() {
    let Some(mut py) = oracle() else { return };
    py.exec("p = FakePrinter(); p.add_object('gcode', FakeGcode()); p.add_object('webhooks', FakeWebhooks()); p.add_object('motion_queuing', FakeMotionQueuing()); c = FakeConfig(p); r = motion_report.load_config(c)").expect("py factory");
    let factory = extras::motion_report::module_factory();
    let load = factory.load_config.expect("load_config");
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer });
    let obj = load(&config).expect("load").expect("object");
    let report: Arc<PrinterMotionReport> = klippy_api::downcast_arc(&obj).expect("downcast");
    check("factory_status", py.eval("r.last_status"), pyvalue_to_record(&report.get_status_value(0.0).expect("status")));
    // Unknown-event dispatch is a no-op on both sides.
    report.handle_analyze_shutdown(&[]).expect("noop");
}

#[test]
fn shutdown_and_stepcompress_filtering() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
r = PrinterMotionReport.__new__(PrinterMotionReport)
r.steppers = {}
r.dtrapqs = {}
# Mirror _handle_analyze_shutdown early-return on missing fields.
def drain(msg, details):
    r = PrinterMotionReport.__new__(PrinterMotionReport)
    r.steppers = {}; r.dtrapqs = {}; r.printer = FakePrinter()
    PrinterMotionReport._handle_analyze_shutdown(r, msg, details)
    return 'ok'
res1 = drain('MCU shutdown', {'reason': 'x'})
res2 = drain('Internal error in stepcompress', {'queue_name': 'q'})
res3 = drain('unknown', {})
"#).expect("py shutdown noop");
    let e1 = py.eval("res1");
    let e2 = py.eval("res2");
    let e3 = py.eval("res3");
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let report = PrinterMotionReport::new(&config).expect("report");
    let mk = |msg: &str, details: &IndexMap<PyValue, PyValue>| -> KResult<()> {
        report.handle_analyze_shutdown(&[EventArg::Value(PyValue::Str(msg.to_string())), EventArg::Value(PyValue::Dict(details.clone()))])
    };
    let mut d1 = IndexMap::new(); d1.insert(PyValue::Str("reason".into()), PyValue::Str("x".into()));
    check("shutdown_missing_fields", e1, Value::str(match mk("MCU shutdown", &d1) { Ok(()) => "ok", _ => "err" }));
    let mut d2 = IndexMap::new(); d2.insert(PyValue::Str("queue_name".into()), PyValue::Str("q".into()));
    check("stepcompress_missing_time", e2, Value::str(match mk("Internal error in stepcompress", &d2) { Ok(()) => "ok", _ => "err" }));
    check("unknown_msg", e3, Value::str(match mk("unknown", &IndexMap::new()) { Ok(()) => "ok", _ => "err" }));
}

#[test]
fn stepper_batch_matches_python() {
    let Some(mut py) = oracle() else { return };
    // A zero step_count in the middle truncates after the marker keeping it.
    py.exec(r#"
m = FakeMcu()
mk = FakeMcuStepper('stepper_x', m, [FakeStep(100,110,0,5,2,1), FakeStep(200,210,10,6,0,0), FakeStep(300,310,20,7,3,-1)], 0.02)
ds = DumpStepper.__new__(DumpStepper)
ds.printer = None
ds.mcu_stepper = mk
ds.last_batch_clock = 0
p1 = ds._process_batch(0.)
p2 = ds._process_batch(0.)
clock = ds.last_batch_clock
"#).expect("py stepper batch");
    let exp_p1 = py.eval("p1");
    let exp_p2 = py.eval("p2");
    let exp_clock = py.eval("clock");
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    let mcu: klippy_api::mcu::Mcu = Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() });
    let stepper: Stepper = Arc::new(FakeStepper {
        name: "stepper_x".into(), mcu: mcu.clone(),
        batches: Mutex::new(vec![(
            vec![hist_step(100, 110, 0, 5, 2, 1), hist_step(200, 210, 10, 6, 0, 0), hist_step(300, 310, 20, 7, 3, -1)],
            3,
        )]),
        step_dist: 0.02,
    });
    let ds = DumpStepper::try_new(printer.clone(), stepper).expect("dump stepper");
    let p1 = ds.process_batch(0.);
    check("stepper_batch_payload", exp_p1, pyvalue_to_record(&p1));
    let p2 = ds.process_batch(0.);
    check("stepper_batch_empty", exp_p2, pyvalue_to_record(&p2));
    check("stepper_batch_clock", exp_clock, Value::int(ds.last_batch_clock()));
}

#[test]
fn trapq_batch_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
import chelper
_fm, _fl = chelper.get_ffi()
def _mk_tq():
    tq = _fl.trapq_alloc()
    _fl.trapq_append(tq, 1.0, 2.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 0.0, 0.0, 0.0)
    _fl.trapq_append(tq, 3.0, 2.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
    return tq
_tq = _mk_tq()
_dq = DumpTrapQ.__new__(DumpTrapQ)
_dq.printer = None; _dq.name = 'toolhead'; _dq.trapq = _tq; _dq.last_batch_msg = (0., 0.); _dq.motion_queuing = FakeMotionQueuing(None)
pa = _dq._process_batch(0.)
pb = _dq._process_batch(0.)
_tq2 = _mk_tq()
_dq2 = DumpTrapQ.__new__(DumpTrapQ)
_dq2.printer = None; _dq2.name = 'toolhead'; _dq2.trapq = _tq2; _dq2.last_batch_msg = (0., 0.); _dq2.motion_queuing = FakeMotionQueuing(4.0)
pc = _dq2._process_batch(0.)
"#).expect("py trapq batch");
    let exp_a = py.eval("pa");
    let exp_b = py.eval("pb");
    let exp_c = py.eval("pc");
    fn make_handle() -> Trapq {
        let handle: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
        {
            let h = klippy_core::motion_handles::trapq_of(&handle).expect("trapq_of");
            let mut ct = h.shared().lock();
            ct.append(1.0, 2.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 0.0, 0.0, 0.0);
            ct.append(3.0, 2.0, 0.0, 0.0, 10.0, 20.0, 30.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        }
        handle
    }
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let dt = DumpTrapQ::try_new(printer.clone(), "toolhead".into(), Some(make_handle())).expect("dump trapq");
    let pa = dt.process_batch(0.);
    check("trapq_batch_payload", exp_a, pyvalue_to_record(&pa));
    let pb = dt.process_batch(0.);
    check("trapq_batch_second", exp_b, pyvalue_to_record(&pb));
    let (_fp2, printer2) = FakePrinter::new();
    printer2.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer2.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: Some(4.0) }) as ObjectRef).unwrap();
    let dt2 = DumpTrapQ::try_new(printer2.clone(), "toolhead".into(), Some(make_handle())).expect("dump trapq drip");
    let pc = dt2.process_batch(0.);
    check("trapq_batch_drip", exp_c, pyvalue_to_record(&pc));
}

#[test]
fn batch_streaming_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
wh = FakeWebhooks()
p = FakePrinter(); p.add_object('webhooks', wh); p.add_object('motion_queuing', FakeMotionQueuing()); p.add_object('gcode', FakeGcode())
m = FakeMcu()
mk = FakeMcuStepper('stepper_x', m, [FakeStep(100,110,0,5,2,1), FakeStep(200,210,10,6,0,0), FakeStep(300,310,20,7,3,-1)], 0.02)
ds = DumpStepper(p, mk)
mux_entry = wh.mux[0][0] + '|' + wh.mux[0][1] + '|' + wh.mux[0][2]
cb = wh.mux[0][3]
conn = FakeClientConnection(); wr = FakeWebRequest({}, conn)
cb(wr)
wr_start = wr.sent
wake = ds.batch_bulk._proc_batch(10.0)
conn_sent = conn.sent
"#).expect("py streaming");
    let exp_mux = py.eval("mux_entry");
    let exp_start = py.eval("wr_start");
    let exp_wake = py.eval("wake");
    let exp_sent = py.eval("conn_sent");
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    let wh: Arc<FakeWebhooks> = Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) });
    printer.add_object("webhooks", wh.clone() as ObjectRef).unwrap();
    let mcu: klippy_api::mcu::Mcu = Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() });
    let stepper: Stepper = Arc::new(FakeStepper {
        name: "stepper_x".into(), mcu: mcu.clone(),
        batches: Mutex::new(vec![(
            vec![hist_step(100, 110, 0, 5, 2, 1), hist_step(200, 210, 10, 6, 0, 0), hist_step(300, 310, 20, 7, 3, -1)],
            3,
        )]),
        step_dist: 0.02,
    });
    let ds = DumpStepper::try_new(printer.clone(), stepper).expect("dump stepper");
    let (path, key, value, cb) = wh.mux.lock().unwrap()[0].clone();
    let (wr, conn_state, wr_sent) = FakeWebRequest::build(PyValue::Dict(IndexMap::new()), false);
    cb(wr).expect("mux cb");
    let wake = ds.batch_bulk()._proc_batch(10.0).expect("proc_batch");
    check("stream_mux_entry", exp_mux, Value::str(&format!("{path}|{key}|{value}")));
    check("stream_start_resp", exp_start, pyvalues_to_record(&wr_sent.lock().unwrap()));
    check("stream_wake", exp_wake, Value::float(wake));
    check("stream_conn_sent", exp_sent, pyvalues_to_record(&conn_state.lock().unwrap().sent));
}

#[test]
fn connect_and_live_status_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
import chelper
_fm, _fl = chelper.get_ffi()
_tt = _fl.trapq_alloc()
_fl.trapq_append(_tt, 8.0, 4.0, 1.0, 0.5, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 0.0, 0.0, 0.0)
_te = _fl.trapq_alloc()
_fl.trapq_append(_te, 8.0, 4.0, 1.0, 0.5, 5.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_p = FakePrinter()
_p.add_object('gcode', FakeGcode()); _p.add_object('webhooks', FakeWebhooks()); _p.add_object('motion_queuing', FakeMotionQueuing())
_p.add_object('mcu', FakeMcu())
_p.add_object('toolhead', FakeToolhead(_tt, [None, None, None, FakeExtruder(_te)]))
_p.add_object('extruder', FakeExtruder(_te))
_p.add_object('manual_stepper', FakeExtruder(None, 'manual_stepper'))
_c = FakeConfig(_p)
_r = motion_report.load_config(_c)
_r._connect()
st_after_connect = _r.get_status(0.0)
st_live = _r.get_status(10.0)
st_cached = _r.get_status(10.1)
"#).expect("py connect/live");
    let exp_connect = py.eval("st_after_connect");
    let exp_live = py.eval("st_live");
    let exp_cached = py.eval("st_cached");
    fn mk_move(handle: &Trapq) {
        let h = klippy_core::motion_handles::trapq_of(handle).expect("trapq_of");
        let mut ct = h.shared().lock();
        ct.append(8.0, 4.0, 1.0, 0.5, 1.0, 2.0, 3.0, 0.6, 0.8, 0.0, 0.0, 0.0, 0.0);
    }
    let tt: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
    mk_move(&tt);
    let te: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
    {
        let h = klippy_core::motion_handles::trapq_of(&te).expect("trapq_of");
        let mut ct = h.shared().lock();
        ct.append(8.0, 4.0, 1.0, 0.5, 5.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    printer.add_object("mcu", Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() }) as ObjectRef).unwrap();
    let extruder_axis: ExtraAxis = Arc::new(FakeExtruder { name: "extruder".into(), trapq: Some(te.clone()) });
    printer.add_object("toolhead", Arc::new(FakeToolhead { trapq: tt, extra_axes: vec![None, None, None, Some(extruder_axis)] }) as ObjectRef).unwrap();
    printer.add_object("extruder", Arc::new(FakeExtruder { name: "extruder".into(), trapq: Some(te) }) as ObjectRef).unwrap();
    printer.add_object("manual_stepper", Arc::new(FakeExtruder { name: "manual_stepper".into(), trapq: None }) as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let report = PrinterMotionReport::new(&config).expect("report");
    report.handle_connect(&[]).expect("connect");
    check("status_after_connect", exp_connect, pyvalue_to_record(&report.get_status_value(0.0).expect("status")));
    check("live_status", exp_live, pyvalue_to_record(&report.get_status_value(10.0).expect("live")));
    check("cached_status", exp_cached, pyvalue_to_record(&report.get_status_value(10.1).expect("cached")));
}

#[test]
fn get_status_error_paths() {
    let Some(mut py) = oracle() else { return };
    py.exec(r#"
def _mk_printer(with_toolhead=True, with_mcu=True):
    p = FakePrinter()
    p.add_object('gcode', FakeGcode()); p.add_object('webhooks', FakeWebhooks()); p.add_object('motion_queuing', FakeMotionQueuing())
    if with_mcu:
        p.add_object('mcu', FakeMcu())
    if with_toolhead:
        p.add_object('toolhead', FakeToolhead(None))
    return p
_pa = _mk_printer(True, True)
_ca = FakeConfig(_pa)
_ra = motion_report.load_config(_ca)
_ra._connect()
del _pa.objects['toolhead']
_pb = _mk_printer(True, True)
_cb = FakeConfig(_pb)
_rb = motion_report.load_config(_cb)
_rb._connect()
del _pb.objects['mcu']
"#).expect("py status errors");
    // OracleResult for a raising eval carries class/message; read them back.
    let py_err_a = py_eval_err(&mut py, "_ra.get_status(10.0)");
    let py_err_b = py_eval_err(&mut py, "_rb.get_status(10.0)");
    fn build() -> (FakePrinter, Printer, Arc<PrinterMotionReport>) {
        let (fp, printer) = FakePrinter::new();
        printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
        printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
        printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
        printer.add_object("mcu", Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() }) as ObjectRef).unwrap();
        let tt: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
        printer.add_object("toolhead", Arc::new(FakeToolhead { trapq: tt, extra_axes: vec![None, None, None] }) as ObjectRef).unwrap();
        let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
        let report = PrinterMotionReport::new(&config).expect("report");
        report.handle_connect(&[]).expect("connect");
        (fp, printer, report)
    }
    let (fp_a, _printer_a, report_a) = build();
    fp_a.objects.lock().unwrap().shift_remove("toolhead");
    check_err("status_missing_toolhead", py_err_a, report_a.get_status_value(10.0));
    let (fp_b, _printer_b, report_b) = build();
    fp_b.objects.lock().unwrap().shift_remove("mcu");
    check_err("status_missing_mcu", py_err_b, report_b.get_status_value(10.0));
}

fn py_eval_err(py: &mut pyoracle::Oracle, expr: &str) -> pyoracle::OracleResult {
    py.eval(expr)
}

#[test]
fn shutdown_logging_matches_python() {
    let Some(mut py) = oracle() else { return };
    let _guard = lock_logs();
    py.exec(r#"
import chelper
_fm, _fl = chelper.get_ffi()
_tq = _fl.trapq_alloc()
_fl.trapq_append(_tq, 4.96, 0.2, 0.0, 0.0, 1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0)
_m = FakeMcu()
_p = FakePrinter()
_p.add_object('gcode', FakeGcode()); _p.add_object('webhooks', FakeWebhooks()); _p.add_object('motion_queuing', FakeMotionQueuing())
_p.add_object('mcu', _m)
_p.add_object('toolhead', FakeToolhead(_tq))
_c = FakeConfig(_p)
_r = motion_report.load_config(_c)
_r.register_stepper(_c, FakeMcuStepper('stepper_x', _m, [FakeStep(4950,4960,0,5,2,1), FakeStep(5050,5060,10,6,3,-1)], 0.01))
_r._connect()
py_clear_logs()
_r._handle_analyze_shutdown('MCU shutdown', {'mcu': 'mcu', 'shutdown_clock': 5000})
_r._handle_analyze_shutdown('Internal error in stepcompress', {'queue_name': 'stepper_x', 'step_gen_time': 5.0})
py_result = py_logs()
"#).expect("py shutdown logs");
    let exp_logs = py.eval("py_result");
    let tt: Trapq = klippy_core::motion_handles::TrapqHandle::alloc();
    {
        let h = klippy_core::motion_handles::trapq_of(&tt).expect("trapq_of");
        let mut ct = h.shared().lock();
        ct.append(4.96, 0.2, 0.0, 0.0, 1.0, 2.0, 3.0, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0);
    }
    let (_fp, printer) = FakePrinter::new();
    printer.add_object("gcode", Arc::new(FakeGcode) as ObjectRef).unwrap();
    printer.add_object("webhooks", Arc::new(FakeWebhooks { mux: Mutex::new(Vec::new()) }) as ObjectRef).unwrap();
    printer.add_object("motion_queuing", Arc::new(FakeMotionQueuing { drip: None }) as ObjectRef).unwrap();
    let mcu_concrete = Arc::new(FakeMcu { name: "mcu".into(), printer: printer.clone() });
    let mcu: klippy_api::mcu::Mcu = mcu_concrete.clone();
    printer.add_object("mcu", mcu_concrete as ObjectRef).unwrap();
    printer.add_object("toolhead", Arc::new(FakeToolhead { trapq: tt, extra_axes: vec![None, None, None] }) as ObjectRef).unwrap();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    let report = PrinterMotionReport::new(&config).expect("report");
    let stepper: Stepper = Arc::new(FakeStepper {
        name: "stepper_x".into(), mcu: mcu.clone(),
        batches: Mutex::new(vec![(
            vec![hist_step(4950, 4960, 0, 5, 2, 1), hist_step(5050, 5060, 10, 6, 3, -1)],
            2,
        )]),
        step_dist: 0.01,
    });
    report.register_stepper(&config, stepper).expect("register_stepper");
    report.handle_connect(&[]).expect("connect");
    let mk_details = |pairs: Vec<(&str, PyValue)>| -> IndexMap<PyValue, PyValue> {
        let mut d = IndexMap::new();
        for (k, v) in pairs { d.insert(PyValue::Str(k.to_string()), v); }
        d
    };
    report.handle_analyze_shutdown(&[
        EventArg::Value(PyValue::Str("MCU shutdown".into())),
        EventArg::Value(PyValue::Dict(mk_details(vec![
            ("mcu", PyValue::Str("mcu".into())),
            ("shutdown_clock", PyValue::Int(PyInt::from_i64(5000))),
        ]))),
    ]).expect("mcu shutdown");
    report.handle_analyze_shutdown(&[
        EventArg::Value(PyValue::Str("Internal error in stepcompress".into())),
        EventArg::Value(PyValue::Dict(mk_details(vec![
            ("queue_name", PyValue::Str("stepper_x".into())),
            ("step_gen_time", PyValue::Float(5.0)),
        ]))),
    ]).expect("stepcompress");
    check("shutdown_logs", exp_logs, logs_value());
}
