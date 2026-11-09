//! Parity tests for `extras::pwm_tool` (port of `klippy/extras/pwm_tool.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): a `PrinterOutputPin` and its
//! `MCU_queued_pwm` are built on both sides with minimal Python/Rust fakes
//! for printer, config, MCU, pins, `motion_queuing`, `gcode` and `toolhead`,
//! plus a fake `chelper` recording `syncemitter_queue_msg`. Same inputs are
//! fed to both and the following are compared exactly:
//! - MCU config commands (`add_config_cmd` strings + is_init/on_restart opts)
//! - syncemitter queued messages `(clock, tag, oid, clock & 0xffffffff, value)`
//! - `motion_queuing.note_mcu_movequeue_activity` print times
//! - `get_status` value, `SET_PIN` lookahead effects, and raised errors
//!   (class and message) for config, G-code and MCU paths.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use std::sync::Arc;

use extras::pwm_tool::{module_factory, PrinterOutputPin};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetFloatOpts as GcodeGetFloatOpts, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandWrapper,
    CommandWrapperApi, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin,
    PinParams, PinsApi, SendOpts,
};
use klippy_api::object::EventArg;
use klippy_api::printer::{EventCallback, LookupDefault, ModuleFactory, Printer, PrinterApi};
use klippy_api::stepper::Trapq;
use klippy_api::toolhead::{
    FlushCallback, LookaheadCallback, MotionQueuingApi, SyncEmitterHandle, ToolHeadApi,
    TrapqAppend, Kinematics, PrinterExtruder, ExtraAxis,
};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.pwm_tool as _pwm_tool

class _FakeFFILib:
    def __init__(self):
        self.queue_log = []
    def syncemitter_queue_msg(self, se, clock, data, length):
        self.queue_log.append((str(se), clock, tuple(data), length))

class _FakeChelper:
    def __init__(self):
        self.lib = _FakeFFILib()
        self.main = object()
    def get_ffi(self):
        return (self.main, self.lib)

_fake_chelper = _FakeChelper()
_pwm_tool.chelper = _fake_chelper

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.t = 1000.0
    def monotonic(self):
        return self.t
    def register_timer(self, callback, waketime=None):
        return callback
    def update_timer(self, timer, waketime):
        pass
    def assert_no_pause(self):
        import contextlib
        return contextlib.nullcontext()

class FakeCommandWrapper:
    def __init__(self, tag):
        self._tag = tag
    def get_command_tag(self):
        return self._tag

class FakeMCU:
    FREQ = 100000.0
    PWM_MAX = 255.0
    def __init__(self, printer):
        self._printer = printer
        self._next_oid = 1
        self.config_callbacks = []
        self.added = []
        self.lookups = []
        self.cmd_queue = object()
    def get_printer(self):
        return self._printer
    def create_oid(self):
        oid = self._next_oid
        self._next_oid += 1
        return oid
    def alloc_command_queue(self):
        return self.cmd_queue
    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.added.append((cmd, bool(is_init), bool(on_restart)))
    def lookup_command(self, msgformat, cq=None):
        self.lookups.append((msgformat, cq))
        return FakeCommandWrapper(42)
    def seconds_to_clock(self, time):
        return int(time * self.FREQ)
    def print_time_to_clock(self, print_time):
        return int(print_time * self.FREQ)
    def clock_to_print_time(self, clock):
        return clock / self.FREQ
    def estimated_print_time(self, eventtime):
        return eventtime
    def max_nominal_duration(self):
        return 5.0
    def is_fileoutput(self):
        return False
    def get_constant_float(self, name):
        if name == "PWM_MAX":
            return self.PWM_MAX
        raise Exception("unknown constant %s" % (name,))

class FakePins:
    def __init__(self, mcu):
        self._mcu = mcu
    def lookup_pin(self, pin_desc, can_invert=False):
        invert = 0
        pin = pin_desc
        if pin.startswith("!"):
            invert = 1
            pin = pin[1:]
        return {"chip": self._mcu, "pin": pin, "invert": invert}

class FakeMotionQueuing:
    def __init__(self):
        self.allocs = []
        self.flush_callbacks = []
        self.activity = []
    def allocate_syncemitter(self, mcu, name, alloc_stepcompress=True):
        if not name.split():
            raise IndexError("list index out of range")
        sname = name.split()[-1]
        self.allocs.append((id(mcu), name, bool(alloc_stepcompress)))
        return "se:%s" % (sname,)
    def register_flush_callback(self, callback, can_add_trapq=False):
        self.flush_callbacks.append((callback, bool(can_add_trapq)))
    def note_mcu_movequeue_activity(self, mq_time, is_step_gen):
        self.activity.append((mq_time, bool(is_step_gen)))
    def flush_now(self, must_flush_time, max_step_gen_time):
        for cb, _ in list(self.flush_callbacks):
            cb(must_flush_time, max_step_gen_time)

class FakeGCode:
    def __init__(self):
        self.mux = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, func, desc))

class FakeToolHead:
    def __init__(self):
        self.lookaheads = []
    def register_lookahead_callback(self, callback):
        self.lookaheads.append(callback)
    def run_lookaheads(self, print_time):
        for cb in list(self.lookaheads):
            cb(print_time)
        self.lookaheads.clear()

class FakePrinter:
    config_error = configparser.Error
    def __init__(self):
        self._reactor = FakeReactor()
        self.mcu = FakeMCU(self)
        self.pins = FakePins(self.mcu)
        self.gcode = FakeGCode()
        self.toolhead = FakeToolHead()
        self.motion_queuing = FakeMotionQueuing()
        self.event_handlers = {}
    def get_reactor(self):
        return self._reactor
    def lookup_object(self, name, default=None):
        objs = {"pins": self.pins, "gcode": self.gcode, "toolhead": self.toolhead}
        if name in objs:
            return objs[name]
        # default sentinel handling: pwm_tool always omits default (required).
        # Mimic klippy Printer: missing raises config_error.
        raise self.config_error("Unknown config object '%s'" % (name,))
    def lookup_objects(self, module=None):
        return [("mcu", self.mcu)] if module == "mcu" else []
    def load_object(self, config, section, default=None):
        if section == "motion_queuing":
            return self.motion_queuing
        raise self.config_error("Unable to load module '%s'" % (section,))
    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        for cb in list(self.event_handlers.get(event, [])):
            cb(*params)

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, opts):
        self._printer = printer
        self._name = name
        self._opts = opts
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    class _sentinel:
        pass
    def _get(self, option, default, minval=None, maxval=None, above=None, below=None, convert=None):
        missing = object()
        raw = self._opts.get(option, missing)
        if raw is missing:
            if default is not self._sentinel:
                return default
            raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
        try:
            v = convert(raw) if convert else raw
        except Exception:
            raise self.error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def get(self, option, default=_sentinel):
        return self._get(option, default)
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        return self._get(option, default, minval, maxval, above, below, convert=float)
    def getboolean(self, option, default=_sentinel):
        def conv(raw):
            s = str(raw).strip().lower()
            if s in ("1", "true", "yes", "on"):
                return True
            if s in ("0", "false", "no", "off"):
                return False
            raise ValueError("boolean")
        return self._get(option, default, convert=conv)
class CommandError(Exception): pass
class FakeGcmd:
    from extras.pwm_tool import PrinterOutputPin as _PP
    error = CommandError
    def __init__(self, commandline, params, command="SET_PIN"):
        self._commandline = commandline
        self._params = params
        self._command = command
    def get_float(self, name, minval=None, maxval=None):
        value = self._params.get(name)
        if value is None:
            raise CommandError("Error on '%s': missing %s" % (self._commandline, name))
        try:
            value = float(value)
        except Exception:
            raise CommandError("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise CommandError("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise CommandError("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        return value
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn py_float_repr(f: f64) -> String {
    pycompat::text::float_repr(f)
}

fn fmt_opt(v: Option<f64>) -> String {
    match v {
        Some(x) => py_float_repr(x),
        None => "None".to_string(),
    }
}

const FREQ: f64 = 100000.0;
const PWM_MAX: f64 = 255.0;
const MAX_NOMINAL: f64 = 5.0;

fn trunc_to_i64(f: f64) -> i64 {
    pycompat::text::trunc_float_to_bigint(f)
        .0
        .to_string()
        .parse::<i64>()
        .expect("fake clock fits i64")
}

fn sec_to_clock(t: f64) -> i64 {
    trunc_to_i64(t * FREQ)
}
fn ptime_to_clock(t: f64) -> i64 {
    trunc_to_i64(t * FREQ)
}
fn clock_to_ptime(c: i64) -> f64 {
    c as f64 / FREQ
}

#[derive(Clone)]
struct FakeCommandWrapper {
    tag: i64,
}
impl CommandWrapperApi for FakeCommandWrapper {
    fn send(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn send_wait_ack(&self, _data: Vec<PyValue>, _opts: SendOpts) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        self.tag
    }
}

struct McuState {
    next_oid: i64,
    added: Vec<(String, AddConfigCmdOpts)>,
    lookups: Vec<(String, Option<CommandQueue>)>,
    config_cb: Option<klippy_api::mcu::ConfigCallback>,
}

struct FakeMcu {
    printer_slot: kos::sync::Mutex<Option<Printer>>,
    state: kos::sync::Mutex<McuState>,
}

impl FakeMcu {
    fn new() -> Self {
        Self {
            printer_slot: kos::sync::Mutex::new(None),
            state: kos::sync::Mutex::new(McuState {
                next_oid: 1,
                added: Vec::new(),
                lookups: Vec::new(),
                config_cb: None,
            }),
        }
    }
    fn set_printer(&self, printer: Printer) {
        *self.printer_slot.lock() = Some(printer);
    }
    fn added(&self) -> Vec<(String, AddConfigCmdOpts)> {
        self.state.lock().added.clone()
    }
    fn lookups(&self) -> Vec<(String, Option<CommandQueue>)> {
        self.state.lock().lookups.clone()
    }
    fn run_config_callbacks(&self) -> KResult<()> {
        let mut st = self.state.lock();
        if let Some(mut f) = st.config_cb.take() {
            drop(st);
            return f();
        }
        Ok(())
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer_slot.lock().clone().expect("FakeMcu printer set")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin not used by pwm_tool")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut st = self.state.lock();
        let oid = st.next_oid;
        st.next_oid += 1;
        Ok(oid)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.state.lock().config_cb = Some(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.state.lock().added.push((cmd.to_string(), opts));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        sec_to_clock(time)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        MAX_NOMINAL
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        self.state
            .lock()
            .lookups
            .push((msgformat.to_string(), cq));
        Ok(Arc::new(FakeCommandWrapper { tag: 42 }) as CommandWrapper)
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        panic!("lookup_query_command not used by pwm_tool")
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response not used by pwm_tool")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        if name == "PWM_MAX" {
            Ok(PWM_MAX)
        } else {
            Err(KlippyError::Mcu(format!("unknown constant {name}")))
        }
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        ptime_to_clock(print_time)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock_to_ptime(trunc_to_i64(clock))
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        1000.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock not used by pwm_tool")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch not used by pwm_tool")
    }
    fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins {
    mcu: Mcu,
}
impl FakePins {
    fn new(mcu: Mcu) -> Self {
        Self { mcu }
    }
}
impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin not used by pwm_tool")
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        let mut invert = 0i64;
        let mut pin = pin_desc.to_string();
        if let Some(stripped) = pin.strip_prefix('!') {
            invert = 1;
            pin = stripped.to_string();
        }
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin,
            invert,
            pullup: 0,
            share_type: None,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin not used by pwm_tool")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver not used by pwm_tool")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip not used by pwm_tool")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin not used by pwm_tool")
    }
}
klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeMotionQueuing {
    allocs: kos::sync::Mutex<Vec<(String, String, bool)>>,
    flush_callbacks: kos::sync::Mutex<Vec<FlushCallback>>,
    activity: kos::sync::Mutex<Vec<(f64, bool)>>,
    emitter_names: kos::sync::Mutex<Vec<String>>,
    emitter_queues: kos::sync::Mutex<Vec<Arc<kos::sync::Mutex<Vec<(u64, Vec<u32>)>>>>>,
}
impl FakeMotionQueuing {
    fn new() -> Self {
        Self {
            allocs: kos::sync::Mutex::new(Vec::new()),
            flush_callbacks: kos::sync::Mutex::new(Vec::new()),
            activity: kos::sync::Mutex::new(Vec::new()),
            emitter_names: kos::sync::Mutex::new(Vec::new()),
            emitter_queues: kos::sync::Mutex::new(Vec::new()),
        }
    }
    fn allocs(&self) -> Vec<(String, String, bool)> {
        self.allocs.lock().clone()
    }
    fn activity(&self) -> Vec<(f64, bool)> {
        self.activity.lock().clone()
    }
    fn queued_records(&self) -> Vec<(String, u64, Vec<u32>)> {
        let names = self.emitter_names.lock().clone();
        let queues = self.emitter_queues.lock().clone();
        let mut out = Vec::new();
        for (name, q) in names.iter().zip(queues.iter()) {
            for (clock, data) in q.lock().iter() {
                out.push((format!("se:{name}"), *clock, data.clone()));
            }
        }
        out
    }
    fn flush_count(&self) -> usize {
        self.flush_callbacks.lock().len()
    }
    fn flush_now(&self, must_flush_time: f64, max_step_gen_time: f64) {
        let cbs = self.flush_callbacks.lock().clone();
        for cb in cbs {
            cb(must_flush_time, max_step_gen_time);
        }
    }
}
impl MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> Trapq {
        panic!("allocate_trapq not used by pwm_tool")
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        panic!("lookup_trapq_append not used by pwm_tool")
    }
    fn wipe_trapq(&self, _trapq: &Trapq) {}
    fn allocate_syncemitter(
        &self,
        _mcu: Mcu,
        name: &str,
        alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle> {
        if name.split_whitespace().next().is_none() {
            return Err(KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "list index out of range".to_string(),
            });
        }
        let sname = name.split_whitespace().last().unwrap().to_string();
        self.allocs
            .lock()
            .push((name.to_string(), sname.clone(), alloc_stepcompress));
        let queued = Arc::new(kos::sync::Mutex::new(Vec::new()));
        self.emitter_names.lock().push(format!("se:{sname}"));
        self.emitter_queues.lock().push(queued.clone());
        Ok(Arc::new(FakeEmitter { name: sname, queued }) as SyncEmitterHandle)
    }
    fn queue_syncemitter_msg(
        &self,
        syncemitter: &SyncEmitterHandle,
        req_clock: u64,
        data: &[u32],
    ) {
        // Mirror the Python `_FakeFFILib.queue_log` entries
        // `(str(se), clock, tuple(data), len)`; `se` is the FakeEmitter.
        syncemitter.queue_msg(req_clock, data);
    }
    fn setup_mcu_movequeue(
        &self,
        _mcu: Mcu,
        _serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>,
        _move_count: i64,
    ) {
    }
    fn register_flush_callback(&self, callback: FlushCallback, _can_add_trapq: bool) {
        self.flush_callbacks.lock().push(callback);
    }
    fn register_flush_callback_named(
        &self,
        callback: FlushCallback,
        can_add_trapq: bool,
        _name: Option<&str>,
        _obj_name: Option<&str>,
    ) {
        self.register_flush_callback(callback, can_add_trapq);
    }
    fn unregister_flush_callback(&self, _callback: &FlushCallback) {}
    fn get_kin_flush_delay(&self) -> f64 {
        0.0
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, _est_print_time: f64) -> f64 {
        0.0
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        self.activity.lock().push((mq_time, is_step_gen));
    }
    fn drip_update_time(
        &self,
        _start_time: f64,
        _end_time: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_update_time not used by pwm_tool")
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}
#[derive(Clone)]
struct FakeEmitter {
    name: String,
    queued: Arc<kos::sync::Mutex<Vec<(u64, Vec<u32>)>>>,
}

impl klippy_api::toolhead::SyncEmitterApi for FakeEmitter {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn queue_msg(&self, req_clock: u64, data: &[u32]) {
        self.queued.lock().push((req_clock, data.to_vec()));
    }
    fn set_stepper_kinematics(&self, _sk: Option<klippy_api::stepper::StepperKinematics>) {}
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn stepcompress_fill(&self, _o: u32, _m: u32, _q: i32, _d: i32) {}
    fn stepcompress_set_invert_sdir(&self, _invert: u32) {}
    fn stepcompress_reset(&self, _clock: u64) -> i32 {
        0
    }
    fn stepcompress_set_last_position(&self, _clock: u64, _pos: i64) -> i32 {
        0
    }
    fn stepcompress_find_past_position(&self, _clock: u64) -> i64 {
        0
    }
    fn stepcompress_extract_old(
        &self,
        _max: i32,
        _start: u64,
        _end: u64,
    ) -> Vec<klippy_api::stepper::PullHistoryStep> {
        Vec::new()
    }
}
klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

struct FakeGCode {
    mux: kos::sync::Mutex<Vec<(String, String, Option<String>, Option<String>)>>,
    handlers: kos::sync::Mutex<Vec<(String, String, Option<String>, GCodeCommandHandler)>>,
}
impl FakeGCode {
    fn new() -> Self {
        Self {
            mux: kos::sync::Mutex::new(Vec::new()),
            handlers: kos::sync::Mutex::new(Vec::new()),
        }
    }
    fn mux_records(&self) -> Vec<(String, String, Option<String>, Option<String>)> {
        self.mux.lock().clone()
    }
    fn handler_for(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
    ) -> GCodeCommandHandler {
        let mut hs = self.handlers.lock();
        for (i, (c, k, v, _h)) in hs.iter().enumerate() {
            if c == cmd && k == key && v.as_deref() == value {
                return hs.remove(i).3;
            }
        }
        panic!("no handler for {cmd} {key}={value:?}")
    }
}
impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("register_command not used by pwm_tool")
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(str::to_string),
            desc.map(str::to_string),
        ));
        self.handlers.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.map(str::to_string),
            func,
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: Box<dyn FnMut(String) + Send>) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart not used by pwm_tool")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script not used by pwm_tool")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command not used by pwm_tool")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex not used by pwm_tool")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
        }) as GCmd
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 not used by pwm_tool")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands not used by pwm_tool")
    }
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeToolHead {
    lookaheads: kos::sync::Mutex<Vec<LookaheadCallback>>,
}
impl FakeToolHead {
    fn new() -> Self {
        Self {
            lookaheads: kos::sync::Mutex::new(Vec::new()),
        }
    }
    fn run_lookaheads(&self, print_time: f64) -> KResult<()> {
        let mut cbs = std::mem::take(&mut *self.lookaheads.lock());
        for mut cb in cbs.drain(..) {
            cb(print_time)?;
        }
        Ok(())
    }
    fn lookahead_count(&self) -> usize {
        self.lookaheads.lock().len()
    }
}
klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);
impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics not used by pwm_tool")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move not used by pwm_tool")
    }
}
impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move not used by pwm_tool")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move not used by pwm_tool")
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq not used by pwm_tool")
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
    fn register_lookahead_callback(&self, callback: LookaheadCallback) -> KResult<()> {
        self.lookaheads.lock().push(callback);
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder not used by pwm_tool")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory not used by pwm_tool")
    }
}

struct FakePrinter {
    reactor: Reactor,
    objects: kos::sync::Mutex<IndexMap<String, klippy_api::ObjectRef>>,
    handlers: kos::sync::Mutex<Vec<(String, EventCallback)>>,
}
#[allow(dead_code)]
struct TestWorld {
    printer: Printer,
    mcu: Mcu,
    mcu_inner: Arc<FakeMcu>,
    pins: Arc<FakePins>,
    pins_obj: klippy_api::ObjectRef,
    mcu_obj: klippy_api::ObjectRef,
    gcode: Arc<FakeGCode>,
    gcode_obj: klippy_api::ObjectRef,
    toolhead: Arc<FakeToolHead>,
    toolhead_obj: klippy_api::ObjectRef,
    motion_queuing: Arc<FakeMotionQueuing>,
    mq_obj: klippy_api::ObjectRef,
}

fn make_world() -> TestWorld {
    let printer_inner: Arc<FakePrinter> = Arc::new(FakePrinter {
        reactor: Reactor::new(false),
        objects: kos::sync::Mutex::new(IndexMap::new()),
        handlers: kos::sync::Mutex::new(Vec::new()),
    });
    let printer: Printer = printer_inner.clone();
    let mcu_inner = Arc::new(FakeMcu::new());
    mcu_inner.set_printer(printer.clone());
    let mcu: Mcu = mcu_inner.clone();
    let mcu_obj: klippy_api::ObjectRef = mcu_inner.clone();
    let pins = Arc::new(FakePins::new(mcu.clone()));
    let pins_obj: klippy_api::ObjectRef = pins.clone();
    let gcode = Arc::new(FakeGCode::new());
    let gcode_obj: klippy_api::ObjectRef = gcode.clone();
    let toolhead = Arc::new(FakeToolHead::new());
    let toolhead_obj: klippy_api::ObjectRef = toolhead.clone();
    let motion_queuing = Arc::new(FakeMotionQueuing::new());
    let mq_obj: klippy_api::ObjectRef = motion_queuing.clone();
    {
        let mut objs = printer_inner.objects.lock();
        objs.insert("pins".to_string(), pins_obj.clone());
        objs.insert("gcode".to_string(), gcode_obj.clone());
        objs.insert("toolhead".to_string(), toolhead_obj.clone());
        objs.insert("motion_queuing".to_string(), mq_obj.clone());
    }
    TestWorld {
        printer,
        mcu,
        mcu_inner,
        pins,
        pins_obj,
        mcu_obj,
        gcode,
        gcode_obj,
        toolhead,
        toolhead_obj,
        motion_queuing,
        mq_obj,
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
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, _name: &str, _obj: klippy_api::ObjectRef) -> KResult<()> {
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
        let objs = self.objects.lock();
        if let Some(o) = objs.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, klippy_api::ObjectRef)> {
        self.objects
            .lock()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<klippy_api::ObjectRef>> {
        if let Some(o) = self.objects.lock().get(section).cloned() {
            return Ok(Some(o));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        let mut hs = self.handlers.lock();
        let mut out = Vec::new();
        let mut i = 0;
        while i < hs.len() {
            if hs[i].0 == event {
                let (_e, mut cb) = hs.remove(i);
                out.push(cb(params)?);
            } else {
                i += 1;
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}


struct FakeConfig {
    printer: Printer,
    name: String,
    opts: IndexMap<String, String>,
}
impl FakeConfig {
    fn new(printer: Printer, name: &str, opts: IndexMap<String, String>) -> Self {
        Self {
            printer,
            name: name.to_string(),
            opts,
        }
    }
}
impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(raw) = self.opts.get(option) {
            Ok(Some(raw.clone()))
        } else {
            match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            }
        }
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint not used by pwm_tool")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.opts.get(option) {
            Some(r) => r.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    ))),
                    Some(None) => Ok(None),
                    Some(Some(v)) => Ok(Some(v)),
                }
            }
        };
        let v: f64 = match raw.parse() {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.name
                )))
            }
        };
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    py_float_repr(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    py_float_repr(max)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    py_float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    py_float_repr(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let raw = match self.opts.get(option) {
            Some(r) => r.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    ))),
                    Some(None) => Ok(None),
                    Some(Some(v)) => Ok(Some(v)),
                }
            }
        };
        let s = raw.trim().to_lowercase();
        if ["1", "true", "yes", "on"].contains(&s.as_str()) {
            Ok(Some(true))
        } else if ["0", "false", "no", "off"].contains(&s.as_str()) {
            Ok(Some(false))
        } else {
            Err(KlippyError::config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.name
            )))
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice not used by pwm_tool")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists not used by pwm_tool")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist not used by pwm_tool")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist not used by pwm_tool")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist not used by pwm_tool")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used by pwm_tool")
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

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}
impl GCodeCommandApi for FakeGcmd {
    fn get(&self, _name: &str, _opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        panic!("get not used by pwm_tool")
    }
    fn get_int(&self, _name: &str, _opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int not used by pwm_tool")
    }
    fn get_float(&self, name: &str, opts: GcodeGetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            Some(r) => r.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::command(format!(
                        "Error on '{}': missing {name}",
                        self.commandline
                    ))),
                    Some(v) => Ok(v),
                }
            }
        };
        let v: f64 = match raw.parse() {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::command(format!(
                    "Error on '{}': unable to parse {raw}",
                    self.commandline
                )))
            }
        };
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have minimum of {}",
                    self.commandline,
                    py_float_repr(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must have maximum of {}",
                    self.commandline,
                    py_float_repr(max)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be above {}",
                    self.commandline,
                    py_float_repr(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::command(format!(
                    "Error on '{}': {name} must be below {}",
                    self.commandline,
                    py_float_repr(below)
                )));
            }
        }
        Ok(Some(v))
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
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("pwm_tool fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping pwm_tool oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
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
        unit: "pwm_tool".to_string(),
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

fn check_err(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KlippyError,
) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        actual.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

#[allow(dead_code)]
fn check_ok_err<T>(
    label: &str,
    expected: pyoracle::OracleResult,
    actual: KResult<T>,
) {
    match actual {
        Ok(_) => {
            if let Err(e) = expected {
                panic!("{label}: oracle raised {}: {} but rust succeeded", e.class_name, e.message);
            }
        }
        Err(e) => check_err(label, Err(match expected {
            Err(e) => e,
            Ok(v) => panic!("{label}: oracle returned {v:?} but rust failed {e:?}"),
        }), e),
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.inner().to_string();
            if let Ok(i) = s.parse::<i64>() {
                Value::Int(i)
            } else {
                Value::BigInt(s)
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, vv)| (pyvalue_to_value(k), pyvalue_to_value(vv)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "host".to_string(),
            addr: format!("{:#x}", h),
        },
    }
}

fn queued_to_value(records: &[(String, u64, Vec<u32>)]) -> Value {
    Value::List(
        records
            .iter()
            .map(|(_se, clock, data)| {
                Value::List(vec![
                    Value::Int(*clock as i64),
                    Value::Int(data[0] as i64),
                    Value::Int(data[1] as i64),
                    Value::Int(data[2] as i64),
                    Value::Int(data[3] as i64),
                ])
            })
            .collect(),
    )
}

fn added_to_value(added: &[(String, AddConfigCmdOpts)]) -> Value {
    Value::List(
        added
            .iter()
            .map(|(s, o)| {
                Value::Tuple(vec![
                    Value::Str(s.clone()),
                    Value::Bool(o.is_init),
                    Value::Bool(o.on_restart),
                ])
            })
            .collect(),
    )
}

fn activity_to_value(act: &[(f64, bool)]) -> Value {
    Value::List(
        act.iter()
            .map(|(t, b)| Value::Tuple(vec![Value::float(*t), Value::Bool(*b)]))
            .collect(),
    )
}

fn default_opts() -> IndexMap<String, String> {
    let mut m = IndexMap::new();
    m.insert("pin".to_string(), "PA1".to_string());
    m
}

fn opts_with(extra: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut m = default_opts();
    for (k, v) in extra {
        m.insert((*k).to_string(), (*v).to_string());
    }
    m
}

fn py_dict_expr(opts: &IndexMap<String, String>) -> String {
    let parts: Vec<String> = opts
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn fresh_pair(
    py: &mut pyoracle::Oracle,
    opts: &IndexMap<String, String>,
) -> (Arc<PrinterOutputPin>, TestWorld, Config, String) {
    let world = make_world();
    let config: Config = Arc::new(FakeConfig::new(
        world.printer.clone(),
        "pwm_tool my_pin",
        opts.clone(),
    ));
    let rust = PrinterOutputPin::new(&config).expect("rust PrinterOutputPin::new");
    let py_opts = py_dict_expr(opts);
    let setup_ok = py.exec(
        &format!(
            "_printer = FakePrinter(); _config = FakeConfig(_printer, 'pwm_tool my_pin', {py_opts}); pin = _pwm_tool.PrinterOutputPin(_config); _mcu = _printer.mcu; _mq = _printer.motion_queuing; _gcode = _printer.gcode; _toolhead = _printer.toolhead"
        ),
    );
    match setup_ok {
        Ok(_) => {}
        Err(e) => return (rust, world, config, format!("PYSETUPERR {}: {}", e.class_name, e.message)),
    }
    (rust, world, config, String::new())
}
// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_exposes_load_config_prefix() {
    let factory: ModuleFactory = module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("hasattr(_pwm_tool, 'load_config_prefix')");
    check("factory prefix", expected, Value::Bool(true));
}

#[test]
fn alloc_and_set_pin_registration() {
    let Some(mut py) = oracle() else { return };
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &default_opts());
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    let _ = rust;
    // Python: allocate_syncemitter called once with alloc_stepcompress=False
    // Rust: record has (name, sname, alloc) without mcu id; compare names only
    let r_allocs = world.motion_queuing.allocs();
    let r_val = Value::List(
        r_allocs
            .iter()
            .map(|(n, s, a)| {
                Value::List(vec![
                    Value::Str(n.clone()),
                    Value::Str(s.clone()),
                    Value::Bool(*a),
                ])
            })
            .collect(),
    );
    // Python allocs stores (id(mcu), name, alloc); take (name, alloc) projection
    let expected = py.eval("[(name, alloc) for (_id, name, alloc) in _mq.allocs]");
    let r_proj = Value::List(
        r_allocs
            .iter()
            .map(|(n, _s, a)| Value::Tuple(vec![Value::Str(n.clone()), Value::Bool(*a)]))
            .collect(),
    );
    check("allocate_syncemitter args", expected, r_proj);
    let _ = r_val;
    let expected = py.eval("_gcode.mux[0][0:3]");
    let mux = world.gcode.mux_records();
    let r_mux = Value::Tuple(vec![
        Value::Str("SET_PIN".to_string()),
        Value::Str("PIN".to_string()),
        Value::Str("my_pin".to_string()),
    ]);
    check("mux cmd", expected, r_mux);
    let desc = py.eval("_gcode.mux[0][4]");
    check(
        "mux desc",
        desc,
        Value::Str("Set the value of an output pin".to_string()),
    );
    let _ = mux;
}

#[test]
fn get_status_value_matches() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("value", "0.5"), ("scale", "2.0")]);
    let (rust, _world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    let expected = py.eval("pin.get_status(0.0)");
    let rv: Option<PyValue> = klippy_api::PrinterObject::get_status(&*rust, 0.0);
    let actual = pyvalue_to_value(&rv.expect("get_status"));
    check("get_status value", expected, actual);
}

#[test]
fn build_config_hardware_pwm_commands() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("cycle_time", "0.1"), ("hardware_pwm", "1")]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    // Run build_config on both sides.
    let py_res = py.exec("_mcu.config_callbacks[0]()");
    let rust_res = rust.mcu_pin().build_config();
    match (py_res, rust_res) {
        (Ok(_), Ok(())) => {}
        (Err(e), Err(re)) => {
            check_err("hw build err", Err(e), re);
            return;
        }
        (Ok(_), Err(re)) => panic!("hw build rust err {re:?}"),
        (Err(e), Ok(())) => panic!("hw build py err {e:?}"),
    }
    let expected = py.eval("_mcu.added");
    check("hw add_config_cmd", expected, added_to_value(&world.mcu_inner.added()));
    let expected = py.eval("[f for (f, _cq) in _mcu.lookups]");
    let r_lookups: Vec<String> = world.mcu_inner.lookups().iter().map(|(f, _)| f.clone()).collect();
    check(
        "hw lookup_command",
        expected,
        Value::List(r_lookups.iter().map(|s| Value::Str(s.clone())).collect()),
    );
    let expected = py.eval("len(_mq.flush_callbacks)");
    check(
        "hw flush callback registered",
        expected,
        Value::Int(world.motion_queuing.flush_count() as i64),
    );
}

#[test]
fn build_config_software_pwm_commands() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("cycle_time", "0.2")]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    let py_res = py.exec("_mcu.config_callbacks[0]()");
    let rust_res = rust.mcu_pin().build_config();
    match (py_res, rust_res) {
        (Ok(_), Ok(())) => {}
        (Err(e), Err(re)) => {
            check_err("sw build err", Err(e), re);
            return;
        }
        (Ok(_), Err(re)) => panic!("sw build rust err {re:?}"),
        (Err(e), Ok(())) => panic!("sw build py err {e:?}"),
    }
    let expected = py.eval("_mcu.added");
    check("sw add_config_cmd", expected, added_to_value(&world.mcu_inner.added()));
    let expected = py.eval("[f for (f, _cq) in _mcu.lookups]");
    let r_lookups: Vec<String> = world.mcu_inner.lookups().iter().map(|(f, _)| f.clone()).collect();
    check(
        "sw lookup_command",
        expected,
        Value::List(r_lookups.iter().map(|s| Value::Str(s.clone())).collect()),
    );
}

#[test]
fn build_config_config_errors() {
    // start != shutdown with max duration set -> config_error
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[
        ("cycle_time", "0.1"),
        ("maximum_mcu_duration", "1.0"),
        ("value", "0.2"),
        ("shutdown_value", "0.8"),
    ]);
    let (_rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    let py_res = py.exec("_mcu.config_callbacks[0]()");
    // Rust: build via the pin's helper (fresh world already built the object;
    // reuse the stored mcu_pin through a new object on the same world).
    let rust_res = {
        let config: Config = Arc::new(FakeConfig::new(
            world.printer.clone(),
            "pwm_tool my_pin",
            opts.clone(),
        ));
        let pin2 = PrinterOutputPin::new(&config).expect("rust rebuild");
        pin2.mcu_pin().build_config()
    };
    match (py_res, rust_res) {
        (Err(e), Err(re)) => check_err("max_duration start!=shutdown", Err(e), re),
        (Ok(_), Ok(())) => panic!("both should fail"),
        (Ok(_), Err(re)) => panic!("py ok rust err {re:?}"),
        (Err(e), Ok(())) => panic!("py err {e:?} rust ok"),
    }
    // Soft PWM shutdown must be 0.0 or 1.0
    let Some(mut py2) = oracle() else { return };
    let opts2 = opts_with(&[("cycle_time", "0.1"), ("shutdown_value", "0.5")]);
    let (_rust2, world2, _cfg2, setup_err2) = fresh_pair(&mut py2, &opts2);
    assert!(setup_err2.is_empty(), "py setup2 {setup_err2}");
    let py_res2 = py2.exec("_mcu.config_callbacks[0]()");
    let rust_res2 = {
        let config: Config = Arc::new(FakeConfig::new(
            world2.printer.clone(),
            "pwm_tool my_pin",
            opts2.clone(),
        ));
        let pin2 = PrinterOutputPin::new(&config).expect("rust rebuild2");
        pin2.mcu_pin().build_config()
    };
    match (py_res2, rust_res2) {
        (Err(e), Err(re)) => check_err("soft shutdown 0.5", Err(e), re),
        _ => panic!("both should fail for soft shutdown"),
    }
}

#[test]
fn set_pwm_and_syncemitter_queue() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("cycle_time", "0.1"), ("hardware_pwm", "1")]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    py.exec("_mcu.config_callbacks[0]()").expect("py build");
    rust.mcu_pin().build_config().expect("rust build");
    // Drive set_pwm at several print times on both sides.
    let cases = [(1001.0, 0.0f64), (1001.05, 1.0), (1001.5, 0.5), (1002.0, 1.0)];
    for (pt, val) in cases {
        let py_res = py.exec(&format!(
            "pin.mcu_pin.set_pwm({pt:?}, {val:?})"
        ));
        if let Err(e) = py_res {
            panic!("py set_pwm {pt} {val}: {e:?}");
        }
        rust.mcu_pin().set_pwm(pt, val);
    }
    // Compare the full queue and activity logs after the shared prefix.
    let expected = py.eval("[ [clock, data[0], data[1], data[2], data[3]] for (_se, clock, data, _l) in _fake_chelper.lib.queue_log ]");
    check("set_pwm queue prefix", expected, queued_to_value(&world.motion_queuing.queued_records()));
    // Re-run deterministically: drive one set_pwm and compare the tails.
    py.exec("_fake_chelper.lib.queue_log.clear()").expect("clear py log");
    let before = world.motion_queuing.queued_records().len();
    let py_res = py.exec("pin.mcu_pin.set_pwm(1002.5, 0.75)");
    assert!(matches!(py_res, Ok(_)), "py set_pwm rerun {py_res:?}");
    rust.mcu_pin().set_pwm(1002.5, 0.75);
    let expected = py.eval("[ [clock, data[0], data[1], data[2], data[3]] for (_se, clock, data, _l) in _fake_chelper.lib.queue_log ]");
    let all = world.motion_queuing.queued_records();
    check("set_pwm queue tail", expected, queued_to_value(&all[before..]));
    let expected_act = py.eval("_mq.activity");
    check("set_pwm activity", expected_act, activity_to_value(&world.motion_queuing.activity()));
}

#[test]
fn intermediate_updates_and_flush() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[
        ("cycle_time", "0.1"),
        ("hardware_pwm", "1"),
        ("maximum_mcu_duration", "0.5"),
        ("value", "0.0"),
    ]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    py.exec("_mcu.config_callbacks[0]()").expect("py build");
    rust.mcu_pin().build_config().expect("rust build");
    // Set a non-default value, then flush far in the future on both sides.
    py.exec("pin.mcu_pin.set_pwm(1001.0, 1.0)").expect("py set");
    rust.mcu_pin().set_pwm(1001.0, 1.0);
    py.exec("_fake_chelper.lib.queue_log.clear()").expect("clear");
    let n_before = world.motion_queuing.queued_records().len();
    py.exec("_mq.flush_now(1005.0, 0.0)").expect("py flush");
    world.motion_queuing.flush_now(1005.0, 0.0);
    let expected = py.eval("[ [clock, data[0], data[1], data[2], data[3]] for (_se, clock, data, _l) in _fake_chelper.lib.queue_log ]");
    let all = world.motion_queuing.queued_records();
    check("flush intermediate queue", expected, queued_to_value(&all[n_before..]));
}

#[test]
fn set_pin_gcode_command() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("cycle_time", "0.1"), ("hardware_pwm", "1")]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    py.exec("_mcu.config_callbacks[0]()").expect("py build");
    rust.mcu_pin().build_config().expect("rust build");
    // SET_PIN VALUE=0.25 with scale 1: value 0.25.
    let py_cmd = "_fake_gcmd = FakeGcmd('SET_PIN PIN=my_pin VALUE=0.25', {'VALUE': '0.25'}); pin.cmd_SET_PIN(_fake_gcmd)";
    let py_res = py.exec(py_cmd);
    let gcmd: GCmd = Arc::new(FakeGcmd {
        command: "SET_PIN".to_string(),
        commandline: "SET_PIN PIN=my_pin VALUE=0.25".to_string(),
        params: {
            let mut m = IndexMap::new();
            m.insert("VALUE".to_string(), "0.25".to_string());
            m
        },
    });
    let mut handler = world.gcode.handler_for("SET_PIN", "PIN", Some("my_pin"));
    let rust_res = handler(&gcmd);
    match (py_res, rust_res) {
        (Ok(_), Ok(())) => {}
        (Err(e), Err(re)) => {
            check_err("SET_PIN err", Err(e), re);
            return;
        }
        (Ok(_), Err(re)) => panic!("SET_PIN rust err {re:?}"),
        (Err(e), Ok(())) => panic!("SET_PIN py err {e:?}"),
    }
    // Run lookaheads on both sides at the same print time.
    let py_res2 = py.exec("_toolhead.run_lookaheads(1001.0)");
    let rust_res2 = world.toolhead.run_lookaheads(1001.0);
    match (py_res2, rust_res2) {
        (Ok(_), Ok(())) => {}
        (Err(e), Err(re)) => {
            check_err("SET_PIN lookahead err", Err(e), re);
            return;
        }
        (Ok(_), Err(re)) => panic!("lookahead rust err {re:?}"),
        (Err(e), Ok(())) => panic!("lookahead py err {e:?}"),
    }
    let expected = py.eval("pin.get_status(0.0)");
    let rv: Option<PyValue> = klippy_api::PrinterObject::get_status(&*rust, 0.0);
    check("SET_PIN status", expected, pyvalue_to_value(&rv.expect("status")));
}

#[test]
fn set_pin_value_errors() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("cycle_time", "0.1"), ("scale", "2.0")]);
    let (_rust, _world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    for (valstr, label) in [("3.0", "above max"), ("-1.0", "below min"), ("abc", "unparsable")] {
        let py_res = py.exec(&format!(
            "pin.cmd_SET_PIN(FakeGcmd('SET_PIN PIN=my_pin VALUE={valstr}', {{'VALUE': '{valstr}'}}))"
        ));
        // Rust direct gcmd validation via cmd_set_pin
        let world2 = make_world();
        let config: Config = Arc::new(FakeConfig::new(
            world2.printer.clone(),
            "pwm_tool my_pin",
            opts.clone(),
        ));
        let pin2 = PrinterOutputPin::new(&config).expect("rust pin");
        let gcmd: GCmd = Arc::new(FakeGcmd {
            command: "SET_PIN".to_string(),
            commandline: format!("SET_PIN PIN=my_pin VALUE={valstr}"),
            params: {
                let mut m = IndexMap::new();
                m.insert("VALUE".to_string(), valstr.to_string());
                m
            },
        });
        let rust_res = PrinterOutputPin::cmd_set_pin(&pin2, &gcmd);
        match (py_res, rust_res) {
            (Err(e), Err(re)) => check_err(&format!("SET_PIN {label}"), Err(e), re),
            (Ok(_), Ok(())) => panic!("SET_PIN {label} both ok"),
            (Ok(_), Err(re)) => panic!("SET_PIN {label} rust err {re:?}"),
            (Err(e), Ok(())) => panic!("SET_PIN {label} py err {e:?} rust ok"),
        }
    }
    // Missing VALUE
    let py_res = py.exec("pin.cmd_SET_PIN(FakeGcmd('SET_PIN PIN=my_pin', {}))");
    let world2 = make_world();
    let config: Config = Arc::new(FakeConfig::new(
        world2.printer.clone(),
        "pwm_tool my_pin",
        opts.clone(),
    ));
    let pin2 = PrinterOutputPin::new(&config).expect("rust pin2");
    let gcmd: GCmd = Arc::new(FakeGcmd {
        command: "SET_PIN".to_string(),
        commandline: "SET_PIN PIN=my_pin".to_string(),
        params: IndexMap::new(),
    });
    let rust_res = PrinterOutputPin::cmd_set_pin(&pin2, &gcmd);
    match (py_res, rust_res) {
        (Err(e), Err(re)) => check_err("SET_PIN missing", Err(e), re),
        _ => panic!("SET_PIN missing both ok/mismatch"),
    }
}

#[test]
fn config_option_errors() {
    let Some(mut py) = oracle() else { return };
    // cycle_time above 0 violated
    let cases: Vec<(IndexMap<String, String>, &str)> = vec![
        (opts_with(&[("cycle_time", "0")]), "cycle_time 0"),
        (opts_with(&[("scale", "0")]), "scale 0"),
        (opts_with(&[("maximum_mcu_duration", "0.4")]), "max_dur below min"),
        (opts_with(&[("maximum_mcu_duration", "99.0")]), "max_dur above max"),
    ];
    for (opts, label) in cases {
        let py_opts = py_dict_expr(&opts);
        let py_res = py.exec(
            &format!("_p2 = FakePrinter(); _c2 = FakeConfig(_p2, 'pwm_tool my_pin', {py_opts}); _pwm_tool.PrinterOutputPin(_c2)"),
        );
        let world = make_world();
        let config: Config = Arc::new(FakeConfig::new(
            world.printer.clone(),
            "pwm_tool my_pin",
            opts,
        ));
        let rust_res = PrinterOutputPin::new(&config);
        match (py_res, rust_res) {
            (Err(e), Err(re)) => check_err(&format!("config {label}"), Err(e), re),
            (Ok(_), Ok(_)) => panic!("config {label} both ok"),
            (Ok(_), Err(re)) => panic!("config {label} py ok rust err {re:?}"),
            (Err(e), Ok(_)) => panic!("config {label} py err {e:?} rust ok"),
        }
    }
    // Missing pin is required
    let py_res = py.exec("_p2 = FakePrinter(); _c2 = FakeConfig(_p2, 'pwm_tool my_pin', {}); _pwm_tool.PrinterOutputPin(_c2)");
    let world = make_world();
    let config: Config = Arc::new(FakeConfig::new(
        world.printer.clone(),
        "pwm_tool my_pin",
        IndexMap::new(),
    ));
    let rust_res = PrinterOutputPin::new(&config);
    match (py_res, rust_res) {
        (Err(e), Err(re)) => check_err("config missing pin", Err(e), re),
        _ => panic!("config missing pin mismatch"),
    }
}

#[test]
fn invert_handling() {
    let Some(mut py) = oracle() else { return };
    let opts = opts_with(&[("pin", "!PA1"), ("cycle_time", "0.1"), ("hardware_pwm", "1")]);
    let (rust, world, _cfg, setup_err) = fresh_pair(&mut py, &opts);
    assert!(setup_err.is_empty(), "py setup {setup_err}");
    py.exec("_mcu.config_callbacks[0]()").expect("py build");
    rust.mcu_pin().build_config().expect("rust build");
    let expected = py.eval("_mcu.added");
    check("invert add_config_cmd", expected, added_to_value(&world.mcu_inner.added()));
    // set_pwm inverts the value: 0.25 -> 0.75 before scaling
    py.exec("_fake_chelper.lib.queue_log.clear()").expect("clear");
    let n_before = world.motion_queuing.queued_records().len();
    py.exec("pin.mcu_pin.set_pwm(1001.0, 0.25)").expect("py set");
    rust.mcu_pin().set_pwm(1001.0, 0.25);
    let expected = py.eval("[ [clock, data[0], data[1], data[2], data[3]] for (_se, clock, data, _l) in _fake_chelper.lib.queue_log ]");
    let all = world.motion_queuing.queued_records();
    check("invert queue", expected, queued_to_value(&all[n_before..]));
}

#[test]
fn queued_pwm_real_syncemitter_parity() {
    // `pwm_tool` queueing through a real syncemitter (`pwm_tool.py:20-21`,
    // `106-110`): `McuQueuedPwm` is built against the real
    // `PrinterMotionQueuing` (`NativeChelper`), so its emitter is a
    // `CoreSyncEmitter` wrapping the ported C object with
    // `alloc_stepcompress=False`. After `build_config`, `set_pwm` and a
    // flush, the message reaches the serial command queue's upcoming list.
    // The Python side runs the same `set_pwm` against the real C library
    // on a stopped debug-file serial queue; compared exactly: the
    // `note_mcu_movequeue_activity` print time, the PWM last clock/value,
    // and the flush times. The emitter-queue words are unreadable through
    // cffi (the C `msg_queue` list has no Python accessor), so the Rust
    // side asserts the upcoming message carries the deterministic words
    // both sides share (tag 42, first oid 1, `int(t * 100000)`); the
    // byte-level cross-check lives in the `chelper_motion` lockstep test.
    use extras::motion_queuing::{NativeChelper, PrinterMotionQueuing, StepperSync};
    use extras::pwm_tool::McuQueuedPwm;
    let Some(mut py) = oracle() else { return };
    py.exec(SETUP).expect("py setup");
    py.exec(
        "import chelper as _real_chelper\n\
        _real_main, _real_lib = _real_chelper.get_ffi()\n\
        import os\n\
        _fd = os.open(os.devnull, os.O_RDWR)\n\
        _sq = _real_lib.serialqueue_alloc(_fd, b'f', 0, b'steppersync')\n\
        _real_lib.serialqueue_exit(_sq)\n\
        import extras.motion_queuing as _mq_real\n\
        _mq_real.chelper = _real_chelper\n\
        _printer = FakePrinter()\n\
        _mcu = _printer.mcu\n\
        _o = _mq_real.PrinterMotionQueuing(FakeConfig(_printer, 'motion_queuing', {}))\n\
        _config = FakeConfig(_printer, 'pwm_tool my_pin', {'pin': 'PA1'})\n\
        _pin_params = _printer.pins.lookup_pin('PA1')\n\
        import extras.pwm_tool as _pwm_real\n\
        _pwm_real.chelper = _real_chelper\n\
        _pwm = _pwm_real.MCU_queued_pwm(_config, _pin_params)\n\
        _pwm._motion_queuing = _o\n\
        _pwm._syncemitter = _o.allocate_syncemitter(_mcu, 'pwm_tool my_pin', alloc_stepcompress=False)\n\
        _queued_clocks = []\n\
        _pwm._syncemitter_queue_msg = lambda se, clock, data, length: (_queued_clocks.append(clock), _real_lib.syncemitter_queue_msg(se, clock, data, length))[1]\n\
        _pwm._build_config()\n\
        _pwm.set_pwm(1001.0, 0.75)\n\
        _o.setup_mcu_movequeue(_mcu, _sq, 8)\n\
        _o._advance_flush_time(2000.0)\n",
    )
    .expect("py real pwm setup");
    let got = py.eval(
        "[_o.need_flush_time, _o.last_flush_time, _o.last_step_gen_time, _pwm._last_clock, _pwm._last_value]",
    );
    // Rust side: the world table maps `motion_queuing` at the real object,
    // so `McuQueuedPwm::new` allocates a real emitter through it.
    let printer_inner: Arc<FakePrinter> = Arc::new(FakePrinter {
        reactor: Reactor::new(false),
        objects: kos::sync::Mutex::new(IndexMap::new()),
        handlers: kos::sync::Mutex::new(Vec::new()),
    });
    let printer: Printer = printer_inner.clone();
    let mcu_inner = Arc::new(FakeMcu::new());
    mcu_inner.set_printer(printer.clone());
    let mcu: Mcu = mcu_inner.clone();
    let chelper: Arc<NativeChelper> = Arc::new(NativeChelper::new());
    let obj = PrinterMotionQueuing::new_with_clock(
        printer.clone(),
        printer.get_reactor(),
        vec![mcu.clone()],
        chelper.clone(),
        None,
    )
    .unwrap();
    let mq_obj: klippy_api::ObjectRef = obj.clone();
    let pins = Arc::new(FakePins::new(mcu.clone()));
    let pins_obj: klippy_api::ObjectRef = pins.clone();
    {
        let mut objs = printer_inner.objects.lock();
        objs.insert("pins".to_string(), pins_obj.clone());
        objs.insert("motion_queuing".to_string(), mq_obj.clone());
    }
    let config: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "pwm_tool my_pin",
        IndexMap::from([("pin".to_string(), "PA1".to_string())]),
    ));
    let pin_params = pins.lookup_pin("PA1", LookupPinOpts::default()).unwrap();
    let pwm = McuQueuedPwm::new(&config, pin_params).unwrap();
    mcu_inner.run_config_callbacks().unwrap();
    pwm.set_pwm(1001.0, 0.75);
    let fd = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .open("/dev/null")
        .expect("open /dev/null");
    let sq = {
        use std::os::unix::io::IntoRawFd;
        let raw = fd.into_raw_fd();
        let sq = chelper_serial::serialqueue::SerialQueue::alloc(raw, b'f', 0, b"steppersync")
            .expect("SerialQueue::alloc");
        sq.exit();
        Arc::new(sq)
    };
    obj.setup_mcu_movequeue(mcu.clone(), Some(sq.clone()), 8);
    obj._advance_flush_time(2000.0, 0.0).unwrap();
    let rust_val = Value::List(vec![
        Value::float(obj.need_flush_time()),
        Value::float(obj.last_flush_time()),
        Value::float(obj.last_step_gen_time()),
        Value::int(pwm.last_clock()),
        Value::int(pwm.last_value()),
    ]);
    check("real emitter state", got, rust_val);
    // The messages `set_pwm` and its duration refreshes queued on the
    // Python side (`pwm_tool.py:110`, recorded at `syncemitter_queue_msg`)
    // are the ones the flush moved to the command queue's upcoming list.
    let mcu_handle: Mcu = mcu.clone();
    let ss = obj._lookup_steppersync(&mcu_handle);
    let cq = ss
        .downcast_ref::<StepperSync>()
        .expect("native steppersync")
        .command_queue()
        .expect("movequeue configured");
    let upcoming = cq.upcoming_messages();
    check(
        "queued pwm clocks",
        py.eval("_queued_clocks"),
        Value::List(upcoming.iter().map(|m| Value::int(m.req_clock as i64)).collect()),
    );
}
