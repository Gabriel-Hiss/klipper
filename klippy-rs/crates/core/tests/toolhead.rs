//! Parity tests for `klippy_core::toolhead` (port of `klippy/toolhead.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`: the real
//! `klippy/toolhead.py` runs in CPython 3.8.2 together with the real `gcode` (its `Coord`
//! and `GCodeCommand`), `configfile.ConfigWrapper` (over a `RawConfigParser`), `stepper`
//! and `kinematics.extruder` modules. Only the objects around the toolhead are Python
//! fakes: the printer, MCU, `motion_queuing`, `gcode`, a kinematics module, extra axes and
//! `chelper` (whose `trapq_set_position` the toolhead calls through cffi).
//!
//! The port runs against Rust fakes of the contract traits with the same behaviour. Both
//! sides append every call they receive (printer lookups, MCU, `motion_queuing`, kinematics,
//! extra axes, `trapq_append`, `respond_info`, events) to one ordered log. A scenario is a
//! list of operations; after each one the test compares the operation's result or raised
//! error (class and message), the new log entries, the log records, `get_status`,
//! `check_busy`, `stats`, the position, the velocity limits and the trapq position set by
//! `set_position`. The config access tracking (options read with their defaults) is compared
//! after construction.
//!
//! Not compared: `reactor.monotonic()` values (the port uses the real reactor, so the
//! `curtime` argument of `toolhead:sync_print_time` is masked), loops that call
//! `reactor.pause` with a constant estimated print time (they never end in the Python
//! either), and the timer machinery behind `update_timer`. The priming timer is covered by
//! running the real reactor in `priming_handler_matches_python`.

use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GCGetFloatOpts,
    GetIntOpts as GCGetIntOpts, GetOpts as GCGetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueue, CommandQueryWrapper, CommandWrapper,
    ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin, PinParams, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{lookup_interface, EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{Stepper, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, ExtraAxisApi, FlushCallback, HomingStateHandle, HomingTargetApi, Kinematics,
    KinematicsApi, MotionQueuingApi, MoveApi, PrinterExtruder, PrinterExtruderApi,
    SyncEmitterHandle, ToolHeadApi, TrapqAppend,
};
use klippy_core::motion_handles::{trapq_of, TrapqHandle};
use klippy_core::toolhead::{
    add_printer_objects, module_factory, register_extruder_objects, register_kinematics_loader,
    DummyExtruder, LookAheadQueue, Move, ToolHead, ToolHeadCommandHelper,
};
use kos::sync::Mutex;
use pycompat::text::float_str;
use pycompat::{PyInt, PyValue};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use reactor::{Reactor, NEVER, NOW};
use std::cell::RefCell;
use std::sync::{Arc, LazyLock};

// ---------------------------------------------------------------------------
// Python side: the real modules plus minimal fakes of the surrounding objects.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, math, logging, configparser
import configfile
import gcode as gcode_mod

LOG = []
TSP = []
LOGLINES = []

class _LogHandler(logging.Handler):
    def emit(self, record):
        LOGLINES.append((record.levelname, record.getMessage(), record.exc_info is not None))
logging.getLogger().addHandler(_LogHandler())

# toolhead.set_position calls ffi_lib.trapq_set_position through cffi.
class _FfiLib:
    def trapq_set_position(self, trapq, print_time, x, y, z):
        TSP.append((print_time, x, y, z))
_chelper = types.ModuleType('chelper')
_chelper.get_ffi = lambda: (None, _FfiLib())
sys.modules['chelper'] = _chelper
# toolhead imports `mcu` but never touches it.
sys.modules['mcu'] = types.ModuleType('mcu')

import toolhead

class _Ctx:
    def __enter__(self):
        return self
    def __exit__(self, *a):
        return False

class FakeReactor:
    NEVER = 9999999999999999.
    NOW = 0.
    def monotonic(self):
        return 0.
    def pause(self, waketime):
        return waketime
    def register_timer(self, cb, waketime=NEVER):
        return ('timer', cb)
    def unregister_timer(self, timer):
        pass
    def update_timer(self, timer, waketime):
        pass
    def assert_no_pause(self):
        return _Ctx()
_reactor = FakeReactor()

class FakePins:
    class error(Exception):
        pass

class _Stub:
    pass

class _Comp:
    def complete(self, value):
        pass

class FakePrinter:
    config_error = configfile.error
    command_error = gcode_mod.CommandError
    def __init__(self, loadable):
        self.objects = {}
        self.handlers = {}
        self.loadable = loadable
    def get_reactor(self):
        return _reactor
    def add_object(self, name, obj):
        LOG.append(('add_object', name))
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile.sentinel):
        LOG.append(('lookup_object', name))
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=configfile.sentinel):
        LOG.append(('load_object', section))
        if section in self.objects:
            return self.objects[section]
        if section in self.loadable:
            self.objects[section] = _Stub()
            return self.objects[section]
        if default is not configfile.sentinel:
            return default
        raise self.config_error("Unable to load module '%s'" % (section,))
    def register_event_handler(self, event, callback):
        LOG.append(('register_event_handler', event))
        self.handlers.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        shown = tuple(params)
        if event == 'toolhead:sync_print_time':
            shown = (None,) + shown[1:]
        LOG.append(('send_event', event, shown))
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def set_rollover_info(self, name, info, log=True):
        LOG.append(('set_rollover_info', name, info, log))
    def invoke_shutdown(self, msg, details={}):
        LOG.append(('invoke_shutdown', msg, details))

class FakeMcu:
    def __init__(self, fileoutput, est):
        self.fileoutput = fileoutput
        self.est = est
    def is_fileoutput(self):
        LOG.append(('mcu.is_fileoutput',))
        return self.fileoutput
    def estimated_print_time(self, eventtime):
        LOG.append(('mcu.estimated_print_time',))
        return self.est

class FakeMq:
    def __init__(self):
        self.flush_cbs = []
        self.step_restart = 0.
    def register_flush_callback(self, cb, can_add_trapq=False):
        LOG.append(('mq.register_flush_callback', can_add_trapq))
        self.flush_cbs.append(cb)
    def allocate_trapq(self):
        LOG.append(('mq.allocate_trapq',))
        return ('trapq',)
    def lookup_trapq_append(self):
        LOG.append(('mq.lookup_trapq_append',))
        def append(trapq, *args):
            LOG.append(('trapq_append',) + args)
        return append
    def get_kin_flush_delay(self):
        LOG.append(('mq.get_kin_flush_delay',))
        return 0.001
    def calc_step_gen_restart(self, est_print_time):
        LOG.append(('mq.calc_step_gen_restart', est_print_time))
        return self.step_restart
    def flush_all_steps(self):
        LOG.append(('mq.flush_all_steps',))
    def note_mcu_movequeue_activity(self, mq_time, is_step_gen=True):
        LOG.append(('mq.note_mcu_movequeue_activity', mq_time, is_step_gen))
    def wipe_trapq(self, trapq):
        LOG.append(('mq.wipe_trapq',))
    def drip_update_time(self, start_time, end_time, drip_completion):
        LOG.append(('mq.drip_update_time', start_time, end_time))
        drip_completion.complete(end_time)

class FakeKin:
    def __init__(self):
        self.limits = None
        self.fail = False
    def check_move(self, move):
        LOG.append(('kin.check_move', move.start_pos, move.end_pos, move.move_d))
        if self.limits is not None:
            move.limit_speed(*self.limits)
        if self.fail:
            raise move.move_error()
    def set_position(self, newpos, homing_axes):
        LOG.append(('kin.set_position', tuple(newpos), homing_axes))
    def get_status(self, eventtime):
        LOG.append(('kin.get_status',))
        return {'homed_axes': 'xy',
                'axis_minimum': gcode_mod.Coord([0., 0., 0.]),
                'axis_maximum': gcode_mod.Coord([200., 200., 180.]),
                'extruder': 'kinextr'}

class FakeAxis:
    def __init__(self, name, gid):
        self.name = name
        self.gid = gid
        self.junction = None
        self.fail = None
    def get_name(self):
        return self.name
    def get_axis_gcode_id(self):
        return self.gid
    def get_trapq(self):
        return None
    def check_move(self, move, ea_index):
        LOG.append(('axis.check_move', self.name, ea_index,
                    move.axes_d[ea_index], move.axes_r[ea_index]))
        if self.fail is not None:
            raise move.move_error(self.fail)
    def calc_junction(self, prev_move, move, ea_index):
        LOG.append(('axis.calc_junction', self.name, ea_index))
        if self.junction is not None:
            return self.junction
        return move.max_cruise_v2
    def process_move(self, print_time, move, ea_index):
        LOG.append(('axis.process_move', self.name, print_time, ea_index,
                    move.start_v, move.cruise_v, move.accel_t, move.cruise_t,
                    move.decel_t, move.axes_d[ea_index]))

class FakeGCode:
    Coord = gcode_mod.Coord
    def __init__(self):
        self.handlers = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        LOG.append(('gcode.register_command', cmd, when_not_ready, desc))
        self.handlers[cmd] = func
    def respond_info(self, msg, log=True):
        LOG.append(('respond_info', msg, log))
    def respond_raw(self, msg):
        LOG.append(('respond_raw', msg))

class LoggedConfig(configfile.ConfigWrapper):
    def has_section(self, section):
        LOG.append(('config.has_section', section))
        return super().has_section(section)

def _kin_module(name, loader):
    mod = types.ModuleType('kinematics.' + name)
    mod.load_kinematics = loader
    sys.modules['kinematics.' + name] = mod

def _raise_cfg(th, cfg):
    raise cfg.error('kin config boom')
def _raise_pins(th, cfg):
    raise cfg.get_printer().lookup_object('pins').error('kin pins boom')
def _raise_generic(th, cfg):
    raise ValueError('kin generic boom')
def _raise_command(th, cfg):
    raise cfg.get_printer().command_error('kin command boom')
_kin_module('fakekin', lambda th, cfg: cfg.get_printer().lookup_object('_kin'))
_kin_module('cfgerr', _raise_cfg)
_kin_module('pinserr', _raise_pins)
_kin_module('generr', _raise_generic)
_kin_module('cmderr', _raise_command)

def make_world(ini, fileoutput, est, has, loadable, axes):
    global printer, mcu, mq, kin, gc, fileconfig, access, cfg, th, helper, AX
    del LOG[:]
    del TSP[:]
    del LOGLINES[:]
    printer = FakePrinter(loadable)
    mcu = FakeMcu(fileoutput, est)
    mq = FakeMq()
    kin = FakeKin()
    gc = FakeGCode()
    if has[0]:
        printer.objects['mcu'] = mcu
    if has[1]:
        printer.objects['motion_queuing'] = mq
    if has[2]:
        printer.objects['gcode'] = gc
    if has[3]:
        printer.objects['pins'] = FakePins()
    printer.objects['_kin'] = kin
    fileconfig = configparser.RawConfigParser(strict=False, inline_comment_prefixes=(';', '#'))
    fileconfig.read_string(ini)
    access = {}
    cfg = LoggedConfig(printer, fileconfig, access, 'printer')
    th = None
    helper = None
    AX = {name: FakeAxis(name, gid) for name, gid in axes}

def _try(f):
    try:
        return ('ok', f())
    except Exception as e:
        return ('exc', type(e).__name__, str(e))

def _none(*args):
    return None

def _set(obj, name, value):
    setattr(obj, name, value)

def _observe():
    return (th.get_status(0.0), th.check_busy(0.0), th.stats(0.0), th.get_position(),
            th.get_max_velocity(), TSP[-1] if TSP else None)

def _drain():
    log = list(LOG)
    del LOG[:]
    lines = list(LOGLINES)
    del LOGLINES[:]
    return log, lines

def _mk(with_helper):
    global th, helper
    th = None
    helper = None
    th = toolhead.ToolHead(cfg)
    if with_helper:
        printer.add_object('toolhead', th)
        helper = toolhead.ToolHeadCommandHelper(cfg)

def _tracking():
    return [(k, v) for k, v in access.items()]

def _construct(with_helper):
    res = _try(lambda: _mk(with_helper))
    obs = _try(_observe) if th is not None else None
    log, lines = _drain()
    return (res, obs, log, _tracking(), lines)

def _step(f):
    res = _try(f)
    obs = _try(_observe)
    log, lines = _drain()
    return (res, obs, log, lines)

def _add_all():
    toolhead.add_printer_objects(cfg)

def _add_objects():
    res = _try(_add_all)
    log, lines = _drain()
    return (res, log, list(printer.objects.keys()), lines)

def _cmd(cmd, line, params):
    gcmd = gcode_mod.GCodeCommand(gc, cmd, line, params, False)
    gc.handlers[cmd](gcmd)

def _mfields(m):
    return (m.start_pos, m.end_pos, m.axes_d, m.axes_r, m.move_d, m.is_kinematic_move,
            m.accel, m.max_cruise_v2)

def _mtiming(m):
    return (m.start_v, m.cruise_v, m.accel_t, m.cruise_t, m.decel_t, m.accel, m.end_pos)

def _move_case(start, end, speed, limit, next_limit, junction):
    m = toolhead.Move(th, start, end, speed)
    before = _mfields(m)
    if limit is not None:
        m.limit_speed(*limit)
    if next_limit is not None:
        m.limit_next_junction_speed(next_limit)
    after = _mfields(m)
    m.set_junction(*junction)
    return (before, after, _mtiming(m))

def _move_error(start, end, msg):
    m = toolhead.Move(th, start, end, 10.)
    if msg is None:
        e = m.move_error()
    else:
        e = m.move_error(msg)
    return (type(e).__name__, str(e))

def _la_scenario(start, moves, limits, next_limits, flush_plan):
    la = toolhead.LookAheadQueue()
    la.set_flush_time(2.0)
    out = []
    pos = list(start)
    for i, (end, speed) in enumerate(moves):
        m = toolhead.Move(th, pos, end, speed)
        if limits[i] is not None:
            m.limit_speed(*limits[i])
        if next_limits[i] is not None:
            m.limit_next_junction_speed(next_limits[i])
        pos = list(m.end_pos)
        r = la.add_move(m)
        out.append(('add', i, bool(r), la.is_empty()))
        lazy = flush_plan.get(i)
        if lazy is not None:
            got = la.flush(lazy=lazy)
            out.append(('flush', i, lazy, [_mtiming(x) for x in got], la.is_empty(),
                        la.get_last() is None))
    got = la.flush()
    out.append(('final', [_mtiming(x) for x in got], la.is_empty(), la.get_last() is None))
    return out
"#;

// ---------------------------------------------------------------------------
// Rust side: shared log, fakes of the contract traits.
// ---------------------------------------------------------------------------

type Log = Arc<Mutex<Vec<Value>>>;

fn vs(s: &str) -> Value {
    Value::str(s)
}
fn vf(x: f64) -> Value {
    Value::float(x)
}
fn vi(x: i64) -> Value {
    Value::Int(x)
}
fn vt(items: Vec<Value>) -> Value {
    Value::Tuple(items)
}
fn vfs(vals: &[f64]) -> Value {
    Value::Tuple(vals.iter().map(|v| vf(*v)).collect())
}
fn vfl(vals: &[f64]) -> Value {
    Value::List(vals.iter().map(|v| vf(*v)).collect())
}
fn push(log: &Log, items: Vec<Value>) {
    log.lock().push(Value::Tuple(items));
}

thread_local! {
    static LINES: RefCell<Vec<Value>> = const { RefCell::new(Vec::new()) };
}

/// Capture `logging` records of the test thread as `(level, message, has traceback)`.
fn install_sink() {
    static SINK: LazyLock<bool> = LazyLock::new(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            let line = vt(vec![
                vs(rec.level.name()),
                vs(rec.message),
                Value::Bool(rec.exc_text.is_some()),
            ]);
            LINES.with(|l| l.borrow_mut().push(line));
        }))
    });
    assert!(*SINK, "another log sink was installed first");
}

fn drain_lines() -> Value {
    Value::List(LINES.with(|l| std::mem::take(&mut *l.borrow_mut())))
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::Int(n.0.to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v))).collect(),
        ),
        PyValue::Host(h) => Value::Opaque { type_name: "host".into(), addr: format!("0x{:x}", h) },
    }
}

struct Stub;
klippy_api::printer_object!(Stub);

struct FakePins;
klippy_api::printer_object!(FakePins);

// ---- MCU ----

struct FakeMcu {
    log: Log,
    printer: Printer,
    fileoutput: bool,
    est: Mutex<f64>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn is_fileoutput(&self) -> bool {
        push(&self.log, vec![vs("mcu.is_fileoutput")]);
        self.fileoutput
    }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(0)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _o: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        time as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        1.0
    }
    fn lookup_command(&self, _m: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn lookup_query_command(
        &self,
        _m: &str,
        _r: &str,
        _o: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _m: &str,
        _o: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("not used")
    }
    fn check_valid_response(&self, _m: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> {
        Ok(1.0)
    }
    fn print_time_to_clock(&self, t: f64) -> i64 {
        t as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        push(&self.log, vec![vs("mcu.estimated_print_time")]);
        *self.est.lock()
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 1.0))
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
}
klippy_api::printer_object!(FakeMcu; dyn McuApi);

// ---- motion_queuing ----

struct FakeMq {
    log: Log,
    trapq: Trapq,
    flush_cbs: Mutex<Vec<(FlushCallback, bool)>>,
    step_restart: Mutex<f64>,
}

impl MotionQueuingApi for FakeMq {
    fn allocate_trapq(&self) -> Trapq {
        push(&self.log, vec![vs("mq.allocate_trapq")]);
        Arc::clone(&self.trapq)
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        push(&self.log, vec![vs("mq.lookup_trapq_append")]);
        let log = Arc::clone(&self.log);
        Arc::new(move |_t, pt, at, ct, dt, x, y, z, rx, ry, rz, sv, cv, acc| {
            push(
                &log,
                vec![
                    vs("trapq_append"),
                    vf(pt),
                    vf(at),
                    vf(ct),
                    vf(dt),
                    vf(x),
                    vf(y),
                    vf(z),
                    vf(rx),
                    vf(ry),
                    vf(rz),
                    vf(sv),
                    vf(cv),
                    vf(acc),
                ],
            );
        })
    }
    fn wipe_trapq(&self, _t: &Trapq) {
        push(&self.log, vec![vs("mq.wipe_trapq")]);
    }
    fn allocate_syncemitter(
        &self,
        _m: Mcu,
        _n: &str,
        _a: bool,
    ) -> KResult<SyncEmitterHandle> {
        Err(KlippyError::Mcu("not used".to_string()))
    }
    fn queue_syncemitter_msg(&self, _s: &SyncEmitterHandle, _req_clock: u64, _data: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: Mcu, _s: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _c: i64) {}
    fn register_flush_callback(&self, cb: FlushCallback, can_add_trapq: bool) {
        push(&self.log, vec![vs("mq.register_flush_callback"), Value::Bool(can_add_trapq)]);
        self.flush_cbs.lock().push((cb, can_add_trapq));
    }
    fn unregister_flush_callback(&self, _cb: &FlushCallback) {}
    fn register_flush_callback_named(&self, cb: FlushCallback, can_add_trapq: bool, _name: Option<&str>, _obj: Option<&str>) {
        self.register_flush_callback(cb, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 {
        push(&self.log, vec![vs("mq.get_kin_flush_delay")]);
        0.001
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        push(&self.log, vec![vs("mq.flush_all_steps")]);
        Ok(())
    }
    fn calc_step_gen_restart(&self, est_print_time: f64) -> f64 {
        push(&self.log, vec![vs("mq.calc_step_gen_restart"), vf(est_print_time)]);
        *self.step_restart.lock()
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        push(
            &self.log,
            vec![vs("mq.note_mcu_movequeue_activity"), vf(mq_time), Value::Bool(is_step_gen)],
        );
    }
    fn drip_update_time(
        &self,
        start_time: f64,
        end_time: f64,
        drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        push(&self.log, vec![vs("mq.drip_update_time"), vf(start_time), vf(end_time)]);
        drip_completion.complete(true);
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}
klippy_api::printer_object!(FakeMq; dyn MotionQueuingApi);

// ---- kinematics ----

struct FakeKin {
    log: Log,
    limits: Mutex<Option<(f64, f64)>>,
    fail: Mutex<bool>,
}

impl KinematicsApi for FakeKin {
    fn class_name(&self) -> &'static str {
        "FakeKin"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn calc_position(&self, _s: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(vec![None, None, None, None])
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        push(&self.log, vec![vs("kin.set_position"), vfs(&newpos), vs(homing_axes)]);
        Ok(())
    }
    fn check_move(&self, mv: &dyn MoveApi) -> KResult<()> {
        push(
            &self.log,
            vec![
                vs("kin.check_move"),
                vfs(&mv.start_pos().0),
                vfs(&mv.end_pos().0),
                vf(mv.move_d()),
            ],
        );
        let limits = *self.limits.lock();
        if let Some((speed, accel)) = limits {
            mv.limit_speed(speed, accel)?;
        }
        if *self.fail.lock() {
            return Err(mv.move_error("Move out of range".to_string()));
        }
        Ok(())
    }
    fn home(&self, _h: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> {
        push(&self.log, vec![vs("kin.get_status")]);
        // `gcode.Coord` pads a short tuple with the int 0.
        let coord = |x: f64, y: f64, z: f64| {
            PyValue::Tuple(vec![
                PyValue::Float(x),
                PyValue::Float(y),
                PyValue::Float(z),
                PyValue::Int(PyInt::from_i64(0)),
            ])
        };
        let mut d = IndexMap::new();
        d.insert(PyValue::Str("homed_axes".to_string()), PyValue::Str("xy".to_string()));
        d.insert(PyValue::Str("axis_minimum".to_string()), coord(0.0, 0.0, 0.0));
        d.insert(PyValue::Str("axis_maximum".to_string()), coord(200.0, 200.0, 180.0));
        d.insert(PyValue::Str("extruder".to_string()), PyValue::Str("kinextr".to_string()));
        Ok(PyValue::Dict(d))
    }
    fn clear_homing_state(&self, _c: &str) -> klippy_api::KResult<()> { Ok(()) }
}
klippy_api::printer_object!(FakeKin; dyn KinematicsApi);

// ---- extra axes ----

struct FakeAxis {
    log: Log,
    name: String,
    gid: String,
    junction: Mutex<Option<f64>>,
    fail: Mutex<Option<String>>,
}

impl ExtraAxisApi for FakeAxis {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_axis_gcode_id(&self) -> Option<String> {
        Some(self.gid.clone())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn check_move(&self, mv: &dyn MoveApi, ea_index: usize) -> KResult<()> {
        push(
            &self.log,
            vec![
                vs("axis.check_move"),
                vs(&self.name),
                vi(ea_index as i64),
                vf(mv.axes_d().0[ea_index]),
                vf(mv.axes_r().0[ea_index]),
            ],
        );
        match &*self.fail.lock() {
            Some(msg) => Err(mv.move_error(msg.clone())),
            None => Ok(()),
        }
    }
    fn calc_junction(&self, _prev: &dyn MoveApi, mv: &dyn MoveApi, ea_index: usize) -> klippy_api::KResult<f64> {
        push(&self.log, vec![vs("axis.calc_junction"), vs(&self.name), vi(ea_index as i64)]);
        Ok(match *self.junction.lock() {
            Some(v) => v,
            None => mv.max_cruise_v2(),
        })
    }
    fn process_move(&self, print_time: f64, mv: &dyn MoveApi, ea_index: usize) {
        push(
            &self.log,
            vec![
                vs("axis.process_move"),
                vs(&self.name),
                vf(print_time),
                vi(ea_index as i64),
                vf(mv.start_v()),
                vf(mv.cruise_v()),
                vf(mv.accel_t()),
                vf(mv.cruise_t()),
                vf(mv.decel_t()),
                vf(mv.axes_d().0[ea_index]),
            ],
        );
    }
}

impl PrinterExtruderApi for FakeAxis {
    fn get_heater(&self) -> KResult<ObjectRef> {
        Err(KlippyError::Command("Extruder not configured".to_string()))
    }
    fn find_past_position(&self, _p: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn stats(&self, _e: f64) -> Option<(bool, String)> {
        None
    }
    fn extruder_stepper(&self) -> Option<klippy_api::toolhead::ExtruderStepper> {
        None
    }
}

// ---- gcode ----

struct FakeGCode {
    log: Log,
    handlers: Mutex<IndexMap<String, Arc<Mutex<GCodeCommandHandler>>>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        push(
            &self.log,
            vec![
                vs("gcode.register_command"),
                vs(cmd),
                Value::Bool(opts.when_not_ready),
                match &opts.desc {
                    Some(d) => vs(d),
                    None => Value::Null,
                },
            ],
        );
        if let Some(handler) = func {
            self.handlers.lock().insert(cmd.to_string(), Arc::new(Mutex::new(handler)));
        }
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _c: &str,
        _k: &str,
        _v: Option<&str>,
        _f: GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _c: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("not used")
    }
    fn create_gcode_command(&self, _c: &str, _l: &str, _p: &IndexMap<String, String>) -> GCmd {
        panic!("not used")
    }
    fn respond_raw(&self, msg: &str) {
        push(&self.log, vec![vs("respond_raw"), vs(msg)]);
    }
    fn respond_info(&self, msg: &str, log: bool) {
        push(&self.log, vec![vs("respond_info"), vs(msg), Value::Bool(log)]);
    }
    fn is_traditional_gcode(&self, _c: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

/// `gcode.GCodeCommand` for the getters the toolhead commands use, with the real error texts.
struct FakeGCommand {
    log: Log,
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGCommand {
    fn error(&self, text: String) -> KlippyError {
        KlippyError::Command(format!("Error on '{}': {}", self.commandline, text))
    }
}

impl GCodeCommandApi for FakeGCommand {
    fn get(&self, name: &str, opts: GCGetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GCGetIntOpts) -> KResult<Option<i64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            };
        };
        let v: i64 = value
            .parse()
            .map_err(|_| self.error(format!("unable to parse {}", value)))?;
        Ok(Some(v))
    }
    fn get_float(&self, name: &str, opts: GCGetFloatOpts) -> KResult<Option<f64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                Some(d) => Ok(d),
                None => Err(self.error(format!("missing {}", name))),
            };
        };
        let v: f64 = value
            .parse()
            .map_err(|_| self.error(format!("unable to parse {}", value)))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.error(format!("{} must have minimum of {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.error(format!("{} must have maximum of {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(self.error(format!("{} must be above {}", name, float_str(m))));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(self.error(format!("{} must be below {}", name, float_str(m))));
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
        self.commandline[self.command.len()..].to_string()
    }
    fn respond_info(&self, msg: &str, log: bool) {
        push(&self.log, vec![vs("respond_info"), vs(msg), Value::Bool(log)]);
    }
    fn respond_raw(&self, msg: &str) {
        push(&self.log, vec![vs("respond_raw"), vs(msg)]);
    }
    fn ack(&self, _m: Option<&str>) -> bool {
        false
    }
}

// ---- config: `configfile.ConfigWrapper` semantics for `get` and `getfloat` ----

struct FakeConfig {
    log: Log,
    printer: Printer,
    section: String,
    options: IndexMap<String, String>,
    tracking: Mutex<IndexMap<(String, String), Value>>,
}

impl FakeConfig {
    fn track(&self, option: &str, value: Value) {
        self.tracking
            .lock()
            .insert((self.section.to_lowercase(), option.to_lowercase()), value);
    }
    fn err(&self, text: String) -> KlippyError {
        KlippyError::config(text)
    }
    fn tracking_value(&self) -> Value {
        Value::List(
            self.tracking
                .lock()
                .iter()
                .map(|((s, o), v)| vt(vec![vt(vec![vs(s), vs(o)]), v.clone()]))
                .collect(),
        )
    }
    fn unused(&self) -> KlippyError {
        KlippyError::config("not used")
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        Arc::clone(&self.printer)
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.options.get(option) {
            Some(v) => {
                self.track(option, vs(v));
                Ok(Some(v.clone()))
            }
            None => match opts.default {
                None => Err(self.err(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => {
                    if let Some(s) = &d {
                        self.track(option, vs(s));
                    }
                    Ok(d)
                }
            },
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Err(self.unused())
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.options.get(option) else {
            return match opts.default {
                None => Err(self.err(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => {
                    if let Some(v) = d {
                        self.track(option, vf(v));
                    }
                    Ok(d)
                }
            };
        };
        let v: f64 = raw.parse().map_err(|_| {
            self.err(format!(
                "Unable to parse option '{}' in section '{}'",
                option, self.section
            ))
        })?;
        self.track(option, vf(v));
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.err(format!(
                    "Option '{}' in section '{}' must have minimum of {}",
                    option,
                    self.section,
                    float_str(m)
                )));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.err(format!(
                    "Option '{}' in section '{}' must have maximum of {}",
                    option,
                    self.section,
                    float_str(m)
                )));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(self.err(format!(
                    "Option '{}' in section '{}' must be above {}",
                    option,
                    self.section,
                    float_str(m)
                )));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(self.err(format!(
                    "Option '{}' in section '{}' must be below {}",
                    option,
                    self.section,
                    float_str(m)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _o: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Err(self.unused())
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(self.unused())
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(self.unused())
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(self.unused())
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(self.unused())
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(self.unused())
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            log: Arc::clone(&self.log),
            printer: Arc::clone(&self.printer),
            section: section.to_string(),
            options: IndexMap::new(),
            tracking: Mutex::new(IndexMap::new()),
        }) as Config
    }
    fn has_section(&self, section: &str) -> bool {
        push(&self.log, vec![vs("config.has_section"), vs(section)]);
        section == self.section
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---- printer ----

struct FakePrinter {
    log: Log,
    reactor: Reactor,
    loadable: Vec<String>,
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<IndexMap<String, Vec<Arc<Mutex<EventCallback>>>>>,
}

impl FakePrinter {
    /// Seed an object without logging (the Python fakes fill `objects` directly).
    fn seed(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
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
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        push(&self.log, vec![vs("add_object"), vs(name)]);
        let mut objs = self.objects.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{}' already created", name)));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        push(&self.log, vec![vs("lookup_object"), vs(name)]);
        match self.objects.lock().get(name) {
            Some(o) => Ok(Some(Arc::clone(o))),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::config(format!("Unknown config object '{}'", name)))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.lock().iter().map(|(k, v)| (k.clone(), Arc::clone(v))).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        push(&self.log, vec![vs("load_object"), vs(section)]);
        if let Some(o) = self.objects.lock().get(section) {
            return Ok(Some(Arc::clone(o)));
        }
        if self.loadable.iter().any(|s| s == section) {
            let stub: ObjectRef = Arc::new(Stub);
            self.objects.lock().insert(section.to_string(), Arc::clone(&stub));
            return Ok(Some(stub));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unable to load module '{}'", section)))
            }
        }
    }
    fn run(&self) -> Option<String> {
        Some("exit".to_string())
    }
    fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool) {
        push(
            &self.log,
            vec![
                vs("set_rollover_info"),
                vs(name),
                match info {
                    Some(s) => vs(s),
                    None => Value::Null,
                },
                Value::Bool(log),
            ],
        );
    }
    fn invoke_shutdown(&self, msg: &str, details: PyValue) {
        push(&self.log, vec![vs("invoke_shutdown"), vs(msg), pyvalue_to_value(&details)]);
    }
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        self.invoke_shutdown(msg, details);
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        push(&self.log, vec![vs("register_event_handler"), vs(event)]);
        self.handlers
            .lock()
            .entry(event.to_string())
            .or_default()
            .push(Arc::new(Mutex::new(callback)));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut shown: Vec<Value> = params
            .iter()
            .map(|a| match a {
                EventArg::Value(v) => pyvalue_to_value(v),
                _ => Value::Null,
            })
            .collect();
        if event == "toolhead:sync_print_time" && !shown.is_empty() {
            shown[0] = Value::Null;
        }
        push(&self.log, vec![vs("send_event"), vs(event), vt(shown)]);
        let cbs: Vec<Arc<Mutex<EventCallback>>> =
            self.handlers.lock().get(event).cloned().unwrap_or_default();
        let mut results = Vec::new();
        for cb in cbs {
            let mut guard = cb.lock();
            let f: &mut EventCallback = &mut guard;
            results.push(f(params)?);
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

// ---------------------------------------------------------------------------
// Worlds: the same objects on both sides.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Setup {
    options: Vec<(&'static str, &'static str)>,
    fileoutput: bool,
    est: f64,
    /// `mcu`, `motion_queuing`, `gcode`, `pins`.
    has: [bool; 4],
    loadable: Vec<&'static str>,
    axes: Vec<(&'static str, &'static str)>,
}

impl Setup {
    fn base() -> Setup {
        Setup {
            options: vec![
                ("max_velocity", "250"),
                ("max_accel", "3000"),
                ("kinematics", "fakekin"),
            ],
            fileoutput: true,
            est: 0.0,
            has: [true; 4],
            loadable: Vec::new(),
            axes: vec![("a", "A"), ("b", "B"), ("e1", "E")],
        }
    }
    fn with_options(extra: &[(&'static str, &'static str)]) -> Setup {
        let mut s = Setup::base();
        for (k, v) in extra {
            if let Some(slot) = s.options.iter_mut().find(|(ok, _)| ok == k) {
                slot.1 = v;
            } else {
                s.options.push((k, v));
            }
        }
        s
    }
    fn ini(&self) -> String {
        let mut s = String::from("[printer]\n");
        for (k, v) in &self.options {
            s.push_str(&format!("{}: {}\n", k, v));
        }
        s
    }
}

fn pyb(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

/// Python float literal (finite values only).
fn pyf(x: f64) -> String {
    format!("{:?}", x)
}
fn pyl(v: &[f64]) -> String {
    format!("[{}]", v.iter().map(|x| pyf(*x)).collect::<Vec<_>>().join(", "))
}
fn pyo(v: Option<f64>) -> String {
    match v {
        Some(x) => pyf(x),
        None => "None".to_string(),
    }
}
fn pyol(v: &[Option<f64>]) -> String {
    format!("[{}]", v.iter().map(|x| pyo(*x)).collect::<Vec<_>>().join(", "))
}
fn pys(s: &str) -> String {
    format!("'{}'", s)
}

fn py_world(s: &Setup) -> String {
    let ini = s.ini().replace('\n', "\\n");
    let loadable = s.loadable.iter().map(|m| pys(m)).collect::<Vec<_>>().join(", ");
    let axes = s
        .axes
        .iter()
        .map(|(n, g)| format!("({}, {})", pys(n), pys(g)))
        .collect::<Vec<_>>()
        .join(", ");
    format!(
        "make_world(\"{}\", {}, {}, ({}, {}, {}, {}), [{}], [{}])",
        ini,
        pyb(s.fileoutput),
        pyf(s.est),
        pyb(s.has[0]),
        pyb(s.has[1]),
        pyb(s.has[2]),
        pyb(s.has[3]),
        loadable,
        axes
    )
}

struct World {
    log: Log,
    printer: Arc<FakePrinter>,
    mcu: Arc<FakeMcu>,
    mq: Arc<FakeMq>,
    kin: Arc<FakeKin>,
    gcode: Arc<FakeGCode>,
    config: Arc<FakeConfig>,
    axes: IndexMap<String, Arc<FakeAxis>>,
    th: Option<Arc<ToolHead>>,
    helper: Option<ToolHeadCommandHelper>,
}

impl World {
    fn new(s: &Setup) -> World {
        install_sink();
        drain_lines();
        register_loaders();
        let log: Log = Arc::new(Mutex::new(Vec::new()));
        let printer = Arc::new(FakePrinter {
            log: Arc::clone(&log),
            reactor: Reactor::new(false),
            loadable: s.loadable.iter().map(|m| m.to_string()).collect(),
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(IndexMap::new()),
        });
        let printer_handle: Printer = Arc::clone(&printer) as Printer;
        let mcu = Arc::new(FakeMcu {
            log: Arc::clone(&log),
            printer: Arc::clone(&printer_handle),
            fileoutput: s.fileoutput,
            est: Mutex::new(s.est),
        });
        let mq = Arc::new(FakeMq {
            log: Arc::clone(&log),
            trapq: TrapqHandle::alloc(),
            flush_cbs: Mutex::new(Vec::new()),
            step_restart: Mutex::new(0.0),
        });
        let kin = Arc::new(FakeKin {
            log: Arc::clone(&log),
            limits: Mutex::new(None),
            fail: Mutex::new(false),
        });
        let gcode = Arc::new(FakeGCode {
            log: Arc::clone(&log),
            handlers: Mutex::new(IndexMap::new()),
        });
        if s.has[0] {
            printer.seed("mcu", Arc::clone(&mcu) as ObjectRef);
        }
        if s.has[1] {
            printer.seed("motion_queuing", Arc::clone(&mq) as ObjectRef);
        }
        if s.has[2] {
            printer.seed("gcode", Arc::clone(&gcode) as ObjectRef);
        }
        if s.has[3] {
            printer.seed("pins", Arc::new(FakePins) as ObjectRef);
        }
        printer.seed("_kin", Arc::clone(&kin) as ObjectRef);
        let options: IndexMap<String, String> =
            s.options.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect();
        let config = Arc::new(FakeConfig {
            log: Arc::clone(&log),
            printer: Arc::clone(&printer_handle),
            section: "printer".to_string(),
            options,
            tracking: Mutex::new(IndexMap::new()),
        });
        let axes = s
            .axes
            .iter()
            .map(|(n, g)| {
                (
                    n.to_string(),
                    Arc::new(FakeAxis {
                        log: Arc::clone(&log),
                        name: n.to_string(),
                        gid: g.to_string(),
                        junction: Mutex::new(None),
                        fail: Mutex::new(None),
                    }),
                )
            })
            .collect();
        World { log, printer, mcu, mq, kin, gcode, config, axes, th: None, helper: None }
    }

    fn cfg(&self) -> Config {
        Arc::clone(&self.config) as Config
    }

    fn drain_log(&self) -> Value {
        Value::List(std::mem::take(&mut *self.log.lock()))
    }

    fn th(&self) -> &Arc<ToolHead> {
        self.th.as_ref().expect("toolhead built")
    }

    fn observe(&self) -> Value {
        let th = match &self.th {
            Some(t) => t,
            None => return Value::Null,
        };
        let status = pyvalue_to_value(&ToolHeadApi::get_status(&**th, 0.0).unwrap());
        let (print_time, est, empty) = th.check_busy(0.0);
        let (active, msg) = ToolHeadApi::stats(&**th, 0.0);
        let pos = th.get_position();
        let (mv, ma) = th.get_max_velocity();
        let tq = th.get_trapq();
        let tsp = {
            let handle = trapq_of(&tq).expect("real trapq");
            let guard = handle.shared().lock();
            match guard.history.front() {
                Some(m) => vt(vec![
                    vf(m.print_time),
                    vf(m.start_pos.x),
                    vf(m.start_pos.y),
                    vf(m.start_pos.z),
                ]),
                None => Value::Null,
            }
        };
        ok(vt(vec![
            status,
            vt(vec![vf(print_time), vf(est), Value::Bool(empty)]),
            vt(vec![Value::Bool(active), vs(&msg)]),
            vfl(&pos),
            vt(vec![vf(mv), vf(ma)]),
            tsp,
        ]))
    }

    /// `ToolHead(config)` (and, with `with_helper`, the `toolhead` object and its command
    /// helper), like `_construct` on the Python side.
    fn construct(&mut self, with_helper: bool) -> Value {
        let res = self.construct_inner(with_helper);
        let obs = self.observe();
        vt(vec![
            unit(res),
            obs,
            self.drain_log(),
            self.config.tracking_value(),
            drain_lines(),
        ])
    }

    fn construct_inner(&mut self, with_helper: bool) -> KResult<()> {
        self.th = None;
        self.helper = None;
        let cfg = self.cfg();
        let th = ToolHead::new(&cfg)?;
        self.th = Some(Arc::clone(&th));
        if with_helper {
            self.printer.add_object("toolhead", th as ObjectRef)?;
            self.helper = Some(ToolHeadCommandHelper::new(&cfg)?);
        }
        Ok(())
    }

    fn step(&mut self, op: &Op) -> Value {
        let res = op.run(self);
        let obs = self.observe();
        vt(vec![res, obs, self.drain_log(), drain_lines()])
    }
}

fn ok(v: Value) -> Value {
    vt(vec![vs("ok"), v])
}
fn exc(e: &KlippyError) -> Value {
    vt(vec![vs("exc"), vs(e.class_name()), vs(e.message())])
}
fn unit(r: KResult<()>) -> Value {
    match r {
        Ok(()) => ok(Value::Null),
        Err(e) => exc(&e),
    }
}

// ---------------------------------------------------------------------------
// Operations (rendered as Python and run on the port).
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Op {
    Move(Vec<f64>, f64),
    ManualMove(Vec<Option<f64>>, f64),
    Dwell(f64),
    WaitMoves,
    FlushStep,
    LastMoveTime,
    SetPosition(Vec<f64>, Option<&'static str>),
    LimitNext(f64),
    Drip(Vec<f64>, f64),
    FlushCb(f64, f64),
    Shutdown,
    SetMaxVel(Option<f64>, Option<f64>, Option<f64>, Option<f64>),
    AddAxis(&'static str, f64),
    RemoveAxis(&'static str),
    SetExtruder(&'static str, f64),
    ExtruderName,
    ExtraAxes,
    LookaheadCb(i64),
    LookaheadError,
    KinLimits(Option<(f64, f64)>),
    KinFail(bool),
    AxisFail(&'static str, Option<&'static str>),
    AxisJunction(&'static str, Option<f64>),
    SetEst(f64),
    StepRestart(f64),
    Cmd(&'static str, &'static str, Vec<(&'static str, &'static str)>),
}

impl Op {
    fn py(&self) -> String {
        match self {
            Op::Move(p, s) => format!("th.move({}, {})", pyl(p), pyf(*s)),
            Op::ManualMove(c, s) => format!("th.manual_move({}, {})", pyol(c), pyf(*s)),
            Op::Dwell(d) => format!("th.dwell({})", pyf(*d)),
            Op::WaitMoves => "th.wait_moves()".to_string(),
            Op::FlushStep => "th.flush_step_generation()".to_string(),
            Op::LastMoveTime => "th.get_last_move_time()".to_string(),
            Op::SetPosition(p, Some(a)) => format!("th.set_position({}, {})", pyl(p), pys(a)),
            Op::SetPosition(p, None) => format!("th.set_position({})", pyl(p)),
            Op::LimitNext(s) => format!("th.limit_next_junction_speed({})", pyf(*s)),
            Op::Drip(p, s) => format!("th.drip_move({}, {}, _Comp())", pyl(p), pyf(*s)),
            Op::FlushCb(a, b) => format!("mq.flush_cbs[0]({}, {})", pyf(*a), pyf(*b)),
            Op::Shutdown => "_none(printer.send_event('klippy:shutdown'))".to_string(),
            Op::SetMaxVel(a, b, c, d) => format!(
                "th.set_max_velocities({}, {}, {}, {})",
                pyo(*a),
                pyo(*b),
                pyo(*c),
                pyo(*d)
            ),
            Op::AddAxis(n, p) => format!("th.add_extra_axis(AX[{}], {})", pys(n), pyf(*p)),
            Op::RemoveAxis(n) => format!("th.remove_extra_axis(AX[{}])", pys(n)),
            Op::SetExtruder(n, p) => format!("th.set_extruder(AX[{}], {})", pys(n), pyf(*p)),
            Op::ExtruderName => "th.get_extruder().get_name()".to_string(),
            Op::ExtraAxes => {
                "[ea.get_name() if ea is not None else None for ea in th.get_extra_axes()]"
                    .to_string()
            }
            Op::LookaheadCb(tag) => format!(
                "th.register_lookahead_callback(lambda t: LOG.append(('lookahead_cb', {}, t)))",
                tag
            ),
            Op::LookaheadError => "th.register_lookahead_callback(lambda t: (_ for _ in ()).throw(ValueError('lookahead failed')))".to_string(),
            Op::KinLimits(Some((s, a))) => {
                format!("_set(kin, 'limits', ({}, {}))", pyf(*s), pyf(*a))
            }
            Op::KinLimits(None) => "_set(kin, 'limits', None)".to_string(),
            Op::KinFail(b) => format!("_set(kin, 'fail', {})", pyb(*b)),
            Op::AxisFail(n, Some(m)) => format!("_set(AX[{}], 'fail', {})", pys(n), pys(m)),
            Op::AxisFail(n, None) => format!("_set(AX[{}], 'fail', None)", pys(n)),
            Op::AxisJunction(n, v) => format!("_set(AX[{}], 'junction', {})", pys(n), pyo(*v)),
            Op::SetEst(v) => format!("_set(mcu, 'est', {})", pyf(*v)),
            Op::StepRestart(v) => format!("_set(mq, 'step_restart', {})", pyf(*v)),
            Op::Cmd(cmd, line, params) => {
                let dict = params
                    .iter()
                    .map(|(k, v)| format!("{}: {}", pys(k), pys(v)))
                    .collect::<Vec<_>>()
                    .join(", ");
                format!("_cmd({}, {}, {{{}}})", pys(cmd), pys(line), dict)
            }
        }
    }

    fn run(&self, w: &mut World) -> Value {
        match self {
            Op::Move(p, s) => unit(w.th().r#move(p.clone(), *s)),
            Op::ManualMove(c, s) => unit(w.th().manual_move(c.clone(), *s)),
            Op::Dwell(d) => unit(w.th().dwell(*d)),
            Op::WaitMoves => unit(w.th().wait_moves()),
            Op::FlushStep => unit(w.th().flush_step_generation()),
            Op::LastMoveTime => match w.th().get_last_move_time() {
                Ok(time) => ok(vf(time)),
                Err(error) => exc(&error),
            },
            Op::SetPosition(p, a) => unit(w.th().set_position(p.clone(), a.unwrap_or(""))),
            Op::LimitNext(s) => {
                w.th().limit_next_junction_speed(*s);
                ok(Value::Null)
            }
            Op::Drip(p, s) => {
                let completion = w.printer.reactor.completion::<bool>();
                unit(w.th().drip_move(p.clone(), *s, &completion))
            }
            Op::FlushCb(a, b) => {
                let cb = Arc::clone(&w.mq.flush_cbs.lock()[0].0);
                unit(cb(*a, *b))
            }
            Op::Shutdown => {
                let r = w.printer.send_event("klippy:shutdown", &[]);
                unit(r.map(|_| ()))
            }
            Op::SetMaxVel(a, b, c, d) => {
                let (mv, ma, scv, mcr) = w.th().set_max_velocities(*a, *b, *c, *d);
                ok(vt(vec![vf(mv), vf(ma), vf(scv), vf(mcr)]))
            }
            Op::AddAxis(n, p) => {
                let ea: ExtraAxis = Arc::clone(&w.axes[*n]) as ExtraAxis;
                unit(w.th().add_extra_axis(ea, *p))
            }
            Op::RemoveAxis(n) => {
                let ea: ExtraAxis = Arc::clone(&w.axes[*n]) as ExtraAxis;
                unit(w.th().remove_extra_axis(&ea))
            }
            Op::SetExtruder(n, p) => {
                let ex: PrinterExtruder = Arc::clone(&w.axes[*n]) as PrinterExtruder;
                w.th().set_extruder(ex, *p);
                ok(Value::Null)
            }
            Op::ExtruderName => ok(vs(&w.th().get_extruder().get_name())),
            Op::ExtraAxes => ok(Value::List(
                w.th()
                    .get_extra_axes()
                    .iter()
                    .map(|ea| match ea {
                        Some(ea) => vs(&ea.get_name()),
                        None => Value::Null,
                    })
                    .collect(),
            )),
            Op::LookaheadCb(tag) => {
                let log = Arc::clone(&w.log);
                let tag = *tag;
                unit(w.th().register_lookahead_callback(Box::new(move |t| {
                    push(&log, vec![vs("lookahead_cb"), vi(tag), vf(t)]);
                    Ok(())
                })))
            }
            Op::LookaheadError => unit(w.th().register_lookahead_callback(Box::new(|_| {
                Err(KlippyError::Internal {
                    class_name: "ValueError".to_string(),
                    message: "lookahead failed".to_string(),
                })
            }))),
            Op::KinLimits(l) => {
                *w.kin.limits.lock() = *l;
                ok(Value::Null)
            }
            Op::KinFail(b) => {
                *w.kin.fail.lock() = *b;
                ok(Value::Null)
            }
            Op::AxisFail(n, m) => {
                *w.axes[*n].fail.lock() = m.map(|s| s.to_string());
                ok(Value::Null)
            }
            Op::AxisJunction(n, v) => {
                *w.axes[*n].junction.lock() = *v;
                ok(Value::Null)
            }
            Op::SetEst(v) => {
                *w.mcu.est.lock() = *v;
                ok(Value::Null)
            }
            Op::StepRestart(v) => {
                *w.mq.step_restart.lock() = *v;
                ok(Value::Null)
            }
            Op::Cmd(cmd, line, params) => {
                let gcmd: GCmd = Arc::new(FakeGCommand {
                    log: Arc::clone(&w.log),
                    command: cmd.to_string(),
                    commandline: line.to_string(),
                    params: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
                });
                let handler = Arc::clone(w.gcode.handlers.lock().get(*cmd).expect("registered"));
                let mut guard = handler.lock();
                let f: &mut GCodeCommandHandler = &mut guard;
                unit(f(&gcmd))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Kinematics loaders (the `kinematics.<name>` modules of the Python side).
// ---------------------------------------------------------------------------

fn load_fakekin(_th: &klippy_api::toolhead::ToolHead, cfg: &Config) -> KResult<Kinematics> {
    let printer = cfg.get_printer();
    Ok(lookup_interface::<dyn KinematicsApi>(&*printer, "_kin", LookupDefault::Raise)?
        .expect("Raise never yields None"))
}
fn load_cfgerr(_th: &klippy_api::toolhead::ToolHead, _cfg: &Config) -> KResult<Kinematics> {
    Err(KlippyError::config("kin config boom"))
}
fn load_pinserr(_th: &klippy_api::toolhead::ToolHead, cfg: &Config) -> KResult<Kinematics> {
    cfg.get_printer().lookup_object("pins", LookupDefault::Raise)?;
    Err(KlippyError::Pins("kin pins boom".to_string()))
}
fn load_generr(_th: &klippy_api::toolhead::ToolHead, _cfg: &Config) -> KResult<Kinematics> {
    Err(KlippyError::Internal {
        class_name: "ValueError".to_string(),
        message: "kin generic boom".to_string(),
    })
}
fn load_cmderr(_th: &klippy_api::toolhead::ToolHead, _cfg: &Config) -> KResult<Kinematics> {
    Err(KlippyError::command("kin command boom"))
}

fn register_loaders() {
    register_kinematics_loader("fakekin", load_fakekin);
    register_kinematics_loader("cfgerr", load_cfgerr);
    register_kinematics_loader("pinserr", load_pinserr);
    register_kinematics_loader("generr", load_generr);
    register_kinematics_loader("cmderr", load_cmderr);
}

/// `kinematics.extruder.add_printer_objects(config)` as far as the fake config can tell: it
/// asks whether an `extruder` section exists.
fn extruder_objects(config: &Config) -> KResult<()> {
    config.has_section("extruder");
    Ok(())
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> pyoracle::Oracle {
    let mut py = pyoracle::Oracle::start().expect(
        "the oracle is required: set KLIPPY_PYTHON to the CPython 3.8.2 of the oracle image",
    );
    if let Err(e) = py.exec(SETUP) {
        panic!("toolhead Python fakes failed: {}: {}", e.class_name, e.message);
    }
    py
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "toolhead".to_string(),
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

/// Build the world on both sides, construct the toolhead and compare the construction.
fn build_pair(py: &mut pyoracle::Oracle, label: &str, setup: &Setup, with_helper: bool) -> World {
    py.exec(&py_world(setup)).expect("py world");
    let expected = py.eval(&format!("_construct({})", pyb(with_helper)));
    let mut world = World::new(setup);
    let actual = world.construct(with_helper);
    check(&format!("{label}: construction"), expected, actual);
    world
}

fn run_ops(py: &mut pyoracle::Oracle, world: &mut World, label: &str, ops: &[Op]) {
    for (i, op) in ops.iter().enumerate() {
        let expected = py.eval(&format!("_step(lambda: {})", op.py()));
        let actual = world.step(op);
        check(&format!("{label}: op {} {}", i, op.py()), expected, actual);
    }
}

/// Run a scenario on a fresh pair of worlds.
fn scenario(label: &str, setup: &Setup, with_helper: bool, ops: &[Op]) {
    let mut py = oracle();
    let mut world = build_pair(&mut py, label, setup, with_helper);
    run_ops(&mut py, &mut world, label, ops);
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 33) as f64) / ((1u64 << 31) as f64)
    }
    fn pick(&mut self, n: usize) -> usize {
        let r = self.next();
        let i = (r * n as f64).floor();
        (i as usize).min(n - 1)
    }
}

/// A deterministic polyline of mixed moves: XY and Z moves, extrude-only moves,
/// reversals and tiny steps, at several speeds.
fn polyline(seed: u64, n: usize, extra: usize) -> Vec<(Vec<f64>, f64)> {
    let mut rng = Lcg(seed);
    let mut pos = vec![0.0f64; 4 + extra];
    let speeds = [5.0, 20.0, 50.0, 100.0, 200.0, 400.0, 1000.0];
    let mut out = Vec::new();
    for _ in 0..n {
        let kind = rng.pick(10);
        let speed = speeds[rng.pick(speeds.len())];
        let step = |r: f64| ((r * 80.0 - 40.0) * 2.0).round() / 2.0;
        match kind {
            0 => {
                // Extrude only.
                pos[3] += ((rng.next() * 10.0) * 4.0).round() / 4.0 + 0.25;
            }
            1 => {
                pos[2] += step(rng.next()) / 8.0;
            }
            2 => {
                // Tiny XY step.
                pos[0] += 0.25;
                pos[3] += 0.01;
            }
            3 => {
                // Back along the previous direction.
                pos[0] -= step(rng.next());
            }
            _ => {
                pos[0] += step(rng.next());
                pos[1] += step(rng.next());
                pos[3] += (rng.next() * 3.0 * 100.0).round() / 100.0;
            }
        }
        for e in 0..extra {
            if rng.pick(3) == 0 {
                pos[4 + e] += ((rng.next() * 6.0) * 4.0).round() / 4.0;
            }
        }
        out.push((pos.clone(), speed));
    }
    out
}

fn move_ops(seed: u64, n: usize, extra: usize) -> Vec<Op> {
    polyline(seed, n, extra).into_iter().map(|(p, s)| Op::Move(p, s)).collect()
}

const JD: f64 = 5.0 * 5.0 * (std::f64::consts::SQRT_2 - 1.0) / 3000.0;
const MCR_ACCEL: f64 = 3000.0 * (1.0 - 0.5);

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_shape() {
    let factory = module_factory();
    assert!(factory.add_printer_objects.is_some());
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
}

/// A Python toolhead with fixed limits for the pure `Move` and `LookAheadQueue` tests.
fn limits_world(py: &mut pyoracle::Oracle) -> Printer {
    let setup = Setup::base();
    py.exec(&py_world(&setup)).expect("py world");
    py.exec("_construct(False)").expect("py construct");
    py.exec(&format!(
        "th.max_velocity = 250.0\nth.max_accel = 3000.0\nth.junction_deviation = {}\nth.mcr_pseudo_accel = {}",
        pyf(JD),
        pyf(MCR_ACCEL)
    ))
    .expect("py limits");
    let world = World::new(&setup);
    Arc::clone(&world.printer) as Printer
}

fn move_fields(m: &Move) -> Value {
    vt(vec![
        vfs(&m.start_pos().0),
        vfs(&m.end_pos().0),
        vfl(&m.axes_d().0),
        vfl(&m.axes_r().0),
        vf(m.move_d()),
        Value::Bool(m.is_kinematic_move()),
        vf(m.accel()),
        vf(m.max_cruise_v2()),
    ])
}

fn move_timing(m: &Move) -> Value {
    vt(vec![
        vf(m.start_v()),
        vf(m.cruise_v()),
        vf(m.accel_t()),
        vf(m.cruise_t()),
        vf(m.decel_t()),
        vf(m.accel()),
        vfs(&m.end_pos().0),
    ])
}

fn new_move(printer: &Printer, start: &[f64], end: &[f64], speed: f64) -> KResult<Move> {
    Move::new(
        start.to_vec(),
        end.to_vec(),
        speed,
        250.0,
        3000.0,
        JD,
        MCR_ACCEL,
        Arc::clone(printer),
    )
}

#[test]
fn move_construction_matches_python() {
    let mut py = oracle();
    let printer = limits_world(&mut py);
    type Case = (Vec<f64>, Vec<f64>, f64, Option<(f64, f64)>, Option<f64>, (f64, f64, f64));
    let cases: Vec<Case> = vec![
        (vec![0.0; 4], vec![10.0, 0.0, 0.0, 5.0], 50.0, None, None, (400.0, 2500.0, 0.0)),
        (vec![1.0, 2.0, 3.0, 4.0], vec![1.0, 2.0, 3.0, 6.5], 50.0, None, None, (0.0, 4.0, 0.0)),
        (vec![1.0; 4], vec![1.0; 4], 50.0, None, None, (0.0, 4.0, 0.0)),
        (vec![0.0; 4], vec![3.0, 4.0, 0.0, 0.0], 1000.0, Some((100.0, 1000.0)), Some(30.0), (0.0, 10000.0, 0.0)),
        (vec![0.0; 4], vec![3.0, 4.0, 0.0, 0.0], 80.0, Some((90.0, 4000.0)), Some(1e3), (100.0, 6400.0, 25.0)),
        (vec![0.0; 4], vec![0.0, 0.0, 5.0, 0.0], 0.0, None, None, (0.0, 4.0, 0.0)),
        (vec![0.0; 4], vec![0.0, 0.0, 0.0, -2.0], 0.0, None, None, (0.0, 4.0, 0.0)),
        (vec![0.0; 4], vec![1e-10, 0.0, 0.0, 0.0], 10.0, None, None, (0.0, 4.0, 0.0)),
        (vec![0.0; 4], vec![0.0, 0.0, 0.0, 1e-9], 10.0, None, None, (0.0, 4.0, 0.0)),
        (vec![0.0; 6], vec![4.0, 0.0, 0.0, 1.0, 3.0, -2.0], 20.0, None, None, (0.0, 400.0, 0.0)),
        (vec![-1.5, 2.25, 0.125, 0.0], vec![-7.5, 12.25, 0.375, 0.5], 33.3, Some((12.0, 500.0)), None, (4.0, 1000.0, 4.0)),
    ];
    for (i, (start, end, speed, limit, next_limit, junction)) in cases.iter().enumerate() {
        let limit_py = match limit {
            Some((s, a)) => format!("({}, {})", pyf(*s), pyf(*a)),
            None => "None".to_string(),
        };
        let expected = py.eval(&format!(
            "_try(lambda: _move_case({}, {}, {}, {}, {}, ({}, {}, {})))",
            pyl(start),
            pyl(end),
            pyf(*speed),
            limit_py,
            pyo(*next_limit),
            pyf(junction.0),
            pyf(junction.1),
            pyf(junction.2)
        ));
        let actual = match new_move(&printer, start, end, *speed) {
            Err(e) => exc(&e),
            Ok(m) => {
                let before = move_fields(&m);
                if let Some((s, a)) = limit {
                    m.limit_speed(*s, *a).expect("valid speed limit");
                }
                if let Some(s) = next_limit {
                    m.limit_next_junction_speed(*s);
                }
                let after = move_fields(&m);
                match m.set_junction(junction.0, junction.1, junction.2) {
                    Err(e) => exc(&e),
                    Ok(()) => ok(vt(vec![before, after, move_timing(&m)])),
                }
            }
        };
        check(&format!("move case {i}"), expected, actual);
    }
}

#[test]
fn move_error_text_matches_python() {
    let mut py = oracle();
    let printer = limits_world(&mut py);
    let cases: Vec<(Vec<f64>, Vec<f64>, Option<&str>)> = vec![
        (vec![0.0; 4], vec![1.0, 2.0, 3.0, 4.0], None),
        (vec![0.0; 4], vec![1.0, 2.0, 3.0, 4.0], Some("Custom fail")),
        (vec![0.0; 4], vec![-0.0004, 1234567.8915, -2.5, 1e-7], Some("Must home axis first")),
        (vec![0.0; 4], vec![0.0, 0.0, 0.0, 0.0005], None),
        (vec![0.0; 4], vec![12.3456, 0.0, 0.0, 0.0], Some("")),
    ];
    for (i, (start, end, msg)) in cases.iter().enumerate() {
        let msg_py = match msg {
            Some(m) => pys(m),
            None => "None".to_string(),
        };
        let expected = py.eval(&format!(
            "_try(lambda: _move_error({}, {}, {}))",
            pyl(start),
            pyl(end),
            msg_py
        ));
        let m = new_move(&printer, start, end, 10.0).expect("move");
        let e = match msg {
            Some(m_) => MoveApi::move_error(&m, m_.to_string()),
            None => m.move_error_default(),
        };
        check(
            &format!("move_error {i}"),
            expected,
            ok(vt(vec![vs(e.class_name()), vs(e.message())])),
        );
    }
}

#[test]
fn lookahead_flush_matches_python() {
    let mut py = oracle();
    let printer = limits_world(&mut py);
    for (case, seed, n) in [(0, 7u64, 36usize), (1, 99, 36), (2, 12345, 48)] {
        let steps = polyline(seed, n, 0);
        // `th.move` skips zero-length moves before they reach the queue.
        let mut moves: Vec<(Vec<f64>, f64)> = Vec::new();
        let mut last = vec![0.0f64; 4];
        for (p, s) in steps {
            if p != last {
                last = p.clone();
                moves.push((p, s));
            }
        }
        let count = moves.len();
        let mut limits: Vec<Option<(f64, f64)>> = vec![None; count];
        let mut next_limits: Vec<Option<f64>> = vec![None; count];
        let mut plan: Vec<(usize, bool)> = vec![(count / 3, true), (count / 2, false)];
        if case == 1 {
            limits[2] = Some((60.0, 1500.0));
            limits[count / 4] = Some((300.0, 800.0));
            next_limits[3] = Some(15.0);
            next_limits[count - 3] = Some(40.0);
            plan.push((count - 5, true));
        }
        plan.sort();
        plan.dedup_by_key(|(i, _)| *i);
        let moves_py = moves
            .iter()
            .map(|(p, s)| format!("({}, {})", pyl(p), pyf(*s)))
            .collect::<Vec<_>>()
            .join(", ");
        let limits_py = limits
            .iter()
            .map(|l| match l {
                Some((s, a)) => format!("({}, {})", pyf(*s), pyf(*a)),
                None => "None".to_string(),
            })
            .collect::<Vec<_>>()
            .join(", ");
        let next_py = next_limits.iter().map(|l| pyo(*l)).collect::<Vec<_>>().join(", ");
        let plan_py = plan
            .iter()
            .map(|(i, lazy)| format!("{}: {}", i, pyb(*lazy)))
            .collect::<Vec<_>>()
            .join(", ");
        let expected = py.eval(&format!(
            "_try(lambda: _la_scenario([0., 0., 0., 0.], [{}], [{}], [{}], {{{}}}))",
            moves_py, limits_py, next_py, plan_py
        ));

        let extra_axes: Vec<ExtraAxis> =
            vec![Arc::new(DummyExtruder::new(Arc::clone(&printer))) as ExtraAxis];
        let mut la = LookAheadQueue::new();
        la.set_flush_time(2.0);
        let mut out: Vec<Value> = Vec::new();
        let mut pos = vec![0.0f64; 4];
        let mut failure: Option<Value> = None;
        'run: {
            for (i, (end, speed)) in moves.iter().enumerate() {
                let m = match new_move(&printer, &pos, end, *speed) {
                    Ok(m) => Arc::new(m),
                    Err(e) => {
                        failure = Some(exc(&e));
                        break 'run;
                    }
                };
                if let Some((s, a)) = limits[i] {
                    m.limit_speed(s, a).expect("valid speed limit");
                }
                if let Some(s) = next_limits[i] {
                    m.limit_next_junction_speed(s);
                }
                pos = m.end_pos().0;
                let r = la.add_move(Arc::clone(&m), &extra_axes).expect("add_move");
                out.push(vt(vec![vs("add"), vi(i as i64), Value::Bool(r), Value::Bool(la.is_empty())]));
                if let Some((_, lazy)) = plan.iter().find(|(j, _)| *j == i) {
                    match la.flush(*lazy) {
                        Err(e) => {
                            failure = Some(exc(&e));
                            break 'run;
                        }
                        Ok(got) => out.push(vt(vec![
                            vs("flush"),
                            vi(i as i64),
                            Value::Bool(*lazy),
                            Value::List(got.iter().map(|m| move_timing(m)).collect()),
                            Value::Bool(la.is_empty()),
                            Value::Bool(la.get_last().is_none()),
                        ])),
                    }
                }
            }
            match la.flush(false) {
                Err(e) => failure = Some(exc(&e)),
                Ok(got) => out.push(vt(vec![
                    vs("final"),
                    Value::List(got.iter().map(|m| move_timing(m)).collect()),
                    Value::Bool(la.is_empty()),
                    Value::Bool(la.get_last().is_none()),
                ])),
            }
        }
        let actual = match failure {
            Some(f) => f,
            None => ok(Value::List(out)),
        };
        check(&format!("lookahead case {case}"), expected, actual);
    }
}

#[test]
fn toolhead_init_matches_python() {
    let cases: Vec<(&str, Setup)> = vec![
        ("defaults", Setup::base()),
        (
            "all options",
            Setup::with_options(&[
                ("minimum_cruise_ratio", "0.25"),
                ("square_corner_velocity", "8.5"),
                ("max_velocity", "300.5"),
                ("max_accel", "1234.5"),
            ]),
        ),
        ("zero scv", Setup::with_options(&[("square_corner_velocity", "0")])),
        ("zero mcr", Setup::with_options(&[("minimum_cruise_ratio", "0")])),
        ("non-file output", {
            let mut s = Setup::base();
            s.fileoutput = false;
            s.est = 12.5;
            s
        }),
        ("missing max_velocity", {
            let mut s = Setup::base();
            s.options.retain(|(k, _)| *k != "max_velocity");
            s
        }),
        ("missing max_accel", {
            let mut s = Setup::base();
            s.options.retain(|(k, _)| *k != "max_accel");
            s
        }),
        ("zero max_velocity", Setup::with_options(&[("max_velocity", "0")])),
        ("negative max_accel", Setup::with_options(&[("max_accel", "-3")])),
        ("unparsable max_velocity", Setup::with_options(&[("max_velocity", "fast")])),
        ("mcr one", Setup::with_options(&[("minimum_cruise_ratio", "1.0")])),
        ("mcr negative", Setup::with_options(&[("minimum_cruise_ratio", "-0.5")])),
        ("scv negative", Setup::with_options(&[("square_corner_velocity", "-1")])),
        ("missing kinematics", {
            let mut s = Setup::base();
            s.options.retain(|(k, _)| *k != "kinematics");
            s
        }),
        ("missing mcu", {
            let mut s = Setup::base();
            s.has[0] = false;
            s
        }),
        ("missing motion_queuing", {
            let mut s = Setup::base();
            s.has[1] = false;
            s
        }),
        ("missing gcode", {
            let mut s = Setup::base();
            s.has[2] = false;
            s
        }),
    ];
    let mut py = oracle();
    for (label, setup) in &cases {
        build_pair(&mut py, label, setup, false);
    }
}

#[test]
fn kinematics_loading_matches_python() {
    let mut py = oracle();
    for name in ["bogus", "cfgerr", "pinserr", "generr", "cmderr", "fakekin"] {
        let setup = Setup::with_options(&[("kinematics", name)]);
        build_pair(&mut py, &format!("kinematics {name}"), &setup, false);
    }
    // The generic failure also looks up `pins`, which can be missing.
    let mut setup = Setup::with_options(&[("kinematics", "generr")]);
    setup.has[3] = false;
    build_pair(&mut py, "generr without pins", &setup, false);
    let mut setup = Setup::with_options(&[("kinematics", "bogus")]);
    setup.has[3] = false;
    build_pair(&mut py, "bogus without pins", &setup, false);
    // A config error does not consult `pins`.
    let mut setup = Setup::with_options(&[("kinematics", "cfgerr")]);
    setup.has[3] = false;
    build_pair(&mut py, "cfgerr without pins", &setup, false);
}

#[test]
fn motion_flow_matches_python() {
    let mut ops: Vec<Op> = vec![Op::SetExtruder("e1", 0.0), Op::LastMoveTime, Op::FlushCb(0.0, 1e9)];
    let moves = move_ops(2024, 60, 0);
    for (i, op) in moves.into_iter().enumerate() {
        ops.push(op);
        match i {
            4 => ops.push(Op::FlushCb(0.0, -1e9)),
            11 => ops.push(Op::KinLimits(Some((120.0, 2000.0)))),
            17 => ops.push(Op::LookaheadCb(1)),
            20 => ops.push(Op::KinLimits(None)),
            23 => ops.push(Op::LimitNext(20.0)),
            29 => ops.push(Op::Dwell(0.5)),
            33 => ops.push(Op::LookaheadCb(2)),
            34 => ops.push(Op::FlushCb(0.0, 1e9)),
            38 => ops.push(Op::LastMoveTime),
            41 => ops.push(Op::WaitMoves),
            45 => ops.push(Op::FlushStep),
            50 => ops.push(Op::StepRestart(30.0)),
            52 => ops.push(Op::ManualMove(vec![Some(10.0), None, Some(2.0)], 80.0)),
            _ => {}
        }
    }
    ops.push(Op::LookaheadCb(3));
    ops.push(Op::LastMoveTime);
    ops.push(Op::LookaheadCb(4));
    ops.push(Op::Dwell(-1.0));
    ops.push(Op::Dwell(0.0));
    scenario("motion flow", &Setup::base(), false, &ops);
}

#[test]
fn can_pause_flow_matches_python() {
    // The estimated print time is far ahead of the moves, so the pause loop never waits.
    let mut setup = Setup::base();
    setup.fileoutput = false;
    setup.est = 1000.0;
    let mut ops: Vec<Op> = vec![Op::SetExtruder("e1", 0.0), Op::LastMoveTime];
    ops.push(Op::StepRestart(1000.4));
    // Short fast moves: the buffered print time stays well below the two second limit.
    let points: [[f64; 4]; 10] = [
        [2.0, 0.0, 0.0, 0.1],
        [4.0, 1.0, 0.0, 0.2],
        [6.0, 1.0, 0.2, 0.3],
        [6.0, 3.0, 0.2, 0.5],
        [4.0, 3.0, 0.2, 0.5],
        [2.0, 2.0, 0.2, 0.6],
        [1.0, 2.0, 0.2, 0.6],
        [1.0, 0.0, 0.2, 0.8],
        [1.0, 0.0, 0.2, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    for (i, p) in points.iter().enumerate() {
        ops.push(Op::Move(p.to_vec(), 100.0));
        if i == 3 {
            ops.push(Op::Dwell(0.25));
        }
    }
    ops.push(Op::FlushCb(0.0, 1e9));
    ops.push(Op::LastMoveTime);
    ops.push(Op::Move(vec![1.0, 1.0, 1.0, 1.0], 100.0));
    ops.push(Op::Drip(vec![3.0, 1.0, 1.0], 100.0));
    ops.push(Op::FlushStep);
    scenario("can_pause flow", &setup, false, &ops);
}

#[test]
fn move_errors_match_python() {
    let mut ops = vec![
        Op::Move(vec![10.0, 0.0, 0.0, 0.0], 50.0),
        // Extrusion without an extruder: the dummy extruder refuses.
        Op::Move(vec![10.0, 0.0, 0.0, 2.0], 50.0),
        Op::Move(vec![12.0, 0.0, 0.0, 2.0], 50.0),
        Op::KinFail(true),
        Op::Move(vec![20.0, 0.0, 0.0, 0.0], 50.0),
        // An extrude-only move does not reach the kinematics check.
        Op::Move(vec![10.0, 0.0, 0.0, 0.0], 50.0),
        Op::KinFail(false),
        Op::Move(vec![20.0, 0.0, 0.0, 0.0], 0.0),
        Op::Move(vec![10.0, 0.0, 0.0, 0.0], 0.0),
        Op::Move(vec![20.0, 5.0, 0.0, 0.0], 50.0),
        Op::ManualMove(vec![Some(1.0), Some(2.0), Some(3.0), None, Some(4.0)], 50.0),
        Op::ManualMove(vec![None, None, None, None], 50.0),
        Op::ManualMove(vec![Some(30.0)], 50.0),
        Op::ManualMove(vec![], 50.0),
        Op::Move(vec![20.0, 5.0, 0.0, 0.0], 50.0),
        Op::LastMoveTime,
    ];
    ops.extend(move_ops(77, 12, 0).into_iter().map(|o| match o {
        Op::Move(mut p, s) => {
            p[3] = 0.0;
            Op::Move(p, s)
        }
        other => other,
    }));
    scenario("move errors", &Setup::base(), false, &ops);
}

#[test]
fn extra_axes_match_python() {
    let mut ops: Vec<Op> = vec![
        Op::ExtraAxes,
        Op::ExtruderName,
        Op::AddAxis("a", 1.5),
        Op::ExtraAxes,
        Op::Move(vec![5.0, 0.0, 0.0, 0.0, 2.0], 50.0),
        Op::Move(vec![5.0, 0.0, 0.0, 0.0, 4.0], 50.0),
        Op::Move(vec![10.0, 5.0, 0.0, 0.0, 4.0], 50.0),
        Op::AxisJunction("a", Some(100.0)),
        Op::Move(vec![10.0, 15.0, 0.0, 0.0, 6.0], 50.0),
        Op::Move(vec![20.0, 15.0, 0.0, 0.0, 6.5], 50.0),
        Op::AxisJunction("a", None),
        Op::AxisFail("a", Some("Axis A limit")),
        Op::Move(vec![25.0, 15.0, 0.0, 0.0, 7.0], 50.0),
        Op::Move(vec![25.0, 15.0, 0.0, 0.0, 7.5], 50.0),
        Op::AxisFail("a", None),
        Op::AddAxis("b", -1.0),
        Op::ExtraAxes,
        Op::Move(vec![30.0, 15.0, 0.0, 0.0, 8.0, 3.0], 50.0),
        Op::Move(vec![30.0, 25.0, 0.0, 0.0, 8.0, 1.0], 50.0),
        Op::RemoveAxis("a"),
        Op::ExtraAxes,
        Op::Move(vec![35.0, 25.0, 0.0, 0.0, 0.5], 50.0),
        Op::RemoveAxis("a"),
        Op::SetExtruder("e1", 2.0),
        Op::ExtruderName,
        Op::ExtraAxes,
        Op::Move(vec![35.0, 25.0, 0.0, 3.0, 0.5], 50.0),
        Op::Move(vec![40.0, 25.0, 0.0, 4.5, 1.5], 50.0),
        Op::AxisFail("e1", Some("Extruder cold")),
        Op::Move(vec![40.0, 25.0, 0.0, 5.0, 1.5], 50.0),
        Op::AxisFail("e1", None),
        Op::RemoveAxis("b"),
        Op::Move(vec![45.0, 25.0, 0.0, 5.0], 50.0),
        Op::LastMoveTime,
        Op::FlushStep,
    ];
    ops.insert(5, Op::LookaheadCb(9));
    scenario("extra axes", &Setup::base(), false, &ops);

    // Extra axes in a long polyline: junction limits and `process_move` calls per move.
    let mut ops: Vec<Op> = vec![Op::AddAxis("a", 0.0), Op::SetExtruder("e1", 0.0)];
    ops.push(Op::AxisJunction("a", Some(400.0)));
    ops.extend(move_ops(31337, 40, 1));
    ops.push(Op::LastMoveTime);
    scenario("extra axes polyline", &Setup::base(), false, &ops);
}

#[test]
fn position_drip_and_shutdown_match_python() {
    let ops = vec![
        Op::SetPosition(vec![5.0, 6.0, 7.0, 0.0], Some("xy")),
        Op::SetPosition(vec![1.0, 2.0, 3.0, 9.0], None),
        Op::Move(vec![10.0, 2.0, 3.0, 9.0], 50.0),
        Op::Move(vec![10.0, 12.0, 3.0, 9.0], 50.0),
        Op::Drip(vec![30.0, 12.0, 3.0], 100.0),
        Op::Move(vec![30.0, 20.0, 3.0, 9.0], 50.0),
        Op::Drip(vec![30.0, 20.0, 3.0], 100.0),
        Op::Drip(vec![30.0, 40.0, 3.0, 99.0], 80.0),
        Op::KinFail(true),
        Op::Drip(vec![60.0, 40.0, 3.0], 80.0),
        Op::KinFail(false),
        Op::Move(vec![40.0, 40.0, 3.0, 9.0], 50.0),
        Op::SetPosition(vec![0.0, 0.0, 0.0, 0.0], Some("")),
        Op::ManualMove(vec![Some(3.0), Some(4.0)], 25.0),
        Op::Move(vec![8.0, 4.0, 0.0, 0.0], 50.0),
        Op::Move(vec![8.0, 9.0, 0.0, 0.0], 50.0),
        Op::Shutdown,
        Op::Move(vec![18.0, 9.0, 0.0, 0.0], 50.0),
        Op::LastMoveTime,
        Op::SetPosition(vec![2.0, 2.0, 2.0, 2.0], Some("z")),
    ];
    scenario("position drip shutdown", &Setup::base(), false, &ops);
}

#[test]
fn stalls_and_priming_states_match_python() {
    let mut ops: Vec<Op> = vec![Op::SetExtruder("e1", 0.0)];
    // Reach the main state, run out of moves, then start again: the late restart counts a stall.
    for (i, op) in move_ops(8, 16, 0).into_iter().enumerate() {
        ops.push(op);
        if i == 14 {
            ops.push(Op::FlushCb(0.0, 1e9));
            ops.push(Op::SetEst(0.0));
        }
    }
    ops.push(Op::Move(vec![1.0, 1.0, 1.0, 1.0], 20.0));
    ops.push(Op::Move(vec![5.0, 1.0, 1.0, 1.0], 20.0));
    ops.push(Op::FlushCb(0.0, 1e9));
    ops.push(Op::SetEst(1.0e6));
    ops.push(Op::Move(vec![9.0, 1.0, 1.0, 1.0], 20.0));
    ops.push(Op::Move(vec![9.0, 5.0, 1.0, 1.0], 20.0));
    ops.push(Op::FlushStep);
    ops.push(Op::Move(vec![9.0, 9.0, 1.0, 1.0], 20.0));
    ops.push(Op::WaitMoves);
    ops.push(Op::LastMoveTime);
    scenario("stalls and priming", &Setup::base(), false, &ops);
}

#[test]
fn velocity_limits_match_python() {
    let ops = vec![
        Op::Move(vec![20.0, 0.0, 0.0, 0.0], 100.0),
        Op::SetMaxVel(Some(100.0), Some(2000.0), Some(7.0), Some(0.25)),
        Op::Move(vec![20.0, 20.0, 0.0, 0.0], 400.0),
        Op::Move(vec![40.0, 20.0, 0.0, 0.0], 400.0),
        Op::SetMaxVel(None, None, None, None),
        Op::SetMaxVel(None, Some(800.0), None, None),
        Op::Move(vec![40.0, 40.0, 0.0, 0.0], 400.0),
        Op::SetMaxVel(Some(60.0), None, Some(0.0), None),
        Op::Move(vec![60.0, 40.0, 0.0, 0.0], 400.0),
        Op::SetMaxVel(None, None, Some(12.5), Some(0.0)),
        Op::Move(vec![60.0, 60.0, 0.0, 0.0], 400.0),
        Op::SetMaxVel(Some(250.0), Some(3000.0), Some(5.0), Some(0.5)),
        Op::Move(vec![0.0, 0.0, 0.0, 0.0], 400.0),
        Op::LastMoveTime,
    ];
    scenario("velocity limits", &Setup::base(), false, &ops);
    let setup = Setup::with_options(&[("minimum_cruise_ratio", "0.9"), ("square_corner_velocity", "2")]);
    scenario("velocity limits mcr", &setup, false, &ops);
}

#[test]
fn command_helper_matches_python() {
    let cmd = Op::Cmd;
    let ops: Vec<Op> = vec![
        cmd("G4", "G4 P500", vec![("P", "500")]),
        cmd("G4", "G4", vec![]),
        cmd("G4", "G4 P0.5", vec![("P", "0.5")]),
        cmd("G4", "G4 P-1", vec![("P", "-1")]),
        cmd("G4", "G4 Pabc", vec![("P", "abc")]),
        Op::Move(vec![10.0, 0.0, 0.0, 0.0], 50.0),
        Op::Move(vec![10.0, 10.0, 0.0, 0.0], 50.0),
        cmd("G4", "G4 P250", vec![("P", "250")]),
        cmd("M400", "M400", vec![]),
        Op::Move(vec![20.0, 10.0, 0.0, 0.0], 50.0),
        cmd("M400", "M400", vec![]),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT", vec![]),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT VELOCITY=100", vec![("VELOCITY", "100")]),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT ACCEL=2000", vec![("ACCEL", "2000")]),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=7",
            vec![("SQUARE_CORNER_VELOCITY", "7")],
        ),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=0.25",
            vec![("MINIMUM_CRUISE_RATIO", "0.25")],
        ),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=4000 SQUARE_CORNER_VELOCITY=0 MINIMUM_CRUISE_RATIO=0",
            vec![
                ("VELOCITY", "300"),
                ("ACCEL", "4000"),
                ("SQUARE_CORNER_VELOCITY", "0"),
                ("MINIMUM_CRUISE_RATIO", "0"),
            ],
        ),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT VELOCITY=0", vec![("VELOCITY", "0")]),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT ACCEL=-1", vec![("ACCEL", "-1")]),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=-1",
            vec![("SQUARE_CORNER_VELOCITY", "-1")],
        ),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=1",
            vec![("MINIMUM_CRUISE_RATIO", "1")],
        ),
        cmd(
            "SET_VELOCITY_LIMIT",
            "SET_VELOCITY_LIMIT MINIMUM_CRUISE_RATIO=-0.5",
            vec![("MINIMUM_CRUISE_RATIO", "-0.5")],
        ),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT VELOCITY=abc", vec![("VELOCITY", "abc")]),
        cmd("SET_VELOCITY_LIMIT", "SET_VELOCITY_LIMIT", vec![]),
        cmd("M204", "M204 S1500", vec![("S", "1500")]),
        cmd("M204", "M204 S0", vec![("S", "0")]),
        cmd("M204", "M204 Sabc", vec![("S", "abc")]),
        cmd("M204", "M204 P2000 T1500", vec![("P", "2000"), ("T", "1500")]),
        cmd("M204", "M204 P1000 T3000", vec![("P", "1000"), ("T", "3000")]),
        cmd("M204", "M204 P1000", vec![("P", "1000")]),
        cmd("M204", "M204 T1000", vec![("T", "1000")]),
        cmd("M204", "M204", vec![]),
        cmd("M204", "M204 P-1 T5", vec![("P", "-1"), ("T", "5")]),
        cmd("M204", "M204 P100 Tx", vec![("P", "100"), ("T", "x")]),
        cmd("M204", "M204 S800 P1 T2", vec![("S", "800"), ("P", "1"), ("T", "2")]),
        Op::Move(vec![30.0, 10.0, 0.0, 0.0], 50.0),
        Op::LastMoveTime,
    ];
    scenario("command helper", &Setup::base(), true, &ops);
    // A toolhead that can pause: G4 and M400 with the estimate far ahead of the print time.
    let mut setup = Setup::base();
    setup.fileoutput = false;
    setup.est = 2000.0;
    let ops: Vec<Op> = vec![
        cmd("G4", "G4 P100", vec![("P", "100")]),
        Op::Move(vec![4.0, 0.0, 0.0, 0.0], 50.0),
        cmd("G4", "G4 P100", vec![("P", "100")]),
        Op::SetEst(1.0e9),
        cmd("M400", "M400", vec![]),
    ];
    scenario("command helper pausing toolhead", &setup, true, &ops);
}

#[test]
fn command_helper_without_objects_matches_python() {
    let mut py = oracle();
    // `ToolHeadCommandHelper` needs the `toolhead` object: construct it without registering it.
    let setup = Setup::base();
    py.exec(&py_world(&setup)).expect("py world");
    py.exec("_construct(False)").expect("py construct");
    let expected = py.eval("_step(lambda: toolhead.ToolHeadCommandHelper(cfg))");
    let mut world = World::new(&setup);
    let _ = world.construct(false);
    let cfg = world.cfg();
    let r = ToolHeadCommandHelper::new(&cfg).map(|_| ());
    let actual = vt(vec![unit(r), world.observe(), world.drain_log(), drain_lines()]);
    check("helper without toolhead object", expected, actual);
}

#[test]
fn add_printer_objects_matches_python() {
    let all = vec![
        "gcode_move",
        "homing",
        "idle_timeout",
        "statistics",
        "manual_probe",
        "tuning_tower",
        "garbage_collection",
    ];
    let mut py = oracle();

    // Without a registered `kinematics.extruder` hook the call fails after the toolhead and its
    // commands exist (there is no Python counterpart: the real module always has the function).
    {
        let mut setup = Setup::base();
        setup.loadable = all.clone();
        let world = World::new(&setup);
        let err = add_printer_objects(&world.cfg()).expect_err("no extruder hook registered");
        assert_eq!(err.class_name(), "AttributeError");
        assert_eq!(
            err.message(),
            "module 'kinematics.extruder' has no attribute 'add_printer_objects'"
        );
        assert!(world.printer.objects.lock().contains_key("toolhead"));
    }

    register_extruder_objects(extruder_objects);
    let mut cases: Vec<(&str, Setup)> = Vec::new();
    let mut s = Setup::base();
    s.loadable = all.clone();
    cases.push(("all modules", s));
    let mut s = Setup::base();
    s.loadable = all.iter().copied().filter(|m| *m != "homing").collect();
    cases.push(("homing missing", s));
    let mut s = Setup::base();
    s.loadable = all.iter().copied().filter(|m| *m != "garbage_collection").collect();
    cases.push(("garbage_collection missing", s));
    let mut s = Setup::with_options(&[("kinematics", "bogus")]);
    s.loadable = all.clone();
    cases.push(("bad kinematics", s));
    let mut s = Setup::with_options(&[("max_velocity", "0")]);
    s.loadable = all.clone();
    cases.push(("bad config", s));

    for (label, setup) in &cases {
        py.exec(&py_world(setup)).expect("py world");
        let expected = py.eval("_add_objects()");
        let world = World::new(setup);
        let r = add_printer_objects(&world.cfg());
        let keys: Vec<Value> =
            world.printer.objects.lock().keys().map(|k| vs(k)).collect();
        let actual = vt(vec![unit(r), world.drain_log(), Value::List(keys), drain_lines()]);
        check(&format!("add_printer_objects {label}"), expected, actual);
    }
}

#[test]
fn priming_handler_matches_python() {
    // The priming timer runs in the real reactor: the moves queue in the "Priming" state, the
    // timer fires after `PRIMING_CMD_TIME` and flushes the lookahead queue. The Python side
    // calls `_priming_handler` directly.
    let mut setup = Setup::base();
    setup.fileoutput = false;
    setup.est = 100.0;
    let first = Op::Move(vec![10.0, 0.0, 0.0, 0.0], 50.0);
    let second = Op::Move(vec![10.0, 10.0, 0.0, 0.0], 50.0);
    let after = Op::Move(vec![0.0, 10.0, 0.0, 0.0], 50.0);

    let mut py = oracle();
    py.exec(&py_world(&setup)).expect("py world");
    let expected_construct = py.eval("_construct(False)");
    let expected_first = py.eval(&format!("_step(lambda: {})", first.py()));
    let expected_second = py.eval(&format!("_step(lambda: {})", second.py()));
    let expected_fire = py.eval("_step(lambda: th._priming_handler(0.))");
    let expected_after = py.eval(&format!("_step(lambda: {})", after.py()));

    let mut world = World::new(&setup);
    let actual_construct = world.construct(false);
    let reactor = world.printer.reactor.clone();
    let results: Arc<Mutex<Vec<Value>>> = Arc::new(Mutex::new(Vec::new()));
    let out = Arc::clone(&results);
    let r2 = reactor.clone();
    let _callback = reactor.register_callback::<_, ()>(
        move |_eventtime| {
            let a = world.step(&first);
            let b = world.step(&second);
            // Let the priming timer expire (it is set to `eventtime + PRIMING_CMD_TIME`).
            let now = r2.monotonic();
            r2.pause(now + 0.4);
            let observed = world.observe();
            let log = world.drain_log();
            let c = vt(vec![ok(vf(NEVER)), observed, log, drain_lines()]);
            let d = world.step(&after);
            out.lock().extend([a, b, c, d]);
            r2.end();
        },
        NOW,
    );
    let r3 = reactor.clone();
    let _safety = reactor.register_timer(
        move |_eventtime| {
            // Safety net: end the reactor if the scenario above never finishes.
            r3.end();
            NEVER
        },
        reactor.monotonic() + 20.0,
    );
    reactor.run();

    let got = results.lock().clone();
    assert_eq!(got.len(), 4, "the scenario did not finish inside the reactor");
    check("priming construction", expected_construct, actual_construct);
    check("priming first move", expected_first, got[0].clone());
    check("priming second move", expected_second, got[1].clone());
    check("priming timer fired", expected_fire, got[2].clone());
    check("move after priming", expected_after, got[3].clone());
}

#[test]
fn lookahead_errors_propagate_through_all_flush_consumers() {
    let consumers = [
        Op::Dwell(0.01),
        Op::WaitMoves,
        Op::LastMoveTime,
        Op::AddAxis("e1", 0.0),
        Op::RemoveAxis("e1"),
        Op::FlushCb(0.0, 1e9),
        Op::Cmd("G4", "G4 P10", vec![("P", "10")]),
        Op::Cmd("M400", "M400", vec![]),
    ];
    for consumer in consumers {
        scenario("lookahead exception", &Setup::base(), true, &[
            Op::Move(vec![10.0, 0.0, 0.0, 0.0], 20.0),
            Op::LastMoveTime,
            Op::Move(vec![20.0, 0.0, 0.0, 0.0], 20.0),
            Op::LookaheadError,
            consumer,
        ]);
    }
}

#[test]
fn move_speed_limit_errors_and_partial_state_match_python() {
    let mut py = oracle();
    let printer = limits_world(&mut py);
    for speed in [0.0, -0.0, 1e308] {
        py.exec("m = toolhead.Move(th, [0., 0., 0., 0.], [10., 0., 0., 0.], 20.)").unwrap();
        let m = new_move(&printer, &[0.0; 4], &[10.0, 0.0, 0.0, 0.0], 20.0).unwrap();
        let expected = py.eval(&format!("(_try(lambda: m.limit_speed({}, 100.)), _mfields(m))", pyf(speed)));
        let actual = vt(vec![unit(m.limit_speed(speed, 100.0)), move_fields(&m)]);
        check("speed-limit exception", expected, actual);
    }
}
