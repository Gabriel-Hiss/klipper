//! Parity tests for `extras::force_move` (port of `klippy/extras/force_move.py`).
//!
//! Everything is compared against the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): `calc_move_time`, the `BUZZ_*`
//! constants, `ForceMove` construction, `register_stepper`, `lookup_stepper`,
//! `manual_move`, and the `STEPPER_BUZZ` / `FORCE_MOVE` /
//! `SET_KINEMATIC_POSITION` commands are driven on both sides with minimal
//! Python fakes for the printer, motion queuing, toolhead, steppers and
//! g-code dispatch. Returned values (floats by bits), raised errors (class
//! and message), recorded collaborator calls, registration tables and log
//! lines are compared exactly with [`recording::assert_equal`].
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

use extras::force_move::{
    calc_move_time, module_factory, ForceMove, ForceMoveApi, BUZZ_DISTANCE, BUZZ_RADIANS_DISTANCE, BUZZ_RADIANS_VELOCITY,
    BUZZ_VELOCITY, CMD_FORCE_MOVE_HELP, CMD_SET_KINEMATIC_POSITION_HELP, CMD_STEPPER_BUZZ_HELP,
};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::mcu::Mcu;
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, Stepper, StepperKinematics, StepperKinematicsApi, Trapq, TrapqApi,
};
use klippy_api::stepper::StepperFactory;
use klippy_api::toolhead::{
    ExtraAxis, FlushCallback, HomingTargetApi, Kinematics, KinematicsApi, LookaheadCallback,
    MotionQueuingApi, PrinterExtruder, SyncEmitterHandle, ToolHeadApi, TrapqAppend,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::text::{float_from_string, float_repr};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::{Arc, Mutex};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import extras.force_move as force_move

class CommandError(Exception):
    pass

class _Sentinel:
    pass
SENTINEL = _Sentinel()

PY_CALLS = []
PY_LOGS = []

class FakeLogging:
    def info(self, msg, *args):
        if args:
            PY_LOGS.append(msg % args)
        else:
            PY_LOGS.append(msg)
force_move.logging = FakeLogging()

CY_ALLOC = ('cartesian_sk', b'x')
CY_ORIG_SK = ('orig_sk',)
CY_ORIG_TQ = ('orig_tq',)

class FakeTrapq:
    pass

class FakeMotionQueuing:
    def __init__(self):
        self.trapq = FakeTrapq()
    def allocate_trapq(self):
        PY_CALLS.append(('allocate_trapq',))
        return self.trapq
    def lookup_trapq_append(self):
        PY_CALLS.append(('lookup_trapq_append',))
        mq = self
        def trapq_append(trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, sv, cv, accel):
            tag = 'TQ' if trapq is mq.trapq else 'TQ_OTHER'
            PY_CALLS.append(('trapq_append', tag, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, sv, cv, accel))
        return trapq_append
    def note_mcu_movequeue_activity(self, mq_time, is_step_gen=True):
        PY_CALLS.append(('note_mcu_movequeue_activity', mq_time, is_step_gen))
    def wipe_trapq(self, trapq):
        tag = 'TQ' if trapq is self.trapq else 'TQ_OTHER'
        PY_CALLS.append(('wipe_trapq', tag))

class FakeChelperLib:
    def cartesian_stepper_alloc(self, axis):
        assert axis == b'x', axis
        return CY_ALLOC
    def free(self, x):
        pass
class FakeChelperMain:
    def gc(self, obj, destructor):
        return obj
class FakeChelper:
    def __init__(self):
        self.lib = FakeChelperLib()
    def get_ffi(self):
        return (FakeChelperMain(), self.lib)
force_move.chelper = FakeChelper()

class FakeKinematics:
    def clear_homing_state(self, axes):
        PY_CALLS.append(('clear_homing_state', axes))

class FakeToolhead:
    def __init__(self, pos=None, last_time=100.0):
        self.pos = list(pos) if pos is not None else [10.0, 20.0, 30.0, 0.0]
        self.last_time = last_time
        self.kin = FakeKinematics()
    def flush_step_generation(self):
        PY_CALLS.append(('flush_step_generation',))
    def get_last_move_time(self):
        return self.last_time
    def dwell(self, delay):
        PY_CALLS.append(('dwell', delay))
    def get_position(self):
        return list(self.pos)
    def set_position(self, newpos, homing_axes=None):
        PY_CALLS.append(('set_position', newpos[0], newpos[1], newpos[2], homing_axes))
        self.pos = list(newpos)
    def get_kinematics(self):
        return self.kin

class FakeStepper:
    def __init__(self, name, radians=False):
        self._name = name
        self._radians = radians
        self.prev_sk = CY_ORIG_SK
        self.prev_tq = CY_ORIG_TQ
    def get_name(self, short=None):
        return self._name
    def units_in_radians(self):
        return self._radians
    def _sk_tag(self, sk):
        if sk == CY_ALLOC:
            return 'SK'
        if sk == CY_ORIG_SK:
            return 'SK_ORIG'
        return 'SK_OTHER'
    def _tq_tag(self, tq):
        if tq is not None and hasattr(tq, '_is_fm_trapq'):
            return 'TQ'
        if tq == CY_ORIG_TQ:
            return 'TQ_ORIG'
        return 'TQ_OTHER'
    def set_stepper_kinematics(self, sk):
        PY_CALLS.append(('set_stepper_kinematics', self._sk_tag(sk)))
        old = self.prev_sk
        self.prev_sk = sk
        return old
    def set_trapq(self, tq):
        PY_CALLS.append(('set_trapq', self._tq_tag(tq)))
        old = self.prev_tq
        self.prev_tq = tq
        return old
    def set_position(self, coord):
        PY_CALLS.append(('stepper_set_position', coord[0], coord[1], coord[2]))

class FakeStepperEnable:
    def __init__(self, did_enable=True):
        self.did_enable = did_enable
    def set_motors_enable(self, names, enable):
        PY_CALLS.append(('set_motors_enable', list(names), enable))
        return self.did_enable

class FakeGcode:
    def register_command(self, cmd, func, desc=None, when_not_ready=False):
        PY_CALLS.append(('register_command', cmd, desc))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        PY_CALLS.append(('register_mux_command', cmd, key, value, desc))

class FakePrinter:
    config_error = configparser.Error
    def __init__(self, mq=None, toolhead=None, gcode=None, stepper_enable=None):
        self.mq = mq
        self.toolhead = toolhead
        self.gcode = gcode
        self.stepper_enable = stepper_enable
    def load_object(self, config, section, default=None):
        if section == 'motion_queuing' and self.mq is not None:
            return self.mq
        raise configparser.Error("Unable to load module '%s'" % (section,))
    def lookup_object(self, name, default=None):
        obj = {'toolhead': self.toolhead, 'gcode': self.gcode,
               'stepper_enable': self.stepper_enable}.get(name)
        if obj is None:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return obj

def _parse_bool(raw):
    v = raw.strip().lower()
    if v in ('true', '1', 'yes', 'on'):
        return True
    if v in ('false', '0', 'no', 'off'):
        return False
    raise ValueError('bad bool')

class FakeConfig:
    error = configparser.Error
    section = 'force_move'
    def __init__(self, printer, options=None):
        self._printer = printer
        self._options = options or {}
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def getboolean(self, option, default=SENTINEL):
        if option not in self._options:
            if default is SENTINEL:
                raise configparser.Error("Option '%s' in section '%s' must be specified"
                                         % (option, self.section))
            return default
        try:
            return _parse_bool(self._options[option])
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'"
                                     % (option, self.section))

class FakeGcmd:
    error = CommandError
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = params
    def get_commandline(self):
        return self._commandline
    def get(self, name, default=SENTINEL, parser=str, minval=None, maxval=None, above=None, below=None):
        if name not in self._params:
            if default is SENTINEL:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        value = self._params[name]
        try:
            value = parser(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s"
                             % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s"
                             % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s"
                             % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s"
                             % (self._commandline, name, below))
        return value
    def get_float(self, name, default=SENTINEL, minval=None, maxval=None, above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval,
                        above=above, below=below)

def make_world(enable=None, position=(10.0, 20.0, 30.0, 0.0), last_time=100.0,
               did_enable=True, radians=False, with_gcode=True):
    PY_CALLS.clear()
    PY_LOGS.clear()
    mq = FakeMotionQueuing()
    mq.trapq._is_fm_trapq = True
    th = FakeToolhead(pos=list(position), last_time=last_time)
    gc = FakeGcode() if with_gcode else None
    se = FakeStepperEnable(did_enable=did_enable)
    p = FakePrinter(mq=mq, toolhead=th, gcode=gc, stepper_enable=se)
    opts = {} if enable is None else {'enable_force_move': enable}
    c = FakeConfig(p, opts)
    fm = force_move.ForceMove(c)
    stepper = FakeStepper('stepper_x', radians=radians)
    fm.register_stepper(c, stepper)
    return fm, mq, th, gc, se, stepper
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum CallEntry {
    AllocateTrapq,
    LookupTrapqAppend,
    TrapqAppend { tq: Trapq, args: [f64; 13] },
    NoteMcuMovequeueActivity(f64, bool),
    WipeTrapq(Trapq),
    FlushStepGeneration,
    Dwell(f64),
    SetPosition([f64; 3], String),
    ClearHomingState(String),
    SetStepperKinematics(Option<StepperKinematics>),
    SetTrapq(Option<Trapq>),
    StepperSetPosition([f64; 3]),
    SetMotorsEnable(Vec<String>, bool),
    RegisterCommand(String, Option<String>),
    RegisterMuxCommand(String, String, Option<String>, Option<String>),
}

type CallLog = Arc<Mutex<Vec<CallEntry>>>;

fn record(calls: &CallLog, entry: CallEntry) {
    calls.lock().expect("call log lock").push(entry);
}

fn s(v: &str) -> Value {
    Value::Str(v.to_string())
}

fn f(v: f64) -> Value {
    Value::float(v)
}

fn sk_tag(h: &StepperKinematics, fm_sk: &StepperKinematics, orig_sk: &StepperKinematics) -> &'static str {
    if Arc::ptr_eq(h, fm_sk) {
        "SK"
    } else if Arc::ptr_eq(h, orig_sk) {
        "SK_ORIG"
    } else {
        "SK_OTHER"
    }
}

fn tq_tag(h: &Trapq, fm_tq: &Trapq, orig_tq: &Trapq) -> &'static str {
    if Arc::ptr_eq(h, fm_tq) {
        "TQ"
    } else if Arc::ptr_eq(h, orig_tq) {
        "TQ_ORIG"
    } else {
        "TQ_OTHER"
    }
}

fn entry_value(
    e: &CallEntry,
    fm_sk: &StepperKinematics,
    fm_tq: &Trapq,
    orig_sk: &StepperKinematics,
    orig_tq: &Trapq,
) -> Value {
    let t = |items: Vec<Value>| Value::Tuple(items);
    match e {
        CallEntry::AllocateTrapq => t(vec![s("allocate_trapq")]),
        CallEntry::LookupTrapqAppend => t(vec![s("lookup_trapq_append")]),
        CallEntry::TrapqAppend { tq, args } => {
            let mut items = vec![s("trapq_append"), s(tq_tag(tq, fm_tq, orig_tq))];
            items.extend(args.iter().map(|&a| f(a)));
            t(items)
        }
        CallEntry::NoteMcuMovequeueActivity(mq_time, is_step_gen) => {
            t(vec![s("note_mcu_movequeue_activity"), f(*mq_time), Value::Bool(*is_step_gen)])
        }
        CallEntry::WipeTrapq(tq) => t(vec![s("wipe_trapq"), s(tq_tag(tq, fm_tq, orig_tq))]),
        CallEntry::FlushStepGeneration => t(vec![s("flush_step_generation")]),
        CallEntry::Dwell(delay) => t(vec![s("dwell"), f(*delay)]),
        CallEntry::SetPosition(pos, axes) => {
            t(vec![s("set_position"), f(pos[0]), f(pos[1]), f(pos[2]), s(axes)])
        }
        CallEntry::ClearHomingState(axes) => t(vec![s("clear_homing_state"), s(axes)]),
        CallEntry::SetStepperKinematics(Some(h)) => {
            t(vec![s("set_stepper_kinematics"), s(sk_tag(h, fm_sk, orig_sk))])
        }
        CallEntry::SetStepperKinematics(None) => t(vec![s("set_stepper_kinematics"), s("SK_NONE")]),
        CallEntry::SetTrapq(Some(h)) => {
            t(vec![s("set_trapq"), s(tq_tag(h, fm_tq, orig_tq))])
        }
        CallEntry::SetTrapq(None) => t(vec![s("set_trapq"), s("TQ_NONE")]),
        CallEntry::StepperSetPosition(pos) => {
            t(vec![s("stepper_set_position"), f(pos[0]), f(pos[1]), f(pos[2])])
        }
        CallEntry::SetMotorsEnable(names, enable) => t(vec![
            s("set_motors_enable"),
            Value::List(names.iter().map(|n| s(n)).collect()),
            Value::Bool(*enable),
        ]),
        CallEntry::RegisterCommand(cmd, desc) => t(vec![
            s("register_command"),
            s(cmd),
            desc.clone().map_or(Value::Null, |d| s(&d)),
        ]),
        CallEntry::RegisterMuxCommand(cmd, key, value, desc) => t(vec![
            s("register_mux_command"),
            s(cmd),
            s(key),
            value.clone().map_or(Value::Null, |v| s(&v)),
            desc.clone().map_or(Value::Null, |d| s(&d)),
        ]),
    }
}

fn calls_value(
    calls: &[CallEntry],
    fm: &ForceMove,
    orig_sk: &StepperKinematics,
    orig_tq: &Trapq,
) -> Value {
    Value::List(
        calls
            .iter()
            .map(|e| entry_value(e, &fm.stepper_kinematics(), &fm.trapq(), orig_sk, orig_tq))
            .collect(),
    )
}

// --- Log capture (lines kept per test thread; the Rust sink is process wide) ---

thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_LINES.with(|lines| lines.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

fn lock_logs() {
    install_log_sink();
    LOG_LINES.with(|lines| lines.borrow_mut().clear());
}

fn take_logs() -> Vec<String> {
    LOG_LINES.with(|lines| std::mem::take(&mut *lines.borrow_mut()))
}

// --- Fake handles ---

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

struct FakeSk;

impl StepperKinematicsApi for FakeSk {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

// --- FakeMotionQueuing ---

struct FakeMotionQueuing {
    calls: CallLog,
    trapq: Trapq,
}

impl MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> Trapq {
        record(&self.calls, CallEntry::AllocateTrapq);
        self.trapq.clone()
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        record(&self.calls, CallEntry::LookupTrapqAppend);
        let calls = self.calls.clone();
        let own_trapq = self.trapq.clone();
        let append: TrapqAppend = Arc::new(
            move |tq: &Trapq, a: f64, b: f64, c: f64, d: f64, e: f64, f1: f64, g: f64, h: f64,
                  i: f64, j: f64, k: f64, l: f64, m: f64| {
                let _ = &own_trapq;
                record(
                    &calls,
                    CallEntry::TrapqAppend {
                        tq: tq.clone(),
                        args: [a, b, c, d, e, f1, g, h, i, j, k, l, m],
                    },
                );
            },
        );
        append
    }
    fn wipe_trapq(&self, trapq: &Trapq) {
        record(&self.calls, CallEntry::WipeTrapq(trapq.clone()));
    }
    fn allocate_syncemitter(
        &self,
        _mcu: Mcu,
        _name: &str,
        _alloc_stepcompress: bool,
    ) -> KResult<SyncEmitterHandle> {
        panic!("allocate_syncemitter is not used by force_move")
    }
    fn queue_syncemitter_msg(&self, _syncemitter: &SyncEmitterHandle, _req_clock: u64, _data: &[u32]) {
        panic!("queue_syncemitter_msg is not used by force_move")
    }
    fn setup_mcu_movequeue(&self, _mcu: Mcu, _serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _move_count: i64) {
        panic!("setup_mcu_movequeue is not used by force_move")
    }
    fn register_flush_callback(&self, _callback: FlushCallback, _can_add_trapq: bool) {
        panic!("register_flush_callback is not used by force_move")
    }
    fn unregister_flush_callback(&self, _callback: &FlushCallback) {
        panic!("unregister_flush_callback is not used by force_move")
    }
    fn register_flush_callback_named(&self, _: FlushCallback, _: bool, _: Option<&str>, _: Option<&str>) {
        panic!("register_flush_callback_named is not used")
    }
    fn get_kin_flush_delay(&self) -> f64 {
        panic!("get_kin_flush_delay is not used by force_move")
    }
    fn check_step_generation_scan_windows(&self) {
        panic!("check_step_generation_scan_windows is not used by force_move")
    }
    fn flush_all_steps(&self) -> KResult<()> {
        panic!("flush_all_steps is not used by force_move")
    }
    fn calc_step_gen_restart(&self, _est_print_time: f64) -> f64 {
        panic!("calc_step_gen_restart is not used by force_move")
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        record(&self.calls, CallEntry::NoteMcuMovequeueActivity(mq_time, is_step_gen));
    }
    fn drip_update_time(
        &self,
        _start_time: f64,
        _end_time: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_update_time is not used by force_move")
    }
    fn check_drip_timing(&self) -> Option<f64> {
        panic!("check_drip_timing is not used by force_move")
    }
}

klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

// --- FakeToolhead ---

struct FakeKinematics {
    calls: CallLog,
}

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        panic!("get_steppers is not used by force_move")
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        panic!("calc_position is not used by force_move")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("kinematics set_position is not used by force_move")
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        panic!("check_move is not used by force_move")
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("home is not used by force_move")
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        panic!("kinematics get_status is not used by force_move")
    }
    fn clear_homing_state(&self, clear_axes: &str) -> klippy_api::KResult<()> {
        record(&self.calls, CallEntry::ClearHomingState(clear_axes.to_string()));
        Ok(())
    }
}

struct FakeToolhead {
    position: Mutex<Vec<f64>>,
    last_move_time: f64,
    calls: CallLog,
    kinematics: Arc<FakeKinematics>,
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.lock().expect("position lock").clone()
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        record(
            &self.calls,
            CallEntry::SetPosition([newpos[0], newpos[1], newpos[2]], homing_axes.to_string()),
        );
        *self.position.lock().expect("position lock") = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        self.kinematics.clone() as Kinematics
    }
    fn flush_step_generation(&self) -> KResult<()> {
        record(&self.calls, CallEntry::FlushStepGeneration);
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(self.last_move_time)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        record(&self.calls, CallEntry::Dwell(delay));
        Ok(())
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by force_move")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by force_move")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by force_move")
    }
    fn wait_moves(&self) -> KResult<()> {
        panic!("wait_moves is not used by force_move")
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        panic!("toolhead get_status is not used by force_move")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by force_move")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by force_move")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by force_move")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by force_move")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by force_move")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by force_move")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by force_move")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by force_move")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by force_move")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by force_move")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by force_move")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("toolhead stats is not used by force_move")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by force_move")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

// --- FakeStepper ---

struct FakeStepper {
    name: String,
    radians: bool,
    calls: CallLog,
    prev_sk: Mutex<Option<StepperKinematics>>,
    prev_tq: Mutex<Option<Trapq>>,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by force_move")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by force_move")
    }
    fn units_in_radians(&self) -> bool {
        self.radians
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by force_move")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration is not used by force_move")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by force_move")
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist is not used by force_move")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by force_move")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance is not used by force_move")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by force_move")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by force_move")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by force_move")
    }
    fn set_position(&self, coord: [f64; 3]) {
        record(&self.calls, CallEntry::StepperSetPosition(coord));
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by force_move")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        panic!("get_mcu_position is not used by force_move")
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        panic!("get_past_mcu_position is not used by force_move")
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: f64) -> f64 {
        panic!("mcu_to_commanded_position is not used by force_move")
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        panic!("dump_steps is not used by force_move")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics is not used by force_move")
    }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        record(&self.calls, CallEntry::SetStepperKinematics(sk.clone()));
        let mut prev = self.prev_sk.lock().expect("prev_sk lock");
        match sk {
            Some(new) => prev.replace(new),
            None => prev.take(),
        }
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by force_move")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq is not used by force_move")
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        record(&self.calls, CallEntry::SetTrapq(tq.clone()));
        let mut prev = self.prev_tq.lock().expect("prev_tq lock");
        match tq {
            Some(new) => prev.replace(new),
            None => prev.take(),
        }
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        panic!("add_active_callback is not used by force_move")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by force_move")
    }
}

// --- FakeStepperEnable ---

struct FakeStepperEnable {
    calls: CallLog,
    did_enable: bool,
}

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        panic!("register_stepper is not used by force_move")
    }
    fn set_motors_enable(&self, stepper_names: &[String], enable: bool) -> KResult<bool> {
        record(&self.calls, CallEntry::SetMotorsEnable(stepper_names.to_vec(), enable));
        Ok(self.did_enable)
    }
    fn motor_off(&self) -> KResult<()> {
        panic!("motor_off is not used by force_move")
    }
    fn lookup_enable(&self, _name: &str) -> KResult<Arc<EnableTracking>> {
        panic!("lookup_enable is not used by force_move")
    }
    fn get_steppers(&self) -> Vec<String> {
        panic!("get_steppers is not used by force_move")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by force_move")
    }
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

// --- FakeGcode ---

struct FakeGcode {
    calls: CallLog,
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        record(&self.calls, CallEntry::RegisterCommand(cmd.to_string(), opts.desc.clone()));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        record(
            &self.calls,
            CallEntry::RegisterMuxCommand(
                cmd.to_string(),
                key.to_string(),
                value.map(str::to_string),
                desc.map(str::to_string),
            ),
        );
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("gcode get_status is not used by force_move")
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {
        panic!("register_output_handler is not used by force_move")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by force_move")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by force_move")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by force_move")
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by force_move")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by force_move")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by force_move")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used by force_move")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by force_move")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by force_move")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by force_move")
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

// --- FakePrinter ---

#[derive(Clone)]
struct FakePrinter {
    objects: Arc<Mutex<IndexMap<String, ObjectRef>>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter { objects: Arc::new(Mutex::new(IndexMap::new())) }
    }
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.objects.lock().expect("objects lock");
        if objs.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().expect("objects lock").get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unknown config object '{name}'")))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().expect("objects lock").get(section).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::config(format!("Unable to load module '{section}'")))
            }
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
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

// --- FakeConfig ---

fn parse_py_bool(raw: &str) -> Option<bool> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "true" | "1" | "yes" | "on" => Some(true),
        "false" | "0" | "no" | "off" => Some(false),
        _ => None,
    }
}

struct FakeConfig {
    printer: Printer,
    options: IndexMap<String, String>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        "force_move".to_string()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get is not used by force_move")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by force_move")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by force_move")
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.options.get(option) {
            Some(raw) => parse_py_bool(raw).map(Some).ok_or_else(|| {
                KlippyError::config(format!(
                    "Unable to parse option '{option}' in section 'force_move'"
                ))
            }),
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section 'force_move' must be specified"
                ))),
                Some(v) => Ok(v),
            },
        }
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by force_move")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by force_move")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by force_move")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by force_move")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by force_move")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by force_move")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by force_move")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by force_move")
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        panic!("get_prefix_options is not used by force_move")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> {
        panic!("deprecate is not used by force_move")
    }
}

// --- FakeGcmd ---

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
}

impl FakeGcmd {
    fn new(command: &str, commandline: &str, params: &[(&str, &str)]) -> GCmd {
        let mut map = IndexMap::new();
        for (k, v) in params {
            map.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: map,
        }) as GCmd
    }
}

fn cmd_missing(commandline: &str, name: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': missing {name}"))
}

fn cmd_unable(commandline: &str, value: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': unable to parse {value}"))
}

fn cmd_min(commandline: &str, name: &str, bound: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': {name} must have minimum of {bound}"))
}

fn cmd_max(commandline: &str, name: &str, bound: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': {name} must have maximum of {bound}"))
}

fn cmd_above(commandline: &str, name: &str, bound: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': {name} must be above {bound}"))
}

fn cmd_below(commandline: &str, name: &str, bound: &str) -> KlippyError {
    KlippyError::command(format!("Error on '{commandline}': {name} must be below {bound}"))
}

fn bound_str(v: &PyValue) -> String {
    v.str()
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let raw = match self.params.get(name) {
            Some(raw) => raw.clone(),
            None => {
                return match opts.default {
                    None => Err(cmd_missing(&self.commandline, name)),
                    Some(v) => Ok(v),
                }
            }
        };
        let value = match opts.parser {
            GetParser::Str => PyValue::Str(raw),
            GetParser::Int => match pycompat::text::int_from_string(&raw, 10) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(cmd_unable(&self.commandline, &raw)),
            },
            GetParser::IntAutoBase => match pycompat::text::int_from_string(&raw, 0) {
                Ok(n) => PyValue::Int(n),
                Err(_) => return Err(cmd_unable(&self.commandline, &raw)),
            },
            GetParser::Float => match float_from_string(&raw) {
                Ok(v) => PyValue::Float(v),
                Err(_) => return Err(cmd_unable(&self.commandline, &raw)),
            },
        };
        let vf = value.to_f64();
        if let Some(minv) = &opts.minval {
            if let (Some(v), Some(m)) = (vf, minv.to_f64()) {
                if v < m {
                    return Err(cmd_min(&self.commandline, name, &bound_str(minv)));
                }
            }
        }
        if let Some(maxv) = &opts.maxval {
            if let (Some(v), Some(m)) = (vf, maxv.to_f64()) {
                if v > m {
                    return Err(cmd_max(&self.commandline, name, &bound_str(maxv)));
                }
            }
        }
        if let Some(above) = &opts.above {
            if let (Some(v), Some(m)) = (vf, above.to_f64()) {
                if v <= m {
                    return Err(cmd_above(&self.commandline, name, &bound_str(above)));
                }
            }
        }
        if let Some(below) = &opts.below {
            if let (Some(v), Some(m)) = (vf, below.to_f64()) {
                if v >= m {
                    return Err(cmd_below(&self.commandline, name, &bound_str(below)));
                }
            }
        }
        Ok(value)
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int is not used by force_move")
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            Some(raw) => raw.clone(),
            None => {
                return match opts.default {
                    None => Err(cmd_missing(&self.commandline, name)),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = match float_from_string(&raw) {
            Ok(v) => v,
            Err(_) => return Err(cmd_unable(&self.commandline, &raw)),
        };
        if let Some(m) = opts.minval {
            if v < m {
                return Err(cmd_min(&self.commandline, name, &float_repr(m)));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(cmd_max(&self.commandline, name, &float_repr(m)));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(cmd_above(&self.commandline, name, &float_repr(m)));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(cmd_below(&self.commandline, name, &float_repr(m)));
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
        String::new()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used by force_move")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by force_move")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("ack is not used by force_move")
    }
}

// ---------------------------------------------------------------------------
// World builders and comparison plumbing.
// ---------------------------------------------------------------------------

struct World {
    fm: Arc<ForceMove>,
    calls: CallLog,
    orig_sk: StepperKinematics,
    orig_tq: Trapq,
}

fn make_world_rust(
    enable: Option<&str>,
    position: Vec<f64>,
    last_time: f64,
    did_enable: bool,
    radians: bool,
) -> (World, Config) {
    let calls: CallLog = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter::new());
    let mut options = IndexMap::new();
    if let Some(v) = enable {
        options.insert("enable_force_move".to_string(), v.to_string());
    }
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() as Printer, options });
    let trapq = Arc::new(FakeTrapq) as Trapq;
    let mq: Arc<FakeMotionQueuing> = Arc::new(FakeMotionQueuing { calls: calls.clone(), trapq });
    let th: Arc<FakeToolhead> = Arc::new(FakeToolhead {
        position: Mutex::new(position),
        last_move_time: last_time,
        calls: calls.clone(),
        kinematics: Arc::new(FakeKinematics { calls: calls.clone() }),
    });
    let gc: Arc<FakeGcode> = Arc::new(FakeGcode { calls: calls.clone() });
    let se: Arc<FakeStepperEnable> = Arc::new(FakeStepperEnable { calls: calls.clone(), did_enable });
    let orig_sk = Arc::new(FakeSk) as StepperKinematics;
    let orig_tq = Arc::new(FakeTrapq) as Trapq;
    let stepper: Stepper = Arc::new(FakeStepper {
        name: "stepper_x".to_string(),
        radians,
        calls: calls.clone(),
        prev_sk: Mutex::new(Some(orig_sk.clone())),
        prev_tq: Mutex::new(Some(orig_tq.clone())),
    });
    printer.add_object("motion_queuing", mq as ObjectRef).expect("add motion_queuing");
    printer.add_object("toolhead", th as ObjectRef).expect("add toolhead");
    printer.add_object("gcode", gc as ObjectRef).expect("add gcode");
    printer.add_object("stepper_enable", se as ObjectRef).expect("add stepper_enable");
    let fm = ForceMove::new(&config).expect("ForceMove::new");
    fm.register_stepper(&config, stepper).expect("register_stepper");
    (World { fm, calls, orig_sk, orig_tq }, config)
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("force_move fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping force_move oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "force_move".to_string(),
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

fn check_calls(
    py: &mut pyoracle::Oracle,
    label: &str,
    world: &World,
) {
    let expected = match py.eval("PY_CALLS") {
        Ok(v) => v,
        Err(e) => panic!("{label}: eval PY_CALLS raised {}: {}", e.class_name, e.message),
    };
    let calls = world.calls.lock().expect("call log lock");
    let actual = calls_value(&calls, &world.fm, &world.orig_sk, &world.orig_tq);
    if let Err(d) = assert_equal(&record_for(label), "calls", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_logs(py: &mut pyoracle::Oracle, label: &str) {
    let expected = match py.eval("PY_LOGS") {
        Ok(v) => v,
        Err(e) => panic!("{label}: eval PY_LOGS raised {}: {}", e.class_name, e.message),
    };
    let actual = Value::List(take_logs().into_iter().map(|l| s(&l)).collect());
    if let Err(d) = assert_equal(&record_for(label), "logs", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_ok(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    match expected {
        Ok(_) => {
            if let Err(e) = actual {
                panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
            }
        }
        Err(e) => panic!("{label}: oracle raised {}: {} but rust call must succeed", e.class_name, e.message),
    }
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KResult<()>) {
    let oracle_exc = match expected {
        Ok(_) => panic!("{label}: oracle returned Ok but rust call must fail"),
        Err(e) => e,
    };
    let rust_err = match actual {
        Ok(()) => panic!("{label}: rust returned Ok but call must fail"),
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

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    check("BUZZ_DISTANCE", py.eval("force_move.BUZZ_DISTANCE"), f(BUZZ_DISTANCE));
    check("BUZZ_VELOCITY", py.eval("force_move.BUZZ_VELOCITY"), f(BUZZ_VELOCITY));
    check("BUZZ_RADIANS_DISTANCE", py.eval("force_move.BUZZ_RADIANS_DISTANCE"), f(BUZZ_RADIANS_DISTANCE));
    check("BUZZ_RADIANS_VELOCITY", py.eval("force_move.BUZZ_RADIANS_VELOCITY"), f(BUZZ_RADIANS_VELOCITY));
    check("help BUZZ", py.eval("force_move.ForceMove.cmd_STEPPER_BUZZ_help"), s(CMD_STEPPER_BUZZ_HELP));
    check("help MOVE", py.eval("force_move.ForceMove.cmd_FORCE_MOVE_help"), s(CMD_FORCE_MOVE_HELP));
    check("help SET", py.eval("force_move.ForceMove.cmd_SET_KINEMATIC_POSITION_help"), s(CMD_SET_KINEMATIC_POSITION_HELP));
}

#[test]
fn calc_move_time_matches() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    let cases: &[(f64, f64, f64)] = &[
        (1.0, 4.0, 0.0),
        (-1.0, 4.0, 0.0),
        (2.0, 5.0, 100.0),
        (-2.0, 5.0, 100.0),
        (0.0, 5.0, 100.0),
        (0.0, 5.0, 0.0),
        (10.0, 50.0, 1000.0),
        (0.5, 1e-9, 1e9),
        (4.25, 2.7, 400.0),
        (1.0, 100.0, 1.0),
        (1.0, 1.0, -5.0),
        (5.0, 0.0, 0.0),
        (5.0, 0.0, 10.0),
        (0.0, 0.0, 0.0),
        (-0.0, 2.0, 3.0),
        (1.0, 2.0, 0.0),
        (123.456, 7.89, 0.01),
    ];
    for (dist, speed, accel) in cases {
        let label = format!("calc_move_time({dist:?}, {speed:?}, {accel:?})");
        let expected = py.eval(&format!("force_move.calc_move_time({dist:?}, {speed:?}, {accel:?})"));
        match expected {
            Ok(v) => {
                let (axis_r, accel_t, cruise_t, cruise_v) = calc_move_time(*dist, *speed, *accel)
                    .unwrap_or_else(|e| panic!("{label}: rust raised {}: {}", e.class_name(), e.message()));
                let actual = Value::Tuple(vec![f(axis_r), f(accel_t), f(cruise_t), f(cruise_v)]);
                check(&label, Ok(v), actual);
            }
            Err(e) => {
                let actual = calc_move_time(*dist, *speed, *accel);
                let rust_err = match actual {
                    Ok(t) => panic!("{label}: rust returned {t:?} but call must fail"),
                    Err(e2) => e2,
                };
                assert_eq!(rust_err.class_name(), e.class_name.as_str(), "{label}: class");
                assert_eq!(rust_err.message(), e.message.as_str(), "{label}: message");
            }
        }
    }
}

#[test]
fn init_defaults() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    assert!(!world.fm.enable_force_move());
    check_calls(&mut py, "init defaults calls", &world);
    check_logs(&mut py, "init defaults logs");
    let names_rs = Value::List(world.fm.stepper_names().iter().map(|n| s(n)).collect());
    check("stepper names", py.eval("list(fm.steppers.keys())"), names_rs);
}

#[test]
fn init_enable_force_move() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world(enable='true')").expect("py make_world");
    let (world, _config) = make_world_rust(Some("true"), vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    assert!(world.fm.enable_force_move());
    check_calls(&mut py, "init enabled calls", &world);
    check_logs(&mut py, "init enabled logs");
}

#[test]
fn init_invalid_enable_value() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    let expected = py.exec(
        "PY_CALLS.clear(); PY_LOGS.clear(); fm = force_move.ForceMove(FakeConfig(FakePrinter(mq=FakeMotionQueuing(), toolhead=FakeToolhead(), gcode=FakeGcode(), stepper_enable=FakeStepperEnable()), {'enable_force_move': 'maybe'}))",
    );
    // Rust side: same invalid option value.
    let calls: CallLog = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter::new());
    let mut options = IndexMap::new();
    options.insert("enable_force_move".to_string(), "maybe".to_string());
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() as Printer, options });
    let trapq = Arc::new(FakeTrapq) as Trapq;
    let mq: Arc<FakeMotionQueuing> = Arc::new(FakeMotionQueuing { calls: calls.clone(), trapq });
    let th: Arc<FakeToolhead> = Arc::new(FakeToolhead {
        position: Mutex::new(vec![10.0, 20.0, 30.0, 0.0]),
        last_move_time: 100.0,
        calls: calls.clone(),
        kinematics: Arc::new(FakeKinematics { calls: calls.clone() }),
    });
    let gc: Arc<FakeGcode> = Arc::new(FakeGcode { calls: calls.clone() });
    let se: Arc<FakeStepperEnable> = Arc::new(FakeStepperEnable { calls: calls.clone(), did_enable: true });
    printer.add_object("motion_queuing", mq as ObjectRef).expect("add mq");
    printer.add_object("toolhead", th as ObjectRef).expect("add th");
    printer.add_object("gcode", gc as ObjectRef).expect("add gc");
    printer.add_object("stepper_enable", se as ObjectRef).expect("add se");
    let actual = ForceMove::new(&config).map(|_| ());
    check_err("invalid enable_force_move", expected, actual);
}

#[test]
fn init_missing_gcode() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    let expected = py.exec(
        "PY_CALLS.clear(); PY_LOGS.clear(); fm = force_move.ForceMove(FakeConfig(FakePrinter(mq=FakeMotionQueuing(), toolhead=FakeToolhead(), gcode=None, stepper_enable=FakeStepperEnable()), {'enable_force_move': 'true'}))",
    );
    let calls: CallLog = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter::new());
    let mut options = IndexMap::new();
    options.insert("enable_force_move".to_string(), "true".to_string());
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() as Printer, options });
    let trapq = Arc::new(FakeTrapq) as Trapq;
    let mq: Arc<FakeMotionQueuing> = Arc::new(FakeMotionQueuing { calls: calls.clone(), trapq });
    let th: Arc<FakeToolhead> = Arc::new(FakeToolhead {
        position: Mutex::new(vec![10.0, 20.0, 30.0, 0.0]),
        last_move_time: 100.0,
        calls: calls.clone(),
        kinematics: Arc::new(FakeKinematics { calls: calls.clone() }),
    });
    let se: Arc<FakeStepperEnable> = Arc::new(FakeStepperEnable { calls: calls.clone(), did_enable: true });
    printer.add_object("motion_queuing", mq as ObjectRef).expect("add mq");
    printer.add_object("toolhead", th as ObjectRef).expect("add th");
    printer.add_object("stepper_enable", se as ObjectRef).expect("add se");
    let actual = ForceMove::new(&config).map(|_| ());
    check_err("missing gcode object", expected, actual);
}

#[test]
fn lookup_stepper_unknown() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    let expected = py.eval("fm.lookup_stepper('nope')");
    check_err("lookup unknown stepper", expected, world.fm.lookup_stepper("nope").map(|_| ()));
}

#[test]
fn force_move_api_lookup_stepper() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    // `angle.py` reaches force_move through `lookup_object('force_move')`, and
    // its `stepper_name` may be None.
    let obj: ObjectRef = world.fm.clone();
    let api = klippy_api::interface::<dyn ForceMoveApi>(&obj).expect("force_move declares ForceMoveApi");
    check_err("lookup None", py.eval("fm.lookup_stepper(None)"), api.lookup_stepper(None).map(|_| ()));
    check_err("lookup unknown", py.eval("fm.lookup_stepper('nope')"), api.lookup_stepper(Some("nope")).map(|_| ()));
    assert!(api.lookup_stepper(Some("stepper_x")).is_ok(), "registered stepper resolves through ForceMoveApi");
}

#[test]
fn manual_move_matches() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    for (dist, speed, accel) in [(2.0, 5.0, 100.0), (-3.0, 10.0, 0.0), (1.0, 4.0, 0.0)] {
        py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
        let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
        world.calls.lock().expect("call log lock").clear();
        py.exec("PY_CALLS.clear()").expect("clear py calls");
        let label = format!("manual_move({dist:?}, {speed:?}, {accel:?})");
        let expected = py.eval(&format!("fm.manual_move(stepper, {dist:?}, {speed:?}, {accel:?})"));
        check_ok(&label, expected, world.fm.manual_move(
            &world
                .fm
                .lookup_stepper("stepper_x")
                .expect("lookup stepper_x"),
            dist,
            speed,
            accel,
        ));
        check_calls(&mut py, &label, &world);
        check_logs(&mut py, &label);
    }
}

#[test]
fn manual_move_zero_speed_errors() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let expected = py.eval("fm.manual_move(stepper, 1.0, 0.0, 0.0)");
    check_err(
        "manual_move zero speed",
        expected,
        world
            .fm
            .manual_move(&world.fm.lookup_stepper("stepper_x").expect("lookup"), 1.0, 0.0, 0.0),
    );
    check_calls(&mut py, "manual_move zero speed calls", &world);
}

#[test]
fn stepper_buzz_linear() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let gcmd_py = "FakeGcmd('STEPPER_BUZZ', 'STEPPER_BUZZ STEPPER=stepper_x', {'STEPPER': 'stepper_x'})";
    let expected = py.eval(&format!("fm.cmd_STEPPER_BUZZ({gcmd_py})"));
    let gcmd_rs = FakeGcmd::new("STEPPER_BUZZ", "STEPPER_BUZZ STEPPER=stepper_x", &[("STEPPER", "stepper_x")]);
    check_ok("STEPPER_BUZZ", expected, world.fm.cmd_stepper_buzz(&gcmd_rs));
    check_calls(&mut py, "STEPPER_BUZZ calls", &world);
    check_logs(&mut py, "STEPPER_BUZZ logs");
}

#[test]
fn stepper_buzz_radians_no_restore() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world(did_enable=False, radians=True)").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, false, true);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let expected = py.eval(
        "fm.cmd_STEPPER_BUZZ(FakeGcmd('STEPPER_BUZZ', 'STEPPER_BUZZ STEPPER=stepper_x', {'STEPPER': 'stepper_x'}))",
    );
    let gcmd_rs = FakeGcmd::new("STEPPER_BUZZ", "STEPPER_BUZZ STEPPER=stepper_x", &[("STEPPER", "stepper_x")]);
    check_ok("STEPPER_BUZZ radians", expected, world.fm.cmd_stepper_buzz(&gcmd_rs));
    check_calls(&mut py, "STEPPER_BUZZ radians calls", &world);
    check_logs(&mut py, "STEPPER_BUZZ radians logs");
}

#[test]
fn force_move_full() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world(enable='true')").expect("py make_world");
    let (world, _config) = make_world_rust(Some("true"), vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let expected = py.eval(
        "fm.cmd_FORCE_MOVE(FakeGcmd('FORCE_MOVE', 'FORCE_MOVE STEPPER=stepper_x DISTANCE=5.5 VELOCITY=3.25 ACCEL=900.0', {'STEPPER': 'stepper_x', 'DISTANCE': '5.5', 'VELOCITY': '3.25', 'ACCEL': '900.0'}))",
    );
    let gcmd_rs = FakeGcmd::new(
        "FORCE_MOVE",
        "FORCE_MOVE STEPPER=stepper_x DISTANCE=5.5 VELOCITY=3.25 ACCEL=900.0",
        &[("STEPPER", "stepper_x"), ("DISTANCE", "5.5"), ("VELOCITY", "3.25"), ("ACCEL", "900.0")],
    );
    check_ok("FORCE_MOVE", expected, world.fm.cmd_force_move(&gcmd_rs));
    check_calls(&mut py, "FORCE_MOVE calls", &world);
    check_logs(&mut py, "FORCE_MOVE logs");
}

#[test]
fn force_move_default_accel() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let expected = py.eval(
        "fm.cmd_FORCE_MOVE(FakeGcmd('FORCE_MOVE', 'FORCE_MOVE STEPPER=stepper_x DISTANCE=-2.0 VELOCITY=1.5', {'STEPPER': 'stepper_x', 'DISTANCE': '-2.0', 'VELOCITY': '1.5'}))",
    );
    let gcmd_rs = FakeGcmd::new(
        "FORCE_MOVE",
        "FORCE_MOVE STEPPER=stepper_x DISTANCE=-2.0 VELOCITY=1.5",
        &[("STEPPER", "stepper_x"), ("DISTANCE", "-2.0"), ("VELOCITY", "1.5")],
    );
    check_ok("FORCE_MOVE default accel", expected, world.fm.cmd_force_move(&gcmd_rs));
    check_calls(&mut py, "FORCE_MOVE default accel calls", &world);
    check_logs(&mut py, "FORCE_MOVE default accel logs");
}

#[test]
fn force_move_errors() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    // Unknown stepper wins over missing DISTANCE (lookup happens first).
    py.exec("fm, mq, th, gc, se, stepper = make_world()").expect("py make_world");
    let (world, _config) = make_world_rust(None, vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    let err_cases: &[(&str, &str, &[(&str, &str)])] = &[
        (
            "FORCE_MOVE unknown stepper",
            "FORCE_MOVE STEPPER=nope DISTANCE=1.0 VELOCITY=1.0",
            &[("STEPPER", "nope"), ("DISTANCE", "1.0"), ("VELOCITY", "1.0")],
        ),
        (
            "FORCE_MOVE missing DISTANCE",
            "FORCE_MOVE STEPPER=stepper_x VELOCITY=1.0",
            &[("STEPPER", "stepper_x"), ("VELOCITY", "1.0")],
        ),
        (
            "FORCE_MOVE missing VELOCITY",
            "FORCE_MOVE STEPPER=stepper_x DISTANCE=1.0",
            &[("STEPPER", "stepper_x"), ("DISTANCE", "1.0")],
        ),
        (
            "FORCE_MOVE VELOCITY zero",
            "FORCE_MOVE STEPPER=stepper_x DISTANCE=1.0 VELOCITY=0",
            &[("STEPPER", "stepper_x"), ("DISTANCE", "1.0"), ("VELOCITY", "0")],
        ),
        (
            "FORCE_MOVE negative ACCEL",
            "FORCE_MOVE STEPPER=stepper_x DISTANCE=1.0 VELOCITY=1.0 ACCEL=-5",
            &[("STEPPER", "stepper_x"), ("DISTANCE", "1.0"), ("VELOCITY", "1.0"), ("ACCEL", "-5")],
        ),
        (
            "FORCE_MOVE bad DISTANCE",
            "FORCE_MOVE STEPPER=stepper_x DISTANCE=abc VELOCITY=1.0",
            &[("STEPPER", "stepper_x"), ("DISTANCE", "abc"), ("VELOCITY", "1.0")],
        ),
    ];
    for (label, commandline, params) in err_cases {
        let params_py: Vec<String> = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect();
        let params_literal = format!("{{{}}}", params_py.join(", "));
        let expected = py.eval(&format!(
            "fm.cmd_FORCE_MOVE(FakeGcmd('FORCE_MOVE', '{commandline}', {params_literal}))"
        ));
        let gcmd_rs = FakeGcmd::new("FORCE_MOVE", commandline, params);
        check_err(label, expected, world.fm.cmd_force_move(&gcmd_rs));
    }
}

#[test]
fn set_kinematic_position_defaults() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    py.exec("fm, mq, th, gc, se, stepper = make_world(enable='true')").expect("py make_world");
    let (world, _config) = make_world_rust(Some("true"), vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    world.calls.lock().expect("call log lock").clear();
    py.exec("PY_CALLS.clear()").expect("clear py calls");
    let expected = py.eval(
        "fm.cmd_SET_KINEMATIC_POSITION(FakeGcmd('SET_KINEMATIC_POSITION', 'SET_KINEMATIC_POSITION', {}))",
    );
    let gcmd_rs = FakeGcmd::new("SET_KINEMATIC_POSITION", "SET_KINEMATIC_POSITION", &[]);
    check_ok("SET_KINEMATIC_POSITION defaults", expected, world.fm.cmd_set_kinematic_position(&gcmd_rs));
    check_calls(&mut py, "SET defaults calls", &world);
    check_logs(&mut py, "SET defaults logs");
}

#[test]
fn set_kinematic_position_variants() {
    lock_logs();
    let Some(mut py) = oracle() else { return; };
    let cases: &[(&str, Vec<(&str, &str)>, &str)] = &[
        (
            "explicit axes",
            vec![("X", "1.5"), ("Y", "-2.25"), ("Z", "0.5"), ("SET_HOMED", "xz")],
            "SET_KINEMATIC_POSITION X=1.5 Y=-2.25 Z=0.5 SET_HOMED=xz",
        ),
        (
            "uppercase homed",
            vec![("SET_HOMED", "XYZ")],
            "SET_KINEMATIC_POSITION SET_HOMED=XYZ",
        ),
        (
            "clear homed",
            vec![("CLEAR_HOMED", "y")],
            "SET_KINEMATIC_POSITION CLEAR_HOMED=y",
        ),
        (
            "clear alias",
            vec![("CLEAR", "x")],
            "SET_KINEMATIC_POSITION CLEAR=x",
        ),
        (
            "clear homed wins",
            vec![("CLEAR", "z"), ("CLEAR_HOMED", "y")],
            "SET_KINEMATIC_POSITION CLEAR=z CLEAR_HOMED=y",
        ),
        (
            "empty clear homed",
            vec![("CLEAR_HOMED", ""), ("CLEAR", "z")],
            "SET_KINEMATIC_POSITION CLEAR_HOMED= CLEAR=z",
        ),
        (
            "no axes letters",
            vec![("SET_HOMED", "abc"), ("CLEAR_HOMED", "q")],
            "SET_KINEMATIC_POSITION SET_HOMED=abc CLEAR_HOMED=q",
        ),
    ];
    for (label, params, commandline) in cases {
        py.exec("fm, mq, th, gc, se, stepper = make_world(enable='true')").expect("py make_world");
        let (world, _config) = make_world_rust(Some("true"), vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
        world.calls.lock().expect("call log lock").clear();
        py.exec("PY_CALLS.clear()").expect("clear py calls");
        let params_py: Vec<String> = params
            .iter()
            .map(|(k, v)| format!("'{k}': '{v}'"))
            .collect();
        let params_literal = format!("{{{}}}", params_py.join(", "));
        let expected = py.eval(&format!(
            "fm.cmd_SET_KINEMATIC_POSITION(FakeGcmd('SET_KINEMATIC_POSITION', '{commandline}', {params_literal}))"
        ));
        let gcmd_rs = FakeGcmd::new("SET_KINEMATIC_POSITION", commandline, params);
        check_ok(label, expected, world.fm.cmd_set_kinematic_position(&gcmd_rs));
        check_calls(&mut py, &format!("{label} calls"), &world);
        check_logs(&mut py, &format!("{label} logs"));
    }
    // Bad X value.
    py.exec("fm, mq, th, gc, se, stepper = make_world(enable='true')").expect("py make_world");
    let (world, _config) = make_world_rust(Some("true"), vec![10.0, 20.0, 30.0, 0.0], 100.0, true, false);
    let expected = py.eval(
        "fm.cmd_SET_KINEMATIC_POSITION(FakeGcmd('SET_KINEMATIC_POSITION', 'SET_KINEMATIC_POSITION X=abc', {'X': 'abc'}))",
    );
    let gcmd_rs = FakeGcmd::new("SET_KINEMATIC_POSITION", "SET_KINEMATIC_POSITION X=abc", &[("X", "abc")]);
    check_err("SET bad X", expected, world.fm.cmd_set_kinematic_position(&gcmd_rs));
}

#[test]
fn load_config_factory_builds_working_object() {
    lock_logs();
    let Some(_py) = oracle() else { return; };
    let factory = module_factory();
    let load_config = factory.load_config.expect("force_move load_config");
    assert!(factory.load_config_prefix.is_none());
    let calls: CallLog = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter::new());
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() as Printer, options: IndexMap::new() });
    let trapq = Arc::new(FakeTrapq) as Trapq;
    let mq: Arc<FakeMotionQueuing> = Arc::new(FakeMotionQueuing { calls: calls.clone(), trapq });
    let th: Arc<FakeToolhead> = Arc::new(FakeToolhead {
        position: Mutex::new(vec![0.0, 0.0, 0.0, 0.0]),
        last_move_time: 0.0,
        calls: calls.clone(),
        kinematics: Arc::new(FakeKinematics { calls: calls.clone() }),
    });
    let gc: Arc<FakeGcode> = Arc::new(FakeGcode { calls: calls.clone() });
    let se: Arc<FakeStepperEnable> = Arc::new(FakeStepperEnable { calls: calls.clone(), did_enable: true });
    printer.add_object("motion_queuing", mq as ObjectRef).expect("add mq");
    printer.add_object("toolhead", th as ObjectRef).expect("add th");
    printer.add_object("gcode", gc as ObjectRef).expect("add gc");
    printer.add_object("stepper_enable", se as ObjectRef).expect("add se");
    let obj = load_config(&config).expect("load_config").expect("force_move object");
    let fm = klippy_api::downcast_arc::<ForceMove>(&obj).expect("downcast to ForceMove");
    assert!(!fm.enable_force_move());
    assert!(fm.stepper_names().is_empty());
}
