//! Parity tests for `extras::homing` (port of `klippy/extras/homing.py`).
//!
//! Expected values come from the live reference through `pyoracle` (CPython 3.8.2 and the
//! frozen `klippy/`): the real `extras.homing` module (`HomingMove`, `Homing`,
//! `PrinterHoming`, `multi_complete`) runs inside the real `klippy/reactor.py` on inert
//! fakes for the printer, `gcode`, `toolhead`, kinematics, rails, endstops, steppers,
//! `probe` and `stepper_enable`. The Rust port runs the same scripted scenario inside the
//! real Rust reactor on the Rust twins of those fakes. A scenario is compared through
//! everything the fakes observe: the ordered call trace (`set_position`, `move`, `dwell`,
//! `drip_move` and whether its completion fired, `home_start` / `home_wait` arguments,
//! `get_past_mcu_position` times, events with their payload as handlers read it, probe
//! session calls, `motor_off`, `G28` registration), the log lines, the returned value, the
//! raised error (class and message), the final toolhead position and every stepper's
//! commanded position and mcu offset (floats by bits).
//!
//! There are no recordings under `tests/fixtures/recordings/` for these units, so
//! replay fixtures do not apply.

use std::sync::Arc;

use extras::homing::{multi_complete, Homing, PrinterHoming};
use extras::manual_probe::ProbeResult;
use extras::probe::{PrinterProbeApi, ProbeSessionApi};
use extras::stepper_enable::{EnableTracking, PrinterStepperEnableApi};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListOpts,
    GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{Endstop, EndstopApi, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, PullHistoryStep, RailApi, Stepper, StepperFactory, StepperKinematics, Trapq,
    McuStepperApi, Rail,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingMoveApi, HomingStateApi, HomingStateHandle, HomingTarget, HomingTargetApi,
    Kinematics, KinematicsApi, LookaheadCallback, MoveApi, PrinterExtruder, ToolHeadApi,
};
use klippy_api::{EventArg, GCell, KlippyError, ObjectRef};
use pycompat::text::{float_repr, str_repr};
use pycompat::PyValue;
use reactor::{Completion, Reactor, ReactorMutex, NOW};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RecordingValue};

// ---------------------------------------------------------------------------
// Scenario description, shared by both sides.
// ---------------------------------------------------------------------------

#[derive(Clone)]
enum Wait {
    Time(f64),
    Cmd(&'static str),
    Mcu(&'static str),
}

#[derive(Clone)]
struct EndSpec {
    /// `ok` / `fail` complete after 5 ms with `False` / `True`, `never` stays pending.
    completion: &'static str,
    /// One entry per `home_wait` call; the last one repeats.
    waits: Vec<Wait>,
    /// The endstop has `get_position_endstop` (a `probe:z_virtual_endstop` wrapper).
    probe: bool,
    /// `get_steppers()` is non-empty.
    attached: bool,
}

#[derive(Clone)]
struct RailSpec {
    name: &'static str,
    steppers: Vec<&'static str>,
    step_dists: Vec<f64>,
    pmin: f64,
    pmax: f64,
    pend: f64,
    positive_dir: bool,
    speed: f64,
    retract_speed: f64,
    retract_dist: f64,
    second_speed: f64,
    end: EndSpec,
}

#[derive(Clone)]
struct Spec {
    rails: Vec<RailSpec>,
    pos: Vec<f64>,
    /// Per stepper: `get_past_mcu_position` returns the halt position minus this lag.
    lag: Vec<(&'static str, Vec<i64>)>,
    /// Per `drip_move` call: how far toward the target the steppers get.
    halt_frac: Vec<f64>,
    drip_err: Vec<Option<&'static str>>,
    shutdown: bool,
    debuginput: bool,
    /// `CommandError` raised by the `homing:homing_move_end` handler.
    end_err: Option<&'static str>,
    /// `ValueError` raised by the `homing:homing_move_end` handler.
    end_internal: bool,
    /// Adjustments set by the `homing:home_rails_end` handler.
    adjust: Vec<(&'static str, f64)>,
    /// Stepper names whose trigger position the `home_rails_end` handler reads.
    query: Vec<&'static str>,
    /// `kin.calc_position` returns `None` for this axis.
    calc_none: Option<usize>,
    /// A `probe` object exists.
    probe: bool,
    probe_z: Vec<f64>,
    probe_err: Vec<Option<&'static str>>,
    probe_empty: bool,
    motor_off_err: Option<&'static str>,
}

#[derive(Clone)]
enum Action {
    G28(Vec<(&'static str, &'static str)>),
    ManualHome {
        rail: usize,
        pos: Vec<Option<f64>>,
        speed: f64,
        probe_pos: bool,
        triggered: bool,
        check_triggered: bool,
    },
    ProbingMove { rail: usize, pos: Vec<Option<f64>>, speed: f64, check_movement: bool },
    HomeRails {
        rails: Vec<usize>,
        axes: Vec<i64>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    },
    CheckFirst { command: &'static str, attempt: Option<&'static str> },
    Multi { comps: Vec<(&'static str, f64)>, wait: f64 },
}

fn end_spec() -> EndSpec {
    EndSpec { completion: "ok", waits: vec![Wait::Time(10.5)], probe: false, attached: true }
}

#[allow(clippy::too_many_arguments)]
fn rail_spec(
    name: &'static str,
    steppers: Vec<&'static str>,
    step_dists: Vec<f64>,
    pmax: f64,
    pend: f64,
    positive_dir: bool,
    speed: f64,
    retract_dist: f64,
) -> RailSpec {
    RailSpec {
        name,
        steppers,
        step_dists,
        pmin: 0.0,
        pmax,
        pend,
        positive_dir,
        speed,
        retract_speed: 10.0,
        retract_dist,
        second_speed: speed / 2.0,
        end: end_spec(),
    }
}

fn base() -> Spec {
    Spec {
        rails: vec![
            rail_spec("stepper_x", vec!["stepper_x"], vec![0.0125], 235.0, 0.0, false, 50.0, 5.0),
            rail_spec("stepper_y", vec!["stepper_y"], vec![0.0125], 235.0, 235.0, true, 40.0, 4.0),
            rail_spec("stepper_z", vec!["stepper_z"], vec![0.0025], 250.0, 0.0, false, 8.0, 2.0),
        ],
        pos: vec![10.0, 20.0, 30.0, 4.0],
        lag: vec![("stepper_x", vec![3]), ("stepper_y", vec![0, -2]), ("stepper_z", vec![1])],
        halt_frac: vec![0.5, 0.75],
        drip_err: vec![None],
        shutdown: false,
        debuginput: false,
        end_err: None,
        end_internal: false,
        adjust: Vec::new(),
        query: Vec::new(),
        calc_none: None,
        probe: true,
        probe_z: vec![1.25, 0.75],
        probe_err: vec![None],
        probe_empty: false,
        motor_off_err: None,
    }
}

/// `base()` with a dual-stepper Z rail (two step sizes behind one endstop).
fn dual_z() -> Spec {
    let mut spec = base();
    spec.rails[2] = rail_spec(
        "stepper_z",
        vec!["stepper_z", "stepper_z1"],
        vec![0.0025, 0.005],
        250.0,
        0.0,
        false,
        8.0,
        2.0,
    );
    spec.lag.push(("stepper_z1", vec![2, 0]));
    spec
}

/// `base()` with a probe endstop on Z (`probe:z_virtual_endstop`).
fn probe_z() -> Spec {
    let mut spec = base();
    spec.rails[2].end.probe = true;
    spec
}

fn seq<T: Clone>(items: &[T], idx: usize) -> T {
    items[idx.min(items.len() - 1)].clone()
}

// ---------------------------------------------------------------------------
// Python literals for the scenario.
// ---------------------------------------------------------------------------

fn py_s(s: &str) -> String {
    format!("{s:?}")
}

fn py_f(x: f64) -> String {
    format!("{x:?}")
}

fn py_b(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_opt_s(o: Option<&str>) -> String {
    match o {
        Some(s) => py_s(s),
        None => "None".to_string(),
    }
}

fn py_opt_f(o: Option<f64>) -> String {
    match o {
        Some(v) => py_f(v),
        None => "None".to_string(),
    }
}

fn py_list<T>(items: &[T], f: impl Fn(&T) -> String) -> String {
    format!("[{}]", items.iter().map(f).collect::<Vec<_>>().join(", "))
}

fn py_wait(w: &Wait) -> String {
    match w {
        Wait::Time(t) => format!("('t', {})", py_f(*t)),
        Wait::Cmd(m) => format!("('cmd', {})", py_s(m)),
        Wait::Mcu(m) => format!("('mcu', {})", py_s(m)),
    }
}

fn py_rail(r: &RailSpec) -> String {
    format!(
        "{{'name': {}, 'steppers': {}, 'step_dists': {}, 'pmin': {}, 'pmax': {}, 'pend': {}, \
         'positive_dir': {}, 'speed': {}, 'retract_speed': {}, 'retract_dist': {}, \
         'second_speed': {}, 'end': {{'completion': {}, 'waits': {}, 'probe': {}, \
         'attached': {}}}}}",
        py_s(r.name),
        py_list(&r.steppers, |s| py_s(s)),
        py_list(&r.step_dists, |v| py_f(*v)),
        py_f(r.pmin),
        py_f(r.pmax),
        py_f(r.pend),
        py_b(r.positive_dir),
        py_f(r.speed),
        py_f(r.retract_speed),
        py_f(r.retract_dist),
        py_f(r.second_speed),
        py_s(r.end.completion),
        py_list(&r.end.waits, py_wait),
        py_b(r.end.probe),
        py_b(r.end.attached),
    )
}

fn py_spec(s: &Spec) -> String {
    let lag: Vec<String> = s
        .lag
        .iter()
        .map(|(n, v)| format!("{}: {}", py_s(n), py_list(v, |x| x.to_string())))
        .collect();
    format!(
        "{{'rails': {}, 'pos': {}, 'lag': {{{}}}, 'halt_frac': {}, 'drip_err': {}, \
         'shutdown': {}, 'debuginput': {}, 'end_err': {}, 'end_internal': {}, 'adjust': {}, \
         'query': {}, 'calc_none': {}, 'probe': {}, 'probe_z': {}, 'probe_err': {}, \
         'probe_empty': {}, 'motor_off_err': {}}}",
        py_list(&s.rails, py_rail),
        py_list(&s.pos, |v| py_f(*v)),
        lag.join(", "),
        py_list(&s.halt_frac, |v| py_f(*v)),
        py_list(&s.drip_err, |e| py_opt_s(*e)),
        py_b(s.shutdown),
        py_b(s.debuginput),
        py_opt_s(s.end_err),
        py_b(s.end_internal),
        py_list(&s.adjust, |(n, a)| format!("({}, {})", py_s(n), py_f(*a))),
        py_list(&s.query, |n| py_s(n)),
        match s.calc_none {
            Some(i) => i.to_string(),
            None => "None".to_string(),
        },
        py_b(s.probe),
        py_list(&s.probe_z, |v| py_f(*v)),
        py_list(&s.probe_err, |e| py_opt_s(*e)),
        py_b(s.probe_empty),
        py_opt_s(s.motor_off_err),
    )
}

fn py_action(a: &Action) -> String {
    match a {
        Action::G28(params) => format!(
            "{{'kind': 'g28', 'params': {}}}",
            py_list(params, |(k, v)| format!("({}, {})", py_s(k), py_s(v)))
        ),
        Action::ManualHome { rail, pos, speed, probe_pos, triggered, check_triggered } => format!(
            "{{'kind': 'manual_home', 'rail': {}, 'pos': {}, 'speed': {}, 'probe_pos': {}, \
             'triggered': {}, 'check_triggered': {}}}",
            rail,
            py_list(pos, |v| py_opt_f(*v)),
            py_f(*speed),
            py_b(*probe_pos),
            py_b(*triggered),
            py_b(*check_triggered)
        ),
        Action::ProbingMove { rail, pos, speed, check_movement } => format!(
            "{{'kind': 'probing_move', 'rail': {}, 'pos': {}, 'speed': {}, \
             'check_movement': {}}}",
            rail,
            py_list(pos, |v| py_opt_f(*v)),
            py_f(*speed),
            py_b(*check_movement)
        ),
        Action::HomeRails { rails, axes, forcepos, movepos } => format!(
            "{{'kind': 'home_rails', 'rails': {}, 'axes': {}, 'forcepos': {}, 'movepos': {}}}",
            py_list(rails, |r| r.to_string()),
            py_list(axes, |r| r.to_string()),
            py_list(forcepos, |v| py_opt_f(*v)),
            py_list(movepos, |v| py_opt_f(*v))
        ),
        Action::CheckFirst { command, attempt } => format!(
            "{{'kind': 'check_first', 'command': {}, 'attempt': {}}}",
            py_s(command),
            py_opt_s(*attempt)
        ),
        Action::Multi { comps, wait } => format!(
            "{{'kind': 'multi', 'comps': {}, 'wait': {}}}",
            py_list(comps, |(k, d)| format!("({}, {})", py_s(k), py_f(*d))),
            py_f(*wait)
        ),
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import collections, logging, configparser
import reactor as reactor_mod
import extras.homing as homing

class CommandError(Exception):
    pass

# `printer.command_error` is `gcode.CommandError` (`klippy/klippy.py:27`).
CommandError.__module__ = "gcode"

class error(Exception):
    pass

_sentinel = object()
LOG = []

class _Handler(logging.Handler):
    def emit(self, record):
        LOG.append("%s:%s" % (record.levelname, record.getMessage()))

logging.getLogger().addHandler(_Handler())
logging.getLogger().setLevel(logging.DEBUG)

HomingInfo = collections.namedtuple('homing_info', [
    'speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir',
    'second_homing_speed'])

def fl(x):
    return repr(float(x))

def fll(v):
    return "[" + ", ".join(fl(x) for x in v) + "]"

def seq(lst, i):
    return lst[min(i, len(lst) - 1)]

def enames(es):
    return [[s.get_name() for s in e.get_steppers()] for e in es]

class World:
    def __init__(self, S, r):
        self.S = S
        self.r = r
        self.trace = []
        self.counts = {}
        self.steppers = []
        self.rails = []
        self.kin = None
        self.toolhead = None
        self.gcode = None
        self.printer = None
    def next(self, key):
        n = self.counts.get(key, 0)
        self.counts[key] = n + 1
        return n
    def t(self, msg):
        self.trace.append(msg)

class FakeStepper:
    def __init__(self, W, name, axis, sd):
        self.W = W
        self.name = name
        self.axis = axis
        self.sd = sd
        self.cmd = 0.0
        self.off = 0.0
    def get_name(self, short=False):
        return self.name
    def get_step_dist(self):
        return self.sd
    def get_commanded_position(self):
        return self.cmd
    def get_mcu_position(self, cmd_pos=None):
        if cmd_pos is None:
            cmd_pos = self.get_commanded_position()
        mcu_pos_dist = cmd_pos + self.off
        mcu_pos = mcu_pos_dist / self.sd
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    def set_position(self, coord):
        mcu_pos = self.get_mcu_position()
        self.cmd = coord[self.axis]
        mcu_pos_dist = mcu_pos * self.sd
        self.off = mcu_pos_dist - self.get_commanded_position()
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self.sd - self.off
    def calc_position_from_coord(self, coord):
        return float(coord[self.axis])
    def get_past_mcu_position(self, print_time):
        k = self.W.next('past:' + self.name)
        self.W.t("past %s %s" % (self.name, fl(print_time)))
        lags = self.W.S['lag'].get(self.name)
        lag = seq(lags, k) if lags else 0
        return self.get_mcu_position() - lag

class FakeEndstop:
    def __init__(self, W, name, steppers, spec):
        self.W = W
        self.name = name
        self.steppers = steppers
        self.spec = spec
    def get_steppers(self):
        return list(self.steppers)
    def home_start(self, print_time, sample_time, sample_count, rest_time, triggered=True):
        W = self.W
        W.t("home_start %s %s %s %s %s %r" % (self.name, fl(print_time), fl(sample_time),
                                            sample_count, fl(rest_time), triggered))
        c = W.r.completion()
        kind = self.spec['completion']
        if kind != 'never':
            W.r.register_callback(lambda e: c.complete(kind == 'fail'),
                                  W.r.monotonic() + 0.005)
        return c
    def home_wait(self, end_time):
        W = self.W
        k = W.next('wait:' + self.name)
        W.t("home_wait %s %s" % (self.name, fl(end_time)))
        w = seq(self.spec['waits'], k)
        if w[0] == 'cmd':
            raise CommandError(w[1])
        if w[0] == 'mcu':
            raise error(w[1])
        return w[1]

class FakeProbeEndstop(FakeEndstop):
    def get_position_endstop(self):
        return 1.0

class FakeRail:
    def __init__(self, W, spec, steppers, endstop):
        self.W = W
        self.spec = spec
        self.steppers = steppers
        self.endstop = endstop
    def get_name(self, short=False):
        return self.spec['name']
    def get_range(self):
        return (self.spec['pmin'], self.spec['pmax'])
    def get_homing_info(self):
        s = self.spec
        return HomingInfo(s['speed'], s['pend'], s['retract_speed'], s['retract_dist'],
                          s['positive_dir'], s['second_speed'])
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return [(self.endstop, self.spec['name'])]

class FakeKin:
    def __init__(self, W):
        self.W = W
    def get_steppers(self):
        return list(self.W.steppers)
    def calc_position(self, sp):
        pos = [sp[rail.get_name()] for rail in self.W.rails]
        n = self.W.S['calc_none']
        if n is not None:
            pos[n] = None
        return pos
    def set_position(self, newpos, homing_axes):
        for st in self.W.steppers:
            st.set_position(newpos[:3])
    def home(self, homing_state):
        for axis in homing_state.get_axes():
            rail = self.W.rails[axis]
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            homing_state.home_rails([rail], forcepos, homepos)

class FakeToolhead:
    def __init__(self, W, pos):
        self.W = W
        self.pos = list(pos)
    def get_position(self):
        return list(self.pos)
    def set_position(self, newpos, homing_axes=""):
        self.W.t("set_position %s %r" % (fll(newpos), homing_axes))
        self.pos = list(newpos)
        self.W.kin.set_position(newpos[:3], homing_axes)
    def get_kinematics(self):
        return self.W.kin
    def flush_step_generation(self):
        self.W.t("flush_step_generation")
    def get_last_move_time(self):
        k = self.W.next('lmt')
        return 10.0 + 0.5 * k
    def dwell(self, delay):
        self.W.t("dwell %s" % (fl(delay),))
    def move(self, newpos, speed):
        self.W.t("move %s %s" % (fll(newpos), fl(speed)))
        for s in self.W.steppers:
            s.cmd = s.calc_position_from_coord(newpos)
        self.pos = list(newpos)
    def drip_move(self, newpos, speed, drip_completion):
        W = self.W
        newpos4 = list(newpos[:3]) + self.pos[3:]
        [newpos4[i] - self.pos[i] for i in (0, 1, 2, 3)]
        k = W.next('drip')
        W.t("drip_move %s %s" % (fll(newpos), fl(speed)))
        W.r.pause(W.r.monotonic() + 0.03)
        W.t("drip_done test=%r" % (drip_completion.test(),))
        frac = seq(W.S['halt_frac'], k)
        for s in W.steppers:
            tgt = s.calc_position_from_coord(newpos)
            s.cmd = s.cmd + (tgt - s.cmd) * frac
        self.pos = list(newpos)
        err = seq(W.S['drip_err'], k)
        if err is not None:
            raise CommandError(err)

class FakeGcmd:
    def __init__(self, command, params):
        self.command = command
        self.params = dict(params)
    def get_command(self):
        return self.command
    def get(self, name, default=_sentinel):
        if name in self.params:
            return self.params[name]
        if default is _sentinel:
            raise CommandError("Error on '%s': missing %s" % (self.command, name))
        return default
    def get_command_parameters(self):
        return dict(self.params)

class FakeGCode:
    def __init__(self, W):
        self.W = W
        self.handlers = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.W.t("register_command %s when_not_ready=%r desc=%r" % (cmd, when_not_ready, desc))
        self.handlers[cmd] = func
    def create_gcode_command(self, command, commandline, params):
        return FakeGcmd(command, params)

class Res:
    def __init__(self, bed_z):
        self.bed_z = bed_z

class FakeSession:
    def __init__(self, W):
        self.W = W
        self.res = []
    def run_probe(self, gcmd):
        W = self.W
        k = W.next('probe_run')
        W.t("run_probe %s %s" % (gcmd.get_command(), gcmd.get_command_parameters()))
        err = seq(W.S['probe_err'], k)
        if err is not None:
            raise CommandError(err)
        if not W.S['probe_empty']:
            self.res.append(Res(seq(W.S['probe_z'], k)))
    def pull_probed_results(self):
        r = self.res
        self.res = []
        return r
    def end_probe_session(self):
        self.W.t("end_probe_session")

class FakeProbe:
    def __init__(self, W):
        self.W = W
    def start_probe_session(self, gcmd):
        self.W.t("start_probe_session %s %s" % (gcmd.get_command(),
                                                gcmd.get_command_parameters()))
        return FakeSession(self.W)

class FakeStepperEnable:
    def __init__(self, W):
        self.W = W
    def motor_off(self):
        self.W.t("motor_off")
        if self.W.S['motor_off_err'] is not None:
            raise CommandError(self.W.S['motor_off_err'])

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, W):
        self.W = W
        self.objects = {}
    def get_reactor(self):
        return self.W.r
    def get_start_args(self):
        return {'debuginput': 'input.gcode' if self.W.S['debuginput'] else None}
    def is_shutdown(self):
        return self.W.S['shutdown']
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def send_event(self, event, *params):
        W = self.W
        S = W.S
        if event == 'homing:homing_move_begin':
            W.t("ev homing_move_begin %s" % (enames(params[0].get_mcu_endstops()),))
        elif event == 'homing:homing_move_end':
            W.t("ev homing_move_end %s nomove=%r" % (enames(params[0].get_mcu_endstops()),
                                                    params[0].check_no_movement()))
            if S['end_err'] is not None:
                raise CommandError(S['end_err'])
            if S['end_internal']:
                raise ValueError('boom')
        elif event == 'homing:home_rails_begin':
            hs, rails = params
            W.t("ev home_rails_begin axes=%s rails=%s" % (hs.get_axes(),
                                                          [r.get_name() for r in rails]))
        elif event == 'homing:home_rails_end':
            hs, rails = params
            trig = []
            for rail in rails:
                for s in rail.get_steppers():
                    try:
                        trig.append(hs.get_trigger_position(s.get_name()))
                    except KeyError:
                        trig.append(None)
            for n in S['query']:
                trig.append(hs.get_trigger_position(n))
            W.t("ev home_rails_end axes=%s rails=%s trig=%s" % (
                hs.get_axes(), [r.get_name() for r in rails], trig))
            for n, a in S['adjust']:
                hs.set_stepper_adjustment(n, a)
        else:
            raise AssertionError(event)

class FakeConfig:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self):
        return self._printer

def build(S, r):
    W = World(S, r)
    printer = FakePrinter(W)
    W.printer = printer
    for i, rs in enumerate(S['rails']):
        sts = []
        for sn, sd in zip(rs['steppers'], rs['step_dists']):
            st = FakeStepper(W, sn, i, sd)
            sts.append(st)
            W.steppers.append(st)
        es = rs['end']
        cls = FakeProbeEndstop if es['probe'] else FakeEndstop
        endstop = cls(W, rs['name'], sts if es['attached'] else [], es)
        W.rails.append(FakeRail(W, rs, sts, endstop))
    W.kin = FakeKin(W)
    W.toolhead = FakeToolhead(W, S['pos'])
    W.gcode = FakeGCode(W)
    printer.objects['gcode'] = W.gcode
    printer.objects['toolhead'] = W.toolhead
    printer.objects['stepper_enable'] = FakeStepperEnable(W)
    if S['probe']:
        printer.objects['probe'] = FakeProbe(W)
    return W

def act(W, A):
    k = A['kind']
    printer = W.printer
    if k == 'g28':
        homing.load_config(FakeConfig(printer))
        W.gcode.handlers['G28'](FakeGcmd('G28', A['params']))
        return None
    if k == 'manual_home':
        ph = homing.load_config(FakeConfig(printer))
        rail = W.rails[A['rail']]
        return list(ph.manual_home(W.toolhead, rail.get_endstops(), A['pos'], A['speed'],
                                   A['probe_pos'], A['triggered'], A['check_triggered']))
    if k == 'probing_move':
        ph = homing.load_config(FakeConfig(printer))
        rail = W.rails[A['rail']]
        return list(ph.probing_move(rail.endstop, A['pos'], A['speed'], A['check_movement']))
    if k == 'home_rails':
        h = homing.Homing(printer)
        h.set_axes(A['axes'])
        h.home_rails([W.rails[i] for i in A['rails']], A['forcepos'], A['movepos'])
        trig = []
        for n in ['stepper_x', 'stepper_y', 'stepper_z', 'stepper_z1', 'bogus']:
            try:
                trig.append(['ok', h.get_trigger_position(n)])
            except Exception as e:
                trig.append(['err', type(e).__name__, str(e)])
        return {'axes': h.get_axes(), 'trig': trig}
    if k == 'check_first':
        ph = homing.load_config(FakeConfig(printer))
        params = {}
        if A['attempt'] is not None:
            params['HOME_ATTEMPT_NUM'] = A['attempt']
        return ph.check_probe_first_home(FakeGcmd(A['command'], params))
    if k == 'multi':
        r = W.r
        comps = []
        for kind, delay in A['comps']:
            c = r.completion()
            if kind != 'never':
                r.register_callback(lambda e, c=c, kind=kind: c.complete(kind == 'fail'),
                                    r.monotonic() + delay)
            comps.append(c)
        cp = homing.multi_complete(printer, comps)
        r.pause(r.monotonic() + A['wait'])
        return [cp.test(), [c.test() for c in comps]]
    raise AssertionError(k)

def execute(S, A, r):
    W = build(S, r)
    ret = None
    err = None
    try:
        ret = act(W, A)
    except Exception as e:
        err = [type(e).__name__, str(e)]
    return {'err': err, 'ret': ret, 'trace': W.trace, 'log': list(LOG),
            'pos': list(W.toolhead.pos), 'cmds': [s.cmd for s in W.steppers],
            'offs': [s.off for s in W.steppers]}

def run(S, A):
    del LOG[:]
    r = reactor_mod.PollReactor()
    out = []
    def main(e):
        try:
            out.append(execute(S, A, r))
        except BaseException as ex:
            out.append({'harness_error': repr(ex)})
        r.end()
    r.register_callback(main)
    r.run()
    return out[0]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

fn fl(x: f64) -> String {
    float_repr(x)
}

fn fll(v: &[f64]) -> String {
    format!("[{}]", v.iter().map(|x| fl(*x)).collect::<Vec<_>>().join(", "))
}

fn pyb(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn strs_repr(v: &[String]) -> String {
    format!("[{}]", v.iter().map(|s| str_repr(s)).collect::<Vec<_>>().join(", "))
}

fn ints_repr(v: &[i64]) -> String {
    format!("[{}]", v.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(", "))
}

fn opt_ints_repr(v: &[Option<i64>]) -> String {
    let items: Vec<String> = v
        .iter()
        .map(|i| match i {
            Some(i) => i.to_string(),
            None => "None".to_string(),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

fn dict_repr(m: &IndexMap<String, String>) -> String {
    let items: Vec<String> =
        m.iter().map(|(k, v)| format!("{}: {}", str_repr(k), str_repr(v))).collect();
    format!("{{{}}}", items.join(", "))
}

fn enames(endstops: &[Endstop]) -> String {
    let items: Vec<String> = endstops
        .iter()
        .map(|e| {
            let names: Vec<String> = e.get_steppers().iter().map(|s| s.get_name(None)).collect();
            strs_repr(&names)
        })
        .collect();
    format!("[{}]", items.join(", "))
}

fn rail_names(rails: &[Rail]) -> String {
    let names: Vec<String> = rails.iter().map(|r| r.get_name(None)).collect();
    strs_repr(&names)
}

/// Python `int(x)` of a float.
fn py_int(x: f64) -> i64 {
    x.trunc() as i64
}

struct World {
    spec: Spec,
    reactor: Reactor,
    trace: GCell<Vec<String>>,
    counts: GCell<IndexMap<String, usize>>,
    steppers: GCell<Vec<Arc<FakeStepper>>>,
    rails: GCell<Vec<Arc<FakeRail>>>,
    kin: GCell<Option<Arc<FakeKin>>>,
}

impl World {
    fn t(&self, msg: String) {
        self.trace.borrow_mut().push(msg);
    }

    fn next(&self, key: &str) -> usize {
        self.counts.with_mut(|counts| {
            let slot = counts.entry(key.to_string()).or_insert(0);
            let n = *slot;
            *slot += 1;
            n
        })
    }

    fn steppers(&self) -> Vec<Arc<FakeStepper>> {
        self.steppers.cloned()
    }
}

struct FakeStepper {
    world: Arc<World>,
    name: String,
    axis: usize,
    sd: f64,
    cmd: GCell<f64>,
    off: GCell<f64>,
}

impl FakeStepper {
    fn mcu_pos(&self, cmd_pos: f64) -> i64 {
        let mcu_pos_dist = cmd_pos + self.off.get();
        let mcu_pos = mcu_pos_dist / self.sd;
        if mcu_pos >= 0.0 {
            py_int(mcu_pos + 0.5)
        } else {
            py_int(mcu_pos - 0.5)
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by homing")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by homing")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians is not used by homing")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by homing")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration is not used by homing")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by homing")
    }
    fn get_step_dist(&self) -> f64 {
        self.sd
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by homing")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance is not used by homing")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by homing")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by homing")
    }
    fn calc_position_from_coord(&self, coord: [f64; 3]) -> f64 {
        coord[self.axis]
    }
    fn set_position(&self, coord: [f64; 3]) {
        let mcu_pos = self.get_mcu_position(None);
        self.cmd.set(coord[self.axis]);
        let mcu_pos_dist = (mcu_pos as f64) * self.sd;
        self.off.set(mcu_pos_dist - self.cmd.get());
    }
    fn get_commanded_position(&self) -> f64 {
        self.cmd.get()
    }
    fn get_mcu_position(&self, cmd_pos: Option<f64>) -> i64 {
        let cmd_pos = match cmd_pos {
            Some(p) => p,
            None => self.get_commanded_position(),
        };
        self.mcu_pos(cmd_pos)
    }
    fn get_past_mcu_position(&self, print_time: f64) -> i64 {
        let k = self.world.next(&format!("past:{}", self.name));
        self.world.t(format!("past {} {}", self.name, fl(print_time)));
        let lag = match self.world.spec.lag.iter().find(|(n, _)| *n == self.name) {
            Some((_, lags)) if !lags.is_empty() => seq(lags, k),
            _ => 0,
        };
        self.get_mcu_position(None) - lag
    }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        (mcu_pos as f64) * self.sd - self.off.get()
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        panic!("dump_steps is not used by homing")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics is not used by homing")
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics is not used by homing")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by homing")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq is not used by homing")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        panic!("set_trapq is not used by homing")
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        panic!("add_active_callback is not used by homing")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by homing")
    }
}

struct FakeEndstop {
    world: Arc<World>,
    name: String,
    steppers: Vec<Stepper>,
    spec: EndSpec,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by homing")
    }
    fn add_stepper(&self, _stepper: Stepper) {
        panic!("add_stepper is not used by homing")
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn home_start(
        &self,
        print_time: f64,
        sample_time: f64,
        sample_count: i64,
        rest_time: f64,
        triggered: Option<bool>,
    ) -> Completion<bool> {
        let w = &self.world;
        w.t(format!(
            "home_start {} {} {} {} {} {}",
            self.name,
            fl(print_time),
            fl(sample_time),
            sample_count,
            fl(rest_time),
            pyb(triggered.expect("homing always passes triggered"))
        ));
        let completion: Completion<bool> = w.reactor.completion();
        if self.spec.completion != "never" {
            let target = completion.clone();
            let fail = self.spec.completion == "fail";
            w.reactor.register_callback(
                move |_eventtime| {
                    target.complete(fail);
                },
                w.reactor.monotonic() + 0.005,
            );
        }
        completion
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        let w = &self.world;
        let k = w.next(&format!("wait:{}", self.name));
        w.t(format!("home_wait {} {}", self.name, fl(home_end_time)));
        match seq(&self.spec.waits, k) {
            Wait::Time(t) => Ok(t),
            Wait::Cmd(msg) => Err(KlippyError::Command(msg.to_string())),
            Wait::Mcu(msg) => Err(KlippyError::Mcu(msg.to_string())),
        }
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("query_endstop is not used by homing")
    }
    fn get_position_endstop(&self) -> Option<f64> {
        if self.spec.probe {
            Some(1.0)
        } else {
            None
        }
    }
}

struct FakeRail {
    spec: RailSpec,
    steppers: Vec<Stepper>,
    endstop: Arc<FakeEndstop>,
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.spec.name.to_string()
    }
    fn get_range(&self) -> (f64, f64) {
        (self.spec.pmin, self.spec.pmax)
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: self.spec.speed,
            position_endstop: self.spec.pend,
            retract_speed: self.spec.retract_speed,
            retract_dist: self.spec.retract_dist,
            positive_dir: self.spec.positive_dir,
            second_homing_speed: self.spec.second_speed,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)> {
        vec![(self.endstop.clone() as Arc<dyn EndstopApi>, self.spec.name.to_string())]
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Arc<dyn EndstopApi>> {
        panic!("lookup_endstop is not used by homing")
    }
    fn add_stepper(
        &self,
        _stepper: Stepper,
        _endstop_pin: Option<&str>,
        _endstop_name: Option<&str>,
    ) -> KResult<()> {
        panic!("add_stepper is not used by homing")
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        panic!("add_stepper_from_config is not used by homing")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by homing")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {
        panic!("set_trapq is not used by homing")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("set_position is not used by homing")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by homing")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by homing")
    }
}

struct FakeKin {
    world: Arc<World>,
}

impl KinematicsApi for FakeKin {
    fn get_steppers(&self) -> Vec<Stepper> {
        self.world.steppers().into_iter().map(|s| s as Stepper).collect()
    }
    fn calc_position(
        &self,
        stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        let rails = self.world.rails.cloned();
        let mut pos: Vec<Option<f64>> = Vec::new();
        for rail in &rails {
            match stepper_positions.get(rail.spec.name) {
                Some(v) => pos.push(Some(*v)),
                None => {
                    return Err(KlippyError::Internal {
                        class_name: "KeyError".to_string(),
                        message: str_repr(rail.spec.name),
                    })
                }
            }
        }
        if let Some(n) = self.world.spec.calc_none {
            pos[n] = None;
        }
        Ok(pos)
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        for st in self.world.steppers() {
            st.set_position([newpos[0], newpos[1], newpos[2]]);
        }
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        panic!("check_move is not used by homing")
    }
    fn home(&self, homing_state: HomingStateHandle) -> KResult<()> {
        for axis in homing_state.get_axes() {
            let rail = self.world.rails.cloned()[axis as usize].clone();
            let (position_min, position_max) = rail.get_range();
            let hi = rail.get_homing_info();
            let mut homepos: Vec<Option<f64>> = vec![None, None, None, None];
            homepos[axis as usize] = Some(hi.position_endstop);
            let mut forcepos = homepos.clone();
            let slot = forcepos[axis as usize].as_mut().expect("just set");
            if hi.positive_dir {
                *slot -= 1.5 * (hi.position_endstop - position_min);
            } else {
                *slot += 1.5 * (position_max - hi.position_endstop);
            }
            homing_state.home_rails(vec![rail as Rail], forcepos, homepos)?;
        }
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing")
    }
    fn clear_homing_state(&self, _clear_axes: &str) {
        panic!("clear_homing_state is not used by homing")
    }
}

struct FakeToolhead {
    world: Arc<World>,
    pos: GCell<Vec<f64>>,
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi, dyn HomingTargetApi);

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.pos.cloned()
    }
    fn set_position(&self, newpos: Vec<f64>, homing_axes: &str) -> KResult<()> {
        self.world.t(format!("set_position {} {}", fll(&newpos), str_repr(homing_axes)));
        self.pos.replace(newpos.clone());
        let kin = self.world.kin.cloned().expect("kinematics built");
        kin.set_position(newpos[..3].to_vec(), homing_axes)
    }
    fn get_kinematics(&self) -> Kinematics {
        self.world.kin.cloned().expect("kinematics built") as Kinematics
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.world.t("flush_step_generation".to_string());
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        let k = self.world.next("lmt");
        10.0 + 0.5 * (k as f64)
    }
    fn dwell(&self, delay: f64) {
        self.world.t(format!("dwell {}", fl(delay)));
    }
    fn drip_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        let w = &self.world;
        let k = w.next("drip");
        w.t(format!("drip_move {} {}", fll(&newpos), fl(speed)));
        w.reactor.pause(w.reactor.monotonic() + 0.03);
        w.t(format!("drip_done test={}", pyb(drip_completion.test())));
        let frac = seq(&w.spec.halt_frac, k);
        for s in w.steppers() {
            let tgt = s.calc_position_from_coord([newpos[0], newpos[1], newpos[2]]);
            let cmd = s.cmd.get();
            s.cmd.set(cmd + (tgt - cmd) * frac);
        }
        self.pos.replace(newpos);
        match seq(&w.spec.drip_err, k) {
            Some(err) => Err(KlippyError::Command(err.to_string())),
            None => Ok(()),
        }
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        let w = &self.world;
        w.t(format!("move {} {}", fll(&newpos), fl(speed)));
        for s in w.steppers() {
            s.cmd.set(s.calc_position_from_coord([newpos[0], newpos[1], newpos[2]]));
        }
        self.pos.replace(newpos);
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by homing")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by homing")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by homing")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by homing")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by homing")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by homing")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by homing")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by homing")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by homing")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by homing")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by homing")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by homing")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by homing")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by homing")
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("stepper_factory is not used by homing")
    }
}

struct FakeGcmd {
    command: String,
    params: IndexMap<String, String>,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        if let Some(v) = self.params.get(name) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            None => Err(KlippyError::Command(format!(
                "Error on '{}': missing {}",
                self.command, name
            ))),
            Some(d) => Ok(d),
        }
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int is not used by homing")
    }
    fn get_float(&self, _name: &str, _opts: GGetFloatOpts) -> KResult<Option<f64>> {
        panic!("get_float is not used by homing")
    }
    fn get_command(&self) -> String {
        self.command.clone()
    }
    fn get_commandline(&self) -> String {
        panic!("get_commandline is not used by homing")
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        panic!("get_raw_command_parameters is not used by homing")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used by homing")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by homing")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("ack is not used by homing")
    }
}

fn gcmd(command: &str, params: &[(&str, &str)]) -> GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeGcmd { command: command.to_string(), params: map })
}

struct FakeGCode {
    world: Arc<World>,
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    /// Runs the handler registered for `cmd`. The handler is taken out of the map for the
    /// call because the real `G28` pauses the reactor.
    fn call(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handler = self
            .handlers
            .with_mut(|handlers| handlers.shift_remove(cmd))
            .expect("handler registered");
        let result = handler(gcmd);
        self.handlers.with_mut(|handlers| handlers.insert(cmd.to_string(), handler));
        result
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let desc = match &opts.desc {
            Some(d) => str_repr(d),
            None => "None".to_string(),
        };
        self.world.t(format!(
            "register_command {} when_not_ready={} desc={}",
            cmd,
            pyb(opts.when_not_ready),
            desc
        ));
        match func {
            Some(func) => {
                self.handlers.with_mut(|handlers| handlers.insert(cmd.to_string(), func));
                Ok(None)
            }
            None => panic!("unregistering is not used by homing"),
        }
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        panic!("register_mux_command is not used by homing")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used by homing")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used by homing")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by homing")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by homing")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by homing")
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by homing")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        _commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGcmd { command: command.to_string(), params: params.clone() })
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by homing")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used by homing")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by homing")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by homing")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by homing")
    }
}

struct FakeSession {
    world: Arc<World>,
    res: GCell<Vec<ProbeResult>>,
}

impl ProbeSessionApi for FakeSession {
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        let w = &self.world;
        let k = w.next("probe_run");
        w.t(format!(
            "run_probe {} {}",
            gcmd.get_command(),
            dict_repr(&gcmd.get_command_parameters())
        ));
        if let Some(err) = seq(&w.spec.probe_err, k) {
            return Err(KlippyError::Command(err.to_string()));
        }
        if !w.spec.probe_empty {
            let z = seq(&w.spec.probe_z, k);
            self.res.borrow_mut().push(ProbeResult {
                bed_x: 0.0,
                bed_y: 0.0,
                bed_z: z,
                test_x: 0.0,
                test_y: 0.0,
                test_z: 0.0,
            });
        }
        Ok(())
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(self.res.replace(Vec::new()))
    }
    fn end_probe_session(&self) -> KResult<()> {
        self.world.t("end_probe_session".to_string());
        Ok(())
    }
}

struct FakeProbe {
    world: Arc<World>,
}

klippy_api::printer_object!(FakeProbe; dyn PrinterProbeApi);

impl PrinterProbeApi for FakeProbe {
    fn get_probe_params(
        &self,
        _gcmd: Option<&GCmd>,
    ) -> KResult<extras::probe::ProbeParams> {
        panic!("get_probe_params is not used by homing")
    }
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        panic!("get_offsets is not used by homing")
    }
    fn start_probe_session(&self, gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        self.world.t(format!(
            "start_probe_session {} {}",
            gcmd.get_command(),
            dict_repr(&gcmd.get_command_parameters())
        ));
        Ok(Arc::new(FakeSession { world: self.world.clone(), res: GCell::new(Vec::new()) }))
    }
}

struct FakeStepperEnable {
    world: Arc<World>,
}

klippy_api::printer_object!(FakeStepperEnable; dyn PrinterStepperEnableApi);

impl PrinterStepperEnableApi for FakeStepperEnable {
    fn register_stepper(&self, _config: &Config, _stepper: &Stepper) -> KResult<()> {
        panic!("register_stepper is not used by homing")
    }
    fn set_motors_enable(&self, _stepper_names: &[String], _enable: bool) -> KResult<bool> {
        panic!("set_motors_enable is not used by homing")
    }
    fn motor_off(&self) -> KResult<()> {
        self.world.t("motor_off".to_string());
        match self.world.spec.motor_off_err {
            Some(msg) => Err(KlippyError::Command(msg.to_string())),
            None => Ok(()),
        }
    }
    fn lookup_enable(&self, _name: &str) -> KResult<Arc<EnableTracking>> {
        panic!("lookup_enable is not used by homing")
    }
    fn get_steppers(&self) -> Vec<String> {
        panic!("get_steppers is not used by homing")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by homing")
    }
}

struct FakePrinter {
    world: Arc<World>,
    objects: GCell<IndexMap<String, ObjectRef>>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut args = IndexMap::new();
        let value = if self.world.spec.debuginput {
            PyValue::Str("input.gcode".to_string())
        } else {
            PyValue::None
        };
        args.insert(PyValue::Str("debuginput".to_string()), value);
        PyValue::Dict(args)
    }
    fn get_reactor(&self) -> Reactor {
        self.world.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        panic!("get_state_message is not used by homing")
    }
    fn is_shutdown(&self) -> bool {
        self.world.spec.shutdown
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {
        panic!("update_error_msg is not used by homing")
    }
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.borrow().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => {
                Err(KlippyError::Config(format!("Unknown config object '{}'", name)))
            }
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by homing")
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by homing")
    }
    fn run(&self) -> Option<String> {
        panic!("run is not used by homing")
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {
        panic!("set_rollover_info is not used by homing")
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_shutdown is not used by homing")
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by homing")
    }
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {
        panic!("register_event_handler is not used by homing")
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let w = &self.world;
        let spec = &w.spec;
        match event {
            "homing:homing_move_begin" => {
                let hmove = params[0].as_handle::<dyn HomingMoveApi>().expect("HomingMove handle");
                w.t(format!("ev homing_move_begin {}", enames(&hmove.get_mcu_endstops())));
            }
            "homing:homing_move_end" => {
                let hmove = params[0].as_handle::<dyn HomingMoveApi>().expect("HomingMove handle");
                let nomove = match hmove.check_no_movement() {
                    Some(name) => str_repr(&name),
                    None => "None".to_string(),
                };
                w.t(format!(
                    "ev homing_move_end {} nomove={}",
                    enames(&hmove.get_mcu_endstops()),
                    nomove
                ));
                if let Some(msg) = spec.end_err {
                    return Err(KlippyError::Command(msg.to_string()));
                }
                if spec.end_internal {
                    return Err(KlippyError::Internal {
                        class_name: "ValueError".to_string(),
                        message: "boom".to_string(),
                    });
                }
            }
            "homing:home_rails_begin" => {
                let hs = params[0].as_handle::<dyn HomingStateApi>().expect("Homing handle");
                let rails = params[1].as_handle_list::<dyn RailApi>().expect("rail list");
                w.t(format!(
                    "ev home_rails_begin axes={} rails={}",
                    ints_repr(&hs.get_axes()),
                    rail_names(&rails)
                ));
            }
            "homing:home_rails_end" => {
                let hs = params[0].as_handle::<dyn HomingStateApi>().expect("Homing handle");
                let rails = params[1].as_handle_list::<dyn RailApi>().expect("rail list");
                let mut trig: Vec<Option<i64>> = Vec::new();
                for rail in &rails {
                    for s in rail.get_steppers() {
                        match hs.get_trigger_position(&s.get_name(None)) {
                            Ok(v) => trig.push(Some(v)),
                            Err(e) if e.class_name() == "KeyError" => trig.push(None),
                            Err(e) => return Err(e),
                        }
                    }
                }
                for n in &spec.query {
                    trig.push(Some(hs.get_trigger_position(n)?));
                }
                w.t(format!(
                    "ev home_rails_end axes={} rails={} trig={}",
                    ints_repr(&hs.get_axes()),
                    rail_names(&rails),
                    opt_ints_repr(&trig)
                ));
                for (n, a) in &spec.adjust {
                    hs.set_stepper_adjustment(n, *a);
                }
            }
            other => panic!("unexpected event {other}"),
        }
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {
        panic!("request_exit is not used by homing")
    }
}

struct FakeConfig {
    printer: Printer,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        panic!("get_name is not used by homing")
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        panic!("get is not used by homing")
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by homing")
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint is not used by homing")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by homing")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by homing")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by homing")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by homing")
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by homing")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by homing")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by homing")
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by homing")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by homing")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("get_prefix_options is not used by homing")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("deprecate is not used by homing")
    }
}

struct Env {
    world: Arc<World>,
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    rails: Vec<Arc<FakeRail>>,
}

fn build(spec: Spec, reactor: Reactor) -> Env {
    let world = Arc::new(World {
        spec: spec.clone(),
        reactor,
        trace: GCell::new(Vec::new()),
        counts: GCell::new(IndexMap::new()),
        steppers: GCell::new(Vec::new()),
        rails: GCell::new(Vec::new()),
        kin: GCell::new(None),
    });
    let printer =
        Arc::new(FakePrinter { world: world.clone(), objects: GCell::new(IndexMap::new()) });
    for (i, rs) in spec.rails.iter().enumerate() {
        let mut sts: Vec<Arc<FakeStepper>> = Vec::new();
        for (sn, sd) in rs.steppers.iter().zip(rs.step_dists.iter()) {
            let st = Arc::new(FakeStepper {
                world: world.clone(),
                name: sn.to_string(),
                axis: i,
                sd: *sd,
                cmd: GCell::new(0.0),
                off: GCell::new(0.0),
            });
            sts.push(st.clone());
            world.steppers.with_mut(|all| all.push(st));
        }
        let attached: Vec<Stepper> = if rs.end.attached {
            sts.iter().map(|s| s.clone() as Stepper).collect()
        } else {
            Vec::new()
        };
        let endstop = Arc::new(FakeEndstop {
            world: world.clone(),
            name: rs.name.to_string(),
            steppers: attached,
            spec: rs.end.clone(),
        });
        let rail = Arc::new(FakeRail {
            spec: rs.clone(),
            steppers: sts.iter().map(|s| s.clone() as Stepper).collect(),
            endstop,
        });
        world.rails.with_mut(|all| all.push(rail));
    }
    world.kin.replace(Some(Arc::new(FakeKin { world: world.clone() })));
    let toolhead =
        Arc::new(FakeToolhead { world: world.clone(), pos: GCell::new(spec.pos.clone()) });
    let gcode = Arc::new(FakeGCode { world: world.clone(), handlers: GCell::new(IndexMap::new()) });
    printer.objects.with_mut(|objects| {
        objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
        objects.insert("toolhead".to_string(), toolhead.clone() as ObjectRef);
        objects.insert(
            "stepper_enable".to_string(),
            Arc::new(FakeStepperEnable { world: world.clone() }) as ObjectRef,
        );
        if spec.probe {
            objects.insert(
                "probe".to_string(),
                Arc::new(FakeProbe { world: world.clone() }) as ObjectRef,
            );
        }
    });
    let rails = world.rails.cloned();
    Env { world, printer, gcode, toolhead, rails }
}

// ---------------------------------------------------------------------------
// Rust twin of the Python `act` / `execute` / `run`.
// ---------------------------------------------------------------------------

fn vstr(s: &str) -> RecordingValue {
    RecordingValue::str(s)
}

fn vfloats(v: &[f64]) -> RecordingValue {
    RecordingValue::List(v.iter().map(|f| RecordingValue::float(*f)).collect())
}

fn vstrs(v: &[String]) -> RecordingValue {
    RecordingValue::List(v.iter().map(|s| vstr(s)).collect())
}

fn verr(e: &KlippyError) -> RecordingValue {
    RecordingValue::List(vec![vstr(e.class_name()), vstr(e.message())])
}

fn act(env: &Env, action: &Action) -> KResult<RecordingValue> {
    let printer: Printer = env.printer.clone();
    let config: Config = Arc::new(FakeConfig { printer: printer.clone() });
    match action {
        Action::G28(params) => {
            let _homing = PrinterHoming::new(&config)?;
            env.gcode.call("G28", &gcmd("G28", params))?;
            Ok(RecordingValue::Null)
        }
        Action::ManualHome { rail, pos, speed, probe_pos, triggered, check_triggered } => {
            let homing = PrinterHoming::new(&config)?;
            let rail = &env.rails[*rail];
            let toolhead: HomingTarget = env.toolhead.clone();
            let endstops = rail.get_endstops();
            let epos = homing.manual_home(
                toolhead,
                endstops,
                pos,
                *speed,
                *probe_pos,
                *triggered,
                *check_triggered,
            )?;
            Ok(vfloats(&epos))
        }
        Action::ProbingMove { rail, pos, speed, check_movement } => {
            let homing = PrinterHoming::new(&config)?;
            let rail = &env.rails[*rail];
            let endstop: Endstop = rail.endstop.clone();
            let epos = homing.probing_move(endstop, pos, *speed, *check_movement)?;
            Ok(vfloats(&epos))
        }
        Action::HomeRails { rails, axes, forcepos, movepos } => {
            let homing = Homing::new(&printer)?;
            homing.set_axes(axes.clone());
            let rails: Vec<Rail> = rails.iter().map(|i| env.rails[*i].clone() as Rail).collect();
            homing.home_rails(&rails, forcepos, movepos)?;
            let mut trig = Vec::new();
            for n in ["stepper_x", "stepper_y", "stepper_z", "stepper_z1", "bogus"] {
                trig.push(match homing.get_trigger_position(n) {
                    Ok(v) => RecordingValue::List(vec![vstr("ok"), RecordingValue::int(v)]),
                    Err(e) => RecordingValue::List(vec![
                        vstr("err"),
                        vstr(e.class_name()),
                        vstr(e.message()),
                    ]),
                });
            }
            Ok(RecordingValue::Dict(vec![
                (
                    vstr("axes"),
                    RecordingValue::List(
                        homing.get_axes().iter().map(|a| RecordingValue::int(*a)).collect(),
                    ),
                ),
                (vstr("trig"), RecordingValue::List(trig)),
            ]))
        }
        Action::CheckFirst { command, attempt } => {
            let homing = PrinterHoming::new(&config)?;
            let mut params: Vec<(&str, &str)> = Vec::new();
            if let Some(a) = attempt {
                params.push(("HOME_ATTEMPT_NUM", a));
            }
            Ok(RecordingValue::Bool(homing.check_probe_first_home(&gcmd(command, &params))?))
        }
        Action::Multi { comps, wait } => {
            let reactor = env.world.reactor.clone();
            let mut completions: Vec<Completion<bool>> = Vec::new();
            for (kind, delay) in comps {
                let c: Completion<bool> = reactor.completion();
                if *kind != "never" {
                    let target = c.clone();
                    let fail = *kind == "fail";
                    reactor.register_callback(
                        move |_eventtime| {
                            target.complete(fail);
                        },
                        reactor.monotonic() + delay,
                    );
                }
                completions.push(c);
            }
            let cp = multi_complete(&printer, completions.clone());
            reactor.pause(reactor.monotonic() + wait);
            Ok(RecordingValue::List(vec![
                RecordingValue::Bool(cp.test()),
                RecordingValue::List(
                    completions.iter().map(|c| RecordingValue::Bool(c.test())).collect(),
                ),
            ]))
        }
    }
}

fn execute(env: &Env, action: &Action) -> RecordingValue {
    let (ret, err) = match act(env, action) {
        Ok(v) => (v, RecordingValue::Null),
        Err(e) => (RecordingValue::Null, verr(&e)),
    };
    let trace = env.world.trace.cloned();
    let log = log_lines();
    let steppers = env.world.steppers();
    RecordingValue::Dict(vec![
        (vstr("err"), err),
        (vstr("ret"), ret),
        (vstr("trace"), vstrs(&trace)),
        (vstr("log"), vstrs(&log)),
        (vstr("pos"), vfloats(&env.toolhead.pos.cloned())),
        (vstr("cmds"), vfloats(&steppers.iter().map(|s| s.cmd.get()).collect::<Vec<_>>())),
        (vstr("offs"), vfloats(&steppers.iter().map(|s| s.off.get()).collect::<Vec<_>>())),
    ])
}

/// Runs the scenario inside the Rust reactor; a watchdog finalizes the reactor when a
/// baton deadlock would otherwise hang the test.
fn run_rust(spec: &Spec, action: &Action) -> RecordingValue {
    clear_log_lines();
    let reactor = Reactor::new(false);
    let env = build(spec.clone(), reactor.clone());
    let out: Arc<kos::sync::Mutex<Option<RecordingValue>>> = Arc::new(kos::sync::Mutex::new(None));
    {
        let out = out.clone();
        let action = action.clone();
        let ender = reactor.clone();
        reactor.register_callback(
            move |_eventtime| {
                let value = execute(&env, &action);
                *out.lock() = Some(value);
                ender.end();
            },
            NOW,
        );
    }
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let watchdog = {
        let reactor = reactor.clone();
        let done = done.clone();
        let fired = fired.clone();
        kos::thread::spawn(move || {
            let deadline = kos::time::monotonic() + 30.0;
            while kos::time::monotonic() < deadline {
                if done.load(std::sync::atomic::Ordering::SeqCst) {
                    return;
                }
                kos::time::sleep(0.01);
            }
            if !done.load(std::sync::atomic::Ordering::SeqCst) {
                fired.store(true, std::sync::atomic::Ordering::SeqCst);
                reactor.finalize();
            }
        })
    };
    reactor.run();
    done.store(true, std::sync::atomic::Ordering::SeqCst);
    watchdog.join();
    assert!(
        !fired.load(std::sync::atomic::Ordering::SeqCst),
        "reactor run timed out after 30s (baton deadlock?)"
    );
    let result = out.lock().take();
    result.expect("scenario finished")
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

thread_local! {
    /// Lines logged on this test's thread; parallel tests cannot clear or
    /// add to each other's capture.
    static LOG: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static SINK: std::sync::Once = std::sync::Once::new();
    SINK.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG.with(|l| l.borrow_mut().push(format!("{}:{}", rec.level.name(), rec.message)));
        }));
    });
}

fn log_lines() -> Vec<String> {
    install_log_sink();
    LOG.with(|l| l.borrow().clone())
}

fn clear_log_lines() {
    install_log_sink();
    LOG.with(|l| l.borrow_mut().clear())
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("homing fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping homing oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and \
                 run with the repo checkout available"
            );
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: RecordingValue) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "homing".to_string(),
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

fn compare(py: &mut pyoracle::Oracle, label: &str, spec: &Spec, action: &Action) {
    let expected = py.eval(&format!("run({}, {})", py_spec(spec), py_action(action)));
    check(label, expected, run_rust(spec, action));
}

fn g28(params: &[(&'static str, &'static str)]) -> Action {
    Action::G28(params.to_vec())
}

fn home_rails(rails: &[usize], axes: &[i64], forcepos: &[Option<f64>], movepos: &[Option<f64>]) -> Action {
    Action::HomeRails {
        rails: rails.to_vec(),
        axes: axes.to_vec(),
        forcepos: forcepos.to_vec(),
        movepos: movepos.to_vec(),
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn g28_homes_every_axis_with_retract() {
    let Some(mut py) = oracle() else { return };
    compare(&mut py, "g28 all axes", &base(), &g28(&[]));
    compare(&mut py, "g28 X only", &base(), &g28(&[("X", "")]));
    compare(&mut py, "g28 Y and Z", &base(), &g28(&[("Y", ""), ("Z", "0")]));
    compare(&mut py, "g28 unrelated params home all", &base(), &g28(&[("HOME_ATTEMPT_NUM", "1")]));
    let mut dual = dual_z();
    dual.halt_frac = vec![0.25, 1.0, 0.5];
    compare(&mut py, "g28 dual-stepper z", &dual, &g28(&[("Z", "")]));
    compare(&mut py, "g28 dual-stepper all", &dual, &g28(&[]));
}

#[test]
fn g28_without_retract_homes_once() {
    let Some(mut py) = oracle() else { return };
    let mut spec = base();
    for rail in &mut spec.rails {
        rail.retract_dist = 0.0;
    }
    compare(&mut py, "g28 no retract", &spec, &g28(&[]));
    // No over-travel: the trigger position equals the halt position.
    spec.lag = vec![("stepper_x", vec![0]), ("stepper_y", vec![0]), ("stepper_z", vec![0])];
    compare(&mut py, "g28 no overtravel", &spec, &g28(&[("X", "")]));
    // Negative lag puts the trigger position beyond the halt position.
    spec.lag = vec![("stepper_x", vec![-7])];
    compare(&mut py, "g28 negative lag", &spec, &g28(&[("X", "")]));
}

#[test]
fn homing_move_errors_and_error_precedence() {
    let Some(mut py) = oracle() else { return };
    // Endstop never triggered.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "no trigger", &spec, &g28(&[("X", "")]));
    // home_wait raises a command error.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Cmd("Timeout during endstop homing")];
    compare(&mut py, "home_wait command error", &spec, &g28(&[("X", "")]));
    // home_wait raises a non-command error: it propagates and G28 does not stop the motors.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Mcu("Lost communication with MCU")];
    compare(&mut py, "home_wait mcu error", &spec, &g28(&[("X", "")]));
    // drip_move error wins over later errors.
    let mut spec = base();
    spec.drip_err = vec![Some("Move out of range")];
    spec.rails[0].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "drip error first", &spec, &g28(&[("X", "")]));
    // homing_move_end handler error becomes the error when no other exists.
    let mut spec = base();
    spec.end_err = Some("Endstop handler failed");
    compare(&mut py, "end event error", &spec, &g28(&[("X", "")]));
    // ... but not when an earlier error exists.
    let mut spec = base();
    spec.end_err = Some("Endstop handler failed");
    spec.rails[0].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "end event error after no trigger", &spec, &g28(&[("X", "")]));
    // A non-command error of the handler propagates unchanged.
    let mut spec = base();
    spec.end_internal = true;
    compare(&mut py, "end event internal error", &spec, &g28(&[("X", "")]));
    // check_triggered stays on for G28: a zero trigger time at the retract move fails too.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Time(4.5), Wait::Time(0.0)];
    compare(&mut py, "no trigger on second home", &spec, &g28(&[("X", "")]));
}

#[test]
fn g28_failure_turns_motors_off_unless_shutdown() {
    let Some(mut py) = oracle() else { return };
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "motor_off after failure", &spec, &g28(&[("X", "")]));
    spec.shutdown = true;
    compare(&mut py, "shutdown replaces error", &spec, &g28(&[("X", "")]));
    spec.shutdown = false;
    spec.motor_off_err = Some("Motor off failed");
    compare(&mut py, "motor_off error replaces homing error", &spec, &g28(&[("X", "")]));
}

#[test]
fn retract_requires_movement_unless_debuginput() {
    let Some(mut py) = oracle() else { return };
    // The second home does not move the stepper: still triggered after retract.
    let mut spec = base();
    spec.halt_frac = vec![0.5, 0.0];
    spec.lag = vec![("stepper_x", vec![0])];
    compare(&mut py, "still triggered after retract", &spec, &g28(&[("X", "")]));
    // With `debuginput` set `check_no_movement` reports nothing.
    spec.debuginput = true;
    compare(&mut py, "debuginput skips movement check", &spec, &g28(&[("X", "")]));
}

#[test]
fn home_offsets_from_stepper_adjustments() {
    let Some(mut py) = oracle() else { return };
    let mut spec = base();
    spec.adjust = vec![("stepper_x", 0.5)];
    compare(&mut py, "adjustment applied", &spec, &g28(&[("X", "")]));
    // An adjustment of zero is not applied.
    spec.adjust = vec![("stepper_x", 0.0)];
    compare(&mut py, "zero adjustment ignored", &spec, &g28(&[("X", "")]));
    // An adjustment for another stepper still triggers the offset path.
    spec.adjust = vec![("stepper_y", -0.25), ("stepper_x", 0.0)];
    compare(&mut py, "adjustment of other axis", &spec, &g28(&[("X", "")]));
    // The kinematics cannot place the homed axis.
    spec.adjust = vec![("stepper_x", 0.5)];
    spec.calc_none = Some(0);
    compare(&mut py, "undetermined position", &spec, &g28(&[("X", "")]));
    // Only forced axes are checked.
    spec.calc_none = Some(1);
    compare(&mut py, "undetermined position of other axis", &spec, &g28(&[("X", "")]));
    // A handler asking for the trigger position of an unknown stepper.
    let mut spec = base();
    spec.query = vec!["bogus"];
    compare(&mut py, "unknown trigger position", &spec, &g28(&[("X", "")]));
}

#[test]
fn degenerate_geometry_raises_python_errors() {
    let Some(mut py) = oracle() else { return };
    // Zero homing speed divides by zero when the endstop rate is computed.
    let mut spec = base();
    spec.rails[0].speed = 0.0;
    compare(&mut py, "zero speed", &spec, &g28(&[("X", "")]));
    // A rail that starts at the endstop has a zero-length homing move, so the retract
    // ratio divides by zero.
    let mut spec = base();
    spec.rails[0].pmax = 0.0;
    compare(&mut py, "zero length move", &spec, &g28(&[("X", "")]));
    // ... and without retract it homes with the minimum rate.
    spec.rails[0].retract_dist = 0.0;
    compare(&mut py, "zero length move without retract", &spec, &g28(&[("X", "")]));
    // A rail whose endstop has no steppers is dropped from the move.
    let mut spec = base();
    spec.rails[0].end.attached = false;
    compare(&mut py, "endstop without steppers", &spec, &g28(&[("X", "")]));
    // Missing rails entry.
    compare(&mut py, "home_rails without rails", &base(), &home_rails(&[], &[0], &[Some(0.0), None, None, None], &[Some(1.0), None, None, None]));
}

#[test]
fn home_rails_with_several_rails_and_completions() {
    let Some(mut py) = oracle() else { return };
    let force = [Some(0.0), Some(0.0), None, None];
    let moves = [Some(10.0), Some(20.0), None, None];
    let spec = base();
    compare(&mut py, "two rails", &spec, &home_rails(&[0, 1], &[0, 1], &force, &moves));
    // One endstop reports a failure: the combined completion ends early.
    let mut spec = base();
    spec.rails[1].end.completion = "fail";
    compare(&mut py, "one failing completion", &spec, &home_rails(&[0, 1], &[0, 1], &force, &moves));
    // One endstop never completes: the drip waits for the full move.
    let mut spec = base();
    spec.rails[1].end.completion = "never";
    compare(&mut py, "one pending completion", &spec, &home_rails(&[0, 1], &[0, 1], &force, &moves));
    // A single endstop that never completes.
    let mut spec = base();
    spec.rails[0].end.completion = "never";
    compare(&mut py, "single pending completion", &spec, &home_rails(&[0], &[0], &force, &moves));
    // Both rails report errors from home_wait: only the first is kept.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Cmd("first")];
    spec.rails[1].end.waits = vec![Wait::Cmd("second")];
    compare(&mut py, "two wait errors", &spec, &home_rails(&[0, 1], &[0, 1], &force, &moves));
    // One rail has no trigger, the other raises: the earlier error is kept.
    let mut spec = base();
    spec.rails[0].end.waits = vec![Wait::Time(0.0)];
    spec.rails[1].end.waits = vec![Wait::Cmd("second")];
    compare(&mut py, "no trigger then wait error", &spec, &home_rails(&[0, 1], &[0, 1], &force, &moves));
    // A partially specified position keeps the toolhead coordinates elsewhere.
    compare(&mut py, "all rails", &spec_all(), &home_rails(&[0, 1, 2], &[0, 1, 2], &[Some(1.0), Some(2.0), Some(3.0)], &[Some(4.0), Some(5.0), Some(6.0)]));
    // A forced position longer than the toolhead position.
    compare(&mut py, "forcepos too long", &base(), &home_rails(&[0], &[0], &[Some(1.0), None, None, None, Some(9.0)], &moves));
    // A forced position shorter than three entries.
    compare(&mut py, "forcepos too short", &base(), &home_rails(&[0], &[0], &[Some(1.0), None], &moves));
}

fn spec_all() -> Spec {
    let mut spec = dual_z();
    spec.halt_frac = vec![0.5];
    spec
}

#[test]
fn multi_complete_follows_python_semantics() {
    let Some(mut py) = oracle() else { return };
    let spec = base();
    let cases: Vec<(&str, Vec<(&'static str, f64)>)> = vec![
        ("empty", vec![]),
        ("single ok", vec![("ok", 0.002)]),
        ("single fail", vec![("fail", 0.002)]),
        ("single never", vec![("never", 0.0)]),
        ("two ok", vec![("ok", 0.002), ("ok", 0.004)]),
        ("ok and never", vec![("ok", 0.002), ("never", 0.0)]),
        ("fail and never", vec![("never", 0.0), ("fail", 0.002)]),
        ("fail then ok", vec![("fail", 0.002), ("ok", 0.006)]),
        ("three mixed", vec![("ok", 0.002), ("ok", 0.003), ("fail", 0.004)]),
        ("three ok", vec![("ok", 0.004), ("ok", 0.002), ("ok", 0.003)]),
    ];
    for (label, comps) in cases {
        compare(&mut py, label, &spec, &Action::Multi { comps, wait: 0.05 });
    }
}

#[test]
fn manual_home_returns_halt_positions_and_checks_shutdown() {
    let Some(mut py) = oracle() else { return };
    let spec = base();
    let pos = vec![Some(100.0), Some(0.0), Some(0.0), Some(0.0)];
    let action = |probe_pos: bool, triggered: bool, check_triggered: bool| Action::ManualHome {
        rail: 0,
        pos: pos.clone(),
        speed: 20.0,
        probe_pos,
        triggered,
        check_triggered,
    };
    compare(&mut py, "manual_home plain", &spec, &action(false, true, true));
    compare(&mut py, "manual_home probe_pos", &spec, &action(true, true, true));
    compare(&mut py, "manual_home untriggered", &spec, &action(false, false, true));
    // No trigger with and without check_triggered.
    let mut no_trigger = base();
    no_trigger.rails[0].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "manual_home no trigger checked", &no_trigger, &action(false, true, true));
    compare(&mut py, "manual_home no trigger unchecked", &no_trigger, &action(false, true, false));
    no_trigger.shutdown = true;
    compare(&mut py, "manual_home shutdown", &no_trigger, &action(false, true, true));
    // A non-command error is not rewritten by the shutdown check.
    let mut mcu = base();
    mcu.rails[0].end.waits = vec![Wait::Mcu("mcu lost")];
    mcu.shutdown = true;
    compare(&mut py, "manual_home mcu error in shutdown", &mcu, &action(false, true, true));
    // A position with an unset entry raises while the endstop rate is computed.
    let mut none_pos = action(false, true, true);
    if let Action::ManualHome { pos, .. } = &mut none_pos {
        pos[1] = None;
    }
    compare(&mut py, "manual_home position with None", &spec, &none_pos);
    // ... and without endstops it raises when the move is created.
    let mut detached = base();
    detached.rails[0].end.attached = false;
    compare(&mut py, "manual_home position with None and no endstop", &detached, &none_pos);
}

#[test]
fn probing_move_checks_movement_and_skew() {
    let Some(mut py) = oracle() else { return };
    let action = |check_movement: bool| Action::ProbingMove {
        rail: 2,
        pos: vec![Some(10.0), Some(20.0), Some(-5.0), Some(4.0)],
        speed: 5.0,
        check_movement,
    };
    let spec = base();
    compare(&mut py, "probing_move", &spec, &action(true));
    // The probe trigger differs from the halt position, so the halt position is computed.
    let mut over = base();
    over.lag = vec![("stepper_z", vec![0])];
    compare(&mut py, "probing_move trigger equals halt", &over, &action(true));
    // Probe triggered before the toolhead moved.
    let mut still = base();
    still.halt_frac = vec![0.0];
    still.lag = vec![("stepper_z", vec![0])];
    compare(&mut py, "probing_move prior to movement", &still, &action(true));
    compare(&mut py, "probing_move prior to movement unchecked", &still, &action(false));
    still.debuginput = true;
    compare(&mut py, "probing_move prior to movement with debuginput", &still, &action(true));
    // Failure paths.
    let mut fail = base();
    fail.rails[2].end.waits = vec![Wait::Time(0.0)];
    compare(&mut py, "probing_move no trigger", &fail, &action(true));
    fail.shutdown = true;
    compare(&mut py, "probing_move shutdown", &fail, &action(true));
    // A probe without steppers is dropped from the move.
    let mut detached = base();
    detached.rails[2].end.attached = false;
    compare(&mut py, "probing_move endstop without steppers", &detached, &action(true));
    // Dual-stepper probe rail.
    compare(&mut py, "probing_move dual steppers", &dual_z(), &action(true));
}

#[test]
fn z_homing_through_the_probe() {
    let Some(mut py) = oracle() else { return };
    let spec = probe_z();
    compare(&mut py, "z via probe", &spec, &g28(&[("Z", "")]));
    compare(&mut py, "all axes with probe z", &spec, &g28(&[]));
    // Without a second pass.
    let mut single = probe_z();
    single.rails[2].retract_dist = 0.0;
    compare(&mut py, "z via probe without retract", &single, &g28(&[("Z", "")]));
    // Failures of the probe session leave it open.
    let mut failing = probe_z();
    failing.probe_err = vec![None, Some("Probe failed")];
    compare(&mut py, "probe error on second run", &failing, &g28(&[("Z", "")]));
    failing.probe_err = vec![Some("Probe failed")];
    compare(&mut py, "probe error on first run", &failing, &g28(&[("Z", "")]));
    // A probe session that returns no result.
    let mut empty = probe_z();
    empty.probe_empty = true;
    compare(&mut py, "probe without results", &empty, &g28(&[("Z", "")]));
    // Missing probe object.
    let mut missing = probe_z();
    missing.probe = false;
    compare(&mut py, "missing probe", &missing, &g28(&[("Z", "")]));
    // Failures through G28 turn the motors off.
    let mut error = probe_z();
    error.probe_err = vec![Some("Probe failed")];
    error.shutdown = true;
    compare(&mut py, "probe error during shutdown", &error, &g28(&[("Z", "")]));
    // Direct requests.
    let force = [None, None, Some(5.0), None];
    let target = [None, None, Some(0.0), None];
    compare(&mut py, "probe z home_rails", &spec, &home_rails(&[2], &[2], &force, &target));
    compare(&mut py, "probe z with xy target", &spec, &home_rails(&[2], &[2], &force, &[Some(1.0), None, Some(0.0), None]));
    compare(&mut py, "probe z with y target", &spec, &home_rails(&[2], &[2], &force, &[None, Some(1.0), Some(0.0), None]));
    compare(&mut py, "probe z short target", &spec, &home_rails(&[2], &[2], &force, &[None]));
    compare(&mut py, "mix of probe and plain rails", &spec, &home_rails(&[0, 2], &[0, 2], &[Some(0.0), None, Some(5.0), None], &[Some(1.0), None, Some(0.0), None]));
    compare(&mut py, "plain rail before probe rail", &spec, &home_rails(&[2, 0], &[2, 0], &[Some(0.0), None, Some(5.0), None], &[Some(1.0), None, Some(0.0), None]));
}

#[test]
fn check_probe_first_home_matches_command_and_attempt() {
    let Some(mut py) = oracle() else { return };
    let spec = base();
    for (label, command, attempt) in [
        ("G28 attempt 1", "G28", Some("1")),
        ("G28 attempt 2", "G28", Some("2")),
        ("G28 attempt 10", "G28", Some("10")),
        ("G28 no attempt", "G28", None),
        ("G28 empty attempt", "G28", Some("")),
        ("PROBE attempt 1", "PROBE", Some("1")),
        ("PROBE no attempt", "PROBE", None),
        ("g28 lower case", "g28", Some("1")),
    ] {
        compare(&mut py, label, &spec, &Action::CheckFirst { command, attempt });
    }
}

#[test]
fn registration_and_load_config() {
    let Some(mut py) = oracle() else { return };
    // `register_command('G28', ...)` without options appears at the top of every trace.
    compare(&mut py, "registration", &base(), &Action::CheckFirst { command: "G28", attempt: Some("1") });
    // The module factory builds the printer object.
    let factory = extras::homing::module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    let reactor = Reactor::new(false);
    let env = build(base(), reactor);
    let printer: Printer = env.printer.clone();
    let config: Config = Arc::new(FakeConfig { printer });
    let obj = (factory.load_config.expect("load_config"))(&config)
        .expect("load_config succeeds")
        .expect("homing object");
    assert!(klippy_api::interface::<dyn klippy_api::toolhead::PrinterHomingApi>(&obj).is_some());
    assert_eq!(
        env.world.trace.cloned(),
        vec!["register_command G28 when_not_ready=False desc=None".to_string()]
    );
}
