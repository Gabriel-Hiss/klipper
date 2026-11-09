//! Parity tests for `kinematics::hybrid_corexy` (port of
//! `klippy/kinematics/hybrid_corexy.py`).
//!
//! Every expected value comes from the live reference: `pyoracle` runs
//! CPython 3.8.2 with the frozen `klippy/` importable, `kinematics.hybrid_corexy`
//! is the real module, and its collaborators are fakes installed on the Python
//! side (`stepper`, the printer, the toolhead, the config wrapper). The Rust
//! side runs the port against fakes of the same contract traits.
//!
//! Compared: the exact config reads (option, default, bounds), the rail and
//! stepper wiring order, the itersolve allocations, the trapq assignment, the
//! `DualCarriages` construction and registration (a real `DualCarriages` on
//! both sides), `calc_position`, `set_position`/limits, `clear_homing_state`,
//! `home`/`home_rails` arguments, `check_move` limits and errors, and the
//! `get_status` dict.
//!
//! Run: `cargo test -p kinematics --test hybrid_corexy`.

use std::any::Any;
use std::sync::Arc;

use indexmap::IndexMap;
use kos::sync::Mutex;
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, Value as Rv};

use kinematics::hybrid_corexy::{build, module_factory, HybridCoreXYKinematics};
use kinematics::idex_modes::{DualCarriages, PRIMARY};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::mcu::{Endstop, EndstopApi, Mcu};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, ExtraAxis, HomingStateApi, HomingStateHandle, Kinematics, LookaheadCallback, MoveApi,
    PrinterExtruder, ToolHead, ToolHeadApi,
};
use klippy_api::{KResult, KlippyError, PrinterObject};
use klippy_core::motion_handles::alloc_stepper_kinematics;

// ---------------------------------------------------------------------------
// Python-side fakes around the real `kinematics.hybrid_corexy`.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import sys
from types import ModuleType
import collections

class Error(Exception):
    pass

class CommandError(Exception):
    pass

_sentinel = object()
LOG = []
RAILS = {}

def _of(v):
    return 'None' if v is None else repr(v)

def _dflt(v):
    return '__sentinel__' if v is _sentinel else repr(v)

class _Sk:
    pass

class _Ffi:
    def gc(self, obj, destructor):
        return obj

class _Lib:
    def dual_carriage_alloc(self):
        return _Sk()
    def dual_carriage_set_sk(self, sk, orig_sk):
        sk.orig_sk = orig_sk
    def dual_carriage_set_transform(self, sk, axis, scale, offs):
        return 0
    def free(self, sk):
        pass

_chelper_mod = ModuleType('chelper')
_chelper_mod.get_ffi = lambda: (_Ffi(), _Lib())
sys.modules['chelper'] = _chelper_mod

HomingInfo = collections.namedtuple('homing_info', [
    'speed', 'position_endstop', 'retract_speed', 'retract_dist',
    'positive_dir', 'second_homing_speed'])

SPECTS = {}
STORE = {}
ERRORS = {}
SECTIONS = set()

def Coord(t):
    t = tuple(t)
    if len(t) < 4:
        t = t + (0,) * (4 - len(t))
    return t

class FakeStepper:
    def __init__(self, name):
        self._name = name
        self.sk = _Sk()
    def get_name(self, short=False):
        return self._name
    def get_stepper_kinematics(self):
        return self.sk
    def set_stepper_kinematics(self, sk):
        old = self.sk
        self.sk = sk
        return old
    def set_trapq(self, tq):
        LOG.append(['set_trapq', self._name])

class FakeEndstop:
    def __init__(self, rail_name):
        self._rail_name = rail_name
    def add_stepper(self, s):
        LOG.append(['endstop_add', self._rail_name, s.get_name()])

class FakeRail:
    def __init__(self, name):
        spec = SPECTS[name]
        self._name = name
        self.position_min = spec[0]
        self.position_max = spec[1]
        self._speed = spec[2]
        self._endstop = spec[3]
        self._positive_dir = spec[4]
        self.steppers = [FakeStepper(name if i == 0 else '%s_%d' % (name, i))
                         for i in range(spec[5])]
        self.endstops = [(FakeEndstop(name), name)]
    def get_name(self, short=False):
        return self._name
    def get_range(self):
        return (self.position_min, self.position_max)
    def get_homing_info(self):
        return HomingInfo(self._speed, self._endstop, 0.0, 0.0,
                          self._positive_dir, 0.0)
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def setup_itersolve(self, alloc_func, *params):
        LOG.append(['itersolve', self._name, alloc_func,
                    params[0].decode() if params else ''])
    def set_position(self, coord):
        LOG.append(['set_position', self._name,
                    ','.join(repr(v) for v in list(coord)[:3])])

def LookupRail(config):
    name = config.get_name()
    LOG.append(['lookup_rail', name])
    rail = FakeRail(name)
    RAILS[name] = rail
    return rail

def LookupMultiRail(config):
    name = config.get_name()
    LOG.append(['lookup_multi_rail', name])
    rail = FakeRail(name)
    RAILS[name] = rail
    return rail

_fake_stepper = ModuleType('stepper')
_fake_stepper.LookupRail = LookupRail
_fake_stepper.LookupMultiRail = LookupMultiRail
sys.modules['stepper'] = _fake_stepper

class FakePrinter:
    config_error = Error
    def __init__(self):
        self.objects = {}
    def add_object(self, name, obj):
        LOG.append(['add_object', name])
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        LOG.append(['register_event_handler', event])
    def send_event(self, event, *params):
        return []

class FakeGCode:
    def __init__(self):
        self.commands = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        LOG.append(['register_command', cmd, 'None' if desc is None else desc])
        self.commands[cmd] = func

class FakeSection:
    def __init__(self, section, printer):
        self.section = section
        self.printer = printer
    def get_name(self):
        return self.section
    def get_printer(self):
        return self.printer
    def getsection(self, sec):
        return FakeSection(sec, self.printer)
    def has_section(self, sec):
        present = sec in SECTIONS
        LOG.append(['has_section', sec, str(present)])
        return present
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        LOG.append(['getfloat', self.section, option, _dflt(default),
                    _of(minval), _of(maxval), _of(above), _of(below)])
        err = ERRORS.get(self.section, {}).get(option)
        if err is not None:
            raise Error(err)
        store = STORE.get(self.section, {})
        if option in store:
            return float(store[option])
        if default is _sentinel:
            raise Error("Option '%s' in section '%s' is not a float"
                        % (option, self.section))
        return default
    def getchoice(self, option, choices, default=_sentinel, note_valid=True):
        LOG.append(['getchoice', self.section, option, repr(list(choices)),
                    _dflt(default)])
        err = ERRORS.get(self.section, {}).get(option)
        if err is not None:
            raise Error(err)
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        store = STORE.get(self.section, {})
        if option in store:
            c = store[option]
        elif default is not _sentinel:
            c = default
        else:
            raise Error("Option '%s' in section '%s' is not a string"
                        % (option, self.section))
        if c not in choices:
            raise Error("Choice '%s' for option '%s' in section '%s'"
                        " is not a valid choice" % (c, option, self.section))
        return choices[c]

class FakeToolhead:
    def __init__(self, max_velocity, max_accel):
        self._max = (max_velocity, max_accel)
        self.Coord = Coord
        self.kin = None
        self.position = [0.0, 0.0, 0.0, 0.0]
        self.flushes = 0
    def get_trapq(self):
        return 'TQ'
    def get_max_velocity(self):
        return self._max
    def get_kinematics(self):
        return self.kin
    def get_position(self):
        return list(self.position)
    def set_position(self, newpos, homing_axes=''):
        self.position = list(newpos)
    def flush_step_generation(self):
        self.flushes += 1

class FakeMove:
    def __init__(self, end_pos, axes_d, move_d):
        self.end_pos = Coord(end_pos)
        self.axes_d = Coord(axes_d)
        self.move_d = move_d
        self.limit_calls = []
    def limit_speed(self, speed, accel):
        self.limit_calls.append((speed, accel))
    def move_error(self, msg='Move out of range'):
        ep = self.end_pos
        return CommandError('%s: %.3f %.3f %.3f [%.3f]'
                            % (msg, ep[0], ep[1], ep[2], ep[3]))

class FakeHoming:
    def __init__(self, axes):
        self._axes = list(axes)
        self.home_rails_calls = []
    def get_axes(self):
        return list(self._axes)
    def home_rails(self, rails, forcepos, movepos):
        self.home_rails_calls.append(
            ([r.get_name() for r in rails], list(forcepos), list(movepos)))

def build(kin_builder):
    global LOG
    LOG = []
    RAILS.clear()
    printer = FakePrinter()
    gcode = FakeGCode()
    toolhead = FakeToolhead(500.0, 3000.0)
    printer.add_object('gcode', gcode)
    printer.add_object('toolhead', toolhead)
    config = FakeSection('printer', printer)
    kin = kin_builder(toolhead, config)
    toolhead.kin = kin
    return kin, toolhead, printer, gcode

import kinematics.hybrid_corexy as hybrid_corexy
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Trace = Mutex<Vec<Vec<String>>>;

fn log(trace: &Trace, parts: &[&str]) {
    trace
        .lock()
        .push(parts.iter().map(|p| p.to_string()).collect());
}

fn num(v: f64) -> String {
    format!("{v:?}")
}

fn opt_num(v: Option<f64>) -> String {
    match v {
        Some(v) => num(v),
        None => "None".to_string(),
    }
}

fn opt_default(v: &Option<Option<f64>>) -> String {
    match v {
        None => "__sentinel__".to_string(),
        Some(v) => opt_num(*v),
    }
}

struct FakeStepper {
    name: String,
    sk: GCell<Option<StepperKinematics>>,
    trace: Arc<Trace>,
}

impl FakeStepper {
    fn new(name: &str, trace: Arc<Trace>) -> FakeStepper {
        FakeStepper {
            name: name.to_string(),
            sk: GCell::new(Some(
                alloc_stepper_kinematics(
                    "cartesian_stepper_alloc",
                    &[PyValue::Bytes(vec![b'y'])],
                )
                .expect("cartesian stepper kinematics"),
            )),
            trace,
        }
    }
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by hybrid_corexy")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        0
    }
    fn units_in_radians(&self) -> bool {
        false
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        (None, false)
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is called on the rail, not the stepper")
    }
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _coord: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        mcu_pos as f64
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        self.sk.cloned()
    }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        self.sk.replace(sk)
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        log(&self.trace, &["set_trapq", &self.name]);
        tq
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeEndstop {
    rail_name: String,
    trace: Arc<Trace>,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by hybrid_corexy")
    }
    fn add_stepper(&self, stepper: Stepper) {
        log(
            &self.trace,
            &["endstop_add", &self.rail_name, &stepper.get_name(None)],
        );
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        panic!("home_start is not used by hybrid_corexy")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        Ok(0)
    }
}

#[derive(Clone)]
struct RailSpec {
    min: f64,
    max: f64,
    speed: f64,
    endstop: f64,
    positive: bool,
    steppers: usize,
}

struct FakeRail {
    name: String,
    spec: RailSpec,
    steppers: Vec<Stepper>,
    endstops: Vec<(Endstop, String)>,
    trace: Arc<Trace>,
}

impl FakeRail {
    fn new(name: &str, spec: &RailSpec, trace: Arc<Trace>) -> FakeRail {
        let mut steppers = Vec::new();
        for i in 0..spec.steppers {
            let stepper_name = if i == 0 {
                name.to_string()
            } else {
                format!("{name}_{i}")
            };
            steppers.push(Arc::new(FakeStepper::new(&stepper_name, trace.clone())) as Stepper);
        }
        let endstop: Endstop = Arc::new(FakeEndstop {
            rail_name: name.to_string(),
            trace: trace.clone(),
        });
        FakeRail {
            name: name.to_string(),
            spec: spec.clone(),
            steppers,
            endstops: vec![(endstop, name.to_string())],
            trace,
        }
    }
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_range(&self) -> (f64, f64) {
        (self.spec.min, self.spec.max)
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: self.spec.speed,
            position_endstop: self.spec.endstop,
            retract_speed: 0.0,
            retract_dist: 0.0,
            positive_dir: self.spec.positive,
            second_homing_speed: 0.0,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn get_endstops(&self) -> Vec<(Endstop, String)> {
        self.endstops.clone()
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Endstop> {
        Err(KlippyError::config("fake rail has no endstop pin"))
    }
    fn add_stepper(
        &self,
        _stepper: Stepper,
        _endstop_pin: Option<&str>,
        _endstop_name: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        Ok(())
    }
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        let param = match params.first() {
            Some(PyValue::Bytes(bytes)) => String::from_utf8_lossy(bytes).into_owned(),
            Some(_) => panic!("unexpected itersolve parameter"),
            None => String::new(),
        };
        log(&self.trace, &["itersolve", &self.name, alloc_func, &param]);
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {}
    fn set_position(&self, coord: [f64; 3]) {
        let joined = format!("{},{},{}", num(coord[0]), num(coord[1]), num(coord[2]));
        log(&self.trace, &["set_position", &self.name, &joined]);
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        0.0
    }
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
}

struct FakeTrapq;

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct FakeFactory {
    rails: IndexMap<String, RailSpec>,
    trace: Arc<Trace>,
}

impl FakeFactory {
    fn make(&self, name: &str) -> KResult<Rail> {
        let spec = self
            .rails
            .get(name)
            .ok_or_else(|| KlippyError::config(format!("no fake rail for section '{name}'")))?;
        Ok(Arc::new(FakeRail::new(name, spec, self.trace.clone())) as Rail)
    }
}

impl StepperFactoryApi for FakeFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper is not used by hybrid_corexy")
    }
    fn lookup_rail(&self, config: &Config, _opts: RailOpts) -> KResult<Rail> {
        let name = config.get_name();
        log(&self.trace, &["lookup_rail", &name]);
        self.make(&name)
    }
    fn lookup_multi_rail(&self, config: &Config, _opts: RailOpts) -> KResult<Rail> {
        let name = config.get_name();
        log(&self.trace, &["lookup_multi_rail", &name]);
        self.make(&name)
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail is not used by hybrid_corexy")
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        panic!("parse_step_distance is not used by hybrid_corexy")
    }
}

struct ConfigShared {
    values: IndexMap<(String, String), PyValue>,
    errors: IndexMap<(String, String), String>,
    sections: Vec<String>,
    trace: Arc<Trace>,
}

struct FakeConfig {
    section: String,
    printer: Printer,
    shared: Arc<ConfigShared>,
}

impl FakeConfig {
    fn key(&self, option: &str) -> (String, String) {
        (self.section.clone(), option.to_string())
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, _option: &str, _opts: GetStrOpts) -> KResult<Option<String>> {
        Ok(None)
    }
    fn getint(&self, _option: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        Ok(None)
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        log(
            &self.shared.trace,
            &[
                "getfloat",
                &self.section,
                option,
                &opt_default(&opts.default),
                &opt_num(opts.minval),
                &opt_num(opts.maxval),
                &opt_num(opts.above),
                &opt_num(opts.below),
            ],
        );
        if let Some(message) = self.shared.errors.get(&self.key(option)) {
            return Err(KlippyError::Config(message.clone()));
        }
        if let Some(value) = self.shared.values.get(&self.key(option)) {
            return match value {
                PyValue::Float(v) => Ok(Some(*v)),
                other => panic!("fake config value is not a float: {other:?}"),
            };
        }
        match opts.default {
            None => Err(KlippyError::Config(format!(
                "Option '{}' in section '{}' is not a float",
                option, self.section
            ))),
            Some(default) => Ok(default),
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let default = match &opts.default {
            None => "__sentinel__".to_string(),
            Some(None) => "None".to_string(),
            Some(Some(v)) => v.repr(),
        };
        log(
            &self.shared.trace,
            &["getchoice", &self.section, option, &choices.repr(), &default],
        );
        if let Some(message) = self.shared.errors.get(&self.key(option)) {
            return Err(KlippyError::Config(message.clone()));
        }
        let choice_list: Vec<PyValue> = match choices {
            PyValue::List(items) => items.clone(),
            PyValue::Tuple(items) => items.clone(),
            PyValue::Dict(map) => map.keys().cloned().collect(),
            other => panic!("unsupported choices value: {other:?}"),
        };
        let value = match self.shared.values.get(&self.key(option)) {
            Some(v) => Some(v.clone()),
            None => match &opts.default {
                None => {
                    return Err(KlippyError::Config(format!(
                        "Option '{}' in section '{}' is not a string",
                        option, self.section
                    )))
                }
                Some(default) => default.clone(),
            },
        };
        match value {
            Some(v) => {
                for choice in &choice_list {
                    if *choice == v {
                        return Ok(choice.clone());
                    }
                }
                Err(KlippyError::Config(format!(
                    "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                    v.str(),
                    option,
                    self.section
                )))
            }
            None => Err(KlippyError::Config(format!(
                "Choice 'None' for option '{}' in section '{}' is not a valid choice",
                option, self.section
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            section: section.to_string(),
            printer: self.printer.clone(),
            shared: self.shared.clone(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        let present = self.shared.sections.iter().any(|s| s == section);
        log(
            &self.shared.trace,
            &["has_section", section, if present { "True" } else { "False" }],
        );
        present
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct FakeGCode {
    trace: Arc<Trace>,
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        log(
            &self.trace,
            &[
                "register_command",
                cmd,
                &opts.desc.clone().unwrap_or_else(|| "None".to_string()),
            ],
        );
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
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by hybrid_corexy")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by hybrid_corexy")
    }
    fn respond_raw(&self, _msg: &str) {}
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

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<String>>,
    trace: Arc<Trace>,
}

impl FakePrinter {
    fn new(trace: Arc<Trace>) -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            trace,
        }
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
        log(&self.trace, &["add_object", name]);
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        log(&self.trace, &["register_event_handler", event]);
        self.handlers.lock().push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeToolHead {
    max_velocity: f64,
    max_accel: f64,
    kin: GCell<Option<Kinematics>>,
    position: GCell<Vec<f64>>,
    flushes: Mutex<i64>,
    factory: StepperFactory,
}

impl FakeToolHead {
    fn new(max_velocity: f64, max_accel: f64, factory: StepperFactory) -> FakeToolHead {
        FakeToolHead {
            max_velocity,
            max_accel,
            kin: GCell::new(None),
            position: GCell::new(vec![0.0, 0.0, 0.0, 0.0]),
            flushes: Mutex::new(0),
            factory,
        }
    }

    fn set_kin(&self, kin: Kinematics) {
        self.kin.replace(Some(kin));
    }
}

impl PrinterObject for FakeToolHead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn ToolHeadApi);
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        self.position.replace(newpos);
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        let mut pos = self.position.cloned();
        for (i, v) in coord.iter().enumerate() {
            if let Some(v) = v {
                pos[i] = *v;
            }
        }
        self.position.replace(pos);
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (self.max_velocity, self.max_accel)
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
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by hybrid_corexy")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {}
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        self.position.cloned()
    }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        self.position.replace(newpos);
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        self.kin
            .cloned()
            .expect("toolhead kinematics installed by the test")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flushes.lock() += 1;
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        0.0
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

type HomeCalls = Vec<(Vec<String>, Vec<Option<f64>>, Vec<Option<f64>>)>;

struct FakeHoming {
    axes: Vec<i64>,
    calls: Mutex<HomeCalls>,
}

impl FakeHoming {
    fn new(axes: Vec<i64>) -> FakeHoming {
        FakeHoming {
            axes,
            calls: Mutex::new(Vec::new()),
        }
    }
}

impl HomingStateApi for FakeHoming {
    fn set_axes(&self, _axes: Vec<i64>) {}
    fn get_axes(&self) -> Vec<i64> {
        self.axes.clone()
    }
    fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: pycompat::PyValue::Str(stepper_name.to_string()).repr(),
        })
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        Ok(())
    }
    fn home_rails(
        &self,
        rails: Vec<Rail>,
        forcepos: Vec<Option<f64>>,
        movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        self.calls.lock().push((
            rails.iter().map(|r| r.get_name(None)).collect(),
            forcepos,
            movepos,
        ));
        Ok(())
    }
}

#[derive(Clone)]
struct FakeMove {
    end_pos: Coord,
    axes_d: Coord,
    move_d: f64,
    limit_calls: Arc<Mutex<Vec<(f64, f64)>>>,
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord {
        Coord(vec![0.0, 0.0, 0.0, 0.0])
    }
    fn end_pos(&self) -> Coord {
        self.end_pos.clone()
    }
    fn axes_d(&self) -> Coord {
        self.axes_d.clone()
    }
    fn axes_r(&self) -> Coord {
        Coord(vec![0.0, 0.0, 0.0, 0.0])
    }
    fn move_d(&self) -> f64 {
        self.move_d
    }
    fn is_kinematic_move(&self) -> bool {
        true
    }
    fn accel(&self) -> f64 {
        0.0
    }
    fn max_cruise_v2(&self) -> f64 {
        0.0
    }
    fn start_v(&self) -> f64 {
        0.0
    }
    fn cruise_v(&self) -> f64 {
        0.0
    }
    fn accel_t(&self) -> f64 {
        0.0
    }
    fn cruise_t(&self) -> f64 {
        0.0
    }
    fn decel_t(&self) -> f64 {
        0.0
    }
    fn limit_speed(&self, speed: f64, accel: f64) {
        self.limit_calls.lock().push((speed, accel));
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos.0;
        let text = pycompat::text::percent_format(
            "%s: %.3f %.3f %.3f [%.3f]",
            &[
                pycompat::text::PctValue::Str(msg),
                pycompat::text::PctValue::Float(ep.first().copied().unwrap_or(0.0)),
                pycompat::text::PctValue::Float(ep.get(1).copied().unwrap_or(0.0)),
                pycompat::text::PctValue::Float(ep.get(2).copied().unwrap_or(0.0)),
                pycompat::text::PctValue::Float(ep.get(3).copied().unwrap_or(0.0)),
            ],
        )
        .expect("percent format");
        KlippyError::Command(text)
    }
}

// ---------------------------------------------------------------------------
// Scenario description shared by both sides.
// ---------------------------------------------------------------------------

struct World {
    rails: Vec<(&'static str, RailSpec)>,
    values: Vec<(&'static str, Vec<(&'static str, PyValue)>)>,
    errors: Vec<(&'static str, Vec<(&'static str, &'static str)>)>,
}

fn spec(min: f64, max: f64, endstop: f64, positive: bool) -> RailSpec {
    RailSpec {
        min,
        max,
        speed: 5.0,
        endstop,
        positive,
        steppers: 1,
    }
}

/// No `[dual_carriage]`; Y carries two steppers so the multi rail is covered.
fn world_plain() -> World {
    let mut y = spec(0.0, 200.0, 0.0, false);
    y.steppers = 2;
    World {
        rails: vec![
            ("stepper_x", spec(0.0, 100.0, 0.0, false)),
            ("stepper_y", y),
            ("stepper_z", spec(0.0, 300.0, 0.0, true)),
        ],
        values: vec![(
            "printer",
            vec![
                ("max_z_velocity", PyValue::Float(25.0)),
                ("max_z_accel", PyValue::Float(200.0)),
            ],
        )],
        errors: Vec::new(),
    }
}

/// No `max_z_velocity` / `max_z_accel`: the toolhead defaults are used.
fn world_defaults() -> World {
    let mut w = world_plain();
    w.values.clear();
    w
}

/// The hybrid IDEX layout: a fourth `corexy_stepper_alloc '+'` rail.
fn world_dual() -> World {
    let mut w = world_plain();
    w.rails
        .push(("dual_carriage", spec(0.0, 250.0, 0.0, true)));
    w.values = vec![
        (
            "printer",
            vec![
                ("max_z_velocity", PyValue::Float(25.0)),
                ("max_z_accel", PyValue::Float(200.0)),
            ],
        ),
        (
            "dual_carriage",
            vec![
                ("axis", PyValue::Str("x".to_string())),
                ("safe_distance", PyValue::Float(10.0)),
            ],
        ),
    ];
    w
}

impl World {
    fn sections(&self) -> Vec<String> {
        let mut sections = vec!["printer".to_string()];
        for (name, _) in &self.rails {
            sections.push((*name).to_string());
        }
        for (name, _) in &self.values {
            if !sections.iter().any(|s| s == name) {
                sections.push((*name).to_string());
            }
        }
        for (name, _) in &self.errors {
            if !sections.iter().any(|s| s == name) {
                sections.push((*name).to_string());
            }
        }
        sections
    }

    fn py_state(&self) -> String {
        let mut out = String::new();
        out.push_str("SPECTS.clear()\nSTORE.clear()\nERRORS.clear()\nSECTIONS.clear()\n");
        out.push_str("SPECTS.update({\n");
        for (name, spec) in &self.rails {
            out.push_str(&format!(
                "    {name:?}: ({}, {}, {}, {}, {}, {}),\n",
                num(spec.min),
                num(spec.max),
                num(spec.speed),
                num(spec.endstop),
                if spec.positive { "True" } else { "False" },
                spec.steppers
            ));
        }
        out.push_str("})\nSTORE.update({\n");
        for (section, options) in &self.values {
            let entries: Vec<String> = options
                .iter()
                .map(|(option, value)| format!("{option:?}: {}", py_literal(value)))
                .collect();
            out.push_str(&format!("    {section:?}: {{{}}},\n", entries.join(", ")));
        }
        out.push_str("})\nERRORS.update({\n");
        for (section, options) in &self.errors {
            let entries: Vec<String> = options
                .iter()
                .map(|(option, message)| format!("{option:?}: {message:?}"))
                .collect();
            out.push_str(&format!("    {section:?}: {{{}}},\n", entries.join(", ")));
        }
        out.push_str("})\nSECTIONS.update({");
        let names: Vec<String> = self
            .sections()
            .iter()
            .map(|s| format!("{s:?}"))
            .collect();
        out.push_str(&names.join(", "));
        out.push_str("})\n");
        out
    }

    fn rust_shared(&self, trace: Arc<Trace>) -> Arc<ConfigShared> {
        let mut shared = ConfigShared {
            values: IndexMap::new(),
            errors: IndexMap::new(),
            sections: self.sections(),
            trace,
        };
        for (section, options) in &self.values {
            for (option, value) in options {
                shared
                    .values
                    .insert(((*section).to_string(), (*option).to_string()), value.clone());
            }
        }
        for (section, options) in &self.errors {
            for (option, message) in options {
                shared
                    .errors
                    .insert(((*section).to_string(), (*option).to_string()), (*message).to_string());
            }
        }
        Arc::new(shared)
    }

    fn rust_rails(&self) -> IndexMap<String, RailSpec> {
        self.rails
            .iter()
            .map(|(name, spec)| ((*name).to_string(), spec.clone()))
            .collect()
    }
}

fn py_literal(value: &PyValue) -> String {
    match value {
        PyValue::Float(v) => num(*v),
        PyValue::Str(s) => format!("{s:?}"),
        other => panic!("unsupported config value {other:?}"),
    }
}

/// Rust rig for one scenario.
struct Rig {
    kin: Arc<HybridCoreXYKinematics>,
    kin_api: Kinematics,
    toolhead: Arc<FakeToolHead>,
    trace: Arc<Trace>,
}

fn rust_build(world: &World) -> KResult<Rig> {
    let trace: Arc<Trace> = Arc::new(Mutex::new(Vec::new()));
    let printer = Arc::new(FakePrinter::new(trace.clone()));
    let printer_api: Printer = printer.clone();
    let gcode = Arc::new(FakeGCode {
        trace: trace.clone(),
    });
    let gcode_obj: ObjectRef = gcode;
    printer.add_object("gcode", gcode_obj)?;
    let factory: StepperFactory = Arc::new(FakeFactory {
        rails: world.rust_rails(),
        trace: trace.clone(),
    });
    let toolhead = Arc::new(FakeToolHead::new(500.0, 3000.0, factory));
    let toolhead_obj: ObjectRef = toolhead.clone();
    printer.add_object("toolhead", toolhead_obj)?;
    let config: Config = Arc::new(FakeConfig {
        section: "printer".to_string(),
        printer: printer_api,
        shared: world.rust_shared(trace.clone()),
    });
    let toolhead_api: ToolHead = toolhead.clone();
    let kin = build(&toolhead_api, &config)?;
    let kin_api: Kinematics = kin.clone();
    toolhead.set_kin(kin_api.clone());
    Ok(Rig {
        kin,
        kin_api,
        toolhead,
        trace,
    })
}

// ---------------------------------------------------------------------------
// Recording helpers.
// ---------------------------------------------------------------------------

fn record_for(seq: u64, label: &str) -> Record {
    Record {
        v: 1,
        unit: "kinematics.hybrid_corexy".to_string(),
        scenario: "parity".to_string(),
        seq,
        function: label.to_string(),
        args: vec![],
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

fn check_equal(seq: u64, label: &str, expected: &Rv, actual: &Rv) {
    assert_equal(&record_for(seq, label), "ret", expected, actual)
        .unwrap_or_else(|d| panic!("{label}: {d}"));
}

fn check_err(seq: u64, label: &str, expected: &recording::Exception, actual: &KlippyError) {
    assert_eq!(
        actual.class_name(),
        expected.class_name,
        "seq {seq} {label}: class"
    );
    assert_eq!(
        actual.message(),
        expected.message,
        "seq {seq} {label}: message"
    );
}

fn floats_to_rec(values: &[Option<f64>]) -> Rv {
    Rv::List(
        values
            .iter()
            .map(|v| match v {
                Some(f) => Rv::float(*f),
                None => Rv::Null,
            })
            .collect(),
    )
}

fn opt_list(values: &[Option<f64>]) -> Rv {
    floats_to_rec(values)
}

fn pyvalue_to_rec(value: &PyValue) -> Rv {
    match value {
        PyValue::None => Rv::Null,
        PyValue::Bool(b) => Rv::Bool(*b),
        PyValue::Int(i) => Rv::Int(i.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Rv::float(*f),
        PyValue::Str(s) => Rv::Str(s.clone()),
        PyValue::Bytes(b) => Rv::Bytes(b.clone()),
        PyValue::List(items) => Rv::List(items.iter().map(pyvalue_to_rec).collect()),
        PyValue::Tuple(items) => Rv::Tuple(items.iter().map(pyvalue_to_rec).collect()),
        PyValue::Dict(map) => Rv::Dict(
            map.iter()
                .map(|(k, v)| (pyvalue_to_rec(k), pyvalue_to_rec(v)))
                .collect(),
        ),
        PyValue::Host(_) => Rv::Null,
    }
}

fn trace_value(trace: &Trace) -> Rv {
    Rv::List(
        trace
            .lock()
            .iter()
            .map(|entry| Rv::List(entry.iter().map(|s| Rv::Str(s.clone())).collect()))
            .collect(),
    )
}

fn homing_calls_value(calls: &HomeCalls) -> Rv {
    Rv::List(
        calls
            .iter()
            .map(|(rails, forcepos, homepos)| {
                Rv::Tuple(vec![
                    Rv::List(rails.iter().map(|r| Rv::Str(r.clone())).collect()),
                    opt_list(forcepos),
                    opt_list(homepos),
                ])
            })
            .collect(),
    )
}

fn limit_calls_value(calls: &[(f64, f64)]) -> Rv {
    Rv::List(
        calls
            .iter()
            .map(|(speed, accel)| Rv::Tuple(vec![Rv::float(*speed), Rv::float(*accel)]))
            .collect(),
    )
}

fn limits_value(limits: &[(f64, f64); 3]) -> Rv {
    Rv::List(
        limits
            .iter()
            .map(|(lo, hi)| Rv::Tuple(vec![Rv::float(*lo), Rv::float(*hi)]))
            .collect(),
    )
}

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(PY_SETUP).expect("python fakes");
    Some(py)
}

/// Builds the scenario on both sides. Returns `None` when both sides raised
/// (the exceptions are then compared).
fn build_both(
    seq: u64,
    py: &mut pyoracle::Oracle,
    world: &World,
    label: &str,
) -> Option<Rig> {
    py.exec(&world.py_state()).expect("python scenario state");
    match py.exec("kin, toolhead, printer, gcode = build(hybrid_corexy.load_kinematics)") {
        Ok(_) => match rust_build(world) {
            Ok(rig) => Some(rig),
            Err(actual) => panic!("{label}: python loaded, rust failed: {actual:?}"),
        },
        Err(expected) => match rust_build(world) {
            Err(actual) => {
                check_err(seq, label, &expected, &actual);
                None
            }
            Ok(_) => panic!(
                "{label}: python raised {} {:?}, rust loaded",
                expected.class_name, expected.message
            ),
        },
    }
}

/// The `LOG` trace both sides produced while loading the scenario.
fn compare_trace(seq: u64, py: &mut pyoracle::Oracle, rig: &Rig, label: &str) {
    let expected = py.eval("LOG").expect("python LOG");
    check_equal(seq, label, &expected, &trace_value(&rig.trace));
}

fn compare_status(seq: u64, py: &mut pyoracle::Oracle, rig: &Rig, label: &str) {
    let expected = py.eval("kin.get_status(0.0)").expect("python status");
    let actual = pyvalue_to_rec(&rig.kin_api.get_status(0.0));
    check_equal(seq, label, &expected, &actual);
}

fn compare_calc_position(
    seq: u64,
    py: &mut pyoracle::Oracle,
    rig: &Rig,
    positions: &[(&str, f64)],
    label: &str,
) {
    let py_map: Vec<String> = positions
        .iter()
        .map(|(name, value)| format!("{name:?}: {}", num(*value)))
        .collect();
    let expected = py
        .eval(&format!("kin.calc_position({{{}}})", py_map.join(", ")))
        .expect("python calc_position");
    let mut map = IndexMap::new();
    for (name, value) in positions {
        map.insert((*name).to_string(), *value);
    }
    let actual = floats_to_rec(&rig.kin_api.calc_position(&map).expect("calc_position"));
    check_equal(seq, label, &expected, &actual);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_is_empty() {
    // `hybrid_corexy.py` exposes only `load_kinematics`; the toolhead imports
    // the module by name, there is no printer section.
    let factory = module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn init_wiring_matches_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping init_wiring_matches_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_plain();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics") else {
        panic!("load_kinematics must succeed");
    };
    compare_trace(2, &mut py, &rig, "init trace");
    compare_status(3, &mut py, &rig, "get_status");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        4,
        "limits",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );
    let expected_steppers = py
        .eval("[s.get_name() for s in kin.get_steppers()]")
        .expect("python steppers");
    let actual_steppers = Rv::List(
        rig.kin
            .get_steppers()
            .iter()
            .map(|s| Rv::Str(s.get_name(None)))
            .collect(),
    );
    check_equal(5, "get_steppers", &expected_steppers, &actual_steppers);
    let expected_max = py
        .eval("(kin.max_z_velocity, kin.max_z_accel)")
        .expect("python max z");
    let kin = &rig.kin;
    let actual_max = Rv::Tuple(vec![
        Rv::float(kin.max_z_velocity()),
        Rv::float(kin.max_z_accel()),
    ]);
    check_equal(6, "max_z limits", &expected_max, &actual_max);
    let expected_axes = py
        .eval("(kin.axes_min, kin.axes_max)")
        .expect("python axes");
    let actual_axes = Rv::Tuple(vec![
        pyvalue_to_rec(&py_axes(kin.axes_min())),
        pyvalue_to_rec(&py_axes(kin.axes_max())),
    ]);
    check_equal(7, "axes ranges", &expected_axes, &actual_axes);
}

/// The `get_status` projection of a stored `Coord` (three floats and the
/// int `0` gcode.Coord pads with).
fn py_axes(coord: &Coord) -> PyValue {
    PyValue::Tuple(vec![
        PyValue::Float(coord.0[0]),
        PyValue::Float(coord.0[1]),
        PyValue::Float(coord.0[2]),
        PyValue::Int(pycompat::PyInt::from_i64(0)),
    ])
}

#[test]
fn config_defaults_match_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping config_defaults_match_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_defaults();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics defaults") else {
        panic!("load_kinematics must succeed");
    };
    compare_trace(2, &mut py, &rig, "config read trace");
    let expected_max = py
        .eval("(kin.max_z_velocity, kin.max_z_accel)")
        .expect("python max z");
    let kin = &rig.kin;
    let actual_max = Rv::Tuple(vec![
        Rv::float(kin.max_z_velocity()),
        Rv::float(kin.max_z_accel()),
    ]);
    check_equal(3, "max_z defaults", &expected_max, &actual_max);
}

#[test]
fn config_errors_match_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping config_errors_match_oracle: set KLIPPY_PYTHON");
        return;
    };
    let mut cases: Vec<(&str, World)> = Vec::new();
    let mut wz = world_plain();
    wz.errors = vec![("printer", vec![("max_z_velocity", "bad velocity")])];
    cases.push(("max_z_velocity", wz));
    let mut wa = world_plain();
    wa.errors = vec![("printer", vec![("max_z_accel", "bad accel")])];
    cases.push(("max_z_accel", wa));
    let mut waxis = world_dual();
    waxis.errors = vec![(
        "dual_carriage",
        vec![("axis", "Choice 'y' for option 'axis' in section 'dual_carriage' is not a valid choice")],
    )];
    cases.push(("dual_carriage axis", waxis));
    let mut wsd = world_dual();
    wsd.errors = vec![("dual_carriage", vec![("safe_distance", "bad safe distance")])];
    cases.push(("safe_distance", wsd));

    for (i, (label, world)) in cases.iter().enumerate() {
        py.exec("LOG = []").expect("reset log");
        py.exec(&world.py_state()).expect("python scenario state");
        let expected = py
            .exec("kin, toolhead, printer, gcode = build(hybrid_corexy.load_kinematics)")
            .expect_err("python must raise");
        let actual = rust_build(world).map(|_| ());
        let err = match actual {
            Ok(()) => panic!("{label}: rust loaded but python raised"),
            Err(err) => err,
        };
        check_err(i as u64, label, &expected, &err);
    }
}

#[test]
fn calc_position_matches_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping calc_position_matches_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_plain();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics") else {
        panic!("load_kinematics must succeed");
    };
    compare_calc_position(
        2,
        &mut py,
        &rig,
        &[
            ("stepper_x", 12.5),
            ("stepper_y", 30.0),
            ("stepper_y_1", -4.25),
            ("stepper_z", 7.25),
        ],
        "calc_position",
    );
    // A missing rail name is a KeyError naming the rail.
    let expected = py
        .eval("kin.calc_position({'stepper_x': 1.0, 'stepper_y': 2.0})")
        .expect_err("python KeyError");
    let mut map = IndexMap::new();
    map.insert("stepper_x".to_string(), 1.0);
    map.insert("stepper_y".to_string(), 2.0);
    let err = rig
        .kin
        .calc_position(&map)
        .expect_err("missing stepper name");
    check_err(3, "calc_position missing key", &expected, &err);
}

#[test]
fn set_position_and_clear_homing_match_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping set_position_and_clear_homing_match_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_plain();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics") else {
        panic!("load_kinematics must succeed");
    };
    py.exec("LOG = []").expect("reset log");
    rig.trace.lock().clear();
    py.exec("kin.set_position([10.0, 20.0, 30.0, 0], 'xy')")
        .expect("python set_position");
    rig.kin
        .set_position(vec![10.0, 20.0, 30.0, 0.0], "xy")
        .expect("set_position");
    let expected_log = py.eval("LOG").expect("python set_position log");
    check_equal(
        2,
        "set_position rail calls",
        &expected_log,
        &trace_value(&rig.trace),
    );
    compare_status(3, &mut py, &rig, "status after set_position");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        4,
        "limits after set_position",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );
    // `clear_homing_state` drops a homed axis back to `(1.0, -1.0)`.
    py.exec("kin.clear_homing_state('x')").expect("python clear");
    rig.kin_api.clear_homing_state("x");
    compare_status(5, &mut py, &rig, "status after clear_homing_state");
    // An unknown homing axis letter is `"xyz".index` -> ValueError.
    let expected = py
        .eval("kin.set_position([1.0, 2.0, 3.0, 0], 'xq')")
        .expect_err("python ValueError");
    let err = rig
        .kin
        .set_position(vec![1.0, 2.0, 3.0, 0.0], "xq")
        .expect_err("unknown axis letter");
    check_err(6, "set_position bad axis", &expected, &err);
    // `update_limits` takes a Python list index (wraps, raises out of range).
    let expected = py
        .eval("kin.update_limits(5, (0.0, 1.0))")
        .expect_err("python IndexError");
    let err = rig
        .kin
        .update_limits(5, (0.0, 1.0))
        .expect_err("index out of range");
    check_err(7, "update_limits bad index", &expected, &err);
    py.exec("kin.update_limits(-1, (5.0, 6.0))").expect("python update");
    rig.kin.update_limits(-1, (5.0, 6.0)).expect("update");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        8,
        "limits after negative update_limits",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );
}

#[test]
fn check_move_matches_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping check_move_matches_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_plain();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics") else {
        panic!("load_kinematics must succeed");
    };
    // Un-homed X/Y movement: "Must home axis first".
    let expected_unhomed = py
        .eval("kin.check_move(FakeMove([150.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], 1.0))")
        .expect_err("python must home");
    let mv = FakeMove {
        end_pos: Coord(vec![150.0, 0.0, 0.0, 0.0]),
        axes_d: Coord(vec![1.0, 0.0, 0.0, 0.0]),
        move_d: 1.0,
        limit_calls: Arc::new(Mutex::new(Vec::new())),
    };
    let err = rig.kin_api.check_move(&mv).expect_err("must home");
    check_err(2, "check_move unhomed", &expected_unhomed, &err);

    // Homed: a pure XY move leaves the speed alone, a Z move slows down.
    py.exec("kin.set_position([50.0, 100.0, 150.0, 0], 'xyz')")
        .expect("python set_position");
    rig.kin
        .set_position(vec![50.0, 100.0, 150.0, 0.0], "xyz")
        .expect("set_position");
    py.exec("mv_xy = FakeMove([50.0, 100.0, 150.0, 0.0], [0.5, 0.25, 0.0, 0.0], 0.75)\nkin.check_move(mv_xy)")
        .expect("python xy move");
    let xy_calls = Arc::new(Mutex::new(Vec::new()));
    let mv_xy = FakeMove {
        end_pos: Coord(vec![50.0, 100.0, 150.0, 0.0]),
        axes_d: Coord(vec![0.5, 0.25, 0.0, 0.0]),
        move_d: 0.75,
        limit_calls: xy_calls.clone(),
    };
    rig.kin_api.check_move(&mv_xy).expect("xy move");
    let expected_xy = py.eval("mv_xy.limit_calls").expect("python xy limits");
    check_equal(
        3,
        "check_move xy limits",
        &expected_xy,
        &limit_calls_value(&xy_calls.lock()),
    );

    py.exec("mv_z = FakeMove([50.0, 100.0, 160.0, 0.0], [1.0, 2.0, 5.0, 0.0], 10.0)\nkin.check_move(mv_z)")
        .expect("python z move");
    let z_calls = Arc::new(Mutex::new(Vec::new()));
    let mv_z = FakeMove {
        end_pos: Coord(vec![50.0, 100.0, 160.0, 0.0]),
        axes_d: Coord(vec![1.0, 2.0, 5.0, 0.0]),
        move_d: 10.0,
        limit_calls: z_calls.clone(),
    };
    rig.kin_api.check_move(&mv_z).expect("z move");
    let expected_z = py.eval("mv_z.limit_calls").expect("python z limits");
    check_equal(
        4,
        "check_move z limits",
        &expected_z,
        &limit_calls_value(&z_calls.lock()),
    );

    // Out of the homed range on a homed axis: "Move out of range".
    let expected_range = py
        .eval("kin.check_move(FakeMove([150.0, 100.0, 150.0, 0.0], [1.0, 0.0, 0.0, 0.0], 1.0))")
        .expect_err("python out of range");
    let mv_range = FakeMove {
        end_pos: Coord(vec![150.0, 100.0, 150.0, 0.0]),
        axes_d: Coord(vec![1.0, 0.0, 0.0, 0.0]),
        move_d: 1.0,
        limit_calls: Arc::new(Mutex::new(Vec::new())),
    };
    let err = rig.kin_api.check_move(&mv_range).expect_err("out of range");
    check_err(5, "check_move out of range", &expected_range, &err);
}

#[test]
fn home_matches_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping home_matches_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_plain();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics") else {
        panic!("load_kinematics must succeed");
    };
    // X homes negative, Z positive: both force position branches run.
    py.exec("hs = FakeHoming([0, 2])\nkin.home(hs)").expect("python home");
    let expected = py.eval("hs.home_rails_calls").expect("python home calls");
    let homing = Arc::new(FakeHoming::new(vec![0, 2]));
    let handle: HomingStateHandle = homing.clone();
    rig.kin_api.home(handle).expect("home");
    check_equal(
        2,
        "home rails",
        &expected,
        &homing_calls_value(&homing.calls.lock()),
    );
}

#[test]
fn dual_carriage_init_matches_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping dual_carriage_init_matches_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_dual();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics dual_carriage") else {
        panic!("load_kinematics must succeed");
    };
    compare_trace(2, &mut py, &rig, "dual_carriage init trace");
    compare_status(3, &mut py, &rig, "dual_carriage status");
    let expected_steppers = py
        .eval("[s.get_name() for s in kin.get_steppers()]")
        .expect("python steppers");
    let actual_steppers = Rv::List(
        rig.kin
            .get_steppers()
            .iter()
            .map(|s| Rv::Str(s.get_name(None)))
            .collect(),
    );
    check_equal(4, "dual_carriage steppers", &expected_steppers, &actual_steppers);
    let expected_dc = py.eval("kin.dc_module.get_status()").expect("python dc status");
    let dc = rig.kin.dc_module().expect("dc module");
    check_equal(5, "dc status", &expected_dc, &pyvalue_to_rec(&dc.get_status(0.0)));
    let expected_axes = py.eval("(kin.axes_min, kin.axes_max)").expect("python axes");
    let kin = &rig.kin;
    let actual_axes = Rv::Tuple(vec![
        pyvalue_to_rec(&py_axes(kin.axes_min())),
        pyvalue_to_rec(&py_axes(kin.axes_max())),
    ]);
    check_equal(6, "dual_carriage axes ranges", &expected_axes, &actual_axes);
    let expected_keys = py
        .eval("list(kin.dc_module.dc_rails.keys())")
        .expect("python dc rails");
    let actual_keys = Rv::List(
        dc.dc_rails()
            .keys()
            .map(|k| Rv::Str(k.clone()))
            .collect(),
    );
    check_equal(7, "dc rail names", &expected_keys, &actual_keys);
}

#[test]
fn dual_carriage_modes_match_oracle() {
    let Some(mut py) = oracle() else {
        eprintln!("skipping dual_carriage_modes_match_oracle: set KLIPPY_PYTHON");
        return;
    };
    let world = world_dual();
    let Some(rig) = build_both(1, &mut py, &world, "load_kinematics dual_carriage") else {
        panic!("load_kinematics must succeed");
    };
    // Default mode: carriage_1 is INACTIVE, X is `pos[0] + pos[1]`.
    let positions = vec![
        ("stepper_x", 12.5),
        ("stepper_y", 30.0),
        ("stepper_z", 7.25),
        ("dual_carriage", 40.0),
    ];
    compare_calc_position(2, &mut py, &rig, &positions, "calc_position primary default");

    // Switch the second carriage to PRIMARY; X becomes `pos[3] - pos[1]`.
    py.exec("dc = kin.dc_module\ndc.activate_dc_mode(dc.dc_rails['dual_carriage'], 'PRIMARY')")
        .expect("python activate");
    let dc = rig.kin.dc_module().expect("dc module").clone();
    let wrapper = dc
        .dc_rails()
        .get("dual_carriage")
        .expect("dual rail wrapper")
        .clone();
    dc.activate_dc_mode(&wrapper, PRIMARY).expect("activate");
    let expected_dc = py.eval("dc.get_status()").expect("python dc status");
    check_equal(5, "dc status primary", &expected_dc, &pyvalue_to_rec(&dc.get_status(0.0)));
    compare_calc_position(6, &mut py, &rig, &positions, "calc_position dual primary");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        7,
        "limits after activate",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );
    let expected_position = py.eval("list(toolhead.position)").expect("python position");
    let actual_position = Rv::List(
        rig.toolhead
            .position
            .cloned()
            .iter()
            .map(|v| Rv::float(*v))
            .collect(),
    );
    check_equal(8, "toolhead position after activate", &expected_position, &actual_position);

    // `set_position` takes the primary rail range for a homed X.
    py.exec("kin.set_position([5.0, 6.0, 7.0, 0], 'x')")
        .expect("python set_position");
    rig.kin
        .set_position(vec![5.0, 6.0, 7.0, 0.0], "x")
        .expect("set_position");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        9,
        "limits after set_position x",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );

    // Switching back to the primary carriage runs `update_limits` on an
    // already-homed X, so the new range is taken.
    py.exec("dc.activate_dc_mode(dc.dc_rails['stepper_x'], 'PRIMARY')")
        .expect("python activate primary");
    let primary = dc
        .dc_rails()
        .get("stepper_x")
        .expect("primary rail wrapper")
        .clone();
    dc.activate_dc_mode(&primary, PRIMARY).expect("activate primary");
    let expected_limits = py.eval("kin.limits").expect("python limits");
    check_equal(
        10,
        "limits after activate primary",
        &expected_limits,
        &limits_value(&rig.kin.limits()),
    );
    let expected_dc = py.eval("dc.get_status()").expect("python dc status");
    check_equal(
        11,
        "dc status primary carriage",
        &expected_dc,
        &pyvalue_to_rec(&dc.get_status(0.0)),
    );

    // Axis 0 homing is delegated to the dual carriage controller.
    py.exec("hs = FakeHoming([0, 1])\nkin.home(hs)").expect("python home");
    let expected = py.eval("hs.home_rails_calls").expect("python home calls");
    let homing = Arc::new(FakeHoming::new(vec![0, 1]));
    let handle: HomingStateHandle = homing.clone();
    rig.kin_api.home(handle).expect("home");
    check_equal(
        13,
        "dual carriage home rails",
        &expected,
        &homing_calls_value(&homing.calls.lock()),
    );
    let expected_dc = py.eval("dc.get_status()").expect("python dc status");
    check_equal(14, "dc status after home", &expected_dc, &pyvalue_to_rec(&dc.get_status(0.0)));
}

#[allow(dead_code)]
fn _assert_types() {
    let _ = std::any::type_name::<DualCarriages>();
    let _ = std::any::type_name::<GetOpts>();
    let _ = std::any::type_name::<GGetFloatOpts>();
    let _ = std::any::type_name::<GGetIntOpts>();
    let _ = std::any::type_name::<dyn PrinterObject>();
}
