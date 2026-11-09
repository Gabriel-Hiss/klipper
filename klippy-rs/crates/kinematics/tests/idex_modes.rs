//! Parity tests for `kinematics::idex_modes` (port of
//! `klippy/kinematics/idex_modes.py`).
//!
//! Every expected value comes from the live reference (`pyoracle`, CPython
//! 3.8.2 plus the frozen `klippy/`). The same scenario is built on both
//! sides: Python fakes for the printer, toolhead, kinematics, rails and
//! steppers around the real `kinematics.idex_modes` and the real
//! `gcode.GCodeCommand`, and Rust fakes of the contract traits around the
//! port, with the real `klippy_core::motion_handles` kinematics handles.
//! Every operation runs on both sides and the outcomes (return value or
//! exception class and message) plus a full state snapshot are compared:
//! rail modes, scales and offsets, the transform installed in every
//! stepper's dual-carriage kinematics, the toolhead position, flushes and
//! manual moves, the limits pushed into the kinematics, the homing log,
//! the events sent, the status dict and the accessors other kinematics
//! call. Floats compare by bits.
//!
//! The `chelper` module is replaced on the Python side by a fake library
//! that records the `dual_carriage_*` calls into plain attributes (the
//! compiled library is not part of the oracle contract).

use kinematics::idex_modes::{
    module_factory, DualCarriages, DualCarriagesApi, DualCarriagesRail, SavedDualCarriageState,
};
use klippy_api::config::Config;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts,
    RegisterCommandOpts,
};
use klippy_api::gcell::GCell;
use klippy_api::object::{lookup_interface, EventArg};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateApi, HomingStateHandle, KinematicsApi, LookaheadCallback, MoveApi,
    PrinterExtruder, ToolHead, ToolHeadApi,
};
use klippy_api::ObjectRef;
use klippy_api::PrinterObject;
use klippy_core::motion_handles::{alloc_stepper_kinematics, kinematics_of, KinAlloc};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::any::Any;
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import sys
from types import ModuleType
import collections

class Error(Exception):
    pass

_sentinel = object()

class _Sk:
    def __init__(self):
        self.orig_sk = None
        self.x_scale = 1.0
        self.x_offs = 0.0
        self.y_scale = 1.0
        self.y_offs = 0.0

class _Ffi:
    def gc(self, obj, destructor):
        return obj

class _Lib:
    def dual_carriage_alloc(self):
        return _Sk()
    def dual_carriage_set_sk(self, sk, orig_sk):
        sk.orig_sk = orig_sk
    def dual_carriage_set_transform(self, sk, axis, scale, offs):
        if axis == b'x':
            sk.x_scale = scale
            sk.x_offs = offs
            return 0
        if axis == b'y':
            sk.y_scale = scale
            sk.y_offs = offs
            return 0
        return -1
    def free(self, sk):
        pass

_chelper_mod = ModuleType('chelper')
_chelper_mod.get_ffi = lambda: (_Ffi(), _Lib())
sys.modules['chelper'] = _chelper_mod

import gcode as real_gcode
import kinematics.idex_modes as idex

class FakeReactor:
    def monotonic(self):
        return 0.0

class FakePrinter:
    config_error = Error
    def __init__(self):
        self.objects = {}
        self.handlers = []
        self.events = []
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=_sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def get_reactor(self):
        return FakeReactor()
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def send_event(self, event, *params):
        self.events.append(event)
        return []
    def fire(self, event):
        for e, cb in list(self.handlers):
            if e == event:
                cb()

class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.help = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.handlers[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
    def respond_info(self, msg, log=True):
        pass
    def respond_raw(self, msg):
        pass

HomingInfo = collections.namedtuple('homing_info', [
    'speed', 'position_endstop', 'retract_speed', 'retract_dist',
    'positive_dir', 'second_homing_speed'])

class FakeStepper:
    def __init__(self, name, sk):
        self.name = name
        self.sk = sk
    def get_stepper_kinematics(self):
        return self.sk
    def set_stepper_kinematics(self, sk):
        old = self.sk
        self.sk = sk
        return old

class FakeRail:
    def __init__(self, name, position_min, position_max, homing_speed, position_endstop, positive_dir, steppers):
        self.name = name
        self.position_min = position_min
        self.position_max = position_max
        self.homing_speed = homing_speed
        self.position_endstop = position_endstop
        self.positive_dir = positive_dir
        self.steppers = steppers
    def get_name(self, short=False):
        if short:
            if self.name.startswith('stepper'):
                return self.name[8:]
            return self.name.split()[-1]
        return self.name
    def get_range(self):
        return self.position_min, self.position_max
    def get_homing_info(self):
        return HomingInfo(self.homing_speed, self.position_endstop, 0.0, 0.0, self.positive_dir, 0.0)
    def get_steppers(self):
        return list(self.steppers)

class FakeKin:
    def __init__(self):
        self.limits_log = []
        self.home_axis_log = []
        self.homed_axes = 'xyz'
    def update_limits(self, axis, rng):
        self.limits_log.append((axis, tuple(rng)))
    def home_axis(self, homing_state, axis, rail):
        self.home_axis_log.append((axis, rail.get_name()))
    def get_status(self, eventtime):
        return {'homed_axes': self.homed_axes}

class FakeToolHead:
    def __init__(self, kin):
        self.kin = kin
        self.position = [10.0, 20.0, 5.0, 0.0]
        self.flush_count = 0
        self.moves = []
    def get_position(self):
        return list(self.position)
    def set_position(self, newpos, homing_axes=()):
        self.position = list(newpos)
    def get_kinematics(self):
        return self.kin
    def flush_step_generation(self):
        self.flush_count += 1
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

def make_gcmd(command, params):
    line = command + ''.join(' ' + k + '=' + v for k, v in params.items())
    return real_gcode.GCodeCommand(gcode, command, line, dict(params), False)

def mk_rail(name, pmin, pmax, speed, endstop, positive_dir, nsteppers):
    steppers = [FakeStepper(name if i == 0 else '%s_%d' % (name, i), _Sk()) for i in range(nsteppers)]
    return FakeRail(name, pmin, pmax, speed, endstop, positive_dir, steppers)

foreign = FakeRail('foreign', 0.0, 1.0, 1.0, 0.0, False, [])

def build(primary_spec, dual_spec, axes, safe_dist):
    printer = FakePrinter()
    gcode = FakeGCode()
    kin = FakeKin()
    toolhead = FakeToolHead(kin)
    printer.add_object('gcode', gcode)
    printer.add_object('toolhead', toolhead)
    prim = [mk_rail(*s) for s in primary_spec]
    dual = [mk_rail(*s) if s is not None else None for s in dual_spec]
    all_steppers = []
    for r in prim + dual:
        if r is not None:
            for s in r.steppers:
                if s not in all_steppers:
                    all_steppers.append(s)
    dc = idex.DualCarriages(printer, prim, dual, axes=axes, safe_dist=safe_dist)
    return printer, gcode, kin, toolhead, prim, dual, all_steppers, dc

def snap():
    all_rails = prim + [x for x in dual if x is not None]
    accessors = (
        dc.get_axes(),
        [(r.get_name(), dc.get_transform(r), dc.is_active(r),
          (dc.get_dc_rail_wrapper(r).get_name() if dc.get_dc_rail_wrapper(r) is not None else None))
         for r in all_rails],
        [(a, (dc.get_primary_rail(a).get_name() if dc.get_primary_rail(a) is not None else None))
         for a in (0, 1, 2)],
        dc.get_transform(foreign),
        dc.is_active(foreign),
        dc.get_dc_rail_wrapper(foreign),
    )
    return (
        [(k, v.mode, v.scale, v.offset, v.safe_dist, v.axis) for k, v in dc.dc_rails.items()],
        [(s.name, s.sk.x_scale, s.sk.x_offs, s.sk.y_scale, s.sk.y_offs) for s in all_steppers],
        list(toolhead.position),
        toolhead.flush_count,
        [(list(c), sp) for c, sp in toolhead.moves],
        list(kin.limits_log),
        list(kin.home_axis_log),
        list(printer.events),
        dc.get_status(),
        accessors,
    )
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakeStepper {
    name: String,
    sk: GCell<Option<StepperKinematics>>,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu {
        panic!("get_mcu is not used by idex_modes")
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
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
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
    fn mcu_to_commanded_position(&self, mcu_pos: f64) -> f64 {
        mcu_pos
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
        tq
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        false
    }
}

struct FakeRail {
    name: String,
    position_min: f64,
    position_max: f64,
    homing_speed: f64,
    position_endstop: f64,
    positive_dir: bool,
    steppers: Vec<Stepper>,
}

impl FakeRail {
    fn short_name(name: &str) -> String {
        if name.starts_with("stepper") {
            name[8..].to_string()
        } else {
            name.split_whitespace().last().unwrap_or(name).to_string()
        }
    }
}

impl RailApi for FakeRail {
    fn get_name(&self, short: Option<bool>) -> String {
        match short {
            Some(true) => Self::short_name(&self.name),
            _ => self.name.clone(),
        }
    }
    fn get_range(&self) -> (f64, f64) {
        (self.position_min, self.position_max)
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: self.homing_speed,
            position_endstop: self.position_endstop,
            retract_speed: 0.0,
            retract_dist: 0.0,
            positive_dir: self.positive_dir,
            second_homing_speed: 0.0,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.clone()
    }
    fn get_endstops(&self) -> Vec<(Arc<dyn klippy_api::mcu::EndstopApi>, String)> {
        Vec::new()
    }
    fn lookup_endstop(
        &self,
        _endstop_pin: Option<&str>,
        _name: &str,
    ) -> KResult<Arc<dyn klippy_api::mcu::EndstopApi>> {
        Err(KlippyError::config("no endstop in fake rail"))
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
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
    fn set_trapq(&self, _tq: Option<Trapq>) {}
    fn set_position(&self, _coord: [f64; 3]) {}
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
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

struct FakeKin {
    limits_log: Mutex<Vec<(i64, (f64, f64))>>,
    home_axis_log: Mutex<Vec<(i64, String)>>,
    homed_axes: GCell<String>,
}

impl FakeKin {
    fn new() -> FakeKin {
        FakeKin {
            limits_log: Mutex::new(Vec::new()),
            home_axis_log: Mutex::new(Vec::new()),
            homed_axes: GCell::new("xyz".to_string()),
        }
    }
}

impl KinematicsApi for FakeKin {
    fn class_name(&self) -> &'static str {
        "FakeKin"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        Vec::new()
    }
    fn calc_position(
        &self,
        _stepper_positions: &indexmap::IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(vec![None, None, None])
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        let mut m = indexmap::IndexMap::new();
        m.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(self.homed_axes.cloned()),
        );
        Ok(PyValue::Dict(m))
    }
    fn clear_homing_state(&self, _clear_axes: &str) -> klippy_api::KResult<()> { Ok(()) }
    fn home_axis(&self, _homing_state: HomingStateHandle, axis: i64, rail: Rail) -> KResult<()> {
        self.home_axis_log
            .lock()
            .push((axis, rail.get_name(Some(false))));
        Ok(())
    }
    fn update_limits(&self, axis: i64, range: (f64, f64)) -> KResult<()> {
        self.limits_log.lock().push((axis, range));
        Ok(())
    }
}

struct FakeToolHead {
    kin: klippy_api::toolhead::Kinematics,
    position: GCell<Vec<f64>>,
    flush_count: Mutex<i64>,
    moves: Mutex<Vec<(Vec<Option<f64>>, f64)>>,
}

impl FakeToolHead {
    fn new(kin: klippy_api::toolhead::Kinematics) -> FakeToolHead {
        FakeToolHead {
            kin,
            position: GCell::new(vec![10.0, 20.0, 5.0, 0.0]),
            flush_count: Mutex::new(0),
            moves: Mutex::new(Vec::new()),
        }
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
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.moves.lock().push((coord.clone(), speed));
        let mut pos = self.position.cloned();
        for (i, v) in coord.iter().enumerate() {
            if let Some(v) = v {
                pos[i] = *v;
            }
        }
        self.position.replace(pos);
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(indexmap::IndexMap::new()))
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
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
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by idex_modes")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        Arc::new(FakeStepperFactory)
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
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        self.kin.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flush_count.lock() += 1;
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
        _drip_completion: &Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

struct FakeStepperFactory;

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        Err(KlippyError::config("stepper factory is not used by idex_modes"))
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::config("stepper factory is not used by idex_modes"))
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::config("stepper factory is not used by idex_modes"))
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        Err(KlippyError::config("stepper factory is not used by idex_modes"))
    }
    fn parse_step_distance(
        &self,
        _config: &Config,
        _units_in_radians: Option<bool>,
        _note_valid: bool,
    ) -> KResult<(f64, f64)> {
        Err(KlippyError::config("stepper factory is not used by idex_modes"))
    }
    fn extruder_stepper(&self, _config: &Config) -> KResult<Option<klippy_api::toolhead::ExtruderStepper>> {
        panic!("extruder_stepper is not used")
    }
}

struct FakeGCode {
    handlers: Mutex<indexmap::IndexMap<String, klippy_api::gcode::GCodeCommandHandler>>,
    help: Mutex<indexmap::IndexMap<String, String>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            handlers: Mutex::new(indexmap::IndexMap::new()),
            help: Mutex::new(indexmap::IndexMap::new()),
        }
    }

    fn take_handler(&self, cmd: &str) -> Option<klippy_api::gcode::GCodeCommandHandler> {
        self.handlers.lock().shift_remove(cmd)
    }
}

impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        let mut handlers = self.handlers.lock();
        let old = handlers.shift_remove(cmd);
        if let Some(h) = func {
            handlers.insert(cmd.to_string(), h);
        }
        if let Some(desc) = opts.desc {
            self.help.lock().insert(cmd.to_string(), desc);
        }
        Ok(old)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        self.help.lock().clone()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
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
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by idex_modes")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by idex_modes")
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
    objects: Mutex<indexmap::IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
    events: Mutex<Vec<String>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(indexmap::IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            events: Mutex::new(Vec::new()),
        }
    }

    /// Runs every handler registered for `event` (the Python fake's `fire`).
    fn fire(&self, event: &str) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                cb(&[])?;
            }
        }
        Ok(())
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
        let mut objects = self.objects.lock();
        if objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objects = self.objects.lock();
        if let Some(obj) = objects.get(name) {
            return Ok(Some(obj.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let objects = self.objects.lock();
            match module {
                None => objects.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
                Some(module) => {
                    let prefix = format!("{module} ");
                    let mut out = Vec::new();
                    if let Some(obj) = objects.get(module) {
                        out.push((module.to_string(), obj.clone()));
                    }
                    for (name, obj) in objects.iter() {
                        if name.starts_with(&prefix) {
                            out.push((name.clone(), obj.clone()));
                        }
                    }
                    out
                }
            }
    
        };
        objects.into_iter().map(|(name, obj)| (name, Some(obj))).collect()
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
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.events.lock().push(event.to_string());
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeHomingState;

impl HomingStateApi for FakeHomingState {
    fn set_axes(&self, _axes: Vec<i64>) {}
    fn get_axes(&self) -> Vec<i64> {
        Vec::new()
    }
    fn get_trigger_position(&self, _stepper_name: &str) -> KResult<i64> {
        Err(KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "fake".to_string(),
        })
    }
    fn set_stepper_adjustment(&self, _stepper_name: &str, _adjustment: f64) {}
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        Ok(())
    }
    fn home_rails(
        &self,
        _rails: Vec<Rail>,
        _forcepos: Vec<Option<f64>>,
        _movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        Ok(())
    }
}

/// `GCodeCommandApi` over the core's own command parser, so the parameter
/// errors come from the ported `gcode.GCodeCommand` logic.
struct FakeGCmd {
    inner: klippy_core::gcode_parse::GCodeCommand,
}

impl klippy_api::gcode::GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let default_str: Option<Option<String>> = opts.default.as_ref().map(|v| match v {
            PyValue::Str(s) => Some(s.clone()),
            PyValue::None => None,
            other => Some(other.str()),
        });
        match self.inner.get_str_opt(name, default_str) {
            Ok(Some(s)) => Ok(PyValue::Str(s)),
            Ok(None) => Ok(PyValue::None),
            Err(e) => Err(KlippyError::command(e.0)),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        self.inner
            .get_int_opt(name, opts.default, opts.minval, opts.maxval)
            .map_err(|e| KlippyError::command(e.0))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        self.inner
            .get_float_opt(name, opts.default, opts.minval, opts.maxval, opts.above, opts.below, opts.ints)
            .map_err(|e| KlippyError::command(e.0))
    }
    fn get_command(&self) -> String {
        self.inner.get_command().to_string()
    }
    fn get_commandline(&self) -> String {
        self.inner.get_commandline().to_string()
    }
    fn get_command_parameters(&self) -> indexmap::IndexMap<String, String> {
        self.inner.get_command_parameters().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.inner.get_raw_command_parameters()
    }
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// ---------------------------------------------------------------------------
// Scenarios and the two rigs.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct RailSpec {
    name: &'static str,
    min: f64,
    max: f64,
    speed: f64,
    endstop: f64,
    positive: bool,
    steppers: usize,
}

fn rs(
    name: &'static str,
    min: f64,
    max: f64,
    speed: f64,
    endstop: f64,
    positive: bool,
    steppers: usize,
) -> RailSpec {
    RailSpec { name, min, max, speed, endstop, positive, steppers }
}

#[derive(Clone)]
struct Scenario {
    primary: Vec<RailSpec>,
    dual: Vec<Option<RailSpec>>,
    axes: Vec<i64>,
    safe_dist: Vec<Option<f64>>,
}

/// Classic IDEX: the carriages home away from each other.
fn sc_idex() -> Scenario {
    Scenario {
        primary: vec![rs("stepper_x", 0.0, 200.0, 50.0, 0.0, false, 1)],
        dual: vec![Some(rs("dual_carriage", 0.0, 200.0, 40.0, 200.0, true, 2))],
        axes: vec![0],
        safe_dist: vec![Some(50.0)],
    }
}

/// Both carriages home to the minimum; the default safe distance is derived
/// from the rail ranges and the dual carriage homes first.
fn sc_same_dir() -> Scenario {
    Scenario {
        primary: vec![rs("stepper_x", 0.0, 150.0, 60.0, 20.0, false, 1)],
        dual: vec![Some(rs("dual_carriage", 20.0, 180.0, 30.0, 0.0, false, 1))],
        axes: vec![0],
        safe_dist: vec![None],
    }
}

/// Both carriages home to the maximum with a zero (falsy) safe distance.
fn sc_positive_same_dir() -> Scenario {
    Scenario {
        primary: vec![rs("stepper_x", 0.0, 200.0, 50.0, 190.0, true, 1)],
        dual: vec![Some(rs("dual_carriage", 0.0, 200.0, 50.0, 200.0, true, 1))],
        axes: vec![0],
        safe_dist: vec![Some(0.0)],
    }
}

/// One primary carriage without a dual rail.
fn sc_primary_only() -> Scenario {
    Scenario {
        primary: vec![rs("stepper_x", 0.0, 200.0, 50.0, 0.0, false, 1)],
        dual: vec![None],
        axes: vec![0],
        safe_dist: vec![None],
    }
}

/// Dual carriages on X and Y.
fn sc_xy() -> Scenario {
    Scenario {
        primary: vec![
            rs("stepper_x", 0.0, 200.0, 50.0, 0.0, false, 1),
            rs("stepper_y", 0.0, 180.0, 45.0, 0.0, false, 1),
        ],
        dual: vec![
            Some(rs("dual_carriage", 0.0, 200.0, 50.0, 200.0, true, 1)),
            Some(rs("dual_carriage_y", 10.0, 170.0, 35.0, 180.0, true, 1)),
        ],
        axes: vec![0, 1],
        safe_dist: vec![Some(30.0), None],
    }
}

/// A `dual_rails` list longer than `primary_rails`, with the extra entry
/// `None`: Python never indexes the other lists for it.
fn sc_dual_longer() -> Scenario {
    Scenario {
        primary: vec![rs("stepper_x", 0.0, 200.0, 50.0, 0.0, false, 1)],
        dual: vec![Some(rs("dual_carriage", 0.0, 200.0, 50.0, 200.0, true, 1)), None],
        axes: vec![0],
        safe_dist: vec![Some(40.0), None],
    }
}

fn scenarios() -> Vec<(&'static str, Scenario)> {
    vec![
        ("idex", sc_idex()),
        ("same_dir", sc_same_dir()),
        ("positive_same_dir", sc_positive_same_dir()),
        ("primary_only", sc_primary_only()),
        ("xy", sc_xy()),
        ("dual_longer", sc_dual_longer()),
    ]
}

fn pf(x: f64) -> String {
    if x.is_nan() {
        "float('nan')".to_string()
    } else if x.is_infinite() {
        if x > 0.0 { "float('inf')".to_string() } else { "float('-inf')".to_string() }
    } else {
        format!("{x:?}")
    }
}

fn pb(b: bool) -> &'static str {
    if b { "True" } else { "False" }
}

fn pylist(values: &[f64]) -> String {
    format!("[{}]", values.iter().map(|v| pf(*v)).collect::<Vec<_>>().join(", "))
}

fn py_rail_spec(r: &RailSpec) -> String {
    format!(
        "({:?}, {}, {}, {}, {}, {}, {})",
        r.name,
        pf(r.min),
        pf(r.max),
        pf(r.speed),
        pf(r.endstop),
        pb(r.positive),
        r.steppers
    )
}

fn py_build_expr(sc: &Scenario) -> String {
    let primary: Vec<String> = sc.primary.iter().map(py_rail_spec).collect();
    let dual: Vec<String> = sc
        .dual
        .iter()
        .map(|d| match d {
            Some(r) => py_rail_spec(r),
            None => "None".to_string(),
        })
        .collect();
    let axes: Vec<String> = sc.axes.iter().map(|a| a.to_string()).collect();
    let safe: Vec<String> = sc
        .safe_dist
        .iter()
        .map(|s| match s {
            Some(v) => pf(*v),
            None => "None".to_string(),
        })
        .collect();
    format!(
        "build([{}], [{}], [{}], [{}])",
        primary.join(", "),
        dual.join(", "),
        axes.join(", "),
        safe.join(", ")
    )
}

struct Rig {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    gcode: Arc<FakeGCode>,
    kin: Arc<FakeKin>,
    toolhead: Arc<FakeToolHead>,
    prim: Vec<Rail>,
    dual: Vec<Option<Rail>>,
    steppers: Vec<Arc<FakeStepper>>,
    foreign: Rail,
    saved: Mutex<Option<SavedDualCarriageState>>,
    dc: Arc<DualCarriages>,
}

fn make_rail(spec: &RailSpec, steppers_out: &mut Vec<Arc<FakeStepper>>) -> Rail {
    let mut rail_steppers: Vec<Stepper> = Vec::new();
    for i in 0..spec.steppers {
        let name = if i == 0 { spec.name.to_string() } else { format!("{}_{}", spec.name, i) };
        let sk = alloc_stepper_kinematics("cartesian_stepper_alloc", &[PyValue::Bytes(vec![b'x'])])
            .expect("cartesian_stepper_alloc");
        let stepper = Arc::new(FakeStepper { name, sk: GCell::new(Some(sk)) });
        steppers_out.push(stepper.clone());
        rail_steppers.push(stepper as Stepper);
    }
    Arc::new(FakeRail {
        name: spec.name.to_string(),
        position_min: spec.min,
        position_max: spec.max,
        homing_speed: spec.speed,
        position_endstop: spec.endstop,
        positive_dir: spec.positive,
        steppers: rail_steppers,
    })
}

fn rust_rig(sc: &Scenario) -> KResult<Rig> {
    let printer = Arc::new(FakePrinter::new());
    let printer_api: Printer = printer.clone();
    let gcode = Arc::new(FakeGCode::new());
    let kin = Arc::new(FakeKin::new());
    let kin_dyn: klippy_api::toolhead::Kinematics = kin.clone();
    let toolhead = Arc::new(FakeToolHead::new(kin_dyn));
    printer.add_object("gcode", gcode.clone() as ObjectRef)?;
    printer.add_object("toolhead", toolhead.clone() as ObjectRef)?;
    let mut steppers: Vec<Arc<FakeStepper>> = Vec::new();
    let prim: Vec<Rail> = sc.primary.iter().map(|s| make_rail(s, &mut steppers)).collect();
    let dual: Vec<Option<Rail>> =
        sc.dual.iter().map(|d| d.as_ref().map(|s| make_rail(s, &mut steppers))).collect();
    let foreign: Rail = Arc::new(FakeRail {
        name: "foreign".to_string(),
        position_min: 0.0,
        position_max: 1.0,
        homing_speed: 1.0,
        position_endstop: 0.0,
        positive_dir: false,
        steppers: Vec::new(),
    });
    let dc = DualCarriages::new(
        printer_api.clone(),
        prim.clone(),
        dual.clone(),
        sc.axes.clone(),
        sc.safe_dist.clone(),
    )?;
    Ok(Rig {
        printer,
        printer_api,
        gcode,
        kin,
        toolhead,
        prim,
        dual,
        steppers,
        foreign,
        saved: Mutex::new(None),
        dc,
    })
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(PY_SETUP).expect("idex_modes python fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping idex_modes oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn start(sc: &Scenario) -> Option<(pyoracle::Oracle, Rig)> {
    let mut py = oracle()?;
    py.exec(&format!(
        "printer, gcode, kin, toolhead, prim, dual, all_steppers, dc = {}",
        py_build_expr(sc)
    ))
    .expect("python build");
    let rig = rust_rig(sc).expect("rust build");
    Some((py, rig))
}

// ---------------------------------------------------------------------------
// Value plumbing.
// ---------------------------------------------------------------------------

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "kinematics.idex_modes".to_string(),
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

fn check_value(label: &str, expected: &Value, actual: &Value) {
    let record = record_for(label);
    if let Err(d) = assert_equal(&record, "ret", expected, actual) {
        panic!("{label}: {d}");
    }
}

/// Compares an oracle outcome with the port's: equal values, or the same
/// exception class and message.
fn outcome(label: &str, py: pyoracle::OracleResult, rust: KResult<Value>) {
    match (py, rust) {
        (Ok(p), Ok(r)) => check_value(label, &p, &r),
        (Err(pe), Err(re)) => {
            assert_eq!(re.class_name(), pe.class_name, "{label}: exception class");
            assert_eq!(re.message(), pe.message, "{label}: exception message");
        }
        (p, r) => panic!("{label}: oracle={p:?} rust={r:?}"),
    }
}

fn fv(f: f64) -> Value {
    Value::float(f)
}

fn sv(s: &str) -> Value {
    Value::str(s)
}

fn iv(i: i64) -> Value {
    Value::int(i)
}

fn tup(items: Vec<Value>) -> Value {
    Value::Tuple(items)
}

fn lst(items: Vec<Value>) -> Value {
    Value::List(items)
}

fn opt_f(v: Option<f64>) -> Value {
    match v {
        Some(f) => fv(f),
        None => Value::Null,
    }
}

fn unit(r: KResult<()>) -> KResult<Value> {
    r.map(|_| Value::Null)
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter().map(|(k, val)| (pyvalue_to_value(k), pyvalue_to_value(val))).collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host:{h}")),
    }
}

/// `(x_scale, x_offs, y_scale, y_offs)` of the dual-carriage kinematics
/// installed in `stepper`.
fn transform_of(stepper: &FakeStepper) -> (f64, f64, f64, f64) {
    let sk = stepper.get_stepper_kinematics().expect("stepper kinematics");
    let transform = kinematics_of(&sk).expect("motion handle").with(|alloc| match alloc {
        KinAlloc::DualCarriage(dc) => {
            let (xs, xo) = dc.x_transform();
            let (ys, yo) = dc.y_transform();
            (xs, xo, ys, yo)
        }
        _ => panic!("stepper {} has no dual carriage kinematics", stepper.name),
    });
    transform
}

fn accessors_value(rig: &Rig) -> Value {
    let api = lookup_interface::<dyn DualCarriagesApi>(
        &*rig.printer_api,
        "dual_carriage",
        LookupDefault::Raise,
    )
    .expect("lookup dual_carriage")
    .expect("Raise never yields None");
    let mut all_rails: Vec<Rail> = rig.prim.clone();
    all_rails.extend(rig.dual.iter().flatten().cloned());
    let per_rail: Vec<Value> = all_rails
        .iter()
        .map(|r| {
            let (scale, offset) = api.get_transform(r);
            let wrapper = match api.get_dc_rail_wrapper(r) {
                Some(w) => sv(&w.get_name()),
                None => Value::Null,
            };
            tup(vec![sv(&r.get_name(Some(false))), tup(vec![fv(scale), fv(offset)]), Value::Bool(api.is_active(r)), wrapper])
        })
        .collect();
    let primaries: Vec<Value> = (0..3)
        .map(|a| {
            tup(vec![
                iv(a),
                match api.get_primary_rail(a) {
                    Some(r) => sv(&r.get_name(Some(false))),
                    None => Value::Null,
                },
            ])
        })
        .collect();
    let (fs, fo) = api.get_transform(&rig.foreign);
    tup(vec![
        lst(api.get_axes().into_iter().map(iv).collect()),
        lst(per_rail),
        lst(primaries),
        tup(vec![fv(fs), fv(fo)]),
        Value::Bool(api.is_active(&rig.foreign)),
        match api.get_dc_rail_wrapper(&rig.foreign) {
            Some(w) => sv(&w.get_name()),
            None => Value::Null,
        },
    ])
}

fn snapshot(rig: &Rig) -> Value {
    let dc = &rig.dc;
    let rails = lst(
        dc.dc_rails()
            .iter()
            .map(|(k, w)| {
                tup(vec![
                    sv(k),
                    sv(&w.mode()),
                    fv(w.scale()),
                    fv(w.offset()),
                    opt_f(w.safe_dist()),
                    iv(w.axis()),
                ])
            })
            .collect(),
    );
    let steppers = lst(
        rig.steppers
            .iter()
            .map(|s| {
                let (a, b, c, d) = transform_of(s);
                tup(vec![sv(&s.name), fv(a), fv(b), fv(c), fv(d)])
            })
            .collect(),
    );
    let position = lst(rig.toolhead.position.cloned().into_iter().map(fv).collect());
    let flush = iv(*rig.toolhead.flush_count.lock());
    let moves = lst(
        rig.toolhead
            .moves
            .lock()
            .iter()
            .map(|(coord, speed)| {
                tup(vec![
                    lst(coord.iter().map(|v| opt_f(*v)).collect()),
                    fv(*speed),
                ])
            })
            .collect(),
    );
    let limits = lst(
        rig.kin
            .limits_log
            .lock()
            .iter()
            .map(|(axis, range)| tup(vec![iv(*axis), tup(vec![fv(range.0), fv(range.1)])]))
            .collect(),
    );
    let home_log = lst(
        rig.kin
            .home_axis_log
            .lock()
            .iter()
            .map(|(axis, name)| tup(vec![iv(*axis), sv(name)]))
            .collect(),
    );
    let events = lst(rig.printer.events.lock().iter().map(|e| sv(e)).collect());
    let status = pyvalue_to_value(&dc.get_status(0.0));
    tup(vec![rails, steppers, position, flush, moves, limits, home_log, events, status, accessors_value(rig)])
}

fn compare_state(py: &mut pyoracle::Oracle, rig: &Rig, label: &str) {
    let expected = py.eval("snap()").unwrap_or_else(|e| panic!("{label}: snap raised {}: {}", e.class_name, e.message));
    check_value(&format!("{label}: state"), &expected, &snapshot(rig));
}

fn saved_value(s: &SavedDualCarriageState) -> Value {
    Value::Dict(vec![
        (
            sv("carriage_modes"),
            Value::Dict(s.carriage_modes.iter().map(|(k, v)| (sv(k), sv(v))).collect()),
        ),
        (
            sv("carriage_positions"),
            Value::Dict(s.carriage_positions.iter().map(|(k, v)| (sv(k), fv(*v))).collect()),
        ),
        (
            sv("toolhead_position"),
            tup(s.toolhead_position.iter().map(|v| fv(*v)).collect()),
        ),
    ])
}

fn set_position(py: &mut pyoracle::Oracle, rig: &Rig, pos: &[f64; 4]) {
    py.exec(&format!("toolhead.position = {}", pylist(pos))).expect("python position");
    rig.toolhead.position.replace(pos.to_vec());
}

fn set_homed(py: &mut pyoracle::Oracle, rig: &Rig, axes: &str) {
    py.exec(&format!("kin.homed_axes = {axes:?}")).expect("python homed axes");
    rig.kin.homed_axes.replace(axes.to_string());
}

fn run_cmd(rig: &Rig, cmd: &str, params: &[(String, String)]) -> KResult<()> {
    let mut handler = rig.gcode.take_handler(cmd).expect("handler registered");
    let mut commandline = cmd.to_string();
    let mut map = indexmap::IndexMap::new();
    for (k, v) in params {
        commandline.push(' ');
        commandline.push_str(k);
        commandline.push('=');
        commandline.push_str(v);
        map.insert(k.clone(), v.clone());
    }
    let inner = klippy_core::gcode_parse::GCodeCommand::new(
        cmd.to_string(),
        commandline,
        map,
        false,
    );
    let gcmd: GCmd = Arc::new(FakeGCmd { inner });
    let res = handler(&gcmd);
    rig.gcode.handlers.lock().insert(cmd.to_string(), handler);
    res
}

enum Step {
    Pos([f64; 4]),
    Cmd(String, Vec<(String, String)>),
    Act(String, &'static str),
    Homed(&'static str),
    Ready,
    SaveApi,
    RestoreApi(i64, f64),
}

fn cmd(command: &str, params: &[(&str, &str)]) -> Step {
    Step::Cmd(
        command.to_string(),
        params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
    )
}

fn run_steps(py: &mut pyoracle::Oracle, rig: &Rig, steps: &[Step], tag: &str) {
    for (i, step) in steps.iter().enumerate() {
        match step {
            Step::Pos(pos) => {
                set_position(py, rig, pos);
                compare_state(py, rig, &format!("{tag}#{i} position"));
            }
            Step::Homed(axes) => {
                set_homed(py, rig, axes);
                compare_state(py, rig, &format!("{tag}#{i} homed"));
            }
            Step::Cmd(command, params) => {
                let label = format!("{tag}#{i} {command} {params:?}");
                let py_params = params
                    .iter()
                    .map(|(k, v)| format!("{k:?}: {v:?}"))
                    .collect::<Vec<_>>()
                    .join(", ");
                let expr = format!(
                    "gcode.handlers[{command:?}](make_gcmd({command:?}, {{{py_params}}}))"
                );
                outcome(&label, py.eval(&expr), unit(run_cmd(rig, command, params)));
                compare_state(py, rig, &label);
            }
            Step::Act(name, mode) => {
                let label = format!("{tag}#{i} activate {name} {mode}");
                let wrapper = rig.dc.dc_rails()[name.as_str()].clone();
                outcome(
                    &label,
                    py.eval(&format!("dc.activate_dc_mode(dc.dc_rails[{name:?}], {mode:?})")),
                    unit(rig.dc.activate_dc_mode(&wrapper, mode)),
                );
                compare_state(py, rig, &label);
            }
            Step::Ready => {
                let label = format!("{tag}#{i} klippy:ready");
                outcome(
                    &label,
                    py.eval("printer.fire('klippy:ready')"),
                    unit(rig.printer.fire("klippy:ready")),
                );
                compare_state(py, rig, &label);
            }
            Step::SaveApi => {
                let label = format!("{tag}#{i} save_dual_carriage_state");
                let expected = py.exec("saved_api = dc.save_dual_carriage_state()");
                let rust = rig.dc.save_dual_carriage_state();
                match (expected, &rust) {
                    (Ok(_), Ok(saved)) => {
                        let py_value = py
                            .eval("saved_api")
                            .unwrap_or_else(|e| panic!("{label}: {}: {}", e.class_name, e.message));
                        check_value(&label, &py_value, &saved_value(saved));
                    }
                    (Err(pe), Err(re)) => {
                        assert_eq!(re.class_name(), pe.class_name, "{label}: class");
                        assert_eq!(re.message(), pe.message, "{label}: message");
                    }
                    (p, r) => panic!("{label}: oracle={p:?} rust={:?}", r.as_ref().map(|_| ())),
                }
                if let Ok(saved) = rust {
                    *rig.saved.lock() = Some(saved);
                }
                compare_state(py, rig, &label);
            }
            Step::RestoreApi(move_flag, speed) => {
                let label = format!("{tag}#{i} restore_dual_carriage_state({move_flag}, {speed})");
                let saved = rig.saved.lock().clone().expect("saved state");
                outcome(
                    &label,
                    py.eval(&format!(
                        "dc.restore_dual_carriage_state(saved_api, {move_flag}, {})",
                        pf(*speed)
                    )),
                    unit(rig.dc.restore_dual_carriage_state(&saved, *move_flag, *speed)),
                );
                compare_state(py, rig, &label);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_is_empty() {
    // `idex_modes.py` is imported by the kinematics modules; it has no
    // `load_config` or `add_printer_objects` entry point.
    let f = module_factory();
    assert_eq!(f.entry_points(), pyoracle::entry_points("kinematics.idex_modes"), "entry points of kinematics.idex_modes");
}

#[test]
fn constructor_registers_and_initial_state() {
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        let label = format!("constructor[{name}]");
        check_value(
            &format!("{label} printer objects"),
            &py.eval("list(printer.objects.keys())").expect("objects"),
            &lst(rig.printer.objects.lock().keys().map(|k| sv(k)).collect()),
        );
        check_value(
            &format!("{label} event handlers"),
            &py.eval("[e for e, cb in printer.handlers]").expect("handlers"),
            &lst(rig.printer.handlers.lock().iter().map(|(e, _)| sv(e)).collect()),
        );
        check_value(
            &format!("{label} gcode handlers"),
            &py.eval("list(gcode.handlers.keys())").expect("gcode handlers"),
            &lst(rig.gcode.handlers.lock().keys().map(|k| sv(k)).collect()),
        );
        check_value(
            &format!("{label} gcode help"),
            &py.eval("list(gcode.help.items())").expect("gcode help"),
            &lst(
                rig.gcode
                    .help
                    .lock()
                    .iter()
                    .map(|(k, v)| tup(vec![sv(k), sv(v)]))
                    .collect(),
            ),
        );
        compare_state(&mut py, &rig, &label);
        // The `PrinterObject` status is the Python `get_status()`.
        let obj = rig
            .printer_api
            .lookup_object("dual_carriage", LookupDefault::Raise)
            .expect("lookup")
            .expect("dual_carriage registered");
        check_value(
            &format!("{label} object status"),
            &py.eval("dc.get_status()").expect("status"),
            &pyvalue_to_value(&obj.get_status(0.0).unwrap().expect("get_status")),
        );
    }
}

#[test]
fn constructor_errors() {
    let mut cases: Vec<(&str, Scenario)> = Vec::new();
    let mut no_stepper_primary = sc_idex();
    no_stepper_primary.primary[0].steppers = 0;
    cases.push(("no_stepper_primary", no_stepper_primary));
    let mut no_stepper_dual = sc_idex();
    no_stepper_dual.dual[0].as_mut().unwrap().steppers = 0;
    cases.push(("no_stepper_dual", no_stepper_dual));
    let mut axes_out_of_range = sc_idex();
    axes_out_of_range.axes = vec![3];
    cases.push(("axes_out_of_range", axes_out_of_range));
    let mut axes_empty = sc_idex();
    axes_empty.axes = vec![];
    cases.push(("axes_empty", axes_empty));
    let mut safe_dist_empty = sc_idex();
    safe_dist_empty.safe_dist = vec![];
    cases.push(("safe_dist_empty", safe_dist_empty));
    let mut dual_empty = sc_idex();
    dual_empty.dual = vec![];
    cases.push(("dual_empty", dual_empty));
    let mut safe_dist_default_missing = sc_same_dir();
    safe_dist_default_missing.safe_dist = vec![];
    cases.push(("safe_dist_default_missing", safe_dist_default_missing));
    for (label, sc) in cases {
        let Some(mut py) = oracle() else { return };
        let py_res = py.exec(&format!(
            "printer, gcode, kin, toolhead, prim, dual, all_steppers, dc = {}",
            py_build_expr(&sc)
        ));
        let rust_res = rust_rig(&sc).map(|_| Value::Null);
        if py_res.is_ok() || rust_res.is_ok() {
            panic!("constructor error [{label}]: oracle={py_res:?} rust={rust_res:?}");
        }
        outcome(&format!("constructor error [{label}]"), py_res, rust_res);
    }
}

#[test]
fn get_dc_order_matches() {
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        let names: Vec<String> = rig.dc.dc_rails().keys().cloned().collect();
        for a in &names {
            for b in &names {
                let wa = rig.dc.dc_rails()[a.as_str()].clone();
                let wb = rig.dc.dc_rails()[b.as_str()].clone();
                outcome(
                    &format!("get_dc_order[{name}] {a} {b}"),
                    py.eval(&format!("dc.get_dc_order(dc.dc_rails[{a:?}], dc.dc_rails[{b:?}])")),
                    Ok(iv(rig.dc.get_dc_order(&wa, &wb))),
                );
            }
        }
    }
}

#[test]
fn activate_modes_and_ranges() {
    let positions: [[f64; 4]; 3] = [
        [10.0, 20.0, 5.0, 0.0],
        [120.0, 60.0, 5.0, 0.0],
        [195.5, 170.0, 1.0, 0.0],
    ];
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        let names: Vec<String> = rig.dc.dc_rails().keys().cloned().collect();
        let toolhead: ToolHead = rig.toolhead.clone();
        for pos in positions {
            set_position(&mut py, &rig, &pos);
            for carriage in &names {
                for mode in ["PRIMARY", "COPY", "MIRROR", "INACTIVE", "COPY", "PRIMARY"] {
                    let label = format!("activate[{name}] {carriage} {mode} at {pos:?}");
                    let wrapper = rig.dc.dc_rails()[carriage.as_str()].clone();
                    outcome(
                        &label,
                        py.eval(&format!(
                            "dc.activate_dc_mode(dc.dc_rails[{carriage:?}], {mode:?})"
                        )),
                        unit(rig.dc.activate_dc_mode(&wrapper, mode)),
                    );
                    compare_state(&mut py, &rig, &label);
                    for axis in rig.dc.get_axes() {
                        outcome(
                            &format!("{label} get_kin_range({axis})"),
                            py.eval(&format!("dc.get_kin_range(toolhead, {axis})")),
                            rig.dc
                                .get_kin_range(&toolhead, axis)
                                .map(|(lo, hi)| tup(vec![fv(lo), fv(hi)])),
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn ready_handler_applies_transforms() {
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        let mut steps = vec![Step::Ready];
        // Change the transforms, then apply them again through the handler.
        if let Some(first) = rig.dc.dc_rails().keys().last() {
            steps.push(Step::Pos([77.0, 20.0, 5.0, 0.0]));
            steps.push(Step::Act(first.clone(), "PRIMARY"));
            steps.push(Step::Ready);
        }
        run_steps(&mut py, &rig, &steps, &format!("ready[{name}]"));
    }
}

fn set_dual_script(names: &[String]) -> Vec<Step> {
    let mut steps = vec![cmd("SET_DUAL_CARRIAGE", &[])];
    for carriage in [
        "nope", "0", "1", "2", "-1", "+1", " 1", "01", "1_0", "x", "0.5",
        "99999999999999999999", "-99999999999999999999",
    ] {
        steps.push(cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", carriage)]));
    }
    for carriage in names {
        steps.push(cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", carriage.as_str())]));
        for mode in ["copy", "COPY", "MIRROR", "INACTIVE", "PRIMARY", "bogus", "Primary", "INACTIVE"] {
            steps.push(cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", carriage.as_str()), ("MODE", mode)]));
        }
    }
    for homed in ["yz", "", "x", "xyz"] {
        steps.push(Step::Homed(homed));
        for carriage in names {
            for mode in ["COPY", "MIRROR"] {
                steps.push(cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", carriage.as_str()), ("MODE", mode)]));
            }
        }
    }
    steps
}

#[test]
fn set_dual_carriage_command() {
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        let names: Vec<String> = rig.dc.dc_rails().keys().cloned().collect();
        run_steps(&mut py, &rig, &set_dual_script(&names), &format!("set[{name}]"));
    }
}

fn save_restore_script(names: &[String], dual_ind: usize) -> Vec<Step> {
    let p = names[0].as_str();
    let d = names[dual_ind].as_str();
    vec![
        cmd("SAVE_DUAL_CARRIAGE_STATE", &[]),
        Step::Pos([120.0, 20.0, 5.0, 0.0]),
        cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", d), ("MODE", "PRIMARY")]),
        cmd("SAVE_DUAL_CARRIAGE_STATE", &[("NAME", "b")]),
        Step::Pos([60.0, 20.0, 5.0, 0.0]),
        cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", p), ("MODE", "COPY")]),
        cmd("SAVE_DUAL_CARRIAGE_STATE", &[("NAME", "c")]),
        Step::Pos([150.0, 20.0, 5.0, 0.0]),
        cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", p), ("MODE", "MIRROR")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("NAME", "c"), ("MOVE_SPEED", "25.5")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("NAME", "b"), ("MOVE", "0")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[]),
        Step::Pos([30.0, 20.0, 5.0, 0.0]),
        cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", d), ("MODE", "MIRROR")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("NAME", "c")]),
        Step::SaveApi,
        cmd("SET_DUAL_CARRIAGE", &[("CARRIAGE", p), ("MODE", "PRIMARY")]),
        Step::RestoreApi(1, 0.0),
        Step::RestoreApi(0, 0.0),
        Step::RestoreApi(1, 33.0),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("NAME", "nope")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("MOVE_SPEED", "0")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("MOVE_SPEED", "-3")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("MOVE", "1.5")]),
        cmd("SAVE_DUAL_CARRIAGE_STATE", &[("NAME", "after")]),
        cmd("RESTORE_DUAL_CARRIAGE_STATE", &[("NAME", "after"), ("MOVE_SPEED", "10")]),
    ]
}

#[test]
fn save_and_restore_state() {
    for (name, sc) in [("idex", sc_idex()), ("same_dir", sc_same_dir()), ("xy", sc_xy())] {
        let Some((mut py, rig)) = start(&sc) else { return };
        let names: Vec<String> = rig.dc.dc_rails().keys().cloned().collect();
        let dual_ind = sc.primary.len();
        run_steps(&mut py, &rig, &save_restore_script(&names, dual_ind), &format!("restore[{name}]"));
    }
}

#[test]
fn home_axis_order_and_final_mode() {
    for (name, sc) in scenarios() {
        let Some((mut py, rig)) = start(&sc) else { return };
        for round in 0..2 {
            for axis in 0..3 {
                let label = format!("home[{name}] round {round} axis {axis}");
                let homing_state: HomingStateHandle = Arc::new(FakeHomingState);
                outcome(
                    &label,
                    py.eval(&format!("dc.home(object(), {axis})")),
                    unit(rig.dc.home(homing_state, axis)),
                );
                compare_state(&mut py, &rig, &label);
            }
            // The second round starts from a non-default state.
            let first = rig.dc.dc_rails().keys().last().cloned();
            if let Some(carriage) = first {
                run_steps(&mut py, &rig, &[Step::Act(carriage, "COPY")], &format!("home[{name}] prep"));
            }
        }
    }
}

struct WrapperCase {
    label: &'static str,
    axis: i64,
    active: bool,
}

#[test]
fn wrapper_direct_units() {
    let p1 = [10.0, 20.0, 5.0, 0.0];
    let p2 = [33.5, 44.5, 7.0, 1.0];
    let short = [1.0];
    let cases = [
        WrapperCase { label: "x_active", axis: 0, active: true },
        WrapperCase { label: "x_inactive", axis: 0, active: false },
        WrapperCase { label: "y_active", axis: 1, active: true },
        WrapperCase { label: "negative_axis", axis: -1, active: false },
        WrapperCase { label: "axis_two", axis: 2, active: true },
    ];
    for case in cases {
        let Some((mut py, rig)) = start(&sc_idex()) else { return };
        let label = format!("wrapper[{}]", case.label);
        py.exec(&format!(
            "w = idex.DualCarriagesRail(printer, prim[0], dual[0], {}, 50.0, {})",
            case.axis,
            pb(case.active)
        ))
        .expect("python wrapper");
        let wrapper = DualCarriagesRail::new(
            rig.printer_api.clone(),
            rig.prim[0].clone(),
            rig.dual[0].clone(),
            case.axis,
            Some(50.0),
            case.active,
        );
        let wrapper_state = |w: &DualCarriagesRail| {
            tup(vec![
                sv(&w.get_name()),
                sv(&w.mode()),
                fv(w.scale()),
                fv(w.offset()),
                opt_f(w.safe_dist()),
                iv(w.axis()),
                Value::Bool(w.is_active()),
            ])
        };
        let py_state = "(w.get_name(), w.mode, w.scale, w.offset, w.safe_dist, w.axis, w.is_active())";
        check_value(
            &format!("{label} initial"),
            &py.eval(py_state).expect("wrapper state"),
            &wrapper_state(&wrapper),
        );
        enum Op<'a> {
            AxisPos(&'a [f64]),
            Activate(&'static str, &'a [f64], Option<&'a [f64]>),
            Inactivate(&'a [f64]),
            Override(f64, &'a [f64]),
            Apply,
        }
        let ops = [
            Op::AxisPos(&p1),
            Op::Activate("MIRROR", &p1, None),
            Op::AxisPos(&p2),
            Op::Activate("COPY", &p2, Some(&p1)),
            Op::Override(2.5, &p2),
            Op::Apply,
            Op::Override(0.5, &p1),
            Op::Inactivate(&p2),
            Op::Override(3.0, &p1),
            Op::Activate("PRIMARY", &p1, None),
            Op::Activate("PRIMARY", &short, None),
            Op::Inactivate(&short),
            Op::AxisPos(&short),
        ];
        for (i, op) in ops.iter().enumerate() {
            let step_label = format!("{label} op {i}");
            let (py_expr, rust): (String, KResult<Value>) = match op {
                Op::AxisPos(pos) => (
                    format!("w.get_axis_position({})", pylist(pos)),
                    wrapper.get_axis_position(pos).map(fv),
                ),
                Op::Activate(mode, pos, old) => (
                    match old {
                        Some(old) => format!(
                            "w.activate({mode:?}, {}, old_position={})",
                            pylist(pos),
                            pylist(old)
                        ),
                        None => format!("w.activate({mode:?}, {})", pylist(pos)),
                    },
                    unit(wrapper.activate(mode, pos, *old)),
                ),
                Op::Inactivate(pos) => (
                    format!("w.inactivate({})", pylist(pos)),
                    unit(wrapper.inactivate(pos)),
                ),
                Op::Override(scale, pos) => (
                    format!("w.override_axis_scaling({}, {})", pf(*scale), pylist(pos)),
                    unit(wrapper.override_axis_scaling(*scale, pos)),
                ),
                Op::Apply => ("w.apply_transform()".to_string(), unit(wrapper.apply_transform())),
            };
            outcome(&step_label, py.eval(&py_expr), rust);
            check_value(
                &format!("{step_label} wrapper state"),
                &py.eval(py_state).expect("wrapper state"),
                &wrapper_state(&wrapper),
            );
            compare_state(&mut py, &rig, &step_label);
        }
    }
}

#[test]
fn second_instance_is_rejected_by_the_printer() {
    // `printer.add_object('dual_carriage', self)` raises when the name is
    // taken (the core printer's `config_error`); the port propagates it
    // after `_init_steppers` ran, exactly like Python.
    let Some((mut py, rig)) = start(&sc_idex()) else { return };
    let py_res = py.exec("dc2 = idex.DualCarriages(printer, prim, dual, axes=[0], safe_dist=[50.0])");
    let rust_res = DualCarriages::new(
        rig.printer_api.clone(),
        rig.prim.clone(),
        rig.dual.clone(),
        vec![0],
        vec![Some(50.0)],
    )
    .map(|_| Value::Null);
    if py_res.is_ok() || rust_res.is_ok() {
        panic!("second instance: oracle={py_res:?} rust={rust_res:?}");
    }
    outcome("second instance", py_res, rust_res);
    compare_state(&mut py, &rig, "second instance");
}
