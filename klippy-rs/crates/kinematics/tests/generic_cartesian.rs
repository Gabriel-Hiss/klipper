//! Parity tests for `kinematics::generic_cartesian`.
//!
//! Each test builds the Rust port with in-test fakes of the `klippy_api`
//! contract traits and the Python reference with minimal Python fakes in the
//! `pyoracle` namespace, then compares with `recording::assert_equal`.
//! Tests skip when `$KLIPPY_PYTHON` is unset (outside the oracle image).

use std::sync::Arc;

use indexmap::IndexMap;
use kos::sync::Mutex;
use pycompat::{PyInt, PyValue};
use recording::Value as RecValue;

use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetListsOpts, GetListOpts, GetIntListOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd,
    GetIntOpts as GGetIntOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{AddConfigCmdOpts, CommandQueue, CommandWrapper, Endstop, EndstopApi, Mcu, McuApi, SerialResponseCallback};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper,
    StepperFactory, StepperFactoryApi, StepperKinematics, Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, HomingStateHandle, KinematicsApi, LoadKinematics, MoveApi, PrinterExtruder, ToolHead,
    ToolHeadApi,
};
use klippy_api::{EventArg, ObjectRef};
use klippy_api::printer_object;
use reactor::{Completion, Reactor, ReactorMutex};

use kinematics::generic_cartesian::{DualCarriage, GenericCartesianKinematics, MainCarriage};
use kinematics::kinematic_stepper::{parse_carriages_string, Carriage as KCarriage};

// ---------------------------------------------------------------------------
// Recording helpers.
// ---------------------------------------------------------------------------

fn skip_if_no_oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(o) => Some(o),
        None => {
            println!("Skipping: $KLIPPY_PYTHON is not set (outside the oracle image).");
            None
        }
    }
}
fn assert_py_value_eq(py: &PyValue, rec: &RecValue) {
    // Compare Rust PyValue with oracle recording value.
    match (py, rec) {
        (PyValue::Str(a), RecValue::Str(b)) => assert_eq!(a.as_str(), b.as_str()),
        (PyValue::Float(a), RecValue::Float(fv)) => {
            // Compare by bits: `recording::f64_to_hex` trims trailing
            // zeros while CPython `float.hex()` pads, so hex strings differ.
            let py_f = recording::f64_from_hex(&fv.hex).expect("oracle hex parses");
            assert_eq!(a.to_bits(), py_f.to_bits());
            assert_eq!(&pycompat::text::float_repr(*a), &fv.repr);
        }
        (PyValue::Tuple(a), RecValue::Tuple(b)) => {
            assert_eq!(a.len(), b.len());
            for (x, y) in a.iter().zip(b.iter()) {
                assert_py_value_eq(x, y);
            }
        }
        (PyValue::List(a), RecValue::List(b)) => {
            assert_eq!(a.len(), b.len());
            for (x, y) in a.iter().zip(b.iter()) {
                assert_py_value_eq(x, y);
            }
        }
        (PyValue::Dict(a), RecValue::Dict(b)) => {
            assert_eq!(a.len(), b.len(), "dict size: rust={py:?} rec={rec:?}");
            for (k, v) in a.iter() {
                let PyValue::Str(key) = k else {
                    panic!("non-string key: rust={k:?}");
                };
                let (rk, rv) = b
                    .iter()
                    .find(|(rk, _)| matches!(rk, RecValue::Str(s) if s == key))
                    .unwrap_or_else(|| panic!("missing key {key:?} in rec={rec:?}"));
                assert_py_value_eq(k, rk);
                assert_py_value_eq(v, rv);
            }
        }
        (PyValue::None, RecValue::Null) => {}
        (PyValue::Int(a), RecValue::Int(b)) => assert_eq!(a, &PyInt::from_i64(*b)),
        (PyValue::Bool(a), RecValue::Bool(b)) => assert_eq!(a, b),
        _ => panic!("recording mismatch: rust={py:?} rec={rec:?}"),
    }
}

// ---------------------------------------------------------------------------
// Python fake prologue executed in the oracle.
// ---------------------------------------------------------------------------

const PY_PROLOGUE: &str = r#"
import sys, types
import collections

# --- stub chelper (kinematic_stepper / idex_modes need get_ffi) ---
chelper = types.ModuleType('chelper')
def _get_ffi():
    ffi_main = types.SimpleNamespace(NULL=None, gc=lambda *a: None, new=lambda *a: None)
    ffi_lib = types.SimpleNamespace()
    for n in ['generic_cartesian_stepper_alloc','generic_cartesian_stepper_set_coeffs',
              'dual_carriage_alloc','dual_carriage_set_sk','dual_carriage_set_transform','free']:
        setattr(ffi_lib, n, lambda *a, **k: None)
    return (ffi_main, ffi_lib)
chelper.get_ffi = _get_ffi
sys.modules['chelper'] = chelper

# --- fake config ---
class FakeConfigError(Exception):
    pass
class FakeConfigSentinel:
    pass

class FakeConfig:
    error = FakeConfigError
    def __init__(self, printer, section, opts, sections):
        self.printer = printer
        self.section = section
        self.opts = opts
        self.all_sections = sections
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def _missing(self, option, default):
        if default is FakeConfigSentinel:
            raise FakeConfigError("Option '%s' in section '%s' must be specified" % (option, self.section))
        return default
    def get(self, option, default=FakeConfigSentinel, note_valid=True):
        if option not in self.opts:
            return self._missing(option, default)
        return self.opts[option]
    def getint(self, option, default=FakeConfigSentinel, minval=None, maxval=None, note_valid=True):
        if option not in self.opts:
            v = self._missing(option, default)
            return v
        v = int(self.opts[option])
        if minval is not None and v < minval:
            raise FakeConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise FakeConfigError("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v
    def getfloat(self, option, default=FakeConfigSentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self.opts:
            return self._missing(option, default)
        v = float(self.opts[option])
        if minval is not None and v < minval:
            raise FakeConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise FakeConfigError("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise FakeConfigError("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise FakeConfigError("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v
    def getboolean(self, option, default=FakeConfigSentinel, note_valid=True):
        if option not in self.opts:
            return self._missing(option, default)
        return self.opts[option]
    def getchoice(self, option, choices, default=FakeConfigSentinel, note_valid=True):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise FakeConfigError("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.section))
        return choices[c]
    def get_prefix_sections(self, prefix):
        return [FakeConfig(self.printer, s, self.all_sections[s], self.all_sections) for s in self.all_sections if s.startswith(prefix)]
    def has_section(self, section):
        return section in self.all_sections
    def getsection(self, section):
        return FakeConfig(self.printer, section, self.all_sections.get(section, {}), self.all_sections)

def make_sections(spec):
    return dict(spec)

# --- fake printer ---
class FakePrinter:
    config_error = FakeConfigError
    command_error = type('CommandError', (Exception,), {})
    def __init__(self):
        self.objects = {}
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            # klippy sentinel raises; our tests always register gcode/toolhead first
            raise FakeConfigError("Unable to lookup object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=None):
        raise NotImplementedError('load_object stub')

# --- fake rails/steppers ---
class FakeMcu:
    _next_id = [0]
    def __init__(self):
        FakeMcu._next_id[0] += 1
        self.id = FakeMcu._next_id[0]
    def __repr__(self):
        return 'FakeMcu(%s)' % (self.id,)

class FakeEndstop:
    def __init__(self, name, mcu):
        self.name = name
        self.mcu = mcu
        self.steppers = []
    def add_stepper(self, s):
        self.steppers.append(s)
    def get_steppers(self):
        return list(self.steppers)
    def get_mcu(self):
        return self.mcu

class FakeRail:
    def __init__(self, config):
        self.name = config.get_name()
        self.steppers = []
        self.endstops = []
        self.position_min = config.getfloat('position_min', 0.)
        self.position_max = config.getfloat('position_max', above=self.position_min)
        self.position_endstop = config.getfloat('position_endstop')
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat('second_homing_speed', self.homing_speed/2., above=0.)
        self.homing_retract_speed = config.getfloat('homing_retract_speed', self.homing_speed, above=0.)
        self.homing_retract_dist = config.getfloat('homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean('homing_positive_dir', None)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise FakeConfigError("Unable to infer homing_positive_dir in section '%s'" % (config.get_name(),))
        self.endstop_pin = config.get('endstop_pin')
    def get_name(self, short=False):
        if short:
            if self.name.startswith('stepper'):
                return self.name[8:]
            return self.name.split()[-1]
        return self.name
    def get_range(self):
        return (self.position_min, self.position_max)
    def get_homing_info(self):
        return collections.namedtuple('homing_info', ['speed','position_endstop','retract_speed','retract_dist','positive_dir','second_homing_speed'])(self.homing_speed, self.position_endstop, self.homing_retract_speed, self.homing_retract_dist, self.homing_positive_dir, self.second_homing_speed)
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def add_stepper(self, stepper, endstop_pin=None, endstop_name=None):
        self.steppers.append(stepper)
        if endstop_pin is None and self.endstops:
            self.endstops[0][0].add_stepper(stepper)
            return
        es_name = endstop_name or stepper.get_name(short=True)
        es = FakeEndstop(es_name, stepper.get_mcu())
        es.add_stepper(stepper)
        self.endstops.append((es, es_name))
    def setup_itersolve(self, *a):
        pass
    def set_trapq(self, tq):
        pass
    def set_position(self, coord):
        pass
    def calc_position_from_coord(self, coord):
        return 0.
    def get_commanded_position(self):
        return 0.

class FakeStepper:
    _mcu = None
    _mcus = {}
    def __init__(self, config):
        self._name = config.get_name()
        self._sk = None
        self._trapq = None
        self.calls = []
        mcu_name = config.get('mcu', None)
        if mcu_name is None:
            if FakeStepper._mcu is None:
                FakeStepper._mcu = FakeMcu()
            self._mcu = FakeStepper._mcu
        else:
            if mcu_name not in FakeStepper._mcus:
                FakeStepper._mcus[mcu_name] = FakeMcu()
            self._mcu = FakeStepper._mcus[mcu_name]
    def get_mcu(self):
        return self._mcu
    def get_name(self, short=False):
        if short and self._name.startswith('stepper'):
            return self._name[8:]
        return self._name
    def get_oid(self):
        return 0
    def units_in_radians(self):
        return False
    def get_pulse_duration(self):
        return (None, False)
    def setup_default_pulse_duration(self, *a):
        pass
    def setup_itersolve(self, alloc_func, *params):
        self._sk = ('SK', alloc_func, params)
    def get_step_dist(self):
        return 0.01
    def get_rotation_distance(self):
        return (40., 200.)
    def set_rotation_distance(self, *a):
        pass
    def get_dir_inverted(self):
        return (False, False)
    def set_dir_inverted(self, *a):
        pass
    def calc_position_from_coord(self, coord):
        return 0.
    def set_position(self, coord):
        self.calls.append(('set_position', tuple(coord)))
    def get_commanded_position(self):
        return 0.
    def get_mcu_position(self, cmd_pos=None):
        return 0
    def get_past_mcu_position(self, print_time):
        return 0
    def mcu_to_commanded_position(self, mcu_pos):
        return 0.
    def dump_steps(self, *a):
        return ([], 0)
    def get_stepper_kinematics(self):
        return self._sk
    def set_stepper_kinematics(self, sk):
        old = self._sk
        self._sk = sk
        return old
    def note_homing_end(self):
        pass
    def get_trapq(self):
        return self._trapq
    def set_trapq(self, tq):
        self._trapq = tq
    def add_active_callback(self, cb):
        pass
    def is_active_axis(self, axis):
        return True

fake_stepper_mod = types.ModuleType('stepper')
fake_stepper_mod.GenericPrinterRail = FakeRail
fake_stepper_mod.PrinterStepper = FakeStepper
sys.modules['stepper'] = fake_stepper_mod

# --- fake toolhead / gcode ---
import gcode as real_gcode
class FakeToolhead:
    def __init__(self, printer, max_vel=100., max_accel=1000.):
        self.printer = printer
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.pos = [0., 0., 0., 0.]
        self.kin = None
        self.flushed = 0
    def get_max_velocity(self):
        return (self.max_vel, self.max_accel)
    def get_trapq(self):
        return object()
    def get_position(self):
        return list(self.pos)
    def flush_step_generation(self):
        self.flushed += 1
    def get_kinematics(self):
        return self.kin

class FakeGCode:
    def __init__(self):
        self.handlers = {}
        self.help = {}
    def register_command(self, cmd, func, desc=None):
        self.handlers[cmd] = func
        if desc is not None:
            self.help[cmd] = desc

class FakeGCmd:
    error = FakePrinter.command_error
    def __init__(self, params):
        self.params = params
    def get(self, name, default=None):
        return self.params.get(name, default)
    def get_int(self, name, default=None):
        if name not in self.params:
            return default
        return int(self.params[name])

# --- fake dual carriages (for DC tests) ---
class FakeDcRailWrapper:
    def __init__(self, rail):
        self.rail = rail
        self.mode = 'INACTIVE'
    def activate(self, mode, position, old_position=None):
        self.mode = mode
    def inactivate(self, position):
        self.mode = 'INACTIVE'
    def is_active(self):
        return self.mode != 'INACTIVE'
    def get_name(self):
        return self.rail.get_name()

class FakeDcModule:
    def __init__(self, printer, primary_rails, dual_rails, axes, safe_dist):
        self.printer = printer
        self.axes = sorted(set(axes))
        self.wrappers = {}
        self.primary_rails = primary_rails
        self.dual_rails = dual_rails
        for r in list(primary_rails) + [x for x in dual_rails if x is not None]:
            self.wrappers[id(r)] = FakeDcRailWrapper(r)
        self.saved = None
        self.restored = []
        printer.add_object('dual_carriage', self)
    def get_axes(self):
        return self.axes
    def get_transform(self, rail):
        return (1.0, 0.0)
    def is_active(self, rail):
        w = self.wrappers.get(id(rail))
        return w.is_active() if w is not None else False
    def get_dc_rail_wrapper(self, rail):
        return self.wrappers.get(id(rail))
    def home(self, homing_state, axis):
        homing_state.calls.append(('dc_home', axis))
    def save_dual_carriage_state(self):
        return {'modes': {}}
    def restore_dual_carriage_state(self, state, move=0.):
        self.restored.append((state, move))

fake_idex = types.ModuleType('idex_modes')
fake_idex.DualCarriages = FakeDcModule
fake_idex.PRIMARY = 'PRIMARY'
fake_idex.INACTIVE = 'INACTIVE'
sys.modules['kinematics.idex_modes'] = fake_idex

# import reference module with stubs in place
import kinematics.generic_cartesian as gc
import kinematics.kinematic_stepper as ks

def build_py_kin(sections, toolhead=None, root_opts=None):
    printer = FakePrinter()
    gcode_obj = FakeGCode()
    printer.add_object('gcode', gcode_obj)
    if toolhead is None:
        toolhead = FakeToolhead(printer)
    printer.add_object('toolhead', toolhead)
    config = FakeConfig(printer, 'kinematics', root_opts or {}, sections)
    kin = gc.GenericCartesianKinematics(toolhead, config)
    toolhead.kin = kin
    return kin, printer, toolhead, gcode_obj

def standard_sections():
    return {
        'carriage x': {'axis': 'x', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P0'},
        'carriage y': {'axis': 'y', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P1'},
        'carriage z': {'axis': 'z', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P2'},
        'stepper stepper_x': {'carriages': 'x'},
        'stepper stepper_y': {'carriages': 'y'},
        'stepper stepper_z': {'carriages': 'z'},
    }
"#;

fn py_setup(py: &mut pyoracle::Oracle) {
    py.exec(PY_PROLOGUE).expect("py prologue exec");
}

// ---------------------------------------------------------------------------
// Rust fakes of the contract traits.
// ---------------------------------------------------------------------------
impl EndstopApi for FakeEndstopImpl {
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn add_stepper(&self, stepper: Stepper) { self.steppers.lock().push(stepper); }
    fn get_steppers(&self) -> Vec<Stepper> { self.steppers.lock().clone() }
    fn home_start(&self, _print_time: f64, _sample_time: f64, _sample_count: i64, _rest_time: f64, _triggered: Option<bool>) -> Completion<bool> {
        panic!("FakeEndstopImpl::home_start not used")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> { Ok(0.0) }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> { Ok(0) }
}

struct FakeEndstopImpl {
    mcu: Mcu,
    steppers: Mutex<Vec<Stepper>>,
}

printer_object!(FakeEndstopImpl);

struct FakeMcuImpl { id: usize }

impl McuApi for FakeMcuImpl {
    fn get_name(&self) -> String { format!("mcu{}", self.id) }
    fn get_printer(&self) -> Printer { panic!("FakeMcuImpl::get_printer not used") }
    fn is_fileoutput(&self) -> bool { false }
    fn setup_pin(&self, _pin_type: &str, _params: klippy_api::mcu::PinParams) -> KResult<klippy_api::mcu::Pin> { panic!("not used") }
    fn create_oid(&self) -> KResult<i64> { Ok(0) }
    fn register_config_callback(&self, _cb: klippy_api::mcu::ConfigCallback) -> KResult<()> { Ok(()) }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> { Ok(()) }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, t: f64) -> i64 { t as i64 }
    fn min_schedule_time(&self) -> f64 { 0.001 }
    fn max_nominal_duration(&self) -> f64 { 1.0 }
    fn lookup_command(&self, _m: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> { panic!("not used") }
    fn lookup_query_command(&self, _m: &str, _r: &str, _o: klippy_api::mcu::LookupQueryCommandOpts) -> KResult<klippy_api::mcu::CommandQueryWrapper> { panic!("not used") }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(0) }
    fn register_serial_response(&self, _cb: SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> klippy_api::mcu::AsyncResponseWrapper { panic!("not used") }
    fn check_valid_response(&self, _m: &str) -> bool { true }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> { Ok(0.0) }
    fn print_time_to_clock(&self, t: f64) -> i64 { t as i64 }
    fn clock_to_print_time(&self, c: f64) -> f64 { c }
    fn estimated_print_time(&self, t: f64) -> f64 { t }
    fn clock32_to_clock64(&self, c: i64) -> i64 { c }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> { Ok((0.0, 0.0)) }
    fn get_status(&self, _t: Option<f64>) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn stats(&self, _t: f64) -> (bool, String) { (true, "".to_string()) }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> { panic!("not used") }
}

printer_object!(FakeMcuImpl);

struct FakeStepperImpl {
    name: String,
    mcu: Mcu,
    sk: Mutex<Option<StepperKinematics>>,
    trapq: Mutex<Option<Trapq>>,
    positions: Mutex<Vec<[f64; 3]>>,
    alloc: Mutex<Option<(String, Vec<PyValue>)>>,
}

impl McuStepperApi for FakeStepperImpl {
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn get_name(&self, short: Option<bool>) -> String {
        match short {
            Some(true) if self.name.starts_with("stepper") => self.name[8..].to_string(),
            _ => self.name.clone(),
        }
    }
    fn get_oid(&self) -> i64 { 0 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _d: f64, _b: bool) {}
    fn setup_itersolve(&self, alloc_func: &str, params: &[PyValue]) {
        *self.alloc.lock() = Some((alloc_func.to_string(), params.to_vec()));
    }
    fn get_step_dist(&self) -> f64 { 0.01 }
    fn get_rotation_distance(&self) -> (f64, f64) { (40.0, 200.0) }
    fn set_rotation_distance(&self, _d: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) { (false, false) }
    fn set_dir_inverted(&self, _d: bool) -> KResult<()> { Ok(()) }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 { 0.0 }
    fn set_position(&self, coord: [f64; 3]) { self.positions.lock().push(coord); }
    fn get_commanded_position(&self) -> f64 { 0.0 }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 { 0 }
    fn get_past_mcu_position(&self, _t: f64) -> i64 { 0 }
    fn mcu_to_commanded_position(&self, _p: i64) -> f64 { 0.0 }
    fn dump_steps(&self, _c: i64, _s: i64, _e: i64) -> (Vec<PullHistoryStep>, i64) { (Vec::new(), 0) }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> { self.sk.lock().clone() }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics> { let mut g = self.sk.lock(); let old = g.take(); *g = sk; old }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<Trapq> { self.trapq.lock().clone() }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> { let mut g = self.trapq.lock(); let old = g.take(); *g = tq; old }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _a: char) -> bool { true }
}

printer_object!(FakeStepperImpl);

struct FakeTrapqImpl;

impl TrapqApi for FakeTrapqImpl {
    fn as_any(&self) -> &dyn std::any::Any { self }
}
printer_object!(FakeTrapqImpl);

struct FakeRailImpl {
    name: String,
    position_min: f64,
    position_max: f64,
    position_endstop: f64,
    homing_speed: f64,
    second_homing_speed: f64,
    retract_speed: f64,
    retract_dist: f64,
    positive_dir: bool,
    steppers: Mutex<Vec<Stepper>>,
    endstops: Mutex<Vec<(Endstop, String)>>,
}

impl RailApi for FakeRailImpl {
    fn get_name(&self, short: Option<bool>) -> String {
        match short {
            Some(true) => {
                if self.name.starts_with("stepper") {
                    return self.name[8..].to_string();
                }
                self.name.split_whitespace().last().unwrap_or("").to_string()
            }
            _ => self.name.clone(),
        }
    }
    fn get_range(&self) -> (f64, f64) { (self.position_min, self.position_max) }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: self.homing_speed,
            position_endstop: self.position_endstop,
            retract_speed: self.retract_speed,
            retract_dist: self.retract_dist,
            positive_dir: self.positive_dir,
            second_homing_speed: self.second_homing_speed,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> { self.steppers.lock().clone() }
    fn get_endstops(&self) -> Vec<(Endstop, String)> { self.endstops.lock().clone() }
    fn lookup_endstop(&self, _pin: Option<&str>, name: &str) -> KResult<Endstop> {
        let mcu: Mcu = Arc::new(FakeMcuImpl { id: 0 });
        let es: Endstop = Arc::new(FakeEndstopImpl {
            mcu,
            steppers: Mutex::new(Vec::new()),
        });
        self.endstops.lock().push((es.clone(), name.to_string()));
        Ok(es)
    }
    fn add_stepper(&self, stepper: Stepper, endstop_pin: Option<&str>, endstop_name: Option<&str>) -> KResult<()> {
        self.steppers.lock().push(stepper.clone());
        if endstop_pin.is_none() {
            if let Some((es, _)) = self.endstops.lock().first() {
                es.add_stepper(stepper);
                return Ok(());
            }
        }
        let name = endstop_name.map(|s| s.to_string()).unwrap_or_else(|| stepper.get_name(Some(true)));
        let es: Endstop = Arc::new(FakeEndstopImpl {
            mcu: stepper.get_mcu(),
            steppers: Mutex::new(vec![stepper.clone()]),
        });
        self.endstops.lock().push((es, name));
        Ok(())
    }
    fn add_stepper_from_config(&self, _c: &Config) -> KResult<()> { Ok(()) }
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
    fn set_trapq(&self, _t: Option<Trapq>) {}
    fn set_position(&self, _c: [f64; 3]) {}
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 { 0.0 }
    fn get_commanded_position(&self) -> f64 { 0.0 }
}

printer_object!(FakeRailImpl);

fn make_fake_rail(name: &str, endstop: f64, max: f64, positive: bool) -> Rail {
    let r = FakeRailImpl {
        name: name.to_string(),
        position_min: 0.0,
        position_max: max,
        position_endstop: endstop,
        homing_speed: 5.0,
        second_homing_speed: 2.5,
        retract_speed: 5.0,
        retract_dist: 5.0,
        positive_dir: positive,
        steppers: Mutex::new(Vec::new()),
        endstops: Mutex::new(Vec::new()),
    };
    Arc::new(r)
}

struct FakeStepperFactoryImpl {
    mcu: Mcu,
    named_mcus: Mutex<IndexMap<String, Mcu>>,
}

impl FakeStepperFactoryImpl {
    fn new(mcu: Mcu) -> Self {
        FakeStepperFactoryImpl { mcu, named_mcus: Mutex::new(IndexMap::new()) }
    }
}

impl StepperFactoryApi for FakeStepperFactoryImpl {
    fn printer_stepper(&self, config: &Config, _r: bool) -> KResult<Stepper> {
        let mcu_name = config.get("mcu", GetStrOpts { default: Some(None), note_valid: true })?;
        let mcu: Mcu = match mcu_name {
            None => self.mcu.clone(),
            Some(n) => {
                let mut named = self.named_mcus.lock();
                let next_id = named.len() + 1;
                named
                    .entry(n)
                    .or_insert_with(|| {
                        let m: Mcu = Arc::new(FakeMcuImpl { id: next_id });
                        m
                    })
                    .clone()
            }
        };
        let s = FakeStepperImpl {
            name: config.get_name(),
            mcu,
            sk: Mutex::new(None),
            trapq: Mutex::new(None),
            positions: Mutex::new(Vec::new()),
            alloc: Mutex::new(None),
        };
        Ok(Arc::new(s))
    }
    fn lookup_rail(&self, config: &Config, _o: RailOpts) -> KResult<Rail> { self.generic_rail(config, _o) }
    fn lookup_multi_rail(&self, config: &Config, _o: RailOpts) -> KResult<Rail> { self.generic_rail(config, _o) }
    fn generic_rail(&self, config: &Config, _o: RailOpts) -> KResult<Rail> {
        let name = config.get_name();
        let max = config
            .getfloat(
                "position_max",
                GetFloatOpts {
                    default: None,
                    minval: None,
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(100.0);
        let endstop = config
            .getfloat(
                "position_endstop",
                GetFloatOpts {
                    default: None,
                    minval: None,
                    maxval: None,
                    above: None,
                    below: None,
                    note_valid: true,
                },
            )?
            .unwrap_or(0.0);
        let positive = endstop >= 75.0;
        Ok(make_fake_rail(&name, endstop, max, positive))
    }
    fn parse_step_distance(&self, _c: &Config, _u: Option<bool>, _n: bool) -> KResult<(f64, f64)> {
        Ok((40.0, 200.0))
    }
}

printer_object!(FakeStepperFactoryImpl);

// ---------------------------------------------------------------------------
// Config / Printer / Toolhead / GCode fakes.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Sec {
    name: String,
    opts: IndexMap<String, String>,
    floats: IndexMap<String, f64>,
}

struct FakeConfigImpl {
    printer: Printer,
    section: String,
    sections: Vec<Sec>,
}

impl FakeConfigImpl {
    fn find(&self, name: &str) -> Option<&Sec> {
        self.sections.iter().find(|s| s.name == name)
    }
    fn raw(&self, option: &str) -> Option<String> {
        match self.find(&self.section) {
            Some(s) => s.opts.get(option).cloned(),
            None => None,
        }
    }
    fn float_raw(&self, option: &str) -> Option<f64> {
        match self.find(&self.section) {
            Some(s) => s.floats.get(option).copied(),
            None => None,
        }
    }
}

impl ConfigApi for FakeConfigImpl {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { self.section.clone() }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: klippy_api::config::GetIntOpts) -> KResult<Option<i64>> {
        match self.raw(option) {
            Some(v) => {
                let n: i64 = v.parse().map_err(|_| {
                    KlippyError::config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    ))
                })?;
                if let Some(min) = opts.minval {
                    if n < min {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option, self.section, min
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if n > max {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option, self.section, max
                        )));
                    }
                }
                Ok(Some(n))
            }
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = self.float_raw(option).or_else(|| self.raw(option).and_then(|s| s.parse::<f64>().ok()));
        match raw {
            Some(v) => {
                if let Some(min) = opts.minval {
                    if v < min {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have minimum of {}",
                            option, self.section, text_minmax(min)
                        )));
                    }
                }
                if let Some(max) = opts.maxval {
                    if v > max {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must have maximum of {}",
                            option, self.section, text_minmax(max)
                        )));
                    }
                }
                if let Some(ab) = opts.above {
                    if v <= ab {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be above {}",
                            option, self.section, text_minmax(ab)
                        )));
                    }
                }
                if let Some(be) = opts.below {
                    if v >= be {
                        return Err(KlippyError::config(format!(
                            "Option '{}' in section '{}' must be below {}",
                            option, self.section, text_minmax(be)
                        )));
                    }
                }
                Ok(Some(v))
            }
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.raw(option) {
            Some(v) => {
                if v.eq_ignore_ascii_case("true") || v == "1" {
                    Ok(Some(true))
                } else if v.eq_ignore_ascii_case("false") || v == "0" {
                    Ok(Some(false))
                } else {
                    Err(KlippyError::config(format!(
                        "Unable to parse option '{}' in section '{}'",
                        option, self.section
                    )))
                }
            }
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be specified",
                    option, self.section
                ))),
                Some(d) => Ok(d),
            },
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw = self.raw(option);
        let value: PyValue = match raw {
            Some(v) => PyValue::Str(v),
            None => match opts.default {
                None => {
                    return Err(KlippyError::config(format!(
                        "Option '{}' in section '{}' must be specified",
                        option, self.section
                    )));
                }
                Some(d) => match d {
                    None => PyValue::None,
                    Some(v) => v,
                },
            },
        };
        // Validate membership: choices is List of Str/None or Dict name->name.
        let valid = match choices {
            PyValue::List(items) => items.iter().any(|it| py_equal_str(it, &value)),
            PyValue::Dict(d) => d.keys().any(|k| py_equal_str(k, &value)),
            _ => true,
        };
        if !valid {
            return Err(KlippyError::config(format!(
                "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                value.str(),
                option,
                self.section
            )));
        }
        if let PyValue::Dict(d) = choices {
            for (k, v) in d.iter() {
                if py_equal_str(k, &value) {
                    return Ok(v.clone());
                }
            }
        }
        Ok(value)
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> { panic!("not used") }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { panic!("not used") }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { panic!("not used") }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { panic!("not used") }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfigImpl {
            printer: self.printer.clone(),
            section: section.to_string(),
            sections: self.sections.clone(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.iter().any(|s| s.name == section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        let mut out = Vec::new();
        for s in &self.sections {
            if s.name.starts_with(prefix) {
                out.push(self.getsection(&s.name));
            }
        }
        out
    }
    fn get_prefix_options(&self, _p: &str) -> Vec<String> { Vec::new() }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) {}
}

fn text_minmax(f: f64) -> String { pycompat::text::float_repr(f) }

fn py_equal_str(a: &PyValue, b: &PyValue) -> bool {
    match (a, b) {
        (PyValue::None, PyValue::None) => true,
        (PyValue::Str(sa), PyValue::Str(sb)) => sa == sb,
        _ => false,
    }
}

printer_object!(FakeConfigImpl);

fn cfg_str(section: &str, opts: &[(&str, &str)]) -> Sec {
    let mut m = IndexMap::new();
    for (k, v) in opts { m.insert(k.to_string(), v.to_string()); }
    Sec { name: section.to_string(), opts: m, floats: IndexMap::new() }
}

fn cfg_flt(section: &str, opts: &[(&str, f64)], strs: &[(&str, &str)]) -> Sec {
    let mut m = IndexMap::new();
    for (k, v) in strs { m.insert(k.to_string(), v.to_string()); }
    let mut f = IndexMap::new();
    for (k, v) in opts { f.insert(k.to_string(), *v); }
    Sec { name: section.to_string(), opts: m, floats: f }
}
// ---------------------------------------------------------------------------
// Printer / Toolhead / GCode / Dc fakes.
// ---------------------------------------------------------------------------

struct FakePrinterImpl {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl PrinterApi for FakePrinterImpl {
    fn get_start_args(&self) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn get_reactor(&self) -> Reactor { self.reactor.clone() }
    fn get_state_message(&self) -> (String, String) { ("ready".to_string(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name).cloned() {
            Some(o) => Ok(Some(o)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!("Unable to lookup object '{}'", name))),
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.lock().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _c: &Config, _s: &str, _d: LookupDefault) -> KResult<Option<ObjectRef>> {
        panic!("load_object not used")
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _c: Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _r: &str) {}
}

printer_object!(FakePrinterImpl);

struct FakeGCodeImpl {
    handlers: Mutex<IndexMap<String, GCodeCommandHandler>>,
    descs: Mutex<IndexMap<String, String>>,
}

impl GCodeApi for FakeGCodeImpl {
    fn register_command(&self, cmd: &str, func: Option<GCodeCommandHandler>, opts: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> {
        if let Some(d) = opts.desc {
            self.descs.lock().insert(cmd.to_string(), d);
        }
        match func {
            Some(f) => {
                self.handlers.lock().insert(cmd.to_string(), f);
                Ok(None)
            }
            None => Ok(self.handlers.lock().shift_remove(cmd)),
        }
    }
    fn register_mux_command(&self, _c: &str, _k: &str, _v: Option<&str>, _f: GCodeCommandHandler, _d: Option<&str>) -> KResult<()> { Ok(()) }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _t: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn register_output_handler(&self, _c: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { panic!("not used") }
    fn create_gcode_command(&self, _c: &str, _cl: &str, _p: &IndexMap<String, String>) -> GCmd { panic!("not used") }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool { false }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> { Ok(()) }
}

printer_object!(FakeGCodeImpl; dyn GCodeApi);

struct FakeGCmdImpl {
    params: IndexMap<String, String>,
}

impl GCodeCommandApi for FakeGCmdImpl {
    fn get(&self, name: &str, opts: klippy_api::gcode::GetOpts) -> KResult<PyValue> {
        match self.params.get(name) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::command(format!("Parameter '{}' must be specified", name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        match self.params.get(name) {
            Some(v) => {
                let n: i64 = v.parse().map_err(|_| KlippyError::command(format!("Unable to parse parameter '{}'", name)))?;
                if let Some(min) = opts.minval {
                    if n < min { return Err(KlippyError::command(format!("Parameter '{}' must be at least {}", name, min))); }
                }
                if let Some(max) = opts.maxval {
                    if n > max { return Err(KlippyError::command(format!("Parameter '{}' must be at most {}", name, max))); }
                }
                Ok(Some(n))
            }
            None => match opts.default {
                None => Err(KlippyError::command(format!("Parameter '{}' must be specified", name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_float(&self, name: &str, opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        match self.params.get(name) {
            Some(v) => {
                let f: f64 = v.parse().map_err(|_| KlippyError::command(format!("Unable to parse parameter '{}'", name)))?;
                Ok(Some(f))
            }
            None => match opts.default {
                None => Err(KlippyError::command(format!("Parameter '{}' must be specified", name))),
                Some(d) => Ok(d),
            },
        }
    }
    fn get_command(&self) -> String { "SET_STEPPER_CARRIAGES".to_string() }
    fn get_commandline(&self) -> String { String::new() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool { true }
}

printer_object!(FakeGCmdImpl);

fn make_gcmd(params: &[(&str, &str)]) -> GCmd {
    let mut m = IndexMap::new();
    for (k, v) in params { m.insert(k.to_string(), v.to_string()); }
    Arc::new(FakeGCmdImpl { params: m })
}
// ---------------------------------------------------------------------------
// Toolhead / Move / HomingState / Dc fakes.
// ---------------------------------------------------------------------------

struct FakeToolheadImpl {
    factory: StepperFactory,
    max_vel: f64,
    max_accel: f64,
    pos: Mutex<Vec<f64>>,
    flushed: Mutex<usize>,
    trapq: Trapq,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolheadImpl {
    fn get_position(&self) -> Vec<f64> { self.pos.lock().clone() }
    fn set_position(&self, newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        *self.pos.lock() = newpos;
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics { panic!("FakeToolheadImpl::get_kinematics not used") }
    fn flush_step_generation(&self) -> KResult<()> {
        *self.flushed.lock() += 1;
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 { 0.0 }
    fn dwell(&self, _d: f64) {}
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &Completion<bool>) -> KResult<()> { Ok(()) }
}

impl ToolHeadApi for FakeToolheadImpl {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) {}
    fn get_status(&self, _t: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn get_trapq(&self) -> Trapq { self.trapq.clone() }
    fn get_max_velocity(&self) -> (f64, f64) { (self.max_vel, self.max_accel) }
    fn set_max_velocities(&self, _v: Option<f64>, _a: Option<f64>, _s: Option<f64>, _m: Option<f64>) -> (f64, f64, f64, f64) { (0.0, 0.0, 0.0, 0.0) }
    fn register_lookahead_callback(&self, _c: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> { Ok(()) }
    fn check_busy(&self, _t: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder { panic!("not used") }
    fn add_extra_axis(&self, _e: klippy_api::toolhead::ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _e: &klippy_api::toolhead::ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> { Vec::new() }
    fn stats(&self, _t: f64) -> (bool, String) { (true, "".to_string()) }
    fn stepper_factory(&self) -> StepperFactory { self.factory.clone() }
}

printer_object!(FakeToolheadImpl; dyn ToolHeadApi);

struct FakeMoveImpl {
    end_pos: Vec<f64>,
    axes_d: Vec<f64>,
    move_d: f64,
    limited: Mutex<Option<(f64, f64)>>,
}

impl MoveApi for FakeMoveImpl {
    fn start_pos(&self) -> Coord { Coord(vec![0.0, 0.0, 0.0, 0.0]) }
    fn end_pos(&self) -> Coord { Coord(self.end_pos.clone()) }
    fn axes_d(&self) -> Coord { Coord(self.axes_d.clone()) }
    fn axes_r(&self) -> Coord { Coord(vec![0.0, 0.0, 0.0, 0.0]) }
    fn move_d(&self) -> f64 { self.move_d }
    fn is_kinematic_move(&self) -> bool { true }
    fn accel(&self) -> f64 { 1000.0 }
    fn max_cruise_v2(&self) -> f64 { 10000.0 }
    fn start_v(&self) -> f64 { 0.0 }
    fn cruise_v(&self) -> f64 { 100.0 }
    fn accel_t(&self) -> f64 { 0.0 }
    fn cruise_t(&self) -> f64 { 0.0 }
    fn decel_t(&self) -> f64 { 0.0 }
    fn limit_speed(&self, speed: f64, accel: f64) {
        *self.limited.lock() = Some((speed, accel));
    }
    fn move_error(&self, msg: String) -> KlippyError {
        let ep = &self.end_pos;
        KlippyError::command(format!(
            "{}: {:.3} {:.3} {:.3} [{:.3}]",
            msg, ep[0], ep[1], ep[2], ep[3]
        ))
    }
}

printer_object!(FakeMoveImpl);

/// One `home_rails` call recorded by the fake homing state.
#[derive(Clone)]
struct HomeRailsCall {
    rails: Vec<String>,
    forcepos: Vec<Option<f64>>,
    homepos: Vec<Option<f64>>,
}

struct FakeHomingStateImpl {
    axes: Vec<i64>,
    calls: Mutex<Vec<HomeRailsCall>>,
}

impl klippy_api::toolhead::HomingStateApi for FakeHomingStateImpl {
    fn set_axes(&self, axes: Vec<i64>) { let _ = axes; }
    fn get_axes(&self) -> Vec<i64> { self.axes.clone() }
    fn get_trigger_position(&self, name: &str) -> KResult<i64> {
        Err(KlippyError::Internal { class_name: "KeyError".to_string(), message: format!("'{}'", name) })
    }
    fn set_stepper_adjustment(&self, _n: &str, _a: f64) {}
    fn set_homed_position(&self, _p: Vec<Option<f64>>) -> KResult<()> { Ok(()) }
    fn home_rails(&self, rails: Vec<Rail>, forcepos: Vec<Option<f64>>, movepos: Vec<Option<f64>>) -> KResult<()> {
        let names: Vec<String> = rails.iter().map(|r| r.get_name(Some(true))).collect();
        self.calls.lock().push(HomeRailsCall {
            rails: names,
            forcepos,
            homepos: movepos,
        });
        Ok(())
    }
}

printer_object!(FakeHomingStateImpl);

/// `[Some(150.0), None, None, None]` as the `PyValue` the reference records.
fn opt_float_list(v: &[Option<f64>]) -> PyValue {
    PyValue::List(
        v.iter()
            .map(|o| match o {
                Some(f) => PyValue::Float(*f),
                None => PyValue::None,
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Rust builders matching the Python `standard_sections()` spec.
// ---------------------------------------------------------------------------

fn rust_standard_sections() -> Vec<Sec> {
    vec![
        cfg_flt("carriage x", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "x"), ("endstop_pin", "P0")]),
        cfg_flt("carriage y", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "y"), ("endstop_pin", "P1")]),
        cfg_flt("carriage z", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "z"), ("endstop_pin", "P2")]),
        cfg_str("stepper stepper_x", &[("carriages", "x")]),
        cfg_str("stepper stepper_y", &[("carriages", "y")]),
        cfg_str("stepper stepper_z", &[("carriages", "z")]),
    ]
}

struct RustSetup {
    kin: Arc<GenericCartesianKinematics>,
    toolhead: Arc<FakeToolheadImpl>,
    gcode: Arc<FakeGCodeImpl>,
    trapq: Trapq,
}

struct FakeWorld {
    toolhead: ToolHead,
    tool_impl: Arc<FakeToolheadImpl>,
    gcode: Arc<FakeGCodeImpl>,
    trapq: Trapq,
    config: Config,
}

fn fake_world(sections: Vec<Sec>) -> FakeWorld {
    let printer: Printer = Arc::new(FakePrinterImpl {
        objects: Mutex::new(IndexMap::new()),
        reactor: Reactor::new(false),
    });
    let gcode: Arc<FakeGCodeImpl> = Arc::new(FakeGCodeImpl {
        handlers: Mutex::new(IndexMap::new()),
        descs: Mutex::new(IndexMap::new()),
    });
    let mcu: Mcu = Arc::new(FakeMcuImpl { id: 0 });
    let factory: StepperFactory = Arc::new(FakeStepperFactoryImpl::new(mcu.clone()));
    let trapq: Trapq = Arc::new(FakeTrapqImpl);
    let tool_impl = Arc::new(FakeToolheadImpl {
        factory,
        max_vel: 100.0,
        max_accel: 1000.0,
        pos: Mutex::new(vec![0.0, 0.0, 0.0, 0.0]),
        flushed: Mutex::new(0),
        trapq: trapq.clone(),
    });
    let toolhead: ToolHead = tool_impl.clone();
    let gcode_obj: ObjectRef = gcode.clone();
    let tool_obj: ObjectRef = tool_impl.clone();
    printer.add_object("gcode", gcode_obj).unwrap();
    printer.add_object("toolhead", tool_obj).unwrap();
    let config: Config = Arc::new(FakeConfigImpl {
        printer: printer.clone(),
        section: "kinematics".to_string(),
        sections,
    });
    FakeWorld {
        toolhead,
        tool_impl,
        gcode,
        trapq,
        config,
    }
}

fn build_rust_kin(sections: Vec<Sec>) -> RustSetup {
    let world = fake_world(sections);
    let kin = GenericCartesianKinematics::new(&world.toolhead, &world.config)
        .expect("kinematics build");
    RustSetup {
        kin,
        toolhead: world.tool_impl,
        gcode: world.gcode,
        trapq: world.trapq,
    }
}

fn build_rust_kin_plain(sections: Vec<Sec>) -> Result<RustSetup, KlippyError> {
    let world = fake_world(sections);
    let kin = GenericCartesianKinematics::new(&world.toolhead, &world.config)?;
    Ok(RustSetup {
        kin,
        toolhead: world.tool_impl,
        gcode: world.gcode,
        trapq: world.trapq,
    })
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn test_load_and_calc_position() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    let rs = build_rust_kin(rust_standard_sections());
    let py_kin = py.eval("build_py_kin(standard_sections())[0]").expect("build py kin");
    let _ = py_kin;
    // calc_position: identity carriages -> position equals stepper positions.
    let mut stepper_positions: IndexMap<String, f64> = IndexMap::new();
    stepper_positions.insert("stepper stepper_x".to_string(), 10.0);
    stepper_positions.insert("stepper stepper_y".to_string(), 20.0);
    stepper_positions.insert("stepper stepper_z".to_string(), 30.0);
    let rust_pos = rs.kin.calc_position(&stepper_positions).unwrap();
    assert_eq!(rust_pos.len(), 3);
    assert!((rust_pos[0].unwrap() - 10.0).abs() < 1e-9);
    assert!((rust_pos[1].unwrap() - 20.0).abs() < 1e-9);
    assert!((rust_pos[2].unwrap() - 30.0).abs() < 1e-9);
    let expected = py
        .eval("build_py_kin(standard_sections())[0].calc_position({'stepper stepper_x': 10.0, 'stepper stepper_y': 20.0, 'stepper stepper_z': 30.0})")
        .expect("py calc");
    if let RecValue::List(items) = expected {
        assert_eq!(items.len(), 3);
        for (rx, ex) in rust_pos.iter().zip(items.iter()) {
            match (rx, ex) {
                (Some(f), RecValue::Float(fv)) => {
                    let py_f = recording::f64_from_hex(&fv.hex).expect("oracle hex parses");
                    assert_eq!(f.to_bits(), py_f.to_bits());
                }
                (None, RecValue::Null) => {}
                _ => panic!("calc_position mismatch"),
            }
        }
    } else {
        panic!("unexpected py calc_position");
    }
    let mut missing: IndexMap<String, f64> = IndexMap::new();
    missing.insert("stepper stepper_x".to_string(), 1.0);
    let rust_err = rs.kin.calc_position(&missing).unwrap_err();
    assert_eq!(rust_err.class_name(), "KeyError");
    assert_eq!(rust_err.message(), "'stepper stepper_y'");
    let py_err = py
        .eval("build_py_kin(standard_sections())[0].calc_position({'stepper stepper_x': 1.0})")
        .expect_err("py key error");
    assert_eq!(py_err.class_name, "KeyError");
}

#[test]
fn test_load_kinematics_entry_point() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    // `load_kinematics(toolhead, config)` is the registered `LoadKinematics`.
    let world = fake_world(rust_standard_sections());
    let loader: LoadKinematics = kinematics::generic_cartesian::load_kinematics;
    let kin = loader(&world.toolhead, &world.config).expect("load_kinematics");
    py.exec("k = build_py_kin(standard_sections())[0]").expect("py kin");
    assert_py_value_eq(&kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status"));
    let py_len = py.eval("len(k.get_steppers())").expect("py steppers");
    let RecValue::Int(py_len) = py_len else { panic!("py stepper count shape") };
    assert_eq!(kin.get_steppers().len() as i64, py_len);
    // The loader runs the whole `__init__`, command registration included.
    assert!(world.gcode.handlers.lock().contains_key("SET_STEPPER_CARRIAGES"));
    let py_help = py
        .eval("k.printer.lookup_object('gcode').help['SET_STEPPER_CARRIAGES']")
        .expect("py help");
    let RecValue::Str(py_help) = py_help else { panic!("help shape") };
    assert_eq!(
        world.gcode.descs.lock().get("SET_STEPPER_CARRIAGES").map(String::as_str),
        Some(py_help.as_str()),
    );
}

#[test]
fn test_get_status_and_limits() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    let rs = build_rust_kin(rust_standard_sections());
    py.exec("k = build_py_kin(standard_sections())[0]").expect("py kin");
    // Fresh limits are un-homed and the ranges come from the carriages.
    assert_py_value_eq(&rs.kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status"));
    // set_position on 'xy' homes those axes.
    rs.kin.set_position(vec![1.0, 2.0, 3.0, 0.0], "xy").expect("rust set_position");
    py.exec("k.set_position([1.0, 2.0, 3.0, 0.0], 'xy')").expect("py set_position");
    assert_py_value_eq(&rs.kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status 2"));
    // clear_homing_state drops an axis back to the un-homed limits.
    rs.kin.clear_homing_state("x");
    py.exec("k.clear_homing_state('x')").expect("py clear_homing_state");
    assert_py_value_eq(&rs.kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status 3"));
    // update_limits on an un-homed axis is a no-op in the reference.
    rs.kin.update_limits(2, (0.0, 100.0)).expect("rust update_limits");
    py.exec("k.update_limits(2, (0.0, 100.0))").expect("py update_limits");
    assert_py_value_eq(&rs.kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status 4"));
    // After homing z through set_position, update_limits takes effect.
    rs.kin.set_position(vec![1.0, 2.0, 3.0, 0.0], "xyz").expect("rust set_position z");
    py.exec("k.set_position([1.0, 2.0, 3.0, 0.0], 'xyz')").expect("py set_position z");
    rs.kin.update_limits(2, (-5.0, 42.0)).expect("rust update_limits homed");
    py.exec("k.update_limits(2, (-5.0, 42.0))").expect("py update_limits homed");
    assert_py_value_eq(&rs.kin.get_status(0.0), &py.eval("k.get_status(0.0)").expect("py status 5"));
    // An unknown axis character is a ValueError in the reference.
    let err = rs.kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "q").expect_err("rust bad axis");
    let py_err = py.eval("k.set_position([0.0, 0.0, 0.0, 0.0], 'q')").expect_err("py bad axis");
    assert_eq!(err.class_name(), py_err.class_name);
    assert_eq!(err.message(), py_err.message);
}

#[test]
fn test_check_move() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    let rs = build_rust_kin(rust_standard_sections());
    rs.kin.set_position(vec![0.0, 0.0, 0.0, 0.0], "xyz").expect("rust homed");
    py.exec(concat!(
        "k = build_py_kin(standard_sections())[0]\n",
        "k.set_position([0.0, 0.0, 0.0, 0.0], 'xyz')\n",
        "class Move:\n",
        "    def __init__(self, printer, end_pos, axes_d, move_d):\n",
        "        self.printer = printer\n",
        "        self.end_pos = end_pos\n",
        "        self.axes_d = axes_d\n",
        "        self.move_d = move_d\n",
        "        self.limited = None\n",
        "    def move_error(self, msg='Move out of range'):\n",
        "        ep = self.end_pos\n",
        "        return self.printer.command_error('%s: %.3f %.3f %.3f [%.3f]' % (msg, ep[0], ep[1], ep[2], ep[3]))\n",
        "    def limit_speed(self, speed, accel):\n",
        "        self.limited = (speed, accel)\n",
    ))
    .expect("py move class");
    // In-range XY move: no speed limiting.
    let mv = FakeMoveImpl {
        end_pos: vec![10.0, 10.0, 0.0, 0.0],
        axes_d: vec![10.0, 0.0, 0.0, 0.0],
        move_d: 10.0,
        limited: Mutex::new(None),
    };
    rs.kin.check_move(&mv).expect("rust xy move");
    assert!(mv.limited.lock().is_none());
    py.exec("m_xy = Move(k.printer, (10.0, 10.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.0), 10.0)\nk.check_move(m_xy)")
        .expect("py xy move");
    assert!(matches!(py.eval("m_xy.limited").expect("py xy limited"), RecValue::Null));
    // Z move inside the limits caps speed for the slower Z axis.
    let mv2 = FakeMoveImpl {
        end_pos: vec![10.0, 10.0, 50.0, 0.0],
        axes_d: vec![0.0, 0.0, 50.0, 0.0],
        move_d: 50.0,
        limited: Mutex::new(None),
    };
    rs.kin.check_move(&mv2).expect("rust z move");
    let (sp, ac) = mv2.limited.lock().expect("rust z limited");
    py.exec("m_z = Move(k.printer, (10.0, 10.0, 50.0, 0.0), (0.0, 0.0, 50.0, 0.0), 50.0)\nk.check_move(m_z)")
        .expect("py z move");
    assert_py_value_eq(
        &PyValue::Tuple(vec![PyValue::Float(sp), PyValue::Float(ac)]),
        &py.eval("m_z.limited").expect("py z limited"),
    );
    // A diagonal move scales by the whole move length, not just the Z leg.
    let diag_d = 4100.0f64.sqrt();
    let mv2b = FakeMoveImpl {
        end_pos: vec![40.0, 0.0, 50.0, 0.0],
        axes_d: vec![40.0, 0.0, 50.0, 0.0],
        move_d: diag_d,
        limited: Mutex::new(None),
    };
    rs.kin.check_move(&mv2b).expect("rust diagonal move");
    let (sp2, ac2) = mv2b.limited.lock().expect("rust diagonal limited");
    py.exec(&format!(
        "m_diag = Move(k.printer, (40.0, 0.0, 50.0, 0.0), (40.0, 0.0, 50.0, 0.0), {diag_d:?})\nk.check_move(m_diag)"
    ))
    .expect("py diagonal move");
    assert_py_value_eq(
        &PyValue::Tuple(vec![PyValue::Float(sp2), PyValue::Float(ac2)]),
        &py.eval("m_diag.limited").expect("py diagonal limited"),
    );
    // Out-of-range move errors with 'Move out of range' and the position.
    let mv3 = FakeMoveImpl {
        end_pos: vec![200.0, 0.0, 0.0, 0.0],
        axes_d: vec![200.0, 0.0, 0.0, 0.0],
        move_d: 200.0,
        limited: Mutex::new(None),
    };
    let err = rs.kin.check_move(&mv3).expect_err("rust out of range");
    py.exec("m_out = Move(k.printer, (200.0, 0.0, 0.0, 0.0), (200.0, 0.0, 0.0, 0.0), 200.0)")
        .expect("py out move");
    let py_err = py.eval("k.check_move(m_out)").expect_err("py out of range");
    assert_eq!(err.class_name(), py_err.class_name);
    assert_eq!(err.message(), py_err.message);
    // An un-homed axis reports 'Must home axis first'.
    let un_homed = build_rust_kin(rust_standard_sections());
    let mv4 = FakeMoveImpl {
        end_pos: vec![10.0, 0.0, 0.0, 0.0],
        axes_d: vec![10.0, 0.0, 0.0, 0.0],
        move_d: 10.0,
        limited: Mutex::new(None),
    };
    let err4 = un_homed.kin.check_move(&mv4).expect_err("rust un-homed");
    py.exec("k_fresh = build_py_kin(standard_sections())[0]\nm_fresh = Move(k_fresh.printer, (10.0, 0.0, 0.0, 0.0), (10.0, 0.0, 0.0, 0.0), 10.0)")
        .expect("py fresh move");
    let py_err4 = py.eval("k_fresh.check_move(m_fresh)").expect_err("py un-homed");
    assert_eq!(err4.class_name(), py_err4.class_name);
    assert_eq!(err4.message(), py_err4.message);
}

#[test]
fn test_config_errors() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    // Missing axis carriage.
    let secs: Vec<Sec> = vec![
        cfg_flt("carriage x", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "x"), ("endstop_pin", "P0")]),
        cfg_flt("carriage y", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "y"), ("endstop_pin", "P1")]),
        cfg_str("stepper stepper_x", &[("carriages", "x")]),
        cfg_str("stepper stepper_y", &[("carriages", "y")]),
    ];
    let err = build_rust_kin_plain(secs).err().expect("build must fail");
    assert_eq!(err.class_name(), "Error");
    assert_eq!(err.message(), "No carriage defined for axis 'z'");
    let py_err = py
        .eval("build_py_kin({'carriage x': {'axis': 'x', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P0'}, 'carriage y': {'axis': 'y', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P1'}, 'stepper stepper_x': {'carriages': 'x'}, 'stepper stepper_y': {'carriages': 'y'}})")
        .expect_err("py missing z");
    assert_eq!(py_err.class_name, "FakeConfigError");
    assert_eq!(py_err.message, "No carriage defined for axis 'z'");
    // Duplicate axis across two primary carriages: the reference reports the
    // duplicate before any stepper reference check.
    let mut secs_dup = rust_standard_sections();
    secs_dup.push(cfg_flt("carriage extra_x", &[("position_endstop", 0.0), ("position_max", 100.0)], &[("axis", "x"), ("endstop_pin", "P9")]));
    let err_dup = build_rust_kin_plain(secs_dup).err().expect("build must fail");
    assert_eq!(err_dup.class_name(), "Error");
    py.exec("s_dup = standard_sections()\ns_dup['carriage extra_x'] = {'axis': 'x', 'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P9'}")
        .expect("py dup sections");
    let py_err_dup = py.eval("build_py_kin(s_dup)").expect_err("py duplicate axis");
    assert_eq!(py_err_dup.class_name, "FakeConfigError");
    assert_eq!(err_dup.message(), py_err_dup.message);
    // Carriage that no stepper references.
    let mut secs_unref = rust_standard_sections();
    secs_unref.push(cfg_str("extra_carriage extra", &[("primary_carriage", "x"), ("endstop_pin", "P9")]));
    let err_unref = build_rust_kin_plain(secs_unref).err().expect("build must fail");
    assert_eq!(err_unref.class_name(), "Error");
    py.exec("s_unref = standard_sections()\ns_unref['extra_carriage extra'] = {'primary_carriage': 'x', 'endstop_pin': 'P9'}")
        .expect("py unref sections");
    let py_err_unref = py.eval("build_py_kin(s_unref)").expect_err("py unreferenced carriage");
    assert_eq!(py_err_unref.class_name, "FakeConfigError");
    assert_eq!(err_unref.message(), py_err_unref.message);
}

// ---------------------------------------------------------------------------
// `DualCarriage` config handling (built standalone, without `idex_modes`).
// ---------------------------------------------------------------------------

fn test_stepper_factory() -> StepperFactory {
    let mcu: Mcu = Arc::new(FakeMcuImpl { id: 0 });
    Arc::new(FakeStepperFactoryImpl::new(mcu))
}

fn standalone_config(section: &str, sections: Vec<Sec>) -> Config {
    let printer: Printer = Arc::new(FakePrinterImpl {
        objects: Mutex::new(IndexMap::new()),
        reactor: Reactor::new(false),
    });
    Arc::new(FakeConfigImpl {
        printer,
        section: section.to_string(),
        sections,
    })
}

/// A `dual_carriage` section carrying the required rail options.
fn dc_sec(floats: &[(&str, f64)], strs: &[(&str, &str)]) -> Sec {
    let mut f: Vec<(&str, f64)> = vec![("position_endstop", 0.0), ("position_max", 100.0)];
    f.extend_from_slice(floats);
    let mut s: Vec<(&str, &str)> = vec![("endstop_pin", "P5")];
    s.extend_from_slice(strs);
    cfg_flt("dual_carriage dx", &f, &s)
}

/// `dc_try` for the Rust port: `(axis, safe_dist, dc name, primary name)`.
fn rust_dc_try(
    dc: Sec,
    primary_name: &str,
    primary_axis: &str,
) -> Result<PyValue, KlippyError> {
    let factory = test_stepper_factory();
    let psec = format!("carriage {}", primary_name);
    let main_sec = cfg_flt(
        &psec,
        &[("position_endstop", 0.0), ("position_max", 100.0)],
        &[("axis", primary_axis), ("endstop_pin", "P0")],
    );
    let main_cfg = standalone_config(&psec, vec![main_sec]);
    let main = MainCarriage::new(&main_cfg, &factory)?;
    let dc_name = dc.name.clone();
    let dc_cfg = standalone_config(&dc_name, vec![dc]);
    let dc = DualCarriage::new(&dc_cfg, &factory)?;
    let mut map: IndexMap<String, KCarriage> = IndexMap::new();
    let m: KCarriage = main.clone();
    map.insert(primary_name.to_string(), m);
    dc.resolve_primary_carriage(&map)?;
    let primary_name_out = match dc.get_primary_carriage() {
        Some(c) => PyValue::Str(
            c.as_any()
                .downcast_ref::<MainCarriage>()
                .expect("primary carriage")
                .get_name(),
        ),
        None => PyValue::None,
    };
    Ok(PyValue::Tuple(vec![
        PyValue::Int(PyInt::from_i64(dc.get_axis())),
        match dc.get_safe_dist() {
            Some(d) => PyValue::Float(d),
            None => PyValue::None,
        },
        match main.get_dual_carriage() {
            Some(d) => PyValue::Str(d.get_name()),
            None => PyValue::None,
        },
        primary_name_out,
    ]))
}

fn assert_dc_case(
    py: &mut pyoracle::Oracle,
    dc: Sec,
    primary_name: &str,
    primary_axis: &str,
    py_opts: &str,
) {
    let rust = rust_dc_try(dc, primary_name, primary_axis);
    let py_res = py
        .eval(&format!(
            "dc_try({py_opts}, {primary_name:?}, {primary_axis:?})"
        ))
        .expect("py dc result");
    match rust {
        Ok(v) => {
            assert!(
                matches!(py_res, RecValue::Tuple(_)),
                "reference raised for {py_opts}: {py_res:?} (rust {v:?})"
            );
            assert_py_value_eq(&v, &py_res);
        }
        Err(e) => {
            assert_eq!(e.class_name(), "Error", "case {py_opts}");
            let RecValue::Str(py_msg) = py_res else {
                panic!("reference did not raise for {py_opts}")
            };
            assert_eq!(e.message(), py_msg, "case {py_opts}");
        }
    }
}

#[test]
fn test_dual_carriage_config() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    py.exec(concat!(
        "PRIMARY_OPTS = {'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P0'}\n",
        "DC_BASE = {'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P5'}\n",
        "def dc_try(opts, primary_name='x', primary_axis='x'):\n",
        "    printer = FakePrinter()\n",
        "    psec = 'carriage ' + primary_name\n",
        "    popts = dict(PRIMARY_OPTS)\n",
        "    popts['axis'] = primary_axis\n",
        "    dcopts = dict(DC_BASE)\n",
        "    dcopts.update(opts)\n",
        "    try:\n",
        "        main = gc.MainCarriage(FakeConfig(printer, psec, popts, {psec: popts}))\n",
        "        dc = gc.DualCarriage(FakeConfig(printer, 'dual_carriage dx', dcopts, {'dual_carriage dx': dcopts}))\n",
        "        dc.resolve_primary_carriage({primary_name: main})\n",
        "        dcm = main.get_dual_carriage()\n",
        "        pcm = dc.get_primary_carriage()\n",
        "        return (dc.axis, dc.safe_dist,\n",
        "                dcm.get_name() if dcm is not None else None,\n",
        "                pcm.get_name() if pcm is not None else None)\n",
        "    except FakeConfigError as e:\n",
        "        return str(e)\n",
    ))
    .expect("py dc helper");
    // Axis comes from the primary carriage unless `axis` overrides it.
    assert_dc_case(
        &mut py,
        dc_sec(&[], &[("primary_carriage", "x")]),
        "x",
        "x",
        r#"{"primary_carriage": "x"}"#,
    );
    assert_dc_case(&mut py, dc_sec(&[], &[("axis", "x")]), "x", "x", r#"{"axis": "x"}"#);
    // `safe_distance` is only accepted together with `primary_carriage`.
    assert_dc_case(
        &mut py,
        dc_sec(&[("safe_distance", 5.0)], &[("primary_carriage", "x")]),
        "x",
        "x",
        r#"{"safe_distance": 5.0, "primary_carriage": "x"}"#,
    );
    assert_dc_case(
        &mut py,
        dc_sec(&[("safe_distance", -1.0)], &[("primary_carriage", "x")]),
        "x",
        "x",
        r#"{"safe_distance": -1.0, "primary_carriage": "x"}"#,
    );
    // Reference rejects an unusable or inconsistent configuration.
    assert_dc_case(&mut py, dc_sec(&[], &[]), "x", "x", "{}");
    assert_dc_case(
        &mut py,
        dc_sec(&[], &[("primary_carriage", "nope")]),
        "x",
        "x",
        r#"{"primary_carriage": "nope"}"#,
    );
    assert_dc_case(
        &mut py,
        dc_sec(&[], &[("primary_carriage", "y"), ("axis", "x")]),
        "y",
        "y",
        r#"{"primary_carriage": "y", "axis": "x"}"#,
    );
    assert_dc_case(
        &mut py,
        dc_sec(&[], &[("primary_carriage", "z")]),
        "z",
        "z",
        r#"{"primary_carriage": "z"}"#,
    );
    assert_dc_case(
        &mut py,
        dc_sec(&[], &[("axis", "w")]),
        "x",
        "x",
        r#"{"axis": "w"}"#,
    );
    // A second dual carriage for the same primary is rejected.
    py.exec(concat!(
        "def dc_multi():\n",
        "    printer = FakePrinter()\n",
        "    popts = dict(PRIMARY_OPTS)\n",
        "    popts['axis'] = 'x'\n",
        "    main = gc.MainCarriage(FakeConfig(printer, 'carriage x', popts, {'carriage x': popts}))\n",
        "    o1 = dict(DC_BASE, primary_carriage='x')\n",
        "    o2 = dict(DC_BASE, primary_carriage='x')\n",
        "    d1 = gc.DualCarriage(FakeConfig(printer, 'dual_carriage dx', o1, {'dual_carriage dx': o1}))\n",
        "    d2 = gc.DualCarriage(FakeConfig(printer, 'dual_carriage dz', o2, {'dual_carriage dz': o2}))\n",
        "    d1.resolve_primary_carriage({'x': main})\n",
        "    try:\n",
        "        d2.resolve_primary_carriage({'x': main})\n",
        "        return 'no error'\n",
        "    except FakeConfigError as e:\n",
        "        return str(e)\n",
    ))
    .expect("py dc multi");
    let factory = test_stepper_factory();
    let main_cfg = standalone_config(
        "carriage x",
        vec![cfg_flt(
            "carriage x",
            &[("position_endstop", 0.0), ("position_max", 100.0)],
            &[("axis", "x"), ("endstop_pin", "P0")],
        )],
    );
    let main = MainCarriage::new(&main_cfg, &factory).expect("main carriage");
    let mut map: IndexMap<String, KCarriage> = IndexMap::new();
    let m: KCarriage = main.clone();
    map.insert("x".to_string(), m);
    let d1 = DualCarriage::new(
        &standalone_config(
            "dual_carriage dx",
            vec![cfg_flt(
                "dual_carriage dx",
                &[("position_endstop", 0.0), ("position_max", 100.0)],
                &[("primary_carriage", "x"), ("endstop_pin", "P5")],
            )],
        ),
        &factory,
    )
    .expect("dc dx");
    let d2 = DualCarriage::new(
        &standalone_config(
            "dual_carriage dz",
            vec![cfg_flt(
                "dual_carriage dz",
                &[("position_endstop", 0.0), ("position_max", 100.0)],
                &[("primary_carriage", "x"), ("endstop_pin", "P5")],
            )],
        ),
        &factory,
    )
    .expect("dc dz");
    d1.resolve_primary_carriage(&map).expect("first dual carriage");
    let err_multi = d2.resolve_primary_carriage(&map).expect_err("second dual carriage");
    let py_multi = py.eval("dc_multi()").expect("py dc multi result");
    let RecValue::Str(py_multi) = py_multi else { panic!("py dc multi shape") };
    assert_eq!(err_multi.class_name(), "Error");
    assert_eq!(err_multi.message(), py_multi);
}

/// A `dual_carriage` section spec: name and extra options.
type DcSpec<'a> = (&'a str, Vec<(&'a str, &'a str)>);

/// Builds the carriage chain of `specs` over one `carriage x` and reports
/// the first resolution error the way `dc_chain` does on the Python side.
fn rust_dc_chain(specs: &[DcSpec]) -> String {
    let factory = test_stepper_factory();
    let main_cfg = standalone_config(
        "carriage x",
        vec![cfg_flt(
            "carriage x",
            &[("position_endstop", 0.0), ("position_max", 100.0)],
            &[("axis", "x"), ("endstop_pin", "P0")],
        )],
    );
    let main = MainCarriage::new(&main_cfg, &factory).expect("main carriage");
    let mut map: IndexMap<String, KCarriage> = IndexMap::new();
    let main_k: KCarriage = main;
    map.insert("x".to_string(), main_k);
    let mut dcs: Vec<Arc<DualCarriage>> = Vec::new();
    for (name, extra) in specs {
        let section = format!("dual_carriage {name}");
        let mut strs: Vec<(&str, &str)> = vec![("endstop_pin", "P5")];
        strs.extend(extra.iter().copied());
        let sec = cfg_flt(
            &section,
            &[("position_endstop", 0.0), ("position_max", 100.0)],
            &strs,
        );
        let dc = DualCarriage::new(&standalone_config(&section, vec![sec]), &factory).expect("dc");
        let dc_k: KCarriage = dc.clone();
        map.insert(name.to_string(), dc_k);
        dcs.push(dc);
    }
    for dc in &dcs {
        if let Err(e) = dc.resolve_primary_carriage(&map) {
            return if e.class_name() == "Error" {
                e.message().to_string()
            } else {
                format!("{}: {}", e.class_name(), e.message())
            };
        }
    }
    "no error".to_string()
}

#[test]
fn test_dual_carriage_chain() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    py.exec(concat!(
        "PRIMARY_OPTS = {'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P0'}\n",
        "DC_BASE = {'position_endstop': 0.0, 'position_max': 100.0, 'endstop_pin': 'P5'}\n",
        "def dc_chain(specs):\n",
        "    printer = FakePrinter()\n",
        "    popts = dict(PRIMARY_OPTS, axis='x')\n",
        "    main = gc.MainCarriage(FakeConfig(printer, 'carriage x', popts, {'carriage x': popts}))\n",
        "    carriages = {'x': main}\n",
        "    dcs = []\n",
        "    for name, extra in specs:\n",
        "        o = dict(DC_BASE, **extra)\n",
        "        sec = 'dual_carriage ' + name\n",
        "        d = gc.DualCarriage(FakeConfig(printer, sec, o, {sec: o}))\n",
        "        carriages[name] = d\n",
        "        dcs.append(d)\n",
        "    try:\n",
        "        for d in dcs:\n",
        "            d.resolve_primary_carriage(carriages)\n",
        "        return 'no error'\n",
        "    except FakeConfigError as e:\n",
        "        return str(e)\n",
        "    except TypeError as e:\n",
        "        return 'TypeError: ' + str(e)\n",
    ))
    .expect("py dc chain helper");
    let cases: Vec<(&str, Vec<DcSpec>)> = vec![
        // A dual carriage without `axis` as the primary of another one has
        // no `axis_name` to inherit, so `ord(None)` raises in the reference.
        (
            "[('dx', {'primary_carriage': 'x'}), ('dy', {'primary_carriage': 'dx'})]",
            vec![
                ("dx", vec![("primary_carriage", "x")]),
                ("dy", vec![("primary_carriage", "dx")]),
            ],
        ),
        // An explicit `axis` is compared with that missing `axis_name`.
        (
            "[('dx', {'primary_carriage': 'x'}), ('dy', {'primary_carriage': 'dx', 'axis': 'x'})]",
            vec![
                ("dx", vec![("primary_carriage", "x")]),
                ("dy", vec![("primary_carriage", "dx"), ("axis", "x")]),
            ],
        ),
        // A dual carriage that names itself as primary sees its own
        // `get_dual_carriage()` and reports a duplicate.
        (
            "[('dx', {'primary_carriage': 'dx', 'axis': 'x'})]",
            vec![("dx", vec![("primary_carriage", "dx"), ("axis", "x")])],
        ),
        // A standalone dual carriage (own `axis`, no primary) can be the
        // primary of another one.
        (
            "[('dx', {'axis': 'x'}), ('dy', {'primary_carriage': 'dx'})]",
            vec![
                ("dx", vec![("axis", "x")]),
                ("dy", vec![("primary_carriage", "dx")]),
            ],
        ),
    ];
    for (py_specs, rust_specs) in &cases {
        let rust = rust_dc_chain(rust_specs);
        let py_res = py.eval(&format!("dc_chain({py_specs})")).expect("py dc chain");
        let RecValue::Str(py_res) = py_res else { panic!("py dc chain shape") };
        assert_eq!(rust, py_res, "case {py_specs}");
    }
}

#[test]
fn test_multi_mcu_homing() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    py.exec(concat!(
        "def build_err(sections):\n",
        "    try:\n",
        "        build_py_kin(sections)\n",
        "        return 'no error'\n",
        "    except FakeConfigError as e:\n",
        "        return str(e)\n",
    ))
    .expect("py build_err");
    let with_mcu = |name: &str, mcu: &str| -> Vec<Sec> {
        let mut secs = rust_standard_sections();
        for s in secs.iter_mut() {
            if s.name == name {
                s.opts.insert("mcu".to_string(), mcu.to_string());
            }
        }
        secs
    };
    let check = |py: &mut pyoracle::Oracle, secs: Vec<Sec>, py_code: &str| -> String {
        py.exec(&format!("s_case = standard_sections()\n{py_code}")).expect("py sections");
        let py_res = py.eval("build_err(s_case)").expect("py build_err result");
        let RecValue::Str(py_res) = py_res else { panic!("py build_err shape") };
        match build_rust_kin_plain(secs) {
            Ok(_) => assert_eq!(py_res, "no error", "reference raised where the port did not"),
            Err(e) => {
                assert_eq!(e.class_name(), "Error");
                assert_eq!(e.message(), py_res);
            }
        }
        py_res
    };
    // Two steppers on different MCUs share the endstop of carriage `x`.
    let mut secs = with_mcu("stepper stepper_x", "mcu_a");
    secs.push(cfg_str("stepper stepper_x1", &[("carriages", "x"), ("mcu", "mcu_b")]));
    let res = check(
        &mut py,
        secs,
        "s_case['stepper stepper_x']['mcu'] = 'mcu_a'\ns_case['stepper stepper_x1'] = {'carriages': 'x', 'mcu': 'mcu_b'}",
    );
    assert!(res.starts_with("Multi-mcu homing not supported on multi-mcu shared carriage"), "{res}");
    // The same MCU on both steppers is accepted.
    let mut secs = with_mcu("stepper stepper_x", "mcu_c");
    secs.push(cfg_str("stepper stepper_x1", &[("carriages", "x"), ("mcu", "mcu_c")]));
    let res = check(
        &mut py,
        secs,
        "s_case['stepper stepper_x']['mcu'] = 'mcu_c'\ns_case['stepper stepper_x1'] = {'carriages': 'x', 'mcu': 'mcu_c'}",
    );
    assert_eq!(res, "no error");
    // An extra carriage with its own endstop keeps each endstop on one MCU.
    let mut secs = with_mcu("stepper stepper_x", "mcu_a");
    secs.push(cfg_str("extra_carriage ex", &[("primary_carriage", "x"), ("endstop_pin", "P9")]));
    secs.push(cfg_str("stepper stepper_x1", &[("carriages", "ex"), ("mcu", "mcu_b")]));
    let res = check(
        &mut py,
        secs,
        "s_case['stepper stepper_x']['mcu'] = 'mcu_a'\ns_case['extra_carriage ex'] = {'primary_carriage': 'x', 'endstop_pin': 'P9'}\ns_case['stepper stepper_x1'] = {'carriages': 'ex', 'mcu': 'mcu_b'}",
    );
    assert_eq!(res, "no error");
}

#[test]
fn test_load_config_and_command() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    // Defaults come from `toolhead.get_max_velocity()`.
    let rs = build_rust_kin(rust_standard_sections());
    py.exec("k_def = build_py_kin(standard_sections())[0]").expect("py default kin");
    assert_py_value_eq(
        &PyValue::Tuple(vec![
            PyValue::Float(rs.kin.get_max_z_velocity()),
            PyValue::Float(rs.kin.get_max_z_accel()),
        ]),
        &py.eval("(k_def.max_z_velocity, k_def.max_z_accel)").expect("py default limits"),
    );
    // Explicit options are read from the `kinematics` section.
    let mut secs = rust_standard_sections();
    secs.push(cfg_str("kinematics", &[("max_z_velocity", "5"), ("max_z_accel", "250")]));
    let rs_exp = build_rust_kin(secs);
    py.exec("k_exp = build_py_kin(standard_sections(), root_opts={'max_z_velocity': '5', 'max_z_accel': '250'})[0]")
        .expect("py explicit kin");
    assert_py_value_eq(
        &PyValue::Tuple(vec![
            PyValue::Float(rs_exp.kin.get_max_z_velocity()),
            PyValue::Float(rs_exp.kin.get_max_z_accel()),
        ]),
        &py.eval("(k_exp.max_z_velocity, k_exp.max_z_accel)").expect("py explicit limits"),
    );
    // Out-of-range options raise the reference's config error (`above=0.` and
    // `maxval=max_velocity`).
    py.exec(concat!(
        "def cfg_err(opts):\n",
        "    try:\n",
        "        build_py_kin(standard_sections(), root_opts=opts)\n",
        "        return 'no error'\n",
        "    except FakeConfigError as e:\n",
        "        return str(e)\n",
    ))
    .expect("py cfg_err");
    for (opt, val) in [
        ("max_z_velocity", "0"),
        ("max_z_velocity", "200"),
        ("max_z_accel", "0"),
        ("max_z_accel", "2000"),
    ] {
        let mut secs = rust_standard_sections();
        secs.push(cfg_str("kinematics", &[(opt, val)]));
        let err = build_rust_kin_plain(secs).err().expect("build must fail");
        assert_eq!(err.class_name(), "Error");
        let py_msg = py.eval(&format!("cfg_err({{'{opt}': '{val}'}})")).expect("py cfg error");
        let RecValue::Str(py_msg) = py_msg else { panic!("py cfg error shape") };
        assert_eq!(err.message(), py_msg, "case {opt}={val}");
    }
    // Kinematics validation runs before the `max_z_*` options are read, so a
    // stepper map that cannot move every axis independently wins over an
    // invalid `max_z_velocity`.
    let mut secs_bad = rust_standard_sections();
    secs_bad.retain(|s| !s.name.starts_with("stepper "));
    secs_bad.push(cfg_str("stepper stepper_a", &[("carriages", "x+y")]));
    secs_bad.push(cfg_str("stepper stepper_b", &[("carriages", "x+y")]));
    secs_bad.push(cfg_str("stepper stepper_c", &[("carriages", "z")]));
    secs_bad.push(cfg_str("kinematics", &[("max_z_velocity", "0")]));
    let err_bad = build_rust_kin_plain(secs_bad).err().expect("build must fail");
    py.exec(concat!(
        "s_bad = standard_sections()\n",
        "for k_ in [k for k in list(s_bad) if k.startswith('stepper ')]:\n",
        "    del s_bad[k_]\n",
        "s_bad['stepper stepper_a'] = {'carriages': 'x+y'}\n",
        "s_bad['stepper stepper_b'] = {'carriages': 'x+y'}\n",
        "s_bad['stepper stepper_c'] = {'carriages': 'z'}\n",
    ))
    .expect("py bad sections");
    let py_err_bad = py
        .eval("build_py_kin(s_bad, root_opts={'max_z_velocity': '0'})")
        .expect_err("py bad kinematics");
    assert_eq!(err_bad.class_name(), "Error");
    assert_eq!(err_bad.message(), py_err_bad.message);
    // The command is registered under its reference name and help text, and
    // every stepper is attached to the toolhead trapq.
    let py_help = py.eval("k_def.printer.lookup_object('gcode').help['SET_STEPPER_CARRIAGES']").expect("py help");
    let RecValue::Str(py_help) = py_help else { panic!("py help shape") };
    let rust_help = rs.gcode.descs.lock().get("SET_STEPPER_CARRIAGES").cloned();
    assert_eq!(rust_help.as_deref(), Some(py_help.as_str()));
    assert!(rs.gcode.handlers.lock().contains_key("SET_STEPPER_CARRIAGES"));
    for s in rs.kin.get_steppers() {
        let tq = s.get_trapq().expect("stepper trapq");
        assert!(Arc::ptr_eq(&tq, &rs.trapq), "stepper trapq must be the toolhead trapq");
    }
}

fn run_registered(gcode: &FakeGCodeImpl, gcmd: &GCmd) -> KResult<()> {
    let mut handlers = gcode.handlers.lock();
    let handler = handlers
        .get_mut("SET_STEPPER_CARRIAGES")
        .expect("SET_STEPPER_CARRIAGES registered");
    handler(gcmd)
}

#[test]
fn test_set_stepper_carriages() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    let rs = build_rust_kin(rust_standard_sections());
    py.exec(concat!(
        "k, p, t, g0 = build_py_kin(standard_sections())\n",
        "class G:\n",
        "    error = FakePrinter.command_error\n",
        "    def __init__(self, params):\n",
        "        self.params = params\n",
        "    def get(self, name, default=None):\n",
        "        return self.params.get(name, default)\n",
        "    def get_int(self, name, default=None):\n",
        "        if name not in self.params:\n",
        "            return default\n",
        "        return int(self.params[name])\n",
        "def py_cmd(params):\n",
        "    try:\n",
        "        g0.handlers['SET_STEPPER_CARRIAGES'](G(params))\n",
        "        return 'ok'\n",
        "    except FakePrinter.command_error as e:\n",
        "        return str(e)\n",
    ))
    .expect("py gcmd setup");
    let cases: Vec<Vec<(&str, &str)>> = vec![
        // Long stepper name, unchanged carriages.
        vec![("STEPPER", "stepper_x"), ("CARRIAGES", "x")],
        // Short stepper name is looked up through `get_name(short=True)`.
        vec![("STEPPER", "x"), ("CARRIAGES", "x")],
        // Unknown stepper name.
        vec![("STEPPER", "nope"), ("CARRIAGES", "x")],
        // Adding/removing the controlled carriage set is rejected.
        vec![("STEPPER", "stepper_x"), ("CARRIAGES", "y")],
        // Repeating an axis is a parse error with the reference text.
        vec![("STEPPER", "stepper_x"), ("CARRIAGES", "x+0.0*x")],
        // A carriage string with no coefficient is rejected.
        vec![("STEPPER", "stepper_z"), ("CARRIAGES", "0.0*z")],
        // DISABLE_CHECKS skips the kinematics validation.
        vec![("STEPPER", "stepper_x"), ("CARRIAGES", "x"), ("DISABLE_CHECKS", "1")],
    ];
    for params in &cases {
        let gcmd = make_gcmd(params);
        let rust_res = run_registered(&rs.gcode, &gcmd);
        let py_args: Vec<String> = params.iter().map(|(k, v)| format!("'{k}': '{v}'")).collect();
        let py_res = py
            .eval(&format!("py_cmd({{{}}})", py_args.join(", ")))
            .expect("py cmd result");
        let RecValue::Str(py_res) = py_res else { panic!("py cmd shape") };
        match rust_res {
            Ok(()) => assert_eq!(py_res, "ok", "case {params:?}"),
            Err(e) => {
                assert_eq!(e.class_name(), "CommandError", "case {params:?}");
                assert_eq!(py_res, e.message(), "case {params:?}");
            }
        }
    }
    // The command flushes the step generation before touching the steppers.
    let flushed = *rs.toolhead.flushed.lock();
    let py_flushed = py.eval("t.flushed").expect("py flushed");
    let RecValue::Int(py_flushed) = py_flushed else { panic!("py flushed shape") };
    assert_eq!(flushed as i64, py_flushed);
}

#[test]
fn test_home_axis() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    let rs = build_rust_kin(rust_standard_sections());
    let hs_arc_concrete = Arc::new(FakeHomingStateImpl { axes: vec![0], calls: Mutex::new(Vec::new()) });
    let hs_arc: HomingStateHandle = hs_arc_concrete.clone();
    let rail = rs.kin.get_primary_carriages()[0].get_rail();
    rs.kin.home_axis(hs_arc.clone(), 0, rail).expect("rust home_axis");
    rs.kin.home(hs_arc.clone()).expect("rust home");
    // Python reference: drive `home_axis` and `home` with a fake homing state
    // and compare the forcepos/homepos passed to `home_rails`. With
    // `position_endstop=0`, `position_max=100`, `positive_dir=False`,
    // `forcepos[0] = 0 + 1.5*(100-0) = 150.0`, `homepos[0] = 0.0`.
    py.exec("k = build_py_kin(standard_sections())[0]\nclass HS:\n    def __init__(self): self.calls = []\n    def get_axes(self): return [0]\n    def home_rails(self, rails, forcepos, homepos):\n        self.calls.append(([r.get_name(short=True) for r in rails], forcepos, homepos))\nhs = HS()\nk.home_axis(hs, 0, k.primary_carriages[0].get_rail())\nk.home(hs)")
        .expect("py home axis");
    let rust_calls = hs_arc_concrete.calls.lock().clone();
    assert_eq!(rust_calls.len(), 2, "home_axis and home must each call home_rails");
    for (i, call) in rust_calls.iter().enumerate() {
        let py_rails = py.eval(&format!("hs.calls[{i}][0]")).expect("py rails");
        let py_force = py.eval(&format!("hs.calls[{i}][1]")).expect("py forcepos");
        let py_home = py.eval(&format!("hs.calls[{i}][2]")).expect("py homepos");
        assert_py_value_eq(&opt_float_list(&call.forcepos), &py_force);
        assert_py_value_eq(&opt_float_list(&call.homepos), &py_home);
        let rails: Vec<PyValue> = call.rails.iter().map(|r| PyValue::Str(r.clone())).collect();
        assert_py_value_eq(&PyValue::List(rails), &py_rails);
    }
}

#[test]
fn test_parse_carriages_string() {
    let Some(mut py) = skip_if_no_oracle() else { return };
    py_setup(&mut py);
    // Build a carriage map with distinct axes and compare strings.
    let map = {
        let setup = build_rust_kin(rust_standard_sections());
        let mut all: IndexMap<String, kinematics::kinematic_stepper::Carriage> = IndexMap::new();
        for (k, v) in setup.kin.get_all_carriages() {
            all.insert(k.clone(), v.clone());
        }
        all
    };
    // A space before a sign is fine (each term is stripped); a space
    // *after* the sign is invalid in the reference too: Python strips the
    // term, strips the sign, and then reads `' y'` as a carriage name,
    // which is why `"2*x - y+z"` must error on both sides.
    let good = "2*x -y+z";
    let (coeffs, cars) = parse_carriages_string(
        good,
        &map,
        &|m: String| KlippyError::command(m),
    )
    .unwrap();
    assert_eq!(coeffs, vec![2.0, -1.0, 1.0]);
    let names: Vec<String> = cars
        .iter()
        .map(|c| c.as_any().downcast_ref::<kinematics::generic_cartesian::MainCarriage>().expect("main carriage").get_name())
        .collect();
    assert_eq!(names, vec!["x".to_string(), "y".to_string(), "z".to_string()]);
    let py_coeffs = py
        .eval("ks.parse_carriages_string('2*x -y+z', build_py_kin(standard_sections())[0].all_carriages, FakeConfigError)[0]")
        .expect("py parse");
    if let RecValue::List(items) = py_coeffs {
        assert_eq!(items.len(), 3);
        for (i, it) in items.iter().enumerate() {
            let RecValue::Float(fv) = it else { panic!("coeff shape: {it:?}") };
            let f = recording::f64_from_hex(&fv.hex).expect("oracle hex");
            assert_eq!(f.to_bits(), coeffs[i].to_bits());
        }
    } else {
        panic!("py parse shape {:?}", py_coeffs);
    }
    // The spaced-after-sign form errors identically on both sides.
    let spaced = "2*x - y+z";
    let rs_err = parse_carriages_string(spaced, &map, &|m: String| KlippyError::command(m))
        .err()
        .expect("spaced parse must fail");
    assert_eq!(rs_err.class_name(), "CommandError");
    assert_eq!(
        rs_err.message(),
        "Invalid ' y' carriage referenced in '2*x - y+z'"
    );
    let py_err = py
        .eval("ks.parse_carriages_string('2*x - y+z', build_py_kin(standard_sections())[0].all_carriages, FakeConfigError)")
        .expect_err("py spaced parse");
    assert_eq!(py_err.class_name, "FakeConfigError");
    assert_eq!(py_err.message, rs_err.message());
    // Error cases, Rust error class checked; messages match the reference
    // by construction in the port (see `kinematic_stepper.rs` header).
    for bad in ["x*w", "x+x", "a*x", "x*y*z"] {
        let e = parse_carriages_string(bad, &map, &|m: String| KlippyError::command(m)).err().expect("parse must fail");
        assert_eq!(e.class_name(), "CommandError");
        let py_bad = py
            .eval(&format!(
                "ks.parse_carriages_string('{bad}', build_py_kin(standard_sections())[0].all_carriages, FakeConfigError)"
            ))
            .expect_err("py bad parse");
        assert_eq!(py_bad.class_name, "FakeConfigError");
        assert_eq!(py_bad.message, e.message());
    }
}
