//! Parity tests for `kinematics::extruder` (port of `klippy/kinematics/extruder.py`).
//!
//! Expected values come live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + `klippy/`): minimal Python fakes for the
//! printer, config, gcode, toolhead, stepper, heaters and motion queuing
//! are driven with the same inputs on both sides, and the resulting
//! status dicts, G-code responses, raised errors (class and message),
//! recorded config accesses, stepper calls, trapq appends and log lines
//! are compared exactly (floats by bits via the recording value format).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone.

use indexmap::IndexMap;
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, SensorFactory, TemperatureCallback, TemperatureSensorApi,
};
use kinematics::extruder::{
    add_printer_objects, module_factory, DummyExtruder, ExtruderStepper,
    PrinterExtruder,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::KResult;
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GCGetFloatOpts, GetIntOpts as GCGetIntOpts,
    GetOpts as GCGetOpts, GCmd, GetParser,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper, CommandWrapper,
    ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams,
    PinResolver, PinsApi, PwmPinApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    McuStepperApi, PullHistoryStep, RailOpts, Stepper, StepperFactoryApi, StepperKinematics,
    Trapq, TrapqApi,
};
use klippy_api::toolhead::{
    Coord, ExtraAxis, ExtraAxisApi, ExtruderStepperApi, HomingStateHandle, Kinematics, KinematicsApi,
    MotionQueuingApi, MoveApi, PrinterExtruder as ThExtruder, ToolHeadApi, TrapqAppend,
};
use klippy_api::{EventArg, KlippyError, ObjectRef};
use pycompat::PyValue;
use kos::sync::Mutex;
use pycompat::PyInt;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use reactor::{Completion, Reactor, ReactorMutex};
use std::any::Any;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python fakes (frozen reference side).
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import math, sys, types, logging as _logging
import configparser
from gcode import CommandError as _CmdErr

config_reads = []
gcode_regs = []
responds = []
rollover = []
events = []
log_lines = []
pa_calls = []
append_calls = []
set_temp_calls = []
heater_setup = []
stepper_calls = []
flush_calls = []
lookahead = []
scan_calls = []

class _Missing: pass
_MISSING = _Missing()

class FakePrinter:
    command_error = _CmdErr
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.handlers = {}
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=_MISSING):
        if name in self.objects:
            return self.objects[name]
        if default is _MISSING:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        out = []
        if module in self.objects:
            out.append((module, self.objects[module]))
        for n, o in self.objects.items():
            if n.startswith(module + ' '):
                out.append((n, o))
        return out
    def load_object(self, config, section, default=_MISSING):
        if section in self.objects:
            return self.objects[section]
        if default is _MISSING:
            raise self.config_error("Unable to load module '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        events.append((event, params))
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def set_rollover_info(self, name, info, log=True):
        rollover.append((name, info, log))

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, name, options, sections=None):
        self._printer = printer
        self._name = name
        self.options = dict(options)
        self._sections = dict(sections or {})
    def get_printer(self): return self._printer
    def get_name(self): return self._name
    def _bounds(self, option, v, minval, maxval, above, below):
        if minval is not None and v < minval:
            raise self.error("Option '%s' in section '%s' must have minimum of %s" % (option, self._name, minval))
        if maxval is not None and v > maxval:
            raise self.error("Option '%s' in section '%s' must have maximum of %s" % (option, self._name, maxval))
        if above is not None and v <= above:
            raise self.error("Option '%s' in section '%s' must be above %s" % (option, self._name, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s" % (option, self._name, below))
        return v
    def get(self, option, default=_MISSING):
        config_reads.append((self._name, 'get', option))
        if option in self.options:
            return self.options[option]
        if default is _MISSING:
            raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
        return default
    def getfloat(self, option, default=_MISSING, minval=None, maxval=None, above=None, below=None):
        config_reads.append((self._name, 'getfloat', option))
        if option not in self.options:
            if default is _MISSING:
                raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        try: v = float(self.options[option])
        except Exception: raise self.error("Unable to parse option '%s' in section '%s'" % (option, self._name))
        return self._bounds(option, v, minval, maxval, above, below)
    def getint(self, option, default=_MISSING, minval=None, maxval=None):
        config_reads.append((self._name, 'getint', option))
        if option not in self.options:
            if default is _MISSING:
                raise self.error("Option '%s' in section '%s' must be specified" % (option, self._name))
            return default
        v = int(self.options[option], 10)
        return self._bounds(option, v, minval, maxval, None, None)
    def getsection(self, section):
        return FakeConfig(self._printer, section, dict(self._sections.get(section, {})))
    def has_section(self, section):
        return section in self._sections

class FakeGCmd:
    error = _CmdErr
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
    def get_command(self): return self._command
    def get_commandline(self): return self._commandline
    def get_command_parameters(self): return self._params
    def respond_info(self, msg, log=True):
        responds.append((msg, log))
    def respond_raw(self, msg):
        responds.append((msg, True))
    def get(self, name, default=_MISSING, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _MISSING:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try: value = parser(value)
        except Exception:
            raise self.error("Error on '%s': unable to parse %s" % (self._commandline, value))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s" % (self._commandline, name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s" % (self._commandline, name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s" % (self._commandline, name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s" % (self._commandline, name, below))
        return value
    def get_int(self, name, default=_MISSING, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)
    def get_float(self, name, default=_MISSING, minval=None, maxval=None, above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval, above=above, below=below)

class FakeGCode:
    def register_command(self, cmd, func, desc=None, when_not_ready=False):
        gcode_regs.append(('cmd', cmd, desc))
    def register_mux_command(self, cmd, key, value, func, desc=None):
        gcode_regs.append(('mux', cmd, key, value, desc))

class FakeToolhead:
    def __init__(self):
        self.flush = 0
        self.active = None
        self.active_pos = None
        self.dummy = None
        self.vel = (250.0, 3000.0)
    def get_max_velocity(self): return self.vel
    def flush_step_generation(self): flush_calls.append(1)
    def register_lookahead_callback(self, cb): lookahead.append(cb)
    def get_extruder(self):
        if self.active is not None:
            return self.active
        return self.dummy
    def set_extruder(self, extruder, pos):
        self.active = extruder
        self.active_pos = pos

class FakeHeater:
    def __init__(self, can_extrude=True, status=None, stats=None):
        self.can_extrude = can_extrude
        self._status = dict(status) if status is not None else {'temperature': 0.0, 'target': 0.0, 'power': 0.0}
        self._stats = stats if stats is not None else (False, 'extruder: target=0 temp=0.0 pwm=0.000')
    def get_status(self, eventtime): return dict(self._status)
    def stats(self, eventtime): return self._stats

class FakeHeaters:
    def __init__(self, heater): self.heater = heater
    def setup_heater(self, config, gcode_id=None):
        heater_setup.append((config.get_name(), gcode_id))
        return self.heater
    def set_temperature(self, heater, temp, wait=False):
        set_temp_calls.append((temp, wait))

_trapq_seq = [0]
class FakeMQ:
    def allocate_trapq(self):
        _trapq_seq[0] += 1
        return ('trapq', _trapq_seq[0])
    def lookup_trapq_append(self):
        def _append(trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, start_v, cruise_v, accel):
            append_calls.append((trapq, print_time, accel_t, cruise_t, decel_t, x, y, z, rx, ry, rz, start_v, cruise_v, accel))
        return _append
    def check_step_generation_scan_windows(self): scan_calls.append(1)

class FakeStepper:
    def __init__(self):
        self.trapq = None
        self.rotation_distance = (5.0, 200.0)
        self.dir_inverted = (False, False)
        self.position = None
        self.sk = None
    def set_stepper_kinematics(self, sk):
        stepper_calls.append(('sk',))
        self.sk = sk
    def get_trapq(self): return self.trapq
    def set_trapq(self, tq):
        stepper_calls.append(('set_trapq', tq))
        self.trapq = tq
    def set_position(self, pos):
        stepper_calls.append(('set_position', tuple(pos)))
        self.position = pos
    def set_rotation_distance(self, rd):
        stepper_calls.append(('set_rotation_distance', rd))
        self.rotation_distance = (rd, self.rotation_distance[1])
    def get_rotation_distance(self):
        stepper_calls.append(('get_rotation_distance',))
        return self.rotation_distance
    def get_dir_inverted(self): return self.dir_inverted
    def set_dir_inverted(self, v):
        stepper_calls.append(('set_dir_inverted', v))
        self.dir_inverted = (v, self.dir_inverted[1])
    def get_past_mcu_position(self, print_time):
        stepper_calls.append(('get_past_mcu_position', print_time))
        return 1234
    def mcu_to_commanded_position(self, mcu_pos):
        stepper_calls.append(('mcu_to_commanded_position', mcu_pos))
        return mcu_pos * 0.5

class FakeMove:
    def __init__(self, **kw):
        self.start_pos = kw.get('start_pos', [0., 0., 0., 0.])
        self.end_pos = kw.get('end_pos', [0., 0., 0., 0.])
        self.axes_d = kw.get('axes_d', [0., 0., 0., 0.])
        self.axes_r = kw.get('axes_r', [0., 0., 0., 1.])
        self.move_d = kw.get('move_d', 0.)
        self.accel = kw.get('accel', 0.)
        self.max_cruise_v2 = kw.get('max_cruise_v2', 0.)
        self.start_v = kw.get('start_v', 0.)
        self.cruise_v = kw.get('cruise_v', 0.)
        self.accel_t = kw.get('accel_t', 0.)
        self.cruise_t = kw.get('cruise_t', 0.)
        self.decel_t = kw.get('decel_t', 0.)
        self.limited = None
        self.is_kinematic_move = False
    def limit_speed(self, speed, accel):
        self.limited = (speed, accel)
    def move_error(self, msg='Move out of range'):
        return _CmdErr(msg)

def make_env(name='extruder', opts=None, vel=(250.0, 3000.0), can_extrude=True, status=None, stats=None):
    _trapq_seq[0] = 0
    printer = FakePrinter()
    gcode = FakeGCode()
    printer.objects['gcode'] = gcode
    toolhead = FakeToolhead()
    toolhead.vel = vel
    toolhead.dummy = extruder.DummyExtruder(printer)
    printer.objects['toolhead'] = toolhead
    heater = FakeHeater(can_extrude, status, stats)
    printer.objects['heaters'] = FakeHeaters(heater)
    mq = FakeMQ()
    printer.objects['motion_queuing'] = mq
    config = FakeConfig(printer, name, dict(opts or {}))
    return printer, gcode, toolhead, heater, mq, config

def clear_registries():
    del config_reads[:]; del responds[:]; del rollover[:]; del events[:]
    del pa_calls[:]; del append_calls[:]; del set_temp_calls[:]
    del heater_setup[:]; del stepper_calls[:]; del flush_calls[:]
    del lookahead[:]; del scan_calls[:]

class _LogCapture(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_LogCapture())
_root.setLevel(0)

_stepper_mod = types.ModuleType('stepper')
def _make_stepper(config):
    stepper_calls.append(('make', config.get_name()))
    return FakeStepper()
_stepper_mod.PrinterStepper = _make_stepper
sys.modules['stepper'] = _stepper_mod

class _FfiLib:
    def extruder_stepper_alloc(self): return 'SK'
    def extruder_stepper_free(self, sk): pass
    def extruder_set_pressure_advance(self, sk, print_time, pa, smooth_time):
        pa_calls.append((print_time, pa, smooth_time))
class _FfiMain:
    def gc(self, obj, free): return obj
_chelper_mod = types.ModuleType('chelper')
_chelper_mod.get_ffi = lambda: (_FfiMain(), _FfiLib())
sys.modules['chelper'] = _chelper_mod

import kinematics.extruder as extruder
"#;

/// Compare one oracle result with the Rust result.
fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extruder".to_string(),
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

/// Compare an oracle exception (class and message) with a Rust error.
fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(oracle_exc.class_name, actual.class_name(), "{label} class");
    assert_eq!(oracle_exc.message, actual.message(), "{label} message");
}


fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("extruder fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping extruder oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}
// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Shared<T> = Arc<Mutex<T>>;

/// `toolhead.register_lookahead_callback(cb)` payload.
type LookaheadCb = Box<dyn FnMut(f64) -> KResult<()> + Send>;

struct FakeTrapq {
    id: i64,
}

impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn make_trapq(id: i64) -> Trapq {
    Arc::new(FakeTrapq { id })
}

fn trapq_id(tq: &Trapq) -> i64 {
    tq.as_any().downcast_ref::<FakeTrapq>().map(|f| f.id).unwrap_or(-1)
}

// --- log capture (one process-wide sink, lines kept per test thread so
// parallel tests cannot clear or read each other's lines) ---

thread_local! {
    static LOG_LINES: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_LINES.with(|l| l.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

fn logs_snapshot() -> Vec<String> {
    install_log_sink();
    LOG_LINES.with(|l| l.borrow().clone())
}

fn logs_clear() {
    install_log_sink();
    LOG_LINES.with(|l| l.borrow_mut().clear());
}
// --- FakeConfig ---

#[derive(Clone)]
struct FakeConfig {
    name: String,
    printer: Option<Printer>,
    options: IndexMap<String, String>,
    sections: IndexMap<String, IndexMap<String, String>>,
    reads: Shared<Vec<(String, String, String)>>,
}

impl FakeConfig {
    fn new(name: &str, options: Vec<(&str, &str)>, printer: Printer) -> Self {
        let mut map = IndexMap::new();
        for (k, v) in options {
            map.insert(k.to_string(), v.to_string());
        }
        FakeConfig { name: name.to_string(), printer: Some(printer), options: map, sections: IndexMap::new(), reads: Arc::new(Mutex::new(Vec::new())) }
    }
}

fn config_error_required(option: &str, section: &str) -> KlippyError {
    KlippyError::config(format!("Option '{option}' in section '{section}' must be specified"))
}

fn config_error_minmax(option: &str, section: &str, kind: &str, bound: &str) -> KlippyError {
    KlippyError::config(format!("Option '{option}' in section '{section}' must have {kind} of {bound}"))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone().expect("fake printer")
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.reads.lock().push((self.name.clone(), "get".to_string(), option.to_string()));
        match (self.options.get(option), opts.default) {
            (Some(v), _) => Ok(Some(v.clone())),
            (None, None) => Err(config_error_required(option, &self.name)),
            (None, Some(d)) => Ok(d),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.reads.lock().push((self.name.clone(), "getint".to_string(), option.to_string()));
        match (self.options.get(option), opts.default) {
            (Some(v), _) => {
                let val: i64 = v.trim().parse().map_err(|_| KlippyError::config(format!("Unable to parse option '{option}' in section '{}'", self.name)))?;
                if let Some(mn) = opts.minval { if val < mn { return Err(config_error_minmax(option, &self.name, "minimum", &mn.to_string())); } }
                if let Some(mx) = opts.maxval { if val > mx { return Err(config_error_minmax(option, &self.name, "maximum", &mx.to_string())); } }
                Ok(Some(val))
            }
            (None, None) => Err(config_error_required(option, &self.name)),
            (None, Some(d)) => Ok(d),
        }
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.reads.lock().push((self.name.clone(), "getfloat".to_string(), option.to_string()));
        match (self.options.get(option), opts.default) {
            (Some(v), _) => {
                let val: f64 = v.trim().parse().map_err(|_| KlippyError::config(format!("Unable to parse option '{option}' in section '{}'", self.name)))?;
                if let Some(mn) = opts.minval { if val < mn { return Err(config_error_minmax(option, &self.name, "minimum", &py_float_str(mn))); } }
                if let Some(mx) = opts.maxval { if val > mx { return Err(config_error_minmax(option, &self.name, "maximum", &py_float_str(mx))); } }
                if let Some(ab) = opts.above { if val <= ab { return Err(KlippyError::config(format!("Option '{option}' in section '{}' must be above {}", self.name, py_float_str(ab)))); } }
                if let Some(be) = opts.below { if val >= be { return Err(KlippyError::config(format!("Option '{option}' in section '{}' must be below {}", self.name, py_float_str(be)))); } }
                Ok(Some(val))
            }
            (None, None) => Err(config_error_required(option, &self.name)),
            (None, Some(d)) => Ok(d),
        }
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, option: &str, _choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        self.reads.lock().push((self.name.clone(), "getchoice".to_string(), option.to_string()));
        match self.options.get(option) {
            Some(v) => Ok(PyValue::Str(v.clone())),
            None => match opts.default {
                Some(d) => Ok(d.unwrap_or(PyValue::None)),
                None => Err(config_error_required(option, &self.name)),
            },
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, section: &str) -> Config {
        let mut opts = IndexMap::new();
        if let Some(map) = self.sections.get(section) {
            for (k, v) in map { opts.insert(k.clone(), v.clone()); }
        }
        let mut c = FakeConfig::new(section, Vec::new(), self.get_printer());
        c.options = opts;
        c.reads = self.reads.clone();
        Arc::new(c)
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> { Vec::new() }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

fn py_float_str(v: f64) -> String {
    // CPython `str(float)` for bounds in config messages.
    pycompat::text::float_repr(v)
}

// --- FakePrinter ---

#[derive(Clone)]
struct FakePrinter {
    objects: Shared<IndexMap<String, ObjectRef>>,
    handlers: Shared<Vec<(String, EventCallback)>>,
    rollover: Shared<Vec<(String, Option<String>, bool)>>,
    events: Shared<Vec<String>>,
}

impl FakePrinter {
    fn new() -> Self {
        FakePrinter {
            objects: Arc::new(Mutex::new(IndexMap::new())),
            handlers: Arc::new(Mutex::new(Vec::new())),
            rollover: Arc::new(Mutex::new(Vec::new())),
            events: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn insert(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.objects.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objs = self.objects.lock();
        if let Some(o) = objs.get(name) { return Ok(Some(o.clone())); }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.lock();
        match module {
            None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            Some(m) => objs.iter().filter(|(k, _)| *k == m || k.starts_with(&format!("{m} "))).map(|(k, v)| (k.clone(), v.clone())).collect(),
        }
    }
    fn load_object(&self, _config: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool) {
        self.rollover.lock().push((name.to_string(), info.map(|s| s.to_string()), log));
    }
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        self.events.lock().push(event.to_string());
        let mut out = Vec::new();
        let mut handlers = self.handlers.lock();
        for (name, cb) in handlers.iter_mut() {
            if name == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

// --- FakeGCode / FakeGCmd ---

#[derive(Clone)]
struct FakeGCode {
    regs: Shared<Vec<String>>,
}
impl FakeGCode {
    fn new() -> Self {
        FakeGCode { regs: Arc::new(Mutex::new(Vec::new())) }
    }
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

fn reg_encode(kind: &str, parts: &[&str]) -> String {
    let mut s = kind.to_string();
    for p in parts {
        s.push('|');
        s.push_str(p);
    }
    s
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        _func: Option<Box<dyn FnMut(&GCmd) -> KResult<()> + Send>>,
        opts: klippy_api::gcode::RegisterCommandOpts,
    ) -> KResult<Option<Box<dyn FnMut(&GCmd) -> KResult<()> + Send>>> {
        let desc = opts.desc.unwrap_or_else(|| "None".to_string());
        self.regs.lock().push(reg_encode("cmd", &[cmd, &desc]));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: Box<dyn FnMut(&GCmd) -> KResult<()> + Send>,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.regs.lock().push(reg_encode("mux", &[cmd, key, value.unwrap_or("None"), desc.unwrap_or("None")]));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _eventtime: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn register_output_handler(&self, _callback: Box<dyn FnMut(String) + Send>) {}
    fn request_restart(&self, _result: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> ReactorMutex { panic!("get_mutex not used by extruder") }
    fn create_gcode_command(&self, command: &str, commandline: &str, params: &IndexMap<String, String>) -> GCmd {
        Arc::new(FakeGCmd::new(command, commandline, params.clone()))
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool { true }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> { Ok(()) }
}

#[derive(Clone)]
struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    responds: Shared<Vec<(String, bool)>>,
}

impl FakeGCmd {
    fn new(command: &str, commandline: &str, params: IndexMap<String, String>) -> Self {
        FakeGCmd { command: command.to_string(), commandline: commandline.to_string(), params, responds: Arc::new(Mutex::new(Vec::new())) }
    }
    fn from_params(command: &str, commandline: &str, params: Vec<(&str, &str)>) -> Self {
        let mut map = IndexMap::new();
        for (k, v) in params { map.insert(k.to_string(), v.to_string()); }
        FakeGCmd::new(command, commandline, map)
    }
}

fn pct_float_str(v: f64) -> String {
    pycompat::text::float_repr(v)
}


impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GCGetOpts) -> KResult<PyValue> {
        let value = self.params.get(name);
        let raw = match value {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => Ok(d),
                };
            }
        };
        let parsed: PyValue = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int => match raw.trim().parse::<i64>() {
                Ok(v) => PyValue::Int(PyInt::from_i64(v)),
                Err(_) => return Err(KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, raw))),
            },
            GetParser::IntAutoBase => match raw.trim().parse::<i64>() {
                Ok(v) => PyValue::Int(PyInt::from_i64(v)),
                Err(_) => return Err(KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, raw))),
            },
            GetParser::Float => match raw.trim().parse::<f64>() {
                Ok(v) => PyValue::Float(v),
                Err(_) => return Err(KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, raw))),
            },
        };
        // min/max/above/below in Python are numeric and only used by get_int/get_float paths.
        let _ = (opts.minval, opts.maxval, opts.above, opts.below);
        Ok(parsed)
    }
    fn get_int(&self, name: &str, opts: GCGetIntOpts) -> KResult<Option<i64>> {
        let value = self.params.get(name);
        let raw = match value {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => Ok(d),
                };
            }
        };
        let val: i64 = raw.trim().parse().map_err(|_| KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, raw)))?;
        if let Some(mn) = opts.minval { if val < mn { return Err(KlippyError::command(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, mn))); } }
        if let Some(mx) = opts.maxval { if val > mx { return Err(KlippyError::command(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, mx))); } }
        Ok(Some(val))
    }
    fn get_float(&self, name: &str, opts: GCGetFloatOpts) -> KResult<Option<f64>> {
        let value = self.params.get(name);
        let raw = match value {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    None => Err(KlippyError::command(format!("Error on '{}': missing {}", self.commandline, name))),
                    Some(d) => Ok(d),
                };
            }
        };
        let val: f64 = raw.trim().parse().map_err(|_| KlippyError::command(format!("Error on '{}': unable to parse {}", self.commandline, raw)))?;
        if let Some(mn) = opts.minval { if val < mn { return Err(KlippyError::command(format!("Error on '{}': {} must have minimum of {}", self.commandline, name, pct_float_str(mn)))); } }
        if let Some(mx) = opts.maxval { if val > mx { return Err(KlippyError::command(format!("Error on '{}': {} must have maximum of {}", self.commandline, name, pct_float_str(mx)))); } }
        if let Some(ab) = opts.above { if val <= ab { return Err(KlippyError::command(format!("Error on '{}': {} must be above {}", self.commandline, name, pct_float_str(ab)))); } }
        if let Some(be) = opts.below { if val >= be { return Err(KlippyError::command(format!("Error on '{}': {} must be below {}", self.commandline, name, pct_float_str(be)))); } }
        Ok(Some(val))
    }
    fn get_command(&self) -> String { self.command.clone() }
    fn get_commandline(&self) -> String { self.commandline.clone() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, msg: &str, log: bool) { self.responds.lock().push((msg.to_string(), log)); }
    fn respond_raw(&self, msg: &str) { self.responds.lock().push((msg.to_string(), true)); }
    fn ack(&self, _msg: Option<&str>) -> bool { false }
}

// --- FakeMcu (unwired: extruder never needs it) ---

#[derive(Clone, Default)]
struct FakeMcu {
    printer: Option<Printer>,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String { "mcu".to_string() }
    fn get_printer(&self) -> Printer { self.printer.clone().expect("fake printer") }
    fn is_fileoutput(&self) -> bool { true }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> { panic!("setup_pin not used by extruder") }
    fn create_oid(&self) -> KResult<i64> { Ok(1) }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> { Ok(()) }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> { Ok(()) }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, time: f64) -> i64 { py_trunc_i64(time * 1000.0) }
    fn min_schedule_time(&self) -> f64 { 0.0 }
    fn max_nominal_duration(&self) -> f64 { 0.0 }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> { panic!("lookup_command not used") }
    fn lookup_query_command(&self, _msgformat: &str, _respformat: &str, _opts: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> { panic!("lookup_query_command not used") }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(1) }
    fn register_serial_response(&self, _cb: SerialResponseCallback, _msg: &str, _oid: Option<i64>) -> AsyncResponseWrapper { panic!("register_serial_response not used") }
    fn check_valid_response(&self, _msgformat: &str) -> bool { false }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> { Ok(1000.0) }
    fn print_time_to_clock(&self, print_time: f64) -> i64 { py_trunc_i64(print_time * 1000.0) }
    fn clock_to_print_time(&self, clock: f64) -> f64 { clock / 1000.0 }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 { 0.0 }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { clock32 }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> { Ok((1000.0, 0.0)) }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> { panic!("create_trigger_dispatch not used") }
}

fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() { return 0; }
    pycompat::text::trunc_float_to_bigint(f).0.to_string().parse::<i64>().unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

// --- FakeStepper ---

#[derive(Clone)]
struct FakeStepper {
    calls: Shared<Vec<String>>,
    trapq: Shared<Option<Trapq>>,
    rotation: Shared<(f64, f64)>,
    dir: Shared<(bool, bool)>,
    position: Shared<Option<[f64; 3]>>,
    sk: Shared<Option<StepperKinematics>>,
}

impl FakeStepper {
    fn new() -> Self {
        FakeStepper {
            calls: Arc::new(Mutex::new(Vec::new())),
            trapq: Arc::new(Mutex::new(None)),
            rotation: Arc::new(Mutex::new((5.0, 200.0))),
            dir: Arc::new(Mutex::new((false, false))),
            position: Arc::new(Mutex::new(None)),
            sk: Arc::new(Mutex::new(None)),
        }
    }
}

fn trapq_repr(tq: &Option<Trapq>) -> String {
    match tq {
        None => "None".to_string(),
        Some(t) => format!("trapq:{}", trapq_id(t)),
    }
}

fn pos_repr(p: &[f64; 3]) -> String {
    format!("[{}, {}, {}]", pt_float(p[0]), pt_float(p[1]), pt_float(p[2]))
}

fn pt_float(v: f64) -> String {
    pycompat::text::float_repr(v)
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu { Arc::new(FakeMcu::default()) }
    fn get_name(&self, _short: Option<bool>) -> String { "stepper".to_string() }
    fn get_oid(&self) -> i64 { 1 }
    fn units_in_radians(&self) -> bool { false }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) { (None, false) }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {}
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 { 0.025 }
    fn get_rotation_distance(&self) -> (f64, f64) {
        self.calls.lock().push("get_rotation_distance".to_string());
        *self.rotation.lock()
    }
    fn set_rotation_distance(&self, rotation_dist: f64) {
        self.calls.lock().push(format!("set_rotation_distance:{}", pt_float(rotation_dist)));
        let mut r = self.rotation.lock();
        r.0 = rotation_dist;
    }
    fn get_dir_inverted(&self) -> (bool, bool) { *self.dir.lock() }
    fn set_dir_inverted(&self, invert_dir: bool) -> KResult<()> {
        self.calls.lock().push(format!("set_dir_inverted:{}", invert_dir));
        let mut d = self.dir.lock();
        d.0 = invert_dir;
        Ok(())
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 { 0.0 }
    fn set_position(&self, coord: [f64; 3]) {
        self.calls.lock().push(format!("set_position:{}", pos_repr(&coord)));
        *self.position.lock() = Some(coord);
    }
    fn get_commanded_position(&self) -> f64 { 0.0 }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 { 0 }
    fn get_past_mcu_position(&self, print_time: f64) -> i64 {
        self.calls.lock().push(format!("get_past_mcu_position:{}", pt_float(print_time)));
        1234
    }
    fn mcu_to_commanded_position(&self, mcu_pos: i64) -> f64 {
        self.calls.lock().push(format!("mcu_to_commanded_position:{}", mcu_pos));
        mcu_pos as f64 * 0.5
    }
    fn dump_steps(&self, _count: i64, _start_clock: i64, _end_clock: i64) -> (Vec<PullHistoryStep>, i64) { (Vec::new(), 0) }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> { self.sk.lock().clone() }
    fn set_stepper_kinematics(&self, sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        self.calls.lock().push("sk_set".to_string());
        let mut guard = self.sk.lock();
        std::mem::replace(&mut *guard, sk)
    }
    fn note_homing_end(&self) -> KResult<()> { Ok(()) }
    fn get_trapq(&self) -> Option<Trapq> { self.trapq.lock().clone() }
    fn set_trapq(&self, tq: Option<Trapq>) -> Option<Trapq> {
        self.calls.lock().push(format!("set_trapq:{}", trapq_repr(&tq)));
        let mut guard = self.trapq.lock();
        std::mem::replace(&mut *guard, tq)
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> { Ok(()) }
    fn is_active_axis(&self, _axis: char) -> bool { true }
}

#[derive(Clone)]
struct FakeStepperFactory {
    steppers: Shared<Vec<Arc<FakeStepper>>>,
}

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        let stepper = Arc::new(FakeStepper::new());
        self.steppers.lock().push(stepper.clone());
        Ok(stepper)
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> { panic!("lookup_rail not used") }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> { panic!("lookup_multi_rail not used") }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<klippy_api::stepper::Rail> { panic!("generic_rail not used") }
    fn parse_step_distance(&self, _config: &Config, _units_in_radians: Option<bool>, _note_valid: bool) -> KResult<(f64, f64)> { Ok((5.0, 200.0)) }
}
// --- FakeMotionQueuing ---

#[derive(Clone)]
struct FakeMQ {
    seq: Shared<i64>,
    appends: Shared<Vec<(i64, Vec<f64>)>>,
    scans: Shared<usize>,
}

impl FakeMQ {
    fn new() -> Self {
        FakeMQ { seq: Arc::new(Mutex::new(0)), appends: Arc::new(Mutex::new(Vec::new())), scans: Arc::new(Mutex::new(0)) }
    }
}
klippy_api::printer_object!(FakeMQ; dyn MotionQueuingApi);

impl MotionQueuingApi for FakeMQ {
    fn allocate_trapq(&self) -> Trapq {
        let mut s = self.seq.lock();
        *s += 1;
        make_trapq(*s)
    }
    fn lookup_trapq_append(&self) -> TrapqAppend {
        let appends = self.appends.clone();
        Arc::new(move |tq: &Trapq, p1: f64, p2: f64, p3: f64, p4: f64, p5: f64, p6: f64, p7: f64, p8: f64, p9: f64, p10: f64, p11: f64, p12: f64, p13: f64| {
            appends.lock().push((trapq_id(tq), vec![p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13]));
        })
    }
    fn wipe_trapq(&self, _trapq: &Trapq) {}
    fn allocate_syncemitter(&self, _mcu: Mcu, _name: &str, _alloc_stepcompress: bool) -> KResult<klippy_api::toolhead::SyncEmitterHandle> { panic!("allocate_syncemitter not used") }
    fn queue_syncemitter_msg(&self, _syncemitter: &klippy_api::toolhead::SyncEmitterHandle, _req_clock: u64, _data: &[u32]) { panic!("queue_syncemitter_msg not used") }
    fn setup_mcu_movequeue(&self, _mcu: Mcu, _serialqueue: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _move_count: i64) {}
    fn register_flush_callback(&self, _callback: klippy_api::toolhead::FlushCallback, _can_add_trapq: bool) {}
    fn unregister_flush_callback(&self, _callback: &klippy_api::toolhead::FlushCallback) {}
    fn register_flush_callback_named(&self, cb: klippy_api::toolhead::FlushCallback, can_add_trapq: bool, _name: Option<&str>, _obj: Option<&str>) {
        self.register_flush_callback(cb, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 { 0.0 }
    fn check_step_generation_scan_windows(&self) { *self.scans.lock() += 1; }
    fn flush_all_steps(&self) -> KResult<()> { Ok(()) }
    fn calc_step_gen_restart(&self, _est_print_time: f64) -> f64 { 0.0 }
    fn note_mcu_movequeue_activity(&self, _mq_time: f64, _is_step_gen: bool) {}
    fn drip_update_time(&self, _start_time: f64, _end_time: f64, _drip_completion: &Completion<bool>) -> KResult<()> { Ok(()) }
    fn check_drip_timing(&self) -> Option<f64> { None }
}

// --- FakeSensor / FakePwmPin / FakePins / FakeHeaters ---
//
// `setup_heater` returns a real `extras::heaters::Heater` (the trait
// signature is concrete there), built on a throwaway printer with a
// fake sensor/pin so the extruder config reads stay unpolluted and the
// heater status is the real initial `{temperature: 0, target: 0, power: 0}`.

struct FakeSensor;

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, _min_temp: f64, _max_temp: f64) -> KResult<()> {
        Ok(())
    }
    fn setup_callback(&self, _callback: TemperatureCallback) {}
    fn get_report_time_delta(&self) -> f64 { 0.1 }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

/// A `heaters.SensorApi` object (what `PrinterHeaters::register_sensor`
/// receives); the extruder never registers one, so it is only shaped.
#[allow(dead_code)]
struct FakePsensor;

impl SensorApi for FakePsensor {
    fn get_temp(&self, _eventtime: f64) -> (f64, f64) { (0.0, 0.0) }
}

klippy_api::printer_object!(FakePsensor; dyn SensorApi);

struct FakePwmPin;

impl PwmPinApi for FakePwmPin {
    fn get_mcu(&self) -> Mcu { Arc::new(FakeMcu::default()) }
    fn setup_max_duration(&self, _max_duration: f64) {}
    fn setup_cycle_time(&self, _cycle_time: f64, _hardware_pwm: Option<bool>) {}
    fn setup_start_value(&self, _start_value: f64, _shutdown_value: f64) {}
    fn next_aligned_print_time(&self, print_time: f64, _allow_early: Option<f64>) -> f64 { print_time }
    fn set_pwm(&self, _print_time: f64, _value: f64) {}
}

struct FakePins;

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("parse_pin not used by extruder"))
    }
    fn lookup_pin(&self, _pin_desc: &str, _opts: LookupPinOpts) -> KResult<PinParams> {
        Err(KlippyError::config("lookup_pin not used by extruder"))
    }
    fn setup_pin(&self, pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        if pin_type == "pwm" {
            Ok(Pin::Pwm(Arc::new(FakePwmPin)))
        } else {
            Err(KlippyError::config(format!("pin type {pin_type} not used by extruder")))
        }
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("get_pin_resolver not used by extruder"))
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> { Ok(()) }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

#[derive(Clone)]
struct FakeHeaters {
    heater: Shared<Option<Arc<Heater>>>,
    setup: Shared<Vec<(String, Option<String>)>>,
    set_temps: Shared<Vec<(f64, bool)>>,
    can_extrude: bool,
}

impl FakeHeaters {
    fn new(can_extrude: bool) -> Self {
        FakeHeaters {
            heater: Arc::new(Mutex::new(None)),
            setup: Arc::new(Mutex::new(Vec::new())),
            set_temps: Arc::new(Mutex::new(Vec::new())),
            can_extrude,
        }
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: SensorFactory) {}
    fn setup_heater(&self, config: &Config, gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        self.setup.lock().push((config.get_name(), gcode_id));
        let name = config.get_name();
        let short = name.split_whitespace().last().unwrap_or("").to_string();
        let printer = FakePrinter::new();
        let printer_handle: Printer = Arc::new(printer.clone());
        printer.insert("pins", Arc::new(FakePins));
        printer.insert("gcode", Arc::new(FakeGCode::new()));
        let dummy: ObjectRef = Arc::new(FakeGCode::new());
        printer.insert(&format!("verify_heater {short}"), dummy.clone());
        printer.insert("pid_calibrate", dummy);
        let min_extrude_temp = if self.can_extrude { "0" } else { "170" };
        let heater_config: Config = Arc::new(FakeConfig::new(
            &name,
            vec![
                ("min_temp", "-100"),
                ("max_temp", "300"),
                ("min_extrude_temp", min_extrude_temp),
                ("control", "watermark"),
                ("heater_pin", "PC1"),
                ("max_power", "1"),
                ("smooth_time", "1"),
                ("max_delta", "2"),
            ],
            printer_handle,
        ));
        let sensor: Arc<dyn TemperatureSensorApi> = Arc::new(FakeSensor);
        let heater = Heater::new(&heater_config, sensor)?;
        *self.heater.lock() = Some(heater.clone());
        Ok(heater)
    }
    fn get_all_heaters(&self) -> Vec<String> { Vec::new() }
    fn lookup_heater(&self, heater_name: &str) -> KResult<Arc<Heater>> {
        match self.heater.lock().clone() {
            Some(h) => Ok(h),
            None => Err(KlippyError::config(format!("Unknown heater '{heater_name}'"))),
        }
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Ok(Arc::new(FakeSensor))
    }
    fn register_sensor(&self, _config: &Config, _psensor: Arc<dyn SensorApi>, _gcode_id: Option<String>) -> KResult<()> {
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<Heater>, temp: f64, wait: bool) -> KResult<()> {
        self.set_temps.lock().push((temp, wait));
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

// --- DummyKinematics (returning empty) ---

struct DummyKinematics;

impl KinematicsApi for DummyKinematics {
    fn get_steppers(&self) -> Vec<Stepper> { Vec::new() }
    fn calc_position(&self, _stepper_positions: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> { Ok(vec![None; 4]) }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> { Ok(()) }
    fn check_move(&self, _mv: &dyn MoveApi) -> KResult<()> { Ok(()) }
    fn home(&self, _homing_state: HomingStateHandle) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn clear_homing_state(&self, _clear_axes: &str) {}
}

klippy_api::printer_object!(DummyKinematics);

// --- FakeToolhead ---

#[derive(Clone)]
struct FakeToolhead {
    flushes: Shared<usize>,
    lookaheads: Shared<Vec<LookaheadCb>>,
    active: Shared<Option<ThExtruder>>,
    active_pos: Shared<Option<f64>>,
    dummy: Shared<Option<ThExtruder>>,
    vel: Shared<(f64, f64)>,
    steppers: Shared<Vec<Arc<FakeStepper>>>,
}

impl FakeToolhead {
    fn new() -> Self {
        FakeToolhead {
            flushes: Arc::new(Mutex::new(0)),
            lookaheads: Arc::new(Mutex::new(Vec::new())),
            active: Arc::new(Mutex::new(None)),
            active_pos: Arc::new(Mutex::new(None)),
            dummy: Arc::new(Mutex::new(None)),
            vel: Arc::new(Mutex::new((250.0, 3000.0))),
            steppers: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn set_dummy(&self, dummy: ThExtruder) {
        *self.dummy.lock() = Some(dummy);
    }
    fn last_stepper(&self) -> Arc<FakeStepper> {
        self.steppers.lock().last().cloned().expect("a stepper was created")
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> { vec![0.0; 4] }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> { Ok(()) }
    fn get_kinematics(&self) -> Kinematics { Arc::new(DummyKinematics) }
    fn flush_step_generation(&self) -> KResult<()> { *self.flushes.lock() += 1; Ok(()) }
    fn get_last_move_time(&self) -> KResult<f64> { Ok(0.0) }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _newpos: Vec<f64>, _speed: f64, _drip_completion: &Completion<bool>) -> KResult<()> { Ok(()) }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn get_trapq(&self) -> Trapq { make_trapq(99) }
    fn get_max_velocity(&self) -> (f64, f64) { *self.vel.lock() }
    fn set_max_velocities(&self, _max_velocity: Option<f64>, _max_accel: Option<f64>, _square_corner_velocity: Option<f64>, _min_cruise_ratio: Option<f64>) -> (f64, f64, f64, f64) { (250.0, 3000.0, 5.0, 0.5) }
    fn register_lookahead_callback(&self, callback: LookaheadCb) -> KResult<()> {
        self.lookaheads.lock().push(callback);
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) { (0.0, 0.0, false) }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, extruder: ThExtruder, extrude_pos: f64) {
        *self.active.lock() = Some(extruder);
        *self.active_pos.lock() = Some(extrude_pos);
    }
    fn get_extruder(&self) -> ThExtruder {
        if let Some(active) = self.active.lock().clone() {
            return active;
        }
        self.dummy.lock().clone().expect("dummy extruder")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> { Vec::new() }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory { Arc::new(FakeStepperFactory { steppers: self.steppers.clone() }) }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

// --- FakeMove ---

struct FakeMove {
    start_pos: Vec<f64>,
    end_pos: Vec<f64>,
    axes_d: Vec<f64>,
    axes_r: Vec<f64>,
    move_d: f64,
    accel: f64,
    max_cruise_v2: f64,
    start_v: f64,
    cruise_v: f64,
    accel_t: f64,
    cruise_t: f64,
    decel_t: f64,
    limited: Mutex<Option<(f64, f64)>>,
}

impl Default for FakeMove {
    fn default() -> Self {
        FakeMove {
            start_pos: Vec::new(),
            end_pos: Vec::new(),
            axes_d: Vec::new(),
            axes_r: Vec::new(),
            move_d: 0.0,
            accel: 0.0,
            max_cruise_v2: 0.0,
            start_v: 0.0,
            cruise_v: 0.0,
            accel_t: 0.0,
            cruise_t: 0.0,
            decel_t: 0.0,
            limited: Mutex::new(None),
        }
    }
}

impl FakeMove {
    fn new(start_pos: Vec<f64>, end_pos: Vec<f64>, axes_d: Vec<f64>, axes_r: Vec<f64>) -> Self {
        FakeMove { start_pos, end_pos, axes_d, axes_r, ..Default::default() }
    }
}

impl MoveApi for FakeMove {
    fn start_pos(&self) -> Coord { Coord(self.start_pos.clone()) }
    fn end_pos(&self) -> Coord { Coord(self.end_pos.clone()) }
    fn axes_d(&self) -> Coord { Coord(self.axes_d.clone()) }
    fn axes_r(&self) -> Coord { Coord(self.axes_r.clone()) }
    fn move_d(&self) -> f64 { self.move_d }
    fn is_kinematic_move(&self) -> bool { false }
    fn accel(&self) -> f64 { self.accel }
    fn max_cruise_v2(&self) -> f64 { self.max_cruise_v2 }
    fn start_v(&self) -> f64 { self.start_v }
    fn cruise_v(&self) -> f64 { self.cruise_v }
    fn accel_t(&self) -> f64 { self.accel_t }
    fn cruise_t(&self) -> f64 { self.cruise_t }
    fn decel_t(&self) -> f64 { self.decel_t }
    fn limit_speed(&self, speed: f64, accel: f64) -> KResult<()> { *self.limited.lock() = Some((speed, accel)); Ok(()) }
    fn move_error(&self, msg: String) -> KlippyError { KlippyError::command(msg) }
}

// --- shared env builder ---

struct Env {
    printer: FakePrinter,
    printer_handle: Printer,
    gcode: FakeGCode,
    toolhead: FakeToolhead,
    mq: FakeMQ,
    heaters: FakeHeaters,
    config: FakeConfig,
}

fn build_env(name: &str, opts: Vec<(&str, &str)>, can_extrude: bool) -> Env {
    let printer = FakePrinter::new();
    let printer_handle: Printer = Arc::new(printer.clone());
    let gcode = FakeGCode::new();
    printer.insert("gcode", Arc::new(gcode.clone()));
    let toolhead = FakeToolhead::new();
    printer.insert("toolhead", Arc::new(toolhead.clone()));
    let heaters = FakeHeaters::new(can_extrude);
    printer.insert("heaters", Arc::new(heaters.clone()));
    let mq = FakeMQ::new();
    printer.insert("motion_queuing", Arc::new(mq.clone()));
    let dummy: ThExtruder = DummyExtruder::new(&printer_handle) as ThExtruder;
    toolhead.set_dummy(dummy);
    let config = FakeConfig::new(name, opts, printer_handle.clone());
    Env { printer, printer_handle, gcode, toolhead, mq, heaters, config }
}

// ---------------------------------------------------------------------------
// Comparison helpers for PyValue vs recording Value.
// ---------------------------------------------------------------------------

fn pv_to_value(pv: &PyValue) -> Value {
    match pv {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(v) => Value::Int(v),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(l) => Value::List(l.iter().map(pv_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pv_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (pv_to_value(k), pv_to_value(v))).collect()),
        PyValue::Host(_) => Value::Null,
    }
}

fn reads_value(reads: &[(String, String, String)]) -> Value {
    Value::List(
        reads
            .iter()
            .map(|(n, k, o)| Value::Tuple(vec![Value::str(n), Value::str(k), Value::str(o)]))
            .collect(),
    )
}

fn regs_value(regs: &[String]) -> Value {
    Value::str(&regs.join("\n"))
}

// --- C reference (`chelper_ref`, frozen `klippy/chelper/kin_extruder.c`) ---
//
// The Python fakes record every `extruder_set_pressure_advance` call the
// reference makes (`pa_calls`). The expected end state is computed by
// replaying those observed calls through the real C reference and reading
// its state back, never by a hand-written emulation.

use core::ffi::c_void;

#[repr(C)]
struct ListNode {
    next: *mut ListNode,
    prev: *mut ListNode,
}

#[repr(C)]
struct ListHead {
    root: ListNode,
}

/// `struct stepper_kinematics` (itersolve.h).
#[repr(C)]
#[allow(dead_code)]
struct RefSk {
    step_dist: f64,
    commanded_pos: f64,
    sc: *mut c_void,
    last_flush_time: f64,
    last_move_time: f64,
    tq: *mut c_void,
    active_flags: i32,
    gen_steps_pre_active: f64,
    gen_steps_post_active: f64,
    calc_position_cb: *mut c_void,
    post_cb: *mut c_void,
}

/// `struct pa_params` (kin_extruder.c).
#[repr(C)]
struct CPaParams {
    pressure_advance: f64,
    active_print_time: f64,
    node: ListNode,
}

/// `struct extruder_stepper` (kin_extruder.c).
#[repr(C)]
struct CExtruderStepper {
    sk: RefSk,
    pa_list: ListHead,
    half_smooth_time: f64,
    inv_half_smooth_time2: f64,
}

unsafe extern "C" {
    fn extruder_stepper_alloc() -> *mut CExtruderStepper;
    fn extruder_stepper_free(sk: *mut RefSk);
    fn extruder_set_pressure_advance(sk: *mut RefSk, print_time: f64, pressure_advance: f64, smooth_time: f64);
}

/// Replay the observed reference `extruder_set_pressure_advance` calls on a
/// fresh C allocation and return its end state as a recording value.
fn sk_value_from_pa_calls(pa_calls: &Value) -> Value {
    chelper_ref::link();
    let Value::List(calls) = pa_calls else { panic!("pa_calls must be a list") };
    let mut observed: Vec<(f64, f64, f64)> = Vec::with_capacity(calls.len());
    for call in calls {
        let Value::Tuple(args) = call else { panic!("pa call must be a tuple") };
        observed.push((value_f64(&args[0]), value_f64(&args[1]), value_f64(&args[2])));
    }
    let (half, inv, pre, post, pa_list) = unsafe {
        let es = extruder_stepper_alloc();
        assert!(!es.is_null(), "C extruder_stepper_alloc returned null");
        for (print_time, pressure_advance, smooth_time) in observed {
            extruder_set_pressure_advance(&mut (*es).sk, print_time, pressure_advance, smooth_time);
        }
        let root = &(*es).pa_list.root as *const ListNode as *mut ListNode;
        let mut pa_list = Vec::new();
        let mut cur = (*root).next;
        while cur != root {
            let pa = &*((cur as *const u8).wrapping_sub(core::mem::offset_of!(CPaParams, node)) as *const CPaParams);
            pa_list.push(pa.pressure_advance);
            pa_list.push(pa.active_print_time);
            cur = (*cur).next;
        }
        let state = (
            (*es).half_smooth_time,
            (*es).inv_half_smooth_time2,
            (*es).sk.gen_steps_pre_active,
            (*es).sk.gen_steps_post_active,
            pa_list,
        );
        extruder_stepper_free(&mut (*es).sk);
        state
    };
    sk_state_value((half, inv, pre, post, pa_list))
}

fn sk_state_value((half, inv, pre, post, pa_list): (f64, f64, f64, f64, Vec<f64>)) -> Value {
    let pa: Vec<Value> = pa_list
        .chunks_exact(2)
        .map(|pair| Value::Tuple(vec![Value::float(pair[0]), Value::float(pair[1])]))
        .collect();
    Value::Tuple(vec![
        Value::float(half),
        Value::float(inv),
        Value::float(pre),
        Value::float(post),
        Value::List(pa),
    ])
}

fn sk_snapshot_value(sk: &StepperKinematics) -> Value {
    let state: (f64, f64, f64, f64, Vec<f64>) = klippy_core::motion_handles::kinematics_of(sk)
        .expect("extruder stepper kinematics handle")
        .with(|alloc| match alloc {
            klippy_core::motion_handles::KinAlloc::Extruder(es) => {
                let mut flat = Vec::with_capacity(es.pa_list.len() * 2);
                for entry in &es.pa_list {
                    flat.push(entry.pressure_advance);
                    flat.push(entry.active_print_time);
                }
                (
                    es.half_smooth_time,
                    es.inv_half_smooth_time2,
                    es.sk.gen_steps_pre_active,
                    es.sk.gen_steps_post_active,
                    flat,
                )
            }
            _ => panic!("non-extruder stepper kinematics"),
        });
    sk_state_value(state)
}

fn value_f64(v: &Value) -> f64 {
    match v {
        Value::Float(f) => f.to_f64().expect("recoverable float"),
        Value::Int(i) => *i as f64,
        other => panic!("expected number, got {other:?}"),
    }
}

/// Replace the final oracle tuple element (the observed `pa_calls`) with
/// the C-reference end state of replaying those calls.
fn splice_sk_expectation(expected: Value) -> Value {
    let Value::Tuple(mut parts) = expected else { panic!("expected oracle tuple") };
    let pa_calls = parts.pop().expect("pa_calls element");
    parts.push(sk_value_from_pa_calls(&pa_calls));
    Value::Tuple(parts)
}


// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_matches() {
    let f = module_factory();
    assert!(f.add_printer_objects.is_some());
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("(hasattr(extruder, 'add_printer_objects'), hasattr(extruder, 'load_config'), hasattr(extruder, 'load_config_prefix'), hasattr(extruder, 'DummyExtruder'), hasattr(extruder, 'PrinterExtruder'), hasattr(extruder, 'ExtruderStepper'))");
    check(
        "module_factory",
        expected,
        Value::Tuple(vec![Value::Bool(true), Value::Bool(false), Value::Bool(false), Value::Bool(true), Value::Bool(true), Value::Bool(true)]),
    );
}

#[test]
fn printer_extruder_init_defaults() {
    let Some(mut py) = oracle() else { return };
    py.exec("clear_registries(); del log_lines[:]\np, g, t, h, m, c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)").expect("py build");
    let expected_py = py.eval("((pe.nozzle_diameter, pe.filament_area, pe.max_extrude_ratio, pe.max_e_velocity, pe.max_e_accel, pe.max_e_dist, pe.instant_corner_v), pe.get_status(0.0), list(config_reads), '\\n'.join('|'.join(str(x) for x in r) for r in gcode_regs), list(heater_setup))").expect("py obs");
    logs_clear();
    let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    let actual = Value::Tuple(vec![
        Value::Tuple(vec![
            Value::float(pe.nozzle_diameter()),
            Value::float(pe.filament_area()),
            Value::float(pe.max_extrude_ratio()),
            Value::float(pe.max_e_velocity()),
            Value::float(pe.max_e_accel()),
            Value::float(pe.max_e_dist()),
            Value::float(pe.instant_corner_v()),
        ]),
        pv_to_value(&pe.status(0.0)),
        reads_value(&env.config.reads.lock()),
        regs_value(&env.gcode.regs.lock()),
        Value::List(env.heaters.setup.lock().iter().map(|(n, id)| Value::Tuple(vec![Value::str(n), Value::str(id.as_deref().unwrap_or("None"))])).collect()),
    ]);
    check("printer_extruder_init_defaults", Ok(expected_py), actual);
    let py_logs = py.eval("[l for l in log_lines if l.startswith('Extruder max_extrude_ratio=')]").expect("py logs");
    let want = match &py_logs {
        Value::List(items) if items.len() == 1 => match &items[0] {
            Value::Str(s) => s.clone(),
            other => panic!("py log line: {other:?}"),
        },
        other => panic!("py logs: {other:?}"),
    };
    let logs = logs_snapshot();
    assert!(logs.contains(&want), "rust logs {logs:?} lack {want:?}");
}

#[test]
fn printer_extruder_init_custom_limits() {
    let Some(mut py) = oracle() else { return };
    py.exec("clear_registries(); del log_lines[:]\np, g, t, h, m, c = make_env('extruder1', {'nozzle_diameter': '0.6', 'filament_diameter': '2.85', 'max_extrude_cross_section': '9.0', 'max_extrude_only_velocity': '40.0', 'max_extrude_only_accel': '4000.0', 'max_extrude_only_distance': '25.0', 'instantaneous_corner_velocity': '0.5'})\npe = extruder.PrinterExtruder(c, 1)").expect("py build");
    let expected_py = py.eval("((pe.nozzle_diameter, pe.filament_area, pe.max_extrude_ratio, (pe.max_e_velocity, pe.max_e_accel, pe.max_e_dist, pe.instant_corner_v)), pe.get_status(0.0), list(config_reads), '\\n'.join('|'.join(str(x) for x in r) for r in gcode_regs), list(heater_setup))").expect("py obs");
    let env = build_env("extruder1", vec![("nozzle_diameter", "0.6"), ("filament_diameter", "2.85"), ("max_extrude_cross_section", "9.0"), ("max_extrude_only_velocity", "40.0"), ("max_extrude_only_accel", "4000.0"), ("max_extrude_only_distance", "25.0"), ("instantaneous_corner_velocity", "0.5")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 1).expect("rust build");
    let actual = Value::Tuple(vec![
        Value::Tuple(vec![
            Value::float(pe.nozzle_diameter()),
            Value::float(pe.filament_area()),
            Value::float(pe.max_extrude_ratio()),
            Value::Tuple(vec![
                Value::float(pe.max_e_velocity()),
                Value::float(pe.max_e_accel()),
                Value::float(pe.max_e_dist()),
                Value::float(pe.instant_corner_v()),
            ]),
        ]),
        pv_to_value(&pe.status(0.0)),
        reads_value(&env.config.reads.lock()),
        regs_value(&env.gcode.regs.lock()),
        Value::List(env.heaters.setup.lock().iter().map(|(n, id)| Value::Tuple(vec![Value::str(n), Value::str(id.as_deref().unwrap_or("None"))])).collect()),
    ]);
    check("printer_extruder_init_custom_limits", Ok(expected_py), actual);
}



#[test]
fn check_move_cases() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, &str, &str)> = vec![
        (
            "below_min_temp",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'}, can_extrude=False)\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[0.,0.,0.,10.], axes_r=[0.,0.,0.,1.])",
            "pe.check_move(mv, 3)",
        ),
        (
            "extrude_only_ok",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[0.,0.,0.,10.], axes_r=[0.,0.,0.,2.])",
            "pe.check_move(mv, 3)",
        ),
        (
            "extrude_only_too_long",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[0.,0.,0.,100.], axes_r=[0.,0.,0.,1.])",
            "pe.check_move(mv, 3)",
        ),
        (
            "overextrude_tiny_ok",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[1.,1.,0.,0.1], axes_r=[1.,1.,0.,8.])",
            "pe.check_move(mv, 3)",
        ),
        (
            "overextrude_error",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[1.,1.,0.,10.], axes_r=[1.,1.,0.,8.], move_d=12.0)",
            "pe.check_move(mv, 3)",
        ),
        (
            "motion_move_ok",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[1.,0.,0.,1.], axes_r=[1.,0.,0.,0.2])",
            "pe.check_move(mv, 3)",
        ),
        (
            "retraction_ok",
            "p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(axes_d=[0.,0.,0.,-5.], axes_r=[0.,0.,0.,-1.])",
            "pe.check_move(mv, 3)",
        ),
    ];
    for (label, setup, call) in cases {
        py.exec(setup).expect("py setup");
        py.exec("clear_registries(); del log_lines[:]").expect("clear");
        let expected = py.eval(call);
        // fetch python observations produced by the call
        // (errors are already in `expected`; side effects via registries)
        let py_extra = py.eval("(mv.limited, list(log_lines))").expect("py extra");
        // rust side
        let (can, mv_vec) = match label {
            "below_min_temp" => (false, (vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 0.0, 0.0, 10.0], vec![0.0, 0.0, 0.0, 1.0])),
            "extrude_only_ok" => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 0.0, 0.0, 10.0], vec![0.0, 0.0, 0.0, 2.0])),
            "extrude_only_too_long" => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 0.0, 0.0, 100.0], vec![0.0, 0.0, 0.0, 1.0])),
            "overextrude_tiny_ok" => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![1.0, 1.0, 0.0, 0.1], vec![1.0, 1.0, 0.0, 8.0])),
            "overextrude_error" => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![1.0, 1.0, 0.0, 10.0], vec![1.0, 1.0, 0.0, 8.0])),
            "motion_move_ok" => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![1.0, 0.0, 0.0, 1.0], vec![1.0, 0.0, 0.0, 0.2])),
            _ => (true, (vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 0.0, 0.0, -5.0], vec![0.0, 0.0, 0.0, -1.0])),
        };
        let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], can);
        let config: Config = Arc::new(env.config.clone());
        let pe = PrinterExtruder::new(&config, 0).expect("rust build");
        logs_clear();
        let mut mv = FakeMove::new(mv_vec.0, vec![0.0, 0.0, 0.0, 0.0], mv_vec.1, mv_vec.2);
        if label == "overextrude_error" {
            mv.move_d = 12.0;
        }
        let result = pe.check_move(&mv, 3);
        match expected {
            Ok(_) => {
                if let Err(e) = result {
                    panic!("{label}: rust raised {}: {}", e.class_name(), e.message());
                }
            }
            Err(e) => {
                match result {
                    Ok(()) => panic!("{label}: rust succeeded but oracle raised {}: {}", e.class_name, e.message),
                    Err(actual) => {
                        assert_eq!(e.class_name, actual.class_name(), "{label} class");
                        assert_eq!(e.message, actual.message(), "{label} message");
                    }
                }
            }
        }
        let limited = *mv.limited.lock();
        let actual_extra = Value::Tuple(vec![
            match limited {
                None => Value::Null,
                Some((a, b)) => Value::Tuple(vec![Value::float(a), Value::float(b)]),
            },
            Value::List(logs_snapshot().iter().filter(|s| s.starts_with("Overextrude")).map(|s| Value::str(s)).collect()),
        ]);
        check(&format!("check_move:{label}:side"), Ok(py_extra), actual_extra);
    }
}

#[test]
fn calc_junction_cases() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("same_ratio", 2.0f64, 2.0f64, 25.0f64, 1.0f64),
        ("diff_ratio", 1.0f64, 3.0f64, 25.0f64, 1.0f64),
        ("zero_prev", 0.0f64, 1.5f64, 100.0f64, 2.0f64),
    ];
    for (label, prev_r, r, cruise, icv) in cases {
        let icv_str = format!("{icv:?}");
        py.exec(&format!("p,g,t,h,m,c = make_env('extruder', {{'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'instantaneous_corner_velocity': '{icv_str}'}})\npe = extruder.PrinterExtruder(c, 0)\nprev = FakeMove(axes_r=[0.,0.,0.,{prev_r:?}], max_cruise_v2=0.0)\nmv = FakeMove(axes_r=[0.,0.,0.,{r:?}], max_cruise_v2={cruise:?})"))
            .expect("py setup");
        let expected = py.eval("pe.calc_junction(prev, mv, 3)").expect("py calc");
        let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75"), ("instantaneous_corner_velocity", icv_str.as_str())], true);
        let config: Config = Arc::new(env.config.clone());
        let pe = PrinterExtruder::new(&config, 0).expect("rust build");
        let prev = FakeMove::new(vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0, 0.0, 0.0, prev_r]);
        let mut mv = FakeMove::new(vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0, 0.0, 0.0, r]);
        mv.max_cruise_v2 = cruise;
        let actual = pe.calc_junction(&prev, &mv, 3);
        check(&format!("calc_junction:{label}"), Ok(expected), Value::float(actual));
    }
}

#[test]
fn process_move_cases() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("with_pa_flag", 1.0f64, 1.0f64, 1.0f64, 1.0f64, true),
        ("extrude_only_no_pa", 0.0f64, 0.0f64, 0.0f64, 2.0f64, false),
    ];
    for (label, x_d, e_d, ratio, end, _pa) in cases {
        py.exec(&format!("p,g,t,h,m,c = make_env('extruder', {{'nozzle_diameter': '0.4', 'filament_diameter': '1.75'}})\npe = extruder.PrinterExtruder(c, 0)\nmv = FakeMove(start_pos=[0.,0.,0.,1.5], end_pos=[0.,0.,0.,{end:?}], axes_d=[{x_d:?},0.,0.,{e_d:?}], axes_r=[1.,0.,0.,{ratio:?}], move_d=5.0, accel=100.0, start_v=10.0, cruise_v=50.0, accel_t=0.1, cruise_t=0.5, decel_t=0.1)"))
            .expect("py setup");
        py.exec("clear_registries()").expect("clear");
        py.eval("pe.process_move(100.0, mv, 3)").expect("py process");
        let expected = py.eval("([(c[0],) + tuple(1.0 if v is True else (0.0 if v is False else v) for v in c[1:]) for c in append_calls], pe.last_position)").expect("py obs");
        let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], true);
        let config: Config = Arc::new(env.config.clone());
        let pe = PrinterExtruder::new(&config, 0).expect("rust build");
        let mut mv = FakeMove::new(vec![0.0, 0.0, 0.0, 1.5], vec![0.0, 0.0, 0.0, end], vec![x_d, 0.0, 0.0, e_d], vec![1.0, 0.0, 0.0, ratio]);
        mv.move_d = 5.0;
        mv.accel = 100.0;
        mv.start_v = 10.0;
        mv.cruise_v = 50.0;
        mv.accel_t = 0.1;
        mv.cruise_t = 0.5;
        mv.decel_t = 0.1;
        pe.process_move(100.0, &mv, 3);
        let appends = env.mq.appends.lock().clone();
        let actual = Value::Tuple(vec![
            Value::List(
                appends
                    .iter()
                    .map(|(id, args)| {
                        let mut items = vec![Value::Tuple(vec![Value::str("trapq"), Value::int(*id)])];
                        for a in args {
                            items.push(Value::float(*a));
                        }
                        Value::Tuple(items)
                    })
                    .collect(),
            ),
            Value::float(pe.last_position()),
        ]);
        check(&format!("process_move:{label}"), Ok(expected), actual);
    }
}
#[test]
fn extruder_stepper_connect_applies_pressure_advance() {
    let Some(mut py) = oracle() else { return };
    py.exec("clear_registries(); del log_lines[:]\np,g,t,h,m,c = make_env('extruder', {'pressure_advance': '0.03', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\nes = extruder.ExtruderStepper(c)").expect("py build");
    py.eval("p.send_event('klippy:connect')").expect("py connect");
    let expected = py.eval("((es.pressure_advance, es.pressure_advance_smooth_time, es.get_status(0.0), es.motion_queue), len(flush_calls), len(scan_calls), list(config_reads), '\\n'.join('|'.join(str(x) for x in r) for r in gcode_regs), list(pa_calls))").expect("py obs");
    let expected = splice_sk_expectation(expected);
    logs_clear();
    let env = build_env("extruder", vec![("pressure_advance", "0.03"), ("step_pin", "PC1"), ("dir_pin", "PC2")], true);
    let config: Config = Arc::new(env.config.clone());
    let es = ExtruderStepper::new(&config).expect("rust build");
    env.printer.send_event("klippy:connect", &[]).expect("rust connect");
    let actual = Value::Tuple(vec![
        Value::Tuple(vec![
            Value::float(es.pressure_advance()),
            Value::float(es.pressure_advance_smooth_time()),
            pv_to_value(&ExtruderStepperApi::get_status(&*es, 0.0)),
            Value::Null,
        ]),
        Value::int(*env.toolhead.flushes.lock() as i64),
        Value::int(*env.mq.scans.lock() as i64),
        reads_value(&env.config.reads.lock()),
        regs_value(&env.gcode.regs.lock()),
        sk_snapshot_value(&es.kinematics()),
    ]);
    check("extruder_stepper_connect", Ok(expected), actual);
}

#[test]
fn set_pressure_advance_command_paths() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("explicit_flush_path", "ADVANCE=0.05 SMOOTH_TIME=0.1"),
        ("defaults_lookahead_path", ""),
        ("zero_pa", "ADVANCE=0.0"),
        ("advance_negative", "ADVANCE=-1.0"),
        ("smooth_time_too_large", "SMOOTH_TIME=0.3"),
    ];
    for (label, params_str) in cases {
        // python: fresh stepper, initial state (pa=0, smooth=0)
        py.exec("clear_registries(); del log_lines[:]\np,g,t,h,m,c = make_env('extruder', {'step_pin': 'PC1', 'dir_pin': 'PC2'})\nes = extruder.ExtruderStepper(c)").expect("py build");
        let params_literal = {
            let mut parts = Vec::new();
            for tok in params_str.split_whitespace() {
                let (k, v) = tok.split_once('=').unwrap();
                parts.push(format!("'{k}': '{v}'"));
            }
            format!("{{{}}}", parts.join(", "))
        };
        // Invoke lookahead callbacks like the Rust side (at print time 7.0)
        // so the reference records the same pressure-advance calls.
        py.exec(&format!("clear_registries(); gc = FakeGCmd('SET_PRESSURE_ADVANCE', 'SET_PRESSURE_ADVANCE {params_str}', {params_literal})\ntry:\n es.cmd_SET_PRESSURE_ADVANCE(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))\nfor _cb in list(lookahead):\n _cb(7.0)")).expect("py cmd");
        let expected = py.eval("(out, (es.pressure_advance, es.pressure_advance_smooth_time, es.get_status(0.0)), list(responds), list(rollover), len(flush_calls), len(lookahead), list(pa_calls))").expect("py obs");
        let expected = splice_sk_expectation(expected);
        // rust
        logs_clear();
        let env = build_env("extruder", vec![("step_pin", "PC1"), ("dir_pin", "PC2")], true);
        let config: Config = Arc::new(env.config.clone());
        let es = ExtruderStepper::new(&config).expect("rust build");
        let mut params = IndexMap::new();
        for tok in params_str.split_whitespace() {
            let (k, v) = tok.split_once('=').unwrap();
            params.insert(k.to_string(), v.to_string());
        }
        let fake_cmd = FakeGCmd::new("SET_PRESSURE_ADVANCE", &format!("SET_PRESSURE_ADVANCE {params_str}"), params);
        let gcmd: GCmd = Arc::new(fake_cmd.clone());
        let flush_before = *env.toolhead.flushes.lock();
        let result = es.cmd_set_pressure_advance(&gcmd);
        let actual_out: Value = match &result {
            Ok(()) => Value::str("ok"),
            Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        {
            let mut cbs = env.toolhead.lookaheads.lock();
            for cb in cbs.iter_mut() {
                cb(7.0).expect("lookahead cb");
            }
        }
        let actual = Value::Tuple(vec![
            actual_out,
            Value::Tuple(vec![
                Value::float(es.pressure_advance()),
                Value::float(es.pressure_advance_smooth_time()),
                pv_to_value(&ExtruderStepperApi::get_status(&*es, 0.0)),
            ]),
            Value::List(fake_cmd.responds.lock().iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
            Value::List(env.printer.rollover.lock().iter().map(|(n, info, l)| Value::Tuple(vec![Value::str(n), Value::str(info.as_deref().unwrap_or("None")), Value::Bool(*l)])).collect()),
            Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
            Value::int(env.toolhead.lookaheads.lock().len() as i64),
            sk_snapshot_value(&es.kinematics()),
        ]);
        check(&format!("set_pressure_advance:{label}"), Ok(expected), actual);
    }
}

#[test]
fn set_e_rotation_distance_cases() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("query", ""),
        ("positive", "DISTANCE=2.5"),
        ("negative", "DISTANCE=-2.5"),
        ("zero", "DISTANCE=0.0"),
    ];
    for (label, params_str) in cases {
        py.exec("clear_registries()\np,g,t,h,m,c = make_env('extruder', {'step_pin': 'PC1', 'dir_pin': 'PC2'})\nes = extruder.ExtruderStepper(c)").expect("py build");
        let params_literal = {
            let mut parts = Vec::new();
            for tok in params_str.split_whitespace() {
                let (k, v) = tok.split_once('=').unwrap();
                parts.push(format!("'{k}': '{v}'"));
            }
            format!("{{{}}}", parts.join(", "))
        };
        py.exec(&format!("clear_registries(); gc = FakeGCmd('SET_EXTRUDER_ROTATION_DISTANCE', 'SET_EXTRUDER_ROTATION_DISTANCE {params_str}', {params_literal})\ntry:\n es.cmd_SET_E_ROTATION_DISTANCE(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))")).expect("py cmd");
        let expected = py.eval("(out, list(responds), (es.stepper.rotation_distance, es.stepper.dir_inverted), len(flush_calls))").expect("py obs");
        let env = build_env("extruder", vec![("step_pin", "PC1"), ("dir_pin", "PC2")], true);
        let config: Config = Arc::new(env.config.clone());
        let es = ExtruderStepper::new(&config).expect("rust build");
        let fake = env.toolhead.last_stepper();
        let mut params = IndexMap::new();
        for tok in params_str.split_whitespace() {
            let (k, v) = tok.split_once('=').unwrap();
            params.insert(k.to_string(), v.to_string());
        }
        let fake_cmd = FakeGCmd::new("SET_EXTRUDER_ROTATION_DISTANCE", &format!("SET_EXTRUDER_ROTATION_DISTANCE {params_str}"), params);
        let gcmd: GCmd = Arc::new(fake_cmd.clone());
        let result = es.cmd_set_e_rotation_distance(&gcmd);
        let actual_out = match &result {
            Ok(()) => Value::str("ok"),
            Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        let responds = fake_cmd.responds.lock().clone();
        let (rd, _spr) = *fake.rotation.lock();
        let (d0, d1) = *fake.dir.lock();
        let actual = Value::Tuple(vec![
            actual_out,
            Value::List(responds.iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
            Value::Tuple(vec![Value::Tuple(vec![Value::float(rd), Value::float(_spr)]), Value::Tuple(vec![Value::Bool(d0), Value::Bool(d1)])]),
            Value::int(*env.toolhead.flushes.lock() as i64),
        ]);
        check(&format!("set_e_rotation_distance:{label}"), Ok(expected), actual);
    }
}

#[test]
fn sync_extruder_motion_cases() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("sync_valid", "MOTION_QUEUE=extruder", true),
        ("sync_missing", "", false),
        ("sync_empty", "MOTION_QUEUE=", true),
    ];
    for (label, params_str, _has) in cases {
        py.exec("clear_registries()\np,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\npe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\nes = pe.extruder_stepper\nclear_registries()").expect("py build");
        let params_literal = {
            let mut parts = Vec::new();
            for tok in params_str.split_whitespace() {
                let (k, v) = tok.split_once('=').unwrap();
                parts.push(format!("'{k}': '{v}'"));
            }
            format!("{{{}}}", parts.join(", "))
        };
        py.exec(&format!("gc = FakeGCmd('SYNC_EXTRUDER_MOTION', 'SYNC_EXTRUDER_MOTION {params_str}', {params_literal})\ntry:\n es.cmd_SYNC_EXTRUDER_MOTION(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))")).expect("py cmd");
        let expected = py.eval("(out, (es.motion_queue, es.stepper.get_trapq(), es.stepper.position if hasattr(es.stepper, 'position') else None), list(responds), list(rollover) if 'rollover' in globals() else [], len(flush_calls), len(scan_calls))").expect("py obs");
        let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75"), ("step_pin", "PC1"), ("dir_pin", "PC2")], true);
        let config: Config = Arc::new(env.config.clone());
        let pe = PrinterExtruder::new(&config, 0).expect("rust build");
        env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
        let es = pe.extruder_stepper_concrete().expect("stepper");
        let stepper = es.stepper();
        let flush_before = *env.toolhead.flushes.lock();
        let mut params = IndexMap::new();
        for tok in params_str.split_whitespace() {
            let (k, v) = tok.split_once('=').unwrap();
            params.insert(k.to_string(), v.to_string());
        }
        let fake_cmd = FakeGCmd::new("SYNC_EXTRUDER_MOTION", &format!("SYNC_EXTRUDER_MOTION {params_str}"), params);
        let gcmd: GCmd = Arc::new(fake_cmd.clone());
        let result = es.cmd_sync_extruder_motion(&gcmd);
        let actual_out = match &result {
            Ok(()) => Value::str("ok"),
            Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        let mq_val = match es.motion_queue() {
            None => Value::Null,
            Some(q) => Value::str(&q),
        };
        let trapq_val = stepper.get_trapq().map(|t| trapq_id(&t)).map(|id| Value::Tuple(vec![Value::str("trapq"), Value::int(id)])).unwrap_or(Value::Null);
        let fake_stepper = env.toolhead.last_stepper();
        let pos = *fake_stepper.position.lock();
        let pos_val = match pos {
            None => Value::Null,
            Some(p) => Value::List(vec![Value::float(p[0]), Value::float(p[1]), Value::float(p[2])]),
        };
        let responds = fake_cmd.responds.lock().clone();
        let actual = Value::Tuple(vec![
            actual_out,
            Value::Tuple(vec![mq_val, trapq_val, pos_val]),
            Value::List(responds.iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
            Value::List(Vec::new()),
            Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
            Value::int(*env.mq.scans.lock() as i64),
        ]);
        check(&format!("sync_extruder_motion:{label}"), Ok(expected), actual);
    }
}

#[test]
fn cmd_m104_variants() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        ("plain", 0, "S=200", "M104"),
        ("plain109", 0, "S=200", "M109"),
        ("zero_no_temp", 0, "T=3 S=0", "M104"),
        ("missing_hot", 0, "T=3 S=200", "M104"),
        ("t0_explicit", 0, "T=0 S=180", "M104"),
    ];
    for (label, _idx, params_str, cmd_name) in cases {
        py.exec("clear_registries()\np,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\nt.active = pe\nclear_registries()").expect("py build");
        let params_literal = {
            let mut parts = Vec::new();
            for tok in params_str.split_whitespace() {
                let (k, v) = tok.split_once('=').unwrap();
                parts.push(format!("'{k}': '{v}'"));
            }
            format!("{{{}}}", parts.join(", "))
        };
        let method = if cmd_name == "M109" { "pe.cmd_M109(gc)" } else { "pe.cmd_M104(gc)" };
        py.exec(&format!("gc = FakeGCmd('{cmd_name}', '{cmd_name} {params_str}', {params_literal})\ntry:\n {method}\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))")).expect("py cmd");
        let expected = py.eval("(out, list(set_temp_calls), list(responds))").expect("py obs");
        let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], true);
        let config: Config = Arc::new(env.config.clone());
        let pe: Arc<PrinterExtruder> = PrinterExtruder::new(&config, 0).expect("rust build");
        env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
        let pe_handle: ThExtruder = pe.clone();
        env.toolhead.set_extruder(pe_handle, 0.0);
        let mut params = IndexMap::new();
        for tok in params_str.split_whitespace() {
            let (k, v) = tok.split_once('=').unwrap();
            params.insert(k.to_string(), v.to_string());
        }
        let fake_cmd = FakeGCmd::new(cmd_name, &format!("{cmd_name} {params_str}"), params);
        let gcmd: GCmd = Arc::new(fake_cmd.clone());
        let wait = cmd_name == "M109";
        let result = pe.cmd_m104(&gcmd, wait);
        let actual_out = match &result {
            Ok(()) => Value::str("ok"),
            Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        let responds = fake_cmd.responds.lock().clone();
        let actual = Value::Tuple(vec![
            actual_out,
            Value::List(env.heaters.set_temps.lock().iter().map(|(t, w)| Value::Tuple(vec![Value::float(*t), Value::Bool(*w)])).collect()),
            Value::List(responds.iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
        ]);
        check(&format!("cmd_m104:{label}"), Ok(expected), actual);
    }
}

#[test]
fn activate_extruder_cases() {
    let Some(mut py) = oracle() else { return };
    // python: two extruders with toolhead driving active
    py.exec("clear_registries()\np,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\nclear_registries()").expect("py build");
    let expected = py.eval("(t.active is pe)").expect("active after init");
    let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    let active_is_pe = {
        let a = env.toolhead.active.lock().clone();
        match a {
            Some(h) => Arc::ptr_eq(&h, &(pe.clone() as ThExtruder)),
            None => false,
        }
    };
    check("activate_init_active", Ok(expected), Value::Bool(active_is_pe));
    // already active
    py.exec("gc = FakeGCmd('ACTIVATE_EXTRUDER', 'ACTIVATE_EXTRUDER EXTRUDER=extruder', {'EXTRUDER': 'extruder'})\nclear_registries(); pe.cmd_ACTIVATE_EXTRUDER(gc)").expect("py activate again");
    let expected2 = py.eval("(list(responds), len(flush_calls), [e[0] for e in events])").expect("py obs2");
    let flush_before = *env.toolhead.flushes.lock();
    let fake_cmd = FakeGCmd::from_params("ACTIVATE_EXTRUDER", "ACTIVATE_EXTRUDER EXTRUDER=extruder", vec![("EXTRUDER", "extruder")]);
    let gcmd: GCmd = Arc::new(fake_cmd.clone());
    pe.cmd_activate_extruder(&gcmd).expect("activate");
    let responds = fake_cmd.responds.lock().clone();
    let actual2 = Value::Tuple(vec![
        Value::List(responds.iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
        Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
        Value::List(env.printer.events.lock().iter().map(|e| Value::str(e)).collect()),
    ]);
    check("activate_already_active", Ok(expected2), actual2);
    // switch: pretend dummy is active
    py.exec("t.active = t.dummy\ngc = FakeGCmd('ACTIVATE_EXTRUDER', 'ACTIVATE_EXTRUDER EXTRUDER=extruder', {'EXTRUDER': 'extruder'})\nclear_registries(); pe.cmd_ACTIVATE_EXTRUDER(gc)").expect("py switch");
    let expected3 = py.eval("(list(responds), len(flush_calls), [e[0] for e in events], (t.active is pe, t.active_pos))").expect("py obs3");
    let flush_before = *env.toolhead.flushes.lock();
    let dummy: ThExtruder = DummyExtruder::new(&env.printer_handle) as ThExtruder;
    env.toolhead.set_extruder(dummy, 0.0);
    let fake_cmd = FakeGCmd::from_params("ACTIVATE_EXTRUDER", "ACTIVATE_EXTRUDER EXTRUDER=extruder", vec![("EXTRUDER", "extruder")]);
    let gcmd: GCmd = Arc::new(fake_cmd.clone());
    pe.cmd_activate_extruder(&gcmd).expect("switch");
    let responds = fake_cmd.responds.lock().clone();
    let active_is_pe = env.toolhead.active.lock().clone().map(|h| Arc::ptr_eq(&h, &(pe.clone() as ThExtruder))).unwrap_or(false);
    let actual3 = Value::Tuple(vec![
        Value::List(responds.iter().map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)])).collect()),
        Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
        Value::List(env.printer.events.lock().iter().map(|e| Value::str(e)).collect()),
        Value::Tuple(vec![Value::Bool(active_is_pe), Value::float(env.toolhead.active_pos.lock().unwrap_or(0.0))]),
    ]);
    check("activate_switch", Ok(expected3), actual3);
}

#[test]
fn find_past_position_and_dummy() {
    let Some(mut py) = oracle() else { return };
    py.exec("p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\npe = extruder.PrinterExtruder(c, 0)\nd = extruder.DummyExtruder(p)").expect("py build");
    let expected = py.eval("(pe.find_past_position(10.0), d.find_past_position(10.0), d.get_name(), d.get_axis_gcode_id(), d.get_trapq() is None, d.calc_junction(FakeMove(max_cruise_v2=3.0), FakeMove(), 3))").expect("py obs");
    let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75"), ("step_pin", "PC1"), ("dir_pin", "PC2")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    let dummy = DummyExtruder::new(&env.printer_handle);
    let past = <PrinterExtruder as klippy_api::toolhead::PrinterExtruderApi>::find_past_position(&*pe, 10.0).expect("past");
    let dummy_past = <DummyExtruder as klippy_api::toolhead::PrinterExtruderApi>::find_past_position(&*dummy, 10.0).expect("dummy past");
    let mut prev = FakeMove::new(vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4]);
    prev.max_cruise_v2 = 3.0;
    let mv = FakeMove::new(vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4]);
    let actual = Value::Tuple(vec![
        Value::float(past),
        Value::float(dummy_past),
        Value::str(&<DummyExtruder as klippy_api::toolhead::ExtraAxisApi>::get_name(&*dummy)),
        Value::str(<DummyExtruder as klippy_api::toolhead::ExtraAxisApi>::get_axis_gcode_id(&*dummy).as_deref().unwrap_or("")),
        Value::Bool(<DummyExtruder as klippy_api::toolhead::ExtraAxisApi>::get_trapq(&*dummy).is_none()),
        Value::float(<DummyExtruder as klippy_api::toolhead::ExtraAxisApi>::calc_junction(&*dummy, &prev, &mv, 3)),
    ]);
    check("find_past_and_dummy", Ok(expected), actual);
    let expected_err = py.exec("d.check_move(FakeMove(), 3)");
    let mv2 = FakeMove::new(vec![0.0; 4], vec![0.0; 4], vec![0.0; 4], vec![0.0; 4]);
    let rust_err = <DummyExtruder as klippy_api::toolhead::ExtraAxisApi>::check_move(&*dummy, &mv2, 3).expect_err("dummy check_move");
    check_err("dummy_check_move", expected_err, rust_err);
    let expected_err2 = py.exec("d.get_heater()");
    let rust_err2 = <DummyExtruder as klippy_api::toolhead::PrinterExtruderApi>::get_heater(&*dummy).err().expect("dummy heater");
    check_err("dummy_get_heater", expected_err2, rust_err2);
    py.exec("es = extruder.ExtruderStepper(FakeConfig(p, 'extruder1', {'step_pin': 'PC1', 'dir_pin': 'PC2'}))").expect("py es");
    let expected3 = py.eval("es.find_past_position(10.0)").expect("py es past");
    let es_config: Config = Arc::new(FakeConfig::new("extruder1", vec![("step_pin", "PC1"), ("dir_pin", "PC2")], env.printer_handle.clone()));
    let es = ExtruderStepper::new(&es_config).expect("rust es");
    let past3 = <ExtruderStepper as klippy_api::toolhead::ExtruderStepperApi>::find_past_position(&*es, 10.0).expect("past3");
    check("stepper_find_past", Ok(expected3), Value::float(past3));
}

#[test]
fn default_set_pressure_advance_cases() {
    let Some(mut py) = oracle() else { return };
    // active extruder has no stepper
    py.exec("p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'})\npe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\nt.active = pe\nes_default = extruder.ExtruderStepper(FakeConfig(p, 'extruder', {'step_pin': 'PC1', 'dir_pin': 'PC2'}))\ngc = FakeGCmd('SET_PRESSURE_ADVANCE', 'SET_PRESSURE_ADVANCE ADVANCE=0.01', {'ADVANCE': '0.01'})").expect("py default setup");
    let expected_exc = py.exec("es_default.cmd_default_SET_PRESSURE_ADVANCE(gc)").expect_err("py default nosetp must fail");
    let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
    let pe_handle: ThExtruder = pe.clone();
    env.toolhead.set_extruder(pe_handle, 0.0);
    let es_config: Config = Arc::new(FakeConfig::new("extruder", vec![("step_pin", "PC1"), ("dir_pin", "PC2")], env.printer_handle.clone()));
    let es = ExtruderStepper::new(&es_config).expect("rust es");
    let gcmd: GCmd = Arc::new(FakeGCmd::from_params("SET_PRESSURE_ADVANCE", "SET_PRESSURE_ADVANCE ADVANCE=0.01", vec![("ADVANCE", "0.01")]));
    let rust_err = es.cmd_default_set_pressure_advance(&gcmd).expect_err("rust nosetp");
    assert_eq!(expected_exc.class_name, rust_err.class_name(), "default nosetp class");
    assert_eq!(expected_exc.message, rust_err.message(), "default nosetp msg");
}

#[test]
fn get_status_merging() {
    let Some(mut py) = oracle() else { return };
    py.exec("p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2', 'pressure_advance': '0.02'})\npe = extruder.PrinterExtruder(c, 0)").expect("py build");
    let expected = py.eval("pe.get_status(0.0)").expect("obs");
    let env = build_env("extruder", vec![("nozzle_diameter", "0.4"), ("filament_diameter", "1.75"), ("step_pin", "PC1"), ("dir_pin", "PC2"), ("pressure_advance", "0.02")], true);
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    check("get_status_with_stepper", Ok(expected), pv_to_value(&pe.status(0.0)));
}

#[test]
fn add_printer_objects_registers() {
    let Some(mut py) = oracle() else { return };
    py.exec("clear_registries()\np,g,t,h,m,c0 = make_env('extruder', {})\nsections = {'extruder': {'nozzle_diameter': '0.4', 'filament_diameter': '1.75'}, 'extruder1': {'nozzle_diameter': '0.6', 'filament_diameter': '2.85'}}\nctop = FakeConfig(p, 'printer', {}, sections)\nextruder.add_printer_objects(ctop)").expect("py add");
    let expected = py.eval("sorted(p.objects.keys())").expect("obs");
    let env = build_env("printer", vec![], true);
    let mut sections = IndexMap::new();
    let mut e0 = IndexMap::new();
    e0.insert("nozzle_diameter".to_string(), "0.4".to_string());
    e0.insert("filament_diameter".to_string(), "1.75".to_string());
    sections.insert("extruder".to_string(), e0);
    let mut e1 = IndexMap::new();
    e1.insert("nozzle_diameter".to_string(), "0.6".to_string());
    e1.insert("filament_diameter".to_string(), "2.85".to_string());
    sections.insert("extruder1".to_string(), e1);
    let mut top = FakeConfig::new("printer", vec![], env.printer_handle.clone());
    top.sections = sections;
    top.reads = env.config.reads.clone();
    let config: Config = Arc::new(top);
    add_printer_objects(&config).expect("rust add");
    let names: Vec<Value> = env.printer.objects.lock().keys().map(|k| Value::str(k)).collect();
    let mut sorted = names.clone();
    sorted.sort_by_key(|a| a.render());
    // Oracle `sorted()` sorts the strings; `Value::str` rendering keeps that order.
    check("add_printer_objects", Ok(expected), Value::List(sorted));
}

#[test]
fn sync_extruder_motion_invalid_targets() {
    let Some(mut py) = oracle() else { return };
    let cases = [("unknown", "nope", false), ("not_extruder", "notanextruder", true)];
    for (label, target, register) in cases {
        let mut src = String::from("clear_registries()\n");
        src.push_str("p,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\n");
        src.push_str("pe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\n");
        if register {
            src.push_str(&format!("p.objects['{target}'] = object()\n"));
        }
        src.push_str("es = pe.extruder_stepper\nclear_registries()\n");
        src.push_str(&format!(
            "gc = FakeGCmd('SYNC_EXTRUDER_MOTION', 'SYNC_EXTRUDER_MOTION MOTION_QUEUE={target}', {{'MOTION_QUEUE': '{target}'}})\n"
        ));
        src.push_str("try:\n es.cmd_SYNC_EXTRUDER_MOTION(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))");
        py.exec(&src).expect("py setup");
        let expected = py
            .eval("(out, es.motion_queue, len(flush_calls), len(scan_calls), list(responds))")
            .expect("py obs");
        let env = build_env(
            "extruder",
            vec![
                ("nozzle_diameter", "0.4"),
                ("filament_diameter", "1.75"),
                ("step_pin", "PC1"),
                ("dir_pin", "PC2"),
            ],
            true,
        );
        let config: Config = Arc::new(env.config.clone());
        let pe = PrinterExtruder::new(&config, 0).expect("rust build");
        env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
        if register {
            env.printer.insert(target, Arc::new(FakeGCode::new()));
        }
        let es = pe.extruder_stepper_concrete().expect("stepper");
        let flush_before = *env.toolhead.flushes.lock();
        let fake_cmd = FakeGCmd::from_params(
            "SYNC_EXTRUDER_MOTION",
            &format!("SYNC_EXTRUDER_MOTION MOTION_QUEUE={target}"),
            vec![("MOTION_QUEUE", target)],
        );
        let gcmd: GCmd = Arc::new(fake_cmd.clone());
        let result = es.cmd_sync_extruder_motion(&gcmd);
        let actual_out = match &result {
            Ok(()) => Value::str("ok"),
            Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
        };
        let mq_val = match es.motion_queue() {
            None => Value::Null,
            Some(q) => Value::str(&q),
        };
        let actual = Value::Tuple(vec![
            actual_out,
            mq_val,
            Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
            Value::int(*env.mq.scans.lock() as i64),
            Value::List(
                fake_cmd
                    .responds
                    .lock()
                    .iter()
                    .map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)]))
                    .collect(),
            ),
        ]);
        check(&format!("sync_invalid:{label}"), Ok(expected), actual);
    }
}

#[test]
fn default_pressure_advance_command_paths() {
    let Some(mut py) = oracle() else { return };
    // The active extruder's stepper rides on the extruder trapq: the command
    // reaches the stepper's own SET_PRESSURE_ADVANCE path.
    py.exec("clear_registries(); del log_lines[:]\np,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\npe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\nt.active = pe\nclear_registries()\ngc = FakeGCmd('SET_PRESSURE_ADVANCE', 'SET_PRESSURE_ADVANCE ADVANCE=0.01', {'ADVANCE': '0.01'})\ntry:\n pe.extruder_stepper.cmd_default_SET_PRESSURE_ADVANCE(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))\nfor _cb in list(lookahead):\n _cb(7.0)").expect("py default");
    let expected = py
        .eval("(out, list(responds), list(rollover), len(flush_calls), list(pa_calls))")
        .expect("py obs");
    let expected = splice_sk_expectation(expected);
    let env = build_env(
        "extruder",
        vec![
            ("nozzle_diameter", "0.4"),
            ("filament_diameter", "1.75"),
            ("step_pin", "PC1"),
            ("dir_pin", "PC2"),
        ],
        true,
    );
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
    let pe_handle: ThExtruder = pe.clone();
    env.toolhead.set_extruder(pe_handle, 0.0);
    let es = pe.extruder_stepper_concrete().expect("stepper");
    let flush_before = *env.toolhead.flushes.lock();
    let fake_cmd = FakeGCmd::from_params(
        "SET_PRESSURE_ADVANCE",
        "SET_PRESSURE_ADVANCE ADVANCE=0.01",
        vec![("ADVANCE", "0.01")],
    );
    let gcmd: GCmd = Arc::new(fake_cmd.clone());
    let result = es.cmd_default_set_pressure_advance(&gcmd);
    let actual_out = match &result {
        Ok(()) => Value::str("ok"),
        Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
    };
    {
        let mut cbs = env.toolhead.lookaheads.lock();
        for cb in cbs.iter_mut() {
            cb(7.0).expect("lookahead cb");
        }
    }
    let actual = Value::Tuple(vec![
        actual_out,
        Value::List(
            fake_cmd
                .responds
                .lock()
                .iter()
                .map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)]))
                .collect(),
        ),
        Value::List(
            env.printer
                .rollover
                .lock()
                .iter()
                .map(|(n, info, l)| {
                    Value::Tuple(vec![
                        Value::str(n),
                        Value::str(info.as_deref().unwrap_or("None")),
                        Value::Bool(*l),
                    ])
                })
                .collect(),
        ),
        Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
        sk_snapshot_value(&es.kinematics()),
    ]);
    check("default_pa:synced", Ok(expected), actual);

    // The stepper is on a foreign trapq: the command refuses to guess.
    py.exec("clear_registries()\np,g,t,h,m,c = make_env('extruder', {'nozzle_diameter': '0.4', 'filament_diameter': '1.75', 'step_pin': 'PC1', 'dir_pin': 'PC2'})\npe = extruder.PrinterExtruder(c, 0)\np.objects['extruder'] = pe\nt.active = pe\npe.extruder_stepper.stepper.set_trapq(('trapq', 99))\nclear_registries()\ngc = FakeGCmd('SET_PRESSURE_ADVANCE', 'SET_PRESSURE_ADVANCE ADVANCE=0.01', {'ADVANCE': '0.01'})\ntry:\n pe.extruder_stepper.cmd_default_SET_PRESSURE_ADVANCE(gc)\n out = 'ok'\nexcept Exception as e:\n out = (type(e).__name__, str(e))").expect("py unsynced");
    let expected = py
        .eval("(out, list(responds), list(rollover), len(flush_calls), list(pa_calls))")
        .expect("py obs");
    let env = build_env(
        "extruder",
        vec![
            ("nozzle_diameter", "0.4"),
            ("filament_diameter", "1.75"),
            ("step_pin", "PC1"),
            ("dir_pin", "PC2"),
        ],
        true,
    );
    let config: Config = Arc::new(env.config.clone());
    let pe = PrinterExtruder::new(&config, 0).expect("rust build");
    env.printer.add_object("extruder", pe.clone() as ObjectRef).expect("add");
    let pe_handle: ThExtruder = pe.clone();
    env.toolhead.set_extruder(pe_handle, 0.0);
    let es = pe.extruder_stepper_concrete().expect("stepper");
    es.stepper().set_trapq(Some(make_trapq(99)));
    let flush_before = *env.toolhead.flushes.lock();
    let fake_cmd = FakeGCmd::from_params(
        "SET_PRESSURE_ADVANCE",
        "SET_PRESSURE_ADVANCE ADVANCE=0.01",
        vec![("ADVANCE", "0.01")],
    );
    let gcmd: GCmd = Arc::new(fake_cmd.clone());
    let result = es.cmd_default_set_pressure_advance(&gcmd);
    let actual_out = match &result {
        Ok(()) => Value::str("ok"),
        Err(e) => Value::Tuple(vec![Value::str(e.class_name()), Value::str(e.message())]),
    };
    let actual = Value::Tuple(vec![
        actual_out,
        Value::List(
            fake_cmd
                .responds
                .lock()
                .iter()
                .map(|(m, l)| Value::Tuple(vec![Value::str(m), Value::Bool(*l)]))
                .collect(),
        ),
        Value::List(
            env.printer
                .rollover
                .lock()
                .iter()
                .map(|(n, info, l)| {
                    Value::Tuple(vec![
                        Value::str(n),
                        Value::str(info.as_deref().unwrap_or("None")),
                        Value::Bool(*l),
                    ])
                })
                .collect(),
        ),
        Value::int((*env.toolhead.flushes.lock() - flush_before) as i64),
        Value::List(Vec::new()),
    ]);
    check("default_pa:foreign_trapq", Ok(expected), actual);
}


