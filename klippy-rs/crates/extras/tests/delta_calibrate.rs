//! Parity tests for `extras::delta_calibrate` (port of
//! `klippy/extras/delta_calibrate.py`).
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + the frozen `klippy/`; the test skips when
//! `KLIPPY_PYTHON` is unset). The oracle builds the real
//! `DeltaCalibrate` with minimal Python fakes mirroring the Rust fakes
//! below (config, printer, g-code, toolhead/kinematics, probe helper),
//! and both sides are driven with identical inputs. Compared exactly:
//! probe points, restored state, `delta_analyze_entry`, G-code command
//! registrations, `DELTA_ANALYZE` parsing results and every error
//! (exception class + message), `measurements_to_distances` output
//! (floats by bits) and `handle_connect` rejection.

use extras::delta_calibrate::{measurements_to_distances, DeltaCalibrate, StablePosition};
use extras::manual_probe::ProbeResult;
use extras::probe::ProbePointsHelper;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetFloatListOpts, GetFloatOpts as CfgFloatOpts, GetListsOpts, GetStrOpts,
    PrinterConfig, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GetFloatOpts as GCmdFloatOpts, GetOpts, GetParser,
    RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::McuStepperApi;
use klippy_api::toolhead::{
    no_kinematics_attr, DeltaCalibration, DeltaCalibrationApi, HomingTargetApi, Kinematics,
    KinematicsApi, ToolHeadApi,
};
use kos::sync::Mutex;
use pycompat::text::{float_from_string, float_str, percent_format, PctValue};
use pycompat::PyValue;
use reactor::Reactor;
use recording::Value as Rv;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Oracle: Python fakes shared by every test.
// ---------------------------------------------------------------------------
// Extras modules import as `extras.<module>`: the oracle puts `klippy/`
// itself on `sys.path` and `klippy` is not a package, so bare
// `import probe` / `import delta_calibrate` raise `ModuleNotFoundError`.
// Core modules under `klippy/` stay bare (`import configfile`).
const SETUP: &str = r#"
import math, logging, collections
import extras.probe as probe
import extras.manual_probe as manual_probe
import extras.delta_calibrate as delta_calibrate
import configfile
import gcode as gcode_mod
ConfigError = configfile.error
CommandError = gcode_mod.CommandError

probe_calls = []
_RealHelper = probe.ProbePointsHelper
class FakeHelper:
    def __init__(self, config, finalize, default_points=None):
        self.config = config
        self.finalize = finalize
        self.default_points = default_points
        probe_calls.append(self)
    def minimum_points(self, n):
        if len(self.default_points) < n:
            raise configfile.error("Need at least %d probe points for %s" % (n, self.config.get_name()))
    def start_probe(self, gcmd):
        probe_calls.append(("start_probe", gcmd.get_command()))
probe.ProbePointsHelper = FakeHelper
_sentinel = object()
class FakeConfig:
    def __init__(self, printer, section, opts):
        self._printer = printer
        self._section = section
        self._opts = opts
    def get_printer(self): return self._printer
    def get_name(self): return self._section
    def get(self, option, default=_sentinel):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self._section))
        return self._opts[option]
    def _check(self, option, v, minval=None, maxval=None, above=None, below=None):
        if minval is not None and v < minval:
            raise ConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise ConfigError("Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        if above is not None and v <= above:
            raise ConfigError("Option '%s' in section '%s' must be above %s" % (option, self._section, above))
        if below is not None and v >= below:
            raise ConfigError("Option '%s' in section '%s' must be below %s" % (option, self._section, below))
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self._section))
        try:
            v = float(self._opts[option])
        except Exception:
            raise ConfigError("Unable to parse option '%s' in section '%s'" % (option, self._section))
        return self._check(option, v, minval, maxval, above, below)
    def getfloatlist(self, option, default=_sentinel, count=None, **kw):
        if option not in self._opts:
            if default is not _sentinel:
                return default
            raise ConfigError("Option '%s' in section '%s' must be specified" % (option, self._section))
        raw = self._opts[option]
        if len(raw.strip()) == 0:
            parts = []
        else:
            parts = [p.strip() for p in raw.split(',')]
        res = [float(p) for p in parts]
        if count is not None and len(res) != count:
            raise ConfigError("Option '%s' in section '%s' must have %d elements" % (option, self._section, count))
        # `configfile.getlists` ends with `return tuple(res)`.
        return tuple(res)

set_log = []
class FakeConfigFile:
    def __init__(self, printer):
        self._printer = printer
    def get_printer(self): return self._printer
    def set(self, section, option, value):
        set_log.append((section, option, str(value)))
    def remove_section(self, section):
        set_log.append(("remove_section", section, ""))

gcode_log = []
class FakeGCode:
    error = CommandError
    def __init__(self):
        self.commands = {}
        self.handlers = {}
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        # `gcode.py:register_command`: a second non-None handler raises
        # `printer.config_error`; func None removes the handler.
        if func is None:
            self.handlers.pop(cmd, None)
            self.commands.pop(cmd, None)
            return
        if cmd in self.handlers:
            raise ConfigError("gcode command %s already registered" % (cmd,))
        self.handlers[cmd] = func
        self.commands[cmd] = desc
    def respond_info(self, msg, log=True):
        gcode_log.append(msg)
    def respond_raw(self, msg):
        gcode_log.append(msg)
    def create_gcode_command(self, command, commandline, params):
        return gcode_mod.GCodeCommand(self, command, commandline, params, False)

toollog = []
class FakeStepper:
    def __init__(self, name, pos): self._name = name; self._pos = pos
    def get_name(self): return self._name
    def get_commanded_position(self): return self._pos

class FakeKinematics:
    def __init__(self, cal, steppers, pos):
        self.cal = cal
        self.steppers = steppers
        self.pos = pos
    def get_steppers(self): return self.steppers
    def calc_position(self, spos): return list(self.pos)
    def get_calibration(self): return self.cal

class FakeToolHead:
    def __init__(self, kin): self.kin = kin
    def get_kinematics(self): return self.kin
    def flush_step_generation(self): toollog.append("flush")
    def manual_move(self, coord, speed): toollog.append((coord, speed))

class FakeReactor:
    # Deterministic virtual clock: `pause` advances eventtime by the
    # requested waketime, so `background_coordinate_descent`'s 5 second
    # report can fire; the reference's report count is real-clock
    # dependent under such a fake, so the tests filter that message.
    def __init__(self): self.t = 0.0
    def monotonic(self): return self.t
    def pause(self, waketime):
        self.t = waketime
        return waketime

class FakePrinter:
    config_error = ConfigError
    command_error = CommandError
    def __init__(self):
        self.handlers = {}
        self.objects = {}
        self.reactor = FakeReactor()
    def get_reactor(self): return self.reactor
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def lookup_object(self, name, default=None):
        if name in self.objects: return self.objects[name]
        raise ConfigError("Unknown config object '%s'" % (name,))
    def lookup_objects(self, module=None):
        return list(self.objects.items())

import math as _math
def _sub(a, b): return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]
def _add(a, b): return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
def _mul(a, s): return [a[0]*s, a[1]*s, a[2]*s]
def _dot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
def _magsq(a): return a[0]**2+a[1]**2+a[2]**2
def _cross(a, b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
def _trilat(coords, radius2):
    c1, c2, c3 = coords
    s21 = _sub(c2, c1); s31 = _sub(c3, c1)
    d = _math.sqrt(_magsq(s21))
    ex = _mul(s21, 1./d)
    i = _dot(ex, s31)
    vect_ey = _sub(s31, _mul(ex, i))
    ey = _mul(vect_ey, 1./_math.sqrt(_magsq(vect_ey)))
    ez = _cross(ex, ey)
    j = _dot(ey, s31)
    x = (radius2[0]-radius2[1]+d**2)/(2.*d)
    y = (radius2[0]-radius2[2]-x**2+(x-i)**2+j**2)/(2.*j)
    z = -_math.sqrt(radius2[0]-x**2-y**2)
    return _add(c1, _add(_mul(ex, x), _add(_mul(ey, y), _mul(ez, z))))

BASE = {'radius': 100.0, 'angle_a': 0.0, 'arm_a': 200.0, 'endstop_a': 300.0, 'stepdist_a': 0.01,
        'angle_b': 120.0, 'arm_b': 200.0, 'endstop_b': 300.0, 'stepdist_b': 0.01,
        'angle_c': 240.0, 'arm_c': 200.0, 'endstop_c': 300.0, 'stepdist_c': 0.01}
class FakeCal:
    def __init__(self, params=None):
        self.p = dict(BASE if params is None else params)
    def _deriv(self, p):
        angs = [p['angle_a'], p['angle_b'], p['angle_c']]
        arms = [p['arm_a'], p['arm_b'], p['arm_c']]
        ends = [p['endstop_a'], p['endstop_b'], p['endstop_c']]
        sds = [p['stepdist_a'], p['stepdist_b'], p['stepdist_c']]
        towers = [(_math.cos(_math.radians(a))*p['radius'], _math.sin(_math.radians(a))*p['radius']) for a in angs]
        absends = [e + _math.sqrt(a**2 - p['radius']**2) for e, a in zip(ends, arms)]
        return towers, absends, arms, sds
    def coordinate_descent_params(self, is_extended):
        adj = ('radius', 'angle_a', 'angle_b', 'endstop_a', 'endstop_b', 'endstop_c')
        if is_extended: adj = adj + ('arm_a', 'arm_b', 'arm_c')
        return adj, dict(self.p)
    def new_calibration(self, params):
        return FakeCal(params)
    def calc_stable_position(self, coord):
        towers, absends, arms, sds = self._deriv(self.p)
        sp = [_math.sqrt(a**2 - (t[0]-coord[0])**2 - (t[1]-coord[1])**2) + coord[2] for t, a in zip(towers, arms)]
        return [(ep - s)/sd for sd, ep, s in zip(sds, absends, sp)]
    def get_position_from_stable(self, stable):
        towers, absends, arms, sds = self._deriv(self.p)
        sphere = [(t[0], t[1], es - sp*sd) for sd, t, es, sp in zip(sds, towers, absends, stable)]
        return _trilat(sphere, [a**2 for a in arms])
    def save_state(self, configfile):
        configfile.set('printer', 'delta_radius', "%.6f" % (self.p['radius'],))

log_lines = []
import logging as _logging
class _Cap(_logging.Handler):
    def emit(self, record): log_lines.append(record.getMessage())
_root = _logging.getLogger()
_root.addHandler(_Cap())
_root.setLevel(0)
"#;

fn rv_f64(v: &Rv) -> f64 {
    match v {
        Rv::Float(f) => f.to_f64().expect("float value"),
        other => panic!("expected float, got {other:?}"),
    }
}

fn rv_str(v: &Rv) -> String {
    match v {
        Rv::Str(s) => s.clone(),
        other => panic!("expected str, got {other:?}"),
    }
}

fn assert_equal_value(label: &str, expected: &Rv, actual: &Rv) {
    // Exact: floats by bits (the `repr` spelling of one value differs
    // between CPython, which switches to scientific notation outside
    // 1e-4..1e16, and Rust's `Display`, so only the bits are trusted,
    // as in `recording::assert_equal`); lists and tuples keep their
    // Python kind, elementwise; strings as-is.
    match (expected, actual) {
        (Rv::Float(a), Rv::Float(b)) => {
            let parse = |s: &str| recording::f64_from_hex(s).map(f64::to_bits);
            assert_eq!(
                parse(&a.hex),
                parse(&b.hex),
                "{label}: float mismatch ({} vs {})",
                a.repr,
                b.repr
            );
        }
        (Rv::Str(a), Rv::Str(b)) => assert_eq!(a, b, "{label}"),
        (Rv::List(a), Rv::List(b)) => {
            assert_eq!(a.len(), b.len(), "{label}: len");
            for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_equal_value(&format!("{label}[{i}]"), x, y);
            }
        }
        (Rv::Tuple(a), Rv::Tuple(b)) => {
            assert_eq!(a.len(), b.len(), "{label}: len");
            for (i, (x, y)) in a.iter().zip(b.iter()).enumerate() {
                assert_equal_value(&format!("{label}[{i}]"), x, y);
            }
        }
        (Rv::Null, Rv::Null) => {}
        (Rv::Int(a), Rv::Int(b)) => assert_eq!(a, b, "{label}"),
        _ => panic!("{label}: shape mismatch\nexpected {expected:?}\nactual   {actual:?}"),
    }
}

fn expectation(py: &mut pyoracle::Oracle, label: &str, expr: &str) -> Rv {
    match py.eval(expr) {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("oracle setup");
            Some(py)
        }
        None => {
            println!(
                "skipping delta_calibrate oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Rust fakes (mirror the Python fakes).
// ---------------------------------------------------------------------------

fn base_params() -> IndexMap<String, f64> {
    let mut m = IndexMap::new();
    m.insert("radius".to_string(), 100.0);
    m.insert("angle_a".to_string(), 0.0);
    m.insert("arm_a".to_string(), 200.0);
    m.insert("endstop_a".to_string(), 300.0);
    m.insert("stepdist_a".to_string(), 0.01);
    m.insert("angle_b".to_string(), 120.0);
    m.insert("arm_b".to_string(), 200.0);
    m.insert("endstop_b".to_string(), 300.0);
    m.insert("stepdist_b".to_string(), 0.01);
    m.insert("angle_c".to_string(), 240.0);
    m.insert("arm_c".to_string(), 200.0);
    m.insert("endstop_c".to_string(), 300.0);
    m.insert("stepdist_c".to_string(), 0.01);
    m
}

// --- Printer fake ---

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<IndexMap<String, Vec<klippy_api::printer::EventCallback>>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(IndexMap::new()),
        }
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        let mut objs = self.objects.lock();
        assert!(!objs.contains_key(name), "duplicate {name}");
        objs.insert(name.to_string(), obj);
    }
    fn event_count(&self, event: &str) -> usize {
        self.handlers
            .lock()
            .get(event)
            .map(|v| v.len())
            .unwrap_or(0)
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        ("".to_string(), "startup".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.objects.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::Config(format!(
                "Printer object '{name}' already created"
            )));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        if let Some(obj) = self.objects.lock().get(name).cloned() {
            return Ok(Some(obj));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::Config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        let objects: Vec<(String, ObjectRef)> = {
            let objs = self.objects.lock();
            match module {
                None => objs.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
                Some(module) => {
                    let prefix = format!("{module} ");
                    let mut out = Vec::new();
                    if let Some(v) = objs.get(module) {
                        out.push((module.to_string(), v.clone()));
                    }
                    for (n, o) in objs.iter() {
                        if n.starts_with(&prefix) {
                            out.push((n.clone(), o.clone()));
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
        panic!("load_object not used by delta_calibrate")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.handlers
            .lock()
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut handlers = self.handlers.lock();
        let Some(cbs) = handlers.get_mut(event) else { return Ok(Vec::new()) };
        let mut out = Vec::new();
        for cb in cbs.iter_mut() {
            out.push(cb(params)?);
        }
        Ok(out)
    }
    fn request_exit(&self, _result: &str) {}
}

// --- Config fake (faithful to configfile._get_wrapper) ---

struct FakeConfig {
    printer: Arc<FakePrinter>,
    section: String,
    opts: IndexMap<String, String>,
}

fn cfg_error(section: &str, option: &str, kind: &str) -> KlippyError {
    match kind {
        "missing" => KlippyError::Config(format!(
            "Option '{option}' in section '{section}' must be specified"
        )),
        "parse" => KlippyError::Config(format!(
            "Unable to parse option '{option}' in section '{section}'"
        )),
        _ => unreachable!(),
    }
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v),
            },
        }
    }
    fn getint(
        &self,
        option: &str,
        opts: klippy_api::config::GetIntOpts,
    ) -> KResult<Option<i64>> {
        match self.opts.get(option) {
            Some(raw) => {
                let v = pycompat::text::int_from_string(raw.trim(), 10)
                    .map_err(|_| cfg_error(&self.section, option, "parse"))?;
                let s = v.0.to_string();
                Ok(Some(s.parse::<i64>().unwrap_or(i64::MAX)))
            }
            None => match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v),
            },
        }
    }
    fn getfloat(&self, option: &str, opts: CfgFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v),
            };
        };
        let v = float_from_string(raw).map_err(|_| cfg_error(&self.section, option, "parse"))?;
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_str(minv)
                )));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_str(maxv)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetBoolOpts,
    ) -> KResult<Option<bool>> {
        panic!("unused")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: klippy_api::config::GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("unused")
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v.expect("getlists default")),
            };
        };
        fn leaf_parts(pattern: &str, sep: &str) -> Vec<String> {
            if pattern.trim().is_empty() {
                Vec::new()
            } else {
                pattern.split(sep).map(|p| p.trim().to_string()).collect()
            }
        }
        if opts.seps.len() == 1 {
            let parts = leaf_parts(raw, &opts.seps[0]);
            let mut out = Vec::new();
            for p in &parts {
                let f = float_from_string(p)
                    .map_err(|_| cfg_error(&self.section, option, "parse"))?;
                out.push(PyValue::Float(f));
            }
            if let Some(c) = opts.count {
                if out.len() != c {
                    return Err(KlippyError::Config(format!(
                        "Option '{option}' in section '{}' must have {c} elements",
                        self.section
                    )));
                }
            }
            return Ok(PyValue::Tuple(out));
        }
        let outer = leaf_parts(raw, &opts.seps[opts.seps.len() - 1]);
        let mut rows = Vec::new();
        for line in outer {
            if line.is_empty() {
                continue;
            }
            let inner = leaf_parts(&line, &opts.seps[0]);
            let mut row = Vec::new();
            for p in inner {
                let f = float_from_string(&p)
                    .map_err(|_| cfg_error(&self.section, option, "parse"))?;
                row.push(PyValue::Float(f));
            }
            if let Some(c) = opts.count {
                if row.len() != c {
                    return Err(KlippyError::Config(format!(
                        "Option '{option}' in section '{}' must have {c} elements",
                        self.section
                    )));
                }
            }
            rows.push(PyValue::Tuple(row));
        }
        Ok(PyValue::Tuple(rows))
    }
    fn getlist(
        &self,
        option: &str,
        opts: klippy_api::config::GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v),
            };
        };
        let parts: Vec<String> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(&opts.sep).map(|p| p.trim().to_string()).collect()
        };
        if let Some(c) = opts.count {
            if parts.len() != c {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have {c} elements",
                    self.section
                )));
            }
        }
        Ok(Some(parts))
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: klippy_api::config::GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("unused")
    }
    fn getfloatlist(
        &self,
        option: &str,
        opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                None => Err(cfg_error(&self.section, option, "missing")),
                Some(v) => Ok(v),
            };
        };
        let parts: Vec<String> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(&opts.sep).map(|p| p.trim().to_string()).collect()
        };
        let mut out = Vec::new();
        for p in &parts {
            out.push(float_from_string(p).map_err(|_| cfg_error(&self.section, option, "parse"))?);
        }
        if let Some(c) = opts.count {
            if out.len() != c {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have {c} elements",
                    self.section
                )));
            }
        }
        Ok(Some(out))
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            section: section.to_string(),
            opts: IndexMap::new(),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        section == self.section
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// --- G-code fakes ---

struct FakeGCode {
    commands: Mutex<Vec<(String, String)>>,
    log: Mutex<Vec<String>>,
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            commands: Mutex::new(Vec::new()),
            log: Mutex::new(Vec::new()),
        }
    }
    fn command_names(&self) -> Vec<String> {
        self.commands.lock().iter().map(|(c, _)| c.clone()).collect()
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        // `gcode.py:register_command`: `func` None removes the handler,
        // a second non-None handler raises `printer.config_error`.
        let mut commands = self.commands.lock();
        if func.is_none() {
            commands.retain(|(c, _)| c != cmd);
            return Ok(None);
        }
        if commands.iter().any(|(c, _)| c == cmd) {
            return Err(KlippyError::Config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        commands.push((cmd.to_string(), opts.desc.clone().unwrap_or_default()));
        Ok(None)
    }
    fn register_mux_command(
        &self,
        _cmd: &str,
        _key: &str,
        _value: Option<&str>,
        _func: klippy_api::gcode::GCodeCommandHandler,
        _desc: Option<&str>,
    ) -> KResult<()> {
        panic!("unused")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("unused")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("unused")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("unused")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("unused")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        Arc::new(FakeGCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: params.clone(),
            gcode_log: Mutex::new(Vec::new()),
        })
    }
    fn respond_raw(&self, msg: &str) {
        self.log.lock().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.lock().push(msg.to_string());
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("unused")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("unused")
    }
}

struct FakeGCmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    gcode_log: Mutex<Vec<String>>,
}

fn cmd_error(commandline: &str, msg: String) -> KlippyError {
    KlippyError::Command(format!("Error on '{commandline}': {msg}"))
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(cmd_error(&self.commandline, format!("missing {name}"))),
                Some(v) => Ok(v),
            };
        };
        let value = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int => {
                let n = pycompat::text::int_from_string(raw, 10).map_err(|_| {
                    cmd_error(&self.commandline, format!("unable to parse {raw}"))
                })?;
                PyValue::Int(n)
            }
            GetParser::IntAutoBase => {
                let n = pycompat::text::int_from_string(raw, 0).map_err(|_| {
                    cmd_error(&self.commandline, format!("unable to parse {raw}"))
                })?;
                PyValue::Int(n)
            }
            GetParser::Float => {
                let f = float_from_string(raw).map_err(|_| {
                    cmd_error(&self.commandline, format!("unable to parse {raw}"))
                })?;
                PyValue::Float(f)
            }
        };
        let num = value.to_f64();
        if let (Some(minv), Some(f)) = (&opts.minval, num) {
            if f < minv.to_f64().unwrap_or(f64::NAN) {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have minimum of {}", minv.str()),
                ));
            }
        }
        if let (Some(maxv), Some(f)) = (&opts.maxval, num) {
            if f > maxv.to_f64().unwrap_or(f64::NAN) {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have maximum of {}", maxv.str()),
                ));
            }
        }
        if let (Some(above), Some(f)) = (&opts.above, num) {
            if f <= above.to_f64().unwrap_or(f64::NAN) {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must be above {}", above.str()),
                ));
            }
        }
        if let (Some(below), Some(f)) = (&opts.below, num) {
            if f >= below.to_f64().unwrap_or(f64::NAN) {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must be below {}", below.str()),
                ));
            }
        }
        Ok(value)
    }
    fn get_int(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetIntOpts,
    ) -> KResult<Option<i64>> {
        let raw = match self.params.get(name) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    None => Err(cmd_error(&self.commandline, format!("missing {name}"))),
                    Some(v) => Ok(v),
                }
            }
        };
        let n = pycompat::text::int_from_string(&raw, 10)
            .map_err(|_| cmd_error(&self.commandline, format!("unable to parse {raw}")))?;
        let num = n.0.to_string().parse::<i64>().unwrap_or(i64::MAX);
        if let Some(minv) = opts.minval {
            if num < minv {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have minimum of {minv}"),
                ));
            }
        }
        if let Some(maxv) = opts.maxval {
            if num > maxv {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have maximum of {maxv}"),
                ));
            }
        }
        Ok(Some(num))
    }
    fn get_float(&self, name: &str, opts: GCmdFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.params.get(name) {
            Some(v) => v.clone(),
            None => {
                return match opts.default {
                    None => Err(cmd_error(&self.commandline, format!("missing {name}"))),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = float_from_string(&raw)
            .map_err(|_| cmd_error(&self.commandline, format!("unable to parse {raw}")))?;
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have minimum of {}", float_str(minv)),
                ));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must have maximum of {}", float_str(maxv)),
                ));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must be above {}", float_str(above)),
                ));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(cmd_error(
                    &self.commandline,
                    format!("{name} must be below {}", float_str(below)),
                ));
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.gcode_log.lock().push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.gcode_log.lock().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

// --- Toolhead / kinematics fakes ---

struct FakeToolHead {
    kin: Arc<FakeKinematics>,
    moves: Mutex<Vec<(Vec<Option<f64>>, f64)>>,
    flush_log: Mutex<Vec<String>>,
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("unused")
    }
    fn get_kinematics(&self) -> Kinematics {
        self.kin.clone()
    }
    fn flush_step_generation(&self) -> KResult<()> {
        self.flush_log.lock().push("flush".to_string());
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
        panic!("unused")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("unused")
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.moves.lock().push((coord, speed));
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("unused")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("unused")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("unused")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("unused")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("unused")
    }
}

struct FakeKinematics {
    cal: Option<Arc<FakeCalibration>>,
    steppers: Vec<Arc<dyn McuStepperApi>>,
    pos: Vec<Option<f64>>,
}

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Arc<dyn McuStepperApi>> {
        self.steppers.clone()
    }
    fn calc_position(
        &self,
        _stepper_positions: &IndexMap<String, f64>,
    ) -> KResult<Vec<Option<f64>>> {
        Ok(self.pos.clone())
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("unused")
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        panic!("unused")
    }
    fn home(&self, _homing_state: klippy_api::toolhead::HomingStateHandle) -> KResult<()> {
        panic!("unused")
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn clear_homing_state(&self, _clear_axes: &str) -> klippy_api::KResult<()> { Ok(()) }
    fn get_calibration(&self) -> KResult<DeltaCalibration> {
        match &self.cal {
            Some(c) => {
                let cal: DeltaCalibration = c.clone();
                Ok(cal)
            }
            None => Err(no_kinematics_attr(self.class_name(), "get_calibration")),
        }
    }
}

struct FakeCalibration {
    params: IndexMap<String, f64>,
}

/// Python `math.sqrt`: a negative argument raises
/// `ValueError: math domain error` (`math.sqrt`, unlike `libm::sqrt`).
fn py_sqrt(x: f64) -> KResult<f64> {
    if x < 0.0 {
        return Err(KlippyError::Other {
            class_name: "ValueError".to_string(),
            message: "math domain error".to_string(),
        });
    }
    Ok(pycompat::libm::sqrt(x))
}

/// The delta geometry the fake derives from its params.
type Rig = ([(f64, f64); 3], [f64; 3], [f64; 3], [f64; 3]);

impl FakeCalibration {
    // Faithful port of `klippy/kinematics/delta.py:169`
    // (`calc_stable_position` / `get_position_from_stable`), written here
    // as a fake: the Rust fake must compute independently of the port
    // under test, just like the Python `FakeCal` in SETUP mirrors the
    // reference through its own math.
    fn rig(&self) -> KResult<Rig> {
        use pycompat::libm;
        let p = &self.params;
        let radius = p["radius"];
        let mut towers = [(0.0, 0.0); 3];
        let mut abs_ends = [0.0; 3];
        let mut arms = [0.0; 3];
        let mut stepdists = [0.0; 3];
        for (i, ax) in ['a', 'b', 'c'].iter().enumerate() {
            let rad = p[&format!("angle_{ax}")]
                * (std::f64::consts::PI / 180.0);
            towers[i] = (libm::cos(rad) * radius, libm::sin(rad) * radius);
            arms[i] = p[&format!("arm_{ax}")];
            abs_ends[i] = p[&format!("endstop_{ax}")]
                + py_sqrt(libm::pow(arms[i], 2.0) - libm::pow(radius, 2.0))?;
            stepdists[i] = p[&format!("stepdist_{ax}")];
        }
        Ok((towers, abs_ends, arms, stepdists))
    }
}

impl DeltaCalibrationApi for FakeCalibration {
    fn coordinate_descent_params(&self, is_extended: bool) -> PyValue {
        let mut adj = vec![
            "radius",
            "angle_a",
            "angle_b",
            "endstop_a",
            "endstop_b",
            "endstop_c",
        ];
        if is_extended {
            adj.push("arm_a");
            adj.push("arm_b");
            adj.push("arm_c");
        }
        let adj_v: Vec<PyValue> = adj.iter().map(|s| PyValue::Str(s.to_string())).collect();
        let mut dict = indexmap::IndexMap::new();
        for (k, v) in self.params.iter() {
            dict.insert(PyValue::Str(k.clone()), PyValue::Float(*v));
        }
        PyValue::Tuple(vec![PyValue::Tuple(adj_v), PyValue::Dict(dict)])
    }
    fn new_calibration(&self, params: &PyValue) -> KResult<DeltaCalibration> {
        let PyValue::Dict(dict) = params else {
            panic!("new_calibration params must be dict");
        };
        let mut map = IndexMap::new();
        for (k, v) in dict {
            let PyValue::Str(name) = k else { panic!("key") };
            map.insert(name.clone(), v.to_f64().expect("float"));
        }
        Ok(Arc::new(FakeCalibration { params: map }))
    }
    fn get_position_from_stable(&self, stable_position: &[f64]) -> KResult<Vec<f64>> {
        use pycompat::libm;
        let (towers, abs_ends, arms, stepdists) = self.rig()?;
        let sphere: [klippy_core::mathutil::Vector3; 3] = [
            [
                towers[0].0,
                towers[0].1,
                abs_ends[0] - stable_position[0] * stepdists[0],
            ],
            [
                towers[1].0,
                towers[1].1,
                abs_ends[1] - stable_position[1] * stepdists[1],
            ],
            [
                towers[2].0,
                towers[2].1,
                abs_ends[2] - stable_position[2] * stepdists[2],
            ],
        ];
        let radius2: klippy_core::mathutil::Vector3 =
            [libm::pow(arms[0], 2.0), libm::pow(arms[1], 2.0), libm::pow(arms[2], 2.0)];
        let p = klippy_core::mathutil::trilateration(&sphere, &radius2)?;
        Ok(vec![p[0], p[1], p[2]])
    }
    fn calc_stable_position(&self, coord: &[f64]) -> KResult<Vec<f64>> {
        use pycompat::libm;
        let (towers, abs_ends, arms, stepdists) = self.rig()?;
        let mut out = Vec::new();
        for k in 0..3 {
            let dx = towers[k].0 - coord[0];
            let dy = towers[k].1 - coord[1];
            let sp = py_sqrt(libm::pow(arms[k], 2.0) - libm::pow(dx, 2.0) - libm::pow(dy, 2.0))?
                + coord[2];
            out.push((abs_ends[k] - sp) / stepdists[k]);
        }
        Ok(out)
    }
    fn save_state(&self, configfile: &PrinterConfig) -> KResult<()> {
        let radius = self.params["radius"];
        let value = percent_format("%.6f", &[PctValue::Float(radius)]).map_err(|e| {
            KlippyError::Other { class_name: e.kind.to_string(), message: e.msg }
        })?;
        configfile.set("printer", "delta_radius", &value).expect("set");
        Ok(())
    }
}

// --- configfile fake ---

struct FakeConfigFile {
    /// `(section, option, value)` in call order; `remove_section` is
    /// recorded as `("remove_section", section, "")`, mirroring the
    /// Python fake's single log.
    calls: Mutex<Vec<(String, String, String)>>,
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

impl FakeConfigFile {
    fn new() -> FakeConfigFile {
        FakeConfigFile { calls: Mutex::new(Vec::new()) }
    }
    fn calls(&self) -> Vec<(String, String, String)> {
        self.calls.lock().clone()
    }
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("unused")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("unused")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("unused")
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("unused")
    }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(
        &self,
        _section: &str,
        _option: &str,
        _value: Option<PyValue>,
        _msg: Option<String>,
    ) {
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(
        &self,
        _mcu: &klippy_api::mcu::Mcu,
        _feature: &str,
        _msg: Option<String>,
    ) {
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.calls
            .lock()
            .push((section.to_string(), option.to_string(), value.to_string()));
        Ok(())
    }
    fn remove_section(&self, section: &str) {
        self.calls
            .lock()
            .push(("remove_section".to_string(), section.to_string(), String::new()));
    }
}

// --- world builders ---

fn make_config(printer: Arc<FakePrinter>, opts: Vec<(&str, &str)>) -> Config {
    let mut map = IndexMap::new();
    for (k, v) in opts {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeConfig {
        printer,
        section: "delta_calibrate".to_string(),
        opts: map,
    }) as Config
}

/// Handles on the fakes registered in a Rust world.
struct Fakes {
    printer: Arc<FakePrinter>,
    configfile: Arc<FakeConfigFile>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolHead>,
}

fn make_printer_with_endpoints(
    cal: Option<Arc<FakeCalibration>>,
    pos: Vec<Option<f64>>,
) -> Fakes {
    let printer = Arc::new(FakePrinter::new());
    let kin = Arc::new(FakeKinematics {
        cal,
        steppers: vec![],
        pos,
    });
    let toolhead = Arc::new(FakeToolHead {
        kin,
        moves: Mutex::new(Vec::new()),
        flush_log: Mutex::new(Vec::new()),
    });
    let gcode = Arc::new(FakeGCode::new());
    let configfile = Arc::new(FakeConfigFile::new());
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("toolhead", toolhead.clone() as ObjectRef);
    printer.add("configfile", configfile.clone() as ObjectRef);
    Fakes { printer, configfile, gcode, toolhead }
}

/// A built `DeltaCalibrate` plus the fakes it talks to.
struct World {
    fakes: Fakes,
    cal: Arc<DeltaCalibrate>,
}

/// Build a matched Rust world + `DeltaCalibrate` for given config opts.
fn world_with(
    cal_params: Option<IndexMap<String, f64>>,
    pos: Vec<Option<f64>>,
    opts: Vec<(&str, &str)>,
) -> World {
    let fakes = make_printer_with_endpoints(
        cal_params.map(|params| Arc::new(FakeCalibration { params })),
        pos,
    );
    let cfg = make_config(fakes.printer.clone(), opts);
    let cal = DeltaCalibrate::new(&cfg).expect("DeltaCalibrate::new");
    World { fakes, cal }
}

/// Build a world whose kinematics restore the base delta parameters.
fn rust_cal(opts: Vec<(&str, &str)>) -> World {
    world_with(Some(base_params()), vec![], opts)
}

/// A `ProbeResult` in the Python field order
/// (`manual_probe.ProbeResult`: bed_x, bed_y, bed_z, test_x, test_y, test_z).
fn probe_result(bed: [f64; 3], test: [f64; 3]) -> ProbeResult {
    ProbeResult {
        bed_x: bed[0],
        bed_y: bed[1],
        bed_z: bed[2],
        test_x: test[0],
        test_y: test[1],
        test_z: test[2],
    }
}

fn err_rv(e: &KlippyError) -> Rv {
    Rv::Tuple(vec![
        Rv::Str(e.class_name().to_string()),
        Rv::Str(e.message().to_string()),
    ])
}

fn gcmd(command: &str, commandline: &str, params: Vec<(&str, &str)>) -> GCmd {
    let mut map = IndexMap::new();
    for (k, v) in params {
        map.insert(k.to_string(), v.to_string());
    }
    let inner: GCmd = Arc::new(FakeGCmd {
        command: command.to_string(),
        commandline: commandline.to_string(),
        params: map,
        gcode_log: Mutex::new(Vec::new()),
    });
    inner
}

// Log capture (lines kept per test thread; the Rust sink is process wide).
thread_local! {
    static LOGS: std::cell::RefCell<Vec<String>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static LOG_ONCE: std::sync::Once = std::sync::Once::new();
    LOG_ONCE.call_once(|| {
        log::set_sink(Box::new(|rec: &log::Record<'_>| {
            LOGS.with(|logs| logs.borrow_mut().push(rec.message.to_string()));
        }));
    });
}

/// Drain the captured log messages of this test's thread.
fn take_log_messages() -> Vec<String> {
    install_log_sink();
    LOGS.with(|logs| std::mem::take(&mut *logs.borrow_mut()))
}

fn rv_points(points: &[(f64, f64)]) -> Rv {
    Rv::List(
        points
            .iter()
            .map(|(x, y)| Rv::Tuple(vec![Rv::float(*x), Rv::float(*y)]))
            .collect(),
    )
}

fn rv_spos(spos: &StablePosition) -> Rv {
    let items = spos.values().iter().map(|v| Rv::float(*v)).collect::<Vec<_>>();
    match spos {
        StablePosition::List(_) => Rv::List(items),
        StablePosition::Tuple(_) => Rv::Tuple(items),
    }
}

fn rv_heights(entries: &[(f64, StablePosition)]) -> Rv {
    Rv::List(
        entries
            .iter()
            .map(|(z, s)| Rv::Tuple(vec![Rv::float(*z), rv_spos(s)]))
            .collect(),
    )
}

fn rv_distances(entries: &[(f64, StablePosition, StablePosition)]) -> Rv {
    Rv::List(
        entries
            .iter()
            .map(|(d, a, b)| Rv::Tuple(vec![Rv::float(*d), rv_spos(a), rv_spos(b)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn init_defaults_registrations_and_restore() {
    let Some(mut py) = oracle() else { return };
    let opts = vec![
        ("radius", "140.5"),
        ("height0", "31.25"),
        ("height0_pos", "1.5,2.5,3.5"),
        ("manual_height0", "40.0"),
        ("manual_height0_pos", "4.5,5.5,6.5"),
        ("distance0", "77.5"),
        ("distance0_pos1", "1.0,2.0,3.0"),
        ("distance0_pos2", "4.0,5.0,6.0"),
    ];
    // --- Python expected ---
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5', 'height0': '31.25', 'height0_pos': '1.5,2.5,3.5', 'manual_height0': '40.0', 'manual_height0_pos': '4.5,5.5,6.5', 'distance0': '77.5', 'distance0_pos1': '1.0,2.0,3.0', 'distance0_pos2': '4.0,5.0,6.0'})\ncal = delta_calibrate.DeltaCalibrate(config)\n",
    )
    .expect("py build");
    let py_points = expectation(&mut py, "py_points", "list(probe_calls[0].default_points)");
    let py_probe_positions =
        expectation(&mut py, "py_probe_positions", "list(cal.last_probe_positions)");
    let py_manual = expectation(&mut py, "py_manual", "list(cal.manual_heights)");
    let py_distances = expectation(&mut py, "py_distances", "list(cal.last_distances)");
    let py_commands = expectation(&mut py, "py_commands", "sorted(gcode.commands.items())");

    // --- Rust actual ---
    let world = rust_cal(opts);
    let cal = &world.cal;
    assert_equal_value("probe_points", &py_points, &rv_points(&cal.probe_points()));
    assert_equal_value(
        "probe_positions",
        &py_probe_positions,
        &rv_heights(&cal.last_probe_positions()),
    );
    assert_equal_value("manual_heights", &py_manual, &rv_heights(&cal.manual_heights()));
    assert_equal_value("last_distances", &py_distances, &rv_distances(&cal.last_distances()));
    let mut rust_cmds = world.fakes.gcode.commands.lock().clone();
    rust_cmds.sort();
    let rust_cmds_rv = Rv::List(
        rust_cmds
            .iter()
            .map(|(c, d)| Rv::Tuple(vec![Rv::Str(c.clone()), Rv::Str(d.clone())]))
            .collect(),
    );
    assert_equal_value("commands", &py_commands, &rust_cmds_rv);
    assert!(world.fakes.printer.event_count("klippy:connect") >= 1);
    assert_eq!(cal.delta_analyze_entry().len(), 1);
}

#[test]
fn radius_required_and_above_zero() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {})\ntry:\n    cal = delta_calibrate.DeltaCalibrate(config)\n    _init_out = ('ok',)\nexcept Exception as e:\n    _init_out = (type(e).__name__, str(e))\n",
    )
    .expect("py missing");
    let py_missing = expectation(&mut py, "py_missing", "_init_out");
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '0'})\ntry:\n    cal = delta_calibrate.DeltaCalibrate(config)\n    _init_out = ('ok',)\nexcept Exception as e:\n    _init_out = (type(e).__name__, str(e))\n",
    )
    .expect("py zero");
    let py_zero = expectation(&mut py, "py_zero", "_init_out");

    let fakes = make_printer_with_endpoints(None, vec![]);
    let cfg_missing = make_config(fakes.printer.clone(), vec![]);
    let err_missing = DeltaCalibrate::new(&cfg_missing).err().expect("missing radius");
    let cfg_zero = make_config(fakes.printer.clone(), vec![("radius", "0")]);
    let err_zero = DeltaCalibrate::new(&cfg_zero).err().expect("zero radius");
    assert_equal_value("missing", &py_missing, &err_rv(&err_missing));
    assert_equal_value("zero", &py_zero, &err_rv(&err_zero));
}

#[test]
fn measurements_to_distances_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "mp = {'SCALE': [0.5], 'CENTER_DISTS': [40.0, 41.0, 42.0, 43.0, 44.0, 45.0], 'CENTER_PILLAR_WIDTHS': [1.0, 2.0, 3.0], 'OUTER_DISTS': [90.0, 91.0, 92.0, 93.0, 94.0, 95.0], 'OUTER_PILLAR_WIDTHS': [4.0, 5.0, 6.0, 7.0, 8.0, 9.0]}",
    )
    .expect("mp");
    let py_out = expectation(
        &mut py,
        "py_m2d",
        "delta_calibrate.measurements_to_distances(mp, FakeCal())",
    );
    let mut mp = indexmap::IndexMap::new();
    // The Rust side drives the same unit through the frozen contract:
    // `measurements_to_distances` takes the calibration trait object.
    let cal: DeltaCalibration = Arc::new(FakeCalibration {
        params: base_params(),
    });
    // SCALE scales the calibration object; at 2.0 the outer positions
    // fall outside the fake's arm reach (radius 100, arm 200) and both
    // sides raise `math.sqrt` domain errors. 0.5 keeps every position
    // inside the geometry, matching real calibration objects.
    mp.insert("SCALE".to_string(), vec![0.5]);
    mp.insert(
        "CENTER_DISTS".to_string(),
        vec![40.0, 41.0, 42.0, 43.0, 44.0, 45.0],
    );
    mp.insert("CENTER_PILLAR_WIDTHS".to_string(), vec![1.0, 2.0, 3.0]);
    mp.insert(
        "OUTER_DISTS".to_string(),
        vec![90.0, 91.0, 92.0, 93.0, 94.0, 95.0],
    );
    mp.insert(
        "OUTER_PILLAR_WIDTHS".to_string(),
        vec![4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
    );
    let rust_out = measurements_to_distances(&mp, &cal).expect("m2d");
    assert_equal_value("m2d", &py_out, &rv_distances(&rust_out));
}

/// Error paths of `measurements_to_distances` (`delta_calibrate.py:35`).
/// The Python expression order matters: the six-entry `center_widths`
/// list is built before `CENTER_DISTS` is read, so a short
/// `CENTER_PILLAR_WIDTHS` raises `IndexError`, not the later `KeyError`.
#[test]
fn measurements_to_distances_error_paths() {
    let Some(mut py) = oracle() else { return };
    let cal: DeltaCalibration = Arc::new(FakeCalibration { params: base_params() });

    // Short CENTER_PILLAR_WIDTHS with no CENTER_DISTS at all.
    py.exec("mp_short = {'SCALE': [1.0], 'CENTER_PILLAR_WIDTHS': [1.0, 2.0]}\ntry:\n    delta_calibrate.measurements_to_distances(mp_short, FakeCal())\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n")
        .expect("py short");
    let py_short = expectation(&mut py, "py_short", "_e");
    let mut mp = IndexMap::new();
    mp.insert("SCALE".to_string(), vec![1.0]);
    mp.insert("CENTER_PILLAR_WIDTHS".to_string(), vec![1.0, 2.0]);
    let err = measurements_to_distances(&mp, &cal).unwrap_err();
    assert_equal_value("short widths", &py_short, &err_rv(&err));

    // Missing SCALE (read before everything else).
    py.exec("mp_no_scale = {'CENTER_PILLAR_WIDTHS': [1.0, 2.0, 3.0]}\ntry:\n    delta_calibrate.measurements_to_distances(mp_no_scale, FakeCal())\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n")
        .expect("py no scale");
    let py_no_scale = expectation(&mut py, "py_no_scale", "_e");
    let mut mp = IndexMap::new();
    mp.insert("CENTER_PILLAR_WIDTHS".to_string(), vec![1.0, 2.0, 3.0]);
    let err = measurements_to_distances(&mp, &cal).unwrap_err();
    assert_equal_value("no scale", &py_no_scale, &err_rv(&err));

    // Empty SCALE list (index 0 out of range).
    py.exec("mp_empty_scale = {'SCALE': []}\ntry:\n    delta_calibrate.measurements_to_distances(mp_empty_scale, FakeCal())\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n")
        .expect("py empty scale");
    let py_empty = expectation(&mut py, "py_empty_scale", "_e");
    let mut mp = IndexMap::new();
    mp.insert("SCALE".to_string(), vec![]);
    let err = measurements_to_distances(&mp, &cal).unwrap_err();
    assert_equal_value("empty scale", &py_empty, &err_rv(&err));
}

#[test]
fn delta_analyze_parsing_logs_and_errors() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\ndef _gcmd(params):\n    return gcode_mod.GCodeCommand(gcode, 'DELTA_ANALYZE', 'DELTA_ANALYZE ' + ' '.join('%s=%s' % kv for kv in params.items()), params, False)\nlog_lines.clear()\n")
    .expect("build");
    py.exec("cal.cmd_DELTA_ANALYZE(_gcmd({'SCALE': '2.5'}))")
        .expect("valid scale");
    let py_entry = expectation(&mut py, "py_entry", "cal.delta_analyze_entry");
    let py_logs = expectation(&mut py, "py_logs", "list(log_lines)");
    py.exec(
        "log_lines.clear()\ntry:\n    cal.cmd_DELTA_ANALYZE(_gcmd({'CENTER_DISTS': 'a,b'}))\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n",
    )
    .expect("bad");
    let py_bad = expectation(&mut py, "py_bad", "_e");
    py.exec(
        "try:\n    cal.cmd_DELTA_ANALYZE(_gcmd({'CENTER_DISTS': '1,2,3'}))\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n",
    )
    .expect("count");
    let py_count = expectation(&mut py, "py_count", "_e");
    py.exec(
        "try:\n    cal.cmd_DELTA_ANALYZE(_gcmd({'CALIBRATE': 'bogus'}))\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n",
    )
    .expect("action");
    let py_action = expectation(&mut py, "py_action", "_e");

    let world = rust_cal(vec![("radius", "140.5")]);
    let cal = &world.cal;
    // Drain the construction lines first.
    take_log_messages();
    cal.cmd_delta_analyze(&gcmd(
        "DELTA_ANALYZE",
        "DELTA_ANALYZE SCALE=2.5",
        vec![("SCALE", "2.5")],
    ))
    .expect("rust scale");
    let py_map: Vec<(String, Vec<f64>)> = match py_entry {
        Rv::Dict(items) => items
            .into_iter()
            .map(|(k, v)| {
                let Rv::List(vals) = v else { panic!("{v:?}") };
                (rv_str(&k), vals.iter().map(rv_f64).collect())
            })
            .collect(),
        other => panic!("{other:?}"),
    };
    let rust_entry = cal.delta_analyze_entry();
    assert_eq!(rust_entry.len(), py_map.len());
    for (k, v) in py_map {
        let rust_v = rust_entry.get(&k).expect("entry key");
        assert_eq!(rust_v.len(), v.len());
        for (a, b) in rust_v.iter().zip(v.iter()) {
            assert_eq!(a.to_bits(), b.to_bits(), "{k}: {a} vs {b}");
        }
    }
    let err1 = cal
        .cmd_delta_analyze(&gcmd(
            "DELTA_ANALYZE",
            "DELTA_ANALYZE CENTER_DISTS=a,b",
            vec![("CENTER_DISTS", "a,b")],
        ))
        .unwrap_err();
    let err2 = cal
        .cmd_delta_analyze(&gcmd(
            "DELTA_ANALYZE",
            "DELTA_ANALYZE CENTER_DISTS=1,2,3",
            vec![("CENTER_DISTS", "1,2,3")],
        ))
        .unwrap_err();
    let err3 = cal
        .cmd_delta_analyze(&gcmd(
            "DELTA_ANALYZE",
            "DELTA_ANALYZE CALIBRATE=bogus",
            vec![("CALIBRATE", "bogus")],
        ))
        .unwrap_err();
    assert_equal_value("bad", &py_bad, &err_rv(&err1));
    assert_equal_value("count", &py_count, &err_rv(&err2));
    assert_equal_value("action", &py_action, &err_rv(&err3));
    let py_strs: Vec<String> = match py_logs {
        Rv::List(items) => items.iter().map(rv_str).collect(),
        other => panic!("{other:?}"),
    };
    let rust_lines: Vec<String> = take_log_messages()
        .into_iter()
        .filter(|m| m.starts_with("DELTA_ANALYZE"))
        .collect();
    assert_eq!(py_strs, rust_lines);
}

#[test]
fn extended_calibration_guards() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\ncal.delta_analyze_entry.update({'CENTER_DISTS': [1.0]*6, 'CENTER_PILLAR_WIDTHS': [1.0]*3})\ntry:\n    cal.do_extended_calibration()\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n")
    .expect("py partial");
    let py_partial = expectation(&mut py, "py_partial", "_e");
    let world = rust_cal(vec![("radius", "140.5")]);
    let cal = &world.cal;
    cal.cmd_delta_analyze(&gcmd(
        "DELTA_ANALYZE",
        "DELTA_ANALYZE CENTER_DISTS=1,1,1,1,1,1 CENTER_PILLAR_WIDTHS=1,1,1",
        vec![
            ("CENTER_DISTS", "1,1,1,1,1,1"),
            ("CENTER_PILLAR_WIDTHS", "1,1,1"),
        ],
    ))
    .expect("ingest");
    let err = cal.do_extended_calibration().unwrap_err();
    assert_equal_value("partial", &py_partial, &err_rv(&err));

    // With no measurements entered, `last_distances` is used, and the
    // empty `last_probe_positions` is reported instead.
    py.exec("printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\ntry:\n    cal.do_extended_calibration()\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\n")
    .expect("py no probe");
    let py_no_probe = expectation(&mut py, "py_no_probe", "_e");
    let world = rust_cal(vec![("radius", "140.5")]);
    let err = world.cal.do_extended_calibration().unwrap_err();
    assert_equal_value("no probe", &py_no_probe, &err_rv(&err));
}

#[test]
fn handle_connect_rejects_non_delta() {
    let Some(_py) = oracle() else { return };
    let fakes = make_printer_with_endpoints(None, vec![]);
    let cfg = make_config(fakes.printer.clone(), vec![("radius", "140.5")]);
    let cal = DeltaCalibrate::new(&cfg).expect("new");
    let err = cal.handle_connect().unwrap_err();
    assert_eq!(err.message(), "Delta calibrate is only for delta printers");
    assert_eq!(err.class_name(), "Error");
}

#[test]
fn probe_helper_points_and_minimum() {
    let Some(_py) = oracle() else { return };
    let fakes = make_printer_with_endpoints(None, vec![]);
    let cfg = make_config(
        fakes.printer.clone(),
        vec![("radius", "100"), ("points", "10,20\n30,40"), ("speed", "60")],
    );
    // The helper lives in the `probe` port (`extras::probe`); the points
    // option replaces `default_points` (`probe.py:426`), and the
    // finalize callback's value means "retry" when it is `"retry"`.
    let defaults = vec![vec![0.0, 0.0], vec![1.0, 2.0], vec![3.0, 4.0]];
    let helper = ProbePointsHelper::new(&cfg, Box::new(|_| Ok(PyValue::None)), Some(defaults))
        .expect("helper");
    assert_eq!(
        helper.probe_points(),
        vec![vec![10.0, 20.0], vec![30.0, 40.0]]
    );
    assert!(helper.minimum_points(3).is_err());
}

/// `save_state` (`delta_calibrate.py:132`): every `configfile.set` /
/// `remove_section` call, in order, with the Python `str()` / `%.3f`
/// spellings of the numbers.
#[test]
fn save_state_writes_config() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5', 'manual_height0': '40.0', 'manual_height0_pos': '4.5,5.5,6.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\nset_log.clear()\ncal.save_state([(31.25, (10.5, -20.25, 30.125)), (-0.5, (0.001, 2.0, -3.0))], [(77.5, (1.0, 2.0, 3.0), (4.0, 5.0, 6.0))], FakeCal())\n",
    )
    .expect("py save_state");
    let py_calls = expectation(&mut py, "py_calls", "list(set_log)");

    let world = rust_cal(vec![
        ("radius", "140.5"),
        ("manual_height0", "40.0"),
        ("manual_height0_pos", "4.5,5.5,6.5"),
    ]);
    let delta_params: DeltaCalibration = Arc::new(FakeCalibration { params: base_params() });
    world
        .cal
        .save_state(
            &[
                (31.25, StablePosition::List([10.5, -20.25, 30.125])),
                (-0.5, StablePosition::List([0.001, 2.0, -3.0])),
            ],
            &[(
                77.5,
                StablePosition::List([1.0, 2.0, 3.0]),
                StablePosition::List([4.0, 5.0, 6.0]),
            )],
            &delta_params,
        )
        .expect("save_state");
    let rust_calls = Rv::List(
        world
            .fakes
            .configfile
            .calls()
            .into_iter()
            .map(|(s, o, v)| Rv::Tuple(vec![Rv::str(&s), Rv::str(&o), Rv::str(&v)]))
            .collect(),
    );
    assert_equal_value("save_state calls", &py_calls, &rust_calls);
}

/// `DELTA_ANALYZE MANUAL_HEIGHT=` (`delta_calibrate.py:224`): the flush,
/// the current cartesian position from the kinematics, the stable
/// position conversion, the response line and the stored entry.
#[test]
fn add_manual_height_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], [12.5, -7.25, 3.125]))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\ndef _gcmd(params):\n    return gcode_mod.GCodeCommand(gcode, 'DELTA_ANALYZE', 'DELTA_ANALYZE ' + ' '.join('%s=%s' % kv for kv in params.items()), params, False)\ntoollog.clear()\ngcode_log.clear()\ncal.cmd_DELTA_ANALYZE(_gcmd({'MANUAL_HEIGHT': '0.75'}))\n_out = (list(toollog), list(gcode_log), list(cal.manual_heights))\n",
    )
    .expect("py manual height");
    let py_out = expectation(&mut py, "py_manual_height", "_out");

    let world = world_with(
        Some(base_params()),
        vec![Some(12.5), Some(-7.25), Some(3.125)],
        vec![("radius", "140.5")],
    );
    world
        .cal
        .cmd_delta_analyze(&gcmd(
            "DELTA_ANALYZE",
            "DELTA_ANALYZE MANUAL_HEIGHT=0.75",
            vec![("MANUAL_HEIGHT", "0.75")],
        ))
        .expect("manual height");
    let rust_out = Rv::Tuple(vec![
        Rv::List(world.fakes.toolhead.flush_log.lock().iter().map(|s| Rv::str(s)).collect()),
        Rv::List(world.fakes.gcode.log.lock().iter().map(|s| Rv::str(s)).collect()),
        rv_heights(&world.cal.manual_heights()),
    ]);
    assert_equal_value("manual height", &py_out, &rust_out);
}

/// Full calibration run (`delta_calibrate.py:155` `probe_finalize`):
/// probe z offsets and restored distance measurements are converted to
/// stable positions, the coordinate descent runs in the background, the
/// four log formats are emitted, the config file is updated and the
/// `SAVE_CONFIG` hint is printed.
#[test]
fn probe_finalize_runs_calibration() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5', 'distance0': '13.2', 'distance0_pos1': '47320,47320,47320', 'distance0_pos2': '47320,47320,46000'})\ncal = delta_calibrate.DeltaCalibrate(config)\nresults = [manual_probe.ProbeResult(0., 0., -0.2, 0., 0., -0.1), manual_probe.ProbeResult(60., 0., -0.17, 60., 0., -0.05), manual_probe.ProbeResult(0., 60., -0.16, 0., 60., -0.08)]\nlog_lines.clear()\ngcode_log.clear()\nset_log.clear()\ncal.probe_finalize(results)\n_out = (list(log_lines), list(set_log), [m for m in gcode_log if m != 'Working on calibration...'])\n",
    )
    .expect("py probe_finalize");
    let py_out = expectation(&mut py, "py_probe_finalize", "_out");

    let world = rust_cal(vec![
        ("radius", "140.5"),
        ("distance0", "13.2"),
        ("distance0_pos1", "47320,47320,47320"),
        ("distance0_pos2", "47320,47320,46000"),
    ]);
    take_log_messages();
    world
        .cal
        .probe_finalize(&[
            probe_result([0.0, 0.0, -0.2], [0.0, 0.0, -0.1]),
            probe_result([60.0, 0.0, -0.17], [60.0, 0.0, -0.05]),
            probe_result([0.0, 60.0, -0.16], [0.0, 60.0, -0.08]),
        ])
        .expect("probe_finalize");
    // The captured sink also carries `mathutil`'s two descent lines
    // (Python emits them from the forked child, so they never reach the
    // parent's log list); keep only the four formats this module owns.
    let rust_logs: Vec<Rv> = take_log_messages()
        .into_iter()
        .filter(|m| {
            m.starts_with("Calculating delta_calibrate")
                || m.starts_with("Calculated delta_calibrate")
                || m.starts_with("height orig:")
                || m.starts_with("distance orig:")
        })
        .map(Rv::Str)
        .collect();
    let rust_calls: Vec<Rv> = world
        .fakes
        .configfile
        .calls()
        .into_iter()
        .map(|(s, o, v)| Rv::Tuple(vec![Rv::str(&s), Rv::str(&o), Rv::str(&v)]))
        .collect();
    let rust_infos: Vec<Rv> = world
        .fakes
        .gcode
        .log
        .lock()
        .iter()
        .filter(|m| *m != "Working on calibration...")
        .map(|m| Rv::str(m))
        .collect();
    let rust_out = Rv::Tuple(vec![
        Rv::List(rust_logs),
        Rv::List(rust_calls),
        Rv::List(rust_infos),
    ]);
    assert_equal_value("probe_finalize", &py_out, &rust_out);
}

/// `cmd_DELTA_CALIBRATE` (`delta_calibrate.py:222`) only delegates to
/// `ProbePointsHelper.start_probe(gcmd)`. With a manual probe already in
/// progress, `manual_probe.verify_no_manual_probe` raises before anything
/// else runs, so both sides must fail with the same `CommandError` —
/// which only happens if the command reached the real helper.
#[test]
fn delta_calibrate_command_delegates_to_probe_helper() {
    let Some(mut py) = oracle() else { return };
    py.exec("probe.ProbePointsHelper = _RealHelper\nprinter = FakePrinter()\ngcode = FakeGCode()\nconfigfile = FakeConfigFile(printer)\ntoolhead = FakeToolHead(FakeKinematics(FakeCal(), [], []))\nprinter.objects['gcode'] = gcode\nprinter.objects['toolhead'] = toolhead\nprinter.objects['configfile'] = configfile\ngcode.register_command('ACCEPT', 'already probing')\nconfig = FakeConfig(printer, 'delta_calibrate', {'radius': '140.5'})\ncal = delta_calibrate.DeltaCalibrate(config)\ntry:\n    cal.cmd_DELTA_CALIBRATE(gcode_mod.GCodeCommand(gcode, 'DELTA_CALIBRATE', 'DELTA_CALIBRATE', {}, False))\n    _e = ('ok',)\nexcept Exception as e:\n    _e = (type(e).__name__, str(e))\nprobe.ProbePointsHelper = FakeHelper\n")
    .expect("py cmd_DELTA_CALIBRATE");
    let py_out = expectation(&mut py, "py_calibrate_cmd", "_e");

    let world = rust_cal(vec![("radius", "140.5")]);
    world
        .fakes
        .gcode
        .register_command(
            "ACCEPT",
            Some(Box::new(|_gcmd: &GCmd| Ok(()))),
            RegisterCommandOpts { when_not_ready: false, desc: None },
        )
        .expect("seed ACCEPT");
    let err = world
        .cal
        .cmd_delta_calibrate(&gcmd("DELTA_CALIBRATE", "DELTA_CALIBRATE", vec![]))
        .unwrap_err();
    assert_equal_value("cmd_DELTA_CALIBRATE", &py_out, &err_rv(&err));
}

/// `load_config` (`delta_calibrate.py:286`) is the module entry point:
/// `module_factory()` exposes it (and nothing else) and it builds the
/// real object, registering both commands.
#[test]
fn module_factory_builds_the_object() {
    let factory = extras::delta_calibrate::module_factory();
    let load_config = factory.load_config.expect("load_config entry point");
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_printer_objects.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    let fakes = make_printer_with_endpoints(
        Some(Arc::new(FakeCalibration { params: base_params() })),
        vec![],
    );
    let cfg = make_config(fakes.printer.clone(), vec![("radius", "140.5")]);
    let obj = load_config(&cfg).expect("load_config").expect("object");
    let registered = fakes.gcode.command_names();
    assert_eq!(registered, vec!["DELTA_CALIBRATE", "DELTA_ANALYZE"]);
    // The entry point returns the `DeltaCalibrate` printer object.
    assert!(klippy_api::object::downcast::<DeltaCalibrate>(&*obj).is_some());
}
