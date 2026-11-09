//! Parity tests for `extras::skew_correction` (port of
//! `klippy/extras/skew_correction.py`).
//!
//! Every expected value comes live from the frozen reference through
//! `pyoracle` (CPython 3.8.2 + `klippy/`): minimal Python fakes for the
//! printer, config wrapper, `gcode`, `gcode_move` and `configfile` build
//! the reference object and drive it with the same inputs the Rust fakes
//! get, and the results (returned values bit-for-bit, raised errors by
//! class and message, `respond_info` texts, configfile `set` /
//! `remove_section` calls, `reset_last_position` calls, recorded move
//! transforms, `get_status` dicts, profile storage and command
//! registrations) are compared with `recording::assert_equal`, plus a canonical
//! fallback for float specials (`nan`/`inf`), which `assert_equal`
//! cannot decode (see `canonical_equal`).
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so replay fixtures do not apply; expected values come from the
//! oracle alone. The test skips when the oracle is unavailable (see
//! `pyoracle`).
//!
//! `gcode_move`'s initial transform is `FakeNextTransform` on both
//! sides: the Python `ToolHead` itself is the initial transform in
//! production, but `PrinterSkew` only uses `move` / `get_position`, so
//! the custom transform is behaviorally identical here.
#![allow(dead_code)]

use extras::skew_correction::{
    calc_skew_factor, load_config as skew_load_config, module_factory, PrinterSkew, SkewProfile,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetIntOpts, GetListOpts,
    GetListsOpts, GetStrOpts, ListParser, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GetFloatOpts as GGetFloatOpts, GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GetParser,
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, MoveTransform, MoveTransformApi,
    OutputCallback, RegisterCommandOpts,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT,
};
use kos::sync::Mutex;
use pycompat::text::{float_from_string, float_str, int_from_string};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import math
from extras import skew_correction as mod

class CommandError(Exception):
    pass

class Error(Exception):
    pass

class _Sentinel:
    pass

_sentinel = _Sentinel()

class FakeGCodeCommand:
    error = CommandError
    def __init__(self, command, commandline, params):
        self._command = command
        self._commandline = commandline
        self._params = dict(params)
        self.infos = []
        self.raws = []
    def get_command(self):
        return self._command
    def get_commandline(self):
        return self._commandline
    def get_command_parameters(self):
        return self._params
    def get_raw_command_parameters(self):
        return ""
    def respond_info(self, msg, log=True):
        self.infos.append([msg, bool(log)])
    def respond_raw(self, msg):
        self.raws.append(msg)
    def ack(self, msg=None):
        return False
    def get(self, name, default=_sentinel, parser=str, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _sentinel:
                raise self.error("Error on '%s': missing %s" % (self._commandline, name))
            return default
        try:
            value = parser(value)
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
    def get_int(self, name, default=_sentinel, minval=None, maxval=None):
        return self.get(name, default, parser=int, minval=minval, maxval=maxval)
    def get_float(self, name, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        return self.get(name, default, parser=float, minval=minval, maxval=maxval, above=above, below=below)

class FakeNextTransform:
    def __init__(self, position):
        self.position = [float(x) for x in position]
        self.moves = []
    def move(self, newpos, speed):
        self.moves.append((list(newpos), speed))
    def get_position(self):
        return list(self.position)

class FakeGCodeMove:
    def __init__(self, position):
        self.current = FakeNextTransform(position)
        self.resets = 0
        self.forces = []
    def set_move_transform(self, transform, force=False):
        prev = self.current
        self.current = transform
        self.forces.append(bool(force))
        return prev
    def reset_last_position(self):
        self.resets += 1

class FakeGCode:
    def __init__(self):
        self.registered = []
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        self.registered.append([cmd, desc])

class FakeConfigFile:
    def __init__(self):
        self.sets = []
        self.removed = []
    def set(self, section, option, value):
        self.sets.append([section, option, str(value)])
    def remove_section(self, section):
        self.removed.append(section)

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.handlers = []
    def register_event_handler(self, event, cb):
        self.handlers.append(event)
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

class FakeConfig:
    def __init__(self, printer, name, sections):
        self._printer = printer
        self._name = name
        self._sections = sections
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def get_prefix_sections(self, prefix):
        return [FakeConfig(self._printer, n, self._sections) for (n, o) in self._sections if n.startswith(prefix)]
    def getfloat(self, option):
        d = dict(self._sections)[self._name]
        if option not in d:
            raise Error("Option '%s' in section '%s' must be specified" % (option, self._name))
        try:
            return float(d[option])
        except Exception:
            raise Error("Unable to parse option '%s' in section '%s'" % (option, self._name))

def build(sections, position=(10.0, 20.0, 5.0, 0.0)):
    printer = FakePrinter()
    gcode = FakeGCode()
    gm = FakeGCodeMove(position)
    cf = FakeConfigFile()
    printer.objects['gcode'] = gcode
    printer.objects['gcode_move'] = gm
    printer.objects['configfile'] = cf
    cfg = FakeConfig(printer, 'skew_correction', sections)
    skew = mod.load_config(cfg)
    return (skew, printer, gcode, gm, cf)

def build_raw(sections):
    printer = FakePrinter()
    printer.objects['gcode'] = FakeGCode()
    printer.objects['gcode_move'] = FakeGCodeMove((0.0, 0.0, 0.0, 0.0))
    printer.objects['configfile'] = FakeConfigFile()
    return mod.load_config(FakeConfig(printer, 'skew_correction', sections))
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("skew fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping skew_correction oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.skew_correction".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: label.to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: Default::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

/// The `recording::assert_equal` float comparison decodes the CPython
/// hex spelling with `f64_from_hex`, which rejects `nan` and `inf`.
/// Both the oracle (CPython `float.hex()`) and `Value::float`
/// (`f64_to_hex`) spell NaN as `nan`, but `py_float_repr` renders it
/// as Rust `NaN`, so any NaN/Inf result diverges. Compare specials
/// semantically instead: NaN matches NaN (payloads are not part of
/// the Python spelling), Inf matches by sign.
fn special_of(v: &Value) -> Option<f64> {
    if let Value::Float(f) = v {
        match f.hex.to_lowercase().as_str() {
            "nan" => Some(f64::NAN),
            "inf" => Some(f64::INFINITY),
            "-inf" => Some(f64::NEG_INFINITY),
            _ => None,
        }
    } else {
        None
    }
}

fn canonical_equal(expected: &Value, actual: &Value) -> bool {
    match (expected, actual) {
        (Value::Float(_), Value::Float(_)) => {
            match (special_of(expected), special_of(actual)) {
                (Some(e), Some(a)) => {
                    (e.is_nan() && a.is_nan()) || (e.is_infinite() && a.is_infinite() && e.signum() == a.signum())
                }
                (None, None) => match (expected, actual) {
                    (Value::Float(e), Value::Float(a)) => {
                        match (e.to_f64(), a.to_f64()) {
                            (Some(x), Some(y)) => x.to_bits() == y.to_bits(),
                            _ => false,
                        }
                    }
                    _ => false,
                },
                _ => false,
            }
        }
        (Value::List(e), Value::List(a))
        | (Value::Tuple(e), Value::Tuple(a)) => {
            e.len() == a.len() && e.iter().zip(a).all(|(x, y)| canonical_equal(x, y))
        }
        (Value::Dict(e), Value::Dict(a)) => {
            e.len() == a.len()
                && e.iter().zip(a).all(|((ek, ev), (ak, av))| {
                    canonical_equal(ek, ak) && canonical_equal(ev, av)
                })
        }
        (Value::Struct { type_name: et, fields: ef }, Value::Struct { type_name: at, fields: af }) => {
            et == at
                && ef.len() == af.len()
                && ef.iter().zip(af).all(|((en, ev), (an, av))| {
                    en == an && canonical_equal(ev, av)
                })
        }
        _ => expected == actual,
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    let rec = record(label);
    match (expected, actual) {
        (Ok(e), Ok(a)) => {
            if let Err(d) = assert_equal(&rec, "ret", &e, &a) {
                if !canonical_equal(&e, &a) {
                    panic!("{label}: {d:?}");
                }
            }
        }
        (Ok(e), Err(err)) => {
            panic!("{label}: oracle returned Ok({e:?}), rust raised {}: {}", err.class_name(), err.message())
        }
        (Err(e), Ok(a)) => {
            panic!("{label}: oracle raised {}: {}, rust returned Ok({a:?})", e.class_name, e.message)
        }
        (Err(e), Err(err)) => {
            assert_eq!(err.class_name(), e.class_name, "{label}: error class");
            assert_eq!(err.message(), e.message, "{label}: error message");
        }
    }
}

fn vfloats(v: &[f64]) -> Value {
    Value::List(v.iter().map(|f| Value::float(*f)).collect())
}

fn pv_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            match s.parse::<i64>() {
                Ok(i) => Value::Int(i),
                Err(_) => Value::BigInt(s),
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pv_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pv_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter().map(|(k, v)| (pv_to_value(k), pv_to_value(v))).collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host{h}")),
    }
}

fn infos_value(infos: &[(String, bool)]) -> Value {
    Value::List(
        infos
            .iter()
            .map(|(m, l)| Value::List(vec![Value::str(m), Value::Bool(*l)]))
            .collect(),
    )
}

fn profiles_value(snaps: &[(String, SkewProfile)]) -> Value {
    Value::List(
        snaps
            .iter()
            .map(|(n, p)| {
                Value::Tuple(vec![
                    Value::str(n),
                    Value::Dict(vec![
                        (Value::str("xy_skew"), Value::float(p.xy_skew)),
                        (Value::str("xz_skew"), Value::float(p.xz_skew)),
                        (Value::str("yz_skew"), Value::float(p.yz_skew)),
                    ]),
                ])
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
    rollovers: Mutex<Vec<(String, Option<String>, bool)>>,
    shutdowns: Mutex<Vec<(String, PyValue)>>,
    exits: Mutex<Vec<String>>,
    error_updates: Mutex<Vec<(String, String)>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            rollovers: Mutex::new(Vec::new()),
            shutdowns: Mutex::new(Vec::new()),
            exits: Mutex::new(Vec::new()),
            error_updates: Mutex::new(Vec::new()),
        }
    }
    fn events(&self) -> Vec<String> {
        self.handlers.lock().iter().map(|(e, _)| e.clone()).collect()
    }
    fn exit_calls(&self) -> Vec<String> {
        self.exits.lock().clone()
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
        ("Ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, old_msg: &str, new_msg: &str) {
        self.error_updates
            .lock()
            .push((old_msg.to_string(), new_msg.to_string()));
    }
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
        if let Some(o) = objects.get(name) {
            return Ok(Some(o.clone()));
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
                None => objects.iter().map(|(n, o)| (n.clone(), o.clone())).collect(),
                Some(m) => {
                    let prefix = format!("{m} ");
                    let mut out = Vec::new();
                    if let Some(o) = objects.get(m) {
                        out.push((m.to_string(), o.clone()));
                    }
                    for (n, o) in objects.iter() {
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
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, name: &str, info: Option<&str>, log: bool) {
        self.rollovers
            .lock()
            .push((name.to_string(), info.map(|s| s.to_string()), log));
    }
    fn invoke_shutdown(&self, msg: &str, details: PyValue) {
        self.shutdowns.lock().push((msg.to_string(), details));
    }
    fn invoke_async_shutdown(&self, msg: &str, details: PyValue) {
        self.shutdowns.lock().push((msg.to_string(), details));
    }
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.lock().push((event.to_string(), callback));
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut handlers = self.handlers.lock();
        let mut out = Vec::new();
        for (e, cb) in handlers.iter_mut() {
            if e == event {
                out.push(cb(params)?);
            }
        }
        Ok(out)
    }
    fn request_exit(&self, result: &str) {
        self.exits.lock().push(result.to_string());
    }
}

type Sections = IndexMap<String, IndexMap<String, String>>;

#[derive(Clone)]
struct FakeConfig {
    printer: Printer,
    name: String,
    sections: Arc<Sections>,
}

impl FakeConfig {
    fn new(
        printer: Printer,
        name: &str,
        sections: Vec<(&str, Vec<(&str, &str)>)>,
    ) -> FakeConfig {
        let mut map: Sections = IndexMap::new();
        for (sec, opts) in sections {
            let mut o: IndexMap<String, String> = IndexMap::new();
            for (k, v) in opts {
                o.insert(k.to_string(), v.to_string());
            }
            map.insert(sec.to_string(), o);
        }
        FakeConfig {
            printer,
            name: name.to_string(),
            sections: Arc::new(map),
        }
    }
    fn raw(&self, option: &str) -> Option<String> {
        self.sections
            .get(&self.name)
            .and_then(|s| s.get(option).cloned())
    }
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        ))
    }
    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.name
        ))
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
        match self.raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let n = int_from_string(raw.trim(), 10)
            .map_err(|_| self.unparsable(option))?;
        let v: i64 = n
            .0
            .to_string()
            .parse()
            .map_err(|_| self.unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {min}",
                    self.name
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {max}",
                    self.name
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let v = float_from_string(&raw).map_err(|_| self.unparsable(option))?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.name,
                    float_str(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.name,
                    float_str(max)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.name,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.name,
                    float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        match raw.trim().to_ascii_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(self.unparsable(option)),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let raw = match self.raw(option) {
            Some(v) => v,
            None => match opts.default {
                None => return Err(self.missing(option)),
                Some(Some(d)) => return Ok(d),
                Some(None) => return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
            },
        };
        let keys: Vec<String> = match choices {
            PyValue::List(items) => items
                .iter()
                .filter_map(|v| {
                    if let PyValue::Str(s) = v {
                        Some(s.clone())
                    } else {
                        None
                    }
                })
                .collect(),
            PyValue::Dict(d) => d
                .keys()
                .filter_map(|v| {
                    if let PyValue::Str(s) = v {
                        Some(s.clone())
                    } else {
                        None
                    }
                })
                .collect(),
            _ => Vec::new(),
        };
        if !keys.contains(&raw) {
            return Err(KlippyError::config(format!(
                "Choice '{raw}' for option '{option}' in section '{}' is not a valid choice",
                self.name
            )));
        }
        Ok(PyValue::Str(raw))
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d.unwrap_or(PyValue::Tuple(Vec::new()))),
            };
        };
        let parts: Vec<String> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(',')
                .map(|p| p.trim().to_string())
                .filter(|p| !p.is_empty())
                .collect()
        };
        if let Some(count) = opts.count {
            if parts.len() != count {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.name
                )));
            }
        }
        let mut out = Vec::new();
        for p in parts {
            out.push(match opts.parser {
                ListParser::Str => PyValue::Str(p),
                ListParser::Int => {
                    let n = int_from_string(&p, 10).map_err(|_| self.unparsable(option))?;
                    PyValue::Int(n)
                }
                ListParser::Float => {
                    let f = float_from_string(&p).map_err(|_| self.unparsable(option))?;
                    PyValue::Float(f)
                }
            });
        }
        Ok(PyValue::Tuple(out))
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        Ok(Some(
            raw.split(&opts.sep)
                .map(|p| p.trim().to_string())
                .filter(|p| !p.is_empty())
                .collect(),
        ))
    }
    fn getintlist(&self, option: &str, opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let mut out = Vec::new();
        for p in raw.split(&opts.sep) {
            let p = p.trim();
            if p.is_empty() {
                continue;
            }
            let n = int_from_string(p, 10).map_err(|_| self.unparsable(option))?;
            out.push(
                n.0.to_string()
                    .parse::<i64>()
                    .map_err(|_| self.unparsable(option))?,
            );
        }
        Ok(Some(out))
    }
    fn getfloatlist(&self, option: &str, opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let Some(raw) = self.raw(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let mut out = Vec::new();
        for p in raw.split(&opts.sep) {
            let p = p.trim();
            if p.is_empty() {
                continue;
            }
            out.push(float_from_string(p).map_err(|_| self.unparsable(option))?);
        }
        Ok(Some(out))
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            name: section.to_string(),
            sections: Arc::clone(&self.sections),
        })
    }
    fn has_section(&self, section: &str) -> bool {
        self.sections.contains_key(section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.sections
            .keys()
            .filter(|s| s.starts_with(prefix))
            .map(|s| self.getsection(s))
            .collect()
    }
    fn get_prefix_options(&self, prefix: &str) -> KResult<Vec<String>> {
        Ok(self.sections
            .get(&self.name)
            .map(|o| {
                o.keys()
                    .filter(|k| k.starts_with(prefix))
                    .cloned()
                    .collect()
            })
            .unwrap_or_default())
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeGCode {
    handlers: Mutex<IndexMap<String, GCodeCommandHandler>>,
    registrations: Mutex<Vec<(String, Option<String>)>>,
    outputs: Mutex<Vec<String>>,
    infos: Mutex<Vec<(String, bool)>>,
    output_handlers: Mutex<Vec<OutputCallback>>,
    scripts: Mutex<Vec<String>>,
    restarts: Mutex<Vec<String>>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode {
            handlers: Mutex::new(IndexMap::new()),
            registrations: Mutex::new(Vec::new()),
            outputs: Mutex::new(Vec::new()),
            infos: Mutex::new(Vec::new()),
            output_handlers: Mutex::new(Vec::new()),
            scripts: Mutex::new(Vec::new()),
            restarts: Mutex::new(Vec::new()),
        }
    }
    fn registrations(&self) -> Vec<(String, Option<String>)> {
        self.registrations.lock().clone()
    }
    fn invoke(&self, cmd: &str, gcmd: &Arc<FakeGCodeCommand>) -> KResult<()> {
        let mut handlers = self.handlers.lock();
        let handler = handlers.get_mut(cmd).expect("handler registered");
        let gcmd_dyn: Arc<dyn GCodeCommandApi> = gcmd.clone();
        handler(&gcmd_dyn)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut handlers = self.handlers.lock();
        if func.is_none() {
            let old = handlers.shift_remove(cmd);
            return Ok(old);
        }
        if handlers.contains_key(cmd) {
            return Err(KlippyError::config(format!(
                "gcode command {cmd} already registered"
            )));
        }
        self.registrations.lock().push((cmd.to_string(), opts.desc));
        handlers.insert(cmd.to_string(), func.unwrap());
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
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, callback: OutputCallback) {
        self.output_handlers.lock().push(callback);
    }
    fn request_restart(&self, result: &str) -> KResult<()> {
        self.restarts.lock().push(result.to_string());
        Ok(())
    }
    fn run_script(&self, script: &str) -> KResult<()> {
        self.scripts.lock().push(script.to_string());
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.scripts.lock().push(script.to_string());
        Ok(())
    }
    fn get_mutex(&self) -> ReactorMutex {
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> Arc<dyn GCodeCommandApi> {
        Arc::new(FakeGCodeCommand::new(
            command, commandline, params.clone(),
        ))
    }
    fn respond_raw(&self, msg: &str) {
        self.outputs.lock().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }
    fn is_traditional_gcode(&self, cmd: &str) -> bool {
        let cmd = cmd.to_uppercase();
        let first = cmd.split_whitespace().next().unwrap_or("");
        if first.is_empty() {
            return false;
        }
        let (letter, rest) = first.split_at(1);
        rest.parse::<f64>().is_ok()
            && letter.chars().next().map(|c| c.is_uppercase()).unwrap_or(false)
            && rest.chars().next().map(|c| c.is_ascii_digit()).unwrap_or(false)
    }
    fn cmd_m112(&self, _gcmd: Option<&Arc<dyn GCodeCommandApi>>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}
impl PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

struct FakeGCodeCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    infos: Mutex<Vec<(String, bool)>>,
    raws: Mutex<Vec<String>>,
}

impl FakeGCodeCommand {
    fn new(command: &str, commandline: &str, params: IndexMap<String, String>) -> FakeGCodeCommand {
        FakeGCodeCommand {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            infos: Mutex::new(Vec::new()),
            raws: Mutex::new(Vec::new()),
        }
    }
    fn infos(&self) -> Vec<(String, bool)> {
        self.infos.lock().clone()
    }
    fn param_err(&self, msg: String) -> KlippyError {
        KlippyError::command(msg)
    }
    fn check_bounds_f(
        &self,
        name: &str,
        value: f64,
        minval: Option<f64>,
        maxval: Option<f64>,
        above: Option<f64>,
        below: Option<f64>,
    ) -> KResult<()> {
        if let Some(min) = minval {
            if value < min {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    float_str(min)
                )));
            }
        }
        if let Some(max) = maxval {
            if value > max {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    float_str(max)
                )));
            }
        }
        if let Some(above) = above {
            if value <= above {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = below {
            if value >= below {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    float_str(below)
                )));
            }
        }
        Ok(())
    }
}

impl GCodeCommandApi for FakeGCodeCommand {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.param_err(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            };
        };
        let value = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int => {
                let n = int_from_string(raw, 10).map_err(|_| {
                    self.param_err(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, raw
                    ))
                })?;
                PyValue::Int(n)
            }
            GetParser::IntAutoBase => {
                let n = int_from_string(raw, 0).map_err(|_| {
                    self.param_err(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, raw
                    ))
                })?;
                PyValue::Int(n)
            }
            GetParser::Float => {
                let f = float_from_string(raw).map_err(|_| {
                    self.param_err(format!(
                        "Error on '{}': unable to parse {}",
                        self.commandline, raw
                    ))
                })?;
                PyValue::Float(f)
            }
        };
        // Bounds (the module only passes them to get_float, via get_float).
        if let (Some(v), Some(min)) = (value.to_f64(), opts.minval.as_ref().and_then(|m| m.to_f64())) {
            if v < min {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline,
                    name,
                    float_str(min)
                )));
            }
        }
        if let (Some(v), Some(max)) = (value.to_f64(), opts.maxval.as_ref().and_then(|m| m.to_f64())) {
            if v > max {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline,
                    name,
                    float_str(max)
                )));
            }
        }
        if let (Some(v), Some(above)) = (value.to_f64(), opts.above.as_ref().and_then(|m| m.to_f64())) {
            if v <= above {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must be above {}",
                    self.commandline,
                    name,
                    float_str(above)
                )));
            }
        }
        if let (Some(v), Some(below)) = (value.to_f64(), opts.below.as_ref().and_then(|m| m.to_f64())) {
            if v >= below {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must be below {}",
                    self.commandline,
                    name,
                    float_str(below)
                )));
            }
        }
        Ok(value)
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.param_err(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            };
        };
        let n = int_from_string(raw, 10).map_err(|_| {
            self.param_err(format!(
                "Error on '{}': unable to parse {}",
                self.commandline, raw
            ))
        })?;
        let v: i64 = n
            .0
            .to_string()
            .parse()
            .map_err(|_| {
                self.param_err(format!(
                    "Error on '{}': unable to parse {}",
                    self.commandline, raw
                ))
            })?;
        if let Some(min) = opts.minval {
            if v < min {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have minimum of {}",
                    self.commandline, name, min
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(self.param_err(format!(
                    "Error on '{}': {} must have maximum of {}",
                    self.commandline, name, max
                )));
            }
        }
        Ok(Some(v))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.param_err(format!(
                    "Error on '{}': missing {}",
                    self.commandline, name
                ))),
                Some(d) => Ok(d),
            };
        };
        let v = float_from_string(raw).map_err(|_| {
            self.param_err(format!(
                "Error on '{}': unable to parse {}",
                self.commandline, raw
            ))
        })?;
        self.check_bounds_f(name, v, opts.minval, opts.maxval, opts.above, opts.below)?;
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
        let start = self.command.len();
        if self.commandline.len() >= start {
            self.commandline[start..].trim_start().to_string()
        } else {
            String::new()
        }
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.infos.lock().push((msg.to_string(), log));
    }
    fn respond_raw(&self, msg: &str) {
        self.raws.lock().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

struct FakeNextTransform {
    position: Mutex<Vec<f64>>,
    moves: Mutex<Vec<(Vec<f64>, f64)>>,
}

impl FakeNextTransform {
    fn new(position: Vec<f64>) -> FakeNextTransform {
        FakeNextTransform {
            position: Mutex::new(position),
            moves: Mutex::new(Vec::new()),
        }
    }
    fn moves(&self) -> Vec<(Vec<f64>, f64)> {
        self.moves.lock().clone()
    }
}

impl MoveTransformApi for FakeNextTransform {
    fn r#move(&self, newpos: Vec<f64>, speed: f64) -> KResult<()> {
        self.moves.lock().push((newpos.clone(), speed));
        Ok(())
    }
    fn get_position(&self) -> KResult<Vec<f64>> {
        Ok(self.position.lock().clone())
    }
}

struct FakeGCodeMove {
    current: MoveTransform,
    next: Arc<FakeNextTransform>,
    resets: Mutex<u64>,
    forces: Mutex<Vec<bool>>,
}

impl FakeGCodeMove {
    fn new(position: Vec<f64>) -> FakeGCodeMove {
        let next = Arc::new(FakeNextTransform::new(position));
        FakeGCodeMove {
            current: MoveTransform::Custom(next.clone()),
            next,
            resets: Mutex::new(0),
            forces: Mutex::new(Vec::new()),
        }
    }
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set_move_transform(
        &self,
        _transform: Option<MoveTransform>,
        force: bool,
    ) -> KResult<Option<MoveTransform>> {
        self.forces.lock().push(force);
        Ok(Some(self.current.clone()))
    }
    fn reset_last_position(&self) -> KResult<()> {
        *self.resets.lock() += 1;
        Ok(())
    }
    fn cmd_g1(&self, _gcmd: &Arc<dyn GCodeCommandApi>) -> KResult<()> {
        Ok(())
    }
}
impl PrinterObject for FakeGCodeMove {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeMoveApi);
}

struct FakeConfigFile {
    printer: Printer,
    file: Mutex<IndexMap<String, IndexMap<String, String>>>,
    sets: Mutex<Vec<(String, String, String)>>,
    removed: Mutex<Vec<String>>,
}

impl FakeConfigFile {
    fn new(printer: Printer) -> FakeConfigFile {
        FakeConfigFile {
            printer,
            file: Mutex::new(IndexMap::new()),
            sets: Mutex::new(Vec::new()),
            removed: Mutex::new(Vec::new()),
        }
    }
    fn empty_config(&self) -> Config {
        Arc::new(FakeConfig::new(
            self.printer.clone(),
            "skew_correction",
            Vec::new(),
        ))
    }
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Ok(self.empty_config())
    }
    fn read_main_config(&self) -> KResult<Config> {
        Ok(self.empty_config())
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        Ok(())
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
    fn deprecate_mcu_code(&self, _mcu: &klippy_api::mcu::Mcu, _feature: &str, _msg: Option<String>) {}
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.sets
            .lock()
            .push((section.to_string(), option.to_string(), value.to_string()));
        self.file
            .lock()
            .entry(section.to_string())
            .or_default()
            .insert(option.to_string(), value.to_string());
        Ok(())
    }
    fn remove_section(&self, section: &str) {
        self.removed.lock().push(section.to_string());
        self.file.lock().shift_remove(section);
    }
}
impl PrinterObject for FakeConfigFile {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

// ---------------------------------------------------------------------------
// World builders (Rust side mirrors the Python `build` helper).
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    gcode_move: Arc<FakeGCodeMove>,
    configfile: Arc<FakeConfigFile>,
    skew: Arc<PrinterSkew>,
}

fn build_skew(
    sections: Vec<(&str, Vec<(&str, &str)>)>,
    printer: &Arc<FakePrinter>,
) -> KResult<Arc<PrinterSkew>> {
    let config: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "skew_correction",
        sections,
    ));
    PrinterSkew::new(&config)
}

fn build_world(
    sections: Vec<(&str, Vec<(&str, &str)>)>,
    position: Vec<f64>,
) -> World {
    let printer = Arc::new(FakePrinter::new());
    let gcode = Arc::new(FakeGCode::new());
    let gcode_move = Arc::new(FakeGCodeMove::new(position));
    let configfile = Arc::new(FakeConfigFile::new(printer.clone()));
    let gcode_obj: ObjectRef = gcode.clone();
    printer.add_object("gcode", gcode_obj).unwrap();
    let gcode_move_obj: ObjectRef = gcode_move.clone();
    printer.add_object("gcode_move", gcode_move_obj).unwrap();
    let configfile_obj: ObjectRef = configfile.clone();
    printer.add_object("configfile", configfile_obj).unwrap();
    let skew = build_skew(sections, &printer).expect("build");
    World {
        printer,
        gcode,
        gcode_move,
        configfile,
        skew,
    }
}

fn py_sections(sections: &[(&str, Vec<(&str, &str)>)]) -> String {
    let mut out = String::from("[");
    for (i, (name, opts)) in sections.iter().enumerate() {
        if i > 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("('{name}', {{"));
        for (j, (k, v)) in opts.iter().enumerate() {
            if j > 0 {
                out.push_str(", ");
            }
            out.push_str(&format!("'{k}': '{v}'"));
        }
        out.push_str("})");
    }
    out.push(']');
    out
}

fn py_floats(v: &[f64]) -> String {
    let parts: Vec<String> = v.iter().map(|f| format!("{f:?}")).collect();
    format!("[{}]", parts.join(", "))
}

fn py_params(params: &[(&str, &str)]) -> String {
    let parts: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("'{k}': '{v}'"))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

fn base_sections() -> Vec<(&'static str, Vec<(&'static str, &'static str)>)> {
    vec![
        ("skew_correction", Vec::new()),
        (
            "skew_correction prof_a",
            vec![
                ("xy_skew", "0.01"),
                ("xz_skew", "-0.02"),
                ("yz_skew", "0.005"),
            ],
        ),
        (
            "skew_correction prof b",
            vec![
                ("xy_skew", "0.1"),
                ("xz_skew", "0.2"),
                ("yz_skew", "0.3"),
            ],
        ),
    ]
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn calc_skew_factor_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let grid: Vec<(f64, f64, f64)> = vec![
        (20.0, 20.0, 20.0),
        (100.0, 105.0, 90.0),
        (50.5, 49.25, 60.0),
        (1.0, 1.0, 1.0),
        (5.0, 1.0, 1.0),
        (1.0, 1.0, 5.0),
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (20.0, 20.0, -5.0),
        (100.0, 90.0, 105.0),
        (7.5, 7.5, 7.5),
        (f64::INFINITY, 1.0, 1.0),
    ];
    for (ac, bd, ad) in grid {
        let label = format!("calc_skew_factor({ac:?}, {bd:?}, {ad:?})");
        let expected = if ac.is_infinite() {
            py.eval(&format!(
                "mod.calc_skew_factor(float('inf'), {bd:?}, {ad:?})"
            ))
        } else {
            py.eval(&format!("mod.calc_skew_factor({ac:?}, {bd:?}, {ad:?})"))
        };
        let actual = calc_skew_factor(ac, bd, ad).map(Value::float);
        check(&label, expected, actual);
    }
}

#[test]
fn init_loads_profiles_and_registers() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "skew, printer, gcode, gm, cf = build({})",
        py_sections(&base_sections())
    ))
    .expect("build");
    let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);

    let expected = py.eval("gcode.registered");
    let actual = world.gcode.registrations();
    let actual_v = Value::List(
        actual
            .iter()
            .map(|(c, d)| {
                Value::List(vec![
                    Value::str(c),
                    match d {
                        Some(s) => Value::str(s),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    );
    check("registered commands", expected, Ok(actual_v));

    let expected = py.eval("printer.handlers");
    let actual_v = Value::List(world.printer.events().iter().map(|e| Value::str(e)).collect());
    check("event handlers", expected, Ok(actual_v));

    let expected = py.eval("list(skew.skew_profiles.items())");
    check(
        "profiles",
        expected,
        Ok(profiles_value(&world.skew.profiles_snapshot())),
    );

    let expected = py.eval("[skew.xy_factor, skew.xz_factor, skew.yz_factor]");
    check(
        "initial factors",
        expected,
        Ok(vfloats(&[
            world.skew.xy_factor(),
            world.skew.xz_factor(),
            world.skew.yz_factor(),
        ])),
    );

    let expected = py.eval("skew.get_status(0.)");
    let status = PrinterObject::get_status(&*world.skew, 0.0).unwrap().expect("status");
    check("get_status", expected, Ok(pv_to_value(&status)));
}

#[test]
fn init_profile_errors_match_oracle() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, Vec<(&str, &str)>)>)> = vec![
        (
            "missing option",
            vec![
                ("skew_correction", Vec::new()),
                (
                    "skew_correction prof",
                    vec![("xy_skew", "0.1"), ("xz_skew", "0.2")],
                ),
            ],
        ),
        (
            "bad float",
            vec![
                ("skew_correction", Vec::new()),
                (
                    "skew_correction prof",
                    vec![
                        ("xy_skew", "abc"),
                        ("xz_skew", "0.2"),
                        ("yz_skew", "0.3"),
                    ],
                ),
            ],
        ),
        (
            "section without space",
            vec![
                ("skew_correction", Vec::new()),
                (
                    "skew_correctionXYZ",
                    vec![
                        ("xy_skew", "0.1"),
                        ("xz_skew", "0.2"),
                        ("yz_skew", "0.3"),
                    ],
                ),
            ],
        ),
    ];
    for (label, sections) in cases {
        let expected = py.eval(&format!("build_raw({})", py_sections(&sections)));
        let printer = Arc::new(FakePrinter::new());
        let actual = build_skew(sections, &printer).map(|_| Value::Null);
        check(label, expected, actual);
    }
}

#[test]
fn transform_math_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "skew, printer, gcode, gm, cf = build({})",
        py_sections(&base_sections())
    ))
    .expect("build");
    let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);

    // Connect both sides (`klippy:connect` installs the transform chain).
    py.eval("skew._handle_connect()").expect("connect");
    world
        .printer
        .send_event(EV_KLIPPY_CONNECT, &[])
        .expect("connect");

    let expected = py.eval("gm.forces");
    let actual_v = Value::List(
        world
            .gcode_move
            .forces
            .lock()
            .iter()
            .map(|f| Value::Bool(*f))
            .collect(),
    );
    check("connect forces", expected, Ok(actual_v));

    py.eval("skew._update_skew(0.01, -0.02, 0.005)")
        .expect("update");
    world.skew.update_skew(0.01, -0.02, 0.005).expect("update");
    let expected = py.eval("gm.resets");
    check(
        "resets after update",
        expected,
        Ok(Value::Int(world.gcode_move.resets.lock().clone() as i64)),
    );

    let positions: Vec<Vec<f64>> = vec![
        vec![1.0, 2.0, 3.0, 4.0],
        vec![0.0, 0.0, 0.0, 0.0],
        vec![-5.5, 100.25, 3.75, 2.5],
        vec![1.0, 2.0, 3.0],
        vec![1.0, 2.0],
        Vec::new(),
    ];
    for pos in positions {
        let label = format!("calc_skew({pos:?})");
        let expected = py.eval(&format!("skew.calc_skew({})", py_floats(&pos)));
        let actual = world.skew.calc_skew(&pos).map(|v| vfloats(&v));
        check(&label, expected, actual);

        let label = format!("calc_unskew({pos:?})");
        let expected = py.eval(&format!("skew.calc_unskew({})", py_floats(&pos)));
        let actual = world.skew.calc_unskew(&pos).map(|v| vfloats(&v));
        check(&label, expected, actual);
    }

    let expected = py.eval("skew.get_position()");
    check(
        "get_position",
        expected,
        MoveTransformApi::get_position(&*world.skew).map(|v| vfloats(&v)),
    );

    // Move through the installed transform: the next (fake) transform
    // records the skewed position.
    py.eval("skew.move([30.0, 10.0, 2.0, 0.0], 100.0)").expect("move");
    MoveTransformApi::r#move(&*world.skew, vec![30.0, 10.0, 2.0, 0.0], 100.0)
        .expect("move");
    let expected = py.eval("skew.next_transform.moves");
    let next = &world.gcode_move.next;
    let actual_v = Value::List(
        next.moves()
            .iter()
            .map(|(p, s)| Value::Tuple(vec![vfloats(p), Value::float(*s)]))
            .collect(),
    );
    check("next transform moves", expected, Ok(actual_v));
}

#[test]
fn get_current_skew_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "skew, printer, gcode, gm, cf = build({})",
        py_sections(&base_sections())
    ))
    .expect("build");
    let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);

    for factors in [
        (0.01, -0.02, 0.005),
        (0.0, 0.0, 0.0),
        (-1.5, 2.25, 0.000001),
        (0.1234567, -0.9876543, std::f64::consts::PI),
    ] {
        let (xy, xz, yz) = factors;
        py.eval(&format!("skew._update_skew({xy:?}, {xz:?}, {yz:?})"))
            .expect("update");
        world.skew.update_skew(xy, xz, yz).expect("update");

        py.exec("gcmd = FakeGCodeCommand('GET_CURRENT_SKEW', 'GET_CURRENT_SKEW', {})")
            .expect("gcmd");
        py.eval("skew.cmd_GET_CURRENT_SKEW(gcmd)").expect("cmd");
        let expected = py.eval("gcmd.infos");

        let gcmd = Arc::new(FakeGCodeCommand::new(
            "GET_CURRENT_SKEW",
            "GET_CURRENT_SKEW",
            IndexMap::new(),
        ));
        world
            .gcode
            .invoke("GET_CURRENT_SKEW", &gcmd)
            .expect("cmd");
        check(
            &format!("GET_CURRENT_SKEW {factors:?}"),
            expected,
            Ok(infos_value(&gcmd.infos())),
        );
    }
}

#[test]
fn calc_measured_skew_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py.exec(&format!(
        "skew, printer, gcode, gm, cf = build({})",
        py_sections(&base_sections())
    ))
    .expect("build");
    let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);

    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("valid", vec![("AC", "20"), ("BD", "21"), ("AD", "19")]),
        ("missing AC", vec![("BD", "21"), ("AD", "19")]),
        ("AC zero", vec![("AC", "0"), ("BD", "21"), ("AD", "19")]),
        ("AC negative", vec![("AC", "-3"), ("BD", "21"), ("AD", "19")]),
        ("AC unparsable", vec![("AC", "x"), ("BD", "21"), ("AD", "19")]),
        ("BD missing", vec![("AC", "20"), ("AD", "19")]),
        ("AD comma", vec![("AC", "20"), ("BD", "21"), ("AD", "1,5")]),
        ("domain error", vec![("AC", "5"), ("BD", "1"), ("AD", "1")]),
        ("inf", vec![("AC", "inf"), ("BD", "1"), ("AD", "1")]),
    ];
    for (label, params) in cases {
        py.exec(&format!(
            "gcmd = FakeGCodeCommand('CALC_MEASURED_SKEW', 'CALC_MEASURED_SKEW {}', {})",
            params
                .iter()
                .map(|(k, v)| format!("{k}={v}"))
                .collect::<Vec<_>>()
                .join(" "),
            py_params(&params)
        ))
        .expect("gcmd");
        let expected = py.eval("skew.cmd_CALC_MEASURED_SKEW(gcmd)");
        let expected_infos = py.eval("gcmd.infos");

        let mut p: IndexMap<String, String> = IndexMap::new();
        for (k, v) in &params {
            p.insert(k.to_string(), v.to_string());
        }
        let gcmd = Arc::new(FakeGCodeCommand::new(
            "CALC_MEASURED_SKEW",
            &format!(
                "CALC_MEASURED_SKEW {}",
                params
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            ),
            p,
        ));
        let actual = world
            .gcode
            .invoke("CALC_MEASURED_SKEW", &gcmd)
            .map(|_| Value::Null);
        check(&format!("CALC_MEASURED_SKEW {label}"), expected, actual);
        check(
            &format!("CALC_MEASURED_SKEW infos {label}"),
            expected_infos,
            Ok(infos_value(&gcmd.infos())),
        );
    }
}

#[test]
fn set_skew_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>, Vec<f64>)> = vec![
        (
            "valid full",
            vec![("XY", "20,21,19"), ("XZ", "20,19,21"), ("YZ", "19,20,21")],
            vec![],
        ),
        (
            "valid spaced",
            vec![("XY", " 20 , 21 , 19 ")],
            vec![],
        ),
        ("partial XZ", vec![("XZ", "30,31,29")], vec![]),
        ("two numbers", vec![("XY", "1,2")], vec![]),
        ("one number", vec![("XY", "1")], vec![]),
        ("empty", vec![("XY", "")], vec![]),
        ("non numeric", vec![("XY", "a,b,c")], vec![]),
        ("four numbers", vec![("XY", "1,2,3,4")], vec![]),
        ("gap", vec![("XY", "1,,3")], vec![]),
        ("domain error", vec![("XY", "1,1,5")], vec![]),
    ];
    for (label, params, _) in cases {
        py.exec(&format!(
            "skew, printer, gcode, gm, cf = build({})",
            py_sections(&base_sections())
        ))
        .expect("build");
        py.exec(&format!(
            "gcmd = FakeGCodeCommand('SET_SKEW', 'SET_SKEW {}', {})",
            params
                .iter()
                .map(|(k, v)| format!("{k}={v}"))
                .collect::<Vec<_>>()
                .join(" "),
            py_params(&params)
        ))
        .expect("gcmd");
        let expected = py.eval("skew.cmd_SET_SKEW(gcmd)");
        let expected_infos = py.eval("gcmd.infos");
        let expected_factors =
            py.eval("[skew.xy_factor, skew.xz_factor, skew.yz_factor]");
        let expected_resets = py.eval("gm.resets");

        let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);
        let mut p: IndexMap<String, String> = IndexMap::new();
        for (k, v) in &params {
            p.insert(k.to_string(), v.to_string());
        }
        let gcmd = Arc::new(FakeGCodeCommand::new(
            "SET_SKEW",
            &format!(
                "SET_SKEW {}",
                params
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            ),
            p,
        ));
        let actual = world.gcode.invoke("SET_SKEW", &gcmd).map(|_| Value::Null);
        check(&format!("SET_SKEW {label}"), expected, actual);
        check(
            &format!("SET_SKEW infos {label}"),
            expected_infos,
            Ok(infos_value(&gcmd.infos())),
        );
        check(
            &format!("SET_SKEW factors {label}"),
            expected_factors,
            Ok(vfloats(&[
                world.skew.xy_factor(),
                world.skew.xz_factor(),
                world.skew.yz_factor(),
            ])),
        );
        check(
            &format!("SET_SKEW resets {label}"),
            expected_resets,
            Ok(Value::Int(world.gcode_move.resets.lock().clone() as i64)),
        );
    }
}

#[test]
fn set_skew_clear_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("CLEAR=1", vec![("CLEAR", "1")]),
        ("CLEAR=2", vec![("CLEAR", "2")]),
        ("CLEAR=-1", vec![("CLEAR", "-1")]),
        (
            "CLEAR=0 with plane",
            vec![("CLEAR", "0"), ("XY", "20,21,19")],
        ),
    ];
    for (label, params) in cases {
        py.exec(&format!(
            "skew, printer, gcode, gm, cf = build({})",
            py_sections(&base_sections())
        ))
        .expect("build");
        // Prime nonzero factors so CLEAR is observable.
        py.eval("skew._update_skew(0.03, 0.04, 0.05)").expect("prime");
        py.exec(&format!(
            "gcmd = FakeGCodeCommand('SET_SKEW', 'SET_SKEW {}', {})",
            params
                .iter()
                .map(|(k, v)| format!("{k}={v}"))
                .collect::<Vec<_>>()
                .join(" "),
            py_params(&params)
        ))
        .expect("gcmd");
        let expected = py.eval("skew.cmd_SET_SKEW(gcmd)");
        let expected_factors =
            py.eval("[skew.xy_factor, skew.xz_factor, skew.yz_factor]");
        let expected_resets = py.eval("gm.resets");

        let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);
        world.skew.update_skew(0.03, 0.04, 0.05).expect("prime");
        let mut p: IndexMap<String, String> = IndexMap::new();
        for (k, v) in &params {
            p.insert(k.to_string(), v.to_string());
        }
        let gcmd = Arc::new(FakeGCodeCommand::new(
            "SET_SKEW",
            &format!(
                "SET_SKEW {}",
                params
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            ),
            p,
        ));
        let actual = world.gcode.invoke("SET_SKEW", &gcmd).map(|_| Value::Null);
        check(&format!("SET_SKEW {label}"), expected, actual);
        check(
            &format!("SET_SKEW factors {label}"),
            expected_factors,
            Ok(vfloats(&[
                world.skew.xy_factor(),
                world.skew.xz_factor(),
                world.skew.yz_factor(),
            ])),
        );
        check(
            &format!("SET_SKEW resets {label}"),
            expected_resets,
            Ok(Value::Int(world.gcode_move.resets.lock().clone() as i64)),
        );
    }
}

#[test]
fn skew_profile_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("LOAD known", vec![("LOAD", "prof_a")]),
        ("LOAD with space", vec![("LOAD", "prof b")]),
        ("LOAD unknown", vec![("LOAD", "nosuch")]),
        ("SAVE", vec![("SAVE", "saved1")]),
        ("REMOVE known", vec![("REMOVE", "prof_a")]),
        ("REMOVE unknown", vec![("REMOVE", "nosuch")]),
        (
            "LOAD plus SAVE precedence",
            vec![("LOAD", "prof_a"), ("SAVE", "saved2")],
        ),
        ("no params", Vec::new()),
    ];
    for (label, params) in cases {
        py.exec(&format!(
            "skew, printer, gcode, gm, cf = build({})",
            py_sections(&base_sections())
        ))
        .expect("build");
        py.exec("skew._update_skew(0.01, -0.02, 0.005)")
            .expect("prime");
        py.exec(&format!(
            "gcmd = FakeGCodeCommand('SKEW_PROFILE', 'SKEW_PROFILE {}', {})",
            params
                .iter()
                .map(|(k, v)| format!("{k}={v}"))
                .collect::<Vec<_>>()
                .join(" "),
            py_params(&params)
        ))
        .expect("gcmd");
        let expected = py.eval("skew.cmd_SKEW_PROFILE(gcmd)");
        let expected_infos = py.eval("gcmd.infos");
        let expected_factors =
            py.eval("[skew.xy_factor, skew.xz_factor, skew.yz_factor]");
        let expected_name = py.eval("skew.current_profile_name");
        let expected_profiles = py.eval("list(skew.skew_profiles.items())");
        let expected_sets = py.eval("cf.sets");
        let expected_removed = py.eval("cf.removed");
        let expected_resets = py.eval("gm.resets");

        let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);
        world.skew.update_skew(0.01, -0.02, 0.005).expect("prime");
        let mut p: IndexMap<String, String> = IndexMap::new();
        for (k, v) in &params {
            p.insert(k.to_string(), v.to_string());
        }
        let gcmd = Arc::new(FakeGCodeCommand::new(
            "SKEW_PROFILE",
            &format!(
                "SKEW_PROFILE {}",
                params
                    .iter()
                    .map(|(k, v)| format!("{k}={v}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            ),
            p,
        ));
        let actual = world
            .gcode
            .invoke("SKEW_PROFILE", &gcmd)
            .map(|_| Value::Null);
        check(&format!("SKEW_PROFILE {label}"), expected, actual);
        check(
            &format!("SKEW_PROFILE infos {label}"),
            expected_infos,
            Ok(infos_value(&gcmd.infos())),
        );
        check(
            &format!("SKEW_PROFILE factors {label}"),
            expected_factors,
            Ok(vfloats(&[
                world.skew.xy_factor(),
                world.skew.xz_factor(),
                world.skew.yz_factor(),
            ])),
        );
        check(
            &format!("SKEW_PROFILE name {label}"),
            expected_name,
            Ok(Value::str(&world.skew.current_profile_name())),
        );
        check(
            &format!("SKEW_PROFILE profiles {label}"),
            expected_profiles,
            Ok(profiles_value(&world.skew.profiles_snapshot())),
        );
        let sets_v = Value::List(
            world
                .configfile
                .sets
                .lock()
                .iter()
                .map(|(s, o, v)| {
                    Value::List(vec![Value::str(s), Value::str(o), Value::str(v)])
                })
                .collect(),
        );
        check(
            &format!("SKEW_PROFILE sets {label}"),
            expected_sets,
            Ok(sets_v),
        );
        let removed_v = Value::List(
            world
                .configfile
                .removed
                .lock()
                .iter()
                .map(|s| Value::str(s))
                .collect(),
        );
        check(
            &format!("SKEW_PROFILE removed {label}"),
            expected_removed,
            Ok(removed_v),
        );
        check(
            &format!("SKEW_PROFILE resets {label}"),
            expected_resets,
            Ok(Value::Int(world.gcode_move.resets.lock().clone() as i64)),
        );
    }
}

#[test]
fn load_config_and_factory() {
    let Some(_py) = oracle() else { return };
    let world = build_world(base_sections(), vec![10.0, 20.0, 5.0, 0.0]);
    let status = PrinterObject::get_status(&*world.skew, 12.5).unwrap().expect("status");
    if let PyValue::Dict(d) = &status {
        assert_eq!(d.len(), 1);
        assert!(matches!(
            d.get(&PyValue::Str("current_profile_name".to_string())),
            Some(PyValue::Str(s)) if s.is_empty()
        ));
    } else {
        panic!("get_status did not return a dict");
    }
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_none());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
    // `load_config` returns the object under the section name. The
    // reference's `build` registers `gcode`, `gcode_move` and
    // `configfile` before loading; the port resolves `gcode` in
    // `__init__` and raises `Unknown config object 'gcode'` without it,
    // so mirror that here.
    let printer = Arc::new(FakePrinter::new());
    let gcode_obj: ObjectRef = Arc::new(FakeGCode::new());
    printer.add_object("gcode", gcode_obj).unwrap();
    let gcode_move_obj: ObjectRef = Arc::new(FakeGCodeMove::new(vec![10.0, 20.0, 5.0, 0.0]));
    printer.add_object("gcode_move", gcode_move_obj).unwrap();
    let configfile_obj: ObjectRef = Arc::new(FakeConfigFile::new(printer.clone()));
    printer.add_object("configfile", configfile_obj).unwrap();
    let config: Config = Arc::new(FakeConfig::new(
        printer.clone(),
        "skew_correction",
        base_sections(),
    ));
    let obj = skew_load_config(&config).expect("load").expect("object");
    let status = PrinterObject::get_status(&*obj, 0.0).unwrap().expect("status");
    if let PyValue::Dict(d) = &status {
        assert_eq!(d.len(), 1);
    } else {
        panic!("get_status did not return a dict");
    }
}
