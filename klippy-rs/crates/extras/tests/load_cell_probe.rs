//! Parity tests for `extras::load_cell_probe` (port of `klippy/extras/load_cell_probe.py`).
//!
//! Expected values come live from the frozen reference through `pyoracle` (CPython 3.8.2,
//! NumPy 1.16.4, the real `klippy/`): the real `extras.load_cell_probe` module is imported
//! and driven with the real `configfile.ConfigWrapper` over a `RawConfigParser` and the real
//! `gcode.GCodeCommand`. The port is driven through Rust fakes of the contract traits with
//! the same inputs, and every result (rendered with Python `repr`) or raised error (class
//! and message) is compared as text.
//!
//! Covered:
//! - `ParamHelper` for int, float and float-list parameters read from config and from G-code
//!   (limits that are `int` versus `float`, the `or` fallback of call-time limits, defaults
//!   returned unvalidated, `list` versus `tuple`, every error text),
//! - `ContinuousTareFilter` equality (list versus tuple notches) and `design_filter`,
//! - `ContinuousTareFilterHelper` construction, `_build_filter` and `update_from_command`,
//! - `LoadCellProbeConfigHelper` (a real `LoadCell` on the Rust side, a duck-typed load cell
//!   on the Python side): `tare_time`, `trigger_force` and `force_safety_limit` from config
//!   and G-code, the tare sample count, the safety range and its error paths, and
//!   `get_grams_per_count`,
//! - `np.average` over the counts column (bit exact against NumPy's pairwise sum) and the
//!   NumPy error texts for malformed sample rows,
//! - `TapAnalysis.to_dict`, `check_sensor_errors`, `FRAC_GRAMS_CONV` and the module factory,
//! - `LoadCellProbeCommands` (the `LOAD_CELL_TEST_TAP` registration, `TAPS` and `TIMEOUT`
//!   parsing, the responses, the timeout error) and `TapSession` (probe results, offsets,
//!   the session life cycle),
//! - the `sensor_type` choices and its `load_config` errors.
//!
//! The SciPy-dependent filter checks are skipped when the oracle interpreter has SciPy (the
//! port does not reproduce a missing SciPy). `LoadCellProbingMove`, `TappingMove` and the
//! construction of `LoadCellPrinterProbe` need a real `MCU_trigger_analog`, toolhead, homing
//! and sensor chip; they are verified on hardware (`ke-only` in the inventory).

use extras::bulk_sensor::ClientCallback;
use extras::load_cell::{BulkSensorAdc, CollectResult, LoadCell};
use extras::probe::{ProbeOffsetsHelperApi, ProbeSessionApi};
use extras::load_cell_probe::*;
use extras::trigger_analog::{CfgErrorKind, MCU_SosFilter};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, IntArgs,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GcmdFloatOpts,
    GetIntOpts as GcmdIntOpts, GetOpts as GcmdGetOpts, GetParser, OutputCallback,
    RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueue, CommandWrapper, ConfigCallback, LookupQueryCommandOpts, Mcu, McuApi, Pin,
    PinParams, SerialResponseCallback, TriggerDispatch,
};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::ObjectRef;
use pycompat::text::{float_from_string, float_repr};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex};
use recording::Value;
use std::collections::VecDeque;
use kos::sync::Mutex;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python side.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, configparser
# mcu.py imports modules that need a serial port or a C compiler.
for _name in ('serialhdl', 'chelper', 'clocksync'):
    try:
        __import__(_name)
    except Exception:
        sys.modules[_name] = types.ModuleType(_name)
import numpy
import configfile
import gcode as gcode_mod
import extras.load_cell_probe as lcp
lcp.np = numpy
try:
    import scipy.signal
    HAS_SCIPY = True
except Exception:
    HAS_SCIPY = False

class CommandError(Exception):
    pass

LOG = []

class FakeGCodeObj:
    def respond_info(self, msg, log=True):
        LOG.append(msg)
    def respond_raw(self, msg):
        pass

class RecGcode:
    def __init__(self):
        self.cmds = {}
    def register_command(self, name, func, desc=None):
        self.cmds[name] = (func, desc)

class RecReactor:
    def monotonic(self):
        return 100.0
    def pause(self, waketime):
        return waketime

class FakePrinter:
    command_error = CommandError
    config_error = configparser.Error
    def __init__(self):
        self.objects = {}
        self.reactor = RecReactor()
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        raise self.config_error("Unknown config object '%s'" % (name,))
    def get_reactor(self):
        return self.reactor
PRINTER = FakePrinter()

def mkcfg(opts, name='load_cell_probe'):
    fc = configparser.RawConfigParser()
    fc.add_section(name)
    for k, v in opts.items():
        fc.set(name, k, v)
    return configfile.ConfigWrapper(PRINTER, fc, {}, name)

def mkcmd(params, cmd='TEST_CMD'):
    line = cmd + ''.join([' %s=%s' % (k, v) for k, v in params.items()])
    return gcode_mod.GCodeCommand(FakeGCodeObj(), cmd, line, dict(params), False)

def res(fn):
    try:
        r = fn()
    except Exception as e:
        return 'raised %s: %s' % (type(e).__name__, e)
    return 'ok %s' % (r,)

class SpsSensor:
    def __init__(self, sps, rng=(-8388608, 8388607)):
        self.sps = sps
        self.rng = rng
    def get_samples_per_second(self):
        return self.sps
    def get_range(self):
        return self.rng

class FakeLoadCell:
    def __init__(self, sensor, counts_per_gram, reference_tare_counts):
        self.sensor = sensor
        self.counts_per_gram = counts_per_gram
        self.reference_tare_counts = reference_tare_counts
    def get_sensor(self):
        return self.sensor
    def get_counts_per_gram(self):
        return self.counts_per_gram
    def get_reference_tare_counts(self):
        return self.reference_tare_counts

class ScriptedMove:
    def __init__(self, results):
        self.results = list(results)
    def probing_test(self, gcmd, timeout):
        LOG.append('probing_test timeout=%r' % (timeout,))
        r = self.results.pop(0)
        if isinstance(r, Exception):
            raise r
        return r

class ScriptedTap:
    def __init__(self, results):
        self.results = list(results)
    def run_tap(self, gcmd):
        LOG.append('run_tap')
        r = self.results.pop(0)
        if isinstance(r, Exception):
            raise r
        return r

class FakeOffsets:
    def __init__(self, offsets):
        self.offsets = offsets
    def get_offsets(self, gcmd=None):
        return self.offsets

class FakeSos:
    def __init__(self):
        self.designs = []
    def set_filter_design(self, design):
        self.designs.append(design)

def render_filter(f):
    return repr([f.sps, f.drift, f.drift_delay, f.buzz, f.buzz_delay, f.notches,
                 f.notch_quality])
"#;

/// Python float literal (`{x}` would print `10` for `10.0`).
fn py_f(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v == f64::INFINITY {
        "float('inf')".to_string()
    } else if v == f64::NEG_INFINITY {
        "float('-inf')".to_string()
    } else {
        format!("{v:?}")
    }
}

fn py_num(n: Num) -> String {
    match n {
        Num::Int(i) => i.to_string(),
        Num::Float(f) => py_f(f),
    }
}

fn py_opt_num(n: Option<Num>) -> String {
    n.map_or("None".to_string(), py_num)
}

fn py_opt_f(v: Option<f64>) -> String {
    v.map_or("None".to_string(), py_f)
}

fn py_opt_i(v: Option<i64>) -> String {
    v.map_or("None".to_string(), |i| i.to_string())
}

fn py_dict(pairs: &[(&str, &str)]) -> String {
    let items: Vec<String> = pairs.iter().map(|(k, v)| format!("'{k}': '{v}'")).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_seq(seq: &FloatSeq) -> String {
    let items = seq.items().iter().map(|f| py_f(*f)).collect::<Vec<_>>().join(", ");
    match seq {
        FloatSeq::List(_) => format!("[{items}]"),
        FloatSeq::Tuple(v) if v.len() == 1 => format!("({items},)"),
        FloatSeq::Tuple(_) => format!("({items})"),
    }
}

// ---------------------------------------------------------------------------
// Rust renderers: the same text as Python `repr`.
// ---------------------------------------------------------------------------

fn r_floats(v: &[f64]) -> String {
    v.iter().map(|f| float_repr(*f)).collect::<Vec<_>>().join(", ")
}

fn r_seq(seq: &FloatSeq) -> String {
    match seq {
        FloatSeq::List(v) => format!("[{}]", r_floats(v)),
        FloatSeq::Tuple(v) if v.len() == 1 => format!("({},)", r_floats(v)),
        FloatSeq::Tuple(v) => format!("({})", r_floats(v)),
    }
}

fn r_param(v: &ParamValue) -> String {
    match v {
        ParamValue::None => "None".to_string(),
        ParamValue::Num(Num::Int(i)) => i.to_string(),
        ParamValue::Num(Num::Float(f)) => float_repr(*f),
        ParamValue::List(seq) => r_seq(seq),
    }
}

fn r_opt_f(v: Option<f64>) -> String {
    v.map_or("None".to_string(), float_repr)
}

fn r_opt_i(v: Option<i64>) -> String {
    v.map_or("None".to_string(), |i| i.to_string())
}

fn r_filter(f: &ContinuousTareFilter) -> String {
    format!(
        "[{}, {}, {}, {}, {}, {}, {}]",
        float_repr(f.sps),
        r_opt_f(f.drift),
        r_opt_i(f.drift_delay),
        r_opt_f(f.buzz),
        r_opt_i(f.buzz_delay),
        r_seq(&f.notches),
        r_opt_f(f.notch_quality),
    )
}

fn r_sections(v: &[Vec<f64>]) -> String {
    format!("[{}]", v.iter().map(|s| format!("[{}]", r_floats(s))).collect::<Vec<_>>().join(", "))
}

/// `res(...)`: `ok <text>` or `raised <class>: <message>`.
fn res<T>(r: KResult<T>, render: impl FnOnce(T) -> String) -> String {
    match r {
        Ok(v) => format!("ok {}", render(v)),
        Err(e) => format!("raised {}: {}", e.class_name(), e.message()),
    }
}

// ---------------------------------------------------------------------------
// Rust side fakes.
// ---------------------------------------------------------------------------
struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
}

impl FakePrinter {
    fn new() -> FakePrinter {
        FakePrinter { objects: Mutex::new(IndexMap::new()) }
    }
}

type Log = Arc<Mutex<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(Mutex::new(Vec::new()))
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        Reactor::new(false)
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".into())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.lock().get(name) {
            Some(object) => Ok(Some(object.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, klippy_api::object::ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<klippy_api::object::ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: klippy_api::printer::EventCallback) {}
    fn send_event(
        &self,
        _event: &str,
        _params: &[klippy_api::object::EventArg],
    ) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

fn printer() -> Printer {
    Arc::new(FakePrinter::new())
}

/// A printer whose `lookup_object` finds the given objects.
fn printer_with(objects: Vec<(&str, ObjectRef)>) -> Printer {
    let printer = FakePrinter::new();
    for (name, object) in objects {
        printer.objects.lock().insert(name.to_string(), object);
    }
    Arc::new(printer)
}

/// `configfile.ConfigWrapper` over a one-section config file.
struct FakeConfig {
    name: String,
    opts: IndexMap<String, String>,
    printer: Printer,
}

impl FakeConfig {
    fn make(opts: &[(&str, &str)]) -> Config {
        FakeConfig::make_with(opts, printer())
    }

    fn make_with(opts: &[(&str, &str)], printer: Printer) -> Config {
        Arc::new(FakeConfig {
            name: "load_cell_probe".to_string(),
            opts: opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            printer,
        })
    }

    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.name
        ))
    }

    fn unparsable(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
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
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                Some(default) => Ok(default),
                None => Err(self.missing(option)),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.missing(option)),
            };
        };
        let v: i64 = raw.trim().parse().map_err(|_| self.unparsable(option))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {m}",
                    self.name
                )));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {m}",
                    self.name
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.missing(option)),
            };
        };
        let v = float_from_string(raw).map_err(|_| self.unparsable(option))?;
        let fail = |what: &str, arg: IntArgs, limit: f64| {
            KlippyError::Config(format!(
                "Option '{option}' in section '{}' {what} {}",
                self.name,
                opts.ints.value(arg, limit).str()
            ))
        };
        if let Some(m) = opts.minval {
            if v < m {
                return Err(fail("must have minimum of", IntArgs::MINVAL, m));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(fail("must have maximum of", IntArgs::MAXVAL, m));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(fail("must be above", IntArgs::ABOVE, m));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(fail("must be below", IntArgs::BELOW, m));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        unimplemented!("getboolean is not used by the helpers under test")
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let PyValue::Dict(choices) = choices else {
            panic!("the module only passes dict choices");
        };
        let key = match self.opts.get(option) {
            Some(v) => PyValue::Str(v.clone()),
            None => match opts.default {
                Some(default) => default.unwrap_or(PyValue::None),
                None => return Err(self.missing(option)),
            },
        };
        match choices.get(&key) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::Config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                key.str(),
                self.name
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        unimplemented!("getlists is not used by the helpers under test")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        unimplemented!("getlist is not used by the helpers under test")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        unimplemented!("getintlist is not used by the helpers under test")
    }
    fn getfloatlist(&self, option: &str, opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        let Some(raw) = self.opts.get(option) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.missing(option)),
            };
        };
        if raw.trim().is_empty() {
            return Ok(Some(Vec::new()));
        }
        let parsed: Result<Vec<f64>, _> =
            raw.split(',').map(|p| float_from_string(p.trim())).collect();
        parsed.map(Some).map_err(|_| self.unparsable(option))
    }
    fn getsection(&self, _section: &str) -> Config {
        unimplemented!("getsection is not used by the helpers under test")
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

/// `gcode.GCodeCommand`.
struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    /// The messages passed to `respond_info`.
    log: Log,
}

impl FakeGcmd {
    fn build(params: &[(&str, &str)], log: Log) -> Arc<FakeGcmd> {
        let command = "TEST_CMD".to_string();
        let mut commandline = command.clone();
        for (k, v) in params {
            commandline.push_str(&format!(" {k}={v}"));
        }
        Arc::new(FakeGcmd {
            command,
            commandline,
            params: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
            log,
        })
    }

    fn make(params: &[(&str, &str)]) -> GCmd {
        FakeGcmd::build(params, new_log())
    }

    /// A command whose `respond_info` messages go to `log`.
    fn make_logged(params: &[(&str, &str)], log: &Log) -> GCmd {
        FakeGcmd::build(params, log.clone())
    }

    fn error(&self, message: String) -> KlippyError {
        KlippyError::Command(format!("Error on '{}': {message}", self.commandline))
    }
}

fn bound_f64(v: &PyValue) -> f64 {
    v.to_f64().expect("numeric bound")
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GcmdGetOpts) -> KResult<PyValue> {
        let Some(raw) = self.params.get(name) else {
            return match opts.default {
                Some(default) => Ok(default),
                None => Err(self.error(format!("missing {name}"))),
            };
        };
        let value = match opts.parser {
            GetParser::Str => PyValue::Str(raw.clone()),
            GetParser::Int => match raw.trim().parse::<i64>() {
                Ok(i) => PyValue::Int(PyInt::from_i64(i)),
                Err(_) => return Err(self.error(format!("unable to parse {raw}"))),
            },
            GetParser::Float => match float_from_string(raw) {
                Ok(f) => PyValue::Float(f),
                Err(_) => return Err(self.error(format!("unable to parse {raw}"))),
            },
            GetParser::IntAutoBase => unimplemented!("not used by the module"),
        };
        if matches!(opts.parser, GetParser::Str) {
            return Ok(value);
        }
        let v = bound_f64(&value);
        if let Some(m) = &opts.minval {
            if v < bound_f64(m) {
                return Err(self.error(format!("{name} must have minimum of {}", m.str())));
            }
        }
        if let Some(m) = &opts.maxval {
            if v > bound_f64(m) {
                return Err(self.error(format!("{name} must have maximum of {}", m.str())));
            }
        }
        if let Some(m) = &opts.above {
            if v <= bound_f64(m) {
                return Err(self.error(format!("{name} must be above {}", m.str())));
            }
        }
        if let Some(m) = &opts.below {
            if v >= bound_f64(m) {
                return Err(self.error(format!("{name} must be below {}", m.str())));
            }
        }
        Ok(value)
    }
    fn get_int(&self, name: &str, opts: GcmdIntOpts) -> KResult<Option<i64>> {
        let py = |v: Option<i64>| v.map(|i| PyValue::Int(PyInt::from_i64(i)));
        let value = self.get(
            name,
            GcmdGetOpts {
                default: opts.default.map(|d| py(d).unwrap_or(PyValue::None)),
                parser: GetParser::Int,
                minval: py(opts.minval),
                maxval: py(opts.maxval),
                ..GcmdGetOpts::default()
            },
        )?;
        Ok(match value {
            PyValue::Int(n) => n.inner().to_string().parse().ok(),
            _ => None,
        })
    }
    fn get_float(&self, name: &str, opts: GcmdFloatOpts) -> KResult<Option<f64>> {
        let py = |v: Option<f64>| v.map(PyValue::Float);
        let value = self.get(
            name,
            GcmdGetOpts {
                default: opts.default.map(|d| py(d).unwrap_or(PyValue::None)),
                parser: GetParser::Float,
                minval: py(opts.minval),
                maxval: py(opts.maxval),
                above: py(opts.above),
                below: py(opts.below),
            },
        )?;
        Ok(match value {
            PyValue::Float(f) => Some(f),
            _ => None,
        })
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.lock().push(msg.to_string());
    }
    fn respond_raw(&self, _msg: &str) {}
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

/// An MCU that only has to survive `MCU_SosFilter::new`.
struct FakeMcu {
    printer: Printer,
}

fn unused(what: &str) -> KlippyError {
    KlippyError::Mcu(format!("{what} is not used by the tests"))
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".into()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(unused("setup_pin"))
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(11)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _time: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, _msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Err(unused("lookup_command"))
    }
    fn lookup_query_command(
        &self,
        _msgformat: &str,
        _respformat: &str,
        _opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Err(unused("lookup_query_command"))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(42)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        struct Noop;
        impl AsyncResponseWrapperApi for Noop {
            fn unregister(&self) {}
        }
        Arc::new(Noop)
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
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Err(unused("get_constant_float"))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(unused("create_trigger_dispatch"))
    }
}

struct SpsSensor {
    sps: f64,
    range: (i64, i64),
    mcu: Mcu,
}

impl BulkSensorAdc for SpsSensor {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn get_samples_per_second(&self) -> f64 {
        self.sps
    }
    fn get_range(&self) -> (i64, i64) {
        self.range
    }
    fn add_client(&self, _callback: ClientCallback) -> KResult<()> {
        Err(unused("add_client"))
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

type CommandTable = IndexMap<String, (Option<String>, GCodeCommandHandler)>;

/// The `gcode` object: records the registered commands and their help strings.
struct FakeGCode {
    commands: Mutex<CommandTable>,
}

impl FakeGCode {
    fn new() -> FakeGCode {
        FakeGCode { commands: Mutex::new(IndexMap::new()) }
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let mut commands = self.commands.lock();
        match func {
            Some(handler) => {
                commands.insert(cmd.to_string(), (opts.desc, handler));
                Ok(None)
            }
            None => Ok(commands.shift_remove(cmd).map(|(_, handler)| handler)),
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
    fn get_mutex(&self) -> ReactorMutex {
        unimplemented!("get_mutex is not used by the module")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        unimplemented!("create_gcode_command is not used by the module")
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

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

/// The `webhooks` object: accepts every registration.
struct FakeWebhooks;

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        _path: &str,
        _key: &str,
        _value: &str,
        _callback: WebHooksCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn call_remote_method(&self, _method: &str, _params: &IndexMap<String, PyValue>) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    let mut py = pyoracle::Oracle::start()?;
    py.exec(SETUP).expect("load_cell_probe fakes");
    Some(py)
}

fn has_scipy(py: &mut pyoracle::Oracle) -> bool {
    match py.eval("HAS_SCIPY") {
        Ok(Value::Bool(b)) => b,
        other => panic!("HAS_SCIPY: {other:?}"),
    }
}

/// Evaluates a Python expression that yields a string and compares it with the Rust text.
fn compare(py: &mut pyoracle::Oracle, label: &str, expr: &str, rust: String) {
    match py.eval(expr) {
        Ok(Value::Str(expected)) => assert_eq!(rust, expected, "{label}\n  python: {expr}"),
        other => panic!("{label}: oracle returned {other:?} for {expr}"),
    }
}

fn run(py: &mut pyoracle::Oracle, code: &str) {
    if let Err(e) = py.exec(code) {
        panic!("oracle exec failed: {}: {}\n{code}", e.class_name, e.message);
    }
}

// ---------------------------------------------------------------------------
// ParamHelper.
// ---------------------------------------------------------------------------

/// (label, sample rate, config options, only constructible without SciPy)
type FilterConfig<'a> = (&'a str, f64, Vec<(&'a str, &'a str)>, bool);

type Ctor = Box<dyn Fn(&Config) -> KResult<ParamHelper>>;

struct Spec {
    label: &'static str,
    py_ctor: &'static str,
    rust_ctor: Ctor,
    configs: Vec<Vec<(&'static str, &'static str)>>,
    gcmds: Vec<(Vec<(&'static str, &'static str)>, Limits)>,
}

/// Call-time `minval`, `maxval`, `above`, `below` of `ParamHelper.get`.
#[derive(Clone, Copy, Default)]
struct Limits {
    minval: Option<Num>,
    maxval: Option<Num>,
    above: Option<Num>,
    below: Option<Num>,
}

const NONE: Limits = Limits { minval: None, maxval: None, above: None, below: None };

fn py_limits(l: &Limits) -> String {
    format!(
        "minval={}, maxval={}, above={}, below={}",
        py_opt_num(l.minval),
        py_opt_num(l.maxval),
        py_opt_num(l.above),
        py_opt_num(l.below)
    )
}

fn limits(minval: Option<Num>, maxval: Option<Num>, above: Option<Num>, below: Option<Num>) -> Limits {
    Limits { minval, maxval, above, below }
}

fn specs() -> Vec<Spec> {
    let int = Num::Int;
    let flt = Num::Float;
    vec![
        Spec {
            label: "int delay",
            py_ctor: "lcp.intParamHelper(cfg, 'delay', default=2, minval=1, maxval=2)",
            rust_ctor: Box::new(|c| int_param_helper(c, "delay", Some(2), Some(1), Some(2))),
            configs: vec![
                vec![],
                vec![("delay", "1")],
                vec![("delay", "2")],
                vec![("delay", "3")],
                vec![("delay", "0")],
                vec![("delay", "abc")],
                vec![("delay", "2.5")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("DELAY", "1")], NONE),
                (vec![("DELAY", "2")], NONE),
                (vec![("DELAY", "9")], NONE),
                (vec![("DELAY", "0")], NONE),
                (vec![("DELAY", "x")], NONE),
                (vec![("DELAY", "1.5")], NONE),
                (vec![("DELAY", "1")], limits(Some(int(2)), None, None, None)),
                (vec![("DELAY", "2")], limits(None, Some(int(0)), None, None)),
                (vec![("DELAY", "2")], limits(Some(int(0)), None, None, None)),
            ],
        },
        Spec {
            label: "int no default",
            py_ctor: "lcp.intParamHelper(cfg, 'samples_x', default=None, minval=-5, maxval=5)",
            rust_ctor: Box::new(|c| int_param_helper(c, "samples_x", None, Some(-5), Some(5))),
            configs: vec![
                vec![],
                vec![("samples_x", "-5")],
                vec![("samples_x", "-6")],
                vec![("samples_x", "5")],
                vec![("samples_x", "6")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("SAMPLES_X", "-6")], NONE),
                (vec![("SAMPLES_X", "7")], NONE),
                (vec![("SAMPLES_X", "0")], NONE),
            ],
        },
        Spec {
            label: "float trigger_force (int limits and default)",
            py_ctor: "lcp.floatParamHelper(cfg, 'trigger_force', default=75, minval=10, maxval=250)",
            rust_ctor: Box::new(|c| {
                float_param_helper(
                    c,
                    "trigger_force",
                    Some(Num::Int(75)),
                    ParamLimits {
                        minval: Some(Num::Int(10)),
                        maxval: Some(Num::Int(250)),
                        ..ParamLimits::default()
                    },
                )
            }),
            configs: vec![
                vec![],
                vec![("trigger_force", "75")],
                vec![("trigger_force", "10")],
                vec![("trigger_force", "9.99")],
                vec![("trigger_force", "250")],
                vec![("trigger_force", "250.5")],
                vec![("trigger_force", "abc")],
                vec![("trigger_force", "nan")],
                vec![("trigger_force", "inf")],
                vec![("trigger_force", "")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("TRIGGER_FORCE", "80")], NONE),
                (vec![("TRIGGER_FORCE", "9")], NONE),
                (vec![("TRIGGER_FORCE", "251")], NONE),
                (vec![("TRIGGER_FORCE", "x")], NONE),
                (vec![("TRIGGER_FORCE", "10")], NONE),
                (vec![("TRIGGER_FORCE", "250")], NONE),
                (vec![("TRIGGER_FORCE", "nan")], NONE),
                (vec![("TRIGGER_FORCE", "-inf")], NONE),
                (vec![("TRIGGER_FORCE", "15")], limits(Some(flt(20.0)), None, None, None)),
                (vec![("TRIGGER_FORCE", "100")], limits(None, None, Some(int(100)), None)),
                (vec![("TRIGGER_FORCE", "95")], limits(None, None, None, Some(flt(90.5)))),
                (vec![("TRIGGER_FORCE", "5")], limits(Some(int(0)), None, None, None)),
            ],
        },
        Spec {
            label: "float tare_time",
            py_ctor: "lcp.floatParamHelper(cfg, 'tare_time', default=4. / 60., minval=0.01, maxval=1.0)",
            rust_ctor: Box::new(|c| {
                float_param_helper(
                    c,
                    "tare_time",
                    Some(Num::Float(4. / 60.)),
                    ParamLimits {
                        minval: Some(Num::Float(0.01)),
                        maxval: Some(Num::Float(1.0)),
                        ..ParamLimits::default()
                    },
                )
            }),
            configs: vec![
                vec![],
                vec![("tare_time", "0.02")],
                vec![("tare_time", "0.009")],
                vec![("tare_time", "1.5")],
                vec![("tare_time", "1.0")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("TARE_TIME", "0.5")], NONE),
                (vec![("TARE_TIME", "0.001")], NONE),
                (vec![("TARE_TIME", "2")], NONE),
            ],
        },
        Spec {
            label: "float buzz (float above, int below)",
            py_ctor: "lcp.floatParamHelper(cfg, 'buzz', default=None, above=79.0, below=200)",
            rust_ctor: Box::new(|c| {
                float_param_helper(
                    c,
                    "buzz",
                    None,
                    ParamLimits {
                        above: Some(Num::Float(79.0)),
                        below: Some(Num::Int(200)),
                        ..ParamLimits::default()
                    },
                )
            }),
            configs: vec![
                vec![],
                vec![("buzz", "100")],
                vec![("buzz", "79")],
                vec![("buzz", "80")],
                vec![("buzz", "200")],
                vec![("buzz", "199.9")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("BUZZ", "79")], NONE),
                (vec![("BUZZ", "100")], NONE),
                (vec![("BUZZ", "200")], NONE),
                (vec![("BUZZ", "90")], limits(None, None, Some(flt(95.0)), None)),
                (vec![("BUZZ", "90")], limits(None, None, Some(flt(0.0)), None)),
            ],
        },
        Spec {
            label: "float plain (minval 0.0)",
            py_ctor: "lcp.floatParamHelper(cfg, 'plain', default=None, minval=0.0)",
            rust_ctor: Box::new(|c| {
                float_param_helper(
                    c,
                    "plain",
                    None,
                    ParamLimits { minval: Some(Num::Float(0.0)), ..ParamLimits::default() },
                )
            }),
            configs: vec![
                vec![],
                vec![("plain", "0")],
                vec![("plain", "-0.5")],
                vec![("plain", "3.25")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("PLAIN", "-1")], NONE),
                (vec![("PLAIN", "1")], limits(Some(flt(2.0)), None, None, None)),
                (vec![("PLAIN", "1")], limits(None, Some(int(0)), None, None)),
                (vec![("PLAIN", "5")], limits(None, Some(flt(4.5)), None, None)),
            ],
        },
        Spec {
            label: "float list notches",
            py_ctor: "lcp.floatListParamHelper(cfg, 'notches', default=[], above=0., below=200, max_len=2)",
            rust_ctor: Box::new(|c| {
                float_list_param_helper(
                    c,
                    "notches",
                    Some(FloatSeq::List(Vec::new())),
                    ParamLimits {
                        above: Some(Num::Float(0.)),
                        below: Some(Num::Int(200)),
                        max_len: Some(2),
                        ..ParamLimits::default()
                    },
                )
            }),
            configs: vec![
                vec![],
                vec![("notches", "10")],
                vec![("notches", "10, 20")],
                vec![("notches", "10,20,30")],
                vec![("notches", "0")],
                vec![("notches", "250")],
                vec![("notches", "abc")],
                vec![("notches", "")],
                vec![("notches", "10,,20")],
            ],
            gcmds: vec![
                (vec![], NONE),
                (vec![("NOTCHES", "")], NONE),
                (vec![("NOTCHES", "  ")], NONE),
                (vec![("NOTCHES", "10,20")], NONE),
                (vec![("NOTCHES", "10,x")], NONE),
                (vec![("NOTCHES", "5,300")], NONE),
                (vec![("NOTCHES", "1,2,3")], NONE),
                (vec![("NOTCHES", "0,5")], NONE),
                (vec![("NOTCHES", "10,50")], limits(None, None, Some(flt(10.0)), Some(flt(100.0)))),
                (vec![("NOTCHES", "50,100")], limits(None, None, Some(flt(10.0)), Some(flt(100.0)))),
                (vec![("NOTCHES", "20,30")], limits(None, None, Some(flt(10.0)), Some(flt(100.0)))),
                (vec![("NOTCHES", "20,30")], limits(None, None, Some(flt(0.0)), Some(flt(25.0)))),
                (vec![("NOTCHES", "20,30")], limits(None, None, None, Some(int(0)))),
            ],
        },
        Spec {
            label: "float list with a default",
            py_ctor: "lcp.floatListParamHelper(cfg, 'pre', default=[5.0, 6.0], above=1.0)",
            rust_ctor: Box::new(|c| {
                float_list_param_helper(
                    c,
                    "pre",
                    Some(FloatSeq::List(vec![5.0, 6.0])),
                    ParamLimits { above: Some(Num::Float(1.0)), ..ParamLimits::default() },
                )
            }),
            configs: vec![vec![], vec![("pre", "7,8")], vec![("pre", "0.5")]],
            gcmds: vec![
                (vec![], NONE),
                (vec![("PRE", "2,3")], NONE),
                (vec![("PRE", "0.5")], NONE),
                (vec![("PRE", "")], NONE),
            ],
        },
    ]
}

#[test]
fn param_helper_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    for spec in specs() {
        for cfg_opts in &spec.configs {
            let label = format!("{} config {:?}", spec.label, cfg_opts);
            run(
                &mut py,
                &format!(
                    "cfg = mkcfg({})\nH = None\nR_CTOR = None\ntry:\n    H = {}\n    R_CTOR = 'ok ' + repr(H.value)\nexcept Exception as e:\n    R_CTOR = 'raised %s: %s' % (type(e).__name__, e)\n",
                    py_dict(cfg_opts),
                    spec.py_ctor
                ),
            );
            let config = FakeConfig::make(cfg_opts);
            let built = (spec.rust_ctor)(&config);
            let rust_ctor = match &built {
                Ok(h) => format!("ok {}", r_param(&h.value)),
                Err(e) => format!("raised {}: {}", e.class_name(), e.message()),
            };
            compare(&mut py, &format!("{label} constructor"), "R_CTOR", rust_ctor);
            let Ok(helper) = built else { continue };
            // `get()` without a command or config is the stored value.
            compare(
                &mut py,
                &format!("{label} get()"),
                "res(lambda: repr(H.get()))",
                res(helper.get(None, None, None, None, None, None), |v| r_param(&v)),
            );
            for (params, call) in &spec.gcmds {
                let label = format!("{label} gcmd {params:?} limits {}", py_limits(call));
                let gcmd = FakeGcmd::make(params);
                compare(
                    &mut py,
                    &label,
                    &format!("res(lambda: repr(H.get(mkcmd({}), {})))", py_dict(params), py_limits(call)),
                    res(
                        helper.get(Some(&gcmd), call.minval, call.maxval, call.above, call.below, None),
                        |v| r_param(&v),
                    ),
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ContinuousTareFilter.
// ---------------------------------------------------------------------------

fn filter(
    sps: f64,
    drift: Option<f64>,
    drift_delay: Option<i64>,
    buzz: Option<f64>,
    buzz_delay: Option<i64>,
    notches: FloatSeq,
    notch_quality: Option<f64>,
) -> ContinuousTareFilter {
    ContinuousTareFilter { sps, drift, drift_delay, buzz, buzz_delay, notches, notch_quality }
}

fn py_filter(f: &ContinuousTareFilter) -> String {
    format!(
        "lcp.ContinuousTareFilter(sps={}, drift={}, drift_delay={}, buzz={}, buzz_delay={}, notches={}, notch_quality={})",
        py_f(f.sps),
        py_opt_f(f.drift),
        py_opt_i(f.drift_delay),
        py_opt_f(f.buzz),
        py_opt_i(f.buzz_delay),
        py_seq(&f.notches),
        py_opt_f(f.notch_quality),
    )
}

fn base_filter() -> ContinuousTareFilter {
    filter(400.0, None, Some(2), None, Some(2), FloatSeq::List(Vec::new()), Some(2.0))
}

#[test]
fn filter_equality_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let base = base_filter();
    let mut pairs: Vec<(String, ContinuousTareFilter, ContinuousTareFilter)> = Vec::new();
    pairs.push(("identical".into(), base.clone(), base.clone()));
    let mut other = base.clone();
    other.sps = 80.0;
    pairs.push(("sps".into(), base.clone(), other));
    let mut other = base.clone();
    other.drift = Some(5.0);
    pairs.push(("drift set".into(), base.clone(), other.clone()));
    pairs.push(("drift equal".into(), other.clone(), other.clone()));
    let mut other = base.clone();
    other.drift_delay = Some(1);
    pairs.push(("drift_delay".into(), base.clone(), other));
    let mut other = base.clone();
    other.buzz = Some(100.0);
    pairs.push(("buzz".into(), base.clone(), other));
    let mut other = base.clone();
    other.buzz_delay = None;
    pairs.push(("buzz_delay".into(), base.clone(), other));
    let mut other = base.clone();
    other.notch_quality = Some(3.0);
    pairs.push(("notch_quality".into(), base.clone(), other));
    let mut a = base.clone();
    a.notches = FloatSeq::List(vec![50.0]);
    let mut b = base.clone();
    b.notches = FloatSeq::Tuple(vec![50.0]);
    pairs.push(("list versus tuple".into(), a.clone(), b.clone()));
    pairs.push(("list versus list".into(), a.clone(), a.clone()));
    pairs.push(("tuple versus tuple".into(), b.clone(), b.clone()));
    let mut c = base.clone();
    c.notches = FloatSeq::Tuple(Vec::new());
    pairs.push(("empty list versus empty tuple".into(), base.clone(), c));
    let mut d = base.clone();
    d.notches = FloatSeq::List(vec![50.0, 60.0]);
    pairs.push(("different notches".into(), a, d));
    for (label, x, y) in &pairs {
        let expr = format!("repr({} == {})", py_filter(x), py_filter(y));
        let rust = if x == y { "True" } else { "False" };
        compare(&mut py, label, &expr, rust.to_string());
    }
}


#[test]
fn filter_design_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let scipy = has_scipy(&mut py);
    let cases: Vec<(&str, ContinuousTareFilter, bool)> = vec![
        // (label, filter, needs the absence of SciPy)
        ("empty", base_filter(), false),
        ("zero drift and buzz are skipped", filter(400.0, Some(0.0), Some(2), Some(0.0), Some(2), FloatSeq::List(vec![]), Some(2.0)), false),
        ("pre-generated lowpass", filter(400.0, None, Some(2), Some(25.0), Some(4), FloatSeq::List(vec![]), Some(2.0)), false),
        ("missing drift order", filter(400.0, Some(10.0), None, None, Some(2), FloatSeq::List(vec![]), Some(2.0)), false),
        ("missing buzz order", filter(400.0, None, Some(2), Some(100.0), None, FloatSeq::List(vec![]), Some(2.0)), false),
        ("highpass needs SciPy", filter(400.0, Some(10.0), Some(2), None, Some(2), FloatSeq::List(vec![]), Some(2.0)), true),
        ("lowpass needs SciPy", filter(400.0, None, Some(2), Some(100.0), Some(2), FloatSeq::List(vec![]), Some(2.0)), true),
        ("notch needs SciPy", filter(400.0, None, Some(2), None, Some(2), FloatSeq::Tuple(vec![50.0]), Some(2.0)), true),
    ];
    for (label, f, needs_no_scipy) in cases {
        if needs_no_scipy && scipy {
            continue;
        }
        for (kind, py_err, rust_kind) in [
            ("config", "configparser.Error", CfgErrorKind::Config),
            ("command", "CommandError", CfgErrorKind::Command),
        ] {
            compare(
                &mut py,
                &format!("{label} ({kind} error)"),
                &format!(
                    "res(lambda: (lambda df: repr([df.get_filter_sections(), df.sample_frequency]))({}.design_filter({py_err})))",
                    py_filter(&f)
                ),
                res(f.design_filter(rust_kind), |df| {
                    format!("[{}, {}]", r_sections(&df.get_filter_sections()), float_repr(df.sample_frequency.get()))
                }),
            );
        }
    }

}

// ---------------------------------------------------------------------------
// ContinuousTareFilterHelper.
// ---------------------------------------------------------------------------

#[test]
fn filter_helper_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let scipy = has_scipy(&mut py);
    // (label, sample rate, config options, constructed without SciPy only)
    let configs: Vec<FilterConfig> = vec![
        ("defaults 400", 400.0, vec![], false),
        ("defaults 80", 80.0, vec![], false),
        ("defaults 1000", 1000.0, vec![], false),
        ("odd rate", 333.5, vec![], false),
        ("empty notch list", 400.0, vec![("notch_filter_frequencies", "")], false),
        ("drift too low", 400.0, vec![("drift_filter_cutoff_frequency", "0.05")], false),
        ("drift too high", 400.0, vec![("drift_filter_cutoff_frequency", "21")], false),
        ("drift delay too high", 400.0, vec![("drift_filter_delay", "3")], false),
        ("buzz below the limit", 400.0, vec![("buzz_filter_cutoff_frequency", "80")], false),
        ("buzz above the rate", 400.0, vec![("buzz_filter_cutoff_frequency", "200")], false),
        ("buzz delay low", 400.0, vec![("buzz_filter_delay", "0")], false),
        ("notches too many", 400.0, vec![("notch_filter_frequencies", "10,20,30")], false),
        ("notch above the rate", 400.0, vec![("notch_filter_frequencies", "250")], false),
        ("notch quality low", 400.0, vec![("notch_filter_quality", "0.4")], false),
        ("notch quality high", 400.0, vec![("notch_filter_quality", "6.5")], false),
        ("low rate buzz needs SciPy", 80.0, vec![("buzz_filter_cutoff_frequency", "39.5")], true),
        ("drift needs SciPy", 400.0, vec![("drift_filter_cutoff_frequency", "10")], true),
        ("buzz needs SciPy", 400.0, vec![("buzz_filter_cutoff_frequency", "100")], true),
        ("notch needs SciPy", 400.0, vec![("notch_filter_frequencies", "50")], true),
    ];
    let gcmds: Vec<Vec<(&str, &str)>> = vec![
        vec![],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "5")],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "0.05")],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "25")],
        vec![("DRIFT_FILTER_DELAY", "1")],
        vec![("DRIFT_FILTER_DELAY", "3")],
        vec![("BUZZ_FILTER_CUTOFF_FREQUENCY", "100")],
        vec![("BUZZ_FILTER_CUTOFF_FREQUENCY", "300")],
        vec![("BUZZ_FILTER_CUTOFF_FREQUENCY", "50")],
        vec![("BUZZ_FILTER_DELAY", "1")],
        vec![("NOTCH_FILTER_FREQUENCIES", "100")],
        vec![("NOTCH_FILTER_FREQUENCIES", "10,20")],
        vec![("NOTCH_FILTER_FREQUENCIES", "1,2,3")],
        vec![("NOTCH_FILTER_FREQUENCIES", "0")],
        vec![("NOTCH_FILTER_FREQUENCIES", "x")],
        vec![("NOTCH_FILTER_FREQUENCIES", "")],
        vec![("NOTCH_FILTER_QUALITY", "3")],
        vec![("NOTCH_FILTER_QUALITY", "7")],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "5"), ("BUZZ_FILTER_CUTOFF_FREQUENCY", "100"), ("NOTCH_FILTER_FREQUENCIES", "50")],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "60"), ("NOTCH_FILTER_FREQUENCIES", "50")],
        vec![("BUZZ_FILTER_CUTOFF_FREQUENCY", "90"), ("NOTCH_FILTER_FREQUENCIES", "95")],
        vec![("DRIFT_FILTER_CUTOFF_FREQUENCY", "5"), ("NOTCH_FILTER_FREQUENCIES", "5")],
    ];
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer() });
    for (label, sps, cfg_opts, needs_no_scipy) in &configs {
        if *needs_no_scipy && scipy {
            continue;
        }
        let label = format!("{label} {cfg_opts:?}");
        run(
            &mut py,
            &format!(
                "cfg = mkcfg({})\nSOS = FakeSos()\nH = None\nR_CTOR = None\ntry:\n    H = lcp.ContinuousTareFilterHelper(cfg, SpsSensor({}), SOS)\n    R_CTOR = 'ok'\nexcept Exception as e:\n    R_CTOR = 'raised %s: %s' % (type(e).__name__, e)\n",
                py_dict(cfg_opts),
                py_f(*sps)
            ),
        );
        let config = FakeConfig::make(cfg_opts);
        let sensor: Arc<dyn BulkSensorAdc> =
            Arc::new(SpsSensor { sps: *sps, range: (-0x800000, 0x7FFFFF), mcu: mcu.clone() });
        let sos = MCU_SosFilter::new(mcu.clone(), CommandQueue(42), 4).expect("sos filter");
        let built = ContinuousTareFilterHelper::new(&config, &sensor, sos);
        let rust_ctor = match &built {
            Ok(_) => "ok".to_string(),
            Err(e) => format!("raised {}: {}", e.class_name(), e.message()),
        };
        compare(&mut py, &format!("{label} constructor"), "R_CTOR", rust_ctor);
        let Ok(helper) = built else { continue };
        compare(
            &mut py,
            &format!("{label} config filter"),
            "res(lambda: render_filter(H._build_filter()))",
            res(helper.build_filter(None), |f| r_filter(&f)),
        );
        for params in &gcmds {
            let gcmd = FakeGcmd::make(params);
            let py_cmd = format!("mkcmd({})", py_dict(params));
            compare(
                &mut py,
                &format!("{label} gcmd {params:?} _build_filter"),
                &format!("res(lambda: render_filter(H._build_filter({py_cmd})))"),
                res(helper.build_filter(Some(&gcmd)), |f| r_filter(&f)),
            );
            compare(
                &mut py,
                &format!("{label} gcmd {params:?} update_from_command"),
                &format!("res(lambda: H.update_from_command({py_cmd}))"),
                res(helper.update_from_command(&gcmd), |_| "None".to_string()),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// NumPy helpers: np.average over the counts column and TapAnalysis.
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum Cell {
    I(i64),
    F(f64),
    N,
}

fn py_cell(c: &Cell) -> String {
    match c {
        Cell::I(i) => i.to_string(),
        Cell::F(f) => py_f(*f),
        Cell::N => "None".to_string(),
    }
}

fn rust_cell(c: &Cell) -> PyValue {
    match c {
        Cell::I(i) => PyValue::Int(PyInt::from_i64(*i)),
        Cell::F(f) => PyValue::Float(*f),
        Cell::N => PyValue::None,
    }
}

/// Rows of cells as a Python literal and as `PyValue` rows (lists, or tuples when `tuple`).
fn py_rows(rows: &[Vec<Cell>], tuple: bool) -> String {
    let rows: Vec<String> = rows
        .iter()
        .map(|r| {
            let items = r.iter().map(py_cell).collect::<Vec<_>>().join(", ");
            match (tuple, r.len()) {
                (false, _) => format!("[{items}]"),
                (true, 1) => format!("({items},)"),
                (true, _) => format!("({items})"),
            }
        })
        .collect();
    format!("[{}]", rows.join(", "))
}

fn rust_rows(rows: &[Vec<Cell>], tuple: bool) -> Vec<PyValue> {
    rows.iter()
        .map(|r| {
            let cells: Vec<PyValue> = r.iter().map(rust_cell).collect();
            if tuple {
                PyValue::Tuple(cells)
            } else {
                PyValue::List(cells)
            }
        })
        .collect()
}

/// Deterministic pseudo random numbers in `[0, 1)`.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[test]
fn average_of_counts_matches_numpy() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let lengths = [1usize, 2, 3, 7, 8, 9, 15, 16, 17, 23, 63, 64, 65, 100, 127, 128, 129, 130, 255, 256, 257, 300, 1000, 4097];
    for &n in &lengths {
        // Integer counts as the sensors deliver them.
        let int_rows: Vec<Vec<Cell>> = (0..n)
            .map(|i| {
                let counts = ((rng.next() - 0.5) * 16_000_000.0) as i64;
                vec![Cell::F(i as f64 * 0.0025), Cell::F(rng.next()), Cell::I(counts), Cell::I(-1234)]
            })
            .collect();
        // Floats over many magnitudes expose any change of the summation order.
        let float_rows: Vec<Vec<Cell>> = (0..n)
            .map(|i| {
                let scale = 10f64.powi((rng.next() * 12.0) as i32 - 3);
                vec![Cell::F(i as f64), Cell::N, Cell::F((rng.next() - 0.5) * scale), Cell::N]
            })
            .collect();
        for (label, rows) in [("int counts", &int_rows), ("float counts", &float_rows)] {
            let expr = format!(
                "res(lambda: repr(numpy.average(numpy.array({})[:, 2].astype(float))))",
                py_rows(rows, false)
            );
            compare(
                &mut py,
                &format!("{label} n={n}"),
                &expr,
                res(average_sample_counts(&rust_rows(rows, false)), float_repr),
            );
        }
    }
    // File output returns one tuple of three zeros.
    let rows = vec![vec![Cell::F(0.0), Cell::F(0.0), Cell::F(0.0)]];
    compare(
        &mut py,
        "file output tuple",
        &format!("res(lambda: repr(numpy.average(numpy.array({})[:, 2].astype(float))))", py_rows(&rows, true)),
        res(average_sample_counts(&rust_rows(&rows, true)), float_repr),
    );
    // Malformed rows.
    let ragged: Vec<Vec<Cell>> = vec![
        vec![Cell::F(1.0), Cell::F(2.0), Cell::I(3), Cell::I(4)],
        vec![Cell::F(1.0), Cell::F(2.0), Cell::I(3)],
    ];
    let short: Vec<Vec<Cell>> = vec![vec![Cell::F(1.0), Cell::F(2.0)], vec![Cell::F(1.0), Cell::F(2.0)]];
    let none_counts: Vec<Vec<Cell>> = vec![vec![Cell::F(1.0), Cell::F(2.0), Cell::N, Cell::I(4)]];
    let none_grams: Vec<Vec<Cell>> = vec![vec![Cell::F(1.0), Cell::N, Cell::I(9), Cell::N], vec![Cell::F(2.0), Cell::N, Cell::I(10), Cell::N]];
    let empty: Vec<Vec<Cell>> = vec![];
    for (label, rows) in [
        ("ragged", &ragged),
        ("too few columns", &short),
        ("None counts", &none_counts),
        ("None elsewhere", &none_grams),
        ("empty", &empty),
    ] {
        compare(
            &mut py,
            label,
            &format!("res(lambda: repr(numpy.average(numpy.array({})[:, 2].astype(float))))", py_rows(rows, false)),
            res(average_sample_counts(&rust_rows(rows, false)), float_repr),
        );
    }
    // Rows that are not sequences give a 1-D array.
    compare(
        &mut py,
        "scalar rows",
        "res(lambda: repr(numpy.average(numpy.array([1.0, 2.0, 3.0])[:, 2].astype(float))))",
        res(
            average_sample_counts(&[PyValue::Float(1.0), PyValue::Float(2.0), PyValue::Float(3.0)]),
            float_repr,
        ),
    );
}

#[test]
fn tap_analysis_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let mut rng = Rng(0xD1B5_4A32_D192_ED03);
    let tap_rows: Vec<Vec<Cell>> = (0..40)
        .map(|i| vec![Cell::F(100.0 + i as f64 * 0.0025), Cell::F((rng.next() - 0.5) * 300.0), Cell::I(i * 7 - 100), Cell::I(55)])
        .collect();
    let cases: Vec<(&str, Vec<Vec<Cell>>, bool)> = vec![
        ("tap rows", tap_rows, false),
        ("one row", vec![vec![Cell::F(1.5), Cell::F(-2.25), Cell::I(3), Cell::I(4)]], false),
        ("integer time and force", vec![vec![Cell::I(1), Cell::I(2), Cell::I(3), Cell::I(4)]], false),
        ("file output tuple", vec![vec![Cell::F(0.0), Cell::F(0.0), Cell::F(0.0)]], true),
        ("tuple rows", vec![vec![Cell::F(1.0), Cell::F(2.0), Cell::I(3), Cell::I(4)], vec![Cell::F(1.5), Cell::F(2.5), Cell::I(5), Cell::I(6)]], true),
        ("no force", vec![vec![Cell::F(1.0), Cell::N, Cell::I(3), Cell::I(4)]], false),
        ("no time", vec![vec![Cell::N, Cell::F(1.0), Cell::I(3), Cell::I(4)]], false),
        ("ragged", vec![vec![Cell::F(1.0), Cell::F(2.0), Cell::I(3)], vec![Cell::F(1.0), Cell::F(2.0)]], false),
        ("one column", vec![vec![Cell::F(1.0)], vec![Cell::F(2.0)]], false),
        ("empty rows", vec![vec![], vec![]], false),
        ("no samples", vec![], false),
        ("nan and inf", vec![vec![Cell::F(f64::NAN), Cell::F(f64::INFINITY), Cell::I(1)], vec![Cell::F(-0.0), Cell::F(f64::NEG_INFINITY), Cell::I(2)]], false),
    ];
    for (label, rows, tuple) in &cases {
        compare(
            &mut py,
            label,
            &format!("res(lambda: repr(lcp.TapAnalysis({}).to_dict()))", py_rows(rows, *tuple)),
            res(TapAnalysis::new(&rust_rows(rows, *tuple)), |t| t.to_dict().repr()),
        );
    }
    // Rows that are not sequences give a 1-D array.
    compare(
        &mut py,
        "scalar rows",
        "res(lambda: repr(lcp.TapAnalysis([1.0, 2.0]).to_dict()))",
        res(TapAnalysis::new(&[PyValue::Float(1.0), PyValue::Float(2.0)]), |t| t.to_dict().repr()),
    );
    compare(
        &mut py,
        "nested element",
        "res(lambda: repr(lcp.TapAnalysis([[1.0, [2.0]], [3.0, [4.0]]]).to_dict()))",
        res(
            TapAnalysis::new(&[
                PyValue::List(vec![PyValue::Float(1.0), PyValue::List(vec![PyValue::Float(2.0)])]),
                PyValue::List(vec![PyValue::Float(3.0), PyValue::List(vec![PyValue::Float(4.0)])]),
            ]),
            |t| t.to_dict().repr(),
        ),
    );
}

// ---------------------------------------------------------------------------
// check_sensor_errors, constants and the module factory.
// ---------------------------------------------------------------------------

#[test]
fn check_sensor_errors_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let rows = vec![vec![Cell::F(1.0), Cell::F(2.0), Cell::I(3), Cell::I(4)], vec![Cell::F(1.5), Cell::N, Cell::I(5), Cell::N]];
    type ErrorCase = (&'static str, &'static str, Option<(i64, i64)>);
    let cases: Vec<ErrorCase> = vec![
        ("no errors", "0", None),
        ("errors only", "(3, 0)", Some((3, 0))),
        ("overflows only", "(0, 5)", Some((0, 5))),
        ("both", "(7, 9)", Some((7, 9))),
    ];
    for (label, py_errors, errors) in cases {
        compare(
            &mut py,
            label,
            &format!(
                "res(lambda: repr(lcp.check_sensor_errors(({}, {}), PRINTER)))",
                py_rows(&rows, false),
                py_errors
            ),
            res(check_sensor_errors(&CollectResult { samples: rust_rows(&rows, false), errors }), |s| {
                PyValue::List(s).repr()
            }),
        );
    }
    compare(
        &mut py,
        "no samples",
        "res(lambda: repr(lcp.check_sensor_errors(([], 0), PRINTER)))",
        res(check_sensor_errors(&CollectResult { samples: Vec::new(), errors: None }), |s| {
            PyValue::List(s).repr()
        }),
    );
}

#[test]
fn constants_and_module_factory_match_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    compare(&mut py, "FRAC_GRAMS_CONV", "repr(lcp.FRAC_GRAMS_CONV)", float_repr(FRAC_GRAMS_CONV));
    // The module defines `load_config` only.
    compare(
        &mut py,
        "entry points",
        "repr([hasattr(lcp, 'load_config'), hasattr(lcp, 'load_config_prefix'), hasattr(lcp, 'add_printer_objects'), hasattr(lcp, 'add_early_printer_objects')])",
        {
            let f = module_factory();
            format!(
                "[{}, {}, {}, {}]",
                if f.load_config.is_some() { "True" } else { "False" },
                if f.load_config_prefix.is_some() { "True" } else { "False" },
                if f.add_printer_objects.is_some() { "True" } else { "False" },
                if f.add_early_printer_objects.is_some() { "True" } else { "False" },
            )
        },
    );
}

// ---------------------------------------------------------------------------
// sensor_type, LoadCellProbeConfigHelper, LOAD_CELL_TEST_TAP and TapSession.
// ---------------------------------------------------------------------------

type Params = Vec<(&'static str, &'static str)>;

#[test]
fn sensor_type_option_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let names = sensor_type_names()
        .iter()
        .map(|n| format!("'{n}'"))
        .collect::<Vec<_>>()
        .join(", ");
    compare(
        &mut py,
        "sensor type names in dict order",
        "repr(list(dict(list(lcp.hx71x.HX71X_SENSOR_TYPES.items()) + list(lcp.ads1220.ADS1220_SENSOR_TYPE.items()) + list(lcp.ads131m0x.ADS131M0X_SENSOR_TYPES.items()))))",
        format!("[{names}]"),
    );
    let load_config = module_factory().load_config.expect("load_config entry point");
    let cases: Vec<Params> = vec![
        vec![],
        vec![("sensor_type", "bogus")],
        vec![("sensor_type", "HX711")],
        vec![("sensor_type", "")],
        vec![("sensor_type", "ads1220x")],
    ];
    for opts in &cases {
        let config = FakeConfig::make(opts);
        compare(
            &mut py,
            &format!("load_config {opts:?}"),
            &format!("res(lambda: lcp.load_config(mkcfg({})))", py_dict(opts)),
            res(load_config(&config), |_| "object".to_string()),
        );
    }
}

struct HelperCase {
    label: &'static str,
    opts: Params,
    sps: f64,
    range: (i64, i64),
    counts_per_gram: Option<f64>,
    reference_tare_counts: Option<i64>,
}

const FULL_RANGE: (i64, i64) = (-0x800000, 0x7FFFFF);

fn helper_case(
    label: &'static str,
    opts: Params,
    counts_per_gram: Option<f64>,
    reference_tare_counts: Option<i64>,
) -> HelperCase {
    HelperCase { label, opts, sps: 400.0, range: FULL_RANGE, counts_per_gram, reference_tare_counts }
}

#[test]
fn config_helper_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    let mcu: Mcu = Arc::new(FakeMcu { printer: printer() });
    let mut cases = vec![
        helper_case("defaults", vec![], Some(1000.5), Some(5000)),
        helper_case("uncalibrated", vec![], None, None),
        helper_case("counts per gram only", vec![], Some(250.0), None),
        helper_case("reference tare only", vec![], None, Some(-4000)),
        helper_case("counts per gram at the overflow limit", vec![], Some(536870912.0), Some(0)),
        helper_case("counts per gram below the overflow limit", vec![], Some(536870911.0), Some(0)),
        helper_case("counts per gram above the overflow limit", vec![], Some(600000000.5), Some(1)),
        helper_case("safety range above the sensor range", vec![], Some(4000.0), Some(400000)),
        helper_case("safety range below the sensor range", vec![], Some(4000.0), Some(-400000)),
        helper_case("small counts per gram", vec![], Some(1.0), Some(123456)),
        helper_case("tare time", vec![("tare_time", "0.5")], Some(100.0), Some(10)),
        helper_case("tare time minimum", vec![("tare_time", "0.01")], Some(100.0), Some(10)),
        helper_case("tare time maximum", vec![("tare_time", "1")], Some(100.0), Some(10)),
        helper_case("tare time too small", vec![("tare_time", "0.005")], Some(100.0), Some(10)),
        helper_case("tare time too large", vec![("tare_time", "1.5")], Some(100.0), Some(10)),
        helper_case("tare time unparsable", vec![("tare_time", "x")], Some(100.0), Some(10)),
        helper_case("trigger force", vec![("trigger_force", "100")], Some(100.0), Some(10)),
        helper_case("trigger force minimum", vec![("trigger_force", "10")], Some(100.0), Some(10)),
        helper_case("trigger force maximum", vec![("trigger_force", "250")], Some(100.0), Some(10)),
        helper_case("trigger force fraction", vec![("trigger_force", "99.5")], Some(100.0), Some(10)),
        helper_case("trigger force too small", vec![("trigger_force", "9.5")], Some(100.0), Some(10)),
        helper_case("trigger force too large", vec![("trigger_force", "250.5")], Some(100.0), Some(10)),
        helper_case("safety limit", vec![("force_safety_limit", "500")], Some(100.0), Some(10)),
        helper_case("safety limit minimum", vec![("force_safety_limit", "100")], Some(100.0), Some(10)),
        helper_case("safety limit maximum", vec![("force_safety_limit", "5000")], Some(100.0), Some(10)),
        helper_case("safety limit too small", vec![("force_safety_limit", "99")], Some(100.0), Some(10)),
        helper_case("safety limit too large", vec![("force_safety_limit", "5001")], Some(100.0), Some(10)),
        helper_case(
            "all options",
            vec![("tare_time", "0.25"), ("trigger_force", "50"), ("force_safety_limit", "1500")],
            Some(333.25),
            Some(-777),
        ),
    ];
    cases.push(HelperCase {
        label: "narrow sensor range",
        opts: vec![],
        sps: 80.0,
        range: (-100000, 100000),
        counts_per_gram: Some(100.0),
        reference_tare_counts: Some(0),
    });
    cases.push(HelperCase {
        label: "narrow sensor range, small limit",
        opts: vec![("force_safety_limit", "500")],
        sps: 80.0,
        range: (-100000, 100000),
        counts_per_gram: Some(100.0),
        reference_tare_counts: Some(0),
    });
    cases.push(HelperCase {
        label: "slow sensor",
        opts: vec![],
        sps: 1.0,
        range: FULL_RANGE,
        counts_per_gram: Some(10.0),
        reference_tare_counts: Some(0),
    });
    cases.push(HelperCase {
        label: "fast sensor",
        opts: vec![("tare_time", "1")],
        sps: 1000.0,
        range: FULL_RANGE,
        counts_per_gram: Some(10.0),
        reference_tare_counts: Some(0),
    });
    let gcmds: Vec<Params> = vec![
        vec![("TARE_TIME", "0.5")],
        vec![("TARE_TIME", "0.005")],
        vec![("TARE_TIME", "2")],
        vec![("TARE_TIME", "x")],
        vec![("TARE_TIME", "nan")],
        vec![("TRIGGER_FORCE", "100")],
        vec![("TRIGGER_FORCE", "5")],
        vec![("TRIGGER_FORCE", "300")],
        vec![("TRIGGER_FORCE", "75.5")],
        vec![("TRIGGER_FORCE", "nan")],
        vec![("FORCE_SAFETY_LIMIT", "500")],
        vec![("FORCE_SAFETY_LIMIT", "50")],
        vec![("FORCE_SAFETY_LIMIT", "6000")],
        vec![("FORCE_SAFETY_LIMIT", "5000")],
        vec![("FORCE_SAFETY_LIMIT", "nan")],
        vec![("FORCE_SAFETY_LIMIT", "x")],
        vec![("TARE_TIME", "0.1"), ("TRIGGER_FORCE", "20"), ("FORCE_SAFETY_LIMIT", "300")],
    ];
    for case in &cases {
        let mut opts: Vec<(&str, &str)> = case.opts.to_vec();
        let cpg_text = case.counts_per_gram.map(float_repr);
        let rtc_text = case.reference_tare_counts.map(|v| v.to_string());
        if let Some(text) = &cpg_text {
            opts.push(("counts_per_gram", text));
        }
        if let Some(text) = &rtc_text {
            opts.push(("reference_tare_counts", text));
        }
        let label = format!("{} {:?}", case.label, case.opts);
        run(
            &mut py,
            &format!(
                "cfg = mkcfg({})\nH = None\nR_CTOR = None\ntry:\n    H = lcp.LoadCellProbeConfigHelper(cfg, FakeLoadCell(SpsSensor({}, ({}, {})), {}, {}))\n    R_CTOR = 'ok'\nexcept Exception as e:\n    R_CTOR = 'raised %s: %s' % (type(e).__name__, e)\n",
                py_dict(&opts),
                py_f(case.sps),
                case.range.0,
                case.range.1,
                py_opt_f(case.counts_per_gram),
                py_opt_i(case.reference_tare_counts),
            ),
        );
        let gcode: ObjectRef = Arc::new(FakeGCode::new());
        let webhooks: ObjectRef = Arc::new(FakeWebhooks);
        let config =
            FakeConfig::make_with(&opts, printer_with(vec![("gcode", gcode), ("webhooks", webhooks)]));
        let sensor: Arc<dyn BulkSensorAdc> =
            Arc::new(SpsSensor { sps: case.sps, range: case.range, mcu: mcu.clone() });
        let load_cell = LoadCell::new(&config, sensor).expect("load cell");
        let built = LoadCellProbeConfigHelper::new(&config, &load_cell);
        let rust_ctor = match &built {
            Ok(_) => "ok".to_string(),
            Err(e) => format!("raised {}: {}", e.class_name(), e.message()),
        };
        compare(&mut py, &format!("{label} constructor"), "R_CTOR", rust_ctor);
        let Ok(helper) = built else { continue };
        compare(
            &mut py,
            &format!("{label} get_grams_per_count"),
            "res(lambda: repr(H.get_grams_per_count()))",
            res(helper.get_grams_per_count(), float_repr),
        );
        let mut calls: Vec<Option<&Params>> = vec![None];
        calls.extend(gcmds.iter().map(Some));
        for params in calls {
            let (py_arg, gcmd) = match params {
                None => (String::new(), None),
                Some(p) => (format!("mkcmd({})", py_dict(p)), Some(FakeGcmd::make(p))),
            };
            let label = format!("{label} gcmd {params:?}");
            let render_num = |v: Option<Num>| v.map_or("None".to_string(), Num::py_str);
            compare(
                &mut py,
                &format!("{label} get_tare_samples"),
                &format!("res(lambda: repr(H.get_tare_samples({py_arg})))"),
                res(helper.get_tare_samples(gcmd.as_ref()), |v| v.to_string()),
            );
            compare(
                &mut py,
                &format!("{label} get_trigger_force_grams"),
                &format!("res(lambda: repr(H.get_trigger_force_grams({py_arg})))"),
                res(helper.get_trigger_force_grams(gcmd.as_ref()), render_num),
            );
            compare(
                &mut py,
                &format!("{label} get_safety_limit_grams"),
                &format!("res(lambda: repr(H.get_safety_limit_grams({py_arg})))"),
                res(helper.get_safety_limit_grams(gcmd.as_ref()), render_num),
            );
            compare(
                &mut py,
                &format!("{label} get_safety_range"),
                &format!("res(lambda: repr(H.get_safety_range({py_arg})))"),
                res(helper.get_safety_range(gcmd.as_ref()), |(lo, hi)| format!("({lo}, {hi})")),
            );
        }
    }
}

/// One scripted answer of `probing_test`.
enum Step {
    Trigger(f64),
    Fail(&'static str),
}

fn py_steps(steps: &[Step]) -> String {
    let items: Vec<String> = steps
        .iter()
        .map(|s| match s {
            Step::Trigger(t) => py_f(*t),
            Step::Fail(m) => format!("CommandError('{m}')"),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

/// Registers `LoadCellProbeCommands` on a fake `gcode`, runs `LOAD_CELL_TEST_TAP` and renders
/// the registrations, the `respond_info` and `probing_test` calls and the outcome.
fn rust_test_tap(params: &[(&str, &str)], steps: Vec<Step>) -> String {
    let gcode = Arc::new(FakeGCode::new());
    let config = FakeConfig::make_with(
        &[],
        printer_with(vec![("gcode", gcode.clone() as ObjectRef)]),
    );
    let log: Log = new_log();
    let script: Arc<Mutex<VecDeque<Step>>> = Arc::new(Mutex::new(steps.into()));
    let probing_test: ProbingTestCb = {
        let log = log.clone();
        Arc::new(move |_gcmd, timeout| {
            log.lock().push(format!("probing_test timeout={}", float_repr(timeout)));
            match script.lock().pop_front().expect("scripted probing_test result") {
                Step::Trigger(time) => Ok(time),
                Step::Fail(message) => Err(KlippyError::Command(message.to_string())),
            }
        })
    };
    LoadCellProbeCommands::new(&config, probing_test).expect("commands");
    let registered = {
        let commands = gcode.commands.lock();
        let items: Vec<String> = commands
            .iter()
            .map(|(name, (desc, _))| {
                format!("('{name}', {})", desc.as_ref().map_or("None".to_string(), |d| format!("'{d}'")))
            })
            .collect();
        format!("[{}]", items.join(", "))
    };
    let gcmd = FakeGcmd::make_logged(params, &log);
    let outcome = {
        let mut commands = gcode.commands.lock();
        let (_, handler) = commands.get_mut("LOAD_CELL_TEST_TAP").expect("LOAD_CELL_TEST_TAP is registered");
        res(handler(&gcmd), |_| "None".to_string())
    };
    let lines = log.lock().join("\n");
    format!("{registered}\n{lines}\n{outcome}")
}

#[test]
fn load_cell_test_tap_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    run(
        &mut py,
        "def run_test_tap(params, results):
    del LOG[:]
    gcode = RecGcode()
    PRINTER.objects['gcode'] = gcode
    lcp.LoadCellProbeCommands(mkcfg({}), ScriptedMove(results))
    registered = repr(sorted((k, v[1]) for k, v in gcode.cmds.items()))
    func = gcode.cmds['LOAD_CELL_TEST_TAP'][0]
    outcome = res(lambda: func(mkcmd(params)))
    return registered + '\\n' + '\\n'.join(LOG) + '\\n' + outcome
",
    );
    let ok = |n: usize| (0..n).map(|i| Step::Trigger(1.5 + i as f64)).collect::<Vec<_>>();
    let cases: Vec<(&str, Params, Vec<Step>)> = vec![
        ("defaults", vec![], ok(3)),
        ("one tap", vec![("TAPS", "1")], ok(1)),
        ("four taps with timeout", vec![("TAPS", "4"), ("TIMEOUT", "5")], ok(4)),
        ("timeout minimum", vec![("TAPS", "1"), ("TIMEOUT", "1")], ok(1)),
        ("timeout maximum", vec![("TAPS", "1"), ("TIMEOUT", "120")], ok(1)),
        ("timeout fraction", vec![("TAPS", "1"), ("TIMEOUT", "2.5")], ok(1)),
        ("no taps", vec![("TAPS", "0")], ok(1)),
        ("too many taps", vec![("TAPS", "11")], ok(1)),
        ("taps unparsable", vec![("TAPS", "x")], ok(1)),
        ("taps float", vec![("TAPS", "2.5")], ok(1)),
        ("timeout too small", vec![("TIMEOUT", "0.5")], ok(1)),
        ("timeout too large", vec![("TIMEOUT", "121")], ok(1)),
        ("timeout unparsable", vec![("TIMEOUT", "x")], ok(1)),
        ("timeout on the first tap", vec![("TAPS", "3")], vec![Step::Trigger(0.0)]),
        ("timeout on the second tap", vec![("TAPS", "3")], vec![Step::Trigger(1.0), Step::Trigger(0.0)]),
        ("probing error", vec![("TAPS", "2")], vec![Step::Fail("Load Cell not calibrated")]),
        (
            "probing error on the second tap",
            vec![("TAPS", "3")],
            vec![Step::Trigger(2.0), Step::Fail("Communication timeout during homing")],
        ),
    ];
    for (label, params, steps) in cases {
        compare(
            &mut py,
            label,
            &format!("run_test_tap({}, {})", py_dict(&params), py_steps(&steps)),
            rust_test_tap(&params, steps),
        );
    }
}

/// One scripted answer of `run_tap`.
enum TapStep {
    Tap(Vec<f64>, bool),
    Fail(&'static str),
}

#[derive(Clone, Copy)]
enum Op {
    Start,
    Run,
    Pull,
    End,
}

struct FixedOffsets((f64, f64, f64));

impl ProbeOffsetsHelperApi for FixedOffsets {
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> (f64, f64, f64) {
        self.0
    }
}

fn py_tap_steps(steps: &[TapStep]) -> String {
    let items: Vec<String> = steps
        .iter()
        .map(|s| match s {
            TapStep::Tap(pos, good) => format!(
                "([{}], {})",
                pos.iter().map(|v| py_f(*v)).collect::<Vec<_>>().join(", "),
                if *good { "True" } else { "False" }
            ),
            TapStep::Fail(m) => format!("CommandError('{m}')"),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

fn py_ops(ops: &[Op]) -> String {
    let items: Vec<&str> = ops
        .iter()
        .map(|op| match op {
            Op::Start => "'start'",
            Op::Run => "'run'",
            Op::Pull => "'pull'",
            Op::End => "'end'",
        })
        .collect();
    format!("[{}]", items.join(", "))
}

type SessionCase = (&'static str, Vec<TapStep>, (f64, f64, f64), Vec<Op>);

fn rust_session(steps: Vec<TapStep>, offsets: (f64, f64, f64), ops: &[Op]) -> String {
    let log: Log = new_log();
    let script: Arc<Mutex<VecDeque<TapStep>>> = Arc::new(Mutex::new(steps.into()));
    let run_tap: RunTapCb = {
        let log = log.clone();
        Arc::new(move |_gcmd| {
            log.lock().push("run_tap".to_string());
            match script.lock().pop_front().expect("scripted run_tap result") {
                TapStep::Tap(pos, good) => Ok((pos, good)),
                TapStep::Fail(message) => Err(KlippyError::Command(message.to_string())),
            }
        })
    };
    let session = Arc::new(TapSession::new(run_tap, Arc::new(FixedOffsets(offsets))));
    let mut out = Vec::new();
    for op in ops {
        let gcmd = FakeGcmd::make(&[]);
        out.push(match op {
            Op::Start => {
                let started = session.start_probe_session(&gcmd);
                let same = std::ptr::eq(
                    Arc::as_ptr(&started) as *const (),
                    Arc::as_ptr(&session) as *const (),
                );
                format!("ok {}", if same { "True" } else { "False" })
            }
            Op::Run => res(session.run_probe(&gcmd), |_| "None".to_string()),
            Op::Pull => res(session.pull_probed_results(), |results| {
                let items: Vec<String> = results
                    .iter()
                    .map(|r| {
                        format!(
                            "({})",
                            [r.bed_x, r.bed_y, r.bed_z, r.test_x, r.test_y, r.test_z]
                                .iter()
                                .map(|v| float_repr(*v))
                                .collect::<Vec<_>>()
                                .join(", ")
                        )
                    })
                    .collect();
                format!("[{}]", items.join(", "))
            }),
            Op::End => res(session.end_probe_session(), |_| "None".to_string()),
        });
    }
    out.extend(log.lock().iter().cloned());
    out.join("\n")
}

#[test]
fn tap_session_matches_python() {
    let Some(mut py) = oracle() else {
        eprintln!("KLIPPY_PYTHON is not set: skipping the load_cell_probe oracle test");
        return;
    };
    run(
        &mut py,
        "def run_session(runs, offs, ops):
    del LOG[:]
    sess = lcp.TapSession(mkcfg({}), ScriptedTap(runs), FakeOffsets(offs), None)
    out = []
    for op in ops:
        if op == 'start':
            out.append(res(lambda: sess.start_probe_session(mkcmd({})) is sess))
        elif op == 'run':
            out.append(res(lambda: sess.run_probe(mkcmd({}))))
        elif op == 'pull':
            out.append(res(lambda: repr([tuple(r) for r in sess.pull_probed_results()])))
        else:
            out.append(res(lambda: sess.end_probe_session()))
    return '\\n'.join(out + LOG)
",
    );
    let tap = |x: f64, y: f64, z: f64| TapStep::Tap(vec![x, y, z], true);
    let cases: Vec<SessionCase> = vec![
        (
            "two taps then pull",
            vec![tap(10.0, 20.0, -0.25), tap(11.5, 21.25, -0.5)],
            (1.5, -2.25, 0.125),
            vec![Op::Run, Op::Run, Op::Pull, Op::Pull],
        ),
        (
            "start, run, end, pull",
            vec![tap(1.0, 2.0, 3.0)],
            (0.0, 0.0, 0.0),
            vec![Op::Start, Op::Run, Op::End, Op::Pull],
        ),
        (
            "pull without a tap",
            vec![],
            (1.0, 1.0, 1.0),
            vec![Op::Pull, Op::End, Op::Pull],
        ),
        (
            "invalid tap is still stored",
            vec![TapStep::Tap(vec![5.0, 6.0, 7.5], false)],
            (0.5, 0.25, -1.0),
            vec![Op::Run, Op::Pull],
        ),
        (
            "failed tap stores nothing",
            vec![tap(1.0, 2.0, 3.0), TapStep::Fail("Probe triggered prior to movement"), tap(4.0, 5.0, 6.0)],
            (0.0, 0.0, 0.5),
            vec![Op::Run, Op::Run, Op::Run, Op::Pull],
        ),
        (
            "extra position values",
            vec![TapStep::Tap(vec![1.0, 2.0, 3.0, 4.0], true)],
            (0.1, 0.2, 0.3),
            vec![Op::Start, Op::Run, Op::Pull],
        ),
    ];
    for (label, steps, offsets, ops) in cases {
        compare(
            &mut py,
            label,
            &format!(
                "run_session({}, ({}, {}, {}), {})",
                py_tap_steps(&steps),
                py_f(offsets.0),
                py_f(offsets.1),
                py_f(offsets.2),
                py_ops(&ops)
            ),
            rust_session(steps, offsets, &ops),
        );
    }
}
