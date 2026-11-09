//! Parity tests for `extras::endstop_phase` (`klippy/extras/endstop_phase.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`: the frozen
//! `klippy/extras/endstop_phase.py` runs against the real `configfile.ConfigWrapper`
//! (over a `configparser.RawConfigParser` built from the same INI text the Rust fake
//! parses), the real `stepper.parse_step_distance` (wrapped only to record its arguments
//! and outcome) and small Python fakes for the printer, gcode, configfile, rails, steppers,
//! homing state and TMC drivers. The Rust port runs the same scenario through Rust fakes of
//! the contract traits. After construction and after every step (`klippy:connect`,
//! `homing:home_rails_end`, G-code commands, TMC offset changes) both sides produce a
//! snapshot that is compared with `recording::assert_equal`: registered events, loaded
//! objects, registered commands (name, help, `when_not_ready`), every config read with its
//! default and bounds, the access-tracking dict (what `note_valid` leaves behind), G-code
//! responses, `gcmd.get` calls, `configfile` calls, log lines, the `parse_step_distance`
//! calls, the stepper adjustments set on the homing state, every `EndstopPhase` attribute
//! (floats by bits), every `PhaseCalc` attribute and the `EndstopPhases.get_status` dict.
//! Raised errors (class and message) are compared for every step.
//!
//! The Rust fake of `stepper.parse_step_distance` replays the outcome the real Python
//! function produced for the same section, because that function belongs to the core
//! `stepper` unit, not to this module.

use extras::endstop_phase::{module_factory, EndstopPhase, EndstopPhases, PhaseCalc};
use extras::tmc::TMCCommandHelperApi;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatListOpts,
    GetFloatOpts, GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::log;
use klippy_api::mcu::{EndstopApi, Mcu};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{
    HomingInfo, McuStepperApi, PullHistoryStep, Rail, RailApi, RailOpts, Stepper, StepperFactory,
    StepperFactoryApi, StepperKinematics, Trapq,
};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateApi, HomingTargetApi, Kinematics, LookaheadCallback, PrinterExtruder,
    ToolHeadApi,
};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use pycompat::text::{float_from_string, float_repr, int_from_string, str_repr};
use pycompat::PyValue;
use reactor::{Completion, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::cell::RefCell;
use std::collections::VecDeque;
use std::sync::{Arc, LazyLock};

// ---------------------------------------------------------------------------
// Python side.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import collections
import configparser
import logging

import configfile
import stepper as real_stepper
import extras.endstop_phase as ep_mod

log_messages = []
class ListHandler(logging.Handler):
    def emit(self, record):
        log_messages.append(record.getMessage())
logging.getLogger().addHandler(ListHandler())
logging.getLogger().setLevel(logging.INFO)

class CommandError(Exception):
    pass

_sentinel = object()

# stepper.parse_step_distance is the core stepper unit: run the real function on a plain
# ConfigWrapper (its reads are not part of this module) and record call and outcome.
psd_calls = []
class StepperProxy:
    def parse_step_distance(self, config, units_in_radians=None, note_valid=False):
        plain = configfile.ConfigWrapper(config.printer, config.fileconfig,
                                         config.access_tracking, config.section)
        try:
            r = real_stepper.parse_step_distance(plain, units_in_radians, note_valid)
        except Exception as e:
            psd_calls.append((config.get_name(), units_in_radians, note_valid,
                              ('err', type(e).__name__, str(e))))
            raise
        psd_calls.append((config.get_name(), units_in_radians, note_valid,
                          ('ok', r[0], r[1])))
        return r
ep_mod.stepper = StepperProxy()

def psd_outcomes(start):
    out = []
    for c in psd_calls[start:]:
        r = c[3]
        if r[0] == 'ok':
            out.append(('ok', r[1].hex(), r[2].hex()))
        else:
            out.append(('err', r[1], r[2]))
    return out

class LoggedConfig(configfile.ConfigWrapper):
    def __init__(self, printer, fileconfig, access_tracking, section, log):
        configfile.ConfigWrapper.__init__(self, printer, fileconfig, access_tracking, section)
        self.log = log
    def _rec(self, method, option, default, minval, maxval, above, below, note_valid, sep, count):
        d = 'sentinel' if default is configfile.sentinel else repr(default)
        self.log.append((method, self.section, option, d, repr(minval), repr(maxval),
                         repr(above), repr(below), note_valid, repr(sep), repr(count)))
    def get(self, option, default=configfile.sentinel, note_valid=True):
        self._rec('get', option, default, None, None, None, None, note_valid, None, None)
        return configfile.ConfigWrapper.get(self, option, default, note_valid)
    def getint(self, option, default=configfile.sentinel, minval=None, maxval=None,
               note_valid=True):
        self._rec('getint', option, default, minval, maxval, None, None, note_valid, None, None)
        return configfile.ConfigWrapper.getint(self, option, default, minval, maxval, note_valid)
    def getfloat(self, option, default=configfile.sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        self._rec('getfloat', option, default, minval, maxval, above, below, note_valid,
                  None, None)
        return configfile.ConfigWrapper.getfloat(self, option, default, minval, maxval,
                                                 above, below, note_valid)
    def getboolean(self, option, default=configfile.sentinel, note_valid=True):
        self._rec('getboolean', option, default, None, None, None, None, note_valid, None, None)
        return configfile.ConfigWrapper.getboolean(self, option, default, note_valid)
    def getintlist(self, option, default=configfile.sentinel, sep=',', count=None,
                   note_valid=True):
        self._rec('getintlist', option, default, None, None, None, None, note_valid, sep, count)
        return configfile.ConfigWrapper.getintlist(self, option, default, sep, count, note_valid)
    def getsection(self, section):
        return LoggedConfig(self.printer, self.fileconfig, self.access_tracking, section,
                            self.log)

class FakeTmc:
    def __init__(self, offset, phases):
        self.offset = offset
        self.phases = phases
    def get_phase_offset(self):
        return (self.offset, self.phases)

class FakeGCode:
    error = CommandError
    def __init__(self):
        self.registered = []
        self.handlers = {}
        self.responded = []
        self.gets = []
    def register_command(self, cmd, func=None, desc=None, when_not_ready=False):
        self.registered.append((cmd, desc, when_not_ready, func is not None))
        self.handlers[cmd] = func
    def respond_info(self, msg, log=True):
        if log:
            logging.info(msg)
        self.responded.append((msg, log))

class FakeGcmd:
    error = CommandError
    def __init__(self, gcode, command, params):
        self.gcode = gcode
        self.command = command
        self.params = params
    def get(self, name, default=_sentinel, parser=str, minval=None, maxval=None,
            above=None, below=None):
        self.gcode.gets.append((name, 'sentinel' if default is _sentinel else repr(default)))
        if name in self.params:
            return parser(self.params[name])
        if default is _sentinel:
            raise CommandError("Error on '%s': missing %s" % (self.command, name))
        return default
    def respond_info(self, msg, log=True):
        self.gcode.respond_info(msg, log)

class FakeConfigFile:
    def __init__(self):
        self.calls = []
    def remove_section(self, section):
        self.calls.append(('remove_section', section))
    def set(self, section, option, value):
        self.calls.append(('set', section, option, value))

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, start_args):
        self.objects = {}
        self.handlers = {}
        self.events = []
        self.load_calls = []
        self.start_args = start_args
    def get_start_args(self):
        return self.start_args
    def register_event_handler(self, event, cb):
        self.events.append(event)
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        return [cb(*params) for cb in list(self.handlers.get(event, []))]
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=configfile.sentinel):
        self.load_calls.append(section)
        if section in self.objects:
            return self.objects[section]
        if section == 'endstop_phase':
            init_func = ep_mod.load_config
        elif section.startswith('endstop_phase '):
            init_func = ep_mod.load_config_prefix
        else:
            raise self.config_error("Unable to load module '%s'" % (section,))
        self.objects[section] = init_func(config.getsection(section))
        return self.objects[section]

class FakeStepper:
    def __init__(self, name):
        self.name = name
    def get_name(self, short=False):
        return self.name

class FakeEndstop:
    def __init__(self, steppers):
        self.steppers = steppers
    def get_steppers(self):
        return [FakeStepper(n) for n in self.steppers]

homing_info = collections.namedtuple('homing_info',
    ['speed', 'position_endstop', 'retract_speed', 'retract_dist', 'positive_dir',
     'second_homing_speed'])

class FakeRail:
    def __init__(self, steppers, endstop_steppers, position_endstop):
        self.steppers = steppers
        self.endstop_steppers = endstop_steppers
        self.position_endstop = position_endstop
    def get_steppers(self):
        return [FakeStepper(n) for n in self.steppers]
    def get_endstops(self):
        if self.endstop_steppers is None:
            return []
        return [(FakeEndstop(self.endstop_steppers), 'endstop')]
    def get_homing_info(self):
        return homing_info(10., self.position_endstop, 5., 2., False, 5.)

class FakeHoming:
    def __init__(self, triggers):
        self.triggers = triggers
        self.adjust = []
    def get_trigger_position(self, name):
        return self.triggers[name]
    def set_stepper_adjustment(self, name, adjustment):
        self.adjust.append((name, adjustment))

p = None
gcode = None
cf = None
fileconfig = None
access = None
reads = None
last_hs = [None]

def setup(ini, debugoutput, tmcs):
    global p, gcode, cf, fileconfig, access, reads
    del log_messages[:]
    del psd_calls[:]
    fileconfig = configparser.RawConfigParser()
    fileconfig.read_string(ini)
    access = {}
    reads = []
    last_hs[0] = None
    p = FakePrinter({'debugoutput': 'out.bin'} if debugoutput else {})
    gcode = FakeGCode()
    cf = FakeConfigFile()
    p.objects['gcode'] = gcode
    p.objects['configfile'] = cf
    for name, offset, phases in tmcs:
        p.objects[name] = FakeTmc(offset, phases)

def section(name):
    return LoggedConfig(p, fileconfig, access, name, reads)

def load(name):
    p.load_object(section(''), name)

def send(event, *params):
    return p.send_event(event, *params)

def home(rails, triggers):
    hs = FakeHoming(triggers)
    last_hs[0] = hs
    return p.send_event('homing:home_rails_end', hs, rails)

def run_cmd(cmd, params):
    gcode.handlers[cmd](FakeGcmd(gcode, cmd, params))

def set_tmc(name, offset):
    p.objects[name].offset = offset

def snap_pc(pc):
    return {'name': pc.name, 'phases': pc.phases, 'tmc': pc.tmc_module is not None,
            'history': pc.phase_history, 'last_phase': pc.last_phase,
            'last_mcu_position': pc.last_mcu_position, 'is_primary': pc.is_primary,
            'stats_only': pc.stats_only}

def snap_ep(ep):
    return {'name': ep.name, 'step_dist': ep.step_dist, 'phases': ep.phases,
            'endstop_phase': ep.endstop_phase, 'align_zero': ep.endstop_align_zero,
            'accuracy': ep.endstop_accuracy, 'phase_accuracy': ep.endstop_phase_accuracy,
            'calc': snap_pc(ep.phase_calc)}

def snap_eps(e):
    return {'tracking': [(k, snap_pc(v)) for k, v in e.tracking.items()],
            'status': e.get_status(0.)}

def snap_env():
    return {'events': list(p.events), 'loads': list(p.load_calls),
            'commands': list(gcode.registered), 'reads': list(reads),
            'access': [(k[0], k[1], repr(v)) for k, v in access.items()],
            'responded': list(gcode.responded), 'gets': list(gcode.gets),
            'configfile': list(cf.calls), 'log': list(log_messages),
            'psd': [(c[0], repr(c[1]), c[2]) for c in psd_calls],
            'adjust': [] if last_hs[0] is None else list(last_hs[0].adjust)}

def snap_all():
    return {'env': snap_env(),
            'eps': [snap_eps(v) for k, v in p.objects.items() if k == 'endstop_phase'],
            'ep': [(k, snap_ep(v)) for k, v in p.objects.items()
                   if k.startswith('endstop_phase ')]}
"#;

// ---------------------------------------------------------------------------
// Value helpers.
// ---------------------------------------------------------------------------

fn s(x: &str) -> Value {
    Value::str(x)
}

fn list(v: Vec<Value>) -> Value {
    Value::List(v)
}

fn tup(v: Vec<Value>) -> Value {
    Value::Tuple(v)
}

fn dict(v: Vec<(&str, Value)>) -> Value {
    Value::Dict(v.into_iter().map(|(k, v)| (Value::str(k), v)).collect())
}

fn opt_int(v: Option<i64>) -> Value {
    match v {
        Some(n) => Value::int(n),
        None => Value::Null,
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.0.to_string().parse::<i64>() {
            Ok(i) => Value::Int(i),
            Err(_) => Value::BigInt(n.0.to_string()),
        },
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(x) => Value::str(x),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(_) => panic!("host values are not used by endstop_phase"),
    }
}

fn py_str(x: &str) -> String {
    let mut out = String::from("'");
    for c in x.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            _ => out.push(c),
        }
    }
    out.push('\'');
    out
}

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_float(v: f64) -> String {
    assert!(v.is_finite());
    format!("{v:?}")
}

fn py_opt_int(v: Option<i64>) -> String {
    match v {
        Some(n) => n.to_string(),
        None => "None".to_string(),
    }
}

fn bool_repr(b: bool) -> String {
    py_bool(b).to_string()
}

// ---------------------------------------------------------------------------
// Log capture (the process-wide sink writes into a thread-local, one per test thread).
// ---------------------------------------------------------------------------

thread_local! {
    static LOG: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

static SINK: LazyLock<()> = LazyLock::new(|| {
    let _ = log::set_sink(Box::new(|rec| {
        LOG.with(|l| l.borrow_mut().push(rec.message.to_string()));
    }));
});

fn install_sink() {
    LazyLock::force(&SINK);
    LOG.with(|l| l.borrow_mut().clear());
}

fn captured_log() -> Vec<String> {
    LOG.with(|l| l.borrow().clone())
}

// ---------------------------------------------------------------------------
// Rust fakes: config.
// ---------------------------------------------------------------------------

/// `[section] -> option -> value`, the parsed INI text both sides read.
#[derive(Clone, Default)]
struct Ini {
    sections: Vec<(String, Vec<(String, String)>)>,
}

impl Ini {
    fn new() -> Ini {
        Ini::default()
    }

    fn section(mut self, name: &str, options: &[(&str, &str)]) -> Ini {
        self.sections.push((
            name.to_string(),
            options.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        ));
        self
    }

    fn text(&self) -> String {
        let mut out = String::new();
        for (name, options) in &self.sections {
            out.push_str(&format!("[{name}]\n"));
            for (k, v) in options {
                out.push_str(&format!("{k}: {v}\n"));
            }
            out.push('\n');
        }
        out
    }
}

struct CfgShared {
    sections: IndexMap<String, IndexMap<String, String>>,
    reads: GCell<Vec<Value>>,
    access: GCell<IndexMap<(String, String), String>>,
}

struct FakeConfig {
    printer: Printer,
    shared: Arc<CfgShared>,
    section: String,
}

fn opt_repr<T>(v: &Option<T>, repr: &dyn Fn(&T) -> String) -> String {
    match v {
        Some(x) => repr(x),
        None => "None".to_string(),
    }
}

fn int_repr(v: &i64) -> String {
    v.to_string()
}

fn float_r(v: &f64) -> String {
    float_repr(*v)
}

#[allow(clippy::ptr_arg)]
fn tuple_repr(v: &Vec<i64>) -> String {
    let parts: Vec<String> = v.iter().map(|x| x.to_string()).collect();
    if parts.len() == 1 {
        format!("({},)", parts[0])
    } else {
        format!("({})", parts.join(", "))
    }
}

#[allow(clippy::ptr_arg)]
fn str_r(v: &String) -> String {
    str_repr(v)
}

impl FakeConfig {
    #[allow(clippy::too_many_arguments)]
    fn rec(
        &self,
        method: &str,
        option: &str,
        default: String,
        minval: String,
        maxval: String,
        above: String,
        below: String,
        note_valid: bool,
        sep: String,
        count: String,
    ) {
        self.shared.reads.borrow_mut().push(tup(vec![
            s(method),
            s(&self.section),
            s(option),
            Value::Str(default),
            Value::Str(minval),
            Value::Str(maxval),
            Value::Str(above),
            Value::Str(below),
            Value::Bool(note_valid),
            Value::Str(sep),
            Value::Str(count),
        ]));
    }

    fn unable(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.section
        ))
    }

    fn track(&self, option: &str, repr: String) {
        self.shared
            .access
            .borrow_mut()
            .insert((self.section.to_lowercase(), option.to_lowercase()), repr);
    }

    /// `ConfigWrapper._get_wrapper` (`klippy/configfile.py:35-60`).
    #[allow(clippy::too_many_arguments)]
    fn get_wrapper<T: PartialOrd + Clone>(
        &self,
        option: &str,
        default: &Option<Option<T>>,
        parse: impl FnOnce(&str) -> KResult<T>,
        minval: &Option<T>,
        maxval: &Option<T>,
        above: &Option<T>,
        below: &Option<T>,
        note_valid: bool,
        repr: &dyn Fn(&T) -> String,
    ) -> KResult<Option<T>> {
        let raw = self
            .shared
            .sections
            .get(&self.section)
            .and_then(|options| options.get(option))
            .cloned();
        let raw = match raw {
            Some(raw) => raw,
            None => {
                return match default {
                    Some(inner) => {
                        if note_valid {
                            if let Some(v) = inner {
                                self.track(option, repr(v));
                            }
                        }
                        Ok(inner.clone())
                    }
                    None => Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    ))),
                };
            }
        };
        let v = parse(&raw)?;
        if note_valid {
            self.track(option, repr(&v));
        }
        let section = &self.section;
        if let Some(m) = minval {
            if v < *m {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have minimum of {}",
                    repr(m)
                )));
            }
        }
        if let Some(m) = maxval {
            if v > *m {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must have maximum of {}",
                    repr(m)
                )));
            }
        }
        if let Some(m) = above {
            if v <= *m {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be above {}",
                    repr(m)
                )));
            }
        }
        if let Some(m) = below {
            if v >= *m {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{section}' must be below {}",
                    repr(m)
                )));
            }
        }
        Ok(Some(v))
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
        self.rec(
            "get",
            option,
            match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => opt_repr(d, &str_r),
            },
            "None".into(),
            "None".into(),
            "None".into(),
            "None".into(),
            opts.note_valid,
            "None".into(),
            "None".into(),
        );
        self.get_wrapper(
            option,
            &opts.default,
            |raw| Ok(raw.to_string()),
            &None,
            &None,
            &None,
            &None,
            opts.note_valid,
            &str_r,
        )
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.rec(
            "getint",
            option,
            match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => opt_repr(d, &int_repr),
            },
            opt_repr(&opts.minval, &int_repr),
            opt_repr(&opts.maxval, &int_repr),
            "None".into(),
            "None".into(),
            opts.note_valid,
            "None".into(),
            "None".into(),
        );
        let unable = self.unable(option);
        self.get_wrapper(
            option,
            &opts.default,
            |raw| {
                int_from_string(raw, 10)
                    .ok()
                    .and_then(|n| n.0.to_string().parse::<i64>().ok())
                    .ok_or(unable)
            },
            &opts.minval,
            &opts.maxval,
            &None,
            &None,
            opts.note_valid,
            &int_repr,
        )
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.rec(
            "getfloat",
            option,
            match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => opt_repr(d, &float_r),
            },
            opt_repr(&opts.minval, &float_r),
            opt_repr(&opts.maxval, &float_r),
            opt_repr(&opts.above, &float_r),
            opt_repr(&opts.below, &float_r),
            opts.note_valid,
            "None".into(),
            "None".into(),
        );
        let unable = self.unable(option);
        self.get_wrapper(
            option,
            &opts.default,
            |raw| float_from_string(raw).map_err(|_| unable),
            &opts.minval,
            &opts.maxval,
            &opts.above,
            &opts.below,
            opts.note_valid,
            &float_r,
        )
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.rec(
            "getboolean",
            option,
            match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => opt_repr(d, &|b: &bool| bool_repr(*b)),
            },
            "None".into(),
            "None".into(),
            "None".into(),
            "None".into(),
            opts.note_valid,
            "None".into(),
            "None".into(),
        );
        let unable = self.unable(option);
        self.get_wrapper(
            option,
            &opts.default,
            |raw| match raw.to_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Ok(true),
                "0" | "no" | "false" | "off" => Ok(false),
                _ => Err(unable),
            },
            &None,
            &None,
            &None,
            &None,
            opts.note_valid,
            &|b: &bool| bool_repr(*b),
        )
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice is not used by endstop_phase")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by endstop_phase")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by endstop_phase")
    }
    fn getintlist(&self, option: &str, opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        self.rec(
            "getintlist",
            option,
            match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => opt_repr(d, &tuple_repr),
            },
            "None".into(),
            "None".into(),
            "None".into(),
            "None".into(),
            opts.note_valid,
            str_repr(&opts.sep),
            match opts.count {
                Some(c) => c.to_string(),
                None => "None".to_string(),
            },
        );
        let unable = self.unable(option);
        let section = self.section.clone();
        let sep = opts.sep.clone();
        let count = opts.count;
        let option_owned = option.to_string();
        self.get_wrapper(
            option,
            &opts.default,
            move |raw| {
                let parts: Vec<&str> = if raw.trim().is_empty() {
                    Vec::new()
                } else {
                    raw.split(sep.as_str()).map(|p| p.trim()).collect()
                };
                let mut res = Vec::new();
                for p in parts {
                    match int_from_string(p, 10)
                        .ok()
                        .and_then(|n| n.0.to_string().parse::<i64>().ok())
                    {
                        Some(n) => res.push(n),
                        None => return Err(unable),
                    }
                }
                if let Some(count) = count {
                    if res.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option_owned}' in section '{section}' must have {count} elements"
                        )));
                    }
                }
                Ok(res)
            },
            &None,
            &None,
            &None,
            &None,
            opts.note_valid,
            &tuple_repr,
        )
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by endstop_phase")
    }
    fn getsection(&self, section: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone(),
            shared: Arc::clone(&self.shared),
            section: section.to_string(),
        })
    }
    fn has_section(&self, _section: &str) -> bool {
        panic!("has_section is not used by endstop_phase")
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        panic!("get_prefix_sections is not used by endstop_phase")
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        panic!("get_prefix_options is not used by endstop_phase")
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {
        panic!("deprecate is not used by endstop_phase")
    }
}

// ---------------------------------------------------------------------------
// Rust fakes: printer, objects, gcode.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<EventCallback>>>,
    events: GCell<Vec<String>>,
    loads: GCell<Vec<String>>,
    debugoutput: bool,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        let mut args = IndexMap::new();
        if self.debugoutput {
            args.insert(PyValue::Str("debugoutput".to_string()), PyValue::Str("out.bin".to_string()));
        }
        PyValue::Dict(args)
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
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("add_object is not used by endstop_phase")
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self.objects.borrow().get(name).cloned();
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by endstop_phase")
    }
    fn load_object(
        &self,
        config: &Config,
        section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.loads.borrow_mut().push(section.to_string());
        let existing = self.objects.borrow().get(section).cloned();
        if let Some(obj) = existing {
            return Ok(Some(obj));
        }
        let factory = module_factory();
        let init = if section == "endstop_phase" {
            factory.load_config
        } else if section.starts_with("endstop_phase ") {
            factory.load_config_prefix
        } else {
            None
        };
        let init = match init {
            Some(init) => init,
            None => {
                return Err(KlippyError::config(format!("Unable to load module '{section}'")))
            }
        };
        let obj = init(&config.getsection(section))?
            .expect("endstop_phase factories always build an object");
        self.objects.borrow_mut().insert(section.to_string(), obj.clone());
        Ok(Some(obj))
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.events.borrow_mut().push(event.to_string());
        self.handlers
            .borrow_mut()
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut callbacks = self
            .handlers
            .borrow_mut()
            .get_mut(event)
            .map(std::mem::take)
            .unwrap_or_default();
        let mut results = Vec::new();
        let mut failure = None;
        for cb in callbacks.iter_mut() {
            match cb(params) {
                Ok(v) => results.push(v),
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }
        let mut handlers = self.handlers.borrow_mut();
        let slot = handlers.entry(event.to_string()).or_default();
        callbacks.extend(std::mem::take(slot));
        *slot = callbacks;
        match failure {
            Some(e) => Err(e),
            None => Ok(results),
        }
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeTmc {
    offset: GCell<Option<i64>>,
    phases: i64,
}

impl TMCCommandHelperApi for FakeTmc {
    fn get_phase_offset(&self) -> (Option<i64>, i64) {
        (self.offset.get(), self.phases)
    }
}

klippy_api::printer_object!(FakeTmc; dyn TMCCommandHelperApi);

struct FakeConfigFile {
    calls: GCell<Vec<Value>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by endstop_phase")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config is not used by endstop_phase")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config is not used by endstop_phase")
    }
    fn log_config(&self, _config: &Config) {
        panic!("log_config is not used by endstop_phase")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("check_unused_options is not used by endstop_phase")
    }
    fn runtime_warning(&self, _msg: &str) {
        panic!("runtime_warning is not used by endstop_phase")
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("deprecate is not used by endstop_phase")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {
        panic!("deprecate_gcode is not used by endstop_phase")
    }
    fn deprecate_mcu_code(&self, _mcu: &Mcu, _feature: &str, _msg: Option<String>) {
        panic!("deprecate_mcu_code is not used by endstop_phase")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by endstop_phase")
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.calls
            .borrow_mut()
            .push(tup(vec![s("set"), s(section), s(option), s(value)]));
    }
    fn remove_section(&self, section: &str) {
        self.calls
            .borrow_mut()
            .push(tup(vec![s("remove_section"), s(section)]));
    }
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

struct FakeGCode {
    registered: GCell<Vec<Value>>,
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
    responded: GCell<Vec<Value>>,
    gets: GCell<Vec<Value>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        self.registered.borrow_mut().push(tup(vec![
            s(cmd),
            match &opts.desc {
                Some(d) => s(d),
                None => Value::Null,
            },
            Value::Bool(opts.when_not_ready),
            Value::Bool(func.is_some()),
        ]));
        match func {
            Some(handler) => Ok(self.handlers.borrow_mut().insert(cmd.to_string(), handler)),
            None => Ok(self.handlers.borrow_mut().shift_remove(cmd)),
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
        panic!("register_mux_command is not used by endstop_phase")
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used by endstop_phase")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by endstop_phase")
    }
    fn register_output_handler(&self, _callback: OutputCallback) {
        panic!("register_output_handler is not used by endstop_phase")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by endstop_phase")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by endstop_phase")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by endstop_phase")
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex is not used by endstop_phase")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by endstop_phase")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by endstop_phase")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        if log {
            klippy_api::log::info(msg);
        }
        self.responded
            .borrow_mut()
            .push(tup(vec![s(msg), Value::Bool(log)]));
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by endstop_phase")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by endstop_phase")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by endstop_phase")
    }
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

struct FakeGcmd {
    gcode: Arc<FakeGCode>,
    command: String,
    params: IndexMap<String, String>,
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GetOpts) -> KResult<PyValue> {
        self.gcode.gets.borrow_mut().push(tup(vec![
            s(name),
            Value::Str(match &opts.default {
                None => "sentinel".to_string(),
                Some(d) => d.repr(),
            }),
        ]));
        if let Some(v) = self.params.get(name) {
            return Ok(PyValue::Str(v.clone()));
        }
        match opts.default {
            Some(d) => Ok(d),
            None => Err(KlippyError::command(format!(
                "Error on '{}': missing {}",
                self.command, name
            ))),
        }
    }
    fn get_int(&self, _name: &str, _opts: GGetIntOpts) -> KResult<Option<i64>> {
        panic!("get_int is not used by endstop_phase")
    }
    fn get_float(&self, _name: &str, _opts: GGetFloatOpts) -> KResult<Option<f64>> {
        panic!("get_float is not used by endstop_phase")
    }
    fn get_command(&self) -> String {
        panic!("get_command is not used by endstop_phase")
    }
    fn get_commandline(&self) -> String {
        panic!("get_commandline is not used by endstop_phase")
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        panic!("get_command_parameters is not used by endstop_phase")
    }
    fn get_raw_command_parameters(&self) -> String {
        panic!("get_raw_command_parameters is not used by endstop_phase")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        GCodeApi::respond_info(&*self.gcode, msg, log);
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by endstop_phase")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("ack is not used by endstop_phase")
    }
}

// ---------------------------------------------------------------------------
// Rust fakes: toolhead and the stepper factory.
// ---------------------------------------------------------------------------

enum Psd {
    Ok(f64, f64),
    Err(KlippyError),
}

struct FakeStepperFactory {
    outcomes: GCell<VecDeque<Psd>>,
    calls: GCell<Vec<Value>>,
}

impl StepperFactoryApi for FakeStepperFactory {
    fn printer_stepper(&self, _config: &Config, _units_in_radians: bool) -> KResult<Stepper> {
        panic!("printer_stepper is not used by endstop_phase")
    }
    fn lookup_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_rail is not used by endstop_phase")
    }
    fn lookup_multi_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("lookup_multi_rail is not used by endstop_phase")
    }
    fn generic_rail(&self, _config: &Config, _opts: RailOpts) -> KResult<Rail> {
        panic!("generic_rail is not used by endstop_phase")
    }
    fn parse_step_distance(
        &self,
        config: &Config,
        units_in_radians: Option<bool>,
        note_valid: bool,
    ) -> KResult<(f64, f64)> {
        self.calls.borrow_mut().push(tup(vec![
            s(&config.get_name()),
            Value::Str(match units_in_radians {
                None => "None".to_string(),
                Some(b) => bool_repr(b),
            }),
            Value::Bool(note_valid),
        ]));
        match self.outcomes.borrow_mut().pop_front() {
            Some(Psd::Ok(rd, spr)) => Ok((rd, spr)),
            Some(Psd::Err(e)) => Err(e),
            None => panic!("no recorded parse_step_distance outcome for this call"),
        }
    }
}

struct FakeToolHead {
    factory: Arc<FakeStepperFactory>,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by endstop_phase")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by endstop_phase")
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("get_kinematics is not used by endstop_phase")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by endstop_phase")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by endstop_phase")
    }
    fn dwell(&self, _delay: f64) {
        panic!("dwell is not used by endstop_phase")
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &Completion<f64>,
    ) -> KResult<()> {
        panic!("drip_move is not used by endstop_phase")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by endstop_phase")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by endstop_phase")
    }
    fn wait_moves(&self) {
        panic!("wait_moves is not used by endstop_phase")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by endstop_phase")
    }
    fn get_trapq(&self) -> Trapq {
        panic!("get_trapq is not used by endstop_phase")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by endstop_phase")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by endstop_phase")
    }
    fn register_lookahead_callback(&self, _callback: LookaheadCallback) -> KResult<()> {
        panic!("register_lookahead_callback is not used by endstop_phase")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by endstop_phase")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by endstop_phase")
    }
    fn set_extruder(&self, _extruder: PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by endstop_phase")
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("get_extruder is not used by endstop_phase")
    }
    fn add_extra_axis(&self, _ea: ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by endstop_phase")
    }
    fn remove_extra_axis(&self, _ea: &ExtraAxis) {
        panic!("remove_extra_axis is not used by endstop_phase")
    }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        panic!("get_extra_axes is not used by endstop_phase")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by endstop_phase")
    }
    fn stepper_factory(&self) -> StepperFactory {
        self.factory.clone()
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

// ---------------------------------------------------------------------------
// Rust fakes: steppers, endstops, rails, homing state.
// ---------------------------------------------------------------------------

struct FakeStepper {
    name: String,
}

fn stepper_of(name: &str) -> Stepper {
    Arc::new(FakeStepper { name: name.to_string() })
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by endstop_phase")
    }
    fn get_name(&self, _short: Option<bool>) -> String {
        self.name.clone()
    }
    fn get_oid(&self) -> i64 {
        panic!("get_oid is not used by endstop_phase")
    }
    fn units_in_radians(&self) -> bool {
        panic!("units_in_radians is not used by endstop_phase")
    }
    fn get_pulse_duration(&self) -> (Option<f64>, bool) {
        panic!("get_pulse_duration is not used by endstop_phase")
    }
    fn setup_default_pulse_duration(&self, _pulse_duration: f64, _step_both_edge: bool) {
        panic!("setup_default_pulse_duration is not used by endstop_phase")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by endstop_phase")
    }
    fn get_step_dist(&self) -> f64 {
        panic!("get_step_dist is not used by endstop_phase")
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        panic!("get_rotation_distance is not used by endstop_phase")
    }
    fn set_rotation_distance(&self, _rotation_dist: f64) {
        panic!("set_rotation_distance is not used by endstop_phase")
    }
    fn get_dir_inverted(&self) -> (bool, bool) {
        panic!("get_dir_inverted is not used by endstop_phase")
    }
    fn set_dir_inverted(&self, _invert_dir: bool) -> KResult<()> {
        panic!("set_dir_inverted is not used by endstop_phase")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by endstop_phase")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("set_position is not used by endstop_phase")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by endstop_phase")
    }
    fn get_mcu_position(&self, _cmd_pos: Option<f64>) -> i64 {
        panic!("get_mcu_position is not used by endstop_phase")
    }
    fn get_past_mcu_position(&self, _print_time: f64) -> i64 {
        panic!("get_past_mcu_position is not used by endstop_phase")
    }
    fn mcu_to_commanded_position(&self, _mcu_pos: i64) -> f64 {
        panic!("mcu_to_commanded_position is not used by endstop_phase")
    }
    fn dump_steps(
        &self,
        _count: i64,
        _start_clock: i64,
        _end_clock: i64,
    ) -> (Vec<PullHistoryStep>, i64) {
        panic!("dump_steps is not used by endstop_phase")
    }
    fn get_stepper_kinematics(&self) -> Option<StepperKinematics> {
        panic!("get_stepper_kinematics is not used by endstop_phase")
    }
    fn set_stepper_kinematics(&self, _sk: Option<StepperKinematics>) -> Option<StepperKinematics> {
        panic!("set_stepper_kinematics is not used by endstop_phase")
    }
    fn note_homing_end(&self) -> KResult<()> {
        panic!("note_homing_end is not used by endstop_phase")
    }
    fn get_trapq(&self) -> Option<Trapq> {
        panic!("get_trapq is not used by endstop_phase")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) -> Option<Trapq> {
        panic!("set_trapq is not used by endstop_phase")
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) + Send>) -> KResult<()> {
        panic!("add_active_callback is not used by endstop_phase")
    }
    fn is_active_axis(&self, _axis: char) -> bool {
        panic!("is_active_axis is not used by endstop_phase")
    }
}

struct FakeEndstop {
    steppers: Vec<String>,
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> Mcu {
        panic!("get_mcu is not used by endstop_phase")
    }
    fn add_stepper(&self, _stepper: Stepper) {
        panic!("add_stepper is not used by endstop_phase")
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.steppers.iter().map(|n| stepper_of(n)).collect()
    }
    fn home_start(
        &self,
        _print_time: f64,
        _sample_time: f64,
        _sample_count: i64,
        _rest_time: f64,
        _triggered: Option<bool>,
    ) -> Completion<bool> {
        panic!("home_start is not used by endstop_phase")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        panic!("home_wait is not used by endstop_phase")
    }
    fn query_endstop(&self, _print_time: f64) -> KResult<i64> {
        panic!("query_endstop is not used by endstop_phase")
    }
}

#[derive(Clone)]
struct RailSpec {
    steppers: Vec<&'static str>,
    endstop_steppers: Option<Vec<&'static str>>,
    position_endstop: f64,
}

fn rail(steppers: &[&'static str], endstop: Option<&[&'static str]>, pe: f64) -> RailSpec {
    RailSpec {
        steppers: steppers.to_vec(),
        endstop_steppers: endstop.map(|e| e.to_vec()),
        position_endstop: pe,
    }
}

fn py_rails(rails: &[RailSpec]) -> String {
    let parts: Vec<String> = rails
        .iter()
        .map(|r| {
            let steppers: Vec<String> = r.steppers.iter().map(|n| py_str(n)).collect();
            let endstop = match &r.endstop_steppers {
                None => "None".to_string(),
                Some(e) => {
                    let names: Vec<String> = e.iter().map(|n| py_str(n)).collect();
                    format!("[{}]", names.join(", "))
                }
            };
            format!(
                "FakeRail([{}], {}, {})",
                steppers.join(", "),
                endstop,
                py_float(r.position_endstop)
            )
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn py_triggers(triggers: &[(&'static str, i64)]) -> String {
    let parts: Vec<String> = triggers
        .iter()
        .map(|(n, v)| format!("{}: {}", py_str(n), v))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

struct FakeRail {
    spec: RailSpec,
}

impl RailApi for FakeRail {
    fn get_name(&self, _short: Option<bool>) -> String {
        panic!("get_name is not used by endstop_phase")
    }
    fn get_range(&self) -> (f64, f64) {
        panic!("get_range is not used by endstop_phase")
    }
    fn get_homing_info(&self) -> HomingInfo {
        HomingInfo {
            speed: 10.0,
            position_endstop: self.spec.position_endstop,
            retract_speed: 5.0,
            retract_dist: 2.0,
            positive_dir: false,
            second_homing_speed: 5.0,
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        self.spec.steppers.iter().map(|n| stepper_of(n)).collect()
    }
    fn get_endstops(&self) -> Vec<(Arc<dyn EndstopApi>, String)> {
        match &self.spec.endstop_steppers {
            None => Vec::new(),
            Some(names) => vec![(
                Arc::new(FakeEndstop { steppers: names.iter().map(|n| n.to_string()).collect() })
                    as Arc<dyn EndstopApi>,
                "endstop".to_string(),
            )],
        }
    }
    fn lookup_endstop(&self, _endstop_pin: Option<&str>, _name: &str) -> KResult<Arc<dyn EndstopApi>> {
        panic!("lookup_endstop is not used by endstop_phase")
    }
    fn add_stepper(
        &self,
        _stepper: Stepper,
        _endstop_pin: Option<&str>,
        _endstop_name: Option<&str>,
    ) -> KResult<()> {
        panic!("add_stepper is not used by endstop_phase")
    }
    fn add_stepper_from_config(&self, _config: &Config) -> KResult<()> {
        panic!("add_stepper_from_config is not used by endstop_phase")
    }
    fn setup_itersolve(&self, _alloc_func: &str, _params: &[PyValue]) {
        panic!("setup_itersolve is not used by endstop_phase")
    }
    fn set_trapq(&self, _tq: Option<Trapq>) {
        panic!("set_trapq is not used by endstop_phase")
    }
    fn set_position(&self, _coord: [f64; 3]) {
        panic!("set_position is not used by endstop_phase")
    }
    fn calc_position_from_coord(&self, _coord: [f64; 3]) -> f64 {
        panic!("calc_position_from_coord is not used by endstop_phase")
    }
    fn get_commanded_position(&self) -> f64 {
        panic!("get_commanded_position is not used by endstop_phase")
    }
}

struct FakeHoming {
    triggers: Vec<(String, i64)>,
    adjust: GCell<Vec<Value>>,
}

impl HomingStateApi for FakeHoming {
    fn set_axes(&self, _axes: Vec<i64>) {
        panic!("set_axes is not used by endstop_phase")
    }
    fn get_axes(&self) -> Vec<i64> {
        panic!("get_axes is not used by endstop_phase")
    }
    fn get_trigger_position(&self, stepper_name: &str) -> KResult<i64> {
        match self.triggers.iter().find(|(n, _)| n == stepper_name) {
            Some((_, v)) => Ok(*v),
            None => Err(KlippyError::Internal {
                class_name: "KeyError".to_string(),
                message: str_repr(stepper_name),
            }),
        }
    }
    fn set_stepper_adjustment(&self, stepper_name: &str, adjustment: f64) {
        self.adjust
            .borrow_mut()
            .push(tup(vec![s(stepper_name), Value::float(adjustment)]));
    }
    fn set_homed_position(&self, _pos: Vec<Option<f64>>) -> KResult<()> {
        panic!("set_homed_position is not used by endstop_phase")
    }
    fn home_rails(
        &self,
        _rails: Vec<Rail>,
        _forcepos: Vec<Option<f64>>,
        _movepos: Vec<Option<f64>>,
    ) -> KResult<()> {
        panic!("home_rails is not used by endstop_phase")
    }
}

// ---------------------------------------------------------------------------
// The Rust environment and its snapshot.
// ---------------------------------------------------------------------------

struct Env {
    printer: Arc<FakePrinter>,
    gcode: Arc<FakeGCode>,
    configfile: Arc<FakeConfigFile>,
    shared: Arc<CfgShared>,
    factory: Arc<FakeStepperFactory>,
    last_hs: GCell<Option<Arc<FakeHoming>>>,
}

impl Env {
    fn new(sc: &Scenario) -> Env {
        install_sink();
        let factory = Arc::new(FakeStepperFactory {
            outcomes: GCell::new(VecDeque::new()),
            calls: GCell::new(Vec::new()),
        });
        let gcode = Arc::new(FakeGCode {
            registered: GCell::new(Vec::new()),
            handlers: GCell::new(IndexMap::new()),
            responded: GCell::new(Vec::new()),
            gets: GCell::new(Vec::new()),
        });
        let configfile = Arc::new(FakeConfigFile { calls: GCell::new(Vec::new()) });
        let printer = Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
            events: GCell::new(Vec::new()),
            loads: GCell::new(Vec::new()),
            debugoutput: sc.debugoutput,
        });
        {
            let mut objects = printer.objects.borrow_mut();
            objects.insert("gcode".to_string(), gcode.clone() as ObjectRef);
            objects.insert("configfile".to_string(), configfile.clone() as ObjectRef);
            objects.insert(
                "toolhead".to_string(),
                Arc::new(FakeToolHead { factory: factory.clone() }) as ObjectRef,
            );
            for (name, offset, phases) in &sc.tmcs {
                objects.insert(
                    name.clone(),
                    Arc::new(FakeTmc { offset: GCell::new(*offset), phases: *phases }) as ObjectRef,
                );
            }
        }
        let mut sections: IndexMap<String, IndexMap<String, String>> = IndexMap::new();
        for (name, options) in &sc.ini.sections {
            sections.insert(
                name.clone(),
                options
                    .iter()
                    .map(|(k, v)| (k.clone(), v.trim().to_string()))
                    .collect::<IndexMap<String, String>>(),
            );
        }
        let shared = Arc::new(CfgShared {
            sections,
            reads: GCell::new(Vec::new()),
            access: GCell::new(IndexMap::new()),
        });
        Env { printer, gcode, configfile, shared, factory, last_hs: GCell::new(None) }
    }

    fn section(&self, name: &str) -> Config {
        Arc::new(FakeConfig {
            printer: self.printer.clone() as Printer,
            shared: Arc::clone(&self.shared),
            section: name.to_string(),
        })
    }

    fn load(&self, name: &str) -> KResult<Value> {
        let config = self.section("");
        self.printer
            .load_object(&config, name, LookupDefault::Raise)
            .map(|_| Value::Null)
    }

    fn send(&self, event: &str, params: &[EventArg]) -> KResult<Value> {
        let results = self.printer.send_event(event, params)?;
        Ok(list(results.iter().map(pyvalue_to_value).collect()))
    }

    fn home(&self, rails: &[RailSpec], triggers: &[(&'static str, i64)]) -> KResult<Value> {
        let hs = Arc::new(FakeHoming {
            triggers: triggers.iter().map(|(n, v)| (n.to_string(), *v)).collect(),
            adjust: GCell::new(Vec::new()),
        });
        *self.last_hs.borrow_mut() = Some(hs.clone());
        let hs_dyn: Arc<dyn HomingStateApi> = hs;
        let rail_args: Vec<EventArg> = rails
            .iter()
            .map(|r| {
                let rail: Rail = Arc::new(FakeRail { spec: r.clone() });
                EventArg::handle(rail)
            })
            .collect();
        self.send(
            "homing:home_rails_end",
            &[EventArg::handle(hs_dyn), EventArg::List(rail_args)],
        )
    }

    fn run_cmd(&self, cmd: &str, params: &[(&'static str, &'static str)]) -> KResult<Value> {
        let gcmd: GCmd = Arc::new(FakeGcmd {
            gcode: self.gcode.clone(),
            command: cmd.to_string(),
            params: params.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        });
        let mut handler = self
            .gcode
            .handlers
            .borrow_mut()
            .shift_remove(cmd)
            .expect("command registered");
        let result = handler(&gcmd);
        self.gcode.handlers.borrow_mut().insert(cmd.to_string(), handler);
        result.map(|_| Value::Null)
    }

    fn set_tmc(&self, name: &str, offset: Option<i64>) {
        let obj = self.printer.objects.borrow().get(name).cloned().expect("tmc object");
        let tmc = downcast_arc::<FakeTmc>(&obj).expect("FakeTmc");
        tmc.offset.set(offset);
    }

    fn snap_env(&self) -> Value {
        let adjust = match &*self.last_hs.borrow() {
            None => Vec::new(),
            Some(hs) => hs.adjust.borrow().clone(),
        };
        dict(vec![
            ("events", list(self.printer.events.borrow().iter().map(|e| s(e)).collect())),
            ("loads", list(self.printer.loads.borrow().iter().map(|e| s(e)).collect())),
            ("commands", list(self.gcode.registered.borrow().clone())),
            ("reads", list(self.shared.reads.borrow().clone())),
            (
                "access",
                list(
                    self.shared
                        .access
                        .borrow()
                        .iter()
                        .map(|((sec, opt), repr)| tup(vec![s(sec), s(opt), s(repr)]))
                        .collect(),
                ),
            ),
            ("responded", list(self.gcode.responded.borrow().clone())),
            ("gets", list(self.gcode.gets.borrow().clone())),
            ("configfile", list(self.configfile.calls.borrow().clone())),
            ("log", list(captured_log().iter().map(|l| s(l)).collect())),
            ("psd", list(self.factory.calls.borrow().clone())),
            ("adjust", list(adjust)),
        ])
    }

    fn snap_all(&self) -> Value {
        let objects: Vec<(String, ObjectRef)> = self
            .printer
            .objects
            .borrow()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect();
        let mut eps = Vec::new();
        let mut ep = Vec::new();
        for (name, obj) in objects {
            if name == "endstop_phase" {
                let e = downcast_arc::<EndstopPhases>(&obj).expect("EndstopPhases");
                eps.push(snap_eps(&e));
            } else if name.starts_with("endstop_phase ") {
                let e = downcast_arc::<EndstopPhase>(&obj).expect("EndstopPhase");
                ep.push(tup(vec![s(&name), snap_ep(&e)]));
            }
        }
        dict(vec![("env", self.snap_env()), ("eps", list(eps)), ("ep", list(ep))])
    }
}

fn snap_pc(pc: &PhaseCalc) -> Value {
    dict(vec![
        ("name", s(pc.name())),
        ("phases", opt_int(pc.phases())),
        ("tmc", Value::Bool(pc.has_tmc_module())),
        (
            "history",
            match pc.phase_history() {
                Some(h) => list(h.iter().map(|n| Value::int(*n)).collect()),
                None => Value::Null,
            },
        ),
        ("last_phase", opt_int(pc.last_phase())),
        ("last_mcu_position", opt_int(pc.last_mcu_position())),
        ("is_primary", Value::Bool(pc.is_primary())),
        ("stats_only", Value::Bool(pc.stats_only())),
    ])
}

fn snap_ep(ep: &EndstopPhase) -> Value {
    dict(vec![
        ("name", s(ep.name())),
        ("step_dist", Value::float(ep.step_dist())),
        ("phases", Value::int(ep.phases())),
        ("endstop_phase", opt_int(ep.endstop_phase())),
        ("align_zero", Value::Bool(ep.endstop_align_zero())),
        (
            "accuracy",
            match ep.endstop_accuracy() {
                Some(a) => Value::float(a),
                None => Value::Null,
            },
        ),
        ("phase_accuracy", Value::int(ep.endstop_phase_accuracy())),
        ("calc", snap_pc(&ep.phase_calc())),
    ])
}

fn snap_eps(eps: &EndstopPhases) -> Value {
    use klippy_api::PrinterObject;
    let tracking: Vec<Value> = eps
        .tracked_names()
        .iter()
        .map(|name| {
            let pc = eps.tracked(name).expect("tracked");
            tup(vec![s(name), snap_pc(&pc)])
        })
        .collect();
    let status = eps.get_status(0.0).expect("EndstopPhases has get_status");
    dict(vec![("tracking", list(tracking)), ("status", pyvalue_to_value(&status))])
}

// ---------------------------------------------------------------------------
// Scenarios.
// ---------------------------------------------------------------------------

enum Step {
    Connect,
    Home { rails: Vec<RailSpec>, triggers: Vec<(&'static str, i64)> },
    Cmd { cmd: &'static str, params: Vec<(&'static str, &'static str)> },
    SetTmc { name: &'static str, offset: Option<i64> },
}

struct Scenario {
    label: String,
    ini: Ini,
    debugoutput: bool,
    tmcs: Vec<(String, Option<i64>, i64)>,
    /// Section names handed to `printer.load_object`, in config-file order.
    loads: Vec<&'static str>,
    steps: Vec<Step>,
}

impl Scenario {
    fn new(label: &str, ini: Ini, loads: &[&'static str]) -> Scenario {
        Scenario {
            label: label.to_string(),
            ini,
            debugoutput: false,
            tmcs: Vec::new(),
            loads: loads.to_vec(),
            steps: Vec::new(),
        }
    }
    fn debugoutput(mut self) -> Scenario {
        self.debugoutput = true;
        self
    }
    fn tmc(mut self, name: &str, offset: Option<i64>, phases: i64) -> Scenario {
        self.tmcs.push((name.to_string(), offset, phases));
        self
    }
    fn step(mut self, step: Step) -> Scenario {
        self.steps.push(step);
        self
    }
    fn connect(self) -> Scenario {
        self.step(Step::Connect)
    }
    fn home(self, rails: Vec<RailSpec>, triggers: &[(&'static str, i64)]) -> Scenario {
        self.step(Step::Home { rails, triggers: triggers.to_vec() })
    }
    fn cmd(self, cmd: &'static str, params: &[(&'static str, &'static str)]) -> Scenario {
        self.step(Step::Cmd { cmd, params: params.to_vec() })
    }
    fn set_tmc(self, name: &'static str, offset: Option<i64>) -> Scenario {
        self.step(Step::SetTmc { name, offset })
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("endstop_phase fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping endstop_phase oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.endstop_phase".to_string(),
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

fn check(label: &str, expected: &Value, actual: &Value) {
    if let Err(d) = assert_equal(&record(label), "ret", expected, actual) {
        panic!("{label}: {d}");
    }
}

fn cmp_result(label: &str, expected: pyoracle::OracleResult, actual: KResult<Value>) {
    match (expected, actual) {
        (Ok(e), Ok(a)) => check(label, &e, &a),
        (Err(e), Err(a)) => {
            assert_eq!(a.class_name(), e.class_name, "{label}: error class ({})", a.message());
            assert_eq!(a.message(), e.message, "{label}: error message");
        }
        (Ok(e), Err(a)) => panic!(
            "{label}: oracle returned {}, rust raised {}: {}",
            e.render(),
            a.class_name(),
            a.message()
        ),
        (Err(e), Ok(a)) => panic!(
            "{label}: oracle raised {}: {}, rust returned {}",
            e.class_name,
            e.message,
            a.render()
        ),
    }
}

fn cmp_snapshot(label: &str, py: &mut pyoracle::Oracle, env: &Env) {
    let expected = py.eval("snap_all()").unwrap_or_else(|e| {
        panic!("{label}: oracle snapshot raised {}: {}", e.class_name, e.message)
    });
    check(&format!("{label}: snapshot"), &expected, &env.snap_all());
}

/// Feeds the outcomes of the `parse_step_distance` calls the real Python made since
/// `taken` into the Rust fake factory.
fn feed_psd(py: &mut pyoracle::Oracle, env: &Env, taken: usize) -> usize {
    let outcomes = py
        .eval(&format!("psd_outcomes({taken})"))
        .unwrap_or_else(|e| panic!("psd outcomes raised {}: {}", e.class_name, e.message));
    let items = match outcomes {
        Value::List(items) => items,
        other => panic!("unexpected psd outcome list {}", other.render()),
    };
    let count = items.len();
    for item in items {
        let fields = match item {
            Value::Tuple(fields) => fields,
            other => panic!("unexpected psd outcome {}", other.render()),
        };
        let text = |v: &Value| match v {
            Value::Str(x) => x.clone(),
            other => panic!("unexpected psd field {}", other.render()),
        };
        let kind = text(&fields[0]);
        let outcome = if kind == "ok" {
            let rd = recording::f64_from_hex(&text(&fields[1])).expect("rotation distance hex");
            let spr = recording::f64_from_hex(&text(&fields[2])).expect("steps per rotation hex");
            Psd::Ok(rd, spr)
        } else {
            let class = text(&fields[1]);
            let message = text(&fields[2]);
            Psd::Err(if class == "Error" {
                KlippyError::config(message)
            } else {
                KlippyError::Internal { class_name: class, message }
            })
        };
        env.factory.outcomes.borrow_mut().push_back(outcome);
    }
    taken + count
}

fn run(sc: Scenario) {
    let Some(mut py) = oracle() else { return };
    let label = sc.label.clone();
    let tmcs: Vec<String> = sc
        .tmcs
        .iter()
        .map(|(n, o, p)| format!("({}, {}, {})", py_str(n), py_opt_int(*o), p))
        .collect();
    py.exec(&format!(
        "setup({}, {}, [{}])",
        py_str(&sc.ini.text()),
        py_bool(sc.debugoutput),
        tmcs.join(", ")
    ))
    .expect("py setup");
    let env = Env::new(&sc);
    let mut taken = 0usize;
    for (i, name) in sc.loads.iter().enumerate() {
        let step_label = format!("{label}: load {i} ({name})");
        let expected = py.exec(&format!("load({})", py_str(name)));
        taken = feed_psd(&mut py, &env, taken);
        let failed = expected.is_err();
        cmp_result(&step_label, expected, env.load(name));
        cmp_snapshot(&step_label, &mut py, &env);
        if failed {
            return;
        }
    }
    for (i, step) in sc.steps.iter().enumerate() {
        let step_label = format!("{label}: step {i}");
        let (expected, actual) = match step {
            Step::Connect => (
                py.eval("send('klippy:connect')"),
                env.send("klippy:connect", &[]),
            ),
            Step::Home { rails, triggers } => (
                py.eval(&format!("home({}, {})", py_rails(rails), py_triggers(triggers))),
                env.home(rails, triggers),
            ),
            Step::Cmd { cmd, params } => {
                let py_params: Vec<String> = params
                    .iter()
                    .map(|(k, v)| format!("{}: {}", py_str(k), py_str(v)))
                    .collect();
                (
                    py.eval(&format!("run_cmd({}, {{{}}})", py_str(cmd), py_params.join(", "))),
                    env.run_cmd(cmd, params),
                )
            }
            Step::SetTmc { name, offset } => {
                let expected = py.eval(&format!("set_tmc({}, {})", py_str(name), py_opt_int(*offset)));
                env.set_tmc(name, *offset);
                (expected, Ok(Value::Null))
            }
        };
        cmp_result(&step_label, expected, actual);
        cmp_snapshot(&step_label, &mut py, &env);
    }
}

// ---------------------------------------------------------------------------
// Config helpers.
// ---------------------------------------------------------------------------

fn ini_with(
    stepper_opts: &[(&str, &str)],
    ep_sections: &[(&str, &[(&str, &str)])],
    with_eps: bool,
) -> Ini {
    let mut ini = Ini::new();
    if with_eps {
        ini = ini.section("endstop_phase", &[]);
    }
    ini = ini.section("stepper_x", stepper_opts);
    for (name, options) in ep_sections {
        ini = ini.section(name, options);
    }
    ini
}

fn default_stepper() -> Vec<(&'static str, &'static str)> {
    vec![("microsteps", "16"), ("rotation_distance", "40")]
}

fn construct(label: &str, stepper: &[(&str, &str)], ep: &[(&str, &str)]) -> Scenario {
    Scenario::new(
        label,
        ini_with(stepper, &[("endstop_phase stepper_x", ep)], false),
        &["endstop_phase stepper_x"],
    )
}

// ---------------------------------------------------------------------------
// Construction tests.
// ---------------------------------------------------------------------------

#[test]
fn construct_defaults() {
    run(construct("defaults", &default_stepper(), &[]));
    run(construct(
        "other microsteps",
        &[("microsteps", "1"), ("rotation_distance", "8")],
        &[],
    ));
    run(construct(
        "256 microsteps with full_steps and gear ratio",
        &[
            ("microsteps", "256"),
            ("rotation_distance", "32"),
            ("full_steps_per_rotation", "400"),
            ("gear_ratio", "80:16"),
        ],
        &[],
    ));
}

#[test]
fn construct_trigger_phase() {
    for value in [
        "3/64", "0/1", "1/3", "63/64", "1/4", "-1/4", "5/5", "7/5", "5", "a/b", "", "3/64/2",
        "1/0", "-1/0", " 2 / 8 ", "0/16", "15/16", "1/2",
    ] {
        run(construct(
            &format!("trigger_phase {value:?}"),
            &default_stepper(),
            &[("trigger_phase", value)],
        ));
    }
}

#[test]
fn construct_endstop_accuracy() {
    for value in [
        "0.1", "0.0125", "0.4", "0.3999999", "0.39", "0.8", "1.0", "0", "-1", "abc", "nan",
        "inf", "1e-9",
    ] {
        run(construct(
            &format!("endstop_accuracy {value} without trigger_phase"),
            &default_stepper(),
            &[("endstop_accuracy", value)],
        ));
        run(construct(
            &format!("endstop_accuracy {value} with trigger_phase"),
            &default_stepper(),
            &[("trigger_phase", "3/64"), ("endstop_accuracy", value)],
        ));
    }
}

#[test]
fn construct_align_zero() {
    for value in ["true", "false", "yes", "off", "1", "maybe", ""] {
        run(construct(
            &format!("endstop_align_zero {value:?}"),
            &default_stepper(),
            &[("trigger_phase", "3/64"), ("endstop_align_zero", value)],
        ));
    }
}

#[test]
fn construct_debugoutput() {
    run(construct("debugoutput", &default_stepper(), &[]).debugoutput());
    run(construct(
        "debugoutput with accuracy",
        &default_stepper(),
        &[("endstop_accuracy", "0.1")],
    )
    .debugoutput());
    run(construct(
        "debugoutput does not hide an accuracy error",
        &default_stepper(),
        &[("endstop_accuracy", "0.5")],
    )
    .debugoutput());
}

#[test]
fn construct_stepper_section_errors() {
    run(Scenario::new(
        "missing stepper section",
        Ini::new().section("endstop_phase stepper_q", &[]),
        &["endstop_phase stepper_q"],
    ));
    run(construct("microsteps 0", &[("microsteps", "0"), ("rotation_distance", "40")], &[]));
    run(construct("microsteps text", &[("microsteps", "x"), ("rotation_distance", "40")], &[]));
    run(construct("missing microsteps", &[("rotation_distance", "40")], &[]));
    run(construct(
        "full steps not multiple of 4",
        &[("microsteps", "16"), ("rotation_distance", "40"), ("full_steps_per_rotation", "201")],
        &[],
    ));
    run(construct(
        "zero gear ratio divides by zero",
        &[("microsteps", "16"), ("rotation_distance", "40"), ("gear_ratio", "0:1")],
        &[],
    ));
    run(construct("missing rotation distance", &[("microsteps", "16")], &[]));
    run(construct(
        "radians stepper",
        &[("microsteps", "16"), ("gear_ratio", "5:1")],
        &[],
    ));
}

#[test]
fn construct_loads_endstop_phases_once() {
    let ini = Ini::new()
        .section("endstop_phase", &[])
        .section("stepper_x", &default_stepper())
        .section("stepper_y", &default_stepper())
        .section("endstop_phase stepper_x", &[("trigger_phase", "3/64")])
        .section("endstop_phase stepper_y", &[("endstop_accuracy", "0.1")]);
    run(Scenario::new(
        "endstop_phase loaded first",
        ini.clone(),
        &["endstop_phase", "endstop_phase stepper_x", "endstop_phase stepper_y"],
    ));
    let ini_no_main = Ini::new()
        .section("stepper_x", &default_stepper())
        .section("stepper_y", &default_stepper())
        .section("endstop_phase stepper_x", &[("trigger_phase", "3/64")])
        .section("endstop_phase stepper_y", &[("endstop_accuracy", "0.1")]);
    run(Scenario::new(
        "endstop_phase loaded by the first prefix section",
        ini_no_main,
        &["endstop_phase stepper_x", "endstop_phase stepper_y"],
    ));
}

#[test]
fn construct_endstop_phases_registers_command() {
    run(Scenario::new("endstop_phases alone", Ini::new().section("endstop_phase", &[]), &["endstop_phase"]));
}

// ---------------------------------------------------------------------------
// Homing tests for [endstop_phase <stepper>].
// ---------------------------------------------------------------------------

fn x_rail(pe: f64) -> Vec<RailSpec> {
    vec![rail(&["stepper_x"], Some(&["stepper_x"]), pe)]
}

#[test]
fn connect_without_and_with_tmc() {
    run(construct("connect without tmc", &default_stepper(), &[]).connect());
    run(construct("connect with tmc", &default_stepper(), &[]).tmc("tmc2130 stepper_x", Some(5), 256).connect());
    run(
        construct("first driver wins", &default_stepper(), &[])
            .tmc("tmc2209 stepper_x", Some(7), 128)
            .tmc("tmc2130 stepper_x", Some(3), 64)
            .tmc("tmc5160 stepper_x", Some(1), 32)
            .connect()
            .home(x_rail(0.0), &[("stepper_x", 100)]),
    );
    for driver in ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"] {
        run(
            construct(&format!("driver {driver}"), &default_stepper(), &[])
                .tmc(&format!("{driver} stepper_x"), Some(2), 64)
                .connect()
                .home(x_rail(0.0), &[("stepper_x", 10)]),
        );
    }
}

#[test]
fn home_learns_phase_then_applies_deltas() {
    let mut sc = construct("learn then adjust", &default_stepper(), &[])
        .tmc("tmc2130 stepper_x", Some(5), 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 100)]);
    for trig in [100, 101, 99, 131, 132, 69, 68, 164, 163, 228, 36, -28, 0, 5000, -5000] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    run(sc);
}

#[test]
fn home_without_tmc_uses_mcu_position() {
    let mut sc = construct("no tmc", &default_stepper(), &[]).connect();
    for trig in [17, 18, 16, 49, 50, 48, 81, 82, -15, 17 + 64 * 1000] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    run(sc);
}

#[test]
fn home_with_configured_trigger_phase_and_accuracy() {
    let mut sc = construct(
        "configured phase",
        &default_stepper(),
        &[("trigger_phase", "10/64"), ("endstop_accuracy", "0.1")],
    )
    .connect();
    for trig in [10, 11, 9, 18, 19, 2, 1, 74, 138, -54, 0, 30] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    run(sc);
}

#[test]
fn home_before_connect_ignores_history() {
    run(construct("home before connect", &default_stepper(), &[]).home(x_rail(0.0), &[("stepper_x", 3)]));
}

#[test]
fn home_align_zero() {
    for pe in [0.0, 0.3, -2.5, 100.0, 0.0125, 1.5e-3, 12345.6789, -0.0] {
        let mut sc = construct(
            &format!("align zero position_endstop {pe:?}"),
            &default_stepper(),
            &[("trigger_phase", "3/64"), ("endstop_align_zero", "true")],
        )
        .connect();
        for trig in [3, 4, 2] {
            sc = sc.home(x_rail(pe), &[("stepper_x", trig)]);
        }
        run(sc);
    }
    for phase in ["0/64", "7/64", "8/64", "33/64", "63/64", "1/3"] {
        run(construct(
            &format!("align zero trigger_phase {phase}"),
            &default_stepper(),
            &[("trigger_phase", phase), ("endstop_align_zero", "true")],
        )
        .connect()
        .home(x_rail(4.2), &[("stepper_x", 9)]));
    }
    run(construct(
        "align zero without trigger_phase",
        &default_stepper(),
        &[("endstop_align_zero", "true")],
    )
    .connect()
    .home(x_rail(4.2), &[("stepper_x", 9)])
    .home(x_rail(4.2), &[("stepper_x", 10)]));
    run(construct(
        "align zero with tiny microsteps",
        &[("microsteps", "1"), ("rotation_distance", "8")],
        &[("trigger_phase", "1/4"), ("endstop_align_zero", "true")],
    )
    .connect()
    .home(x_rail(2.2), &[("stepper_x", 5)]));
}

#[test]
fn home_phase_unknown() {
    run(construct("phase unknown", &default_stepper(), &[])
        .tmc("tmc2130 stepper_x", None, 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 5)])
        .set_tmc("tmc2130 stepper_x", Some(4))
        .home(x_rail(0.0), &[("stepper_x", 5)]));
    run(construct("phase unknown with debugoutput", &default_stepper(), &[])
        .debugoutput()
        .tmc("tmc2130 stepper_x", None, 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 5)])
        .home(x_rail(0.0), &[("stepper_x", 6)]));
}

#[test]
fn home_rail_matching() {
    let ini = Ini::new()
        .section("stepper_x", &default_stepper())
        .section("stepper_y", &default_stepper())
        .section("endstop_phase stepper_x", &[("trigger_phase", "3/64")])
        .section("endstop_phase stepper_y", &[]);
    let two_rails = vec![
        rail(&["stepper_x", "stepper_x1"], Some(&["stepper_x", "stepper_x1"]), 0.0),
        rail(&["stepper_y"], Some(&["stepper_y"]), 1.0),
    ];
    run(Scenario::new("rail matching", ini.clone(), &["endstop_phase stepper_x", "endstop_phase stepper_y"])
        .connect()
        .home(two_rails.clone(), &[("stepper_x", 3), ("stepper_x1", 9), ("stepper_y", 20)])
        .home(two_rails.clone(), &[("stepper_x", 4), ("stepper_x1", 9), ("stepper_y", 21)])
        .home(two_rails.clone(), &[("stepper_x", 5), ("stepper_x1", 9), ("stepper_y", 22)]));
    // The endstop's first stepper decides, not the rail's first stepper.
    let odd = vec![rail(&["stepper_y"], Some(&["stepper_x", "stepper_y"]), 0.0)];
    run(Scenario::new("endstop first stepper decides", ini.clone(), &["endstop_phase stepper_x", "endstop_phase stepper_y"])
        .connect()
        .home(odd, &[("stepper_x", 3), ("stepper_y", 4)]));
    run(Scenario::new("no rails", ini, &["endstop_phase stepper_x", "endstop_phase stepper_y"])
        .connect()
        .home(Vec::new(), &[]));
}

#[test]
fn home_error_paths() {
    run(construct("rail without endstops", &default_stepper(), &[])
        .connect()
        .home(vec![rail(&["stepper_x"], None, 0.0)], &[("stepper_x", 3)]));
    run(construct("endstop without steppers", &default_stepper(), &[])
        .connect()
        .home(vec![rail(&["stepper_x"], Some(&[]), 0.0)], &[("stepper_x", 3)]));
    run(construct("missing trigger position", &default_stepper(), &[])
        .connect()
        .home(x_rail(0.0), &[("stepper_y", 3)]));
    run(construct("incorrect phase error text", &default_stepper(), &[("trigger_phase", "10/64")])
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 50)])
        .home(x_rail(0.0), &[("stepper_x", 10)]));
}

// ---------------------------------------------------------------------------
// [endstop_phase] statistics and ENDSTOP_PHASE_CALIBRATE.
// ---------------------------------------------------------------------------

fn stats_ini() -> Ini {
    Ini::new()
        .section("endstop_phase", &[])
        .section("stepper_x", &default_stepper())
        .section("stepper_z", &default_stepper())
        .section("stepper_z1", &default_stepper())
}

fn z_rails() -> Vec<RailSpec> {
    vec![rail(&["stepper_z", "stepper_z1"], Some(&["stepper_z"]), 0.5)]
}

#[test]
fn calibrate_report_without_steppers() {
    run(Scenario::new("no steppers", stats_ini(), &["endstop_phase"])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
        .connect());
}

#[test]
fn calibrate_stats_only_steppers() {
    let mut sc = Scenario::new("stats only", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", Some(3), 64)
        .tmc("tmc2209 stepper_z", Some(9), 128)
        .tmc("tmc5160 stepper_z1", Some(1), 32)
        .connect()
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
    for trig in [10, 11, 10, 12, 10, 9, 10, 60, 61, 62] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    sc = sc.cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
    for (i, trig) in [5, 6, 5, 7, 5, 4].into_iter().enumerate() {
        sc = sc.home(z_rails(), &[("stepper_z", trig), ("stepper_z1", trig + i as i64)]);
    }
    sc = sc
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z1")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_nope")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "")]);
    run(sc);
}

#[test]
fn calibrate_stepper_without_phase_information() {
    let y_rails = vec![rail(&["stepper_y"], Some(&["stepper_y"]), 0.0)];
    run(Scenario::new("no tmc, no phases", stats_ini(), &["endstop_phase"])
        .connect()
        .home(y_rails.clone(), &[("stepper_y", 3)])
        .home(y_rails, &[("stepper_y", 4)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_y")]));
}

#[test]
fn calibrate_stats_phase_unknown() {
    run(Scenario::new("stats phase unknown", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", None, 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 3)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
        .set_tmc("tmc2130 stepper_x", Some(2))
        .home(x_rail(0.0), &[("stepper_x", 3)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")]));
    run(Scenario::new("stats phase unknown with debugoutput", stats_ini(), &["endstop_phase"])
        .debugoutput()
        .tmc("tmc2130 stepper_x", None, 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 3)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")]));
}

#[test]
fn calibrate_with_endstop_phase_sections() {
    let ini = stats_ini()
        .section("endstop_phase stepper_x", &[("trigger_phase", "10/64")])
        .section("endstop_phase stepper_z1", &[]);
    let mut sc = Scenario::new(
        "endstop_phase sections feed the tracker",
        ini,
        &["endstop_phase", "endstop_phase stepper_x", "endstop_phase stepper_z1"],
    )
    .tmc("tmc2130 stepper_x", Some(1), 64)
    .connect()
    .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
    .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z1")])
    .cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
    for trig in [9, 10, 11, 9, 10] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    sc = sc
        .home(z_rails(), &[("stepper_z", 4), ("stepper_z1", 8)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z1")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z")]);
    run(sc);
}

#[test]
fn calibrate_before_connect_has_no_history() {
    let ini = stats_ini().section("endstop_phase stepper_x", &[]);
    run(Scenario::new(
        "section before connect",
        ini,
        &["endstop_phase", "endstop_phase stepper_x"],
    )
    .home(x_rail(0.0), &[("stepper_x", 3)])
    .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
    .cmd("ENDSTOP_PHASE_CALIBRATE", &[]));
}

#[test]
fn calibrate_wraparound_and_ties() {
    // Samples that straddle the wrap point and a bimodal history (equal costs).
    let mut sc = Scenario::new("wraparound", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", Some(0), 64)
        .connect();
    for trig in [62, 63, 0, 1, 63, 0, 62, 1, 63] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    sc = sc.cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")]);
    run(sc);
    let mut sc = Scenario::new("bimodal", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", Some(0), 64)
        .connect();
    for trig in [10, 42, 10, 42] {
        sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
    }
    sc = sc.cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")]);
    run(sc);
    let mut sc = Scenario::new("single sample", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", Some(0), 64)
        .connect()
        .home(x_rail(0.0), &[("stepper_x", 0)])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
    sc = sc.home(x_rail(0.0), &[("stepper_x", 63)]).cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
    run(sc);
}

#[test]
fn calibrate_odd_phase_counts() {
    for phases in [1, 2, 3, 4, 7, 12, 100] {
        let mut sc = Scenario::new(&format!("{phases} phases"), stats_ini(), &["endstop_phase"])
            .tmc("tmc2130 stepper_x", Some(0), phases)
            .connect();
        for trig in [0, 1, 2, phases - 1, phases, phases + 1, -1] {
            sc = sc.home(x_rail(0.0), &[("stepper_x", trig)]);
        }
        sc = sc
            .cmd("ENDSTOP_PHASE_CALIBRATE", &[])
            .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")]);
        run(sc);
    }
}

#[test]
fn calibrate_pseudo_random_history() {
    let mut state: u64 = 0x2545_F491_4F6C_DD1D;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    let mut sc = Scenario::new("pseudo random", stats_ini(), &["endstop_phase"])
        .tmc("tmc2130 stepper_x", Some(11), 128)
        .tmc("tmc2209 stepper_z", Some(-3), 64)
        .connect();
    for i in 0..40 {
        let x = (next() % 400) as i64 - 200;
        let z = (next() % 90) as i64;
        sc = sc.home(x_rail(0.0), &[("stepper_x", x)]);
        sc = sc.home(
            vec![rail(&["stepper_z"], Some(&["stepper_z"]), 0.5)],
            &[("stepper_z", z)],
        );
        if i % 7 == 6 {
            sc = sc.cmd("ENDSTOP_PHASE_CALIBRATE", &[]);
        }
    }
    sc = sc
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_x")])
        .cmd("ENDSTOP_PHASE_CALIBRATE", &[("STEPPER", "stepper_z")]);
    run(sc);
}

// ---------------------------------------------------------------------------
// Module entry points.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_shape() {
    let factory = module_factory();
    assert!(factory.load_config.is_some());
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}
