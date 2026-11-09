//! Parity tests for `extras::probe` (port of `klippy/extras/probe.py`).
//!
//! Expected values come from the live reference through `pyoracle` (CPython 3.8.2 + the frozen
//! `klippy/`). The Python side runs the real `extras.probe`, `extras.manual_probe`,
//! `configfile.ConfigWrapper` and `gcode.GCodeCommand` against minimal fakes of the printer,
//! gcode dispatch, toolhead, homing, pins and `gcode_macro` objects. The Rust side runs the port
//! against fakes of the same contract traits with identical inputs. Every step compares the
//! G-code responses, scripts run by the activate/deactivate templates, toolhead moves, homing
//! calls, endstop queries, configfile writes, status dicts and raised errors (class and message).
//!
//! No recordings exist under `tests/fixtures/recordings/` for this unit, so expected values come
//! from the oracle alone.

use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::manual_probe::{ManualProbe, ProbeResult};
use extras::probe::{
    calc_probe_z_average, lookup_minimum_z, module_factory, run_single_probe,
    DescendToEndstopHelper, HomingViaProbeHelper, LookupZSteppers, PrinterProbe,
    PrinterProbeApi, ProbeCommandHelper, ProbeOffsetsHelper, ProbeOffsetsHelperApi,
    ProbeParameterHelper, ProbeParameterHelperApi, ProbeParams, ProbePointsHelper, ProbeResultList,
    ProbeSessionApi, HINT_TIMEOUT,
};
use extras_macros_templates::gcode_macro::Value as TemplateValue;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts, ListParser, PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCodeMoveApi, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, GetParser, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueryWrapper, CommandQueue,
    CommandWrapper, ConfigCallback, Endstop, EndstopApi, LookupPinOpts, LookupQueryCommandOpts,
    Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::printer::{
    EventCallback, LookupDefault, Printer, PrinterApi, EV_GCODE_COMMAND_ERROR,
    EV_KLIPPY_MCU_IDENTIFY, EV_PROBE_UPDATE_RESULTS,
};
use klippy_api::stepper::{McuStepperApi, Stepper, StepperFactory, Trapq, TrapqApi};
use klippy_api::toolhead::{
    ExtraAxis, HomingStateHandle, HomingTarget, HomingTargetApi, Kinematics, KinematicsApi,
    PrinterExtruder, PrinterHomingApi, ToolHeadApi,
};
use klippy_api::{EventArg, GCell, ObjectRef, PrinterObject};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::{Arc, Weak};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------
const SETUP: &str = r#"
import configparser
import configfile
import gcode as gcode_mod
import extras.manual_probe as manual_probe
import extras.probe as probe

CommandError = gcode_mod.CommandError

class FakeReactor:
    def monotonic(self):
        return 0.5

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=configfile.sentinel):
        return self.lookup_object(section, default)
    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]

class FakeGCode:
    error = CommandError
    Coord = gcode_mod.Coord
    def __init__(self):
        self.handlers = {}
        self.help = {}
        self.log = []
        self.th = None
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        old = self.handlers.get(cmd)
        if func is None:
            if old is not None:
                del self.handlers[cmd]
            if cmd in self.help:
                del self.help[cmd]
            return old
        if old is not None:
            raise configparser.Error('gcode command %s already registered' % (cmd,))
        self.handlers[cmd] = func
        if desc is not None:
            self.help[cmd] = desc
        return None
    def create_gcode_command(self, command, commandline, params):
        return gcode_mod.GCodeCommand(self, command, commandline, params, False)
    def respond_info(self, msg, log=True):
        self.log.append(msg)
    def respond_raw(self, msg):
        self.log.append(msg)
    def run_script_from_command(self, script):
        self.log.append('script: ' + script)
        if script == 'MOVE' and self.th is not None:
            self.th.position[0] += 1.0

class FakeStepper:
    def __init__(self, name):
        self.name = name
    def get_name(self, short=False):
        return self.name
    def get_commanded_position(self):
        return 0.0
    def is_active_axis(self, axis):
        return self.name.endswith(axis)

class FakeKin:
    def __init__(self, th):
        self.th = th
    def get_steppers(self):
        return [FakeStepper('stepper_x'), FakeStepper('stepper_y'), FakeStepper('stepper_z')]
    def calc_position(self, spos):
        return list(self.th.position)

class FakeToolhead:
    def __init__(self, position, homed):
        self.position = list(position)
        self.homed = homed
        self.moves = []
        self.time_calls = 0
        self.kin = FakeKin(self)
    def get_position(self):
        return list(self.position)
    def get_status(self, eventtime):
        return {'homed_axes': self.homed}
    def get_last_move_time(self):
        self.time_calls += 1
        return float(self.time_calls)
    def get_kinematics(self):
        return self.kin
    def flush_step_generation(self):
        pass
    def manual_move(self, coord, speed):
        self.moves.append((list(coord), speed))
        for i, v in enumerate(coord):
            if v is not None:
                self.position[i] = v

class FakeHoming:
    def __init__(self, th):
        self.th = th
        self.script = []
        self.calls = []
        self.first_checks = 0
    def check_probe_first_home(self, gcmd):
        self.first_checks += 1
        return (gcmd.get_command() == 'G28'
                and gcmd.get('HOME_ATTEMPT_NUM', None) == '1')
    def probing_move(self, mcu_probe, pos, speed, check_movement=True):
        self.calls.append((list(pos), speed, check_movement))
        item = self.script.pop(0)
        if isinstance(item, str):
            raise CommandError(item)
        epos = list(pos)
        epos[2] = item
        self.th.position = list(epos)
        return epos

class FakeEndstop:
    def __init__(self, pin):
        self.pin = pin
        self.added = []
        self.queries = []
        self.value = 0
    def add_stepper(self, stepper):
        self.added.append(stepper.get_name())
    def query_endstop(self, print_time):
        self.queries.append(print_time)
        return self.value

class FakePins:
    def __init__(self):
        self.setups = []
        self.chips = []
        self.chip = None
        self.endstop = None
    def setup_pin(self, pin_type, pin_desc):
        self.setups.append((pin_type, pin_desc))
        self.endstop = FakeEndstop(pin_desc)
        return self.endstop
    def register_chip(self, name, chip):
        self.chips.append(name)
        self.chip = chip

class FakeConfigFile:
    def __init__(self):
        self.writes = []
    def set(self, section, option, value):
        self.writes.append((section, option, value))

class FakeGCodeMove:
    def __init__(self, offset):
        self.offset = offset
    def get_status(self, eventtime=None):
        return {'homing_origin': gcode_mod.Coord((0.0, 0.0, self.offset))}

class FakeTemplate:
    def __init__(self, gcode, script):
        self.gcode = gcode
        self.script = script
    def run_gcode_from_command(self, context=None):
        self.gcode.run_script_from_command(self.script)

class FakeGCodeMacro:
    def __init__(self, gcode):
        self.gcode = gcode
        self.loaded = []
    def load_template(self, config, option, default=configfile.sentinel):
        self.loaded.append((config.get_name(), option))
        return FakeTemplate(self.gcode, config.get(option, default))

class FakeSession:
    def __init__(self, owner):
        self.owner = owner
    def run_probe(self, gcmd):
        self.owner.log.append(('run', gcmd.get_command()))
    def pull_probed_results(self):
        return [manual_probe.ProbeResult(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)]
    def end_probe_session(self):
        self.owner.log.append(('end',))

class FakeProbe:
    def __init__(self):
        self.log = []
    def get_probe_params(self, gcmd=None):
        return {'probe_speed': 5.0, 'lift_speed': 6.0, 'samples': 1,
                'sample_retract_dist': 2.0, 'samples_tolerance': 0.1,
                'samples_tolerance_retries': 0, 'samples_result': 'average'}
    def get_offsets(self, gcmd=None):
        return (0.5, 0.25, 1.0)
    def start_probe_session(self, gcmd):
        self.log.append(('start',))
        return FakeSession(self)

class World:
    pass

def make_world(sections, homed='xyz', offset=0.0):
    w = World()
    fc = configparser.RawConfigParser()
    for name, opts in sections:
        fc.add_section(name)
        for key, value in opts:
            fc.set(name, key, value)
    w.p = p = FakePrinter()
    w.g = g = FakeGCode()
    w.th = th = FakeToolhead([10.0, 20.0, 5.0, 0.0], homed)
    g.th = th
    w.homing = FakeHoming(th)
    w.pins = FakePins()
    w.cf = FakeConfigFile()
    w.gm = FakeGCodeMove(offset)
    w.macro = FakeGCodeMacro(g)
    for name, obj in (('gcode', g), ('toolhead', th), ('homing', w.homing),
                      ('pins', w.pins), ('configfile', w.cf), ('gcode_move', w.gm),
                      ('gcode_macro', w.macro)):
        p.add_object(name, obj)
    w.cfg = lambda name: configfile.ConfigWrapper(p, fc, {}, name)
    w.mp = manual_probe.load_config(w.cfg('manual_probe'))
    p.add_object('manual_probe', w.mp)
    w.probe = None
    return w

def add_probe(w):
    w.probe = probe.load_config(w.cfg('probe'))
    w.p.add_object('probe', w.probe)

def make_gcmd(w, cmd, params=None):
    params = dict(params or {})
    line = cmd + ''.join(' %s=%s' % (k, v) for k, v in params.items())
    return w.g.create_gcode_command(cmd, line, params)

def run(w, cmd, params=None):
    return w.g.handlers[cmd](make_gcmd(w, cmd, params))
"#;

// ---------------------------------------------------------------------------
// Configuration tables shared by both sides.
// ---------------------------------------------------------------------------

type Table = Vec<(String, Vec<(String, String)>)>;

const REMOVE: &str = "<remove>";

fn section(name: &str, opts: &[(&str, &str)]) -> (String, Vec<(String, String)>) {
    (
        name.to_string(),
        opts.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
    )
}

/// The default test printer: `[printer]`, `[stepper_z]` and a `[probe]` section whose options
/// `probe_opts` override (a value of [`REMOVE`] drops the option), plus `more` sections.
fn table(probe_opts: &[(&str, &str)], more: &[(&str, &[(&str, &str)])]) -> Table {
    let mut probe_section: Vec<(String, String)> = vec![
        ("pin".to_string(), "^PA1".to_string()),
        ("z_offset".to_string(), "1.5".to_string()),
        ("x_offset".to_string(), "2.0".to_string()),
        ("y_offset".to_string(), "-3.0".to_string()),
    ];
    for (key, value) in probe_opts {
        probe_section.retain(|(k, _)| k != key);
        if *value != REMOVE {
            probe_section.push((key.to_string(), value.to_string()));
        }
    }
    let mut out: Table = vec![
        section("printer", &[("kinematics", "cartesian")]),
        section("stepper_z", &[("position_min", "-3.0")]),
        ("probe".to_string(), probe_section),
    ];
    for (name, opts) in more {
        out.push(section(name, opts));
    }
    out
}

fn lit(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('\'');
    out
}

fn table_literal(t: &Table) -> String {
    let sections: Vec<String> = t
        .iter()
        .map(|(name, opts)| {
            let opts: Vec<String> = opts
                .iter()
                .map(|(k, v)| format!("({}, {})", lit(k), lit(v)))
                .collect();
            format!("({}, [{}])", lit(name), opts.join(", "))
        })
        .collect();
    format!("[{}]", sections.join(", "))
}

fn py_params(params: &[(&str, &str)]) -> String {
    let items: Vec<String> = params
        .iter()
        .map(|(k, v)| format!("{}: {}", lit(k), lit(v)))
        .collect();
    format!("{{{}}}", items.join(", "))
}

fn commandline(cmd: &str, params: &[(&str, &str)]) -> String {
    let mut line = cmd.to_string();
    for (k, v) in params {
        line.push_str(&format!(" {k}={v}"));
    }
    line
}

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<IndexMap<String, Vec<EventCallback>>>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(IndexMap::new()),
        })
    }

    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }

    fn lookup(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let found = self.objects.borrow().get(name).cloned();
        match found {
            Some(obj) => Ok(Some(obj)),
            None => match default {
                LookupDefault::Raise => Err(KlippyError::config(format!(
                    "Unknown config object '{name}'"
                ))),
                LookupDefault::ReturnNone => Ok(None),
            },
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
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup(name, default)
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects
            .borrow()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        self.lookup(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers
            .borrow_mut()
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let taken = self.handlers.borrow_mut().shift_remove(event);
        let mut callbacks = taken.unwrap_or_default();
        let mut out = Vec::new();
        let mut failure = None;
        for callback in callbacks.iter_mut() {
            match callback(params) {
                Ok(value) => out.push(value),
                Err(e) => {
                    failure = Some(e);
                    break;
                }
            }
        }
        let mut map = self.handlers.borrow_mut();
        let added = map.shift_remove(event).unwrap_or_default();
        callbacks.extend(added);
        map.insert(event.to_string(), callbacks);
        match failure {
            Some(e) => Err(e),
            None => Ok(out),
        }
    }
    fn request_exit(&self, _r: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    table: Arc<Table>,
}

impl FakeConfig {
    fn make(printer: Printer, name: &str, table: Arc<Table>) -> Config {
        Arc::new(FakeConfig { printer, name: name.to_string(), table })
    }

    fn opt(&self, option: &str) -> Option<String> {
        let (_, opts) = self.table.iter().find(|(n, _)| *n == self.name)?;
        opts.iter().find(|(k, _)| k == option).map(|(_, v)| v.clone())
    }

    fn missing<T>(&self, option: &str, default: Option<Option<T>>) -> KResult<Option<T>> {
        match default {
            Some(d) => Ok(d),
            None => Err(KlippyError::config(format!(
                "Option '{option}' in section '{}' must be specified",
                self.name
            ))),
        }
    }

    fn parse_error(&self, option: &str) -> KlippyError {
        KlippyError::config(format!(
            "Unable to parse option '{option}' in section '{}'",
            self.name
        ))
    }

    fn bound_error(&self, option: &str, what: &str, bound: &str) -> KlippyError {
        KlippyError::config(format!(
            "Option '{option}' in section '{}' must {what} {bound}",
            self.name
        ))
    }
}

fn float_str(v: f64) -> String {
    PyValue::Float(v).str()
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opt(option) {
            Some(v) => Ok(Some(v)),
            None => self.missing(option, opts.default),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.opt(option) else {
            return self.missing(option, opts.default);
        };
        let v: i64 = raw.trim().parse().map_err(|_| self.parse_error(option))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.bound_error(option, "have minimum of", &m.to_string()));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.bound_error(option, "have maximum of", &m.to_string()));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let Some(raw) = self.opt(option) else {
            return self.missing(option, opts.default);
        };
        let v: f64 = raw.trim().parse().map_err(|_| self.parse_error(option))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.bound_error(option, "have minimum of", &float_str(m)));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.bound_error(option, "have maximum of", &float_str(m)));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(self.bound_error(option, "be above", &float_str(m)));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(self.bound_error(option, "be below", &float_str(m)));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        let Some(raw) = self.opt(option) else {
            return self.missing(option, opts.default);
        };
        match raw.trim().to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(Some(true)),
            "0" | "no" | "false" | "off" => Ok(Some(false)),
            _ => Err(self.parse_error(option)),
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        let chosen = match self.opt(option) {
            Some(v) => PyValue::Str(v),
            None => match opts.default {
                Some(Some(d)) => d,
                Some(None) => PyValue::None,
                None => {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.name
                    )))
                }
            },
        };
        let PyValue::List(keys) = choices else {
            panic!("the probe module passes a list of choices");
        };
        if !keys.iter().any(|k| k.str() == chosen.str()) {
            return Err(KlippyError::config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                chosen.str(),
                self.name
            )));
        }
        Ok(chosen)
    }
    fn getlists(&self, option: &str, opts: GetListsOpts) -> KResult<PyValue> {
        let Some(raw) = self.opt(option) else {
            return match opts.default {
                Some(Some(d)) => Ok(d),
                Some(None) => Ok(PyValue::None),
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.name
                ))),
            };
        };
        assert_eq!(opts.seps.len(), 2, "two separators");
        assert_eq!(opts.parser, ListParser::Float, "float points");
        let mut rows: Vec<PyValue> = Vec::new();
        if !raw.trim().is_empty() {
            for part in raw.split(opts.seps[1].as_str()).map(|p| p.trim()) {
                if part.is_empty() {
                    continue;
                }
                let mut row: Vec<PyValue> = Vec::new();
                if !part.trim().is_empty() {
                    for item in part.split(opts.seps[0].as_str()).map(|p| p.trim()) {
                        let v: f64 = item.parse().map_err(|_| self.parse_error(option))?;
                        row.push(PyValue::Float(v));
                    }
                }
                if let Some(count) = opts.count {
                    if row.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have {count} elements",
                            self.name
                        )));
                    }
                }
                rows.push(PyValue::Tuple(row));
            }
        }
        Ok(PyValue::Tuple(rows))
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, section: &str) -> Config {
        FakeConfig::make(self.printer.clone(), section, self.table.clone())
    }
    fn has_section(&self, section: &str) -> bool {
        self.table.iter().any(|(n, _)| n == section)
    }
    fn get_prefix_sections(&self, prefix: &str) -> Vec<Config> {
        self.table
            .iter()
            .filter(|(n, _)| n.starts_with(prefix))
            .map(|(n, _)| self.getsection(n))
            .collect()
    }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeGcmd {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    log: Arc<GCell<Vec<String>>>,
}

impl FakeGcmd {
    fn error(&self, msg: String) -> KlippyError {
        KlippyError::command(format!("Error on '{}': {msg}", self.commandline))
    }
}

impl GCodeCommandApi for FakeGcmd {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.error(format!("missing {name}"))),
                Some(d) => Ok(d),
            };
        };
        match opts.parser {
            GetParser::Str => Ok(PyValue::Str(value.clone())),
            GetParser::Float => value
                .parse::<f64>()
                .map(PyValue::Float)
                .map_err(|_| self.error(format!("unable to parse {value}"))),
            GetParser::Int | GetParser::IntAutoBase => value
                .parse::<i64>()
                .map(|n| PyValue::Int(PyInt::from_i64(n)))
                .map_err(|_| self.error(format!("unable to parse {value}"))),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.error(format!("missing {name}"))),
                Some(d) => Ok(d),
            };
        };
        let v: i64 = value
            .parse()
            .map_err(|_| self.error(format!("unable to parse {value}")))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.error(format!("{name} must have minimum of {m}")));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.error(format!("{name} must have maximum of {m}")));
            }
        }
        Ok(Some(v))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let Some(value) = self.params.get(name) else {
            return match opts.default {
                None => Err(self.error(format!("missing {name}"))),
                Some(d) => Ok(d),
            };
        };
        let v: f64 = value
            .parse()
            .map_err(|_| self.error(format!("unable to parse {value}")))?;
        if let Some(m) = opts.minval {
            if v < m {
                return Err(self.error(format!("{name} must have minimum of {}", float_str(m))));
            }
        }
        if let Some(m) = opts.maxval {
            if v > m {
                return Err(self.error(format!("{name} must have maximum of {}", float_str(m))));
            }
        }
        if let Some(m) = opts.above {
            if v <= m {
                return Err(self.error(format!("{name} must be above {}", float_str(m))));
            }
        }
        if let Some(m) = opts.below {
            if v >= m {
                return Err(self.error(format!("{name} must be below {}", float_str(m))));
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
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.borrow_mut().push(msg.to_string());
    }
    fn respond_raw(&self, msg: &str) {
        self.log.borrow_mut().push(msg.to_string());
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        false
    }
}

type SharedHandler = Arc<GCell<GCodeCommandHandler>>;

struct FakeGCode {
    handlers: GCell<IndexMap<String, SharedHandler>>,
    help: GCell<IndexMap<String, String>>,
    log: Arc<GCell<Vec<String>>>,
    position: Arc<GCell<Vec<f64>>>,
}

impl FakeGCode {
    fn new(position: Arc<GCell<Vec<f64>>>) -> Arc<FakeGCode> {
        Arc::new(FakeGCode {
            handlers: GCell::new(IndexMap::new()),
            help: GCell::new(IndexMap::new()),
            log: Arc::new(GCell::new(Vec::new())),
            position,
        })
    }

    fn commands(&self) -> Vec<String> {
        let mut v: Vec<String> = self.handlers.borrow().keys().cloned().collect();
        v.sort();
        v
    }

    fn make_gcmd(&self, command: &str, commandline: &str, params: IndexMap<String, String>) -> GCmd {
        Arc::new(FakeGcmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params,
            log: self.log.clone(),
        })
    }

    fn dispatch(&self, cmd: &str, params: &[(&str, &str)]) -> KResult<()> {
        let handler = self.handlers.borrow().get(cmd).cloned();
        let Some(handler) = handler else {
            panic!("command {cmd} is not registered");
        };
        let map: IndexMap<String, String> = params
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        let gcmd = self.make_gcmd(cmd, &commandline(cmd, params), map);
        let mut guard = handler.borrow_mut();
        (*guard)(&gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<GCodeCommandHandler>,
        opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        let old = self.handlers.borrow().get(cmd).cloned();
        match func {
            None => {
                self.handlers.borrow_mut().shift_remove(cmd);
                self.help.borrow_mut().shift_remove(cmd);
                // The removed handler is shared with a running call at most, so it is not
                // handed back.
                let _ = old;
                Ok(None)
            }
            Some(f) => {
                if old.is_some() {
                    return Err(KlippyError::config(format!(
                        "gcode command {cmd} already registered"
                    )));
                }
                self.handlers
                    .borrow_mut()
                    .insert(cmd.to_string(), Arc::new(GCell::new(f)));
                if let Some(d) = opts.desc {
                    self.help.borrow_mut().insert(cmd.to_string(), d);
                }
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        _c: &str,
        _k: &str,
        _v: Option<&str>,
        _f: GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        self.help.cloned()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
    fn request_restart(&self, _result: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, script: &str) -> KResult<()> {
        self.log.borrow_mut().push(format!("script: {script}"));
        if script == "MOVE" {
            self.position.borrow_mut()[0] += 1.0;
        }
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
    ) -> GCmd {
        self.make_gcmd(command, commandline, params.clone())
    }
    fn respond_raw(&self, msg: &str) {
        self.log.borrow_mut().push(msg.to_string());
    }
    fn respond_info(&self, msg: &str, _log: bool) {
        self.log.borrow_mut().push(msg.to_string());
    }
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

struct FakeConfigFile {
    writes: GCell<Vec<(String, String, String)>>,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used")
    }
    fn read_config(&self, _f: &str) -> KResult<Config> {
        panic!("not used")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("not used")
    }
    fn log_config(&self, _c: &Config) {}
    fn check_unused_options(&self, _c: &Config) -> KResult<()> {
        Ok(())
    }
    fn runtime_warning(&self, _m: &str) {}
    fn deprecate(&self, _s: &str, _o: &str, _v: Option<PyValue>, _m: Option<String>) {}
    fn deprecate_gcode(&self, _c: &str, _o: klippy_api::config::DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _m: &Mcu, _f: &str, _m2: Option<String>) {}
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn set(&self, section: &str, option: &str, value: &str) -> KResult<()> {
        self.writes
            .borrow_mut()
            .push((section.to_string(), option.to_string(), value.to_string()));
        Ok(())
    }
    fn remove_section(&self, _s: &str) {}
}

impl PrinterObject for FakeConfigFile {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterConfigApi);
}

struct FakeGCodeMove {
    offset: f64,
}

impl GCodeMoveApi for FakeGCodeMove {
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("homing_origin".to_string()),
            PyValue::Tuple(vec![
                PyValue::Float(0.0),
                PyValue::Float(0.0),
                PyValue::Float(self.offset),
                PyValue::Int(PyInt::from_i64(0)),
            ]),
        );
        PyValue::Dict(d)
    }
    fn set_move_transform(
        &self,
        _t: Option<klippy_api::gcode::MoveTransform>,
        _f: bool,
    ) -> KResult<Option<klippy_api::gcode::MoveTransform>> {
        Ok(None)
    }
    fn reset_last_position(&self) -> KResult<()> { Ok(()) }
    fn cmd_g1(&self, _gcmd: &GCmd) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGCodeMove; dyn GCodeMoveApi);

struct FakeGCodeMacro {
    printer: Printer,
    loaded: GCell<Vec<(String, String)>>,
}

impl PrinterGCodeMacroApi for FakeGCodeMacro {
    fn load_template(
        &self,
        config: &Config,
        option: &str,
        default: Option<&str>,
    ) -> KResult<TemplateWrapper> {
        self.loaded
            .borrow_mut()
            .push((config.get_name(), option.to_string()));
        let script = config
            .get(
                option,
                GetStrOpts {
                    default: default.map(|d| Some(d.to_string())),
                    note_valid: true,
                },
            )?
            .expect("script text");
        TemplateWrapper::new(&self.printer, &format!("{}:{}", config.get_name(), option), &script)
    }
    fn create_template_context(&self, _eventtime: Option<f64>) -> IndexMap<String, TemplateValue> {
        IndexMap::new()
    }
}

klippy_api::printer_object!(FakeGCodeMacro; dyn PrinterGCodeMacroApi);

struct FakeTrapq;
impl TrapqApi for FakeTrapq {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    fn extract_old(&self, _out: &mut [chelper_motion::trapq::PullMove], _max: i32, _start_time: f64, _end_time: f64) -> i32 {
        0
    }
}

struct FakeStepper {
    name: String,
}

impl McuStepperApi for FakeStepper {
    fn get_mcu(&self) -> Mcu {
        panic!("not used")
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
    fn setup_default_pulse_duration(&self, _p: f64, _b: bool) {}
    fn setup_itersolve(&self, _a: &str, _p: &[PyValue]) {}
    fn get_step_dist(&self) -> f64 {
        0.0
    }
    fn get_rotation_distance(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_rotation_distance(&self, _r: f64) {}
    fn get_dir_inverted(&self) -> (bool, bool) {
        (false, false)
    }
    fn set_dir_inverted(&self, _i: bool) -> KResult<()> {
        Ok(())
    }
    fn calc_position_from_coord(&self, _c: [f64; 3]) -> f64 {
        0.0
    }
    fn set_position(&self, _c: [f64; 3]) {}
    fn get_commanded_position(&self) -> f64 {
        0.0
    }
    fn get_mcu_position(&self, _c: Option<f64>) -> i64 {
        0
    }
    fn get_past_mcu_position(&self, _p: f64) -> i64 {
        0
    }
    fn mcu_to_commanded_position(&self, _m: f64) -> f64 {
        0.0
    }
    fn dump_steps(
        &self,
        _c: i64,
        _s: i64,
        _e: i64,
    ) -> (Vec<klippy_api::stepper::PullHistoryStep>, i64) {
        (Vec::new(), 0)
    }
    fn get_stepper_kinematics(&self) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn set_stepper_kinematics(
        &self,
        _sk: Option<klippy_api::stepper::StepperKinematics>,
    ) -> Option<klippy_api::stepper::StepperKinematics> {
        None
    }
    fn note_homing_end(&self) -> KResult<()> {
        Ok(())
    }
    fn get_trapq(&self) -> Option<Trapq> {
        None
    }
    fn set_trapq(&self, _t: Option<Trapq>) -> Option<Trapq> {
        None
    }
    fn add_active_callback(&self, _cb: Box<dyn FnMut(f64) -> KResult<()> + Send>) -> KResult<()> {
        Ok(())
    }
    fn is_active_axis(&self, axis: char) -> bool {
        self.name.ends_with(axis)
    }
}

fn fake_steppers() -> Vec<Stepper> {
    ["stepper_x", "stepper_y", "stepper_z"]
        .iter()
        .map(|n| Arc::new(FakeStepper { name: n.to_string() }) as Stepper)
        .collect()
}

struct FakeKinematics {
    position: Arc<GCell<Vec<f64>>>,
}

impl KinematicsApi for FakeKinematics {
    fn class_name(&self) -> &'static str {
        "FakeKinematics"
    }
    fn get_steppers(&self) -> Vec<Stepper> {
        fake_steppers()
    }
    fn calc_position(&self, _spos: &IndexMap<String, f64>) -> KResult<Vec<Option<f64>>> {
        Ok(self.position.borrow().iter().map(|v| Some(*v)).collect())
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn check_move(&self, _mv: &dyn klippy_api::toolhead::MoveApi) -> KResult<()> {
        Ok(())
    }
    fn home(&self, _h: HomingStateHandle) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn clear_homing_state(&self, _c: &str) -> klippy_api::KResult<()> { Ok(()) }
}

struct FakeToolhead {
    position: Arc<GCell<Vec<f64>>>,
    homed: String,
    moves: GCell<Vec<(Vec<Option<f64>>, f64)>>,
    time_calls: GCell<i64>,
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        self.position.cloned()
    }
    fn set_position(&self, newpos: Vec<f64>, _h: &str) -> KResult<()> {
        self.position.replace(newpos);
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        Arc::new(FakeKinematics { position: self.position.clone() })
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        let n = self.time_calls.get() + 1;
        self.time_calls.set(n);
        Ok(n as f64)
    }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, coord: Vec<Option<f64>>, speed: f64) -> KResult<()> {
        self.moves.borrow_mut().push((coord.clone(), speed));
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
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> {
        let mut d = IndexMap::new();
        d.insert(
            PyValue::Str("homed_axes".to_string()),
            PyValue::Str(self.homed.clone()),
        );
        Ok(PyValue::Dict(d))
    }
    fn get_trapq(&self) -> Trapq {
        Arc::new(FakeTrapq)
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _v: Option<f64>,
        _a: Option<f64>,
        _s: Option<f64>,
        _m: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(
        &self,
        _c: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("not used")
    }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("not used")
    }
}

klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

type ProbingCall = (Vec<Option<f64>>, f64, bool);
type PointsCase<'a> = (&'a str, &'a Table, Option<&'a Vec<Vec<f64>>>, &'a str);

#[derive(Clone, Debug)]
enum Step {
    Z(f64),
    Err(&'static str),
}

struct FakeHoming {
    position: Arc<GCell<Vec<f64>>>,
    script: GCell<VecDeque<Step>>,
    calls: GCell<Vec<ProbingCall>>,
    first_checks: GCell<i64>,
}

impl PrinterHomingApi for FakeHoming {
    fn manual_home(
        &self,
        _toolhead: HomingTarget,
        _endstops: Vec<(Endstop, String)>,
        _pos: Vec<Option<f64>>,
        _speed: f64,
        _probe_pos: bool,
        _triggered: bool,
        _check_triggered: bool,
    ) -> KResult<Vec<f64>> {
        panic!("manual_home is not used by the probe tests")
    }
    fn probing_move(
        &self,
        _mcu_probe: Endstop,
        pos: Vec<Option<f64>>,
        speed: f64,
        check_movement: bool,
    ) -> KResult<Vec<f64>> {
        self.calls.borrow_mut().push((pos.clone(), speed, check_movement));
        let item = self.script.borrow_mut().pop_front().expect("scripted probing move");
        match item {
            Step::Err(msg) => Err(KlippyError::command(msg)),
            Step::Z(z) => {
                let mut epos: Vec<f64> = pos.iter().map(|v| v.expect("full position")).collect();
                epos[2] = z;
                self.position.replace(epos.clone());
                Ok(epos)
            }
        }
    }
    fn check_probe_first_home(&self, gcmd: &GCmd) -> KResult<bool> {
        self.first_checks.set(self.first_checks.get() + 1);
        if gcmd.get_command() != "G28" {
            return Ok(false);
        }
        let attempt = gcmd.get(
            "HOME_ATTEMPT_NUM",
            GGetOpts { default: Some(PyValue::None), ..GGetOpts::default() },
        )?;
        Ok(matches!(attempt, PyValue::Str(s) if s == "1"))
    }
}

klippy_api::printer_object!(FakeHoming; dyn PrinterHomingApi);

struct FakeEndstop {
    added: GCell<Vec<String>>,
    queries: GCell<Vec<f64>>,
    value: GCell<i64>,
}

impl FakeEndstop {
    fn new() -> Arc<FakeEndstop> {
        Arc::new(FakeEndstop {
            added: GCell::new(Vec::new()),
            queries: GCell::new(Vec::new()),
            value: GCell::new(0),
        })
    }
}

impl EndstopApi for FakeEndstop {
    fn get_mcu(&self) -> klippy_api::KResult<Mcu> {
        panic!("not used")
    }
    fn add_stepper(&self, stepper: Stepper) {
        self.added.borrow_mut().push(stepper.get_name(None));
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
    ) -> klippy_api::KResult<Completion<bool>> {
        panic!("not used")
    }
    fn home_wait(&self, _home_end_time: f64) -> KResult<f64> {
        Ok(0.0)
    }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        self.queries.borrow_mut().push(print_time);
        Ok(self.value.get())
    }
}

struct FakePins {
    setups: GCell<Vec<(String, String)>>,
    chips: GCell<Vec<(String, Chip)>>,
    endstop: GCell<Option<Arc<FakeEndstop>>>,
}

impl FakePins {
    fn chip_names(&self) -> Vec<String> {
        self.chips.borrow().iter().map(|(name, _)| name.clone()).collect()
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _d: &str, _o: ParsePinOpts) -> KResult<PinParams> {
        panic!("not used")
    }
    fn lookup_pin(&self, _d: &str, _o: LookupPinOpts) -> KResult<PinParams> {
        panic!("not used")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setups
            .borrow_mut()
            .push((pin_type.to_string(), pin_desc.to_string()));
        let endstop = FakeEndstop::new();
        self.endstop.replace(Some(endstop.clone()));
        Ok(Pin::Endstop(endstop))
    }
    fn reset_pin_sharing(&self, _p: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _c: &str) -> KResult<PinResolver> {
        panic!("not used")
    }
    fn register_chip(&self, chip_name: &str, chip: Chip) -> KResult<()> {
        self.chips.borrow_mut().push((chip_name.to_string(), chip));
        Ok(())
    }
    fn allow_multi_use_pin(&self, _d: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeMcu;

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("not used")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> {
        panic!("not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        panic!("not used")
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        panic!("not used")
    }
    fn add_config_cmd(&self, _c: &str, _o: AddConfigCmdOpts) -> KResult<()> {
        panic!("not used")
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _o: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _t: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, _m: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("not used")
    }
    fn lookup_query_command(
        &self,
        _m: &str,
        _r: &str,
        _o: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("not used")
    }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(0)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _m: &str,
        _o: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("not used")
    }
    fn check_valid_response(&self, _m: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> {
        panic!("not used")
    }
    fn print_time_to_clock(&self, _p: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _c: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _e: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, _c: i64) -> i64 {
        0
    }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> {
        Ok((0.0, 0.0))
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("not used")
    }
}

/// `FakeProbe` of the Python setup: a probe object with a canned session.
struct FakeProbe {
    log: GCell<Vec<String>>,
}

struct FakeSession {
    owner: Arc<FakeProbe>,
}

impl ProbeSessionApi for FakeSession {
    fn run_probe(&self, gcmd: &GCmd) -> KResult<()> {
        self.owner.log.borrow_mut().push(format!("run {}", gcmd.get_command()));
        Ok(())
    }
    fn pull_probed_results(&self) -> KResult<Vec<ProbeResult>> {
        Ok(vec![ProbeResult {
            bed_x: 1.0,
            bed_y: 2.0,
            bed_z: 3.0,
            test_x: 4.0,
            test_y: 5.0,
            test_z: 6.0,
        }])
    }
    fn end_probe_session(&self) -> KResult<()> {
        self.owner.log.borrow_mut().push("end".to_string());
        Ok(())
    }
}

struct FakeProbeHandle(Arc<FakeProbe>);

impl PrinterProbeApi for FakeProbeHandle {
    fn get_probe_params(&self, _gcmd: Option<&GCmd>) -> KResult<ProbeParams> {
        Ok(ProbeParams {
            probe_speed: 5.0,
            lift_speed: 6.0,
            samples: 1,
            sample_retract_dist: 2.0,
            samples_tolerance: 0.1,
            samples_tolerance_retries: 0,
            samples_result: "average".to_string(),
        })
    }
    fn get_offsets(&self, _gcmd: Option<&GCmd>) -> KResult<(f64, f64, f64)> {
        Ok((0.5, 0.25, 1.0))
    }
    fn start_probe_session(&self, _gcmd: &GCmd) -> KResult<Arc<dyn ProbeSessionApi>> {
        self.0.log.borrow_mut().push("start".to_string());
        Ok(Arc::new(FakeSession { owner: self.0.clone() }))
    }
}

// ---------------------------------------------------------------------------
// The Rust world.
// ---------------------------------------------------------------------------

struct RustWorld {
    fp: Arc<FakePrinter>,
    printer: Printer,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    homing: Arc<FakeHoming>,
    pins: Arc<FakePins>,
    configfile: Arc<FakeConfigFile>,
    macros: Arc<FakeGCodeMacro>,
    table: Arc<Table>,
    probe: Option<Arc<PrinterProbe>>,
    _manual: Arc<ManualProbe>,
}

impl RustWorld {
    fn cfg(&self, name: &str) -> Config {
        FakeConfig::make(self.printer.clone(), name, self.table.clone())
    }

    fn add_probe(&mut self) -> KResult<()> {
        let probe = PrinterProbe::new(&self.cfg("probe"))?;
        self.fp.add("probe", probe.clone() as ObjectRef);
        self.probe = Some(probe);
        Ok(())
    }

    fn run(&self, cmd: &str, params: &[(&str, &str)]) -> KResult<()> {
        self.gcode.dispatch(cmd, params)
    }

    fn gcmd(&self, cmd: &str, params: &[(&str, &str)]) -> GCmd {
        let map: IndexMap<String, String> = params
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        self.gcode.make_gcmd(cmd, &commandline(cmd, params), map)
    }

    fn api(&self) -> Arc<dyn PrinterProbeApi> {
        self.probe.clone().expect("probe object") as Arc<dyn PrinterProbeApi>
    }

    fn script(&self, steps: &[Step]) {
        self.homing.script.borrow_mut().extend(steps.iter().cloned());
    }
}

fn rust_world(table: &Table, homed: &str, offset: f64) -> RustWorld {
    let fp = FakePrinter::new();
    let printer: Printer = fp.clone();
    let position = Arc::new(GCell::new(vec![10.0, 20.0, 5.0, 0.0]));
    let gcode = FakeGCode::new(position.clone());
    let toolhead = Arc::new(FakeToolhead {
        position: position.clone(),
        homed: homed.to_string(),
        moves: GCell::new(Vec::new()),
        time_calls: GCell::new(0),
    });
    let homing = Arc::new(FakeHoming {
        position,
        script: GCell::new(VecDeque::new()),
        calls: GCell::new(Vec::new()),
        first_checks: GCell::new(0),
    });
    let pins = Arc::new(FakePins {
        setups: GCell::new(Vec::new()),
        chips: GCell::new(Vec::new()),
        endstop: GCell::new(None),
    });
    let configfile = Arc::new(FakeConfigFile { writes: GCell::new(Vec::new()) });
    let macros = Arc::new(FakeGCodeMacro {
        printer: printer.clone(),
        loaded: GCell::new(Vec::new()),
    });
    fp.add("gcode", gcode.clone() as ObjectRef);
    fp.add("toolhead", toolhead.clone() as ObjectRef);
    fp.add("homing", homing.clone() as ObjectRef);
    fp.add("pins", pins.clone() as ObjectRef);
    fp.add("configfile", configfile.clone() as ObjectRef);
    fp.add("gcode_move", Arc::new(FakeGCodeMove { offset }) as ObjectRef);
    fp.add("gcode_macro", macros.clone() as ObjectRef);
    let table = Arc::new(table.clone());
    let manual_cfg = FakeConfig::make(printer.clone(), "manual_probe", table.clone());
    let manual = ManualProbe::new(&manual_cfg).expect("manual probe");
    fp.add("manual_probe", manual.clone() as ObjectRef);
    RustWorld {
        fp,
        printer,
        gcode,
        toolhead,
        homing,
        pins,
        configfile,
        macros,
        table,
        probe: None,
        _manual: manual,
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("probe fakes");
            Some(py)
        }
        None => {
            println!("skipping probe oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "probe".to_string(),
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

/// Both sides must fail with the same class and message, or both succeed.
fn check_both<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    match (expected, actual) {
        (Ok(_), Ok(_)) => {}
        (Err(py), Err(rs)) => {
            assert_eq!(rs.class_name(), py.class_name.as_str(), "{label}: exception class");
            assert_eq!(rs.message(), py.message.as_str(), "{label}: exception message");
        }
        (Ok(v), Err(rs)) => panic!("{label}: oracle returned {v:?} but rust failed: {rs}"),
        (Err(py), Ok(_)) => panic!(
            "{label}: oracle raised {}: {} but rust succeeded",
            py.class_name, py.message
        ),
    }
}

fn pyvalue_to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse().unwrap_or(0)),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(l) => Value::List(l.iter().map(pyvalue_to_value).collect()),
        PyValue::Tuple(t) => Value::Tuple(t.iter().map(pyvalue_to_value).collect()),
        PyValue::Dict(d) => Value::Dict(
            d.iter()
                .map(|(k, v)| (pyvalue_to_value(k), pyvalue_to_value(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::Opaque {
            type_name: "host".to_string(),
            addr: format!("{h:#x}"),
        },
    }
}

fn str_list(v: &[String]) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn floats(v: &[f64]) -> Value {
    Value::List(v.iter().map(|f| Value::float(*f)).collect())
}

fn opt_floats(v: &[Option<f64>]) -> Value {
    Value::List(
        v.iter()
            .map(|f| match f {
                Some(f) => Value::float(*f),
                None => Value::Null,
            })
            .collect(),
    )
}

fn probe_result_value(r: &ProbeResult) -> Value {
    Value::Tuple(vec![
        Value::float(r.bed_x),
        Value::float(r.bed_y),
        Value::float(r.bed_z),
        Value::float(r.test_x),
        Value::float(r.test_y),
        Value::float(r.test_z),
    ])
}

fn results_value(v: &[ProbeResult]) -> Value {
    Value::List(v.iter().map(probe_result_value).collect())
}

fn params_value(p: &ProbeParams) -> Value {
    Value::Dict(vec![
        (Value::str("probe_speed"), Value::float(p.probe_speed)),
        (Value::str("lift_speed"), Value::float(p.lift_speed)),
        (Value::str("samples"), Value::int(p.samples)),
        (Value::str("sample_retract_dist"), Value::float(p.sample_retract_dist)),
        (Value::str("samples_tolerance"), Value::float(p.samples_tolerance)),
        (
            Value::str("samples_tolerance_retries"),
            Value::int(p.samples_tolerance_retries),
        ),
        (Value::str("samples_result"), Value::str(&p.samples_result)),
    ])
}

fn offsets_value(o: (f64, f64, f64)) -> Value {
    Value::Tuple(vec![Value::float(o.0), Value::float(o.1), Value::float(o.2)])
}

fn status_value(w: &RustWorld) -> Value {
    let probe = w.probe.as_ref().expect("probe object");
    pyvalue_to_value(&PrinterObject::get_status(&**probe, 0.0).unwrap().expect("probe status"))
}

fn script_literal(steps: &[Step]) -> String {
    let items: Vec<String> = steps
        .iter()
        .map(|s| match s {
            Step::Z(z) => format!("{z:?}"),
            Step::Err(msg) => lit(msg),
        })
        .collect();
    format!("[{}]", items.join(", "))
}

/// Builds the same world on both sides (no `[probe]` object yet).
fn new_world(py: &mut pyoracle::Oracle, t: &Table, homed: &str, offset: f64) -> RustWorld {
    py.exec(&format!(
        "W = make_world({}, {}, {:?})",
        table_literal(t),
        lit(homed),
        offset
    ))
    .expect("python world");
    rust_world(t, homed, offset)
}

/// Builds the world and the `[probe]` object on both sides.
fn probe_world(py: &mut pyoracle::Oracle, t: &Table, homed: &str, offset: f64) -> RustWorld {
    let mut w = new_world(py, t, homed, offset);
    py.exec("add_probe(W)").expect("python probe");
    w.add_probe().expect("rust probe");
    w
}

fn set_script(py: &mut pyoracle::Oracle, w: &RustWorld, steps: &[Step]) {
    py.exec(&format!("W.homing.script = {}", script_literal(steps)))
        .expect("python script");
    w.homing.script.replace(VecDeque::new());
    w.script(steps);
}

fn compare_state(py: &mut pyoracle::Oracle, w: &RustWorld, label: &str) {
    check(&format!("{label}: log"), py.eval("W.g.log"), str_list(&w.gcode.log.cloned()));
    check(
        &format!("{label}: moves"),
        py.eval("W.th.moves"),
        Value::List(
            w.toolhead
                .moves
                .cloned()
                .iter()
                .map(|(c, s)| Value::Tuple(vec![opt_floats(c), Value::float(*s)]))
                .collect(),
        ),
    );
    check(
        &format!("{label}: position"),
        py.eval("W.th.position"),
        floats(&w.toolhead.position.cloned()),
    );
    check(
        &format!("{label}: homing calls"),
        py.eval("W.homing.calls"),
        Value::List(
            w.homing
                .calls
                .cloned()
                .iter()
                .map(|(p, s, c)| {
                    Value::Tuple(vec![opt_floats(p), Value::float(*s), Value::Bool(*c)])
                })
                .collect(),
        ),
    );
    check(
        &format!("{label}: first-home checks"),
        py.eval("W.homing.first_checks"),
        Value::int(w.homing.first_checks.get()),
    );
    check(
        &format!("{label}: move time calls"),
        py.eval("W.th.time_calls"),
        Value::int(w.toolhead.time_calls.get()),
    );
    check(
        &format!("{label}: configfile writes"),
        py.eval("W.cf.writes"),
        Value::List(
            w.configfile
                .writes
                .cloned()
                .iter()
                .map(|(s, o, v)| Value::Tuple(vec![Value::str(s), Value::str(o), Value::str(v)]))
                .collect(),
        ),
    );
    let queries = match w.pins.endstop.cloned() {
        Some(e) => e.queries.cloned(),
        None => Vec::new(),
    };
    check(
        &format!("{label}: endstop queries"),
        py.eval("W.pins.endstop.queries if W.pins.endstop is not None else []"),
        floats(&queries),
    );
    check(
        &format!("{label}: registered commands"),
        py.eval("sorted(W.g.handlers.keys())"),
        str_list(&w.gcode.commands()),
    );
    if w.probe.is_some() {
        check(&format!("{label}: status"), py.eval("W.probe.get_status(0.0)"), status_value(w));
    }
}

/// Runs one G-code command on both sides and compares the result and the world.
fn step(
    py: &mut pyoracle::Oracle,
    w: &RustWorld,
    label: &str,
    cmd: &str,
    params: &[(&str, &str)],
) {
    let expected = py.exec(&format!("run(W, {}, {})", lit(cmd), py_params(params)));
    let actual = w.run(cmd, params);
    check_both(label, expected, actual);
    compare_state(py, w, label);
}

/// `gcode:command_error` as the dispatcher sends it after a failed command.
fn command_error_event(py: &mut pyoracle::Oracle, w: &RustWorld, label: &str) {
    let expected = py.exec("W.p.send_event('gcode:command_error')");
    let actual = w.printer.send_event(EV_GCODE_COMMAND_ERROR, &[]);
    check_both(label, expected, actual);
    compare_state(py, w, label);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_exposes_load_config_only() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.probe"), "entry points of extras.probe");
}

#[test]
fn hint_timeout_text() {
    let Some(mut py) = oracle() else { return };
    check("HINT_TIMEOUT", py.eval("probe.HINT_TIMEOUT"), Value::str(HINT_TIMEOUT));
}

fn result_list(rows: &[[f64; 6]]) -> Vec<ProbeResult> {
    rows.iter()
        .map(|r| ProbeResult {
            bed_x: r[0],
            bed_y: r[1],
            bed_z: r[2],
            test_x: r[3],
            test_y: r[4],
            test_z: r[5],
        })
        .collect()
}

fn py_results(rows: &[[f64; 6]]) -> String {
    let items: Vec<String> = rows
        .iter()
        .map(|r| format!("manual_probe.ProbeResult({:?}, {:?}, {:?}, {:?}, {:?}, {:?})", r[0], r[1], r[2], r[3], r[4], r[5]))
        .collect();
    format!("[{}]", items.join(", "))
}

#[test]
fn calc_probe_z_average_matches_python() {
    let Some(mut py) = oracle() else { return };
    let sets: Vec<(&str, Vec<[f64; 6]>)> = vec![
        ("single", vec![[1.5, 2.5, 0.123456789, 1.0, 2.0, 0.3]]),
        (
            "two",
            vec![
                [1.0, 2.0, 0.1, 3.0, 4.0, 0.7],
                [1.5, 2.25, -0.2, 3.5, 4.75, 0.9],
            ],
        ),
        (
            "three unsorted",
            vec![
                [10.0, 20.0, 0.30000000000000004, 1.0, 1.0, 1.0],
                [10.5, 20.5, 0.1, 2.0, 2.0, 2.0],
                [10.25, 20.25, 0.2, 3.0, 3.0, 3.0],
            ],
        ),
        (
            "four with ties",
            vec![
                [1.0, 1.0, 0.5, 1.0, 1.0, 1.0],
                [2.0, 2.0, 0.25, 2.0, 2.0, 2.0],
                [3.0, 3.0, 0.5, 3.0, 3.0, 3.0],
                [4.0, 4.0, 0.125, 4.0, 4.0, 4.0],
            ],
        ),
        (
            "five",
            vec![
                [0.1, 0.2, 0.7, 0.3, 0.4, 0.5],
                [0.2, 0.3, 0.6, 0.4, 0.5, 0.6],
                [0.3, 0.4, 0.5, 0.5, 0.6, 0.7],
                [0.4, 0.5, 0.4, 0.6, 0.7, 0.8],
                [0.5, 0.6, 0.3, 0.7, 0.8, 0.9],
            ],
        ),
    ];
    for (label, rows) in &sets {
        let list = result_list(rows);
        for method in ["average", "median", "other"] {
            let expected = py.eval(&format!(
                "probe.calc_probe_z_average({}, {})",
                py_results(rows),
                lit(method)
            ));
            let actual = calc_probe_z_average(&list, method).expect("rust average");
            check(&format!("{label} {method}"), expected, probe_result_value(&actual));
        }
        let expected = py.eval(&format!("probe.calc_probe_z_average({})", py_results(rows)));
        let actual = calc_probe_z_average(&list, "average").expect("rust default method");
        check(&format!("{label} default"), expected, probe_result_value(&actual));
    }
    for method in ["average", "median"] {
        let expected = py.eval(&format!("probe.calc_probe_z_average([], {})", lit(method)));
        check_both(
            &format!("empty {method}"),
            expected,
            calc_probe_z_average(&[], method),
        );
    }
}

#[test]
fn parameter_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    let custom: &[(&str, &str)] = &[
        ("speed", "7.5"),
        ("lift_speed", "12.0"),
        ("samples", "3"),
        ("sample_retract_dist", "1.25"),
        ("samples_result", "median"),
        ("samples_tolerance", "0.05"),
        ("samples_tolerance_retries", "2"),
    ];
    let gcmds: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("no parameters", vec![]),
        (
            "all overridden",
            vec![
                ("PROBE_SPEED", "3.5"),
                ("LIFT_SPEED", "9"),
                ("SAMPLES", "4"),
                ("SAMPLE_RETRACT_DIST", "0.5"),
                ("SAMPLES_TOLERANCE", "0.01"),
                ("SAMPLES_TOLERANCE_RETRIES", "1"),
                ("SAMPLES_RESULT", "median"),
            ],
        ),
        ("unknown samples result", vec![("SAMPLES_RESULT", "mean")]),
        ("probe speed not above 0", vec![("PROBE_SPEED", "0")]),
        ("lift speed not a number", vec![("LIFT_SPEED", "abc")]),
        ("samples below minimum", vec![("SAMPLES", "0")]),
        ("retract not above 0", vec![("SAMPLE_RETRACT_DIST", "0")]),
        ("tolerance negative", vec![("SAMPLES_TOLERANCE", "-1")]),
        ("retries negative", vec![("SAMPLES_TOLERANCE_RETRIES", "-1")]),
        ("samples not an int", vec![("SAMPLES", "2.5")]),
    ];
    for (cfg_label, opts) in [("defaults", &[][..]), ("custom", custom)] {
        let t = table(opts, &[]);
        let w = new_world(&mut py, &t, "xyz", 0.0);
        py.exec("PH = probe.ProbeParameterHelper(W.cfg('probe'))").expect("python helper");
        let helper = ProbeParameterHelper::new(&w.cfg("probe")).expect("rust helper");
        let expected = py.eval("PH.get_probe_params()");
        let actual = helper.get_probe_params(None).expect("rust params");
        check(&format!("{cfg_label}: get_probe_params(None)"), expected, params_value(&actual));
        for (label, params) in &gcmds {
            let expected = py.eval(&format!(
                "PH.get_probe_params(make_gcmd(W, 'PROBE', {}))",
                py_params(params)
            ));
            let gcmd = w.gcmd("PROBE", params);
            let actual = helper.get_probe_params(Some(&gcmd));
            match (expected, actual) {
                (Ok(v), Ok(p)) => check(
                    &format!("{cfg_label}: {label}"),
                    Ok(v),
                    params_value(&p),
                ),
                (e, a) => check_both(&format!("{cfg_label}: {label}"), e, a),
            }
        }
    }
}

#[test]
fn parameter_and_offset_config_errors() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("speed not above 0", vec![("speed", "0")]),
        ("speed not a number", vec![("speed", "fast")]),
        ("lift_speed not above 0", vec![("lift_speed", "-1")]),
        ("samples below minimum", vec![("samples", "0")]),
        ("sample_retract_dist not above 0", vec![("sample_retract_dist", "0")]),
        ("unknown samples_result", vec![("samples_result", "mean")]),
        ("samples_tolerance negative", vec![("samples_tolerance", "-0.5")]),
        ("retries negative", vec![("samples_tolerance_retries", "-1")]),
        ("missing z_offset", vec![("z_offset", REMOVE)]),
        ("x_offset not a number", vec![("x_offset", "left")]),
    ];
    for (label, opts) in &cases {
        let t = table(opts, &[]);
        let w = new_world(&mut py, &t, "xyz", 0.0);
        let expected = py.exec("probe.ProbeParameterHelper(W.cfg('probe'))");
        check_both(
            &format!("{label}: parameter helper"),
            expected,
            ProbeParameterHelper::new(&w.cfg("probe")),
        );
        let expected = py.exec("probe.ProbeOffsetsHelper(W.cfg('probe'))");
        check_both(
            &format!("{label}: offsets helper"),
            expected,
            ProbeOffsetsHelper::new(&w.cfg("probe")),
        );
    }
}

#[test]
fn offsets_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    for (label, opts) in [
        ("configured", &[][..]),
        ("defaults for x and y", &[("x_offset", REMOVE), ("y_offset", REMOVE)][..]),
        ("negative z", &[("z_offset", "-0.75")][..]),
    ] {
        let t = table(opts, &[]);
        let w = new_world(&mut py, &t, "xyz", 0.0);
        py.exec("OFF = probe.ProbeOffsetsHelper(W.cfg('probe'))").expect("python offsets");
        let helper = ProbeOffsetsHelper::new(&w.cfg("probe")).expect("rust offsets");
        check(label, py.eval("OFF.get_offsets()"), offsets_value(helper.get_offsets(None)));
        let gcmd = w.gcmd("PROBE", &[]);
        check(
            &format!("{label} with gcmd"),
            py.eval("OFF.get_offsets(make_gcmd(W, 'PROBE'))"),
            offsets_value(ProbeOffsetsHelperApi::get_offsets(&*helper, Some(&gcmd))),
        );
    }
}

#[test]
fn lookup_minimum_z_matches_python() {
    let Some(mut py) = oracle() else { return };
    let printer_only = section("printer", &[("kinematics", "cartesian")]);
    let probe_section = section("probe", &[("pin", "^PA1"), ("z_offset", "1.0")]);
    let tables: Vec<(&str, Table)> = vec![
        (
            "stepper_z position_min",
            vec![
                printer_only.clone(),
                section("stepper_z", &[("position_min", "-3.5")]),
                probe_section.clone(),
            ],
        ),
        (
            "stepper_z without position_min",
            vec![
                printer_only.clone(),
                section("stepper_z", &[("step_pin", "PA0")]),
                probe_section.clone(),
            ],
        ),
        (
            "printer minimum_z_position",
            vec![
                section("printer", &[("kinematics", "cartesian"), ("minimum_z_position", "-1.25")]),
                probe_section.clone(),
            ],
        ),
        ("neither", vec![printer_only.clone(), probe_section.clone()]),
        (
            "carriage on z",
            vec![
                printer_only.clone(),
                section("carriage carriage_x", &[("axis", "x"), ("position_min", "-9.0")]),
                section("carriage carriage_z", &[("axis", "z"), ("position_min", "-2.0")]),
                probe_section.clone(),
            ],
        ),
    ];
    for (label, t) in &tables {
        let w = new_world(&mut py, t, "xyz", 0.0);
        let expected = py.eval("probe.lookup_minimum_z(W.cfg('probe'))");
        let actual = lookup_minimum_z(&w.cfg("probe")).map(Value::float);
        match (expected, actual) {
            (Ok(v), Ok(a)) => check(label, Ok(v), a),
            (e, a) => check_both(label, e, a),
        }
    }
}

#[test]
fn init_registers_commands_pin_templates_and_status() {
    let Some(mut py) = oracle() else { return };
    let t = table(&[("activate_gcode", "ACTIVATE"), ("deactivate_gcode", "DEACTIVATE")], &[]);
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    check(
        "commands",
        py.eval("sorted(W.g.handlers.keys())"),
        str_list(&w.gcode.commands()),
    );
    check(
        "help texts",
        py.eval("sorted(W.g.help.items())"),
        Value::List({
            let mut help: Vec<(String, String)> = w.gcode.get_command_help().into_iter().collect();
            help.sort();
            help.iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::str(v)]))
                .collect()
        }),
    );
    check(
        "pin setups",
        py.eval("W.pins.setups"),
        Value::List(
            w.pins
                .setups
                .cloned()
                .iter()
                .map(|(t, d)| Value::Tuple(vec![Value::str(t), Value::str(d)]))
                .collect(),
        ),
    );
    check(
        "chips",
        py.eval("W.pins.chips"),
        str_list(&w.pins.chip_names()),
    );
    check(
        "loaded templates",
        py.eval("W.macro.loaded"),
        Value::List(
            w.macros
                .loaded
                .cloned()
                .iter()
                .map(|(s, o)| Value::Tuple(vec![Value::str(s), Value::str(o)]))
                .collect(),
        ),
    );
    compare_state(&mut py, &w, "initial");
    // A second probe on the same printer registers the commands twice.
    let expected = py.exec("add_probe(W)");
    let result = PrinterProbe::new(&w.cfg("probe")).map(|_| ());
    check_both("duplicate command registration", expected, result);
}

#[test]
fn probe_config_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("missing z_offset", vec![("z_offset", REMOVE)]),
        ("missing pin", vec![("pin", REMOVE)]),
        ("speed not above 0", vec![("speed", "0")]),
        ("unknown samples_result", vec![("samples_result", "mean")]),
        ("bad deactivate_on_each_sample", vec![("deactivate_on_each_sample", "maybe")]),
    ];
    for (label, opts) in &cases {
        let t = table(opts, &[]);
        let mut w = new_world(&mut py, &t, "xyz", 0.0);
        let expected = py.exec("add_probe(W)");
        check_both(label, expected, w.add_probe());
    }
}

#[test]
fn probe_command_single_and_multiple_samples() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "PROBE", "PROBE", &[]);
    // Several samples with retract moves in between.
    set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.5625), Step::Z(0.4375)]);
    step(
        &mut py,
        &w,
        "PROBE SAMPLES=3",
        "PROBE",
        &[("SAMPLES", "3"), ("SAMPLES_TOLERANCE", "1.0"), ("SAMPLE_RETRACT_DIST", "1.0")],
    );
    set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.5625), Step::Z(0.4375)]);
    step(
        &mut py,
        &w,
        "PROBE median",
        "PROBE",
        &[
            ("SAMPLES", "3"),
            ("SAMPLES_TOLERANCE", "1.0"),
            ("SAMPLES_RESULT", "median"),
            ("PROBE_SPEED", "2.5"),
            ("LIFT_SPEED", "11.0"),
        ],
    );
    // Tolerance exceeded once, then a retry succeeds.
    set_script(
        &mut py,
        &w,
        &[Step::Z(0.5), Step::Z(0.75), Step::Z(0.5), Step::Z(0.5)],
    );
    step(
        &mut py,
        &w,
        "PROBE retry",
        "PROBE",
        &[
            ("SAMPLES", "2"),
            ("SAMPLES_TOLERANCE", "0.01"),
            ("SAMPLES_TOLERANCE_RETRIES", "1"),
        ],
    );
    // Tolerance exceeded without retries left.
    set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.75)]);
    step(
        &mut py,
        &w,
        "PROBE tolerance",
        "PROBE",
        &[("SAMPLES", "2"), ("SAMPLES_TOLERANCE", "0.01")],
    );
    command_error_event(&mut py, &w, "command_error after tolerance");
    // The failed parameter lookup leaves the session open until the command_error event.
    step(&mut py, &w, "PROBE bad samples", "PROBE", &[("SAMPLES", "0")]);
    command_error_event(&mut py, &w, "command_error after bad samples");
}

#[test]
fn probe_errors_homing_and_timeout_hint() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    step(&mut py, &w, "homing error", "PROBE", &[]);
    command_error_event(&mut py, &w, "event after homing error");
    set_script(&mut py, &w, &[Step::Err("Timeout during endstop homing")]);
    step(&mut py, &w, "timeout hint", "PROBE", &[]);
    command_error_event(&mut py, &w, "event after timeout");
    // Unhomed Z is rejected before the probing move.
    let w = probe_world(&mut py, &table(&[], &[]), "xy", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "unhomed", "PROBE", &[]);
    command_error_event(&mut py, &w, "event after unhomed");
}

#[test]
fn query_probe_and_status_fields() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "PROBE", "PROBE", &[]);
    step(&mut py, &w, "QUERY_PROBE open", "QUERY_PROBE", &[]);
    py.exec("W.pins.endstop.value = 1").expect("python endstop value");
    w.pins.endstop.cloned().expect("endstop").value.set(1);
    step(&mut py, &w, "QUERY_PROBE triggered", "QUERY_PROBE", &[]);
}

#[test]
fn command_helper_without_query_and_z_offset_commands() {
    let Some(mut py) = oracle() else { return };
    let w = new_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec(
        "FP = FakeProbe(); H = probe.ProbeCommandHelper(W.cfg('probe'), FP, None, can_set_z_offset=False)",
    )
    .expect("python helper");
    let fake = Arc::new(FakeProbe { log: GCell::new(Vec::new()) });
    let handle: Arc<FakeProbeHandle> = Arc::new(FakeProbeHandle(fake.clone()));
    let weak_handle: Weak<FakeProbeHandle> = Arc::downgrade(&handle);
    let weak: Weak<dyn PrinterProbeApi> = weak_handle;
    let helper = ProbeCommandHelper::new(&w.cfg("probe"), weak, None, false).expect("rust helper");
    compare_state(&mut py, &w, "registered");
    check(
        "help texts",
        py.eval("sorted(W.g.help.items())"),
        Value::List({
            let mut help: Vec<(String, String)> = w.gcode.get_command_help().into_iter().collect();
            help.sort();
            help.iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::str(v)]))
                .collect()
        }),
    );
    step(&mut py, &w, "QUERY_PROBE unsupported", "QUERY_PROBE", &[]);
    step(&mut py, &w, "PROBE", "PROBE", &[]);
    check("probe log", py.eval("[e[0] if len(e) == 1 else e[0] + ' ' + e[1] for e in FP.log]"), str_list(&fake.log.cloned()));
    check("helper status", py.eval("H.get_status(0.0)"), pyvalue_to_value(&helper.get_status(0.0)));
    let _ = (&handle, &helper);
}

#[test]
fn update_results_event_replaces_the_result() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec(
        "def twist(results):\n    results[0] = results[0]._replace(bed_z=results[0].bed_z + 0.25)\nW.p.register_event_handler('probe:update_results', twist)",
    )
    .expect("python handler");
    let handler: EventCallback = Box::new(|args| {
        let list = args[0]
            .as_handle::<ProbeResultList>()
            .expect("the results list handle");
        let mut results = list.borrow_mut();
        results[0].bed_z += 0.25;
        Ok(PyValue::None)
    });
    w.printer.register_event_handler(EV_PROBE_UPDATE_RESULTS, handler);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "PROBE with twist", "PROBE", &[]);
}

#[test]
fn probe_accuracy_matches_python() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(
        &mut py,
        &w,
        &[Step::Z(0.5), Step::Z(0.5625), Step::Z(0.4375), Step::Z(0.5), Step::Z(0.375)],
    );
    step(
        &mut py,
        &w,
        "PROBE_ACCURACY odd",
        "PROBE_ACCURACY",
        &[("SAMPLES", "5"), ("SAMPLE_RETRACT_DIST", "1.5")],
    );
    set_script(
        &mut py,
        &w,
        &[Step::Z(0.5), Step::Z(0.5625), Step::Z(0.4375), Step::Z(0.5)],
    );
    step(&mut py, &w, "PROBE_ACCURACY even", "PROBE_ACCURACY", &[("SAMPLES", "4")]);
    set_script(&mut py, &w, &vec![Step::Z(0.5); 10]);
    step(&mut py, &w, "PROBE_ACCURACY default", "PROBE_ACCURACY", &[]);
    step(&mut py, &w, "PROBE_ACCURACY bad samples", "PROBE_ACCURACY", &[("SAMPLES", "0")]);
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    step(&mut py, &w, "PROBE_ACCURACY failure", "PROBE_ACCURACY", &[("SAMPLES", "2")]);
    command_error_event(&mut py, &w, "event after accuracy failure");
}

#[test]
fn probe_calibrate_and_z_offset_apply() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.25)]);
    step(&mut py, &w, "PROBE_CALIBRATE", "PROBE_CALIBRATE", &[]);
    // A second calibration while the manual probe is active.
    step(&mut py, &w, "PROBE_CALIBRATE again", "PROBE_CALIBRATE", &[]);
    step(&mut py, &w, "TESTZ", "TESTZ", &[("Z", "-4.5")]);
    step(&mut py, &w, "ACCEPT", "ACCEPT", &[]);
    // Aborting writes nothing.
    set_script(&mut py, &w, &[Step::Z(0.25)]);
    step(&mut py, &w, "PROBE_CALIBRATE for abort", "PROBE_CALIBRATE", &[("SPEED", "7.0")]);
    step(&mut py, &w, "ABORT", "ABORT", &[]);
    // Z_OFFSET_APPLY_PROBE with a zero and a nonzero homing origin.
    step(&mut py, &w, "Z_OFFSET_APPLY_PROBE zero", "Z_OFFSET_APPLY_PROBE", &[]);
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.5);
    step(&mut py, &w, "Z_OFFSET_APPLY_PROBE", "Z_OFFSET_APPLY_PROBE", &[]);
    let w = probe_world(&mut py, &table(&[("z_offset", "-0.125")], &[]), "xyz", -0.0625);
    step(&mut py, &w, "Z_OFFSET_APPLY_PROBE negative", "Z_OFFSET_APPLY_PROBE", &[]);
}

#[test]
fn probe_calibrate_probe_failure_leaves_no_manual_probe() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    step(&mut py, &w, "PROBE_CALIBRATE failure", "PROBE_CALIBRATE", &[]);
    command_error_event(&mut py, &w, "event after calibrate failure");
}

#[test]
fn stow_on_each_sample_false_runs_scripts_once() {
    let Some(mut py) = oracle() else { return };
    let t = table(
        &[
            ("deactivate_on_each_sample", "False"),
            ("activate_gcode", "ACTIVATE"),
            ("deactivate_gcode", "DEACTIVATE"),
        ],
        &[],
    );
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.5)]);
    step(
        &mut py,
        &w,
        "PROBE SAMPLES=2",
        "PROBE",
        &[("SAMPLES", "2"), ("SAMPLES_TOLERANCE", "1.0")],
    );
    // A homing error keeps the probe deployed until the session ends.
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    step(&mut py, &w, "failure keeps the probe deployed", "PROBE", &[]);
    command_error_event(&mut py, &w, "command_error stows the probe");
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "PROBE after recovery", "PROBE", &[]);
}

#[test]
fn toolhead_moved_by_activate_and_deactivate_scripts() {
    let Some(mut py) = oracle() else { return };
    let t = table(&[("activate_gcode", "MOVE")], &[]);
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "activate moves the toolhead", "PROBE", &[]);
    command_error_event(&mut py, &w, "event after activate failure");
    let t = table(&[("deactivate_gcode", "MOVE")], &[]);
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    step(&mut py, &w, "deactivate moves the toolhead", "PROBE", &[]);
    command_error_event(&mut py, &w, "event after deactivate failure");
    // The deactivate script also runs after a failed descent, and its error wins.
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    step(&mut py, &w, "deactivate error replaces the homing error", "PROBE", &[]);
}

#[test]
fn probe_session_api_g28_and_state_errors() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec("GC = make_gcmd(W, 'G28', {'HOME_ATTEMPT_NUM': '1'})").expect("python gcmd");
    let gc = w.gcmd("G28", &[("HOME_ATTEMPT_NUM", "1")]);
    set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.625)]);
    // G28 probing prints nothing and skips the movement check for the first attempt.
    let expected = py.exec("S = W.probe.start_probe_session(GC); S.run_probe(GC)");
    let session = w.api().start_probe_session(&gc);
    let actual = session.as_ref().map_err(|e| e.clone()).and_then(|s| s.run_probe(&gc));
    check_both("G28 session run", expected, actual);
    let session = session.expect("rust session");
    check(
        "G28 pulled results",
        py.eval("S.pull_probed_results()"),
        results_value(&session.pull_probed_results().expect("pull")),
    );
    compare_state(&mut py, &w, "after G28 probe");
    // Starting a second session before ending the first is an internal error.
    let expected = py.exec("W.probe.start_probe_session(GC)");
    check_both("second start", expected, w.api().start_probe_session(&gc));
    let expected = py.exec("S.end_probe_session()");
    check_both("end", expected, session.end_probe_session());
    let expected = py.exec("S.end_probe_session()");
    check_both("end twice", expected, session.end_probe_session());
    let expected = py.exec("S.run_probe(GC)");
    check_both("run after end", expected, session.run_probe(&gc));
    // run_single_probe is start, run, pull and end.
    py.exec("GC2 = make_gcmd(W, 'PROBE', {})").expect("python gcmd 2");
    let gc2 = w.gcmd("PROBE", &[]);
    set_script(&mut py, &w, &[Step::Z(0.75)]);
    let expected = py.eval("probe.run_single_probe(W.probe, GC2)");
    let actual = run_single_probe(&w.api(), &gc2).expect("rust run_single_probe");
    check("run_single_probe", expected, probe_result_value(&actual));
    compare_state(&mut py, &w, "after run_single_probe");
}

#[test]
fn command_error_event_ends_a_deployed_session() {
    let Some(mut py) = oracle() else { return };
    let t = table(
        &[
            ("deactivate_on_each_sample", "False"),
            ("activate_gcode", "ACTIVATE"),
            ("deactivate_gcode", "DEACTIVATE"),
        ],
        &[],
    );
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    py.exec("GC = make_gcmd(W, 'PROBE', {})").expect("python gcmd");
    let gc = w.gcmd("PROBE", &[]);
    set_script(&mut py, &w, &[Step::Z(0.5)]);
    let expected = py.exec("S = W.probe.start_probe_session(GC); S.run_probe(GC)");
    let session = w.api().start_probe_session(&gc).expect("rust session");
    check_both("run", expected, session.run_probe(&gc));
    compare_state(&mut py, &w, "deployed");
    command_error_event(&mut py, &w, "event ends the session");
    let expected = py.exec("S.end_probe_session()");
    check_both("end after event", expected, session.end_probe_session());
    // The idle helper ignores the event.
    command_error_event(&mut py, &w, "event on an idle helper");
}

#[test]
fn descend_to_endstop_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    for always in [false, true] {
        let t = table(&[], &[]);
        let w = new_world(&mut py, &t, "xyz", 0.0);
        let py_always = if always { "True" } else { "False" };
        py.exec(&format!(
            "E = FakeEndstop('x'); OFF = probe.ProbeOffsetsHelper(W.cfg('probe')); PH = probe.ProbeParameterHelper(W.cfg('probe')); D = probe.DescendToEndstopHelper(W.cfg('probe'), E, OFF, PH, always_check_movement={py_always})"
        ))
        .expect("python descend helper");
        let endstop = FakeEndstop::new();
        let offsets = ProbeOffsetsHelper::new(&w.cfg("probe")).expect("offsets");
        let params = ProbeParameterHelper::new(&w.cfg("probe")).expect("params");
        let helper = DescendToEndstopHelper::new(
            &w.cfg("probe"),
            endstop.clone() as Endstop,
            offsets as Arc<dyn ProbeOffsetsHelperApi>,
            params as Arc<dyn ProbeParameterHelperApi>,
            always,
        )
        .expect("rust descend helper");
        set_script(&mut py, &w, &[Step::Z(0.5), Step::Z(0.25), Step::Err("Move out of range")]);
        // Plain command, first-attempt G28 and a G28 retry.
        let cases: [(&str, Vec<(&str, &str)>); 3] = [
            ("PROBE", vec![]),
            ("G28", vec![("HOME_ATTEMPT_NUM", "1")]),
            ("G28", vec![("HOME_ATTEMPT_NUM", "2")]),
        ];
        for (i, (cmd, params)) in cases.iter().enumerate() {
            py.exec(&format!("GC = make_gcmd(W, {}, {})", lit(cmd), py_params(params)))
                .expect("python gcmd");
            let gcmd = w.gcmd(cmd, params);
            let expected = py.exec("D.descend_until_trigger(GC)");
            check_both(
                &format!("always={always} case {i}"),
                expected,
                helper.descend_until_trigger(&gcmd),
            );
            compare_state(&mut py, &w, &format!("always={always} case {i}"));
        }
        check(
            "pulled trigger positions",
            py.eval("D.pull_trigger_positions()"),
            results_value(&helper.pull_trigger_positions()),
        );
        check(
            "pull empties the list",
            py.eval("D.pull_trigger_positions()"),
            results_value(&helper.pull_trigger_positions()),
        );
        set_script(&mut py, &w, &[Step::Z(0.125)]);
        py.exec("GC = make_gcmd(W, 'PROBE', {})").expect("python gcmd");
        let gcmd = w.gcmd("PROBE", &[]);
        py.exec("D.descend_until_trigger(GC); D.clear_trigger_positions()").expect("python clear");
        helper.descend_until_trigger(&gcmd).expect("rust descend");
        helper.clear_trigger_positions();
        check(
            "cleared trigger positions",
            py.eval("D.pull_trigger_positions()"),
            results_value(&helper.pull_trigger_positions()),
        );
    }
}

#[test]
fn lookup_z_steppers_adds_z_steppers_at_mcu_identify() {
    let Some(mut py) = oracle() else { return };
    let w = new_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec("E = FakeEndstop('x'); probe.LookupZSteppers(W.cfg('probe'), E.add_stepper)")
        .expect("python lookup");
    let endstop = FakeEndstop::new();
    let _lookup = LookupZSteppers::new(&w.cfg("probe"), {
        let endstop = endstop.clone();
        Box::new(move |stepper| {
            endstop.add_stepper(stepper);
            Ok(())
        })
    });
    check("before identify", py.eval("E.added"), str_list(&endstop.added.cloned()));
    py.exec("W.p.send_event('klippy:mcu_identify')").expect("python identify");
    w.printer.send_event(EV_KLIPPY_MCU_IDENTIFY, &[]).expect("rust identify");
    check("after identify", py.eval("E.added"), str_list(&endstop.added.cloned()));
}

#[test]
fn probe_endstop_wrapper_adds_z_steppers_to_the_pin() {
    let Some(mut py) = oracle() else { return };
    let w = probe_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec("W.p.send_event('klippy:mcu_identify')").expect("python identify");
    w.printer.send_event(EV_KLIPPY_MCU_IDENTIFY, &[]).expect("rust identify");
    check(
        "stepper names on the probe pin",
        py.eval("W.pins.endstop.added"),
        str_list(&w.pins.endstop.cloned().expect("endstop").added.cloned()),
    );
}

fn pin_params(pin: &str, invert: i64, pullup: i64) -> PinParams {
    PinParams {
        chip: Arc::new(FakeMcu) as Mcu,
        chip_name: "probe".to_string(),
        pin: pin.to_string(),
        invert,
        pullup,
        share_type: None,
    }
}

#[test]
fn homing_via_probe_helper_matches_python() {
    let Some(mut py) = oracle() else { return };
    let w = new_world(&mut py, &table(&[], &[]), "xyz", 0.0);
    py.exec("H = probe.HomingViaProbeHelper(W.cfg('probe'), 1.5, lambda t: 1)")
        .expect("python helper");
    let callback: extras::probe::QueryEndstopCb = Arc::new(|_t: f64| -> KResult<i64> { Ok(1) });
    let helper = HomingViaProbeHelper::new(&w.cfg("probe"), 1.5, Some(callback)).expect("rust helper");
    check(
        "registered chip",
        py.eval("W.pins.chips"),
        str_list(&w.pins.chip_names()),
    );
    let chip = w.pins.chips.cloned().first().expect("chip").1.clone();
    // The virtual endstop pin is the helper itself.
    let pin = chip.setup_pin("endstop", pin_params("z_virtual_endstop", 0, 0)).expect("setup_pin");
    let Pin::Endstop(endstop) = pin else { panic!("endstop pin") };
    check(
        "setup_pin returns the helper",
        py.eval("H.setup_pin('endstop', {'pin': 'z_virtual_endstop', 'invert': 0, 'pullup': 0}) is H"),
        Value::Bool(Arc::as_ptr(&endstop) as *const () == Arc::as_ptr(&helper) as *const ()),
    );
    for (label, pin_type, pin, invert, pullup) in [
        ("wrong type", "pwm", "z_virtual_endstop", 0, 0),
        ("wrong pin", "endstop", "other", 0, 0),
        ("inverted", "endstop", "z_virtual_endstop", 1, 0),
        ("pullup", "endstop", "z_virtual_endstop", 0, 1),
        ("pulldown", "endstop", "z_virtual_endstop", 0, -1),
    ] {
        let expected = py.exec(&format!(
            "H.setup_pin({}, {{'pin': {}, 'invert': {invert}, 'pullup': {pullup}}})",
            lit(pin_type),
            lit(pin)
        ));
        check_both(label, expected, chip.setup_pin(pin_type, pin_params(pin, invert, pullup)));
    }
    // MCU_endstop interface of the helper.
    check("get_steppers", py.eval("len(H.get_steppers())"), Value::int(endstop.get_steppers().len() as i64));
    check(
        "get_position_endstop",
        py.eval("H.get_position_endstop()"),
        Value::float(endstop.get_position_endstop().expect("position endstop")),
    );
    check("helper get_position_endstop", py.eval("H.get_position_endstop()"), Value::float(helper.get_position_endstop()));
    check(
        "query_endstop with callback",
        py.eval("int(H.query_endstop(0.25))"),
        Value::int(endstop.query_endstop(0.25).expect("query")),
    );
    let expected = py.exec("H.home_wait(1.0)");
    check_both("home_wait", expected, endstop.home_wait(1.0));
    let expected = py.exec("H.get_mcu()");
    check_both("get_mcu", expected, endstop.get_mcu());
    let expected = py.exec("H.home_start(1.0, 0.001, 4, 0.1)");
    check_both("home_start", expected, endstop.home_start(1.0, 0.001, 4, 0.1, None));
    py.exec("H.add_stepper(FakeStepper('stepper_z'))").expect("python add_stepper");
    endstop.add_stepper(fake_steppers().remove(2));
    check("add_stepper is a no-op", py.eval("len(H.get_steppers())"), Value::int(endstop.get_steppers().len() as i64));
    // Without a callback the query is always open.
    py.exec("H2 = probe.HomingViaProbeHelper(W.cfg('probe'), 2.5)").expect("python helper 2");
    let helper2 = HomingViaProbeHelper::new(&w.cfg("probe"), 2.5, None).expect("rust helper 2");
    let chips = w.pins.chips.cloned();
    let chip2 = chips.last().expect("second chip").1.clone();
    let Pin::Endstop(endstop2) = chip2
        .setup_pin("endstop", pin_params("z_virtual_endstop", 0, 0))
        .expect("setup_pin 2")
    else {
        panic!("endstop pin")
    };
    check(
        "query_endstop without callback",
        py.eval("int(H2.query_endstop(0.5))"),
        Value::int(endstop2.query_endstop(0.5).expect("query 2")),
    );
    check("second position endstop", py.eval("H2.get_position_endstop()"), Value::float(helper2.get_position_endstop()));
}

#[test]
fn probe_points_helper_automatic_mode() {
    let Some(mut py) = oracle() else { return };
    let points = "0.0, 0.0\n20.0, 0.0\n10.0, 20.0";
    let t = table(
        &[],
        &[("bed_tilt", &[("points", points), ("horizontal_move_z", "4.0"), ("speed", "60.0")])],
    );
    for (label, retry, use_offsets) in [("plain", false, false), ("retry with offsets", true, true)] {
        let w = probe_world(&mut py, &t, "xyz", 0.0);
        let log: Arc<GCell<Vec<Vec<ProbeResult>>>> = Arc::new(GCell::new(Vec::new()));
        py.exec(&format!(
            "W.cb_log = []\ndef mkcb(w, retry_first):\n    def cb(results):\n        w.cb_log.append(list(results))\n        if retry_first and len(w.cb_log) == 1:\n            return 'retry'\n        return None\n    return cb\nW.helper = probe.ProbePointsHelper(W.cfg('bed_tilt'), mkcb(W, {}))\nW.helper.use_xy_offsets({})",
            if retry { "True" } else { "False" },
            if use_offsets { "True" } else { "False" },
        ))
        .expect("python points helper");
        let callback: extras::probe::ProbeFinalizeCallback = {
            let log = log.clone();
            Box::new(move |results| {
                log.borrow_mut().push(results.to_vec());
                if retry && log.borrow().len() == 1 {
                    return Ok(PyValue::Str("retry".to_string()));
                }
                Ok(PyValue::None)
            })
        };
        let helper = ProbePointsHelper::new(&w.cfg("bed_tilt"), callback, None).expect("rust helper");
        helper.use_xy_offsets(use_offsets);
        check(
            &format!("{label}: probe_points"),
            py.eval("[tuple(p) for p in W.helper.probe_points]"),
            Value::List(
                helper
                    .probe_points()
                    .iter()
                    .map(|p| Value::Tuple(p.iter().map(|v| Value::float(*v)).collect()))
                    .collect(),
            ),
        );
        let expected = py.exec("W.helper.minimum_points(4)");
        check_both(&format!("{label}: minimum_points"), expected, helper.minimum_points(4));
        let expected = py.exec("W.helper.minimum_points(3)");
        check_both(&format!("{label}: minimum_points ok"), expected, helper.minimum_points(3));
        let steps: Vec<Step> = vec![Step::Z(0.5), Step::Z(0.625), Step::Z(0.375), Step::Z(0.5), Step::Z(0.5), Step::Z(0.5)];
        set_script(&mut py, &w, &steps);
        py.exec("GC = make_gcmd(W, 'BED_TILT_CALIBRATE', {})").expect("python gcmd");
        let gcmd = w.gcmd("BED_TILT_CALIBRATE", &[]);
        let expected = py.exec("W.helper.start_probe(GC)");
        check_both(&format!("{label}: start_probe"), expected, helper.start_probe(&gcmd));
        compare_state(&mut py, &w, label);
        check(
            &format!("{label}: callback results"),
            py.eval("W.cb_log"),
            Value::List(log.cloned().iter().map(|r| results_value(r)).collect()),
        );
        check(
            &format!("{label}: lift speed"),
            py.eval("W.helper.get_lift_speed()"),
            Value::float(helper.get_lift_speed()),
        );
    }
}

#[test]
fn probe_points_helper_horizontal_move_z_and_session_errors() {
    let Some(mut py) = oracle() else { return };
    let t = table(
        &[],
        &[("bed_tilt", &[("points", "0.0, 0.0\n20.0, 0.0"), ("horizontal_move_z", "4.0")])],
    );
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    py.exec("W.helper = probe.ProbePointsHelper(W.cfg('bed_tilt'), lambda results: None)")
        .expect("python points helper");
    let helper = ProbePointsHelper::new(&w.cfg("bed_tilt"), Box::new(|_r| Ok(PyValue::None)), None)
        .expect("rust helper");
    py.exec("GC = make_gcmd(W, 'BED_TILT_CALIBRATE', {'HORIZONTAL_MOVE_Z': '1.0'})").expect("python gcmd");
    let gcmd = w.gcmd("BED_TILT_CALIBRATE", &[("HORIZONTAL_MOVE_Z", "1.0")]);
    let expected = py.exec("W.helper.start_probe(GC)");
    check_both("horizontal_move_z below the z_offset", expected, helper.start_probe(&gcmd));
    compare_state(&mut py, &w, "after rejected start");
    // A failed probe leaves the session open until the command_error event.
    set_script(&mut py, &w, &[Step::Err("Move out of range")]);
    py.exec("GC = make_gcmd(W, 'BED_TILT_CALIBRATE', {'HORIZONTAL_MOVE_Z': '5.0'})").expect("python gcmd");
    let gcmd = w.gcmd("BED_TILT_CALIBRATE", &[("HORIZONTAL_MOVE_Z", "5.0")]);
    let expected = py.exec("W.helper.start_probe(GC)");
    check_both("homing error", expected, helper.start_probe(&gcmd));
    compare_state(&mut py, &w, "after failed probe");
    command_error_event(&mut py, &w, "event after failed points probe");
    // Update the points and ask for more than there are.
    let expected = py.exec("W.helper.update_probe_points([(1.0, 2.0)], 2)");
    check_both(
        "update_probe_points too few",
        expected,
        helper.update_probe_points(vec![vec![1.0, 2.0]], 2),
    );
    let expected = py.exec("W.helper.update_probe_points([(1.0, 2.0), (3.0, 4.0)], 2)");
    check_both(
        "update_probe_points",
        expected,
        helper.update_probe_points(vec![vec![1.0, 2.0], vec![3.0, 4.0]], 2),
    );
    check(
        "updated points",
        py.eval("[tuple(p) for p in W.helper.probe_points]"),
        Value::List(
            helper
                .probe_points()
                .iter()
                .map(|p| Value::Tuple(p.iter().map(|v| Value::float(*v)).collect()))
                .collect(),
        ),
    );
}

#[test]
fn probe_points_helper_config_points_and_defaults() {
    let Some(mut py) = oracle() else { return };
    let with_points = table(&[], &[("bed_tilt", &[("points", "1.0, 2.0\n3.0, 4.0\n5.0, 6.0")])]);
    let without_points = table(&[], &[("bed_tilt", &[("speed", "30.0")])]);
    let bad_count = table(&[], &[("bed_tilt", &[("points", "1.0, 2.0, 3.0")])]);
    let bad_speed = table(&[], &[("bed_tilt", &[("points", "1.0, 2.0"), ("speed", "0")])]);
    let default_points: Vec<Vec<f64>> = vec![vec![7.0, 8.0], vec![9.0, 10.0]];
    let empty_points: Vec<Vec<f64>> = Vec::new();
    let cases: Vec<PointsCase> = vec![
        ("points required", &without_points, None, "None"),
        ("points from config", &with_points, None, "None"),
        ("default points", &without_points, Some(&default_points), "[(7.0, 8.0), (9.0, 10.0)]"),
        ("config overrides default points", &with_points, Some(&default_points), "[(7.0, 8.0), (9.0, 10.0)]"),
        ("empty default points", &without_points, Some(&empty_points), "[]"),
        ("point with three values", &bad_count, None, "None"),
        ("speed not above 0", &bad_speed, None, "None"),
    ];
    for (label, t, defaults, py_defaults) in cases {
        let w = new_world(&mut py, t, "xyz", 0.0);
        let expected = py.exec(&format!(
            "W.helper = probe.ProbePointsHelper(W.cfg('bed_tilt'), lambda results: None, default_points={py_defaults})"
        ));
        let actual = ProbePointsHelper::new(
            &w.cfg("bed_tilt"),
            Box::new(|_r| Ok(PyValue::None)),
            defaults.cloned(),
        );
        match (expected, actual) {
            (Ok(_), Ok(helper)) => check(
                label,
                py.eval("[tuple(p) for p in W.helper.probe_points]"),
                Value::List(
                    helper
                        .probe_points()
                        .iter()
                        .map(|p| Value::Tuple(p.iter().map(|v| Value::float(*v)).collect()))
                        .collect(),
                ),
            ),
            (e, a) => check_both(label, e, a),
        }
    }
}

fn run_manual_cycle(py: &mut pyoracle::Oracle, w: &RustWorld, label: &str) {
    step(py, w, &format!("{label}: TESTZ"), "TESTZ", &[("Z", "-1.0")]);
    step(py, w, &format!("{label}: ACCEPT"), "ACCEPT", &[]);
}

#[test]
fn probe_points_helper_manual_mode() {
    let Some(mut py) = oracle() else { return };
    let t = table(
        &[],
        &[("bed_tilt", &[("points", "0.0, 0.0\n20.0, 0.0\n10.0, 20.0"), ("horizontal_move_z", "4.0")])],
    );
    // METHOD=manual with a probe present, with one retry requested by the callback.
    let w = probe_world(&mut py, &t, "xyz", 0.0);
    let log: Arc<GCell<Vec<Vec<ProbeResult>>>> = Arc::new(GCell::new(Vec::new()));
    py.exec(
        "W.cb_log = []\ndef cb(results):\n    W.cb_log.append(list(results))\n    if len(W.cb_log) == 1:\n        return 'retry'\n    return None\nW.helper = probe.ProbePointsHelper(W.cfg('bed_tilt'), cb)",
    )
    .expect("python points helper");
    let callback: extras::probe::ProbeFinalizeCallback = {
        let log = log.clone();
        Box::new(move |results| {
            log.borrow_mut().push(results.to_vec());
            if log.borrow().len() == 1 {
                return Ok(PyValue::Str("retry".to_string()));
            }
            Ok(PyValue::None)
        })
    };
    let helper = ProbePointsHelper::new(&w.cfg("bed_tilt"), callback, None).expect("rust helper");
    py.exec("GC = make_gcmd(W, 'BED_TILT_CALIBRATE', {'METHOD': 'MANUAL'})").expect("python gcmd");
    let gcmd = w.gcmd("BED_TILT_CALIBRATE", &[("METHOD", "MANUAL")]);
    let expected = py.exec("W.helper.start_probe(GC)");
    check_both("start manual", expected, helper.start_probe(&gcmd));
    compare_state(&mut py, &w, "manual start");
    // A second start while the manual probe is active.
    let expected = py.exec("W.helper.start_probe(GC)");
    check_both("start while manual probe active", expected, helper.start_probe(&gcmd));
    for round in 0..2 {
        for point in 0..3 {
            run_manual_cycle(&mut py, &w, &format!("round {round} point {point}"));
        }
        check(
            &format!("round {round}: callback results"),
            py.eval("W.cb_log"),
            Value::List(log.cloned().iter().map(|r| results_value(r)).collect()),
        );
    }
    // Aborting stops the sequence.
    py.exec("W.helper.start_probe(GC)").expect("python restart");
    helper.start_probe(&gcmd).expect("rust restart");
    step(&mut py, &w, "ABORT", "ABORT", &[]);
}

#[test]
fn probe_points_helper_without_probe_uses_manual_probing() {
    let Some(mut py) = oracle() else { return };
    let t = table(
        &[],
        &[("bed_tilt", &[("points", "0.0, 0.0\n20.0, 0.0"), ("horizontal_move_z", "3.0"), ("speed", "40.0")])],
    );
    let w = new_world(&mut py, &t, "xyz", 0.0);
    let log: Arc<GCell<Vec<Vec<ProbeResult>>>> = Arc::new(GCell::new(Vec::new()));
    py.exec(
        "W.cb_log = []\ndef cb(results):\n    W.cb_log.append(list(results))\n    return None\nW.helper = probe.ProbePointsHelper(W.cfg('bed_tilt'), cb)",
    )
    .expect("python points helper");
    let callback: extras::probe::ProbeFinalizeCallback = {
        let log = log.clone();
        Box::new(move |results| {
            log.borrow_mut().push(results.to_vec());
            Ok(PyValue::None)
        })
    };
    let helper = ProbePointsHelper::new(&w.cfg("bed_tilt"), callback, None).expect("rust helper");
    py.exec("GC = make_gcmd(W, 'BED_TILT_CALIBRATE', {'HORIZONTAL_MOVE_Z': '6.0'})").expect("python gcmd");
    let gcmd = w.gcmd("BED_TILT_CALIBRATE", &[("HORIZONTAL_MOVE_Z", "6.0")]);
    let expected = py.exec("W.helper.start_probe(GC)");
    check_both("start without a probe", expected, helper.start_probe(&gcmd));
    compare_state(&mut py, &w, "start without a probe");
    for point in 0..2 {
        step(&mut py, &w, &format!("point {point}: TESTZ"), "TESTZ", &[("Z", "-2.0")]);
        step(&mut py, &w, &format!("point {point}: ACCEPT"), "ACCEPT", &[]);
    }
    check(
        "callback results",
        py.eval("W.cb_log"),
        Value::List(log.cloned().iter().map(|r| results_value(r)).collect()),
    );
    check(
        "lift speed without a probe",
        py.eval("W.helper.get_lift_speed()"),
        Value::float(helper.get_lift_speed()),
    );
}

#[test]
fn load_config_factory_builds_the_probe_object() {
    let Some(mut py) = oracle() else { return };
    let factory = module_factory();
    let load_config = factory.load_config.expect("load_config");
    let t = table(&[], &[]);
    let w = new_world(&mut py, &t, "xyz", 0.0);
    let obj = load_config(&w.cfg("probe")).expect("factory load_config").expect("object");
    let rust: Arc<PrinterProbe> = klippy_api::downcast_arc(&obj).expect("downcast");
    py.exec("add_probe(W)").expect("python probe");
    check(
        "factory status",
        py.eval("W.probe.get_status(0.0)"),
        pyvalue_to_value(&PrinterObject::get_status(&*rust, 0.0).unwrap().expect("status")),
    );
    let gcmd = w.gcmd("PROBE", &[]);
    py.exec("GC = make_gcmd(W, 'PROBE', {})").expect("python gcmd");
    check(
        "get_probe_params",
        py.eval("W.probe.get_probe_params(GC)"),
        params_value(&PrinterProbeApi::get_probe_params(&*rust, Some(&gcmd)).expect("params")),
    );
    check(
        "get_offsets",
        py.eval("W.probe.get_offsets(GC)"),
        offsets_value(PrinterProbeApi::get_offsets(&*rust, Some(&gcmd)).expect("offsets")),
    );
    // The object exposes the probe interface to other modules.
    let interface = klippy_api::interface::<dyn PrinterProbeApi>(&obj);
    assert!(interface.is_some(), "the probe object declares dyn PrinterProbeApi");
}
