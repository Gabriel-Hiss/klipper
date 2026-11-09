//! Parity tests for `extras::led` (port of `klippy/extras/led.py`).
//!
//! Expected values come from `pyoracle` (CPython 3.8.2 with the frozen
//! `klippy/`). The Python side builds a real `led.PrinterPWMLED` /
//! `led.LEDHelper` (with the real `output_pin.PrinterTemplateEvaluator`) on
//! minimal fakes for the printer, reactor, config, MCU, `pins`, `toolhead`,
//! `gcode`, `display_template` and `gcode_macro`; the Rust side builds the
//! ported objects on fakes of the matching contract traits with identical
//! setup. Command objects are the real ones on both sides (`gcode.GCodeCommand`
//! and `klippy_core::gcode_parse::GCodeCommand`), so parameter names, defaults,
//! bounds and error texts are compared as produced by the reference parser.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test led
//! ```
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit, so
//! every expectation comes from the live oracle.

#![allow(dead_code, unused_imports, clippy::disallowed_types, clippy::new_ret_no_self)]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use extras::display::display::{DisplayGroup, DisplayTemplateApi, GlyphTable, PrinterDisplayTemplateApi};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::led::{module_factory, LEDHelper, PrinterPWMLED};
use extras_macros_templates::gcode_macro::Value;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetFloatOpts as GGetFloatOpts,
    GetIntOpts as GGetIntOpts, GetOpts as GGetOpts, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AdcCallback, AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinsApi, PwmPin, PwmPinApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{ExtraAxis, HomingTargetApi, Kinematics, LookaheadCallback, ToolHeadApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, NOW};
use recording::value::py_float_repr;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RValue};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.led as led
import gcode

_sentinel = object()

class Error(Exception):
    pass

class CommandError(Exception):
    pass

class FakeLogging:
    def __init__(self): self.msgs = []
    def exception(self, msg, *args): self.msgs.append(msg)

class FakeMutex:
    def __init__(self, reactor): self.reactor = reactor
    def __enter__(self): self.reactor.mutex_locked = True; return self
    def __exit__(self, *args): self.reactor.mutex_locked = False

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.callbacks = []
        self.timers = []
        self.mutex_locked = False
    def mutex(self, locked=False):
        self.mutex_locked = locked
        return FakeMutex(self)
    def monotonic(self): return 100.0
    def register_callback(self, cb, waketime=None):
        self.callbacks.append(cb)
        return cb
    def register_timer(self, cb, waketime=None):
        self.timers.append(cb)
        return ('timer', len(self.timers) - 1)
    def unregister_timer(self, timer):
        self.timers[timer[1]] = None
    def run_callbacks(self):
        cbs = self.callbacks
        self.callbacks = []
        for cb in cbs:
            cb(100.0)
    def run_timers(self, eventtime):
        cbs = [cb for cb in self.timers if cb is not None]
        self.timers = []
        for cb in cbs:
            cb(eventtime)

class FakeMcu:
    def __init__(self): self.min_sched = 0.02; self.max_dur = 1.0
    def min_schedule_time(self): return self.min_sched
    def max_nominal_duration(self): return self.max_dur
    def estimated_print_time(self, eventtime): return 7.0

class FakePwmPin:
    def __init__(self, mcu, name):
        self._mcu = mcu; self.name = name
        self.calls = []; self.sets = []
    def get_mcu(self): return self._mcu
    def setup_max_duration(self, v): self.calls.append(('maxdur', v))
    def setup_cycle_time(self, ct, hw): self.calls.append(('cycle', ct, hw))
    def setup_start_value(self, sv, sh): self.calls.append(('start', sv, sh))
    def set_pwm(self, pt, v): self.sets.append((pt, v))

class FakePins:
    def __init__(self, mcu): self.mcu = mcu; self.setup = []; self.pins = []
    def setup_pin(self, pin_type, pin_desc):
        self.setup.append((pin_type, pin_desc))
        pin = FakePwmPin(self.mcu, pin_desc)
        self.pins.append(pin)
        return pin

class FakeToolhead:
    def __init__(self): self.lookahead = []
    def register_lookahead_callback(self, cb): self.lookahead.append(cb)
    def run_lookahead(self, print_time):
        cbs = self.lookahead
        self.lookahead = []
        for cb in cbs:
            cb(print_time)

class FakeGCode:
    def __init__(self): self.mux = []; self.handlers = {}
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))
        self.handlers[cmd] = func

class FakeDispatch:
    def respond_info(self, msg, log=True): pass
    def respond_raw(self, msg): pass

class FakePrinter:
    def __init__(self):
        self.reactor = FakeReactor()
        self.mcu = FakeMcu()
        self.pins = FakePins(self.mcu)
        self.toolhead = FakeToolhead()
        self.gcode = FakeGCode()
        self.objs = {}
        self.logs = []
    def get_reactor(self): return self.reactor
    def lookup_object(self, name, default=_sentinel):
        objs = {'mcu': self.mcu, 'pins': self.pins,
                'toolhead': self.toolhead, 'gcode': self.gcode}
        if name in objs: return objs[name]
        if name in self.objs: return self.objs[name]
        if default is _sentinel:
            raise Error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj): self.objs[name] = obj
    def load_object(self, config, section, default=_sentinel):
        if section in self.objs: return self.objs[section]
        if default is _sentinel:
            raise Error("Unable to load module '%s'" % (section,))
        return default
    command_error = CommandError
    config_error = Error

class FakeDisplayTemplate:
    def __init__(self, text): self.text = text; self.renders = 0
    def get_params(self): return {}
    def render(self, context, **kwargs):
        self.renders += 1
        return self.text

class FakeDisplayTemplates:
    def __init__(self, templates): self.templates = dict(templates)
    def get_display_templates(self): return dict(self.templates)

class FakeMacro:
    def __init__(self): self.contexts = []
    def create_template_context(self, eventtime):
        self.contexts.append(eventtime)
        return {'eventtime': eventtime}

class FakeConfig:
    def __init__(self, printer, name, values):
        self.printer = printer; self.name = name
        self.values = dict(values); self.reads = []
    def get_printer(self): return self.printer
    def get_name(self): return self.name
    def _value(self, option, default):
        if option in self.values: return self.values[option]
        if default is not _sentinel: return default
        raise Error("Could not find option '%s' in section '%s'"
                    % (option, self.name))
    def get(self, option, default=_sentinel):
        self.reads.append(('get', option, None if default is _sentinel else default))
        return self._value(option, default)
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None):
        self.reads.append(('getfloat', option,
                           None if default is _sentinel else default,
                           minval, maxval, above, below))
        v = float(self._value(option, default))
        if minval is not None and v < minval:
            raise Error("Option '%s' in section '%s' must have minimum of %s"
                        % (option, self.name, minval))
        if maxval is not None and v > maxval:
            raise Error("Option '%s' in section '%s' must have maximum of %s"
                        % (option, self.name, maxval))
        if above is not None and v <= above:
            raise Error("Option '%s' in section '%s' must be above %s"
                        % (option, self.name, above))
        if below is not None and v >= below:
            raise Error("Option '%s' in section '%s' must be below %s"
                        % (option, self.name, below))
        return v
    def getboolean(self, option, default=_sentinel):
        self.reads.append(('getboolean', option,
                           None if default is _sentinel else default))
        return bool(self._value(option, default))
    def error(self, msg): return Error(msg)

def new_printer():
    printer = FakePrinter()
    printer.objs['display_template'] = FakeDisplayTemplates({})
    printer.objs['gcode_macro'] = FakeMacro()
    return printer

def make_pwmled(values):
    printer = new_printer()
    config = FakeConfig(printer, 'led myled', values)
    obj = led.PrinterPWMLED(config)
    return printer, config, obj

def make_helper(values, count):
    printer = new_printer()
    config = FakeConfig(printer, 'led myled', values)
    updates = []
    def update(led_state, print_time):
        updates.append(([list(c) for c in led_state], print_time))
    helper = led.LEDHelper(config, update, count)
    return printer, config, updates, helper

def make_gcmd(command, commandline, params):
    return gcode.GCodeCommand(FakeDispatch(), command, commandline,
                              dict(params), False)

def install_template(name, text):
    tmpl = FakeDisplayTemplate(text)
    printer = new_printer()
    printer.objs['display_template'] = FakeDisplayTemplates({name: tmpl})
    return printer
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut m = self.objects.lock().unwrap();
        if m.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Duplicate object '{}'",
                name
            )));
        }
        m.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        if let Some(o) = m.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{}'",
                name
            ))),
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects
            .lock()
            .unwrap()
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
    }
    fn load_object(
        &self,
        _c: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        if let Some(o) = m.get(section) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unable to load module '{}'",
                section
            ))),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _cb: EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _r: &str) {}
}

// --- Config ---

struct FakeConfig {
    printer: Printer,
    section: String,
    values: IndexMap<String, String>,
    reads: Mutex<Vec<RValue>>,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, pairs: &[(&str, &str)]) -> Arc<FakeConfig> {
        let mut values = IndexMap::new();
        for (k, v) in pairs {
            values.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            values,
            reads: Mutex::new(Vec::new()),
        })
    }

    fn record(&self, value: RValue) {
        self.reads.lock().unwrap().push(value);
    }

    fn value(&self, option: &str, default: Option<Option<&str>>) -> KResult<Option<String>> {
        if let Some(v) = self.values.get(option) {
            return Ok(Some(v.clone()));
        }
        match default {
            Some(d) => Ok(d.map(|s| s.to_string())),
            None => Err(KlippyError::config(format!(
                "Could not find option '{}' in section '{}'",
                option, self.section
            ))),
        }
    }
}

fn rv_opt_float(v: Option<f64>) -> RValue {
    match v {
        Some(x) => RValue::float(x),
        None => RValue::Null,
    }
}

fn rv_opt_str(v: Option<&str>) -> RValue {
    match v {
        Some(s) => RValue::str(s),
        None => RValue::Null,
    }
}

fn rv_opt_bool(v: Option<bool>) -> RValue {
    match v {
        Some(b) => RValue::Bool(b),
        None => RValue::Null,
    }
}

fn parse_float(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(0.)
}

fn parse_bool(s: &str) -> bool {
    matches!(s.trim(), "1" | "true" | "True" | "yes" | "on")
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let default = opts.default.as_ref().map(|d| d.as_deref());
        self.record(RValue::Tuple(vec![
            RValue::str("get"),
            RValue::str(option),
            rv_opt_str(default.flatten()),
        ]));
        self.value(option, default)
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> {
        panic!("getint unused in led")
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        self.record(RValue::Tuple(vec![
            RValue::str("getfloat"),
            RValue::str(option),
            rv_opt_float(opts.default.flatten()),
            rv_opt_float(opts.minval),
            rv_opt_float(opts.maxval),
            rv_opt_float(opts.above),
            rv_opt_float(opts.below),
        ]));
        let v = match self.values.get(option) {
            Some(s) => parse_float(s),
            None => match opts.default {
                Some(Some(d)) => d,
                Some(None) => return Ok(None),
                None => {
                    return Err(KlippyError::config(format!(
                        "Could not find option '{}' in section '{}'",
                        option, self.section
                    )))
                }
            },
        };
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have minimum of {}",
                    option,
                    self.section,
                    py_float_repr(mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have maximum of {}",
                    option,
                    self.section,
                    py_float_repr(mx)
                )));
            }
        }
        if let Some(a) = opts.above {
            if v <= a {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be above {}",
                    option,
                    self.section,
                    py_float_repr(a)
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be below {}",
                    option,
                    self.section,
                    py_float_repr(b)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.record(RValue::Tuple(vec![
            RValue::str("getboolean"),
            RValue::str(option),
            rv_opt_bool(opts.default.flatten()),
        ]));
        match self.values.get(option) {
            Some(s) => Ok(Some(parse_bool(s))),
            None => match opts.default {
                Some(d) => Ok(d),
                None => Err(KlippyError::config(format!(
                    "Could not find option '{}' in section '{}'",
                    option, self.section
                ))),
            },
        }
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice unused in led")
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists unused in led")
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist unused in led")
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist unused in led")
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist unused in led")
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection unused in led")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _p: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) {}
}

// --- MCU / pins ---

struct FakeMcu {
    min_sched: f64,
    max_dur: f64,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("get_printer unused")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> {
        panic!("MCU setup_pin unused")
    }
    fn create_oid(&self) -> KResult<i64> {
        Ok(1)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, _c: &str, _o: AddConfigCmdOpts) -> KResult<()> {
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _o: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, _t: f64) -> i64 {
        0
    }
    fn min_schedule_time(&self) -> f64 {
        self.min_sched
    }
    fn max_nominal_duration(&self) -> f64 {
        self.max_dur
    }
    fn lookup_command(&self, _f: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        panic!("unused")
    }
    fn lookup_query_command(
        &self,
        _f: &str,
        _r: &str,
        _o: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        panic!("unused")
    }
    fn try_lookup_command(&self, _f: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _m: &str,
        _o: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("unused")
    }
    fn check_valid_response(&self, _f: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> {
        Ok(0.)
    }
    fn print_time_to_clock(&self, _t: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, c: f64) -> f64 {
        c
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        7.0
    }
    fn clock32_to_clock64(&self, c: i64) -> i64 {
        c
    }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> {
        Ok((0., 0.))
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("unused")
    }
}

type PinCalls = Mutex<Vec<RValue>>;

struct FakePwmPin {
    mcu: Mcu,
    calls: PinCalls,
    sets: Mutex<Vec<(f64, f64)>>,
}

impl PwmPinApi for FakePwmPin {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone()
    }
    fn setup_max_duration(&self, v: f64) {
        self.calls
            .lock()
            .unwrap()
            .push(RValue::Tuple(vec![RValue::str("maxdur"), RValue::float(v)]));
    }
    fn setup_cycle_time(&self, ct: f64, hw: Option<bool>) {
        self.calls.lock().unwrap().push(RValue::Tuple(vec![
            RValue::str("cycle"),
            RValue::float(ct),
            RValue::Bool(hw.unwrap_or(false)),
        ]));
    }
    fn setup_start_value(&self, a: f64, b: f64) {
        self.calls.lock().unwrap().push(RValue::Tuple(vec![
            RValue::str("start"),
            RValue::float(a),
            RValue::float(b),
        ]));
    }
    fn next_aligned_print_time(&self, pt: f64, _a: Option<f64>) -> f64 {
        pt
    }
    fn set_pwm(&self, pt: f64, v: f64) {
        self.sets.lock().unwrap().push((pt, v));
    }
}

struct FakePins {
    mcu: Mcu,
    setups: Mutex<Vec<RValue>>,
    pwms: Mutex<Vec<Arc<FakePwmPin>>>,
}

impl FakePins {
    fn new(mcu: Mcu) -> Arc<FakePins> {
        Arc::new(FakePins {
            mcu,
            setups: Mutex::new(Vec::new()),
            pwms: Mutex::new(Vec::new()),
        })
    }

    fn pins(&self) -> Vec<Arc<FakePwmPin>> {
        self.pwms.lock().unwrap().clone()
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _d: &str, _o: ParsePinOpts) -> KResult<PinParams> {
        panic!("unused")
    }
    fn lookup_pin(&self, _d: &str, _o: LookupPinOpts) -> KResult<PinParams> {
        panic!("unused")
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setups.lock().unwrap().push(RValue::Tuple(vec![
            RValue::str(pin_type),
            RValue::str(pin_desc),
        ]));
        let pin = Arc::new(FakePwmPin {
            mcu: self.mcu.clone(),
            calls: Mutex::new(Vec::new()),
            sets: Mutex::new(Vec::new()),
        });
        self.pwms.lock().unwrap().push(pin.clone());
        Ok(Pin::Pwm(pin))
    }
    fn reset_pin_sharing(&self, _p: PinParams) {}
    fn get_pin_resolver(&self, _c: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("unused")
    }
    fn register_chip(&self, _c: &str, _c2: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _d: &str) -> KResult<()> {
        Ok(())
    }
}

// --- toolhead / gcode ---

struct FakeToolhead {
    lookahead: Mutex<Vec<LookaheadCallback>>,
}

impl FakeToolhead {
    fn run_lookahead(&self, print_time: f64) {
        let cbs: Vec<LookaheadCallback> = std::mem::take(&mut *self.lookahead.lock().unwrap());
        for mut cb in cbs {
            cb(print_time).expect("lookahead callback");
        }
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _p: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) {}
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_trapq(&self) -> Trapq {
        panic!("unused")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0., 0.)
    }
    fn set_max_velocities(
        &self,
        _a: Option<f64>,
        _b: Option<f64>,
        _c: Option<f64>,
        _d: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0., 0., 0., 0.)
    }
    fn register_lookahead_callback(&self, cb: LookaheadCallback) -> KResult<()> {
        self.lookahead.lock().unwrap().push(cb);
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0., 0., false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: klippy_api::toolhead::PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("unused")
    }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) {}
    fn remove_extra_axis(&self, _e: &ExtraAxis) {}
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (false, String::new())
    }
    fn stepper_factory(&self) -> StepperFactory {
        panic!("unused")
    }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        vec![0., 0., 0., 0.]
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> Kinematics {
        panic!("unused")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> f64 {
        0.
    }
    fn dwell(&self, _d: f64) {}
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &reactor::Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

struct FakeGCode {
    mux: Mutex<Vec<RValue>>,
    handlers: Mutex<Vec<(String, GCodeCommandHandler)>>,
}

impl FakeGCode {
    fn invoke(&self, cmd: &str, gcmd: &GCmd) -> KResult<()> {
        let mut handlers = self.handlers.lock().unwrap();
        let (_, func) = handlers
            .iter_mut()
            .find(|(name, _)| name == cmd)
            .expect("command is registered");
        func(gcmd)
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _c: &str,
        _f: Option<GCodeCommandHandler>,
        _o: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.lock().unwrap().push(RValue::Tuple(vec![
            RValue::str(cmd),
            RValue::str(key),
            rv_opt_str(value),
            rv_opt_str(desc),
        ]));
        self.handlers
            .lock()
            .unwrap()
            .push((cmd.to_string(), func));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _cb: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> {
        Ok(())
    }
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("unused")
    }
    fn create_gcode_command(
        &self,
        _c: &str,
        _cl: &str,
        _p: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("unused")
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

/// `gcode.GCodeCommand` on top of the ported parse layer, so parameter names,
/// defaults, bounds and error texts are the reference implementation's.
struct RealGCommand {
    inner: klippy_core::gcode_parse::GCodeCommand,
}

impl GCodeCommandApi for RealGCommand {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match opts.parser {
            klippy_api::gcode::GetParser::Str => {
                let default = opts.default.as_ref().map(|d| match d {
                    PyValue::None => None,
                    other => Some(other.str()),
                });
                match self
                    .inner
                    .get_str_opt(name, default)
                    .map_err(|e| KlippyError::command(e.0))?
                {
                    Some(v) => Ok(PyValue::Str(v)),
                    None => Ok(PyValue::None),
                }
            }
            _ => panic!("only str parsing is used by led"),
        }
    }
    fn get_int(&self, name: &str, opts: GGetIntOpts) -> KResult<Option<i64>> {
        self.inner
            .get_int_opt(name, opts.default, opts.minval, opts.maxval)
            .map_err(|e| KlippyError::command(e.0))
    }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        self.inner
            .get_float_opt(
                name,
                opts.default,
                opts.minval,
                opts.maxval,
                opts.above,
                opts.below,
            )
            .map_err(|e| KlippyError::command(e.0))
    }
    fn get_command(&self) -> String {
        self.inner.get_command().to_string()
    }
    fn get_commandline(&self) -> String {
        self.inner.get_commandline().to_string()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.inner.get_command_parameters().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        self.inner.get_raw_command_parameters()
    }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool {
        false
    }
}

fn gcmd(command: &str, commandline: &str, params: &[(&str, &str)]) -> GCmd {
    let mut p = IndexMap::new();
    for (k, v) in params {
        p.insert(k.to_string(), v.to_string());
    }
    Arc::new(RealGCommand {
        inner: klippy_core::gcode_parse::GCodeCommand::new(
            command.to_string(),
            commandline.to_string(),
            p,
            false,
        ),
    })
}

// --- display / macro ---

struct FakeDisplayTemplate {
    text: String,
    renders: Mutex<usize>,
}

impl DisplayTemplateApi for FakeDisplayTemplate {
    fn get_params(&self) -> IndexMap<String, PyValue> {
        IndexMap::new()
    }
    fn render_values(
        &self,
        _context: &IndexMap<String, Value>,
        _kwargs: IndexMap<String, Value>,
    ) -> KResult<String> {
        *self.renders.lock().unwrap() += 1;
        Ok(self.text.clone())
    }
}

struct FakeDisplayTemplates {
    map: Mutex<IndexMap<String, Arc<FakeDisplayTemplate>>>,
}

impl PrinterDisplayTemplateApi for FakeDisplayTemplates {
    fn get_display_templates(&self) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        let mut out = IndexMap::new();
        for (k, v) in self.map.lock().unwrap().iter() {
            let handle: Arc<dyn DisplayTemplateApi> = v.clone();
            out.insert(k.clone(), handle);
        }
        out
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        IndexMap::new()
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        IndexMap::new()
    }
}

struct FakeMacro {
    contexts: Mutex<Vec<f64>>,
}

impl PrinterGCodeMacroApi for FakeMacro {
    fn load_template(&self, _config: &Config, _option: &str, _default: Option<&str>) -> KResult<TemplateWrapper> {
        unreachable!("led never loads templates")
    }
    fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value> {
        let eventtime = eventtime.expect("eventtime");
        self.contexts.lock().unwrap().push(eventtime);
        let mut out = IndexMap::new();
        out.insert(
            "eventtime".to_string(),
            Value::py(PyValue::Float(eventtime)),
        );
        out
    }
}

klippy_api::printer_object!(FakeDisplayTemplates; dyn PrinterDisplayTemplateApi);
klippy_api::printer_object!(FakeMacro; dyn PrinterGCodeMacroApi);
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakePins; dyn PinsApi);
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);
klippy_api::printer_object!(FakeMcu; dyn McuApi);
klippy_api::printer_object!(FakePrinter);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("led fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping led oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary \
                 and run with the repo checkout available"
            );
            None
        }
    }
}

fn record(label: &str) -> Record {
    Record {
        v: 1,
        unit: "led".to_string(),
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

fn check(label: &str, expected: pyoracle::OracleResult, actual: RValue) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Err(d) = assert_equal(&record(label), "ret", &expected, &actual) {
        panic!("{label}: {d}");
    }
}

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but the rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(
        actual.class_name(),
        oracle_exc.class_name.as_str(),
        "{label}: exception class"
    );
    assert_eq!(
        actual.message(),
        oracle_exc.message.as_str(),
        "{label}: exception message"
    );
}

fn rv_colors(state: &[Vec<f64>]) -> RValue {
    RValue::List(
        state
            .iter()
            .map(|c| RValue::Tuple(c.iter().map(|v| RValue::float(*v)).collect()))
            .collect(),
    )
}

fn rv_sets(sets: &[(f64, f64)]) -> RValue {
    RValue::List(
        sets.iter()
            .map(|(pt, v)| RValue::Tuple(vec![RValue::float(*pt), RValue::float(*v)]))
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Rust world.
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    pins: Arc<FakePins>,
    toolhead: Arc<FakeToolhead>,
    gcode: Arc<FakeGCode>,
    reactor: Reactor,
    handle: kos::thread::JoinHandle,
}

impl World {
    fn config(&self, pairs: &[(&str, &str)]) -> (Config, Arc<FakeConfig>) {
        let cfg = FakeConfig::new(self.printer_api.clone(), "led myled", pairs);
        let dyn_cfg: Config = cfg.clone();
        (dyn_cfg, cfg)
    }

    fn pin_sets(&self) -> Vec<Vec<(f64, f64)>> {
        self.pins
            .pins()
            .iter()
            .map(|p| p.sets.lock().unwrap().clone())
            .collect()
    }

    fn pin_calls(&self) -> Vec<RValue> {
        self.pins
            .pins()
            .iter()
            .map(|p| RValue::List(p.calls.lock().unwrap().clone()))
            .collect()
    }

    /// `end()` only clears the process flag, so the dispatcher stays in its
    /// poll until something writes the self-pipe. Poke it, wait for `run()`
    /// to return, and only then finalize.
    fn stop(self) {
        self.reactor.end();
        poke(&self.reactor);
        self.handle.join();
        self.reactor.finalize();
    }
}

fn start_reactor(reactor: &Reactor) -> kos::thread::JoinHandle {
    let rr = reactor.clone();
    let handle = kos::thread::spawn(move || rr.run());
    // Wait until the dispatch thread is armed.
    let (tx, rx) = kos::sync::channel();
    for _ in 0..200 {
        let t = tx.clone();
        reactor.register_async_callback(move |et| t.put(et), NOW);
        if rx.get_timeout(0.05).is_some() {
            break;
        }
    }
    handle
}

fn world() -> World {
    let mcu = Arc::new(FakeMcu { min_sched: 0.02, max_dur: 1.0 });
    let pins = FakePins::new(mcu.clone() as Mcu);
    let toolhead = Arc::new(FakeToolhead { lookahead: Mutex::new(Vec::new()) });
    let gcode = Arc::new(FakeGCode {
        mux: Mutex::new(Vec::new()),
        handlers: Mutex::new(Vec::new()),
    });
    let gcode_macro = Arc::new(FakeMacro { contexts: Mutex::new(Vec::new()) });
    let templates = Arc::new(FakeDisplayTemplates { map: Mutex::new(IndexMap::new()) });
    let reactor = Reactor::new(false);
    let printer = Arc::new(FakePrinter {
        objects: Mutex::new(IndexMap::new()),
        reactor: reactor.clone(),
    });
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("pins", pins.clone() as ObjectRef);
    printer.add("toolhead", toolhead.clone() as ObjectRef);
    printer.add("mcu", mcu.clone() as ObjectRef);
    printer.add("gcode_macro", gcode_macro as ObjectRef);
    printer.add("display_template", templates as ObjectRef);
    let handle = start_reactor(&reactor);
    World {
        printer: printer.clone(),
        printer_api: printer,
        mcu,
        pins,
        toolhead,
        gcode,
        reactor,
        handle,
    }
}

/// Wakes the reactor thread: timers registered from a non-greenlet thread do
/// not write the self-pipe, so a test that schedules work from the main thread
/// must poke the dispatch loop the way `register_async_callback` does.
fn poke(reactor: &Reactor) {
    reactor.register_async_callback(|_| {}, NOW);
}

fn wait_for<F: Fn() -> bool>(reactor: &Reactor, pred: F) -> bool {
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if pred() {
            return true;
        }
        poke(reactor);
        kos::time::sleep(0.005);
    }
    pred()
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_registers_prefix_only() {
    let factory = module_factory();
    assert!(factory.load_config_prefix.is_some(), "load_config_prefix");
    assert!(factory.load_config.is_none(), "load_config");
    assert!(
        factory.add_early_printer_objects.is_none(),
        "add_early_printer_objects"
    );
    assert!(
        factory.add_printer_objects.is_none(),
        "add_printer_objects"
    );
}

#[test]
fn construction_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1', 'blue_pin': 'gpio2', \
         'initial_RED': '0.5', 'initial_GREEN': '0.25', 'cycle_time': '0.02', \
         'hardware_pwm': '1'})",
    )
    .expect("python construction");
    let expected = py
        .eval(
            "(config.reads, printer.pins.setup, [p.calls for p in printer.pins.pins], \
             printer.gcode.mux, obj.get_status()['color_data'])",
        )
        .expect("python construction values");

    let w = world();
    let (cfg, cfg_fake) = w.config(&[
        ("red_pin", "gpio1"),
        ("blue_pin", "gpio2"),
        ("initial_RED", "0.5"),
        ("initial_GREEN", "0.25"),
        ("cycle_time", "0.02"),
        ("hardware_pwm", "1"),
    ]);
    let factory = module_factory();
    let obj = factory.load_config_prefix.expect("load_config_prefix")(&cfg)
        .expect("construction")
        .expect("object");
    let led: Arc<PrinterPWMLED> = klippy_api::downcast_arc(&obj).expect("PrinterPWMLED");
    let actual = RValue::Tuple(vec![
        RValue::List(cfg_fake.reads.lock().unwrap().clone()),
        RValue::List(w.pins.setups.lock().unwrap().clone()),
        RValue::List(w.pin_calls()),
        RValue::List(w.gcode.mux.lock().unwrap().clone()),
        rv_colors(&led.led_helper().color_data()),
    ]);
    check("construction", Ok(expected), actual);
    w.stop();
}

#[test]
fn construction_defaults_match() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer, config, obj = make_pwmled({'white_pin': 'gpio4'})")
        .expect("python construction");
    let expected = py
        .eval(
            "(config.reads, printer.pins.setup, [p.calls for p in printer.pins.pins], \
             obj.get_status()['color_data'])",
        )
        .expect("python construction values");

    let w = world();
    let (cfg, cfg_fake) = w.config(&[("white_pin", "gpio4")]);
    let factory = module_factory();
    let obj = factory.load_config_prefix.expect("load_config_prefix")(&cfg)
        .expect("construction")
        .expect("object");
    let led: Arc<PrinterPWMLED> = klippy_api::downcast_arc(&obj).expect("PrinterPWMLED");
    let actual = RValue::Tuple(vec![
        RValue::List(cfg_fake.reads.lock().unwrap().clone()),
        RValue::List(w.pins.setups.lock().unwrap().clone()),
        RValue::List(w.pin_calls()),
        rv_colors(&led.led_helper().color_data()),
    ]);
    check("construction_defaults", Ok(expected), actual);
    w.stop();
}

#[test]
fn no_pin_definitions_error_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec("printer = new_printer()\n\
             config = FakeConfig(printer, 'led myled', {})\n\
             def build():\n\
             \x20   led.PrinterPWMLED(config)\n")
        .expect("python setup");
    let expected = py.eval("build()");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[]);
    let result = PrinterPWMLED::new(&cfg);
    match result {
        Ok(_) => panic!("no_pin_definitions: rust constructed an object"),
        Err(e) => check_err("no_pin_definitions", expected, &e),
    }
    w.stop();
}

#[test]
fn cycle_time_above_error_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = new_printer()\n\
         config = FakeConfig(printer, 'led myled', {'red_pin': 'gpio1', 'cycle_time': '1.5'})\n\
         def build():\n\
         \x20   led.PrinterPWMLED(config)\n",
    )
    .expect("python setup");
    let expected = py.eval("build()");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1"), ("cycle_time", "1.5")]);
    match PrinterPWMLED::new(&cfg) {
        Ok(_) => panic!("cycle_time_above: rust constructed an object"),
        Err(e) => check_err("cycle_time_above", expected, &e),
    }
    w.stop();
}

#[test]
fn set_led_sync_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1', 'green_pin': 'gpio2'})\n\
         gcmd = make_gcmd('SET_LED', 'SET_LED RED=0.5 GREEN=0.25', \
                          {'RED': '0.5', 'GREEN': '0.25'})\n\
         printer.gcode.handlers['SET_LED'](gcmd)\n\
         printer.toolhead.run_lookahead(10.0)\n\
         printer.reactor.run_callbacks()\n",
    )
    .expect("python set_led");
    let expected = py
        .eval("(obj.get_status()['color_data'], [p.sets for p in printer.pins.pins])")
        .expect("python set_led values");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1"), ("green_pin", "gpio2")]);
    let obj = PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED",
        "SET_LED RED=0.5 GREEN=0.25",
        &[("RED", "0.5"), ("GREEN", "0.25")],
    );
    w.gcode.invoke("SET_LED", &command).expect("SET_LED");
    w.toolhead.run_lookahead(10.0);
    assert!(
        wait_for(&w.reactor, || w.pin_sets().iter().any(|s| !s.is_empty())),
        "transmit callback did not run"
    );
    let sets: Vec<RValue> = w.pin_sets().iter().map(|s| rv_sets(s)).collect();
    // Stop the reactor before reading the color state: the render/transmit
    // callbacks run on the reactor thread.
    w.stop();
    let actual = RValue::Tuple(vec![
        rv_colors(&obj.led_helper().color_data()),
        RValue::List(sets),
    ]);
    check("set_led_sync", Ok(expected), actual);
}

#[test]
fn set_led_no_sync_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1'})\n\
         gcmd = make_gcmd('SET_LED', 'SET_LED RED=0.25 SYNC=0', {'RED': '0.25', 'SYNC': '0'})\n\
         printer.gcode.handlers['SET_LED'](gcmd)\n\
         printer.reactor.run_callbacks()\n",
    )
    .expect("python set_led");
    let expected = py
        .eval("(obj.get_status()['color_data'], [p.sets for p in printer.pins.pins], \
               printer.toolhead.lookahead)")
        .expect("python set_led values");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    let obj = PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED",
        "SET_LED RED=0.25 SYNC=0",
        &[("RED", "0.25"), ("SYNC", "0")],
    );
    w.gcode.invoke("SET_LED", &command).expect("SET_LED");
    assert!(
        wait_for(&w.reactor, || w.pin_sets().iter().any(|s| !s.is_empty())),
        "transmit callback did not run"
    );
    let sets: Vec<RValue> = w.pin_sets().iter().map(|s| rv_sets(s)).collect();
    w.stop();
    let actual = RValue::Tuple(vec![
        rv_colors(&obj.led_helper().color_data()),
        RValue::List(sets),
        RValue::List(Vec::new()),
    ]);
    check("set_led_no_sync", Ok(expected), actual);
}

#[test]
fn set_led_transmit_zero_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1'})\n\
         gcmd = make_gcmd('SET_LED', 'SET_LED RED=1 TRANSMIT=0 SYNC=0', \
                          {'RED': '1', 'TRANSMIT': '0', 'SYNC': '0'})\n\
         printer.gcode.handlers['SET_LED'](gcmd)\n\
         printer.reactor.run_callbacks()\n",
    )
    .expect("python set_led");
    let expected = py
        .eval("(obj.get_status()['color_data'], [p.sets for p in printer.pins.pins])")
        .expect("python set_led values");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    let obj = PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED",
        "SET_LED RED=1 TRANSMIT=0 SYNC=0",
        &[("RED", "1"), ("TRANSMIT", "0"), ("SYNC", "0")],
    );
    w.gcode.invoke("SET_LED", &command).expect("SET_LED");
    kos::time::sleep(0.05);
    let sets: Vec<RValue> = w.pin_sets().iter().map(|s| rv_sets(s)).collect();
    w.stop();
    let actual = RValue::Tuple(vec![
        rv_colors(&obj.led_helper().color_data()),
        RValue::List(sets),
    ]);
    check("set_led_transmit_zero", Ok(expected), actual);
}

#[test]
fn set_led_bad_index_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1'})\n\
         gcmd = make_gcmd('SET_LED', 'SET_LED RED=0.5 INDEX=2', {'RED': '0.5', 'INDEX': '2'})\n\
         def run():\n\
         \x20   printer.gcode.handlers['SET_LED'](gcmd)\n",
    )
    .expect("python setup");
    let expected = py.eval("run()");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED",
        "SET_LED RED=0.5 INDEX=2",
        &[("RED", "0.5"), ("INDEX", "2")],
    );
    match w.gcode.invoke("SET_LED", &command) {
        Ok(()) => panic!("set_led_bad_index: rust accepted INDEX=2"),
        Err(e) => check_err("set_led_bad_index", expected, &e),
    }
    w.stop();
}

#[test]
fn set_led_bad_color_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, obj = make_pwmled({'red_pin': 'gpio1'})\n\
         gcmd = make_gcmd('SET_LED', 'SET_LED RED=2', {'RED': '2'})\n\
         def run():\n\
         \x20   printer.gcode.handlers['SET_LED'](gcmd)\n",
    )
    .expect("python setup");
    let expected = py.eval("run()");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd("SET_LED", "SET_LED RED=2", &[("RED", "2")]);
    match w.gcode.invoke("SET_LED", &command) {
        Ok(()) => panic!("set_led_bad_color: rust accepted RED=2"),
        Err(e) => check_err("set_led_bad_color", expected, &e),
    }
    w.stop();
}

#[test]
fn helper_index_and_full_color_match() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer, config, updates, helper = make_helper({'initial_RED': '0.1'}, 3)\n\
         g1 = make_gcmd('SET_LED', 'SET_LED INDEX=2 RED=0.5', {'INDEX': '2', 'RED': '0.5'})\n\
         helper.cmd_SET_LED(g1)\n\
         g2 = make_gcmd('SET_LED', 'SET_LED BLUE=0.75', {'BLUE': '0.75'})\n\
         helper.cmd_SET_LED(g2)\n\
         g3 = make_gcmd('SET_LED', 'SET_LED BLUE=0.75', {'BLUE': '0.75'})\n\
         helper.cmd_SET_LED(g3)\n",
    )
    .expect("python helper");
    let expected = py
        .eval("(helper.get_status()['color_data'], printer.gcode.mux)")
        .expect("python helper values");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("initial_RED", "0.1")]);
    let updates: Arc<Mutex<usize>> = Arc::new(Mutex::new(0));
    let update_func: extras::led::LedUpdateFunc = {
        let updates = updates.clone();
        Arc::new(move |_state, _print_time| {
            *updates.lock().unwrap() += 1;
            Ok(())
        })
    };
    let helper = LEDHelper::new(&cfg, update_func, 3).expect("LEDHelper");
    let g1 = gcmd(
        "SET_LED",
        "SET_LED INDEX=2 RED=0.5",
        &[("INDEX", "2"), ("RED", "0.5")],
    );
    helper.cmd_set_led(&g1).expect("SET_LED INDEX=2");
    let g2 = gcmd("SET_LED", "SET_LED BLUE=0.75", &[("BLUE", "0.75")]);
    helper.cmd_set_led(&g2).expect("SET_LED BLUE");
    let g3 = gcmd("SET_LED", "SET_LED BLUE=0.75", &[("BLUE", "0.75")]);
    helper.cmd_set_led(&g3).expect("SET_LED BLUE repeat");
    let actual = RValue::Tuple(vec![
        rv_colors(&helper.color_data()),
        RValue::List(w.gcode.mux.lock().unwrap().clone()),
    ]);
    check("helper_index_full", Ok(expected), actual);
    w.stop();
}

// ---------------------------------------------------------------------------
// `_template_update` (the parse/clamp/fill logic led.py owns).
// ---------------------------------------------------------------------------

static LOGS: Mutex<Vec<(String, String)>> = Mutex::new(Vec::new());

fn install_log_sink() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOGS.lock()
                .unwrap()
                .push((rec.level.name().to_string(), rec.message.to_string()));
        }));
    });
}

#[test]
fn template_update_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "import extras.led as _led\n\
         _led.logging = FakeLogging()\n\
         printer, config, updates, helper = make_helper({}, 2)\n\
         helper._template_update(1, '0.5,0.25')\n\
         helper._template_update(2, '1.5,-0.5,0.5,2.0')\n\
         helper._template_update(1, 'nan,inf,0.125')\n\
         helper._template_update(2, '1_0,0,0,0')\n\
         helper._template_update(1, 'a,b')\n\
         helper._template_update(2, '0.1,0.2,0.3,0.4,0.5,0.6')\n\
         helper._template_update(1, '0.5,2,-1,0.25,0.75')\n",
    )
    .expect("python template updates");
    let expected = py
        .eval("(helper.get_status()['color_data'], _led.logging.msgs)")
        .expect("python template values");

    install_log_sink();
    let log_start = LOGS.lock().unwrap().len();
    let w = world();
    let (cfg, _cfg_fake) = w.config(&[]);
    let update_func: extras::led::LedUpdateFunc = Arc::new(|_state, _print_time| Ok(()));
    let helper = LEDHelper::new(&cfg, update_func, 2).expect("LEDHelper");
    helper.template_update(1, "0.5,0.25");
    helper.template_update(2, "1.5,-0.5,0.5,2.0");
    helper.template_update(1, "nan,inf,0.125");
    helper.template_update(2, "1_0,0,0,0");
    helper.template_update(1, "a,b");
    helper.template_update(2, "0.1,0.2,0.3,0.4,0.5,0.6");
    helper.template_update(1, "0.5,2,-1,0.25,0.75");
    let logs: Vec<RValue> = LOGS
        .lock()
        .unwrap()
        .iter()
        .skip(log_start)
        .map(|(_, msg)| RValue::str(msg))
        .collect();
    let actual = RValue::Tuple(vec![rv_colors(&helper.color_data()), RValue::List(logs)]);
    check("template_update", Ok(expected), actual);
    w.stop();
}

// ---------------------------------------------------------------------------
// `SET_LED_TEMPLATE` end to end through the real template evaluator.
// ---------------------------------------------------------------------------

#[test]
fn set_led_template_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = new_printer()\n\
         printer.objs['display_template'] = FakeDisplayTemplates(\
             {'static': FakeDisplayTemplate('0.5,0.25,0.125,1.0')})\n\
         config = FakeConfig(printer, 'led myled', {'red_pin': 'gpio1'})\n\
         obj = led.PrinterPWMLED(config)\n\
         gcmd = make_gcmd('SET_LED_TEMPLATE', 'SET_LED_TEMPLATE TEMPLATE=static', \
                          {'TEMPLATE': 'static'})\n\
         printer.gcode.handlers['SET_LED_TEMPLATE'](gcmd)\n\
         printer.reactor.run_timers(100.0)\n\
         printer.reactor.run_callbacks()\n",
    )
    .expect("python set_led_template");
    let expected = py
        .eval("(obj.get_status()['color_data'], [p.sets for p in printer.pins.pins], \
               printer.gcode.mux)")
        .expect("python set_led_template values");

    let w = world();
    // The template is looked up through the evaluator's snapshot of the
    // `display_template` object taken at construction time.
    {
        let templates: ObjectRef = w.printer.objects.lock().unwrap()["display_template"].clone();
        let fake = klippy_api::downcast::<FakeDisplayTemplates>(&*templates).expect("templates");
        fake.map.lock().unwrap().insert(
            "static".to_string(),
            Arc::new(FakeDisplayTemplate {
                text: "0.5,0.25,0.125,1.0".to_string(),
                renders: Mutex::new(0),
            }),
        );
    }
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    let obj = PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED_TEMPLATE",
        "SET_LED_TEMPLATE TEMPLATE=static",
        &[("TEMPLATE", "static")],
    );
    w.gcode
        .invoke("SET_LED_TEMPLATE", &command)
        .expect("SET_LED_TEMPLATE");
    assert!(
        wait_for(&w.reactor, || w.pin_sets().iter().any(|s| !s.is_empty())),
        "template render/flush chain did not reach the pin"
    );
    let sets: Vec<RValue> = w.pin_sets().iter().map(|s| rv_sets(s)).collect();
    let mux = RValue::List(w.gcode.mux.lock().unwrap().clone());
    // The render timer keeps firing on the reactor thread until it stops.
    w.stop();
    let actual = RValue::Tuple(vec![
        rv_colors(&obj.led_helper().color_data()),
        RValue::List(sets),
        mux,
    ]);
    check("set_led_template", Ok(expected), actual);
}

#[test]
fn set_led_template_bad_index_matches() {
    let Some(mut py) = oracle() else { return };
    py.exec(
        "printer = new_printer()\n\
         config = FakeConfig(printer, 'led myled', {'red_pin': 'gpio1'})\n\
         obj = led.PrinterPWMLED(config)\n\
         gcmd = make_gcmd('SET_LED_TEMPLATE', 'SET_LED_TEMPLATE INDEX=0 TEMPLATE=static', \
                          {'INDEX': '0', 'TEMPLATE': 'static'})\n\
         def run():\n\
         \x20   printer.gcode.handlers['SET_LED_TEMPLATE'](gcmd)\n",
    )
    .expect("python setup");
    let expected = py.eval("run()");

    let w = world();
    let (cfg, _cfg_fake) = w.config(&[("red_pin", "gpio1")]);
    PrinterPWMLED::new(&cfg).expect("construction");
    let command = gcmd(
        "SET_LED_TEMPLATE",
        "SET_LED_TEMPLATE INDEX=0 TEMPLATE=static",
        &[("INDEX", "0"), ("TEMPLATE", "static")],
    );
    match w.gcode.invoke("SET_LED_TEMPLATE", &command) {
        Ok(()) => panic!("set_led_template_bad_index: rust accepted INDEX=0"),
        Err(e) => check_err("set_led_template_bad_index", expected, &e),
    }
    w.stop();
}
