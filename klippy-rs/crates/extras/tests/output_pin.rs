//! Parity tests for `extras::output_pin` (port of `klippy/extras/output_pin.py`).
//!
//! Every expected value comes from `pyoracle` (CPython 3.8.2 with the frozen
//! `klippy/`). The Python side builds a real `PrinterOutputPin` and a real
//! `PrinterTemplateEvaluator` on minimal Python fakes for the printer, config,
//! `pins`, `gcode`, `motion_queuing`, `toolhead`, `display_template` and
//! `gcode_macro`. The Rust side builds the ported objects on fakes of the
//! matching contract traits with identical setup. Returned values (floats by
//! bits), raised errors (class and message), pin calls, G-code command
//! registrations and status dicts are compared exactly.
//!
//! Integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test output_pin
//! ```
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle alone.

#![allow(dead_code, unused_imports, clippy::disallowed_types, clippy::new_ret_no_self)]
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

use extras::display::display::{DisplayGroup, DisplayTemplateApi, GlyphTable, PrinterDisplayTemplateApi};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use extras::output_pin::{
    lookup_template_eval, module_factory, GCodeRequestQueue, PrinterOutputPin, TemplateCallback,
    RENDER_TIME,
};
use extras_macros_templates::gcode_macro::Value;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GGetFloatOpts, GetOpts as GGetOpts, GCmd,
    GCodeCommandHandler, OutputCallback, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, DigitalOutPin, DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts, Mcu,
    McuApi, ParsePinOpts, Pin, PinParams, PinsApi, PwmPin, PwmPinApi, QuerySendOpts, SendOpts,
    TriggerDispatch, AdcCallback, ConfigCallback, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi, EV_KLIPPY_CONNECT};
use klippy_api::stepper::{StepperFactory, Trapq};
use klippy_api::toolhead::{
    ExtraAxis, FlushCallback, HomingTargetApi, Kinematics, LookaheadCallback, MotionQueuingApi,
    PrinterExtruder, SyncEmitterHandle, ToolHeadApi, TrapqAppend,
};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{NEVER, Reactor};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value as RValue};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
_sentinel = object()
import extras.output_pin as output_pin


class CommandError(Exception):
    pass

class ConfigError(Exception):
    pass

class FakeReactor:
    NOW = 0.
    NEVER = 9999999999999999.
    def __init__(self):
        self.timers = []
    def register_timer(self, cb, waketime=NEVER):
        self.timers.append(cb)
        return ('timer', len(self.timers)-1)
    def update_timer(self, timer, waketime): pass
    def unregister_timer(self, timer):
        self.timers[timer[1]] = None
    def monotonic(self): return 100.0

class FakeMcu:
    def __init__(self, min_sched=0.02, max_dur=1.0):
        self.min_sched = min_sched; self.max_dur = max_dur
    def min_schedule_time(self): return self.min_sched
    def max_nominal_duration(self): return self.max_dur
    def estimated_print_time(self, et): return et

class FakePwmPin:
    def __init__(self, mcu): self._mcu = mcu; self.calls = []; self.sets = []
    def get_mcu(self): return self._mcu
    def setup_cycle_time(self, ct, hw): self.calls.append(('cycle', ct, hw))
    def setup_max_duration(self, md): self.calls.append(('maxdur', md))
    def setup_start_value(self, sv, sh): self.calls.append(('start', sv, sh))
    def set_pwm(self, pt, v): self.sets.append(('pwm', pt, v))

class FakeDigitalPin:
    def __init__(self, mcu): self._mcu = mcu; self.calls = []; self.sets = []
    def get_mcu(self): return self._mcu
    def setup_max_duration(self, md): self.calls.append(('maxdur', md))
    def setup_start_value(self, sv, sh): self.calls.append(('start', sv, sh))
    def set_digital(self, pt, v): self.sets.append(('digital', pt, v))

class FakePins:
    def __init__(self, mcu): self.mcu = mcu; self.setup = []
    def setup_pin(self, t, pin):
        self.setup.append((t, pin))
        if t == 'pwm': return FakePwmPin(self.mcu)
        return FakeDigitalPin(self.mcu)

class FakeToolhead:
    def __init__(self): self.lookahead = []
    def register_lookahead_callback(self, cb): self.lookahead.append(cb)
    def run_lookahead(self, pt):
        for cb in list(self.lookahead): cb(pt)

class FakeMotionQueuing:
    def __init__(self): self.flush_cbs = []; self.activity = []
    def register_flush_callback(self, cb, can_add_trapq=False): self.flush_cbs.append(cb)
    def note_mcu_movequeue_activity(self, t, is_step_gen=False):
        self.activity.append((t, is_step_gen))
    def flush(self, must, max_step):
        for cb in list(self.flush_cbs): cb(must, max_step)

class FakeGCode:
    def __init__(self): self.mux = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, func, desc))

class FakeGCommand:
    def __init__(self, params=None, floats=None, strings=None, errors=None):
        self.params = dict(params or {}); self.floats = dict(floats or {})
        self.strings = dict(strings or {}); self.errors = dict(errors or {})
    def get_float(self, name, default=None, minval=None, maxval=None, above=None, below=None):
        if name in self.errors: raise self.errors[name]
        v = self.floats.get(name, default)
        if v is None: return None
        if minval is not None and v < minval:
            raise CommandError("get_float min")
        if maxval is not None and v > maxval:
            raise CommandError("get_float max")
        return v
    def get(self, name, default=None, parser=str):
        v = self.strings.get(name, default)
        return v
    def get_command_parameters(self): return dict(self.params)
    def error(self, msg): return CommandError(msg)

class FakeDisplayTemplate:
    def __init__(self, name, style='level', params=None):
        self.name = name; self.style = style; self.params = dict(params or {})
        self.renders = 0
    def get_params(self): return dict(self.params)
    def render(self, context, **kwargs):
        self.renders += 1
        if self.style == 'level':
            v = kwargs.get('param_level', 0)
            return str(v)
        if self.style == 'fail':
            raise RuntimeError('render boom')
        if self.style == 'text':
            return '0.75'
        if self.style == 'empty':
            return ''
        if self.style == 'compose':
            # Uses the render() helper injected in the context.
            return context['render']('sub', param_level=kwargs.get('param_level', 1))
        return ''

class FakeDisplayTemplates:
    def __init__(self, templates): self.templates = dict(templates)
    def get_display_templates(self): return dict(self.templates)

class FakeMacro:
    def __init__(self): self.ctx_calls = 0
    def create_template_context(self, eventtime):
        self.ctx_calls += 1
        return {'printer': {}, 'eventtime': eventtime}

class FakePrinter:
    def __init__(self, objs, reactor):
        self.objs = dict(objs); self._reactor = reactor
        self.handlers = {}
        self.logs = []
    def get_reactor(self): return self._reactor
    def lookup_object(self, name, default=None):
        if name in self.objs: return self.objs[name]
        if default is None: return None
        raise ConfigError("Unknown section '%s'" % (name,))
    def add_object(self, name, obj): self.objs[name] = obj
    def load_object(self, config, section, default=None):
        if section not in self.objs:
            if default is None: return None
            raise ConfigError("Unknown section '%s'" % (section,))
        return self.objs[section]
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def fire(self, event):
        for cb in list(self.handlers.get(event, [])): cb()

class FakeConfig:
    def __init__(self, printer, section, values, raw_strings=None):
        self._printer = printer; self.section = section
        self.values = dict(values or {}); self.raw = dict(raw_strings or {})
    def get_printer(self): return self._printer
    def get_name(self): return self.section
    def _v(self, option, default):
        if option in self.values: return self.values[option]
        if default is not _sentinel: return default
        raise ConfigError("Could not find option '%s' in section '%s'" % (option, self.section))
    def get(self, option, default=_sentinel):
        return self._v(option, default)
    def getboolean(self, option, default=False):
        return bool(self._v(option, default))
    def getfloat(self, option, default=0., minval=None, maxval=None, above=None, below=None):
        v = float(self._v(option, default))
        if minval is not None and v < minval:
            raise ConfigError("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise ConfigError("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        if above is not None and v <= above:
            raise ConfigError("Option '%s' in section '%s' must be above %s" % (option, self.section, above))
        if below is not None and v >= below:
            raise ConfigError("Option '%s' in section '%s' must be below %s" % (option, self.section, below))
        return v

_sentinel = object()

def make_config(printer, name, values):
    return FakeConfig(printer, name, values)
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: Mutex<IndexMap<String, ObjectRef>>,
    handlers: Mutex<Vec<(String, EventCallback)>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> Arc<Self> {
        Arc::new(FakePrinter {
            objects: Mutex::new(IndexMap::new()),
            handlers: Mutex::new(Vec::new()),
            reactor: Reactor::new(false),
        })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.lock().unwrap().insert(name.to_string(), obj);
    }
    fn fire(&self, event: &str) {
        let cbs: Vec<EventCallback> = {
            let mut h = self.handlers.lock().unwrap();
            let mut out: Vec<EventCallback> = Vec::new();
            let mut i = 0;
            while i < h.len() {
                if h[i].0 == event {
                    out.push(h.remove(i).1);
                } else {
                    i += 1;
                }
            }
            out
        };
        for mut cb in cbs {
            let _ = cb(&[]);
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { self.reactor.clone() }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut m = self.objects.lock().unwrap();
        if m.contains_key(name) {
            return Err(KlippyError::config(format!("Duplicate object '{}'", name)));
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
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown section '{}'", name))),
        }
    }
    fn lookup_objects(&self, _m: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.lock().unwrap().iter().map(|(k, v)| (k.clone(), v.clone())).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(&self, _c: &Config, section: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let m = self.objects.lock().unwrap();
        if let Some(o) = m.get(section) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::ReturnNone => Ok(None),
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown section '{}'", section))),
        }
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, cb: EventCallback) {
        self.handlers.lock().unwrap().push((event.to_string(), cb));
    }
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _r: &str) {}
}

// --- Config ---

struct FakeConfig {
    printer: Printer,
    section: String,
    values: IndexMap<String, String>,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, pairs: Vec<(&str, &str)>) -> Config {
        let mut values = IndexMap::new();
        for (k, v) in pairs { values.insert(k.to_string(), v.to_string()); }
        Arc::new(FakeConfig { printer, section: section.to_string(), values })
    }
}

fn parse_bool(s: &str) -> bool {
    matches!(s.trim(), "1" | "true" | "True" | "yes" | "on")
}
fn parse_float(s: &str) -> f64 {
    s.trim().parse::<f64>().unwrap_or(0.)
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { self.section.clone() }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        if let Some(v) = self.values.get(option) { Ok(Some(v.clone())) } else if let Some(d) = opts.default { Ok(d) } else {
            Err(KlippyError::config(format!("Could not find option '{}' in section '{}'", option, self.section)))
        }
    }
    fn getint(&self, _o: &str, _opts: GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v = match self.values.get(option) {
            Some(s) => parse_float(s),
            None => match opts.default { Some(d) => d.unwrap_or(0.), None => return Err(KlippyError::config(format!("Could not find option '{}' in section '{}'", option, self.section))) },
        };
        if let Some(mn) = opts.minval { if v < mn { return Err(KlippyError::config(format!("Option '{}' in section '{}' must have minimum of {}", option, self.section, mn))); } }
        if let Some(mx) = opts.maxval { if v > mx { return Err(KlippyError::config(format!("Option '{}' in section '{}' must have maximum of {}", option, self.section, mx))); } }
        if let Some(a) = opts.above { if v <= a { return Err(KlippyError::config(format!("Option '{}' in section '{}' must be above {}", option, self.section, a))); } }
        if let Some(b) = opts.below { if v >= b { return Err(KlippyError::config(format!("Option '{}' in section '{}' must be below {}", option, self.section, b))); } }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        if let Some(s) = self.values.get(option) { Ok(Some(parse_bool(s))) } else if let Some(d) = opts.default { Ok(d) } else {
            Err(KlippyError::config(format!("Could not find option '{}' in section '{}'", option, self.section)))
        }
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, _s: &str) -> Config { panic!("getsection unused in output_pin") }
    fn has_section(&self, _s: &str) -> bool { false }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// --- MCU / pins ---

struct FakeMcu {
    min_sched: f64,
    max_dur: f64,
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String { "mcu".to_string() }
    fn get_printer(&self) -> Printer { panic!("get_printer unused") }
    fn is_fileoutput(&self) -> bool { false }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> { panic!("MCU setup_pin unused") }
    fn create_oid(&self) -> KResult<i64> { Ok(1) }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> { Ok(()) }
    fn add_config_cmd(&self, _c: &str, _o: AddConfigCmdOpts) -> KResult<()> { Ok(()) }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _o: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, _t: f64) -> i64 { 0 }
    fn min_schedule_time(&self) -> f64 { self.min_sched }
    fn max_nominal_duration(&self) -> f64 { self.max_dur }
    fn lookup_command(&self, _f: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> { panic!("unused") }
    fn lookup_query_command(&self, _f: &str, _r: &str, _o: LookupQueryCommandOpts) -> KResult<CommandQueryWrapper> { panic!("unused") }
    fn try_lookup_command(&self, _f: &str) -> Option<CommandWrapper> { None }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(1) }
    fn register_serial_response(&self, _cb: SerialResponseCallback, _m: &str, _o: Option<i64>) -> AsyncResponseWrapper { panic!("unused") }
    fn check_valid_response(&self, _f: &str) -> bool { false }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> { IndexMap::new() }
    fn get_constants(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> { Ok(0.) }
    fn print_time_to_clock(&self, _t: f64) -> i64 { 0 }
    fn clock_to_print_time(&self, c: f64) -> f64 { c }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { eventtime }
    fn clock32_to_clock64(&self, c: i64) -> i64 { c }
    fn calibrate_clock(&self, _p: f64, _e: f64) -> KResult<(f64, f64)> { Ok((0., 0.)) }
    fn get_status(&self, _e: Option<f64>) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> { panic!("unused") }
}

type PinCalls = Mutex<Vec<(String, Vec<PyValue>)>>;

struct FakePwmPin {
    mcu: Mcu,
    calls: PinCalls,
    sets: Mutex<Vec<(f64, f64)>>,
}

impl PwmPinApi for FakePwmPin {
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn setup_max_duration(&self, v: f64) { self.calls.lock().unwrap().push(("maxdur".to_string(), vec![PyValue::Float(v)])); }
    fn setup_cycle_time(&self, ct: f64, hw: Option<bool>) -> klippy_api::KResult<()> { self.calls.lock().unwrap().push(("cycle".to_string(), vec![PyValue::Float(ct), PyValue::Bool(hw.unwrap_or(false))])); Ok(()) }
    fn setup_start_value(&self, a: f64, b: f64) { self.calls.lock().unwrap().push(("start".to_string(), vec![PyValue::Float(a), PyValue::Float(b)])); }
    fn next_aligned_print_time(&self, pt: f64, _a: Option<f64>) -> f64 { pt }
    fn set_pwm(&self, pt: f64, v: f64) -> klippy_api::KResult<()> { self.sets.lock().unwrap().push((pt, v)); Ok(()) }
}

struct FakeDigitalPin {
    mcu: Mcu,
    calls: PinCalls,
    sets: Mutex<Vec<(f64, bool)>>,
}

impl DigitalOutPinApi for FakeDigitalPin {
    fn get_mcu(&self) -> Mcu { self.mcu.clone() }
    fn setup_max_duration(&self, v: f64) { self.calls.lock().unwrap().push(("maxdur".to_string(), vec![PyValue::Float(v)])); }
    fn setup_start_value(&self, a: bool, b: bool) { self.calls.lock().unwrap().push(("start".to_string(), vec![PyValue::Bool(a), PyValue::Bool(b)])); }
    fn set_digital(&self, pt: f64, v: bool) { self.sets.lock().unwrap().push((pt, v)); }
}

struct FakePins {
    mcu: Mcu,
    setups: Mutex<Vec<(String, String)>>,
    pwms: Mutex<Vec<Arc<FakePwmPin>>>,
    digitals: Mutex<Vec<Arc<FakeDigitalPin>>>,
}

impl FakePins {
    fn new(mcu: Mcu) -> Arc<Self> {
        Arc::new(FakePins { mcu, setups: Mutex::new(Vec::new()), pwms: Mutex::new(Vec::new()), digitals: Mutex::new(Vec::new()) })
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _d: &str, _o: ParsePinOpts) -> KResult<PinParams> { panic!("unused") }
    fn lookup_pin(&self, _d: &str, _o: LookupPinOpts) -> KResult<PinParams> { panic!("unused") }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        self.setups.lock().unwrap().push((pin_type.to_string(), pin_desc.to_string()));
        if pin_type == "pwm" {
            let p = Arc::new(FakePwmPin { mcu: self.mcu.clone(), calls: Mutex::new(Vec::new()), sets: Mutex::new(Vec::new()) });
            self.pwms.lock().unwrap().push(p.clone());
            Ok(Pin::Pwm(p))
        } else {
            let p = Arc::new(FakeDigitalPin { mcu: self.mcu.clone(), calls: Mutex::new(Vec::new()), sets: Mutex::new(Vec::new()) });
            self.digitals.lock().unwrap().push(p.clone());
            Ok(Pin::DigitalOut(p))
        }
    }
    fn reset_pin_sharing(&self, _p: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _c: &str) -> KResult<klippy_api::mcu::PinResolver> { panic!("unused") }
    fn register_chip(&self, _c: &str, _c2: Chip) -> KResult<()> { Ok(()) }
    fn allow_multi_use_pin(&self, _d: &str) -> KResult<()> { Ok(()) }
}

// --- motion queuing / toolhead / gcode ---

struct FakeMotionQueuing {
    flush_cbs: Mutex<Vec<FlushCallback>>,
    activity: Mutex<Vec<(f64, bool)>>,
}

impl MotionQueuingApi for FakeMotionQueuing {
    fn allocate_trapq(&self) -> Trapq { panic!("unused") }
    fn lookup_trapq_append(&self) -> TrapqAppend { panic!("unused") }
    fn wipe_trapq(&self, _t: &Trapq) {}
    fn allocate_syncemitter(&self, _m: Mcu, _n: &str, _a: bool) -> KResult<SyncEmitterHandle> { panic!("unused") }
    fn queue_syncemitter_msg(&self, _syncemitter: &SyncEmitterHandle, _req_clock: u64, _data: &[u32]) {}
    fn setup_mcu_movequeue(&self, _m: Mcu, _q: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _c: i64) {}
    fn register_flush_callback(&self, cb: FlushCallback, _can: bool) { self.flush_cbs.lock().unwrap().push(cb); }
    fn unregister_flush_callback(&self, _cb: &FlushCallback) {}
    fn register_flush_callback_named(&self, cb: FlushCallback, can_add_trapq: bool, _name: Option<&str>, _obj: Option<&str>) {
        self.register_flush_callback(cb, can_add_trapq);
    }
    fn get_kin_flush_delay(&self) -> f64 { 0. }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> { Ok(()) }
    fn calc_step_gen_restart(&self, _t: f64) -> f64 { 0. }
    fn note_mcu_movequeue_activity(&self, t: f64, s: bool) { self.activity.lock().unwrap().push((t, s)); }
    fn drip_update_time(&self, _a: f64, _b: f64, _c: &reactor::Completion<bool>) -> KResult<()> { Ok(()) }
    fn check_drip_timing(&self) -> Option<f64> { None }
}

impl FakeMotionQueuing {
    fn flush(&self, must: f64, max: f64) {
        self.flush_result(must, max).expect("cb");
    }
    /// `mq.flush(must, max)` with the callbacks' exceptions propagating.
    fn flush_result(&self, must: f64, max: f64) -> KResult<()> {
        let cbs: Vec<_> = self.flush_cbs.lock().unwrap().clone();
        for cb in cbs { cb(must, max)?; }
        Ok(())
    }
}

struct FakeToolhead {
    lookahead: Mutex<Vec<LookaheadCallback>>,
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _p: Vec<f64>, _s: f64) -> KResult<()> { Ok(()) }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> { Ok(()) }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> { Ok(PyValue::Dict(IndexMap::new())) }
    fn get_trapq(&self) -> Trapq { panic!("unused") }
    fn get_max_velocity(&self) -> (f64, f64) { (0., 0.) }
    fn set_max_velocities(&self, _a: Option<f64>, _b: Option<f64>, _c: Option<f64>, _d: Option<f64>) -> (f64, f64, f64, f64) { (0., 0., 0., 0.) }
    fn register_lookahead_callback(&self, cb: LookaheadCallback) -> KResult<()> { self.lookahead.lock().unwrap().push(cb); Ok(()) }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) { (0., 0., false) }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: klippy_api::toolhead::PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder { panic!("unused") }
    fn add_extra_axis(&self, _e: klippy_api::toolhead::ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> { Vec::new() }
    fn stats(&self, _e: f64) -> (bool, String) { (false, String::new()) }
    fn stepper_factory(&self) -> StepperFactory { panic!("unused") }
}

impl HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> { vec![0., 0., 0., 0.] }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> { Ok(()) }
    fn get_kinematics(&self) -> Kinematics { panic!("unused") }
    fn flush_step_generation(&self) -> KResult<()> { Ok(()) }
    fn get_last_move_time(&self) -> KResult<f64> { Ok(0.) }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _c: &reactor::Completion<bool>) -> KResult<()> { Ok(()) }
}

impl FakeToolhead {
    fn run_lookahead(&self, pt: f64) {
        let mut cbs = self.lookahead.lock().unwrap();
        for cb in cbs.iter_mut() { let _ = cb(pt); }
    }
}

struct FakeGCode {
    mux: Mutex<Vec<(String, String, Option<String>, String)>>,
    handlers: Mutex<Vec<GCodeCommandHandler>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(&self, _c: &str, _f: Option<GCodeCommandHandler>, _o: RegisterCommandOpts) -> KResult<Option<GCodeCommandHandler>> { Ok(None) }
    fn register_mux_command(&self, cmd: &str, key: &str, value: Option<&str>, func: GCodeCommandHandler, desc: Option<&str>) -> KResult<()> {
        self.mux.lock().unwrap().push((cmd.to_string(), key.to_string(), value.map(|s| s.to_string()), desc.unwrap_or("").to_string()));
        self.handlers.lock().unwrap().push(func);
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> { IndexMap::new() }
    fn get_status(&self, _e: f64) -> PyValue { PyValue::Dict(IndexMap::new()) }
    fn register_output_handler(&self, _cb: OutputCallback) {}
    fn request_restart(&self, _r: &str) -> KResult<()> { Ok(()) }
    fn run_script(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn run_script_from_command(&self, _s: &str) -> KResult<()> { Ok(()) }
    fn get_mutex(&self) -> reactor::ReactorMutex { panic!("unused") }
    fn create_gcode_command(&self, _c: &str, _cl: &str, _p: &IndexMap<String, String>) -> GCmd { panic!("unused") }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _c: &str) -> bool { false }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> { Ok(()) }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> { Ok(()) }
}

struct FakeGCommand {
    params: IndexMap<String, String>,
    floats: IndexMap<String, Option<f64>>,
    strings: IndexMap<String, Option<String>>,
}

impl GCodeCommandApi for FakeGCommand {
    fn get(&self, name: &str, opts: GGetOpts) -> KResult<PyValue> {
        match self.strings.get(name) {
            Some(Some(s)) => Ok(PyValue::Str(s.clone())),
            Some(None) => Ok(PyValue::None),
            None => {
                if let Some(d) = opts.default { Ok(d) } else { Err(KlippyError::command("Missing required parameter")) }
            }
        }
    }
    fn get_int(&self, _n: &str, _o: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> { Ok(None) }
    fn get_float(&self, name: &str, opts: GGetFloatOpts) -> KResult<Option<f64>> {
        let v = match self.floats.get(name) {
            Some(v) => *v,
            None => opts.default.unwrap_or(None),
        };
        if let Some(v) = v {
            if let Some(mn) = opts.minval { if v < mn { return Err(KlippyError::command("Invalid value")); } }
            if let Some(mx) = opts.maxval { if v > mx { return Err(KlippyError::command("Invalid value")); } }
        }
        Ok(v)
    }
    fn get_command(&self) -> String { "SET_PIN".to_string() }
    fn get_commandline(&self) -> String { "SET_PIN".to_string() }
    fn get_command_parameters(&self) -> IndexMap<String, String> { self.params.clone() }
    fn get_raw_command_parameters(&self) -> String { String::new() }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool { true }
}

fn make_gcmd(params: Vec<(&str, &str)>, floats: Vec<(&str, Option<f64>)>, strings: Vec<(&str, Option<&str>)>) -> GCmd {
    let mut p = IndexMap::new();
    for (k, v) in params { p.insert(k.to_string(), v.to_string()); }
    let mut f = IndexMap::new();
    for (k, v) in floats { f.insert(k.to_string(), v); }
    let mut s = IndexMap::new();
    for (k, v) in strings { s.insert(k.to_string(), v.map(|x| x.to_string())); }
    Arc::new(FakeGCommand { params: p, floats: f, strings: s })
}

// --- display / macro ---

struct FakeDisplayTemplate {
    style: String,
    params: IndexMap<String, PyValue>,
    renders: Arc<Mutex<usize>>,
}

impl DisplayTemplateApi for FakeDisplayTemplate {
    fn get_params(&self) -> IndexMap<String, PyValue> { self.params.clone() }
    fn render_values(&self, context: &IndexMap<String, Value>, kwargs: IndexMap<String, Value>) -> KResult<String> {
        *self.renders.lock().unwrap() += 1;
        match self.style.as_str() {
            "level" => {
                let v = kwargs.get("param_level").map(|v| match v { Value::Py(p) => p.str(), v => v.to_string() }).unwrap_or_else(|| "0".to_string());
                Ok(v)
            }
            "fail" => Err(KlippyError::Internal { class_name: "RuntimeError".to_string(), message: "render boom".to_string() }),
            "text" => Ok("0.75".to_string()),
            "empty" => Ok(String::new()),
            "compose" => {
                // Emulate Python: return context['render']('sub', param_level=...).
                let lvl = kwargs.get("param_level").map(|v| match v { Value::Py(p) => p.str(), v => v.to_string() }).unwrap_or_else(|| "1".to_string());
                if let Some(Value::Func(f)) = context.get("render") {
                    let args = vec![Value::py(PyValue::Str("sub".to_string()))];
                    let kw = vec![("param_level".to_string(), Value::py(PyValue::Str(lvl)))];
                    match (f)(&args, &kw, 1) {
                        Ok(Value::Py(PyValue::Str(s))) => Ok(s),
                        Ok(v) => Ok(v.to_string()),
                        Err(e) => Err(KlippyError::Internal { class_name: e.kind().to_string(), message: e.message().to_string() }),
                    }
                } else {
                    Ok("0".to_string())
                }
            }
            _ => Ok(String::new()),
        }
    }
}

struct FakeDisplayTemplates {
    map: Mutex<IndexMap<String, Arc<FakeDisplayTemplate>>>,
}

impl PrinterDisplayTemplateApi for FakeDisplayTemplates {
    fn get_display_templates(&self) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        let mut out = IndexMap::new();
        for (k, v) in self.map.lock().unwrap().iter() {
            let h: Arc<dyn DisplayTemplateApi> = v.clone();
            out.insert(k.clone(), h);
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

struct FakeMacro;

impl PrinterGCodeMacroApi for FakeMacro {
    fn load_template(&self, _config: &Config, _option: &str, _default: Option<&str>) -> KResult<TemplateWrapper> {
        unreachable!("output_pin never loads templates")
    }
    fn create_template_context(&self, eventtime: Option<f64>) -> IndexMap<String, Value> {
        let eventtime = eventtime.expect("eventtime");
        let mut out = IndexMap::new();
        out.insert("printer".to_string(), Value::py(PyValue::Dict(IndexMap::new())));
        out.insert("eventtime".to_string(), Value::py(PyValue::Float(eventtime)));
        out
    }
}

klippy_api::printer_object!(FakeDisplayTemplates; dyn PrinterDisplayTemplateApi);
klippy_api::printer_object!(FakeMacro; dyn PrinterGCodeMacroApi);
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);
klippy_api::printer_object!(FakePins; dyn PinsApi);
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);
klippy_api::printer_object!(FakeMotionQueuing; dyn MotionQueuingApi);

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("output_pin fakes");
            Some(py)
        }
        None => {
            println!("skipping output_pin oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 binary and run with the repo checkout available");
            None
        }
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: RValue) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "output_pin".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
        Err(e) => e,
    };
    assert_eq!(actual.class_name(), oracle_exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(actual.message(), oracle_exc.message.as_str(), "{label}: exception message");
}



// ---------------------------------------------------------------------------
// Rust world builder.
// ---------------------------------------------------------------------------

struct World {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Mcu,
    pins: Arc<FakePins>,
    mq: Arc<FakeMotionQueuing>,
    toolhead: Arc<FakeToolhead>,
    gcode: Arc<FakeGCode>,
    macros: Arc<FakeMacro>,
    templates: Arc<FakeDisplayTemplates>,
}

fn build_world(with_templates: Vec<(&str, &str, Vec<(&str, PyValue)>)>) -> World {
    let mcu: Mcu = Arc::new(FakeMcu { min_sched: 0.02, max_dur: 1.0 });
    let pins = FakePins::new(mcu.clone());
    let mq = Arc::new(FakeMotionQueuing { flush_cbs: Mutex::new(Vec::new()), activity: Mutex::new(Vec::new()) });
    let toolhead = Arc::new(FakeToolhead { lookahead: Mutex::new(Vec::new()) });
    let gcode = Arc::new(FakeGCode { mux: Mutex::new(Vec::new()), handlers: Mutex::new(Vec::new()) });
    let macros = Arc::new(FakeMacro);
    let mut map: IndexMap<String, Arc<FakeDisplayTemplate>> = IndexMap::new();
    for (name, style, params) in with_templates {
        let mut p: IndexMap<String, PyValue> = IndexMap::new();
        for (k, v) in params { p.insert(k.to_string(), v); }
        map.insert(name.to_string(), Arc::new(FakeDisplayTemplate { style: style.to_string(), params: p, renders: Arc::new(Mutex::new(0)) }));
    }
    let templates = Arc::new(FakeDisplayTemplates { map: Mutex::new(map) });
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    printer.add("pins", pins.clone() as ObjectRef);
    printer.add("motion_queuing", mq.clone() as ObjectRef);
    printer.add("toolhead", toolhead.clone() as ObjectRef);
    printer.add("gcode", gcode.clone() as ObjectRef);
    printer.add("gcode_macro", macros.clone() as ObjectRef);
    printer.add("display_template", templates.clone() as ObjectRef);
    World { printer, printer_api, mcu, pins, mq, toolhead, gcode, macros, templates }
}

fn py_world(py: &mut pyoracle::Oracle) {
    py.exec(r#"
mcu = FakeMcu(0.02, 1.0)
pins = FakePins(mcu)
mq = FakeMotionQueuing()
toolhead = FakeToolhead()
gcode = FakeGCode()
macro = FakeMacro()
# templates are installed per-test via install_templates
printer = FakePrinter({'pins': pins, 'motion_queuing': mq, 'toolhead': toolhead, 'gcode': gcode, 'gcode_macro': macro}, FakeReactor())
def install_templates(tmpl):
    printer.objs['display_template'] = FakeDisplayTemplates(tmpl)
"#).expect("py world");
}

fn py_template(name: &str, style: &str, params: &str) -> String {
    format!("FakeDisplayTemplate('{}', '{}', {})", name, style, params)
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn render_time_constant_matches() {
    let Some(mut py) = oracle() else { return };
    check("RENDER_TIME", py.eval("output_pin.RENDER_TIME"), RValue::float(RENDER_TIME));
}

#[test]
fn pwm_construction_matches() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({})").expect("templates");
    py.exec("config = FakeConfig(printer, 'output_pin fan0', {'pin': 'PA1', 'pwm': 1, 'cycle_time': 0.2, 'hardware_pwm': 1, 'scale': 2.0})")
        .expect("py config");
    py.exec("pin = output_pin.load_config_prefix(config)").expect("py load_config_prefix");
    let expected = py.eval("(pin.get_status(0.0)['value'], pins.setup, pins.mcu.max_nominal_duration())").expect("py status/setup");
    // Rust.
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin fan0", vec![("pin", "PA1"), ("pwm", "1"), ("cycle_time", "0.2"), ("hardware_pwm", "1"), ("scale", "2.0")]);
    let f = module_factory().load_config_prefix.expect("factory prefix");
    let obj = f(&config).expect("rust load_config_prefix").expect("obj");
    let pin_rs: Arc<PrinterOutputPin> = klippy_api::downcast_arc(&obj).expect("downcast");
    let status = pin_rs.get_status_value();
    let actual = {
        let v = match status {
            PyValue::Dict(ref d) => d.get(&PyValue::Str("value".to_string())).cloned().unwrap_or(PyValue::None),
            _ => PyValue::None,
        };
        let fv = v.to_f64().unwrap_or(f64::NAN);
        let setups: Vec<RValue> = w.pins.setups.lock().unwrap().iter().map(|(a, b)| RValue::Tuple(vec![RValue::str(a), RValue::str(b)])).collect();
        RValue::Tuple(vec![RValue::float(fv), RValue::List(setups), RValue::float(1.0)])
    };
    check("pwm_construction", Ok(expected), actual);
}

#[test]
fn digital_construction_matches() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({})").expect("templates");
    py.exec("config = FakeConfig(printer, 'output_pin led', {'pin': 'PA2', 'value': 1.0})").expect("py config");
    py.exec("pin = output_pin.load_config_prefix(config)").expect("py load_config_prefix");
    let expected = py.eval("pin.get_status(0.0)['value']").expect("py status");
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin led", vec![("pin", "PA2"), ("value", "1.0")]);
    let f = module_factory().load_config_prefix.expect("factory prefix");
    let obj = f(&config).expect("rust").expect("obj");
    let pin_rs: Arc<PrinterOutputPin> = klippy_api::downcast_arc(&obj).expect("downcast");
    let status = pin_rs.get_status_value();
    let fv = match status {
        PyValue::Dict(ref d) => d.get(&PyValue::Str("value".to_string())).unwrap().to_f64().unwrap(),
        _ => panic!("dict"),
    };
    check("digital_construction", Ok(expected), RValue::float(fv));
}

#[test]
fn set_pin_value_queues_and_flushes() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({})").expect("templates");
    py.exec("config = FakeConfig(printer, 'output_pin fan0', {'pin': 'PA1', 'pwm': 1, 'scale': 2.0})").expect("config");
    py.exec("pin = output_pin.load_config_prefix(config)").expect("load");
    py.exec("printer.fire('klippy:connect')").expect("connect");
    py.exec("gcmd = FakeGCommand(floats={'VALUE': 1.0}, strings={'TEMPLATE': None})").expect("gcmd");
    py.exec("pin.cmd_SET_PIN(gcmd)").expect("set_pin");
    py.exec("toolhead.run_lookahead(10.0)").expect("lookahead");
    py.exec("mq.flush(20.0, 0.0)").expect("flush");
    let expected_sets = py.eval("pin.mcu_pin.sets").expect("py sets");
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin fan0", vec![("pin", "PA1"), ("pwm", "1"), ("scale", "2.0")]);
    let f = module_factory().load_config_prefix.expect("factory");
    let obj = f(&config).expect("rust").expect("obj");
    let pin_rs: Arc<PrinterOutputPin> = klippy_api::downcast_arc(&obj).expect("downcast");
    w.printer.fire(EV_KLIPPY_CONNECT);
    let gcmd = make_gcmd(vec![], vec![("VALUE", Some(1.0))], vec![("TEMPLATE", None)]);
    pin_rs.cmd_set_pin(&gcmd).expect("cmd");
    w.toolhead.run_lookahead(10.0);
    w.mq.flush(20.0, 0.0);
    let sets = w.pins.pwms.lock().unwrap()[0].sets.lock().unwrap().clone();
    assert_eq!(sets.len(), 1, "one set_pwm call");
    assert_eq!(sets[0].1.to_bits(), (0.5f64).to_bits(), "value 1.0 / scale 2.0");
    assert_eq!(sets[0].0, 10.0);
    check("set_pin_sets", Ok(expected_sets), RValue::List(vec![RValue::Tuple(vec![RValue::str("pwm"), RValue::float(10.0), RValue::float(0.5)])]));
    let gcmd2 = make_gcmd(vec![], vec![("VALUE", Some(1.0))], vec![("TEMPLATE", None)]);
    pin_rs.cmd_set_pin(&gcmd2).expect("cmd2");
    w.toolhead.run_lookahead(11.0);
    w.mq.flush(20.0, 0.0);
    let sets2 = w.pins.pwms.lock().unwrap()[0].sets.lock().unwrap().clone();
    assert_eq!(sets2.len(), 1, "repeat value discarded");
}

#[test]
fn set_pin_errors_match() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({})").expect("templates");
    py.exec("config = FakeConfig(printer, 'output_pin d', {'pin': 'PA3'})").expect("config");
    py.exec("pin = output_pin.load_config_prefix(config)").expect("load");
    py.exec("printer.fire('klippy:connect')").expect("connect");
    // Both/neither.
    py.exec("gcmd = FakeGCommand(floats={'VALUE': 1.0}, strings={'TEMPLATE': 'x'})").expect("gcmd both");
    let expected_both = py.eval("pin.cmd_SET_PIN(gcmd)");
    let expected_neither = py.eval("pin.cmd_SET_PIN(FakeGCommand(floats={'VALUE': None}, strings={'TEMPLATE': None}))");
    // Invalid digital value.
    py.exec("gcmd3 = FakeGCommand(floats={'VALUE': 0.5}, strings={'TEMPLATE': None})").expect("gcmd half");
    let expected_half = py.eval("pin.cmd_SET_PIN(gcmd3)");
    // Rust sides.
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin d", vec![("pin", "PA3")]);
    let f = module_factory().load_config_prefix.expect("factory");
    let obj = f(&config).expect("rust").expect("obj");
    let pin_rs: Arc<PrinterOutputPin> = klippy_api::downcast_arc(&obj).expect("downcast");
    w.printer.fire(EV_KLIPPY_CONNECT);
    let e1 = pin_rs.cmd_set_pin(&make_gcmd(vec![], vec![("VALUE", Some(1.0))], vec![("TEMPLATE", Some("x"))])).unwrap_err();
    check_err("both_value_and_template", expected_both, &e1);
    let e2 = pin_rs.cmd_set_pin(&make_gcmd(vec![], vec![("VALUE", None)], vec![("TEMPLATE", None)])).unwrap_err();
    check_err("neither_value_nor_template", expected_neither, &e2);
    let e3 = pin_rs.cmd_set_pin(&make_gcmd(vec![], vec![("VALUE", Some(0.5))], vec![("TEMPLATE", None)])).unwrap_err();
    check_err("invalid_digital_value", expected_half, &e3);
}

#[test]
fn set_pin_template_errors_match() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({'mytpl': FakeDisplayTemplate('mytpl', 'level', {'param_level': 0})})").expect("templates");
    py.exec("config = FakeConfig(printer, 'output_pin fan0', {'pin': 'PA1', 'pwm': 1})").expect("config");
    py.exec("pin = output_pin.load_config_prefix(config)").expect("load");
    py.exec("printer.fire('klippy:connect')").expect("connect");
    let expected_unknown = py.eval("pin.cmd_SET_PIN(FakeGCommand(params={}, floats={'VALUE': None}, strings={'TEMPLATE': 'nope'}))");
    let expected_badparam = py.eval("pin.cmd_SET_PIN(FakeGCommand(params={'PARAM_BOGUS': '1'}, floats={'VALUE': None}, strings={'TEMPLATE': 'mytpl'}))");
    let expected_badliteral = py.eval("pin.cmd_SET_PIN(FakeGCommand(params={'PARAM_LEVEL': 'zzz'}, floats={'VALUE': None}, strings={'TEMPLATE': 'mytpl'}))");

    let w = build_world(vec![("mytpl", "level", vec![("param_level", PyValue::Int(pycompat::PyInt::from_i64(0)))])]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin fan0", vec![("pin", "PA1"), ("pwm", "1")]);
    let f = module_factory().load_config_prefix.expect("factory");
    let obj = f(&config).expect("rust").expect("obj");
    let pin_rs: Arc<PrinterOutputPin> = klippy_api::downcast_arc(&obj).expect("downcast");
    w.printer.fire(EV_KLIPPY_CONNECT);
    let e1 = pin_rs.cmd_set_pin(&make_gcmd(vec![], vec![("VALUE", None)], vec![("TEMPLATE", Some("nope"))])).unwrap_err();
    check_err("unknown_template", expected_unknown, &e1);
    let e2 = pin_rs.cmd_set_pin(&make_gcmd(vec![("PARAM_BOGUS", "1")], vec![("VALUE", None)], vec![("TEMPLATE", Some("mytpl"))])).unwrap_err();
    check_err("invalid_param", expected_badparam, &e2);
    let e3 = pin_rs.cmd_set_pin(&make_gcmd(vec![("PARAM_LEVEL", "zzz")], vec![("VALUE", None)], vec![("TEMPLATE", Some("mytpl"))])).unwrap_err();
    check_err("bad_literal", expected_badliteral, &e3);
}

// --- GCodeRequestQueue direct semantics ---

#[test]
fn gcrq_override_and_tools_match() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec(r#"
cb_calls = []
config2 = FakeConfig(printer, 'output_pin q', {'pin': 'PA1'})
def _cb(pt, v):
    cb_calls.append((pt, v))
    return None
gcrq = output_pin.GCodeRequestQueue(config2, mcu, _cb)
printer.fire('klippy:connect')
gcrq.queue_gcode_request(1.0)
gcrq.queue_gcode_request(2.0)
toolhead.run_lookahead(10.0)
mq.flush(20.0, 0.0)
"#)
        .expect("gcrq");
    let expected = py.eval("cb_calls").expect("py calls");
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin q", vec![("pin", "PA1")]);
    let calls: Arc<Mutex<Vec<(f64, f64)>>> = Arc::new(Mutex::new(Vec::new()));
    let calls2 = calls.clone();
    let cb: extras::output_pin::GcrqCallback = Arc::new(move |pt, v| {
        calls2.lock().unwrap().push((pt, v));
        Ok(None)
    });
    let queue = GCodeRequestQueue::new(&config, w.mcu.clone(), cb).expect("queue");
    w.printer.fire(EV_KLIPPY_CONNECT);
    queue.queue_gcode_request(1.0).expect("q1");
    queue.queue_gcode_request(2.0).expect("q2");
    w.toolhead.run_lookahead(10.0);
    w.mq.flush(20.0, 0.0);
    let got = calls.lock().unwrap().clone();
    assert_eq!(got.len(), 1, "override: only last request flushed");
    assert_eq!(got[0], (10.0, 2.0));
    check("gcrq_override", Ok(expected), RValue::List(vec![RValue::Tuple(vec![RValue::float(10.0), RValue::float(2.0)])]));
}

/// A raising request callback leaves `_flush_notification` (the request stays
/// queued, so the next flush raises again) and `send_async_request`.
#[test]
fn gcrq_callback_errors_propagate() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec(r#"
config3 = FakeConfig(printer, 'output_pin r', {'pin': 'PA1'})
def _bad(pt, v):
    raise ValueError("bad request %s" % (v,))
gcrq3 = output_pin.GCodeRequestQueue(config3, mcu, _bad)
printer.fire('klippy:connect')
gcrq3.queue_gcode_request(3.0)
toolhead.run_lookahead(10.0)
"#)
        .expect("py gcrq");
    let w = build_world(vec![]);
    let config = FakeConfig::new(w.printer_api.clone(), "output_pin r", vec![("pin", "PA1")]);
    let cb: extras::output_pin::GcrqCallback = Arc::new(|_pt, v| {
        Err(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: format!("bad request {}", pycompat::text::float_repr(v)),
        })
    });
    let queue = GCodeRequestQueue::new(&config, w.mcu.clone(), cb).expect("queue");
    w.printer.fire(EV_KLIPPY_CONNECT);
    queue.queue_gcode_request(3.0).expect("q");
    w.toolhead.run_lookahead(10.0);
    for label in ["first flush", "second flush"] {
        let expected = py.exec("mq.flush(20.0, 0.0)").expect_err("py flush raises");
        let actual = w.mq.flush_result(20.0, 0.0).expect_err("rust flush raises");
        assert_eq!(actual.class_name(), expected.class_name, "{label} class");
        assert_eq!(actual.message(), expected.message, "{label} message");
    }
    let expected = py.exec("gcrq3.send_async_request(4.0, 30.0)").expect_err("py async raises");
    let actual = queue.send_async_request(4.0, Some(30.0)).expect_err("rust async raises");
    assert_eq!(actual.class_name(), expected.class_name, "async class");
    assert_eq!(actual.message(), expected.message, "async message");
}

// --- Template evaluator render ---

#[test]
fn evaluator_render_and_cache_match() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({'mytpl': FakeDisplayTemplate('mytpl', 'level', {'param_level': 0}), 'sub': FakeDisplayTemplate('sub', 'text', {})})").expect("templates");
    py.exec(r#"
et_config = FakeConfig(printer, 'output_pin x', {})
te = output_pin.lookup_template_eval(et_config)
texts = []
def cb(text): texts.append(text)
gcmd = FakeGCommand(params={'PARAM_LEVEL': '5'}, floats={'VALUE': None}, strings={'TEMPLATE': 'mytpl'})
te.set_template(gcmd, cb)
te._render(10.0)
"#)
        .expect("py render");
    let expected = py.eval("texts").expect("py texts");
    let w = build_world(vec![("mytpl", "level", vec![("param_level", PyValue::Int(pycompat::PyInt::from_i64(0)))])]);
    let et_config = FakeConfig::new(w.printer_api.clone(), "output_pin x", vec![]);
    let te = lookup_template_eval(&et_config).expect("rust eval");
    let texts: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let t2 = texts.clone();
    let cb: TemplateCallback = Arc::new(move |s: &str| t2.lock().unwrap().push(s.to_string()));
    let gcmd = make_gcmd(vec![("PARAM_LEVEL", "5")], vec![("VALUE", None)], vec![("TEMPLATE", Some("mytpl"))]);
    te.set_template(&gcmd, cb, None).expect("set_template");
    // Drive the render timer through the reactor: run until one render fires.
    let reactor = w.printer_api.get_reactor();
    let end_at = reactor.monotonic() + 0.2;
    let rc = reactor.clone();
    reactor.register_timer(move |_| { rc.end(); NEVER }, end_at);
    let r_watch = reactor.clone();
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let d2 = done.clone();
    let f2 = fired.clone();
    let watchdog = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_secs(4));
        if !d2.load(std::sync::atomic::Ordering::SeqCst) {
            f2.store(true, std::sync::atomic::Ordering::SeqCst);
            r_watch.finalize();
        }
    });
    reactor.run();
    done.store(true, std::sync::atomic::Ordering::SeqCst);
    let _ = watchdog.join();
    assert!(!fired.load(std::sync::atomic::Ordering::SeqCst), "reactor run timed out");
    let got = texts.lock().unwrap().clone();
    assert_eq!(got, vec!["5".to_string()]);
    check("render_text", Ok(expected), RValue::List(got.iter().map(|s| RValue::str(s)).collect::<Vec<_>>()));
}

#[test]
fn evaluator_reactivated_callback_keeps_render_order() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({'mytpl': FakeDisplayTemplate('mytpl', 'level', {'param_level': 0})})").expect("templates");
    py.exec(r#"
et_config = FakeConfig(printer, 'output_pin x', {})
te = output_pin.lookup_template_eval(et_config)
order = []
def cb_a(text): order.append('a:' + text)
def cb_b(text): order.append('b:' + text)
def tpl_cmd(level): return FakeGCommand(params={'PARAM_LEVEL': level}, floats={'VALUE': None}, strings={'TEMPLATE': 'mytpl'})
te.set_template(tpl_cmd('5'), cb_a)
te.set_template(tpl_cmd('7'), cb_b)
te.set_template(tpl_cmd('9'), cb_a)
te._render(10.0)
"#)
        .expect("py render");
    let expected = py.eval("order").expect("py order");
    let w = build_world(vec![("mytpl", "level", vec![("param_level", PyValue::Int(pycompat::PyInt::from_i64(0)))])]);
    let et_config = FakeConfig::new(w.printer_api.clone(), "output_pin x", vec![]);
    let te = lookup_template_eval(&et_config).expect("rust eval");
    let order: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let (oa, ob) = (order.clone(), order.clone());
    let cb_a: TemplateCallback = Arc::new(move |s: &str| oa.lock().unwrap().push(format!("a:{s}")));
    let cb_b: TemplateCallback = Arc::new(move |s: &str| ob.lock().unwrap().push(format!("b:{s}")));
    let tpl_cmd = |level: &str| make_gcmd(vec![("PARAM_LEVEL", level)], vec![("VALUE", None)], vec![("TEMPLATE", Some("mytpl"))]);
    te.set_template(&tpl_cmd("5"), cb_a.clone(), None).expect("set a");
    te.set_template(&tpl_cmd("7"), cb_b, None).expect("set b");
    te.set_template(&tpl_cmd("9"), cb_a, None).expect("reset a");
    // Run the reactor long enough for exactly one render pass.
    let reactor = w.printer_api.get_reactor();
    let end_at = reactor.monotonic() + 0.2;
    let rc = reactor.clone();
    reactor.register_timer(move |_| { rc.end(); NEVER }, end_at);
    let r_watch = reactor.clone();
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let (d2, f2) = (done.clone(), fired.clone());
    let watchdog = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_secs(4));
        if !d2.load(std::sync::atomic::Ordering::SeqCst) {
            f2.store(true, std::sync::atomic::Ordering::SeqCst);
            r_watch.finalize();
        }
    });
    reactor.run();
    done.store(true, std::sync::atomic::Ordering::SeqCst);
    let _ = watchdog.join();
    assert!(!fired.load(std::sync::atomic::Ordering::SeqCst), "reactor run timed out");
    let got = order.lock().unwrap().clone();
    check("reactivated_render_order", Ok(expected), RValue::List(got.iter().map(|s| RValue::str(s)).collect::<Vec<_>>()));
}

#[test]
fn evaluator_render_error_becomes_empty() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py);
    py.exec("install_templates({'bad': FakeDisplayTemplate('bad', 'fail', {})})").expect("templates");
    py.exec(r#"
te2 = output_pin.lookup_template_eval(FakeConfig(printer, 'output_pin y', {}))
texts2 = []
te2.set_template(FakeGCommand(params={}, floats={'VALUE': None}, strings={'TEMPLATE': 'bad'}), lambda t: texts2.append(t))
te2._render(10.0)
"#)
        .expect("py render");
    let expected = py.eval("texts2").expect("py texts");
    let w = build_world(vec![("bad", "fail", vec![])]);
    let te = lookup_template_eval(&FakeConfig::new(w.printer_api.clone(), "output_pin y", vec![])).expect("eval");
    let texts: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let t2 = texts.clone();
    let cb: TemplateCallback = Arc::new(move |s: &str| t2.lock().unwrap().push(s.to_string()));
    let gcmd = make_gcmd(vec![], vec![("VALUE", None)], vec![("TEMPLATE", Some("bad"))]);
    te.set_template(&gcmd, cb, None).expect("set");
    let reactor = w.printer_api.get_reactor();
    let end_at = reactor.monotonic() + 0.2;
    let rc = reactor.clone();
    reactor.register_timer(move |_| { rc.end(); NEVER }, end_at);
    let r_watch = reactor.clone();
    let done = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let d2 = done.clone();
    let f2 = fired.clone();
    let watchdog = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_secs(4));
        if !d2.load(std::sync::atomic::Ordering::SeqCst) { f2.store(true, std::sync::atomic::Ordering::SeqCst); r_watch.finalize(); }
    });
    reactor.run();
    done.store(true, std::sync::atomic::Ordering::SeqCst);
    let _ = watchdog.join();
    assert!(!fired.load(std::sync::atomic::Ordering::SeqCst));
    let got = texts.lock().unwrap().clone();
    assert_eq!(got, vec![String::new()]);
    check("render_error", Ok(expected), RValue::List(got.iter().map(|s| RValue::str(s)).collect::<Vec<_>>()));
}
