//! Parity tests for `extras::fan_generic` (port of `klippy/extras/fan_generic.py`).
//!
//! The reference is driven through the real `extras.fan_generic` module,
//! which builds the real `extras.fan` fan and the real
//! `output_pin.PrinterTemplateEvaluator` on Python fakes for the printer,
//! config, pins, MCU, motion queuing, toolhead and gcode; the port builds the
//! same objects through `extras::fan_generic` on the Rust fakes below.
//! Expected values come from the live oracle only (there are no recordings
//! for this unit); floats are compared bit-for-bit through the recording
//! value format and errors by class and message.
//!
//! Compared: construction (the fan with `default_shutdown_speed=0.`, every
//! fan config option with its default, pin setup calls, `fan_name`,
//! `SET_FAN_SPEED`/`FAN` mux registration and help text, the
//! `template_evaluator` lookup), `SET_FAN_SPEED` with `SPEED` driven through
//! toolhead lookahead and the motion-queue flush (PWM calls, movequeue
//! activity, status), every `SET_FAN_SPEED` error path, the `TEMPLATE` path
//! through the real template evaluator, `_template_update` for valid and
//! invalid texts (log level/message plus the traceback exception text) and
//! `get_status` delegation.
//!
//! The integration owner runs:
//!
//! ```sh
//! cargo test -p extras --test fan_generic
//! ```

#![allow(clippy::type_complexity)]

use std::sync::Arc;

use extras::fan_generic::{
    load_config_prefix, module_factory, PrinterFanGeneric, CMD_SET_FAN_SPEED_HELP,
};
use extras::display::display::{DisplayGroup, DisplayTemplateApi, GlyphTable, PrinterDisplayTemplateApi};
use extras::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts, GetIntListOpts,
    GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCmd, GCodeApi, GCodeCommandApi, GetFloatOpts as GcmdFloatOpts, GetOpts as GcmdGetOpts,
    GetParser, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AsyncResponseWrapper, AsyncResponseWrapperApi, Chip, CommandQueryWrapper, CommandQueue,
    CommandWrapper, ConfigCallback, DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts, Mcu,
    McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, PwmPinApi, SerialResponseCallback,
    TriggerDispatch,
};
use klippy_api::object::{EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{
    ExtraAxis, LookaheadCallback, MotionQueuingApi, PrinterExtruder, ToolHeadApi,
};
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

/// `kos::sync::Mutex` with `Default`, so the fakes can derive it.
struct Mutex<T>(kos::sync::Mutex<T>);

impl<T> Mutex<T> {
    fn new(value: T) -> Self {
        Mutex(kos::sync::Mutex::new(value))
    }
    fn lock(&self) -> kos::sync::MutexGuard<'_, T> {
        self.0.lock()
    }
}

impl<T: Default> Default for Mutex<T> {
    fn default() -> Self {
        Mutex::new(T::default())
    }
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process. They mirror the Rust
// fakes below: same MCU constants, same pin fan-out, same error messages.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging
import extras.fan_generic as fan_generic
import gcode as gcode_module

class Error(Exception):
    pass
error = Error

FREQ = 12000000.0

class _Sentinel: pass
_sentinel = _Sentinel()

_log_records = []
class _LogHandler(logging.Handler):
    def emit(self, record):
        _log_records.append((record.levelname, record.getMessage(),
                             record.exc_info is not None))
logging.getLogger().addHandler(_LogHandler(level=0))
logging.getLogger().setLevel(0)

class FakeReactor:
    def monotonic(self):
        return 1000.0

class FakeMCU:
    name = 'mcu'
    def __init__(self):
        self.min_schedule = 0.1
    def min_schedule_time(self):
        return self.min_schedule
    def estimated_print_time(self, eventtime):
        return eventtime

class FakePwmPin:
    def __init__(self, mcu):
        self._mcu = mcu
    def setup_max_duration(self, d):
        pwm_setup.append(('max_duration', d))
    def setup_cycle_time(self, t, hw):
        pwm_setup.append(('cycle_time', t, hw))
    def setup_start_value(self, s, sh):
        pwm_setup.append(('start_value', s, sh))
    def set_pwm(self, print_time, value):
        pwm_calls.append((print_time, value))
    def get_mcu(self):
        return self._mcu

class FakeDigitalPin:
    def __init__(self, mcu):
        self._mcu = mcu
    def setup_max_duration(self, d):
        digital_setup.append(('max_duration', d))
    def set_digital(self, print_time, value):
        digital_calls.append((print_time, bool(value)))
    def get_mcu(self):
        return self._mcu

class FakePins:
    def __init__(self, mcu):
        self.mcu = mcu
    def parse_pin(self, pin_desc, can_invert=False, can_pullup=False):
        desc = pin_desc.strip()
        pullup = invert = 0
        if can_pullup and (desc.startswith('^') or desc.startswith('~')):
            pullup = 1
            if desc.startswith('~'):
                pullup = -1
            desc = desc[1:].strip()
        if can_invert and desc.startswith('!'):
            invert = 1
            desc = desc[1:].strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        if chip_name != 'mcu':
            raise error("Unknown pin chip name '%s'" % (chip_name,))
        if [c for c in '^~!:' if c in pin] or ''.join(pin.split()) != pin:
            raise error("Invalid pin description '%s'" % (pin_desc,))
        return {'chip': self.mcu, 'chip_name': chip_name, 'pin': pin,
                'invert': invert, 'pullup': pullup}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        return self.parse_pin(pin_desc, can_invert, can_pullup)
    def setup_pin(self, pin_type, pin_desc):
        if pin_type == 'pwm':
            self.parse_pin(pin_desc, can_invert=True)
            return FakePwmPin(self.mcu)
        if pin_type == 'digital_out':
            self.parse_pin(pin_desc, can_invert=True)
            return FakeDigitalPin(self.mcu)
        raise error("unknown pin type '%s'" % (pin_type,))

class FakeMq:
    def __init__(self):
        self.flush_cbs = []
        self.activities = []
    def register_flush_callback(self, cb, can_add_trapq=False):
        self.flush_cbs.append(cb)
    def note_mcu_movequeue_activity(self, t, is_step_gen=False):
        self.activities.append((t, is_step_gen))
    def flush(self, must):
        for cb in list(self.flush_cbs):
            cb(must, 0.0)

class FakeToolHead:
    def __init__(self):
        self.cbs = []
    def register_lookahead_callback(self, cb):
        self.cbs.append(cb)
    def fire(self, pt):
        cbs, self.cbs = self.cbs, []
        for cb in cbs:
            cb(pt)

class FakeGcode:
    def __init__(self):
        self.mux_calls = []
        self.handlers = {}
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux_calls.append((cmd, key, value, desc))
        self.handlers[(cmd, key, value)] = func
    def respond_raw(self, msg):
        pass
    def respond_info(self, msg, log=True):
        pass

class FakeConfigFile:
    def read_config(self, filename):
        return _FakeSectionSource()

class _FakeSectionSource:
    def get_prefix_sections(self, prefix):
        return []

class FakeGCodeMacro:
    def create_template_context(self, eventtime):
        return {}

class FakePrinter:
    def __init__(self):
        self.mcu = FakeMCU()
        self.pins = FakePins(self.mcu)
        self.mq = FakeMq()
        self.toolhead = FakeToolHead()
        self.gcode = FakeGcode()
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
        self.objects['gcode_macro'] = FakeGCodeMacro()
        self.objects['configfile'] = FakeConfigFile()
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=_sentinel):
        if name == 'pins':
            return self.pins
        if name == 'motion_queuing':
            return self.mq
        if name == 'toolhead':
            return self.toolhead
        if name == 'gcode':
            return self.gcode
        if name in self.objects:
            return self.objects[name]
        if default is _sentinel:
            raise error("Unknown config object '%s'" % (name,))
        return default
    def load_object(self, config, section, default=_sentinel):
        if section in self.objects:
            return self.objects[section]
        return self.lookup_object(section, default)
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def send_event(self, event, *params):
        for cb in list(self.handlers.get(event, [])):
            cb(*params)
    def add_object(self, name, obj):
        self.objects[name] = obj

class FakeConfig:
    error = error
    def __init__(self, printer, name, values):
        self.printer = printer
        self.name = name
        self.values = values
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.name
    def get(self, option, default=_sentinel):
        if option in self.values:
            return self.values[option]
        if default is _sentinel:
            raise error("Option '%s' in section '%s' must be specified"
                        % (option, self.name))
        return default
    def getint(self, option, default=_sentinel, minval=None, maxval=None):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified"
                            % (option, self.name))
            return default
        try:
            v = int(self.values[option].strip())
        except:
            raise error("Option '%s' is not an integer" % (option,))
        if minval is not None and v < minval:
            raise error("Option '%s' must be minimum %s" % (option, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' must be maximum %s" % (option, maxval))
        return v
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None,
                 above=None, below=None):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified"
                            % (option, self.name))
            return default
        try:
            v = float(self.values[option].strip())
        except:
            raise error("Option '%s' is not a float" % (option,))
        if minval is not None and v < minval:
            raise error("Option '%s' must be minimum %s" % (option, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' must be maximum %s" % (option, maxval))
        if above is not None and v <= above:
            raise error("Option '%s' must be above %s" % (option, above))
        if below is not None and v >= below:
            raise error("Option '%s' must be below %s" % (option, below))
        return v
    def getboolean(self, option, default=_sentinel):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified"
                            % (option, self.name))
            return default
        v = self.values[option].strip().lower()
        if v in ('true', 'yes', 'on', '1'):
            return True
        if v in ('false', 'no', 'off', '0'):
            return False
        raise error("Option '%s' is not a boolean" % (option,))
    def get_prefix_sections(self, prefix):
        return []
    def get_prefix_options(self, prefix):
        return []

pwm_calls = []
digital_calls = []
pwm_setup = []
digital_setup = []

def setup(vals, name='fan_generic myfan'):
    pr = FakePrinter()
    pwm_setup.clear()
    pwm_calls.clear()
    digital_setup.clear()
    digital_calls.clear()
    cfg = FakeConfig(pr, name, vals)
    fg = fan_generic.PrinterFanGeneric(cfg)
    pr.send_event('klippy:connect')
    return pr, cfg, fg

def make_gcmd(pr, params, commandline):
    return gcode_module.GCodeCommand(pr.gcode, 'SET_FAN_SPEED', commandline,
                                     params, False)

def call_mux(pr, params, commandline, fan='myfan'):
    gcmd = make_gcmd(pr, params, commandline)
    return pr.gcode.handlers[('SET_FAN_SPEED', 'FAN', fan)](gcmd)

def pwm_values():
    return [v for t, v in pwm_calls]

def digital_values():
    return [v for t, v in digital_calls]
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Default)]
struct FakePrinter {
    inner: Arc<Mutex<PrinterInner>>,
}

#[derive(Default)]
struct PrinterInner {
    objects: IndexMap<String, ObjectRef>,
    handlers: IndexMap<String, Vec<Box<dyn FnMut(&[EventArg]) -> KResult<PyValue> + Send>>>,
    reactor: Option<Reactor>,
}

impl FakePrinter {
    fn new() -> Self {
        let p = Self::default();
        p.inner.lock().reactor = Some(Reactor::new(false));
        p
    }
    fn reactor(&self) -> Reactor {
        self.inner.lock().reactor.clone().unwrap()
    }
    fn send(&self, event: &str, params: &[EventArg]) {
        let mut cbs: Vec<_> = {
            let mut inner = self.inner.lock();
            inner.handlers.shift_remove(event).unwrap_or_default()
        };
        for cb in cbs.iter_mut() {
            cb(params).unwrap();
        }
        if !cbs.is_empty() {
            let mut inner = self.inner.lock();
            inner.handlers.entry(event.to_string()).or_default().extend(cbs);
        }
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor()
    }
    fn get_state_message(&self) -> (String, String) {
        ("ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old: &str, _new: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.inner.lock();
        if inner.objects.contains_key(name) {
            return Err(KlippyError::config(format!(
                "Printer object '{}' already created",
                name
            )));
        }
        inner.objects.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let inner = self.inner.lock();
        Ok(match inner.objects.get(name) {
            Some(o) => Some(o.clone()),
            None => match default {
                LookupDefault::Raise => {
                    return Err(KlippyError::config(format!(
                        "Unknown config object '{}'",
                        name
                    )))
                }
                LookupDefault::ReturnNone => None,
            },
        })
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.inner
            .lock()
            .objects
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
        self.lookup_object(section, default)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: klippy_api::printer::EventCallback) {
        self.inner
            .lock()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut cbs: Vec<_> = {
            let mut inner = self.inner.lock();
            inner.handlers.shift_remove(event).unwrap_or_default()
        };
        let mut rets = Vec::new();
        for cb in cbs.iter_mut() {
            rets.push(cb(params)?);
        }
        if !cbs.is_empty() {
            let mut inner = self.inner.lock();
            inner.handlers.entry(event.to_string()).or_default().extend(cbs);
        }
        Ok(rets)
    }
    fn request_exit(&self, _r: &str) {}
}

klippy_api::printer_object!(FakePrinter);

#[derive(Clone)]
struct FakeConfig {
    printer: Printer,
    name: String,
    values: Arc<Mutex<IndexMap<String, String>>>,
}

impl FakeConfig {
    fn new(printer: Printer, name: &str, vals: &[(&str, &str)]) -> Self {
        let mut m = IndexMap::new();
        for (k, v) in vals {
            m.insert(k.to_string(), v.to_string());
        }
        FakeConfig {
            printer,
            name: name.to_string(),
            values: Arc::new(Mutex::new(m)),
        }
    }
    fn get_raw(&self, opt: &str) -> Option<String> {
        self.values.lock().get(opt).cloned()
    }
}

fn missing(section: &str, opt: &str) -> KlippyError {
    KlippyError::config(format!(
        "Option '{}' in section '{}' must be specified",
        opt, section
    ))
}

fn parse_int(s: &str) -> Option<i64> {
    s.trim().parse().ok()
}
fn parse_float(s: &str) -> Option<f64> {
    s.trim().parse().ok()
}
fn parse_bool(s: &str) -> Option<bool> {
    match s.trim().to_ascii_lowercase().as_str() {
        "true" | "yes" | "on" | "1" => Some(true),
        "false" | "no" | "off" | "0" => Some(false),
        _ => None,
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
        match self.get_raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(missing(&self.name, option)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.get_raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(missing(&self.name, option)),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = parse_int(&raw)
            .ok_or_else(|| KlippyError::config(format!("Option '{}' is not an integer", option)))?;
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be minimum {}",
                    option, mn
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be maximum {}",
                    option, mx
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let raw = match self.get_raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(missing(&self.name, option)),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = parse_float(&raw)
            .ok_or_else(|| KlippyError::config(format!("Option '{}' is not a float", option)))?;
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be minimum {:?}",
                    option, mn
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be maximum {:?}",
                    option, mx
                )));
            }
        }
        if let Some(a) = opts.above {
            if v <= a {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be above {:?}",
                    option, a
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::config(format!(
                    "Option '{}' must be below {:?}",
                    option, b
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        match self.get_raw(option) {
            Some(v) => parse_bool(&v).map(Some).ok_or_else(|| {
                KlippyError::config(format!("Option '{}' is not a boolean", option))
            }),
            None => match opts.default {
                None => Err(missing(&self.name, option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Err(KlippyError::config("getchoice not faked"))
    }
    fn getlists(&self, _o: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Err(KlippyError::config("getlists not faked"))
    }
    fn getlist(&self, _o: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::config("getlist not faked"))
    }
    fn getintlist(&self, _o: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::config("getintlist not faked"))
    }
    fn getfloatlist(&self, _o: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::config("getfloatlist not faked"))
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection not faked")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> {
        Ok(Vec::new())
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

#[derive(Clone, Default)]
struct FakeMcu {
    inner: Arc<Mutex<McuInner>>,
}

#[derive(Default)]
struct McuInner {
    oid_next: i64,
    config_cmds: Vec<(String, bool, bool)>,
}

impl FakeMcu {
    fn new() -> Self {
        Self::default()
    }
}

struct FakeAsyncResp;
impl AsyncResponseWrapperApi for FakeAsyncResp {
    fn unregister(&self) {}
}

struct FakeCmd;
impl klippy_api::mcu::CommandWrapperApi for FakeCmd {
    fn send(&self, _d: Vec<PyValue>, _o: klippy_api::mcu::SendOpts) -> KResult<()> {
        Ok(())
    }
    fn send_wait_ack(&self, _d: Vec<PyValue>, _o: klippy_api::mcu::SendOpts) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        1
    }
}

struct FakeQuery;
impl klippy_api::mcu::CommandQueryWrapperApi for FakeQuery {
    fn send(&self, _d: Vec<PyValue>, _o: klippy_api::mcu::QuerySendOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn send_with_preface(
        &self,
        _p: CommandWrapper,
        _pd: Vec<PyValue>,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        panic!("mcu.get_printer not faked")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _t: &str, _p: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("mcu.setup_pin not faked"))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut inner = self.inner.lock();
        let oid = inner.oid_next;
        inner.oid_next += 1;
        Ok(oid)
    }
    fn register_config_callback(&self, _cb: ConfigCallback) -> KResult<()> {
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: klippy_api::mcu::AddConfigCmdOpts) -> KResult<()> {
        self.inner
            .lock()
            .config_cmds
            .push((cmd.to_string(), opts.is_init, opts.on_restart));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, oid: i64) -> i64 {
        oid + 100
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 12_000_000.0) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(&self, _m: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd))
    }
    fn lookup_query_command(
        &self,
        _m: &str,
        _r: &str,
        _o: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        Ok(Arc::new(FakeQuery))
    }
    fn try_lookup_command(&self, _m: &str) -> Option<CommandWrapper> {
        Some(Arc::new(FakeCmd))
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        _cb: SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        Arc::new(FakeAsyncResp)
    }
    fn check_valid_response(&self, _m: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _n: &str) -> KResult<f64> {
        Ok(0.0)
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * 12_000_000.0) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 12_000_000.0
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 12_000_000.0))
    }
    fn get_status(&self, _e: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (true, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        Err(KlippyError::config("not faked"))
    }
}

#[derive(Clone, Default)]
struct FakePwm {
    inner: Arc<Mutex<PwmInner>>,
    mcu: Option<Mcu>,
}

#[derive(Default)]
struct PwmInner {
    max_duration: Vec<f64>,
    cycle: Vec<(f64, Option<bool>)>,
    start: Vec<(f64, f64)>,
    pwm_calls: Vec<(f64, f64)>,
}

impl FakePwm {
    fn new(mcu: Mcu) -> Self {
        FakePwm {
            inner: Arc::new(Mutex::new(PwmInner::default())),
            mcu: Some(mcu),
        }
    }
    fn pwm_calls(&self) -> Vec<(f64, f64)> {
        self.inner.lock().pwm_calls.clone()
    }
    fn setup_value(&self) -> Value {
        let inner = self.inner.lock();
        let mut out = Vec::new();
        for d in &inner.max_duration {
            out.push(Value::Tuple(vec![
                Value::str("max_duration"),
                Value::float(*d),
            ]));
        }
        for (t, hw) in &inner.cycle {
            out.push(Value::Tuple(vec![
                Value::str("cycle_time"),
                Value::float(*t),
                Value::Bool(hw.expect("fan passes hardware_pwm")),
            ]));
        }
        for (s, sh) in &inner.start {
            out.push(Value::Tuple(vec![
                Value::str("start_value"),
                Value::float(*s),
                Value::float(*sh),
            ]));
        }
        Value::List(out)
    }
}

impl PwmPinApi for FakePwm {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone().expect("FakePwm mcu bound")
    }
    fn setup_max_duration(&self, d: f64) {
        self.inner.lock().max_duration.push(d);
    }
    fn setup_cycle_time(&self, t: f64, hw: Option<bool>) -> klippy_api::KResult<()> {
        self.inner.lock().cycle.push((t, hw));
        Ok(())
    }
    fn setup_start_value(&self, s: f64, sh: f64) {
        self.inner.lock().start.push((s, sh));
    }
    fn next_aligned_print_time(&self, t: f64, _a: Option<f64>) -> f64 {
        t
    }
    fn set_pwm(&self, print_time: f64, value: f64) -> klippy_api::KResult<()> {
        self.inner.lock().pwm_calls.push((print_time, value));
        Ok(())
    }
}

#[derive(Clone, Default)]
struct FakeDigital {
    inner: Arc<Mutex<DigitalInner>>,
    mcu: Option<Mcu>,
}

#[derive(Default)]
struct DigitalInner {
    max_duration: Vec<f64>,
    digital_calls: Vec<(f64, bool)>,
}

impl FakeDigital {
    fn new(mcu: Mcu) -> Self {
        FakeDigital {
            inner: Arc::new(Mutex::new(DigitalInner::default())),
            mcu: Some(mcu),
        }
    }
    fn digital_calls(&self) -> Vec<(f64, bool)> {
        self.inner.lock().digital_calls.clone()
    }
    fn setup_value(&self) -> Value {
        Value::List(
            self.inner
                .lock()
                .max_duration
                .iter()
                .map(|d| Value::Tuple(vec![Value::str("max_duration"), Value::float(*d)]))
                .collect(),
        )
    }
}

impl DigitalOutPinApi for FakeDigital {
    fn get_mcu(&self) -> Mcu {
        self.mcu.clone().expect("FakeDigital mcu bound")
    }
    fn setup_max_duration(&self, d: f64) {
        self.inner.lock().max_duration.push(d);
    }
    fn setup_start_value(&self, _s: bool, _sh: bool) {}
    fn set_digital(&self, print_time: f64, value: bool) {
        self.inner.lock().digital_calls.push((print_time, value));
    }
}

#[derive(Clone)]
struct FakePins {
    mcu: Mcu,
    pwm: Arc<FakePwm>,
    digital: Arc<FakeDigital>,
}

impl FakePins {
    fn new(mcu: Mcu) -> Self {
        FakePins {
            pwm: Arc::new(FakePwm::new(mcu.clone())),
            digital: Arc::new(FakeDigital::new(mcu.clone())),
            mcu,
        }
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let pin = if let Some((_, p)) = desc.split_once(':') {
            p.trim().to_string()
        } else {
            desc.trim_start_matches(['!', '^', '~']).to_string()
        };
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin,
            invert: 0,
            pullup: 0,
            share_type: None,
        })
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let mut p = self.parse_pin(
            pin_desc,
            ParsePinOpts {
                can_invert: opts.can_invert,
                can_pullup: opts.can_pullup,
            },
        )?;
        p.share_type = opts.share_type;
        Ok(p)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        match pin_type {
            "pwm" => {
                let _ = self.parse_pin(
                    pin_desc,
                    ParsePinOpts {
                        can_invert: true,
                        can_pullup: false,
                    },
                )?;
                Ok(Pin::Pwm(self.pwm.clone()))
            }
            "digital_out" => {
                let _ = self.parse_pin(
                    pin_desc,
                    ParsePinOpts {
                        can_invert: true,
                        can_pullup: false,
                    },
                )?;
                Ok(Pin::DigitalOut(self.digital.clone()))
            }
            _ => Err(KlippyError::config(format!(
                "unknown pin type '{}'",
                pin_type
            ))),
        }
    }
    fn reset_pin_sharing(&self, _p: PinParams) -> klippy_api::KResult<()> { Ok(()) }
    fn get_pin_resolver(&self, _c: &str) -> KResult<PinResolver> {
        Err(KlippyError::config("no resolver"))
    }
    fn register_chip(&self, _c: &str, _chip: Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _p: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

#[derive(Clone, Default)]
struct FakeMq {
    inner: Arc<Mutex<MqInner>>,
}

#[derive(Default)]
struct MqInner {
    flush_cbs: Vec<klippy_api::toolhead::FlushCallback>,
    activities: Vec<(f64, bool)>,
}

impl FakeMq {
    fn flush(&self, must_flush_time: f64) {
        let cbs = self.inner.lock().flush_cbs.clone();
        for cb in cbs {
            cb(must_flush_time, 0.0).expect("cb");
        }
    }
    fn activities(&self) -> Vec<(f64, bool)> {
        self.inner.lock().activities.clone()
    }
}

impl MotionQueuingApi for FakeMq {
    fn allocate_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("trapq not faked")
    }
    fn lookup_trapq_append(&self) -> klippy_api::toolhead::TrapqAppend {
        panic!("trapq_append not faked")
    }
    fn wipe_trapq(&self, _t: &klippy_api::stepper::Trapq) {}
    fn allocate_syncemitter(
        &self,
        _m: Mcu,
        _n: &str,
        _a: bool,
    ) -> KResult<klippy_api::toolhead::SyncEmitterHandle> {
        Err(KlippyError::config("syncemitter not faked"))
    }
    fn queue_syncemitter_msg(
        &self,
        _s: &klippy_api::toolhead::SyncEmitterHandle,
        _r: u64,
        _d: &[u32],
    ) {
    }
    fn setup_mcu_movequeue(&self, _m: Mcu, _s: Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>>, _c: i64) {}
    fn register_flush_callback(&self, cb: klippy_api::toolhead::FlushCallback, _c: bool) {
        self.inner.lock().flush_cbs.push(cb);
    }
    fn unregister_flush_callback(&self, _cb: &klippy_api::toolhead::FlushCallback) {}
    fn register_flush_callback_named(&self, cb: klippy_api::toolhead::FlushCallback, _c: bool, _n: Option<&str>, _o: Option<&str>) {
        self.register_flush_callback(cb, _c);
    }
    fn get_kin_flush_delay(&self) -> f64 {
        0.0
    }
    fn check_step_generation_scan_windows(&self) {}
    fn flush_all_steps(&self) -> KResult<()> {
        Ok(())
    }
    fn calc_step_gen_restart(&self, _e: f64) -> f64 {
        0.0
    }
    fn note_mcu_movequeue_activity(&self, mq_time: f64, is_step_gen: bool) {
        self.inner.lock().activities.push((mq_time, is_step_gen));
    }
    fn drip_update_time(
        &self,
        _s: f64,
        _e: f64,
        _d: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
    fn check_drip_timing(&self) -> Option<f64> {
        None
    }
}

klippy_api::printer_object!(FakeMq; dyn MotionQueuingApi);

#[derive(Clone, Default)]
struct FakeToolHead {
    inner: Arc<Mutex<Vec<LookaheadCallback>>>,
}

impl FakeToolHead {
    fn fire(&self, print_time: f64) {
        let cbs: Vec<LookaheadCallback> = {
            let mut slot = self.inner.lock();
            std::mem::take(&mut *slot)
        };
        for mut cb in cbs {
            cb(print_time).expect("lookahead callback");
        }
    }
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0, 0.0]
    }
    fn set_position(&self, _n: Vec<f64>, _h: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("kinematics not faked")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(0.0)
    }
    fn dwell(&self, _d: f64) -> KResult<()> { Ok(()) }
    fn drip_move(&self, _n: Vec<f64>, _s: f64, _d: &reactor::Completion<bool>) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _c: Vec<Option<f64>>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> { Ok(()) }
    fn get_status(&self, _e: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::Dict(IndexMap::new()))
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("trapq not faked")
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
    fn register_lookahead_callback(&self, callback: LookaheadCallback) -> KResult<()> {
        self.inner.lock().push(callback);
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(&self, _e: PrinterExtruder, _p: f64) {}
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("extruder not faked")
    }
    fn add_extra_axis(&self, _e: ExtraAxis, _p: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _e: &ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _e: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper factory not faked")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

/// `gcode_macro` object the template evaluator loads (`create_template_context`).
struct FakeGCodeMacro;

impl PrinterGCodeMacroApi for FakeGCodeMacro {
    fn load_template(&self, _config: &Config, _option: &str, _default: Option<&str>) -> KResult<TemplateWrapper> {
        unreachable!("fan_generic never loads templates")
    }
    fn create_template_context(
        &self,
        _eventtime: Option<f64>,
    ) -> IndexMap<String, extras_macros_templates::gcode_macro::Value> {
        IndexMap::new()
    }
}

impl PrinterObject for FakeGCodeMacro {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterGCodeMacroApi);
}

struct FakeEmptyDisplayTemplates;

impl PrinterDisplayTemplateApi for FakeEmptyDisplayTemplates {
    fn get_display_templates(&self) -> IndexMap<String, Arc<dyn DisplayTemplateApi>> {
        IndexMap::new()
    }
    fn get_display_data_groups(&self) -> IndexMap<String, Arc<DisplayGroup>> {
        IndexMap::new()
    }
    fn get_display_glyphs(&self) -> GlyphTable {
        IndexMap::new()
    }
}

impl PrinterObject for FakeEmptyDisplayTemplates {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn PrinterDisplayTemplateApi);
}

#[derive(Clone)]
struct FakeGcodeCmd {
    command: String,
    commandline: String,
    params: Arc<Mutex<IndexMap<String, String>>>,
}

impl FakeGcodeCmd {
    fn new(command: &str, commandline: &str, params: &[(&str, &str)]) -> Self {
        let mut m = IndexMap::new();
        for (k, v) in params {
            m.insert(k.to_string(), v.to_string());
        }
        FakeGcodeCmd {
            command: command.to_string(),
            commandline: commandline.to_string(),
            params: Arc::new(Mutex::new(m)),
        }
    }
    /// `GCodeCommand.error` (`klippy/gcode.py:25`).
    fn error(&self, msg: &str) -> KlippyError {
        KlippyError::command(format!("Error on '{}': {}", self.commandline, msg))
    }
}

impl GCodeCommandApi for FakeGcodeCmd {
    fn get(&self, name: &str, opts: GcmdGetOpts) -> KResult<PyValue> {
        let Some(raw) = self.params.lock().get(name).cloned() else {
            return match opts.default {
                None => Err(self.error(&format!("missing {name}"))),
                Some(d) => Ok(d),
            };
        };
        let value = match opts.parser {
            GetParser::Str => PyValue::Str(raw),
            GetParser::Float => match pycompat::text::float_from_string(&raw) {
                Ok(v) => PyValue::Float(v),
                Err(_) => return Err(self.error(&format!("unable to parse {raw}"))),
            },
            GetParser::Int => match pycompat::text::int_from_string(&raw, 10) {
                Ok(v) => PyValue::Int(v),
                Err(_) => return Err(self.error(&format!("unable to parse {raw}"))),
            },
            GetParser::IntAutoBase => match pycompat::text::int_from_string(&raw, 0) {
                Ok(v) => PyValue::Int(v),
                Err(_) => return Err(self.error(&format!("unable to parse {raw}"))),
            },
        };
        let as_float = value.to_f64();
        if let (Some(v), Some(b)) = (as_float, opts.minval.as_ref()) {
            if let Some(bf) = b.to_f64() {
                if v < bf {
                    return Err(self.error(&format!("{name} must have minimum of {}", b.str())));
                }
            }
        }
        if let (Some(v), Some(b)) = (as_float, opts.maxval.as_ref()) {
            if let Some(bf) = b.to_f64() {
                if v > bf {
                    return Err(self.error(&format!("{name} must have maximum of {}", b.str())));
                }
            }
        }
        if let (Some(v), Some(a)) = (as_float, opts.above.as_ref()) {
            if let Some(af) = a.to_f64() {
                if v <= af {
                    return Err(self.error(&format!("{name} must be above {}", a.str())));
                }
            }
        }
        if let (Some(v), Some(b)) = (as_float, opts.below.as_ref()) {
            if let Some(bf) = b.to_f64() {
                if v >= bf {
                    return Err(self.error(&format!("{name} must be below {}", b.str())));
                }
            }
        }
        Ok(value)
    }
    fn get_int(&self, name: &str, opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        let raw = self.params.lock().get(name).cloned();
        let Some(raw) = raw else {
            return match opts.default {
                None => Err(self.error(&format!("missing {name}"))),
                Some(v) => Ok(v),
            };
        };
        let v = pycompat::text::int_from_string(&raw, 10)
            .map_err(|_| self.error(&format!("unable to parse {raw}")))?;
        Ok(Some(v.0.to_string().parse::<i64>().unwrap_or(0)))
    }
    fn get_float(&self, name: &str, opts: GcmdFloatOpts) -> KResult<Option<f64>> {
        let raw = self.params.lock().get(name).cloned();
        let Some(raw) = raw else {
            return match opts.default {
                None => Err(self.error(&format!("missing {name}"))),
                Some(v) => Ok(v),
            };
        };
        let v = pycompat::text::float_from_string(&raw)
            .map_err(|_| self.error(&format!("unable to parse {raw}")))?;
        if let Some(mn) = opts.minval {
            if v < mn {
                return Err(self.error(&format!(
                    "{name} must have minimum of {}",
                    pycompat::text::float_str(mn)
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(self.error(&format!(
                    "{name} must have maximum of {}",
                    pycompat::text::float_str(mx)
                )));
            }
        }
        if let Some(a) = opts.above {
            if v <= a {
                return Err(self.error(&format!(
                    "{name} must be above {}",
                    pycompat::text::float_str(a)
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(self.error(&format!(
                    "{name} must be below {}",
                    pycompat::text::float_str(b)
                )));
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
        self.params.lock().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        let origline = self.commandline.clone();
        let mut start = self.command.len().min(origline.len());
        if origline.get(start..).is_some_and(|s| s.starts_with(' ')) {
            start += 1;
        }
        origline.get(start..).unwrap_or("").to_string()
    }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool {
        false
    }
}

#[derive(Clone, Default)]
struct FakeGcode {
    inner: Arc<Mutex<GcodeInner>>,
}

#[derive(Default)]
struct GcodeInner {
    mux_regs: Vec<(String, String, Option<String>, Option<String>)>,
    handlers: IndexMap<(String, String, String), Box<dyn FnMut(&GCmd) -> KResult<()> + Send>>,
}

impl FakeGcode {
    fn mux_regs_value(&self) -> Value {
        Value::List(
            self.inner
                .lock()
                .mux_regs
                .iter()
                .map(|(cmd, key, value, desc)| {
                    Value::Tuple(vec![
                        Value::Str(cmd.clone()),
                        Value::Str(key.clone()),
                        match value {
                            Some(v) => Value::Str(v.clone()),
                            None => Value::Null,
                        },
                        match desc {
                            Some(d) => Value::Str(d.clone()),
                            None => Value::Null,
                        },
                    ])
                })
                .collect(),
        )
    }
    fn call_mux(&self, fan: &str, gcmd: &GCmd) -> KResult<()> {
        let key = (
            "SET_FAN_SPEED".to_string(),
            "FAN".to_string(),
            fan.to_string(),
        );
        let mut inner = self.inner.lock();
        match inner.handlers.get_mut(&key) {
            Some(cb) => cb(gcmd),
            None => Err(KlippyError::command(format!(
                "unknown mux command {} {}={}",
                key.0, key.1, key.2
            ))),
        }
    }
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        cmd: &str,
        func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
        let _ = (cmd, func);
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: klippy_api::gcode::GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        let mut inner = self.inner.lock();
        inner.mux_regs.push((
            cmd.to_string(),
            key.to_string(),
            value.map(|v| v.to_string()),
            desc.map(|d| d.to_string()),
        ));
        inner.handlers.insert(
            (
                cmd.to_string(),
                key.to_string(),
                value.unwrap_or("").to_string(),
            ),
            func,
        );
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(&self, _cb: klippy_api::gcode::OutputCallback) {}
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
        panic!("mutex not faked")
    }
    fn create_gcode_command(
        &self,
        command: &str,
        commandline: &str,
        params: &IndexMap<String, String>,
    ) -> GCmd {
        let pairs: Vec<(&str, &str)> = params
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        Arc::new(FakeGcodeCmd::new(command, commandline, &pairs))
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _g: Option<&GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

// ---------------------------------------------------------------------------
// Helpers.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("fan_generic fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping fan_generic oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
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
        unit: "fan_generic".to_string(),
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

/// Compares a Python call that returns `None` (or raises) with a Rust
/// unit-returning call.
fn check_unit(label: &str, expected: pyoracle::OracleResult, actual: &KResult<()>) {
    match (expected, actual) {
        (Ok(v), Ok(())) => check(label, Ok(v), Value::Null),
        (Err(e), Err(k)) => {
            assert_eq!(e.class_name, k.class_name(), "{label}: error class");
            assert_eq!(e.message, k.message(), "{label}: error message");
        }
        (Err(e), Ok(())) => panic!(
            "{label}: rust returned Ok but oracle raised {}: {}",
            e.class_name, e.message
        ),
        (Ok(v), Err(k)) => panic!(
            "{label}: oracle returned {} but rust raised {}: {}",
            v.render(),
            k.class_name(),
            k.message()
        ),
    }
}

/// Compares a Python call that must raise with a Rust call that must fail.
fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {} but rust call must fail", v.render()),
        Err(e) => e,
    };
    assert_eq!(actual.class_name(), oracle_exc.class_name.as_str(), "{label}: class");
    assert_eq!(actual.message(), oracle_exc.message.as_str(), "{label}: message");
}

fn py_config_expr(vals: &[(&str, &str)]) -> String {
    let mut s = "{".to_string();
    for (i, (k, v)) in vals.iter().enumerate() {
        if i > 0 {
            s.push_str(", ");
        }
        s.push_str(&format!("'{k}': '{v}'"));
    }
    s.push('}');
    s
}

fn status_value(status: &PyValue) -> Value {
    match status {
        PyValue::Dict(m) => {
            let get = |k: &str| -> Value {
                match m.get(&PyValue::Str(k.to_string())) {
                    Some(PyValue::Float(f)) => Value::float(*f),
                    Some(PyValue::None) => Value::Null,
                    other => Value::str(&format!("{:?}", other)),
                }
            };
            Value::Tuple(vec![get("speed"), get("rpm")])
        }
        _ => Value::Null,
    }
}

fn calls_value(calls: &[(f64, f64)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(t, v)| Value::Tuple(vec![Value::float(*t), Value::float(*v)]))
            .collect(),
    )
}

fn values_only(calls: &[(f64, f64)]) -> Value {
    Value::List(calls.iter().map(|(_, v)| Value::float(*v)).collect())
}

fn digital_value(calls: &[(f64, bool)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(t, v)| Value::Tuple(vec![Value::float(*t), Value::Bool(*v)]))
            .collect(),
    )
}

fn digital_values_only(calls: &[(f64, bool)]) -> Value {
    Value::List(calls.iter().map(|(_, v)| Value::Bool(*v)).collect())
}

fn activities_value(calls: &[(f64, bool)]) -> Value {
    digital_value(calls)
}

// ---------------------------------------------------------------------------
// Harness.
// ---------------------------------------------------------------------------

struct Harness {
    pins: FakePins,
    mq: FakeMq,
    toolhead: FakeToolHead,
    gcode: FakeGcode,
    fg: Arc<PrinterFanGeneric>,
}

fn harness_with(name: &str, vals: &[(&str, &str)]) -> Harness {
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let mcu = Arc::new(FakeMcu::new());
    let mcu_ref: Mcu = mcu.clone();
    let pins = FakePins::new(mcu_ref.clone());
    let mq = FakeMq::default();
    let toolhead = FakeToolHead::default();
    let gcode = FakeGcode::default();
    printer.add_object("pins", Arc::new(pins.clone())).unwrap();
    printer
        .add_object("motion_queuing", Arc::new(mq.clone()))
        .unwrap();
    printer
        .add_object("toolhead", Arc::new(toolhead.clone()))
        .unwrap();
    printer.add_object("gcode", Arc::new(gcode.clone())).unwrap();
    printer
        .add_object("gcode_macro", Arc::new(FakeGCodeMacro))
        .unwrap();
    printer
        .add_object("display_template", Arc::new(FakeEmptyDisplayTemplates))
        .unwrap();
    let config: Config = Arc::new(FakeConfig::new(printer_arc, name, vals));
    let fg = PrinterFanGeneric::new(&config).expect("rust PrinterFanGeneric::new");
    printer.send("klippy:connect", &[]);
    Harness {
        pins,
        mq,
        toolhead,
        gcode,
        fg,
    }
}

fn harness(vals: &[(&str, &str)]) -> Harness {
    harness_with("fan_generic myfan", vals)
}

fn py_setup(py: &mut pyoracle::Oracle, vals: &[(&str, &str)]) {
    let expr = py_config_expr(vals);
    py.exec(&format!("pr, cfg, fg = setup({expr})"))
        .expect("py setup");
}

fn gcmd(params: &[(&str, &str)], commandline: &str) -> GCmd {
    Arc::new(FakeGcodeCmd::new("SET_FAN_SPEED", commandline, params))
}

// ---------------------------------------------------------------------------
// Log capture (lines kept per test thread; the Rust sink is process wide).
// ---------------------------------------------------------------------------

type CapturedLog = (String, String, Option<String>);

thread_local! {
    static LOG_CAPTURE: std::cell::RefCell<Vec<CapturedLog>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            LOG_CAPTURE.with(|logs| {
                logs.borrow_mut().push((
                    rec.level.name().to_string(),
                    rec.message.to_string(),
                    rec.exc_text.map(|s| s.to_string()),
                ));
            });
        }));
    });
}

fn log_capture() {
    install_log_sink();
    LOG_CAPTURE.with(|logs| logs.borrow_mut().clear());
}

fn captured_logs() -> Vec<CapturedLog> {
    LOG_CAPTURE.with(|logs| logs.borrow().clone())
}

fn logs_value(captured: &[CapturedLog]) -> Value {
    Value::List(
        captured
            .iter()
            .map(|(level, message, _)| {
                Value::Tuple(vec![Value::Str(level.clone()), Value::Str(message.clone())])
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_registers_load_config_prefix() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.fan_generic"), "entry points of extras.fan_generic");
}

#[test]
fn construction_and_registration_matches_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);
    let h = harness(&vals);

    check(
        "fan_name",
        py.eval("fg.fan_name"),
        Value::Str(h.fg.fan_name().to_string()),
    );
    check(
        "mux registration",
        py.eval("pr.gcode.mux_calls"),
        h.gcode.mux_regs_value(),
    );
    check(
        "help text",
        py.eval("fan_generic.PrinterFanGeneric.cmd_SET_FAN_SPEED_help"),
        Value::Str(CMD_SET_FAN_SPEED_HELP.to_string()),
    );
    // `fan.Fan(config, default_shutdown_speed=0.)`: the pin setup calls show
    // the shutdown start value (0.) and the cycle time / hardware_pwm reads.
    check(
        "pwm setup defaults",
        py.eval("list(pwm_setup)"),
        h.pins.pwm.setup_value(),
    );
    check(
        "digital setup defaults",
        py.eval("list(digital_setup)"),
        h.pins.digital.setup_value(),
    );
    check(
        "initial status",
        py.eval("(fg.get_status(1.0)['speed'], fg.get_status(1.0)['rpm'])"),
        status_value(&h.fg.get_status(1.0)),
    );
}

#[test]
fn construction_config_variants_match_python() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        (
            "all-options",
            vec![
                ("pin", "pa0"),
                ("enable_pin", "pa1"),
                ("cycle_time", "0.025"),
                ("hardware_pwm", "true"),
                ("max_power", "0.6"),
                ("shutdown_speed", "0.9"),
                ("kick_start_time", "0.2"),
                ("off_below", "0.1"),
            ],
        ),
        ("shutdown-default", vec![("pin", "pa0")]),
        (
            "shutdown-above-max-power",
            vec![("pin", "pa0"), ("max_power", "0.5"), ("shutdown_speed", "0.75")],
        ),
        (
            "hardware-pwm-off",
            vec![("pin", "pa0"), ("hardware_pwm", "no"), ("cycle_time", "0.5")],
        ),
        (
            "enable-pin-only",
            vec![("pin", "pa0"), ("enable_pin", "!pa1")],
        ),
    ];
    for (label, vals) in cases {
        py_setup(&mut py, &vals);
        let h = harness(&vals);
        check(
            &format!("{label}: pwm setup"),
            py.eval("list(pwm_setup)"),
            h.pins.pwm.setup_value(),
        );
        check(
            &format!("{label}: digital setup"),
            py.eval("list(digital_setup)"),
            h.pins.digital.setup_value(),
        );
        check(
            &format!("{label}: fan name"),
            py.eval("fg.fan_name"),
            Value::Str(h.fg.fan_name().to_string()),
        );
    }
}

#[test]
fn cmd_speed_forwarding_matches_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);
    let h = harness(&vals);

    // (params, commandline, lookahead fire time, motion-queue flush time).
    let steps: Vec<(Vec<(&str, &str)>, &str, f64, f64)> = vec![
        (
            vec![("FAN", "myfan"), ("SPEED", "0.5")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.5",
            50.0,
            60.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "0")],
            "SET_FAN_SPEED FAN=myfan SPEED=0",
            61.0,
            70.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "0.7")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.7",
            71.0,
            80.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "1")],
            "SET_FAN_SPEED FAN=myfan SPEED=1",
            81.0,
            90.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "0.25")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.25",
            95.0,
            91.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "2.5")],
            "SET_FAN_SPEED FAN=myfan SPEED=2.5",
            96.0,
            105.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "0.001")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.001",
            106.0,
            115.0,
        ),
    ];
    for (i, (params, commandline, fire, flush)) in steps.iter().enumerate() {
        let expr = py_config_expr(params);
        py.exec(&format!(
            "res = call_mux(pr, {expr}, {commandline:?}); pr.toolhead.fire({fire:?}); pr.mq.flush({flush:?})"
        ))
        .expect("py command step");
        let rust_res = h.gcode.call_mux("myfan", &gcmd(params, commandline));
        h.toolhead.fire(*fire);
        h.mq.flush(*flush);
        check_unit(&format!("step {i} result"), py.eval("res"), &rust_res);
        check(
            &format!("step {i}: pwm calls"),
            py.eval("list(pwm_calls)"),
            calls_value(&h.pins.pwm.pwm_calls()),
        );
        check(
            &format!("step {i}: digital calls"),
            py.eval("list(digital_calls)"),
            digital_value(&h.pins.digital.digital_calls()),
        );
        check(
            &format!("step {i}: movequeue activity"),
            py.eval("list(pr.mq.activities)"),
            activities_value(&h.mq.activities()),
        );
        check(
            &format!("step {i}: status"),
            py.eval("(fg.get_status(1.0)['speed'], fg.get_status(1.0)['rpm'])"),
            status_value(&h.fg.get_status(1.0)),
        );
    }
}

#[test]
fn cmd_speed_enable_pin_matches_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0"), ("enable_pin", "pa1"), ("kick_start_time", "0")];
    py_setup(&mut py, &vals);
    let h = harness(&vals);
    let steps: Vec<(Vec<(&str, &str)>, &str, f64, f64)> = vec![
        (
            vec![("FAN", "myfan"), ("SPEED", "0.5")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.5",
            20.0,
            25.0,
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "0")],
            "SET_FAN_SPEED FAN=myfan SPEED=0",
            26.0,
            30.0,
        ),
    ];
    for (i, (params, commandline, fire, flush)) in steps.iter().enumerate() {
        let expr = py_config_expr(params);
        py.exec(&format!(
            "call_mux(pr, {expr}, {commandline:?}); pr.toolhead.fire({fire:?}); pr.mq.flush({flush:?})"
        ))
        .expect("py enable step");
        h.gcode
            .call_mux("myfan", &gcmd(params, commandline))
            .expect("rust mux command");
        h.toolhead.fire(*fire);
        h.mq.flush(*flush);
        check(
            &format!("step {i}: pwm calls"),
            py.eval("list(pwm_calls)"),
            calls_value(&h.pins.pwm.pwm_calls()),
        );
        check(
            &format!("step {i}: digital calls"),
            py.eval("list(digital_calls)"),
            digital_value(&h.pins.digital.digital_calls()),
        );
    }
}

#[test]
fn cmd_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);
    let h = harness(&vals);

    let cases: Vec<(Vec<(&str, &str)>, &str)> = vec![
        (vec![("FAN", "myfan")], "SET_FAN_SPEED FAN=myfan"),
        (
            vec![("FAN", "myfan"), ("SPEED", "0.5"), ("TEMPLATE", "tmpl")],
            "SET_FAN_SPEED FAN=myfan SPEED=0.5 TEMPLATE=tmpl",
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "-0.1")],
            "SET_FAN_SPEED FAN=myfan SPEED=-0.1",
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "abc")],
            "SET_FAN_SPEED FAN=myfan SPEED=abc",
        ),
        (
            vec![("FAN", "myfan"), ("SPEED", "")],
            "SET_FAN_SPEED FAN=myfan SPEED=",
        ),
    ];
    for (i, (params, commandline)) in cases.iter().enumerate() {
        let expr = py_config_expr(params);
        py.exec(&format!(
            "gcmd = make_gcmd(pr, {expr}, {commandline:?})"
        ))
        .expect("py gcmd");
        let py_res = py.eval("fg.cmd_SET_FAN_SPEED(gcmd)");
        let rust_res = h.fg.cmd_set_fan_speed(&gcmd(params, commandline));
        let err = rust_res.expect_err("rust must fail");
        check_err(&format!("error case {i}"), py_res, err);
        // No pin side effect on any error path.
        check(
            &format!("error case {i}: no pwm calls"),
            py.eval("list(pwm_calls)"),
            calls_value(&h.pins.pwm.pwm_calls()),
        );
    }
}

#[test]
fn cmd_template_path_matches_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);

    // Unknown display template: the real evaluator raises `gcmd.error`.
    let unknown = (
        vec![("FAN", "myfan"), ("TEMPLATE", "tmpl")],
        "SET_FAN_SPEED FAN=myfan TEMPLATE=tmpl",
    );
    let expr = py_config_expr(&unknown.0);
    py.exec(&format!("gcmd = make_gcmd(pr, {expr}, {:?})", unknown.1))
        .expect("py gcmd");
    let py_res = py.eval("fg.cmd_SET_FAN_SPEED(gcmd)");
    let h = harness(&vals);
    let rust_res = h.fg.cmd_set_fan_speed(&gcmd(&unknown.0, unknown.1));
    check_err("unknown template", py_res, rust_res.expect_err("must fail"));

    // Empty template: `set_template` finds no template, activation is a
    // no-op and the command succeeds without touching the fan.
    let empty = (
        vec![("FAN", "myfan"), ("TEMPLATE", "")],
        "SET_FAN_SPEED FAN=myfan TEMPLATE=",
    );
    let expr = py_config_expr(&empty.0);
    py.exec(&format!(
        "gcmd = make_gcmd(pr, {expr}, {:?}); res = fg.cmd_SET_FAN_SPEED(gcmd)",
        empty.1
    ))
    .expect("py empty template");
    let rust_res = h.fg.cmd_set_fan_speed(&gcmd(&empty.0, empty.1));
    check_unit("empty template", py.eval("res"), &rust_res);
    check(
        "empty template: pwm calls",
        py.eval("list(pwm_calls)"),
        calls_value(&h.pins.pwm.pwm_calls()),
    );
    check(
        "empty template: status",
        py.eval("(fg.get_status(1.0)['speed'], fg.get_status(1.0)['rpm'])"),
        status_value(&h.fg.get_status(1.0)),
    );
}

#[test]
fn template_update_matches_python() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);
    let h = harness(&vals);

    let texts: Vec<(&str, &str)> = vec![
        ("valid", "0.5"),
        ("valid", "0"),
        ("valid", "1"),
        ("valid", " 0.25 "),
        ("valid", "-0.0"),
        ("valid", "1e-2"),
        ("valid", "1_0.5"),
        ("valid", "inf"),
        ("valid", "-inf"),
        ("valid", "1E+3"),
        ("valid", "nan"),
        ("invalid", "abc"),
        ("invalid", "0x1"),
        ("invalid", ""),
        ("invalid", "  "),
        ("invalid", "--1"),
        ("invalid", "1.0.0"),
        ("invalid", "nanx"),
        ("valid", "0.75"),
        ("invalid", "e"),
    ];
    for (kind, text) in texts {
        py.exec("_log_records.clear()").expect("py clear logs");
        py.exec(&format!("res = fg._template_update({text:?})"))
            .expect("py template update");
        log_capture();
        let rust_res = h.fg.template_update(text);
        let captured = captured_logs();
        check_unit(&format!("{kind} {text:?} result"), py.eval("res"), &rust_res);
        check(
            &format!("{kind} {text:?} logs"),
            py.eval("[(r[0], r[1]) for r in _log_records]"),
            logs_value(&captured),
        );
        check(
            &format!("{kind} {text:?} log exc_info"),
            py.eval("[r[2] for r in _log_records]"),
            Value::List(
                captured
                    .iter()
                    .map(|(_, _, exc)| Value::Bool(exc.is_some()))
                    .collect(),
            ),
        );
        check(
            &format!("{kind} {text:?} pwm values"),
            py.eval("pwm_values()"),
            values_only(&h.pins.pwm.pwm_calls()),
        );
        check(
            &format!("{kind} {text:?} digital values"),
            py.eval("digital_values()"),
            digital_values_only(&h.pins.digital.digital_calls()),
        );
        check(
            &format!("{kind} {text:?} status"),
            py.eval("(fg.get_status(1.0)['speed'], fg.get_status(1.0)['rpm'])"),
            status_value(&h.fg.get_status(1.0)),
        );
    }

    // The traceback carried by the error record names the ValueError the
    // Python `float()` raised.
    log_capture();
    h.fg.template_update("abc").expect("rust template update");
    let captured = captured_logs();
    assert_eq!(captured.len(), 1, "one log record per invalid text");
    let exc_text = captured[0].2.as_ref().expect("exception text");
    match py.eval("float('abc')") {
        Err(e) => assert!(
            exc_text.contains(&format!("{}: {}", e.class_name, e.message)),
            "traceback text {exc_text:?} vs oracle {}: {}",
            e.class_name,
            e.message
        ),
        Ok(v) => panic!("oracle float('abc') did not raise: {}", v.render()),
    }
}

#[test]
fn fan_name_section_parsing_matches_python() {
    let cases = [
        "fan_generic myfan",
        "fan_generic a b c",
        "fan_generic\tmulti\tspace",
        "fan_generic solo",
        "fan_generic  leading  spaces",
    ];
    for (i, name) in cases.iter().enumerate() {
        let Some(mut py) = oracle() else { return };
        let vals = [("pin", "pa0")];
        let expr = py_config_expr(&vals);
        py.exec(&format!(
            "pr, cfg, fg = setup({expr}, name={name:?})"
        ))
        .expect("py setup named");
        let h = harness_with(name, &vals);
        check(
            &format!("name case {i}"),
            py.eval("fg.fan_name"),
            Value::Str(h.fg.fan_name().to_string()),
        );
        check(
            &format!("name case {i} mux"),
            py.eval("pr.gcode.mux_calls"),
            h.gcode.mux_regs_value(),
        );
    }

    // A section name with no tokens raises the Python IndexError.
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    let expr = py_config_expr(&vals);
    let expected = py.eval(&format!(
        "fan_generic.PrinterFanGeneric(FakeConfig(FakePrinter(), '', {expr}))"
    ));
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let mcu: Mcu = Arc::new(FakeMcu::new());
    let pins = FakePins::new(mcu.clone());
    printer.add_object("pins", Arc::new(pins)).unwrap();
    printer
        .add_object("motion_queuing", Arc::new(FakeMq::default()))
        .unwrap();
    printer
        .add_object("toolhead", Arc::new(FakeToolHead::default()))
        .unwrap();
    printer
        .add_object("gcode", Arc::new(FakeGcode::default()))
        .unwrap();
    printer
        .add_object("gcode_macro", Arc::new(FakeGCodeMacro))
        .unwrap();
    printer
        .add_object("display_template", Arc::new(FakeEmptyDisplayTemplates))
        .unwrap();
    let config: Config = Arc::new(FakeConfig::new(printer_arc, "", &vals));
    let err = PrinterFanGeneric::new(&config).err().expect("rust must fail");
    check_err("empty name", expected, err);
}

#[test]
fn load_config_prefix_builds_object() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_setup(&mut py, &vals);

    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let mcu: Mcu = Arc::new(FakeMcu::new());
    let pins = FakePins::new(mcu.clone());
    let gcode = FakeGcode::default();
    printer.add_object("pins", Arc::new(pins.clone())).unwrap();
    printer
        .add_object("motion_queuing", Arc::new(FakeMq::default()))
        .unwrap();
    printer
        .add_object("toolhead", Arc::new(FakeToolHead::default()))
        .unwrap();
    printer.add_object("gcode", Arc::new(gcode.clone())).unwrap();
    printer
        .add_object("gcode_macro", Arc::new(FakeGCodeMacro))
        .unwrap();
    printer
        .add_object("display_template", Arc::new(FakeEmptyDisplayTemplates))
        .unwrap();
    let config: Config = Arc::new(FakeConfig::new(printer_arc, "fan_generic myfan", &vals));
    let obj = load_config_prefix(&config)
        .expect("load_config_prefix")
        .expect("object");
    // The factory builds the same fan (pin setup) and registers the same mux.
    check(
        "factory pwm setup",
        py.eval("list(pwm_setup)"),
        pins.pwm.setup_value(),
    );
    check(
        "factory mux registration",
        py.eval("pr.gcode.mux_calls"),
        gcode.mux_regs_value(),
    );
    let status = obj
        .get_status(1.0).unwrap()
        .expect("PrinterFanGeneric has get_status");
    check(
        "factory status",
        py.eval("(fg.get_status(1.0)['speed'], fg.get_status(1.0)['rpm'])"),
        status_value(&status),
    );
}
