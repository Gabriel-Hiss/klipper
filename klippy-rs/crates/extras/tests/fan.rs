//! Parity tests for `extras::fan` (port of `klippy/extras/fan.py`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): both sides are driven with
//! minimal Python fakes for the printer, config, pins and MCU, given
//! identical constructor arguments, and the following are compared
//! exactly (floats bit-for-bit via the recording value format, errors
//! by class and message):
//!
//! - construction defaults and every pin setup call (`max_duration`,
//!   `cycle_time`, `hardware_pwm`, start and shutdown values, enable pin),
//! - `_apply_speed` kick / normal / discard sequences and PWM calls,
//! - async `set_speed` through the kick window,
//! - `off_below` and `max_power` scaling and clamping,
//! - enable pin toggling with and without kick,
//! - config validation errors for every option the fan and tachometer read,
//! - the `M106` / `M107` handlers registered on gcode, run through toolhead
//!   lookahead and motion-queue flush (movequeue activity compared),
//!   including `M106 S-1`,
//! - `set_speed_from_command` before `klippy:connect` (AttributeError),
//! - tachometer RPM after `counter_state` serial responses,
//! - `gcode:request_restart` turning the fan off.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

#![allow(clippy::type_complexity)]
use extras::fan::{load_config, module_factory, Fan, PrinterFan};
use extras::output_pin::GcrqCallbackReturn;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GcmdFloatOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AsyncResponseWrapper, AsyncResponseWrapperApi, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts,
    McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, PwmPinApi, SerialResponseCallback,
    TriggerDispatch, Mcu,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{ExtraAxis, LookaheadCallback, MotionQueuingApi, PrinterExtruder, ToolHeadApi};
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

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
// fakes below: same MCU constants, same error messages, same call logs.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import extras.fan as _fan
import configparser

class Error(Exception):
    pass
error = Error

class CommandError(Exception):
    pass

FREQ = 12000000.0

class _Sentinel: pass
_sentinel = _Sentinel()

class FakeReactor:
    def monotonic(self):
        return 1000.0

class FakeMCU:
    name = 'mcu'
    def __init__(self):
        self.oid_count = 0
        self.config_callbacks = []
        self.config_cmds = []
        self.serial_cb = None
        self.serial = None
        self.min_schedule = 0.1
    def create_oid(self):
        oid = self.oid_count
        self.oid_count += 1
        return oid
    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init, on_restart))
    def get_query_slot(self, oid):
        return oid + 100
    def seconds_to_clock(self, time):
        return int(time * FREQ)
    def clock32_to_clock64(self, clock32):
        return clock32
    def clock_to_print_time(self, clock):
        return float(clock) / FREQ
    def min_schedule_time(self):
        return self.min_schedule
    def estimated_print_time(self, eventtime):
        return eventtime
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_cb = cb
        self.serial = (msg, oid)
        return object()

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
        return {'chip': self.mcu, 'chip_name': chip_name, 'pin': pin, 'invert': invert, 'pullup': pullup}
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

class FakeGcmd:
    def __init__(self, params):
        self._params = params
    def get_float(self, name, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        value = self._params.get(name)
        if value is None:
            if default is _sentinel:
                raise CommandError("Error on '': missing %s" % (name,))
            return default
        try:
            value = float(value)
        except:
            raise CommandError("Error on '': unable to parse %s" % (value,))
        if minval is not None and value < minval:
            raise CommandError("Error on '': %s must have minimum of %s" % (name, minval))
        if maxval is not None and value > maxval:
            raise CommandError("Error on '': %s must have maximum of %s" % (name, maxval))
        if above is not None and value <= above:
            raise CommandError("Error on '': %s must be above %s" % (name, above))
        if below is not None and value >= below:
            raise CommandError("Error on '': %s must be below %s" % (name, below))
        return value

class FakeGcode:
    def __init__(self):
        self.handlers = {}
    def register_command(self, cmd, func, desc=None, when_not_ready=False):
        self.handlers[cmd] = func

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
            raise error("Option '%s' in section '%s' must be specified" % (option, self.name))
        return default
    def getint(self, option, default=_sentinel, minval=None, maxval=None):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified" % (option, self.name))
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
    def getfloat(self, option, default=_sentinel, minval=None, maxval=None, above=None, below=None):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified" % (option, self.name))
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
                raise error("Option '%s' in section '%s' must be specified" % (option, self.name))
            return default
        v = self.values[option].strip().lower()
        if v in ('true', 'yes', 'on', '1'):
            return True
        if v in ('false', 'no', 'off', '0'):
            return False
        raise error("Option '%s' is not a boolean" % (option,))

pwm_calls = []
digital_calls = []
pwm_setup = []
digital_setup = []
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
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
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
    fn register_event_handler(
        &self,
        event: &str,
        callback: klippy_api::printer::EventCallback,
    ) {
        self.inner
            .lock()
            .handlers
            .entry(event.to_string())
            .or_default()
            .push(callback);
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
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
    fn get(
        &self,
        option: &str,
        opts: GetStrOpts,
    ) -> KResult<Option<String>> {
        match self.get_raw(option) {
            Some(v) => Ok(Some(v)),
            None => match opts.default {
                None => Err(missing(&self.name, option)),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getint(
        &self,
        option: &str,
        opts: GetIntOpts,
    ) -> KResult<Option<i64>> {
        let raw = match self.get_raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(missing(&self.name, option)),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = parse_int(&raw).ok_or_else(|| {
            KlippyError::config(format!(
                "Option '{}' is not an integer",
                option
            ))
        })?;
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
    fn getfloat(
        &self,
        option: &str,
        opts: GetFloatOpts,
    ) -> KResult<Option<f64>> {
        let raw = match self.get_raw(option) {
            Some(v) => v,
            None => {
                return match opts.default {
                    None => Err(missing(&self.name, option)),
                    Some(v) => Ok(v),
                }
            }
        };
        let v = parse_float(&raw).ok_or_else(|| {
            KlippyError::config(format!(
                "Option '{}' is not a float",
                option
            ))
        })?;
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
    fn getboolean(
        &self,
        option: &str,
        opts: GetBoolOpts,
    ) -> KResult<Option<bool>> {
        match self.get_raw(option) {
            Some(v) => parse_bool(&v).map(Some).ok_or_else(|| {
                KlippyError::config(format!(
                    "Option '{}' is not a boolean",
                    option
                ))
            }),
            None => match opts.default {
                None => Err(missing(&self.name, option)),
                Some(v) => Ok(v),
            },
        }
    }
    fn getchoice(
        &self,
        _o: &str,
        _c: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::config("getchoice not faked"))
    }
    fn getlists(
        &self,
        _o: &str,
        _opts: GetListsOpts,
    ) -> KResult<PyValue> {
        Err(KlippyError::config("getlists not faked"))
    }
    fn getlist(
        &self,
        _o: &str,
        _opts: GetListOpts,
    ) -> KResult<Option<Vec<String>>> {
        Err(KlippyError::config("getlist not faked"))
    }
    fn getintlist(
        &self,
        _o: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        Err(KlippyError::config("getintlist not faked"))
    }
    fn getfloatlist(
        &self,
        _o: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        Err(KlippyError::config("getfloatlist not faked"))
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection not faked")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(
        &self,
        _p: &str,
    ) -> Vec<Config> {
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
    config_cbs: Vec<ConfigCallback>,
    config_cmds: Vec<(String, bool, bool)>,
    serial_cbs: Vec<(String, Arc<dyn Fn(PyValue) + Send + Sync>)>,
}

impl FakeMcu {
    fn new() -> Self {
        Self::default()
    }
    fn config_cmds(&self) -> Vec<(String, bool, bool)> {
        self.inner.lock().config_cmds.clone()
    }
    fn fire_serial(&self, params: PyValue) {
        let cbs: Vec<_> = self
            .inner
            .lock()
            .serial_cbs
            .iter()
            .map(|(_, cb)| cb.clone())
            .collect();
        for cb in cbs {
            cb(params.clone());
        }
    }
    fn fire_config(&self) {
        let mut cbs: Vec<_> = {
            let mut inner = self.inner.lock();
            std::mem::take(&mut inner.config_cbs)
        };
        for cb in cbs.iter_mut() {
            cb().unwrap();
        }
        if !cbs.is_empty() {
            let mut inner = self.inner.lock();
            inner.config_cbs.extend(cbs);
        }
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
    fn send_wait_ack(
        &self,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::SendOpts,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        1
    }
}

struct FakeQuery;
impl klippy_api::mcu::CommandQueryWrapperApi for FakeQuery {
    fn send(
        &self,
        _d: Vec<PyValue>,
        _o: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
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
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.inner.lock().config_cbs.push(cb);
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
    fn lookup_command(
        &self,
        _m: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
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
        cb: SerialResponseCallback,
        msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        self.inner
            .lock()
            .serial_cbs
            .push((msg.to_string(), cb));
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
    fn calibrate_clock(
        &self,
        print_time: f64,
        _eventtime: f64,
    ) -> KResult<(f64, f64)> {
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
        FakePwm { inner: Arc::new(Mutex::new(PwmInner::default())), mcu: Some(mcu) }
    }
    fn pwm_calls(&self) -> Vec<(f64, f64)> {
        self.inner.lock().pwm_calls.clone()
    }
    fn setup_value(&self) -> Value {
        let inner = self.inner.lock();
        let mut out = Vec::new();
        for d in &inner.max_duration {
            out.push(Value::Tuple(vec![Value::str("max_duration"), Value::float(*d)]));
        }
        for (t, hw) in &inner.cycle {
            out.push(Value::Tuple(vec![
                Value::str("cycle_time"),
                Value::float(*t),
                Value::Bool(hw.expect("fan passes hardware_pwm")),
            ]));
        }
        for (s, sh) in &inner.start {
            out.push(Value::Tuple(vec![Value::str("start_value"), Value::float(*s), Value::float(*sh)]));
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
        FakeDigital { inner: Arc::new(Mutex::new(DigitalInner::default())), mcu: Some(mcu) }
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
    fn parse_pin(
        &self,
        pin_desc: &str,
        _opts: ParsePinOpts,
    ) -> KResult<PinParams> {
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
    fn lookup_pin(
        &self,
        pin_desc: &str,
        opts: LookupPinOpts,
    ) -> KResult<PinParams> {
        let mut p = self.parse_pin(pin_desc, ParsePinOpts {
            can_invert: opts.can_invert,
            can_pullup: opts.can_pullup,
        })?;
        p.share_type = opts.share_type;
        Ok(p)
    }
    fn setup_pin(&self, pin_type: &str, pin_desc: &str) -> KResult<Pin> {
        match pin_type {
            "pwm" => {
                let _ = self.parse_pin(
                    pin_desc,
                    ParsePinOpts { can_invert: true, can_pullup: false },
                )?;
                Ok(Pin::Pwm(self.pwm.clone()))
            }
            "digital_out" => {
                let _ = self.parse_pin(
                    pin_desc,
                    ParsePinOpts { can_invert: true, can_pullup: false },
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
    fn flush(&self, must: f64) {
        let cbs: Vec<_> = self.inner.lock().flush_cbs.clone();
        for cb in cbs {
            cb(must, 0.0).expect("cb");
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
    fn queue_syncemitter_msg(&self, _s: &klippy_api::toolhead::SyncEmitterHandle, _r: u64, _d: &[u32]) {}
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
        self.inner
            .lock()
            .activities
            .push((mq_time, is_step_gen));
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
    fn fire(&self, pt: f64) {
        let cbs = std::mem::take(&mut *self.inner.lock());
        for mut cb in cbs {
            cb(pt).unwrap();
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
    fn drip_move(
        &self,
        _n: Vec<f64>,
        _s: f64,
        _d: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _n: Vec<f64>, _s: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(
        &self,
        _c: Vec<Option<f64>>,
        _s: f64,
    ) -> KResult<()> {
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
    fn register_lookahead_callback(
        &self,
        callback: LookaheadCallback,
    ) -> KResult<()> {
        self.inner.lock().push(callback);
        Ok(())
    }
    fn check_busy(&self, _e: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _s: f64) {}
    fn set_extruder(
        &self,
        _e: PrinterExtruder,
        _p: f64,
    ) {
    }
    fn get_extruder(&self) -> PrinterExtruder {
        panic!("extruder not faked")
    }
    fn add_extra_axis(
        &self,
        _e: ExtraAxis,
        _p: f64,
    ) -> KResult<()> {
        Ok(())
    }
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

#[derive(Clone, Default)]
struct FakeGcodeCmd {
    params: Arc<Mutex<IndexMap<String, String>>>,
}

impl FakeGcodeCmd {
    fn new(params: &[(&str, &str)]) -> Self {
        let mut m = IndexMap::new();
        for (k, v) in params {
            m.insert(k.to_string(), v.to_string());
        }
        FakeGcodeCmd { params: Arc::new(Mutex::new(m)) }
    }
}

impl GCodeCommandApi for FakeGcodeCmd {
    fn get(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetOpts,
    ) -> KResult<PyValue> {
        let _ = opts;
        let raw = self
            .params
            .lock()
            .get(name)
            .cloned()
            .unwrap_or_default();
        Ok(PyValue::Str(raw))
    }
    fn get_int(
        &self,
        name: &str,
        opts: klippy_api::gcode::GetIntOpts,
    ) -> KResult<Option<i64>> {
        let raw = self.params.lock().get(name).cloned();
        match raw {
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '': missing {}",
                    name
                ))),
                Some(v) => Ok(v),
            },
            Some(s) => Ok(parse_int(&s)),
        }
    }
    fn get_float(
        &self,
        name: &str,
        opts: GcmdFloatOpts,
    ) -> KResult<Option<f64>> {
        let raw = self.params.lock().get(name).cloned();
        match raw {
            None => match opts.default {
                None => Err(KlippyError::command(format!(
                    "Error on '': missing {}",
                    name
                ))),
                Some(v) => Ok(v),
            },
            Some(s) => {
                let v = parse_float(&s).ok_or_else(|| {
                    KlippyError::command(format!(
                        "Error on '': unable to parse {}",
                        s
                    ))
                })?;
                if let Some(mn) = opts.minval {
                    if v < mn {
                        return Err(KlippyError::command(format!(
                            "Error on '': {} must have minimum of {:?}",
                            name, mn
                        )));
                    }
                }
                if let Some(mx) = opts.maxval {
                    if v > mx {
                        return Err(KlippyError::command(format!(
                            "Error on '': {} must have maximum of {:?}",
                            name, mx
                        )));
                    }
                }
                if let Some(a) = opts.above {
                    if v <= a {
                        return Err(KlippyError::command(format!(
                            "Error on '': {} must be above {:?}",
                            name, a
                        )));
                    }
                }
                if let Some(b) = opts.below {
                    if v >= b {
                        return Err(KlippyError::command(format!(
                            "Error on '': {} must be below {:?}",
                            name, b
                        )));
                    }
                }
                Ok(Some(v))
            }
        }
    }
    fn get_command(&self) -> String {
        "M106".to_string()
    }
    fn get_commandline(&self) -> String {
        String::new()
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        self.params.lock().clone()
    }
    fn get_raw_command_parameters(&self) -> String {
        String::new()
    }
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn respond_raw(&self, _m: &str) {}
    fn ack(&self, _m: Option<&str>) -> bool {
        true
    }
}

#[derive(Clone, Default)]
struct FakeGcode {
    inner: Arc<Mutex<GcodeInner>>,
}

#[derive(Default)]
struct GcodeInner {
    handlers: IndexMap<String, Box<dyn FnMut(&klippy_api::gcode::GCmd) -> KResult<()> + Send>>,
}

impl FakeGcode {
    fn commands(&self) -> Vec<String> {
        let mut names: Vec<String> = self.inner.lock().handlers.keys().cloned().collect();
        names.sort();
        names
    }
    fn call(&self, cmd: &str, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let mut inner = self.inner.lock();
        match inner.handlers.get_mut(cmd) {
            Some(cb) => cb(gcmd),
            None => Err(KlippyError::command(format!(
                "unknown command {}",
                cmd
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
        let mut inner = self.inner.lock();
        match func {
            None => Ok(inner.handlers.swap_remove(cmd)),
            Some(f) => {
                if inner.handlers.contains_key(cmd) {
                    return Err(KlippyError::config(format!(
                        "gcode command {} already registered",
                        cmd
                    )));
                }
                inner.handlers.insert(cmd.to_string(), f);
                Ok(None)
            }
        }
    }
    fn register_mux_command(
        &self,
        _c: &str,
        _k: &str,
        _v: Option<&str>,
        _f: klippy_api::gcode::GCodeCommandHandler,
        _d: Option<&str>,
    ) -> KResult<()> {
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn register_output_handler(
        &self,
        _cb: klippy_api::gcode::OutputCallback,
    ) {
    }
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
        _command: &str,
        _commandline: &str,
        params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        let mut m = IndexMap::new();
        for (k, v) in params.iter() {
            m.insert(k.clone(), v.clone());
        }
        Arc::new(FakeGcodeCmd { params: Arc::new(Mutex::new(m)) })
    }
    fn respond_raw(&self, _m: &str) {}
    fn respond_info(&self, _m: &str, _l: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        true
    }
    fn cmd_m112(&self, _g: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _c: &[String], _n: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeGcode; dyn GCodeApi);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("fan fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping fan oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "fan".to_string(),
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

fn check_err(label: &str, expected: pyoracle::OracleResult, actual: KlippyError) {
    let oracle_exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but rust call must fail"),
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

#[allow(clippy::too_many_lines)]
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

fn apply_value(r: &GcrqCallbackReturn) -> Value {
    match r {
        Some((a, t)) => Value::Tuple(vec![Value::str(a), Value::float(*t)]),
        None => Value::Null,
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

struct Harness {
    printer: FakePrinter,
    config: Config,
    mcu: Arc<FakeMcu>,
    pins: FakePins,
    mq: FakeMq,
    toolhead: FakeToolHead,
    gcode: FakeGcode,
}

fn harness(config_vals: &[(&str, &str)]) -> Harness {
    let printer = FakePrinter::new();
    let printer_arc: Printer = Arc::new(printer.clone());
    let mcu = Arc::new(FakeMcu::new());
    let mcu_ref: Mcu = mcu.clone();
    let pins = FakePins::new(mcu_ref.clone());
    let mq = FakeMq::default();
    let toolhead = FakeToolHead::default();
    let gcode = FakeGcode::default();
    printer.add_object("pins", Arc::new(pins.clone())).unwrap();
    printer.add_object("motion_queuing", Arc::new(mq.clone())).unwrap();
    printer.add_object("toolhead", Arc::new(toolhead.clone())).unwrap();
    printer.add_object("gcode", Arc::new(gcode.clone())).unwrap();
    let config: Config = Arc::new(FakeConfig::new(printer_arc.clone(), "fan", config_vals));
    Harness { printer, config, mcu, pins, mq, toolhead, gcode }
}

fn py_build(py: &mut pyoracle::Oracle, vals: &[(&str, &str)]) {
    py_build_with(py, vals, "");
}

/// Builds the Python fan; `extra_args` is appended to the `Fan(cfg` call.
fn py_build_with(py: &mut pyoracle::Oracle, vals: &[(&str, &str)], extra_args: &str) {
    let expr = py_config_expr(vals);
    py.exec(&format!(
        "pr = FakePrinter(); pwm_calls.clear(); digital_calls.clear(); pwm_setup.clear(); digital_setup.clear(); cfg = FakeConfig(pr, 'fan', {expr}); fan = _fan.Fan(cfg{extra_args}); pr.send_event('klippy:connect')"
    ))
    .expect("py build fan");
}

fn py_build_printer_fan(py: &mut pyoracle::Oracle, vals: &[(&str, &str)]) {
    let expr = py_config_expr(vals);
    py.exec(&format!(
        "pr = FakePrinter(); pwm_calls.clear(); digital_calls.clear(); pwm_setup.clear(); digital_setup.clear(); cfg = FakeConfig(pr, 'fan', {expr}); pf = _fan.PrinterFan(cfg); pr.send_event('klippy:connect')"
    ))
    .expect("py build printer fan");
}

fn build_fan(h: &Harness) -> Arc<Fan> {
    let fan = Fan::new(&h.config, 0.0).unwrap();
    h.printer.send("klippy:connect", &[]);
    fan
}

fn build_printer_fan(h: &Harness) -> Arc<PrinterFan> {
    let pf = PrinterFan::new(&h.config).unwrap();
    h.printer.send("klippy:connect", &[]);
    pf
}

fn digital_value(calls: &[(f64, bool)]) -> Value {
    Value::List(
        calls
            .iter()
            .map(|(t, v)| Value::Tuple(vec![Value::float(*t), Value::Bool(*v)]))
            .collect(),
    )
}

fn activities_value(calls: &[(f64, bool)]) -> Value {
    digital_value(calls)
}

/// Compares every pin side effect the fan produced against the oracle.
fn check_pins(label: &str, py: &mut pyoracle::Oracle, h: &Harness) {
    check(
        &format!("{label}: pwm calls"),
        py.eval("list(pwm_calls)"),
        calls_value(&h.pins.pwm.pwm_calls()),
    );
    check(
        &format!("{label}: digital calls"),
        py.eval("list(digital_calls)"),
        digital_value(&h.pins.digital.digital_calls()),
    );
}

fn check_setup(label: &str, py: &mut pyoracle::Oracle, h: &Harness) {
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
}

fn check_status(label: &str, py: &mut pyoracle::Oracle, status_expr: &str, actual: &PyValue) {
    check(
        &format!("{label}: status"),
        py.eval(&format!("({status_expr}['speed'], {status_expr}['rpm'])")),
        status_value(actual),
    );
}

fn gcmd(params: &[(&str, &str)]) -> klippy_api::gcode::GCmd {
    Arc::new(FakeGcodeCmd::new(params))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_load_config() {
    let factory = module_factory();
    assert_eq!(factory.entry_points(), pyoracle::entry_points("extras.fan"), "entry points of extras.fan");
}

#[test]
fn construction_defaults_apply_speed_and_status() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let fan = build_fan(&h);

    check_setup("defaults", &mut py, &h);
    check_status("initial", &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
    // Direct apply triggers kick: returns repeat and runs full speed once.
    check(
        "apply-repeat",
        py.eval("fan._apply_speed(100.0, 0.5)"),
        apply_value(&fan.apply_speed(100.0, 0.5).expect("apply_speed")),
    );
    check_pins("after kick", &mut py, &h);
    check(
        "fan-value-after-kick",
        py.eval("fan.last_fan_value"),
        Value::float(fan.last_fan_value()),
    );
    check(
        "req-value-after-kick",
        py.eval("fan.last_req_value"),
        Value::float(fan.last_req_value()),
    );
    // After kick expires the same value applies normally.
    check(
        "apply-normal",
        py.eval("fan._apply_speed(100.2, 0.5)"),
        apply_value(&fan.apply_speed(100.2, 0.5).expect("apply_speed")),
    );
    check_pins("after normal", &mut py, &h);
    // Setting same value again is discarded (no new pwm call).
    check(
        "apply-discard",
        py.eval("fan._apply_speed(101.0, 0.5)"),
        apply_value(&fan.apply_speed(101.0, 0.5).expect("apply_speed")),
    );
    check_pins("after discard", &mut py, &h);
    check_status("after apply", &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));

    // Async path: set_speed repeats through kick expiry (two pwm calls).
    py_build(&mut py, &vals);
    let h2 = harness(&vals);
    let fan2 = build_fan(&h2);
    py.exec("fan.set_speed(0.5, 300.0)").expect("py async set");
    fan2.set_speed(0.5, Some(300.0)).unwrap();
    check_pins("async-repeat", &mut py, &h2);
    check_status("async", &mut py, "fan.get_status(1.0)", &fan2.get_status(1.0));

    // load_config builds the PrinterFan and its status comes through the
    // printer object.
    py_build_printer_fan(&mut py, &vals);
    let h3 = harness(&vals);
    let obj = load_config(&h3.config).unwrap().expect("object");
    let status = obj.get_status(1.0).unwrap().expect("fan object reports status");
    check_status("load_config", &mut py, "pf.get_status(1.0)", &status);
}

#[test]
fn pin_setup_options() {
    let Some(mut py) = oracle() else { return };
    let cases: [(&str, Vec<(&str, &str)>, f64); 5] = [
        (
            "all-options",
            vec![
                ("pin", "pa0"),
                ("enable_pin", "pa1"),
                ("cycle_time", "0.025"),
                ("hardware_pwm", "true"),
                ("max_power", "0.6"),
                ("shutdown_speed", "0.9"),
            ],
            0.0,
        ),
        ("shutdown-default-zero", vec![("pin", "pa0")], 0.0),
        ("shutdown-default-one", vec![("pin", "pa0")], 1.0),
        (
            "shutdown-above-max-power",
            vec![("pin", "pa0"), ("max_power", "0.5"), ("shutdown_speed", "0.75")],
            0.0,
        ),
        (
            "hardware-pwm-off",
            vec![("pin", "pa0"), ("hardware_pwm", "no"), ("cycle_time", "0.5")],
            0.25,
        ),
    ];
    for (label, vals, default_shutdown) in cases {
        py_build_with(&mut py, &vals, &format!(", default_shutdown_speed={default_shutdown:?}"));
        let h = harness(&vals);
        let _fan = Fan::new(&h.config, default_shutdown).unwrap();
        check_setup(label, &mut py, &h);
    }
}

#[test]
fn kick_start_and_off_below() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0"), ("off_below", "0.2"), ("max_power", "0.8"), ("kick_start_time", "0.25")];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let fan = build_fan(&h);

    // (value, print_time): below off_below, kick from zero, small step (no
    // kick), large step (kick), clamp at max_power, drop below off_below.
    let steps: [(f64, f64); 8] = [
        (0.1, 10.0),
        (0.3, 11.0),
        (0.5, 12.0),
        (0.95, 13.0),
        (5.0, 14.0),
        (0.15, 15.0),
        (0.0, 16.0),
        (0.9, 17.0),
    ];
    for (i, (value, print_time)) in steps.iter().enumerate() {
        py.exec(&format!("fan.set_speed({value:?}, {print_time:?})"))
            .expect("py set_speed");
        fan.set_speed(*value, Some(*print_time)).unwrap();
        check_pins(&format!("step {i}"), &mut py, &h);
        check_status(&format!("step {i}"), &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
        check(
            &format!("step {i}: last_fan_value"),
            py.eval("fan.last_fan_value"),
            Value::float(fan.last_fan_value()),
        );
    }
}

#[test]
fn off_below_and_max_power() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("off_below", "0.2"),
        ("max_power", "0.5"),
        ("kick_start_time", "0"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let fan = build_fan(&h);

    for (label, value, print_time) in [
        ("below-off-below", 0.1, 10.0),
        ("scaled", 0.8, 11.0),
        ("clamped", 5.0, 12.0),
    ] {
        py.exec(&format!("fan.set_speed({value:?}, {print_time:?})")).expect("py set_speed");
        fan.set_speed(value, Some(print_time)).unwrap();
        check_pins(label, &mut py, &h);
    }
}

#[test]
fn enable_pin_toggles() {
    let Some(mut py) = oracle() else { return };
    for (label, extra) in [("no-kick", ("kick_start_time", "0")), ("kick", ("kick_start_time", "0.1"))] {
        let vals = [("pin", "pa0"), ("enable_pin", "pa1"), extra];
        py_build(&mut py, &vals);
        let h = harness(&vals);
        let fan = build_fan(&h);
        check_setup(label, &mut py, &h);
        for (i, (value, print_time)) in [(0.5, 20.0), (0.7, 21.0), (0.0, 22.0), (0.0, 23.0), (1.0, 24.0)]
            .into_iter()
            .enumerate()
        {
            py.exec(&format!("fan.set_speed({value:?}, {print_time:?})")).expect("py set_speed");
            fan.set_speed(value, Some(print_time)).unwrap();
            check_pins(&format!("{label} step {i}"), &mut py, &h);
        }
    }
}

#[test]
fn config_validation_errors() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("max-power-high", vec![("pin", "pa0"), ("max_power", "2.0")]),
        ("max-power-zero", vec![("pin", "pa0"), ("max_power", "0")]),
        ("max-power-text", vec![("pin", "pa0"), ("max_power", "abc")]),
        ("kick-negative", vec![("pin", "pa0"), ("kick_start_time", "-1")]),
        ("off-below-high", vec![("pin", "pa0"), ("off_below", "1.5")]),
        ("cycle-time", vec![("pin", "pa0"), ("cycle_time", "0")]),
        ("hardware-pwm", vec![("pin", "pa0"), ("hardware_pwm", "maybe")]),
        ("shutdown-high", vec![("pin", "pa0"), ("shutdown_speed", "1.5")]),
        ("missing-pin", vec![("max_power", "1.0"), ("cycle_time", "0.010")]),
        ("ppr-zero", vec![("pin", "pa0"), ("tachometer_pin", "pa2"), ("tachometer_ppr", "0")]),
        ("ppr-text", vec![("pin", "pa0"), ("tachometer_pin", "pa2"), ("tachometer_ppr", "x")]),
        ("poll-zero", vec![("pin", "pa0"), ("tachometer_pin", "pa2"), ("tachometer_poll_interval", "0")]),
    ];
    for (label, vals) in cases {
        let expr = py_config_expr(&vals);
        let expected = py.eval(&format!("_fan.Fan(FakeConfig(FakePrinter(), 'fan', {expr}))"));
        let h = harness(&vals);
        let err = Fan::new(&h.config, 0.0).err().expect("rust must fail");
        check_err(label, expected, err);
    }
}

#[test]
fn gcode_commands_register_and_run() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_build_printer_fan(&mut py, &vals);
    let h = harness(&vals);
    let pf = build_printer_fan(&h);

    check(
        "registered-commands",
        py.eval("sorted(pr.gcode.handlers.keys())"),
        Value::List(
            h.gcode
                .commands()
                .iter()
                .map(|c| Value::str(c))
                .collect(),
        ),
    );

    // (command, params, lookahead fire time, flush time). The third entry
    // is flushed before its request time, the fourth flushes both.
    let steps: [(&str, Vec<(&str, &str)>, f64, f64); 8] = [
        ("M106", vec![("S", "255")], 50.0, 60.0),
        ("M106", vec![("S", "128")], 61.0, 70.0),
        ("M107", vec![], 71.0, 80.0),
        ("M106", vec![], 81.0, 90.0),
        ("M106", vec![("S", "0")], 91.0, 100.0),
        ("M106", vec![("S", "100")], 110.0, 105.0),
        ("M106", vec![("S", "200")], 111.0, 120.0),
        ("M107", vec![], 121.0, 130.0),
    ];
    for (i, (cmd, params, fire, flush)) in steps.iter().enumerate() {
        let expr = py_config_expr(params);
        py.exec(&format!(
            "pr.gcode.handlers['{cmd}'](FakeGcmd({expr})); pr.toolhead.fire({fire:?}); pr.mq.flush({flush:?})"
        ))
        .expect("py gcode step");
        h.gcode.call(cmd, &gcmd(params)).unwrap();
        h.toolhead.fire(*fire);
        h.mq.flush(*flush);
        check_pins(&format!("step {i} ({cmd})"), &mut py, &h);
        check(
            &format!("step {i}: movequeue activity"),
            py.eval("list(pr.mq.activities)"),
            activities_value(&h.mq.activities()),
        );
        check_status(&format!("step {i}"), &mut py, "pf.get_status(1.0)", &pf.get_status(1.0));
    }
}

#[test]
fn gcode_below_minval_errors() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    py_build_printer_fan(&mut py, &vals);
    let expected = py.eval("pr.gcode.handlers['M106'](FakeGcmd({'S': '-1'}))");
    let h = harness(&vals);
    let _pf = build_printer_fan(&h);
    let err = h
        .gcode
        .call("M106", &gcmd(&[("S", "-1")]))
        .expect_err("S below minval must fail");
    check_err("m106-minval", expected, err);
}

#[test]
fn queueing_before_connect_raises() {
    let Some(mut py) = oracle() else { return };
    let vals = [("pin", "pa0")];
    let expr = py_config_expr(&vals);
    py.exec(&format!(
        "pr = FakePrinter(); cfg = FakeConfig(pr, 'fan', {expr}); fan = _fan.Fan(cfg)"
    ))
    .expect("py build unconnected fan");
    let expected = py.eval("fan.set_speed_from_command(0.5)");
    let h = harness(&vals);
    let fan = Fan::new(&h.config, 0.0).unwrap();
    let err = fan.set_speed_from_command(0.5).expect_err("toolhead is unset");
    check_err("unconnected", expected, err);
}

#[test]
fn tachometer_rpm() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("tachometer_pin", "pa2"),
        ("tachometer_ppr", "2"),
        ("tachometer_poll_interval", "0.0015"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let fan = build_fan(&h);

    // Fire config callbacks to emit config_counter/query_counter commands.
    py.exec("for cb in list(pr.mcu.config_callbacks): cb()").expect("py build config");
    h.mcu.fire_config();
    check(
        "config-cmds",
        py.eval("list(pr.mcu.config_cmds)"),
        Value::List(
            h.mcu
                .config_cmds()
                .iter()
                .map(|(s, a, b)| Value::Tuple(vec![Value::str(s), Value::Bool(*a), Value::Bool(*b)]))
                .collect(),
        ),
    );
    check_status("rpm-zero", &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
    // First sample primes; second computes freq. Clocks in MCU ticks at
    // 12 MHz: count_time 1s later with 60 counts -> freq 60, rpm 60*30/2=900.
    let mk = |next_clock: i64, count: i64, count_clock: i64| -> PyValue {
        let mut m = IndexMap::new();
        m.insert(PyValue::Str("next_clock".to_string()), PyValue::Int(PyInt::from_i64(next_clock)));
        m.insert(PyValue::Str("count".to_string()), PyValue::Int(PyInt::from_i64(count)));
        m.insert(PyValue::Str("count_clock".to_string()), PyValue::Int(PyInt::from_i64(count_clock)));
        PyValue::Dict(m)
    };
    py.exec("pr.mcu.serial_cb({'next_clock': 18000, 'count': 0, 'count_clock': 0})").expect("py serial 1");
    h.mcu.fire_serial(mk(18_000, 0, 0));
    check_status("rpm-after-first", &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
    py.exec("pr.mcu.serial_cb({'next_clock': 12018000, 'count': 60, 'count_clock': 12000000})").expect("py serial 2");
    h.mcu.fire_serial(mk(12_018_000, 60, 12_000_000));
    check_status("rpm-900", &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
}

#[test]
fn tachometer_ppr_default_and_value() {
    let Some(mut py) = oracle() else { return };
    // Default ppr is 2; ppr 4 halves the RPM.
    for (label, extra) in [("ppr-default", None), ("ppr-4", Some(("tachometer_ppr", "4")))] {
        let mut vals = vec![("pin", "pa0"), ("tachometer_pin", "pa2")];
        if let Some(e) = extra {
            vals.push(e);
        }
        py_build(&mut py, &vals);
        let h = harness(&vals);
        let fan = build_fan(&h);
        py.exec("for cb in list(pr.mcu.config_callbacks): cb()").expect("py build config");
        h.mcu.fire_config();
        let mk = |next_clock: i64, count: i64, count_clock: i64| -> PyValue {
            let mut m = IndexMap::new();
            m.insert(PyValue::Str("next_clock".to_string()), PyValue::Int(PyInt::from_i64(next_clock)));
            m.insert(PyValue::Str("count".to_string()), PyValue::Int(PyInt::from_i64(count)));
            m.insert(PyValue::Str("count_clock".to_string()), PyValue::Int(PyInt::from_i64(count_clock)));
            PyValue::Dict(m)
        };
        py.exec("pr.mcu.serial_cb({'next_clock': 18000, 'count': 0, 'count_clock': 0})").expect("py serial 1");
        h.mcu.fire_serial(mk(18_000, 0, 0));
        py.exec("pr.mcu.serial_cb({'next_clock': 12018000, 'count': 90, 'count_clock': 12000000})").expect("py serial 2");
        h.mcu.fire_serial(mk(12_018_000, 90, 12_000_000));
        check_status(label, &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
    }
}

#[test]
fn request_restart_turns_off() {
    let Some(mut py) = oracle() else { return };
    for (label, kick) in [("no-kick", "0"), ("kick", "0.1")] {
        let vals = [("pin", "pa0"), ("enable_pin", "pa1"), ("kick_start_time", kick)];
        py_build(&mut py, &vals);
        let h = harness(&vals);
        let fan = build_fan(&h);

        py.exec("fan.set_speed(0.7, 5.0); pr.send_event('gcode:request_restart', 6.0)").expect("py restart");
        fan.set_speed(0.7, Some(5.0)).unwrap();
        h.printer.send("gcode:request_restart", &[EventArg::Value(PyValue::Float(6.0))]);
        check_pins(label, &mut py, &h);
        check_status(label, &mut py, "fan.get_status(1.0)", &fan.get_status(1.0));
    }
}
