//! Parity tests for `extras::temperature_fan` (port of
//! `klippy/extras/temperature_fan.py`).
//!
//! The reference is driven through the real `extras.temperature_fan` module
//! (which builds the real `extras.fan` fan on Python fakes for pins/MCU),
//! and the port through `extras::temperature_fan`, with the same config,
//! sensor callbacks and G-code parameters. Expected values come from the
//! live oracle only (there are no recordings for this unit); floats are
//! compared bit-for-bit through the recording value format and errors by
//! class and message.
//!
//! Compared: construction (sensor `setup_minmax`, callback registration,
//! `register_sensor`, the `SET_TEMPERATURE_FAN_TARGET` mux registration,
//! fan pin setup), `get_status` / `get_temp`, fan PWM/digital side
//! effects of the bang-bang and PID callbacks, `set_tf_speed` rate
//! limiting and clamping, `set_temp` / `set_min_speed` / `set_max_speed`
//! zero-skip behaviour, every `SET_TEMPERATURE_FAN_TARGET` error path,
//! and every config validation error.

#![allow(clippy::type_complexity)]
use extras::heaters::{
    Heater, PrinterHeatersApi, SensorApi, TemperatureCallback, TemperatureSensorApi,
};
use extras::temperature_fan::{load_config_prefix, module_factory, TemperatureFan};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GetFloatOpts as GcmdFloatOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AsyncResponseWrapper, AsyncResponseWrapperApi, Chip, CommandQueue, CommandQueryWrapper,
    CommandWrapper, ConfigCallback, DigitalOutPinApi, LookupPinOpts, LookupQueryCommandOpts,
    McuApi, ParsePinOpts, Pin, PinParams, PinResolver, PinsApi, PwmPinApi,
    SerialResponseCallback, TriggerDispatch, Mcu,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{
    ExtraAxis, LookaheadCallback, MotionQueuingApi, PrinterExtruder, ToolHeadApi,
};
use pycompat::PyValue;
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
import extras.temperature_fan as _tf
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

class FakeSensor:
    def __init__(self):
        self.cb = None
    def setup_minmax(self, mn, mx):
        sensor_minmax.append((mn, mx))
    def setup_callback(self, cb):
        self.cb = cb
        sensor_cb.append(cb)
    def get_report_time_delta(self):
        return 0.5

class FakeHeaters:
    def __init__(self):
        self.sensor = FakeSensor()
    def setup_sensor(self, config):
        return self.sensor
    def register_sensor(self, config, psensor, gcode_id=None):
        registered.append((config.get_name(), psensor is not None, gcode_id))

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
    def register_mux_command(self, cmd, key, value, func, desc=None):
        mux_regs.append((cmd, key, value, desc))
        self.handlers[(cmd, key, value)] = func

class FakePrinter:
    def __init__(self):
        self.mcu = FakeMCU()
        self.pins = FakePins(self.mcu)
        self.mq = FakeMq()
        self.toolhead = FakeToolHead()
        self.gcode = FakeGcode()
        self.heaters = FakeHeaters()
        self.objects = {}
        self.handlers = {}
        self.reactor = FakeReactor()
    def command_error(self, msg):
        raise CommandError(msg)
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
        if name == 'heaters':
            return self.heaters
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
            raise error("Option '%s' in section '%s' must have minimum of %s" % (option, self.name, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' in section '%s' must have maximum of %s" % (option, self.name, maxval))
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
            raise error("Option '%s' in section '%s' must have minimum of %s" % (option, self.name, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' in section '%s' must have maximum of %s" % (option, self.name, maxval))
        if above is not None and v <= above:
            raise error("Option '%s' in section '%s' must be above %s" % (option, self.name, above))
        if below is not None and v >= below:
            raise error("Option '%s' in section '%s' must be below %s" % (option, self.name, below))
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
    def getchoice(self, option, choices, default=_sentinel):
        if option not in self.values:
            if default is _sentinel:
                raise error("Option '%s' in section '%s' must be specified" % (option, self.name))
            c = default
        else:
            c = self.values[option]
        if c not in choices:
            raise error("Choice '%s' for option '%s' in section '%s' is not a valid choice" % (c, option, self.name))
        return choices[c]

pwm_calls = []
digital_calls = []
pwm_setup = []
digital_setup = []
sensor_minmax = []
sensor_cb = []
registered = []
mux_regs = []
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
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.inner
            .lock()
            .objects
            .iter()
            .map(|(k, v)| (k.clone(), v.clone()))
            .collect()
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
                    "Option '{}' in section '{}' must have minimum of {:?}",
                    option, self.name, mn
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have maximum of {:?}",
                    option, self.name, mx
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
                    "Option '{}' in section '{}' must have minimum of {:?}",
                    option, self.name, mn
                )));
            }
        }
        if let Some(mx) = opts.maxval {
            if v > mx {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must have maximum of {:?}",
                    option, self.name, mx
                )));
            }
        }
        if let Some(a) = opts.above {
            if v <= a {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be above {:?}",
                    option, self.name, a
                )));
            }
        }
        if let Some(b) = opts.below {
            if v >= b {
                return Err(KlippyError::config(format!(
                    "Option '{}' in section '{}' must be below {:?}",
                    option, self.name, b
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
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let key: PyValue = if let Some(raw) = self.get_raw(option) {
            PyValue::Str(raw)
        } else {
            match opts.default {
                None => {
                    return Err(missing(&self.name, option));
                }
                Some(d) => d.unwrap_or(PyValue::None),
            }
        };
        match choices {
            PyValue::Dict(m) => {
                if let Some(v) = m.get(&key) {
                    Ok(v.clone())
                } else {
                    Err(KlippyError::config(format!(
                        "Choice '{}' for option '{}' in section '{}' is not a valid choice",
                        key.str(),
                        option,
                        self.name
                    )))
                }
            }
            _ => Ok(PyValue::None),
        }
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
    fn setup_cycle_time(&self, t: f64, hw: Option<bool>) {
        self.inner.lock().cycle.push((t, hw));
    }
    fn setup_start_value(&self, s: f64, sh: f64) {
        self.inner.lock().start.push((s, sh));
    }
    fn next_aligned_print_time(&self, t: f64, _a: Option<f64>) -> f64 {
        t
    }
    fn set_pwm(&self, print_time: f64, value: f64) {
        self.inner.lock().pwm_calls.push((print_time, value));
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
    fn reset_pin_sharing(&self, _p: PinParams) {}
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
    fn get_status(&self, _e: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
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

struct FakeSensor {
    report_delta: f64,
    minmax: klippy_api::GCell<Option<(f64, f64)>>,
    callback: klippy_api::GCell<Option<TemperatureCallback>>,
}

impl FakeSensor {
    fn new(report_delta: f64) -> Arc<Self> {
        Arc::new(FakeSensor {
            report_delta,
            minmax: klippy_api::GCell::new(None),
            callback: klippy_api::GCell::new(None),
        })
    }
    fn minmax(&self) -> Option<(f64, f64)> {
        *self.minmax.borrow()
    }
    fn fire(&self, read_time: f64, temp: f64) {
        let mut slot = self.callback.borrow_mut();
        if let Some(cb) = slot.as_mut() {
            cb(read_time, temp);
        } else {
            panic!("no callback registered");
        }
    }
}

impl TemperatureSensorApi for FakeSensor {
    fn setup_minmax(&self, min_temp: f64, max_temp: f64) -> KResult<()> {
        *self.minmax.borrow_mut() = Some((min_temp, max_temp));
        Ok(())
    }
    fn setup_callback(&self, callback: TemperatureCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }
    fn get_report_time_delta(&self) -> f64 {
        self.report_delta
    }
}

klippy_api::printer_object!(FakeSensor; dyn TemperatureSensorApi);

#[derive(Clone)]
struct FakeHeaters {
    sensor: Arc<FakeSensor>,
    registered: Arc<Mutex<Vec<String>>>,
}

impl FakeHeaters {
    fn new() -> Self {
        FakeHeaters {
            sensor: FakeSensor::new(0.5),
            registered: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn registered(&self) -> Vec<String> {
        self.registered.lock().clone()
    }
}

impl PrinterHeatersApi for FakeHeaters {
    fn add_sensor_factory(&self, _sensor_type: &str, _factory: extras::heaters::SensorFactory) {}
    fn setup_heater(&self, _config: &Config, _gcode_id: Option<String>) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("setup_heater not faked"))
    }
    fn get_all_heaters(&self) -> Vec<String> {
        Vec::new()
    }
    fn lookup_heater(&self, _heater_name: &str) -> KResult<Arc<Heater>> {
        Err(KlippyError::config("lookup_heater not faked"))
    }
    fn setup_sensor(&self, _config: &Config) -> KResult<Arc<dyn TemperatureSensorApi>> {
        Ok(self.sensor.clone())
    }
    fn register_sensor(
        &self,
        config: &Config,
        _psensor: Arc<dyn SensorApi>,
        _gcode_id: Option<String>,
    ) -> KResult<()> {
        self.registered.lock().push(config.get_name());
        Ok(())
    }
    fn register_monitor(&self, _config: &Config) {}
    fn turn_off_all_heaters(&self, _print_time: f64) {}
    fn set_temperature(&self, _heater: &Arc<Heater>, _temp: f64, _wait: bool) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakeHeaters; dyn PrinterHeatersApi);

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
        "SET_TEMPERATURE_FAN_TARGET".to_string()
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
    mux: Vec<(String, String, Option<String>, Option<String>, Box<dyn FnMut(&klippy_api::gcode::GCmd) -> KResult<()> + Send>)>,
}

impl FakeGcode {
    fn mux_regs(&self) -> Vec<(String, String, Option<String>, Option<String>)> {
        self.inner
            .lock()
            .mux
            .iter()
            .map(|(c, k, v, d, _)| (c.clone(), k.clone(), v.clone(), d.clone()))
            .collect()
    }
    fn call_mux(&self, key: &str, value: &str, gcmd: &klippy_api::gcode::GCmd) -> KResult<()> {
        let mut inner = self.inner.lock();
        for (c, k, v, _d, handler) in inner.mux.iter_mut() {
            if c == "SET_TEMPERATURE_FAN_TARGET" && k == key && v.as_deref() == Some(value) {
                return handler(gcmd);
            }
        }
        Err(KlippyError::command(format!(
            "unknown mux command {} {}",
            key, value
        )))
    }
}

impl GCodeApi for FakeGcode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<klippy_api::gcode::GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<klippy_api::gcode::GCodeCommandHandler>> {
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
        self.inner.lock().mux.push((
            cmd.to_string(),
            key.to_string(),
            value.map(|s| s.to_string()),
            desc.map(|s| s.to_string()),
            func,
        ));
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
        false
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
            py.exec(SETUP).expect("temperature_fan fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping temperature_fan oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "temperature_fan".to_string(),
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
            Value::Tuple(vec![get("speed"), get("rpm"), get("temperature"), get("target")])
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

fn minmax_value(v: Option<(f64, f64)>) -> Value {
    match v {
        Some((a, b)) => Value::Tuple(vec![Value::float(a), Value::float(b)]),
        None => Value::Null,
    }
}

fn mux_value(regs: &[(String, String, Option<String>, Option<String>)]) -> Value {
    Value::List(
        regs
            .iter()
            .map(|(c, k, v, d)| {
                Value::Tuple(vec![
                    Value::str(c),
                    Value::str(k),
                    match v {
                        Some(s) => Value::str(s),
                        None => Value::Null,
                    },
                    match d {
                        Some(s) => Value::str(s),
                        None => Value::Null,
                    },
                ])
            })
            .collect(),
    )
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

struct Harness {
    printer: FakePrinter,
    config: Config,
    pins: FakePins,
    gcode: FakeGcode,
    heaters: FakeHeaters,
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
    let heaters = FakeHeaters::new();
    printer.add_object("pins", Arc::new(pins.clone())).unwrap();
    printer.add_object("motion_queuing", Arc::new(mq.clone())).unwrap();
    printer.add_object("toolhead", Arc::new(toolhead.clone())).unwrap();
    printer.add_object("gcode", Arc::new(gcode.clone())).unwrap();
    printer.add_object("heaters", Arc::new(heaters.clone())).unwrap();
    let config: Config =
        Arc::new(FakeConfig::new(printer_arc.clone(), "temperature_fan fsf", config_vals));
    Harness { printer, config, pins, gcode, heaters }
}

fn py_build(py: &mut pyoracle::Oracle, vals: &[(&str, &str)]) {
    let expr = py_config_expr(vals);
    py.exec(&format!(
        "pr = FakePrinter(); pwm_calls.clear(); digital_calls.clear(); pwm_setup.clear(); digital_setup.clear(); sensor_minmax.clear(); sensor_cb.clear(); registered.clear(); mux_regs.clear(); cfg = FakeConfig(pr, 'temperature_fan fsf', {expr}); tf = _tf.load_config_prefix(cfg); pr.send_event('klippy:connect')"
    ))
    .expect("py build temperature fan");
}

fn build_tf(h: &Harness) -> Arc<TemperatureFan> {
    let tf = TemperatureFan::new(&h.config).unwrap();
    h.printer.send("klippy:connect", &[]);
    tf
}

/// Fires one sensor callback on both sides with the same read_time/temp.
fn fire(label: &str, py: &mut pyoracle::Oracle, h: &Harness, read_time: f64, temp: f64) {
    // Rust floats into Python code must render as Python float literals:
    // `{x}` prints 1 for 1.0 (an int), so use {:?}; bools need True/False.
    py.exec(&format!(
        "sensor_cb[0]({read_time:?}, {temp:?})"
    ))
    .expect("py fire");
    h.heaters.sensor.fire(read_time, temp);
    check_pins(label, py, h);
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
        Value::List(
            h.pins
                .digital
                .digital_calls()
                .iter()
                .map(|(t, v)| Value::Tuple(vec![Value::float(*t), Value::Bool(*v)]))
                .collect(),
        ),
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

fn check_status(label: &str, py: &mut pyoracle::Oracle, tf: &Arc<TemperatureFan>) {
    check(
        &format!("{label}: status"),
        py.eval("(lambda s: (s['speed'], s['rpm'], s['temperature'], s['target']))(tf.get_status(0.0))"),
        status_value(&tf.get_status(0.0)),
    );
}

fn check_get_temp(label: &str, py: &mut pyoracle::Oracle, tf: &Arc<TemperatureFan>) {
    let (a, b) = tf.get_temp(0.0);
    check(
        &format!("{label}: get_temp"),
        py.eval("tf.get_temp(0.0)"),
        Value::Tuple(vec![Value::float(a), Value::float(b)]),
    );
}

fn check_speeds(label: &str, py: &mut pyoracle::Oracle, tf: &Arc<TemperatureFan>) {
    check(
        &format!("{label}: speeds"),
        py.eval("(tf.get_min_speed(), tf.get_max_speed())"),
        Value::Tuple(vec![Value::float(tf.get_min_speed()), Value::float(tf.get_max_speed())]),
    );
}

fn gcmd(params: &[(&str, &str)]) -> klippy_api::gcode::GCmd {
    Arc::new(FakeGcodeCmd::new(params))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_load_config_prefix() {
    let factory = module_factory();
    assert!(factory.load_config.is_none());
    assert!(factory.load_config_prefix.is_some());
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn load_config_prefix_returns_object() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
    ];
    py_build(&mut py, &vals);
    check(
        "load-config-prefix type",
        py.eval("type(tf).__name__"),
        Value::str("TemperatureFan"),
    );
    let h = harness(&vals);
    let obj = load_config_prefix(&h.config).expect("load_config_prefix");
    assert!(obj.is_some());
    let obj = obj.unwrap();
    let tf_any = klippy_api::downcast_arc::<TemperatureFan>(&obj).expect("downcast");
    check_status("after-build", &mut py, &tf_any);
}

#[test]
fn construction_defaults() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);

    check(
        "sensor-minmax",
        py.eval("list(sensor_minmax)"),
        Value::List(vec![minmax_value(h.heaters.sensor.minmax())]),
    );
    check(
        "mux-registered",
        py.eval("list(mux_regs)"),
        mux_value(&h.gcode.mux_regs()),
    );
    check(
        "register-sensor",
        py.eval("[r[0] for r in registered]"),
        Value::List(h.heaters.registered().iter().map(|s| Value::str(s)).collect()),
    );
    check_setup("construction", &mut py, &h);
    check_status("initial", &mut py, &tf);
    check_get_temp("initial", &mut py, &tf);
    check_speeds("defaults", &mut py, &tf);
}

#[test]
fn default_target_follows_max_temp() {
    let Some(mut py) = oracle() else { return };
    // With max_temp <= 40 the default target is max_temp itself.
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "30"),
        ("control", "watermark"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);
    check_status("target-max", &mut py, &tf);
    check_get_temp("target-max", &mut py, &tf);
}

#[test]
fn bang_bang_transitions() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
        ("max_delta", "5"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);

    // target=40, delta=5. Equal-to-boundary temperatures flip the state.
    let steps: [(f64, f64); 6] = [
        (1.0, 35.0),  // == target-delta -> heating=true, speed 0 suppressed
        (2.0, 44.0),  // < target+delta, stays heating -> still 0
        (3.0, 45.0),  // == target+delta -> heating=false, speed 1.0 sent
        (4.0, 36.0),  // inside band, stays cooling, same speed suppressed
        (7.0, 35.0),  // heating again, speed 0 sent
        (8.0, 35.0),  // same value, suppressed
    ];
    for (i, (t, temp)) in steps.iter().enumerate() {
        fire(&format!("bang-{i}"), &mut py, &h, *t, *temp);
        check_status(&format!("bang-{i}"), &mut py, &tf);
        check_get_temp(&format!("bang-{i}"), &mut py, &tf);
    }
}

#[test]
fn pid_control() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "50"),
        ("control", "pid"),
        ("pid_Kp", "255"),
        ("pid_Ki", "127.5"),
        ("pid_Kd", "127.5"),
        ("min_speed", "0.3"),
        ("max_speed", "1"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);

    // Steps exercise: smoothed derivative, integral clamped to
    // temp_integ_max, integral stored only while unsaturated (co ==
    // bounded_co), the time_diff >= pid_deriv_time resync branch, and
    // min_speed flooring with suppression.
    let steps: [(f64, f64); 5] = [
        (1.0, 30.0),
        (2.0, 35.0),
        (5.0, 45.0),
        (6.0, 38.0),
        (7.0, 39.5),
    ];
    for (i, (t, temp)) in steps.iter().enumerate() {
        fire(&format!("pid-{i}"), &mut py, &h, *t, *temp);
        check_status(&format!("pid-{i}"), &mut py, &tf);
        check_get_temp(&format!("pid-{i}"), &mut py, &tf);
    }
}

#[test]
fn pid_deriv_time_override() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "50"),
        ("control", "pid"),
        ("pid_Kp", "76.5"),
        ("pid_Ki", "0"),
        ("pid_Kd", "0"),
        ("pid_deriv_time", "0.5"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);
    let steps: [(f64, f64); 3] = [(0.25, 20.0), (2.0, 60.0), (2.5, 20.0)];
    for (i, (t, temp)) in steps.iter().enumerate() {
        fire(&format!("pidd-{i}"), &mut py, &h, *t, *temp);
        check_status(&format!("pidd-{i}"), &mut py, &tf);
    }
}

#[test]
fn set_tf_speed_clamping_and_suppression() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);

    let steps: [(f64, f64); 5] = [
        (1.0, 0.5),   // sent as-is
        (2.0, 0.1),   // clamped up to min_speed 0.3
        (2.5, 0.3),   // identical, suppressed
        (7.0, 0.32),  // small change but past the 3.75s window -> sent
        (8.0, 0.0),   // zero is valid and sent
    ];
    for (i, (t, v)) in steps.iter().enumerate() {
        py.exec(&format!("tf.set_tf_speed({t:?}, {v:?})")).expect("py set_tf_speed");
        tf.set_tf_speed(*t, *v).unwrap();
        check_pins(&format!("tf-speed-{i}"), &mut py, &h);
        check_status(&format!("tf-speed-{i}"), &mut py, &tf);
    }

    // A zero target forces the speed to zero whatever the request.
    py.exec("tf.set_temp(0.0); tf.set_tf_speed(9.0, 0.9)").expect("py set_tf_speed zero");
    tf.set_temp(0.0).unwrap();
    tf.set_tf_speed(9.0, 0.9).unwrap();
    check_pins("tf-speed-zero", &mut py, &h);
    check_status("tf-speed-zero", &mut py, &tf);
}

#[test]
fn gcode_command() {
    let Some(mut py) = oracle() else { return };
    let vals = [
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
    ];
    py_build(&mut py, &vals);
    let h = harness(&vals);
    let tf = build_tf(&h);

    let py_call = |expr: &str| format!(
        "pr.gcode.handlers[('SET_TEMPERATURE_FAN_TARGET', 'TEMPERATURE_FAN', 'fsf')](FakeGcmd({expr}))"
    );

    let ok_cases: &[(&[(&str, &str)], &str)] = &[
        (&[("TARGET", "50")], "target-50"),
        (&[("MIN_SPEED", "0.7")], "min-0.7"),
        (&[("TARGET", "0")], "target-zero"),   // 0 skips the range check
        (&[("MIN_SPEED", "0")], "min-zero"),   // 0 skips the range check
    ];
    for (params, label) in ok_cases {
        let expr = py_config_expr(params);
        py.exec(&py_call(&expr)).expect("py command ok");
        h.gcode
            .call_mux("TEMPERATURE_FAN", "fsf", &gcmd(params))
            .expect("rust command ok");
        check_get_temp(label, &mut py, &tf);
        check_speeds(label, &mut py, &tf);
        check_status(label, &mut py, &tf);
    }

    let err_cases: &[(&[(&str, &str)], &str)] = &[
        (&[("TARGET", "70")], "target-high"),
        (&[("TARGET", "-5")], "target-low"),
        (&[("MIN_SPEED", "0.9"), ("MAX_SPEED", "0.5")], "min-greater-max"),
        (&[("MIN_SPEED", "2.0")], "min-range"),
        (&[("MAX_SPEED", "2.0")], "max-range"),
    ];
    for (params, label) in err_cases {
        let expr = py_config_expr(params);
        let expected = py.eval(&py_call(&expr));
        let err = h
            .gcode
            .call_mux("TEMPERATURE_FAN", "fsf", &gcmd(params))
            .expect_err("command must fail");
        check_err(label, expected, err);
    }

    // min-speed and max-speed range messages are also reachable directly
    // (the in-command path for a negative max always hits the
    // min-greater-than-max error first).
    let expected = py.eval("tf.set_max_speed(-0.5)");
    let err = tf.set_max_speed(-0.5).expect_err("negative max must fail");
    check_err("set-max-negative", expected, err);
    let expected = py.eval("tf.set_min_speed(1.5)");
    let err = tf.set_min_speed(1.5).expect_err("over max must fail");
    check_err("set-min-over", expected, err);
    // Zero speeds skip validation.
    py.exec("tf.set_max_speed(0.0); tf.set_min_speed(0.0)").expect("py zero speeds");
    tf.set_max_speed(0.0).unwrap();
    tf.set_min_speed(0.0).unwrap();
    check_speeds("zero-speeds", &mut py, &tf);
}

#[test]
fn config_errors() {
    let Some(mut py) = oracle() else { return };
    let base: Vec<(&str, &str)> = vec![
        ("pin", "pa0"),
        ("min_temp", "0"),
        ("max_temp", "60"),
        ("control", "watermark"),
    ];
    // Each case replaces or drops one entry of the base config.
    fn with<'a>(base: &[(&'a str, &'a str)], drop: &str, extra: Option<(&'a str, &'a str)>) -> Vec<(&'a str, &'a str)> {
        let mut v: Vec<(&'a str, &'a str)> = base
            .iter()
            .cloned()
            .filter(|(k, _)| *k != drop)
            .collect();
        if let Some(e) = extra {
            v.push(e);
        }
        v
    }
    let cases: Vec<(&str, Vec<(&str, &str)>)> = vec![
        ("missing-min-temp", with(&base, "min_temp", None)),
        ("min-temp-low", with(&base, "min_temp", Some(("min_temp", "-300")))),
        ("min-temp-text", with(&base, "min_temp", Some(("min_temp", "abc")))),
        ("missing-max-temp", with(&base, "max_temp", None)),
        ("max-temp-not-above", with(&base, "max_temp", Some(("max_temp", "0")))),
        ("missing-control", with(&base, "control", None)),
        ("bad-control", with(&base, "control", Some(("control", "foo")))),
        ("max-delta-zero", with(&base, "zzz", Some(("max_delta", "0")))),
        ("pid-missing-kp", with(&base, "control", Some(("control", "pid")))),
        (
            "pid-deriv-zero",
            with(&base, "control", Some(("control", "pid")))
                .into_iter()
                .chain(vec![("pid_Kp", "255"), ("pid_Ki", "0"), ("pid_Kd", "0"), ("pid_deriv_time", "0")])
                .collect(),
        ),
        ("max-speed-high", with(&base, "zzz", Some(("max_speed", "1.5")))),
        ("max-speed-zero", with(&base, "zzz", Some(("max_speed", "0")))),
        ("min-speed-negative", with(&base, "zzz", Some(("min_speed", "-0.1")))),
        ("target-above-max", with(&base, "zzz", Some(("target_temp", "70")))),
        ("target-below-min", with(&base, "zzz", Some(("target_temp", "-1")))),
        ("missing-pin", with(&base, "pin", None)),
    ];
    for (label, vals) in cases {
        let expr = py_config_expr(&vals);
        let expected = py.eval(&format!(
            "_tf.load_config_prefix(FakeConfig(FakePrinter(), 'temperature_fan fsf', {expr}))"
        ));
        let h = harness(&vals);
        let err = TemperatureFan::new(&h.config).err().expect("rust must fail");
        check_err(label, expected, err);
    }
}
