//! Parity tests for `extras::neopixel` (port of `klippy/extras/neopixel.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): a fresh `PrinterNeoPixel` is
//! built on both sides with minimal Python fakes for the printer, pins, MCU,
//! G-code dispatcher and template evaluator, then fed identical config,
//! color states and klippy events, and the config errors, MCU command
//! streams, framebuffer bytes, status dicts and log lines are compared
//! exactly.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit,
//! so replay fixtures do not apply; expected values come from the oracle
//! alone.
//!
//! Assumptions about the sibling modules: `crate::led::LEDHelper` exposes
//! `new(config, update_func: LedUpdateFunc, led_count: usize)
//! -> KResult<Arc<LEDHelper>>` with `LedUpdateFunc = Arc<dyn Fn(&[LedColor],
//! Option<f64>) -> KResult<()> + Send + Sync>` and
//! `get_status(&self, eventtime: f64) -> PyValue`. The template evaluator
//! is the real `output_pin.PrinterTemplateEvaluator` on both sides, built
//! from `display_template` / `gcode_macro` fakes.

use extras::neopixel::{
    PrinterNeoPixel, BACKGROUND_PRIORITY_CLOCK, BIT_MAX_TIME, MAX_MCU_SIZE,
    RESET_MIN_TIME,
};
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi,
    LookupPinOpts, LookupQueryCommandOpts, McuApi, ParsePinOpts, Pin, PinParams,
    PinsApi, QuerySendOpts, SendOpts, TriggerDispatch, Chip, PinResolver,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{ObjectRef, EventArg, PrinterObject};
use pycompat::PyValue;
use pycompat::text::float_str;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import logging
import extras.neopixel as neopixel

LOG_LINES = []
class _LogHandler(logging.Handler):
    def emit(self, record):
        LOG_LINES.append(record.getMessage())
_root = logging.getLogger()
_root.handlers = [_LogHandler()]
_root.setLevel(logging.INFO)
_root.propagate = False

SENTINEL = object()

class FakeReactorMutex:
    def __enter__(self): return self
    def __exit__(self, *args): return False

class FakeReactor:
    def mutex(self): return FakeReactorMutex()
    def register_callback(self, cb): pass
    def monotonic(self): return 0.0

class FakeGCode:
    def __init__(self): self.mux = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append((cmd, key, value, desc))

class FakeDisplayTemplates:
    def __init__(self): self.templates = {}
    def get_display_templates(self): return dict(self.templates)

class FakeMacro:
    def __init__(self): self.contexts = []
    def create_template_context(self, eventtime):
        self.contexts.append(eventtime)
        return {'eventtime': eventtime}

class FakeCmd:
    def __init__(self, mcu, msg): self.mcu = mcu; self.msg = msg
    def send(self, params, minclock=0, reqclock=0):
        self.mcu.sent_updates.append((self.msg, list(params), minclock, reqclock))

class FakeQueryCmd:
    def __init__(self, mcu, msg, resp, oid):
        self.mcu = mcu; self.msg = msg; self.resp = resp; self.oid = oid
    def send(self, params, minclock=0, reqclock=0, retry=True):
        self.mcu.sent_queries.append((self.msg, list(params), minclock, reqclock))
        idx = min(len(self.mcu.sent_queries) - 1, len(self.mcu.success_seq) - 1)
        return {'oid': self.oid, 'success': self.mcu.success_seq[idx]}

class FakeMcu:
    def __init__(self, success_seq=None):
        self.oid_seq = 0
        self.callbacks = []
        self.config_cmds = []
        self.update_lookups = []
        self.query_lookups = []
        self.alloc_count = 0
        self.sent_updates = []
        self.sent_queries = []
        self.success_seq = [True] if success_seq is None else list(success_seq)
        self.freq = 1000000.0
    def create_oid(self): self.oid_seq += 1; return self.oid_seq
    def register_config_callback(self, cb): self.callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append(cmd)
    def seconds_to_clock(self, t): return int(t * self.freq)
    def alloc_command_queue(self):
        self.alloc_count += 1
        return object()
    def lookup_command(self, msg, cq=None):
        self.update_lookups.append(msg)
        return FakeCmd(self, msg)
    def lookup_query_command(self, msg, resp, oid=None, cq=None, is_async=False):
        self.query_lookups.append((msg, resp, oid))
        return FakeQueryCmd(self, msg, resp, oid)
    def print_time_to_clock(self, pt): return int(pt * self.freq)

class FakePins:
    def __init__(self, mcu): self.mcu = mcu
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        return {'chip': self.mcu, 'pin': pin_desc}

class FakePrinter:
    config_error = configparser.Error
    command_error = Exception
    def __init__(self, mcu, start_args=None):
        self.mcu = mcu
        self.gcode = FakeGCode()
        self.objects = {
            'pins': FakePins(mcu),
            'gcode': self.gcode,
            'display_template': FakeDisplayTemplates(),
            'gcode_macro': FakeMacro(),
            'mcu': mcu,
        }
        self.handlers = []
        self.start_args = dict(start_args) if start_args else {}
    def lookup_object(self, name, default=SENTINEL):
        if name in self.objects:
            return self.objects[name]
        if default is SENTINEL:
            raise configparser.Error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        if name in self.objects:
            raise configparser.Error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def load_object(self, config, section, default=SENTINEL):
        if section in self.objects:
            return self.objects[section]
        if default is SENTINEL:
            raise configparser.Error("Unable to load module '%s'" % (section,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def send_event(self, event, *params):
        return [cb(*params) for (e, cb) in self.handlers if e == event]
    def get_start_args(self): return self.start_args
    def get_reactor(self): return FakeReactor()

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, vals, section='neopixel test'):
        self._printer = printer
        self._vals = dict(vals)
        self._section = section
    def get_printer(self): return self._printer
    def get_name(self): return self._section
    def get(self, option, default=None, note_valid=True):
        if option in self._vals: return self._vals[option]
        if default is None:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        return default
    def getint(self, option, default=None, minval=None, maxval=None, note_valid=True):
        if option in self._vals: v = self._vals[option]
        elif default is None:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        else: v = default
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        return v
    def getlist(self, option, default=None, sep=',', note_valid=True):
        if option in self._vals: return list(self._vals[option])
        if default is None:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        return list(default)
    def getfloat(self, option, default=None, minval=None, maxval=None, above=None, below=None, note_valid=True):
        if option in self._vals: v = float(self._vals[option])
        elif default is None:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self._section))
        else: v = float(default)
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self._section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self._section, maxval))
        if above is not None and v <= above:
            raise configparser.Error("Option '%s' in section '%s' must be above %s" % (option, self._section, above))
        if below is not None and v >= below:
            raise configparser.Error("Option '%s' in section '%s' must be below %s" % (option, self._section, below))
        return v
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum FakeVal {
    Str(String),
    Int(i64),
    List(Vec<String>),
    Float(f64),
}

fn missing_msg(section: &str, option: &str) -> String {
    format!("Option '{option}' in section '{section}' must be specified")
}

struct PrinterInner {
    objects: Vec<(String, ObjectRef)>,
    handlers: Vec<(String, EventCallback)>,
    start_args: PyValue,
}

struct FakePrinter {
    shared: Arc<kos::sync::Mutex<PrinterInner>>,
}

impl Clone for FakePrinter {
    fn clone(&self) -> Self {
        FakePrinter { shared: self.shared.clone() }
    }
}

impl FakePrinter {
    fn new(objects: Vec<(String, ObjectRef)>, start_args: PyValue) -> Self {
        FakePrinter {
            shared: Arc::new(kos::sync::Mutex::new(PrinterInner {
                objects,
                handlers: Vec::new(),
                start_args,
            })),
    }
    }
    fn handler_events(&self) -> Vec<String> {
        self.shared.lock().handlers.iter().map(|(e, _)| e.clone()).collect()
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.shared.lock().start_args.clone()
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
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut inner = self.shared.lock();
        if inner.objects.iter().any(|(n, _)| n == name) {
            return Err(KlippyError::config(format!(
                "Printer object '{name}' already created"
            )));
        }
        inner.objects.push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(
        &self,
        name: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let inner = self.shared.lock();
        for (n, o) in &inner.objects {
            if n == name {
                return Ok(Some(o.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unknown config object '{name}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        Vec::new()
    }
    fn load_object(
        &self,
        _config: &Config,
        section: &str,
        default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        let inner = self.shared.lock();
        for (n, o) in &inner.objects {
            if n == section {
                return Ok(Some(o.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!(
                "Unable to load module '{section}'"
            ))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.shared.lock().handlers.push((event.to_string(), callback));
    }
    fn send_event(
        &self,
        event: &str,
        params: &[EventArg],
    ) -> KResult<Vec<PyValue>> {
        let handlers = {
            let mut inner = self.shared.lock();
            std::mem::take(&mut inner.handlers)
        };
        let mut matching: Vec<EventCallback> = Vec::new();
        let mut rest: Vec<(String, EventCallback)> = Vec::new();
        for (e, cb) in handlers {
            if e == event {
                matching.push(cb);
            } else {
                rest.push((e, cb));
            }
        }
        {
            let mut inner = self.shared.lock();
            inner.handlers = rest;
        }
        let mut results = Vec::new();
        for mut cb in matching {
            results.push(cb(params)?);
        }
        Ok(results)
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    vals: indexmap::IndexMap<String, FakeVal>,
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.vals.get(option) {
            Some(FakeVal::Str(v)) => Ok(Some(v.clone())),
            Some(_) => panic!("get on non-string option {option}"),
            None => match opts.default {
                None => Err(KlippyError::config(missing_msg(&self.section, option))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let v: i64 = match self.vals.get(option) {
            Some(FakeVal::Int(v)) => *v,
            Some(_) => panic!("getint on non-int option {option}"),
            None => match opts.default {
                None => {
                    return Err(KlippyError::config(missing_msg(&self.section, option)))
                }
                Some(None) => return Ok(None),
                Some(Some(v)) => v,
            },
        };
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {min}",
                    self.section
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {max}",
                    self.section
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, option: &str, opts: GetFloatOpts) -> KResult<Option<f64>> {
        let v: f64 = match self.vals.get(option) {
            Some(FakeVal::Float(v)) => *v,
            Some(_) => panic!("getfloat on non-float option {option}"),
            None => match opts.default {
                None => {
                    return Err(KlippyError::config(missing_msg(&self.section, option)))
                }
                Some(None) => return Ok(None),
                Some(Some(v)) => v,
            },
        };
        if let Some(min) = opts.minval {
            if v < min {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {}",
                    self.section,
                    float_str(min)
                )));
            }
        }
        if let Some(max) = opts.maxval {
            if v > max {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {}",
                    self.section,
                    float_str(max)
                )));
            }
        }
        if let Some(above) = opts.above {
            if v <= above {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be above {}",
                    self.section,
                    float_str(above)
                )));
            }
        }
        if let Some(below) = opts.below {
            if v >= below {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be below {}",
                    self.section,
                    float_str(below)
                )));
            }
        }
        Ok(Some(v))
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by neopixel")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by neopixel")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by neopixel")
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.vals.get(option) {
            Some(FakeVal::List(v)) => Ok(Some(v.clone())),
            Some(_) => panic!("getlist on non-list option {option}"),
            None => match opts.default {
                None => Err(KlippyError::config(missing_msg(&self.section, option))),
                Some(None) => Ok(None),
                Some(Some(v)) => Ok(Some(v)),
            },
        }
    }
    fn getintlist(
        &self,
        _option: &str,
        _opts: GetIntListOpts,
    ) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by neopixel")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by neopixel")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by neopixel")
    }
    fn has_section(&self, _section: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> {
        Vec::new()
    }
    fn get_prefix_options(&self, _prefix: &str) -> Vec<String> {
        Vec::new()
    }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) {}
}

struct McuState {
    oid_next: i64,
    callbacks: Vec<klippy_api::mcu::ConfigCallback>,
    config_cmds: Vec<String>,
    update_lookups: Vec<String>,
    query_lookups: Vec<(String, String, Option<i64>)>,
    alloc_next: i64,
    sent_updates: Vec<(String, Vec<PyValue>, SendOpts)>,
    sent_queries: Vec<(String, Vec<PyValue>, QuerySendOpts)>,
    success_seq: Vec<bool>,
    freq: f64,
}

fn trunc_i64(x: f64) -> i64 {
    pycompat::text::trunc_float_to_bigint(x)
        .inner()
        .to_string()
        .parse::<i64>()
        .unwrap_or(if x < 0.0 { i64::MIN } else { i64::MAX })
}

struct FakeMcu {
    state: Arc<kos::sync::Mutex<McuState>>,
    printer: Printer,
}

impl FakeMcu {
    fn new(success_seq: Vec<bool>) -> Self {
        FakeMcu {
            state: Arc::new(kos::sync::Mutex::new(McuState {
                oid_next: 0,
                callbacks: Vec::new(),
                config_cmds: Vec::new(),
                update_lookups: Vec::new(),
                query_lookups: Vec::new(),
                alloc_next: 0,
                sent_updates: Vec::new(),
                sent_queries: Vec::new(),
                success_seq,
                freq: 1_000_000.0,
            })),
            printer: Arc::new(FakePrinter::new(Vec::new(), PyValue::Dict(indexmap::IndexMap::new()))),
        }
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".to_string()
    }
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by neopixel")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut s = self.state.lock();
        s.oid_next += 1;
        Ok(s.oid_next)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.state.lock().callbacks.push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.state.lock().config_cmds.push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        let freq = self.state.lock().freq;
        trunc_i64(time * freq)
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by neopixel")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by neopixel")
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        _cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        self.state.lock().update_lookups.push(msgformat.to_string());
        Ok(Arc::new(FakeUpdateCmd {
            state: self.state.clone(),
            msgformat: msgformat.to_string(),
        }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.state.lock().query_lookups.push((
            msgformat.to_string(),
            respformat.to_string(),
            opts.oid,
        ));
        Ok(Arc::new(FakeSendCmd {
            state: self.state.clone(),
            msgformat: msgformat.to_string(),
            respformat: respformat.to_string(),
            oid: opts.oid,
        }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        panic!("try_lookup_command is not used by neopixel")
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut s = self.state.lock();
        s.alloc_next += 1;
        CommandQueue(s.alloc_next)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        panic!("register_serial_response is not used by neopixel")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        false
    }
    fn get_enumerations(&self) -> indexmap::IndexMap<String, indexmap::IndexMap<String, i64>> {
        indexmap::IndexMap::new()
    }
    fn get_constants(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by neopixel")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        let freq = self.state.lock().freq;
        trunc_i64(print_time * freq)
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        panic!("clock_to_print_time is not used by neopixel")
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by neopixel")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by neopixel")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by neopixel")
    }
}

struct FakeUpdateCmd {
    state: Arc<kos::sync::Mutex<McuState>>,
    msgformat: String,
}

impl CommandWrapperApi for FakeUpdateCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.state.lock().sent_updates.push((self.msgformat.clone(), data, opts));
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.send(data, opts)
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeSendCmd {
    state: Arc<kos::sync::Mutex<McuState>>,
    msgformat: String,
    respformat: String,
    oid: Option<i64>,
}

impl CommandQueryWrapperApi for FakeSendCmd {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        let mut s = self.state.lock();
        s.sent_queries.push((self.msgformat.clone(), data, opts));
        let idx = (s.sent_queries.len() - 1).min(s.success_seq.len() - 1);
        let success = s.success_seq[idx];
        let _ = &self.respformat;
        let mut dict = indexmap::IndexMap::new();
        dict.insert(
            PyValue::Str("oid".to_string()),
            PyValue::Int(pycompat::PyInt::from_i64(self.oid.unwrap_or(0))),
        );
        dict.insert(PyValue::Str("success".to_string()), PyValue::Bool(success));
        Ok(PyValue::Dict(dict))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        self.send(data, opts)
    }
}

struct FakePins {
    mcu: Arc<dyn McuApi>,
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by neopixel")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        Ok(PinParams {
            chip: self.mcu.clone(),
            chip_name: "mcu".to_string(),
            pin: pin_desc.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type.clone(),
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by neopixel")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used by neopixel")
    }
    fn register_chip(&self, _chip_name: &str, _chip: Chip) -> KResult<()> {
        panic!("register_chip is not used by neopixel")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by neopixel")
    }
}

type MuxRegs = Arc<kos::sync::Mutex<Vec<(String, String, String, String)>>>;

struct FakeGCode {
    regs: MuxRegs,
}

klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("register_command is not used by neopixel")
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.regs.lock().push((
            cmd.to_string(),
            key.to_string(),
            value.unwrap_or_default().to_string(),
            desc.unwrap_or_default().to_string(),
        ));
        Ok(())
    }
    fn get_command_help(&self) -> indexmap::IndexMap<String, String> {
        indexmap::IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(indexmap::IndexMap::new())
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {}
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
        Reactor::new(false).mutex(false)
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &indexmap::IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by neopixel")
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

// The template evaluator is the real `output_pin.PrinterTemplateEvaluator`
// on both sides (via `lookup_template_eval`); it needs `display_template`
// and `gcode_macro` objects, faked below.
struct FakeDisplayTemplates;

klippy_api::printer_object!(FakeDisplayTemplates; dyn extras::display::display::PrinterDisplayTemplateApi);

impl extras::display::display::PrinterDisplayTemplateApi for FakeDisplayTemplates {
    fn get_display_templates(
        &self,
    ) -> indexmap::IndexMap<String, std::sync::Arc<dyn extras::display::display::DisplayTemplateApi>> {
        indexmap::IndexMap::new()
    }
    fn get_display_data_groups(
        &self,
    ) -> indexmap::IndexMap<String, std::sync::Arc<extras::display::display::DisplayGroup>> {
        indexmap::IndexMap::new()
    }
    fn get_display_glyphs(&self) -> extras::display::display::GlyphTable {
        indexmap::IndexMap::new()
    }
}

struct FakeMacro {
    contexts: kos::sync::Mutex<Vec<f64>>,
}

klippy_api::printer_object!(FakeMacro; dyn extras::gcode_macro::PrinterGCodeMacroApi);

impl extras::gcode_macro::PrinterGCodeMacroApi for FakeMacro {
    fn load_template(
        &self,
        _config: &klippy_api::config::Config,
        _option: &str,
        _default: Option<&str>,
    ) -> klippy_api::error::KResult<extras::gcode_macro::TemplateWrapper> {
        unreachable!("neopixel never loads templates")
    }
    fn create_template_context(
        &self,
        eventtime: Option<f64>,
    ) -> indexmap::IndexMap<String, extras_macros_templates::gcode_macro::Value> {
        let eventtime = eventtime.expect("eventtime");
        self.contexts.lock().push(eventtime);
        let mut out = indexmap::IndexMap::new();
        out.insert(
            "eventtime".to_string(),
            extras_macros_templates::gcode_macro::Value::py(PyValue::Float(eventtime)),
        );
        out
    }
}
// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("neopixel fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping neopixel oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn pyvalue_to_record(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => n.inner().to_string().parse::<i64>().map(Value::Int).unwrap_or_else(|_| Value::BigInt(n.inner().to_string())),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::Str(s.clone()),
        PyValue::Bytes(b) => Value::Bytes(b.clone()),
        PyValue::List(items) => Value::List(items.iter().map(pyvalue_to_record).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyvalue_to_record).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter()
                .map(|(k, v)| (pyvalue_to_record(k), pyvalue_to_record(v)))
                .collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host:{h}")),
    }
}

fn record_for(label: &str) -> Record {
    Record {
        v: 1,
        unit: "neopixel".to_string(),
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

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = record_for(label);
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

fn updates_value(sends: &[(String, Vec<PyValue>, SendOpts)]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|(fmt, data, opts)| {
                Value::Tuple(vec![
                    Value::str(fmt),
                    pyvalue_to_record(&PyValue::List(data.clone())),
                    Value::Int(opts.minclock),
                    Value::Int(opts.reqclock),
                ])
            })
            .collect(),
    )
}

fn queries_value(sends: &[(String, Vec<PyValue>, QuerySendOpts)]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|(fmt, data, opts)| {
                Value::Tuple(vec![
                    Value::str(fmt),
                    pyvalue_to_record(&PyValue::List(data.clone())),
                    Value::Int(opts.minclock),
                    Value::Int(opts.reqclock),
                ])
            })
            .collect(),
    )
}

struct Harness {
    printer: Printer,
    fake_printer: FakePrinter,
    mcu: Arc<FakeMcu>,
    regs: MuxRegs,
}

// Builds both sides with the same typed config and start args.
#[allow(clippy::too_many_arguments)]
fn build_pair(
    py: &mut pyoracle::Oracle,
    vals_py: &str,
    vals_rust: Vec<(String, FakeVal)>,
    start_args: &[(String, Option<String>)],
    success_seq: Vec<bool>,
) -> (PrinterNeoPixel, Harness) {
    let mut start_map = indexmap::IndexMap::new();
    for (k, v) in start_args {
        let val = match v {
            Some(s) => PyValue::Str(s.clone()),
            None => PyValue::None,
        };
        start_map.insert(PyValue::Str(k.clone()), val);
    }
    let mcu = Arc::new(FakeMcu::new(success_seq.clone()));
    let mcu_dyn: Arc<dyn McuApi> = mcu.clone();
    let regs: MuxRegs = Arc::new(kos::sync::Mutex::new(Vec::new()));
    let gcode_obj: ObjectRef = Arc::new(FakeGCode { regs: regs.clone() });
    let pins_obj: ObjectRef = Arc::new(FakePins { mcu: mcu_dyn.clone() });
    let dt_obj: ObjectRef = Arc::new(FakeDisplayTemplates);
    let macro_obj: ObjectRef = Arc::new(FakeMacro { contexts: kos::sync::Mutex::new(Vec::new()) });
    let fake_printer = FakePrinter::new(
        vec![
            ("pins".to_string(), pins_obj),
            ("gcode".to_string(), gcode_obj),
            ("display_template".to_string(), dt_obj),
            ("gcode_macro".to_string(), macro_obj),
        ],
        PyValue::Dict(start_map),
    );
    let printer: Printer = Arc::new(fake_printer.clone());
    let mut map = indexmap::IndexMap::new();
    for (k, v) in vals_rust {
        map.insert(k, v);
    }
    let config: Config = Arc::new(FakeConfig {
        printer: printer.clone(),
        section: "neopixel test".to_string(),
        vals: map,
    });
    let succ_py = if success_seq.is_empty() {
        "None".to_string()
    } else {
        format!("[{}]", success_seq.iter().map(|b| if *b { "True" } else { "False" }).collect::<Vec<_>>().join(", "))
    };
    let start_py = {
        let parts: Vec<String> = start_args
            .iter()
            .map(|(k, v)| {
                let val = match v {
                    Some(s) => format!("'{s}'"),
                    None => "None".to_string(),
                };
                format!("'{k}': {val}")
            })
            .collect();
        format!("{{{}}}", parts.join(", "))
    };
    py.exec(&format!(
        "LOG_LINES.clear(); mcu = FakeMcu(success_seq={succ_py}); p = FakePrinter(mcu, start_args={start_py}); c = FakeConfig(p, {vals_py}); npi = neopixel.load_config_prefix(c)"
    ))
    .expect("py load_config_prefix");
    let harness = Harness {
        printer: printer.clone(),
        fake_printer: fake_printer.clone(),
        mcu: mcu.clone(),
        regs: regs.clone(),
    };
    // Rebuild Rust printer state shared with harness: PrinterNeoPixel clones
    // the printer handle, so fakes stay observable through Harness.
    let _ = &harness.fake_printer;
    let rust = PrinterNeoPixel::new(&config).expect("rust PrinterNeoPixel::new");
    // Replace mcu handle: new() used the same FakeMcu Arc (shared state).
    (rust, harness)
}

fn default_vals(pin: &str) -> Vec<(String, FakeVal)> {
    vec![("pin".to_string(), FakeVal::Str(pin.to_string()))]
}

fn py_vals(pin: &str, extra: &str) -> String {
    format!("{{'pin': '{pin}'{extra}}}")
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check("background", py.eval("neopixel.BACKGROUND_PRIORITY_CLOCK"), Value::Int(BACKGROUND_PRIORITY_CLOCK));
    check("bit_max", py.eval("neopixel.BIT_MAX_TIME"), Value::float(BIT_MAX_TIME));
    check("reset_min", py.eval("neopixel.RESET_MIN_TIME"), Value::float(RESET_MIN_TIME));
    check("max_mcu", py.eval("neopixel.MAX_MCU_SIZE"), Value::Int(MAX_MCU_SIZE as i64));
    for msg in [
        "config_neopixel oid=%d pin=%s data_size=%d",
        "neopixel_update oid=%c pos=%hu data=%*s",
        "neopixel_send oid=%c",
        "neopixel_result oid=%c success=%c",
    ] {
        let expr = format!("'{}'", msg);
        let expected = py.eval(&expr);
        check(&format!("mcu fmt {msg}"), expected, Value::str(msg));
    }
}

#[test]
fn factory_shape() {
    let factory = extras::neopixel::module_factory();
    assert!(factory.load_config.is_none(), "neopixel has no load_config");
    assert!(factory.load_config_prefix.is_some(), "neopixel exposes load_config_prefix");
    assert!(factory.add_early_printer_objects.is_none());
    assert!(factory.add_printer_objects.is_none());
}

#[test]
fn default_config_builds() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    for expr in [
        ("oid", "npi.oid"),
        ("pin", "npi.pin"),
        ("color_data", "bytes(npi.color_data)"),
        ("old_color_data", "bytes(npi.old_color_data)"),
    ] {
        let label = format!("default_config_builds {}", expr.0);
        let expected = py.eval(expr.1);
        let actual = match expr.0 {
            "oid" => Value::Int(rust.oid()),
            "pin" => Value::str(rust.pin()),
            "color_data" => Value::bytes(&rust.color_data()),
            _ => Value::bytes(&rust.old_color_data()),
        };
        check(&label, expected, actual);
    }
    let expected = py.eval("[e for (e, _) in p.handlers]");
    check("handlers", expected, Value::List(harness.fake_printer.handler_events().into_iter().map(|s| Value::str(&s)).collect()));
    let expected = py.eval("len(mcu.callbacks)");
    check("callbacks", expected, Value::Int(harness.mcu.state.lock().callbacks.len() as i64));
    let expected = py.eval("[(c, k, v, d) for (c, k, v, d) in p.gcode.mux]");
    check(
        "mux",
        expected,
        Value::List(
            harness.regs.lock().iter().map(|(c, k, v, d)| Value::Tuple(vec![Value::str(c), Value::str(k), Value::str(v), Value::str(d)])).collect(),
        ),
    );
}

#[test]
fn initial_colors_derive_bytes() {
    let Some(mut py) = oracle() else { return };
    let vals_rust = vec![
        ("pin".to_string(), FakeVal::Str("PA1".to_string())),
        ("initial_RED".to_string(), FakeVal::Float(1.0)),
        ("initial_GREEN".to_string(), FakeVal::Float(0.5)),
        ("initial_BLUE".to_string(), FakeVal::Float(0.0)),
        ("initial_WHITE".to_string(), FakeVal::Float(0.25)),
    ];
    let (rust, _h) = build_pair(
        &mut py,
        "{'pin': 'PA1', 'initial_RED': 1.0, 'initial_GREEN': 0.5, 'initial_BLUE': 0.0, 'initial_WHITE': 0.25}",
        vals_rust,
        &[],
        vec![true],
    );
    let expected = py.eval("bytes(npi.color_data)");
    check("initial bytes", expected, Value::bytes(&rust.color_data()));
    let expected = py.eval("npi.get_status()");
    let rust_status = <PrinterNeoPixel as PrinterObject>::get_status(&rust, 0.0)
        .unwrap_or(PyValue::None);
    // get_status(eventtime) passes eventtime through; both sides ignore it.
    let expected2 = py.eval("npi.get_status(0.0)");
    check("status", expected2, pyvalue_to_record(&rust_status));
    let _ = expected;
}

#[test]
fn explicit_chain_and_color_orders() {
    let Some(mut py) = oracle() else { return };
    let vals_rust = vec![
        ("pin".to_string(), FakeVal::Str("PA1".to_string())),
        ("chain_count".to_string(), FakeVal::Int(3)),
        ("color_order".to_string(), FakeVal::List(vec!["RGB".to_string(), "RGBW".to_string(), "BGR".to_string()])),
        ("initial_RED".to_string(), FakeVal::Float(0.2)),
        ("initial_GREEN".to_string(), FakeVal::Float(0.4)),
        ("initial_BLUE".to_string(), FakeVal::Float(0.6)),
        ("initial_WHITE".to_string(), FakeVal::Float(0.8)),
    ];
    let (rust, _h) = build_pair(
        &mut py,
        "{'pin': 'PA1', 'chain_count': 3, 'color_order': ['RGB', 'RGBW', 'BGR'], 'initial_RED': 0.2, 'initial_GREEN': 0.4, 'initial_BLUE': 0.6, 'initial_WHITE': 0.8}",
        vals_rust,
        &[],
        vec![true],
    );
    let expected = py.eval("bytes(npi.color_data)");
    check("chain bytes", expected, Value::bytes(&rust.color_data()));
    let expected = py.eval("npi.get_status()");
    let rust_status = <PrinterNeoPixel as PrinterObject>::get_status(&rust, 0.0).unwrap_or(PyValue::None);
    check("chain status", expected, pyvalue_to_record(&rust_status));
}

#[test]
fn single_entry_expands() {
    let Some(mut py) = oracle() else { return };
    let vals_rust = vec![
        ("pin".to_string(), FakeVal::Str("PA1".to_string())),
        ("chain_count".to_string(), FakeVal::Int(4)),
        ("color_order".to_string(), FakeVal::List(vec!["RGBW".to_string()])),
    ];
    let (rust, _h) = build_pair(
        &mut py,
        "{'pin': 'PA1', 'chain_count': 4, 'color_order': ['RGBW']}",
        vals_rust,
        &[],
        vec![true],
    );
    let expected = py.eval("bytes(npi.color_data)");
    check("expanded bytes", expected, Value::bytes(&rust.color_data()));
}

#[test]
fn build_config_mcu_command() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA0", ""), default_vals("PA0"), &[], vec![true]);
    let expected = py.eval("npi.build_config()");
    let actual = match rust.build_config() {
        Ok(()) => Value::Null,
        Err(e) => panic!("rust build_config: {}: {}", e.class_name(), e.message()),
    };
    check("build_config ret", expected, actual);
    let expected = py.eval("list(mcu.config_cmds)");
    check(
        "config_cmds",
        expected,
        Value::List(harness.mcu.state.lock().config_cmds.iter().map(|s| Value::str(s)).collect()),
    );
    let expected = py.eval("list(mcu.update_lookups)");
    check(
        "update_lookups",
        expected,
        Value::List(harness.mcu.state.lock().update_lookups.iter().map(|s| Value::str(s)).collect()),
    );
    let expected = py.eval("list(mcu.query_lookups)");
    check(
        "query_lookups",
        expected,
        Value::List(
            harness
                .mcu
                .state
                .lock()
                .query_lookups
                .iter()
                .map(|(a, b, o)| Value::Tuple(vec![
                    Value::str(a),
                    Value::str(b),
                    match o {
                        Some(v) => Value::Int(*v),
                        None => Value::Null,
                    },
                ]))
                .collect(),
        ),
    );
}

#[test]
fn send_data_no_change_no_traffic() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    let expected = py.eval("npi.build_config()");
    check("setup build", expected, Value::Null);
    rust.build_config().expect("rust build");
    // Initial old_data is color_data ^ 1, so the first send DOES transmit.
    // First align old to new, then verify a no-change send is silent.
    py.eval("npi.send_data()").expect("py first send");
    rust.send_data(None).expect("rust first send");
    harness.mcu.state.lock().sent_updates.clear();
    harness.mcu.state.lock().sent_queries.clear();
    py.exec("mcu.sent_updates.clear(); mcu.sent_queries.clear()").expect("py clear");
    let expected = py.eval("npi.send_data()");
    check("silent ret", expected, {
        match rust.send_data(None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_updates)");
    check("silent updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
    let expected = py.eval("list(mcu.sent_queries)");
    check("silent queries", expected, queries_value(&harness.mcu.state.lock().sent_queries));
}

#[test]
fn send_data_diff_batching() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    let expected = py.eval("npi.build_config()");
    check("setup build", expected, Value::Null);
    rust.build_config().expect("rust build");
    // One GRB LED defaults to black; set two bytes via a crafted state.
    let py_state = "[(1.0, 0.0, 0.5, 0.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[1.0, 0.0, 0.5, 0.0]];
    let expected = py.eval(&format!("npi.update_color_data({py_state})"));
    check("py update", expected, {
        match rust.update_color_data(&rust_state) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust update: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("bytes(npi.color_data)");
    check("color_data", expected, Value::bytes(&rust.color_data()));
    let expected = py.eval("npi.send_data()");
    check("send ret", expected, {
        match rust.send_data(None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_updates)");
    check("updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
    let expected = py.eval("list(mcu.sent_queries)");
    check("queries", expected, queries_value(&harness.mcu.state.lock().sent_queries));
    let expected = py.eval("bytes(npi.old_color_data)");
    check("old_color_data", expected, Value::bytes(&rust.old_color_data()));
}

#[test]
fn send_data_batches_close_changes() {
    let Some(mut py) = oracle() else { return };
    let vals_rust = vec![
        ("pin".to_string(), FakeVal::Str("PA1".to_string())),
        ("chain_count".to_string(), FakeVal::Int(6)),
        ("color_order".to_string(), FakeVal::List(vec!["RGB".to_string()])),
    ];
    let (rust, harness) = build_pair(
        &mut py,
        "{'pin': 'PA1', 'chain_count': 6, 'color_order': ['RGB']}",
        vals_rust,
        &[],
        vec![true],
    );
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    // Change bytes 0 and 3 (gap 3 -> merges) and byte 12 (gap 9 from 3 ->
    // still merges because pos+5 >= nextpos is false (3+5 < 12), so it must
    // stay split). This exercises both branches of the batching rule.
    let py_state = "[(1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)]";
    let rust_state: Vec<[f64; 4]> = vec![
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
    ];
    let expected = py.eval(&format!("npi.update_color_data({py_state})"));
    check("py update", expected, {
        match rust.update_color_data(&rust_state) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust update: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("npi.send_data()");
    check("send ret", expected, {
        match rust.send_data(None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_updates)");
    check("batched updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
}

static RUST_LOGS: std::sync::LazyLock<kos::sync::Mutex<Vec<String>>> =
    std::sync::LazyLock::new(|| kos::sync::Mutex::new(Vec::new()));

fn install_log_sink() {
    let _ = klippy_api::log::set_sink(Box::new(|rec| {
        RUST_LOGS.lock().push(rec.message.to_string());
    }));
}

#[test]
fn send_data_retries_then_logs_on_failure() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![false]);
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    let py_state = "[(1.0, 0.0, 0.0, 0.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[1.0, 0.0, 0.0, 0.0]];
    let expected = py.eval(&format!("npi.update_color_data({py_state})"));
    check("py update", expected, {
        match rust.update_color_data(&rust_state) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust update: {}: {}", e.class_name(), e.message()),
        }
    });
    install_log_sink();
    RUST_LOGS.lock().clear();
    let expected = py.eval("npi.send_data()");
    check("send ret", expected, {
        match rust.send_data(None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_queries)");
    check("8 queries", expected, queries_value(&harness.mcu.state.lock().sent_queries));
    let expected = py.eval("list(LOG_LINES)");
    check(
        "log lines",
        expected,
        Value::List(RUST_LOGS.lock().iter().map(|s| Value::str(s)).collect()),
    );
}

#[test]
fn send_data_debugoutput_skips_query() {
    let Some(mut py) = oracle() else { return };
    let start = [("debugoutput".to_string(), Some("/tmp/out".to_string()))];
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &start, vec![true]);
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    let py_state = "[(0.0, 1.0, 0.0, 0.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[0.0, 1.0, 0.0, 0.0]];
    py.eval(&format!("npi.update_color_data({py_state})")).expect("py update");
    rust.update_color_data(&rust_state).expect("rust update");
    let expected = py.eval("npi.send_data()");
    check("send ret", expected, {
        match rust.send_data(None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_updates)");
    check("debug updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
    let expected = py.eval("list(mcu.sent_queries)");
    check("debug queries", expected, queries_value(&harness.mcu.state.lock().sent_queries));
    let expected = py.eval("bytes(npi.old_color_data)");
    check("debug old", expected, Value::bytes(&rust.old_color_data()));
}

#[test]
fn send_data_print_time_minclock() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    let py_state = "[(0.0, 0.0, 1.0, 0.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[0.0, 0.0, 1.0, 0.0]];
    py.eval(&format!("npi.update_color_data({py_state})")).expect("py update");
    rust.update_color_data(&rust_state).expect("rust update");
    let expected = py.eval("npi.send_data(1.5)");
    check("send ret", expected, {
        match rust.send_data(Some(1.5)) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust send: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("list(mcu.sent_queries)");
    check("minclock queries", expected, queries_value(&harness.mcu.state.lock().sent_queries));
}

#[test]
fn send_data_before_build_config() {
    let Some(mut py) = oracle() else { return };
    // No build_config: initial old_data differs (color_data ^ 1), so a send
    // with changed bytes must raise AttributeError, while a send with no
    // changes is silent.
    let (rust, _h) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    let py_state = "[(1.0, 1.0, 1.0, 1.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[1.0, 1.0, 1.0, 1.0]];
    py.exec(&format!("npi.update_color_data({py_state})")).expect("py update");
    let expected = py.eval("npi.send_data()");
    let actual = rust.update_color_data(&rust_state).and_then(|()| rust.send_data(None));
    match (expected, actual) {
        (Err(oracle_exc), Err(rust_err)) => {
            assert_eq!(rust_err.class_name(), oracle_exc.class_name.as_str(), "attribute class");
            assert_eq!(rust_err.message(), oracle_exc.message.as_str(), "attribute message");
        }
        (Ok(v), Ok(())) => panic!("both should fail: {v:?}"),
        (Ok(v), Err(e)) => panic!("oracle ok {v:?} but rust {e:?}"),
        (Err(e), Ok(())) => panic!("oracle {} but rust ok", e.message),
    }
}

#[test]
fn update_leds_end_to_end() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    let py_state = "[(0.5, 0.25, 0.75, 1.0)]";
    let rust_state: Vec<[f64; 4]> = vec![[0.5, 0.25, 0.75, 1.0]];
    let expected = py.eval(&format!("npi.update_leds({py_state}, None)"));
    check("update ret", expected, {
        match rust.update_leds(&rust_state, None) {
            Ok(()) => Value::Null,
            Err(e) => panic!("rust update_leds: {}: {}", e.class_name(), e.message()),
        }
    });
    let expected = py.eval("bytes(npi.color_data)");
    check("e2e bytes", expected, Value::bytes(&rust.color_data()));
    let expected = py.eval("list(mcu.sent_updates)");
    check("e2e updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
}

#[test]
fn klippy_connect_event_sends() {
    let Some(mut py) = oracle() else { return };
    let (rust, harness) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    py.eval("npi.build_config()").expect("py build");
    rust.build_config().expect("rust build");
    let expected = py.eval("p.send_event('klippy:connect')");
    let rust_results = harness
        .printer
        .send_event("klippy:connect", &[])
        .expect("rust connect");
    check(
        "connect rust",
        expected,
        Value::List(rust_results.iter().map(pyvalue_to_record).collect()),
    );
    let expected = py.eval("list(mcu.sent_updates)");
    check("connect updates", expected, updates_value(&harness.mcu.state.lock().sent_updates));
}

#[test]
fn init_missing_pin() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {}))");
    let result = match PrinterNeoPixel::new(&{
        let mcu: Arc<dyn McuApi> = Arc::new(FakeMcu::new(vec![true]));
        let gcode_obj: ObjectRef = Arc::new(FakeGCode { regs: Arc::new(kos::sync::Mutex::new(Vec::new())) });
        let pins_obj: ObjectRef = Arc::new(FakePins { mcu: mcu.clone() });
        let dt_obj: ObjectRef = Arc::new(FakeDisplayTemplates);
        let macro_obj: ObjectRef = Arc::new(FakeMacro { contexts: kos::sync::Mutex::new(Vec::new()) });
        let fake_printer = FakePrinter::new(
        vec![("pins".to_string(), pins_obj), ("gcode".to_string(), gcode_obj), ("display_template".to_string(), dt_obj), ("gcode_macro".to_string(), macro_obj)],
            PyValue::Dict(indexmap::IndexMap::new()),
        );
        let printer: Printer = Arc::new(fake_printer);
        Arc::new(FakeConfig { printer, section: "neopixel test".to_string(), vals: indexmap::IndexMap::new() }) as Config
    }) {
        Ok(_) => panic!("missing pin must fail"),
        Err(e) => e,
    };
    check_err("missing pin", expected, &result);
}

#[test]
fn init_color_order_mismatch() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {'pin': 'PA1', 'chain_count': 3, 'color_order': ['GRB', 'RGB']}))");
    let result = match build_new_for_init_error(&[("pin", FakeVal::Str("PA1".into())), ("chain_count", FakeVal::Int(3)), ("color_order", FakeVal::List(vec!["GRB".into(), "RGB".into()]))]) {
        Ok(_) => panic!("mismatch must fail"),
        Err(e) => e,
    };
    check_err("mismatch", expected, &result);
}

fn build_new_for_init_error(vals: &[(&str, FakeVal)]) -> KResult<PrinterNeoPixel> {
    let mcu: Arc<dyn McuApi> = Arc::new(FakeMcu::new(vec![true]));
    let gcode_obj: ObjectRef = Arc::new(FakeGCode { regs: Arc::new(kos::sync::Mutex::new(Vec::new())) });
    let pins_obj: ObjectRef = Arc::new(FakePins { mcu: mcu.clone() });
    let dt_obj: ObjectRef = Arc::new(FakeDisplayTemplates);
    let macro_obj: ObjectRef = Arc::new(FakeMacro { contexts: kos::sync::Mutex::new(Vec::new()) });
    let fake_printer = FakePrinter::new(
        vec![("pins".to_string(), pins_obj), ("gcode".to_string(), gcode_obj), ("display_template".to_string(), dt_obj), ("gcode_macro".to_string(), macro_obj)],
        PyValue::Dict(indexmap::IndexMap::new()),
    );
    let printer: Printer = Arc::new(fake_printer);
    let mut map = indexmap::IndexMap::new();
    for (k, v) in vals {
        map.insert(k.to_string(), v.clone());
    }
    let config: Config = Arc::new(FakeConfig { printer, section: "neopixel test".to_string(), vals: map });
    PrinterNeoPixel::new(&config)
}

#[test]
fn init_invalid_color_order() {
    let Some(mut py) = oracle() else { return };
    for bad in ["", "GR", "RGBRGB", "RRR", "RGBWXYZ", "rgb", "GRB "] {
        let expected = py.eval(&format!(
            "neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {{'pin': 'PA1', 'color_order': ['{bad}']}}))"
        ));
        let result = match build_new_for_init_error(&[("pin", FakeVal::Str("PA1".into())), ("color_order", FakeVal::List(vec![bad.to_string()]))]) {
            Ok(_) => panic!("invalid {bad:?} must fail"),
            Err(e) => e,
        };
        check_err(&format!("invalid {bad:?}"), expected, &result);
    }
}

#[test]
fn init_chain_too_long() {
    let Some(mut py) = oracle() else { return };
    // 167 RGBW LEDs -> 668 bytes > 500.
    let expected = py.eval("neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {'pin': 'PA1', 'chain_count': 167, 'color_order': ['RGBW']}))");
    let result = match build_new_for_init_error(&[("pin", FakeVal::Str("PA1".into())), ("chain_count", FakeVal::Int(167)), ("color_order", FakeVal::List(vec!["RGBW".into()]))]) {
        Ok(_) => panic!("chain too long must fail"),
        Err(e) => e,
    };
    check_err("chain too long", expected, &result);
}

#[test]
fn init_chain_count_minval() {
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {'pin': 'PA1', 'chain_count': 0}))");
    let result = match build_new_for_init_error(&[("pin", FakeVal::Str("PA1".into())), ("chain_count", FakeVal::Int(0))]) {
        Ok(_) => panic!("minval must fail"),
        Err(e) => e,
    };
    check_err("chain minval", expected, &result);
}

#[test]
fn byte_conversion_edges() {
    let Some(mut py) = oracle() else { return };
    let (rust, _h) = build_pair(&mut py, &py_vals("PA1", ""), default_vals("PA1"), &[], vec![true]);
    let cases: Vec<([f64; 4], &str)> = vec![
        ([0.0, 0.0, 0.0, 0.0], "[(0.0, 0.0, 0.0, 0.0)]"),
        ([1.0, 1.0, 1.0, 1.0], "[(1.0, 1.0, 1.0, 1.0)]"),
        ([0.5, 0.5, 0.5, 0.5], "[(0.5, 0.5, 0.5, 0.5)]"),
        ([0.2, 0.4, 0.6, 0.8], "[(0.2, 0.4, 0.6, 0.8)]"),
        ([0.1, 0.9, 0.01, 0.99], "[(0.1, 0.9, 0.01, 0.99)]"),
    ];
    for (i, (rgba, py_lit)) in cases.iter().enumerate() {
        py.exec(&format!("npi.update_color_data({py_lit})")).expect("py update");
        let expected = py.eval("bytes(npi.color_data)");
        let actual = match rust.update_color_data(std::slice::from_ref(rgba)) {
            Ok(()) => Value::bytes(&rust.color_data()),
            Err(e) => panic!("case {i}: rust {}: {}", e.class_name(), e.message()),
        };
        check(&format!("byte case {i}"), expected, actual);
    }
}

#[test]
fn load_config_prefix_factory_builds() {
    let Some(mut py) = oracle() else { return };
    let factory = extras::neopixel::module_factory();
    let load_config_prefix = factory.load_config_prefix.expect("load_config_prefix");
    let mcu: Arc<dyn McuApi> = Arc::new(FakeMcu::new(vec![true]));
    let gcode_obj: ObjectRef = Arc::new(FakeGCode { regs: Arc::new(kos::sync::Mutex::new(Vec::new())) });
    let pins_obj: ObjectRef = Arc::new(FakePins { mcu: mcu.clone() });
    let dt_obj: ObjectRef = Arc::new(FakeDisplayTemplates);
    let macro_obj: ObjectRef = Arc::new(FakeMacro { contexts: kos::sync::Mutex::new(Vec::new()) });
    let fake_printer = FakePrinter::new(
        vec![("pins".to_string(), pins_obj), ("gcode".to_string(), gcode_obj), ("display_template".to_string(), dt_obj), ("gcode_macro".to_string(), macro_obj)],
        PyValue::Dict(indexmap::IndexMap::new()),
    );
    let printer: Printer = Arc::new(fake_printer);
    let mut map = indexmap::IndexMap::new();
    map.insert("pin".to_string(), FakeVal::Str("PA1".to_string()));
    let config: Config = Arc::new(FakeConfig { printer, section: "neopixel test".to_string(), vals: map });
    let obj: ObjectRef = load_config_prefix(&config).expect("factory").expect("object");
    let rust: Arc<PrinterNeoPixel> =
        klippy_api::downcast_arc(&obj).expect("downcast to PrinterNeoPixel");
    let expected = py.eval("neopixel.load_config_prefix(FakeConfig(FakePrinter(FakeMcu()), {'pin': 'PA1'})).oid");
    check("factory oid", expected, Value::Int(rust.oid()));
}
