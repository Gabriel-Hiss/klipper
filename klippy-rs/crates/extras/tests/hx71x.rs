//! Parity tests for `extras::hx71x` (port of `klippy/extras/hx71x.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The Python side runs the real
//! `extras.hx71x`, `extras.bulk_sensor` and `configfile` modules over
//! minimal fakes of the printer, reactor, MCU and pins. The Rust side runs
//! the port and the real `extras::bulk_sensor` port over the same fakes.
//!
//! Both sides append one text line per observable effect to a trace: MCU
//! config commands, oid allocation, command lookups, commands sent (format
//! and argument values), status queries, the `logging` records, raised errors
//! (class and message), and result values (`repr`, so floats compare by their
//! exact shortest spelling). The two traces must be identical.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit.

use extras::hx71x::{
    module_factory, HX71xBase, HX711, HX717, HX71X_SENSOR_TYPES, SAMPLE_ERROR_DESYNC,
    SAMPLE_ERROR_LONG_READ, UPDATE_INTERVAL,
};
use extras::trigger_analog::TriggerAnalogSensor;
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log::Level;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams,
    PinResolver, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::str_repr;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::cell::RefCell;
use std::sync::{Arc, Once};

// ---------------------------------------------------------------------------
// Trace shared by every Rust fake (the Python side appends to `LOG`)
// ---------------------------------------------------------------------------

thread_local! {
    static TRACE: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn tr(line: String) {
    TRACE.with(|t| t.borrow_mut().push(line));
}

fn take_trace() -> Vec<String> {
    TRACE.with(|t| std::mem::take(&mut *t.borrow_mut()))
}

/// Every `logging` record the port and the batch helper emit is traced.
fn install_log_sink() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            let level = match rec.level {
                Level::Debug => "debug",
                Level::Info => "info",
                Level::Warning => "warning",
                Level::Error if rec.exc_text.is_some() => "exception",
                Level::Error => "error",
                Level::Critical => "critical",
            };
            tr(format!("{level}|{}", rec.message));
        }));
    });
}

fn exc(e: &KlippyError) {
    tr(format!("exc|{}|{}", e.class_name(), e.message()));
}

/// `guard(...)`: log the error, return the value.
fn guard<T>(r: KResult<T>) -> Option<T> {
    match r {
        Ok(v) => Some(v),
        Err(e) => {
            exc(&e);
            None
        }
    }
}

fn res(label: &str, v: &PyValue) {
    tr(format!("res|{label}|{}", v.repr()));
}

fn pb(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn pstr(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn list_repr(v: &[PyValue]) -> String {
    format!("[{}]", v.iter().map(PyValue::repr).collect::<Vec<_>>().join(", "))
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, configparser, struct
import gcode as gcode_mod
import configfile as configfile_mod

LOG = []

def take_log():
    out = list(LOG)
    del LOG[:]
    return out

class _FakeLogging:
    @staticmethod
    def _emit(level, msg, args):
        text = (msg % args) if args else msg
        LOG.append("%s|%s" % (level, text))
    @staticmethod
    def info(msg, *args):
        _FakeLogging._emit("info", msg, args)
    @staticmethod
    def error(msg, *args):
        _FakeLogging._emit("error", msg, args)
    @staticmethod
    def exception(msg, *args):
        _FakeLogging._emit("exception", msg, args)
    @staticmethod
    def warning(msg, *args):
        _FakeLogging._emit("warning", msg, args)
    @staticmethod
    def debug(msg, *args):
        _FakeLogging._emit("debug", msg, args)

import extras.bulk_sensor as bulk_mod
import extras.hx71x as hx_mod

for _m in (bulk_mod, hx_mod):
    _m.logging = _FakeLogging

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

class FakeTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime

class FakeReactor:
    NEVER = 9999999999999999.
    def monotonic(self):
        return 1000.
    def register_timer(self, callback, waketime=NEVER):
        return FakeTimer(callback, waketime)
    def unregister_timer(self, timer):
        timer.waketime = self.NEVER

class FakePrinter:
    config_error = configparser.Error
    command_error = gcode_mod.CommandError
    def __init__(self):
        self.objects = {}
        self.reactor = FakeReactor()
        self.shutdown = False
    def get_reactor(self):
        return self.reactor
    def is_shutdown(self):
        return self.shutdown
    def lookup_object(self, name, default=configfile_mod.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile_mod.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default

def norm(data):
    return [bytes(p) if isinstance(p, (list, bytearray)) else p for p in data]

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        LOG.append("send|%s|%r|%d|%d" % (self.fmt, norm(list(data)), minclock, reqclock))
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        LOG.append("wait_ack|%s|%r|%d|%d" % (self.fmt, norm(list(data)), minclock, reqclock))

class FakeQuery:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0, retry=True):
        d = norm(list(data))
        LOG.append("qsend|%s|%r|%d|%d|%s" % (self.fmt, d, minclock, reqclock, retry))
        if self.fmt.startswith("query_hx71x_status"):
            return dict(self.mcu.status)
        raise Exception("unexpected query %s" % (self.fmt,))

class FakeMcu:
    def __init__(self, printer, name):
        self.printer = printer
        self.name = name
        self.next_oid = 0
        self.config_cbs = []
        self.serial_cbs = []
        self.status = {}
    def get_name(self):
        return self.name
    def get_printer(self):
        return self.printer
    def create_oid(self):
        self.next_oid += 1
        LOG.append("create_oid|%s|%d" % (self.name, self.next_oid))
        return self.next_oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        LOG.append("config_cmd|%s|%r|%r|%r" % (self.name, cmd, is_init, on_restart))
    def alloc_command_queue(self):
        return 7
    def register_config_callback(self, cb):
        self.config_cbs.append(cb)
        LOG.append("config_cb|%s" % (self.name,))
    def lookup_command(self, fmt, cq=None):
        LOG.append("lookup_command|%s|%s|%r" % (self.name, fmt, cq))
        return FakeCmd(self, fmt)
    def lookup_query_command(self, fmt, resp, oid=None, cq=None, is_async=False):
        LOG.append("lookup_query|%s|%s|%s|%r|%r" % (self.name, fmt, resp, oid, cq))
        return FakeQuery(self, fmt)
    def register_serial_response(self, cb, msg, oid=None):
        LOG.append("serial_response|%s|%s|%r" % (self.name, msg, oid))
        self.serial_cbs.append(cb)
    def seconds_to_clock(self, t):
        return int(t * 1000000.)
    def clock32_to_clock64(self, c):
        return c
    def clock_to_print_time(self, c):
        return c / 1000000.
    def run_config_cbs(self):
        for cb in list(self.config_cbs):
            cb()

class FakePins:
    error = _PinsError
    def __init__(self):
        self.chips = {}
        self.active = {}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        desc = pin_desc.strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        if chip_name not in self.chips:
            raise self.error("Unknown pin chip name '%s'" % (chip_name,))
        share_name = "%s:%s" % (chip_name, pin)
        if share_name in self.active:
            return self.active[share_name]
        params = {'chip': self.chips[chip_name], 'chip_name': chip_name,
                  'pin': pin, 'invert': 0, 'pullup': 0, 'share_type': share_type}
        self.active[share_name] = params
        return params

P = MCU = AUX = PINS = None
S = {}
CLIENT_OK = [True]

def make_world():
    global P, MCU, AUX, PINS, S
    S = {}
    CLIENT_OK[0] = True
    P = FakePrinter()
    MCU = FakeMcu(P, "mcu")
    AUX = FakeMcu(P, "aux")
    PINS = FakePins()
    PINS.chips = {"mcu": MCU, "aux": AUX}
    P.objects = {"pins": PINS, "mcu": MCU, "mcu aux": AUX}
    del LOG[:]

def make_config(section, values):
    fc = configparser.RawConfigParser()
    fc.add_section(section)
    for k, v in values:
        fc.set(section, k, v)
    return configfile_mod.ConfigWrapper(P, fc, {}, section)

def guard(fn, *args, **kw):
    try:
        return fn(*args, **kw)
    except Exception as e:
        LOG.append("exc|%s|%s" % (type(e).__name__, e))
        return None

def res(label, v):
    LOG.append("res|%s|%r" % (label, v))

def build(kind, section, values):
    cfg = make_config(section, values)
    s = hx_mod.HX71X_SENSOR_TYPES[kind](cfg)
    S[section] = s
    return s

def run_cbs():
    guard(MCU.run_config_cbs)
    guard(AUX.run_config_cbs)

def pack(values):
    return struct.pack('<%di' % len(values), *values)

def st(clock, seq, buffered, ovf):
    MCU.status = {"clock": clock, "query_ticks": 40, "next_sequence": seq,
                  "buffered": buffered, "possible_overflows": ovf}

def feed(mcu, seq, data):
    for cb in mcu.serial_cbs:
        cb({'sequence': seq, 'data': bytes(data)})

def client(msg):
    LOG.append("client|%r" % (msg,))
    return CLIENT_OK[0]

def snap(section, label):
    s = S[section]
    res(label + ".snap", (s.last_error_count, s.consecutive_fails,
                          s.batch_bulk.is_started, s.get_status(0.)))
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    reactor: Reactor,
    shutdown: GCell<bool>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(Vec::new()),
            reactor: Reactor::new(false),
            shutdown: GCell::new(false),
        })
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".to_string(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        self.shutdown.get()
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::Config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by hx71x")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        tr(format!("shutdown|{msg}"));
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().push((event.to_string(), callback));
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("send_event is not used by hx71x")
    }
    fn request_exit(&self, _result: &str) {}
}

/// A `ConfigWrapper` over a list of `(option, value)` strings, with the
/// `configfile.py` semantics of the getters `hx71x` calls.
struct FakeConfig {
    printer: Printer,
    section: String,
    values: Vec<(String, String)>,
}

impl FakeConfig {
    fn config(printer: Printer, section: &str, values: &[(&str, &str)]) -> Config {
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            values: values.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect(),
        })
    }
    fn value(&self, option: &str) -> Option<&str> {
        self.values.iter().find(|(k, _)| k == option).map(|(_, v)| v.as_str())
    }
    fn missing(&self, option: &str) -> KlippyError {
        KlippyError::Config(format!(
            "Option '{option}' in section '{}' must be specified",
            self.section
        ))
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
        match self.value(option) {
            Some(v) => Ok(Some(v.to_string())),
            None => match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let Some(raw) = self.value(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let Ok(v) = raw.trim().parse::<i64>() else {
            return Err(KlippyError::Config(format!(
                "Unable to parse option '{option}' in section '{}'",
                self.section
            )));
        };
        Ok(Some(v))
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by hx71x")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by hx71x")
    }
    /// `configfile.py:76` for the two mappings `hx71x` passes: an `int` keyed
    /// one reads the option with `getint`, a `str` keyed one with `get`.
    fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        let PyValue::Dict(map) = choices else { panic!("choices are a dict") };
        let c = if matches!(map.keys().next(), Some(PyValue::Int(_))) {
            let default = match opts.default {
                None => None,
                Some(Some(PyValue::Int(n))) => {
                    Some(Some(i64::try_from(&n.0).expect("small default")))
                }
                other => panic!("unexpected int choice default {other:?}"),
            };
            let v = self.getint(option, GetIntOpts { default, ..GetIntOpts::default() })?;
            int(v.expect("getint yields a value"))
        } else {
            let default = match opts.default {
                None => None,
                Some(Some(PyValue::Str(s))) => Some(Some(s)),
                other => panic!("unexpected str choice default {other:?}"),
            };
            let v = self.get(option, GetStrOpts { default, note_valid: true })?;
            PyValue::Str(v.expect("get yields a value"))
        };
        match map.get(&c) {
            Some(v) => Ok(v.clone()),
            None => Err(KlippyError::Config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                c.str(),
                self.section
            ))),
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by hx71x")
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        panic!("getlist is not used by hx71x")
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by hx71x")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by hx71x")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by hx71x")
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

/// The bulk status the `query_hx71x_status` command answers with.
struct ChipState {
    status: GCell<Vec<(String, i64)>>,
}

fn status(clock: i64, seq: i64, buffered: i64, ovf: i64) -> Vec<(String, i64)> {
    [
        ("clock", clock),
        ("query_ticks", 40),
        ("next_sequence", seq),
        ("buffered", buffered),
        ("possible_overflows", ovf),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), *v))
    .collect()
}

struct FakeCmd {
    fmt: String,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        tr(format!("send|{}|{}|{}|{}", self.fmt, list_repr(&data), opts.minclock, opts.reqclock));
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        tr(format!(
            "wait_ack|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock
        ));
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQuery {
    fmt: String,
    chip: Arc<ChipState>,
}

impl CommandQueryWrapperApi for FakeQuery {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        tr(format!(
            "qsend|{}|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock,
            pb(opts.retry)
        ));
        if self.fmt.starts_with("query_hx71x_status") {
            let mut d = IndexMap::new();
            for (k, v) in self.chip.status.borrow().iter() {
                d.insert(pstr(k), int(*v));
            }
            return Ok(PyValue::Dict(d));
        }
        panic!("unexpected query {}", self.fmt)
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by hx71x")
    }
}

struct FakeAsync;

impl AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    name: String,
    printer: GCell<Option<Printer>>,
    next_oid: GCell<i64>,
    config_cbs: GCell<Vec<ConfigCallback>>,
    serial_cbs: GCell<Vec<SerialResponseCallback>>,
    chip: Arc<ChipState>,
}

impl FakeMcu {
    fn new(name: &str) -> Arc<FakeMcu> {
        Arc::new(FakeMcu {
            name: name.to_string(),
            printer: GCell::new(None),
            next_oid: GCell::new(0),
            config_cbs: GCell::new(Vec::new()),
            serial_cbs: GCell::new(Vec::new()),
            chip: Arc::new(ChipState { status: GCell::new(Vec::new()) }),
        })
    }
    fn run_config_cbs(&self) -> KResult<()> {
        let mut cbs = std::mem::take(&mut *self.config_cbs.borrow_mut());
        let mut result = Ok(());
        for cb in cbs.iter_mut() {
            if let Err(e) = cb() {
                result = Err(e);
                break;
            }
        }
        let mut current = self.config_cbs.borrow_mut();
        cbs.extend(current.drain(..));
        *current = cbs;
        result
    }
    fn feed(&self, seq: i64, data: &[u8]) {
        let cbs: Vec<SerialResponseCallback> = self.serial_cbs.borrow().clone();
        for cb in cbs {
            let mut d = IndexMap::new();
            d.insert(pstr("sequence"), int(seq));
            d.insert(pstr("data"), PyValue::Bytes(data.to_vec()));
            cb(PyValue::Dict(d));
        }
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.printer.borrow().clone().expect("printer set")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin is not used by hx71x")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut v = self.next_oid.borrow_mut();
        *v += 1;
        tr(format!("create_oid|{}|{}", self.name, *v));
        Ok(*v)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.config_cbs.borrow_mut().push(cb);
        tr(format!("config_cb|{}", self.name));
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        tr(format!(
            "config_cmd|{}|{}|{}|{}",
            self.name,
            str_repr(cmd),
            pb(opts.is_init),
            pb(opts.on_restart)
        ));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        (time * 1000000.) as i64
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(
        &self,
        msgformat: &str,
        cq: Option<CommandQueue>,
    ) -> KResult<CommandWrapper> {
        tr(format!(
            "lookup_command|{}|{}|{}",
            self.name,
            msgformat,
            cq.map_or("None".to_string(), |c| c.0.to_string())
        ));
        Ok(Arc::new(FakeCmd { fmt: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        tr(format!(
            "lookup_query|{}|{}|{}|{}|{}",
            self.name,
            msgformat,
            respformat,
            opts.oid.map_or("None".to_string(), |o| o.to_string()),
            opts.cq.map_or("None".to_string(), |c| c.0.to_string())
        ));
        Ok(Arc::new(FakeQuery { fmt: msgformat.to_string(), chip: self.chip.clone() }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        tr(format!(
            "serial_response|{}|{}|{}",
            self.name,
            msg,
            oid.map_or("None".to_string(), |o| o.to_string())
        ));
        self.serial_cbs.borrow_mut().push(cb);
        Arc::new(FakeAsync)
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
        panic!("get_constant_float is not used by hx71x")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by hx71x")
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000000.
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by hx71x")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by hx71x")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::None
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (false, String::new())
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by hx71x")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins {
    chips: GCell<IndexMap<String, Mcu>>,
    active: GCell<IndexMap<String, PinParams>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by hx71x")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc),
            Some((c, p)) => (c.trim(), p.trim()),
        };
        let chip = self.chips.borrow().get(chip_name).cloned();
        let Some(chip) = chip else {
            return Err(KlippyError::Other {
                class_name: "error".to_string(),
                message: format!("Unknown pin chip name '{chip_name}'"),
            });
        };
        let share_name = format!("{chip_name}:{pin}");
        if let Some(existing) = self.active.borrow().get(&share_name) {
            return Ok(existing.clone());
        }
        let params = PinParams {
            chip,
            chip_name: chip_name.to_string(),
            pin: pin.to_string(),
            invert: 0,
            pullup: 0,
            share_type: opts.share_type,
        };
        self.active.borrow_mut().insert(share_name, params.clone());
        Ok(params)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin is not used by hx71x")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by hx71x")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used by hx71x")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by hx71x")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by hx71x")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

// ---------------------------------------------------------------------------
// World: the Rust mirror of `make_world()`
// ---------------------------------------------------------------------------

struct Rig {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    aux: Arc<FakeMcu>,
    sensors: GCell<IndexMap<String, Arc<HX71xBase>>>,
    client_ok: Arc<GCell<bool>>,
}

impl Rig {
    fn new() -> Rig {
        install_log_sink();
        take_trace();
        let printer = FakePrinter::new();
        let printer_api: Printer = printer.clone();
        let mcu = FakeMcu::new("mcu");
        let aux = FakeMcu::new("aux");
        *mcu.printer.borrow_mut() = Some(printer_api.clone());
        *aux.printer.borrow_mut() = Some(printer_api.clone());
        let pins = Arc::new(FakePins {
            chips: GCell::new(IndexMap::new()),
            active: GCell::new(IndexMap::new()),
        });
        pins.chips.borrow_mut().insert("mcu".to_string(), mcu.clone());
        pins.chips.borrow_mut().insert("aux".to_string(), aux.clone());
        printer.add_object("pins", pins).expect("pins");
        printer.add_object("mcu", mcu.clone()).expect("mcu");
        printer.add_object("mcu aux", aux.clone()).expect("mcu aux");
        take_trace();
        Rig {
            printer,
            printer_api,
            mcu,
            aux,
            sensors: GCell::new(IndexMap::new()),
            client_ok: Arc::new(GCell::new(true)),
        }
    }

    fn config(&self, section: &str, values: &[(&str, &str)]) -> Config {
        FakeConfig::config(self.printer_api.clone(), section, values)
    }

    /// `guard(build, kind, section, values)`.
    fn build(
        &self,
        kind: &str,
        section: &str,
        values: &[(&str, &str)],
    ) -> Option<Arc<HX71xBase>> {
        let cfg = self.config(section, values);
        let ctor = HX71X_SENSOR_TYPES
            .iter()
            .find(|(name, _)| *name == kind)
            .expect("sensor kind")
            .1;
        match ctor(&cfg) {
            Ok(sensor) => {
                self.sensors.borrow_mut().insert(section.to_string(), sensor.clone());
                Some(sensor)
            }
            Err(e) => {
                exc(&e);
                None
            }
        }
    }

    fn sensor(&self, section: &str) -> Arc<HX71xBase> {
        self.sensors.borrow().get(section).cloned().expect("sensor built")
    }

    fn run_cbs(&self) {
        guard(self.mcu.run_config_cbs());
        guard(self.aux.run_config_cbs());
    }

    /// `client(msg)` of the Python setup.
    fn client(&self) -> extras::bulk_sensor::ClientCallback {
        let ok = self.client_ok.clone();
        Box::new(move |msg| {
            tr(format!("client|{}", msg.repr()));
            Ok(ok.get())
        })
    }
}

fn thin<T: ?Sized>(a: &Arc<T>) -> *const () {
    Arc::as_ptr(a) as *const ()
}

fn py_values(values: &[(&str, &str)]) -> String {
    format!(
        "[{}]",
        values.iter().map(|(k, v)| format!("({k:?}, {v:?})")).collect::<Vec<_>>().join(", ")
    )
}

fn py_ints(values: &[i64]) -> String {
    format!("[{}]", values.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", "))
}

/// `pack(values)` of the Python setup: little-endian `int32` samples.
fn pack(values: &[i64]) -> Vec<u8> {
    values
        .iter()
        .flat_map(|v| i32::try_from(*v).expect("an int32 sample").to_le_bytes())
        .collect()
}

/// `snap(section, label)`.
fn snap(rig: &Rig, section: &str, label: &str) {
    let s = rig.sensor(section);
    res(
        &format!("{label}.snap"),
        &PyValue::Tuple(vec![
            int(s.last_error_count()),
            int(s.consecutive_fails()),
            PyValue::Bool(s.batch_bulk.is_started()),
            s.get_status(0.0),
        ]),
    );
}

// ---------------------------------------------------------------------------
// Comparison plumbing
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("hx71x fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping hx71x oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn py_run(py: &mut pyoracle::Oracle, code: &str) {
    if let Err(e) = py.exec(code) {
        panic!("oracle failed on `{code}`: {}: {}", e.class_name, e.message);
    }
}

/// Compare the Rust trace against the Python `LOG`, then clear both.
fn check(label: &str, py: &mut pyoracle::Oracle) {
    let expected = py.eval("take_log()");
    let actual: Vec<String> = take_trace();
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    if let Value::List(items) = &expected {
        let expected_lines: Vec<String> = items
            .iter()
            .map(|v| match v {
                Value::Str(s) => s.clone(),
                other => panic!("{label}: trace entry is not a string: {other:?}"),
            })
            .collect();
        if expected_lines != actual {
            let mut report = format!("{label}: traces differ\n");
            let n = expected_lines.len().max(actual.len());
            for i in 0..n {
                let e = expected_lines.get(i).map_or("<none>", String::as_str);
                let a = actual.get(i).map_or("<none>", String::as_str);
                let mark = if e == a { "  " } else { "!!" };
                report.push_str(&format!("{mark} {i:3} py: {e}\n{mark}     rs: {a}\n"));
            }
            panic!("{report}");
        }
    }
    let actual_value = Value::List(actual.iter().map(|s| Value::str(s)).collect());
    let record = Record {
        v: 1,
        unit: "extras.hx71x".to_string(),
        scenario: label.to_string(),
        seq: 0,
        function: "trace".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    if let Err(d) = assert_equal(&record, "ret", &expected, &actual_value) {
        panic!("{label}: {d}");
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn module_surface_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(
        &mut py,
        "res('consts', (hx_mod.UPDATE_INTERVAL, hx_mod.SAMPLE_ERROR_DESYNC, hx_mod.SAMPLE_ERROR_LONG_READ))\n\
         res('types', list(hx_mod.HX71X_SENSOR_TYPES))\n\
         res('entry points', (hasattr(hx_mod, 'load_config'), hasattr(hx_mod, 'load_config_prefix'), \
         hasattr(hx_mod, 'add_printer_objects'), hasattr(hx_mod, 'add_early_printer_objects')))",
    );
    install_log_sink();
    take_trace();
    res(
        "consts",
        &PyValue::Tuple(vec![
            PyValue::Float(UPDATE_INTERVAL),
            int(SAMPLE_ERROR_DESYNC),
            int(SAMPLE_ERROR_LONG_READ),
        ]),
    );
    res(
        "types",
        &PyValue::List(HX71X_SENSOR_TYPES.iter().map(|(name, _)| pstr(name)).collect()),
    );
    let f = module_factory();
    res(
        "entry points",
        &PyValue::Tuple(vec![
            PyValue::Bool(f.load_config.is_some()),
            PyValue::Bool(f.load_config_prefix.is_some()),
            PyValue::Bool(f.add_printer_objects.is_some()),
            PyValue::Bool(f.add_early_printer_objects.is_some()),
        ]),
    );
    check("module surface", &mut py);
}

#[test]
fn constructors_match_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "a = guard(hx_mod.HX711, make_config('load_cell a', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')]))\n\
         b = guard(hx_mod.HX717, make_config('load_cell b', [('dout_pin', 'PA6'), ('sclk_pin', 'PA7'), ('gain', 'B-8')]))\n\
         res('types', (a.sensor_type, b.sensor_type, a.sps, b.sps, a.gain_channel, b.gain_channel, a.oid, b.oid))",
    );
    let rig = Rig::new();
    let a = guard(HX711(&rig.config("load_cell a", &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])))
        .expect("hx711");
    let b = guard(HX717(&rig.config(
        "load_cell b",
        &[("dout_pin", "PA6"), ("sclk_pin", "PA7"), ("gain", "B-8")],
    )))
    .expect("hx717");
    res(
        "types",
        &PyValue::Tuple(vec![
            pstr(a.sensor_type),
            pstr(b.sensor_type),
            int(a.sps),
            int(b.sps),
            int(a.gain_channel),
            int(b.gain_channel),
            int(a.oid),
            int(b.oid),
        ]),
    );
    check("direct constructors", &mut py);
}

/// One build of a sensor with an optional run of the MCU config callbacks.
fn init_case(
    py: &mut pyoracle::Oracle,
    label: &str,
    kind: &str,
    section: &str,
    values: &[(&str, &str)],
    cbs: bool,
) {
    py_run(
        py,
        &format!(
            "make_world()\n\
             s = guard(build, {kind:?}, {section:?}, {})\n\
             if s is not None:\n    \
                 res('attrs', (s.name, s.sensor_type, s.oid, s.dout_pin, s.sclk_pin, s.sps, \
                 s.gain_channel, s.get_samples_per_second(), s.get_range(), s.get_mcu().get_name(), \
                 s.get_mcu() is MCU, s.get_mcu() is AUX))\n    \
                 res('status', s.get_status(0.))\n    \
                 res('err', [s.lookup_sensor_error(c) for c in (0, 1, -7, 2 ** 31)])\n\
             if {}:\n    run_cbs()",
            py_values(values),
            if cbs { "True" } else { "False" }
        ),
    );
    let rig = Rig::new();
    if let Some(s) = rig.build(kind, section, values) {
        let mcu: Mcu = rig.mcu.clone();
        let aux: Mcu = rig.aux.clone();
        let m = s.get_mcu();
        let (lo, hi) = s.get_range();
        res(
            "attrs",
            &PyValue::Tuple(vec![
                pstr(&s.name),
                pstr(s.sensor_type),
                int(s.oid),
                pstr(&s.dout_pin),
                pstr(&s.sclk_pin),
                int(s.sps),
                int(s.gain_channel),
                int(s.get_samples_per_second()),
                PyValue::Tuple(vec![int(lo), int(hi)]),
                pstr(&m.get_name()),
                PyValue::Bool(thin(&m) == thin(&mcu)),
                PyValue::Bool(thin(&m) == thin(&aux)),
            ]),
        );
        res("status", &s.get_status(0.0));
        res(
            "err",
            &PyValue::List(
                [0_i64, 1, -7, 2_i64.pow(31)]
                    .iter()
                    .map(|c| pstr(&s.lookup_sensor_error(*c)))
                    .collect(),
            ),
        );
    }
    if cbs {
        rig.run_cbs();
    }
    check(label, py);
}

#[test]
fn init_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let pins = [("dout_pin", "PA4"), ("sclk_pin", "PA5")];
    let with = |extra: &[(&'static str, &'static str)]| {
        let mut v = pins.to_vec();
        v.extend_from_slice(extra);
        v
    };
    init_case(&mut py, "hx711 defaults", "hx711", "load_cell lc", &pins, true);
    init_case(
        &mut py,
        "hx711 sps 10 gain B-32",
        "hx711",
        "load_cell lc",
        &with(&[("sample_rate", "10"), ("gain", "B-32")]),
        true,
    );
    init_case(
        &mut py,
        "hx711 gain A-64 without callbacks",
        "hx711",
        "load_cell lc",
        &with(&[("gain", "A-64")]),
        false,
    );
    init_case(&mut py, "hx717 defaults", "hx717", "load_cell lc", &pins, true);
    init_case(
        &mut py,
        "hx717 sps 20 gain B-8",
        "hx717",
        "load_cell lc",
        &with(&[("sample_rate", "20"), ("gain", "B-8")]),
        true,
    );
    init_case(
        &mut py,
        "hx717 sps 80 gain B-64",
        "hx717",
        "load_cell lc",
        &with(&[("sample_rate", "80"), ("gain", "B-64")]),
        true,
    );
    init_case(
        &mut py,
        "hx717 sps 10 gain A-64",
        "hx717",
        "load_cell lc",
        &with(&[("sample_rate", "10"), ("gain", "A-64")]),
        true,
    );
    init_case(
        &mut py,
        "hx717 sps 320 explicit",
        "hx717",
        "load_cell lc",
        &with(&[("sample_rate", "320")]),
        true,
    );
    init_case(
        &mut py,
        "second mcu, single word section",
        "hx711",
        "load_cell",
        &[("dout_pin", "aux:PB1"), ("sclk_pin", "aux:PB2")],
        true,
    );
    init_case(
        &mut py,
        "name is the last word",
        "hx717",
        "load_cell my sensor",
        &[("dout_pin", " mcu:PA4 "), ("sclk_pin", "mcu: PA5")],
        true,
    );
}

#[test]
fn init_errors_match_reference() {
    let Some(mut py) = oracle() else { return };
    let pins = [("dout_pin", "PA4"), ("sclk_pin", "PA5")];
    let with = |extra: &[(&'static str, &'static str)]| {
        let mut v = pins.to_vec();
        v.extend_from_slice(extra);
        v
    };
    init_case(&mut py, "missing dout_pin", "hx711", "load_cell lc", &[("sclk_pin", "PA5")], true);
    init_case(&mut py, "missing sclk_pin", "hx711", "load_cell lc", &[("dout_pin", "PA4")], true);
    init_case(&mut py, "missing both pins", "hx717", "load_cell lc", &[], true);
    init_case(
        &mut py,
        "dout on the main mcu, sclk on aux",
        "hx711",
        "load_cell lc",
        &[("dout_pin", "PA4"), ("sclk_pin", "aux:PB2")],
        true,
    );
    init_case(
        &mut py,
        "dout on aux, sclk on the main mcu",
        "hx717",
        "load_cell lc",
        &[("dout_pin", "aux:PB1"), ("sclk_pin", "PA5")],
        true,
    );
    init_case(
        &mut py,
        "unknown dout chip",
        "hx711",
        "load_cell lc",
        &[("dout_pin", "foo:PA4"), ("sclk_pin", "PA5")],
        true,
    );
    init_case(
        &mut py,
        "unknown sclk chip",
        "hx711",
        "load_cell lc",
        &[("dout_pin", "PA4"), ("sclk_pin", "foo:PA5")],
        true,
    );
    init_case(&mut py, "hx711 sps 5", "hx711", "load_cell lc", &with(&[("sample_rate", "5")]), true);
    init_case(
        &mut py,
        "hx711 sps 320",
        "hx711",
        "load_cell lc",
        &with(&[("sample_rate", "320")]),
        true,
    );
    init_case(
        &mut py,
        "hx717 sps 40",
        "hx717",
        "load_cell lc",
        &with(&[("sample_rate", "40")]),
        true,
    );
    init_case(
        &mut py,
        "sps not an int",
        "hx711",
        "load_cell lc",
        &with(&[("sample_rate", "abc")]),
        true,
    );
    init_case(
        &mut py,
        "sps float",
        "hx711",
        "load_cell lc",
        &with(&[("sample_rate", "80.0")]),
        true,
    );
    init_case(&mut py, "sps empty", "hx717", "load_cell lc", &with(&[("sample_rate", "")]), true);
    init_case(&mut py, "hx711 gain B-64", "hx711", "load_cell lc", &with(&[("gain", "B-64")]), true);
    init_case(&mut py, "hx711 gain B-8", "hx711", "load_cell lc", &with(&[("gain", "B-8")]), true);
    init_case(&mut py, "gain unknown", "hx717", "load_cell lc", &with(&[("gain", "x")]), true);
    init_case(&mut py, "gain lowercase", "hx717", "load_cell lc", &with(&[("gain", "a-128")]), true);
}

#[test]
fn setup_trigger_analog_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "s = build('hx711', 'load_cell lc', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')])\n\
         s2 = build('hx717', 'load_cell aux', [('dout_pin', 'aux:PB1'), ('sclk_pin', 'aux:PB2'), ('sample_rate', '20')])\n\
         run_cbs()\n\
         s.setup_trigger_analog(12)\n\
         s2.setup_trigger_analog(0)\n\
         res('ta sps', float(s.get_samples_per_second()))\n\
         res('ta sps 2', float(s2.get_samples_per_second()))\n\
         res('ta mcu', (s.get_mcu().get_name(), s2.get_mcu().get_name()))\n\
         res('ta err', (s.lookup_sensor_error(3), s2.lookup_sensor_error(-1)))",
    );
    let rig = Rig::new();
    let s = rig
        .build("hx711", "load_cell lc", &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])
        .expect("sensor");
    let s2 = rig
        .build(
            "hx717",
            "load_cell aux",
            &[("dout_pin", "aux:PB1"), ("sclk_pin", "aux:PB2"), ("sample_rate", "20")],
        )
        .expect("sensor");
    rig.run_cbs();
    // The `trigger_analog` view of the sensors.
    let a: Arc<dyn TriggerAnalogSensor> = s;
    let b: Arc<dyn TriggerAnalogSensor> = s2;
    guard(a.setup_trigger_analog(12));
    guard(b.setup_trigger_analog(0));
    res("ta sps", &PyValue::Float(a.get_samples_per_second()));
    res("ta sps 2", &PyValue::Float(b.get_samples_per_second()));
    res(
        "ta mcu",
        &PyValue::Tuple(vec![pstr(&a.get_mcu().get_name()), pstr(&b.get_mcu().get_name())]),
    );
    res(
        "ta err",
        &PyValue::Tuple(vec![
            pstr(&a.lookup_sensor_error(3).expect("message")),
            pstr(&b.lookup_sensor_error(-1).expect("message")),
        ]),
    );
    check("trigger analog", &mut py);
}

/// `_convert_samples` on `rows` of `(ptime, val)`.
fn convert_case(
    py: &mut pyoracle::Oracle,
    rig: &Rig,
    label: &str,
    section: &str,
    rows: &[(f64, i64)],
) {
    let literal = format!(
        "[{}]",
        rows.iter().map(|(t, v)| format!("({t:?}, {v})")).collect::<Vec<_>>().join(", ")
    );
    py_run(
        py,
        &format!(
            "samples = {literal}\n\
             guard(S[{section:?}]._convert_samples, samples)\n\
             res({label:?}, samples)\n\
             res({label:?} + '.errors', S[{section:?}].last_error_count)"
        ),
    );
    let sensor = rig.sensor(section);
    let mut samples: Vec<PyValue> = rows
        .iter()
        .map(|(t, v)| PyValue::Tuple(vec![PyValue::Float(*t), int(*v)]))
        .collect();
    guard(sensor._convert_samples(&mut samples));
    res(label, &PyValue::List(samples));
    res(&format!("{label}.errors"), &int(sensor.last_error_count()));
    check(label, py);
}

#[test]
fn convert_samples_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build('hx711', 'load_cell lc', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')])",
    );
    let rig = Rig::new();
    rig.build("hx711", "load_cell lc", &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])
        .expect("sensor");
    check("convert setup", &mut py);
    let sec = "load_cell lc";
    convert_case(&mut py, &rig, "empty", sec, &[]);
    convert_case(
        &mut py,
        &rig,
        "rounding",
        sec,
        &[
            (1.0000004999999, 1),
            (0.0000005, -1),
            (1234.5678905, 123_456),
            (2.5e-6, -3),
            (1e-7, 0x7FFFFF),
            (99999.99999949, -0x800000),
            (0.3333333333, 0x123456),
            (5e-7, 8_388_608),
        ],
    );
    convert_case(
        &mut py,
        &rig,
        "full scale",
        sec,
        &[(1.0, 0x7FFFFF), (2.5, -0x800000), (3.5, 0), (4.25, 0x3FFFFFFF), (5.0, -0x7FFFFFFF)],
    );
    convert_case(
        &mut py,
        &rig,
        "long read in the middle",
        sec,
        &[(1.5, 3), (2.5, SAMPLE_ERROR_LONG_READ), (3.5, 4), (4.5, 5)],
    );
    convert_case(
        &mut py,
        &rig,
        "desync first",
        sec,
        &[(1.5, SAMPLE_ERROR_DESYNC), (2.5, 4)],
    );
    convert_case(
        &mut py,
        &rig,
        "two errors count once",
        sec,
        &[(1.5, 1), (2.5, SAMPLE_ERROR_DESYNC), (3.5, SAMPLE_ERROR_LONG_READ), (4.5, 9)],
    );
    convert_case(
        &mut py,
        &rig,
        "neighbours of the sentinels",
        sec,
        &[
            (1.5, SAMPLE_ERROR_DESYNC + 1),
            (2.5, SAMPLE_ERROR_LONG_READ - 1),
            (3.5, SAMPLE_ERROR_LONG_READ + 1),
            (4.5, 0x7FFFFFFF),
        ],
    );
}

#[test]
fn before_config_callbacks_match_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "s = build('hx711', 'load_cell lc', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')])\n\
         guard(s._start_measurements)\n\
         snap('load_cell lc', 'start')\n\
         guard(s._finish_measurements)\n\
         guard(s._process_batch, 1.0)\n\
         guard(s.add_client, client)\n\
         snap('load_cell lc', 'add client')\n\
         P.shutdown = True\n\
         guard(s._finish_measurements)\n\
         snap('load_cell lc', 'shutdown finish')",
    );
    let rig = Rig::new();
    let s = rig
        .build("hx711", "load_cell lc", &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])
        .expect("sensor");
    let sec = "load_cell lc";
    guard(s._start_measurements());
    snap(&rig, sec, "start");
    guard(s._finish_measurements());
    guard(s._process_batch(1.0));
    guard(s.add_client(rig.client()));
    snap(&rig, sec, "add client");
    rig.printer.shutdown.set(true);
    guard(s._finish_measurements());
    snap(&rig, sec, "shutdown finish");
    check("before config callbacks", &mut py);
}

/// Both sides: set the status the next query answers with.
fn do_status(py: &mut pyoracle::Oracle, rig: &Rig, clock: i64, seq: i64, buffered: i64, ovf: i64) {
    py_run(py, &format!("st({clock}, {seq}, {buffered}, {ovf})"));
    *rig.mcu.chip.status.borrow_mut() = status(clock, seq, buffered, ovf);
}

/// Both sides: one `sensor_bulk_data` message.
fn do_feed(py: &mut pyoracle::Oracle, rig: &Rig, seq: i64, values: &[i64]) {
    py_run(py, &format!("feed(MCU, {seq}, pack({}))", py_ints(values)));
    rig.mcu.feed(seq, &pack(values));
}

/// Both sides: one run of the batch callback, as the reactor timer does.
fn do_proc(py: &mut pyoracle::Oracle, rig: &Rig, section: &str, label: &str, eventtime: f64) {
    py_run(
        py,
        &format!(
            "res({label:?}, guard(S[{section:?}].batch_bulk._proc_batch, {eventtime:?}))\n\
             snap({section:?}, {label:?})"
        ),
    );
    let next = guard(rig.sensor(section).batch_bulk._proc_batch(eventtime));
    res(label, &next.map_or(PyValue::None, PyValue::Float));
    snap(rig, section, label);
}

#[test]
fn batch_pipeline_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let sec = "load_cell lc";
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build('hx711', 'load_cell lc', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')])\n\
         run_cbs()",
    );
    let rig = Rig::new();
    let s = rig
        .build("hx711", sec, &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])
        .expect("sensor");
    rig.run_cbs();
    check("pipeline setup", &mut py);

    // The first client starts the sensor: rest ticks, log line, clock reset.
    do_status(&mut py, &rig, 1000, 0, 0, 0);
    py_run(&mut py, &format!("guard(S[{sec:?}].add_client, client)\nsnap({sec:?}, 'started')"));
    guard(s.add_client(rig.client()));
    snap(&rig, sec, "started");
    check("start", &mut py);

    // Twelve samples fill one message; the saturation values and zero pass.
    do_status(&mut py, &rig, 101_000, 1, 0, 0);
    do_feed(
        &mut py,
        &rig,
        0,
        &[1, 2, 3, -5, 0x7FFFFF, -0x800000, 100_000, 7, 8, 9, 10, 11],
    );
    do_proc(&mut py, &rig, sec, "full block", 1000.5);
    // A partial message plus one buffered sample.
    do_status(&mut py, &rig, 201_000, 2, 4, 0);
    do_feed(&mut py, &rig, 1, &[-100, 0, 0x123456, -0x123456, 77]);
    do_proc(&mut py, &rig, sec, "partial block", 1000.6);
    // No new message: the client still gets an empty batch.
    do_status(&mut py, &rig, 301_000, 3, 0, 0);
    do_proc(&mut py, &rig, sec, "empty batch", 1000.7);
    check("plain batches", &mut py);

    // Four overflow batches only count; the fifth restarts the sensor.
    for (k, label) in ["overflow 1", "overflow 2", "overflow 3", "overflow 4"].iter().enumerate() {
        let n = 4 + k as i64;
        do_status(&mut py, &rig, 100_000 * n + 1000, n, 0, 1 + k as i64);
        do_feed(&mut py, &rig, n - 1, &[n, -n, n * 1000]);
        do_proc(&mut py, &rig, sec, label, 1000.8 + k as f64);
    }
    check("overflows below the limit", &mut py);
    do_status(&mut py, &rig, 801_000, 8, 0, 5);
    do_feed(&mut py, &rig, 7, &[5, 6, 7, 8]);
    do_proc(&mut py, &rig, sec, "overflow restart", 1010.0);
    check("overflow restart", &mut py);

    // A quiet batch after the restart; then an error sample restarts again.
    do_status(&mut py, &rig, 901_000, 9, 0, 5);
    do_feed(&mut py, &rig, 8, &[1, 2, 3]);
    do_proc(&mut py, &rig, sec, "after restart", 1011.0);
    do_status(&mut py, &rig, 1_001_000, 10, 0, 5);
    do_feed(&mut py, &rig, 9, &[1, 2, SAMPLE_ERROR_LONG_READ, 4]);
    do_proc(&mut py, &rig, sec, "long read restart", 1012.0);
    check("error restart", &mut py);

    // A desync sample first yields no samples; an error wins over overflows.
    do_status(&mut py, &rig, 1_101_000, 11, 0, 5);
    do_feed(&mut py, &rig, 10, &[SAMPLE_ERROR_DESYNC, 5]);
    do_proc(&mut py, &rig, sec, "desync restart", 1013.0);
    do_status(&mut py, &rig, 1_201_000, 12, 0, 6);
    do_feed(&mut py, &rig, 11, &[3, SAMPLE_ERROR_DESYNC]);
    do_proc(&mut py, &rig, sec, "error and overflow", 1014.0);
    check("error priority", &mut py);

    // One overflow batch, then a quiet one clears the counter.
    do_status(&mut py, &rig, 1_301_000, 13, 0, 7);
    do_feed(&mut py, &rig, 12, &[9, 9]);
    do_proc(&mut py, &rig, sec, "one overflow", 1015.0);
    do_status(&mut py, &rig, 1_401_000, 14, 0, 7);
    do_feed(&mut py, &rig, 13, &[9]);
    do_proc(&mut py, &rig, sec, "overflow cleared", 1016.0);
    check("overflow counter reset", &mut py);

    // The client declines: the helper stops the sensor.
    py_run(&mut py, "CLIENT_OK[0] = False");
    rig.client_ok.set(false);
    do_status(&mut py, &rig, 1_501_000, 15, 0, 7);
    do_feed(&mut py, &rig, 14, &[4, 5]);
    do_proc(&mut py, &rig, sec, "client declines", 1017.0);
    check("stop", &mut py);

    // A new client restarts it; at shutdown the stop skips the serial link.
    py_run(&mut py, "CLIENT_OK[0] = True");
    rig.client_ok.set(true);
    do_status(&mut py, &rig, 1_601_000, 16, 0, 7);
    py_run(&mut py, &format!("guard(S[{sec:?}].add_client, client)\nsnap({sec:?}, 'restarted')"));
    guard(s.add_client(rig.client()));
    snap(&rig, sec, "restarted");
    py_run(&mut py, "CLIENT_OK[0] = False\nP.shutdown = True");
    rig.client_ok.set(false);
    rig.printer.shutdown.set(true);
    do_status(&mut py, &rig, 1_701_000, 17, 0, 7);
    do_feed(&mut py, &rig, 16, &[1]);
    do_proc(&mut py, &rig, sec, "stop at shutdown", 1018.0);
    check("shutdown stop", &mut py);
}

#[test]
fn second_sensor_on_aux_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let sec = "load_cell probe";
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build('hx717', 'load_cell main', [('dout_pin', 'PA4'), ('sclk_pin', 'PA5')])\n\
         build('hx717', 'load_cell probe', [('dout_pin', 'aux:PB1'), ('sclk_pin', 'aux:PB2'), ('sample_rate', '10')])\n\
         run_cbs()",
    );
    let rig = Rig::new();
    rig.build("hx717", "load_cell main", &[("dout_pin", "PA4"), ("sclk_pin", "PA5")])
        .expect("sensor");
    let probe = rig
        .build(
            "hx717",
            sec,
            &[("dout_pin", "aux:PB1"), ("sclk_pin", "aux:PB2"), ("sample_rate", "10")],
        )
        .expect("sensor");
    rig.run_cbs();
    check("two sensors setup", &mut py);

    // The aux sensor answers from its own MCU's status and queue.
    py_run(
        &mut py,
        "AUX.status = {'clock': 5000, 'query_ticks': 40, 'next_sequence': 0, 'buffered': 0, 'possible_overflows': 0}\n\
         guard(S['load_cell probe'].add_client, client)\n\
         snap('load_cell probe', 'started')",
    );
    *rig.aux.chip.status.borrow_mut() = status(5000, 0, 0, 0);
    guard(probe.add_client(rig.client()));
    snap(&rig, sec, "started");
    py_run(
        &mut py,
        "AUX.status = {'clock': 55000, 'query_ticks': 40, 'next_sequence': 1, 'buffered': 8, 'possible_overflows': 0}\n\
         feed(AUX, 0, pack([10, -20, 30, 40, 50, 60]))\n\
         feed(MCU, 0, pack([99, 98]))\n\
         res('aux batch', guard(S['load_cell probe'].batch_bulk._proc_batch, 2000.0))\n\
         snap('load_cell probe', 'aux batch')",
    );
    *rig.aux.chip.status.borrow_mut() = status(55000, 1, 8, 0);
    rig.aux.feed(0, &pack(&[10, -20, 30, 40, 50, 60]));
    rig.mcu.feed(0, &pack(&[99, 98]));
    let next = guard(probe.batch_bulk._proc_batch(2000.0));
    res("aux batch", &next.map_or(PyValue::None, PyValue::Float));
    snap(&rig, sec, "aux batch");
    check("aux sensor batch", &mut py);
}
