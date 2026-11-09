//! Parity tests for `extras::mpu9250` (port of `klippy/extras/mpu9250.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The Python side runs the real
//! `extras.mpu9250`, `extras.bus`, `extras.bulk_sensor` and `gcode`
//! modules over minimal fakes of the printer, reactor, MCU, pins,
//! toolhead and webhooks. The Rust side runs the port, the real
//! `extras::bus` / `extras::bulk_sensor` ports and the real
//! `klippy_core::gcode::GCodeDispatch` over the same fakes.
//!
//! Both sides append one text line per observable effect to a trace: MCU
//! config commands, command lookups, commands sent (format and argument
//! values), I2C register traffic, toolhead calls, webhooks registrations,
//! G-code output lines, raised errors (class and message), the `logging`
//! records this module and the batch helper emit, and the result values
//! (`repr`, so floats compare by their exact shortest spelling). The two
//! traces must be identical.
//!
//! The MCU fake returns a constant estimated print time, so the
//! `reactor.monotonic()` value itself is not compared: both sides still
//! verify the `+ 0.020` offset and the clock conversion.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit.

use extras::adxl345::AccelChipApi;
use extras::mpu9250::{
    module_factory, mpu_dev_id, sample_rate_div, MPU9250, BATCH_UPDATES, FREEFALL_ACCEL, SCALE,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatListOpts, GetFloatOpts,
    GetIntListOpts, GetIntOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::GCodeApi;
use klippy_api::log::Level;
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, AsyncResponseWrapperApi, CommandQueryWrapper,
    CommandQueryWrapperApi, CommandQueue, CommandWrapper, CommandWrapperApi, ConfigCallback,
    LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams,
    PinResolver, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::{HomingTargetApi, ToolHeadApi};
use klippy_api::webhooks::{WebHooksApi, WebHooksCallback};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use pycompat::text::{float_repr, str_repr};
use pycompat::{PyInt, PyValue};
use klippy_core::gcode::GCodeDispatch;
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

/// Only the log records this module and the batch helper emit are traced.
fn install_log_sink() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            let msg = rec.message;
            if msg.starts_with("MPU9250 ") || msg.starts_with("BatchBulkHelper") {
                let level = match rec.level {
                    Level::Debug => "debug",
                    Level::Info => "info",
                    Level::Warning => "warning",
                    Level::Error => "exception",
                    Level::Critical => "critical",
                };
                tr(format!("{level}|{msg}"));
            }
        }));
    });
}

fn exc(e: &KlippyError) {
    tr(format!("exc|{}|{}", e.class_name(), e.message()));
}

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

/// Replace `YYYYMMDD_HHMMSS` timestamps (the default `NAME`).
fn normalize_ts(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::new();
    let mut i = 0;
    while i < b.len() {
        let is_ts = i + 15 <= b.len()
            && b[i..i + 8].iter().all(u8::is_ascii_digit)
            && b[i + 8] == b'_'
            && b[i + 9..i + 15].iter().all(u8::is_ascii_digit);
        if is_ts {
            out.push_str("<ts>");
            i += 15;
        } else {
            let ch = s[i..].chars().next().expect("char boundary");
            out.push(ch);
            i += ch.len_utf8();
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import sys, types, configparser, re, os, struct
import gcode as gcode_mod
import configfile as configfile_mod

LOG = []

def take_log():
    out = [re.sub(r"[0-9]{8}_[0-9]{6}", "<ts>", s) for s in LOG]
    del LOG[:]
    return out

class _FakeLogging:
    @staticmethod
    def _emit(level, msg, args):
        text = (msg % args) if args else msg
        if text.startswith("MPU9250 ") or text.startswith("BatchBulkHelper"):
            LOG.append("%s|%s" % (level, text))
    @staticmethod
    def info(msg, *args):
        _FakeLogging._emit("info", msg, args)
    @staticmethod
    def exception(msg, *args):
        _FakeLogging._emit("exception", msg, args)
    @staticmethod
    def warning(msg, *args):
        _FakeLogging._emit("warning", msg, args)
    @staticmethod
    def debug(msg, *args):
        _FakeLogging._emit("debug", msg, args)

class _McuError(Exception):
    pass
_McuError.__name__ = "error"
_mcu_mod = types.ModuleType("mcu")
_mcu_mod.error = _McuError
def _get_printer_mcu(printer, name):
    if name == 'mcu':
        return printer.lookup_object(name)
    return printer.lookup_object('mcu ' + name)
_mcu_mod.get_printer_mcu = _get_printer_mcu
sys.modules['mcu'] = _mcu_mod

import extras.bulk_sensor as bulk_mod
import extras.bus as bus_mod
import extras.adxl345 as adxl_mod
import extras.mpu9250 as mpu_mod

for _m in (gcode_mod, bulk_mod, bus_mod, mpu_mod):
    _m.logging = _FakeLogging

class _FakeProcess:
    def __init__(self, target=None):
        self.target = target
        self.daemon = False
    def start(self):
        self.target()

class _FakeMP:
    Process = _FakeProcess

class _FakeTime:
    @staticmethod
    def strftime(fmt):
        return "20240101_000000"

adxl_mod.multiprocessing = _FakeMP
adxl_mod.time = _FakeTime

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

class FakeTimer:
    def __init__(self, callback, waketime):
        self.callback = callback
        self.waketime = waketime

class FakeReactor:
    NEVER = 9999999999999999.
    def mutex(self, is_locked=False):
        return None
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
        self.handlers = {}
        self.reactor = FakeReactor()
    def get_start_args(self):
        return {}
    def get_reactor(self):
        return self.reactor
    def get_state_message(self):
        return ("Printer is ready", "ready")
    def lookup_object(self, name, default=configfile_mod.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile_mod.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, callback):
        self.handlers.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.handlers.get(event, [])]
    def invoke_shutdown(self, msg, details={}):
        LOG.append("shutdown|%s" % (msg,))

def norm(data):
    return [bytes(p) if isinstance(p, (list, bytearray)) else p for p in data]

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        d = norm(list(data))
        LOG.append("send|%s|%r|%d|%d" % (self.fmt, d, minclock, reqclock))
        if self.fmt.startswith("i2c_transfer") and len(d) == 3 and d[2] == 0 and len(d[1]) >= 2:
            reg, val = d[1][0], d[1][1]
            if reg not in self.mcu.stuck:
                self.mcu.regs[reg] = val
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        LOG.append("wait_ack|%s|%r|%d|%d" % (self.fmt, norm(list(data)), minclock, reqclock))

class FakeQuery:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0, retry=True):
        d = norm(list(data))
        LOG.append("qsend|%s|%r|%d|%d|%s" % (self.fmt, d, minclock, reqclock, retry))
        if self.fmt.startswith("i2c_transfer"):
            write, read_len = d[1], d[2]
            resp = bytes(self.mcu.regs.get(write[0], 0) for _ in range(read_len))
            return {"i2c_bus_status": "SUCCESS", "response": resp}
        if self.fmt.startswith("query_mpu9250_status"):
            return dict(self.mcu.status)
        raise Exception("unexpected query %s" % (self.fmt,))

STATUS0 = {"clock": 1000, "query_ticks": 40, "next_sequence": 0, "buffered": 0,
           "possible_overflows": 0}
STATUS1 = {"clock": 101000, "query_ticks": 50, "next_sequence": 3, "buffered": 5,
           "possible_overflows": 2}

class FakeMcu:
    error = _McuError
    def __init__(self, printer, name):
        self.printer = printer
        self.name = name
        self.next_oid = 0
        self.config_cbs = []
        self.serial_cbs = []
        self.regs = {0x75: 0x71}
        self.stuck = set()
        self.status = dict(STATUS0)
    def get_name(self):
        return self.name
    def get_printer(self):
        return self.printer
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
    def is_fileoutput(self):
        return False
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
    def try_lookup_command(self, fmt):
        return None
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
    def estimated_print_time(self, eventtime):
        return 0.0
    def print_time_to_clock(self, print_time):
        return int(print_time * 1000000.)
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
    def reset_pin_sharing(self, pin_params):
        del self.active["%s:%s" % (pin_params['chip_name'], pin_params['pin'])]

class FakeToolHead:
    def __init__(self):
        self.times = [100.0]
        self.on_dwell = None
    def get_last_move_time(self):
        t = self.times.pop(0) if len(self.times) > 1 else self.times[0]
        LOG.append("get_last_move_time|%r" % (t,))
        return t
    def wait_moves(self):
        LOG.append("wait_moves")
    def dwell(self, delay):
        LOG.append("dwell|%r" % (delay,))
        if self.on_dwell is not None:
            self.on_dwell()

class FakeWebhooks:
    def __init__(self):
        self.endpoints = {}
    def register_mux_endpoint(self, path, key, value, callback):
        LOG.append("mux_endpoint|%s|%s|%s" % (path, key, value))
        self.endpoints[(path, value)] = callback

class FakeConn:
    def __init__(self):
        self.closed = False
    def is_closed(self):
        return self.closed
    def send(self, data):
        LOG.append("conn_send|%r" % (data,))

class FakeWebRequest:
    def __init__(self, conn):
        self.conn = conn
    def get_client_connection(self):
        return self.conn
    def get_dict(self, item, default=None):
        return default
    def send(self, data):
        LOG.append("req_send|%r" % (data,))

P = MCU = AUX = PINS = GC = TH = WH = None
CH = {}

def make_world():
    global P, MCU, AUX, PINS, GC, TH, WH, CH
    CH = {}
    P = FakePrinter()
    MCU = FakeMcu(P, "mcu")
    AUX = FakeMcu(P, "aux")
    PINS = FakePins()
    PINS.chips = {"mcu": MCU, "aux": AUX}
    GC = gcode_mod.GCodeDispatch(P)
    GC.register_output_handler(lambda msg: LOG.append("out|%s" % (msg,)))
    TH = FakeToolHead()
    WH = FakeWebhooks()
    P.objects = {"pins": PINS, "gcode": GC, "toolhead": TH, "webhooks": WH,
                 "mcu": MCU, "mcu aux": AUX}
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

def build_chip(section, values):
    cfg = make_config(section, values)
    if len(section.split()) == 1:
        chip = mpu_mod.load_config(cfg)
    else:
        chip = mpu_mod.load_config_prefix(cfg)
    P.objects[section] = chip
    CH[section] = chip
    return chip

def help_res():
    res("help", sorted((k, h) for k, h in GC.get_command_help().items() if k.startswith("ACCELEROMETER_")))

def run_cbs():
    guard(MCU.run_config_cbs)
    guard(AUX.run_config_cbs)

def ready():
    GC._handle_ready()
    del LOG[:]

def gcmd_run(line):
    try:
        GC._process_commands([line], need_ack=False)
    except Exception as e:
        LOG.append("exc|%s|%s" % (type(e).__name__, e))

def feed(mcu, seq, data):
    for cb in mcu.serial_cbs:
        cb({'sequence': seq, 'data': bytes(data)})

def frame(x, y, z):
    return struct.pack(">hhh", x, y, z)

def qmsg(times):
    return {'data': [(t, t * 2.0, -t, t + 0.25) for t in times]}

def samples_of(q):
    return [tuple(s) for s in q.get_samples()]

def read_file(path):
    with open(path) as f:
        data = f.read()
    os.remove(path)
    return data
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes
// ---------------------------------------------------------------------------

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    reactor: Reactor,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(Vec::new()),
            reactor: Reactor::new(false),
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
        false
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
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by mpu9250")
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
    fn send_event(&self, event: &str, params: &[EventArg]) -> KResult<Vec<PyValue>> {
        let mut taken = std::mem::take(&mut *self.handlers.borrow_mut());
        let mut out = Vec::new();
        let mut failure = None;
        for (name, cb) in taken.iter_mut() {
            if name == event {
                match cb(params) {
                    Ok(v) => out.push(v),
                    Err(e) => {
                        failure = Some(e);
                        break;
                    }
                }
            }
        }
        let mut current = self.handlers.borrow_mut();
        taken.extend(current.drain(..));
        *current = taken;
        match failure {
            Some(e) => Err(e),
            None => Ok(out),
        }
    }
    fn request_exit(&self, _result: &str) {}
}

/// A `ConfigWrapper` over a list of `(option, value)` strings, with the
/// `configfile.py` semantics of the getters `mpu9250` and `bus` call.
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
        if let Some(minval) = opts.minval {
            if v < minval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minval}",
                    self.section
                )));
            }
        }
        if let Some(maxval) = opts.maxval {
            if v > maxval {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxval}",
                    self.section
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat is not used by mpu9250")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by mpu9250")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by mpu9250")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by mpu9250")
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let Some(raw) = self.value(option) else {
            return match opts.default {
                None => Err(self.missing(option)),
                Some(d) => Ok(d),
            };
        };
        let parts: Vec<String> = if raw.trim().is_empty() {
            Vec::new()
        } else {
            raw.split(opts.sep.as_str()).map(|p| p.trim().to_string()).collect()
        };
        if let Some(count) = opts.count {
            if parts.len() != count {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' must have {count} elements",
                    self.section
                )));
            }
        }
        Ok(Some(parts))
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist is not used by mpu9250")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by mpu9250")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by mpu9250")
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

/// Register file and bulk status shared by one MCU's commands.
struct ChipState {
    regs: GCell<IndexMap<i64, i64>>,
    stuck: GCell<Vec<i64>>,
    status: GCell<Vec<(String, i64)>>,
}

fn status0() -> Vec<(String, i64)> {
    [
        ("clock", 1000),
        ("query_ticks", 40),
        ("next_sequence", 0),
        ("buffered", 0),
        ("possible_overflows", 0),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), *v))
    .collect()
}

fn status1() -> Vec<(String, i64)> {
    [
        ("clock", 101000),
        ("query_ticks", 50),
        ("next_sequence", 3),
        ("buffered", 5),
        ("possible_overflows", 2),
    ]
    .iter()
    .map(|(k, v)| (k.to_string(), *v))
    .collect()
}

struct FakeCmd {
    fmt: String,
    chip: Arc<ChipState>,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        tr(format!(
            "send|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock
        ));
        if self.fmt.starts_with("i2c_transfer") {
            let Some(PyValue::Bytes(b)) = data.get(1) else {
                panic!("i2c_transfer carries bytes")
            };
            let read_len = match data.get(2) {
                Some(PyValue::Int(n)) => i64::try_from(&n.0).unwrap_or(0),
                _ => 0,
            };
            if read_len == 0 && b.len() >= 2 {
                let (reg, val) = (b[0] as i64, b[1] as i64);
                if !self.chip.stuck.borrow().contains(&reg) {
                    self.chip.regs.borrow_mut().insert(reg, val);
                }
            }
        }
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
        if self.fmt.starts_with("i2c_transfer") {
            let Some(PyValue::Bytes(write)) = data.get(1) else {
                panic!("i2c_transfer carries bytes")
            };
            let read_len = match data.get(2) {
                Some(PyValue::Int(n)) => i64::try_from(&n.0).unwrap_or(0),
                _ => 0,
            };
            let reg = write.first().copied().unwrap_or(0) as i64;
            let value = self.chip.regs.borrow().get(&reg).copied().unwrap_or(0);
            let resp = vec![value as u8; read_len.max(0) as usize];
            let mut d = IndexMap::new();
            d.insert(pstr("i2c_bus_status"), pstr("SUCCESS"));
            d.insert(pstr("response"), PyValue::Bytes(resp));
            return Ok(PyValue::Dict(d));
        }
        if self.fmt.starts_with("query_mpu9250_status") {
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
        panic!("send_with_preface is not used by mpu9250")
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
        let mut regs = IndexMap::new();
        regs.insert(0x75, 0x71);
        Arc::new(FakeMcu {
            name: name.to_string(),
            printer: GCell::new(None),
            next_oid: GCell::new(0),
            config_cbs: GCell::new(Vec::new()),
            serial_cbs: GCell::new(Vec::new()),
            chip: Arc::new(ChipState {
                regs: GCell::new(regs),
                stuck: GCell::new(Vec::new()),
                status: GCell::new(status0()),
            }),
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
    fn set_reg(&self, reg: i64, val: i64) {
        self.chip.regs.borrow_mut().insert(reg, val);
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
        panic!("setup_pin is not used by mpu9250")
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
        Ok(Arc::new(FakeCmd { fmt: msgformat.to_string(), chip: self.chip.clone() }))
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
        panic!("get_constant_float is not used by mpu9250")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        (print_time * 1000000.) as i64
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000000.
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by mpu9250")
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
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by mpu9250")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins {
    chips: GCell<IndexMap<String, Mcu>>,
    active: GCell<IndexMap<String, PinParams>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by mpu9250")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc),
            Some((c, p)) => (c.trim(), p.trim()),
        };
        let chip = self.chips.borrow().get(chip_name).cloned();
        let Some(chip) = chip else {
            return Err(KlippyError::Pins(format!("Unknown pin chip name '{chip_name}'")));
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
        panic!("setup_pin is not used by mpu9250")
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) -> klippy_api::KResult<()> {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        self.active.borrow_mut().shift_remove(&share_name);
        Ok(())
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used by mpu9250")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by mpu9250")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by mpu9250")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeToolHead {
    times: GCell<Vec<f64>>,
    on_dwell: GCell<Option<Box<dyn FnMut() + Send>>>,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by mpu9250")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by mpu9250")
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("get_kinematics is not used by mpu9250")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by mpu9250")
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        let mut times = self.times.borrow_mut();
        let t = if times.len() > 1 { times.remove(0) } else { times[0] };
        tr(format!("get_last_move_time|{}", float_repr(t)));
        Ok(t)
    }
    fn dwell(&self, delay: f64) -> KResult<()> {
        tr(format!("dwell|{}", float_repr(delay)));
        let hook = self.on_dwell.borrow_mut().take();
        if let Some(mut hook) = hook {
            hook();
            *self.on_dwell.borrow_mut() = Some(hook);
        }
        Ok(())
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by mpu9250")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by mpu9250")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by mpu9250")
    }
    fn wait_moves(&self) -> KResult<()> {
        tr("wait_moves".to_string());
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by mpu9250")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by mpu9250")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by mpu9250")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by mpu9250")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by mpu9250")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by mpu9250")
    }
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by mpu9250")
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder is not used by mpu9250")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by mpu9250")
    }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by mpu9250")
    }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("get_extra_axes is not used by mpu9250")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by mpu9250")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by mpu9250")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeWebhooks {
    endpoints: GCell<Vec<(String, String, WebHooksCallback)>>,
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        panic!("register_endpoint is not used by mpu9250")
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        tr(format!("mux_endpoint|{path}|{key}|{value}"));
        self.endpoints.borrow_mut().push((path.to_string(), value.to_string(), callback));
        Ok(())
    }
    fn call_remote_method(
        &self,
        _method: &str,
        _params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        panic!("call_remote_method is not used by mpu9250")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);


// ---------------------------------------------------------------------------
// World: the Rust mirror of `make_world()`
// ---------------------------------------------------------------------------

struct Rig {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    aux: Arc<FakeMcu>,
    th: Arc<FakeToolHead>,
    gcode: Arc<GCodeDispatch>,
    chips: GCell<IndexMap<String, Arc<MPU9250>>>,
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
        let gcode = Arc::new(GCodeDispatch::new(printer_api.clone()));
        gcode.register_output_handler(Box::new(|msg| tr(format!("out|{msg}"))));
        let th = Arc::new(FakeToolHead {
            times: GCell::new(vec![100.0]),
            on_dwell: GCell::new(None),
        });
        let wh = Arc::new(FakeWebhooks { endpoints: GCell::new(Vec::new()) });
        printer.add_object("pins", pins).expect("pins");
        printer.add_object("gcode", gcode.clone()).expect("gcode");
        printer.add_object("toolhead", th.clone()).expect("toolhead");
        printer.add_object("webhooks", wh.clone()).expect("webhooks");
        printer.add_object("mcu", mcu.clone()).expect("mcu");
        printer.add_object("mcu aux", aux.clone()).expect("mcu aux");
        take_trace();
        Rig { printer, printer_api, mcu, aux, th, gcode, chips: GCell::new(IndexMap::new()) }
    }

    fn config(&self, section: &str, values: &[(&str, &str)]) -> Config {
        FakeConfig::config(self.printer_api.clone(), section, values)
    }

    fn build(&self, section: &str, values: &[(&str, &str)]) -> Option<Arc<MPU9250>> {
        let cfg = self.config(section, values);
        let factory = module_factory();
        let loader = if section.split_whitespace().count() == 1 {
            factory.load_config
        } else {
            factory.load_config_prefix
        }
        .expect("entry point");
        match loader(&cfg) {
            Ok(Some(obj)) => {
                self.printer.add_object(section, obj.clone()).expect("add_object");
                let chip = downcast_arc::<MPU9250>(&obj).expect("an MPU9250");
                self.chips.borrow_mut().insert(section.to_string(), chip.clone());
                Some(chip)
            }
            Ok(None) => panic!("load_config returned no object"),
            Err(e) => {
                exc(&e);
                None
            }
        }
    }

    fn chip(&self, section: &str) -> Arc<MPU9250> {
        self.chips.borrow().get(section).cloned().expect("chip built")
    }

    fn help_res(&self) {
        let mut items: Vec<(String, String)> = self
            .gcode
            .get_command_help()
            .into_iter()
            .filter(|(k, _)| k.starts_with("ACCELEROMETER_"))
            .collect();
        items.sort();
        let v = PyValue::List(
            items.iter().map(|(k, h)| PyValue::Tuple(vec![pstr(k), pstr(h)])).collect(),
        );
        res("help", &v);
    }

    fn run_cbs(&self) {
        guard(self.mcu.run_config_cbs());
        guard(self.aux.run_config_cbs());
    }

    fn ready(&self) {
        self.printer.send_event("klippy:ready", &[]).expect("ready handlers");
        take_trace();
    }

    fn gcmd(&self, line: &str) {
        guard(self.gcode.process_commands(&[line.to_string()], false));
    }

    fn set_times(&self, times: &[f64]) {
        *self.th.times.borrow_mut() = times.to_vec();
    }

    fn set_dwell_hook(&self, hook: impl FnMut() + Send + 'static) {
        *self.th.on_dwell.borrow_mut() = Some(Box::new(hook));
    }
}

fn pv_axes(axes: &[(usize, f64)]) -> PyValue {
    PyValue::List(
        axes.iter().map(|(i, s)| PyValue::Tuple(vec![int(*i as i64), PyValue::Float(*s)])).collect(),
    )
}

fn py_values(values: &[(&str, &str)]) -> String {
    format!(
        "[{}]",
        values.iter().map(|(k, v)| format!("({k:?}, {v:?})")).collect::<Vec<_>>().join(", ")
    )
}

fn bytes_list(b: &[u8]) -> String {
    format!("{b:?}")
}

/// One big-endian `>hhh` bulk sample.
fn sample_bytes(x: i16, y: i16, z: i16) -> Vec<u8> {
    let mut out = Vec::new();
    for v in [x, y, z] {
        out.extend_from_slice(&v.to_be_bytes());
    }
    out
}

// ---------------------------------------------------------------------------
// Comparison plumbing
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("mpu9250 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping mpu9250 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
    let actual: Vec<String> = take_trace().iter().map(|s| normalize_ts(s)).collect();
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
        unit: "extras.mpu9250".to_string(),
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
fn module_factory_entry_points() {
    let f = module_factory();
    assert!(f.load_config.is_some());
    assert!(f.load_config_prefix.is_some());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn constants_match_reference() {
    let Some(mut py) = oracle() else { return };
    let expected = py
        .eval("(mpu_mod.SCALE, mpu_mod.FREEFALL_ACCEL, mpu_mod.BATCH_UPDATES, mpu_mod.FIFO_SIZE, mpu_mod.MPU9250_ADDR)")
        .expect("py constants");
    let actual = Value::Tuple(vec![
        Value::float(SCALE),
        Value::float(FREEFALL_ACCEL),
        Value::float(BATCH_UPDATES),
        Value::int(512),
        Value::int(0x68),
    ]);
    let record = Record {
        v: 1,
        unit: "extras.mpu9250".to_string(),
        scenario: "constants".to_string(),
        seq: 0,
        function: "constants".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    };
    assert_equal(&record, "ret", &expected, &actual).expect("constants");
    for dev_id in [0x74i64, 0x73, 0x71, 0x70, 0x68, 0x75, 0x69, 0x00, 0x99, -1] {
        let expected = py
            .eval(&format!("mpu_mod.MPU_DEV_IDS.get({dev_id})"))
            .expect("py dev id");
        let actual = match mpu_dev_id(dev_id) {
            Some(name) => Value::str(name),
            None => Value::Null,
        };
        assert_equal(&record, "ret", &expected, &actual).expect("dev id");
    }
    for rate in [4000i64, 3999, 0, -1] {
        let expected = py
            .eval(&format!("mpu_mod.SAMPLE_RATE_DIVS.get({rate})"))
            .expect("py rate");
        let actual = match sample_rate_div(rate) {
            Some(code) => Value::int(code),
            None => Value::Null,
        };
        assert_equal(&record, "ret", &expected, &actual).expect("rate code");
    }
}

#[test]
fn read_axes_map_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let cases: Vec<Vec<(&str, &str)>> = vec![
        vec![],
        vec![("axes_map", "x,y,z")],
        vec![("axes_map", "-x, y ,-z")],
        vec![("axes_map", "z,y,x")],
        vec![("axes_map", "x,y")],
        vec![("axes_map", "x,y,z,x")],
        vec![("axes_map", "q,y,z")],
        vec![("axes_map", "")],
        vec![("axes_map", "x;y;z")],
    ];
    for (n, values) in cases.iter().enumerate() {
        py_run(&mut py, "make_world()");
        py_run(
            &mut py,
            &format!(
                "res('axes', guard(adxl_mod.read_axes_map, make_config('mpu9250', {}), \
                 mpu_mod.SCALE, mpu_mod.SCALE, mpu_mod.SCALE))",
                py_values(values)
            ),
        );
        let rig = Rig::new();
        let cfg = rig.config("mpu9250", values);
        if let Some(axes) = guard(extras::adxl345::read_axes_map(&cfg, SCALE, SCALE, SCALE)) {
            res("axes", &pv_axes(&axes));
        } else {
            res("axes", &PyValue::None);
        }
        check(&format!("axes_map case {n} {values:?}"), &mut py);
    }
}

fn init_case(py: &mut pyoracle::Oracle, label: &str, section: &str, values: &[(&str, &str)], cbs: bool) {
    py_run(py, "make_world()");
    py_run(
        py,
        &format!(
            "chip = guard(build_chip, {section:?}, {})\n\
             help_res()\n\
             if chip is not None:\n    res('attrs', (chip.axes_map, chip.data_rate, chip.oid, chip.name))\n\
             if {}:\n    run_cbs()",
            py_values(values),
            pb(cbs)
        ),
    );
    let rig = Rig::new();
    let chip = rig.build(section, values);
    rig.help_res();
    if let Some(chip) = chip {
        let attrs = PyValue::Tuple(vec![
            pv_axes(&chip.axes_map().expect("axes_map")),
            int(chip.data_rate().expect("data_rate")),
            int(chip.oid().expect("oid")),
            pstr(&chip.name()),
        ]);
        res("attrs", &attrs);
    }
    if cbs {
        rig.run_cbs();
    }
    check(label, py);
}

#[test]
fn init_matches_reference() {
    let Some(mut py) = oracle() else { return };
    init_case(&mut py, "default chip", "mpu9250", &[], true);
    init_case(&mut py, "default chip without callbacks", "mpu9250", &[], false);
    init_case(&mut py, "named chip", "mpu9250 bed", &[], true);
    init_case(&mut py, "explicit rate", "mpu9250", &[("rate", "4000")], true);
    init_case(
        &mut py,
        "custom axes and i2c",
        "mpu9250",
        &[("axes_map", "z,y,x"), ("i2c_mcu", "aux"), ("i2c_speed", "100000"), ("i2c_address", "104")],
        true,
    );
}

#[test]
fn init_errors_match_reference() {
    let Some(mut py) = oracle() else { return };
    for (n, values) in [
        vec![("rate", "1000")],
        vec![("rate", "abc")],
        vec![("axes_map", "x,y")],
        vec![("axes_map", "q,y,z")],
        vec![("i2c_speed", "100")],
        vec![("i2c_address", "200")],
        vec![("rate", "4000"), ("axes_map", "-y,-x,-z")],
    ]
    .iter()
    .enumerate()
    {
        py_run(&mut py, "make_world()");
        py_run(
            &mut py,
            &format!(
                "chip = guard(build_chip, 'mpu9250', {})\n\
                 if chip is not None:\n    help_res()\n    res('attrs', (chip.axes_map, chip.data_rate, chip.oid, chip.name))",
                py_values(values)
            ),
        );
        let rig = Rig::new();
        let chip = rig.build("mpu9250", values);
        if let Some(chip) = chip {
            rig.help_res();
            let attrs = PyValue::Tuple(vec![
                pv_axes(&chip.axes_map().expect("axes_map")),
                int(chip.data_rate().expect("data_rate")),
                int(chip.oid().expect("oid")),
                pstr(&chip.name()),
            ]);
            res("attrs", &attrs);
        }
        check(&format!("init error case {n} {values:?}"), &mut py);
    }
}

#[test]
fn register_access_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "chip = build_chip('mpu9250', [])\n\
         run_cbs()\n\
         res('dev_id', chip.read_reg(0x75))\n\
         res('reg_0x6b', chip.read_reg(0x6b))\n\
         guard(chip.set_reg, 0x6b, 0x140)\n\
         res('reg_0x6b_after', chip.read_reg(0x6b))\n\
         guard(chip.set_reg, 0x1c, -1)\n\
         res('reg_0x1c_after', chip.read_reg(0x1c))\n\
         res('empty_read', guard(chip.read_reg, 0x19))",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    rig.run_cbs();
    if let Some(v) = guard(chip.read_reg(0x75)) {
        res("dev_id", &int(v));
    } else {
        res("dev_id", &PyValue::None);
    }
    if let Some(v) = guard(chip.read_reg(0x6b)) {
        res("reg_0x6b", &int(v));
    } else {
        res("reg_0x6b", &PyValue::None);
    }
    guard(chip.set_reg_minclock(0x6b, 0x140, 0));
    if let Some(v) = guard(chip.read_reg(0x6b)) {
        res("reg_0x6b_after", &int(v));
    } else {
        res("reg_0x6b_after", &PyValue::None);
    }
    guard(chip.set_reg_minclock(0x1c, -1, 0));
    if let Some(v) = guard(chip.read_reg(0x1c)) {
        res("reg_0x1c_after", &int(v));
    } else {
        res("reg_0x1c_after", &PyValue::None);
    }
    // No value was written to 0x19: both sides return the default 0.
    if let Some(v) = guard(chip.read_reg(0x19)) {
        res("empty_read", &int(v));
    } else {
        res("empty_read", &PyValue::None);
    }
    check("register access", &mut py);

    // The same traffic through the ACCELEROMETER_DEBUG_* commands.
    py_run(&mut py, "ready()\ngcmd_run('ACCELEROMETER_DEBUG_READ REG=0x6b')\ngcmd_run('ACCELEROMETER_DEBUG_WRITE REG=0x6b VAL=0x2a')");
    rig.ready();
    rig.gcmd("ACCELEROMETER_DEBUG_READ REG=0x6b");
    rig.gcmd("ACCELEROMETER_DEBUG_WRITE REG=0x6b VAL=0x2a");
    check("debug commands", &mut py);

    // The AccelChipApi trait view returns the same values.
    py_run(
        &mut py,
        "res('trait_dev_id', CH['mpu9250'].read_reg(0x75))\n\
         res('trait_name', CH['mpu9250'].name)",
    );
    take_trace();
    let api: &dyn AccelChipApi = chip.as_ref();
    if let Some(v) = guard(api.read_reg(0x75)) {
        res("trait_dev_id", &int(v));
    } else {
        res("trait_dev_id", &PyValue::None);
    }
    res("trait_name", &pstr(&api.name()));
    check("chip api trait", &mut py);
}

#[test]
fn convert_samples_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let cases = [
        vec![],
        vec![(1000.0, 0i64, 0i64, 0i64)],
        vec![(1000.5, 4096, -8192, 12345)],
        vec![(1.1234567, -32768, 32767, -1)],
        vec![(2.0, 1, 2, 3), (3.0, -1, -2, -3)],
        vec![(9.0, 4000, 4000, 4000)],
    ];
    for (n, rows) in cases.iter().enumerate() {
        py_run(&mut py, "make_world()");
        let py_rows = format!(
            "[{}]",
            rows.iter()
                .map(|(t, x, y, z)| format!("({t:?}, {x}, {y}, {z})"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        py_run(
            &mut py,
            &format!(
                "chip = build_chip('mpu9250', [])\n\
                 samples = {py_rows}\n\
                 chip._convert_samples(samples)\n\
                 res('converted', samples)"
            ),
        );
        let rig = Rig::new();
        let chip = rig.build("mpu9250", &[]).expect("chip");
        let mut samples: Vec<PyValue> = rows
            .iter()
            .map(|(t, x, y, z)| {
                PyValue::Tuple(vec![
                    PyValue::Float(*t),
                    int(*x),
                    int(*y),
                    int(*z),
                ])
            })
            .collect();
        guard(chip._convert_samples(&mut samples));
        res("converted", &PyValue::List(samples));
        check(&format!("convert case {n}"), &mut py);
    }

    // A reordered / negated axes map applies the same permutation.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "chip = build_chip('mpu9250', [('axes_map', '-z,y,-x')])\n\
         samples = [(1.0, 100, 200, 300)]\n\
         chip._convert_samples(samples)\n\
         res('converted', samples)",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[("axes_map", "-z,y,-x")]).expect("chip");
    let mut samples = vec![PyValue::Tuple(vec![
        PyValue::Float(1.0),
        int(100),
        int(200),
        int(300),
    ])];
    guard(chip._convert_samples(&mut samples));
    res("converted", &PyValue::List(samples));
    check("convert reordered axes", &mut py);
}

#[test]
fn start_finish_measurements_match_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build_chip('mpu9250', [])\n\
         run_cbs()\n\
         res('start', guard(CH['mpu9250']._start_measurements))\n\
         res('finish', guard(CH['mpu9250']._finish_measurements))",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    rig.run_cbs();
    guard(chip._start_measurements());
    res("start", &PyValue::None);
    guard(chip._finish_measurements());
    res("finish", &PyValue::None);
    check("start and finish", &mut py);

    // An unknown device id is a command error before any register write.
    py_run(
        &mut py,
        "MCU.regs[0x75] = 0x99\n\
         res('start', guard(CH['mpu9250']._start_measurements))",
    );
    rig.mcu.set_reg(0x75, 0x99);
    guard(chip._start_measurements());
    res("start", &PyValue::None);
    check("invalid device id", &mut py);

    // A defective-but-known id logs the "Found" line and proceeds.
    py_run(
        &mut py,
        "MCU.regs[0x75] = 0x75\n\
         res('start', guard(CH['mpu9250']._start_measurements))\n\
         res('finish', guard(CH['mpu9250']._finish_measurements))",
    );
    rig.mcu.set_reg(0x75, 0x75);
    guard(chip._start_measurements());
    res("start", &PyValue::None);
    guard(chip._finish_measurements());
    res("finish", &PyValue::None);
    check("defective device id", &mut py);
}

#[test]
fn process_batch_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build_chip('mpu9250', [])\n\
         run_cbs()\n\
         CH['mpu9250']._start_measurements()\n\
         res('empty', guard(CH['mpu9250']._process_batch, 1000.5))\n\
         MCU.status = dict(STATUS1)\n\
         feed(MCU, 1, frame(1000, -2000, 3000) + frame(-4000, 5000, -6000))\n\
         res('batch', guard(CH['mpu9250']._process_batch, 1000.5))",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    rig.run_cbs();
    guard(chip._start_measurements());
    if let Some(v) = guard(chip._process_batch(1000.5)) {
        res("empty", &v);
    } else {
        res("empty", &PyValue::None);
    }
    *rig.mcu.chip.status.borrow_mut() = status1();
    let mut bytes = sample_bytes(1000, -2000, 3000);
    bytes.extend_from_slice(&sample_bytes(-4000, 5000, -6000));
    rig.mcu.feed(1, &bytes);
    if let Some(v) = guard(chip._process_batch(1000.5)) {
        res("batch", &v);
    } else {
        res("batch", &PyValue::None);
    }
    check("process batch", &mut py);
}

/// Both sides: the dwell hook of the query command feeds one batch of frames
/// and runs the periodic batch callback, as the reactor timer would.
fn py_dwell_hook(chip: &str, frames: &[u8]) -> String {
    format!(
        "def hook():\n    MCU.status = dict(STATUS1)\n    feed(MCU, 1, {})\n    \
         CH[{chip:?}].batch_bulk._proc_batch(1000.5)\nTH.on_dwell = hook",
        bytes_list(frames)
    )
}

fn rs_dwell_hook(rig: &Rig, chip: &str, frames: Vec<u8>) {
    let mcu = rig.mcu.clone();
    let chip = rig.chip(chip);
    rig.set_dwell_hook(move || {
        *mcu.chip.status.borrow_mut() = status1();
        mcu.feed(1, &frames);
        guard(chip.batch_bulk().expect("batch_bulk")._proc_batch(1000.5));
    });
}

#[test]
fn query_command_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "build_chip('mpu9250', [])\n\
         build_chip('mpu9250 bed', [('i2c_mcu', 'aux')])\n\
         run_cbs()\n\
         ready()",
    );
    let rig = Rig::new();
    rig.build("mpu9250", &[]).expect("chip");
    rig.build("mpu9250 bed", &[("i2c_mcu", "aux")]).expect("chip");
    rig.run_cbs();
    rig.ready();

    let mut fa = sample_bytes(4096, 8192, -4096);
    fa.extend_from_slice(&sample_bytes(-100, 200, -300));
    // Window [0, 5] holds every sample.
    py_run(&mut py, "TH.times = [0.0, 5.0]");
    py_run(&mut py, &py_dwell_hook("mpu9250", &fa));
    py_run(&mut py, "gcmd_run('ACCELEROMETER_QUERY CHIP=mpu9250')");
    rig.set_times(&[0.0, 5.0]);
    rs_dwell_hook(&rig, "mpu9250", fa.clone());
    rig.gcmd("ACCELEROMETER_QUERY CHIP=mpu9250");
    check("query with samples", &mut py);

    // The window starts after the last sample: nothing is found.
    py_run(&mut py, "TH.on_dwell = None\nTH.times = [900.0, 901.0]\ngcmd_run('ACCELEROMETER_QUERY CHIP=bed')");
    rig.set_times(&[900.0, 901.0]);
    *rig.th.on_dwell.borrow_mut() = None;
    rig.gcmd("ACCELEROMETER_QUERY CHIP=bed");
    check("query without samples", &mut py);

    // The default (unnamed) selection reaches the first chip.
    py_run(&mut py, "TH.on_dwell = None\nTH.times = [0.0]\ngcmd_run('ACCELEROMETER_QUERY')");
    rig.set_times(&[0.0]);
    rig.gcmd("ACCELEROMETER_QUERY");
    check("query default chip", &mut py);
}

#[test]
fn start_failures_match_reference() {
    let Some(mut py) = oracle() else { return };
    // Starting before the config callbacks ran: the I2C query command
    // was never looked up.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "chip = build_chip('mpu9250', [])\n\
         res('started', guard(chip.start_internal_client) is not None)",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    let client = guard(chip.start_internal_client());
    res("started", &PyValue::Bool(client.is_some()));
    check("start before config callbacks", &mut py);

    // An unknown device id stops the batch helper.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        "chip = build_chip('mpu9250', [])\n\
         run_cbs()\n\
         MCU.regs[0x75] = 0x00\n\
         res('started', guard(chip.start_internal_client) is not None)\n\
         res('is_started', chip.batch_bulk.is_started)",
    );
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    rig.run_cbs();
    rig.mcu.set_reg(0x75, 0x00);
    let client = guard(chip.start_internal_client());
    res("started", &PyValue::Bool(client.is_some()));
    res("is_started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    check("unknown device id", &mut py);
}

#[test]
fn chip_api_object_is_reachable_through_the_printer() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()\nbuild_chip('mpu9250', [])");
    let rig = Rig::new();
    let chip = rig.build("mpu9250", &[]).expect("chip");
    let obj = rig
        .printer
        .lookup_object("mpu9250", LookupDefault::Raise)
        .expect("lookup")
        .expect("object");
    assert!(downcast_arc::<MPU9250>(&obj).is_some());
    assert_eq!(chip.name(), "mpu9250");
    take_trace();
}
