//! Parity tests for `extras::lis2dw` (port of `klippy/extras/lis2dw.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). The Python side runs the real
//! `extras.lis2dw`, `extras.adxl345`, `extras.bus`, `extras.bulk_sensor`,
//! `gcode` and `configfile` modules over minimal fakes of the printer,
//! reactor, MCU, pins, toolhead and webhooks. The Rust side runs the port,
//! the real `extras::adxl345` / `extras::bus` / `extras::bulk_sensor` ports
//! and the real `klippy_core::gcode::GCodeDispatch` over the same fakes.
//!
//! Both sides append one text line per observable effect to a trace: MCU
//! config commands, command lookups, commands sent (format and argument
//! values), SPI and I2C register traffic, toolhead calls, webhooks
//! registrations and sends, G-code output lines, raised errors (class and
//! message), the `logging` records this module and the batch helper emit, and
//! the result values (`repr`, so floats compare by their exact shortest
//! spelling). The two traces must be identical.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this unit.

use extras::adxl345::AccelChipApi;
use extras::lis2dw::{
    module_factory, BATCH_UPDATES, FREEFALL_ACCEL, I2C_SERIAL_TYPE, LIS2DW, LIS2DW_DEV_ID,
    LIS2DW_SCALE, LIS2DW_TYPE, LIS3DH_DEV_ID, LIS3DH_SCALE, LIS3DH_TYPE, LIS_I2C_ADDR,
    REG_LIS2DW_CTRL_REG1_ADDR, REG_LIS2DW_CTRL_REG2_ADDR, REG_LIS2DW_CTRL_REG3_ADDR,
    REG_LIS2DW_CTRL_REG4_ADDR, REG_LIS2DW_CTRL_REG5_ADDR, REG_LIS2DW_CTRL_REG6_ADDR,
    REG_LIS2DW_FIFO_CTRL, REG_LIS2DW_FIFO_SAMPLES, REG_LIS2DW_OUT_XH_ADDR,
    REG_LIS2DW_OUT_XL_ADDR, REG_LIS2DW_OUT_YH_ADDR, REG_LIS2DW_OUT_YL_ADDR,
    REG_LIS2DW_OUT_ZH_ADDR, REG_LIS2DW_OUT_ZL_ADDR, REG_LIS2DW_STATUS_REG_ADDR,
    REG_LIS2DW_WHO_AM_I_ADDR, REG_MOD_READ, SPI_SERIAL_TYPE,
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
use klippy_api::webhooks::{
    ClientConnection, ClientConnectionApi, WebHooksApi, WebHooksCallback, WebRequestApi,
};
use klippy_api::{downcast_arc, EventArg, GCell, ObjectRef};
use klippy_core::gcode::GCodeDispatch;
use pycompat::text::{float_repr, str_repr};
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

fn peek_trace() -> Vec<String> {
    TRACE.with(|t| t.borrow().clone())
}

/// Only the log records this module and the batch helper emit are traced;
/// the G-code and bus ports log their own messages.
fn install_log_sink() {
    static ONCE: Once = Once::new();
    ONCE.call_once(|| {
        klippy_api::log::set_sink(Box::new(|rec| {
            let msg = rec.message;
            if msg.starts_with("LIS2DW")
                || msg.starts_with("LIS3DH")
                || msg.starts_with("BatchBulkHelper")
            {
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
import sys, types, configparser, re, os
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
        if text.startswith(("LIS2DW", "LIS3DH", "BatchBulkHelper")):
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
import extras.lis2dw as lis_mod

for _m in (gcode_mod, bulk_mod, bus_mod, adxl_mod, lis_mod):
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
        if self.fmt.startswith("spi_send"):
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
        if self.fmt.startswith("spi_transfer"):
            reg = d[1][0] & 0x7f
            return {"response": bytes([0, self.mcu.regs.get(reg, 0)])}
        if self.fmt.startswith("i2c_transfer"):
            w = d[1]
            status = self.mcu.i2c_status
            resp = b""
            if status == "SUCCESS":
                if len(w) == 2:
                    if w[0] not in self.mcu.stuck:
                        self.mcu.regs[w[0]] = w[1]
                else:
                    resp = bytes([self.mcu.regs.get(w[0], 0)])
            return {"i2c_bus_status": status, "response": resp}
        if self.fmt.startswith("query_lis2dw_status"):
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
        self.regs = {0x0f: 0x44}
        self.stuck = set()
        self.status = dict(STATUS0)
        self.i2c_status = "SUCCESS"
    def get_name(self):
        return self.name
    def get_printer(self):
        return self.printer
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
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
    def is_fileoutput(self):
        return False
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
    if section.split()[0] == "lis3dh":
        chip = lis_mod.LIS2DW(cfg, lis_mod.LIS3DH_TYPE)
    elif len(section.split()) == 1:
        chip = lis_mod.load_config(cfg)
    else:
        chip = lis_mod.load_config_prefix(cfg)
    P.objects[section] = chip
    CH[section] = chip
    return chip

def help_res():
    res("help", sorted((k, h) for k, h in GC.get_command_help().items() if k.startswith("ACCELEROMETER_")))

def run_cbs():
    MCU.run_config_cbs()
    AUX.run_config_cbs()

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
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by adxl345")
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
/// `configfile.py` semantics of the getters `adxl345` and `bus` call.
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
        panic!("getfloat is not used by adxl345")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean is not used by adxl345")
    }
    fn getchoice(
        &self,
        _option: &str,
        _choices: &PyValue,
        _opts: GetChoiceOpts,
    ) -> KResult<PyValue> {
        panic!("getchoice is not used by adxl345")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists is not used by adxl345")
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
        panic!("getintlist is not used by adxl345")
    }
    fn getfloatlist(
        &self,
        _option: &str,
        _opts: GetFloatListOpts,
    ) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist is not used by adxl345")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection is not used by adxl345")
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

/// Register file and bulk status shared by one MCU's commands.
struct ChipState {
    regs: GCell<IndexMap<i64, i64>>,
    stuck: GCell<Vec<i64>>,
    status: GCell<Vec<(String, i64)>>,
    i2c_status: GCell<String>,
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
        if self.fmt.starts_with("spi_send") {
            let Some(PyValue::Bytes(b)) = data.get(1) else {
                panic!("spi_send carries bytes")
            };
            let (reg, val) = (b[0] as i64, b[1] as i64);
            if !self.chip.stuck.borrow().contains(&reg) {
                self.chip.regs.borrow_mut().insert(reg, val);
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
        if self.fmt.starts_with("spi_transfer") {
            let Some(PyValue::Bytes(b)) = data.get(1) else {
                panic!("spi_transfer carries bytes")
            };
            let reg = (b[0] & 0x7f) as i64;
            let value = self.chip.regs.borrow().get(&reg).copied().unwrap_or(0);
            let mut d = IndexMap::new();
            d.insert(pstr("response"), PyValue::Bytes(vec![0, value as u8]));
            return Ok(PyValue::Dict(d));
        }
        if self.fmt.starts_with("i2c_transfer") {
            let Some(PyValue::Bytes(w)) = data.get(1) else {
                panic!("i2c_transfer carries bytes")
            };
            let status = self.chip.i2c_status.borrow().clone();
            let mut resp = Vec::new();
            if status == "SUCCESS" {
                if w.len() == 2 {
                    if !self.chip.stuck.borrow().contains(&(w[0] as i64)) {
                        self.chip.regs.borrow_mut().insert(w[0] as i64, w[1] as i64);
                    }
                } else {
                    let value = self.chip.regs.borrow().get(&(w[0] as i64)).copied().unwrap_or(0);
                    resp.push(value as u8);
                }
            }
            let mut d = IndexMap::new();
            d.insert(pstr("i2c_bus_status"), PyValue::Str(status));
            d.insert(pstr("response"), PyValue::Bytes(resp));
            return Ok(PyValue::Dict(d));
        }
        if self.fmt.starts_with("query_lis2dw_status") {
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
        panic!("send_with_preface is not used by adxl345")
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
        regs.insert(0x0f, 0x44);
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
                i2c_status: GCell::new("SUCCESS".to_string()),
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
    fn stick(&self, reg: i64) {
        self.chip.stuck.borrow_mut().push(reg);
    }
    fn set_i2c_status(&self, status: &str) {
        *self.chip.i2c_status.borrow_mut() = status.to_string();
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
        panic!("setup_pin is not used by adxl345")
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
        pycompat::text::trunc_float_to_bigint(time * 1000000.)
            .into_bigint()
            .try_into()
            .expect("fake MCU clock fits i64")
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
        panic!("get_constant_float is not used by adxl345")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by adxl345")
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 1000000.
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by adxl345")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by adxl345")
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
        panic!("create_trigger_dispatch is not used by adxl345")
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins {
    chips: GCell<IndexMap<String, Mcu>>,
    active: GCell<IndexMap<String, PinParams>>,
}

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by adxl345")
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
        panic!("setup_pin is not used by adxl345")
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) {
        let share_name = format!("{}:{}", pin_params.chip_name, pin_params.pin);
        self.active.borrow_mut().shift_remove(&share_name);
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<PinResolver> {
        panic!("get_pin_resolver is not used by adxl345")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by adxl345")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by adxl345")
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeToolHead {
    times: GCell<Vec<f64>>,
    on_dwell: GCell<Option<Box<dyn FnMut() + Send>>>,
}

impl HomingTargetApi for FakeToolHead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by adxl345")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by adxl345")
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("get_kinematics is not used by adxl345")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by adxl345")
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
        panic!("drip_move is not used by adxl345")
    }
}

impl ToolHeadApi for FakeToolHead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by adxl345")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by adxl345")
    }
    fn wait_moves(&self) -> KResult<()> {
        tr("wait_moves".to_string());
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by adxl345")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by adxl345")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by adxl345")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by adxl345")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by adxl345")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by adxl345")
    }
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by adxl345")
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder is not used by adxl345")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> {
        panic!("add_extra_axis is not used by adxl345")
    }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> {
        panic!("remove_extra_axis is not used by adxl345")
    }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("get_extra_axes is not used by adxl345")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by adxl345")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by adxl345")
    }
}

klippy_api::printer_object!(FakeToolHead; dyn ToolHeadApi);

struct FakeWebhooks {
    endpoints: GCell<Vec<(String, String, WebHooksCallback)>>,
}

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(&self, _path: &str, _callback: WebHooksCallback) -> KResult<()> {
        panic!("register_endpoint is not used by adxl345")
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
        panic!("call_remote_method is not used by adxl345")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::None
    }
}

klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

struct FakeConn {
    closed: GCell<bool>,
}

impl ClientConnectionApi for FakeConn {
    fn uid(&self) -> u64 {
        0
    }
    fn dump_request_log(&self) {}
    fn set_client_info(&self, _client_info: PyValue, _state_msg: Option<String>) {}
    fn close(&self) {
        self.closed.set(true);
    }
    fn is_closed(&self) -> bool {
        self.closed.get()
    }
    fn send(&self, data: PyValue) {
        tr(format!("conn_send|{}", data.repr()));
    }
}

struct FakeWebRequest {
    conn: Arc<FakeConn>,
}

impl WebRequestApi for FakeWebRequest {
    fn client_conn(&self) -> ClientConnection {
        self.conn.clone()
    }
    fn id(&self) -> Option<PyValue> {
        None
    }
    fn method(&self) -> String {
        String::new()
    }
    fn params(&self) -> PyValue {
        PyValue::None
    }
    fn response(&self) -> Option<PyValue> {
        None
    }
    fn is_error(&self) -> bool {
        false
    }
    fn get_client_connection(&self) -> ClientConnection {
        self.conn.clone()
    }
    fn get(&self, _item: &str, _default: Option<PyValue>) -> KResult<PyValue> {
        panic!("get is not used by adxl345")
    }
    fn get_str(&self, _item: &str, _default: Option<String>) -> KResult<String> {
        panic!("get_str is not used by adxl345")
    }
    fn get_int(&self, _item: &str, _default: Option<i64>) -> KResult<i64> {
        panic!("get_int is not used by adxl345")
    }
    fn get_float(&self, _item: &str, _default: Option<f64>) -> KResult<f64> {
        panic!("get_float is not used by adxl345")
    }
    fn get_dict(&self, _item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        Ok(default.unwrap_or(PyValue::None))
    }
    fn get_method(&self) -> String {
        String::new()
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        tr(format!("req_send|{}", data.repr()));
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        None
    }
}

// ---------------------------------------------------------------------------
// World: the Rust mirror of `make_world()`
// ---------------------------------------------------------------------------

struct Rig {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    aux: Arc<FakeMcu>,
    th: Arc<FakeToolHead>,
    wh: Arc<FakeWebhooks>,
    gcode: Arc<GCodeDispatch>,
    chips: GCell<IndexMap<String, Arc<LIS2DW>>>,
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
        Rig { printer, printer_api, mcu, aux, th, wh, gcode, chips: GCell::new(IndexMap::new()) }
    }

    fn config(&self, section: &str, values: &[(&str, &str)]) -> Config {
        FakeConfig::config(self.printer_api.clone(), section, values)
    }

    /// `guard(build_chip, section, values)`.
    fn build(&self, section: &str, values: &[(&str, &str)]) -> Option<Arc<LIS2DW>> {
        let cfg = self.config(section, values);
        let result = if section.starts_with("lis3dh") {
            LIS2DW::new(&cfg, LIS3DH_TYPE).map(|chip| {
                let obj: ObjectRef = chip;
                Some(obj)
            })
        } else {
            let factory = module_factory();
            let loader = if section.split_whitespace().count() == 1 {
                factory.load_config
            } else {
                factory.load_config_prefix
            }
            .expect("entry point");
            loader(&cfg)
        };
        match result {
            Ok(Some(obj)) => {
                self.printer.add_object(section, obj.clone()).expect("add_object");
                let chip = downcast_arc::<LIS2DW>(&obj).expect("a LIS2DW");
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

    fn chip(&self, section: &str) -> Arc<LIS2DW> {
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
        self.mcu.run_config_cbs().expect("main MCU config callbacks");
        self.aux.run_config_cbs().expect("aux MCU config callbacks");
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

/// Deterministic raw bulk data: `n` frames of three little-endian `i16`.
fn frames(seed: u32, n: usize) -> Vec<u8> {
    let mut s = seed;
    let mut next = move || {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        (s >> 24) as u8
    };
    (0..n * 6).map(|_| next()).collect()
}

fn bytes_list(b: &[u8]) -> String {
    format!("{b:?}")
}

// ---------------------------------------------------------------------------
// Comparison plumbing
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("lis2dw fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping lis2dw oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.lis2dw".to_string(),
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

fn wait_for_file(path: &str) -> String {
    for _ in 0..200 {
        if let Ok(s) = std::fs::read_to_string(path) {
            if !s.is_empty() {
                let _ = std::fs::remove_file(path);
                return s;
            }
        }
        kos::time::sleep(0.05);
    }
    panic!("{path} was not written");
}

fn tmp_name(tag: &str) -> String {
    format!("t{}_{tag}", std::process::id())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn record(scenario: &str) -> Record {
    Record {
        v: 1,
        unit: "extras.lis2dw".to_string(),
        scenario: scenario.to_string(),
        seq: 0,
        function: "value".to_string(),
        args: Vec::new(),
        ret: None,
        exc: None,
        side_effects: SideEffects::default(),
        object_id: None,
        kind: RecordKind::Python,
    }
}

const SPI_PA4: &[(&str, &str)] = &[("cs_pin", "PA4")];
const I2C_AUX: &[(&str, &str)] = &[("i2c_mcu", "aux")];

#[test]
fn module_factory_entry_points() {
    let Some(mut py) = oracle() else { return };
    let names = [
        "load_config", "load_config_prefix", "add_early_printer_objects",
        "add_printer_objects",
    ];
    let f = module_factory();
    let actual = [
        f.load_config.is_some(), f.load_config_prefix.is_some(),
        f.add_early_printer_objects.is_some(), f.add_printer_objects.is_some(),
    ];
    for (name, present) in names.into_iter().zip(actual) {
        let expected = py.eval(&format!("callable(getattr(lis_mod, {name:?}, None))"))
            .expect("reference entry point");
        assert_equal(&record(name), "ret", &expected, &Value::Bool(present))
            .expect("module entry point parity");
    }
}

#[test]
fn constants_match_reference() {
    let Some(mut py) = oracle() else { return };
    let ints: Vec<(&str, i64)> = vec![
        ("REG_LIS2DW_WHO_AM_I_ADDR", REG_LIS2DW_WHO_AM_I_ADDR),
        ("REG_LIS2DW_CTRL_REG1_ADDR", REG_LIS2DW_CTRL_REG1_ADDR),
        ("REG_LIS2DW_CTRL_REG2_ADDR", REG_LIS2DW_CTRL_REG2_ADDR),
        ("REG_LIS2DW_CTRL_REG3_ADDR", REG_LIS2DW_CTRL_REG3_ADDR),
        ("REG_LIS2DW_CTRL_REG4_ADDR", REG_LIS2DW_CTRL_REG4_ADDR),
        ("REG_LIS2DW_CTRL_REG5_ADDR", REG_LIS2DW_CTRL_REG5_ADDR),
        ("REG_LIS2DW_CTRL_REG6_ADDR", REG_LIS2DW_CTRL_REG6_ADDR),
        ("REG_LIS2DW_STATUS_REG_ADDR", REG_LIS2DW_STATUS_REG_ADDR),
        ("REG_LIS2DW_OUT_XL_ADDR", REG_LIS2DW_OUT_XL_ADDR),
        ("REG_LIS2DW_OUT_XH_ADDR", REG_LIS2DW_OUT_XH_ADDR),
        ("REG_LIS2DW_OUT_YL_ADDR", REG_LIS2DW_OUT_YL_ADDR),
        ("REG_LIS2DW_OUT_YH_ADDR", REG_LIS2DW_OUT_YH_ADDR),
        ("REG_LIS2DW_OUT_ZL_ADDR", REG_LIS2DW_OUT_ZL_ADDR),
        ("REG_LIS2DW_OUT_ZH_ADDR", REG_LIS2DW_OUT_ZH_ADDR),
        ("REG_LIS2DW_FIFO_CTRL", REG_LIS2DW_FIFO_CTRL),
        ("REG_LIS2DW_FIFO_SAMPLES", REG_LIS2DW_FIFO_SAMPLES),
        ("REG_MOD_READ", REG_MOD_READ),
        ("LIS2DW_DEV_ID", LIS2DW_DEV_ID),
        ("LIS3DH_DEV_ID", LIS3DH_DEV_ID),
        ("LIS_I2C_ADDR", LIS_I2C_ADDR),
    ];
    for (name, value) in ints {
        let expected = py.eval(&format!("lis_mod.{name}")).expect("py constant");
        assert_equal(&record(name), "ret", &expected, &Value::int(value)).expect(name);
    }
    let floats: Vec<(&str, f64)> = vec![
        ("FREEFALL_ACCEL", FREEFALL_ACCEL),
        ("LIS2DW_SCALE", LIS2DW_SCALE),
        ("LIS3DH_SCALE", LIS3DH_SCALE),
        ("BATCH_UPDATES", BATCH_UPDATES),
    ];
    for (name, value) in floats {
        let expected = py.eval(&format!("lis_mod.{name}")).expect("py constant");
        assert_equal(&record(name), "ret", &expected, &Value::float(value)).expect(name);
    }
    let strs: Vec<(&str, &str)> = vec![
        ("LIS2DW_TYPE", LIS2DW_TYPE),
        ("LIS3DH_TYPE", LIS3DH_TYPE),
        ("SPI_SERIAL_TYPE", SPI_SERIAL_TYPE),
        ("I2C_SERIAL_TYPE", I2C_SERIAL_TYPE),
    ];
    for (name, value) in strs {
        let expected = py.eval(&format!("lis_mod.{name}")).expect("py constant");
        assert_equal(&record(name), "ret", &expected, &Value::str(value)).expect(name);
    }
}

/// One build of a chip with an optional run of the MCU config callbacks.
fn init_case(
    py: &mut pyoracle::Oracle,
    label: &str,
    section: &str,
    values: &[(&str, &str)],
    cbs: bool,
) {
    py_run(py, "make_world()");
    py_run(
        py,
        &[
            format!("chip = guard(build_chip, {section:?}, {})", py_values(values)),
            "help_res()".to_string(),
            "if chip is not None:".to_string(),
            "    res('attrs', (chip.axes_map, chip.data_rate, chip.oid, chip.name, chip.bus_type, chip.lis_type))"
                .to_string(),
            format!("if {}:", pb(cbs)),
            "    run_cbs()".to_string(),
        ]
        .join("\n"),
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
            pstr(chip.bus_type().expect("bus_type")),
            pstr(chip.lis_type()),
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
    init_case(&mut py, "spi chip", "lis2dw", SPI_PA4, true);
    init_case(&mut py, "spi chip without callbacks", "lis2dw", SPI_PA4, false);
    init_case(&mut py, "i2c chip", "lis2dw", &[], true);
    init_case(&mut py, "i2c chip without callbacks", "lis2dw", &[], false);
    init_case(
        &mut py,
        "named spi chip on aux",
        "lis2dw bed",
        &[
            ("cs_pin", "aux:PB1"),
            ("axes_map", "-z,x,-y"),
            ("spi_speed", "2000000"),
            ("spi_bus", "spi1"),
        ],
        true,
    );
    init_case(
        &mut py,
        "named i2c chip on aux",
        "lis2dw bed",
        &[
            ("i2c_mcu", "aux"),
            ("i2c_address", "24"),
            ("i2c_speed", "100000"),
            ("i2c_bus", "i2c1"),
            ("axes_map", "y,-x,z"),
        ],
        true,
    );
    init_case(&mut py, "lis3dh spi", "lis3dh", SPI_PA4, true);
    init_case(&mut py, "lis3dh i2c", "lis3dh x", &[], true);
    init_case(&mut py, "lis3dh i2c on aux", "lis3dh y", I2C_AUX, false);
    init_case(&mut py, "single word other name", "bar", SPI_PA4, true);
}

#[test]
fn init_errors_match_reference() {
    let Some(mut py) = oracle() else { return };
    type InitCase<'a> = (&'a str, &'a str, Vec<(&'a str, &'a str)>);
    let cases: Vec<InitCase> = vec![
        ("short axes_map", "lis2dw", vec![("cs_pin", "PA4"), ("axes_map", "x,y")]),
        ("invalid axes_map", "lis2dw", vec![("cs_pin", "PA4"), ("axes_map", "a,b,c")]),
        ("invalid axes_map on i2c", "lis2dw", vec![("axes_map", "x,y,q")]),
        ("lis3dh invalid axes_map", "lis3dh", vec![("axes_map", "x;y;z")]),
        ("unknown pin chip", "lis2dw", vec![("cs_pin", "zzz:PA1")]),
        ("slow spi", "lis2dw", vec![("cs_pin", "PA4"), ("spi_speed", "5000")]),
        ("unknown i2c mcu", "lis2dw", vec![("i2c_mcu", "zzz")]),
        ("i2c address too large", "lis2dw", vec![("i2c_address", "200")]),
        ("i2c address negative", "lis2dw", vec![("i2c_address", "-1")]),
        ("slow i2c", "lis2dw", vec![("i2c_speed", "5000")]),
        ("unparsable i2c speed", "lis2dw", vec![("i2c_speed", "fast")]),
    ];
    for (label, section, values) in cases {
        init_case(&mut py, label, section, &values, true);
    }

    // Name clashes: the later chips reuse a name or the default slot.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "guard(build_chip, 'lis2dw bed', [('cs_pin', 'PA4')])",
            "guard(build_chip, 'lis2dw bed', [('cs_pin', 'PA5')])",
            "guard(build_chip, 'lis2dw', [('cs_pin', 'PA6')])",
            "guard(build_chip, 'bar', [('cs_pin', 'PA7')])",
            "guard(build_chip, 'lis2dw', [('cs_pin', 'PA8')])",
            "guard(build_chip, 'lis3dh', [('cs_pin', 'PA9')])",
            "guard(build_chip, 'lis3dh x', [('cs_pin', 'PB1')])",
            "help_res()",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    rig.build("lis2dw bed", &[("cs_pin", "PA4")]);
    rig.build("lis2dw bed", &[("cs_pin", "PA5")]);
    rig.build("lis2dw", &[("cs_pin", "PA6")]);
    rig.build("bar", &[("cs_pin", "PA7")]);
    rig.build("lis2dw", &[("cs_pin", "PA8")]);
    rig.build("lis3dh", &[("cs_pin", "PA9")]);
    rig.build("lis3dh x", &[("cs_pin", "PB1")]);
    rig.help_res();
    check("name clashes", &mut py);
}

fn opt_int(v: KResult<i64>) -> PyValue {
    guard(v).map_or(PyValue::None, int)
}

#[test]
fn register_access_matches_reference() {
    let Some(mut py) = oracle() else { return };
    // SPI chip on the main MCU.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "chip = build_chip('lis2dw', [('cs_pin', 'PA4')])",
            "res('early read', guard(chip.read_reg, 0x0f))",
            "guard(chip.set_reg, 0x21, 0x06)",
            "run_cbs()",
            "res('devid', guard(chip.read_reg, 0x0f))",
            "MCU.regs[0x10] = 0x77",
            "res('reg 0x10', guard(chip.read_reg, 0x10))",
            "res('unset reg', guard(chip.read_reg, 0x11))",
            "guard(chip.set_reg, 0x21, 0x06)",
            "res('ctrl2', guard(chip.read_reg, 0x21))",
            "guard(chip.set_reg, 0x21, 0x1ff)",
            "guard(chip.set_reg, 0x25, 5, minclock=1234)",
            "MCU.stuck.add(0x2e)",
            "guard(chip.set_reg, 0x2e, 0xc0)",
            "guard(chip.set_reg, 0x2e, 0)",
            "res('regs', sorted(MCU.regs.items()))",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    let chip = rig.build("lis2dw", SPI_PA4).expect("chip");
    res("early read", &opt_int(chip.read_reg(0x0f)));
    guard(chip.set_reg(0x21, 0x06));
    rig.run_cbs();
    res("devid", &opt_int(chip.read_reg(0x0f)));
    rig.mcu.set_reg(0x10, 0x77);
    res("reg 0x10", &opt_int(chip.read_reg(0x10)));
    res("unset reg", &opt_int(chip.read_reg(0x11)));
    guard(chip.set_reg(0x21, 0x06));
    res("ctrl2", &opt_int(chip.read_reg(0x21)));
    guard(chip.set_reg(0x21, 0x1ff));
    guard(chip.set_reg_minclock(0x25, 5, 1234));
    rig.mcu.stick(0x2e);
    guard(chip.set_reg(0x2e, 0xc0));
    guard(chip.set_reg(0x2e, 0));
    res("regs", &regs_value(&rig.mcu));
    check("spi register access", &mut py);

    // I2C chip on the aux MCU, with bus errors.
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "chip = build_chip('lis2dw bed', [('i2c_mcu', 'aux')])",
            "res('early read', guard(chip.read_reg, 0x0f))",
            "guard(chip.set_reg, 0x21, 0x06)",
            "run_cbs()",
            "res('devid', guard(chip.read_reg, 0x0f))",
            "AUX.regs[0x10] = 0x77",
            "res('reg 0x10', guard(chip.read_reg, 0x10))",
            "res('unset reg', guard(chip.read_reg, 0x11))",
            "guard(chip.set_reg, 0x21, 0x06)",
            "res('ctrl2', guard(chip.read_reg, 0x21))",
            "guard(chip.set_reg, 0x21, 0x1ff)",
            "guard(chip.set_reg, 0x25, 5, minclock=1234)",
            "AUX.stuck.add(0x2e)",
            "guard(chip.set_reg, 0x2e, 0xc0)",
            "guard(chip.set_reg, 0x2e, 0)",
            "AUX.i2c_status = 'NACK'",
            "res('nack read', guard(chip.read_reg, 0x0f))",
            "guard(chip.set_reg, 0x21, 0x06)",
            "AUX.i2c_status = 'SUCCESS'",
            "res('regs', sorted(AUX.regs.items()))",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    let chip = rig.build("lis2dw bed", I2C_AUX).expect("chip");
    res("early read", &opt_int(chip.read_reg(0x0f)));
    guard(chip.set_reg(0x21, 0x06));
    rig.run_cbs();
    res("devid", &opt_int(chip.read_reg(0x0f)));
    rig.aux.set_reg(0x10, 0x77);
    res("reg 0x10", &opt_int(chip.read_reg(0x10)));
    res("unset reg", &opt_int(chip.read_reg(0x11)));
    guard(chip.set_reg(0x21, 0x06));
    res("ctrl2", &opt_int(chip.read_reg(0x21)));
    guard(chip.set_reg(0x21, 0x1ff));
    guard(chip.set_reg_minclock(0x25, 5, 1234));
    rig.aux.stick(0x2e);
    guard(chip.set_reg(0x2e, 0xc0));
    guard(chip.set_reg(0x2e, 0));
    rig.aux.set_i2c_status("NACK");
    res("nack read", &opt_int(chip.read_reg(0x0f)));
    guard(chip.set_reg(0x21, 0x06));
    rig.aux.set_i2c_status("SUCCESS");
    res("regs", &regs_value(&rig.aux));
    check("i2c register access", &mut py);
}

/// `sorted(M.regs.items())`.
fn regs_value(mcu: &FakeMcu) -> PyValue {
    let mut items: Vec<(i64, i64)> = mcu.chip.regs.borrow().iter().map(|(k, v)| (*k, *v)).collect();
    items.sort();
    PyValue::List(
        items.into_iter().map(|(k, v)| PyValue::Tuple(vec![int(k), int(v)])).collect(),
    )
}

fn convert_case(
    py: &mut pyoracle::Oracle,
    label: &str,
    section: &str,
    values: &[(&str, &str)],
) {
    // Raw samples: hand-picked extremes plus a deterministic spread.
    let raw = frames(7, 40);
    let mut rows: Vec<[i64; 3]> = raw
        .chunks(6)
        .map(|c| {
            [
                i16::from_le_bytes([c[0], c[1]]) as i64,
                i16::from_le_bytes([c[2], c[3]]) as i64,
                i16::from_le_bytes([c[4], c[5]]) as i64,
            ]
        })
        .collect();
    rows.push([0, 0, 0]);
    rows.push([32767, -32768, 1]);
    rows.push([-32768, 32767, -1]);
    rows.push([12345, -6789, 4096]);
    let ptimes: Vec<f64> =
        (0..rows.len()).map(|i| 12.3456789012 + i as f64 * 0.0031213).collect();
    let literal = format!(
        "[{}]",
        rows.iter()
            .zip(&ptimes)
            .map(|(r, t)| format!("({t:?}, {}, {}, {})", r[0], r[1], r[2]))
            .collect::<Vec<_>>()
            .join(", ")
    );
    py_run(py, "make_world()");
    py_run(
        py,
        &[
            format!("chip = build_chip({section:?}, {})", py_values(values)),
            format!("fr = {literal}"),
            "guard(chip._convert_samples, fr)".to_string(),
            "res('frames', fr)".to_string(),
            "fr2 = [(1.5, 1, 2, 3), (2.5, 1, 2)]".to_string(),
            "guard(chip._convert_samples, fr2)".to_string(),
            "res('frames2', fr2)".to_string(),
            "fr3 = [(1.5, 1, 2, 3), (2.5, 1, 2, 3, 4)]".to_string(),
            "guard(chip._convert_samples, fr3)".to_string(),
            "res('frames3', fr3)".to_string(),
            "fr4 = [(1.5, 1.25, -2.5, 3), [2.5, 100, 200, 300]]".to_string(),
            "guard(chip._convert_samples, fr4)".to_string(),
            "res('frames4', fr4)".to_string(),
            "fr5 = []".to_string(),
            "guard(chip._convert_samples, fr5)".to_string(),
            "res('frames5', fr5)".to_string(),
            "res('errors', chip.last_error_count)".to_string(),
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    let chip = rig.build(section, values).expect("chip");
    let frame = |t: f64, a: [i64; 3]| {
        PyValue::Tuple(vec![PyValue::Float(t), int(a[0]), int(a[1]), int(a[2])])
    };
    let mut fr: Vec<PyValue> = rows.iter().zip(&ptimes).map(|(r, t)| frame(*t, *r)).collect();
    guard(chip._convert_samples(&mut fr));
    res("frames", &PyValue::List(fr));
    let short = PyValue::Tuple(vec![PyValue::Float(2.5), int(1), int(2)]);
    let mut fr2 = vec![frame(1.5, [1, 2, 3]), short];
    guard(chip._convert_samples(&mut fr2));
    res("frames2", &PyValue::List(fr2));
    let long = PyValue::Tuple(vec![PyValue::Float(2.5), int(1), int(2), int(3), int(4)]);
    let mut fr3 = vec![frame(1.5, [1, 2, 3]), long];
    guard(chip._convert_samples(&mut fr3));
    res("frames3", &PyValue::List(fr3));
    let floats = PyValue::Tuple(vec![
        PyValue::Float(1.5),
        PyValue::Float(1.25),
        PyValue::Float(-2.5),
        int(3),
    ]);
    let as_list = PyValue::List(vec![PyValue::Float(2.5), int(100), int(200), int(300)]);
    let mut fr4 = vec![floats, as_list];
    guard(chip._convert_samples(&mut fr4));
    res("frames4", &PyValue::List(fr4));
    let mut fr5: Vec<PyValue> = Vec::new();
    guard(chip._convert_samples(&mut fr5));
    res("frames5", &PyValue::List(fr5));
    res("errors", &int(chip.last_error_count().expect("errors")));
    check(label, py);
}

#[test]
fn convert_samples_matches_reference() {
    let Some(mut py) = oracle() else { return };
    convert_case(&mut py, "default axes", "lis2dw", SPI_PA4);
    convert_case(
        &mut py,
        "remapped axes",
        "lis2dw bed",
        &[("cs_pin", "PA4"), ("axes_map", "-z,x,-y")],
    );
    convert_case(
        &mut py,
        "swapped axes on i2c",
        "lis2dw other",
        &[("axes_map", "y,-x,z")],
    );
    convert_case(&mut py, "lis3dh scale", "lis3dh", SPI_PA4);
    convert_case(
        &mut py,
        "lis3dh remapped axes",
        "lis3dh bed",
        &[("cs_pin", "PA4"), ("axes_map", "-z,-y,-x")],
    );
}

#[test]
fn debug_commands_match_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "build_chip('lis2dw', [('cs_pin', 'PA4')])",
            "build_chip('lis2dw bed', [('i2c_mcu', 'aux')])",
            "run_cbs()",
            "AUX.regs[0x0f] = 0x33",
            "ready()",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    rig.build("lis2dw", SPI_PA4).expect("chip");
    rig.build("lis2dw bed", I2C_AUX).expect("chip");
    rig.run_cbs();
    rig.aux.set_reg(0x0f, 0x33);
    rig.ready();
    let lines = [
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=0x0f",
        "ACCELEROMETER_DEBUG_READ REG=15",
        "ACCELEROMETER_DEBUG_READ CHIP=bed REG=0x0f",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=0o17",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=0x7f",
        "ACCELEROMETER_DEBUG_READ CHIP=bed REG=0x7f",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=128",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=-1",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=abc",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=1.5",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw",
        "ACCELEROMETER_DEBUG_READ CHIP=nope REG=0",
        "ACCELEROMETER_DEBUG_READ CHIP=lis REG=0",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x21 VAL=0x06",
        "ACCELEROMETER_DEBUG_READ CHIP=lis2dw REG=0x21",
        "ACCELEROMETER_DEBUG_WRITE CHIP=bed REG=37 VAL=0x34",
        "ACCELEROMETER_DEBUG_READ CHIP=bed REG=37",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x25 VAL=255",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x25 VAL=256",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x25 VAL=-1",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x25",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw VAL=1",
        "ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=200 VAL=1",
    ];
    for line in lines {
        py_run(&mut py, &format!("gcmd_run({line:?})"));
        rig.gcmd(line);
    }
    // A register that ignores writes makes `set_reg` fail its read back.
    py_run(
        &mut py,
        "MCU.stuck.add(0x2e)\nAUX.stuck.add(0x2e)\n\
         gcmd_run('ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x2e VAL=0xc0')\n\
         gcmd_run('ACCELEROMETER_DEBUG_WRITE CHIP=bed REG=0x2e VAL=0xc0')",
    );
    rig.mcu.stick(0x2e);
    rig.aux.stick(0x2e);
    rig.gcmd("ACCELEROMETER_DEBUG_WRITE CHIP=lis2dw REG=0x2e VAL=0xc0");
    rig.gcmd("ACCELEROMETER_DEBUG_WRITE CHIP=bed REG=0x2e VAL=0xc0");
    check("debug commands", &mut py);
}

/// Both sides: the dwell hook of the query command feeds one batch of frames
/// and runs the periodic batch callback, as the reactor timer would.
fn py_dwell_hook(chip: &str, mcu: &str, frames_a: &[u8], frames_b: &[u8]) -> String {
    [
        "def hook():".to_string(),
        format!("    {mcu}.status = dict(STATUS1)"),
        format!("    feed({mcu}, 1, {})", bytes_list(frames_a)),
        format!("    feed({mcu}, 2, {})", bytes_list(frames_b)),
        format!("    CH[{chip:?}].batch_bulk._proc_batch(1000.5)"),
        "TH.on_dwell = hook".to_string(),
    ]
    .join("\n")
}

fn rs_dwell_hook(rig: &Rig, chip: &str, mcu: Arc<FakeMcu>, frames_a: Vec<u8>, frames_b: Vec<u8>) {
    let chip = rig.chip(chip);
    rig.set_dwell_hook(move || {
        *mcu.chip.status.borrow_mut() = status1();
        mcu.feed(1, &frames_a);
        mcu.feed(2, &frames_b);
        guard(chip.batch_bulk().expect("batch_bulk")._proc_batch(1000.5));
    });
}

#[test]
fn query_command_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "build_chip('lis2dw', [('cs_pin', 'PA4')])",
            "build_chip('lis2dw bed', [('i2c_mcu', 'aux')])",
            "run_cbs()",
            "ready()",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    rig.build("lis2dw", SPI_PA4).expect("chip");
    rig.build("lis2dw bed", I2C_AUX).expect("chip");
    rig.run_cbs();
    rig.ready();

    for (section, chip_arg, mcu_py, seed) in
        [("lis2dw", "lis2dw", "MCU", 11u32), ("lis2dw bed", "bed", "AUX", 13u32)]
    {
        let rs_mcu = if mcu_py == "MCU" { rig.mcu.clone() } else { rig.aux.clone() };
        let fa = frames(seed, 10);
        let fb = frames(seed + 1, 4);
        // Window [0, 5] holds every sample.
        py_run(&mut py, "TH.times = [0.0, 5.0]");
        py_run(&mut py, &py_dwell_hook(section, mcu_py, &fa, &fb));
        py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_QUERY CHIP={chip_arg}')"));
        rig.set_times(&[0.0, 5.0]);
        rs_dwell_hook(&rig, section, rs_mcu.clone(), fa.clone(), fb.clone());
        rig.gcmd(&format!("ACCELEROMETER_QUERY CHIP={chip_arg}"));
        check(&format!("query with samples on {section}"), &mut py);

        // The client of the finished query is dropped by the next batch,
        // which stops the chip.
        py_run(
            &mut py,
            &[
                format!("{mcu_py}.status = dict(STATUS1)"),
                format!("feed({mcu_py}, 4, [1, 2, 3, 4, 5, 6])"),
                format!("res('next batch', guard(CH[{section:?}].batch_bulk._proc_batch, 1001.0))"),
                format!("res('started', CH[{section:?}].batch_bulk.is_started)"),
            ]
            .join("\n"),
        );
        *rs_mcu.chip.status.borrow_mut() = status1();
        rs_mcu.feed(4, &[1, 2, 3, 4, 5, 6]);
        let chip = rig.chip(section);
        let next = guard(chip.batch_bulk().expect("bb")._proc_batch(1001.0));
        res("next batch", &next.map_or(PyValue::None, PyValue::Float));
        res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
        check(&format!("query client dropped on {section}"), &mut py);
    }

    // The window starts after the last sample: nothing is found.
    py_run(
        &mut py,
        "TH.on_dwell = None\nTH.times = [900.0, 901.0]\ngcmd_run('ACCELEROMETER_QUERY CHIP=bed')",
    );
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
fn measure_command_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let name = tmp_name("m");
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "build_chip('lis2dw', [('cs_pin', 'PA4')])",
            "build_chip('lis2dw bed', [('i2c_mcu', 'aux')])",
            "run_cbs()",
            "ready()",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    rig.build("lis2dw", SPI_PA4).expect("chip");
    rig.build("lis2dw bed", I2C_AUX).expect("chip");
    rig.run_cbs();
    rig.ready();

    let fa = frames(21, 10);
    let fb = frames(22, 6);

    for (section, chip_arg, file_prefix) in
        [("lis2dw", "lis2dw", "lis2dw"), ("lis2dw bed", "bed", "lis2dw-bed")]
    {
        let mcu_py = if section == "lis2dw" { "MCU" } else { "AUX" };
        let rs_mcu = if section == "lis2dw" { rig.mcu.clone() } else { rig.aux.clone() };
        let tag = format!("{name}_{chip_arg}");
        let path = format!("/tmp/{file_prefix}-{tag}.csv");

        // Start, feed one batch, then stop with an invalid and a valid NAME.
        py_run(&mut py, "TH.times = [0.0, 5.0]");
        py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_MEASURE CHIP={chip_arg}')"));
        rig.set_times(&[0.0, 5.0]);
        rig.gcmd(&format!("ACCELEROMETER_MEASURE CHIP={chip_arg}"));
        py_run(
            &mut py,
            &[
                format!("{mcu_py}.status = dict(STATUS1)"),
                format!("feed({mcu_py}, 1, {})", bytes_list(&fa)),
                format!("feed({mcu_py}, 2, {})", bytes_list(&fb)),
                format!("res('batch', guard(CH[{section:?}].batch_bulk._proc_batch, 1000.5))"),
            ]
            .join("\n"),
        );
        *rs_mcu.chip.status.borrow_mut() = status1();
        rs_mcu.feed(1, &fa);
        rs_mcu.feed(2, &fb);
        let batch = guard(rig.chip(section).batch_bulk().expect("bb")._proc_batch(1000.5));
        res("batch", &batch.map_or(PyValue::None, PyValue::Float));

        py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_MEASURE CHIP={chip_arg} NAME=bad!name')"));
        rig.gcmd(&format!("ACCELEROMETER_MEASURE CHIP={chip_arg} NAME=bad!name"));
        py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_MEASURE CHIP={chip_arg} NAME=')"));
        rig.gcmd(&format!("ACCELEROMETER_MEASURE CHIP={chip_arg} NAME="));

        py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_MEASURE CHIP={chip_arg} NAME={tag}')"));
        py_run(&mut py, &format!("res('csv', read_file({path:?}))"));
        rig.gcmd(&format!("ACCELEROMETER_MEASURE CHIP={chip_arg} NAME={tag}"));
        res("csv", &PyValue::Str(wait_for_file(&path)));
        check(&format!("measure {section}"), &mut py);
    }

    // Names with dashes and underscores are accepted.
    let tag = format!("{name}-a_b");
    let path = format!("/tmp/lis2dw-{tag}.csv");
    py_run(&mut py, "TH.times = [0.0, 5.0]");
    py_run(&mut py, "gcmd_run('ACCELEROMETER_MEASURE')");
    py_run(&mut py, &format!("gcmd_run('ACCELEROMETER_MEASURE NAME={tag}')"));
    py_run(&mut py, &format!("res('csv', read_file({path:?}))"));
    rig.set_times(&[0.0, 5.0]);
    rig.gcmd("ACCELEROMETER_MEASURE");
    rig.gcmd(&format!("ACCELEROMETER_MEASURE NAME={tag}"));
    res("csv", &PyValue::Str(wait_for_file(&path)));
    check("measure default chip with dashed name", &mut py);
}

#[test]
fn measure_default_name_matches_reference() {
    let Some(mut py) = oracle() else { return };
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            "build_chip('lis2dw', [('cs_pin', 'PA4')])",
            "run_cbs()",
            "ready()",
            "TH.times = [0.0, 5.0]",
            "gcmd_run('ACCELEROMETER_MEASURE')",
            "gcmd_run('ACCELEROMETER_MEASURE')",
            "os.remove('/tmp/lis2dw-20240101_000000.csv')",
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    rig.build("lis2dw", SPI_PA4).expect("chip");
    rig.run_cbs();
    rig.ready();
    rig.set_times(&[0.0, 5.0]);
    rig.gcmd("ACCELEROMETER_MEASURE");
    rig.gcmd("ACCELEROMETER_MEASURE");
    let written = peek_trace()
        .iter()
        .find_map(|l| {
            l.strip_prefix("out|// Writing raw accelerometer data to ")
                .and_then(|r| r.strip_suffix(" file"))
                .map(str::to_string)
        })
        .expect("a file is announced");
    wait_for_file(&written);
    check("measure with the default name", &mut py);
}

#[test]
fn batch_pipeline_matches_reference() {
    let Some(mut py) = oracle() else { return };
    let values: &[(&str, &str)] = &[("cs_pin", "PA4"), ("axes_map", "z,-y,x")];
    py_run(&mut py, "make_world()");
    py_run(
        &mut py,
        &[
            format!("chip = build_chip('lis2dw bed', {})", py_values(values)),
            "run_cbs()".to_string(),
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    let chip = rig.build("lis2dw bed", values).expect("chip");
    rig.run_cbs();
    check("pipeline setup", &mut py);

    let fa = frames(31, 10);
    let fb = frames(32, 7);
    let fc = frames(33, 3);

    // The dump endpoint registers the web client and starts the chip.
    py_run(
        &mut py,
        &[
            "cb = WH.endpoints[('lis2dw/dump_lis2dw', 'bed')]",
            "conn = FakeConn()",
            "cb(FakeWebRequest(conn))",
            "res('started', chip.batch_bulk.is_started)",
        ]
        .join("\n"),
    );
    let (_, _, endpoint) = rig.wh.endpoints.borrow()[0].clone();
    let conn = Arc::new(FakeConn { closed: GCell::new(false) });
    guard(endpoint(Arc::new(FakeWebRequest { conn: conn.clone() })));
    res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    check("endpoint start", &mut py);

    // An empty batch only reschedules.
    py_run(&mut py, "res('idle', guard(chip.batch_bulk._proc_batch, 1000.5))");
    res(
        "idle",
        &guard(chip.batch_bulk().expect("bb")._proc_batch(1000.5)).map_or(PyValue::None, PyValue::Float),
    );
    check("idle batch", &mut py);

    // Two batches with frames reach the web client.
    py_run(
        &mut py,
        &[
            "MCU.status = dict(STATUS1)".to_string(),
            format!("feed(MCU, 1, {})", bytes_list(&fa)),
            format!("feed(MCU, 2, {})", bytes_list(&fb)),
            "res('batch', guard(chip.batch_bulk._proc_batch, 1000.6))".to_string(),
            "res('errors', chip.last_error_count)".to_string(),
        ]
        .join("\n"),
    );
    *rig.mcu.chip.status.borrow_mut() = status1();
    rig.mcu.feed(1, &fa);
    rig.mcu.feed(2, &fb);
    res(
        "batch",
        &guard(chip.batch_bulk().expect("bb")._proc_batch(1000.6)).map_or(PyValue::None, PyValue::Float),
    );
    res("errors", &int(chip.last_error_count().expect("errors")));
    check("batches", &mut py);

    // Closing the connection stops the helper on the next batch.
    py_run(
        &mut py,
        &[
            "conn.closed = True".to_string(),
            "MCU.status['next_sequence'] = 4".to_string(),
            format!("feed(MCU, 3, {})", bytes_list(&fc)),
            "res('stop batch', guard(chip.batch_bulk._proc_batch, 1000.7))".to_string(),
            "res('started', chip.batch_bulk.is_started)".to_string(),
        ]
        .join("\n"),
    );
    conn.closed.set(true);
    {
        let mut st = rig.mcu.chip.status.borrow_mut();
        for (k, v) in st.iter_mut() {
            if k == "next_sequence" {
                *v = 4;
            }
        }
    }
    rig.mcu.feed(3, &fc);
    res(
        "stop batch",
        &guard(chip.batch_bulk().expect("bb")._proc_batch(1000.7)).map_or(PyValue::None, PyValue::Float),
    );
    res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    check("stop", &mut py);

    // A new internal client restarts the chip.
    py_run(
        &mut py,
        &[
            "TH.times = [3.0, 4.0]",
            "q = guard(chip.start_internal_client)",
            "res('started', chip.batch_bulk.is_started)",
        ]
        .join("\n"),
    );
    rig.set_times(&[3.0, 4.0]);
    guard(chip.start_internal_client());
    res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    check("restart", &mut py);
}

/// A change to the fake chip before or during a start/stop run.
enum Fx {
    Reg(i64, i64),
    Stick(i64),
    I2c(&'static str),
}

fn fx_py(mcu: &str, fx: &[Fx]) -> String {
    fx.iter()
        .map(|f| match f {
            Fx::Reg(r, v) => format!("{mcu}.regs[{r}] = {v}"),
            Fx::Stick(r) => format!("{mcu}.stuck.add({r})"),
            Fx::I2c(s) => format!("{mcu}.i2c_status = {s:?}"),
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn fx_rs(mcu: &FakeMcu, fx: &[Fx]) {
    for f in fx {
        match f {
            Fx::Reg(r, v) => mcu.set_reg(*r, *v),
            Fx::Stick(r) => mcu.stick(*r),
            Fx::I2c(s) => mcu.set_i2c_status(s),
        }
    }
}

struct StartCase {
    label: &'static str,
    section: &'static str,
    values: &'static [(&'static str, &'static str)],
    aux: bool,
    pre: Vec<Fx>,
    post: Vec<Fx>,
}

fn start_case(py: &mut pyoracle::Oracle, case: &StartCase) {
    let mcu_py = if case.aux { "AUX" } else { "MCU" };
    py_run(py, "make_world()");
    py_run(
        py,
        &[
            format!("chip = build_chip({:?}, {})", case.section, py_values(case.values)),
            "run_cbs()".to_string(),
            fx_py(mcu_py, &case.pre),
            "q = guard(chip.start_internal_client)".to_string(),
            "res('started', chip.batch_bulk.is_started)".to_string(),
            fx_py(mcu_py, &case.post),
            "if q is not None:".to_string(),
            "    guard(q.finish_measurements)".to_string(),
            format!("    {mcu_py}.status = dict(STATUS1)"),
            format!("    feed({mcu_py}, 1, [1, 2, 3, 4, 5, 6])"),
            "    res('batch', guard(chip.batch_bulk._proc_batch, 1000.5))".to_string(),
            "    res('started', chip.batch_bulk.is_started)".to_string(),
            format!("res('regs', sorted({mcu_py}.regs.items()))"),
        ]
        .join("\n"),
    );
    let rig = Rig::new();
    let mcu = if case.aux { rig.aux.clone() } else { rig.mcu.clone() };
    let chip = rig.build(case.section, case.values).expect("chip");
    rig.run_cbs();
    fx_rs(&mcu, &case.pre);
    let q = guard(chip.start_internal_client());
    res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    fx_rs(&mcu, &case.post);
    if let Some(q) = q {
        guard(q.finish_measurements());
        *mcu.chip.status.borrow_mut() = status1();
        mcu.feed(1, &[1, 2, 3, 4, 5, 6]);
        res(
            "batch",
            &guard(chip.batch_bulk().expect("bb")._proc_batch(1000.5))
                .map_or(PyValue::None, PyValue::Float),
        );
        res("started", &PyValue::Bool(chip.batch_bulk().expect("bb").is_started()));
    }
    res("regs", &regs_value(&mcu));
    check(case.label, py);
}

#[test]
fn start_and_stop_match_reference() {
    let Some(mut py) = oracle() else { return };
    fn case(
        label: &'static str,
        section: &'static str,
        values: &'static [(&'static str, &'static str)],
        aux: bool,
        pre: Vec<Fx>,
        post: Vec<Fx>,
    ) -> StartCase {
        StartCase { label, section, values, aux, pre, post }
    }
    let cases = vec![
        case("lis2dw spi ok", "lis2dw", SPI_PA4, false, vec![], vec![]),
        case("lis2dw spi wrong id", "lis2dw", SPI_PA4, false, vec![Fx::Reg(0x0f, 0)], vec![]),
        case("lis2dw spi stuck ctrl2", "lis2dw", SPI_PA4, false, vec![Fx::Stick(0x21)], vec![]),
        case("lis2dw spi stuck ctrl6", "lis2dw", SPI_PA4, false, vec![Fx::Stick(0x25)], vec![]),
        case("lis2dw spi stuck fifo", "lis2dw", SPI_PA4, false, vec![Fx::Stick(0x2e)], vec![]),
        case("lis2dw spi stuck ctrl1", "lis2dw", SPI_PA4, false, vec![Fx::Stick(0x20)], vec![]),
        case("lis2dw spi stuck fifo on stop", "lis2dw", SPI_PA4, false, vec![], vec![Fx::Stick(0x2e)]),
        case("lis2dw i2c ok", "lis2dw bed", I2C_AUX, true, vec![], vec![]),
        case("lis2dw i2c wrong id", "lis2dw bed", I2C_AUX, true, vec![Fx::Reg(0x0f, 0x33)], vec![]),
        case("lis2dw i2c stuck ctrl6", "lis2dw bed", I2C_AUX, true, vec![Fx::Stick(0x25)], vec![]),
        case("lis2dw i2c stuck fifo", "lis2dw bed", I2C_AUX, true, vec![Fx::Stick(0x2e)], vec![]),
        case("lis2dw i2c stuck ctrl1", "lis2dw bed", I2C_AUX, true, vec![Fx::Stick(0x20)], vec![]),
        case("lis2dw i2c bus error", "lis2dw bed", I2C_AUX, true, vec![Fx::I2c("NACK")], vec![]),
        case("lis2dw i2c stuck fifo on stop", "lis2dw bed", I2C_AUX, true, vec![], vec![Fx::Stick(0x2e)]),
        case("lis2dw i2c bus error on stop", "lis2dw bed", I2C_AUX, true, vec![], vec![Fx::I2c("NACK")]),
        case("lis3dh spi default id", "lis3dh", SPI_PA4, false, vec![], vec![]),
        case("lis3dh spi ok", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx()], vec![]),
        case("lis3dh spi stuck ctrl1", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx(), Fx::Stick(0x20)], vec![]),
        case("lis3dh spi stuck ctrl2", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx(), Fx::Stick(0x21)], vec![]),
        case("lis3dh spi stuck ctrl4", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx(), Fx::Stick(0x23)], vec![]),
        case("lis3dh spi stuck ctrl5", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx(), Fx::Stick(0x24)], vec![]),
        case("lis3dh spi stuck fifo", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx(), Fx::Stick(0x2e)], vec![]),
        case("lis3dh spi stuck fifo on stop", "lis3dh", SPI_PA4, false, vec![lis3dh_id_fx()], vec![Fx::Stick(0x2e)]),
        case("lis3dh i2c ok", "lis3dh x", I2C_AUX, true, vec![lis3dh_id_fx()], vec![]),
        case("lis3dh i2c wrong id", "lis3dh x", I2C_AUX, true, vec![], vec![]),
        case("lis3dh i2c stuck ctrl5", "lis3dh x", I2C_AUX, true, vec![lis3dh_id_fx(), Fx::Stick(0x24)], vec![]),
    ];
    for c in &cases {
        start_case(&mut py, c);
    }
}

fn lis3dh_id_fx() -> Fx {
    Fx::Reg(0x0f, 0x33)
}

#[test]
fn chip_api_object_is_reachable_through_the_printer() {
    let Some(mut py) = oracle() else { return };
    py_run(
        &mut py,
        "make_world()\nchip = build_chip('lis2dw bed', [('cs_pin', 'PA4')])\nres('interface attrs', (chip.name, chip.name, chip.lis_type, chip.bus_type))",
    );
    // `resonance_tester` looks the chip up by name and calls
    // `start_internal_client`; the object must expose the chip interface.
    let rig = Rig::new();
    let chip = rig.build("lis2dw bed", SPI_PA4).expect("chip");
    let obj = rig
        .printer_api
        .lookup_object("lis2dw bed", LookupDefault::Raise)
        .expect("lookup")
        .expect("object");
    let api = klippy_api::require_interface::<dyn AccelChipApi>(&obj, "lis2dw bed")
        .expect("chip interface");
    res("interface attrs", &PyValue::Tuple(vec![
        pstr(&api.name()),
        pstr(&chip.name()),
        pstr(chip.lis_type()),
        pstr(chip.bus_type().expect("bus_type")),
    ]));
    check("chip lookup interface", &mut py);
    py_run(
        &mut py,
        "chip = build_chip('lis3dh x', [])\nres('lis3dh attrs', (chip.lis_type, chip.bus_type, chip.data_rate))",
    );
    let chip = rig.build("lis3dh x", &[]).expect("chip");
    res("lis3dh attrs", &PyValue::Tuple(vec![
        pstr(chip.lis_type()),
        pstr(chip.bus_type().expect("bus_type")),
        int(chip.data_rate().expect("data_rate")),
    ]));
    check("lis3dh constructor", &mut py);
}
