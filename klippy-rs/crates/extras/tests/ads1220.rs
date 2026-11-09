//! Parity tests for `klippy/extras/ads1220.py`.
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): Python objects are built with
//! minimal fakes for printer / pins / mcu / config, then driven with
//! the same calls as the Rust port driven through fakes of the
//! `klippy_api` contract traits. Compared exactly: config reads
//! with defaults, MCU config commands (format strings and
//! `is_init`/`on_restart`), command lookups, command sends (format
//! and argument values), returned values (floats by bits), raised
//! errors (class and message), the end-to-end batch dict through
//! the real `bus` and `bulk_sensor` ports on both sides, and the
//! `lookup_sensor_error` TypeError quirk.
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit, so expected values come from the oracle alone.

use std::sync::Arc;

use extras::ads1220::{hexify, module_factory, Ads1220, SENSOR_TYPE_NAME};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts, GetIntOpts,
    GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandQueryWrapperApi, CommandWrapper,
    CommandWrapperApi, ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, Mcu, McuApi,
    ParsePinOpts, Pin, PinParams, PinResolver, PinResolverApi, PinsApi, QuerySendOpts, SendOpts,
    SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::trunc_float_to_bigint;
use pycompat::{PyInt, PyValue};
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import struct as _struct
from extras import ads1220
from extras import bus
from extras import bulk_sensor

class _CommandError(Exception):
    pass
_CommandError.__name__ = "CommandError"

_sent = object()

def _norm(data):
    return [bytes(p) if isinstance(p, list) else p for p in data]

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append("send|%s|%r|%r|%r" % (self.fmt, _norm(list(data)), minclock, reqclock))
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append("wait_ack|%s|%r|%r|%r" % (self.fmt, _norm(list(data)), minclock, reqclock))

class FakeQuery:
    def __init__(self, mcu, fmt, resp):
        self.mcu = mcu
        self.fmt = fmt
        self.resp = resp
    def _resp(self):
        if self.fmt.startswith("spi_transfer"):
            return {"response": self.mcu.spi_responses.pop(0)}
        if self.fmt.startswith("query_ads1220_status"):
            return self.mcu.status_responses.pop(0)
        return {"response": b""}
    def send(self, data, minclock=0, reqclock=0, retry=True):
        self.mcu.sends.append("qsend|%s|%r|%r|%r|%r" % (self.fmt, _norm(list(data)), minclock, reqclock, retry))
        return self._resp()
    def send_with_preface(self, preface_cmd, preface_data, data, minclock=0, reqclock=0, retry=True):
        self.mcu.sends.append("preface|%s|<spi_send>|%r|%r|%r|%r|%r" % (self.fmt, _norm(list(preface_data)), _norm(list(data)), minclock, reqclock, retry))
        return self._resp()

class FakeMcu:
    def __init__(self, printer, name="mcu"):
        self._printer = printer
        self._name = name
        self._next_oid = 0
        self.config_cmds = []
        self.lookups = []
        self.sends = []
        self.config_cbs = []
        self.serial_cbs = []
        self.enumerations = {}
        self.constants = {}
        self.fileoutput = False
        self.spi_responses = []
        self.status_responses = []
        self.known = set()
    def get_name(self):
        return self._name
    def get_printer(self):
        return self._printer
    def get_enumerations(self):
        return self.enumerations
    def get_constants(self):
        return self.constants
    def is_fileoutput(self):
        return self.fileoutput
    def create_oid(self):
        self._next_oid += 1
        return self._next_oid
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append("%r|%r|%r" % (cmd, is_init, on_restart))
    def alloc_command_queue(self):
        return 7
    def register_config_callback(self, cb):
        self.config_cbs.append(cb)
    def try_lookup_command(self, fmt):
        self.lookups.append("try|%s" % fmt)
        if fmt in self.known:
            return FakeCmd(self, fmt)
        return None
    def lookup_command(self, fmt, cq=None):
        self.lookups.append("cmd|%s|%r" % (fmt, cq))
        return FakeCmd(self, fmt)
    def lookup_query_command(self, fmt, resp, oid=None, cq=None, is_async=False):
        self.lookups.append("query|%s|%s|%r|%r" % (fmt, resp, oid, cq))
        return FakeQuery(self, fmt, resp)
    def seconds_to_clock(self, t):
        return int(t * 1000000)
    def clock32_to_clock64(self, clock32):
        return clock32
    def clock_to_print_time(self, clock):
        return clock
    def register_serial_response(self, cb, fmt, oid=None):
        self.serial_cbs.append((cb, fmt, oid))
    def run_config_cbs(self):
        for cb in list(self.config_cbs):
            cb()
    def fire_serial(self, params):
        for cb, fmt, oid in list(self.serial_cbs):
            cb(params)

class FakeResolver:
    def __init__(self, pins):
        self.pins = pins
    def reserve_pin(self, pin, reserve_name):
        self.pins.reserved.append("%s|%s" % (pin, reserve_name))

class FakePins:
    def __init__(self, printer):
        self.printer = printer
        self.chips = {}
        self.active = {}
        self.reserved = []
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        desc = pin_desc.strip()
        if ':' not in desc:
            chip_name, pin = 'mcu', desc
        else:
            chip_name, pin = [s.strip() for s in desc.split(':', 1)]
        if chip_name not in self.chips:
            raise Exception("Unknown pin chip name '%s'" % (chip_name,))
        params = {'chip': self.chips[chip_name], 'chip_name': chip_name,
                  'pin': pin, 'invert': 0, 'pullup': 0, 'share_type': share_type}
        self.active["%s:%s" % (chip_name, pin)] = params
        return params
    def reset_pin_sharing(self, pin_params):
        share_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        self.active.pop(share_name, None)
    def get_pin_resolver(self, chip_name):
        if chip_name not in self.chips:
            raise Exception("Unknown chip name '%s'" % (chip_name,))
        return FakeResolver(self)

class FakeReactor:
    NEVER = float('inf')
    def monotonic(self):
        return 10.0
    def register_timer(self, cb, waketime):
        return "timer"
    def unregister_timer(self, t):
        pass

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.shutdown = False
        self.reactor = FakeReactor()
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            return None
        raise configparser.Error("missing %s" % name)
    def is_shutdown(self):
        return self.shutdown
    def command_error(self, msg):
        return _CommandError(msg)
    def get_reactor(self):
        return self.reactor
    def add_object(self, name, obj):
        self.objects[name] = obj

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, values):
        self._printer = printer
        self.section = section
        self.values = values
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def get(self, option, default=_sent, note_valid=True):
        if option not in self.values:
            if default is not _sent:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return self.values[option]
    def getint(self, option, default=_sent, minval=None, maxval=None, note_valid=True):
        if option not in self.values:
            if default is not _sent:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        v = int(self.values[option])
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v
    def getfloat(self, option, default=_sent, minval=None, maxval=None, note_valid=True):
        if option not in self.values:
            if default is not _sent:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return float(self.values[option])
    def getboolean(self, option, default=False, note_valid=True):
        v = self.values.get(option, default)
        if isinstance(v, str):
            return v.lower() in ('1', 'true', 'yes', 'on')
        return bool(v)
    def getchoice(self, option, choices, default=_sent, note_valid=True):
        if option in self.values:
            v = self.values[option]
            if v not in choices:
                raise configparser.Error("invalid choice")
            return choices[v]
        if default is _sent:
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        if default not in choices:
            raise configparser.Error("invalid choice")
        return choices[default]
    def error(self, msg):
        return configparser.Error(msg)

def make_world(section, values, fileoutput=False):
    p = FakePrinter()
    mcu = FakeMcu(p, "mcu")
    mcu.fileoutput = fileoutput
    pins = FakePins(p)
    pins.chips = {"mcu": mcu}
    p.objects = {"pins": pins, "mcu": mcu}
    cfg = FakeConfig(p, section, values)
    return p, mcu, pins, cfg
"#;

// ---------------------------------------------------------------------------
// Rust-side fakes.
// ---------------------------------------------------------------------------

type Log = Arc<GCell<Vec<String>>>;

fn new_log() -> Log {
    Arc::new(GCell::new(Vec::new()))
}

fn py_repr(v: &PyValue) -> String {
    match v {
        PyValue::None => "None".to_string(),
        PyValue::Bool(b) => {
            if *b { "True".to_string() } else { "False".to_string() }
        }
        PyValue::Int(n) => n.0.to_string(),
        PyValue::Float(f) => pycompat::text::float_repr(*f),
        PyValue::Str(s) => pycompat::text::str_repr(s),
        PyValue::Bytes(b) => pycompat::text::bytes_repr(b),
        PyValue::List(l) => format!("[{}]", l.iter().map(py_repr).collect::<Vec<_>>().join(", ")),
        PyValue::Tuple(t) => {
            if t.len() == 1 {
                format!("({},)", py_repr(&t[0]))
            } else {
                format!("({})", t.iter().map(py_repr).collect::<Vec<_>>().join(", "))
            }
        }
        PyValue::Dict(_) => format!("{v:?}"),
        PyValue::Host(h) => format!("<host {h}>"),
    }
}

fn list_repr(v: &[PyValue]) -> String {
    format!("[{}]", v.iter().map(py_repr).collect::<Vec<_>>().join(", "))
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.0.to_string().parse().unwrap()),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(to_value).collect()),
        PyValue::Dict(pairs) => Value::Dict(
            pairs
                .iter()
                .map(|(k, v)| (to_value(k), to_value(v)))
                .collect(),
        ),
        PyValue::Host(_) => panic!("host value cannot be compared"),
    }
}

struct FakeCmd {
    log: Log,
    fmt: String,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.log.borrow_mut().push(format!(
            "send|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock
        ));
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.log.borrow_mut().push(format!(
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
    log: Log,
    fmt: String,
    spi_responses: Arc<GCell<Vec<Vec<u8>>>>,
    status_responses: Arc<GCell<Vec<PyValue>>>,
}

impl FakeQuery {
    fn resp(&self) -> PyValue {
        let mut d = IndexMap::new();
        if self.fmt.starts_with("spi_transfer") {
            let response = self.spi_responses.borrow_mut().remove(0);
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(response));
        } else if self.fmt.starts_with("query_ads1220_status") {
            return self.status_responses.borrow_mut().remove(0);
        } else {
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(Vec::new()));
        }
        PyValue::Dict(d)
    }
}

impl CommandQueryWrapperApi for FakeQuery {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        fn pb(b: bool) -> &'static str {
            if b { "True" } else { "False" }
        }
        self.log.borrow_mut().push(format!(
            "qsend|{}|{}|{}|{}|{}",
            self.fmt,
            list_repr(&data),
            opts.minclock,
            opts.reqclock,
            pb(opts.retry)
        ));
        Ok(self.resp())
    }
    fn send_with_preface(
        &self,
        preface_cmd: CommandWrapper,
        preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        fn pb(b: bool) -> &'static str {
            if b { "True" } else { "False" }
        }
        let _ = preface_cmd.get_command_tag();
        self.log.borrow_mut().push(format!(
            "preface|{}|<spi_send>|{}|{}|{}|{}|{}",
            self.fmt,
            list_repr(&preface_data),
            list_repr(&data),
            opts.minclock,
            opts.reqclock,
            pb(opts.retry)
        ));
        Ok(self.resp())
    }
}

struct FakeMcu {
    name: String,
    printer: GCell<Option<Printer>>,
    next_oid: GCell<i64>,
    config_cmds: Log,
    lookups: Log,
    sends: Log,
    known: GCell<Vec<String>>,
    enumerations: GCell<IndexMap<String, IndexMap<String, i64>>>,
    constants: GCell<IndexMap<String, String>>,
    fileoutput: GCell<bool>,
    config_cbs: GCell<Vec<ConfigCallback>>,
    serial_cbs: Log,
    serial_cb: GCell<Option<SerialResponseCallback>>,
    status_responses: Arc<GCell<Vec<PyValue>>>,
    spi_responses: Arc<GCell<Vec<Vec<u8>>>>,
}

impl FakeMcu {
    fn new(name: &str) -> Arc<FakeMcu> {
        Arc::new(FakeMcu {
            name: name.to_string(),
            printer: GCell::new(None),
            next_oid: GCell::new(0),
            config_cmds: new_log(),
            lookups: new_log(),
            sends: new_log(),
            known: GCell::new(Vec::new()),
            enumerations: GCell::new(IndexMap::new()),
            constants: GCell::new(IndexMap::new()),
            fileoutput: GCell::new(false),
            config_cbs: GCell::new(Vec::new()),
            serial_cbs: new_log(),
            serial_cb: GCell::new(None),
            status_responses: Arc::new(GCell::new(Vec::new())),
            spi_responses: Arc::new(GCell::new(Vec::new())),
        })
    }
    fn run_config_cbs(&self) {
        let mut cbs = std::mem::take(&mut *self.config_cbs.borrow_mut());
        for cb in cbs.iter_mut() {
            cb().expect("config callback");
        }
        *self.config_cbs.borrow_mut() = cbs;
    }
    fn fire_serial(&self, oid: i64, sequence: i64, data: &[u8]) {
        if let Some(cb) = self.serial_cb.borrow().clone() {
            let mut d = IndexMap::new();
            d.insert(PyValue::Str("oid".to_string()), PyValue::Int(PyInt::from_i64(oid)));
            d.insert(
                PyValue::Str("sequence".to_string()),
                PyValue::Int(PyInt::from_i64(sequence)),
            );
            d.insert(PyValue::Str("data".to_string()), PyValue::Bytes(data.to_vec()));
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
        self.fileoutput.get()
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::Mcu("setup_pin not used".to_string()))
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut v = self.next_oid.borrow_mut();
        *v += 1;
        Ok(*v)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.config_cbs.borrow_mut().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        fn pb(b: bool) -> &'static str {
            if b { "True" } else { "False" }
        }
        self.config_cmds.borrow_mut().push(format!(
            "{}|{}|{}",
            pycompat::text::str_repr(cmd),
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
        let n = trunc_float_to_bigint(time * 1000000.0);
        n.0.to_string().parse::<i64>().unwrap_or(0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.1
    }
    fn max_nominal_duration(&self) -> f64 {
        3.0
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        self.lookups.borrow_mut().push(format!(
            "cmd|{}|{}",
            msgformat,
            cq.map(|c| format!("{}", c.0)).unwrap_or_else(|| "None".to_string())
        ));
        Ok(Arc::new(FakeCmd { log: self.sends.clone(), fmt: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.lookups.borrow_mut().push(format!(
            "query|{}|{}|{}|{}",
            msgformat,
            respformat,
            opts.oid.map(|o| o.to_string()).unwrap_or_else(|| "None".to_string()),
            opts.cq.map(|c| c.0.to_string()).unwrap_or_else(|| "None".to_string())
        ));
        Ok(Arc::new(FakeQuery {
            log: self.sends.clone(),
            fmt: msgformat.to_string(),
            spi_responses: self.spi_responses.clone(),
            status_responses: self.status_responses.clone(),
        }))
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.lookups.borrow_mut().push(format!("try|{msgformat}"));
        if self.known.borrow().iter().any(|k| k == msgformat) {
            Some(Arc::new(FakeCmd { log: self.sends.clone(), fmt: msgformat.to_string() }))
        } else {
            None
        }
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(7)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        self.serial_cbs.borrow_mut().push(format!(
            "{}|{}",
            msg,
            oid.map(|o| o.to_string()).unwrap_or_else(|| "None".to_string())
        ));
        *self.serial_cb.borrow_mut() = Some(cb);
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.enumerations.borrow().clone()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.constants.borrow().clone()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        Ok(0.0)
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 0.0))
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
        Err(KlippyError::Mcu("not used".to_string()))
    }
}

struct FakeAsync;
impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakeResolver {
    reserved: Log,
}

impl PinResolverApi for FakeResolver {
    fn reserve_pin(&self, pin: &str, reserve_name: &str) -> KResult<()> {
        self.reserved.borrow_mut().push(format!("{pin}|{reserve_name}"));
        Ok(())
    }
    fn alias_pin(&self, _alias: &str, _pin: &str) -> KResult<()> {
        Ok(())
    }
    fn update_command(&self, cmd: &str) -> KResult<String> {
        Ok(cmd.to_string())
    }
}

struct FakePins {
    chips: GCell<IndexMap<String, Mcu>>,
    active: GCell<IndexMap<String, PinParams>>,
    reserved: Log,
    resets: Log,
}

impl FakePins {
    fn new() -> Arc<FakePins> {
        Arc::new(FakePins {
            chips: GCell::new(IndexMap::new()),
            active: GCell::new(IndexMap::new()),
            reserved: new_log(),
            resets: new_log(),
        })
    }
}

impl PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: ParsePinOpts) -> KResult<PinParams> {
        let desc = pin_desc.trim();
        let (chip_name, pin) = match desc.split_once(':') {
            None => ("mcu", desc.trim()),
            Some((c, p)) => (c.trim(), p.trim()),
        };
        let chip = self.chips.borrow().get(chip_name).cloned();
        match chip {
            None => Err(KlippyError::Pins(format!("Unknown pin chip name '{chip_name}'"))),
            Some(chip) => Ok(PinParams {
                chip,
                chip_name: chip_name.to_string(),
                pin: pin.to_string(),
                invert: 0,
                pullup: 0,
                share_type: None,
            }),
        }
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let pp = self.parse_pin(
            pin_desc,
            ParsePinOpts { can_invert: opts.can_invert, can_pullup: opts.can_pullup },
        )?;
        let share_name = format!("{}:{}", pp.chip_name, pp.pin);
        if let Some(existing) = self.active.borrow().get(&share_name) {
            return Ok(existing.clone());
        }
        let mut pp = pp;
        pp.share_type = opts.share_type;
        self.active.borrow_mut().insert(share_name, pp.clone());
        Ok(pp)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::Pins("setup_pin not used".to_string()))
    }
    fn reset_pin_sharing(&self, pin_params: PinParams) -> klippy_api::KResult<()> {
        self.resets.borrow_mut().push(format!("{}:{}", pin_params.chip_name, pin_params.pin));
        self.active.borrow_mut().shift_remove(&format!("{}:{}", pin_params.chip_name, pin_params.pin));
        Ok(())
    }
    fn get_pin_resolver(&self, chip_name: &str) -> KResult<PinResolver> {
        if !self.chips.borrow().contains_key(chip_name) {
            return Err(KlippyError::Pins(format!("Unknown chip name '{chip_name}'")));
        }
        Ok(Arc::new(FakeResolver { reserved: self.reserved.clone() }))
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    shutdown: GCell<bool>,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter { objects: GCell::new(IndexMap::new()), shutdown: GCell::new(false) })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
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
        self.shutdown.get()
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let obj = self.objects.borrow().get(name).cloned();
        match obj {
            Some(o) => Ok(Some(o)),
            None => match _default {
                LookupDefault::Raise => Err(KlippyError::Config(format!("Unknown object '{}'", name))),
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
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, _event: &str, _callback: EventCallback) {}
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _result: &str) {}
}

struct FakeConfig {
    printer: Printer,
    section: String,
    values: GCell<IndexMap<String, String>>,
    calls: Log,
}

impl FakeConfig {
    fn new(printer: Printer, section: &str, values: &[(&str, &str)]) -> Config {
        let mut m = IndexMap::new();
        for (k, v) in values {
            m.insert(k.to_string(), v.to_string());
        }
        Arc::new(FakeConfig {
            printer,
            section: section.to_string(),
            values: GCell::new(m),
            calls: new_log(),
        })
    }
}

fn missing(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        self.calls.borrow_mut().push(format!("get|{option}"));
        match self.values.borrow().get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(missing(&self.section, option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        self.calls.borrow_mut().push(format!("getint|{option}"));
        match self.values.borrow().get(option) {
            Some(v) => {
                let parsed: i64 = match v.parse() {
                    Ok(n) => n,
                    Err(_) => {
                        return Err(KlippyError::Config(format!(
                            "Unable to parse option '{option}' in section '{}'",
                            self.section
                        )))
                    }
                };
                if let Some(minval) = opts.minval {
                    if parsed < minval {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have minimum of {minval}",
                            self.section
                        )));
                    }
                }
                if let Some(maxval) = opts.maxval {
                    if parsed > maxval {
                        return Err(KlippyError::Config(format!(
                            "Option '{option}' in section '{}' must have maximum of {maxval}",
                            self.section
                        )));
                    }
                }
                Ok(Some(parsed))
            }
            None => match opts.default {
                None => Err(missing(&self.section, option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        Ok(None)
    }
    fn getboolean(&self, option: &str, opts: GetBoolOpts) -> KResult<Option<bool>> {
        self.calls.borrow_mut().push(format!("getboolean|{option}"));
        match self.values.borrow().get(option) {
            Some(v) => Ok(Some(matches!(v.to_lowercase().as_str(), "1" | "true" | "yes" | "on"))),
            None => match opts.default {
                None => Err(missing(&self.section, option)),
                Some(d) => Ok(d),
            },
        }
    }
    fn getchoice(&self, option: &str, choices: &PyValue, opts: GetChoiceOpts) -> KResult<PyValue> {
        self.calls.borrow_mut().push(format!("getchoice|{option}"));
        let dict = match choices {
            PyValue::Dict(d) => d,
            _ => return Err(KlippyError::Config("invalid choice".to_string())),
        };
        match self.values.borrow().get(option) {
            Some(v) => dict
                .get(&PyValue::Str(v.clone()))
                .cloned()
                .ok_or_else(|| KlippyError::Config("invalid choice".to_string())),
            None => match opts.default {
                None => Err(missing(&self.section, option)),
                Some(None) => Err(missing(&self.section, option)),
                Some(Some(PyValue::Str(key))) => dict
                    .get(&PyValue::Str(key.clone()))
                    .cloned()
                    .ok_or_else(|| KlippyError::Config("invalid choice".to_string())),
                Some(Some(other)) => Ok(other),
            },
        }
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        Ok(None)
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used by ads1220")
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

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("ads1220 fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping ads1220 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.ads1220".to_string(),
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

fn str_list(v: &[String]) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn py_err(py: &mut pyoracle::Oracle, code: &str) -> pyoracle::OracleResult {
    py.exec(code).expect("oracle err probe");
    match py.eval("err") {
        Ok(Value::Tuple(items)) => match items.as_slice() {
            [Value::Str(class_name), Value::Str(message)] => Err(recording::Exception {
                class_name: class_name.clone(),
                message: message.clone(),
            }),
            _ => panic!("oracle err probe: captured value is not an error tuple: {items:?}"),
        },
        Ok(other) => panic!("oracle err probe: no error captured (err = {other:?})"),
        Err(e) => Err(e),
    }
}

struct World {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    pins: Arc<FakePins>,
}

fn world(fileoutput: bool) -> World {
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let mcu = FakeMcu::new("mcu");
    *mcu.printer.borrow_mut() = Some(printer_api.clone());
    *mcu.fileoutput.borrow_mut() = fileoutput;
    let pins = FakePins::new();
    pins.chips.borrow_mut().insert("mcu".to_string(), mcu.clone());
    printer.add("pins", pins.clone());
    printer.add("mcu", mcu.clone());
    World { printer, printer_api, mcu, pins }
}

fn py_world(py: &mut pyoracle::Oracle, section: &str, values: &[(&str, &str)], fileoutput: bool) {
    let py_bool = if fileoutput { "True" } else { "False" };
    let values_expr = values
        .iter()
        .map(|(k, v)| format!("{:?}: {:?}", k, v))
        .collect::<Vec<_>>()
        .join(", ");
    py.exec(&format!(
        "p, mcu, pins, cfg = make_world({section:?}, {{{values}}}, fileoutput={fileoutput})",
        section = section,
        values = values_expr,
        fileoutput = py_bool,
    ))
    .expect("py world");
}

fn bulk_status(clock: i64, query_ticks: i64, next_sequence: i64, buffered: i64, possible_overflows: i64) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("clock".to_string()), PyValue::Int(PyInt::from_i64(clock)));
    d.insert(
        PyValue::Str("query_ticks".to_string()),
        PyValue::Int(PyInt::from_i64(query_ticks)),
    );
    d.insert(
        PyValue::Str("next_sequence".to_string()),
        PyValue::Int(PyInt::from_i64(next_sequence)),
    );
    d.insert(PyValue::Str("buffered".to_string()), PyValue::Int(PyInt::from_i64(buffered)));
    d.insert(
        PyValue::Str("possible_overflows".to_string()),
        PyValue::Int(PyInt::from_i64(possible_overflows)),
    );
    PyValue::Dict(d)
}

/// (`sensor.gain`, `sensor.sps`, ..., `sensor.name`) like the oracle eval.
fn fields_value(sensor: &Arc<Ads1220>) -> Value {
    Value::Tuple(vec![
        Value::int(sensor.gain()),
        Value::int(sensor.sps()),
        Value::Bool(sensor.is_turbo()),
        Value::int(sensor.mux()),
        Value::Bool(sensor.pga_bypass()),
        Value::int(sensor.vref()),
        Value::str(sensor.data_ready_pin()),
        Value::str(sensor.name()),
    ])
}

const BASE_VALUES: &[(&str, &str)] = &[("cs_pin", "PA0"), ("data_ready_pin", "PA1")];

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_no_entry_points() {
    let f = module_factory();
    assert_eq!(f.entry_points(), pyoracle::entry_points("extras.ads1220"), "entry points of extras.ads1220");
    let mut py = pyoracle::Oracle::start().expect("oracle");
    py.exec("import extras.ads1220").expect("import extras.ads1220");
    assert_eq!(
        py.eval("list(extras.ads1220.ADS1220_SENSOR_TYPE)"),
        Ok(recording::value::Value::List(vec![recording::value::Value::str(SENSOR_TYPE_NAME)])),
        "ADS1220_SENSOR_TYPE keys"
    );
}

#[test]
fn hexify_matches_python() {
    let Some(mut py) = oracle() else { return };
    for bytes in [vec![0xffu8, 0x1], vec![0u8, 0u8, 0u8, 0u8], vec![0xab, 0xcd], vec![]] {
        let py_bytes = bytes.iter().map(|b| format!("{}", b)).collect::<Vec<_>>().join(",");
        let expected = py
            .eval(&format!("ads1220.hexify(bytearray([{}]))", py_bytes))
            .expect("oracle hexify");
        let actual = Value::str(&hexify(&bytes));
        check(&format!("hexify {bytes:?}"), Ok(expected), actual);
    }
}

#[test]
fn init_defaults_and_config_matches_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");

    let expected = py
        .eval("(sensor.gain, sensor.sps, sensor.is_turbo, sensor.mux, sensor.pga_bypass, sensor.vref, sensor.data_ready_pin, sensor.name)")
        .expect("oracle fields");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds");
    let expected_lookups = py.eval("mcu.lookups").expect("py lookups");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    check("init defaults fields", Ok(expected), fields_value(&sensor));
    check(
        "init defaults config cmds",
        Ok(expected_cmds),
        str_list(&w.mcu.config_cmds.borrow().clone()),
    );
    check(
        "init defaults lookups",
        Ok(expected_lookups),
        str_list(&w.mcu.lookups.borrow().clone()),
    );
}

#[test]
fn init_chip_options_match_python() {
    let Some(mut py) = oracle() else { return };
    // Non-turbo rate, low gain, PGA-forcing mux, analog-supply vref.
    let values: &[(&str, &str)] = &[
        ("cs_pin", "PA0"),
        ("data_ready_pin", "PA1"),
        ("gain", "2"),
        ("sample_rate", "20"),
        ("input_mux", "AIN0_AVSS"),
        ("pga_bypass", "false"),
        ("vref", "analog_supply"),
    ];
    py_world(&mut py, "load_cell kc", values, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");
    let expected = py
        .eval("(sensor.gain, sensor.sps, sensor.is_turbo, sensor.mux, sensor.pga_bypass, sensor.vref)")
        .expect("oracle fields");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", values);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    check("chip options fields", Ok(expected), Value::Tuple(vec![
        Value::int(sensor.gain()),
        Value::int(sensor.sps()),
        Value::Bool(sensor.is_turbo()),
        Value::int(sensor.mux()),
        Value::Bool(sensor.pga_bypass()),
        Value::int(sensor.vref()),
    ]));
    check(
        "chip options config cmds (256000 non-turbo)",
        Ok(expected_cmds),
        str_list(&w.mcu.config_cmds.borrow().clone()),
    );

    // Turbo rate at the shared "90" key: turbo wins, speed is 512000.
    let values2: &[(&str, &str)] = &[
        ("cs_pin", "PA0"),
        ("data_ready_pin", "PA1"),
        ("sample_rate", "90"),
        ("pga_bypass", "true"),
    ];
    py_world(&mut py, "load_cell kc", values2, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 2");
    py.exec("mcu.run_config_cbs()").expect("py cbs 2");
    let expected = py
        .eval("(sensor.gain, sensor.sps, sensor.is_turbo, sensor.pga_bypass)")
        .expect("oracle fields 2");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds 2");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", values2);
    let sensor = Ads1220::new(&config).expect("rust init 2");
    w.mcu.run_config_cbs();
    check("turbo 90 fields", Ok(expected), Value::Tuple(vec![
        Value::int(sensor.gain()),
        Value::int(sensor.sps()),
        Value::Bool(sensor.is_turbo()),
        Value::Bool(sensor.pga_bypass()),
    ]));
    check(
        "turbo 90 config cmds (512000)",
        Ok(expected_cmds),
        str_list(&w.mcu.config_cmds.borrow().clone()),
    );
}

#[test]
fn init_errors_match_python() {
    let Some(mut py) = oracle() else { return };

    // REF1 conflicts with the default AIN0_AIN1 mux (0b0000).
    let values: &[(&str, &str)] = &[
        ("cs_pin", "PA0"),
        ("data_ready_pin", "PA1"),
        ("vref", "REF1"),
    ];
    py_world(&mut py, "load_cell kc", values, false);
    let expected = py_err(
        &mut py,
        "err = None\ntry:\n    sensor = ads1220.ADS1220(cfg)\nexcept Exception as e:\n    err = (type(e).__name__, str(e))",
    );
    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", values);
    let actual = Ads1220::new(&config);
    check_err(
        "vref mux conflict",
        expected,
        actual.err().as_ref().expect("must fail"),
    );

    // data_ready_pin on another MCU (cs_pin stays on mcu).
    let values: &[(&str, &str)] = &[
        ("cs_pin", "PA0"),
        ("data_ready_pin", "aux:PA1"),
    ];
    py_world(&mut py, "load_cell kc", values, false);
    py.exec("aux = FakeMcu(p, 'aux')\npins.chips['aux'] = aux").expect("aux chip");
    let expected = py_err(
        &mut py,
        "err = None\ntry:\n    sensor = ads1220.ADS1220(cfg)\nexcept Exception as e:\n    err = (type(e).__name__, str(e))",
    );
    let w = world(false);
    let aux = FakeMcu::new("aux");
    *aux.printer.borrow_mut() = Some(w.printer_api.clone());
    w.pins.chips.borrow_mut().insert("aux".to_string(), aux);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", values);
    let actual = Ads1220::new(&config);
    check_err(
        "data_ready_pin other mcu",
        expected,
        actual.err().as_ref().expect("must fail"),
    );

    // Invalid gain choice.
    let values: &[(&str, &str)] = &[
        ("cs_pin", "PA0"),
        ("data_ready_pin", "PA1"),
        ("gain", "3"),
    ];
    py_world(&mut py, "load_cell kc", values, false);
    let expected = py_err(
        &mut py,
        "err = None\ntry:\n    sensor = ads1220.ADS1220(cfg)\nexcept Exception as e:\n    err = (type(e).__name__, str(e))",
    );
    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", values);
    let actual = Ads1220::new(&config);
    check_err("invalid gain choice", expected, actual.err().as_ref().expect("must fail"));
}

#[test]
fn register_flow_matches_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");
    py.exec(
        "mcu.spi_responses = [bytes([0, 0, 0, 0, 0]), bytes([0, 14, 148, 0, 0])]",
    )
    .expect("py responses");
    py.exec("sensor.reset_chip()\nsensor.setup_chip()").expect("py reset+setup");
    let expected_sends = py.eval("mcu.sends").expect("py sends");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().extend([
        vec![0, 0, 0, 0, 0],
        vec![0, 14, 148, 0, 0],
    ]);
    sensor.reset_chip().expect("rust reset");
    sensor.setup_chip().expect("rust setup");
    check(
        "register flow sends",
        Ok(expected_sends),
        str_list(&w.mcu.sends.borrow().clone()),
    );

    // Bad reset state raises command_error on both sides.
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 2");
    py.exec("mcu.run_config_cbs()").expect("py cbs 2");
    py.exec("mcu.spi_responses = [bytes([0, 0xff, 0x1, 0x2, 0x3])]").expect("py responses 2");
    let expected = py_err(
        &mut py,
        "err = None\ntry:\n    sensor.reset_chip()\nexcept Exception as e:\n    err = (type(e).__name__, str(e))",
    );
    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init 2");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().push(vec![0, 0xff, 0x1, 0x2, 0x3]);
    let actual = sensor.reset_chip();
    check_err("bad reset state", expected, actual.err().as_ref().expect("must fail"));

    // Bad register read-back raises command_error; fileoutput swallows it.
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 3");
    py.exec("mcu.run_config_cbs()").expect("py cbs 3");
    py.exec("mcu.spi_responses = [bytes([0, 0, 0, 0, 0])]").expect("py responses 3");
    let expected = py_err(
        &mut py,
        "err = None\ntry:\n    sensor.write_reg(0x0, [0x43, 14, 148, 0, 0])\nexcept Exception as e:\n    err = (type(e).__name__, str(e))",
    );
    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init 3");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().push(vec![0, 0, 0, 0, 0]);
    let actual = sensor.write_reg(0x0, &[0x43, 14, 148, 0, 0]);
    check_err("bad register read-back", expected, actual.err().as_ref().expect("must fail"));

    // fileoutput=True: reset_chip ignores the bad state and add_config_cmd
    // ordering is unchanged; write_reg also returns without raising.
    py_world(&mut py, "load_cell kc", BASE_VALUES, true);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 4");
    py.exec("mcu.run_config_cbs()").expect("py cbs 4");
    py.exec("mcu.spi_responses = [bytes([0, 1, 2, 3, 4]), bytes([0, 0, 0, 0, 0])]").expect("py responses 4");
    py.exec("sensor.reset_chip()\nsensor.write_reg(0x0, [0x43, 14, 148, 0, 0])").expect("py fileoutput ok");
    let expected_sends = py.eval("mcu.sends").expect("py sends 4");

    let w = world(true);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init 4");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().extend([vec![0, 1, 2, 3, 4], vec![0, 0, 0, 0, 0]]);
    sensor.reset_chip().expect("rust fileoutput reset ok");
    sensor.write_reg(0x0, &[0x43, 14, 148, 0, 0]).expect("rust fileoutput write ok");
    check(
        "fileoutput sends",
        Ok(expected_sends),
        str_list(&w.mcu.sends.borrow().clone()),
    );
}

#[test]
fn start_finish_measurements_match_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");
    py.exec("mcu.spi_responses = [bytes([0, 0, 0, 0, 0]), bytes([0, 14, 148, 0, 0])]")
        .expect("py spi");
    py.exec("mcu.status_responses = [{'clock': 1000, 'query_ticks': 1, 'next_sequence': 0, 'buffered': 0, 'possible_overflows': 0}]")
        .expect("py status");
    py.exec("sensor._start_measurements()").expect("py start");
    let expected_sends = py.eval("mcu.sends").expect("py sends");
    py.exec("sensor._finish_measurements()").expect("py finish");
    let expected_sends2 = py.eval("mcu.sends").expect("py sends 2");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().extend([vec![0, 0, 0, 0, 0], vec![0, 14, 148, 0, 0]]);
    w.mcu.status_responses.borrow_mut().push(bulk_status(1000, 1, 0, 0, 0));
    sensor.start_measurements().expect("rust start");
    check(
        "start sends",
        Ok(expected_sends),
        str_list(&w.mcu.sends.borrow().clone()),
    );
    sensor.finish_measurements().expect("rust finish");
    check(
        "finish sends",
        Ok(expected_sends2),
        str_list(&w.mcu.sends.borrow().clone()),
    );

    // Shutdown: finish_measurements returns without sending.
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 2");
    py.exec("mcu.run_config_cbs()").expect("py cbs 2");
    py.exec("p.shutdown = True").expect("py shutdown");
    py.exec("sensor._finish_measurements()").expect("py finish shutdown");
    let expected_sends = py.eval("mcu.sends").expect("py sends 3");

    let w = world(false);
    *w.printer.shutdown.borrow_mut() = true;
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init 2");
    w.mcu.run_config_cbs();
    sensor.finish_measurements().expect("rust finish shutdown");
    check(
        "finish shutdown sends",
        Ok(expected_sends),
        str_list(&w.mcu.sends.borrow().clone()),
    );
}

#[test]
fn process_batch_end_to_end_matches_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");
    py.exec("mcu.spi_responses = [bytes([0, 0, 0, 0, 0]), bytes([0, 14, 148, 0, 0])]")
        .expect("py spi");
    py.exec("mcu.status_responses = [{'clock': 1000, 'query_ticks': 1, 'next_sequence': 0, 'buffered': 0, 'possible_overflows': 0}, {'clock': 2000, 'query_ticks': 1, 'next_sequence': 1, 'buffered': 0, 'possible_overflows': 0}]").expect("py status");
    py.exec("sensor._start_measurements()").expect("py start");
    py.exec(
        "mcu.fire_serial({'oid': 2, 'sequence': 0, 'data': _struct.pack('<ii', 8388607, -8388608)})",
    )
    .expect("py fire");
    let expected = py.eval("sensor._process_batch(0.0)").expect("py batch");
    let expected_status = py.eval("sensor.get_status(0)").expect("py status");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().extend([vec![0, 0, 0, 0, 0], vec![0, 14, 148, 0, 0]]);
    w.mcu.status_responses.borrow_mut().push(bulk_status(1000, 1, 0, 0, 0));
    w.mcu.status_responses.borrow_mut().push(bulk_status(2000, 1, 1, 0, 0));
    sensor.start_measurements().expect("rust start");
    let mut data = Vec::new();
    data.extend_from_slice(&8388607i32.to_le_bytes());
    data.extend_from_slice(&(-8388608i32).to_le_bytes());
    w.mcu.fire_serial(2, 0, &data);
    let batch = sensor.process_batch(0.0).expect("rust batch");
    check("batch dict", Ok(expected), to_value(&batch));
    check(
        "get_status after batch",
        Ok(expected_status),
        to_value(&sensor.get_status(0.0)),
    );
}

#[test]
fn conversion_quirk_and_trigger_match_python() {
    let Some(mut py) = oracle() else { return };
    // `_convert_samples` on well-formed input.
    py.exec(
        "s = [(0.123456789, 8388607), (1.0, -8388608), (2.5, 4194304)]\nads1220.ADS1220._convert_samples(None, s)",
    )
    .expect("py convert");
    let expected = py.eval("s").expect("oracle converted");

    let mut samples = vec![
        PyValue::Tuple(vec![PyValue::Float(0.123456789), PyValue::Int(PyInt::from_i64(8388607))]),
        PyValue::Tuple(vec![PyValue::Float(1.0), PyValue::Int(PyInt::from_i64(-8388608))]),
        PyValue::Tuple(vec![PyValue::Float(2.5), PyValue::Int(PyInt::from_i64(4194304))]),
    ];
    Ads1220::convert_samples(&mut samples).expect("rust convert");
    let actual = Value::List(samples.iter().map(to_value).collect());
    check("converted samples", Ok(expected), actual);

    // lookup_sensor_error raises TypeError ("not all arguments converted...").
    let expected = py.eval("ads1220.ADS1220.lookup_sensor_error(None, 1)");
    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    check_err(
        "lookup_sensor_error quirk",
        expected,
        sensor.lookup_sensor_error(1).err().as_ref().expect("must fail"),
    );

    // get_range
    assert_eq!(sensor.get_range(), (-0x800000, 0x7FFFFF));
    let expected = py.eval("ads1220.ADS1220.get_range(None)").expect("oracle range");
    check(
        "get_range",
        Ok(expected),
        Value::Tuple(vec![
            Value::int(sensor.get_range().0),
            Value::int(sensor.get_range().1),
        ]),
    );

    // Trigger-analog config command, created with is_init=True.
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init 2");
    py.exec("sensor.setup_trigger_analog(9)").expect("py trigger");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init 2");
    sensor.setup_trigger_analog(9).expect("rust trigger");
    check(
        "trigger analog cmds",
        Ok(expected_cmds),
        str_list(&w.mcu.config_cmds.borrow().clone()),
    );

    // get_status / get_samples_per_second / get_mcu name.
    let expected = py.eval("sensor.get_status(0)").expect("py status");
    check("status fresh", Ok(expected), to_value(&sensor.get_status(0.0)));
    assert_eq!(sensor.get_samples_per_second(), 660);
    assert_eq!(sensor.get_mcu().get_name(), "mcu");
}

// `_proc_batch` batch timer driving is exercised implicitly through the
// real BatchBulkHelper; the observable contract here is that add_client
// starts the helper (is_started + start_cb sends) and _stop halts it.
#[test]
fn add_client_starts_batching_match_python() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, "load_cell kc", BASE_VALUES, false);
    py.exec("sensor = ads1220.ADS1220(cfg)").expect("py init");
    py.exec("mcu.run_config_cbs()").expect("py cbs");
    py.exec("mcu.spi_responses = [bytes([0, 0, 0, 0, 0]), bytes([0, 14, 148, 0, 0])]")
        .expect("py spi");
    py.exec("mcu.status_responses = [{'clock': 1000, 'query_ticks': 1, 'next_sequence': 0, 'buffered': 0, 'possible_overflows': 0}]")
        .expect("py status");
    py.exec("sensor.add_client(lambda msg: True)").expect("py add_client");
    let expected_sends = py.eval("mcu.sends").expect("py sends");
    let expected_started = py.eval("sensor.batch_bulk.is_started").expect("py started");
    py.exec("sensor.batch_bulk._stop()").expect("py stop");
    let expected_sends2 = py.eval("mcu.sends").expect("py sends 2");
    let expected_started2 = py.eval("sensor.batch_bulk.is_started").expect("py started 2");

    let w = world(false);
    let config = FakeConfig::new(w.printer_api.clone(), "load_cell kc", BASE_VALUES);
    let sensor = Ads1220::new(&config).expect("rust init");
    w.mcu.run_config_cbs();
    w.mcu.spi_responses.borrow_mut().extend([vec![0, 0, 0, 0, 0], vec![0, 14, 148, 0, 0]]);
    w.mcu.status_responses.borrow_mut().push(bulk_status(1000, 1, 0, 0, 0));
    sensor
        .add_client(Box::new(|_msg: PyValue| Ok(true)))
        .expect("rust add_client");
    let rust_started = sensor.batch_helper().is_started();
    check("client started", Ok(expected_started), Value::Bool(rust_started));
    check(
        "add_client sends",
        Ok(expected_sends),
        str_list(&w.mcu.sends.borrow().clone()),
    );
    sensor.batch_helper()._stop().expect("rust stop");
    check(
        "client stopped",
        Ok(expected_started2),
        Value::Bool(sensor.batch_helper().is_started()),
    );
    check(
        "stop sends",
        Ok(expected_sends2),
        str_list(&w.mcu.sends.borrow().clone()),
    );
}
