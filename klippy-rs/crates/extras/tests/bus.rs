//! Parity tests for `extras::bus` (port of `klippy/extras/bus.py`).
//!
//! Every expected value comes from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`): Python objects are built with
//! minimal fakes for printer / pins / mcu / configfile / config, then
//! driven with the same calls as the Rust port driven through fakes of
//! the contract traits. Compared exactly: MCU config commands (format
//! strings and `is_init`), command lookups, command sends (format and
//! argument values), getters, return values, raised errors (class and
//! message), config options read with defaults and minima, pin-sharing
//! resets, reserved pins, deprecated-MCU-code notices, shutdown
//! invocations and queued I2C writes.
//!
//! There are no recordings under `tests/fixtures/recordings/` for this
//! unit, so expected values come from the oracle alone.

use extras::bus::{
    mcu_i2c_from_config, mcu_spi_from_config, module_factory, resolve_bus_name,
    McuBusDigitalOut, McuBusDigitalOutApi, McuI2c, McuI2cApi, McuSpi, McuSpiApi,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, DeprecateGcodeOpts, GetBoolOpts, GetChoiceOpts, GetFloatOpts,
    GetFloatListOpts, GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
    PrinterConfigApi,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandQueryWrapperApi,
    CommandWrapper, CommandWrapperApi, ConfigCallback, LookupPinOpts,
    LookupQueryCommandOpts, Mcu, McuApi, ParsePinOpts, Pin, PinParams, PinResolver,
    PinResolverApi, PinsApi, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, GCell, ObjectRef};
use pycompat::text::trunc_float_to_bigint;
use pycompat::PyValue;
use reactor::Reactor;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python-side fakes, built once per oracle process.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import configparser
import sys
import types
_mcu_mod = types.ModuleType("mcu")
class _McuModError(Exception):
    pass
_McuModError.__name__ = "error"
_mcu_mod.error = _McuModError
def _get_printer_mcu(printer, name):
    if name == 'mcu':
        return printer.lookup_object(name)
    return printer.lookup_object('mcu ' + name)
_mcu_mod.get_printer_mcu = _get_printer_mcu
sys.modules['mcu'] = _mcu_mod
import extras.bus as bus

class _PinsError(Exception):
    pass
_PinsError.__name__ = "error"

class _McuError(Exception):
    pass
_McuError.__name__ = "error"

_sentinel = object()

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
            return {"response": b"\xde\xad\xbe\xef"}
        if self.fmt.startswith("i2c_read"):
            return {"response": b"\x10\x20"}
        if self.fmt.startswith("i2c_transfer"):
            return {"i2c_bus_status": self.mcu.i2c_status, "response": b"\x99"}
        return {"response": b""}
    def send(self, data, minclock=0, reqclock=0, retry=True):
        self.mcu.sends.append("qsend|%s|%r|%r|%r|%r" % (self.fmt, _norm(list(data)), minclock, reqclock, retry))
        return self._resp()
    def send_with_preface(self, preface_cmd, preface_data, data, minclock=0, reqclock=0, retry=True):
        self.mcu.sends.append("preface|%s|<spi_send>|%r|%r|%r|%r|%r" % (self.fmt, _norm(list(preface_data)), _norm(list(data)), minclock, reqclock, retry))
        return self._resp()

class FakeMcu:
    error = _McuError
    def __init__(self, printer, name="mcu"):
        self._printer = printer
        self._name = name
        self._next_oid = 0
        self.config_cmds = []
        self.lookups = []
        self.sends = []
        self.config_cbs = []
        self.serial_cbs = []
        self.known = set()
        self.enumerations = {}
        self.constants = {}
        self.fileoutput = False
        self.i2c_status = "SUCCESS"
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
    def register_serial_response(self, cb, fmt, oid=None):
        self.serial_cbs.append("%s|%s" % (fmt, oid))

class FakeResolver:
    def __init__(self, pins):
        self.pins = pins
    def reserve_pin(self, pin, reserve_name):
        self.pins.reserved.append("%s|%s" % (pin, reserve_name))

class FakePins:
    error = _PinsError
    def __init__(self, printer):
        self.printer = printer
        self.chips = {}
        self.active = {}
        self.reserved = []
        self.resets = []
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
        self.resets.append("%s:%s" % (pin_params['chip_name'], pin_params['pin']))
        share_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])
        del self.active[share_name]
    def get_pin_resolver(self, chip_name):
        if chip_name not in self.chips:
            raise self.error("Unknown chip name '%s'" % (chip_name,))
        return FakeResolver(self)

class FakeConfigFile:
    def __init__(self, printer):
        self.printer = printer
        self.deprecated = []
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        self.deprecated.append("%s|%r" % (feature, msg))
    def get_printer(self):
        return self.printer

class FakePrinter:
    def __init__(self):
        self.objects = {}
        self.handlers = {}
        self.shutdowns = []
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            return None
        raise configparser.Error("missing %s" % name)
    def add_object(self, name, obj):
        self.objects[name] = obj
    def register_event_handler(self, event, cb):
        self.handlers.setdefault(event, []).append(cb)
    def invoke_shutdown(self, msg, details={}):
        self.shutdowns.append(msg)

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
    def get(self, option, default=_sentinel, note_valid=True):
        if option not in self.values:
            if default is not _sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return self.values[option]
    def getint(self, option, default=_sentinel, minval=None, maxval=None, note_valid=True):
        if option not in self.values:
            if default is not _sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        try:
            v = int(self.values[option])
        except Exception:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v

def make_world(section="test", values=None, known=None, enumerations=None, constants=None, fileoutput=False, i2c_status="SUCCESS"):
    p = FakePrinter()
    mcu = FakeMcu(p, "mcu")
    mcu.known = set(known or [])
    mcu.enumerations = enumerations or {}
    mcu.constants = constants or {}
    mcu.fileoutput = fileoutput
    mcu.i2c_status = i2c_status
    aux = FakeMcu(p, "aux")
    pins = FakePins(p)
    pins.chips = {"mcu": mcu, "aux": aux}
    cf = FakeConfigFile(p)
    p.objects = {"pins": pins, "configfile": cf, "mcu": mcu, "mcu aux": aux}
    cfg = FakeConfig(p, section, values or {})
    return p, mcu, aux, pins, cf, cfg
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
        PyValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
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
    status: Arc<GCell<String>>,
}

impl FakeQuery {
    fn resp(&self) -> PyValue {
        let mut d = IndexMap::new();
        if self.fmt.starts_with("spi_transfer") {
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(vec![0xde, 0xad, 0xbe, 0xef]));
        } else if self.fmt.starts_with("i2c_read") {
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(vec![0x10, 0x20]));
        } else if self.fmt.starts_with("i2c_transfer") {
            d.insert(PyValue::Str("i2c_bus_status".to_string()), PyValue::Str(self.status.borrow().clone()));
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(vec![0x99]));
        } else {
            d.insert(PyValue::Str("response".to_string()), PyValue::Bytes(Vec::new()));
        }
        PyValue::Dict(d)
    }
}

impl CommandQueryWrapperApi for FakeQuery {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        fn pb(b: bool) -> &'static str { if b { "True" } else { "False" } }
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
        fn pb(b: bool) -> &'static str { if b { "True" } else { "False" } }
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
    i2c_status: Arc<GCell<String>>,
    config_cbs: GCell<Vec<ConfigCallback>>,
    serial_cbs: Log,
    serial_cb: GCell<Option<SerialResponseCallback>>,
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
            i2c_status: Arc::new(GCell::new("SUCCESS".to_string())),
            config_cbs: GCell::new(Vec::new()),
            serial_cbs: new_log(),
            serial_cb: GCell::new(None),
        })
    }
    fn run_config_cbs(&self) {
        let mut cbs = std::mem::take(&mut *self.config_cbs.borrow_mut());
        for cb in cbs.iter_mut() {
            cb().expect("config callback");
        }
        *self.config_cbs.borrow_mut() = cbs;
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String { self.name.clone() }
    fn get_printer(&self) -> Printer { self.printer.borrow().clone().expect("printer set") }
    fn is_fileoutput(&self) -> bool { self.fileoutput.get() }
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
        fn pb(b: bool) -> &'static str { if b { "True" } else { "False" } }
        self.config_cmds.borrow_mut().push(format!(
            "{}|{}|{}",
            pycompat::text::str_repr(cmd),
            pb(opts.is_init),
            pb(opts.on_restart)
        ));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 { 0 }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        let n = trunc_float_to_bigint(time * 1000000.0);
        n.0.to_string().parse::<i64>().unwrap_or(0)
    }
    fn min_schedule_time(&self) -> f64 { 0.1 }
    fn max_nominal_duration(&self) -> f64 { 3.0 }
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
        Ok(Arc::new(FakeQuery { log: self.sends.clone(), fmt: msgformat.to_string(), status: self.i2c_status.clone() }))
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.lookups.borrow_mut().push(format!("try|{msgformat}"));
        if self.known.borrow().iter().any(|k| k == msgformat) {
            Some(Arc::new(FakeCmd { log: self.sends.clone(), fmt: msgformat.to_string() }))
        } else {
            None
        }
    }
    fn alloc_command_queue(&self) -> CommandQueue { CommandQueue(7) }
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
    fn check_valid_response(&self, _msgformat: &str) -> bool { true }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.enumerations.borrow().clone()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.constants.borrow().clone()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> { Ok(0.0) }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 { 0 }
    fn clock_to_print_time(&self, clock: f64) -> f64 { clock }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { eventtime }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { clock32 }
    fn calibrate_clock(&self, print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, 0.0))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn stats(&self, _eventtime: f64) -> (bool, String) { (false, String::new()) }
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
    fn alias_pin(&self, _alias: &str, _pin: &str) -> KResult<()> { Ok(()) }
    fn update_command(&self, cmd: &str) -> KResult<String> { Ok(cmd.to_string()) }
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
        let pp = self.parse_pin(pin_desc, ParsePinOpts {
            can_invert: opts.can_invert,
            can_pullup: opts.can_pullup,
        })?;
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
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> { Ok(()) }
}

klippy_api::printer_object!(FakePins; dyn PinsApi);

struct FakeConfigFile {
    deprecated: Log,
}

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer not used by bus")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        Err(KlippyError::Config("not used".to_string()))
    }
    fn read_main_config(&self) -> KResult<Config> {
        Err(KlippyError::Config("not used".to_string()))
    }
    fn log_config(&self, _config: &Config) {}
    fn check_unused_options(&self, _config: &Config) -> KResult<()> { Ok(()) }
    fn runtime_warning(&self, _msg: &str) {}
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {}
    fn deprecate_gcode(&self, _cmd: &str, _opts: DeprecateGcodeOpts) {}
    fn deprecate_mcu_code(&self, _mcu: &Mcu, feature: &str, msg: Option<String>) {
        self.deprecated.borrow_mut().push(format!("{feature}|{:?}", msg));
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue { PyValue::None }
    fn set(&self, _section: &str, _option: &str, _value: &str) -> KResult<()> { Ok(()) }
    fn remove_section(&self, _section: &str) {}
}

klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    handlers: GCell<Vec<(String, EventCallback)>>,
    shutdowns: Log,
}

impl FakePrinter {
    fn new() -> Arc<FakePrinter> {
        Arc::new(FakePrinter {
            objects: GCell::new(IndexMap::new()),
            handlers: GCell::new(Vec::new()),
            shutdowns: new_log(),
        })
    }
    fn add(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }
    fn fire_connect_one(&self) -> KResult<()> {
        // Find first klippy:connect handler and call it once. EventCallback
        // is FnMut, so temporarily take it out of the list.
        let pos = {
            let h = self.handlers.borrow();
            h.iter().position(|(e, _)| e == "klippy:connect")
        };
        let Some(pos) = pos else { return Ok(()) };
        let mut cb = {
            let mut h = self.handlers.borrow_mut();
            // EventCallback does not implement Clone; replace with a no-op so
            // we can move it out, then put the original back.
            let (e, orig) = h.remove(pos);
            let noop: EventCallback = Box::new(|_| Ok(PyValue::None));
            h.insert(pos, (e, noop));
            orig
        };
        let r = cb(&[]);
        let mut h = self.handlers.borrow_mut();
        h[pos].1 = cb;
        r.map(|_| ())
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue { PyValue::None }
    fn get_reactor(&self) -> Reactor { Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { (String::new(), "ready".to_string()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.add(name, obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(self.objects.borrow().get(name).cloned())
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, Option<ObjectRef>)> {
        self.objects.borrow().iter().map(|(k, v)| (k.clone(), v.clone())).map(|(name, obj)| (name, Some(obj))).collect()
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {}
    fn register_event_handler(&self, event: &str, callback: EventCallback) {
        self.handlers.borrow_mut().push((event.to_string(), callback));
    }
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
    fn calls(&self) -> Vec<String> {
        self.calls.borrow().clone()
    }
}

fn missing(section: &str, option: &str) -> KlippyError {
    KlippyError::Config(format!(
        "Option '{option}' in section '{section}' must be specified"
    ))
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn get_name(&self) -> String { self.section.clone() }
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
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> { Ok(None) }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> { Ok(None) }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> { Ok(PyValue::None) }
    fn getlist(&self, _option: &str, _opts: GetListOpts) -> KResult<Option<Vec<String>>> { Ok(None) }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> { Ok(None) }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> { Ok(None) }
    fn getsection(&self, _section: &str) -> Config { panic!("getsection not used by bus") }
    fn has_section(&self, _section: &str) -> bool { false }
    fn get_prefix_sections(&self, _prefix: &str) -> Vec<Config> { Vec::new() }
    fn get_prefix_options(&self, _prefix: &str) -> KResult<Vec<String>> { Ok(Vec::new()) }
    fn deprecate(&self, _option: &str, _value: Option<PyValue>) -> KResult<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bus fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping bus oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "extras.bus".to_string(),
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

struct World {
    printer: Arc<FakePrinter>,
    printer_api: Printer,
    mcu: Arc<FakeMcu>,
    aux: Arc<FakeMcu>,
    pins: Arc<FakePins>,
    configfile: Arc<FakeConfigFile>,
}

fn world() -> World {
    let printer = FakePrinter::new();
    let printer_api: Printer = printer.clone();
    let mcu = FakeMcu::new("mcu");
    let aux = FakeMcu::new("aux");
    *mcu.printer.borrow_mut() = Some(printer_api.clone());
    *aux.printer.borrow_mut() = Some(printer_api.clone());
    let pins = FakePins::new();
    pins.chips.borrow_mut().insert("mcu".to_string(), mcu.clone());
    pins.chips.borrow_mut().insert("aux".to_string(), aux.clone());
    let configfile = Arc::new(FakeConfigFile { deprecated: new_log() });
    printer.add("pins", pins.clone());
    printer.add("configfile", configfile.clone());
    printer.add("mcu", mcu.clone());
    printer.add("mcu aux", aux.clone());
    World { printer, printer_api, mcu, aux, pins, configfile }
}

fn py_world(py: &mut pyoracle::Oracle, known: &[&str], enumerations: &str, constants: &str, fileoutput: bool, i2c_status: &str) {
    let py_bool = if fileoutput { "True" } else { "False" };
    py.exec(&format!(
        "p, mcu, aux, pins, cf, cfg = make_world(known=[{known}], enumerations={enumerations}, constants={constants}, fileoutput={fileoutput}, i2c_status={i2c_status:?})",
        known = known.iter().map(|s| format!("{:?}", s)).collect::<Vec<_>>().join(","),
        enumerations = enumerations,
        constants = constants,
        fileoutput = py_bool,
        i2c_status = i2c_status,
    ))
    .expect("py world");
}

fn rust_world(known: &[&str], enumerations: IndexMap<String, IndexMap<String, i64>>, constants: IndexMap<String, String>, fileoutput: bool, i2c_status: &str) -> World {
    let w = world();
    *w.mcu.known.borrow_mut() = known.iter().map(|s| s.to_string()).collect();
    *w.mcu.enumerations.borrow_mut() = enumerations;
    *w.mcu.constants.borrow_mut() = constants;
    *w.mcu.fileoutput.borrow_mut() = fileoutput;
    *w.mcu.i2c_status.borrow_mut() = i2c_status.to_string();
    w
}

fn enums(pairs: &[(&str, i64)]) -> IndexMap<String, i64> {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k.to_string(), *v);
    }
    m
}

fn enums_map(name: &str, pairs: &[(&str, i64)]) -> IndexMap<String, IndexMap<String, i64>> {
    let mut m = IndexMap::new();
    m.insert(name.to_string(), enums(pairs));
    m
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_has_no_entry_points() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn resolve_bus_name_cases() {
    let Some(mut py) = oracle() else { return };
    // No enumerations, bus given.
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    let expected = py
        .eval("bus.resolve_bus_name(mcu, 'spi_bus', 'spi1')")
        .expect("oracle resolve");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let actual = resolve_bus_name(&mcu, "spi_bus", Some("spi1")).expect("rust resolve");
    check("resolve no-enums given bus", Ok(expected), Value::str(&actual));

    // Enumerations present, bus None resolves via value 0.
    py_world(&mut py, &[], "{'spi_bus': {'spi1': 0, 'spi2': 1}}", "{}", false, "SUCCESS");
    let expected = py
        .eval("bus.resolve_bus_name(mcu, 'spi_bus', None)")
        .expect("oracle resolve 0");
    let w = rust_world(
        &[],
        enums_map("spi_bus", &[("spi1", 0), ("spi2", 1)]),
        IndexMap::new(),
        false,
        "SUCCESS",
    );
    let mcu: Mcu = w.mcu.clone();
    let actual = resolve_bus_name(&mcu, "spi_bus", None).expect("rust resolve 0");
    check("resolve via 0", Ok(expected), Value::str(&actual));

    // Unknown bus.
    py_world(&mut py, &[], "{'spi_bus': {'spi1': 0}}", "{}", false, "SUCCESS");
    let expected = py.eval("bus.resolve_bus_name(mcu, 'spi_bus', 'nope')");
    let w = rust_world(&[], enums_map("spi_bus", &[("spi1", 0)]), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let actual = resolve_bus_name(&mcu, "spi_bus", Some("nope"));
    check_err("resolve unknown bus", expected, actual.err().as_ref().expect("must fail"));

    // Must specify when no 0 enumeration.
    py_world(&mut py, &[], "{'spi_bus': {'spi1': 1}}", "{}", false, "SUCCESS");
    let expected = py.eval("bus.resolve_bus_name(mcu, 'spi_bus', None)");
    let w = rust_world(&[], enums_map("spi_bus", &[("spi1", 1)]), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let actual = resolve_bus_name(&mcu, "spi_bus", None);
    check_err("resolve must specify", expected, actual.err().as_ref().expect("must fail"));

    // Reserved pins recorded on both sides.
    py_world(&mut py, &[], "{'spi_bus': {'spi1': 0}}", "{'BUS_PINS_spi1': 'PA1,PA2'}", false, "SUCCESS");
    let _ = py.eval("bus.resolve_bus_name(mcu, 'spi_bus', 'spi1')").expect("oracle reserve");
    let w = rust_world(
        &[],
        enums_map("spi_bus", &[("spi1", 0)]),
        {
            let mut c = IndexMap::new();
            c.insert("BUS_PINS_spi1".to_string(), "PA1,PA2".to_string());
            c
        },
        false,
        "SUCCESS",
    );
    let mcu: Mcu = w.mcu.clone();
    let _ = resolve_bus_name(&mcu, "spi_bus", Some("spi1")).expect("rust reserve");
    let py_reserved = py.eval("pins.reserved").expect("py reserved");
    let rust_reserved = w.pins.reserved.borrow().clone();
    check("reserved pins", Ok(py_reserved), str_list(&rust_reserved));
}

#[test]
fn spi_hardware_full_flow() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("spi = bus.MCU_SPI(mcu, 'spi1', 'PA5', 0, 100000)").expect("py spi new");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds");
    let expected_oid = py.eval("spi.get_oid()").expect("py oid");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let spi = McuSpi::new(mcu, Some("spi1".to_string()), Some("PA5".to_string()), 0, 100000, None, false)
        .expect("rust spi new");
    check("spi hardware config cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("spi oid", Ok(expected_oid), Value::int(spi.get_oid()));

    // spi_send before build_config goes through is_init config cmd.
    py.exec("spi.spi_send([1, 2, 255])").expect("py spi_send init");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds2");
    spi.spi_send(&[1, 2, 255], SendOpts::default()).expect("rust spi_send init");
    check("spi send init cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));

    // build_config + transfer + preface.
    py.exec("spi.build_config()").expect("py build");
    w.mcu.run_config_cbs();
    let expected_lookups = py.eval("mcu.lookups").expect("py lookups");
    let expected_cmds = py.eval("mcu.config_cmds").expect("py cmds3");
    check("spi build lookups", Ok(expected_lookups), str_list(&w.mcu.lookups.borrow().clone()));
    check("spi build cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));

    let expected = py.eval("spi.spi_transfer([0x81, 0])['response']").expect("py transfer");
    let actual = spi.spi_transfer(&[0x81, 0], QuerySendOpts::default()).expect("rust transfer");
    let resp = match &actual {
        PyValue::Dict(d) => d.get(&PyValue::Str("response".to_string())).cloned().unwrap_or(PyValue::None),
        _ => PyValue::None,
    };
    let resp_bytes = match resp {
        PyValue::Bytes(b) => b,
        _ => panic!("response bytes"),
    };
    check("spi transfer response", Ok(expected), Value::bytes(&resp_bytes));

    let expected = py.eval("spi.spi_transfer_with_preface([0x01], [0x02])['response']").expect("py preface");
    let _ = spi.spi_transfer_with_preface(&[0x01], &[0x02], QuerySendOpts::default()).expect("rust preface");
    let expected_sends = py.eval("mcu.sends").expect("py sends");
    check("spi sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));
    let _ = expected;
}

#[test]
fn spi_without_cs_and_shutdown_msg() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("spi = bus.MCU_SPI(mcu, None, None, 3, 250000, None, True)").expect("py spi");
    py.exec("spi.setup_shutdown_msg([0x00, 0x7e, 0xff])").expect("py shutdown");
    let expected_cmds = py.eval("mcu.config_cmds").expect("cmds");
    let expected_queue = py.eval("spi.get_command_queue()").expect("queue");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let spi = McuSpi::new(mcu, None, None, 3, 250000, None, true).expect("rust spi");
    spi.setup_shutdown_msg(&[0x00, 0x7e, 0xff]).expect("rust shutdown");
    check("spi without cs cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("spi queue", Ok(expected_queue), Value::int(spi.get_command_queue().0));
    check("spi get_mcu name", Ok(Value::str("mcu")), Value::str(&spi.get_mcu().get_name()));
}

#[test]
fn spi_software_bus_ticks_and_fallback() {
    let Some(mut py) = oracle() else { return };
    // Ticks supported.
    py_world(
        &mut py,
        &["spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u"],
        "{}",
        "{}",
        false,
        "SUCCESS",
    );
    py.exec("spi = bus.MCU_SPI(mcu, None, 'PA5', 1, 100000, ('PB1', 'PB2', 'PB3'))").expect("py spi sw");
    py.exec("spi.build_config()").expect("py build sw");
    let expected_cmds = py.eval("mcu.config_cmds").expect("sw cmds");
    let expected_fmt = py.eval("spi.config_fmt").expect("sw fmt");

    let w = rust_world(
        &["spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u mode=%u pulse_ticks=%u"],
        IndexMap::new(),
        IndexMap::new(),
        false,
        "SUCCESS",
    );
    let mcu: Mcu = w.mcu.clone();
    let spi = McuSpi::new(
        mcu,
        None,
        Some("PA5".to_string()),
        1,
        100000,
        Some(["PB1".to_string(), "PB2".to_string(), "PB3".to_string()]),
        false,
    )
    .expect("rust spi sw");
    spi.build_config().expect("rust build sw");
    check("spi sw cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    // config_fmt is internal; the last config cmd carries it.
    let _ = expected_fmt;

    // Ticks not supported: deprecation + software bus fmt.
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("spi2 = bus.MCU_SPI(mcu, None, 'PA5', 1, 100000, ('PB1', 'PB2', 'PB3'))").expect("py spi sw2");
    py.exec("spi2.build_config()").expect("py build sw2");
    let expected_cmds = py.eval("mcu.config_cmds").expect("sw2 cmds");
    let expected_dep = py.eval("cf.deprecated").expect("sw2 dep");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let spi = McuSpi::new(
        mcu,
        None,
        Some("PA5".to_string()),
        1,
        100000,
        Some(["PB1".to_string(), "PB2".to_string(), "PB3".to_string()]),
        false,
    )
    .expect("rust spi sw2");
    spi.build_config().expect("rust build sw2");
    check("spi sw fallback cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("spi sw fallback dep", Ok(expected_dep), str_list(&w.configfile.deprecated.borrow().clone()));
}

#[test]
fn spi_from_config_hardware_software_and_errors() {
    let Some(mut py) = oracle() else { return };
    // Hardware.
    py.exec("p, mcu, aux, pins, cf, cfg = make_world(section='spi_test', values={'cs_pin': 'PA5', 'spi_speed': '500000', 'spi_bus': 'spi1'})").expect("py cfg");
    py.exec("spi = bus.MCU_SPI_from_config(cfg, 0)").expect("py from cfg");
    let expected_cmds = py.eval("mcu.config_cmds").expect("cmds");
    let expected_calls = py.eval("spi.get_oid()").expect("oid");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "spi_test", &[("cs_pin", "PA5"), ("spi_speed", "500000"), ("spi_bus", "spi1")]);
    let spi = mcu_spi_from_config(&cfg, 0, "cs_pin", 100000, None, false).expect("rust from cfg");
    check("spi from cfg cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("spi from cfg oid", Ok(expected_calls), Value::int(spi.get_oid()));

    // cs_pin None resets sharing.
    py.exec("p, mcu, aux, pins, cf, cfg2 = make_world(section='spi_test', values={'cs_pin': 'None'})").expect("py cfg2");
    py.exec("spi2 = bus.MCU_SPI_from_config(cfg2, 0)").expect("py from cfg2");
    let expected_resets = py.eval("pins.resets").expect("resets");
    let expected_cmds2 = py.eval("mcu.config_cmds").expect("cmds2");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "spi_test", &[("cs_pin", "None")]);
    let _ = mcu_spi_from_config(&cfg, 0, "cs_pin", 100000, None, false).expect("rust from cfg2");
    check("spi None resets", Ok(expected_resets), str_list(&w.pins.resets.borrow().clone()));
    check("spi None cmds", Ok(expected_cmds2), str_list(&w.mcu.config_cmds.borrow().clone()));

    // Software pins on another mcu.
    py.exec("p, mcu, aux, pins, cf, cfg3 = make_world(section='spi_test', values={'cs_pin': 'PA5', 'spi_software_sclk_pin': 'PB3', 'spi_software_miso_pin': 'PB1', 'spi_software_mosi_pin': 'aux:PB2'})").expect("py cfg3");
    let expected = py.eval("bus.MCU_SPI_from_config(cfg3, 0)");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "spi_test", &[("cs_pin", "PA5"), ("spi_software_sclk_pin", "PB3"), ("spi_software_miso_pin", "PB1"), ("spi_software_mosi_pin", "aux:PB2")]);
    let actual = mcu_spi_from_config(&cfg, 0, "cs_pin", 100000, None, false);
    check_err("spi sw wrong mcu", expected, actual.err().as_ref().expect("must fail"));

    // spi_speed below minimum.
    py.exec("p, mcu, aux, pins, cf, cfg4 = make_world(section='spi_test', values={'cs_pin': 'PA5', 'spi_speed': '100'})").expect("py cfg4");
    let expected = py.eval("bus.MCU_SPI_from_config(cfg4, 0)");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "spi_test", &[("cs_pin", "PA5"), ("spi_speed", "100")]);
    let actual = mcu_spi_from_config(&cfg, 0, "cs_pin", 100000, None, false);
    check_err("spi speed min", expected, actual.err().as_ref().expect("must fail"));
}

#[test]
fn i2c_new_style_flow() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &["i2c_read oid=%c reg=%*s read_len=%u"], "{}", "{}", false, "SUCCESS");
    py.exec("i2c = bus.MCU_I2C(mcu, 'i2c1', 0x20, 100000)").expect("py i2c");
    py.exec("i2c.build_config()").expect("py build");
    let expected_cmds = py.eval("mcu.config_cmds").expect("cmds");
    let expected_lookups = py.eval("mcu.lookups").expect("lookups");
    let expected_dep = py.eval("cf.deprecated").expect("dep");
    let expected_addr = py.eval("i2c.get_i2c_address()").expect("addr");

    let w = rust_world(&["i2c_read oid=%c reg=%*s read_len=%u"], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(mcu, Some("i2c1".to_string()), 0x20, 100000, None, false).expect("rust i2c");
    i2c.build_config().expect("rust build");
    check("i2c new cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("i2c new lookups", Ok(expected_lookups), str_list(&w.mcu.lookups.borrow().clone()));
    check("i2c new dep", Ok(expected_dep), str_list(&w.configfile.deprecated.borrow().clone()));
    check("i2c addr", Ok(expected_addr), Value::int(i2c.get_i2c_address()));

    py.exec("i2c.i2c_write([0x01, 0x02])").expect("py write");
    i2c.i2c_write(&[0x01, 0x02], QuerySendOpts::default()).expect("rust write");
    let expected = py.eval("i2c.i2c_read([0x03], 2)['response']").expect("py read");
    let actual = i2c.i2c_read(&[0x03], 2, QuerySendOpts::default()).expect("rust read");
    let resp = match &actual {
        PyValue::Dict(d) => d.get(&PyValue::Str("response".to_string())).cloned().unwrap_or(PyValue::None),
        _ => PyValue::None,
    };
    let resp_bytes = match resp {
        PyValue::Bytes(b) => b,
        _ => panic!("response bytes"),
    };
    check("i2c read response", Ok(expected), Value::bytes(&resp_bytes));
    py.exec("i2c.i2c_write_noack([0x04])").expect("py noack");
    i2c.i2c_write_noack(&[0x04], SendOpts::default()).expect("rust noack");
    let expected_sends = py.eval("mcu.sends").expect("sends");
    check("i2c new sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));

    // i2c_transfer has no command in this style.
    let expected = py.eval("i2c.i2c_transfer([0x05], 1)");
    let actual = i2c.i2c_transfer(&[0x05], 1, QuerySendOpts::default());
    check_err("i2c transfer unconfigured cmd", expected, actual.err().as_ref().expect("must fail"));
}

#[test]
fn i2c_transfer_style_and_shutdown_on_error() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("i2c = bus.MCU_I2C(mcu, 'i2c1', 0x21, 100000)").expect("py i2c");
    py.exec("i2c.build_config()").expect("py build");
    py.exec("i2c.i2c_write([0x01])").expect("py write");
    let expected_sends = py.eval("mcu.sends").expect("sends");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(mcu, Some("i2c1".to_string()), 0x21, 100000, None, false).expect("rust i2c");
    i2c.build_config().expect("rust build");
    i2c.i2c_write(&[0x01], QuerySendOpts::default()).expect("rust write");
    check("i2c transfer style sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));

    // Error status shuts down.
    py.exec("mcu.i2c_status = 'TIMEOUT'").expect("status py");
    py.exec("i2c.i2c_transfer([0x02], 1)").expect("py transfer err");
    let expected_shutdowns = py.eval("p.shutdowns").expect("shutdowns");
    *w.mcu.i2c_status.borrow_mut() = "TIMEOUT".to_string();
    let _ = i2c.i2c_transfer(&[0x02], 1, QuerySendOpts::default()).expect("rust transfer err");
    check("i2c error shutdown", Ok(expected_shutdowns), str_list(&w.printer.shutdowns.borrow().clone()));
}

#[test]
fn i2c_fileoutput_and_async_write_only() {
    let Some(mut py) = oracle() else { return };
    // Fileoutput: transfer becomes a plain command.
    py_world(&mut py, &[], "{}", "{}", true, "SUCCESS");
    py.exec("i2c = bus.MCU_I2C(mcu, 'i2c1', 0x22, 100000)").expect("py i2c");
    py.exec("i2c.build_config()").expect("py build");
    py.exec("r = i2c.i2c_transfer([0x01], 2)").expect("py transfer");
    let expected = py.eval("r").expect("transfer ret");
    let expected_sends = py.eval("mcu.sends").expect("sends");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), true, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(mcu, Some("i2c1".to_string()), 0x22, 100000, None, false).expect("rust i2c");
    i2c.build_config().expect("rust build");
    let actual = i2c.i2c_transfer(&[0x01], 2, QuerySendOpts::default()).expect("rust transfer");
    check("i2c fileoutput transfer ret", Ok(expected), match actual {
        PyValue::None => Value::Null,
        _ => panic!("None expected"),
    });
    check("i2c fileoutput sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));

    // Async write only.
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("i2c2 = bus.MCU_I2C(mcu, 'i2c1', 0x23, 100000, None, True)").expect("py i2c2");
    py.exec("i2c2.build_config()").expect("py build2");
    let expected_serial = py.eval("mcu.serial_cbs").expect("serial");
    let expected_read_err = py.eval("i2c2.i2c_read([0x01], 1)");
    py.exec("i2c2.i2c_write([0x09])").expect("py async write");
    py.exec("i2c2.i2c_write_noack([0x0a])").expect("py async noack");
    let expected_sends2 = py.eval("mcu.sends").expect("sends2");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(mcu, Some("i2c1".to_string()), 0x23, 100000, None, true).expect("rust i2c2");
    i2c.build_config().expect("rust build2");
    check(
        "i2c async serial registration",
        Ok(expected_serial),
        str_list(&w.mcu.serial_cbs.borrow().clone()),
    );
    let actual_err = i2c.i2c_read(&[0x01], 1, QuerySendOpts::default());
    check_err("i2c async read write-only", expected_read_err, actual_err.err().as_ref().expect("must fail"));
    i2c.i2c_write(&[0x09], QuerySendOpts::default()).expect("rust async write");
    i2c.i2c_write_noack(&[0x0a], SendOpts::default()).expect("rust async noack");
    check("i2c async sends", Ok(expected_sends2), str_list(&w.mcu.sends.borrow().clone()));
}

#[test]
fn i2c_queued_writes_flush_on_connect() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &["i2c_read oid=%c reg=%*s read_len=%u"], "{}", "{}", false, "SUCCESS");
    py.exec("i2c = bus.MCU_I2C(mcu, 'i2c1', 0x24, 100000)").expect("py i2c");
    py.exec("i2c.i2c_write([0x01])").expect("py queued write");
    py.exec("i2c.i2c_write([0x02])").expect("py queued write2");
    py.exec("i2c.build_config()").expect("py build");
    py.exec("p.handlers['klippy:connect'][0]()").expect("py connect");
    let expected_sends = py.eval("mcu.sends").expect("flush sends");

    let w = rust_world(&["i2c_read oid=%c reg=%*s read_len=%u"], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(mcu, Some("i2c1".to_string()), 0x24, 100000, None, false).expect("rust i2c");
    i2c.i2c_write(&[0x01], QuerySendOpts::default()).expect("queued1");
    i2c.i2c_write(&[0x02], QuerySendOpts::default()).expect("queued2");
    i2c.build_config().expect("build");
    w.printer.fire_connect_one().expect("connect");
    check("i2c flush sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));
}

#[test]
fn i2c_from_config_cases() {
    let Some(mut py) = oracle() else { return };
    // Named MCU + default address.
    py.exec("p, mcu, aux, pins, cf, cfg = make_world(section='i2c_test', values={'i2c_mcu': 'aux', 'i2c_speed': '400000'})").expect("py cfg");
    py.exec("i2c = bus.MCU_I2C_from_config(cfg, default_addr=0x33)").expect("py from cfg");
    let expected = py.eval("aux.config_cmds").expect("aux cmds");
    let expected_addr = py.eval("i2c.get_i2c_address()").expect("addr");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "i2c_test", &[("i2c_mcu", "aux"), ("i2c_speed", "400000")]);
    let i2c = mcu_i2c_from_config(&cfg, Some(0x33), 100000, false).expect("rust from cfg");
    check("i2c from cfg cmds", Ok(expected), str_list(&w.aux.config_cmds.borrow().clone()));
    check("i2c from cfg addr", Ok(expected_addr), Value::int(i2c.get_i2c_address()));

    // Missing required address.
    py.exec("p, mcu, aux, pins, cf, cfg2 = make_world(section='i2c_test', values={})").expect("py cfg2");
    let expected = py.eval("bus.MCU_I2C_from_config(cfg2)");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "i2c_test", &[]);
    let actual = mcu_i2c_from_config(&cfg, None, 100000, false);
    check_err("i2c missing address", expected, actual.err().as_ref().expect("must fail"));

    // Software pins on another mcu.
    py.exec("p, mcu, aux, pins, cf, cfg3 = make_world(section='i2c_test', values={'i2c_address': '51', 'i2c_software_scl_pin': 'PB1', 'i2c_software_sda_pin': 'aux:PB2'})").expect("py cfg3");
    let expected = py.eval("bus.MCU_I2C_from_config(cfg3, default_addr=0x33)");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let cfg = FakeConfig::new(w.printer_api.clone(), "i2c_test", &[("i2c_address", "51"), ("i2c_software_scl_pin", "PB1"), ("i2c_software_sda_pin", "aux:PB2")]);
    let actual = mcu_i2c_from_config(&cfg, Some(0x33), 100000, false);
    check_err("i2c sw wrong mcu", expected, actual.err().as_ref().expect("must fail"));
}

#[test]
fn i2c_software_bus_ticks() {
    let Some(mut py) = oracle() else { return };
    py_world(
        &mut py,
        &["i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u"],
        "{}",
        "{}",
        false,
        "SUCCESS",
    );
    py.exec("i2c = bus.MCU_I2C(mcu, None, 0x20, 100000, ('PB1', 'PB2'))").expect("py i2c sw");
    py.exec("i2c.build_config()").expect("py build");
    let expected_cmds = py.eval("mcu.config_cmds").expect("cmds");

    let w = rust_world(
        &["i2c_set_sw_bus oid=%c scl_pin=%u sda_pin=%u pulse_ticks=%u address=%u"],
        IndexMap::new(),
        IndexMap::new(),
        false,
        "SUCCESS",
    );
    let mcu: Mcu = w.mcu.clone();
    let i2c = McuI2c::new(
        mcu,
        None,
        0x20,
        100000,
        Some(["PB1".to_string(), "PB2".to_string()]),
        false,
    )
    .expect("rust i2c sw");
    i2c.build_config().expect("rust build");
    check("i2c sw cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
}

#[test]
fn bus_digital_out_flow() {
    let Some(mut py) = oracle() else { return };
    py_world(&mut py, &[], "{}", "{}", false, "SUCCESS");
    py.exec("out = bus.MCU_bus_digital_out(mcu, 'PA5', None, 1)").expect("py out");
    py.exec("out.update_digital_out(True)").expect("py update init");
    let expected_cmds = py.eval("mcu.config_cmds").expect("cmds");
    let expected_oid = py.eval("out.get_oid()").expect("oid");

    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let out = McuBusDigitalOut::new(mcu, "PA5", None, 1).expect("rust out");
    out.update_digital_out(true, SendOpts::default()).expect("rust update init");
    check("digital out cmds", Ok(expected_cmds), str_list(&w.mcu.config_cmds.borrow().clone()));
    check("digital out oid", Ok(expected_oid), Value::int(out.get_oid()));

    py.exec("out.build_config()").expect("py build");
    out.build_config().expect("rust build");
    py.exec("out.update_digital_out(False)").expect("py update");
    out.update_digital_out(false, SendOpts::default()).expect("rust update");
    let expected_sends = py.eval("mcu.sends").expect("sends");
    let expected_lookups = py.eval("mcu.lookups").expect("lookups");
    check("digital out sends", Ok(expected_sends), str_list(&w.mcu.sends.borrow().clone()));
    check("digital out lookups", Ok(expected_lookups), str_list(&w.mcu.lookups.borrow().clone()));

    // Pin on another mcu.
    // Pin on another mcu raises pins.error.
    let expected = py.eval("bus.MCU_bus_digital_out(mcu, 'aux:PA5')");
    let w = rust_world(&[], IndexMap::new(), IndexMap::new(), false, "SUCCESS");
    let mcu: Mcu = w.mcu.clone();
    let actual = McuBusDigitalOut::new(mcu, "aux:PA5", None, 0);
    check_err("digital out wrong mcu", expected, actual.err().as_ref().expect("must fail"));
}
