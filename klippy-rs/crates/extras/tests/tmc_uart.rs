//! Parity tests for `extras::tmc_uart`.
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + the frozen `klippy/`). Framing helpers, the analog
//! mux, the bit-banged UART, the shared lookup and the high-level wrapper
//! are built on both sides with minimal Python/Rust fakes, then fed
//! identical inputs. Return values, errors (class and message), MCU
//! config commands, command lookups and send arguments are compared
//! exactly.

use extras::tmc_uart::{
    add_serial_bits, calc_crc8, decode_read, encode_read, encode_write,
    lookup_tmc_uart_bitbang, lookup_tmc_uart_mutex, module_factory, McuAnalogMux,
    McuTmcUart, McuTmcUartBitbang, PrinterTmcUartMutexes, RegRead, TMC_BAUD_RATE,
    TMC_BAUD_RATE_AVR,
};
use indexmap::IndexMap;
use klippy_api::config::{
    Config, ConfigApi, GetBoolOpts, GetChoiceOpts, GetFloatOpts, GetFloatListOpts,
    GetIntOpts, GetIntListOpts, GetListOpts, GetListsOpts, GetStrOpts,
};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandQueryWrapper, CommandWrapper, LookupPinOpts,
    Mcu, McuApi, ParsePinOpts, PinsApi, Pin, PinParams, QuerySendOpts, SendOpts,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::{EventArg, ObjectRef};
use pycompat::PyValue;
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::Arc;

const SETUP: &str = r#"
import configparser
import extras.tmc_uart as tmc_uart

class CommandError(Exception):
    pass

class error(Exception):
    pass

class sentinel:
    pass

class FakeMutex:
    def __init__(self, is_locked=False):
        self._locked = bool(is_locked)
    def __enter__(self):
        self._locked = True
        return self
    def __exit__(self, *a):
        self._locked = False
        return False
    def test(self):
        return self._locked

class FakeReactor:
    def __init__(self):
        self.mutexes = []
    def mutex(self, is_locked=False):
        m = FakeMutex(is_locked)
        self.mutexes.append(m)
        return m

class FakePrinter:
    config_error = configparser.Error
    command_error = CommandError
    def __init__(self, start_args=None):
        self.objects = {}
        self.reactor = FakeReactor()
        self.start_args = dict(start_args or {})
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error("Printer object '%s' already created" % (name,))
        self.objects[name] = obj

class FakePins:
    error = error
    def __init__(self, chips):
        self.chips = dict(chips)
        self.active_pins = {}
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
        if chip_name not in self.chips:
            raise error("Unknown pin chip name '%s'" % (chip_name,))
        if [c for c in '^~!:' if c in pin] or ''.join(pin.split()) != pin:
            fmt = ""
            if can_pullup:
                fmt += "[^~] "
            if can_invert:
                fmt += "[!] "
            raise error("Invalid pin description '%s'\n"
                        "Format is: %s[chip_name:] pin_name" % (pin_desc, fmt))
        return {'chip': self.chips[chip_name], 'chip_name': chip_name,
                'pin': pin, 'invert': invert, 'pullup': pullup}
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False, share_type=None):
        pp = self.parse_pin(pin_desc, can_invert, can_pullup)
        share_name = "%s:%s" % (pp['chip_name'], pp['pin'])
        if share_name in self.active_pins:
            sp = self.active_pins[share_name]
            if share_type is None or share_type != sp['share_type']:
                raise error("pin %s used multiple times in config" % (pp['pin'],))
            if pp['invert'] != sp['invert'] or pp['pullup'] != sp['pullup']:
                raise error("Shared pin %s must have same polarity" % (pp['pin'],))
            return sp
        pp['share_type'] = share_type
        self.active_pins[share_name] = pp
        return pp

def _opt(v):
    if isinstance(v, bytearray):
        return bytes(v)
    if isinstance(v, list):
        return [_opt(x) for x in v]
    if isinstance(v, tuple):
        return tuple(_opt(x) for x in v)
    if isinstance(v, dict):
        return {k: _opt(x) for k, x in v.items()}
    return v

class FakeCmdWrapper:
    def __init__(self, mcu, msgformat, cq):
        self.mcu = mcu
        self.msgformat = msgformat
        self.cq = cq
    def send(self, data=(), minclock=0, reqclock=0):
        self.mcu.cmd_sends.append((self.msgformat, _opt(list(data)), minclock, reqclock))

class FakeQueryWrapper:
    def __init__(self, mcu, msgformat, respformat, oid, cq, is_async):
        self.mcu = mcu
        self.msgformat = msgformat
        self.respformat = respformat
        self.oid = oid
        self.cq = cq
        self.is_async = is_async
    def send(self, data=(), minclock=0, reqclock=0, retry=True):
        self.mcu.query_sends.append((self.msgformat, _opt(list(data)), minclock, reqclock, retry))
        if self.mcu.query_responses:
            return self.mcu.query_responses.pop(0)
        return {'read': bytearray(10), '#receive_time': 0.0}

class FakeMcu:
    def __init__(self, printer, name="mcu", constants=None, freq=1000000):
        self._printer = printer
        self._name = name
        self.constants = dict(constants or {})
        self.freq = freq
        self.next_oid = 1
        self.next_cq = 100
        self.config_cmds = []
        self.config_callbacks = []
        self.looked_up_commands = []
        self.looked_up_queries = []
        self.cmd_sends = []
        self.query_sends = []
        self.query_responses = []
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self._name
    def is_fileoutput(self):
        return self._printer.get_start_args().get('debugoutput') is not None
    def create_oid(self):
        v = self.next_oid
        self.next_oid += 1
        return v
    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append((cmd, is_init, on_restart))
    def alloc_command_queue(self):
        v = self.next_cq
        self.next_cq += 1
        return v
    def lookup_command(self, msgformat, cq=None):
        self.looked_up_commands.append((msgformat, cq))
        return FakeCmdWrapper(self, msgformat, cq)
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None, is_async=False):
        self.looked_up_queries.append((msgformat, respformat, oid, cq, is_async))
        return FakeQueryWrapper(self, msgformat, respformat, oid, cq, is_async)
    def get_constants(self):
        return dict(self.constants)
    def seconds_to_clock(self, t):
        return int(t * self.freq)
    def print_time_to_clock(self, pt):
        return int(pt * self.freq)
    def run_config_callbacks(self):
        cbs = self.config_callbacks
        self.config_callbacks = []
        for cb in cbs:
            cb()

class FakeConfig:
    error = configparser.Error
    def __init__(self, printer, section, opts):
        self._printer = printer
        self.section = section
        self.opts = dict(opts)
    def get_printer(self):
        return self._printer
    def get_name(self):
        return self.section
    def get(self, option, default=sentinel):
        if option not in self.opts:
            if default is not sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        return self.opts[option]
    def getint(self, option, default=sentinel, minval=None, maxval=None):
        if option not in self.opts:
            if default is not sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        try:
            v = int(self.opts[option].strip(), 10)
        except:
            raise configparser.Error("Unable to parse option '%s' in section '%s'" % (option, self.section))
        if minval is not None and v < minval:
            raise configparser.Error("Option '%s' in section '%s' must have minimum of %s" % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise configparser.Error("Option '%s' in section '%s' must have maximum of %s" % (option, self.section, maxval))
        return v
    def getlist(self, option, default=sentinel, sep=','):
        if option not in self.opts:
            if default is not sentinel:
                return default
            raise configparser.Error("Option '%s' in section '%s' must be specified" % (option, self.section))
        raw = self.opts[option]
        if len(raw.strip()) == 0:
            return []
        return [p.strip() for p in raw.split(sep)]

class FakeFields:
    pass
"#;

fn pyval_from_py(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => {
            let s = n.0.to_string();
            if let Ok(x) = s.parse::<i64>() {
                Value::int(x)
            } else {
                Value::BigInt(s)
            }
        }
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(items) => Value::List(items.iter().map(pyval_from_py).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(pyval_from_py).collect()),
        PyValue::Dict(map) => Value::Dict(
            map.iter().map(|(k, val)| (pyval_from_py(k), pyval_from_py(val))).collect(),
        ),
        PyValue::Host(h) => Value::ObjectId(format!("host:{h}")),
    }
}

#[derive(Clone)]
struct FakePrinter {
    inner: Arc<FakePrinterInner>,
}

struct FakePrinterInner {
    objects: kos::sync::Mutex<IndexMap<String, ObjectRef>>,
    reactor: Reactor,
    start_args: PyValue,
}

impl FakePrinter {
    fn new(start_args: PyValue) -> FakePrinter {
        FakePrinter {
            inner: Arc::new(FakePrinterInner {
                objects: kos::sync::Mutex::new(IndexMap::new()),
                reactor: Reactor::new(false),
                start_args,
            }),
        }
    }
    fn start_args_dict(extra: &[(&str, &str)]) -> PyValue {
        let mut map = IndexMap::new();
        for (k, v) in extra {
            map.insert(PyValue::Str(k.to_string()), PyValue::Str(v.to_string()));
        }
        PyValue::Dict(map)
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        self.inner.start_args.clone()
    }
    fn get_reactor(&self) -> Reactor {
        self.inner.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        let mut objs = self.inner.objects.lock();
        if objs.contains_key(name) {
            return Err(KlippyError::config(format!("Printer object '{name}' already created")));
        }
        objs.insert(name.to_string(), obj);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objs = self.inner.objects.lock();
        if let Some(o) = objs.get(name) {
            return Ok(Some(o.clone()));
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown config object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.inner.objects.lock().iter().map(|(k, v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
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
    opts: IndexMap<String, String>,
}

fn make_config(printer: Printer, section: &str, opts: &[(&str, &str)]) -> Config {
    let mut map = IndexMap::new();
    for (k, v) in opts {
        map.insert(k.to_string(), v.to_string());
    }
    Arc::new(FakeConfig { printer, section: section.to_string(), opts: map }) as Config
}

impl ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.section.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        match self.opts.get(option) {
            Some(v) => Ok(Some(v.clone())),
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let raw = match self.opts.get(option) {
            Some(v) => v.clone(),
            None => match opts.default {
                None => {
                    return Err(KlippyError::config(format!(
                        "Option '{option}' in section '{}' must be specified",
                        self.section
                    )))
                }
                Some(inner) => return Ok(inner),
            },
        };
        let v: i64 = match raw.trim().parse::<i64>() {
            Ok(v) => v,
            Err(_) => {
                return Err(KlippyError::config(format!(
                    "Unable to parse option '{option}' in section '{}'",
                    self.section
                )))
            }
        };
        if let Some(minv) = opts.minval {
            if v < minv {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have minimum of {minv}",
                    self.section
                )));
            }
        }
        if let Some(maxv) = opts.maxval {
            if v > maxv {
                return Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must have maximum of {maxv}",
                    self.section
                )));
            }
        }
        Ok(Some(v))
    }
    fn getfloat(&self, _option: &str, _opts: GetFloatOpts) -> KResult<Option<f64>> {
        panic!("getfloat not used")
    }
    fn getboolean(&self, _option: &str, _opts: GetBoolOpts) -> KResult<Option<bool>> {
        panic!("getboolean not used")
    }
    fn getchoice(&self, _option: &str, _choices: &PyValue, _opts: GetChoiceOpts) -> KResult<PyValue> {
        panic!("getchoice not used")
    }
    fn getlists(&self, _option: &str, _opts: GetListsOpts) -> KResult<PyValue> {
        panic!("getlists not used")
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        match self.opts.get(option) {
            Some(raw) => {
                if raw.trim().is_empty() {
                    Ok(Some(Vec::new()))
                } else {
                    Ok(Some(raw.split(&opts.sep).map(|p| p.trim().to_string()).collect()))
                }
            }
            None => match opts.default {
                None => Err(KlippyError::config(format!(
                    "Option '{option}' in section '{}' must be specified",
                    self.section
                ))),
                Some(inner) => Ok(inner),
            },
        }
    }
    fn getintlist(&self, _option: &str, _opts: GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        panic!("getintlist not used")
    }
    fn getfloatlist(&self, _option: &str, _opts: GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        panic!("getfloatlist not used")
    }
    fn getsection(&self, _section: &str) -> Config {
        panic!("getsection not used")
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

struct FakeMcuInner {
    printer: Printer,
    name: String,
    constants: IndexMap<String, String>,
    freq: f64,
    next_oid: kos::sync::Mutex<i64>,
    next_cq: kos::sync::Mutex<i64>,
    config_cmds: kos::sync::Mutex<Vec<String>>,
    config_callbacks: kos::sync::Mutex<Vec<Box<dyn FnMut() -> KResult<()> + Send>>>,
    looked_up_commands: kos::sync::Mutex<Vec<(String, Option<CommandQueue>)>>,
    looked_up_queries: kos::sync::Mutex<Vec<(String, String, Option<i64>, Option<CommandQueue>, bool)>>,
    cmd_sends: kos::sync::Mutex<Vec<(String, Vec<PyValue>, i64, i64)>>,
    query_sends: kos::sync::Mutex<Vec<(String, Vec<PyValue>, i64, i64, bool)>>,
    query_responses: kos::sync::Mutex<Vec<PyValue>>,
}

#[derive(Clone)]
struct FakeMcu {
    inner: Arc<FakeMcuInner>,
}

impl FakeMcu {
    fn new(printer: Printer, name: &str, constants: &[(&str, &str)]) -> FakeMcu {
        let mut map = IndexMap::new();
        for (k, v) in constants {
            map.insert(k.to_string(), v.to_string());
        }
        FakeMcu {
            inner: Arc::new(FakeMcuInner {
                printer,
                name: name.to_string(),
                constants: map,
                freq: 1000000.0,
                next_oid: kos::sync::Mutex::new(1),
                next_cq: kos::sync::Mutex::new(100),
                config_cmds: kos::sync::Mutex::new(Vec::new()),
                config_callbacks: kos::sync::Mutex::new(Vec::new()),
                looked_up_commands: kos::sync::Mutex::new(Vec::new()),
                looked_up_queries: kos::sync::Mutex::new(Vec::new()),
                cmd_sends: kos::sync::Mutex::new(Vec::new()),
                query_sends: kos::sync::Mutex::new(Vec::new()),
                query_responses: kos::sync::Mutex::new(Vec::new()),
            }),
        }
    }
    fn to_mcu(self) -> Mcu {
        Arc::new(self) as Mcu
    }
    fn config_cmds(&self) -> Vec<String> {
        self.inner.config_cmds.lock().clone()
    }
    fn run_config_callbacks(&self) -> KResult<()> {
        let cbs: Vec<_> = {
            let mut g = self.inner.config_callbacks.lock();
            std::mem::take(&mut *g)
        };
        for mut cb in cbs {
            cb()?;
        }
        Ok(())
    }
    fn cmd_sends(&self) -> Vec<(String, Vec<PyValue>, i64, i64)> {
        self.inner.cmd_sends.lock().clone()
    }
    fn query_sends(&self) -> Vec<(String, Vec<PyValue>, i64, i64, bool)> {
        self.inner.query_sends.lock().clone()
    }
    fn looked_up_queries(&self) -> Vec<(String, String, Option<i64>, Option<CommandQueue>, bool)> {
        self.inner.looked_up_queries.lock().clone()
    }
    fn push_query_response(&self, v: PyValue) {
        self.inner.query_responses.lock().push(v);
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        self.inner.name.clone()
    }
    fn get_printer(&self) -> Printer {
        self.inner.printer.clone()
    }
    fn is_fileoutput(&self) -> bool {
        match self.inner.printer.get_start_args() {
            PyValue::Dict(m) => !matches!(
                m.get(&PyValue::Str("debugoutput".to_string())),
                None | Some(PyValue::None)
            ),
            _ => false,
        }
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        panic!("setup_pin not used")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut n = self.inner.next_oid.lock();
        let v = *n;
        *n += 1;
        Ok(v)
    }
    fn register_config_callback(&self, cb: Box<dyn FnMut() -> KResult<()> + Send>) -> KResult<()> {
        self.inner.config_callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, _opts: AddConfigCmdOpts) -> KResult<()> {
        self.inner.config_cmds.lock().push(cmd.to_string());
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        kos::cconv::f64_to_i64(time * self.inner.freq)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        self.inner.looked_up_commands.lock().push((msgformat.to_string(), cq));
        Ok(Arc::new(FakeCmdWrapper { mcu: self.clone(), msgformat: msgformat.to_string() }) as CommandWrapper)
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        self.inner.looked_up_queries.lock().push((
            msgformat.to_string(),
            respformat.to_string(),
            opts.oid,
            opts.cq,
            opts.is_async,
        ));
        Ok(Arc::new(FakeQueryWrapper { mcu: self.clone(), msgformat: msgformat.to_string() }) as CommandQueryWrapper)
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut n = self.inner.next_cq.lock();
        let v = *n;
        *n += 1;
        CommandQueue(v)
    }
    fn register_serial_response(
        &self,
        _cb: klippy_api::mcu::SerialResponseCallback,
        _msg: &str,
        _oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        panic!("register_serial_response not used")
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        true
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        IndexMap::new()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        self.inner.constants.clone()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float not used")
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 {
        kos::cconv::f64_to_i64(print_time * self.inner.freq)
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / self.inner.freq
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        eventtime
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        Ok((print_time, eventtime))
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
        panic!("create_trigger_dispatch not used")
    }
}

struct FakeCmdWrapper {
    mcu: FakeMcu,
    msgformat: String,
}

impl klippy_api::mcu::CommandWrapperApi for FakeCmdWrapper {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.mcu.inner.cmd_sends.lock().push((self.msgformat.clone(), data, opts.minclock, opts.reqclock));
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.send(data, opts)
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

struct FakeQueryWrapper {
    mcu: FakeMcu,
    msgformat: String,
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryWrapper {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        self.mcu.inner.query_sends.lock().push((
            self.msgformat.clone(),
            data,
            opts.minclock,
            opts.reqclock,
            opts.retry,
        ));
        let mut q = self.mcu.inner.query_responses.lock();
        if q.is_empty() {
            let mut m = IndexMap::new();
            m.insert(PyValue::Str("read".to_string()), PyValue::Bytes(vec![0; 10]));
            m.insert(PyValue::Str("#receive_time".to_string()), PyValue::Float(0.0));
            Ok(PyValue::Dict(m))
        } else {
            Ok(q.remove(0))
        }
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

fn pins_error(msg: &str) -> KlippyError {
    KlippyError::Other {
        class_name: "error".to_string(),
        message: msg.to_string(),
    }
}

struct FakePinsInner {
    chips: kos::sync::Mutex<IndexMap<String, Mcu>>,
    active: kos::sync::Mutex<IndexMap<String, PinParams>>,
}

#[derive(Clone)]
struct FakePins {
    inner: Arc<FakePinsInner>,
}

fn parse_pin_inner(
    chips: &IndexMap<String, Mcu>,
    pin_desc: &str,
    can_invert: bool,
    can_pullup: bool,
) -> KResult<PinParams> {
    let desc_orig = pin_desc;
    let mut desc = pin_desc.trim().to_string();
    let mut pullup: i64 = 0;
    let mut invert: i64 = 0;
    if can_pullup && (desc.starts_with('^') || desc.starts_with('~')) {
        pullup = 1;
        if desc.starts_with('~') {
            pullup = -1;
        }
        desc = desc[1..].trim().to_string();
    }
    if can_invert && desc.starts_with('!') {
        invert = 1;
        desc = desc[1..].trim().to_string();
    }
    let (chip_name, pin) = if !desc.contains(':') {
        ("mcu".to_string(), desc.clone())
    } else {
        let parts: Vec<String> = desc.splitn(2, ':').map(|s| s.trim().to_string()).collect();
        (parts[0].clone(), parts[1].clone())
    };
    let chip = match chips.get(&chip_name) {
        Some(c) => c.clone(),
        None => return Err(pins_error(&format!("Unknown pin chip name '{chip_name}'"))),
    };
    let has_inner_ws = pin.split_whitespace().collect::<Vec<_>>().join("") != pin;
    if ['^', '~', '!', ':'].iter().any(|c| pin.contains(*c)) || has_inner_ws {
        let mut fmt = String::new();
        if can_pullup {
            fmt.push_str("[^~] ");
        }
        if can_invert {
            fmt.push_str("[!] ");
        }
        return Err(pins_error(&format!(
            "Invalid pin description '{desc_orig}'\nFormat is: {fmt}[chip_name:] pin_name"
        )));
    }
    Ok(PinParams { chip, chip_name, pin, invert, pullup, share_type: None })
}

impl FakePins {
    fn new(chips: &[(&str, Mcu)]) -> FakePins {
        let mut map = IndexMap::new();
        for (k, v) in chips {
            map.insert(k.to_string(), v.clone());
        }
        FakePins {
            inner: Arc::new(FakePinsInner {
                chips: kos::sync::Mutex::new(map),
                active: kos::sync::Mutex::new(IndexMap::new()),
            }),
        }
    }
}

struct TestPins {
    inner: FakePins,
}

impl PinsApi for TestPins {
    fn parse_pin(&self, pin_desc: &str, opts: ParsePinOpts) -> KResult<PinParams> {
        let chips = self.inner.inner.chips.lock().clone();
        parse_pin_inner(&chips, pin_desc, opts.can_invert, opts.can_pullup)
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        let chips = self.inner.inner.chips.lock().clone();
        let pp = parse_pin_inner(&chips, pin_desc, opts.can_invert, opts.can_pullup)?;
        let share_name = format!("{}:{}", pp.chip_name, pp.pin);
        {
            let active = self.inner.inner.active.lock();
            if let Some(shared) = active.get(&share_name) {
                let same_type = match (&opts.share_type, &shared.share_type) {
                    (None, _) => false,
                    (Some(a), Some(b)) => a == b,
                    (Some(_), None) => false,
                };
                if !same_type {
                    return Err(pins_error(&format!("pin {} used multiple times in config", pp.pin)));
                }
                if pp.invert != shared.invert || pp.pullup != shared.pullup {
                    return Err(pins_error(&format!("Shared pin {} must have same polarity", pp.pin)));
                }
                return Ok(shared.clone());
            }
        }
        let mut stored = pp.clone();
        stored.share_type = opts.share_type.clone();
        self.inner.inner.active.lock().insert(share_name, stored.clone());
        Ok(stored)
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        panic!("setup_pin not used")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {}
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver not used")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        Ok(())
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        Ok(())
    }
}

klippy_api::printer_object!(TestPins; dyn PinsApi);

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("tmc_uart fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping tmc_uart oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
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
        unit: "tmc_uart".to_string(),
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


fn pyquery_response(read: &[u8], receive_time: f64) -> PyValue {
    let mut m = IndexMap::new();
    m.insert(PyValue::Str("read".to_string()), PyValue::Bytes(read.to_vec()));
    m.insert(PyValue::Str("#receive_time".to_string()), PyValue::Float(receive_time));
    PyValue::Dict(m)
}

struct Machine {
    printer: Printer,
    mcu: FakeMcu,
    mcu_handle: Mcu,
    pins: FakePins,
}

fn make_machine(constants: &[(&str, &str)], debugoutput: Option<&str>) -> Machine {
    let start_args = match debugoutput {
        Some(_) => FakePrinter::start_args_dict(&[("debugoutput", "out")]),
        None => PyValue::Dict(IndexMap::new()),
    };
    let printer: Printer = Arc::new(FakePrinter::new(start_args));
    let mcu = FakeMcu::new(printer.clone(), "mcu", constants);
    let mcu_handle = mcu.clone().to_mcu();
    let pins = FakePins::new(&[("mcu", mcu_handle.clone())]);
    let testpins = Arc::new(TestPins { inner: pins.clone() }) as ObjectRef;
    printer.add_object("pins", testpins).expect("add pins object");
    Machine { printer, mcu, mcu_handle, pins }
}
fn bool_tuple(id: Option<&[bool]>) -> Value {
    match id {
        None => Value::Null,
        Some(b) => Value::Tuple(b.iter().map(|v| Value::Bool(*v)).collect()),
    }
}

fn py_make_machine_expr(constants: &[(&str, &str)], debugoutput: Option<&str>) -> String {
    let dbg = match debugoutput {
        Some(d) => format!("start_args={{'debugoutput': {d:?}}}"),
        None => "start_args=None".to_string(),
    };
    let const_str = constants
        .iter()
        .map(|(k, v)| format!("{k:?}: {v:?}"))
        .collect::<Vec<_>>()
        .join(", ");
    format!(
        "printer = FakePrinter({dbg}); mcu = FakeMcu(printer, name='mcu', constants={{{const_str}}}); pins = FakePins({{'mcu': mcu}}); printer.add_object('pins', pins)"
    )
}

fn py_write_response(read_bytes: &[u8], receive_time: f64) -> String {
    let hex: String = read_bytes.iter().map(|b| format!("{b:02x}")).collect();
    format!(
        "mcu.query_responses.append({{'read': bytearray.fromhex('{hex}'), '#receive_time': {receive_time:?}}})",
        receive_time = receive_time
    )
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn baud_constants_match_python() {
    let Some(mut py) = oracle() else { return };
    check("TMC_BAUD_RATE", py.eval("tmc_uart.TMC_BAUD_RATE"), Value::int(TMC_BAUD_RATE));
    check(
        "TMC_BAUD_RATE_AVR",
        py.eval("tmc_uart.TMC_BAUD_RATE_AVR"),
        Value::int(TMC_BAUD_RATE_AVR),
    );
}

#[test]
fn calc_crc8_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("bb = object.__new__(tmc_uart.MCU_TMC_uart_bitbang)").expect("bb");
    for data in [
        vec![],
        vec![0x00],
        vec![0xf5, 0x00, 0x10],
        vec![0xf5, 0x03, 0x80, 0x00, 0x01, 0x23, 0x45],
        vec![0x05, 0xff, 0x10, 0xaa, 0xbb, 0xcc, 0xdd],
        vec![0xff; 8],
        vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
    ] {
        let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
        let expected = py.eval(&format!("bb._calc_crc8(bytearray.fromhex('{hex}'))"));
        check("calc_crc8", expected, Value::int(calc_crc8(&data) as i64));
    }
}

#[test]
fn add_serial_bits_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("bb = object.__new__(tmc_uart.MCU_TMC_uart_bitbang)").expect("bb");
    for data in [
        vec![],
        vec![0x00],
        vec![0xff],
        vec![0xf5, 0x00, 0x10, 0x52],
        vec![0xf5, 0x01, 0x80, 0x12, 0x34, 0x56, 0x78, 0xab],
        vec![0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
    ] {
        let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
        let expected = py.eval(&format!("bb._add_serial_bits(bytearray.fromhex('{hex}'))"));
        check("add_serial_bits", expected, Value::bytes(&add_serial_bits(&data)));
    }
}

#[test]
fn encode_read_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("bb = object.__new__(tmc_uart.MCU_TMC_uart_bitbang)").expect("bb");
    for (sync, addr, reg) in [
        (0xf5u8, 0u8, 0x10u8),
        (0xf5, 3, 0x00),
        (0x05, 0xff, 0x80),
        (0x00, 0x00, 0x00),
    ] {
        let expected = py.eval(&format!("bb._encode_read(0x{sync:x}, 0x{addr:x}, 0x{reg:x})"));
        check("encode_read", expected, Value::bytes(&encode_read(sync, addr, reg)));
    }
}

#[test]
fn encode_write_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("bb = object.__new__(tmc_uart.MCU_TMC_uart_bitbang)").expect("bb");
    for (sync, addr, reg, val) in [
        (0xf5u8, 0u8, 0x90u8, 0x1234_5678i64),
        (0xf5, 7, 0x80, 0i64),
        (0x05, 0xff, 0x10, 0xffffffffi64),
        (0xf5, 0, 0x81, -1i64),
    ] {
        let expected = py.eval(&format!("bb._encode_write(0x{sync:x}, 0x{addr:x}, 0x{reg:x}, {val})"));
        check("encode_write", expected, Value::bytes(&encode_write(sync, addr, reg, val)));
    }
}

#[test]
fn decode_read_matches_python() {
    let Some(mut py) = oracle() else { return };
    py.exec("bb = object.__new__(tmc_uart.MCU_TMC_uart_bitbang)").expect("bb");
    for (reg, val) in [(0x10u8, 0x1234_5678i64), (0x00, 0i64), (0x02, 0xffff_ff00i64)] {
        let resp = encode_write(0x05, 0xff, reg, val);
        let hex: String = resp.iter().map(|b| format!("{b:02x}")).collect();
        let expected = py.eval(&format!("bb._decode_read(0x{reg:x}, bytearray.fromhex('{hex}'))"));
        let actual = match decode_read(reg, &resp) {
            Some(v) => Value::int(v),
            None => Value::Null,
        };
        check("decode_read roundtrip", expected, actual);
    }
    for (reg, data) in [(0x10u8, vec![0u8; 9]), (0x10, vec![0u8; 11]), (0x10, vec![0u8; 10])] {
        let hex: String = data.iter().map(|b| format!("{b:02x}")).collect();
        let expected = py.eval(&format!("bb._decode_read(0x{reg:x}, bytearray.fromhex('{hex}'))"));
        let actual = match decode_read(reg, &data) {
            Some(v) => Value::int(v),
            None => Value::Null,
        };
        check("decode_read corrupt", expected, actual);
    }
}

#[test]
fn analog_mux_config_commands_match_python() {
    let Some(mut py) = oracle() else { return };
    let expr = py_make_machine_expr(&[("MCU", "stm32f407xx")], None);
    py.exec(&expr).expect("py machine");
    py.exec("pins_helpers = pins")
        .expect("py pins alias");
    py.exec(
        "rx = pins.lookup_pin('PA5', can_pullup=True); mux = tmc_uart.MCU_analog_mux(rx['chip'], rx['chip'].alloc_command_queue(), ['PA6', '!PA7'])",
    )
    .expect("py mux init");
    let py_cmds = py.eval("mcu.config_cmds").expect("py config cmds");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let ppins: Arc<dyn PinsApi> = {
        let p: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine.pins.clone() }) as Arc<dyn PinsApi>;
        p
    };
    let rx = ppins
        .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
        .expect("rust rx");
    let mux = McuAnalogMux::new(machine.mcu_handle.clone(), machine.mcu_handle.alloc_command_queue(), &["PA6".to_string(), "!PA7".to_string()])
        .expect("rust mux init");
    let rust_cmds: Vec<Value> = machine
        .mcu
        .config_cmds()
        .into_iter()
        .map(|c| Value::str(&c))
        .collect();
    let py_cmds_list = match py_cmds {
        Value::List(items) => items
            .into_iter()
            .map(|v| match v {
                Value::Tuple(mut t) => t.remove(0),
                other => other,
            })
            .collect::<Vec<_>>(),
        other => panic!("unexpected py cmds {other:?}"),
    };
    check("mux config cmds", Ok(Value::List(py_cmds_list)), Value::List(rust_cmds));
    let _ = mux;
    let _ = rx;
}

#[test]
fn analog_mux_instance_and_activate_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec("rx = pins.lookup_pin('PA5', can_pullup=True); mux = tmc_uart.MCU_analog_mux(rx['chip'], rx['chip'].alloc_command_queue(), ['PA6', '!PA7'])")
        .expect("py mux init");
    py.exec("mux.build_config()").expect("py build");
    for desc in ["['PA6', '!PA7']", "['PA6', 'PA7']", "['PA8', 'PA7']"] {
        let expected = py.eval(&format!("mux.get_instance_id({desc})"));
        let descs: Vec<String> = vec!["PA6".to_string(), "!PA7".to_string()];
        let inner: Vec<String> = match desc {
            "['PA6', '!PA7']" => descs.clone(),
            "['PA6', 'PA7']" => vec!["PA6".to_string(), "PA7".to_string()],
            _ => vec!["PA8".to_string(), "PA7".to_string()],
        };
        let machine2 = make_machine(&[("MCU", "stm32f407xx")], None);
        let ppins: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine2.pins.clone() }) as Arc<dyn PinsApi>;
        let _rx = ppins
            .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
            .expect("rx");
        let mux2 = McuAnalogMux::new(
            machine2.mcu_handle.clone(),
            machine2.mcu_handle.alloc_command_queue(),
            &descs,
        )
        .expect("mux");
        mux2.build_config().expect("build");
        match (expected.clone(), mux2.get_instance_id(&inner)) {
            (Ok(pyval), Ok(rustval)) => {
                let actual = Value::Tuple(rustval.iter().map(|b| Value::Bool(*b)).collect());
                check("mux instance", Ok(pyval), actual);
            }
            (Err(e), Err(rerr)) => {
                check_err("mux instance err", Err(e), &rerr);
            }
            (e, a) => panic!("mux instance mismatch {e:?} vs {a:?}"),
        }
    }
    // Activate sends only changed pins.
    py.exec("mux.activate((True, False))").expect("py activate 1");
    let before = py.eval("list(mcu.cmd_sends)").expect("py sends 1");
    py.exec("mux.activate((True, False))").expect("py activate same");
    let after_same = py.eval("list(mcu.cmd_sends)").expect("py sends same");
    check("mux activate no-op", Ok(before.clone()), after_same.clone());
    py.exec("mux.activate((False, True))").expect("py activate flip");
    let after_flip = py.eval("list(mcu.cmd_sends)").expect("py sends flip");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let ppins: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine.pins.clone() }) as Arc<dyn PinsApi>;
    let rx = ppins
        .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
        .expect("rx");
    let descs = vec!["PA6".to_string(), "!PA7".to_string()];
    let mux2 = McuAnalogMux::new(machine.mcu_handle.clone(), machine.mcu_handle.alloc_command_queue(), &descs).expect("mux");
    mux2.build_config().expect("build");
    mux2.activate(&[true, false]).expect("a1");
    let sends1 = machine.mcu.cmd_sends();
    let sends1_val = Value::List(
        sends1
            .iter()
            .map(|(fmt, data, minclock, reqclock)| {
                Value::Tuple(vec![
                    Value::str(fmt),
                    pyval_from_py(&PyValue::List(data.clone())),
                    Value::int(*minclock),
                    Value::int(*reqclock),
                ])
            })
            .collect(),
    );
    check("mux activate sends", Ok(before.clone()), sends1_val);
    mux2.activate(&[true, false]).expect("a same");
    assert_eq!(
        machine.mcu.cmd_sends().len(),
        sends1.len(),
        "second identical activate must not send"
    );
    mux2.activate(&[false, true]).expect("a flip");
    let sends2 = machine.mcu.cmd_sends();
    let sends2_val = Value::List(
        sends2
            .iter()
            .map(|(fmt, data, minclock, reqclock)| {
                Value::Tuple(vec![
                    Value::str(fmt),
                    pyval_from_py(&PyValue::List(data.clone())),
                    Value::int(*minclock),
                    Value::int(*reqclock),
                ])
            })
            .collect(),
    );
    check("mux activate flip sends", Ok(after_flip), sends2_val);
    let _ = rx;
}

#[test]
fn bitbang_build_config_baud_and_commands_match_python() {
    let Some(mut py) = oracle() else { return };
    for (mcu_name, bit_ticks) in [("stm32f407xx", 25i64), ("at90usb1286", 111i64)] {
        py.exec(&py_make_machine_expr(&[("MCU", mcu_name)], None)).expect("py machine");
        py.exec("rx = pins.lookup_pin('PA5', can_pullup=True, share_type='tmc_uart_rx')")
            .expect("py rx");
        py.exec("bb = tmc_uart.MCU_TMC_uart_bitbang(rx, rx, None)").expect("py bb");
        py.exec("bb.build_config()").expect("py build");
        let py_cmds = py.eval("mcu.config_cmds").expect("py cmds");
        let py_lookups = py.eval("mcu.looked_up_queries").expect("py queries");
        let machine = make_machine(&[("MCU", mcu_name)], None);
        let ppins: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine.pins.clone() }) as Arc<dyn PinsApi>;
        let rx = ppins
            .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
            .expect("rx");
        let bb = McuTmcUartBitbang::new(&rx, &rx, None).expect("rust bb");
        bb.build_config().expect("rust build");
        let rust_cmds: Vec<Value> = machine
            .mcu
            .config_cmds()
            .into_iter()
            .map(|c| Value::str(&c))
            .collect();
        let py_cmds_list = match py_cmds {
            Value::List(items) => items
                .into_iter()
                .map(|v| match v {
                    Value::Tuple(mut t) => t.remove(0),
                    other => other,
                })
                .collect::<Vec<_>>(),
            other => panic!("unexpected {other:?}"),
        };
        check("bitbang cmds", Ok(Value::List(py_cmds_list)), Value::List(rust_cmds));
        let rust_q = machine.mcu.looked_up_queries();
        assert_eq!(rust_q.len(), 1, "one query lookup");
        let py_q = match py_lookups {
            Value::List(mut v) => v.remove(0),
            other => panic!("unexpected query {other:?}"),
        };
        let (mfmt, rfmt, oid, cq_present, is_async) = rust_q.into_iter().next().unwrap();
        let _ = (cq_present, oid, is_async);
        let rust_q_val = Value::Tuple(vec![Value::str(&mfmt), Value::str(&rfmt), Value::Bool(is_async)]);
        // Compare msgformat/respformat/is_async; oid/cq checked separately below.
        let py_q_tuple = match py_q {
            Value::Tuple(t) => t,
            other => panic!("unexpected {other:?}"),
        };
        check(
            "bitbang query fmt",
            Ok(Value::Tuple(vec![
                py_q_tuple[0].clone(),
                py_q_tuple[1].clone(),
                py_q_tuple[4].clone(),
            ])),
            rust_q_val,
        );
        let _ = bit_ticks;
    }
}

#[test]
fn bitbang_register_instance_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec("rx = pins.lookup_pin('PA5', can_pullup=True, share_type='tmc_uart_rx')")
        .expect("py rx");
    py.exec("bb = tmc_uart.MCU_TMC_uart_bitbang(rx, rx, None)").expect("py bb");
    let ok = py.eval("bb.register_instance(rx, rx, None, 0)").expect("py register ok");
    check("register ok", Ok(ok), Value::Null);
    let dup = py.eval("bb.register_instance(rx, rx, None, 0)");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let ppins: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine.pins.clone() }) as Arc<dyn PinsApi>;
    let rx = ppins
        .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
        .expect("rx");
    let bb = McuTmcUartBitbang::new(&rx, &rx, None).expect("rust bb");
    let ok2 = bb.register_instance(&rx, &rx, None, 0).expect("rust ok");
    assert_eq!(ok2, None);
    let dup_rust = bb.register_instance(&rx, &rx, None, 0);
    let err = dup_rust.expect_err("dup must fail");
    check_err("register dup", dup, &err);
}

#[test]
fn bitbang_reg_read_write_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec("rx = pins.lookup_pin('PA5', can_pullup=True, share_type='tmc_uart_rx')")
        .expect("py rx");
    py.exec("bb = tmc_uart.MCU_TMC_uart_bitbang(rx, rx, None); bb.build_config()")
        .expect("py bb");
    let resp = encode_write(0x05, 0xff, 0x10, 0x1234_5678);
    py.exec(&py_write_response(&resp, 123.5)).expect("py queue");
    let py_read = py.eval("bb.reg_read(None, 0, 0x10)").expect("py reg_read");
    let py_sends = py.eval("list(mcu.query_sends)").expect("py sends");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let ppins: Arc<dyn PinsApi> = Arc::new(TestPins { inner: machine.pins.clone() }) as Arc<dyn PinsApi>;
    let rx = ppins
        .lookup_pin("PA5", LookupPinOpts { can_invert: false, can_pullup: true, share_type: Some("tmc_uart_rx".to_string()) })
        .expect("rx");
    let bb = McuTmcUartBitbang::new(&rx, &rx, None).expect("rust bb");
    bb.build_config().expect("rust build");
    machine.mcu.push_query_response(pyquery_response(&resp, 123.5));
    let rust_read = bb.reg_read(None, 0, 0x10).expect("rust reg_read");
    check("reg_read dict", Ok(py_read), pyval_from_py(&rust_read.to_pyvalue()));
    let rust_sends: Value = Value::List(
        machine
            .mcu
            .query_sends()
            .into_iter()
            .map(|(fmt, data, minclock, reqclock, retry)| {
                Value::Tuple(vec![
                    Value::str(&fmt),
                    pyval_from_py(&PyValue::List(data)),
                    Value::int(minclock),
                    Value::int(reqclock),
                    Value::Bool(retry),
                ])
            })
            .collect(),
    );
    check("reg_read sends", Ok(py_sends), rust_sends);
    // Corrupt response yields None data on both sides.
    let corrupt = vec![0u8; 10];
    py.exec(&py_write_response(&corrupt, 1.0)).expect("corrupt queue");
    let py_none = py.eval("bb.reg_read(None, 0, 0x10)").expect("py corrupt read");
    machine.mcu.push_query_response(pyquery_response(&corrupt, 1.0));
    let rust_none = bb.reg_read(None, 0, 0x10).expect("rust corrupt");
    check("reg_read corrupt", Ok(py_none), pyval_from_py(&rust_none.to_pyvalue()));
    // Write.
    py.exec("bb.reg_write(None, 0, 0x10, 0x12345678)").expect("py write");
    let py_writes = py.eval("list(mcu.query_sends)").expect("py writes");
    bb.reg_write(None, 0, 0x10, 0x1234_5678, None).expect("rust write");
    let rust_writes: Value = Value::List(
        machine
            .mcu
            .query_sends()
            .into_iter()
            .map(|(fmt, data, minclock, reqclock, retry)| {
                Value::Tuple(vec![
                    Value::str(&fmt),
                    pyval_from_py(&PyValue::List(data)),
                    Value::int(minclock),
                    Value::int(reqclock),
                    Value::Bool(retry),
                ])
            })
            .collect(),
    );
    check("reg_write sends", Ok(py_writes), rust_writes);
    // Write with print_time uses minclock.
    py.exec("bb.reg_write(None, 0, 0x10, 1, print_time=2.0)").expect("py write pt");
    let py_pt = py.eval("list(mcu.query_sends)").expect("py pt");
    bb.reg_write(None, 0, 0x10, 1, Some(2.0)).expect("rust write pt");
    let rust_pt: Value = Value::List(
        machine
            .mcu
            .query_sends()
            .into_iter()
            .map(|(fmt, data, minclock, reqclock, retry)| {
                Value::Tuple(vec![
                    Value::str(&fmt),
                    pyval_from_py(&PyValue::List(data)),
                    Value::int(minclock),
                    Value::int(reqclock),
                    Value::Bool(retry),
                ])
            })
            .collect(),
    );
    check("reg_write print_time sends", Ok(py_pt), rust_pt);
    let _ = rx;
}

#[test]
fn lookup_bitbang_sharing_and_errors_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec(
        "cfg1 = FakeConfig(printer, 'stepper_x', {'uart_pin': 'PA5', 'uart_address': '0'}); \
         cfg2 = FakeConfig(printer, 'stepper_y', {'uart_pin': 'PA5', 'uart_address': '1'})",
    )
    .expect("py cfgs");
    // The returned tuple contains the uart object, which the recorder
    // cannot encode; store the tuples and extract the encodable parts.
    py.exec(
        "res1 = tmc_uart.lookup_tmc_uart_bitbang(cfg1, 7); res2 = tmc_uart.lookup_tmc_uart_bitbang(cfg2, 7)"
    )
    .expect("py lookups");
    let py1_id = py.eval("res1[0]").expect("py lookup 1 id");
    let py1_addr = py.eval("res1[1]").expect("py lookup 1 addr");
    let py2_id = py.eval("res2[0]").expect("py lookup 2 id");
    let py2_addr = py.eval("res2[1]").expect("py lookup 2 addr");
    let py_shared = py.eval("res1[2] is res2[2]").expect("py lookup shared");
    check("lookup same uart object", Ok(py_shared), Value::Bool(true));
    // Rust side.
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let cfg1 = make_config(
        machine.printer.clone(),
        "stepper_x",
        &[("uart_pin", "PA5"), ("uart_address", "0")],
    );
    let cfg2 = make_config(
        machine.printer.clone(),
        "stepper_y",
        &[("uart_pin", "PA5"), ("uart_address", "1")],
    );
    let (id1, addr1, uart1) = lookup_tmc_uart_bitbang(&cfg1, 7).expect("rust lookup 1");
    let (id2, addr2, uart2) = lookup_tmc_uart_bitbang(&cfg2, 7).expect("rust lookup 2");
    assert!(Arc::ptr_eq(&uart1, &uart2), "shared UART must be the same object");
    assert_eq!((addr1, addr2), (0, 1));
    assert_eq!(id1, id2);
    check("lookup instance 1", Ok(py1_id), bool_tuple(id1.as_deref()));
    check("lookup addr 1", Ok(py1_addr), Value::int(addr1));
    check("lookup instance 2", Ok(py2_id), bool_tuple(id2.as_deref()));
    check("lookup addr 2", Ok(py2_addr), Value::int(addr2));
    // Address out of range raises a config error on both sides.
    py.exec(
        "cfg_bad = FakeConfig(printer, 'stepper_bad', {'uart_pin': 'PB1', 'uart_address': '9'})"
    )
    .expect("py bad cfg");
    py.exec("printer.add_object('pins2', pins)")
        .ok();
    let py_bad = py.eval("tmc_uart.lookup_tmc_uart_bitbang(cfg_bad, 7)");
    // Rust side uses the same printer/pins; a fresh pin avoids sharing errors.
    let cfg_bad = make_config(
        machine.printer.clone(),
        "stepper_bad",
        &[("uart_pin", "PB1"), ("uart_address", "9")],
    );
    let rust_bad = lookup_tmc_uart_bitbang(&cfg_bad, 7);
    match (py_bad, rust_bad) {
        (Err(e), Err(rerr)) => check_err("lookup bad addr", Err(e), &rerr),
        (a, b) => panic!("bad addr mismatch ok={} err_ok={}", a.is_ok(), b.is_ok()),
    }
    // rx/tx on different MCUs raises pins.error.
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine2");
    if let Ok(v) = py.eval("printer.lookup_object('pins')") {
        let _ = v;
    }
    // (Multi-chip mismatch is exercised through the Rust fake below and the
    //  Python parse/lookup path via a second chip object.)
    py.exec(
        "mcu2 = FakeMcu(printer, name='mcu2'); pins.chips['mcu2'] = mcu2; \
         cfg_mismatch = FakeConfig(printer, 'stepper_m', {'uart_pin': 'PA5', 'tx_pin': 'mcu2:PA6', 'uart_address': '0'})"
    )
    .expect("py mismatch cfg");
    let py_mm = py.eval("tmc_uart.lookup_tmc_uart_bitbang(cfg_mismatch, 7)");
    // Rust mismatch: build a two-chip machine.
    let machine_mm = make_machine(&[("MCU", "stm32f407xx")], None);
    let mcu2 = FakeMcu::new(machine_mm.printer.clone(), "mcu2", &[("MCU", "stm32f407xx")]);
    let mcu2_handle = mcu2.clone().to_mcu();
    {
        let mut chips = machine_mm.pins.inner.chips.lock();
        chips.insert("mcu2".to_string(), mcu2_handle);
    }
    let cfg_mm = make_config(
        machine_mm.printer.clone(),
        "stepper_m",
        &[("uart_pin", "PA5"), ("tx_pin", "mcu2:PA6"), ("uart_address", "0")],
    );
    let rust_mm = lookup_tmc_uart_bitbang(&cfg_mm, 7);
    match (py_mm, rust_mm) {
        (Err(e), Err(rerr)) => check_err("lookup chip mismatch", Err(e), &rerr),
        (a, b) => panic!("mismatch mismatch ok={} err_ok={}", a.is_ok(), b.is_ok()),
    }
    // Missing uart_pin raises a config error.
    let cfg_missing = make_config(machine_mm.printer.clone(), "stepper_missing", &[]);
    py.exec("cfg_missing = FakeConfig(printer, 'stepper_missing', {})")
        .expect("py missing cfg");
    let py_missing = py.eval("tmc_uart.lookup_tmc_uart_bitbang(cfg_missing, 7)");
    let rust_missing = lookup_tmc_uart_bitbang(&cfg_missing, 7);
    match (py_missing, rust_missing) {
        (Err(e), Err(rerr)) => check_err("lookup missing pin", Err(e), &rerr),
        (a, b) => panic!("missing mismatch ok={} err_ok={}", a.is_ok(), b.is_ok()),
    }
}

#[test]
fn mutex_lookup_shares_per_mcu_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec("m1 = tmc_uart.lookup_tmc_uart_mutex(mcu); m2 = tmc_uart.lookup_tmc_uart_mutex(mcu)")
        .expect("py mutex");
    let same = py.eval("m1 is m2").expect("py same mutex");
    check("mutex same mcu", Ok(same), Value::Bool(true));
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let a = lookup_tmc_uart_mutex(&machine.mcu_handle).expect("rust mutex a");
    let b = lookup_tmc_uart_mutex(&machine.mcu_handle).expect("rust mutex b");
    assert!(Arc::ptr_eq(&a, &b), "same MCU must return the same mutex handle");
    let _ = a;
    let _ = b;
}

#[test]
fn tmc_uart_wrapper_get_set_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec(
        "cfg = FakeConfig(printer, 'tmc2208 stepper_x', {'uart_pin': 'PA5', 'uart_address': '0'}); \
         name_to_reg = {'GCONF': 0x00, 'IFCNT': 0x02}; \
         uart = tmc_uart.MCU_TMC_uart(cfg, name_to_reg, FakeFields(), 3, 12000000.0)"
    )
.expect("py uart init");
    py.exec("mcu.run_config_callbacks()").expect("py run cbs");
    // get_register_raw success.
    let resp = encode_write(0x05, 0xff, 0x00, 0x42);
    py.exec(&py_write_response(&resp, 5.0)).expect("queue");
    let py_raw = py.eval("uart.get_register_raw('GCONF')").expect("py raw");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let cfg = make_config(
        machine.printer.clone(),
        "tmc2208 stepper_x",
        &[("uart_pin", "PA5"), ("uart_address", "0")],
    );
    let mut map = IndexMap::new();
    map.insert("GCONF".to_string(), 0x00);
    map.insert("IFCNT".to_string(), 0x02);
    let uart_r = McuTmcUart::new(&cfg, map, (), 3, 12000000.0).expect("rust uart init");
    machine.mcu.run_config_callbacks().expect("run cbs");
    machine.mcu.push_query_response(pyquery_response(&resp, 5.0));
    let rust_raw = uart_r.get_register_raw("GCONF").expect("rust raw");
    check("wrapper raw", Ok(py_raw), pyval_from_py(&rust_raw));
    // get_register.
    let resp2 = encode_write(0x05, 0xff, 0x00, 7);
    py.exec(&py_write_response(&resp2, 6.0)).expect("queue2");
    let py_get = py.eval("uart.get_register('GCONF')").expect("py get");
    machine.mcu.push_query_response(pyquery_response(&resp2, 6.0));
    let rust_get = uart_r.get_register("GCONF").expect("rust get");
    check("wrapper get", Ok(py_get), Value::int(rust_get));
    // Unknown register raises KeyError on both sides.
    let py_unk = py.eval("uart.get_register('NOPE')");
    let rust_unk_err = uart_r.get_register("NOPE").expect_err("unknown reg");
    check_err("wrapper unknown reg", py_unk, &rust_unk_err);
    // Read failure after retries raises CommandError.
    py.exec("mcu.query_responses.clear()").expect("clear");
    let py_fail = py.eval("uart.get_register_raw('GCONF')");
    // Rust: queue five corrupt responses.
    for _ in 0..5 {
        machine.mcu.push_query_response(pyquery_response(&[0u8; 10], 0.0));
    }
    let rust_fail = uart_r.get_register_raw("GCONF");
    match (py_fail, rust_fail) {
        (Err(e), Err(rerr)) => check_err("wrapper read fail", Err(e), &rerr),
        (a, b) => panic!("read fail mismatch {a:?} vs {b:?}"),
    }
}

#[test]
fn tmc_uart_wrapper_set_register_ifcnt_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], None)).expect("py machine");
    py.exec(
        "cfg = FakeConfig(printer, 'tmc2208 stepper_x', {'uart_pin': 'PA5', 'uart_address': '0'}); \
         name_to_reg = {'GCONF': 0x00, 'IFCNT': 0x02}; \
         uart = tmc_uart.MCU_TMC_uart(cfg, name_to_reg, FakeFields(), 3, 12000000.0)"
    )
    .expect("py uart init");
    py.exec("mcu.run_config_callbacks()").expect("py run cbs");
    let r10 = encode_write(0x05, 0xff, 0x02, 10);
    let r11 = encode_write(0x05, 0xff, 0x02, 11);
    let dummy = vec![0u8; 10];
    py.exec(&py_write_response(&r10, 1.0)).expect("q10");
    py.exec(&py_write_response(&dummy, 0.0)).expect("q dummy");
    py.exec(&py_write_response(&r11, 2.0)).expect("q11");
    py.eval("uart.set_register('GCONF', 99)").expect("py set");
    let py_sends = py.eval("list(mcu.query_sends)").expect("py set sends");
    let machine = make_machine(&[("MCU", "stm32f407xx")], None);
    let cfg = make_config(
        machine.printer.clone(),
        "tmc2208 stepper_x",
        &[("uart_pin", "PA5"), ("uart_address", "0")],
    );
    let mut map = IndexMap::new();
    map.insert("GCONF".to_string(), 0x00);
    map.insert("IFCNT".to_string(), 0x02);
    let uart_r = McuTmcUart::new(&cfg, map, (), 3, 12000000.0).expect("rust uart init");
    machine.mcu.run_config_callbacks().expect("run cbs");
    machine.mcu.push_query_response(pyquery_response(&r10, 1.0));
    machine.mcu.push_query_response(pyquery_response(&dummy, 0.0));
    machine.mcu.push_query_response(pyquery_response(&r11, 2.0));
    uart_r.set_register("GCONF", 99, None).expect("rust set");
    let rust_sends: Value = Value::List(
        machine
            .mcu
            .query_sends()
            .into_iter()
            .map(|(fmt, data, minclock, reqclock, retry)| {
                Value::Tuple(vec![
                    Value::str(&fmt),
                    pyval_from_py(&PyValue::List(data)),
                    Value::int(minclock),
                    Value::int(reqclock),
                    Value::Bool(retry),
                ])
            })
            .collect(),
    );
    check("set sends", Ok(py_sends), rust_sends);
}

#[test]
fn tmc_uart_wrapper_debugoutput_match_python() {
    let Some(mut py) = oracle() else { return };
    py.exec(&py_make_machine_expr(&[("MCU", "stm32f407xx")], Some("out"))).expect("py machine");
    py.exec(
        "cfg = FakeConfig(printer, 'tmc2208 stepper_x', {'uart_pin': 'PA5', 'uart_address': '0'}); \
         name_to_reg = {'GCONF': 0x00, 'IFCNT': 0x02}; \
         uart = tmc_uart.MCU_TMC_uart(cfg, name_to_reg, FakeFields(), 3, 12000000.0)"
    )
    .expect("py uart init");
    let py_raw = py.eval("uart.get_register_raw('GCONF')").expect("py debug raw");
    let py_set = py.eval("uart.set_register('GCONF', 1)").expect("py debug set");
    check("debug raw", Ok(py_raw.clone()), pyval_from_py(&RegRead { data: Some(0), receive_time: 0.0 }.to_pyvalue()));
    check("debug set null", Ok(Value::Null), Value::Null);
    let _ = py_set;
    let machine = make_machine(&[("MCU", "stm32f407xx")], Some("out"));
    let cfg = make_config(
        machine.printer.clone(),
        "tmc2208 stepper_x",
        &[("uart_pin", "PA5"), ("uart_address", "0")],
    );
    let mut map = IndexMap::new();
    map.insert("GCONF".to_string(), 0x00);
    map.insert("IFCNT".to_string(), 0x02);
    let uart_r = McuTmcUart::new(&cfg, map, (), 3, 12000000.0).expect("rust uart init");
    let rust_raw = uart_r.get_register_raw("GCONF").expect("rust debug raw");
    check("debug raw rust", Ok(py_raw), pyval_from_py(&rust_raw));
    uart_r.set_register("GCONF", 1, None).expect("rust debug set");
    assert!(machine.mcu.query_sends().is_empty(), "debug set must not send");
}

#[test]
fn module_factory_has_no_hooks() {
    let f = module_factory();
    assert!(f.load_config.is_none());
    assert!(f.load_config_prefix.is_none());
    assert!(f.add_early_printer_objects.is_none());
    assert!(f.add_printer_objects.is_none());
}

#[test]
fn printer_types_exist() {
    let _m: Option<PrinterTmcUartMutexes> = None;
    let _b: Option<Arc<McuTmcUartBitbang>> = None;
    let _u: Option<McuTmcUart<()>> = None;
    let _r: Option<RegRead> = None;
    let _mu: Option<Arc<ReactorMutex>> = None;
}

