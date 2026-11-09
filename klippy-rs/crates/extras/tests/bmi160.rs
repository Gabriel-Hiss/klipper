//! Parity tests for `extras::bmi160` (port of `klippy/extras/bmi160.py`).
//!
//! Expected values come from `pyoracle`: the real `extras.bmi160` module is
//! imported with the real `extras.bus`, `extras.adxl345` and
//! `extras.bulk_sensor` modules it builds on, driven by a fake printer,
//! config, MCU, reactor, gcode, pins and webhooks object (the same fakes the
//! Rust port runs against). Compared: every MCU config command (with
//! `on_restart` / `is_init`), every command sent (tag, arguments, `minclock`),
//! the pins looked up, the `ACCELEROMETER_*` mux registrations, the webhooks
//! endpoint and the batch header it sends, raised errors (class and message),
//! the batch dict and the `BMI160` log lines.

use extras::bmi160::{
    read_axes_map, Bmi160, BATCH_UPDATES, BMI160_DEV_ID, BMI_I2C_ADDR, CMD_ACC_PM_NORMAL,
    CMD_ACC_PM_SUSPEND, CMD_FIFO_FLUSH, FREEFALL_ACCEL, REG_ACC_CONF, REG_ACC_DATA_START,
    REG_ACC_RANGE, REG_CHIPID, REG_CMD, REG_FIFO_CONFIG_0, REG_FIFO_CONFIG_1, REG_FIFO_DATA,
    REG_FIFO_DOWNS, REG_FIFO_LENGTH_0, REG_MOD_READ, SCALE, SET_ACC_CONF_1600HZ,
    SET_ACC_RANGE_16G, SET_FIFO_CONFIG_1, SET_FIFO_DOWNS,
};
use indexmap::IndexMap;
use klippy_api::config::{Config, GetIntOpts, GetListOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, GCodeCommandHandler, RegisterCommandOpts};
use klippy_api::mcu::{
    AddConfigCmdOpts, CommandQueue, CommandWrapper, CommandWrapperApi, Mcu, McuApi, Pin, PinParams,
    SendOpts,
};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{LookupDefault, Printer, PrinterApi};
use klippy_api::webhooks::{ClientConnection, ClientConnectionApi, WebHooksApi, WebHooksCallback, WebRequest, WebRequestApi};
use pycompat::PyValue;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use kos::sync::Mutex;
use reactor::Reactor;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python setup.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging

class Error(Exception):
    pass

class CommandError(Exception):
    pass

class FakeReactor:
    NEVER = 9999999999999999.0
    def __init__(self):
        self.pauses = []
        self.timers = []
    def pause(self, t):
        self.pauses.append(t)
    def monotonic(self):
        return 100.0
    def register_timer(self, cb, waketime):
        self.timers.append(waketime)
        return len(self.timers)
    def unregister_timer(self, timer):
        pass

FAKE_REACTOR = FakeReactor()

DEFAULT_RESPONSE = {
    'response': b'\x00\x00',
    'i2c_bus_status': 'SUCCESS',
    'clock': 0,
    'next_sequence': 0,
    'buffered': 0,
    'possible_overflows': 0,
    'query_ticks': 0,
}

def _norm(value):
    if isinstance(value, (bytes, bytearray, list)):
        return [int(x) for x in value]
    return value

class FakeCmd:
    def __init__(self, tag):
        self.tag = tag
    def send(self, data, minclock=0, reqclock=0, retry=False):
        FAKE_MCU.sends.append(('send', self.tag, [_norm(x) for x in data], minclock))
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        FAKE_MCU.sends.append(('wait_ack', self.tag, [_norm(x) for x in data], minclock))

class FakeQueryCmd:
    def __init__(self, tag):
        self.tag = tag
    def send(self, data, minclock=0, reqclock=0, retry=False):
        FAKE_MCU.sends.append(('send', self.tag, [_norm(x) for x in data], minclock))
        return FAKE_MCU.pop_response()
    def send_with_preface(self, preface_cmd, preface_data, data,
                          minclock=0, reqclock=0):
        FAKE_MCU.sends.append(('send', self.tag, [_norm(x) for x in data], minclock))
        return FAKE_MCU.pop_response()

class FakeMcu:
    def __init__(self):
        self.oid = 0
        self.commands = []
        self.sends = []
        self.config_cbs = []
        self.serial_cbs = []
        self.script = []
        self.printer = None
    def create_oid(self):
        self.oid += 1
        return self.oid
    def add_config_cmd(self, cmd, on_restart=False, is_init=False):
        self.commands.append((cmd, bool(on_restart), bool(is_init)))
    def register_config_callback(self, cb):
        self.config_cbs.append(cb)
    def seconds_to_clock(self, t):
        return int(t * 4000000)
    def lookup_command(self, fmt, cq=None):
        return FakeCmd(fmt)
    def lookup_query_command(self, fmt, respfmt, oid=None, cq=None):
        return FakeQueryCmd(fmt)
    def try_lookup_command(self, fmt):
        return None
    def alloc_command_queue(self):
        return 'cq'
    def is_fileoutput(self):
        return False
    def get_name(self):
        return 'mcu'
    def get_printer(self):
        return self.printer
    def get_enumerations(self):
        return {}
    def get_constants(self):
        return {}
    def clock32_to_clock64(self, clock32):
        return clock32
    def clock_to_print_time(self, clock):
        return 0.0
    def print_time_to_clock(self, t):
        return 0
    def estimated_print_time(self, eventtime):
        return 0.0
    def register_serial_response(self, cb, msg, oid=None):
        self.serial_cbs.append((cb, msg, oid))
    def pop_response(self):
        if self.script:
            return dict(self.script.pop(0))
        return dict(DEFAULT_RESPONSE)
    def run_config(self):
        for cb in self.config_cbs:
            cb()

class FakePins:
    def __init__(self):
        self.lookups = []
    def error(self, msg):
        return Error(msg)
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False,
                   share_type=None):
        self.lookups.append(pin_desc)
        chip_name, sep, pin = pin_desc.partition(':')
        if not sep:
            chip_name, pin = 'mcu', pin_desc
        return {'chip': FAKE_MCU, 'chip_name': chip_name, 'pin': pin,
                'invert': 0, 'pullup': 0, 'share_type': share_type}
    def reset_pin_sharing(self, pin_params):
        pass
    def get_pin_resolver(self, chip_name):
        raise Error('get_pin_resolver not used')

class FakeGCode:
    def __init__(self):
        self.mux = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        if any(c == cmd and v == value for (c, k, v, d) in self.mux):
            raise Error('duplicate')
        self.mux.append((cmd, key, value, desc))

class FakeToolhead:
    def __init__(self):
        self.last_move_time = 100.0
        self.waits = 0
    def get_last_move_time(self):
        return self.last_move_time
    def wait_moves(self):
        self.waits += 1
    def dwell(self, delay):
        pass

class FakeWebhooks:
    def __init__(self):
        self.endpoints = []
    def register_mux_endpoint(self, path, key, value, callback):
        self.endpoints.append((path, key, value, callback))

class FakeConn:
    def is_closed(self):
        return True
    def send(self, data):
        pass

class FakeRequest:
    def __init__(self):
        self.sent = []
    def get_client_connection(self):
        return FakeConn()
    def get_dict(self, name, default=None):
        return default
    def send(self, data):
        self.sent.append(data)

class FakePrinter:
    command_error = CommandError
    error = Error
    def __init__(self):
        self.objects = {}
        self.handlers = []
        self.shutdown = []
    def get_reactor(self):
        return FAKE_REACTOR
    def lookup_object(self, name, default=None):
        if name in self.objects:
            return self.objects[name]
        if default is None:
            raise Error("Unknown object '%s'" % (name,))
        return default
    def register_event_handler(self, event, cb):
        self.handlers.append((event, cb))
    def invoke_shutdown(self, msg):
        self.shutdown.append(msg)

class FakeConfig:
    error = Error
    def __init__(self, section, opts):
        self.section = section
        self.opts = dict(opts)
    def get_printer(self):
        return FAKE_PRINTER
    def get_name(self):
        return self.section
    def get(self, option, default=None, note_valid=True):
        return self.opts.get(option, default)
    def getint(self, option, default=None, minval=None, maxval=None,
               note_valid=True):
        if option not in self.opts:
            if default is None:
                raise Error("Option '%s' in section '%s' is not a valid integer"
                            % (option, self.section))
            return default
        value = int(self.opts[option])
        if minval is not None and value < minval:
            raise Error("Option '%s' in section '%s' is not a valid integer"
                        % (option, self.section))
        if maxval is not None and value > maxval:
            raise Error("Option '%s' in section '%s' is not a valid integer"
                        % (option, self.section))
        return value
    def getlist(self, option, default=None, count=None):
        if option not in self.opts:
            return default
        value = self.opts[option]
        if len(value.strip()) == 0:
            res = []
        else:
            res = [p.strip() for p in value.split(',')]
        if count is not None and len(res) != count:
            raise Error("Option '%s' in section '%s' must have %d elements"
                        % (option, self.section, count))
        return res

class _Capture(logging.Handler):
    def __init__(self):
        logging.Handler.__init__(self)
        self.lines = []
    def emit(self, record):
        self.lines.append(record.getMessage())

CAPTURE = _Capture()
logging.getLogger().addHandler(CAPTURE)
logging.getLogger().setLevel(logging.INFO)

FAKE_MCU = FakeMcu()
FAKE_PINS = FakePins()
FAKE_GCODE = FakeGCode()
FAKE_TOOLHEAD = FakeToolhead()
FAKE_WEBHOOKS = FakeWebhooks()
FAKE_PRINTER = FakePrinter()

import extras.bmi160 as bmi160_mod

def make_obj(section, opts, script=()):
    FAKE_MCU.oid = 0
    FAKE_MCU.commands = []
    FAKE_MCU.sends = []
    FAKE_MCU.config_cbs = []
    FAKE_MCU.serial_cbs = []
    FAKE_MCU.script = [dict(r) for r in script]
    FAKE_MCU.printer = FAKE_PRINTER
    FAKE_PINS.lookups = []
    FAKE_GCODE.mux = []
    FAKE_WEBHOOKS.endpoints = []
    FAKE_TOOLHEAD.waits = 0
    FAKE_REACTOR.timers = []
    del CAPTURE.lines[:]
    FAKE_PRINTER.objects = {
        'pins': FAKE_PINS,
        'gcode': FAKE_GCODE,
        'toolhead': FAKE_TOOLHEAD,
        'webhooks': FAKE_WEBHOOKS,
        'mcu': FAKE_MCU,
    }
    return bmi160_mod.BMI160(FakeConfig(section, opts))

def snapshot(obj):
    return (list(FAKE_MCU.commands), list(FAKE_MCU.sends),
            list(FAKE_GCODE.mux), list(FAKE_PINS.lookups),
            [(p, k, v) for (p, k, v, cb) in FAKE_WEBHOOKS.endpoints],
            obj.name, obj.bus_type)

def run_config():
    FAKE_MCU.run_config()

def start(obj):
    FAKE_MCU.run_config()
    return obj._start_measurements()

def finish(obj):
    FAKE_MCU.run_config()
    return obj._finish_measurements()

def batch(obj):
    FAKE_MCU.run_config()
    return obj._process_batch(0.0)

def log_lines():
    return [line for line in CAPTURE.lines if line.startswith('BMI160')]
"#;

// ---------------------------------------------------------------------------
// Rust fakes.
// ---------------------------------------------------------------------------

/// One command sent to the MCU, recorded the way the Python fake records it.
#[derive(Clone, Debug)]
struct McuSend {
    kind: &'static str,
    tag: String,
    data: Vec<PyValue>,
    minclock: i64,
}

type Sends = Arc<Mutex<Vec<McuSend>>>;

struct FakePrinter {
    reactor: Reactor,
    objects: Mutex<Vec<(String, ObjectRef)>>,
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_reactor(&self) -> Reactor {
        self.reactor.clone()
    }
    fn get_state_message(&self) -> (String, String) {
        ("Printer is ready".into(), "ready".into())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _o: &str, _n: &str) {}
    fn add_object(&self, name: &str, obj: ObjectRef) -> KResult<()> {
        self.objects.lock().push((name.to_string(), obj));
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        let objs = self.objects.lock();
        for (n, o) in objs.iter() {
            if n == name {
                return Ok(Some(o.clone()));
            }
        }
        match default {
            LookupDefault::Raise => Err(KlippyError::config(format!("Unknown object '{name}'"))),
            LookupDefault::ReturnNone => Ok(None),
        }
    }
    fn lookup_objects(&self, module: Option<&str>) -> Vec<(String, ObjectRef)> {
        let objs = self.objects.lock();
        match module {
            None => objs.clone(),
            Some(m) => objs
                .iter()
                .filter(|(n, _)| n == m || n.starts_with(&format!("{m} ")))
                .cloned()
                .collect(),
        }
    }
    fn load_object(&self, _config: &Config, _section: &str, _default: LookupDefault) -> KResult<Option<ObjectRef>> {
        Ok(None)
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _n: &str, _i: Option<&str>, _l: bool) {}
    fn invoke_shutdown(&self, _m: &str, _d: PyValue) {}
    fn invoke_async_shutdown(&self, _m: &str, _d: PyValue) {}
    fn register_event_handler(&self, _e: &str, _cb: klippy_api::printer::EventCallback) {}
    fn send_event(&self, _e: &str, _p: &[EventArg]) -> KResult<Vec<PyValue>> {
        Ok(Vec::new())
    }
    fn request_exit(&self, _r: &str) {}
}

struct FakeConfig {
    printer: Printer,
    name: String,
    options: Mutex<Vec<(String, String)>>,
}

impl klippy_api::config::ConfigApi for FakeConfig {
    fn get_printer(&self) -> Printer {
        self.printer.clone()
    }
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn get(&self, option: &str, opts: GetStrOpts) -> KResult<Option<String>> {
        let opts_map = self.options.lock();
        for (k, v) in opts_map.iter() {
            if k == option {
                return Ok(Some(v.clone()));
            }
        }
        match opts.default {
            None => Err(KlippyError::config(format!("Option '{option}' not found in section '{}'", self.name))),
            Some(inner) => Ok(inner),
        }
    }
    fn getint(&self, option: &str, opts: GetIntOpts) -> KResult<Option<i64>> {
        let present = self.options.lock().iter().any(|(k, _)| k == option);
        if present {
            match self.get(option, GetStrOpts::default())? {
                Some(v) => Ok(Some(v.parse::<i64>().map_err(|_| {
                    KlippyError::config(format!("Option '{option}' is not a valid integer"))
                })?)),
                None => Ok(None),
            }
        } else {
            match opts.default {
                None => Err(KlippyError::config(format!("Option '{option}' not found in section '{}'", self.name))),
                Some(inner) => Ok(inner),
            }
        }
    }
    fn getfloat(&self, _o: &str, opts: klippy_api::config::GetFloatOpts) -> KResult<Option<f64>> {
        Ok(opts.default.flatten())
    }
    fn getboolean(&self, _o: &str, _opts: klippy_api::config::GetBoolOpts) -> KResult<Option<bool>> {
        Ok(None)
    }
    fn getchoice(&self, _o: &str, _c: &PyValue, _opts: klippy_api::config::GetChoiceOpts) -> KResult<PyValue> {
        Ok(PyValue::None)
    }
    fn getlists(&self, _o: &str, _opts: klippy_api::config::GetListsOpts) -> KResult<PyValue> {
        Ok(PyValue::List(vec![]))
    }
    fn getlist(&self, option: &str, opts: GetListOpts) -> KResult<Option<Vec<String>>> {
        let opts_map = self.options.lock();
        for (k, v) in opts_map.iter() {
            if k == option {
                let parts: Vec<String> = if v.trim().is_empty() {
                    Vec::new()
                } else {
                    v.split(',').map(|s| s.trim().to_string()).collect()
                };
                if let Some(count) = opts.count {
                    if parts.len() != count {
                        return Err(KlippyError::config(format!(
                            "Option '{option}' in section '{}' must have {count} elements",
                            self.name
                        )));
                    }
                }
                return Ok(Some(parts));
            }
        }
        match opts.default {
            None => Err(KlippyError::config(format!("Option '{option}' not found in section '{}'", self.name))),
            Some(inner) => Ok(inner),
        }
    }
    fn getintlist(&self, _o: &str, _opts: klippy_api::config::GetIntListOpts) -> KResult<Option<Vec<i64>>> {
        Ok(None)
    }
    fn getfloatlist(&self, _o: &str, _opts: klippy_api::config::GetFloatListOpts) -> KResult<Option<Vec<f64>>> {
        Ok(None)
    }
    fn getsection(&self, _s: &str) -> Config {
        panic!("getsection not used")
    }
    fn has_section(&self, _s: &str) -> bool {
        false
    }
    fn get_prefix_sections(&self, _p: &str) -> Vec<Config> {
        vec![]
    }
    fn get_prefix_options(&self, _p: &str) -> KResult<Vec<String>> {
        Ok(vec![])
    }
    fn deprecate(&self, _o: &str, _v: Option<PyValue>) -> KResult<()> { Ok(()) }
}

struct FakeCmd {
    sends: Sends,
    tag: String,
}

impl CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.sends.lock().push(McuSend {
            kind: "send",
            tag: self.tag.clone(),
            data,
            minclock: opts.minclock,
        });
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        self.sends.lock().push(McuSend {
            kind: "wait_ack",
            tag: self.tag.clone(),
            data,
            minclock: opts.minclock,
        });
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        0
    }
}

/// The response a scripted MCU returns when the script is exhausted
/// (`DEFAULT_RESPONSE` in the Python setup).
fn default_response() -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("response".into()), PyValue::Bytes(vec![0x00, 0x00]));
    d.insert(PyValue::Str("i2c_bus_status".into()), PyValue::Str("SUCCESS".into()));
    for key in ["clock", "next_sequence", "buffered", "possible_overflows", "query_ticks"] {
        d.insert(PyValue::Str(key.into()), PyValue::Int(0_i64.into()));
    }
    PyValue::Dict(d)
}

struct FakeQueryCmd {
    sends: Sends,
    tag: String,
    script: Arc<Mutex<Vec<PyValue>>>,
}

impl FakeQueryCmd {
    fn record_and_reply(&self, data: Vec<PyValue>, opts: klippy_api::mcu::QuerySendOpts) -> PyValue {
        self.sends.lock().push(McuSend {
            kind: "send",
            tag: self.tag.clone(),
            data,
            minclock: opts.minclock,
        });
        let mut script = self.script.lock();
        if script.is_empty() {
            default_response()
        } else {
            script.remove(0)
        }
    }
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, opts: klippy_api::mcu::QuerySendOpts) -> KResult<PyValue> {
        Ok(self.record_and_reply(data, opts))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: klippy_api::mcu::QuerySendOpts,
    ) -> KResult<PyValue> {
        Ok(self.record_and_reply(data, opts))
    }
}

struct FakeAsyncResponse;

impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsyncResponse {
    fn unregister(&self) {}
}

struct FakeMcu {
    oid_counter: klippy_api::gcell::GCell<i64>,
    printer: Mutex<Option<Printer>>,
    commands: Arc<Mutex<Vec<(String, bool, bool)>>>,
    sends: Sends,
    query_script: Arc<Mutex<Vec<PyValue>>>,
    config_callbacks: Mutex<Vec<klippy_api::mcu::ConfigCallback>>,
    serial_callbacks: Mutex<Vec<(klippy_api::mcu::SerialResponseCallback, String, Option<i64>)>>,
}

impl FakeMcu {
    fn run_config(&self) -> KResult<()> {
        let mut callbacks = self.config_callbacks.lock();
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        Ok(())
    }

    fn feed_bulk_data(&self, params: PyValue) {
        let callbacks = self.serial_callbacks.lock();
        let (cb, _, _) = callbacks.first().expect("bulk data queue registered");
        cb(params);
    }
}

impl McuApi for FakeMcu {
    fn get_name(&self) -> String {
        "mcu".into()
    }
    fn get_printer(&self) -> Printer {
        self.printer
            .lock()
            .clone()
            .expect("rig attaches the fake printer to the mcu")
    }
    fn is_fileoutput(&self) -> bool {
        false
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<Pin> {
        Err(KlippyError::config("setup_pin not used"))
    }
    fn create_oid(&self) -> KResult<i64> {
        let v = self.oid_counter.get();
        self.oid_counter.set(v + 1);
        Ok(v)
    }
    fn register_config_callback(&self, cb: klippy_api::mcu::ConfigCallback) -> KResult<()> {
        self.config_callbacks.lock().push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.commands
            .lock()
            .push((cmd.to_string(), opts.on_restart, opts.is_init));
        Ok(())
    }
    fn request_move_queue_slot(&self) {}
    fn get_query_slot(&self, _oid: i64) -> i64 {
        0
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        kos::cconv::f64_to_i64(time * 4_000_000.0)
    }
    fn min_schedule_time(&self) -> f64 {
        0.0
    }
    fn max_nominal_duration(&self) -> f64 {
        0.0
    }
    fn lookup_command(&self, msgformat: &str, _cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        Ok(Arc::new(FakeCmd { sends: self.sends.clone(), tag: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        _respformat: &str,
        _opts: klippy_api::mcu::LookupQueryCommandOpts,
    ) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        Ok(Arc::new(FakeQueryCmd {
            sends: self.sends.clone(),
            tag: msgformat.to_string(),
            script: self.query_script.clone(),
        }))
    }
    fn try_lookup_command(&self, _msgformat: &str) -> Option<CommandWrapper> {
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        CommandQueue(1)
    }
    fn register_serial_response(
        &self,
        cb: klippy_api::mcu::SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> klippy_api::mcu::AsyncResponseWrapper {
        self.serial_callbacks.lock().push((cb, msg.to_string(), oid));
        Arc::new(FakeAsyncResponse)
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
        Err(KlippyError::config("no constants"))
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        0
    }
    fn clock_to_print_time(&self, _clock: f64) -> f64 {
        0.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        0.0
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _pt: f64, _et: f64) -> KResult<(f64, f64)> {
        Err(KlippyError::config("calibrate_clock not faked"))
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        Err(KlippyError::config("create_trigger_dispatch not used"))
    }
}

klippy_api::printer_object!(FakeMcu; dyn McuApi);

struct FakePins {
    mcu: Mcu,
    lookups: Arc<Mutex<Vec<String>>>,
}

impl FakePins {
    fn parse(&self, pin_desc: &str) -> PinParams {
        self.lookups.lock().push(pin_desc.to_string());
        let (chip_name, pin) = match pin_desc.split_once(':') {
            Some((chip, pin)) => (chip, pin),
            None => ("mcu", pin_desc),
        };
        PinParams {
            chip: self.mcu.clone(),
            chip_name: chip_name.to_string(),
            pin: pin.to_string(),
            invert: 0,
            pullup: 0,
            share_type: None,
        }
    }
}

impl klippy_api::mcu::PinsApi for FakePins {
    fn parse_pin(&self, pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        Ok(self.parse(pin_desc))
    }
    fn lookup_pin(&self, pin_desc: &str, _opts: klippy_api::mcu::LookupPinOpts) -> KResult<PinParams> {
        Ok(self.parse(pin_desc))
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<Pin> {
        Err(KlippyError::config("setup_pin not used"))
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
impl klippy_api::PrinterObject for FakePins {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn klippy_api::mcu::PinsApi);
}

type MuxReg = (String, String, Option<String>, Option<String>);

struct FakeGCode {
    mux: Arc<Mutex<Vec<MuxReg>>>,
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        Ok(None)
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        _func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        let mut m = self.mux.lock();
        if m.iter().any(|(c, _, v, _)| c == cmd && v.as_deref() == value) {
            return Err(KlippyError::config("mux command already registered"));
        }
        m.push((
            cmd.to_string(),
            key.to_string(),
            value.map(str::to_string),
            desc.map(str::to_string),
        ));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
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
    fn get_mutex(&self) -> reactor::ReactorMutex {
        panic!("get_mutex not used")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> klippy_api::gcode::GCmd {
        panic!("create_gcode_command not used")
    }
    fn respond_raw(&self, _msg: &str) {}
    fn respond_info(&self, _msg: &str, _log: bool) {}
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        false
    }
    fn cmd_m112(&self, _gcmd: Option<&klippy_api::gcode::GCmd>) -> KResult<()> {
        Ok(())
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        Ok(())
    }
}
impl klippy_api::PrinterObject for FakeGCode {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn GCodeApi);
}

struct FakeToolhead {
    last_move_time: f64,
    waits: Arc<Mutex<i64>>,
}

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        vec![0.0, 0.0, 0.0]
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        Ok(())
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("get_kinematics not used")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        Ok(())
    }
    fn get_last_move_time(&self) -> KResult<f64> {
        Ok(self.last_move_time)
    }
    fn dwell(&self, _delay: f64) -> KResult<()> { Ok(()) }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        Ok(())
    }
}

impl klippy_api::toolhead::ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        Ok(())
    }
    fn wait_moves(&self) -> KResult<()> {
        *self.waits.lock() += 1;
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq not used")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        (0.0, 0.0)
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        (0.0, 0.0, 0.0, 0.0)
    }
    fn register_lookahead_callback(&self, _callback: klippy_api::toolhead::LookaheadCallback) -> KResult<()> {
        Ok(())
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        (0.0, 0.0, false)
    }
    fn limit_next_junction_speed(&self, _speed: f64) {}
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {}
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder not used")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) -> KResult<()> { Ok(()) }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) -> KResult<()> { Ok(()) }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        Vec::new()
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        (true, String::new())
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory not used")
    }
}

impl klippy_api::PrinterObject for FakeToolhead {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn klippy_api::toolhead::ToolHeadApi);
}

struct Endpoint {
    path: String,
    key: String,
    value: String,
    callback: WebHooksCallback,
}

struct FakeWebHooks {
    endpoints: Arc<Mutex<Vec<Endpoint>>>,
}

impl WebHooksApi for FakeWebHooks {
    fn register_endpoint(&self, path: &str, callback: WebHooksCallback) -> KResult<()> {
        self.endpoints.lock().push(Endpoint {
            path: path.to_string(),
            key: String::new(),
            value: String::new(),
            callback,
        });
        Ok(())
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        self.endpoints.lock().push(Endpoint {
            path: path.to_string(),
            key: key.to_string(),
            value: value.to_string(),
            callback,
        });
        Ok(())
    }
    fn call_remote_method(&self, _method: &str, _params: &IndexMap<String, PyValue>) -> KResult<()> {
        Ok(())
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
}

impl klippy_api::PrinterObject for FakeWebHooks {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn WebHooksApi);
}

struct FakeConn;

impl ClientConnectionApi for FakeConn {
    fn uid(&self) -> u64 {
        0
    }
    fn dump_request_log(&self) {}
    fn set_client_info(&self, _client_info: PyValue, _state_msg: Option<String>) {}
    fn close(&self) {}
    fn is_closed(&self) -> bool {
        true
    }
    fn send(&self, _data: PyValue) {}
}

struct FakeRequest {
    sent: Mutex<Vec<PyValue>>,
}

impl WebRequestApi for FakeRequest {
    fn client_conn(&self) -> ClientConnection {
        Arc::new(FakeConn)
    }
    fn id(&self) -> Option<PyValue> {
        None
    }
    fn method(&self) -> String {
        "bmi160/dump_bmi160".into()
    }
    fn params(&self) -> PyValue {
        PyValue::Dict(IndexMap::new())
    }
    fn response(&self) -> Option<PyValue> {
        None
    }
    fn is_error(&self) -> bool {
        false
    }
    fn get_client_connection(&self) -> ClientConnection {
        Arc::new(FakeConn)
    }
    fn get(&self, _item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        Ok(default.unwrap_or(PyValue::None))
    }
    fn get_str(&self, _item: &str, default: Option<String>) -> KResult<String> {
        Ok(default.unwrap_or_default())
    }
    fn get_int(&self, _item: &str, default: Option<i64>) -> KResult<i64> {
        Ok(default.unwrap_or_default())
    }
    fn get_float(&self, _item: &str, default: Option<f64>) -> KResult<f64> {
        Ok(default.unwrap_or_default())
    }
    fn get_dict(&self, _item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        Ok(default.unwrap_or_else(|| PyValue::Dict(IndexMap::new())))
    }
    fn get_method(&self) -> String {
        "bmi160/dump_bmi160".into()
    }
    fn set_error(&self, _error: KlippyError) {}
    fn send(&self, data: PyValue) -> KResult<()> {
        self.sent.lock().push(data);
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        None
    }
}

// ---------------------------------------------------------------------------
// Rig: the fake printer every fake object is registered on.
// ---------------------------------------------------------------------------

struct Rig {
    printer: Printer,
    mcu: Arc<FakeMcu>,
    commands: Arc<Mutex<Vec<(String, bool, bool)>>>,
    sends: Sends,
    mux: Arc<Mutex<Vec<MuxReg>>>,
    endpoints: Arc<Mutex<Vec<Endpoint>>>,
    pins: Arc<Mutex<Vec<String>>>,
}

/// One scripted MCU query response: `{'response': bytes}` (the i2c bus status
/// is always `SUCCESS`, as the MCU reports it).
fn resp_bytes(bytes: &[u8]) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("response".into()), PyValue::Bytes(bytes.to_vec()));
    d.insert(PyValue::Str("i2c_bus_status".into()), PyValue::Str("SUCCESS".into()));
    PyValue::Dict(d)
}

/// A scripted `sensor_bulk_status` response.
fn resp_status(clock: i64, next_sequence: i64, buffered: i64, overflows: i64, query_ticks: i64) -> PyValue {
    let mut d = IndexMap::new();
    d.insert(PyValue::Str("clock".into()), PyValue::Int(clock.into()));
    d.insert(PyValue::Str("next_sequence".into()), PyValue::Int(next_sequence.into()));
    d.insert(PyValue::Str("buffered".into()), PyValue::Int(buffered.into()));
    d.insert(PyValue::Str("possible_overflows".into()), PyValue::Int(overflows.into()));
    d.insert(PyValue::Str("query_ticks".into()), PyValue::Int(query_ticks.into()));
    PyValue::Dict(d)
}

fn rig(script: Vec<PyValue>) -> Rig {
    let reactor = Reactor::new(true);
    let commands = Arc::new(Mutex::new(Vec::new()));
    let sends: Sends = Arc::new(Mutex::new(Vec::new()));
    let mcu = Arc::new(FakeMcu {
        oid_counter: klippy_api::gcell::GCell::new(1),
        printer: Mutex::new(None),
        commands: commands.clone(),
        sends: sends.clone(),
        query_script: Arc::new(Mutex::new(script)),
        config_callbacks: Mutex::new(Vec::new()),
        serial_callbacks: Mutex::new(Vec::new()),
    });
    let pins = Arc::new(Mutex::new(Vec::new()));
    let fake_pins = FakePins { mcu: mcu.clone() as Mcu, lookups: pins.clone() };
    let mux = Arc::new(Mutex::new(Vec::new()));
    let fake_gcode = FakeGCode { mux: mux.clone() };
    let fake_toolhead = FakeToolhead { last_move_time: 100.0, waits: Arc::new(Mutex::new(0)) };
    let endpoints = Arc::new(Mutex::new(Vec::new()));
    let fake_webhooks = FakeWebHooks { endpoints: endpoints.clone() };
    let mcu_object: ObjectRef = mcu.clone();
    let printer = Arc::new(FakePrinter {
        reactor: reactor.clone(),
        objects: Mutex::new(vec![
            ("pins".into(), Arc::new(fake_pins) as ObjectRef),
            ("gcode".into(), Arc::new(fake_gcode) as ObjectRef),
            ("toolhead".into(), Arc::new(fake_toolhead) as ObjectRef),
            ("webhooks".into(), Arc::new(fake_webhooks) as ObjectRef),
            ("mcu".into(), mcu_object),
        ]),
    }) as Printer;
    *mcu.printer.lock() = Some(printer.clone());
    Rig {
        printer,
        mcu: mcu.clone(),
        commands,
        sends,
        mux,
        endpoints,
        pins,
    }
}

fn load_chip(rig: &Rig, section: &str, opts: Vec<(&str, &str)>) -> KResult<Arc<Bmi160>> {
    let cfg = FakeConfig {
        printer: rig.printer.clone(),
        name: section.to_string(),
        options: Mutex::new(opts.into_iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()),
    };
    let config: Config = Arc::new(cfg);
    Bmi160::new(&config)
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

fn oracle() -> Option<pyoracle::Oracle> {
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("bmi160 fakes");
            Some(py)
        }
        None => {
            println!("skipping bmi160 oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2");
            None
        }
    }
}

/// Runs statements in the shared oracle namespace. `pyeval` only accepts one
/// expression, so statement sequences go through `exec` and the results are
/// read back with a bare expression.
fn py_stmt(py: &mut pyoracle::Oracle, code: &str) {
    if let Err(e) = py.exec(code) {
        panic!("oracle statement `{code}` raised {}: {}", e.class_name, e.message);
    }
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.bmi160".to_string(),
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

fn check_exc(label: &str, expected: pyoracle::OracleResult, actual: &KlippyError) {
    match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?}, rust raised {}: {}", actual.class_name(), actual.message()),
        Err(e) => {
            assert_eq!(actual.class_name(), e.class_name, "{label}: class");
            assert_eq!(actual.message(), e.message, "{label}: message");
        }
    }
}

/// Recording value for a `PyValue` (`recording` encodes Python values the same way).
///
/// Buffers become a list of ints: the Python fake normalizes `bytes`/`list`
/// arguments the same way, because `bmi160.py` hands the bus a list of ints
/// where the port hands it a `&[u8]`.
fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => match n.inner().to_string().parse::<i64>() {
            Ok(v) => Value::Int(v),
            Err(_) => Value::BigInt(n.inner().to_string()),
        },
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::List(b.iter().map(|x| Value::int(*x as i64)).collect()),
        PyValue::List(items) => Value::List(items.iter().map(to_value).collect()),
        PyValue::Tuple(items) => Value::Tuple(items.iter().map(to_value).collect()),
        PyValue::Dict(d) => Value::Dict(d.iter().map(|(k, v)| (to_value(k), to_value(v))).collect()),
        PyValue::Host(_) => Value::Null,
    }
}

fn commands_value(commands: &[(String, bool, bool)]) -> Value {
    Value::List(
        commands
            .iter()
            .map(|(cmd, on_restart, is_init)| {
                Value::Tuple(vec![Value::str(cmd), Value::Bool(*on_restart), Value::Bool(*is_init)])
            })
            .collect(),
    )
}

fn sends_value(sends: &[McuSend]) -> Value {
    Value::List(
        sends
            .iter()
            .map(|s| {
                Value::Tuple(vec![
                    Value::str(s.kind),
                    Value::str(&s.tag),
                    Value::List(s.data.iter().map(to_value).collect()),
                    Value::int(s.minclock),
                ])
            })
            .collect(),
    )
}

fn mux_value(mux: &[MuxReg]) -> Value {
    Value::List(
        mux.iter()
            .map(|(cmd, key, value, desc)| {
                Value::Tuple(vec![
                    Value::str(cmd),
                    Value::str(key),
                    value.as_deref().map(Value::str).unwrap_or(Value::Null),
                    desc.as_deref().map(Value::str).unwrap_or(Value::Null),
                ])
            })
            .collect(),
    )
}

fn pins_value(pins: &[String]) -> Value {
    Value::List(pins.iter().map(|p| Value::str(p)).collect())
}

fn endpoints_value(endpoints: &[Endpoint]) -> Value {
    Value::List(
        endpoints
            .iter()
            .map(|e| {
                Value::Tuple(vec![Value::str(&e.path), Value::str(&e.key), Value::str(&e.value)])
            })
            .collect(),
    )
}

/// The same tuple `snapshot()` builds on the Python side.
fn snapshot(rig: &Rig, chip: &Bmi160) -> Value {
    Value::Tuple(vec![
        commands_value(&rig.commands.lock()),
        sends_value(&rig.sends.lock()),
        mux_value(&rig.mux.lock()),
        pins_value(&rig.pins.lock()),
        endpoints_value(&rig.endpoints.lock()),
        Value::str(&chip.name()),
        Value::str(&chip.bus_type()),
    ])
}

fn sent_values(sent: &[PyValue]) -> Value {
    Value::List(sent.iter().map(to_value).collect())
}

/// Python source for a scripted list of `{'response': bytes([...])}` dicts.
fn py_responses(script: &[Vec<u8>]) -> String {
    let parts: Vec<String> = script
        .iter()
        .map(|bytes| {
            format!(
                "{{'response': bytes([{}]), 'i2c_bus_status': 'SUCCESS'}}",
                bytes.iter().map(|b| b.to_string()).collect::<Vec<_>>().join(", ")
            )
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

fn py_bytes(bytes: &[u8]) -> String {
    format!("bytes([{}])", bytes.iter().map(|b| b.to_string()).collect::<Vec<_>>().join(", "))
}

// ---------------------------------------------------------------------------
// Log capture (`BMI160` is the only module logging in this binary).
// ---------------------------------------------------------------------------

thread_local! {
    /// Lines logged on this test's thread; parallel tests cannot clear or
    /// add to each other's capture.
    static LOG_LINES: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// Installs the process-wide log sink the first time a test asks for it.
fn install_log_sink() {
    static INSTALLED: std::sync::Once = std::sync::Once::new();
    INSTALLED.call_once(|| {
        klippy_api::log::set_sink(Box::new(|record| {
            LOG_LINES.with(|l| l.borrow_mut().push(record.message.to_string()));
        }));
    });
}

/// Empties this thread's capture before a scenario.
fn capture_logs() {
    install_log_sink();
    LOG_LINES.with(|l| l.borrow_mut().clear());
}

fn bmi160_log_lines() -> Vec<String> {
    LOG_LINES.with(|l| l.borrow().iter().filter(|line| line.starts_with("BMI160")).cloned().collect())
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_header_constants_match_oracle() {
    let Some(mut py) = oracle() else { return };
    let names = [
        "REG_CHIPID",
        "REG_ACC_DATA_START",
        "REG_ACC_CONF",
        "REG_ACC_RANGE",
        "REG_FIFO_DOWNS",
        "REG_FIFO_CONFIG_0",
        "REG_FIFO_CONFIG_1",
        "REG_FIFO_DATA",
        "REG_FIFO_LENGTH_0",
        "REG_CMD",
        "REG_MOD_READ",
        "CMD_ACC_PM_SUSPEND",
        "CMD_ACC_PM_NORMAL",
        "CMD_FIFO_FLUSH",
        "BMI160_DEV_ID",
        "SET_ACC_CONF_1600HZ",
        "SET_ACC_RANGE_16G",
        "SET_FIFO_CONFIG_1",
        "SET_FIFO_DOWNS",
        "FREEFALL_ACCEL",
        "SCALE",
        "BATCH_UPDATES",
        "BMI_I2C_ADDR",
    ];
    let expr = format!(
        "({})",
        names.iter().map(|n| format!("bmi160_mod.{n}")).collect::<Vec<_>>().join(", ")
    );
    let expected = py.eval(&expr);
    let actual = Value::Tuple(vec![
        Value::int(REG_CHIPID),
        Value::int(REG_ACC_DATA_START),
        Value::int(REG_ACC_CONF),
        Value::int(REG_ACC_RANGE),
        Value::int(REG_FIFO_DOWNS),
        Value::int(REG_FIFO_CONFIG_0),
        Value::int(REG_FIFO_CONFIG_1),
        Value::int(REG_FIFO_DATA),
        Value::int(REG_FIFO_LENGTH_0),
        Value::int(REG_CMD),
        Value::int(REG_MOD_READ),
        Value::int(CMD_ACC_PM_SUSPEND),
        Value::int(CMD_ACC_PM_NORMAL),
        Value::int(CMD_FIFO_FLUSH),
        Value::int(BMI160_DEV_ID),
        Value::int(SET_ACC_CONF_1600HZ),
        Value::int(SET_ACC_RANGE_16G),
        Value::int(SET_FIFO_CONFIG_1),
        Value::int(SET_FIFO_DOWNS),
        Value::float(FREEFALL_ACCEL),
        Value::float(SCALE),
        Value::float(BATCH_UPDATES),
        Value::int(BMI_I2C_ADDR),
    ]);
    check("module header", expected, actual);
    assert_eq!(
        extras::bmi160::HEADER,
        ["time", "x_acceleration", "y_acceleration", "z_acceleration"]
    );
}

#[test]
fn init_spi_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py_stmt(
        &mut py,
        &format!("obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {})", py_responses(&[])),
    );
    let expected = py.eval("snapshot(obj)");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    check("init spi", expected, snapshot(&rig, &chip));

    // The MCU config callbacks run the bus and the chip build_config.
    py_stmt(&mut py, "run_config()");
    let expected = py.eval("snapshot(obj)");
    rig.mcu.run_config().expect("rust build_config");
    check("init spi build_config", expected, snapshot(&rig, &chip));
}

#[test]
fn init_i2c_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py_stmt(&mut py, &format!("obj = make_obj('bmi160', {{}}, {})", py_responses(&[])));
    let expected = py.eval("snapshot(obj)");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![]).expect("rust load");
    check("init i2c", expected, snapshot(&rig, &chip));

    py_stmt(&mut py, "run_config()");
    let expected = py.eval("snapshot(obj)");
    rig.mcu.run_config().expect("rust build_config");
    check("init i2c build_config", expected, snapshot(&rig, &chip));
}

#[test]
fn start_measurements_spi_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    // One scripted response per `spi_transfer` (writes included: `set_reg`
    // verifies by reading the register back).
    let script = vec![
        vec![0, 0xFF], // dummy read 0x7F
        vec![0, 0xD1], // chip id read
        vec![0, 0x00], // wake write 0x7E
        vec![0, 0x00], // conf write 0x40
        vec![0, 0x2C], // verify read 0x40
        vec![0, 0x00], // range write 0x41
        vec![0, 0x0C], // verify read 0x41
        vec![0, 0x00], // fifo downs write 0x45
        vec![0, 0x00], // verify read 0x45
        vec![0, 0x00], // fifo config write 0x47
        vec![0, 0x40], // verify read 0x47
        vec![0, 0x00], // flush write 0x7E
    ];
    capture_logs();
    py_stmt(
        &mut py,
        &format!(
            "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); start(obj)",
            py_responses(&script)
        ),
    );
    let expected = py.eval("(snapshot(obj), log_lines())");
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    chip.start_measurements().expect("rust start");
    let logs = bmi160_log_lines();
    let actual = Value::Tuple(vec![
        snapshot(&rig, &chip),
        Value::List(logs.iter().map(|l| Value::str(l)).collect()),
    ]);
    check("start spi", expected, actual);
}

#[test]
fn start_measurements_i2c_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let script = vec![
        vec![0xD1], // chip id read
        vec![0x00], // wake write 0x7E
        vec![0x00], // conf write 0x40
        vec![0x2C], // verify read 0x40
        vec![0x00], // range write 0x41
        vec![0x0C], // verify read 0x41
        vec![0x00], // fifo downs write 0x45
        vec![0x00], // verify read 0x45
        vec![0x00], // fifo config write 0x47
        vec![0x40], // verify read 0x47
        vec![0x00], // flush write 0x7E
    ];
    py_stmt(
        &mut py,
        &format!("obj = make_obj('bmi160', {{}}, {}); start(obj)", py_responses(&script)),
    );
    let expected = py.eval("snapshot(obj)");
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    let chip = load_chip(&rig, "bmi160", vec![]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    chip.start_measurements().expect("rust start");
    check("start i2c", expected, snapshot(&rig, &chip));
}

#[test]
fn invalid_chip_id_error_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let script = vec![vec![0, 0xFF], vec![0, 0x00]];
    let expected = py.exec(&format!(
        "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); start(obj)",
        py_responses(&script)
    ));
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let err = chip.start_measurements().expect_err("rust start must fail");
    check_exc("invalid chip id", expected, &err);
}

#[test]
fn read_reg_short_response_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let script = vec![vec![0x00]];
    let expected = py.exec(&format!(
        "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); run_config(); obj.read_reg(0x40)",
        py_responses(&script)
    ));
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let err = chip.read_reg(0x40).expect_err("rust read_reg must fail");
    check_exc("read_reg short response", expected, &err);
}

#[test]
fn set_reg_verify_failure_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let script = vec![vec![0, 0x00]];
    let expected = py.exec(&format!(
        "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); run_config(); obj.set_reg(0x40, 0x2C)",
        py_responses(&script)
    ));
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let err = chip.set_reg(0x40, 0x2C).expect_err("rust set_reg must fail");
    check_exc("set_reg verify", expected, &err);
    assert_eq!(
        err.message(),
        "Failed to set BMI160 register [0x40] to 0x2c: got 0x0. This is generally indicative of \
         connection problems (e.g. faulty wiring) or a faulty bmi160 chip."
    );
}

#[test]
fn set_reg_cmd_register_is_not_verified() {
    let Some(mut py) = oracle() else { return };
    let expected = py.exec(&format!(
        "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); run_config(); obj.set_reg(0x7E, 0x11)",
        py_responses(&[])
    ));
    assert!(expected.is_ok(), "oracle set_reg(REG_CMD) must succeed: {expected:?}");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    chip.set_reg(0x7E, 0x11).expect("rust set_reg(REG_CMD)");
    let expected = py.eval("list(FAKE_MCU.sends)");
    let sends = rig.sends.lock().clone();
    check("set_reg cmd register", expected, sends_value(&sends));
}

#[test]
fn finish_measurements_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    capture_logs();
    py_stmt(
        &mut py,
        &format!("obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); finish(obj)", py_responses(&[])),
    );
    let expected = py.eval("(snapshot(obj), log_lines())");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    chip.finish_measurements().expect("rust finish");
    let logs = bmi160_log_lines();
    let actual = Value::Tuple(vec![
        snapshot(&rig, &chip),
        Value::List(logs.iter().map(|l| Value::str(l)).collect()),
    ]);
    check("finish", expected, actual);
}

#[test]
fn convert_samples_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py_stmt(
        &mut py,
        "obj = make_obj('bmi160', {'cs_pin': 'mcu:PA0', 'axes_map': '-z,y,x'}, [])",
    );
    py_stmt(
        &mut py,
        "s = [(1.1234567, 100, -200, 300), (0.0, 0, 0, 0), (2.5, -1, 2, -3)]; obj._convert_samples(s)",
    );
    let expected = py.eval("s");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0"), ("axes_map", "-z,y,x")])
        .expect("rust load");
    let mut samples = vec![
        PyValue::Tuple(vec![
            PyValue::Float(1.1234567),
            PyValue::Int(100_i64.into()),
            PyValue::Int((-200_i64).into()),
            PyValue::Int(300_i64.into()),
        ]),
        PyValue::Tuple(vec![
            PyValue::Float(0.0),
            PyValue::Int(0_i64.into()),
            PyValue::Int(0_i64.into()),
            PyValue::Int(0_i64.into()),
        ]),
        PyValue::Tuple(vec![
            PyValue::Float(2.5),
            PyValue::Int((-1_i64).into()),
            PyValue::Int(2_i64.into()),
            PyValue::Int((-3_i64).into()),
        ]),
    ];
    chip.convert_samples(&mut samples).expect("rust convert");
    check("convert samples", expected, Value::List(samples.iter().map(to_value).collect()));
}

#[test]
fn process_batch_empty_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py_stmt(
        &mut py,
        &format!("obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); result = batch(obj)", py_responses(&[])),
    );
    let expected = py.eval("(result, list(FAKE_MCU.sends))");
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let result = chip.process_batch().expect("rust batch");
    let sends = rig.sends.lock().clone();
    check(
        "process_batch empty",
        expected,
        Value::Tuple(vec![to_value(&result), sends_value(&sends)]),
    );
}

#[test]
fn process_batch_with_samples_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let bulk = b"\x01\x00\x02\x00\x03\x00";
    py_stmt(
        &mut py,
        "obj = make_obj('bmi160', {'cs_pin': 'mcu:PA0'}, [{'clock': 4000, 'next_sequence': 1, 'buffered': 6, 'possible_overflows': 0, 'query_ticks': 0}]); run_config()",
    );
    py_stmt(
        &mut py,
        &format!("FAKE_MCU.serial_cbs[0][0]({{'sequence': 0, 'data': {}}})", py_bytes(bulk)),
    );
    py_stmt(&mut py, "result = obj._process_batch(0.0)");
    let expected = py.eval("(result, list(FAKE_MCU.sends))");
    let rig = rig(vec![resp_status(4000, 1, 6, 0, 0)]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let mut params = IndexMap::new();
    params.insert(PyValue::Str("sequence".into()), PyValue::Int(0_i64.into()));
    params.insert(PyValue::Str("data".into()), PyValue::Bytes(bulk.to_vec()));
    rig.mcu.feed_bulk_data(PyValue::Dict(params));
    let result = chip.process_batch().expect("rust batch");
    let sends = rig.sends.lock().clone();
    check(
        "process_batch samples",
        expected,
        Value::Tuple(vec![to_value(&result), sends_value(&sends)]),
    );
}

#[test]
fn batch_client_endpoint_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let script = vec![
        vec![0, 0xFF],
        vec![0, 0xD1],
        vec![0, 0x00],
        vec![0, 0x00],
        vec![0, 0x2C],
        vec![0, 0x00],
        vec![0, 0x0C],
        vec![0, 0x00],
        vec![0, 0x00],
        vec![0, 0x00],
        vec![0, 0x40],
        vec![0, 0x00],
    ];
    capture_logs();
    py_stmt(
        &mut py,
        &format!(
            "obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {}); run_config(); \
             req = FakeRequest(); FAKE_WEBHOOKS.endpoints[0][3](req)",
            py_responses(&script)
        ),
    );
    let expected = py.eval("(req.sent, list(FAKE_MCU.sends))");
    let rig = rig(script.iter().map(|b| resp_bytes(b)).collect());
    // The batch helper only holds a weak reference, so the chip stays bound.
    let _chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    rig.mcu.run_config().expect("rust build_config");
    let callback = rig.endpoints.lock()[0].callback.clone();
    let request = Arc::new(FakeRequest { sent: Mutex::new(Vec::new()) });
    callback(request.clone() as WebRequest).expect("rust endpoint callback");
    let sent = request.sent.lock().clone();
    let sends = rig.sends.lock().clone();
    check(
        "batch endpoint",
        expected,
        Value::Tuple(vec![sent_values(&sent), sends_value(&sends)]),
    );
}

#[test]
fn read_axes_map_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    py_stmt(&mut py, "obj = make_obj('bmi160', {'axes_map': '-z,y,x'}, [])");
    let expected = py.eval("obj.axes_map");
    let rig = rig(vec![]);
    let cfg = FakeConfig {
        printer: rig.printer.clone(),
        name: "bmi160".into(),
        options: Mutex::new(vec![("axes_map".into(), "-z,y,x".into())]),
    };
    let config: Config = Arc::new(cfg);
    let map = read_axes_map(&config, SCALE, SCALE, SCALE).expect("rust axes");
    let actual = Value::List(
        map.iter()
            .map(|(p, s)| Value::Tuple(vec![Value::int(*p as i64), Value::float(*s)]))
            .collect(),
    );
    check("axes_map", expected, actual);
}

#[test]
fn read_axes_map_invalid_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    let expected = py.exec("obj = make_obj('bmi160', {'axes_map': 'a,b,c'}, [])");
    let rig = rig(vec![]);
    let cfg = FakeConfig {
        printer: rig.printer.clone(),
        name: "bmi160".into(),
        options: Mutex::new(vec![("axes_map".into(), "a,b,c".into())]),
    };
    let config: Config = Arc::new(cfg);
    let err = read_axes_map(&config, SCALE, SCALE, SCALE).expect_err("rust axes must fail");
    check_exc("axes_map invalid", expected, &err);
}

#[test]
fn accel_chip_interface_matches_oracle() {
    let Some(mut py) = oracle() else { return };
    // `resonance_tester` finds the chip with `printer.lookup_object(name)` and
    // requires `start_internal_client`, `read_reg`, `set_reg` and `name`.
    py_stmt(
        &mut py,
        &format!("obj = make_obj('bmi160', {{'cs_pin': 'mcu:PA0'}}, {})", py_responses(&[])),
    );
    let expected = py.eval(
        "(hasattr(obj, 'start_internal_client'), hasattr(obj, 'read_reg'), \
         hasattr(obj, 'set_reg'), obj.name)",
    );
    let rig = rig(vec![]);
    let chip = load_chip(&rig, "bmi160", vec![("cs_pin", "mcu:PA0")]).expect("rust load");
    let object: ObjectRef = chip;
    let iface = klippy_api::interface::<dyn extras::adxl345::AccelChipApi>(&object);
    let (declared, name) = match &iface {
        Some(iface) => (true, iface.name()),
        None => (false, String::new()),
    };
    let actual = Value::Tuple(vec![
        Value::Bool(declared),
        Value::Bool(declared),
        Value::Bool(declared),
        Value::str(&name),
    ]);
    check("accelerometer interface", expected, actual);
}

#[test]
fn module_factory_has_load_entries() {
    let f = extras::bmi160::module_factory();
    assert!(f.load_config.is_some());
    assert!(f.load_config_prefix.is_some());
}
