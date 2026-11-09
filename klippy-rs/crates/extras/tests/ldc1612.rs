//! Parity tests for `extras::ldc1612` (port of `klippy/extras/ldc1612.py`).
//!
//! Every expectation comes from the live reference through `pyoracle`
//! (CPython 3.8.2 with the frozen `klippy/`). Both sides build the sensor
//! through the production constructor (`ldc1612.LDC1612(config, cal)` and
//! `LDC1612::new(&config, cal)`) on top of the real collaborators:
//! `configfile.ConfigWrapper` (config defaults, ranges and error messages),
//! `bus.MCU_I2C_from_config` / `MCU_I2C` (the I2C traffic) and
//! `bulk_sensor.FixedFreqReader` / `BatchBulkHelper`. Only the printer, the
//! MCU, `pins`, `gcode`, `toolhead`, `configfile`, `webhooks` and the
//! calibration object are faked, identically on both sides. The fake MCU
//! records config commands, command lookups and every command sent, and
//! answers `i2c_transfer` and `query_status_ldc1612` queries from queued
//! replies.
//!
//! Compared: MCU config commands, command lookups, I2C register traffic,
//! `query_ldc1612` sends, the `LDC_CALIBRATE_DRIVE_CURRENT` mux command and
//! its responses and `configfile.set` write-back, the `ldc1612/dump_ldc1612`
//! endpoint, config access tracking (options and defaults), config errors,
//! `intb_pin` handling, the `max_sensor_hz` runtime warning, sensor error
//! enumeration lookups, `_process_batch` results and log lines, start/finish
//! logs, the invalid-id command error and the I2C failure paths.
//!
//! No recordings exist under `tests/fixtures/recordings/` for these units.

use extras::ldc1612::{module_factory, LDC1612};
use extras::probe_eddy_current::{DriftCompensationApi, EddyCalibrationApi, Measures};
use indexmap::IndexMap;
use klippy_api::config::{Config, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{
    GCodeApi, GCodeCommandApi, GCodeCommandHandler, GCmd, GetOpts, RegisterCommandOpts,
};
use klippy_api::mcu::{
    AddConfigCmdOpts, AsyncResponseWrapper, CommandQueryWrapper, CommandQueue, CommandWrapper,
    ConfigCallback, LookupPinOpts, LookupQueryCommandOpts, McuApi, PinParams, PinsApi,
    QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::webhooks::WebHooksApi;
use klippy_api::{EventArg, GCell, ObjectRef};
use klippy_core::configfile::{ConfigFileReader, ConfigWrapper};
use pycompat::text::float_repr;
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, ReactorMutex};
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::Arc;

// ---------------------------------------------------------------------------
// Python side: fakes around the real configfile, bus and bulk_sensor.
// ---------------------------------------------------------------------------

const SETUP: &str = r#"
import logging
import struct
import configfile
import gcode
import extras.ldc1612 as ldc1612

class FakeReactor:
    NEVER = 9999999999999999.
    def monotonic(self):
        return 1000.0
    def register_timer(self, callback, waketime):
        return (callback, waketime)
    def unregister_timer(self, timer_handler):
        pass

class FakePrinter:
    command_error = gcode.CommandError
    config_error = configfile.error
    def __init__(self):
        self.objects = {}
        self.reactor = FakeReactor()
        self.events = []
        self.shutdowns = []
    def get_reactor(self):
        return self.reactor
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def register_event_handler(self, event, callback):
        self.events.append(event)
    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

def fmt_data(data):
    return "[%s]" % (", ".join(
        str(list(x)) if isinstance(x, (bytes, bytearray, list)) else str(x)
        for x in data),)

class FakeCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append("%s|send|%s|%d|%d"
                              % (self.fmt, fmt_data(data), minclock, reqclock))
    def send_wait_ack(self, data, minclock=0, reqclock=0):
        self.mcu.sends.append("%s|send_wait_ack|%s|%d|%d"
                              % (self.fmt, fmt_data(data), minclock, reqclock))

class FakeQueryCmd:
    def __init__(self, mcu, fmt):
        self.mcu = mcu
        self.fmt = fmt
    def send(self, data, minclock=0, reqclock=0, retry=True):
        self.mcu.sends.append("%s|query|%s|%d|%d|%s" % (
            self.fmt, fmt_data(data), minclock, reqclock, retry))
        if self.fmt.startswith("query_status_ldc1612"):
            return dict(self.mcu.status)
        if data[2] == 0:
            return {'i2c_bus_status': 'SUCCESS', 'response': b''}
        status, response = self.mcu.i2c_replies.pop(0)
        return {'i2c_bus_status': status, 'response': bytes(response)}

class FakeMcu:
    def __init__(self, printer, fileoutput, enumerations):
        self.printer = printer
        self._fileoutput = fileoutput
        self._enumerations = enumerations
        self._next_oid = 0
        self._next_cq = 0
        self.config_cmds = []
        self.lookups = []
        self.sends = []
        self.callbacks = []
        self.serial_cb = None
        self.status = {'clock': 100000, 'next_sequence': 10, 'buffered': 32,
                       'possible_overflows': 0, 'query_ticks': 100}
        self.i2c_replies = []
    def get_name(self):
        return 'mcu'
    def get_printer(self):
        return self.printer
    def is_fileoutput(self):
        return self._fileoutput
    def create_oid(self):
        self._next_oid += 1
        return self._next_oid - 1
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        self.config_cmds.append("%s|%s|%s" % (cmd, is_init, on_restart))
    def alloc_command_queue(self):
        self._next_cq += 1
        return self._next_cq
    def register_config_callback(self, cb):
        self.callbacks.append(cb)
    def try_lookup_command(self, msgformat):
        self.lookups.append("try|%s" % (msgformat,))
        return None
    def lookup_command(self, msgformat, cq=None):
        self.lookups.append("cmd|%s|%s" % (msgformat, cq))
        return FakeCmd(self, msgformat)
    def lookup_query_command(self, msgformat, respformat, oid=None, cq=None,
                             is_async=False):
        self.lookups.append("query|%s|%s|%s|%s|%s"
                            % (msgformat, respformat, oid, cq, is_async))
        return FakeQueryCmd(self, msgformat)
    def register_serial_response(self, cb, msg, oid=None):
        self.lookups.append("serial|%s|%s" % (msg, oid))
        self.serial_cb = cb
    def get_enumerations(self):
        return self._enumerations
    def get_constants(self):
        return {}
    def seconds_to_clock(self, time):
        return int(time * 12000000)
    def clock_to_print_time(self, clock):
        return clock / 12000000.
    def clock32_to_clock64(self, clock32):
        return clock32
    def run_callbacks(self):
        for cb in self.callbacks:
            cb()

class FakePins:
    error = configfile.error
    def __init__(self, chip):
        self.chip = chip
        self.looked = []
    def lookup_pin(self, pin_desc, can_invert=False, can_pullup=False,
                   share_type=None):
        self.looked.append("%s|%s|%s|%s"
                           % (pin_desc, can_invert, can_pullup, share_type))
        return {'chip': self.chip, 'chip_name': 'mcu', 'pin': 'PB1',
                'invert': 0, 'pullup': 0}

class FakeGCode:
    def __init__(self):
        self.mux = []
        self.handlers = []
    def register_mux_command(self, cmd, key, value, func, desc=None):
        self.mux.append("%s|%s|%s|%s" % (cmd, key, value, desc))
        self.handlers.append(func)

class FakeGcmd:
    def __init__(self):
        self.responses = []
    def respond_info(self, msg, log=True):
        self.responses.append("%s|%s" % (msg, log))

class FakeToolhead:
    def __init__(self):
        self.calls = []
    def dwell(self, delay):
        self.calls.append("dwell %s" % (delay,))
    def wait_moves(self):
        self.calls.append("wait_moves")

class FakeConfigFileObj:
    def __init__(self):
        self.sets = []
        self.warnings = []
    def set(self, section, option, value):
        self.sets.append("%s|%s|%s" % (section, option, value))
    def runtime_warning(self, msg):
        self.warnings.append(msg)

class FakeWebhooks:
    def __init__(self):
        self.mux = []
    def register_mux_endpoint(self, path, key, value, callback):
        self.mux.append("%s|%s|%s" % (path, key, value))

class FakeCal:
    def __init__(self, freqs, zpos, offset=None):
        self._freqs = list(freqs)
        self._zpos = list(zpos)
        self._offset = offset
    def get_calibration(self):
        return list(self._freqs), list(self._zpos)
    def apply_calibration(self, samples):
        if self._offset is not None:
            for i, (t, f, z) in enumerate(samples):
                samples[i] = (t, f, z + self._offset)

LOG_LINES = []
class _LogHandler(logging.Handler):
    def emit(self, record):
        LOG_LINES.append("%s:%s" % (record.levelname, record.getMessage()))
logging.getLogger().addHandler(_LogHandler())
logging.getLogger().setLevel(0)

class NS:
    pass
w = NS()

def build(section, text, fileoutput, enumerations, cal, intb_wrong_mcu):
    w.p = FakePrinter()
    w.mcu = FakeMcu(w.p, fileoutput, enumerations)
    w.other_mcu = FakeMcu(w.p, fileoutput, enumerations)
    w.gcode = FakeGCode()
    w.toolhead = FakeToolhead()
    w.cfl = FakeConfigFileObj()
    w.webhooks = FakeWebhooks()
    w.pins = FakePins(w.other_mcu if intb_wrong_mcu else w.mcu)
    w.p.objects.update({'mcu': w.mcu, 'gcode': w.gcode,
                        'toolhead': w.toolhead, 'configfile': w.cfl,
                        'pins': w.pins, 'webhooks': w.webhooks})
    fileconfig = configfile.ConfigFileReader().build_fileconfig(
        text, "printer.cfg")
    w.config = configfile.ConfigWrapper(w.p, fileconfig, {}, section)
    w.ldc = None
    w.ldc = ldc1612.LDC1612(w.config, cal)

def access_tracking():
    return ["%s|%s|%r" % (k[0], k[1], v)
            for k, v in w.config.access_tracking.items()]
"#;

// ---------------------------------------------------------------------------
// Rust side: the same fakes.
// ---------------------------------------------------------------------------

fn py_bool(b: bool) -> &'static str {
    if b {
        "True"
    } else {
        "False"
    }
}

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn int_of(v: &PyValue) -> i64 {
    match v {
        PyValue::Int(n) => n.inner().to_string().parse().expect("small int"),
        other => panic!("expected an int, got {other:?}"),
    }
}

/// `fmt_data(data)` of the Python fake.
fn fmt_data(data: &[PyValue]) -> String {
    let parts: Vec<String> = data
        .iter()
        .map(|v| match v {
            PyValue::Int(n) => n.inner().to_string(),
            PyValue::Bytes(b) => format!("{b:?}"),
            PyValue::List(xs) => {
                format!("[{}]", xs.iter().map(|x| int_of(x).to_string()).collect::<Vec<_>>().join(", "))
            }
            other => panic!("unexpected command argument {other:?}"),
        })
        .collect();
    format!("[{}]", parts.join(", "))
}

#[derive(Default)]
struct FakePrinter {
    objects: GCell<IndexMap<String, ObjectRef>>,
    reactor: GCell<Option<Reactor>>,
    events: GCell<Vec<String>>,
    shutdowns: GCell<Vec<String>>,
}

impl FakePrinter {
    fn put(&self, name: &str, obj: ObjectRef) {
        self.objects.borrow_mut().insert(name.to_string(), obj);
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::None
    }
    fn get_reactor(&self) -> Reactor {
        let mut reactor = self.reactor.borrow_mut();
        reactor.get_or_insert_with(|| Reactor::new(false)).clone()
    }
    fn get_state_message(&self) -> (String, String) {
        (String::new(), "ready".to_string())
    }
    fn is_shutdown(&self) -> bool {
        false
    }
    fn update_error_msg(&self, _old_msg: &str, _new_msg: &str) {}
    fn add_object(&self, _name: &str, _obj: ObjectRef) -> KResult<()> {
        panic!("add_object is not used by ldc1612")
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name) {
            Some(obj) => Ok(Some(obj.clone())),
            None => match default {
                LookupDefault::Raise => {
                    Err(KlippyError::config(format!("Unknown config object '{name}'")))
                }
                LookupDefault::ReturnNone => Ok(None),
            },
        }
    }
    fn lookup_objects(&self, _module: Option<&str>) -> Vec<(String, ObjectRef)> {
        panic!("lookup_objects is not used by ldc1612")
    }
    fn load_object(
        &self,
        _config: &Config,
        _section: &str,
        _default: LookupDefault,
    ) -> KResult<Option<ObjectRef>> {
        panic!("load_object is not used by ldc1612")
    }
    fn run(&self) -> Option<String> {
        None
    }
    fn set_rollover_info(&self, _name: &str, _info: Option<&str>, _log: bool) {}
    fn invoke_shutdown(&self, msg: &str, _details: PyValue) {
        self.shutdowns.borrow_mut().push(msg.to_string());
    }
    fn invoke_async_shutdown(&self, _msg: &str, _details: PyValue) {
        panic!("invoke_async_shutdown is not used by ldc1612")
    }
    fn register_event_handler(&self, event: &str, _callback: EventCallback) {
        self.events.borrow_mut().push(event.to_string());
    }
    fn send_event(&self, _event: &str, _params: &[EventArg]) -> KResult<Vec<PyValue>> {
        panic!("send_event is not used by ldc1612")
    }
    fn request_exit(&self, _result: &str) {}
}

/// Shared recorder of one fake MCU and its command objects.
#[derive(Default)]
struct McuLog {
    next_oid: i64,
    next_cq: i64,
    config_cmds: Vec<String>,
    lookups: Vec<String>,
    sends: Vec<String>,
    callbacks: Vec<ConfigCallback>,
    serial_cb: Option<SerialResponseCallback>,
    status: IndexMap<String, i64>,
    i2c_replies: VecDeque<(String, Vec<u8>)>,
}

struct FakeCmd {
    log: Arc<GCell<McuLog>>,
    fmt: String,
}

impl klippy_api::mcu::CommandWrapperApi for FakeCmd {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let line = format!("{}|send|{}|{}|{}", self.fmt, fmt_data(&data), opts.minclock, opts.reqclock);
        self.log.borrow_mut().sends.push(line);
        Ok(())
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let line = format!(
            "{}|send_wait_ack|{}|{}|{}",
            self.fmt,
            fmt_data(&data),
            opts.minclock,
            opts.reqclock
        );
        self.log.borrow_mut().sends.push(line);
        Ok(())
    }
    fn get_command_tag(&self) -> i64 {
        panic!("get_command_tag is not used by ldc1612")
    }
}

struct FakeQueryCmd {
    log: Arc<GCell<McuLog>>,
    fmt: String,
}

fn str_key(k: &str) -> PyValue {
    PyValue::Str(k.to_string())
}

impl klippy_api::mcu::CommandQueryWrapperApi for FakeQueryCmd {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        let mut log = self.log.borrow_mut();
        log.sends.push(format!(
            "{}|query|{}|{}|{}|{}",
            self.fmt,
            fmt_data(&data),
            opts.minclock,
            opts.reqclock,
            py_bool(opts.retry)
        ));
        let mut params = IndexMap::new();
        if self.fmt.starts_with("query_status_ldc1612") {
            for (k, v) in log.status.iter() {
                params.insert(str_key(k), py_int(*v));
            }
            return Ok(PyValue::Dict(params));
        }
        let (status, response) = if int_of(&data[2]) == 0 {
            ("SUCCESS".to_string(), Vec::new())
        } else {
            log.i2c_replies.pop_front().ok_or_else(|| KlippyError::Internal {
                class_name: "IndexError".to_string(),
                message: "pop from empty list".to_string(),
            })?
        };
        params.insert(str_key("i2c_bus_status"), PyValue::Str(status));
        params.insert(str_key("response"), PyValue::Bytes(response));
        Ok(PyValue::Dict(params))
    }
    fn send_with_preface(
        &self,
        _preface_cmd: CommandWrapper,
        _preface_data: Vec<PyValue>,
        _data: Vec<PyValue>,
        _opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        panic!("send_with_preface is not used by ldc1612")
    }
}

struct FakeAsync;
impl klippy_api::mcu::AsyncResponseWrapperApi for FakeAsync {
    fn unregister(&self) {}
}

struct FakeMcu {
    log: Arc<GCell<McuLog>>,
    printer: Printer,
    fileoutput: bool,
    enums: IndexMap<String, IndexMap<String, i64>>,
}
klippy_api::printer_object!(FakeMcu; dyn McuApi);

impl FakeMcu {
    fn new(printer: Printer, fileoutput: bool, enums: IndexMap<String, IndexMap<String, i64>>) -> Arc<FakeMcu> {
        let mcu = Arc::new(FakeMcu {
            log: Arc::new(GCell::new(McuLog::default())),
            printer,
            fileoutput,
            enums,
        });
        mcu.set_status(100000, 10, 32, 0, 100);
        mcu
    }
    fn set_status(&self, clock: i64, next_sequence: i64, buffered: i64, overflows: i64, query_ticks: i64) {
        let mut status = IndexMap::new();
        status.insert("clock".to_string(), clock);
        status.insert("next_sequence".to_string(), next_sequence);
        status.insert("buffered".to_string(), buffered);
        status.insert("possible_overflows".to_string(), overflows);
        status.insert("query_ticks".to_string(), query_ticks);
        self.log.borrow_mut().status = status;
    }
    fn queue_replies(&self, replies: &[(&str, Vec<u8>)]) {
        self.log.borrow_mut().i2c_replies =
            replies.iter().map(|(s, r)| (s.to_string(), r.clone())).collect();
    }
    fn run_callbacks(&self) -> KResult<()> {
        let mut callbacks = std::mem::take(&mut self.log.borrow_mut().callbacks);
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        Ok(())
    }
    fn feed_bulk(&self, sequence: i64, data: Vec<u8>) {
        let cb = self.log.borrow().serial_cb.clone().expect("sensor_bulk_data registered");
        let mut params = IndexMap::new();
        params.insert(str_key("sequence"), py_int(sequence));
        params.insert(str_key("data"), PyValue::Bytes(data));
        cb(PyValue::Dict(params));
    }
    fn config_cmds(&self) -> Value {
        strs(&self.log.borrow().config_cmds)
    }
    fn lookups(&self) -> Value {
        strs(&self.log.borrow().lookups)
    }
    fn sends(&self) -> Value {
        strs(&self.log.borrow().sends)
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
        self.fileoutput
    }
    fn setup_pin(&self, _pin_type: &str, _pin_params: PinParams) -> KResult<klippy_api::mcu::Pin> {
        panic!("setup_pin is not used by ldc1612")
    }
    fn create_oid(&self) -> KResult<i64> {
        let mut log = self.log.borrow_mut();
        log.next_oid += 1;
        Ok(log.next_oid - 1)
    }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.log.borrow_mut().callbacks.push(cb);
        Ok(())
    }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        let line = format!("{}|{}|{}", cmd, py_bool(opts.is_init), py_bool(opts.on_restart));
        self.log.borrow_mut().config_cmds.push(line);
        Ok(())
    }
    fn request_move_queue_slot(&self) {
        panic!("request_move_queue_slot is not used by ldc1612")
    }
    fn get_query_slot(&self, _oid: i64) -> i64 {
        panic!("get_query_slot is not used by ldc1612")
    }
    fn seconds_to_clock(&self, time: f64) -> i64 {
        kos::cconv::f64_to_i64(time * 12000000.0)
    }
    fn min_schedule_time(&self) -> f64 {
        panic!("min_schedule_time is not used by ldc1612")
    }
    fn max_nominal_duration(&self) -> f64 {
        panic!("max_nominal_duration is not used by ldc1612")
    }
    fn lookup_command(&self, msgformat: &str, cq: Option<CommandQueue>) -> KResult<CommandWrapper> {
        let line = format!("cmd|{}|{}", msgformat, cq.map_or("None".to_string(), |c| c.0.to_string()));
        self.log.borrow_mut().lookups.push(line);
        Ok(Arc::new(FakeCmd { log: self.log.clone(), fmt: msgformat.to_string() }))
    }
    fn lookup_query_command(
        &self,
        msgformat: &str,
        respformat: &str,
        opts: LookupQueryCommandOpts,
    ) -> KResult<CommandQueryWrapper> {
        let line = format!(
            "query|{}|{}|{}|{}|{}",
            msgformat,
            respformat,
            opts.oid.map_or("None".to_string(), |o| o.to_string()),
            opts.cq.map_or("None".to_string(), |c| c.0.to_string()),
            py_bool(opts.is_async)
        );
        self.log.borrow_mut().lookups.push(line);
        Ok(Arc::new(FakeQueryCmd { log: self.log.clone(), fmt: msgformat.to_string() }))
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<CommandWrapper> {
        self.log.borrow_mut().lookups.push(format!("try|{msgformat}"));
        None
    }
    fn alloc_command_queue(&self) -> CommandQueue {
        let mut log = self.log.borrow_mut();
        log.next_cq += 1;
        CommandQueue(log.next_cq)
    }
    fn register_serial_response(
        &self,
        cb: SerialResponseCallback,
        msg: &str,
        oid: Option<i64>,
    ) -> AsyncResponseWrapper {
        let mut log = self.log.borrow_mut();
        log.lookups.push(format!("serial|{}|{}", msg, oid.map_or("None".to_string(), |o| o.to_string())));
        log.serial_cb = Some(cb);
        Arc::new(FakeAsync)
    }
    fn check_valid_response(&self, _msgformat: &str) -> bool {
        panic!("check_valid_response is not used by ldc1612")
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        self.enums.clone()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        IndexMap::new()
    }
    fn get_constant_float(&self, _name: &str) -> KResult<f64> {
        panic!("get_constant_float is not used by ldc1612")
    }
    fn print_time_to_clock(&self, _print_time: f64) -> i64 {
        panic!("print_time_to_clock is not used by ldc1612")
    }
    fn clock_to_print_time(&self, clock: f64) -> f64 {
        clock / 12000000.0
    }
    fn estimated_print_time(&self, _eventtime: f64) -> f64 {
        panic!("estimated_print_time is not used by ldc1612")
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        clock32
    }
    fn calibrate_clock(&self, _print_time: f64, _eventtime: f64) -> KResult<(f64, f64)> {
        panic!("calibrate_clock is not used by ldc1612")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by ldc1612")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by ldc1612")
    }
    fn serialqueue(&self) -> Option<std::sync::Arc<chelper_serial::serialqueue::SerialQueue>> {
        None
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        panic!("create_trigger_dispatch is not used by ldc1612")
    }
}

struct FakePins {
    chip: klippy_api::mcu::Mcu,
    looked: GCell<Vec<String>>,
}
klippy_api::printer_object!(FakePins; dyn PinsApi);

impl PinsApi for FakePins {
    fn parse_pin(&self, _pin_desc: &str, _opts: klippy_api::mcu::ParsePinOpts) -> KResult<PinParams> {
        panic!("parse_pin is not used by ldc1612")
    }
    fn lookup_pin(&self, pin_desc: &str, opts: LookupPinOpts) -> KResult<PinParams> {
        self.looked.borrow_mut().push(format!(
            "{}|{}|{}|{}",
            pin_desc,
            py_bool(opts.can_invert),
            py_bool(opts.can_pullup),
            opts.share_type.as_deref().unwrap_or("None")
        ));
        Ok(PinParams {
            chip: self.chip.clone(),
            chip_name: "mcu".to_string(),
            pin: "PB1".to_string(),
            invert: 0,
            pullup: 0,
            share_type: None,
        })
    }
    fn setup_pin(&self, _pin_type: &str, _pin_desc: &str) -> KResult<klippy_api::mcu::Pin> {
        panic!("setup_pin is not used by ldc1612")
    }
    fn reset_pin_sharing(&self, _pin_params: PinParams) {
        panic!("reset_pin_sharing is not used by ldc1612")
    }
    fn get_pin_resolver(&self, _chip_name: &str) -> KResult<klippy_api::mcu::PinResolver> {
        panic!("get_pin_resolver is not used by ldc1612")
    }
    fn register_chip(&self, _chip_name: &str, _chip: klippy_api::mcu::Chip) -> KResult<()> {
        panic!("register_chip is not used by ldc1612")
    }
    fn allow_multi_use_pin(&self, _pin_desc: &str) -> KResult<()> {
        panic!("allow_multi_use_pin is not used by ldc1612")
    }
}

#[derive(Default)]
struct FakeGCode {
    mux: GCell<Vec<String>>,
    handlers: GCell<Vec<Option<GCodeCommandHandler>>>,
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl FakeGCode {
    fn run_handler(&self, index: usize, gcmd: &GCmd) -> KResult<()> {
        let mut handler = self.handlers.borrow_mut()[index].take().expect("mux handler registered");
        let result = handler(gcmd);
        self.handlers.borrow_mut()[index] = Some(handler);
        result
    }
}

impl GCodeApi for FakeGCode {
    fn register_command(
        &self,
        _cmd: &str,
        _func: Option<GCodeCommandHandler>,
        _opts: RegisterCommandOpts,
    ) -> KResult<Option<GCodeCommandHandler>> {
        panic!("register_command is not used by ldc1612")
    }
    fn register_mux_command(
        &self,
        cmd: &str,
        key: &str,
        value: Option<&str>,
        func: GCodeCommandHandler,
        desc: Option<&str>,
    ) -> KResult<()> {
        self.mux.borrow_mut().push(format!(
            "{}|{}|{}|{}",
            cmd,
            key,
            value.unwrap_or("None"),
            desc.unwrap_or("None")
        ));
        self.handlers.borrow_mut().push(Some(func));
        Ok(())
    }
    fn get_command_help(&self) -> IndexMap<String, String> {
        panic!("get_command_help is not used by ldc1612")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by ldc1612")
    }
    fn register_output_handler(&self, _callback: klippy_api::gcode::OutputCallback) {
        panic!("register_output_handler is not used by ldc1612")
    }
    fn request_restart(&self, _result: &str) -> KResult<()> {
        panic!("request_restart is not used by ldc1612")
    }
    fn run_script(&self, _script: &str) -> KResult<()> {
        panic!("run_script is not used by ldc1612")
    }
    fn run_script_from_command(&self, _script: &str) -> KResult<()> {
        panic!("run_script_from_command is not used by ldc1612")
    }
    fn get_mutex(&self) -> ReactorMutex {
        panic!("get_mutex is not used by ldc1612")
    }
    fn create_gcode_command(
        &self,
        _command: &str,
        _commandline: &str,
        _params: &IndexMap<String, String>,
    ) -> GCmd {
        panic!("create_gcode_command is not used by ldc1612")
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("respond_raw is not used by ldc1612")
    }
    fn respond_info(&self, _msg: &str, _log: bool) {
        panic!("respond_info is not used by ldc1612")
    }
    fn is_traditional_gcode(&self, _cmd: &str) -> bool {
        panic!("is_traditional_gcode is not used by ldc1612")
    }
    fn cmd_m112(&self, _gcmd: Option<&GCmd>) -> KResult<()> {
        panic!("cmd_m112 is not used by ldc1612")
    }
    fn process_commands(&self, _commands: &[String], _need_ack: bool) -> KResult<()> {
        panic!("process_commands is not used by ldc1612")
    }
}

#[derive(Default)]
struct FakeGCmd {
    responses: GCell<Vec<String>>,
}

impl GCodeCommandApi for FakeGCmd {
    fn get(&self, _name: &str, _opts: GetOpts) -> KResult<PyValue> {
        panic!("gcmd.get is not used by ldc1612")
    }
    fn get_int(&self, _name: &str, _opts: klippy_api::gcode::GetIntOpts) -> KResult<Option<i64>> {
        panic!("gcmd.get_int is not used by ldc1612")
    }
    fn get_float(&self, _name: &str, _opts: klippy_api::gcode::GetFloatOpts) -> KResult<Option<f64>> {
        panic!("gcmd.get_float is not used by ldc1612")
    }
    fn get_command(&self) -> String {
        panic!("gcmd.get_command is not used by ldc1612")
    }
    fn get_commandline(&self) -> String {
        panic!("gcmd.get_commandline is not used by ldc1612")
    }
    fn get_command_parameters(&self) -> IndexMap<String, String> {
        panic!("gcmd.get_command_parameters is not used by ldc1612")
    }
    fn get_raw_command_parameters(&self) -> String {
        panic!("gcmd.get_raw_command_parameters is not used by ldc1612")
    }
    fn respond_info(&self, msg: &str, log: bool) {
        self.responses.borrow_mut().push(format!("{}|{}", msg, py_bool(log)));
    }
    fn respond_raw(&self, _msg: &str) {
        panic!("gcmd.respond_raw is not used by ldc1612")
    }
    fn ack(&self, _msg: Option<&str>) -> bool {
        panic!("gcmd.ack is not used by ldc1612")
    }
}

#[derive(Default)]
struct FakeToolhead {
    calls: GCell<Vec<String>>,
}
klippy_api::printer_object!(FakeToolhead; dyn ToolHeadApi);

impl klippy_api::toolhead::HomingTargetApi for FakeToolhead {
    fn get_position(&self) -> Vec<f64> {
        panic!("get_position is not used by ldc1612")
    }
    fn set_position(&self, _newpos: Vec<f64>, _homing_axes: &str) -> KResult<()> {
        panic!("set_position is not used by ldc1612")
    }
    fn get_kinematics(&self) -> klippy_api::toolhead::Kinematics {
        panic!("get_kinematics is not used by ldc1612")
    }
    fn flush_step_generation(&self) -> KResult<()> {
        panic!("flush_step_generation is not used by ldc1612")
    }
    fn get_last_move_time(&self) -> f64 {
        panic!("get_last_move_time is not used by ldc1612")
    }
    fn dwell(&self, delay: f64) {
        self.calls.borrow_mut().push(format!("dwell {}", float_repr(delay)));
    }
    fn drip_move(
        &self,
        _newpos: Vec<f64>,
        _speed: f64,
        _drip_completion: &reactor::Completion<bool>,
    ) -> KResult<()> {
        panic!("drip_move is not used by ldc1612")
    }
}

impl ToolHeadApi for FakeToolhead {
    fn r#move(&self, _newpos: Vec<f64>, _speed: f64) -> KResult<()> {
        panic!("move is not used by ldc1612")
    }
    fn manual_move(&self, _coord: Vec<Option<f64>>, _speed: f64) -> KResult<()> {
        panic!("manual_move is not used by ldc1612")
    }
    fn wait_moves(&self) {
        self.calls.borrow_mut().push("wait_moves".to_string());
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by ldc1612")
    }
    fn get_trapq(&self) -> klippy_api::stepper::Trapq {
        panic!("get_trapq is not used by ldc1612")
    }
    fn get_max_velocity(&self) -> (f64, f64) {
        panic!("get_max_velocity is not used by ldc1612")
    }
    fn set_max_velocities(
        &self,
        _max_velocity: Option<f64>,
        _max_accel: Option<f64>,
        _square_corner_velocity: Option<f64>,
        _min_cruise_ratio: Option<f64>,
    ) -> (f64, f64, f64, f64) {
        panic!("set_max_velocities is not used by ldc1612")
    }
    fn register_lookahead_callback(
        &self,
        _callback: klippy_api::toolhead::LookaheadCallback,
    ) -> KResult<()> {
        panic!("register_lookahead_callback is not used by ldc1612")
    }
    fn check_busy(&self, _eventtime: f64) -> (f64, f64, bool) {
        panic!("check_busy is not used by ldc1612")
    }
    fn limit_next_junction_speed(&self, _speed: f64) {
        panic!("limit_next_junction_speed is not used by ldc1612")
    }
    fn set_extruder(&self, _extruder: klippy_api::toolhead::PrinterExtruder, _extrude_pos: f64) {
        panic!("set_extruder is not used by ldc1612")
    }
    fn get_extruder(&self) -> klippy_api::toolhead::PrinterExtruder {
        panic!("get_extruder is not used by ldc1612")
    }
    fn add_extra_axis(&self, _ea: klippy_api::toolhead::ExtraAxis, _axis_pos: f64) {
        panic!("add_extra_axis is not used by ldc1612")
    }
    fn remove_extra_axis(&self, _ea: &klippy_api::toolhead::ExtraAxis) {
        panic!("remove_extra_axis is not used by ldc1612")
    }
    fn get_extra_axes(&self) -> Vec<Option<klippy_api::toolhead::ExtraAxis>> {
        panic!("get_extra_axes is not used by ldc1612")
    }
    fn stats(&self, _eventtime: f64) -> (bool, String) {
        panic!("stats is not used by ldc1612")
    }
    fn stepper_factory(&self) -> klippy_api::stepper::StepperFactory {
        panic!("stepper_factory is not used by ldc1612")
    }
}

#[derive(Default)]
struct FakeConfigFile {
    sets: GCell<Vec<String>>,
    warnings: GCell<Vec<String>>,
}
klippy_api::printer_object!(FakeConfigFile; dyn PrinterConfigApi);

impl PrinterConfigApi for FakeConfigFile {
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by ldc1612")
    }
    fn read_config(&self, _filename: &str) -> KResult<Config> {
        panic!("read_config is not used by ldc1612")
    }
    fn read_main_config(&self) -> KResult<Config> {
        panic!("read_main_config is not used by ldc1612")
    }
    fn log_config(&self, _config: &Config) {
        panic!("log_config is not used by ldc1612")
    }
    fn check_unused_options(&self, _config: &Config) -> KResult<()> {
        panic!("check_unused_options is not used by ldc1612")
    }
    fn runtime_warning(&self, msg: &str) {
        self.warnings.borrow_mut().push(msg.to_string());
    }
    fn deprecate(&self, _section: &str, _option: &str, _value: Option<PyValue>, _msg: Option<String>) {
        panic!("deprecate is not used by ldc1612")
    }
    fn deprecate_gcode(&self, _cmd: &str, _opts: klippy_api::config::DeprecateGcodeOpts) {
        panic!("deprecate_gcode is not used by ldc1612")
    }
    fn deprecate_mcu_code(&self, _mcu: &klippy_api::mcu::Mcu, _feature: &str, _msg: Option<String>) {
        panic!("deprecate_mcu_code is not used by ldc1612")
    }
    fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        panic!("get_status is not used by ldc1612")
    }
    fn set(&self, section: &str, option: &str, value: &str) {
        self.sets.borrow_mut().push(format!("{section}|{option}|{value}"));
    }
    fn remove_section(&self, _section: &str) {
        panic!("remove_section is not used by ldc1612")
    }
}

#[derive(Default)]
struct FakeWebhooks {
    mux: GCell<Vec<String>>,
}
klippy_api::printer_object!(FakeWebhooks; dyn WebHooksApi);

impl WebHooksApi for FakeWebhooks {
    fn register_endpoint(
        &self,
        _path: &str,
        _callback: klippy_api::webhooks::WebHooksCallback,
    ) -> KResult<()> {
        panic!("register_endpoint is not used by ldc1612")
    }
    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        _callback: klippy_api::webhooks::WebHooksCallback,
    ) -> KResult<()> {
        self.mux.borrow_mut().push(format!("{path}|{key}|{value}"));
        Ok(())
    }
    fn call_remote_method(&self, _method: &str, _params: &IndexMap<String, PyValue>) -> KResult<()> {
        panic!("call_remote_method is not used by ldc1612")
    }
    fn get_status(&self, _eventtime: f64) -> PyValue {
        panic!("get_status is not used by ldc1612")
    }
}

/// The Python `FakeCal`: only the two calls `LDC1612` makes on its
/// calibration argument do anything.
struct FakeCal {
    freqs: Vec<f64>,
    zpos: Vec<f64>,
    offset: Option<f64>,
}

impl EddyCalibrationApi for FakeCal {
    fn get_printer(&self) -> Printer {
        panic!("get_printer is not used by ldc1612")
    }
    fn note_z_calibration_start(&self) -> KResult<()> {
        panic!("note_z_calibration_start is not used by ldc1612")
    }
    fn note_z_calibration_finish(&self) -> KResult<()> {
        panic!("note_z_calibration_finish is not used by ldc1612")
    }
    fn register_drift_compensation(&self, _comp: Arc<dyn DriftCompensationApi>) {
        panic!("register_drift_compensation is not used by ldc1612")
    }
    fn verify_calibrated(&self) -> KResult<()> {
        panic!("verify_calibrated is not used by ldc1612")
    }
    fn get_calibration(&self) -> (Vec<f64>, Vec<f64>) {
        (self.freqs.clone(), self.zpos.clone())
    }
    fn apply_calibration(&self, samples: &mut Measures) -> KResult<()> {
        if let Some(offset) = self.offset {
            for sample in samples.iter_mut() {
                sample.2 += offset;
            }
        }
        Ok(())
    }
    fn freq_to_height(&self, _freq: f64) -> KResult<f64> {
        panic!("freq_to_height is not used by ldc1612")
    }
    fn height_to_freq(&self, _height: f64) -> KResult<f64> {
        panic!("height_to_freq is not used by ldc1612")
    }
}

// ---------------------------------------------------------------------------
// Comparison plumbing.
// ---------------------------------------------------------------------------

static LOG_BUF: std::sync::LazyLock<Arc<kos::sync::Mutex<Vec<String>>>> =
    std::sync::LazyLock::new(|| {
        let buf = Arc::new(kos::sync::Mutex::new(Vec::new()));
        let sink = buf.clone();
        klippy_api::log::set_sink(Box::new(move |rec| {
            sink.lock().push(format!("{}:{}", rec.level.name(), rec.message));
        }));
        buf
    });
static SERIAL: std::sync::LazyLock<kos::sync::Mutex<()>> =
    std::sync::LazyLock::new(|| kos::sync::Mutex::new(()));

/// Starts the oracle with the fakes loaded. The tests share the process-wide
/// log sink, so each one holds `SERIAL` for its whole run.
fn oracle() -> Option<(pyoracle::Oracle, kos::sync::MutexGuard<'static, ()>)> {
    let guard = SERIAL.lock();
    std::sync::LazyLock::force(&LOG_BUF);
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("ldc1612 oracle setup");
            Some((py, guard))
        }
        None => {
            println!("skipping ldc1612 oracle tests: KLIPPY_PYTHON is not set");
            None
        }
    }
}

fn clear_logs(py: &mut pyoracle::Oracle) {
    LOG_BUF.lock().clear();
    py.exec("LOG_LINES[:] = []").expect("clear python logs");
}

fn rust_logs() -> Value {
    strs(&LOG_BUF.lock())
}

fn strs(items: &[String]) -> Value {
    Value::List(items.iter().map(|s| Value::str(s)).collect())
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "extras.ldc1612".to_string(),
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

fn check_err<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    let exc = match expected {
        Ok(v) => panic!("{label}: oracle returned {v:?} but must raise"),
        Err(e) => e,
    };
    let err = match actual {
        Ok(_) => panic!("{label}: rust call succeeded but must fail with {}: {}", exc.class_name, exc.message),
        Err(e) => e,
    };
    assert_eq!(err.class_name(), exc.class_name.as_str(), "{label}: exception class");
    assert_eq!(err.message(), exc.message.as_str(), "{label}: exception message");
}

/// `Ok(None)` from an `exec`/`eval` that returned nothing, against a Rust
/// call that must succeed.
fn check_ok<T>(label: &str, expected: pyoracle::OracleResult, actual: KResult<T>) {
    if let Err(e) = &actual {
        panic!("{label}: rust call failed: {}: {}", e.class_name(), e.message());
    }
    check(label, expected, Value::Null);
}

fn to_value(v: &PyValue) -> Value {
    match v {
        PyValue::None => Value::Null,
        PyValue::Bool(b) => Value::Bool(*b),
        PyValue::Int(n) => Value::int(n.inner().to_string().parse().expect("small int")),
        PyValue::Float(f) => Value::float(*f),
        PyValue::Str(s) => Value::str(s),
        PyValue::Bytes(b) => Value::bytes(b),
        PyValue::List(xs) => Value::List(xs.iter().map(to_value).collect()),
        PyValue::Tuple(xs) => Value::Tuple(xs.iter().map(to_value).collect()),
        PyValue::Dict(map) => {
            Value::Dict(map.iter().map(|(k, val)| (to_value(k), to_value(val))).collect())
        }
        other => panic!("unexpected value {other:?}"),
    }
}

/// A Python string literal for `s`.
fn py_str(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            '\n' => out.push_str("\\n"),
            c => out.push(c),
        }
    }
    out.push('\'');
    out
}

fn py_float(v: f64) -> String {
    if v.is_nan() {
        "float('nan')".to_string()
    } else if v.is_infinite() {
        if v > 0. { "float('inf')".to_string() } else { "float('-inf')".to_string() }
    } else {
        format!("{v:?}")
    }
}

fn py_float_list(vs: &[f64]) -> String {
    format!("[{}]", vs.iter().map(|v| py_float(*v)).collect::<Vec<_>>().join(", "))
}

fn py_bytes_list(b: &[u8]) -> String {
    format!("[{}]", b.iter().map(|x| x.to_string()).collect::<Vec<_>>().join(", "))
}

fn queue_replies(py: &mut pyoracle::Oracle, world: &World, replies: &[(&str, Vec<u8>)]) {
    world.mcu.queue_replies(replies);
    let items: Vec<String> =
        replies.iter().map(|(s, r)| format!("({}, {})", py_str(s), py_bytes_list(r))).collect();
    py.exec(&format!("w.mcu.i2c_replies = [{}]", items.join(", "))).expect("queue i2c replies");
}

fn set_status(py: &mut pyoracle::Oracle, world: &World, status: [i64; 5]) {
    let [clock, next_sequence, buffered, overflows, query_ticks] = status;
    world.mcu.set_status(clock, next_sequence, buffered, overflows, query_ticks);
    py.exec(&format!(
        "w.mcu.status = {{'clock': {clock}, 'next_sequence': {next_sequence}, 'buffered': {buffered}, 'possible_overflows': {overflows}, 'query_ticks': {query_ticks}}}"
    ))
    .expect("set status");
}

fn pack_be(vals: &[u32]) -> Vec<u8> {
    vals.iter().flat_map(|v| v.to_be_bytes()).collect()
}

fn feed_bulk(py: &mut pyoracle::Oracle, world: &World, sequence: i64, vals: &[u32]) {
    world.mcu.feed_bulk(sequence, pack_be(vals));
    let args: Vec<String> = vals.iter().map(|v| v.to_string()).collect();
    py.exec(&format!(
        "w.mcu.serial_cb({{'sequence': {sequence}, 'data': struct.pack('>{}I', {})}})",
        vals.len(),
        args.join(", ")
    ))
    .expect("feed bulk data");
}

// ---------------------------------------------------------------------------
// World: one identically configured sensor on each side.
// ---------------------------------------------------------------------------

const SECTION: &str = "probe_eddy_current z1";

#[derive(Default)]
struct Opts {
    options: Vec<(&'static str, &'static str)>,
    fileoutput: bool,
    enums: Vec<(&'static str, i64)>,
    cal: Option<(Vec<f64>, Vec<f64>, Option<f64>)>,
    intb_wrong_mcu: bool,
}

struct World {
    printer: Arc<FakePrinter>,
    mcu: Arc<FakeMcu>,
    gcode: Arc<FakeGCode>,
    toolhead: Arc<FakeToolhead>,
    cfl: Arc<FakeConfigFile>,
    webhooks: Arc<FakeWebhooks>,
    pins: Arc<FakePins>,
    tracking: Arc<GCell<IndexMap<(String, String), PyValue>>>,
    ldc: KResult<Arc<LDC1612>>,
}

impl World {
    fn ldc(&self) -> &Arc<LDC1612> {
        match &self.ldc {
            Ok(ldc) => ldc,
            Err(e) => panic!("rust LDC1612::new failed: {}: {}", e.class_name(), e.message()),
        }
    }
    fn access_tracking(&self) -> Value {
        let items: Vec<String> = self
            .tracking
            .borrow()
            .iter()
            .map(|((section, option), v)| format!("{section}|{option}|{}", v.repr()))
            .collect();
        strs(&items)
    }
}

/// Builds both sides; returns the Python construction result with the world.
fn build(py: &mut pyoracle::Oracle, opts: &Opts) -> (pyoracle::OracleResult, World) {
    let mut text = format!("[{SECTION}]\n");
    for (k, v) in &opts.options {
        text.push_str(&format!("{k}: {v}\n"));
    }
    let py_enums = if opts.enums.is_empty() {
        "{}".to_string()
    } else {
        let inner: Vec<String> = opts.enums.iter().map(|(k, v)| format!("{}: {v}", py_str(k))).collect();
        format!("{{'ldc1612_error:': {{{}}}}}", inner.join(", "))
    };
    let py_cal = match &opts.cal {
        Some((freqs, zpos, offset)) => format!(
            "FakeCal({}, {}, {})",
            py_float_list(freqs),
            py_float_list(zpos),
            offset.map_or("None".to_string(), py_float)
        ),
        None => "None".to_string(),
    };
    let expected = py.exec(&format!(
        "build({}, {}, {}, {}, {}, {})",
        py_str(SECTION),
        py_str(&text),
        py_bool(opts.fileoutput),
        py_enums,
        py_cal,
        py_bool(opts.intb_wrong_mcu)
    ));

    let mut enums = IndexMap::new();
    if !opts.enums.is_empty() {
        let inner: IndexMap<String, i64> = opts.enums.iter().map(|(k, v)| (k.to_string(), *v)).collect();
        enums.insert("ldc1612_error:".to_string(), inner);
    }
    let printer = Arc::new(FakePrinter::default());
    let printer_ref: Printer = printer.clone();
    let mcu = FakeMcu::new(printer_ref.clone(), opts.fileoutput, enums.clone());
    let other_mcu = FakeMcu::new(printer_ref.clone(), opts.fileoutput, enums);
    let gcode = Arc::new(FakeGCode::default());
    let toolhead = Arc::new(FakeToolhead::default());
    let cfl = Arc::new(FakeConfigFile::default());
    let webhooks = Arc::new(FakeWebhooks::default());
    let pin_chip: klippy_api::mcu::Mcu = if opts.intb_wrong_mcu { other_mcu } else { mcu.clone() };
    let pins = Arc::new(FakePins { chip: pin_chip, looked: GCell::new(Vec::new()) });
    printer.put("mcu", mcu.clone());
    printer.put("gcode", gcode.clone());
    printer.put("toolhead", toolhead.clone());
    printer.put("configfile", cfl.clone());
    printer.put("pins", pins.clone());
    printer.put("webhooks", webhooks.clone());
    let fileconfig = ConfigFileReader::build_fileconfig(&text, "printer.cfg").expect("parse config text");
    let tracking = Arc::new(GCell::new(IndexMap::new()));
    let config: Config = Arc::new(ConfigWrapper::new(
        printer_ref,
        Arc::new(fileconfig),
        tracking.clone(),
        SECTION.to_string(),
    ));
    let calibration = opts.cal.as_ref().map(|(freqs, zpos, offset)| {
        Arc::new(FakeCal { freqs: freqs.clone(), zpos: zpos.clone(), offset: *offset })
            as Arc<dyn EddyCalibrationApi>
    });
    let ldc = LDC1612::new(&config, calibration);
    let world = World { printer, mcu, gcode, toolhead, cfl, webhooks, pins, tracking, ldc };
    (expected, world)
}

fn build_ok(py: &mut pyoracle::Oracle, opts: &Opts) -> World {
    let (expected, world) = build(py, opts);
    if let Err(e) = expected {
        panic!("python build failed: {}: {}", e.class_name, e.message);
    }
    world.ldc();
    world
}

fn run_callbacks(py: &mut pyoracle::Oracle, world: &World) {
    let rust = world.mcu.run_callbacks();
    check_ok("config callbacks", py.exec("w.mcu.run_callbacks()"), rust);
}

/// Construction results and every recorded side effect.
fn compare_construction(py: &mut pyoracle::Oracle, world: &World, tag: &str) {
    let ldc = world.ldc();
    let label = |s: &str| format!("{tag}: {s}");
    check(&label("config cmds"), py.eval("w.mcu.config_cmds"), world.mcu.config_cmds());
    check(&label("lookups"), py.eval("w.mcu.lookups"), world.mcu.lookups());
    check(&label("gcode mux"), py.eval("w.gcode.mux"), strs(&world.gcode.mux.borrow()));
    check(&label("webhooks mux"), py.eval("w.webhooks.mux"), strs(&world.webhooks.mux.borrow()));
    check(&label("printer events"), py.eval("w.p.events"), strs(&world.printer.events.borrow()));
    check(&label("access tracking"), py.eval("access_tracking()"), world.access_tracking());
    check(&label("warnings"), py.eval("w.cfl.warnings"), strs(&world.cfl.warnings.borrow()));
    check(&label("oid"), py.eval("w.ldc.oid"), Value::int(ldc.oid()));
    check(&label("name"), py.eval("w.ldc.name"), Value::str(ldc.name()));
    check(&label("clock_freq"), py.eval("w.ldc.clock_freq"), Value::int(ldc.clock_freq()));
    check(&label("sensor_div"), py.eval("w.ldc.sensor_div"), Value::int(ldc.sensor_div()));
    check(&label("freq_conv"), py.eval("w.ldc.freq_conv"), Value::float(ldc.freq_conv()));
    check(
        &label("drive current"),
        py.eval("w.ldc.dccal.get_drive_current()"),
        Value::int(ldc.dccal().get_drive_current()),
    );
    check(
        &label("samples per second"),
        py.eval("w.ldc.get_samples_per_second()"),
        Value::int(ldc.get_samples_per_second()),
    );
    check(
        &label("raw to frequency"),
        py.eval("w.ldc.convert_raw_to_frequency(12345)"),
        Value::float(ldc.convert_raw_to_frequency(12345.0)),
    );
    let raw = ldc.convert_frequency_to_raw(7654321.25).expect("convert_frequency_to_raw");
    check(&label("frequency to raw"), py.eval("w.ldc.convert_frequency_to_raw(7654321.25)"), Value::int(raw));
    check(&label("batch interval"), py.eval("w.ldc.batch_bulk.batch_interval"), Value::float(ldc.batch_bulk().batch_interval()));
    check(&label("bytes per sample"), py.eval("w.ldc.ffreader.bytes_per_sample"), Value::int(ldc.ffreader().bytes_per_sample));
    check(&label("samples per block"), py.eval("w.ldc.ffreader.samples_per_block"), Value::int(ldc.ffreader().samples_per_block));
    check(&label("get_mcu"), py.eval("w.ldc.get_mcu() is w.mcu"), Value::Bool(Arc::ptr_eq(&ldc.get_mcu(), &(world.mcu.clone() as klippy_api::mcu::Mcu))));
}

/// Runs the config callbacks, then `_start_measurements` on both sides with
/// valid chip ids queued.
fn start_both(py: &mut pyoracle::Oracle, world: &World) {
    run_callbacks(py, world);
    queue_replies(py, world, &[("SUCCESS", vec![0x54, 0x49]), ("SUCCESS", vec![0x30, 0x55])]);
    set_status(py, world, [100000, 10, 32, 0, 100]);
    let rust = world.ldc().start_measurements();
    check_ok("start measurements", py.exec("w.ldc._start_measurements()"), rust);
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn module_factory_matches_reference_entry_points() {
    let Some((mut py, _guard)) = oracle() else { return };
    let factory = module_factory();
    check(
        "module entry points",
        py.eval("[hasattr(ldc1612, n) for n in ('load_config', 'load_config_prefix', 'add_early_printer_objects', 'add_printer_objects')]"),
        Value::List(vec![
            Value::Bool(factory.load_config.is_some()),
            Value::Bool(factory.load_config_prefix.is_some()),
            Value::Bool(factory.add_early_printer_objects.is_some()),
            Value::Bool(factory.add_printer_objects.is_some()),
        ]),
    );
}

#[test]
fn defaults_construction() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    compare_construction(&mut py, &world, "defaults");
    run_callbacks(&mut py, &world);
    compare_construction(&mut py, &world, "defaults after build_config");
}

#[test]
fn custom_options_construction_and_start() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts {
        options: vec![
            ("frequency", "8000000"),
            ("max_sensor_hz", "9000000"),
            ("reg_drive_current", "20"),
            ("i2c_address", "43"),
            ("i2c_speed", "100000"),
        ],
        ..Opts::default()
    };
    let world = build_ok(&mut py, &opts);
    compare_construction(&mut py, &world, "custom");
    start_both(&mut py, &world);
    check("custom start sends", py.eval("w.mcu.sends"), world.mcu.sends());
}

#[test]
fn build_config_sensor_errors() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts { enums: vec![("wd_timeout", 3), ("bus_error", 7)], ..Opts::default() };
    let world = build_ok(&mut py, &opts);
    for code in [3, 7, 99] {
        check(
            &format!("sensor error {code} before build_config"),
            py.eval(&format!("w.ldc.lookup_sensor_error({code})")),
            Value::str(&world.ldc().lookup_sensor_error(code)),
        );
    }
    run_callbacks(&mut py, &world);
    check("config cmds after build_config", py.eval("w.mcu.config_cmds"), world.mcu.config_cmds());
    check("lookups after build_config", py.eval("w.mcu.lookups"), world.mcu.lookups());
    for code in [3, 7, 99] {
        check(
            &format!("sensor error {code}"),
            py.eval(&format!("w.ldc.lookup_sensor_error({code})")),
            Value::str(&world.ldc().lookup_sensor_error(code)),
        );
    }
}

#[test]
fn intb_pin_config() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts { options: vec![("intb_pin", "PC0")], ..Opts::default() };
    let world = build_ok(&mut py, &opts);
    compare_construction(&mut py, &world, "intb_pin");
    check("intb_pin lookup", py.eval("w.pins.looked"), strs(&world.pins.looked.borrow()));
}

#[test]
fn intb_pin_on_other_mcu_errors() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts { options: vec![("intb_pin", "PC0")], intb_wrong_mcu: true, ..Opts::default() };
    let (expected, world) = build(&mut py, &opts);
    let World { ldc, .. } = world;
    check_err("intb_pin on other mcu", expected, ldc);
}

#[test]
fn config_errors() {
    let Some((mut py, _guard)) = oracle() else { return };
    let cases: Vec<(&str, Vec<(&'static str, &'static str)>)> = vec![
        ("frequency below minimum", vec![("frequency", "100")]),
        ("frequency above maximum", vec![("frequency", "50000000")]),
        ("frequency not an int", vec![("frequency", "abc")]),
        ("max_sensor_hz below minimum", vec![("max_sensor_hz", "100.0")]),
        ("max_sensor_hz above maximum", vec![("max_sensor_hz", "30000000")]),
        ("reg_drive_current above maximum", vec![("reg_drive_current", "99")]),
        ("reg_drive_current below minimum", vec![("reg_drive_current", "-1")]),
        ("i2c_speed below minimum", vec![("i2c_speed", "1000")]),
        (
            "first failing option wins",
            vec![("frequency", "1"), ("i2c_speed", "1000"), ("reg_drive_current", "99")],
        ),
        ("frequency before max_sensor_hz", vec![("max_sensor_hz", "1"), ("frequency", "1")]),
    ];
    for (label, options) in cases {
        let (expected, world) = build(&mut py, &Opts { options, ..Opts::default() });
        check(&format!("{label}: config cmds"), py.eval("w.mcu.config_cmds"), world.mcu.config_cmds());
        let World { ldc, .. } = world;
        check_err(label, expected, ldc);
    }
}

#[test]
fn calibration_max_sensor_hz_warning() {
    let Some((mut py, _guard)) = oracle() else { return };
    let cases: Vec<(&str, Vec<f64>)> = vec![
        ("above max_sensor_hz", vec![1_000_000.0, 6_000_000.5, 5_500_000.0]),
        ("equal to max_sensor_hz", vec![5_000_000.0]),
        ("below max_sensor_hz", vec![4_000_000.0, 3_000_000.0]),
        ("empty calibration", vec![]),
    ];
    for (label, freqs) in cases {
        let zpos = vec![0.5; freqs.len()];
        let world = build_ok(&mut py, &Opts { cal: Some((freqs, zpos, None)), ..Opts::default() });
        check(label, py.eval("w.cfl.warnings"), strs(&world.cfl.warnings.borrow()));
    }
}

#[test]
fn setup_trigger_analog_config_cmd() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    let rust = world.ldc().setup_trigger_analog(7);
    check_ok("setup_trigger_analog", py.exec("w.ldc.setup_trigger_analog(7)"), rust);
    check("trigger analog config cmds", py.eval("w.mcu.config_cmds"), world.mcu.config_cmds());
}

#[test]
fn start_measurements_register_program() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    clear_logs(&mut py);
    start_both(&mut py, &world);
    check("start sends", py.eval("w.mcu.sends"), world.mcu.sends());
    check("start logs", py.eval("LOG_LINES"), rust_logs());
    check("start error count", py.eval("w.ldc.last_error_count"), Value::int(world.ldc().last_error_count()));
}

#[test]
fn start_measurements_invalid_id() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    run_callbacks(&mut py, &world);
    queue_replies(&mut py, &world, &[("SUCCESS", vec![0x12, 0x34]), ("SUCCESS", vec![0xab, 0xcd])]);
    let rust = world.ldc().start_measurements();
    check_err("invalid id", py.exec("w.ldc._start_measurements()"), rust);
    check("invalid id sends", py.eval("w.mcu.sends"), world.mcu.sends());
}

#[test]
fn start_measurements_fileoutput_skips_id_check() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts { fileoutput: true, ..Opts::default() });
    clear_logs(&mut py);
    start_both(&mut py, &world);
    check("fileoutput lookups", py.eval("w.mcu.lookups"), world.mcu.lookups());
    check("fileoutput sends", py.eval("w.mcu.sends"), world.mcu.sends());
    check("fileoutput logs", py.eval("LOG_LINES"), rust_logs());
    check("fileoutput read_reg", py.eval("w.ldc.read_reg(0x7e)"), Value::int(world.ldc().read_reg(0x7e).expect("read_reg")));
}

#[test]
fn read_reg_i2c_failures() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    run_callbacks(&mut py, &world);
    queue_replies(&mut py, &world, &[("NACK", vec![])]);
    let rust = world.ldc().read_reg(0x7f);
    check_err("read_reg bus error", py.eval("w.ldc.read_reg(0x7f)"), rust);
    check("read_reg bus error shutdown", py.eval("w.p.shutdowns"), strs(&world.printer.shutdowns.borrow()));
    queue_replies(&mut py, &world, &[("SUCCESS", vec![0x54])]);
    let rust = world.ldc().read_reg(0x7e);
    check_err("read_reg short response", py.eval("w.ldc.read_reg(0x7e)"), rust);
    queue_replies(&mut py, &world, &[("SUCCESS", vec![0x02, 0x80, 0x11])]);
    let rust = world.ldc().read_reg(0x1e).expect("read_reg");
    check("read_reg long response", py.eval("w.ldc.read_reg(0x1e)"), Value::int(rust));
    check("read_reg sends", py.eval("w.mcu.sends"), world.mcu.sends());
}

#[test]
fn finish_before_build_config_errors() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    let rust = world.ldc().finish_measurements();
    check_err("finish before build_config", py.exec("w.ldc._finish_measurements()"), rust);
}

#[test]
fn finish_measurements_halts() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    start_both(&mut py, &world);
    clear_logs(&mut py);
    let rust = world.ldc().finish_measurements();
    check_ok("finish measurements", py.exec("w.ldc._finish_measurements()"), rust);
    check("finish sends", py.eval("w.mcu.sends"), world.mcu.sends());
    check("finish logs", py.eval("LOG_LINES"), rust_logs());
}

#[test]
fn process_batch_samples_and_calibration() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts { cal: Some((vec![1.0], vec![0.0], Some(2.5))), ..Opts::default() };
    let world = build_ok(&mut py, &opts);
    start_both(&mut py, &world);
    set_status(&mut py, &world, [300000, 11, 4, 0, 100]);
    feed_bulk(&mut py, &world, 9, &[1000, 2_000_000, 0x03ff_fff0, 12345678]);
    feed_bulk(&mut py, &world, 10, &[22_000_000]);
    clear_logs(&mut py);
    let rust = to_value(&world.ldc().process_batch(1.5).expect("rust process_batch"));
    check("process_batch samples", py.eval("w.ldc._process_batch(1.5)"), rust);
    check("process_batch logs", py.eval("LOG_LINES"), rust_logs());
    check("process_batch sends", py.eval("w.mcu.sends"), world.mcu.sends());
}

#[test]
fn process_batch_empty() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    start_both(&mut py, &world);
    set_status(&mut py, &world, [300000, 11, 0, 0, 100]);
    let rust = to_value(&world.ldc().process_batch(1.5).expect("rust process_batch"));
    check("process_batch empty", py.eval("w.ldc._process_batch(1.5)"), rust);
    check("process_batch empty sends", py.eval("w.mcu.sends"), world.mcu.sends());
}

#[test]
fn process_batch_error_classes() {
    let Some((mut py, _guard)) = oracle() else { return };
    let opts = Opts { enums: vec![("wd_timeout", 3)], ..Opts::default() };
    let world = build_ok(&mut py, &opts);
    start_both(&mut py, &world);
    set_status(&mut py, &world, [300000, 11, 0, 3, 100]);
    let samples: [u32; 12] = [
        0x0000_0005,
        0,
        0x8000_0001,
        0x4000_1234,
        0x2000_0002,
        0x1000_0001,
        0x03ff_ffff,
        0x0400_0000,
        0xffff_0003,
        0x3000_0003,
        0xffff_1234,
        0xf000_0000,
    ];
    feed_bulk(&mut py, &world, 9, &samples);
    clear_logs(&mut py);
    let rust = to_value(&world.ldc().process_batch(1.5).expect("rust process_batch"));
    check("process_batch error samples", py.eval("w.ldc._process_batch(1.5)"), rust);
    check("process_batch error logs", py.eval("LOG_LINES"), rust_logs());
    check("process_batch error count", py.eval("w.ldc.last_error_count"), Value::int(world.ldc().last_error_count()));
    // Only encoded errors: every sample is dropped and the batch is empty.
    set_status(&mut py, &world, [500000, 12, 0, 3, 100]);
    feed_bulk(&mut py, &world, 11, &[0xffff_0003, 0xffff_0003]);
    clear_logs(&mut py);
    let rust = to_value(&world.ldc().process_batch(2.0).expect("rust process_batch"));
    check("process_batch only encoded errors", py.eval("w.ldc._process_batch(2.0)"), rust);
    check("process_batch only encoded errors logs", py.eval("LOG_LINES"), rust_logs());
    check("process_batch error count total", py.eval("w.ldc.last_error_count"), Value::int(world.ldc().last_error_count()));
}

#[test]
fn ldc_calibrate_drive_current_command() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    run_callbacks(&mut py, &world);
    queue_replies(
        &mut py,
        &world,
        &[
            ("SUCCESS", vec![0x54, 0x49]),
            ("SUCCESS", vec![0x30, 0x55]),
            ("SUCCESS", vec![0x16, 0x01]),
            ("SUCCESS", vec![0x02, 0x80]),
        ],
    );
    set_status(&mut py, &world, [100000, 10, 32, 0, 100]);
    clear_logs(&mut py);
    let gcmd = Arc::new(FakeGCmd::default());
    let gcmd_ref: GCmd = gcmd.clone();
    let rust = world.gcode.run_handler(0, &gcmd_ref);
    check_ok("calibrate", py.exec("py_gcmd = FakeGcmd()\nw.gcode.handlers[0](py_gcmd)"), rust);
    check("calibrate responses", py.eval("py_gcmd.responses"), strs(&gcmd.responses.borrow()));
    check("calibrate toolhead", py.eval("w.toolhead.calls"), strs(&world.toolhead.calls.borrow()));
    check("calibrate configfile set", py.eval("w.cfl.sets"), strs(&world.cfl.sets.borrow()));
    check("calibrate sends", py.eval("w.mcu.sends"), world.mcu.sends());
    check("calibrate logs", py.eval("LOG_LINES"), rust_logs());
    check("calibrate started", py.eval("w.ldc.batch_bulk.is_started"), Value::Bool(world.ldc().batch_bulk().is_started()));
    // The next batch reaches the finished client, which unregisters and
    // stops the measurements.
    set_status(&mut py, &world, [300000, 11, 0, 0, 100]);
    feed_bulk(&mut py, &world, 9, &[1000, 2000]);
    clear_logs(&mut py);
    let rust = world.ldc().batch_bulk()._proc_batch(5.0).expect("rust _proc_batch");
    check("calibrate batch waketime", py.eval("w.ldc.batch_bulk._proc_batch(5.0)"), Value::float(rust));
    check("calibrate stopped", py.eval("w.ldc.batch_bulk.is_started"), Value::Bool(world.ldc().batch_bulk().is_started()));
    check("calibrate stop sends", py.eval("w.mcu.sends"), world.mcu.sends());
    check("calibrate stop logs", py.eval("LOG_LINES"), rust_logs());
}

#[test]
fn ldc_calibrate_drive_current_invalid_id() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    run_callbacks(&mut py, &world);
    queue_replies(&mut py, &world, &[("SUCCESS", vec![0, 0]), ("SUCCESS", vec![0, 0])]);
    clear_logs(&mut py);
    let gcmd = Arc::new(FakeGCmd::default());
    let gcmd_ref: GCmd = gcmd.clone();
    let rust = world.gcode.run_handler(0, &gcmd_ref);
    check_err("calibrate invalid id", py.exec("py_gcmd = FakeGcmd()\nw.gcode.handlers[0](py_gcmd)"), rust);
    check("calibrate invalid id logs", py.eval("LOG_LINES"), rust_logs());
    check("calibrate invalid id responses", py.eval("py_gcmd.responses"), strs(&gcmd.responses.borrow()));
    check("calibrate invalid id toolhead", py.eval("w.toolhead.calls"), strs(&world.toolhead.calls.borrow()));
    check("calibrate invalid id started", py.eval("w.ldc.batch_bulk.is_started"), Value::Bool(world.ldc().batch_bulk().is_started()));
}

#[test]
fn convert_frequency_to_raw_errors() {
    let Some((mut py, _guard)) = oracle() else { return };
    let world = build_ok(&mut py, &Opts::default());
    for v in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        check_err(
            &format!("convert_frequency_to_raw({})", py_float(v)),
            py.eval(&format!("w.ldc.convert_frequency_to_raw({})", py_float(v))),
            world.ldc().convert_frequency_to_raw(v),
        );
    }
    for v in [0.0, -1234.5, 1e9] {
        check(
            &format!("convert_frequency_to_raw({})", py_float(v)),
            py.eval(&format!("w.ldc.convert_frequency_to_raw({})", py_float(v))),
            Value::int(world.ldc().convert_frequency_to_raw(v).expect("finite")),
        );
    }
}
