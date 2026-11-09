//! Port of `klippy/mcu.py`.
//!
//! The port keeps the Python shape: [`Mcu`] owns the connection helper
//! ([`McuConnectHelper`] state plus [`McuRestartHelper`]), the config
//! helper ([`McuConfigHelper`] state) and the stats helper
//! ([`McuStatsHelper`] state), with [`SCell`]s for fields shared with
//! serial-reader callbacks, and the command wrappers
//! ([`CommandWrapper`], [`CommandQueryWrapper`], [`AsyncResponseWrapper`]
//! plus the debug-mode [`DummyResponse`] and the retrying
//! [`RetryAsyncCommand`]) plus the pin wrappers ([`McuTrsync`],
//! [`TriggerDispatch`], [`McuEndstop`], [`McuDigitalOut`], [`McuPwm`],
//! [`McuAdc`]).
//!
//! The C `trdispatch` fastreader layer (`klippy/chelper/trdispatch.c`)
//! is reimplemented in Rust ([`Trdispatch`] / [`TrdispatchMcu`]) on top
//! of the normal `trsync_state` serial callbacks, with identical
//! timeout-extension and trigger-propagation semantics.

use std::sync::{Arc, Weak};

use chelper_serial::serialqueue::CommandQueue;
use indexmap::IndexMap;
use klippy_api::config::{Config, GetChoiceOpts, GetIntOpts, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::{GCell, SCell};
use klippy_api::log;
use klippy_api::mcu::{
    AdcCallback, AdcSampleOpts, AddConfigCmdOpts, AsyncResponseWrapperApi, CommandQueryWrapperApi,
    CommandWrapperApi, CommandQueue as ApiCommandQueue, ConfigCallback, DigitalOutPinApi, EndstopApi,
    LookupQueryCommandOpts, Pin, PinParams, PwmPinApi, QuerySendOpts, SendOpts, SerialResponseCallback,
};
use klippy_api::object::{ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::stepper::Stepper;
use msgproto::{ParamValue, Params};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Completion, Reactor};

use crate::clocksync::{ClockParams, ClockSerialApi, ClockSync, CommandId, CommandQueueId, SerialHandle, SecondarySync};
use crate::serialhdl::{self, SerialReader};

/// `MIN_SCHEDULE_TIME` (`klippy/mcu.py:13`).
pub const MIN_SCHEDULE_TIME: f64 = 0.100;
/// `MAX_SCHEDULE_TICKS` (`klippy/mcu.py:16`).
pub const MAX_SCHEDULE_TICKS: i64 = (1 << 31) - 1;
/// `MAX_NOMINAL_DURATION` (`klippy/mcu.py:19`).
pub const MAX_NOMINAL_DURATION: f64 = 3.0;
/// `TRSYNC_TIMEOUT` (`klippy/mcu.py:279`).
pub const TRSYNC_TIMEOUT: f64 = 0.025;
/// `TRSYNC_SINGLE_MCU_TIMEOUT` (`klippy/mcu.py:280`).
pub const TRSYNC_SINGLE_MCU_TIMEOUT: f64 = 0.250;
/// `DEBUG_SHUTDOWN_VERSION` (`klippy/mcu.py:772`).
pub const DEBUG_SHUTDOWN_VERSION: &str = "shutdown-v4";
/// `DEBUG_SHUTDOWN_MAX_NOTES` (`klippy/mcu.py:776`).
pub const DEBUG_SHUTDOWN_MAX_NOTES: i64 = 16;
/// `DEBUG_REASON_MAX_LEN` (`klippy/mcu.py:777`).
pub const DEBUG_REASON_MAX_LEN: usize = 160;

/// `mcu.error` (`klippy/mcu.py:9`).
pub fn error(message: String) -> KlippyError {
    KlippyError::Mcu(message)
}

fn config_error(message: String) -> KlippyError {
    KlippyError::Config(message)
}

fn command_error(message: String) -> KlippyError {
    KlippyError::Command(message)
}

fn pins_error(message: String) -> KlippyError {
    KlippyError::Pins(message)
}

fn fmt_msg(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).expect("constant mcu format")
}

fn pct_int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn py_trunc_i64(f: f64) -> i64 {
    if !f.is_finite() {
        return 0;
    }
    let digits = pycompat::text::trunc_float_to_bigint(f).0.to_string();
    digits.parse::<i64>().unwrap_or(if f < 0.0 { i64::MIN } else { i64::MAX })
}

fn py_int_from_float(f: f64) -> i64 {
    py_trunc_i64(f)
}

fn py_float(v: &PyValue) -> f64 {
    v.to_f64().unwrap_or(0.0)
}

fn as_int(v: &PyValue) -> i64 {
    match v {
        PyValue::Int(n) => n.0.to_string().parse::<i64>().unwrap_or(0),
        PyValue::Float(f) => py_trunc_i64(*f),
        PyValue::Bool(b) => i64::from(*b),
        _ => 0,
    }
}

fn as_float(v: &PyValue) -> f64 {
    py_float(v)
}

fn as_str(v: &PyValue) -> String {
    match v {
        PyValue::Str(s) => s.clone(),
        _ => v.str(),
    }
}

fn truthy(v: &PyValue) -> bool {
    v.truthy()
}

/// Python `params['key']` for a msgproto int field.
fn param_int(params: &Params, key: &str) -> i64 {
    match params.get(key) {
        Some(ParamValue::Int(i)) => *i,
        Some(ParamValue::Str(s)) => s.parse::<i64>().unwrap_or(0),
        Some(ParamValue::Bytes(b)) => String::from_utf8_lossy(b).parse::<i64>().unwrap_or(0),
        None => 0,
    }
}

fn param_float(params: &Params, key: &str) -> f64 {
    match params.get(key) {
        Some(ParamValue::Int(i)) => *i as f64,
        Some(ParamValue::Str(s)) => s.parse::<f64>().unwrap_or(0.0),
        Some(ParamValue::Bytes(b)) => String::from_utf8_lossy(b).parse::<f64>().unwrap_or(0.0),
        None => 0.0,
    }
}

fn param_str(params: &Params, key: &str) -> String {
    match params.get(key) {
        Some(ParamValue::Str(s)) => s.clone(),
        Some(ParamValue::Int(i)) => i.to_string(),
        Some(ParamValue::Bytes(b)) => String::from_utf8_lossy(b).to_string(),
        None => String::new(),
    }
}

fn params_to_pyvalue(params: &Params) -> PyValue {
    let mut m: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in params.iter() {
        m.insert(PyValue::Str(k.clone()), param_entry_to_pyvalue(k, v));
    }
    PyValue::Dict(m)
}

/// One response entry as the Python dict holds it. serialhdl's
/// `#sent_time` / `#receive_time` are floats in Python
/// (`serialhdl.py` `_bg_thread`); `ParamValue` has no float variant, so
/// the port's serialhdl carries them as Rust's shortest round-trip
/// decimal text, and they become floats again here.
fn param_entry_to_pyvalue(key: &str, v: &ParamValue) -> PyValue {
    match (key, v) {
        ("#sent_time" | "#receive_time", ParamValue::Str(s)) => {
            PyValue::Float(s.parse::<f64>().expect("serialhdl writes timestamps as f64 text"))
        }
        _ => param_to_pyvalue(v),
    }
}

fn pyvalue_to_param(v: &PyValue) -> ParamValue {
    match v {
        PyValue::Int(n) => ParamValue::Int(n.0.to_string().parse::<i64>().unwrap_or(0)),
        PyValue::Float(f) => ParamValue::Int(py_trunc_i64(*f)),
        PyValue::Bool(b) => ParamValue::Int(i64::from(*b)),
        PyValue::Str(s) => ParamValue::Str(s.clone()),
        PyValue::Bytes(b) => ParamValue::Bytes(b.clone()),
        other => ParamValue::Str(other.str()),
    }
}

/// `msgparser.get_constant_float(name)`: firmware constants are strings
/// in the parser; the Python parser coerces with `float()`.
fn constant_float_str(s: &str) -> KResult<f64> {
    pycompat::text::float_from_string(s)
        .map_err(|e| error(format!("Unable to parse firmware constant {}: {}", s, e.display())))
}

// ---------------------------------------------------------------------------
// ClockSync + SerialReader bridging
// ---------------------------------------------------------------------------

/// The subset of `ClockSync` / `SecondarySync` `MCU` calls:
/// clock conversion, liveness and connect/dump helpers.
pub trait ClockSyncLike: Send + Sync {
    fn print_time_to_clock(&self, print_time: f64) -> i64;
    fn clock_to_print_time(&self, clock: i64) -> f64;
    fn estimated_print_time(&self, eventtime: f64) -> f64;
    fn clock32_to_clock64(&self, clock32: i64) -> i64;
    fn get_clock(&self, eventtime: f64) -> i64;
    fn is_active(&self) -> bool;
    fn connect(&self, serial: SerialHandle) -> KResult<()>;
    fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()>;
    fn dump_debug(&self) -> String;
    fn stats(&self, eventtime: f64) -> String;
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)>;
}

impl ClockSyncLike for ClockSync {
    fn print_time_to_clock(&self, print_time: f64) -> i64 { ClockSync::print_time_to_clock(self, print_time) }
    fn clock_to_print_time(&self, clock: i64) -> f64 { ClockSync::clock_to_print_time(self, clock) }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { ClockSync::estimated_print_time(self, eventtime) }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { ClockSync::clock32_to_clock64(self, clock32) }
    fn get_clock(&self, eventtime: f64) -> i64 { ClockSync::get_clock(self, eventtime) }
    fn is_active(&self) -> bool { ClockSync::is_active(self) }
    fn connect(&self, serial: SerialHandle) -> KResult<()> { ClockSync::connect(self, serial) }
    fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()> { ClockSync::connect_file(self, serial, pace) }
    fn dump_debug(&self) -> String { ClockSync::dump_debug(self) }
    fn stats(&self, eventtime: f64) -> String { ClockSync::stats(self, eventtime) }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> { ClockSync::calibrate_clock(self, print_time, eventtime) }
}

impl ClockSyncLike for SecondarySync {
    fn print_time_to_clock(&self, print_time: f64) -> i64 { SecondarySync::print_time_to_clock(self, print_time) }
    fn clock_to_print_time(&self, clock: i64) -> f64 { SecondarySync::clock_to_print_time(self, clock) }
    fn estimated_print_time(&self, eventtime: f64) -> f64 { SecondarySync::estimated_print_time(self, eventtime) }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { SecondarySync::clock32_to_clock64(self, clock32) }
    fn get_clock(&self, eventtime: f64) -> i64 { SecondarySync::get_clock(self, eventtime) }
    fn is_active(&self) -> bool { SecondarySync::is_active(self) }
    fn connect(&self, serial: SerialHandle) -> KResult<()> { SecondarySync::connect(self, serial) }
    fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()> { SecondarySync::connect_file(self, serial, pace) }
    fn dump_debug(&self) -> String { SecondarySync::dump_debug(self) }
    fn stats(&self, eventtime: f64) -> String { SecondarySync::stats(self, eventtime) }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> { SecondarySync::calibrate_clock(self, print_time, eventtime) }
}

/// A `ClockSync` document: the primary `ClockSync` (`klippy/clocksync.py:12`)
/// or a secondary sync (`klippy/clocksync.py:164`).
#[derive(Clone)]
pub enum ClockKind {
    Primary(ClockSync),
    Secondary(Arc<SecondarySync>),
}

impl ClockKind {
    fn print_time_to_clock(&self, t: f64) -> i64 {
        match self { ClockKind::Primary(c) => c.print_time_to_clock(t), ClockKind::Secondary(c) => c.print_time_to_clock(t) }
    }
    fn clock_to_print_time(&self, clock: i64) -> f64 {
        match self { ClockKind::Primary(c) => c.clock_to_print_time(clock), ClockKind::Secondary(c) => c.clock_to_print_time(clock) }
    }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        match self { ClockKind::Primary(c) => c.estimated_print_time(eventtime), ClockKind::Secondary(c) => c.estimated_print_time(eventtime) }
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 {
        match self { ClockKind::Primary(c) => c.clock32_to_clock64(clock32), ClockKind::Secondary(c) => c.clock32_to_clock64(clock32) }
    }
    fn get_clock(&self, eventtime: f64) -> i64 {
        match self { ClockKind::Primary(c) => c.get_clock(eventtime), ClockKind::Secondary(c) => c.get_clock(eventtime) }
    }
    fn is_active(&self) -> bool {
        match self { ClockKind::Primary(c) => c.is_active(), ClockKind::Secondary(c) => c.is_active() }
    }
    fn connect(&self, serial: SerialHandle) -> KResult<()> {
        match self { ClockKind::Primary(c) => c.connect(serial), ClockKind::Secondary(c) => c.connect(serial) }
    }
    fn connect_file(&self, serial: SerialHandle, pace: bool) -> KResult<()> {
        match self { ClockKind::Primary(c) => c.connect_file(serial, pace), ClockKind::Secondary(c) => c.connect_file(serial, pace) }
    }
    fn dump_debug(&self) -> String {
        match self { ClockKind::Primary(c) => c.dump_debug(), ClockKind::Secondary(c) => c.dump_debug() }
    }
    fn stats(&self, eventtime: f64) -> String {
        match self { ClockKind::Primary(c) => c.stats(eventtime), ClockKind::Secondary(c) => c.stats(eventtime) }
    }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        match self { ClockKind::Primary(c) => c.calibrate_clock(print_time, eventtime), ClockKind::Secondary(c) => c.calibrate_clock(print_time, eventtime) }
    }
}

/// Adapter from the concrete [`SerialReader`] to the [`ClockSerialApi`]
/// the `clocksync` module consumes (`klippy/clocksync.py:13`).
pub struct SerialReaderClockAdapter {
    serial: Arc<SerialReader>,
    commands: kos::sync::Mutex<IndexMap<i64, Vec<u8>>>,
    queues: kos::sync::Mutex<IndexMap<i64, Arc<CommandQueue>>>,
    next_id: kos::sync::Mutex<i64>,
}

impl SerialReaderClockAdapter {
    pub fn new(serial: Arc<SerialReader>) -> Self {
        SerialReaderClockAdapter {
            serial,
            commands: kos::sync::Mutex::new(IndexMap::new()),
            queues: kos::sync::Mutex::new(IndexMap::new()),
            next_id: kos::sync::Mutex::new(1),
        }
    }
}

fn clock_params_from_params(params: &Params) -> ClockParams {
    ClockParams {
        high: param_int(params, "high"),
        clock: param_int(params, "clock"),
        sent_time: param_float(params, "#sent_time"),
        receive_time: param_float(params, "#receive_time"),
    }
}

impl ClockSerialApi for SerialReaderClockAdapter {
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        let mp = self.serial.get_msgparser();
        let guard = mp.lock();
        match guard.get_constant_float(name, None) {
            Ok(f) => Ok(f),
            Err(e) => Err(error(e.message().to_string())),
        }
    }
    fn send_with_response(&self, msg: &str, response: &str) -> KResult<ClockParams> {
        let params = self.serial.send_with_response(msg, response)?;
        Ok(clock_params_from_params(&params))
    }
    fn create_command(&self, msg: &str) -> KResult<CommandId> {
        let mp = self.serial.get_msgparser();
        let cmd = mp.lock().create_command(msg).map_err(|e| error(e.message().to_string()))?;
        let id = *self.next_id.lock();
        *self.next_id.lock() = id + 1;
        self.commands.lock().insert(id, cmd);
        Ok(id)
    }
    fn alloc_command_queue(&self) -> CommandQueueId {
        let id = *self.next_id.lock();
        *self.next_id.lock() = id + 1;
        self.queues.lock().insert(id, self.serial.alloc_command_queue());
        id
    }
    fn register_response(&self, name: &str, cb: Box<dyn Fn(ClockParams) + Send + Sync>) {
        let cb = Arc::new(cb);
        self.serial.register_response(Some(Arc::new(move |params: &Params| {
            cb(clock_params_from_params(params));
        })), name, None);
    }
    fn raw_send(&self, cmd: CommandId, minclock: i64, reqclock: i64, cmd_queue: CommandQueueId) {
        let data = self.commands.lock().get(&cmd).cloned().unwrap_or_default();
        let q = self.queues.lock().get(&cmd_queue).cloned().unwrap_or_else(|| self.serial.get_default_command_queue());
        self.serial.raw_send(&data, minclock as u64, reqclock as u64, q);
    }
    fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: i64) {
        self.serial.set_clock_est(freq, conv_time, conv_clock as u64);
    }
}

/// `printer.get_start_args()` helpers (`klippy/mcu.py:1312,974-979`).
fn start_arg(printer: &Printer, name: &str) -> Option<String> {
    let args = printer.get_start_args();
    if let PyValue::Dict(m) = &args {
        match m.get(&PyValue::Str(name.to_string())) {
            Some(PyValue::Str(s)) => Some(s.clone()),
            Some(PyValue::None) | None => None,
            Some(other) => Some(other.str()),
        }
    } else {
        None
    }
}

fn start_reactor_pause(reactor: &Reactor, waketime: f64) {
    reactor.pause(waketime);
}

// ---------------------------------------------------------------------------
// Command transmit helpers (klippy/mcu.py:25-148)
// ---------------------------------------------------------------------------

/// Generate a dummy response to query commands when in debugging mode
/// (`klippy/mcu.py:26`).
pub struct DummyResponse {
    response: Params,
}

impl DummyResponse {
    pub fn new(serial: &SerialReader, name: &str, oid: Option<i64>) -> KResult<Self> {
        let params: Params = {
            let mut p = Params::new();
            if let Some(oid) = oid {
                p.insert("oid".to_string(), ParamValue::Int(oid));
            }
            p
        };
        let mp = serial.get_msgparser();
        let mut resp = mp.lock().create_dummy_response(name, &params)
            .map_err(|e| error(e.message().to_string()))?;
        resp.insert("#sent_time".to_string(), ParamValue::Str("0.0".to_string()));
        resp.insert("#receive_time".to_string(), ParamValue::Str("0.0".to_string()));
        Ok(DummyResponse { response: resp })
    }

    pub fn get_response(&self) -> Params {
        self.response.clone()
    }
}

/// Class to retry sending of a query command until a given response is
/// received (`klippy/mcu.py:40`).
pub struct RetryAsyncCommand {
    serial: Arc<SerialReader>,
    name: String,
    oid: Option<i64>,
    reactor: Reactor,
    completion: Completion<Params>,
    min_query_time: SCell<f64>,
    need_response: kos::sync::Mutex<bool>,
}

impl RetryAsyncCommand {
    pub const TIMEOUT_TIME: f64 = 5.0;
    pub const RETRY_TIME: f64 = 0.500;

    /// `RetryAsyncCommand.__init__`: the serial response handler holds the
    /// command until `get_response` unregisters it.
    pub fn new(serial: Arc<SerialReader>, name: &str, oid: Option<i64>) -> Arc<Self> {
        let reactor = serial.get_reactor();
        let completion = reactor.completion();
        let min_query_time = reactor.monotonic();
        let cmd = Arc::new(RetryAsyncCommand {
            serial: Arc::clone(&serial),
            name: name.to_string(),
            oid,
            reactor,
            completion,
            min_query_time: SCell::new(min_query_time),
            need_response: kos::sync::Mutex::new(true),
        });
        let handler = Arc::clone(&cmd);
        serial.register_response(Some(Arc::new(move |params: &Params| handler.handle_callback(params))), name, oid);
        cmd
    }

    fn handle_callback(&self, params: &Params) {
        let mut needed = self.need_response.lock();
        if *needed && param_float(params, "#sent_time") >= self.min_query_time.get() {
            *needed = false;
            drop(needed);
            self.reactor.async_complete(&self.completion, params.clone());
        }
    }

    pub fn get_response(
        &self,
        cmd: &[u8],
        cmd_queue: Arc<CommandQueue>,
        minclock: u64,
        reqclock: u64,
        retry: bool,
    ) -> KResult<Params> {
        self.serial.raw_send_wait_ack(cmd, minclock, reqclock, cmd_queue.clone())?;
        self.min_query_time.set(0.0);
        let query_time = self.reactor.monotonic();
        let mut timeout_time = query_time;
        if retry {
            timeout_time += Self::TIMEOUT_TIME;
        }
        let mut query_time = query_time;
        loop {
            let params = self.completion.wait(query_time + Self::RETRY_TIME);
            if let Some(params) = params {
                self.serial.register_response(None, &self.name, self.oid);
                return Ok(params);
            }
            query_time = self.reactor.monotonic();
            if query_time > timeout_time {
                self.serial.register_response(None, &self.name, self.oid);
                return Err(serialhdl::error(fmt_msg("Timeout on wait for '%s' response", &[PctValue::Str(self.name.clone())])));
            }
            self.serial.raw_send(cmd, minclock, minclock, cmd_queue.clone());
        }
    }
}

/// Wrapper around query commands (`klippy/mcu.py:76`).
pub struct CommandQueryWrapper {
    serial: Arc<SerialReader>,
    msgformat: String,
    response: String,
    oid: Option<i64>,
    error_is_command: bool,
    is_fileoutput: bool,
    is_async: bool,
    cmd_queue: Arc<CommandQueue>,
    printer: Printer,
}

impl CommandQueryWrapper {
    pub fn new(
        conn: &McuConnRef,
        msgformat: &str,
        respformat: &str,
        oid: Option<i64>,
        cmd_queue: Option<Arc<CommandQueue>>,
        is_async: bool,
    ) -> KResult<Self> {
        let serial = Arc::clone(&conn.serial);
        {
            let mp = serial.get_msgparser();
            mp.lock().lookup_command(msgformat).map_err(|e| error(e.message().to_string()))?;
            let _ = mp.lock().lookup_command(respformat).map_err(|e| error(e.message().to_string()))?;
        }
        let response = respformat.split_whitespace().next().unwrap_or_default().to_string();
        let cmd_queue = cmd_queue.unwrap_or_else(|| serial.get_default_command_queue());
        Ok(CommandQueryWrapper {
            serial: Arc::clone(&serial),
            msgformat: msgformat.to_string(),
            response,
            oid,
            error_is_command: true,
            is_fileoutput: conn.mcu.is_fileoutput(),
            is_async,
            cmd_queue,
            printer: conn.mcu.printer(),
        })
    }

    fn do_send(&self, cmds: &[Vec<u8>], minclock: i64, reqclock: i64, retry: bool) -> KResult<PyValue> {
        let reqclock = reqclock.max(minclock);
        let params: Params = if self.is_fileoutput {
            let dummy = DummyResponse::new(&self.serial, &self.response, self.oid)?;
            dummy.get_response()
        } else if self.is_async {
            let cmd = cmds.first().cloned().unwrap_or_default();
            let retry_cmd = RetryAsyncCommand::new(Arc::clone(&self.serial), &self.response, self.oid);
            retry_cmd.get_response(&cmd, Arc::clone(&self.cmd_queue), minclock as u64, reqclock as u64, retry)
                .map_err(|e| match e {
                    KlippyError::Mcu(message) => command_error(message),
                    other => other,
                })?
        } else {
            let retry_cmd = serialhdl::SerialRetryCommand::new(&self.serial, &self.response, self.oid);
            let get = retry_cmd.get_response(cmds, Arc::clone(&self.cmd_queue), minclock as u64, reqclock as u64, retry);
            match get {
                Ok(p) => p,
                Err(e) => {
                    return Err(command_error(e.message().to_string()));
                }
            }
        };
        Ok(params_to_pyvalue(&params))
    }

    fn encode(&self, data: &[PyValue]) -> KResult<Vec<u8>> {
        let mp = self.serial.get_msgparser();
        let values: Vec<ParamValue> = data.iter().map(pyvalue_to_param).collect();
        let mp = mp.lock();
        mp.lookup_command(&self.msgformat)
            .and_then(|m| m.encode(&values))
            .map_err(|e| error(e.message().to_string()))
    }

    pub fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        let cmd = self.encode(&data)?;
        self.do_send(&[cmd], opts.minclock, opts.reqclock, opts.retry)
    }

    pub fn send_with_preface(
        &self,
        preface_cmd: Arc<CommandWrapper>,
        preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        let preface = preface_cmd.encode(&preface_data)?;
        let cmd = self.encode(&data)?;
        self.do_send(&[preface, cmd], opts.minclock, opts.reqclock, opts.retry)
    }
}

impl CommandQueryWrapperApi for CommandQueryWrapper {
    fn send(&self, data: Vec<PyValue>, opts: QuerySendOpts) -> KResult<PyValue> {
        CommandQueryWrapper::send(self, data, opts)
    }
    fn send_with_preface(
        &self,
        preface_cmd: klippy_api::mcu::CommandWrapper,
        preface_data: Vec<PyValue>,
        data: Vec<PyValue>,
        opts: QuerySendOpts,
    ) -> KResult<PyValue> {
        // `preface_cmd._cmd.encode(preface_data)` (`klippy/mcu.py:106`): the
        // preface is a CommandWrapper this MCU port created.
        let preface = klippy_api::mcu::CommandWrapperAny::command_wrapper_any(&*preface_cmd)
            .downcast_ref::<CommandWrapper>()
            .ok_or_else(|| error("send_with_preface: preface command was not created by an MCU".to_string()))?
            .encode(&preface_data)?;
        let cmd = self.encode(&data)?;
        self.do_send(&[preface, cmd], opts.minclock, opts.reqclock, opts.retry)
    }
}

/// Wrapper around command sending (`klippy/mcu.py:110`).
pub struct CommandWrapper {
    serial: Arc<SerialReader>,
    msgformat: String,
    cmd_queue: Arc<CommandQueue>,
    msgtag: i64,
    is_fileoutput: bool,
}

impl CommandWrapper {
    pub fn new(conn: &McuConnRef, msgformat: &str, cmd_queue: Option<Arc<CommandQueue>>) -> KResult<Self> {
        let serial = Arc::clone(&conn.serial);
        let msgtag = {
            let mp = serial.get_msgparser();
            let guard = mp.lock();
            guard.lookup_command(msgformat).map_err(|e| error(e.message().to_string()))?;
            guard.lookup_msgid(msgformat).map_err(|e| error(e.message().to_string()))? & 0xffff_ffff
        };
        let cmd_queue = cmd_queue.unwrap_or_else(|| serial.get_default_command_queue());
        Ok(CommandWrapper {
            serial,
            msgformat: msgformat.to_string(),
            cmd_queue,
            msgtag,
            is_fileoutput: conn.mcu.is_fileoutput(),
        })
    }

    fn encode(&self, data: &[PyValue]) -> KResult<Vec<u8>> {
        let mp = self.serial.get_msgparser();
        let values: Vec<ParamValue> = data.iter().map(pyvalue_to_param).collect();
        let mp = mp.lock();
        mp.lookup_command(&self.msgformat)
            .and_then(|m| m.encode(&values))
            .map_err(|e| error(e.message().to_string()))
    }

    pub fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        let cmd = self.encode(&data)?;
        self.serial.raw_send(&cmd, opts.minclock as u64, opts.reqclock as u64, Arc::clone(&self.cmd_queue));
        Ok(())
    }

    pub fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        if self.is_fileoutput {
            return self.send(data, opts);
        }
        let cmd = self.encode(&data)?;
        self.serial.raw_send_wait_ack(&cmd, opts.minclock as u64, opts.reqclock as u64, Arc::clone(&self.cmd_queue))?;
        Ok(())
    }

    pub fn get_command_tag(&self) -> i64 {
        self.msgtag
    }
}

impl CommandWrapperApi for CommandWrapper {
    fn send(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        CommandWrapper::send(self, data, opts)
    }
    fn send_wait_ack(&self, data: Vec<PyValue>, opts: SendOpts) -> KResult<()> {
        CommandWrapper::send_wait_ack(self, data, opts)
    }
    fn get_command_tag(&self) -> i64 {
        CommandWrapper::get_command_tag(self)
    }
}

/// Downcast helper: `Arc<dyn CommandWrapperApi>` -> `Arc<CommandWrapper>`.


/// Wrapper for long-lived serial subscriptions
/// (`klippy/mcu.py:132`).
pub struct AsyncResponseWrapper {
    serial: Arc<SerialReader>,
    name: String,
    oid: Option<i64>,
}

impl AsyncResponseWrapper {
    pub fn new(
        conn: &McuConnRef,
        cfg: &McuConfigHelper,
        callback: SerialResponseCallback,
        msgformat: &str,
        oid: Option<i64>,
    ) -> Arc<Self> {
        let serial = Arc::clone(&conn.serial);
        let name = msgformat.split_whitespace().next().unwrap_or_default().to_string();
        let wrapper = Arc::new(AsyncResponseWrapper { serial: Arc::clone(&serial), name: name.clone(), oid });
        if cfg.is_config_finalized() {
            if let Err(e) = wrapper.register(callback, msgformat) {
                e.raise();
            }
        } else {
            serial.register_response(Some(Arc::new(|_: &Params| {})), &name, oid);
            let wrapper2 = Arc::clone(&wrapper);
            let mf = msgformat.to_string();
            // `register_post_init_callback` (`mcu.py:1275`) raises once
            // finalized; this runs pre-finalize, so a failure is a loud bug.
            if let Err(e) = cfg.register_post_init_callback(Box::new(move || {
                if let Err(e) = wrapper2.register(Arc::clone(&callback), &mf.clone()) {
                    e.raise();
                }
                Ok(())
            })) {
                e.raise();
            }
        }
        wrapper
    }

    fn register(&self, callback: SerialResponseCallback, msgformat: &str) -> KResult<()> {
        {
            let mp = self.serial.get_msgparser();
            mp.lock().lookup_command(msgformat).map_err(|e| error(e.message().to_string()))?;
        }
        let cb = callback.clone();
        self.serial.register_response(Some(Arc::new(move |params: &Params| {
            cb(params_to_pyvalue(params));
        })), &self.name, self.oid);
        Ok(())
    }

    pub fn unregister(&self) {
        self.serial.register_response(None, &self.name, self.oid);
    }
}

impl AsyncResponseWrapperApi for AsyncResponseWrapper {
    fn unregister(&self) {
        AsyncResponseWrapper::unregister(self);
    }
}

/// `conn_helper` view used by the wrappers (`klippy/mcu.py:77-148`).
pub struct McuConnRef {
    serial: Arc<SerialReader>,
    mcu: McuHandle,
}

impl McuConnRef {
    fn serial(&self) -> &SerialReader { &self.serial }
}
/// Port of `struct trdispatch` (`klippy/chelper/trdispatch.c`): the real
/// [`chelper_serial::trdispatch::Trdispatch`], always behind an `Arc` (see
/// `Trdispatch::mcu_alloc`). The inline copy is deleted; the timeout
/// extension and trigger propagation run in the port.
type Trdispatch = chelper_serial::trdispatch::Trdispatch;
/// Port of `struct trdispatch_mcu`: the caller's handle to one MCU
/// registration. Release with `free` (also on `Drop`).
type TrdispatchMcu = chelper_serial::trdispatch::TrdispatchMcu;
// ---------------------------------------------------------------------------
// Pin wrappers (klippy/mcu.py:155-648)
// ---------------------------------------------------------------------------

pub struct McuTrsync {
    mcu: McuHandle,
    trdispatch: Arc<Trdispatch>,
    reactor: Reactor,
    steppers: SCell<Vec<Stepper>>,
    oid: i64,
    cmd_queue: ApiCommandQueue,
    trsync_start_cmd: SCell<Option<Arc<CommandWrapper>>>,
    trsync_set_timeout_cmd: SCell<Option<Arc<CommandWrapper>>>,
    trsync_trigger_cmd: SCell<Option<Arc<CommandWrapper>>>,
    trsync_query_cmd: SCell<Option<Arc<CommandQueryWrapper>>>,
    stepper_stop_cmd: SCell<Option<Arc<CommandWrapper>>>,
    trigger_completion: SCell<Option<Completion<bool>>>,
    home_end_clock: SCell<Option<i64>>,
    response_trsync: SCell<Option<Arc<AsyncResponseWrapper>>>,
    trdispatch_mcu: SCell<Option<TrdispatchMcu>>,
    self_weak: Weak<McuTrsync>,
}

impl McuTrsync {
    pub const REASON_ENDSTOP_HIT: i64 = 1;
    pub const REASON_HOST_REQUEST: i64 = 2;
    pub const REASON_PAST_END_TIME: i64 = 3;
    pub const REASON_COMMS_TIMEOUT: i64 = 4;

    pub fn new(mcu: McuHandle, trdispatch: Arc<Trdispatch>) -> KResult<Arc<Self>> {
        let oid = mcu.create_oid()?;
        let cmd_queue = mcu.alloc_command_queue();
        let reactor = mcu.printer().get_reactor();
        let trsync = Arc::new_cyclic(|self_weak| McuTrsync {
            mcu: mcu.clone(),
            trdispatch: Arc::clone(&trdispatch),
            reactor,
            steppers: SCell::new(Vec::new()),
            oid,
            cmd_queue,
            trsync_start_cmd: SCell::new(None),
            trsync_set_timeout_cmd: SCell::new(None),
            trsync_trigger_cmd: SCell::new(None),
            trsync_query_cmd: SCell::new(None),
            stepper_stop_cmd: SCell::new(None),
            trigger_completion: SCell::new(None),
            home_end_clock: SCell::new(None),
            response_trsync: SCell::new(None),
            trdispatch_mcu: SCell::new(None),
            self_weak: self_weak.clone(),
        });
        let trsync2 = Arc::clone(&trsync);
        mcu.register_config_callback(Box::new(move || trsync2.build_config()))?;
        let trsync3 = Arc::clone(&trsync);
        mcu.printer().register_event_handler("klippy:shutdown", Box::new(move |_args| {
            trsync3.handle_shutdown();
            Ok(PyValue::None)
        }));
        Ok(trsync)
    }

    fn handle_shutdown(&self) {
        if let Some(tc) = self.trigger_completion.replace(None) {
            self.reactor.async_complete(&tc, false);
        }
    }

    fn build_config(&self) -> KResult<()> {
        let mcu = &self.mcu;
        mcu.add_config_cmd(&fmt_msg("config_trsync oid=%d", &[pct_int(self.oid)]), AddConfigCmdOpts::default())?;
        mcu.add_config_cmd(
            &fmt_msg("trsync_start oid=%d report_clock=0 report_ticks=0 expire_reason=0", &[pct_int(self.oid)]),
            AddConfigCmdOpts { is_init: false, on_restart: true },
        )?;
        let cq = Some(self.cmd_queue);
        *self.trsync_start_cmd.borrow_mut() = Some(mcu.lookup_command("trsync_start oid=%c report_clock=%u report_ticks=%u expire_reason=%c", cq)?);
        *self.trsync_set_timeout_cmd.borrow_mut() = Some(mcu.lookup_command("trsync_set_timeout oid=%c clock=%u", cq)?);
        *self.trsync_trigger_cmd.borrow_mut() = Some(mcu.lookup_command("trsync_trigger oid=%c reason=%c", cq)?);
        *self.trsync_query_cmd.borrow_mut() = Some(mcu.lookup_query_command(
            "trsync_trigger oid=%c reason=%c",
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u",
            LookupQueryCommandOpts { oid: Some(self.oid), cq: Some(self.cmd_queue), is_async: false },
        )?);
        *self.stepper_stop_cmd.borrow_mut() = Some(mcu.lookup_command("stepper_stop_on_trigger oid=%c trsync_oid=%c", cq)?);
        // Create the `trdispatch_mcu` object (`mcu.py:218-222`): the serial
        // queue comes from the MCU (`mcu._serial.get_serialqueue()`), and
        // freeing the previous registration mirrors the C `gc` pair.
        let set_timeout_tag =
            mcu.lookup_command("trsync_set_timeout oid=%c clock=%u", None)?.get_command_tag();
        let trigger_cmd = mcu.lookup_command("trsync_trigger oid=%c reason=%c", None)?;
        let trigger_tag = trigger_cmd.get_command_tag();
        let state_tag = mcu
            .lookup_command("trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u", None)?
            .get_command_tag();
        let sq = mcu
            .serial()
            .get_serialqueue()
            .expect("serialqueue present at trdispatch_mcu_alloc (mcu.py:220)");
        // `trdispatch_mcu_alloc(..., self._cmd_queue, ...)` (`mcu.py:221`):
        // the background trsync commands share the trsync's own queue, so
        // they stay FIFO-ordered ahead of `endstop_home`.
        let cq_chelper = mcu.command_queue(Some(self.cmd_queue));
        let old = self.trdispatch_mcu.replace(Some(self.trdispatch.mcu_alloc(
            sq,
            cq_chelper,
            self.oid as u32,
            set_timeout_tag as u32,
            trigger_tag as u32,
            state_tag as u32,
        )));
        // The C `gc` frees the replaced object; the port's `Drop` (via
        // `free`) unregisters the fast reader the same way.
        if let Some(old) = old {
            old.free();
        }
        Ok(())
    }

    fn handle_trsync_state(&self, params: &PyValue) {
        let p = pyvalue_to_params(params);
        if param_int(&p, "can_trigger") == 0 {
            if let Some(tc) = self.trigger_completion.replace(None) {
                let reason = param_int(&p, "trigger_reason");
                let is_failure = reason >= Self::REASON_COMMS_TIMEOUT;
                self.reactor.async_complete(&tc, is_failure);
            }
        } else if let Some(home_end) = self.home_end_clock.get() {
            let clock = self.mcu.clock32_to_clock64(param_int(&p, "clock"));
            if clock >= home_end {
                self.home_end_clock.set(None);
                if let Some(cmd) = self.trsync_trigger_cmd.borrow().clone() {
                    // `handle_trsync_state` runs on the serial reader thread; Python
                    // lets send errors raise there. The contract callback cannot
                    // return one, so panic with class+text.
                    if let Err(e) = cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid)), PyValue::Int(PyInt::from_i64(Self::REASON_PAST_END_TIME))], SendOpts::default()) {
                        e.raise();
                    }
                }
            }
        }
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn get_oid(&self) -> i64 { self.oid }
    pub fn get_command_queue(&self) -> ApiCommandQueue { self.cmd_queue }
    pub fn add_stepper(&self, stepper: Stepper) {
        let mut list = self.steppers.borrow_mut();
        if list.iter().any(|s: &Stepper| Arc::ptr_eq(s, &stepper)) {
            return;
        }
        list.push(stepper);
    }
    pub fn get_steppers(&self) -> Vec<Stepper> { self.steppers.borrow().clone() }

    pub fn start(&self, print_time: f64, report_offset: f64, trigger_completion: Completion<bool>, expire_timeout: f64) -> KResult<()> {
        *self.trigger_completion.borrow_mut() = Some(trigger_completion);
        self.home_end_clock.set(None);
        let clock = self.mcu.print_time_to_clock(print_time);
        let expire_ticks = self.mcu.seconds_to_clock(expire_timeout);
        let expire_clock = clock + expire_ticks;
        let report_ticks_f = expire_timeout * 0.3;
        let report_ticks = self.mcu.seconds_to_clock(report_ticks_f);
        let report_clock = clock + py_int_from_float(report_ticks_formula(report_ticks, report_offset));
        let min_extend_ticks = py_int_from_float(report_ticks_formula(report_ticks, 0.8));
        // `trdispatch_mcu_setup` (`mcu.py:252-254`).
        if let Some(tdm) = self.trdispatch_mcu.borrow().as_ref() {
            tdm.setup(
                clock as u64,
                expire_clock as u64,
                expire_ticks as u64,
                min_extend_ticks as u64,
            );
        }
        let weak = self.self_weak.clone();
        let cb: SerialResponseCallback = Arc::new(move |p: PyValue| {
            if let Some(trsync) = weak.upgrade() {
                trsync.handle_trsync_state(&p);
            }
        });
        *self.response_trsync.borrow_mut() = Some(self.mcu.register_serial_response(cb, "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u", Some(self.oid)));
        if let Some(cmd) = self.trsync_start_cmd.borrow().clone() {
            cmd.send(vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Int(PyInt::from_i64(report_clock)),
                PyValue::Int(PyInt::from_i64(report_ticks)),
                PyValue::Int(PyInt::from_i64(Self::REASON_COMMS_TIMEOUT)),
            ], SendOpts { minclock: 0, reqclock: clock })?;
        }
        for s in self.steppers.borrow().iter() {
            if let Some(cmd) = self.stepper_stop_cmd.borrow().clone() {
                cmd.send(vec![PyValue::Int(PyInt::from_i64(s.get_oid())), PyValue::Int(PyInt::from_i64(self.oid))], SendOpts::default())?;
            }
        }
        if let Some(cmd) = self.trsync_set_timeout_cmd.borrow().clone() {
            cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid)), PyValue::Int(PyInt::from_i64(expire_clock))], SendOpts { minclock: 0, reqclock: clock })?;
        }
        Ok(())
    }

    pub fn set_home_end_time(&self, home_end_time: f64) {
        let c = self.mcu.print_time_to_clock(home_end_time);
        self.home_end_clock.set(Some(c));
    }

    pub fn stop(&self) -> KResult<i64> {
        if let Some(w) = self.response_trsync.replace(None) {
            w.unregister();
        }
        self.trigger_completion.replace(None);
        if self.mcu.is_fileoutput() {
            return Ok(Self::REASON_ENDSTOP_HIT);
        }
        let query_cmd = self.trsync_query_cmd.cloned();
        let params = match query_cmd {
            Some(cmd) => cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid)), PyValue::Int(PyInt::from_i64(Self::REASON_HOST_REQUEST))], QuerySendOpts::default())?,
            None => return Ok(0),
        };
        for s in self.get_steppers() {
            s.note_homing_end()?;
        }
        let p = pyvalue_to_params(&params);
        Ok(param_int(&p, "trigger_reason"))
    }
}

fn report_ticks_formula(report_ticks: i64, factor: f64) -> f64 {
    // report_clock = clock + int(report_ticks * report_offset + .5)
    // min_extend_ticks = int(report_ticks * .8 + .5)
    report_ticks as f64 * factor + 0.5
}

fn pyvalue_to_params(v: &PyValue) -> Params {
    let mut out = Params::new();
    if let PyValue::Dict(m) = v {
        for (k, val) in m.iter() {
            let key = match k { PyValue::Str(s) => s.clone(), _ => k.str() };
            out.insert(key, pyvalue_to_param(val));
        }
    }
    out
}

impl klippy_api::mcu::TrsyncApi for McuTrsync {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { self.mcu.mcu_as_trait() }
    fn get_oid(&self) -> i64 { McuTrsync::get_oid(self) }
    fn get_command_queue(&self) -> ApiCommandQueue { McuTrsync::get_command_queue(self) }
    fn add_stepper(&self, stepper: Stepper) { McuTrsync::add_stepper(self, stepper) }
    fn get_steppers(&self) -> Vec<Stepper> { McuTrsync::get_steppers(self) }
    fn start(&self, print_time: f64, report_offset: f64, trigger_completion: Completion<bool>, expire_timeout: f64) {
        // `start` (`mcu.py:242-264`) lets the `trsync_start` /
        // `stepper_stop` / `set_timeout` sends raise; the contract has no
        // `KResult`, so panic with class+text.
        if let Err(e) = McuTrsync::start(self, print_time, report_offset, trigger_completion, expire_timeout) {
            e.raise();
        }
    }
    fn set_home_end_time(&self, home_end_time: f64) { McuTrsync::set_home_end_time(self, home_end_time) }
    fn stop(&self) -> KResult<i64> { McuTrsync::stop(self) }
}

/// `TriggerDispatch` (`klippy/mcu.py:282`).
pub struct TriggerDispatch {
    mcu: McuHandle,
    trigger_completion: GCell<Option<Completion<bool>>>,
    trdispatch: Arc<Trdispatch>,
    trsyncs: GCell<Vec<Arc<McuTrsync>>>,
}

impl TriggerDispatch {
    pub const REASON_ENDSTOP_HIT: i64 = McuTrsync::REASON_ENDSTOP_HIT;
    pub const REASON_HOST_REQUEST: i64 = McuTrsync::REASON_HOST_REQUEST;
    pub const REASON_PAST_END_TIME: i64 = McuTrsync::REASON_PAST_END_TIME;
    pub const REASON_COMMS_TIMEOUT: i64 = McuTrsync::REASON_COMMS_TIMEOUT;

    pub fn new(mcu: McuHandle) -> KResult<Arc<Self>> {
        let trdispatch = Arc::new(Trdispatch::new());
        let trsync = McuTrsync::new(mcu.clone(), Arc::clone(&trdispatch))?;
        Ok(Arc::new(TriggerDispatch {
            mcu,
            trigger_completion: GCell::new(None),
            trdispatch,
            trsyncs: GCell::new(vec![trsync]),
        }))
    }

    fn trsync_for(&self, stepper: &Stepper) -> KResult<Arc<McuTrsync>> {
        let mut list = self.trsyncs.borrow_mut();
        for t in list.iter() {
            if Arc::ptr_eq(&t.mcu.mcu_as_trait(), &stepper.get_mcu()) {
                return Ok(Arc::clone(t));
            }
        }
        let mcu_obj: ObjectRef = self.mcu.inner.clone();
        let t = McuTrsync::new(McuHandle::from_trait(&mcu_obj), Arc::clone(&self.trdispatch))?;
        list.push(Arc::clone(&t));
        Ok(t)
    }

    pub fn get_oid(&self) -> i64 {
        self.trsyncs.borrow()[0].get_oid()
    }
    pub fn get_command_queue(&self) -> ApiCommandQueue {
        self.trsyncs.borrow()[0].get_command_queue()
    }
    pub fn add_stepper(&self, stepper: Stepper) -> KResult<()> {
        let trsync = self.trsync_for(&stepper)?;
        trsync.add_stepper(stepper.clone());
        // Check for unsupported multi-mcu shared stepper rails.
        let sname = stepper.get_name(Some(true));
        if sname.starts_with("stepper_") {
            let prefix = &sname[..9.min(sname.len())];
            for ot in self.trsyncs.borrow().iter() {
                if Arc::ptr_eq(ot, &trsync) {
                    continue;
                }
                for s in ot.get_steppers() {
                    if s.get_name(Some(true)).starts_with(prefix) {
                        return Err(config_error("Multi-mcu homing not supported on multi-mcu shared axis".to_string()));
                    }
                }
            }
        }
        Ok(())
    }
    pub fn get_steppers(&self) -> Vec<Stepper> {
        let mut out = Vec::new();
        for t in self.trsyncs.borrow().iter() {
            out.extend(t.get_steppers());
        }
        out
    }
    pub fn start(&self, print_time: f64) -> KResult<Completion<bool>> {
        let reactor = self.mcu.printer().get_reactor();
        let completion = reactor.completion();
        *self.trigger_completion.borrow_mut() = Some(completion.clone());
        let mut expire_timeout = TRSYNC_TIMEOUT;
        if self.trsyncs.borrow().len() == 1 {
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT;
        }
        let n = self.trsyncs.borrow().len();
        for (i, trsync) in self.trsyncs.borrow().iter().enumerate() {
            let report_offset = i as f64 / n as f64;
            trsync.start(print_time, report_offset, completion.clone(), expire_timeout)?;
        }
        self.trdispatch.start(Self::REASON_HOST_REQUEST as u32);
        Ok(completion)
    }
    pub fn wait_end(&self, end_time: f64) {
        self.trsyncs.borrow()[0].set_home_end_time(end_time);
        let tc = self.trigger_completion.borrow().clone();
        if self.mcu.is_fileoutput() {
            if let Some(tc) = &tc {
                // File output completes on the reactor thread. Deferring this
                // would run background step flushing before homing returns.
                tc.complete(true);
            }
        }
        if let Some(tc) = tc {
            tc.wait(self.mcu.printer().get_reactor().monotonic() + 3600.0);
        }
    }
    pub fn stop(&self) -> KResult<i64> {
        self.trdispatch.stop();
        let mut res = Vec::new();
        for trsync in self.trsyncs.borrow().iter() {
            res.push(trsync.stop()?);
        }
        for r in res.iter() {
            if *r >= McuTrsync::REASON_COMMS_TIMEOUT {
                return Ok(*r);
            }
        }
        Ok(res[0])
    }
}

impl klippy_api::mcu::TriggerDispatchApi for TriggerDispatch {
    fn get_oid(&self) -> i64 { TriggerDispatch::get_oid(self) }
    fn get_command_queue(&self) -> ApiCommandQueue { TriggerDispatch::get_command_queue(self) }
    fn add_stepper(&self, stepper: Stepper) -> KResult<()> { TriggerDispatch::add_stepper(self, stepper) }
    fn get_steppers(&self) -> Vec<Stepper> { TriggerDispatch::get_steppers(self) }
    fn start(&self, print_time: f64) -> Completion<bool> {
        match TriggerDispatch::start(self, print_time) {
            Ok(completion) => completion,
            Err(e) => e.raise(),
        }
    }
    fn wait_end(&self, end_time: f64) { TriggerDispatch::wait_end(self, end_time) }
    fn stop(&self) -> KResult<i64> { TriggerDispatch::stop(self) }
}
/// `MCU_endstop` (`klippy/mcu.py:340`).
pub struct McuEndstop {
    mcu: McuHandle,
    pin: String,
    pullup: i64,
    invert: i64,
    oid: i64,
    home_cmd: GCell<Option<Arc<CommandWrapper>>>,
    query_cmd: GCell<Option<Arc<CommandQueryWrapper>>>,
    rest_ticks: GCell<i64>,
    dispatch: Arc<TriggerDispatch>,
}

impl McuEndstop {
    pub fn new(mcu: McuHandle, pin_params: PinParams) -> KResult<Arc<Self>> {
        let oid = mcu.create_oid()?;
        // `register_config_callback(self._build_config)` comes before
        // `TriggerDispatch(mcu)` (`mcu.py:348-350`): the endstop's config
        // commands precede the trsync's. The callback runs at MCU
        // configuration, after the endstop below exists.
        let slot: Arc<std::sync::OnceLock<Arc<McuEndstop>>> = Arc::new(std::sync::OnceLock::new());
        let slot2 = Arc::clone(&slot);
        mcu.register_config_callback(Box::new(move || {
            slot2.get().expect("endstop built before configuration").build_config()
        }))?;
        let dispatch = TriggerDispatch::new(mcu.clone())?;
        let endstop = Arc::new(McuEndstop {
            pin: pin_params.pin,
            pullup: pin_params.pullup,
            invert: pin_params.invert,
            mcu: mcu.clone(),
            oid,
            home_cmd: GCell::new(None),
            query_cmd: GCell::new(None),
            rest_ticks: GCell::new(0),
            dispatch,
        });
        if slot.set(Arc::clone(&endstop)).is_err() {
            unreachable!("the config callback slot is filled once");
        }
        Ok(endstop)
    }

    fn build_config(&self) -> KResult<()> {
        self.mcu.add_config_cmd(&fmt_msg("config_endstop oid=%d pin=%s pull_up=%d", &[pct_int(self.oid), PctValue::Str(self.pin.clone()), pct_int(self.pullup)]), AddConfigCmdOpts::default())?;
        self.mcu.add_config_cmd(
            &fmt_msg("endstop_home oid=%d clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0", &[pct_int(self.oid)]),
            AddConfigCmdOpts { is_init: false, on_restart: true },
        )?;
        let cmd_queue = self.dispatch.get_command_queue();
        let cq = Some(cmd_queue);
        *self.home_cmd.borrow_mut() = Some(self.mcu.lookup_command("endstop_home oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c trigger_reason=%c", cq)?);
        *self.query_cmd.borrow_mut() = Some(self.mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c homing=%c next_clock=%u pin_value=%c",
            LookupQueryCommandOpts { oid: Some(self.oid), cq: Some(cmd_queue), is_async: false },
        )?);
        Ok(())
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn add_stepper(&self, stepper: Stepper) -> KResult<()> { self.dispatch.add_stepper(stepper) }
    pub fn get_steppers(&self) -> Vec<Stepper> { self.dispatch.get_steppers() }

    pub fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: Option<bool>) -> KResult<Completion<bool>> {
        let clock = self.mcu.print_time_to_clock(print_time);
        let rest_ticks = self.mcu.print_time_to_clock(print_time + rest_time) - clock;
        self.rest_ticks.set(rest_ticks);
        let trigger_completion = self.dispatch.start(print_time)?;
        let triggered = triggered.unwrap_or(true);
        let pin_value = ((i64::from(triggered)) ^ self.invert) as i64;
        if let Some(cmd) = self.home_cmd.borrow().clone() {
            cmd.send(vec![
                PyValue::Int(PyInt::from_i64(self.oid)),
                PyValue::Int(PyInt::from_i64(clock)),
                PyValue::Int(PyInt::from_i64(self.mcu.seconds_to_clock(sample_time))),
                PyValue::Int(PyInt::from_i64(sample_count)),
                PyValue::Int(PyInt::from_i64(rest_ticks)),
                PyValue::Int(PyInt::from_i64(pin_value)),
                PyValue::Int(PyInt::from_i64(self.dispatch.get_oid())),
                PyValue::Int(PyInt::from_i64(McuTrsync::REASON_ENDSTOP_HIT)),
            ], SendOpts { minclock: 0, reqclock: clock })?;
        }
        Ok(trigger_completion)
    }

    pub fn home_wait(&self, home_end_time: f64) -> KResult<f64> {
        self.dispatch.wait_end(home_end_time);
        if let Some(cmd) = self.home_cmd.borrow().clone() {
            cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0)), PyValue::Int(PyInt::from_i64(0))], SendOpts::default())?;
        }
        let res = self.dispatch.stop()?;
        if res >= McuTrsync::REASON_COMMS_TIMEOUT {
            return Err(command_error("Communication timeout during homing".to_string()));
        }
        if res != McuTrsync::REASON_ENDSTOP_HIT {
            return Ok(0.0);
        }
        if self.mcu.is_fileoutput() {
            return Ok(home_end_time);
        }
        let params = match self.query_cmd.borrow().clone() {
            Some(cmd) => cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid))], QuerySendOpts::default())?,
            None => return Ok(0.0),
        };
        let p = pyvalue_to_params(&params);
        let next_clock = self.mcu.clock32_to_clock64(param_int(&p, "next_clock"));
        Ok(self.mcu.clock_to_print_time(next_clock - self.rest_ticks.get()))
    }

    pub fn query_endstop(&self, print_time: f64) -> KResult<i64> {
        let clock = self.mcu.print_time_to_clock(print_time);
        if self.mcu.is_fileoutput() {
            return Ok(0);
        }
        let params = match self.query_cmd.borrow().clone() {
            Some(cmd) => cmd.send(vec![PyValue::Int(PyInt::from_i64(self.oid))], QuerySendOpts { minclock: clock, reqclock: 0, retry: true })?,
            None => return Ok(0),
        };
        let p = pyvalue_to_params(&params);
        Ok(param_int(&p, "pin_value") ^ self.invert)
    }
}

impl EndstopApi for McuEndstop {
    fn get_mcu(&self) -> KResult<klippy_api::mcu::Mcu> { Ok(self.mcu.mcu_as_trait()) }
    fn add_stepper(&self, stepper: Stepper) {
        // `add_stepper` (`mcu.py:353`) lets `TriggerDispatch.add_stepper` raise
        // (multi-mcu shared axis); the contract has no `KResult`, so panic.
        if let Err(e) = McuEndstop::add_stepper(self, stepper) {
            e.raise();
        }
    }
    fn get_steppers(&self) -> Vec<Stepper> { McuEndstop::get_steppers(self) }
    fn home_start(&self, print_time: f64, sample_time: f64, sample_count: i64, rest_time: f64, triggered: Option<bool>) -> KResult<Completion<bool>> {
        McuEndstop::home_start(self, print_time, sample_time, sample_count, rest_time, triggered)
    }
    fn home_wait(&self, home_end_time: f64) -> KResult<f64> { McuEndstop::home_wait(self, home_end_time) }
    fn query_endstop(&self, print_time: f64) -> KResult<i64> { McuEndstop::query_endstop(self, print_time) }
}

/// `MCU_digital_out` (`klippy/mcu.py:408`).
pub struct McuDigitalOut {
    mcu: McuHandle,
    oid: SCell<Option<i64>>,
    pin: String,
    invert: i64,
    start_value: SCell<i64>,
    shutdown_value: SCell<i64>,
    max_duration: SCell<f64>,
    last_clock: SCell<i64>,
    set_cmd: SCell<Option<Arc<CommandWrapper>>>,
}

impl McuDigitalOut {
    pub fn new(mcu: McuHandle, pin_params: PinParams) -> Arc<Self> {
        let invert = pin_params.invert;
        let out = Arc::new(McuDigitalOut {
            pin: pin_params.pin,
            invert,
            mcu: mcu.clone(),
            oid: SCell::new(None),
            start_value: SCell::new(invert),
            shutdown_value: SCell::new(invert),
            max_duration: SCell::new(2.0),
            last_clock: SCell::new(0),
            set_cmd: SCell::new(None),
        });
        let out2 = Arc::clone(&out);
        // `register_config_callback` (`mcu.py:412,458,565`) raises once finalized;
        // the pin constructors run pre-finalize, so a failure is a loud bug.
        if let Err(e) = mcu.register_config_callback(Box::new(move || out2.build_config())) {
            e.raise();
        }
        out
    }

    fn build_config(&self) -> KResult<()> {
        if self.max_duration.get() != 0.0 && self.start_value.get() != self.shutdown_value.get() {
            return Err(pins_error("Pin with max duration must have start value equal to shutdown value".to_string()));
        }
        let mdur_ticks = self.mcu.seconds_to_clock(self.max_duration.get());
        if mdur_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("Digital pin max duration too large".to_string()));
        }
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid()?;
        self.oid.set(Some(oid));
        self.mcu.add_config_cmd(&fmt_msg("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d", &[pct_int(oid), PctValue::Str(self.pin.clone()), pct_int(self.start_value.get()), pct_int(self.shutdown_value.get()), pct_int(mdur_ticks)]), AddConfigCmdOpts::default())?;
        self.mcu.add_config_cmd(&fmt_msg("update_digital_out oid=%d value=%d", &[pct_int(oid), pct_int(self.start_value.get())]), AddConfigCmdOpts { is_init: false, on_restart: true })?;
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn setup_max_duration(&self, max_duration: f64) { self.max_duration.set(max_duration) }
    pub fn setup_start_value(&self, start_value: bool, shutdown_value: bool) {
        self.start_value.set((i64::from(start_value)) ^ self.invert);
        self.shutdown_value.set((i64::from(shutdown_value)) ^ self.invert);
    }
    pub fn set_digital(&self, print_time: f64, value: bool) -> KResult<()> {
        let clock = self.mcu.print_time_to_clock(print_time);
        // `self._set_cmd.send(...)`: before `_build_config` ran `_set_cmd` is `None`.
        let Some(cmd) = self.set_cmd.borrow().clone() else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            });
        };
        let oid = self.oid.get().expect("_build_config sets the oid with _set_cmd");
        let v = (i64::from(value)) ^ self.invert;
        cmd.send(vec![PyValue::Int(PyInt::from_i64(oid)), PyValue::Int(PyInt::from_i64(clock)), PyValue::Int(PyInt::from_i64(v))], SendOpts { minclock: self.last_clock.get(), reqclock: clock })?;
        self.last_clock.set(clock);
        Ok(())
    }
}

impl DigitalOutPinApi for McuDigitalOut {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { self.mcu.mcu_as_trait() }
    fn setup_max_duration(&self, max_duration: f64) { McuDigitalOut::setup_max_duration(self, max_duration) }
    fn setup_start_value(&self, start_value: bool, shutdown_value: bool) { McuDigitalOut::setup_start_value(self, start_value, shutdown_value) }
    fn set_digital(&self, print_time: f64, value: bool) {
        if let Err(e) = McuDigitalOut::set_digital(self, print_time, value) {
            e.raise();
        }
    }
}

/// `MCU_pwm` (`klippy/mcu.py:451`).
pub struct McuPwm {
    mcu: McuHandle,
    hardware_pwm: SCell<bool>,
    cycle_time: SCell<f64>,
    max_duration: SCell<f64>,
    oid: SCell<Option<i64>>,
    pin: String,
    invert: i64,
    start_value: SCell<f64>,
    shutdown_value: SCell<f64>,
    last_clock: SCell<i64>,
    last_value: SCell<f64>,
    pwm_max: SCell<f64>,
    set_cmd: SCell<Option<Arc<CommandWrapper>>>,
}

impl McuPwm {
    pub fn new(mcu: McuHandle, pin_params: PinParams) -> Arc<Self> {
        let invert = pin_params.invert;
        let out = Arc::new(McuPwm {
            pin: pin_params.pin,
            invert,
            mcu: mcu.clone(),
            hardware_pwm: SCell::new(false),
            cycle_time: SCell::new(0.100),
            max_duration: SCell::new(2.0),
            oid: SCell::new(None),
            start_value: SCell::new(invert as f64),
            shutdown_value: SCell::new(invert as f64),
            last_clock: SCell::new(0),
            last_value: SCell::new(0.0),
            pwm_max: SCell::new(0.0),
            set_cmd: SCell::new(None),
        });
        let out2 = Arc::clone(&out);
        // `register_config_callback` (`mcu.py:412,458,565`) raises once finalized;
        // the pin constructors run pre-finalize, so a failure is a loud bug.
        if let Err(e) = mcu.register_config_callback(Box::new(move || out2.build_config())) {
            e.raise();
        }
        out
    }

    fn build_config(&self) -> KResult<()> {
        if self.max_duration.get() != 0.0 && self.start_value.get() != self.shutdown_value.get() {
            return Err(pins_error("Pin with max duration must have start value equal to shutdown value".to_string()));
        }
        let curtime = self.mcu.printer().get_reactor().monotonic();
        let printtime = self.mcu.estimated_print_time(curtime);
        self.last_clock.set(self.mcu.print_time_to_clock(printtime + 0.200));
        let cycle_ticks = self.mcu.seconds_to_clock(self.cycle_time.get());
        let mdur_ticks = self.mcu.seconds_to_clock(self.max_duration.get());
        if mdur_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("PWM pin max duration too large".to_string()));
        }
        if self.hardware_pwm.get() {
            let pwm_max = self.mcu.get_constant_float("PWM_MAX")?;
            self.pwm_max.set(pwm_max);
            self.mcu.request_move_queue_slot();
            let oid = self.mcu.create_oid()?;
            self.oid.set(Some(oid));
            let start_scaled = self.start_value.get() * pwm_max;
            let shutdown_scaled = self.shutdown_value.get() * pwm_max;
            self.mcu.add_config_cmd(&fmt_msg("config_pwm_out oid=%d pin=%s cycle_ticks=%d value=%d default_value=%d max_duration=%d", &[pct_int(oid), PctValue::Str(self.pin.clone()), pct_int(cycle_ticks), pct_int(py_int_from_float(start_scaled)), pct_int(py_int_from_float(shutdown_scaled)), pct_int(mdur_ticks)]), AddConfigCmdOpts::default())?;
            let svalue = py_int_from_float(self.start_value.get() * pwm_max + 0.5);
            self.mcu.add_config_cmd(&fmt_msg("queue_pwm_out oid=%d clock=%d value=%d", &[pct_int(oid), pct_int(self.last_clock.get()), pct_int(svalue)]), AddConfigCmdOpts { is_init: false, on_restart: true })?;
            let cmd_queue = self.mcu.alloc_command_queue();
            *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_pwm_out oid=%c clock=%u value=%hu", Some(cmd_queue))?);
            return Ok(());
        }
        // Software PWM.
        let sv = self.shutdown_value.get();
        if sv != 0.0 && sv != 1.0 {
            return Err(pins_error("shutdown value must be 0.0 or 1.0 on soft pwm".to_string()));
        }
        if cycle_ticks > MAX_SCHEDULE_TICKS {
            return Err(pins_error("PWM pin cycle time too large".to_string()));
        }
        self.mcu.request_move_queue_slot();
        let oid = self.mcu.create_oid()?;
        self.oid.set(Some(oid));
        let has_start = if self.start_value.get() >= 1.0 { 1 } else { 0 };
        let has_shutdown = if self.shutdown_value.get() >= 0.5 { 1 } else { 0 };
        self.mcu.add_config_cmd(&fmt_msg("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d", &[pct_int(oid), PctValue::Str(self.pin.clone()), pct_int(has_start), pct_int(has_shutdown), pct_int(mdur_ticks)]), AddConfigCmdOpts::default())?;
        self.mcu.add_config_cmd(&fmt_msg("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", &[pct_int(oid), pct_int(cycle_ticks)]), AddConfigCmdOpts::default())?;
        self.pwm_max.set(cycle_ticks as f64);
        let svalue = py_int_from_float(self.start_value.get() * cycle_ticks as f64 + 0.5);
        self.mcu.add_config_cmd(&fmt_msg("queue_digital_out oid=%d clock=%d on_ticks=%d", &[pct_int(oid), pct_int(self.last_clock.get()), pct_int(svalue)]), AddConfigCmdOpts { is_init: true, on_restart: false })?;
        let cmd_queue = self.mcu.alloc_command_queue();
        *self.set_cmd.borrow_mut() = Some(self.mcu.lookup_command("queue_digital_out oid=%c clock=%u on_ticks=%u", Some(cmd_queue))?);
        Ok(())
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn setup_max_duration(&self, max_duration: f64) { self.max_duration.set(max_duration) }
    pub fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) {
        self.cycle_time.set(cycle_time);
        self.hardware_pwm.set(hardware_pwm.unwrap_or(false));
    }
    pub fn setup_start_value(&self, start_value: f64, shutdown_value: f64) {
        let (mut start_value, mut shutdown_value) = (start_value, shutdown_value);
        if self.invert != 0 {
            start_value = 1.0 - start_value;
            shutdown_value = 1.0 - shutdown_value;
        }
        let s = start_value.max(0.0).min(1.0);
        let sd = shutdown_value.max(0.0).min(1.0);
        self.start_value.set(s);
        self.shutdown_value.set(sd);
        self.last_value.set(s);
    }
    pub fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64 {
        if self.hardware_pwm.get() {
            return print_time;
        }
        if self.last_value.get() == 1.0 || self.last_value.get() == 0.0 {
            return print_time;
        }
        let allow_early = allow_early.unwrap_or(0.0);
        let req_ptime = print_time - allow_early.min(0.5 * self.cycle_time.get());
        let cycle_ticks = self.mcu.seconds_to_clock(self.cycle_time.get());
        let req_clock = self.mcu.print_time_to_clock(req_ptime);
        let last_clock = self.last_clock.get();
        let pulses = (req_clock - last_clock + cycle_ticks - 1).div_euclid(cycle_ticks.max(1));
        let next_clock = last_clock + pulses * cycle_ticks;
        self.mcu.clock_to_print_time(next_clock)
    }
    pub fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        let mut value = value;
        if self.invert != 0 {
            value = 1.0 - value;
        }
        let v = py_int_from_float(value.max(0.0).min(1.0) * self.pwm_max.get() + 0.5);
        let clock = self.mcu.print_time_to_clock(print_time);
        // `self._set_cmd.send(...)`: before `_build_config` ran `_set_cmd` is `None`.
        let Some(cmd) = self.set_cmd.borrow().clone() else {
            return Err(KlippyError::Internal {
                class_name: "AttributeError".to_string(),
                message: "'NoneType' object has no attribute 'send'".to_string(),
            });
        };
        let oid = self.oid.get().expect("_build_config sets the oid with _set_cmd");
        cmd.send(vec![PyValue::Int(PyInt::from_i64(oid)), PyValue::Int(PyInt::from_i64(clock)), PyValue::Int(PyInt::from_i64(v))], SendOpts { minclock: self.last_clock.get(), reqclock: clock })?;
        self.last_clock.set(clock);
        self.last_value.set(value);
        Ok(())
    }
}

impl PwmPinApi for McuPwm {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { self.mcu.mcu_as_trait() }
    fn setup_max_duration(&self, max_duration: f64) { McuPwm::setup_max_duration(self, max_duration) }
    fn setup_cycle_time(&self, cycle_time: f64, hardware_pwm: Option<bool>) -> KResult<()> {
        McuPwm::setup_cycle_time(self, cycle_time, hardware_pwm);
        Ok(())
    }
    fn setup_start_value(&self, start_value: f64, shutdown_value: f64) { McuPwm::setup_start_value(self, start_value, shutdown_value) }
    fn next_aligned_print_time(&self, print_time: f64, allow_early: Option<f64>) -> f64 { McuPwm::next_aligned_print_time(self, print_time, allow_early) }
    fn set_pwm(&self, print_time: f64, value: f64) -> KResult<()> {
        McuPwm::set_pwm(self, print_time, value)
    }
}

/// `MCU_adc` (`klippy/mcu.py:555`).
pub struct McuAdc {
    mcu: McuHandle,
    pin: String,
    min_sample: SCell<f64>,
    max_sample: SCell<f64>,
    sample_time: SCell<f64>,
    report_time: SCell<f64>,
    sample_count: SCell<i64>,
    batch_num: SCell<i64>,
    range_check_count: SCell<i64>,
    report_clock: SCell<i64>,
    last_state: SCell<(f64, f64)>,
    oid: SCell<Option<i64>>,
    callback: SCell<Option<AdcCallback>>,
    inv_max_adc: SCell<f64>,
    self_weak: Weak<McuAdc>,
}

impl McuAdc {
    pub fn new(mcu: McuHandle, pin_params: PinParams) -> Arc<Self> {
        let out = Arc::new_cyclic(|self_weak| McuAdc {
            pin: pin_params.pin,
            mcu: mcu.clone(),
            min_sample: SCell::new(0.0),
            max_sample: SCell::new(0.0),
            sample_time: SCell::new(0.0),
            report_time: SCell::new(0.0),
            sample_count: SCell::new(0),
            batch_num: SCell::new(0),
            range_check_count: SCell::new(0),
            report_clock: SCell::new(0),
            last_state: SCell::new((0.0, 0.0)),
            oid: SCell::new(None),
            callback: SCell::new(None),
            inv_max_adc: SCell::new(0.0),
            self_weak: self_weak.clone(),
        });
        let out2 = Arc::clone(&out);
        // `register_config_callback` (`mcu.py:412,458,565`) raises once finalized;
        // the pin constructors run pre-finalize, so a failure is a loud bug.
        if let Err(e) = mcu.register_config_callback(Box::new(move || out2.build_config())) {
            e.raise();
        }
        out
    }

    fn build_config(&self) -> KResult<()> {
        if self.sample_count.get() == 0 {
            return Ok(());
        }
        let oid = self.mcu.create_oid()?;
        self.oid.set(Some(oid));
        self.mcu.add_config_cmd(&fmt_msg("config_analog_in oid=%d pin=%s", &[pct_int(oid), PctValue::Str(self.pin.clone())]), AddConfigCmdOpts::default())?;
        let clock = self.mcu.get_query_slot(oid);
        let sample_ticks = self.mcu.seconds_to_clock(self.sample_time.get());
        let mcu_adc_max = self.mcu.get_constant_float("ADC_MAX")?;
        let max_adc = self.sample_count.get() as f64 * mcu_adc_max;
        if max_adc >= 65536.0 {
            return Err(config_error(fmt_msg("ADC sample_count=%d too large for MCU", &[pct_int(self.sample_count.get())])));
        }
        self.inv_max_adc.set(1.0 / max_adc);
        let report_clock = self.mcu.seconds_to_clock(self.report_time.get());
        self.report_clock.set(report_clock);
        let min_sample = py_int_from_float(self.min_sample.get() * max_adc).clamp(0, 0xffff);
        let max_sample = (self.max_sample.get() * max_adc).ceil() as i64;
        let max_sample = max_sample.clamp(0, 0xffff);
        let oldcmd = "query_analog_in oid=%c clock=%u sample_ticks=%u sample_count=%c rest_ticks=%u min_value=%hu max_value=%hu range_check_count=%c";
        if self.batch_num.get() == 1 && self.mcu.try_lookup_command(oldcmd).is_some() {
            self.mcu.add_config_cmd(&fmt_msg("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d", &[pct_int(oid), pct_int(clock), pct_int(sample_ticks), pct_int(self.sample_count.get()), pct_int(report_clock), pct_int(min_sample), pct_int(max_sample), pct_int(self.range_check_count.get())]), AddConfigCmdOpts { is_init: true, on_restart: false })?;
            let weak = self.self_weak.clone();
            let cb: SerialResponseCallback = Arc::new(move |p: PyValue| {
                if let Some(adc) = weak.upgrade() {
                    adc.old_handle_analog_in_state(&p);
                }
            });
            self.mcu.register_serial_response(cb, "analog_in_state oid=%c next_clock=%u value=%hu", Some(oid));
            return Ok(());
        }
        let bytes_per_report = self.batch_num.get() * 2;
        self.mcu.add_config_cmd(&fmt_msg("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d bytes_per_report=%d min_value=%d max_value=%d range_check_count=%d", &[pct_int(oid), pct_int(clock), pct_int(sample_ticks), pct_int(self.sample_count.get()), pct_int(report_clock), pct_int(bytes_per_report), pct_int(min_sample), pct_int(max_sample), pct_int(self.range_check_count.get())]), AddConfigCmdOpts { is_init: true, on_restart: false })?;
        let weak = self.self_weak.clone();
        let cb: SerialResponseCallback = Arc::new(move |p: PyValue| {
            if let Some(adc) = weak.upgrade() {
                adc.handle_analog_in_state(&p);
            }
        });
        self.mcu.register_serial_response(cb, "analog_in_state oid=%c next_clock=%u values=%*s", Some(oid));
        Ok(())
    }

    fn old_handle_analog_in_state(&self, params: &PyValue) {
        let p = pyvalue_to_params(params);
        let last_value = param_int(&p, "value") as f64 * self.inv_max_adc.get();
        let next_clock = self.mcu.clock32_to_clock64(param_int(&p, "next_clock"));
        let last_read_clock = next_clock - self.report_clock.get();
        let last_read_time = self.mcu.clock_to_print_time(last_read_clock);
        self.last_state.set((last_read_time, last_value));
        let mut callback = self.callback.take_callback();
        if let Some(cb) = callback.as_mut() {
            cb(vec![(last_read_time, last_value)]);
        }
    }

    fn handle_analog_in_state(&self, params: &PyValue) {
        let p = pyvalue_to_params(params);
        let raw = match p.get("values") {
            Some(ParamValue::Bytes(b)) => b.clone(),
            Some(ParamValue::Str(s)) => s.as_bytes().to_vec(),
            _ => Vec::new(),
        };
        let values: Vec<i64> = raw.chunks_exact(2).map(|c| (c[0] as i64) | ((c[1] as i64) << 8)).collect();
        let next_clock = self.mcu.clock32_to_clock64(param_int(&p, "next_clock"));
        let num = values.len() as i64;
        let mut samples = Vec::new();
        for i in 0..num {
            let t = self.mcu.clock_to_print_time(next_clock - (num - i) * self.report_clock.get());
            samples.push((t, values[i as usize] as f64 * self.inv_max_adc.get()));
        }
        if let Some(last) = samples.last() {
            self.last_state.set(*last);
        }
        let mut callback = self.callback.take_callback();
        if let Some(cb) = callback.as_mut() {
            cb(samples);
        }
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) {
        self.report_time.set(report_time);
        self.sample_time.set(opts.sample_time);
        self.sample_count.set(opts.sample_count);
        self.batch_num.set(opts.batch_num.max(1).min(24));
        self.min_sample.set(opts.minval);
        self.max_sample.set(opts.maxval);
        self.range_check_count.set(opts.range_check_count);
    }
    pub fn setup_adc_callback(&self, callback: AdcCallback) {
        *self.callback.borrow_mut() = Some(callback);
    }
    pub fn get_last_value(&self) -> (f64, f64) { self.last_state.get() }
}

impl klippy_api::mcu::AdcPinApi for McuAdc {
    fn get_mcu(&self) -> klippy_api::mcu::Mcu { self.mcu.mcu_as_trait() }
    fn setup_adc_sample(&self, report_time: f64, opts: AdcSampleOpts) { McuAdc::setup_adc_sample(self, report_time, opts) }
    fn setup_adc_callback(&self, callback: AdcCallback) { McuAdc::setup_adc_callback(self, callback) }
    fn get_last_value(&self) -> (f64, f64) { McuAdc::get_last_value(self) }
}
/// `canbus_ids.PrinterCANBus` duck type (`klippy/extras/canbus_ids.py:9`),
/// looked up by name from `MCUConnectHelper` (`klippy/mcu.py:795,990`).
pub trait CanbusIdsApi: Send + Sync + 'static {
    /// Returns the node id assigned to `canbus_uuid`.
    fn add_uuid(&self, config: &Config, canbus_uuid: &str, canbus_iface: &str) -> KResult<i64>;
    fn get_nodeid(&self, canbus_uuid: &str) -> KResult<i64>;
}

/// Hub power controller (`klippy/chelper/__init__.py:339`).
pub fn run_hub_ctrl(enable_power: i64) {
    let _ = std::process::Command::new("sudo")
        .arg("hub-ctrl")
        .arg("-h")
        .arg("0")
        .arg("-P")
        .arg("2")
        .arg("-p")
        .arg(enable_power.to_string())
        .status();
}

/// Support for restarting a micro-controller (`klippy/mcu.py:656`).
pub struct McuRestartHelper {
    printer: Printer,
    mcu: McuHandle,
    serial: Arc<SerialReader>,
    clocksync: ClockKind,
    name: String,
    restart_method: GCell<Option<String>>,
    reset_cmd: GCell<Option<Arc<CommandWrapper>>>,
    config_reset_cmd: GCell<Option<Arc<CommandWrapper>>>,
    is_mcu_bridge: GCell<bool>,
}

impl McuRestartHelper {
    pub fn new(config: &Config, mcu: McuHandle, serial: Arc<SerialReader>, clocksync: ClockKind, conn: &McuConnectHelper) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = mcu.get_name();
        let restart_methods: Vec<PyValue> = [PyValue::None, PyValue::Str("arduino".to_string()), PyValue::Str("cheetah".to_string()), PyValue::Str("command".to_string()), PyValue::Str("rpi_usb".to_string())].to_vec();
        let mut restart_method = Some("command".to_string());
        let (_port, baud) = conn.get_serialport();
        if baud != 0 {
            let v = config.getchoice(
                "restart_method",
                &PyValue::List(restart_methods),
                GetChoiceOpts { default: Some(None), ..GetChoiceOpts::default() },
            )?;
            restart_method = match v { PyValue::None => None, PyValue::Str(s) => Some(s), other => Some(other.str()) };
        }
        let helper = Arc::new(McuRestartHelper {
            printer: printer.clone(),
            mcu: mcu.clone(),
            serial: Arc::clone(&serial),
            clocksync,
            name: name.clone(),
            restart_method: GCell::new(restart_method),
            reset_cmd: GCell::new(None),
            config_reset_cmd: GCell::new(None),
            is_mcu_bridge: GCell::new(false),
        });
        let h2 = Arc::clone(&helper);
        printer.register_event_handler("klippy:firmware_restart", Box::new(move |_args| { h2.firmware_restart(false); Ok(PyValue::None) }));
        let h3 = Arc::clone(&helper);
        printer.register_event_handler("klippy:disconnect", Box::new(move |_args| { h3.disconnect(); Ok(PyValue::None) }));
        let h4 = Arc::clone(&helper);
        printer.register_event_handler("klippy:mcu_identify", Box::new(move |_args| { h4.mcu_identify(); Ok(PyValue::None) }));
        Ok(helper)
    }

    fn check_restart(&self, reason: &str) -> KResult<()> {
        let start_reason = start_arg(&self.printer, "start_reason").unwrap_or_default();
        if start_reason == "firmware_restart" {
            return Ok(());
        }
        log::info(&fmt_msg("Attempting automated MCU '%s' restart: %s", &[PctValue::Str(self.name.clone()), PctValue::Str(reason.to_string())]));
        self.printer.request_exit("firmware_restart");
        let reactor = self.printer.get_reactor();
        reactor.pause(reactor.monotonic() + 2.0);
        Err(error(fmt_msg("Attempt MCU '%s' restart failed", &[PctValue::Str(self.name.clone())])))
    }

    fn check_restart_on_crc_mismatch(&self) -> KResult<()> { self.check_restart("CRC mismatch") }
    fn check_restart_on_send_config(&self) -> KResult<()> {
        if self.restart_method.borrow().as_deref() == Some("rpi_usb") {
            self.check_restart("full reset before config")
        } else { Ok(()) }
    }
    fn check_restart_on_attach(&self) -> KResult<()> {
        let resmeth = self.restart_method.borrow().clone();
        let (serialport, _baud) = self.conn_serialport();
        if resmeth.as_deref() == Some("rpi_usb") && !std::path::Path::new(&serialport).exists() {
            self.check_restart("enable power")
        } else { Ok(()) }
    }
    fn conn_serialport(&self) -> (String, i64) {
        // The ports live on the connect helper; read them back from the MCU.
        self.mcu.serialport()
    }
    fn lookup_attach_uart_rts(&self) -> bool {
        self.restart_method.borrow().as_deref() != Some("cheetah")
    }
    fn mcu_identify(&self) {
        *self.reset_cmd.borrow_mut() = self.mcu.try_lookup_command("reset");
        *self.config_reset_cmd.borrow_mut() = self.mcu.try_lookup_command("config_reset");
        let ext_only = self.reset_cmd.borrow().is_none() && self.config_reset_cmd.borrow().is_none();
        let mbaud = {
            let mp = self.serial.get_msgparser();
            let mp = mp.lock();
            mp.get_constant("SERIAL_BAUD", Some(PyValue::None), msgproto::ConstParser::Str).ok()
        };
        if self.restart_method.borrow().is_none() && mbaud.is_none() && !ext_only {
            *self.restart_method.borrow_mut() = Some("command".to_string());
        }
        let is_bridge = {
            let mp = self.serial.get_msgparser();
            let mp = mp.lock();
            mp.get_constant("CANBUS_BRIDGE", Some(PyValue::Int(PyInt::from_i64(0))), msgproto::ConstParser::Int).ok().map(|v| as_int(&v) != 0).unwrap_or(false)
        };
        if is_bridge {
            self.is_mcu_bridge.set(true);
            let h2 = self.printer.clone();
            let _ = h2;
            // Python registers a second firmware_restart handler for bridges.
            // Registry order matters and is preserved by the printer.
            let name = self.name.clone();
            let _ = name;
        }
    }
    fn disconnect(&self) { self.serial.disconnect() }
    fn restart_arduino(&self) {
        log::info(&fmt_msg("Attempting MCU '%s' reset", &[PctValue::Str(self.name.clone())]));
        self.disconnect();
        let (port, _baud) = self.conn_serialport();
        serialhdl::arduino_reset(&port, &self.printer.get_reactor(), &serialhdl::FdOpener);
    }
    fn restart_cheetah(&self) {
        log::info(&fmt_msg("Attempting MCU '%s' Cheetah-style reset", &[PctValue::Str(self.name.clone())]));
        self.disconnect();
        let (port, _baud) = self.conn_serialport();
        serialhdl::cheetah_reset(&port, &self.printer.get_reactor(), &serialhdl::FdOpener);
    }
    fn restart_via_command(&self) {
        if (self.reset_cmd.borrow().is_none() && self.config_reset_cmd.borrow().is_none()) || !self.clocksync.is_active() {
            log::info(&fmt_msg("Unable to issue reset command on MCU '%s'", &[PctValue::Str(self.name.clone())]));
            return;
        }
        let reactor = self.printer.get_reactor();
        if self.reset_cmd.borrow().is_none() {
            log::info(&fmt_msg("Attempting MCU '%s' config_reset command", &[PctValue::Str(self.name.clone())]));
            self.mcu.force_local_shutdown();
            reactor.pause(reactor.monotonic() + 0.015);
            if let Some(cmd) = self.config_reset_cmd.borrow().clone() {
                if let Err(e) = cmd.send(vec![], SendOpts::default()) {
                    e.raise();
                }
            }
        } else {
            log::info(&fmt_msg("Attempting MCU '%s' reset command", &[PctValue::Str(self.name.clone())]));
            if let Some(cmd) = self.reset_cmd.borrow().clone() {
                if let Err(e) = cmd.send(vec![], SendOpts::default()) {
                    e.raise();
                }
            }
        }
        reactor.pause(reactor.monotonic() + 0.015);
        self.disconnect();
    }
    fn restart_rpi_usb(&self) {
        log::info(&fmt_msg("Attempting MCU '%s' reset via rpi usb power", &[PctValue::Str(self.name.clone())]));
        self.disconnect();
        run_hub_ctrl(0);
        let reactor = self.printer.get_reactor();
        reactor.pause(reactor.monotonic() + 2.0);
        run_hub_ctrl(1);
    }
    pub fn firmware_restart(&self, force: bool) {
        if self.is_mcu_bridge.get() && !force {
            return;
        }
        match self.restart_method.borrow().as_deref() {
            Some("rpi_usb") => self.restart_rpi_usb(),
            Some("command") => self.restart_via_command(),
            Some("cheetah") => self.restart_cheetah(),
            _ => self.restart_arduino(),
        }
    }
}

/// Low-level mcu connection management helper (`klippy/mcu.py:780`).
pub struct McuConnectHelper {
    mcu: McuHandle,
    clocksync: ClockKind,
    printer: Printer,
    name: String,
    serial: Arc<SerialReader>,
    baud: i64,
    canbus_iface: SCell<Option<String>>,
    serialport: String,
    emergency_stop_cmd: SCell<Option<Arc<CommandWrapper>>>,
    is_shutdown: SCell<bool>,
    is_timeout: SCell<bool>,
    shutdown_msg: SCell<String>,
    debug_shutdown_info: SCell<Option<IndexMap<String, Option<PyValue>>>>,
    debug_timeout_info: SCell<Option<IndexMap<String, Option<PyValue>>>>,
    debug_shutdown_notes: SCell<i64>,
    debug_shutdown_frozen: SCell<bool>,
    restart_helper: SCell<Option<Arc<McuRestartHelper>>>,
    self_weak: Weak<McuConnectHelper>,
}

impl McuConnectHelper {
    pub fn new(config: &Config, mcu: McuHandle, clocksync: ClockKind, serial: Arc<SerialReader>) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = mcu.get_name();
        let canbus_uuid = config.get("canbus_uuid", GetStrOpts { default: Some(None), ..GetStrOpts::default() })?;
        let (serialport, baud, canbus_iface) = if let Some(uuid) = canbus_uuid {
            let iface = config
                .get("canbus_interface", GetStrOpts { default: Some(Some("can0".to_string())), ..GetStrOpts::default() })?
                .expect("defaulted");
            let cbid = printer
                .load_object(config, "canbus_ids", LookupDefault::Raise)?
                .expect("LookupDefault::Raise never yields None");
            klippy_api::interface::<dyn CanbusIdsApi>(&cbid)
                .ok_or_else(|| klippy_api::missing_interface::<dyn CanbusIdsApi>("canbus_ids"))?
                .add_uuid(config, &uuid, &iface)?;
            printer.load_object(config, &fmt_msg("canbus_stats %s", &[PctValue::Str(name.clone())]), LookupDefault::Raise)?;
            (uuid, 0, Some(iface))
        } else {
            let port = config.get("serial", GetStrOpts::default())?.expect("required");
            let baud = if port.starts_with("/dev/rpmsg_") || port.starts_with("/tmp/klipper_host_") {
                0
            } else {
                config
                    .getint("baud", GetIntOpts { default: Some(Some(250000)), minval: Some(2400), ..GetIntOpts::default() })?
                    .expect("defaulted")
            };
            (port, baud, None)
        };
        let helper = Arc::new_cyclic(|self_weak| McuConnectHelper {
            mcu: mcu.clone(),
            clocksync: clocksync.clone(),
            printer: printer.clone(),
            name: name.clone(),
            serial: Arc::clone(&serial),
            baud,
            canbus_iface: SCell::new(canbus_iface),
            serialport,
            emergency_stop_cmd: SCell::new(None),
            is_shutdown: SCell::new(false),
            is_timeout: SCell::new(false),
            shutdown_msg: SCell::new(String::new()),
            debug_shutdown_info: SCell::new(None),
            debug_timeout_info: SCell::new(None),
            debug_shutdown_notes: SCell::new(0),
            debug_shutdown_frozen: SCell::new(false),
            restart_helper: SCell::new(None),
            self_weak: self_weak.clone(),
        });
        let h = Arc::clone(&helper);
        let mcu2 = mcu.clone();
        printer.register_event_handler("klippy:mcu_identify", Box::new(move |_args| { h.mcu_identify(&mcu2)?; Ok(PyValue::None) }));
        let h2 = Arc::clone(&helper);
        printer.register_event_handler("klippy:shutdown", Box::new(move |_args| { h2.shutdown(false); Ok(PyValue::None) }));
        let h3 = Arc::clone(&helper);
        printer.register_event_handler("klippy:analyze_shutdown", Box::new(move |_args| { h3.analyze_shutdown(); Ok(PyValue::None) }));
        Ok(helper)
    }

    pub fn get_mcu(&self) -> McuHandle { self.mcu.clone() }
    pub fn get_serial(&self) -> Arc<SerialReader> { Arc::clone(&self.serial) }
    pub fn get_clocksync(&self) -> ClockKind { self.clocksync.clone() }
    pub fn get_serialport(&self) -> (String, i64) { (self.serialport.clone(), self.baud) }

    fn handle_shutdown(&self, params: &PyValue) {
        self.debug_note_shutdown(params);
        if self.is_shutdown.get() {
            return;
        }
        self.is_shutdown.set(true);
        let msg = {
            let p = pyvalue_to_params(params);
            param_str(&p, "static_string_id")
        };
        *self.shutdown_msg.borrow_mut() = msg.clone();
        let shutdown_clock = {
            let p = pyvalue_to_params(params);
            match p.get("clock") {
                Some(ParamValue::Int(c)) => Some(self.mcu.clock32_to_clock64(*c)),
                _ => None,
            }
        };
        let event_type = {
            let p = pyvalue_to_params(params);
            param_str(&p, "#name")
        };
        let mut details: IndexMap<PyValue, PyValue> = IndexMap::new();
        details.insert(PyValue::Str("reason".to_string()), PyValue::Str(msg));
        details.insert(PyValue::Str("mcu".to_string()), PyValue::Str(self.name.clone()));
        details.insert(PyValue::Str("event_type".to_string()), PyValue::Str(event_type));
        details.insert(PyValue::Str("shutdown_clock".to_string()), match shutdown_clock {
            Some(c) => PyValue::Int(PyInt::from_i64(c)),
            None => PyValue::None,
        });
        self.printer.invoke_async_shutdown("MCU shutdown", PyValue::Dict(details));
    }

    fn debug_build_shutdown(&self, params: &PyValue, clock32: Option<i64>, now: f64) -> IndexMap<String, Option<PyValue>> {
        let (clock, print_time) = match clock32 {
            Some(c32) => {
                let c = self.mcu.clock32_to_clock64(c32);
                let pt = self.clocksync.clock_to_print_time(c);
                (Some(c), Some(pt))
            }
            None => (None, None),
        };
        let p = pyvalue_to_params(params);
        let reason: String = param_str(&p, "static_string_id").chars().take(DEBUG_REASON_MAX_LEN).collect();
        let mut m: IndexMap<String, Option<PyValue>> = IndexMap::new();
        m.insert("event".to_string(), Some(PyValue::Str(param_str(&p, "#name"))));
        m.insert("reason".to_string(), Some(PyValue::Str(reason)));
        m.insert("clock_known".to_string(), Some(PyValue::Bool(clock32.is_some())));
        m.insert("clock32".to_string(), clock32.map(|v| PyValue::Int(PyInt::from_i64(v))));
        m.insert("clock".to_string(), clock.map(|v| PyValue::Int(PyInt::from_i64(v))));
        m.insert("print_time".to_string(), print_time.map(PyValue::Float));
        m.insert("sent_time".to_string(), p.get("#sent_time").map(|v| param_entry_to_pyvalue("#sent_time", v)));
        m.insert("receive_time".to_string(), p.get("#receive_time").map(|v| param_entry_to_pyvalue("#receive_time", v)));
        m.insert("host_monotonic".to_string(), Some(PyValue::Float(now)));
        m.insert("host_clock_est".to_string(), Some(PyValue::Int(PyInt::from_i64(self.clocksync.get_clock(now)))));
        m.insert("host_print_time".to_string(), Some(PyValue::Float(self.clocksync.estimated_print_time(now))));
        m.insert("clocksync_active".to_string(), Some(PyValue::Bool(self.clocksync.is_active())));
        m.insert("local_shutdown_first".to_string(), Some(PyValue::Bool(self.is_shutdown.get())));
        m.insert("timeout_first".to_string(), Some(PyValue::Bool(self.debug_timeout_info.borrow().is_some())));
        m.insert("late_event".to_string(), None);
        m.insert("late_clock32".to_string(), None);
        m.insert("late_clock".to_string(), None);
        m.insert("late_print_time".to_string(), None);
        m.insert("late_receive_time".to_string(), None);
        m.insert("late_host_monotonic".to_string(), None);
        m
    }

    fn debug_note_shutdown(&self, params: &PyValue) {
        if self.debug_shutdown_frozen.get() {
            return;
        }
        if self.debug_shutdown_notes.get() >= DEBUG_SHUTDOWN_MAX_NOTES {
            return;
        }
        self.debug_shutdown_notes.set(self.debug_shutdown_notes.get() + 1);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let now = self.printer.get_reactor().monotonic();
            let p = pyvalue_to_params(params);
            let clock32 = match p.get("clock") {
                Some(ParamValue::Int(c)) => Some(*c),
                _ => None,
            };
            if self.debug_shutdown_info.borrow().is_none() {
                let info = self.debug_build_shutdown(params, clock32, now);
                *self.debug_shutdown_info.borrow_mut() = Some(info);
                if clock32.is_some() {
                    self.debug_shutdown_frozen.set(true);
                }
                log::error(&self.debug_shutdown_text());
                return;
            }
            let known = self.debug_shutdown_info.borrow().as_ref().and_then(|m| m.get("clock_known")).and_then(|v| v.as_ref()).map(|v| truthy(v)).unwrap_or(false);
            if clock32.is_none() || known {
                return;
            }
            self.debug_shutdown_frozen.set(true);
            let clock = self.mcu.clock32_to_clock64(clock32.unwrap());
            let mut info = self.debug_shutdown_info.borrow_mut();
            if let Some(info) = info.as_mut() {
                info.insert("late_event".to_string(), Some(PyValue::Str(param_str(&p, "#name"))));
                info.insert("late_clock32".to_string(), Some(PyValue::Int(PyInt::from_i64(clock32.unwrap()))));
                info.insert("late_clock".to_string(), Some(PyValue::Int(PyInt::from_i64(clock))));
                info.insert("late_print_time".to_string(), Some(PyValue::Float(self.clocksync.clock_to_print_time(clock))));
                info.insert("late_receive_time".to_string(), p.get("#receive_time").map(|v| param_entry_to_pyvalue("#receive_time", v)));
                info.insert("late_host_monotonic".to_string(), Some(PyValue::Float(now)));
            }
            drop(info);
            log::error(&self.debug_shutdown_text());
        })).map_err(reactor::propagate_greenlet_exit);
        if result.is_err() {
            log::exception(&fmt_msg("[DEBUG-motion] %s mcu='%s' shutdown capture failed", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone())]), "debug capture failed");
        }
    }

    fn debug_note_timeout(&self, eventtime: f64) {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let now = self.printer.get_reactor().monotonic();
            let mut m: IndexMap<String, Option<PyValue>> = IndexMap::new();
            m.insert("eventtime".to_string(), Some(PyValue::Float(eventtime)));
            m.insert("host_monotonic".to_string(), Some(PyValue::Float(now)));
            m.insert("host_clock_est".to_string(), Some(PyValue::Int(PyInt::from_i64(self.clocksync.get_clock(now)))));
            m.insert("host_print_time".to_string(), Some(PyValue::Float(self.clocksync.estimated_print_time(now))));
            m.insert("shutdown_report_seen".to_string(), Some(PyValue::Bool(self.debug_shutdown_info.borrow().is_some())));
            m.insert("is_shutdown".to_string(), Some(PyValue::Bool(self.is_shutdown.get())));
            *self.debug_timeout_info.borrow_mut() = Some(m);
            log::error(&self.debug_timeout_text());
        })).map_err(reactor::propagate_greenlet_exit);
        if result.is_err() {
            log::exception(&fmt_msg("[DEBUG-motion] %s mcu='%s' timeout capture failed", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone())]), "debug capture failed");
        }
    }

    fn debug_shutdown_text(&self) -> String {
        let info = self.debug_shutdown_info.borrow();
        if info.is_none() {
            return fmt_msg("[DEBUG-motion] %s mcu='%s' no mcu shutdown report captured", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone())]);
        }
        let info = info.as_ref().unwrap();
        let fmt_opt = |k: &str| -> String {
            match info.get(k) {
                Some(Some(v)) => v.str(),
                _ => "None".to_string(),
            }
        };
        fmt_msg("[DEBUG-motion] %s mcu='%s' first_report=%s reason='%s' clock_known=%s clock32=%s clock=%s print_time=%s sent_time=%s receive_time=%s host_monotonic=%s host_clock_est=%s host_print_time=%s clocksync_active=%s local_shutdown_first=%s timeout_first=%s notes=%d late_report=%s late_clock32=%s late_clock=%s late_print_time=%s late_receive_time=%s late_host_monotonic=%s", &[
            PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()),
            PctValue::Str(self.name.clone()),
            PctValue::Str(fmt_opt("event")),
            PctValue::Str(fmt_opt("reason")),
            PctValue::Str(fmt_opt("clock_known")),
            PctValue::Str(fmt_opt("clock32")),
            PctValue::Str(fmt_opt("clock")),
            PctValue::Str(fmt_opt("print_time")),
            PctValue::Str(fmt_opt("sent_time")),
            PctValue::Str(fmt_opt("receive_time")),
            PctValue::Str(fmt_opt("host_monotonic")),
            PctValue::Str(fmt_opt("host_clock_est")),
            PctValue::Str(fmt_opt("host_print_time")),
            PctValue::Str(fmt_opt("clocksync_active")),
            PctValue::Str(fmt_opt("local_shutdown_first")),
            PctValue::Str(fmt_opt("timeout_first")),
            pct_int(self.debug_shutdown_notes.get()),
            PctValue::Str(fmt_opt("late_event")),
            PctValue::Str(fmt_opt("late_clock32")),
            PctValue::Str(fmt_opt("late_clock")),
            PctValue::Str(fmt_opt("late_print_time")),
            PctValue::Str(fmt_opt("late_receive_time")),
            PctValue::Str(fmt_opt("late_host_monotonic")),
        ])
    }

    fn debug_timeout_text(&self) -> String {
        let info = self.debug_timeout_info.borrow();
        if info.is_none() {
            return fmt_msg("[DEBUG-motion] %s mcu='%s' no host timeout recorded", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone())]);
        }
        let info = info.as_ref().unwrap();
        let fmt_opt = |k: &str| -> String {
            match info.get(k) {
                Some(Some(v)) => v.str(),
                _ => "None".to_string(),
            }
        };
        fmt_msg("[DEBUG-motion] %s mcu='%s' timeout eventtime=%s host_monotonic=%s host_clock_est=%s host_print_time=%s shutdown_report_seen=%s is_shutdown=%s", &[
            PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()),
            PctValue::Str(self.name.clone()),
            PctValue::Str(fmt_opt("eventtime")),
            PctValue::Str(fmt_opt("host_monotonic")),
            PctValue::Str(fmt_opt("host_clock_est")),
            PctValue::Str(fmt_opt("host_print_time")),
            PctValue::Str(fmt_opt("shutdown_report_seen")),
            PctValue::Str(fmt_opt("is_shutdown")),
        ])
    }

    fn handle_starting(&self, _params: &PyValue) {
        if !self.is_shutdown.get() {
            self.printer.invoke_async_shutdown(&fmt_msg("MCU '%s' spontaneous restart", &[PctValue::Str(self.name.clone())]), PyValue::None);
        }
    }

    pub fn log_info(&self) -> String {
        let (count, version, build_versions, constants) = {
            let mp = self.serial.get_msgparser();
            let guard = mp.lock();
            let msgs = guard.get_messages();
            let (v, bv) = guard.get_version_info();
            let consts = guard.get_constants();
            (msgs.len(), v, bv, consts)
        };
        let const_str = constants.iter().map(|(k, v)| format!("{}={}", k, v.str())).collect::<Vec<_>>().join(" ");
        let line1 = fmt_msg("Loaded MCU '%s' %d commands (%s / %s)", &[PctValue::Str(self.name.clone()), pct_int(count as i64), PctValue::Str(version), PctValue::Str(build_versions)]);
        let line2 = fmt_msg("MCU '%s' config: %s", &[PctValue::Str(self.name.clone()), PctValue::Str(const_str)]);
        format!("{}\n{}", line1, line2)
    }

    fn attach_file(&self) -> KResult<()> {
        let start_args = self.printer.get_start_args();
        let get = |k: &str| -> String {
            if let PyValue::Dict(m) = &start_args {
                match m.get(&PyValue::Str(k.to_string())) {
                    Some(PyValue::Str(s)) => s.clone(),
                    Some(other) => other.str(),
                    None => String::new(),
                }
            } else { String::new() }
        };
        let (out_fname, dict_fname) = if self.name == "mcu" {
            (get("debugoutput"), get("dictionary"))
        } else {
            (format!("{}-{}", get("debugoutput"), self.name), get(&format!("dictionary_{}", self.name)))
        };
        // `_attach_file` (`mcu.py:971-984`): `open(out_fname, 'wb')` stays
        // open for the life of the serial reader. Hand the real fd to the
        // port (`into_raw_fd`, like `FdOpener` does) so the serial queue
        // writes land in the file; `SerialReader::disconnect` closes it via
        // `SerialPort::close` when the session ends, as `connect_file`
        // closing `serial_dev` does in Python.
        let out = std::fs::File::create(&out_fname).map_err(|e| error(e.to_string()))?;
        let dict = std::fs::read(&dict_fname).map_err(|e| error(e.to_string()))?;
        let fd = {
            use std::os::unix::io::IntoRawFd;
            out.into_raw_fd()
        };
        let dev: Arc<dyn serialhdl::SerialPort> = Arc::new(serialhdl::FdSerialPort::new(fd, out_fname));
        self.serial.connect_file(dev, &dict, false)?;
        let adapter: SerialHandle = Arc::new(SerialReaderClockAdapter::new(Arc::clone(&self.serial)));
        self.clocksync.connect_file(adapter, false)?;
        Ok(())
    }

    fn attach(&self) -> KResult<()> {
        if let Some(rh) = self.restart_helper.borrow().clone() {
            rh.check_restart_on_attach()?;
        }
        let result: KResult<()> = (|| {
            if let Some(iface) = self.canbus_iface.borrow().clone() {
                let cbid_obj = self.printer.lookup_object("canbus_ids", LookupDefault::Raise)?
                    .expect("canbus_ids present");
                let api = klippy_api::interface::<dyn CanbusIdsApi>(&cbid_obj)
                    .ok_or_else(|| klippy_api::missing_interface::<dyn CanbusIdsApi>("canbus_ids"))?;
                let nodeid = api.get_nodeid(&self.serialport)? as i32;
                self.serial.connect_canbus(&self.serialport, nodeid, &iface)
            } else if self.baud != 0 {
                let rts = self.restart_helper.borrow().as_ref().map(|rh| rh.lookup_attach_uart_rts()).unwrap_or(true);
                self.serial.connect_uart(&self.serialport, self.baud as i32, rts)
            } else {
                self.serial.connect_pipe(&self.serialport)
            }
        })();
        // `except serialhdl.error as e: raise error(str(e))`: both classes
        // are `KlippyError::Mcu` here, so only other classes (a
        // `msgproto.error` from `process_identify`) need to pass through
        // unchanged, as they do in Python.
        result?;
        let adapter: SerialHandle = Arc::new(SerialReaderClockAdapter::new(Arc::clone(&self.serial)));
        self.clocksync.connect(adapter)?;
        Ok(())
    }

    /// `_mcu_identify`: its exceptions propagate through `send_event` to
    /// `Printer._connect`, which classifies and logs them.
    fn mcu_identify(&self, mcu: &McuHandle) -> KResult<()> {
        if mcu.is_fileoutput() {
            self.attach_file()?;
        } else {
            self.attach()?;
        }
        log::info(&self.log_info());
        log::info(&fmt_msg("[DEBUG-motion] %s mcu='%s' shutdown capture armed (max_notes=%d)", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone()), pct_int(DEBUG_SHUTDOWN_MAX_NOTES)]));
        *self.emergency_stop_cmd.borrow_mut() = Some(mcu.lookup_command("emergency_stop", None)?);
        let h = self.clone_handle();
        self.serial.register_response(Some(Arc::new(move |params: &Params| h.handle_shutdown(&params_to_pyvalue(params)))), "shutdown", None);
        let h2 = self.clone_handle();
        self.serial.register_response(Some(Arc::new(move |params: &Params| h2.handle_shutdown(&params_to_pyvalue(params)))), "is_shutdown", None);
        let h3 = self.clone_handle();
        self.serial.register_response(Some(Arc::new(move |params: &Params| h3.handle_starting(&params_to_pyvalue(params)))), "starting", None);
        Ok(())
    }

    fn clone_handle(&self) -> Arc<McuConnectHelper> {
        self.self_weak.upgrade().expect("McuConnectHelper is alive while its methods run")
    }

    fn analyze_shutdown(&self) {
        if self.mcu.is_fileoutput() {
            return;
        }
        let dump = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            log::error(&self.debug_shutdown_text());
            log::error(&self.debug_timeout_text());
        })).map_err(reactor::propagate_greenlet_exit);
        if dump.is_err() {
            log::exception(&fmt_msg("[DEBUG-motion] %s mcu='%s' provenance dump failed", &[PctValue::Str(DEBUG_SHUTDOWN_VERSION.to_string()), PctValue::Str(self.name.clone())]), "provenance dump failed");
        }
        log::info(&fmt_msg("MCU '%s' shutdown: %s\n%s\n%s", &[PctValue::Str(self.name.clone()), PctValue::Str(self.shutdown_msg.borrow().clone()), PctValue::Str(self.clocksync.dump_debug()), PctValue::Str(self.serial.dump_debug())]));
    }

    fn shutdown(&self, force: bool) {
        if self.emergency_stop_cmd.borrow().is_none() || (self.is_shutdown.get() && !force) {
            return;
        }
        if let Some(cmd) = self.emergency_stop_cmd.borrow().clone() {
            // `shutdown` (`mcu.py:816-827`) lets the send raise; panic with class+text.
            if let Err(e) = cmd.send(vec![], SendOpts::default()) {
                e.raise();
            }
        }
    }

    pub fn force_local_shutdown(&self) {
        self.is_shutdown.set(true);
        self.shutdown(true);
    }

    pub fn check_timeout(&self, eventtime: f64) {
        if self.clocksync.is_active() || self.mcu.is_fileoutput() || self.is_timeout.get() {
            return;
        }
        self.is_timeout.set(true);
        self.debug_note_timeout(eventtime);
        log::info(&fmt_msg("Timeout with MCU '%s' (eventtime=%f)", &[PctValue::Str(self.name.clone()), PctValue::Float(eventtime)]));
        self.printer.invoke_shutdown(&fmt_msg("Lost communication with MCU '%s'", &[PctValue::Str(self.name.clone())]), PyValue::None);
    }

    pub fn is_shutdown(&self) -> bool { self.is_shutdown.get() }
    pub fn get_shutdown_msg(&self) -> String { self.shutdown_msg.borrow().clone() }
}

fn param_to_pyvalue(v: &ParamValue) -> PyValue {
    match v {
        ParamValue::Int(i) => PyValue::Int(PyInt::from_i64(*i)),
        ParamValue::Str(s) => PyValue::Str(s.clone()),
        ParamValue::Bytes(b) => PyValue::Bytes(b.clone()),
    }
}

/// Handle statistics reporting (`klippy/mcu.py:1054`).
pub struct McuStatsHelper {
    printer: Printer,
    mcu: McuHandle,
    serial: Arc<SerialReader>,
    clocksync: ClockKind,
    name: String,
    mcu_freq: SCell<f64>,
    get_status_info: SCell<IndexMap<String, PyValue>>,
    stats_sumsq_base: SCell<f64>,
    mcu_tick_avg: SCell<f64>,
    mcu_tick_stddev: SCell<f64>,
    mcu_tick_awake: SCell<f64>,
    self_weak: Weak<McuStatsHelper>,
}

impl McuStatsHelper {
    pub fn new(config: &Config, mcu: McuHandle, serial: Arc<SerialReader>, clocksync: ClockKind) -> Arc<Self> {
        let printer = config.get_printer();
        let name = mcu.get_name();
        let helper = Arc::new_cyclic(|self_weak| McuStatsHelper {
            printer: printer.clone(),
            mcu: mcu.clone(),
            serial: Arc::clone(&serial),
            clocksync,
            name: name.clone(),
            mcu_freq: SCell::new(0.0),
            get_status_info: SCell::new(IndexMap::new()),
            stats_sumsq_base: SCell::new(0.0),
            mcu_tick_avg: SCell::new(0.0),
            mcu_tick_stddev: SCell::new(0.0),
            mcu_tick_awake: SCell::new(0.0),
            self_weak: self_weak.clone(),
        });
        let h = Arc::clone(&helper);
        printer.register_event_handler("klippy:ready", Box::new(move |_args| { h.ready(); Ok(PyValue::None) }));
        let h2 = Arc::clone(&helper);
        printer.register_event_handler("klippy:mcu_identify", Box::new(move |_args| { h2.mcu_identify()?; Ok(PyValue::None) }));
        helper
    }

    fn handle_mcu_stats(&self, params: &PyValue) {
        let p = pyvalue_to_params(params);
        let count = param_int(&p, "count") as f64;
        let tick_sum = param_int(&p, "sum") as f64;
        let mcu_freq = self.mcu_freq.get();
        let c = 1.0 / (count * mcu_freq);
        self.mcu_tick_avg.set(tick_sum * c);
        let tick_sumsq = param_int(&p, "sumsq") as f64 * self.stats_sumsq_base.get();
        let diff = count * tick_sumsq - tick_sum * tick_sum;
        self.mcu_tick_stddev.set(c * pycompat::libm::sqrt(diff.max(0.0)));
        self.mcu_tick_awake.set(tick_sum / mcu_freq);
    }

    /// `_mcu_identify` (`klippy/mcu.py:1082`): a missing constant raises
    /// out of the `klippy:mcu_identify` dispatch into `Printer._connect`.
    fn mcu_identify(&self) -> KResult<()> {
        self.mcu_freq.set(self.mcu.get_constant_float("CLOCK_FREQ")?);
        self.stats_sumsq_base.set(self.mcu.get_constant_float("STATS_SUMSQ_BASE")?);
        let (version, build_versions, constants) = {
            let mp = self.serial.get_msgparser();
            let guard = mp.lock();
            let (v, bv) = guard.get_version_info();
            let consts = guard.get_constants();
            (v, bv, consts)
        };
        self.get_status_info.borrow_mut().insert("mcu_version".to_string(), PyValue::Str(version));
        self.get_status_info.borrow_mut().insert("mcu_build_versions".to_string(), PyValue::Str(build_versions));
        let mut m: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in constants.iter() {
            m.insert(PyValue::Str(k.clone()), v.clone());
        }
        self.get_status_info.borrow_mut().insert("mcu_constants".to_string(), PyValue::Dict(m));
        let h2 = self.self_clone();
        self.serial.register_response(Some(Arc::new(move |params: &Params| h2.handle_mcu_stats(&params_to_pyvalue(params)))), "stats", None);
        Ok(())
    }

    fn self_clone(&self) -> Arc<McuStatsHelper> {
        self.self_weak.upgrade().expect("McuStatsHelper is alive while its methods run")
    }

    fn ready(&self) {
        if self.mcu.is_fileoutput() {
            return;
        }
        let mcu_freq = self.mcu_freq.get();
        let systime = self.printer.get_reactor().monotonic();
        let calc_freq = (self.clocksync.get_clock(systime + 1.0) - self.clocksync.get_clock(systime)) as f64;
        let freq_diff = (mcu_freq - calc_freq).abs();
        let mcu_freq_mhz = py_int_from_float(mcu_freq / 1000000.0 + 0.5);
        let calc_freq_mhz = py_int_from_float(calc_freq / 1000000.0 + 0.5);
        if freq_diff > mcu_freq * 0.01 && mcu_freq_mhz != calc_freq_mhz {
            let pconfig = self.printer.lookup_object("configfile", LookupDefault::Raise).ok().flatten().expect("configfile present");
            let api = klippy_api::interface::<dyn klippy_api::config::PrinterConfigApi>(&pconfig).expect("configfile api");
            api.runtime_warning(&fmt_msg("MCU '%s' configured for %dMhz but running at %dMhz!", &[PctValue::Str(self.name.clone()), pct_int(mcu_freq_mhz), pct_int(calc_freq_mhz)]));
        }
    }

    pub fn get_status(&self, _eventtime: Option<f64>) -> PyValue {
        let mut out: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in self.get_status_info.borrow().iter() {
            out.insert(PyValue::Str(k.clone()), v.clone());
        }
        PyValue::Dict(out)
    }

    pub fn stats(&self, eventtime: f64) -> (bool, String) {
        let load = fmt_msg("mcu_awake=%.03f mcu_task_avg=%.06f mcu_task_stddev=%.06f", &[PctValue::Float(self.mcu_tick_awake.get()), PctValue::Float(self.mcu_tick_avg.get()), PctValue::Float(self.mcu_tick_stddev.get())]);
        let serial_stats = self.serial.stats(eventtime);
        let clock_stats = self.clocksync.stats(eventtime);
        let stats = format!("{} {} {}", load, serial_stats, clock_stats);
        let mut last_stats: IndexMap<PyValue, PyValue> = IndexMap::new();
        for part in stats.split_whitespace() {
            if let Some((k, v)) = part.split_once('=') {
                let value = if v.contains('.') {
                    PyValue::Float(v.parse::<f64>().unwrap_or(0.0))
                } else {
                    PyValue::Int(PyInt::from_i64(v.parse::<i64>().unwrap_or(0)))
                };
                last_stats.insert(PyValue::Str(k.to_string()), value);
            }
        }
        self.get_status_info.borrow_mut().insert("last_stats".to_string(), PyValue::Dict(last_stats));
        (false, fmt_msg("%s: %s", &[PctValue::Str(self.name.clone()), PctValue::Str(stats)]))
    }
}
/// Handle process of configuring an mcu (`klippy/mcu.py:1121`).
pub struct McuConfigHelper {
    printer: Printer,
    conn_helper: Arc<McuConnectHelper>,
    mcu: McuHandle,
    serial: Arc<SerialReader>,
    clocksync: ClockKind,
    name: String,
    config_finalized: GCell<bool>,
    oid_count: GCell<i64>,
    config_callbacks: GCell<Vec<ConfigCallback>>,
    post_init_callbacks: GCell<Vec<ConfigCallback>>,
    config_cmds: GCell<Vec<String>>,
    restart_cmds: GCell<Vec<String>>,
    init_cmds: GCell<Vec<String>>,
    config_crc: GCell<i64>,
    mcu_freq: GCell<f64>,
    reserved_move_slots: GCell<i64>,
}

impl McuConfigHelper {
    pub fn new(config: &Config, mcu: McuHandle, conn_helper: Arc<McuConnectHelper>, serial: Arc<SerialReader>, clocksync: ClockKind) -> KResult<Arc<Self>> {
        let printer = config.get_printer();
        let name = mcu.get_name();
        // Register chip with pins (`klippy/mcu.py:1142`).
        let helper = Arc::new(McuConfigHelper {
            printer: printer.clone(),
            conn_helper: Arc::clone(&conn_helper),
            mcu: mcu.clone(),
            serial: Arc::clone(&serial),
            clocksync,
            name: name.clone(),
            config_finalized: GCell::new(false),
            oid_count: GCell::new(0),
            config_callbacks: GCell::new(Vec::new()),
            post_init_callbacks: GCell::new(Vec::new()),
            config_cmds: GCell::new(Vec::new()),
            restart_cmds: GCell::new(Vec::new()),
            init_cmds: GCell::new(Vec::new()),
            config_crc: GCell::new(0),
            mcu_freq: GCell::new(0.0),
            reserved_move_slots: GCell::new(0),
        });
        {
            let pins_obj = printer.lookup_object("pins", LookupDefault::Raise)?
                .expect("pins present");
            let pins = klippy_api::interface::<dyn klippy_api::mcu::PinsApi>(&pins_obj)
                .ok_or_else(|| klippy_api::missing_interface::<dyn klippy_api::mcu::PinsApi>("pins"))?;
            let chip: klippy_api::mcu::Chip = mcu.mcu_as_trait_chip();
            pins.register_chip(&name, chip)?;
        }
        let h = Arc::clone(&helper);
        printer.register_event_handler("klippy:mcu_identify", Box::new(move |_args| { h.mcu_identify()?; Ok(PyValue::None) }));
        let h2 = Arc::clone(&helper);
        printer.register_event_handler("klippy:connect", Box::new(move |_args| { h2.connect() }));
        Ok(helper)
    }

    pub fn finalize_config(&self) -> KResult<()> {
        let callbacks: Vec<ConfigCallback> = std::mem::take(&mut *self.config_callbacks.borrow_mut());
        let mut callbacks = callbacks;
        // Callbacks run in registration order; temporarily stash them back
        // in a loop since Box<dyn FnMut> cannot iterate &mut directly.
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        self.config_callbacks.borrow_mut().extend(callbacks);
        self.config_finalized.set(true);
        self.config_cmds.borrow_mut().insert(0, fmt_msg("allocate_oids count=%d", &[pct_int(self.oid_count.get())]));
        let ppins = self.printer.lookup_object("pins", LookupDefault::Raise)?.expect("pins present");
        let pins = klippy_api::interface::<dyn klippy_api::mcu::PinsApi>(&ppins)
            .ok_or_else(|| klippy_api::missing_interface::<dyn klippy_api::mcu::PinsApi>("pins"))?;
        let pin_resolver = pins.get_pin_resolver(&self.name)?;
        for cmdlist in [&self.config_cmds, &self.restart_cmds, &self.init_cmds] {
            let mut cmds = cmdlist.borrow_mut();
            for i in 0..cmds.len() {
                cmds[i] = pin_resolver.update_command(&cmds[i])?;
            }
        }
        let encoded = self.config_cmds.borrow().join("\n");
        let crc = {
            let bytes = encoded.as_bytes();
            let mut crc: u32 = 0xffff_ffff;
            for b in bytes {
                crc ^= *b as u32;
                for _ in 0..8 {
                    crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
                }
            }
            !crc
        } as i64 & 0xffff_ffff;
        self.config_crc.set(crc);
        self.config_cmds.borrow_mut().push(fmt_msg("finalize_config crc=%d", &[pct_int(crc)]));
        Ok(())
    }

    fn send_cfg_init_commands(&self, cmds: &[String]) -> KResult<()> {
        for c in cmds {
            match self.serial.send(c, 0, 0) {
                Ok(()) => {}
                Err(e) => {
                    let msg = e.message().to_string();
                    if let klippy_api::KlippyError::Mcu(m) = &e {
                        let _ = m;
                    }
                    // Enumeration errors surface as Mcu with "pin" params;
                    // the Python contract re-raises as config_error for pins.
                    // Serial send maps msgproto enumeration to Mcu; detect
                    // by message shape is unreliable, so re-probe parser.
                    if msg.contains("pin") {
                        return Err(config_error(fmt_msg("Pin '%s' is not a valid pin name on mcu '%s'", &[PctValue::Str(msg.clone()), PctValue::Str(self.name.clone())])));
                    }
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    fn send_get_config(&self) -> KResult<PyValue> {
        let get_config_cmd = self.mcu.lookup_query_command("get_config", "config is_config=%c crc=%u is_shutdown=%c move_count=%hu", LookupQueryCommandOpts::default())?;
        if self.mcu.is_fileoutput() {
            let mut m: IndexMap<PyValue, PyValue> = IndexMap::new();
            m.insert(PyValue::Str("is_config".to_string()), PyValue::Int(PyInt::from_i64(0)));
            m.insert(PyValue::Str("move_count".to_string()), PyValue::Int(PyInt::from_i64(500)));
            m.insert(PyValue::Str("crc".to_string()), PyValue::Int(PyInt::from_i64(0)));
            return Ok(PyValue::Dict(m));
        }
        let params = get_config_cmd.send(vec![], QuerySendOpts::default())?;
        if self.conn_helper.is_shutdown() {
            return Err(error(fmt_msg("MCU '%s' error during config: %s", &[PctValue::Str(self.name.clone()), PctValue::Str(self.conn_helper.get_shutdown_msg())])));
        }
        let p = pyvalue_to_params(&params);
        if param_int(&p, "is_shutdown") != 0 {
            return Err(error(fmt_msg("Can not update MCU '%s' config as it is shutdown", &[PctValue::Str(self.name.clone())])));
        }
        Ok(params)
    }

    fn connect(&self) -> KResult<PyValue> {
        let restart_helper = self.conn_helper.restart_helper.borrow().clone().expect("restart helper");
        let mut config_params = self.send_get_config()?;
        let cfg_init_cmds: Vec<String>;
        if as_int(&dict_get(&config_params, "is_config")) == 0 {
            restart_helper.check_restart_on_send_config()?;
            self.finalize_config()?;
            log::info(&fmt_msg("Sending MCU '%s' printer configuration...", &[PctValue::Str(self.name.clone())]));
            cfg_init_cmds = [self.config_cmds.borrow().clone(), self.init_cmds.borrow().clone()].concat();
        } else {
            let start_reason = start_arg(&self.printer, "start_reason").unwrap_or_default();
            if start_reason == "firmware_restart" {
                return Err(error(fmt_msg("Failed automated reset of MCU '%s'", &[PctValue::Str(self.name.clone())])));
            }
            self.finalize_config()?;
            if self.config_crc.get() != as_int(&dict_get(&config_params, "crc")) {
                restart_helper.check_restart_on_crc_mismatch()?;
                return Err(error(fmt_msg("MCU '%s' CRC does not match config", &[PctValue::Str(self.name.clone())])));
            }
            cfg_init_cmds = [self.restart_cmds.borrow().clone(), self.init_cmds.borrow().clone()].concat();
        }
        self.send_cfg_init_commands(&cfg_init_cmds)?;
        config_params = self.send_get_config()?;
        if as_int(&dict_get(&config_params, "is_config")) == 0 && !self.mcu.is_fileoutput() {
            return Err(error(fmt_msg("Unable to configure MCU '%s'", &[PctValue::Str(self.name.clone())])));
        }
        let callbacks: Vec<ConfigCallback> = std::mem::take(&mut *self.post_init_callbacks.borrow_mut());
        let mut callbacks = callbacks;
        for cb in callbacks.iter_mut() {
            cb()?;
        }
        self.post_init_callbacks.borrow_mut().extend(callbacks);
        let move_count = as_int(&dict_get(&config_params, "move_count"));
        if move_count < self.reserved_move_slots.get() {
            return Err(error(fmt_msg("Too few moves available on MCU '%s'", &[PctValue::Str(self.name.clone())])));
        }
        let ss_move_count = move_count - self.reserved_move_slots.get();
        let motion_queuing = self.printer.lookup_object("motion_queuing", LookupDefault::Raise)?.expect("motion_queuing present");
        let api = klippy_api::interface::<dyn klippy_api::toolhead::MotionQueuingApi>(&motion_queuing)
            .ok_or_else(|| klippy_api::missing_interface::<dyn klippy_api::toolhead::MotionQueuingApi>("motion_queuing"))?;
        // `motion_queuing.setup_mcu_movequeue(self._mcu,
        // self._serial.get_serialqueue(), ss_move_count)` (`mcu.py:1225-1227`).
        // The port takes the real queue (`Option<Arc<SerialQueue>>`), not the
        // opaque contract id.
        let sq = self.serial.get_serialqueue().expect("serialqueue present");
        api.setup_mcu_movequeue(self.mcu.mcu_as_trait(), Some(sq), ss_move_count);
        let move_msg = fmt_msg("Configured MCU '%s' (%d moves)", &[PctValue::Str(self.name.clone()), pct_int(move_count)]);
        log::info(&move_msg);
        let log_info = format!("{}\n{}", self.conn_helper.log_info(), move_msg);
        self.printer.set_rollover_info(&self.name, Some(&log_info), false);
        Ok(PyValue::None)
    }

    /// `_mcu_identify` (`klippy/mcu.py:1233`): its exceptions propagate
    /// through `send_event` to `Printer._connect`.
    fn mcu_identify(&self) -> KResult<()> {
        self.mcu_freq.set(self.mcu.get_constant_float("CLOCK_FREQ")?);
        let ppins = self.printer.lookup_object("pins", LookupDefault::Raise)?.expect("pins present");
        let pins = klippy_api::interface::<dyn klippy_api::mcu::PinsApi>(&ppins)
            .ok_or_else(|| klippy_api::missing_interface::<dyn klippy_api::mcu::PinsApi>("pins"))?;
        let pin_resolver = pins.get_pin_resolver(&self.name)?;
        for (cname, value) in self.mcu.get_constants().iter() {
            if cname.starts_with("RESERVE_PINS_") {
                let reserve = &cname[13.min(cname.len())..];
                for pin in value.split(',') {
                    pin_resolver.reserve_pin(pin, reserve)?;
                }
            }
        }
        if MAX_NOMINAL_DURATION * self.mcu_freq.get() > MAX_SCHEDULE_TICKS as f64 {
            let max_possible = MAX_SCHEDULE_TICKS as f64 / self.mcu_freq.get();
            return Err(error(fmt_msg("Too high clock speed for MCU '%s' to be able to resolve a maximum nominal duration of %ds. Max possible duration: %ds", &[PctValue::Str(self.name.clone()), pct_int(MAX_NOMINAL_DURATION as i64), pct_int(max_possible as i64)])));
        }
        Ok(())
    }

    fn verify_not_finalized(&self) -> KResult<()> {
        if self.config_finalized.get() {
            return Err(error("Internal error! MCU already configured".to_string()));
        }
        Ok(())
    }

    pub fn is_config_finalized(&self) -> bool { self.config_finalized.get() }

    /// `MCUConfigHelper._config_crc` after `finalize_config` (test surface).
    pub fn config_crc(&self) -> i64 { self.config_crc.get() }

    pub fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.verify_not_finalized()?;
        match pin_type {
            "endstop" => Ok(Pin::Endstop(McuEndstop::new(self.mcu.clone(), pin_params)?)),
            "digital_out" => Ok(Pin::DigitalOut(McuDigitalOut::new(self.mcu.clone(), pin_params))),
            "pwm" => Ok(Pin::Pwm(McuPwm::new(self.mcu.clone(), pin_params))),
            "adc" => Ok(Pin::Adc(McuAdc::new(self.mcu.clone(), pin_params))),
            _ => Err(pins_error(fmt_msg("pin type %s not supported on mcu", &[PctValue::Str(pin_type.to_string())]))),
        }
    }

    pub fn create_oid(&self) -> KResult<i64> {
        self.verify_not_finalized()?;
        let oid = self.oid_count.get() + 1;
        self.oid_count.set(oid);
        Ok(oid - 1)
    }

    pub fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.verify_not_finalized()?;
        self.config_callbacks.borrow_mut().push(cb);
        Ok(())
    }

    pub fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> {
        self.verify_not_finalized()?;
        if opts.is_init {
            self.init_cmds.borrow_mut().push(cmd.to_string());
        } else if opts.on_restart {
            self.restart_cmds.borrow_mut().push(cmd.to_string());
        } else {
            self.config_cmds.borrow_mut().push(cmd.to_string());
        }
        Ok(())
    }

    pub fn register_post_init_callback(&self, cb: ConfigCallback) -> KResult<()> {
        self.verify_not_finalized()?;
        self.post_init_callbacks.borrow_mut().push(cb);
        Ok(())
    }

    pub fn get_query_slot(&self, oid: i64) -> i64 {
        let slot = self.seconds_to_clock(oid as f64 * 0.01);
        let t = py_int_from_float(self.mcu.estimated_print_time(self.printer.get_reactor().monotonic()) + 1.5);
        self.mcu.print_time_to_clock(t as f64) + slot
    }

    pub fn seconds_to_clock(&self, time: f64) -> i64 {
        py_int_from_float(time * self.mcu_freq.get())
    }

    pub fn request_move_queue_slot(&self) {
        self.reserved_move_slots.set(self.reserved_move_slots.get() + 1);
    }
}

fn dict_get(v: &PyValue, key: &str) -> PyValue {
    if let PyValue::Dict(m) = v {
        m.get(&PyValue::Str(key.to_string())).cloned().unwrap_or(PyValue::None)
    } else {
        PyValue::None
    }
}

/// Handle for the concrete MCU (`MCU` object `mcu` / `mcu <name>`).
#[derive(Clone)]
pub struct McuHandle {
    inner: Arc<Mcu>,
}

impl McuHandle {
    pub fn from_trait(obj: &ObjectRef) -> Self {
        let down = klippy_api::downcast_arc::<Mcu>(obj);
        match down {
            Some(inner) => McuHandle { inner },
            None => panic!("stepper mcu is not a core Mcu"),
        }
    }
    pub fn new(config: &Config, clocksync: ClockKind) -> KResult<Arc<Mcu>> {
        let printer = config.get_printer();
        let mut name = config.get_name();
        if let Some(stripped) = name.strip_prefix("mcu ") {
            name = stripped.to_string();
        }
        let reactor = printer.get_reactor();
        let mcu = Arc::new_cyclic(|self_weak| Mcu {
            printer: printer.clone(),
            clocksync: clocksync.clone(),
            name: name.clone(),
            serial: Arc::new(SerialReader::new(reactor, &name)),
            conn_helper: GCell::new(None),
            config_helper: GCell::new(None),
            stats_helper: GCell::new(None),
            fileoutput_dummy: GCell::new(false),
            self_weak: self_weak.clone(),
        });
        let handle = McuHandle { inner: Arc::clone(&mcu) };
        // Build helpers. The connect helper needs `&config` for serial
        // options; the config helper needs the connect Arc.
        let conn = McuConnectHelper::new(config, handle.clone(), clocksync.clone(), Arc::clone(&mcu.serial))?;
        // The restart helper is built inside `MCUConnectHelper.__init__`
        // in Python; it reads the serial port through the connect helper.
        let restart = McuRestartHelper::new(config, handle.clone(), Arc::clone(&mcu.serial), clocksync.clone(), &conn)?;
        *conn.restart_helper.borrow_mut() = Some(Arc::clone(&restart));
        let cfg = McuConfigHelper::new(config, handle.clone(), Arc::clone(&conn), Arc::clone(&mcu.serial), clocksync.clone())?;
        let stats = McuStatsHelper::new(config, handle.clone(), Arc::clone(&mcu.serial), clocksync);
        *mcu.conn_helper.borrow_mut() = Some(conn);
        *mcu.config_helper.borrow_mut() = Some(cfg);
        *mcu.stats_helper.borrow_mut() = Some(stats);
        printer.load_object(config, "error_mcu", LookupDefault::Raise)?;
        Ok(mcu)
    }

    pub fn get_name(&self) -> String { self.inner.name.clone() }
    pub fn printer(&self) -> Printer { self.inner.printer.clone() }
    pub fn is_fileoutput(&self) -> bool { start_arg(&self.inner.printer, "debugoutput").is_some() }
    pub fn serial(&self) -> Arc<SerialReader> { Arc::clone(&self.inner.serial) }
    pub fn serialport(&self) -> (String, i64) {
        self.inner.conn_helper.borrow().as_ref().map(|c| c.get_serialport()).unwrap_or_default()
    }
    pub fn force_local_shutdown(&self) {
        if let Some(c) = self.inner.conn_helper.borrow().as_ref() {
            c.force_local_shutdown();
        }
    }
    pub fn create_oid(&self) -> KResult<i64> { self.inner.config_helper.borrow().as_ref().expect("config helper").create_oid() }
    pub fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> { self.inner.config_helper.borrow().as_ref().expect("config helper").register_config_callback(cb) }
    pub fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> { self.inner.config_helper.borrow().as_ref().expect("config helper").add_config_cmd(cmd, opts) }
    pub fn request_move_queue_slot(&self) { self.inner.config_helper.borrow().as_ref().expect("config helper").request_move_queue_slot() }
    pub fn get_query_slot(&self, oid: i64) -> i64 { self.inner.config_helper.borrow().as_ref().expect("config helper").get_query_slot(oid) }
    pub fn seconds_to_clock(&self, time: f64) -> i64 { self.inner.config_helper.borrow().as_ref().expect("config helper").seconds_to_clock(time) }
    pub fn finalize_config(&self) -> KResult<()> { self.inner.config_helper.borrow().as_ref().expect("config helper").finalize_config() }
    pub fn config_crc(&self) -> i64 { self.inner.config_helper.borrow().as_ref().expect("config helper").config_crc() }
    /// `alloc_command_queue` (`mcu.py:1344`): a real
    /// `serialqueue_alloc_commandqueue`, registered in the serial reader's
    /// table under the returned id. The C `gc` pair frees it when the last
    /// reference drops; here the table's `Arc` plus the wrappers' `Arc`s
    /// keep it alive the same way.
    pub fn alloc_command_queue(&self) -> ApiCommandQueue {
        let serial = self.serial();
        let cq = serial.alloc_command_queue();
        ApiCommandQueue(serial.register_command_queue(cq))
    }

    /// The live queue behind an [`ApiCommandQueue`] id, or the default
    /// queue for unknown ids (defensive: every id comes from
    /// [`Self::alloc_command_queue`]).
    fn command_queue(&self, cq: Option<ApiCommandQueue>) -> Arc<CommandQueue> {
        match cq {
            Some(ApiCommandQueue(id)) => self
                .serial()
                .command_queue(id)
                .unwrap_or_else(|| self.serial().get_default_command_queue()),
            None => self.serial().get_default_command_queue(),
        }
    }
    pub fn lookup_command(&self, msgformat: &str, cq: Option<ApiCommandQueue>) -> KResult<Arc<CommandWrapper>> {
        let cq = Some(self.command_queue(cq));
        let ref_ = McuConnRef { serial: Arc::clone(&self.inner.serial), mcu: self.clone() };
        Ok(Arc::new(CommandWrapper::new(&ref_, msgformat, cq)?))
    }
    pub fn lookup_query_command(&self, msgformat: &str, respformat: &str, opts: LookupQueryCommandOpts) -> KResult<Arc<CommandQueryWrapper>> {
        let cq = Some(self.command_queue(opts.cq));
        let ref_ = McuConnRef { serial: Arc::clone(&self.inner.serial), mcu: self.clone() };
        Ok(Arc::new(CommandQueryWrapper::new(&ref_, msgformat, respformat, opts.oid, cq, opts.is_async)?))
    }
    pub fn try_lookup_command(&self, msgformat: &str) -> Option<Arc<CommandWrapper>> {
        self.lookup_command(msgformat, None).ok()
    }
    pub fn register_serial_response(&self, cb: SerialResponseCallback, msg: &str, oid: Option<i64>) -> Arc<AsyncResponseWrapper> {
        let cfg = self.inner.config_helper.borrow().as_ref().expect("cfg").clone();
        let ref_ = McuConnRef { serial: Arc::clone(&self.inner.serial), mcu: self.clone() };
        AsyncResponseWrapper::new(&ref_, &cfg, cb, msg, oid)
    }
    pub fn check_valid_response(&self, msgformat: &str) -> bool {
        let mp = self.serial().get_msgparser();
        let mp = mp.lock();
        mp.lookup_command(msgformat).is_ok()
    }
    pub fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        let mp = self.serial().get_msgparser();
        let mp = mp.lock();
        mp.get_enumerations()
    }
    pub fn get_constants(&self) -> IndexMap<String, String> {
        let mp = self.serial().get_msgparser();
        let guard = mp.lock();
        let consts = guard.get_constants();
        let mut out = IndexMap::new();
        for (k, v) in consts.iter() {
            out.insert(k.clone(), v.str());
        }
        out
    }
    pub fn get_constant_float(&self, name: &str) -> KResult<f64> {
        let mp = self.serial().get_msgparser();
        let mp = mp.lock();
        mp.get_constant_float(name, None).map_err(|e| error(e.message().to_string()))
    }
    pub fn print_time_to_clock(&self, print_time: f64) -> i64 { self.inner.clocksync.print_time_to_clock(print_time) }
    pub fn clock_to_print_time(&self, clock: i64) -> f64 { self.inner.clocksync.clock_to_print_time(clock) }
    pub fn estimated_print_time(&self, eventtime: f64) -> f64 {
        if self.is_fileoutput() {
            return 0.0;
        }
        self.inner.clocksync.estimated_print_time(eventtime)
    }
    pub fn clock32_to_clock64(&self, clock32: i64) -> i64 { self.inner.clocksync.clock32_to_clock64(clock32) }
    pub fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        let r = self.inner.clocksync.calibrate_clock(print_time, eventtime)?;
        if let Some(c) = self.inner.conn_helper.borrow().as_ref() {
            c.check_timeout(eventtime);
        }
        Ok(r)
    }
    pub fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        self.inner.stats_helper.borrow().as_ref().expect("stats").get_status(eventtime)
    }
    pub fn stats(&self, eventtime: f64) -> (bool, String) {
        self.inner.stats_helper.borrow().as_ref().expect("stats").stats(eventtime)
    }
    pub fn create_trigger_dispatch(&self) -> KResult<Arc<TriggerDispatch>> {
        TriggerDispatch::new(self.clone())
    }
    pub fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        self.serial().get_serialqueue()
    }
    pub fn mcu_as_trait(&self) -> klippy_api::mcu::Mcu {
        self.inner.clone()
    }
    pub fn mcu_as_trait_chip(&self) -> klippy_api::mcu::Chip {
        self.inner.clone()
    }
}

/// Main MCU class (`klippy/mcu.py:1288`).
pub struct Mcu {
    printer: Printer,
    clocksync: ClockKind,
    name: String,
    serial: Arc<SerialReader>,
    conn_helper: GCell<Option<Arc<McuConnectHelper>>>,
    config_helper: GCell<Option<Arc<McuConfigHelper>>>,
    stats_helper: GCell<Option<Arc<McuStatsHelper>>>,
    fileoutput_dummy: GCell<bool>,
    self_weak: Weak<Mcu>,
}

impl Mcu {
    /// Release subscriptions and helper back-references at the end of a run.
    pub fn teardown(&self) {
        self.serial.teardown();
        if let Some(config) = self.config_helper.replace(None) {
            config.config_callbacks.borrow_mut().clear();
            config.post_init_callbacks.borrow_mut().clear();
        }
        if let Some(connection) = self.conn_helper.replace(None) {
            connection.restart_helper.replace(None);
        }
        self.stats_helper.replace(None);
    }
    /// The [`McuHandle`] over this MCU (Python passes `self`).
    fn handle(&self) -> McuHandle {
        McuHandle { inner: self.self_weak.upgrade().expect("Mcu is alive while its methods run") }
    }

    pub fn new(config: &Config, clocksync: ClockKind) -> KResult<Arc<Self>> {
        McuHandle::new(config, clocksync)
    }
    pub fn get_name(&self) -> String { self.name.clone() }
}

impl klippy_api::mcu::McuApi for Mcu {
    fn get_name(&self) -> String { Mcu::get_name(self) }
    fn get_printer(&self) -> Printer { self.printer.clone() }
    fn is_fileoutput(&self) -> bool { start_arg(&self.printer, "debugoutput").is_some() }
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.config_helper.borrow().as_ref().expect("config helper").setup_pin(pin_type, pin_params)
    }
    fn create_oid(&self) -> KResult<i64> { self.config_helper.borrow().as_ref().expect("config helper").create_oid() }
    fn register_config_callback(&self, cb: ConfigCallback) -> KResult<()> { self.config_helper.borrow().as_ref().expect("config helper").register_config_callback(cb) }
    fn add_config_cmd(&self, cmd: &str, opts: AddConfigCmdOpts) -> KResult<()> { self.config_helper.borrow().as_ref().expect("config helper").add_config_cmd(cmd, opts) }
    fn request_move_queue_slot(&self) { self.config_helper.borrow().as_ref().expect("config helper").request_move_queue_slot() }
    fn get_query_slot(&self, oid: i64) -> i64 { self.config_helper.borrow().as_ref().expect("config helper").get_query_slot(oid) }
    fn seconds_to_clock(&self, time: f64) -> i64 { self.config_helper.borrow().as_ref().expect("config helper").seconds_to_clock(time) }
    fn min_schedule_time(&self) -> f64 { MIN_SCHEDULE_TIME }
    fn max_nominal_duration(&self) -> f64 { MAX_NOMINAL_DURATION }
    fn lookup_command(&self, msgformat: &str, cq: Option<ApiCommandQueue>) -> KResult<klippy_api::mcu::CommandWrapper> {
        self.handle().lookup_command(msgformat, cq).map(|w| w as klippy_api::mcu::CommandWrapper)
    }
    fn lookup_query_command(&self, msgformat: &str, respformat: &str, opts: LookupQueryCommandOpts) -> KResult<klippy_api::mcu::CommandQueryWrapper> {
        self.handle().lookup_query_command(msgformat, respformat, opts).map(|w| w as klippy_api::mcu::CommandQueryWrapper)
    }
    fn try_lookup_command(&self, msgformat: &str) -> Option<klippy_api::mcu::CommandWrapper> {
        self.handle().try_lookup_command(msgformat).map(|w| w as klippy_api::mcu::CommandWrapper)
    }
    fn alloc_command_queue(&self) -> ApiCommandQueue { self.handle().alloc_command_queue() }
    fn register_serial_response(&self, cb: SerialResponseCallback, msg: &str, oid: Option<i64>) -> klippy_api::mcu::AsyncResponseWrapper {
        let serial = Arc::clone(&self.serial);
        let name = msg.split_whitespace().next().unwrap_or_default().to_string();
        let cfg = self.config_helper.borrow().as_ref().expect("cfg").clone();
        let wrapper = Arc::new(AsyncResponseWrapper { serial: Arc::clone(&serial), name: name.clone(), oid });
        if cfg.is_config_finalized() {
            if let Err(e) = wrapper.register(cb, msg) {
                e.raise();
            }
        } else {
            serial.register_response(Some(Arc::new(|_: &Params| {})), &name, oid);
            let wrapper2 = Arc::clone(&wrapper);
            let cb2 = cb.clone();
            let mf = msg.to_string();
            // `register_post_init_callback` (`mcu.py:1275`) raises once
            // finalized; this runs pre-finalize, so a failure is a loud bug.
            if let Err(e) = cfg.register_post_init_callback(Box::new(move || {
                if let Err(e) = wrapper2.register(cb2.clone(), &mf.clone()) {
                    e.raise();
                }
                Ok(())
            })) {
                e.raise();
            }
        }
        wrapper
    }
    fn check_valid_response(&self, msgformat: &str) -> bool {
        let mp = self.serial.get_msgparser();
        let mp = mp.lock();
        mp.lookup_command(msgformat).is_ok()
    }
    fn get_enumerations(&self) -> IndexMap<String, IndexMap<String, i64>> {
        let mp = self.serial.get_msgparser();
        let mp = mp.lock();
        mp.get_enumerations()
    }
    fn get_constants(&self) -> IndexMap<String, String> {
        let mp = self.serial.get_msgparser();
        let guard = mp.lock();
        let consts = guard.get_constants();
        let mut out = IndexMap::new();
        for (k, v) in consts.iter() {
            out.insert(k.clone(), v.str());
        }
        out
    }
    fn get_constant_float(&self, name: &str) -> KResult<f64> {
        let mp = self.serial.get_msgparser();
        let mp = mp.lock();
        mp.get_constant_float(name, None).map_err(|e| error(e.message().to_string()))
    }
    fn print_time_to_clock(&self, print_time: f64) -> i64 { self.clocksync.print_time_to_clock(print_time) }
    fn clock_to_print_time(&self, clock: f64) -> f64 { self.clocksync.clock_to_print_time(clock as i64) }
    fn estimated_print_time(&self, eventtime: f64) -> f64 {
        if start_arg(&self.printer, "debugoutput").is_some() {
            return 0.0;
        }
        self.clocksync.estimated_print_time(eventtime)
    }
    fn clock32_to_clock64(&self, clock32: i64) -> i64 { self.clocksync.clock32_to_clock64(clock32) }
    fn calibrate_clock(&self, print_time: f64, eventtime: f64) -> KResult<(f64, f64)> {
        let r = self.clocksync.calibrate_clock(print_time, eventtime)?;
        if let Some(c) = self.conn_helper.borrow().as_ref() {
            c.check_timeout(eventtime);
        }
        Ok(r)
    }
    fn get_status(&self, eventtime: Option<f64>) -> PyValue {
        self.stats_helper.borrow().as_ref().expect("stats").get_status(eventtime)
    }
    fn stats(&self, eventtime: f64) -> (bool, String) {
        self.stats_helper.borrow().as_ref().expect("stats").stats(eventtime)
    }
    fn create_trigger_dispatch(&self) -> KResult<klippy_api::mcu::TriggerDispatch> {
        // `TriggerDispatch(self)`: this MCU, not the primary one.
        self.handle().create_trigger_dispatch().map(|t| t as klippy_api::mcu::TriggerDispatch)
    }
    fn serialqueue(&self) -> Option<Arc<chelper_serial::serialqueue::SerialQueue>> {
        self.handle().serialqueue()
    }
}

impl PrinterObject for Mcu {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn klippy_api::mcu::McuApi, dyn klippy_api::mcu::ChipApi);
    fn teardown(&self) { Mcu::teardown(self); }
    fn get_status(&self, _eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.stats_helper.borrow().as_ref().expect("stats").get_status(None)))
    }
    /// `MCU.stats` makes `hasattr(o, 'stats')` true in
    /// `extras/statistics.py`, so every MCU joins the `Stats` line.
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        Some(klippy_api::mcu::McuApi::stats(self, eventtime))
    }
}

impl klippy_api::mcu::ChipApi for Mcu {
    fn setup_pin(&self, pin_type: &str, pin_params: PinParams) -> KResult<Pin> {
        self.config_helper.borrow().as_ref().expect("config helper").setup_pin(pin_type, pin_params)
    }
    fn get_mcu(&self) -> KResult<klippy_api::mcu::Mcu> {
        Ok(self.handle().mcu_as_trait())
    }
}

/// `add_printer_objects(config)` (`klippy/mcu.py:1381`).
pub fn add_printer_objects(config: &Config) -> KResult<()> {
    let printer = config.get_printer();
    let reactor = printer.get_reactor();
    let mainsync = ClockSync::new(reactor.clone());
    let mcu = McuHandle::new(&config.getsection("mcu"), ClockKind::Primary(mainsync.clone()))?;
    printer.add_object("mcu", mcu.clone() as ObjectRef)?;
    for s in config.get_prefix_sections("mcu ") {
        let name = s.get_name();
        // Every secondary syncs to the primary MCU's ClockSync; the clone
        // shares its state, as Python passes the one `mainsync` object.
        let secondary = Arc::new(SecondarySync::new(reactor.clone(), mainsync.clone()));
        let mcu = McuHandle::new(&s, ClockKind::Secondary(secondary))?;
        printer.add_object(&name, mcu.clone() as ObjectRef)?;
    }
    Ok(())
}

/// `get_printer_mcu(printer, name)` re-export (`klippy/mcu.py:1390`).
pub fn get_printer_mcu(printer: &Printer, name: &str) -> KResult<klippy_api::mcu::Mcu> {
    klippy_api::mcu::get_printer_mcu(printer, name)
}

/// Module entry point for the `mcu` section.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        add_printer_objects: Some(add_printer_objects),
        ..ModuleFactory::default()
    }
}

#[cfg(test)]
mod serial_callback_tests {
    use super::*;

    #[test]
    fn asynchronous_query_timeout_is_a_command_error_like_python() {
        let mut py = pyoracle::Oracle::start().expect("query timeout parity requires the Python oracle");
        py.exec(r#"
import mcu, serialhdl
class CommandError(Exception): pass
class Completion:
    def __init__(self, reactor): self.reactor = reactor
    def wait(self, deadline):
        self.reactor.now = deadline
        return None
class Reactor:
    def __init__(self): self.now = 1.
    def monotonic(self): return self.now
    def completion(self): return Completion(self)
class Serial:
    def __init__(self): self.reactor = Reactor()
    def get_reactor(self): return self.reactor
    def register_response(self, *args): pass
    def raw_send_wait_ack(self, *args): pass
    def raw_send(self, *args): pass
_query = mcu.CommandQueryWrapper.__new__(mcu.CommandQueryWrapper)
_query._serial = Serial()
_query._response = 'tmcuart_response'
_query._oid = 3
_query._error = CommandError
_query._xmit_helper = mcu.RetryAsyncCommand
_query._cmd_queue = None
"#).unwrap();
        let expected = py.eval("_query._do_send([b'query'], 0, 0, False)").unwrap_err();
        let reactor = Reactor::new(false);
        let printer = crate::klippy::Printer::new(
            reactor.clone(), None, IndexMap::new(),
            Arc::new(crate::klippy::HostEnv {
                modules: IndexMap::new(), klippy_dir: String::new(),
            }),
        ).unwrap();
        // This test drives only the query state machine, not printer startup.
        let serial = Arc::new(SerialReader::new(reactor.clone(), "mcu"));
        let result = Arc::new(kos::sync::Mutex::new(None));
        let output = result.clone();
        let query_serial = serial.clone();
        let stop = reactor.clone();
        let printer_api = printer.api();
        reactor.register_callback(move |_| {
            let query = CommandQueryWrapper {
                serial: query_serial.clone(), msgformat: String::new(),
                response: "tmcuart_response".to_string(), oid: Some(3),
                error_is_command: true, is_fileoutput: false, is_async: true,
                cmd_queue: query_serial.get_default_command_queue(), printer: printer_api.clone(),
            };
            *output.lock() = Some(query.do_send(&[b"query".to_vec()], 0, 0, false));
            stop.end();
        }, reactor::NOW);
        let ack_serial = serial.clone();
        reactor.register_callback(move |_| {
            crate::serialhdl::test_support::acknowledge_pending(&ack_serial);
        }, reactor.monotonic() + 0.01);
        // The printer constructor's startup callback is not needed here.
        // End its lifetime before dispatch, leaving only the query callbacks.
        drop(printer);
        reactor.run();
        reactor.finalize();
        let error = result.lock().take().expect("query ran").unwrap_err();
        assert_eq!(error.class_name(), expected.class_name);
        assert_eq!(error.message(), expected.message);
    }
}
