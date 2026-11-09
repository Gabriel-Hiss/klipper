//! Port of `klippy/serialhdl.py`.
//!
//! Serial port management for firmware communication. The port keeps
//! the Python shape: [`SerialReader`] owns the message parser, the
//! chelper `serialqueue`, the background thread that pulls messages
//! and dispatches callbacks, and the reactor completions that track
//! pending `#sent_time` notifications. [`SerialRetryCommand`] is the
//! retrying query helper, and the module level functions
//! [`stk500v2_leave`], [`cheetah_reset`] and [`arduino_reset`]
//! implement the hardware reset sequences.
//!
//! The pyserial / python-can / `os.open` layer of the Python file is
//! abstracted by [`SerialPort`] and [`SerialOpener`] so unit tests
//! can supply fakes. The default opener uses
//! `std::fs::OpenOptions` for pipe and UART opens (the virtual-clock
//! contract only covers clocks, polls, threads and locks; file
//! open is not interposed). SocketCAN remains a contract gap, see
//! [`FdOpener`].

use std::sync::Arc;

use chelper_serial::serialqueue::{alloc_commandqueue, CommandQueue, SerialQueue};
use indexmap::IndexMap;
use klippy_api::{log, KlippyError, KResult};
use msgproto::{MessageParser, ParamValue, Params};
use pycompat::text::{percent_format, PctValue};
use pycompat::{PyInt, PyValue};

pub const CANBUS_ID_ADMIN: i32 = 0x3f0;

/// Registration id of a live `CommandQueue` in [`SerialReader::cmd_queues`].
/// Python passes the queue object itself; the port's contract handle
/// (`klippy_api::mcu::CommandQueue`, an opaque `i64`) carries this id.
pub type CommandQueueId = i64;
pub const CMD_SET_NODEID: u8 = 0x01;

/// Callback for one `(name, oid)` handler (`serialhdl.py:244`).
pub type HandlerCallback = Arc<dyn Fn(&Params) + Send + Sync>;

// ---------------------------------------------------------------------------
// errors
// ---------------------------------------------------------------------------

/// `serialhdl.error` (maps to `KlippyError::Mcu` in this port).
pub fn error(message: String) -> KlippyError {
    KlippyError::Mcu(message)
}

fn raise_error(warn_prefix: &str, message: String) -> KlippyError {
    error(format!("{}{}", warn_prefix, message))
}

// ---------------------------------------------------------------------------
// transport abstraction (pyserial / python-can / os.open layer)
// ---------------------------------------------------------------------------

/// Minimal serial device interface used by the reset sequences and by
/// [`SerialReader::start_session`]. Mirrors the small part of
/// `serial.Serial` the Python file uses: `fileno`, `baudrate`,
/// `read`/`write`, `rts`/`dtr`, `open`/`close`, `port`.
pub trait SerialPort: Send + Sync {
    fn fileno(&self) -> i32;
    fn baudrate(&self) -> i32;
    fn set_baudrate(&self, baud: i32);
    fn read(&self, n: usize) -> Vec<u8>;
    fn write(&self, data: &[u8]) -> usize;
    fn rts(&self) -> bool;
    fn set_rts(&self, v: bool);
    fn dtr(&self) -> bool;
    fn set_dtr(&self, v: bool);
    fn open(&self);
    fn close(&self);
}

/// File descriptor backed serial port with in-memory baud / rts / dtr
/// state. `read`/`write` go through [`kos::fd`].
pub struct FdSerialPort {
    fd: kos::sync::Mutex<Option<i32>>,
    baud: kos::sync::Mutex<i32>,
    rts: kos::sync::Mutex<bool>,
    dtr: kos::sync::Mutex<bool>,
    port_name: String,
}

impl FdSerialPort {
    pub fn new(fd: i32, port_name: impl Into<String>) -> Self {
        Self {
            fd: kos::sync::Mutex::new(Some(fd)),
            baud: kos::sync::Mutex::new(9600),
            rts: kos::sync::Mutex::new(false),
            dtr: kos::sync::Mutex::new(false),
            port_name: port_name.into(),
        }
    }

    pub fn with_baud(fd: i32, port_name: impl Into<String>, baud: i32) -> Self {
        let s = Self::new(fd, port_name);
        *s.baud.lock() = baud;
        s
    }

    pub fn port_name(&self) -> &str {
        &self.port_name
    }
}

impl SerialPort for FdSerialPort {
    fn fileno(&self) -> i32 {
        self.fd.lock().unwrap_or(0)
    }
    fn baudrate(&self) -> i32 {
        *self.baud.lock()
    }
    fn set_baudrate(&self, baud: i32) {
        *self.baud.lock() = baud;
    }
    fn read(&self, n: usize) -> Vec<u8> {
        let fd = self.fileno();
        if fd < 0 {
            return Vec::new();
        }
        let mut buf = vec![0u8; n];
        match kos::fd::read(fd, &mut buf) {
            Ok(m) => {
                buf.truncate(m);
                buf
            }
            Err(_) => Vec::new(),
        }
    }
    fn write(&self, data: &[u8]) -> usize {
        let fd = self.fileno();
        if fd < 0 {
            return 0;
        }
        match kos::fd::write(fd, data) {
            Ok(m) => m,
            Err(_) => 0,
        }
    }
    fn rts(&self) -> bool {
        *self.rts.lock()
    }
    fn set_rts(&self, v: bool) {
        *self.rts.lock() = v;
    }
    fn dtr(&self) -> bool {
        *self.dtr.lock()
    }
    fn set_dtr(&self, v: bool) {
        *self.dtr.lock() = v;
    }
    fn open(&self) {}
    fn close(&self) {
        if let Some(fd) = self.fd.lock().take() {
            kos::fd::close(fd);
        }
    }
}

/// Opener for the three transports `SerialReader` supports. The
/// default [`FdOpener`] handles pipe and UART opens; SocketCAN
/// `open_canbus` returns an OS error, which the caller maps through
/// the retry loop. Integration owners that need SocketCAN implement
/// this trait (contract gap: `kos` has no `AF_CAN` socket helper yet).
pub trait SerialOpener: Send + Sync {
    fn open_pipe(&self, filename: &str) -> Result<Arc<dyn SerialPort>, String>;
    fn open_uart(&self, serialport: &str, baud: i32) -> Result<Arc<dyn SerialPort>, String>;
    fn open_canbus(
        &self,
        _canbus_iface: &str,
        _filters: &[(u32, u32)],
        _txid: i32,
    ) -> Result<Arc<dyn SerialPort>, String> {
        Err("socketcan open not supported by default opener".to_string())
    }
}

/// Default opener based on `std::fs::OpenOptions`.
pub struct FdOpener;

impl SerialOpener for FdOpener {
    fn open_pipe(&self, filename: &str) -> Result<Arc<dyn SerialPort>, String> {
        use std::os::unix::io::IntoRawFd;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(filename)
            .map_err(|e| e.to_string())?;
        let dup = file.try_clone().map_err(|e| e.to_string())?;
        let fd = dup.into_raw_fd();
        Ok(Arc::new(FdSerialPort::new(fd, filename)))
    }

    fn open_uart(&self, serialport: &str, baud: i32) -> Result<Arc<dyn SerialPort>, String> {
        use std::os::unix::io::IntoRawFd;
        let file = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(serialport)
            .map_err(|e| e.to_string())?;
        let dup = file.try_clone().map_err(|e| e.to_string())?;
        let fd = dup.into_raw_fd();
        Ok(Arc::new(FdSerialPort::with_baud(fd, serialport, baud)))
    }
}

// ---------------------------------------------------------------------------
// formatting helpers
// ---------------------------------------------------------------------------

/// Format `msg % params` like Python; falls back to `msg` if the
/// format fails (the Python code passes well-formed strings).
fn fmt_msg(msg: &str, params: &[PctValue]) -> String {
    match percent_format(msg, params) {
        Ok(s) => s,
        Err(_) => msg.to_string(),
    }
}

/// `str(params)` for a msgproto params map, i.e. the Python `dict`
/// repr `{'#name': '...', 'oid': 1}`.
fn params_repr(params: &Params) -> String {
    let mut parts = Vec::new();
    for (k, v) in params.iter() {
        parts.push(format!(
            "{}: {}",
            pycompat::text::str_repr(k),
            match v {
                ParamValue::Str(s) => pycompat::text::str_repr(s),
                ParamValue::Bytes(b) => pycompat::text::bytes_repr(b),
                ParamValue::Int(i) => i.to_string(),
            }
        ));
    }
    format!("{{{}}}", parts.join(", "))
}

// ---------------------------------------------------------------------------
// SerialReader
// ---------------------------------------------------------------------------

/// Shared state for the background thread.
struct BgShared {
    reactor: reactor::Reactor,
    warn_prefix: String,
    msgparser: Arc<kos::sync::Mutex<MessageParser>>,
    handlers: Arc<kos::sync::Mutex<IndexMap<(String, Option<i64>), HandlerCallback>>>,
    pending: Arc<kos::sync::Mutex<IndexMap<u64, reactor::Completion<Option<Params>>>>>,
}

pub struct SerialReader {
    reactor: reactor::Reactor,
    warn_prefix: String,
    mcu_name: String,
    sq_name: Vec<u8>,
    msgparser: Arc<kos::sync::Mutex<MessageParser>>,
    serial_dev: kos::sync::Mutex<Option<Arc<dyn SerialPort>>>,
    serialqueue: kos::sync::Mutex<Option<Arc<SerialQueue>>>,
    default_cmd_queue: Arc<CommandQueue>,
    cmd_queues: kos::sync::Mutex<IndexMap<CommandQueueId, Arc<CommandQueue>>>,
    next_cmd_queue_id: kos::sync::Mutex<CommandQueueId>,
    stats_buf: kos::sync::Mutex<[u8; 4096]>,
    bg_thread: kos::sync::Mutex<Option<kos::thread::JoinHandle>>,
    last_notify_id: kos::sync::Mutex<u64>,
    pending_notifications: Arc<kos::sync::Mutex<IndexMap<u64, reactor::Completion<Option<Params>>>>>,
    handlers: Arc<kos::sync::Mutex<IndexMap<(String, Option<i64>), HandlerCallback>>>,
    shared: Arc<BgShared>,
    opener: Arc<dyn SerialOpener>,
}

impl SerialReader {
    pub fn new(reactor: reactor::Reactor, mcu_name: &str) -> Self {
        Self::new_with_opener(reactor, mcu_name, Arc::new(FdOpener))
    }

    pub fn new_with_opener(
        reactor: reactor::Reactor,
        mcu_name: &str,
        opener: Arc<dyn SerialOpener>,
    ) -> Self {
        let warn_prefix = if mcu_name.is_empty() {
            String::new()
        } else {
            fmt_msg("mcu '%s': ", &[PctValue::Str(mcu_name.to_string())])
        };
        let sq_name_full = fmt_msg("serialq %s", &[PctValue::Str(mcu_name.to_string())]);
        let sq_name = sq_name_full.chars().take(15).collect::<String>().into_bytes();
        let msgparser = Arc::new(kos::sync::Mutex::new(MessageParser::new(&warn_prefix)));
        let handlers = Arc::new(kos::sync::Mutex::new(IndexMap::new()));
        let pending: Arc<kos::sync::Mutex<IndexMap<u64, reactor::Completion<Option<Params>>>>> =
            Arc::new(kos::sync::Mutex::new(IndexMap::new()));
        let shared = Arc::new(BgShared {
            reactor: reactor.clone(),
            warn_prefix: warn_prefix.clone(),
            msgparser: msgparser.clone(),
            handlers: handlers.clone(),
            pending: pending.clone(),
        });
        let r = SerialReader {
            reactor,
            warn_prefix,
            mcu_name: mcu_name.to_string(),
            sq_name,
            msgparser,
            serial_dev: kos::sync::Mutex::new(None),
            serialqueue: kos::sync::Mutex::new(None),
            default_cmd_queue: alloc_commandqueue(),
            cmd_queues: kos::sync::Mutex::new(IndexMap::new()),
            next_cmd_queue_id: kos::sync::Mutex::new(0),
            stats_buf: kos::sync::Mutex::new([0u8; 4096]),
            bg_thread: kos::sync::Mutex::new(None),
            last_notify_id: kos::sync::Mutex::new(0),
            pending_notifications: pending,
            handlers,
            shared,
            opener,
        };
        // Seed the two default handlers exactly like __init__.
        let wp = r.warn_prefix.clone();
        r.register_response(
            Some(Arc::new(move |params: &Params| {
                let msgid = params
                    .get("#msgid")
                    .map(|v| match v {
                        ParamValue::Int(i) => *i,
                        _ => 0,
                    })
                    .unwrap_or(0);
                let msg = params
                    .get("#msg")
                    .map(|v| match v {
                        ParamValue::Bytes(b) => b.clone(),
                        ParamValue::Str(s) => s.clone().into_bytes(),
                        ParamValue::Int(i) => i.to_string().into_bytes(),
                    })
                    .unwrap_or_default();
                let line = fmt_msg(
                    "%sUnknown message %d (len %d) while identifying",
                    &[
                        PctValue::Str(wp.clone()),
                        PctValue::Int(PyInt::from_i64(msgid)),
                        PctValue::Int(PyInt::from_i64(msg.len() as i64)),
                    ],
                );
                log::debug(&line);
            })),
            "#unknown",
            None,
        );
        let wp2 = r.warn_prefix.clone();
        r.register_response(
            Some(Arc::new(move |params: &Params| {
                let name = params
                    .get("#name")
                    .map(|v| match v {
                        ParamValue::Str(s) => s.clone(),
                        _ => String::new(),
                    })
                    .unwrap_or_default();
                let msg = params
                    .get("#msg")
                    .map(|v| match v {
                        ParamValue::Bytes(b) => pycompat::text::bytes_repr(b),
                        ParamValue::Str(s) => s.clone(),
                        ParamValue::Int(i) => i.to_string(),
                    })
                    .unwrap_or_default();
                let line = fmt_msg(
                    "%s%s: %s",
                    &[
                        PctValue::Str(wp2.clone()),
                        PctValue::Str(name),
                        PctValue::Str(msg),
                    ],
                );
                log::info(&line);
            })),
            "#output",
            None,
        );
        r
    }

    pub fn get_reactor(&self) -> reactor::Reactor {
        self.reactor.clone()
    }

    pub fn get_msgparser(&self) -> Arc<kos::sync::Mutex<MessageParser>> {
        self.msgparser.clone()
    }

    pub fn get_serialqueue(&self) -> Option<Arc<SerialQueue>> {
        let sq = self.serialqueue.lock();
        sq.clone()
    }

    pub fn get_default_command_queue(&self) -> Arc<CommandQueue> {
        self.default_cmd_queue.clone()
    }

    pub fn warn_prefix(&self) -> &str {
        &self.warn_prefix
    }

    pub fn mcu_name(&self) -> &str {
        &self.mcu_name
    }

    pub fn sq_name(&self) -> &[u8] {
        &self.sq_name
    }

    // -- background thread --------------------------------------------------

    fn bg_thread_entry(&self, sq: Arc<SerialQueue>) {
        {
            let shared = self.shared.clone();
            let reactor = self.reactor.clone();
            let sq2 = sq.clone();
            let name = fmt_msg("serialhdl %s", &[PctValue::Str(self.mcu_name.clone())]);
            let handle = kos::thread::spawn_named(&name.chars().take(15).collect::<String>(), move || {
                let shared = shared;
                let sq = sq2;
                loop {
                    let response = sq.pull();
                    let response = match response {
                        Some(r) => r,
                        None => break,
                    };
                    let count = response.len;
                    if response.notify_id != 0 {
                        let mut params = Params::new();
                        params.insert(
                            "#sent_time".to_string(),
                            ParamValue::Str(response.sent_time.to_string()),
                        );
                        params.insert(
                            "#receive_time".to_string(),
                            ParamValue::Str(response.receive_time.to_string()),
                        );
                        let completion = {
                            let mut pending = shared.pending.lock();
                            pending.shift_remove(&response.notify_id)
                        };
                        if let Some(completion) = completion {
                            reactor.async_complete(&completion, Some(params));
                        }
                        continue;
                    }
                    let slice = if count >= 0 && (count as usize) <= response.msg.len() {
                        &response.msg[..count as usize]
                    } else {
                        &response.msg[..]
                    };
                    let parsed = shared.msgparser.lock().parse(slice);
                    let mut params = match parsed {
                        Ok(p) => p,
                        Err(e) => {
                            let line = fmt_msg(
                                "%sException in serial callback",
                                &[PctValue::Str(shared.warn_prefix.clone())],
                            );
                            log::exception(&line, &e.to_string());
                            continue;
                        }
                    };
                    params.insert(
                        "#sent_time".to_string(),
                        ParamValue::Str(response.sent_time.to_string()),
                    );
                    params.insert(
                        "#receive_time".to_string(),
                        ParamValue::Str(response.receive_time.to_string()),
                    );
                    let name = params
                        .get("#name")
                        .map(|v| match v {
                            ParamValue::Str(s) => s.clone(),
                            _ => String::new(),
                        })
                        .unwrap_or_default();
                    let oid = params.get("oid").and_then(|v| match v {
                        ParamValue::Int(i) => Some(*i),
                        _ => None,
                    });
                    let hdl = {
                        let handlers = shared.handlers.lock();
                        handlers.get(&(name.clone(), oid)).cloned()
                    };
                    let params_d = params;
                    let hdl = match hdl {
                        Some(h) => h,
                        None => {
                            // handle_default
                            let line = fmt_msg(
                                "%sgot %s",
                                &[
                                    PctValue::Str(shared.warn_prefix.clone()),
                                    PctValue::Str(params_repr(&params_d)),
                                ],
                            );
                            log::warning(&line);
                            continue;
                        }
                    };
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        hdl(&params_d);
                    }));
                    if result.is_err() {
                        let line = fmt_msg(
                            "%sException in serial callback",
                            &[PctValue::Str(shared.warn_prefix.clone())],
                        );
                        log::exception(&line, "callback panic");
                    }
                }
            });
            *self.bg_thread.lock() = Some(handle);
        }
    }

    // -- identify / session -------------------------------------------------

    fn get_identify_data(&self, _eventtime: f64) -> Option<Vec<u8>> {
        let mut identify_data: Vec<u8> = Vec::new();
        loop {
            let msg = fmt_msg(
                "identify offset=%d count=%d",
                &[
                    PctValue::Int(PyInt::from_i64(identify_data.len() as i64)),
                    PctValue::Int(PyInt::from_i64(40)),
                ],
            );
            let params = match self.send_with_response(&msg, "identify_response") {
                Ok(p) => p,
                Err(_) => {
                    let line = fmt_msg(
                        "%sWait for identify_response",
                        &[PctValue::Str(self.warn_prefix.clone())],
                    );
                    log::exception(&line, "identify timeout");
                    return None;
                }
            };
            let offset = params
                .get("offset")
                .map(|v| match v {
                    ParamValue::Int(i) => *i,
                    _ => 0,
                })
                .unwrap_or(0);
            if offset as usize == identify_data.len() {
                let msgdata = params
                    .get("data")
                    .map(|v| match v {
                        ParamValue::Bytes(b) => b.clone(),
                        ParamValue::Str(s) => s.clone().into_bytes(),
                        _ => Vec::new(),
                    })
                    .unwrap_or_default();
                if msgdata.is_empty() {
                    return Some(identify_data);
                }
                identify_data.extend_from_slice(&msgdata);
            }
        }
    }

    fn start_session(&self, serial_dev: Arc<dyn SerialPort>, serial_fd_type: u8, client_id: i32) -> bool {
        *self.serial_dev.lock() = Some(serial_dev.clone());
        let sq = SerialQueue::alloc(serial_dev.fileno(), serial_fd_type, client_id, &self.sq_name);
        let sq = match sq {
            Some(s) => Arc::new(s),
            None => return false,
        };
        *self.serialqueue.lock() = Some(sq.clone());
        self.bg_thread_entry(sq.clone());

        // Obtain dict: drive identify inline (Python registers
        // `_get_identify_data` as a reactor callback and waits +5s;
        // the callback body is exactly `get_identify_data`).
        let identify_data = self.get_identify_data(self.reactor.monotonic());
        let identify_data = match identify_data {
            Some(d) => d,
            None => {
                let line = fmt_msg("%sTimeout on connect", &[PctValue::Str(self.warn_prefix.clone())]);
                log::info(&line);
                self.disconnect();
                return false;
            }
        };
        let mut newparser = MessageParser::new(&self.warn_prefix);
        if newparser.process_identify(&identify_data, true).is_err() {
            // Python would raise; treat as failure path of the session
            return false;
        }
        {
            let mut mp = self.msgparser.lock();
            *mp = newparser;
        }
        let wp = self.warn_prefix.clone();
        self.register_response(
            Some(Arc::new(move |params: &Params| {
                let msgid = params
                    .get("#msgid")
                    .map(|v| match v {
                        ParamValue::Int(i) => *i,
                        _ => 0,
                    })
                    .unwrap_or(0);
                let msg = params
                    .get("#msg")
                    .map(|v| match v {
                        ParamValue::Bytes(b) => pycompat::text::bytes_repr(b),
                        ParamValue::Str(s) => s.clone(),
                        ParamValue::Int(i) => i.to_string(),
                    })
                    .unwrap_or_default();
                let line = fmt_msg(
                    "%sUnknown message type %d: %s",
                    &[
                        PctValue::Str(wp.clone()),
                        PctValue::Int(PyInt::from_i64(msgid)),
                        PctValue::Str(msg),
                    ],
                );
                log::warning(&line);
            })),
            "#unknown",
            None,
        );
        // baud adjust
        let (wire_freq, receive_window) = {
            let mp = self.msgparser.lock();
            let wf = if serial_fd_type == b'c' {
                mp.get_constant_float("CANBUS_FREQUENCY", Some(PyValue::None))
            } else {
                mp.get_constant_float("SERIAL_BAUD", Some(PyValue::None))
            };
            let rw = mp.get_constant_int("RECEIVE_WINDOW", Some(PyValue::None));
            (wf, rw)
        };
        if let Ok(wf) = wire_freq {
            if let Some(sq) = self.serialqueue.lock().clone() {
                sq.set_wire_frequency(wf);
            }
        }
        if let Ok(rw) = receive_window {
            if let Some(sq) = self.serialqueue.lock().clone() {
                let v = if rw > i32::MAX as i64 {
                    i32::MAX
                } else if rw < i32::MIN as i64 {
                    i32::MIN
                } else {
                    rw as i32
                };
                sq.set_receive_window(v);
            }
        }
        true
    }

    // -- connect* ------------------------------------------------------------

    pub fn connect_canbus(
        &self,
        canbus_uuid: &str,
        canbus_nodeid: i32,
        canbus_iface: &str,
    ) -> KResult<()> {
        let txid = canbus_nodeid * 2 + 256;
        let filters = vec![(txid as u32 + 1, 0x7ffu32)];
        // Prep for SET_NODEID
        let uuid_val: i64 = match i64::from_str_radix(canbus_uuid, 16) {
            Ok(v) => v,
            Err(_) => -1,
        };
        if uuid_val < 0 || uuid_val > 0xffffffffffff {
            return Err(raise_error(&self.warn_prefix, "Invalid CAN uuid".to_string()));
        }
        let uuid_bytes: Vec<u8> = (0..6)
            .map(|i| ((uuid_val as u64 >> (40 - i * 8)) & 0xff) as u8)
            .collect();
        let mut set_id_cmd: Vec<u8> = vec![CMD_SET_NODEID];
        set_id_cmd.extend_from_slice(&uuid_bytes);
        set_id_cmd.push(canbus_nodeid as u8);
        let _set_id_msg = set_id_cmd;

        let line = fmt_msg("%sStarting CAN connect", &[PctValue::Str(self.warn_prefix.clone())]);
        log::info(&line);
        let start_time = self.reactor.monotonic();
        loop {
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(raise_error(&self.warn_prefix, "Unable to connect".to_string()));
            }
            let bus_result = self.opener.open_canbus(canbus_iface, &filters, txid);
            let bus = match bus_result {
                Ok(b) => b,
                Err(e) => {
                    let line = fmt_msg(
                        "%sUnable to open CAN port: %s",
                        &[
                            PctValue::Str(self.warn_prefix.clone()),
                            PctValue::Str(e),
                        ],
                    );
                    log::warning(&line);
                    self.reactor.pause(self.reactor.monotonic() + 5.0);
                    continue;
                }
            };
            let ret = self.start_session(bus, b'c', txid);
            if !ret {
                continue;
            }
            // Verify mapping
            let verify = (|| -> KResult<Vec<u8>> {
                let params = self.send_with_response("get_canbus_id", "canbus_id")?;
                let got = params
                    .get("canbus_uuid")
                    .map(|v| match v {
                        ParamValue::Bytes(b) => b.clone(),
                        ParamValue::Str(s) => s.clone().into_bytes(),
                        _ => Vec::new(),
                    })
                    .unwrap_or_default();
                if got == uuid_bytes {
                    Ok(got)
                } else {
                    Err(error("uuid mismatch".to_string()))
                }
            })();
            if verify.is_ok() {
                break;
            } else {
                let line = fmt_msg(
                    "%sError in canbus_uuid check",
                    &[PctValue::Str(self.warn_prefix.clone())],
                );
                log::exception(&line, "canbus_uuid mismatch");
            }
            let line = fmt_msg(
                "%sFailed to match canbus_uuid - retrying..",
                &[PctValue::Str(self.warn_prefix.clone())],
            );
            log::info(&line);
            self.disconnect();
        }
        Ok(())
    }

    pub fn connect_pipe(&self, filename: &str) -> KResult<()> {
        let line = fmt_msg("%sStarting connect", &[PctValue::Str(self.warn_prefix.clone())]);
        log::info(&line);
        let start_time = self.reactor.monotonic();
        loop {
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(raise_error(&self.warn_prefix, "Unable to connect".to_string()));
            }
            let dev = match self.opener.open_pipe(filename) {
                Ok(d) => d,
                Err(e) => {
                    let line = fmt_msg(
                        "%sUnable to open port: %s",
                        &[
                            PctValue::Str(self.warn_prefix.clone()),
                            PctValue::Str(e),
                        ],
                    );
                    log::warning(&line);
                    self.reactor.pause(self.reactor.monotonic() + 5.0);
                    continue;
                }
            };
            let ret = self.start_session(dev, b'u', 0);
            if ret {
                break;
            }
        }
        Ok(())
    }

    pub fn connect_uart(&self, serialport: &str, baud: i32, rts: bool) -> KResult<()> {
        let line = fmt_msg("%sStarting serial connect", &[PctValue::Str(self.warn_prefix.clone())]);
        log::info(&line);
        let start_time = self.reactor.monotonic();
        loop {
            if self.reactor.monotonic() > start_time + 90.0 {
                return Err(raise_error(&self.warn_prefix, "Unable to connect".to_string()));
            }
            let dev = match self.opener.open_uart(serialport, baud) {
                Ok(d) => d,
                Err(e) => {
                    let line = fmt_msg(
                        "%sUnable to open serial port: %s",
                        &[
                            PctValue::Str(self.warn_prefix.clone()),
                            PctValue::Str(e),
                        ],
                    );
                    log::warning(&line);
                    self.reactor.pause(self.reactor.monotonic() + 5.0);
                    continue;
                }
            };
            dev.set_rts(rts);
            dev.set_baudrate(baud);
            dev.open();
            stk500v2_leave(&*dev, &self.reactor);
            let ret = self.start_session(dev, b'u', 0);
            // NOTE: `ret` unused beyond loop control; `stk500v2_leave` runs before
            // `start_session` in Python; the above matches the order.
            if ret {
                break;
            }
        }
        Ok(())
    }

    pub fn connect_file(&self, debugoutput: Arc<dyn SerialPort>, dictionary: &[u8], _pace: bool) -> KResult<()> {
        *self.serial_dev.lock() = Some(debugoutput.clone());
        {
            let mut mp = self.msgparser.lock();
            mp.process_identify(dictionary, false)
                .map_err(|e| error(e.message().to_string()))?;
        }
        let sq = SerialQueue::alloc(debugoutput.fileno(), b'f', 0, &self.sq_name);
        if let Some(sq) = sq {
            *self.serialqueue.lock() = Some(Arc::new(sq));
        }
        Ok(())
    }

    pub fn set_clock_est(&self, freq: f64, conv_time: f64, conv_clock: u64) {
        if let Some(sq) = self.serialqueue.lock().clone() {
            sq.set_clock_est(freq, conv_time, conv_clock);
        }
    }

    pub fn disconnect(&self) {
        if let Some(sq) = self.serialqueue.lock().take() {
            // SerialQueue::exit joins its own background thread.
            sq.exit();
            // `free` semantics of ffi gc drop (no background thread join here)
            if let Some(h) = self.bg_thread.lock().take() {
                h.join();
            }
        }
        if let Some(dev) = self.serial_dev.lock().take() {
            dev.close();
        }
        {
            let mut pending = self.pending_notifications.lock();
            let values: Vec<_> = pending.values().cloned().collect();
            pending.clear();
            for c in values {
                c.complete(None);
            }
        }
    }

    pub fn stats(&self, _eventtime: f64) -> String {
        if self.serialqueue.lock().is_none() {
            return String::new();
        }
        let sq = self.serialqueue.lock().clone().unwrap();
        let mut buf = self.stats_buf.lock();
        sq.get_stats(&mut buf[..]);
        let end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        String::from_utf8_lossy(&buf[..end]).to_string()
    }

    pub fn dump_debug(&self) -> String {
        let mut out: Vec<String> = Vec::new();
        out.push(fmt_msg(
            "Dumping serial stats: %s",
            &[PctValue::Str(self.stats(self.reactor.monotonic()))],
        ));
        let (scount_msgs, rcount_msgs) = match self.serialqueue.lock().clone() {
            Some(sq) => {
                let s = sq.extract_old(true, 1024);
                let r = sq.extract_old(false, 1024);
                (s, r)
            }
            None => (Vec::new(), Vec::new()),
        };
        out.push(fmt_msg(
            "Dumping send queue %d messages",
            &[PctValue::Int(PyInt::from_i64(scount_msgs.len() as i64))],
        ));
        for (i, msg) in scount_msgs.iter().enumerate() {
            let cmds = {
                let mp = self.msgparser.lock();
                let slice = if msg.len >= 0 && (msg.len as usize) <= msg.msg.len() {
                    &msg.msg[..msg.len as usize]
                } else {
                    &msg.msg[..]
                };
                mp.dump(slice).unwrap_or_default()
            };
            out.push(fmt_msg(
                "Sent %d %f %f %d: %s",
                &[
                    PctValue::Int(PyInt::from_i64(i as i64)),
                    PctValue::Float(msg.receive_time),
                    PctValue::Float(msg.sent_time),
                    PctValue::Int(PyInt::from_i64(msg.len as i64)),
                    PctValue::Str(cmds.join(", ")),
                ],
            ));
        }
        out.push(fmt_msg(
            "Dumping receive queue %d messages",
            &[PctValue::Int(PyInt::from_i64(rcount_msgs.len() as i64))],
        ));
        for (i, msg) in rcount_msgs.iter().enumerate() {
            let cmds = {
                let mp = self.msgparser.lock();
                let slice = if msg.len >= 0 && (msg.len as usize) <= msg.msg.len() {
                    &msg.msg[..msg.len as usize]
                } else {
                    &msg.msg[..]
                };
                mp.dump(slice).unwrap_or_default()
            };
            out.push(fmt_msg(
                "Receive: %d %f %f %d: %s",
                &[
                    PctValue::Int(PyInt::from_i64(i as i64)),
                    PctValue::Float(msg.receive_time),
                    PctValue::Float(msg.sent_time),
                    PctValue::Int(PyInt::from_i64(msg.len as i64)),
                    PctValue::Str(cmds.join(", ")),
                ],
            ));
        }
        out.join("\n")
    }

    // -- response callbacks -------------------------------------------------

    pub fn register_response(&self, callback: Option<HandlerCallback>, name: &str, oid: Option<i64>) {
        let mut handlers = self.handlers.lock();
        if callback.is_none() {
            handlers.shift_remove(&(name.to_string(), oid));
        } else {
            handlers.insert((name.to_string(), oid), callback.unwrap());
        }
    }

    pub fn handle_default(&self, params: &Params) {
        let line = fmt_msg(
            "%sgot %s",
            &[
                PctValue::Str(self.warn_prefix.clone()),
                PctValue::Str(params_repr(params)),
            ],
        );
        log::warning(&line);
    }

    // -- sending -------------------------------------------------------------

    pub fn raw_send(&self, cmd: &[u8], minclock: u64, reqclock: u64, cmd_queue: Arc<CommandQueue>) {
        if let Some(sq) = self.serialqueue.lock().clone() {
            sq.send(&cmd_queue, cmd, minclock, reqclock, 0);
        }
    }

    pub fn raw_send_wait_ack(
        &self,
        cmd: &[u8],
        minclock: u64,
        reqclock: u64,
        cmd_queue: Arc<CommandQueue>,
    ) -> KResult<Params> {
        *self.last_notify_id.lock() += 1;
        let nid = *self.last_notify_id.lock();
        let completion = self.reactor.completion();
        {
            let mut pending = self.pending_notifications.lock();
            pending.insert(nid, completion.clone());
        }
        if let Some(sq) = self.serialqueue.lock().clone() {
            sq.send(&cmd_queue, cmd, minclock, reqclock, nid);
        }
        // `completion.wait()` parks forever (NEVER); Python's
        // `raw_send_wait_ack` has no timeout here.
        let params = completion.wait(reactor::NEVER);
        match params {
            Some(Some(v)) => Ok(v),
            _ => Err(raise_error(&self.warn_prefix, "Serial connection closed".to_string())),
        }
    }

    pub fn send(&self, msg: &str, minclock: u64, reqclock: u64) -> KResult<()> {
        let cmd = {
            let mp = self.msgparser.lock();
            mp.create_command(msg).map_err(|e| error(e.message().to_string()))?
        };
        self.raw_send(&cmd, minclock, reqclock, self.default_cmd_queue.clone());
        Ok(())
    }

    pub fn send_with_response(&self, msg: &str, response: &str) -> KResult<Params> {
        let cmd = {
            let mp = self.msgparser.lock();
            mp.create_command(msg).map_err(|e| error(e.message().to_string()))?
        };
        let src = SerialRetryCommand::new(self, response, None);
        src.get_response(&[cmd], self.default_cmd_queue.clone(), 0, 0, true)
    }

    /// `serialqueue_alloc_commandqueue` (`serialhdl.py:272-274`): a real
    /// queue, freed by the C `gc` pair. The `Mcu` layer keeps the `Arc`
    /// alive (see `SerialQueueTable`) so messages scheduled on it survive
    /// until the serial queue transmits them.
    pub fn alloc_command_queue(&self) -> Arc<CommandQueue> {
        alloc_commandqueue()
    }

    /// Look up a live command queue by its registration id.
    pub fn command_queue(&self, id: CommandQueueId) -> Option<Arc<CommandQueue>> {
        self.cmd_queues.lock().get(&id).cloned()
    }

    /// Register a live command queue and return its id. The table holds an
    /// `Arc`, so the C `free` only runs when the id is forgotten *and* no
    /// wrapper or serial queue references it anymore.
    pub fn register_command_queue(&self, cq: Arc<CommandQueue>) -> CommandQueueId {
        let id = self.next_cmd_queue_id.lock().wrapping_add(1);
        *self.next_cmd_queue_id.lock() = id;
        self.cmd_queues.lock().insert(id, cq);
        id
    }
}

// ---------------------------------------------------------------------------
// SerialRetryCommand
// ---------------------------------------------------------------------------

/// `serialhdl.py:313` — query command with retries.
pub struct SerialRetryCommand<'a> {
    pub serial: &'a SerialReader,
    pub name: String,
    pub oid: Option<i64>,
    pub last_params: Arc<kos::sync::Mutex<Option<Params>>>,
}

impl<'a> SerialRetryCommand<'a> {
    pub fn new(serial: &'a SerialReader, name: &str, oid: Option<i64>) -> Self {
        let last_params: Arc<kos::sync::Mutex<Option<Params>>> =
            Arc::new(kos::sync::Mutex::new(None));
        let cell = last_params.clone();
        let cb: HandlerCallback = Arc::new(move |params: &Params| {
            *cell.lock() = Some(params.clone());
        });
        serial.register_response(Some(cb), name, oid);
        SerialRetryCommand {
            serial,
            name: name.to_string(),
            oid,
            last_params,
        }
    }

    pub fn handle_callback(&self, params: &Params) {
        *self.last_params.lock() = Some(params.clone());
    }

    pub fn get_response(
        &self,
        cmds: &[Vec<u8>],
        cmd_queue: Arc<CommandQueue>,
        minclock: u64,
        reqclock: u64,
        retry: bool,
    ) -> KResult<Params> {
        let mut retries: i32 = 5;
        let mut retry_delay: f64 = 0.010;
        if !retry {
            retries = 0;
        }
        let serial = self.serial;
        loop {
            for cmd in &cmds[..cmds.len().saturating_sub(1)] {
                serial.raw_send(cmd, minclock, reqclock, cmd_queue.clone());
            }
            serial.raw_send_wait_ack(&cmds[cmds.len() - 1], minclock, reqclock, cmd_queue.clone())?;
            if let Some(p) = self.last_params.lock().take() {
                serial.register_response(None, &self.name, self.oid);
                return Ok(p);
            }
            if retries <= 0 {
                serial.register_response(None, &self.name, self.oid);
                return Err(error(fmt_msg(
                    "Unable to obtain '%s' response",
                    &[PctValue::Str(self.name.clone())],
                )));
            }
            let reactor = serial.get_reactor();
            reactor.pause(reactor.monotonic() + retry_delay);
            retries -= 1;
            retry_delay *= 2.0;
        }
    }
}

// ---------------------------------------------------------------------------
// module-level reset helpers
// ---------------------------------------------------------------------------

/// `serialhdl.py:346` — AVR stk500v2 leave programmer sequence.
pub fn stk500v2_leave(ser: &dyn SerialPort, reactor: &reactor::Reactor) {
    log::debug("Starting stk500v2 leave programmer sequence");
    kos::termios::clear_hupcl(ser.fileno());
    let origbaud = ser.baudrate();
    ser.set_baudrate(2400);
    ser.read(1);
    ser.set_baudrate(115200);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.read(4096);
    ser.write(b"\x1b\x01\x00\x01\x0e\x11\x04");
    reactor.pause(reactor.monotonic() + 0.050);
    let res = ser.read(4096);
    let line = fmt_msg("Got %s from stk500v2", &[PctValue::Str(pycompat::text::bytes_repr(&res))]);
    log::debug(&line);
    ser.set_baudrate(origbaud);
}

/// `serialhdl.py:363` — Fysetc Cheetah v1.2 reset.
pub fn cheetah_reset(serialport: &str, reactor: &reactor::Reactor, opener: &dyn SerialOpener) {
    // Open the serial port with RTS asserted (2400 baud, exclusive).
    let ser = opener.open_uart(serialport, 2400).unwrap_or_else(|_| {
        let (rfd, _wfd) = kos::fd::pipe().unwrap_or((0, 0));
        Arc::new(FdSerialPort::new(rfd, serialport)) as Arc<dyn SerialPort>
    });
    ser.set_rts(true);
    ser.open();
    ser.read(1);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(false);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_rts(false);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(false);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.close();
}

/// `serialhdl.py:390` — Arduino style reset.
pub fn arduino_reset(serialport: &str, reactor: &reactor::Reactor, opener: &dyn SerialOpener) {
    let ser = opener.open_uart(serialport, 2400).unwrap_or_else(|_| {
        let (rfd, _wfd) = kos::fd::pipe().unwrap_or((0, 0));
        Arc::new(FdSerialPort::new(rfd, serialport)) as Arc<dyn SerialPort>
    });
    ser.read(1);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(true);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.set_dtr(false);
    reactor.pause(reactor.monotonic() + 0.100);
    ser.close();
}

// ---------------------------------------------------------------------------
// module factory (serialhdl has no load_config in Python; keep empty)
// ---------------------------------------------------------------------------

pub fn module_factory() -> klippy_api::printer::ModuleFactory {
    klippy_api::printer::ModuleFactory::default()
}
