//! Port of the `palette2` module (`klippy/extras/palette2.py`).
//!
//! Palette 2 MMU support: a serial link to the Palette 2 (firmware 9.0.9 and newer) driven by
//! three reactor timers (read, write, heartbeat) plus an optional smart-load timer, and the
//! `O*` Omega G-code commands that the slicer output carries.
//!
//! # Mapping notes
//!
//! - `import serial` (pyserial) becomes [`SerialPort`] / [`SerialOpener`]. [`open_posix_serial`]
//!   is the production opener (`serial.Serial(port, baud, timeout=0, write_timeout=0)` on Linux)
//!   and [`Palette2::new`] takes the opener so tests can stand in for the device. Limits of the
//!   real opener: only baud rates `cfsetspeed` accepts are supported (pyserial falls back to a
//!   custom-speed ioctl for the others), and the `TIOCMBIS` DTR/RTS ioctls are skipped because
//!   the kernel asserts both lines when a tty is opened.
//! - `queue.Queue` is a `VecDeque` in [`GCell`] state: every producer and consumer is a reactor
//!   greenlet, exactly as in Python.
//! - An uncaught Python exception inside a timer callback ends klippy through the reactor.
//!   The timer closures therefore panic with `Class: message` when the callback fails, which
//!   `klippy_core` turns into the "Unhandled exception during run" path. The `run_*` methods
//!   return the `KResult` so callers can see the error without the panic.
//! - `self.virtual_sdcard` and `self.pause_resume` are duck-typed objects in Python. They are
//!   reached through [`crate::virtual_sdcard::VirtualSDApi`] and
//!   [`crate::pause_resume::PauseResumeApi`], the interfaces their owners define (assumed
//!   signatures are listed in the porting report).

use std::collections::VecDeque;
use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::config::{Config, GetFloatOpts, GetIntOpts, GetStrOpts};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCmd, GCode, GCodeApi, GCodeCommandHandler, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};
use klippy_api::toolhead::ToolHeadApi;
use klippy_api::{lookup_interface, require_interface, KResult, KlippyError, ObjectRef, PrinterObject};
use pycompat::text::{float_from_string, float_str, int_from_string, percent_format, PctValue, PyError};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};

use crate::heaters::HeaterApi;
use crate::pause_resume::PauseResumeApi;
use crate::virtual_sdcard::VirtualSDApi;

/// `COMMAND_HEARTBEAT` (`palette2.py:18`).
const COMMAND_HEARTBEAT: &str = "O99";
/// `COMMAND_CUT` (`palette2.py:19`).
const COMMAND_CUT: &str = "O10 D5";
/// `COMMAND_CLEAR` (`palette2.py:20-26`).
const COMMAND_CLEAR: [&str; 6] = [
    "O10 D5",
    "O10 D0 D0 D0 DFFE1",
    "O10 D1 D0 D0 DFFE1",
    "O10 D2 D0 D0 DFFE1",
    "O10 D3 D0 D0 DFFE1",
    "O10 D4 D0 D0 D0069",
];
/// `COMMAND_FILENAME` (`palette2.py:27`).
const COMMAND_FILENAME: &str = "O51";
/// `COMMAND_FILENAMES_DONE` (`palette2.py:28`).
const COMMAND_FILENAMES_DONE: &str = "O52";
/// `COMMAND_FIRMWARE` (`palette2.py:29`).
const COMMAND_FIRMWARE: &str = "O50";
/// `COMMAND_PING` (`palette2.py:30`).
const COMMAND_PING: &str = "O31";
/// `COMMAND_SMART_LOAD_STOP` (`palette2.py:31`).
const COMMAND_SMART_LOAD_STOP: &str = "O102 D1";

/// `HEARTBEAT_SEND` (`palette2.py:33`).
const HEARTBEAT_SEND: f64 = 5.;
/// `HEARTBEAT_TIMEOUT` (`palette2.py:34`): `(HEARTBEAT_SEND * 2.) + 1.`.
const HEARTBEAT_TIMEOUT: f64 = (HEARTBEAT_SEND * 2.) + 1.;
/// `SETUP_TIMEOUT` (`palette2.py:35`), an int in Python, always used in float arithmetic.
const SETUP_TIMEOUT: f64 = 10.;

/// `SERIAL_TIMER` (`palette2.py:37`).
const SERIAL_TIMER: f64 = 0.1;
/// `AUTOLOAD_TIMER` (`palette2.py:38`).
const AUTOLOAD_TIMER: f64 = 5.;

/// `INFO_NOT_CONNECTED` (`palette2.py:40`).
const INFO_NOT_CONNECTED: &str = "Palette 2 is not connected, connect first";

/// `cmd_Connect_Help` (`palette2.py:140`).
pub const CMD_CONNECT_HELP: &str = "Connect to the Palette 2";
/// `cmd_Disconnect_Help` (`palette2.py:175`).
pub const CMD_DISCONNECT_HELP: &str = "Disconnect from the Palette 2";
/// `cmd_Clear_Help` (`palette2.py:191`).
pub const CMD_CLEAR_HELP: &str = "Clear the input and output of the Palette 2";
/// `cmd_Cut_Help` (`palette2.py:199`).
pub const CMD_CUT_HELP: &str = "Cut the outgoing filament";
/// `cmd_Smart_Load_Help` (`palette2.py:206`).
pub const CMD_SMART_LOAD_HELP: &str = "Automatically load filament through the extruder";
/// `cmd_O1_help` (`palette2.py:234`).
pub const CMD_O1_HELP: &str = "Initialize the print, and check connection with the Palette 2";
/// `cmd_O9_help` (`palette2.py:250`).
pub const CMD_O9_HELP: &str = "Reset print information";

// ---------------------------------------------------------------------------
// Serial port (pyserial)
// ---------------------------------------------------------------------------

/// `serial.SerialException`: the only exception class `palette2` catches from pyserial.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SerialException(pub String);

/// Failure of [`SerialOpener`]: pyserial raises either `SerialException` (caught by
/// `cmd_Connect`) or another class such as `ValueError` for a bad baud rate (uncaught).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SerialOpenError {
    /// `serial.SerialException`.
    Serial(SerialException),
    /// Any other Python exception.
    Other(KlippyError),
}

/// The `serial.Serial` instance (`palette2.py:152`), opened with `timeout=0, write_timeout=0`.
pub trait SerialPort: Send {
    /// `serial.read()`: up to one byte without waiting, empty when nothing is pending.
    fn read(&mut self) -> Result<Vec<u8>, SerialException>;
    /// `serial.write(data)`: bytes written without waiting.
    fn write(&mut self, data: &[u8]) -> Result<usize, SerialException>;
    /// `serial.close()`.
    fn close(&mut self);
}

/// `serial.Serial(port, baud, timeout=0, write_timeout=0)`.
pub type SerialOpener =
    Arc<dyn Fn(&str, i64) -> Result<Box<dyn SerialPort>, SerialOpenError> + Send + Sync>;

/// The production [`SerialOpener`].
pub fn posix_serial_opener() -> SerialOpener {
    Arc::new(open_posix_serial)
}

#[cfg(any(target_arch = "mips", target_arch = "mips32r6", target_arch = "mips64", target_arch = "mips64r6"))]
mod open_flags {
    pub const O_NOCTTY: i32 = 0x0800;
    pub const O_NONBLOCK: i32 = 0x0080;
}
#[cfg(not(any(target_arch = "mips", target_arch = "mips32r6", target_arch = "mips64", target_arch = "mips64r6")))]
mod open_flags {
    pub const O_NOCTTY: i32 = 0o400;
    pub const O_NONBLOCK: i32 = 0o4000;
}

unsafe extern "C" {
    fn cfmakeraw(termios_p: *mut kos::termios::Termios);
    fn cfsetspeed(termios_p: *mut kos::termios::Termios, speed: std::os::raw::c_uint)
        -> std::os::raw::c_int;
}

/// `c_cflag`/`c_iflag` bits shared by the Linux x86 and MIPS termios ABIs.
const CLOCAL: u32 = 0o4000;
const CREAD: u32 = 0o200;
const CSTOPB: u32 = 0o100;
const CRTSCTS: u32 = 0o20000000000;
const IXANY: u32 = 0o4000;
const IXOFF: u32 = 0o10000;
const TCIFLUSH: i32 = 0;

/// A pyserial-style non-blocking tty.
struct PosixSerial {
    fd: i32,
}

/// `serial.Serial(port, baud, timeout=0, write_timeout=0)` on Linux (`serialposix.py`).
pub fn open_posix_serial(port: &str, baud: i64) -> Result<Box<dyn SerialPort>, SerialOpenError> {
    use std::os::unix::fs::OpenOptionsExt;
    use std::os::unix::io::IntoRawFd;

    let serial_err = |msg: String| SerialOpenError::Serial(SerialException(msg));
    // `Serial.baudrate` setter (`serialutil.py`), run before `open()`.
    if baud < 0 {
        return Err(SerialOpenError::Other(KlippyError::Internal {
            class_name: "ValueError".to_string(),
            message: format!("Not a valid baudrate: {}", baud),
        }));
    }
    // `os.open` failure: `SerialException(msg.errno, "could not open port {}: {}".format(port,
    // msg))`, whose `str` prefixes `[Errno N] ` to the text, and the `OSError` text is
    // `[Errno N] strerror: 'port'`.
    let file = std::fs::OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(open_flags::O_NOCTTY | open_flags::O_NONBLOCK)
        .open(port)
        .map_err(|e| match e.raw_os_error() {
            Some(errno) => serial_err(format!(
                "[Errno {errno}] could not open port {port}: [Errno {errno}] {}: {}",
                kos::fd::strerror(errno),
                pycompat::text::str_repr(port)
            )),
            None => serial_err(format!("could not open port {}: {}", port, e)),
        })?;
    let fd = file.into_raw_fd();
    let configure = || -> Result<(), String> {
        let speed = u32::try_from(baud).map_err(|_| "Invalid argument".to_string())?;
        let mut t = kos::termios::tcgetattr(fd).map_err(|e| e.to_string())?;
        // SAFETY: `t` is a valid termios; both calls only write into it.
        let rc = unsafe {
            cfmakeraw(&mut t);
            cfsetspeed(&mut t, speed)
        };
        if rc != 0 {
            return Err(std::io::Error::last_os_error().to_string());
        }
        t.c_iflag &= !(IXANY | IXOFF);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= !(CSTOPB | CRTSCTS);
        kos::termios::tcsetattr(fd, kos::termios::TCSANOW, &t).map_err(|e| e.to_string())?;
        kos::termios::tcflush(fd, TCIFLUSH).map_err(|e| e.to_string())
    };
    if let Err(msg) = configure() {
        kos::fd::close(fd);
        return Err(serial_err(format!("Could not configure port: {}", msg)));
    }
    Ok(Box::new(PosixSerial { fd }))
}

impl PosixSerial {
    fn not_open() -> SerialException {
        SerialException("Attempting to use a port that is not open".to_string())
    }
}

impl SerialPort for PosixSerial {
    fn read(&mut self) -> Result<Vec<u8>, SerialException> {
        if self.fd < 0 {
            return Err(Self::not_open());
        }
        // `select.select([fd], [], [], 0)` followed by `os.read(fd, 1)`.
        let mut fds = [kos::fd::PollFd::new(self.fd, kos::fd::POLLIN)];
        match kos::fd::poll_retry(&mut fds, 0) {
            Ok(0) => return Ok(Vec::new()),
            Ok(_) => {}
            Err(e) => return Err(SerialException(format!("read failed: {}", e))),
        }
        let mut buf = [0u8; 1];
        match kos::fd::read(self.fd, &mut buf) {
            Ok(0) => Err(SerialException(
                "device reports readiness to read but returned no data (device disconnected or \
                 multiple access on port?)"
                    .to_string(),
            )),
            Ok(n) => Ok(buf[..n].to_vec()),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
                ) =>
            {
                Ok(Vec::new())
            }
            Err(e) => Err(SerialException(format!("read failed: {}", e))),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<usize, SerialException> {
        if self.fd < 0 {
            return Err(Self::not_open());
        }
        match kos::fd::write(self.fd, data) {
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::Interrupted
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(SerialException(format!("write failed: {}", e))),
        }
    }

    fn close(&mut self) {
        if self.fd >= 0 {
            kos::fd::close(self.fd);
            self.fd = -1;
        }
    }
}

impl Drop for PosixSerial {
    fn drop(&mut self) {
        self.close();
    }
}

// ---------------------------------------------------------------------------
// Python helpers
// ---------------------------------------------------------------------------

fn internal(class_name: &str, message: impl Into<String>) -> KlippyError {
    KlippyError::Internal { class_name: class_name.to_string(), message: message.into() }
}

fn py_error(e: PyError) -> KlippyError {
    KlippyError::Internal { class_name: e.kind.to_string(), message: e.msg }
}

fn pct(fmt: &str, args: &[PctValue]) -> KResult<String> {
    percent_format(fmt, args).map_err(py_error)
}

fn list_index_error() -> KlippyError {
    internal("IndexError", "list index out of range")
}

/// `str.isspace` of one character: Rust's `White_Space` plus the ASCII separators
/// `\x1c`..`\x1f` that Python treats as whitespace.
fn py_is_space(c: char) -> bool {
    c.is_whitespace() || ('\x1c'..='\x1f').contains(&c)
}

/// `str.strip()`.
fn py_strip(s: &str) -> &str {
    s.trim_matches(py_is_space)
}

/// `str.split()`.
fn py_split(s: &str) -> Vec<String> {
    s.split(py_is_space).filter(|p| !p.is_empty()).map(str::to_string).collect()
}

/// `s[start:]` for a non-negative `start`, in code points.
fn slice_from(s: &str, start: usize) -> String {
    s.chars().skip(start).collect()
}

/// `s[start:end]` for non-negative bounds, in code points.
fn slice_range(s: &str, start: usize, end: usize) -> String {
    s.chars().skip(start).take(end.saturating_sub(start)).collect()
}

/// `raw.decode('UTF-8', errors='ignore')`.
fn decode_utf8_ignore(raw: &[u8]) -> String {
    let mut out = String::new();
    for chunk in raw.utf8_chunks() {
        out.push_str(chunk.valid());
    }
    out
}

fn int_of(s: &str, base: i64) -> KResult<PyInt> {
    int_from_string(s, base).map_err(py_error)
}

fn i64_of(n: &PyInt) -> Option<i64> {
    n.inner().to_string().parse::<i64>().ok()
}

fn pct_int(n: &PyInt) -> PctValue {
    PctValue::Int(n.clone())
}

fn attribute_error_none(attr: &str) -> KlippyError {
    internal("AttributeError", format!("'NoneType' object has no attribute '{}'", attr))
}

/// Runs a timer body; an error is an uncaught Python exception, which ends klippy.
fn raise_in_timer(result: KResult<f64>) -> f64 {
    match result {
        Ok(wake) => wake,
        Err(e) => e.raise(),
    }
}

fn lookup<T>(printer: &Printer, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    lookup_interface::<T>(&**printer, name, LookupDefault::Raise)?
        .ok_or_else(|| internal("AttributeError", format!("printer object '{}' is missing", name)))
}

// ---------------------------------------------------------------------------
// The object
// ---------------------------------------------------------------------------

/// One entry of `omega_pings` / `omega_pongs`: `{"number": int, "percent": float}`.
#[derive(Clone, Debug)]
struct Ping {
    number: i64,
    percent: f64,
}

impl Ping {
    fn to_py(&self) -> PyValue {
        let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
        d.insert(PyValue::Str("number".to_string()), PyValue::Int(PyInt::from_i64(self.number)));
        d.insert(PyValue::Str("percent".to_string()), PyValue::Float(self.percent));
        PyValue::Dict(d)
    }
}

/// Mutable attributes of the Python object.
struct State {
    serial: Option<Box<dyn SerialPort>>,
    files: Vec<String>,
    is_setup_complete: bool,
    is_splicing: bool,
    is_loading: bool,
    remaining_load_length: Option<PyInt>,
    omega_algorithms: Vec<String>,
    omega_algorithms_counter: usize,
    omega_splices: Vec<(PyInt, String)>,
    omega_splices_counter: usize,
    omega_pings: Vec<Ping>,
    omega_pongs: Vec<Ping>,
    omega_current_ping: Option<String>,
    omega_header: Vec<Option<String>>,
    omega_header_counter: usize,
    omega_last_command: String,
    /// Only written by `cmd_O25` (the Python `omega_drives`), logged there.
    omega_drives: Vec<String>,
    read_timer: Option<Timer>,
    read_buffer: String,
    read_queue: VecDeque<String>,
    write_timer: Option<Timer>,
    write_queue: VecDeque<Option<String>>,
    heartbeat_timer: Option<Timer>,
    heartbeat: Option<f64>,
    signal_disconnect: bool,
    is_printing: bool,
    smart_load_timer: Option<Timer>,
}

impl State {
    /// `Palette2._reset` (`palette2.py:115-131`) on a fresh state, plus the attributes that
    /// `__init__` sets after calling it.
    fn new() -> State {
        State {
            serial: None,
            files: Vec::new(),
            is_setup_complete: false,
            is_splicing: false,
            is_loading: false,
            remaining_load_length: None,
            omega_algorithms: Vec::new(),
            omega_algorithms_counter: 0,
            omega_splices: Vec::new(),
            omega_splices_counter: 0,
            omega_pings: Vec::new(),
            omega_pongs: Vec::new(),
            omega_current_ping: None,
            omega_header: vec![None; 9],
            omega_header_counter: 0,
            omega_last_command: String::new(),
            omega_drives: Vec::new(),
            read_timer: None,
            read_buffer: String::new(),
            read_queue: VecDeque::new(),
            write_timer: None,
            write_queue: VecDeque::new(),
            heartbeat_timer: None,
            heartbeat: None,
            signal_disconnect: false,
            is_printing: false,
            smart_load_timer: None,
        }
    }

    /// `Palette2._reset`.
    fn reset(&mut self) {
        self.files = Vec::new();
        self.is_setup_complete = false;
        self.is_splicing = false;
        self.is_loading = false;
        self.remaining_load_length = None;
        self.omega_algorithms = Vec::new();
        self.omega_algorithms_counter = 0;
        self.omega_splices = Vec::new();
        self.omega_splices_counter = 0;
        self.omega_pings = Vec::new();
        self.omega_pongs = Vec::new();
        self.omega_current_ping = None;
        self.omega_header = vec![None; 9];
        self.omega_header_counter = 0;
        self.omega_last_command = String::new();
    }
}

/// Port of the `Palette2` class (`klippy/extras/palette2.py:43`).
pub struct Palette2 {
    printer: Printer,
    reactor: Reactor,
    virtual_sdcard: Arc<dyn VirtualSDApi>,
    pause_resume: Arc<dyn PauseResumeApi>,
    gcode: GCode,
    opener: SerialOpener,
    serial_port: String,
    baud: i64,
    feedrate_splice: f64,
    feedrate_normal: f64,
    auto_load_speed: i64,
    auto_cancel_variation: Option<f64>,
    /// Late-bound `self` for handlers and timers that are registered before the `Arc` exists.
    me: Arc<GCell<Weak<Palette2>>>,
    st: GCell<State>,
}

type SelfSlot = Arc<GCell<Weak<Palette2>>>;

/// A G-code handler that resolves `self` when it runs.
fn late_handler(
    slot: &SelfSlot,
    f: impl Fn(&Palette2, &GCmd) -> KResult<()> + Send + 'static,
) -> GCodeCommandHandler {
    let slot = slot.clone();
    Box::new(move |gcmd| {
        let this = slot
            .cloned()
            .upgrade()
            .ok_or_else(|| internal("ReferenceError", "weakly-referenced object no longer exists"))?;
        f(&this, gcmd)
    })
}

impl Palette2 {
    /// `Palette2.__init__` (`palette2.py:44-113`).
    pub fn new(config: &Config, opener: SerialOpener) -> KResult<Arc<Palette2>> {
        let printer = config.get_printer();
        let reactor = printer.get_reactor();
        let virtual_sdcard_obj = match printer.load_object(config, "virtual_sdcard", LookupDefault::Raise) {
            Ok(obj) => obj,
            Err(KlippyError::Config(_)) => {
                return Err(KlippyError::Config(
                    "Palette 2 requires [virtual_sdcard] to work, please add it to your config!"
                        .to_string(),
                ));
            }
            Err(e) => return Err(e),
        };
        let virtual_sdcard = require_loaded::<dyn VirtualSDApi>(virtual_sdcard_obj, "virtual_sdcard")?;
        let pause_resume_obj = match printer.load_object(config, "pause_resume", LookupDefault::Raise) {
            Ok(obj) => obj,
            Err(KlippyError::Config(_)) => {
                return Err(KlippyError::Config(
                    "Palette 2 requires [pause_resume] to work, please add it to your config!"
                        .to_string(),
                ));
            }
            Err(e) => return Err(e),
        };
        let pause_resume = require_loaded::<dyn PauseResumeApi>(pause_resume_obj, "pause_resume")?;
        printer.load_object(config, "gcode_move", LookupDefault::Raise)?;
        let gcode: GCode = lookup::<dyn GCodeApi>(&printer, "gcode")?;

        let slot: SelfSlot = Arc::new(GCell::new(Weak::new()));
        let desc = |text: &str| RegisterCommandOpts { when_not_ready: false, desc: Some(text.to_string()) };
        gcode.register_command(
            "PALETTE_CONNECT",
            Some(late_handler(&slot, |p, g| p.cmd_connect(g))),
            desc(CMD_CONNECT_HELP),
        )?;
        gcode.register_command(
            "PALETTE_DISCONNECT",
            Some(late_handler(&slot, |p, _| p.cmd_disconnect())),
            desc(CMD_DISCONNECT_HELP),
        )?;
        gcode.register_command(
            "PALETTE_CLEAR",
            Some(late_handler(&slot, |p, g| p.cmd_clear(g))),
            desc(CMD_CLEAR_HELP),
        )?;
        gcode.register_command(
            "PALETTE_CUT",
            Some(late_handler(&slot, |p, g| p.cmd_cut(g))),
            desc(CMD_CUT_HELP),
        )?;
        gcode.register_command(
            "PALETTE_SMART_LOAD",
            Some(late_handler(&slot, |p, g| p.cmd_smart_load(g))),
            desc(CMD_SMART_LOAD_HELP),
        )?;

        let serial_port = config.get("serial", GetStrOpts::default())?.unwrap_or_default();
        if serial_port.is_empty() {
            return Err(KlippyError::Config("Invalid serial port specific for Palette 2".to_string()));
        }
        let baud = config
            .getint("baud", GetIntOpts { default: Some(Some(115200)), ..GetIntOpts::default() })?
            .unwrap_or(115200);
        let feedrate_splice = config
            .getfloat(
                "feedrate_splice",
                GetFloatOpts {
                    default: Some(Some(0.8)),
                    minval: Some(0.),
                    maxval: Some(1.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(0.8);
        let feedrate_normal = config
            .getfloat(
                "feedrate_normal",
                GetFloatOpts {
                    default: Some(Some(1.0)),
                    minval: Some(0.),
                    maxval: Some(1.),
                    ..GetFloatOpts::default()
                },
            )?
            .unwrap_or(1.0);
        let auto_load_speed = config
            .getint("auto_load_speed", GetIntOpts { default: Some(Some(2)), ..GetIntOpts::default() })?
            .unwrap_or(2);
        let auto_cancel_variation = config.getfloat(
            "auto_cancel_variation",
            GetFloatOpts {
                default: Some(None),
                minval: Some(0.01),
                maxval: Some(0.2),
                ..GetFloatOpts::default()
            },
        )?;

        let this = Arc::new(Palette2 {
            printer,
            reactor,
            virtual_sdcard,
            pause_resume,
            gcode,
            opener,
            serial_port,
            baud,
            feedrate_splice,
            feedrate_normal,
            auto_load_speed,
            auto_cancel_variation,
            me: slot.clone(),
            st: GCell::new(State::new()),
        });
        slot.replace(Arc::downgrade(&this));

        // Omega code matchers: `O0`..`O32` (`palette2.py:90-99`).
        for i in 0..33u32 {
            let cmd = format!("O{}", i);
            let (handler, desc) = this.omega_handler(i);
            this.gcode.register_command(
                &cmd,
                Some(handler),
                RegisterCommandOpts { when_not_ready: false, desc },
            )?;
        }
        Ok(this)
    }

    fn arc(&self) -> Arc<Palette2> {
        self.me.cloned().upgrade().expect("Palette2 is alive while its callbacks run")
    }

    /// `getattr(self, 'cmd_O<i>', None)` with its `_help`, or `cmd_OmegaDefault`.
    fn omega_handler(&self, i: u32) -> (GCodeCommandHandler, Option<String>) {
        let slot = &self.me;
        match i {
            1 => (late_handler(slot, |p, g| p.cmd_o1(g)), Some(CMD_O1_HELP.to_string())),
            9 => (late_handler(slot, |p, g| p.cmd_o9(g)), Some(CMD_O9_HELP.to_string())),
            21 => (late_handler(slot, |p, g| p.cmd_o21(g)), None),
            22 => (late_handler(slot, |p, g| p.cmd_o22(g)), None),
            23 => (late_handler(slot, |p, g| p.cmd_o23(g)), None),
            24 => (late_handler(slot, |p, g| p.cmd_o24(g)), None),
            25 => (late_handler(slot, |p, g| p.cmd_o25(g)), None),
            26 => (late_handler(slot, |p, g| p.cmd_o26(g)), None),
            27 => (late_handler(slot, |p, g| p.cmd_o27(g)), None),
            28 => (late_handler(slot, |p, g| p.cmd_o28(g)), None),
            29 => (late_handler(slot, |p, g| p.cmd_o29(g)), None),
            30 => (late_handler(slot, |p, g| p.cmd_o30(g)), None),
            31 => (late_handler(slot, |p, g| p.cmd_o31(g)), None),
            32 => (late_handler(slot, |p, g| p.cmd_o32(g)), None),
            _ => (late_handler(slot, |p, g| p.cmd_omega_default(g)), None),
        }
    }

    // ---- state accessors -------------------------------------------------------------

    /// `self.serial is not None`.
    pub fn serial_connected(&self) -> bool {
        self.st.borrow().serial.is_some()
    }

    /// `self.is_printing`.
    pub fn is_printing(&self) -> bool {
        self.st.borrow().is_printing
    }

    /// `self.is_loading`.
    pub fn is_loading(&self) -> bool {
        self.st.borrow().is_loading
    }

    /// `self.is_splicing`.
    pub fn is_splicing(&self) -> bool {
        self.st.borrow().is_splicing
    }

    /// `self.is_setup_complete`.
    pub fn is_setup_complete(&self) -> bool {
        self.st.borrow().is_setup_complete
    }

    /// `self.signal_disconnect`.
    pub fn signal_disconnect(&self) -> bool {
        self.st.borrow().signal_disconnect
    }

    /// `self.heartbeat`.
    pub fn heartbeat(&self) -> Option<f64> {
        self.st.borrow().heartbeat
    }

    /// `self.smart_load_timer is not None`.
    pub fn smart_load_timer_active(&self) -> bool {
        self.st.borrow().smart_load_timer.is_some()
    }

    /// `(read_timer, write_timer, heartbeat_timer)` each `is not None`.
    pub fn timers_active(&self) -> (bool, bool, bool) {
        let st = self.st.borrow();
        (st.read_timer.is_some(), st.write_timer.is_some(), st.heartbeat_timer.is_some())
    }

    // ---- helpers -----------------------------------------------------------------------

    fn put(&self, line: Option<String>) {
        self.st.borrow_mut().write_queue.push_back(line);
    }

    fn put_str(&self, line: &str) {
        self.put(Some(line.to_string()));
    }

    /// `Palette2._check_P2` (`palette2.py:133-138`).
    fn check_p2(&self, gcmd: Option<&GCmd>) -> bool {
        if self.st.borrow().serial.is_some() {
            return true;
        }
        if let Some(gcmd) = gcmd {
            gcmd.respond_info(INFO_NOT_CONNECTED, true);
        }
        false
    }

    // ---- G-code commands ---------------------------------------------------------------

    /// `cmd_Connect` (`palette2.py:142-173`).
    pub fn cmd_connect(&self, gcmd: &GCmd) -> KResult<()> {
        if self.st.borrow().serial.is_some() {
            gcmd.respond_info("Palette 2 serial port is already active, disconnect first", true);
            return Ok(());
        }

        self.st.borrow_mut().signal_disconnect = false;
        log::info(&format!(
            "Connecting to Palette 2 on port ({}) at ({})",
            self.serial_port, self.baud
        ));
        match (self.opener)(&self.serial_port, self.baud) {
            Ok(port) => self.st.borrow_mut().serial = Some(port),
            Err(SerialOpenError::Serial(_)) => {
                gcmd.respond_info("Unable to connect to the Palette 2", true);
                return Ok(());
            }
            Err(SerialOpenError::Other(e)) => return Err(e),
        }

        {
            let mut st = self.st.borrow_mut();
            st.write_queue.clear();
            st.read_queue.clear();
        }

        let this = self.arc();
        let read_timer = self
            .reactor
            .register_timer(move |t| raise_in_timer(this.run_read(t)), NOW);
        self.st.borrow_mut().read_timer = Some(read_timer);
        let this = self.arc();
        let write_timer = self
            .reactor
            .register_timer(move |t| raise_in_timer(this.run_write(t)), NOW);
        self.st.borrow_mut().write_timer = Some(write_timer);
        let this = self.arc();
        let heartbeat_timer = self
            .reactor
            .register_timer(move |t| raise_in_timer(this.run_heartbeat(t)), NOW);
        self.st.borrow_mut().heartbeat_timer = Some(heartbeat_timer);

        // Tell the device we're alive
        self.put_str("\n");
        self.put_str(COMMAND_FIRMWARE);
        self.wait_for_heartbeat()
    }

    /// `cmd_Disconnect` (`palette2.py:177-189`); the G-code handler ignores `gcmd`.
    pub fn cmd_disconnect(&self) -> KResult<()> {
        self.gcode.respond_info("Disconnecting from Palette 2", true);
        let serial = self.st.borrow_mut().serial.take();
        if let Some(mut serial) = serial {
            serial.close();
        }

        {
            let st = self.st.borrow();
            for timer in [&st.read_timer, &st.write_timer, &st.heartbeat_timer] {
                match timer {
                    Some(timer) => self.reactor.unregister_timer(timer),
                    None => return Err(attribute_error_none("waketime")),
                }
            }
        }
        let mut st = self.st.borrow_mut();
        st.read_timer = None;
        st.write_timer = None;
        st.heartbeat = None;
        st.is_printing = false;
        Ok(())
    }

    /// `cmd_Clear` (`palette2.py:193-197`).
    pub fn cmd_clear(&self, gcmd: &GCmd) -> KResult<()> {
        log::info("Clearing Palette 2 input and output");
        if self.check_p2(Some(gcmd)) {
            for line in COMMAND_CLEAR {
                self.put_str(line);
            }
        }
        Ok(())
    }

    /// `cmd_Cut` (`palette2.py:201-204`).
    pub fn cmd_cut(&self, gcmd: &GCmd) -> KResult<()> {
        log::info("Cutting outgoing filament in Palette 2");
        if self.check_p2(Some(gcmd)) {
            self.put_str(COMMAND_CUT);
        }
        Ok(())
    }

    /// `cmd_Smart_Load` (`palette2.py:208-214`).
    pub fn cmd_smart_load(&self, gcmd: &GCmd) -> KResult<()> {
        if self.check_p2(Some(gcmd)) {
            if !self.st.borrow().is_loading {
                gcmd.respond_info("Cannot auto load when the Palette 2 is not ready", true);
                return Ok(());
            }
            self.p2cmd_o102()?;
        }
        Ok(())
    }

    /// `cmd_OmegaDefault` (`palette2.py:216-219`).
    pub fn cmd_omega_default(&self, gcmd: &GCmd) -> KResult<()> {
        log::debug(&format!("Omega Code: {}", gcmd.get_command()));
        if self.check_p2(Some(gcmd)) {
            self.put(Some(gcmd.get_commandline()));
        }
        Ok(())
    }

    /// `_wait_for_heartbeat` (`palette2.py:221-232`). Waits forever while no heartbeat has
    /// ever arrived, like the Python loop.
    fn wait_for_heartbeat(&self) -> KResult<()> {
        let start_ts = self.reactor.monotonic();
        let mut curr_ts = start_ts;
        loop {
            let waiting = match self.st.borrow().heartbeat {
                None => true,
                Some(hb) => hb < (curr_ts - SETUP_TIMEOUT) && start_ts > (curr_ts - SETUP_TIMEOUT),
            };
            if !waiting {
                break;
            }
            curr_ts = self.reactor.pause(curr_ts + 1.);
        }

        let heartbeat = self.st.borrow().heartbeat;
        let Some(heartbeat) = heartbeat else {
            return Err(internal(
                "TypeError",
                "'<' not supported between instances of 'NoneType' and 'float'",
            ));
        };
        if heartbeat < (curr_ts - SETUP_TIMEOUT) {
            self.st.borrow_mut().signal_disconnect = true;
            return Err(KlippyError::Command("No response from Palette 2".to_string()));
        }
        Ok(())
    }

    /// `cmd_O1` (`palette2.py:237-248`).
    pub fn cmd_o1(&self, gcmd: &GCmd) -> KResult<()> {
        log::info("Initializing print with Palette 2");
        if !self.check_p2(Some(gcmd)) {
            return Err(KlippyError::Command(
                "Cannot initialize print, palette 2 is not connected".to_string(),
            ));
        }

        {
            let st = self.st.borrow();
            match st.heartbeat_timer.as_ref() {
                Some(timer) => self.reactor.update_timer(timer, NOW),
                None => return Err(attribute_error_none("timer_is_running")),
            }
        }
        self.wait_for_heartbeat()?;
        self.put(Some(gcmd.get_commandline()));
        self.gcode.respond_info("Palette 2 waiting on user to complete setup", true);
        self.pause_resume.send_pause_command()
    }

    /// `cmd_O9` (`palette2.py:252-256`).
    pub fn cmd_o9(&self, gcmd: &GCmd) -> KResult<()> {
        log::info("Print finished, resetting Palette 2 state");
        if self.check_p2(Some(gcmd)) {
            self.put(Some(gcmd.get_commandline()));
        }
        self.st.borrow_mut().is_printing = false;
        Ok(())
    }

    /// `cmd_O21` (`palette2.py:258-262`).
    pub fn cmd_o21(&self, gcmd: &GCmd) -> KResult<()> {
        log::debug(&format!("Omega version: {}", gcmd.get_commandline()));
        let mut st = self.st.borrow_mut();
        st.reset();
        st.omega_header[0] = Some(gcmd.get_commandline());
        st.is_printing = true;
        Ok(())
    }

    fn set_header(&self, idx: usize, label: &str, gcmd: &GCmd) {
        let commandline = gcmd.get_commandline();
        log::debug(&format!("{}: {}", label, commandline));
        self.st.borrow_mut().omega_header[idx] = Some(commandline);
    }

    /// `cmd_O22` (`palette2.py:264-266`).
    pub fn cmd_o22(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(1, "Omega printer profile", gcmd);
        Ok(())
    }

    /// `cmd_O23` (`palette2.py:268-270`).
    pub fn cmd_o23(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(2, "Omega slicer profile", gcmd);
        Ok(())
    }

    /// `cmd_O24` (`palette2.py:272-274`).
    pub fn cmd_o24(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(3, "Omega PPM", gcmd);
        Ok(())
    }

    /// `cmd_O25` (`palette2.py:276-285`).
    pub fn cmd_o25(&self, gcmd: &GCmd) -> KResult<()> {
        let commandline = gcmd.get_commandline();
        log::debug(&format!("Omega inputs: {}", commandline));
        let mut drives = py_split(&slice_from(&commandline, 4));
        for (idx, drive) in drives.iter_mut().enumerate() {
            let state: String = drive.chars().take(2).collect();
            if state == "D1" {
                *drive = format!("U{}", 60 + idx);
            }
        }
        let drives: Vec<String> = drives.into_iter().filter(|d| d != "D0").collect();
        let rendered =
            PyValue::List(drives.iter().map(|d| PyValue::Str(d.clone())).collect()).repr();
        {
            let mut st = self.st.borrow_mut();
            st.omega_header[4] = Some(commandline);
            st.omega_drives = drives;
        }
        log::info(&format!("Omega drives: {}", rendered));
        Ok(())
    }

    /// `cmd_O26` (`palette2.py:287-289`).
    /// Unlike the other header logs, the reference omits the colon here.
    pub fn cmd_o26(&self, gcmd: &GCmd) -> KResult<()> {
        let commandline = gcmd.get_commandline();
        log::debug(&format!("Omega splices {}", commandline));
        self.st.borrow_mut().omega_header[5] = Some(commandline);
        Ok(())
    }

    /// `cmd_O27` (`palette2.py:291-293`).
    pub fn cmd_o27(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(6, "Omega pings", gcmd);
        Ok(())
    }

    /// `cmd_O28` (`palette2.py:295-297`).
    pub fn cmd_o28(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(7, "Omega MSF NA", gcmd);
        Ok(())
    }

    /// `cmd_O29` (`palette2.py:299-301`).
    pub fn cmd_o29(&self, gcmd: &GCmd) -> KResult<()> {
        self.set_header(8, "Omega MSF NH", gcmd);
        Ok(())
    }

    /// `cmd_O30` (`palette2.py:303-315`).
    pub fn cmd_o30(&self, gcmd: &GCmd) -> KResult<()> {
        // The slices cannot raise `IndexError`, so the first `except` never runs.
        let commandline = gcmd.get_commandline();
        let param_drive = slice_range(&commandline, 5, 6);
        let param_distance = slice_from(&commandline, 8);
        match int_from_string(&param_drive, 10) {
            Ok(drive) => self.st.borrow_mut().omega_splices.push((drive, param_distance.clone())),
            Err(_) => gcmd.respond_info("Incorrectly formatted splice command", true),
        }
        log::debug(&format!(
            "Omega splice command drive {} distance {}",
            param_drive, param_distance
        ));
        Ok(())
    }

    /// `cmd_O31` (`palette2.py:317-324`).
    pub fn cmd_o31(&self, gcmd: &GCmd) -> KResult<()> {
        if self.check_p2(Some(gcmd)) {
            let commandline = gcmd.get_commandline();
            self.st.borrow_mut().omega_current_ping = Some(commandline.clone());
            log::debug(&format!("Omega ping command: {}", commandline));

            self.put_str(COMMAND_PING);
            // The Python builds the command and never runs it.
            let mut params: IndexMap<String, String> = IndexMap::new();
            params.insert("P".to_string(), "10".to_string());
            let _ = self.gcode.create_gcode_command("G4", "G4", &params);
        }
        Ok(())
    }

    /// `cmd_O32` (`palette2.py:326-328`).
    pub fn cmd_o32(&self, gcmd: &GCmd) -> KResult<()> {
        let commandline = gcmd.get_commandline();
        log::debug(&format!("Omega algorithm: {}", commandline));
        self.st.borrow_mut().omega_algorithms.push(commandline);
        Ok(())
    }

    // ---- Palette 2 -> printer commands --------------------------------------------------

    /// `p2cmd_O20` (`palette2.py:330-366`).
    fn p2cmd_o20(&self, params: &[String]) -> KResult<()> {
        if !self.st.borrow().is_printing {
            return Ok(());
        }

        let first = params.first().ok_or_else(list_index_error)?;
        // First print, we can ignore
        if first == "D5" {
            log::info("First print on Palette");
            return Ok(());
        }

        // `except (TypeError, IndexError)` cannot fire here; a `ValueError` is uncaught.
        let n = int_of(&slice_from(first, 1), 10)?;
        match i64_of(&n) {
            Some(0) => {
                let counter = self.st.borrow().omega_header_counter;
                log::info(&format!("Sending omega header {}", counter));
                let header = self.st.borrow().omega_header.get(counter).cloned();
                let header = header.ok_or_else(list_index_error)?;
                self.put(header);
                self.st.borrow_mut().omega_header_counter = counter + 1;
            }
            Some(1) => {
                let counter = self.st.borrow().omega_splices_counter;
                log::info(&format!("Sending splice info {}", counter));
                let splice = self.st.borrow().omega_splices.get(counter).cloned();
                let (drive, distance) = splice.ok_or_else(list_index_error)?;
                let line = pct("O30 D%d D%s", &[pct_int(&drive), PctValue::Str(distance)])?;
                self.put(Some(line));
                self.st.borrow_mut().omega_splices_counter = counter + 1;
            }
            Some(2) => {
                let ping = self.st.borrow().omega_current_ping.clone();
                log::info(&format!(
                    "Sending current ping info {}",
                    ping.as_deref().unwrap_or("None")
                ));
                self.put(ping);
            }
            Some(4) => {
                let counter = self.st.borrow().omega_algorithms_counter;
                log::info(&format!("Sending algorithm info {}", counter));
                let algorithm = self.st.borrow().omega_algorithms.get(counter).cloned();
                let algorithm = algorithm.ok_or_else(list_index_error)?;
                self.put(Some(algorithm));
                self.st.borrow_mut().omega_algorithms_counter = counter + 1;
            }
            Some(8) => {
                log::info("Resending the last command to Palette 2");
                let last = self.st.borrow().omega_last_command.clone();
                self.put(Some(last));
            }
            _ => {}
        }
        Ok(())
    }

    /// `check_ping_variation` of `p2cmd_O34` (`palette2.py:372-379`).
    fn check_ping_variation(&self, last_ping: f64) -> KResult<()> {
        if let Some(variation) = self.auto_cancel_variation {
            let ping_max = 100. + (variation * 100.);
            let ping_min = 100. - (variation * 100.);
            if last_ping < ping_min || last_ping > ping_max {
                log::info("Ping variation is too high, cancelling print");
                self.gcode.run_script("CANCEL_PRINT")?;
            }
        }
        Ok(())
    }

    /// `p2cmd_O34` (`palette2.py:368-393`).
    fn p2cmd_o34(&self, params: &[String]) -> KResult<()> {
        if !self.st.borrow().is_printing {
            return Ok(());
        }

        if params.len() > 2 {
            let percent = float_from_string(&slice_from(&params[1], 1)).map_err(py_error)?;
            if params[0] == "D1" {
                let number = self.st.borrow().omega_pings.len() as i64 + 1;
                let line = pct(
                    "Ping %d, %d percent",
                    &[PctValue::Int(PyInt::from_i64(number)), PctValue::Float(percent)],
                )?;
                log::info(&line);
                self.st.borrow_mut().omega_pings.push(Ping { number, percent });
                self.check_ping_variation(percent)?;
            } else if params[0] == "D2" {
                let number = self.st.borrow().omega_pongs.len() as i64 + 1;
                let line = pct(
                    "Pong %d, %d percent",
                    &[PctValue::Int(PyInt::from_i64(number)), PctValue::Float(percent)],
                )?;
                log::info(&line);
                self.st.borrow_mut().omega_pongs.push(Ping { number, percent });
            }
        }
        Ok(())
    }

    /// `p2cmd_O40` (`palette2.py:395-397`).
    fn p2cmd_o40(&self) -> KResult<()> {
        log::info("Resume request from Palette 2");
        self.pause_resume.send_resume_command()
    }

    /// `p2cmd_O50` (`palette2.py:399-420`).
    fn p2cmd_o50(&self, params: &[String]) -> KResult<()> {
        if params.len() > 1 {
            let fw = slice_from(&params[0], 1);
            log::info(&format!("Palette 2 firmware version {} detected", fw));

            if fw.as_str() < "9.0.9" {
                return Err(KlippyError::Command(
                    "Palette 2 firmware version is too old, update to at least 9.0.9".to_string(),
                ));
            }
        } else {
            let files: Vec<String> = self
                .virtual_sdcard
                .get_file_list(true)?
                .into_iter()
                .map(|(file, _size)| file)
                .filter(|file| file.contains(".mcf.gcode"))
                .collect();
            self.st.borrow_mut().files = files.clone();
            for file in &files {
                self.put(Some(format!("{} D{}", COMMAND_FILENAME, file)));
            }
            self.put_str(COMMAND_FILENAMES_DONE);
        }
        Ok(())
    }

    /// `p2cmd_O53` (`palette2.py:422-429`).
    fn p2cmd_o53(&self, params: &[String]) -> KResult<()> {
        if params.len() > 1 && params[0] == "D1" {
            // `except (TypeError, IndexError)` covers the list lookup only; a `ValueError`
            // from `int()` and errors of `run_script` are uncaught.
            let idx = int_of(&slice_from(&params[1], 1), 16)?;
            let file = {
                let st = self.st.borrow();
                let len = st.files.len() as i64;
                i64_of(&idx).and_then(|i| {
                    let pos = if i < 0 { i + len } else { i };
                    if (0..len).contains(&pos) {
                        st.files.get((len - 1 - pos) as usize).cloned()
                    } else {
                        None
                    }
                })
            };
            match file {
                Some(file) => self.gcode.run_script(&format!("SDCARD_PRINT_FILE FILENAME={}", file))?,
                None => log::error("O53 has invalid command parameters"),
            }
        }
        Ok(())
    }

    /// `p2cmd_O88` (`palette2.py:431-437`).
    fn p2cmd_o88(&self, params: &[String]) -> KResult<()> {
        log::error("Palette 2 error detected");
        match params.first() {
            Some(first) => {
                let error = int_of(&slice_from(first, 1), 16)?;
                log::error(&format!("Palette 2 error code {}", error.inner()));
            }
            None => log::error("Unable to parse Palette 2 error"),
        }
        Ok(())
    }

    /// `p2cmd_O97` (`palette2.py:439-486`).
    fn p2cmd_o97(&self, params: &[String]) -> KResult<()> {
        #[derive(Clone, Copy)]
        enum Handler {
            PrintCancelling,
            PrintCancelled,
            LoadingOffset,
            LoadingOffsetStart,
            FeedrateStart,
            FeedrateEnd,
        }

        let mut matchers: Vec<(Handler, usize, &[&str])> = Vec::new();
        if self.st.borrow().is_printing {
            matchers.push((Handler::PrintCancelling, 2, &["U0", "D2"][..]));
            matchers.push((Handler::PrintCancelled, 2, &["U0", "D3"][..]));
            matchers.push((Handler::LoadingOffset, 2, &["U39"][..]));
            matchers.push((Handler::LoadingOffsetStart, 1, &["U39"][..]));
        }
        matchers.push((Handler::FeedrateStart, 3, &["U25", "D0"][..]));
        matchers.push((Handler::FeedrateEnd, 3, &["U25", "D1"][..]));

        // `_param_Matcher`: the first matcher whose prefix equals the parameters wins.
        for (handler, min_len, pattern) in matchers {
            if params.len() >= min_len
                && pattern.iter().enumerate().all(|(i, expected)| *expected == params[i])
            {
                return match handler {
                    Handler::PrintCancelling => {
                        log::info("Print Cancelling");
                        self.gcode.run_script("CLEAR_PAUSE")?;
                        self.gcode.run_script("CANCEL_PRINT")
                    }
                    Handler::PrintCancelled => {
                        log::info("Print Cancelled");
                        self.st.borrow_mut().reset();
                        Ok(())
                    }
                    Handler::LoadingOffsetStart => {
                        log::info("Waiting for user to load filament into printer");
                        self.st.borrow_mut().is_loading = true;
                        Ok(())
                    }
                    Handler::LoadingOffset => {
                        let remaining = int_of(&slice_from(&params[1], 1), 10)?;
                        self.st.borrow_mut().remaining_load_length = Some(remaining.clone());
                        log::debug(&pct(
                            "Loading filamant remaining %d",
                            &[pct_int(&remaining)],
                        )?);
                        let timer = {
                            let mut st = self.st.borrow_mut();
                            if remaining.sign() >= 0 && st.smart_load_timer.is_some() {
                                st.smart_load_timer.take()
                            } else {
                                None
                            }
                        };
                        if let Some(timer) = timer {
                            log::info("Smart load filament is complete");
                            self.reactor.unregister_timer(&timer);
                            self.st.borrow_mut().is_loading = false;
                        }
                        Ok(())
                    }
                    Handler::FeedrateStart => {
                        log::info(&pct(
                            "Setting feedrate to %f for splice",
                            &[PctValue::Float(self.feedrate_splice)],
                        )?);
                        self.st.borrow_mut().is_splicing = true;
                        self.gcode.run_script(&pct(
                            "M220 S%d",
                            &[PctValue::Float(self.feedrate_splice * 100.)],
                        )?)
                    }
                    Handler::FeedrateEnd => {
                        log::info(&pct(
                            "Setting feedrate to %f splice done",
                            &[PctValue::Float(self.feedrate_normal)],
                        )?);
                        self.st.borrow_mut().is_splicing = false;
                        self.gcode.run_script(&pct(
                            "M220 S%d",
                            &[PctValue::Float(self.feedrate_normal * 100.)],
                        )?)
                    }
                };
            }
        }
        Ok(())
    }

    /// `p2cmd_O100` (`palette2.py:488-491`).
    fn p2cmd_o100(&self) -> KResult<()> {
        log::info("Pause request from Palette 2");
        self.st.borrow_mut().is_setup_complete = true;
        self.pause_resume.send_pause_command()
    }

    /// `p2cmd_O102` (`palette2.py:493-504`).
    fn p2cmd_o102(&self) -> KResult<()> {
        let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
        let heater_obj = toolhead.get_extruder().get_heater()?;
        let heater = require_interface::<dyn HeaterApi>(&heater_obj, "extruder heater")?;
        if !heater.can_extrude() {
            self.put_str(COMMAND_SMART_LOAD_STOP);
            self.gcode.respond_info(
                "Unable to auto load filament, extruder is below minimum temp",
                true,
            );
            return Ok(());
        }

        if self.st.borrow().smart_load_timer.is_none() {
            log::info("Smart load starting");
            let this = self.arc();
            let timer = self
                .reactor
                .register_timer(move |t| raise_in_timer(this.run_smart_load(t)), NOW);
            self.st.borrow_mut().smart_load_timer = Some(timer);
        }
        Ok(())
    }

    /// `p2cmd` (`palette2.py:506-519`). The `Omega parameters are invalid` check never fires
    /// in Python: `all(res)` runs over non-empty strings only.
    pub fn p2cmd(&self, line: &str) -> KResult<()> {
        let t = py_split(line);
        let (ocode, params) = t.split_first().ok_or_else(list_index_error)?;
        match ocode.as_str() {
            "O20" => self.p2cmd_o20(params),
            "O34" => self.p2cmd_o34(params),
            "O40" => self.p2cmd_o40(),
            "O50" => self.p2cmd_o50(params),
            "O53" => self.p2cmd_o53(params),
            "O88" => self.p2cmd_o88(params),
            "O97" => self.p2cmd_o97(params),
            "O100" => self.p2cmd_o100(),
            "O102" => self.p2cmd_o102(),
            _ => Ok(()),
        }
    }

    // ---- timers --------------------------------------------------------------------------

    /// `_run_Read` (`palette2.py:533-580`).
    pub fn run_read(&self, eventtime: f64) -> KResult<f64> {
        if self.st.borrow().signal_disconnect {
            self.cmd_disconnect()?;
            return Ok(NEVER);
        }

        // Do non-blocking reads from serial and try to find lines
        loop {
            let read = match self.st.borrow_mut().serial.as_mut() {
                Some(serial) => serial.read(),
                None => return Err(attribute_error_none("read")),
            };
            let raw_bytes = match read {
                Ok(raw_bytes) => raw_bytes,
                Err(_) => {
                    log::error("Unable to communicate with the Palette 2");
                    self.cmd_disconnect()?;
                    return Ok(NEVER);
                }
            };
            if raw_bytes.is_empty() {
                break;
            }
            let new_buffer = decode_utf8_ignore(&raw_bytes);
            let mut st = self.st.borrow_mut();
            let mut text_buffer = format!("{}{}", st.read_buffer, new_buffer);
            while let Some(i) = text_buffer.find('\n') {
                let line = py_strip(&text_buffer[..=i]).to_string();
                st.read_queue.push_back(line);
                text_buffer = text_buffer[i + 1..].to_string();
            }
            st.read_buffer = text_buffer;
        }

        // Process any decoded lines from the device
        loop {
            let Some(text_line) = self.st.borrow_mut().read_queue.pop_front() else {
                break;
            };

            if !(text_line.contains(COMMAND_HEARTBEAT) || text_line.contains("Connection Okay")) {
                log::debug(&pct(
                    "%0.3f P2 -> : %s",
                    &[PctValue::Float(eventtime), PctValue::Str(text_line.clone())],
                )?);
            }

            // Received a heartbeat from the device
            if text_line == COMMAND_HEARTBEAT {
                self.st.borrow_mut().heartbeat = Some(eventtime);
            } else {
                match text_line.chars().next() {
                    Some('O') => self.p2cmd(&text_line)?,
                    Some(_) => {}
                    None => return Err(internal("IndexError", "string index out of range")),
                }
            }
        }

        Ok(eventtime + SERIAL_TIMER)
    }

    /// `_run_Heartbeat` (`palette2.py:582-592`).
    pub fn run_heartbeat(&self, eventtime: f64) -> KResult<f64> {
        self.put_str(COMMAND_HEARTBEAT);
        let eventtime = self.reactor.pause(eventtime + 5.);
        let heartbeat = self.st.borrow().heartbeat;
        if let Some(heartbeat) = heartbeat {
            if heartbeat != 0. && heartbeat < (eventtime - HEARTBEAT_TIMEOUT) {
                log::error("P2 has not responded to heartbeat");
                let (is_printing, is_setup_complete) = {
                    let st = self.st.borrow();
                    (st.is_printing, st.is_setup_complete)
                };
                if !is_printing || is_setup_complete {
                    self.cmd_disconnect()?;
                    return Ok(NEVER);
                }
            }
        }
        Ok(eventtime + HEARTBEAT_SEND)
    }

    /// `_run_Write` (`palette2.py:594-616`).
    pub fn run_write(&self, eventtime: f64) -> KResult<f64> {
        loop {
            let Some(text_line) = self.st.borrow_mut().write_queue.pop_front() else {
                break;
            };
            let text_line = match text_line {
                Some(line) if !line.is_empty() => line,
                _ => continue,
            };

            self.st.borrow_mut().omega_last_command = text_line.clone();
            let l = py_strip(&text_line);
            if !l.contains(COMMAND_HEARTBEAT) {
                log::debug(&format!("{} -> P2 : {}", float_str(self.reactor.monotonic()), l));
            }
            let terminated_line = format!("{}\n", l);
            let written = match self.st.borrow_mut().serial.as_mut() {
                Some(serial) => serial.write(terminated_line.as_bytes()),
                None => return Err(attribute_error_none("write")),
            };
            if written.is_err() {
                log::error("Unable to communicate with the Palette 2");
                self.st.borrow_mut().signal_disconnect = true;
                return Ok(NEVER);
            }
            return Ok(eventtime + SERIAL_TIMER);
        }
        Ok(eventtime + SERIAL_TIMER)
    }

    /// `_run_Smart_Load` (`palette2.py:618-640`).
    pub fn run_smart_load(&self, eventtime: f64) -> KResult<f64> {
        let (is_splicing, remaining) = {
            let st = self.st.borrow();
            (st.is_splicing, st.remaining_load_length.clone())
        };
        if !is_splicing {
            let remaining = remaining.ok_or_else(|| {
                internal(
                    "TypeError",
                    "'<' not supported between instances of 'NoneType' and 'int'",
                )
            })?;
            if remaining.sign() < 0 {
                // Make sure toolhead class isn't busy
                let toolhead = lookup::<dyn ToolHeadApi>(&self.printer, "toolhead")?;
                let (print_time, est_print_time, lookahead_empty) = toolhead.check_busy(eventtime);
                let idle_time = est_print_time - print_time;
                if !lookahead_empty || idle_time < 0.5 {
                    // `max(0., min(1., print_time - est_print_time))`
                    let diff = print_time - est_print_time;
                    let capped = if diff < 1. { diff } else { 1. };
                    let delay = if capped > 0. { capped } else { 0. };
                    return Ok(eventtime + delay);
                }

                let magnitude = PyInt::new(-remaining.inner().clone());
                let half = match magnitude.to_f64_exact() {
                    Some(v) if (v / 2.).is_finite() => v / 2.,
                    _ => {
                        return Err(internal(
                            "OverflowError",
                            "integer division result too large for a float",
                        ))
                    }
                };
                // `min(50, extrude / 2)`, then `if extrude <= 10: extrude = 1`
                let extrude = if half < 50. {
                    if half <= 10. {
                        PctValue::Int(PyInt::from_i64(1))
                    } else {
                        PctValue::Float(half)
                    }
                } else {
                    PctValue::Int(PyInt::from_i64(50))
                };
                log::info(&pct(
                    "Smart loading %dmm filament with %dmm remaining",
                    &[extrude.clone(), pct_int(&magnitude)],
                )?);

                self.gcode.run_script("G92 E0")?;
                let feed = PyInt::new(PyInt::from_i64(self.auto_load_speed).into_bigint() * 60i64);
                self.gcode
                    .run_script(&pct("G1 E%d F%d", &[extrude, pct_int(&feed)])?)?;
                return Ok(NOW);
            }
        }
        Ok(eventtime + AUTOLOAD_TIMER)
    }
}

/// The `T` interface of an object `load_object` returned.
fn require_loaded<T>(obj: Option<ObjectRef>, name: &str) -> KResult<Arc<T>>
where
    T: ?Sized + Send + Sync + 'static,
{
    let obj = obj.ok_or_else(|| internal("AttributeError", format!("printer object '{}' is missing", name)))?;
    require_interface::<T>(&obj, name)
}

impl PrinterObject for Palette2 {
    klippy_api::printer_object_base!();

    /// `get_status` (`palette2.py:642-650`).
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let st = self.st.borrow();
        let mut status: IndexMap<PyValue, PyValue> = IndexMap::new();
        status.insert(
            PyValue::Str("ping".to_string()),
            st.omega_pings.last().map(Ping::to_py).unwrap_or(PyValue::None),
        );
        status.insert(
            PyValue::Str("remaining_load_length".to_string()),
            match &st.remaining_load_length {
                Some(n) => PyValue::Int(n.clone()),
                None => PyValue::None,
            },
        );
        status.insert(PyValue::Str("is_splicing".to_string()), PyValue::Bool(st.is_splicing));
        Ok(Some(PyValue::Dict(status)))
    }
}

/// `load_config(config)` (`palette2.py:653-654`).
fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let this = Palette2::new(config, posix_serial_opener())?;
    Ok(Some(this as ObjectRef))
}

/// Module factory entry points (`module_factory` convention).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
