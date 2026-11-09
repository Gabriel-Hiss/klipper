//! Port of `klippy/webhooks.py` (the unix-socket JSON API used by Moonraker).
//!
//! Message framing, endpoint dispatch, mux endpoints, remote methods,
//! G-code subscription and `objects/list|query|subscribe` follow
//! `webhooks.py` line by line: JSON terminated by a single `0x03` byte,
//! 4096-byte reads with partial-frame buffering, each request run in its
//! own `register_callback` greenlet, response `{id, result|error}`.
//! The JSON codec is the `json` branch of `webhooks.py:9-37` (the
//! `msgspec` import is absent in the reference runtime), reproduced by
//! `pycompat::json` plus the `bytes` decoding `json.loads` applies.

use std::collections::VecDeque;
use std::path::Path;
use std::sync::{Arc, Weak};

use indexmap::IndexMap;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::GCodeApi;
use klippy_api::log;
use klippy_api::object::{self, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{
    ModuleFactory, Printer, EV_KLIPPY_ANALYZE_SHUTDOWN, EV_KLIPPY_DISCONNECT,
};
use klippy_api::webhooks::{
    ClientConnectionApi, WebHooksApi, WebHooksCallback, WebRequestApi,
};
use pycompat::text::{percent_format, PctValue};
use pycompat::value::py_equal;
use pycompat::{json, PyInt, PyValue};
use reactor::{Completion, FdHandler, Reactor, Timer, NEVER, NOW};

/// `REQUEST_LOG_SIZE` (`klippy/webhooks.py:39`).
pub const REQUEST_LOG_SIZE: usize = 20;
/// `SUBSCRIPTION_REFRESH_TIME` (`klippy/webhooks.py:479`).
pub const SUBSCRIPTION_REFRESH_TIME: f64 = 0.25;

const EBADF: i32 = 9;
const EAGAIN: i32 = 11;

// ---------------------------------------------------------------------------
// Small `PyValue` / formatting helpers
// ---------------------------------------------------------------------------

fn py_int(v: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(v))
}

fn py_str(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}

fn py_dict(pairs: Vec<(PyValue, PyValue)>) -> PyValue {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k, v);
    }
    PyValue::Dict(m)
}

fn empty_dict() -> PyValue {
    PyValue::Dict(IndexMap::new())
}

/// `fmt_msg` equivalent: format a static Python `%` template, which
/// cannot fail for the literal templates used below.
fn fmt_msg(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|e| panic!("webhooks format {fmt:?}: {}", e.display()))
}

/// `printer.command_error(msg)`.
fn cmd_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Command(msg.into())
}

/// `printer.config_error(msg)`.
fn config_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Config(msg.into())
}

/// `WebRequestError(msg)` (`klippy/webhooks.py:41`, a `CommandError`
/// subclass, carried as `Other` per the `klippy_api` contract).
pub(crate) fn web_request_error(msg: impl Into<String>) -> KlippyError {
    KlippyError::Other {
        class_name: "webhooks.WebRequestError".to_string(),
        message: msg.into(),
    }
}

/// Whether `err` is caught by `except printer.command_error` in
/// `_process_request` (`klippy/webhooks.py:269`), which includes the
/// `WebRequestError` subclass.
fn is_command_error(err: &KlippyError) -> bool {
    match err {
        KlippyError::Command(_) => true,
        KlippyError::Other { .. } | KlippyError::Internal { .. } => matches!(
            err.qualified_class_name(),
            "webhooks.WebRequestError" | "gcode.CommandError"
        ),
        _ => false,
    }
}

/// Dict lookup by string key (JSON object keys are strings).
fn dict_get_str<'a>(m: &'a IndexMap<PyValue, PyValue>, key: &str) -> Option<&'a PyValue> {
    m.get(&PyValue::Str(key.to_string()))
}

/// `dict.get(key, default)` for a `PyValue` that must be a dict.
fn py_dict_get(v: &PyValue, key: &str) -> Option<PyValue> {
    match v {
        PyValue::Dict(m) => dict_get_str(m, key).cloned(),
        _ => None,
    }
}

/// Whether a `PyValue` is falsy (`bool(v)`), for the
/// `not server_address` check (`klippy/webhooks.py:126`).
fn is_falsy(v: &PyValue) -> bool {
    !v.truthy()
}

/// `pycompat` error -> log text. Python logs `str(e)`, which for
/// `TypeError`/`ValueError` is the bare message, not `kind: msg`.
fn json_error_text(e: &pycompat::PyError) -> String {
    e.msg.clone()
}

/// `logging.exception(msg)` traceback text: the port has no Python
/// traceback; the normalizer table covers traceback bodies.
fn exc_text(err: &KlippyError) -> String {
    err.exception_line()
}

/// `UnicodeDecodeError.__str__` (`Objects/exceptions.c`).
fn unicode_decode_error(
    codec: &str,
    data: &[u8],
    start: usize,
    end: usize,
    reason: &str,
) -> KlippyError {
    let message = if end == start + 1 {
        format!(
            "'{codec}' codec can't decode byte 0x{:02x} in position {start}: {reason}",
            data[start]
        )
    } else {
        format!(
            "'{codec}' codec can't decode bytes in position {start}-{}: {reason}",
            end - 1
        )
    };
    KlippyError::Other {
        class_name: "UnicodeDecodeError".to_string(),
        message,
    }
}

/// `data.decode('utf-8')`. Python decodes with `surrogatepass`, which
/// accepts encoded surrogates (`ED A0..BF xx`) into a `str` holding a
/// lone surrogate; a Rust `String` cannot, so those stay an error here.
fn decode_utf8(data: &[u8]) -> KResult<String> {
    match std::str::from_utf8(data) {
        Ok(s) => Ok(s.to_string()),
        Err(e) => {
            let start = e.valid_up_to();
            let first = data[start];
            match e.error_len() {
                None => Err(unicode_decode_error(
                    "utf-8",
                    data,
                    start,
                    data.len(),
                    "unexpected end of data",
                )),
                Some(_) if !(0xC2..=0xF4).contains(&first) => Err(unicode_decode_error(
                    "utf-8",
                    data,
                    start,
                    start + 1,
                    "invalid start byte",
                )),
                Some(n) => Err(unicode_decode_error(
                    "utf-8",
                    data,
                    start,
                    start + n,
                    "invalid continuation byte",
                )),
            }
        }
    }
}

/// `data.decode('utf-16' | 'utf-16-le' | 'utf-16-be')`; `bom` selects
/// the `utf-16` form that consumes a leading byte order mark.
fn decode_utf16(data: &[u8], bom: bool, big_endian: bool) -> KResult<String> {
    let mut pos = 0usize;
    let mut big = big_endian;
    if bom && data.len() >= 2 {
        match (data[0], data[1]) {
            (0xFF, 0xFE) => {
                pos = 2;
                big = false;
            }
            (0xFE, 0xFF) => {
                pos = 2;
                big = true;
            }
            _ => {}
        }
    }
    let unit = |p: usize| -> u16 {
        if big {
            u16::from_be_bytes([data[p], data[p + 1]])
        } else {
            u16::from_le_bytes([data[p], data[p + 1]])
        }
    };
    // CPython names the byte order the decoder settled on in its errors.
    let codec = if big { "utf-16-be" } else { "utf-16-le" };
    let mut out = String::new();
    while pos < data.len() {
        if data.len() - pos < 2 {
            return Err(unicode_decode_error(
                codec,
                data,
                pos,
                data.len(),
                "truncated data",
            ));
        }
        let ch = unit(pos);
        match ch & 0xFC00 {
            0xD800 => {
                if pos + 2 >= data.len() {
                    return Err(unicode_decode_error(
                        codec,
                        data,
                        pos,
                        data.len(),
                        "unexpected end of data",
                    ));
                }
                let ch2 = unit(pos + 2);
                if ch2 & 0xFC00 != 0xDC00 {
                    return Err(unicode_decode_error(
                        codec,
                        data,
                        pos,
                        pos + 2,
                        "illegal UTF-16 surrogate",
                    ));
                }
                let cp = 0x10000 + (((ch as u32) & 0x3FF) << 10) + ((ch2 as u32) & 0x3FF);
                out.push(char::from_u32(cp).expect("valid surrogate pair"));
                pos += 4;
            }
            0xDC00 => {
                return Err(unicode_decode_error(
                    codec,
                    data,
                    pos,
                    pos + 2,
                    "illegal encoding",
                ));
            }
            _ => {
                out.push(char::from_u32(ch as u32).expect("non-surrogate unit"));
                pos += 2;
            }
        }
    }
    Ok(out)
}

/// `data.decode('utf-32' | 'utf-32-le' | 'utf-32-be')`.
fn decode_utf32(data: &[u8], bom: bool, big_endian: bool) -> KResult<String> {
    let mut pos = 0usize;
    let mut big = big_endian;
    if bom && data.len() >= 4 {
        match (data[0], data[1], data[2], data[3]) {
            (0xFF, 0xFE, 0x00, 0x00) => {
                pos = 4;
                big = false;
            }
            (0x00, 0x00, 0xFE, 0xFF) => {
                pos = 4;
                big = true;
            }
            _ => {}
        }
    }
    let codec = if big { "utf-32-be" } else { "utf-32-le" };
    let mut out = String::new();
    while pos < data.len() {
        if data.len() - pos < 4 {
            return Err(unicode_decode_error(
                codec,
                data,
                pos,
                data.len(),
                "truncated data",
            ));
        }
        let raw = [data[pos], data[pos + 1], data[pos + 2], data[pos + 3]];
        let cp = if big {
            u32::from_be_bytes(raw)
        } else {
            u32::from_le_bytes(raw)
        };
        match char::from_u32(cp) {
            Some(c) => out.push(c),
            None => {
                let reason = if cp > 0x10FFFF {
                    "code point not in range(0x110000)"
                } else {
                    "code point in surrogate code point range(0xd800, 0xe000)"
                };
                return Err(unicode_decode_error(codec, data, pos, pos + 4, reason));
            }
        }
        pos += 4;
    }
    Ok(out)
}

/// `s.decode(json.detect_encoding(s), 'surrogatepass')`: what
/// `json.loads` does with a `bytes` argument.
fn decode_json_bytes(b: &[u8]) -> KResult<String> {
    if b.starts_with(&[0xFF, 0xFE, 0x00, 0x00]) || b.starts_with(&[0x00, 0x00, 0xFE, 0xFF]) {
        return decode_utf32(b, true, false);
    }
    if b.starts_with(&[0xFF, 0xFE]) || b.starts_with(&[0xFE, 0xFF]) {
        return decode_utf16(b, true, false);
    }
    if b.starts_with(&[0xEF, 0xBB, 0xBF]) {
        return decode_utf8(&b[3..]);
    }
    if b.len() >= 4 {
        if b[0] == 0 {
            // 00 00 -- -- is utf-32-be, 00 XX -- -- is utf-16-be.
            return if b[1] != 0 {
                decode_utf16(b, false, true)
            } else {
                decode_utf32(b, false, true)
            };
        }
        if b[1] == 0 {
            // XX 00 00 00 is utf-32-le, XX 00 00 XX and XX 00 XX -- are utf-16-le.
            return if b[2] != 0 || b[3] != 0 {
                decode_utf16(b, false, false)
            } else {
                decode_utf32(b, false, false)
            };
        }
    } else if b.len() == 2 {
        if b[0] == 0 {
            return decode_utf16(b, false, true);
        }
        if b[1] == 0 {
            return decode_utf16(b, false, false);
        }
    }
    decode_utf8(b)
}

/// The rest of `json.loads(request)` after [`decode_json_bytes`]. CPython's
/// "Unexpected UTF-8 BOM" check only runs for a `str` argument; the request
/// is `bytes`, so a U+FEFF left after decoding (a second BOM) just fails to
/// parse with "Expecting value".
fn loads_text(text: &str) -> KResult<PyValue> {
    json::loads_decoded(text).map_err(|e| KlippyError::Other {
        class_name: "json.decoder.JSONDecodeError".to_string(),
        message: e.msg,
    })
}

/// `OSError` raised by a failed system call, with the subclass and the
/// `[Errno N] strerror: 'filename'` text CPython builds.
fn os_error(e: &std::io::Error, filename: Option<&str>) -> KlippyError {
    let Some(errno) = e.raw_os_error() else {
        return KlippyError::Internal {
            class_name: "OSError".to_string(),
            message: e.to_string(),
        };
    };
    let text = e.to_string();
    let strerror = text
        .strip_suffix(&format!(" (os error {errno})"))
        .unwrap_or(&text)
        .to_string();
    let class_name = match errno {
        1 | 13 => "PermissionError",
        2 => "FileNotFoundError",
        3 => "ProcessLookupError",
        4 => "InterruptedError",
        10 => "ChildProcessError",
        11 | 114 | 115 => "BlockingIOError",
        17 => "FileExistsError",
        20 => "NotADirectoryError",
        21 => "IsADirectoryError",
        32 | 108 => "BrokenPipeError",
        103 => "ConnectionAbortedError",
        104 => "ConnectionResetError",
        110 => "TimeoutError",
        111 => "ConnectionRefusedError",
        _ => "OSError",
    };
    let message = match filename {
        Some(f) => format!("[Errno {errno}] {strerror}: {}", pycompat::text::str_repr(f)),
        None => format!("[Errno {errno}] {strerror}"),
    };
    KlippyError::Internal {
        class_name: class_name.to_string(),
        message,
    }
}

// ---------------------------------------------------------------------------
// Host information (`info` endpoint: `os`/`socket`/`sys` at request time)
// ---------------------------------------------------------------------------

/// Host values shown by the `info` endpoint (`klippy/webhooks.py:377-386`),
/// captured once at `WebHooks` creation. The Python code reads them at
/// request time, but ids and the executable never change during a run.
#[derive(Clone, Debug)]
pub struct HostInfo {
    /// `socket.gethostname()`.
    pub hostname: String,
    /// `normpath(join(dirname(__file__), ".."))`.
    pub klipper_path: String,
    /// `sys.executable`.
    pub python_path: String,
    /// `os.getpid()`.
    pub process_id: i64,
    /// `os.getuid()`.
    pub user_id: i64,
    /// `os.getgid()`.
    pub group_id: i64,
}

extern "C" {
    fn getpid() -> i32;
    fn getuid() -> u32;
    fn getgid() -> u32;
    fn gethostname(name: *mut u8, len: usize) -> i32;
}

impl HostInfo {
    /// Values of the running process. `klipper_path` is
    /// `normpath(join(dirname(__file__), ".."))` with `webhooks.py`'s
    /// directory, the recorded klippy source directory; the Python oracle
    /// injects its own fakes for all fields (see the unit tests).
    pub fn current() -> HostInfo {
        HostInfo {
            hostname: read_hostname(),
            klipper_path: klipper_path_from_dir(&klippy_api::paths::klippy_dir()),
            python_path: std::env::current_exe()
                .map(|p| p.to_string_lossy().into_owned())
                .unwrap_or_default(),
            process_id: unsafe { getpid() } as i64,
            user_id: unsafe { getuid() } as i64,
            group_id: unsafe { getgid() } as i64,
        }
    }
}

fn read_hostname() -> String {
    let mut buf = vec![0u8; 256];
    let rc = unsafe { gethostname(buf.as_mut_ptr(), buf.len()) };
    if rc != 0 {
        return String::new();
    }
    let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    String::from_utf8_lossy(&buf[..len]).into_owned()
}

/// Lexical `normpath(join(dir, ".."))` in the `os.path` sense, without
/// touching the filesystem.
fn klipper_path_from_dir(dir: &str) -> String {
    let joined = if dir.is_empty() {
        "..".to_string()
    } else if dir.ends_with('/') {
        format!("{dir}..")
    } else {
        format!("{dir}/..")
    };
    normpath(&joined)
}

/// POSIX `normpath` for the simple relative/absolute shapes involved
/// here (`.` / `..` segments, single leading slash).
fn normpath(path: &str) -> String {
    let absolute = path.starts_with('/');
    let mut out: Vec<&str> = Vec::new();
    for part in path.split('/') {
        if part.is_empty() || part == "." {
            continue;
        }
        if part == ".." {
            if matches!(out.last(), Some(&"..")) || out.is_empty() && !absolute {
                out.push("..");
            } else if !out.is_empty() {
                out.pop();
            }
        } else {
            out.push(part);
        }
    }
    let mut s = out.join("/");
    if absolute {
        s = format!("/{s}");
    }
    if s.is_empty() {
        if absolute {
            "/".to_string()
        } else {
            ".".to_string()
        }
    } else {
        s
    }
}

/// `os.remove(file_path)` (`klippy/webhooks.py:164-172`): a missing
/// file is ignored, a file that still exists is a logged raise.
fn remove_socket_file(file_path: &str) -> KResult<()> {
    match std::fs::remove_file(file_path) {
        Ok(()) => Ok(()),
        Err(e) => {
            if Path::new(file_path).exists() {
                let msg = fmt_msg(
                    "webhooks: Unable to delete socket file '%s'",
                    &[PctValue::Str(file_path.to_string())],
                );
                let err = os_error(&e, Some(file_path));
                log::exception(&msg, &exc_text(&err));
                Err(err)
            } else {
                Ok(())
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Raw socket seam (real fd sockets and recording fakes)
// ---------------------------------------------------------------------------
/// The subset of `socket.socket` used here (`klippy/webhooks.py:130-148`,
/// 222-313): a real fd socket or a recording fake in tests.
pub trait RawSocket: Send + Sync + 'static {
    /// `sock.fileno()`.
    fn fileno(&self) -> i32;
    /// `sock.recv(4096)`: the whole chunk, or the socket errno.
    fn recv(&self, max: usize) -> Result<Vec<u8>, i32>;

    /// `sock.send(buf)`.
    fn send(&self, data: &[u8]) -> Result<usize, i32>;
    /// `sock.close()`.
    fn close(&self);
}

/// A `socket.socket(AF_UNIX, SOCK_STREAM)` kept as a raw fd.
pub struct FdSocket {
    fd: i32,
}

impl FdSocket {
    /// Wrap an already-created (and nonblocking) socket fd.
    pub fn new(fd: i32) -> FdSocket {
        FdSocket { fd }
    }
}

impl RawSocket for FdSocket {
    fn fileno(&self) -> i32 {
        self.fd
    }
    fn recv(&self, max: usize) -> Result<Vec<u8>, i32> {
        let mut buf = vec![0u8; max];
        match kos::fd::read(self.fd, &mut buf) {
            Ok(n) => {
                buf.truncate(n);
                Ok(buf)
            }
            Err(e) => Err(e.raw_os_error().unwrap_or(0)),
        }
    }
    fn send(&self, data: &[u8]) -> Result<usize, i32> {
        match kos::fd::write(self.fd, data) {
            Ok(n) => Ok(n),
            Err(e) => Err(e.raw_os_error().unwrap_or(0)),
        }
    }
    fn close(&self) {
        kos::fd::close(self.fd);
    }
}

/// Handle to a client connection (`Arc<dyn ClientConnectionApi>`).
pub type ClientConnHandle = Arc<dyn ClientConnectionApi>;
/// Handle to a decoded request (`Arc<dyn WebRequestApi>`).
pub type WebRequestHandle = Arc<dyn WebRequestApi>;

// ---------------------------------------------------------------------------
// `WebRequest` (`klippy/webhooks.py:53-115`)
// ---------------------------------------------------------------------------

/// A decoded webhooks request (`klippy/webhooks.py:53`).
pub struct WebRequest {
    client_conn: ClientConnHandle,
    id: Option<PyValue>,
    method: String,
    params: PyValue,
    response: GCell<Option<PyValue>>,
    is_error: GCell<bool>,
}

impl WebRequest {
    /// WebRequest(client_conn, request) (`klippy/webhooks.py:55-66`).
    ///
    /// Raises `ValueError("Not a top-level dictionary")` /
    /// `ValueError("Invalid request type")` for non-dict requests or
    /// non-`str` method / non-dict params, and `JSONDecodeError` for
    /// malformed JSON (the `json` branch of `webhooks.py:31-34`).
    #[allow(clippy::new_ret_no_self)]
    pub fn new(client_conn: ClientConnHandle, request: &[u8]) -> KResult<WebRequestHandle> {
        let text = decode_json_bytes(request)?;
        let base = loads_text(&text)?;
        let PyValue::Dict(base) = base else {
            return Err(KlippyError::Other {
                class_name: "ValueError".to_string(),
                message: "Not a top-level dictionary".to_string(),
            });
        };
        let id = dict_get_str(&base, "id").cloned().filter(|v| !matches!(v, PyValue::None));
        let method = dict_get_str(&base, "method").cloned();
        let params = dict_get_str(&base, "params").cloned().unwrap_or_else(empty_dict);
        let method_ok = matches!(method, Some(PyValue::Str(_)));
        let params_ok = matches!(params, PyValue::Dict(_));
        if !method_ok || !params_ok {
            return Err(KlippyError::Other {
                class_name: "ValueError".to_string(),
                message: "Invalid request type".to_string(),
            });
        }
        let method = match method {
            Some(PyValue::Str(s)) => s,
            _ => unreachable!("checked"),
        };
        Ok(Arc::new(WebRequest {
            client_conn,
            id,
            method,
            params,
            response: GCell::new(None),
            is_error: GCell::new(false),
        }))
    }

    /// Shared `get(item, default, types)` body (`klippy/webhooks.py:71-78`).
    /// `default=None` (outer) is the `Sentinel` (required); a present
    /// item is strictly type-checked, a missing one skips the check.
    fn get_checked(
        &self,
        item: &str,
        default: Option<PyValue>,
        valid: impl Fn(&PyValue) -> bool,
    ) -> KResult<PyValue> {
        let map = match &self.params {
            PyValue::Dict(m) => m,
            _ => return Ok(self.params.clone()),
        };
        match dict_get_str(map, item) {
            Some(value) => {
                if !valid(value) {
                    let msg = fmt_msg("Invalid Argument Type [%s]", &[PctValue::Str(item.to_string())]);
                    return Err(web_request_error(msg));
                }
                Ok(value.clone())
            }
            None => match default {
                Some(v) => Ok(v),
                None => {
                    let msg = fmt_msg("Missing Argument [%s]", &[PctValue::Str(item.to_string())]);
                    Err(web_request_error(msg))
                }
            },
        }
    }
}

impl WebRequestApi for WebRequest {
    fn client_conn(&self) -> ClientConnHandle {
        self.client_conn.clone()
    }
    fn id(&self) -> Option<PyValue> {
        self.id.clone()
    }
    fn method(&self) -> String {
        self.method.clone()
    }
    fn params(&self) -> PyValue {
        self.params.clone()
    }
    fn response(&self) -> Option<PyValue> {
        self.response.borrow().clone()
    }
    fn is_error(&self) -> bool {
        *self.is_error.borrow()
    }
    fn get_client_connection(&self) -> ClientConnHandle {
        self.client_conn.clone()
    }
    fn get(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        self.get_checked(item, default, |_| true)
    }
    fn get_str(&self, item: &str, default: Option<String>) -> KResult<String> {
        let default = default.map(PyValue::Str);
        match self.get_checked(item, default, |v| matches!(v, PyValue::Str(_)))? {
            PyValue::Str(s) => Ok(s),
            // A non-str default cannot occur (typed default above);
            // a present non-str was rejected by the type check.
            other => Ok(other.str()),
        }
    }
    fn get_int(&self, item: &str, default: Option<i64>) -> KResult<i64> {
        let default = default.map(py_int);
        match self.get_checked(item, default, |v| matches!(v, PyValue::Int(_)))? {
            PyValue::Int(n) => n
                .inner()
                .to_string()
                .parse::<i64>()
                .map_err(|_| KlippyError::Internal {
                    class_name: "OverflowError".to_string(),
                    message: "int too large to convert to i64".to_string(),
                }),
            other => Ok(other
                .str()
                .parse::<i64>()
                .unwrap_or(0)),
        }
    }
    fn get_float(&self, item: &str, default: Option<f64>) -> KResult<f64> {
        let default = default.map(PyValue::Float);
        match self.get_checked(item, default, |v| {
            matches!(v, PyValue::Int(_) | PyValue::Float(_))
        })? {
            PyValue::Float(f) => Ok(f),
            PyValue::Int(n) => match n.to_f64_exact() {
                Some(f) if f.is_finite() => Ok(f),
                _ => Err(KlippyError::Internal {
                    class_name: "OverflowError".to_string(),
                    message: "int too large to convert to float".to_string(),
                }),
            },
            other => other.str().parse::<f64>().map_err(|_| KlippyError::Internal {
                class_name: "ValueError".to_string(),
                message: format!("could not convert string to float: {:?}", other.str()),
            }),
        }
    }
    fn get_dict(&self, item: &str, default: Option<PyValue>) -> KResult<PyValue> {
        self.get_checked(item, default, |v| matches!(v, PyValue::Dict(_)))
    }
    fn get_method(&self) -> String {
        self.method.clone()
    }
    fn set_error(&self, error: KlippyError) {
        self.is_error.set(true);
        self.response.replace(Some(py_dict(vec![
            (py_str("error"), py_str("WebRequestError")),
            (py_str("message"), py_str(error.message())),
        ])));
    }
    fn send(&self, data: PyValue) -> KResult<()> {
        if self.response.borrow().is_some() {
            return Err(web_request_error("Multiple calls to send not allowed"));
        }
        self.response.replace(Some(data));
        Ok(())
    }
    fn finish(&self) -> Option<PyValue> {
        let id = self.id.clone()?;
        let rtype = if *self.is_error.borrow() { "error" } else { "result" };
        if self.response.borrow().is_none() {
            // No error was set and the user never executed send: the
            // default response is `{}`, stored like Python does so a
            // later `send` is a "Multiple calls" error.
            self.response.replace(Some(empty_dict()));
        }
        let response = self.response.borrow().clone().expect("set above");
        Some(py_dict(vec![(py_str("id"), id), (py_str(rtype), response)]))
    }
}

// ---------------------------------------------------------------------------
// `ClientConnection` (`klippy/webhooks.py:187-314`)
// ---------------------------------------------------------------------------

/// A webhooks client connection (`klippy/webhooks.py:187`).
pub struct ClientConnection {
    printer: Printer,
    webhooks: Arc<WebHooks>,
    reactor: Reactor,
    server: Arc<ServerSocket>,
    uid: GCell<u64>,
    sock: GCell<Option<Arc<dyn RawSocket>>>,
    fd_handle: GCell<Option<FdHandler>>,
    partial_data: GCell<Vec<u8>>,
    send_buffer: GCell<Vec<u8>>,
    is_blocking: GCell<bool>,
    blocking_count: GCell<i64>,
    request_log: GCell<VecDeque<(f64, Vec<u8>)>>,
}

impl ClientConnection {
    /// ClientConnection(server, sock) (`klippy/webhooks.py:188-201`):
    /// the socket is set nonblocking and registered for both read and
    /// write readiness, then `set_client_info("?", "New connection")`.
    pub fn new(server: &Arc<ServerSocket>, sock: Arc<dyn RawSocket>) -> Arc<ClientConnection> {
        let conn = Arc::new(ClientConnection {
            printer: server.printer(),
            webhooks: server.webhooks(),
            reactor: server.reactor(),
            server: server.clone(),
            uid: GCell::new(0),
            sock: GCell::new(Some(sock)),
            fd_handle: GCell::new(None),
            partial_data: GCell::new(Vec::new()),
            send_buffer: GCell::new(Vec::new()),
            is_blocking: GCell::new(false),
            blocking_count: GCell::new(0),
            request_log: GCell::new(VecDeque::new()),
        });
        conn.uid.set(Arc::as_ptr(&conn) as usize as u64);
        ClientConnection::register_fds(&conn);
        conn.set_client_info(py_str("?"), Some("New connection".to_string()));
        conn
    }

    fn register_fds(conn: &Arc<ClientConnection>) {
        let reactor = conn.reactor.clone();
        let read_conn = conn.clone();
        let write_conn = conn.clone();
        let fd = conn.sock.borrow().as_ref().map(|s| s.fileno()).unwrap_or(-1);
        let handler = reactor.register_fd(
            fd,
            move |eventtime: f64| {
                read_conn.process_received(eventtime);
            },
            Some(move |eventtime: f64| {
                write_conn.do_send(Some(eventtime));
            }),
        );
        conn.fd_handle.replace(Some(handler));
    }

    fn uid(&self) -> u64 {
        self.uid.get()
    }


    /// `send_buffer` snapshot (tests).
    pub fn send_buffer(&self) -> Vec<u8> {
        self.send_buffer.borrow().clone()
    }
    /// `partial_data` snapshot (tests).
    pub fn partial_data(&self) -> Vec<u8> {
        self.partial_data.borrow().clone()
    }
    /// `is_blocking` state (tests).
    pub fn is_blocking(&self) -> bool {
        *self.is_blocking.borrow()
    }
    /// `blocking_count` state (tests).
    pub fn blocking_count(&self) -> i64 {
        self.blocking_count.get()
    }
    /// Sets `blocking_count` (used by `ServerSocket::stats`).
    pub fn set_blocking_count(&self, value: i64) {
        self.blocking_count.set(value);
    }
    /// Sets `is_blocking` (used by `ServerSocket::stats` tests).
    pub fn set_blocking(&self, value: bool) {
        self.is_blocking.set(value);
    }
    /// `request_log` snapshot: `(eventtime, request)` pairs (tests).
    pub fn request_log(&self) -> Vec<(f64, Vec<u8>)> {
        self.request_log.borrow().iter().cloned().collect()
    }

    /// `_process_request` (`klippy/webhooks.py:265-280`), dispatched by
    /// the reactor callback in `process_received`; also the single
    /// funnel tests drive directly (the Python `_process_request`).
    pub fn process_request(self: &Arc<Self>, web_request: WebRequestHandle) {
        let func = match self.webhooks.get_callback(&web_request.get_method()) {
            Ok(cb) => Some(cb),
            Err(e) => {
                self.error_out(web_request, e);
                return;
            }
        };
        let func = func.unwrap();
        let res = func(web_request.clone());
        match res {
            Ok(()) => {}
            Err(e) if is_command_error(&e) => {
                web_request.set_error(e);
            }
            Err(e) => {
                let msg = fmt_msg(
                    "Internal Error on WebRequest: %s",
                    &[PctValue::Str(web_request.get_method())],
                );
                log::exception(&msg, &exc_text(&e));
                web_request.set_error(e);
                self.printer.invoke_shutdown(&msg, empty_dict());
            }
        }
        self.finish_and_send(web_request);
    }

    fn error_out(&self, web_request: WebRequestHandle, e: KlippyError) {
        web_request.set_error(e);
        self.finish_and_send(web_request);
    }

    fn finish_and_send(&self, web_request: WebRequestHandle) {
        let Some(result) = web_request.finish() else {
            return;
        };
        self.send(result);
    }

    /// `process_received` (`klippy/webhooks.py:237-263`).
    pub fn process_received(self: &Arc<Self>, eventtime: f64) {
        let data = match self.sock.borrow().as_ref().map(|s| s.recv(4096)) {
            Some(Ok(v)) => v,
            Some(Err(EBADF)) => Vec::new(),
            Some(Err(_)) => return,
            None => Vec::new(),
        };
        if data.is_empty() {
            // Socket Closed.
            self.close();
            return;
        }
        let mut parts: Vec<Vec<u8>> = data
            .split(|&b| b == b'\x03')
            .map(|s| s.to_vec())
            .collect();
        {
            let mut first = self.partial_data.replace(Vec::new());
            first.extend_from_slice(&parts[0]);
            parts[0] = first;
        }
        let partial = parts.pop().unwrap_or_default();
        self.partial_data.replace(partial);
        let handle: ClientConnHandle = self.clone();
        let conn = self.clone();
        let reactor = self.reactor.clone();
        for req in parts {
            {
                let mut log = self.request_log.borrow_mut();
                log.push_back((eventtime, req.clone()));
                while log.len() > REQUEST_LOG_SIZE {
                    log.pop_front();
                }
            }
            let web_req = match WebRequest::new(handle.clone(), &req) {
                Ok(wr) => wr,
                Err(e) => {
                    let msg = fmt_msg(
                        "webhooks: Error decoding Server Request %s",
                        &[PctValue::Bytes(req)],
                    );
                    log::exception(&msg, &exc_text(&e));
                    continue;
                }
            };
            let conn2 = conn.clone();
            let web_req = web_req.clone();
            reactor.register_callback(
                move |_: f64| {
                    conn2.process_request(web_req.clone());
                },
                NOW,
            );
        }
    }

    /// `_do_send` (`klippy/webhooks.py:294-313`).
    pub fn do_send(&self, _eventtime: Option<f64>) {
        if self.fd_handle.borrow().is_none() {
            return;
        }
        let sent = {
            let buf = self.send_buffer.borrow().clone();
            match self.sock.borrow().as_ref().map(|s| s.send(&buf)) {
                Some(Err(e)) if e == EAGAIN => 0,
                Some(Err(_)) => {
                    let msg = fmt_msg(
                        "webhooks: socket write error %d",
                        &[PctValue::Int(PyInt::from_u64(self.uid()))],
                    );
                    log::info(&msg);
                    self.close();
                    return;
                }
                Some(Ok(n)) => n,
                None => 0,
            }
        };
        let len = self.send_buffer.borrow().len();
        if sent < len {
            if !self.is_blocking.get() {
                if let Some(h) = self.fd_handle.borrow().as_ref() {
                    self.reactor.set_fd_wake(h, false, true);
                }
                self.is_blocking.set(true);
                self.blocking_count.set(5);
            }
        } else if self.is_blocking.get() {
            if let Some(h) = self.fd_handle.borrow().as_ref() {
                self.reactor.set_fd_wake(h, true, false);
            }
            self.is_blocking.set(false);
        }
        {
            let mut buf = self.send_buffer.borrow_mut();
            if sent < buf.len() {
                buf.drain(..sent);
            } else {
                buf.clear();
            }
        }
    }
}

impl ClientConnection {
    fn set_client_info_impl(&self, client_info: PyValue, state_msg: Option<String>) {
        let state_msg = match state_msg {
            Some(m) => m,
            None => {
                let rep = PctValue::Str(client_info.repr());
                fmt_msg("Client info %s", &[rep])
            }
        };
        let msg = fmt_msg(
            "webhooks client %s: %s",
            &[
                PctValue::Int(PyInt::from_u64(self.uid())),
                PctValue::Str(state_msg),
            ],
        );
        log::info(&msg);
        let log_id = fmt_msg("webhooks %s", &[PctValue::Int(PyInt::from_u64(self.uid()))]);
        if matches!(client_info, PyValue::None) {
            self.printer.set_rollover_info(&log_id, None, false);
            return;
        }
        let rollover_msg = fmt_msg(
            "webhooks client %s: %s",
            &[
                PctValue::Int(PyInt::from_u64(self.uid())),
                PctValue::Str(client_info.repr()),
            ],
        );
        self.printer.set_rollover_info(&log_id, Some(&rollover_msg), false);
    }
}

impl ClientConnectionApi for ClientConnection {
    fn uid(&self) -> u64 {
        self.uid()
    }
    fn dump_request_log(&self) {
        let mut out: Vec<String> = vec![fmt_msg(
            "Dumping %d requests for client %d",
            &[
                PctValue::Int(PyInt::from_i64(
                    self.request_log.borrow().len() as i64
                )),
                PctValue::Int(PyInt::from_u64(self.uid())),
            ],
        )];
        for (eventtime, request) in self.request_log.borrow().iter() {
            out.push(fmt_msg(
                "Received %f: %s",
                &[PctValue::Float(*eventtime), PctValue::Bytes(request.clone())],
            ));
        }
        log::info(&out.join("\n"));
    }
    fn set_client_info(&self, client_info: PyValue, state_msg: Option<String>) {
        self.set_client_info_impl(client_info, state_msg);
    }
    fn close(&self) {
        if self.fd_handle.borrow().is_none() {
            return;
        }
        self.set_client_info(PyValue::None, Some("Disconnected".to_string()));
        if let Some(h) = self.fd_handle.replace(None) {
            self.reactor.unregister_fd(&h);
        }
        if let Some(sock) = self.sock.borrow().as_ref() {
            sock.close();
        }
        self.server.pop_client(self.uid());
    }
    fn is_closed(&self) -> bool {
        self.fd_handle.borrow().is_none()
    }
    fn send(&self, data: PyValue) {
        match json::dumps_compact(&data) {
            Ok(text) => {
                let mut jmsg = text.into_bytes();
                jmsg.push(b'\x03');
                self.send_buffer.borrow_mut().extend_from_slice(&jmsg);
            }
            Err(e) => {
                let msg = fmt_msg("json encoding error: %s", &[PctValue::Str(json_error_text(&e))]);
                log::exception(&msg, &e.display());
                self.printer.invoke_shutdown(&msg, empty_dict());
                return;
            }
        }
        if !self.is_blocking.get() {
            self.do_send(None);
        }
    }
}

// ---------------------------------------------------------------------------
// `ServerSocket` (`klippy/webhooks.py:116-186`)
// ---------------------------------------------------------------------------

/// The unix-socket listener (`klippy/webhooks.py:116`).
pub struct ServerSocket {
    printer: Printer,
    webhooks: Arc<WebHooks>,
    reactor: Reactor,
    sock: GCell<Option<i32>>,
    fd_handle: GCell<Option<FdHandler>>,
    clients: GCell<IndexMap<u64, Arc<ClientConnection>>>,
}

impl ServerSocket {
    /// ServerSocket(webhooks, printer) (`klippy/webhooks.py:117-139`).
    /// Returned early (disabled) when there is no `apiserver` start
    /// argument or `debuginput` is set (file-input mode disables the
    /// server, even before event registration).
    pub fn new(webhooks: Arc<WebHooks>, printer: &Printer) -> KResult<Arc<ServerSocket>> {
        let reactor = printer.get_reactor();
        let start_args = printer.get_start_args();
        let start_map = match &start_args {
            PyValue::Dict(m) => m.clone(),
            other => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".to_string(),
                    message: format!(
                        "'{}' object has no attribute 'get'",
                        pycompat::value::type_name(other)
                    ),
                });
            }
        };
        let server_address = dict_get_str(&start_map, "apiserver")
            .filter(|v| !is_falsy(v))
            .cloned();
        let is_fileinput = dict_get_str(&start_map, "debuginput")
            .map(|v| !matches!(v, PyValue::None))
            .unwrap_or(false);
        let server = Arc::new(ServerSocket {
            printer: printer.clone(),
            webhooks: webhooks.clone(),
            reactor: reactor.clone(),
            sock: GCell::new(None),
            fd_handle: GCell::new(None),
            clients: GCell::new(IndexMap::new()),
        });
        if server_address.is_none() || is_fileinput {
            return Ok(server);
        }
        let addr = match server_address.unwrap() {
            PyValue::Str(s) => s,
            // Non-string truthy `apiserver` values fail in
            // `os.remove` with a `TypeError` in Python.
            other => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: format!(
                        "remove: path should be string, bytes or os.PathLike, not {}",
                        pycompat::value::type_name(&other)
                    ),
                });
            }
        };
        remove_socket_file(&addr)?;
        let fd = kos::socket::unix_bind(&addr).map_err(|e| os_error(&e, None))?;
        kos::fd::set_nonblock(fd).map_err(|e| os_error(&e, None))?;
        kos::socket::listen(fd, 1).map_err(|e| os_error(&e, None))?;
        server.sock.set(Some(fd));
        let server_r = server.clone();
        let handler = reactor.register_fd(
            fd,
            move |eventtime: f64| {
                server_r.handle_accept(eventtime);
            },
            None::<fn(f64)>,
        );
        server.fd_handle.replace(Some(handler));
        let disc = server.clone();
        printer.register_event_handler(
            EV_KLIPPY_DISCONNECT,
            Box::new(move |_args: &[EventArg]| {
                disc.handle_disconnect();
                Ok(PyValue::None)
            }),
        );
        let ana = server.clone();
        printer.register_event_handler(
            EV_KLIPPY_ANALYZE_SHUTDOWN,
            Box::new(move |args: &[EventArg]| {
                let (msg, details) = analyze_shutdown_args(args);
                ana.handle_analyze_shutdown(&msg, &details);
                Ok(PyValue::None)
            }),
        );
        Ok(server)
    }

    /// Printer handle (for `ClientConnection::new`).
    pub fn printer(&self) -> Printer {
        self.printer.clone()
    }
    /// `webhooks` handle (for `ClientConnection::new`).
    pub fn webhooks(&self) -> Arc<WebHooks> {
        self.webhooks.clone()
    }
    /// Reactor handle.
    pub fn reactor(&self) -> Reactor {
        self.reactor.clone()
    }

    /// `_handle_accept` (`klippy/webhooks.py:141-148`).
    pub fn handle_accept(self: &Arc<Self>, _eventtime: f64) {
        let Some(fd) = self.sock.cloned() else {
            return;
        };
        let nfd = match kos::socket::accept(fd) {
            Ok(nfd) => nfd,
            Err(_) => return,
        };
        let _ = kos::fd::set_nonblock(nfd);
        let client = ClientConnection::new(self, Arc::new(FdSocket::new(nfd)));
        self.track_client(client);
    }

    /// Tracks an accepted client (`clients[client.uid] = client`,
    /// `klippy/webhooks.py:148`). `_handle_accept` calls this;
    /// tests drive it directly for the `stats` pruning path.
    pub fn track_client(&self, client: Arc<ClientConnection>) {
        self.clients.borrow_mut().insert(client.uid(), client);
    }

    /// `_handle_disconnect` (`klippy/webhooks.py:150-158`).
    pub fn handle_disconnect(&self) {
        // `list(self.clients.values())`: the snapshot must not keep the cell
        // borrowed, `close` untracks the client from it.
        let clients: Vec<Arc<ClientConnection>> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            ClientConnectionApi::close(&*client);
        }
        if let Some(fd) = self.sock.cloned() {
            if let Some(h) = self.fd_handle.replace(None) {
                self.reactor.unregister_fd(&h);
            }
            kos::fd::close(fd);
        }
        self.sock.set(None);
    }

    /// `_handle_analyze_shutdown` (`klippy/webhooks.py:160-162`).
    pub fn handle_analyze_shutdown(&self, _msg: &str, _details: &PyValue) {
        let clients: Vec<Arc<ClientConnection>> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            ClientConnectionApi::dump_request_log(&*client);
        }
    }

    /// `pop_client` (`klippy/webhooks.py:174-175`).
    pub fn pop_client(&self, client_id: u64) {
        self.clients.borrow_mut().shift_remove(&client_id);
    }

    /// `stats` (`klippy/webhooks.py:177-185`): called once per second.
    pub fn stats(&self, _eventtime: f64) -> (bool, String) {
        let clients: Vec<Arc<ClientConnection>> = self.clients.borrow().values().cloned().collect();
        for client in clients {
            if client.is_blocking() {
                let count = client.blocking_count() - 1;
                client.set_blocking_count(count);
                if count < 0 {
                    let msg = fmt_msg(
                        "Closing unresponsive client %s",
                        &[PctValue::Int(PyInt::from_u64(client.uid()))],
                    );
                    log::info(&msg);
                    ClientConnectionApi::close(&*client);
                }
            }
        }
        (false, "".to_string())
    }

    /// Number of live clients (tests).
    pub fn client_count(&self) -> usize {
        self.clients.borrow().len()
    }
    /// A client by uid (tests).
    pub fn client(&self, uid: u64) -> Option<Arc<ClientConnection>> {
        self.clients.borrow().get(&uid).cloned()
    }
    /// All tracked clients in insertion order (tests).
    pub fn clients_snapshot(&self) -> Vec<Arc<ClientConnection>> {
        self.clients.borrow().values().cloned().collect()
    }
    /// Whether the listener socket is bound (tests).
    pub fn is_enabled(&self) -> bool {
        self.sock.borrow().is_some()
    }
}

fn analyze_shutdown_args(args: &[EventArg]) -> (String, PyValue) {
    let msg = args.first().and_then(|a| match a {
        EventArg::Value(PyValue::Str(s)) => Some(s.clone()),
        _ => None,
    }).unwrap_or_default();
    let details = args.get(1).and_then(|a| match a {
        EventArg::Value(v) => Some(v.clone()),
        _ => None,
    }).unwrap_or_else(|| PyValue::None);
    (msg, details)
}

// ---------------------------------------------------------------------------
// `WebHooks` (`klippy/webhooks.py:315-437`)
// ---------------------------------------------------------------------------

struct MuxEntry {
    key: String,
    values: IndexMap<String, WebHooksCallback>,
}

/// Mux endpoint dispatch (`_mux_endpoints`, `klippy/webhooks.py:320`).
/// Lives behind `Arc` so the endpoint closure installed by
/// `register_mux_endpoint` (which runs on `&self` with no `Arc`)
/// can dispatch without capturing `Arc<WebHooks>`.
pub struct MuxTable {
    entries: GCell<IndexMap<String, MuxEntry>>,
}

impl Default for MuxTable {
    fn default() -> Self {
        MuxTable { entries: GCell::new(IndexMap::new()) }
    }
}

impl MuxTable {
    /// Whether a mux endpoint exists for `path` (tests).
    pub fn has(&self, path: &str) -> bool {
        self.entries.borrow().contains_key(path)
    }

    /// Value-validation part of `register_mux_endpoint`
    /// (`klippy/webhooks.py:338-347`), after the endpoint itself
    /// has been registered by `WebHooks::register_mux_endpoint`.
    pub fn register(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        if !self.entries.borrow().contains_key(path) {
            self.entries
                .borrow_mut()
                .insert(path.to_string(), MuxEntry { key: key.to_string(), values: IndexMap::new() });
        }
        let mut entries = self.entries.borrow_mut();
        let entry = entries.get_mut(path).expect("inserted");
        if entry.key != key {
            let msg = fmt_msg(
                "mux endpoint %s %s %s may have only one key (%s)",
                &[
                    PctValue::Str(path.to_string()),
                    PctValue::Str(key.to_string()),
                    PctValue::Str(value.to_string()),
                    PctValue::Str(entry.key.clone()),
                ],
            );
            return Err(config_error(msg));
        }
        if entry.values.contains_key(value) {
            let prev_keys: Vec<String> = entry
                .values
                .keys()
                .map(|k| pycompat::text::str_repr(k))
                .collect();
            let prev_repr = format!("{{{}}}", prev_keys.join(", "));
            let msg = fmt_msg(
                "mux endpoint %s %s %s already registered (%s)",
                &[
                    PctValue::Str(path.to_string()),
                    PctValue::Str(key.to_string()),
                    PctValue::Str(value.to_string()),
                    PctValue::Str(prev_repr),
                ],
            );
            return Err(config_error(msg));
        }
        entry.values.insert(value.to_string(), callback);
        Ok(())
    }

    /// `_handle_mux` (`klippy/webhooks.py:349-358`).
    pub fn dispatch(&self, web_request: WebRequestHandle) -> KResult<()> {
        let (key, values) = {
            let entries = self.entries.borrow();
            let entry = entries
                .get(&web_request.get_method())
                .ok_or_else(|| web_request_error("internal mux dispatch"))?;
            (
                entry.key.clone(),
                entry.values.iter().map(|(k, v)| (k.clone(), v.clone())).collect::<Vec<_>>(),
            )
        };
        let key_param = web_request.get(&key, None)?;
        // `key_param not in values` hashes the value first.
        match &key_param {
            PyValue::List(_) => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "unhashable type: 'list'".to_string(),
                });
            }
            PyValue::Dict(_) => {
                return Err(KlippyError::Internal {
                    class_name: "TypeError".to_string(),
                    message: "unhashable type: 'dict'".to_string(),
                });
            }
            _ => {}
        }
        let found = match &key_param {
            PyValue::Str(s) => values.iter().find(|(k, _)| k == s).map(|(_, cb)| cb.clone()),
            _ => None,
        };
        let Some(cb) = found else {
            let msg = fmt_msg(
                "The value '%s' is not valid for %s",
                &[PctValue::Str(key_param.str()), PctValue::Str(key.clone())],
            );
            return Err(web_request_error(msg));
        };
        cb(web_request)
    }
}

struct HooksState {
    endpoints: IndexMap<String, WebHooksCallback>,
    remote: IndexMap<String, IndexMap<u64, (ClientConnHandle, PyValue)>>,
}

/// The `webhooks` printer object (`klippy/webhooks.py:315`).
pub struct WebHooks {
    printer: Printer,
    host: HostInfo,
    mux: Arc<MuxTable>,
    state: GCell<HooksState>,
    sconn: GCell<Option<Arc<ServerSocket>>>,
}

impl WebHooks {
    /// WebHooks(printer) (`klippy/webhooks.py:316-326`), with `host`
    /// standing in for the `os`/`socket`/`sys` reads of the `info`
    /// endpoint (`klippy/webhooks.py:377-386`).
    pub fn new(printer: &Printer, host: HostInfo) -> KResult<Arc<WebHooks>> {
        let wh = Arc::new(WebHooks {
            printer: printer.clone(),
            host,
            mux: Arc::new(MuxTable::default()),
            state: GCell::new(HooksState {
                endpoints: IndexMap::new(),
                remote: IndexMap::new(),
            }),
            sconn: GCell::new(None),
        });
        let c = wh.clone();
        wh.register_endpoint(
            "list_endpoints",
            Arc::new(move |wr| c.handle_list_endpoints(wr)),
        )?;
        let c = wh.clone();
        wh.register_endpoint("shakehands", Arc::new(move |wr| c.handle_shakehands(wr)))?;
        let wh_c = wh.clone();
        wh.register_endpoint(
            "info",
            Arc::new(move |wr| wh_c.handle_info(wr)),
        )?;
        let c = wh.clone();
        wh.register_endpoint(
            "emergency_stop",
            Arc::new(move |wr| c.handle_estop(wr)),
        )?;
        let c = wh.clone();
        wh.register_endpoint(
            "register_remote_method",
            Arc::new(move |wr| c.handle_rpc_registration(wr)),
        )?;
        let sconn = ServerSocket::new(wh.clone(), printer)?;
        wh.sconn.replace(Some(sconn));
        Ok(wh)
    }

    /// `get_connection` (`klippy/webhooks.py:403-404`).
    pub fn get_connection(&self) -> Arc<ServerSocket> {
        self.sconn
            .borrow()
            .clone()
            .expect("WebHooks::new installs the server socket")
    }

    /// Registered endpoint paths in registration order (tests).
    pub fn endpoint_paths(&self) -> Vec<String> {
        self.state.borrow().endpoints.keys().cloned().collect()
    }

    fn status_value(&self, _eventtime: f64) -> PyValue {
        let (state_message, state) = self.printer.get_state_message();
        py_dict(vec![
            (py_str("state"), py_str(&state)),
            (py_str("state_message"), py_str(&state_message)),
        ])
    }

    fn handle_list_endpoints(&self, web_request: WebRequestHandle) -> KResult<()> {
        let endpoints: Vec<PyValue> = self
            .state
            .borrow()
            .endpoints
            .keys()
            .map(|s| py_str(s))
            .collect();
        web_request.send(py_dict(vec![(py_str("endpoints"), PyValue::List(endpoints))]))
    }

    fn handle_shakehands(&self, web_request: WebRequestHandle) -> KResult<()> {
        let (state_message, state) = self.printer.get_state_message();
        let curtime = self.printer.get_reactor().monotonic();
        web_request.send(py_dict(vec![
            (py_str("state"), py_str(&state)),
            (py_str("state_message"), py_str(&state_message)),
            (py_str("shakehands_id"), web_request.id().unwrap_or(PyValue::None)),
            (py_str("curtime"), PyValue::Float(curtime)),
        ]))
    }

    fn handle_info(&self, web_request: WebRequestHandle) -> KResult<()> {
        let client_info = web_request.get_dict("client_info", Some(PyValue::None))?;
        if !matches!(client_info, PyValue::None) {
            web_request.get_client_connection().set_client_info(client_info, None);
        }
        let (state_message, state) = self.printer.get_state_message();
        let response = py_dict(vec![
            (py_str("state"), py_str(&state)),
            (py_str("state_message"), py_str(&state_message)),
            (py_str("hostname"), py_str(&self.host.hostname)),
            (py_str("klipper_path"), py_str(&self.host.klipper_path)),
            (py_str("python_path"), py_str(&self.host.python_path)),
            (py_str("process_id"), py_int(self.host.process_id)),
            (py_str("user_id"), py_int(self.host.user_id)),
            (py_str("group_id"), py_int(self.host.group_id)),
        ]);
        let start_args = self.printer.get_start_args();
        let mut map = match response {
            PyValue::Dict(m) => m,
            _ => unreachable!("dict"),
        };
        for sa in ["log_file", "config_file", "software_version", "cpu_info"] {
            let v = py_dict_get(&start_args, sa).unwrap_or(PyValue::None);
            map.insert(py_str(sa), v);
        }
        web_request.send(PyValue::Dict(map))
    }

    fn handle_estop(&self, web_request: WebRequestHandle) -> KResult<()> {
        let _ = web_request;
        self.printer
            .invoke_shutdown("Shutdown due to webhooks request", empty_dict());
        Ok(())
    }

    fn handle_rpc_registration(&self, web_request: WebRequestHandle) -> KResult<()> {
        let template = web_request.get_dict("response_template", None)?;
        let method = web_request.get_str("remote_method", None)?;
        let new_conn = web_request.get_client_connection();
        let msg = fmt_msg(
            "webhooks: registering remote method '%s' for connection id: %d",
            &[
                PctValue::Str(method.clone()),
                PctValue::Int(PyInt::from_u64(new_conn.uid())),
            ],
        );
        log::info(&msg);
        self.state
            .borrow_mut()
            .remote
            .entry(method)
            .or_default()
            .insert(new_conn.uid(), (new_conn, template));
        Ok(())
    }


    /// `get_callback` (`klippy/webhooks.py:406-412`).
    pub fn get_callback(&self, path: &str) -> KResult<WebHooksCallback> {
        let cb = self.state.borrow().endpoints.get(path).cloned();
        match cb {
            Some(cb) => Ok(cb),
            None => {
                let msg = fmt_msg(
                    "webhooks: No registered callback for path '%s'",
                    &[PctValue::Str(path.to_string())],
                );
                log::info(&msg);
                Err(web_request_error(msg))
            }
        }
    }
}

impl WebHooksApi for WebHooks {
    fn register_endpoint(&self, path: &str, callback: WebHooksCallback) -> KResult<()> {
        if self.state.borrow().endpoints.contains_key(path) {
            return Err(web_request_error("Path already registered to an endpoint"));
        }
        self.state.borrow_mut().endpoints.insert(path.to_string(), callback);
        Ok(())
    }

    fn register_mux_endpoint(
        &self,
        path: &str,
        key: &str,
        value: &str,
        callback: WebHooksCallback,
    ) -> KResult<()> {
        if !self.mux.has(path) {
            let mux = self.mux.clone();
            self.register_endpoint(path, Arc::new(move |wr| mux.dispatch(wr)))?;
        }
        self.mux.register(path, key, value, callback)
    }

    fn call_remote_method(
        &self,
        method: &str,
        params: &IndexMap<String, PyValue>,
    ) -> KResult<()> {
        let conn_map = {
            let st = self.state.borrow();
            st.remote.get(method).cloned()
        };
        let conn_map = match conn_map {
            Some(m) => m,
            None => {
                let msg = fmt_msg("Remote method '%s' not registered", &[PctValue::Str(method.to_string())]);
                return Err(cmd_error(msg));
            }
        };
        let kwargs = py_dict(
            params
                .iter()
                .map(|(k, v)| (py_str(k), v.clone()))
                .collect(),
        );
        let mut valid: IndexMap<u64, (ClientConnHandle, PyValue)> = IndexMap::new();
        for (uid, (conn, template)) in conn_map.iter() {
            if !conn.is_closed() {
                valid.insert(*uid, (conn.clone(), template.clone()));
                // `{'params': kwargs}` then `update(template)`.
                let mut out = IndexMap::new();
                out.insert(py_str("params"), kwargs.clone());
                if let PyValue::Dict(t) = template {
                    for (k, v) in t.iter() {
                        out.insert(k.clone(), v.clone());
                    }
                }
                conn.send(PyValue::Dict(out));
            }
        }
        if valid.is_empty() {
            self.state.borrow_mut().remote.shift_remove(method);
            let msg = fmt_msg(
                "No active connections for method '%s'",
                &[PctValue::Str(method.to_string())],
            );
            return Err(cmd_error(msg));
        }
        self.state.borrow_mut().remote.insert(method.to_string(), valid);
        Ok(())
    }

    fn get_status(&self, eventtime: f64) -> PyValue {
        self.status_value(eventtime)
    }
}

impl PrinterObject for WebHooks {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn WebHooksApi);
    fn teardown(&self) {
        if let Some(server) = self.sconn.replace(None) {
            server.handle_disconnect();
        }
        let (endpoints, remote) = {
            let mut state = self.state.borrow_mut();
            (std::mem::take(&mut state.endpoints), std::mem::take(&mut state.remote))
        };
        let mux = self.mux.entries.replace(IndexMap::new());
        drop((endpoints, remote, mux));
    }
    fn get_status(&self, eventtime: f64) -> klippy_api::KResult<Option<PyValue>> {
        Ok(Some(self.status_value(eventtime)))
    }
    fn stats(&self, eventtime: f64) -> Option<(bool, String)> {
        self.sconn.borrow().clone().map(|s| s.stats(eventtime))
    }
}


// ---------------------------------------------------------------------------
// `GCodeHelper` (`klippy/webhooks.py:439-478`)
// ---------------------------------------------------------------------------

// (the gcode output-handler signature comes from `klippy_api::gcode`.)

/// G-code bridge (`klippy/webhooks.py:439`); not itself a printer object.
pub struct GCodeHelper {
    gcode: klippy_api::gcode::GCode,
    clients: GCell<IndexMap<u64, (ClientConnHandle, PyValue)>>,
    is_output_registered: GCell<bool>,
    self_weak: GCell<Option<Weak<GCodeHelper>>>,
}

impl GCodeHelper {
    /// GCodeHelper(printer) (`klippy/webhooks.py:440-454`).
    pub fn new(printer: &Printer) -> KResult<Arc<GCodeHelper>> {
        let gcode = object::lookup_interface::<dyn GCodeApi>(&**printer, "gcode", klippy_api::printer::LookupDefault::Raise)?
            .expect("Raise yields Some");
        let wh = object::lookup_interface::<dyn WebHooksApi>(&**printer, "webhooks", klippy_api::printer::LookupDefault::Raise)?
            .expect("Raise yields Some");
        let helper = Arc::new(GCodeHelper {
            gcode,
            clients: GCell::new(IndexMap::new()),
            is_output_registered: GCell::new(false),
            self_weak: GCell::new(None),
        });
        helper.self_weak.replace(Some(Arc::downgrade(&helper)));
        let c = helper.clone();
        wh.register_endpoint("gcode/help", Arc::new(move |wr| c.handle_help(wr)))?;
        let c = helper.clone();
        wh.register_endpoint("gcode/script", Arc::new(move |wr| c.handle_script(wr)))?;
        let c = helper.clone();
        wh.register_endpoint("gcode/restart", Arc::new(move |wr| c.handle_restart(wr)))?;
        let c = helper.clone();
        wh.register_endpoint(
            "gcode/firmware_restart",
            Arc::new(move |wr| c.handle_firmware_restart(wr)),
        )?;
        let c = helper.clone();
        wh.register_endpoint(
            "gcode/subscribe_output",
            Arc::new(move |wr| c.handle_subscribe_output(wr)),
        )?;
        Ok(helper)
    }

    fn handle_help(&self, web_request: WebRequestHandle) -> KResult<()> {
        let help = self.gcode.get_command_help();
        let pairs: Vec<(PyValue, PyValue)> = help
            .iter()
            .map(|(k, v)| (py_str(k), py_str(v)))
            .collect();
        web_request.send(py_dict(pairs))
    }

    fn handle_script(&self, web_request: WebRequestHandle) -> KResult<()> {
        let script = web_request.get_str("script", None)?;
        self.gcode.run_script(&script)
    }

    fn handle_restart(&self, web_request: WebRequestHandle) -> KResult<()> {
        let _ = web_request;
        self.gcode.run_script("restart")
    }

    fn handle_firmware_restart(&self, web_request: WebRequestHandle) -> KResult<()> {
        let _ = web_request;
        self.gcode.run_script("firmware_restart")
    }

    /// `_output_callback` (`klippy/webhooks.py:463-470`).
    pub fn output_callback(&self, msg: &str) {
        let snapshot: Vec<(u64, (ClientConnHandle, PyValue))> = self
            .clients
            .borrow()
            .iter()
            .map(|(uid, entry)| (*uid, entry.clone()))
            .collect();
        for (uid, (conn, template)) in snapshot {
            if conn.is_closed() {
                self.clients.borrow_mut().shift_remove(&uid);
                continue;
            }
            let mut tmp = match &template {
                PyValue::Dict(m) => m.clone(),
                _ => IndexMap::new(),
            };
            tmp.insert(
                py_str("params"),
                py_dict(vec![(py_str("response"), py_str(msg))]),
            );
            conn.send(PyValue::Dict(tmp));
        }
    }

    fn handle_subscribe_output(&self, web_request: WebRequestHandle) -> KResult<()> {
        let conn = web_request.get_client_connection();
        let template = web_request.get_dict("response_template", Some(empty_dict()))?;
        self.clients
            .borrow_mut()
            .insert(conn.uid(), (conn, template));
        if !self.is_output_registered.get() {
            self.is_output_registered.set(true);
            let weak = self.self_weak.borrow().clone();
            self.gcode.register_output_handler(Box::new(move |msg: String| {
                if let Some(helper) = weak.as_ref().and_then(|w| w.upgrade()) {
                    helper.output_callback(&msg);
                }
            }));
        }
        Ok(())
    }

    /// Whether the output handler is registered (tests).
    pub fn is_output_registered(&self) -> bool {
        self.is_output_registered.get()
    }
    /// Number of subscribed clients (tests).
    pub fn client_count(&self) -> usize {
        self.clients.borrow().len()
    }
}

// ---------------------------------------------------------------------------
// `QueryStatusHelper` (`klippy/webhooks.py:481-572`)
// ---------------------------------------------------------------------------

/// `printer-object name -> {field -> value}`, the shape of
/// `query` / `last_query` in `_do_query`.
type StatusMap = IndexMap<String, IndexMap<String, PyValue>>;

/// The `objects` argument of `objects/query|subscribe`: a printer-object
/// name to `None` (all fields) or a field list. One dict is shared by the
/// pending one-shot query and the subscription entry `_handle_query`
/// stores, and `_do_query` freezes `None` entries into field lists in
/// place (`klippy/webhooks.py:521-524`), so it is shared here too.
pub type Subscription = Arc<GCell<IndexMap<String, Option<Vec<String>>>>>;

/// Wraps a validated `objects` dict as a shareable [`Subscription`].
pub fn new_subscription(objects: IndexMap<String, Option<Vec<String>>>) -> Subscription {
    Arc::new(GCell::new(objects))
}

/// A live subscription: `(cconn, objects, cconn.send, template)`.
#[derive(Clone)]
struct SubClient {
    conn: ClientConnHandle,
    objects: Subscription,
    template: PyValue,
}

/// A pending one-shot query `(None, objects, complete.complete, {})`.
struct PendingQuery {
    objects: Subscription,
    completion: Completion<PyValue>,
}

/// What `_handle_query` keeps between enqueueing the query and reading
/// its answer: the completion, the shared `objects`, the template and
/// the client connection.
pub struct QueryTicket {
    completion: Completion<PyValue>,
    objects: Subscription,
    template: PyValue,
    conn: ClientConnHandle,
}

/// `objects/query|subscribe` backend (`klippy/webhooks.py:481`).
pub struct QueryStatusHelper {
    printer: Printer,
    clients: GCell<IndexMap<u64, SubClient>>,
    pending: GCell<Vec<PendingQuery>>,
    last_query: GCell<StatusMap>,
    query_timer: GCell<Option<Timer>>,
    self_weak: GCell<Option<Weak<QueryStatusHelper>>>,
}

impl QueryStatusHelper {
    /// QueryStatusHelper(printer) (`klippy/webhooks.py:482-492`).
    pub fn new(printer: &Printer) -> KResult<Arc<QueryStatusHelper>> {
        let webhooks = object::lookup_interface::<dyn WebHooksApi>(
            &**printer,
            "webhooks",
            klippy_api::printer::LookupDefault::Raise,
        )?
        .expect("Raise yields Some");
        let helper = Arc::new(QueryStatusHelper {
            printer: printer.clone(),
            clients: GCell::new(IndexMap::new()),
            pending: GCell::new(Vec::new()),
            last_query: GCell::new(IndexMap::new()),
            query_timer: GCell::new(None),
            self_weak: GCell::new(None),
        });
        helper.self_weak.replace(Some(Arc::downgrade(&helper)));
        let c = helper.clone();
        webhooks.register_endpoint("objects/list", Arc::new(move |wr| c.handle_list(wr)))?;
        let c = helper.clone();
        webhooks.register_endpoint(
            "objects/query",
            Arc::new(move |wr| c.handle_query(wr, false)),
        )?;
        let c = helper.clone();
        webhooks.register_endpoint(
            "objects/subscribe",
            Arc::new(move |wr| c.handle_query(wr, true)),
        )?;
        Ok(helper)
    }

    /// `_handle_list` (`klippy/webhooks.py:493-496`). `hasattr(o,
    /// 'get_status')` is `PrinterObject::get_status` not returning `Ok(None)`
    /// (a raising `get_status` still exists); a section stored as `None`
    /// has no `get_status`.
    pub fn handle_list(&self, web_request: WebRequestHandle) -> KResult<()> {
        let objects: Vec<PyValue> = self
            .printer
            .lookup_objects(None)
            .into_iter()
            .filter(|(_, o)| o.as_ref().is_some_and(|o| !matches!(o.get_status(0.), Ok(None))))
            .map(|(n, _)| py_str(&n))
            .collect();
        web_request.send(py_dict(vec![(py_str("objects"), PyValue::List(objects))]))
    }

    /// `_do_query` (`klippy/webhooks.py:497-543`): the timer body.
    /// Returns the next waketime (`NEVER` when no subscription
    /// remains, else `eventtime + SUBSCRIPTION_REFRESH_TIME`).
    pub fn do_query(&self, eventtime: f64) -> f64 {
        let last_query = self.last_query.replace(IndexMap::new());
        let mut query: StatusMap = IndexMap::new();
        // `msglist`: pending one-shot queries first, then the live clients.
        let pending = self.pending.replace(Vec::new());
        let live: Vec<(u64, SubClient)> = self
            .clients
            .borrow()
            .iter()
            .map(|(uid, client)| (*uid, client.clone()))
            .collect();
        let reactor = self.printer.get_reactor();
        {
            let _no_pause = reactor.assert_no_pause();
            for p in pending {
                let cquery =
                    self.query_subscription(&p.objects, &mut query, &last_query, eventtime, true);
                p.completion
                    .complete(query_message(&empty_dict(), eventtime, &cquery));
            }
            for (uid, client) in live {
                if client.conn.is_closed() {
                    self.clients.borrow_mut().shift_remove(&uid);
                    continue;
                }
                let cquery = self.query_subscription(
                    &client.objects,
                    &mut query,
                    &last_query,
                    eventtime,
                    false,
                );
                if !cquery.is_empty() {
                    client
                        .conn
                        .send(query_message(&client.template, eventtime, &cquery));
                }
            }
        }
        let no_subscriptions = query.is_empty();
        self.last_query.replace(query);
        if no_subscriptions {
            // Unregister the timer if there are no longer any subscriptions.
            if let Some(timer) = self.query_timer.replace(None) {
                reactor.unregister_timer(&timer);
            }
            return NEVER;
        }
        eventtime + SUBSCRIPTION_REFRESH_TIME
    }

    /// One client's status computation (`klippy/webhooks.py:512-532`).
    /// Returns the changed-fields dict; a `None` field list is frozen
    /// into the list of the object's current keys inside `subscription`.
    fn query_subscription(
        &self,
        subscription: &Subscription,
        query: &mut StatusMap,
        last_query: &StatusMap,
        eventtime: f64,
        is_query: bool,
    ) -> StatusMap {
        let mut cquery: StatusMap = IndexMap::new();
        let obj_names: Vec<String> = subscription.borrow().keys().cloned().collect();
        for obj_name in obj_names {
            if !query.contains_key(&obj_name) {
                let res = self.fetch_status(&obj_name, eventtime);
                query.insert(obj_name.clone(), res);
            }
            let res = &query[&obj_name];
            let current = subscription.borrow().get(&obj_name).cloned().flatten();
            let req_items: Vec<String> = match current {
                Some(items) => items,
                None => {
                    let items: Vec<String> = res.keys().cloned().collect();
                    if !items.is_empty() {
                        subscription
                            .borrow_mut()
                            .insert(obj_name.clone(), Some(items.clone()));
                    }
                    items
                }
            };
            let lres = last_query.get(&obj_name);
            let mut cres: IndexMap<String, PyValue> = IndexMap::new();
            for ri in &req_items {
                let rd = res.get(ri).cloned().unwrap_or(PyValue::None);
                let prev = lres
                    .and_then(|m| m.get(ri))
                    .cloned()
                    .unwrap_or(PyValue::None);
                if is_query || !py_equal(&rd, &prev) {
                    cres.insert(ri.clone(), rd);
                }
            }
            if !cres.is_empty() || is_query {
                cquery.insert(obj_name, cres);
            }
        }
        cquery
    }

    /// `po = lookup_object(obj_name, None)` and `po.get_status(eventtime)`;
    /// an object without `get_status` reads as `{}`. A raising
    /// `get_status` escapes the `_do_query` timer like the Python exception.
    fn fetch_status(&self, obj_name: &str, eventtime: f64) -> IndexMap<String, PyValue> {
        let obj = self
            .printer
            .lookup_object(obj_name, klippy_api::printer::LookupDefault::ReturnNone)
            .ok()
            .flatten();
        let Some(obj) = obj else {
            return IndexMap::new();
        };
        match obj.get_status(eventtime) {
            Ok(Some(PyValue::Dict(m))) => m
                .iter()
                .filter_map(|(k, v)| match k {
                    PyValue::Str(s) => Some((s.clone(), v.clone())),
                    _ => None,
                })
                .collect(),
            Err(e) => e.raise(),
            // The contract says `get_status` returns a dict.
            Ok(_) => IndexMap::new(),
        }
    }

    /// `_handle_query` from argument validation to the timer start
    /// (`klippy/webhooks.py:545-565`), without the completion wait.
    /// [`handle_query`](Self::handle_query) chains it with
    /// [`finish_query`](Self::finish_query), which blocks on the
    /// completion; tests call the halves separately and run
    /// [`do_query`](Self::do_query) in between, like the reactor does.
    pub fn begin_query(
        &self,
        web_request: &WebRequestHandle,
        is_subscribe: bool,
    ) -> KResult<QueryTicket> {
        let objects_value = web_request.get_dict("objects", None)?;
        let objects = parse_objects_arg(&objects_value)?;
        let conn = web_request.get_client_connection();
        let template = web_request.get_dict("response_template", Some(empty_dict()))?;
        if is_subscribe {
            self.clients.borrow_mut().shift_remove(&conn.uid());
        }
        let reactor = self.printer.get_reactor();
        let completion = reactor.completion::<PyValue>();
        self.pending.borrow_mut().push(PendingQuery {
            objects: objects.clone(),
            completion: completion.clone(),
        });
        if self.query_timer.borrow().is_none() {
            let weak = self.self_weak.borrow().clone();
            let timer = reactor.register_timer(
                move |eventtime: f64| match weak.as_ref().and_then(|w| w.upgrade()) {
                    Some(s) => s.do_query(eventtime),
                    None => NEVER,
                },
                NOW,
            );
            self.query_timer.replace(Some(timer));
        }
        Ok(QueryTicket {
            completion,
            objects,
            template,
            conn,
        })
    }

    /// `_handle_query` tail (`klippy/webhooks.py:566-570`):
    /// wait for the completion, `send(msg['params'])`, and for
    /// `objects/subscribe` register the client after the send.
    pub fn finish_query(
        &self,
        web_request: &WebRequestHandle,
        ticket: QueryTicket,
        is_subscribe: bool,
    ) -> KResult<()> {
        let QueryTicket {
            completion,
            objects,
            template,
            conn,
        } = ticket;
        let msg = completion.wait(NEVER).ok_or_else(|| KlippyError::Internal {
            class_name: "RuntimeError".to_string(),
            message: "query completion did not complete".to_string(),
        })?;
        let params = py_dict_get(&msg, "params").ok_or_else(|| KlippyError::Internal {
            class_name: "KeyError".to_string(),
            message: "'params'".to_string(),
        })?;
        web_request.send(params)?;
        if is_subscribe {
            self.clients.borrow_mut().insert(
                conn.uid(),
                SubClient {
                    conn,
                    objects,
                    template,
                },
            );
        }
        Ok(())
    }

    /// `_handle_query` (`klippy/webhooks.py:544-570`) and
    /// `_handle_subscribe` (`klippy/webhooks.py:571-572`).
    pub fn handle_query(&self, web_request: WebRequestHandle, is_subscribe: bool) -> KResult<()> {
        let ticket = self.begin_query(&web_request, is_subscribe)?;
        self.finish_query(&web_request, ticket, is_subscribe)
    }

    /// Number of pending one-shot queries (tests).
    pub fn pending_len(&self) -> usize {
        self.pending.borrow().len()
    }
    /// Whether the refresh timer is armed (tests).
    pub fn has_timer(&self) -> bool {
        self.query_timer.borrow().is_some()
    }
    /// Number of subscribed clients (tests).
    pub fn client_count(&self) -> usize {
        self.clients.borrow().len()
    }
    /// The field lists of one subscribed client's `objects` (tests).
    pub fn client_objects(&self, uid: u64) -> Option<IndexMap<String, Option<Vec<String>>>> {
        self.clients.borrow().get(&uid).map(|c| c.objects.cloned())
    }
    /// `last_query` snapshot (tests).
    pub fn last_query_snapshot(&self) -> IndexMap<String, IndexMap<String, PyValue>> {
        self.last_query.borrow().clone()
    }
}

/// `tmp = dict(template); tmp['params'] = {'eventtime': .., 'status': ..}`.
fn query_message(template: &PyValue, eventtime: f64, cquery: &StatusMap) -> PyValue {
    let mut tmp = match template {
        PyValue::Dict(m) => m.clone(),
        _ => IndexMap::new(),
    };
    tmp.insert(
        py_str("params"),
        py_dict(vec![
            (py_str("eventtime"), PyValue::Float(eventtime)),
            (py_str("status"), status_to_py(cquery)),
        ]),
    );
    PyValue::Dict(tmp)
}

fn status_to_py(cquery: &StatusMap) -> PyValue {
    let mut outer = IndexMap::new();
    for (obj, fields) in cquery {
        let mut inner = IndexMap::new();
        for (k, v) in fields {
            inner.insert(py_str(k), v.clone());
        }
        outer.insert(py_str(obj), PyValue::Dict(inner));
    }
    PyValue::Dict(outer)
}

/// `objects` argument validation + conversion
/// (`klippy/webhooks.py:546-553`): dict of `str -> null | [str]`.
pub fn parse_objects_arg(v: &PyValue) -> KResult<Subscription> {
    let PyValue::Dict(m) = v else {
        // `get_dict` already guarantees a dict; defensive.
        return Err(web_request_error("Invalid argument"));
    };
    let mut out = IndexMap::new();
    for (k, val) in m.iter() {
        let PyValue::Str(name) = k else {
            return Err(web_request_error("Invalid argument"));
        };
        match val {
            PyValue::None => {
                out.insert(name.clone(), None);
            }
            PyValue::List(items) => {
                let mut fields = Vec::new();
                for item in items {
                    match item {
                        PyValue::Str(s) => fields.push(s.clone()),
                        _ => return Err(web_request_error("Invalid argument")),
                    }
                }
                out.insert(name.clone(), Some(fields));
            }
            _ => return Err(web_request_error("Invalid argument")),
        }
    }
    Ok(new_subscription(out))
}

// ---------------------------------------------------------------------------
// `add_early_printer_objects` + module factory (`klippy/webhooks.py:574-577`)
// ---------------------------------------------------------------------------

/// `add_early_printer_objects(printer)` (`klippy/webhooks.py:574-577`).
pub fn add_early_printer_objects(printer: &Printer) -> KResult<()> {
    let wh = WebHooks::new(printer, HostInfo::current())?;
    printer.add_object("webhooks", wh as ObjectRef)?;
    GCodeHelper::new(printer)?;
    QueryStatusHelper::new(printer)?;
    Ok(())
}

/// Module factory (early printer objects only; there is no `load_config`).
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        add_early_printer_objects: Some(add_early_printer_objects),
        ..Default::default()
    }
}
