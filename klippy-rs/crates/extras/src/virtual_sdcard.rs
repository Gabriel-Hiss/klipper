//! Port of the `virtual_sdcard` module (`klippy/extras/virtual_sdcard.py`).
//!
//! `VirtualSD` prints a G-code file straight from a host directory: it
//! registers `M20`..`M30`, `SDCARD_RESET_FILE` and `SDCARD_PRINT_FILE`, runs
//! the print from a reactor timer (`work_handler`) and keeps the fork's
//! `[DEBUG-motion] v4` producer timing rings that `klippy:analyze_shutdown`
//! dumps.
//!
//! The Python file object is `io.open(fname, 'r', newline='')`, a
//! `TextIOWrapper` over a `BufferedReader`. `read(8192)` returns 8192
//! characters (not bytes), decoding errors surface when the chunk holding
//! the bad bytes is decoded, and `seek(position)` takes a byte offset.
//! [`TextFile`] reproduces that reading algorithm (UTF-8 decoding in raw
//! chunks of `max(8192, b2cratio * remaining)` bytes, the incremental
//! newline decoder holding back a trailing `\r`) so chunk boundaries, error
//! points and the debug counters follow the reference.

use std::collections::VecDeque;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{Config, GetStrOpts};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcell::GCell;
use klippy_api::gcode::{GCmd, GCode, GCodeApi, GCodeCommandHandler, GetIntOpts, GetOpts, RegisterCommandOpts};
use klippy_api::log;
use klippy_api::object::{load_interface, lookup_interface, EventArg, ObjectRef, PrinterObject};
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer, EV_KLIPPY_ANALYZE_SHUTDOWN, EV_VIRTUAL_SDCARD_RESET_FILE};
use pycompat::text::{percent_format, str_repr, PctValue};
use pycompat::{PyInt, PyValue};
use reactor::{Reactor, Timer, NEVER, NOW};

use crate::gcode_macro::{PrinterGCodeMacroApi, TemplateWrapper};
use crate::print_stats::PrintStatsApi;

/// `VALID_GCODE_EXTS`.
const VALID_GCODE_EXTS: [&str; 3] = ["gcode", "g", "gco"];

/// `DEFAULT_ERROR_GCODE` (the Python literal starts right after the
/// opening quotes, so the text begins with a newline).
const DEFAULT_ERROR_GCODE: &str =
    "\n{% if 'heaters' in printer %}\n   TURN_OFF_HEATERS\n{% endif %}\n";

const DEBUG_CHUNK_RING_SIZE: usize = 24;
const DEBUG_SLOW_RING_SIZE: usize = 16;
const DEBUG_SLOW_CMD_TIME: f64 = 0.020;
const DEBUG_SLOW_YIELD_TIME: f64 = 0.020;
const DEBUG_CMD_NAME_MAX: usize = 24;
const DEBUG_DUMP_ENTRIES: usize = 10;

/// `cmd_SDCARD_RESET_FILE_help` (two adjacent Python literals).
const CMD_SDCARD_RESET_FILE_HELP: &str = "Clears a loaded SD File. Stops the print if necessary";
/// `cmd_SDCARD_PRINT_FILE_help` (two adjacent Python literals).
const CMD_SDCARD_PRINT_FILE_HELP: &str =
    "Loads a SD file and starts the print.  May include files in subdirectories.";

/// `TextIOWrapper._CHUNK_SIZE`.
const TEXT_CHUNK_SIZE: usize = 8192;

/// The methods other modules call on the `virtual_sdcard` object:
/// `pause_resume` (`is_active`, `do_pause`, `do_resume`, `do_cancel`),
/// `sdcard_loop` (`is_cmd_from_sd`, `get_file_position`, `set_file_position`),
/// `palette2` and the display menu (`get_file_list`).
pub trait VirtualSDApi: Send + Sync + 'static {
    /// `VirtualSD.get_file_list(check_subdirs)`: `(relative path, size)`.
    fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>>;
    /// `VirtualSD.is_active()`.
    fn is_active(&self) -> bool;
    /// `VirtualSD.do_pause()`.
    /// Pauses the reactor.
    fn do_pause(&self);
    /// `VirtualSD.do_resume()`; raises `SD busy` while a print runs.
    fn do_resume(&self) -> KResult<()>;
    /// `VirtualSD.do_cancel()`.
    /// Pauses the reactor.
    fn do_cancel(&self) -> KResult<()>;
    /// `VirtualSD.is_cmd_from_sd()`.
    fn is_cmd_from_sd(&self) -> bool;
    /// `VirtualSD.get_file_position()` (`next_file_position`).
    fn get_file_position(&self) -> i64;
    /// `VirtualSD.set_file_position(pos)`.
    fn set_file_position(&self, pos: i64);
}

// ---------------------------------------------------------------------------
// Python exception text helpers
// ---------------------------------------------------------------------------

/// A Python exception as `logging.exception` would print its last line and
/// as it would surface when nothing catches it.
#[derive(Clone, Debug)]
struct PyExc {
    class: &'static str,
    msg: String,
}

impl PyExc {
    fn new(class: &'static str, msg: impl Into<String>) -> PyExc {
        PyExc { class, msg: msg.into() }
    }

    /// The traceback's final line.
    fn text(&self) -> String {
        format!("{}: {}", self.class, self.msg)
    }

    fn into_error(self) -> KlippyError {
        KlippyError::Internal { class_name: self.class.to_string(), message: self.msg }
    }
}

/// `OSError` subclass and `[Errno N] strerror` text for an `io::Error`;
/// `filename` is the path CPython appends (`: 'path'`).
fn os_error(e: &std::io::Error, filename: Option<&str>) -> PyExc {
    let Some(errno) = e.raw_os_error() else {
        return PyExc::new("OSError", e.to_string());
    };
    let class = match errno {
        1 | 13 => "PermissionError",
        2 => "FileNotFoundError",
        17 => "FileExistsError",
        20 => "NotADirectoryError",
        21 => "IsADirectoryError",
        _ => "OSError",
    };
    let full = e.to_string();
    let suffix = format!(" (os error {errno})");
    let strerror = full.strip_suffix(&suffix).unwrap_or(&full);
    let msg = match filename {
        Some(f) => format!("[Errno {errno}] {strerror}: {}", str_repr(f)),
        None => format!("[Errno {errno}] {strerror}"),
    };
    PyExc::new(class, msg)
}

fn attribute_error_none(attr: &str) -> PyExc {
    PyExc::new("AttributeError", format!("'NoneType' object has no attribute '{attr}'"))
}

fn error_text(e: &KlippyError) -> String {
    e.exception_line()
}

// ---------------------------------------------------------------------------
// Small Python string and path helpers
// ---------------------------------------------------------------------------

/// `str.isspace` for one character (`\x1c`-`\x1f` included).
fn is_py_space(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{1f}')
}

/// `s.strip()`.
fn py_strip(s: &str) -> &str {
    s.trim_matches(is_py_space)
}

/// `"..." % args` with CPython formatting.
fn pf(fmt: &str, args: &[PctValue]) -> String {
    percent_format(fmt, args).unwrap_or_else(|e| e.display())
}

/// `os.path.join(a, b)` for two components.
fn path_join(a: &str, b: &str) -> String {
    if b.starts_with('/') {
        b.to_string()
    } else if a.is_empty() || a.ends_with('/') {
        format!("{a}{b}")
    } else {
        format!("{a}/{b}")
    }
}

/// `os.path.normpath(path)` (`posixpath.py`, CPython 3.8).
fn normpath(path: &str) -> String {
    if path.is_empty() {
        return ".".to_string();
    }
    let mut initial_slashes = if path.starts_with('/') { 1 } else { 0 };
    if initial_slashes == 1 && path.starts_with("//") && !path.starts_with("///") {
        initial_slashes = 2;
    }
    let mut new_comps: Vec<&str> = Vec::new();
    for comp in path.split('/') {
        if comp.is_empty() || comp == "." {
            continue;
        }
        if comp != ".."
            || (initial_slashes == 0 && new_comps.is_empty())
            || new_comps.last() == Some(&"..")
        {
            new_comps.push(comp);
        } else if !new_comps.is_empty() {
            new_comps.pop();
        }
    }
    let mut out = "/".repeat(initial_slashes);
    out.push_str(&new_comps.join("/"));
    if out.is_empty() {
        ".".to_string()
    } else {
        out
    }
}

/// `os.path.expanduser(path)` (`posixpath.py`, CPython 3.8).
fn expanduser(path: &str) -> String {
    if !path.starts_with('~') {
        return path.to_string();
    }
    let i = path[1..].find('/').map_or(path.len(), |j| j + 1);
    let userhome = if i == 1 {
        match std::env::var_os("HOME") {
            Some(home) => home.to_string_lossy().into_owned(),
            None => match pw_dir_of_uid() {
                Some(dir) => dir,
                None => return path.to_string(),
            },
        }
    } else {
        match pw_dir_of_name(&path[1..i]) {
            Some(dir) => dir,
            None => return path.to_string(),
        }
    };
    let userhome = userhome.trim_end_matches('/');
    let joined = format!("{userhome}{}", &path[i..]);
    if joined.is_empty() {
        "/".to_string()
    } else {
        joined
    }
}

#[repr(C)]
struct Passwd {
    pw_name: *const std::os::raw::c_char,
    pw_passwd: *const std::os::raw::c_char,
    pw_uid: u32,
    pw_gid: u32,
    pw_gecos: *const std::os::raw::c_char,
    pw_dir: *const std::os::raw::c_char,
    pw_shell: *const std::os::raw::c_char,
}

// SAFETY: glibc password database lookups; the returned record is read
// before any other call that could overwrite it.
unsafe extern "C" {
    fn getpwnam(name: *const std::os::raw::c_char) -> *mut Passwd;
    fn getpwuid(uid: u32) -> *mut Passwd;
    fn getuid() -> u32;
}

/// `pw_dir` of a `struct passwd*` (`None` for a null record).
///
/// # Safety
/// `pw` must be null or a valid pointer returned by `getpwnam`/`getpwuid`.
unsafe fn pw_dir(pw: *mut Passwd) -> Option<String> {
    if pw.is_null() {
        return None;
    }
    let dir = unsafe { (*pw).pw_dir };
    if dir.is_null() {
        return None;
    }
    Some(unsafe { std::ffi::CStr::from_ptr(dir) }.to_string_lossy().into_owned())
}

/// `pwd.getpwuid(os.getuid()).pw_dir`.
fn pw_dir_of_uid() -> Option<String> {
    unsafe { pw_dir(getpwuid(getuid())) }
}

/// `pwd.getpwnam(name).pw_dir`.
fn pw_dir_of_name(name: &str) -> Option<String> {
    let name = std::ffi::CString::new(name).ok()?;
    unsafe { pw_dir(getpwnam(name.as_ptr())) }
}

// ---------------------------------------------------------------------------
// TextIOWrapper emulation
// ---------------------------------------------------------------------------

/// `io.open(fname, 'r', newline='')`: a text file read through
/// `TextIOWrapper.read(n)` / `seek(position)` with the UTF-8 codec.
struct TextFile {
    /// `f.name`.
    name: String,
    file: File,
    /// `_decoded_chars` and `_decoded_chars_used`.
    decoded: Vec<char>,
    used: usize,
    /// Incomplete trailing UTF-8 sequence kept by the incremental decoder.
    utf8_pending: Vec<u8>,
    /// `IncrementalNewlineDecoder.pendingcr`.
    pending_cr: bool,
    /// `_b2cratio`.
    b2cratio: f64,
}

/// `UnicodeDecodeError` text for the bytes of `buf` that `from_utf8` rejected.
fn utf8_error(buf: &[u8], start: usize, len: Option<usize>) -> PyExc {
    // `len == None` is an incomplete sequence at the very end of the input.
    let (end, reason) = match len {
        None => (buf.len(), "unexpected end of data"),
        Some(l) => {
            let b = buf[start];
            let reason = if (0x80..=0xC1).contains(&b) || b >= 0xF5 {
                "invalid start byte"
            } else {
                "invalid continuation byte"
            };
            (start + l, reason)
        }
    };
    let msg = if end - start == 1 {
        format!(
            "'utf-8' codec can't decode byte 0x{:02x} in position {start}: {reason}",
            buf[start]
        )
    } else {
        format!(
            "'utf-8' codec can't decode bytes in position {start}-{}: {reason}",
            end - 1
        )
    };
    PyExc::new("UnicodeDecodeError", msg)
}

impl TextFile {
    /// `f = io.open(fname, 'r', newline=''); f.seek(0, os.SEEK_END);
    /// fsize = f.tell(); f.seek(0)`.
    fn open(path: &str) -> Result<(TextFile, i64), PyExc> {
        let file = File::open(path).map_err(|e| os_error(&e, Some(path)))?;
        let meta = file.metadata().map_err(|e| os_error(&e, Some(path)))?;
        if meta.is_dir() {
            return Err(PyExc::new(
                "IsADirectoryError",
                format!("[Errno 21] Is a directory: {}", str_repr(path)),
            ));
        }
        let mut tf = TextFile {
            name: path.to_string(),
            file,
            decoded: Vec::new(),
            used: 0,
            utf8_pending: Vec::new(),
            pending_cr: false,
            b2cratio: 0.0,
        };
        let size = tf.file.seek(SeekFrom::End(0)).map_err(|e| os_error(&e, None))?;
        tf.reset_decoder();
        tf.seek(0)?;
        Ok((tf, size as i64))
    }

    fn reset_decoder(&mut self) {
        self.decoded.clear();
        self.used = 0;
        self.utf8_pending.clear();
        self.pending_cr = false;
    }

    /// `f.seek(position)` with a plain byte-offset cookie.
    fn seek(&mut self, pos: i64) -> Result<(), PyExc> {
        if pos < 0 {
            return Err(PyExc::new("ValueError", format!("negative seek position {pos}")));
        }
        self.file.seek(SeekFrom::Start(pos as u64)).map_err(|e| os_error(&e, None))?;
        self.reset_decoder();
        Ok(())
    }

    /// `textiowrapper_get_decoded_chars(n)`.
    fn take_decoded(&mut self, n: usize) -> String {
        let end = (self.used + n).min(self.decoded.len());
        let out: String = self.decoded[self.used..end].iter().collect();
        self.used = end;
        out
    }

    /// `decoder.decode(chunk, final)` of the `IncrementalNewlineDecoder`
    /// (`translate=False`) over the incremental UTF-8 decoder.
    fn decode(&mut self, chunk: &[u8], is_final: bool) -> Result<Vec<char>, PyExc> {
        let mut buf = std::mem::take(&mut self.utf8_pending);
        buf.extend_from_slice(chunk);
        let text: String = match std::str::from_utf8(&buf) {
            Ok(s) => s.to_string(),
            Err(e) => {
                let valid = e.valid_up_to();
                match e.error_len() {
                    Some(l) => return Err(utf8_error(&buf, valid, Some(l))),
                    None => {
                        if is_final {
                            return Err(utf8_error(&buf, valid, None));
                        }
                        self.utf8_pending = buf[valid..].to_vec();
                        // The prefix was validated by `from_utf8`.
                        String::from_utf8_lossy(&buf[..valid]).into_owned()
                    }
                }
            }
        };
        let mut output: Vec<char> = text.chars().collect();
        if self.pending_cr && (is_final || !output.is_empty()) {
            output.insert(0, '\r');
            self.pending_cr = false;
        }
        // Retain a trailing `\r` until the next chunk, even when not translating.
        if !is_final && output.last() == Some(&'\r') {
            output.pop();
            self.pending_cr = true;
        }
        Ok(output)
    }

    /// `textiowrapper_read_chunk(size_hint)`; `Ok(false)` at end of file.
    fn read_chunk(&mut self, size_hint: usize) -> Result<bool, PyExc> {
        let hint = kos::cconv::f64_to_i64(self.b2cratio.max(1.0) * size_hint as f64);
        let chunk_size = TEXT_CHUNK_SIZE.max(hint.max(0) as usize);
        let mut raw = vec![0u8; chunk_size];
        let nbytes = loop {
            match self.file.read(&mut raw) {
                Ok(n) => break n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(os_error(&e, None)),
            }
        };
        raw.truncate(nbytes);
        let eof = nbytes == 0;
        let decoded = self.decode(&raw, eof)?;
        let nchars = decoded.len();
        self.decoded = decoded;
        self.used = 0;
        self.b2cratio = if nchars > 0 { nbytes as f64 / nchars as f64 } else { 0.0 };
        Ok(!eof || nchars > 0)
    }

    /// `f.read(n)`: up to `n` characters.
    fn read(&mut self, n: usize) -> Result<String, PyExc> {
        let mut result = self.take_decoded(n);
        let mut remaining = n - result.chars().count();
        while remaining > 0 {
            if !self.read_chunk(remaining)? {
                break;
            }
            let part = self.take_decoded(remaining);
            remaining -= part.chars().count();
            result.push_str(&part);
        }
        Ok(result)
    }
}

// ---------------------------------------------------------------------------
// Debug timing rings
// ---------------------------------------------------------------------------

/// `DebugSDChunk`: producer timing for one sdcard read chunk.
#[derive(Clone, Debug)]
struct DebugSDChunk {
    seq: i64,
    start: f64,
    position: i64,
    nbytes: i64,
    read_time: f64,
    yield_time: f64,
    line_count: i64,
    cmd_count: i64,
    cmd_time: f64,
    peak_cmd_time: f64,
    slow_cmd_time: f64,
    slow_cmd_name: String,
    mutex_pauses: i64,
    mutex_time: f64,
    total_time: f64,
    state: &'static str,
}

impl DebugSDChunk {
    fn new(seq: i64, start: f64, position: i64) -> DebugSDChunk {
        DebugSDChunk {
            seq,
            start,
            position,
            nbytes: 0,
            read_time: 0.,
            yield_time: 0.,
            line_count: 0,
            cmd_count: 0,
            cmd_time: 0.,
            peak_cmd_time: 0.,
            slow_cmd_time: 0.,
            slow_cmd_name: "-".to_string(),
            mutex_pauses: 0,
            mutex_time: 0.,
            total_time: 0.,
            state: "active",
        }
    }

    /// `DebugSDChunk.desc()`.
    fn desc(&self) -> String {
        pf(
            concat!(
                "seq=%d at=%.6f pos=%d bytes=%d lines=%d read=%.6f",
                " yield=%.6f cmds=%d cmd_time=%.6f peak_cmd=%.6f",
                " slow_cmd=%.6f slow_cmd_name=%s mutex_pauses=%d",
                " mutex_time=%.6f total=%.6f state=%s"
            ),
            &[
                PctValue::Int(PyInt::from_i64(self.seq)),
                PctValue::Float(self.start),
                PctValue::Int(PyInt::from_i64(self.position)),
                PctValue::Int(PyInt::from_i64(self.nbytes)),
                PctValue::Int(PyInt::from_i64(self.line_count)),
                PctValue::Float(self.read_time),
                PctValue::Float(self.yield_time),
                PctValue::Int(PyInt::from_i64(self.cmd_count)),
                PctValue::Float(self.cmd_time),
                PctValue::Float(self.peak_cmd_time),
                PctValue::Float(self.slow_cmd_time),
                PctValue::Str(self.slow_cmd_name.clone()),
                PctValue::Int(PyInt::from_i64(self.mutex_pauses)),
                PctValue::Float(self.mutex_time),
                PctValue::Float(self.total_time),
                PctValue::Str(self.state.to_string()),
            ],
        )
    }
}

/// A chunk shared by the work handler's local and the rings, like the
/// Python object referenced from both.
type ChunkRef = Arc<GCell<DebugSDChunk>>;

/// `debug_cmd_name(line)`: short command name; the line itself is never logged.
pub fn debug_cmd_name(line: &str) -> String {
    let mut name = py_strip(line);
    if let Some(pos) = name.find(' ') {
        if pos > 0 {
            name = &name[..pos];
        }
    }
    if name.chars().count() > DEBUG_CMD_NAME_MAX {
        let head: String = name.chars().take(DEBUG_CMD_NAME_MAX).collect();
        format!("{head}..")
    } else {
        name.to_string()
    }
}

/// `deque(maxlen=n).append(item)`.
fn ring_push<T>(ring: &mut VecDeque<T>, cap: usize, item: T) {
    ring.push_back(item);
    while ring.len() > cap {
        ring.pop_front();
    }
}

/// `list(deque)[-DEBUG_DUMP_ENTRIES:]`.
fn ring_tail<T: Clone>(ring: &VecDeque<T>) -> Vec<T> {
    let skip = ring.len().saturating_sub(DEBUG_DUMP_ENTRIES);
    ring.iter().skip(skip).cloned().collect()
}

#[derive(Clone)]
struct SlowCmd {
    at: f64,
    dur: f64,
    pos: i64,
    name: String,
    ok: bool,
}

#[derive(Clone)]
struct SlowYield {
    at: f64,
    dur: f64,
    kind: &'static str,
    pos: i64,
}

// ---------------------------------------------------------------------------
// VirtualSD
// ---------------------------------------------------------------------------

/// Mutable `self.*` fields of the Python class.
struct State {
    current_file: Option<TextFile>,
    file_position: i64,
    file_size: i64,
    must_pause_work: bool,
    cmd_from_sd: bool,
    next_file_position: i64,
    work_timer: Option<Timer>,
    debug_chunk_seq: i64,
    debug_chunk_count: i64,
    debug_active_chunk: Option<ChunkRef>,
    debug_chunks: VecDeque<ChunkRef>,
    debug_slow_chunks: VecDeque<ChunkRef>,
    debug_slow_cmds: VecDeque<SlowCmd>,
    debug_slow_yields: VecDeque<SlowYield>,
    debug_cmd_count: i64,
    debug_cmd_time: f64,
    debug_read_time: f64,
    debug_yield_time: f64,
    debug_worst_yield: f64,
    debug_worst_cmd_time: f64,
    debug_worst_cmd_name: String,
}

struct Inner {
    printer: Printer,
    reactor: Reactor,
    gcode: GCode,
    print_stats: Arc<dyn PrintStatsApi>,
    on_error_gcode: TemplateWrapper,
    sdcard_dirname: String,
    state: GCell<State>,
}

/// The `VirtualSD` printer object (`virtual_sdcard.py:62`).
pub struct VirtualSD {
    inner: Arc<Inner>,
}

impl VirtualSD {
    /// `VirtualSD.__init__(config)` (`virtual_sdcard.py:63-106`).
    pub fn new(config: &Config) -> KResult<VirtualSD> {
        let printer = config.get_printer();
        // sdcard state
        let sd = config
            .get("path", GetStrOpts::default())?
            .ok_or_else(|| KlippyError::config("Option 'path' must be specified"))?;
        let sdcard_dirname = normpath(&expanduser(&sd));
        // Print Stat Tracking
        let print_stats = load_interface::<dyn PrintStatsApi>(
            &*printer,
            config,
            "print_stats",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        // Work timer
        let reactor = printer.get_reactor();
        // Error handling
        let gcode_macro = load_interface::<dyn PrinterGCodeMacroApi>(
            &*printer,
            config,
            "gcode_macro",
            LookupDefault::Raise,
        )?
        .expect("Raise never yields None");
        let on_error_gcode =
            gcode_macro.load_template(config, "on_error_gcode", Some(DEFAULT_ERROR_GCODE))?;
        // Register commands
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise never yields None");

        let inner = Arc::new(Inner {
            printer: printer.clone(),
            reactor,
            gcode,
            print_stats,
            on_error_gcode,
            sdcard_dirname,
            state: GCell::new(State {
                current_file: None,
                file_position: 0,
                file_size: 0,
                must_pause_work: false,
                cmd_from_sd: false,
                next_file_position: 0,
                work_timer: None,
                debug_chunk_seq: 0,
                debug_chunk_count: 0,
                debug_active_chunk: None,
                debug_chunks: VecDeque::new(),
                debug_slow_chunks: VecDeque::new(),
                debug_slow_cmds: VecDeque::new(),
                debug_slow_yields: VecDeque::new(),
                debug_cmd_count: 0,
                debug_cmd_time: 0.,
                debug_read_time: 0.,
                debug_yield_time: 0.,
                debug_worst_yield: 0.,
                debug_worst_cmd_time: 0.,
                debug_worst_cmd_name: "-".to_string(),
            }),
        });

        type Handler = fn(&Arc<Inner>, &GCmd) -> KResult<()>;
        let commands: [(&str, Handler); 7] = [
            ("M20", Inner::cmd_m20),
            ("M21", Inner::cmd_m21),
            ("M23", Inner::cmd_m23),
            ("M24", Inner::cmd_m24),
            ("M25", Inner::cmd_m25),
            ("M26", Inner::cmd_m26),
            ("M27", Inner::cmd_m27),
        ];
        for (name, handler) in commands {
            inner.register(name, handler, None)?;
        }
        for name in ["M28", "M29", "M30"] {
            inner.register(name, Inner::cmd_error, None)?;
        }
        inner.register(
            "SDCARD_RESET_FILE",
            Inner::cmd_sdcard_reset_file,
            Some(CMD_SDCARD_RESET_FILE_HELP),
        )?;
        inner.register(
            "SDCARD_PRINT_FILE",
            Inner::cmd_sdcard_print_file,
            Some(CMD_SDCARD_PRINT_FILE_HELP),
        )?;
        let shutdown_inner = Arc::clone(&inner);
        printer.register_event_handler(
            EV_KLIPPY_ANALYZE_SHUTDOWN,
            Box::new(move |_params: &[EventArg]| {
                shutdown_inner.handle_analyze_shutdown();
                Ok(PyValue::None)
            }),
        );
        Ok(VirtualSD { inner })
    }

    /// `VirtualSD.progress()`.
    pub fn progress(&self) -> f64 {
        self.inner.progress()
    }

    /// `VirtualSD.file_path()`.
    pub fn file_path(&self) -> Option<String> {
        self.inner.file_path()
    }
}

impl VirtualSDApi for VirtualSD {
    fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        self.inner.get_file_list(check_subdirs)
    }
    fn is_active(&self) -> bool {
        self.inner.is_active()
    }
    fn do_pause(&self) {
        self.inner.do_pause()
    }
    fn do_resume(&self) -> KResult<()> {
        self.inner.do_resume()
    }
    fn do_cancel(&self) -> KResult<()> {
        self.inner.do_cancel()
    }
    fn is_cmd_from_sd(&self) -> bool {
        self.inner.state.borrow().cmd_from_sd
    }
    fn get_file_position(&self) -> i64 {
        self.inner.state.borrow().next_file_position
    }
    fn set_file_position(&self, pos: i64) {
        self.inner.state.borrow_mut().next_file_position = pos;
    }
}

impl PrinterObject for VirtualSD {
    klippy_api::printer_object_base!();
    klippy_api::printer_interfaces!(dyn VirtualSDApi);

    /// `VirtualSD.get_status(eventtime)` (`virtual_sdcard.py:244`).
    fn get_status(&self, _eventtime: f64) -> KResult<Option<PyValue>> {
        let inner = &self.inner;
        let mut status: IndexMap<PyValue, PyValue> = IndexMap::new();
        status.insert(
            PyValue::Str("file_path".to_string()),
            match inner.file_path() {
                Some(p) => PyValue::Str(p),
                None => PyValue::None,
            },
        );
        status.insert(PyValue::Str("progress".to_string()), PyValue::Float(inner.progress()));
        status.insert(PyValue::Str("is_active".to_string()), PyValue::Bool(inner.is_active()));
        let (position, size) = {
            let st = inner.state.borrow();
            (st.file_position, st.file_size)
        };
        status.insert(
            PyValue::Str("file_position".to_string()),
            PyValue::Int(PyInt::from_i64(position)),
        );
        status.insert(
            PyValue::Str("file_size".to_string()),
            PyValue::Int(PyInt::from_i64(size)),
        );
        Ok(Some(PyValue::Dict(status)))
    }

    /// `VirtualSD.stats(eventtime)` (`virtual_sdcard.py:215`).
    fn stats(&self, _eventtime: f64) -> Option<(bool, String)> {
        let st = self.inner.state.borrow();
        if st.work_timer.is_none() {
            return Some((false, String::new()));
        }
        Some((true, format!("sd_pos={}", st.file_position)))
    }
}

impl Inner {
    fn now(&self) -> f64 {
        self.reactor.monotonic()
    }

    /// `self.gcode.register_command(cmd, handler[, desc=...])`.
    fn register(
        self: &Arc<Self>,
        cmd: &str,
        handler: fn(&Arc<Inner>, &GCmd) -> KResult<()>,
        desc: Option<&str>,
    ) -> KResult<()> {
        let inner = Arc::clone(self);
        let func: GCodeCommandHandler = Box::new(move |gcmd: &GCmd| handler(&inner, gcmd));
        self.gcode.register_command(
            cmd,
            Some(func),
            RegisterCommandOpts { when_not_ready: false, desc: desc.map(|d| d.to_string()) },
        )?;
        Ok(())
    }

    // -- klippy:analyze_shutdown ------------------------------------------

    /// `_handle_analyze_shutdown(msg, details)` (`virtual_sdcard.py:107`).
    fn handle_analyze_shutdown(&self) {
        // The Python wraps the dump in a try/except that only guards against
        // exceptions from the string formatting; there is nothing to catch.
        self.debug_dump_timing();
        let active = self.state.borrow().work_timer.is_some();
        if !active {
            return;
        }
        let (readpos, readcount, file_position, read) = {
            let mut st = self.state.borrow_mut();
            st.must_pause_work = true;
            let file_position = st.file_position;
            let readpos = (file_position - 1024).max(0);
            let readcount = file_position - readpos;
            let read = match st.current_file.as_mut() {
                None => Err(attribute_error_none("seek")),
                Some(f) => f.seek(readpos).and_then(|_| f.read((readcount + 128) as usize)),
            };
            (readpos, readcount, file_position, read)
        };
        let data = match read {
            Ok(d) => d,
            Err(e) => {
                log::exception("virtual_sdcard shutdown read", &e.text());
                return;
            }
        };
        let head: String = data.chars().take(readcount as usize).collect();
        let tail: String = data.chars().skip(readcount as usize).collect();
        log::info(&pf(
            "Virtual sdcard (%d): %s\nUpcoming (%d): %s",
            &[
                PctValue::Int(PyInt::from_i64(readpos)),
                PctValue::Str(str_repr(&head)),
                PctValue::Int(PyInt::from_i64(file_position)),
                PctValue::Str(str_repr(&tail)),
            ],
        ));
    }

    /// `_debug_dump_timing()` (`virtual_sdcard.py:127`).
    fn debug_dump_timing(&self) {
        let (summary, pending, slow_chunks, chunks, slow_cmds, slow_yields) = {
            let st = self.state.borrow();
            let summary = pf(
                concat!(
                    "[DEBUG-motion] version=host-v4 sdcard timing chunks=%d cmds=%d",
                    " cmd_total=%.6f worst_cmd=%.6f worst_cmd_name=%s read_total=%.6f",
                    " yield_total=%.6f worst_yield_over=%.6f pending=%s pos=%d/%d",
                    " cmd_from_sd=%s must_pause=%s active=%s"
                ),
                &[
                    PctValue::Int(PyInt::from_i64(st.debug_chunk_count)),
                    PctValue::Int(PyInt::from_i64(st.debug_cmd_count)),
                    PctValue::Float(st.debug_cmd_time),
                    PctValue::Float(st.debug_worst_cmd_time),
                    PctValue::Str(st.debug_worst_cmd_name.clone()),
                    PctValue::Float(st.debug_read_time),
                    PctValue::Float(st.debug_yield_time),
                    PctValue::Float(st.debug_worst_yield),
                    PctValue::Bool(st.debug_active_chunk.is_some()),
                    PctValue::Int(PyInt::from_i64(st.file_position)),
                    PctValue::Int(PyInt::from_i64(st.file_size)),
                    PctValue::Bool(st.cmd_from_sd),
                    PctValue::Bool(st.must_pause_work),
                    PctValue::Bool(st.work_timer.is_some()),
                ],
            );
            (
                summary,
                st.debug_active_chunk.clone(),
                ring_tail(&st.debug_slow_chunks),
                ring_tail(&st.debug_chunks),
                ring_tail(&st.debug_slow_cmds),
                ring_tail(&st.debug_slow_yields),
            )
        };
        log::error(&summary);
        if let Some(chunk) = pending {
            log::error(&format!("[DEBUG-motion] sdcard chunk pending {}", chunk.borrow().desc()));
        }
        for c in slow_chunks {
            log::error(&format!("[DEBUG-motion] sdcard chunk slow {}", c.borrow().desc()));
        }
        for c in chunks {
            log::error(&format!("[DEBUG-motion] sdcard chunk recent {}", c.borrow().desc()));
        }
        for c in slow_cmds {
            log::error(&pf(
                "[DEBUG-motion] sdcard cmd at=%.6f dur=%.6f pos=%d name=%s ok=%s",
                &[
                    PctValue::Float(c.at),
                    PctValue::Float(c.dur),
                    PctValue::Int(PyInt::from_i64(c.pos)),
                    PctValue::Str(c.name),
                    PctValue::Bool(c.ok),
                ],
            ));
        }
        for y in slow_yields {
            log::error(&pf(
                "[DEBUG-motion] sdcard yield at=%.6f dur=%.6f kind=%s pos=%d",
                &[
                    PctValue::Float(y.at),
                    PctValue::Float(y.dur),
                    PctValue::Str(y.kind.to_string()),
                    PctValue::Int(PyInt::from_i64(y.pos)),
                ],
            ));
        }
    }

    // -- debug rings ------------------------------------------------------

    /// `_debug_start_chunk(t_start, t_end, data)`; `data` is the character
    /// count of the text read, `None` when the read failed.
    fn debug_start_chunk(&self, t_start: f64, t_end: f64, data_len: Option<usize>) -> ChunkRef {
        let chunk = {
            let mut st = self.state.borrow_mut();
            st.debug_chunk_seq += 1;
            let mut chunk = DebugSDChunk::new(st.debug_chunk_seq, t_start, st.file_position);
            chunk.read_time = t_end - t_start;
            st.debug_read_time += chunk.read_time;
            let chunk: ChunkRef = Arc::new(GCell::new(chunk));
            st.debug_active_chunk = Some(chunk.clone());
            chunk
        };
        match data_len {
            None => {
                chunk.borrow_mut().nbytes = -1;
                self.debug_close_chunk(t_end, "read_error");
            }
            Some(n) => chunk.borrow_mut().nbytes = n as i64,
        }
        chunk
    }

    /// `_debug_close_chunk(t_end, state)`.
    fn debug_close_chunk(&self, t_end: f64, state: &'static str) {
        let mut st = self.state.borrow_mut();
        let Some(chunk) = st.debug_active_chunk.take() else {
            return;
        };
        let slow = {
            let mut c = chunk.borrow_mut();
            c.state = state;
            c.total_time = t_end - c.start;
            state != "next_read"
                || c.slow_cmd_time > DEBUG_SLOW_CMD_TIME
                || c.yield_time > DEBUG_SLOW_YIELD_TIME
        };
        st.debug_chunk_count += 1;
        ring_push(&mut st.debug_chunks, DEBUG_CHUNK_RING_SIZE, chunk.clone());
        if slow {
            ring_push(&mut st.debug_slow_chunks, DEBUG_SLOW_RING_SIZE, chunk);
        }
    }

    /// `_debug_note_cmd(chunk, line, t_start, t_end, ok)`.
    fn debug_note_cmd(
        &self,
        chunk: Option<&ChunkRef>,
        line: &str,
        t_start: f64,
        t_end: f64,
        ok: bool,
    ) {
        let dur = t_end - t_start;
        let mut st = self.state.borrow_mut();
        st.debug_cmd_count += 1;
        st.debug_cmd_time += dur;
        if let Some(chunk) = chunk {
            let mut c = chunk.borrow_mut();
            c.cmd_count += 1;
            c.cmd_time += dur;
            if dur > c.peak_cmd_time {
                c.peak_cmd_time = dur;
            }
        }
        if dur <= DEBUG_SLOW_CMD_TIME && ok {
            return;
        }
        // Slow or failed command - resolve a short name and keep the timing
        let name = debug_cmd_name(line);
        if let Some(chunk) = chunk {
            let mut c = chunk.borrow_mut();
            if dur > c.slow_cmd_time {
                c.slow_cmd_time = dur;
                c.slow_cmd_name = name.clone();
            }
        }
        if dur > st.debug_worst_cmd_time {
            st.debug_worst_cmd_time = dur;
            st.debug_worst_cmd_name = name.clone();
        }
        let pos = st.file_position;
        ring_push(
            &mut st.debug_slow_cmds,
            DEBUG_SLOW_RING_SIZE,
            SlowCmd { at: t_start, dur, pos, name, ok },
        );
    }

    /// `_debug_note_yield(chunk, t_start, dur, kind, expect)`.
    fn debug_note_yield(
        &self,
        chunk: Option<&ChunkRef>,
        t_start: f64,
        dur: f64,
        kind: &'static str,
        expect: f64,
    ) {
        let mut st = self.state.borrow_mut();
        st.debug_yield_time += dur;
        if let Some(chunk) = chunk {
            let mut c = chunk.borrow_mut();
            if kind == "read" {
                c.yield_time += dur;
            } else {
                c.mutex_pauses += 1;
                c.mutex_time += dur;
            }
        }
        let over = dur - expect;
        if over > st.debug_worst_yield {
            st.debug_worst_yield = over;
        }
        if over > DEBUG_SLOW_YIELD_TIME {
            let pos = st.file_position;
            ring_push(
                &mut st.debug_slow_yields,
                DEBUG_SLOW_RING_SIZE,
                SlowYield { at: t_start, dur, kind, pos },
            );
        }
    }

    // -- file list and status ---------------------------------------------

    /// `get_file_list(check_subdirs)` (`virtual_sdcard.py:219`).
    fn get_file_list(&self, check_subdirs: bool) -> KResult<Vec<(String, u64)>> {
        if check_subdirs {
            let mut flist: Vec<(String, u64)> = Vec::new();
            self.walk_files(&self.sdcard_dirname, &mut flist).map_err(PyExc::into_error)?;
            flist.sort_by_cached_key(|f| f.0.to_lowercase());
            return Ok(flist);
        }
        match self.list_top_level() {
            Ok(list) => Ok(list),
            Err(e) => {
                log::exception("virtual_sdcard get_file_list", &e.text());
                Err(KlippyError::command("Unable to get file list"))
            }
        }
    }

    /// The non-recursive branch: `os.listdir` plus `isfile` / `getsize`.
    fn list_top_level(&self) -> Result<Vec<(String, u64)>, PyExc> {
        let dname = &self.sdcard_dirname;
        let rd = std::fs::read_dir(dname).map_err(|e| os_error(&e, Some(dname)))?;
        let mut filenames: Vec<String> = Vec::new();
        for entry in rd {
            let entry = entry.map_err(|e| os_error(&e, Some(dname)))?;
            filenames.push(entry.file_name().to_string_lossy().into_owned());
        }
        filenames.sort_by_cached_key(|n| n.to_lowercase());
        let mut out = Vec::new();
        for fname in filenames {
            if fname.starts_with('.') {
                continue;
            }
            let full = path_join(dname, &fname);
            let is_file = std::fs::metadata(&full).map(|m| m.is_file()).unwrap_or(false);
            if !is_file {
                continue;
            }
            let size = std::fs::metadata(&full).map_err(|e| os_error(&e, Some(&full)))?.len();
            out.push((fname, size));
        }
        Ok(out)
    }

    /// One `os.walk(top, followlinks=True)` level and its recursion,
    /// appending the g-code files found.
    fn walk_files(&self, top: &str, flist: &mut Vec<(String, u64)>) -> Result<(), PyExc> {
        // `scandir` errors are ignored by `os.walk` (no `onerror`).
        let Ok(rd) = std::fs::read_dir(top) else {
            return Ok(());
        };
        let mut dirs: Vec<String> = Vec::new();
        let mut files: Vec<String> = Vec::new();
        for entry in rd {
            let Ok(entry) = entry else {
                break;
            };
            let name = entry.file_name().to_string_lossy().into_owned();
            let is_dir = std::fs::metadata(entry.path()).map(|m| m.is_dir()).unwrap_or(false);
            if is_dir {
                dirs.push(name);
            } else {
                files.push(name);
            }
        }
        for name in &files {
            let ext = match name.rfind('.') {
                Some(i) => &name[i + 1..],
                None => name.as_str(),
            };
            if !VALID_GCODE_EXTS.contains(&ext) {
                continue;
            }
            let full_path = path_join(top, name);
            let r_path: String =
                full_path.chars().skip(self.sdcard_dirname.chars().count() + 1).collect();
            let size = std::fs::metadata(&full_path)
                .map_err(|e| os_error(&e, Some(&full_path)))?
                .len();
            flist.push((r_path, size));
        }
        for dirname in &dirs {
            self.walk_files(&path_join(top, dirname), flist)?;
        }
        Ok(())
    }

    /// `file_path()`.
    fn file_path(&self) -> Option<String> {
        self.state.borrow().current_file.as_ref().map(|f| f.name.clone())
    }

    /// `progress()`.
    fn progress(&self) -> f64 {
        let st = self.state.borrow();
        if st.file_size != 0 {
            st.file_position as f64 / st.file_size as f64
        } else {
            0.
        }
    }

    /// `is_active()`.
    fn is_active(&self) -> bool {
        self.state.borrow().work_timer.is_some()
    }

    // -- pause / resume / cancel --------------------------------------------

    /// `do_pause()`.
    fn do_pause(&self) {
        let active = self.state.borrow().work_timer.is_some();
        if active {
            self.state.borrow_mut().must_pause_work = true;
            loop {
                {
                    let st = self.state.borrow();
                    if st.work_timer.is_none() || st.cmd_from_sd {
                        break;
                    }
                }
                self.reactor.pause(self.now() + 0.001);
            }
        }
    }

    /// `do_resume()`.
    fn do_resume(self: &Arc<Self>) -> KResult<()> {
        if self.state.borrow().work_timer.is_some() {
            return Err(KlippyError::command("SD busy"));
        }
        self.state.borrow_mut().must_pause_work = false;
        let inner = Arc::clone(self);
        let timer = self.reactor.register_timer(move |et| inner.work_handler(et), NOW);
        self.state.borrow_mut().work_timer = Some(timer);
        Ok(())
    }

    /// `self.current_file.close(); self.current_file = None`.
    fn close_current_file(&self) -> KResult<()> {
        match self.state.borrow_mut().current_file.take() {
            Some(_) => Ok(()),
            None => Err(attribute_error_none("close").into_error()),
        }
    }

    /// `do_cancel()`.
    fn do_cancel(&self) -> KResult<()> {
        if self.state.borrow().current_file.is_some() {
            self.do_pause();
            self.close_current_file()?;
            self.print_stats.note_cancel();
        }
        let mut st = self.state.borrow_mut();
        st.file_position = 0;
        st.file_size = 0;
        Ok(())
    }

    // -- G-code commands -----------------------------------------------------

    /// `cmd_error(gcmd)`.
    fn cmd_error(_this: &Arc<Self>, _gcmd: &GCmd) -> KResult<()> {
        Err(KlippyError::command("SD write not supported"))
    }

    /// `_reset_file()`.
    fn reset_file(&self) -> KResult<()> {
        if self.state.borrow().current_file.is_some() {
            self.do_pause();
            self.close_current_file()?;
        }
        {
            let mut st = self.state.borrow_mut();
            st.file_position = 0;
            st.file_size = 0;
        }
        self.print_stats.reset();
        self.printer.send_event(EV_VIRTUAL_SDCARD_RESET_FILE, &[])?;
        Ok(())
    }

    /// `cmd_SDCARD_RESET_FILE(gcmd)`.
    fn cmd_sdcard_reset_file(this: &Arc<Self>, _gcmd: &GCmd) -> KResult<()> {
        if this.state.borrow().cmd_from_sd {
            return Err(KlippyError::command("SDCARD_RESET_FILE cannot be run from the sdcard"));
        }
        this.reset_file()
    }

    /// `cmd_SDCARD_PRINT_FILE(gcmd)`.
    fn cmd_sdcard_print_file(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        if this.state.borrow().work_timer.is_some() {
            return Err(KlippyError::command("SD busy"));
        }
        this.reset_file()?;
        let filename = match gcmd.get("FILENAME", GetOpts::default())? {
            PyValue::Str(s) => s,
            other => other.str(),
        };
        // `filename[0]` raises IndexError on an empty value.
        let Some(first) = filename.chars().next() else {
            return Err(PyExc::new("IndexError", "string index out of range").into_error());
        };
        let filename = if first == '/' { filename[1..].to_string() } else { filename };
        this.load_file(gcmd, &filename, true)?;
        this.do_resume()
    }

    /// `cmd_M20(gcmd)`: list the SD card.
    fn cmd_m20(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let files = this.get_file_list(false)?;
        gcmd.respond_raw("Begin file list");
        for (fname, fsize) in files {
            gcmd.respond_raw(&format!("{fname} {fsize}"));
        }
        gcmd.respond_raw("End file list");
        Ok(())
    }

    /// `cmd_M21(gcmd)`: initialize the SD card.
    fn cmd_m21(_this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        gcmd.respond_raw("SD card ok");
        Ok(())
    }

    /// `cmd_M23(gcmd)`: select an SD file.
    fn cmd_m23(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        if this.state.borrow().work_timer.is_some() {
            return Err(KlippyError::command("SD busy"));
        }
        this.reset_file()?;
        let raw = gcmd.get_raw_command_parameters();
        let filename = py_strip(&raw);
        let filename = filename.strip_prefix('/').unwrap_or(filename);
        this.load_file(gcmd, filename, false)
    }

    /// `_load_file(gcmd, filename, check_subdirs)`.
    fn load_file(&self, gcmd: &GCmd, filename: &str, check_subdirs: bool) -> KResult<()> {
        let files = self.get_file_list(check_subdirs)?;
        let mut files_by_lower: IndexMap<String, &str> = IndexMap::new();
        for (fname, _) in &files {
            files_by_lower.insert(fname.to_lowercase(), fname.as_str());
        }
        let opened = (|| -> Result<(TextFile, i64), PyExc> {
            let mut fname = filename;
            if !files.iter().any(|f| f.0 == fname) {
                let key = fname.to_lowercase();
                fname = match files_by_lower.get(&key) {
                    Some(v) => *v,
                    None => return Err(PyExc::new("KeyError", str_repr(&key))),
                };
            }
            let fname = path_join(&self.sdcard_dirname, fname);
            TextFile::open(&fname)
        })();
        let (f, fsize) = match opened {
            Ok(v) => v,
            Err(e) => {
                log::exception("virtual_sdcard file open", &e.text());
                return Err(KlippyError::command("Unable to open file"));
            }
        };
        gcmd.respond_raw(&format!("File opened:{filename} Size:{fsize}"));
        gcmd.respond_raw("File selected");
        {
            let mut st = self.state.borrow_mut();
            st.current_file = Some(f);
            st.file_position = 0;
            st.file_size = fsize;
        }
        self.print_stats.set_current_file(filename);
        Ok(())
    }

    /// `cmd_M24(gcmd)`: start/resume an SD print.
    fn cmd_m24(this: &Arc<Self>, _gcmd: &GCmd) -> KResult<()> {
        this.do_resume()
    }

    /// `cmd_M25(gcmd)`: pause an SD print.
    fn cmd_m25(this: &Arc<Self>, _gcmd: &GCmd) -> KResult<()> {
        this.do_pause();
        Ok(())
    }

    /// `cmd_M26(gcmd)`: set the SD position.
    fn cmd_m26(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        if this.state.borrow().work_timer.is_some() {
            return Err(KlippyError::command("SD busy"));
        }
        let pos = gcmd
            .get_int("S", GetIntOpts { default: None, minval: Some(0), maxval: None })?
            .expect("required S parameter");
        this.state.borrow_mut().file_position = pos;
        Ok(())
    }

    /// `cmd_M27(gcmd)`: report the SD print status.
    fn cmd_m27(this: &Arc<Self>, gcmd: &GCmd) -> KResult<()> {
        let (printing, position, size) = {
            let st = this.state.borrow();
            (st.current_file.is_some(), st.file_position, st.file_size)
        };
        if !printing {
            gcmd.respond_raw("Not SD printing.");
            return Ok(());
        }
        gcmd.respond_raw(&format!("SD printing byte {position}/{size}"));
        Ok(())
    }

    // -- background work timer ------------------------------------------------

    /// `work_handler(eventtime)` (`virtual_sdcard.py:377`).
    fn work_handler(self: &Arc<Self>, _eventtime: f64) -> f64 {
        log::info(&format!(
            "Starting SD card print (position {})",
            self.state.borrow().file_position
        ));
        {
            let st = self.state.borrow();
            if let Some(timer) = st.work_timer.as_ref() {
                self.reactor.unregister_timer(timer);
            }
        }
        let seek = {
            let mut st = self.state.borrow_mut();
            let pos = st.file_position;
            match st.current_file.as_mut() {
                Some(f) => f.seek(pos),
                None => Err(attribute_error_none("seek")),
            }
        };
        if let Err(e) = seek {
            log::exception("virtual_sdcard seek", &e.text());
            self.state.borrow_mut().work_timer = None;
            return NEVER;
        }
        self.print_stats.note_start();
        let gcode_mutex = self.gcode.get_mutex();
        let mut partial_input = String::new();
        let mut lines: Vec<String> = Vec::new();
        let mut error_message: Option<String> = None;
        // [DEBUG-motion] v4 - bounded producer timing (no per-line logging)
        let mut chunk: Option<ChunkRef> = None;
        while !self.state.borrow().must_pause_work {
            if lines.is_empty() {
                // Read more data
                let t_read = self.now();
                if chunk.is_some() {
                    self.debug_close_chunk(t_read, "next_read");
                }
                let read = {
                    let mut st = self.state.borrow_mut();
                    match st.current_file.as_mut() {
                        Some(f) => f.read(8192),
                        None => Err(attribute_error_none("read")),
                    }
                };
                let data = match read {
                    Ok(d) => Some(d),
                    Err(e) => {
                        log::exception("virtual_sdcard read", &e.text());
                        None
                    }
                };
                chunk = Some(self.debug_start_chunk(
                    t_read,
                    self.now(),
                    data.as_ref().map(|d| d.chars().count()),
                ));
                let Some(data) = data else {
                    break;
                };
                if data.is_empty() {
                    // End of file
                    self.state.borrow_mut().current_file = None;
                    log::info("Finished SD card print");
                    self.gcode.respond_raw("Done printing file");
                    break;
                }
                let mut split: Vec<String> = data.split('\n').map(str::to_string).collect();
                split[0] = format!("{partial_input}{}", split[0]);
                partial_input = split.pop().unwrap_or_default();
                split.reverse();
                lines = split;
                if let Some(c) = &chunk {
                    c.borrow_mut().line_count = lines.len() as i64;
                }
                let t_yield = self.now();
                self.reactor.pause(NOW);
                self.debug_note_yield(chunk.as_ref(), t_yield, self.now() - t_yield, "read", 0.);
                continue;
            }
            // Pause if any other request is pending in the gcode class
            if gcode_mutex.test() {
                let t_pause = self.now();
                self.reactor.pause(t_pause + 0.050);
                self.debug_note_yield(chunk.as_ref(), t_pause, self.now() - t_pause, "mutex", 0.050);
                continue;
            }
            // Dispatch command
            let line = lines.pop().unwrap_or_default();
            let next_file_position = {
                let mut st = self.state.borrow_mut();
                st.cmd_from_sd = true;
                let next = st.file_position + line.len() as i64 + 1;
                st.next_file_position = next;
                next
            };
            let t_cmd = self.now();
            match self.gcode.run_script(&line) {
                Ok(()) => {
                    let t_cmd_end = self.now();
                    self.debug_note_cmd(chunk.as_ref(), &line, t_cmd, t_cmd_end, true);
                }
                Err(KlippyError::Command(msg)) => {
                    let t_cmd_end = self.now();
                    error_message = Some(msg);
                    let on_error = self
                        .on_error_gcode
                        .render(None)
                        .and_then(|script| self.gcode.run_script(&script));
                    if let Err(e) = on_error {
                        log::exception("virtual_sdcard on_error", &error_text(&e));
                    }
                    self.debug_note_cmd(chunk.as_ref(), &line, t_cmd, t_cmd_end, false);
                    break;
                }
                Err(e) => {
                    let t_cmd_end = self.now();
                    log::exception("virtual_sdcard dispatch", &error_text(&e));
                    self.debug_note_cmd(chunk.as_ref(), &line, t_cmd, t_cmd_end, false);
                    break;
                }
            }
            let (skip, file_position) = {
                let mut st = self.state.borrow_mut();
                st.cmd_from_sd = false;
                st.file_position = st.next_file_position;
                (st.next_file_position != next_file_position, st.file_position)
            };
            // Do we need to skip around?
            if skip {
                let seek = {
                    let mut st = self.state.borrow_mut();
                    match st.current_file.as_mut() {
                        Some(f) => f.seek(file_position),
                        None => Err(attribute_error_none("seek")),
                    }
                };
                if let Err(e) = seek {
                    log::exception("virtual_sdcard seek", &e.text());
                    self.debug_close_chunk(self.now(), "seek_error");
                    self.state.borrow_mut().work_timer = None;
                    return NEVER;
                }
                lines.clear();
                partial_input.clear();
            }
        }
        if chunk.is_some() {
            self.debug_close_chunk(self.now(), "exit");
        }
        let file_position = self.state.borrow().file_position;
        log::info(&format!("Exiting SD card print (position {file_position})"));
        let has_file = {
            let mut st = self.state.borrow_mut();
            st.work_timer = None;
            st.cmd_from_sd = false;
            st.current_file.is_some()
        };
        if let Some(msg) = error_message {
            self.print_stats.note_error(&msg);
        } else if has_file {
            self.print_stats.note_pause();
        } else {
            self.print_stats.note_complete();
        }
        NEVER
    }
}

/// `load_config(config)` (`virtual_sdcard.py:486`).
pub fn load_config(config: &Config) -> KResult<Option<ObjectRef>> {
    let obj = VirtualSD::new(config)?;
    Ok(Some(Arc::new(obj) as ObjectRef))
}

/// Module factory: section `virtual_sdcard` dispatches to [`load_config`].
pub fn module_factory() -> ModuleFactory {
    ModuleFactory {
        load_config: Some(load_config),
        load_config_prefix: None,
        add_early_printer_objects: None,
        add_printer_objects: None,
    }
}
