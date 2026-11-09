//! Port of `klippy/util.py`.
//!
//! Covers the whole unit: the low-level Unix helpers (`fix_sigint`,
//! `set_nonblock`, `clear_hupcl`, `create_pty`), the MCU build dump
//! (`_try_read_file`, `dump_file_stats`, `dump_mcu_build`), the dropped
//! Python 2 wrapper stub (`setup_python2_wrappers`), and the host
//! identity helpers (`get_cpu_info`, `get_device_info`,
//! `get_linux_version`, `get_version_from_file`, `_get_repo_info`,
//! `get_git_version`).
//!
//! Two host integrations are explicit inputs rather than implicit
//! globals, because the Python module reads `__file__` and `/proc`:
//! - `dump_mcu_build(build_dir)` and `get_git_version(klippy_src, ...)`
//!   take the directory the Python module derives from `__file__`.
//! - Pure parsing steps are exposed as `*_from_data` helpers so
//!   generated inputs can be exercised without `/proc` or a git repo.
//!
//! Text paths (`open(..., 'r')`, universal newlines, the `%` formatting
//! and `json.loads` for `klipper.dict`) go through `pycompat` exactly
//! as the Python code does. File descriptors go through `kos`.
//! Per-file timestamps use the local timezone via glibc `localtime_r` /
//! `asctime_r` (the same calls Python's `time.localtime` /
//! `time.asctime` use). Traceback bodies in debug logs are
//! irreproducible without a Python frame stack; they are collapsed to
//! a deterministic `Traceback (most recent call last):` text and the
//! frozen normalizer (`parity/normalize/table.json` `nan-traceback`)
//! covers them in parity diffs.

use indexmap::IndexMap;
use klippy_api::log;
use kos::{fd as kos_fd, pty as kos_pty, termios as kos_termios};
use pycompat::text::{percent_format, PctValue, PyError};
use pycompat::{json, PyInt, PyValue};
use std::os::unix::fs::MetadataExt;
use std::process::Command;

use crate::configfile::{py_dirname, py_join};

/// Result of a function that raises a Python exception on error.
pub type PyResult<T> = Result<T, PyError>;

// ---------------------------------------------------------------------------
// Low-level Unix commands (`klippy/util.py:14-47`)
// ---------------------------------------------------------------------------

pub const SIGINT: i32 = 2;
pub const SIG_DFL: usize = 0;

unsafe extern "C" {
    fn signal(signum: i32, handler: usize) -> usize;
}

/// `signal.signal(signal.SIGINT, signal.SIG_DFL)` (`util.py:15`).
///
/// Runs at import time in Python (and from host startup here). Returns
/// the previous handler as `None` like the Python call, which discards
/// it. The glibc `signal` call is used directly because `kos` exposes
/// no signal helper; semantics are identical.
pub fn fix_sigint() -> PyResult<PyValue> {
    // SAFETY: handler 0 is SIG_DFL; signum 2 is SIGINT on Linux.
    unsafe {
        signal(SIGINT, SIG_DFL);
    }
    Ok(PyValue::None)
}

/// `set_nonblock(fd)` (`util.py:20`). Raises `OSError` on a bad fd,
/// like `fcntl` does. A negative fd is rejected by CPython's fd
/// conversion (`fcntl.conv_descriptor`) with `ValueError` before the
/// syscall runs.
pub fn set_nonblock(fd: i32) -> PyResult<PyValue> {
    if fd < 0 {
        return Err(negative_fd_error(fd));
    }
    kos_fd::set_nonblock(fd).map(|()| PyValue::None).map_err(|e| os_error(&e, None, "OSError"))
}

/// `clear_hupcl(fd)` (`util.py:25`). `tcgetattr` errors propagate as
/// `termios.error`; `tcsetattr` errors are ignored, like the Python
/// `try`/`except termios.error: pass`. A negative fd is rejected by
/// CPython's fd conversion (`termios.fdconv`) with `ValueError` before
/// the syscall runs.
pub fn clear_hupcl(fd: i32) -> PyResult<PyValue> {
    if fd < 0 {
        return Err(negative_fd_error(fd));
    }
    let mut attrs = kos_termios::tcgetattr(fd).map_err(|e| os_error(&e, None, "error"))?;
    attrs.c_cflag &= !kos_termios::HUPCL;
    let _ = kos_termios::tcsetattr(fd, kos_termios::TCSADRAIN, &attrs);
    Ok(PyValue::None)
}

/// `ValueError: file descriptor cannot be a negative integer (-N)` —
/// what CPython raises for a negative fd argument in `termios` /
/// `fcntl` (via `PyObject_AsFileDescriptor`).
fn negative_fd_error(fd: i32) -> PyError {
    PyError::new(
        "ValueError",
        format!("file descriptor cannot be a negative integer ({fd})"),
    )
}

/// `create_pty(ptyname)` (`util.py:34`). Returns the master fd as an
/// int, raising `OSError` on failure.
pub fn create_pty(ptyname: &str) -> PyResult<PyValue> {
    let master = kos_pty::create_pty(ptyname).map_err(|e| os_error(&e, Some(ptyname), "OSError"))?;
    Ok(PyValue::Int(PyInt::from(master as i64)))
}


// ---------------------------------------------------------------------------
// `_try_read_file` and the MCU build dump (`util.py:54-93`)
// ---------------------------------------------------------------------------

/// Python `_try_read_file(filename, maxsize=32*1024)` (`util.py:54`).
///
/// Text mode with universal newlines; `maxsize` is a character count
/// (CPython `f.read(maxsize)`). Returns `None` on `OSError`, logging
/// the debug line the Python code logs. Non-UTF-8 data raises
/// `UnicodeDecodeError` in Python (not caught by the `except`); here
/// it is also logged and yields `None` (the unit only reads ASCII
/// host files).
pub fn try_read_file(filename: &str, maxsize: usize) -> Option<String> {
    match try_read_file_silent(filename, maxsize) {
        Ok(text) => Some(text),
        Err(e) => {
            log::debug(&fmt_msg(
                "Exception on read %s: %s",
                &[PctValue::Str(filename.to_string()), PctValue::Str(e.traceback())],
            ));
            None
        }
    }
}

/// A failure inside `_try_read_file`'s `with` block (`util.py:56-57`).
#[derive(Clone, Debug)]
pub struct ReadFileError {
    /// The raised exception.
    pub error: PyError,
    /// `util.py` line that raised it: 56 for `open()`, 57 for `f.read()`.
    pub line: u32,
}

impl ReadFileError {
    /// Whether the `except (IOError, OSError)` clause catches it.
    pub fn is_os_error(&self) -> bool {
        self.error.kind != "UnicodeDecodeError"
    }

    /// `traceback.format_exc()` inside `_try_read_file`: one frame in
    /// `util.py` of the recorded klippy source directory.
    pub fn traceback(&self) -> String {
        let source = if self.line == 56 {
            "with open(filename, 'r') as f:"
        } else {
            "return f.read(maxsize)"
        };
        format!(
            "Traceback (most recent call last):\n  File \"{}\", line {}, in _try_read_file\n    {}\n{}: {}\n",
            klippy_api::paths::klippy_path(&["util.py"]),
            self.line,
            source,
            self.error.kind,
            self.error.msg
        )
    }
}

/// Strict reader behind [`try_read_file`]: `Ok(text)` on success, the
/// exception and its `util.py` line otherwise.
pub fn try_read_file_silent(filename: &str, maxsize: usize) -> Result<String, ReadFileError> {
    use std::io::Read;
    const EISDIR: i32 = 21;
    let open_failed = |e: &std::io::Error| ReadFileError { error: open_error(e, filename), line: 56 };
    let mut file = std::fs::File::open(filename).map_err(|e| open_failed(&e))?;
    let mut bytes = Vec::new();
    if let Err(e) = file.read_to_end(&mut bytes) {
        // CPython's text-mode `open()` refuses a directory itself; the
        // read only fails there on Linux.
        if e.raw_os_error() == Some(EISDIR) {
            return Err(open_failed(&e));
        }
        return Err(ReadFileError { error: os_error(&e, None, os_error_class(&e)), line: 57 });
    }
    let text = String::from_utf8(bytes).map_err(|_| ReadFileError {
        error: PyError::new("UnicodeDecodeError", "'utf-8' codec can't decode bytes".to_string()),
        line: 57,
    })?;
    // Universal newlines: Python text mode translates \r\n and \r to \n.
    let text = text.replace("\r\n", "\n").replace('\r', "\n");
    let truncated: String = text.chars().take(maxsize).collect();
    Ok(truncated)
}

/// The `OSError` subclass CPython raises for an errno
/// (`Objects/exceptions.c`, `errnomap`).
fn os_error_class(e: &std::io::Error) -> &'static str {
    match e.raw_os_error().unwrap_or(0) {
        11 | 114 | 115 => "BlockingIOError",
        10 => "ChildProcessError",
        32 | 108 => "BrokenPipeError",
        103 => "ConnectionAbortedError",
        111 => "ConnectionRefusedError",
        104 => "ConnectionResetError",
        17 => "FileExistsError",
        2 => "FileNotFoundError",
        21 => "IsADirectoryError",
        20 => "NotADirectoryError",
        4 => "InterruptedError",
        1 | 13 => "PermissionError",
        3 => "ProcessLookupError",
        110 => "TimeoutError",
        _ => "OSError",
    }
}

/// The exception `open(filename, 'r')` raises: errno subclass and
/// `[Errno N] strerror: 'filename'`.
fn open_error(e: &std::io::Error, filename: &str) -> PyError {
    let base = os_error(e, None, os_error_class(e));
    PyError::new(base.kind, format!("{}: {}", base.msg, pycompat::text::str_repr(filename)))
}

/// `time.asctime(time.localtime(mtime))` (`util.py:68`) through
/// [`kos::time::localtime`], so `TZ` applies like in Python. Returns
/// `None` where `time.localtime` raises (caught by `dump_file_stats`).
pub fn asctime_from_mtime(mtime: f64) -> Option<String> {
    kos::time::localtime(mtime).map(|tm| kos::time::asctime(&tm))
}

/// `dump_file_stats(build_dir, filename)` (`util.py:63`).
pub fn dump_file_stats(build_dir: &str, filename: &str) -> PyValue {
    let fname = py_join(build_dir, filename);
    if let Some(msg) = dump_file_stats_line(&fname) {
        log::info(&msg);
    } else {
        log::info(&fmt_msg("No build file %s", &[PctValue::Str(fname.clone())]));
    }
    PyValue::None
}

fn dump_file_stats_line(fname: &str) -> Option<String> {
    let meta = std::fs::metadata(fname).ok()?;
    let fsize = meta.len();
    let mtime = meta.mtime() as f64 + (meta.mtime_nsec() as f64) * 1e-9;
    let timestr = asctime_from_mtime(mtime)?;
    Some(fmt_msg(
        "Build file %s(%d): %s",
        &[
            PctValue::Str(fname.to_string()),
            PctValue::Int(PyInt::from(fsize as i64)),
            PctValue::Str(timestr),
        ],
    ))
}

/// `dump_mcu_build()` (`util.py:74`), with the `build_dir` the Python
/// code computes as `os.path.join(os.path.dirname(__file__), '..')`.
pub fn dump_mcu_build(build_dir: &str) -> PyValue {
    dump_file_stats(build_dir, ".config");
    let data = try_read_file(&py_join(build_dir, ".config"), 32 * 1024);
    if let Some(data) = data {
        log::info(&format!(
            "========= Last MCU build config =========\n{}=======================",
            data
        ));
    }
    dump_file_stats(build_dir, "out/klipper.dict");
    if let Some(line) = klipper_dict_line(&py_join(build_dir, "out/klipper.dict")) {
        for msg in line {
            log::info(&msg);
        }
    }
    dump_file_stats(build_dir, "out/klipper.elf");
    PyValue::None
}

/// The three `Last MCU build ...` info lines, or `None` when the
/// Python `try` block would `pass` (missing file, bad JSON, or a
/// non-dict shape aborting partway). Lines already emitted before an
/// abort are still returned, like Python logs them before raising.
fn klipper_dict_line(dict_path: &str) -> Option<Vec<String>> {
    // Python calls `_try_read_file` here, which logs the debug line on
    // `OSError` (UTF-8 errors escape it, but the bare `except` below
    // in `dump_mcu_build` swallows them without logging).
    let raw = match try_read_file_silent(dict_path, 32 * 1024) {
        Ok(text) => text,
        Err(e) => {
            if e.is_os_error() {
                log::debug(&fmt_msg(
                    "Exception on read %s: %s",
                    &[PctValue::Str(dict_path.to_string()), PctValue::Str(e.traceback())],
                ));
            }
            return None;
        }
    };
    let data = json::loads(&raw).ok()?;
    let dict = match data {
        PyValue::Dict(m) => m,
        _ => return Some(Vec::new()),
    };
    let get = |key: &str| -> String {
        dict.get(&PyValue::Str(key.to_string())).map(pyvalue_str).unwrap_or_default()
    };
    let mut out = Vec::new();
    out.push(fmt_msg(
        "Last MCU build version: %s",
        &[PctValue::Str(get("version"))],
    ));
    out.push(fmt_msg(
        "Last MCU build tools: %s",
        &[PctValue::Str(get("build_versions"))],
    ));
    match dict.get(&PyValue::Str("config".to_string())) {
        None => {
            out.push(fmt_msg("Last MCU build config: %s", &[PctValue::Str(String::new())]));
        }
        Some(PyValue::Dict(config)) => {
            let parts: Vec<String> = config
                .iter()
                .map(|(k, v)| {
                    let ks = pyvalue_str(k);
                    let vs = pyvalue_str(v);
                    fmt_msg("%s=%s", &[PctValue::Str(ks), PctValue::Str(vs)])
                })
                .collect();
            out.push(fmt_msg(
                "Last MCU build config: %s",
                &[PctValue::Str(parts.join(" "))],
            ));
        }
        Some(_) => {
            // `data.get('config', {}).items()` raises; the first two
            // logs are already emitted.
        }
    }
    Some(out)
}

// ---------------------------------------------------------------------------
// Python 2 wrappers (`util.py:100-109`)
// ---------------------------------------------------------------------------

/// `setup_python2_wrappers()` (`util.py:100`). On Python 3 this is a
/// no-op; the port is always Python 3, so it stays a no-op (the
/// import-time call is the caller's `setup_python2_wrappers()`).
pub fn setup_python2_wrappers() -> PyValue {
    PyValue::None
}

// ---------------------------------------------------------------------------
// Host identity (`util.py:116-144`)
// ---------------------------------------------------------------------------

/// `"%d core %s" % (core_count, model_name)` from `cpuinfo` text
/// (`util.py:116`) without reading `/proc`.
pub fn cpu_info_from_data(data: &str) -> String {
    let lines: Vec<(String, String)> = data
        .split('\n')
        .filter_map(|l| {
            let mut parts = l.splitn(2, ':');
            let key = parts.next()?;
            let val = parts.next()?;
            Some((py_strip(key), py_strip(val)))
        })
        .collect();
    let core_count = lines.iter().filter(|(k, _)| k == "processor").count();
    let mut model_name = "?".to_string();
    for (k, v) in &lines {
        if k == "model name" {
            model_name = v.clone();
            break;
        }
    }
    // `dict(lines).get("model name", "?")`: last occurrence wins.
    for (k, v) in lines.iter().rev() {
        if k == "model name" {
            model_name = v.clone();
            break;
        }
    }
    fmt_msg(
        "%d core %s",
        &[
            PctValue::Int(PyInt::from(core_count as i64)),
            PctValue::Str(model_name),
        ],
    )
}

/// `data.rstrip(' \0').strip()` (`util.py:132`) from already-read
/// model text (the `/proc/device-tree/model` or DMI fallback).
pub fn device_info_from_data(data: &str) -> String {
    py_strip(&py_rstrip_chars(data, &[' ', '\0']))
}

/// `data.strip()` (`util.py:138`) from already-read `/proc/version`.
pub fn linux_version_from_data(data: &str) -> String {
    py_strip(data)
}

/// `data.rstrip()` (`util.py:144`) from already-read `.version`.
pub fn version_from_data(data: &str) -> String {
    py_rstrip(data)
}

/// `get_cpu_info()` (`util.py:116`). Reads `/proc/cpuinfo`.
pub fn get_cpu_info() -> String {
    match try_read_file("/proc/cpuinfo", 1024 * 1024) {
        Some(data) => cpu_info_from_data(&data),
        None => "?".to_string(),
    }
}

/// `get_device_info()` (`util.py:126`). Falls back to the DMI product
/// name, then `"?"`.
pub fn get_device_info() -> String {
    if let Some(data) = try_read_file("/proc/device-tree/model", 32 * 1024) {
        return device_info_from_data(&data);
    }
    if let Some(data) = try_read_file("/sys/class/dmi/id/product_name", 32 * 1024) {
        return device_info_from_data(&data);
    }
    "?".to_string()
}

/// `get_linux_version()` (`util.py:134`). Reads `/proc/version`.
pub fn get_linux_version() -> String {
    match try_read_file("/proc/version", 32 * 1024) {
        Some(data) => linux_version_from_data(&data),
        None => "?".to_string(),
    }
}

/// `get_version_from_file(klippy_src)` (`util.py:140`).
pub fn get_version_from_file(klippy_src: &str) -> String {
    match try_read_file(&py_join(klippy_src, ".version"), 32 * 1024) {
        Some(data) => version_from_data(&data),
        None => "?".to_string(),
    }
}

// ---------------------------------------------------------------------------
// Git version (`util.py:146-236`)
// ---------------------------------------------------------------------------

/// `{"branch", "remote", "url"}` from `_get_repo_info(gitdir)`.
#[derive(Clone, Debug, PartialEq)]
pub struct RepoInfo {
    pub branch: String,
    pub remote: String,
    pub url: String,
}

/// `{"version", "file_status", "branch", "remote", "url"}` from
/// `get_git_version()`. `file_status` is a list of string pairs
/// (`(code, filename)`), reproducing the Python split quirk.
#[derive(Clone, Debug, PartialEq)]
pub struct GitInfo {
    pub version: String,
    pub file_status: Vec<Vec<String>>,
    pub branch: String,
    pub remote: String,
    pub url: String,
}

/// Subprocess result: `(stdout_bytes, stderr_bytes, returncode)`.
/// `None` when the spawn itself fails (like `Popen` raising `OSError`).
pub fn run_git(args: &[&str]) -> Option<(Vec<u8>, Vec<u8>, i32)> {
    let out = Command::new(args[0]).args(&args[1..]).output().ok()?;
    let code = out.status.code().unwrap_or(-1);
    Some((out.stdout, out.stderr, code))
}

/// `str(branch_list.strip().decode()).split("\n")` (`util.py:157`).
pub fn parse_branch_lines(branch_list: &[u8]) -> Vec<String> {
    py_strip_bytes(branch_list).split('\n').map(|s| s.to_string()).collect()
}

/// `lines` step of `_get_repo_info`: first `*` line gives the branch.
/// Returns `None` when no line matches (Python `else` logs and returns).
/// Returns `Err(())` when an empty line is reached first: Python's
/// `line[0]` raises `IndexError`, caught by the bare `except`.
pub fn branch_from_lines(lines: &[String]) -> Result<Option<String>, ()> {
    for line in lines {
        if line.is_empty() {
            return Err(());
        }
        if line.starts_with('*') {
            return Ok(Some(py_strip(&line[1..])));
        }
    }
    Ok(None)
}

/// Detached-HEAD split (`util.py:166`): `branch.strip("()").split()[-1].split("/", 1)`.
/// Returns the remote when the split has exactly two parts.
pub fn detached_remote(branch: &str) -> Option<String> {
    let stripped = branch.trim_matches(|c| c == '(' || c == ')');
    let last = stripped.split_whitespace().last()?;
    let mut parts = last.splitn(2, '/');
    let first = parts.next()?;
    let second = parts.next()?;
    let _ = second;
    if last.contains('/') { Some(first.to_string()) } else { None }
}

/// `status = [l.split(None, 1) for l in str(stat.strip().decode()).split('\n')]` (`util.py:220`).
pub fn parse_status(stat: &[u8]) -> Vec<Vec<String>> {
    py_strip_bytes(stat)
        .split('\n')
        .map(py_split_none_1)
        .collect()
}

/// `_get_repo_info(gitdir)` (`util.py:146`), with an injectable git
/// runner for tests (live calls use [`run_git`]).
pub fn get_repo_info_with_runner<F>(gitdir: &str, run: &F) -> RepoInfo
where
    F: Fn(&[&str]) -> Option<(Vec<u8>, Vec<u8>, i32)>,
{
    let mut info = RepoInfo {
        branch: "?".to_string(),
        remote: "?".to_string(),
        url: "?".to_string(),
    };
    match get_repo_info_inner(gitdir, run, &mut info) {
        Ok(()) => {}
        Err(_) => {}
    }
    info
}

/// `_get_repo_info(gitdir)` with the live git runner.
pub fn get_repo_info(gitdir: &str) -> RepoInfo {
    get_repo_info_with_runner(gitdir, &run_git)
}

fn get_repo_info_inner<F>(gitdir: &str, run: &F, info: &mut RepoInfo) -> Result<(), ()>
where
    F: Fn(&[&str]) -> Option<(Vec<u8>, Vec<u8>, i32)>,
{
    let branch_out = run(&["git", "-C", gitdir, "branch", "--no-color"])
        .ok_or_else(|| log_fetch_error())?;
    if branch_out.2 != 0 {
        log::debug(&fmt_msg(
            "Error running git branch: %s",
            &[PctValue::Bytes(branch_out.1.clone())],
        ));
        return Err(());
    }
    let lines = parse_branch_lines(&branch_out.0);
    match branch_from_lines(&lines) {
        Ok(Some(branch)) => info.branch = branch,
        Ok(None) => {
            log::debug(&fmt_msg(
                "Unable to find current branch:\n%s",
                &[PctValue::Bytes(branch_out.0.clone())],
            ));
            return Err(());
        }
        Err(()) => {
            log_fetch_error();
            return Err(());
        }
    }
    if info.branch.starts_with("(HEAD detached") {
        if let Some(remote) = detached_remote(&info.branch) {
            info.remote = remote;
        } else {
            return Err(());
        }
    } else {
        let key = fmt_msg("branch.%s.remote", &[PctValue::Str(info.branch.clone())]);
        let config_out = run(&["git", "-C", gitdir, "config", "--get", &key])
            .ok_or_else(|| log_fetch_error())?;
        if config_out.2 != 0 {
            log::debug(&fmt_msg(
                "Error running git config: %s",
                &[PctValue::Bytes(config_out.1.clone())],
            ));
            return Err(());
        }
        info.remote = py_strip_bytes(&config_out.0);
    }
    let remote = info.remote.clone();
    let url_out = run(&["git", "-C", gitdir, "remote", "get-url", &remote])
        .ok_or_else(|| log_fetch_error())?;
    if url_out.2 != 0 {
        log::debug(&fmt_msg(
            "Error running git remote get-url: %s",
            &[PctValue::Bytes(url_out.1.clone())],
        ));
        return Err(());
    }
    info.url = py_strip_bytes(&url_out.0);
    Ok(())
}

fn log_fetch_error() -> () {
    log::debug(&fmt_msg(
        "Error fetching repo info: %s",
        &[PctValue::Str(traceback_text(&PyError::new("OSError", "spawn failed".to_string())))],
    ));
}

/// `get_git_version(from_file)` (`util.py:195`), with
/// `klippy_src = os.path.dirname(__file__)` and
/// `gitdir = os.path.join(klippy_src, '..')` made explicit, and an
/// injectable git runner for tests.
pub fn get_git_version_with_runner<F>(klippy_src: &str, from_file: bool, run: &F) -> GitInfo
where
    F: Fn(&[&str]) -> Option<(Vec<u8>, Vec<u8>, i32)>,
{
    let mut info = GitInfo {
        version: "?".to_string(),
        file_status: Vec::new(),
        branch: "?".to_string(),
        remote: "?".to_string(),
        url: "?".to_string(),
    };
    let gitdir = py_join(klippy_src, "..");
    match get_git_version_inner(klippy_src, &gitdir, from_file, run, &mut info) {
        Ok(()) => {}
        Err(_) => {
            if from_file {
                info.version = get_version_from_file(klippy_src);
            }
        }
    }
    info
}

fn get_git_version_inner<F>(
    _klippy_src: &str,
    gitdir: &str,
    _from_file: bool,
    run: &F,
    info: &mut GitInfo,
) -> Result<(), ()>
where
    F: Fn(&[&str]) -> Option<(Vec<u8>, Vec<u8>, i32)>,
{
    let desc = run(&[
        "git",
        "-C",
        gitdir,
        "describe",
        "--always",
        "--tags",
        "--long",
        "--dirty",
    ])
    .ok_or_else(|| log_run_error())?;
    if desc.2 != 0 {
        log::debug(&fmt_msg(
            "Error getting git version: %s",
            &[PctValue::Bytes(desc.1.clone())],
        ));
        return Err(());
    }
    info.version = py_strip_bytes(&desc.0);
    let stat = run(&["git", "-C", gitdir, "status", "--porcelain", "--ignored"])
        .ok_or_else(|| log_run_error())?;
    if stat.2 == 0 {
        info.file_status = parse_status(&stat.0);
    } else {
        log::debug(&fmt_msg(
            "Error getting git status: %s",
            &[PctValue::Bytes(stat.1.clone())],
        ));
    }
    let repo_info = get_repo_info_with_runner(gitdir, run);
    info.branch = repo_info.branch;
    info.remote = repo_info.remote;
    info.url = repo_info.url;
    Ok(())
}

fn log_run_error() -> () {
    log::debug(&fmt_msg(
        "Exception on run: %s",
        &[PctValue::Str(traceback_text(&PyError::new("OSError", "spawn failed".to_string())))],
    ));
}

/// `get_git_version(from_file)` with the live `klippy_src`.
pub fn get_git_version(klippy_src: &str, from_file: bool) -> GitInfo {
    get_git_version_with_runner(klippy_src, from_file, &run_git)
}

// ---------------------------------------------------------------------------
// Python text helpers (str.strip / rstrip / split / decode)
// ---------------------------------------------------------------------------

/// `s.strip()` with CPython whitespace.
pub fn py_strip(s: &str) -> String {
    s.trim().to_string()
}

/// `s.rstrip()` (trailing CPython whitespace).
pub fn py_rstrip(s: &str) -> String {
    s.trim_end().to_string()
}

/// `s.rstrip(' \0')` (explicit chars, not whitespace).
pub fn py_rstrip_chars(s: &str, chars: &[char]) -> String {
    s.trim_end_matches(chars).to_string()
}

/// `b.strip().decode()` as used for git output: strip ASCII whitespace
/// on the bytes, then decode UTF-8 lossily as the oracle does for
/// ASCII git output (`str(bytes, 'utf-8')` with strict errors would
/// raise on bad bytes; git output is ASCII in practice).
pub fn py_strip_bytes(b: &[u8]) -> String {
    String::from_utf8_lossy(b).trim().to_string()
}

/// `l.split(None, 1)`: split on runs of whitespace, max one split.
pub fn py_split_none_1(line: &str) -> Vec<String> {
    let trimmed = line.trim();
    if trimmed.is_empty() {
        return Vec::new();
    }
    match trimmed.find(char::is_whitespace) {
        Some(pos) => {
            let first = trimmed[..pos].to_string();
            let rest = trimmed[pos..].trim_start();
            if rest.is_empty() {
                vec![first]
            } else {
                vec![first, rest.to_string()]
            }
        }
        None => vec![trimmed.to_string()],
    }
}

/// `str(v)` for the scalar values `klipper.dict` carries.
pub fn pyvalue_str(v: &PyValue) -> String {
    match v {
        PyValue::None => "None".to_string(),
        PyValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
        PyValue::Int(n) => n.0.to_string(),
        PyValue::Float(f) => pycompat::text::float_str(*f),
        PyValue::Str(s) => s.clone(),
        PyValue::Bytes(b) => pycompat::text::bytes_repr(b),
        PyValue::List(xs) => {
            let parts: Vec<String> = xs.iter().map(|x| pycompat::text::str_repr(&pyvalue_str(x))).collect();
            format!("[{}]", parts.join(", "))
        }
        PyValue::Tuple(xs) => {
            let parts: Vec<String> = xs.iter().map(|x| pycompat::text::str_repr(&pyvalue_str(x))).collect();
            if parts.len() == 1 {
                format!("({},)", parts[0])
            } else {
                format!("({})", parts.join(", "))
            }
        }
        PyValue::Dict(_) => format!("{v:?}"),
        PyValue::Host(h) => format!("<host object at {:#x}>", h),
    }
}

/// `dirname(__file__)` helper kept for hosts that derive `klippy_src`.
pub fn klippy_src_from_file(file: &str) -> String {
    py_dirname(file)
}

/// `os.path.join(klippy_src, '..')` (the `gitdir`).
pub fn gitdir_from_klippy_src(klippy_src: &str) -> String {
    py_join(klippy_src, "..")
}

// ---------------------------------------------------------------------------
// Internal helpers (formatting, errors, tracebacks)
// ---------------------------------------------------------------------------

fn fmt_msg(msg: &str, params: &[PctValue]) -> String {
    match percent_format(msg, params) {
        Ok(s) => s,
        Err(_) => msg.to_string(),
    }
}

fn os_error(e: &std::io::Error, filename: Option<&str>, kind: &'static str) -> PyError {
    let errno = e.raw_os_error().unwrap_or(0);
    let strerror: String = match errno {
        1 => "Operation not permitted".to_string(),
        2 => "No such file or directory".to_string(),
        5 => "Input/output error".to_string(),
        9 => "Bad file descriptor".to_string(),
        13 => "Permission denied".to_string(),
        22 => "Invalid argument".to_string(),
        _ => {
            let s = e.to_string();
            match s.rfind(" (os error ") {
                Some(pos) => s[..pos].to_string(),
                None => s,
            }
        }
    };
    let _ = filename;
    PyError::new(kind, format!("[Errno {errno}] {strerror}"))
}

/// Deterministic `Traceback (most recent call last):` text for debug
/// logs. The Python original carries frame paths and line numbers
/// that cannot be reproduced; the normalizer collapses these bodies.
fn traceback_text(e: &PyError) -> String {
    format!(
        "Traceback (most recent call last):\n  File \"util.py\", line 0, in <module>\n{}: {}",
        e.kind, e.msg
    )
}

// Re-exported for hosts/tests that need insertion-ordered dicts.
pub fn empty_dict() -> IndexMap<String, PyValue> {
    IndexMap::new()
}
