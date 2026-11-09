//! Port of the `queuelogger` unit (`klippy/queuelogger.py`).
//!
//! The module implements asynchronous logging from a background thread:
//! `QueueHandler` formats each record in the producer thread and enqueues
//! it, `QueueListener` (a `TimedRotatingFileHandler` with `when='midnight'`
//! and `backupCount=5`) writes it from the background thread. The root
//! logger state (`MainQueueHandler`, root level, handler list) is kept in
//! [`ROOT`]; [`setup_bg_logging`] and [`clear_bg_logging`] mirror the
//! Python module functions.
//!
//! Formatting: records are written with `logging`'s default formatter,
//! `'%(message)s'` (no `levelname`/`name` fields; `klippy.py` never
//! installs a custom formatter), with any cached `exc_text` appended
//! after a newline exactly like `logging.Formatter.format`.
//!
//! Logging integration: the process-wide sink in `klippy_api::log` is
//! installed once by [`ensure_sink`] and forwards every record to the
//! current [`QueueListener`] channel while respecting the root level
//! (`logging.getLogger()` level filtering). Before any setup the sink is
//! absent and records fall back to stderr exactly like Python's
//! `lastResort`, which also uses the default formatter (plain message).
//! After [`clear_bg_logging`] the handler is removed and the root level
//! reverts to `WARNING`, so later records follow the same stderr
//! fallback for `WARNING` and above and are dropped below it.
//!
//! Time handling: rotation follows CPython 3.8's
//! `TimedRotatingFileHandler` for `when='midnight'`, `utc=False`: the
//! first `rolloverAt` comes from the log file's mtime, the rotated file
//! is `<baseFilename>.<YYYY-MM-DD>` named after the interval it covers,
//! and both use local time (`TZ`) with the DST corrections of
//! `doRollover`. Every conversion goes through [`kos::time::localtime`].

use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::os::unix::fs::MetadataExt;
use std::sync::Arc;

use indexmap::IndexMap;
use kos::cconv;
use kos::sync::{channel, Mutex, Receiver, Sender};
use kos::thread::{self, JoinHandle};
use kos::time;
use klippy_api::error::{KResult, KlippyError};
use klippy_api::log::{self, Level};
use klippy_api::printer::ModuleFactory;
use pycompat::text::{percent_format, PctValue};

/// `TimedRotatingFileHandler(..., backupCount=5)` (`klippy/queuelogger.py:26-27`).
pub const BACKUP_COUNT: usize = 5;
/// Rollover interval for `when='midnight'`, in seconds (one day; also
/// `logging.handlers._MIDNIGHT`).
pub const ROLLOVER_INTERVAL: i64 = 86400;
/// Default formatter of `logging.Formatter` (`'%(message)s'`). `klippy.py`
/// never installs a custom formatter, so every handler writes the expanded
/// message only.
pub const DEFAULT_FORMAT: &str = "%(message)s";
/// Root logger name (`logging.getLogger().name == 'root'`). Only used for
/// stderr fallback bookkeeping; the default formatter never renders it.
pub const ROOT_NAME: &str = "root";
/// `name` of the `logging.makeLogRecord({'msg': ..., 'level': logging.INFO})`
/// record built in `doRollover`. The default formatter never renders it,
/// but it is kept for parity with the reference record.
pub const ROLLOVER_RECORD_NAME: &str = "None";

// ---------------------------------------------------------------------------
// Levels (`logging` module constants).
// ---------------------------------------------------------------------------

/// Python `logging` level int for a [`Level`].
pub fn level_int(level: Level) -> i64 {
    match level {
        Level::Debug => 10,
        Level::Info => 20,
        Level::Warning => 30,
        Level::Error => 40,
        Level::Critical => 50,
    }
}

/// [`Level`] for a Python `logging` level int, or `None` for an unknown
/// value (`logging.getLevelName` would return `"Level %s"` instead).
pub fn level_from_int(value: i64) -> Option<Level> {
    match value {
        10 => Some(Level::Debug),
        20 => Some(Level::Info),
        30 => Some(Level::Warning),
        40 => Some(Level::Error),
        50 => Some(Level::Critical),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Record formatting.
// ---------------------------------------------------------------------------

/// One log record as it crosses the queue: message already expanded in
/// the producer thread (`record.msg = record.message`, `record.args =
/// None`), exception text cached (`record.exc_text`) with `exc_info`
/// cleared (`klippy/queuelogger.py:12-19`).
#[derive(Clone, Debug, PartialEq)]
pub struct PendingRecord {
    /// `record.levelno` / `record.levelname`.
    pub level: Level,
    /// `record.message` after `%` expansion.
    pub message: String,
    /// Cached traceback text (`record.exc_text`), if any.
    pub exc_text: Option<String>,
}

impl PendingRecord {
    pub fn new(level: Level, message: impl Into<String>, exc_text: Option<String>) -> Self {
        PendingRecord { level, message: message.into(), exc_text }
    }
}

/// `Handler.format(record)` with the default formatter: the expanded
/// message only. When `exc_text` is present it is appended with a
/// preceding newline (only added when the message does not already end
/// with one) exactly like `logging.Formatter.format`. `level` and
/// `logger_name` never reach the output.
pub fn format_record(_level: Level, _logger_name: &str, message: &str, exc_text: Option<&str>) -> String {
    let mut out = message.to_string();
    if let Some(exc) = exc_text {
        if !out.ends_with('\n') {
            out.push('\n');
        }
        out.push_str(exc);
    }
    out
}

/// Root-logger record formatting (`logging.getLogger()` has name `'root'`).
pub fn format_root_record(level: Level, message: &str, exc_text: Option<&str>) -> String {
    format_record(level, ROOT_NAME, message, exc_text)
}

/// The synthetic record emitted by `doRollover`
/// (`logging.makeLogRecord({'msg': ..., 'level': logging.INFO})`).
pub fn format_rollover_record(message: &str) -> String {
    format_record(Level::Info, ROLLOVER_RECORD_NAME, message, None)
}

// ---------------------------------------------------------------------------
// Local time (`time.localtime` / `time.strftime` through `kos::time`).
// ---------------------------------------------------------------------------

/// `time.localtime(t)` for a time the wall clock or the filesystem
/// produced; those always convert.
fn local(t: i64) -> time::Tm {
    time::localtime(t as f64).expect("wall-clock times convert to local time")
}

/// `int(time.time())`.
fn int_time() -> i64 {
    cconv::f64_to_i64(time::wall_time())
}

/// `time.asctime(time.localtime(t))` (`klippy/queuelogger.py:53-54`).
pub fn asctime(epoch_secs: f64) -> String {
    let tm = time::localtime(epoch_secs).expect("wall-clock times convert to local time");
    time::asctime(&tm)
}

/// `=============== Log rollover at %s ===============` (`klippy/queuelogger.py:52-54`).
pub fn rollover_header(epoch_secs: f64) -> String {
    percent_format(
        "=============== Log rollover at %s ===============",
        &[PctValue::Str(asctime(epoch_secs))],
    )
    .expect("static rollover format")
}

/// `computeRollover(currentTime)` for `when='MIDNIGHT'`, `utc=False`,
/// `atTime=None`: the next local midnight after `current_time`.
pub fn compute_rollover(current_time: i64) -> i64 {
    let t = local(current_time);
    let current = (i64::from(t.tm_hour) * 60 + i64::from(t.tm_min)) * 60 + i64::from(t.tm_sec);
    let mut r = ROLLOVER_INTERVAL - current;
    if r < 0 {
        r += ROLLOVER_INTERVAL;
    }
    current_time + r
}

/// The time arithmetic of `doRollover` at `current_time`
/// (`int(time.time())`): the rotated-file suffix
/// (`time.strftime('%Y-%m-%d', timeTuple)` for the interval that ends at
/// `rollover_at`, DST-corrected) and the next `rolloverAt`.
pub fn rollover_plan(rollover_at: i64, current_time: i64) -> (String, i64) {
    let dst_now = local(current_time).tm_isdst;
    let t = rollover_at - ROLLOVER_INTERVAL;
    let mut time_tuple = local(t);
    if dst_now != time_tuple.tm_isdst {
        let addend = if dst_now != 0 { 3600 } else { -3600 };
        time_tuple = local(t + addend);
    }
    let suffix = time::strftime("%Y-%m-%d", &time_tuple);
    let mut new_rollover_at = compute_rollover(current_time);
    while new_rollover_at <= current_time {
        new_rollover_at += ROLLOVER_INTERVAL;
    }
    if dst_now != local(new_rollover_at).tm_isdst {
        new_rollover_at += if dst_now == 0 { -3600 } else { 3600 };
    }
    (suffix, new_rollover_at)
}

/// `shouldRollover(record)`: `int(time.time()) >= rolloverAt`.
pub fn should_rollover_at(rollover_at: i64, now: i64) -> bool {
    now >= rollover_at
}

// ---------------------------------------------------------------------------
// QueueListener (`klippy/queuelogger.py:24-56`).
// ---------------------------------------------------------------------------

struct Shared {
    base_filename: String,
    stream: Mutex<File>,
    rollover_info: Mutex<IndexMap<String, String>>,
    rollover_at: Mutex<i64>,
    sender: Sender<Option<PendingRecord>>,
    stopped: Mutex<bool>,
}

pub struct QueueListener {
    shared: Arc<Shared>,
    thread: Mutex<Option<JoinHandle>>,
}

impl QueueListener {
    fn new_with_channel(
        filename: &str,
        receiver: Receiver<Option<PendingRecord>>,
        sender: Sender<Option<PendingRecord>>,
    ) -> KResult<QueueListener> {
        let base = absolutize(filename);
        // `open(self.baseFilename, 'a')` / `os.stat(filename)`: the errno subclass of
        // `OSError` with `[Errno N] strerror: 'path'`.
        let os_error = |e: std::io::Error| crate::webhooks::os_error(&e, Some(&base));
        let file = OpenOptions::new().append(true).create(true).open(&base).map_err(os_error)?;
        // The file exists once opened, so `__init__` takes
        // `os.stat(filename)[ST_MTIME]`.
        let mtime = file.metadata().map_err(os_error)?.mtime();
        let shared = Arc::new(Shared {
            base_filename: base,
            stream: Mutex::new(file),
            rollover_info: Mutex::new(IndexMap::new()),
            rollover_at: Mutex::new(compute_rollover(mtime)),
            sender: sender.clone(),
            stopped: Mutex::new(false),
        });
        let thread_shared = Arc::clone(&shared);
        let handle = thread::spawn(move || {
            loop {
                let item = receiver.get();
                match item {
                    None => break,
                    Some(None) => break,
                    Some(Some(record)) => {
                        write_record_via(&thread_shared, &record);
                    }
                }
            }
        });
        Ok(QueueListener { shared, thread: Mutex::new(Some(handle)) })
    }

    /// `QueueListener(filename)` (`klippy/queuelogger.py:25`).
    pub fn new(filename: &str) -> KResult<QueueListener> {
        let (sender, receiver) = channel::<Option<PendingRecord>>();
        QueueListener::new_with_channel(filename, receiver, sender)
    }

    /// `ql.baseFilename` (`TimedRotatingFileHandler.baseFilename`).
    pub fn base_filename(&self) -> &str {
        &self.shared.base_filename
    }

    /// `ql.filename` as passed to the constructor.
    pub fn filename(&self) -> &str {
        &self.shared.base_filename
    }

    /// `ql.bg_queue` (sender half; `qsize()` / `empty()` equivalents).
    pub fn queue_size(&self) -> usize {
        self.shared.sender.qsize()
    }

    /// `ql.bg_queue.empty()`.
    pub fn queue_empty(&self) -> bool {
        self.shared.sender.empty()
    }

    /// Current `rolloverAt`.
    pub fn rollover_at(&self) -> i64 {
        *self.shared.rollover_at.lock()
    }

    /// Set the `rolloverAt` used for the next rotation. Test/host hook
    /// mirroring the Python attribute of the same name.
    pub fn set_rollover_at(&self, value: i64) {
        *self.shared.rollover_at.lock() = value;
    }

    /// `ql.set_rollover_info(name, info)` (`klippy/queuelogger.py:41-45`).
    /// `None` removes the entry (`pop(name, None)`).
    pub fn set_rollover_info(&self, name: &str, info: Option<&str>) {
        let mut map = self.shared.rollover_info.lock();
        match info {
            None => {
                map.shift_remove(name);
            }
            Some(text) => {
                map.insert(name.to_string(), text.to_string());
            }
        }
    }

    /// `ql.clear_rollover_info()` (`klippy/queuelogger.py:46-47`).
    pub fn clear_rollover_info(&self) {
        self.shared.rollover_info.lock().clear();
    }

    /// Snapshot of `rollover_info` sorted by name, in the order
    /// `doRollover` emits it.
    pub fn rollover_info_sorted(&self) -> Vec<(String, String)> {
        let map = self.shared.rollover_info.lock();
        let mut out: Vec<(String, String)> =
            map.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
        out.sort_by(|a, b| a.0.cmp(&b.0));
        out
    }

    /// `ql.stop()` (`klippy/queuelogger.py:38-40`): enqueue `None` and
    /// join the background thread.
    pub fn stop(&self) {
        if *self.shared.stopped.lock() {
            return;
        }
        *self.shared.stopped.lock() = true;
        self.shared.sender.put(None);
        let handle = self.thread.lock().take();
        if let Some(h) = handle {
            h.join();
        }
    }

    /// `ql.doRollover()` (`klippy/queuelogger.py:48-56`) at the current
    /// time.
    pub fn do_rollover(&self) -> KResult<()> {
        self.do_rollover_at(int_time())
    }

    /// `ql.doRollover()` with `int(time.time())` equal to `current_time`
    /// (the header still shows the real `time.asctime()`, as in Python).
    pub fn do_rollover_at(&self, current_time: i64) -> KResult<()> {
        do_rollover_via(&self.shared, current_time).map_err(|e| KlippyError::Other {
            class_name: "OSError".to_string(),
            message: format!("{e}"),
        })
    }

    /// `handle(record)` for one already-formatted pending record, on the
    /// calling thread (same path as the background thread).
    pub fn handle_pending(&self, record: &PendingRecord) {
        write_record_via(&self.shared, record);
    }

    /// Enqueue a record for the background thread (the `QueueHandler.emit`
    /// equivalent used when driving this listener without the process-wide
    /// root logger, e.g. in tests).
    pub fn emit(&self, record: PendingRecord) {
        self.shared.sender.put(Some(record));
    }
}
impl std::fmt::Debug for QueueListener {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("QueueListener")
            .field("base_filename", &self.shared.base_filename)
            .field("rollover_at", &self.rollover_at())
            .finish()
    }
}

impl Drop for QueueListener {
    fn drop(&mut self) {
        // Python leaves a non-daemon thread running; `kos` detaches the
        // handle on drop. `stop()` is the normal shutdown path.
        let handle = self.thread.lock().take();
        if let Some(h) = handle {
            h.detach();
        }
    }
}

/// `BaseRotatingHandler.emit`: roll over first when due, then write the
/// record. An `OSError` from the rollover goes to `handleError` (a
/// report on stderr) and drops the record, like Python.
fn write_record_via(shared: &Arc<Shared>, record: &PendingRecord) {
    let now = int_time();
    if should_rollover_at(*shared.rollover_at.lock(), now) {
        if let Err(e) = do_rollover_via(shared, now) {
            eprintln!("--- Logging error ---\nOSError: {e}");
            return;
        }
    }
    let line = format_root_record(record.level, &record.message, record.exc_text.as_deref());
    write_line(shared, &line);
}

fn write_line(shared: &Arc<Shared>, line: &str) {
    let mut stream = shared.stream.lock();
    if writeln!(stream, "{line}").and_then(|()| stream.flush()).is_err() {
        eprintln!("--- Logging error ---\nFailed to write log record");
    }
}

/// `QueueListener.doRollover` at `current_time`: the
/// `TimedRotatingFileHandler.doRollover` file work (rename the log to
/// `<base>.<suffix>`, delete old backups, reopen), then the info record.
fn do_rollover_via(shared: &Arc<Shared>, current_time: i64) -> std::io::Result<()> {
    let rollover_at = *shared.rollover_at.lock();
    let (suffix, new_rollover_at) = rollover_plan(rollover_at, current_time);
    let base = &shared.base_filename;
    let dfn = format!("{base}.{suffix}");
    {
        let mut stream = shared.stream.lock();
        stream.flush()?;
        if fs::metadata(&dfn).is_ok() {
            fs::remove_file(&dfn)?;
        }
        if fs::metadata(base).is_ok() {
            fs::rename(base, &dfn)?;
        }
        for path in files_to_delete(base, BACKUP_COUNT)? {
            fs::remove_file(path)?;
        }
        *stream = OpenOptions::new().append(true).create(true).open(base)?;
    }
    *shared.rollover_at.lock() = new_rollover_at;
    let mut lines: Vec<String> = {
        let map = shared.rollover_info.lock();
        let mut names: Vec<&String> = map.keys().collect();
        names.sort();
        names.into_iter().map(|n| map[n].clone()).collect()
    };
    lines.push(rollover_header(time::wall_time()));
    write_line(shared, &format_rollover_record(&lines.join("\n")));
    Ok(())
}

fn absolutize(filename: &str) -> String {
    if filename.starts_with('/') {
        filename.to_string()
    } else {
        let cwd = std::env::current_dir()
            .map(|p| p.to_string_lossy().into_owned())
            .unwrap_or_default();
        if cwd.is_empty() {
            filename.to_string()
        } else if cwd.ends_with('/') {
            format!("{cwd}{filename}")
        } else {
            format!("{cwd}/{filename}")
        }
    }
}

/// `extMatch` for `when='midnight'`: `^\d{4}-\d{2}-\d{2}(\.\w+)?$`
/// (`re.ASCII`; `$` also matches before a final newline).
fn ext_match(suffix: &str) -> bool {
    let s = suffix.strip_suffix('\n').unwrap_or(suffix).as_bytes();
    let digits = |range: std::ops::Range<usize>| s[range].iter().all(u8::is_ascii_digit);
    if s.len() < 10 || !digits(0..4) || s[4] != b'-' || !digits(5..7) || s[7] != b'-' || !digits(8..10) {
        return false;
    }
    match &s[10..] {
        [] => true,
        [b'.', word @ ..] => {
            !word.is_empty() && word.iter().all(|c| c.is_ascii_alphanumeric() || *c == b'_')
        }
        _ => false,
    }
}

/// `getFilesToDelete()`: backups `<baseName>.<suffix>` beyond the newest
/// `backup_count`, oldest first.
fn files_to_delete(base_filename: &str, backup_count: usize) -> std::io::Result<Vec<String>> {
    let split = base_filename.rfind('/').map(|i| i + 1).unwrap_or(0);
    let (dir, base_name) = base_filename.split_at(split);
    let dir_name = match dir.trim_end_matches('/') {
        "" if !dir.is_empty() => "/",
        trimmed => trimmed,
    };
    let prefix = format!("{base_name}.");
    let mut result = Vec::new();
    for entry in fs::read_dir(dir_name)? {
        let name = entry?.file_name().to_string_lossy().into_owned();
        if name.strip_prefix(&prefix).is_some_and(ext_match) {
            result.push(format!("{dir}{name}"));
        }
    }
    if result.len() < backup_count {
        return Ok(Vec::new());
    }
    result.sort();
    result.truncate(result.len() - backup_count);
    Ok(result)
}

// ---------------------------------------------------------------------------
// Root logger (`MainQueueHandler`).
// ---------------------------------------------------------------------------

struct RootState {
    level: Mutex<Level>,
    sender: Mutex<Option<Sender<Option<PendingRecord>>>>,
}

static ROOT: std::sync::LazyLock<Arc<RootState>> = std::sync::LazyLock::new(|| {
    Arc::new(RootState {
        level: Mutex::new(Level::Warning),
        sender: Mutex::new(None),
    })
});

fn root_state() -> Arc<RootState> {
    Arc::clone(&ROOT)
}

/// Install the process-wide sink (once). Later calls only update the
/// current listener's channel and level.
fn ensure_sink() {
    let state = root_state();
    let sink_state = Arc::clone(&state);
    log::set_sink(Box::new(move |rec| {
        let level = *sink_state.level.lock();
        let guard = sink_state.sender.lock();
        match guard.as_ref() {
            Some(sender) => {
                if rec.level >= level {
                    let pending = PendingRecord {
                        level: rec.level,
                        message: rec.message.to_string(),
                        exc_text: rec.exc_text.map(|t| t.to_string()),
                    };
                    sender.put(Some(pending));
                }
            }
            None => {
                // No handler attached (`lastResort`): emit the formatted
                // record to stderr only when it passes the root level.
                // `StreamHandler` writes `format(record)` plus a newline;
                // the cached traceback is part of `format` output.
                if rec.level >= level {
                    let line = format_root_record(rec.level, rec.message, rec.exc_text);
                    eprintln!("{line}");
                }
            }
        }
    }));
}

/// Current root level (`logging.getLogger().level`; `WARNING` by default).
pub fn root_level() -> Level {
    *root_state().level.lock()
}

/// `root.setLevel(level)`.
pub fn root_set_level(level: Level) {
    *root_state().level.lock() = level;
}

/// Whether `MainQueueHandler` is currently attached.
pub fn main_handler_installed() -> bool {
    root_state().sender.lock().is_some()
}

/// `setup_bg_logging(filename, debuglevel)` (`klippy/queuelogger.py:60-67`).
pub fn setup_bg_logging(filename: &str, debuglevel: Level) -> KResult<QueueListener> {
    let (sender, receiver) = channel::<Option<PendingRecord>>();
    let listener = QueueListener::new_with_channel(filename, receiver, sender.clone())?;
    ensure_sink();
    *root_state().sender.lock() = Some(sender);
    *root_state().level.lock() = debuglevel;
    Ok(listener)
}

/// `clear_bg_logging()` (`klippy/queuelogger.py:69-75`).
pub fn clear_bg_logging() {
    if main_handler_installed() {
        *root_state().sender.lock() = None;
        *root_state().level.lock() = Level::Warning;
    }
}

/// `module_factory`: the Python module has no `load_config` /
/// `add_printer_objects` entry points.
pub fn module_factory() -> ModuleFactory {
    ModuleFactory::default()
}
