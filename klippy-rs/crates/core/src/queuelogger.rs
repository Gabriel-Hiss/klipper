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
//! Time handling: rotation and `time.asctime()` use UTC civil arithmetic
//! from [`kos::time::wall_time`]. The host pins `TZ=UTC` for parity runs
//! (see `configfile::datestr_from_wall_time`), so UTC conversion matches
//! the local-time behaviour of the reference under the harness; on other
//! timezones the conversion is still deterministic but in UTC.

use std::fs::{self, File, OpenOptions};
use std::io::{Seek, SeekFrom, Write};
use std::path::PathBuf;
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
pub const BACKUP_COUNT: u32 = 5;
/// Rollover interval for `when='midnight'`, in seconds (one day).
pub const ROLLOVER_INTERVAL: f64 = 86400.0;
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
// UTC civil time (mirrors `time.asctime` / `time.strftime` under `TZ=UTC`).
// ---------------------------------------------------------------------------

fn civil_from_days(days: i64) -> (i64, i64, i64) {
    // Howard Hinnant's algorithm.
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}


fn weekday_from_days(days: i64) -> i64 {
    // 1970-01-01 (day 0) was a Thursday; return 0=Sunday .. 6=Saturday.
    (days + 4).rem_euclid(7)
}

fn split_epoch(epoch_secs: f64) -> (i64, i64, i64, i64, i64, i64, i64) {
    let secs = cconv::f64_to_i64(epoch_secs);
    let days = secs.div_euclid(86400);
    let secs_of_day = secs.rem_euclid(86400);
    let (y, m, d) = civil_from_days(days);
    let wd = weekday_from_days(days);
    (y, m, d, secs_of_day / 3600, (secs_of_day % 3600) / 60, secs_of_day % 60, wd)
}

/// `time.asctime(time.localtime(t))` (`klippy/queuelogger.py:53-54`).
/// `t` is wall-clock seconds; the conversion is UTC (see module docs).
pub fn asctime(epoch_secs: f64) -> String {
    let (y, m, d, hh, mm, ss, wd) = split_epoch(epoch_secs);
    const WEEKDAYS: [&str; 7] = ["Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"];
    const MONTHS: [&str; 12] =
        ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"];
    let wname = WEEKDAYS[wd as usize];
    let mname = MONTHS[(m - 1) as usize];
    // `%e` pads the day of month with a space (`"Jan  1"`).
    if d < 10 {
        format!("{wname} {mname}  {d} {hh:02}:{mm:02}:{ss:02} {y}")
    } else {
        format!("{wname} {mname} {d} {hh:02}:{mm:02}:{ss:02} {y}")
    }
}

/// `time.strftime("%Y-%m-%d", time.localtime(t))`: the rotated-file suffix
/// for `when='midnight'` (and for non-hourly `when` values generally).
pub fn date_suffix(epoch_secs: f64) -> String {
    let (y, m, d, _, _, _, _) = split_epoch(epoch_secs);
    format!("{y:04}-{m:02}-{d:02}")
}

/// `=============== Log rollover at %s ===============` (`klippy/queuelogger.py:52-54`).
pub fn rollover_header(epoch_secs: f64) -> String {
    percent_format(
        "=============== Log rollover at %s ===============",
        &[PctValue::Str(asctime(epoch_secs))],
    )
    .expect("static rollover format")
}

/// Next midnight strictly after `epoch_secs` (UTC). Python's
/// `computeRollover` for `midnight` returns the next local midnight,
/// honouring DST transitions; in UTC this is the next multiple of 86400.
pub fn next_midnight(epoch_secs: f64) -> f64 {
    let secs = cconv::f64_to_i64(epoch_secs);
    let days = secs.div_euclid(86400);
    ((days + 1) * 86400) as f64
}

/// Initial `rolloverAt` (`TimedRotatingFileHandler.__init__` calls
/// `computeRollover(current_time)` with `current_time = time.time()`).
pub fn rollover_at_start(now: f64) -> f64 {
    next_midnight(now)
}

/// `shouldRollover(record)`: the current time has reached `rolloverAt`.
pub fn should_rollover_at(rollover_at: f64, now: f64) -> bool {
    now >= rollover_at
}

// ---------------------------------------------------------------------------
// QueueListener (`klippy/queuelogger.py:24-56`).
// ---------------------------------------------------------------------------

struct Shared {
    base_filename: String,
    stream: Mutex<File>,
    rollover_info: Mutex<IndexMap<String, String>>,
    rollover_at: Mutex<f64>,
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
        let file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(&base)
            .map_err(|e| KlippyError::Other {
                class_name: "OSError".to_string(),
                message: format!("{e}"),
            })?;
        let now = time::wall_time();
        let shared = Arc::new(Shared {
            base_filename: base,
            stream: Mutex::new(file),
            rollover_info: Mutex::new(IndexMap::new()),
            rollover_at: Mutex::new(rollover_at_start(now)),
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
    pub fn rollover_at(&self) -> f64 {
        *self.shared.rollover_at.lock()
    }

    /// Set the `rolloverAt` used for the next rotation. Test/host hook
    /// mirroring the Python attribute of the same name.
    pub fn set_rollover_at(&self, value: f64) {
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

    /// `ql.doRollover()` (`klippy/queuelogger.py:48-56`).
    pub fn do_rollover(&self) -> KResult<()> {
        do_rollover_via(&self.shared)
    }

    /// `handle(record)` for one already-formatted pending record.
    /// Rotation is checked before writing, exactly like
    /// `BaseRotatingFileHandler.handle` -> `shouldRollover` ->
    /// `doRollover` ordering.
    pub fn handle_pending(&self, record: &PendingRecord) {
        if should_rollover_at(self.rollover_at(), time::wall_time()) {
            let _ = self.do_rollover();
        }
        let line = format_root_record(record.level, &record.message, record.exc_text.as_deref());
        let mut stream = self.shared.stream.lock();
        let _ = writeln!(stream, "{line}");
        let _ = stream.flush();
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

fn write_record_via(shared: &Arc<Shared>, record: &PendingRecord) {
    if should_rollover_at(*shared.rollover_at.lock(), time::wall_time()) {
        let _ = do_rollover_via(shared);
    }
    let line = format_root_record(record.level, &record.message, record.exc_text.as_deref());
    let mut stream = shared.stream.lock();
    if writeln!(stream, "{line}").is_err() {
        eprintln!("Failed to write log record");
    }
    let _ = stream.flush();
}

fn do_rollover_via(shared: &Arc<Shared>) -> KResult<()> {
    // `TimedRotatingFileHandler.doRollover`: close, rename, delete old
    // backups, reopen, then emit the info blocks.
    let now = time::wall_time();
    let prefix = rotated_prefix(&shared.base_filename);
    let suffix = date_suffix(now);
    let dest = format!("{prefix}.{suffix}{}", rotated_extension(&shared.base_filename));
    {
        let mut stream = shared.stream.lock();
        let _ = stream.flush();
        // Rename the current file into its dated backup.
        match fs::rename(&shared.base_filename, &dest) {
            Ok(()) => {}
            Err(_) => {
                // `FileHandler.doRollover` swallows rename failures; the
                // reopen below still proceeds like CPython's `delay=False`
                // path. Keep the old stream if rotation failed.
            }
        }
        let reopened = OpenOptions::new()
            .append(true)
            .create(true)
            .open(&shared.base_filename)
            .map_err(|e| KlippyError::Other {
                class_name: "OSError".to_string(),
                message: format!("{e}"),
            })?;
        *stream = reopened;
    }
    cleanup_old_rotated(&shared.base_filename, BACKUP_COUNT);
    *shared.rollover_at.lock() = next_midnight(now);
    let lines: Vec<String> = {
        let map = shared.rollover_info.lock();
        let mut names: Vec<String> = map.keys().cloned().collect();
        names.sort();
        names.iter().map(|n| map[n].clone()).collect()
    };
    let mut body_lines = lines;
    body_lines.push(rollover_header(now));
    let msg = body_lines.join("\n");
    let line = format_rollover_record(&msg);
    let mut stream = shared.stream.lock();
    let _ = writeln!(stream, "{line}");
    let _ = stream.flush();
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

fn split_base(filename: &str) -> (&str, &str) {
    match filename.rfind('/') {
        Some(i) => (&filename[..i + 1], &filename[i + 1..]),
        None => ("", filename),
    }
}

fn splitext(base: &str) -> (&str, &str) {
    match base.rfind('.') {
        Some(i) if i > 0 => (&base[..i], &base[i..]),
        _ => (base, ""),
    }
}

fn rotated_prefix(base_filename: &str) -> String {
    let (dir, file) = split_base(base_filename);
    let (stem, _) = splitext(file);
    format!("{dir}{stem}")
}

fn rotated_extension(base_filename: &str) -> String {
    let (_, file) = split_base(base_filename);
    let (_, ext) = splitext(file);
    ext.to_string()
}

fn cleanup_old_rotated(base_filename: &str, backup_count: u32) {
    if backup_count == 0 {
        return;
    }
    let prefix = rotated_prefix(base_filename);
    let (dir, _) = split_base(base_filename);
    let dir_path = if dir.is_empty() { ".".to_string() } else { dir.to_string() };
    let prefix_file = match prefix.rfind('/') {
        Some(i) => &prefix[i + 1..],
        None => &prefix[..],
    };
    let entries = match fs::read_dir(&dir_path) {
        Ok(e) => e,
        Err(_) => return,
    };
    let mut dated: Vec<String> = Vec::new();
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().into_owned();
        let Some(rest) = name.strip_prefix(&format!("{prefix_file}.")) else {
            continue;
        };
        // `YYYY-MM-DD` followed by the extension (or end of name).
        let bytes: Vec<char> = rest.chars().collect();
        if bytes.len() < 10 {
            continue;
        }
        let head: String = bytes.iter().take(10).collect();
        let digits: String = head.chars().filter(|c| *c >= '0' && *c <= '9').collect();
        let dashes: String = head.chars().filter(|c| *c == '-').collect();
        if digits.len() != 8 || dashes.len() != 2 {
            continue;
        }
        dated.push(entry.path().to_string_lossy().into_owned());
    }
    dated.sort();
    let keep = backup_count as usize;
    if dated.len() > keep {
        for path in dated.iter().take(dated.len() - keep) {
            let _ = fs::remove_file(path);
        }
    }
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

// Keep `Seek`/`SeekFrom`/`PathBuf` referenced for parity with the Python
// stream (file position after rotation). Unused imports would otherwise
// warn under `cargo clippy`.
#[allow(dead_code)]
fn _unused_stream_refs(_: &File, _: &PathBuf, _: SeekFrom) -> Result<(), std::io::Error> {
    Ok(())
}

/// Seek to the end of the current stream (append-mode invariant helper).
#[allow(dead_code)]
fn seek_to_end(stream: &mut File) -> std::io::Result<u64> {
    stream.seek(SeekFrom::End(0))
}
