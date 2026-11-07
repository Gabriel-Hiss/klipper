//! The `logging` module contract.
//!
//! Python klippy logs through the root logger: `logging.info("fmt %s", arg)`. The port formats
//! the message at the call site exactly as Python would (`pycompat::text::percent_format`), then
//! calls one of these functions. `queuelogger` (the ported `klippy/queuelogger.py`) installs the
//! sink that applies the root logger's level, formatter and handlers; until a sink is installed,
//! records go to stderr as `LEVEL:root:message` like Python's `logging.basicConfig` default.
//! `exception` appends the traceback text the caller supplies (the port has no Python traceback;
//! the normalizer table covers traceback bodies).

use std::sync::OnceLock;

/// Python logging levels.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Debug = 10,
    Info = 20,
    Warning = 30,
    Error = 40,
    Critical = 50,
}

impl Level {
    /// `logging.getLevelName(level)`.
    pub fn name(self) -> &'static str {
        match self {
            Level::Debug => "DEBUG",
            Level::Info => "INFO",
            Level::Warning => "WARNING",
            Level::Error => "ERROR",
            Level::Critical => "CRITICAL",
        }
    }
}

/// One log record as `logging.LogRecord` would carry it after `%` formatting.
pub struct Record<'a> {
    pub level: Level,
    pub message: &'a str,
    /// Traceback text for `logging.exception`, already formatted.
    pub exc_text: Option<&'a str>,
}

pub type Sink = Box<dyn Fn(&Record<'_>) + Send + Sync>;

static SINK: OnceLock<Sink> = OnceLock::new();

/// Installs the process-wide sink (called once by the ported `queuelogger`/klippy startup).
/// Returns `false` if a sink was already installed.
pub fn set_sink(sink: Sink) -> bool {
    SINK.set(sink).is_ok()
}

pub fn log(level: Level, message: &str, exc_text: Option<&str>) {
    let rec = Record { level, message, exc_text };
    match SINK.get() {
        Some(sink) => sink(&rec),
        None => {
            eprintln!("{}:root:{}", level.name(), message);
            if let Some(t) = exc_text {
                eprintln!("{t}");
            }
        }
    }
}

pub fn debug(message: &str) {
    log(Level::Debug, message, None)
}
pub fn info(message: &str) {
    log(Level::Info, message, None)
}
pub fn warning(message: &str) {
    log(Level::Warning, message, None)
}
pub fn error(message: &str) {
    log(Level::Error, message, None)
}
/// `logging.exception(msg)`: error level plus the traceback text.
pub fn exception(message: &str, traceback: &str) {
    log(Level::Error, message, Some(traceback))
}
