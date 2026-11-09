//! Parity tests for `klippy_core::queuelogger` (port of
//! `klippy/queuelogger.py`, both units `queuelogger.ModuleHeader` and
//! `queuelogger.ClearBgLogging`).
//!
//! Expected values come from the live reference through `pyoracle`
//! (CPython 3.8.2 + frozen `klippy/`): level constants, the default
//! formatter, `time.asctime`, `TimedRotatingFileHandler` rollover times,
//! rotated-file names and backup deletion, and the `rollover_info` dict
//! behaviour are compared directly. Both sides run in a zone with DST
//! (`TZ` below), so local-time handling is exercised. File delivery, FIFO
//! order and root-level filtering are checked against the Python listener
//! driven in the oracle with temp files, plus Rust-side round-trips
//! through the ported listener.

use klippy_core::queuelogger::*;
use klippy_api::log::{self, Level};
use kos::sync::Mutex;
use kos::time;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::sync::LazyLock;

/// A zone with DST (spring forward 2023-03-12 07:00 UTC, fall back
/// 2023-11-05 06:00 UTC), as a POSIX string so no tzdata is needed.
const TZ: &str = "EST5EDT,M3.2.0,M11.1.0";

static ZONE: LazyLock<()> = LazyLock::new(|| {
    std::env::set_var("TZ", TZ);
    time::tzset();
});

/// Every test enters the zone before any local-time conversion.
fn local_zone() {
    LazyLock::force(&ZONE);
}

static GLOBAL_TEST_LOCK: LazyLock<Mutex<()>> = LazyLock::new(|| Mutex::new(()));

fn records_root_check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "queuelogger".to_string(),
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

fn oracle() -> Option<pyoracle::Oracle> {
    local_zone();
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(&format!(
                "import logging, logging.handlers, time, queuelogger, os, tempfile\n\
                 os.environ['TZ'] = {TZ:?}\n\
                 time.tzset()"
            ))
            .expect("queuelogger oracle setup");
            Some(py)
        }
        None => {
            println!(
                "skipping queuelogger oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

fn temp_path(tag: &str) -> String {
    let dir = std::env::temp_dir();
    let id = std::process::id();
    let t = time::monotonic().to_bits();
    let p = dir.join(format!("klippy-ql-{id}-{t:x}-{tag}.log"));
    p.to_string_lossy().into_owned()
}

fn read_file(path: &str) -> String {
    std::fs::read_to_string(path).unwrap_or_default()
}

fn wait_contains(path: &str, needle: &str, secs: f64) -> String {
    let deadline = time::monotonic() + secs;
    loop {
        let content = read_file(path);
        if content.contains(needle) {
            return content;
        }
        if time::monotonic() >= deadline {
            panic!("timed out waiting for {needle:?} in {path}; got {content:?}");
        }
        time::sleep(0.005);
    }
}

fn cleanup(path: &str) {
    let _ = std::fs::remove_file(path);
    // Rotated backups: remove any sibling starting with the stem.
    if let Some(dir) = std::path::Path::new(path).parent() {
        let stem = format!(
            "{}.",
            std::path::Path::new(path)
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default()
        );
        if let Ok(entries) = std::fs::read_dir(dir) {
            for e in entries.flatten() {
                let name = e.file_name().to_string_lossy().into_owned();
                if name.starts_with(&stem) && name != *path {
                    let _ = std::fs::remove_file(e.path());
                }
            }
        }
    }
}

#[test]
fn levels_and_format_match_oracle() {
    let guard = GLOBAL_TEST_LOCK.lock();
    let _ = guard;
    let Some(mut py) = oracle() else { return };

    records_root_check(
        "python level constants",
        py.eval("(logging.DEBUG, logging.INFO, logging.WARNING, logging.ERROR, logging.CRITICAL)"),
        Value::Tuple(vec![
            Value::int(level_int(Level::Debug)),
            Value::int(level_int(Level::Info)),
            Value::int(level_int(Level::Warning)),
            Value::int(level_int(Level::Error)),
            Value::int(level_int(Level::Critical)),
        ]),
    );
    records_root_check(
        "level_from_int",
        py.eval("(logging.getLevelName(10), logging.getLevelName(20), logging.getLevelName(30), logging.getLevelName(40), logging.getLevelName(50))"),
        Value::Tuple(vec![
            Value::str(level_from_int(10).map(|l| l.name()).unwrap_or("")),
            Value::str(level_from_int(20).map(|l| l.name()).unwrap_or("")),
            Value::str(level_from_int(30).map(|l| l.name()).unwrap_or("")),
            Value::str(level_from_int(40).map(|l| l.name()).unwrap_or("")),
            Value::str(level_from_int(50).map(|l| l.name()).unwrap_or("")),
        ]),
    );
    // Default formatter: `logging.Formatter().format(record)` is
    // `'%(message)s'` (the default `fmt`), so only the expanded message
    // survives; level and logger name never reach the output.
    for (label, level, msg, expected_rust) in [
        ("info", Level::Info, "Starting Klippy...", "Starting Klippy..."),
        ("debug", Level::Debug, "a%b c", "a%b c"),
        ("warning multiline", Level::Warning, "line1\nline2", "line1\nline2"),
        ("empty", Level::Error, "", ""),
        ("critical", Level::Critical, "shutdown", "shutdown"),
    ] {
        let py_level = match level {
            Level::Debug => "logging.DEBUG",
            Level::Info => "logging.INFO",
            Level::Warning => "logging.WARNING",
            Level::Error => "logging.ERROR",
            Level::Critical => "logging.CRITICAL",
        };
        // Build the message expr with repr to avoid quoting issues.
        let msg_repr = format!("{msg:?}");
        let expr = format!(
            "logging.Formatter().format(logging.LogRecord('root', {py_level}, 'path', 1, {msg_repr}, (), None))"
        );
        records_root_check(
            &format!("default formatter {label}"),
            py.eval(&expr),
            Value::str(&format_root_record(level, msg, None)),
        );
        assert_eq!(format_root_record(level, msg, None), expected_rust);
        assert_eq!(format_root_record(level, msg, None), format_record(level, "anything", msg, None));
    }
    // The default formatter matches the source text of the constant.
    records_root_check(
        "DEFAULT_FORMAT",
        py.eval("logging.Formatter()._fmt"),
        Value::str(DEFAULT_FORMAT),
    );
    // Traceback text is appended with a newline. The cached `exc_text`
    // is set directly so the formatter appends it verbatim.
    let expr = "logging.Formatter().format((lambda r: (setattr(r, 'exc_text', 'Traceback (most recent call last):'), r)[1])(logging.LogRecord('root', logging.ERROR, 'path', 1, 'boom', (), None)))";
    records_root_check(
        "formatter with traceback",
        py.eval(expr),
        Value::str(&format_root_record(Level::Error, "boom", Some("Traceback (most recent call last):"))),
    );
}

/// `int(time.time())` inputs around both DST changes of [`TZ`].
const INT_TIMES: [i64; 11] = [
    0, 1, 86399, 86400, 1678593600, 1678604399, 1678604400, 1699156800, 1699163999, 1699164000,
    1700000000,
];

#[test]
fn time_helpers_match_oracle() {
    let _ = GLOBAL_TEST_LOCK.lock();
    let Some(mut py) = oracle() else { return };
    let epochs = [0.0, 86400.0, 1700000000.0, 1700000000.5, -1.0, 1234567890.0, 946684800.0, 4102444800.0];
    for t in epochs {
        let t_repr = format!("{t:?}");
        records_root_check(
            &format!("asctime {t}"),
            py.eval(&format!("time.asctime(time.localtime({t_repr}))")),
            Value::str(&asctime(t)),
        );
        records_root_check(
            &format!("rollover header {t}"),
            py.eval(&format!(
                "\"=============== Log rollover at %s ===============\" % (time.asctime(time.localtime({t_repr})),)"
            )),
            Value::str(&rollover_header(t)),
        );
    }
    py.exec(
        "_hfd, _hname = tempfile.mkstemp(suffix='.log')\nos.close(_hfd)\n\
         _h = logging.handlers.TimedRotatingFileHandler(_hname, when='midnight', backupCount=5)",
    )
    .expect("python handler");
    for t in INT_TIMES {
        records_root_check(
            &format!("computeRollover {t}"),
            py.eval(&format!("_h.computeRollover({t})")),
            Value::int(compute_rollover(t)),
        );
    }
    py.exec("_h.close()\nos.remove(_hname)").expect("python handler cleanup");
}

/// Python helper: build a `QueueListener` in a fresh directory holding
/// `files`, run `doRollover()` with `time.time()` returning `current`, and
/// report the directory listing and the new `rolloverAt`.
const PY_ROLLOVER: &str = r#"
def py_rollover(files, rollover_at, current):
    d = tempfile.mkdtemp()
    for name in files:
        open(os.path.join(d, name), 'w').close()
    ql = queuelogger.QueueListener(os.path.join(d, 'k.log'))
    ql.rolloverAt = rollover_at
    real_time = time.time
    time.time = lambda: current
    try:
        ql.doRollover()
    finally:
        time.time = real_time
    ql.stop()
    ql.close()
    names = sorted(os.listdir(d))
    for name in names:
        os.remove(os.path.join(d, name))
    os.rmdir(d)
    return (names, ql.rolloverAt)
"#;

fn rust_rollover(tag: &str, files: &[&str], rollover_at: i64, current: i64) -> Value {
    let dir = std::env::temp_dir().join(format!(
        "klippy-ql-{}-{:x}-{tag}",
        std::process::id(),
        time::monotonic().to_bits()
    ));
    std::fs::create_dir(&dir).expect("temp dir");
    for name in files {
        std::fs::write(dir.join(name), "").expect("seed file");
    }
    let listener = QueueListener::new(&dir.join("k.log").to_string_lossy()).expect("QueueListener");
    listener.set_rollover_at(rollover_at);
    listener.do_rollover_at(current).expect("do_rollover_at");
    listener.stop();
    let mut names: Vec<String> = std::fs::read_dir(&dir)
        .expect("list dir")
        .map(|e| e.expect("dir entry").file_name().to_string_lossy().into_owned())
        .collect();
    names.sort();
    std::fs::remove_dir_all(&dir).expect("remove temp dir");
    Value::Tuple(vec![
        Value::List(names.iter().map(|n| Value::str(n)).collect()),
        Value::int(listener.rollover_at()),
    ])
}

#[test]
fn rollover_matches_oracle() {
    let _ = GLOBAL_TEST_LOCK.lock();
    let Some(mut py) = oracle() else { return };
    py.exec(PY_ROLLOVER).expect("python rollover helper");
    // Rotation names and the next rolloverAt across both DST changes,
    // on time and late.
    for t0 in [1678500000, 1678590000, 1699090000, 1699150000, 1700000000] {
        let rollover_at = compute_rollover(t0);
        for current in [rollover_at, rollover_at + 5, rollover_at + 2 * 86400 + 100] {
            records_root_check(
                &format!("doRollover at {current} (rolloverAt {rollover_at})"),
                py.eval(&format!("py_rollover([], {rollover_at}, {current})")),
                rust_rollover("plan", &[], rollover_at, current),
            );
        }
    }
    // Backup deletion: only `k.log.YYYY-MM-DD[.ext]` names count, oldest
    // first, keeping `backupCount`; an existing destination is replaced.
    let files = [
        "k.log.2023-01-01", "k.log.2023-01-02", "k.log.2023-01-03", "k.log.2023-01-04",
        "k.log.2023-01-05.gz", "k.log.2023-01-06", "k.log.2023-11-13", "k.log.notes",
        "k.log.2023-1-07", "k.log.2023-01-08.tar.gz", "other.log.2023-01-01", "k.2023-01-09.log",
    ];
    let files_repr = format!("[{}]", files.iter().map(|f| format!("{f:?}")).collect::<Vec<_>>().join(", "));
    let rollover_at = compute_rollover(1699900000);
    records_root_check(
        "doRollover backups",
        py.eval(&format!("py_rollover({files_repr}, {rollover_at}, {})", rollover_at + 10)),
        rust_rollover("backups", &files, rollover_at, rollover_at + 10),
    );
}

#[test]
fn initial_rollover_from_mtime_matches_oracle() {
    let _ = GLOBAL_TEST_LOCK.lock();
    let Some(mut py) = oracle() else { return };
    for mtime in [1678590000i64, 1678604400, 1699160000, 1700000000] {
        py.exec(&format!(
            "_fd, _name = tempfile.mkstemp(suffix='.log')\nos.close(_fd)\n\
             os.utime(_name, ({mtime}, {mtime}))\n_ql = queuelogger.QueueListener(_name)\n\
             _ql.stop()\n_ql.close()\nos.remove(_name)"
        ))
        .expect("python listener");
        let path = temp_path("mtime");
        let file = std::fs::File::create(&path).expect("create log");
        let stamp = std::time::UNIX_EPOCH + std::time::Duration::from_secs(mtime as u64);
        file.set_modified(stamp).expect("set mtime");
        drop(file);
        let listener = QueueListener::new(&path).expect("QueueListener");
        records_root_check(
            &format!("initial rolloverAt for mtime {mtime}"),
            py.eval("_ql.rolloverAt"),
            Value::int(listener.rollover_at()),
        );
        listener.stop();
        cleanup(&path);
    }
}

#[test]
fn rollover_info_match_oracle() {
    let _ = GLOBAL_TEST_LOCK.lock();
    let Some(mut py) = oracle() else { return };
    py.exec(
        "import tempfile, os\nfd, fname = tempfile.mkstemp(suffix='.log')\nos.close(fd)\nql = queuelogger.QueueListener(fname)",
    )
    .expect("python QueueListener");
    let rust_path = temp_path("rollinfo");
    let rust = QueueListener::new(&rust_path).expect("rust QueueListener");

    let steps: Vec<(String, Option<String>)> = vec![
        ("versions".into(), Some("v1".into())),
        ("zebra".into(), Some("z".into())),
        ("versions".into(), Some("v2".into())),
        ("zebra".into(), None),
        ("alpha".into(), Some("a\nb".into())),
    ];
    for (name, info) in &steps {
        let name_repr = format!("{name:?}");
        match info {
            Some(text) => {
                let text_repr = format!("{text:?}");
                py.exec(&format!("ql.set_rollover_info({name_repr}, {text_repr})"))
                    .expect("py set");
                rust.set_rollover_info(name, Some(text));
            }
            None => {
                py.exec(&format!("ql.set_rollover_info({name_repr}, None)"))
                    .expect("py pop");
                rust.set_rollover_info(name, None);
            }
        }
        let expected = py
            .eval("sorted(ql.rollover_info.items())")
            .expect("py rollover_info");
        let sorted = rust.rollover_info_sorted();
        let actual = Value::List(
            sorted
                .iter()
                .map(|(k, v)| Value::Tuple(vec![Value::str(k), Value::str(v)]))
                .collect::<Vec<_>>(),
        );
        records_root_check("rollover_info sorted", Ok(expected), actual);
    }
    py.exec("ql.clear_rollover_info()").expect("py clear");
    rust.clear_rollover_info();
    assert!(rust.rollover_info_sorted().is_empty());
    py.exec("ql.stop()").expect("py stop");
    rust.stop();
    cleanup(&rust_path);
    let _ = py.eval("ql.rollover_info");
}

#[test]
fn do_rollover_writes_sorted_info_and_header() {
    local_zone();
    let rust_path = temp_path("rollover");
    let listener = QueueListener::new(&rust_path).expect("QueueListener");
    listener.set_rollover_info("zebra", Some("Z\nZ2"));
    listener.set_rollover_info("versions", Some("Args: ['klippy.py']\nGit version: 'abc'"));
    listener.do_rollover().expect("do_rollover");
    let content = read_file(&rust_path);
    assert!(
        content.contains("Args: ['klippy.py']"),
        "rolled content should contain versions block, got {content:?}"
    );
    assert!(
        content.contains("=============== Log rollover at"),
        "rolled content should contain header, got {content:?}"
    );
    // Sorted: versions precedes zebra.
    let vi = content.find("Args:").unwrap();
    let zi = content.find('Z').unwrap();
    assert!(vi < zi, "info blocks must be sorted by name");
    // The rollover record is a plain `%(message)s` record whose body is
    // the joined info blocks followed by the header; the header is the
    // last line of the file (`makeLogRecord` name is never rendered).
    let last_line = content.lines().next_back().unwrap_or("");
    assert!(
        last_line.starts_with("=============== Log rollover at "),
        "rollover header must be the last line, got {last_line:?}"
    );
    listener.stop();
    cleanup(&rust_path);
}

#[test]
fn file_delivery_fifo_and_format() {
    local_zone();
    let path = temp_path("fifo");
    let listener = QueueListener::new(&path).expect("QueueListener");
    listener.emit(PendingRecord::new(Level::Info, "Starting Klippy...", None));
    listener.emit(PendingRecord::new(Level::Warning, "warn %s", None));
    listener.emit(PendingRecord::new(
        Level::Error,
        "boom",
        Some("Traceback (most recent call last):".to_string()),
    ));
    let content = wait_contains(&path, "boom", 5.0);
    // Plain `%(message)s` output: no level/name fields.
    assert!(content.contains("Starting Klippy..."), "{content:?}");
    assert!(content.contains("warn %s"), "{content:?}");
    assert!(content.contains("boom\nTraceback"), "{content:?}");
    assert!(!content.contains("INFO:"), "no level prefix, got {content:?}");
    // FIFO: first line is the first emit.
    let first = content.lines().next().unwrap_or("");
    assert_eq!(first, "Starting Klippy...");
    listener.stop();
    // After stop the thread is joined; a later emit only queues.
    listener.emit(PendingRecord::new(Level::Info, "after stop", None));
    assert_eq!(listener.queue_size(), 1);
    cleanup(&path);
}

#[test]
fn file_bytes_match_python_listener() {
    let _ = GLOBAL_TEST_LOCK.lock();
    let Some(mut py) = oracle() else { return };
    // Python side: drive the reference QueueListener with the same
    // records through `handle` (the path the background thread takes).
    py.exec(
        "import tempfile, os\nfd, pfile = tempfile.mkstemp(suffix='.log')\nos.close(fd)\nql2 = queuelogger.QueueListener(pfile)\nfor _msg, _lvl in [('Starting Klippy...', logging.INFO), ('warn %s', logging.WARNING)]:\n    ql2.handle(logging.makeLogRecord({'msg': _msg, 'level': _lvl}))\n_r = logging.makeLogRecord({'msg': 'boom', 'level': logging.ERROR})\n_r.exc_text = 'Traceback (most recent call last):'\nql2.handle(_r)",
    )
    .expect("python listener records");
    // Rust side: same records through the background queue.
    let path = temp_path("bytes");
    let listener = QueueListener::new(&path).expect("QueueListener");
    listener.emit(PendingRecord::new(Level::Info, "Starting Klippy...", None));
    listener.emit(PendingRecord::new(Level::Warning, "warn %s", None));
    listener.emit(PendingRecord::new(
        Level::Error,
        "boom",
        Some("Traceback (most recent call last):".to_string()),
    ));
    let rust_content = wait_contains(&path, "Traceback", 5.0);
    listener.stop();
    let python_value = py.eval("open(pfile).read()").expect("py file read");
    let python_content = match python_value {
        Value::Str(s) => s.to_string(),
        other => panic!("py file read returned {other:?}"),
    };
    assert_eq!(rust_content, python_content, "file bytes must match reference");
    cleanup(&path);
}

#[test]
fn handle_pending_writes_without_queue() {
    local_zone();
    let path = temp_path("handle");
    let listener = QueueListener::new(&path).expect("QueueListener");
    listener.handle_pending(&PendingRecord::new(Level::Debug, "direct", None));
    let content = read_file(&path);
    assert!(content.contains("direct"), "{content:?}");
    assert!(!content.contains("DEBUG:"), "no level prefix, got {content:?}");
    listener.stop();
    cleanup(&path);
}

#[test]
fn setup_clear_and_level_filtering() {
    let guard = GLOBAL_TEST_LOCK.lock();
    let _ = guard;
    let Some(mut py) = oracle() else { return };
    // Rust side: set up global listener and observe filtering.
    let path = temp_path("setup");
    let listener = setup_bg_logging(&path, Level::Info).expect("rust setup");
    assert!(main_handler_installed());
    assert_eq!(root_level(), Level::Info);
    // Python side: root level after setup.
    py.exec("import queuelogger as _ql\nimport tempfile, os\nfd, pf = tempfile.mkstemp(suffix='.log')\nos.close(fd)\n_q = _ql.setup_bg_logging(pf, logging.INFO)")
        .expect("py setup");
    records_root_check(
        "python root level after INFO setup",
        py.eval("logging.getLogger().level"),
        Value::int(level_int(root_level())),
    );
    assert!(main_handler_installed());
    assert_eq!(root_level(), Level::Info);
    log::debug("debug dropped");
    log::info("info kept");
    log::warning("warning kept");
    let _ = wait_contains(&path, "info kept", 5.0);
    // Give the debug record a chance to (not) land.
    time::sleep(0.05);
    let content = read_file(&path);
    let _ = content;
    assert!(content.contains("info kept"), "{content:?}");
    assert!(content.contains("warning kept"), "{content:?}");
    assert!(!content.contains("debug dropped"), "{content:?}");
    // clear_bg_logging: root reverts to WARNING, handler removed.
    py.exec("queuelogger.clear_bg_logging()").expect("py clear");
    clear_bg_logging();
    assert!(!main_handler_installed());
    assert_eq!(root_level(), Level::Warning);
    log::info("after clear info");
    time::sleep(0.05);
    let after = read_file(&path);
    assert!(!after.contains("after clear info"), "cleared handler must not write, got {after:?}");
    listener.stop();
    cleanup(&path);
    // Python comparison for root level after clear.
    records_root_check(
        "python root level after clear",
        py.eval("logging.getLogger().level"),
        Value::int(level_int(root_level())),
    );
}


#[test]
fn open_failure_is_oserror() {
    local_zone();
    let bad = "/definitely_missing_klippy_dir_xyz/klippy.log";
    let err = QueueListener::new(bad).expect_err("open must fail");
    assert_eq!(err.class_name(), "OSError");
    assert!(!err.message().is_empty());
}
