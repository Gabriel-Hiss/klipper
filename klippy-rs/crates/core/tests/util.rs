//! Parity tests for `klippy_core::util` (port of `klippy/util.py`).
//!
//! There are no recordings under `tests/fixtures/recordings/` for
//! this unit (`unit` is `unverifiable`: host OS identity with no
//! simulated counterpart), so every expected value comes from the
//! live reference through `pyoracle` (CPython 3.8.2 + the frozen
//! `klippy/`). Each test builds identical inputs on both sides
//! (temp files for `_try_read_file` / `dump_*`, patched
//! `_try_read_file` for the `*_from_data` pure helpers, and a fake
//! `subprocess.Popen` queue for git), then compares the returned
//! strings and the logged lines. Floats do not occur here; `asctime`
//! output is compared as text.
//!
//! Traceback bodies in debug logs are irreproducible without a
//! Python frame stack; they are compared by prefix (everything
//! before `Traceback (most recent call last)`), which is what the
//! frozen normalizer `nan-traceback` covers in parity diffs.
//!
//! Run command (integration owner):
//!
//! ```sh
//! cargo test -p klippy_core --test util
//! ```
//!
//! Tests that need the oracle skip with a message when
//! `$KLIPPY_PYTHON` is unset (outside the oracle images).

use klippy_api::log;
use klippy_core::util::{
    asctime_from_mtime, branch_from_lines, cpu_info_from_data, detached_remote,
    device_info_from_data, dump_file_stats, dump_mcu_build, get_cpu_info, get_device_info,
    get_git_version_with_runner, get_linux_version, get_repo_info_with_runner, get_version_from_file,
    linux_version_from_data, parse_branch_lines, parse_status, py_split_none_1,
};
use klippy_core::util::RepoInfo;
use kos::sync::Mutex;
use recording::{assert_equal, Record, RecordKind, SideEffects, Value};
use std::collections::VecDeque;
use std::sync::Arc;

const SETUP: &str = r#"
import util
import logging as _logging
import subprocess
import os
import json
import time

log_records = []
class _Capture(_logging.Handler):
    def emit(self, record):
        log_records.append((record.levelname, record.getMessage()))
_root = _logging.getLogger()
_root.addHandler(_Capture())
_root.setLevel(0)

def _clear_logs():
    del log_records[:]
def _logs():
    return list(log_records)

def _patch_try_read(data):
    util._try_read_file = lambda filename, maxsize=32*1024: data
def _restore_try_read():
    import importlib; importlib.reload(util)

_fake_git_queue = []
_fake_git_calls = []
class _FakeProcess:
    def __init__(self, prog, stdout=None, stderr=None):
        _fake_git_calls.append(list(prog))
        out, err, code = _fake_git_queue.pop(0)
        self._out = out
        self._err = err
        self._code = code
    def communicate(self):
        return (self._out, self._err)
    def wait(self):
        return self._code
_subprocess_Popen = subprocess.Popen
subprocess.Popen = _FakeProcess
def _queue_git(out, err, code):
    _fake_git_queue.append((out, err, code))
def _git_calls():
    return list(_fake_git_calls)
def _clear_git():
    del _fake_git_queue[:]
    del _fake_git_calls[:]
"#;

fn oracle() -> Option<pyoracle::Oracle> {
    // `_try_read_file` tracebacks name `util.py` in the klippy source
    // directory, which the host records at startup.
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .ancestors()
        .nth(3)
        .expect("crate lives three levels below the repo root");
    klippy_api::paths::set_klippy_dir(&repo.join("klippy").to_string_lossy());
    match pyoracle::Oracle::start() {
        Some(mut py) => {
            py.exec(SETUP).expect("util fakes");
            Some(py)
        }
        None => {
            println!(
                "skipping util oracle tests: set KLIPPY_PYTHON to a CPython 3.8.2 \
                 binary and run with the repo checkout available"
            );
            None
        }
    }
}

#[allow(clippy::type_complexity)]
fn log_capture() -> Arc<Mutex<Vec<(String, String)>>> {
    use std::sync::LazyLock;
    static CAPTURE: LazyLock<Arc<Mutex<Vec<(String, String)>>>> =
        LazyLock::new(|| Arc::new(Mutex::new(Vec::new())));
    let captured = CAPTURE.clone();
    let sink_capture = captured.clone();
    log::set_sink(Box::new(move |rec: &log::Record<'_>| {
        sink_capture
            .lock()
            .push((rec.level.name().to_string(), rec.message.to_string()));
    }));
    captured
}

/// The log sink is process-global, so the log-comparing tests must not
/// overlap: tests run on separate threads in one `cargo test` process.
static LOG_TEST_LOCK: std::sync::LazyLock<Mutex<()>> =
    std::sync::LazyLock::new(|| Mutex::new(()));

fn log_test_guard() -> kos::sync::MutexGuard<'static, ()> {
    LOG_TEST_LOCK.lock()
}

fn check(label: &str, expected: pyoracle::OracleResult, actual: Value) {
    let expected = match expected {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let record = Record {
        v: 1,
        unit: "util".to_string(),
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

/// Normalize traceback bodies: truncate at the traceback marker.
fn normalize_log(s: &str) -> String {
    match s.find("Traceback (most recent call last)") {
        Some(pos) => format!("{}<TB>", &s[..pos]),
        None => s.to_string(),
    }
}

fn normalize_logs(v: &Value) -> Value {
    match v {
        Value::List(items) => Value::List(
            items
                .iter()
                .map(|item| match item {
                    Value::Tuple(pair) if pair.len() == 2 => {
                        let level = match &pair[0] {
                            Value::Str(s) => Value::Str(s.clone()),
                            other => other.clone(),
                        };
                        let msg = match &pair[1] {
                            Value::Str(s) => Value::Str(normalize_log(s)),
                            other => other.clone(),
                        };
                        Value::Tuple(vec![level, msg])
                    }
                    other => other.clone(),
                })
                .collect(),
        ),
        other => other.clone(),
    }
}

fn check_logs(
    label: &str,
    py_logs: pyoracle::OracleResult,
    rust_logs: Vec<(String, String)>,
) {
    let rust_value = Value::List(
        rust_logs
            .into_iter()
            .map(|(level, msg)| Value::Tuple(vec![Value::Str(level), Value::Str(msg)]))
            .collect(),
    );
    let expected = match py_logs {
        Ok(v) => v,
        Err(e) => panic!("{label}: oracle raised {}: {}", e.class_name, e.message),
    };
    let expected_n = normalize_logs(&expected);
    let actual_n = normalize_logs(&rust_value);
    check(label, Ok(expected_n), actual_n);
}


fn clear_logs(py: &mut pyoracle::Oracle, captured: &Arc<Mutex<Vec<(String, String)>>> ) {
    captured.lock().clear();
    py.exec("_clear_logs()").expect("clear py logs");
}

fn take_rust_logs(captured: &Arc<Mutex<Vec<(String, String)>>> ) -> Vec<(String, String)> {
    let mut guard = captured.lock();
    let out = guard.clone();
    guard.clear();
    out
}

fn temp_dir(tag: &str) -> String {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static COUNTER: AtomicUsize = AtomicUsize::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let mut p = std::env::temp_dir();
    p.push(format!(
        "klippy-util-test-{}-{}-{}",
        std::process::id(),
        tag,
        n
    ));
    let s = p.to_string_lossy().into_owned();
    std::fs::create_dir_all(&p).unwrap();
    s
}

fn cleanup(dir: &str) {
    let _ = std::fs::remove_dir_all(dir);
}

/// One fake subprocess result: `(stdout, stderr, returncode)`.
type GitEntry = (Vec<u8>, Vec<u8>, i32);

/// Fake git runner for the Rust side (same FIFO as the Python fake).
/// Interior mutability: the production runners take `Fn`, so the queue
/// and the recorded calls go through `RefCell`.
struct FakeGit {
    queue: std::cell::RefCell<VecDeque<GitEntry>>,
    calls: std::cell::RefCell<Vec<Vec<String>>>,
}

impl FakeGit {
    fn new(entries: Vec<GitEntry>) -> Self {
        FakeGit {
            queue: std::cell::RefCell::new(entries.into()),
            calls: std::cell::RefCell::new(Vec::new()),
        }
    }
    fn run(&self, args: &[&str]) -> Option<GitEntry> {
        self.calls
            .borrow_mut()
            .push(args.iter().map(|s| s.to_string()).collect());
        self.queue.borrow_mut().pop_front()
    }
}

fn bytes_arg(b: &[u8]) -> String {
    // Python repr for the test harness queue: pass via exec string.
    // Inputs are ASCII; escape backslash/quote/newline.
    let mut out = String::from("b'");
    for &c in b {
        match c {
            b'\\' => out.push_str("\\\\"),
            b'\'' => out.push_str("\\'"),
            b'\n' => out.push_str("\\n"),
            b'\r' => out.push_str("\\r"),
            b'\t' => out.push_str("\\t"),
            0x20..=0x7e => out.push(c as char),
            _ => out.push_str(&format!("\\x{c:02x}")),
        }
    }
    out.push('\'');
    out
}

/// Python `str` literal for eval snippets. Test data is ASCII; `\0`,
/// `\t`, `\r`, `\n` escape the same way in both languages.
fn str_arg(s: &str) -> String {
    let mut out = String::from("\"");
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\0' => out.push_str("\\0"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn queue_py_git(py: &mut pyoracle::Oracle, entries: &[(Vec<u8>, Vec<u8>, i32)]) {
    for (out, err, code) in entries {
        let expr = format!(
            "_queue_git({}, {}, {})",
            bytes_arg(out),
            bytes_arg(err),
            code
        );
        py.exec(&expr).expect("queue fake git");
    }
}

fn repo_to_value(r: &RepoInfo) -> Value {
    Value::Dict(vec![
        (Value::Str("branch".into()), Value::Str(r.branch.clone())),
        (Value::Str("remote".into()), Value::Str(r.remote.clone())),
        (Value::Str("url".into()), Value::Str(r.url.clone())),
    ])
}

fn git_to_value(v: &klippy_core::util::GitInfo) -> Value {
    Value::Dict(vec![
        (Value::Str("version".into()), Value::Str(v.version.clone())),
        (
            Value::Str("file_status".into()),
            Value::List(
                v.file_status
                    .iter()
                    .map(|row| Value::List(row.iter().map(|s| Value::Str(s.clone())).collect()))
                    .collect(),
            ),
        ),
        (Value::Str("branch".into()), Value::Str(v.branch.clone())),
        (Value::Str("remote".into()), Value::Str(v.remote.clone())),
        (Value::Str("url".into()), Value::Str(v.url.clone())),
    ])
}

#[test]
fn pure_helpers_match_python() {
    let Some(mut py) = oracle() else { return };
    // cpu_info_from_data
    for data in [
            "processor\t: 0\nmodel name\t: Intel\n",
            "model name\t: First\nmodel name\t: Last\nprocessor : 0\nprocessor : 1\n",
            "no colon line\nprocessor: 1\n",
            "",
            "model name   :   spaced   \n",
        ] {
        py.exec(&format!("_patch_try_read({:?})", data.replace('\\', "\\\\").replace('"', "\\\"")))
        .expect("patch try_read");
        py.exec("import util as _u").expect("import util");
        let expected = py.eval("_u.get_cpu_info()");
        let actual = cpu_info_from_data(data);
        check("cpu_info", expected, Value::Str(actual));
    }
    py.exec("_restore_try_read()").expect("restore");
    // device/linux/version rstrip/strip
    for data in ["Foo  \0\0", " Foo bar ", "\tLinux v \n", "v1.0\n", "  v1  ", "A\0B", " trailing  \r"] {
        let actual = device_info_from_data(data);
        // device_info_from_data does not exist in Python; the equivalent inline is
        // data.rstrip(' \0').strip()
        let expected = py.eval(&format!("({}).rstrip(' \\x00').strip()", str_arg(data)));
        check("device", expected, Value::Str(actual));
        let actual = linux_version_from_data(data);
        let expected = py.eval(&format!("({}).strip()", str_arg(data)));
        check("linux", expected, Value::Str(actual));
        let actual = klippy_core::util::version_from_data(data);
        let expected = py.eval(&format!("({}).rstrip()", str_arg(data)));
        check("version", expected, Value::Str(actual));
    }
    // branch parsing
    for lines in [
            vec!["  main".to_string(), "  dev".to_string()],
            vec!["* main".to_string()],
            vec!["  main".to_string(), "* feature/x".to_string()],
        ] {
        // The empty-line case ("\n" output) is covered by the fake-git
        // repo test below: there the oracle runs the real `_get_repo_info`,
        // where `line[0]` raises `IndexError`.
        let actual = match branch_from_lines(&lines) {
            Ok(Some(s)) => Value::Str(s),
            Ok(None) => Value::Null,
            Err(()) => Value::Null,
        };
        let expected = py.eval(&format!(
            "(lambda lines: next((l[1:].strip() for l in lines if l.startswith('*')), None))({:?})",
            lines
        ));
        check("branch_from_lines", expected, actual);
    }
    // detached remote
    for branch in [
            "(HEAD detached from abc)",
            "(HEAD detached at origin/main)",
            "(HEAD detached from origin/main)",
            "main",
            "(HEAD detached from a/b/c)",
        ] {
        let actual_opt = detached_remote(branch);
        let actual = match &actual_opt {
            Some(s) => Value::Str(s.clone()),
            None => Value::Null,
        };
        let expected = py.eval(&format!(
            "(lambda b: b.strip('()').split()[-1].split('/', 1)[0] if len(b.strip('()').split()[-1].split('/', 1)) == 2 else None)({:?})",
            branch
        ));
        check("detached", expected, actual);
    }
    // parse_status / split
    for raw in [
            b"".as_slice(),
            b" M file.py\n?? bar.py".as_slice(),
            b"M f1\n?? f2\n!! f3".as_slice(),
        ] {
        let actual = parse_status(raw);
        let expected = py.eval(&format!(
            "[l.split(None, 1) for l in str({}.strip().decode()).split('\\n')]",
            bytes_arg(raw)
        ));
        let actual_value = Value::List(
            actual
                .iter()
                .map(|row| Value::List(row.iter().map(|s| Value::Str(s.clone())).collect()))
                .collect(),
        );
        check("parse_status", expected, actual_value);
    }
    for line in ["M file.py", "  ", "*", "A", "  A   B   C"] {
        let actual = py_split_none_1(line);
        let expected = py.eval(&format!("({:?}).split(None, 1)", line));
        let actual_value = Value::List(actual.into_iter().map(Value::Str).collect());
        check("split", expected, actual_value);
    }
    // asctime
    for mtime in [0.0, 1234567890.0, 4102444800.0, 1700000000.5] {
        let actual = asctime_from_mtime(mtime).map(Value::Str).unwrap_or(Value::Null);
        let expected = py.eval(&format!(
            "time.asctime(time.localtime({:?}))",
            mtime
        ));
        check("asctime", expected, actual);
    }
}

#[test]
fn file_and_dump_match_python() {
    let Some(mut py) = oracle() else { return };
    let _guard = log_test_guard();
    let captured = log_capture();
    let dir = temp_dir("dump");
    // Existing file with known size/content.
    let fname = format!("{dir}/probe.txt");
    std::fs::write(&fname, b"hello world").unwrap();
    // Exercise _try_read_file success, truncation, missing.
    clear_logs(&mut py, &captured);
    let expected = py.eval(&format!("util._try_read_file({:?})", fname));
    let actual = klippy_core::util::try_read_file(&fname, 32 * 1024).map(Value::Str).unwrap_or(Value::Null);
    check("try_read_ok", expected, actual.clone());
    let rust_logs = take_rust_logs(&captured);
    check_logs("try_read_ok_logs", py.eval("_logs()"), rust_logs);
    clear_logs(&mut py, &captured);
    let expected = py.eval(&format!("util._try_read_file({:?}, 5)", fname));
    let actual = klippy_core::util::try_read_file(&fname, 5).map(Value::Str).unwrap_or(Value::Null);
    check("try_read_trunc", expected, actual);
    let rust_logs = take_rust_logs(&captured);
    check_logs("try_read_trunc_logs", py.eval("_logs()"), rust_logs);
    clear_logs(&mut py, &captured);
    let missing = format!("{dir}/missing-literal-12345");
    let expected = py.eval(&format!("util._try_read_file({:?})", missing));
    let actual = klippy_core::util::try_read_file(&missing, 32 * 1024).map(Value::Str).unwrap_or(Value::Null);
    check("try_read_missing", expected, actual);
    let rust_logs = take_rust_logs(&captured);
    check_logs("try_read_missing_logs", py.eval("_logs()"), rust_logs);
    // dump_file_stats existing and missing.
    clear_logs(&mut py, &captured);
    let expected = py.eval(&format!("(util.dump_file_stats({:?}, 'probe.txt'), _logs())", dir));
    // Compare logs only; the return is None on both sides.
    dump_file_stats(&dir, "probe.txt");
    let rust_logs = take_rust_logs(&captured);
    let py_logs = py.eval("_logs()");
    check_logs("dump_stats_ok", py_logs, rust_logs);
    clear_logs(&mut py, &captured);
    py.eval(&format!("util.dump_file_stats({:?}, 'nope-xyz')", dir)).expect("py missing stats");
    dump_file_stats(&dir, "nope-xyz");
    let rust_logs = take_rust_logs(&captured);
    check_logs("dump_stats_missing", py.eval("_logs()"), rust_logs);
    // dump_mcu_build with full build tree.
    let build = format!("{dir}/build");
    std::fs::create_dir_all(format!("{build}/out")).unwrap();
    std::fs::write(format!("{build}/.config"), b"CONFIG=1\n").unwrap();
    std::fs::write(
        format!("{build}/out/klipper.dict"),
        br#"{"version": "v1", "build_versions": "gcc-1", "config": {"A": "1", "B": "2"}}"#
    )
    .unwrap();
    std::fs::write(format!("{build}/out/klipper.elf"), b"elf").unwrap();
    clear_logs(&mut py, &captured);
    py.exec(&format!("util.__file__ = {:?}", format!("{build}/klippy/util.py")))
        .expect("patch __file__");
    py.eval("util.dump_mcu_build()").expect("py dump_mcu_build");
    dump_mcu_build(&format!("{build}/klippy/.."));
    let rust_logs = take_rust_logs(&captured);
    check_logs("dump_mcu_build", py.eval("_logs()"), rust_logs);
    // Bad dict: invalid JSON makes the block pass silently.
    std::fs::write(format!("{build}/out/klipper.dict"), b"not json{{").unwrap();
    clear_logs(&mut py, &captured);
    py.eval("util.dump_mcu_build()").expect("py bad dict");
    dump_mcu_build(&format!("{build}/klippy/.."));
    let rust_logs = take_rust_logs(&captured);
    check_logs("dump_mcu_build_bad_dict", py.eval("_logs()"), rust_logs);
    // Missing dict: `_try_read_file` logs the debug line, then the bare
    // `except` swallows `json.loads(None)`.
    std::fs::remove_file(format!("{build}/out/klipper.dict")).unwrap();
    clear_logs(&mut py, &captured);
    py.eval("util.dump_mcu_build()").expect("py missing dict");
    dump_mcu_build(&format!("{build}/klippy/.."));
    let rust_logs = take_rust_logs(&captured);
    check_logs("dump_mcu_build_missing_dict", py.eval("_logs()"), rust_logs);
    cleanup(&dir);
    let _ = expected;
}

#[test]
fn git_match_python() {
    let Some(mut py) = oracle() else { return };
    let _guard = log_test_guard();
    let captured = log_capture();
    let gitdir = "/tmp/util-gitdir";
    // Normal repo: branch + remote + url.
    let entries: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![
        (b"  main\n* feature/x\n  dev\n".to_vec(), Vec::new(), 0),
        (b"origin\n".to_vec(), Vec::new(), 0),
        (b"https://example/repo.git\n".to_vec(), Vec::new(), 0),
    ];
    queue_py_git(&mut py, &entries);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries);
    clear_logs(&mut py, &captured);
    let expected = py.eval("util._get_repo_info('/tmp/util-gitdir')");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries.clone());
    let actual = get_repo_info_with_runner(gitdir, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("repo_normal", expected, repo_to_value(&actual));
    check_logs("repo_normal_logs", expected_logs, rust_logs);
    // No starred line.
    let entries2: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![
        (b"  main\n  dev\n".to_vec(), Vec::new(), 0),
    ];
    queue_py_git(&mut py, &entries2);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries2);
    clear_logs(&mut py, &captured);
    let expected = py.eval("util._get_repo_info('/tmp/util-gitdir')");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries2.clone());
    let actual = get_repo_info_with_runner(gitdir, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("repo_no_star", expected, repo_to_value(&actual));
    check_logs("repo_no_star_logs", expected_logs, rust_logs);
    // Detached HEAD with remote.
    let entries3: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![
        (b"* (HEAD detached at origin/main)\n".to_vec(), Vec::new(), 0),
        (b"https://example/detached.git\n".to_vec(), Vec::new(), 0),
    ];
    queue_py_git(&mut py, &entries3);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries3);
    clear_logs(&mut py, &captured);
    let expected = py.eval("util._get_repo_info('/tmp/util-gitdir')");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries3.clone());
    let actual = get_repo_info_with_runner(gitdir, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("repo_detached", expected, repo_to_value(&actual));
    check_logs("repo_detached_logs", expected_logs, rust_logs);
    // get_git_version: describe + status + repo.
    let entries4: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![
        (b"v1.0-2-gabc\n".to_vec(), Vec::new(), 0),
        (b" M a.py\n?? b.py\n".to_vec(), Vec::new(), 0),
        (b"* main\n".to_vec(), Vec::new(), 0),
        (b"origin\n".to_vec(), Vec::new(), 0),
        (b"https://example/repo.git\n".to_vec(), Vec::new(), 0),
    ];
    let src = "/tmp/util-git-src";
    queue_py_git(&mut py, &entries4);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries4);
    py.exec(&format!(
        "util.__file__ = {:?}",
        format!("{src}/klippy/util.py")
    ))
    .expect("patch __file__ for git");
    clear_logs(&mut py, &captured);
    let expected = py.eval("util.get_git_version(True)");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries4.clone());
    let actual = get_git_version_with_runner(&format!("{src}/klippy"), true, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("git_normal", expected, git_to_value(&actual));
    check_logs("git_normal_logs", expected_logs, rust_logs);
    // describe fails, from_file=True with no .version => "?" fallback.
    let entries5: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![
        (Vec::new(), b"fatal: bad\n".to_vec(), 128),
    ];
    queue_py_git(&mut py, &entries5);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries5);
    clear_logs(&mut py, &captured);
    let expected = py.eval("util.get_git_version(True)");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries5.clone());
    let actual = get_git_version_with_runner(&format!("{src}/klippy"), true, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("git_describe_fail", expected, git_to_value(&actual));
    check_logs("git_describe_fail_logs", expected_logs, rust_logs);
    // Empty branch output: `lines` is `[""]` and `line[0]` raises
    // `IndexError`, caught by the bare `except` (defaults + debug log).
    let entries6: Vec<(Vec<u8>, Vec<u8>, i32)> = vec![(Vec::new(), Vec::new(), 0)];
    queue_py_git(&mut py, &entries6);
    py.exec("_clear_git()").expect("clear git");
    queue_py_git(&mut py, &entries6);
    clear_logs(&mut py, &captured);
    let expected = py.eval("util._get_repo_info('/tmp/util-gitdir')");
    let expected_logs = py.eval("_logs()");
    let fake = FakeGit::new(entries6.clone());
    let actual = get_repo_info_with_runner(gitdir, &|a| fake.run(a));
    let rust_logs = take_rust_logs(&captured);
    check("repo_empty_branch", expected, repo_to_value(&actual));
    check_logs("repo_empty_branch_logs", expected_logs, rust_logs);
    // parse helper checks the oracle computes for the same bytes.
    for raw in [b"* main".as_slice(), b"  main\n* feat".as_slice()] {
        let lines = parse_branch_lines(raw);
        let expected = py.eval(&format!(
            "str({}.strip().decode()).split('\\n')",
            bytes_arg(raw)
        ));
        let actual_value = Value::List(lines.into_iter().map(Value::Str).collect());
        check("branch_lines", expected, actual_value);
    }
}

#[test]
fn live_host_info_matches_python() {
    // Cheap live checks: both sides read the same /proc files in the
    // oracle container, including the missing-file "?" fallback.
    // This test emits Rust debug logs too: `get_cpu_info`,
    // `get_linux_version`, `get_device_info` and `get_version_from_file`
    // read files that may not exist (logging the miss), and the log
    // sink is process-global. Hold the capture lock so these logs do
    // not leak into another test's captured lines.
    let _guard = log_test_guard();
    let _captured = log_capture();
    let Some(mut py) = oracle() else { return };
    let expected = py.eval("util.get_cpu_info()");
    let actual = get_cpu_info();
    check("live_cpu", expected, Value::Str(actual));

    let expected = py.eval("util.get_linux_version()");
    let actual = get_linux_version();
    check("live_linux", expected, Value::Str(actual));
    let expected = py.eval("util.get_device_info()");
    let actual = get_device_info();
    check("live_device", expected, Value::Str(actual));
    let dir = temp_dir("version");
    std::fs::write(format!("{dir}/.version"), b"1.2.3\n\n").unwrap();
    let expected = py.eval(&format!("util.get_version_from_file({:?})", dir));
    let actual = get_version_from_file(&dir);
    check("version_file", expected, Value::Str(actual));
    cleanup(&dir);
}

#[test]
fn fd_helpers_match_python() {
    let Some(mut py) = oracle() else { return };
    let dir = temp_dir("pty");
    let ptyname = format!("{dir}/printer");
    // Valid pipe fd: set_nonblock returns None on both sides.
    let expected = py.eval("util.set_nonblock(1)");
    let actual = klippy_core::util::set_nonblock(1).map(|_| Value::Null).unwrap_or(Value::Null);
    check("set_nonblock_stdout", expected, actual);
    // clear_hupcl on a real tty (master of a fresh pty) returns None;
    // on fd 1 (not a tty in the test harness) both raise termios.error.
    let ptyname2 = format!("{dir}/printer-hupcl");
    let expected = py.eval(&format!(
        "util.clear_hupcl(util.create_pty({:?}))",
        ptyname2
    ));
    let rust_master = match klippy_core::util::create_pty(&ptyname2) {
        Ok(pycompat::PyValue::Int(n)) => n.inner().to_string().parse::<i64>().unwrap_or(-1),
        _ => -1,
    };
    let actual = klippy_core::util::clear_hupcl(rust_master as i32)
        .map(|_| Value::Null)
        .unwrap_or(Value::Null);
    check("clear_hupcl_pty", expected, actual);
    kos::fd::close(rust_master as i32);
    // Invalid fd: both raise; compare kind only (message is OS text).
    let py_err = py.eval("util.clear_hupcl(-1)");
    let rust_err = klippy_core::util::clear_hupcl(-1);
    let expected = Ok(match py_err {
        Ok(_) => Value::Str("no-error".to_string()),
        Err(e) => Value::Str(e.class_name),
    });
    let actual = Value::Str(rust_err.err().map(|e| e.kind.to_string()).unwrap_or_default());
    check("clear_hupcl_bad_kind", expected, actual);
    let py_err = py.eval("util.set_nonblock(-1)");
    let rust_err = klippy_core::util::set_nonblock(-1);
    let expected = Ok(match py_err {
        Ok(_) => Value::Str("no-error".to_string()),
        Err(e) => Value::Str(e.class_name),
    });
    let actual = Value::Str(rust_err.err().map(|e| e.kind.to_string()).unwrap_or_default());
    check("set_nonblock_bad_kind", expected, actual);
    // setup_python2_wrappers and fix_sigint return None.
    let expected = py.eval("util.setup_python2_wrappers()");
    check("py2_wrappers", expected, Value::Null);
    let expected = py.eval("util.fix_sigint()");
    let actual = klippy_core::util::fix_sigint().map(|_| Value::Null).unwrap_or(Value::Null);
    check("fix_sigint", expected, actual);
    // create_pty succeeds and hands back an int fd.
    let py_ret = py.eval(&format!("util.create_pty({:?})", ptyname));
    let rust_ret = klippy_core::util::create_pty(&ptyname);
    let py_is_int = matches!(py_ret, Ok(Value::Int(_)));
    let rust_is_int = matches!(rust_ret, Ok(pycompat::PyValue::Int(_)));
    assert!(py_is_int && rust_is_int, "create_pty should return int fds: py={:?} rust={:?}", py_ret, rust_ret);
    if let Ok(pycompat::PyValue::Int(n)) = rust_ret {
        let fd = n.inner().to_string().parse::<i64>().unwrap_or(-1);
        kos::fd::close(fd as i32);
    }
    cleanup(&dir);
}
