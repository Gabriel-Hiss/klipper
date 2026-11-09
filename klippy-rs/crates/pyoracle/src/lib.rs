//! Test-time client for the live reference oracle (`parity/pyoracle/oracle.py`).
//!
//! The oracle is the frozen CPython 3.8.2 + `klippy/` the port must match. Tests use it to
//! compute expected values for generated inputs instead of transcribing them by hand:
//!
//! ```ignore
//! let Some(mut py) = pyoracle::Oracle::start() else { return };   // skipped outside the oracle
//! py.exec("import msgproto").unwrap();
//! let expected = py.eval("msgproto.crc16_ccitt(b'abc')");          // Ok(Value) or Err(Exception)
//! recording::assert_equal(&record, &expected, ...);
//! ```
//!
//! Values come back in the recording format (`parity/record/FORMAT.md`), so they compare with
//! `recording::assert_equal` exactly like replayed recordings. Exceptions carry the CPython
//! exception type name and `str(exc)`.
//!
//! The interpreter is `$KLIPPY_PYTHON` (set inside the x86 and MIPS oracle images). A test that
//! needs the oracle fails when it is unset: a silently skipped comparison proves nothing, and a
//! passing run must mean the reference was consulted. Run these tests in the oracle images.

use recording::{Exception, Value};
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

/// A running reference interpreter with one shared namespace.
pub struct Oracle {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}

/// Result of an oracle request: the encoded value, or the exception Python raised.
pub type OracleResult = Result<Value, Exception>;

fn script_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../parity/pyoracle/oracle.py")
}

fn json_string(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

impl Oracle {
    /// Starts the oracle. Panics when `$KLIPPY_PYTHON` is not set (see the module docs); the
    /// `Option` return stays for the callers' `let Some(..) else` shape and is always `Some`.
    pub fn start() -> Option<Oracle> {
        let python = std::env::var("KLIPPY_PYTHON")
            .unwrap_or_else(|_| panic!("KLIPPY_PYTHON is not set: oracle tests run only in the oracle images"));
        let mut child = Command::new(python)
            .arg(script_path())
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .expect("start the python oracle");
        let stdin = child.stdin.take().expect("oracle stdin");
        let stdout = BufReader::new(child.stdout.take().expect("oracle stdout"));
        Some(Oracle { child, stdin, stdout })
    }

    fn request(&mut self, line: String) -> OracleResult {
        writeln!(self.stdin, "{line}").expect("write to oracle");
        self.stdin.flush().expect("flush oracle");
        let mut answer = String::new();
        self.stdout.read_line(&mut answer).expect("read from oracle");
        let record = recording::parse_line(answer.trim()).expect("oracle answer");
        match record.exc {
            Some(exc) => Err(exc),
            None => Ok(record.ret.unwrap_or(Value::Null)),
        }
    }

    /// Runs statements in the shared namespace (imports, setup, object construction).
    pub fn exec(&mut self, code: &str) -> OracleResult {
        self.request(format!("{{\"op\":\"exec\",\"code\":{}}}", json_string(code)))
    }

    /// Evaluates one expression in the shared namespace.
    pub fn eval(&mut self, expr: &str) -> OracleResult {
        self.request(format!("{{\"op\":\"eval\",\"expr\":{}}}", json_string(expr)))
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}
