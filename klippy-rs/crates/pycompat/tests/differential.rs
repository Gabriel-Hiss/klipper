//! Differential tests against the CPython oracle.
//!
//! Each test builds a deterministic corpus (fixed seeds plus hand-picked
//! edge cases), runs every case through `pycompat` and through
//! `parity/pycompat-oracle/oracle.py` under the interpreter named by
//! `PYCOMPAT_PYTHON`, and reports every divergence at the end of the test
//! (not just the first). The oracle is required: without `PYCOMPAT_PYTHON`
//! the tests fail with the setup instruction.
//!
//! ```sh
//! PYCOMPAT_PYTHON=python3.8 cargo test -p pycompat --test differential
//! ```
//!
//! `PYCOMPAT_PYTHON` must point to a `python3.8` binary; other versions
//! (notably 3.12+, where `sum` switched to Neumaier summation) produce
//! different results and must not be used.
//!
//! Cases CPython answers with values a Rust `String`/`f64` cannot hold are
//! left out: lone surrogates (`chr(0xd800)`, `json.loads('"\ud800"')`) and
//! complex powers (`(-8.0) ** (1/3)`).

use pycompat::int::{int_cmp, int_divmod, int_floor_div, int_is_odd, int_mod, int_pow};
use pycompat::json::{dumps_opts, loads, DumpsOpts};
use pycompat::text::{
    cmp_bigint_float, float_floor_div, float_from_string, float_mod, float_pow, float_py_hash, float_repr,
    float_str, float_true_div, format_value, int_from_string, percent_format, percent_format_map, py_sum_floats,
    round_py, str_format, str_repr, trunc_float_to_bigint, try_round_py_nd, PctValue, PyError,
};
use pycompat::value::{py_rich_compare, type_name, PyValue};
use pycompat::PyInt;
use serde_json::{json, Value};
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::str::FromStr;

// ---------------------------------------------------------------------------
// Oracle client.
// ---------------------------------------------------------------------------

/// `oracle.py` over its JSON-lines protocol: one request line, one
/// response line, in order.
struct Oracle {
    child: Child,
    writer: Option<ChildStdin>,
    reader: BufReader<ChildStdout>,
}

#[derive(Clone, Debug, PartialEq)]
enum OracleResp {
    Ok(String),
    Err { kind: String, msg: String },
}

/// Starts the oracle; panics when it is unavailable, since a run without
/// it proves nothing.
fn spawn_oracle() -> Oracle {
    let py = std::env::var("PYCOMPAT_PYTHON")
        .unwrap_or_else(|_| panic!("PYCOMPAT_PYTHON is not set; point it at the reference python3.8"));
    // `parity/` lives at the repository root, three levels above this crate.
    let oracle = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../parity/pycompat-oracle/oracle.py");
    assert!(oracle.exists(), "pycompat oracle not found at {}", oracle.display());
    let mut child = Command::new(&py)
        .arg(oracle.as_os_str())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap_or_else(|e| panic!("cannot start {py}: {e}"));
    let writer = child.stdin.take().expect("piped stdin");
    let stdout = child.stdout.take().expect("piped stdout");
    Oracle { child, writer: Some(writer), reader: BufReader::new(stdout) }
}

fn parse_response(line: &str) -> OracleResp {
    let v: Value = serde_json::from_str(line.trim()).unwrap_or_else(|e| panic!("bad oracle line {line:?}: {e}"));
    if v["ok"].as_bool().unwrap_or(false) {
        let r = &v["result"];
        OracleResp::Ok(match r {
            Value::String(s) => s.clone(),
            Value::Bool(b) => b.to_string(),
            Value::Number(n) => n.to_string(),
            other => serde_json::to_string(other).expect("serializable"),
        })
    } else {
        OracleResp::Err {
            kind: v["kind"].as_str().unwrap_or("?").to_string(),
            msg: v["msg"].as_str().unwrap_or("?").to_string(),
        }
    }
}

impl Oracle {
    /// Sends all requests from a writer thread while reading responses, so
    /// large batches do not deadlock on full pipes.
    fn call_many(&mut self, reqs: &[(&str, Value)]) -> Vec<OracleResp> {
        let writer = self.writer.as_mut().expect("oracle stdin open");
        let reader = &mut self.reader;
        std::thread::scope(|s| {
            s.spawn(move || {
                let mut w = BufWriter::new(writer);
                for (op, args) in reqs {
                    serde_json::to_writer(&mut w, &json!({"op": op, "args": args})).expect("write request");
                    w.write_all(b"\n").expect("write request");
                }
                w.flush().expect("flush requests");
            });
            reqs.iter()
                .map(|_| {
                    let mut line = String::new();
                    reader.read_line(&mut line).expect("read oracle response");
                    assert!(!line.is_empty(), "oracle exited early");
                    parse_response(&line)
                })
                .collect()
        })
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        drop(self.writer.take());
        let _ = self.child.wait();
    }
}

// ---------------------------------------------------------------------------
// Divergence collection.
// ---------------------------------------------------------------------------

type Outcome = Result<String, PyError>;

/// Runs `f`, turning a panic into a `RustPanic` error so one bad case
/// cannot hide the others.
fn outcome(f: impl FnOnce() -> Outcome) -> Outcome {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|e| {
        let msg = e
            .downcast_ref::<String>()
            .cloned()
            .or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string()))
            .unwrap_or_else(|| "panic".to_string());
        Err(PyError::new("RustPanic", msg))
    })
}

/// How a successful result is compared.
#[derive(Clone, Copy)]
enum Cmp {
    /// Byte-identical text.
    Text,
    /// Both sides are `f64` bit patterns in hex; every NaN equals every NaN.
    FloatBits,
}

struct Case {
    op: &'static str,
    args: Value,
    ctx: String,
    rust: Outcome,
    cmp: Cmp,
}

fn case(op: &'static str, args: Value, ctx: impl Into<String>, rust: Outcome) -> Case {
    Case { op, args, ctx: ctx.into(), rust, cmp: Cmp::Text }
}

fn float_case(op: &'static str, args: Value, ctx: impl Into<String>, rust: Result<f64, PyError>) -> Case {
    Case { op, args, ctx: ctx.into(), rust: rust.map(bits_hex), cmp: Cmp::FloatBits }
}

struct Report {
    name: &'static str,
    cases: usize,
    /// `(op, description)` of every divergence.
    diffs: Vec<(&'static str, String)>,
}

impl Report {
    fn new(name: &'static str) -> Self {
        Report { name, cases: 0, diffs: Vec::new() }
    }

    fn run(&mut self, oc: &mut Oracle, cases: Vec<Case>) {
        let reqs: Vec<(&str, Value)> = cases.iter().map(|c| (c.op, c.args.clone())).collect();
        let resps = oc.call_many(&reqs);
        for (c, r) in cases.iter().zip(resps) {
            self.compare(c, &r);
        }
    }

    fn compare(&mut self, c: &Case, oracle: &OracleResp) {
        self.cases += 1;
        let same = match (oracle, &c.rust) {
            (OracleResp::Ok(o), Ok(r)) => match c.cmp {
                Cmp::Text => o == r,
                Cmp::FloatBits => {
                    let (a, b) = (f64_from_bits_hex(o), f64_from_bits_hex(r));
                    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
                }
            },
            (OracleResp::Err { kind, msg }, Err(e)) => kind == e.kind && *msg == e.msg,
            _ => false,
        };
        if !same {
            let o = match oracle {
                OracleResp::Ok(o) => format!("Ok({o:?})"),
                OracleResp::Err { kind, msg } => format!("{kind}: {msg:?}"),
            };
            let r = match &c.rust {
                Ok(r) => format!("Ok({r:?})"),
                Err(e) => format!("{}: {:?}", e.kind, e.msg),
            };
            self.diffs.push((c.op, format!("{}\n      cpython: {o}\n      rust:    {r}", c.ctx)));
        }
    }

    /// Panics listing, per oracle op, the divergence count and up to
    /// `PER_OP` examples.
    fn finish(self) {
        assert!(self.cases > 0, "{}: no cases ran", self.name);
        if self.diffs.is_empty() {
            eprintln!("{}: {} cases match CPython", self.name, self.cases);
            return;
        }
        const PER_OP: usize = 40;
        let mut ops: Vec<&'static str> = Vec::new();
        for (op, _) in &self.diffs {
            if !ops.contains(op) {
                ops.push(op);
            }
        }
        let mut text = String::new();
        for op in ops {
            let of_op: Vec<&String> = self.diffs.iter().filter(|(o, _)| *o == op).map(|(_, d)| d).collect();
            text.push_str(&format!("\n[{op}] {} divergences:", of_op.len()));
            for d in of_op.iter().take(PER_OP) {
                text.push_str("\n  ");
                text.push_str(d);
            }
        }
        panic!("{}: {} of {} cases diverge from CPython:{}", self.name, self.diffs.len(), self.cases, text);
    }
}

// ---------------------------------------------------------------------------
// Case generation helpers.
// ---------------------------------------------------------------------------

/// Deterministic PRNG (xorshift64*).
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
    fn pick<'a, T>(&mut self, xs: &'a [T]) -> &'a T {
        &xs[self.below(xs.len())]
    }
}

fn bits_hex(f: f64) -> String {
    format!("{:016x}", f.to_bits())
}

fn f64_from_bits_hex(h: &str) -> f64 {
    f64::from_bits(u64::from_str_radix(h.trim(), 16).unwrap_or_else(|_| panic!("bad float bits {h:?}")))
}

fn big(s: &str) -> PyInt {
    PyInt(num_bigint::BigInt::from_str(s).expect("decimal int"))
}

fn int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

fn pct_payload(v: &PctValue) -> Value {
    match v {
        PctValue::Str(s) => json!({"t": "str", "v": s}),
        PctValue::Bool(b) => json!({"t": "bool", "v": b}),
        PctValue::Int(n) => json!({"t": "int", "v": n.0.to_string()}),
        PctValue::Float(f) => json!({"t": "float", "bits": bits_hex(*f)}),
        PctValue::Bytes(b) => json!({"t": "bytes", "hex": hex(b)}),
        PctValue::None => json!({"t": "none"}),
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{:02x}", x)).collect()
}

fn pyvalue_payload(v: &PyValue) -> Value {
    match v {
        PyValue::None => json!({"t": "none"}),
        PyValue::Bool(b) => json!({"t": "bool", "v": b}),
        PyValue::Int(n) => json!({"t": "int", "v": n.0.to_string()}),
        PyValue::Float(f) => json!({"t": "float", "bits": bits_hex(*f)}),
        PyValue::Str(s) => json!({"t": "str", "v": s}),
        PyValue::Bytes(b) => json!({"t": "bytes", "hex": hex(b)}),
        PyValue::List(xs) => json!({"t": "list", "items": xs.iter().map(pyvalue_payload).collect::<Vec<_>>()}),
        PyValue::Tuple(xs) => json!({"t": "tuple", "items": xs.iter().map(pyvalue_payload).collect::<Vec<_>>()}),
        PyValue::Dict(d) => json!({
            "t": "dict",
            "items": d.iter().map(|(k, v)| json!([pyvalue_payload(k), pyvalue_payload(v)])).collect::<Vec<_>>()
        }),
        PyValue::Host(h) => json!({"t": "host", "v": h}),
    }
}

/// Floats covering signs, zeros, specials, extremes and rounding edges.
fn edge_floats() -> Vec<f64> {
    vec![
        0.0,
        -0.0,
        1.0,
        -1.0,
        0.5,
        1.5,
        2.5,
        -2.5,
        0.1,
        0.2,
        0.3,
        1.0 / 3.0,
        2.0 / 3.0,
        9.5,
        0.125,
        0.0001,
        0.00001,
        1e-7,
        1e15,
        1e16,
        1e17,
        1e22,
        1e23,
        123456.789,
        -987.654321,
        1234567.891,
        9.999999,
        0.999999999,
        999999.5,
        9007199254740992.0,
        9007199254740993.0,
        f64::MAX,
        -f64::MAX,
        f64::MIN_POSITIVE,
        5e-324,
        -5e-324,
        1e308,
        1e-308,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NAN,
    ]
}

/// Random floats: raw bit patterns, normal numbers, subnormals, and short
/// decimals nudged by one ulp.
fn random_floats(rng: &mut Rng, n: usize) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let r = rng.next_u64();
        let f = match i % 5 {
            0 => f64::from_bits(r),
            1 => f64::from_bits((r & 0x800F_FFFF_FFFF_FFFF) | (((r >> 20) % 2046 + 1) << 52)),
            2 => f64::from_bits(r & 0x800F_FFFF_FFFF_FFFF),
            3 => {
                let digits = rng.below(17) as u32 + 1;
                let mant = rng.next_u64() % 10u64.pow(digits.min(17));
                let exp = rng.below(640) as i32 - 330;
                let x: f64 = format!("{}e{}", mant, exp).parse().expect("float text");
                match rng.below(3) {
                    0 => x,
                    1 => f64::from_bits(x.to_bits().wrapping_add(1)),
                    _ => f64::from_bits(x.to_bits().wrapping_sub(1)),
                }
            }
            _ => {
                let k = rng.below(60) as i32 - 30;
                let x = 10f64.powi(k) * (rng.below(2000) as f64 + 0.5) / 1000.0;
                if rng.below(2) == 0 {
                    x
                } else {
                    -x
                }
            }
        };
        out.push(f);
    }
    out
}

/// `PctValue`s of every type, with edge magnitudes.
fn pct_values() -> Vec<PctValue> {
    let mut v = vec![
        int(0),
        int(1),
        int(-1),
        int(7),
        int(-7),
        int(42),
        int(65),
        int(255),
        int(-255),
        int(1234567),
        int(-1234567),
        int(0x10FFFF),
        int(0x110000),
        int(1 << 31),
        int(i64::MIN),
        PctValue::Int(big("18446744073709551616")),
        PctValue::Int(big("1000000000000000000000000000000")),
        PctValue::Int(big("-10000000000000000000000003")),
        PctValue::Int(big(&format!("1{}", "0".repeat(400)))),
        PctValue::Bool(true),
        PctValue::Bool(false),
        PctValue::Str(String::new()),
        PctValue::Str("a".to_string()),
        PctValue::Str("hello world".to_string()),
        PctValue::Str("\u{e9}\u{65e5}\u{672c}".to_string()),
        PctValue::Str("'q\"".to_string()),
        PctValue::Str("\n\t\u{7f}\u{a0}\u{200b}".to_string()),
        PctValue::None,
        PctValue::Bytes(b"ab\x00'".to_vec()),
    ];
    v.extend(edge_floats().into_iter().map(PctValue::Float));
    v
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn float_repr_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("float_repr");
    let mut rng = Rng::new(0xC0FFEE);
    let mut floats = edge_floats();
    for k in -330..=310 {
        let x: f64 = format!("1e{}", k).parse().expect("power of ten");
        for d in [-2i64, -1, 0, 1, 2] {
            floats.push(f64::from_bits((x.to_bits() as i64 + d) as u64));
        }
    }
    for k in 0..64u32 {
        let p = 2f64.powi(53) + k as f64;
        floats.push(p);
        floats.push(2f64.powi(k as i32 * 16 - 512));
    }
    floats.extend(random_floats(&mut rng, 40_000));
    let mut cases = Vec::new();
    for &f in &floats {
        let args = json!({"bits": bits_hex(f)});
        cases.push(case("float_repr", args.clone(), format!("repr({f:e}) bits {}", bits_hex(f)), Ok(float_repr(f))));
        cases.push(case("float_str", args.clone(), format!("str({f:e})"), Ok(float_str(f))));
        cases.push(case(
            "float_py_hash",
            args,
            format!("hash({f:e}) bits {}", bits_hex(f)),
            Ok(float_py_hash(f).to_string()),
        ));
    }
    report.run(&mut oc, cases);
    report.finish();
}

/// A `%` conversion spec from its parts.
fn pct_spec(flags: &str, width: &str, prec: &str, conv: char) -> String {
    format!("%{flags}{width}{prec}{conv}")
}

fn percent_case(fmt: &str, args: Vec<PctValue>) -> Case {
    let payload = json!({"fmt": fmt, "args": args.iter().map(pct_payload).collect::<Vec<_>>()});
    let rust = outcome(|| percent_format(fmt, &args));
    case("percent_format", payload, format!("{fmt:?} % {args:?}"), rust)
}

fn percent_map_case(fmt: &str, map: &[(String, PctValue)]) -> Case {
    let payload = json!({
        "fmt": fmt,
        "map": map.iter().map(|(k, v)| json!([k, pct_payload(v)])).collect::<Vec<_>>()
    });
    let rust = outcome(|| percent_format_map(fmt, map));
    case("percent_format_map", payload, format!("{fmt:?} % {map:?}"), rust)
}

#[test]
fn percent_format_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("percent_format");
    let mut rng = Rng::new(0x5EED_0001);
    let flag_chars = ['-', '+', ' ', '#', '0'];
    let mut flag_sets: Vec<String> = (0..32u32)
        .map(|m| flag_chars.iter().enumerate().filter(|(i, _)| m & (1 << i) != 0).map(|(_, c)| *c).collect())
        .collect();
    flag_sets.extend(["00", "+-", "0-", "#-0", " +", "--", "##"].map(String::from));
    let widths = ["", "0", "1", "5", "12", "*"];
    let precs = ["", ".", ".0", ".1", ".3", ".12", ".*"];
    let convs = ['d', 'i', 'u', 'o', 'x', 'X', 'e', 'E', 'f', 'F', 'g', 'G', 'c', 's', 'r', 'a'];
    let values = pct_values();
    let star_widths = [int(5), int(-8), int(0), PctValue::Bool(true), PctValue::Float(3.0), PctValue::Int(big("1180591620717411303424"))];
    let star_precs = [int(2), int(-3), int(0), PctValue::Int(big("1099511627776")), PctValue::Str("x".to_string())];
    let build = |flags: &str, width: &str, prec: &str, conv: char, v: &PctValue, sw: &PctValue, sp: &PctValue| {
        let fmt = pct_spec(flags, width, prec, conv);
        let mut args = Vec::new();
        if width == "*" {
            args.push(sw.clone());
        }
        if prec == ".*" {
            args.push(sp.clone());
        }
        args.push(v.clone());
        percent_case(&fmt, args)
    };
    let mut cases = Vec::new();
    // Every flag/width/precision combination for representative values.
    let core = [int(0), int(-7), int(255), PctValue::Float(1.5), PctValue::Float(-0.0), PctValue::Float(f64::NAN), PctValue::Float(f64::NEG_INFINITY), PctValue::Str("hi".to_string()), PctValue::Bool(true)];
    for conv in convs {
        for v in &core {
            for flags in &flag_sets {
                for width in widths {
                    for prec in precs {
                        cases.push(build(flags, width, prec, conv, v, &star_widths[0], &star_precs[0]));
                    }
                }
            }
        }
    }
    // Random combinations over every value, star argument and conversion.
    for _ in 0..60_000 {
        let conv = *rng.pick(&convs);
        let v = rng.pick(&values).clone();
        let flags = rng.pick(&flag_sets).clone();
        let width = *rng.pick(&widths);
        let prec = *rng.pick(&precs);
        let sw = rng.pick(&star_widths).clone();
        let sp = rng.pick(&star_precs).clone();
        cases.push(build(&flags, width, prec, conv, &v, &sw, &sp));
    }
    // Every conversion on every value without modifiers.
    for conv in convs.iter().copied().chain("%yk\u{1}\u{e9}".chars()) {
        for v in &values {
            cases.push(percent_case(&format!("%{conv}"), vec![v.clone()]));
        }
    }
    // Parser edge cases and klippy's own formats.
    let fixed: Vec<(&str, Vec<PctValue>)> = vec![
        ("", vec![]),
        ("%", vec![]),
        ("abc%", vec![int(1)]),
        ("%5", vec![int(1)]),
        ("%.", vec![int(1)]),
        ("%-", vec![int(1)]),
        ("%(a)s", vec![int(1)]),
        ("%s %s", vec![int(1)]),
        ("%s", vec![int(1), int(2)]),
        ("x", vec![int(1)]),
        ("%%", vec![]),
        ("%%", vec![int(1)]),
        ("%5%", vec![]),
        ("%5%", vec![int(1)]),
        ("%-5%", vec![int(1)]),
        ("%ld %hi %Lf %lf", vec![int(1), int(2), PctValue::Float(1.5), PctValue::Float(2.5)]),
        ("%l", vec![int(1)]),
        ("%lld", vec![int(1)]),
        ("%*", vec![int(1)]),
        ("%.*", vec![int(1)]),
        ("%*.*f", vec![int(10), int(3), PctValue::Float(2.46802)]),
        ("%-*.*f|", vec![int(-10), int(-3), PctValue::Float(2.46802)]),
        ("%0*d", vec![int(-6), int(42)]),
        ("%099999999999999999999d", vec![int(1)]),
        ("%.99999999999d", vec![int(1)]),
        ("%.2147483645d", vec![int(1)]),
        ("%c", vec![int(-1)]),
        ("%c", vec![int(0x10FFFF)]),
        ("%c", vec![PctValue::Int(big("1180591620717411303424"))]),
        ("%5c|%-5c", vec![PctValue::Str("x".to_string()), int(65)]),
        ("%.3f", vec![PctValue::Float(0.123456)]),
        ("%d", vec![int(42)]),
        ("%s %r", vec![PctValue::Str("a\nb".to_string()), PctValue::Float(1.5)]),
        ("%08x", vec![int(0xDEADBEEF)]),
        ("%-10s|", vec![PctValue::Str("hi".to_string())]),
        ("%+d % d", vec![int(5), PctValue::Float(5.0)]),
        ("%.2f%%", vec![PctValue::Float(12.345)]),
        ("%#x %#o", vec![int(255), int(8)]),
        ("%#X %#o %#x", vec![int(-255), int(-8), int(0)]),
        ("%.0f %.0f %.0f %.0f", vec![PctValue::Float(0.5), PctValue::Float(1.5), PctValue::Float(2.5), PctValue::Float(-0.5)]),
        ("%.1f %.2f %.3f", vec![PctValue::Float(0.05), PctValue::Float(0.125), PctValue::Float(1.0005)]),
        ("%.20f %.30e %.40g", vec![PctValue::Float(0.1), PctValue::Float(0.1), PctValue::Float(0.1)]),
        ("%.300f", vec![PctValue::Float(5e-324)]),
        ("%.400g", vec![PctValue::Float(f64::MAX)]),
        ("%f %e %g", vec![PctValue::Int(big(&format!("1{}", "0".repeat(400)))), int(1), int(1)]),
        ("%f", vec![PctValue::Int(big("9007199254740993"))]),
        ("%x", vec![PctValue::Float(1.5)]),
        ("%d %i %u", vec![PctValue::Float(-2.7), PctValue::Float(1e20), PctValue::Bool(true)]),
        ("%s|%5s|%.1s|%-5.2s|", vec![PctValue::None, PctValue::None, PctValue::Bytes(b"xyz".to_vec()), PctValue::Str("\u{e9}t\u{e9}".to_string())]),
        ("%a %r", vec![PctValue::Str("\u{e9}\u{10000}\u{a0}".to_string()), PctValue::Str("\u{e9}\u{10000}\u{a0}".to_string())]),
        ("Ignoring adc sample %.3f/%.3f in heater %s", vec![PctValue::Float(25.0004999), PctValue::Float(0.5), PctValue::Str("extruder".to_string())]),
        ("Sensor '%s' temperature %s not in range %.3f:%.3f", vec![PctValue::Str("x".to_string()), PctValue::Str("1".to_string()), PctValue::Float(-273.15), PctValue::Float(1e3)]),
        ("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d", vec![int(3), PctValue::Str("PA1".to_string()), int(0), int(1), int(12_000_000)]),
        ("Received %f: %s", vec![PctValue::Float(12.0000005), PctValue::Bytes(b"{\"id\": 1}".to_vec())]),
    ];
    for (fmt, args) in fixed {
        cases.push(percent_case(fmt, args));
    }
    // Mapping form.
    let map: Vec<(String, PctValue)> = vec![
        ("a".to_string(), PctValue::Str("x".to_string())),
        ("b".to_string(), int(7)),
        ("".to_string(), PctValue::Float(2.5)),
        ("a(b)".to_string(), int(-3)),
        ("f".to_string(), PctValue::Float(0.1)),
        ("a".to_string(), PctValue::Str("y'z".to_string())),
    ];
    for fmt in [
        "%(a)s %(b)d",
        "%(a)s %s",
        "%s",
        "%r|%10s",
        "%(a)r %(a)a",
        "%(missing)s",
        "%(a)*d",
        "%(b)*d",
        "%(a",
        "%(a(b))s",
        "%(a)",
        "%()s %()r",
        "%(f).3f %(f)10.2e %(b)#x",
        "%d",
        "%f",
        "%c",
        "%(b)s %(b)s %s",
        "hello",
        "%%",
        "%(b)05d|%(b)-5d|%(b)+d",
        "%(é)s",
    ] {
        cases.push(percent_map_case(fmt, &map));
    }
    cases.push(percent_map_case("%(a)s", &[]));
    cases.push(percent_map_case("%s", &[]));
    report.run(&mut oc, cases);
    report.finish();
}

fn format_case(v: &PctValue, spec: &str) -> Case {
    let payload = json!({"value": pct_payload(v), "spec": spec});
    let rust = outcome(|| format_value(v, spec));
    case("format_value", payload, format!("format({v:?}, {spec:?})"), rust)
}

fn str_format_case(template: &str, args: &[PctValue], kwargs: &[(String, PctValue)]) -> Case {
    let payload = json!({
        "template": template,
        "args": args.iter().map(pct_payload).collect::<Vec<_>>(),
        "kwargs": kwargs.iter().map(|(k, v)| json!([k, pct_payload(v)])).collect::<Vec<_>>()
    });
    let rust = outcome(|| str_format(template, args, kwargs));
    case("str_format", payload, format!("{template:?}.format({args:?}, {kwargs:?})"), rust)
}

#[test]
fn format_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("format");
    let mut rng = Rng::new(0x5EED_0002);
    let aligns = ["", "<", ">", "^", "=", "x<", "*>", "0^", "0=", " =", "\u{e9}<", "{<", "1>"];
    let signs = ["", "+", "-", " "];
    let alts = ["", "#"];
    let zeros = ["", "0"];
    let widths = ["", "0", "1", "7", "15", "\u{663}"];
    let groups = ["", ",", "_", ",_", "_,"];
    let precs = ["", ".0", ".1", ".3", ".10", ".", ".\u{662}"];
    let types = ["", "b", "c", "d", "e", "E", "f", "F", "g", "G", "n", "o", "s", "x", "X", "%", "z", "r", "a"];
    let values = pct_values();
    let mut cases = Vec::new();
    // Every type on every value for plain and padded specs.
    for v in &values {
        for ty in types {
            for prefix in ["", "10", "<10", "^12", "=+12", "012", ",", "_", "#", "+", " ", ".2", "#010.3"] {
                cases.push(format_case(v, &format!("{prefix}{ty}")));
            }
        }
    }
    // Random specs over all components.
    for _ in 0..150_000 {
        let spec = format!(
            "{}{}{}{}{}{}{}{}",
            rng.pick(&aligns),
            rng.pick(&signs),
            rng.pick(&alts),
            rng.pick(&zeros),
            rng.pick(&widths),
            rng.pick(&groups),
            rng.pick(&precs),
            rng.pick(&types)
        );
        let v = rng.pick(&values).clone();
        cases.push(format_case(&v, &spec));
    }
    // Malformed and boundary specs.
    for spec in [
        "..", ".x", "1.2.3", "{", "}", "xx", "<<", ">>10", "10x10", "99999999999999999999", ".99999999999999999999",
        "=10s", "+s", "#s", ",s", "_s", ",c", "_b", "_x", ",x", "_n", ",n", ",d", "_d", "+c", "#c", "=c", ".1c",
        ".1d", ".1x", "0c", "05c", "\u{0}>5", "\u{1f}", "\u{80}", "\u{2028}<9", ".2147483648f", ".2147483647",
    ] {
        for v in &values {
            cases.push(format_case(v, spec));
        }
    }
    // str.format templates.
    let args = vec![
        PctValue::Float(1.0),
        int(42),
        PctValue::Str("abc".to_string()),
        PctValue::Bool(true),
        PctValue::None,
        PctValue::Bytes(b"xy".to_vec()),
        int(-3),
    ];
    let kwargs = vec![
        ("name".to_string(), PctValue::Str("n".to_string())),
        ("x".to_string(), PctValue::Float(2.5)),
        ("w".to_string(), int(10)),
        ("s".to_string(), PctValue::Str(".3".to_string())),
    ];
    for t in [
        "x={:.3f} y={name}", "{} {} {}", "{0} {1} {0}", "{} {1}", "{0} {}", "{2!r:>10}", "{name!s:^9}", "{name!a}",
        "{x:{w}.{w}}", "{x:{w}{s}}", "{0:{1:{2}}}", "{:{}}", "{:{}}{}", "{x:{}}", "{0[1]}", "{2[1]}", "{2[9]}",
        "{2[x]}", "{1[0]}", "{5[1]}", "{5[x]}", "{4[0]}", "{0.real}", "{name.nope}", "{0.}", "{0..x}", "{0[]}",
        "{1.real:>5}", "{1.imag}", "{1.numerator}", "{1.denominator}", "{3.real}", "{3.denominator}", "{0.imag}",
        "{0.numerator}", "{x.real.imag}", "{4.real}", "{2.real}", "{1.real[0]}",
        "{0[}", "{0]}", "{0[1]x}", "{0[1].y}", "{", "}", "a{", "a}", "{{}}", "{{{0}}}", "}}{{", "{0!}", "{0!x}",
        "{0!rr}", "{0!r", "{0!", "{0:", "{0", "{ 0}", "{-1}", "{99}", "{missing}", "{\u{660}}", "{0:{}}", "{:,}",
        "{x:%}", "{w:#x}", "{w:c}", "{4}", "{4:s}", "{4!r:>8}", "{3:d}", "{3}", "{3:}", "{5}", "{5:>5}", "{5!s:>9}",
        "{0:=+10.2f}", "{}\u{e9}{}", "{0:\u{600}>5}", "{2:{{}}}", "{x!r:{w}}", "{name:{0}}", "{0}{name}{1}",
        "{name}{}", "{6:03}", "{6:+}", "{6:x}", "{1:_b}", "{1:,.2f}", "{x:.0%}", "{x:e}", "{x:g}", "{x:n}",
        "{w:{w}}", "{2:.{w}}", "{x:{w},}",
    ] {
        cases.push(str_format_case(t, &args, &kwargs));
    }
    cases.push(str_format_case("{} {}", &args[..1], &[]));
    cases.push(str_format_case("{name}", &[], &[]));
    report.run(&mut oc, cases);
    report.finish();
}

/// Strings exercising `int()`/`float()` parsing: whitespace, signs,
/// underscores, radix prefixes, Unicode digits and spellings of specials.
fn numeric_strings() -> Vec<String> {
    let spaces = ["", " ", "\t", "\n \x0b\x0c\r", "\u{a0}", "\u{3000}", "\u{85}", "\u{2028}", "\x1c", "\x1f"];
    let signs = ["", "+", "-", "--", "+-"];
    let bodies = [
        "0", "00", "1", "7", "010", "0010", "1_000", "1__0", "_1", "1_", "0_0", "0_7", "123456789012345678901234567890",
        "0x1f", "0X1F", "0x_1f", "0x__1", "0x", "0x_", "0o17", "0O17", "0o8", "0b101", "0B11", "0b_1", "0b2", "ff",
        "FF", "z", "Z9", "zz", "1e5", "1E5", "1.5", "1.", ".5", ".", "1e", "1e+", "1e-5", "1e+05", "1_0.5", "1.5_0",
        "1_0e1_0", "1e1_0", "1._5", "1_.5", "1e_5", "inf", "Infinity", "INFINITY", "iNf", "infinit", "infinityx",
        "nan", "NaN", "nAn", "nanx", "-inf", "\u{663}", "\u{661}\u{662}", "\u{ff11}\u{ff12}", "1\u{662}",
        "\u{b2}", "\u{661}.\u{665}", "1 2", "12\x00", "\x003", "1.5e308", "1e400", "-1e400", "4.9e-324",
        "2e-324", "2.4703282292062328e-324", "2.4703282292062327e-324", "0.1e-999999999999", "1e999999999999",
        "0e999999999999", "1.7976931348623157e308", "1.7976931348623159e308", "9007199254740993",
        "123456789012345678901234567890e-10", "0x1p3", "1,5", "\u{e9}", "", "+", "-", "_",
    ];
    let mut out = Vec::new();
    for (i, body) in bodies.iter().enumerate() {
        for (j, sign) in signs.iter().enumerate() {
            let lead = spaces[(i + j) % spaces.len()];
            let trail = spaces[(i * 3 + j * 7) % spaces.len()];
            out.push(format!("{sign}{body}"));
            out.push(format!("{lead}{sign}{body}{trail}"));
        }
        for sp in spaces {
            out.push(format!("{sp}{body}{sp}"));
        }
    }
    out.push("9".repeat(300));
    out.push(format!("x{}", "9".repeat(300)));
    out.push(format!("1{}", "_0".repeat(60)));
    out.push(" ".repeat(5));
    out.push("\u{3000}\u{a0}".to_string());
    out
}

#[test]
fn round_and_convert_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("round_and_convert");
    let mut rng = Rng::new(0xBADA55);
    let mut floats = edge_floats();
    floats.extend([0.5, 1.5, 2.5, 3.5, -0.5, -1.5, -2.5, 4.5e15, 2.675, 1.005, 0.285, 5e-5, 1.25, 1.35, 12345.5]);
    floats.extend(random_floats(&mut rng, 3_000));
    let mut cases = Vec::new();
    for &f in &floats {
        let args = json!({"bits": bits_hex(f)});
        cases.push(case("round_py", args.clone(), format!("round({f:e})"), outcome(|| round_py(f).map(|n| n.0.to_string()))));
        for nd in [-400i64, -309, -308, -307, -300, -20, -3, -1, 0, 1, 2, 3, 6, 9, 15, 17, 22, 50, 300, 323, 324, 400] {
            cases.push(float_case(
                "round_py_nd",
                json!({"bits": bits_hex(f), "ndigits": nd}),
                format!("round({f:e}, {nd}) bits {}", bits_hex(f)),
                try_round_py_nd(f, nd),
            ));
        }
        if f.is_finite() {
            cases.push(case("trunc_float_to_bigint", args, format!("int({f:e})"), Ok(trunc_float_to_bigint(f).0.to_string())));
        }
    }
    for s in numeric_strings() {
        cases.push(float_case("float_from_string", json!({"s": s}), format!("float({s:?})"), float_from_string(&s)));
        for base in [0i64, 2, 8, 10, 16, 36, 1, 37, -1] {
            cases.push(case(
                "int_from_string",
                json!({"s": s, "base": base}),
                format!("int({s:?}, {base})"),
                outcome(|| int_from_string(&s, base).map(|n| n.0.to_string())),
            ));
        }
    }
    report.run(&mut oc, cases);
    report.finish();
}

#[test]
fn int_float_arith_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("int_float_arith");
    let ints = [
        "-1000000000000000000000000000000", "-12345678901234567890", "-9223372036854775808", "-7", "-3", "-2", "-1",
        "0", "1", "2", "3", "7", "10", "9223372036854775807", "18446744073709551616", "10000000000000000000000007",
    ];
    let mut cases = Vec::new();
    for a in ints {
        let pa = big(a);
        cases.push(case("int_is_odd", json!({"a": a}), format!("{a} % 2"), Ok(int_is_odd(&pa).to_string())));
        cases.push(case("int_py_hash", json!({"a": a}), format!("hash({a})"), Ok(pa.py_hash().to_string())));
        for e in [0u64, 1, 2, 3, 5, 63, 64, 100] {
            cases.push(case("int_pow", json!({"a": a, "e": e}), format!("{a} ** {e}"), outcome(|| Ok(int_pow(&pa, e).0.to_string()))));
        }
        for b in ints {
            let pb = big(b);
            let ab = json!({"a": a, "b": b});
            cases.push(case("int_cmp", ab.clone(), format!("cmp({a}, {b})"), Ok((int_cmp(&pa, &pb) as i8).to_string())));
            // The int division helpers document that callers reject zero.
            if b == "0" {
                continue;
            }
            cases.push(case("int_floor_div", ab.clone(), format!("{a} // {b}"), outcome(|| Ok(int_floor_div(&pa, &pb).0.to_string()))));
            cases.push(case("int_mod", ab.clone(), format!("{a} % {b}"), outcome(|| Ok(int_mod(&pa, &pb).0.to_string()))));
            cases.push(case(
                "int_divmod",
                ab,
                format!("divmod({a}, {b})"),
                outcome(|| {
                    let (q, r) = int_divmod(&pa, &pb);
                    Ok(serde_json::to_string(&json!([q.0.to_string(), r.0.to_string()])).expect("json"))
                }),
            ));
        }
    }
    let mut rng = Rng::new(0xF10A7);
    let mut floats = vec![
        0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 3.0, 7.5, -7.5, 0.1, -0.1, 10.0, 1e16, 2f64.powi(53), 1e308,
        -1e308, 1e-308, 5e-324, 1e-10, 1e300, 1.0000000000000002, 0.9999999999999999, 1.5, -1.5, 1074.5, -1074.5,
        -1075.0, 1024.0, 2.5e-5, f64::INFINITY, f64::NEG_INFINITY, f64::NAN,
    ];
    floats.extend(random_floats(&mut rng, 25).into_iter().filter(|f| f.is_finite() && f.abs() < 1e30 && f.abs() > 1e-30));
    for &a in &floats {
        for &b in &floats {
            let ab = json!({"a": bits_hex(a), "b": bits_hex(b)});
            let ctx = |op: &str| format!("{a:e} {op} {b:e} (bits {} {})", bits_hex(a), bits_hex(b));
            cases.push(float_case("float_floor_div", ab.clone(), ctx("//"), float_floor_div(a, b)));
            cases.push(float_case("float_mod", ab.clone(), ctx("%"), float_mod(a, b)));
            cases.push(float_case("float_true_div", ab.clone(), ctx("/"), float_true_div(a, b)));
            // Python answers a complex number for a negative finite base
            // with a finite non-integer exponent.
            let complex = a < 0.0 && a.is_finite() && b.is_finite() && b != b.floor();
            if !complex {
                cases.push(float_case("float_pow", ab, ctx("**"), float_pow(a, b)));
            }
        }
    }
    for a in ints {
        for &b in floats.iter().filter(|f| !f.is_nan()) {
            cases.push(case(
                "cmp_bigint_float",
                json!({"a": a, "bits": bits_hex(b)}),
                format!("cmp({a}, {b:e})"),
                Ok((cmp_bigint_float(&big(a).0, b) as i8).to_string()),
            ));
        }
    }
    let lists: Vec<Vec<f64>> = vec![
        vec![],
        vec![1e16, 1.0, -1e16, 1.0],
        vec![0.1; 10],
        vec![-0.0, -0.0],
        vec![1e308, 1e308, -1e308],
        vec![f64::INFINITY, f64::NEG_INFINITY],
        floats.clone(),
    ];
    for xs in lists {
        cases.push(float_case(
            "py_sum_floats",
            json!({"bits_list": xs.iter().map(|f| bits_hex(*f)).collect::<Vec<_>>()}),
            format!("sum({xs:?})"),
            Ok(py_sum_floats(&xs)),
        ));
    }
    report.run(&mut oc, cases);
    report.finish();
}

/// `PyValue`s for repr/compare/JSON sweeps, nested containers included.
fn py_values() -> Vec<PyValue> {
    let i = |v: i64| PyValue::Int(PyInt::from_i64(v));
    let s = |v: &str| PyValue::Str(v.to_string());
    let dict = |items: Vec<(PyValue, PyValue)>| PyValue::Dict(items.into_iter().collect());
    vec![
        PyValue::None,
        PyValue::Bool(true),
        PyValue::Bool(false),
        i(0),
        i(1),
        i(-1),
        PyValue::Int(big("18446744073709551616")),
        PyValue::Float(1.0),
        PyValue::Float(0.5),
        PyValue::Float(-0.0),
        PyValue::Float(1e16),
        PyValue::Float(f64::NAN),
        PyValue::Float(f64::INFINITY),
        PyValue::Float(f64::NEG_INFINITY),
        s(""),
        s("a"),
        s("b"),
        s("a'b\"c\n"),
        s("it's"),
        s("\u{7f}\u{80}\u{a0}\u{ad}\u{e9}\u{2028}\u{200b}\u{feff}\u{1f600}\u{e000}\u{10ffff}"),
        PyValue::Bytes(vec![0, 1, 255, b'a', b'\'', b'"']),
        PyValue::Bytes(b"it's".to_vec()),
        PyValue::List(vec![]),
        PyValue::List(vec![i(1), s("x")]),
        PyValue::List(vec![i(1), i(2)]),
        PyValue::Tuple(vec![]),
        PyValue::Tuple(vec![i(1)]),
        PyValue::Tuple(vec![i(1), PyValue::Float(2.0)]),
        dict(vec![]),
        dict(vec![(s("a"), PyValue::Float(1.5)), (s("b"), PyValue::List(vec![i(1), PyValue::None]))]),
        dict(vec![(i(1), s("one")), (PyValue::Float(2.5), PyValue::Bool(false)), (PyValue::None, i(0))]),
    ]
}

/// Strings over the whole code space (surrogates excluded), for the
/// printability table behind `repr`.
fn unicode_strings() -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut cp = 0u32;
    while cp <= 0x10FFFF {
        if let Some(c) = char::from_u32(cp) {
            cur.push(c);
            if cur.chars().count() == 24 {
                out.push(std::mem::take(&mut cur));
            }
        }
        cp += if cp < 0x3000 { 1 } else { 13 };
    }
    out.push(cur);
    out
}

/// Turns a successfully parsed value into the oracle's payload text.
fn loads_outcome(doc: &str) -> Outcome {
    outcome(|| loads(doc).map(|v| serde_json::to_string(&pyvalue_payload(&v)).expect("json")))
}

#[test]
fn value_and_json_differential() {
    let mut oc = spawn_oracle();
    let mut report = Report::new("value_and_json");
    let values = py_values();
    let mut cases = Vec::new();
    let mut reprs: Vec<PyValue> = values.clone();
    reprs.extend(unicode_strings().into_iter().map(PyValue::Str));
    for v in &reprs {
        let payload = json!({"value": pyvalue_payload(v)});
        cases.push(case("py_repr", payload.clone(), format!("repr({v:?})"), Ok(v.repr())));
        cases.push(case("py_str", payload.clone(), format!("str({v:?})"), Ok(v.str())));
        cases.push(case("type_name", payload.clone(), format!("type({v:?})"), Ok(type_name(v).to_string())));
        cases.push(case("py_truthy", payload, format!("bool({v:?})"), Ok(v.truthy().to_string())));
    }
    for s in unicode_strings() {
        cases.push(case("str_repr", json!({"s": s}), format!("repr({s:?})"), Ok(str_repr(&s))));
    }
    for v in values.iter().filter(|v| matches!(v, PyValue::Bool(_) | PyValue::Int(_) | PyValue::Float(_))) {
        cases.push(case("py_hash", json!({"value": pyvalue_payload(v)}), format!("hash({v:?})"), Ok(v.py_hash().to_string())));
    }
    for a in &values {
        for b in &values {
            for op in ["==", "!=", "<", "<=", ">", ">="] {
                cases.push(case(
                    "py_rich_compare",
                    json!({"op": op, "a": pyvalue_payload(a), "b": pyvalue_payload(b)}),
                    format!("{a:?} {op} {b:?}"),
                    outcome(|| {
                        py_rich_compare(op, a, b).map(|r| r.to_string()).map_err(|m| PyError::new("TypeError", m))
                    }),
                ));
            }
        }
    }
    // json.dumps over values and options.
    let mut docs: Vec<PyValue> = values.clone();
    docs.push(PyValue::List(values.iter().filter(|v| !matches!(v, PyValue::Bytes(_))).cloned().collect()));
    docs.push(PyValue::Dict(
        [
            (PyValue::Float(f64::NAN), PyValue::Int(PyInt::from_i64(1))),
            (PyValue::Float(-0.0), PyValue::Int(PyInt::from_i64(2))),
            (PyValue::Bool(false), PyValue::Str("\u{0}\u{1f}\u{7f}\u{80}\u{ffff}\u{10000}/\\\"".to_string())),
        ]
        .into_iter()
        .collect(),
    ));
    docs.push(PyValue::Dict([(PyValue::Tuple(vec![]), PyValue::None)].into_iter().collect()));
    docs.push(PyValue::Dict([(PyValue::Float(f64::INFINITY), PyValue::None)].into_iter().collect()));
    let opt_sets = [
        (", ", ": ", true, true),
        (",", ":", true, true),
        (", ", ": ", false, true),
        (",", ":", true, false),
        (";", "=", false, false),
    ];
    for v in &docs {
        for (item_sep, key_sep, ensure_ascii, allow_nan) in opt_sets {
            let opts = DumpsOpts { item_sep: item_sep.to_string(), key_sep: key_sep.to_string(), ensure_ascii, allow_nan };
            cases.push(case(
                "json_dumps",
                json!({
                    "value": pyvalue_payload(v),
                    "opts": {"separators": [item_sep, key_sep], "ensure_ascii": ensure_ascii, "allow_nan": allow_nan}
                }),
                format!("json.dumps({v:?}, {opts:?})"),
                outcome(|| dumps_opts(v, &opts)),
            ));
        }
    }
    // json.loads: valid documents (round trips of the dumps above), fixed
    // malformed documents, and every prefix of a few rich documents.
    let mut load_docs: Vec<String> = vec![
        "{\"a\": 1.5}", "", "  ", "012", "[012]", "-01", "01.5", "1.", "1e", "1e+", "1e-", "1.5e3", "1.5e", "1E400",
        "-", "-x", "+1", "-0", "-0.0", "[1,]", "[1 2]", "{\"a\" 1}", "{\"a\": 1, }", "{,}", "{\"a\":1,\"a\":2}", "nul",
        "tru", "fals", "Na", "Infinit", "-Infinit", "\"unterminated", "\"\\x\"", "\"\\u12G4\"", "\"\\u12\"",
        "\"\\u1234", "\"\\u1234\"", "\"\\", "\"\\\"", "\"\u{e9}\" x", "[\"\u{fc}\",", "{\"\u{e9}\": 1, }",
        "[1]\n\n x", "NaN", "-Infinity", "[Infinity, 1]", "\"\\ud83d\\ude00\"", "\"\\uD83D\\uDE00x\"",
        "\"\\ud83d\\u12G4\"", "\"a\u{1}b\"", "\"a\tb\"", "\"\\/\\b\\f\\n\\r\\t\\\"\\\\\"", "\u{feff}1", "1\u{feff}",
        " \t\r\n[ \t\r\n1 \t\r\n] \t\r\n", "\u{a0}1", "[[[[]]]]", "{\"a\":{\"b\":[{}]}}", "123456789012345678901234567890",
        "-123456789012345678901234567890.5e-3", "1.7976931348623159e308", "4.9e-324", "[1e999999999999999]",
        "\"\u{10000}\u{e9}\"", "[true,false,null]", "[true false]", "{\"a\":}", "{\"a\"", "{\"a\":1", "[", "]", "}",
        "\"\\u0000\"", "0x10", ".5", "[.5]", "[-.5]", "1_000",
    ]
    .into_iter()
    .map(String::from)
    .collect();
    for v in &docs {
        for (item_sep, key_sep) in [(", ", ": "), (",", ":")] {
            let opts = DumpsOpts { item_sep: item_sep.to_string(), key_sep: key_sep.to_string(), ensure_ascii: false, allow_nan: true };
            if let Ok(text) = dumps_opts(v, &opts) {
                load_docs.push(text);
            }
        }
    }
    for rich in ["{\"k\": [1, -2.5e-3, \"s\\u00e9\\n\", true, null, {\"x\": NaN}], \"\u{e9}\": -Infinity}", "[\"a\\\"b\", 0, -0.0, 1E+2]"] {
        let chars: Vec<char> = rich.chars().collect();
        for n in 0..chars.len() {
            load_docs.push(chars[..n].iter().collect());
            let mut mutated: String = chars[..n].iter().collect();
            mutated.push(['x', ',', '"', '\\', ']', '}', ' ', '1'][n % 8]);
            mutated.extend(&chars[n..]);
            load_docs.push(mutated);
        }
    }
    for doc in load_docs {
        cases.push(case("json_loads", json!({"s": doc}), format!("json.loads({doc:?})"), loads_outcome(&doc)));
    }
    report.run(&mut oc, cases);
    report.finish();
}
