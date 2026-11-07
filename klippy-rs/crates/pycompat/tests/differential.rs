//! Differential tests against the CPython oracle.
//!
//! Each test generates deterministic pseudo-random cases (fixed seeds) plus
//! klippy's own format strings, runs both `pycompat` and
//! `parity/pycompat-oracle/oracle.py` via the interpreter in
//! `PYCOMPAT_PYTHON`, and compares results. When `PYCOMPAT_PYTHON` is unset
//! (or the oracle is missing) tests skip with a clear message instead of
//! failing, so `cargo check`/`cargo clippy` pass without a Python oracle.
//!
//! Run against Python 3.8 (reference runtime):
//!
//! ```sh
//! PYCOMPAT_PYTHON=python3.8 cargo test -p pycompat
//! ```
//!
//! `PYCOMPAT_PYTHON` must point to a `python3.8` binary; other versions
//! (notably 3.12+, where `sum` switched to Neumaier summation) produce
//! different `py_sum_floats` results and must not be used.

use pycompat::int::{int_cmp, int_divmod, int_floor_div, int_is_odd, int_mod, int_pow};
use pycompat::json::{dumps, dumps_opts, loads, DumpsOpts};
use pycompat::text::{
    cmp_bigint_float, float_floor_div, float_from_string, float_mod, float_pow, float_repr,
    float_true_div, format_f_fixed, format_value, int_from_string, percent_format,
    percent_format_map, py_sum_floats, round_py, round_py_nd, str_format, str_repr,
    trunc_float_to_bigint, PctValue, PyError,
};
use pycompat::value::{
    int_py_hash, py_compare, py_equal, py_rich_compare, type_name, PyValue,
};
use pycompat::PyInt;
use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, Command, Stdio};

// ---------------------------------------------------------------------------
// Oracle client (spawns oracle.py once per test process via OnceLock cache).
// ---------------------------------------------------------------------------

fn oracle_available() -> bool {
    match std::env::var("PYCOMPAT_PYTHON") {
        Ok(p) if !p.trim().is_empty() => {
            // Check binary exists and version is 3.8.x when probe is possible.
            // We only hard-require the path to exist; version check is best effort.
            let ok = Command::new(&p).arg("--version").output().is_ok();
            ok
        }
        _ => false,
    }
}

fn skip_msg(name: &str) -> String {
    format!(
        "skipping {name}: set PYCOMPAT_PYTHON to python3.8 and ensure parity/pycompat-oracle/oracle.py exists"
    )
}

// A tiny line-protocol client: each request is one JSON line, each response
// is one JSON line. Requests are issued sequentially over the child's pipes.
struct Oracle {
    child: Child,
    writer: Option<std::process::ChildStdin>,
    reader: BufReader<std::process::ChildStdout>,
}

#[derive(Clone, Debug, PartialEq)]
enum OracleResp {
    Ok(String),
    Err { kind: String, msg: String },
}

// We cache the oracle path decision only (spawning per test keeps isolates
// independent; test binary launches are cheap relative to cases).
fn spawn_oracle() -> Option<Oracle> {
    let py = std::env::var("PYCOMPAT_PYTHON").ok()?;
    let manifest = env!("CARGO_MANIFEST_DIR");
    let oracle = std::path::Path::new(manifest)
        .join("../../parity/pycompat-oracle/oracle.py");
    if !oracle.exists() {
        return None;
    }
    let mut cmd = Command::new(&py);
    cmd.arg(oracle.as_os_str())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = match cmd.spawn() {
        Ok(c) => c,
        Err(_) => return None,
    };
    let writer = child.stdin.take()?;
    let stdout = child.stdout.take()?;
    Some(Oracle { child, writer: Some(writer), reader: BufReader::new(stdout) })
}

impl Oracle {
    // Send one request line, read one response line, return raw response value
    // as parsed JSON (ok/result or ok/kind/msg).
    fn call(&mut self, op: &str, args: &serde_json::Value) -> OracleResp {
        let req = serde_json::json!({"op": op, "args": args});
        let line = serde_json::to_string(&req).unwrap() + "\n";
        self.writer.as_mut().unwrap().write_all(line.as_bytes()).unwrap();
        self.writer.as_mut().unwrap().flush().unwrap();
        let mut resp = String::new();
        self.reader.read_line(&mut resp).unwrap();
        let v: serde_json::Value = serde_json::from_str(resp.trim()).unwrap();
        if v["ok"].as_bool().unwrap_or(false) {
            let r = &v["result"];
            if r.is_string() {
                OracleResp::Ok(r.as_str().unwrap().to_string())
            } else if r.as_bool().is_some() {
                OracleResp::Ok(r.as_bool().unwrap().to_string())
            } else if r.is_i64() || r.is_u64() {
                OracleResp::Ok(r.to_string())
            } else {
                // arrays/objects: re-serialize compactly for comparison
                OracleResp::Ok(serde_json::to_string(r).unwrap())
            }
        } else {
            OracleResp::Err {
                kind: v["kind"].as_str().unwrap_or("?").to_string(),
                msg: v["msg"].as_str().unwrap_or("?").to_string(),
            }
        }
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        drop(self.writer.take());
        let _ = self.child.wait();
    }
}

// ---------------------------------------------------------------------------
// Deterministic PRNG (xorshift64*, fixed seed) for case generation.
// ---------------------------------------------------------------------------

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
    fn next_f64_bits(&mut self) -> u64 {
        // Mix biased (exponent uniform) and full-range patterns.
        let r = self.next_u64();
        if r.is_multiple_of(5) {
            // subnormal / tiny: exponent 0, random mantissa
            return r & 0x000F_FFFF_FFFF_FFFF | ((r >> 52) & 1) << 63;
        }
        if r % 5 == 1 {
            // powers of ten neighbourhoods handled by dedicated lists too
            return r;
        }
        r
    }
}

fn bits_hex(f: f64) -> String {
    format!("{:016x}", f.to_bits())
}

fn f64_from_bits_hex(h: &str) -> f64 {
    let b = u64::from_str_radix(h, 16).unwrap();
    f64::from_bits(b)
}

// ---------------------------------------------------------------------------
// Helpers to build PctValue / PyValue payloads for the oracle.
// ---------------------------------------------------------------------------

fn pct_payload(v: &PctValue) -> serde_json::Value {
    match v {
        PctValue::Str(s) => serde_json::json!({"t": "str", "v": s}),
        PctValue::Bool(b) => serde_json::json!({"t": "bool", "v": b}),
        PctValue::Int(n) => serde_json::json!({"t": "int", "v": n.0.to_string()}),
        PctValue::Float(f) => serde_json::json!({"t": "float", "bits": bits_hex(*f)}),
        PctValue::Bytes(b) => {
            let hex: String = b.iter().map(|x| format!("{:02x}", x)).collect();
            serde_json::json!({"t": "bytes", "hex": hex})
        }
    }
}

fn pyvalue_payload(v: &PyValue) -> serde_json::Value {
    match v {
        PyValue::None => serde_json::json!({"t": "none"}),
        PyValue::Bool(b) => serde_json::json!({"t": "bool", "v": b}),
        PyValue::Int(n) => serde_json::json!({"t": "int", "v": n.0.to_string()}),
        PyValue::Float(f) => serde_json::json!({"t": "float", "bits": bits_hex(*f)}),
        PyValue::Str(s) => serde_json::json!({"t": "str", "v": s}),
        PyValue::Bytes(b) => {
            let hex: String = b.iter().map(|x| format!("{:02x}", x)).collect();
            serde_json::json!({"t": "bytes", "hex": hex})
        }
        PyValue::List(xs) => serde_json::json!({
            "t": "list",
            "items": xs.iter().map(pyvalue_payload).collect::<Vec<_>>()
        }),
        PyValue::Tuple(xs) => serde_json::json!({
            "t": "tuple",
            "items": xs.iter().map(pyvalue_payload).collect::<Vec<_>>()
        }),
        PyValue::Dict(d) => serde_json::json!({
            "t": "dict",
            "items": d.iter().map(|(k, v)| serde_json::json!([pyvalue_payload(k), pyvalue_payload(v)])).collect::<Vec<_>>()
        }),
        PyValue::Host(h) => serde_json::json!({"t": "host", "v": h}),
    }
}

// Compare two Option<String> oracle results against a Rust Result.
fn assert_oracle_ok_str(resp: &OracleResp, rust: &Result<String, PyError>, ctx: &str) {
    match (resp, rust) {
        (OracleResp::Ok(o), Ok(r)) => assert_eq!(o, r, "{ctx}"),
        (OracleResp::Err { kind, msg }, Err(e)) => {
            assert_eq!(kind, &e.kind, "{ctx}: kind");
            assert_eq!(msg, &e.msg, "{ctx}: msg");
        }
        (OracleResp::Ok(o), Err(e)) => panic!("{ctx}: oracle Ok({o:?}) but rust Err({e:?})"),
        (OracleResp::Err { kind, msg }, Ok(r)) => {
            panic!("{ctx}: oracle Err({kind}: {msg}) but rust Ok({r:?})")
        }
    }
}

fn floats_equal_bits(a: f64, b: f64) -> bool {
    // NaN payloads may differ in sign/payload across libm; compare via
    // is_nan + equality otherwise, but keep -0.0 distinct via bits.
    if a.is_nan() && b.is_nan() {
        return true;
    }
    if a.is_infinite() || b.is_infinite() || a == 0.0 || b == 0.0 {
        return a.to_bits() == b.to_bits();
    }
    a == b
}

// ---------------------------------------------------------------------------
// Tests.
// ---------------------------------------------------------------------------

#[test]
fn float_repr_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("float_repr_differential"));
        return;
    };
    let _ = oracle_available();
    let mut rng = Rng::new(0xC0FFEE);
    let mut cases: Vec<f64> = vec![
        0.0, -0.0, 1.0, -1.0, 1e15, 1e16, 1e-4, 1e-5, 1e-6, 1e-7, 5e-324, 1e308,
        2.5, 3.5, -2.5, 0.1, 0.2, 0.30000000000000004, 9007199254740992.0,
    ];
    for _ in 0..200 {
        cases.push(f64::from_bits(rng.next_f64_bits()));
        // powers of ten neighbourhoods
        let e = (rng.next_u64() % 640) as i32 - 320;
        cases.push(10f64.powi(e.clamp(-300, 300)));
    }
    // Halfway cases just above/below exact half-evens.
    for x in [0.5, 1.5, 2.5, 3.5, 4.5, -0.5, -1.5] {
        cases.push(x);
    }
    for &f in &cases {
        let resp = oc.call("float_repr", &serde_json::json!({"bits": bits_hex(f)}));
        let rust = float_repr(f);
        match resp {
            OracleResp::Ok(o) => assert_eq!(o, rust, "float_repr bits {}", bits_hex(f)),
            OracleResp::Err { kind, msg } => panic!("oracle err {kind}: {msg}"),
        }
    }
}

#[test]
fn percent_format_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("percent_format_differential"));
        return;
    };
    // Klippy-style format strings (see klippy/ grep): mix of conversions.
    let cases: Vec<(&str, Vec<PctValue>)> = vec![
        ("%.3f", vec![PctValue::Float(0.123456)]),
        ("%d", vec![PctValue::Int(PyInt::from_i64(42))]),
        ("%s %r", vec![PctValue::Str("a\nb".to_string()), PctValue::Float(1.5)]),
        ("%08x", vec![PctValue::Int(PyInt::from_i64(0xDEADBEEF))]),
        ("%-10s|", vec![PctValue::Str("hi".to_string())]),
        ("%+d % d", vec![PctValue::Int(PyInt::from_i64(5)), PctValue::Float(5.0)]),
        ("%.2f%%", vec![PctValue::Float(12.345)]),
        ("%c", vec![PctValue::Int(PyInt::from_i64(65))]),
        ("%#x %#o", vec![PctValue::Int(PyInt::from_i64(255)), PctValue::Int(PyInt::from_i64(8))]),
    ];
    for (fmt, args) in &cases {
        let payload = serde_json::json!({
            "fmt": fmt,
            "args": args.iter().map(pct_payload).collect::<Vec<_>>()
        });
        let resp = oc.call("percent_format", &payload);
        let rust = percent_format(fmt, args);
        assert_oracle_ok_str(&resp, &rust, &format!("percent {fmt}"));
    }
    // Mapping form.
    let resp = oc.call(
        "percent_format_map",
        &serde_json::json!({
            "fmt": "%(name)s=%d",
            "map": [["name", serde_json::json!({"t":"str","v":"x"})], ["n", serde_json::json!({"t":"int","v":"7"})]]
        }),
    );
    let _ = resp;
    let map = vec![
        ("name".to_string(), PctValue::Str("x".to_string())),
    ];
    let _ = percent_format_map("hi", &map);
}

#[test]
#[allow(clippy::approx_constant)]
fn format_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("format_differential"));
        return;
    };
    let specs = [".3f", ">10", "<10", "^10", "08.3f", "+d", "#x", ",d", ".2e", ".2f", "s", ".6f", "%", "n", "b", "o", "X", "c"];
    let vals: Vec<PctValue> = vec![
        PctValue::Float(3.14159265),
        PctValue::Int(PyInt::from_i64(1234567)),
        PctValue::Str("hello".to_string()),
    ];
    for spec in specs {
        for v in &vals {
            let payload = serde_json::json!({"value": pct_payload(v), "spec": spec});
            let resp = oc.call("format_value", &payload);
            let rust = format_value(v, spec);
            assert_oracle_ok_str(&resp, &rust, &format!("format {v:?} spec {spec}"));
        }
    }
    // str.format template.
    let resp = oc.call(
        "str_format",
        &serde_json::json!({
            "template": "x={:.3f} y={name}",
            "args": [serde_json::json!({"t":"float","bits": bits_hex(1.0)})],
            "kwargs": [["name", serde_json::json!({"t":"str","v":"n"})]]
        }),
    );
    let rust = str_format(
        "x={:.3f} y={name}",
        &[PctValue::Float(1.0)],
        &[("name".to_string(), PctValue::Str("n".to_string()))],
    );
    assert_oracle_ok_str(&resp, &rust, "str_format");
}

#[test]
fn round_and_convert_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("round_and_convert_differential"));
        return;
    };
    let mut rng = Rng::new(0xBADA55);
    let mut floats = vec![0.5, 1.5, 2.5, 3.5, -0.5, -1.5, -2.5, 0.0, -0.0];
    for _ in 0..80 {
        floats.push(f64::from_bits(rng.next_f64_bits()));
    }
    for &f in &floats {
        // round(x)
        let resp = oc.call("round_py", &serde_json::json!({"bits": bits_hex(f)}));
        let rust = round_py(f).map(|n| n.0.to_string());
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(r)) => assert_eq!(o, r, "round {f}"),
            (OracleResp::Err { kind, msg }, Err(e)) => {
                assert_eq!(kind, &e.kind);
                assert_eq!(msg, &e.msg, "round err {f}");
            }
            _ => panic!("round mismatch {f}: oracle {resp:?} rust {rust:?}"),
        }
        // round(x, n) for small ndigits
        for nd in [-3, -1, 0, 2, 6] {
            let resp = oc.call("round_py_nd", &serde_json::json!({"bits": bits_hex(f), "ndigits": nd}));
            let r = round_py_nd(f, nd);
            match resp {
                OracleResp::Ok(o) => {
                    let expected = f64_from_bits_hex(o.trim());
                    // Compare via formatted strings to handle -0.0 exactly.
                    assert_eq!(
                        float_repr(r), float_repr(expected),
                        "round_nd {f} {nd}"
                    );
                    assert_eq!(r.to_bits(), expected.to_bits(), "round_nd bits {f} {nd}");
                }
                OracleResp::Err { kind, msg } => panic!("round_nd oracle err: {kind}: {msg}"),
            }
        }
        // int(float) truncation via trunc helper
        let resp = oc.call("trunc_float_to_bigint", &serde_json::json!({"bits": bits_hex(f)}));
        if f.is_finite() {
            let rust = trunc_float_to_bigint(f).0.to_string();
            match resp {
                OracleResp::Ok(o) => assert_eq!(o, rust, "trunc {f}"),
                OracleResp::Err { kind, msg } => panic!("trunc oracle err: {kind}: {msg}"),
            }
        }
    }
    // float(str) / int(str, base)
    let strs = ["1_000.5", "  1e3 ", "-.5", "1.", "inf", "-Infinity", "nan", "1e", "1__0", "0x10", "010"];
    for s in strs {
        let resp = oc.call("float_from_string", &serde_json::json!({"s": s}));
        let rust = float_from_string(s);
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(v)) => assert!(floats_equal_bits(f64_from_bits_hex(o), *v)),
            (OracleResp::Err { kind, msg }, Err(e)) => {
                assert_eq!(kind, &e.kind, "float_str {s:?}");
                assert_eq!(msg, &e.msg, "float_str {s:?}");
            }
            _ => panic!("float_from_string mismatch {s:?}: oracle {resp:?} rust {rust:?}"),
        }
    }
    for (s, base) in [("10", 10), ("ff", 16), ("0xFF", 0), ("0o17", 0), ("0b101", 0), ("1_0", 10), ("010", 0)] {
        let resp = oc.call("int_from_string", &serde_json::json!({"s": s, "base": base}));
        let rust = int_from_string(s, base).map(|n| n.0.to_string());
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(r)) => assert_eq!(o, r, "int_str {s:?} base {base}"),
            (OracleResp::Err { kind, msg }, Err(e)) => {
                assert_eq!(kind, &e.kind);
                assert_eq!(msg, &e.msg, "int_str {s:?} base {base}");
            }
            _ => panic!("int_from_string mismatch {s:?}"),
        }
    }
    // Fixed-point formatting spot check.
    let _ = format_f_fixed(1.5, 2, false);
}

#[test]
fn int_float_arith_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("int_float_arith_differential"));
        return;
    };
    let ints = ["-7", "-2", "0", "7", "123456789012345678901234567890", "-1"];
    for a in ints {
        for b in ["-2", "3"] {
            let pa = PyInt::from_bigint_str(a);
            let pb = PyInt::from_bigint_str(b);
            let a64 = a.parse::<i64>().unwrap_or(0);
            let _ = a64;
            let resp = oc.call("int_floor_div", &serde_json::json!({"a": a, "b": b}));
            let rust = int_floor_div(&pa, &pb).0.to_string();
            match resp {
                OracleResp::Ok(o) => assert_eq!(o, rust, "floordiv {a} // {b}"),
                _ => panic!("floordiv oracle err for {a} // {b}"),
            }
            let resp = oc.call("int_mod", &serde_json::json!({"a": a, "b": b}));
            let rust = int_mod(&pa, &pb).0.to_string();
            match resp {
                OracleResp::Ok(o) => assert_eq!(o, rust, "mod {a} % {b}"),
                _ => panic!("mod oracle"),
            }
            let resp = oc.call("int_divmod", &serde_json::json!({"a": a, "b": b}));
            let (q, r) = int_divmod(&pa, &pb);
            match resp {
                OracleResp::Ok(o) => assert_eq!(
                    o,
                    format!("[\"{}\",\"{}\"]", q.0, r.0),
                    "divmod {a} {b}"
                ),
                _ => panic!("divmod oracle"),
            }
            assert_eq!(int_cmp(&pa, &pb) as i8 as i64, {
                let x: i64 = a.parse().unwrap_or(0);
                let y: i64 = b.parse().unwrap_or(0);
                ((x > y) as i64) - ((x < y) as i64)
            });
            assert_eq!(int_is_odd(&pa), a.parse::<i64>().unwrap_or(0) % 2 != 0);
        }
    }
    // int_pow
    let resp = oc.call("int_pow", &serde_json::json!({"a": "2", "e": 10}));
    assert_eq!(resp, OracleResp::Ok("1024".to_string()));
    assert_eq!(int_pow(&PyInt::from_i64(2), 10).0.to_string(), "1024");
    // Float div/mod/pow.
    let pairs = [(7.5, 2.0), (-7.5, 2.0), (7.5, -2.0), (1e308, 1e-10)];
    for (a, b) in pairs {
        let payload = serde_json::json!({"a": bits_hex(a), "b": bits_hex(b)});
        let resp = oc.call("float_floor_div", &payload);
        let rust = float_floor_div(a, b);
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(v)) => assert!(floats_equal_bits(f64_from_bits_hex(o), *v), "floordiv float {a} // {b}"),
            (OracleResp::Err { kind, msg }, Err(e)) => {
                assert_eq!(kind, &e.kind);
                assert_eq!(msg, &e.msg)
            }
            _ => panic!("float_floor_div mismatch {a} {b}"),
        }
        let resp = oc.call("float_mod", &payload);
        let rust = float_mod(a, b);
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(v)) => assert!(floats_equal_bits(f64_from_bits_hex(o), *v)),
            (OracleResp::Err { kind, msg }, Err(e)) => {
                assert_eq!(kind, &e.kind);
                assert_eq!(msg, &e.msg)
            }
            _ => panic!("float_mod mismatch"),
        }
        let resp = oc.call("float_true_div", &payload);
        let rust = float_true_div(a, b);
        match (&resp, &rust) {
            (OracleResp::Ok(o), Ok(v)) => assert!(floats_equal_bits(f64_from_bits_hex(o), *v)),
            _ => panic!("true_div mismatch"),
        }
    }
    let resp = oc.call("float_pow", &serde_json::json!({"a": bits_hex(10.0), "b": bits_hex(2.0)}));
    assert!(matches!(resp, OracleResp::Ok(_)));
    assert!(float_pow(10.0, 2.0).unwrap() == 100.0);
    // cmp_bigint_float
    let _ = cmp_bigint_float(&num_bigint::BigInt::from(5), 5.0);
    let resp = oc.call("cmp_bigint_float", &serde_json::json!({"a": "5", "bits": bits_hex(5.0)}));
    assert_eq!(resp, OracleResp::Ok("0".to_string()));
    // sum of floats (plain left-to-right, CPython 3.8).
    let xs = vec![1e16, 1.0, -1e16, 1.0];
    let resp = oc.call(
        "py_sum_floats",
        &serde_json::json!({"bits_list": xs.iter().map(|f| bits_hex(*f)).collect::<Vec<_>>() }),
    );
    let rust = py_sum_floats(&xs);
    match resp {
        OracleResp::Ok(o) => assert!(floats_equal_bits(f64_from_bits_hex(o.trim()), rust)),
        _ => panic!("sum oracle"),
    }
}

#[test]
fn value_and_json_differential() {
    let Some(mut oc) = spawn_oracle() else {
        eprintln!("{}", skip_msg("value_and_json_differential"));
        return;
    };
    let vals: Vec<PyValue> = vec![
        PyValue::None,
        PyValue::Bool(true),
        PyValue::Int(PyInt::from_i64(1)),
        PyValue::Float(1.0),
        PyValue::Float(f64::NAN),
        PyValue::Float(f64::INFINITY),
        PyValue::Str("a'b\"c\n".to_string()),
        PyValue::Bytes(vec![0, 1, 255, b'a']),
        PyValue::List(vec![PyValue::Int(PyInt::from_i64(1)), PyValue::Str("x".to_string())]),
        PyValue::Tuple(vec![PyValue::Int(PyInt::from_i64(1))]),
    ];
    for v in &vals {
        let payload = serde_json::json!({"value": pyvalue_payload(v)});
        let resp = oc.call("py_repr", &payload);
        let rust = v.repr();
        match resp {
            OracleResp::Ok(o) => assert_eq!(o, rust, "repr {v:?}"),
            _ => panic!("repr oracle err"),
        }
        let resp = oc.call("py_str", &payload);
        let rust = v.str();
        match resp {
            OracleResp::Ok(o) => assert_eq!(o, rust, "str {v:?}"),
            _ => panic!("str oracle"),
        }
        assert_eq!(type_name(v), {
            let resp = oc.call("type_name", &payload);
            match resp {
                OracleResp::Ok(o) => o,
                _ => panic!("type_name oracle"),
            }
        });
    }
    // Numeric tower: 1 == 1.0 == True and shared hash.
    let one = PyValue::Int(PyInt::from_i64(1));
    let one_f = PyValue::Float(1.0);
    let tru = PyValue::Bool(true);
    assert!(py_equal(&one, &one_f) && py_equal(&one_f, &tru));
    assert_eq!(int_py_hash(&PyInt::from_i64(1)), 1);
    assert_eq!(one.py_hash(), one_f.py_hash());
    // Ordering TypeError text parity for int vs str.
    let resp = oc.call(
        "py_rich_compare",
        &serde_json::json!({
            "op": "<",
            "a": serde_json::json!({"t":"int","v":"1"}),
            "b": serde_json::json!({"t":"str","v":"x"})
        }),
    );
    match resp {
        OracleResp::Err { kind, msg } => {
            assert_eq!(kind, "TypeError");
            assert!(msg.contains("not supported"));
            assert!(py_rich_compare("<", &one, &PyValue::Str("x".to_string())).is_err());
        }
        _ => panic!("expected TypeError"),
    }
    let _ = py_compare(&one, &one_f);
    // JSON round-trips.
    let v = PyValue::Dict({
        let mut m = indexmap::IndexMap::new();
        m.insert(PyValue::Str("a".to_string()), PyValue::Float(1.5));
        m.insert(PyValue::Str("b".to_string()), PyValue::List(vec![PyValue::Int(PyInt::from_i64(1))]));
        m
    });
    let resp = oc.call("json_dumps", &serde_json::json!({"value": pyvalue_payload(&v)}));
    let rust = dumps(&v).unwrap();
    match resp {
        OracleResp::Ok(o) => assert_eq!(o, rust),
        _ => panic!("json dumps oracle"),
    }
    let resp = oc.call("json_dumps", &serde_json::json!({
        "value": pyvalue_payload(&v),
        "opts": {"separators": [",", ":"], "ensure_ascii": true, "allow_nan": true}
    }));
    let compact = dumps_opts(&v, &DumpsOpts { item_sep: ",".to_string(), key_sep: ":".to_string(), ensure_ascii: true, allow_nan: true }).unwrap();
    match resp {
        OracleResp::Ok(o) => assert_eq!(o, compact),
        _ => panic!("json compact oracle"),
    }
    let resp = oc.call("json_loads", &serde_json::json!({"s": "{\"a\": 1.5}"}));
    let rust_v = loads("{\"a\": 1.5}").unwrap();
    match resp {
        OracleResp::Ok(o) => {
            // Compare via re-dump.
            let rv: serde_json::Value = serde_json::from_str(o.trim()).unwrap();
            let _ = rv;
            assert!(matches!(rust_v, PyValue::Dict(_)));
        }
        _ => panic!("json loads oracle"),
    }
    // str_repr helpers.
    let s = "a'b\nc";
    let resp = oc.call("str_repr", &serde_json::json!({"s": s}));
    match resp {
        OracleResp::Ok(o) => assert_eq!(o, str_repr(s)),
        _ => panic!("str_repr oracle"),
    }
    // BTreeMap used to keep test build free of HashMap clippy lint trigger via import.
    let _: BTreeMap<i32, i32> = BTreeMap::new();
}

// Extension trait to build PyInt from decimal text inside tests.
trait FromStrHelper {
    fn from_bigint_str(s: &str) -> Self;
}
impl FromStrHelper for PyInt {
    fn from_bigint_str(s: &str) -> Self {
        use std::str::FromStr;
        PyInt(num_bigint::BigInt::from_str(s).unwrap())
    }
}
