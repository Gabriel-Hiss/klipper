//! Value model for recordings, mirroring `parity/record/FORMAT.md`.
//!
//! The recorder encodes Python/C values as tagged JSON objects so that a
//! replay can tell the types apart and compare them exactly:
//!
//! | `Value`     | JSON encoding                                              |
//! |-------------|------------------------------------------------------------|
//! | `Null`      | `null`                                                     |
//! | `Bool`      | `true` / `false`                                           |
//! | `Int`       | JSON number within `i64` range                             |
//! | `BigInt`    | `{"$int": "digits"}` (out-of-range Python integers)        |
//! | `Float`     | `{"$float": {"hex": "0x1.9ap-4", "repr": "0.4"}}`          |
//! | `Bytes`     | `{"$bytes": "deadbeef"}` (lowercase hex)                   |
//! | `Str`       | JSON string                                                |
//! | `List`      | JSON array                                                 |
//! | `Tuple`     | `{"$tuple": [...]}`                                        |
//! | `Dict`      | `{"$dict": [[key, value], ...]}` in insertion order        |
//! | `Struct`    | `{"$struct": {"type": "...", "fields": [[name, v], ...]}}` |
//! | `Opaque`    | `{"$opaque": {"type": "...", "addr": "0x..."}}`            |
//! | `ObjectId`  | `{"$oid": "Thermistor#3"}` (stable object identity)        |
//!
//! Floats and bytes are never emitted bare, so a bare JSON number is
//! always an `Int` and a bare JSON array is always a `List`.

use std::fmt;

use crate::hexfloat::{f64_from_hex, f64_to_hex};

/// A lossless float: CPython hex-float spelling plus the `repr` spelling.
/// Both spellings must match at replay time; either alone can hide a
/// one-ULP difference, and together they are conclusive.
#[derive(Debug, Clone, PartialEq)]
pub struct FloatVal {
    pub hex: String,
    pub repr: String,
}

impl FloatVal {
    pub fn new(x: f64) -> FloatVal {
        FloatVal {
            hex: f64_to_hex(x),
            repr: py_float_repr(x),
        }
    }

    /// The `f64` value, recovered from the hex spelling.
    pub fn to_f64(&self) -> Option<f64> {
        f64_from_hex(&self.hex)
    }
}

/// CPython 3.8 `repr(float)` spelling: shortest round-trip decimal.
/// Rust `Display` for `f64` is also shortest round-trip since 1.32, and
/// the spellings agree for all values except the specials, which match
/// anyway (`inf`, `-inf`, `nan`). Only one adjustment is needed: Rust
/// prints integral floats without a fraction (`1` for `1.0`), while
/// CPython keeps `.0` (`1.0`).
pub fn py_float_repr(x: f64) -> String {
    if x.is_infinite() || x.is_nan() {
        return x.to_string();
    }
    let s = x.to_string();
    if s.contains('.') || s.contains('e') {
        s
    } else {
        format!("{s}.0")
    }
}

/// One recorded value.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Int(i64),
    BigInt(String),
    Float(FloatVal),
    Bytes(Vec<u8>),
    Str(String),
    List(Vec<Value>),
    Tuple(Vec<Value>),
    Dict(Vec<(Value, Value)>),
    Struct {
        type_name: String,
        fields: Vec<(String, Value)>,
    },
    Opaque {
        type_name: String,
        addr: String,
    },
    ObjectId(String),
}

impl Value {
    pub fn int(x: i64) -> Value {
        Value::Int(x)
    }

    pub fn float(x: f64) -> Value {
        Value::Float(FloatVal::new(x))
    }

    pub fn bytes(x: &[u8]) -> Value {
        Value::Bytes(x.to_vec())
    }

    pub fn str(s: &str) -> Value {
        Value::Str(s.to_string())
    }

    /// Short printable rendering for divergence reports.
    pub fn render(&self) -> String {
        match self {
            Value::Null => "null".to_string(),
            Value::Bool(b) => b.to_string(),
            Value::Int(i) => i.to_string(),
            Value::BigInt(s) => s.clone(),
            Value::Float(f) => format!("{} ({})", f.repr, f.hex),
            Value::Bytes(b) => format!("bytes(0x{})", hex_encode(b)),
            Value::Str(s) => format!("{s:?}"),
            Value::List(items) => {
                let parts: Vec<String> = items.iter().map(Value::render).collect();
                format!("[{}]", parts.join(", "))
            }
            Value::Tuple(items) => {
                let parts: Vec<String> = items.iter().map(Value::render).collect();
                format!("({})", parts.join(", "))
            }
            Value::Dict(pairs) => {
                let parts: Vec<String> = pairs
                    .iter()
                    .map(|(k, v)| format!("{}: {}", k.render(), v.render()))
                    .collect();
                format!("{{{}}}", parts.join(", "))
            }
            Value::Struct { type_name, fields } => {
                let parts: Vec<String> = fields
                    .iter()
                    .map(|(name, v)| format!("{name}={}", v.render()))
                    .collect();
                format!("{type_name} {{{}}}", parts.join(", "))
            }
            Value::Opaque { type_name, addr } => {
                format!("{type_name} @ {addr}")
            }
            Value::ObjectId(id) => id.clone(),
        }
    }
}

/// An exception raised instead of a return value.
#[derive(Debug, Clone, PartialEq)]
pub struct Exception {
    pub class_name: String,
    pub message: String,
}

/// Side effects observed during the call.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct SideEffects {
    /// Mutated arguments, as `(argument index, value after the call)` pairs.
    pub mutated_args: Vec<(usize, Value)>,
    /// MCU messages emitted during the call, as raw bytes.
    pub mcu_messages: Vec<Vec<u8>>,
    /// Log lines emitted during the call, oldest first.
    pub log_lines: Vec<String>,
}

/// The shape of a recorded call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecordKind {
    /// Python unit call (profile tracer).
    Python,
    /// Python-to-C boundary call (wrapping the cffi `lib`).
    CBoundary,
    /// C-to-C internal call (wrapping linker interposition).
    CInternal,
}

/// One line of a recording file: one call.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub v: u64,
    pub unit: String,
    pub scenario: String,
    pub seq: u64,
    pub function: String,
    pub args: Vec<Value>,
    pub ret: Option<Value>,
    pub exc: Option<Exception>,
    pub side_effects: SideEffects,
    pub object_id: Option<String>,
    pub kind: RecordKind,
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.render())
    }
}

/// Lowercase hex encoding, used for `Bytes` values.
pub fn hex_encode(data: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(data.len() * 2);
    for b in data {
        out.push(DIGITS[(b >> 4) as usize] as char);
        out.push(DIGITS[(b & 0xf) as usize] as char);
    }
    out
}

/// Decode lowercase or uppercase hex into bytes.
pub fn hex_decode(s: &str) -> Result<Vec<u8>, String> {
    if s.len() % 2 != 0 {
        return Err(format!("odd hex length: {s}"));
    }
    let digits = s.as_bytes();
    let mut out = Vec::with_capacity(s.len() / 2);
    let mut i = 0;
    while i < digits.len() {
        let hi = hex_digit(digits[i])?;
        let lo = hex_digit(digits[i + 1])?;
        out.push((hi << 4) | lo);
        i += 2;
    }
    Ok(out)
}

fn hex_digit(c: u8) -> Result<u8, String> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(format!("bad hex digit: {}", c as char)),
    }
}
