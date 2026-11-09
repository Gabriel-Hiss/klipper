//! Dynamic value type reproducing CPython 3.8 object semantics
//! (`Objects/object.c`, `Objects/typeobject.c`).
//!
//! `repr`/`str` match CPython for None, bool, int, float, str, bytes,
//! list, tuple and dict; numeric equality/hash reconcile the tower
//! (`1 == 1.0 == True`, `Objects/floatobject.c:float_hash`,
//! `Objects/longobject.c:long_hash`); ordering raises
//! `TypeError: '<' not supported between instances of 'X' and 'Y'`
//! cross-type where CPython does.

use crate::int::PyInt;
use crate::text;
use indexmap::IndexMap;
use std::hash::{Hash, Hasher};
use num_bigint::BigInt;

/// Dynamic Python value (opaque host objects are handles, not data).
#[derive(Clone, Debug, Default)]
pub enum PyValue {
    /// `None`
    #[default]
    None,
    /// `bool`
    Bool(bool),
    /// `int`
    Int(PyInt),
    /// `float`
    Float(f64),
    /// `str`
    Str(String),
    /// `bytes`
    Bytes(Vec<u8>),
    /// `list`
    List(Vec<PyValue>),
    /// `tuple`
    Tuple(Vec<PyValue>),
    /// `dict` (insertion ordered)
    Dict(IndexMap<PyValue, PyValue>),
    /// Opaque host object handle (escape hatch for Python objects the
    /// port keeps on the host side).
    Host(u64),
}

/// CPython type name of a value (`type(v).__name__`).
pub fn type_name(v: &PyValue) -> &'static str {
    match v {
        PyValue::None => "NoneType",
        PyValue::Bool(_) => "bool",
        PyValue::Int(_) => "int",
        PyValue::Float(_) => "float",
        PyValue::Str(_) => "str",
        PyValue::Bytes(_) => "bytes",
        PyValue::List(_) => "list",
        PyValue::Tuple(_) => "tuple",
        PyValue::Dict(_) => "dict",
        PyValue::Host(_) => "object",
    }
}

impl PyValue {
    /// `bool(v)` (`Objects/object.c:PyObject_IsTrue`): None/0/0.0/""/b""/[]/{} are False.
    pub fn truthy(&self) -> bool {
        match self {
            PyValue::None => false,
            PyValue::Bool(b) => *b,
            PyValue::Int(n) => !n.is_zero(),
            PyValue::Float(f) => *f != 0.0,
            PyValue::Str(s) => !s.is_empty(),
            PyValue::Bytes(b) => !b.is_empty(),
            PyValue::List(l) => !l.is_empty(),
            PyValue::Tuple(t) => !t.is_empty(),
            PyValue::Dict(d) => !d.is_empty(),
            PyValue::Host(_) => true,
        }
    }

    /// `str(v)` (`Objects/object.c:PyObject_Str`).
    pub fn str(&self) -> String {
        match self {
            PyValue::None => "None".to_string(),
            PyValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Float(f) => text::float_repr(*f),
            PyValue::Str(s) => s.clone(),
            PyValue::Bytes(b) => text::bytes_repr(b),
            PyValue::List(_) | PyValue::Tuple(_) | PyValue::Dict(_) => self.repr(),
            PyValue::Host(h) => format!("<host object at {:#x}>", h),
        }
    }

    /// `repr(v)` (`Objects/object.c:PyObject_Repr`).
    pub fn repr(&self) -> String {
        match self {
            PyValue::None => "None".to_string(),
            PyValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
            PyValue::Int(n) => n.0.to_string(),
            PyValue::Float(f) => text::float_repr(*f),
            PyValue::Str(s) => text::str_repr(s),
            PyValue::Bytes(b) => text::bytes_repr(b),
            PyValue::List(l) => {
                let parts: Vec<String> = l.iter().map(|v| v.repr()).collect();
                format!("[{}]", parts.join(", "))
            }
            PyValue::Tuple(t) => {
                if t.len() == 1 {
                    format!("({},)", t[0].repr())
                } else {
                    let parts: Vec<String> = t.iter().map(|v| v.repr()).collect();
                    format!("({})", parts.join(", "))
                }
            }
            PyValue::Dict(d) => {
                let parts: Vec<String> = d
                    .iter()
                    .map(|(k, v)| format!("{}: {}", k.repr(), v.repr()))
                    .collect();
                format!("{{{}}}", parts.join(", "))
            }
            PyValue::Host(h) => format!("<host object at {:#x}>", h),
        }
    }

    /// Identity-style hash for dict keys, CPython-compatible for
    /// numbers (`PyObject_Hash`): `hash(1) == hash(1.0) == hash(True)`.
    /// Unhashable containers panic in CPython (`unhashable type`);
    /// here they return 0 and callers validate via [`is_hashable`].
    pub fn py_hash(&self) -> i64 {
        match self {
            PyValue::None => 0x345678,
            PyValue::Bool(b) => PyInt::from_i64(if *b { 1 } else { 0 }).py_hash(),
            PyValue::Int(n) => n.py_hash(),
            PyValue::Float(f) => text::float_py_hash(*f),
            PyValue::Str(s) => {
                // CPython str hash uses SipHash with random key; the exact
                // value is not comparable across runs. Use a deterministic
                // FNV-like mix so equal strings collide (only requirement).
                let mut h: i64 = -1;
                for b in s.as_bytes() {
                    // splitmix-ish
                    let mut x = (*b as i64).wrapping_add(0x9e3779b97f4a7c15u64 as i64);
                    x ^= x >> 29;
                    x = x.wrapping_mul(0xbf58476d1ce4e5b9u64 as i64);
                    x ^= x >> 32;
                    h = h.wrapping_add(x);
                }
                if h == -1 { -2 } else { h }
            }
            PyValue::Bytes(b) => {
                let mut h: i64 = -1;
                for &x in b {
                    let mut v = (x as i64).wrapping_add(0x9e3779b97f4a7c15u64 as i64);
                    v ^= v >> 29;
                    v = v.wrapping_mul(0xbf58476d1ce4e5b9u64 as i64);
                    v ^= v >> 32;
                    h = h.wrapping_add(v);
                }
                if h == -1 { -2 } else { h }
            }
            PyValue::Tuple(t) => {
                // Tuples are hashable when all elements are.
                let mut h: i64 = 0x345678;
                for el in t {
                    let m = el.py_hash();
                    h = h.wrapping_add(m.wrapping_mul(1000003));
                }
                if h == -1 { -2 } else { h }
            }
            _ => 0,
        }
    }

    /// Whether the value can be a dict key (CPython `PyObject_Hash`).
    pub fn is_hashable(&self) -> bool {
        match self {
            PyValue::List(_) | PyValue::Dict(_) => false,
            PyValue::Tuple(t) => t.iter().all(|v| v.is_hashable()),
            _ => true,
        }
    }

    /// Numeric coercion to `f64` for cross-type compares (CPython
    /// `PyNumber_Float`, `Objects/abstract.c`): ints round half-even and
    /// give `None` when they overflow a float.
    pub fn to_f64(&self) -> Option<f64> {
        match self {
            PyValue::Float(f) => Some(*f),
            PyValue::Int(n) => text::int_to_f64(&n.0),
            PyValue::Bool(b) => Some(if *b { 1.0 } else { 0.0 }),
            _ => None,
        }
    }
}

/// Key repr used in `KeyError` messages (CPython `Objects/exceptions.c:KeyError`).
pub fn repr_key(s: &str) -> String {
    text::str_repr(s)
}

/// `PyValue` equality (`Objects/object.c:PyObject_RichCompare` with `Py_EQ`).
/// Numeric tower: `1 == 1.0 == True` exactly (`PyObject_RichCompare` on
/// `long`/`float` compares exact rational values, not via double rounding).
pub fn py_equal(a: &PyValue, b: &PyValue) -> bool {
    // Cheap identity fast path.
    match (a, b) {
        (PyValue::None, PyValue::None) => true,
        (PyValue::Str(x), PyValue::Str(y)) => x == y,
        (PyValue::Bytes(x), PyValue::Bytes(y)) => x == y,
        (PyValue::Host(x), PyValue::Host(y)) => x == y,
        (PyValue::List(x), PyValue::List(y)) => {
            x.len() == y.len() && x.iter().zip(y.iter()).all(|(p, q)| py_equal(p, q))
        }
        (PyValue::Tuple(x), PyValue::Tuple(y)) => {
            x.len() == y.len() && x.iter().zip(y.iter()).all(|(p, q)| py_equal(p, q))
        }
        (PyValue::Dict(x), PyValue::Dict(y)) => {
            if x.len() != y.len() {
                return false;
            }
            // Order-insensitive: each key of x found in y with equal value.
            x.iter().all(|(k, v)| {
                y.iter()
                    .find(|(k2, _)| py_equal(k, k2))
                    .map(|(_, v2)| py_equal(v, v2))
                    .unwrap_or(false)
            })
        }
        _ => numeric_equal(a, b),
    }
}

fn is_number(v: &PyValue) -> bool {
    matches!(v, PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_))
}

fn numeric_equal(a: &PyValue, b: &PyValue) -> bool {
    if !is_number(a) || !is_number(b) {
        // int/str etc are unequal, never TypeError for ==
        return false;
    }
    // bool vs number handled numerically
    let af = as_float_exact(a);
    let bf = as_float_exact(b);
    // Exact: if either is nan, unequal.
    match (af, bf) {
        (Num::Nan, _) | (_, Num::Nan) => false,
        (Num::Inf(s1), Num::Inf(s2)) => s1 == s2,
        (Num::Inf(_), _) => false,
        (_, Num::Inf(_)) => false,
        (Num::Rat(ref x), Num::Rat(ref y)) => x == y,
    }
}

pub enum Num {
    Nan,
    Inf(bool),
    Rat(crate::text::Rat),
}

/// Exact numeric value used for `==` (avoids double-rounding artifacts).
pub fn as_float_exact(v: &PyValue) -> Num {
    match v {
        PyValue::Float(f) => {
            if f.is_nan() {
                Num::Nan
            } else if f.is_infinite() {
                Num::Inf(*f > 0.0)
            } else {
                Num::Rat(crate::text::Rat::from_f64(*f))
            }
        }
        PyValue::Int(n) => Num::Rat(crate::text::Rat::from_bigint(n.0.clone())),
        PyValue::Bool(b) => Num::Rat(crate::text::Rat::from_bigint(BigInt::from(if *b { 1i64 } else { 0i64 }))),
        _ => Num::Nan,
    }
}

/// Ordering (`Py_LT` etc.) as CPython (`Objects/object.c:do_richcompare`).
/// Returns `Ok(Ordering)` or the CPython `TypeError` text:
/// `'<' not supported between instances of 'int' and 'str'`.
pub fn py_compare(a: &PyValue, b: &PyValue) -> Result<std::cmp::Ordering, String> {
    // numbers compare numerically (nan -> everything unequal, but ordering
    // helpers treat nan as: any ordering with nan is False; for the
    // Ordering API we emulate by returning Less when unordered? Instead
    // expose `py_rich_compare` for exact boolean semantics; this wrapper
    // maps nan to a canonical Less to keep total-order callers sane.
    match (a, b) {
        (PyValue::Str(x), PyValue::Str(y)) => Ok(x.cmp(y)),
        (PyValue::Bytes(x), PyValue::Bytes(y)) => Ok(x.cmp(y)),
        (PyValue::Tuple(x), PyValue::Tuple(y)) => {
            for (p, q) in x.iter().zip(y.iter()) {
                if !py_equal(p, q) {
                    return py_compare(p, q);
                }
            }
            Ok(x.len().cmp(&y.len()))
        }
        (PyValue::List(x), PyValue::List(y)) => {
            for (p, q) in x.iter().zip(y.iter()) {
                if !py_equal(p, q) {
                    return py_compare(p, q);
                }
            }
            Ok(x.len().cmp(&y.len()))
        }
        (PyValue::Dict(_), PyValue::Dict(_)) => Err(format!(
            "'<' not supported between instances of '{}' and '{}'",
            type_name(a),
            type_name(b)
        )),
        _ if is_number(a) && is_number(b) => {
            let af = as_float_exact(a);
            let bf = as_float_exact(b);
            match (af, bf) {
                (Num::Nan, _) | (_, Num::Nan) => {
                    // CPython: nan comparisons are all False; for Ordering api
                    // report Less (callers needing exact booleans use rich compare).
                    Ok(std::cmp::Ordering::Less)
                }
                (Num::Inf(s1), Num::Inf(s2)) => Ok(match (s1, s2) {
                    (true, true) | (false, false) => std::cmp::Ordering::Equal,
                    (true, false) => std::cmp::Ordering::Greater,
                    (false, true) => std::cmp::Ordering::Less,
                }),
                (Num::Inf(false), _) => Ok(std::cmp::Ordering::Less),
                (Num::Inf(true), _) => Ok(std::cmp::Ordering::Greater),
                (_, Num::Inf(false)) => Ok(std::cmp::Ordering::Greater),
                (_, Num::Inf(true)) => Ok(std::cmp::Ordering::Less),
                (Num::Rat(ref x), Num::Rat(ref y)) => Ok(x.cmp(y)),
            }
        }
        _ => Err(format!(
            "'<' not supported between instances of '{}' and '{}'",
            type_name(a),
            type_name(b)
        )),
    }
}

/// Exact rich-compare booleans (`PyObject_RichCompareBool`): `nan`
/// orderings are `False`, lists and tuples compare at their first unequal
/// item with the same operator (`Objects/listobject.c:list_richcompare`),
/// and unordered pairs raise `TypeError: '<=' not supported between
/// instances of 'X' and 'Y'` naming the operator used.
pub fn py_rich_compare(op: &str, a: &PyValue, b: &PyValue) -> Result<bool, String> {
    use std::cmp::Ordering;
    let apply = |ord: Ordering| -> bool {
        match op {
            "<" => ord == Ordering::Less,
            "<=" => ord != Ordering::Greater,
            ">" => ord == Ordering::Greater,
            _ => ord != Ordering::Less,
        }
    };
    match op {
        "==" => return Ok(py_equal(a, b)),
        "!=" => return Ok(!py_equal(a, b)),
        "<" | "<=" | ">" | ">=" => {}
        _ => return Err(format!("unknown comparison operator {}", op)),
    }
    match (a, b) {
        (PyValue::Str(x), PyValue::Str(y)) => Ok(apply(x.cmp(y))),
        (PyValue::Bytes(x), PyValue::Bytes(y)) => Ok(apply(x.cmp(y))),
        (PyValue::List(x), PyValue::List(y)) | (PyValue::Tuple(x), PyValue::Tuple(y)) => {
            match x.iter().zip(y.iter()).find(|(p, q)| !py_equal(p, q)) {
                Some((p, q)) => py_rich_compare(op, p, q),
                None => Ok(apply(x.len().cmp(&y.len()))),
            }
        }
        _ if is_number(a) && is_number(b) => {
            let (af, bf) = (as_float_exact(a), as_float_exact(b));
            if matches!(af, Num::Nan) || matches!(bf, Num::Nan) {
                return Ok(false);
            }
            py_compare(a, b).map(apply)
        }
        _ => Err(format!(
            "'{}' not supported between instances of '{}' and '{}'",
            op,
            type_name(a),
            type_name(b)
        )),
    }
}

/// `hash` of an `int` value usable as a dict key (`long_hash`).
pub fn int_py_hash(n: &PyInt) -> i64 {
    n.py_hash()
}

/// `hash` of a `float` value. `1.0`/`1`/`True` share it.
pub fn float_py_hash(f: f64) -> i64 {
    text::float_py_hash(f)
}

impl PartialEq for PyValue {
    /// `PyObject_RichCompare` with `Py_EQ` (numeric tower, see `py_equal`).
    fn eq(&self, other: &Self) -> bool {
        py_equal(self, other)
    }
}
impl Eq for PyValue {}
impl Hash for PyValue {
    /// `PyObject_Hash`: numbers hash by CPython value (`long_hash`/
    /// `float_hash`) so `1`/`1.0`/`True` collide; equal strings/bytes hash
    /// equally (values are randomised in CPython and only need to collide
    /// within this process).
    fn hash<H: Hasher>(&self, state: &mut H) {
        match self {
            PyValue::None => 0x345678_i64.hash(state),
            PyValue::Bool(b) => PyInt::from_i64(if *b { 1 } else { 0 }).py_hash().hash(state),
            PyValue::Int(n) => n.py_hash().hash(state),
            PyValue::Float(f) => text::float_py_hash(*f).hash(state),
            PyValue::Str(s) => {
                0x53737472u32.hash(state);
                s.hash(state);
            }
            PyValue::Bytes(b) => {
                0x42595445u32.hash(state);
                b.hash(state);
            }
            PyValue::Tuple(t) => {
                0x54504c45u32.hash(state);
                t.hash(state);
            }
            PyValue::List(xs) => {
                xs.hash(state);
            }
            PyValue::Dict(d) => {
                for (k, v) in d.iter() {
                    k.hash(state);
                    v.hash(state);
                }
            }
            PyValue::Host(h) => h.hash(state),
        }
    }
}
