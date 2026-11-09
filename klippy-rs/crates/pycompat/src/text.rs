//! Float text, `%` formatting, `format()` mini-language, `str.format`,
//! rounding and numeric parsing reproducing CPython 3.8.2.
//!
//! Each section ports the CPython function it names branch for branch, so
//! texts, padding quirks and exception messages match:
//! - float text: `Python/pystrtod.c:format_float_short` over the digits of
//!   `Python/dtoa.c:_Py_dg_dtoa` (modes 0, 2 and 3), computed exactly from
//!   the binary value here (`repr` digits come from Rust's shortest
//!   round-trip formatter, which picks the same digits as mode 0);
//! - `%`: `Objects/unicodeobject.c:PyUnicode_Format`;
//! - `format()`: `Python/formatter_unicode.c` plus `object.__format__`;
//! - `str.format`: `Objects/stringlib/unicode_format.h`;
//! - `round`: `Objects/floatobject.c:float___round___impl`/`double_round`;
//! - `int(str, base)`: `Objects/longobject.c:PyLong_FromUnicodeObject`;
//! - `float(str)`: `Objects/floatobject.c:PyFloat_FromString`;
//! - float `/`, `//`, `%`, `**`: `Objects/floatobject.c`.
//!
//! Strings here are Rust `String`s, which cannot hold lone surrogates:
//! `chr()` of a surrogate code point (`%c`, `format(x, 'c')`) is reported as
//! a `ValueError` instead of producing one.

use crate::int::PyInt;
use crate::libm;
use crate::unicode;
use num_bigint::{BigInt, BigUint, Sign};
use num_integer::Integer;
use num_traits::{One, ToPrimitive, Zero};
use std::borrow::Cow;
use std::cmp::Ordering;

/// Error text mirroring a CPython exception `str(e)`.
#[derive(Debug, Clone, PartialEq)]
pub struct PyError {
    /// Exception class name (`ValueError`, `OverflowError`, ...).
    pub kind: &'static str,
    /// Exception message (the part after `ValueError: `).
    pub msg: String,
}

impl PyError {
    /// New error with kind and message (CPython `ValueError`/`TypeError`/`OverflowError`).
    pub fn new(kind: &'static str, msg: String) -> Self {
        PyError { kind, msg }
    }
    /// `"{kind}: {msg}"` like CPython `str()` of an exception (`Objects/exceptions.c`).
    pub fn display(&self) -> String {
        format!("{}: {}", self.kind, self.msg)
    }
}

impl std::fmt::Display for PyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.display())
    }
}

impl std::error::Error for PyError {}

fn value_error(msg: impl Into<String>) -> PyError {
    PyError::new("ValueError", msg.into())
}

fn type_error(msg: impl Into<String>) -> PyError {
    PyError::new("TypeError", msg.into())
}

fn overflow_error(msg: impl Into<String>) -> PyError {
    PyError::new("OverflowError", msg.into())
}

/// Exact rational `num / den` with `den > 0`, used for exact numeric
/// comparison of ints and floats (`Objects/longobject.c` vs
/// `Objects/floatobject.c`: comparisons use exact values, so `1 == 1.0`
/// exactly, not via double rounding). Finite floats decompose exactly as
/// `m * 2^e`; ints are `den == 1`.
#[derive(Clone, Debug)]
pub struct Rat {
    /// Numerator.
    pub num: BigInt,
    /// Denominator (`> 0`).
    pub den: BigInt,
}

impl Rat {
    /// Exact rational for an integer (CPython `Objects/longobject.c` exact value).
    pub fn from_bigint(n: BigInt) -> Self {
        Rat { num: n, den: BigInt::one() }
    }

    /// Exact rational for a finite `f64` (`f = m * 2^e` exactly, CPython `Objects/floatobject.c`).
    /// Callers must not pass `nan`/`inf`.
    pub fn from_f64(f: f64) -> Self {
        debug_assert!(f.is_finite());
        if f == 0.0 {
            return Rat::from_bigint(BigInt::zero());
        }
        let neg = f < 0.0;
        let a = libm::fabs(f);
        let bits = a.to_bits();
        let raw_exp = ((bits >> 52) & 0x7FF) as i32;
        let mant_bits = bits & 0x000F_FFFF_FFFF_FFFF;
        let (m, e): (BigInt, i32) = if raw_exp == 0 {
            (BigInt::from(mant_bits), -1074)
        } else {
            (BigInt::from((1u64 << 52) | mant_bits), raw_exp - 1023 - 52)
        };
        let (num, den) = if e >= 0 {
            (m << (e as usize), BigInt::one())
        } else {
            (m, BigInt::one() << ((-e) as usize))
        };
        if neg {
            Rat { num: -num, den }
        } else {
            Rat { num, den }
        }
    }
}

impl PartialEq for Rat {
    fn eq(&self, other: &Self) -> bool {
        (&self.num * &other.den) == (&other.num * &self.den)
    }
}

impl Eq for Rat {}

impl PartialOrd for Rat {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Rat {
    fn cmp(&self, other: &Self) -> Ordering {
        (&self.num * &other.den).cmp(&(&other.num * &self.den))
    }
}

// ---------------------------------------------------------------------------
// Exact decimal digits (`Python/dtoa.c:_Py_dg_dtoa`).
// ---------------------------------------------------------------------------

fn pow10(n: u64) -> BigInt {
    BigInt::from(10u32).pow(u32::try_from(n).expect("decimal exponent fits u32"))
}

/// `floor(log10(a))` for finite `a > 0`, exact (the libm estimate only
/// picks the starting point; exact comparisons settle it).
fn exact_exp10(a: f64) -> i64 {
    let mut e10 = libm::floor(libm::log10(a)) as i64;
    let r = Rat::from_f64(a);
    // Sign of `r - 10^k`.
    let cmp_pow10 = |k: i64| -> Ordering {
        let p = pow10(k.unsigned_abs());
        if k >= 0 {
            r.num.cmp(&(p * &r.den))
        } else {
            (&r.num * p).cmp(&r.den)
        }
    };
    while cmp_pow10(e10 + 1) != Ordering::Less {
        e10 += 1;
    }
    while cmp_pow10(e10) == Ordering::Less {
        e10 -= 1;
    }
    e10
}

/// `num / den` (`num >= 0`, `den > 0`) rounded to the nearest integer,
/// ties to even.
fn round_half_even_div(num: &BigInt, den: &BigInt) -> BigInt {
    let (q, r) = num.div_rem(den);
    let r2: BigInt = r << 1;
    match r2.cmp(den) {
        Ordering::Less => q,
        Ordering::Greater => q + 1,
        Ordering::Equal => {
            if q.is_odd() {
                q + 1
            } else {
                q
            }
        }
    }
}

/// `r * 10^shift` rounded half-even to an integer (`r >= 0`).
fn scaled_round(r: &Rat, shift: i64) -> BigInt {
    let p = pow10(shift.unsigned_abs());
    if shift >= 0 {
        round_half_even_div(&(&r.num * p), &r.den)
    } else {
        round_half_even_div(&r.num, &(&r.den * p))
    }
}

/// Digit-generation modes of `_Py_dg_dtoa` that CPython uses.
#[derive(Clone, Copy)]
enum Dtoa {
    /// Mode 0: shortest digits that round-trip (`repr`).
    Shortest,
    /// Mode 2: `n` significant digits (`n <= 0` acts as 1).
    Significant(i64),
    /// Mode 3: digits through the `10^-n` place (`n` may be negative).
    Fixed(i64),
}

/// `_Py_dg_dtoa(a, mode, ndigits)` for finite `a >= 0`: the digits with
/// trailing zeros stripped and the decimal point position `decpt`
/// (value `0.d1d2... * 10^decpt`). Ties round half-even on the exact
/// binary value, as dtoa does. Zero gives `("0", 1)`; a mode-3 result that
/// rounds to zero gives `("", -n)`.
fn dtoa(a: f64, mode: Dtoa) -> (String, i64) {
    if a == 0.0 {
        return ("0".to_string(), 1);
    }
    let strip = |s: String| -> String {
        let t = s.trim_end_matches('0');
        t.to_string()
    };
    match mode {
        Dtoa::Shortest => {
            let s = format!("{:e}", a);
            let (mant, exp) = s.split_once('e').expect("`{:e}` output has an exponent");
            let digits: String = mant.chars().filter(|c| *c != '.').collect();
            let exp: i64 = exp.parse().expect("`{:e}` exponent is an integer");
            let (digits, decpt) = shortest_tie_even(a, strip(digits), exp + 1);
            (strip(digits), decpt)
        }
        Dtoa::Significant(n) => {
            // A double has at most 767 significant decimal digits, so more
            // requested digits only add zeros, which dtoa strips anyway.
            let n = n.clamp(1, 800);
            let r = Rat::from_f64(a);
            let mut k = exact_exp10(a);
            let mut q = scaled_round(&r, n - 1 - k);
            if q == pow10(n as u64) {
                q = pow10(n as u64 - 1);
                k += 1;
            }
            (strip(q.to_string()), k + 1)
        }
        Dtoa::Fixed(n) => {
            // Past 1074 decimals every double is exact; below -330 every
            // double rounds to zero.
            if n < -330 {
                return (String::new(), -n);
            }
            let n = n.min(1100);
            let r = Rat::from_f64(a);
            let q = scaled_round(&r, n);
            if q.is_zero() {
                return (String::new(), -n);
            }
            let s = q.to_string();
            let decpt = s.len() as i64 - n;
            (strip(s), decpt)
        }
    }
}

/// Rust's shortest formatter breaks an exact tie between two equally short,
/// equally close digit strings upward; dtoa mode 0 picks the even one
/// (`repr(1000000000000000.25) == '1000000000000000.2'`). Given Rust's
/// digits `d` (value `0.d * 10^decpt`), returns the neighbour of the same
/// length that dtoa would choose.
fn shortest_tie_even(a: f64, d: String, decpt: i64) -> (String, i64) {
    let n = d.len() as i64;
    let scale = decpt - n;
    let exact = Rat::from_f64(a);
    let value = |q: &BigInt| -> Rat {
        let p = pow10(scale.unsigned_abs());
        if scale >= 0 {
            Rat::from_bigint(q * p)
        } else {
            Rat { num: q.clone(), den: p }
        }
    };
    // |q*10^scale - a| as a rational (shared denominator for comparison).
    let dist = |q: &BigInt| -> Rat {
        let v = value(q);
        let num = (&v.num * &exact.den - &exact.num * &v.den).magnitude().clone();
        Rat { num: BigInt::from(num), den: &v.den * &exact.den }
    };
    let q: BigInt = d.parse().expect("decimal digits");
    if q.is_even() {
        return (d, decpt);
    }
    let dq = dist(&q);
    for alt in [&q - 1u32, &q + 1u32] {
        if alt.to_string().len() as i64 != n || dist(&alt) != dq {
            continue;
        }
        let text = format!("{}e{}", alt, scale);
        if text.parse::<f64>().ok() == Some(a) {
            return (alt.to_string(), decpt);
        }
    }
    (d, decpt)
}

/// `format_float_short` (`Python/pystrtod.c`): lays out the dtoa digits for
/// format code `e`, `f`, `g` or `r`.
#[allow(clippy::too_many_arguments)]
fn format_float_short(
    d: f64,
    code: char,
    mode: Dtoa,
    precision: i64,
    always_add_sign: bool,
    add_dot_0_if_integer: bool,
    use_alt_formatting: bool,
    upper: bool,
) -> String {
    let mut out = String::new();
    if !d.is_finite() {
        // The sign of a nan is ignored.
        if d.is_infinite() && d.is_sign_negative() {
            out.push('-');
        } else if always_add_sign {
            out.push('+');
        }
        out.push_str(match (d.is_nan(), upper) {
            (false, false) => "inf",
            (false, true) => "INF",
            (true, false) => "nan",
            (true, true) => "NAN",
        });
        return out;
    }
    let (digits, decpt) = dtoa(libm::fabs(d), mode);
    let digits = digits.as_bytes();
    let digits_len = digits.len() as i64;
    let mut decpt = decpt;
    let mut vdigits_end = digits_len;
    let mut use_exp = false;
    let mut exp = 0i64;
    match code {
        'e' => {
            use_exp = true;
            vdigits_end = precision;
        }
        'f' => vdigits_end = decpt + precision,
        'g' => {
            let limit = if add_dot_0_if_integer { precision - 1 } else { precision };
            if decpt <= -4 || decpt > limit {
                use_exp = true;
            }
            if use_alt_formatting {
                vdigits_end = precision;
            }
        }
        'r' => {
            if decpt <= -4 || decpt > 16 {
                use_exp = true;
            }
        }
        _ => unreachable!("format_float_short code {code}"),
    }
    if use_exp {
        exp = decpt - 1;
        decpt = 1;
    }
    let vdigits_start = if decpt <= 0 { decpt - 1 } else { 0 };
    vdigits_end = if !use_exp && add_dot_0_if_integer {
        vdigits_end.max(decpt + 1)
    } else {
        vdigits_end.max(decpt)
    };
    let zeros = |out: &mut String, n: i64| {
        for _ in 0..n.max(0) {
            out.push('0');
        }
    };
    if d.is_sign_negative() {
        out.push('-');
    } else if always_add_sign {
        out.push('+');
    }
    if decpt <= 0 {
        zeros(&mut out, decpt - vdigits_start);
        out.push('.');
        zeros(&mut out, -decpt);
    } else {
        zeros(&mut out, -vdigits_start);
    }
    let text = |b: &[u8]| String::from_utf8(b.to_vec()).expect("ASCII digits");
    if 0 < decpt && decpt <= digits_len {
        out.push_str(&text(&digits[..decpt as usize]));
        out.push('.');
        out.push_str(&text(&digits[decpt as usize..]));
    } else {
        out.push_str(&text(digits));
    }
    if digits_len < decpt {
        zeros(&mut out, decpt - digits_len);
        out.push('.');
        zeros(&mut out, vdigits_end - decpt);
    } else {
        zeros(&mut out, vdigits_end - digits_len);
    }
    if out.ends_with('.') && !use_alt_formatting {
        out.pop();
    }
    if use_exp {
        out.push(if upper { 'E' } else { 'e' });
        out.push(if exp < 0 { '-' } else { '+' });
        out.push_str(&format!("{:02}", exp.abs()));
    }
    out
}

/// `PyOS_double_to_string(val, format_code, precision, flags)`
/// (`Python/pystrtod.c`) for codes `e E f F g G r`.
fn double_to_string(val: f64, code: char, precision: i64, sign: bool, add_dot_0: bool, alt: bool) -> String {
    let upper = code.is_ascii_uppercase();
    let (code, mode, precision) = match code {
        'e' | 'E' => ('e', Dtoa::Significant(precision + 1), precision + 1),
        'f' | 'F' => ('f', Dtoa::Fixed(precision), precision),
        'g' | 'G' => {
            let p = if precision == 0 { 1 } else { precision };
            ('g', Dtoa::Significant(p), p)
        }
        'r' => ('r', Dtoa::Shortest, 0),
        _ => unreachable!("PyOS_double_to_string code {code}"),
    };
    format_float_short(val, code, mode, precision, sign, add_dot_0, alt, upper)
}

// ---------------------------------------------------------------------------
// Float repr / str / hash.
// ---------------------------------------------------------------------------

/// CPython `float.__repr__` (`PyOS_double_to_string(x, 'r', 0,
/// Py_DTSF_ADD_DOT_0)`): shortest round-trip digits, exponent notation when
/// the decimal point position is `<= -4` or `> 16` (`1e+16`, `1e-05`,
/// `1000000000000000.0`, `0.0001`), `inf`, `nan`, `-0.0`.
pub fn float_repr(f: f64) -> String {
    double_to_string(f, 'r', 0, false, true, false)
}

/// CPython `str(float)` equals `repr(float)` in 3.8.
pub fn float_str(f: f64) -> String {
    float_repr(f)
}

/// CPython float hash (`Python/pyhash.c:_Py_HashDouble`).
/// Used so `1 == 1.0 == True` map to the same key hash.
pub fn float_py_hash(f: f64) -> i64 {
    if f.is_nan() {
        0
    } else if f.is_infinite() {
        if f > 0.0 {
            314159
        } else {
            -314159
        }
    } else if f == 0.0 {
        0
    } else {
        // Exact rational: f = p/q with q a power of two, hash = p * inv(q)
        // mod (2**61 - 1), matching integral floats with `int` hashes.
        let negative = f < 0.0;
        let r = Rat::from_f64(libm::fabs(f));
        const MOD: u64 = (1u64 << 61) - 1;
        let mod_b = BigInt::from(MOD);
        let p_mod = (&r.num % &mod_b).to_u64().unwrap_or(0);
        let den_mod = (&r.den % &mod_b).to_u64().unwrap_or(0);
        let inv = modpow_u(den_mod, MOD - 2, MOD);
        let h = ((p_mod as u128 * inv as u128) % MOD as u128) as u64 as i64;
        let h = if negative { -h } else { h };
        if h == -1 {
            -2
        } else {
            h
        }
    }
}

fn modpow_u(base: u64, exp: u64, modulus: u64) -> u64 {
    let mut r: u128 = 1;
    let mut b = (base % modulus) as u128;
    let mut e = exp;
    while e > 0 {
        if e & 1 == 1 {
            r = r * b % modulus as u128;
        }
        b = b * b % modulus as u128;
        e >>= 1;
    }
    r as u64
}

/// Decimal-exact fixed-point text, C `"%.{prec}f"`: correctly rounded
/// half-even from the exact binary value (`inf`/`nan` as `%f` prints them).
pub fn format_f_fixed(f: f64, prec: usize, upper: bool) -> String {
    double_to_string(f, if upper { 'F' } else { 'f' }, prec as i64, false, false, false)
}

// ---------------------------------------------------------------------------
// Scalar values, repr and conversions.
// ---------------------------------------------------------------------------

/// Scalar value for `%` formatting, `format()` and `str.format`: mirrors the
/// CPython types klippy passes.
#[derive(Clone, Debug)]
pub enum PctValue {
    /// `str`
    Str(String),
    /// `bool`
    Bool(bool),
    /// `int`
    Int(PyInt),
    /// `float`
    Float(f64),
    /// `bytes`
    Bytes(Vec<u8>),
    /// `None`
    None,
}

impl PctValue {
    /// Convert to the `%s` text (CPython `str()`).
    pub fn as_str(&self) -> String {
        match self {
            PctValue::Str(s) => s.clone(),
            _ => self.as_repr(),
        }
    }
    /// Convert to the `%r` text (CPython `repr()`).
    pub fn as_repr(&self) -> String {
        match self {
            PctValue::Str(s) => str_repr(s),
            PctValue::Bool(b) => if *b { "True" } else { "False" }.to_string(),
            PctValue::Int(n) => n.0.to_string(),
            PctValue::Float(f) => float_repr(*f),
            PctValue::Bytes(b) => bytes_repr(b),
            PctValue::None => "None".to_string(),
        }
    }
}

fn type_name(v: &PctValue) -> &'static str {
    match v {
        PctValue::Str(_) => "str",
        PctValue::Bool(_) => "bool",
        PctValue::Int(_) => "int",
        PctValue::Float(_) => "float",
        PctValue::Bytes(_) => "bytes",
        PctValue::None => "NoneType",
    }
}

/// CPython 3.8 `str.__repr__` (`Objects/unicodeobject.c:unicode_repr`):
/// single quotes unless the text has `'` but no `"`; escapes `\\`, the
/// quote, `\t \n \r`, other ASCII controls and DEL as `\xNN`, and
/// non-printable non-ASCII characters as `\xNN`, `\uNNNN` or `\UNNNNNNNN`.
pub fn str_repr(s: &str) -> String {
    quote_string(s, false)
}

/// CPython `ascii(str)`: `repr` with every non-ASCII character escaped.
pub fn str_repr_ascii(s: &str) -> String {
    quote_string(s, true)
}

/// CPython `bytes.__repr__` (`Objects/bytesobject.c:PyBytes_Repr`).
pub fn bytes_repr(b: &[u8]) -> String {
    let has_single = b.contains(&b'\'');
    let has_double = b.contains(&b'"');
    let quote = if has_single && !has_double { b'"' } else { b'\'' };
    let mut out = String::from("b");
    out.push(quote as char);
    for &c in b {
        if c == quote || c == b'\\' {
            out.push('\\');
            out.push(c as char);
        } else if c == b'\t' {
            out.push_str("\\t");
        } else if c == b'\n' {
            out.push_str("\\n");
        } else if c == b'\r' {
            out.push_str("\\r");
        } else if !(0x20..0x7f).contains(&c) {
            out.push_str(&format!("\\x{:02x}", c));
        } else {
            out.push(c as char);
        }
    }
    out.push(quote as char);
    out
}

fn push_code_escape(out: &mut String, c: char) {
    let o = c as u32;
    if o <= 0xff {
        out.push_str(&format!("\\x{:02x}", o));
    } else if o <= 0xffff {
        out.push_str(&format!("\\u{:04x}", o));
    } else {
        out.push_str(&format!("\\U{:08x}", o));
    }
}

fn quote_string(s: &str, ascii_only: bool) -> String {
    let has_single = s.contains('\'');
    let has_double = s.contains('"');
    let quote = if has_single && !has_double { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        if c == quote || c == '\\' {
            out.push('\\');
            out.push(c);
        } else if c == '\t' {
            out.push_str("\\t");
        } else if c == '\n' {
            out.push_str("\\n");
        } else if c == '\r' {
            out.push_str("\\r");
        } else if c < ' ' || c == '\x7f' {
            out.push_str(&format!("\\x{:02x}", c as u32));
        } else if c.is_ascii() || (unicode::is_printable(c) && !ascii_only) {
            out.push(c);
        } else {
            push_code_escape(&mut out, c);
        }
    }
    out.push(quote);
    out
}

/// `ascii(obj)` from `repr(obj)`: backslash-escape the non-ASCII characters.
fn ascii_of_repr(repr: &str) -> String {
    let mut out = String::with_capacity(repr.len());
    for c in repr.chars() {
        if c.is_ascii() {
            out.push(c);
        } else {
            push_code_escape(&mut out, c);
        }
    }
    out
}

/// Exact `int` to `float` (`Objects/longobject.c:PyLong_AsDouble`): rounds
/// half-even; `None` when the result overflows (`OverflowError: int too
/// large to convert to float`).
pub(crate) fn int_to_f64(n: &BigInt) -> Option<f64> {
    let neg = n.sign() == Sign::Minus;
    let mag = n.magnitude();
    let bits = mag.bits();
    let v = if bits <= 53 {
        mag.to_u64().expect("53-bit magnitude") as f64
    } else {
        let shift = bits - 53;
        let mut q = (mag >> shift).to_u64().expect("53-bit quotient");
        let rem = mag - (BigUint::from(q) << shift);
        let half = BigUint::one() << (shift - 1);
        if rem > half || (rem == half && q & 1 == 1) {
            q += 1;
        }
        // q <= 2^53 is exact; scale by 2^shift exactly (or overflow to inf).
        let e = shift as i64 + i64::from(q == 1u64 << 53);
        let q = if q == 1u64 << 53 { q >> 1 } else { q };
        if e + 53 > 1024 {
            return None;
        }
        let mut v = q as f64;
        let mut e = e;
        while e > 0 {
            let step = e.min(1000);
            v *= 2f64.powi(step as i32);
            e -= step;
        }
        v
    };
    if v.is_infinite() {
        return None;
    }
    Some(if neg { -v } else { v })
}

fn int_too_large_for_float() -> PyError {
    overflow_error("int too large to convert to float")
}

/// `int(float)` truncation toward zero with big-int overflow
/// (CPython `Objects/longobject.c:PyLong_FromDouble`). Callers handle
/// `inf`/`nan` (`OverflowError`/`ValueError`) first.
pub fn trunc_float_to_bigint(f: f64) -> PyInt {
    debug_assert!(f.is_finite());
    if f == 0.0 {
        return PyInt::from_i64(0);
    }
    let neg = f < 0.0;
    let a = libm::fabs(f);
    let bits = a.to_bits();
    let raw_exp = ((bits >> 52) & 0x7ff) as i32;
    let mant_bits = bits & 0x000f_ffff_ffff_ffff;
    if raw_exp == 0 {
        return PyInt::from_i64(0);
    }
    let e: i32 = raw_exp - 1023 - 52;
    let m_int: u64 = (1u64 << 52) | mant_bits;
    let val = if e >= 0 {
        BigInt::from(m_int) << (e as usize)
    } else if e < -64 {
        return PyInt::from_i64(0);
    } else {
        BigInt::from(m_int) >> ((-e) as usize)
    };
    if neg {
        PyInt(-val)
    } else {
        PyInt(val)
    }
}

/// `PyFloat_AsDouble(v)`: float value of a number (`int` rounds exactly
/// half-even; too large raises `OverflowError: int too large to convert to
/// float`); other types raise `TypeError: must be real number, not X`.
pub fn as_float(v: &PctValue) -> Result<f64, PyError> {
    match v {
        PctValue::Float(f) => Ok(*f),
        PctValue::Int(n) => int_to_f64(&n.0).ok_or_else(int_too_large_for_float),
        PctValue::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        _ => Err(type_error(format!("must be real number, not {}", type_name(v)))),
    }
}

/// `PyNumber_ToBase(n, base)` (with `alt`) / `_PyLong_Format`: sign, then
/// `0b`/`0o`/`0x` for non-decimal bases when `alt`, then lowercase digits.
fn long_to_base(n: &BigInt, base: u32, alt: bool) -> String {
    let mut out = String::new();
    if n.sign() == Sign::Minus {
        out.push('-');
    }
    if alt {
        match base {
            2 => out.push_str("0b"),
            8 => out.push_str("0o"),
            16 => out.push_str("0x"),
            _ => {}
        }
    }
    out.push_str(&n.magnitude().to_str_radix(base));
    out
}

fn chr(code: i64) -> Result<char, PyError> {
    if !(0..=0x10ffff).contains(&code) {
        return Err(overflow_error("%c arg not in range(0x110000)"));
    }
    char::from_u32(code as u32).ok_or_else(|| {
        value_error(format!("%c arg {:#x} is a surrogate, which pycompat strings cannot hold", code))
    })
}

// ---------------------------------------------------------------------------
// `%` formatting (`Objects/unicodeobject.c:PyUnicode_Format`).
// ---------------------------------------------------------------------------

const F_LJUST: u8 = 1;
const F_SIGN: u8 = 2;
const F_BLANK: u8 = 4;
const F_ALT: u8 = 8;
const F_ZERO: u8 = 16;

/// The object a `%` conversion consumes: a value, or the mapping itself
/// (`'%s' % {'a': 1}` formats the dict).
#[derive(Clone, Copy)]
enum PctArg<'a> {
    Val(&'a PctValue),
    Dict(&'a [(String, PctValue)]),
}

/// `dict` lookup over `(key, value)` pairs: the last pair wins, as when the
/// pairs build a dict.
fn map_get<'a>(map: &'a [(String, PctValue)], key: &str) -> Option<&'a PctValue> {
    map.iter().rev().find(|(k, _)| k == key).map(|(_, v)| v)
}

fn dict_repr(map: &[(String, PctValue)]) -> String {
    let mut keys: Vec<&str> = Vec::new();
    for (k, _) in map {
        if !keys.contains(&k.as_str()) {
            keys.push(k);
        }
    }
    let parts: Vec<String> = keys
        .iter()
        .map(|k| format!("{}: {}", str_repr(k), map_get(map, k).expect("key present").as_repr()))
        .collect();
    format!("{{{}}}", parts.join(", "))
}

impl PctArg<'_> {
    fn type_name(&self) -> &'static str {
        match self {
            PctArg::Val(v) => type_name(v),
            PctArg::Dict(_) => "dict",
        }
    }
    fn str(&self) -> String {
        match self {
            PctArg::Val(v) => v.as_str(),
            PctArg::Dict(m) => dict_repr(m),
        }
    }
    fn repr(&self) -> String {
        match self {
            PctArg::Val(v) => v.as_repr(),
            PctArg::Dict(m) => dict_repr(m),
        }
    }
    /// `PyLong_Check`: the value as an int (`bool` included).
    fn as_long(&self) -> Option<BigInt> {
        match self {
            PctArg::Val(PctValue::Int(n)) => Some(n.0.clone()),
            PctArg::Val(PctValue::Bool(b)) => Some(BigInt::from(*b as u8)),
            _ => None,
        }
    }
}

struct PctSpecArg {
    ch: char,
    flags: u8,
    width: isize,
    prec: i32,
    sign: bool,
}

/// Formatted text of one conversion: written as is (CPython fast paths) or
/// passed through `unicode_format_arg_output` for width/sign handling.
enum PctOut {
    Direct(String),
    Padded(String),
}

struct PctFormatter<'a> {
    fmt: Vec<char>,
    pos: usize,
    cnt: isize,
    tuple: &'a [PctValue],
    dict: Option<&'a [(String, PctValue)]>,
    args: PctArg<'a>,
    arglen: isize,
    argidx: isize,
    out: String,
}

impl<'a> PctFormatter<'a> {
    /// `PyUnicode_READ` at `i`; the terminating NUL past the end.
    fn read(&self, i: usize) -> char {
        self.fmt.get(i).copied().unwrap_or('\0')
    }

    fn next_arg(&mut self) -> Result<PctArg<'a>, PyError> {
        if self.argidx < self.arglen {
            let i = self.argidx;
            self.argidx += 1;
            return Ok(if self.arglen < 0 { self.args } else { PctArg::Val(&self.tuple[i as usize]) });
        }
        Err(type_error("not enough arguments for format string"))
    }

    fn run(mut self) -> Result<String, PyError> {
        self.cnt = self.fmt.len() as isize;
        loop {
            self.cnt -= 1;
            if self.cnt < 0 {
                break;
            }
            if self.read(self.pos) != '%' {
                let start = self.pos;
                self.pos += 1;
                while self.cnt >= 0 && self.read(self.pos) != '%' {
                    self.pos += 1;
                    self.cnt -= 1;
                }
                if self.cnt < 0 {
                    self.pos -= 1;
                }
                self.out.extend(&self.fmt[start..self.pos]);
            } else {
                self.pos += 1;
                self.format_arg()?;
            }
        }
        if self.argidx < self.arglen && self.dict.is_none() {
            return Err(type_error("not all arguments converted during string formatting"));
        }
        Ok(self.out)
    }

    /// `unicode_format_arg`.
    fn format_arg(&mut self) -> Result<(), PyError> {
        let ch = self.read(self.pos);
        if ch == '%' {
            self.pos += 1;
            self.cnt -= 1;
            self.out.push('%');
            return Ok(());
        }
        let mut arg = PctSpecArg { ch, flags: 0, width: -1, prec: -1, sign: false };
        self.parse_arg(&mut arg)?;
        match self.format_one(&mut arg)? {
            PctOut::Direct(s) => self.out.push_str(&s),
            PctOut::Padded(s) => self.output(&mut arg, &s),
        }
        if self.dict.is_some() && self.argidx < self.arglen {
            return Err(type_error("not all arguments converted during string formatting"));
        }
        Ok(())
    }

    /// `unicode_format_arg_parse`: mapping key, flags, width, precision,
    /// length modifier.
    fn parse_arg(&mut self, arg: &mut PctSpecArg) -> Result<(), PyError> {
        if arg.ch == '(' {
            let Some(map) = self.dict else {
                return Err(type_error("format requires a mapping"));
            };
            self.pos += 1;
            self.cnt -= 1;
            let keystart = self.pos;
            let mut pcount = 1;
            while pcount > 0 {
                self.cnt -= 1;
                if self.cnt < 0 {
                    break;
                }
                arg.ch = self.read(self.pos);
                if arg.ch == ')' {
                    pcount -= 1;
                } else if arg.ch == '(' {
                    pcount += 1;
                }
                self.pos += 1;
            }
            if self.cnt < 0 || pcount > 0 {
                return Err(value_error("incomplete format key"));
            }
            let key: String = self.fmt[keystart..self.pos - 1].iter().collect();
            let v = map_get(map, &key).ok_or_else(|| PyError::new("KeyError", str_repr(&key)))?;
            self.args = PctArg::Val(v);
            self.arglen = -1;
            self.argidx = -2;
        }
        loop {
            self.cnt -= 1;
            if self.cnt < 0 {
                break;
            }
            arg.ch = self.read(self.pos);
            self.pos += 1;
            arg.flags |= match arg.ch {
                '-' => F_LJUST,
                '+' => F_SIGN,
                ' ' => F_BLANK,
                '#' => F_ALT,
                '0' => F_ZERO,
                _ => break,
            };
        }
        if arg.ch == '*' {
            let v = self.next_arg()?;
            let n = v.as_long().ok_or_else(|| type_error("* wants int"))?;
            arg.width = n
                .to_isize()
                .ok_or_else(|| overflow_error("Python int too large to convert to C ssize_t"))?;
            if arg.width < 0 {
                arg.flags |= F_LJUST;
                arg.width = -arg.width;
            }
            self.cnt -= 1;
            if self.cnt >= 0 {
                arg.ch = self.read(self.pos);
                self.pos += 1;
            }
        } else if arg.ch.is_ascii_digit() {
            arg.width = (arg.ch as u8 - b'0') as isize;
            loop {
                self.cnt -= 1;
                if self.cnt < 0 {
                    break;
                }
                arg.ch = self.read(self.pos);
                self.pos += 1;
                if !arg.ch.is_ascii_digit() {
                    break;
                }
                let d = (arg.ch as u8 - b'0') as isize;
                if arg.width > (isize::MAX - d) / 10 {
                    return Err(value_error("width too big"));
                }
                arg.width = arg.width * 10 + d;
            }
        }
        if arg.ch == '.' {
            arg.prec = 0;
            self.cnt -= 1;
            if self.cnt >= 0 {
                arg.ch = self.read(self.pos);
                self.pos += 1;
            }
            if arg.ch == '*' {
                let v = self.next_arg()?;
                let n = v.as_long().ok_or_else(|| type_error("* wants int"))?;
                arg.prec = n
                    .to_i32()
                    .ok_or_else(|| overflow_error("Python int too large to convert to C int"))?;
                if arg.prec < 0 {
                    arg.prec = 0;
                }
                self.cnt -= 1;
                if self.cnt >= 0 {
                    arg.ch = self.read(self.pos);
                    self.pos += 1;
                }
            } else if arg.ch.is_ascii_digit() {
                arg.prec = (arg.ch as u8 - b'0') as i32;
                loop {
                    self.cnt -= 1;
                    if self.cnt < 0 {
                        break;
                    }
                    arg.ch = self.read(self.pos);
                    self.pos += 1;
                    if !arg.ch.is_ascii_digit() {
                        break;
                    }
                    let d = (arg.ch as u8 - b'0') as i32;
                    if arg.prec > (i32::MAX - d) / 10 {
                        return Err(value_error("precision too big"));
                    }
                    arg.prec = arg.prec * 10 + d;
                }
            }
        }
        if self.cnt >= 0 && matches!(arg.ch, 'h' | 'l' | 'L') {
            self.cnt -= 1;
            if self.cnt >= 0 {
                arg.ch = self.read(self.pos);
                self.pos += 1;
            }
        }
        if self.cnt < 0 {
            return Err(value_error("incomplete format"));
        }
        Ok(())
    }

    /// `unicode_format_arg_format`.
    fn format_one(&mut self, arg: &mut PctSpecArg) -> Result<PctOut, PyError> {
        let v = self.next_arg()?;
        let exact_int = matches!(v, PctArg::Val(PctValue::Int(_)));
        match arg.ch {
            's' | 'r' | 'a' => {
                if exact_int && arg.width == -1 && arg.prec == -1 {
                    return Ok(PctOut::Direct(v.str()));
                }
                Ok(PctOut::Padded(match arg.ch {
                    's' => v.str(),
                    'r' => v.repr(),
                    _ => ascii_of_repr(&v.repr()),
                }))
            }
            'i' | 'd' | 'u' | 'o' | 'x' | 'X' => {
                let out = mainformatlong(v, arg)?;
                if let PctOut::Padded(_) = out {
                    arg.sign = true;
                }
                Ok(out)
            }
            'e' | 'E' | 'f' | 'F' | 'g' | 'G' => {
                let x = match v {
                    PctArg::Val(v) => as_float(v)?,
                    PctArg::Dict(_) => return Err(type_error("must be real number, not dict")),
                };
                let prec = if arg.prec < 0 { 6 } else { arg.prec as i64 };
                let s = double_to_string(x, arg.ch, prec, false, false, arg.flags & F_ALT != 0);
                if arg.width == -1 && arg.prec == -1 && arg.flags & (F_SIGN | F_BLANK) == 0 {
                    return Ok(PctOut::Direct(s));
                }
                arg.sign = true;
                Ok(PctOut::Padded(s))
            }
            'c' => {
                let c = formatchar(v)?;
                if arg.width == -1 && arg.prec == -1 {
                    return Ok(PctOut::Direct(c.to_string()));
                }
                Ok(PctOut::Padded(c.to_string()))
            }
            ch => Err(value_error(format!(
                "unsupported format character '{}' ({:#x}) at index {}",
                if (' '..='~').contains(&ch) || ch == '\x1f' { ch } else { '?' },
                ch as u32,
                self.pos - 1
            ))),
        }
    }

    /// `unicode_format_arg_output`: sign, `0x`/`0o` prefix, fill and width.
    fn output(&mut self, arg: &mut PctSpecArg, s: &str) {
        let chars: Vec<char> = s.chars().collect();
        let fill = if arg.sign && arg.flags & F_ZERO != 0 { '0' } else { ' ' };
        let mut len = chars.len() as isize;
        if (arg.width == -1 || arg.width <= len)
            && (arg.prec == -1 || arg.prec as isize >= len)
            && arg.flags & (F_SIGN | F_BLANK) == 0
        {
            self.out.push_str(s);
            return;
        }
        if matches!(arg.ch, 's' | 'r' | 'a') && arg.prec >= 0 && len > arg.prec as isize {
            len = arg.prec as isize;
        }
        let mut pindex = 0usize;
        let mut signchar = '\0';
        if arg.sign {
            let c0 = chars[0];
            if c0 == '-' || c0 == '+' {
                signchar = c0;
                len -= 1;
                pindex += 1;
            } else if arg.flags & F_SIGN != 0 {
                signchar = '+';
            } else if arg.flags & F_BLANK != 0 {
                signchar = ' ';
            } else {
                arg.sign = false;
            }
        }
        if arg.width < len {
            arg.width = len;
        }
        if arg.sign {
            if fill != ' ' {
                self.out.push(signchar);
            }
            if arg.width > len {
                arg.width -= 1;
            }
        }
        let alt_prefix = arg.flags & F_ALT != 0 && matches!(arg.ch, 'x' | 'X' | 'o');
        if alt_prefix {
            if fill != ' ' {
                self.out.push('0');
                self.out.push(arg.ch);
                pindex += 2;
            }
            arg.width -= 2;
            if arg.width < 0 {
                arg.width = 0;
            }
            len -= 2;
        }
        if arg.width > len && arg.flags & F_LJUST == 0 {
            for _ in 0..arg.width - len {
                self.out.push(fill);
            }
            arg.width = len;
        }
        if fill == ' ' {
            if arg.sign {
                self.out.push(signchar);
            }
            if alt_prefix {
                self.out.push('0');
                self.out.push(arg.ch);
                pindex += 2;
            }
        }
        if len > 0 {
            self.out.extend(&chars[pindex..pindex + len as usize]);
        }
        if arg.width > len {
            for _ in 0..arg.width - len {
                self.out.push(' ');
            }
        }
    }
}

/// `mainformatlong`: `%d %i %u %o %x %X` on ints and floats.
fn mainformatlong(v: PctArg<'_>, arg: &PctSpecArg) -> Result<PctOut, PyError> {
    let ty = arg.ch;
    let wrongtype = || {
        let what = if matches!(ty, 'o' | 'x' | 'X') { "an integer" } else { "a number" };
        type_error(format!("%{} format: {} is required, not {}", ty, what, v.type_name()))
    };
    let iobj = match v {
        PctArg::Val(PctValue::Int(n)) => n.0.clone(),
        PctArg::Val(PctValue::Bool(b)) => BigInt::from(*b as u8),
        PctArg::Val(PctValue::Float(f)) => {
            if matches!(ty, 'o' | 'x' | 'X') {
                return Err(wrongtype());
            }
            if f.is_nan() {
                return Err(value_error("cannot convert float NaN to integer"));
            }
            if f.is_infinite() {
                return Err(overflow_error("cannot convert float infinity to integer"));
            }
            trunc_float_to_bigint(*f).0
        }
        _ => return Err(wrongtype()),
    };
    let base = match ty {
        'o' => 8,
        'x' | 'X' => 16,
        _ => 10,
    };
    let alt = arg.flags & F_ALT != 0;
    if matches!(v, PctArg::Val(PctValue::Int(_)))
        && arg.width == -1
        && arg.prec == -1
        && arg.flags & (F_SIGN | F_BLANK) == 0
        && ty != 'X'
    {
        return Ok(PctOut::Direct(long_to_base(&iobj, base, alt)));
    }
    Ok(PctOut::Padded(format_long(&iobj, alt, arg.prec, ty)?))
}

/// `_PyUnicode_FormatLong(val, alt, prec, type)`.
fn format_long(val: &BigInt, alt: bool, prec: i32, ty: char) -> Result<String, PyError> {
    if prec > i32::MAX - 3 {
        return Err(overflow_error("precision too large"));
    }
    let base = match ty {
        'o' => 8,
        'x' | 'X' => 16,
        _ => 10,
    };
    let full = long_to_base(val, base, true);
    let sign = val.sign() == Sign::Minus;
    let mut nondigits: String = if sign { "-".to_string() } else { String::new() };
    let mut digits = &full[sign as usize..];
    if base != 10 {
        if alt {
            nondigits.push_str(&digits[..2]);
        }
        digits = &digits[2..];
    }
    let numdigits = digits.len() as i64;
    let mut out = nondigits;
    if prec as i64 > numdigits {
        for _ in 0..prec as i64 - numdigits {
            out.push('0');
        }
    }
    out.push_str(digits);
    if ty == 'X' {
        out = out.chars().map(|c| if ('a'..='x').contains(&c) { c.to_ascii_uppercase() } else { c }).collect();
    }
    Ok(out)
}

/// `formatchar`: `%c` from a one-character str or an int code point.
fn formatchar(v: PctArg<'_>) -> Result<char, PyError> {
    let err = || type_error("%c requires int or char");
    match v {
        PctArg::Val(PctValue::Str(s)) => {
            let mut it = s.chars();
            match (it.next(), it.next()) {
                (Some(c), None) => Ok(c),
                _ => Err(err()),
            }
        }
        _ => {
            let n = v.as_long().ok_or_else(err)?;
            // `PyLong_AsLong` overflow is replaced by the `%c` TypeError.
            let x = n.to_i64().ok_or_else(err)?;
            chr(x)
        }
    }
}

/// `fmt % values` with a tuple of values (CPython `str.__mod__`,
/// `Objects/unicodeobject.c:PyUnicode_Format`). Errors carry the CPython
/// exception class and text (`TypeError: not enough arguments for format
/// string`, `ValueError: unsupported format character 'y' (0x79) at index
/// 1`, ...).
pub fn percent_format(fmt: &str, args: &[PctValue]) -> Result<String, PyError> {
    PctFormatter {
        fmt: fmt.chars().collect(),
        pos: 0,
        cnt: 0,
        tuple: args,
        dict: None,
        args: PctArg::Dict(&[]),
        arglen: args.len() as isize,
        argidx: 0,
        out: String::new(),
    }
    .run()
}

/// `fmt % {name: value}` (CPython `str.__mod__` with a dict). Keys repeat
/// as in a dict built from the pairs (the last one wins); conversions
/// without a `(key)` format the dict itself.
pub fn percent_format_map(fmt: &str, map: &[(String, PctValue)]) -> Result<String, PyError> {
    PctFormatter {
        fmt: fmt.chars().collect(),
        pos: 0,
        cnt: 0,
        tuple: &[],
        dict: Some(map),
        args: PctArg::Dict(map),
        arglen: -1,
        argidx: -2,
        out: String::new(),
    }
    .run()
}

// ---------------------------------------------------------------------------
// `format()` mini-language (`Python/formatter_unicode.c`).
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum Thousands {
    No,
    Comma,
    Underscore,
    UnderFour,
}

struct FormatSpec {
    fill: char,
    align: char,
    alternate: bool,
    sign: char,
    width: isize,
    thousands: Thousands,
    precision: isize,
    ty: char,
}

/// `get_integer`: decimal digits (any Unicode `Nd`) at `*pos`.
fn spec_integer(spec: &[char], pos: &mut usize) -> Result<(isize, usize), PyError> {
    let mut acc: isize = 0;
    let mut n = 0;
    while let Some(d) = spec.get(*pos).and_then(|&c| unicode::to_decimal(c)) {
        let d = d as isize;
        if acc > (isize::MAX - d) / 10 {
            return Err(value_error("Too many decimal digits in format string"));
        }
        acc = acc * 10 + d;
        *pos += 1;
        n += 1;
    }
    Ok((acc, n))
}

fn code_text(c: char) -> String {
    if (c as u32) > 32 && (c as u32) < 128 {
        c.to_string()
    } else {
        format!("\\x{:x}", c as u32)
    }
}

/// `parse_internal_render_format_spec`.
fn parse_format_spec(spec: &[char], default_type: char, default_align: char) -> Result<FormatSpec, PyError> {
    let is_align = |c: char| matches!(c, '<' | '>' | '=' | '^');
    let end = spec.len();
    let mut pos = 0;
    let mut f = FormatSpec {
        fill: ' ',
        align: default_align,
        alternate: false,
        sign: '\0',
        width: -1,
        thousands: Thousands::No,
        precision: -1,
        ty: default_type,
    };
    let mut align_specified = false;
    let mut fill_specified = false;
    if end - pos >= 2 && is_align(spec[pos + 1]) {
        f.align = spec[pos + 1];
        f.fill = spec[pos];
        fill_specified = true;
        align_specified = true;
        pos += 2;
    } else if end - pos >= 1 && is_align(spec[pos]) {
        f.align = spec[pos];
        align_specified = true;
        pos += 1;
    }
    if pos < end && matches!(spec[pos], ' ' | '+' | '-') {
        f.sign = spec[pos];
        pos += 1;
    }
    if pos < end && spec[pos] == '#' {
        f.alternate = true;
        pos += 1;
    }
    if !fill_specified && pos < end && spec[pos] == '0' {
        f.fill = '0';
        if !align_specified {
            f.align = '=';
        }
        pos += 1;
    }
    let (width, consumed) = spec_integer(spec, &mut pos)?;
    if consumed > 0 {
        f.width = width;
    }
    let both = || value_error("Cannot specify both ',' and '_'.");
    if pos < end && spec[pos] == ',' {
        f.thousands = Thousands::Comma;
        pos += 1;
    }
    if pos < end && spec[pos] == '_' {
        if f.thousands != Thousands::No {
            return Err(both());
        }
        f.thousands = Thousands::Underscore;
        pos += 1;
    }
    if pos < end && spec[pos] == ',' {
        return Err(both());
    }
    if pos < end && spec[pos] == '.' {
        pos += 1;
        let (precision, consumed) = spec_integer(spec, &mut pos)?;
        if consumed == 0 {
            return Err(value_error("Format specifier missing precision"));
        }
        f.precision = precision;
    }
    if end - pos > 1 {
        return Err(value_error("Invalid format specifier"));
    }
    if end - pos == 1 {
        f.ty = spec[pos];
    }
    if f.thousands != Thousands::No {
        match f.ty {
            'd' | 'e' | 'f' | 'g' | 'E' | 'G' | '%' | 'F' | '\0' => {}
            'b' | 'o' | 'x' | 'X' if f.thousands == Thousands::Underscore => {
                f.thousands = Thousands::UnderFour;
            }
            ty => {
                let sep = if f.thousands == Thousands::Comma { ',' } else { '_' };
                return Err(value_error(format!("Cannot specify '{}' with '{}'.", sep, code_text(ty))));
            }
        }
    }
    Ok(f)
}

fn unknown_presentation_type(ty: char, type_name: &str) -> PyError {
    value_error(format!("Unknown format code '{}' for object of type '{}'", code_text(ty), type_name))
}

/// Separator and group size of `get_locale_info`. Type `n` uses the C
/// locale of the reference runtime (no grouping), like no separator.
fn locale_grouping(spec: &FormatSpec) -> (&'static str, Option<usize>) {
    if spec.ty == 'n' {
        return ("", None);
    }
    match spec.thousands {
        Thousands::No => ("", None),
        Thousands::Comma => (",", Some(3)),
        Thousands::Underscore => ("_", Some(3)),
        Thousands::UnderFour => ("_", Some(4)),
    }
}

/// `_PyUnicode_InsertThousandsGrouping`: `digits` grouped from the right,
/// zero-extended on the left to `min_width`.
fn insert_grouping(digits: &[char], min_width: isize, sep: &str, group: Option<usize>) -> Vec<char> {
    let sep: Vec<char> = sep.chars().collect();
    let sep_len = sep.len() as isize;
    let mut min_width = min_width.max(0);
    let mut remaining = digits.len() as isize;
    let mut digits_pos = digits.len();
    // Built right to left, reversed at the end.
    let mut rev: Vec<char> = Vec::new();
    let mut use_sep = false;
    let emit = |rev: &mut Vec<char>, n_chars: isize, n_zeros: isize, use_sep: bool, digits_pos: &mut usize| {
        if use_sep {
            rev.extend(sep.iter().rev());
        }
        let n = n_chars as usize;
        rev.extend(digits[*digits_pos - n..*digits_pos].iter().rev());
        *digits_pos -= n;
        for _ in 0..n_zeros {
            rev.push('0');
        }
    };
    let mut broken = false;
    if let Some(g) = group {
        loop {
            let len = (g as isize).min(remaining.max(min_width).max(1));
            let n_zeros = (len - remaining).max(0);
            let n_chars = remaining.min(len).max(0);
            emit(&mut rev, n_chars, n_zeros, use_sep, &mut digits_pos);
            use_sep = true;
            remaining -= n_chars;
            min_width -= len;
            if remaining <= 0 && min_width <= 0 {
                broken = true;
                break;
            }
            min_width -= sep_len;
        }
    }
    if !broken {
        let len = remaining.max(min_width).max(1);
        let n_zeros = (len - remaining).max(0);
        let n_chars = remaining.min(len).max(0);
        emit(&mut rev, n_chars, n_zeros, use_sep, &mut digits_pos);
    }
    rev.reverse();
    rev
}

/// `calc_number_widths` + `fill_number`: lays out
/// `<lpad><sign><prefix><spad><grouped digits><decimal><remainder><rpad>`.
/// `number[n_start..n_end]` holds the digits, the optional `.` and the
/// remainder (`n_remainder` chars at the end).
#[allow(clippy::too_many_arguments)]
fn render_number(
    number: &[char],
    prefix: &[char],
    sign_char: char,
    n_start: usize,
    n_end: usize,
    n_remainder: usize,
    has_decimal: bool,
    spec: &FormatSpec,
    toupper: bool,
) -> String {
    let (sep, group) = locale_grouping(spec);
    let n_digits = n_end - n_start - n_remainder - has_decimal as usize;
    let (n_sign, sign) = match spec.sign {
        '+' => (1, if sign_char == '-' { '-' } else { '+' }),
        ' ' => (1, if sign_char == '-' { '-' } else { ' ' }),
        _ if sign_char == '-' => (1, '-'),
        _ => (0, '\0'),
    };
    let n_non_digit = (n_sign + prefix.len() + has_decimal as usize + n_remainder) as isize;
    let min_width = if spec.fill == '0' && spec.align == '=' { spec.width - n_non_digit } else { 0 };
    let digits = &number[n_start..n_start + n_digits];
    let mut grouped = if n_digits == 0 { Vec::new() } else { insert_grouping(digits, min_width, sep, group) };
    let n_padding = spec.width - (n_non_digit + grouped.len() as isize);
    let (lpad, spad, rpad) = if n_padding > 0 {
        match spec.align {
            '<' => (0, 0, n_padding),
            '^' => (n_padding / 2, 0, n_padding - n_padding / 2),
            '=' => (0, n_padding, 0),
            _ => (n_padding, 0, 0),
        }
    } else {
        (0, 0, 0)
    };
    let mut out = String::new();
    let pad = |out: &mut String, n: isize| {
        for _ in 0..n {
            out.push(spec.fill);
        }
    };
    pad(&mut out, lpad);
    if n_sign == 1 {
        out.push(sign);
    }
    for &c in prefix {
        out.push(if toupper { c.to_ascii_uppercase() } else { c });
    }
    pad(&mut out, spad);
    if toupper {
        grouped.iter_mut().for_each(|c| *c = c.to_ascii_uppercase());
    }
    out.extend(grouped);
    let mut d_pos = n_start + n_digits;
    if has_decimal {
        out.push('.');
        d_pos += 1;
    }
    out.extend(&number[d_pos..d_pos + n_remainder]);
    pad(&mut out, rpad);
    out
}

/// `format_string_internal`.
fn format_string_internal(s: &str, spec: &FormatSpec) -> Result<String, PyError> {
    if spec.sign != '\0' {
        return Err(value_error("Sign not allowed in string format specifier"));
    }
    if spec.alternate {
        return Err(value_error("Alternate form (#) not allowed in string format specifier"));
    }
    if spec.align == '=' {
        return Err(value_error("'=' alignment not allowed in string format specifier"));
    }
    let chars: Vec<char> = s.chars().collect();
    let mut len = chars.len() as isize;
    if (spec.width == -1 || spec.width <= len) && (spec.precision == -1 || spec.precision >= len) {
        return Ok(s.to_string());
    }
    if spec.precision >= 0 && len >= spec.precision {
        len = spec.precision;
    }
    let total = if spec.width >= 0 { len.max(spec.width) } else { len };
    let lpad = match spec.align {
        '>' => total - len,
        '^' => (total - len) / 2,
        _ => 0,
    };
    let rpad = total - len - lpad;
    let mut out = String::new();
    for _ in 0..lpad {
        out.push(spec.fill);
    }
    out.extend(&chars[..len as usize]);
    for _ in 0..rpad {
        out.push(spec.fill);
    }
    Ok(out)
}

/// `format_long_internal` for types `b c d o x X n`.
fn format_long_internal(n: &BigInt, spec: &FormatSpec) -> Result<String, PyError> {
    if spec.precision != -1 {
        return Err(value_error("Precision not allowed in integer format specifier"));
    }
    if spec.ty == 'c' {
        if spec.sign != '\0' {
            return Err(value_error("Sign not allowed with integer format specifier 'c'"));
        }
        if spec.alternate {
            return Err(value_error("Alternate form (#) not allowed with integer format specifier 'c'"));
        }
        let x = n.to_i64().ok_or_else(|| overflow_error("Python int too large to convert to C long"))?;
        let c = [chr(x)?];
        return Ok(render_number(&c, &[], '\0', 0, 1, 1, false, spec, false));
    }
    let (base, skip) = match spec.ty {
        'b' => (2, 2),
        'o' => (8, 2),
        'x' | 'X' => (16, 2),
        _ => (10, 0),
    };
    let tmp: Vec<char> = long_to_base(n, base, true).chars().collect();
    let mut prefix_at = 0;
    let mut skip = skip;
    let mut sign_char = '\0';
    if tmp[0] == '-' {
        sign_char = '-';
        prefix_at += 1;
        skip += 1;
    }
    let n_prefix = if spec.alternate { skip - prefix_at } else { 0 };
    let prefix = &tmp[prefix_at..prefix_at + n_prefix];
    Ok(render_number(&tmp, prefix, sign_char, skip, tmp.len(), 0, false, spec, spec.ty == 'X'))
}

/// `format_float_internal` for types `'' e E f F g G n %`.
fn format_float_internal(val: f64, spec: &FormatSpec) -> Result<String, PyError> {
    if spec.precision > i32::MAX as isize {
        return Err(value_error("precision too big"));
    }
    let mut precision = spec.precision as i64;
    let mut default_precision = 6;
    let mut ty = spec.ty;
    let mut add_dot_0 = false;
    if ty == '\0' {
        add_dot_0 = true;
        ty = 'r';
        default_precision = 0;
    }
    if ty == 'n' {
        ty = 'g';
    }
    let mut val = val;
    let mut add_pct = false;
    if ty == '%' {
        ty = 'f';
        val *= 100.0;
        add_pct = true;
    }
    if precision < 0 {
        precision = default_precision;
    } else if ty == 'r' {
        ty = 'g';
    }
    let mut buf = double_to_string(val, ty, precision, false, add_dot_0, spec.alternate);
    if add_pct {
        buf.push('%');
    }
    let chars: Vec<char> = buf.chars().collect();
    let mut index = 0;
    let mut sign_char = '\0';
    if chars[0] == '-' {
        sign_char = '-';
        index = 1;
    }
    // `parse_number`: integer digits, then an optional '.', then the rest.
    let end = chars.len();
    let mut p = index;
    while p < end && chars[p].is_ascii_digit() {
        p += 1;
    }
    let has_decimal = p < end && chars[p] == '.';
    let remainder_start = if has_decimal { p + 1 } else { p };
    Ok(render_number(&chars, &[], sign_char, index, end, end - remainder_start, has_decimal, spec, false))
}

/// `format(v, spec)` (`PyObject_Format` → `__format__`): `str`, `int`,
/// `bool` and `float` use the `formatter_unicode.c` mini-language
/// `[[fill]align][sign][#][0][width][,|_][.precision][type]`; `None` and
/// `bytes` use `object.__format__` (empty spec only).
pub fn format_value(v: &PctValue, spec: &str) -> Result<String, PyError> {
    if spec.is_empty() {
        return Ok(v.as_str());
    }
    let spec: Vec<char> = spec.chars().collect();
    match v {
        PctValue::Str(s) => {
            let f = parse_format_spec(&spec, 's', '<')?;
            match f.ty {
                's' => format_string_internal(s, &f),
                ty => Err(unknown_presentation_type(ty, "str")),
            }
        }
        PctValue::Int(_) | PctValue::Bool(_) => {
            let n = match v {
                PctValue::Int(n) => n.0.clone(),
                PctValue::Bool(b) => BigInt::from(*b as u8),
                _ => unreachable!(),
            };
            let f = parse_format_spec(&spec, 'd', '>')?;
            match f.ty {
                'b' | 'c' | 'd' | 'o' | 'x' | 'X' | 'n' => format_long_internal(&n, &f),
                'e' | 'E' | 'f' | 'F' | 'g' | 'G' | '%' => {
                    let x = int_to_f64(&n).ok_or_else(int_too_large_for_float)?;
                    format_float_internal(x, &f)
                }
                ty => Err(unknown_presentation_type(ty, type_name(v))),
            }
        }
        PctValue::Float(x) => {
            let f = parse_format_spec(&spec, '\0', '>')?;
            match f.ty {
                '\0' | 'e' | 'E' | 'f' | 'F' | 'g' | 'G' | 'n' | '%' => format_float_internal(*x, &f),
                ty => Err(unknown_presentation_type(ty, "float")),
            }
        }
        PctValue::None | PctValue::Bytes(_) => Err(type_error(format!(
            "unsupported format string passed to {}.__format__",
            type_name(v)
        ))),
    }
}

// ---------------------------------------------------------------------------
// `str.format` (`Objects/stringlib/unicode_format.h`).
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
enum AutoNumberState {
    Init,
    Auto,
    Manual,
}

struct AutoNumber {
    state: AutoNumberState,
    next: usize,
}

/// One `{field_name!conversion:format_spec}` replacement field.
struct Field<'t> {
    name: &'t [char],
    spec: &'t [char],
    spec_needs_expanding: bool,
    conversion: Option<char>,
}

/// `get_integer` of `unicode_format.h`: `None` for an empty or non-decimal
/// name.
fn field_integer(s: &[char]) -> Result<Option<usize>, PyError> {
    if s.is_empty() {
        return Ok(None);
    }
    let mut acc: usize = 0;
    for &c in s {
        let Some(d) = unicode::to_decimal(c) else {
            return Ok(None);
        };
        let d = d as usize;
        if acc > (isize::MAX as usize - d) / 10 {
            return Err(value_error("Too many decimal digits in format string"));
        }
        acc = acc * 10 + d;
    }
    Ok(Some(acc))
}

/// `parse_field`: reads the field after its `{`, through the closing `}`.
fn parse_field<'t>(input: &'t [char], pos: &mut usize) -> Result<Field<'t>, PyError> {
    let end = input.len();
    let name_start = *pos;
    let mut c = '\0';
    while *pos < end {
        c = input[*pos];
        *pos += 1;
        match c {
            '{' => return Err(value_error("unexpected '{' in field name")),
            '[' => {
                while *pos < end && input[*pos] != ']' {
                    *pos += 1;
                }
                continue;
            }
            '}' | ':' | '!' => break,
            _ => continue,
        }
    }
    let name = &input[name_start..*pos - 1];
    let mut field = Field { name, spec: &[], spec_needs_expanding: false, conversion: None };
    if c == '!' || c == ':' {
        if c == '!' {
            if *pos >= end {
                return Err(value_error("end of string while looking for conversion specifier"));
            }
            field.conversion = Some(input[*pos]);
            *pos += 1;
            if *pos < end {
                let c2 = input[*pos];
                *pos += 1;
                if c2 == '}' {
                    return Ok(field);
                }
                if c2 != ':' {
                    return Err(value_error("expected ':' after conversion specifier"));
                }
            }
        }
        let spec_start = *pos;
        let mut count = 1;
        while *pos < end {
            let c3 = input[*pos];
            *pos += 1;
            if c3 == '{' {
                field.spec_needs_expanding = true;
                count += 1;
            } else if c3 == '}' {
                count -= 1;
                if count == 0 {
                    field.spec = &input[spec_start..*pos - 1];
                    return Ok(field);
                }
            }
        }
        return Err(value_error("unmatched '{' in format spec"));
    } else if c != '}' {
        return Err(value_error("expected '}' before end of string"));
    }
    Ok(field)
}

/// `get_field_object`: positional/keyword lookup, then `.attr`/`[item]`.
/// Attribute lookups model the numeric data attributes (`real`, `imag`,
/// `numerator`, `denominator`); other names raise CPython's
/// `AttributeError` for a missing attribute (method objects, whose text
/// holds a memory address, are not modelled).
fn get_field_object<'v>(
    name: &[char],
    args: &'v [PctValue],
    kwargs: &'v [(String, PctValue)],
    auto: &mut AutoNumber,
) -> Result<Cow<'v, PctValue>, PyError> {
    let mut i = 0;
    while i < name.len() && name[i] != '.' && name[i] != '[' {
        i += 1;
    }
    let first = &name[..i];
    let first_idx = field_integer(first)?;
    let empty = first.is_empty();
    let numeric = empty || first_idx.is_some();
    if auto.state == AutoNumberState::Init && numeric {
        auto.state = if empty { AutoNumberState::Auto } else { AutoNumberState::Manual };
    }
    if numeric {
        if auto.state == AutoNumberState::Manual {
            if empty {
                return Err(value_error(
                    "cannot switch from manual field specification to automatic field numbering",
                ));
            }
        } else if !empty {
            return Err(value_error(
                "cannot switch from automatic field numbering to manual field specification",
            ));
        }
    }
    let index = if empty {
        auto.next += 1;
        Some(auto.next - 1)
    } else {
        first_idx
    };
    let mut obj: Cow<'v, PctValue> = match index {
        None => {
            let key: String = first.iter().collect();
            Cow::Borrowed(
                kwargs
                    .iter()
                    .rev()
                    .find(|(k, _)| *k == key)
                    .map(|(_, v)| v)
                    .ok_or_else(|| PyError::new("KeyError", str_repr(&key)))?,
            )
        }
        Some(k) => Cow::Borrowed(args.get(k).ok_or_else(|| {
            PyError::new("IndexError", format!("Replacement index {} out of range for positional args tuple", k))
        })?),
    };
    while i < name.len() {
        let c = name[i];
        i += 1;
        match c {
            '.' => {
                let start = i;
                while i < name.len() && name[i] != '.' && name[i] != '[' {
                    i += 1;
                }
                if start == i {
                    return Err(value_error("Empty attribute in format string"));
                }
                let attr: String = name[start..i].iter().collect();
                obj = Cow::Owned(get_attr(&obj, &attr)?);
            }
            '[' => {
                let start = i;
                let mut seen = false;
                while i < name.len() {
                    let c = name[i];
                    i += 1;
                    if c == ']' {
                        seen = true;
                        break;
                    }
                }
                if !seen {
                    return Err(value_error("Missing ']' in format string"));
                }
                let item = &name[start..i - 1];
                let idx = field_integer(item)?;
                if item.is_empty() {
                    return Err(value_error("Empty attribute in format string"));
                }
                obj = Cow::Owned(get_item(&obj, idx)?);
            }
            _ => {
                return Err(value_error("Only '.' or '[' may follow ']' in format field specifier"));
            }
        }
    }
    Ok(obj)
}

/// `getattr(obj, name)` for the data attributes of `int`, `bool` and
/// `float` (`int.real` of a bool is the int `0`/`1`).
fn get_attr(obj: &PctValue, attr: &str) -> Result<PctValue, PyError> {
    let as_int = |o: &PctValue| match o {
        PctValue::Int(n) => Some(n.clone()),
        PctValue::Bool(b) => Some(PyInt::from_i64(*b as i64)),
        _ => None,
    };
    let found = match (obj, attr) {
        (PctValue::Int(_) | PctValue::Bool(_), "real" | "numerator") => as_int(obj).map(PctValue::Int),
        (PctValue::Int(_) | PctValue::Bool(_), "imag") => Some(int(0)),
        (PctValue::Int(_) | PctValue::Bool(_), "denominator") => Some(int(1)),
        (PctValue::Float(f), "real") => Some(PctValue::Float(*f)),
        (PctValue::Float(_), "imag") => Some(PctValue::Float(0.0)),
        _ => None,
    };
    found.ok_or_else(|| {
        PyError::new("AttributeError", format!("'{}' object has no attribute '{}'", type_name(obj), attr))
    })
}

fn int(v: i64) -> PctValue {
    PctValue::Int(PyInt::from_i64(v))
}

/// `obj[idx]` (integer index) or `obj["key"]` (`idx == None`).
fn get_item(obj: &PctValue, idx: Option<usize>) -> Result<PctValue, PyError> {
    match (obj, idx) {
        (PctValue::Str(_), None) => Err(type_error("string indices must be integers")),
        (PctValue::Bytes(_), None) => Err(type_error("byte indices must be integers or slices, not str")),
        (PctValue::Str(s), Some(k)) => s
            .chars()
            .nth(k)
            .map(|c| PctValue::Str(c.to_string()))
            .ok_or_else(|| PyError::new("IndexError", "string index out of range".to_string())),
        (PctValue::Bytes(b), Some(k)) => b
            .get(k)
            .map(|&x| PctValue::Int(PyInt::from_i64(x as i64)))
            .ok_or_else(|| PyError::new("IndexError", "index out of range".to_string())),
        _ => Err(type_error(format!("'{}' object is not subscriptable", type_name(obj)))),
    }
}

/// `do_conversion`: `!r`, `!s`, `!a`.
fn do_conversion(obj: &PctValue, conversion: char) -> Result<String, PyError> {
    match conversion {
        'r' => Ok(obj.as_repr()),
        's' => Ok(obj.as_str()),
        'a' => Ok(ascii_of_repr(&obj.as_repr())),
        c => Err(value_error(if (c as u32) > 32 && (c as u32) < 127 {
            format!("Unknown conversion specifier {}", c)
        } else {
            format!("Unknown conversion specifier \\x{:x}", c as u32)
        })),
    }
}

/// `build_string` + `do_markup` + `MarkupIterator_next`.
fn build_string(
    input: &[char],
    args: &[PctValue],
    kwargs: &[(String, PctValue)],
    recursion_depth: i32,
    auto: &mut AutoNumber,
) -> Result<String, PyError> {
    if recursion_depth <= 0 {
        return Err(value_error("Max string recursion exceeded"));
    }
    let mut out = String::new();
    let end = input.len();
    let mut pos = 0;
    while pos < end {
        let start = pos;
        let mut c = '\0';
        let mut markup_follows = false;
        while pos < end {
            c = input[pos];
            pos += 1;
            if c == '{' || c == '}' {
                markup_follows = true;
                break;
            }
        }
        let at_end = pos >= end;
        let mut len = pos - start;
        if c == '}' && (at_end || input[pos] != c) {
            return Err(value_error("Single '}' encountered in format string"));
        }
        if at_end && c == '{' {
            return Err(value_error("Single '{' encountered in format string"));
        }
        if !at_end {
            if input[pos] == c {
                pos += 1;
                markup_follows = false;
            } else {
                len -= 1;
            }
        }
        out.extend(&input[start..start + len]);
        if !markup_follows {
            continue;
        }
        let field = parse_field(input, &mut pos)?;
        let obj = get_field_object(field.name, args, kwargs, auto)?;
        let obj = match field.conversion {
            Some(conv) => Cow::Owned(PctValue::Str(do_conversion(&obj, conv)?)),
            None => obj,
        };
        let spec: String = if field.spec_needs_expanding {
            build_string(field.spec, args, kwargs, recursion_depth - 1, auto)?
        } else {
            field.spec.iter().collect()
        };
        out.push_str(&format_value(&obj, &spec)?);
    }
    Ok(out)
}

/// `template.format(*args, **kwargs)` (CPython `str.format`,
/// `Objects/stringlib/unicode_format.h`): literal text with `{{`/`}}`
/// escapes, automatic or manual numbering, keyword fields, `[index]`
/// lookups, `!r`/`!s`/`!a` conversions and nested `{...}` inside specs
/// (two levels), each field rendered by [`format_value`].
pub fn str_format(template: &str, args: &[PctValue], kwargs: &[(String, PctValue)]) -> Result<String, PyError> {
    let input: Vec<char> = template.chars().collect();
    let mut auto = AutoNumber { state: AutoNumberState::Init, next: 0 };
    build_string(&input, args, kwargs, 2, &mut auto)
}

// ---------------------------------------------------------------------------
// round (`Objects/floatobject.c:float___round___impl`).
// ---------------------------------------------------------------------------

/// `round(x)`: nearest integer, ties to even (`round(2.5) == 2`).
/// `inf` raises `OverflowError: cannot convert float infinity to integer`,
/// `nan` raises `ValueError: cannot convert float NaN to integer`.
pub fn round_py(x: f64) -> Result<PyInt, PyError> {
    let mut rounded = x.round();
    if (x - rounded).abs() == 0.5 {
        rounded = 2.0 * (x / 2.0).round();
    }
    if rounded.is_nan() {
        return Err(value_error("cannot convert float NaN to integer"));
    }
    if rounded.is_infinite() {
        return Err(overflow_error("cannot convert float infinity to integer"));
    }
    Ok(trunc_float_to_bigint(rounded))
}

/// `round(x, ndigits)`: `x` rounded half-even at the `10^-ndigits` place on
/// its exact binary value (`double_round` via dtoa mode 3). `inf`/`nan`
/// return themselves, `ndigits > 323` returns `x`, `ndigits < -308` returns
/// a zero with the sign of `x`. A result too large for a float (only
/// reachable with `ndigits <= -308`) is `OverflowError: rounded value too
/// large to represent`.
pub fn try_round_py_nd(x: f64, ndigits: i64) -> Result<f64, PyError> {
    if !x.is_finite() {
        return Ok(x);
    }
    if ndigits > 323 {
        return Ok(x);
    }
    if ndigits < -308 {
        return Ok(0.0 * x);
    }
    let (digits, decpt) = dtoa(libm::fabs(x), Dtoa::Fixed(ndigits));
    let text = format!(
        "{}0{}e{}",
        if x.is_sign_negative() { "-" } else { "" },
        digits,
        decpt - digits.len() as i64
    );
    let rounded: f64 = text.parse().expect("dtoa digits parse as a float");
    if rounded.is_infinite() {
        return Err(overflow_error("rounded value too large to represent"));
    }
    Ok(rounded)
}

/// [`try_round_py_nd`] for the values klippy rounds, where the result
/// always fits: the overflow case returns `inf` with the sign of `x`
/// instead of the `OverflowError`.
pub fn round_py_nd(x: f64, ndigits: i64) -> f64 {
    try_round_py_nd(x, ndigits).unwrap_or(f64::INFINITY.copysign(x))
}

// ---------------------------------------------------------------------------
// Parsing: int(str[, base]), float(str).
// ---------------------------------------------------------------------------

/// `Py_ISSPACE` (ASCII whitespace of `pyctype.c`).
fn is_ascii_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

/// `_PyUnicode_TransformDecimalAndSpaceToASCII`: non-ASCII whitespace
/// becomes `' '`, decimal digits become `'0'..'9'`, and the first other
/// non-ASCII character becomes `'?'` and ends the text.
fn transform_decimal_and_space(s: &str) -> Vec<u8> {
    if s.is_ascii() {
        return s.as_bytes().to_vec();
    }
    let mut out = Vec::with_capacity(s.len());
    for c in s.chars() {
        if (c as u32) < 127 {
            out.push(c as u8);
        } else if unicode::is_non_ascii_space(c) {
            out.push(b' ');
        } else if let Some(d) = unicode::to_decimal(c) {
            out.push(b'0' + d as u8);
        } else {
            out.push(b'?');
            break;
        }
    }
    out
}

/// `repr(s)` cut to 200 characters (`%.200R`).
fn repr_200(s: &str) -> String {
    str_repr(s).chars().take(200).collect()
}

/// `PyLong_FromString(buf, &end, base)` on the transformed text, returning
/// the value only when the whole buffer was consumed.
fn long_from_ascii(buf: &[u8], base: u32) -> Option<BigInt> {
    // A NUL ends the C string, leaving the rest of the buffer unconsumed.
    if buf.contains(&0) {
        return None;
    }
    let at = |i: usize| buf.get(i).copied().unwrap_or(0);
    let digit_value = |b: u8| -> u32 {
        match b {
            b'0'..=b'9' => (b - b'0') as u32,
            b'a'..=b'z' => (b - b'a') as u32 + 10,
            b'A'..=b'Z' => (b - b'A') as u32 + 10,
            _ => 37,
        }
    };
    let mut i = 0;
    while i < buf.len() && is_ascii_space(buf[i]) {
        i += 1;
    }
    let mut negative = false;
    if at(i) == b'+' {
        i += 1;
    } else if at(i) == b'-' {
        i += 1;
        negative = true;
    }
    let mut base = base;
    let mut error_if_nonzero = false;
    if base == 0 {
        base = if at(i) != b'0' {
            10
        } else {
            match at(i + 1) {
                b'x' | b'X' => 16,
                b'o' | b'O' => 8,
                b'b' | b'B' => 2,
                _ => {
                    error_if_nonzero = true;
                    10
                }
            }
        };
    }
    if at(i) == b'0'
        && ((base == 16 && matches!(at(i + 1), b'x' | b'X'))
            || (base == 8 && matches!(at(i + 1), b'o' | b'O'))
            || (base == 2 && matches!(at(i + 1), b'b' | b'B')))
    {
        i += 2;
        if at(i) == b'_' {
            i += 1;
        }
    }
    if at(i) == b'_' {
        return None;
    }
    let start = i;
    let mut prev = 0u8;
    let mut digits = Vec::new();
    while digit_value(at(i)) < base || at(i) == b'_' {
        if at(i) == b'_' {
            if prev == b'_' {
                return None;
            }
        } else {
            digits.push(at(i));
        }
        prev = at(i);
        i += 1;
    }
    if prev == b'_' || i == start {
        return None;
    }
    let mut value = BigInt::from(BigUint::parse_bytes(&digits, base).expect("validated digits"));
    if error_if_nonzero && !value.is_zero() {
        return None;
    }
    while i < buf.len() && is_ascii_space(buf[i]) {
        i += 1;
    }
    if i != buf.len() {
        return None;
    }
    if negative {
        value = -value;
    }
    Some(value)
}

/// `int(s, base)` (CPython `long_new_impl` → `PyLong_FromUnicodeObject`):
/// strips whitespace (ASCII plus Unicode `str.isspace()`, but not
/// `\x1c`..`\x1f`), accepts a sign, Unicode decimal digits, single
/// underscores between digits and after a `0x`/`0o`/`0b` prefix (`base` 0
/// or the matching base); `base == 0` rejects leading zeros of a nonzero
/// value. Errors: `ValueError: int() base must be >= 2 and <= 36, or 0`,
/// `ValueError: invalid literal for int() with base 10: 'x'` (repr cut to
/// 200 characters).
pub fn int_from_string(s: &str, base: i64) -> Result<PyInt, PyError> {
    if (base != 0 && base < 2) || base > 36 {
        return Err(value_error("int() base must be >= 2 and <= 36, or 0"));
    }
    let buf = transform_decimal_and_space(s);
    long_from_ascii(&buf, base as u32)
        .map(PyInt)
        .ok_or_else(|| value_error(format!("invalid literal for int() with base {}: {}", base, repr_200(s))))
}

/// `_Py_dg_strtod` with the `_Py_parse_inf_or_nan` fallback: the longest
/// float prefix of `b` and its length, or `None` when nothing parses.
fn strtod_prefix(b: &[u8]) -> Option<(f64, usize)> {
    let at = |i: usize| b.get(i).copied().unwrap_or(0);
    let mut i = 0;
    if matches!(at(0), b'+' | b'-') {
        i = 1;
    }
    let int_start = i;
    while at(i).is_ascii_digit() {
        i += 1;
    }
    let mut ndigits = i - int_start;
    if at(i) == b'.' {
        i += 1;
        let frac_start = i;
        while at(i).is_ascii_digit() {
            i += 1;
        }
        ndigits += i - frac_start;
    }
    if ndigits == 0 {
        return parse_inf_or_nan(b);
    }
    if matches!(at(i), b'e' | b'E') {
        let e_start = i;
        i += 1;
        if matches!(at(i), b'+' | b'-') {
            i += 1;
        }
        let exp_digits = i;
        while at(i).is_ascii_digit() {
            i += 1;
        }
        if i == exp_digits {
            i = e_start;
        }
    }
    let text = std::str::from_utf8(&b[..i]).expect("ASCII float text");
    Some((text.parse().expect("validated float syntax"), i))
}

/// `_Py_parse_inf_or_nan`: `[+-](inf|infinity|nan)`, case-insensitive.
fn parse_inf_or_nan(b: &[u8]) -> Option<(f64, usize)> {
    let mut i = 0;
    let mut negate = false;
    match b.first() {
        Some(b'-') => {
            negate = true;
            i = 1;
        }
        Some(b'+') => i = 1,
        _ => {}
    }
    let matches_at = |i: usize, word: &[u8]| b.len() >= i + word.len() && b[i..i + word.len()].eq_ignore_ascii_case(word);
    if matches_at(i, b"inf") {
        i += 3;
        if matches_at(i, b"inity") {
            i += 5;
        }
        Some((if negate { f64::NEG_INFINITY } else { f64::INFINITY }, i))
    } else if matches_at(i, b"nan") {
        // `_Py_dg_stdnan(negate)`: the default quiet NaN with the sign bit.
        let bits = 0x7ff8_0000_0000_0000u64 | (u64::from(negate) << 63);
        Some((f64::from_bits(bits), i + 3))
    } else {
        None
    }
}

/// `float_from_string_inner`: strip ASCII whitespace, parse, demand that
/// the parse ends at the stripped end.
fn float_from_ascii(t: &[u8], s: &str) -> Result<f64, PyError> {
    let mut start = 0;
    let mut last = t.len();
    while start < last && is_ascii_space(t[start]) {
        start += 1;
    }
    while start + 1 < last && is_ascii_space(t[last - 1]) {
        last -= 1;
    }
    match strtod_prefix(&t[start..]) {
        Some((x, n)) if start + n == last => Ok(x),
        // `PyOS_string_to_double` reports an empty text itself, quoting
        // the (empty) C string instead of the argument.
        None if start == last => Err(value_error("could not convert string to float: ''")),
        _ => Err(value_float_error(s)),
    }
}

fn value_float_error(s: &str) -> PyError {
    value_error(format!("could not convert string to float: {}", str_repr(s)))
}

/// `float(s)` (CPython `PyFloat_FromString`): strips ASCII and Unicode
/// whitespace, accepts Unicode decimal digits, `inf`/`infinity`/`nan` in any
/// case with a sign, and single underscores between digits. Errors:
/// `ValueError: could not convert string to float: 'x'`.
pub fn float_from_string(s: &str) -> Result<f64, PyError> {
    let buf = transform_decimal_and_space(s);
    let cstr_end = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    if !buf[..cstr_end].contains(&b'_') {
        return float_from_ascii(&buf, s);
    }
    // `_Py_string_to_number_with_underscores`.
    let mut dup = Vec::with_capacity(buf.len());
    let mut prev = 0u8;
    for &c in &buf[..cstr_end] {
        if c == b'_' {
            if !prev.is_ascii_digit() {
                return Err(value_float_error(s));
            }
        } else {
            dup.push(c);
            if prev == b'_' && !c.is_ascii_digit() {
                return Err(value_float_error(s));
            }
        }
        prev = c;
    }
    if prev == b'_' || cstr_end != buf.len() {
        return Err(value_float_error(s));
    }
    float_from_ascii(&dup, s)
}

// ---------------------------------------------------------------------------
// Mixed int/float arithmetic helpers used by value.rs.
// ---------------------------------------------------------------------------

/// Exact comparison of a `BigInt` against an `f64` as rationals
/// (CPython `Objects/floatobject.c:float_richcompare`).
/// Returns `Ordering` of `bigint - float`; callers handle `nan`.
pub fn cmp_bigint_float(a: &BigInt, b: f64) -> Ordering {
    if b.is_nan() {
        return Ordering::Equal;
    }
    if b.is_infinite() {
        return if b > 0.0 { Ordering::Less } else { Ordering::Greater };
    }
    Rat::from_bigint(a.clone()).cmp(&Rat::from_f64(b))
}

/// `float_divmod` (`Objects/floatobject.c`): `(a // b, a % b)` with the
/// remainder sign of `b` and the floor quotient snapped to an integer.
fn float_divmod(vx: f64, wx: f64) -> Result<(f64, f64), PyError> {
    if wx == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float divmod()".to_string()));
    }
    let mut m = libm::fmod(vx, wx);
    let mut div = (vx - m) / wx;
    if m != 0.0 {
        if (wx < 0.0) != (m < 0.0) {
            m += wx;
            div -= 1.0;
        }
    } else {
        m = 0.0f64.copysign(wx);
    }
    let floordiv = if div != 0.0 {
        let mut f = libm::floor(div);
        if div - f > 0.5 {
            f += 1.0;
        }
        f
    } else {
        0.0f64.copysign(vx / wx)
    };
    Ok((floordiv, m))
}

/// Float floor division `a // b` (CPython `float_floor_div`, the quotient
/// of `float_divmod`; division by zero raises `ZeroDivisionError: float
/// divmod()`).
pub fn float_floor_div(a: f64, b: f64) -> Result<f64, PyError> {
    float_divmod(a, b).map(|(q, _)| q)
}

/// Float modulo `a % b` (CPython `float_rem`): `fmod` corrected to the sign
/// of `b`, zero remainders take the sign of `b`; `ZeroDivisionError: float
/// modulo`.
pub fn float_mod(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float modulo".to_string()));
    }
    let mut m = libm::fmod(a, b);
    if m != 0.0 {
        if (b < 0.0) != (m < 0.0) {
            m += b;
        }
    } else {
        m = 0.0f64.copysign(b);
    }
    Ok(m)
}

/// True division `a / b` for floats (CPython `float_div`).
pub fn float_true_div(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float division by zero".to_string()));
    }
    Ok(a / b)
}

/// `a ** b` for floats (CPython `float_pow`): the special cases of
/// `Objects/floatobject.c` (`x ** 0 == 1`, `1 ** nan == 1`, infinities,
/// signed zeros, `ZeroDivisionError: 0.0 cannot be raised to a negative
/// power`), then C `pow`, whose `ERANGE` (overflow, or underflow to a
/// nonzero subnormal) is `OverflowError: (34, 'Numerical result out of
/// range')`. A negative base with a non-integer exponent is complex in
/// Python; this returns C `pow`'s `nan` and callers handle that case.
pub fn float_pow(iv: f64, iw: f64) -> Result<f64, PyError> {
    let is_odd_integer = |x: f64| libm::fmod(libm::fabs(x), 2.0) == 1.0;
    if iw == 0.0 {
        return Ok(1.0);
    }
    if iv.is_nan() {
        return Ok(iv);
    }
    if iw.is_nan() {
        return Ok(if iv == 1.0 { 1.0 } else { iw });
    }
    if iw.is_infinite() {
        let av = libm::fabs(iv);
        return Ok(if av == 1.0 {
            1.0
        } else if (iw > 0.0) == (av > 1.0) {
            libm::fabs(iw)
        } else {
            0.0
        });
    }
    if iv.is_infinite() {
        let odd = is_odd_integer(iw);
        return Ok(if iw > 0.0 {
            if odd {
                iv
            } else {
                libm::fabs(iv)
            }
        } else if odd {
            0.0f64.copysign(iv)
        } else {
            0.0
        });
    }
    if iv == 0.0 {
        let odd = is_odd_integer(iw);
        if iw < 0.0 {
            return Err(PyError::new(
                "ZeroDivisionError",
                "0.0 cannot be raised to a negative power".to_string(),
            ));
        }
        return Ok(if odd { iv } else { 0.0 });
    }
    let mut iv = iv;
    let mut negate_result = false;
    if iv < 0.0 {
        if iw != libm::floor(iw) {
            return Ok(libm::pow(iv, iw));
        }
        iv = -iv;
        negate_result = is_odd_integer(iw);
    }
    if iv == 1.0 {
        return Ok(if negate_result { -1.0 } else { 1.0 });
    }
    const ERANGE: i32 = 34;
    let (mut ix, mut errno) = libm::pow_errno(iv, iw);
    // `_Py_ADJUST_ERANGE1`.
    if errno == 0 {
        if ix.is_infinite() {
            errno = ERANGE;
        }
    } else if errno == ERANGE && ix == 0.0 {
        errno = 0;
    }
    if negate_result {
        ix = -ix;
    }
    if errno != 0 {
        return Err(if errno == ERANGE {
            overflow_error("(34, 'Numerical result out of range')")
        } else {
            value_error(format!("({}, 'Numerical argument out of domain')", errno))
        });
    }
    Ok(ix)
}

/// `sum` of floats as CPython 3.8 (plain left-to-right addition; Neumaier
/// summation only arrived in 3.12). The empty sum is `0.0` here; CPython's
/// `sum([])` is the int `0`, which callers handle.
pub fn py_sum_floats(xs: &[f64]) -> f64 {
    let mut acc = 0.0f64;
    for &x in xs {
        acc += x;
    }
    acc
}
