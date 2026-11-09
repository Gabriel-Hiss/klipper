//! Float text, `%` formatting, `format()` mini-language, rounding and
//! numeric parsing reproducing CPython 3.8.2 (`Objects/floatobject.c`,
//! `Python/pystrtod.c`, `Python/format.c`, `bltinmodule.c`).
//!
//! - `float_repr`/`float_str`: shortest round-trip decimal; CPython `repr`
//!   and `str` for floats share this path in 3.8 (they differ only for
//!   integral values: both use `1.0` with a trailing `.0`).
//! - `percent_format`: `str.__mod__` with `%s %r %d %i %u %f %F %e %E %g %G
//!   %x %X %o %c %%`, flags `# 0 - space +`, width `*` or number,
//!   precision `.` number or `*`, mapping-key `%(name)s`.
//! - `format_spec`: `format(x, spec)` mini-language
//!   `[[fill]align][sign][#][0][width][grouping][.precision][type]`.
//!   Types: empty, `s`, `e E f F g G % b c d o x X n`.
//! - `round_py`/`round_py_nd`: correctly rounded half-even
//!   (CPython `Objects/floatobject.c:float_round`, `pystrtod.c`
//!   correctly-rounded decimal conversion).
//! - `int_from_float`/`float_from_string`/`int_from_string`:
//!   parsing with CPython syntax and exact error texts
//!   (`Objects/floatobject.c:float_from_string`,
//!   `Objects/abstract.c:PyNumber_Long`).

use crate::int::PyInt;
use crate::libm;
use num_bigint::BigInt;
use num_integer::Integer;
use num_traits::{One, ToPrimitive, Zero};
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
// Float repr / str
// ---------------------------------------------------------------------------

/// Shortest round-trip digits for a finite nonzero `f` as
/// `(digits_without_point, decimal_exponent)` where the value equals
/// `0.d1d2... * 10^exp` i.e. decimal point just left of `d1`.
/// Uses Rust's `{:e}` (Grisu-shortest) parsed back to raw digits.
fn shortest_digits(f: f64) -> (String, i32) {
    // Rust formats e.g. "1.2345e0", "5e-324".
    let s = format!("{:e}", f64_abs(f));
    // s is like "1.234e5" or "1e5".
    let epos = s.find('e').unwrap();
    let mant = &s[..epos];
    let exp: i32 = s[epos + 1..].parse().unwrap_or(0);
    let digits: String = mant.chars().filter(|c| *c != '.').collect();
    // {:e} gives d.ddd x 10^exp, so 0.ddd x 10^(exp+1).
    (digits, exp + 1)
}

fn f64_abs(f: f64) -> f64 {
    libm::fabs(f)
}

/// Format finite `f` like CPython `float.__repr__` / `str` in 3.8:
/// shortest round-trip, fixed notation unless the decimal point is at or
/// beyond the CPython threshold (`floatobject.c:PyOS_double_to_string`
/// with `format_code='r'`): exponent notation when `decpt <= -4` or
/// `decpt > 16` where `decpt` is the `exp` above. This yields `1e+16`,
/// `1e-05`, `1000000000000000.0` for 1e15, `0.0001` for 1e-4.
pub fn float_repr(f: f64) -> String {
    float_repr_inner(f, true)
}

/// CPython `str(float)` equals `repr(float)` in 3.8.
pub fn float_str(f: f64) -> String {
    float_repr_inner(f, true)
}

fn float_repr_inner(f: f64, add_dot_zero: bool) -> String {
    if f.is_nan() {
        return "nan".to_string();
    }
    if f.is_infinite() {
        return if f > 0.0 { "inf".to_string() } else { "-inf".to_string() };
    }
    if f == 0.0 {
        return if f.is_sign_negative() { "-0.0".to_string() } else { "0.0".to_string() };
    }
    let neg = f < 0.0;
    let (digits, exp) = shortest_digits(f);
    let decpt = exp; // position of decimal point relative to first digit (+1 based)
    let use_exp = !(-3..=16).contains(&decpt);
    let body = if use_exp {
        // d.ddd e+/-XX with at least two exponent digits
        let mut mant = String::new();
        mant.push(digits.as_bytes()[0] as char);
        if digits.len() > 1 {
            mant.push('.');
            mant.push_str(&digits[1..]);
        }
        let e = decpt - 1;
        let sign = if e >= 0 { "+" } else { "-" };
        let abs_e = e.abs();
        let es = if abs_e < 10 {
            format!("{:02}", abs_e)
        } else {
            format!("{}", abs_e)
        };
        format!("{}e{}{}", mant, sign, es)
    } else if decpt <= 0 {
        // 0.000ddd
        let zeros = (-decpt) as usize;
        format!("0.{}{}", "0".repeat(zeros), digits)
    } else if decpt as usize >= digits.len() {
        // ddd000 with .0
        let zeros = decpt as usize - digits.len();
        format!("{}{}.0", digits, "0".repeat(zeros))
    } else {
        let d = decpt as usize;
        let mut s = digits[..d].to_string();
        s.push('.');
        s.push_str(&digits[d..]);
        if add_dot_zero {
            // no-op, already has fraction
        }
        s
    };
    if neg {
        format!("-{}", body)
    } else {
        body
    }
}

/// CPython float hash (`Objects/floatobject.c:float_hash`).
/// Used so `1 == 1.0 == True` map to the same key hash.
/// Returns the CPython hash value.
pub fn float_py_hash(f: f64) -> i64 {
    if f.is_nan() {
        0
    } else if f.is_infinite() {
        if f > 0.0 { 314159 } else { -314159 }
    } else if f == 0.0 {
        0
    } else {
        // Exact rational: f = p/q with q a power of two, hash = p * inv(q)
        // mod (2**61 - 1). Matches CPython's `float_hash` exactly,
        // including integral floats agreeing with `int` hashes.
        let negative = f < 0.0;
        let a = libm::fabs(f);
        let bits = a.to_bits();
        let raw_exp = ((bits >> 52) & 0x7FF) as i32;
        let mant_bits = bits & 0x000F_FFFF_FFFF_FFFF;
        let (m, e): (BigInt, i32) = if raw_exp == 0 {
            (BigInt::from(mant_bits), -1074)
        } else {
            (BigInt::from((1u64 << 52) | mant_bits), raw_exp - 1023 - 52)
        };
        let (num, den): (BigInt, BigInt) = if e >= 0 {
            (m << (e as usize), BigInt::one())
        } else {
            (m, BigInt::one() << ((-e) as usize))
        };
        const MOD: u64 = (1u64 << 61) - 1;
        let mod_b = BigInt::from(MOD);
        let p_mod = (&num % &mod_b).to_u64().unwrap_or(0);
        // den is 2^k; reduce mod MOD then invert via Fermat (MOD prime).
        let den_mod = (&den % &mod_b).to_u64().unwrap_or(0);
        let inv = modpow_u(den_mod, MOD - 2, MOD);
        let h = ((p_mod as u128 * inv as u128) % MOD as u128) as u64 as i64;
        if negative {
            let nh = -h;
            if nh == -1 { -2 } else { nh }
        } else if h == -1 {
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

// ---------------------------------------------------------------------------
// Integer radix / sign formatting helpers (shared by % and format)
// ---------------------------------------------------------------------------


fn bigint_abs_bigstr(n: &BigInt, radix: u32, upper: bool) -> String {
    let mag = if n.sign() == num_bigint::Sign::Minus { -n.clone() } else { n.clone() };
    if mag.is_zero() {
        return "0".to_string();
    }
    let r = BigInt::from(radix as i32);
    let mut cur = mag;
    let mut out = String::new();
    while !cur.is_zero() {
        let (q, rem) = cur.div_rem(&r);
        let d = rem.to_i64().unwrap_or(0) as u32;
        let c = if d < 10 {
            (b'0' + d as u8) as char
        } else if upper {
            (b'A' + (d - 10) as u8) as char
        } else {
            (b'a' + (d - 10) as u8) as char
        };
        out.push(c);
        cur = q;
    }
    out.chars().rev().collect()
}

#[derive(Clone, Copy, PartialEq)]
enum Align {
    Left,
    Right,
    Center,
    Zero,
}

/// Pad `body` (including sign+prefix already merged into `raw`) to `width`.
/// `sign_len` = number of leading sign/prefix chars that stay next to the zero
/// padding when `Align::Zero`. `width` counts characters (ASCII only in our tests).
fn pad(body: &str, width: Option<usize>, align: Align, fill: char, sign_len: usize) -> String {
    let w = width.unwrap_or(0);
    let len = body.chars().count();
    if len >= w {
        return body.to_string();
    }
    let padn = w - len;
    match align {
        Align::Left => format!("{}{}", body, fill.to_string().repeat(padn)),
        Align::Right => format!("{}{}", fill.to_string().repeat(padn), body),
        Align::Center => {
            let left = padn / 2;
            let right = padn - left;
            format!("{}{}{}", fill.to_string().repeat(left), body, fill.to_string().repeat(right))
        }
        Align::Zero => {
            let (head, tail) = body.split_at(body.char_indices().nth(sign_len).map(|(i, _)| i).unwrap_or(body.len()));
            format!("{}{}{}", head, "0".repeat(padn), tail)
        }
    }
}

/// Apply width/align with CPython defaults for the given type.
/// Defaults: `s` left, numeric right.
fn apply_width(
    body: String,
    width: Option<usize>,
    align: Option<Align>,
    fill: char,
    zero_pad: bool,
    numeric: bool,
    sign_len: usize,
) -> String {
    let a = match align {
        Some(x) => x,
        None => {
            if zero_pad {
                Align::Zero
            } else if numeric {
                Align::Right
            } else {
                Align::Left
            }
        }
    };
    pad(&body, width, a, fill, sign_len)
}

// ---------------------------------------------------------------------------
// % / str mode formatting core.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct PctSpec {
    flags: String,
    width: Option<usize>,
    star_width: bool,
    precision: Option<usize>,
    star_prec: bool,
    conv: char,
    key: Option<String>,
}

fn parse_pct_fmt(fmt: &str) -> Result<Vec<(String, PctSpec)>, String> {
    // Returns list of (literal_before, spec). Errors as CPython ValueError strs.
    let chars: Vec<char> = fmt.chars().collect();
    let mut i = 0;
    let mut out = Vec::new();
    let mut lit = String::new();
    while i < chars.len() {
        if chars[i] == '%' {
            if i + 1 < chars.len() && chars[i + 1] == '%' {
                lit.push('%');
                i += 2;
                continue;
            }
            // mapping key?
            let mut key: Option<String> = None;
            let mut j = i + 1;
            if j < chars.len() && chars[j] == '(' {
                let mut k = String::new();
                let mut kk = j + 1;
                while kk < chars.len() && chars[kk] != ')' {
                    k.push(chars[kk]);
                    kk += 1;
                }
                if kk >= chars.len() {
                    // CPython: incomplete format key. Raise.
                    return Err("incomplete format key".to_string());
                }
                key = Some(k);
                j = kk + 1;
            }
            let mut flags = String::new();
            while j < chars.len() && matches!(chars[j], '#' | '0' | '-' | ' ' | '+') {
                flags.push(chars[j]);
                j += 1;
            }
            let mut star_width = false;
            let mut width: Option<usize> = None;
            if j < chars.len() && chars[j] == '*' {
                star_width = true;
                j += 1;
            } else {
                let start = j;
                while j < chars.len() && chars[j].is_ascii_digit() {
                    j += 1;
                }
                if j > start {
                    let w: String = chars[start..j].iter().collect();
                    width = Some(w.parse().unwrap_or(0));
                    // leading 0 already captured as flag; e.g. "%05d" => flag 0 width 5
                    // Our flag scan consumed '0' as flag, correct.
                }
            }
            let mut precision: Option<usize> = None;
            let mut star_prec = false;
            if j < chars.len() && chars[j] == '.' {
                j += 1;
                if j < chars.len() && chars[j] == '*' {
                    star_prec = true;
                    j += 1;
                } else {
                    let start = j;
                    while j < chars.len() && chars[j].is_ascii_digit() {
                        j += 1;
                    }
                    if j == start {
                        precision = Some(0);
                    } else {
                        let s: String = chars[start..j].iter().collect();
                        precision = Some(s.parse().unwrap_or(0));
                    }
                }
            }
            if j >= chars.len() {
                let rest: String = fmt.chars().skip(i).collect();
                return Err(format!("incomplete format: {}", rest));
            }
            let conv = chars[j];
            j += 1;
            out.push((
                lit.clone(),
                PctSpec {
                    flags,
                    width,
                    star_width,
                    precision,
                    star_prec,
                    conv,
                    key,
                },
            ));
            lit.clear();
            i = j;
        } else {
            lit.push(chars[i]);
            i += 1;
        }
    }
    out.push((lit, PctSpec::end()));
    Ok(out)
}

impl PctSpec {
    fn end() -> Self {
        PctSpec {
            flags: String::new(),
            width: None,
            star_width: false,
            precision: None,
            star_prec: false,
            conv: '\0',
            key: None,
        }
    }
    fn is_end(&self) -> bool {
        self.conv == '\0'
    }
}

// Value view for %-formatting (plain scalars; containers not needed).

/// Scalar value for `%` formatting: mirrors CPython's accepted types.
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
}

impl PctValue {
    /// Convert to the `%s` text (CPython `str()`).
    pub fn as_str(&self) -> String {
        match self {
            PctValue::Str(s) => s.clone(),
            PctValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
            PctValue::Int(n) => n.0.to_string(),
            PctValue::Float(f) => float_repr(*f),
            PctValue::Bytes(b) => bytes_repr(b),
        }
    }
    /// Convert to the `%r` text (CPython `repr()`).
    pub fn as_repr(&self) -> String {
        match self {
            PctValue::Str(s) => str_repr(s),
            PctValue::Bool(b) => if *b { "True".to_string() } else { "False".to_string() },
            PctValue::Int(n) => n.0.to_string(),
            PctValue::Float(f) => float_repr(*f),
            PctValue::Bytes(b) => bytes_repr(b),
        }
    }
}

/// CPython 3.8 `str` repr quoting (used by `%r` and `PyValue` repr).
/// Chooses single quotes, switches to double if the string contains a
/// single quote but no double quote; escapes backslash, quotes and
/// control characters; non-printable uses `\x..`, `\u....`, `\U........`.
pub fn str_repr(s: &str) -> String {
    quote_string(s, '\'', false)
}

/// CPython `bytes` repr: `b'...'` with same rules as str.
pub fn bytes_repr(b: &[u8]) -> String {
    // ASCII fast path with CPython-like escaping.
    let has_single = b.contains(&b'\'');
    let has_double = b.contains(&b'"');
    let quote = if has_single && !has_double { b'"' as char } else { b'\'' as char };
    let mut out = String::from("b");
    out.push(quote);
    for &c in b {
        if c == b'\\' {
            out.push_str("\\\\");
        } else if c == quote as u8 {
            out.push('\\');
            out.push(c as char);
        } else if (0x20..0x7f).contains(&c) {
            out.push(c as char);
        } else if c == b'\n' {
            out.push_str("\\n");
        } else if c == b'\r' {
            out.push_str("\\r");
        } else if c == b'\t' {
            out.push_str("\\t");
        } else {
            out.push_str(&format!("\\x{:02x}", c));
        }
    }
    out.push(quote);
    out
}

fn quote_string(s: &str, _hint: char, ascii_only: bool) -> String {
    let has_single = s.contains('\'');
    let has_double = s.contains('"');
    let quote = if has_single && !has_double { '"' } else { '\'' };
    let mut out = String::new();
    out.push(quote);
    for c in s.chars() {
        if c == '\\' {
            out.push_str("\\\\");
        } else if c == quote {
            out.push('\\');
            out.push(c);
        } else if c == '\n' {
            out.push_str("\\n");
        } else if c == '\r' {
            out.push_str("\\r");
        } else if c == '\t' {
            out.push_str("\\t");
        } else if c.is_control() || (ascii_only && !c.is_ascii()) {
            let o = c as u32;
            if o < 0x100 {
                out.push_str(&format!("\\x{:02x}", o));
            } else if o < 0x10000 {
                out.push_str(&format!("\\u{:04x}", o));
            } else {
                out.push_str(&format!("\\U{:08x}", o));
            }
        } else {
            out.push(c);
        }
    }
    out.push(quote);
    out
}

/// String with ASCII-only escaping (CPython `ascii()` / `repr` with `ensure_ascii`); exposed for JSON.
pub fn str_repr_ascii(s: &str) -> String {
    quote_string(s, '\'', true)
}

fn as_long(v: &PctValue) -> Result<PyInt, String> {
    match v {
        PctValue::Int(n) => Ok(n.clone()),
        PctValue::Bool(b) => Ok(PyInt::from_i64(if *b { 1 } else { 0 })),
        PctValue::Float(f) => {
            if f.is_nan() {
                Err("cannot convert float NaN to integer".to_string())
            } else if f.is_infinite() {
                Err("cannot convert float infinity to integer".to_string())
            } else {
                Ok(trunc_float_to_bigint(*f))
            }
        }
        _ => Err(format!("an integer is required (got type {})", type_name(v))),
    }
}

fn type_name(v: &PctValue) -> &'static str {
    match v {
        PctValue::Str(_) => "str",
        PctValue::Bool(_) => "bool",
        PctValue::Int(_) => "int",
        PctValue::Float(_) => "float",
        PctValue::Bytes(_) => "bytes",
    }
}

/// `int(float)` truncation toward zero with big-int overflow
/// (CPython `Objects/abstract.c:PyNumber_Long` on floats).
/// Inf/NaN raise with the exact CPython texts.
pub fn trunc_float_to_bigint(f: f64) -> PyInt {
    debug_assert!(f.is_finite());
    if f == 0.0 {
        return PyInt::from_i64(0);
    }
    let neg = f < 0.0;
    let a = libm::fabs(f);
    // Exact: a = m * 2^e; truncate drops fractional bits.
    let bits = a.to_bits();
    let raw_exp = ((bits >> 52) & 0x7ff) as i32;
    let mant_bits = bits & 0x000f_ffff_ffff_ffff;
    if raw_exp == 0 {
        // subnormal < 1 -> 0
        return PyInt::from_i64(0);
    }
    let e: i32 = raw_exp - 1023 - 52; // a = m_int * 2^e, m_int includes hidden bit
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

/// Convert float to `f64` for `%f/%g`-family (uses `__float__` semantics:
/// int converts exactly via `to_f64`, which matches CPython's overflow
/// to `OverflowError` for huge ints converted to float).
pub fn as_float(v: &PctValue) -> Result<f64, PyError> {
    match v {
        PctValue::Float(f) => Ok(*f),
        PctValue::Int(n) => n
            .0
            .to_f64()
            .ok_or_else(|| PyError::new("OverflowError", "int too large to convert to float".to_string())),
        PctValue::Bool(b) => Ok(if *b { 1.0 } else { 0.0 }),
        _ => Err(PyError::new(
            "TypeError",
            format!("must be real number, not {}", type_name(v)),
        )),
    }
}

fn format_int_conv(
    v: &PctValue,
    conv: char,
    flags: &str,
    width: Option<usize>,
    prec: Option<usize>,
    star_width_neg: bool,
) -> Result<String, PyError> {
    // CPython %d/%i/%u/%x/%X/%o/%c on bool/int/float.
    let neg_flag_original = flags.contains('-');
    let _ = neg_flag_original;
    if conv == 'c' {
        // %c: int -> single char (chr), str of len 1, bytes of len 1.
        let ch: char = match v {
            PctValue::Int(n) => {
                let code = n.0.to_i64().ok_or_else(|| {
                    PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string())
                })?;
                if !(0..=0x10FFFF).contains(&code) {
                    return Err(PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string()));
                }
                char::from_u32(code as u32).ok_or_else(|| {
                    PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string())
                })?
            }
            PctValue::Str(s) => {
                let mut cs = s.chars();
                match (cs.next(), cs.next()) {
                    (Some(c), None) => c,
                    _ => return Err(PyError::new("TypeError", "%c requires int or char".to_string())),
                }
            }
            PctValue::Bytes(b) => {
                if b.len() != 1 {
                    return Err(PyError::new("TypeError", "%c requires int or char".to_string()));
                }
                b[0] as char
            }
            PctValue::Bool(b) => if *b { '\u{1}' } else { '\0' },
            PctValue::Float(_) => {
                return Err(PyError::new("TypeError", "%c requires int or char".to_string()));
            }
        };
        let body = ch.to_string();
        let align = if flags.contains('-') { Some(Align::Left) } else { None };
        return Ok(apply_width(body, width, align, ' ', false, false, 0));
    }
    let n = as_long(v).map_err(|m| {
        if m.starts_with("cannot convert") {
            if m.contains("NaN") {
                PyError::new("ValueError", m)
            } else {
                PyError::new("OverflowError", m)
            }
        } else {
            PyError::new("TypeError", format!("%{} format: a real number is required, not {}", conv, type_name(v)))
        }
    })?;
    let radix = match conv {
        'd' | 'i' | 'u' => 10,
        'x' | 'X' => 16,
        'o' => 8,
        _ => unreachable!(),
    };
    let upper = conv == 'X';
    let negative = n.0.sign() == num_bigint::Sign::Minus;
    let mut digits = bigint_abs_bigstr(n.inner(), radix, upper);
    // precision for integers: minimum digits; precision 0 with value 0 => empty
    if let Some(p) = prec {
        if p == 0 && digits == "0" {
            digits = String::new();
        } else if digits.len() < p {
            digits = format!("{}{}", "0".repeat(p - digits.len()), digits);
        }
    }
    let mut prefix = String::new();
    if negative {
        prefix.push('-');
    } else if flags.contains('+') && radix == 10 {
        prefix.push('+');
    } else if flags.contains(' ') && radix == 10 {
        prefix.push(' ');
    }
    if flags.contains('#') {
        match conv {
            'x' => prefix.push_str("0x"),
            'X' => prefix.push_str("0X"),
            'o' => {
                if !digits.starts_with('0') {
                    digits = format!("0{}", digits);
                }
            }
            _ => {}
        }
    }
    let body = format!("{}{}", prefix, digits);
    let sign_len = prefix.len();
    let zero = flags.contains('0') && !flags.contains('-') && prec.is_none();
    let align = if flags.contains('-') {
        Some(Align::Left)
    } else if zero {
        Some(Align::Zero)
    } else {
        None
    };
    let _ = star_width_neg;
    Ok(apply_width(body, width, align, ' ', false, true, sign_len))
}

fn format_float_conv(
    v: &PctValue,
    conv: char,
    flags: &str,
    width: Option<usize>,
    prec: Option<usize>,
) -> Result<String, PyError> {
    let f = as_float(v)?;
    let prec = prec.unwrap_or(6);
    let mut body: String;
    let mut prefix_len_for_zero: usize;
    // inf/nan ignore precision and #/0 width rules per CPython (pad with spaces).
    if f.is_nan() || f.is_infinite() {
        let base = if f.is_nan() {
            if conv == 'F' { "NAN" } else { "nan" }
        } else if f > 0.0 {
            if conv == 'F' { "INF" } else { "inf" }
        } else if conv == 'F' {
            "-INF"
        } else {
            "-inf"
        };
        if f.is_infinite() && f > 0.0 {
            if flags.contains('+') {
                body = format!("+{}", base);
            } else if flags.contains(' ') {
                body = format!(" {}", base);
            } else {
                body = base.to_string();
            }
        } else {
            body = base.to_string();
        }
        // inf/nan: '#' and '0' ignored, width pads with spaces.
        let align = if flags.contains('-') { Some(Align::Left) } else { None };
        let sign_len = if body.starts_with('-') || body.starts_with('+') || body.starts_with(' ') { 1 } else { 0 };
        let _ = sign_len;
        return Ok(apply_width(body, width, align, ' ', false, true, 0));
    }
    match conv {
        'f' | 'F' => {
            body = format_f_fixed(f, prec, conv == 'F');
            let _ = &mut body;
            prefix_len_for_zero = signed_prefix_len(&body);
            // '#' keep trailing zeros / decimal point
            if flags.contains('#') && !body.contains('.') {
                body.push('.');
                prefix_len_for_zero = signed_prefix_len(&body);
            }
        }
        'e' | 'E' => {
            body = format_f_exp(f, prec, conv == 'E');
            if flags.contains('#') && !body.contains('.') {
                if let Some(pos) = body.find(['e', 'E']) {
                    let (a, b) = body.split_at(pos);
                    body = format!("{}.{}", a, b);
                } else {
                    body.push('.');
                }
            }
            prefix_len_for_zero = signed_prefix_len(&body);
        }
        'g' | 'G' => {
            body = format_f_general(f, prec, conv == 'G', flags.contains('#'));
            if flags.contains('#') && !body.contains('.') && !body.contains('e') && !body.contains('E') {
                body.push('.');
            }
            prefix_len_for_zero = signed_prefix_len(&body);
        }
        _ => unreachable!(),
    }
    // sign flags
    if !body.starts_with('-') {
        if flags.contains('+') {
            body = format!("+{}", body);
            prefix_len_for_zero += 1;
        } else if flags.contains(' ') {
            body = format!(" {}", body);
            prefix_len_for_zero += 1;
        }
    } else if flags.contains('+') {
        // already negative, no-op (CPython keeps '-')
    }
    let zero = flags.contains('0') && !flags.contains('-');
    let align = if flags.contains('-') {
        Some(Align::Left)
    } else if zero {
        Some(Align::Zero)
    } else {
        None
    };
    Ok(apply_width(body, width, align, ' ', false, true, prefix_len_for_zero))
}

fn signed_prefix_len(body: &str) -> usize {
    let mut n = 0;
    if body.starts_with('-') || body.starts_with('+') || body.starts_with(' ') {
        n += 1;
    }
    if body[n..].starts_with("0x") || body[n..].starts_with("0X") {
        n += 2;
    }
    n
}

/// Decimal-exact fixed-point formatting for `%f`/`format` `f`:
/// correctly rounded half-even from the exact binary value.
pub fn format_f_fixed(f: f64, prec: usize, upper: bool) -> String {
    let _ = upper;
    if f == 0.0 {
        let sign = if f.is_sign_negative() { "-" } else { "" };
        if prec == 0 {
            return format!("{}0", sign);
        }
        let mut s = format!("{}0.", sign);
        s.push_str(&"0".repeat(prec));
        return s;
    }
    // Use rounding to decimal string at prec digits after point via big-rational.
    let rounded = round_to_precision(f, prec as i64);
    // `rounded` is a decimal string without sign applied after.
    // We formatted magnitude; prepend sign.
    let neg = f < 0.0;
    let mag = rounded;
    // Ensure decimal point and digits
    let _ = upper;
    if neg && !mag.starts_with('-') {
        format!("-{}", mag)
    } else {
        mag
    }
}

/// Fixed notation via big-rationals (shared by %f and format f).
/// Rounds the exact binary value to `prec` digits after the point,
/// half-even, returning a plain decimal string (no sign handling for -0).
fn round_to_precision(f: f64, prec: i64) -> String {
    // Handle special handled by callers.
    if !f.is_finite() {
        return float_repr(f);
    }
    let neg = f < 0.0 || (f == 0.0 && f.is_sign_negative());
    let a = f64_abs(f);
    // a = m * 2^e
    let bits = a.to_bits();
    let raw_exp = ((bits >> 52) & 0x7ff) as i32;
    let mant_bits = bits & 0x000f_ffff_ffff_ffff;
    let (m_int, e): (BigInt, i32) = if raw_exp == 0 {
        (BigInt::from(mant_bits), -1074)
    } else {
        (BigInt::from((1u64 << 52) | mant_bits), raw_exp - 1023 - 52)
    };
    // scaled = a * 10^prec = m_int * 2^e * 10^prec
    // round half-even to integer, then insert point.
    let (num, den) = if e >= 0 {
        if prec >= 0 {
            let pow10 = BigInt::from(10i64).pow(u32::try_from(prec).unwrap());
            (((m_int.clone() << (e as usize)) * pow10), BigInt::one())
        } else {
            let pow10 = BigInt::from(10i64).pow(u32::try_from(-prec).unwrap());
            ((m_int.clone() << (e as usize)), pow10)
        }
    } else {
        let shift = (-e) as usize;
        if prec >= 0 {
            let pow10 = BigInt::from(10i64).pow(u32::try_from(prec).unwrap());
            ((m_int * pow10), (BigInt::one() << shift))
        } else {
            let pow10 = BigInt::from(10i64).pow(u32::try_from(-prec).unwrap());
            (m_int, (BigInt::one() << shift) * pow10)
        }
    };
    let q = round_half_even_div(&num, &den);
    // decimal string with `prec` digits after point
    let mut ds = q.to_string();
    if prec > 0 {
        let p = prec as usize;
        if ds.len() <= p {
            ds = format!("{}{}", "0".repeat(p - ds.len() + 1), ds);
        }
        let split = ds.len() - p;
        ds = format!("{}.{}", &ds[..split], &ds[split..]);
    } else if prec < 0 {
        ds = format!("{}{}", ds, "0".repeat((-prec) as usize));
    }
    if neg {
        format!("-{}", ds)
    } else {
        ds
    }
}

/// `round_half_even_div(num, den)` for non-negative `num`.
fn round_half_even_div(num: &BigInt, den: &BigInt) -> BigInt {
    let (q, r) = num.div_rem(den);
    let r2 = r * &BigInt::from(2i64);
    match r2.cmp(den) {
        Ordering::Less => q.clone(),
        Ordering::Greater => q.clone() + BigInt::one(),
        Ordering::Equal => {
            if (&q % &BigInt::from(2i64)).is_one() {
                q.clone() + BigInt::one()
            } else {
                q.clone()
            }
        }
    }
}


fn format_f_exp(f: f64, prec: usize, upper: bool) -> String {
    // CPython %e: one digit before point, prec after, exponent at least 2 digits.
    let neg = f < 0.0 || (f == 0.0 && f.is_sign_negative());
    let a = f64_abs(f);
    if a == 0.0 {
        let mant = if prec == 0 {
            "0".to_string()
        } else {
            format!("0.{}", "0".repeat(prec))
        };
        let s = format!("{}e+00", mant);
        let s = if upper { s.replace('e', "E") } else { s };
        return if neg { format!("-{}", s) } else { s };
    }
    // Exact: find e10 with 10^e10 <= a < 10^(e10+1), then round a * 10^(prec - e10).
    let mut exp10 = exact_exp10(a);
    let r = Rat::from_f64(a);
    let scale: i64 = prec as i64 - exp10 as i64;
    // scaled = a * 10^scale rounded to integer.
    let (num, den) = if scale >= 0 {
        (r.num.clone() * BigInt::from(10i64).pow(scale as u32), r.den.clone())
    } else {
        (r.num.clone(), r.den.clone() * BigInt::from(10i64).pow((-scale) as u32))
    };
    let mut q = round_half_even_div(&num, &den);
    // q should be in [10^prec, 10^(prec+1)); if rounding pushed it to
    // 10^(prec+1) (e.g. 9.99 -> 10.00), divide by 10 and bump exponent.
    let pow_prec_plus1 = BigInt::from(10i64).pow((prec as u32) + 1);
    if q >= pow_prec_plus1 {
        q /= BigInt::from(10i64);
        exp10 += 1;
    }
    let mut ds = q.to_string();
    // Pad to prec+1 digits (e.g. a == 1.0, prec == 6 -> "1000000").
    let need = prec + 1;
    while ds.len() < need {
        ds = format!("0{}", ds);
    }
    // Drop any extra digits beyond prec+1 (should not happen after carry fix).
    while ds.len() > need {
        ds.pop();
    }
    let mant_s = if prec == 0 {
        ds
    } else {
        format!("{}.{}", &ds[..1], &ds[1..])
    };
    let exp = exp10;
    let sign = if exp >= 0 { "+" } else { "-" };
    let es = format!("{:02}", exp.abs());
    let e = if upper { 'E' } else { 'e' };
    let s = format!("{}{}{}{}", mant_s, e, sign, es);
    if neg {
        format!("-{}", s)
    } else {
        s
    }
}
fn exact_exp10(a: f64) -> i32 {
    // a > 0 finite. Estimate with libm log10 (allowed: it only picks the
    // starting point), then correct with exact rational comparisons.
    let mut e10 = libm::floor(libm::log10(a)) as i32;
    let r = Rat::from_f64(a);
    // Compare r against 10^k exactly: r.num/r.den ? 10^k.
    let cmp_pow10 = |k: i32| -> Ordering {
        if k >= 0 {
            let p = BigInt::from(10i64).pow(k as u32);
            // r - 10^k = (r.num - p*r.den)/r.den
            (r.num.clone() - p * r.den.clone()).cmp(&BigInt::zero())
        } else {
            let p = BigInt::from(10i64).pow((-k) as u32);
            // r - 10^k = (r.num*p - r.den)/ (r.den*p)
            (r.num.clone() * p - r.den.clone()).cmp(&BigInt::zero())
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


/// `%g` formatting: choose fixed vs exp using the precision rules
/// (CPython `format.c:format_float_single` with `g`).
fn format_f_general(f: f64, prec: usize, upper: bool, keep_trailing: bool) -> String {
    // Strip trailing zeros unless keep_trailing ('#').
    if f == 0.0 {
        let sign = if f.is_sign_negative() { "-" } else { "" };
        if prec == 0 {
            return if keep_trailing { format!("{}0.", sign) } else { format!("{}0", sign) };
        }
        // "0", "0.0", etc with prec digits of significance
        let s = if keep_trailing {
            if prec <= 1 {
                format!("{}0.", sign)
            } else {
                format!("{}0.{}", sign, "0".repeat(prec - 1))
            }
        } else {
            format!("{}0", sign)
        };
        return s;
    }
    let a = f64_abs(f);
    let e10 = exact_exp10(a);
    // CPython rule: use exp if e10 < -4 or e10 >= prec (prec default 6).
    let p = if prec == 0 { 1 } else { prec as i32 };
    let use_exp = e10 < -4 || e10 >= p;
    if use_exp {
        let mut s = format_f_exp(f, (p - 1).max(0) as usize, upper);
        if !keep_trailing {
            s = strip_trailing_zeros_exp(s);
        }
        s
    } else {
        // fixed with p-1-e10 digits after point
        let digits_after = (p - 1 - e10).max(0) as usize;
        let mut s = format_f_fixed(f, digits_after, upper);
        if !keep_trailing && s.contains('.') {
            while s.ends_with('0') {
                s.pop();
            }
            if s.ends_with('.') {
                s.pop();
            }
        }
        s
    }
}

fn strip_trailing_zeros_exp(s: String) -> String {
    // e.g. "1.500e+05" -> "1.5e+05"
    if let Some(ep) = s.find(['e', 'E']) {
        let (m, ex) = s.split_at(ep);
        let mut m = m.to_string();
        if m.contains('.') {
            while m.ends_with('0') {
                m.pop();
            }
            if m.ends_with('.') {
                m.pop();
            }
        }
        format!("{}{}", m, ex)
    } else {
        s
    }
}

// ---------------------------------------------------------------------------
// Public %-formatting entry points.
// ---------------------------------------------------------------------------

/// `fmt % values` (tuple form) reproducing CPython `str.__mod__`
/// (`Objects/stringlib/codecs.h` + `format.c` for the `%` operator).
/// `args` is the argument tuple; a single-element tuple or mapping form
/// is supported via [`percent_format_map`]. Each public behaviour cites
/// CPython as documented in the task.
///
/// Returns the formatted string or the CPython exception text.
/// `PyError.kind` is `TypeError`/`ValueError`/`OverflowError` as CPython.
pub fn percent_format(fmt: &str, args: &[PctValue]) -> Result<String, PyError> {
    let parts = parse_pct_fmt(fmt).map_err(|m| PyError::new("ValueError", m))?;
    // Validate stars consume args first (CPython consumes in order: width, prec, value).
    let mut arg_idx = 0usize;
    let mut out = String::new();
    // Check for end marker
    let has_end = parts.last().map(|(_, s)| s.is_end()).unwrap_or(false);
    if !has_end {
        return Err(PyError::new("ValueError", "incomplete format".to_string()));
    }
    let n_parts = parts.len() - 1;
    for (lit, spec) in &parts[..n_parts] {
        out.push_str(lit);
        // resolve stars
        let mut width = spec.width;
        let mut prec = spec.precision;
        let mut star_width_neg = false;
        if spec.star_width {
            let v = args.get(arg_idx);
            match v {
                Some(PctValue::Int(n)) => {
                    let w = n.0.to_i64().ok_or_else(|| {
                        PyError::new("OverflowError", "width out of range".to_string())
                    })?;
                    if w < 0 {
                        // negative width => left align
                        star_width_neg = true;
                        width = Some((-w) as usize);
                    } else {
                        width = Some(w as usize);
                    }
                    arg_idx += 1;
                }
                _ => {
                    return Err(PyError::new("TypeError", "* wants int".to_string()));
                }
            }
        }
        if spec.star_prec {
            let v = args.get(arg_idx);
            match v {
                Some(PctValue::Int(n)) => {
                    let p = n.0.to_i64().ok_or_else(|| {
                        PyError::new("OverflowError", "precision out of range".to_string())
                    })?;
                    if p < 0 {
                        prec = None; // CPython treats as omitted
                    } else {
                        prec = Some(p as usize);
                    }
                    arg_idx += 1;
                }
                _ => {
                    return Err(PyError::new("TypeError", "* wants int".to_string()));
                }
            }
        }
        // resolve value
        let val = if spec.key.is_some() {
            // mapping form requires map; plain tuple form errors
            return Err(PyError::new("TypeError", "format requires a mapping".to_string()));
        } else {
            let v = args.get(arg_idx);
            match v {
                Some(x) => {
                    arg_idx += 1;
                    x
                }
                None => {
                    return Err(PyError::new("TypeError", "not enough arguments for format string".to_string()));
                }
            }
        };
        out.push_str(&format_one(spec, val, width, prec, star_width_neg)?);
    }
    out.push_str(&parts[n_parts].0);
    if arg_idx < args.len() {
        return Err(PyError::new("TypeError", "not all arguments converted during string formatting".to_string()));
    }
    Ok(out)
}

/// Mapping form `fmt % {name: value}` (CPython `str.__mod__` with dict).
pub fn percent_format_map(fmt: &str, map: &[(String, PctValue)]) -> Result<String, PyError> {
    let parts = parse_pct_fmt(fmt).map_err(|m| PyError::new("ValueError", m))?;
    let mut out = String::new();
    for (lit, spec) in &parts {
        out.push_str(lit);
        if spec.is_end() {
            continue;
        }
        let key = spec.key.clone().ok_or_else(|| {
            PyError::new("TypeError", "format requires a mapping".to_string())
        })?;
        let (_, v) = map
            .iter()
            .find(|(k, _)| *k == key)
            .ok_or_else(|| PyError::new("KeyError", crate::value::repr_key(&key).to_string()))?;
        let _ = key;
        out.push_str(&format_one(spec, v, spec.width, spec.precision, false)?);
    }
    Ok(out)
}

fn format_one(
    spec: &PctSpec,
    v: &PctValue,
    width: Option<usize>,
    prec: Option<usize>,
    star_width_neg: bool,
) -> Result<String, PyError> {
    let flags = &spec.flags;
    // `%`-formatting right-aligns every conversion; `-` or a negative `*`
    // width left-aligns (the `str.format` default of left for strings does
    // not apply here).
    let text_align = if flags.contains('-') || star_width_neg { Align::Left } else { Align::Right };
    match spec.conv {
        's' => {
            let mut s = v.as_str();
            if let Some(p) = prec {
                s = s.chars().take(p).collect();
            }
            Ok(apply_width(s, width, Some(text_align), ' ', false, false, 0))
        }
        'r' => {
            let mut s = v.as_repr();
            if let Some(p) = prec {
                s = s.chars().take(p).collect();
            }
            Ok(apply_width(s, width, Some(text_align), ' ', false, false, 0))
        }
        'a' => {
            // %a = ascii(repr)
            let raw = v.as_repr();
            let ascii = raw
                .chars()
                .flat_map(|c| {
                    if c.is_ascii() {
                        vec![c]
                    } else {
                        let o = c as u32;
                        if o < 0x100 {
                            format!("\\x{:02x}", o).chars().collect::<Vec<_>>()
                        } else if o < 0x10000 {
                            format!("\\u{:04x}", o).chars().collect::<Vec<_>>()
                        } else {
                            format!("\\U{:08x}", o).chars().collect::<Vec<_>>()
                        }
                    }
                })
                .collect::<String>();
            let mut s = ascii;
            if let Some(p) = prec {
                s = s.chars().take(p).collect();
            }
            Ok(apply_width(s, width, Some(text_align), ' ', false, false, 0))
        }
        'd' | 'i' | 'u' | 'x' | 'X' | 'o' | 'c' => {
            format_int_conv(v, spec.conv, flags, width, prec, star_width_neg)
        }
        'f' | 'F' | 'e' | 'E' | 'g' | 'G' => {
            format_float_conv(v, spec.conv, flags, width, prec)
        }
        _ => Err(PyError::new(
            "ValueError",
            format!("unsupported format character '{}'", spec.conv),
        )),
    }
}

// ---------------------------------------------------------------------------
// format() mini-language.
// ---------------------------------------------------------------------------

/// Parse and apply a `format()` spec for a scalar value
/// (`"{}" .format(x)` / `format(x, spec)`), types per `Objects/stringlib/
/// format.c:format_float_single` and `format_int`.
pub fn format_value(v: &PctValue, spec: &str) -> Result<String, PyError> {
    if spec.is_empty() {
        return Ok(v.as_str());
    }
    // [[fill]align][sign][z][#][0][width][grouping][,|.precision][type]
    let chars: Vec<char> = spec.chars().collect();
    let mut idx = 0;
    let mut fill = ' ';
    let mut align: Option<Align> = None;
    if chars.len() >= 2 && matches!(chars[1], '<' | '>' | '^' | '=') {
        fill = chars[0];
        align = Some(match chars[1] {
            '<' => Align::Left,
            '>' => Align::Right,
            '^' => Align::Center,
            '=' => Align::Zero,
            _ => unreachable!(),
        });
        idx = 2;
    } else if !chars.is_empty() && matches!(chars[0], '<' | '>' | '^' | '=') {
        align = Some(match chars[0] {
            '<' => Align::Left,
            '>' => Align::Right,
            '^' => Align::Center,
            '=' => Align::Zero,
            _ => unreachable!(),
        });
        idx = 1;
    }
    // sign
    let mut sign_opt: Option<char> = None;
    if idx < chars.len() && matches!(chars[idx], '+' | '-' | ' ') {
        sign_opt = Some(chars[idx]);
        idx += 1;
    }
    // z (Python 3.10+; treat as compat no-op for 3.8: error if present?)
    if idx < chars.len() && chars[idx] == 'z' {
        // CPython 3.8 has no 'z'; unknown spec char -> ValueError
        return Err(PyError::new("ValueError", format!("Unknown format code 'z' for object of type '{}'", type_name(v))));
    }
    let mut alt = false;
    if idx < chars.len() && chars[idx] == '#' {
        alt = true;
        idx += 1;
    }
    let mut zero_flag = false;
    if idx < chars.len() && chars[idx] == '0' {
        zero_flag = true;
        idx += 1;
    }
    // width
    let mut width: Option<usize> = None;
    let mut w = String::new();
    while idx < chars.len() && chars[idx].is_ascii_digit() {
        w.push(chars[idx]);
        idx += 1;
    }
    if !w.is_empty() {
        width = Some(w.parse().map_err(|_| PyError::new("ValueError", "invalid width".to_string()))?);
    }
    // grouping
    let mut grouping = false;
    if idx < chars.len() && (chars[idx] == ',' || chars[idx] == '_') {
        grouping = true;
        idx += 1;
    }
    // precision
    let mut prec: Option<usize> = None;
    if idx < chars.len() && chars[idx] == '.' {
        idx += 1;
        let mut p = String::new();
        while idx < chars.len() && chars[idx].is_ascii_digit() {
            p.push(chars[idx]);
            idx += 1;
        }
        if p.is_empty() {
            prec = Some(0);
        } else {
            prec = Some(p.parse().unwrap_or(0));
        }
    }
    // type
    let mut conv: Option<char> = None;
    if idx < chars.len() {
        conv = Some(chars[idx]);
        idx += 1;
    }
    if idx != chars.len() {
        return Err(PyError::new("ValueError", format!("Invalid format specifier '{}'", spec)));
    }
    let conv = conv.unwrap_or('\0');
    format_value_inner(v, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill)
}
#[allow(clippy::too_many_arguments)]
fn format_value_inner(
    v: &PctValue,
    conv: char,
    sign_opt: Option<char>,
    alt: bool,
    zero_flag: bool,
    width: Option<usize>,
    grouping: bool,
    prec: Option<usize>,
    align: Option<Align>,
    fill: char,
) -> Result<String, PyError> {
    // Match CPython type errors:
    // - "s" only for str-like; "d" etc need numbers.
    // No conversion (default): use repr/str per type (str for numbers is float_str)
    if conv == '\0' {
        match v {
            PctValue::Str(_) => {
                let mut s = v.as_str();
                if let Some(p) = prec {
                    s = s.chars().take(p).collect();
                }
                return Ok(apply_width(s, width, align, fill, zero_flag, false, 0));
            }
            PctValue::Bytes(_) => {
                let mut s = bytes_repr_bytes_for_format(v);
                if let Some(p) = prec {
                    s = s.chars().take(p).collect();
                }
                return Ok(apply_width(s, width, align, fill, zero_flag, false, 0));
            }
            _ => {
                // numbers: format like str with fill/align rules
            }
        }
    }
    match v {
        PctValue::Int(n) => format_int_minilang(n, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill),
        PctValue::Bool(b) => {
            // bool formats as int for numeric types
            let n = PyInt::from_i64(if *b { 1 } else { 0 });
            format_int_minilang(&n, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill)
        }
        PctValue::Float(f) => format_float_minilang(*f, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill),
        PctValue::Str(s) => {
            if conv != '\0' && conv != 's' {
                return Err(PyError::new(
                    "ValueError",
                    format!("Unknown format code '{}' for object of type 'str'", conv),
                ));
            }
            let mut t = s.clone();
            if let Some(p) = prec {
                t = t.chars().take(p).collect();
            }
            let a = align.unwrap_or(Align::Left);
            Ok(pad(&t, width, a, fill, 0))
        }
        PctValue::Bytes(_) => {
            if conv != '\0' && conv != 's' {
                return Err(PyError::new(
                    "ValueError",
                    format!("Unknown format code '{}' for object of type 'bytes'", conv),
                ));
            }
            let t = bytes_repr_bytes_for_format(v);
            let a = align.unwrap_or(Align::Left);
            Ok(pad(&t, width, a, fill, 0))
        }
    }
}

fn bytes_repr_bytes_for_format(v: &PctValue) -> String {
    match v {
        PctValue::Bytes(b) => bytes_repr(b),
        _ => v.as_str(),
    }
}
#[allow(clippy::too_many_arguments)]
fn format_int_minilang(
    n: &PyInt,
    conv: char,
    sign_opt: Option<char>,
    alt: bool,
    zero_flag: bool,
    width: Option<usize>,
    grouping: bool,
    prec: Option<usize>,
    align: Option<Align>,
    fill: char,
) -> Result<String, PyError> {
    let radix = match conv {
        '\0' | 'd' | 'n' => 10,
        'b' => 2,
        'x' | 'X' => 16,
        'o' => 8,
        'c' => {
            // chr
            let code = n.0.to_i64().ok_or_else(|| {
                PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string())
            })?;
            if !(0..=0x10FFFF).contains(&code) {
                return Err(PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string()));
            }
            let c = char::from_u32(code as u32).ok_or_else(|| {
                PyError::new("OverflowError", "chr() arg not in range(0x110000)".to_string())
            })?;
            let s = c.to_string();
            let a = align.unwrap_or(Align::Left);
            return Ok(pad(&s, width, a, fill, 0));
        }
        'e' | 'E' | 'f' | 'F' | 'g' | 'G' | '%' | 's' => {
            // int accepts float presentation via float conversion
            let f = n.0.to_f64().ok_or_else(|| {
                PyError::new("OverflowError", "int too large to convert to float".to_string())
            })?;
            return format_float_minilang(f, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill);
        }
        _ => {
            return Err(PyError::new(
                "ValueError",
                format!("Unknown format code '{}' for object of type 'int'", conv),
            ));
        }
    };
    let upper = conv == 'X';
    let negative = n.0.sign() == num_bigint::Sign::Minus;
    let mut digits = bigint_abs_bigstr(n.inner(), radix, upper);
    if let Some(p) = prec {
        if p == 0 && digits == "0" {
            digits = String::new();
        } else if digits.len() < p {
            digits = format!("{}{}", "0".repeat(p - digits.len()), digits);
        }
    }
    if grouping && (radix == 10) {
        digits = group_thousands(&digits);
    }
    let mut prefix = String::new();
    if negative {
        prefix.push('-');
    } else {
        match sign_opt {
            Some('+') => prefix.push('+'),
            Some(' ') => prefix.push(' '),
            _ => {}
        }
    }
    if alt {
        match conv {
            'b' => prefix.push_str("0b"),
            'o' => {
                if digits != "0" && !digits.starts_with('0') {
                    digits = format!("0{}", digits);
                }
            }
            'x' => prefix.push_str("0x"),
            'X' => prefix.push_str("0X"),
            _ => {}
        }
    }
    let body = format!("{}{}", prefix, digits);
    let sign_len = prefix.len();
    // zero_flag implies '=' align when no align given and no precision
    let a = if align.is_some() {
        align
    } else if zero_flag && prec.is_none() {
        Some(Align::Zero)
    } else {
        Some(Align::Right)
    };
    Ok(apply_width(body, width, a, fill, false, true, sign_len))
}

fn group_thousands(digits: &str) -> String {
    let mut out = String::new();
    for (i, c) in digits.chars().rev().enumerate() {
        if i > 0 && i % 3 == 0 {
            out.push(',');
        }
        out.push(c);
    }
    out.chars().rev().collect()
}
#[allow(clippy::too_many_arguments)]
fn format_float_minilang(
    f: f64,
    conv: char,
    sign_opt: Option<char>,
    alt: bool,
    zero_flag: bool,
    width: Option<usize>,
    grouping: bool,
    prec: Option<usize>,
    align: Option<Align>,
    fill: char,
) -> Result<String, PyError> {
    // NaN/Inf: precision/grouping ignored, sign rules apply.
    if f.is_nan() || f.is_infinite() {
        let mut s = if f.is_nan() {
            if conv == 'F' || conv == 'E' || conv == 'G' { "NAN" } else { "nan" }
        } else if f > 0.0 {
            if matches!(conv, 'F' | 'E' | 'G') { "INF" } else { "inf" }
        } else if matches!(conv, 'F' | 'E' | 'G') {
            "-INF"
        } else {
            "-inf"
        }
        .to_string();
        if f.is_infinite() && f > 0.0 {
            match sign_opt {
                Some('+') => s = format!("+{}", s),
                Some(' ') => s = format!(" {}", s),
                _ => {}
            }
        }
        let a = align.unwrap_or(Align::Right);
        return Ok(pad(&s, width, a, fill, 0));
    }
    if grouping {
        // CPython ',' grouping for floats: implemented for f/e/g.
        let inner = format_float_minilang(f, conv, sign_opt, alt, false, None, false, prec, None, ' ')?;
        // insert commas in integer part
        let s = apply_grouping_float(&inner);
        let a = align.unwrap_or(Align::Right);
        return Ok(pad(&s, width, a, fill, signed_prefix_len(&s)));
    }
    let mut body: String;
    match conv {
        '\0' => {
            body = float_repr(f);
        }
        'e' | 'E' => body = format_f_exp(f, prec.unwrap_or(6), conv == 'E'),
        'f' | 'F' => body = format_f_fixed(f, prec.unwrap_or(6), conv == 'F'),
        'g' | 'G' => body = format_f_general(f, prec.unwrap_or(6), conv == 'G', alt),
        '%' => {
            let v = f * 100.0;
            let mut s = format_f_fixed(v, prec.unwrap_or(6), false);
            s.push('%');
            body = s;
        }
        's' => {
            body = float_repr(f);
        }
        'd' | 'b' | 'o' | 'x' | 'X' | 'c' | 'n' => {
            let n = trunc_float_to_bigint(f);
            return format_int_minilang(&n, conv, sign_opt, alt, zero_flag, width, grouping, prec, align, fill);
        }
        _ => {
            return Err(PyError::new(
                "ValueError",
                format!("Unknown format code '{}' for object of type 'float'", conv),
            ));
        }
    }
    if alt && (conv == 'e' || conv == 'E' || conv == 'g' || conv == 'G') && !body.contains('.') && !body.contains(['e', 'E']) {
        // keep point (already handled in general)
    }
    // sign options
    if !body.starts_with('-') {
        match sign_opt {
            Some('+') => body = format!("+{}", body),
            Some(' ') => body = format!(" {}", body),
            _ => {}
        }
    }
    let sign_len = signed_prefix_len(&body);
    let a = if align.is_some() {
        align
    } else if zero_flag {
        Some(Align::Zero)
    } else {
        Some(Align::Right)
    };
    Ok(apply_width(body, width, a, fill, false, true, sign_len))
}

fn apply_grouping_float(s: &str) -> String {
    // Add ',' to integer part, '_' handling same for simplicity.
    let (sign, rest) = if let Some(r) = s.strip_prefix('-') {
        ("-", r)
    } else if s.starts_with('+') || s.starts_with(' ') {
        (&s[..1], &s[1..])
    } else {
        ("", s)
    };
    // split exponent
    let (mant, exp) = match rest.find(['e', 'E']) {
        Some(p) => (&rest[..p], &rest[p..]),
        None => (rest, ""),
    };
    let (intp, frac) = match mant.find('.') {
        Some(p) => (&mant[..p], &mant[p..]),
        None => (mant, ""),
    };
    // strip trailing '%'
    let (intp2, pct) = if let Some(st) = intp.strip_suffix('%') {
        (st, "%")
    } else {
        (intp, "")
    };
    format!("{}{}{}{}{}{}", sign, group_thousands(intp2), frac, exp, pct, "")
}

/// Python `format` template `"...".format(...)`: only `{}` `{0}` `{name}`
/// and `{{` `}}` are needed by klippy/Jinja templates in this crate;
/// format specs after `:` are delegated to [`format_value`].
/// Returns error texts matching CPython `str.format` (`StringIO` errors).
pub fn str_format(template: &str, args: &[PctValue], kwargs: &[(String, PctValue)]) -> Result<String, PyError> {
    let chars: Vec<char> = template.chars().collect();
    let mut out = String::new();
    let mut i = 0;
    let mut auto_idx = 0usize;
    while i < chars.len() {
        match chars[i] {
            '{' => {
                if i + 1 < chars.len() && chars[i + 1] == '{' {
                    out.push('{');
                    i += 2;
                    continue;
                }
                // find closing '}'
                let mut depth = 1i32;
                let mut j = i + 1;
                while j < chars.len() && depth > 0 {
                    if chars[j] == '{' {
                        depth += 1;
                    } else if chars[j] == '}' {
                        depth -= 1;
                    }
                    j += 1;
                }
                if depth != 0 {
                    return Err(PyError::new("ValueError", "Single '{' encountered in format string".to_string()));
                }
                let field = &template[i + 1..j - 1];
                // field: [arg_name][:spec]
                let (name, spec) = match field.find(':') {
                    Some(p) => (&field[..p], &field[p + 1..]),
                    None => (field, ""),
                };
                // attribute/index access `[.]` etc are not used by klippy
                // templates through this helper; treat as part of name lookup
                let name = name.trim();
                let val: &PctValue = if name.is_empty() {
                    let v = args.get(auto_idx).ok_or_else(|| {
                        PyError::new("IndexError", format!("Replacement index {} out of range for positional args tuple", auto_idx))
                    })?;
                    auto_idx += 1;
                    v
                } else if let Ok(idxp) = name.parse::<usize>() {
                    if name.chars().all(|c| c.is_ascii_digit()) {
                        args.get(idxp).ok_or_else(|| {
                            PyError::new("IndexError", "tuple index out of range".to_string())
                        })?
                    } else {
                        // keyword
                        kwargs
                            .iter()
                            .find(|(k, _)| k == name)
                            .map(|(_, v)| v)
                            .ok_or_else(|| PyError::new("KeyError", crate::value::repr_key(name).to_string()))?
                    }
                } else {
                    // allow "0.attr"/"0[idx]" by taking the base before '.'/'['
                    let base = name
                        .split(['.', '['])
                        .next()
                        .unwrap_or("");
                    if base.is_empty() {
                        // shouldn't happen
                        args.get(auto_idx).ok_or_else(|| {
                            PyError::new("IndexError", "Replacement index out of range".to_string())
                        })?
                    } else if let Ok(num) = base.parse::<usize>() {
                        let _ = num;
                        args.get(num).ok_or_else(|| {
                            PyError::new("IndexError", "tuple index out of range".to_string())
                        })?
                    } else {
                        kwargs
                            .iter()
                            .find(|(k, _)| k == name)
                            .map(|(_, v)| v)
                            .ok_or_else(|| PyError::new("KeyError", crate::value::repr_key(name).to_string()))?
                    }
                };
                // spec may contain nested fields; CPython resolves them;
                // klippy templates only use static specs ({:.6f}, etc.).
                let spec_resolved = if spec.contains('{') {
                    str_format(spec, args, kwargs)?
                } else {
                    spec.to_string()
                };
                let text = if spec_resolved.is_empty() { val.as_str() } else { format_value(val, &spec_resolved)? };
                out.push_str(&text);
                i = j;
            }
            '}' => {
                if i + 1 < chars.len() && chars[i + 1] == '}' {
                    out.push('}');
                    i += 2;
                    continue;
                }
                return Err(PyError::new("ValueError", "Single '}' encountered in format string".to_string()));
            }
            c => {
                out.push(c);
                i += 1;
            }
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// round
// ---------------------------------------------------------------------------

/// `round(x)` to nearest integer, half-even, returning a `PyInt`
/// (CPython `Objects/floatobject.c:float_round` without `ndigits`).
/// `round(2.5) == 2`, `round(3.5) == 4`; `round(x)` on inf raises
/// `OverflowError: cannot convert float infinity to integer` and on nan
/// raises `ValueError: cannot convert float NaN to integer`? Actually
/// `round(inf)` raises `OverflowError: cannot convert float infinity
/// to integer` and `round(nan)` raises `ValueError: cannot convert float
/// NaN to integer` — matched here.
pub fn round_py(x: f64) -> Result<PyInt, PyError> {
    if x.is_nan() {
        return Err(PyError::new("ValueError", "cannot convert float NaN to integer".to_string()));
    }
    if x.is_infinite() {
        return Err(PyError::new("OverflowError", "cannot convert float infinity to integer".to_string()));
    }
    if x == 0.0 {
        // preserve -0.0 -> 0
        return Ok(PyInt::from_i64(0));
    }
    Ok(round_exact_half_even(x))
}

/// `round(x, ndigits)` correctly rounded half-even to decimal places
/// (CPython `Objects/floatobject.c:float_round` with `ndigits`).
/// Returns the nearest `f64` to the decimal rounding of the exact
/// binary value. Inf/nan pass through; huge `ndigits` (>= 324) is
/// identity within float resolution.
pub fn round_py_nd(x: f64, ndigits: i64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    if ndigits >= 324 {
        // No-op: decimal rounding beyond float resolution.
        // CPython still returns x (possibly with -0.0 handled below).
        if x == 0.0 {
            return x;
        }
        return x;
    }
    if ndigits < -324 {
        // rounds to 0.0
        return if x.is_sign_negative() { -0.0 } else { 0.0 };
    }
    if x == 0.0 {
        return x;
    }
    let s = round_to_precision(x, ndigits);
    // Parse back; Rust parse is correctly rounded (matches CPython strtod).
    s.parse::<f64>().unwrap_or(x)
}


/// Exact half-even to integer via the exact binary value.
fn round_exact_half_even(x: f64) -> PyInt {
    // x = m * 2^e exactly.
    let neg = x < 0.0;
    let a = f64_abs(x);
    let bits = a.to_bits();
    let raw_exp = ((bits >> 52) & 0x7ff) as i32;
    if raw_exp == 0 {
        return PyInt::from_i64(0);
    }
    let mant_bits = bits & 0x000f_ffff_ffff_ffff;
    let m_int: BigInt = BigInt::from((1u64 << 52) | mant_bits);
    let e: i32 = raw_exp - 1023 - 52;
    if e >= 0 {
        let v = m_int << (e as usize);
        return if neg { PyInt(-v) } else { PyInt(v) };
    }
    let shift = (-e) as usize;
    let den = BigInt::one() << shift;
    let q = round_half_even_div(&m_int, &den);
    if neg {
        PyInt(-q)
    } else {
        PyInt(q)
    }
}

// ---------------------------------------------------------------------------
// Parsing: float(str), int(str[, base])
// ---------------------------------------------------------------------------

/// `float(str)` per CPython `Objects/floatobject.c:float_from_string`
/// (`Python/pystrtod.c:PyOS_string_to_double`): strips `White_Space`
/// (U+001C..U+001F are not stripped),
/// decimal digits with single underscores between digits
/// (`"1_0"` ok), exponent `e/E`. On failure:
/// `ValueError: could not convert string to float: 'x'` (repr of arg),
/// `TypeError: float() argument must be...` — this helper takes `&str`
/// so only the ValueError path applies.
pub fn float_from_string(s: &str) -> Result<f64, PyError> {
    let t = trim_py_space(s);
    if t.is_empty() {
        return Err(value_float_error(s));
    }
    if let Some(v) = parse_special_float(t) {
        return Ok(v);
    }
    if !valid_decimal_float_syntax(t) {
        return Err(value_float_error(s));
    }
    // Remove underscores for Rust parse (syntax already validated).
    let cleaned: String = t.chars().filter(|&c| c != '_').collect();
    match cleaned.parse::<f64>() {
        Ok(v) => {
            // CPython rejects e.g. "1e" already via syntax; Rust would too.
            // Also reject things like "0x1p3" — syntax check covers it.
            Ok(v)
        }
        Err(_) => Err(value_float_error(s)),
    }
}

fn value_float_error(s: &str) -> PyError {
    PyError::new(
        "ValueError",
        format!("could not convert string to float: {}", str_repr(s)),
    )
}

fn trim_py_space(s: &str) -> &str {
    // `int()`/`float()` strip `White_Space` but not U+001C..U+001F (verified
    // against the oracle: `int('\x1c42')` raises), so plain
    // `char::is_whitespace` is the exact set here, unlike `str.strip()`.
    s.trim_matches(|c: char| c.is_whitespace())
}

fn parse_special_float(t: &str) -> Option<f64> {
    let lower = t.to_ascii_lowercase();
    let (sign, rest) = if let Some(r) = lower.strip_prefix('+') {
        (1.0, r)
    } else if let Some(r) = lower.strip_prefix('-') {
        (-1.0, r)
    } else {
        (1.0, lower.as_str())
    };
    match rest {
        "inf" | "infinity" => Some(sign * f64::INFINITY),
        "nan" => Some(sign * f64::NAN),
        _ => None,
    }
}

/// Validate digits/underscores/exponent grammar.
fn valid_decimal_float_syntax(t: &str) -> bool {
    // [sign] (digits [. [digits]] | . digits) ([eE] [sign] digits)?
    let b = t.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let mut digits_before = 0;
    let mut prev_digit = false;
    let mut saw_dot = false;
    let mut digits_after = 0;
    while i < b.len() {
        let c = b[i];
        if c.is_ascii_digit() {
            prev_digit = true;
            if saw_dot {
                digits_after += 1;
            } else {
                digits_before += 1;
            }
            i += 1;
        } else if c == b'_' {
            // underscore must be between digits
            if !prev_digit {
                return false;
            }
            // next must be digit (checked in next loop iteration via prev reset)
            // Must not be consecutive: require next to be digit.
            if i + 1 >= b.len() || !b[i + 1].is_ascii_digit() {
                return false;
            }
            prev_digit = false;
            i += 1;
        } else if c == b'.' {
            if saw_dot {
                return false;
            }
            saw_dot = true;
            // dot must be adjacent to digits on at least one side; handled by counts
            prev_digit = false;
            i += 1;
        } else {
            break;
        }
    }
    if digits_before == 0 && digits_after == 0 {
        return false;
    }
    if !not_prev_underscore(t) {
        return false;
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        i += 1;
        if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
            i += 1;
        }
        let mut exp_digits = 0;
        let mut exp_prev = false;
        while i < b.len() {
            let c = b[i];
            if c.is_ascii_digit() {
                exp_digits += 1;
                exp_prev = true;
                i += 1;
            } else if c == b'_' {
                if !exp_prev {
                    return false;
                }
                if i + 1 >= b.len() || !b[i + 1].is_ascii_digit() {
                    return false;
                }
                exp_prev = false;
                i += 1;
            } else {
                return false;
            }
        }
        if exp_digits == 0 {
            return false;
        }
    }
    i == b.len()
}

fn not_prev_underscore(_t: &str) -> bool {
    true
}

/// `int(str[, base])` per CPython `Objects/abstract.c:PyNumber_Long`
/// (`Objects/longobject.c`): strips whitespace, optional sign, then
/// digits in `base` (2..36); `base==0` detects `0x/0o/0b` or rejects
/// leading zeros (`"010"` with base 0 is invalid). Underscores allowed
/// between digits as for floats. Error:
/// `ValueError: invalid literal for int() with base 10: 'x'`.
pub fn int_from_string(s: &str, base: i64) -> Result<PyInt, PyError> {
    if !(0..=36).contains(&base) || base == 1 {
        // CPython: base must be 0 or 2..36
        if !(base == 0 || (2..=36).contains(&base)) {
            return Err(PyError::new(
                "ValueError",
                "int() base must be >= 2 and <= 36, or 0".to_string(),
            ));
        }
    }
    let t = trim_py_space(s);
    // sign
    let (neg, rest) = if let Some(r) = t.strip_prefix('-') {
        (true, r)
    } else if let Some(r) = t.strip_prefix('+') {
        (false, r)
    } else {
        (false, t)
    };
    if rest.is_empty() {
        return Err(int_error(s, base));
    }
    let (eff_base, digits_str) = if base == 0 {
        if rest.len() >= 2 && (rest.starts_with("0x") || rest.starts_with("0X")) {
            (16, &rest[2..])
        } else if rest.len() >= 2 && (rest.starts_with("0o") || rest.starts_with("0O")) {
            (8, &rest[2..])
        } else if rest.len() >= 2 && (rest.starts_with("0b") || rest.starts_with("0B")) {
            (2, &rest[2..])
        } else if rest.len() > 1 && rest.starts_with('0') {
            // CPython: int("010", 0) -> ValueError (leading zeros)
            return Err(int_error(s, base));
        } else {
            (10, rest)
        }
    } else {
        // explicit base also accepts prefix? CPython: int("0x10", 16) is ok?
        // CPython allows prefix with explicit base only if it matches?
        // In 3.8, int("0x10", 16) == 16, int("0x10", 10) fails.
        let strip = match base {
            16 if rest.starts_with("0x") || rest.starts_with("0X") => 2,
            8 if rest.starts_with("0o") || rest.starts_with("0O") => 2,
            2 if rest.starts_with("0b") || rest.starts_with("0B") => 2,
            _ => 0,
        };
        let stripped = if strip == 0 { rest } else { &rest[strip..] };
        (base, stripped)
    };
    if digits_str.is_empty() {
        return Err(int_error(s, base));
    }
    // validate digits with underscores
    let mut prev_digit = false;
    let mut any_digit = false;
    let mut cleaned = String::new();
    for c in digits_str.chars() {
        if c == '_' {
            if !prev_digit {
                return Err(int_error(s, base));
            }
            // lookahead required digit — enforced by prev reset + end check
            prev_digit = false;
        } else if c.is_ascii_alphanumeric() {
            let v = digit_val(c);
            match v {
                Some(d) if (d as i64) < eff_base => {
                    any_digit = true;
                    prev_digit = true;
                    cleaned.push(c);
                }
                _ => return Err(int_error(s, base)),
            }
        } else {
            return Err(int_error(s, base));
        }
    }
    if !any_digit || !prev_digit {
        // trailing underscore => prev_digit false
        return Err(int_error(s, base));
    }
    // build BigInt
    let digits_upper = cleaned.to_ascii_uppercase();
    let mut val = BigInt::zero();
    let rad = BigInt::from(eff_base);
    for c in digits_upper.chars() {
        let d = digit_val(c).unwrap() as i64;
        val = val * rad.clone() + BigInt::from(d);
    }
    if neg {
        val = -val;
    }
    Ok(PyInt(val))
}

fn digit_val(c: char) -> Option<u32> {
    match c {
        '0'..='9' => Some(c as u32 - '0' as u32),
        'a'..='z' => Some(c as u32 - 'a' as u32 + 10),
        'A'..='Z' => Some(c as u32 - 'A' as u32 + 10),
        _ => None,
    }
}

fn int_error(s: &str, base: i64) -> PyError {
    PyError::new(
        "ValueError",
        format!(
            "invalid literal for int() with base {}: {}",
            if base == 0 { 0 } else { base },
            str_repr(s)
        ),
    )
}

// ---------------------------------------------------------------------------
// Mixed int/float arithmetic helpers used by value.rs.
// ---------------------------------------------------------------------------

/// Exact comparison of a `BigInt` against an `f64` as rationals
/// (CPython `Objects/longobject.c:long_compare` vs float).
/// Returns `Ordering` of `bigint - float`.
pub fn cmp_bigint_float(a: &BigInt, b: f64) -> Ordering {
    if b.is_nan() {
        // CPython: comparisons with nan are False; equality uses
        // numeric Eq where nan != everything. Ordering used here only
        // when both are not nan (caller guards).
        return Ordering::Equal;
    }
    if b.is_infinite() {
        return if b > 0.0 { Ordering::Less } else { Ordering::Greater };
    }
    if b == 0.0 {
        if a.is_zero() {
            return Ordering::Equal;
        }
        return if a.sign() == num_bigint::Sign::Minus { Ordering::Less } else { Ordering::Greater };
    }
    let neg = b < 0.0;
    let bi_neg = a.sign() == num_bigint::Sign::Minus;
    if neg != bi_neg {
        return if bi_neg { Ordering::Less } else { Ordering::Greater };
    }
    // Compare magnitudes.
    let mag_cmp = cmp_magnitude(a, f64_abs(b));
    if neg {
        mag_cmp.reverse()
    } else {
        mag_cmp
    }
}

fn cmp_magnitude(a: &BigInt, b_pos: f64) -> Ordering {
    // |a| vs b_pos (>0)
    // b_pos = m * 2^e
    let bits = b_pos.to_bits();
    let raw_exp = ((bits >> 52) & 0x7ff) as i32;
    let mant_bits = bits & 0x000f_ffff_ffff_ffff;
    let (m, e): (BigInt, i32) = if raw_exp == 0 {
        (BigInt::from(mant_bits), -1074)
    } else {
        (BigInt::from((1u64 << 52) | mant_bits), raw_exp - 1023 - 52)
    };
    let mag = if a.sign() == num_bigint::Sign::Minus { -a.clone() } else { a.clone() };
    // Compare mag ? m * 2^e
    if e >= 0 {
        let rhs = m << (e as usize);
        mag.cmp(&rhs)
    } else {
        let shift = (-e) as usize;
        let lhs = mag.clone() << shift;
        lhs.cmp(&m)
    }
}

/// Float floor division `a // b` (CPython `Objects/floatobject.c`).
/// Both are floats (ints pre-converted to float with possible overflow).
pub fn float_floor_div(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float floor division by zero".to_string()));
    }
    if a.is_nan() || b.is_nan() || a.is_infinite() {
        return Ok(f64::NAN);
    }
    if b.is_infinite() {
        // x // inf is 0.0 (negative dividend gives -0.0 via floor(-0.0)).
        return Ok(libm::floor(a / b));
    }
    let q = libm::floor(a / b);
    Ok(q)
}

/// Float modulo `a % b` (CPython `Objects/floatobject.c:float_mod`:
/// `mod = a - (floor(a/b) * b)` with -0.0 correction).
pub fn float_mod(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float modulo".to_string()));
    }
    if a.is_nan() || b.is_nan() || a.is_infinite() {
        return Ok(f64::NAN);
    }
    if b.is_infinite() {
        if a == 0.0 {
            return Ok(if b < 0.0 { -0.0 } else { 0.0 });
        }
        if (a > 0.0) == (b > 0.0) {
            return Ok(a);
        }
        return Ok(if b > 0.0 { f64::INFINITY } else { f64::NEG_INFINITY });
    }
    let q = libm::floor(a / b);
    let mut m = a - q * b;
    if m == 0.0 {
        m = if b < 0.0 { -0.0 } else { 0.0 };
    }
    Ok(m)
}

/// True division `a / b` for floats (CPython).
pub fn float_true_div(a: f64, b: f64) -> Result<f64, PyError> {
    if b == 0.0 {
        return Err(PyError::new("ZeroDivisionError", "float division by zero".to_string()));
    }
    Ok(a / b)
}

/// `a ** b` for floats via libm `pow` (CPython `Objects/floatobject.c:float_pow`).
/// `0.0 ** negative` raises `ZeroDivisionError`. A finite result that
/// overflows sets `errno = ERANGE`, which CPython turns into
/// `PyErr_SetFromErrno(PyExc_OverflowError)`: the message is
/// `(34, 'Numerical result out of range')`.
/// `(-1.0) ** 0.5` would be complex in Python; here it yields `nan` from
/// libm `pow` and callers handle the complex case separately.
pub fn float_pow(a: f64, b: f64) -> Result<f64, PyError> {
    if a == 0.0 && b < 0.0 {
        return Err(PyError::new(
            "ZeroDivisionError",
            "0.0 cannot be raised to a negative power".to_string(),
        ));
    }
    let r = libm::pow(a, b);
    if r.is_infinite() && a.is_finite() && b.is_finite() {
        return Err(PyError::new(
            "OverflowError",
            "(34, 'Numerical result out of range')".to_string(),
        ));
    }
    Ok(r)
}

/// `sum` of floats as CPython 3.8 (plain left-to-right addition).
/// CPython 3.8 `sum` of a list of floats is naive `acc + x` in order
/// (Neumaier summation only arrived in 3.12). Order matters for parity.
pub fn py_sum_floats(xs: &[f64]) -> f64 {
    let mut acc = 0.0f64;
    // CPython sum([]) == 0 (int). For all-float lists, first element
    // seeds the accumulator when empty handling passes 0? Actually
    // sum([1.5]) == 1.5 (0.0 + 1.5 == 1.5). For empty, returns int 0.
    // Callers handle the empty-int case; here float sum of list.
    for &x in xs {
        acc += x;
    }
    acc
}
