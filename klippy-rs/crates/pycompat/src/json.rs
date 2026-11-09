//! `json.dumps`/`json.loads` reproducing CPython 3.8's C accelerator
//! (`Modules/_json.c`, driven by `json/encoder.py` and `json/decoder.py`).
//!
//! - [`dumps`] uses CPython defaults: separators `(', ', ': ')`,
//!   `ensure_ascii=True`, `allow_nan=True` (outputs `Infinity`, `-Infinity`
//!   and `NaN`). Use [`dumps_compact`] for the `separators=(',', ':')` form
//!   klippy uses in `webhooks.py` and in `extras/gcode_macro.py`.
//! - Floats encode with `float_repr`; non-finite floats (values and keys)
//!   raise `ValueError: Out of range float values are not JSON compliant`
//!   when `allow_nan=False`. Ints encode in decimal at arbitrary
//!   precision. Strings escape `"`, `\`, `\b \f \n \r \t` and other
//!   controls as `\u00XX`; `ensure_ascii=True` also escapes DEL and every
//!   non-ASCII character as `\uXXXX` (surrogate pairs above U+FFFF).
//! - Dict keys coerce as CPython: `str` unchanged, `int` decimal,
//!   `float` via `float_repr`, `True`/`False` -> `"true"`/`"false"`,
//!   `None` -> `"null"`. Other key types raise
//!   `TypeError: keys must be str, int, float, bool or None, not list`.
//! - [`loads`] parses ints into `PyInt` (arbitrary precision), floats
//!   with `float(str)` rounding, strings with all escapes and surrogate
//!   pairs, arrays/objects, `null`/`true`/`false` and
//!   `NaN`/`Infinity`/`-Infinity`. Whitespace is space/tab/newline/CR.
//!   Errors are `JSONDecodeError` with the scanner's texts and character
//!   positions (`Expecting value: line L column C (char N)`, `Expecting
//!   ',' delimiter`, `Expecting property name enclosed in double quotes`,
//!   `Expecting ':' delimiter`, `Extra data`, `Unterminated string
//!   starting at`, `Invalid control character at`, `Invalid \escape`,
//!   `Invalid \uXXXX escape`, `Unexpected UTF-8 BOM (decode using
//!   utf-8-sig)`). An unpaired surrogate escape, which CPython keeps in the
//!   resulting `str`, cannot live in a Rust `String` and raises
//!   `ValueError` instead.

use crate::int::PyInt;
use crate::text::{float_from_string, float_repr, PyError};
use crate::value::PyValue;
use num_bigint::BigInt;
use num_traits::Zero;

/// Options for [`dumps_opts`], mirroring CPython `json.dumps` keywords.
#[derive(Clone, Debug)]
pub struct DumpsOpts {
    /// Item separator; default `", "`. CPython default is `(', ', ': ')`.
    pub item_sep: String,
    /// Key separator; default `": "`.
    pub key_sep: String,
    /// Escape non-ASCII as `\uXXXX` (CPython default `True`).
    pub ensure_ascii: bool,
    /// Allow `NaN`/`Infinity` (CPython default `True`).
    pub allow_nan: bool,
}

impl Default for DumpsOpts {
    fn default() -> Self {
        DumpsOpts {
            item_sep: ", ".to_string(),
            key_sep: ": ".to_string(),
            ensure_ascii: true,
            allow_nan: true,
        }
    }
}

/// `json.dumps(obj)` with CPython defaults.
pub fn dumps(v: &PyValue) -> Result<String, PyError> {
    dumps_opts(v, &DumpsOpts::default())
}

/// `json.dumps(obj, separators=(',', ':'))` as used in klippy `webhooks.py`.
pub fn dumps_compact(v: &PyValue) -> Result<String, PyError> {
    dumps_opts(
        v,
        &DumpsOpts {
            item_sep: ",".to_string(),
            key_sep: ":".to_string(),
            ensure_ascii: true,
            allow_nan: true,
        },
    )
}

/// `json.dumps(obj, **opts)` with explicit options.
pub fn dumps_opts(v: &PyValue, opts: &DumpsOpts) -> Result<String, PyError> {
    let mut out = String::new();
    encode(v, opts, &mut out)?;
    Ok(out)
}

fn encode(v: &PyValue, opts: &DumpsOpts, out: &mut String) -> Result<(), PyError> {
    match v {
        PyValue::None => out.push_str("null"),
        PyValue::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        PyValue::Int(n) => out.push_str(&n.0.to_string()),
        PyValue::Float(f) => out.push_str(&encode_float(*f, opts)?),
        PyValue::Str(s) => encode_str(s, opts.ensure_ascii, out),
        PyValue::Bytes(_) => {
            return Err(PyError::new(
                "TypeError",
                "Object of type bytes is not JSON serializable".to_string(),
            ));
        }
        PyValue::List(xs) | PyValue::Tuple(xs) => {
            out.push('[');
            for (i, x) in xs.iter().enumerate() {
                if i > 0 {
                    out.push_str(&opts.item_sep);
                }
                encode(x, opts, out)?;
            }
            out.push(']');
        }
        PyValue::Dict(d) => {
            out.push('{');
            for (i, (k, val)) in d.iter().enumerate() {
                if i > 0 {
                    out.push_str(&opts.item_sep);
                }
                let key_str = coerce_key(k, opts)?;
                encode_str(&key_str, opts.ensure_ascii, out);
                out.push_str(&opts.key_sep);
                encode(val, opts, out)?;
            }
            out.push('}');
        }
        PyValue::Host(_) => {
            return Err(PyError::new(
                "TypeError",
                "Object of type object is not JSON serializable".to_string(),
            ));
        }
    }
    Ok(())
}

/// `encoder_encode_float` (`Modules/_json.c`).
fn encode_float(f: f64, opts: &DumpsOpts) -> Result<String, PyError> {
    if f.is_finite() {
        return Ok(float_repr(f));
    }
    if !opts.allow_nan {
        return Err(PyError::new(
            "ValueError",
            "Out of range float values are not JSON compliant".to_string(),
        ));
    }
    Ok(if f > 0.0 {
        "Infinity"
    } else if f < 0.0 {
        "-Infinity"
    } else {
        "NaN"
    }
    .to_string())
}

/// `ascii_escape_unicode` (`ensure_ascii=True`) and `escape_unicode`
/// (`Modules/_json.c`).
fn encode_str(s: &str, ensure_ascii: bool, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            '\u{0008}' => out.push_str("\\b"),
            '\u{000c}' => out.push_str("\\f"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            ' '..='~' => out.push(c),
            c if ensure_ascii => {
                let o = c as u32;
                if o < 0x10000 {
                    out.push_str(&format!("\\u{:04x}", o));
                } else {
                    let v = o - 0x10000;
                    let hi = 0xD800 + (v >> 10);
                    let lo = 0xDC00 + (v & 0x3FF);
                    out.push_str(&format!("\\u{:04x}\\u{:04x}", hi, lo));
                }
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn coerce_key(k: &PyValue, opts: &DumpsOpts) -> Result<String, PyError> {
    match k {
        PyValue::Str(s) => Ok(s.clone()),
        PyValue::Int(n) => Ok(n.0.to_string()),
        PyValue::Float(f) => encode_float(*f, opts),
        PyValue::Bool(b) => Ok(if *b { "true".to_string() } else { "false".to_string() }),
        PyValue::None => Ok("null".to_string()),
        other => Err(PyError::new(
            "TypeError",
            format!(
                "keys must be str, int, float, bool or None, not {}",
                crate::value::type_name(other)
            ),
        )),
    }
}

/// `json.loads(s)` for a `str` argument, with CPython parsing and error texts.
pub fn loads(s: &str) -> Result<PyValue, PyError> {
    if s.starts_with('\u{feff}') {
        return Err(Parser::new(s).err("Unexpected UTF-8 BOM (decode using utf-8-sig)", 0));
    }
    loads_decoded(s)
}

/// `json.loads(b)` after `b` (a `bytes` argument) was decoded with the detected
/// encoding: CPython checks for the UTF-8 BOM only when given a `str`, so a
/// U+FEFF left after decoding goes to the parser.
pub fn loads_decoded(s: &str) -> Result<PyValue, PyError> {
    let mut p = Parser::new(s);
    p.skip_ws();
    let v = p.parse_value()?;
    p.skip_ws();
    if p.pos < p.text.len() {
        return Err(p.err("Extra data", p.pos));
    }
    Ok(v)
}

struct Parser<'a> {
    text: &'a str,
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(s: &'a str) -> Self {
        Parser { text: s, bytes: s.as_bytes(), pos: 0 }
    }

    fn line_col(&self, pos: usize) -> (usize, usize) {
        let mut line = 1usize;
        let mut col = 1usize;
        for (i, ch) in self.text.char_indices() {
            if i >= pos {
                break;
            }
            if ch == '\n' {
                line += 1;
                col = 1;
            } else {
                col += 1;
            }
        }
        (line, col)
    }

    /// CPython's `JSONDecodeError` text: `pos` is a character index, so the
    /// byte offset is converted.
    fn err(&self, msg: &str, pos: usize) -> PyError {
        let (line, col) = self.line_col(pos);
        let char_pos = self.text[..pos].chars().count();
        PyError::new(
            "JSONDecodeError",
            format!("{}: line {} column {} (char {})", msg, line, col, char_pos),
        )
    }

    fn skip_ws(&mut self) {
        while self.pos < self.bytes.len() && matches!(self.bytes[self.pos], b' ' | b'\t' | b'\n' | b'\r') {
            self.pos += 1;
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn parse_value(&mut self) -> Result<PyValue, PyError> {
        self.skip_ws();
        match self.peek() {
            Some(b'{') => self.parse_object(),
            Some(b'[') => self.parse_array(),
            Some(b'"') => Ok(PyValue::Str(self.parse_string()?)),
            Some(b't') => self.parse_lit("true", PyValue::Bool(true)),
            Some(b'f') => self.parse_lit("false", PyValue::Bool(false)),
            Some(b'n') => self.parse_lit("null", PyValue::None),
            Some(b'N') => self.parse_lit("NaN", PyValue::Float(f64::NAN)),
            Some(b'I') => self.parse_lit("Infinity", PyValue::Float(f64::INFINITY)),
            Some(b'-') => {
                if self.text[self.pos..].starts_with("-Infinity") {
                    self.pos += "-Infinity".len();
                    Ok(PyValue::Float(f64::NEG_INFINITY))
                } else {
                    self.parse_number()
                }
            }
            Some(c) if c.is_ascii_digit() => self.parse_number(),
            _ => Err(self.err("Expecting value", self.pos)),
        }
    }

    fn parse_lit(&mut self, lit: &str, val: PyValue) -> Result<PyValue, PyError> {
        if self.text[self.pos..].starts_with(lit) {
            self.pos += lit.len();
            Ok(val)
        } else {
            Err(self.err("Expecting value", self.pos))
        }
    }

    fn parse_object(&mut self) -> Result<PyValue, PyError> {
        // consume '{'
        self.pos += 1;
        let mut map = indexmap::IndexMap::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.pos += 1;
            return Ok(PyValue::Dict(map));
        }
        loop {
            self.skip_ws();
            if self.peek() != Some(b'"') {
                return Err(self.err("Expecting property name enclosed in double quotes", self.pos));
            }
            let key = self.parse_string()?;
            self.skip_ws();
            if self.peek() != Some(b':') {
                return Err(self.err("Expecting ':' delimiter", self.pos));
            }
            self.pos += 1;
            let val = self.parse_value()?;
            map.insert(PyValue::Str(key), val);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                    continue;
                }
                Some(b'}') => {
                    self.pos += 1;
                    return Ok(PyValue::Dict(map));
                }
                _ => return Err(self.err("Expecting ',' delimiter", self.pos)),
            }
        }
    }

    fn parse_array(&mut self) -> Result<PyValue, PyError> {
        self.pos += 1;
        let mut xs = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.pos += 1;
            return Ok(PyValue::List(xs));
        }
        loop {
            let v = self.parse_value()?;
            xs.push(v);
            self.skip_ws();
            match self.peek() {
                Some(b',') => {
                    self.pos += 1;
                    continue;
                }
                Some(b']') => {
                    self.pos += 1;
                    return Ok(PyValue::List(xs));
                }
                _ => return Err(self.err("Expecting ',' delimiter", self.pos)),
            }
        }
    }

    /// `scanstring_unicode` (`Modules/_json.c`, `strict=True`): `self.pos`
    /// is on the opening quote. Error positions follow the C scanner (the
    /// backslash for a bad escape, the `u` for a bad `\uXXXX`).
    fn parse_string(&mut self) -> Result<String, PyError> {
        let begin = self.pos;
        self.pos += 1;
        let mut out = String::new();
        loop {
            let chunk = self.pos;
            let mut terminator = None;
            while self.pos < self.bytes.len() {
                let b = self.bytes[self.pos];
                if b == b'"' || b == b'\\' {
                    terminator = Some(b);
                    break;
                }
                if b <= 0x1f {
                    return Err(self.err("Invalid control character at", self.pos));
                }
                self.pos += 1;
            }
            let Some(c) = terminator else {
                return Err(self.err("Unterminated string starting at", begin));
            };
            out.push_str(&self.text[chunk..self.pos]);
            self.pos += 1;
            if c == b'"' {
                return Ok(out);
            }
            if self.pos == self.bytes.len() {
                return Err(self.err("Unterminated string starting at", begin));
            }
            let e = self.bytes[self.pos];
            if e != b'u' {
                let ch = match e {
                    b'"' => '"',
                    b'\\' => '\\',
                    b'/' => '/',
                    b'b' => '\u{0008}',
                    b'f' => '\u{000c}',
                    b'n' => '\n',
                    b'r' => '\r',
                    b't' => '\t',
                    _ => return Err(self.err("Invalid \\escape", self.pos - 1)),
                };
                out.push(ch);
                self.pos += 1;
                continue;
            }
            let u_pos = self.pos;
            self.pos += 1;
            // The four digits must be followed by another character.
            if self.text[self.pos..].chars().nth(4).is_none() {
                return Err(self.err("Invalid \\uXXXX escape", u_pos));
            }
            let mut cp = self.parse_hex4(u_pos)?;
            if (0xD800..0xDC00).contains(&cp)
                && self.text[self.pos..].chars().nth(6).is_some()
                && self.bytes[self.pos] == b'\\'
                && self.bytes[self.pos + 1] == b'u'
            {
                let first_end = self.pos;
                self.pos += 2;
                let lo = self.parse_hex4(first_end + 1)?;
                if (0xDC00..0xE000).contains(&lo) {
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                } else {
                    self.pos = first_end;
                }
            }
            // CPython keeps an unpaired surrogate in the str; a Rust
            // `String` cannot hold one.
            let ch = char::from_u32(cp).ok_or_else(|| {
                PyError::new(
                    "ValueError",
                    format!("unpaired surrogate \\u{:04x} in JSON string cannot be represented", cp),
                )
            })?;
            out.push(ch);
        }
    }

    /// Four hex digits at `self.pos`; a bad digit reports `u_pos`.
    fn parse_hex4(&mut self, u_pos: usize) -> Result<u32, PyError> {
        let mut v: u32 = 0;
        for _ in 0..4 {
            let d = self
                .bytes
                .get(self.pos)
                .and_then(|&b| (b as char).to_digit(16))
                .ok_or_else(|| self.err("Invalid \\uXXXX escape", u_pos))?;
            v = v * 16 + d;
            self.pos += 1;
        }
        Ok(v)
    }

    fn parse_number(&mut self) -> Result<PyValue, PyError> {
        let start = self.pos;
        if self.peek() == Some(b'-') {
            self.pos += 1;
        }
        // `NUMBER_RE = (-?(?:0|[1-9]\d*))(\.\d+)?([eE][-+]?\d+)?`: the
        // fraction and exponent are optional, so a trailing `.` / `e` or a
        // digit after a leading `0` ends the number (and the caller reports
        // what follows).
        let is_digit = |c: Option<u8>| c.map(|c| c.is_ascii_digit()).unwrap_or(false);
        if self.peek() == Some(b'0') {
            self.pos += 1;
        } else if is_digit(self.peek()) {
            while is_digit(self.peek()) {
                self.pos += 1;
            }
        } else {
            return Err(self.err("Expecting value", start));
        }
        let mut is_float = false;
        if self.peek() == Some(b'.') && is_digit(self.bytes.get(self.pos + 1).copied()) {
            is_float = true;
            self.pos += 1;
            while is_digit(self.peek()) {
                self.pos += 1;
            }
        }
        if matches!(self.peek(), Some(b'e') | Some(b'E')) {
            let sign = matches!(self.bytes.get(self.pos + 1), Some(b'+') | Some(b'-'));
            let digit_at = self.pos + 1 + usize::from(sign);
            if is_digit(self.bytes.get(digit_at).copied()) {
                is_float = true;
                self.pos = digit_at;
                while is_digit(self.peek()) {
                    self.pos += 1;
                }
            }
        }
        let tok = &self.text[start..self.pos];
        if is_float {
            // CPython parses floats with the same rules as float(str) for
            // these tokens; leading '-' and digits are already validated.
            let f = float_from_string(tok).map_err(|e| self.err(&e.msg, start))?;
            Ok(PyValue::Float(f))
        } else {
            let mut val = BigInt::zero();
            let neg = tok.starts_with('-');
            let digits = if neg { &tok[1..] } else { tok };
            for ch in digits.chars() {
                let d = (ch as u8 - b'0') as i64;
                val = &val * &BigInt::from(10i64) + &BigInt::from(d);
            }
            if neg {
                val = -val;
            }
            Ok(PyValue::Int(PyInt(val)))
        }
    }
}
