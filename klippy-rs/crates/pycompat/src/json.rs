//! `json.dumps`/`json.loads` reproducing CPython `json`
//! (`json/encoder.py`, `json/decoder.py`).
//!
//! - [`dumps`] uses CPython defaults: separators `(', ', ': ')`,
//!   `ensure_ascii=True`, `allow_nan=True` (outputs `Infinity`, `-Infinity`
//!   and `NaN`, which CPython's strict JSON decoder refuses on load).
//!   Use [`dumps_compact`] for the `separators=(',', ':')` form klippy uses
//!   in `webhooks.py` (`json.dumps(obj, separators=(',', ':'))`) and in
//!   `extras/gcode_macro.py` to validate literals.
//! - Floats encode with `float_repr` (shortest round-trip); `NaN`/`inf`
//!   encode per `allow_nan` (`ValueError: Out of range float values are not
//!   JSON compliant: ...` when `allow_nan=False`). Ints encode in decimal
//!   at arbitrary precision. Strings encode with `"`/`\`/`/`/control
//!   escapes plus `\b \f \n \r \t`; `ensure_ascii=True` also escapes
//!   non-ASCII as `\uXXXX` with surrogate pairs, `ensure_ascii=False`
//!   emits raw UTF-8.
//! - Dict keys coerce as CPython: `str` unchanged, `int` decimal,
//!   `float` via `float_repr`, `True`/`False` -> `"true"`/`"false"`,
//!   `None` -> `"null"`. Other key types raise
//!   `TypeError: keys must be str, int, float, bool or None, not list` (and
//!   similarly for other types).
//! - [`loads`] parses ints into `PyInt` (arbitrary precision, leading
//!   zeros rejected), numbers with `.`/`e` as `float` via `float_from_string`
//!   semantics, strings with all escapes, arrays/objects, and `null`/
//!   `true`/`false` (plus `NaN`/`Infinity`/`-Infinity`, which CPython
//!   accepts by default). Whitespace is space/tab/newline/CR only.
//!   Errors are `ValueError` with CPython texts: `Expecting value: line L
//!   column C (char N)`, `Expecting ',' delimiter: ...`,
//!   `Expecting property name enclosed in double quotes: ...`,
//!   `Expecting ':' delimiter: ...`, `Extra data: ...`,
//!   `Unterminated string starting at: ...`,
//!   `Invalid control character at: ...`, `Invalid \escape: ...`,
//!   `Invalid \uXXXX escape: ...`, `The JSON object must be str, bytes or
//!   bytearray, not ...` for non-string input.

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
        PyValue::Float(f) => {
            if f.is_nan() {
                if !opts.allow_nan {
                    return Err(PyError::new(
                        "ValueError",
                        "Out of range float values are not JSON compliant: NaN".to_string(),
                    ));
                }
                out.push_str("NaN");
            } else if f.is_infinite() {
                if !opts.allow_nan {
                    let name = if *f > 0.0 { "Infinity" } else { "-Infinity" };
                    return Err(PyError::new(
                        "ValueError",
                        format!("Out of range float values are not JSON compliant: {}", name),
                    ));
                }
                out.push_str(if *f > 0.0 { "Infinity" } else { "-Infinity" });
            } else {
                out.push_str(&float_repr(*f));
            }
        }
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
                let key_str = coerce_key(k)?;
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
            '/' => out.push('/'),
            c if (c as u32) < 0x20 => {
                out.push_str(&format!("\\u{:04x}", c as u32));
            }
            c if ensure_ascii && !c.is_ascii() => {
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

fn coerce_key(k: &PyValue) -> Result<String, PyError> {
    match k {
        PyValue::Str(s) => Ok(s.clone()),
        PyValue::Int(n) => Ok(n.0.to_string()),
        PyValue::Float(f) => {
            if f.is_nan() {
                Ok("NaN".to_string())
            } else if f.is_infinite() {
                Ok(if *f > 0.0 { "Infinity".to_string() } else { "-Infinity".to_string() })
            } else {
                Ok(float_repr(*f))
            }
        }
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

/// `json.loads(s)` with CPython parsing and error texts.
pub fn loads(s: &str) -> Result<PyValue, PyError> {
    let mut p = Parser::new(s);
    p.skip_ws();
    let v = p.parse_value()?;
    p.skip_ws();
    if p.pos < p.text.len() {
        let (line, col) = p.line_col(p.pos);
        let _ = (line, col);
        return Err(PyError::new(
            "ValueError",
            format!("Extra data: line {} column {} (char {})", line, col, p.pos),
        ));
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

    fn err(&self, msg: &str, pos: usize) -> PyError {
        let (line, col) = self.line_col(pos);
        PyError::new(
            "ValueError",
            format!("{}: line {} column {} (char {})", msg, line, col, pos),
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
            Some(b'n') => {
                if self.text[self.pos..].starts_with("NaN") {
                    self.pos += 3;
                    Ok(PyValue::Float(f64::NAN))
                } else {
                    self.parse_lit("null", PyValue::None)
                }
            }
            Some(b'I') => self.parse_lit("Infinity", PyValue::Float(f64::INFINITY)),
            Some(b'-') => {
                if self.text[self.pos..].starts_with("-Infinity") {
                    self.pos += "-Infinity".len();
                    Ok(PyValue::Float(f64::NEG_INFINITY))
                } else {
                    self.parse_number()
                }
            }
            Some(c) if c == b'+' || c.is_ascii_digit() => self.parse_number(),
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

    fn parse_string(&mut self) -> Result<String, PyError> {
        let start = self.pos;
        // consume '"'
        self.pos += 1;
        let mut out = String::new();
        while self.pos < self.bytes.len() {
            let b = self.bytes[self.pos];
            if b == b'"' {
                self.pos += 1;
                return Ok(out);
            }
            if b == b'\\' {
                self.pos += 1;
                if self.pos >= self.bytes.len() {
                    break;
                }
                let e = self.bytes[self.pos];
                self.pos += 1;
                match e {
                    b'"' => out.push('"'),
                    b'\\' => out.push('\\'),
                    b'/' => out.push('/'),
                    b'b' => out.push('\u{0008}'),
                    b'f' => out.push('\u{000c}'),
                    b'n' => out.push('\n'),
                    b'r' => out.push('\r'),
                    b't' => out.push('\t'),
                    b'u' => {
                        let cp = self.parse_hex4()?;
                        if (0xD800..0xDC00).contains(&cp) {
                            // surrogate pair
                            if self.pos + 1 < self.bytes.len()
                                && self.bytes[self.pos] == b'\\'
                                && self.bytes[self.pos + 1] == b'u'
                            {
                                self.pos += 2;
                                let lo = self.parse_hex4()?;
                                if !(0xDC00..0xE000).contains(&lo) {
                                    return Err(self.err("Invalid \\uXXXX escape", start));
                                }
                                let v = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                                let c = char::from_u32(v).ok_or_else(|| self.err("Invalid \\uXXXX escape", start))?;
                                out.push(c);
                            } else {
                                return Err(self.err("Invalid \\uXXXX escape", start));
                            }
                        } else if (0xDC00..0xE000).contains(&cp) {
                            return Err(self.err("Invalid \\uXXXX escape", start));
                        } else {
                            let c = char::from_u32(cp).ok_or_else(|| self.err("Invalid \\uXXXX escape", start))?;
                            out.push(c);
                        }
                    }
                    _ => return Err(self.err("Invalid \\escape", self.pos - 1)),
                }
                continue;
            }
            if b < 0x20 {
                return Err(self.err("Invalid control character at", self.pos));
            }
            // copy one UTF-8 char
            let rest = &self.text[self.pos..];
            let mut it = rest.chars();
            let ch = it.next().unwrap_or('\0');
            out.push(ch);
            self.pos += ch.len_utf8();
        }
        let (line, col) = self.line_col(start);
        Err(PyError::new(
            "ValueError",
            format!("Unterminated string starting at: line {} column {} (char {})", line, col, start),
        ))
    }

    fn parse_hex4(&mut self) -> Result<u32, PyError> {
        let mut v: u32 = 0;
        for _ in 0..4 {
            if self.pos >= self.bytes.len() {
                return Err(self.err("Invalid \\uXXXX escape", self.pos));
            }
            let c = self.bytes[self.pos] as char;
            let d = c.to_digit(16).ok_or_else(|| self.err("Invalid \\uXXXX escape", self.pos))?;
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
        // int part
        if self.peek() == Some(b'0') {
            self.pos += 1;
            if self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                return Err(self.err("Expecting value", start));
            }
        } else if self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
            while self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                self.pos += 1;
            }
        } else {
            return Err(self.err("Expecting value", start));
        }
        let mut is_float = false;
        if self.peek() == Some(b'.') {
            is_float = true;
            self.pos += 1;
            if !self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                return Err(self.err("Expecting value", start));
            }
            while self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                self.pos += 1;
            }
        }
        if matches!(self.peek(), Some(b'e') | Some(b'E')) {
            is_float = true;
            self.pos += 1;
            if matches!(self.peek(), Some(b'+') | Some(b'-')) {
                self.pos += 1;
            }
            if !self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                return Err(self.err("Expecting value", start));
            }
            while self.peek().map(|c| c.is_ascii_digit()).unwrap_or(false) {
                self.pos += 1;
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
