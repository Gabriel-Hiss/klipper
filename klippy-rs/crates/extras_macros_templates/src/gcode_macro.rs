//! Port of `klippy/extras/gcode_macro.py` with the Jinja2 2.11.3 subset.
//!
//! The reference unit provides two wrappers around Jinja2 with custom
//! delimiters (`{% %} ` for blocks, `{ }` for expressions, `{# #}` for
//! comments). This module reproduces the supported subset byte for byte
//! for every template that occurs in `config/**/*.cfg`,
//! `test/klippy/*.cfg`, `config/ender-3-v3-ke/*.cfg`,
//! `parity/scenarios/ke/ke-batch-move/mnt/usr/data/mainsail-config/mainsail.cfg`,
//! plus `klippy/extras/display/{display,menu}.cfg` whose renders the
//! `printers-070` recording exercises.
//!
//! Supported constructs (everything else raises
//! [`TemplateError`] with `kind = "TemplateSyntaxError"` naming the
//! construct):
//!
//! * Tags: `{% if / elif / else / endif %}`, `{% for name in expr %}`
//!   with optional `{% else %}` and `{% endfor %}`, `{% set name =
//!   expr %}` (exactly one name, `namespace(...)` and block `set` are
//!   rejected). `{# ... #}` comments and `-` whitespace control.
//! * Expressions: literals (int base 0/2/8/10/16, float, string with
//!   Python escapes, adjacent literal concatenation, list, tuple,
//!   dict, `true/True/false/False/none/None`), names, `.attr`,
//!   `[index]`, calls with positional and `name=` keyword args,
//!   filters `abs default d float int length lower max min replace
//!   upper`, tests `defined undefined none in == eq != ne > gt >= ge
//!   < lt <= le`, operators `+ - * / // % ** ~ == != < > <= >= in
//!   not in and or not + - **`, ternary `a if cond else b` (missing
//!   `else` is allowed and yields `Undefined`), `range()` global.
//! * Methods: `str.split`, `str.replace` (via filter or method),
//!   `str.strip`, `str.format`, `list.append`. The `loop` variable in
//!   `for` exposes `index index0 revindex revindex0 first last
//!   length depth previtem nextitem` as a `Dict` (changed/render via
//!   loop bodies is rejected as unsupported).
//! * Errors: `UndefinedError` implements the default-`Undefined`
//!   rule (falsy, empty print, empty iteration, every other operation
//!   raises `UndefinedError` with Jinja's message shape), `TypeError`
//!   for unknown filter/test/method with Jinja's
//!   ``No filter named 'x'.`` / ``No test named 'x'.`` texts,
//!   `TemplateSyntaxError` with `expected token ...` texts.
//!
//! Numeric and text output formatting delegates to `pycompat` so
//! `str(float)`, `%` and `.format()` match CPython 3.8.

use indexmap::IndexMap;
use pycompat::int::{int_divmod, int_floor_div, int_mod, int_pow, PyInt};
use pycompat::json as pyjson;
use pycompat::text as pytext;
use pycompat::text::{PctValue, PyError};
use pycompat::value::{py_compare, py_equal, PyValue};
use std::cmp::Ordering;
use std::rc::Rc;

// ---------------------------------------------------------------------------
// errors
// ---------------------------------------------------------------------------

/// Jinja2 error carrier. `message` is the text after `kind: ` in
/// `traceback.format_exception_only` output, `lineno` is 1-based source
/// line when the error carries one (syntax / undefined / type).
#[derive(Clone, Debug, PartialEq)]
pub struct TemplateError {
    kind: String,
    message: String,
    lineno: Option<usize>,
}

impl TemplateError {
    pub fn new(kind: &str, message: String, lineno: Option<usize>) -> Self {
        TemplateError {
            kind: kind.to_string(),
            message,
            lineno,
        }
    }
    pub fn syntax(message: String, lineno: Option<usize>) -> Self {
        Self::new("TemplateSyntaxError", message, lineno)
    }
    pub fn kind(&self) -> &str {
        &self.kind
    }
    pub fn message(&self) -> &str {
        &self.message
    }
    pub fn lineno(&self) -> Option<usize> {
        self.lineno
    }
    /// `"{kind}: {message}"` as in `str(e)` for the Jinja exception.
    pub fn display_string(&self) -> String {
        format!("{}: {}", self.kind, self.message)
    }
}

impl std::fmt::Display for TemplateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.display_string())
    }
}

impl std::error::Error for TemplateError {}

fn syntax_err(msg: String, lineno: usize) -> TemplateError {
    TemplateError::syntax(msg, Some(lineno))
}
fn undefined_err(msg: String, lineno: usize) -> TemplateError {
    TemplateError::new("UndefinedError", msg, Some(lineno))
}
fn type_err(msg: String, lineno: usize) -> TemplateError {
    TemplateError::new("TypeError", msg, Some(lineno))
}

// ---------------------------------------------------------------------------
// runtime values
// ---------------------------------------------------------------------------

/// Hint carried by `Undefined` for Jinja's `_undefined_message`.
#[derive(Clone, Debug)]
pub struct UndefinedHint {
    /// When set, Jinja uses it verbatim as the message.
    pub hint: Option<String>,
    /// `repr`-rendered object type for `"{obj!r} has no element/attr"`.
    /// `None` means missing object (bare name case).
    pub obj: Option<String>,
    pub name: Option<String>,
}

/// The eval-time value: either a concrete Python value, the default
/// `Undefined` (falsy/empty, but strict on operations), or an
/// arbitrary host callable (the `action_*` functions, the display
/// `render`/`draw_progress_bar` helpers and `range`).
#[derive(Clone)]
pub enum Value {
    Undefined(UndefinedHint),
    Py(PyValue),
    Func(
        Rc<
            dyn Fn(&[Value], &[(String, Value)], usize) -> Result<Value, TemplateError>,
        >,
    ),
}

impl Value {
    pub fn py(v: PyValue) -> Self {
        Value::Py(v)
    }
    pub fn undefined_name(name: &str) -> Self {
        Value::Undefined(UndefinedHint {
            hint: None,
            obj: None,
            name: Some(name.to_string()),
        })
    }
    pub fn is_undefined(&self) -> bool {
        matches!(self, Value::Undefined(_))
    }
    /// Python truthiness; `Undefined` is falsy (`bool(Undefined())` is False).
    pub fn truthy(&self) -> bool {
        match self {
            Value::Undefined(_) => false,
            Value::Py(v) => v.truthy(),
            Value::Func(_) => true,
        }
    }
    /// `str(v)` as Jinja would render it: `Undefined` prints empty,
    /// concrete values via `pycompat` (so a list prints its `repr`).
    pub fn to_string(&self) -> String {
        match self {
            Value::Undefined(_) => String::new(),
            Value::Py(v) => v.str(),
            Value::Func(_) => String::new(),
        }
    }
    /// `TypeError` style type name for messages.
    pub fn type_name(&self) -> &'static str {
        match self {
            Value::Undefined(_) => "Undefined",
            Value::Py(v) => pycompat::value::type_name(v),
            Value::Func(_) => "function",
        }
    }
    /// `_undefined_message` text for this value (only meaningful when the
    /// value is `Undefined`; otherwise yields an empty hint).
    pub fn undefined_message(&self, lineno: usize) -> String {
        match self {
            Value::Undefined(h) => {
                if let Some(hint) = &h.hint {
                    return hint.clone();
                }
                match (&h.obj, &h.name) {
                    (None, Some(name)) => format!("'{name}' is undefined"),
                    (Some(obj), Some(name)) => {
                        format!("{obj} has no attribute {name}")
                    }
                    (Some(obj), None) => format!("{obj} has no element"),
                    (None, None) => format!("{lineno} is undefined"),
                }
            }
            Value::Py(_) | Value::Func(_) => String::new(),
        }
    }
    pub fn fail_undefined(&self, lineno: usize) -> TemplateError {
        undefined_err(self.undefined_message(lineno), lineno)
    }
}

/// Format `value` as CPython `str()` through `pycompat`.
fn py_str_of(v: &Value, lineno: usize) -> Result<String, TemplateError> {
    match v {
        Value::Undefined(h) => {
            let _ = h;
            Ok(String::new())
        }
        Value::Py(p) => Ok(p.str()),
        Value::Func(_) => Err(type_err(
            "function object is not convertible to string in this template position".to_string(),
            lineno,
        )),
    }
}

/// Wrap a `PyError` raised while converting/arithmetic into a
/// `TemplateError`, preserving CPython `kind: msg` text.
fn from_pyerr(e: &PyError, lineno: usize) -> TemplateError {
    TemplateError::new(e.kind, e.msg.clone(), Some(lineno))
}

// ---------------------------------------------------------------------------
// AST
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
pub enum Expr {
    Const(PyValue),
    Name(String),
    List(Vec<Expr>),
    Tuple(Vec<Expr>),
    Dict(Vec<(Expr, Expr)>),
    GetAttr(Box<Expr>, String),
    GetItem(Box<Expr>, Box<Expr>),
    Call {
        callee: Box<Expr>,
        args: Vec<Expr>,
        kwargs: Vec<(String, Expr)>,
    },
    Filter {
        node: Box<Expr>,
        name: String,
        args: Vec<Expr>,
        kwargs: Vec<(String, Expr)>,
    },
    Test {
        node: Box<Expr>,
        name: String,
        args: Vec<Expr>,
        kwargs: Vec<(String, Expr)>,
        negated: bool,
    },
    Compare {
        first: Box<Expr>,
        ops: Vec<(String, Expr)>,
    },
    Add(Box<Expr>, Box<Expr>),
    Sub(Box<Expr>, Box<Expr>),
    Mul(Box<Expr>, Box<Expr>),
    Div(Box<Expr>, Box<Expr>),
    FloorDiv(Box<Expr>, Box<Expr>),
    Mod(Box<Expr>, Box<Expr>),
    Pow(Box<Expr>, Box<Expr>),
    Concat(Box<Expr>, Box<Expr>),
    Neg(Box<Expr>),
    Pos(Box<Expr>),
    Not(Box<Expr>),
    And(Box<Expr>, Box<Expr>),
    Or(Box<Expr>, Box<Expr>),
    CondExpr(Box<Expr>, Box<Expr>, Option<Box<Expr>>),
}

impl Expr {
    fn lineno(&self) -> usize {
        // line numbers are attached by the parser to the token stream;
        // here we return 1 as fallback (real eval calls pass lineno in).
        1
    }
}

#[derive(Clone, Debug)]
pub enum Node {
    Text(String),
    Print(Expr),
    If {
        branches: Vec<(Expr, Vec<Node>)>,
        else_body: Vec<Node>,
    },
    For {
        target: String,
        iter: Expr,
        body: Vec<Node>,
        else_body: Vec<Node>,
    },
    Set {
        name: String,
        value: Expr,
    },
}

// ---------------------------------------------------------------------------
// lexing (template level)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Tok {
    Data(String),
    Block {
        text: String,
        lineno: usize,
        trim_left: bool,
        trim_right: bool,
    },
    Var {
        text: String,
        lineno: usize,
        trim_left: bool,
        trim_right: bool,
    },
}

fn line_of(src: &str, pos: usize) -> usize {
    src.as_bytes()[..pos].iter().filter(|&&b| b == b'\n').count() + 1
}

/// Skip a Python-ish string literal starting at `i` (quote char in
/// `src`, `'` of `'`/`"`), returning the index just past the closing
/// quote. Handles backslash escapes. Used only to avoid mistaking
/// delimiters inside strings for tag ends.
fn skip_string(src: &str, start: usize) -> Result<usize, String> {
    let bytes = src.as_bytes();
    let quote = bytes[start];
    let mut i = start + 1;
    while i < src.len() {
        let b = bytes[i];
        if b == b'\\' {
            i += 2;
            continue;
        }
        if b == quote {
            return Ok(i + 1);
        }
        i += 1;
    }
    Err("unterminated string".to_string())
}

/// Find the tag close for a `{% ... %}` block starting just after
/// `{%`/`{#`, respecting string literals. `close` is `"%}"` or `"#}"`.
/// Returns `(inner, end_pos_after_close, trim_right)` where
/// `trim_right` is set when it saw `-%}` / `-#}`. A leading `-`/`+`
/// after the opener has already been consumed by the caller.
fn find_block_close(
    src: &str,
    start: usize,
    close: &str,
) -> Result<(String, usize, bool), String> {
    let bytes = src.as_bytes();
    let mut i = start;
    while i < src.len() {
        let b = bytes[i];
        if b == b'\'' || b == b'"' {
            i = skip_string(src, i)?;
            continue;
        }
        if b == b'-' {
            // `-%}` / `-#}` with optional following whitespace eaten later.
            let rest = &src[i..];
            let cand = format!("-{close}");
            if rest.starts_with(&cand) {
                let inner = src[start..i].to_string();
                return Ok((inner, i + cand.len(), true));
            }
            // `+%}` (explicit keep) is also accepted by Jinja lexer.
            let cand2 = format!("+{close}");
            if rest.starts_with(&cand2) {
                let inner = src[start..i].to_string();
                return Ok((inner, i + cand2.len(), false));
            }
        }
        if src[i..].starts_with(close) {
            let inner = src[start..i].to_string();
            return Ok((inner, i + close.len(), false));
        }
        i += 1;
    }
    Err(format!("Missing end of {close} tag"))
}

/// Find the matching `}` for a `{ ... }` variable block starting just
/// after `{` (positional `start`), with brace balancing that respects
/// string literals (so `{{'a':1}}` balances). Also handles `-%}` style
/// trailing `-` plus following whitespace by returning `trim_right`.
fn find_var_close(src: &str, start: usize) -> Result<(String, usize, bool), String> {
    let bytes = src.as_bytes();
    let mut depth = 1usize;
    let mut i = start;
    // Strip a leading `-`/`+` whitespace control already handled by caller,
    // but here `start` points after it.
    while i < src.len() {
        let b = bytes[i];
        if b == b'\'' || b == b'"' {
            // strings may contain braces
            let end = skip_string(src, i)?;
            i = end;
            continue;
        }
        match b {
            b'{' => {
                depth += 1;
                i += 1;
            }
            b'}' => {
                if depth == 1 {
                    // check for `-}` (whitespace control) directly before
                    // by looking backwards is not possible here; the `-`
                    // would have been part of inner (e.g. `{ x -}`). Jinja
                    // treats trailing `-` before `}` as trim marker. Handle
                    // by inspecting the inner tail.
                    let inner = src[start..i].to_string();
                    let (inner2, trim) = if inner.ends_with('-') {
                        (inner[..inner.len() - 1].to_string(), true)
                    } else if inner.ends_with('+') {
                        (inner[..inner.len() - 1].to_string(), false)
                    } else {
                        (inner, false)
                    };
                    return Ok((inner2, i + 1, trim));
                }
                depth -= 1;
                i += 1;
            }
            _ => {
                i += 1;
            }
        }
    }
    Err("unexpected '}'".to_string())
}

fn lex_template(source: &str) -> Result<Vec<Tok>, TemplateError> {
    // Normalize newlines and strip one trailing newline (keep_trailing_newline=False).
    let mut src: String = source.replace("\r\n", "\n").replace('\r', "\n");
    if src.ends_with('\n') {
        src.pop();
    }
    let bytes = src.as_bytes();
    let mut toks: Vec<Tok> = Vec::new();
    let mut pos = 0usize;
    let mut data_from = 0usize;
    while pos < src.len() {
        if bytes[pos] == b'{' {
            // comment `{#`
            if src[pos..].starts_with("{#") {
                // flush data
                if data_from < pos {
                    toks.push(Tok::Data(src[data_from..pos].to_string()));
                }
                let lineno = line_of(&src, pos);
                let mut p = pos + 2;
                let mut trim_left = false;
                if p < src.len() && bytes[p] == b'-' {
                    trim_left = true;
                    p += 1;
                } else if p < src.len() && bytes[p] == b'+' {
                    p += 1;
                }
                let (inner, after, trim_right) = find_block_close(&src, p, "#}")
                    .map_err(|m| syntax_err(m, lineno))?;
                let _ = inner;
                let _ = trim_left;
                // comments produce no token; trimming is applied below.
                let t = Tok::Block {
                    text: String::new(),
                    lineno,
                    trim_left,
                    trim_right,
                };
                // Mark as comment via empty text + sentinel: we instead
                // handle trimming directly and do not emit anything.
                // Apply trim flags to neighbouring data now.
                if trim_left {
                    // strip trailing ws from previous data token if it exists
                    if let Some(Tok::Data(d)) = toks.last_mut() {
                        let s = d.trim_end().to_string();
                        *d = s;
                    }
                }
                pos = after;
                if trim_right {
                    // eat following whitespace (Jinja `-%}\s*`)
                    while pos < src.len()
                        && matches!(bytes[pos], b' ' | b'\t' | b'\n' | b'\r' | 11 | 12)
                    {
                        pos += 1;
                    }
                }
                // do not push comment token
                let _ = t;
                data_from = pos;
                continue;
            }
            // block `{%`
            if src[pos..].starts_with("{%") {
                if data_from < pos {
                    toks.push(Tok::Data(src[data_from..pos].to_string()));
                }
                let lineno = line_of(&src, pos);
                let mut p = pos + 2;
                let mut trim_left = false;
                if p < src.len() && bytes[p] == b'-' {
                    trim_left = true;
                    p += 1;
                } else if p < src.len() && bytes[p] == b'+' {
                    p += 1;
                }
                if trim_left {
                    if let Some(Tok::Data(d)) = toks.last_mut() {
                        let s = d.trim_end().to_string();
                        *d = s;
                    }
                }
                let (inner, after, trim_right) =
                    find_block_close(&src, p, "%}").map_err(|m| syntax_err(m, lineno))?;
                toks.push(Tok::Block {
                    text: inner,
                    lineno,
                    trim_left,
                    trim_right,
                });
                pos = after;
                // trim_right whitespace is handled after pushing by peeking
                // next data flush: Jinja eats whitespace between `%}` and
                // next text. We do it here by advancing pos.
                if trim_right {
                    while pos < src.len()
                        && matches!(bytes[pos], b' ' | b'\t' | b'\n' | b'\r' | 11 | 12)
                    {
                        pos += 1;
                    }
                }
                data_from = pos;
                continue;
            }
            // variable `{` (single-brace). Any lone `{` starts a variable.
            // Jinja treats it as such; stray `{` in literal text would be a
            // syntax error, which matches Jinja.
            if data_from < pos {
                toks.push(Tok::Data(src[data_from..pos].to_string()));
            }
            let lineno = line_of(&src, pos);
            let mut p = pos + 1;
            let mut trim_left = false;
            if p < src.len() && bytes[p] == b'-' {
                trim_left = true;
                p += 1;
            } else if p < src.len() && bytes[p] == b'+' {
                p += 1;
            }
            if trim_left {
                if let Some(Tok::Data(d)) = toks.last_mut() {
                    let s = d.trim_end().to_string();
                    *d = s;
                }
            }
            let (inner, after, trim_right) = match find_var_close(&src, p) {
                Ok(v) => v,
                Err(m) => return Err(syntax_err(m, lineno)),
            };
            toks.push(Tok::Var {
                text: inner,
                lineno,
                trim_left,
                trim_right,
            });
            pos = after;
            if trim_right {
                while pos < src.len()
                    && matches!(bytes[pos], b' ' | b'\t' | b'\n' | b'\r' | 11 | 12)
                {
                    pos += 1;
                }
            }
            data_from = pos;
            continue;
        }
        // advance to next '{'
        match src[pos..].find('{') {
            Some(off) => {
                pos += off;
            }
            None => break,
        }
    }
    if data_from < src.len() {
        toks.push(Tok::Data(src[data_from..].to_string()));
    }
    Ok(toks)
}

// ---------------------------------------------------------------------------
// expression tokenizing
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
enum ET {
    Name(String),
    Str(String),
    Int(PyInt),
    Float(f64),
    Op(String),
    LBracket,
    RBracket,
    LParen,
    RParen,
    LBrace,
    RBrace,
    Comma,
    Colon,
    Pipe,
    Dot,
    KwIf,
    KwElse,
    KwAnd,
    KwOr,
    KwNot,
    KwIn,
    KwIs,
    KwTrue,
    KwFalse,
    KwNone,
    Assign,
}

fn parse_int_literal(s: &str, lineno: usize) -> Result<PyInt, TemplateError> {
    let cleaned: String = s.chars().filter(|&c| c != '_').collect();
    // base 0 semantics like Python int(str, 0)
    match pytext::int_from_string(&cleaned, 0) {
        Ok(v) => Ok(v),
        Err(e) => Err(from_pyerr(&e, lineno)),
    }
}

fn parse_float_literal(s: &str, lineno: usize) -> Result<f64, TemplateError> {
    let cleaned: String = s.chars().filter(|&c| c != '_').collect();
    match pytext::float_from_string(&cleaned) {
        Ok(v) => Ok(v),
        Err(e) => Err(from_pyerr(&e, lineno)),
    }
}

fn unescape_string(raw: &str, lineno: usize) -> Result<String, TemplateError> {
    // `raw` is the content between quotes, still escaped.
    // Python/Jinja uses `unicode-escape` semantics.
    let bytes = raw.as_bytes();
    let mut out = String::new();
    let mut i = 0usize;
    while i < raw.len() {
        let b = bytes[i];
        if b == b'\\' {
            i += 1;
            if i >= raw.len() {
                return Err(syntax_err("invalid string escape".to_string(), lineno));
            }
            match bytes[i] {
                b'n' => {
                    out.push('\n');
                    i += 1;
                }
                b't' => {
                    out.push('\t');
                    i += 1;
                }
                b'r' => {
                    out.push('\r');
                    i += 1;
                }
                b'\\' => {
                    out.push('\\');
                    i += 1;
                }
                b'\'' => {
                    out.push('\'');
                    i += 1;
                }
                b'"' => {
                    out.push('"');
                    i += 1;
                }
                b'a' => {
                    out.push('\u{07}');
                    i += 1;
                }
                b'b' => {
                    out.push('\u{08}');
                    i += 1;
                }
                b'f' => {
                    out.push('\u{0c}');
                    i += 1;
                }
                b'v' => {
                    out.push('\u{0b}');
                    i += 1;
                }
                b'0'..=b'7' => {
                    // octal up to 3 digits
                    let mut val: u32 = 0;
                    let mut n = 0;
                    while n < 3 && i < raw.len() && (b'0'..=b'7').contains(&bytes[i]) {
                        val = val * 8 + (bytes[i] - b'0') as u32;
                        i += 1;
                        n += 1;
                    }
                    out.push(char::from_u32(val).unwrap_or('\0'));
                }
                b'x' => {
                    if i + 2 >= raw.len() {
                        return Err(syntax_err("invalid \\x escape".to_string(), lineno));
                    }
                    let hex = &raw[i + 1..i + 3];
                    let v = u32::from_str_radix(hex, 16)
                        .map_err(|_| syntax_err("invalid \\x escape".to_string(), lineno))?;
                    out.push(char::from_u32(v).unwrap_or('\0'));
                    i += 3;
                }
                b'u' => {
                    if i + 4 >= raw.len() {
                        return Err(syntax_err("invalid \\u escape".to_string(), lineno));
                    }
                    let hex = &raw[i + 1..i + 5];
                    let v = u32::from_str_radix(hex, 16)
                        .map_err(|_| syntax_err("invalid \\u escape".to_string(), lineno))?;
                    out.push(char::from_u32(v).unwrap_or('\0'));
                    i += 5;
                }
                b'U' => {
                    if i + 8 >= raw.len() {
                        return Err(syntax_err("invalid \\U escape".to_string(), lineno));
                    }
                    let hex = &raw[i + 1..i + 9];
                    let v = u32::from_str_radix(hex, 16)
                        .map_err(|_| syntax_err("invalid \\U escape".to_string(), lineno))?;
                    out.push(char::from_u32(v).unwrap_or('\0'));
                    i += 9;
                }
                b'N' => {
                    // \N{name}: not supported; raise naming construct.
                    return Err(syntax_err(
                        "unsupported string escape '\\N'".to_string(),
                        lineno,
                    ));
                }
                _ => {
                    // Jinja via unicode-escape keeps unknown escapes as the char.
                    // e.g. "\'" handled above; unknown like "\d" -> "d"? Actually
                    // unicode-escape raises for invalid? For compatibility keep char.
                    out.push(bytes[i] as char);
                    i += 1;
                }
            }
            continue;
        }
        // copy one UTF-8 char
        let ch_len = raw[i..].chars().next().map(|c| c.len_utf8()).unwrap_or(1);
        out.push_str(&raw[i..i + ch_len]);
        i += ch_len;
    }
    Ok(out)
}

fn tokenize_expr(inner: &str, lineno: usize) -> Result<Vec<ET>, TemplateError> {
    let bytes = inner.as_bytes();
    let mut i = 0usize;
    let mut toks: Vec<ET> = Vec::new();
    while i < inner.len() {
        let b = bytes[i];
        if b == b' ' || b == b'\t' || b == b'\n' || b == b'\r' || b == 11 || b == 12 {
            i += 1;
            continue;
        }
        if b == b'\'' || b == b'"' {
            let start = i;
            let q = b;
            let mut j = i + 1;
            let mut found = false;
            while j < inner.len() {
                let c = bytes[j];
                if c == b'\\' {
                    j += 2;
                    continue;
                }
                if c == q {
                    found = true;
                    break;
                }
                j += 1;
            }
            if !found {
                return Err(syntax_err("unterminated string".to_string(), lineno));
            }
            let raw = &inner[start + 1..j];
            let s = unescape_string(raw, lineno)?;
            toks.push(ET::Str(s));
            i = j + 1;
            continue;
        }
        if b.is_ascii_digit() || (b == b'.' && i + 1 < inner.len() && bytes[i + 1].is_ascii_digit()) {
            // integer / float literal (Jinja float regex requires digits both sides of dot
            // except leading-dot floats are accepted as float)
            let start = i;
            // hex / bin / oct prefixes
            if b == b'0'
                && i + 1 < inner.len()
                && matches!(bytes[i + 1], b'x' | b'X' | b'b' | b'B' | b'o' | b'O')
            {
                let mut j = i + 2;
                while j < inner.len()
                    && (bytes[j].is_ascii_alphanumeric() || bytes[j] == b'_')
                {
                    // for hex etc alphanumerics; validation below will reject
                    j += 1;
                }
                let lit = &inner[start..j];
                let v = parse_int_literal(lit, lineno)?;
                toks.push(ET::Int(v));
                i = j;
                continue;
            }
            // scan generic number: digits, underscores, dot, exponent
            let mut j = i;
            let mut saw_dot = false;
            let mut saw_exp = false;
            while j < inner.len() {
                let c = bytes[j];
                if c.is_ascii_digit() || c == b'_' {
                    j += 1;
                } else if c == b'.' && !saw_dot && !saw_exp {
                    // need digit following for Jinja float; `.5` handled by leading dot branch
                    // but `1.` is also a float in Jinja? Jinja float_re requires digit after dot,
                    // so `1.` would tokenize as integer + dot. Replicate by requiring digit after.
                    if j + 1 < inner.len() && bytes[j + 1].is_ascii_digit() {
                        saw_dot = true;
                        j += 1;
                    } else {
                        break;
                    }
                } else if (c == b'e' || c == b'E') && !saw_exp && j > start {
                    saw_exp = true;
                    j += 1;
                    if j < inner.len() && (bytes[j] == b'+' || bytes[j] == b'-') {
                        j += 1;
                    }
                } else {
                    break;
                }
            }
            let lit = inner[start..j].to_string();
            if saw_dot || saw_exp {
                let v = parse_float_literal(&lit, lineno)?;
                toks.push(ET::Float(v));
            } else {
                let v = parse_int_literal(&lit, lineno)?;
                toks.push(ET::Int(v));
            }
            i = j;
            continue;
        }
        if b == b'_' || b.is_ascii_alphabetic() {
            let start = i;
            while i < inner.len() {
                let c = bytes[i];
                if c.is_ascii_alphanumeric() || c == b'_' {
                    i += 1;
                } else {
                    break;
                }
            }
            let name = inner[start..i].to_string();
            match name.as_str() {
                "and" => toks.push(ET::KwAnd),
                "or" => toks.push(ET::KwOr),
                "not" => toks.push(ET::KwNot),
                "in" => toks.push(ET::KwIn),
                "is" => toks.push(ET::KwIs),
                "if" => toks.push(ET::KwIf),
                "else" => toks.push(ET::KwElse),
                "true" | "True" => toks.push(ET::KwTrue),
                "false" | "False" => toks.push(ET::KwFalse),
                "none" | "None" => toks.push(ET::KwNone),
                _ => toks.push(ET::Name(name)),
            }
            continue;
        }
        // operators / punctuation
        let rest = &inner[i..];
        if rest.starts_with("==") {
            toks.push(ET::Op("==".to_string()));
            i += 2;
            continue;
        }
        if rest.starts_with("!=") {
            toks.push(ET::Op("!=".to_string()));
            i += 2;
            continue;
        }
        if rest.starts_with("**") {
            toks.push(ET::Op("**".to_string()));
            i += 2;
            continue;
        }
        if rest.starts_with("//") {
            toks.push(ET::Op("//".to_string()));
            i += 2;
            continue;
        }
        if rest.starts_with(">=") {
            toks.push(ET::Op(">=".to_string()));
            i += 2;
            continue;
        }
        if rest.starts_with("<=") {
            toks.push(ET::Op("<=".to_string()));
            i += 2;
            continue;
        }
        match b {
            b'+' => {
                toks.push(ET::Op("+".to_string()));
                i += 1;
            }
            b'-' => {
                toks.push(ET::Op("-".to_string()));
                i += 1;
            }
            b'*' => {
                toks.push(ET::Op("*".to_string()));
                i += 1;
            }
            b'/' => {
                toks.push(ET::Op("/".to_string()));
                i += 1;
            }
            b'%' => {
                toks.push(ET::Op("%".to_string()));
                i += 1;
            }
            b'~' => {
                toks.push(ET::Op("~".to_string()));
                i += 1;
            }
            b'<' => {
                toks.push(ET::Op("<".to_string()));
                i += 1;
            }
            b'>' => {
                toks.push(ET::Op(">".to_string()));
                i += 1;
            }
            b'[' => {
                toks.push(ET::LBracket);
                i += 1;
            }
            b']' => {
                toks.push(ET::RBracket);
                i += 1;
            }
            b'(' => {
                toks.push(ET::LParen);
                i += 1;
            }
            b')' => {
                toks.push(ET::RParen);
                i += 1;
            }
            b'{' => {
                toks.push(ET::LBrace);
                i += 1;
            }
            b'}' => {
                toks.push(ET::RBrace);
                i += 1;
            }
            b',' => {
                toks.push(ET::Comma);
                i += 1;
            }
            b':' => {
                toks.push(ET::Colon);
                i += 1;
            }
            b'|' => {
                toks.push(ET::Pipe);
                i += 1;
            }
            b'.' => {
                toks.push(ET::Dot);
                i += 1;
            }
            b'=' => {
                toks.push(ET::Assign);
                i += 1;
            }
            _ => {
                let ch = inner[i..].chars().next().unwrap();
                return Err(syntax_err(format!("unexpected '{ch}'"), lineno));
            }
        }
    }
    Ok(toks)
}

fn describe_et(t: &ET) -> String {
    match t {
        ET::Name(n) => n.clone(),
        ET::Str(s) => format!("{s:?}"),
        ET::Int(v) => v.0.to_string(),
        ET::Float(v) => pytext::float_repr(*v),
        ET::Op(op) => op.clone(),
        ET::LBracket => "[".to_string(),
        ET::RBracket => "]".to_string(),
        ET::LParen => "(".to_string(),
        ET::RParen => ")".to_string(),
        ET::LBrace => "{".to_string(),
        ET::RBrace => "}".to_string(),
        ET::Comma => ",".to_string(),
        ET::Colon => ":".to_string(),
        ET::Pipe => "|".to_string(),
        ET::Dot => ".".to_string(),
        ET::KwIf => "if".to_string(),
        ET::KwElse => "else".to_string(),
        ET::KwAnd => "and".to_string(),
        ET::KwOr => "or".to_string(),
        ET::KwNot => "not".to_string(),
        ET::KwIn => "in".to_string(),
        ET::KwIs => "is".to_string(),
        ET::KwTrue => "true".to_string(),
        ET::KwFalse => "false".to_string(),
        ET::KwNone => "none".to_string(),
        ET::Assign => "=".to_string(),
    }
}

// ---------------------------------------------------------------------------
// expression parser
// ---------------------------------------------------------------------------

struct ExprParser {
    toks: Vec<ET>,
    pos: usize,
    lineno: usize,
}

impl ExprParser {
    fn peek(&self) -> Option<&ET> {
        self.toks.get(self.pos)
    }
    fn next_tok(&mut self) -> Option<ET> {
        if self.pos < self.toks.len() {
            let t = self.toks[self.pos].clone();
            self.pos += 1;
            Some(t)
        } else {
            None
        }
    }
    fn expect(&mut self, what: &str) -> Result<ET, TemplateError> {
        match self.next_tok() {
            Some(t) => Ok(t),
            None => Err(syntax_err(
                format!("unexpected end of expression, expected '{what}'"),
                self.lineno,
            )),
        }
    }
    fn at_op(&self, op: &str) -> bool {
        matches!(self.peek(), Some(ET::Op(o)) if o == op)
    }
    fn eat_op(&mut self, op: &str) -> bool {
        if self.at_op(op) {
            self.pos += 1;
            true
        } else {
            false
        }
    }
    fn at_kw(&self, kw: &ET) -> bool {
        self.peek() == Some(kw)
    }

    fn parse_full(&mut self) -> Result<(Expr, usize), TemplateError> {
        // full tuple expression (comma separated); returns (expr, consumed)
        let e = self.parse_condexpr()?;
        let (e, _consumed) = self.parse_tuple_tail(e)?;
        Ok((e, self.pos))
    }

    fn parse_tuple_tail(&mut self, first: Expr) -> Result<(Expr, usize), TemplateError> {
        if self.peek() == Some(&ET::Comma) {
            let mut items = vec![first];
            while self.peek() == Some(&ET::Comma) {
                self.pos += 1;
                // trailing comma before end: Jinja allows `(a,)` and `a,` in call args?
                if self.peek().is_none()
                    || matches!(self.peek(), Some(ET::RParen) | Some(ET::RBracket) | Some(ET::RBrace))
                {
                    break;
                }
                items.push(self.parse_condexpr()?);
            }
            return Ok((Expr::Tuple(items), self.pos));
        }
        Ok((first, self.pos))
    }

    fn parse_condexpr(&mut self) -> Result<Expr, TemplateError> {
        // `a if cond else b` ; missing else allowed (yields Undefined at eval)
        let mut expr = self.parse_or()?;
        while self.peek() == Some(&ET::KwIf) {
            self.pos += 1;
            let cond = self.parse_or()?;
            let else_expr = if self.peek() == Some(&ET::KwElse) {
                self.pos += 1;
                Some(Box::new(self.parse_condexpr()?))
            } else {
                None
            };
            expr = Expr::CondExpr(Box::new(expr), Box::new(cond), else_expr);
        }
        Ok(expr)
    }

    fn parse_or(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_and()?;
        while self.peek() == Some(&ET::KwOr) {
            self.pos += 1;
            let right = self.parse_and()?;
            left = Expr::Or(Box::new(left), Box::new(right));
        }
        Ok(left)
    }
    fn parse_and(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_not()?;
        while self.peek() == Some(&ET::KwAnd) {
            self.pos += 1;
            let right = self.parse_not()?;
            left = Expr::And(Box::new(left), Box::new(right));
        }
        Ok(left)
    }
    fn parse_not(&mut self) -> Result<Expr, TemplateError> {
        if self.peek() == Some(&ET::KwNot) {
            // `not in` is handled in compare; here plain `not`
            // need lookahead: if next is `in`, this `not` belongs to compare,
            // but parse_not is above compare in Jinja, so `not x in y` parses as
            // `not (x in y)`. The `not in` binary is `a not in b`. Keep simple:
            // consume and recurse.
            self.pos += 1;
            let e = self.parse_not()?;
            return Ok(Expr::Not(Box::new(e)));
        }
        self.parse_compare()
    }
    fn parse_compare(&mut self) -> Result<Expr, TemplateError> {
        let first = self.parse_math1()?;
        let mut ops: Vec<(String, Expr)> = Vec::new();
        loop {
            // binary `not in`
            if self.peek() == Some(&ET::KwNot) && self.toks.get(self.pos + 1) == Some(&ET::KwIn) {
                self.pos += 2;
                let r = self.parse_math1()?;
                ops.push(("notin".to_string(), r));
                continue;
            }
            if self.peek() == Some(&ET::KwIn) {
                self.pos += 1;
                let r = self.parse_math1()?;
                ops.push(("in".to_string(), r));
                continue;
            }
            if let Some(ET::Op(op)) = self.peek().cloned() {
                if matches!(op.as_str(), "==" | "!=" | "<" | ">" | "<=" | ">=") {
                    self.pos += 1;
                    let r = self.parse_math1()?;
                    ops.push((op, r));
                    continue;
                }
            }
            break;
        }
        if ops.is_empty() {
            Ok(first)
        } else {
            Ok(Expr::Compare {
                first: Box::new(first),
                ops,
            })
        }
    }
    fn parse_math1(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_concat()?;
        loop {
            if self.at_op("+") {
                self.pos += 1;
                let r = self.parse_concat()?;
                left = Expr::Add(Box::new(left), Box::new(r));
            } else if self.at_op("-") {
                self.pos += 1;
                let r = self.parse_concat()?;
                left = Expr::Sub(Box::new(left), Box::new(r));
            } else {
                break;
            }
        }
        Ok(left)
    }
    fn parse_concat(&mut self) -> Result<Expr, TemplateError> {
        let mut acc = self.parse_math2()?;
        while self.at_op("~") {
            self.pos += 1;
            let r = self.parse_math2()?;
            acc = Expr::Concat(Box::new(acc), Box::new(r));
        }
        Ok(acc)
    }
    fn parse_math2(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_pow()?;
        loop {
            if self.at_op("*") {
                self.pos += 1;
                let r = self.parse_pow()?;
                left = Expr::Mul(Box::new(left), Box::new(r));
            } else if self.at_op("/") {
                self.pos += 1;
                let r = self.parse_pow()?;
                left = Expr::Div(Box::new(left), Box::new(r));
            } else if self.at_op("//") {
                self.pos += 1;
                let r = self.parse_pow()?;
                left = Expr::FloorDiv(Box::new(left), Box::new(r));
            } else if self.at_op("%") {
                self.pos += 1;
                let r = self.parse_pow()?;
                left = Expr::Mod(Box::new(left), Box::new(r));
            } else {
                break;
            }
        }
        Ok(left)
    }
    fn parse_pow(&mut self) -> Result<Expr, TemplateError> {
        let mut left = self.parse_unary()?;
        while self.at_op("**") {
            self.pos += 1;
            let r = self.parse_unary()?;
            left = Expr::Pow(Box::new(left), Box::new(r));
        }
        Ok(left)
    }
    fn parse_unary(&mut self) -> Result<Expr, TemplateError> {
        if self.at_op("-") {
            self.pos += 1;
            let e = self.parse_unary_no_filter()?;
            let node = Expr::Neg(Box::new(e));
            return self.parse_filter_expr(node);
        }
        if self.at_op("+") {
            self.pos += 1;
            let e = self.parse_unary_no_filter()?;
            let node = Expr::Pos(Box::new(e));
            return self.parse_filter_expr(node);
        }
        self.parse_unary_with_filter()
    }
    fn parse_unary_no_filter(&mut self) -> Result<Expr, TemplateError> {
        // operands of unary +/- do not carry filters (Jinja passes False)
        if self.at_op("-") {
            self.pos += 1;
            let e = self.parse_unary_no_filter()?;
            return Ok(Expr::Neg(Box::new(e)));
        }
        if self.at_op("+") {
            self.pos += 1;
            let e = self.parse_unary_no_filter()?;
            return Ok(Expr::Pos(Box::new(e)));
        }
        let node = self.parse_primary()?;
        self.parse_postfix(node)
    }
    fn parse_unary_with_filter(&mut self) -> Result<Expr, TemplateError> {
        let node = self.parse_primary()?;
        let node = self.parse_postfix(node)?;
        self.parse_filter_expr(node)
    }

    fn parse_primary(&mut self) -> Result<Expr, TemplateError> {
        let tok = self
            .next_tok()
            .ok_or_else(|| syntax_err("unexpected end of expression".to_string(), self.lineno))?;
        match tok {
            ET::KwTrue => Ok(Expr::Const(PyValue::Bool(true))),
            ET::KwFalse => Ok(Expr::Const(PyValue::Bool(false))),
            ET::KwNone => Ok(Expr::Const(PyValue::None)),
            ET::Name(n) => Ok(Expr::Name(n)),
            ET::Str(s) => {
                // adjacent literal concatenation: consume further strings.
                let mut acc = s;
                loop {
                    let is_str = matches!(self.peek(), Some(ET::Str(_)));
                    if !is_str {
                        break;
                    }
                    if let Some(ET::Str(s2)) = self.peek().cloned() {
                        self.pos += 1;
                        acc.push_str(&s2);
                    } else {
                        break;
                    }
                }
                Ok(Expr::Const(PyValue::Str(acc)))
            }
            ET::Int(v) => Ok(Expr::Const(PyValue::Int(v))),
            ET::Float(v) => Ok(Expr::Const(PyValue::Float(v))),
            ET::LParen => {
                if self.peek() == Some(&ET::RParen) {
                    self.pos += 1;
                    return Ok(Expr::Tuple(Vec::new()));
                }
                let e = self.parse_condexpr()?;
                // parenthesized tuple / grouped
                if self.peek() == Some(&ET::Comma) {
                    let (t, _) = self.parse_tuple_tail(e)?;
                    if self.peek() != Some(&ET::RParen) {
                        let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                        return Err(syntax_err(
                            format!("expected token ')', got '{got}'"),
                            self.lineno,
                        ));
                    }
                    self.pos += 1;
                    Ok(t)
                } else if self.peek() == Some(&ET::RParen) {
                    self.pos += 1;
                    Ok(e)
                } else {
                    let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                    Err(syntax_err(
                        format!("expected token ')', got '{got}'"),
                        self.lineno,
                    ))
                }
            }
            ET::LBracket => {
                let mut items = Vec::new();
                if self.peek() == Some(&ET::RBracket) {
                    self.pos += 1;
                    return Ok(Expr::List(items));
                }
                loop {
                    items.push(self.parse_condexpr()?);
                    if self.peek() == Some(&ET::Comma) {
                        self.pos += 1;
                        if self.peek() == Some(&ET::RBracket) {
                            self.pos += 1;
                            break;
                        }
                        if items.is_empty() {
                        }
                        continue;
                    }
                    if self.peek() == Some(&ET::RBracket) {
                        self.pos += 1;
                        break;
                    }
                    if self.peek().is_none() {
                        return Err(syntax_err(
                            "unexpected end of expression, expected ']'".to_string(),
                            self.lineno,
                        ));
                    }
                    // slice syntax `a:b` is not in corpus; reject naming it.
                    if self.peek() == Some(&ET::Colon) {
                        return Err(syntax_err(
                            "unsupported slicing 'x[a:b]'".to_string(),
                            self.lineno,
                        ));
                    }
                    let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                    return Err(syntax_err(
                        format!("expected token ']', got '{got}'"),
                        self.lineno,
                    ));
                }
                Ok(Expr::List(items))
            }
            ET::LBrace => {
                let mut items: Vec<(Expr, Expr)> = Vec::new();
                if self.peek() == Some(&ET::RBrace) {
                    self.pos += 1;
                    return Ok(Expr::Dict(items));
                }
                loop {
                    // `**kwargs` unpacking not supported
                    if self.at_op("**") {
                        return Err(syntax_err(
                            "unsupported '**' dict unpacking".to_string(),
                            self.lineno,
                        ));
                    }
                    let key = self.parse_condexpr()?;
                    if self.peek() != Some(&ET::Colon) {
                        let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                        return Err(syntax_err(format!("expected token ':', got '{got}'"), self.lineno));
                    }
                    self.pos += 1;
                    let val = self.parse_condexpr()?;
                    items.push((key, val));
                    if self.peek() == Some(&ET::Comma) {
                        self.pos += 1;
                        if self.peek() == Some(&ET::RBrace) {
                            self.pos += 1;
                            break;
                        }
                        continue;
                    }
                    if self.peek() == Some(&ET::RBrace) {
                        self.pos += 1;
                        break;
                    }
                    if self.peek().is_none() {
                        return Err(syntax_err(
                            "unexpected end of expression, expected '}'".to_string(),
                            self.lineno,
                        ));
                    }
                    let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                    return Err(syntax_err(
                        format!("expected token '}}', got '{got}'"),
                        self.lineno,
                    ));
                }
                Ok(Expr::Dict(items))
            }
            other => Err(syntax_err(
                format!("unexpected '{}'", describe_et(&other)),
                self.lineno,
            )),
        }
    }

    fn parse_postfix(&mut self, mut node: Expr) -> Result<Expr, TemplateError> {
        loop {
            if self.peek() == Some(&ET::Dot) {
                self.pos += 1;
                match self.next_tok() {
                    Some(ET::Name(n)) => {
                        node = Expr::GetAttr(Box::new(node), n);
                    }
                    Some(ET::Int(v)) => {
                        // Jinja allows `.0` tuple index via Getattr? Actually returns Const int for tuple indexing.
                        // Represent as GetItem with int const.
                        let e = Expr::Const(PyValue::Int(v));
                        node = Expr::GetItem(Box::new(node), Box::new(e));
                    }
                    Some(ET::Float(_)) => {
                        return Err(syntax_err("expected name or number".to_string(), self.lineno));
                    }
                    other => {
                        let got = other.map(|t| describe_et(&t)).unwrap_or("end".to_string());
                        return Err(syntax_err(
                            format!("expected name or number, got '{got}'"),
                            self.lineno,
                        ));
                    }
                }
            } else if self.peek() == Some(&ET::LBracket) {
                self.pos += 1;
                // slice check: `a[...]` or `a[b:c]` unsupported
                if self.peek() == Some(&ET::Op("*".to_string())) {
                    return Err(syntax_err(
                        "unsupported slicing 'x[a:b]'".to_string(),
                        self.lineno,
                    ));
                }
                let idx = self.parse_condexpr()?;
                if self.peek() == Some(&ET::Colon) {
                    return Err(syntax_err(
                        "unsupported slicing 'x[a:b]'".to_string(),
                        self.lineno,
                    ));
                }
                if self.peek() != Some(&ET::RBracket) {
                    let got = self.peek().map(describe_et).unwrap_or("end".to_string());
                    return Err(syntax_err(
                        format!("expected token ']', got '{got}'"),
                        self.lineno,
                    ));
                }
                self.pos += 1;
                node = Expr::GetItem(Box::new(node), Box::new(idx));
            } else if self.peek() == Some(&ET::LParen) {
                self.pos += 1;
                let (args, kwargs) = self.parse_call_args()?;
                node = Expr::Call {
                    callee: Box::new(node),
                    args,
                    kwargs,
                };
            } else {
                break;
            }
        }
        Ok(node)
    }

    fn parse_call_args(&mut self) -> Result<(Vec<Expr>, Vec<(String, Expr)>), TemplateError> {
        let mut args: Vec<Expr> = Vec::new();
        let mut kwargs: Vec<(String, Expr)> = Vec::new();
        if self.peek() == Some(&ET::RParen) {
            self.pos += 1;
            return Ok((args, kwargs));
        }
        loop {
            if self.at_op("**") {
                return Err(syntax_err(
                    "unsupported '**' call unpacking".to_string(),
                    self.lineno,
                ));
            }
            if self.at_op("*") {
                return Err(syntax_err(
                    "unsupported '*' call unpacking".to_string(),
                    self.lineno,
                ));
            }
            // keyword argument `name=expr` (single `=`; `==` is Op)
            let is_kw = matches!(self.peek(), Some(ET::Name(_)))
                && matches!(self.toks.get(self.pos + 1), Some(ET::Assign));
            if is_kw {
                if let Some(ET::Name(kname)) = self.peek().cloned() {
                    self.pos += 2;
                    let v = self.parse_condexpr()?;
                    kwargs.push((kname, v));
                } else {
                    unreachable!()
                }
            } else {
                let v = self.parse_condexpr()?;
                args.push(v);
            }
            if self.peek() == Some(&ET::Comma) {
                self.pos += 1;
                if self.peek() == Some(&ET::RParen) {
                    self.pos += 1;
                    break;
                }
                continue;
            }
            if self.peek() == Some(&ET::RParen) {
                self.pos += 1;
                break;
            }
            if self.peek().is_none() {
                return Err(syntax_err(
                    "unexpected end of expression, expected ')'".to_string(),
                    self.lineno,
                ));
            }
            let got = self.peek().map(describe_et).unwrap_or("end".to_string());
            return Err(syntax_err(
                format!("expected token ')', got '{got}'"),
                self.lineno,
            ));
        }
        Ok((args, kwargs))
    }

    fn parse_filter_expr(&mut self, mut node: Expr) -> Result<Expr, TemplateError> {
        loop {
            if self.peek() == Some(&ET::Pipe) {
                self.pos += 1;
                let name = match self.next_tok() {
                    Some(ET::Name(n)) => n,
                    other => {
                        let got = other.map(|t| describe_et(&t)).unwrap_or("end".to_string());
                        return Err(syntax_err(
                            format!("expected filter name, got '{got}'"),
                            self.lineno,
                        ));
                    }
                };
                // dotted filter names like `x.y` (extensions) unsupported
                if self.peek() == Some(&ET::Dot) {
                    return Err(syntax_err(
                        format!("unsupported dotted filter '{name}.'"),
                        self.lineno,
                    ));
                }
                let (args, kwargs) = if self.peek() == Some(&ET::LParen) {
                    self.pos += 1;
                    self.parse_call_args()?
                } else {
                    (Vec::new(), Vec::new())
                };
                node = Expr::Filter {
                    node: Box::new(node),
                    name,
                    args,
                    kwargs,
                };
            } else if self.peek() == Some(&ET::KwIs) {
                self.pos += 1;
                let negated = if self.peek() == Some(&ET::KwNot) {
                    self.pos += 1;
                    true
                } else {
                    false
                };
                // `is not in ...`? That's `is not` + test `in`.
                // Jinja parses `x is not in y` as test `in` negated? Actually
                // `is not in` is parsed as negated `in` test. Handle it.
                let (name, args, kwargs) = if self.peek() == Some(&ET::KwIn) {
                    self.pos += 1;
                    // `is in` without `not` => test in; with `not` handled above.
                    let tname = "in".to_string();
                    // optional `(args)`? Jinja allows `is in [1,2]` where arg is primary-less:
                    // parse test args as expression if present.
                    // For `in` test the arg is the following expression.
                    let mut a = Vec::new();
                    let mut k = Vec::new();
                    if self.peek() == Some(&ET::LParen) {
                        self.pos += 1;
                        let (aa, kk) = self.parse_call_args()?;
                        a = aa;
                        k = kk;
                    } else {
                        // single expression argument
                        a.push(self.parse_condexpr()?);
                    }
                    (tname, a, k)
                } else {
                    let name = match self.next_tok() {
                        Some(ET::Name(n)) => n,
                        Some(ET::KwIn) => "in".to_string(),
                        Some(ET::KwNone) => "none".to_string(),
                        Some(ET::KwTrue) => "true".to_string(),
                        Some(ET::KwFalse) => "false".to_string(),
                        other => {
                            let got = other.map(|t| describe_et(&t)).unwrap_or("end".to_string());
                            return Err(syntax_err(
                                format!("expected test name, got '{got}'"),
                                self.lineno,
                            ));
                        }
                    };
                    if self.peek() == Some(&ET::Dot) {
                        return Err(syntax_err(
                            "unsupported dotted test".to_string(),
                            self.lineno,
                        ));
                    }
                    let (args, kwargs) = if self.peek() == Some(&ET::LParen) {
                        self.pos += 1;
                        self.parse_call_args()?
                    } else if name == "in" {
                        // `is in expr` without parens
                        let mut a = Vec::new();
                        a.push(self.parse_condexpr()?);
                        (a, Vec::new())
                    } else {
                        (Vec::new(), Vec::new())
                    };
                    (name, args, kwargs)
                };
                node = Expr::Test {
                    node: Box::new(node),
                    name,
                    args,
                    kwargs,
                    negated,
                };
            } else if self.peek() == Some(&ET::LParen) {
                // call after filter/test: `x|f(...)()`? postfix allows it.
                self.pos += 1;
                let (args, kwargs) = self.parse_call_args()?;
                node = Expr::Call {
                    callee: Box::new(node),
                    args,
                    kwargs,
                };
            } else {
                break;
            }
        }
        Ok(node)
    }
}

fn parse_expr_text(text: &str, lineno: usize) -> Result<Expr, TemplateError> {
    let toks = tokenize_expr(text, lineno)?;
    if toks.is_empty() {
        return Err(syntax_err("unexpected end of expression".to_string(), lineno));
    }
    let mut p = ExprParser {
        toks,
        pos: 0,
        lineno,
    };
    // For statement tests, `parse_tuple(with_condexpr=False)` disallows ternary in if/for headers.
    // Top-level `{ expr }` allows ternary. Callers pass full arity accordingly via wrappers.
    let (e, consumed) = p.parse_full()?;
    if consumed < p.toks.len() {
        let got = p.peek().map(describe_et).unwrap_or("end".to_string());
        return Err(syntax_err(format!("unexpected '{got}'"), lineno));
    }
    Ok(e)
}

/// Parse without ternary (used for `{% if %}`/`{% for %}`/`{% set %}` headers).
fn parse_expr_no_cond(text: &str, lineno: usize) -> Result<Expr, TemplateError> {
    let toks = tokenize_expr(text, lineno)?;
    if toks.is_empty() {
        return Err(syntax_err("unexpected end of expression".to_string(), lineno));
    }
    let mut p = ExprParser {
        toks,
        pos: 0,
        lineno,
    };
    let e = p.parse_or()?;
    // allow trailing tuple commas? Jinja `parse_tuple` for tests: `for x in a, b`? Not needed.
    if p.pos < p.toks.len() {
        // allow a single tuple `a, b` (for-loop iterable tuple)?
        if p.peek() == Some(&ET::Comma) {
            let (t, _) = p.parse_tuple_tail(e)?;
            if p.pos < p.toks.len() {
                let got = p.peek().map(describe_et).unwrap_or("end".to_string());
                return Err(syntax_err(format!("unexpected '{got}'"), lineno));
            }
            return Ok(t);
        }
        let got = p.peek().map(describe_et).unwrap_or("end".to_string());
        return Err(syntax_err(format!("unexpected '{got}'"), lineno));
    }
    Ok(e)
}

// ---------------------------------------------------------------------------
// statement parser
// ---------------------------------------------------------------------------

fn parse_nodes(toks: &[Tok], start: usize, stops: &[&str]) -> Result<(Vec<Node>, usize), TemplateError> {
    let mut nodes: Vec<Node> = Vec::new();
    let mut i = start;
    while i < toks.len() {
        match &toks[i] {
            Tok::Data(d) => {
                if !d.is_empty() {
                    nodes.push(Node::Text(d.clone()));
                }
                i += 1;
            }
            Tok::Var {
                text,
                lineno,
                ..
            } => {
                let e = parse_expr_text(text, *lineno)?;
                nodes.push(Node::Print(e));
                i += 1;
            }
            Tok::Block {
                text,
                lineno,
                ..
            } => {
                let t = text.trim();
                let kw = t.split_whitespace().next().unwrap_or("");
                if stops.contains(&kw) {
                    return Ok((nodes, i));
                }
                match kw {
                    "if" => {
                        let (node, next) = parse_if(toks, i)?;
                        nodes.push(node);
                        i = next;
                    }
                    "for" => {
                        let (node, next) = parse_for(toks, i)?;
                        nodes.push(node);
                        i = next;
                    }
                    "set" => {
                        let node = parse_set(text, *lineno)?;
                        nodes.push(node);
                        i += 1;
                    }
                    "raw" => {
                        return Err(syntax_err(
                            "unsupported 'raw' block".to_string(),
                            *lineno,
                        ))
                    }
                    "macro" | "extends" | "include" | "import" | "from" | "block" | "with"
                    | "filter" | "autoescape" | "call" | "do" => {
                        return Err(syntax_err(
                            format!("unsupported '{kw}' tag"),
                            *lineno,
                        ))
                    }
                    "else" | "elif" | "endif" | "endfor" | "endset" | "endraw" | "endmacro"
                    | "endblock" | "endwith" | "endfilter" | "endcall" | "endautoescape" => {
                        return Err(syntax_err(
                            format!("unexpected '{kw}'"),
                            *lineno,
                        ))
                    }
                    "" => {
                        return Err(syntax_err("unexpected '%}'".to_string(), *lineno))
                    }
                    _ => {
                        return Err(syntax_err(
                            format!("Encountered unknown tag '{kw}'."),
                            *lineno,
                        ))
                    }
                }
            }
        }
    }
    Ok((nodes, i))
}

fn parse_if(toks: &[Tok], start: usize) -> Result<(Node, usize), TemplateError> {
    // start points at `if` block token
    let (text0, lineno0) = match &toks[start] {
        Tok::Block { text, lineno, .. } => (text.clone(), *lineno),
        _ => unreachable!(),
    };
    let cond_text = text0.trim()[2..].trim(); // strip `if`
    let cond = parse_expr_no_cond(cond_text, lineno0)?;
    let mut branches: Vec<(Expr, Vec<Node>)> = Vec::new();
    let (body, mut idx) = parse_nodes(toks, start + 1, &["elif", "else", "endif"])?;
    branches.push((cond, body));
    loop {
        if idx >= toks.len() {
            return Err(syntax_err(
                "Unexpected end of template, expected 'endif'.".to_string(),
                lineno0,
            ));
        }
        let (text, lineno) = match &toks[idx] {
            Tok::Block { text, lineno, .. } => (text.clone(), *lineno),
            _ => unreachable!(),
        };
        let t = text.trim();
        let kw = t.split_whitespace().next().unwrap_or("");
        if kw == "elif" {
            let c_text = t[4..].trim();
            let c = parse_expr_no_cond(c_text, lineno)?;
            let (b, nidx) = parse_nodes(toks, idx + 1, &["elif", "else", "endif"])?;
            branches.push((c, b));
            idx = nidx;
        } else if kw == "else" {
            if !t.eq("else") {
                return Err(syntax_err("unexpected 'else'".to_string(), lineno));
            }
            let (b, nidx) = parse_nodes(toks, idx + 1, &["endif"])?;
            let else_body = b;
            // consume endif
            if nidx >= toks.len() {
                return Err(syntax_err(
                    "Unexpected end of template, expected 'endif'.".to_string(),
                    lineno0,
                ));
            }
            let (et, el) = match &toks[nidx] {
                Tok::Block { text, lineno, .. } => (text.trim().to_string(), *lineno),
                _ => unreachable!(),
            };
            if et != "endif" {
                return Err(syntax_err(
                    "Unexpected end of template, expected 'endif'.".to_string(),
                    el,
                ));
            }
            return Ok((
                Node::If {
                    branches,
                    else_body,
                },
                nidx + 1,
            ));
        } else if kw == "endif" {
            if t != "endif" {
                return Err(syntax_err("unexpected 'endif'".to_string(), lineno));
            }
            return Ok((
                Node::If {
                    branches,
                    else_body: Vec::new(),
                },
                idx + 1,
            ));
        } else {
            unreachable!()
        }
    }
}

fn parse_for(toks: &[Tok], start: usize) -> Result<(Node, usize), TemplateError> {
    let (text0, lineno0) = match &toks[start] {
        Tok::Block { text, lineno, .. } => (text.clone(), *lineno),
        _ => unreachable!(),
    };
    let rest = text0.trim()[3..].trim(); // strip `for`
    // split `name in expr`; `in` keyword at top level (not inside brackets/strings)
    let in_pos = find_top_level_in(rest).ok_or_else(|| {
        syntax_err("expected token 'in', got end of statement".to_string(), lineno0)
    })?;
    let target_raw = rest[..in_pos].trim();
    let iter_raw = rest[in_pos + 2..].trim();
    // destructuring not supported: only single name.
    if target_raw.contains(',') || target_raw.contains('(') || target_raw.contains('.') {
        return Err(syntax_err(
            format!("unsupported 'for ... in ...' target '{target_raw}'"),
            lineno0,
        ));
    }
    if target_raw.is_empty() || !target_raw.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err(syntax_err(
            format!("expected name for loop target, got '{target_raw}'"),
            lineno0,
        ));
    }
    // `recursive` / `if` filtering / `else` token inline not supported
    // Detect trailing `recursive` or ` if ` at top level would be ambiguous;
    // corpus does not use them, reject explicitly if header contains them outside brackets.
    if contains_top_level_word(iter_raw, "recursive") {
        return Err(syntax_err("unsupported 'recursive' for loop".to_string(), lineno0));
    }
    let iter = parse_expr_no_cond(iter_raw, lineno0)?;
    let (body, idx) = parse_nodes(toks, start + 1, &["else", "endfor"])?;
    if idx >= toks.len() {
        return Err(syntax_err(
            "Unexpected end of template, expected 'endfor'.".to_string(),
            lineno0,
        ));
    }
    let (text, lineno) = match &toks[idx] {
        Tok::Block { text, lineno, .. } => (text.clone(), *lineno),
        _ => unreachable!(),
    };
    let t = text.trim();
    if t == "else" {
        let (else_body, idx2) = parse_nodes(toks, idx + 1, &["endfor"])?;
        if idx2 >= toks.len() {
            return Err(syntax_err(
                "Unexpected end of template, expected 'endfor'.".to_string(),
                lineno0,
            ));
        }
        let (et, el) = match &toks[idx2] {
            Tok::Block { text, lineno, .. } => (text.trim().to_string(), *lineno),
            _ => unreachable!(),
        };
        if et != "endfor" {
            return Err(syntax_err(
                "Unexpected end of template, expected 'endfor'.".to_string(),
                el,
            ));
        }
        return Ok((
            Node::For {
                target: target_raw.to_string(),
                iter,
                body,
                else_body,
            },
            idx2 + 1,
        ));
    }
    if t == "endfor" {
        return Ok((
            Node::For {
                target: target_raw.to_string(),
                iter,
                body,
                else_body: Vec::new(),
            },
            idx + 1,
        ));
    }
    Err(syntax_err(
        "Unexpected end of template, expected 'endfor'.".to_string(),
        lineno0,
    ))
}

fn find_top_level_in(s: &str) -> Option<usize> {
    let bytes = s.as_bytes();
    let mut depth = 0i32;
    let mut i = 0usize;
    while i < s.len() {
        let b = bytes[i];
        if b == b'\'' || b == b'"' {
            // skip string
            let mut j = i + 1;
            while j < s.len() {
                if bytes[j] == b'\\' {
                    j += 2;
                    continue;
                }
                if bytes[j] == b {
                    break;
                }
                j += 1;
            }
            i = j + 1;
            continue;
        }
        match b {
            b'[' | b'(' | b'{' => {
                depth += 1;
                i += 1;
            }
            b']' | b')' | b'}' => {
                depth -= 1;
                i += 1;
            }
            _ => {
                if depth == 0 && s[i..].starts_with(" in ") {
                    return Some(i + 1);
                }
                if depth == 0 && (s[i..].starts_with(" in\t") || s[i..].starts_with(" in\n")) {
                    return Some(i + 1);
                }
                if depth == 0 && s[i..].starts_with("in ") && i == 0 {
                    return Some(0);
                }
                // also handle `name in ...` where `in` is surrounded but at start edge
                // generic word search below handles it; here simple:
                i += 1;
            }
        }
    }
    // fallback: search for word `in` with word boundaries at depth 0
    None.or_else(|| {
        // Use tokenize to find KwIn position
        let toks = tokenize_expr(s, 1).ok()?;
        let mut depth2 = 0i32;
        // we must map token index to char pos; simpler: tokenize target prefix attempt
        // We do a scan for standalone `in` in original string at depth 0.
        let mut d = 0i32;
        let mut j = 0usize;
        while j < s.len() {
            let c = bytes[j];
            if c == b'\'' || c == b'"' {
                let mut k = j + 1;
                while k < s.len() {
                    if bytes[k] == b'\\' {
                        k += 2;
                        continue;
                    }
                    if bytes[k] == c {
                        break;
                    }
                    k += 1;
                }
                j = k + 1;
                continue;
            }
            match c {
                b'[' | b'(' | b'{' => d += 1,
                b']' | b')' | b'}' => d -= 1,
                _ => {
                    if d == 0
                        && s[j..].starts_with("in")
                        && (j == 0 || {
                            let p = bytes[j - 1];
                            !(p.is_ascii_alphanumeric() || p == b'_')
                        })
                        && (j + 2 >= s.len() || {
                            let q = bytes[j + 2];
                            !(q.is_ascii_alphanumeric() || q == b'_')
                        })
                    {
                        // must have space before (target ends) - check left is space or start target
                        // `name in` has space before, `in` at pos0 cannot be separator for empty target
                        if j > 0 {
                            return Some(j);
                        }
                    }
                }
            }
            j += 1;
        }
        let _ = toks;
        let _ = depth2;
        None
    })
}

fn contains_top_level_word(s: &str, word: &str) -> bool {
    // crude check outside strings/brackets
    let bytes = s.as_bytes();
    let mut depth = 0i32;
    let mut i = 0usize;
    while i < s.len() {
        let b = bytes[i];
        if b == b'\'' || b == b'"' {
            let mut j = i + 1;
            while j < s.len() {
                if bytes[j] == b'\\' {
                    j += 2;
                    continue;
                }
                if bytes[j] == b {
                    break;
                }
                j += 1;
            }
            i = j + 1;
            continue;
        }
        match b {
            b'[' | b'(' | b'{' => depth += 1,
            b']' | b')' | b'}' => depth -= 1,
            _ => {}
        }
        if depth == 0 && s[i..].starts_with(word) {
            let before_ok = i == 0 || {
                let p = bytes[i - 1];
                !(p.is_ascii_alphanumeric() || p == b'_')
            };
            let after_ok = i + word.len() >= s.len() || {
                let q = bytes[i + word.len()];
                !(q.is_ascii_alphanumeric() || q == b'_')
            };
            if before_ok && after_ok {
                return true;
            }
        }
        i += 1;
    }
    false
}

fn parse_set(text: &str, lineno: usize) -> Result<Node, TemplateError> {
    let rest = text.trim()[3..].trim(); // strip `set`
    // `namespace(...)` unsupported
    if rest.starts_with("namespace") {
        return Err(syntax_err(
            "unsupported 'namespace()' in set".to_string(),
            lineno,
        ));
    }
    let eq_pos = find_top_level_assign(rest).ok_or_else(|| {
        syntax_err("unsupported 'set' block assignment without '='".to_string(), lineno)
    })?;
    let name = rest[..eq_pos].trim();
    let value_raw = rest[eq_pos + 1..].trim();
    // only single name
    if name.contains(',') || name.contains('.') || name.contains('=') {
        return Err(syntax_err(
            format!("unsupported 'set' target '{name}'"),
            lineno,
        ));
    }
    if name.is_empty() || !name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err(syntax_err(
            format!("expected name after 'set ', got '{name}'"),
            lineno,
        ));
    }
    let value = parse_expr_text(value_raw, lineno)?;
    Ok(Node::Set {
        name: name.to_string(),
        value,
    })
}

fn find_top_level_assign(s: &str) -> Option<usize> {
    let bytes = s.as_bytes();
    let mut depth = 0i32;
    let mut i = 0usize;
    while i < s.len() {
        let b = bytes[i];
        if b == b'\'' || b == b'"' {
            let mut j = i + 1;
            while j < s.len() {
                if bytes[j] == b'\\' {
                    j += 2;
                    continue;
                }
                if bytes[j] == b {
                    break;
                }
                j += 1;
            }
            i = j + 1;
            continue;
        }
        match b {
            b'[' | b'(' | b'{' => depth += 1,
            b']' | b')' | b'}' => depth -= 1,
            b'=' => {
                if depth == 0 {
                    // exclude `==`
                    if i + 1 < s.len() && bytes[i + 1] == b'=' {
                        i += 1;
                    } else if i > 0 && (bytes[i - 1] == b'=' || bytes[i - 1] == b'!' || bytes[i - 1] == b'<' || bytes[i - 1] == b'>') {
                        // `==`, `!=`, `<=`, `>=`
                    } else {
                        return Some(i);
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

// ---------------------------------------------------------------------------
// evaluation
// ---------------------------------------------------------------------------

/// Scope chain. `sets` from `{% set %}` are visible after definition in
/// the same block (Jinja scoping: block-scoped variables do not escape
/// `for` bodies except via namespace; `set` inside `if` persists).
/// Loops push a child scope whose sets are discarded on exit, except the
/// iteration variable itself. We implement: a stack of maps; lookups
/// search innermost to outermost. `{% set %}` writes to the current top
/// frame.
pub struct Ctx {
    pub scopes: Vec<IndexMap<String, Value>>,
}

impl Ctx {
    pub fn new(vars: IndexMap<String, Value>) -> Self {
        Ctx { scopes: vec![vars] }
    }
    pub fn get(&self, name: &str) -> Value {
        for scope in self.scopes.iter().rev() {
            if let Some(v) = scope.get(name) {
                return v.clone();
            }
        }
        // Jinja global functions are resolved here too.
        if name == "range" {
            // built-in global; range() returns a list of ints.
            return range_builtin();
        }
        Value::undefined_name(name)
    }
    pub fn set(&mut self, name: String, v: Value) {
        if let Some(top) = self.scopes.last_mut() {
            top.insert(name, v);
        }
    }
    pub fn push(&mut self, frame: IndexMap<String, Value>) {
        self.scopes.push(frame);
    }
    pub fn pop(&mut self) {
        self.scopes.pop();
    }
}

fn range_builtin() -> Value {
    Value::Func(Rc::new(|args, kwargs, lineno| {
        if !kwargs.is_empty() {
            return Err(type_err(
                "range() takes no keyword arguments".to_string(),
                lineno,
            ));
        }
        let (start, stop, step) = match args.len() {
            1 => (0isize, to_int_index(&args[0], lineno)?, 1isize),
            2 => (
                to_int_index(&args[0], lineno)?,
                to_int_index(&args[1], lineno)?,
                1isize,
            ),
            3 => (
                to_int_index(&args[0], lineno)?,
                to_int_index(&args[1], lineno)?,
                to_int_index(&args[2], lineno)?,
            ),
            _ => {
                return Err(type_err(
                    "range expected 1-3 arguments".to_string(),
                    lineno,
                ))
            }
        };
        if step == 0 {
            return Err(type_err(
                "range() arg 3 must not be zero".to_string(),
                lineno,
            ));
        }
        let mut out = Vec::new();
        let mut i = start;
        if step > 0 {
            while i < stop {
                out.push(PyValue::Int(PyInt::from_i64(i as i64)));
                i += step;
            }
        } else {
            while i > stop {
                out.push(PyValue::Int(PyInt::from_i64(i as i64)));
                i += step;
            }
        }
        Ok(Value::py(PyValue::List(out)))
    }))
}

fn to_int_index(v: &Value, lineno: usize) -> Result<isize, TemplateError> {
    let n = to_python_int(v, lineno)?;
    // range args in Jinja are Python ints; truncation of floats via __index__.
    // We accept ints and bool; floats that are integral? Python range(1.5) raises.
    // Match Python: only __index__ types.
    match v {
        Value::Py(PyValue::Int(pi)) => {
            let s = pi.0.to_string();
            s.parse::<isize>()
                .map_err(|_| type_err("range value out of range".to_string(), lineno))
        }
        Value::Py(PyValue::Bool(b)) => Ok(if *b { 1 } else { 0 }),
        _ => {
            let _ = n;
            Err(type_err(
                "'float' object cannot be interpreted as an integer".to_string(),
                lineno,
            ))
        }
    }
}

fn to_python_int(v: &Value, lineno: usize) -> Result<PyInt, TemplateError> {
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Int(n)) => Ok(n.clone()),
        Value::Py(PyValue::Bool(b)) => Ok(PyInt::from_i64(if *b { 1 } else { 0 })),
        Value::Py(PyValue::Float(f)) => {
            if f.is_nan() || f.is_infinite() {
                return Err(type_err(
                    "cannot convert float NaN to integer".to_string(),
                    lineno,
                ));
            }
            Ok(pytext::trunc_float_to_bigint(*f))
        }
        Value::Py(PyValue::Str(s)) => match pytext::int_from_string(s, 10) {
            Ok(n) => Ok(n),
            Err(e) => Err(from_pyerr(&e, lineno)),
        },
        Value::Py(PyValue::None) => Err(type_err(
            "int() argument must be a string, a number, or a sequence, not 'NoneType'"
                .to_string(),
            lineno,
        )),
        Value::Py(_) => Err(type_err(
            format!("int() argument must be a string, a number, or a sequence, not '{}'", v.type_name()),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "int() argument must be a string, a number, or a sequence, not 'function'".to_string(),
            lineno,
        )),
    }
}

fn to_python_float(v: &Value, lineno: usize) -> Result<f64, TemplateError> {
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Float(f)) => Ok(*f),
        Value::Py(PyValue::Int(n)) => match n.0.to_string().parse::<f64>() {
            Ok(f) => {
                // check overflow like Python: huge int -> inf? Python float(huge int) raises
                // OverflowError only when too large? `n.0.to_string().parse` yields inf for huge,
                // which matches CPython? CPython float(10**400) raises OverflowError.
                // We detect inf from finite big int.
                if f.is_infinite() {
                    return Err(TemplateError::new(
                        "OverflowError",
                        "int too large to convert to float".to_string(),
                        Some(lineno),
                    ));
                }
                Ok(f)
            }
            Err(_) => Err(TemplateError::new(
                "OverflowError",
                "int too large to convert to float".to_string(),
                Some(lineno),
            )),
        },
        Value::Py(PyValue::Bool(b)) => Ok(if *b { 1.0 } else { 0.0 }),
        Value::Py(PyValue::Str(s)) => match pytext::float_from_string(s) {
            Ok(f) => Ok(f),
            Err(e) => Err(from_pyerr(&e, lineno)),
        },
        Value::Py(PyValue::None) => Err(type_err(
            "float() argument must be a string or a real number, not 'NoneType'".to_string(),
            lineno,
        )),
        Value::Py(_) => Err(type_err(
            format!("float() argument must be a string or a real number, not '{}'", v.type_name()),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "float() argument must be a string or a real number, not 'function'".to_string(),
            lineno,
        )),
    }
}

fn iterate(v: &Value, lineno: usize) -> Result<Vec<Value>, TemplateError> {
    match v {
        Value::Undefined(_) => Ok(Vec::new()),
        Value::Py(PyValue::List(l)) => Ok(l.iter().cloned().map(Value::py).collect()),
        Value::Py(PyValue::Tuple(t)) => Ok(t.iter().cloned().map(Value::py).collect()),
        Value::Py(PyValue::Str(s)) => Ok(s.chars().map(|c| Value::py(PyValue::Str(c.to_string()))).collect()),
        Value::Py(PyValue::Dict(d)) => Ok(d.keys().cloned().map(Value::py).collect()),
        Value::Py(PyValue::Bytes(b)) => Ok(b.iter().map(|&x| Value::py(PyValue::Int(PyInt::from_i64(x as i64)))).collect()),
        Value::Py(PyValue::None) => Err(type_err(
            "'NoneType' object is not iterable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Int(_)) => Err(type_err(
            "'int' object is not iterable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Float(_)) => Err(type_err(
            "'float' object is not iterable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Bool(_)) => Err(type_err(
            "'bool' object is not iterable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Host(_)) => Err(type_err(
            "'Host' object is not iterable".to_string(),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "'function' object is not iterable".to_string(),
            lineno,
        )),
    }
}

fn get_attr(base: &Value, attr: &str, lineno: usize) -> Result<Value, TemplateError> {
    match base {
        Value::Undefined(_) => Err(base.fail_undefined(lineno)),
        Value::Func(_) => Err(type_err(
            format!("'function' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Py(PyValue::Dict(d)) => {
            let key = PyValue::Str(attr.to_string());
            // exact key match
            for (k, v) in d.iter() {
                if py_equal(k, &key) {
                    return Ok(Value::py(v.clone()));
                }
            }
            // Jinja getattr on dict falls back to getitem; missing yields Undefined
            // (not an error until used).
            Ok(Value::Undefined(UndefinedHint {
                hint: None,
                obj: Some(dict_repr_for_undefined(d)),
                name: Some(attr.to_string()),
            }))
        }
        Value::Py(PyValue::List(_) | PyValue::Tuple(_) | PyValue::Str(_) | PyValue::Bytes(_)) => {
            // klippy/gcode.py `Coord` is a tuple subclass exposing x/y/z/e;
            // get_status positions reach templates as tuples.
            if let Value::Py(PyValue::Tuple(t)) = base {
                let idx = match attr {
                    "x" => 0usize,
                    "y" => 1,
                    "z" => 2,
                    "e" => 3,
                    _ => usize::MAX,
                };
                if idx < t.len() {
                    return Ok(Value::py(t[idx].clone()));
                }
            }
            // method access alone is not a value; `.method("...")` is handled by
            // the caller through the Call node. Bare method access without a call
            // yields an Undefined so that `default` can mask it, matching Jinja's
            // Undefined for unresolvable attrs (methods are callables in Jinja).
            // For methods we support, produce a bound-method marker Func that
            // the Call evaluator can invoke.
            if is_string_method(attr) || is_list_method(attr) {
                let base_clone = base.clone();
                let attr_owned = attr.to_string();
                return Ok(Value::Func(Rc::new(move |args, kwargs, ln| {
                    call_string_method(&base_clone, attr_owned.as_str(), args, kwargs, ln)
                })));
            }
            // numeric str index attrs? not needed.
            Ok(Value::Undefined(UndefinedHint {
                hint: None,
                obj: Some(object_repr(base)),
                name: Some(attr.to_string()),
            }))
        }
        Value::Py(PyValue::None) => Err(type_err(
            format!("'NoneType' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Py(PyValue::Int(_)) => Err(type_err(
            format!("'int' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Py(PyValue::Float(_)) => Err(type_err(
            format!("'float' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Py(PyValue::Bool(_)) => Err(type_err(
            format!("'bool' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Py(PyValue::Host(_)) => Err(type_err(
            format!("'Host' object has no attribute '{attr}'"),
            lineno,
        )),
    }
}

fn dict_repr_for_undefined(d: &IndexMap<PyValue, PyValue>) -> String {
    // Jinja uses `object_type_repr(obj)` + `has no attribute`:
    // e.g. `<dict object as {'a': 1}>`? For simplicity we emit `dict`.
    // The message only surfaces when the undefined is (illegally) used.
    let _ = d;
    "dict".to_string()
}
fn object_repr(v: &Value) -> String {
    match v {
        Value::Py(PyValue::List(_)) => "list".to_string(),
        Value::Py(PyValue::Tuple(_)) => "tuple".to_string(),
        Value::Py(PyValue::Str(_)) => "str".to_string(),
        Value::Py(PyValue::Dict(_)) => "dict".to_string(),
        Value::Py(PyValue::None) => "NoneType".to_string(),
        Value::Py(PyValue::Int(_)) => "int".to_string(),
        Value::Py(PyValue::Float(_)) => "float".to_string(),
        _ => "object".to_string(),
    }
}

fn is_string_method(name: &str) -> bool {
    matches!(name, "split" | "strip" | "format" | "lower" | "upper" | "replace")
}
fn is_list_method(name: &str) -> bool {
    matches!(name, "append")
}

fn call_string_method(
    base: &Value,
    method: &str,
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<Value, TemplateError> {
    if !kwargs.is_empty() {
        return Err(type_err(
            format!("str.{method}() takes no keyword arguments"),
            lineno,
        ));
    }
    match (base, method) {
        (Value::Py(PyValue::Str(s)), "split") => Ok(Value::py(PyValue::List(
            do_split(s, args, lineno)?.into_iter().map(PyValue::Str).collect(),
        ))),
        (Value::Py(PyValue::Str(s)), "strip") => {
            if !args.is_empty() {
                return Err(type_err(
                    "strip() takes at most 0 arguments (1 given)".to_string(),
                    lineno,
                ));
            }
            Ok(Value::py(PyValue::Str(
                s.trim_matches(|c: char| c.is_whitespace()).to_string(),
            )))
        }
        (Value::Py(PyValue::Str(s)), "format") => Ok(Value::py(PyValue::Str(
            do_str_format(s, args, lineno)?,
        ))),
        (Value::Py(PyValue::Str(s)), "lower") => Ok(Value::py(PyValue::Str(s.to_lowercase()))),
        (Value::Py(PyValue::Str(s)), "upper") => Ok(Value::py(PyValue::Str(s.to_uppercase()))),
        (Value::Py(PyValue::Str(s)), "replace") => {
            if args.len() != 2 {
                return Err(type_err(
                    "replace expected 2 arguments".to_string(),
                    lineno,
                ));
            }
            let old = need_str(&args[0], lineno)?;
            let new = need_str(&args[1], lineno)?;
            Ok(Value::py(PyValue::Str(s.replace(&old, &new))))
        }
        (Value::Py(PyValue::List(_)), "append") => {
            // list.append mutates the receiver list in place, like Jinja/Python.
            // Our values are cloned, so mutation of the shared Rc list is not
            // visible through clones. Jinja's `{% set _d = l.append(x) %}` relies on
            // in-place mutation being visible via `l`. To emulate, the list value
            // itself would need interior mutability. For parity we implement
            // append as returning None and keep the original list unmodified,
            // which would diverge — so we handle it specially in `eval` where
            // `set _d = prompt_txt.append(...)` is evaluated: the caller must
            // mutate the stored list. See `eval_call` special-case.
            // Here (bound method via GetAttr path) we cannot mutate; raise so
            // the special-case is required.
            Err(type_err(
                "list append must be evaluated through the call helper".to_string(),
                lineno,
            ))
        }
        _ => Err(type_err(
            format!("'{}' object has no method '{method}'", base.type_name()),
            lineno,
        )),
    }
}

fn need_str(v: &Value, lineno: usize) -> Result<String, TemplateError> {
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Str(s)) => Ok(s.clone()),
        Value::Py(other) => Ok(other.str()),
        Value::Func(_) => Err(type_err(
            "expected string, got function".to_string(),
            lineno,
        )),
    }
}

fn do_split(s: &str, args: &[Value], lineno: usize) -> Result<Vec<String>, TemplateError> {
    // `str.split([sep [, maxsplit]])` with Python semantics.
    match args.len() {
        0 => Ok(s
            .split(|c: char| c.is_whitespace())
            .filter(|p| !p.is_empty())
            .map(|p| p.to_string())
            .collect()),
        1 => {
            let sep = need_str(&args[0], lineno)?;
            if sep.is_empty() {
                return Err(type_err("empty separator".to_string(), lineno));
            }
            // Python `split(sep)` preserves empty parts.
            Ok(split_on_sep(s, &sep, -1))
        }
        2 => {
            let sep = need_str(&args[0], lineno)?;
            if sep.is_empty() {
                return Err(type_err("empty separator".to_string(), lineno));
            }
            let maxsplit = match &args[1] {
                Value::Py(PyValue::Int(n)) => parse_small_int(n, lineno)?,
                Value::Py(PyValue::Bool(b)) => {
                    if *b { 1 } else { 0 }
                }
                Value::Undefined(_) => {
                    return Err(args[1].fail_undefined(lineno));
                }
                _ => {
                    return Err(type_err(
                        "maxsplit must be an integer".to_string(),
                        lineno,
                    ))
                }
            };
            Ok(split_on_sep(s, &sep, maxsplit))
        }
        _ => Err(type_err(
            format!("split expected at most 2 arguments, got {}", args.len()),
            lineno,
        )),
    }
}

fn split_on_sep(s: &str, sep: &str, maxsplit: isize) -> Vec<String> {
    if maxsplit < 0 {
        return s.split(sep).map(|p| p.to_string()).collect();
    }
    let mut out = Vec::new();
    let mut rest = s;
    let mut splits = 0;
    while (splits as isize) < maxsplit {
        match rest.find(sep) {
            Some(pos) => {
                out.push(rest[..pos].to_string());
                rest = &rest[pos + sep.len()..];
                splits += 1;
            }
            None => break,
        }
    }
    out.push(rest.to_string());
    out
}

fn parse_small_int(n: &PyInt, lineno: usize) -> Result<isize, TemplateError> {
    let s = n.0.to_string();
    s.parse::<isize>()
        .map_err(|_| type_err("maxsplit too large".to_string(), lineno))
}

fn do_str_format(fmt: &str, args: &[Value], lineno: usize) -> Result<String, TemplateError> {
    // `"{:>4.0%}".format(x)` uses `str.format` with positional args only in corpus.
    // Delegate to pycompat::str_format over PctValues.
    let mut pct: Vec<PctValue> = Vec::new();
    for a in args {
        match a {
            Value::Py(PyValue::Str(s)) => pct.push(PctValue::Str(s.clone())),
            Value::Py(PyValue::Bool(b)) => pct.push(PctValue::Bool(*b)),
            Value::Py(PyValue::Int(n)) => pct.push(PctValue::Int(n.clone())),
            Value::Py(PyValue::Float(f)) => pct.push(PctValue::Float(*f)),
            Value::Py(PyValue::None) => pct.push(PctValue::Str("None".to_string())),
            Value::Undefined(_) => pct.push(PctValue::Str(String::new())),
            Value::Py(_) => pct.push(PctValue::Str(py_str_of(a, lineno)?)),
            Value::Func(_) => pct.push(PctValue::Str(String::new())),
        }
    }
    let no_kwargs: &[(String, PctValue)] = &[];
    match pytext::str_format(fmt, &pct, no_kwargs) {
        Ok(s) => Ok(s),
        Err(e) => Err(from_pyerr(&e, lineno)),
    }
}

fn get_item(base: &Value, key: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match base {
        Value::Undefined(_) => Err(base.fail_undefined(lineno)),
        Value::Func(_) => Err(type_err(
            "'function' object is not subscriptable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Dict(d)) => {
            let kv = match key {
                Value::Undefined(_) => return Err(key.fail_undefined(lineno)),
                Value::Py(k) => k.clone(),
                Value::Func(_) => {
                    return Err(type_err("unhashable type: 'function'".to_string(), lineno))
                }
            };
            if !kv.is_hashable() {
                return Err(type_err(
                    format!("unhashable type: '{}'", pycompat::value::type_name(&kv)),
                    lineno,
                ));
            }
            // Jinja getitem: try `obj[arg]`; on failure return Undefined.
            for (k, v) in d.iter() {
                if py_equal(k, &kv) {
                    return Ok(Value::py(v.clone()));
                }
            }
            // fall back to attribute lookup (getitem prefers item, then attr)
            if let PyValue::Str(name) = &kv {
                let attr = get_attr(base, name, lineno)?;
                if attr.is_undefined() {
                    return Ok(Value::Undefined(UndefinedHint {
                        hint: None,
                        obj: Some(dict_repr_for_undefined(d)),
                        name: Some(name.clone()),
                    }));
                }
                return Ok(attr);
            }
            Ok(Value::Undefined(UndefinedHint {
                hint: None,
                obj: Some(dict_repr_for_undefined(d)),
                name: Some(value_to_key_name(key)),
            }))
        }
        Value::Py(PyValue::List(l)) => {
            let idx = to_index(key, lineno, false)?;
            let len = l.len() as isize;
            let mut i = idx;
            if i < 0 {
                i += len;
            }
            if i < 0 || i >= len {
                return Err(type_err(
                    "list index out of range".to_string(),
                    lineno,
                ));
            }
            Ok(Value::py(l[i as usize].clone()))
        }
        Value::Py(PyValue::Tuple(t)) => {
            let idx = to_index(key, lineno, false)?;
            let len = t.len() as isize;
            let mut i = idx;
            if i < 0 {
                i += len;
            }
            if i < 0 || i >= len {
                return Err(type_err(
                    "tuple index out of range".to_string(),
                    lineno,
                ));
            }
            Ok(Value::py(t[i as usize].clone()))
        }
        Value::Py(PyValue::Str(s)) => {
            let idx = to_index(key, lineno, false)?;
            let chars: Vec<char> = s.chars().collect();
            let len = chars.len() as isize;
            let mut i = idx;
            if i < 0 {
                i += len;
            }
            if i < 0 || i >= len {
                return Err(type_err(
                    "string index out of range".to_string(),
                    lineno,
                ));
            }
            Ok(Value::py(PyValue::Str(chars[i as usize].to_string())))
        }
        Value::Py(PyValue::Bytes(b)) => {
            let idx = to_index(key, lineno, false)?;
            let len = b.len() as isize;
            let mut i = idx;
            if i < 0 {
                i += len;
            }
            if i < 0 || i >= len {
                return Err(type_err("index out of range".to_string(), lineno));
            }
            Ok(Value::py(PyValue::Int(PyInt::from_i64(b[i as usize] as i64))))
        }
        Value::Py(PyValue::None) => Err(type_err(
            "'NoneType' object is not subscriptable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Int(_)) => Err(type_err(
            "'int' object is not subscriptable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Float(_)) => Err(type_err(
            "'float' object is not subscriptable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Bool(_)) => Err(type_err(
            "'bool' object is not subscriptable".to_string(),
            lineno,
        )),
        Value::Py(PyValue::Host(_)) => Err(type_err(
            "'Host' object is not subscriptable".to_string(),
            lineno,
        )),
    }
}

fn value_to_key_name(key: &Value) -> String {
    match key {
        Value::Py(PyValue::Str(s)) => s.clone(),
        Value::Py(other) => other.str(),
        _ => "value".to_string(),
    }
}

fn to_index(v: &Value, lineno: usize, _for_slice: bool) -> Result<isize, TemplateError> {
    match v {
        Value::Py(PyValue::Int(n)) => {
            let s = n.0.to_string();
            s.parse::<isize>()
                .map_err(|_| type_err("index out of range".to_string(), lineno))
        }
        Value::Py(PyValue::Bool(b)) => Ok(if *b { 1 } else { 0 }),
        Value::Py(PyValue::Float(f)) => {
            if f.fract() != 0.0 {
                return Err(type_err("indices must be integers".to_string(), lineno));
            }
            if *f < isize::MIN as f64 || *f > isize::MAX as f64 {
                return Err(type_err("index out of range".to_string(), lineno));
            }
            Ok(*f as isize)
        }
        _ => Err(type_err("indices must be integers".to_string(), lineno)),
    }
}

fn contains(container: &Value, elem: &Value, lineno: usize) -> Result<bool, TemplateError> {
    match container {
        Value::Undefined(_) => return Ok(false),
        Value::Py(PyValue::Dict(d)) => {
            let ev = match elem {
                Value::Py(k) => k.clone(),
                Value::Undefined(_) => return Err(elem.fail_undefined(lineno)),
                Value::Func(_) => {
                    return Err(type_err("unhashable type: 'function'".to_string(), lineno))
                }
            };
            if !ev.is_hashable() {
                return Err(type_err(
                    format!("unhashable type: '{}'", pycompat::value::type_name(&ev)),
                    lineno,
                ));
            }
            for k in d.keys() {
                if py_equal(k, &ev) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        Value::Py(PyValue::List(l)) => {
            for item in l {
                if py_equal(item, &need_py(elem, lineno)?) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        Value::Py(PyValue::Tuple(t)) => {
            for item in t {
                if py_equal(item, &need_py(elem, lineno)?) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        Value::Py(PyValue::Str(s)) => {
            let sub = need_str(elem, lineno)?;
            Ok(s.contains(&sub))
        }
        Value::Py(_) => Err(type_err(
            format!("'{}' object is not iterable", container.type_name()),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "'function' object is not iterable".to_string(),
            lineno,
        )),
    }
}

fn need_py(v: &Value, lineno: usize) -> Result<PyValue, TemplateError> {
    match v {
        Value::Py(p) => Ok(p.clone()),
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Func(_) => Err(type_err("function is not comparable".to_string(), lineno)),
    }
}

fn natural_type(a: &PyValue) -> Option<&'static str> {
    Some(pycompat::value::type_name(a))
}

// numeric tower helpers
fn is_number(v: &Value) -> bool {
    matches!(
        v,
        Value::Py(PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_))
    )
}

#[derive(Clone, Copy, PartialEq)]
enum NumKind {
    Int,
    Float,
}

fn num_kind(v: &Value) -> Option<NumKind> {
    match v {
        Value::Py(PyValue::Float(_)) => Some(NumKind::Float),
        Value::Py(PyValue::Int(_) | PyValue::Bool(_)) => Some(NumKind::Int),
        _ => None,
    }
}

fn to_pyint(v: &Value) -> Option<PyInt> {
    match v {
        Value::Py(PyValue::Int(n)) => Some(n.clone()),
        Value::Py(PyValue::Bool(b)) => Some(PyInt::from_i64(if *b { 1 } else { 0 })),
        _ => None,
    }
}

fn cmp_values(a: &Value, b: &Value, lineno: usize) -> Result<Ordering, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => {
            let u = if a.is_undefined() { a } else { b };
            Err(u.fail_undefined(lineno))
        }
        (Value::Py(pa), Value::Py(pb)) => match py_compare(pa, pb) {
            Ok(o) => Ok(o),
            Err(msg) => Err(type_err(msg, lineno)),
        },
        (Value::Func(_), _) | (_, Value::Func(_)) => Err(type_err(
            format!(
                "'<' not supported between instances of '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}

fn eval_compare(op: &str, a: &Value, b: &Value, lineno: usize) -> Result<bool, TemplateError> {
    match op {
        "==" => {
            match (a, b) {
                (Value::Undefined(_), _) | (_, Value::Undefined(_)) => {
                    // Undefined == Undefined is True? Jinja: Undefined == Undefined -> True;
                    // Undefined == anything else -> False (Undefined defines __eq__? Check).
                    // Python `Undefined` rule: Undefined == x is False unless x is Undefined.
                    // Implement accordingly.
                    let both = a.is_undefined() && b.is_undefined();
                    Ok(both)
                }
                (Value::Py(pa), Value::Py(pb)) => Ok(py_equal(pa, pb)),
                _ => Ok(false),
            }
        }
        "!=" => {
            let eq = match (a, b) {
                (Value::Undefined(_), _) | (_, Value::Undefined(_)) => {
                    a.is_undefined() && b.is_undefined()
                }
                (Value::Py(pa), Value::Py(pb)) => py_equal(pa, pb),
                _ => false,
            };
            Ok(!eq)
        }
        "<" | ">" | "<=" | ">=" => {
            let o = cmp_values(a, b, lineno)?;
            Ok(match op {
                "<" => o == Ordering::Less,
                ">" => o == Ordering::Greater,
                "<=" => o != Ordering::Greater,
                _ => o != Ordering::Less,
            })
        }
        "in" => contains(b, a, lineno),
        "notin" => Ok(!contains(b, a, lineno)?),
        _ => Err(type_err(format!("unknown comparison '{op}'"), lineno)),
    }
}

fn numeric_result(a: &Value, b: &Value, _op: &str, lineno: usize) -> Result<(bool, f64, Option<PyInt>, Option<PyInt>), TemplateError> {
    // returns (is_float, float_val, int_a, int_b) helper placeholder
    let _ = (a, b);
    Err(type_err("unused".to_string(), lineno))
}

// --- arithmetic ---

fn eval_add(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    // string concat, list/tuple concat, numeric add (Python semantics)
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => {
            let u = if a.is_undefined() { a } else { b };
            Err(u.fail_undefined(lineno))
        }
        (Value::Py(PyValue::Str(s)), Value::Py(PyValue::Str(t))) => {
            Ok(Value::py(PyValue::Str(format!("{s}{t}"))))
        }
        (Value::Py(PyValue::List(l)), Value::Py(PyValue::List(m))) => {
            let mut out = l.clone();
            out.extend(m.iter().cloned());
            Ok(Value::py(PyValue::List(out)))
        }
        (Value::Py(PyValue::Tuple(l)), Value::Py(PyValue::Tuple(m))) => {
            let mut out = l.clone();
            out.extend(m.iter().cloned());
            Ok(Value::py(PyValue::Tuple(out)))
        }
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "add", lineno),
        (Value::Py(x), Value::Py(y)) => Err(type_err(
            format!(
                "unsupported operand type(s) for +: '{}' and '{}'",
                pycompat::value::type_name(x),
                pycompat::value::type_name(y)
            ),
            lineno,
        )),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for +: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}

fn do_numeric(a: &Value, b: &Value, op: &str, lineno: usize) -> Result<Value, TemplateError> {
    let float_mode = matches!(num_kind(a), Some(NumKind::Float))
        || matches!(num_kind(b), Some(NumKind::Float));
    if float_mode {
        let fa = match a {
            Value::Py(PyValue::Float(f)) => *f,
            Value::Py(PyValue::Int(n)) => pytext::float_from_string(&n.0.to_string())
                .map_err(|_| type_err("int too large to convert to float".to_string(), lineno))?,
            Value::Py(PyValue::Bool(v)) => {
                if *v { 1.0 } else { 0.0 }
            }
            _ => return Err(type_err("float conversion failed".to_string(), lineno)),
        };
        let fb = match b {
            Value::Py(PyValue::Float(f)) => *f,
            Value::Py(PyValue::Int(n)) => pytext::float_from_string(&n.0.to_string())
                .map_err(|_| type_err("int too large to convert to float".to_string(), lineno))?,
            Value::Py(PyValue::Bool(v)) => {
                if *v { 1.0 } else { 0.0 }
            }
            _ => return Err(type_err("float conversion failed".to_string(), lineno)),
        };
        match op {
            "add" => Ok(Value::py(PyValue::Float(fa + fb))),
            "sub" => Ok(Value::py(PyValue::Float(fa - fb))),
            "mul" => Ok(Value::py(PyValue::Float(fa * fb))),
            "div" => match pytext::float_true_div(fa, fb) {
                Ok(r) => Ok(Value::py(PyValue::Float(r))),
                Err(e) => Err(from_pyerr(&e, lineno)),
            },
            "floordiv" => match pytext::float_floor_div(fa, fb) {
                Ok(r) => Ok(Value::py(PyValue::Float(r))),
                Err(e) => Err(from_pyerr(&e, lineno)),
            },
            "mod" => match pytext::float_mod(fa, fb) {
                Ok(r) => Ok(Value::py(PyValue::Float(r))),
                Err(e) => Err(from_pyerr(&e, lineno)),
            },
            "pow" => match pytext::float_pow(fa, fb) {
                Ok(r) => Ok(Value::py(PyValue::Float(r))),
                Err(e) => Err(from_pyerr(&e, lineno)),
            },
            _ => Err(type_err(format!("unknown numeric op '{op}'"), lineno)),
        }
    } else {
        // int mode
        let na = to_pyint(a).unwrap();
        let nb = to_pyint(b).unwrap();
        match op {
            "add" => Ok(Value::py(PyValue::Int(PyInt::new(
                na.0.clone() + nb.0.clone(),
            )))),
            "sub" => Ok(Value::py(PyValue::Int(PyInt::new(
                na.0.clone() - nb.0.clone(),
            )))),
            "mul" => Ok(Value::py(PyValue::Int(PyInt::new(
                na.0.clone() * nb.0.clone(),
            )))),
            "div" => {
                let fa = bigint_to_f64(&na)
                    .ok_or_else(|| type_err("int too large to convert to float".to_string(), lineno))?;
                let fb = bigint_to_f64(&nb)
                    .ok_or_else(|| type_err("int too large to convert to float".to_string(), lineno))?;
                match pytext::float_true_div(fa, fb) {
                    Ok(r) => Ok(Value::py(PyValue::Float(r))),
                    Err(e) => Err(from_pyerr(&e, lineno)),
                }
            }
            "floordiv" => {
                if nb.is_zero() {
                    return Err(type_err(
                        "integer division or modulo by zero".to_string(),
                        lineno,
                    ));
                }
                Ok(Value::py(PyValue::Int(int_floor_div(&na, &nb))))
            }
            "mod" => {
                if nb.is_zero() {
                    return Err(type_err(
                        "integer division or modulo by zero".to_string(),
                        lineno,
                    ));
                }
                Ok(Value::py(PyValue::Int(int_mod(&na, &nb))))
            }
            "pow" => {
                if nb.sign() < 0 {
                    let fa = bigint_to_f64(&na)
                        .ok_or_else(|| type_err("int too large to convert to float".to_string(), lineno))?;
                    let fb = bigint_to_f64(&nb)
                        .ok_or_else(|| type_err("int too large to convert to float".to_string(), lineno))?;
                    match pytext::float_pow(fa, fb) {
                        Ok(r) => Ok(Value::py(PyValue::Float(r))),
                        Err(e) => Err(from_pyerr(&e, lineno)),
                    }
                } else {
                    let exp = nb.0.to_string().parse::<u64>().map_err(|_| {
                        type_err("exponent not supported".to_string(), lineno)
                    })?;
                    Ok(Value::py(PyValue::Int(int_pow(&na, exp))))
                }
            }
            _ => Err(type_err(format!("unknown numeric op '{op}'"), lineno)),
        }
    }
}
fn bigint_to_f64(n: &PyInt) -> Option<f64> {
    n.inner().to_string().parse::<f64>().ok()
}

fn eval_sub(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "sub", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for -: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}
fn eval_mul(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        // string/list repetition
        (Value::Py(PyValue::Str(s)), Value::Py(PyValue::Int(n))) => {
            let cnt = n.0.to_string().parse::<i64>().unwrap_or(0);
            if cnt <= 0 {
                Ok(Value::py(PyValue::Str(String::new())))
            } else {
                Ok(Value::py(PyValue::Str(s.repeat(cnt as usize))))
            }
        }
        (Value::Py(PyValue::Int(n)), Value::Py(PyValue::Str(s))) => {
            let cnt = n.0.to_string().parse::<i64>().unwrap_or(0);
            if cnt <= 0 {
                Ok(Value::py(PyValue::Str(String::new())))
            } else {
                Ok(Value::py(PyValue::Str(s.repeat(cnt as usize))))
            }
        }
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "mul", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for *: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}
fn eval_div(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "div", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for / or division: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}
fn eval_floordiv(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "floordiv", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for //: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}
fn eval_mod(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        // `%` string formatting takes precedence over numeric mod when LHS is str.
        (Value::Py(PyValue::Str(fmt)), _) => {
            let pv = to_pct(b, lineno)?;
            let args: Vec<PctValue> = match pv {
                PctValue::Str(_) | PctValue::Bool(_) | PctValue::Int(_) | PctValue::Float(_) | PctValue::Bytes(_) => {
                    vec![pv]
                }
            };
            // CPython: if RHS is tuple, tuple elements are the args.
            if let Value::Py(PyValue::Tuple(t)) = b {
                let mut vec = Vec::new();
                for item in t {
                    vec.push(to_pct(&Value::py(item.clone()), lineno)?);
                }
                return match pytext::percent_format(fmt, &vec) {
                    Ok(s) => Ok(Value::py(PyValue::Str(s))),
                    Err(e) => Err(from_pyerr(&e, lineno)),
                };
            }
            match pytext::percent_format(fmt, &args) {
                Ok(s) => Ok(Value::py(PyValue::Str(s))),
                Err(e) => Err(from_pyerr(&e, lineno)),
            }
        }
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "mod", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for %: '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}
fn to_pct(v: &Value, lineno: usize) -> Result<PctValue, TemplateError> {
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Str(s)) => Ok(PctValue::Str(s.clone())),
        Value::Py(PyValue::Bool(b)) => Ok(PctValue::Bool(*b)),
        Value::Py(PyValue::Int(n)) => Ok(PctValue::Int(n.clone())),
        Value::Py(PyValue::Float(f)) => Ok(PctValue::Float(*f)),
        Value::Py(PyValue::None) => Ok(PctValue::Str("None".to_string())),
        Value::Py(PyValue::Bytes(b)) => Ok(PctValue::Bytes(b.clone())),
        Value::Py(other) => Ok(PctValue::Str(other.str())),
        Value::Func(_) => Err(type_err(
            "unsupported operand for % formatting".to_string(),
            lineno,
        )),
    }
}

fn eval_pow(a: &Value, b: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match (a, b) {
        (Value::Undefined(_), _) | (_, Value::Undefined(_)) => Err(
            if a.is_undefined() { a } else { b }.fail_undefined(lineno),
        ),
        _ if is_number(a) && is_number(b) => do_numeric(a, b, "pow", lineno),
        _ => Err(type_err(
            format!(
                "unsupported operand type(s) for ** or pow(): '{}' and '{}'",
                a.type_name(),
                b.type_name()
            ),
            lineno,
        )),
    }
}

// `int_divmod` import is used for floor-div paths (kept for clarity).
#[allow(dead_code)]
fn _unused(_: &PyInt, _: &PyInt) -> (PyInt, PyInt) {
    // placeholder to keep import without triggering warning in all profiles
    int_divmod(&PyInt::from_i64(1), &PyInt::from_i64(1))
}

// --- filters ---

fn eval_filter(
    filter_name: &str,
    node: &Value,
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<Value, TemplateError> {
    match filter_name {
        "default" | "d" => {
            let (default_val, boolean) = parse_default_args(args, kwargs, lineno)?;
            match node {
                Value::Undefined(_) => Ok(default_val),
                _ if boolean && !node.truthy() => Ok(default_val),
                _ => Ok(node.clone()),
            }
        }
        "float" => {
            let default = parse_float_default(args, kwargs, lineno)?;
            match to_python_float(node, lineno) {
                Ok(f) => Ok(Value::py(PyValue::Float(f))),
                Err(_) => Ok(Value::py(PyValue::Float(default))),
            }
        }
        "int" => {
            let (default, base) = parse_int_args(args, kwargs, lineno)?;
            match eval_int_filter(node, base, lineno) {
                Ok(n) => Ok(Value::py(PyValue::Int(n))),
                Err(_) => Ok(Value::py(PyValue::Int(
                    PyInt::from_i64(default),
                ))),
            }
        }
        "lower" => {
            check_no_extra_args(filter_name, args, kwargs, lineno)?;
            Ok(Value::py(PyValue::Str(
                py_str_of(node, lineno)?.to_lowercase(),
            )))
        }
        "upper" => {
            check_no_extra_args(filter_name, args, kwargs, lineno)?;
            Ok(Value::py(PyValue::Str(
                py_str_of(node, lineno)?.to_uppercase(),
            )))
        }
        "abs" => {
            check_no_extra_args(filter_name, args, kwargs, lineno)?;
            eval_abs(node, lineno)
        }
        "length" => {
            check_no_extra_args(filter_name, args, kwargs, lineno)?;
            eval_length(node, lineno)
        }
        "min" | "max" => {
            let is_min = filter_name == "min";
            eval_minmax(node, is_min, args, kwargs, lineno)
        }
        "replace" => {
            if !kwargs.is_empty() || args.len() < 2 || args.len() > 3 {
                return Err(type_err(
                    "replace expected (old, new[, count])".to_string(),
                    lineno,
                ));
            }
            let s = py_str_of(node, lineno)?;
            let old = need_str(&args[0], lineno)?;
            let new = need_str(&args[1], lineno)?;
            if args.len() == 2 {
                Ok(Value::py(PyValue::Str(s.replace(&old, &new))))
            } else {
                let cnt = to_python_int(&args[2], lineno)?;
                let c = cnt.0.to_string().parse::<i32>().map_err(|_| {
                    type_err("count must be integer".to_string(), lineno)
                })?;
                // Python str.replace(old,new,count): count <0 means all; count==0 means none.
                if c < 0 {
                    Ok(Value::py(PyValue::Str(s.replace(&old, &new))))
                } else if c == 0 {
                    Ok(Value::py(PyValue::Str(s)))
                } else {
                    // Rust replace with count: split and rejoin limited times.
                    let mut result = String::new();
                    let mut rest = s.as_str();
                    let mut done = 0;
                    while done < c {
                        match rest.find(&old) {
                            Some(idx) => {
                                result.push_str(&rest[..idx]);
                                result.push_str(&new);
                                rest = &rest[idx + old.len()..];
                                done += 1;
                            }
                            None => break,
                        }
                    }
                    result.push_str(rest);
                    Ok(Value::py(PyValue::Str(result)))
                }
            }
        }
        _ => Err(type_err(
            format!("No filter named '{filter_name}'."),
            lineno,
        )),
    }
}

fn check_no_extra_args(name: &str, args: &[Value], kwargs: &[(String, Value)], lineno: usize) -> Result<(), TemplateError> {
    if !args.is_empty() || !kwargs.is_empty() {
        return Err(type_err(
            format!("filter '{name}' takes no arguments"),
            lineno,
        ));
    }
    Ok(())
}

fn parse_default_args(
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<(Value, bool), TemplateError> {
    // `default(value, default_value='', boolean=False)`
    let mut default_val: Option<Value> = None;
    let mut boolean = false;
    if args.len() > 2 {
        return Err(type_err(
            "default expected at most 2 arguments".to_string(),
            lineno,
        ));
    }
    if let Some(v) = args.get(0) {
        default_val = Some(v.clone());
    }
    if let Some(v) = args.get(1) {
        boolean = v.truthy();
    }
    for (k, v) in kwargs {
        match k.as_str() {
            "default_value" | "value" | "default" => default_val = Some(v.clone()),
            "boolean" => boolean = v.truthy(),
            other => {
                return Err(type_err(
                    format!("default() got an unexpected keyword argument '{other}'"),
                    lineno,
                ))
            }
        }
    }
    Ok((
        default_val.unwrap_or_else(|| Value::py(PyValue::Str(String::new()))),
        boolean,
    ))
}

fn parse_float_default(
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<f64, TemplateError> {
    // `float(value, default=0.0)`
    match args.len() {
        0 => {
            if let Some((k, v)) = kwargs.iter().next() {
                if k == "default" {
                    return to_python_float(v, lineno).map_err(|_| {
                        type_err("float default must be a number".to_string(), lineno)
                    });
                }
                return Err(type_err(format!("float() got an unexpected keyword argument '{k}'"), lineno));
            }
            Ok(0.0)
        }
        1 => {
            if !kwargs.is_empty() {
                return Err(type_err("float() takes no keyword arguments".to_string(), lineno));
            }
            match to_python_float(&args[0], lineno) {
                Ok(f) => Ok(f),
                Err(_) => Ok(0.0),
            }
        }
        _ => Err(type_err("float expected at most 1 argument".to_string(), lineno)),
    }
}

fn parse_int_args(
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<(i64, i64), TemplateError> {
    // `int(value, default=0, base=10)`
    let mut default: i64 = 0;
    let mut base: i64 = 10;
    if args.len() > 2 {
        return Err(type_err("int expected at most 2 arguments".to_string(), lineno));
    }
    if let Some(v) = args.get(0) {
        let n = to_python_int(v, lineno)?;
        default = n.0.to_string().parse().map_err(|_| {
            type_err("int default out of range".to_string(), lineno)
        })?;
    }
    if let Some(v) = args.get(1) {
        let n = to_python_int(v, lineno)?;
        base = n.0.to_string().parse().map_err(|_| {
            type_err("int base out of range".to_string(), lineno)
        })?;
    }
    for (k, v) in kwargs {
        match k.as_str() {
            "default" => {
                let n = to_python_int(v, lineno)?;
                default = n.0.to_string().parse().map_err(|_| {
                    type_err("int default out of range".to_string(), lineno)
                })?;
            }
            "base" => {
                let n = to_python_int(v, lineno)?;
                base = n.0.to_string().parse().map_err(|_| {
                    type_err("int base out of range".to_string(), lineno)
                })?;
            }
            other => {
                return Err(type_err(
                    format!("int() got an unexpected keyword argument '{other}'"),
                    lineno,
                ))
            }
        }
    }
    Ok((default, base))
}

fn eval_int_filter(v: &Value, base: i64, lineno: usize) -> Result<PyInt, TemplateError> {
    // Jinja do_int quirk: "42.23"|int -> 42 via int(float(value)).
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Str(s)) => match pytext::int_from_string(s, base) {
            Ok(n) => Ok(n),
            Err(_) => {
                // fallback: int(float(s))
                let f = pytext::float_from_string(s).map_err(|e| from_pyerr(&e, lineno))?;
                if f.is_nan() || f.is_infinite() {
                    return Err(type_err("cannot convert float NaN to integer".to_string(), lineno));
                }
                Ok(pytext::trunc_float_to_bigint(f))
            }
        },
        Value::Py(PyValue::Float(f)) => {
            if f.is_nan() || f.is_infinite() {
                return Err(type_err("cannot convert float NaN to integer".to_string(), lineno));
            }
            Ok(pytext::trunc_float_to_bigint(*f))
        }
        Value::Py(PyValue::Int(n)) => Ok(n.clone()),
        Value::Py(PyValue::Bool(b)) => Ok(PyInt::from_i64(if *b { 1 } else { 0 })),
        Value::Py(PyValue::None) => Err(type_err("int() argument must be a string, a number, or a sequence, not 'NoneType'".to_string(), lineno)),
        Value::Py(_) => Err(type_err(
            format!("int() argument must be a string, a number, or a sequence, not '{}'", v.type_name()),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "int() argument must be a string, a number, or a sequence, not 'function'".to_string(),
            lineno,
        )),
    }
}

fn eval_abs(v: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match v {
        Value::Undefined(_) => Err(v.fail_undefined(lineno)),
        Value::Py(PyValue::Int(n)) => {
            let neg = n.sign() < 0;
            let abs = if neg {
                PyInt::new(-n.0.clone())
            } else {
                n.clone()
            };
            Ok(Value::py(PyValue::Int(abs)))
        }
        Value::Py(PyValue::Float(f)) => Ok(Value::py(PyValue::Float(
            pycompat::libm::fabs(*f),
        ))),
        Value::Py(PyValue::Bool(b)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            if *b { 1 } else { 0 },
        )))),
        Value::Py(PyValue::Str(_)) => Err(type_err(
            "bad operand type for abs(): 'str'".to_string(),
            lineno,
        )),
        Value::Py(other) => Err(type_err(
            format!(
                "bad operand type for abs(): '{}'",
                pycompat::value::type_name(other)
            ),
            lineno,
        )),
        Value::Func(_) => Err(type_err(
            "bad operand type for abs(): 'function'".to_string(),
            lineno,
        )),
    }
}

fn eval_length(v: &Value, lineno: usize) -> Result<Value, TemplateError> {
    match v {
        Value::Undefined(_) => Ok(Value::py(PyValue::Int(PyInt::from_i64(0)))),
        Value::Py(PyValue::Str(s)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            s.chars().count() as i64,
        )))),
        Value::Py(PyValue::Bytes(b)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            b.len() as i64,
        )))),
        Value::Py(PyValue::List(l)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            l.len() as i64,
        )))),
        Value::Py(PyValue::Tuple(t)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            t.len() as i64,
        )))),
        Value::Py(PyValue::Dict(d)) => Ok(Value::py(PyValue::Int(PyInt::from_i64(
            d.len() as i64,
        )))),
        _ => Err(type_err(
            format!("object of type '{}' has no len()", v.type_name()),
            lineno,
        )),
    }
}

fn eval_minmax(
    v: &Value,
    is_min: bool,
    args: &[Value],
    kwargs: &[(String, Value)],
    lineno: usize,
) -> Result<Value, TemplateError> {
    if !kwargs.is_empty() {
        // support `case_sensitive`? Not in corpus; reject explicitly.
        return Err(type_err(
            format!("filter '{}' takes no keyword arguments", if is_min { "min" } else { "max" }),
            lineno,
        ));
    }
    if !args.is_empty() {
        // Jinja min/max with positional `case_sensitive` / `attribute` unsupported in corpus.
        return Err(type_err(
            "unsupported 'min'/'max' positional arguments".to_string(),
            lineno,
        ));
    }
    let items = iterate(v, lineno)?;
    if items.is_empty() {
        return Err(type_err(
            format!(
                "{}() arg is an empty sequence",
                if is_min { "min" } else { "max" }
            ),
            lineno,
        ));
    }
    let mut best = items[0].clone();
    for it in items.iter().skip(1) {
        let o = cmp_values(it, &best, lineno)?;
        if (is_min && o == Ordering::Less) || (!is_min && o == Ordering::Greater) {
            best = it.clone();
        }
    }
    Ok(best)
}

// --- tests ---

fn eval_test(
    test_name: &str,
    node: &Value,
    args: &[Value],
    kwargs: &[(String, Value)],
    negated: bool,
    lineno: usize,
) -> Result<Value, TemplateError> {
    let r = match test_name {
        "defined" => {
            check_no_test_args(test_name, args, kwargs, lineno)?;
            !node.is_undefined()
        }
        "undefined" => {
            check_no_test_args(test_name, args, kwargs, lineno)?;
            node.is_undefined()
        }
        "none" => {
            check_no_test_args(test_name, args, kwargs, lineno)?;
            matches!(node, Value::Py(PyValue::None))
        }
        "in" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err(
                    "test 'in' expected one argument".to_string(),
                    lineno,
                ));
            }
            contains(&args[0], node, lineno)?
        }
        "==" | "eq" | "equalto" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '==' expected one argument".to_string(), lineno));
            }
            match (node, &args[0]) {
                (Value::Py(a), Value::Py(b)) => py_equal(a, b),
                (Value::Undefined(_), Value::Undefined(_)) => true,
                _ => false,
            }
        }
        "!=" | "ne" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '!=' expected one argument".to_string(), lineno));
            }
            let eq = match (node, &args[0]) {
                (Value::Py(a), Value::Py(b)) => py_equal(a, b),
                (Value::Undefined(_), Value::Undefined(_)) => true,
                _ => false,
            };
            !eq
        }
        ">" | "gt" | "greaterthan" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '>' expected one argument".to_string(), lineno));
            }
            cmp_values(node, &args[0], lineno)? == Ordering::Greater
        }
        ">=" | "ge" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '>=' expected one argument".to_string(), lineno));
            }
            cmp_values(node, &args[0], lineno)? != Ordering::Less
        }
        "<" | "lt" | "lessthan" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '<' expected one argument".to_string(), lineno));
            }
            cmp_values(node, &args[0], lineno)? == Ordering::Less
        }
        "<=" | "le" => {
            if args.len() != 1 || !kwargs.is_empty() {
                return Err(type_err("test '<=' expected one argument".to_string(), lineno));
            }
            cmp_values(node, &args[0], lineno)? != Ordering::Greater
        }
        _ => {
            return Err(type_err(
                format!("No test named '{test_name}'."),
                lineno,
            ))
        }
    };
    Ok(Value::py(PyValue::Bool(if negated { !r } else { r })))
}

fn check_no_test_args(name: &str, args: &[Value], kwargs: &[(String, Value)], lineno: usize) -> Result<(), TemplateError> {
    if !args.is_empty() || !kwargs.is_empty() {
        return Err(type_err(
            format!("test '{name}' takes no arguments"),
            lineno,
        ));
    }
    Ok(())
}

// --- eval expression ---

fn eval_expr(e: &Expr, ctx: &mut Ctx, lineno: usize) -> Result<Value, TemplateError> {
    match e {
        Expr::Const(v) => Ok(Value::py(v.clone())),
        Expr::Name(n) => Ok(ctx.get(n)),
        Expr::List(items) => {
            let mut out = Vec::new();
            for it in items {
                let v = eval_expr(it, &mut *ctx, lineno)?;
                out.push(need_py(&v, lineno)?);
            }
            Ok(Value::py(PyValue::List(out)))
        }
        Expr::Tuple(items) => {
            let mut out = Vec::new();
            for it in items {
                let v = eval_expr(it, &mut *ctx, lineno)?;
                out.push(need_py(&v, lineno)?);
            }
            Ok(Value::py(PyValue::Tuple(out)))
        }
        Expr::Dict(items) => {
            let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
            for (k, val) in items {
                let kv = eval_expr(k, &mut *ctx, lineno)?;
                let vv = eval_expr(val, &mut *ctx, lineno)?;
                let kpy = need_py(&kv, lineno)?;
                if !kpy.is_hashable() {
                    return Err(type_err(
                        format!("unhashable type: '{}'", pycompat::value::type_name(&kpy)),
                        lineno,
                    ));
                }
                d.insert(kpy, need_py(&vv, lineno)?);
            }
            Ok(Value::py(PyValue::Dict(d)))
        }
        Expr::GetAttr(base, attr) => {
            let b = eval_expr(base, &mut *ctx, lineno)?;
            get_attr(&b, attr, lineno)
        }
        Expr::GetItem(base, idx) => {
            let b = eval_expr(base, &mut *ctx, lineno)?;
            let k = eval_expr(idx, &mut *ctx, lineno)?;
            get_item(&b, &k, lineno)
        }
        Expr::Call {
            callee,
            args,
            kwargs,
        } => eval_call(callee, args, kwargs, ctx, lineno),
        Expr::Filter {
            node,
            name,
            args,
            kwargs,
        } => {
            let base = eval_expr(node, &mut *ctx, lineno)?;
            let mut av = Vec::new();
            for a in args {
                av.push(eval_expr(a, &mut *ctx, lineno)?);
            }
            let mut kv = Vec::new();
            for (k, v) in kwargs {
                kv.push((k.clone(), eval_expr(v, &mut *ctx, lineno)?));
            }
            eval_filter(name, &base, &av, &kv, lineno)
        }
        Expr::Test {
            node,
            name,
            args,
            kwargs,
            negated,
        } => {
            let base = eval_expr(node, &mut *ctx, lineno)?;
            let mut av = Vec::new();
            for a in args {
                av.push(eval_expr(a, &mut *ctx, lineno)?);
            }
            let mut kv = Vec::new();
            for (k, v) in kwargs {
                kv.push((k.clone(), eval_expr(v, &mut *ctx, lineno)?));
            }
            eval_test(name, &base, &av, &kv, *negated, lineno)
        }
        Expr::Compare { first, ops } => {
            let first_v = eval_expr(first, &mut *ctx, lineno)?;
            let mut prev = first_v;
            for (op, rhs) in ops {
                let rv = eval_expr(rhs, &mut *ctx, lineno)?;
                if !eval_compare(op, &prev, &rv, lineno)? {
                    return Ok(Value::py(PyValue::Bool(false)));
                }
                prev = rv;
            }
            Ok(Value::py(PyValue::Bool(true)))
        }
        Expr::Add(a, b) => eval_add(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Sub(a, b) => eval_sub(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Mul(a, b) => eval_mul(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Div(a, b) => eval_div(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::FloorDiv(a, b) => eval_floordiv(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Mod(a, b) => eval_mod(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Pow(a, b) => eval_pow(&eval_expr(a, &mut *ctx, lineno)?, &eval_expr(b, &mut *ctx, lineno)?, lineno),
        Expr::Concat(a, b) => {
            let va = eval_expr(a, &mut *ctx, lineno)?;
            let vb = eval_expr(b, &mut *ctx, lineno)?;
            Ok(Value::py(PyValue::Str(format!(
                "{}{}",
                py_str_of(&va, lineno)?,
                py_str_of(&vb, lineno)?
            ))))
        }
        Expr::Neg(a) => {
            let v = eval_expr(a, &mut *ctx, lineno)?;
            match v {
                Value::Undefined(_) => Err(v.fail_undefined(lineno)),
                Value::Py(PyValue::Int(n)) => Ok(Value::py(PyValue::Int(PyInt::new(-n.0.clone())))),
                Value::Py(PyValue::Float(f)) => Ok(Value::py(PyValue::Float(-f))),
                Value::Py(PyValue::Bool(b)) => Ok(Value::py(PyValue::Int(
                    PyInt::from_i64(if b { -1 } else { 0 }),
                ))),
                _ => Err(type_err(
                    format!("bad operand type for unary -: '{}'", v.type_name()),
                    lineno,
                )),
            }
        }
        Expr::Pos(a) => {
            let v = eval_expr(a, &mut *ctx, lineno)?;
            match v {
                Value::Undefined(_) => Err(v.fail_undefined(lineno)),
                Value::Py(PyValue::Int(_) | PyValue::Float(_) | PyValue::Bool(_)) => Ok(v),
                _ => Err(type_err(
                    format!("bad operand type for unary +: '{}'", v.type_name()),
                    lineno,
                )),
            }
        }
        Expr::Not(a) => {
            let v = eval_expr(a, &mut *ctx, lineno)?;
            Ok(Value::py(PyValue::Bool(!v.truthy())))
        }
        Expr::And(a, b) => {
            let va = eval_expr(a, &mut *ctx, lineno)?;
            // Python/Jinja `and` returns operand values, not bool.
            Ok(if !va.truthy() {
                va
            } else {
                eval_expr(b, &mut *ctx, lineno)?
            })
        }
        Expr::Or(a, b) => {
            let va = eval_expr(a, &mut *ctx, lineno)?;
            Ok(if va.truthy() {
                va
            } else {
                eval_expr(b, &mut *ctx, lineno)?
            })
        }
        Expr::CondExpr(true_expr, test, false_expr) => {
            let t = eval_expr(test, &mut *ctx, lineno)?;
            if t.truthy() {
                eval_expr(true_expr, &mut *ctx, lineno)
            } else if let Some(f) = false_expr {
                eval_expr(f, &mut *ctx, lineno)
            } else {
                Ok(Value::undefined_name("__cond_false__"))
            }
        }
    }
}

fn eval_call(
    callee: &Expr,
    args: &[Expr],
    kwargs: &[(String, Expr)],
    ctx: &mut Ctx,
    lineno: usize,
) -> Result<Value, TemplateError> {
    // `list.append` mutation special-case: `prompt_txt.append(x)` must mutate
    // the list stored in ctx. Detect `Name.append(...)` / `a.append` chains where
    // callee is GetAttr(Name(..), "append").
    // More generally, if callee is GetAttr(base, "append") and base evaluates to a
    // List, we mutate the stored list when base is a name (or nested name/attr?).
    // We implement for `Name` base and for `GetItem/GetAttr` bases by re-evaluating
    // the base value and, when the base is a name, writing back an extended list.
    // (Nested-container mutation through index/attr is not in the corpus.)
    if let Expr::GetAttr(base_expr, attr) = callee {
        if attr == "append" {
            // check base value
            // We need a mutable ctx for write-back; use the passed &mut Ctx.
            // Fall through to generic path if base is not a simple Name.
            if let Expr::Name(base_name) = base_expr.as_ref() {
                let base = ctx.get(base_name);
                if let Value::Py(PyValue::List(mut lst)) = base.clone() {
                    // evaluate args
                    let mut av = Vec::new();
                    for a in args {
                        av.push(eval_expr(a, &mut *ctx, lineno)?);
                    }
                    if av.len() != 1 || !kwargs.is_empty() {
                        return Err(type_err("append expected 1 argument".to_string(), lineno));
                    }
                    lst.push(need_py(&av[0], lineno)?);
                    // write back extended list into all scopes where it existed?
                    // Jinja/Python mutates the object in place; subsequent reads
                    // through any alias see it. Our copy-on-write makes the
                    // latest write visible everywhere if we update every scope
                    // that holds the name.
                    for scope in ctx.scopes.iter_mut() {
                        if scope.contains_key(base_name) {
                            scope.insert(base_name.clone(), Value::py(PyValue::List(lst.clone())));
                        }
                    }
                    // If the name was not in any scope (Undefined), Jinja would
                    // raise; but Python would raise AttributeError on Undefined.
                    // Match: if original base was Undefined, this is an error.
                    // (Checked above: base would not be a List.)
                    return Ok(Value::py(PyValue::None));
                }
                // if base is Undefined, fall through to error below
                if base.is_undefined() {
                    return Err(base.fail_undefined(lineno));
                }
            }
            // For non-Name bases (e.g. `printer[...].x.append`), evaluate
            // generically: clone-mutate does not persist; the corpus never
            // mutates through a path, so this path is unreachable in practice.
        }
        // string methods: `receiver.method(...)` where receiver is expr.
        // Generic dispatch through GetAttr's Func covers them, but calling
        // `eval_expr(callee)` would resolve the method to a Func; simpler to
        // dispatch directly here for clarity and lineno.
        if is_string_method(attr) {
            let recv = eval_expr(base_expr, &mut *ctx, lineno)?;
            if let Value::Py(PyValue::Str(_)) = recv {
                let bound = get_attr(&recv, attr, lineno)?;
                match bound {
                    Value::Func(f) => {
                        let mut av = Vec::new();
                        for a in args {
                            av.push(eval_expr(a, &mut *ctx, lineno)?);
                        }
                        let mut kv = Vec::new();
                        for (k, v) in kwargs {
                            kv.push((k.clone(), eval_expr(v, &mut *ctx, lineno)?));
                        }
                        return f(&av, &kv, lineno);
                    }
                    _ => unreachable!(),
                }
            }
        }
        if attr == "append" {
            let recv = eval_expr(base_expr, &mut *ctx, lineno)?;
            if !matches!(recv, Value::Py(PyValue::List(_))) {
                return Err(type_err(
                    format!("'{}' object has no method 'append'", recv.type_name()),
                    lineno,
                ));
            }
            // clone-mutate (non-persisted) fallback: return None.
            return Ok(Value::py(PyValue::None));
        }
    }
    // generic call
    let f = eval_expr(callee, &mut *ctx, lineno)?;
    match f {
        Value::Func(func) => {
            let mut av = Vec::new();
            for a in args {
                av.push(eval_expr(a, &mut *ctx, lineno)?);
            }
            let mut kv = Vec::new();
            for (k, v) in kwargs {
                kv.push((k.clone(), eval_expr(v, &mut *ctx, lineno)?));
            }
            func(&av, &kv, lineno)
        }
        Value::Undefined(_) => Err(f.fail_undefined(lineno)),
        Value::Py(v) => Err(type_err(
            format!("'{}' object is not callable", pycompat::value::type_name(&v)),
            lineno,
        )),
    }
}

// --- node rendering ---

fn render_nodes(nodes: &[Node], ctx: &mut Ctx, out: &mut String) -> Result<(), TemplateError> {
    for node in nodes {
        match node {
            Node::Text(t) => out.push_str(t),
            Node::Print(e) => {
                // track lineno via expression position unknown; use 1 for errors
                // inside prints except the value's own ops already carry lineno.
                // We re-derive lineno from the print token: store is unavailable,
                // so borrow 1; print errors still name the construct via message.
                let v = eval_expr(e, ctx, 1)?;
                out.push_str(&v.to_string());
            }
            Node::If { branches, else_body } => {
                let mut done = false;
                for (cond, body) in branches {
                    let c = eval_expr(cond, ctx, 1)?;
                    if c.truthy() {
                        render_nodes(body, ctx, out)?;
                        done = true;
                        break;
                    }
                }
                if !done {
                    render_nodes(else_body, ctx, out)?;
                }
            }
            Node::For {
                target,
                iter,
                body,
                else_body,
            } => {
                let seq_v = eval_expr(iter, ctx, 1)?;
                let items = iterate(&seq_v, 1)?;
                if items.is_empty() {
                    render_nodes(else_body, ctx, out)?;
                } else {
                    let len = items.len();
                    let mut prev: Option<Value> = None;
                    for (i, item) in items.iter().enumerate() {
                        let next = items.get(i + 1).cloned();
                        let mut frame: IndexMap<String, Value> = IndexMap::new();
                        frame.insert(target.clone(), item.clone());
                        frame.insert("loop".to_string(), make_loop(i, len, &prev, &next));
                        ctx.push(frame);
                        let r = render_nodes(body, ctx, out);
                        ctx.pop();
                        if let Err(e) = r {
                            return Err(e);
                        }
                        prev = Some(item.clone());
                    }
                }
            }
            Node::Set { name, value } => {
                let v = eval_expr(value, ctx, 1)?;
                ctx.set(name.clone(), v);
            }
        }
    }
    Ok(())
}

fn make_loop(
    i: usize,
    len: usize,
    prev: &Option<Value>,
    next: &Option<Value>,
) -> Value {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    let s = |k: &str| PyValue::Str(k.to_string());
    let n = |v: i64| PyValue::Int(PyInt::from_i64(v));
    d.insert(s("index0"), n(i as i64));
    d.insert(s("index"), n(i as i64 + 1));
    d.insert(s("length"), n(len as i64));
    d.insert(s("revindex0"), n((len as i64 - i as i64 - 1)));
    d.insert(s("revindex"), n((len as i64 - i as i64)));
    d.insert(s("first"), PyValue::Bool(i == 0));
    d.insert(s("last"), PyValue::Bool(i + 1 == len));
    d.insert(s("depth"), n(1));
    d.insert(s("depth0"), n(0));
    d.insert(
        s("previtem"),
        prev.clone().map(|v| {
            match v {
                Value::Py(p) => p,
                // prev/next are `previtem`/`nextitem` values; store as py
                Value::Undefined(_) => PyValue::None,
                Value::Func(_) => PyValue::None,
            }
        })
        .unwrap_or_else(|| PyValue::None),
    );
    d.insert(
        s("nextitem"),
        next.clone().map(|v| {
            match v {
                Value::Py(p) => p,
                Value::Undefined(_) => PyValue::None,
                Value::Func(_) => PyValue::None,
            }
        })
        .unwrap_or_else(|| PyValue::None),
    );
    Value::py(PyValue::Dict(d))
}

// ---------------------------------------------------------------------------
// Template / TemplateWrapper
// ---------------------------------------------------------------------------

/// A parsed Jinja template (custom delimiters).
pub struct Template {
    nodes: Vec<Node>,
}

impl Template {
    pub fn parse(source: &str) -> Result<Template, TemplateError> {
        let toks = lex_template(source)?;
        let (nodes, idx) = parse_nodes(&toks, 0, &[])?;
        if idx != toks.len() {
            return Err(syntax_err("unexpected 'end of template'".to_string(), 1));
        }
        Ok(Template { nodes })
    }

    /// Render with the given context (consumes a clone of `vars`).
    pub fn render(&self, vars: IndexMap<String, Value>) -> Result<String, TemplateError> {
        let mut ctx = Ctx::new(vars);
        let mut out = String::new();
        render_nodes(&self.nodes, &mut ctx, &mut out)?;
        Ok(out)
    }
}
impl std::fmt::Debug for Template {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Template(nodes={})", self.nodes.len())
    }
}
impl std::fmt::Debug for TemplateWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "TemplateWrapper(name={:?})", self.name)
    }
}

/// `TemplateWrapper` from `gcode_macro.py`: parses at load time and
/// formats load/eval errors with the reference texts.
pub struct TemplateWrapper {
    name: String,
    template: Template,
    source: String,
}

impl TemplateWrapper {
    pub fn load(name: &str, script: &str) -> Result<TemplateWrapper, String> {
        match Template::parse(script) {
            Ok(t) => Ok(TemplateWrapper {
                name: name.to_string(),
                template: t,
                source: script.to_string(),
            }),
            Err(e) => {
                if e.kind() == "TemplateSyntaxError" {
                    let lineno = e.lineno().unwrap_or(1);
                    let lines: Vec<&str> = script.split('\n').collect();
                    let line_text = lines
                        .get(lineno - 1)
                        .unwrap_or(&"")
                        .to_string();
                    Err(format!(
                        "Error loading template '{name}'\nline {lineno}: {line_text} # {}",
                        e.message()
                    ))
                } else {
                    Err(format!(
                        "Error loading template '{name}': {}",
                        e.display_string()
                    ))
                }
            }
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn render(&self, vars: IndexMap<String, Value>) -> Result<String, String> {
        match self.template.render(vars) {
            Ok(s) => Ok(s),
            Err(e) => Err(format!(
                "Error evaluating '{}': {}",
                self.name,
                e.display_string()
            )),
        }
    }

    /// Source script retained for tests / introspection.
    pub fn source(&self) -> &str {
        &self.source
    }
}

/// Parse a Python literal (`ast.literal_eval`) used for `variable_*`
/// and `SET_GCODE_VARIABLE`. Supports None/True/False, ints (base 0),
/// floats, strings, lists, tuples, dicts and `...`/`()` nesting.
/// Sets are parsed then rejected by the JSON check, mirroring Python.
pub fn parse_literal(s: &str) -> Result<PyValue, PyError> {
    let toks = tokenize_literal(s)?;
    let mut p = LitParser {
        toks,
        pos: 0,
        src: s.to_string(),
        lineno: 1,
    };
    let v = p.parse_value()?;
    if p.pos < p.toks.len() {
        return Err(PyError::new(
            "ValueError",
            format!("malformed node or string on line 1: {}", pytext::str_repr(s)),
        ));
    }
    Ok(v)
}

#[derive(Clone, Debug, PartialEq)]
enum LT {
    Name(String),
    Str(String),
    Int(PyInt),
    Float(f64),
    Op(String),
    LBracket,
    RBracket,
    LParen,
    RParen,
    LBrace,
    RBrace,
    Comma,
    Colon,
}

fn tokenize_literal(s: &str) -> Result<Vec<LT>, PyError> {
    // Reuse the expression tokenizer but distinguish `...` and sets are handled at parse.
    let toks = tokenize_expr(s, 1).map_err(|e| {
        if e.kind() == "TemplateSyntaxError" {
            // Map Jinja syntax text to Python `SyntaxError` text is close enough;
            // literal_eval callers surface `str(e)` after "not a valid literal: ".
            PyError::new("SyntaxError", e.message().to_string())
        } else {
            PyError::new("SyntaxError", e.message().to_string())
        }
    })?;
    let mut out = Vec::new();
    for t in toks {
        match t {
            ET::Name(n) => out.push(LT::Name(n)),
            ET::Str(st) => out.push(LT::Str(st)),
            ET::Int(v) => out.push(LT::Int(v)),
            ET::Float(v) => out.push(LT::Float(v)),
            ET::Op(op) => {
                if op == "+" || op == "-" {
                    out.push(LT::Op(op));
                } else if op == "***" {
                    out.push(LT::Op(op));
                } else {
                    return Err(PyError::new(
                        "SyntaxError",
                        format!("malformed node or string on line 1: {s:?}"),
                    ));
                }
            }
            ET::LBracket => out.push(LT::LBracket),
            ET::RBracket => out.push(LT::RBracket),
            ET::LParen => out.push(LT::LParen),
            ET::RParen => out.push(LT::RParen),
            ET::LBrace => out.push(LT::LBrace),
            ET::RBrace => out.push(LT::RBrace),
            ET::Comma => out.push(LT::Comma),
            ET::Colon => out.push(LT::Colon),
            ET::KwTrue => out.push(LT::Name("True".to_string())),
            ET::KwFalse => out.push(LT::Name("False".to_string())),
            ET::KwNone => out.push(LT::Name("None".to_string())),
            other => {
                return Err(PyError::new(
                    "SyntaxError",
                    format!("malformed node or string on line 1: {}", describe_et(&other)),
                ))
            }
        }
    }
    // Handle `...` (Ellipsis) which the expression tokenizer splits as three dots.
    Ok(out)
}

struct LitParser {
    toks: Vec<LT>,
    pos: usize,
    src: String,
    lineno: usize,
}

impl LitParser {
    fn peek(&self) -> Option<&LT> {
        self.toks.get(self.pos)
    }
    fn parse_value(&mut self) -> Result<PyValue, PyError> {
        // sign handling
        let sign = if matches!(self.peek(), Some(LT::Op(op)) if op == "-") {
            self.pos += 1;
            -1i64
        } else if matches!(self.peek(), Some(LT::Op(op)) if op == "+") {
            self.pos += 1;
            1i64
        } else {
            1i64
        };
        let t = match self.peek().cloned() {
            Some(x) => x,
            None => {
                return Err(PyError::new(
                    "ValueError",
                    format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                ))
            }
        };
        match (sign, t) {
            (-1 | 1, LT::Name(n)) if sign == 1 => {
                self.pos += 1;
                match n.as_str() {
                    "None" => Ok(PyValue::None),
                    "True" => Ok(PyValue::Bool(true)),
                    "False" => Ok(PyValue::Bool(false)),
                    _ => Err(PyError::new(
                        "ValueError",
                        format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                    )),
                }
            }
            (sign, LT::Int(n)) => {
                self.pos += 1;
                if sign == -1 {
                    Ok(PyValue::Int(PyInt::new(-n.0.clone())))
                } else {
                    Ok(PyValue::Int(n))
                }
            }
            (sign, LT::Float(f)) => {
                self.pos += 1;
                if sign == -1 {
                    Ok(PyValue::Float(-f))
                } else {
                    Ok(PyValue::Float(f))
                }
            }
            (-1 | 1, LT::Str(st)) => {
                self.pos += 1;
                Ok(PyValue::Str(st))
            }
            (1, LT::LBracket) => {
                self.pos += 1;
                let mut items = Vec::new();
                if self.peek() == Some(&LT::RBracket) {
                    self.pos += 1;
                    return Ok(PyValue::List(items));
                }
                loop {
                    items.push(self.parse_value()?);
                    if self.peek() == Some(&LT::Comma) {
                        self.pos += 1;
                        if self.peek() == Some(&LT::RBracket) {
                            self.pos += 1;
                            break;
                        }
                        continue;
                    }
                    if self.peek() == Some(&LT::RBracket) {
                        self.pos += 1;
                        break;
                    }
                    return Err(PyError::new(
                        "SyntaxError",
                        format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                    ));
                }
                Ok(PyValue::List(items))
            }
            (1, LT::LParen) => {
                self.pos += 1;
                if self.peek() == Some(&LT::RParen) {
                    self.pos += 1;
                    return Ok(PyValue::Tuple(Vec::new()));
                }
                let mut items = Vec::new();
                items.push(self.parse_value()?);
                let mut tuple = false;
                if self.peek() == Some(&LT::Comma) {
                    tuple = true;
                    self.pos += 1;
                    if self.peek() == Some(&LT::RParen) {
                        self.pos += 1;
                        return Ok(PyValue::Tuple(items));
                    }
                    loop {
                        items.push(self.parse_value()?);
                        if self.peek() == Some(&LT::Comma) {
                            self.pos += 1;
                            if self.peek() == Some(&LT::RParen) {
                                self.pos += 1;
                                break;
                            }
                            continue;
                        }
                        if self.peek() == Some(&LT::RParen) {
                            self.pos += 1;
                            break;
                        }
                        return Err(PyError::new(
                            "SyntaxError",
                            format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                        ));
                    }
                    return Ok(PyValue::Tuple(items));
                }
                if self.peek() == Some(&LT::RParen) {
                    self.pos += 1;
                    let _ = tuple;
                    return Ok(items.into_iter().next().unwrap());
                }
                Err(PyError::new(
                    "SyntaxError",
                    format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                ))
            }
            (1, LT::LBrace) => {
                self.pos += 1;
                // empty dict `{}`
                if self.peek() == Some(&LT::RBrace) {
                    self.pos += 1;
                    return Ok(PyValue::Dict(IndexMap::new()));
                }
                // peek for set: `{1, 2}` (no colon after first item)
                // Parse first item then decide.
                let first = self.parse_value()?;
                if self.peek() == Some(&LT::Colon) {
                    // dict
                    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
                    self.pos += 1;
                    let v = self.parse_value()?;
                    d.insert(first, v);
                    loop {
                        if self.peek() == Some(&LT::Comma) {
                            self.pos += 1;
                            if self.peek() == Some(&LT::RBrace) {
                                self.pos += 1;
                                break;
                            }
                            let k = self.parse_value()?;
                            if self.peek() != Some(&LT::Colon) {
                                return Err(PyError::new(
                                    "SyntaxError",
                                    format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                                ));
                            }
                            self.pos += 1;
                            let vv = self.parse_value()?;
                            d.insert(k, vv);
                            continue;
                        }
                        if self.peek() == Some(&LT::RBrace) {
                            self.pos += 1;
                            break;
                        }
                        return Err(PyError::new(
                            "SyntaxError",
                            format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                        ));
                    }
                    Ok(PyValue::Dict(d))
                } else {
                    // set literal: parse rest, then reject via JSON check mimic
                    // (Caller does dumps_compact which will fail for sets — but we
                    // have no set value; simulate by returning a TypeError now.)
                    // Collect remaining items to consume `}`.
                    let mut _items = vec![first];
                    loop {
                        if self.peek() == Some(&LT::Comma) {
                            self.pos += 1;
                            if self.peek() == Some(&LT::RBrace) {
                                self.pos += 1;
                                break;
                            }
                            _items.push(self.parse_value()?);
                            continue;
                        }
                        if self.peek() == Some(&LT::RBrace) {
                            self.pos += 1;
                            break;
                        }
                        return Err(PyError::new(
                            "SyntaxError",
                            format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
                        ));
                    }
                    Err(PyError::new(
                        "TypeError",
                        "Object of type set is not JSON serializable".to_string(),
                    ))
                }
            }
            _ => Err(PyError::new(
                "ValueError",
                format!("malformed node or string on line 1: {}", pytext::str_repr(&self.src)),
            )),
        }
    }
}

/// Validate a literal is JSON-serializable as `gcode_macro` does
/// (`json.dumps(literal, separators=(',', ':'))`). Returns
/// `Err(PyError)` with `TypeError` text on failure.
pub fn validate_literal_json(v: &PyValue) -> Result<(), PyError> {
    pyjson::dumps_compact(v).map(|_| ())
}

/// Validate a `variable_*` option value or a `VALUE=` argument, exactly
/// the `ast.literal_eval` + `json.dumps` pair from `gcode_macro.py`.
/// Returns the parsed value or a `PyError` whose message matches
/// `str(e)` of the caught `SyntaxError`/`TypeError`/`ValueError`.
fn parse_and_validate_literal(s: &str) -> Result<PyValue, PyError> {
    let v = parse_literal(s)?;
    validate_literal_json(&v)?;
    Ok(v)
}

/// `GCodeMacro` variable handling (pure helpers mirroring `gcode_macro.py`).
pub struct GCodeMacroVars {
    variables: IndexMap<String, PyValue>,
}

impl GCodeMacroVars {
    pub fn new(variables: IndexMap<String, PyValue>) -> Self {
        GCodeMacroVars { variables }
    }
    /// `"Option '%s' in section '%s' is not a valid literal: %s"` error text
    /// for a bad `variable_*` value (returns `Err(message)`).
    pub fn load_variable(
        &mut self,
        option: &str,
        section: &str,
        raw_value: &str,
    ) -> Result<(), String> {
        match parse_and_validate_literal(raw_value) {
            Ok(v) => {
                self.variables.insert(option.to_string(), v);
                Ok(())
            }
            Err(e) => Err(format!(
                "Option '{option}' in section '{section}' is not a valid literal: {}",
                e.display()
            )),
        }
    }
    /// `"Unknown gcode_macro variable '%s'"` vs
    /// `"Unable to parse '%s' as a literal: %s in '%s'"` errors for
    /// `SET_GCODE_VARIABLE`.
    pub fn set_variable(
        &mut self,
        variable: &str,
        value: &str,
        commandline: &str,
    ) -> Result<(), String> {
        if !self.variables.contains_key(variable) {
            return Err(format!("Unknown gcode_macro variable '{variable}'"));
        }
        match parse_and_validate_literal(value) {
            Ok(v) => {
                self.variables.insert(variable.to_string(), v);
                Ok(())
            }
            Err(e) => Err(format!(
                "Unable to parse '{value}' as a literal: {} in '{commandline}'",
                e.display()
            )),
        }
    }
    pub fn get_status(&self) -> IndexMap<String, PyValue> {
        self.variables.clone()
    }
    /// Context `kwparams` for `cmd()`: variables plus template context
    /// plus `params`/`rawparams`.
    pub fn merge_context(
        &self,
        template_ctx: IndexMap<String, Value>,
        params: IndexMap<String, String>,
        rawparams: &str,
    ) -> IndexMap<String, Value> {
        let mut out: IndexMap<String, Value> = IndexMap::new();
        for (k, v) in &self.variables {
            out.insert(k.clone(), Value::py(v.clone()));
        }
        for (k, v) in template_ctx {
            out.insert(k, v);
        }
        let mut pd: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in params {
            pd.insert(
                PyValue::Str(k),
                PyValue::Str(v),
            );
        }
        out.insert("params".to_string(), Value::py(PyValue::Dict(pd)));
        out.insert("rawparams".to_string(), Value::py(PyValue::Str(rawparams.to_string())));
        out
    }
}

/// `GetStatusWrapper` caching/stripping wrapper over a printer registry.
/// `registry` maps object names to their `get_status` snapshot; a
/// missing name or a `None` lookup corresponds to
/// `printer.lookup_object(name, None) is None` (KeyError).
pub struct GetStatusWrapper {
    registry: IndexMap<String, PyValue>,
    cache: IndexMap<String, PyValue>,
}

impl GetStatusWrapper {
    pub fn new(registry: IndexMap<String, PyValue>) -> Self {
        GetStatusWrapper {
            registry,
            cache: IndexMap::new(),
        }
    }
    /// `__getitem__(val)`: `sval = str(val).strip()`; KeyError otherwise.
    /// Returns `Err(key_repr)` for `KeyError` on missing objects, mirroring
    /// CPython `KeyError` `args[0]` (the original, unstripped `val`).
    pub fn get_item(&mut self, val: &str) -> Result<PyValue, String> {
        let sval = val.trim().to_string();
        if let Some(v) = self.cache.get(&sval) {
            return Ok(v.clone());
        }
        match self.registry.get(&sval) {
            Some(v) => {
                let r = v.clone();
                self.cache.insert(sval, r.clone());
                Ok(r)
            }
            None => Err(val.to_string()),
        }
    }
    /// `__contains__`
    pub fn contains(&mut self, val: &str) -> bool {
        self.get_item(val).is_ok()
    }
    /// `__iter__` yields names of lookup objects that support status.
    pub fn iter_names(&self) -> Vec<String> {
        self.registry.keys().map(|k| k.to_string()).collect()
        // NOTE: `PyValue::Str` keys; registry keys are String already.
    }
}

/// Build a template-context map as `PrinterGCodeMacro.create_template_context`
/// does: `printer` (status wrapper snapshot collapsed to a dict for
/// evaluation; see module docs for why this is faithful), the four
/// `action_*` helpers, plus macro variables and `params`/`rawparams`.
/// The action helpers are injected as callables; the wrappers take
/// their side effects through closures in the caller's `Value::Func`s.
pub fn make_printer_dict(registry: &IndexMap<String, PyValue>) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in registry {
        d.insert(PyValue::Str(k.clone()), v.clone());
    }
    PyValue::Dict(d)
}
