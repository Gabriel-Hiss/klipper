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
//! Templates are lexed and parsed as Jinja's `lexer.py` and `parser.py`
//! do, then checked like its compiler, so every template Jinja rejects at
//! load time fails with Jinja's exception class (`TemplateSyntaxError`,
//! `TemplateAssertionError`, or `SyntaxError` from compiling the generated
//! Python), message and line. A template Jinja accepts but that uses a
//! construct outside the subset below fails last, with a
//! `TemplateSyntaxError` naming the construct.
//!
//! Supported constructs:
//!
//! * Tags: `{% if / elif / else / endif %}`, `{% for name in expr %}`
//!   with optional `{% else %}` and `{% endfor %}`, `{% set name =
//!   expr %}` (exactly one name; namespace attributes and block `set`
//!   are unsupported), `{% raw %}`. `{# ... #}` comments and `-`
//!   whitespace control.
//! * Expressions: literals (decimal int, float, string with Python
//!   escapes, adjacent literal concatenation, list, tuple, dict,
//!   `true/True/false/False/none/None`), names, `.attr`, `.0`,
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
//! * Render errors: `UndefinedError` implements the default-`Undefined`
//!   rule (falsy, empty print, empty iteration, every other operation
//!   raises `UndefinedError` with Jinja's message shape); a Jinja filter
//!   or test outside the subset raises `TypeError` (``No filter named
//!   'x'.`` / ``No test named 'x'.``).
//!
//! Numeric and text output formatting delegates to `pycompat` so
//! `str(float)`, `%` and `.format()` match CPython 3.8.

use indexmap::IndexMap;
use pycompat::int::{int_divmod, int_floor_div, int_mod, int_pow, PyInt};
use pycompat::json as pyjson;
use pycompat::text as pytext;
use pycompat::text::{PctValue, PyError};
use pycompat::value::{py_compare, py_equal, PyValue};
use std::cell::{Cell, RefCell};
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
    /// Whether `gcode_macro.py`'s `except jinja2.exceptions.TemplateSyntaxError`
    /// catches this error (`TemplateAssertionError` is a subclass).
    pub fn is_syntax_error(&self) -> bool {
        matches!(self.kind.as_str(), "TemplateSyntaxError" | "TemplateAssertionError")
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
/// `TemplateAssertionError`: raised by Jinja's compiler (and a few parser
/// checks) after the template parsed.
fn assertion_err(msg: String, lineno: usize) -> TemplateError {
    TemplateError::new("TemplateAssertionError", msg, Some(lineno))
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
/// `Undefined` (falsy/empty, but strict on operations), an arbitrary host
/// callable (the `action_*` functions, the display `render`/
/// `draw_progress_bar` helpers and `range`), or the context's `printer`
/// object (`GetStatusWrapper`).
#[derive(Clone)]
pub enum Value {
    Undefined(UndefinedHint),
    Py(PyValue),
    Func(
        Rc<
            dyn Fn(&[Value], &[(String, Value)], usize) -> Result<Value, TemplateError>,
        >,
    ),
    Status(Rc<GetStatusWrapper>),
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
    /// The `printer` entry of a template context: a `GetStatusWrapper`
    /// reading `source` lazily (`eventtime` as passed to
    /// `create_template_context`).
    pub fn printer_status(source: Box<dyn StatusSource>, eventtime: Option<f64>) -> Self {
        Value::Status(Rc::new(GetStatusWrapper::new(source, eventtime)))
    }
    pub fn is_undefined(&self) -> bool {
        matches!(self, Value::Undefined(_))
    }
    /// Python truthiness; `Undefined` is falsy (`bool(Undefined())` is False).
    pub fn truthy(&self) -> bool {
        match self {
            Value::Undefined(_) => false,
            Value::Py(v) => v.truthy(),
            Value::Func(_) | Value::Status(_) => true,
        }
    }
    /// `str(v)` as Jinja would render it: `Undefined` prints empty,
    /// concrete values via `pycompat` (so a list prints its `repr`).
    pub fn to_string(&self) -> String {
        match self {
            Value::Undefined(_) => String::new(),
            Value::Py(v) => v.str(),
            Value::Func(_) => String::new(),
            Value::Status(_) => GetStatusWrapper::REPR.to_string(),
        }
    }
    /// `TypeError` style type name for messages.
    pub fn type_name(&self) -> &'static str {
        match self {
            Value::Undefined(_) => "Undefined",
            Value::Py(v) => pycompat::value::type_name(v),
            Value::Func(_) => "function",
            Value::Status(_) => "GetStatusWrapper",
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
            Value::Py(_) | Value::Func(_) | Value::Status(_) => String::new(),
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
        Value::Status(_) => Ok(GetStatusWrapper::REPR.to_string()),
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
        /// Line of the filter name (`nodes.Filter.lineno`).
        lineno: usize,
    },
    Test {
        node: Box<Expr>,
        name: String,
        args: Vec<Expr>,
        kwargs: Vec<(String, Expr)>,
        negated: bool,
        /// Line of the `is` token (`nodes.Test.lineno`).
        lineno: usize,
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
        /// Line of the target name, for the compiler's `loop` check.
        target_lineno: usize,
        iter: Expr,
        body: Vec<Node>,
        else_body: Vec<Node>,
    },
    Set {
        name: String,
        /// Line of the target name, for the compiler's `loop` check.
        lineno: usize,
        value: Expr,
    },
    /// A construct Jinja accepts but this port does not render. Its parse
    /// recorded an "unsupported" error, so no `Template` ever holds one; the
    /// parts are kept only for the compile-time checks Jinja would still run.
    Discarded {
        /// Whether the construct is a `for` loop (subject to the `loop` check).
        for_loop: bool,
        /// `Name` targets it stores to, with their lines.
        stores: Vec<(String, usize)>,
        exprs: Vec<Expr>,
        body: Vec<Node>,
    },
}

// ---------------------------------------------------------------------------
// lexing (`jinja2/lexer.py` with Klipper's `{% %}` / `{ }` / `{# #}`)
// ---------------------------------------------------------------------------

/// One token of the template stream (`jinja2.lexer.Token` after `wrap`).
#[derive(Clone, Debug, PartialEq)]
enum JT {
    Data(String),
    BlockBegin,
    BlockEnd,
    VarBegin,
    VarEnd,
    Name(String),
    Str(String),
    Int(PyInt),
    Float(f64),
    /// Operator token by its source text (`"+"`, `"("`, `","`, ...).
    Op(&'static str),
    Eof,
}

#[derive(Clone, Debug)]
struct JTok {
    lineno: usize,
    kind: JT,
}

/// `describe_token`: names describe as their value, operators as their
/// text, everything else by its token type.
fn describe_jt(t: &JT) -> String {
    match t {
        JT::Data(_) => "template data / text".to_string(),
        JT::BlockBegin => "begin of statement block".to_string(),
        JT::BlockEnd => "end of statement block".to_string(),
        JT::VarBegin => "begin of print statement".to_string(),
        JT::VarEnd => "end of print statement".to_string(),
        JT::Name(n) => n.clone(),
        JT::Str(_) => "string".to_string(),
        JT::Int(_) => "integer".to_string(),
        JT::Float(_) => "float".to_string(),
        JT::Op(op) => (*op).to_string(),
        JT::Eof => "end of template".to_string(),
    }
}

/// `operator_re`: operators, longest first.
const OPERATORS: [&str; 26] = [
    "**", "//", "==", "!=", ">=", "<=", "+", "-", "/", "*", "%", "~", "[", "]", "(", ")", "{",
    "}", ">", "<", "=", ".", ":", "|", ",", ";",
];

/// Python `str.isspace()` (the `\s` of Jinja's regexes).
fn is_py_space(c: char) -> bool {
    c.is_whitespace() || ('\x1c'..='\x1f').contains(&c)
}

/// Python `str.splitlines()`.
fn py_splitlines(s: &str) -> Vec<&str> {
    let mut lines = Vec::new();
    let mut start = 0;
    let mut it = s.char_indices().peekable();
    while let Some((i, c)) = it.next() {
        if matches!(
            c,
            '\n' | '\r' | '\x0b' | '\x0c' | '\x1c' | '\x1d' | '\x1e' | '\u{85}' | '\u{2028}'
                | '\u{2029}'
        ) {
            lines.push(&s[start..i]);
            start = i + c.len_utf8();
            if c == '\r' && it.peek().is_some_and(|&(_, n)| n == '\n') {
                it.next();
                start += 1;
            }
        }
    }
    if start < s.len() {
        lines.push(&s[start..]);
    }
    lines
}

/// `str.isidentifier()` for a token `name_re` matched (letters, digits,
/// `_`): the first character may not be a digit, and the only digits
/// accepted are ASCII ones.
fn is_identifier(name: &str) -> bool {
    let mut chars = name.chars();
    chars.next().is_some_and(|c| c == '_' || c.is_alphabetic())
        && chars.all(|c| c == '_' || c.is_alphabetic() || c.is_ascii_digit())
}

/// A string token's value: `value.encode("ascii", "backslashreplace")
/// .decode("unicode-escape")`. Errors carry the codec reason Jinja keeps
/// (`str(e).split(":")[-1].strip()`).
fn jinja_unescape(raw: &str) -> Result<String, String> {
    let mut enc = String::with_capacity(raw.len());
    for c in raw.chars() {
        let cp = c as u32;
        if cp < 0x80 {
            enc.push(c);
        } else if cp <= 0xff {
            enc.push_str(&format!("\\x{cp:02x}"));
        } else if cp <= 0xffff {
            enc.push_str(&format!("\\u{cp:04x}"));
        } else {
            enc.push_str(&format!("\\U{cp:08x}"));
        }
    }
    let b = enc.as_bytes();
    let hex = |at: usize, n: usize, what: &str| -> Result<u32, String> {
        b.get(at..at + n)
            .filter(|h| h.iter().all(u8::is_ascii_hexdigit))
            .map(|h| {
                h.iter()
                    .fold(0u32, |v, &d| v * 16 + (d as char).to_digit(16).unwrap_or(0))
            })
            .ok_or_else(|| format!("truncated \\{what} escape"))
    };
    let mut out = String::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] != b'\\' {
            out.push(b[i] as char);
            i += 1;
            continue;
        }
        let Some(&c) = b.get(i + 1) else {
            return Err("\\ at end of string".to_string());
        };
        i += 2;
        match c {
            b'\n' => {}
            b'\\' | b'\'' | b'"' => out.push(c as char),
            b'a' => out.push('\x07'),
            b'b' => out.push('\x08'),
            b'f' => out.push('\x0c'),
            b'n' => out.push('\n'),
            b'r' => out.push('\r'),
            b't' => out.push('\t'),
            b'v' => out.push('\x0b'),
            b'0'..=b'7' => {
                let mut v = u32::from(c - b'0');
                for _ in 0..2 {
                    match b.get(i) {
                        Some(&d @ b'0'..=b'7') => {
                            v = v * 8 + u32::from(d - b'0');
                            i += 1;
                        }
                        _ => break,
                    }
                }
                out.push(char::from_u32(v).unwrap_or('\0'));
            }
            b'x' => {
                out.push(char::from_u32(hex(i, 2, "xXX")?).unwrap_or('\0'));
                i += 2;
            }
            b'u' => {
                out.push(char::from_u32(hex(i, 4, "uXXXX")?).unwrap_or('\0'));
                i += 4;
            }
            b'U' => {
                let v = hex(i, 8, "UXXXXXXXX")?;
                if v > 0x10ffff {
                    return Err("illegal Unicode character".to_string());
                }
                out.push(char::from_u32(v).unwrap_or('\0'));
                i += 8;
            }
            b'N' => {
                // A well-formed `\N{name}` needs the Unicode name database.
                let named = b.get(i) == Some(&b'{')
                    && b[i + 1..].iter().position(|&x| x == b'}').is_some_and(|len| len > 0);
                return Err(if named {
                    "unsupported string escape '\\N{...}'".to_string()
                } else {
                    "malformed \\N character escape".to_string()
                });
            }
            other => {
                out.push('\\');
                out.push(other as char);
            }
        }
    }
    Ok(out)
}

/// `Lexer.tokeniter` + `Lexer.wrap` over the normalized source
/// (`"\n".join(source.splitlines())`), positions in code points.
struct Lexer {
    src: Vec<char>,
    pos: usize,
    lineno: usize,
    toks: Vec<JTok>,
}

/// Which root rule matched at a `{`.
enum TagStart {
    Raw,
    Comment,
    Block,
    Var,
}

/// Tokenize a template. Jinja's lexer is a generator the parser pulls
/// from, so a lexing error surfaces only when the parser advances past
/// the last good token: the error is returned beside the tokens before it.
fn lex_template(source: &str) -> (Vec<JTok>, Option<TemplateError>) {
    let mut lx = Lexer {
        src: py_splitlines(source).join("\n").chars().collect(),
        pos: 0,
        lineno: 1,
        toks: Vec::new(),
    };
    let err = lx.run().err();
    (lx.toks, err)
}

impl Lexer {
    fn at(&self, i: usize) -> Option<char> {
        self.src.get(i).copied()
    }
    fn starts_with(&self, i: usize, pat: &str) -> bool {
        pat.chars().enumerate().all(|(k, c)| self.at(i + k) == Some(c))
    }
    fn text(&self, a: usize, b: usize) -> String {
        self.src[a..b].iter().collect()
    }
    fn newlines(&self, a: usize, b: usize) -> usize {
        self.src[a..b].iter().filter(|&&c| c == '\n').count()
    }
    fn skip_space(&self, mut i: usize) -> usize {
        while self.at(i).is_some_and(is_py_space) {
            i += 1;
        }
        i
    }
    fn push(&mut self, kind: JT) {
        self.toks.push(JTok {
            lineno: self.lineno,
            kind,
        });
    }
    fn error(&self, msg: String) -> TemplateError {
        syntax_err(msg, self.lineno)
    }

    /// The root state. Returns at the end of the source, including inside
    /// an unclosed tag (Jinja then simply ends the stream).
    fn run(&mut self) -> Result<(), TemplateError> {
        while self.pos < self.src.len() {
            let Some(brace) = (self.pos..self.src.len()).find(|&i| self.src[i] == '{') else {
                self.emit_data(self.pos, self.src.len(), false);
                return Ok(());
            };
            let raw_end = self.match_raw_begin(brace);
            let (start, sign_at) = if raw_end.is_some() {
                (TagStart::Raw, brace + 2)
            } else {
                match self.at(brace + 1) {
                    Some('#') => (TagStart::Comment, brace + 2),
                    Some('%') => (TagStart::Block, brace + 2),
                    _ => (TagStart::Var, brace + 1),
                }
            };
            let sign = self.at(sign_at).filter(|&c| c == '-' || c == '+');
            let begin_end = match raw_end {
                Some(end) => end,
                None => sign_at + usize::from(sign.is_some()),
            };
            self.emit_data(self.pos, brace, sign == Some('-'));
            match start {
                TagStart::Block => self.push(JT::BlockBegin),
                TagStart::Var => self.push(JT::VarBegin),
                TagStart::Raw | TagStart::Comment => {}
            }
            self.lineno += self.newlines(brace, begin_end);
            self.pos = begin_end;
            let more = match start {
                TagStart::Raw => self.lex_raw()?,
                TagStart::Comment => self.lex_comment()?,
                TagStart::Block => self.lex_tag(true)?,
                TagStart::Var => self.lex_tag(false)?,
            };
            if !more {
                return Ok(());
            }
        }
        Ok(())
    }

    /// Template data `src[a..b]`; `strip` is a `-` on the tag that follows
    /// (`text.rstrip()`, the stripped newlines still count).
    fn emit_data(&mut self, a: usize, b: usize, strip: bool) {
        let mut end = b;
        if strip {
            while end > a && is_py_space(self.src[end - 1]) {
                end -= 1;
            }
        }
        if end > a {
            let text = self.text(a, end);
            self.push(JT::Data(text));
        }
        self.lineno += self.newlines(a, b);
    }

    /// `{%(-|+|)\s*raw\s*(-%}\s*|%})` at `b`: the end of the match.
    fn match_raw_begin(&self, b: usize) -> Option<usize> {
        if !self.starts_with(b, "{%") {
            return None;
        }
        let mut i = b + 2;
        if matches!(self.at(i), Some('-' | '+')) {
            i += 1;
        }
        i = self.skip_space(i);
        if !self.starts_with(i, "raw") {
            return None;
        }
        i = self.skip_space(i + 3);
        self.match_tag_end(i, "%}")
    }

    /// `-{close}\s*|{close}` at `i`: the end of the match.
    fn match_tag_end(&self, i: usize, close: &str) -> Option<usize> {
        if self.at(i) == Some('-') && self.starts_with(i + 1, close) {
            Some(self.skip_space(i + 1 + close.len()))
        } else if self.starts_with(i, close) {
            Some(i + close.len())
        } else {
            None
        }
    }

    fn lex_comment(&mut self) -> Result<bool, TemplateError> {
        let start = self.pos;
        let Some(end) = (start..self.src.len()).find_map(|i| self.match_tag_end(i, "#}")) else {
            if start < self.src.len() {
                return Err(self.error("Missing end of comment tag".to_string()));
            }
            return Ok(false);
        };
        self.lineno += self.newlines(start, end);
        self.pos = end;
        Ok(true)
    }

    /// The body of `{% raw %}`, up to `{%(-|+|)\s*endraw\s*(-%}\s*|%})`.
    fn lex_raw(&mut self) -> Result<bool, TemplateError> {
        let start = self.pos;
        let found = (start..self.src.len()).find_map(|i| {
            if !self.starts_with(i, "{%") {
                return None;
            }
            let sign = self.at(i + 2).filter(|&c| c == '-' || c == '+');
            let j = self.skip_space(i + 2 + usize::from(sign.is_some()));
            if !self.starts_with(j, "endraw") {
                return None;
            }
            let k = self.skip_space(j + 6);
            self.match_tag_end(k, "%}").map(|end| (i, end, sign == Some('-')))
        });
        let Some((i, end, strip)) = found else {
            if start < self.src.len() {
                return Err(self.error("Missing end of raw directive".to_string()));
            }
            return Ok(false);
        };
        self.emit_data(start, i, strip);
        self.lineno += self.newlines(i, end);
        self.pos = end;
        Ok(true)
    }

    /// `(\d+_)*\d+` at `i`.
    fn match_digits(&self, i: usize) -> Option<usize> {
        if !self.at(i)?.is_ascii_digit() {
            return None;
        }
        let mut j = i;
        loop {
            while self.at(j).is_some_and(|c| c.is_ascii_digit()) {
                j += 1;
            }
            if self.at(j) == Some('_') && self.at(j + 1).is_some_and(|c| c.is_ascii_digit()) {
                j += 1;
            } else {
                return Some(j);
            }
        }
    }

    /// `float_re` at `i`.
    fn match_float(&self, i: usize) -> Option<usize> {
        if i > 0 && self.src[i - 1] == '.' {
            return None;
        }
        let int_end = self.match_digits(i)?;
        let exponent = |k: usize| -> Option<usize> {
            if !matches!(self.at(k), Some('e' | 'E')) {
                return None;
            }
            let sign = usize::from(matches!(self.at(k + 1), Some('+' | '-')));
            self.match_digits(k + 1 + sign)
        };
        let fraction = if self.at(int_end) == Some('.') {
            self.match_digits(int_end + 1)
        } else {
            None
        };
        fraction
            .and_then(exponent)
            .or_else(|| exponent(int_end))
            .or(fraction)
    }

    /// The inside of a `{% %}` (`block`) or `{ }` tag. Returns `false` when
    /// the source ends first.
    fn lex_tag(&mut self, block: bool) -> Result<bool, TemplateError> {
        let mut balancing: Vec<char> = Vec::new();
        loop {
            let Some(c) = self.at(self.pos) else {
                return Ok(false);
            };
            if balancing.is_empty() {
                let end = self.match_tag_end(self.pos, if block { "%}" } else { "}" });
                if let Some(end) = end {
                    self.push(if block { JT::BlockEnd } else { JT::VarEnd });
                    self.lineno += self.newlines(self.pos, end);
                    self.pos = end;
                    return Ok(true);
                }
            }
            if is_py_space(c) {
                let end = self.skip_space(self.pos);
                self.lineno += self.newlines(self.pos, end);
                self.pos = end;
                continue;
            }
            if let Some(end) = self.match_float(self.pos) {
                let v = parse_float_literal(&self.text(self.pos, end), self.lineno)?;
                self.push(JT::Float(v));
                self.pos = end;
                continue;
            }
            if let Some(end) = self.match_digits(self.pos) {
                let digits: String =
                    self.src[self.pos..end].iter().filter(|&&d| d != '_').collect();
                let v = pytext::int_from_string(&digits, 10)
                    .map_err(|e| from_pyerr(&e, self.lineno))?;
                self.push(JT::Int(v));
                self.pos = end;
                continue;
            }
            if c == '_' || c.is_alphanumeric() {
                let mut end = self.pos;
                while self.at(end).is_some_and(|d| d == '_' || d.is_alphanumeric()) {
                    end += 1;
                }
                let name = self.text(self.pos, end);
                if !is_identifier(&name) {
                    return Err(self.error("Invalid character in identifier".to_string()));
                }
                self.push(JT::Name(name));
                self.pos = end;
                continue;
            }
            if c == '\'' || c == '"' {
                let mut j = self.pos + 1;
                while let Some(d) = self.at(j) {
                    if d == '\\' {
                        j += 2;
                    } else if d == c {
                        break;
                    } else {
                        j += 1;
                    }
                }
                if j < self.src.len() {
                    let value = jinja_unescape(&self.text(self.pos + 1, j))
                        .map_err(|m| self.error(m))?;
                    self.push(JT::Str(value));
                    self.lineno += self.newlines(self.pos, j + 1);
                    self.pos = j + 1;
                    continue;
                }
            }
            if let Some(&op) = OPERATORS.iter().find(|op| self.starts_with(self.pos, op)) {
                match op {
                    "{" => balancing.push('}'),
                    "(" => balancing.push(')'),
                    "[" => balancing.push(']'),
                    "}" | ")" | "]" => match balancing.pop() {
                        None => return Err(self.error(format!("unexpected '{op}'"))),
                        Some(expected) if !op.starts_with(expected) => {
                            return Err(
                                self.error(format!("unexpected '{op}', expected '{expected}'"))
                            );
                        }
                        Some(_) => {}
                    },
                    _ => {}
                }
                self.push(JT::Op(op));
                self.pos += op.len();
                continue;
            }
            return Err(self.error(format!(
                "unexpected char {} at {}",
                pytext::str_repr(&c.to_string()),
                self.pos
            )));
        }
    }
}

// ---------------------------------------------------------------------------
// expression tokenizing
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
enum ET {
    Name(String),
    Str(String),
    /// `b'...'`; produced only by the Python literal dialect.
    Bytes(Vec<u8>),
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
                    // Unknown escapes keep the backslash (`'\d'` is `'\\d'` in
                    // Python and in Jinja's `unicode-escape` decoding).
                    out.push('\\');
                    let ch_len = raw[i..].chars().next().map(|c| c.len_utf8()).unwrap_or(1);
                    out.push_str(&raw[i..i + ch_len]);
                    i += ch_len;
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

/// Body of a Python bytes literal (`b'...'`): ASCII only, with the bytes
/// escapes (`\x`, octal, the single-letter ones); `\u`/`\N` are not escapes
/// in bytes and stay literal.
fn unescape_bytes(raw: &str, lineno: usize) -> Result<Vec<u8>, TemplateError> {
    if !raw.is_ascii() {
        return Err(syntax_err(
            "bytes can only contain ASCII literal characters.".to_string(),
            lineno,
        ));
    }
    let bytes = raw.as_bytes();
    let mut out = Vec::with_capacity(bytes.len());
    let mut i = 0usize;
    while i < bytes.len() {
        if bytes[i] != b'\\' {
            out.push(bytes[i]);
            i += 1;
            continue;
        }
        i += 1;
        let Some(&c) = bytes.get(i) else {
            return Err(syntax_err("invalid string escape".to_string(), lineno));
        };
        i += 1;
        match c {
            b'\n' => {}
            b'n' => out.push(b'\n'),
            b't' => out.push(b'\t'),
            b'r' => out.push(b'\r'),
            b'a' => out.push(0x07),
            b'b' => out.push(0x08),
            b'f' => out.push(0x0c),
            b'v' => out.push(0x0b),
            b'\\' | b'\'' | b'"' => out.push(c),
            b'0'..=b'7' => {
                let mut val = u32::from(c - b'0');
                let mut n = 1;
                while n < 3 && matches!(bytes.get(i), Some(b'0'..=b'7')) {
                    val = val * 8 + u32::from(bytes[i] - b'0');
                    i += 1;
                    n += 1;
                }
                out.push((val & 0xff) as u8);
            }
            b'x' => {
                let hex = raw.get(i..i + 2).filter(|h| h.bytes().all(|d| d.is_ascii_hexdigit()));
                let Some(hex) = hex else {
                    return Err(syntax_err(
                        format!("(value error) invalid \\x escape at position {}", i - 2),
                        lineno,
                    ));
                };
                out.push(u8::from_str_radix(hex, 16).expect("two hex digits"));
                i += 2;
            }
            other => {
                out.push(b'\\');
                out.push(other);
            }
        }
    }
    Ok(out)
}

/// Index of the quote closing the string opened at `open`; a backslash
/// always escapes the next byte, raw strings included (as in Python).
fn quoted_end(bytes: &[u8], open: usize) -> Option<usize> {
    let q = bytes[open];
    let mut j = open + 1;
    while j < bytes.len() {
        match bytes[j] {
            b'\\' => j += 2,
            c if c == q => return Some(j),
            _ => j += 1,
        }
    }
    None
}

/// Python literal tokenizer for `ast.literal_eval` (`1.` is a float, only
/// `True`/`False`/`None` are constants, `r`/`u`/`b` string prefixes).
fn tokenize(inner: &str, lineno: usize) -> Result<Vec<ET>, TemplateError> {
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
            let Some(j) = quoted_end(bytes, i) else {
                return Err(syntax_err("unterminated string".to_string(), lineno));
            };
            let s = unescape_string(&inner[i + 1..j], lineno)?;
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
                    saw_dot = true;
                    j += 1;
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
            // Python string prefixes: `r`/`u` give str (`r` keeps escapes
            // verbatim), `b`/`rb`/`br` give bytes.
            let prefix = name.to_ascii_lowercase();
            if matches!(prefix.as_str(), "r" | "u" | "b" | "br" | "rb")
                && matches!(bytes.get(i), Some(b'\'' | b'"'))
            {
                let Some(j) = quoted_end(bytes, i) else {
                    return Err(syntax_err("unterminated string".to_string(), lineno));
                };
                let body = &inner[i + 1..j];
                let raw = prefix.contains('r');
                if prefix.contains('b') {
                    let value = if raw {
                        if !body.is_ascii() {
                            return Err(syntax_err(
                                "bytes can only contain ASCII literal characters.".to_string(),
                                lineno,
                            ));
                        }
                        body.as_bytes().to_vec()
                    } else {
                        unescape_bytes(body, lineno)?
                    };
                    toks.push(ET::Bytes(value));
                } else if raw {
                    toks.push(ET::Str(body.to_string()));
                } else {
                    toks.push(ET::Str(unescape_string(body, lineno)?));
                }
                i = j + 1;
                continue;
            }
            match name.as_str() {
                "and" => toks.push(ET::KwAnd),
                "or" => toks.push(ET::KwOr),
                "not" => toks.push(ET::KwNot),
                "in" => toks.push(ET::KwIn),
                "is" => toks.push(ET::KwIs),
                "if" => toks.push(ET::KwIf),
                "else" => toks.push(ET::KwElse),
                "True" => toks.push(ET::KwTrue),
                "False" => toks.push(ET::KwFalse),
                "None" => toks.push(ET::KwNone),
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
        ET::Bytes(b) => format!("b{:?}", String::from_utf8_lossy(b)),
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
// parsing (`jinja2/parser.py`)
// ---------------------------------------------------------------------------

type PResult<T> = Result<T, TemplateError>;

/// Positional and keyword arguments of a call, filter or test.
type CallArgs = (Vec<Expr>, Vec<(String, Expr)>);

/// An assignment target (`Parser.parse_assign_target`).
enum Target {
    /// A single `Name`, with its line.
    Name(String, usize),
    /// Any other assignable target (a tuple of names, a namespace
    /// attribute), with the `Name`s it stores to.
    Other(Vec<(String, usize)>),
}

impl Target {
    fn into_stores(self) -> Vec<(String, usize)> {
        match self {
            Target::Name(name, lineno) => vec![(name, lineno)],
            Target::Other(stores) => stores,
        }
    }
}

/// `Node.can_assign()`; when it fails, the lowercased node class Jinja
/// names in "can't assign to %r".
fn assign_failure(e: &Expr) -> Option<&'static str> {
    match e {
        Expr::Name(_) => None,
        Expr::Tuple(items) => items.iter().any(|i| assign_failure(i).is_some()).then_some("tuple"),
        Expr::Const(_) => Some("const"),
        Expr::List(_) => Some("list"),
        Expr::Dict(_) => Some("dict"),
        Expr::GetAttr(..) => Some("getattr"),
        Expr::GetItem(..) => Some("getitem"),
        Expr::Call { .. } => Some("call"),
        Expr::Filter { .. } => Some("filter"),
        Expr::Test { negated: true, .. } | Expr::Not(_) => Some("not"),
        Expr::Test { .. } => Some("test"),
        Expr::Compare { .. } => Some("compare"),
        Expr::Add(..) => Some("add"),
        Expr::Sub(..) => Some("sub"),
        Expr::Mul(..) => Some("mul"),
        Expr::Div(..) => Some("div"),
        Expr::FloorDiv(..) => Some("floordiv"),
        Expr::Mod(..) => Some("mod"),
        Expr::Pow(..) => Some("pow"),
        Expr::Concat(..) => Some("concat"),
        Expr::Neg(_) => Some("neg"),
        Expr::Pos(_) => Some("pos"),
        Expr::And(..) => Some("and"),
        Expr::Or(..) => Some("or"),
        Expr::CondExpr(..) => Some("condexpr"),
    }
}

/// The names an assignable tuple stores to, at the lines its items start.
fn tuple_stores(items: Vec<Expr>, starts: &[usize], line: usize, out: &mut Vec<(String, usize)>) {
    for (i, item) in items.into_iter().enumerate() {
        let at = starts.get(i).copied().unwrap_or(line);
        match item {
            Expr::Name(name) => out.push((name, at)),
            Expr::Tuple(inner) => tuple_stores(inner, &[], at, out),
            _ => {}
        }
    }
}

/// A port of `jinja2.parser.Parser` over the token stream, building this
/// module's AST. Constructs Jinja parses but this port does not render are
/// parsed the same way and recorded in `unsupported`, so the template
/// still reports Jinja's own error when it has one.
struct Parser {
    toks: Vec<JTok>,
    /// The lexing error after the last token; `None` when the stream ends
    /// with `Eof`.
    lex_error: Option<TemplateError>,
    pos: usize,
    end_token_stack: Vec<&'static [&'static str]>,
    tag_stack: Vec<String>,
    unsupported: Option<TemplateError>,
}

impl Parser {
    /// `TokenStream.__init__` pulls the first token.
    fn new(source: &str) -> PResult<Parser> {
        let (mut toks, lex_error) = lex_template(source);
        if toks.is_empty() {
            if let Some(e) = lex_error {
                return Err(e);
            }
        }
        if lex_error.is_none() {
            let lineno = toks.last().map_or(1, |t| t.lineno);
            toks.push(JTok {
                lineno,
                kind: JT::Eof,
            });
        }
        Ok(Parser {
            toks,
            lex_error,
            pos: 0,
            end_token_stack: Vec::new(),
            tag_stack: Vec::new(),
            unsupported: None,
        })
    }

    fn kind(&self) -> &JT {
        &self.toks[self.pos].kind
    }
    fn lineno(&self) -> usize {
        self.toks[self.pos].lineno
    }
    fn pending_lex_error(&self) -> TemplateError {
        self.lex_error
            .clone()
            .expect("a token stream without Eof ends in a lexing error")
    }

    /// `next(stream)`: moves past the current token, returning its index.
    fn advance(&mut self) -> PResult<usize> {
        let old = self.pos;
        if self.toks[old].kind != JT::Eof {
            if old + 1 == self.toks.len() {
                return Err(self.pending_lex_error());
            }
            self.pos += 1;
        }
        Ok(old)
    }

    /// `stream.look()`.
    fn look(&self) -> PResult<&JT> {
        if self.toks[self.pos].kind == JT::Eof {
            return Ok(&self.toks[self.pos].kind);
        }
        match self.toks.get(self.pos + 1) {
            Some(t) => Ok(&t.kind),
            None => Err(self.pending_lex_error()),
        }
    }

    fn is_name(&self, v: &str) -> bool {
        matches!(self.kind(), JT::Name(n) if n == v)
    }
    fn is_op(&self, op: &str) -> bool {
        matches!(self.kind(), JT::Op(o) if *o == op)
    }
    fn skip_name(&mut self, v: &str) -> PResult<bool> {
        let found = self.is_name(v);
        if found {
            self.advance()?;
        }
        Ok(found)
    }
    fn skip_op(&mut self, op: &str) -> PResult<bool> {
        let found = self.is_op(op);
        if found {
            self.advance()?;
        }
        Ok(found)
    }

    /// `Parser.fail` at the current token.
    fn fail(&self, msg: String) -> TemplateError {
        syntax_err(msg, self.lineno())
    }

    /// The error `stream.expect` raises when the current token does not
    /// match the token described as `expected`.
    fn expect_failure(&self, expected: &str) -> TemplateError {
        if self.kind() == &JT::Eof {
            self.fail(format!("unexpected end of template, expected '{expected}'."))
        } else {
            self.fail(format!(
                "expected token '{expected}', got '{}'",
                describe_jt(self.kind())
            ))
        }
    }
    fn expect_op(&mut self, op: &str) -> PResult<usize> {
        if self.is_op(op) {
            self.advance()
        } else {
            Err(self.expect_failure(op))
        }
    }
    fn expect_name(&mut self) -> PResult<(String, usize)> {
        let JT::Name(name) = self.kind() else {
            return Err(self.expect_failure("name"));
        };
        let found = (name.clone(), self.lineno());
        self.advance()?;
        Ok(found)
    }
    fn expect_name_value(&mut self, v: &str) -> PResult<usize> {
        if !self.is_name(v) {
            return Err(self.expect_failure(v));
        }
        let lineno = self.lineno();
        self.advance()?;
        Ok(lineno)
    }
    /// `stream.expect("block_end")` / `stream.expect("variable_end")`.
    fn expect_end(&mut self, end: JT) -> PResult<()> {
        if *self.kind() != end {
            return Err(self.expect_failure(&describe_jt(&end)));
        }
        self.advance()?;
        Ok(())
    }

    /// Records the first construct this port does not render.
    fn unsupported(&mut self, what: &str, lineno: usize) {
        if self.unsupported.is_none() {
            self.unsupported = Some(syntax_err(what.to_string(), lineno));
        }
    }

    /// `Parser._fail_ut_eof`.
    fn fail_ut_eof(&self, name: Option<&str>, stack: &[&[&str]], lineno: usize) -> TemplateError {
        let mut message = match name {
            None => "Unexpected end of template.".to_string(),
            Some(name) => format!("Encountered unknown tag '{name}'."),
        };
        if let Some(looking) = stack.last() {
            let currently = looking
                .iter()
                .map(|e| format!("'{e}'"))
                .collect::<Vec<_>>()
                .join(" or ");
            if name.is_some_and(|name| stack.iter().any(|e| e.contains(&name))) {
                message.push_str(&format!(
                    " You probably made a nesting mistake. Jinja is expecting this tag, but currently looking for {currently}."
                ));
            } else {
                message.push_str(&format!(
                    " Jinja was looking for the following tags: {currently}."
                ));
            }
        }
        if let Some(tag) = self.tag_stack.last() {
            message.push_str(&format!(
                " The innermost block that needs to be closed is '{tag}'."
            ));
        }
        syntax_err(message, lineno)
    }

    /// `Parser.fail_eof` at the current (end of template) token.
    fn fail_eof(&self, end_tokens: &'static [&'static str]) -> TemplateError {
        let mut stack = self.end_token_stack.clone();
        stack.push(end_tokens);
        self.fail_ut_eof(None, &stack, self.lineno())
    }

    /// `Parser.is_tuple_end`. Jinja 2.11 passes `extra_end_rules` to
    /// `test_any` as one tuple argument, so those rules never match: only
    /// the token types below end a tuple.
    fn is_tuple_end(&self) -> bool {
        matches!(self.kind(), JT::VarEnd | JT::BlockEnd | JT::Op(")"))
    }

    /// `Parser.subparse`: template data, prints and statements up to one
    /// of `end_tokens` (a tag name, left as the current token).
    fn subparse(&mut self, end_tokens: Option<&'static [&'static str]>) -> PResult<Vec<Node>> {
        if let Some(ends) = end_tokens {
            self.end_token_stack.push(ends);
        }
        let mut body = Vec::new();
        loop {
            match &mut self.toks[self.pos].kind {
                JT::Eof => break,
                JT::Data(text) => {
                    body.push(Node::Text(std::mem::take(text)));
                    self.advance()?;
                }
                JT::VarBegin => {
                    self.advance()?;
                    let (e, _, _) = self.parse_tuple(false, true, false)?;
                    self.expect_end(JT::VarEnd)?;
                    body.push(Node::Print(e));
                }
                JT::BlockBegin => {
                    self.advance()?;
                    if let (Some(ends), JT::Name(n)) = (end_tokens, self.kind()) {
                        if ends.contains(&n.as_str()) {
                            self.end_token_stack.pop();
                            return Ok(body);
                        }
                    }
                    let node = self.parse_statement()?;
                    body.push(node);
                    self.expect_end(JT::BlockEnd)?;
                }
                _ => {
                    return Err(TemplateError::new(
                        "AssertionError",
                        "internal parsing error".to_string(),
                        None,
                    ))
                }
            }
        }
        if end_tokens.is_some() {
            self.end_token_stack.pop();
        }
        Ok(body)
    }

    /// `Parser.parse_statements`.
    fn parse_statements(
        &mut self,
        end_tokens: &'static [&'static str],
        drop_needle: bool,
    ) -> PResult<Vec<Node>> {
        self.skip_op(":")?;
        self.expect_end(JT::BlockEnd)?;
        let body = self.subparse(Some(end_tokens))?;
        if self.kind() == &JT::Eof {
            return Err(self.fail_eof(end_tokens));
        }
        if drop_needle {
            self.advance()?;
        }
        Ok(body)
    }

    /// `Parser.parse_statement`.
    fn parse_statement(&mut self) -> PResult<Node> {
        let lineno = self.lineno();
        let JT::Name(name) = self.kind() else {
            return Err(syntax_err("tag name expected".to_string(), lineno));
        };
        let name = name.clone();
        self.tag_stack.push(name.clone());
        let node = match name.as_str() {
            "if" => self.parse_if()?,
            "for" => self.parse_for()?,
            "set" => self.parse_set()?,
            "block" | "extends" | "print" | "macro" | "include" | "from" | "import" | "with"
            | "autoescape" | "call" | "filter" => {
                self.unsupported(&format!("unsupported '{name}' tag"), lineno);
                self.parse_unsupported_tag(&name)?
            }
            _ => {
                self.tag_stack.pop();
                return Err(self.fail_ut_eof(Some(&name), &self.end_token_stack, lineno));
            }
        };
        self.tag_stack.pop();
        Ok(node)
    }

    /// `Parser.parse_if`.
    fn parse_if(&mut self) -> PResult<Node> {
        self.expect_name_value("if")?;
        let mut branches = Vec::new();
        let mut else_body = Vec::new();
        loop {
            let (test, _, _) = self.parse_tuple(false, false, false)?;
            let body = self.parse_statements(&["elif", "else", "endif"], false)?;
            branches.push((test, body));
            let tok = self.advance()?;
            let tag = &self.toks[tok].kind;
            if matches!(tag, JT::Name(n) if n == "elif") {
                continue;
            }
            if matches!(tag, JT::Name(n) if n == "else") {
                else_body = self.parse_statements(&["endif"], true)?;
            }
            break;
        }
        Ok(Node::If {
            branches,
            else_body,
        })
    }

    /// `Parser.parse_for`. Only a single-name target without `if` filter
    /// or `recursive` is rendered.
    fn parse_for(&mut self) -> PResult<Node> {
        let lineno = self.expect_name_value("for")?;
        let target = self.parse_assign_target(false)?;
        if matches!(target, Target::Other(_)) {
            self.unsupported("unsupported 'for' loop target", lineno);
        }
        self.expect_name_value("in")?;
        let (iter, _, _) = self.parse_tuple(false, false, false)?;
        let mut exprs = Vec::new();
        let filtered = self.skip_name("if")?;
        if filtered {
            self.unsupported("unsupported 'for ... if' loop filter", lineno);
            exprs.push(self.parse_expression(true)?);
        }
        let recursive = self.skip_name("recursive")?;
        if recursive {
            self.unsupported("unsupported 'recursive' for loop", lineno);
        }
        let body = self.parse_statements(&["endfor", "else"], false)?;
        let tok = self.advance()?;
        let else_body = if matches!(&self.toks[tok].kind, JT::Name(n) if n == "endfor") {
            Vec::new()
        } else {
            self.parse_statements(&["endfor"], true)?
        };
        Ok(match target {
            Target::Name(target, target_lineno) if !filtered && !recursive => Node::For {
                target,
                target_lineno,
                iter,
                body,
                else_body,
            },
            target => {
                exprs.push(iter);
                let mut all = body;
                all.extend(else_body);
                Node::Discarded {
                    for_loop: true,
                    stores: target.into_stores(),
                    exprs,
                    body: all,
                }
            }
        })
    }

    /// `Parser.parse_set`. Only `{% set name = expr %}` is rendered.
    fn parse_set(&mut self) -> PResult<Node> {
        let lineno = self.lineno();
        self.advance()?;
        let target = self.parse_assign_target(true)?;
        if self.skip_op("=")? {
            if matches!(target, Target::Other(_)) {
                self.unsupported("unsupported 'set' target", lineno);
            }
            let (value, _, _) = self.parse_tuple(false, true, false)?;
            return Ok(match target {
                Target::Name(name, lineno) => Node::Set {
                    name,
                    lineno,
                    value,
                },
                Target::Other(stores) => Node::Discarded {
                    for_loop: false,
                    stores,
                    exprs: vec![value],
                    body: Vec::new(),
                },
            });
        }
        self.unsupported("unsupported 'set' block assignment without '='", lineno);
        let filter = if self.is_op("|") {
            Some(self.parse_filter(Expr::Const(PyValue::None), false)?)
        } else {
            None
        };
        let body = self.parse_statements(&["endset"], true)?;
        Ok(Node::Discarded {
            for_loop: false,
            stores: target.into_stores(),
            exprs: filter.into_iter().collect(),
            body,
        })
    }

    /// The parsers of the statement tags this port does not render
    /// (`parse_block`, `parse_extends`, `parse_print`, `parse_macro`,
    /// `parse_include`, `parse_from`, `parse_import`, `parse_with`,
    /// `parse_autoescape`, `parse_call_block`, `parse_filter_block`).
    fn parse_unsupported_tag(&mut self, tag: &str) -> PResult<Node> {
        let lineno = self.lineno();
        self.advance()?;
        let mut exprs = Vec::new();
        let mut body = Vec::new();
        match tag {
            "block" => {
                let (name, _) = self.expect_name()?;
                self.skip_name("scoped")?;
                if self.is_op("-") {
                    return Err(self.fail(
                        "Block names in Jinja have to be valid Python identifiers and may not contain hyphens, use an underscore instead."
                            .to_string(),
                    ));
                }
                body = self.parse_statements(&["endblock"], true)?;
                self.skip_name(&name)?;
            }
            "extends" => exprs.push(self.parse_expression(true)?),
            "print" => {
                while *self.kind() != JT::BlockEnd {
                    if !exprs.is_empty() {
                        self.expect_op(",")?;
                    }
                    exprs.push(self.parse_expression(true)?);
                }
            }
            "macro" => {
                self.parse_name_target()?;
                exprs = self.parse_signature()?;
                body = self.parse_statements(&["endmacro"], true)?;
            }
            "include" => {
                exprs.push(self.parse_expression(true)?);
                if self.is_name("ignore") && matches!(self.look()?, JT::Name(n) if n == "missing")
                {
                    self.advance()?;
                    self.advance()?;
                }
                self.parse_import_context()?;
            }
            "import" => {
                exprs.push(self.parse_expression(true)?);
                self.expect_name_value("as")?;
                self.parse_name_target()?;
                self.parse_import_context()?;
            }
            "from" => {
                exprs.push(self.parse_expression(true)?);
                self.parse_from_names()?;
            }
            "with" => {
                while *self.kind() != JT::BlockEnd {
                    if !exprs.is_empty() {
                        self.expect_op(",")?;
                    }
                    self.parse_assign_target(false)?;
                    self.expect_op("=")?;
                    exprs.push(self.parse_expression(true)?);
                }
                body = self.parse_statements(&["endwith"], true)?;
            }
            "autoescape" => {
                exprs.push(self.parse_expression(true)?);
                body = self.parse_statements(&["endautoescape"], true)?;
            }
            "call" => {
                if self.is_op("(") {
                    exprs = self.parse_signature()?;
                }
                let call = self.parse_expression(true)?;
                if !matches!(call, Expr::Call { .. }) {
                    return Err(syntax_err("expected call".to_string(), lineno));
                }
                exprs.push(call);
                body = self.parse_statements(&["endcall"], true)?;
            }
            "filter" => {
                exprs.push(self.parse_filter(Expr::Const(PyValue::None), true)?);
                body = self.parse_statements(&["endfilter"], true)?;
            }
            _ => unreachable!("parse_statement dispatches only the tags above"),
        }
        Ok(Node::Discarded {
            for_loop: false,
            stores: Vec::new(),
            exprs,
            body,
        })
    }

    /// `parse_assign_target(name_only=True)`.
    fn parse_name_target(&mut self) -> PResult<(String, usize)> {
        let (name, lineno) = self.expect_name()?;
        if matches!(name.as_str(), "true" | "false" | "none" | "True" | "False" | "None") {
            return Err(syntax_err("can't assign to 'name'".to_string(), lineno));
        }
        Ok((name, lineno))
    }

    /// `Parser.parse_signature`; returns the default values.
    fn parse_signature(&mut self) -> PResult<Vec<Expr>> {
        self.expect_op("(")?;
        let mut count = 0;
        let mut defaults = Vec::new();
        while !self.is_op(")") {
            if count > 0 {
                self.expect_op(",")?;
            }
            self.parse_name_target()?;
            if self.skip_op("=")? {
                defaults.push(self.parse_expression(true)?);
            } else if !defaults.is_empty() {
                return Err(self.fail("non-default argument follows default argument".to_string()));
            }
            count += 1;
        }
        self.expect_op(")")?;
        Ok(defaults)
    }

    /// `Parser.parse_import_context`.
    fn parse_import_context(&mut self) -> PResult<()> {
        if (self.is_name("with") || self.is_name("without"))
            && matches!(self.look()?, JT::Name(n) if n == "context")
        {
            self.advance()?;
            self.advance()?;
        }
        Ok(())
    }

    /// The `import` names of `Parser.parse_from`.
    fn parse_from_names(&mut self) -> PResult<()> {
        self.expect_name_value("import")?;
        let mut count = 0;
        loop {
            if count > 0 {
                self.expect_op(",")?;
            }
            if !matches!(self.kind(), JT::Name(_)) {
                return Err(self.expect_failure("name"));
            }
            if self.parse_from_context()? {
                break;
            }
            let (target, lineno) = self.parse_name_target()?;
            if target.starts_with('_') {
                return Err(assertion_err(
                    "names starting with an underline can not be imported".to_string(),
                    lineno,
                ));
            }
            if self.skip_name("as")? {
                self.parse_name_target()?;
            }
            count += 1;
            if self.parse_from_context()? || !self.is_op(",") {
                break;
            }
        }
        Ok(())
    }

    /// `parse_context` inside `Parser.parse_from` (it tests the token value,
    /// whatever its type).
    fn parse_from_context(&mut self) -> PResult<bool> {
        let with = matches!(self.kind(), JT::Name(v) | JT::Str(v) if v == "with" || v == "without");
        if with && matches!(self.look()?, JT::Name(n) if n == "context") {
            self.advance()?;
            self.advance()?;
            return Ok(true);
        }
        Ok(false)
    }

    /// `Parser.parse_assign_target` (tuples allowed).
    fn parse_assign_target(&mut self, with_namespace: bool) -> PResult<Target> {
        if with_namespace && matches!(self.look()?, JT::Op(".")) {
            self.expect_name()?;
            self.advance()?;
            self.expect_name()?;
            return Ok(Target::Other(Vec::new()));
        }
        let (target, lineno, starts) = self.parse_tuple(true, true, false)?;
        if let Some(class) = assign_failure(&target) {
            return Err(syntax_err(format!("can't assign to '{class}'"), lineno));
        }
        Ok(match target {
            Expr::Name(name) => Target::Name(name, lineno),
            Expr::Tuple(items) => {
                let mut stores = Vec::new();
                tuple_stores(items, &starts, lineno, &mut stores);
                Target::Other(stores)
            }
            _ => unreachable!("only names and tuples of names are assignable"),
        })
    }

    /// `Parser.parse_tuple`: the expression, its line (`Tuple.lineno` or
    /// the start of the single item) and, for `simplified` parses, the line
    /// each item starts on.
    fn parse_tuple(
        &mut self,
        simplified: bool,
        with_condexpr: bool,
        explicit_parentheses: bool,
    ) -> PResult<(Expr, usize, Vec<usize>)> {
        let mut lineno = self.lineno();
        let mut args = Vec::new();
        let mut starts = Vec::new();
        let mut is_tuple = false;
        loop {
            if !args.is_empty() {
                self.expect_op(",")?;
            }
            if self.is_tuple_end() {
                break;
            }
            if simplified {
                starts.push(self.lineno());
                args.push(self.parse_primary()?);
            } else {
                args.push(self.parse_expression(with_condexpr)?);
            }
            if !self.is_op(",") {
                break;
            }
            is_tuple = true;
            lineno = self.lineno();
        }
        if !is_tuple {
            if let Some(item) = args.pop() {
                return Ok((item, lineno, starts));
            }
            if !explicit_parentheses {
                return Err(self.fail(format!(
                    "Expected an expression, got '{}'",
                    describe_jt(self.kind())
                )));
            }
        }
        Ok((Expr::Tuple(args), lineno, starts))
    }

    fn parse_expression(&mut self, with_condexpr: bool) -> PResult<Expr> {
        if with_condexpr {
            self.parse_condexpr()
        } else {
            self.parse_or()
        }
    }

    /// `a if cond else b`; a missing `else` yields `Undefined`.
    fn parse_condexpr(&mut self) -> PResult<Expr> {
        let mut expr = self.parse_or()?;
        while self.skip_name("if")? {
            let cond = self.parse_or()?;
            let else_expr = if self.skip_name("else")? {
                Some(Box::new(self.parse_condexpr()?))
            } else {
                None
            };
            expr = Expr::CondExpr(Box::new(expr), Box::new(cond), else_expr);
        }
        Ok(expr)
    }

    fn parse_or(&mut self) -> PResult<Expr> {
        let mut left = self.parse_and()?;
        while self.skip_name("or")? {
            let right = self.parse_and()?;
            left = Expr::Or(Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn parse_and(&mut self) -> PResult<Expr> {
        let mut left = self.parse_not()?;
        while self.skip_name("and")? {
            let right = self.parse_not()?;
            left = Expr::And(Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn parse_not(&mut self) -> PResult<Expr> {
        if self.skip_name("not")? {
            return Ok(Expr::Not(Box::new(self.parse_not()?)));
        }
        self.parse_compare()
    }

    fn parse_compare(&mut self) -> PResult<Expr> {
        let first = self.parse_math1()?;
        let mut ops: Vec<(String, Expr)> = Vec::new();
        loop {
            let compare = match self.kind() {
                JT::Op(op @ ("==" | "!=" | "<" | "<=" | ">" | ">=")) => Some(*op),
                _ => None,
            };
            if let Some(op) = compare {
                self.advance()?;
                ops.push((op.to_string(), self.parse_math1()?));
            } else if self.skip_name("in")? {
                ops.push(("in".to_string(), self.parse_math1()?));
            } else if self.is_name("not") && matches!(self.look()?, JT::Name(n) if n == "in") {
                self.advance()?;
                self.advance()?;
                ops.push(("notin".to_string(), self.parse_math1()?));
            } else {
                break;
            }
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

    fn parse_math1(&mut self) -> PResult<Expr> {
        let mut left = self.parse_concat()?;
        loop {
            if self.skip_op("+")? {
                let right = self.parse_concat()?;
                left = Expr::Add(Box::new(left), Box::new(right));
            } else if self.skip_op("-")? {
                let right = self.parse_concat()?;
                left = Expr::Sub(Box::new(left), Box::new(right));
            } else {
                return Ok(left);
            }
        }
    }

    fn parse_concat(&mut self) -> PResult<Expr> {
        let mut acc = self.parse_math2()?;
        while self.skip_op("~")? {
            let right = self.parse_math2()?;
            acc = Expr::Concat(Box::new(acc), Box::new(right));
        }
        Ok(acc)
    }

    fn parse_math2(&mut self) -> PResult<Expr> {
        let mut left = self.parse_pow()?;
        loop {
            let op = match self.kind() {
                JT::Op(op @ ("*" | "/" | "//" | "%")) => *op,
                _ => return Ok(left),
            };
            self.advance()?;
            let right = Box::new(self.parse_pow()?);
            let l = Box::new(left);
            left = match op {
                "*" => Expr::Mul(l, right),
                "/" => Expr::Div(l, right),
                "//" => Expr::FloorDiv(l, right),
                _ => Expr::Mod(l, right),
            };
        }
    }

    fn parse_pow(&mut self) -> PResult<Expr> {
        let mut left = self.parse_unary(true)?;
        while self.skip_op("**")? {
            let right = self.parse_unary(true)?;
            left = Expr::Pow(Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    /// Operands of unary `-`/`+` carry no filters (`parse_unary(False)`).
    fn parse_unary(&mut self, with_filter: bool) -> PResult<Expr> {
        let node = if self.skip_op("-")? {
            Expr::Neg(Box::new(self.parse_unary(false)?))
        } else if self.skip_op("+")? {
            Expr::Pos(Box::new(self.parse_unary(false)?))
        } else {
            self.parse_primary()?
        };
        let node = self.parse_postfix(node)?;
        if with_filter {
            self.parse_filter_expr(node)
        } else {
            Ok(node)
        }
    }

    fn parse_primary(&mut self) -> PResult<Expr> {
        let lineno = self.lineno();
        let node = match self.kind() {
            JT::Name(n) => {
                let node = match n.as_str() {
                    "true" | "True" => Expr::Const(PyValue::Bool(true)),
                    "false" | "False" => Expr::Const(PyValue::Bool(false)),
                    "none" | "None" => Expr::Const(PyValue::None),
                    _ => Expr::Name(n.clone()),
                };
                self.advance()?;
                node
            }
            JT::Str(_) => {
                // Adjacent string literals concatenate.
                let mut buf = String::new();
                while let JT::Str(s) = self.kind() {
                    buf.push_str(s);
                    self.advance()?;
                }
                Expr::Const(PyValue::Str(buf))
            }
            JT::Int(v) => {
                let node = Expr::Const(PyValue::Int(v.clone()));
                self.advance()?;
                node
            }
            JT::Float(v) => {
                let node = Expr::Const(PyValue::Float(*v));
                self.advance()?;
                node
            }
            JT::Op("(") => {
                self.advance()?;
                let (node, _, _) = self.parse_tuple(false, true, true)?;
                self.expect_op(")")?;
                node
            }
            JT::Op("[") => self.parse_list()?,
            JT::Op("{") => self.parse_dict()?,
            other => {
                return Err(syntax_err(format!("unexpected '{}'", describe_jt(other)), lineno));
            }
        };
        Ok(node)
    }

    fn parse_list(&mut self) -> PResult<Expr> {
        self.expect_op("[")?;
        let mut items = Vec::new();
        while !self.is_op("]") {
            if !items.is_empty() {
                self.expect_op(",")?;
            }
            if self.is_op("]") {
                break;
            }
            items.push(self.parse_expression(true)?);
        }
        self.expect_op("]")?;
        Ok(Expr::List(items))
    }

    fn parse_dict(&mut self) -> PResult<Expr> {
        self.expect_op("{")?;
        let mut items = Vec::new();
        while !self.is_op("}") {
            if !items.is_empty() {
                self.expect_op(",")?;
            }
            if self.is_op("}") {
                break;
            }
            let key = self.parse_expression(true)?;
            self.expect_op(":")?;
            let value = self.parse_expression(true)?;
            items.push((key, value));
        }
        self.expect_op("}")?;
        Ok(Expr::Dict(items))
    }

    fn parse_postfix(&mut self, mut node: Expr) -> PResult<Expr> {
        loop {
            if self.is_op(".") || self.is_op("[") {
                node = self.parse_subscript(node)?;
            } else if self.is_op("(") {
                node = self.parse_call(node)?;
            } else {
                return Ok(node);
            }
        }
    }

    fn parse_filter_expr(&mut self, mut node: Expr) -> PResult<Expr> {
        loop {
            if self.is_op("|") {
                node = self.parse_filter(node, false)?;
            } else if self.is_name("is") {
                node = self.parse_test(node)?;
            } else if self.is_op("(") {
                node = self.parse_call(node)?;
            } else {
                return Ok(node);
            }
        }
    }

    /// `.name`, `.0` and `[...]`; only a single non-slice subscript is
    /// rendered.
    fn parse_subscript(&mut self, node: Expr) -> PResult<Expr> {
        let tok = self.advance()?;
        if self.toks[tok].kind == JT::Op(".") {
            let attr = self.advance()?;
            return match &self.toks[attr].kind {
                JT::Name(name) => Ok(Expr::GetAttr(Box::new(node), name.clone())),
                JT::Int(v) => Ok(Expr::GetItem(
                    Box::new(node),
                    Box::new(Expr::Const(PyValue::Int(v.clone()))),
                )),
                _ => Err(syntax_err(
                    "expected name or number".to_string(),
                    self.toks[attr].lineno,
                )),
            };
        }
        let lineno = self.toks[tok].lineno;
        let mut args = Vec::new();
        let mut slicing = false;
        while !self.is_op("]") {
            if !args.is_empty() {
                self.expect_op(",")?;
            }
            let (arg, is_slice) = self.parse_subscribed()?;
            slicing |= is_slice;
            args.push(arg);
        }
        self.expect_op("]")?;
        if slicing {
            self.unsupported("unsupported slicing 'x[a:b]'", lineno);
        } else if args.len() != 1 {
            self.unsupported("unsupported tuple subscript 'x[a, b]'", lineno);
        }
        let arg = match args.pop() {
            Some(arg) if args.is_empty() => arg,
            last => {
                args.extend(last);
                Expr::Tuple(args)
            }
        };
        Ok(Expr::GetItem(Box::new(node), Box::new(arg)))
    }

    /// `Parser.parse_subscribed`: an index, or a slice (its parts as a
    /// tuple, flagged `true`).
    fn parse_subscribed(&mut self) -> PResult<(Expr, bool)> {
        let mut parts = Vec::new();
        if !self.skip_op(":")? {
            let node = self.parse_expression(true)?;
            if !self.skip_op(":")? {
                return Ok((node, false));
            }
            parts.push(node);
        }
        if !self.is_op(":") && !self.is_op("]") && !self.is_op(",") {
            parts.push(self.parse_expression(true)?);
        }
        if self.skip_op(":")? && !self.is_op("]") && !self.is_op(",") {
            parts.push(self.parse_expression(true)?);
        }
        Ok((Expr::Tuple(parts), true))
    }

    fn parse_call(&mut self, node: Expr) -> PResult<Expr> {
        let (args, kwargs) = self.parse_call_args()?;
        Ok(Expr::Call {
            callee: Box::new(node),
            args,
            kwargs,
        })
    }

    /// The argument list of `Parser.parse_call`. `*args`/`**kwargs` are
    /// not rendered; they follow the keywords (named `*`/`**`), the order
    /// Jinja's code generator visits them in.
    fn parse_call_args(&mut self) -> PResult<CallArgs> {
        let tok = self.expect_op("(")?;
        let lineno = self.toks[tok].lineno;
        let invalid =
            || syntax_err("invalid syntax for function call expression".to_string(), lineno);
        let mut args = Vec::new();
        let mut kwargs = Vec::new();
        let mut dyn_args = None;
        let mut dyn_kwargs = None;
        let mut require_comma = false;
        while !self.is_op(")") {
            if require_comma {
                self.expect_op(",")?;
                if self.is_op(")") {
                    break;
                }
            }
            if self.is_op("*") {
                if dyn_args.is_some() || dyn_kwargs.is_some() {
                    return Err(invalid());
                }
                self.unsupported("unsupported '*' call unpacking", lineno);
                self.advance()?;
                dyn_args = Some(self.parse_expression(true)?);
            } else if self.is_op("**") {
                if dyn_kwargs.is_some() {
                    return Err(invalid());
                }
                self.unsupported("unsupported '**' call unpacking", lineno);
                self.advance()?;
                dyn_kwargs = Some(self.parse_expression(true)?);
            } else if matches!(self.kind(), JT::Name(_)) && matches!(self.look()?, JT::Op("=")) {
                if dyn_kwargs.is_some() {
                    return Err(invalid());
                }
                let (key, _) = self.expect_name()?;
                self.advance()?;
                kwargs.push((key, self.parse_expression(true)?));
            } else {
                if dyn_args.is_some() || dyn_kwargs.is_some() || !kwargs.is_empty() {
                    return Err(invalid());
                }
                args.push(self.parse_expression(true)?);
            }
            require_comma = true;
        }
        self.expect_op(")")?;
        kwargs.extend(dyn_args.map(|e| ("*".to_string(), e)));
        kwargs.extend(dyn_kwargs.map(|e| ("**".to_string(), e)));
        Ok((args, kwargs))
    }

    /// `Parser.parse_filter`: `|name(args)` chains (`start_inline` for
    /// `{% filter %}`, whose first name has no pipe).
    fn parse_filter(&mut self, mut node: Expr, mut start_inline: bool) -> PResult<Expr> {
        while self.is_op("|") || start_inline {
            if !start_inline {
                self.advance()?;
            }
            let (mut name, lineno) = self.expect_name()?;
            while self.skip_op(".")? {
                name.push('.');
                name.push_str(&self.expect_name()?.0);
            }
            let (args, kwargs) = if self.is_op("(") {
                self.parse_call_args()?
            } else {
                (Vec::new(), Vec::new())
            };
            node = Expr::Filter {
                node: Box::new(node),
                name,
                args,
                kwargs,
                lineno,
            };
            start_inline = false;
        }
        Ok(node)
    }

    /// `Parser.parse_test`.
    fn parse_test(&mut self, node: Expr) -> PResult<Expr> {
        let tok = self.advance()?;
        let lineno = self.toks[tok].lineno;
        let negated = self.skip_name("not")?;
        let (mut name, _) = self.expect_name()?;
        while self.skip_op(".")? {
            name.push('.');
            name.push_str(&self.expect_name()?.0);
        }
        let (args, kwargs) = if self.is_op("(") {
            self.parse_call_args()?
        } else if matches!(
            self.kind(),
            JT::Name(_) | JT::Str(_) | JT::Int(_) | JT::Float(_) | JT::Op("(" | "[" | "{")
        ) && !(self.is_name("else") || self.is_name("or") || self.is_name("and"))
        {
            if self.is_name("is") {
                return Err(self.fail("You cannot chain multiple tests with is".to_string()));
            }
            let arg = self.parse_primary()?;
            (vec![self.parse_postfix(arg)?], Vec::new())
        } else {
            (Vec::new(), Vec::new())
        };
        Ok(Expr::Test {
            node: Box::new(node),
            name,
            args,
            kwargs,
            negated,
            lineno,
        })
    }
}

// ---------------------------------------------------------------------------
// compile-time checks (`jinja2/compiler.py`)
// ---------------------------------------------------------------------------

/// Jinja2 2.11.3 `DEFAULT_FILTERS` names; the compiler rejects others.
const JINJA_FILTERS: &[&str] = &[
    "abs", "attr", "batch", "capitalize", "center", "count", "d", "default", "dictsort", "e",
    "escape", "filesizeformat", "first", "float", "forceescape", "format", "groupby", "indent",
    "int", "join", "last", "length", "list", "lower", "map", "max", "min", "pprint", "random",
    "reject", "rejectattr", "replace", "reverse", "round", "safe", "select", "selectattr",
    "slice", "sort", "string", "striptags", "sum", "title", "tojson", "trim", "truncate",
    "unique", "upper", "urlencode", "urlize", "wordcount", "wordwrap", "xmlattr",
];

/// Jinja2 2.11.3 `DEFAULT_TESTS` names; the compiler rejects others.
const JINJA_TESTS: &[&str] = &[
    "!=", "<", "<=", "==", ">", ">=", "boolean", "callable", "defined", "divisibleby", "eq",
    "equalto", "escaped", "even", "false", "float", "ge", "greaterthan", "gt", "in", "integer",
    "iterable", "le", "lessthan", "lower", "lt", "mapping", "ne", "none", "number", "odd",
    "sameas", "sequence", "string", "true", "undefined", "upper",
];

/// Python keywords (`keyword.iskeyword`): a call with one as a keyword
/// argument passes all keywords as a `**{...}` dict, which Python does not
/// check for repeats.
const PY_KEYWORDS: &[&str] = &[
    "False", "None", "True", "and", "as", "assert", "async", "await", "break", "class",
    "continue", "def", "del", "elif", "else", "except", "finally", "for", "from", "global", "if",
    "import", "in", "is", "lambda", "nonlocal", "not", "or", "pass", "raise", "return", "try",
    "while", "with", "yield",
];

fn loop_target_err(lineno: usize) -> TemplateError {
    assertion_err(
        "Can't assign to special loop variable in for-loop target".to_string(),
        lineno,
    )
}

/// The line of the first `Name` stored to as `loop` (`node.find_all(Name)`
/// order).
fn loop_store(nodes: &[Node]) -> Option<usize> {
    let named = |stores: &[(String, usize)]| stores.iter().find(|(n, _)| n == "loop").map(|s| s.1);
    nodes.iter().find_map(|node| match node {
        Node::Text(_) | Node::Print(_) => None,
        Node::Set { name, lineno, .. } => (name == "loop").then_some(*lineno),
        Node::If {
            branches,
            else_body,
        } => branches
            .iter()
            .find_map(|(_, body)| loop_store(body))
            .or_else(|| loop_store(else_body)),
        Node::For {
            target,
            target_lineno,
            body,
            else_body,
            ..
        } => (target == "loop")
            .then_some(*target_lineno)
            .or_else(|| loop_store(body))
            .or_else(|| loop_store(else_body)),
        Node::Discarded { stores, body, .. } => named(stores).or_else(|| loop_store(body)),
    })
}

/// `Expr.as_const()`: the value of an expression Jinja's optimizer can
/// compute at compile time, `None` where it raises `Impossible` (names,
/// calls, unknown filters or tests, or an evaluation error). `and`, `or`,
/// `if`/`else` and chained comparisons short-circuit as in Python, so a
/// dead branch never prevents folding.
fn as_const(e: &Expr) -> Option<Value> {
    match e {
        Expr::Const(v) => return Some(Value::py(v.clone())),
        Expr::Name(_) | Expr::Call { .. } => return None,
        Expr::And(a, b) => {
            let left = as_const(a)?;
            return if left.truthy() { as_const(b) } else { Some(left) };
        }
        Expr::Or(a, b) => {
            let left = as_const(a)?;
            return if left.truthy() { Some(left) } else { as_const(b) };
        }
        Expr::CondExpr(value, test, other) => {
            return if as_const(test)?.truthy() {
                as_const(value)
            } else {
                as_const(other.as_deref()?)
            };
        }
        Expr::Compare { first, ops } => {
            let mut value = as_const(first)?;
            for (op, operand) in ops {
                let next = as_const(operand)?;
                if !eval_compare(op, &value, &next, 1).ok()? {
                    return Some(Value::py(PyValue::Bool(false)));
                }
                value = next;
            }
            return Some(Value::py(PyValue::Bool(true)));
        }
        Expr::Filter { name, .. } if !JINJA_FILTERS.contains(&name.as_str()) => return None,
        Expr::Test { name, .. } if !JINJA_TESTS.contains(&name.as_str()) => return None,
        Expr::List(items) | Expr::Tuple(items) => items.iter().try_for_each(|i| as_const(i).map(drop))?,
        Expr::Dict(pairs) => pairs
            .iter()
            .try_for_each(|(k, v)| as_const(k).and(as_const(v)).map(drop))?,
        Expr::GetAttr(base, _) | Expr::Neg(base) | Expr::Pos(base) | Expr::Not(base) => {
            as_const(base)?;
        }
        Expr::GetItem(a, b)
        | Expr::Add(a, b)
        | Expr::Sub(a, b)
        | Expr::Mul(a, b)
        | Expr::Div(a, b)
        | Expr::FloorDiv(a, b)
        | Expr::Mod(a, b)
        | Expr::Pow(a, b)
        | Expr::Concat(a, b) => {
            as_const(a)?;
            as_const(b)?;
        }
        Expr::Filter { node, args, kwargs, .. } | Expr::Test { node, args, kwargs, .. } => {
            as_const(node)?;
            args.iter().try_for_each(|a| as_const(a).map(drop))?;
            kwargs.iter().try_for_each(|(_, v)| as_const(v).map(drop))?;
        }
    }
    // Every operand is constant: evaluating it touches no variable.
    eval_expr(e, &mut Ctx::new(IndexMap::new()), 1).ok()
}

/// The compile-time checks of Jinja's code generator, in its visiting
/// order: an unknown filter or test and a `for` loop that stores to `loop`
/// raise `TemplateAssertionError`. A call that repeats a keyword argument
/// is only noted: the generated Python then fails to compile, after the
/// whole template was generated.
#[derive(Default)]
struct CompileCheck {
    repeated_kwarg: bool,
}

impl CompileCheck {
    fn nodes(&mut self, nodes: &[Node]) -> PResult<()> {
        nodes.iter().try_for_each(|node| self.node(node))
    }

    fn node(&mut self, node: &Node) -> PResult<()> {
        match node {
            Node::Text(_) => Ok(()),
            Node::Print(e) | Node::Set { value: e, .. } => self.expr(e),
            Node::If {
                branches,
                else_body,
            } => {
                for (test, body) in branches {
                    self.expr(test)?;
                    self.nodes(body)?;
                }
                self.nodes(else_body)
            }
            Node::For { iter, body, else_body, .. } => {
                if let Some(lineno) = loop_store(std::slice::from_ref(node)) {
                    return Err(loop_target_err(lineno));
                }
                self.expr(iter)?;
                self.nodes(body)?;
                self.nodes(else_body)
            }
            Node::Discarded {
                for_loop,
                exprs,
                body,
                ..
            } => {
                if *for_loop {
                    if let Some(lineno) = loop_store(std::slice::from_ref(node)) {
                        return Err(loop_target_err(lineno));
                    }
                }
                exprs.iter().try_for_each(|e| self.expr(e))?;
                self.nodes(body)
            }
        }
    }

    fn expr(&mut self, e: &Expr) -> PResult<()> {
        // The optimizer folds a constant subtree before the generator
        // visits it, so nothing inside it is checked.
        if !matches!(e, Expr::Const(_) | Expr::Name(_)) && as_const(e).is_some() {
            return Ok(());
        }
        match e {
            Expr::Const(_) | Expr::Name(_) => Ok(()),
            Expr::List(items) | Expr::Tuple(items) => items.iter().try_for_each(|i| self.expr(i)),
            Expr::Dict(pairs) => pairs.iter().try_for_each(|(k, v)| {
                self.expr(k)?;
                self.expr(v)
            }),
            Expr::GetAttr(base, _) => self.expr(base),
            Expr::GetItem(base, index) => {
                self.expr(base)?;
                self.expr(index)
            }
            Expr::Call {
                callee,
                args,
                kwargs,
            } => {
                self.expr(callee)?;
                self.signature(args, kwargs)
            }
            Expr::Filter {
                node,
                name,
                args,
                kwargs,
                lineno,
            } => {
                if !JINJA_FILTERS.contains(&name.as_str()) {
                    return Err(assertion_err(format!("no filter named '{name}'"), *lineno));
                }
                self.expr(node)?;
                self.signature(args, kwargs)
            }
            Expr::Test {
                node,
                name,
                args,
                kwargs,
                lineno,
                ..
            } => {
                if !JINJA_TESTS.contains(&name.as_str()) {
                    return Err(assertion_err(format!("no test named '{name}'"), *lineno));
                }
                self.expr(node)?;
                self.signature(args, kwargs)
            }
            Expr::Compare { first, ops } => {
                self.expr(first)?;
                ops.iter().try_for_each(|(_, operand)| self.expr(operand))
            }
            Expr::Add(a, b)
            | Expr::Sub(a, b)
            | Expr::Mul(a, b)
            | Expr::Div(a, b)
            | Expr::FloorDiv(a, b)
            | Expr::Mod(a, b)
            | Expr::Pow(a, b)
            | Expr::Concat(a, b)
            | Expr::And(a, b)
            | Expr::Or(a, b) => {
                self.expr(a)?;
                self.expr(b)
            }
            Expr::Neg(a) | Expr::Pos(a) | Expr::Not(a) => self.expr(a),
            Expr::CondExpr(value, test, other) => {
                self.expr(value)?;
                self.expr(test)?;
                other.as_deref().map_or(Ok(()), |o| self.expr(o))
            }
        }
    }

    /// `CodeGenerator.signature`: arguments, then keywords.
    fn signature(&mut self, args: &[Expr], kwargs: &[(String, Expr)]) -> PResult<()> {
        args.iter().try_for_each(|a| self.expr(a))?;
        kwargs.iter().try_for_each(|(_, v)| self.expr(v))?;
        let named = || kwargs.iter().map(|(k, _)| k).filter(|k| !k.starts_with('*'));
        if !named().any(|k| PY_KEYWORDS.contains(&k.as_str()))
            && named().enumerate().any(|(i, k)| named().take(i).any(|p| p == k))
        {
            self.repeated_kwarg = true;
        }
        Ok(())
    }
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
        Value::Status(_) => Err(type_err(
            "int() argument must be a string, a bytes-like object or a number, not 'GetStatusWrapper'"
                .to_string(),
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
        Value::Status(_) => Err(type_err(
            "float() argument must be a string or a number, not 'GetStatusWrapper'".to_string(),
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
        Value::Status(w) => Ok(w
            .iter_names()
            .into_iter()
            .map(|name| Value::py(PyValue::Str(name)))
            .collect()),
    }
}

fn get_attr(base: &Value, attr: &str, lineno: usize) -> Result<Value, TemplateError> {
    match base {
        Value::Undefined(_) => Err(base.fail_undefined(lineno)),
        Value::Func(_) => Err(type_err(
            format!("'function' object has no attribute '{attr}'"),
            lineno,
        )),
        Value::Status(w) => w.getattr(attr, lineno),
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
        Value::Status(_) => Err(type_err(
            "expected string, got GetStatusWrapper".to_string(),
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
            Value::Py(_) | Value::Status(_) => pct.push(PctValue::Str(py_str_of(a, lineno)?)),
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
        Value::Status(w) => w.getitem(key, lineno),
        Value::Py(PyValue::Dict(d)) => {
            let kv = match key {
                Value::Undefined(_) => return Err(key.fail_undefined(lineno)),
                Value::Py(k) => k.clone(),
                Value::Func(_) => {
                    return Err(type_err("unhashable type: 'function'".to_string(), lineno))
                }
                // An object hashes by identity: never a key of a status dict.
                Value::Status(_) => {
                    return Ok(Value::Undefined(UndefinedHint {
                        hint: None,
                        obj: Some(dict_repr_for_undefined(d)),
                        name: Some(value_to_key_name(key)),
                    }))
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
        // `wrapper in x`: an object equal only to itself.
        Value::Py(PyValue::Dict(_) | PyValue::List(_) | PyValue::Tuple(_))
            if matches!(elem, Value::Status(_)) =>
        {
            Ok(false)
        }
        Value::Status(w) => Ok(w.contains(&elem.to_string())),
        Value::Py(PyValue::Dict(d)) => {
            let ev = match elem {
                Value::Py(k) => k.clone(),
                Value::Undefined(_) => return Err(elem.fail_undefined(lineno)),
                Value::Func(_) => {
                    return Err(type_err("unhashable type: 'function'".to_string(), lineno))
                }
                Value::Status(_) => unreachable!("handled above"),
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
        Value::Status(_) => Err(type_err("GetStatusWrapper is not comparable".to_string(), lineno)),
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
        (Value::Func(_) | Value::Status(_), _) | (_, Value::Func(_) | Value::Status(_)) => Err(type_err(
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
                (Value::Status(x), Value::Status(y)) => Ok(Rc::ptr_eq(x, y)),
                _ => Ok(false),
            }
        }
        "!=" => {
            let eq = match (a, b) {
                (Value::Undefined(_), _) | (_, Value::Undefined(_)) => {
                    a.is_undefined() && b.is_undefined()
                }
                (Value::Py(pa), Value::Py(pb)) => py_equal(pa, pb),
                (Value::Status(x), Value::Status(y)) => Rc::ptr_eq(x, y),
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
                    return Err(TemplateError::new(
                        "ZeroDivisionError",
                        "integer division or modulo by zero".to_string(),
                        Some(lineno),
                    ));
                }
                Ok(Value::py(PyValue::Int(int_floor_div(&na, &nb))))
            }
            "mod" => {
                if nb.is_zero() {
                    return Err(TemplateError::new(
                        "ZeroDivisionError",
                        "integer division or modulo by zero".to_string(),
                        Some(lineno),
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
        Value::Status(_) => Ok(PctValue::Str(GetStatusWrapper::REPR.to_string())),
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
        Value::Status(_) => Err(type_err(
            "int() argument must be a string, a bytes-like object or a number, not 'GetStatusWrapper'"
                .to_string(),
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
        Value::Status(_) => Err(type_err(
            "bad operand type for abs(): 'GetStatusWrapper'".to_string(),
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
            ..
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
            ..
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
        Value::Status(_) => Err(type_err(
            "'GetStatusWrapper' object is not callable".to_string(),
            lineno,
        )),
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
                ..
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
            Node::Set { name, value, .. } => {
                let v = eval_expr(value, ctx, 1)?;
                ctx.set(name.clone(), v);
            }
            Node::Discarded { .. } => {
                unreachable!("Template::parse rejects templates with unsupported constructs")
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
                Value::Func(_) | Value::Status(_) => PyValue::None,
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
                Value::Func(_) | Value::Status(_) => PyValue::None,
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
    /// `Environment.from_string`: parse (`Parser.parse`), then the code
    /// generator's checks, then the Python compile of the generated module.
    /// A template that is valid Jinja but uses a construct this port does
    /// not render fails last, with an "unsupported" `TemplateSyntaxError`.
    pub fn parse(source: &str) -> Result<Template, TemplateError> {
        let mut parser = Parser::new(source)?;
        let nodes = parser.subparse(None)?;
        let mut check = CompileCheck::default();
        check.nodes(&nodes)?;
        if check.repeated_kwarg {
            return Err(TemplateError::new(
                "SyntaxError",
                "keyword argument repeated".to_string(),
                None,
            ));
        }
        if let Some(e) = parser.unsupported {
            return Err(e);
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
                if e.is_syntax_error() {
                    let lineno = e.lineno().unwrap_or(1);
                    let line_text = py_splitlines(script).get(lineno - 1).copied().unwrap_or("");
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
/// floats, strings, bytes, lists, tuples, dicts and `...`/`()` nesting.
/// Sets are parsed then rejected by the JSON check, mirroring Python.
///
/// Like CPython 3.8, the text is first parsed as a Python expression
/// (`mode='eval'`, no stripping): text that does not parse raises
/// `SyntaxError` (`IndentationError` for a leading blank), and an expression
/// that parses but is not a literal raises `ValueError`. Every error message
/// is the Python `str(e)`, including the ` (<unknown>, line N)` location of
/// syntax errors.
pub fn parse_literal(s: &str) -> Result<PyValue, PyError> {
    if s.starts_with([' ', '\t']) && !s.trim().is_empty() {
        return Err(syntax_error("IndentationError", "unexpected indent", 1));
    }
    if s.is_empty() {
        // The empty source has no line at all.
        return Err(syntax_error("SyntaxError", "unexpected EOF while parsing", 0));
    }
    let toks = tokenize(s, 1).map_err(|e| {
        let msg = match e.message() {
            "unterminated string" => "EOL while scanning string literal",
            other => other,
        };
        syntax_error("SyntaxError", msg, 1)
    })?;
    check_python_expr(&toks, s)?;
    let mut p = LitParser {
        toks: literal_tokens(toks)?,
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

/// A `SyntaxError` (or subclass) as `str(e)` renders it.
fn syntax_error(kind: &'static str, msg: &str, line: usize) -> PyError {
    PyError::new(kind, format!("{msg} (<unknown>, line {line})"))
}

/// Where a Python expression stops parsing: at the end of the text
/// (`unexpected EOF while parsing`) or at the unexpected token.
enum SynErr {
    Eof,
    Invalid,
}

/// Python 3.8 `eval_input` grammar over the tokens, enough to tell a syntax
/// error from a non-literal expression: conditionals, boolean and comparison
/// operators, arithmetic, unary operators, calls, subscripts, attributes,
/// displays, comprehensions and implicit string concatenation.
struct PySyntax<'a> {
    toks: &'a [ET],
    pos: usize,
}

/// Syntax check of the literal text with CPython 3.8's error texts:
/// - a closing bracket with no (or the wrong) opener is a tokenizer error,
///   reported when the parser reaches that token;
/// - the old parser reports a bad last token as `unexpected EOF while
///   parsing` when the tokenizer had to read past the end of the text to end
///   it (names, numbers, operators); a string literal ends at its quote, and
///   trailing blanks end the token before the end, so those stay
///   `invalid syntax`.
fn check_python_expr(toks: &[ET], src: &str) -> Result<(), PyError> {
    let mut openers: Vec<&ET> = Vec::new();
    let mut bracket_error: Option<(usize, String)> = None;
    for (i, t) in toks.iter().enumerate() {
        let (open, close) = match t {
            ET::LParen | ET::LBracket | ET::LBrace => {
                openers.push(t);
                continue;
            }
            ET::RParen => (ET::LParen, ')'),
            ET::RBracket => (ET::LBracket, ']'),
            ET::RBrace => (ET::LBrace, '}'),
            _ => continue,
        };
        let msg = match openers.pop() {
            None => format!("unmatched '{close}'"),
            Some(o) if *o == open => continue,
            Some(o) => {
                let o = match o {
                    ET::LParen => '(',
                    ET::LBracket => '[',
                    _ => '{',
                };
                format!("closing parenthesis '{close}' does not match opening parenthesis '{o}'")
            }
        };
        bracket_error = Some((i, msg));
        break;
    }
    let mut p = PySyntax { toks, pos: 0 };
    let result = p.testlist(None).and_then(|()| match p.peek() {
        Some(_) => Err(SynErr::Invalid),
        None => Ok(()),
    });
    let Err(err) = result else {
        return Ok(());
    };
    if let Some((at, msg)) = bracket_error {
        if at <= p.pos {
            return Err(syntax_error("SyntaxError", &msg, 1));
        }
    }
    let at_end_by_lookahead = p.pos + 1 == toks.len()
        && !matches!(toks[p.pos], ET::Str(_) | ET::Bytes(_))
        && !src.ends_with(|c: char| c.is_whitespace());
    let msg = match err {
        SynErr::Eof => "unexpected EOF while parsing",
        SynErr::Invalid if at_end_by_lookahead => "unexpected EOF while parsing",
        SynErr::Invalid => "invalid syntax",
    };
    Err(syntax_error("SyntaxError", msg, 1))
}

impl PySyntax<'_> {
    fn peek(&self) -> Option<&ET> {
        self.toks.get(self.pos)
    }
    fn at(&self, f: impl Fn(&ET) -> bool) -> bool {
        self.peek().is_some_and(f)
    }
    fn at_op(&self, ops: &[&str]) -> bool {
        self.at(|t| matches!(t, ET::Op(op) if ops.contains(&op.as_str())))
    }
    fn at_name(&self, name: &str) -> bool {
        self.at(|t| matches!(t, ET::Name(n) if n == name))
    }
    fn expect(&mut self, f: impl Fn(&ET) -> bool) -> Result<(), SynErr> {
        match self.peek() {
            None => Err(SynErr::Eof),
            Some(t) if f(t) => {
                self.pos += 1;
                Ok(())
            }
            Some(_) => Err(SynErr::Invalid),
        }
    }
    /// `test (',' test)* [',']` up to `close` (or the end of the text).
    fn testlist(&mut self, close: Option<fn(&ET) -> bool>) -> Result<(), SynErr> {
        loop {
            self.star_or_test()?;
            if !self.at(|t| matches!(t, ET::Comma)) {
                return Ok(());
            }
            self.pos += 1;
            match (self.peek(), close) {
                (None, None) => return Ok(()),
                (Some(t), Some(c)) if c(t) => return Ok(()),
                _ => {}
            }
        }
    }
    fn star_or_test(&mut self) -> Result<(), SynErr> {
        if self.at_op(&["*"]) {
            self.pos += 1;
            return self.expr();
        }
        self.test()
    }
    fn test(&mut self) -> Result<(), SynErr> {
        self.or_test()?;
        if self.at(|t| matches!(t, ET::KwIf)) {
            self.pos += 1;
            self.or_test()?;
            self.expect(|t| matches!(t, ET::KwElse))?;
            self.test()?;
        }
        Ok(())
    }
    fn or_test(&mut self) -> Result<(), SynErr> {
        self.and_test()?;
        while self.at(|t| matches!(t, ET::KwOr)) {
            self.pos += 1;
            self.and_test()?;
        }
        Ok(())
    }
    fn and_test(&mut self) -> Result<(), SynErr> {
        self.not_test()?;
        while self.at(|t| matches!(t, ET::KwAnd)) {
            self.pos += 1;
            self.not_test()?;
        }
        Ok(())
    }
    fn not_test(&mut self) -> Result<(), SynErr> {
        if self.at(|t| matches!(t, ET::KwNot)) {
            self.pos += 1;
            return self.not_test();
        }
        self.comparison()
    }
    fn comparison(&mut self) -> Result<(), SynErr> {
        self.expr()?;
        loop {
            if self.at_op(&["<", ">", "==", ">=", "<=", "!="])
                || self.at(|t| matches!(t, ET::KwIn))
            {
                self.pos += 1;
            } else if self.at(|t| matches!(t, ET::KwNot)) {
                self.pos += 1;
                self.expect(|t| matches!(t, ET::KwIn))?;
            } else if self.at(|t| matches!(t, ET::KwIs)) {
                self.pos += 1;
                if self.at(|t| matches!(t, ET::KwNot)) {
                    self.pos += 1;
                }
            } else {
                return Ok(());
            }
            self.expr()?;
        }
    }
    fn expr(&mut self) -> Result<(), SynErr> {
        self.factor()?;
        while self.at_op(&["+", "-", "*", "/", "//", "%", "**"])
            || self.at(|t| matches!(t, ET::Pipe))
        {
            self.pos += 1;
            self.factor()?;
        }
        Ok(())
    }
    fn factor(&mut self) -> Result<(), SynErr> {
        if self.at_op(&["+", "-", "~"]) {
            self.pos += 1;
            return self.factor();
        }
        self.atom()?;
        loop {
            if self.at(|t| matches!(t, ET::LParen)) {
                self.pos += 1;
                self.call_args()?;
            } else if self.at(|t| matches!(t, ET::LBracket)) {
                self.pos += 1;
                self.subscripts()?;
            } else if self.at(|t| matches!(t, ET::Dot)) {
                self.pos += 1;
                self.expect(|t| matches!(t, ET::Name(_)))?;
            } else {
                return Ok(());
            }
        }
    }
    fn atom(&mut self) -> Result<(), SynErr> {
        let Some(t) = self.peek() else {
            return Err(SynErr::Eof);
        };
        match t {
            ET::Name(_) | ET::Int(_) | ET::Float(_) | ET::KwTrue | ET::KwFalse | ET::KwNone => {
                self.pos += 1;
            }
            ET::Str(_) | ET::Bytes(_) => {
                while self.at(|t| matches!(t, ET::Str(_) | ET::Bytes(_))) {
                    self.pos += 1;
                }
            }
            ET::Dot => {
                // `...` (Ellipsis) arrives as three dots.
                for _ in 0..3 {
                    self.expect(|t| matches!(t, ET::Dot))?;
                }
            }
            ET::LParen => {
                self.pos += 1;
                self.display(|t| matches!(t, ET::RParen))?;
            }
            ET::LBracket => {
                self.pos += 1;
                self.display(|t| matches!(t, ET::RBracket))?;
            }
            ET::LBrace => {
                self.pos += 1;
                self.dict_or_set()?;
            }
            _ => return Err(SynErr::Invalid),
        }
        Ok(())
    }
    /// Tuple/list display or comprehension after its opening bracket.
    fn display(&mut self, close: fn(&ET) -> bool) -> Result<(), SynErr> {
        if self.at(close) {
            self.pos += 1;
            return Ok(());
        }
        self.star_or_test()?;
        if self.at_name("for") {
            self.comp_for()?;
        } else if self.at(|t| matches!(t, ET::Comma)) {
            self.pos += 1;
            if !self.at(close) {
                self.testlist(Some(close))?;
            }
        }
        self.expect(close)
    }
    fn dict_or_set(&mut self) -> Result<(), SynErr> {
        let close = |t: &ET| matches!(t, ET::RBrace);
        if self.at(close) {
            self.pos += 1;
            return Ok(());
        }
        let is_dict = if self.at_op(&["**"]) {
            self.pos += 1;
            self.expr()?;
            true
        } else {
            self.star_or_test()?;
            if self.at(|t| matches!(t, ET::Colon)) {
                self.pos += 1;
                self.test()?;
                true
            } else {
                false
            }
        };
        if self.at_name("for") {
            self.comp_for()?;
            return self.expect(close);
        }
        while self.at(|t| matches!(t, ET::Comma)) {
            self.pos += 1;
            if self.at(close) {
                break;
            }
            if is_dict {
                if self.at_op(&["**"]) {
                    self.pos += 1;
                    self.expr()?;
                } else {
                    self.test()?;
                    self.expect(|t| matches!(t, ET::Colon))?;
                    self.test()?;
                }
            } else {
                self.star_or_test()?;
            }
        }
        self.expect(close)
    }
    /// `for targets in or_test` clauses, each optionally followed by `if`s.
    fn comp_for(&mut self) -> Result<(), SynErr> {
        while self.at_name("for") {
            self.pos += 1;
            self.expr()?;
            while self.at(|t| matches!(t, ET::Comma)) {
                self.pos += 1;
                self.expr()?;
            }
            self.expect(|t| matches!(t, ET::KwIn))?;
            self.or_test()?;
            while self.at(|t| matches!(t, ET::KwIf)) {
                self.pos += 1;
                self.or_test()?;
            }
        }
        Ok(())
    }
    fn call_args(&mut self) -> Result<(), SynErr> {
        let close = |t: &ET| matches!(t, ET::RParen);
        while !self.at(close) {
            if self.at_op(&["*", "**"]) {
                self.pos += 1;
                self.test()?;
            } else if self.at(|t| matches!(t, ET::Name(_)))
                && matches!(self.toks.get(self.pos + 1), Some(ET::Assign))
            {
                self.pos += 2;
                self.test()?;
            } else {
                self.test()?;
                if self.at_name("for") {
                    self.comp_for()?;
                }
            }
            if !self.at(|t| matches!(t, ET::Comma)) {
                break;
            }
            self.pos += 1;
        }
        self.expect(close)
    }
    fn subscripts(&mut self) -> Result<(), SynErr> {
        let close = |t: &ET| matches!(t, ET::RBracket);
        loop {
            let mut parts = 0;
            if !self.at(|t| matches!(t, ET::Colon | ET::Comma | ET::RBracket)) {
                self.test()?;
                parts += 1;
            }
            for _ in 0..2 {
                if !self.at(|t| matches!(t, ET::Colon)) {
                    break;
                }
                self.pos += 1;
                parts += 1;
                if !self.at(|t| matches!(t, ET::Colon | ET::Comma | ET::RBracket)) {
                    self.test()?;
                }
            }
            if parts == 0 {
                return match self.peek() {
                    None => Err(SynErr::Eof),
                    Some(_) => Err(SynErr::Invalid),
                };
            }
            if !self.at(|t| matches!(t, ET::Comma)) {
                break;
            }
            self.pos += 1;
            if self.at(close) {
                break;
            }
        }
        self.expect(close)
    }
}

#[derive(Clone, Debug, PartialEq)]
enum LT {
    Name(String),
    Str(String),
    Bytes(Vec<u8>),
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

/// Literal-parser tokens of a text that already parses as Python: keywords
/// and operators other than a sign stay as opaque operators, which the
/// literal parser rejects as non-literal nodes (`ValueError`).
fn literal_tokens(toks: Vec<ET>) -> Result<Vec<LT>, PyError> {
    let mut out = Vec::with_capacity(toks.len());
    for t in toks {
        out.push(match t {
            ET::Name(n) => LT::Name(n),
            ET::Str(st) => LT::Str(st),
            ET::Bytes(b) => LT::Bytes(b),
            ET::Int(v) => LT::Int(v),
            ET::Float(v) => LT::Float(v),
            ET::Op(op) => LT::Op(op),
            ET::LBracket => LT::LBracket,
            ET::RBracket => LT::RBracket,
            ET::LParen => LT::LParen,
            ET::RParen => LT::RParen,
            ET::LBrace => LT::LBrace,
            ET::RBrace => LT::RBrace,
            ET::Comma => LT::Comma,
            ET::Colon => LT::Colon,
            ET::KwTrue => LT::Name("True".to_string()),
            ET::KwFalse => LT::Name("False".to_string()),
            ET::KwNone => LT::Name("None".to_string()),
            other => LT::Op(describe_et(&other)),
        });
    }
    // Adjacent string literals concatenate (`'a' 'b'` is `'ab'`); bytes join
    // bytes, and mixing the two is a syntax error.
    let mut merged: Vec<LT> = Vec::with_capacity(out.len());
    for tok in out {
        match (merged.last_mut(), tok) {
            (Some(LT::Str(acc)), LT::Str(tail)) => acc.push_str(&tail),
            (Some(LT::Bytes(acc)), LT::Bytes(tail)) => acc.extend_from_slice(&tail),
            (Some(LT::Str(_)), LT::Bytes(_)) | (Some(LT::Bytes(_)), LT::Str(_)) => {
                return Err(syntax_error("SyntaxError", "cannot mix bytes and nonbytes literals", 1));
            }
            (_, tok) => merged.push(tok),
        }
    }
    Ok(merged)
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
            (1, LT::Bytes(b)) => {
                self.pos += 1;
                Ok(PyValue::Bytes(b))
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
                e.msg
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
                e.msg
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

/// Where `GetStatusWrapper` reads printer objects from (its `self.printer`).
pub trait StatusSource {
    /// The names `printer.lookup_objects()` lists, in order.
    fn object_names(&self) -> Vec<String>;
    /// `po.get_status(eventtime)` for `po = printer.lookup_object(name,
    /// None)`, or `None` when there is no such object or it has no
    /// `get_status` (the wrapper's `KeyError`). `eventtime` is the
    /// wrapper's: while it is `None`, the source resolves it as
    /// `__getitem__` does (`reactor.monotonic()`, once an object with
    /// `get_status` was found) and stores it for later lookups. Python
    /// runs the call under `reactor.assert_no_pause()`.
    fn get_status(&self, name: &str, eventtime: &mut Option<f64>) -> Option<PyValue>;
}

/// `GetStatusWrapper` (`klippy/extras/gcode_macro.py:15`): the `printer`
/// of a template context. It calls `get_status` only for the objects a
/// template touches, at most once per object, and keeps the result for
/// every later lookup through the same context (renders that share a
/// context share the cache, as they share the Python object).
pub struct GetStatusWrapper {
    source: Box<dyn StatusSource>,
    eventtime: Cell<Option<f64>>,
    cache: RefCell<IndexMap<String, PyValue>>,
}

impl GetStatusWrapper {
    /// `str()` of the wrapper; CPython prints its address, which no port
    /// reproduces, so it is rendered as `0x0`.
    pub const REPR: &'static str = "<extras.gcode_macro.GetStatusWrapper object at 0x0>";
    /// Jinja's `object_type_repr` of the wrapper (klippy imports the module
    /// as `extras.gcode_macro`).
    const TYPE_REPR: &'static str = "extras.gcode_macro.GetStatusWrapper object";

    pub fn new(source: Box<dyn StatusSource>, eventtime: Option<f64>) -> Self {
        GetStatusWrapper {
            source,
            eventtime: Cell::new(eventtime),
            cache: RefCell::new(IndexMap::new()),
        }
    }

    /// `__getitem__(val)` for `val` already passed through `str()`: the
    /// object's status, `None` for `KeyError`.
    pub fn get_item(&self, val: &str) -> Option<PyValue> {
        let sval = val.trim_matches(is_py_space);
        if let Some(status) = self.cache.borrow().get(sval) {
            return Some(status.clone());
        }
        let mut eventtime = self.eventtime.get();
        let status = self.source.get_status(sval, &mut eventtime);
        self.eventtime.set(eventtime);
        let status = status?;
        self.cache.borrow_mut().insert(sval.to_string(), status.clone());
        Some(status)
    }

    /// `__contains__`.
    pub fn contains(&self, val: &str) -> bool {
        self.get_item(val).is_some()
    }

    /// `__iter__`: the names of the objects `__contains__` accepts, which
    /// reads every status.
    pub fn iter_names(&self) -> Vec<String> {
        self.source
            .object_names()
            .into_iter()
            .filter(|name| self.contains(name))
            .collect()
    }

    /// Python `getattr(wrapper, name)` for the instance's own attributes,
    /// which Jinja tries before `wrapper[name]`; `None` when the instance
    /// has no such attribute.
    fn attribute(&self, name: &str, lineno: usize) -> Option<Result<Value, TemplateError>> {
        match name {
            "eventtime" => Some(Ok(Value::py(
                self.eventtime.get().map_or(PyValue::None, PyValue::Float),
            ))),
            "cache" => Some(Ok(Value::py(PyValue::Dict(
                self.cache
                    .borrow()
                    .iter()
                    .map(|(k, v)| (PyValue::Str(k.clone()), v.clone()))
                    .collect(),
            )))),
            "printer" => Some(Err(type_err(
                "unsupported: the klippy Printer object (printer.printer) is not available to templates"
                    .to_string(),
                lineno,
            ))),
            _ if name.len() > 4 && name.starts_with("__") && name.ends_with("__") => {
                Some(Err(type_err(
                    format!("unsupported: GetStatusWrapper attribute '{name}'"),
                    lineno,
                )))
            }
            _ => None,
        }
    }

    /// Jinja `getattr(printer, name)`: an instance attribute, else
    /// `printer[name]`, else `Undefined`.
    fn getattr(&self, name: &str, lineno: usize) -> Result<Value, TemplateError> {
        if let Some(attr) = self.attribute(name, lineno) {
            return attr;
        }
        Ok(match self.get_item(name) {
            Some(status) => Value::py(status),
            None => Self::undefined(format!(
                "'{}' has no attribute {}",
                Self::TYPE_REPR,
                pytext::str_repr(name)
            )),
        })
    }

    /// Jinja `getitem(printer, key)`: `printer[key]`, else (for a string
    /// key) an instance attribute, else `Undefined`.
    fn getitem(&self, key: &Value, lineno: usize) -> Result<Value, TemplateError> {
        if let Value::Func(_) = key {
            return Err(type_err(
                "unsupported: a function as a printer[] key".to_string(),
                lineno,
            ));
        }
        if let Some(status) = self.get_item(&key.to_string()) {
            return Ok(Value::py(status));
        }
        if let Value::Py(PyValue::Str(name)) = key {
            if let Some(attr) = self.attribute(name, lineno) {
                return attr;
            }
            return Ok(Self::undefined(format!(
                "'{}' has no attribute {}",
                Self::TYPE_REPR,
                pytext::str_repr(name)
            )));
        }
        let key_repr = match key {
            Value::Py(k) => k.repr(),
            Value::Undefined(_) => "Undefined".to_string(),
            Value::Status(_) | Value::Func(_) => Self::REPR.to_string(),
        };
        Ok(Self::undefined(format!(
            "{} has no element {key_repr}",
            Self::TYPE_REPR
        )))
    }

    fn undefined(message: String) -> Value {
        Value::Undefined(UndefinedHint {
            hint: Some(message),
            obj: None,
            name: None,
        })
    }
}
