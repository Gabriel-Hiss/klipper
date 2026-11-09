//! Port of the `gcode.py` parse/dispatch fragments (`klippy/gcode.py`).
//!
//! Only the parsing and registration layer is ported here: [`GCodeCommand`]
//! (parameter accessors and their error texts), [`parse_line`] (the
//! [`GCodeDispatch::_process_commands`](klippy/gcode.py) line grammar),
//! [`is_traditional_gcode`], extended-parameter parsing via `pytext::shlex`,
//! and the [`Dispatch`] handler tables (`register_command` /
//! `register_mux_command` name validation and mux dispatch rules).
//! Printer, reactor, toolhead and response-sink collaborators are not
//! dependencies of this module; their effects are represented as plain
//! values ([`CommandError`]) so the parse contract can be verified without
//! the full host.

use std::rc::Rc;

use indexmap::IndexMap;
use klippy_api::config::IntArgs;
use pycompat::text::{float_from_string, float_str, int_from_string};
use pytext::pyre;
use pytext::shlex::Shlex;

/// `klippy/gcode.py` `CommandError`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CommandError(pub String);

impl std::fmt::Display for CommandError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for CommandError {}

/// `printer.config_error` mirror for registration failures. The message
/// text matches `configparser.Error` / `Printer.config_error` as raised
/// by `register_command` / `register_mux_command`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for ConfigError {}

/// `klippy/gcode.py` `Coord` (padded 4-tuple with `x/y/z/e` access).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coord(pub [f64; 4]);

impl Coord {
    pub fn new(t: &[f64]) -> Self {
        let mut v = [0.0f64; 4];
        for (i, &x) in t.iter().take(4).enumerate() {
            v[i] = x;
        }
        Coord(v)
    }
    pub fn x(self) -> f64 {
        self.0[0]
    }
    pub fn y(self) -> f64 {
        self.0[1]
    }
    pub fn z(self) -> f64 {
        self.0[2]
    }
    pub fn e(self) -> f64 {
        self.0[3]
    }
}

// ---------------------------------------------------------------------------
// Python string helpers (ASCII-complete; unicode approximated by Rust
// `char` predicates which agree with CPython on the gcode alphabet).
// ---------------------------------------------------------------------------

fn is_py_space(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}' | '\u{1d}' | '\u{1e}' | '\u{1f}')
}

fn py_strip(s: &str) -> &str {
    s.trim_matches(is_py_space)
}

fn py_upper(s: &str) -> String {
    s.chars().flat_map(|c| c.to_uppercase()).collect()
}

/// Python `str.isdigit` for a single character (decimal digits).
fn is_py_digit(c: char) -> bool {
    c.is_ascii_digit()
}

fn is_py_upper(c: char) -> bool {
    c.is_uppercase()
}

fn is_py_alnum(c: char) -> bool {
    c.is_alphanumeric()
}

fn bound_str_i(v: i64) -> String {
    v.to_string()
}

/// Python slicing `s[a:b]` on code points, where `a`/`b` are exact
/// (possibly negative or out-of-range) indices. `chars` is the string
/// split into code points.
fn py_slice(chars: &[char], start: isize, end: isize) -> String {
    let len = chars.len() as isize;
    let mut a = start;
    let mut b = end;
    if a < 0 {
        a += len;
    }
    if b < 0 {
        b += len;
    }
    if a < 0 {
        a = 0;
    }
    if b < 0 {
        b = 0;
    }
    if a > len {
        a = len;
    }
    if b > len {
        b = len;
    }
    if a >= b {
        return String::new();
    }
    chars[a as usize..b as usize].iter().collect()
}

/// Python `str.find(sub)` on code points; `-1` when absent.
fn py_find(chars: &[char], sub: &[char]) -> isize {
    if sub.is_empty() {
        return 0;
    }
    if sub.len() > chars.len() {
        return -1;
    }
    for i in 0..=chars.len() - sub.len() {
        if &chars[i..i + sub.len()] == sub {
            return i as isize;
        }
    }
    -1
}

/// Python `str.rfind(sub)` on code points; `-1` when absent.
fn py_rfind(chars: &[char], sub: &[char]) -> isize {
    if sub.is_empty() {
        return chars.len() as isize;
    }
    if sub.len() > chars.len() {
        return -1;
    }
    for i in (0..=chars.len() - sub.len()).rev() {
        if &chars[i..i + sub.len()] == sub {
            return i as isize;
        }
    }
    -1
}

// ---------------------------------------------------------------------------
// Line parsing (`GCodeDispatch._process_commands`)
// ---------------------------------------------------------------------------

pub const ARGS_RE_PATTERN: &str = "([A-Z_]+|[A-Z*])";

/// One parsed gcode line.
#[derive(Debug, Clone, PartialEq)]
pub struct ParsedLine {
    /// `origline`: the stripped original line (case preserved).
    pub origline: String,
    /// `cmd`: upper-cased command token (may contain a space, e.g.
    /// `M117 hello`).
    pub cmd: String,
    /// `params`: upper-cased letter keys to stripped value strings, in
    /// first-seen order.
    pub params: IndexMap<String, String>,
}

fn args_split(upper_line: &str) -> Vec<String> {
    pyre::compile(ARGS_RE_PATTERN)
        .map(|re| re.split(upper_line))
        .unwrap_or_else(|_| vec![upper_line.to_string()])
}

fn join_slice(parts: &[String], start: usize, end: usize) -> String {
    let mut out = String::new();
    let s = start.min(parts.len());
    let e = end.min(parts.len());
    if s < e {
        for p in &parts[s..e] {
            out.push_str(p);
        }
    }
    out
}

/// Parse one input line exactly as `GCodeDispatch._process_commands` does
/// (lines 202-219): strip, cut at the first `;`, split the upper-cased
/// line with `([A-Z_]+|[A-Z*])`, skip a leading `N` line number, build
/// the params dict.
pub fn parse_line(line: &str) -> ParsedLine {
    let origline_owned = py_strip(line).to_string();
    let mut work = origline_owned.clone();
    if let Some(pos) = work.find(';') {
        work.truncate(pos);
    }
    let upper = py_upper(&work);
    let parts = args_split(&upper);
    let cmd = if join_slice(&parts, 0, 2) == "N" {
        py_strip(&join_slice(&parts, 3, 5)).to_string()
    } else {
        py_strip(&join_slice(&parts, 0, 3)).to_string()
    };
    let mut params: IndexMap<String, String> = IndexMap::new();
    let mut i = 1usize;
    while i < parts.len() {
        let key = parts[i].clone();
        let val = if i + 1 < parts.len() {
            py_strip(&parts[i + 1]).to_string()
        } else {
            String::new()
        };
        params.insert(key, val);
        i += 2;
    }
    ParsedLine {
        origline: origline_owned,
        cmd,
        params,
    }
}

// ---------------------------------------------------------------------------
// `is_traditional_gcode`
// ---------------------------------------------------------------------------

/// Port of `GCodeDispatch.is_traditional_gcode` (lines 125-132).
/// True for `G1`, `M104`, `T0`, ...: first whitespace token upper-cased
/// parses as `letter + float` with an upper-case first char and a digit
/// second char.
pub fn is_traditional_gcode(cmd: &str) -> bool {
    let upper = py_upper(cmd);
    let token = upper.split_whitespace().next().unwrap_or("");
    // `float(cmd[1:])` must succeed.
    let chars: Vec<char> = token.chars().collect();
    if chars.len() < 2 {
        return false;
    }
    let tail: String = chars[1..].iter().collect();
    let parsed = match float_from_string(&tail) {
        Ok(v) => v,
        Err(_) => return false,
    };
    let _ = parsed;
    is_py_upper(chars[0]) && is_py_digit(chars[1])
}

// ---------------------------------------------------------------------------
// Extended-parameter parsing (`_get_extended_params`)
// ---------------------------------------------------------------------------

/// Apply `_get_extended_params` (lines 266-281) to a mutable params map:
/// `shlex` over the raw parameter text (`posix`, whitespace split,
/// commenters `#;`), each token split on the first `=`, key upper-cased.
/// On any `ValueError` the caller raises `Malformed command '<line>'`.
/// Returns the new params on success.
pub fn parse_extended_params(
    rawparams: &str,
    commandline: &str,
) -> Result<IndexMap<String, String>, CommandError> {
    let tokens = match Shlex::new_klippy(rawparams).tokens() {
        Ok(t) => t,
        Err(_) => {
            return Err(CommandError(format!(
                "Malformed command '{}'",
                commandline
            )));
        }
    };
    let mut out: IndexMap<String, String> = IndexMap::new();
    for tok in tokens {
        let eq = tok.find('=');
        match eq {
            Some(pos) => {
                let k = &tok[..pos];
                let v = &tok[pos + 1..];
                out.insert(py_upper(k), v.to_string());
            }
            None => {
                return Err(CommandError(format!(
                    "Malformed command '{}'",
                    commandline
                )));
            }
        }
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// `GCodeCommand`
// ---------------------------------------------------------------------------

/// Parameter parser kind for [`GCodeCommand::get_typed`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParserKind {
    Str,
    Int,
    Float,
}

/// A parsed gcode command (`klippy/gcode.py` `GCodeCommand`, lines 24-96).
#[derive(Debug, Clone)]
pub struct GCodeCommand {
    command: String,
    commandline: String,
    params: IndexMap<String, String>,
    need_ack: bool,
}

impl GCodeCommand {
    pub fn new(
        command: String,
        commandline: String,
        params: IndexMap<String, String>,
        need_ack: bool,
    ) -> Self {
        GCodeCommand {
            command,
            commandline,
            params,
            need_ack,
        }
    }

    pub fn from_parsed(parsed: &ParsedLine, need_ack: bool) -> Self {
        GCodeCommand {
            command: parsed.cmd.clone(),
            commandline: parsed.origline.clone(),
            params: parsed.params.clone(),
            need_ack,
        }
    }

    pub fn get_command(&self) -> &str {
        &self.command
    }

    pub fn get_commandline(&self) -> &str {
        &self.commandline
    }

    pub fn get_command_parameters(&self) -> &IndexMap<String, String> {
        &self.params
    }

    pub fn set_command(&mut self, cmd: String) {
        self.command = cmd;
    }

    pub fn set_params(&mut self, params: IndexMap<String, String>) {
        self.params = params;
    }

    pub fn need_ack(&self) -> bool {
        self.need_ack
    }

    /// `get_raw_command_parameters` (lines 40-53).
    pub fn get_raw_command_parameters(&self) -> String {
        let command = &self.command;
        let origline = &self.commandline;
        let cmd_chars: Vec<char> = command.chars().collect();
        let orig_chars: Vec<char> = origline.chars().collect();
        let orig_upper_chars: Vec<char> = py_upper(origline).chars().collect();
        let cmd_len = cmd_chars.len() as isize;
        let orig_len = orig_chars.len() as isize;
        let mut param_start: isize = cmd_len;
        let mut param_end: isize = orig_len;
        // `origline[:param_start].upper() != command`
        let prefix = py_slice(&orig_chars, 0, param_start);
        if py_upper(&prefix) != *command {
            let found = py_find(&orig_upper_chars, &cmd_chars);
            param_start += found;
            let end = py_rfind(&orig_chars, &['*']);
            if end >= 0 {
                let tail = py_slice(&orig_chars, end + 1, orig_len);
                if tail.chars().all(is_py_digit) && !tail.is_empty() {
                    param_end = end;
                }
            }
        }
        // `origline[param_start:param_start+1].isspace()`
        let one = py_slice(&orig_chars, param_start, param_start + 1);
        if one.chars().all(is_py_space) && !one.is_empty() {
            // Python `.isspace()` is False for `''`, True for one space
            // character; `all` on a single char matches exactly.
            param_start += 1;
        }
        py_slice(&orig_chars, param_start, param_end)
    }

    /// `ack()` (lines 54-62). Returns the emitted `ok` line, or `None`
    /// when the command does not need an ack. Consumes the ack.
    pub fn ack(&mut self, msg: Option<&str>) -> Option<String> {
        if !self.need_ack {
            return None;
        }
        let ok = match msg {
            Some(m) if !m.is_empty() => format!("ok {}", m),
            _ => "ok".to_string(),
        };
        self.need_ack = false;
        Some(ok)
    }

    fn missing(name: &str, commandline: &str) -> CommandError {
        CommandError(format!(
            "Error on '{}': missing {}",
            commandline, name
        ))
    }

    fn unable(commandline: &str, value: &str) -> CommandError {
        CommandError(format!(
            "Error on '{}': unable to parse {}",
            commandline, value
        ))
    }

    /// Generic `get(name, default, parser, ...)` with bounds as floats.
    /// `default` is `None` for the `sentinel` (missing is an error),
    /// `Some(v)` for the explicit default which is returned unchanged
    /// when the parameter is absent.
    pub fn get_float(
        &self,
        name: &str,
        default: Option<f64>,
        minval: Option<f64>,
        maxval: Option<f64>,
        above: Option<f64>,
        below: Option<f64>,
    ) -> Result<f64, CommandError> {
        match self.get_typed_float(
            name,
            default.map(Some),
            minval,
            maxval,
            above,
            below,
            IntArgs::NONE,
        ) {
            Ok(Some(v)) => Ok(v),
            Ok(None) => Ok(0.0),
            Err(e) => Err(e),
        }
    }

    /// Nullable float getter reproducing the Python `None` default:
    /// outer `None` is the sentinel, `Some(None)` is the Python `None`
    /// default, `Some(Some(v))` is a float default. `ints` marks the
    /// bounds Python passes as `int` (printed without `.0`).
    #[allow(clippy::too_many_arguments)]
    pub fn get_float_opt(
        &self,
        name: &str,
        default: Option<Option<f64>>,
        minval: Option<f64>,
        maxval: Option<f64>,
        above: Option<f64>,
        below: Option<f64>,
        ints: IntArgs,
    ) -> Result<Option<f64>, CommandError> {
        self.get_typed_float(name, default, minval, maxval, above, below, ints)
    }

    #[allow(clippy::too_many_arguments)]
    fn get_typed_float(
        &self,
        name: &str,
        default: Option<Option<f64>>,
        minval: Option<f64>,
        maxval: Option<f64>,
        above: Option<f64>,
        below: Option<f64>,
        ints: IntArgs,
    ) -> Result<Option<f64>, CommandError> {
        let commandline = self.commandline.clone();
        let value = match self.params.get(name) {
            None => {
                return match default {
                    None => Err(Self::missing(name, &commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let parsed = match float_from_string(&value) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        if let Some(mn) = minval {
            if parsed < mn {
                return Err(CommandError(format!(
                    "Error on '{}': {} must have minimum of {}",
                    commandline,
                    name,
                    ints.value(IntArgs::MINVAL, mn).str()
                )));
            }
        }
        if let Some(mx) = maxval {
            if parsed > mx {
                return Err(CommandError(format!(
                    "Error on '{}': {} must have maximum of {}",
                    commandline,
                    name,
                    ints.value(IntArgs::MAXVAL, mx).str()
                )));
            }
        }
        if let Some(ab) = above {
            if parsed <= ab {
                return Err(CommandError(format!(
                    "Error on '{}': {} must be above {}",
                    commandline,
                    name,
                    ints.value(IntArgs::ABOVE, ab).str()
                )));
            }
        }
        if let Some(be) = below {
            if parsed >= be {
                return Err(CommandError(format!(
                    "Error on '{}': {} must be below {}",
                    commandline,
                    name,
                    ints.value(IntArgs::BELOW, be).str()
                )));
            }
        }
        Ok(Some(parsed))
    }

    pub fn get_int(
        &self,
        name: &str,
        default: Option<i64>,
        minval: Option<i64>,
        maxval: Option<i64>,
    ) -> Result<i64, CommandError> {
        match self.get_int_opt(name, default.map(Some), minval, maxval) {
            Ok(Some(v)) => Ok(v),
            Ok(None) => Ok(0),
            Err(e) => Err(e),
        }
    }

    pub fn get_int_opt(
        &self,
        name: &str,
        default: Option<Option<i64>>,
        minval: Option<i64>,
        maxval: Option<i64>,
    ) -> Result<Option<i64>, CommandError> {
        let commandline = self.commandline.clone();
        let value = match self.params.get(name) {
            None => {
                return match default {
                    None => Err(Self::missing(name, &commandline)),
                    Some(v) => Ok(v),
                };
            }
            Some(v) => v.clone(),
        };
        let pyint = match int_from_string(&value, 10) {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        // Exact comparison via decimal text: the Python `int` value can
        // exceed `i64`; comparisons stay exact through `to_string` until
        // the value is known to fit. Bounds in gcode are small, so the
        // `i64` conversion below only fails on out-of-range huge ints,
        // which we report as an unable-to-parse error (documented
        // deviation; recordings only exercise small ints).
        let parsed: i64 = match pyint.inner().to_string().parse::<i64>() {
            Ok(v) => v,
            Err(_) => return Err(Self::unable(&commandline, &value)),
        };
        if let Some(mn) = minval {
            if parsed < mn {
                return Err(CommandError(format!(
                    "Error on '{}': {} must have minimum of {}",
                    commandline,
                    name,
                    bound_str_i(mn)
                )));
            }
        }
        if let Some(mx) = maxval {
            if parsed > mx {
                return Err(CommandError(format!(
                    "Error on '{}': {} must have maximum of {}",
                    commandline,
                    name,
                    bound_str_i(mx)
                )));
            }
        }
        Ok(Some(parsed))
    }

    pub fn get_str(
        &self,
        name: &str,
        default: Option<String>,
    ) -> Result<String, CommandError> {
        match self.get_str_opt(name, default.map(Some)) {
            Ok(Some(v)) => Ok(v),
            Ok(None) => Ok(String::new()),
            Err(e) => Err(e),
        }
    }

    pub fn get_str_opt(
        &self,
        name: &str,
        default: Option<Option<String>>,
    ) -> Result<Option<String>, CommandError> {
        match self.params.get(name) {
            None => match default {
                None => Err(Self::missing(name, &self.commandline)),
                Some(v) => Ok(v),
            },
            Some(v) => Ok(Some(v.clone())),
        }
    }

    /// Generic typed getter used by mux dispatch: `parser` selects
    /// `str` / `int` / `float` exactly as `GCodeCommand.get`.
    pub fn get_typed(
        &self,
        name: &str,
        default: Option<Option<String>>,
        parser: ParserKind,
    ) -> Result<Option<String>, CommandError> {
        match parser {
            ParserKind::Str => self.get_str_opt(name, default),
            ParserKind::Int => {
                let d = default.map(|o| o.and_then(|s| s.parse::<i64>().ok()));
                self.get_int_opt(name, d, None, None)
                    .map(|o| o.map(|v| v.to_string()))
            }
            ParserKind::Float => {
                let d = default.map(|o| o.and_then(|s| float_from_string(&s).ok()));
                self.get_float_opt(name, d, None, None, None, None, IntArgs::NONE)
                    .map(|o| o.map(float_str))
            }
        }
    }

    /// `_get_extended_params` mutation (lines 266-281): parses the raw
    /// parameter text with `shlex` and replaces `self._params`.
    pub fn apply_extended_params(&mut self) -> Result<(), CommandError> {
        let raw = self.get_raw_command_parameters();
        let eparams = parse_extended_params(&raw, &self.commandline)?;
        self.params = eparams;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Registration and mux (`register_command`, `register_mux_command`)
// ---------------------------------------------------------------------------

pub type GcodeHandler = Rc<dyn Fn(&mut GCodeCommand) -> Result<(), CommandError>>;

fn validate_extended_name(cmd: &str) -> Result<(), ConfigError> {
    // Mirrors lines 145-149: non-traditional names must be upper-case,
    // alphanumeric after `_` -> `A`, not start with a digit, and not have
    // a digit at index 1.
    let upper_ok = py_upper(cmd) == cmd;
    let sanitized: String = cmd.chars().map(|c| if c == '_' { 'A' } else { c }).collect();
    let alnum_ok = !sanitized.is_empty() && sanitized.chars().all(is_py_alnum);
    let first = cmd.chars().next();
    let first_bad = matches!(first, Some(c) if is_py_digit(c));
    let second = cmd.chars().nth(1);
    let second_bad = matches!(second, Some(c) if is_py_digit(c));
    if !upper_ok || !alnum_ok || first_bad || second_bad {
        return Err(ConfigError(format!(
            "Can't register '{}' as it is an invalid name",
            cmd
        )));
    }
    Ok(())
}

/// Python `dict` repr for the mux error message (`%s` of `prev_values`).
/// Keys render as quoted strings (or `None`); values render as function
/// names in the Python `<function name>` shape without the address,
/// which cannot be byte-stable across interpreters.
fn python_dict_repr(m: &IndexMap<Option<String>, String>) -> String {
    let mut parts: Vec<String> = Vec::new();
    for (k, v) in m {
        let ks = match k {
            Some(s) => format!("'{}'", s),
            None => "None".to_string(),
        };
        parts.push(format!("{}: <function {}>", ks, v));
    }
    format!("{{{}}}", parts.join(", "))
}

/// Handler table of the ported `GCodeDispatch` (minus the printer).
///
/// `ready_gcode_handlers` holds every registration; `base_gcode_handlers`
/// holds only `when_not_ready` entries. `gcode_handlers` points at one of
/// the two depending on readiness, exactly as the Python.
pub type MuxEntry = (String, IndexMap<Option<String>, GcodeHandler>, IndexMap<Option<String>, String>);

pub struct Dispatch {
    ready: IndexMap<String, GcodeHandler>,
    base: IndexMap<String, GcodeHandler>,
    help: IndexMap<String, String>,
    mux: IndexMap<String, MuxEntry>,
}

impl Dispatch {
    pub fn new() -> Self {
        Dispatch {
            ready: IndexMap::new(),
            base: IndexMap::new(),
            help: IndexMap::new(),
            mux: IndexMap::new(),
        }
    }

    pub fn is_traditional_gcode(cmd: &str) -> bool {
        is_traditional_gcode(cmd)
    }

    /// `register_command(cmd, func, when_not_ready, desc)` (lines 133-157).
    /// Pass `handler = None` to unregister and obtain the old handler.
    pub fn register_command(
        &mut self,
        cmd: &str,
        handler: Option<(String, GcodeHandler)>,
        when_not_ready: bool,
        desc: Option<&str>,
    ) -> Result<Option<GcodeHandler>, ConfigError> {
        match handler {
            None => {
                let old = self.ready.shift_remove(cmd);
                self.base.shift_remove(cmd);
                Ok(old)
            }
            Some((name, func)) => {
                if self.ready.contains_key(cmd) {
                    return Err(ConfigError(format!(
                        "gcode command {} already registered",
                        cmd
                    )));
                }
                let stored: GcodeHandler = if is_traditional_gcode(cmd) {
                    func
                } else {
                    validate_extended_name(cmd)?;
                    let orig = func;
                    Rc::new(move |gcmd: &mut GCodeCommand| -> Result<(), CommandError> {
                        gcmd.apply_extended_params()?;
                        orig(gcmd)
                    })
                };
                let cmd_s = cmd.to_string();
                let _ = name;
                self.ready.insert(cmd_s.clone(), stored.clone());
                if when_not_ready {
                    self.base.insert(cmd_s.clone(), stored);
                }
                if let Some(d) = desc {
                    self.help.insert(cmd_s, d.to_string());
                }
                Ok(None)
            }
        }
    }

    /// `register_mux_command(cmd, key, value, func, desc)` (lines 158-173).
    /// `value = None` is the Python-`None` default arm.
    pub fn register_mux_command(
        &mut self,
        cmd: &str,
        key: &str,
        value: Option<String>,
        handler_name: String,
        func: GcodeHandler,
        desc: Option<&str>,
    ) -> Result<(), ConfigError> {
        if !self.mux.contains_key(cmd) {
            let mux_cmd = cmd.to_string();
            let key_s = key.to_string();
            let dispatcher: GcodeHandler = Rc::new(move |_gcmd: &mut GCodeCommand| {
                let _ = (&mux_cmd, &key_s);
                // Real dispatch clones out through `cmd_mux` below; this
                // placeholder is replaced by the registered entry.
                Err(CommandError(format!(
                    "mux dispatcher without values for {}",
                    mux_cmd
                )))
            });
            // Replace with a no-op that will be resolved per-call by the
            // dispatch loop (`dispatch_mux`). Keep the entry so duplicate
            // detection matches the Python lambda registration.
            self.register_command(
                cmd,
                Some(("mux".to_string(), dispatcher)),
                false,
                desc,
            )?;
            self.mux.insert(
                cmd.to_string(),
                (key.to_string(), IndexMap::new(), IndexMap::new()),
            );
        }
        let (prev_key, values, value_names) =
            self.mux.get_mut(cmd).expect("mux entry just inserted");
        if prev_key != key {
            return Err(ConfigError(format!(
                "mux command {} {} {} may have only one key ({})",
                cmd,
                key,
                value.as_deref().unwrap_or("None"),
                prev_key
            )));
        }
        if values.contains_key(&value) {
            let repr = python_dict_repr(value_names);
            return Err(ConfigError(format!(
                "mux command {} {} {} already registered ({})",
                cmd,
                key,
                value.as_deref().unwrap_or("None"),
                repr
            )));
        }
        value_names.insert(value.clone(), handler_name);
        values.insert(value, func);
        Ok(())
    }

    /// `_cmd_mux` (lines 317-336) with the exact error text.
    pub fn cmd_mux(
        &self,
        command: &str,
        gcmd: &mut GCodeCommand,
    ) -> Result<(), CommandError> {
        let (key, values, _) = match self.mux.get(command) {
            Some(v) => v,
            None => return Err(CommandError(format!("no mux values for {}", command))),
        };
        let key_param: Option<String> = if values.contains_key(&None) {
            gcmd.get_str_opt(key, Some(None))?
        } else {
            Some(gcmd.get_str(key, None)?)
        };
        if !values.contains_key(&key_param) {
            let shown = key_param.as_deref().unwrap_or("None");
            let mut keys: Vec<String> = Vec::new();
            let mut guess = String::new();
            for v in values.keys() {
                let Some(vs) = v else { continue };
                keys.push(format!("'{}'", vs));
                if let Some(kp) = &key_param {
                    if !kp.is_empty() && vs.contains(kp.as_str()) {
                        guess = format!(". Did you mean '{}'?", vs);
                    }
                }
            }
            if guess.is_empty() {
                guess = format!(". Options: {}", keys.join(", "));
            }
            return Err(CommandError(format!(
                "The value '{}' is not valid for {}{}",
                shown, key, guess
            )));
        }
        let func = values
            .get(&key_param)
            .expect("checked above")
            .clone();
        func(gcmd)
    }
}
impl Default for Dispatch {
    fn default() -> Self {
        Self::new()
    }
}


// `M117`/`M118`/`M23` redispatch, `M104`/`M140` off, `M106`/`M107` off,
// `M105`, `M21`, empty and unknown branches of `cmd_default`
// (lines 283-316), without the printer-ready surroundings.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DefaultAction {
    AckTemp,
    Ignore,
    Redispatch(String),
    Unknown(String),
}

pub fn classify_default(
    cmd: &str,
    params: &IndexMap<String, String>,
    commandline: &str,
    is_fileinput: bool,
) -> DefaultAction {
    if cmd == "M105" {
        return DefaultAction::AckTemp;
    }
    if cmd == "M21" {
        return DefaultAction::Ignore;
    }
    let mut holder: IndexMap<String, String> = IndexMap::new();
    for (k, v) in params {
        holder.insert(k.clone(), v.clone());
    }
    let probe = GCodeCommand::new(cmd.to_string(), commandline.to_string(), holder, false);
    if cmd.contains(' ') {
        let real = cmd.split_whitespace().next().unwrap_or("");
        if ["M117", "M118", "M23"].contains(&real) {
            return DefaultAction::Redispatch(real.to_string());
        }
        return DefaultAction::Unknown(cmd.to_string());
    }
    if (cmd == "M140" || cmd == "M104") && probe.get_float("S", Some(0.0), None, None, None, None).unwrap_or(0.0) == 0.0 {
        return DefaultAction::Ignore;
    }
    if cmd == "M107" {
        return DefaultAction::Ignore;
    }
    if cmd == "M106"
        && (probe.get_float("S", Some(1.0), None, None, None, None).unwrap_or(1.0) == 0.0 || is_fileinput)
    {
        return DefaultAction::Ignore;
    }
    if cmd.is_empty() {
        return DefaultAction::Ignore;
    }
    DefaultAction::Unknown(cmd.to_string())
}

/// `respond_info` text (lines 250-254) and the `Unknown command` text.
pub fn format_unknown(cmd: &str) -> String {
    format!("Unknown command:\"{}\"", cmd)
}

pub fn format_info(msg: &str) -> String {
    let lines: Vec<&str> = msg.trim().split('\n').map(|l| l.trim()).collect();
    format!("// {}", lines.join("\n// "))
}
