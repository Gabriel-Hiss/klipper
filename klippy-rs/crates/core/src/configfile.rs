//! Port of `klippy/configfile.py`.
//!
//! Covers the whole unit: reading the main config with `[include]` file
//! support, the autosave block split and `SAVE_CONFIG` rewrite, typed
//! [`ConfigWrapper`] access with access tracking, the unused-options
//! check, and the deprecation warning store.
//!
//! The reader, get wrappers and autosave helper reproduce
//! `RawConfigParser(strict=False, inline_comment_prefixes=(';' , '#'))`
//! semantics through [`pytext::configparser`] and Python number semantics
//! through [`pycompat::text`]. Every error message equals the Python
//! `str(exc)` text.
//!
//! Print-time text is exact. Two host integrations are explicit inputs
//! rather than implicit globals, because the later `klippy` crate owns them:
//! - `SAVE_CONFIG` timestamp: the caller passes the `datestr`
//!   (`time.strftime("-%Y%m%d_%H%M%S")` in Python). [`datestr_from_wall_time`]
//!   provides a UTC rendering from `kos::time::wall_time`.
//! - `gcode.request_restart('restart')`: after a successful
//!   [`ConfigAutoSave::save_config`], [`ConfigAutoSave::restart_requested`]
//!   is set for the host loop to act on.
//! - `PrinterConfig::check_unused_options` takes the valid section names
//!   (`printer.lookup_objects()`) as an argument.
//! - Deprecate version defaults for MCU codes take the version strings
//!   as arguments.
//!
//! `ConfigFileReader` additionally accepts extra mount prefixes used by
//! the parity harness to map absolute include paths (e.g. `/usr/data`) to
//! a scenario directory. Production passes no mounts and reads the real
//! filesystem.

use std::cell::RefCell;
use std::cmp::Ordering;
use std::fs;
use std::path::PathBuf;
use std::rc::Rc;

use indexmap::IndexMap;
use pycompat::{PyInt, PyValue};
use pycompat::text::{cmp_bigint_float, float_from_string, float_str, int_from_string};
use pytext::configparser::{Error as ConfigError, RawConfigParser};

/// The exact autosave marker, including the leading and trailing newline.
pub const AUTOSAVE_HEADER: &str =
    "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n#*#\n";
/// Help text registered for the `SAVE_CONFIG` g-code command.
pub const SAVE_CONFIG_HELP: &str = "Overwrite config file and restart";

// ---------------------------------------------------------------------------
// Path helpers mirroring `os.path` on POSIX.
// ---------------------------------------------------------------------------

/// `os.path.dirname(path)`.
pub fn py_dirname(path: &str) -> String {
    let trimmed = path.trim_end_matches('/');
    if trimmed.is_empty() {
        return if path.starts_with('/') { "/".to_string() } else { String::new() };
    }
    match trimmed.rfind('/') {
        Some(0) => "/".to_string(),
        Some(idx) => trimmed[..idx].to_string(),
        None => String::new(),
    }
}

/// `os.path.join(dir, spec)`.
pub fn py_join(dirname: &str, spec: &str) -> String {
    if spec.starts_with('/') {
        return spec.to_string();
    }
    if dirname.is_empty() {
        return spec.to_string();
    }
    if dirname.ends_with('/') {
        format!("{dirname}{spec}")
    } else {
        format!("{dirname}/{spec}")
    }
}

/// `os.path.abspath(path)` (no symlink resolution, like the Python one).
pub fn py_abspath(path: &str) -> String {
    let combined: PathBuf = if PathBuf::from(path).is_absolute() {
        PathBuf::from(path)
    } else {
        std::env::current_dir()
            .unwrap_or_else(|_| PathBuf::from("."))
            .join(path)
    };
    let absolute = combined.to_string_lossy().into_owned();
    let mut parts: Vec<&str> = Vec::new();
    for part in absolute.split('/') {
        match part {
            "" | "." => {}
            ".." => {
                parts.pop();
            }
            other => parts.push(other),
        }
    }
    format!("/{}", parts.join("/"))
}

/// `glob.has_magic(spec)`: any of `*`, `?`, `[`.
pub fn has_glob_magic(spec: &str) -> bool {
    spec.chars().any(|c| c == '*' || c == '?' || c == '[')
}

/// `fnmatch` name matching for one path component (`*`, `?`, `[seq]`,
/// `[!seq]`, ranges inside brackets). Backslash is a literal character.
fn bracket_match(chars: &[char], start: usize, subject: char) -> Option<(usize, bool)> {
    // chars[start] == '['
    let mut j = start + 1;
    if j >= chars.len() {
        return None;
    }
    let negated = chars[j] == '!';
    if negated {
        j += 1;
        if j >= chars.len() {
            return None;
        }
    }
    if chars[j] == ']' {
        j += 1; // a `]` right after `[` or `[!` is a literal member
    }
    let mut k = j;
    while k < chars.len() && chars[k] != ']' {
        k += 1;
    }
    if k >= chars.len() {
        return None; // no closing bracket: `[` is literal
    }
    let content = &chars[j..k];
    let mut matched = false;
    let mut i = 0;
    while i < content.len() {
        if i + 2 < content.len() && content[i + 1] == '-' {
            let lo = content[i];
            let hi = content[i + 2];
            if lo <= subject && subject <= hi {
                matched = true;
            }
            i += 3;
        } else {
            if content[i] == subject {
                matched = true;
            }
            i += 1;
        }
    }
    Some((k - start + 1, matched != negated))
}

fn fnmatch_name(pattern: &str, name: &str) -> bool {
    let p: Vec<char> = pattern.chars().collect();
    let s: Vec<char> = name.chars().collect();
    let (mut pi, mut si) = (0usize, 0usize);
    let mut star: Option<usize> = None;
    let mut star_si = 0usize;
    while si < s.len() {
        if pi < p.len() {
            match p[pi] {
                '*' => {
                    star = Some(pi);
                    star_si = si;
                    pi += 1;
                    continue;
                }
                '?' => {
                    pi += 1;
                    si += 1;
                    continue;
                }
                '[' => {
                    if let Some((consumed, matched)) = bracket_match(&p, pi, s[si]) {
                        if matched {
                            pi += consumed;
                            si += 1;
                            continue;
                        }
                    } else if s[si] == '[' {
                        pi += 1;
                        si += 1;
                        continue;
                    }
                }
                c if c == s[si] => {
                    pi += 1;
                    si += 1;
                    continue;
                }
                _ => {}
            }
        }
        if let Some(star_idx) = star {
            pi = star_idx + 1;
            star_si += 1;
            si = star_si;
            continue;
        }
        return false;
    }
    while pi < p.len() && p[pi] == '*' {
        pi += 1;
    }
    pi == p.len()
}

fn join_base(base: &str, name: &str) -> String {
    if base == "." {
        name.to_string()
    } else if base.ends_with('/') {
        format!("{base}{name}")
    } else {
        format!("{base}/{name}")
    }
}

/// `glob.glob(pattern)` restricted to the documented klippy use:
/// absolute or cwd-relative patterns, magic per component.
fn glob_local(pattern: &str) -> Vec<String> {
    if pattern.is_empty() {
        return Vec::new();
    }
    let absolute = pattern.starts_with('/');
    let comps: Vec<&str> = pattern.split('/').collect();
    let last_idx = comps.iter().rposition(|c| !c.is_empty()).unwrap_or(comps.len());
    let mut bases = vec![if absolute { "/".to_string() } else { ".".to_string() }];
    for (idx, comp) in comps.iter().enumerate() {
        if comp.is_empty() {
            continue;
        }
        let is_last = idx == last_idx;
        if has_glob_magic(comp) {
            let mut next = Vec::new();
            for base in &bases {
                let Ok(entries) = fs::read_dir(base) else {
                    continue;
                };
                for entry in entries.flatten() {
                    let name = entry.file_name().to_string_lossy().into_owned();
                    if fnmatch_name(comp, &name) {
                        let candidate = join_base(base, &name);
                        if is_last || entry.path().is_dir() {
                            next.push(candidate);
                        }
                    }
                }
            }
            bases = next;
        } else {
            let mut next = Vec::new();
            for base in &bases {
                let candidate = join_base(base, comp);
                let meta = fs::metadata(&candidate);
                if is_last {
                    if meta.is_ok() {
                        next.push(candidate);
                    }
                } else if meta.map(|m| m.is_dir()).unwrap_or(false) {
                    next.push(candidate);
                }
            }
            bases = next;
        }
    }
    bases
}

/// Extra mount prefixes used to translate absolute paths for harness runs.
#[derive(Clone, Default, Debug)]
pub struct Mounts {
    /// `(requested absolute prefix, local root)`, longest match wins.
    pub entries: Vec<(String, PathBuf)>,
}

impl Mounts {
    pub fn new(entries: Vec<(String, PathBuf)>) -> Self {
        Mounts { entries }
    }

    fn find(&self, path: &str) -> Option<&(String, PathBuf)> {
        let mut best: Option<&(String, PathBuf)> = None;
        for entry in &self.entries {
            let (prefix, _) = entry;
            let under = path == prefix.as_str()
                || match path.strip_prefix(prefix.as_str()) {
                    Some(rest) => rest.starts_with('/'),
                    None => false,
                };
            if under && best.map(|b| prefix.len() > b.0.len()).unwrap_or(true) {
                best = Some(entry);
            }
        }
        best
    }

    /// Map a requested path to the path on this host filesystem.
    fn to_local(&self, requested: &str) -> String {
        match self.find(requested) {
            Some((prefix, local)) => {
                let rest = requested[prefix.len()..].trim_start_matches('/');
                let path = if rest.is_empty() {
                    local.clone()
                } else {
                    local.join(rest)
                };
                path.to_string_lossy().into_owned()
            }
            None => requested.to_string(),
        }
    }

    /// Map a glob result from the local form back to the requested form.
    fn to_requested(&self, local: &str) -> String {
        for (prefix, root) in &self.entries {
            let root_s = root.to_string_lossy();
            if local == root_s.as_ref()
                || local
                    .strip_prefix(root_s.as_ref())
                    .map(|rest| rest.starts_with('/'))
                    .unwrap_or(false)
            {
                let rest = &local[root_s.len()..];
                return format!("{prefix}{rest}");
            }
        }
        local.to_string()
    }

    /// `glob.glob(include_glob)` in requested path form (unsorted, as Python).
    pub fn glob(&self, requested_pattern: &str) -> Vec<String> {
        let local = self.to_local(requested_pattern);
        glob_local(&local)
            .into_iter()
            .map(|l| self.to_requested(&l))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// ConfigFileReader
// ---------------------------------------------------------------------------

/// Port of `ConfigFileReader`: file reading, `#` stripping, parsing and
/// include expansion.
#[derive(Debug, Clone)]
pub struct ConfigFileReader {
    /// Mounts translating requested paths (empty in production).
    pub mounts: Mounts,
    /// Log messages emitted by reads (plain message text).
    pub log_lines: Vec<String>,
}

impl Default for ConfigFileReader {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigFileReader {
    pub fn new() -> Self {
        ConfigFileReader {
            mounts: Mounts::default(),
            log_lines: Vec::new(),
        }
    }

    pub fn with_mounts(mounts: Mounts) -> Self {
        ConfigFileReader {
            mounts,
            log_lines: Vec::new(),
        }
    }

    /// `read_config_file(filename)` (universal newlines: `\r\n` and `\r`
    /// become `\n`).
    pub fn read_config_file(&mut self, filename: &str) -> Result<String, ConfigError> {
        let local = self.mounts.to_local(filename);
        match fs::read_to_string(&local) {
            Ok(text) => Ok(text.replace("\r\n", "\n").replace('\r', "\n")),
            Err(_) => {
                let msg = format!("Unable to open config file {filename}");
                self.log_lines.push(msg.clone());
                Err(ConfigError::new(msg))
            }
        }
    }

    /// `build_config_string(fileconfig)`: `write()` then `strip()`.
    pub fn build_config_string(fileconfig: &RawConfigParser) -> String {
        fileconfig.write().trim().to_string()
    }

    /// `append_fileconfig` (trailing `#` comments cut before parsing).
    pub fn append_fileconfig(
        &mut self,
        fileconfig: &mut RawConfigParser,
        data: &str,
        filename: &str,
    ) -> Result<(), ConfigError> {
        if data.is_empty() {
            return Ok(());
        }
        let lines: Vec<String> = data
            .split('\n')
            .map(|line| match line.find('#') {
                Some(pos) => line[..pos].to_string(),
                None => line.to_string(),
            })
            .collect();
        fileconfig.read_file(&lines.join("\n"), filename)
    }

    /// `_create_fileconfig`.
    pub fn create_fileconfig() -> RawConfigParser {
        RawConfigParser::new_klippy()
    }

    /// `build_fileconfig(data, filename)` (no includes).
    pub fn build_fileconfig(
        data: &str,
        filename: &str,
    ) -> Result<RawConfigParser, ConfigError> {
        let mut fileconfig = Self::create_fileconfig();
        let mut reader = ConfigFileReader::new();
        reader.append_fileconfig(&mut fileconfig, data, filename)?;
        Ok(fileconfig)
    }

    /// `build_fileconfig_with_includes(data, filename)`.
    pub fn build_fileconfig_with_includes(
        &mut self,
        data: &str,
        filename: &str,
    ) -> Result<RawConfigParser, ConfigError> {
        let mut fileconfig = Self::create_fileconfig();
        let mut visited = Vec::new();
        self.parse_config(data, filename, &mut fileconfig, &mut visited)?;
        Ok(fileconfig)
    }

    fn resolve_include(
        &mut self,
        source_filename: &str,
        include_spec: &str,
        fileconfig: &mut RawConfigParser,
        visited: &mut Vec<String>,
    ) -> Result<Vec<String>, ConfigError> {
        let dirname = py_dirname(source_filename);
        let include_glob = py_join(&dirname, include_spec);
        let mut include_filenames = self.mounts.glob(&include_glob);
        if include_filenames.is_empty() && !has_glob_magic(&include_glob) {
            return Err(ConfigError::new(format!(
                "Include file '{include_glob}' does not exist"
            )));
        }
        include_filenames.sort();
        for include_filename in &include_filenames {
            let clone = include_filename.clone();
            let include_data = self.read_config_file(&clone)?;
            self.parse_config(&include_data, &clone, fileconfig, visited)?;
        }
        Ok(include_filenames)
    }

    fn parse_config(
        &mut self,
        data: &str,
        filename: &str,
        fileconfig: &mut RawConfigParser,
        visited: &mut Vec<String>,
    ) -> Result<(), ConfigError> {
        let path = py_abspath(filename);
        if visited.contains(&path) {
            return Err(ConfigError::new(format!(
                "Recursive include of config file '{filename}'"
            )));
        }
        visited.push(path.clone());
        let mut buf: Vec<String> = Vec::new();
        for raw_line in data.split('\n') {
            let line = match raw_line.find('#') {
                Some(pos) => raw_line[..pos].to_string(),
                None => raw_line.to_string(),
            };
            let header = sectcre_header(&line);
            let is_include = header
                .as_deref()
                .map(|h| h.starts_with("include "))
                .unwrap_or(false);
            if is_include {
                self.append_fileconfig(fileconfig, &buf.join("\n"), filename)?;
                buf.clear();
                let include_spec = header.unwrap()[8..].trim().to_string();
                self.resolve_include(filename, &include_spec, fileconfig, visited)?;
            } else {
                buf.push(line);
            }
        }
        self.append_fileconfig(fileconfig, &buf.join("\n"), filename)?;
        visited.retain(|p| p != &path);
        Ok(())
    }
}

/// `RawConfigParser.SECTCRE.match(line)` header group (greedy to last `]`).
fn sectcre_header(line: &str) -> Option<String> {
    if !line.starts_with('[') {
        return None;
    }
    let last = line.rfind(']')?;
    if last < 2 {
        return None;
    }
    let header = &line[1..last];
    if header.is_empty() || header.contains('\n') {
        return None;
    }
    Some(header.to_string())
}

// ---------------------------------------------------------------------------
// Typed access
// ---------------------------------------------------------------------------

/// A numeric bound for `getint`/`getfloat`, formatted with `%s` in errors.
#[derive(Clone, Debug)]
pub enum Bound {
    Int(PyInt),
    Float(f64),
}

impl Bound {
    pub fn int(v: i64) -> Bound {
        Bound::Int(PyInt::from_i64(v))
    }

    pub fn float(v: f64) -> Bound {
        Bound::Float(v)
    }

    /// `%s` text (`float_str` for floats, decimal for ints).
    pub fn to_display(&self) -> String {
        match self {
            Bound::Int(n) => n.inner().to_string(),
            Bound::Float(f) => float_str(*f),
        }
    }

    /// Exact comparison of `int_value` with the bound (`None` for NaN:
    /// every Python comparison against NaN is false).
    fn cmp_int(&self, value: &PyInt) -> Option<Ordering> {
        match self {
            Bound::Int(b) => Some(value.inner().cmp(b.inner())),
            Bound::Float(f) => {
                if f.is_nan() {
                    None
                } else if *f == f64::INFINITY {
                    Some(Ordering::Less)
                } else if *f == f64::NEG_INFINITY {
                    Some(Ordering::Greater)
                } else {
                    Some(cmp_bigint_float(value.inner(), *f))
                }
            }
        }
    }

    /// Exact comparison of `float_value` with the bound.
    fn cmp_float(&self, value: f64) -> Option<Ordering> {
        match self {
            Bound::Float(b) => value.partial_cmp(b),
            Bound::Int(b) => {
                if value.is_nan() {
                    None
                } else if value == f64::INFINITY {
                    Some(Ordering::Greater)
                } else if value == f64::NEG_INFINITY {
                    Some(Ordering::Less)
                } else {
                    Some(cmp_bigint_float(b.inner(), value).reverse())
                }
            }
        }
    }
}

/// Sentinel default: `None` means *"no default"* (missing option raises).
/// `Some(None)` is the default `None` (missing returns `None`, untracked);
/// `Some(Some(v))` is a real default value. Getter signatures use
/// `OptDefault<Option<X>>` for their value type `X`.
pub type OptDefault<T> = Option<T>;

/// A parsed list value (Python tuple).
#[derive(Clone, Debug, PartialEq)]
pub enum ListVal {
    Text(String),
    Int(PyInt),
    Float(f64),
    Nested(Vec<ListVal>),
}

impl ListVal {
    pub fn to_pyvalue(&self) -> PyValue {
        match self {
            ListVal::Text(s) => PyValue::Str(s.clone()),
            ListVal::Int(n) => PyValue::Int(n.clone()),
            ListVal::Float(f) => PyValue::Float(*f),
            ListVal::Nested(items) => {
                PyValue::Tuple(items.iter().map(ListVal::to_pyvalue).collect())
            }
        }
    }

    /// First innermost leaf, used by tests to infer the element parser.
    pub fn first_leaf(&self) -> Option<&ListVal> {
        match self {
            ListVal::Nested(items) => items.first().and_then(ListVal::first_leaf),
            leaf => Some(leaf),
        }
    }
}

impl std::fmt::Display for ListVal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_pyvalue().repr())
    }
}

/// Element parser for `getlists` (`str`, `int` or `float`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ListParser {
    Str,
    Int,
    Float,
}

/// A choice key (`None`, `int` or `str`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ChoiceKey {
    Null,
    Int(PyInt),
    Text(String),
}

impl ChoiceKey {
    fn to_pyvalue(&self) -> PyValue {
        match self {
            ChoiceKey::Null => PyValue::None,
            ChoiceKey::Int(n) => PyValue::Int(n.clone()),
            ChoiceKey::Text(s) => PyValue::Str(s.clone()),
        }
    }

}

/// Choices for `getchoice`: a list (converted to an identity map) or an
/// explicit map.
#[derive(Clone, Debug)]
pub enum ChoiceSet {
    List(Vec<ChoiceKey>),
    Map(Vec<(ChoiceKey, PyValue)>),
}

impl ChoiceSet {
    pub fn from_list(keys: Vec<ChoiceKey>) -> ChoiceSet {
        ChoiceSet::List(keys)
    }

    pub fn from_map(entries: Vec<(ChoiceKey, PyValue)>) -> ChoiceSet {
        ChoiceSet::Map(entries)
    }

    /// First-key-is-`int` test from `getchoice` (after list conversion).
    fn is_int_mode(&self) -> bool {
        let first = match self {
            ChoiceSet::List(keys) => keys.first(),
            ChoiceSet::Map(entries) => entries.first().map(|(k, _)| k),
        };
        matches!(first, Some(ChoiceKey::Int(_)))
    }

    fn lookup(&self, key: &ChoiceKey) -> Option<PyValue> {
        match self {
            ChoiceSet::List(keys) => {
                if keys.iter().any(|k| k == key) {
                    Some(key.to_pyvalue())
                } else {
                    None
                }
            }
            ChoiceSet::Map(entries) => entries
                .iter()
                .find(|(k, _)| k == key)
                .map(|(_, v)| v.clone()),
        }
    }
}

fn parse_error(section: &str, option: &str) -> ConfigError {
    ConfigError::new(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

/// `lparser` from `getlists`. Count errors are config errors; element
/// parse errors are reported as plain parse errors.
fn list_parse(
    value: &str,
    pos: usize,
    seps: &[String],
    count: Option<i64>,
    parser: ListParser,
    section: &str,
    option: &str,
) -> Result<ListVal, ConfigError> {
    let parts: Vec<String> = if value.trim().is_empty() {
        Vec::new()
    } else {
        value
            .split(seps[pos].as_str())
            .map(|p| p.trim().to_string())
            .collect()
    };
    if pos > 0 {
        let mut out = Vec::new();
        for p in parts.iter().filter(|p| !p.is_empty()) {
            out.push(list_parse(p, pos - 1, seps, count, parser, section, option)?);
        }
        return Ok(ListVal::Nested(out));
    }
    let mut res = Vec::new();
    for p in &parts {
        let item = match parser {
            ListParser::Str => ListVal::Text(p.clone()),
            ListParser::Int => match int_from_string(p, 10) {
                Ok(n) => ListVal::Int(n),
                Err(_) => return Err(parse_error(section, option)),
            },
            ListParser::Float => match float_from_string(p) {
                Ok(n) => ListVal::Float(n),
                Err(_) => return Err(parse_error(section, option)),
            },
        };
        res.push(item);
    }
    if let Some(n) = count {
        if res.len() as i64 != n {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{section}' must have {n} elements"
            )));
        }
    }
    Ok(ListVal::Nested(res))
}

/// Store handed to wrappers so `deprecate` reaches the printer config.
#[derive(Debug, Default)]
pub struct DeprecatedStore {
    seen: Vec<Vec<(String, String)>>,
    /// Deprecation warning dicts in insertion order.
    pub warnings: Vec<IndexMap<String, PyValue>>,
    /// Log messages (only `runtime_warning` logs).
    pub log_lines: Vec<String>,
}

fn py_str(v: &PyValue) -> String {
    match v {
        PyValue::Str(s) => s.clone(),
        other => other.repr(),
    }
}

fn dict_key(data: &IndexMap<String, PyValue>) -> Vec<(String, String)> {
    data.iter()
        .map(|(k, v)| {
            let tag = match v {
                PyValue::None => format!("N:{}", v.repr()),
                PyValue::Str(_) => format!("S:{}", v.repr()),
                other => format!("O:{}", other.repr()),
            };
            (k.clone(), tag)
        })
        .collect()
}

impl DeprecatedStore {
    pub fn new() -> Self {
        Self::default()
    }

    fn add(&mut self, data: IndexMap<String, PyValue>) -> bool {
        let key = dict_key(&data);
        if self.seen.contains(&key) {
            return false;
        }
        self.seen.push(key);
        self.warnings.push(data);
        true
    }
}

/// Typed config access for one section, sharing the file config and the
/// access-tracking store across wrappers.
#[derive(Clone)]
pub struct ConfigWrapper {
    inner: Rc<ConfigWrapperInner>,
}

struct ConfigWrapperInner {
    fileconfig: Rc<RawConfigParser>,
    access_tracking: Rc<RefCell<IndexMap<(String, String), PyValue>>>,
    deprecated: Rc<RefCell<DeprecatedStore>>,
    section: String,
}

impl ConfigWrapper {
    pub fn new(
        fileconfig: Rc<RawConfigParser>,
        access_tracking: Rc<RefCell<IndexMap<(String, String), PyValue>>>,
        deprecated: Rc<RefCell<DeprecatedStore>>,
        section: String,
    ) -> Self {
        ConfigWrapper {
            inner: Rc::new(ConfigWrapperInner {
                fileconfig,
                access_tracking,
                deprecated,
                section,
            }),
        }
    }

    /// `get_name()`.
    pub fn section_name(&self) -> &str {
        &self.inner.section
    }

    /// `fileconfig` (the shared parser).
    pub fn fileconfig(&self) -> &RawConfigParser {
        &self.inner.fileconfig
    }

    /// `getsection(section)`.
    pub fn getsection(&self, section: &str) -> ConfigWrapper {
        ConfigWrapper {
            inner: Rc::new(ConfigWrapperInner {
                fileconfig: Rc::clone(&self.inner.fileconfig),
                access_tracking: Rc::clone(&self.inner.access_tracking),
                deprecated: Rc::clone(&self.inner.deprecated),
                section: section.to_string(),
            }),
        }
    }

    /// `has_section(section)`.
    pub fn has_section(&self, section: &str) -> bool {
        self.inner.fileconfig.has_section(section)
    }

    /// `get_prefix_sections(prefix)`.
    pub fn get_prefix_sections(&self, prefix: &str) -> Vec<ConfigWrapper> {
        self.inner
            .fileconfig
            .sections()
            .into_iter()
            .filter(|s| s.starts_with(prefix))
            .map(|s| self.getsection(&s))
            .collect()
    }

    /// `get_prefix_options(prefix)`. Untracked. Raises `NoSectionError` for
    /// a missing section, as Python does.
    pub fn get_prefix_options(&self, prefix: &str) -> Result<Vec<String>, ConfigError> {
        let opts = self.inner.fileconfig.options(&self.inner.section)?;
        Ok(opts.into_iter().filter(|o| o.starts_with(prefix)).collect())
    }

    /// `deprecate(option, value=None)`.
    pub fn deprecate(&self, option: &str, value: Option<PyValue>) {
        {
            let fconfig = &self.inner.fileconfig;
            if !fconfig.has_option(&self.inner.section, option) {
                return;
            }
        }
        let (mut res, msg) = match value {
            None => {
                let mut res = IndexMap::new();
                res.insert("type".to_string(), PyValue::Str("deprecated_option".to_string()));
                let msg = format!(
                    "Option '{}' in section '{}' is deprecated.",
                    option, self.inner.section
                );
                (res, msg)
            }
            Some(v) => {
                let mut res = IndexMap::new();
                res.insert("type".to_string(), PyValue::Str("deprecated_value".to_string()));
                res.insert("value".to_string(), v.clone());
                let msg = format!(
                    "Value '{}' in option '{}' in section '{}' is deprecated.",
                    py_str(&v),
                    option,
                    self.inner.section
                );
                (res, msg)
            }
        };
        res.insert("message".to_string(), PyValue::Str(msg));
        res.insert("section".to_string(), PyValue::Str(self.inner.section.clone()));
        res.insert("option".to_string(), PyValue::Str(option.to_string()));
        self.inner.deprecated.borrow_mut().add(res);
    }

    fn track(&self, option: &str, value: PyValue) {
        self.inner
            .access_tracking
            .borrow_mut()
            .insert(
                (self.inner.section.to_lowercase(), option.to_lowercase()),
                value,
            );
    }

    fn missing_error(&self, option: &str) -> ConfigError {
        ConfigError::new(format!(
            "Option '{option}' in section '{}' must be specified",
            self.inner.section
        ))
    }

    /// `_get_wrapper` core: missing/default handling, parser call, tracking.
    fn get_typed<T: Clone>(
        &self,
        option: &str,
        default: &OptDefault<T>,
        note_valid: bool,
        is_none_default: &dyn Fn(&T) -> bool,
        to_pyvalue: &dyn Fn(&T) -> PyValue,
        parser: impl FnOnce() -> Result<T, ConfigError>,
    ) -> Result<T, ConfigError> {
        if !self
            .inner
            .fileconfig
            .has_option(&self.inner.section, option)
        {
            let Some(d) = default else {
                return Err(self.missing_error(option));
            };
            if note_valid && !is_none_default(d) {
                self.track(option, to_pyvalue(d));
            }
            return Ok(d.clone());
        }
        let v = parser()?;
        if note_valid {
            self.track(option, to_pyvalue(&v));
        }
        Ok(v)
    }

    /// `get(option, default, note_valid)`.
    pub fn get(
        &self,
        option: &str,
        default: &OptDefault<Option<String>>,
        note_valid: bool,
    ) -> Result<Option<String>, ConfigError> {
        let section = self.inner.section.clone();
        let fconfig = Rc::clone(&self.inner.fileconfig);
        self.get_typed(
            option,
            default,
            note_valid,
            &|d| d.is_none(),
            &|d| PyValue::Str(d.clone().unwrap_or_default()),
            move || {
                let opt = option.to_string();
                match fconfig.get(&section, &opt) {
                    Ok(Some(v)) => Ok(Some(v)),
                    ok @ Ok(None) => Ok(ok.unwrap()),
                    Err(e) => {
                        if let ConfigError::ValueError(_) = e {
                            Err(parse_error(&section, &opt))
                        } else {
                            Err(e)
                        }
                    }
                }
            },
        )
    }

    /// `getint(option, default, minval, maxval, note_valid)`.
    pub fn getint(
        &self,
        option: &str,
        default: &OptDefault<Option<PyInt>>,
        minval: Option<Bound>,
        maxval: Option<Bound>,
        note_valid: bool,
    ) -> Result<Option<PyInt>, ConfigError> {
        let section = self.inner.section.clone();
        // `_get_wrapper` returns a defaulted value before any bound check.
        let from_file = self.inner.fileconfig.has_option(&section, option);
        let closure_section = section.clone();
        let fconfig = Rc::clone(&self.inner.fileconfig);
        let v = self.get_typed(
            option,
            default,
            note_valid,
            &|d| d.is_none(),
            &|d| PyValue::Int(d.clone().unwrap_or_else(|| PyInt::from_i64(0))),
            move || {
                let opt = option.to_string();
                let raw = match fconfig.get(&closure_section, &opt) {
                    Ok(Some(v)) => v,
                    Ok(None) => return Err(parse_error(&closure_section, &opt)),
                    Err(e) => {
                        if let ConfigError::ValueError(_) = e {
                            return Err(parse_error(&closure_section, &opt));
                        }
                        return Err(e);
                    }
                };
                int_from_string(&raw, 10).map_err(|_| parse_error(&closure_section, &opt)).map(Some)
            },
        )?;
        if let (true, Some(n)) = (from_file, &v) {
            if let Some(b) = &minval {
                if b.cmp_int(n) == Some(Ordering::Greater) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must have minimum of {}",
                        b.to_display()
                    )));
                }
            }
            if let Some(b) = &maxval {
                if b.cmp_int(n) == Some(Ordering::Less) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must have maximum of {}",
                        b.to_display()
                    )));
                }
            }
        }
        Ok(v)
    }

    /// `getfloat(option, default, minval, maxval, above, below, note_valid)`.
    #[allow(clippy::too_many_arguments)]
    pub fn getfloat(
        &self,
        option: &str,
        default: &OptDefault<Option<f64>>,
        minval: Option<Bound>,
        maxval: Option<Bound>,
        above: Option<Bound>,
        below: Option<Bound>,
        note_valid: bool,
    ) -> Result<Option<f64>, ConfigError> {
        let section = self.inner.section.clone();
        // `_get_wrapper` returns a defaulted value before any bound check.
        let from_file = self.inner.fileconfig.has_option(&section, option);
        let closure_section = section.clone();
        let fconfig = Rc::clone(&self.inner.fileconfig);
        let v = self.get_typed(
            option,
            default,
            note_valid,
            &|d| d.is_none(),
            &|d| PyValue::Float(d.unwrap_or(0.0)),
            move || {
                let opt = option.to_string();
                let raw = match fconfig.get(&closure_section, &opt) {
                    Ok(Some(v)) => v,
                    Ok(None) => return Err(parse_error(&closure_section, &opt)),
                    Err(e) => {
                        if let ConfigError::ValueError(_) = e {
                            return Err(parse_error(&closure_section, &opt));
                        }
                        return Err(e);
                    }
                };
                float_from_string(&raw).map_err(|_| parse_error(&closure_section, &opt)).map(Some)
            },
        )?;
        if let (true, Some(n)) = (from_file, v) {
            if let Some(b) = &minval {
                if b.cmp_float(n) == Some(Ordering::Greater) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must have minimum of {}",
                        b.to_display()
                    )));
                }
            }
            if let Some(b) = &maxval {
                if b.cmp_float(n) == Some(Ordering::Less) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must have maximum of {}",
                        b.to_display()
                    )));
                }
            }
            if let Some(b) = &above {
                if matches!(b.cmp_float(n), Some(Ordering::Less) | Some(Ordering::Equal)) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must be above {}",
                        b.to_display()
                    )));
                }
            }
            if let Some(b) = &below {
                if matches!(b.cmp_float(n), Some(Ordering::Greater) | Some(Ordering::Equal)) {
                    return Err(ConfigError::new(format!(
                        "Option '{option}' in section '{section}' must be below {}",
                        b.to_display()
                    )));
                }
            }
        }
        Ok(v)
    }

    /// `getboolean(option, default, note_valid)`.
    pub fn getboolean(
        &self,
        option: &str,
        default: &OptDefault<Option<bool>>,
        note_valid: bool,
    ) -> Result<Option<bool>, ConfigError> {
        let section = self.inner.section.clone();
        let fconfig = Rc::clone(&self.inner.fileconfig);
        self.get_typed(
            option,
            default,
            note_valid,
            &|d| d.is_none(),
            &|d| PyValue::Bool(d.unwrap_or(false)),
            move || {
                let opt = option.to_string();
                let raw = match fconfig.get(&section, &opt) {
                    Ok(Some(v)) => v,
                    Ok(None) => return Err(parse_error(&section, &opt)),
                    Err(e) => {
                        if let ConfigError::ValueError(_) = e {
                            return Err(parse_error(&section, &opt));
                        }
                        return Err(e);
                    }
                };
                match raw.to_lowercase().as_str() {
                    "1" | "yes" | "true" | "on" => Ok(Some(true)),
                    "0" | "no" | "false" | "off" => Ok(Some(false)),
                    _ => Err(parse_error(&section, &opt)),
                }
            },
        )
    }

    /// `getchoice(option, choices, default, note_valid)`. The default is a
    /// plain `PyValue` (sentinel when the outer `Option` is `None`).
    pub fn getchoice(
        &self,
        option: &str,
        choices: &ChoiceSet,
        default: &Option<PyValue>,
        note_valid: bool,
    ) -> Result<PyValue, ConfigError> {
        let int_mode = choices.is_int_mode();
        let key: ChoiceKey;
        if !self
            .inner
            .fileconfig
            .has_option(&self.inner.section, option)
        {
            let Some(d) = default else {
                return Err(self.missing_error(option));
            };
            if note_valid && !matches!(d, PyValue::None) {
                self.track(option, d.clone());
            }
            key = match d {
                PyValue::None => ChoiceKey::Null,
                PyValue::Int(n) => ChoiceKey::Int(n.clone()),
                PyValue::Str(s) => ChoiceKey::Text(s.clone()),
                other => ChoiceKey::Text(py_str(other)),
            };
        } else if int_mode {
            match self.getint(
                option,
                &Some(None),
                None,
                None,
                note_valid,
            )? {
                Some(n) => key = ChoiceKey::Int(n),
                None => key = ChoiceKey::Null,
            }
        } else {
            match self.get(option, &Some(None), note_valid)? {
                Some(s) => key = ChoiceKey::Text(s),
                None => key = ChoiceKey::Null,
            }
        }
        match choices.lookup(&key) {
            Some(v) => Ok(v),
            None => {
                let c = match &key {
                    ChoiceKey::Null => "None".to_string(),
                    ChoiceKey::Int(n) => n.inner().to_string(),
                    ChoiceKey::Text(s) => s.clone(),
                };
                Err(ConfigError::new(format!(
                    "Choice '{c}' for option '{option}' in section '{}' is not a valid choice",
                    self.inner.section
                )))
            }
        }
    }

    /// `getlists(option, default, seps, count, parser, note_valid)`.
    pub fn getlists(
        &self,
        option: &str,
        default: &OptDefault<Option<ListVal>>,
        seps: &[String],
        count: Option<i64>,
        parser: ListParser,
        note_valid: bool,
    ) -> Result<Option<ListVal>, ConfigError> {
        let section = self.inner.section.clone();
        let fconfig = Rc::clone(&self.inner.fileconfig);
        let seps_owned = if seps.is_empty() { vec![",".to_string()] } else { seps.to_vec() };
        self.get_typed(
            option,
            default,
            note_valid,
            &|d| d.is_none(),
            &|d| d.clone().unwrap_or(ListVal::Nested(Vec::new())).to_pyvalue(),
            move || {
                let opt = option.to_string();
                let raw = match fconfig.get(&section, &opt) {
                    Ok(Some(v)) => v,
                    Ok(None) => return Err(parse_error(&section, &opt)),
                    Err(e) => {
                        if let ConfigError::ValueError(_) = e {
                            return Err(parse_error(&section, &opt));
                        }
                        return Err(e);
                    }
                };
                let last = seps_owned.len().saturating_sub(1);
                list_parse(
                    &raw,
                    last,
                    &seps_owned,
                    count,
                    parser,
                    &section,
                    &opt,
                )
                .map(Some)
            },
        )
    }

    /// `getlist(option, default, sep, count, note_valid)`.
    pub fn getlist(
        &self,
        option: &str,
        default: &OptDefault<Option<ListVal>>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> Result<Option<ListVal>, ConfigError> {
        let seps = vec![sep.to_string()];
        self.getlists(option, default, &seps, count, ListParser::Str, note_valid)
    }

    /// `getintlist(option, default, sep, count, note_valid)`.
    pub fn getintlist(
        &self,
        option: &str,
        default: &OptDefault<Option<ListVal>>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> Result<Option<ListVal>, ConfigError> {
        let seps = vec![sep.to_string()];
        self.getlists(option, default, &seps, count, ListParser::Int, note_valid)
    }

    /// `getfloatlist(option, default, sep, count, note_valid)`.
    pub fn getfloatlist(
        &self,
        option: &str,
        default: &OptDefault<Option<ListVal>>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> Result<Option<ListVal>, ConfigError> {
        let seps = vec![sep.to_string()];
        self.getlists(option, default, &seps, count, ListParser::Float, note_valid)
    }
}


// ---------------------------------------------------------------------------
// Autosave
// ---------------------------------------------------------------------------

/// Result of a successful `SAVE_CONFIG` rewrite.
#[derive(Clone, Debug)]
pub struct SaveConfigResult {
    /// Full file text after the rewrite.
    pub data: String,
    pub backup_name: String,
    pub temp_name: String,
}

/// Split the file at the first autosave header.
///
/// Returns `(regular_data, autosave_data, log_lines)`. Line prefixes
/// `#*# ` are stripped from autosave lines.
pub fn find_autosave_data(data: &str) -> (String, String, Vec<String>) {
    let mut regular_data = data.to_string();
    let mut autosave_data = String::new();
    let mut log_lines = Vec::new();
    if let Some(pos) = data.find(AUTOSAVE_HEADER) {
        regular_data = data[..pos].to_string();
        autosave_data = data[pos + AUTOSAVE_HEADER.len()..].trim().to_string();
    }
    if regular_data.contains("\n#*# ") || autosave_data.contains(AUTOSAVE_HEADER) {
        log_lines.push(
            "Can't read autosave from config file - autosave state corrupted".to_string(),
        );
        return (data.to_string(), String::new(), log_lines);
    }
    for line in autosave_data.split('\n') {
        let corrupted_line = (!line.starts_with("#*#")
            || (line.chars().count() >= 4 && !line.starts_with("#*# ")))
            && !autosave_data.is_empty();
        if corrupted_line {
            log_lines.push(
                "Can't read autosave from config file - modifications after header".to_string(),
            );
            return (data.to_string(), String::new(), log_lines);
        }
    }
    let mut out: Vec<String> = vec![String::new()];
    for line in autosave_data.split('\n') {
        out.push(line.chars().skip(4).collect());
    }
    out.push(String::new());
    (regular_data, out.join("\n"), log_lines)
}

/// `_strip_duplicates`: comment out fields of `data` already present in
/// `fileconfig` (including their continuation lines).
pub fn strip_duplicates(data: &str, fileconfig: &RawConfigParser) -> String {
    let mut lines: Vec<String> = data.split('\n').map(|s| s.to_string()).collect();
    let mut section: Option<String> = None;
    let mut is_dup_field = false;
    for (lineno, line) in lines.clone().into_iter().enumerate() {
        let mut pruned = match line.find(['#', ';']) {
            Some(pos) => line[..pos].to_string(),
            None => line.clone(),
        };
        while pruned.ends_with(char::is_whitespace) {
            pruned.pop();
        }
        if pruned.is_empty() {
            continue;
        }
        let first = pruned.chars().next().unwrap();
        if first.is_whitespace() {
            if is_dup_field {
                lines[lineno] = format!("#{line}");
            }
            continue;
        }
        is_dup_field = false;
        if first == '[' {
            let chars: Vec<char> = pruned.chars().collect();
            if chars.len() >= 2 {
                let inner: String = chars[1..chars.len() - 1].iter().collect();
                section = Some(inner.trim().to_string());
            } else {
                section = Some(String::new());
            }
            continue;
        }
        let field: String = pruned
            .chars()
            .take_while(|c| c.is_ascii_alphanumeric() || *c == '_')
            .collect();
        // Python passes the (possibly None) section through; a section that
        // cannot exist maps to a sentinel that matches nothing.
        let sect = section.clone().unwrap_or_else(|| "\u{0}klippy-none".to_string());
        if fileconfig.has_option(&sect, &field) {
            is_dup_field = true;
            lines[lineno] = format!("#{line}");
        }
    }
    lines.join("\n")
}

/// `ConfigAutoSave` port (command registration and restart delivery are
/// host-side; see the module docs).
pub struct ConfigAutoSave {
    /// Autosave-only parser (overrides).
    pub fileconfig: RawConfigParser,
    pub status_save_pending: IndexMap<String, Option<IndexMap<String, String>>>,
    pub save_config_pending: bool,
    pub config_file: String,
    pub mounts: Mounts,
    /// Log messages emitted by autosave operations.
    pub log_lines: Vec<String>,
    /// Set after a successful `SAVE_CONFIG` (the host restarts the printer).
    pub restart_requested: bool,
}

impl ConfigAutoSave {
    pub fn new(config_file: String, mounts: Mounts) -> Self {
        ConfigAutoSave {
            fileconfig: RawConfigParser::new_klippy(),
            status_save_pending: IndexMap::new(),
            save_config_pending: false,
            config_file,
            mounts,
            log_lines: Vec::new(),
            restart_requested: false,
        }
    }

    /// `_find_autosave_data` (logs warnings on corruption).
    fn find_autosave_data(&mut self, data: &str) -> (String, String) {
        let (regular, autosave, logs) = find_autosave_data(data);
        self.log_lines.extend(logs);
        (regular, autosave)
    }

    /// `load_main_config`.
    pub fn load_main_config(&mut self) -> Result<(RawConfigParser, RawConfigParser), ConfigError> {
        let filename = self.config_file.clone();
        let mut reader = ConfigFileReader::with_mounts(self.mounts.clone());
        let data = reader.read_config_file(&filename)?;
        let (regular_data, autosave_data) = self.find_autosave_data(&data);
        let mut regular_fileconfig =
            reader.build_fileconfig_with_includes(&regular_data, &filename)?;
        let stripped = strip_duplicates(&autosave_data, &regular_fileconfig);
        self.fileconfig = ConfigFileReader::build_fileconfig(&stripped, &filename)?;
        reader.append_fileconfig(&mut regular_fileconfig, &stripped, "*AUTOSAVE*")?;
        self.log_lines.extend(reader.log_lines);
        Ok((regular_fileconfig, self.fileconfig.clone()))
    }

    /// `get_status` payload.
    pub fn get_status(&self) -> (bool, IndexMap<String, Option<IndexMap<String, String>>>) {
        (self.save_config_pending, self.status_save_pending.clone())
    }

    /// `set(section, option, value)` (`value` is already `str(value)`).
    pub fn set(&mut self, section: &str, option: &str, value: &str) -> Result<(), ConfigError> {
        if !self.fileconfig.has_section(section) {
            self.fileconfig.add_section(section)?;
        }
        self.fileconfig.set(section, option, Some(value.to_string()))?;
        let mut pending = self.status_save_pending.clone();
        match pending.get(section) {
            Some(Some(items)) => {
                let mut items = items.clone();
                items.insert(option.to_string(), value.to_string());
                pending.insert(section.to_string(), Some(items));
            }
            _ => {
                let mut items = IndexMap::new();
                items.insert(option.to_string(), value.to_string());
                pending.insert(section.to_string(), Some(items));
            }
        }
        self.status_save_pending = pending;
        self.save_config_pending = true;
        self.log_lines
            .push(format!("save_config: set [{section}] {option} = {value}"));
        Ok(())
    }

    /// `remove_section(section)`.
    pub fn remove_section(&mut self, section: &str) {
        if self.fileconfig.has_section(section) {
            self.fileconfig.remove_section(section);
            let mut pending = self.status_save_pending.clone();
            pending.insert(section.to_string(), None);
            self.status_save_pending = pending;
            self.save_config_pending = true;
        } else if matches!(
            self.status_save_pending.get(section),
            Some(Some(_))
        ) {
            let mut pending = self.status_save_pending.clone();
            pending.shift_remove(section);
            self.status_save_pending = pending;
            self.save_config_pending = true;
        }
    }

    /// `_disallow_include_conflicts`: passthrough error text.
    fn disallow_include_conflicts(
        &self,
        regular_fileconfig: &RawConfigParser,
    ) -> Result<(), ConfigError> {
        for section in self.fileconfig.sections() {
            let opts = self.fileconfig.options(&section)?;
            for option in opts {
                if regular_fileconfig.has_option(&section, &option) {
                    return Err(ConfigError::new(format!(
                        "SAVE_CONFIG section '{section}' option '{option}' conflicts with included value"
                    )));
                }
            }
        }
        Ok(())
    }

    /// Pure text part of `cmd_SAVE_CONFIG`: validate the existing file and
    /// build the new file text (no file writes).
    fn build_save_config_data(
        &mut self,
        existing_data: &str,
        cfgname: &str,
    ) -> Result<String, ConfigError> {
        // Create string containing autosave data
        let mut reader = ConfigFileReader::with_mounts(self.mounts.clone());
        let autosave_text = ConfigFileReader::build_config_string(&self.fileconfig);
        let mut lines: Vec<String> = autosave_text
            .split('\n')
            .map(|l| format!("#*# {l}").trim().to_string())
            .collect();
        lines.insert(0, format!("\n{}", AUTOSAVE_HEADER.trim_end()));
        lines.push(String::new());
        let autosave_data = lines.join("\n");
        let (regular_data, _old) = self.find_autosave_data(existing_data);
        let regular_data = strip_duplicates(&regular_data, &self.fileconfig);
        let data = format!("{}{}", regular_data.trim_end(), autosave_data);
        let (new_regular_data, new_autosave_data) = self.find_autosave_data(&data);
        if new_autosave_data.is_empty() {
            return Err(ConfigError::new(
                "Existing config autosave is corrupted. Can't complete SAVE_CONFIG",
            ));
        }
        let regular_fileconfig =
            match reader.build_fileconfig_with_includes(&new_regular_data, cfgname) {
                Ok(f) => f,
                Err(_) => {
                    return Err(ConfigError::new(
                        "Unable to parse existing config on SAVE_CONFIG",
                    ));
                }
            };
        self.disallow_include_conflicts(&regular_fileconfig)?;
        self.log_lines.extend(reader.log_lines);
        Ok(data)
    }

    /// `cmd_SAVE_CONFIG` except for the timestamp (caller passes
    /// `time.strftime("-%Y%m%d_%H%M%S")`, e.g. `"-20240102_030405"`).
    pub fn save_config(
        &mut self,
        cfgname: &str,
        datestr: &str,
    ) -> Result<Option<SaveConfigResult>, ConfigError> {
        if self.fileconfig.sections().is_empty() {
            return Ok(None);
        }
        let mut reader = ConfigFileReader::with_mounts(self.mounts.clone());
        let existing = match reader.read_config_file(cfgname) {
            Ok(d) => d,
            Err(_) => {
                self.log_lines.extend(reader.log_lines);
                return Err(ConfigError::new(
                    "Unable to read existing config on SAVE_CONFIG",
                ));
            }
        };
        let data = self.build_save_config_data(&existing, cfgname)?;
        let (backup_name, temp_name) = save_config_names(cfgname, datestr);
        self.log_lines.push(format!(
            "SAVE_CONFIG to '{cfgname}' (backup in '{backup_name}')"
        ));
        let write_result: Result<(), ()> = (|| {
            let local_temp = self.mounts.to_local(&temp_name);
            let local_cfg = self.mounts.to_local(cfgname);
            let local_backup = self.mounts.to_local(&backup_name);
            fs::write(&local_temp, &data).map_err(|_| ())?;
            fs::rename(&local_cfg, &local_backup).map_err(|_| ())?;
            fs::rename(&local_temp, &local_cfg).map_err(|_| ())?;
            Ok(())
        })();
        if write_result.is_err() {
            return Err(ConfigError::new(
                "Unable to write config file during SAVE_CONFIG",
            ));
        }
        self.restart_requested = true;
        Ok(Some(SaveConfigResult {
            data,
            backup_name,
            temp_name,
        }))
    }
}

/// Backup/temp names for `SAVE_CONFIG` (`.cfg` suffix rule from Python).
pub fn save_config_names(cfgname: &str, datestr: &str) -> (String, String) {
    let mut backup_name = format!("{cfgname}{datestr}");
    let mut temp_name = format!("{cfgname}_autosave");
    if let Some(stripped) = cfgname.strip_suffix(".cfg") {
        backup_name = format!("{}{datestr}.cfg", stripped);
        temp_name = format!("{}_autosave.cfg", stripped);
    }
    (backup_name, temp_name)
}

/// `time.strftime("-%Y%m%d_%H%M%S")` from wall-clock seconds (UTC).
///
/// Python uses local time; the host pins `TZ=UTC` for parity runs. Passing
/// an explicit `datestr` to [`ConfigAutoSave::save_config`] avoids any
/// timezone dependence.
pub fn datestr_from_wall_time(epoch_secs: f64) -> String {
    let secs = epoch_secs.trunc() as i64;
    let days = secs.div_euclid(86400);
    let secs_of_day = secs.rem_euclid(86400);
    let (h, m, s) = (
        secs_of_day / 3600,
        (secs_of_day % 3600) / 60,
        secs_of_day % 60,
    );
    // Civil from days (Howard Hinnant's algorithm).
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let mo = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if mo <= 2 { y + 1 } else { y };
    format!("-{y:04}{mo:02}{d:02}_{h:02}{m:02}{s:02}")
}

// ---------------------------------------------------------------------------
// ConfigValidate
// ---------------------------------------------------------------------------

/// `ConfigValidate`: access tracking and the unused-options check.
pub struct ConfigValidate {
    status_settings: IndexMap<String, IndexMap<String, PyValue>>,
    access_tracking: Rc<RefCell<IndexMap<(String, String), PyValue>>>,
    autosave_options: IndexMap<(String, String), ()>,
}

impl Default for ConfigValidate {
    fn default() -> Self {
        Self::new()
    }
}

impl ConfigValidate {
    pub fn new() -> Self {
        ConfigValidate {
            status_settings: IndexMap::new(),
            access_tracking: Rc::new(RefCell::new(IndexMap::new())),
            autosave_options: IndexMap::new(),
        }
    }

    /// `start_access_tracking(autosave_fileconfig)`. Returns the store to
    /// share with wrappers.
    pub fn start_access_tracking(
        &mut self,
        autosave_fileconfig: &RawConfigParser,
    ) -> Rc<RefCell<IndexMap<(String, String), PyValue>>> {
        self.autosave_options.clear();
        for section in autosave_fileconfig.sections() {
            for option in autosave_fileconfig.options(&section).unwrap_or_default() {
                self.autosave_options
                    .insert((section.to_lowercase(), option.to_lowercase()), ());
            }
        }
        self.access_tracking = Rc::new(RefCell::new(IndexMap::new()));
        Rc::clone(&self.access_tracking)
    }

    /// `check_unused(fileconfig)` with the printer object names supplied by
    /// the caller (`printer.lookup_objects()` in Python).
    pub fn check_unused(
        &mut self,
        fileconfig: &RawConfigParser,
        valid_section_names: &[&str],
    ) -> Result<(), ConfigError> {
        // Don't warn on fields set in autosave segment
        let mut merged = self.access_tracking.borrow().clone();
        for (k, ()) in self.autosave_options.iter() {
            merged.insert(k.clone(), PyValue::Int(PyInt::from_i64(1)));
        }
        let mut valid_sections: IndexMap<String, ()> = IndexMap::new();
        for name in valid_section_names {
            valid_sections.insert((*name).to_string(), ());
        }
        for ((section, _), _) in merged.iter() {
            valid_sections.insert(section.clone(), ());
        }
        for section_name in fileconfig.sections() {
            let section = section_name.to_lowercase();
            if !valid_sections.contains_key(&section) {
                return Err(ConfigError::new(format!(
                    "Section '{section}' is not a valid config section"
                )));
            }
            for option in fileconfig.options(&section_name)? {
                let option_lower = option.to_lowercase();
                if !merged.contains_key(&(section.clone(), option_lower.clone())) {
                    return Err(ConfigError::new(format!(
                        "Option '{option_lower}' is not valid in section '{section}'"
                    )));
                }
            }
        }
        // Setup get_status()
        self.build_status_settings();
        // Clear tracking state
        self.access_tracking.borrow_mut().clear();
        self.autosave_options.clear();
        Ok(())
    }

    /// `_build_status_settings`.
    pub fn build_status_settings(&mut self) {
        self.status_settings = IndexMap::new();
        let tracking = self.access_tracking.borrow().clone();
        for ((section, option), value) in tracking.iter() {
            self.status_settings
                .entry(section.clone())
                .or_default()
                .insert(option.clone(), value.clone());
        }
    }

    /// `get_status` payload: `{section: {option: value}}`.
    pub fn get_status(&self) -> IndexMap<String, IndexMap<String, PyValue>> {
        let mut out = IndexMap::new();
        for (section, options) in self.status_settings.iter() {
            out.insert(section.clone(), options.clone());
        }
        out
    }
}

fn settings_to_pyvalue(
    settings: &IndexMap<String, IndexMap<String, PyValue>>,
) -> PyValue {
    let mut outer = IndexMap::new();
    for (section, options) in settings.iter() {
        let mut inner = IndexMap::new();
        for (option, value) in options.iter() {
            inner.insert(PyValue::Str(option.clone()), value.clone());
        }
        outer.insert(PyValue::Str(section.clone()), PyValue::Dict(inner));
    }
    PyValue::Dict(outer)
}

// ---------------------------------------------------------------------------
// PrinterConfig
// ---------------------------------------------------------------------------

/// `PrinterConfig`: the `configfile` object.
pub struct PrinterConfig {
    pub autosave: ConfigAutoSave,
    pub validate: ConfigValidate,
    pub deprecated: Rc<RefCell<DeprecatedStore>>,
    /// Raw config (`{section: {option: str}}`).
    pub status_raw_config: IndexMap<String, IndexMap<String, String>>,
}

impl PrinterConfig {
    pub fn new(config_file: String) -> Self {
        Self::with_mounts(config_file, Mounts::default())
    }

    pub fn with_mounts(config_file: String, mounts: Mounts) -> Self {
        let autosave = ConfigAutoSave::new(config_file, mounts);
        PrinterConfig {
            autosave,
            validate: ConfigValidate::new(),
            deprecated: Rc::new(RefCell::new(DeprecatedStore::new())),
            status_raw_config: IndexMap::new(),
        }
    }

    /// `read_config(filename)` (no includes, fresh tracking).
    pub fn read_config(&self, filename: &str) -> Result<ConfigWrapper, ConfigError> {
        let mut reader = ConfigFileReader::with_mounts(self.autosave.mounts.clone());
        let data = reader.read_config_file(filename)?;
        let fileconfig = ConfigFileReader::build_fileconfig(&data, filename)?;
        Ok(ConfigWrapper::new(
            Rc::new(fileconfig),
            Rc::new(RefCell::new(IndexMap::new())),
            Rc::clone(&self.deprecated),
            "printer".to_string(),
        ))
    }

    /// `read_main_config`.
    pub fn read_main_config(&mut self) -> Result<ConfigWrapper, ConfigError> {
        let (fileconfig, autosave_fileconfig) = self.autosave.load_main_config()?;
        let access_tracking = self.validate.start_access_tracking(&autosave_fileconfig);
        let config = ConfigWrapper::new(
            Rc::new(fileconfig),
            access_tracking,
            Rc::clone(&self.deprecated),
            "printer".to_string(),
        );
        self.build_status_config(&config)?;
        Ok(config)
    }

    /// `log_config(config)` (returns the rollover text).
    pub fn log_config(&self, config: &ConfigWrapper) -> String {
        let text = ConfigFileReader::build_config_string(config.fileconfig());
        format!("===== Config file =====\n{text}\n=======================")
    }

    /// `check_unused_options(config)`; `valid_sections` are the printer
    /// object names (`printer.lookup_objects()` in Python).
    pub fn check_unused_options(
        &mut self,
        config: &ConfigWrapper,
        valid_sections: &[&str],
    ) -> Result<(), ConfigError> {
        let fileconfig = config.fileconfig().clone();
        self.validate.check_unused(&fileconfig, valid_sections)
    }

    fn build_status_config(&mut self, config: &ConfigWrapper) -> Result<(), ConfigError> {
        self.status_raw_config = IndexMap::new();
        for section in config.get_prefix_sections("") {
            let mut section_status = IndexMap::new();
            for option in section.get_prefix_options("")? {
                let value = section.get(&option, &None, false)?.unwrap_or_default();
                section_status.insert(option.clone(), value);
            }
            self.status_raw_config
                .insert(section.section_name().to_string(), section_status);
        }
        Ok(())
    }

    /// `runtime_warning(msg)`.
    pub fn runtime_warning(&self, msg: &str) {
        let mut res = IndexMap::new();
        res.insert(
            "type".to_string(),
            PyValue::Str("runtime_warning".to_string()),
        );
        res.insert("message".to_string(), PyValue::Str(msg.to_string()));
        let did_add = self.deprecated.borrow_mut().add(res);
        if did_add {
            self.deprecated.borrow_mut().log_lines.push(msg.to_string());
        }
    }

    /// `deprecate(section, option, value=None, msg=None)`.
    pub fn deprecate(
        &self,
        section: &str,
        option: &str,
        value: Option<PyValue>,
        msg: Option<String>,
    ) {
        let (mut res, defmsg) = match value {
            None => {
                let mut res = IndexMap::new();
                res.insert(
                    "type".to_string(),
                    PyValue::Str("deprecated_option".to_string()),
                );
                let defmsg = format!("Option '{option}' in section '{section}' is deprecated.");
                (res, defmsg)
            }
            Some(v) => {
                let mut res = IndexMap::new();
                res.insert(
                    "type".to_string(),
                    PyValue::Str("deprecated_value".to_string()),
                );
                res.insert("value".to_string(), v.clone());
                let defmsg = format!(
                    "Value '{}' in option '{option}' in section '{section}' is deprecated.",
                    py_str(&v)
                );
                (res, defmsg)
            }
        };
        let msg = msg.unwrap_or(defmsg);
        res.insert("message".to_string(), PyValue::Str(msg));
        res.insert("section".to_string(), PyValue::Str(section.to_string()));
        res.insert("option".to_string(), PyValue::Str(option.to_string()));
        self.deprecated.borrow_mut().add(res);
    }

    /// `deprecate_gcode(cmd, param=None, value=None, msg=None)`.
    pub fn deprecate_gcode(
        &self,
        cmd: &str,
        param: Option<&str>,
        value: Option<&str>,
        msg: Option<String>,
    ) {
        let defmsg = match (param, value) {
            (None, _) => format!("Command '{cmd}' is deprecated."),
            (Some(p), None) => format!("Parameter '{p}' in command '{cmd}' is deprecated."),
            (Some(p), Some(v)) => format!("Value '{p}={v}' in command '{cmd}' is deprecated."),
        };
        let msg = msg.unwrap_or(defmsg);
        let mut res = IndexMap::new();
        res.insert(
            "type".to_string(),
            PyValue::Str("deprecated_gcode".to_string()),
        );
        res.insert("message".to_string(), PyValue::Str(msg));
        res.insert("command".to_string(), PyValue::Str(cmd.to_string()));
        res.insert(
            "parameter".to_string(),
            param.map(|s| PyValue::Str(s.to_string())).unwrap_or(PyValue::None),
        );
        let vstr = match value {
            None => "None".to_string(),
            Some(v) => v.to_string(),
        };
        res.insert("value".to_string(), PyValue::Str(vstr));
        self.deprecated.borrow_mut().add(res);
    }

    /// `deprecate_mcu_code(mcu, feature, msg=None)`. The MCU is identified
    /// by name; versions are the host software version and the MCU version
    /// string (only used for the default message).
    pub fn deprecate_mcu_code(
        &self,
        mcu_name: &str,
        feature: &str,
        msg: Option<String>,
        software_version: &str,
        mcu_version: &str,
    ) {
        let defmsg = format!(
            "MCU '{mcu_name}' has deprecated code (it is missing feature '{feature}'). Recompiling and flashing is recommended (MCU version '{mcu_version}', host version '{software_version}')."
        );
        let msg = msg.unwrap_or(defmsg);
        let mut res = IndexMap::new();
        res.insert(
            "type".to_string(),
            PyValue::Str("deprecated_mcu_code".to_string()),
        );
        res.insert("message".to_string(), PyValue::Str(msg));
        res.insert("mcu".to_string(), PyValue::Str(mcu_name.to_string()));
        res.insert("feature".to_string(), PyValue::Str(feature.to_string()));
        self.deprecated.borrow_mut().add(res);
    }

    /// `set(section, option, value)`.
    pub fn set(&mut self, section: &str, option: &str, value: &str) -> Result<(), ConfigError> {
        self.autosave.set(section, option, value)
    }

    /// `remove_section(section)`.
    pub fn remove_section(&mut self, section: &str) {
        self.autosave.remove_section(section)
    }

    /// `cmd_SAVE_CONFIG` (timestamp passed by the caller).
    pub fn save_config(
        &mut self,
        datestr: &str,
    ) -> Result<Option<SaveConfigResult>, ConfigError> {
        let cfgname = self.autosave.config_file.clone();
        self.autosave.save_config(&cfgname, datestr)
    }

    /// `get_status` (without `eventtime`, unused by Python).
    pub fn get_status(&self) -> PyValue {
        let mut status = IndexMap::new();
        let mut raw = IndexMap::new();
        for (section, options) in self.status_raw_config.iter() {
            let mut inner = IndexMap::new();
            for (option, value) in options.iter() {
                inner.insert(PyValue::Str(option.clone()), PyValue::Str(value.clone()));
            }
            raw.insert(PyValue::Str(section.clone()), PyValue::Dict(inner));
        }
        status.insert(PyValue::Str("config".to_string()), PyValue::Dict(raw));
        let warnings: Vec<PyValue> = self
            .deprecated
            .borrow()
            .warnings
            .iter()
            .map(|w| PyValue::Dict(w.iter().map(|(k, v)| (PyValue::Str(k.clone()), v.clone())).collect()))
            .collect();
        status.insert(PyValue::Str("warnings".to_string()), PyValue::List(warnings));
        let (pending, items) = self.autosave.get_status();
        status.insert(
            PyValue::Str("save_config_pending".to_string()),
            PyValue::Bool(pending),
        );
        let mut items_dict = IndexMap::new();
        for (section, entry) in items.iter() {
            let v = match entry {
                None => PyValue::None,
                Some(options) => {
                    let mut inner = IndexMap::new();
                    for (option, value) in options.iter() {
                        inner.insert(
                            PyValue::Str(option.clone()),
                            PyValue::Str(value.clone()),
                        );
                    }
                    PyValue::Dict(inner)
                }
            };
            items_dict.insert(PyValue::Str(section.clone()), v);
        }
        status.insert(
            PyValue::Str("save_config_pending_items".to_string()),
            PyValue::Dict(items_dict),
        );
        status.insert(
            PyValue::Str("settings".to_string()),
            settings_to_pyvalue(&self.validate.get_status()),
        );
        PyValue::Dict(status)
    }
}

