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
//! Runtime wrappers implement `klippy_api`'s configuration interfaces.
//! `PrinterConfig::new` registers `SAVE_CONFIG`; the command writes the
//! configuration and requests a restart after releasing mutable state.
//! Standalone autosave helpers accept an explicit backup timestamp for replay.
//!
//! `ConfigFileReader` additionally accepts extra mount prefixes used by
//! the parity harness to map absolute include paths (e.g. `/usr/data`) to
//! a scenario directory. Production passes no mounts and reads the real
//! filesystem.

use klippy_api::GCell;
use std::fs;
use std::path::PathBuf;
use std::sync::Arc;

use indexmap::IndexMap;
use pycompat::{PyInt, PyValue};
use pycompat::text::{float_from_string, int_from_string};
use pycompat::value::{py_equal, py_rich_compare, type_name};
use pytext::configparser::{Error as ConfigError, RawConfigParser};
use klippy_api::config::{self as api, Config, ConfigApi, ListParser, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCodeApi, RegisterCommandOpts};
use klippy_api::object::lookup_interface;
use klippy_api::printer::{LookupDefault, ModuleFactory, Printer};

/// The exact autosave marker, including the leading and trailing newline.
pub const AUTOSAVE_HEADER: &str =
    "\n#*# <---------------------- SAVE_CONFIG ---------------------->\n#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.\n#*#\n";
/// Help text registered for the `SAVE_CONFIG` g-code command.
pub const SAVE_CONFIG_HELP: &str = "Overwrite config file and restart";

// ---------------------------------------------------------------------------
// Path helpers mirroring `os.path` on POSIX.
// ---------------------------------------------------------------------------

/// `os.path.split(path)`.
fn py_split(path: &str) -> (&str, &str) {
    let end = path.rfind('/').map(|index| index + 1).unwrap_or(0);
    let (head, tail) = path.split_at(end);
    if !head.is_empty() && !head.chars().all(|c| c == '/') {
        (head.trim_end_matches('/'), tail)
    } else {
        (head, tail)
    }
}

/// `os.path.dirname(path)`.
pub fn py_dirname(path: &str) -> String {
    py_split(path).0.to_string()
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
    let prefix = if absolute.starts_with("//") && !absolute.starts_with("///") { "//" } else { "/" };
    format!("{prefix}{}", parts.join("/"))
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
    let content_start = j;
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
    let content = &chars[content_start..k];
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

/// `os.path.lexists(path)`.
fn py_lexists(path: &str) -> bool {
    fs::symlink_metadata(path).is_ok()
}

/// `os.path.isdir(path)`.
fn py_isdir(path: &str) -> bool {
    fs::metadata(path).map(|m| m.is_dir()).unwrap_or(false)
}

/// `glob._iterdir(dirname, dironly)`: entry names in directory order.
fn glob_iterdir(dirname: &str, dironly: bool) -> Vec<String> {
    let dir = if dirname.is_empty() { "." } else { dirname };
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    entries
        .flatten()
        .filter(|entry| !dironly || fs::metadata(entry.path()).map(|m| m.is_dir()).unwrap_or(false))
        .map(|entry| entry.file_name().to_string_lossy().into_owned())
        .collect()
}

/// `glob._glob1(dirname, pattern, dironly)`.
fn glob1(dirname: &str, pattern: &str, dironly: bool) -> Vec<String> {
    let hidden_ok = pattern.starts_with('.');
    glob_iterdir(dirname, dironly)
        .into_iter()
        .filter(|name| hidden_ok || !name.starts_with('.'))
        .filter(|name| fnmatch_name(pattern, name))
        .collect()
}

/// `glob._glob0(dirname, basename, dironly)` (`dironly` is unused there).
fn glob0(dirname: &str, basename: &str) -> Vec<String> {
    let found = if basename.is_empty() {
        py_isdir(dirname)
    } else {
        py_lexists(&py_join(dirname, basename))
    };
    if found { vec![basename.to_string()] } else { Vec::new() }
}

/// `glob._iglob(pathname, recursive=False, dironly)`; `glob.glob(pattern)`
/// is `iglob(pattern, false)`. Result paths keep the pattern's spelling.
fn iglob(pathname: &str, dironly: bool) -> Vec<String> {
    let (dirname, basename) = py_split(pathname);
    if !has_glob_magic(pathname) {
        let found = if basename.is_empty() { py_isdir(dirname) } else { py_lexists(pathname) };
        return if found { vec![pathname.to_string()] } else { Vec::new() };
    }
    if dirname.is_empty() {
        return glob1(dirname, basename, dironly);
    }
    let dirs = if dirname != pathname && has_glob_magic(dirname) {
        iglob(dirname, true)
    } else {
        vec![dirname.to_string()]
    };
    let magic = has_glob_magic(basename);
    let mut out = Vec::new();
    for dir in &dirs {
        let names = if magic { glob1(dir, basename, dironly) } else { glob0(dir, basename) };
        out.extend(names.iter().map(|name| py_join(dir, name)));
    }
    out
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
        iglob(&local, false)
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
                klippy_api::log::error(&msg);
                Err(ConfigError::new(msg))
            }
        }
    }

    /// `build_config_string(fileconfig)`: `write()` then `strip()`.
    pub fn build_config_string(fileconfig: &RawConfigParser) -> String {
        py_strip(&fileconfig.write()).to_string()
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
            let include_spec = sectcre_header(&line).and_then(|h| h.strip_prefix("include "));
            if let Some(include_spec) = include_spec {
                self.append_fileconfig(fileconfig, &buf.join("\n"), filename)?;
                buf.clear();
                let include_spec = py_strip(include_spec).to_string();
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

/// `RawConfigParser.SECTCRE.match(line)` header group. CPython 3.8 spells
/// the group `[^]]+`, so the header ends at the first `]`.
fn sectcre_header(line: &str) -> Option<&str> {
    let rest = line.strip_prefix('[')?;
    let end = rest.find(']')?;
    (end > 0).then(|| &rest[..end])
}

/// `str.isspace()` for one character: Rust's `White_Space` set plus the
/// four information separators (U+001C..U+001F) CPython also strips.
fn py_isspace(c: char) -> bool {
    c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c)
}

/// `str.strip()`.
fn py_strip(s: &str) -> &str {
    s.trim_matches(py_isspace)
}

/// `str.rstrip()`.
fn py_rstrip(s: &str) -> &str {
    s.trim_end_matches(py_isspace)
}

// ---------------------------------------------------------------------------
// Typed access
// ---------------------------------------------------------------------------

/// A configparser error as the Python exception it is (`configparser.Error`
/// itself is `KlippyError::Config`; subclasses keep their class name).
pub fn config_error(e: ConfigError) -> KlippyError {
    let class_name = match &e {
        ConfigError::Error(_) => return KlippyError::Config(e.to_string()),
        ConfigError::DuplicateSectionError { .. } => "DuplicateSectionError",
        ConfigError::DuplicateOptionError { .. } => "DuplicateOptionError",
        ConfigError::NoSectionError { .. } => "NoSectionError",
        ConfigError::NoOptionError { .. } => "NoOptionError",
        ConfigError::ParsingError { .. } => "ParsingError",
        ConfigError::MissingSectionHeaderError { .. } => "MissingSectionHeaderError",
        ConfigError::InterpolationMissingOptionError { .. } => "InterpolationMissingOptionError",
        ConfigError::InterpolationSyntaxError { .. } => "InterpolationSyntaxError",
        ConfigError::InterpolationDepthError { .. } => "InterpolationDepthError",
        ConfigError::ValueError(_) => "ValueError",
    };
    KlippyError::Other { class_name: class_name.into(), message: e.to_string() }
}

fn type_error(message: String) -> KlippyError {
    KlippyError::Internal { class_name: "TypeError".into(), message }
}

fn unhashable(value: &PyValue) -> KlippyError {
    type_error(format!("unhashable type: '{}'", type_name(value)))
}

fn parse_error(section: &str, option: &str) -> ConfigError {
    ConfigError::new(format!(
        "Unable to parse option '{option}' in section '{section}'"
    ))
}

/// The `parser` argument of `_get_wrapper`.
#[derive(Clone, Copy, Debug)]
pub enum OptionParser<'a> {
    /// `fileconfig.get`.
    Str,
    /// `fileconfig.getint`.
    Int,
    /// `fileconfig.getfloat`.
    Float,
    /// `fileconfig.getboolean`.
    Bool,
    /// The `fcparser` closure of `getlists`.
    Lists { seps: &'a [String], count: Option<i64>, parser: ListParser },
}

/// The `minval`, `maxval`, `above` and `below` arguments of
/// `_get_wrapper`; `None` is the Python `None` (no check).
#[derive(Clone, Debug, Default)]
pub struct Limits {
    pub minval: Option<PyValue>,
    pub maxval: Option<PyValue>,
    pub above: Option<PyValue>,
    pub below: Option<PyValue>,
}

/// `lparser` from `getlists`. Count errors are `configparser.Error`s;
/// element parse errors and `str.split('')` become "Unable to parse".
fn list_parse(
    value: &str,
    pos: usize,
    seps: &[String],
    count: Option<i64>,
    parser: ListParser,
    section: &str,
    option: &str,
) -> Result<PyValue, ConfigError> {
    let parts: Vec<&str> = if py_strip(value).is_empty() {
        Vec::new()
    } else if seps[pos].is_empty() {
        return Err(parse_error(section, option));
    } else {
        value.split(seps[pos].as_str()).map(py_strip).collect()
    };
    if pos > 0 {
        let mut out = Vec::new();
        for p in parts.into_iter().filter(|p| !p.is_empty()) {
            out.push(list_parse(p, pos - 1, seps, count, parser, section, option)?);
        }
        return Ok(PyValue::Tuple(out));
    }
    let mut res = Vec::with_capacity(parts.len());
    for p in parts {
        res.push(match parser {
            ListParser::Str => PyValue::Str(p.to_string()),
            ListParser::Int => PyValue::Int(
                int_from_string(p, 10).map_err(|_| parse_error(section, option))?,
            ),
            ListParser::Float => PyValue::Float(
                float_from_string(p).map_err(|_| parse_error(section, option))?,
            ),
        });
    }
    if let Some(n) = count {
        if res.len() as i64 != n {
            return Err(ConfigError::new(format!(
                "Option '{option}' in section '{section}' must have {n} elements"
            )));
        }
    }
    Ok(PyValue::Tuple(res))
}

/// `PrinterConfig.status_warnings` with the `_add_deprecated` dedup.
#[derive(Debug, Default)]
pub struct DeprecatedStore {
    /// Warning dicts in insertion order.
    pub warnings: Vec<IndexMap<String, PyValue>>,
    /// Log messages (only `runtime_warning` logs).
    pub log_lines: Vec<String>,
}

impl DeprecatedStore {
    pub fn new() -> Self {
        Self::default()
    }

    /// `_add_deprecated(data)`: the dedup key `tuple(data.items())` is
    /// compared with Python `==`, so every stored warning is a key.
    fn add(&mut self, data: IndexMap<String, PyValue>) -> bool {
        let seen = self.warnings.iter().any(|warning| {
            warning.len() == data.len()
                && warning
                    .iter()
                    .zip(data.iter())
                    .all(|((k1, v1), (k2, v2))| k1 == k2 && py_equal(v1, v2))
        });
        if seen {
            return false;
        }
        self.warnings.push(data);
        true
    }
}

/// Typed config access for one section, sharing the file config and the
/// access-tracking store across wrappers.
#[derive(Clone)]
pub struct ConfigWrapper {
    inner: Arc<ConfigWrapperInner>,
}

struct ConfigWrapperInner {
    printer: Printer,
    fileconfig: Arc<RawConfigParser>,
    access_tracking: Arc<GCell<IndexMap<(String, String), PyValue>>>,
    section: String,
}

impl ConfigWrapper {
    pub fn new(
        printer: Printer,
        fileconfig: Arc<RawConfigParser>,
        access_tracking: Arc<GCell<IndexMap<(String, String), PyValue>>>,
        section: String,
    ) -> Self {
        ConfigWrapper {
            inner: Arc::new(ConfigWrapperInner {
                printer,
                fileconfig,
                access_tracking,
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
            inner: Arc::new(ConfigWrapperInner {
                printer: Arc::clone(&self.inner.printer),
                fileconfig: Arc::clone(&self.inner.fileconfig),
                access_tracking: Arc::clone(&self.inner.access_tracking),
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
        if !self.inner.fileconfig.has_option(&self.inner.section, option) {
            return;
        }
        let pconfig = lookup_interface::<dyn PrinterConfigApi>(
            &*self.inner.printer, "configfile", LookupDefault::Raise)
            .expect("ConfigApi::deprecate cannot return lookup errors")
            .expect("Raise lookup returns an object");
        pconfig.deprecate(&self.inner.section, option, value, None);
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

    /// The `parser(self.section, option)` call of `_get_wrapper`.
    fn parse_option(&self, parser: OptionParser<'_>, option: &str) -> Result<PyValue, ConfigError> {
        let section = self.inner.section.as_str();
        let raw = match self.inner.fileconfig.get(section, option) {
            Ok(raw) => raw,
            Err(ConfigError::ValueError(_)) => return Err(parse_error(section, option)),
            Err(e) => return Err(e),
        };
        let Some(raw) = raw else {
            // `int(None)`, `None.lower()` and `None.strip()` are not
            // configparser errors.
            return match parser {
                OptionParser::Str => Ok(PyValue::None),
                _ => Err(parse_error(section, option)),
            };
        };
        match parser {
            OptionParser::Str => Ok(PyValue::Str(raw)),
            OptionParser::Int => int_from_string(&raw, 10)
                .map(PyValue::Int)
                .map_err(|_| parse_error(section, option)),
            OptionParser::Float => float_from_string(&raw)
                .map(PyValue::Float)
                .map_err(|_| parse_error(section, option)),
            OptionParser::Bool => match raw.to_lowercase().as_str() {
                "1" | "yes" | "true" | "on" => Ok(PyValue::Bool(true)),
                "0" | "no" | "false" | "off" => Ok(PyValue::Bool(false)),
                _ => Err(parse_error(section, option)),
            },
            OptionParser::Lists { seps, count, parser } => match seps.len().checked_sub(1) {
                Some(last) => list_parse(&raw, last, seps, count, parser, section, option),
                // `lparser(value, -1)`: an empty value is `()`, any other
                // value indexes the empty `seps` tuple.
                None if py_strip(&raw).is_empty() => Ok(PyValue::Tuple(Vec::new())),
                None => Err(parse_error(section, option)),
            },
        }
    }

    /// `_get_wrapper(parser, option, default, minval, maxval, above, below,
    /// note_valid)`. `default: None` is the `sentinel` (required option);
    /// any other default is returned unchanged when the option is absent.
    pub fn get_wrapper(
        &self,
        parser: OptionParser<'_>,
        option: &str,
        default: Option<&PyValue>,
        limits: &Limits,
        note_valid: bool,
    ) -> KResult<PyValue> {
        if !self.inner.fileconfig.has_option(&self.inner.section, option) {
            let Some(default) = default else {
                return Err(config_error(self.missing_error(option)));
            };
            if note_valid && *default != PyValue::None {
                self.track(option, default.clone());
            }
            return Ok(default.clone());
        }
        let v = self.parse_option(parser, option).map_err(config_error)?;
        if note_valid {
            self.track(option, v.clone());
        }
        let checks = [
            (&limits.minval, "<", "must have minimum of"),
            (&limits.maxval, ">", "must have maximum of"),
            (&limits.above, "<=", "must be above"),
            (&limits.below, ">=", "must be below"),
        ];
        for (bound, op, text) in checks {
            let Some(bound) = bound else { continue };
            if py_rich_compare(op, &v, bound).map_err(type_error)? {
                return Err(KlippyError::Config(format!(
                    "Option '{option}' in section '{}' {text} {}",
                    self.inner.section,
                    bound.str()
                )));
            }
        }
        Ok(v)
    }

    /// `get(option, default, note_valid)`.
    pub fn get(&self, option: &str, default: Option<&PyValue>, note_valid: bool) -> KResult<PyValue> {
        self.get_wrapper(OptionParser::Str, option, default, &Limits::default(), note_valid)
    }

    /// `getint(option, default, minval, maxval, note_valid)`.
    pub fn getint(
        &self,
        option: &str,
        default: Option<&PyValue>,
        minval: Option<PyValue>,
        maxval: Option<PyValue>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        let limits = Limits { minval, maxval, ..Limits::default() };
        self.get_wrapper(OptionParser::Int, option, default, &limits, note_valid)
    }

    /// `getfloat(option, default, minval, maxval, above, below, note_valid)`.
    #[allow(clippy::too_many_arguments)]
    pub fn getfloat(
        &self,
        option: &str,
        default: Option<&PyValue>,
        minval: Option<PyValue>,
        maxval: Option<PyValue>,
        above: Option<PyValue>,
        below: Option<PyValue>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        let limits = Limits { minval, maxval, above, below };
        self.get_wrapper(OptionParser::Float, option, default, &limits, note_valid)
    }

    /// `getboolean(option, default, note_valid)`.
    pub fn getboolean(&self, option: &str, default: Option<&PyValue>, note_valid: bool) -> KResult<PyValue> {
        self.get_wrapper(OptionParser::Bool, option, default, &Limits::default(), note_valid)
    }

    /// `getchoice(option, choices, default, note_valid)`; `choices` is the
    /// Python list or dict.
    pub fn getchoice(
        &self,
        option: &str,
        choices: &PyValue,
        default: Option<&PyValue>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        let entries: Vec<(PyValue, PyValue)> = match choices {
            // `{i: i for i in choices}`: an equal later item keeps the first
            // key object and replaces its value.
            PyValue::List(items) => {
                let mut entries: Vec<(PyValue, PyValue)> = Vec::new();
                for item in items {
                    if !item.is_hashable() {
                        return Err(unhashable(item));
                    }
                    match entries.iter_mut().find(|(key, _)| py_equal(key, item)) {
                        Some(entry) => entry.1 = item.clone(),
                        None => entries.push((item.clone(), item.clone())),
                    }
                }
                entries
            }
            PyValue::Dict(map) => map.iter().map(|(k, v)| (k.clone(), v.clone())).collect(),
            other if other.truthy() => {
                return Err(KlippyError::Internal {
                    class_name: "AttributeError".into(),
                    message: format!("'{}' object has no attribute 'keys'", type_name(other)),
                })
            }
            _ => Vec::new(),
        };
        let c = if matches!(entries.first(), Some((PyValue::Int(_), _))) {
            self.getint(option, default, None, None, note_valid)?
        } else {
            self.get(option, default, note_valid)?
        };
        if !c.is_hashable() {
            return Err(unhashable(&c));
        }
        match entries.into_iter().find(|(key, _)| py_equal(key, &c)) {
            Some((_, value)) => Ok(value),
            None => Err(KlippyError::Config(format!(
                "Choice '{}' for option '{option}' in section '{}' is not a valid choice",
                c.str(),
                self.inner.section
            ))),
        }
    }

    /// `getlists(option, default, seps, count, parser, note_valid)`.
    pub fn getlists(
        &self,
        option: &str,
        default: Option<&PyValue>,
        seps: &[String],
        count: Option<i64>,
        parser: ListParser,
        note_valid: bool,
    ) -> KResult<PyValue> {
        let parser = OptionParser::Lists { seps, count, parser };
        self.get_wrapper(parser, option, default, &Limits::default(), note_valid)
    }

    /// `getlist(option, default, sep, count, note_valid)`.
    pub fn getlist(
        &self,
        option: &str,
        default: Option<&PyValue>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        self.getlists(option, default, &[sep.to_string()], count, ListParser::Str, note_valid)
    }

    /// `getintlist(option, default, sep, count, note_valid)`.
    pub fn getintlist(
        &self,
        option: &str,
        default: Option<&PyValue>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        self.getlists(option, default, &[sep.to_string()], count, ListParser::Int, note_valid)
    }

    /// `getfloatlist(option, default, sep, count, note_valid)`.
    pub fn getfloatlist(
        &self,
        option: &str,
        default: Option<&PyValue>,
        sep: &str,
        count: Option<i64>,
        note_valid: bool,
    ) -> KResult<PyValue> {
        self.getlists(option, default, &[sep.to_string()], count, ListParser::Float, note_valid)
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
        autosave_data = py_strip(&data[pos + AUTOSAVE_HEADER.len()..]).to_string();
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
    // Python starts with `section = None`; `has_option(None, ..)` and
    // `has_option('', ..)` both look only at the DEFAULT section.
    let mut section = String::new();
    let mut is_dup_field = false;
    for line in lines.iter_mut() {
        let pruned = py_rstrip(match line.find(['#', ';']) {
            Some(pos) => &line[..pos],
            None => line.as_str(),
        });
        let Some(first) = pruned.chars().next() else {
            continue;
        };
        if py_isspace(first) {
            if is_dup_field {
                line.insert(0, '#');
            }
            continue;
        }
        is_dup_field = false;
        if first == '[' {
            let inner = &pruned[1..];
            let inner = match inner.char_indices().last() {
                Some((last, _)) => &inner[..last],
                None => inner,
            };
            section = py_strip(inner).to_string();
            continue;
        }
        let end = pruned
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or(pruned.len());
        if fileconfig.has_option(&section, &pruned[..end]) {
            is_dup_field = true;
            line.insert(0, '#');
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
        for message in &logs { klippy_api::log::warning(message); }
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
        let message = format!("save_config: set [{section}] {option} = {value}");
        klippy_api::log::info(&message);
        self.log_lines.push(message);
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
            .map(|l| py_strip(&format!("#*# {l}")).to_string())
            .collect();
        lines.insert(0, format!("\n{}", py_rstrip(AUTOSAVE_HEADER)));
        lines.push(String::new());
        let autosave_data = lines.join("\n");
        let (regular_data, _old) = self.find_autosave_data(existing_data);
        let regular_data = strip_duplicates(&regular_data, &self.fileconfig);
        let data = format!("{}{}", py_rstrip(&regular_data), autosave_data);
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
                    self.log_lines.extend(reader.log_lines);
                    let message = "Unable to parse existing config on SAVE_CONFIG";
                    klippy_api::log::error(message);
                    self.log_lines.push(message.into());
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
                let message = "Unable to read existing config on SAVE_CONFIG";
                klippy_api::log::error(message);
                self.log_lines.push(message.into());
                return Err(ConfigError::new(
                    "Unable to read existing config on SAVE_CONFIG",
                ));
            }
        };
        let data = self.build_save_config_data(&existing, cfgname)?;
        let (backup_name, temp_name) = save_config_names(cfgname, datestr);
        let message = format!("SAVE_CONFIG to '{cfgname}' (backup in '{backup_name}')");
        klippy_api::log::info(&message);
        self.log_lines.push(message);
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
            let message = "Unable to write config file during SAVE_CONFIG";
            klippy_api::log::error(message);
            self.log_lines.push(message.into());
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
    let secs = kos::cconv::f64_to_i64(epoch_secs);
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
    access_tracking: Arc<GCell<IndexMap<(String, String), PyValue>>>,
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
            access_tracking: Arc::new(GCell::new(IndexMap::new())),
            autosave_options: IndexMap::new(),
        }
    }

    /// `start_access_tracking(autosave_fileconfig)`. Returns the store to
    /// share with wrappers.
    pub fn start_access_tracking(
        &mut self,
        autosave_fileconfig: &RawConfigParser,
    ) -> Arc<GCell<IndexMap<(String, String), PyValue>>> {
        self.autosave_options.clear();
        for section in autosave_fileconfig.sections() {
            for option in autosave_fileconfig.options(&section).unwrap_or_default() {
                self.autosave_options
                    .insert((section.to_lowercase(), option.to_lowercase()), ());
            }
        }
        self.access_tracking = Arc::new(GCell::new(IndexMap::new()));
        Arc::clone(&self.access_tracking)
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

/// Configs handed out through `PrinterConfigApi` with their parsers, so
/// `log_config`/`check_unused_options` reach `config.fileconfig`.
type ExposedConfigs = Vec<(std::sync::Weak<dyn ConfigApi>, Arc<RawConfigParser>)>;

/// `PrinterConfig`: the `configfile` object.
pub struct PrinterConfig {
    printer: Printer,
    pub autosave: GCell<ConfigAutoSave>,
    pub validate: GCell<ConfigValidate>,
    pub deprecated: Arc<GCell<DeprecatedStore>>,
    status_raw_config: GCell<IndexMap<String, IndexMap<String, PyValue>>>,
    fileconfigs: GCell<ExposedConfigs>,
}

impl PrinterConfig {
    pub fn new(printer: Printer) -> KResult<Arc<Self>> {
        Self::with_mounts(printer, Mounts::default())
    }

    pub fn with_mounts(printer: Printer, mounts: Mounts) -> KResult<Arc<Self>> {
        let args = printer.get_start_args();
        let config_file = match dict_item(&args, "config_file") {
            Some(PyValue::Str(filename)) => filename.clone(),
            _ => return Err(KlippyError::Internal {
                class_name: "KeyError".into(), message: "'config_file'".into(),
            }),
        };
        let gcode = lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise)?
            .expect("Raise lookup returns an object");
        let config = Arc::new(PrinterConfig {
            printer,
            autosave: GCell::new(ConfigAutoSave::new(config_file, mounts)),
            validate: GCell::new(ConfigValidate::new()),
            deprecated: Arc::new(GCell::new(DeprecatedStore::new())),
            status_raw_config: GCell::new(IndexMap::new()),
            fileconfigs: GCell::new(Vec::new()),
        });
        let command_config = Arc::clone(&config);
        gcode.register_command(
            "SAVE_CONFIG",
            Some(Box::new(move |_| {
                let saved = command_config.save_config(
                    &datestr_from_wall_time(kos::time::wall_time()))
                    .map_err(|e| KlippyError::Command(e.to_string()))?
                    .is_some();
                if saved {
                    let gcode = lookup_interface::<dyn GCodeApi>(
                        &*command_config.printer, "gcode", LookupDefault::Raise)?
                        .expect("Raise lookup returns an object");
                    gcode.request_restart("restart")?;
                }
                Ok(())
            })),
            RegisterCommandOpts { desc: Some(SAVE_CONFIG_HELP.into()), ..Default::default() },
        )?;
        Ok(config)
    }

    /// `read_config(filename)` (no includes, fresh tracking).
    pub fn read_config(&self, filename: &str) -> KResult<ConfigWrapper> {
        let mut reader = ConfigFileReader::with_mounts(self.autosave.borrow().mounts.clone());
        let data = reader.read_config_file(filename).map_err(config_error)?;
        let fileconfig = ConfigFileReader::build_fileconfig(&data, filename).map_err(config_error)?;
        Ok(ConfigWrapper::new(
            Arc::clone(&self.printer),
            Arc::new(fileconfig),
            Arc::new(GCell::new(IndexMap::new())),
            "printer".to_string(),
        ))
    }

    /// `read_main_config`.
    pub fn read_main_config(&self) -> KResult<ConfigWrapper> {
        let (fileconfig, autosave_fileconfig) =
            self.autosave.borrow_mut().load_main_config().map_err(config_error)?;
        let access_tracking = self.validate.borrow_mut().start_access_tracking(&autosave_fileconfig);
        let config = ConfigWrapper::new(
            Arc::clone(&self.printer),
            Arc::new(fileconfig),
            access_tracking,
            "printer".to_string(),
        );
        self.build_status_config(&config)?;
        Ok(config)
    }

    fn build_status_config(&self, config: &ConfigWrapper) -> KResult<()> {
        let mut raw_config = IndexMap::new();
        for section in config.get_prefix_sections("") {
            let mut section_status = IndexMap::new();
            for option in section.get_prefix_options("").map_err(config_error)? {
                let value = section.get(&option, None, false)?;
                section_status.insert(option, value);
            }
            raw_config.insert(section.section_name().to_string(), section_status);
        }
        self.status_raw_config.replace(raw_config);
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
            klippy_api::log::warning(msg);
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
        let (mut res, defmsg) = match value.filter(|v| *v != PyValue::None) {
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
                    v.str()
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
    pub fn set(&self, section: &str, option: &str, value: &str) -> Result<(), ConfigError> {
        self.autosave.borrow_mut().set(section, option, value)
    }

    /// `remove_section(section)`.
    pub fn remove_section(&self, section: &str) {
        self.autosave.borrow_mut().remove_section(section)
    }

    /// `cmd_SAVE_CONFIG` (timestamp passed by the caller).
    pub fn save_config(
        &self,
        datestr: &str,
    ) -> Result<Option<SaveConfigResult>, ConfigError> {
        let mut autosave = self.autosave.borrow_mut();
        let cfgname = autosave.config_file.clone();
        autosave.save_config(&cfgname, datestr)
    }

    /// `get_status` (without `eventtime`, unused by Python).
    pub fn get_status(&self) -> PyValue {
        let mut status = IndexMap::new();
        let mut raw = IndexMap::new();
        for (section, options) in self.status_raw_config.borrow().iter() {
            let mut inner = IndexMap::new();
            for (option, value) in options.iter() {
                inner.insert(PyValue::Str(option.clone()), value.clone());
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
        let (pending, items) = self.autosave.borrow().get_status();
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
            settings_to_pyvalue(&self.validate.borrow().get_status()),
        );
        PyValue::Dict(status)
    }
}


mod runtime;
pub use runtime::module_factory;
use runtime::dict_item;
