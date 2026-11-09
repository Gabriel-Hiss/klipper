//! Port of the `configparser` subset used by `klippy/`.
//!
//! The reference is CPython 3.8 `Lib/configparser.py`. Klippy builds the main
//! parser as `configparser.RawConfigParser(strict=False,
//! inline_comment_prefixes=(';' , '#'))` in `klippy/configfile.py` and the
//! variables file as plain `configparser.ConfigParser()` in
//! `klippy/extras/save_variables.py`. Constructors below bake in those
//! arguments.
//!
//! The section regex is byte-identical to `RawConfigParser._SECT_TMPL` of
//! CPython 3.8.2, namely `\[ (?P<header>[^]]+) \]` with `re.VERBOSE`, and
//! the option regex is byte-identical to `RawConfigParser._OPT_TMPL`
//! formatted with `delim="=|:"`, namely `(?P<option>.*?) \s*(?P<vi>=|:)
//! \s* (?P<value>.*)$` with `re.VERBOSE`.
//! They are realised as the hand-written functions `sectcre_match` and
//! `optcre_match` so that `.`, `\s`, `\S` and `$` keep Python semantics for
//! single-line inputs without a generic regex engine. `allow_no_value` is
//! always `False`, so the valueless template is not needed.
//!
//! `DEFAULTSECT` is `"DEFAULT"` and `optionxform` lowercases.

use core::fmt;

/// Python `str` `repr`, for error messages.
fn py_repr(s: &str) -> String {
    // CPython chooses single quotes unless the string contains a single
    // quote and no double quote, in which case it chooses double quotes.
    let has_single = s.contains('\'');
    let has_double = s.contains('"');
    let quote = if has_single && !has_double { '"' } else { '\'' };
    let mut out = String::with_capacity(s.len() + 2);
    out.push(quote);
    for c in s.chars() {
        if c == quote || c == '\\' {
            out.push('\\');
            out.push(c);
        } else if c == '\n' {
            out.push_str("\\n");
        } else if c == '\r' {
            out.push_str("\\r");
        } else if c == '\t' {
            out.push_str("\\t");
        } else if c.is_control() {
            let code = c as u32;
            if code <= 0xff {
                out.push_str(&format!("\\x{code:02x}"));
            } else if code <= 0xffff {
                out.push_str(&format!("\\u{code:04x}"));
            } else {
                out.push_str(&format!("\\U{code:08x}"));
            }
        } else {
            out.push(c);
        }
    }
    out.push(quote);
    out
}

/// Port of the CPython `configparser` exception classes.
///
/// `Display` returns `str(exc)` in CPython. The generic `Error(String)`
/// variant mirrors `configparser.Error(msg)`.
#[derive(Debug, Clone, PartialEq)]
pub enum Error {
    /// `configparser.Error(msg)`.
    Error(String),
    /// `configparser.DuplicateSectionError(section, source, lineno)`.
    DuplicateSectionError {
        section: String,
        source: Option<String>,
        lineno: Option<usize>,
    },
    /// `configparser.DuplicateOptionError(section, option, source, lineno)`.
    DuplicateOptionError {
        section: Option<String>,
        option: String,
        source: Option<String>,
        lineno: Option<usize>,
    },
    /// `configparser.NoSectionError(section)`.
    NoSectionError { section: String },
    /// `configparser.NoOptionError(option, section)`.
    NoOptionError { option: String, section: String },
    /// `configparser.ParsingError(source)`.
    ParsingError {
        source: String,
        errors: Vec<(usize, String)>,
    },
    /// `configparser.MissingSectionHeaderError(filename, lineno, line)`.
    /// `line` is the raw line as iterated, including its newline.
    MissingSectionHeaderError {
        filename: String,
        lineno: usize,
        line: String,
    },
    /// `configparser.InterpolationMissingOptionError`.
    InterpolationMissingOptionError {
        option: String,
        section: String,
        rawval: String,
        reference: String,
    },
    /// `configparser.InterpolationSyntaxError`.
    InterpolationSyntaxError {
        option: String,
        section: String,
        msg: String,
    },
    /// `configparser.InterpolationDepthError`.
    InterpolationDepthError {
        option: String,
        section: String,
        rawval: String,
    },
    /// A plain `ValueError` raised by `add_section`, `set`,
    /// `before_set` validation, `int`/`float` conversion or
    /// `_convert_to_boolean`. The inner text is `str(exc)`.
    ValueError(String),
}

impl Error {
    /// Plain `configparser.Error(msg)`.
    pub fn new(msg: impl Into<String>) -> Self {
        Self::Error(msg.into())
    }
    /// Keep `ParsingError` source for tests.
    pub fn parsing_source(&self) -> Option<&str> {
        match self {
            Self::ParsingError { source, .. } => Some(source),
            _ => None,
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Error(msg) | Self::ValueError(msg) => write!(f, "{msg}"),
            Self::DuplicateSectionError {
                section,
                source,
                lineno,
            } => {
                let sec_repr = py_repr(section);
                match source {
                    Some(src) => {
                        let mut msg = format!("While reading from {}", py_repr(src));
                        if let Some(lineno) = lineno {
                            msg.push_str(&format!(" [line {lineno:2}]"));
                        }
                        msg.push_str(&format!(": section {sec_repr} already exists"));
                        write!(f, "{msg}")
                    }
                    None => write!(f, "Section {sec_repr} already exists"),
                }
            }
            Self::DuplicateOptionError {
                section,
                option,
                source,
                lineno,
            } => {
                let opt_repr = py_repr(option);
                let sec_repr = py_repr(section.as_deref().unwrap_or(""));
                match source {
                    Some(src) => {
                        let mut msg = format!("While reading from {}", py_repr(src));
                        if let Some(lineno) = lineno {
                            msg.push_str(&format!(" [line {lineno:2}]"));
                        }
                        msg.push_str(&format!(
                            ": option {opt_repr} in section {sec_repr} already exists"
                        ));
                        write!(f, "{msg}")
                    }
                    None => {
                        write!(
                            f,
                            "Option {opt_repr} in section {sec_repr} already exists"
                        )
                    }
                }
            }
            Self::NoSectionError { section } => {
                write!(f, "No section: {}", py_repr(section))
            }
            Self::NoOptionError { option, section } => {
                write!(
                    f,
                    "No option {} in section: {}",
                    py_repr(option),
                    py_repr(section)
                )
            }
            Self::ParsingError { source, errors } => {
                let mut msg = format!("Source contains parsing errors: {}", py_repr(source));
                for (lineno, line_repr) in errors {
                    msg.push_str(&format!("\n\t[line {lineno:2}]: {line_repr}"));
                }
                write!(f, "{msg}")
            }
            Self::MissingSectionHeaderError {
                filename,
                lineno,
                line,
            } => write!(
                f,
                "File contains no section headers.\nfile: {}, line: {}\n{}",
                py_repr(filename),
                lineno,
                py_repr(line)
            ),
            Self::InterpolationMissingOptionError {
                option,
                section,
                rawval,
                reference,
            } => write!(
                f,
                "Bad value substitution: option {} in section {} contains an interpolation key {} which is not a valid option name. Raw value: {}",
                py_repr(option),
                py_repr(section),
                py_repr(reference),
                py_repr(rawval)
            ),
            Self::InterpolationSyntaxError { msg, .. } => write!(f, "{msg}"),
            Self::InterpolationDepthError {
                option,
                section,
                rawval,
            } => write!(
                f,
                "Recursion limit exceeded in value substitution: option {} in section {} contains an interpolation key which cannot be substituted in {} steps. Raw value: {}",
                py_repr(option),
                py_repr(section),
                10,
                py_repr(rawval)
            ),
        }
    }
}

impl std::error::Error for Error {}

/// `str.isspace()` for one character: CPython 3.8 `str.strip()` treats
/// U+001C..U+001F as whitespace on top of Rust's `White_Space` set, so
/// `re` `\s` (used by `SECTCRE`/`OPTCRE`/`NONSPACECRE`) matches them too.
fn is_space(c: char) -> bool {
    c.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&c)
}

/// `str.strip()` with no arguments.
fn strip_line(s: &str) -> String {
    s.trim_matches(is_space).to_string()
}

/// `SECTCRE.match(value)`: returns the `header` group. CPython 3.8.2 spells
/// the group `[^]]+`, so the header ends at the first `]`.
fn sectcre_match(value: &str) -> Option<String> {
    if !value.starts_with('[') {
        return None;
    }
    let end = value.find(']')?;
    if end < 2 {
        return None;
    }
    let header = &value[1..end];
    if header.is_empty() || header.contains('\n') {
        return None;
    }
    Some(header.to_string())
}

/// `OPTCRE_NV` without the valueless part. Returns
/// `(option, delimiter, value)`.
fn optcre_match(value: &str) -> Option<(String, char, String)> {
    let delim_pos = value.find(['=', ':'])?;
    let delim = value[delim_pos..].chars().next()?;
    let opt_raw = &value[..delim_pos];
    let opt_end = opt_raw
        .char_indices()
        .rev()
        .find(|(_, c)| !is_space(*c))
        .map(|(i, c)| i + c.len_utf8())
        .unwrap_or(0);
    let option = &opt_raw[..opt_end];
    let val_raw = &value[delim_pos + delim.len_utf8()..];
    let val_start = val_raw
        .char_indices()
        .find(|(_, c)| !is_space(*c))
        .map(|(i, _)| i)
        .unwrap_or(val_raw.len());
    Some((option.to_string(), delim, val_raw[val_start..].to_string()))
}

#[derive(Debug, Clone)]
struct Section {
    name: String,
    options: Vec<(String, Option<String>)>,
}

impl Section {
    fn get_index(&self, option: &str) -> Option<usize> {
        self.options
            .iter()
            .position(|(k, _)| k == option)
    }
}

#[allow(dead_code)]
enum OptStore {
    List(Vec<String>),
    Single(Option<String>),
}

#[allow(dead_code)]
/// Parser state mirroring `RawConfigParser._sections` and `_defaults`.
#[derive(Debug, Clone)]
pub struct RawConfigParser {
    strict: bool,
    inline_comment_prefixes: Vec<char>,
    comment_prefixes: Vec<char>,
    default_section: String,
    interpolation: bool,
    allow_no_value: bool,
    empty_lines_in_values: bool,
    sections: Vec<Section>,
    defaults: Vec<(String, Option<String>)>,
}

impl RawConfigParser {
    /// `RawConfigParser(strict=False, inline_comment_prefixes=(';' , '#'))`
    /// with the defaults `delimiters=('=' , ':')` and
    /// `comment_prefixes=('#' , ';')`, as in `klippy/configfile.py`.
    pub fn new_klippy() -> Self {
        Self {
            strict: false,
            inline_comment_prefixes: vec![';', '#'],
            comment_prefixes: vec!['#', ';'],
            default_section: "DEFAULT".to_string(),
            interpolation: false,
            allow_no_value: false,
            empty_lines_in_values: true,
            sections: Vec::new(),
            defaults: Vec::new(),
        }
    }

    /// `ConfigParser()` defaults: `strict=True`,
    /// `inline_comment_prefixes=None`, `BasicInterpolation`.
    pub fn new_interpolating() -> Self {
        Self {
            strict: true,
            inline_comment_prefixes: Vec::new(),
            comment_prefixes: vec!['#', ';'],
            default_section: "DEFAULT".to_string(),
            interpolation: true,
            allow_no_value: false,
            empty_lines_in_values: true,
            sections: Vec::new(),
            defaults: Vec::new(),
        }
    }

    /// `parser.default_section`.
    pub fn default_section(&self) -> &str {
        &self.default_section
    }

    /// `parser.optionxform(optionstr)`, which lowercases.
    pub fn optionxform(&self, option: &str) -> String {
        option.to_lowercase()
    }

    /// `parser.sections()`.
    pub fn sections(&self) -> Vec<String> {
        self.sections
            .iter()
            .map(|s| s.name.clone())
            .collect()
    }

    /// `parser.has_section(section)`.
    pub fn has_section(&self, section: &str) -> bool {
        self.sections.iter().any(|s| s.name == section)
    }

    fn section_index(&self, section: &str) -> Option<usize> {
        self.sections.iter().position(|s| s.name == section)
    }

    /// `parser.has_option(section, option)`.
    pub fn has_option(&self, section: &str, option: &str) -> bool {
        let opt = self.optionxform(option);
        if section.is_empty() || section == self.default_section {
            return self
                .defaults
                .iter()
                .any(|(k, _)| *k == opt);
        }
        match self.section_index(section) {
            None => false,
            Some(idx) => {
                self.sections[idx].get_index(&opt).is_some()
                    || self
                        .defaults
                        .iter()
                        .any(|(k, _)| *k == opt)
            }
        }
    }

    /// `parser.options(section)`. There is no special case for `""` or
    /// `"DEFAULT"`: both raise `NoSectionError`, like `self._sections[section]`.
    pub fn options(&self, section: &str) -> Result<Vec<String>, Error> {
        let idx = self
            .section_index(section)
            .ok_or_else(|| Error::NoSectionError {
                section: section.to_string(),
            })?;
        let mut opts: Vec<String> = self.sections[idx]
            .options
            .iter()
            .map(|(k, _)| k.clone())
            .collect();
        for (k, _) in &self.defaults {
            if !opts.contains(k) {
                opts.push(k.clone());
            }
        }
        Ok(opts)
    }

    fn unified_get_raw(
        &self,
        section: &str,
        option: &str,
    ) -> Result<Option<String>, Error> {
        let opt = self.optionxform(option);
        // `_unify_values`: only the default section itself falls back to the
        // defaults; `""` raises `NoSectionError` like any missing section.
        if section == self.default_section {
            for (k, v) in &self.defaults {
                if *k == opt {
                    return Ok(v.clone());
                }
            }
            return Err(Error::NoOptionError {
                option: opt,
                section: section.to_string(),
            });
        }
        let idx = match self.section_index(section) {
            Some(i) => i,
            None => {
                return Err(Error::NoSectionError {
                    section: section.to_string(),
                })
            }
        };
        if let Some(pos) = self.sections[idx].get_index(&opt) {
            return Ok(self.sections[idx].options[pos].1.clone());
        }
        for (k, v) in &self.defaults {
            if *k == opt {
                return Ok(v.clone());
            }
        }
        Err(Error::NoOptionError {
            option: opt,
            section: section.to_string(),
        })
    }

    /// `parser.get(section, option)` with no `fallback`, no `raw`/`vars`
    /// keyword arguments. `RawConfigParser` returns the stored value;
    /// `ConfigParser` expands interpolations.
    pub fn get(&self, section: &str, option: &str) -> Result<Option<String>, Error> {
        let raw = self.unified_get_raw(section, option)?;
        match raw {
            None => Ok(None),
            Some(value) => {
                if self.interpolation {
                    let expanded = self.interpolate(section, option, &value)?;
                    Ok(Some(expanded))
                } else {
                    Ok(Some(value))
                }
            }
        }
    }

    /// `parser.getint(section, option)`.
    pub fn getint(&self, section: &str, option: &str) -> Result<i64, Error> {
        let value = self
            .get(section, option)?
            .ok_or_else(|| Error::NoOptionError {
                option: self.optionxform(option),
                section: section.to_string(),
            })?;
        py_int(&value).map_err(Error::ValueError)
    }

    /// `parser.getfloat(section, option)`.
    pub fn getfloat(&self, section: &str, option: &str) -> Result<f64, Error> {
        let value = self
            .get(section, option)?
            .ok_or_else(|| Error::NoOptionError {
                option: self.optionxform(option),
                section: section.to_string(),
            })?;
        py_float(&value).map_err(Error::ValueError)
    }

    /// `parser.getboolean(section, option)`.
    pub fn getboolean(&self, section: &str, option: &str) -> Result<bool, Error> {
        let value = self
            .get(section, option)?
            .ok_or_else(|| Error::NoOptionError {
                option: self.optionxform(option),
                section: section.to_string(),
            })?;
        match value.to_lowercase().as_str() {
            "1" | "yes" | "true" | "on" => Ok(true),
            "0" | "no" | "false" | "off" => Ok(false),
            _ => Err(Error::ValueError(format!("Not a boolean: {value}"))),
        }
    }

    /// `parser.items(section)`. The default section is special: its items are
    /// the defaults (`_sections[section]` lookup skipped); `""` raises
    /// `NoSectionError` through `options`.
    pub fn items(&self, section: &str) -> Result<Vec<(String, Option<String>)>, Error> {
        if section == self.default_section {
            let mut out = Vec::with_capacity(self.defaults.len());
            for (name, _) in &self.defaults {
                let v = self.get(section, name)?;
                out.push((name.clone(), v));
            }
            return Ok(out);
        }
        let names = self.options(section)?;
        let mut out = Vec::with_capacity(names.len());
        for name in names {
            let v = self.get(section, &name)?;
            out.push((name, v));
        }
        Ok(out)
    }

    /// `parser.items()` with the section omitted is unused by klippy.
    /// `parser.defaults()`.
    pub fn defaults(&self) -> Vec<(String, Option<String>)> {
        self.defaults.clone()
    }

    /// `parser.add_section(section)`.
    pub fn add_section(&mut self, section: &str) -> Result<(), Error> {
        if section == self.default_section {
            return Err(Error::ValueError(format!(
                "Invalid section name: {}",
                py_repr(section)
            )));
        }
        if self.has_section(section) {
            return Err(Error::DuplicateSectionError {
                section: section.to_string(),
                source: None,
                lineno: None,
            });
        }
        self.sections.push(Section {
            name: section.to_string(),
            options: Vec::new(),
        });
        Ok(())
    }

    /// `parser.set(section, option, value)`.
    pub fn set(
        &mut self,
        section: &str,
        option: &str,
        value: Option<String>,
    ) -> Result<(), Error> {
        if let Some(ref v) = value {
            if self.interpolation {
                self.validate_interpolation(v)?;
            }
        }
        let opt = self.optionxform(option);
        if section.is_empty() || section == self.default_section {
            if let Some((_, slot)) = self.defaults.iter_mut().find(|(k, _)| *k == opt) {
                *slot = value;
            } else {
                self.defaults.push((opt, value));
            }
            return Ok(());
        }
        let idx = match self.section_index(section) {
            Some(i) => i,
            None => {
                return Err(Error::NoSectionError {
                    section: section.to_string(),
                })
            }
        };
        if let Some(pos) = self.sections[idx].get_index(&opt) {
            self.sections[idx].options[pos].1 = value;
        } else {
            self.sections[idx].options.push((opt, value));
        }
        Ok(())
    }

    /// `parser.remove_section(section)`.
    pub fn remove_section(&mut self, section: &str) -> bool {
        let Some(idx) = self.section_index(section) else {
            return false;
        };
        self.sections.remove(idx);
        true
    }

    /// `parser.remove_option(section, option)`.
    pub fn remove_option(&mut self, section: &str, option: &str) -> Result<bool, Error> {
        let opt = self.optionxform(option);
        if section.is_empty() || section == self.default_section {
            let Some(pos) = self.defaults.iter().position(|(k, _)| *k == opt) else {
                return Ok(false);
            };
            self.defaults.remove(pos);
            return Ok(true);
        }
        let idx = match self.section_index(section) {
            Some(i) => i,
            None => {
                return Err(Error::NoSectionError {
                    section: section.to_string(),
                })
            }
        };
        let Some(pos) = self.sections[idx].get_index(&opt) else {
            return Ok(false);
        };
        self.sections[idx].options.remove(pos);
        Ok(true)
    }

    /// `parser.write(fp)` with the default `space_around_delimiters=True`.
    /// Returns the `.ini` text. Comments are not preserved.
    pub fn write(&self) -> String {
        self.write_with_spacing(true)
    }

    /// `parser.write(fp, space_around_delimiters)`.
    pub fn write_with_spacing(&self, space_around_delimiters: bool) -> String {
        let delim = if space_around_delimiters {
            " = ".to_string()
        } else {
            "=".to_string()
        };
        let mut out = String::new();
        if !self.defaults.is_empty() {
            write_section(&mut out, &self.default_section, &self.defaults, &delim);
        }
        for section in &self.sections {
            write_section(&mut out, &section.name, &section.options, &delim);
        }
        out
    }

    /// `parser.read_file(f, source)`.
    pub fn read_file(&mut self, data: &str, source: &str) -> Result<(), Error> {
        self.read_internal(data, source)
    }

    /// `parser.read_string(string, source='<string>')`.
    pub fn read_string(&mut self, data: &str) -> Result<(), Error> {
        self.read_internal(data, "<string>")
    }

    /// `parser.read(filename)`: files that cannot be opened are silently
    /// ignored. Returns the list of successfully read files.
    pub fn read(&mut self, filename: &str) -> Result<Vec<String>, Error> {
        let content = match std::fs::read_to_string(filename) {
            Ok(s) => s,
            Err(_) => return Ok(Vec::new()),
        };
        // CPython `read` decodes text and iterates lines; the `data`
        // klippy feeds through `read_file` already replaced `\r\n`.
        self.read_internal(&content, filename)?;
        Ok(vec![filename.to_string()])
    }

    fn read_internal(&mut self, data: &str, source: &str) -> Result<(), Error> {
        // `read_file` iterates the file object line by line, keeping the
        // trailing newline on each line. Normalise `data` to that form.
        let mut lines: Vec<String> = Vec::new();
        let bytes: Vec<char> = data.chars().collect();
        let mut cur = String::new();
        for c in bytes {
            cur.push(c);
            if c == '\n' {
                lines.push(core::mem::take(&mut cur));
            }
        }
        if !cur.is_empty() {
            lines.push(cur);
        }

        // Mutable section view: indices into `self.sections`, or `-2` for
        // defaults. Stored values are multiline lists until
        // `_join_multiline_values`.
        let mut cursect_idx: Option<isize> = None; // None = no section
        let mut optname: Option<String> = None;
        let mut optline: Option<String> = None;
        let mut indent_level: usize = 0;
        let mut sectname: Option<String> = None;
        // Temporary (section, options) mailbox.
        let mut temp_defaults: Vec<(String, OptStore)> = Vec::new();
        let mut temp_sections: Vec<(String, Vec<(String, OptStore)>)> =
            Vec::new();

        fn temp_get<'a>(
            is_default: bool,
            sectname: Option<&str>,
            temp_defaults: &'a mut Vec<(String, OptStore)>,
            temp_sections: &'a mut Vec<(String, Vec<(String, OptStore)>)>,
        ) -> &'a mut Vec<(String, OptStore)> {
            if is_default {
                return temp_defaults;
            }
            let name = sectname.unwrap();
            let pos = temp_sections
                .iter()
                .position(|(n, _)| n == name)
                .unwrap_or_else(|| {
                    temp_sections.push((name.to_string(), Vec::new()));
                    temp_sections.len() - 1
                });
            &mut temp_sections[pos].1
        }

        let mut elements_added: Vec<(Option<String>, String)> = Vec::new();
        let mut parse_err: Option<Error> = None;

        for (index, line) in lines.iter().enumerate() {
            let lineno = index + 1;
            // Inline comment stripping (`_read` lines 1000-1020).
            let mut comment_start: Option<usize> = None;
            if !self.inline_comment_prefixes.is_empty() {
                let chars: Vec<char> = line.chars().collect();
                let mut best: Option<usize> = None;
                for &prefix in &self.inline_comment_prefixes {
                    let mut from = 0usize;
                    loop {
                        // byte index of prefix at or after char `from`
                        let byte_from = char_to_byte(&chars, from);
                        let Some(rel) = line[byte_from..].find(prefix) else {
                            break;
                        };
                        let byte_idx = byte_from + rel;
                        let char_idx = byte_to_char(&chars, byte_idx);
                        if char_idx == 0
                            || chars
                                .get(char_idx - 1)
                                .map(|c| is_space(*c))
                                .unwrap_or(false)
                        {
                            best = Some(match best {
                                None => char_idx,
                                Some(b) => b.min(char_idx),
                            });
                        }
                        from = char_idx + 1;
                    }
                }
                comment_start = best;
            }
            // Full-line comment stripping (`line.strip()`, Unicode).
            let stripped = strip_line(line);
            if self
                .comment_prefixes
                .iter()
                .any(|&p| stripped.starts_with(p))
            {
                comment_start = Some(0);
            }

            let value = match comment_start {
                Some(cs) => {
                    let byte = char_to_byte(&line.chars().collect::<Vec<_>>(), cs);
                    strip_line(&line[..byte])
                }
                None => strip_line(line),
            };
            if value.is_empty() {
                if self.empty_lines_in_values {
                    if comment_start.is_none() {
                        if let (Some(_), Some(name)) = (cursect_idx, optname.as_deref()) {
                            let is_default = cursect_idx == Some(-2);
                            let opts = temp_get(
                                is_default,
                                sectname.as_deref(),
                                &mut temp_defaults,
                                &mut temp_sections,
                            );
                            if let Some((_, store)) =
                                opts.iter_mut().find(|(k, _)| k == name)
                            {
                                if matches!(store, OptStore::List(_)) {
                                    if let OptStore::List(list) = store {
                                        list.push(String::new());
                                    }
                                }
                            }
                        }
                    }
                } else {
                    indent_level = usize::MAX;
                }
                continue;
            }

            // Continuation line?
            let first_nonspace: Option<usize> = line
                .chars()
                .position(|c| !is_space(c));
            let cur_indent = first_nonspace.unwrap_or(0);
            let is_continuation = cursect_idx.is_some()
                && optname.is_some()
                && cur_indent > indent_level;
            if is_continuation {
                let name = optname.clone().unwrap();
                let is_default = cursect_idx == Some(-2);
                let opts = temp_get(
                    is_default,
                    sectname.as_deref(),
                    &mut temp_defaults,
                    &mut temp_sections,
                );
                if let Some((_, OptStore::List(list))) =
                    opts.iter_mut().find(|(k, _)| *k == name)
                {
                    list.push(value.clone());
                }
            } else {
                indent_level = cur_indent;
                // Section header?
                if let Some(header) = sectcre_match(&value) {
                    sectname = Some(header.clone());
                    let exists = self.sections.iter().any(|s| s.name == header)
                        || temp_sections
                            .iter()
                            .any(|(n, _)| *n == header);
                    if exists {
                        if self.strict
                            && elements_added
                                .contains(&(Some(header.clone()), String::new()))
                        {
                            // duplicate section number marker
                            return Err(Error::DuplicateSectionError {
                                section: header.clone(),
                                source: Some(source.to_string()),
                                lineno: Some(lineno),
                            });
                        }
                        cursect_idx = Some(self.section_index(&header).map(|i| i as isize).unwrap_or(-1));
                        if cursect_idx == Some(-1) {
                            // Exists only in temp (created earlier in this
                            // parse): keep -1 as sentinel for temp.
                        } else if header == self.default_section {
                            cursect_idx = Some(-2);
                        }
                        if header == self.default_section {
                            cursect_idx = Some(-2);
                        }
                        elements_added.push((Some(header.clone()), String::new()));
                    } else if header == self.default_section {
                        cursect_idx = Some(-2);
                    } else {
                        cursect_idx = Some(-1);
                        // create section now (order), keep temp mirror
                        if !self.sections.iter().any(|s| s.name == header) {
                            self.sections.push(Section {
                                name: header.clone(),
                                options: Vec::new(),
                            });
                        }
                        temp_sections.push((header.clone(), Vec::new()));
                        elements_added.push((Some(header.clone()), String::new()));
                    }
                    optname = None;
                    optline = None;
                } else if cursect_idx.is_none() {
                    // No section header yet.
                    return Err(Error::MissingSectionHeaderError {
                        filename: source.to_string(),
                        lineno,
                        line: line.clone(),
                    });
                } else {
                    // Option line.
                    match optcre_match(&value) {
                        Some((opt_raw, _vi, optval)) => {
                            if opt_raw.is_empty() {
                                parse_err =
                                    Some(handle_error(parse_err, source, lineno, line));
                                continue;
                            }
                            // `optname.rstrip()` (Unicode).
                            let name = self.optionxform(opt_raw.trim_end_matches(is_space));
                            let sect_key = sectname.clone();
                            if self.strict
                                && elements_added
                                    .contains(&(sect_key.clone(), name.clone()))
                            {
                                return Err(Error::DuplicateOptionError {
                                    section: sect_key,
                                    option: name.clone(),
                                    source: Some(source.to_string()),
                                    lineno: Some(lineno),
                                });
                            }
                            elements_added.push((sect_key, name.clone()));
                            optname = Some(name.clone());
                            optline = Some(line.clone());
                            let is_default = cursect_idx == Some(-2);
                            let opts = temp_get(
                                is_default,
                                sectname.as_deref(),
                                &mut temp_defaults,
                                &mut temp_sections,
                            );
                            let store = OptStore::List(vec![strip_line(&optval)]);
                            if let Some((_, slot)) =
                                opts.iter_mut().find(|(k, _)| *k == name)
                            {
                                *slot = store;
                            } else {
                                opts.push((name, store));
                            }
                        }
                        None => {
                            parse_err =
                                Some(handle_error(parse_err, source, lineno, line));
                        }
                    }
                }
            }
        }

        // `_join_multiline_values` plus `_interpolation.before_read`.
        // Apply temp mailbox into the parser, in parse order.
        for (name, store) in temp_defaults {
            let joined = join_store(store);
            let joined = if self.interpolation {
                // `before_read` is identity for BasicInterpolation.
                joined
            } else {
                joined
            };
            if let Some((_, slot)) =
                self.defaults.iter_mut().find(|(k, _)| *k == name)
            {
                *slot = joined;
            } else {
                self.defaults.push((name, joined));
            }
        }
        for (sec_name, opts) in temp_sections {
            let Some(sec_idx) = self.section_index(&sec_name) else {
                continue;
            };
            for (name, store) in opts {
                let joined = join_store(store);
                if let Some(pos) =
                    self.sections[sec_idx].get_index(&name)
                {
                    self.sections[sec_idx].options[pos].1 = joined;
                } else {
                    self.sections[sec_idx]
                        .options
                        .push((name, joined));
                }
            }
        }
        let _ = &optline;

        if let Some(e) = parse_err {
            return Err(e);
        }
        Ok(())
    }

    fn validate_interpolation(&self, value: &str) -> Result<(), Error> {
        // `BasicInterpolation.before_set`.
        let tmp: String = value.replace("%%", "");
        let cleaned = remove_keycre(&tmp);
        if cleaned.contains('%') {
            let pos = cleaned.find('%').unwrap_or(0);
            return Err(Error::ValueError(format!(
                "invalid interpolation syntax in {} at position {}",
                py_repr(value),
                pos
            )));
        }
        Ok(())
    }

    fn interpolate(
        &self,
        section: &str,
        option: &str,
        value: &str,
    ) -> Result<String, Error> {
        self.interpolate_inner(section, option, value, 1, value.to_string())
    }

    fn interpolate_inner(
        &self,
        section: &str,
        option: &str,
        rest: &str,
        depth: usize,
        rawval: String,
    ) -> Result<String, Error> {
        if depth > 10 {
            return Err(Error::InterpolationDepthError {
                option: option.to_string(),
                section: section.to_string(),
                rawval,
            });
        }
        let mut accum = String::new();
        let mut rest = rest.to_string();
        while !rest.is_empty() {
            let Some(p) = rest.find('%') else {
                accum.push_str(&rest);
                return Ok(accum);
            };
            if p > 0 {
                accum.push_str(&rest[..p]);
                rest = rest[p..].to_string();
                continue;
            }
            let c = rest.chars().nth(1);
            if c == Some('%') {
                accum.push('%');
                rest = rest[2..].to_string();
            } else if c == Some('(') {
                match parse_keycre(&rest) {
                    None => {
                        return Err(Error::InterpolationSyntaxError {
                            option: option.to_string(),
                            section: section.to_string(),
                            msg: format!(
                                "bad interpolation variable reference {}",
                                py_repr(&rest)
                            ),
                        })
                    }
                    Some((var, end)) => {
                        let var = self.optionxform(&var);
                        let v = match self.unified_get_raw(section, &var) {
                            Ok(Some(v)) => v,
                            _ => {
                                return Err(Error::InterpolationMissingOptionError {
                                    option: option.to_string(),
                                    section: section.to_string(),
                                    rawval: rawval.clone(),
                                    reference: var,
                                })
                            }
                        };
                        rest = rest[end..].to_string();
                        if v.contains('%') {
                            accum.push_str(&self.interpolate_inner(
                                section,
                                option,
                                &v,
                                depth + 1,
                                rawval.clone(),
                            )?);
                        } else {
                            accum.push_str(&v);
                        }
                    }
                }
            } else {
                return Err(Error::InterpolationSyntaxError {
                    option: option.to_string(),
                    section: section.to_string(),
                    msg: format!(
                        "'%%' must be followed by '%%' or '(', found: {}",
                        py_repr(&rest)
                    ),
                });
            }
        }
        Ok(accum)
    }
}

fn write_section(
    out: &mut String,
    section_name: &str,
    items: &[(String, Option<String>)],
    delimiter: &str,
) {
    out.push_str(&format!("[{section_name}]\n"));
    for (key, value) in items {
        match value {
            None => {
                out.push_str(&format!("{key}\n"));
            }
            Some(v) => {
                let body = v.replace('\n', "\n\t");
                out.push_str(&format!("{key}{delimiter}{body}\n"));
            }
        }
    }
    out.push('\n');
}

fn handle_error(
    exc: Option<Error>,
    source: &str,
    lineno: usize,
    line: &str,
) -> Error {
    let mut errors = match &exc {
        Some(Error::ParsingError { errors, .. }) => errors.clone(),
        _ => Vec::new(),
    };
    errors.push((lineno, py_repr(line)));
    Error::ParsingError {
        source: source.to_string(),
        errors,
    }
}

fn join_store(store: OptStore) -> Option<String> {
    match store {
        OptStore::Single(v) => v,
        OptStore::List(list) => {
            // `'\n'.join(val).rstrip()` (Unicode).
            let joined = list.join("\n");
            Some(joined.trim_end_matches(is_space).to_string())
        }
    }
}

fn char_to_byte(chars: &[char], char_idx: usize) -> usize {
    let mut byte = 0;
    for (i, c) in chars.iter().enumerate() {
        if i == char_idx {
            return byte;
        }
        byte += c.len_utf8();
    }
    byte
}

fn byte_to_char(chars: &[char], byte_idx: usize) -> usize {
    let mut byte = 0;
    for (i, c) in chars.iter().enumerate() {
        if byte == byte_idx {
            return i;
        }
        byte += c.len_utf8();
    }
    chars.len()
}

/// `re.sub(r"%\(([^)]+)\)s", "", value)`.
fn remove_keycre(value: &str) -> String {
    let mut out = String::new();
    let mut rest = value;
    while let Some(start) = rest.find("%(") {
        out.push_str(&rest[..start + 2]);
        let after = &rest[start + 2..];
        match after.find(')') {
            Some(close) => {
                let inner = &after[..close];
                if !inner.is_empty() && after[close..].starts_with(")s") {
                    rest = &after[close + 2..];
                    continue;
                }
                out.push_str(after);
                rest = "";
                break;
            }
            None => {
                out.push_str(after);
                rest = "";
                break;
            }
        }
    }
    if !rest.is_empty() {
        out.push_str(rest);
    }
    out
}

/// Match `%\(([^)]+)\)s` at the start of `rest`. Returns `(group, end)`.
fn parse_keycre(rest: &str) -> Option<(String, usize)> {
    if !rest.starts_with("%(") {
        return None;
    }
    let close = rest.find(')')?;
    let inner = &rest[2..close];
    if inner.is_empty() || !rest[close..].starts_with(")s") {
        return None;
    }
    Some((inner.to_string(), close + 2))
}

/// Python `int(value)` with base 10. `int()` strips `White_Space` but not
/// U+001C..U+001F (verified against the oracle), so this keeps Rust's
/// `trim`, unlike the `str.strip()` sites above.
fn py_int(value: &str) -> Result<i64, String> {
    let original = value;
    let trimmed = value.trim();
    // Python allows underscores between digits (3.6+).
    let cleaned: String = trimmed.chars().filter(|&c| c != '_').collect();
    // Reject misplaced underscores like Python does.
    fn underscores_ok(s: &str) -> bool {
        for (i, c) in s.chars().enumerate() {
            if c == '_' && (i == 0 || i + 1 >= s.len()) {
                return false;
            }
        }
        // Must not start/end with sign-adjacent underscore after sign; a
        // simple check is enough for config values.
        !(s.starts_with('_') || s.ends_with('_'))
    }
    if !underscores_ok(trimmed) {
        return Err(format!(
            "invalid literal for int() with base 10: {}",
            py_repr(original)
        ));
    }
    cleaned.parse::<i64>().map_err(|_| {
        format!(
            "invalid literal for int() with base 10: {}",
            py_repr(original)
        )
    })
}

/// Python `float(value)`. Same strip set as `int()` (U+001C..U+001F are not
/// stripped), so this keeps Rust's `trim` too.
fn py_float(value: &str) -> Result<f64, String> {
    let original = value;
    let trimmed = value.trim();
    // Python accepts `nan`, `inf`, underscores; Rust `parse` accepts `inf`,
    // `NaN` in any case. Normalise underscores out after validation.
    let no_underscore: String = trimmed.chars().filter(|&c| c != '_').collect();
    let lowered = no_underscore.to_ascii_lowercase();
    match lowered.as_str() {
        "nan" | "+nan" | "-nan" => {
            return Ok(f64::NAN * if trimmed.starts_with('-') { -1.0 } else { 1.0 })
        }
        "inf" | "+inf" | "infinity" | "+infinity" => return Ok(f64::INFINITY),
        "-inf" | "-infinity" => return Ok(f64::NEG_INFINITY),
        _ => {}
    }
    no_underscore.parse::<f64>().map_err(|_| {
        format!(
            "could not convert string to float: {}",
            py_repr(original)
        )
    })
}

/// Interpolating parser used by `save_variables.py`.
pub type ConfigParser = RawConfigParser;
