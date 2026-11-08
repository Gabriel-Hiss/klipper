//! Port of the `re` subset used by `klippy/`.
//!
//! The reference is CPython 3.8 `Lib/sre_compile.py` semantics for the
//! patterns below. Each exact pattern string is accepted by [`compile`];
//! any other pattern is rejected with [`ReError`].
//!
//! # Pattern list
//!
//! | # | Pattern | Location | Operations |
//! |---|---|---|
//! | 1 | `[#;].*$` | `klippy/configfile.py:271` (`comment_r`) | `sub("", line)` |
//! | 2 | `[^A-Za-z0-9_].*$` | `klippy/configfile.py:272` (`value_r`) | `sub("", line)` |
//! | 3 | `\{(?P<eval>[^}]*)\}` | `klippy/console.py:31` (`re_eval`) | `split(line)` |
//! | 4 | `([A-Z_]+|[A-Z*])` | `klippy/gcode.py:201` (`args_r`) | `split(line.upper())` |
//! | 5 | `^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)` | `klippy/gcode.py:432` (`m112_r`) | `match(line)` |
//! | 6 | `(?P<prefix>[ _]pin=)(?P<name>[^ ]*)` | `klippy/pins.py:16` (`re_pin`) | `sub(pin_fixup, cmd)` |
//! | 7 | `(\w+)\s*\((.*)\)$` | `klippy/extras/shaper_defs.py:162` and `:172` | `match(shaper_name)` |
//! | 8 | `(?:(\w+)\s*=\s*)?\s*([\d.]+)` | `klippy/extras/shaper_defs.py:179` | `findall(args)` |
//! | 9 | `(\~.*?\~)` | `klippy/extras/display/menu.py:156` | `split(name)` |
//! | 10 | `[+-]` | `klippy/kinematics/kinematic_stepper.py:12` (`pat`) | `search(str, pos)` |
//!
//! `.` matches any character except `\n`. `$` matches at the end of the
//! string or just before a trailing `\n`. `\w`, `\d` and `\s` are unicode.
//! `^` matches at the start. Alternation is left-to-right, greedy
//! quantifiers prefer the longest match and lazy quantifiers the shortest,
//! exactly as CPython backtracking does. `re.split` with capturing groups
//! keeps the delimiters. `findall` with groups returns tuples of group
//! strings, with `None` mapped to `""`.

use core::fmt;

/// Error for a pattern outside the supported list.
#[derive(Debug, Clone, PartialEq)]
pub enum ReError {
    /// Pattern not in the list.
    UnsupportedPattern { pattern: String },
}

impl fmt::Display for ReError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedPattern { pattern } => write!(
                f,
                "unsupported re pattern (not in the klippy pattern list): {pattern}"
            ),
        }
    }
}

impl std::error::Error for ReError {}

/// One entry of [`PATTERNS`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PatternInfo {
    /// 1-based index in [`PATTERNS`].
    pub index: usize,
    /// Exact pattern string.
    pub pattern: &'static str,
    /// `file:line` of the `re.compile` / `re.match` / `re.split` site.
    pub location: &'static str,
    /// Operations klippy performs with the pattern.
    pub operation: &'static str,
}

/// The fixed pattern list, in the order above.
pub const PATTERNS: &[PatternInfo] = &[
    PatternInfo {
        index: 1,
        pattern: "[#;].*$",
        location: "klippy/configfile.py:271",
        operation: "sub",
    },
    PatternInfo {
        index: 2,
        pattern: "[^A-Za-z0-9_].*$",
        location: "klippy/configfile.py:272",
        operation: "sub",
    },
    PatternInfo {
        index: 3,
        pattern: "\\{(?P<eval>[^}]*)\\}",
        location: "klippy/console.py:31",
        operation: "split",
    },
    PatternInfo {
        index: 4,
        pattern: "([A-Z_]+|[A-Z*])",
        location: "klippy/gcode.py:201",
        operation: "split",
    },
    PatternInfo {
        index: 5,
        pattern: "^(?:[nN][0-9]+)?\\s*[mM]112(?:\\s|$)",
        location: "klippy/gcode.py:432",
        operation: "match",
    },
    PatternInfo {
        index: 6,
        pattern: "(?P<prefix>[ _]pin=)(?P<name>[^ ]*)",
        location: "klippy/pins.py:16",
        operation: "sub",
    },
    PatternInfo {
        index: 7,
        pattern: "(\\w+)\\s*\\((.*)\\)$",
        location: "klippy/extras/shaper_defs.py:162",
        operation: "match",
    },
    PatternInfo {
        index: 8,
        pattern: "(?:(\\w+)\\s*=\\s*)?\\s*([\\d.]+)",
        location: "klippy/extras/shaper_defs.py:179",
        operation: "findall",
    },
    PatternInfo {
        index: 9,
        pattern: "(\\~.*?\\~)",
        location: "klippy/extras/display/menu.py:156",
        operation: "split",
    },
    PatternInfo {
        index: 10,
        pattern: "[+-]",
        location: "klippy/kinematics/kinematic_stepper.py:12",
        operation: "search",
    },
];

#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    CommentSub,
    ValueSub,
    EvalSplit,
    ArgsSplit,
    M112Match,
    PinSub,
    ShaperMatch,
    ShaperFindall,
    TildeSplit,
    PlusMinusSearch,
}

/// Compiled pattern. Construction only succeeds for the strings in
/// [`PATTERNS`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PyRegex {
    kind: Kind,
    pattern: &'static str,
}

impl PyRegex {
    /// Exact pattern string.
    pub fn pattern(&self) -> &'static str {
        self.pattern
    }

    /// `pattern.search(string, pos)`: finds the leftmost match at or after
    /// the unicode scalar index `pos`. Returns `(start, end)`.
    pub fn search_at(&self, text: &str, pos: usize) -> Option<(usize, usize)> {
        let chars: Vec<char> = text.chars().collect();
        match self.kind {
            Kind::PlusMinusSearch => {
                let mut i = pos.min(chars.len());
                while i < chars.len() {
                    if chars[i] == '+' || chars[i] == '-' {
                        return Some((i, i + 1));
                    }
                    i += 1;
                }
                None
            }
            Kind::CommentSub | Kind::ValueSub | Kind::EvalSplit | Kind::ArgsSplit
            | Kind::M112Match | Kind::PinSub | Kind::ShaperMatch | Kind::ShaperFindall
            | Kind::TildeSplit => {
                // `search` is only used for pattern 10 by klippy.
                let _ = chars;
                None
            }
        }
    }

    /// `pattern.search(string)`: leftmost match, or `None`.
    pub fn search(&self, text: &str) -> Option<(usize, usize)> {
        self.search_at(text, 0)
    }

    /// `pattern.match(string)`: anchored at the start. Returns the groups
    /// (without the whole match), or `None`.
    pub fn match_str(&self, text: &str) -> Option<Vec<Option<String>>> {
        match self.kind {
            Kind::M112Match => {
                if matches_m112(text) {
                    Some(Vec::new())
                } else {
                    None
                }
            }
            Kind::ShaperMatch => parse_shaper_match(text),
            _ => None,
        }
    }

    /// `pattern.match(string)` boolean.
    pub fn is_match(&self, text: &str) -> bool {
        self.match_str(text).is_some()
    }

    /// `re.split(pattern, string)` and `pattern.split(string)`: splits on
    /// non-overlapping matches, keeping capturing groups.
    pub fn split(&self, text: &str) -> Vec<String> {
        match self.kind {
            Kind::EvalSplit => split_eval(text),
            Kind::ArgsSplit => split_args(text),
            Kind::TildeSplit => split_tilde(text),
            _ => Vec::new(),
        }
    }

    /// `pattern.sub(repl, string)` where `repl` is a replacement string.
    pub fn sub(&self, repl: &str, text: &str) -> String {
        match self.kind {
            Kind::CommentSub => sub_comment(text, repl),
            Kind::ValueSub => sub_value(text, repl),
            _ => text.to_string(),
        }
    }

    /// `pattern.sub(function, string)` for the pin pattern.
    /// `function` receives `(prefix, name)` and returns the replacement.
    pub fn sub_pin<F>(&self, text: &str, mut f: F) -> String
    where
        F: FnMut(&str, &str) -> String,
    {
        if self.kind != Kind::PinSub {
            return text.to_string();
        }
        sub_pin(text, &mut f)
    }

    /// `re.findall(pattern, string)`. Groups of `None` become `""`.
    pub fn findall(&self, text: &str) -> Vec<Vec<String>> {
        match self.kind {
            Kind::ShaperFindall => findall_shaper(text),
            _ => Vec::new(),
        }
    }
}

/// Compile a pattern string. Only the strings in [`PATTERNS`] pass.
pub fn compile(pattern: &str) -> Result<PyRegex, ReError> {
    for info in PATTERNS {
        if info.pattern == pattern {
            let kind = match info.index {
                1 => Kind::CommentSub,
                2 => Kind::ValueSub,
                3 => Kind::EvalSplit,
                4 => Kind::ArgsSplit,
                5 => Kind::M112Match,
                6 => Kind::PinSub,
                7 => Kind::ShaperMatch,
                8 => Kind::ShaperFindall,
                9 => Kind::TildeSplit,
                _ => Kind::PlusMinusSearch,
            };
            return Ok(PyRegex {
                kind,
                pattern: info.pattern,
            });
        }
    }
    Err(ReError::UnsupportedPattern {
        pattern: pattern.to_string(),
    })
}

fn is_word(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

fn is_digit(c: char) -> bool {
    c.is_ascii_digit() || c.is_numeric() && c.is_ascii()
        || (c as u32) >= 0x660 && (c as u32) <= 0x669
}

fn is_space(c: char) -> bool {
    c.is_whitespace()
}

fn sub_comment(text: &str, repl: &str) -> String {
    // `[#;].*$`: first `#` or `;`, then everything to the end of the line
    // (`.` excludes `\n`). `sub` without `count` replaces every match, so a
    // later `#` on the next line also matches; `.` stops at `\n`.
    let mut out = String::with_capacity(text.len());
    let lines: Vec<&str> = text.split('\n').collect();
    for (idx, line) in lines.iter().enumerate() {
        if idx > 0 {
            out.push('\n');
        }
        match line.find(['#', ';']) {
            Some(pos) => {
                out.push_str(&line[..pos]);
                out.push_str(repl);
            }
            None => out.push_str(line),
        }
    }
    out
}

fn sub_value(text: &str, repl: &str) -> String {
    let mut out = String::with_capacity(text.len());
    let lines: Vec<&str> = text.split('\n').collect();
    for (idx, line) in lines.iter().enumerate() {
        if idx > 0 {
            out.push('\n');
        }
        let mut done = false;
        for (byte, c) in line.char_indices() {
            let is_ok = c.is_ascii_alphanumeric() || c == '_';
            if !is_ok {
                out.push_str(&line[..byte]);
                out.push_str(repl);
                done = true;
                break;
            }
        }
        if !done {
            out.push_str(line);
        }
    }
    out
}

fn split_eval(text: &str) -> Vec<String> {
    // `\{(?P<eval>[^}]*)\}`: `{`, non-greedy `}`-free body, `}`.
    let mut parts = Vec::new();
    let mut rest = text;
    loop {
        let Some(open) = rest.find('{') else {
            parts.push(rest.to_string());
            break;
        };
        let after_open = open + 1;
        // Find the first `}` after `{`.
        let Some(close_rel) = rest[after_open..].find('}') else {
            parts.push(rest.to_string());
            break;
        };
        let close = after_open + close_rel;
        parts.push(rest[..open].to_string());
        parts.push(rest[after_open..close].to_string());
        rest = &rest[close + 1..];
    }
    parts
}

fn split_args(text: &str) -> Vec<String> {
    // `([A-Z_]+|[A-Z*])`: runs of uppercase letters plus `_`, or a single
    // uppercase letter or `*`. `re.split` keeps the captured delimiter.
    let chars: Vec<char> = text.chars().collect();
    let mut parts = Vec::new();
    let mut last = 0usize;
    let mut i = 0usize;
    while i < chars.len() {
        let c = chars[i];
        let is_word = c.is_ascii_uppercase() || c == '_';
        if is_word {
            let start = i;
            let mut j = i;
            while j < chars.len() && (chars[j].is_ascii_uppercase() || chars[j] == '_') {
                j += 1;
            }
            parts.push(chars[last..start].iter().collect());
            parts.push(chars[start..j].iter().collect());
            last = j;
            i = j;
        } else if c.is_ascii_uppercase() || c == '*' {
            parts.push(chars[last..i].iter().collect());
            parts.push(c.to_string());
            last = i + 1;
            i += 1;
        } else {
            i += 1;
        }
    }
    parts.push(chars[last..].iter().collect());
    parts
}

fn matches_m112(text: &str) -> bool {
    // `^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)`
    // `$` is end of string or just before a trailing `\n`.
    let effective = text.strip_suffix('\n').unwrap_or(text);
    let chars: Vec<char> = effective.chars().collect();
    let mut i = 0usize;
    // Optional `[nN][0-9]+`.
    if i < chars.len() && (chars[i] == 'n' || chars[i] == 'N') {
        let mut j = i + 1;
        let mut digits = 0usize;
        while j < chars.len() && chars[j].is_ascii_digit() {
            j += 1;
            digits += 1;
        }
        if digits > 0 {
            i = j;
        }
    }
    while i < chars.len() && is_space(chars[i]) {
        i += 1;
    }
    if i + 4 > chars.len() {
        return false;
    }
    if chars[i] != 'm' && chars[i] != 'M' {
        return false;
    }
    if chars[i + 1] != '1' || chars[i + 2] != '1' || chars[i + 3] != '2' {
        return false;
    }
    i += 4;
    if i >= chars.len() {
        return true;
    }
    is_space(chars[i])
}

fn sub_pin(text: &str, f: &mut impl FnMut(&str, &str) -> String) -> String {
    // `(?P<prefix>[ _]pin=)(?P<name>[^ ]*)`
    let needle = "pin=";
    let mut out = String::with_capacity(text.len());
    let mut start = 0usize;
    let bytes = text.as_bytes();
    let mut i = 0usize;
    while i < text.len() {
        // Look for `pin=` with a preceding space or underscore.
        if text[i..].starts_with(needle)
            && i > 0
        {
            let prev = text[..i].chars().next_back().unwrap();
            if prev == ' ' || prev == '_' {
                let prefix_start = prev.len_utf8() + i - prev.len_utf8();
                // Flush text before the prefix.
                out.push_str(&text[start..prefix_start]);
                let prefix = &text[prefix_start..i + needle.len()];
                let name_start = i + needle.len();
                let mut name_end = name_start;
                while name_end < text.len() {
                    let c = text[name_end..].chars().next().unwrap();
                    if c == ' ' {
                        break;
                    }
                    name_end += c.len_utf8();
                }
                let name = &text[name_start..name_end];
                out.push_str(&f(prefix, name));
                start = name_end;
                i = name_end;
                continue;
            }
        }
        let c = text[i..].chars().next().unwrap();
        i += c.len_utf8();
        let _ = bytes;
    }
    out.push_str(&text[start..]);
    out
}

fn parse_shaper_match(text: &str) -> Option<Vec<Option<String>>> {
    // `(\w+)\s*\((.*)\)$`: `\w+`, optional spaces, `(`, `.*`, `)`, `$`.
    // `.*` is greedy, so the final `)` is the last one on the line.
    let effective = text.strip_suffix('\n').unwrap_or(text);
    let chars: Vec<char> = effective.chars().collect();
    let mut i = 0usize;
    let name_start = i;
    while i < chars.len() && is_word(chars[i]) {
        i += 1;
    }
    if i == name_start {
        return None;
    }
    let name: String = chars[name_start..i].iter().collect();
    while i < chars.len() && is_space(chars[i]) {
        i += 1;
    }
    if i >= chars.len() || chars[i] != '(' {
        return None;
    }
    i += 1;
    if effective.chars().count() == 0 {
        return None;
    }
    // The closing `)` must be the last character (before optional newline).
    let last = chars.len();
    if last == 0 || chars[last - 1] != ')' {
        return None;
    }
    if i > last - 1 {
        return None;
    }
    let args: String = chars[i..last - 1].iter().collect();
    Some(vec![Some(name), Some(args)])
}

fn findall_shaper(text: &str) -> Vec<Vec<String>> {
    // `(?:(\w+)\s*=\s*)?\s*([\d.]+)` with Python `findall`.
    // Greedy `[\d.]+` takes digits and dots. The optional `\w+=` prefix is
    // tried first at each position. `findall` scans left to right and
    // advances by one character on failure.
    let chars: Vec<char> = text.chars().collect();
    let mut out = Vec::new();
    let mut i = 0usize;
    while i < chars.len() {
        match try_shaper_findall_at(&chars, i) {
            Some((key, value, next)) => {
                out.push(vec![
                    key.unwrap_or_default(),
                    value,
                ]);
                i = next.max(i + 1);
            }
            None => {
                i += 1;
            }
        }
    }
    out
}

fn try_shaper_findall_at(
    chars: &[char],
    start: usize,
) -> Option<(Option<String>, String, usize)> {
    let mut i = start;
    let mut key: Option<String> = None;
    // Try `(?:(\w+)\s*=\s*)?`.
    if i < chars.len() && is_word(chars[i]) {
        let mut j = i;
        while j < chars.len() && is_word(chars[j]) {
            j += 1;
        }
        let mut k = j;
        while k < chars.len() && is_space(chars[k]) {
            k += 1;
        }
        if k < chars.len() && chars[k] == '=' {
            let mut m = k + 1;
            while m < chars.len() && is_space(chars[m]) {
                m += 1;
            }
            key = Some(chars[i..j].iter().collect());
            i = m;
        }
    }
    while i < chars.len() && is_space(chars[i]) {
        i += 1;
    }
    let value_start = i;
    while i < chars.len() && (is_digit(chars[i]) || chars[i] == '.') {
        i += 1;
    }
    if i == value_start {
        return None;
    }
    let value: String = chars[value_start..i].iter().collect();
    Some((key, value, i))
}

fn split_tilde(text: &str) -> Vec<String> {
    // `(\~.*?\~)`: lazy `~...~`. `re.split` keeps the captured delimiter.
    let chars: Vec<char> = text.chars().collect();
    let mut parts = Vec::new();
    let mut last = 0usize;
    let mut i = 0usize;
    let mut found = false;
    while i < chars.len() {
        if chars[i] == '~' {
            // Find the next `~` (lazy: the nearest one gives the shortest
            // match).
            let mut j = i + 1;
            while j < chars.len() && chars[j] != '~' {
                j += 1;
            }
            if j < chars.len() {
                parts.push(chars[last..i].iter().collect());
                parts.push(chars[i..=j].iter().collect());
                last = j + 1;
                i = j + 1;
                found = true;
                continue;
            }
        }
        i += 1;
    }
    if found {
        parts.push(chars[last..].iter().collect());
        parts
    } else {
        vec![text.to_string()]
    }
}
