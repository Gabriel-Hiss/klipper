//! Integration tests using klippy's own inputs.
//!
//! Every expectation comes from the live reference through `pyoracle`
//! (stdlib `configparser`, `shlex` and `re` on CPython 3.8.2), run on the
//! same inputs as the port.
//!
//! Parser options: `RawConfigParser(strict=False,
//! inline_comment_prefixes=(';' , '#'))` per `klippy/configfile.py`.
//! Shlex: `shlex.shlex(s, posix=True)`, `whitespace_split=True`,
//! `commenters='#;'` per `klippy/gcode.py`.

use pytext::configparser::RawConfigParser;
use pytext::pyre;
use pytext::shlex::Shlex;
use recording::Value;
use std::path::{Path, PathBuf};

/// Reference helpers: the klippy parser and snapshot of `parity/pytext-oracle/oracle.py`,
/// and `('ok', value)` / `('err', str(exc))` outcomes.
const PY_SETUP: &str = r#"
import configparser, io, os, re, shlex, glob, importlib.util
_spec = importlib.util.spec_from_file_location('pytext_oracle', os.path.join(REPO, 'parity', 'pytext-oracle', 'oracle.py'))
pytext_oracle = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pytext_oracle)

def outcome(fn):
    try:
        return ('ok', fn())
    except Exception as e:
        return ('err', str(e))

def parser(data, source):
    return pytext_oracle.klippy_parser(data, source)

def written(p):
    return pytext_oracle._write(p)

def lex(text):
    def tokens():
        l = shlex.shlex(text, posix=True)
        l.whitespace_split = True
        l.commenters = '#;'
        return list(l)
    return outcome(tokens)

def snapshot(relpath):
    # `parser_snapshot` of oracle.py, with the written text (its `written` is always None).
    def snap():
        with open(os.path.join(REPO, relpath), 'r') as f:
            data = f.read().replace('\r\n', '\n')
        p = parser(data, relpath)
        return {'sections': [{'name': name, 'items': [[k, v] for k, v in p.items(name)]}
                             for name in p.sections()],
                'defaults': [[k, v] for k, v in p.defaults().items()],
                'written': written(p)}
    return outcome(snap)
"#;

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../..")
}

fn oracle() -> pyoracle::Oracle {
    let mut py = pyoracle::Oracle::start().expect("oracle");
    let root = repo_root().to_string_lossy().replace('\\', "/");
    py.exec(&format!("REPO = {root:?}\n{PY_SETUP}")).unwrap_or_else(|e| panic!("oracle setup: {e:?}"));
    py
}

fn py_eval(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    py.eval(expr).unwrap_or_else(|e| panic!("oracle `{expr}`: {e:?}"))
}

/// A Python string literal for `s`.
fn lit(s: &str) -> String {
    let mut out = String::from("'");
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\'' => out.push_str("\\'"),
            ' '..='~' => out.push(c),
            _ => out.push_str(&format!("\\U{:08x}", c as u32)),
        }
    }
    out.push('\'');
    out
}

fn ok(v: Value) -> Value {
    Value::Tuple(vec![Value::str("ok"), v])
}

fn outcome<T, E: std::fmt::Display>(r: Result<T, E>, value: impl FnOnce(T) -> Value) -> Value {
    match r {
        Ok(v) => ok(value(v)),
        Err(e) => Value::Tuple(vec![Value::str("err"), Value::str(&e.to_string())]),
    }
}

fn opt_str(v: Option<String>) -> Value {
    v.map_or(Value::Null, |s| Value::str(&s))
}

fn strs(v: Vec<String>) -> Value {
    Value::List(v.iter().map(|s| Value::str(s)).collect())
}

fn items(v: Vec<(String, Option<String>)>) -> Value {
    Value::List(v.into_iter().map(|(k, v)| Value::Tuple(vec![Value::str(&k), opt_str(v)])).collect())
}

/// A klippy parser over `data` on both sides; `exprs` are `(python expression on p, port
/// value)` pairs compared in order.
fn check_parser(py: &mut pyoracle::Oracle, data: &str, source: &str, exprs: Vec<(&str, Value)>) {
    py.exec(&format!("p = parser({}, {})", lit(data), lit(source))).expect("python parser");
    for (expr, actual) in exprs {
        assert_eq!(actual, py_eval(py, &format!("outcome(lambda: {expr})")), "{expr} on {data:?}");
    }
}

fn read(data: &str, source: &str) -> RawConfigParser {
    let mut p = RawConfigParser::new_klippy();
    p.read_file(data, source).unwrap();
    p
}

#[test]
fn parses_simple_section() {
    let mut py = oracle();
    let data = "[printer]\nmax_velocity: 700\n";
    let p = read(data, "test.cfg");
    check_parser(
        &mut py,
        data,
        "test.cfg",
        vec![
            ("p.sections()", ok(strs(p.sections()))),
            ("p.get('printer', 'max_velocity')", outcome(p.get("printer", "max_velocity"), opt_str)),
            ("p.getint('printer', 'max_velocity')", outcome(p.getint("printer", "max_velocity"), Value::int)),
        ],
    );
}

#[test]
fn delimiters_and_optionxform() {
    let mut py = oracle();
    let data = "[s]\nKey = Value\nkey2: other\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![
            ("p.get('s', 'KEY')", outcome(p.get("s", "KEY"), opt_str)),
            ("p.get('s', 'key2')", outcome(p.get("s", "key2"), opt_str)),
        ],
    );
}

#[test]
fn inline_comments() {
    let mut py = oracle();
    let data = "[s]\na: 1 # comment\nb: 2 ; comment\nc: 3#not a comment\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![
            ("p.get('s', 'a')", outcome(p.get("s", "a"), opt_str)),
            ("p.get('s', 'b')", outcome(p.get("s", "b"), opt_str)),
            ("p.get('s', 'c')", outcome(p.get("s", "c"), opt_str)),
        ],
    );
}

#[test]
fn continuation_and_empty_lines() {
    let mut py = oracle();
    let data = "[s]\ngcode:\n    M104 S0\n    M140 S0\n";
    let p = read(data, "t.cfg");
    check_parser(&mut py, data, "t.cfg", vec![("p.get('s', 'gcode')", outcome(p.get("s", "gcode"), opt_str))]);
}

#[test]
fn duplicates_strict_false() {
    let mut py = oracle();
    let data = "[s]\na: 1\n[s]\na: 2\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![("p.sections()", ok(strs(p.sections()))), ("p.get('s', 'a')", outcome(p.get("s", "a"), opt_str))],
    );
}

#[test]
fn write_format() {
    let mut py = oracle();
    let data = "[s]\na: 1\nb: 2\n";
    let p = read(data, "t.cfg");
    check_parser(&mut py, data, "t.cfg", vec![("written(p)", ok(Value::str(&p.write())))]);
}

#[test]
fn missing_header_error() {
    let mut py = oracle();
    let mut p = RawConfigParser::new_klippy();
    let actual = outcome(p.read_file("a: 1\n", "my.cfg"), |_| Value::Null);
    assert_eq!(actual, py_eval(&mut py, "outcome(lambda: parser('a: 1\\n', 'my.cfg') and None)"));
}

#[test]
fn no_section_error() {
    let mut py = oracle();
    let p = RawConfigParser::new_klippy();
    check_parser(&mut py, "", "t.cfg", vec![("p.get('nope', 'x')", outcome(p.get("nope", "x"), opt_str))]);
}

#[test]
fn no_option_error() {
    let mut py = oracle();
    let data = "[s]\na: 1\n";
    let p = read(data, "t.cfg");
    check_parser(&mut py, data, "t.cfg", vec![("p.get('s', 'b')", outcome(p.get("s", "b"), opt_str))]);
}

#[test]
fn section_header_ends_at_first_bracket() {
    // `[include [ab].cfg]` names the glob `[ab`, `[a]b]` is section `a`.
    let mut py = oracle();
    for data in ["[include [ab]\nx: 1\n", "[a]b]\nx: 1\n", "[a] extra\nx: 1\n"] {
        let p = read(data, "t.cfg");
        let first = p.sections().first().cloned().unwrap_or_default();
        check_parser(
            &mut py,
            data,
            "t.cfg",
            vec![
                ("p.sections()", ok(strs(p.sections()))),
                ("p.get(p.sections()[0], 'x')", outcome(p.get(&first, "x"), opt_str)),
            ],
        );
    }
}

#[test]
fn options_rejects_empty_and_default_section() {
    let mut py = oracle();
    let data = "[DEFAULT]\ndopt: 9\n[a]\nx: 1\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![
            ("p.options('')", outcome(p.options(""), strs)),
            ("p.options('DEFAULT')", outcome(p.options("DEFAULT"), strs)),
            ("p.options('a')", outcome(p.options("a"), strs)),
            ("p.items('DEFAULT')", outcome(p.items("DEFAULT"), items)),
            ("p.get('DEFAULT', 'dopt')", outcome(p.get("DEFAULT", "dopt"), opt_str)),
            ("p.get('', 'dopt')", outcome(p.get("", "dopt"), opt_str)),
            ("p.items('')", outcome(p.items(""), items)),
        ],
    );
}

#[test]
fn control_whitespace_strips_like_python() {
    // `str.strip`/`str.isspace` treat U+001C..U+001F as whitespace.
    let mut py = oracle();
    let data = "[s]\na\x1c: 1\nb:\x1c2\x1c\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![("p.get('s', 'a')", outcome(p.get("s", "a"), opt_str)), ("p.get('s', 'b')", outcome(p.get("s", "b"), opt_str))],
    );
    let data = "[s]\na: 1 \x1c# c\n";
    let p = read(data, "t.cfg");
    check_parser(&mut py, data, "t.cfg", vec![("p.get('s', 'a')", outcome(p.get("s", "a"), opt_str))]);
    let data = "[s]\ng:\n\x1c  cont\n";
    let p = read(data, "t.cfg");
    check_parser(&mut py, data, "t.cfg", vec![("p.get('s', 'g')", outcome(p.get("s", "g"), opt_str))]);
    // `int()`/`float()` do not strip U+001C..U+001F mid-string.
    let data = "[s]\nn: 4\x1c2\nf: 1\x1c.5\n";
    let p = read(data, "t.cfg");
    check_parser(
        &mut py,
        data,
        "t.cfg",
        vec![
            ("p.get('s', 'n')", outcome(p.get("s", "n"), opt_str)),
            ("p.getint('s', 'n')", outcome(p.getint("s", "n"), Value::int)),
            ("p.getfloat('s', 'f')", outcome(p.getfloat("s", "f"), Value::float)),
        ],
    );
}

fn check_shlex(py: &mut pyoracle::Oracle, text: &str) {
    let mut lex = Shlex::new_klippy(text);
    assert_eq!(outcome(lex.tokens(), strs), py_eval(py, &format!("lex({})", lit(text))), "shlex {text:?}");
}

#[test]
fn shlex_basic() {
    let mut py = oracle();
    check_shlex(&mut py, "M104 S200");
}

#[test]
fn shlex_quotes_and_comments() {
    let mut py = oracle();
    check_shlex(&mut py, "M117 \"hello world\" ; comment");
}

#[test]
fn shlex_errors() {
    let mut py = oracle();
    check_shlex(&mut py, "FOO \"unclosed");
    check_shlex(&mut py, "FOO bar\\");
}

#[test]
fn re_split_gcode_args() {
    let mut py = oracle();
    let pattern = "([A-Z_]+|[A-Z*])";
    let r = pyre::compile(pattern).unwrap();
    assert_eq!(
        strs(r.split("G1 X10 Y20")),
        py_eval(&mut py, &format!("re.compile({}).split('G1 X10 Y20')", lit(pattern))),
    );
}

#[test]
fn re_match_m112() {
    let mut py = oracle();
    let pattern = "^(?:[nN][0-9]+)?\\s*[mM]112(?:\\s|$)";
    let r = pyre::compile(pattern).unwrap();
    for text in ["M112", "N10 M112", "M1120"] {
        assert_eq!(
            Value::Bool(r.is_match(text)),
            py_eval(&mut py, &format!("re.compile({}).search({}) is not None", lit(pattern), lit(text))),
            "match {text:?}"
        );
    }
}

#[test]
fn re_tilde_split() {
    let mut py = oracle();
    let pattern = "(\\~.*?\\~)";
    let r = pyre::compile(pattern).unwrap();
    assert_eq!(strs(r.split("a~b~c")), py_eval(&mut py, &format!("re.compile({}).split('a~b~c')", lit(pattern))));
}

/// The port's `re` subset names a pattern it does not implement (CPython compiles it);
/// a port-only limit with no reference counterpart.
#[test]
fn re_rejects_unknown() {
    let err = pyre::compile("[0-9]+").expect_err("must reject");
    let text = err.to_string();
    assert!(text.contains("unsupported"), "{text}");
    assert!(text.contains("[0-9]+"), "{text}");
}

/// The relative paths of the `.cfg` files directly in `dir` (relative to the repo root),
/// sorted like `sorted(glob.glob(...))`.
fn cfg_files(dir: &str) -> Vec<String> {
    let full = repo_root().join(dir);
    let mut out: Vec<String> = std::fs::read_dir(&full)
        .unwrap_or_else(|e| panic!("{}: {e}", full.display()))
        .map(|e| e.expect("dir entry").file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".cfg"))
        .map(|n| format!("{dir}/{n}"))
        .collect();
    out.sort();
    out
}

/// The reference `snapshot` (sections with items, defaults, written text) of the port's parser.
fn snapshot(p: &RawConfigParser) -> Value {
    let pairs = |v: Vec<(String, Option<String>)>| {
        Value::List(v.into_iter().map(|(k, v)| Value::List(vec![Value::str(&k), opt_str(v)])).collect())
    };
    let sections = p
        .sections()
        .into_iter()
        .map(|name| {
            let section_items = p.items(&name).unwrap_or_else(|e| panic!("items({name}): {e}"));
            Value::Dict(vec![(Value::str("name"), Value::str(&name)), (Value::str("items"), pairs(section_items))])
        })
        .collect();
    Value::Dict(vec![
        (Value::str("sections"), Value::List(sections)),
        (Value::str("defaults"), pairs(p.defaults())),
        (Value::str("written"), Value::str(&p.write())),
    ])
}

/// Every config file and shlex case of `parity/pytext-oracle/oracle.py`, compared with the
/// live reference.
#[test]
fn oracle_fixtures_consistent_when_present() {
    let mut py = oracle();
    let mut files = Vec::new();
    for dir in ["test/klippy", "config", "config/ender-3-v3-ke"] {
        files.extend(cfg_files(dir));
    }
    assert!(!files.is_empty(), "no config files found under {}", repo_root().display());
    let py_files = py_eval(
        &mut py,
        "[os.path.relpath(f, REPO) for d in ('test/klippy', 'config', 'config/ender-3-v3-ke') \
         for f in sorted(glob.glob(os.path.join(REPO, d, '*.cfg')))]",
    );
    assert_eq!(Value::List(files.iter().map(|f| Value::str(f)).collect()), py_files, "config file list");
    for rel in &files {
        let text = std::fs::read_to_string(repo_root().join(rel))
            .unwrap_or_else(|e| panic!("{rel}: {e}"))
            .replace("\r\n", "\n");
        let mut p = RawConfigParser::new_klippy();
        let actual = outcome(p.read_file(&text, rel), |_| snapshot(&p));
        assert_eq!(actual, py_eval(&mut py, &format!("snapshot({})", lit(rel))), "{rel}");
    }
    let shlex_cases = py_eval(&mut py, "[c['input'] for c in pytext_oracle.shlex_cases()]");
    let Value::List(cases) = shlex_cases else { panic!("shlex cases: {shlex_cases:?}") };
    assert!(!cases.is_empty(), "no shlex cases");
    for case in cases {
        let Value::Str(text) = case else { panic!("shlex case {case:?}") };
        check_shlex(&mut py, &text);
    }
}
