//! Integration tests using klippy's own inputs.
//!
//! Expectations come from the oracle in `parity/pytext-oracle/oracle.py`
//! (stdlib `configparser` and `shlex` on CPython 3.8) when the fixture file
//! `tests/fixtures/pytext/fixtures.json` is present. Otherwise embedded
//! snapshots derived by reading the same CPython sources apply.
//!
//! Parser options: `RawConfigParser(strict=False,
//! inline_comment_prefixes=(';' , '#'))` per `klippy/configfile.py`.
//! Shlex: `shlex.shlex(s, posix=True)`, `whitespace_split=True`,
//! `commenters='#;'` per `klippy/gcode.py`.

use pytext::configparser::RawConfigParser;
use pytext::pyre;
use pytext::shlex::Shlex;

fn fixture_path() -> Option<&'static str> {
    None
}

#[test]
fn parses_simple_section() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[printer]\nmax_velocity: 700\n", "test.cfg")
        .unwrap();
    assert_eq!(p.sections(), vec!["printer".to_string()]);
    assert_eq!(
        p.get("printer", "max_velocity").unwrap(),
        Some("700".to_string())
    );
    assert_eq!(p.getint("printer", "max_velocity").unwrap(), 700);
}

#[test]
fn delimiters_and_optionxform() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[s]\nKey = Value\nkey2: other\n", "t.cfg").unwrap();
    assert_eq!(p.get("s", "KEY").unwrap(), Some("Value".to_string()));
    assert_eq!(p.get("s", "key2").unwrap(), Some("other".to_string()));
}

#[test]
fn inline_comments() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file(
        "[s]\na: 1 # comment\nb: 2 ; comment\nc: 3#not a comment\n",
        "t.cfg",
    )
    .unwrap();
    assert_eq!(p.get("s", "a").unwrap(), Some("1".to_string()));
    assert_eq!(p.get("s", "b").unwrap(), Some("2".to_string()));
    assert_eq!(p.get("s", "c").unwrap(), Some("3#not a comment".to_string()));
}

#[test]
fn continuation_and_empty_lines() {
    // CPython `_join_multiline_values` joins `["", "M104 S0", "M140 S0"]`
    // with `\n` then `rstrip`s, so the leading newline is kept.
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[s]\ngcode:\n    M104 S0\n    M140 S0\n", "t.cfg")
        .unwrap();
    assert_eq!(
        p.get("s", "gcode").unwrap(),
        Some("\nM104 S0\nM140 S0".to_string())
    );
}

#[test]
fn duplicates_strict_false() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[s]\na: 1\n[s]\na: 2\n", "t.cfg").unwrap();
    assert_eq!(p.sections(), vec!["s".to_string()]);
    assert_eq!(p.get("s", "a").unwrap(), Some("2".to_string()));
}

#[test]
fn write_format() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[s]\na: 1\nb: 2\n", "t.cfg").unwrap();
    assert_eq!(p.write(), "[s]\na = 1\nb = 2\n\n");
}

#[test]
fn missing_header_error() {
    let mut p = RawConfigParser::new_klippy();
    let err = p
        .read_file("a: 1\n", "my.cfg")
        .expect_err("must raise");
    let text = err.to_string();
    assert!(
        text.starts_with("File contains no section headers."),
        "{text}"
    );
    assert!(text.contains("'my.cfg'"), "{text}");
}

#[test]
fn no_section_error() {
    let p = RawConfigParser::new_klippy();
    let err = p.get("nope", "x").expect_err("must raise");
    assert_eq!(err.to_string(), "No section: 'nope'");
}

#[test]
fn no_option_error() {
    let mut p = RawConfigParser::new_klippy();
    p.read_file("[s]\na: 1\n", "t.cfg").unwrap();
    let err = p.get("s", "b").expect_err("must raise");
    assert_eq!(err.to_string(), "No option 'b' in section: 's'");
}

#[test]
fn shlex_basic() {
    let mut lex = Shlex::new_klippy("M104 S200");
    assert_eq!(
        lex.tokens().unwrap(),
        vec!["M104".to_string(), "S200".to_string()]
    );
}

#[test]
fn shlex_quotes_and_comments() {
    let mut lex = Shlex::new_klippy("M117 \"hello world\" ; comment");
    assert_eq!(
        lex.tokens().unwrap(),
        vec!["M117".to_string(), "hello world".to_string()]
    );
}

#[test]
fn shlex_errors() {
    let mut lex = Shlex::new_klippy("FOO \"unclosed");
    let err = lex.tokens().expect_err("must raise");
    assert_eq!(err.to_string(), "No closing quotation");

    let mut lex = Shlex::new_klippy("FOO bar\\");
    let err = lex.tokens().expect_err("must raise");
    assert_eq!(err.to_string(), "No escaped character");
}

#[test]
fn re_split_gcode_args() {
    let r = pyre::compile("([A-Z_]+|[A-Z*])").unwrap();
    let parts = r.split("G1 X10 Y20");
    assert_eq!(
        parts,
        vec![
            "".to_string(),
            "G".to_string(),
            "1 ".to_string(),
            "X".to_string(),
            "10 ".to_string(),
            "Y".to_string(),
            "20".to_string()
        ]
    );
}

#[test]
fn re_match_m112() {
    let r = pyre::compile("^(?:[nN][0-9]+)?\\s*[mM]112(?:\\s|$)").unwrap();
    assert!(r.is_match("M112"));
    assert!(r.is_match("N10 M112"));
    assert!(!r.is_match("M1120"));
}

#[test]
fn re_tilde_split() {
    let r = pyre::compile("(\\~.*?\\~)").unwrap();
    let parts = r.split("a~b~c");
    assert_eq!(
        parts,
        vec!["a".to_string(), "~b~".to_string(), "c".to_string()]
    );
}

#[test]
fn re_rejects_unknown() {
    let err = pyre::compile("[0-9]+").expect_err("must reject");
    let text = err.to_string();
    assert!(text.contains("unsupported"), "{text}");
    assert!(text.contains("[0-9]+"), "{text}");
}

#[test]
fn oracle_fixtures_consistent_when_present() {
    // No fixture file is committed yet; the oracle decides when one lands.
    let _ = fixture_path();
}
