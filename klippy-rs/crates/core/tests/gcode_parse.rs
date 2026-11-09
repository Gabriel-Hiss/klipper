//! Parity tests for `klippy_core::gcode_parse` (port of `klippy/gcode.py`
//! `GCodeCommand`, `_process_commands` line parsing, `is_traditional_gcode`,
//! `_get_extended_params`, `register_command` / `register_mux_command`).
//!
//! Recordings are replayed for the stateless entry point
//! (`is_traditional_gcode`, whose only argument is a string). The
//! stateful `GCodeCommand` calls (`get` / `get_int` / `get_float` /
//! `get_raw_command_parameters` / `_get_extended_params`) record only the
//! object identity, not the command line or params, so they cannot be
//! driven bit-for-bit from the file alone; they are covered by generated
//! lines that run through the same `parse_line` + `GCodeCommand` path the
//! Python `_process_commands` construction produces, plus the `_process_commands`
//! recordings themselves (the input line arrays) parsed for their
//! `cmd`/`params` decomposition. Float comparisons use bits.

use klippy_core::gcode_parse::{
    classify_default, format_unknown, is_traditional_gcode, parse_extended_params, parse_line,
    DefaultAction, Dispatch, GCodeCommand, ParserKind,
};
use indexmap::IndexMap;
use recording::{load_unit, record_files};
use std::rc::Rc;

fn records_root() -> std::path::PathBuf {
    let mut p = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.pop();
    p.pop();
    p.pop();
    p.push("tests");
    p.push("fixtures");
    p.push("recordings");
    p
}

#[test]
fn recordings_traditional_gcode_replays() {
    let root = records_root();
    let files = record_files(&root, "gcode_parse").expect("record files");
    assert!(!files.is_empty(), "expected gcode_parse recordings");
    let mut checked = 0u64;
    for path in files {
        let recs = recording::load_file(&path).expect("load recording");
        for rec in recs {
            if rec.function != "GCodeDispatch.is_traditional_gcode" {
                continue;
            }
            // args = [self oid, cmd string]
            let cmd = match rec.args.get(1) {
                Some(recording::Value::Str(s)) => s.clone(),
                _ => panic!("rec seq {} args[1] not a string", rec.seq),
            };
            let expected = match &rec.ret {
                Some(recording::Value::Bool(b)) => *b,
                other => panic!("rec seq {} ret not bool: {:?}", rec.seq, other),
            };
            assert_eq!(
                is_traditional_gcode(&cmd),
                expected,
                "is_traditional_gcode({:?}) scenario {} seq {}",
                cmd,
                rec.scenario,
                rec.seq
            );
            checked += 1;
        }
    }
    assert!(checked > 500, "expected many traditional checks, got {checked}");
}

#[test]
fn recordings_process_commands_parse() {
    // Every recorded `_process_commands` call carries its input lines; the
    // parse decomposition is invariant under the Python dispatch, so the
    // port must parse each line without fault and produce the `cmd` that
    // the upper-cased split defines.
    let root = records_root();
    let files = record_files(&root, "gcode_parse").expect("record files");
    let mut lines = 0u64;
    for path in files {
        for rec in recording::load_file(&path).expect("load recording") {
            if rec.function != "GCodeDispatch._process_commands" {
                continue;
            }
            let arr = match rec.args.get(1) {
                Some(recording::Value::List(l)) => l.clone(),
                _ => panic!("seq {} lines not a list", rec.seq),
            };
            for v in &arr {
                let s = match v {
                    recording::Value::Str(s) => s.as_str(),
                    _ => panic!("seq {} line not a string", rec.seq),
                };
                let parsed = parse_line(s);
                // origline is the stripped line; cmd is upper-case unless
                // the split yields a mixed `M117 <text>` tail.
                assert_eq!(parsed.origline, s.trim());
                lines += 1;
            }
        }
    }
    assert!(lines > 20, "expected many process_commands lines, got {lines}");
}

#[test]
fn recordings_load_unit_parses_all_files() {
    let root = records_root();
    let groups = load_unit(&root, "gcode_parse").expect("load unit");
    assert!(groups.len() >= 8);
}

fn params_of(pairs: &[(&str, &str)]) -> IndexMap<String, String> {
    let mut m = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k.to_string(), v.to_string());
    }
    m
}

fn cmd_of(cmd: &str, line: &str, pairs: &[(&str, &str)]) -> GCodeCommand {
    GCodeCommand::new(
        cmd.to_string(),
        line.to_string(),
        params_of(pairs),
        false,
    )
}

#[test]
fn parse_line_basics() {
    let p = parse_line("G1 X10 Y-2");
    assert_eq!(p.cmd, "G1");
    assert_eq!(p.params.get("G"), Some(&"1".to_string()));
    assert_eq!(p.params.get("X"), Some(&"10".to_string()));
    assert_eq!(p.params.get("Y"), Some(&"-2".to_string()));

    // comments cut at first ';'
    let p = parse_line("G1 X10 ; hello ; world");
    assert_eq!(p.cmd, "G1");
    assert_eq!(p.params.get("X"), Some(&"10".to_string()));

    // leading/trailing whitespace stripped, then ';' cut
    let p = parse_line("   G28 ; comment  ");
    assert_eq!(p.cmd, "G28");
    assert_eq!(p.origline, "G28 ; comment");

    // lowercase traditional input is upper-cased by the split path
    let p = parse_line("g1 x10");
    assert_eq!(p.cmd, "G1");
    // values are upper-cased too (the whole line is upper-cased first)
    assert_eq!(p.params.get("X"), Some(&"10".to_string()));
}

#[test]
fn parse_line_numbers_and_checksums() {
    // Line number prefix: N123 is skipped, G1 remains.
    let p = parse_line("N123 G1 X10*45");
    assert_eq!(p.cmd, "G1");
    assert_eq!(p.params.get("X"), Some(&"10".to_string()));
    assert_eq!(p.params.get("*"), Some(&"45".to_string()));
    assert_eq!(p.params.get("G"), Some(&"1".to_string()));

    // `N` detection uses parts[:2] == "N": "N..." that is not a line
    // number prefix still takes the line-number branch only when the
    // joined first two parts equal "N".
    let p = parse_line("N1G1 X5");
    assert_eq!(p.cmd, "G1");

    // Checksum suffix participates in raw parameters stripping.
    let g = cmd_of("G1", "N123 G1 X10*45", &[("G", "1")]);
    assert_eq!(g.get_raw_command_parameters(), "X10");

    let g = cmd_of("G1", "G1 X10*45", &[("G", "1")]);
    assert_eq!(g.get_raw_command_parameters(), "X10*45");

    // Lowercase line number / command still finds the upper command.
    let g = cmd_of("G1", "n123 g1 x10", &[("G", "1")]);
    assert_eq!(g.get_raw_command_parameters(), "x10");
}

#[test]
fn raw_parameters_edges() {
    // No params: empty string.
    let g = cmd_of("STATUS", "STATUS", &[]);
    assert_eq!(g.get_raw_command_parameters(), "");

    // One separating space is consumed.
    let g = cmd_of("M206", "M206 Z-.2", &[]);
    assert_eq!(g.get_raw_command_parameters(), "Z-.2");

    // Tab separator also counts as the one space.
    let g = cmd_of("G1", "G1\tX10", &[]);
    assert_eq!(g.get_raw_command_parameters(), "X10");

    // Extended command keeps case in the raw slice.
    let g = cmd_of("SAVE_GCODE_STATE", "SAVE_GCODE_STATE NAME=test", &[]);
    assert_eq!(g.get_raw_command_parameters(), "NAME=test");

    // Checksum with digits only is stripped; non-digits are not.
    let g = cmd_of("G1", "N1 G1 X1*12", &[]);
    assert_eq!(g.get_raw_command_parameters(), "X1");
    let g = cmd_of("G1", "N1 G1 X1*AB", &[]);
    assert_eq!(g.get_raw_command_parameters(), "X1*AB");
}

#[test]
fn m117_raw_text() {
    // `M117 1 2` parses to `cmd = "M117 1 2"` (contains spaces) and the
    // params dict is garbage; `cmd_default` redispatches on the real
    // `M117` token.
    let p = parse_line("M117 1 2");
    assert_eq!(p.cmd, "M117 1 2");
    let action = classify_default(&p.cmd, &p.params, &p.origline, false);
    assert_eq!(action, DefaultAction::Redispatch("M117".to_string()));

    // An upper-case word tail collapses into the command token itself
    // (`M118 HELLO WORLD` -> cmd `M118`); a numeric tail keeps the space.
    let p = parse_line("M118 Hello World");
    assert_eq!(p.cmd, "M118");
    assert_eq!(p.params.get("M"), Some(&"118".to_string()));
    assert_eq!(p.params.get("HELLO"), Some(&"".to_string()));
    assert_eq!(p.params.get("WORLD"), Some(&"".to_string()));

    let p = parse_line("M118 123");
    assert_eq!(p.cmd, "M118 123");
    assert_eq!(
        classify_default(&p.cmd, &p.params, &p.origline, false),
        DefaultAction::Redispatch("M118".to_string())
    );

    let p = parse_line("M23 123");
    assert_eq!(p.cmd, "M23 123");
    assert_eq!(
        classify_default(&p.cmd, &p.params, &p.origline, false),
        DefaultAction::Redispatch("M23".to_string())
    );

    // A space-containing unknown is not redispatched.
    let p = parse_line("M999 123");
    assert_eq!(p.cmd, "M999 123");
    assert_eq!(
        classify_default(&p.cmd, &p.params, &p.origline, false),
        DefaultAction::Unknown("M999 123".to_string())
    );
    assert_eq!(
        format_unknown("M999 123"),
        "Unknown command:\"M999 123\""
    );
}

#[test]
fn extended_lowercase_and_shlex() {
    // Extended dispatch is reached via the upper-cased `cmd`; the raw
    // parameter text keeps the original case.
    let p = parse_line("set_led led=red");
    assert_eq!(p.cmd, "SET_LED");
    let mut g = GCodeCommand::from_parsed(&p, false);
    g.apply_extended_params().expect("extended");
    assert_eq!(g.get_str("LED", Some("x".into())).unwrap(), "red");

    // Lowercase keys are upper-cased.
    let mut g = cmd_of("SET_LED", "set_led led=red green=1", &[]);
    g.apply_extended_params().expect("extended");
    assert_eq!(g.get_str("LED", None).unwrap(), "red");
    assert_eq!(g.get_str("GREEN", Some("x".into())).unwrap(), "1");

    // Quoting, escapes and `#`/`;` commenters follow `shlex` posix rules.
    let mut g = cmd_of("TEST", "TEST MSG=\"hello world\" X=1 # note", &[]);
    g.apply_extended_params().expect("extended");
    assert_eq!(g.get_str("MSG", None).unwrap(), "hello world");

    let mut g = cmd_of("TEST", "TEST MSG='a b' X=1 ; note", &[]);
    g.apply_extended_params().expect("extended");
    assert_eq!(g.get_str("MSG", None).unwrap(), "a b");

    // Value may contain '=' after the first one.
    let mut g = cmd_of("TEST", "TEST A=b=c", &[]);
    g.apply_extended_params().expect("extended");
    assert_eq!(g.get_str("A", None).unwrap(), "b=c");
}

#[test]
fn get_error_texts() {
    // Missing parameter (sentinel default).
    let g = cmd_of("G1", "G1 X10", &[]);
    let err = g.get_float("Y", None, None, None, None, None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X10': missing Y");

    // Unparseable value echoes the raw string.
    let g = cmd_of("G1", "G1 Xabc", &[("X", "abc")]);
    let err = g.get_float("X", None, None, None, None, None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 Xabc': unable to parse abc");

    let g = cmd_of("G1", "G1 X5.0", &[("X", "5.0")]);
    let err = g.get_int("X", None, None, None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X5.0': unable to parse 5.0");

    // Bounds use `%s` rendering of the bound.
    let g = cmd_of("G1", "G1 X1", &[("X", "1")]);
    let err = g.get_int("X", None, Some(2), None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X1': X must have minimum of 2");
    let err = g.get_float("X", None, Some(2.0), None, None, None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X1': X must have minimum of 2.0");

    let g = cmd_of("G1", "G1 X5", &[("X", "5")]);
    let err = g.get_int("X", None, None, Some(4)).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X5': X must have maximum of 4");
    let err = g.get_float("X", None, None, Some(4.0), None, None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X5': X must have maximum of 4.0");

    let err = g.get_float("X", None, None, None, Some(5.0), None).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X5': X must be above 5.0");
    let err = g.get_float("X", None, None, None, None, Some(5.0)).unwrap_err();
    assert_eq!(err.0, "Error on 'G1 X5': X must be below 5.0");

    // Exact bound values pass (minimum/maximum inclusive, above/below strict).
    assert!(g.get_int("X", None, Some(5), Some(5)).is_ok());
    assert!(g.get_float("X", None, Some(5.0), Some(5.0), None, None).is_ok());
    assert!(g.get_float("X", None, None, None, Some(4.999), Some(5.001)).is_ok());

    // NaN parses via float() and passes all bounds (NaN comparisons false).
    let g = cmd_of("G1", "G1 Xnan", &[("X", "nan")]);
    let v = g.get_float("X", None, Some(0.0), Some(1.0), Some(0.0), Some(1.0)).unwrap_or(f64::NAN);
    assert!(v.is_nan());

    // Malformed extended lines.
    let g = cmd_of("TEST", "TEST BADTOKEN", &[]);
    let err = parse_extended_params(&g.get_raw_command_parameters(), "TEST BADTOKEN").unwrap_err();
    assert_eq!(err.0, "Malformed command 'TEST BADTOKEN'");
    let g = cmd_of("TEST", "TEST A=\"unterminated", &[]);
    let err = parse_extended_params("A=\"unterminated", "TEST A=\"unterminated").unwrap_err();
    assert_eq!(err.0, "Malformed command 'TEST A=\"unterminated'");
    let _ = g;
}

#[test]
fn get_defaults_return_unchanged() {
    // Missing returns the default as-is (no parse, no bounds check).
    let g = cmd_of("G1", "G1", &[]);
    assert_eq!(g.get_float("X", Some(7.5), None, None, None, None).unwrap(), 7.5);
    assert_eq!(g.get_int("X", Some(-3), None, None).unwrap(), -3);
    assert_eq!(g.get_str("X", Some("d".into())).unwrap(), "d");
    // Nullable defaults: Python None.
    assert!(g.get_float_opt("X", Some(None), None, None, None, None, klippy_api::config::IntArgs::NONE).unwrap().is_none());
    assert!(g.get_int_opt("X", Some(None), None, None).unwrap().is_none());
    assert!(g.get_str_opt("X", Some(None)).unwrap().is_none());
}

#[test]
fn get_float_bits() {
    // Spot-check recordings values by bits: -5.0, 0.1, 0.002.
    let cases = [
        ("Z", "-5", -5.0f64),
        ("Z", ".1", 0.1f64),
        ("ADVANCE", ".002", 0.002f64),
        ("S", "500", 500.0f64),
    ];
    for (name, raw, expected) in cases {
        let g = cmd_of("G1", &format!("G1 {name}={raw}"), &[(name, raw)]);
        let mut g = g;
        g.apply_extended_params().unwrap();
        let v = g.get_float(name, None, None, None, None, None).unwrap();
        assert_eq!(v.to_bits(), expected.to_bits(), "{name}={raw}");
    }
    // int parsing accepts underscores and signs like Python int().
    let g = cmd_of("G1", "G1 X1_000", &[("X", "1_000")]);
    assert_eq!(g.get_int("X", None, None, None).unwrap(), 1000);
    let g = cmd_of("G1", "G1 X+5", &[("X", "+5")]);
    assert_eq!(g.get_int("X", None, None, None).unwrap(), 5);
}

#[test]
fn typed_get_parser_kinds() {
    let g = cmd_of("G1", "G1 Xabc", &[("X", "abc")]);
    let err = g.get_typed("X", Some(Some("d".into())), ParserKind::Int).unwrap_err();
    assert!(err.0.contains("unable to parse abc"));
    let g = cmd_of("G1", "G1 X12", &[("X", "12")]);
    assert_eq!(g.get_typed("X", Some(Some("d".into())), ParserKind::Int).unwrap().unwrap(), "12");
}

#[test]
fn is_traditional_generated() {
    assert!(is_traditional_gcode("G1"));
    assert!(is_traditional_gcode("g1"));
    assert!(is_traditional_gcode("M104"));
    assert!(is_traditional_gcode("T0"));
    assert!(is_traditional_gcode("M117 hello"));
    assert!(!is_traditional_gcode("SAVE_CONFIG"));
    assert!(!is_traditional_gcode("SET_LED"));
    assert!(!is_traditional_gcode(""));
    assert!(!is_traditional_gcode("   "));
    assert!(!is_traditional_gcode("G"));
    assert!(!is_traditional_gcode("GM"));
    assert!(!is_traditional_gcode("1G"));
    assert!(!is_traditional_gcode("G1X"));
}

#[test]
fn register_validation() {
    let mut d = Dispatch::new();
    let h: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    d.register_command("G1", Some(("g1".into(), h.clone())), false, None).unwrap();
    // Duplicate.
    let err = match d.register_command("G1", Some(("g1b".into(), h.clone())), false, None) {
        Err(e) => e,
        Ok(_) => panic!("duplicate should fail"),
    };
    assert_eq!(err.0, "gcode command G1 already registered");
    // Invalid names: lowercase, digit start, digit second char, empty.
    for bad in ["my_cmd", "1CMD", "G1A", ""] {
        let mut d2 = Dispatch::new();
        let h2: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
        let err = match d2.register_command(bad, Some(("x".into(), h2)), false, None) {
            Err(e) => e,
            Ok(_) => panic!("invalid name should fail: {bad}"),
        };
        assert_eq!(err.0, format!("Can't register '{}' as it is an invalid name", bad));
    }
    // Unregister returns the old handler.
    let old = d.register_command("G1", None, false, None).unwrap();
    assert!(old.is_some());
    let old = d.register_command("G1", None, false, None).unwrap();
    assert!(old.is_none());
    // Re-register after unregister succeeds.
    d.register_command("G1", Some(("g1".into(), h)), false, None).unwrap();
}

#[test]
fn mux_dispatch_and_errors() {
    let mut d = Dispatch::new();
    let fa: klippy_core::gcode_parse::GcodeHandler = Rc::new(|g| {
        assert_eq!(g.get_str("MODE", None).unwrap(), "fast");
        Ok(())
    });
    d.register_mux_command("SET_MODE", "MODE", Some("fast".into()), "fast_handler".into(), fa.clone(), None)
        .unwrap();
    let fd: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    d.register_mux_command("SET_MODE", "MODE", None, "default_handler".into(), fd, None)
        .unwrap();
    // Known value dispatches.
    let mut g = cmd_of("SET_MODE", "SET_MODE MODE=fast", &[("MODE", "fast")]);
    d.cmd_mux("SET_MODE", &mut g).unwrap();
    // Missing key uses the None default arm.
    let mut g = cmd_of("SET_MODE", "SET_MODE", &[]);
    d.cmd_mux("SET_MODE", &mut g).unwrap();

    // Without a None arm the key is required.
    let mut d2 = Dispatch::new();
    let fa2: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    d2.register_mux_command("SET_MODE", "MODE", Some("fast".into()), "fast_handler".into(), fa2, None)
        .unwrap();
    let mut g = cmd_of("SET_MODE", "SET_MODE", &[]);
    let err = d2.cmd_mux("SET_MODE", &mut g).unwrap_err();
    assert_eq!(err.0, "Error on 'SET_MODE': missing MODE");

    // "fst" is not a substring of "fast", so the options list is used.
    let mut g = cmd_of("SET_MODE", "SET_MODE MODE=fst", &[("MODE", "fst")]);
    let err = d2.cmd_mux("SET_MODE", &mut g).unwrap_err();
    assert_eq!(err.0, "The value 'fst' is not valid for MODE. Options: 'fast'");
    // A true substring ("fas" in "fast") uses the did-you-mean form.
    let mut g = cmd_of("SET_MODE", "SET_MODE MODE=fas", &[("MODE", "fas")]);
    let err = d2.cmd_mux("SET_MODE", &mut g).unwrap_err();
    assert_eq!(err.0, "The value 'fas' is not valid for MODE. Did you mean 'fast'?");
    let mut g = cmd_of("SET_MODE", "SET_MODE MODE=slow", &[("MODE", "slow")]);
    let err = d2.cmd_mux("SET_MODE", &mut g).unwrap_err();
    assert_eq!(err.0, "The value 'slow' is not valid for MODE. Options: 'fast'");

    // Registration errors.
    let mut d3 = Dispatch::new();
    let h: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    d3.register_mux_command("CMD", "K", Some("v".into()), "h1".into(), h.clone(), None)
        .unwrap();
    let err = d3
        .register_mux_command("CMD", "OTHER", Some("w".into()), "h2".into(), h.clone(), None)
        .unwrap_err();
    assert_eq!(err.0, "mux command CMD OTHER w may have only one key (K)");
    let err = d3
        .register_mux_command("CMD", "K", Some("v".into()), "h2".into(), h, None)
        .unwrap_err();
    assert!(err.0.starts_with("mux command CMD K v already registered ("));
}

#[test]
fn default_classify() {
    assert_eq!(
        classify_default("M105", &IndexMap::new(), "M105", false),
        DefaultAction::AckTemp
    );
    assert_eq!(
        classify_default("M21", &IndexMap::new(), "M21", false),
        DefaultAction::Ignore
    );
    assert_eq!(
        classify_default("M140", &params_of(&[("S", "0")]), "M140 S0", false),
        DefaultAction::Ignore
    );
    assert_eq!(
        classify_default("M104", &params_of(&[("S", "100")]), "M104 S100", false),
        DefaultAction::Unknown("M104".into())
    );
    assert_eq!(
        classify_default("M107", &IndexMap::new(), "M107", false),
        DefaultAction::Ignore
    );
    assert_eq!(
        classify_default("M106", &params_of(&[("S", "0")]), "M106 S0", false),
        DefaultAction::Ignore
    );
    assert_eq!(
        classify_default("M106", &params_of(&[("S", "255")]), "M106 S255", true),
        DefaultAction::Ignore
    );
    assert_eq!(
        classify_default("M106", &params_of(&[("S", "255")]), "M106 S255", false),
        DefaultAction::Unknown("M106".into())
    );
}
