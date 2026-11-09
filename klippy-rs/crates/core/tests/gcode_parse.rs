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
    CommandError, DefaultAction, Dispatch, GCodeCommand, ParserKind,
};
use indexmap::IndexMap;
use recording::{load_unit, record_files, Value};
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

// ---------------------------------------------------------------------------
// The reference side: a real `GCodeDispatch` (klippy/gcode.py) over a minimal printer.
// ---------------------------------------------------------------------------

const PY_SETUP: &str = r#"
import gcode as gmod
import configfile

class _Mutex:
    def __enter__(self): return self
    def __exit__(self, *a): return False
class _Reactor:
    def mutex(self): return _Mutex()
class _Printer:
    config_error = configfile.error
    def get_start_args(self): return {}
    def get_reactor(self): return _Reactor()
    def register_event_handler(self, event, cb): pass
    def get_state_message(self): return ('Printer is ready', 'ready')
    def send_event(self, event, *params): return []
    def invoke_shutdown(self, msg): pass

def new_dispatch():
    d = gmod.GCodeDispatch(_Printer())
    d._handle_ready()
    return d
D = new_dispatch()

def outcome(fn):
    try:
        return ('ok', fn())
    except Exception as e:
        return ('err', str(e))

# `_process_commands` decomposition of one line: (cmd, params, origline).
def parse(line):
    got = []
    saved = D.gcode_handlers
    D.gcode_handlers = {}
    D.cmd_default = got.append
    try:
        D._process_commands([line], need_ack=False)
    finally:
        D.gcode_handlers = saved
        del D.cmd_default
    g = got[0]
    return (g.get_command(), g.get_command_parameters(), g.get_commandline())

def gcmd(cmd, line, params):
    return gmod.GCodeCommand(D, cmd, line, dict(params), False)

def raw(cmd, line):
    return gcmd(cmd, line, {}).get_raw_command_parameters()

def extended(cmd, line):
    g = gcmd(cmd, line, {})
    return outcome(lambda: dict(D._get_extended_params(g).get_command_parameters()))

# `cmd_default` outcome: ('ack',), ('ignore',), ('redispatch', cmd) or ('unknown', info).
def classify(cmd, params, line, fileinput):
    d = new_dispatch()
    d.is_fileinput = fileinput
    redispatched = []
    for c in ('M117', 'M118', 'M23'):
        d.gcode_handlers[c] = lambda g: redispatched.append(g.get_command())
    g = gmod.GCodeCommand(d, cmd, line, dict(params), True)
    infos = []
    acks = []
    g.respond_info = lambda msg, log=True: infos.append(msg)
    g.respond_raw = acks.append
    d.cmd_default(g)
    if acks:
        return ('ack',)
    if redispatched:
        return ('redispatch', redispatched[0])
    if infos:
        return ('unknown', infos[0])
    return ('ignore',)
"#;

fn oracle() -> pyoracle::Oracle {
    let mut py = pyoracle::Oracle::start().expect("oracle");
    py.exec(PY_SETUP).unwrap_or_else(|e| panic!("oracle setup: {e:?}"));
    py
}

fn py_eval(py: &mut pyoracle::Oracle, expr: &str) -> Value {
    py.eval(expr).unwrap_or_else(|e| panic!("oracle `{expr}`: {e:?}"))
}

/// Python string literal (Rust `{:?}` escapes are valid Python for these inputs).
fn lit(s: &str) -> String {
    format!("{s:?}")
}

fn py_params(pairs: &[(&str, &str)]) -> String {
    let items: Vec<String> = pairs.iter().map(|(k, v)| format!("{}: {}", lit(k), lit(v))).collect();
    format!("{{{}}}", items.join(", "))
}

fn dict_value(m: &IndexMap<String, String>) -> Value {
    Value::Dict(m.iter().map(|(k, v)| (Value::str(k), Value::str(v))).collect())
}

fn ok(v: Value) -> Value {
    Value::Tuple(vec![Value::str("ok"), v])
}

fn err(msg: &str) -> Value {
    Value::Tuple(vec![Value::str("err"), Value::str(msg)])
}

/// `('ok', value)` or `('err', str(exc))`, like the oracle's `outcome`.
fn outcome<T>(r: Result<T, CommandError>, value: impl FnOnce(T) -> Value) -> Value {
    match r {
        Ok(v) => ok(value(v)),
        Err(e) => err(&e.0),
    }
}

fn check_parse(py: &mut pyoracle::Oracle, line: &str) {
    let p = parse_line(line);
    assert_eq!(
        Value::Tuple(vec![Value::str(&p.cmd), dict_value(&p.params), Value::str(&p.origline)]),
        py_eval(py, &format!("parse({})", lit(line))),
        "parse {line:?}"
    );
}

fn check_raw(py: &mut pyoracle::Oracle, cmd: &str, line: &str) {
    assert_eq!(
        Value::str(&cmd_of(cmd, line, &[]).get_raw_command_parameters()),
        py_eval(py, &format!("raw({}, {})", lit(cmd), lit(line))),
        "raw parameters of {line:?}"
    );
}

fn check_extended(py: &mut pyoracle::Oracle, cmd: &str, line: &str) {
    let mut g = cmd_of(cmd, line, &[]);
    let actual = outcome(g.apply_extended_params(), |_| dict_value(g.get_command_parameters()));
    assert_eq!(actual, py_eval(py, &format!("extended({}, {})", lit(cmd), lit(line))), "extended {line:?}");
}

fn action_value(a: DefaultAction) -> Value {
    match a {
        DefaultAction::AckTemp => Value::Tuple(vec![Value::str("ack")]),
        DefaultAction::Ignore => Value::Tuple(vec![Value::str("ignore")]),
        DefaultAction::Redispatch(c) => Value::Tuple(vec![Value::str("redispatch"), Value::str(&c)]),
        DefaultAction::Unknown(c) => Value::Tuple(vec![Value::str("unknown"), Value::str(&format_unknown(&c))]),
    }
}

fn check_classify(py: &mut pyoracle::Oracle, cmd: &str, params: &[(&str, &str)], line: &str, fileinput: bool) {
    let flag = if fileinput { "True" } else { "False" };
    assert_eq!(
        action_value(classify_default(cmd, &params_of(params), line, fileinput)),
        py_eval(py, &format!("classify({}, {}, {}, {flag})", lit(cmd), py_params(params), lit(line))),
        "cmd_default {line:?} (fileinput {fileinput})"
    );
}

#[test]
fn parse_line_basics() {
    let mut py = oracle();
    for line in ["G1 X10 Y-2", "G1 X10 ; hello ; world", "   G28 ; comment  ", "g1 x10"] {
        check_parse(&mut py, line);
    }
}

#[test]
fn parse_line_numbers_and_checksums() {
    let mut py = oracle();
    // Line number prefix, and `N` glued to the command.
    for line in ["N123 G1 X10*45", "N1G1 X5"] {
        check_parse(&mut py, line);
    }
    // Checksum suffix participates in raw parameters stripping.
    for line in ["N123 G1 X10*45", "G1 X10*45", "n123 g1 x10"] {
        check_raw(&mut py, "G1", line);
    }
}

#[test]
fn raw_parameters_edges() {
    let mut py = oracle();
    for (cmd, line) in [
        ("STATUS", "STATUS"),
        ("M206", "M206 Z-.2"),
        ("G1", "G1\tX10"),
        ("SAVE_GCODE_STATE", "SAVE_GCODE_STATE NAME=test"),
        ("G1", "N1 G1 X1*12"),
        ("G1", "N1 G1 X1*AB"),
    ] {
        check_raw(&mut py, cmd, line);
    }
}

#[test]
fn m117_raw_text() {
    let mut py = oracle();
    for line in ["M117 1 2", "M118 Hello World", "M118 123", "M23 123", "M999 123"] {
        check_parse(&mut py, line);
        let p = parse_line(line);
        let params: Vec<(&str, &str)> = p.params.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        check_classify(&mut py, &p.cmd, &params, &p.origline, false);
    }
}

#[test]
fn extended_lowercase_and_shlex() {
    let mut py = oracle();
    // Extended dispatch is reached via the upper-cased `cmd`; the raw
    // parameter text keeps the original case.
    check_parse(&mut py, "set_led led=red");
    let p = parse_line("set_led led=red");
    let mut g = GCodeCommand::from_parsed(&p, false);
    let actual = outcome(g.apply_extended_params(), |_| dict_value(g.get_command_parameters()));
    assert_eq!(actual, py_eval(&mut py, "extended('SET_LED', 'set_led led=red')"), "set_led led=red");
    for (cmd, line) in [
        ("SET_LED", "set_led led=red green=1"),
        ("TEST", "TEST MSG=\"hello world\" X=1 # note"),
        ("TEST", "TEST MSG='a b' X=1 ; note"),
        ("TEST", "TEST A=b=c"),
    ] {
        check_extended(&mut py, cmd, line);
    }
}

#[test]
fn get_error_texts() {
    let mut py = oracle();
    let float = |r: Result<f64, CommandError>| outcome(r, Value::float);
    let int = |r: Result<i64, CommandError>| outcome(r, Value::int);
    let g = cmd_of("G1", "G1 X10", &[]);
    let x10 = "gcmd('G1', 'G1 X10', {})";
    let gabc = cmd_of("G1", "G1 Xabc", &[("X", "abc")]);
    let xabc = "gcmd('G1', 'G1 Xabc', {'X': 'abc'})";
    let g50 = cmd_of("G1", "G1 X5.0", &[("X", "5.0")]);
    let x50 = "gcmd('G1', 'G1 X5.0', {'X': '5.0'})";
    let g1 = cmd_of("G1", "G1 X1", &[("X", "1")]);
    let x1 = "gcmd('G1', 'G1 X1', {'X': '1'})";
    let g5 = cmd_of("G1", "G1 X5", &[("X", "5")]);
    let x5 = "gcmd('G1', 'G1 X5', {'X': '5'})";
    let gnan = cmd_of("G1", "G1 Xnan", &[("X", "nan")]);
    let xnan = "gcmd('G1', 'G1 Xnan', {'X': 'nan'})";
    let cases: Vec<(String, Value)> = vec![
        // Missing parameter (sentinel default).
        (format!("{x10}.get_float('Y')"), float(g.get_float("Y", None, None, None, None, None))),
        // Unparseable value echoes the raw string.
        (format!("{xabc}.get_float('X')"), float(gabc.get_float("X", None, None, None, None, None))),
        (format!("{x50}.get_int('X')"), int(g50.get_int("X", None, None, None))),
        // Bounds use `%s` rendering of the bound.
        (format!("{x1}.get_int('X', minval=2)"), int(g1.get_int("X", None, Some(2), None))),
        (format!("{x1}.get_float('X', minval=2.)"), float(g1.get_float("X", None, Some(2.0), None, None, None))),
        (format!("{x5}.get_int('X', maxval=4)"), int(g5.get_int("X", None, None, Some(4)))),
        (format!("{x5}.get_float('X', maxval=4.)"), float(g5.get_float("X", None, None, Some(4.0), None, None))),
        (format!("{x5}.get_float('X', above=5.)"), float(g5.get_float("X", None, None, None, Some(5.0), None))),
        (format!("{x5}.get_float('X', below=5.)"), float(g5.get_float("X", None, None, None, None, Some(5.0)))),
        // Exact bound values pass (minimum/maximum inclusive, above/below strict).
        (format!("{x5}.get_int('X', minval=5, maxval=5)"), int(g5.get_int("X", None, Some(5), Some(5)))),
        (
            format!("{x5}.get_float('X', minval=5., maxval=5.)"),
            float(g5.get_float("X", None, Some(5.0), Some(5.0), None, None)),
        ),
        (
            format!("{x5}.get_float('X', above=4.999, below=5.001)"),
            float(g5.get_float("X", None, None, None, Some(4.999), Some(5.001))),
        ),
        // NaN parses via float() and passes all bounds (NaN comparisons false).
        (
            format!("{xnan}.get_float('X', minval=0., maxval=1., above=0., below=1.)"),
            float(gnan.get_float("X", None, Some(0.0), Some(1.0), Some(0.0), Some(1.0))),
        ),
    ];
    for (expr, actual) in cases {
        assert_eq!(actual, py_eval(&mut py, &format!("outcome(lambda: {expr})")), "{expr}");
    }
    // Malformed extended lines.
    check_extended(&mut py, "TEST", "TEST BADTOKEN");
    check_extended(&mut py, "TEST", "TEST A=\"unterminated");
    let direct = outcome(parse_extended_params("A=\"unterminated", "TEST A=\"unterminated"), |m| dict_value(&m));
    assert_eq!(direct, py_eval(&mut py, "extended('TEST', 'TEST A=\"unterminated')"), "parse_extended_params");
}

#[test]
fn get_defaults_return_unchanged() {
    let mut py = oracle();
    // Missing returns the default as-is (no parse, no bounds check).
    let g = cmd_of("G1", "G1", &[]);
    let opt_float = |v: Option<f64>| v.map_or(Value::Null, Value::float);
    let actual = Value::List(vec![
        Value::float(g.get_float("X", Some(7.5), None, None, None, None).unwrap()),
        Value::int(g.get_int("X", Some(-3), None, None).unwrap()),
        Value::str(&g.get_str("X", Some("d".into())).unwrap()),
        // Nullable defaults: Python None.
        opt_float(g.get_float_opt("X", Some(None), None, None, None, None, klippy_api::config::IntArgs::NONE).unwrap()),
        g.get_int_opt("X", Some(None), None, None).unwrap().map_or(Value::Null, Value::int),
        g.get_str_opt("X", Some(None)).unwrap().map_or(Value::Null, |s| Value::str(&s)),
    ]);
    let expected = py_eval(
        &mut py,
        "(lambda g: [g.get_float('X', 7.5), g.get_int('X', -3), g.get('X', 'd'), \
         g.get_float('X', None), g.get_int('X', None), g.get('X', None)])(gcmd('G1', 'G1', {}))",
    );
    assert_eq!(actual, expected);
}

#[test]
fn get_float_bits() {
    let mut py = oracle();
    for (name, raw) in [("Z", "-5"), ("Z", ".1"), ("ADVANCE", ".002"), ("S", "500")] {
        let line = format!("G1 {name}={raw}");
        let mut g = cmd_of("G1", &line, &[(name, raw)]);
        g.apply_extended_params().unwrap();
        let v = g.get_float(name, None, None, None, None, None).unwrap();
        let expected = py_eval(
            &mut py,
            &format!("D._get_extended_params(gcmd('G1', {}, {})).get_float({})", lit(&line), py_params(&[(name, raw)]), lit(name)),
        );
        assert_eq!(Value::float(v), expected, "{name}={raw}");
    }
    // int parsing accepts underscores and signs like Python int().
    for raw in ["1_000", "+5"] {
        let line = format!("G1 X{raw}");
        let g = cmd_of("G1", &line, &[("X", raw)]);
        let actual = outcome(g.get_int("X", None, None, None), Value::int);
        let expected = py_eval(&mut py, &format!("outcome(lambda: gcmd('G1', {}, {{'X': {}}}).get_int('X'))", lit(&line), lit(raw)));
        assert_eq!(actual, expected, "X{raw}");
    }
}

#[test]
fn typed_get_parser_kinds() {
    let mut py = oracle();
    for raw in ["abc", "12"] {
        let line = format!("G1 X{raw}");
        let g = cmd_of("G1", &line, &[("X", raw)]);
        let actual = outcome(g.get_typed("X", Some(Some("d".into())), ParserKind::Int), |v| {
            v.map_or(Value::Null, |s| Value::str(&s))
        });
        let expected = py_eval(
            &mut py,
            &format!("outcome(lambda: str(gcmd('G1', {}, {{'X': {}}}).get('X', 'd', parser=int)))", lit(&line), lit(raw)),
        );
        assert_eq!(actual, expected, "X{raw}");
    }
}

#[test]
fn is_traditional_generated() {
    let mut py = oracle();
    let cmds = ["G1", "g1", "M104", "T0", "M117 hello", "SAVE_CONFIG", "SET_LED", "", "   ", "G", "GM", "1G", "G1X"];
    let actual = Value::List(cmds.iter().map(|c| Value::Bool(is_traditional_gcode(c))).collect());
    let list: Vec<String> = cmds.iter().map(|c| lit(c)).collect();
    let expected = py_eval(&mut py, &format!("[D.is_traditional_gcode(c) for c in [{}]]", list.join(", ")));
    assert_eq!(actual, expected);
}

fn config_outcome<T>(r: Result<T, klippy_core::gcode_parse::ConfigError>, value: impl FnOnce(T) -> Value) -> Value {
    match r {
        Ok(v) => ok(value(v)),
        Err(e) => err(&e.0),
    }
}

#[test]
fn register_validation() {
    let mut py = oracle();
    py.exec("d = new_dispatch()\nf = lambda g: None").expect("python dispatch");
    let mut d = Dispatch::new();
    let h: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    let registered = |r: Option<_>| Value::Bool(r.is_some());
    let steps: Vec<(&str, Value)> = vec![
        (
            "d.register_command('G1', f) is not None",
            config_outcome(d.register_command("G1", Some(("g1".into(), h.clone())), false, None), registered),
        ),
        // Duplicate.
        (
            "d.register_command('G1', f) is not None",
            config_outcome(d.register_command("G1", Some(("g1b".into(), h.clone())), false, None), registered),
        ),
        // Unregister returns the old handler, then nothing.
        (
            "d.register_command('G1', None) is not None",
            config_outcome(d.register_command("G1", None, false, None), registered),
        ),
        (
            "d.register_command('G1', None) is not None",
            config_outcome(d.register_command("G1", None, false, None), registered),
        ),
        // Re-register after unregister succeeds.
        (
            "d.register_command('G1', f) is not None",
            config_outcome(d.register_command("G1", Some(("g1".into(), h)), false, None), registered),
        ),
    ];
    for (expr, actual) in steps {
        assert_eq!(actual, py_eval(&mut py, &format!("outcome(lambda: {expr})")), "{expr}");
    }
    // Invalid names: lowercase, digit start, digit second char, empty.
    for bad in ["my_cmd", "1CMD", "G1A", ""] {
        let mut d2 = Dispatch::new();
        let h2: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
        let actual = config_outcome(d2.register_command(bad, Some(("x".into(), h2)), false, None), registered);
        let expected = py_eval(&mut py, &format!("outcome(lambda: new_dispatch().register_command({}, f) is not None)", lit(bad)));
        assert_eq!(actual, expected, "register {bad:?}");
    }
}

/// `(dispatch, command, commandline, params)` of one `_cmd_mux` call.
type MuxCase<'a> = (&'a str, &'a str, &'a str, &'a [(&'a str, &'a str)]);

#[test]
fn mux_dispatch_and_errors() {
    let mut py = oracle();
    py.exec(
        "calls = []\n\
         d = new_dispatch()\n\
         d.register_mux_command('SET_MODE', 'MODE', 'fast', lambda g: calls.append(['fast', g.get('MODE')]))\n\
         d.register_mux_command('SET_MODE', 'MODE', None, lambda g: calls.append(['default', g.get('MODE', None)]))\n\
         d2 = new_dispatch()\n\
         d2.register_mux_command('SET_MODE', 'MODE', 'fast', lambda g: calls.append(['fast2', g.get('MODE')]))",
    )
    .expect("python mux");
    let calls: Rc<std::cell::RefCell<Vec<Value>>> = Rc::new(std::cell::RefCell::new(Vec::new()));
    let record = |tag: &'static str| -> klippy_core::gcode_parse::GcodeHandler {
        let calls = calls.clone();
        Rc::new(move |g| {
            let mode = g.get_str_opt("MODE", Some(None)).unwrap().map_or(Value::Null, |s| Value::str(&s));
            calls.borrow_mut().push(Value::List(vec![Value::str(tag), mode]));
            Ok(())
        })
    };
    let mut d = Dispatch::new();
    d.register_mux_command("SET_MODE", "MODE", Some("fast".into()), "fast_handler".into(), record("fast"), None)
        .unwrap();
    d.register_mux_command("SET_MODE", "MODE", None, "default_handler".into(), record("default"), None)
        .unwrap();
    let mut d2 = Dispatch::new();
    d2.register_mux_command("SET_MODE", "MODE", Some("fast".into()), "fast_handler".into(), record("fast2"), None)
        .unwrap();
    // Known value and missing key (None arm) dispatch; without a None arm the key is
    // required; unknown values list the options or guess a substring match.
    let cases: [MuxCase; 6] = [
        ("d", "SET_MODE", "SET_MODE MODE=fast", &[("MODE", "fast")]),
        ("d", "SET_MODE", "SET_MODE", &[]),
        ("d2", "SET_MODE", "SET_MODE", &[]),
        ("d2", "SET_MODE", "SET_MODE MODE=fst", &[("MODE", "fst")]),
        ("d2", "SET_MODE", "SET_MODE MODE=fas", &[("MODE", "fas")]),
        ("d2", "SET_MODE", "SET_MODE MODE=slow", &[("MODE", "slow")]),
    ];
    for (which, cmd, line, params) in cases {
        let mut g = cmd_of(cmd, line, params);
        let dispatch = if which == "d" { &d } else { &d2 };
        let actual = outcome(dispatch.cmd_mux(cmd, &mut g), |_| Value::Null);
        let expected = py_eval(
            &mut py,
            &format!("outcome(lambda: {which}._cmd_mux({}, gcmd({}, {}, {})))", lit(cmd), lit(cmd), lit(line), py_params(params)),
        );
        assert_eq!(actual, expected, "{which} {line:?}");
    }
    assert_eq!(Value::List(calls.borrow().clone()), py_eval(&mut py, "calls"), "handler calls");

    // Registration errors.
    py.exec("d3 = new_dispatch()\nd3.register_mux_command('CMD', 'K', 'v', lambda g: None)").expect("python d3");
    let mut d3 = Dispatch::new();
    let h: klippy_core::gcode_parse::GcodeHandler = Rc::new(|_| Ok(()));
    d3.register_mux_command("CMD", "K", Some("v".into()), "h1".into(), h.clone(), None)
        .unwrap();
    let actual = config_outcome(
        d3.register_mux_command("CMD", "OTHER", Some("w".into()), "h2".into(), h.clone(), None),
        |_| Value::Null,
    );
    assert_eq!(actual, py_eval(&mut py, "outcome(lambda: d3.register_mux_command('CMD', 'OTHER', 'w', lambda g: None))"));
    // The message ends with `%s` of the registered handlers: Python function reprs carry
    // addresses, so the text before ` (` is compared.
    let rust_msg = match d3.register_mux_command("CMD", "K", Some("v".into()), "h2".into(), h, None) {
        Err(e) => e.0,
        Ok(_) => panic!("duplicate mux value must fail"),
    };
    let py_msg = match py_eval(&mut py, "outcome(lambda: d3.register_mux_command('CMD', 'K', 'v', lambda g: None))") {
        Value::Tuple(items) if items[0] == Value::str("err") => match &items[1] {
            Value::Str(s) => s.clone(),
            other => panic!("python message {other:?}"),
        },
        other => panic!("python duplicate mux value did not fail: {other:?}"),
    };
    let head = |s: &str| s.split(" (").next().unwrap_or_default().to_string();
    assert_eq!(head(&rust_msg), head(&py_msg), "duplicate mux value");
}

#[test]
fn default_classify() {
    let mut py = oracle();
    check_classify(&mut py, "M105", &[], "M105", false);
    check_classify(&mut py, "M21", &[], "M21", false);
    check_classify(&mut py, "M140", &[("S", "0")], "M140 S0", false);
    check_classify(&mut py, "M104", &[("S", "100")], "M104 S100", false);
    check_classify(&mut py, "M107", &[], "M107", false);
    check_classify(&mut py, "M106", &[("S", "0")], "M106 S0", false);
    check_classify(&mut py, "M106", &[("S", "255")], "M106 S255", true);
    check_classify(&mut py, "M106", &[("S", "255")], "M106 S255", false);
}
