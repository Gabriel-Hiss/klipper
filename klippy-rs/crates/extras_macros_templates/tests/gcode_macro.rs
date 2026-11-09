//! Parity tests for `klippy/extras/gcode_macro.py`.
//!
//! The test replays every recording in
//! `tests/fixtures/recordings/gcode_macro_template/*.jsonl` and renders
//! the macro corpus with contexts derived from the recordings'
//! `GetStatusWrapper.__getitem__` returns (converted to `PyValue` via
//! `recording::Value` -> `pycompat::PyValue`), plus hand-written
//! Jinja2 2.11.3 semantics cases under `jinja_*`.
//!
//! Object state is rebuilt the way Python built it:
//!
//! * `ConfigWrapper#N` scripts for the `None`-default loads are obtained
//!   by parsing the scenario's config with `pytext::RawConfigParser`
//!   and collecting `(section, option)` in file order. The `seq` order
//!   of `load_template` in each recording matches the order in which
//!   Klipper visits sections during `load_object` startup; when the
//!   recording carries an explicit script string (idle `gcode`, KE
//!   `on_error_gcode`/`runout_gcode`), that string is used directly.
//! * `GetStatusWrapper.__getitem__` returns are recorded dicts; they
//!   become the `printer` registry for renders (`printer[name]`).
//! * `TemplateWrapper.render` contexts are empty (the recorder passes
//!   object ids, not dict contents). Renders therefore use the
//!   macro-local context shape of `GCodeMacro.cmd`: macro variables +
//!   `printer` + `params`/`rawparams` + `action_*` stubs. Expected
//!   output is the recording's `ret` string, compared exactly.
//!
//! Integration owner runs:
//!
//! ```sh
//! wsl -d Ubuntu -u root -- bash -lc "mountpoint -q /mnt/o || (mkdir -p /mnt/o && mount -t drvfs O: /mnt/o); cd /mnt/o/omp-pilotgcodemacro-41b93f/klippy-rs && CARGO_TARGET_DIR=/root/target/omp-pilotgcodemacro-41b93f cargo test -p extras_macros_templates --test gcode_macro"
//! ```

use extras_macros_templates::gcode_macro::{
    GetStatusWrapper, GCodeMacroVars, StatusSource, Template, TemplateWrapper, Value,
};
use indexmap::IndexMap;
use pycompat::int::PyInt;
use pycompat::value::PyValue;
use pytext::configparser::RawConfigParser;
use recording::load_file;
use recording::Value as RValue;
use std::cell::RefCell;
use std::path::{Path, PathBuf};
use std::rc::Rc;

fn repo_root() -> PathBuf {
    // `CARGO_MANIFEST_DIR` is `klippy-rs/crates/<crate>`; repo root is three levels up.
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .and_then(|p| p.parent())
        .and_then(|p| p.parent())
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."))
}

fn rec_root() -> PathBuf {
    repo_root().join("tests/fixtures/recordings/gcode_macro_template")
}

fn to_py(rc: &RValue) -> PyValue {
    match rc {
        RValue::Null => PyValue::None,
        RValue::Bool(b) => PyValue::Bool(*b),
        RValue::Int(i) => PyValue::Int(PyInt::from_i64(*i)),
        RValue::BigInt(s) => {
            let parsed = pycompat::text::int_from_string(s, 10).unwrap_or_else(|e| {
                panic!("bad bigint {s}: {e}")
            });
            PyValue::Int(parsed)
        }
        RValue::Float(f) => PyValue::Float(f.to_f64().unwrap_or(f64::NAN)),
        RValue::Bytes(b) => PyValue::Bytes(b.clone()),
        RValue::Str(s) => PyValue::Str(s.clone()),
        RValue::List(items) => PyValue::List(items.iter().map(to_py).collect()),
        RValue::Tuple(items) => PyValue::Tuple(items.iter().map(to_py).collect()),
        RValue::Dict(pairs) => {
            let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
            for (k, v) in pairs {
                d.insert(to_py(k), to_py(v));
            }
            PyValue::Dict(d)
        }
        RValue::Struct { .. } => PyValue::None,
        RValue::Opaque { .. } => PyValue::None,
        RValue::ObjectId(_) => PyValue::None,
    }
}

/// Registry captured from recorded `GetStatusWrapper` returns.
fn registry_from_recordings(scenario: &str) -> IndexMap<String, PyValue> {
    registry_from_recordings_upto(scenario, u64::MAX)
}

/// Same as [`registry_from_recordings`], but only `GetStatusWrapper`
/// lookups at or before `max_seq` count, rebuilding the printer state
/// as Python saw it at a given recorded render.
fn registry_from_recordings_upto(
    scenario: &str,
    max_seq: u64,
) -> IndexMap<String, PyValue> {
    let path = rec_root().join(format!("{scenario}.jsonl"));
    let records = load_file(&path).unwrap_or_default();
    let mut reg: IndexMap<String, PyValue> = IndexMap::new();
    for r in &records {
        if r.seq > max_seq {
            continue;
        }
        if r.function == "GetStatusWrapper.__getitem__" {
            if let Some(RValue::Str(name)) = r.args.get(1) {
                if let Some(ret) = &r.ret {
                    if *ret != RValue::Null {
                        reg.insert(name.clone(), to_py(ret));
                    }
                }
            }
        }
    }
    reg
}

fn config_scripts(config_path: &str) -> Vec<(String, String, String)> {
    // (section, option, value) in file order for `gcode`-like options plus
    // `text`, `on_error_gcode`, `runout_gcode`, `insert_gcode`,
    // `press_gcode`, `release_gcode`, `name`, `enable`, `input*`.
    let full = repo_root().join(config_path);
    let text = std::fs::read_to_string(&full).unwrap_or_default();
    let mut p = RawConfigParser::new_klippy();
    p.read_string(&text).unwrap();
    let mut out = Vec::new();
    let wanted = [
        "gcode",
        "text",
        "on_error_gcode",
        "runout_gcode",
        "insert_gcode",
        "press_gcode",
        "release_gcode",
        "name",
        "enable",
        "input",
        "input_min",
        "input_max",
    ];
    for section in p.sections() {
        for (opt, val) in p.items(&section).unwrap_or_default() {
            if wanted.contains(&opt.as_str()) {
                if let Some(v) = val {
                    out.push((section.clone(), opt, v));
                }
            }
        }
    }
    out
}

fn param_value(s: &str) -> PyValue {
    // `params` values are command strings in Klipper.
    PyValue::Str(s.to_string())
}

/// Recorded `get_status` results standing in for the printer's objects.
struct RecordedStatuses(IndexMap<String, PyValue>);

impl StatusSource for RecordedStatuses {
    fn object_names(&self) -> Vec<String> {
        self.0.keys().cloned().collect()
    }
    fn get_status(&self, name: &str, _eventtime: &mut Option<f64>) -> Option<PyValue> {
        self.0.get(name).cloned()
    }
}

fn make_ctx(
    printer: &PyValue,
    respond: Rc<RefCell<Vec<String>>>,
    raise: Rc<RefCell<Option<String>>>,
) -> IndexMap<String, Value> {
    let PyValue::Dict(objects) = printer else {
        panic!("printer registry must be a dict, got {printer:?}");
    };
    let statuses = objects
        .iter()
        .map(|(k, v)| (k.str(), v.clone()))
        .collect();
    let mut ctx: IndexMap<String, Value> = IndexMap::new();
    ctx.insert(
        "printer".to_string(),
        Value::printer_status(Box::new(RecordedStatuses(statuses)), None),
    );
    let r1 = respond.clone();
    ctx.insert(
        "action_respond_info".to_string(),
        Value::Func(Rc::new(move |args, _kwargs, _ln| {
            let msg = match args.get(0) {
                Some(Value::Py(PyValue::Str(s))) => s.clone(),
                Some(v) => v.to_string(),
                None => String::new(),
            };
            r1.borrow_mut().push(msg);
            Ok(Value::py(PyValue::Str(String::new())))
        })),
    );
    let r2 = raise.clone();
    ctx.insert(
        "action_raise_error".to_string(),
        Value::Func(Rc::new(move |args, _kwargs, ln| {
            let msg = match args.get(0) {
                Some(Value::Py(PyValue::Str(s))) => s.clone(),
                Some(v) => v.to_string(),
                None => String::new(),
            };
            *r2.borrow_mut() = Some(msg.clone());
            Err(extras_macros_templates::gcode_macro::TemplateError::new(
                "TemplateError",
                msg,
                Some(ln),
            ))
        })),
    );
    let r3 = Rc::new(RefCell::new(Vec::new()));
    ctx.insert(
        "action_emergency_stop".to_string(),
        Value::Func(Rc::new(move |args, _kwargs, _ln| {
            let msg = match args.get(0) {
                Some(Value::Py(PyValue::Str(s))) => s.clone(),
                Some(v) => v.to_string(),
                None => "action_emergency_stop".to_string(),
            };
            r3.borrow_mut().push(msg);
            Ok(Value::py(PyValue::Str(String::new())))
        })),
    );
    let r4 = Rc::new(RefCell::new(Vec::new()));
    ctx.insert(
        "action_call_remote_method".to_string(),
        Value::Func(Rc::new(move |args, _kwargs, _ln| {
            let m = match args.get(0) {
                Some(Value::Py(PyValue::Str(s))) => s.clone(),
                Some(v) => v.to_string(),
                None => String::new(),
            };
            r4.borrow_mut().push(m);
            Ok(Value::py(PyValue::Str(String::new())))
        })),
    );
    ctx
}

fn dict_of(pairs: Vec<(&str, PyValue)>) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        d.insert(PyValue::Str(k.to_string()), v);
    }
    PyValue::Dict(d)
}

fn float_v(f: f64) -> PyValue {
    PyValue::Float(f)
}

/// Read a config file the way `klippy/configfile.py` does: each line is
/// truncated at the first `#` before it reaches configparser
/// (`ConfigFileReader.append_fileconfig`), so standalone comment lines
/// become blank lines, which are then kept inside multiline values
/// (`empty_lines_in_values`). Feeding Jinja the script strings Klippy
/// produced is what the recordings captured.
fn klippy_read_config(text: &str) -> RawConfigParser {
    let normalized = text.replace("\r\n", "\n");
    let stripped: Vec<&str> = normalized
        .split('\n')
        .map(|line| match line.find('#') {
            Some(pos) => &line[..pos],
            None => line,
        })
        .collect();
    let joined = stripped.join("\n");
    let mut p = RawConfigParser::new_klippy();
    p.read_string(&joined).unwrap();
    p
}

// ---------------------------------------------------------------------------
// Recording replay: every load_template parses; explicit scripts checked.
// ---------------------------------------------------------------------------

#[test]
fn recordings_replay_loads() {
    let root = rec_root();
    let files = [
        "commands-001-example-cartesian.jsonl",
        "gcode_arcs-001-gcode-arcs.jsonl",
        "input_shaper-001-input-shaper.jsonl",
        "ke-batch-move.jsonl",
        "macros-001-macros.jsonl",
        "pressure_advance-001-pressure-advance.jsonl",
        "printers-070-printer-creality-ender3-2018.jsonl",
        "temperature-001-temperature.jsonl",
    ];
    for file in files {
        let records = load_file(&root.join(file)).expect("load recording");
        assert!(!records.is_empty(), "{file} must have records");
        // scenario -> config for None-default scripts
        let scenario = records[0].scenario.clone();
        let config_path: Option<&str> = match scenario.as_str() {
            "macros-001-macros" => Some("test/klippy/macros.cfg"),
            "ke-batch-move" => Some("config/ender-3-v3-ke/printer.cfg"),
            "printers-070-printer-creality-ender3-2018" => {
                Some("config/printer-creality-ender3-2018.cfg")
            }
            _ => None,
        };
        let scripts = config_path
            .map(|c| {
                let mut all = config_scripts(c);
                // KE includes mainsail + gcode_macro.cfg
                if scenario == "ke-batch-move" {
                    all.extend(config_scripts(
                        "config/ender-3-v3-ke/gcode_macro.cfg",
                    ));
                    all.extend(config_scripts(
                        "parity/scenarios/ke/ke-batch-move/mnt/usr/data/mainsail-config/mainsail.cfg",
                    ));
                }
                all
            })
            .unwrap_or_default();
        let mut none_idx = 0usize;
        for r in &records {
            if r.function != "PrinterGCodeMacro.load_template" {
                continue;
            }
            // args: [self oid, config oid, option, default]
            let option = match r.args.get(2) {
                Some(RValue::Str(s)) => s.clone(),
                _ => continue,
            };
            let default = match r.args.get(3) {
                Some(RValue::Str(s)) => Some(s.clone()),
                _ => None,
            };
            let script = match &default {
                Some(s) => s.clone(),
                None => {
                    // lookup next script matching option in order
                    let mut found = None;
                    while none_idx < scripts.len() {
                        let s = &scripts[none_idx];
                        none_idx += 1;
                        if s.1 == option {
                            found = Some(s.2.clone());
                            break;
                        }
                    }
                    match found {
                        Some(s) => s,
                        None => {
                            // recordings include templates from display/menu
                            // configs not in the paid corpus; parsing is still
                            // verified below with empty config fallback.
                            continue;
                        }
                    }
                }
            };
            // name is "config.get_name():option" in Python; the section
            // prefix is unknown from OIDs; use option as name.
            let name = format!("test:{option}");
            match TemplateWrapper::load(&name, &script) {
                Ok(_) => {}
                Err(msg) => panic!("{file} seq {}: load failed: {msg}", r.seq),
            }
        }
    }
}

// ---------------------------------------------------------------------------
// macros-001 renders: compare against recorded `ret` strings exactly.
// ---------------------------------------------------------------------------

#[test]
fn macros_renders_match_recordings() {
    let scenario = "macros-001-macros";
    let reg = registry_from_recordings(scenario);
    // `gcode_move` and `toolhead` etc. exist; build printer dict.
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in &reg {
        d.insert(PyValue::Str(k.clone()), v.clone());
    }
    let printer = PyValue::Dict(d);

    // Templates fetched from config (same order Python loaded them).
    let scripts = config_scripts("test/klippy/macros.cfg");
    let get = |opt: &str, sub: &str| -> String {
        for (sec, o, v) in &scripts {
            if o == opt && sec.contains(sub) {
                return v.clone();
            }
        }
        panic!("script {opt} {sub} not found")
    };

    // TESTIT (seq 11)
    {
        let s = get("gcode", "TESTIT");
        // `[gcode_macro TESTIT]` section name is "gcode_macro TESTIT"
        let w = TemplateWrapper::load("gcode_macro TESTIT:gcode", &s).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        let expect = "\nTEST_SAVE_RESTORE\nTEST_expression\nTEST_variable\nTEST_param T=123\nTEST_unicode\nTEST_in";
        assert_eq!(got, expect);
    }
    // TEST_SAVE_RESTORE (seq 13)
    {
        let s = get("gcode", "TEST_SAVE_RESTORE");
        let w = TemplateWrapper::load("gcode_macro TEST_SAVE_RESTORE:gcode", &s).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(
            got,
            "\nSAVE_GCODE_STATE NAME=TESTIT1\nG92 Z10\nRESTORE_GCODE_STATE NAME=TESTIT1\nG92 Z0"
        );
    }
    // TEST_expression (seq 17): both branches eval to empty-ish lines.
    {
        let s = get("gcode", "TEST_expression");
        let w = TemplateWrapper::load("test", &s).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond.clone(), raise);
        let got = w.render(ctx).unwrap();
        // Action is called (respond) and the template yields "\n\n\n"
        // under gcode_move.x == 0.0 (else branch).
        assert_eq!(got, "\n\n\n");
        assert_eq!(respond.borrow().as_slice(), ["TEST_expression"]);
    }
    // TEST_variable (seq 22)
    {
        let s = get("gcode", "TEST_variable");
        // There are TEST_variable and TEST_variable_part2; match exact section.
        let mut picked = None;
        for (sec, o, v) in &scripts {
            if sec == "gcode_macro TEST_variable" && o == "gcode" {
                picked = Some(v.clone());
            }
        }
        let _ = s;
        let raw = picked.unwrap();
        let w = TemplateWrapper::load("test", &raw).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        // At the seq-22 render Python saw `t=12.0` (seq 21 lookup); the
        // later `t=17` snapshot (seq 24) belongs to TEST_variable_part2.
        let reg_at_render = registry_from_recordings_upto(scenario, 21);
        let mut d22: IndexMap<PyValue, PyValue> = IndexMap::new();
        for (k, v) in &reg_at_render {
            d22.insert(PyValue::Str(k.clone()), v.clone());
        }
        let printer22 = PyValue::Dict(d22);
        let mut ctx = make_ctx(&printer22, respond.clone(), raise);
        // `t` macro variable comes from `variable_t: 12.0`
        ctx.insert("t".to_string(), Value::py(float_v(12.0)));
        let got = w.render(ctx).unwrap();
        assert_eq!(
            got,
            "\n\n\n\nSET_GCODE_VARIABLE MACRO=TEST_variable VARIABLE=t VALUE=17\nTEST_variable_part2"
        );
        assert_eq!(respond.borrow().as_slice(), ["TEST_variable"]);
    }
    // TEST_variable_part2 (seq 25 -> "\n\n")
    {
        let mut raw = None;
        for (sec, o, v) in &scripts {
            if sec == "gcode_macro TEST_variable_part2" && o == "gcode" {
                raw = Some(v.clone());
            }
        }
        let w = TemplateWrapper::load("test", &raw.unwrap()).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond.clone(), raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n");
        assert_eq!(
            respond.borrow().as_slice(),
            ["TEST_variable_part2"]
        );
    }
    // TEST_param (seq 29 -> "\n\n")
    {
        let mut raw = None;
        for (sec, o, v) in &scripts {
            if sec == "gcode_macro TEST_param" && o == "gcode" {
                raw = Some(v.clone());
            }
        }
        let w = TemplateWrapper::load("test", &raw.unwrap()).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let mut ctx = make_ctx(&printer, respond.clone(), raise);
        let mut pd: IndexMap<PyValue, PyValue> = IndexMap::new();
        pd.insert(PyValue::Str("T".to_string()), param_value("123"));
        ctx.insert("params".to_string(), Value::py(PyValue::Dict(pd)));
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n");
        assert_eq!(respond.borrow().as_slice(), ["TEST_param"]);
    }
    // TEST_unicode (seq 32 -> "G28")
    {
        let mut raw = None;
        for (sec, o, v) in &scripts {
            if sec == "gcode_macro TEST_unicode" && o == "gcode" {
                raw = Some(v.clone());
            }
        }
        let w = TemplateWrapper::load("test", &raw.unwrap()).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "G28");
    }
    // TEST_in (seq 37 -> "\n")
    {
        let mut raw = None;
        for (sec, o, v) in &scripts {
            if sec == "gcode_macro TEST_in" && o == "gcode" {
                raw = Some(v.clone());
            }
        }
        let w = TemplateWrapper::load("test", &raw.unwrap()).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n");
    }
}

// ---------------------------------------------------------------------------
// Recorded GetStatusWrapper behavior (KeyError/None for missing objects).
// ---------------------------------------------------------------------------

#[test]
fn get_status_wrapper_matches_recordings() {
    // seq 35: `printer["abc"]` -> null in recording (KeyError path).
    let reg = registry_from_recordings("macros-001-macros");
    let w = GetStatusWrapper::new(Box::new(RecordedStatuses(reg.clone())), None);
    assert!(w.get_item("abc").is_none());
    assert!(w.get_item("toolhead").is_some());
    // stripping
    assert!(w.get_item("  toolhead  ").is_some());
    assert!(w.contains("toolhead"));
    assert!(!w.contains("abc"));
    // iteration yields status object names
    assert!(w.iter_names().contains(&"toolhead".to_string()));
    // macros seq 16/20 etc. values round-trip through `to_py`.
    let v = w.get_item("gcode_move").unwrap();
    if let PyValue::Dict(d) = v {
        // speed 1500.0 from recording hex 0x1.77p+10
        let speed = d
            .get(&PyValue::Str("speed".to_string()))
            .cloned()
            .unwrap();
        assert_eq!(speed, PyValue::Float(1500.0));
    } else {
        panic!("gcode_move should be dict");
    }
}

// ---------------------------------------------------------------------------
// printersen-070 display renders (display.cfg subset) vs recordings.
// ---------------------------------------------------------------------------

#[test]
fn printers070_display_renders_match() {
    let reg = registry_from_recordings("printers-070-printer-creality-ender3-2018");
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in &reg {
        d.insert(PyValue::Str(k.clone()), v.clone());
    }
    let printer = PyValue::Dict(d);

    let display_text = std::fs::read_to_string(
        repo_root().join("klippy/extras/display/display.cfg"),
    )
    .unwrap();
    let p = klippy_read_config(&display_text);
    let get_text = |section: &str| -> String {
        p.get(section, "text").unwrap().unwrap()
    };

    // `_heater_temperature` with param_heater_name=extruder (seq 307).
    {
        let script = get_text("display_template _heater_temperature");
        let w = TemplateWrapper::load("display_template _heater_temperature:text", &script).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let mut ctx = make_ctx(&printer, respond, raise);
        ctx.insert(
            "param_heater_name".to_string(),
            Value::py(PyValue::Str("extruder".to_string())),
        );
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n\n\n\n~extruder~\n\n\n  0\n\n\n~degrees~\n");
    }
    // `_heater_temperature` heater_bed (seq 315).
    {
        let script = get_text("display_template _heater_temperature");
        let w = TemplateWrapper::load("t", &script).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let mut ctx = make_ctx(&printer, respond, raise);
        ctx.insert(
            "param_heater_name".to_string(),
            Value::py(PyValue::Str("heater_bed".to_string())),
        );
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n\n\n\n\n~bed~\n\n\n\n  0\n\n\n~degrees~\n");
    }
    // `_fan_speed` (seq 311).
    {
        let script = get_text("display_template _fan_speed");
        let w = TemplateWrapper::load("t", &script).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n\n\n~fan1~\n\n  0%\n");
    }
    // `_printing_time` (seq 324).
    {
        let script = get_text("display_template _printing_time");
        let w = TemplateWrapper::load("t", &script).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n-1:59");
    }
    // `_print_status` (seq 329, position present).
    {
        let script = get_text("display_template _print_status");
        let w = TemplateWrapper::load("t", &script).unwrap();
        let respond = Rc::new(RefCell::new(Vec::new()));
        let raise = Rc::new(RefCell::new(None));
        let ctx = make_ctx(&printer, respond, raise);
        let got = w.render(ctx).unwrap();
        assert_eq!(got, "\n\n\nX0   Y0   Z1.00 \n");
    }
}

// ---------------------------------------------------------------------------
// Corpus renders: every macro in the paid corpus parses, and key macros
// render exactly under representative contexts.
// ---------------------------------------------------------------------------

#[test]
fn corpus_templates_all_parse() {
    let configs = [
        "config/sample-macros.cfg",
        "config/sample-mmu2s-diy.cfg",
        "config/sample-pwm-tool.cfg",
        "config/ender-3-v3-ke/gcode_macro.cfg",
        "parity/scenarios/ke/ke-batch-move/mnt/usr/data/mainsail-config/mainsail.cfg",
        "test/klippy/macros.cfg",
        "test/klippy/exclude_object.cfg",
        "test/klippy/sdcard_loop.cfg",
        "test/klippy/generic_cartesian_iqex.cfg",
        "test/klippy/generic_cartesian_itex.cfg",
        "config/printer-geeetech-A10T-A20T-2021.cfg",
        "config/printer-lulzbot-taz6-dual-v3-2017.cfg",
        "config/printer-anycubic-4maxpro-2.0-2021.cfg",
        "config/sample-raspberry-pi.cfg",
        "config/printer-sunlu-t3-2022.cfg",
        "config/printer-velleman-k8800-2017.cfg",
        "config/printer-creality-cr30-2021.cfg",
        "klippy/extras/display/display.cfg",
        "klippy/extras/display/menu.cfg",
    ];
    for cfg in configs {
        for (section, opt, val) in config_scripts(cfg) {
            TemplateWrapper::load(&format!("{section}:{opt}"), &val).unwrap_or_else(|msg| {
                panic!("{cfg} [{section}] {opt}: {msg}\n---\n{val}")
            });
        }
    }
}

#[test]
fn corpus_mainsail_pause_resume() {
    let scripts = config_scripts(
        "parity/scenarios/ke/ke-batch-move/mnt/usr/data/mainsail-config/mainsail.cfg",
    );
    let find = |section: &str| -> String {
        for (sec, opt, v) in &scripts {
            if sec == section && opt == "gcode" {
                return v.clone();
            }
        }
        panic!("section {section} not found")
    };
    let printer = PyValue::Dict({
        let m: IndexMap<PyValue, PyValue> = IndexMap::new();
        m
    });
    let _ = printer;
    // RESUME macro parses (rich mainsail constructs).
    let w = TemplateWrapper::load("gcode_macro RESUME:gcode", &find("gcode_macro RESUME")).unwrap();
    assert!(w.name().contains("RESUME"));
}

#[test]
fn corpus_sample_macros_start_print() {
    let scripts = config_scripts("config/sample-macros.cfg");
    let find = |section: &str| -> String {
        for (sec, opt, v) in &scripts {
            if sec == section && opt == "gcode" {
                return v.clone();
            }
        }
        panic!("section {section} not found")
    };
    let printer = dict_of(vec![]);
    let respond = Rc::new(RefCell::new(Vec::new()));
    let raise = Rc::new(RefCell::new(None));
    let mut ctx = make_ctx(&printer, respond, raise);
    let mut pd: IndexMap<PyValue, PyValue> = IndexMap::new();
    pd.insert(PyValue::Str("BED_TEMP".to_string()), param_value("65"));
    pd.insert(PyValue::Str("EXTRUDER_TEMP".to_string()), param_value("210"));
    ctx.insert("params".to_string(), Value::py(PyValue::Dict(pd)));
    let w = TemplateWrapper::load("t", &find("gcode_macro START_PRINT")).unwrap();
    let got = w.render(ctx).unwrap();
    assert!(got.contains("M140 S65.0"), "got: {got}");
    assert!(got.contains("M109 S210.0"), "got: {got}");
}

#[test]
fn corpus_ke_start_print() {
    let scripts = config_scripts("config/ender-3-v3-ke/gcode_macro.cfg");
    let find = |section: &str| -> String {
        for (sec, opt, v) in &scripts {
            if sec == section && opt == "gcode" {
                return v.clone();
            }
        }
        panic!("section {section} not found")
    };
    // filament detected -> no raise
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    let mut fs: IndexMap<PyValue, PyValue> = IndexMap::new();
    fs.insert(
        PyValue::Str("filament_detected".to_string()),
        PyValue::Bool(true),
    );
    d.insert(
        PyValue::Str("filament_switch_sensor filament_sensor".to_string()),
        PyValue::Dict(fs),
    );
    let printer = PyValue::Dict(d);
    let respond = Rc::new(RefCell::new(Vec::new()));
    let raise = Rc::new(RefCell::new(None::<String>));
    let mut ctx = make_ctx(&printer, respond, raise);
    let mut pd: IndexMap<PyValue, PyValue> = IndexMap::new();
    pd.insert(PyValue::Str("BED_TEMP".to_string()), param_value("60"));
    pd.insert(PyValue::Str("EXTRUDER_TEMP".to_string()), param_value("200"));
    ctx.insert("params".to_string(), Value::py(PyValue::Dict(pd)));
    let w = TemplateWrapper::load("t", &find("gcode_macro START_PRINT")).unwrap();
    let got = w.render(ctx).unwrap();
    assert!(got.contains("M140 S60.0"), "got: {got}");
    assert!(got.contains("M109 S200.0"), "got: {got}");
}

#[test]
fn corpus_m486_range_loop() {
    let scripts = config_scripts("test/klippy/exclude_object.cfg");
    let mut raw = None;
    for (sec, opt, v) in &scripts {
        if sec == "gcode_macro M486" && opt == "gcode" {
            raw = Some(v.clone());
        }
    }
    let raw = raw.unwrap();
    let w = TemplateWrapper::load("t", &raw).unwrap();
    let printer = dict_of(vec![
        (
            "exclude_object",
            PyValue::Dict({
                let mut m: IndexMap<PyValue, PyValue> = IndexMap::new();
                m.insert(
                    PyValue::Str("current_object".to_string()),
                    PyValue::None,
                );
                m
            }),
        ),
    ]);
    let respond = Rc::new(RefCell::new(Vec::new()));
    let raise = Rc::new(RefCell::new(None));
    let mut ctx = make_ctx(&printer, respond, raise);
    let mut pd: IndexMap<PyValue, PyValue> = IndexMap::new();
    pd.insert(PyValue::Str("T".to_string()), param_value("2"));
    ctx.insert("params".to_string(), Value::py(PyValue::Dict(pd)));
    let got = w.render(ctx).unwrap();
    assert!(got.contains("EXCLUDE_OBJECT RESET=1"), "got: {got}");
    assert!(got.contains("NAME=0"), "got: {got}");
    assert!(got.contains("NAME=1"), "got: {got}");
}

#[test]
fn corpus_m117_replace_escape() {
    let scripts = config_scripts("config/sample-macros.cfg");
    let find = |section: &str| -> String {
        for (sec, opt, v) in &scripts {
            if sec == section && opt == "gcode" {
                return v.clone();
            }
        }
        panic!("section {section} not found")
    };
    let printer = dict_of(vec![]);
    let respond = Rc::new(RefCell::new(Vec::new()));
    let raise = Rc::new(RefCell::new(None));
    let mut ctx = make_ctx(&printer, respond, raise);
    ctx.insert(
        "rawparams".to_string(),
        Value::py(PyValue::Str("Hello \"world\"; comment".to_string())),
    );
    let w = TemplateWrapper::load("t", &find("gcode_macro M117")).unwrap();
    let got = w.render(ctx).unwrap();
    assert!(got.contains("Hello"), "got: {got}");
}

// ---------------------------------------------------------------------------
// Jinja2 semantics cases (values transcribed from Jinja2 2.11.3).
// ---------------------------------------------------------------------------

fn render_str(script: &str, vars: IndexMap<String, Value>) -> String {
    Template::parse(script)
        .unwrap_or_else(|e| panic!("parse {script:?}: {e}"))
        .render(vars)
        .unwrap_or_else(|e| panic!("render {script:?}: {e}"))
}

fn ctx1(pairs: Vec<(&str, Value)>) -> IndexMap<String, Value> {
    let mut m: IndexMap<String, Value> = IndexMap::new();
    for (k, v) in pairs {
        m.insert(k.to_string(), v);
    }
    m
}

fn py_str(s: &str) -> Value {
    Value::py(PyValue::Str(s.to_string()))
}
fn py_int(i: i64) -> Value {
    Value::py(PyValue::Int(PyInt::from_i64(i)))
}
#[allow(dead_code)]
fn py_float(f: f64) -> Value {
    Value::py(PyValue::Float(f))
}
fn py_bool(b: bool) -> Value {
    Value::py(PyValue::Bool(b))
}

#[test]
fn jinja_default_float_int() {
    // missing params -> default
    assert_eq!(render_str("{ x|default(60)|float }", ctx1(vec![])), "60.0");
    assert_eq!(render_str("{ x|default(60) }", ctx1(vec![])), "60");
    assert_eq!(
        render_str("{ '42.9'|int }", ctx1(vec![])),
        "42"
    );
    assert_eq!(render_str("{ 'abc'|float }", ctx1(vec![])), "0.0");
    assert_eq!(render_str("{ 'abc'|int }", ctx1(vec![])), "0");
    // default with boolean=True on falsy
    assert_eq!(
        render_str("{ ''|default('empty', true) }", ctx1(vec![])),
        "empty"
    );
    assert_eq!(render_str("{ ''|default('empty') }", ctx1(vec![])), "");
    // default with dict and attribute (mainsail `|default({})`)
    assert_eq!(
        render_str("{ y|default({}) }", ctx1(vec![])),
        "{}"
    );
}

#[test]
fn jinja_filter_precedence_and_numeric() {
    // `1.0 / S | float` -> 1.0 / (S|float)
    assert_eq!(
        render_str("{ 1.0 / S | float }", ctx1(vec![("S", py_str("2"))])),
        "0.5"
    );
    // `range(T|int)` + loop
    assert_eq!(
        render_str(
            "{% for i in range(2) %}{i}{% endfor %}",
            ctx1(vec![]),
        ),
        "01"
    );
    // min/max
    assert_eq!(
        render_str("{ [20.0, 5.0, 10.0]|min }", ctx1(vec![])),
        "5.0"
    );
    assert_eq!(
        render_str("{ [20.0, 5.0, 10.0]|max }", ctx1(vec![])),
        "20.0"
    );
    // abs/lower/upper/length
    assert_eq!(render_str("{ -5|abs }", ctx1(vec![])), "5");
    assert_eq!(
        render_str("{ 'ABC'|lower }", ctx1(vec![])),
        "abc"
    );
    assert_eq!(
        render_str("{ 'abc'|upper }", ctx1(vec![])),
        "ABC"
    );
    assert_eq!(render_str("{ 'hello'|length }", ctx1(vec![])), "5");
    // replace filter
    assert_eq!(
        render_str("{ 'aab'|replace('a', 'x') }", ctx1(vec![])),
        "xxb"
    );
}

#[test]
fn jinja_conditionals_and_tests() {
    assert_eq!(
        render_str(
            "{% if 'T' in params %}T{% else %}N{% endif %}",
            ctx1(vec![(
                "params",
                py_dict(vec![("T", "1")]),
            )]),
        ),
        "T"
    );
    assert_eq!(
        render_str(
            "{% if x is defined %}D{% else %}U{% endif %}",
            ctx1(vec![]),
        ),
        "U"
    );
    assert_eq!(
        render_str(
            "{% if x is not defined %}D{% else %}U{% endif %}",
            ctx1(vec![]),
        ),
        "D"
    );
    assert_eq!(
        render_str(
            "{% if a == 1 and b != 2 %}Y{% else %}N{% endif %}",
            ctx1(vec![("a", py_int(1)), ("b", py_int(3))]),
        ),
        "Y"
    );
    // chained compare
    assert_eq!(
        render_str("{% if 1 < 2 < 3 %}Y{% endif %}", ctx1(vec![])),
        "Y"
    );
    // `not in`
    assert_eq!(
        render_str(
            "{% if 'x' not in params %}Y{% endif %}",
            ctx1(vec![("params", py_dict(vec![]))]),
        ),
        "Y"
    );
    // `is none`
    assert_eq!(
        render_str("{% if x is none %}N{% endif %}", ctx1(vec![("x", Value::py(PyValue::None))])),
        "N"
    );
    // elif/else chain
    assert_eq!(
        render_str(
            "{% if x == 0 %}zero{% elif x == 1 %}one{% else %}many{% endif %}",
            ctx1(vec![("x", py_int(1))]),
        ),
        "one"
    );
}

fn py_dict(pairs: Vec<(&str, &str)>) -> Value {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        d.insert(PyValue::Str(k.to_string()), PyValue::Str(v.to_string()));
    }
    Value::py(PyValue::Dict(d))
}

#[test]
fn jinja_ternary_concat_percent() {
    assert_eq!(
        render_str("{ 'a' if x else 'b' }", ctx1(vec![("x", py_bool(true))])),
        "a"
    );
    assert_eq!(
        render_str("{ 'a' if x else 'b' }", ctx1(vec![("x", py_bool(false))])),
        "b"
    );
    // chained else-if ternary
    assert_eq!(
        render_str(
            "{ 1 if x == 1 else 2 if x == 2 else 3 }",
            ctx1(vec![("x", py_int(2))]),
        ),
        "2"
    );
    // `~` concat
    assert_eq!(
        render_str("{ 'X=' ~ 5 }", ctx1(vec![])),
        "X=5"
    );
    // `%` formatting with tuple
    assert_eq!(
        render_str("{ '%3.0f' % 1.5 }", ctx1(vec![])),
        "  2"
    );
    assert_eq!(
        render_str("{ '%02d:%02d' % (61, 1) }", ctx1(vec![])),
        "61:01"
    );
    // adjacent literal concatenation before `%`
    assert_eq!(
        render_str(
            "{ \"A \" \"B\" % () }",
            ctx1(vec![]),
        )
        .trim(),
        "A B",
    );
}

#[test]
fn jinja_undefined_semantics() {
    // printing Undefined is empty
    assert_eq!(render_str("a{ missing }b", ctx1(vec![])), "ab");
    // iterating Undefined is empty
    assert_eq!(
        render_str(
            "{% for x in missing %}X{% else %}E{% endfor %}",
            ctx1(vec![]),
        ),
        "E"
    );
    // boolean context is False
    assert_eq!(
        render_str(
            "{% if missing %}Y{% else %}N{% endif %}",
            ctx1(vec![]),
        ),
        "N"
    );
    // arithmetic on Undefined raises UndefinedError with Jinja text
    let err = Template::parse("{ missing + 1 }")
        .unwrap()
        .render(ctx1(vec![]))
        .unwrap_err();
    assert_eq!(err.kind(), "UndefinedError");
    assert_eq!(err.message(), "'missing' is undefined");
    // attribute access on Undefined raises too
    let err = Template::parse("{ missing.attr }")
        .unwrap()
        .render(ctx1(vec![]))
        .unwrap_err();
    assert_eq!(err.kind(), "UndefinedError");
}

#[test]
fn jinja_whitespace_control_and_comments() {
    assert_eq!(
        render_str("a  {%- if true %}b{% endif -%}  c", ctx1(vec![])),
        "abc"
    );
    assert_eq!(
        render_str("a{# comment #}b", ctx1(vec![])),
        "ab"
    );
    // trailing newline stripping (keep_trailing_newline=False)
    assert_eq!(render_str("hello\n", ctx1(vec![])), "hello");
    assert_eq!(render_str("hello\n\n", ctx1(vec![])), "hello\n");
}

#[test]
fn jinja_strings_and_literals() {
    // \x escape
    assert_eq!(
        render_str("{ '\\x23' }", ctx1(vec![])),
        "#"
    );
    // nested braces: `{{'a': 1}}` inside single-brace var
    let out = render_str("VALUE={{'a': a}}", ctx1(vec![("a", py_int(1))]));
    assert_eq!(out, "VALUE={'a': 1}");
    // list indexing incl. negative? (out of range not tested); dict subscript
    assert_eq!(
        render_str("{ d['k'] }", ctx1(vec![("d", dict_str_int(vec![("k", 7)]))])),
        "7"
    );
}

fn dict_str_int(pairs: Vec<(&str, i64)>) -> Value {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        d.insert(PyValue::Str(k.to_string()), PyValue::Int(PyInt::from_i64(v)));
    }
    Value::py(PyValue::Dict(d))
}

#[test]
fn jinja_for_loop_and_loop_var() {
    let out = render_str(
        "{% for i in [10, 20] %}{loop.index}:{loop.index0}:{loop.length}:{loop.first}:{loop.last}:{loop.revindex}:{i};{% endfor %}",
        ctx1(vec![]),
    );
    assert_eq!(out, "1:0:2:True:False:2:10;2:1:2:False:True:1:20;");
    // `for` iterates dict keys in insertion order
    let out = render_str(
        "{% for k in d %}{k},{% endfor %}",
        ctx1(vec![("d", dict_str_int(vec![("b", 1), ("a", 2)]))]),
    );
    assert_eq!(out, "b,a,");
}

#[test]
fn jinja_string_methods() {
    assert_eq!(
        render_str("{ 'a;b;c'.split(';')[1] }", ctx1(vec![])),
        "b"
    );
    assert_eq!(
        render_str(
            "{ rawparams.split(';', 1)[0].split('\\x23', 1)[0]|replace('\"', '\\\\\"') }",
            ctx1(vec![("rawparams", py_str("hi \"x\";c#d"))]),
        ),
        "hi \\\"x\\\""
    );
    assert_eq!(
        render_str("{ '{:>4.0%}'.format(0) }", ctx1(vec![])),
        "  0%"
    );
    assert_eq!(
        render_str("{ ('  hi  '.strip()) }", ctx1(vec![])),
        "hi"
    );
    // list.append mutation
    let out = render_str(
        "{% set l = [] %}{% set _d = l.append(1) %}{% set _e = l.append(2) %}{l|length}",
        ctx1(vec![]),
    );
    assert_eq!(out, "2");
}

/// Constructs Jinja accepts but this port does not render fail to load,
/// naming the construct. Jinja's own load errors (unknown tags, filters,
/// tests, syntax) are compared with the oracle in `tests/template_errors.rs`.
#[test]
fn jinja_errors_name_constructs() {
    // unsupported macro tag is named
    let e = Template::parse("{% macro x() %}{% endmacro %}").unwrap_err();
    assert_eq!(e.kind(), "TemplateSyntaxError");
    assert!(e.message().contains("macro"), "{}", e.message());
    // slicing is named as unsupported
    let e = Template::parse("{ a[1:2] }").unwrap_err();
    assert!(e.message().contains("slicing"), "{}", e.message());
}

// ---------------------------------------------------------------------------
// GCodeMacroVars + literal_eval parity.
// ---------------------------------------------------------------------------

#[test]
fn literal_eval_parity() {
    // valid literals round-trip through json dumps
    for raw in [
        "12.0",
        "17",
        "{'enable': False, 'call': \"PAUSE\"}",
        "{ 'restore': False, 'temp': 0 }",
        "'hello'",
        "\"world\"",
        "True",
        "False",
        "None",
        "[1, 2, 'a']",
        "(1, 2)",
        "{}",
        "[]",
    ] {
        let v = extras_macros_templates::gcode_macro::parse_literal(raw).unwrap_or_else(|e| {
            panic!("parse_literal {raw:?}: {e}")
        });
        extras_macros_templates::gcode_macro::validate_literal_json(&v).unwrap_or_else(|e| {
            panic!("validate_literal_json {raw:?}: {e}")
        });
    }
    let mut vars = GCodeMacroVars::new(IndexMap::new());
    let msg = vars
        .set_variable("missing", "1", "SET_GCODE_VARIABLE MACRO=X VARIABLE=missing VALUE=1")
        .unwrap_err();
    assert_eq!(msg, "Unknown gcode_macro variable 'missing'");
    // set_variable parse error text
    let mut vars2 = GCodeMacroVars::new({
        let mut m: IndexMap<String, PyValue> = IndexMap::new();
        m.insert("t".to_string(), PyValue::Int(PyInt::from_i64(1)));
        m
    });
    let msg = vars2
        .set_variable("t", "not a literal", "SET_GCODE_VARIABLE MACRO=X VARIABLE=t VALUE=not a literal")
        .unwrap_err();
    assert!(
        msg.starts_with("Unable to parse 'not a literal' as a literal: "),
        "{msg}"
    );
    assert!(msg.contains(" in 'SET_GCODE_VARIABLE"), "{msg}");
}
