//! Parity tests for `klippy/extras/gcode_macro.py`.
//!
//! The test replays every recording in
//! `tests/fixtures/recordings/gcode_macro_template/*.jsonl` and renders
//! the macro corpus with contexts derived from the recordings'
//! `GetStatusWrapper.__getitem__` returns (converted to `PyValue` via
//! `recording::Value` -> `pycompat::PyValue`), plus Jinja2 2.11.3
//! semantics cases under `jinja_*` and corpus renders under `corpus_*`,
//! each compared with the live reference through `pyoracle`.
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
    let text = std::fs::read_to_string(&full).unwrap_or_else(|e| panic!("{}: {e}", full.display()));
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
    fn get_status(&self, name: &str, _eventtime: &mut Option<f64>) -> Result<Option<PyValue>, extras_macros_templates::gcode_macro::TemplateError> {
        Ok(self.0.get(name).cloned())
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
    assert!(w.get_item("abc").unwrap().is_none());
    assert!(w.get_item("toolhead").unwrap().is_some());
    // stripping
    assert!(w.get_item("  toolhead  ").unwrap().is_some());
    assert!(w.contains("toolhead").unwrap());
    assert!(!w.contains("abc").unwrap());
    // iteration yields status object names
    assert!(w.iter_names().unwrap().contains(&"toolhead".to_string()));
    // macros seq 16/20 etc. values round-trip through `to_py`.
    let v = w.get_item("gcode_move").unwrap().unwrap();
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

/// The reference side: Klipper's Jinja environment (`gcode_macro.py:83`), a printer whose
/// objects report fixed statuses through the real `GetStatusWrapper`, and the action stubs
/// of `make_ctx`.
const PY_SETUP: &str = r#"
import sys, ast, json, contextlib
sys.path.insert(0, '__KLIPPY__')
import jinja2
import gcode
import extras.gcode_macro as gcode_macro

class _Reactor:
    def monotonic(self):
        return 1.0
    def assert_no_pause(self):
        return contextlib.nullcontext()

class _Obj:
    def __init__(self, status):
        self.status = status
    def get_status(self, eventtime):
        return self.status

class _Printer:
    def __init__(self, statuses):
        self.objects = {name: _Obj(status) for name, status in statuses.items()}
    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)
    def lookup_objects(self, module=None):
        return list(self.objects.items())
    def get_reactor(self):
        return _Reactor()

_env = jinja2.Environment('{%', '%}', '{', '}')

def _outcome(fn):
    try:
        return fn()
    except Exception as e:
        return '%s: %s' % (type(e).__name__, e)

def render_case(src, ctx):
    return _outcome(lambda: _env.from_string(src).render(ctx))

def _raise_error(msg):
    raise gcode.CommandError(msg)

# The printer, `gcode` and `gcode_macro` objects a `TemplateWrapper` looks up.
class _FakeGCode:
    error = gcode.CommandError
    config_error = Exception
    create_template_context = None
    def lookup_object(self, name):
        return self

def wrapper_name(name, src):
    return gcode_macro.TemplateWrapper(_FakeGCode(), _env, name, src).name

def parse_wrapper(name, src):
    try:
        gcode_macro.TemplateWrapper(_FakeGCode(), _env, name, src)
    except Exception as e:
        return str(e)
    return 'None'

def render_macro(src, statuses, extra):
    ctx = {'printer': gcode_macro.GetStatusWrapper(_Printer(statuses)),
           'action_respond_info': lambda msg: "",
           'action_raise_error': _raise_error,
           'action_emergency_stop': lambda msg="action_emergency_stop": "",
           'action_call_remote_method': lambda method, **kw: ""}
    ctx.update(extra)
    try:
        return ('ok', gcode_macro.TemplateWrapper(_FakeGCode(), _env, 't', src).render(ctx))
    except gcode.CommandError as e:
        return ('err', str(e))

def checked_literal(raw):
    literal = ast.literal_eval(raw)
    json.dumps(literal, separators=(',', ':'))
    return literal

class _GCmd:
    error = gcode.CommandError
    def __init__(self, params, line):
        self.params = params
        self.line = line
    def get(self, name):
        return self.params[name]
    def get_commandline(self):
        return self.line
def set_variable_error(variables, variable, value, line):
    m = object.__new__(gcode_macro.GCodeMacro)
    m.variables = dict(variables)
    try:
        m.cmd_SET_GCODE_VARIABLE(_GCmd({'VARIABLE': variable, 'VALUE': value}, line))
    except Exception as e:
        return str(e)
"#;

fn klippy_path() -> String {
    repo_root().join("klippy").to_string_lossy().replace('\\', "/")
}

fn oracle() -> pyoracle::Oracle {
    let mut py = pyoracle::Oracle::start().expect("oracle");
    if let Err(e) = py.exec(&PY_SETUP.replace("__KLIPPY__", &klippy_path())) {
        panic!("oracle setup: {}: {}", e.class_name, e.message);
    }
    py
}

/// A Python string literal for `s`.
fn py_literal(s: &str) -> String {
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

fn oracle_str(py: &mut pyoracle::Oracle, expr: &str) -> String {
    match py.eval(expr) {
        Ok(RValue::Str(s)) => s,
        other => panic!("oracle `{expr}`: {other:?}"),
    }
}

/// `TemplateWrapper::load` outcome in the reference's `parse_wrapper` spelling.
fn rust_parse(name: &str, src: &str) -> String {
    match TemplateWrapper::load(name, src) {
        Ok(_) => "None".to_string(),
        Err(msg) => msg.to_string(),
    }
}

/// Renders the `gcode` option of `section` from `config` on both sides with the printer
/// objects of `statuses` and the extra context entries (`params`, `rawparams`).
fn check_macro(config: &str, section: &str, statuses: &PyValue, extra: Vec<(&str, PyValue)>) {
    let script = config_scripts(config)
        .into_iter()
        .find(|(sec, opt, _)| sec == section && opt == "gcode")
        .map(|(_, _, v)| v)
        .unwrap_or_else(|| panic!("{config}: section {section} not found"));
    let mut ctx = make_ctx(statuses, Rc::new(RefCell::new(Vec::new())), Rc::new(RefCell::new(None)));
    for (k, v) in &extra {
        ctx.insert(k.to_string(), Value::py(v.clone()));
    }
    let w = TemplateWrapper::load("t", &script).unwrap_or_else(|e| panic!("{section}: {e}"));
    let got = match w.render(ctx) {
        Ok(out) => RValue::Tuple(vec![RValue::str("ok"), RValue::str(&out)]),
        Err(msg) => RValue::Tuple(vec![RValue::str("err"), RValue::str(&msg)]),
    };
    let extra_lit: Vec<String> = extra.iter().map(|(k, v)| format!("{}: {}", py_literal(k), v.repr())).collect();
    let mut py = oracle();
    let expr = format!("render_macro({}, {}, {{{}}})", py_literal(&script), statuses.repr(), extra_lit.join(", "));
    let expected = py.eval(&expr).unwrap_or_else(|e| panic!("oracle `{expr}`: {e:?}"));
    assert_eq!(got, expected, "{config} [{section}]");
}

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
    let mut py = oracle();
    let mut checked = 0;
    for cfg in configs {
        for (section, opt, val) in config_scripts(cfg) {
            let name = format!("{section}:{opt}");
            let expected = oracle_str(&mut py, &format!("parse_wrapper({}, {})", py_literal(&name), py_literal(&val)));
            assert_eq!(rust_parse(&name, &val), expected, "{cfg} [{section}] {opt}\n---\n{val}");
            checked += 1;
        }
    }
    assert!(checked > 0, "no templates found in the corpus");
}

#[test]
fn corpus_mainsail_pause_resume() {
    let scripts = config_scripts(
        "parity/scenarios/ke/ke-batch-move/mnt/usr/data/mainsail-config/mainsail.cfg",
    );
    let script = scripts
        .iter()
        .find(|(sec, opt, _)| sec == "gcode_macro RESUME" && opt == "gcode")
        .map(|(_, _, v)| v.clone())
        .expect("section gcode_macro RESUME");
    // RESUME macro parses (rich mainsail constructs) and keeps its name.
    let name = "gcode_macro RESUME:gcode";
    let mut py = oracle();
    assert_eq!(
        rust_parse(name, &script),
        oracle_str(&mut py, &format!("parse_wrapper({}, {})", py_literal(name), py_literal(&script)))
    );
    let w = TemplateWrapper::load(name, &script).unwrap();
    assert_eq!(
        w.name(),
        oracle_str(&mut py, &format!("wrapper_name({}, {})", py_literal(name), py_literal(&script)))
    );
}

#[test]
fn corpus_sample_macros_start_print() {
    check_macro(
        "config/sample-macros.cfg",
        "gcode_macro START_PRINT",
        &dict_of(vec![]),
        vec![("params", dict_of(vec![("BED_TEMP", param_value("65")), ("EXTRUDER_TEMP", param_value("210"))]))],
    );
}

#[test]
fn corpus_ke_start_print() {
    // filament detected -> no raise
    let statuses = dict_of(vec![(
        "filament_switch_sensor filament_sensor",
        dict_of(vec![("filament_detected", PyValue::Bool(true))]),
    )]);
    check_macro(
        "config/ender-3-v3-ke/gcode_macro.cfg",
        "gcode_macro START_PRINT",
        &statuses,
        vec![("params", dict_of(vec![("BED_TEMP", param_value("60")), ("EXTRUDER_TEMP", param_value("200"))]))],
    );
}

#[test]
fn corpus_m486_range_loop() {
    let statuses = dict_of(vec![("exclude_object", dict_of(vec![("current_object", PyValue::None)]))]);
    check_macro(
        "test/klippy/exclude_object.cfg",
        "gcode_macro M486",
        &statuses,
        vec![("params", dict_of(vec![("T", param_value("2"))]))],
    );
}

#[test]
fn corpus_m117_replace_escape() {
    check_macro(
        "config/sample-macros.cfg",
        "gcode_macro M117",
        &dict_of(vec![]),
        vec![("rawparams", PyValue::Str("Hello \"world\"; comment".to_string()))],
    );
}

// ---------------------------------------------------------------------------
// Jinja2 semantics cases, each rendered by the reference Jinja2 2.11.3.
// ---------------------------------------------------------------------------

fn render_str(script: &str, vars: IndexMap<String, Value>) -> String {
    match Template::parse(script) {
        Err(e) => format!("{}: {}", e.kind(), e.message()),
        Ok(t) => match t.render(vars) {
            Ok(out) => out,
            Err(e) => format!("{}: {}", e.kind(), e.message()),
        },
    }
}

/// Render context for both sides: `(name, value)` pairs.
type Ctx<'a> = Vec<(&'a str, PyValue)>;

fn ctx1(pairs: &Ctx) -> IndexMap<String, Value> {
    pairs.iter().map(|(k, v)| (k.to_string(), Value::py(v.clone()))).collect()
}

/// Python dict literal of a context.
fn py_ctx(pairs: &Ctx) -> String {
    let items: Vec<String> = pairs.iter().map(|(k, v)| format!("{}: {}", py_literal(k), v.repr())).collect();
    format!("{{{}}}", items.join(", "))
}

fn py_str(s: &str) -> PyValue {
    PyValue::Str(s.to_string())
}
fn py_int(i: i64) -> PyValue {
    PyValue::Int(PyInt::from_i64(i))
}

/// `src` rendered by the port and by the reference against the same context; the outputs
/// (or `Class: message` of the raised exception) must be equal.
fn check_jinja(py: &mut pyoracle::Oracle, src: &str, ctx: Ctx) {
    let expected = match py.eval(&format!("render_case({}, {})", py_literal(src), py_ctx(&ctx))) {
        Ok(RValue::Str(s)) => s,
        other => panic!("oracle at {src:?}: {other:?}"),
    };
    assert_eq!(render_str(src, ctx1(&ctx)), expected, "{src:?}");
}

#[test]
fn jinja_default_float_int() {
    let mut py = oracle();
    // missing params -> default
    check_jinja(&mut py, "{ x|default(60)|float }", vec![]);
    check_jinja(&mut py, "{ x|default(60) }", vec![]);
    check_jinja(&mut py, "{ '42.9'|int }", vec![]);
    check_jinja(&mut py, "{ 'abc'|float }", vec![]);
    check_jinja(&mut py, "{ 'abc'|int }", vec![]);
    // default with boolean=True on falsy
    check_jinja(&mut py, "{ ''|default('empty', true) }", vec![]);
    check_jinja(&mut py, "{ ''|default('empty') }", vec![]);
    // default with dict and attribute (mainsail `|default({})`)
    check_jinja(&mut py, "{ y|default({}) }", vec![]);
}

#[test]
fn jinja_filter_precedence_and_numeric() {
    let mut py = oracle();
    // `1.0 / S | float` -> 1.0 / (S|float)
    check_jinja(&mut py, "{ 1.0 / S | float }", vec![("S", py_str("2"))]);
    // `range(T|int)` + loop
    check_jinja(&mut py, "{% for i in range(2) %}{i}{% endfor %}", vec![]);
    // min/max
    check_jinja(&mut py, "{ [20.0, 5.0, 10.0]|min }", vec![]);
    check_jinja(&mut py, "{ [20.0, 5.0, 10.0]|max }", vec![]);
    // abs/lower/upper/length
    check_jinja(&mut py, "{ -5|abs }", vec![]);
    check_jinja(&mut py, "{ 'ABC'|lower }", vec![]);
    check_jinja(&mut py, "{ 'abc'|upper }", vec![]);
    check_jinja(&mut py, "{ 'hello'|length }", vec![]);
    // replace filter
    check_jinja(&mut py, "{ 'aab'|replace('a', 'x') }", vec![]);
}

#[test]
fn jinja_conditionals_and_tests() {
    let mut py = oracle();
    check_jinja(&mut py, "{% if 'T' in params %}T{% else %}N{% endif %}", vec![("params", py_dict(vec![("T", "1")]))]);
    check_jinja(&mut py, "{% if x is defined %}D{% else %}U{% endif %}", vec![]);
    check_jinja(&mut py, "{% if x is not defined %}D{% else %}U{% endif %}", vec![]);
    check_jinja(
        &mut py,
        "{% if a == 1 and b != 2 %}Y{% else %}N{% endif %}",
        vec![("a", py_int(1)), ("b", py_int(3))],
    );
    // chained compare
    check_jinja(&mut py, "{% if 1 < 2 < 3 %}Y{% endif %}", vec![]);
    // `not in`
    check_jinja(&mut py, "{% if 'x' not in params %}Y{% endif %}", vec![("params", py_dict(vec![]))]);
    // `is none`
    check_jinja(&mut py, "{% if x is none %}N{% endif %}", vec![("x", PyValue::None)]);
    // elif/else chain
    check_jinja(
        &mut py,
        "{% if x == 0 %}zero{% elif x == 1 %}one{% else %}many{% endif %}",
        vec![("x", py_int(1))],
    );
}

fn py_dict(pairs: Vec<(&str, &str)>) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        d.insert(PyValue::Str(k.to_string()), PyValue::Str(v.to_string()));
    }
    PyValue::Dict(d)
}

#[test]
fn jinja_ternary_concat_percent() {
    let mut py = oracle();
    check_jinja(&mut py, "{ 'a' if x else 'b' }", vec![("x", PyValue::Bool(true))]);
    check_jinja(&mut py, "{ 'a' if x else 'b' }", vec![("x", PyValue::Bool(false))]);
    // chained else-if ternary
    check_jinja(&mut py, "{ 1 if x == 1 else 2 if x == 2 else 3 }", vec![("x", py_int(2))]);
    // `~` concat
    check_jinja(&mut py, "{ 'X=' ~ 5 }", vec![]);
    // `%` formatting with tuple
    check_jinja(&mut py, "{ '%3.0f' % 1.5 }", vec![]);
    check_jinja(&mut py, "{ '%02d:%02d' % (61, 1) }", vec![]);
    // adjacent literal concatenation before `%`
    check_jinja(&mut py, "{ \"A \" \"B\" % () }", vec![]);
}

#[test]
fn jinja_undefined_semantics() {
    let mut py = oracle();
    // printing Undefined is empty
    check_jinja(&mut py, "a{ missing }b", vec![]);
    // iterating Undefined is empty
    check_jinja(&mut py, "{% for x in missing %}X{% else %}E{% endfor %}", vec![]);
    // boolean context is False
    check_jinja(&mut py, "{% if missing %}Y{% else %}N{% endif %}", vec![]);
    // arithmetic on Undefined raises UndefinedError
    check_jinja(&mut py, "{ missing + 1 }", vec![]);
    // attribute access on Undefined raises too
    check_jinja(&mut py, "{ missing.attr }", vec![]);
}

#[test]
fn jinja_whitespace_control_and_comments() {
    let mut py = oracle();
    check_jinja(&mut py, "a  {%- if true %}b{% endif -%}  c", vec![]);
    check_jinja(&mut py, "a{# comment #}b", vec![]);
    // trailing newline stripping (keep_trailing_newline=False)
    check_jinja(&mut py, "hello\n", vec![]);
    check_jinja(&mut py, "hello\n\n", vec![]);
}

#[test]
fn jinja_strings_and_literals() {
    let mut py = oracle();
    // \x escape
    check_jinja(&mut py, "{ '\\x23' }", vec![]);
    // nested braces: `{{'a': 1}}` inside single-brace var
    check_jinja(&mut py, "VALUE={{'a': a}}", vec![("a", py_int(1))]);
    // dict subscript
    check_jinja(&mut py, "{ d['k'] }", vec![("d", dict_str_int(vec![("k", 7)]))]);
}

fn dict_str_int(pairs: Vec<(&str, i64)>) -> PyValue {
    let mut d: IndexMap<PyValue, PyValue> = IndexMap::new();
    for (k, v) in pairs {
        d.insert(PyValue::Str(k.to_string()), PyValue::Int(PyInt::from_i64(v)));
    }
    PyValue::Dict(d)
}

#[test]
fn jinja_for_loop_and_loop_var() {
    let mut py = oracle();
    check_jinja(
        &mut py,
        "{% for i in [10, 20] %}{loop.index}:{loop.index0}:{loop.length}:{loop.first}:{loop.last}:{loop.revindex}:{i};{% endfor %}",
        vec![],
    );
    // `for` iterates dict keys in insertion order
    check_jinja(&mut py, "{% for k in d %}{k},{% endfor %}", vec![("d", dict_str_int(vec![("b", 1), ("a", 2)]))]);
}

#[test]
fn jinja_string_methods() {
    let mut py = oracle();
    check_jinja(&mut py, "{ 'a;b;c'.split(';')[1] }", vec![]);
    check_jinja(
        &mut py,
        "{ rawparams.split(';', 1)[0].split('\\x23', 1)[0]|replace('\"', '\\\\\"') }",
        vec![("rawparams", py_str("hi \"x\";c#d"))],
    );
    check_jinja(&mut py, "{ '{:>4.0%}'.format(0) }", vec![]);
    check_jinja(&mut py, "{ ('  hi  '.strip()) }", vec![]);
    // list.append mutation
    check_jinja(
        &mut py,
        "{% set l = [] %}{% set _d = l.append(1) %}{% set _e = l.append(2) %}{l|length}",
        vec![],
    );
}

/// Constructs Jinja accepts but this port does not render fail to load,
/// naming the construct (a port limitation: the reference renders them).
/// Jinja's own load errors (unknown tags, filters, tests, syntax) are
/// compared with the oracle in `tests/template_errors.rs`.
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

fn to_rvalue(v: &PyValue) -> RValue {
    match v {
        PyValue::None => RValue::Null,
        PyValue::Bool(b) => RValue::Bool(*b),
        PyValue::Int(n) => RValue::Int(n.0.to_string().parse().expect("int fits i64")),
        PyValue::Float(f) => RValue::float(*f),
        PyValue::Str(s) => RValue::str(s),
        PyValue::Bytes(b) => RValue::bytes(b),
        PyValue::List(items) => RValue::List(items.iter().map(to_rvalue).collect()),
        PyValue::Tuple(items) => RValue::Tuple(items.iter().map(to_rvalue).collect()),
        PyValue::Dict(pairs) => RValue::Dict(pairs.iter().map(|(k, v)| (to_rvalue(k), to_rvalue(v))).collect()),
        PyValue::Host(_) => panic!("host value cannot be compared"),
    }
}

/// `(macro variables, VARIABLE, VALUE, commandline)` of one `SET_GCODE_VARIABLE`.
type SetVariableCase<'a> = (&'a [(&'a str, PyValue)], &'a str, &'a str, &'a str);

#[test]
fn literal_eval_parity() {
    let mut py = oracle();
    // valid literals: the parsed value and the json check, against
    // `ast.literal_eval` + `json.dumps` (gcode_macro.py:156-157)
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
        let expected = py
            .eval(&format!("checked_literal({})", py_literal(raw)))
            .unwrap_or_else(|e| panic!("oracle literal {raw:?}: {e:?}"));
        assert_eq!(to_rvalue(&v), expected, "literal {raw:?}");
    }
    // SET_GCODE_VARIABLE error texts (gcode_macro.py:175-185)
    let cases: [SetVariableCase; 2] = [
        (&[], "missing", "1", "SET_GCODE_VARIABLE MACRO=X VARIABLE=missing VALUE=1"),
        (
            &[("t", PyValue::Int(PyInt::from_i64(1)))],
            "t",
            "not a literal",
            "SET_GCODE_VARIABLE MACRO=X VARIABLE=t VALUE=not a literal",
        ),
    ];
    for (variables, variable, value, line) in cases {
        let mut vars = GCodeMacroVars::new(variables.iter().map(|(k, v)| (k.to_string(), v.clone())).collect());
        let msg = vars.set_variable(variable, value, line).unwrap_err();
        let expected = py
            .eval(&format!(
                "set_variable_error({}, {}, {}, {})",
                py_ctx(&variables.to_vec()),
                py_literal(variable),
                py_literal(value),
                py_literal(line)
            ))
            .expect("oracle set_variable");
        assert_eq!(RValue::str(&msg), expected, "SET_GCODE_VARIABLE {variable}={value}");
    }
}
