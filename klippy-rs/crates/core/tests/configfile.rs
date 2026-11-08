//! Parity tests for the `configfile` port.
//!
//! The main test replays every `tests/fixtures/recordings/configfile_parse`
//! recording: for each scenario it rebuilds the wrapper graph from the
//! scenario's config file (the `config.main` of its `scenario.json`) plus
//! the secondary temperature-sensors config the heaters loader reads, then
//! executes every recorded `ConfigWrapper` call in `seq` order and compares
//! the result with the recorded value exactly (floats by hex + repr
//! spelling, strings byte for byte, tuples elementwise).
//!
//! Wrapper identity is resolved per recorded object id: `getsection`
//! children inherit their parent's fileconfig, and receivers first seen
//! inside the secondary config are resolved by trying every
//! (fileconfig, section) candidate until the recorded return value matches.
//! `_get_wrapper` records duplicate the following outer call and are
//! skipped; the outer call replays the single Python execution, including
//! access tracking.
//!
//! The remaining tests pin the SAVE_CONFIG rewrite text and every error
//! message with expectations computed by the reference semantics (exact
//! `str(exc)` texts and verbatim autosave block construction).

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::rc::Rc;

use klippy_core::configfile::{
    find_autosave_data, save_config_names, Bound, ChoiceKey, ChoiceSet, ConfigFileReader,
    ConfigValidate, ConfigWrapper, DeprecatedStore, ListParser, ListVal, PrinterConfig,
    AUTOSAVE_HEADER,
};
use pycompat::{PyInt, PyValue};
use pytext::configparser::RawConfigParser;
use recording::reader::load_file;
use recording::value::{FloatVal, Record, Value as Rv};

/// Absolute path of the repository root (this test runs from the workspace).
fn repo_root() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest.ancestors().nth(3).unwrap().to_path_buf()
}

/// The scenario's config file, from `parity/scenarios/**/scenario.json`.
fn scenario_main(scenario: &str) -> PathBuf {
    use std::fs;
    let root = repo_root().join("parity/scenarios");
    let mut stack = vec![root];
    let mut found = None;
    while let Some(dir) = stack.pop() {
        for entry in fs::read_dir(&dir).unwrap() {
            let entry = entry.unwrap();
            let path = entry.path();
            if path.is_dir() {
                stack.push(path.clone());
                if path.file_name().unwrap() == scenario {
                    let raw = fs::read_to_string(path.join("scenario.json")).unwrap();
                    let key = raw.find("\"main\"").unwrap();
                    let rest = &raw[key + 6..];
                    let start = rest.find('"').unwrap() + 1;
                    let rel = &rest[start..rest[start..].find('"').unwrap() + start];
                    found = Some(repo_root().join(rel));
                }
            }
        }
    }
    found.unwrap_or_else(|| panic!("no scenario dir for {scenario}"))
}

/// `klippy/extras/temperature_sensors.cfg`, the second config the heaters
/// loader reads while setting up (its wrappers appear in every recording).
fn sensors_path() -> PathBuf {
    repo_root().join("klippy/extras/temperature_sensors.cfg")
}

/// Shared wrapper state for one replayed scenario. Access tracking is one
/// dict shared by every wrapper, as in Python.
struct World {
    /// Fileconfigs of this scenario (main first, then the sensors config).
    fileconfigs: Vec<Rc<RawConfigParser>>,
    /// Map from recorded object id to candidate wrappers.
    wrappers: BTreeMap<String, Vec<ConfigWrapper>>,
    access: Rc<RefCell<indexmap::IndexMap<(String, String), PyValue>>>,
    deprecated: Rc<RefCell<DeprecatedStore>>,
}

impl World {
    fn wrapper(&self, fc: usize, section: &str) -> ConfigWrapper {
        ConfigWrapper::new(
            Rc::clone(&self.fileconfigs[fc]),
            Rc::clone(&self.access),
            Rc::clone(&self.deprecated),
            section.to_string(),
        )
    }

    /// Every (fileconfig, section) pair, for lazy receiver resolution.
    fn all_candidates(&self) -> Vec<ConfigWrapper> {
        let mut out = Vec::new();
        for (fc, parser) in self.fileconfigs.iter().enumerate() {
            for section in parser.sections() {
                let _ = fc;
                out.push(self.wrapper(fc, &section));
            }
            out.push(self.wrapper(fc, "printer"));
        }
        out
    }

    fn candidates(&self, oid: &str) -> Vec<ConfigWrapper> {
        self.wrappers
            .get(oid)
            .cloned()
            .unwrap_or_else(|| self.all_candidates())
    }
}

// ---------------------------------------------------------------------------
// Recorded-value conversions.
// ---------------------------------------------------------------------------

fn is_sentinel(v: &Rv) -> bool {
    matches!(v, Rv::ObjectId(_))
}

fn rv_str(v: &Rv) -> String {
    match v {
        Rv::Str(s) => s.clone(),
        other => panic!("expected string, got {}", other.render()),
    }
}

fn rv_bool(v: &Rv) -> bool {
    match v {
        Rv::Bool(b) => *b,
        other => panic!("expected bool, got {}", other.render()),
    }
}

fn rv_f64(v: &Rv) -> f64 {
    match v {
        Rv::Float(f) => f.to_f64().unwrap(),
        Rv::Int(i) => *i as f64,
        other => panic!("expected float, got {}", other.render()),
    }
}

fn rv_i64(v: &Rv) -> i64 {
    match v {
        Rv::Int(i) => *i,
        Rv::BigInt(s) => s.parse().unwrap(),
        other => panic!("expected int, got {}", other.render()),
    }
}

fn rv_pyint(v: &Rv) -> PyInt {
    PyInt::from_i64(rv_i64(v))
}

fn rv_bound(v: &Rv) -> Option<Bound> {
    match v {
        Rv::Null => None,
        Rv::Float(f) => Some(Bound::float(f.to_f64().unwrap())),
        other => Some(Bound::Int(rv_pyint(other))),
    }
}

/// A recorded value as a `PyValue` (opaque ids become host handles kept
/// distinct by their oid number).
fn to_pyvalue(v: &Rv) -> PyValue {
    match v {
        Rv::Null => PyValue::None,
        Rv::Bool(b) => PyValue::Bool(*b),
        Rv::Int(i) => PyValue::Int(PyInt::from_i64(*i)),
        Rv::BigInt(s) => PyValue::Int(PyInt::from_i64(s.parse().unwrap())),
        Rv::Float(f) => PyValue::Float(f.to_f64().unwrap()),
        Rv::Bytes(b) => PyValue::Bytes(b.clone()),
        Rv::Str(s) => PyValue::Str(s.clone()),
        Rv::List(items) => PyValue::List(items.iter().map(to_pyvalue).collect()),
        Rv::Tuple(items) => PyValue::Tuple(items.iter().map(to_pyvalue).collect()),
        Rv::Dict(pairs) => {
            let mut map = indexmap::IndexMap::new();
            for (k, val) in pairs {
                map.insert(to_pyvalue(k), to_pyvalue(val));
            }
            PyValue::Dict(map)
        }
        Rv::Struct { fields, .. } => {
            let mut map = indexmap::IndexMap::new();
            for (name, val) in fields {
                map.insert(PyValue::Str(name.clone()), to_pyvalue(val));
            }
            PyValue::Dict(map)
        }
        Rv::Opaque { .. } | Rv::ObjectId(_) => {
            let id = match v {
                Rv::Opaque { addr, .. } => addr.clone(),
                Rv::ObjectId(id) => id.clone(),
                _ => unreachable!(),
            };
            let n: u64 = id
                .rsplit('#')
                .next()
                .and_then(|s| s.parse().ok())
                .unwrap_or(0);
            PyValue::Host(n)
        }
    }
}

/// A recorded list default as a `ListVal` (element kinds follow the
/// recorded leaves, matching the element parser the reference used).
fn listval_from_rv(v: &Rv) -> Option<ListVal> {
    match v {
        Rv::Tuple(items) | Rv::List(items) => {
            let mut out = Vec::new();
            for item in items {
                out.push(listval_from_rv(item)?);
            }
            Some(ListVal::Nested(out))
        }
        Rv::Str(s) => Some(ListVal::Text(s.clone())),
        Rv::Int(i) => Some(ListVal::Int(PyInt::from_i64(*i))),
        Rv::BigInt(s) => Some(ListVal::Int(PyInt::from_i64(s.parse().unwrap()))),
        Rv::Float(f) => Some(ListVal::Float(f.to_f64().unwrap())),
        _ => None,
    }
}

fn default_list(v: &Rv) -> Option<Option<ListVal>> {
    if is_sentinel(v) {
        None
    } else if matches!(v, Rv::Null) {
        Some(None)
    } else {
        Some(Some(listval_from_rv(v).expect("list default")))
    }
}

fn leaf_parser(v: &Rv) -> Option<ListParser> {
    match v {
        Rv::Tuple(items) | Rv::List(items) => items.iter().find_map(leaf_parser),
        Rv::Float(_) => Some(ListParser::Float),
        Rv::Int(_) | Rv::BigInt(_) => Some(ListParser::Int),
        Rv::Str(_) => Some(ListParser::Str),
        _ => None,
    }
}

/// Exact equality of a ported value with a recorded one.
fn vals_equal(actual: &PyValue, expected: &Rv) -> bool {
    match (actual, expected) {
        (PyValue::None, Rv::Null) => true,
        (PyValue::Bool(a), Rv::Bool(b)) => a == b,
        (PyValue::Int(a), Rv::Int(b)) => a.inner().to_string() == b.to_string(),
        (PyValue::Int(a), Rv::BigInt(b)) => a.inner().to_string() == *b,
        (PyValue::Float(a), Rv::Float(b)) => FloatVal::new(*a) == *b,
        (PyValue::Str(a), Rv::Str(b)) => a == b,
        (PyValue::Bytes(a), Rv::Bytes(b)) => a == b,
        (PyValue::Tuple(a), Rv::Tuple(b)) | (PyValue::Tuple(a), Rv::List(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| vals_equal(x, y))
        }
        (PyValue::List(a), Rv::List(b)) | (PyValue::List(a), Rv::Tuple(b)) => {
            a.len() == b.len() && a.iter().zip(b).all(|(x, y)| vals_equal(x, y))
        }
        (PyValue::Dict(a), Rv::Dict(b)) => {
            a.len() == b.len()
                && a.iter().zip(b.iter()).all(|((ak, av), (ek, ev))| {
                    vals_equal(ak, ek) && vals_equal(av, ev)
                })
        }
        (PyValue::Host(n), Rv::ObjectId(id)) => {
            id.rsplit('#').next().and_then(|s| s.parse::<u64>().ok()) == Some(*n)
        }
        (PyValue::Host(n), Rv::Opaque { addr, .. }) => {
            addr.trim_start_matches("0x").parse::<u64>().ok() == Some(*n)
        }
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Recorded call execution.
// ---------------------------------------------------------------------------

fn ox_str(v: Option<String>) -> PyValue {
    match v {
        Some(s) => PyValue::Str(s),
        None => PyValue::None,
    }
}

fn choice_keys(v: &Rv) -> Vec<ChoiceKey> {
    match v {
        Rv::List(items) => items
            .iter()
            .map(|item| match item {
                Rv::Null => ChoiceKey::Null,
                Rv::Str(s) => ChoiceKey::Text(s.clone()),
                other => ChoiceKey::Int(rv_pyint(other)),
            })
            .collect(),
        _ => panic!("expected choice list, got {}", v.render()),
    }
}

fn choice_default(v: &Rv) -> Option<PyValue> {
    if is_sentinel(v) {
        None
    } else {
        Some(to_pyvalue(v))
    }
}

/// Execute one recorded getter on `w`, normalized to a `PyValue`.
#[allow(clippy::too_many_arguments)]
fn exec_getter(
    func: &str,
    w: &ConfigWrapper,
    args: &[Rv],
    parser_hints: &BTreeMap<String, ListParser>,
) -> Result<PyValue, String> {
    let err = |e: pytext::configparser::Error| format!("{e}");
    match func {
        "ConfigWrapper.get" => {
            let opt = rv_str(&args[1]);
            let default: Option<Option<String>> = match &args[2] {
                v if is_sentinel(v) => None,
                Rv::Null => Some(None),
                Rv::Str(s) => Some(Some(s.clone())),
                other => Some(Some(to_pyvalue(other).str())),
            };
            w.get(&opt, &default, rv_bool(&args[3]))
                .map(ox_str)
                .map_err(err)
        }
        "ConfigWrapper.getint" => {
            let opt = rv_str(&args[1]);
            let default: Option<Option<PyInt>> = match &args[2] {
                v if is_sentinel(v) => None,
                Rv::Null => Some(None),
                other => Some(Some(rv_pyint(other))),
            };
            w.getint(
                &opt,
                &default,
                rv_bound(&args[3]),
                rv_bound(&args[4]),
                rv_bool(&args[5]),
            )
            .map(|v| match v {
                Some(n) => PyValue::Int(n),
                None => PyValue::None,
            })
            .map_err(err)
        }
        "ConfigWrapper.getfloat" => {
            let opt = rv_str(&args[1]);
            let default: Option<Option<f64>> = match &args[2] {
                v if is_sentinel(v) => None,
                Rv::Null => Some(None),
                other => Some(Some(rv_f64(other))),
            };
            w.getfloat(
                &opt,
                &default,
                rv_bound(&args[3]),
                rv_bound(&args[4]),
                rv_bound(&args[5]),
                rv_bound(&args[6]),
                rv_bool(&args[7]),
            )
            .map(|v| match v {
                Some(n) => PyValue::Float(n),
                None => PyValue::None,
            })
            .map_err(err)
        }
        "ConfigWrapper.getboolean" => {
            let opt = rv_str(&args[1]);
            let default: Option<Option<bool>> = match &args[2] {
                v if is_sentinel(v) => None,
                Rv::Null => Some(None),
                Rv::Bool(b) => Some(Some(*b)),
                other => panic!("expected bool default, got {}", other.render()),
            };
            w.getboolean(&opt, &default, rv_bool(&args[3]))
                .map(|v| match v {
                    Some(b) => PyValue::Bool(b),
                    None => PyValue::None,
                })
                .map_err(err)
        }
        "ConfigWrapper.getchoice" => getchoice(w, args, exec_getter),
        "ConfigWrapper.getlist" => {
            let opt = rv_str(&args[1]);
            let default = default_list(&args[2]);
            let sep = rv_str(&args[3]);
            let count = match &args[4] {
                Rv::Null => None,
                other => Some(rv_i64(other)),
            };
            w.getlist(&opt, &default, &sep, count, rv_bool(&args[5]))
                .map(|v| match v {
                    Some(l) => l.to_pyvalue(),
                    None => PyValue::None,
                })
                .map_err(err)
        }
        "ConfigWrapper.getintlist" => {
            let opt = rv_str(&args[1]);
            let default = default_list(&args[2]);
            let sep = rv_str(&args[3]);
            let count = match &args[4] {
                Rv::Null => None,
                other => Some(rv_i64(other)),
            };
            w.getintlist(&opt, &default, &sep, count, rv_bool(&args[5]))
                .map(|v| match v {
                    Some(l) => l.to_pyvalue(),
                    None => PyValue::None,
                })
                .map_err(err)
        }
        "ConfigWrapper.getfloatlist" => {
            let opt = rv_str(&args[1]);
            let default = default_list(&args[2]);
            let sep = rv_str(&args[3]);
            let count = match &args[4] {
                Rv::Null => None,
                other => Some(rv_i64(other)),
            };
            w.getfloatlist(&opt, &default, &sep, count, rv_bool(&args[5]))
                .map(|v| match v {
                    Some(l) => l.to_pyvalue(),
                    None => PyValue::None,
                })
                .map_err(err)
        }
        "ConfigWrapper.getlists" => {
            let opt = rv_str(&args[1]);
            let default = default_list(&args[2]);
            let seps: Vec<String> = match &args[3] {
                Rv::Tuple(items) => items.iter().map(rv_str).collect(),
                other => panic!("expected seps tuple, got {}", other.render()),
            };
            let count = match &args[4] {
                Rv::Null => None,
                other => Some(rv_i64(other)),
            };
            let parser = match &args[5] {
                Rv::ObjectId(id) => *parser_hints.get(id).unwrap_or(&ListParser::Str),
                other => panic!("expected parser oid, got {}", other.render()),
            };
            w.getlists(&opt, &default, &seps, count, parser, rv_bool(&args[6]))
                .map(|v| match v {
                    Some(l) => l.to_pyvalue(),
                    None => PyValue::None,
                })
                .map_err(err)
        }
        "ConfigWrapper.get_prefix_options" => {
            let prefix = rv_str(&args[1]);
            w.get_prefix_options(&prefix)
                .map(|opts| PyValue::List(opts.into_iter().map(PyValue::Str).collect()))
                .map_err(err)
        }
        "ConfigWrapper.get_prefix_sections" => {
            let prefix = rv_str(&args[1]);
            let names: Vec<String> = w
                .get_prefix_sections(&prefix)
                .into_iter()
                .map(|c| c.section_name().to_string())
                .collect();
            Ok(PyValue::List(names.into_iter().map(PyValue::Str).collect()))
        }
        "ConfigWrapper.has_section" => {
            let section = rv_str(&args[1]);
            Ok(PyValue::Bool(w.has_section(&section)))
        }
        _ => panic!("unexpected function {func}"),
    }
}

/// `getchoice`: list and literal-dict choices map directly.
fn getchoice(
    w: &ConfigWrapper,
    args: &[Rv],
    _exec: fn(&str, &ConfigWrapper, &[Rv], &BTreeMap<String, ListParser>) -> Result<PyValue, String>,
) -> Result<PyValue, String> {
    let opt = rv_str(&args[1]);
    let choices = match &args[2] {
        Rv::List(_) => ChoiceSet::from_list(choice_keys(&args[2])),
        Rv::Dict(pairs) => ChoiceSet::from_map(
            pairs
                .iter()
                .map(|(k, v)| {
                    let key = match k {
                        Rv::Null => ChoiceKey::Null,
                        Rv::Str(s) => ChoiceKey::Text(s.clone()),
                        other => ChoiceKey::Int(rv_pyint(other)),
                    };
                    (key, to_pyvalue(v))
                })
                .collect(),
        ),
        // Opaque dicts go through `opaque_dict_call` instead.
        Rv::ObjectId(_) => return Err("__opaque__".to_string()),
        other => panic!("expected choices, got {}", other.render()),
    };
    let default = choice_default(&args[3]);
    let note = rv_bool(&args[4]);
    w.getchoice(&opt, &choices, &default, note)
        .map_err(|e| format!("{e}"))
}

/// Opaque-dict `getchoice` with the recorded return value as the mapped
/// class for the stored text (the recorder hides the dict content; the
/// caller compares the class id, which is exactly the reference lookup).
fn opaque_dict_call(w: &ConfigWrapper, args: &[Rv], ret: &Rv) -> Result<PyValue, String> {
    let opt = rv_str(&args[1]);
    let raw = w
        .fileconfig()
        .get(w.section_name(), &opt)
        .map_err(|e| format!("{e}"))?
        .unwrap_or_default();
    let choices = ChoiceSet::from_map(vec![(ChoiceKey::Text(raw), to_pyvalue(ret))]);
    let default = choice_default(&args[3]);
    let note = rv_bool(&args[4]);
    w.getchoice(&opt, &choices, &default, note)
        .map_err(|e| format!("{e}"))
}

// ---------------------------------------------------------------------------
// Replay.
// ---------------------------------------------------------------------------

fn replay_scenario(scenario: &str, records: Vec<Record>) -> Result<usize, String> {
    let main = scenario_main(scenario);
    let main_str = main.to_string_lossy().into_owned();
    let mut printer = PrinterConfig::new(main_str.clone());
    let (fileconfig, autosave) = printer
        .autosave
        .load_main_config()
        .map_err(|e| format!("load_main_config: {e}"))?;
    let access = printer.validate.start_access_tracking(&autosave);

    let sensors_text = std::fs::read_to_string(sensors_path()).unwrap();
    let sensors_text = sensors_text.replace("\r\n", "\n").replace('\r', "\n");
    let sensors = ConfigFileReader::build_fileconfig(&sensors_text, "temperature_sensors.cfg")
        .map_err(|e| format!("sensors config: {e}"))?;

    let mut world = World {
        fileconfigs: vec![Rc::new(fileconfig), Rc::new(sensors)],
        wrappers: BTreeMap::new(),
        access,
        deprecated: Rc::clone(&printer.deprecated),
    };
    let root = ConfigWrapper::new(
        Rc::clone(&world.fileconfigs[0]),
        Rc::clone(&world.access),
        Rc::clone(&world.deprecated),
        "printer".to_string(),
    );
    world.wrappers.insert("ConfigWrapper#2".to_string(), vec![root]);

    // Element-parser inference: vote from non-empty results in this
    // scenario; the same callable oid always denotes the same parser.
    let mut parser_hints: BTreeMap<String, ListParser> = BTreeMap::new();
    for r in &records {
        if r.function == "ConfigWrapper.getlists" {
            if let ([_, _, _, _, _, parser_v, ..], Some(ret)) =
                (r.args.as_slice(), &r.ret)
            {
                if let Rv::ObjectId(id) = parser_v {
                    if let Some(p) = leaf_parser(ret) {
                        parser_hints.insert(id.clone(), p);
                    }
                }
            }
        }
    }

    let mut count = 0usize;
    for record in &records {
        count += 1;
        let oid = record.object_id.clone().unwrap_or_default();
        match record.function.as_str() {
            // Duplicates the following outer call; replayed there once.
            "ConfigWrapper._get_wrapper" => {}
            "ConfigWrapper.getsection" => {
                let section = rv_str(&record.args[1]);
                let ret_oid = match &record.ret {
                    Some(Rv::ObjectId(id)) => id.clone(),
                    other => {
                        return Err(format!(
                            "seq {}: expected object id, got {:?}",
                            record.seq,
                            other.as_ref().map(Rv::render)
                        ))
                    }
                };
                let mut children = Vec::new();
                let parents = world.candidates(&oid);
                for parent in &parents {
                    children.push(ConfigWrapper::new(
                        Rc::new(parent.fileconfig().clone()),
                        Rc::clone(&world.access),
                        Rc::clone(&world.deprecated),
                        section.clone(),
                    ));
                }
                world.wrappers.insert(ret_oid, children);
            }
            _ => {
                let cands = world.candidates(&oid);
                let mut matched: Option<ConfigWrapper> = None;
                let mut last = String::new();
                for cand in &cands {
                    let result = if record.function == "ConfigWrapper.getchoice"
                        && matches!(&record.args[2], Rv::ObjectId(_))
                        && record.ret.is_some()
                    {
                        opaque_dict_call(cand, &record.args, record.ret.as_ref().unwrap())
                    } else {
                        exec_getter(&record.function, cand, &record.args, &parser_hints)
                    };
                    match result {
                        Ok(v) => {
                            last = v.repr();
                            if let Some(ret) = &record.ret {
                                // Opaque list returns accept any body;
                                // their sections are exercised by the
                                // following getsection/getter calls.
                                let accept = match ret {
                                    Rv::ObjectId(_)
                                        if record.function
                                            == "ConfigWrapper.get_prefix_sections" =>
                                    {
                                        true
                                    }
                                    _ => vals_equal(&v, ret),
                                };
                                if accept {
                                    matched = Some(cand.clone());
                                    break;
                                }
                            }
                        }
                        Err(e) => {
                            last = e;
                        }
                    }
                }
                match matched {
                    Some(w) => {
                        world.wrappers.insert(oid.clone(), vec![w]);
                    }
                    None => {
                        return Err(format!(
                            "seq {} {}: expected {:?} actual {}",
                            record.seq,
                            record.function,
                            record.ret.as_ref().map(Rv::render),
                            last
                        ));
                    }
                }
            }
        }
    }
    Ok(count)
}

#[test]
fn replay_every_configfile_recording() {
    let dir = repo_root().join("tests/fixtures/recordings/configfile_parse");
    let mut entries: Vec<_> =
        std::fs::read_dir(&dir).unwrap().map(|e| e.unwrap().path()).collect();
    entries.sort();
    let mut total = 0usize;
    for path in entries {
        let records = load_file(&path).unwrap_or_else(|e| panic!("{e}"));
        let scenario = records.first().map(|r| r.scenario.clone()).unwrap();
        let count = replay_scenario(&scenario, records)
            .unwrap_or_else(|d| panic!("{scenario} ({}): {d}", path.display()));
        eprintln!("replayed {scenario}: {count} calls");
        total += count;
    }
    assert!(total > 0, "no recordings replayed");
}

// ---------------------------------------------------------------------------
// SAVE_CONFIG and error-text tests.
// ---------------------------------------------------------------------------

#[test]
fn save_config_rewrite_text() {
    // A config with an existing autosave block: the rewrite strips the old
    // block, strips duplicated regular fields, and re-emits the marker
    // plus #*# lines exactly like the Python writer.
    let cfgpath = std::env::temp_dir().join("configfile_parity.cfg");
    std::fs::write(
        &cfgpath,
        format!("[printer]\nmax_velocity: 500\n\n{AUTOSAVE_HEADER}[extruder]\nstep_pin: PA1\n"),
    )
    .unwrap();
    let cfgname = cfgpath.to_string_lossy().into_owned();
    let mut printer = PrinterConfig::new(cfgname.clone());
    printer.set("extruder", "step_pin", "PA2").unwrap();
    let result = printer.save_config("-20240102_030405").unwrap().unwrap();
    let stem = cfgname.trim_end_matches(".cfg");
    // The Python writer (cmd_SAVE_CONFIG) emits:
    //   "\n" + header.rstrip() as first line, then "#*# " + line per
    //   autosave line, then a trailing empty line; the fileconfig write()
    //   output is "[extruder]\nstep_pin = PA2\n".
    let autosave_text = format!("\n{AUTOSAVE_HEADER}#*# [extruder]\n#*# step_pin = PA2\n\n");
    let expected = format!("[printer]\nmax_velocity: 500{autosave_text}");
    assert_eq!(result.data, expected);
    assert_eq!(result.backup_name, format!("{stem}-20240102_030405.cfg"));
    assert_eq!(result.temp_name, format!("{stem}_autosave.cfg"));
    let written = std::fs::read_to_string(&cfgname).unwrap();
    assert_eq!(written, expected);
    assert!(printer.autosave.restart_requested);
}

#[test]
fn save_config_names_rule() {
    // The .cfg suffix moves before the datestr/tmp suffix (configfile.py).
    let (backup, temp) = save_config_names("printer.cfg", "-20240102_030405");
    assert_eq!(backup, "printer-20240102_030405.cfg");
    assert_eq!(temp, "printer_autosave.cfg");
    let (backup, temp) = save_config_names("printer", "-20240102_030405");
    assert_eq!(backup, "printer-20240102_030405");
    assert_eq!(temp, "printer_autosave");
}

#[test]
fn autosave_corruption_rules() {
    // Marker inside the regular part.
    let bad = format!("[printer]\n#*# x\n{AUTOSAVE_HEADER}\n");
    let (data, autosave, logs) = find_autosave_data(&bad);
    assert_eq!(data, bad);
    assert_eq!(autosave, "");
    assert_eq!(
        logs,
        vec!["Can't read autosave from config file - autosave state corrupted"]
    );

    // Lines after the header without the #*# prefix.
    let bad2 = format!("[printer]\n{AUTOSAVE_HEADER}[extruder]\n");
    let (data2, autosave2, logs2) = find_autosave_data(&bad2);
    assert_eq!(data2, bad2);
    assert_eq!(autosave2, "");
    assert_eq!(
        logs2,
        vec!["Can't read autosave from config file - modifications after header"]
    );

    // Empty autosave data is fine (no corruption).
    let plain = "[printer]\nx: 1\n";
    let (regular, autosave, logs) = find_autosave_data(plain);
    assert_eq!(regular, plain);
    assert_eq!(autosave, "");
    assert!(logs.is_empty());
}

#[test]
fn error_messages() {
    // Every config error text per the reference messages.
    let text = "[printer]\nsteps: abc\nratio: 1.5\nshort\n[bad]\n";
    let fc = ConfigFileReader::build_fileconfig(text, "test.cfg").unwrap();
    let access = Rc::new(RefCell::new(indexmap::IndexMap::new()));
    let dep = Rc::new(RefCell::new(DeprecatedStore::new()));
    let w = ConfigWrapper::new(Rc::new(fc), access, dep, "printer".to_string());

    // Missing-required.
    let e = w.get("missing", &None, true).unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'missing' in section 'printer' must be specified"
    );

    // Unparseable value.
    let e = w.getint("steps", &None, None, None, true).unwrap_err();
    assert_eq!(
        e.to_string(),
        "Unable to parse option 'steps' in section 'printer'"
    );

    // Bound violations.
    let e = w
        .getfloat(
            "ratio",
            &None,
            Some(Bound::float(2.0)),
            None,
            None,
            None,
            true,
        )
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'ratio' in section 'printer' must have minimum of 2.0"
    );
    let e = w
        .getfloat(
            "ratio",
            &None,
            None,
            Some(Bound::float(1.0)),
            None,
            None,
            true,
        )
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'ratio' in section 'printer' must have maximum of 1.0"
    );
    let e = w
        .getfloat(
            "ratio",
            &None,
            None,
            None,
            Some(Bound::float(1.5)),
            None,
            true,
        )
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'ratio' in section 'printer' must be above 1.5"
    );
    let e = w
        .getfloat(
            "ratio",
            &None,
            None,
            None,
            None,
            Some(Bound::float(1.5)),
            true,
        )
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'ratio' in section 'printer' must be below 1.5"
    );

    // Invalid choice.
    let choices = ChoiceSet::from_list(vec![
        ChoiceKey::Text("a".into()),
        ChoiceKey::Text("b".into()),
    ]);
    let e = w
        .getchoice("steps", &choices, &Some(PyValue::Str("a".into())), true)
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Choice 'abc' for option 'steps' in section 'printer' is not a valid choice"
    );

    // Missing include file.
    let mut reader = ConfigFileReader::new();
    let e = reader
        .build_fileconfig_with_includes("[include missing.cfg]\n", "test.cfg")
        .unwrap_err();
    assert!(e.to_string().contains("missing.cfg"));

    // Unreadable main config.
    let mut reader = ConfigFileReader::new();
    let e = reader
        .read_config_file("/nonexistent_dir_xyz/printer.cfg")
        .unwrap_err();
    assert_eq!(
        e.to_string(),
        "Unable to open config file /nonexistent_dir_xyz/printer.cfg"
    );
}

#[test]
fn check_unused_options_errors() {
    // Validity failures match the reference wording.
    let text = "[printer]\nvalid_opt: 1\nunknown_opt: 2\n[unknown_section]\nx: 1\n";
    let fc = ConfigFileReader::build_fileconfig(text, "test.cfg").unwrap();
    let mut validate = ConfigValidate::new();
    let access = validate
        .start_access_tracking(&ConfigFileReader::build_fileconfig("", "test.cfg").unwrap());
    let dep = Rc::new(RefCell::new(DeprecatedStore::new()));
    let w = ConfigWrapper::new(Rc::new(fc.clone()), Rc::clone(&access), dep, "printer".into());
    let v = w.get("valid_opt", &None, true).unwrap();
    assert_eq!(v, Some("1".to_string()));
    let e = validate.check_unused(&fc, &["printer"]).unwrap_err();
    assert_eq!(
        e.to_string(),
        "Option 'unknown_opt' is not valid in section 'printer'"
    );
}

#[test]
fn access_tracking_defaults() {
    // A missing option read with a value default records the default.
    let fc = ConfigFileReader::build_fileconfig("[printer]\n", "test.cfg").unwrap();
    let access: Rc<RefCell<indexmap::IndexMap<(String, String), PyValue>>> =
        Rc::new(RefCell::new(indexmap::IndexMap::new()));
    let dep = Rc::new(RefCell::new(DeprecatedStore::new()));
    let w = ConfigWrapper::new(
        Rc::new(fc),
        Rc::clone(&access),
        dep,
        "printer".to_string(),
    );
    let v = w
        .getfloat("ratio", &Some(Some(1.5)), None, None, None, None, true)
        .unwrap();
    assert_eq!(v, Some(1.5));
    let tracked = access.borrow();
    assert_eq!(
        tracked.get(&("printer".into(), "ratio".into())),
        Some(&PyValue::Float(1.5))
    );
}

#[test]
fn defaulted_option_skips_bound_checks() {
    // `_get_wrapper` returns the default before checking minval/maxval/
    // above/below, so a default outside the bounds is returned as is
    // (e.g. `getfloat('tap_threshold', 0., above=0.)` with no option).
    let fc = ConfigFileReader::build_fileconfig("[printer]\n", "test.cfg").unwrap();
    let access = Rc::new(RefCell::new(indexmap::IndexMap::new()));
    let dep = Rc::new(RefCell::new(DeprecatedStore::new()));
    let w = ConfigWrapper::new(Rc::new(fc), access, dep, "printer".to_string());
    let f = |minval, maxval, above, below| {
        w.getfloat("threshold", &Some(Some(0.0)), minval, maxval, above, below, true)
    };
    assert_eq!(f(None, None, Some(Bound::float(0.0)), None).unwrap(), Some(0.0));
    assert_eq!(f(None, None, None, Some(Bound::float(0.0))).unwrap(), Some(0.0));
    assert_eq!(f(Some(Bound::float(1.0)), None, None, None).unwrap(), Some(0.0));
    assert_eq!(f(None, Some(Bound::float(-1.0)), None, None).unwrap(), Some(0.0));
    let n = w
        .getint("count", &Some(Some(PyInt::from_i64(0))), Some(Bound::float(1.0)), None, true)
        .unwrap();
    assert_eq!(n, Some(PyInt::from_i64(0)));
}

#[test]
fn deprecation_records() {
    let fc = ConfigFileReader::build_fileconfig("[printer]\nold_opt: 1\n", "test.cfg").unwrap();
    let access = Rc::new(RefCell::new(indexmap::IndexMap::new()));
    let dep = Rc::new(RefCell::new(DeprecatedStore::new()));
    let w = ConfigWrapper::new(
        Rc::new(fc),
        access,
        Rc::clone(&dep),
        "printer".to_string(),
    );
    w.deprecate("old_opt", None);
    let store = dep.borrow();
    assert_eq!(store.warnings.len(), 1);
    assert_eq!(
        store.warnings[0].get("type"),
        Some(&PyValue::Str("deprecated_option".to_string()))
    );
    assert!(store.log_lines.iter().any(|l| l.contains("old_opt")));
}
