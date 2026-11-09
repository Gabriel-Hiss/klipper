//! Parity tests for the `configfile` port.
//!
//! The main test replays every `tests/fixtures/recordings/configfile_parse`
//! recording: for each scenario it rebuilds the wrapper graph from the
//! scenario's config file (the `config.main` of its `scenario.json`) plus
//! the secondary configs modules read through `PrinterConfig.read_config`
//! (`temperature_sensors.cfg`, `display.cfg`, `menu.cfg`), then executes
//! every recorded `ConfigWrapper` call in `seq` order and compares the
//! result with the recorded value exactly (Python type, floats by bits,
//! strings byte for byte, tuples elementwise).
//!
//! Receivers created outside the recorded calls (roots returned by
//! `read_main_config`/`read_config`, elements of opaque prefix-section
//! lists) start as every (fileconfig, section) pair and keep only the
//! candidates that reproduce each recorded result.
//! Generated cases use the live Python oracle for all expected results,
//! exceptions, SAVE_CONFIG output, warnings and status dictionaries.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use indexmap::IndexMap;
use klippy_api::config::{self as api, Config, PrinterConfigApi};
use klippy_api::error::{KResult, KlippyError};
use klippy_api::gcode::{GCmd, GCodeApi, GCodeCommandHandler, OutputCallback, RegisterCommandOpts};
use klippy_api::object::{EventArg, ObjectRef};
use klippy_api::printer::{EventCallback, LookupDefault, Printer, PrinterApi};
use klippy_api::GCell;
use klippy_core::configfile::{
    find_autosave_data, save_config_names, ConfigAutoSave, ConfigFileReader, ConfigValidate,
    ConfigWrapper, Mounts, PrinterConfig, AUTOSAVE_HEADER,
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

/// `Scenario.resolve` (`parity/harness/run.py:69`): the scenario directory,
/// then the repository root, then `parity/scenarios`.
fn resolve(scenario_dir: &Path, rel: &str) -> PathBuf {
    let root = repo_root();
    [scenario_dir.join(rel), root.join(rel), root.join("parity/scenarios").join(rel)]
        .into_iter()
        .find(|path| path.exists())
        .unwrap_or_else(|| panic!("cannot resolve {rel}"))
}

/// Stage the scenario config as the recording harness did
/// (`parity/harness/run.py:136` `stage_scenario`, `SCENARIO.md` "mounts"):
/// the main file is copied into a private `config/` directory with every
/// mount prefix rewritten to `<temp_root>/mounts/<prefix>`, and the
/// `files` entries are copied beside it. The recorded run's random
/// `temp_root` is part of the recorded option values whenever they name
/// a mount, so it is taken from the recording text; `Mounts` serves the
/// rewritten absolute paths from the scenario's mount directory.
fn stage_scenario(scenario: &str, recording_text: &str) -> (String, Mounts, TempConfig) {
    let mut stack = vec![repo_root().join("parity/scenarios")];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).unwrap() {
            let path = entry.unwrap().path();
            if !path.is_dir() { continue; }
            if path.file_name().unwrap() != scenario { stack.push(path); continue; }
            let raw = std::fs::read_to_string(path.join("scenario.json")).unwrap();
            let data = pycompat::json::loads(&raw).unwrap();
            let config = item(&data, "config");
            const ROOT_PREFIX: &str = "/tmp/harness-";
            let temp_root = match recording_text.find(ROOT_PREFIX) {
                Some(start) => {
                    let after = start + ROOT_PREFIX.len();
                    let end = recording_text[after..]
                        .find(['/', '"'])
                        .map_or(recording_text.len(), |offset| after + offset);
                    recording_text[start..end].to_string()
                }
                None => "/tmp/harness-unrecorded".to_string(),
            };
            let stage = TempConfig::new(&format!("replay-{scenario}"));
            let config_dir = stage.0.join("config");
            std::fs::create_dir_all(&config_dir).unwrap();
            let main_src = resolve(&path, &item(config, "main").str());
            let mut text = std::fs::read_to_string(&main_src).unwrap()
                .replace("\r\n", "\n").replace('\r', "\n");
            let PyValue::List(entries) = item(config, "mounts") else { panic!("mounts list") };
            let mut mounts = Vec::new();
            for entry in entries {
                let prefix = item(entry, "prefix").str();
                let staged = if prefix.is_empty() {
                    format!("{temp_root}/mounts/mnt")
                } else {
                    let staged = format!("{temp_root}/mounts/{}", prefix.trim_start_matches('/'));
                    text = text.replace(&prefix, &staged);
                    staged
                };
                mounts.push((staged, resolve(&path, &item(entry, "dir").str())));
            }
            let main = config_dir.join(main_src.file_name().unwrap());
            std::fs::write(&main, text).unwrap();
            let PyValue::List(files) = item(config, "files") else { panic!("files list") };
            for file in files {
                let src = resolve(&path, &file.str());
                std::fs::copy(&src, config_dir.join(src.file_name().unwrap())).unwrap();
            }
            return (main.to_string_lossy().into_owned(), Mounts::new(mounts), stage);
        }
    }
    panic!("no scenario dir for {scenario}")
}

/// A config the modules load with `pconfig.read_config(filename)`
/// (`heaters.py:267`, `display.py:120`, `menu.py:1004`): read and parse
/// without includes, exactly as `PrinterConfig.read_config` does.
fn module_config(relative: &str) -> RawConfigParser {
    let path = repo_root().join(relative).to_string_lossy().into_owned();
    let data = ConfigFileReader::new().read_config_file(&path).unwrap();
    ConfigFileReader::build_fileconfig(&data, &path).unwrap()
}

/// Shared wrapper state for one replayed scenario. Access tracking is one
/// dict shared by every wrapper, as in Python.
struct World {
    printer: Printer,
    /// Fileconfigs of this scenario (main first, then the module configs).
    fileconfigs: Vec<Arc<RawConfigParser>>,
    wrappers: BTreeMap<String, Vec<ConfigWrapper>>,
    access: Arc<GCell<IndexMap<(String, String), PyValue>>>,
}

impl World {
    fn wrapper(&self, fc: usize, section: &str) -> ConfigWrapper {
        ConfigWrapper::new(
            Arc::clone(&self.printer),
            Arc::clone(&self.fileconfigs[fc]),
            Arc::clone(&self.access),
            section.to_string(),
        )
    }

    /// Every (fileconfig, section) pair, for lazy receiver resolution.
    fn all_candidates(&self) -> Vec<ConfigWrapper> {
        let mut out = Vec::new();
        for (fc, parser) in self.fileconfigs.iter().enumerate() {
            for section in parser.sections() {
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

/// Opaque arguments in default position are the `sentinel` class.
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

fn rv_i64(v: &Rv) -> i64 {
    match v {
        Rv::Int(i) => *i,
        Rv::BigInt(s) => s.parse().unwrap(),
        other => panic!("expected int, got {}", other.render()),
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
        Rv::Dict(pairs) => PyValue::Dict(pairs.iter().map(|(k, v)| (to_pyvalue(k), to_pyvalue(v))).collect()),
        Rv::Struct { fields, .. } => PyValue::Dict(
            fields.iter().map(|(name, v)| (PyValue::Str(name.clone()), to_pyvalue(v))).collect(),
        ),
        Rv::Opaque { addr: id, .. } | Rv::ObjectId(id) => {
            PyValue::Host(id.rsplit('#').next().and_then(|s| s.parse().ok()).unwrap_or(0))
        }
    }
}

/// A recorded `default` argument (`None` is the sentinel).
fn default_arg(v: &Rv) -> Option<PyValue> {
    if is_sentinel(v) { None } else { Some(to_pyvalue(v)) }
}

/// A recorded `minval`/`maxval`/`above`/`below` argument.
fn bound_arg(v: &Rv) -> Option<PyValue> {
    match v {
        Rv::Null => None,
        other => Some(to_pyvalue(other)),
    }
}

/// A recorded `count` argument.
fn count_arg(v: &Rv) -> Option<i64> {
    match v {
        Rv::Null => None,
        other => Some(rv_i64(other)),
    }
}

fn leaf_parser(v: &Rv) -> Option<api::ListParser> {
    match v {
        Rv::Tuple(items) | Rv::List(items) => items.iter().find_map(leaf_parser),
        Rv::Float(_) => Some(api::ListParser::Float),
        Rv::Int(_) | Rv::BigInt(_) => Some(api::ListParser::Int),
        Rv::Str(_) => Some(api::ListParser::Str),
        _ => None,
    }
}

/// Exact equality of a ported value with a recorded one: same Python type,
/// floats by bits (the recorded hex is CPython's spelling).
fn vals_equal(actual: &PyValue, expected: &Rv) -> bool {
    match (actual, expected) {
        (PyValue::None, Rv::Null) => true,
        (PyValue::Bool(a), Rv::Bool(b)) => a == b,
        (PyValue::Int(a), Rv::Int(b)) => a.inner().to_string() == b.to_string(),
        (PyValue::Int(a), Rv::BigInt(b)) => a.inner().to_string() == *b,
        (PyValue::Float(a), Rv::Float(b)) => b
            .to_f64()
            .is_some_and(|b| b.to_bits() == a.to_bits() || (a.is_nan() && b.is_nan())),
        (PyValue::Str(a), Rv::Str(b)) => a == b,
        (PyValue::Bytes(a), Rv::Bytes(b)) => a == b,
        (PyValue::Tuple(a), Rv::Tuple(b)) | (PyValue::List(a), Rv::List(b)) => {
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
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Recorded call execution.
// ---------------------------------------------------------------------------

fn port_error(e: KlippyError) -> String {
    format!("{}: {}", e.class_name(), e.message())
}

/// Execute one recorded `ConfigWrapper` call on `w`.
fn exec_getter(
    func: &str,
    w: &ConfigWrapper,
    args: &[Rv],
    parser_hints: &BTreeMap<String, api::ListParser>,
) -> Result<PyValue, String> {
    let opt = rv_str(&args[1]);
    let result = match func {
        "ConfigWrapper.get" => w.get(&opt, default_arg(&args[2]).as_ref(), rv_bool(&args[3])),
        "ConfigWrapper.getint" => w.getint(
            &opt,
            default_arg(&args[2]).as_ref(),
            bound_arg(&args[3]),
            bound_arg(&args[4]),
            rv_bool(&args[5]),
        ),
        "ConfigWrapper.getfloat" => w.getfloat(
            &opt,
            default_arg(&args[2]).as_ref(),
            bound_arg(&args[3]),
            bound_arg(&args[4]),
            bound_arg(&args[5]),
            bound_arg(&args[6]),
            rv_bool(&args[7]),
        ),
        "ConfigWrapper.getboolean" => {
            w.getboolean(&opt, default_arg(&args[2]).as_ref(), rv_bool(&args[3]))
        }
        "ConfigWrapper.getchoice" => w.getchoice(
            &opt,
            &to_pyvalue(&args[2]),
            default_arg(&args[3]).as_ref(),
            rv_bool(&args[4]),
        ),
        "ConfigWrapper.getlist" => w.getlist(
            &opt,
            default_arg(&args[2]).as_ref(),
            &rv_str(&args[3]),
            count_arg(&args[4]),
            rv_bool(&args[5]),
        ),
        "ConfigWrapper.getintlist" => w.getintlist(
            &opt,
            default_arg(&args[2]).as_ref(),
            &rv_str(&args[3]),
            count_arg(&args[4]),
            rv_bool(&args[5]),
        ),
        "ConfigWrapper.getfloatlist" => w.getfloatlist(
            &opt,
            default_arg(&args[2]).as_ref(),
            &rv_str(&args[3]),
            count_arg(&args[4]),
            rv_bool(&args[5]),
        ),
        "ConfigWrapper.getlists" => {
            let seps: Vec<String> = match &args[3] {
                Rv::Tuple(items) => items.iter().map(rv_str).collect(),
                other => panic!("expected seps tuple, got {}", other.render()),
            };
            let parser = match &args[5] {
                Rv::ObjectId(id) => *parser_hints.get(id).unwrap_or(&api::ListParser::Str),
                other => panic!("expected parser oid, got {}", other.render()),
            };
            w.getlists(
                &opt,
                default_arg(&args[2]).as_ref(),
                &seps,
                count_arg(&args[4]),
                parser,
                rv_bool(&args[6]),
            )
        }
        "ConfigWrapper.get_prefix_options" => {
            return w
                .get_prefix_options(&opt)
                .map(|opts| PyValue::List(opts.into_iter().map(PyValue::Str).collect()))
                .map_err(|e| e.to_string());
        }
        "ConfigWrapper.get_prefix_sections" => {
            return Ok(PyValue::List(
                w.get_prefix_sections(&opt)
                    .into_iter()
                    .map(|c| PyValue::Str(c.section_name().to_string()))
                    .collect(),
            ));
        }
        "ConfigWrapper.has_section" => return Ok(PyValue::Bool(w.has_section(&opt))),
        _ => panic!("unexpected function {func}"),
    };
    result.map_err(port_error)
}

/// Opaque-dict `getchoice`: the recorder hides the dict, so the choices
/// map the stored text to the recorded return object; the comparison then
/// checks the reference lookup path (option text, default, tracking).
fn opaque_dict_call(w: &ConfigWrapper, args: &[Rv], ret: &Rv) -> Result<PyValue, String> {
    let opt = rv_str(&args[1]);
    let raw = w
        .fileconfig()
        .get(w.section_name(), &opt)
        .map_err(|e| e.to_string())?
        .map(PyValue::Str)
        .unwrap_or(PyValue::None);
    let choices = PyValue::Dict([(raw, to_pyvalue(ret))].into_iter().collect());
    w.getchoice(&opt, &choices, default_arg(&args[3]).as_ref(), rv_bool(&args[4]))
        .map_err(port_error)
}

// ---------------------------------------------------------------------------
// Replay.
// ---------------------------------------------------------------------------

fn replay_scenario(scenario: &str, records: Vec<Record>, recording_text: &str) -> Result<usize, String> {
    let (main, mounts, _stage) = stage_scenario(scenario, recording_text);
    let mut autosave = ConfigAutoSave::new(main, mounts);
    let (fileconfig, saved) = autosave.load_main_config()
        .map_err(|e| format!("load_main_config: {e}"))?;
    let mut validate = ConfigValidate::new();
    let access = validate.start_access_tracking(&saved);

    let mut world = World {
        printer: FakePrinter::new(String::new()),
        fileconfigs: vec![
            Arc::new(fileconfig),
            Arc::new(module_config("klippy/extras/temperature_sensors.cfg")),
            Arc::new(module_config("klippy/extras/display/display.cfg")),
            Arc::new(module_config("klippy/extras/display/menu.cfg")),
        ],
        wrappers: BTreeMap::new(),
        access,
    };
    let root = world.wrapper(0, "printer");
    world.wrappers.insert("ConfigWrapper#2".to_string(), vec![root]);

    // The recorder hides the `parser` callable of `getlists`; the same oid
    // is always the same callable, so its kind follows the element type of
    // any non-empty result it produced in this scenario.
    let mut parser_hints: BTreeMap<String, api::ListParser> = BTreeMap::new();
    for r in &records {
        if r.function != "ConfigWrapper.getlists" { continue; }
        if let (Some(Rv::ObjectId(id)), Some(ret)) = (r.args.get(5), &r.ret) {
            if let Some(p) = leaf_parser(ret) {
                parser_hints.insert(id.clone(), p);
            }
        }
    }

    let mut count = 0usize;
    for (index, record) in records.iter().enumerate() {
        count += 1;
        let oid = record.object_id.clone().unwrap_or_default();
        // `_get_wrapper` returns inside the getter recorded right after it,
        // with the same receiver and result.
        let call = if record.function == "ConfigWrapper._get_wrapper" {
            records.get(index + 1).ok_or("missing outer getter after _get_wrapper")?
        } else {
            record
        };
        match call.function.as_str() {
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
                let mut children: Vec<ConfigWrapper> = Vec::new();
                for parent in world.candidates(&oid) {
                    if children.iter().any(|c| std::ptr::eq(c.fileconfig(), parent.fileconfig())) {
                        continue;
                    }
                    children.push(parent.getsection(&section));
                }
                world.wrappers.insert(ret_oid, children);
            }
            _ => {
                let Some(ret) = &record.ret else {
                    return Err(format!("seq {} {}: no recorded return value", record.seq, record.function));
                };
                let mut matched = Vec::new();
                let mut last = String::new();
                for cand in world.candidates(&oid) {
                    let result = if call.function == "ConfigWrapper.getchoice"
                        && matches!(&call.args[2], Rv::ObjectId(_))
                    {
                        opaque_dict_call(&cand, &call.args, ret)
                    } else {
                        exec_getter(&call.function, &cand, &call.args, &parser_hints)
                    };
                    match result {
                        Ok(v) => {
                            // Opaque prefix-section lists accept any body;
                            // their elements are resolved by later calls.
                            let accept = match ret {
                                Rv::ObjectId(_) if call.function == "ConfigWrapper.get_prefix_sections" => true,
                                _ => vals_equal(&v, ret),
                            };
                            last = v.repr();
                            if accept {
                                matched.push(cand);
                            }
                        }
                        Err(e) => {
                            // The recorder logs a call that raised as
                            // `ret: null` (printers-070 seq 563-564: the
                            // `getboolean` that `menu.py:39` catches), so a
                            // recorded null also admits `configparser.Error`.
                            if matches!(ret, Rv::Null) && e.starts_with("Error: ") {
                                matched.push(cand);
                            }
                            last = e;
                        }
                    }
                }
                if matched.is_empty() {
                    return Err(format!(
                        "seq {} {}: expected {} actual {}",
                        record.seq, record.function, ret.render(), last
                    ));
                }
                world.wrappers.insert(oid.clone(), matched);
            }
        }
        let actual_mutations = match (record.function.as_str(), record.args.get(2)) {
            // `choices = {i: i for i in choices}` rebinds the list argument.
            ("ConfigWrapper.getchoice", Some(Rv::List(keys))) => {
                let mut choices = IndexMap::new();
                for key in keys {
                    let key = to_pyvalue(key);
                    choices.insert(key.clone(), key);
                }
                vec![(2, record_value(&PyValue::Dict(choices)))]
            }
            _ => Vec::new(),
        };
        assert_eq!(record.side_effects.mutated_args, actual_mutations, "seq {} mutated arguments", record.seq);
        assert!(record.side_effects.mcu_messages.is_empty(), "seq {} unexpected MCU output", record.seq);
        assert!(record.side_effects.log_lines.is_empty(), "seq {} unexpected log output", record.seq);
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
        let recording_text = std::fs::read_to_string(&path).unwrap();
        let scenario = records.first().map(|r| r.scenario.clone()).unwrap();
        let count = replay_scenario(&scenario, records, &recording_text)
            .unwrap_or_else(|d| panic!("{scenario} ({}): {d}", path.display()));
        eprintln!("replayed {scenario}: {count} calls");
        total += count;
    }
    assert!(total > 0, "no recordings replayed");
}

fn item<'a>(value: &'a PyValue, key: &str) -> &'a PyValue {
    let PyValue::Dict(values) = value else { panic!("expected dict") };
    values.get(&PyValue::Str(key.into())).unwrap_or_else(|| panic!("missing {key}"))
}

struct FakePrinter {
    filename: String,
    objects: GCell<IndexMap<String, ObjectRef>>,
    rollover: GCell<Vec<String>>,
}

impl FakePrinter {
    fn new(filename: String) -> Arc<Self> {
        let printer = Arc::new(Self {
            filename, objects: GCell::new(IndexMap::new()), rollover: GCell::new(Vec::new()),
        });
        printer.add_object("gcode", Arc::new(FakeGCode::default())).unwrap();
        printer
    }
}

impl PrinterApi for FakePrinter {
    fn get_start_args(&self) -> PyValue {
        PyValue::Dict([
            (PyValue::Str("config_file".into()), PyValue::Str(self.filename.clone())),
            (PyValue::Str("software_version".into()), PyValue::Str("host-test".into())),
        ].into_iter().collect())
    }
    fn get_reactor(&self) -> reactor::Reactor { reactor::Reactor::new(false) }
    fn get_state_message(&self) -> (String, String) { ("ready".into(), "ready".into()) }
    fn is_shutdown(&self) -> bool { false }
    fn update_error_msg(&self, _: &str, _: &str) {}
    fn add_object(&self, name: &str, object: ObjectRef) -> KResult<()> {
        self.objects.borrow_mut().insert(name.into(), object);
        Ok(())
    }
    fn lookup_object(&self, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        match self.objects.borrow().get(name).cloned() {
            value @ Some(_) => Ok(value),
            None if default == LookupDefault::ReturnNone => Ok(None),
            None => Err(KlippyError::Config(format!("Unknown config object '{name}'"))),
        }
    }
    fn lookup_objects(&self, _: Option<&str>) -> Vec<(String, ObjectRef)> {
        self.objects.borrow().iter().map(|(k,v)| (k.clone(), v.clone())).collect()
    }
    fn load_object(&self, _: &Config, name: &str, default: LookupDefault) -> KResult<Option<ObjectRef>> {
        self.lookup_object(name, default)
    }
    fn run(&self) -> Option<String> { None }
    fn set_rollover_info(&self, _: &str, value: Option<&str>, _: bool) {
        if let Some(value) = value { self.rollover.borrow_mut().push(value.into()); }
    }
    fn invoke_shutdown(&self, _: &str, _: PyValue) {}
    fn invoke_async_shutdown(&self, _: &str, _: PyValue) {}
    fn register_event_handler(&self, _: &str, _: EventCallback) {}
    fn send_event(&self, _: &str, _: &[EventArg]) -> KResult<Vec<PyValue>> { Ok(Vec::new()) }
    fn request_exit(&self, _: &str) {}
}

#[derive(Default)]
struct FakeGCode {
    handlers: GCell<IndexMap<String, GCodeCommandHandler>>,
    help: GCell<IndexMap<String, String>>,
    restarts: GCell<Vec<String>>,
}
klippy_api::printer_object!(FakeGCode; dyn GCodeApi);

impl GCodeApi for FakeGCode {
    fn register_command(&self, name: &str, handler: Option<GCodeCommandHandler>, opts: RegisterCommandOpts)
        -> KResult<Option<GCodeCommandHandler>> {
        if let Some(help) = opts.desc { self.help.borrow_mut().insert(name.into(), help); }
        let old = self.handlers.borrow_mut().shift_remove(name);
        if let Some(handler) = handler { self.handlers.borrow_mut().insert(name.into(), handler); }
        Ok(old)
    }
    fn register_mux_command(&self, _: &str, _: &str, _: Option<&str>, _: GCodeCommandHandler, _: Option<&str>) -> KResult<()> { panic!("unused mux") }
    fn get_command_help(&self) -> IndexMap<String, String> { self.help.cloned() }
    fn get_status(&self, _: f64) -> PyValue { PyValue::None }
    fn register_output_handler(&self, _: OutputCallback) { panic!("unused output") }
    fn request_restart(&self, result: &str) -> KResult<()> {
        self.restarts.borrow_mut().push(result.into());
        Ok(())
    }
    fn run_script(&self, _: &str) -> KResult<()> { panic!("unused script") }
    fn run_script_from_command(&self, _: &str) -> KResult<()> { panic!("unused script") }
    fn get_mutex(&self) -> reactor::ReactorMutex { panic!("unused mutex") }
    fn create_gcode_command(&self, _: &str, _: &str, _: &IndexMap<String, String>) -> GCmd { panic!("unused command construction") }
    fn respond_raw(&self, _: &str) { panic!("unused response") }
    fn respond_info(&self, _: &str, _: bool) { panic!("unused response") }
    fn is_traditional_gcode(&self, _: &str) -> bool { false }
    fn cmd_m112(&self, _: Option<&GCmd>) -> KResult<()> { panic!("unused emergency stop") }
    fn process_commands(&self, _: &[String], _: bool) -> KResult<()> { panic!("unused process") }
}

struct TempConfig(PathBuf);
impl TempConfig {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!("configfile-{}-{label}", std::process::id()));
        std::fs::create_dir_all(&path).unwrap();
        Self(path)
    }
    fn filename(&self) -> String { self.0.join("printer.cfg").to_string_lossy().into_owned() }
}
impl Drop for TempConfig {
    fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); }
}

fn record_value(value: &PyValue) -> Rv {
    match value {
        PyValue::None => Rv::Null,
        PyValue::Bool(v) => Rv::Bool(*v),
        PyValue::Int(v) => match v.inner().to_string().parse() {
            Ok(n) => Rv::Int(n), Err(_) => Rv::BigInt(v.inner().to_string()),
        },
        PyValue::Float(v) => Rv::Float(FloatVal {
            hex: recording::f64_to_hex(*v), repr: pycompat::text::float_str(*v),
        }),
        PyValue::Str(v) => Rv::Str(v.clone()),
        PyValue::Bytes(v) => Rv::Bytes(v.clone()),
        PyValue::Tuple(v) => Rv::Tuple(v.iter().map(record_value).collect()),
        PyValue::List(v) => Rv::List(v.iter().map(record_value).collect()),
        PyValue::Dict(v) => Rv::Dict(v.iter().map(|(k,v)| (record_value(k), record_value(v))).collect()),
        PyValue::Host(_) => panic!("unexpected opaque generated value"),
    }
}

fn compare(label: &str, expected: pyoracle::OracleResult, actual: KResult<PyValue>) {
    match (expected, actual) {
        (Ok(expected), Ok(actual)) => {
            let record = Record {
                v: 1, unit: "configfile".into(), scenario: label.into(), seq: 0,
                function: label.into(), args: Vec::new(), ret: None, exc: None,
                side_effects: Default::default(), object_id: None,
                kind: recording::value::RecordKind::Python,
            };
            recording::assert_equal(&record, "ret", &expected, &record_value(&actual)).unwrap();
        }
        (Err(expected), Err(actual)) => {
            assert_eq!(expected.class_name, actual.class_name(), "{label}");
            assert_eq!(expected.message, actual.message(), "{label}");
        }
        (expected, actual) => panic!("{label}: reference {expected:?}, port {actual:?}"),
    }
}

fn oracle() -> Option<pyoracle::Oracle> {
    let Some(mut py) = pyoracle::Oracle::start() else {
        eprintln!("configfile oracle skipped: KLIPPY_PYTHON is not set");
        return None;
    };
    py.exec(r#"
import configfile, logging, types, os
class CommandError(Exception): pass
messages = []
class Capture(logging.Handler):
    def emit(self, record):
        messages.append(record.getMessage())
logging.getLogger().handlers = [Capture()]
logging.getLogger().setLevel(logging.INFO)
class GCode:
    def __init__(self):
        self.handlers = {}
        self.help = {}
        self.restarts = []
    def register_command(self, name, handler, desc=None):
        self.handlers[name] = handler
        self.help[name] = desc
    def request_restart(self, result):
        self.restarts.append(result)
class Printer:
    command_error = CommandError
    def __init__(self, filename):
        self.start_args = {'config_file': filename, 'software_version': 'host-test'}
        self.objects = {'gcode': GCode()}
        self.rollover = []
    def get_start_args(self):
        return self.start_args
    def lookup_object(self, name):
        return self.objects[name]
    def lookup_objects(self):
        return list(self.objects.items())
    def set_rollover_info(self, name, value):
        self.rollover.append(value)
def setup(filename):
    global p, pc
    p = Printer(filename)
    pc = configfile.PrinterConfig(p)
    p.objects['configfile'] = pc
def split_data(data):
    messages.clear()
    result = pc.autosave._find_autosave_data(data)
    return (result[0], result[1], list(messages))
def save_data(filename, text, changes):
    with open(filename, 'w') as f:
        f.write(text)
    setup(filename)
    pc.autosave.fileconfig = configfile.ConfigFileReader()._create_fileconfig()
    messages.clear()
    for section, option, value in changes:
        pc.set(section, option, value)
    configfile.time.strftime = lambda fmt: '-20240102_030405'
    pc.autosave.cmd_SAVE_CONFIG(types.SimpleNamespace(error=CommandError))
    with open(filename) as f:
        text = f.read()
    return (text, list(messages), p.objects['gcode'].restarts)
"#).unwrap();
    Some(py)
}

fn scalar<T>(value: KResult<Option<T>>, convert: impl FnOnce(T) -> PyValue) -> KResult<PyValue> {
    value.map(|v| v.map(convert).unwrap_or(PyValue::None))
}

#[test]
fn oracle_getters_and_error_messages() {
    let Some(mut py) = oracle() else { return };
    let temp = TempConfig::new("getters");
    let filename = temp.filename();
    let text = "[printer]\nsteps: abc\nratio: 1.5\ninteger: 3\nflag: no\nbad_flag: perhaps\nlist: 1, 2\nnested: 1,2;3,4\nempty:\nold_opt: 1\n";
    std::fs::write(&filename, text).unwrap();
    py.exec(&format!("setup({filename:?})\nw = pc.read_main_config()")).unwrap();
    let printer: Printer = FakePrinter::new(filename);
    let pc = PrinterConfig::new(printer.clone()).unwrap();
    printer.add_object("configfile", pc.clone()).unwrap();
    // The trait object and the inherent wrapper share one wrapper state, so
    // both feed the access tracking compared below.
    let wi = pc.read_main_config().unwrap();
    let w: Config = Arc::new(wi.clone());
    let int = |n: i64| PyValue::Int(PyInt::from_i64(n));
    let get = |s: &str| scalar(w.get(s, Default::default()), PyValue::Str);
    let getint = |s: &str, opts| scalar(w.getint(s, opts), |v| PyValue::Int(v.into()));
    let getfloat = |s: &str, opts| scalar(w.getfloat(s, opts), PyValue::Float);
    let cases = vec![
        ("w.get('missing')", get("missing")),
        ("w.getint('steps')", getint("steps", api::GetIntOpts::default())),
        ("w.getint('integer', minval=4)", getint("integer", api::GetIntOpts { minval: Some(4), ..Default::default() })),
        ("w.getint('integer', maxval=2)", getint("integer", api::GetIntOpts { maxval: Some(2), ..Default::default() })),
        ("w.getfloat('ratio', minval=2.)", getfloat("ratio", api::GetFloatOpts { minval: Some(2.0), ..Default::default() })),
        ("w.getfloat('ratio', maxval=1.)", getfloat("ratio", api::GetFloatOpts { maxval: Some(1.0), ..Default::default() })),
        ("w.getfloat('ratio', above=1.5)", getfloat("ratio", api::GetFloatOpts { above: Some(1.5), ..Default::default() })),
        ("w.getfloat('ratio', below=1.5)", getfloat("ratio", api::GetFloatOpts { below: Some(1.5), ..Default::default() })),
        ("w.getfloat('ratio', minval=1., maxval=2.)", getfloat("ratio", api::GetFloatOpts { minval: Some(1.0), maxval: Some(2.0), ..Default::default() })),
        ("w.getfloat('missing', 0., above=0.)", getfloat("missing", api::GetFloatOpts { default: Some(Some(0.0)), above: Some(0.0), ..Default::default() })),
        ("w.getfloat('optional', None)", getfloat("optional", api::GetFloatOpts { default: Some(None), ..Default::default() })),
        ("w.getboolean('flag')", scalar(w.getboolean("flag", Default::default()), PyValue::Bool)),
        ("w.getboolean('bad_flag')", scalar(w.getboolean("bad_flag", Default::default()), PyValue::Bool)),
        ("w.getlists('list', parser=int)", w.getlists("list", api::GetListsOpts { parser: api::ListParser::Int, ..Default::default() })),
        ("w.getlists('list', count=3)", w.getlists("list", api::GetListsOpts { count: Some(3), ..Default::default() })),
        ("w.getlists('nested', seps=(',', ';'), parser=float)", w.getlists("nested", api::GetListsOpts { seps: vec![",".into(), ";".into()], parser: api::ListParser::Float, ..Default::default() })),
        ("w.getlists('empty', seps=())", w.getlists("empty", api::GetListsOpts { seps: vec![], ..Default::default() })),
        ("w.getlists('list', seps=('',))", w.getlists("list", api::GetListsOpts { seps: vec!["".into()], ..Default::default() })),
        ("w.getlists('empty')", w.getlists("empty", Default::default())),
        ("w.getchoice('steps', ['a', 'b'])", w.getchoice("steps", &PyValue::List(vec![PyValue::Str("a".into()), PyValue::Str("b".into())]), Default::default())),
        ("w.getchoice('missing_choice', {1: 'one'}, default='one')", w.getchoice("missing_choice", &PyValue::Dict([(PyValue::Int(PyInt::from_i64(1)), PyValue::Str("one".into()))].into_iter().collect()), api::GetChoiceOpts { default: Some(Some(PyValue::Str("one".into()))), ..Default::default() })),
        // Python defaults and bounds of any type, through the inherent API.
        ("w.getfloat('missing_float', 0, minval=1.)", wi.getfloat("missing_float", Some(&int(0)), Some(PyValue::Float(1.0)), None, None, None, true)),
        ("w.getint('integer', minval=3.5)", wi.getint("integer", None, Some(PyValue::Float(3.5)), None, true)),
        ("w.getint('integer', maxval=3.0)", wi.getint("integer", None, None, Some(PyValue::Float(3.0)), true)),
        ("w.getfloat('ratio', above=1)", wi.getfloat("ratio", None, None, None, Some(int(1)), None, true)),
        ("w.getfloat('ratio', below=1)", wi.getfloat("ratio", None, None, None, None, Some(int(1)), true)),
        ("w.get('missing_str', 5)", wi.get("missing_str", Some(&int(5)), true)),
        ("w.getboolean('missing_bool', 0)", wi.getboolean("missing_bool", Some(&int(0)), true)),
        ("w.getlists('missing_list', [])", wi.getlists("missing_list", Some(&PyValue::List(vec![])), &[",".to_string()], None, api::ListParser::Str, true)),
        ("w.getintlist('nested', sep=';')", wi.getintlist("nested", None, ";", None, true)),
        ("w.getchoice('integer', [3, 3.0])", wi.getchoice("integer", &PyValue::List(vec![int(3), PyValue::Float(3.0)]), None, true)),
        ("w.getchoice('steps', ('a',))", wi.getchoice("steps", &PyValue::Tuple(vec![PyValue::Str("a".into())]), None, true)),
        ("w.getchoice('steps', {})", wi.getchoice("steps", &PyValue::Dict(IndexMap::new()), None, true)),
    ];
    for (expression, actual) in cases { compare(expression, py.eval(expression), actual); }
    pc.validate.borrow_mut().build_status_settings();
    py.exec("pc.validate._build_status_settings()").unwrap();
    compare("access tracking", py.eval("pc.get_status(0.)['settings']"),
        Ok(item(&pc.get_status(), "settings").clone()));
    for text in ["[printer]\nshort\n", "option: no-section\n"] {
        py.exec(&format!("bad_text = {text:?}")).unwrap();
        let actual = ConfigFileReader::build_fileconfig(text, "test.cfg")
            .map(|_| PyValue::None).map_err(config_error);
        compare("parser error", py.eval("configfile.ConfigFileReader().build_fileconfig(bad_text, 'test.cfg') and None"), actual);
    }
}

fn config_error(error: pytext::configparser::Error) -> KlippyError {
    use pytext::configparser::Error;
    let class_name = match &error {
        Error::Error(_) => "Error",
        Error::ParsingError { .. } => "ParsingError",
        Error::MissingSectionHeaderError { .. } => "MissingSectionHeaderError",
        Error::ValueError(_) => "ValueError",
        _ => panic!("unexpected generated parser error {error:?}"),
    };
    KlippyError::Other { class_name: class_name.into(), message: error.to_string() }
}

#[test]
fn oracle_autosave_and_save_config() {
    let Some(mut py) = oracle() else { return };
    let temp = TempConfig::new("autosave");
    let filename = temp.filename();
    py.exec(&format!("setup({filename:?})")).unwrap();
    for text in [
        "[printer]\nx: 1\n".to_string(),
        format!("[printer]\n{AUTOSAVE_HEADER}"),
        format!("[printer]\n#*# x\n{AUTOSAVE_HEADER}\n"),
        format!("[printer]\n{AUTOSAVE_HEADER}[extruder]\n"),
        format!("[printer]\n{AUTOSAVE_HEADER}#*# [extruder]\n#*# x = 1\n"),
        format!("[printer]\n{AUTOSAVE_HEADER}#*#x\n"),
        format!("[printer]\n{AUTOSAVE_HEADER}#*# [extruder]\n{AUTOSAVE_HEADER}#*# x = 1\n"),
    ] {
        let (regular, saved, logs) = find_autosave_data(&text);
        compare("autosave split", py.eval(&format!("split_data({text:?})")), Ok(PyValue::Tuple(vec![
            PyValue::Str(regular), PyValue::Str(saved),
            PyValue::List(logs.into_iter().map(PyValue::Str).collect()),
        ])));
    }
    let included = temp.0.join("included.cfg");
    std::fs::write(included, "[extruder]\nstep_pin: included\n").unwrap();
    for text in [
        "[printer]\nmax_velocity: 500\n".to_string(),
        format!("[printer]\nmax_velocity: 500\n{AUTOSAVE_HEADER}#*# [extruder]\n#*# step_pin = PA1\n"),
        "[printer]\nmax_velocity: 500\n[extruder]\nstep_pin: PA1\n  continuation\n".into(),
        format!("[printer]\n{AUTOSAVE_HEADER}[extruder]\nstep_pin: PA1\n"),
        "[include included.cfg]\n[printer]\n".into(),
        "[printer]\nshort\n".into(),
    ] {
        let expected = py.eval(&format!("save_data({filename:?}, {text:?}, [('extruder', 'step_pin', 'PA2')])"));
        std::fs::write(&filename, &text).unwrap();
        let mut save = ConfigAutoSave::new(filename.clone(), Mounts::default());
        save.set("extruder", "step_pin", "PA2").unwrap();
        let actual = save.save_config(&filename, "-20240102_030405")
            .map(|saved| {
                let saved = saved.expect("autosave has sections");
                let written = std::fs::read_to_string(&filename).unwrap();
                assert_eq!(written, saved.data);
                let (backup, temporary) = save_config_names(&filename, "-20240102_030405");
                assert_eq!(saved.backup_name, backup);
                assert_eq!(saved.temp_name, temporary);
                assert_eq!(std::fs::read_to_string(backup).unwrap(), text);
                PyValue::Tuple(vec![
                    PyValue::Str(written),
                    PyValue::List(save.log_lines.iter().cloned().map(PyValue::Str).collect()),
                    PyValue::List(if save.restart_requested { vec![PyValue::Str("restart".into())] } else { vec![] }),
                ])
            }).map_err(|e| KlippyError::Command(e.to_string()));
        compare("SAVE_CONFIG", expected, actual);
    }
}

#[test]
fn oracle_runtime_status_warnings_and_validation() {
    let Some(mut py) = oracle() else { return };
    let temp = TempConfig::new("status");
    let filename = temp.filename();
    let text = "[printer]\nold_opt: 1\nratio: 1.5\n";
    std::fs::write(&filename, text).unwrap();
    py.exec(&format!("setup({filename:?})\nw = pc.read_main_config()")).unwrap();
    let fake = FakePrinter::new(filename.clone());
    let printer: Printer = fake.clone();
    (klippy_core::configfile::module_factory().add_early_printer_objects.unwrap())(&printer);
    let pc = klippy_api::lookup_interface::<dyn PrinterConfigApi>(&*printer, "configfile", LookupDefault::Raise).unwrap().unwrap();
    let w = pc.read_main_config().unwrap();
    assert!(Arc::ptr_eq(&w.get_printer(), &printer));
    compare("initial status", py.eval("pc.get_status(0.)"), Ok(pc.get_status(Some(0.0))));
    w.deprecate("old_opt", None);
    w.deprecate("old_opt", Some(PyValue::None));
    w.deprecate("missing", None);
    pc.deprecate("printer", "old_opt", Some(PyValue::Str("1".into())), None);
    pc.runtime_warning("runtime warning");
    pc.runtime_warning("runtime warning");
    pc.deprecate_gcode("OLD", Default::default());
    pc.deprecate_gcode("OLD", api::DeprecateGcodeOpts { param: Some("P".into()), value: Some(PyValue::Float(1.5)), msg: None });
    pc.deprecate("printer", "ratio", Some(PyValue::Int(PyInt::from_i64(1))), None);
    pc.deprecate("printer", "ratio", Some(PyValue::Float(1.0)), Some("Value '1' in option 'ratio' in section 'printer' is deprecated.".into()));
    py.exec(r#"
messages.clear()
w.deprecate('old_opt')
w.deprecate('old_opt', None)
w.deprecate('missing')
pc.deprecate('printer', 'old_opt', '1')
pc.runtime_warning('runtime warning')
pc.runtime_warning('runtime warning')
pc.deprecate_gcode('OLD')
pc.deprecate_gcode('OLD', 'P', 1.5)
pc.deprecate('printer', 'ratio', 1)
pc.deprecate('printer', 'ratio', 1.0, "Value '1' in option 'ratio' in section 'printer' is deprecated.")
"#).unwrap();
    compare("warnings", py.eval("pc.get_status(0.)"), Ok(pc.get_status(None)));
    let concrete = klippy_api::downcast_arc::<PrinterConfig>(&printer.lookup_object("configfile", LookupDefault::Raise).unwrap().unwrap()).unwrap();
    compare("warning logs", py.eval("messages"), Ok(PyValue::List(concrete.deprecated.borrow().log_lines.iter().cloned().map(PyValue::Str).collect())));
    let _ = w.get("old_opt", Default::default()).unwrap();
    py.eval("w.get('old_opt')").unwrap();
    compare("unused option", py.eval("pc.check_unused_options(w)"), pc.check_unused_options(&w).map(|_| PyValue::None));
    let _ = w.getfloat("ratio", Default::default()).unwrap();
    py.eval("w.getfloat('ratio')").unwrap();
    compare("validated settings", py.eval("pc.check_unused_options(w)"), pc.check_unused_options(&w).map(|_| PyValue::None));
    compare("settings status", py.eval("pc.get_status(0.)"), Ok(pc.get_status(None)));
    pc.log_config(&w);
    py.exec("pc.log_config(w)").unwrap();
    compare("rollover", py.eval("p.rollover"), Ok(PyValue::List(fake.rollover.borrow().iter().cloned().map(PyValue::Str).collect())));
    for (section, option, value) in [("extruder", "step_pin", "PA2"), ("extruder", "rotation_distance", "7")] {
        pc.set(section, option, value);
        py.exec(&format!("pc.set({section:?}, {option:?}, {value:?})")).unwrap();
    }
    let snapshot = pc.get_status(None);
    pc.remove_section("extruder");
    py.exec("snapshot = pc.get_status(0.)\npc.remove_section('extruder')").unwrap();
    compare("immutable status snapshot", py.eval("snapshot"), Ok(snapshot));
    compare("removed pending section", py.eval("pc.get_status(0.)"), Ok(pc.get_status(None)));
    pc.set("extruder", "step_pin", "PA3");
    py.exec("pc.set('extruder', 'step_pin', 'PA3')").unwrap();
    compare("recreated pending section", py.eval("pc.get_status(0.)"), Ok(pc.get_status(None)));
    let secondary = temp.0.join("secondary.cfg").to_string_lossy().into_owned();
    std::fs::write(&secondary, "[printer]\nratio: 7.0\n").unwrap();
    compare("secondary config", py.eval(&format!("pc.read_config({secondary:?}).getfloat('ratio')")),
        scalar(pc.read_config(&secondary).unwrap().getfloat("ratio", Default::default()), PyValue::Float));
    let gcode = klippy_api::lookup_interface::<dyn GCodeApi>(&*printer, "gcode", LookupDefault::Raise).unwrap().unwrap();
    compare("SAVE_CONFIG help", py.eval("p.objects['gcode'].help"), Ok(PyValue::Dict(gcode.get_command_help().into_iter().map(|(k,v)| (PyValue::Str(k), PyValue::Str(v))).collect())));
    let mut handler = gcode.register_command("SAVE_CONFIG", None, Default::default()).unwrap().unwrap();
    let dispatch = klippy_core::gcode::GCodeDispatch::new(printer);
    let command = dispatch.create_gcode_command("SAVE_CONFIG", "SAVE_CONFIG", &IndexMap::new());
    handler(&command).unwrap();
    let stored_gcode = fake.objects.borrow().get("gcode").unwrap().clone();
    let stored_gcode = klippy_api::downcast_arc::<FakeGCode>(&stored_gcode).unwrap();
    compare("restart delivery", py.eval("(p.objects['gcode'].request_restart('restart'), p.objects['gcode'].restarts)[1]"),
        Ok(PyValue::List(stored_gcode.restarts.borrow().iter().cloned().map(PyValue::Str).collect())));
}

#[test]
fn oracle_reader_includes_and_paths() {
    let Some(mut py) = oracle() else { return };
    let temp = TempConfig::new("includes");
    let filename = temp.filename();
    std::fs::write(temp.0.join("a.cfg"), "[printer]\nx: first\n").unwrap();
    std::fs::write(temp.0.join("b.cfg"), "[printer]\nx: second\n").unwrap();
    std::fs::write(temp.0.join(".hidden.cfg"), "[printer]\nx: hidden\n").unwrap();
    std::fs::write(temp.0.join("].cfg"), "[printer]\nx: bracket\n").unwrap();
    for text in [
        "[include [ab].cfg]\n[printer]\ny: last\n",
        "[include *.cfg]\n",
        "[include []].cfg]\n",
        "[include absent*.cfg]\n[printer]\nx: 1\n",
        "[include missing.cfg]\n",
        "[include a.cfg]\n[include a.cfg]\n",
    ] {
        py.exec(&format!("reader = configfile.ConfigFileReader()\ndata = {text:?}\nfilename = {filename:?}")).unwrap();
        let mut reader = ConfigFileReader::new();
        let actual = reader.build_fileconfig_with_includes(text, &filename)
            .map(|fc| PyValue::Str(ConfigFileReader::build_config_string(&fc))).map_err(config_error);
        compare("include parsing", py.eval("reader.build_config_string(reader.build_fileconfig_with_includes(data, filename))"), actual);
    }
    // `glob` returns non-magic paths as spelled, so the error names `./a.cfg`.
    std::fs::write(temp.0.join("a.cfg"), "[include ./b.cfg]\n").unwrap();
    std::fs::write(temp.0.join("b.cfg"), "[include a.cfg]\n").unwrap();
    let text = "[include a.cfg]\n";
    let mut reader = ConfigFileReader::new();
    compare("recursive include", py.eval(&format!("reader.build_fileconfig_with_includes({text:?}, {filename:?})")),
        reader.build_fileconfig_with_includes(text, &filename).map(|_| PyValue::None).map_err(config_error));
    for path in ["", "a", "a/", "a//b", "/", "//", "///a", "//a/b", "/a/../b"] {
        compare("dirname", py.eval(&format!("os.path.dirname({path:?})")),
            Ok(PyValue::Str(klippy_core::configfile::py_dirname(path))));
        compare("abspath", py.eval(&format!("os.path.abspath({path:?})")),
            Ok(PyValue::Str(klippy_core::configfile::py_abspath(path))));
    }
    let missing = temp.0.join("missing.cfg").to_string_lossy().into_owned();
    compare("unreadable config", py.eval(&format!("reader.read_config_file({missing:?})")),
        reader.read_config_file(&missing).map(PyValue::Str).map_err(config_error));
}
